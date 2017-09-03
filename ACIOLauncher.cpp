#ifndef _WIN32_WINNT            // Specifies that the minimum required platform is Windows Vista.
#define _WIN32_WINNT 0x0600     // Change this to the appropriate value to target other versions of Windows.
#endif

#include <stdio.h>
#include <tchar.h>
#include <windows.h>
#include <sys/timeb.h> 

#define WAIT()                Sleep( 100 )
#define LONGWAIT()            Sleep( 1000 )

/* Define to 1 to get verbose debugging */
#define VERBOSE_DEBUG         0

/**
 * Define to 1 to enable card reader mode, currently only works with
 * slotted readers and will output card IDs with all 0's with wavepass
 * hardware. If this is enabled, hitting the blank button will toggle
 * card reader mode which allows for card IDs to be read off cards and
 * displayed.
 */
#define CARD_READER_MODE      0

/* Length of an ACIO packet without payload or checksum */
#define HEADER_LENGTH         5

/* Minimum packet length including header and checksum */
#define MINIMUM_PACKET_LENGTH (HEADER_LENGTH + 1)

/* Start of Message */
#define SOM                   0xAA

/* Location of various parts of the protocol */
#define ID_LOCATION           0
#define COMMAND_HIGH_LOCATION 1
#define COMMAND_LOW_LOCATION  2
#define LENGTH_HIGH_LOCATION  3
#define LENGTH_LOW_LOCATION   4

/* Length of the card ID in bytes */
#define CARD_LENGTH           8

/* Seconds to wait for a selection before booting the default option */
#define TIMEOUT_SECONDS       60

typedef enum
{
    STATE_FRONT_SENSOR,
    STATE_INSERTED,
    STATE_READ,
    STATE_REMOVED,
    STATE_UNKNOWN,
} card_state_t;

typedef enum
{
    STATE_NOT_READY,
    STATE_GET_READY,
    STATE_READY,
    STATE_EJECT,
} reader_state_t;

/* Constants for limitations on how long INI file pieces can be */
#define MAX_GAME_NAME_LENGTH 63
#define MAX_GAME_LOCATION_LENGTH 511

typedef struct
{
	char location[MAX_GAME_LOCATION_LENGTH + 1];
	char name[MAX_GAME_NAME_LENGTH + 1];
} launcher_program_t;

/* Debug global which can be set via command line flag --debug */
unsigned int debug = 0;

/* Debug macros that are enabled via the --debug flag */
#define DEBUG_PRINTF(...) do { if(debug) { printf(__VA_ARGS__); } } while(0)
#define DEBUG_PRINT_HEX(data, length) do { if(debug) { printHex(data, length); } } while(0)

void printHex(const unsigned char * const data, int length )
{
    printf( "Length: %d bytes\n", length );

    for( int x = 0; x < length; x++ )
    {
        printf( "%02X ", data[x] );
    }

    printf( "\n" );
}

HANDLE OpenSerial( const _TCHAR *arg, int baud )
{
    HANDLE hSerial = CreateFile(arg, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
	if (hSerial == INVALID_HANDLE_VALUE) { return hSerial; }

	int rate;

	switch( baud )
	{
		case 4800:
			rate = CBR_4800;
			break;
		case 9600:
			rate = CBR_9600;
			break;
		case 19200:
			rate = CBR_19200;
			break;
		case 38400:
			rate = CBR_38400;
			break;
		case 57600:
			rate = CBR_57600;
			break;
		case 115200:
			rate = CBR_115200;
			break;
		default:
			rate = CBR_9600;
			break;
	}

    DCB dcbSerialParams = {0};

    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    dcbSerialParams.BaudRate = rate;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;
	dcbSerialParams.fOutxCtsFlow = 0;
	dcbSerialParams.fOutxDsrFlow = 0;
	dcbSerialParams.fDtrControl = DTR_CONTROL_DISABLE;
	dcbSerialParams.fDsrSensitivity = 0;
	dcbSerialParams.fOutX = 0;
	dcbSerialParams.fInX = 0;
	dcbSerialParams.fRtsControl = RTS_CONTROL_DISABLE;

    SetCommState(hSerial, &dcbSerialParams);

    COMMTIMEOUTS timeouts = { 0 };

    timeouts.ReadIntervalTimeout = 1;
    timeouts.ReadTotalTimeoutConstant = 1;
    timeouts.ReadTotalTimeoutMultiplier = 1;

    SetCommTimeouts(hSerial, &timeouts);
    return hSerial;
}

const unsigned char * const getPacketData( unsigned int *readerId,
                                     unsigned int *command,
                                     unsigned int *len,
                                     unsigned int *checksum,
                                     const unsigned char * const packet,
                                     unsigned int length )
{
	/* Exit early if we didn't get a good packet */
	if (length < MINIMUM_PACKET_LENGTH) { return NULL; }

    for( unsigned int i = 0; i <= length - MINIMUM_PACKET_LENGTH; i++ )
    {
        const unsigned char * const data = &packet[i];

        if( data[0] != SOM )
        {
            /* Get command */
            *readerId   = data[ID_LOCATION];
            *command    = (data[COMMAND_HIGH_LOCATION] << 8) | data[COMMAND_LOW_LOCATION];
            *len        = (data[LENGTH_HIGH_LOCATION] << 8) | data[LENGTH_LOW_LOCATION];
            *checksum   = data[(*len) + HEADER_LENGTH];
            return data + HEADER_LENGTH;
        }
    }

    return NULL;
}

static unsigned char calcPacketChecksum( const unsigned char * const data )
{
    unsigned char *packet = (unsigned char *)data;

    /* Dirty */
    while( 1 )
    {
        /* Get to start of packet */
        if( packet[0] == SOM )
        {
            packet++;
            continue;
        }

        unsigned int length = ((packet[LENGTH_HIGH_LOCATION] << 8) | packet[LENGTH_LOW_LOCATION]);
        unsigned int sum = 0;

        /* Skip SOM, not part of packet CRC */
        for( unsigned int i = 0; i < length + HEADER_LENGTH; i++ )
        {
            sum += packet[i];
        }

        return sum & 0xFF;
    }
}

int probeReader( HANDLE serial )
{
    const unsigned char packet_probe[] = { 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, };
    unsigned char data[1024];
    DWORD length;

	for (int i = 0; i < 16; i++)
	{
		WriteFile( serial, packet_probe, sizeof( packet_probe ), &length, 0 );
		ReadFile( serial, data, sizeof( data ), &length, 0 );

		if( length == sizeof( packet_probe ) )
		{
			return 0;
		}
	}

    return 1;
}

int getReaderCount( HANDLE serial )
{
    const unsigned char count_probe[] = { 0xAA, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x02, };
    unsigned char data[1024];
    DWORD length;

    WriteFile( serial, count_probe, sizeof( count_probe ), &length, 0 );
    WAIT();
    ReadFile( serial, data, sizeof( data ), &length, 0 );

    unsigned int readerId;
    unsigned int command;
    unsigned int len;
    unsigned int checksum;
    const unsigned char * const payload = getPacketData( &readerId, &command, &len, &checksum, data, length );

    if( payload == NULL ) { return 0; }
    if( len != 1 ) { return 0; }

    return payload[0];
}

void initReader( HANDLE serial, unsigned int id, int whichInit )
{
    unsigned int init_length[3] = { 7, 7, 8 }; 
    unsigned char init_probe[3][8] = { { 0xAA, (id + 1), 0x00, 0x03, 0x00, 0x00, 0xFF, },
                                       { 0xAA, (id + 1), 0x01, 0x00, 0x00, 0x00, 0xFF, },
                                       { 0xAA, (id + 1), 0x01, 0x30, 0x00, 0x01, 0x00, 0xFF, } };
    unsigned char data[1024];
    DWORD length;

    /* Fix up checksum since ID is variable */
    init_probe[whichInit][init_length[whichInit] - 1] = calcPacketChecksum( init_probe[whichInit] );

    WriteFile( serial, init_probe[whichInit], init_length[whichInit], &length, 0 );
    WAIT();
    ReadFile( serial, data, sizeof( data ), &length, 0 );
}

const char * const getReaderVersion( HANDLE serial, unsigned int id )
{
    unsigned char version_probe[] = { 0xAA, (id + 1), 0x00, 0x02, 0x00, 0x00, 0xFF, };
    static char version[33] = { 0x00 };
    unsigned char data[1024];
    DWORD length;

    /* Fix up checksum since ID is variable */
    version_probe[6] = calcPacketChecksum( version_probe );

    WriteFile( serial, version_probe, sizeof( version_probe ), &length, 0 );
    WAIT();
    ReadFile( serial, data, sizeof( data ), &length, 0 );

    unsigned int readerId;
    unsigned int command;
    unsigned int len;
    unsigned int checksum;
    const unsigned char * const payload = getPacketData( &readerId, &command, &len, &checksum, data, length );

    if( payload != NULL && len == 44 )
    {
        memcpy( version, &payload[12], 32 );
        version[32] = 0x00;

        /* Spaces, for display */
        for( int i = 0; i < 32; i++ )
        {
            if( version[i] == 0x00 ) { version[i] = 0x20; }
        }
    }

    return (const char * const)version;
}

void getReaderState( HANDLE serial, unsigned int id, card_state_t *state, unsigned int *keypresses, unsigned char *cardId )
{
    unsigned char state_probe[] = { 0xAA, (id + 1), 0x01, 0x34, 0x00, 0x01, 0x10, 0xFF, };
    unsigned char data[1024];
    DWORD length;

    /* Fix up checksum since ID is variable */
    state_probe[7] = calcPacketChecksum( state_probe );

    /* Sane return */
    *keypresses = 0;
    memset( cardId, 0, CARD_LENGTH );

    WriteFile( serial, state_probe, sizeof( state_probe ), &length, 0 );
    WAIT();
    ReadFile( serial, data, sizeof( data ), &length, 0 );

    unsigned int readerId;
    unsigned int command;
    unsigned int len;
    unsigned int checksum;
    const unsigned char * const payload = getPacketData( &readerId, &command, &len, &checksum, data, length );

    if( payload != NULL && len == 16 )
    {
        *keypresses = (payload[14] << 8) | payload[15];

        *state = STATE_UNKNOWN;
        switch( payload[1] )
        {
            case 0x00:
                *state = STATE_REMOVED;
                break;
            case 0x10:
                *state = STATE_FRONT_SENSOR;
                break;
            case 0x30:
                *state = STATE_INSERTED;
                break;
            default:
                if( VERBOSE_DEBUG ) fprintf( stderr, "Unknown card state %02X!\n", payload[1] );
                break;
        }

        if( payload[0] != 0x00 && payload[11] == 0x00 && *state == STATE_INSERTED )
        {
            memcpy( cardId, &payload[2], CARD_LENGTH );
            *state = STATE_READ;
        }
    }
}

void setReaderState( HANDLE serial, unsigned int id, reader_state_t state )
{
    unsigned char state_request[] = { 0xAA, (id + 1), 0x01, 0x35, 0x00, 0x02, 0x10, 0xFF, 0xFF, };
    unsigned char data[1024];
    DWORD length;

    /* Set state */
    switch( state )
    {
        case STATE_NOT_READY:
            state_request[7] = 0x00;
            break;
        case STATE_GET_READY:
            state_request[7] = 0x03;
            break;
        case STATE_READY:
            state_request[7] = 0x11;
            break;
        case STATE_EJECT:
            state_request[7] = 0x12;
            break;
        default:
            if( VERBOSE_DEBUG ) fprintf( stderr, "Unknown reader state %02X!\n", state );
            state_request[7] = 0xFF;
            break;
    }

    /* Fix up checksum since ID is variable */
    state_request[8] = calcPacketChecksum( state_request );

    WriteFile( serial, state_request, sizeof( state_request ), &length, 0 );
    WAIT();
    ReadFile( serial, data, sizeof( data ), &length, 0 );
}

void requestCardId( HANDLE serial, unsigned int id )
{
    unsigned char id_request[] = { 0xAA, (id + 1), 0x01, 0x31, 0x00, 0x01, 0x10, 0xFF, };
    unsigned char data[1024];
    DWORD length;

    /* Fix up checksum since ID is variable */
    id_request[7] = calcPacketChecksum( id_request );

    WriteFile( serial, id_request, sizeof( id_request ), &length, 0 );
    WAIT();
    ReadFile( serial, data, sizeof( data ), &length, 0 );
}

/**
 * Loads an INI file with the following format:
 *
 * [Name of game to launch]
 * launch=<location of batch/executable>
 */
launcher_program_t *LoadSettings( _TCHAR *ini_file, unsigned int *final_length )
{
	launcher_program_t *progs = 0;
	*final_length = 0;
	unsigned int got_name = 0;
	launcher_program_t temp;

	// Open the file
	HANDLE hFile = CreateFile(ini_file, GENERIC_READ, FILE_SHARE_READ, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
	char buffer[16384];
	unsigned int eof = 0;
	unsigned int eol = 0;
	unsigned int buflen = 0;

	if (hFile == 0)
	{
		return progs;
	}

	memset( &temp, 0, sizeof(temp) );
	memset( buffer, 0, sizeof(buffer) );

	while( !eof )
	{
		DWORD length;
		ReadFile( hFile, buffer + buflen, 1, &length, 0 );
		if (length == 0)
		{
			eof = 1;
			eol = 1;
		}
		else if( *(buffer + buflen) == '\r' )
		{
			/* Ignore \r completely */
			*(buffer + buflen) = 0;
		}
		else if( *(buffer + buflen) == '\n' )
		{
			/* End of line */
			*(buffer + buflen) = 0;
			eol = 1;
		}
		else
		{
			/* Valid thing */
			buflen++;
		}

		if ( eol == 1 )
		{
			/* Process line */
			if (buffer[0] == '[' && buffer[buflen - 1] == ']' && buflen > 2)
			{
				buffer[buflen - 1] = 0;
				char *game = buffer + 1;

				/* Copy this into temp structure */
				strcpy_s( temp.name, MAX_GAME_NAME_LENGTH, game );
				got_name = 1;
			}
			else
			{
				if (strncmp(buffer, "launch", 6) == 0) {
					unsigned int loc = 6;
					// Find equals sign after space
					while (loc < buflen && buffer[loc] == ' ' || buffer[loc] == '\t' ) { loc++; }
					if (loc < buflen)
					{
						if (buffer[loc] == '=')
						{
							loc++;
							while (loc < buflen && buffer[loc] == ' ' || buffer[loc] == '\t' ) { loc++; }
							if (loc < buflen)
							{
								char *launch = buffer + loc;

								if( got_name == 1 )
								{
									/* We have a name to associate with this */
									strcpy_s( temp.location, MAX_GAME_LOCATION_LENGTH, launch );
									got_name = 0;

									/* Make a new spot for this, copy in */
									(*final_length)++;
									progs = (launcher_program_t *)realloc( progs, sizeof(launcher_program_t) * (*final_length) );
									memcpy( progs + ((*final_length) - 1), &temp, sizeof(launcher_program_t) );
									memset( &temp, 0, sizeof(temp) );
								}
							}
						}
					}
				}
			}

			/* Reset buffer */
			if (buflen > 0)
			{
				memset( buffer, 0, sizeof(buffer) );
				buflen = 0;
			}

			/* Not end of line anymore */
			eol = 0;
		}
	}

	return progs;
}

int _tmain(int argc, _TCHAR* argv[])
{
    /* Ensure command is good */
    if( argc < 2 )
    {
        fprintf( stderr, "Missing ini file argument!\n" );
        return 1;
    }
	if( argc > 3 )
	{
        fprintf( stderr, "Too many arguments specified!\n" );
        return 1;
    }
	if( argc == 2 && wcscmp(argv[1], L"--debug") == 0)
	{
        fprintf( stderr, "Missing ini file argument!\n" );
        return 1;
    }

	/* Optional arguments */
	_TCHAR *inifile;
	if( argc == 3 )
	{
		if (wcscmp(argv[2], L"--debug") == 0)
		{
			inifile = argv[1];
			debug = 1;
		}
		else if (wcscmp(argv[1], L"--debug") == 0)
		{
			inifile = argv[2];
			debug = 1;
		}
		else
		{
			fprintf( stderr, "Too many arguments specified!\n" );
			return 1;
		}
	}
	else
	{
		inifile = argv[1];
	}

	/* Display if we're debugging */
	if( debug )
	{
		printf( "Enabling debug mode!\n" );
	}

	/* Read settings */
	unsigned int num_programs = 0;
	launcher_program_t *settings = LoadSettings( inifile, &num_programs );

	if( num_programs < 1 )
	{
		fprintf( stderr, "No games configured to launch!\n" );
		return 1;
	}

	/* Walk serial chain, finding readers */
    HANDLE serial = NULL;
	const _TCHAR *comport[4] = { L"COM1", L"COM2", L"COM3", L"COM4" };
	unsigned int which = 3;

	/* Try to open the reader indefinitely */
	printf( "Initializing readers...\n" );
	while( 1 )
	{
		/* Try next reaer */
		which = (which + 1) % 4;
		DEBUG_PRINTF("Attempting to probe readers on %ls,57600\n", comport[which]);
		serial = OpenSerial( comport[which], 57600 );

		if (serial == INVALID_HANDLE_VALUE)
		{
			DEBUG_PRINTF("Couldn't open com port!\n");
			serial = NULL;
			LONGWAIT();
			continue;
		}

		/* Ensure we start fresh */
		PurgeComm( serial, PURGE_RXABORT | PURGE_RXCLEAR | PURGE_TXABORT | PURGE_TXCLEAR );

		/* Init */
		if( !probeReader( serial ) )
		{
			/* Init success! */
			DEBUG_PRINTF("Succeeded in probing readers!\n");
			break;
		}

		/* Perhaps they need to be opened in 9600 baud instead (slotted readers) */
		CloseHandle( serial );
		DEBUG_PRINTF("Attempting to probe readers on %ls,9600\n", comport[which]);
		serial = OpenSerial( comport[which], 9600 );

		if (serial == INVALID_HANDLE_VALUE)
		{
			DEBUG_PRINTF("Couldn't open com port!\n");
			serial = NULL;
			LONGWAIT();
			continue;
		}

		/* Ensure we start fresh */
		PurgeComm( serial, PURGE_RXABORT | PURGE_RXCLEAR | PURGE_TXABORT | PURGE_TXCLEAR );

		/* Init */
		if( !probeReader( serial ) )
		{
			/* Init success! */
			DEBUG_PRINTF("Succeeded in probing readers!\n");
			break;
		}
		
		/* Failed, close */
		DEBUG_PRINTF("Failed to probe readers!\n");
		CloseHandle( serial );
		serial = NULL;
		LONGWAIT();
	}

    /* Get count */
    int count = getReaderCount( serial );
	DEBUG_PRINTF( "Saw %d readers!\n", count );

    /* Get version of all readers */
    for( int x = 0; x < count; x++ )
    {
		/* Print out reader version in debug mode */
		DEBUG_PRINTF( "Reader %d returned version %s\n", x + 1, getReaderVersion( serial, x ) );

        /* Walk init routine */
        initReader( serial, x, 0 );
        initReader( serial, x, 1 );
        initReader( serial, x, 2 );
        
        /* Set ready for keys only */
        setReaderState( serial, x, STATE_EJECT );
    }

    printf( "Make a selection on the reader to boot a game.\n" );
	printf( "%s will boot in %d seconds.\n\n", settings[0].name, TIMEOUT_SECONDS );

	for (unsigned int i = 0; i < num_programs; i++)
	{
		printf( "[%d] %s\n", i + 1, settings[i].name );
	}

	/* Whether we're in read mode */
	unsigned int read = 0;

	/* For exiting on defaults */
	struct timeb start, end;
	ftime(&start);

	/* For key input debouncing */
	unsigned int *old_keypresses = (unsigned int *)malloc(sizeof(unsigned int) * count);

	/* Actual game to load */
	unsigned int game = 0;

	/* Loop until time's up, then boot */
    while( game == 0 )
    {
		ftime(&end);
		if (end.time - start.time > TIMEOUT_SECONDS)
		{
			printf( "No selection made, booting %s.\n", settings[0].name );
			game = 1;
			break;
		}

        for( int x = 0; x < count; x++ )
        {
            unsigned int currentpresses;
            unsigned char cardId[CARD_LENGTH];
            card_state_t state;

			/* Find new presses only */
            getReaderState( serial, x, &state, &currentpresses, cardId );
			unsigned int keypresses = currentpresses & (~old_keypresses[x]);
			old_keypresses[x] = currentpresses;

			if (keypresses & 0x0002 )
			{
				game = 1;
			}
			if (keypresses & 0x0020 )
			{
				game = 2;
			}
			if (keypresses & 0x0200 )
			{
				game = 3;
			}
			if (keypresses & 0x0004 )
			{
				game = 4;
			}
			if (keypresses & 0x0040 )
			{
				game = 5;
			}
			if (keypresses & 0x0400 )
			{
				game = 6;
			}
			if (keypresses & 0x0008 )
			{
				game = 7;
			}
			if (keypresses & 0x0080 )
			{
				game = 8;
			}
			if (keypresses & 0x0800 )
			{
				game = 9;
			}

			if (game != 0)
			{
				/* Chose a game! */
				if (game > num_programs)
				{
					/* Bad game ID, we don't have this many */
					game = 0;
				}
				else
				{
					/* Time to boot this game! */
					printf( "Booting %s.\n", settings[game - 1].name );
					break;
				}
			}

            if( CARD_READER_MODE && (keypresses & 0x0100) )
            {
                /* Pressed empty button, go into/out of card read mode */
				if (!read)
				{
					printf( "Entering card read mode, insert card!\n" );
					for( int y = 0; y < count; y++ )
					{
						setReaderState( serial, y, STATE_EJECT );
						setReaderState( serial, y, STATE_NOT_READY );
						setReaderState( serial, y, STATE_GET_READY );
						setReaderState( serial, y, STATE_READY );
					}
					read = 1;
				}
				else
				{
					printf( "Entering menu mode!\n" );
					for( int y = 0; y < count; y++ )
					{
						setReaderState( serial, y, STATE_EJECT );
						setReaderState( serial, y, STATE_NOT_READY );
						setReaderState( serial, y, STATE_GET_READY );
					}
					read = 0;
				}
                break;
            }

			/* Only read cards if we are in card read mode */
			if (!read) { continue; }

			/* Reset time, so we stay here forever */
			ftime(&start);

            if( state == STATE_INSERTED )
            {
                /* Ask for card */
                requestCardId( serial, x );
            }

            if( state == STATE_READ )
            {
                /* Display card */
                printf( "Card read: %02X%02X%02X%02X%02X%02X%02X%02X\n",
                        cardId[0], cardId[1], cardId[2], cardId[3],
                        cardId[4], cardId[5], cardId[6], cardId[7] );

                LONGWAIT();
                setReaderState( serial, x, STATE_EJECT );
                LONGWAIT();

                /* Walk init routine again.  For some reason, it reads the same
                 * card ID the second time through, so I just reinit. */
                initReader( serial, x, 0 );
                initReader( serial, x, 1 );
                initReader( serial, x, 2 );

                setReaderState( serial, x, STATE_NOT_READY );
                setReaderState( serial, x, STATE_GET_READY );
                setReaderState( serial, x, STATE_READY );
            }
        }
    }

	/* Close the reader so we can let the game talk to it */
	for( int y = 0; y < count; y++ )
	{
		setReaderState( serial, y, STATE_EJECT );
		setReaderState( serial, y, STATE_NOT_READY );
		setReaderState( serial, y, STATE_GET_READY );
	}
	CloseHandle( serial );

	if (game > 0)
	{
		/* Launch actual game */
		system(settings[game - 1].location);
	}

    return 0;
}
