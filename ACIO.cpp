#include <stdio.h>

#include "ACIO.h"
#include "Debug.h"

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

/* Number of times to try initializing a reader by sending baud probe */
#define INITIALIZATION_TRIES  8

/* Number of return baud probes to receive before we consider ourselves initialized */
#define MIN_PROBES_RECEIVED   5

#define WAIT()                Sleep( 100 )
#define LONGWAIT()            Sleep( 1000 )

ACIO::ACIO()
{
	/* Initialize readers first */
	printf( "Initializing readers" );
    InitReaders();
	printf( "\n" );

	/* Get version of all readers */
    for( unsigned int x = 0; x < readerCount; x++ )
    {
		/* Print out reader version in debug mode */
		const char * const version = getReaderVersion( serial, x );
		char * typestring;
		reader_type_t type = getReaderType( serial, x );

		switch( type ) {
			case TYPE_UNKNOWN:
				typestring = "unknown";
				break;
			case TYPE_READER:
				typestring = "reader";
				break;
			case TYPE_DISPENSER:
				typestring = "dispenser";
				break;
			case TYPE_LEDBOARD:
				typestring = "LED board";
				break;
			case TYPE_IOBOARD:
				typestring = "IO board";
				break;
		}

		DEBUG_PRINTF( "Reader %d is type %s version %s\n", x + 1, typestring, version );

        /* Walk init routine */
        initReader( serial, x, 0 );
        initReader( serial, x, 1 );
        initReader( serial, x, 2 );

        /* Set ready for keys only */
        setReaderState( serial, x, STATE_EJECT );
    }

	/* For key input debouncing */
	old_keypresses = (unsigned int *)malloc(sizeof(unsigned int) * readerCount);
}

ACIO::~ACIO(void)
{
	/* Kill memory used for keypresses */
	free(old_keypresses);

	/* Close the reader so we can let the game talk to it */
	for( unsigned int y = 0; y < readerCount; y++ )
	{
		setReaderState( serial, y, STATE_EJECT );
	}
	CloseHandle( serial );
}

bool ACIO::getCardInserted(int reader)
{
	unsigned int currentpresses;
	getReaderState( serial, reader, &state, &currentpresses, cardId );
	return (state == STATE_READ || state == STATE_INSERTED);
}

/**
 * TODO: For some reason this returns one card read in the past. So, on first
 * read it returns garbage, on second read it returns the first card, etc. Fix
 * that at some point.
 */
void ACIO::getCardID(int reader, unsigned char *outCardId)
{
	while( !getCardInserted(reader) ) { requestCardId( serial, reader ); }
	while( !getCardInserted(reader) ) { requestCardId( serial, reader ); }
	memcpy(outCardId, cardId, CARD_LENGTH);
}

reader_keypress_t ACIO::getPressedKey(int reader)
{
	unsigned int currentpresses;
	getReaderState( serial, reader, &state, &currentpresses, cardId );
	unsigned int keypresses = currentpresses & (~old_keypresses[reader]);
	old_keypresses[reader] = currentpresses;

	if (keypresses & 0x0002 )
	{
		return KEY_1;
	}
	if (keypresses & 0x0020 )
	{
		return KEY_2;
	}
	if (keypresses & 0x0200 )
	{
		return KEY_3;
	}
	if (keypresses & 0x0004 )
	{
		return KEY_4;
	}
	if (keypresses & 0x0040 )
	{
		return KEY_5;
	}
	if (keypresses & 0x0400 )
	{
		return KEY_6;
	}
	if (keypresses & 0x0008 )
	{
		return KEY_7;
	}
	if (keypresses & 0x0080 )
	{
		return KEY_8;
	}
	if (keypresses & 0x0800 )
	{
		return KEY_9;
	}
	if (keypresses & 0x0001 )
	{
		return KEY_0;
	}
	if (keypresses & 0x0010 )
	{
		return KEY_00;
	}
	if (keypresses & 0x0100 )
	{
		return KEY_BLANK;
	}

	return KEY_NONE;
}

void ACIO::requestCardInsert(int reader)
{
    setReaderState( serial, reader, STATE_EJECT );

    /* Walk init routine again.  For some reason, it reads the same
     * card ID the second time through, so I just reinit. */
    initReader( serial, reader, 0 );
    initReader( serial, reader, 1 );
    initReader( serial, reader, 2 );

	setReaderState( serial, reader, STATE_NOT_READY );
	setReaderState( serial, reader, STATE_GET_READY );
	setReaderState( serial, reader, STATE_READY );
}

void ACIO::requestCardEject(int reader)
{
	setReaderState( serial, reader, STATE_EJECT );
}

HANDLE ACIO::openSerial( const _TCHAR *arg, int baud )
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

const unsigned char * const ACIO::getPacketData(
	unsigned int *readerId,
    unsigned int *command,
    unsigned int *len,
    unsigned int *checksum,
    const unsigned char * const packet,
    unsigned int length
) {
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

unsigned char ACIO::calcPacketChecksum( const unsigned char * const data )
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

int ACIO::probeReader( HANDLE hSerial )
{
    const unsigned char packet_probe[] = {
		0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
		0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
		0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
		0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
	};
    unsigned char data[1024];
    DWORD length;

	WriteFile( hSerial, packet_probe, sizeof( packet_probe ), &length, 0 );
	ReadFile( hSerial, data, sizeof( data ), &length, 0 );

	if (length < MIN_PROBES_RECEIVED)
	{
		return 0;
	}

	for (unsigned int i = 0; i < length; i++)
	{
		if (data[i] != 0xAA)
		{
			return 0;
		}
	}

	/* Clear out any additional init the reader sends */
	while (true)
	{
		ReadFile( hSerial, data, sizeof( data ), &length, 0 );
		if (length == 0)
		{
			return 1;
		}
	}
}

unsigned int ACIO::getReaderCount( HANDLE hSerial )
{
    const unsigned char count_probe[] = { 0xAA, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x02, };
    unsigned char data[1024];
    DWORD length;

    WriteFile( hSerial, count_probe, sizeof( count_probe ), &length, 0 );
    WAIT();
    ReadFile( hSerial, data, sizeof( data ), &length, 0 );

    unsigned int readerId;
    unsigned int command;
    unsigned int len;
    unsigned int checksum;
    const unsigned char * const payload = getPacketData( &readerId, &command, &len, &checksum, data, length );

    if( payload == NULL ) { return 0; }
    if( len != 1 ) { return 0; }

    return payload[0];
}

void ACIO::initReader( HANDLE hSerial, unsigned int id, int whichInit )
{
    unsigned int init_length[3] = { 7, 7, 8 };
    unsigned char init_probe[3][8] = { { 0xAA, (id + 1), 0x00, 0x03, 0x00, 0x00, 0xFF, },
                                       { 0xAA, (id + 1), 0x01, 0x00, 0x00, 0x00, 0xFF, },
                                       { 0xAA, (id + 1), 0x01, 0x30, 0x00, 0x01, 0x00, 0xFF, } };
    unsigned char data[1024];
    DWORD length;

    /* Fix up checksum since ID is variable */
    init_probe[whichInit][init_length[whichInit] - 1] = calcPacketChecksum( init_probe[whichInit] );

    WriteFile( hSerial, init_probe[whichInit], init_length[whichInit], &length, 0 );
    WAIT();
    ReadFile( hSerial, data, sizeof( data ), &length, 0 );
}

const char * const ACIO::getReaderVersion( HANDLE hSerial, unsigned int id )
{
    unsigned char version_probe[] = { 0xAA, (id + 1), 0x00, 0x02, 0x00, 0x00, 0xFF, };
    static char version[33] = { 0x00 };
    unsigned char data[1024];
    DWORD length;

    /* Fix up checksum since ID is variable */
    version_probe[6] = calcPacketChecksum( version_probe );

    WriteFile( hSerial, version_probe, sizeof( version_probe ), &length, 0 );
    WAIT();
    ReadFile( hSerial, data, sizeof( data ), &length, 0 );

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

reader_type_t ACIO::getReaderType( HANDLE hSerial, unsigned int id )
{
    unsigned char version_probe[] = { 0xAA, (id + 1), 0x00, 0x02, 0x00, 0x00, 0xFF, };
    static char code[5] = { 0x00 };
    unsigned char data[1024];
    DWORD length;

    /* Fix up checksum since ID is variable */
    version_probe[6] = calcPacketChecksum( version_probe );

    WriteFile( hSerial, version_probe, sizeof( version_probe ), &length, 0 );
    WAIT();
    ReadFile( hSerial, data, sizeof( data ), &length, 0 );

    unsigned int readerId;
    unsigned int command;
    unsigned int len;
    unsigned int checksum;
    const unsigned char * const payload = getPacketData( &readerId, &command, &len, &checksum, data, length );

    if( payload != NULL && len == 44 )
    {
        memcpy( code, &payload[8], 4 );
        code[4] = 0x00;

		if (strcmp(code, "ICCA") == 0) {
			return TYPE_READER;
		}
		if (strcmp(code, "ICCB") == 0) {
			return TYPE_READER;
		}
		if (strcmp(code, "ICCC") == 0) {
		  return TYPE_READER;
		}
		if (strcmp(code, "HBHI") == 0) {
			return TYPE_DISPENSER;
		}
		if (strcmp(code, "LEDB") == 0) {
			return TYPE_LEDBOARD;
		}
		if (strcmp(code, "HDXS") == 0) {
			return TYPE_LEDBOARD;
		}
		if (strcmp(code, "KFCA") == 0) {
			return TYPE_IOBOARD;
		}
    }

    return TYPE_UNKNOWN;
}

void ACIO::getReaderState( HANDLE hSerial, unsigned int id, card_state_t *state, unsigned int *keypresses, unsigned char *cardId )
{
    unsigned char state_probe[] = { 0xAA, (id + 1), 0x01, 0x34, 0x00, 0x01, 0x10, 0xFF, };
    unsigned char data[1024];
    DWORD length;

    /* Fix up checksum since ID is variable */
    state_probe[7] = calcPacketChecksum( state_probe );

    /* Sane return */
    *keypresses = 0;

    WriteFile( hSerial, state_probe, sizeof( state_probe ), &length, 0 );
    WAIT();
    ReadFile( hSerial, data, sizeof( data ), &length, 0 );

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

void ACIO::setReaderState( HANDLE hSerial, unsigned int id, reader_state_t state )
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

    WriteFile( hSerial, state_request, sizeof( state_request ), &length, 0 );
    WAIT();
    ReadFile( hSerial, data, sizeof( data ), &length, 0 );
}

void ACIO::requestCardId( HANDLE hSerial, unsigned int id )
{
    unsigned char id_request[] = { 0xAA, (id + 1), 0x01, 0x31, 0x00, 0x01, 0x10, 0xFF, };
    unsigned char data[1024];
    DWORD length;

    /* Fix up checksum since ID is variable */
    id_request[7] = calcPacketChecksum( id_request );

    WriteFile( hSerial, id_request, sizeof( id_request ), &length, 0 );
    WAIT();
    ReadFile( hSerial, data, sizeof( data ), &length, 0 );
}

void ACIO::InitReader(const _TCHAR *comport)
{
	HANDLE hSerial = NULL;
	const unsigned int baudrate[2] = { 57600, 9600 };

	/* Loop until some thread claims the handle, and then exit */
	while (serial == NULL)
	{
		for (unsigned int baud = 0; baud < 2; baud++)
		{
			DEBUG_PRINTF("Attempting to probe readers on %ls,%d\n", comport, baudrate[baud]);
			hSerial = openSerial( comport, baudrate[baud] );

			if (hSerial == INVALID_HANDLE_VALUE)
			{
				DEBUG_PRINTF("Couldn't open com port!\n");
				WAIT();
				continue;
			}

			/* Ensure we start fresh */
			PurgeComm( hSerial, PURGE_RXABORT | PURGE_RXCLEAR | PURGE_TXABORT | PURGE_TXCLEAR );

			/* Init */
			for (int i = 0; i < INITIALIZATION_TRIES; i++)
			{
				if( probeReader( hSerial ) )
				{
					/* Get count */
					unsigned int count = getReaderCount( hSerial );
					if (count > 0)
					{
						/* Make sure at least one is a reader */
						for( unsigned int x = 0; x < count; x++ )
						{
							/* Print out reader version in debug mode */
							if (getReaderType( hSerial, x ) == TYPE_READER)
							{
								DEBUG_PRINTF( "Saw %d readers!\n", count );
								serial = hSerial;
								readerCount = count;
								return;
							}
						}
					}
				}

				if (serial != NULL)
				{
					/* Exit early if we lost the race */
					break;
				}
			}

			/* Failed, close */
			CloseHandle( hSerial );
		}
	}
}

void ACIO::InitReaders()
{
	/* Walk serial chain, finding readers */
    const _TCHAR *comport[4] = { L"COM1", L"COM2", L"COM3", L"COM4" };
	thread_context_t threadContexts[4];
	HANDLE threadHandles[4];
	const char *pattern[4] = { "\b\b\b   ", "\b\b\b.  ", "\b\b\b.. ", "\b\b\b..." };
	unsigned int pno = 0;

	/* Init the global serial handle */
	serial = NULL;

	/* Put a space into location where we will be backspacing, for laziness */
	NON_DEBUG_PRINTF( "   " );
	DEBUG_PRINTF( "...\n" );

	/* Kick off threads for finding the right serial port */
	for (unsigned int i = 0; i < 4; i++)
	{
		threadContexts[i].acio = this;
		threadContexts[i].port = comport[i];
		threadHandles[i] = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)initThread, &threadContexts[i], 0, NULL);
	}

	/* Try to open the reader indefinitely */
	while( serial == NULL )
	{
		/* Wait for threads to find the readers */
		NON_DEBUG_PRINTF( "%s", pattern[((pno++) / 10) % 4] );
		WAIT();
	}

	/* Join on threads */
	for (unsigned int i = 0; i < 4; i++)
	{
		WaitForSingleObject(threadHandles[i], INFINITE);
	}
}
