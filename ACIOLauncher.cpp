#ifndef _WIN32_WINNT            // Specifies that the minimum required platform is Windows Vista.
#define _WIN32_WINNT 0x0600     // Change this to the appropriate value to target other versions of Windows.
#endif

#include <stdio.h>
#include <tchar.h>
#include <windows.h>
#include "ACIO.h"
#include "Menu.h"
#include "Debug.h"

/**
 * Define to 1 to enable card reader mode, currently only works with
 * slotted readers and will output card IDs with all 0's with wavepass
 * hardware. If this is enabled, hitting the blank button will toggle
 * card reader mode which allows for card IDs to be read off cards and
 * displayed.
 */
#define CARD_READER_MODE      0

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
			debug = true;
		}
		else if (wcscmp(argv[1], L"--debug") == 0)
		{
			inifile = argv[2];
			debug = true;
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
	DEBUG_PRINTF( "Enabling debug mode!\n" );
	
	/* Initialize menu */
	Menu *menu = new Menu(inifile);

	if( menu->NumberOfEntries() < 1 )
	{
		fprintf( stderr, "No games configured to launch!\n" );
		return 1;
	}

	/* Walk serial chain, finding readers */
	ACIO *readers = new ACIO();

	/* Whether we're in read mode */
	unsigned int read = 0;

	/* Actual game to load */
	char *path = NULL;
	bool selected = false;

	/* It may have taken a long time to init */
	menu->ResetTimeout();

	/* Display user prompts */
	menu->DisplayPrompt();
	menu->DisplayGames();

	/* Loop until time's up, then boot */
    while( !selected )
    {
		menu->Tick();
		if (menu->ShouldBootDefault())
		{
			printf( "No selection made, booting %s.\n", menu->GetSelectedName() );
			path = menu->GetSelectedPath();
			selected = true;
			break;
		}

        for( unsigned int x = 0; x < readers->getCount(); x++ )
        {
			unsigned int game = 0;
			bool request_reader_toggle = false;

			switch(readers->getPressedKey(x)) {
				case KEY_1:
					game = 1;
					break;
				case KEY_2:
					game = 2;
					break;
				case KEY_3:
					game = 3;
					break;
				case KEY_4:
					game = 4;
					break;
				case KEY_5:
					game = 5;
					break;
				case KEY_6:
					game = 6;
					break;
				case KEY_7:
					game = 7;
					break;
				case KEY_8:
					game = 8;
					break;
				case KEY_9:
					game = 9;
					break;
				case KEY_0:
					menu->PageUp();
					break;
				case KEY_00:
					menu->PageDown();
					break;
				case KEY_BLANK:
					request_reader_toggle = true;
					break;
			}

			if (game != 0)
			{
				/* Chose a game! */
				if (menu->SelectGame(game))
				{
					/* Time to boot this game! */
					printf( "Booting %s.\n", menu->GetSelectedName() );
					path = menu->GetSelectedPath();
					selected = true;
					break;
				}
			}

            if( CARD_READER_MODE && request_reader_toggle )
			{
                /* Pressed empty button, go into/out of card read mode */
				if (!read)
				{
					printf( "Entering card read mode, insert card!\n" );
					for( unsigned int y = 0; y < readers->getCount(); y++ )
					{
						readers->requestCardInsert(y);
					}
					read = 1;
				}
				else
				{
					printf( "Entering menu mode!\n" );
					for( unsigned int y = 0; y < readers->getCount(); y++ )
					{
						readers->requestCardEject(y);
					}
					read = 0;
				}
                break;
            }

			/* Only read cards if we are in card read mode */
			if (!read) { continue; }

			/* Reset time, so we stay here forever */
			menu->ResetTimeout();

            if( readers->getCardInserted(x) )
            {
                /* Ask for card */
				unsigned char cardId[CARD_LENGTH];
				readers->getCardID(x, cardId);

                /* Display card */
                printf( "Card read: %02X%02X%02X%02X%02X%02X%02X%02X\n",
                        cardId[0], cardId[1], cardId[2], cardId[3],
                        cardId[4], cardId[5], cardId[6], cardId[7] );

				readers->requestCardEject(x);
				readers->requestCardInsert(x);
            }
        }
    }

	/* Kill menu and ACIO connection */
	delete readers;
	delete menu;

	if (path != NULL)
	{
		/* Launch actual game */
		system(path);
	}

    return 0;
}
