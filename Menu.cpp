#include <stdio.h>
#include <windows.h>

#include "Menu.h"
#include "Debug.h"

Menu::Menu(_TCHAR *inifile)
{
	/* Read settings */
	settings = LoadSettings( inifile, &num_programs );

	/* Set up pagination */
	start = 0;
	end = num_programs < GAMES_PER_PAGE ? num_programs : GAMES_PER_PAGE;
	selected = 0;

	/* For exiting on defaults */
	ftime(&beginning);

	/* No cursor please! */
	HANDLE out = GetStdHandle(STD_OUTPUT_HANDLE);
    CONSOLE_CURSOR_INFO cursorInfo;

    GetConsoleCursorInfo(out, &cursorInfo);
    cursorInfo.bVisible = false;
    SetConsoleCursorInfo(out, &cursorInfo);
}

Menu::~Menu(void)
{
	HANDLE out = GetStdHandle(STD_OUTPUT_HANDLE);
    CONSOLE_CURSOR_INFO cursorInfo;

    GetConsoleCursorInfo(out, &cursorInfo);
    cursorInfo.bVisible = true;
    SetConsoleCursorInfo(out, &cursorInfo);
}

void Menu::DisplayPrompt()
{
	/* Prompt the user */
	if (!debug) { system("cls"); }
    printf( "Make a selection on the reader to boot a game.\n" );
	printf( "%s will boot in %d seconds.\n\n", settings[0].name, TIMEOUT_SECONDS );
}

void Menu::DisplayGames()
{
	for (unsigned int i = start; i < end; i++)
	{
		printf( "[%d]  %s\n", (i - start) + 1, settings[i].name );
	}

	printf( "\n" );

	if (start > 0)
	{
		printf( "[0]  Previous Page\n" );
	}

	if (end < num_programs)
	{
		printf( "[00] Next Page\n" );
	}
}

void Menu::PageDown()
{
	if (end < num_programs)
	{
		/* Move up one page */
		start = start + GAMES_PER_PAGE;
		end = start + GAMES_PER_PAGE;
		end = end >= num_programs ? num_programs : end;

		/* print games */
		DisplayPrompt();
		DisplayGames();
	}
}

void Menu::PageUp()
{
	if (start > 0)
	{
		/* Move up one page */
		start = start - GAMES_PER_PAGE;
		start = start < 0 ? 0 : start;
		end = start + GAMES_PER_PAGE;
		end = end >= num_programs ? num_programs : end;

		/* print games */
		DisplayPrompt();
		DisplayGames();
	}
}

void Menu::ResetTimeout()
{
	ftime(&beginning);
}

void Menu::Tick()
{
	ftime(&current);
}

bool Menu::ShouldBootDefault()
{
	ftime(&current);
	return (current.time - beginning.time) > TIMEOUT_SECONDS;
}

bool Menu::SelectGame(unsigned int game)
{
	if (game < 1 || game > 9)
	{
		/* Out of bounds */
		return false;
	}

	/* Zero-based instead of one-based, and account for pagination */
	game--;
	game += start;

	if (game >= num_programs)
	{
		return false;
	}
	else
	{
		selected = game;
		return true;
	}
}

/**
 * Loads an INI file with the following format:
 *
 * [Name of game to launch]
 * launch=<location of batch/executable>
 */
launcher_program_t *Menu::LoadSettings( _TCHAR *ini_file, unsigned int *final_length )
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
