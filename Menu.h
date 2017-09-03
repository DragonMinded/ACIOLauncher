#pragma once

#include <tchar.h>
#include <sys/timeb.h>

/* Seconds to wait for a selection before booting the default option */
#define TIMEOUT_SECONDS       60

/* Constants for limitations on how long INI file pieces can be */
#define MAX_GAME_NAME_LENGTH 63
#define MAX_GAME_LOCATION_LENGTH 511

typedef struct
{
	char location[MAX_GAME_LOCATION_LENGTH + 1];
	char name[MAX_GAME_NAME_LENGTH + 1];
} launcher_program_t;

class Menu
{
	public:
		Menu(_TCHAR *inifile);
		~Menu(void);

		unsigned int NumberOfEntries() { return num_programs; }
		char *GetSelectedName() { return settings[selected].name; }
		char *GetSelectedPath() { return settings[selected].location; }
		void DisplayPrompt();
		void DisplayGames();
		bool SelectGame(unsigned int game);

		void Tick();
		void ResetTimeout();
		bool ShouldBootDefault();
	private:
		unsigned int num_programs;
		unsigned int start;
		unsigned int end;
		unsigned int selected;
		launcher_program_t *settings;
		struct timeb beginning;
		struct timeb current;

		launcher_program_t *LoadSettings( _TCHAR *ini_file, unsigned int *final_length );
};
