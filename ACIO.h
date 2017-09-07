#pragma once

#include <tchar.h>
#include <windows.h>

/* Length of the card ID in bytes */
#define CARD_LENGTH           8

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

typedef enum
{
	KEY_NONE,
	KEY_1,
	KEY_2,
	KEY_3,
	KEY_4,
	KEY_5,
	KEY_6,
	KEY_7,
	KEY_8,
	KEY_9,
	KEY_0,
	KEY_00,
	KEY_BLANK,
} reader_keypress_t;

typedef struct
{
	void *acio;
	const _TCHAR *port;
} thread_context_t;

class ACIO
{
	public:
		ACIO(void);
		~ACIO(void);

		unsigned int getCount() { return readerCount; }
		reader_keypress_t getPressedKey(int reader);
		void requestCardInsert(int reader);
		void requestCardEject(int reader);
		bool getCardInserted(int reader);
		void getCardID(int reader, unsigned char *cardId);

	private:
		unsigned int readerCount;
		HANDLE serial;
		unsigned int *old_keypresses;
        unsigned char cardId[CARD_LENGTH];
		card_state_t state;

		HANDLE openSerial( const _TCHAR *arg, int baud );
		const unsigned char * const getPacketData(
			unsigned int *readerId,
			unsigned int *command,
			unsigned int *len,
			unsigned int *checksum,
			const unsigned char * const packet,
			unsigned int length
		);
		unsigned char calcPacketChecksum( const unsigned char * const data );
		int probeReader( HANDLE serial );
		unsigned int getReaderCount( HANDLE serial );
		void initReader( HANDLE serial, unsigned int id, int whichInit );
		const char * const getReaderVersion( HANDLE serial, unsigned int id );
		void getReaderState( HANDLE serial, unsigned int id, card_state_t *state, unsigned int *keypresses, unsigned char *cardId );
		void setReaderState( HANDLE serial, unsigned int id, reader_state_t state );
		void requestCardId( HANDLE serial, unsigned int id );

		void InitReaders();
		void InitReader(const _TCHAR *comport);
		static DWORD initThread(LPVOID* param)
		{
			thread_context_t *tc = (thread_context_t *)param;
			((ACIO *)tc->acio)->InitReader(tc->port);
			return 0;
		};
};
