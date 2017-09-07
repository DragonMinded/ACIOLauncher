#pragma once

/* Define to 1 to get verbose debugging */
#define VERBOSE_DEBUG         0

/* Debug global which can be set via command line flag --debug */
extern bool debug;

/* Debug macros that are enabled via the --debug flag */
#define DEBUG_PRINTF(...) do { if(debug) { printf(__VA_ARGS__); } } while(0)
#define DEBUG_PRINT_HEX(data, length) do { if(debug) { printHex(data, length); } } while(0)

/* Macros that are enabled when debug is not enabled */
#define NON_DEBUG_PRINTF(...) do { if(!debug) { printf(__VA_ARGS__); } } while(0)

void printHex(const unsigned char * const data, int length );