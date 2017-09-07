#include <stdio.h>

#include "Debug.h"

/* Debug global which can be set via command line flag --debug */
bool debug = false;

void printHex(const unsigned char * const data, int length )
{
    printf( "Length: %d bytes\n", length );

    for( int x = 0; x < length; x++ )
    {
        printf( "%02X ", data[x] );
    }

    printf( "\n" );
}