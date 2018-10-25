#ifndef __APP_H
#define __APP_H
#include <stdint.h>

#define MIN( a, b ) ( ( ( a ) < ( b ) ) ? ( a ) : ( b ) )
#define MAX( a, b ) ( ( ( a ) > ( b ) ) ? ( a ) : ( b ) )
#define POW2( n ) ( 1 << n )

int32_t randr( int32_t min, int32_t max ); //Computes a random number between min and max
void memcpy1( uint8_t *dst, uint8_t *src, uint16_t size ); //Copies size elements of src array to dst array
void memset1( uint8_t *dst, uint8_t value, uint16_t size ); //Set size elements of dst array with value
int8_t Nibble2HexChar( uint8_t a ); //Converts a nibble to an hexadecimal character


#endif


