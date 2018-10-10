#ifndef __ALGORITHM_H
#define __ALGORITHM_H
#include <stdint.h>

#define MIN( a, b ) ( ( ( a ) < ( b ) ) ? ( a ) : ( b ) )
#define MAX( a, b ) ( ( ( a ) > ( b ) ) ? ( a ) : ( b ) )
#define POW2( n ) ( 1 << n )

#define LSHIFT(v, r) do {                                       \
  int i;                                                  \
  for (i = 0; i < 15; i++)                                \
			(r)[i] = (v)[i] << 1 | (v)[i + 1] >> 7;         \
			(r)[15] = (v)[15] << 1;                                 \
    } while (0)
    
#define XOR(v, r) do {                                          \
	int i;                                                  \
	for (i = 0; i < 16; i++)     \
	    {	\
                    (r)[i] = (r)[i] ^ (v)[i]; \
	    }                          \
    } while (0) \

		
int32_t randr( int32_t min, int32_t max ); //Computes a random number between min and max
void mem_Copy( uint8_t *dst, uint8_t *src, uint16_t size ); //Copies size elements of src array to dst array
void mem_Init( uint8_t *dst, uint8_t value, uint16_t size ); //Set size elements of dst array with value
int8_t Nibble2HexChar( uint8_t a ); //Converts a nibble to an hexadecimal character


#endif


