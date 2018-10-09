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


		
		
		
/***************************************************************************************************************
AES
**************************************************************************************************************/
#if 1
#  define AES_ENC_PREKEYED  /* AES encryption with a precomputed key schedule  */
#endif
#if 0
#  define AES_DEC_PREKEYED  /* AES decryption with a precomputed key schedule  */
#endif
#if 0
#  define AES_ENC_128_OTFK  /* AES encryption with 'on the fly' 128 bit keying */
#endif
#if 0
#  define AES_DEC_128_OTFK  /* AES decryption with 'on the fly' 128 bit keying */
#endif
#if 0
#  define AES_ENC_256_OTFK  /* AES encryption with 'on the fly' 256 bit keying */
#endif
#if 0
#  define AES_DEC_256_OTFK  /* AES decryption with 'on the fly' 256 bit keying */
#endif

#define N_ROW                   4
#define N_COL                   4
#define N_BLOCK   (N_ROW * N_COL)
#define N_MAX_ROUNDS           14

typedef unsigned char uint_8t;

typedef uint_8t return_type;

/*  Warning: The key length for 256 bit keys overflows a byte
    (see comment below)
*/

typedef uint_8t length_type;

typedef struct
{   uint_8t ksch[(N_MAX_ROUNDS + 1) * N_BLOCK];
    uint_8t rnd;
} aes_context;

/*  The following calls are for a precomputed key schedule

    NOTE: If the length_type used for the key length is an
    unsigned 8-bit character, a key length of 256 bits must
    be entered as a length in bytes (valid inputs are hence
    128, 192, 16, 24 and 32).
*/

#if defined( AES_ENC_PREKEYED ) || defined( AES_DEC_PREKEYED )

return_type aes_set_key( const unsigned char key[],
                         length_type keylen,
                         aes_context ctx[1] );
#endif

#if defined( AES_ENC_PREKEYED )

return_type aes_encrypt( const unsigned char in[N_BLOCK],
                         unsigned char out[N_BLOCK],
                         const aes_context ctx[1] );

return_type aes_cbc_encrypt( const unsigned char *in,
                         unsigned char *out,
                         int n_block,
                         unsigned char iv[N_BLOCK],
                         const aes_context ctx[1] );
#endif

#if defined( AES_DEC_PREKEYED )

return_type aes_decrypt( const unsigned char in[N_BLOCK],
                         unsigned char out[N_BLOCK],
                         const aes_context ctx[1] );

return_type aes_cbc_decrypt( const unsigned char *in,
                         unsigned char *out,
                         int n_block,
                         unsigned char iv[N_BLOCK],
                         const aes_context ctx[1] );
#endif

/*  The following calls are for 'on the fly' keying.  In this case the
    encryption and decryption keys are different.

    The encryption subroutines take a key in an array of bytes in
    key[L] where L is 16, 24 or 32 bytes for key lengths of 128,
    192, and 256 bits respectively.  They then encrypts the input
    data, in[] with this key and put the reult in the output array
    out[].  In addition, the second key array, o_key[L], is used
    to output the key that is needed by the decryption subroutine
    to reverse the encryption operation.  The two key arrays can
    be the same array but in this case the original key will be
    overwritten.

    In the same way, the decryption subroutines output keys that
    can be used to reverse their effect when used for encryption.

    Only 128 and 256 bit keys are supported in these 'on the fly'
    modes.
*/

#if defined( AES_ENC_128_OTFK )
void aes_encrypt_128( const unsigned char in[N_BLOCK],
                      unsigned char out[N_BLOCK],
                      const unsigned char key[N_BLOCK],
                      uint_8t o_key[N_BLOCK] );
#endif

#if defined( AES_DEC_128_OTFK )
void aes_decrypt_128( const unsigned char in[N_BLOCK],
                      unsigned char out[N_BLOCK],
                      const unsigned char key[N_BLOCK],
                      unsigned char o_key[N_BLOCK] );
#endif

#if defined( AES_ENC_256_OTFK )
void aes_encrypt_256( const unsigned char in[N_BLOCK],
                      unsigned char out[N_BLOCK],
                      const unsigned char key[2 * N_BLOCK],
                      unsigned char o_key[2 * N_BLOCK] );
#endif

#if defined( AES_DEC_256_OTFK )
void aes_decrypt_256( const unsigned char in[N_BLOCK],
                      unsigned char out[N_BLOCK],
                      const unsigned char key[2 * N_BLOCK],
                      unsigned char o_key[2 * N_BLOCK] );
#endif
											
											
#define AES_CMAC_KEY_LENGTH     16
#define AES_CMAC_DIGEST_LENGTH  16											
void AES_CMAC2(uint8_t key[AES_CMAC_KEY_LENGTH],uint8_t *data1,uint8_t len1,uint8_t *data2,uint8_t len2, uint32_t *mic);
void AES_CMAC_Join(uint8_t key[AES_CMAC_KEY_LENGTH],uint8_t *data,uint8_t len , uint32_t *mic);
void Decrypt_Join(uint8_t *key, uint8_t *buffer,  uint8_t *decBuffer, uint16_t size);


											
		
#endif


