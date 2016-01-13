/*
 *  NIST SP800-38C compliant CCM implementation
 *
 *  Copyright (C) 2014, ARM Limited, All Rights Reserved
 *
 *  This file is part of mbed TLS (https://tls.mbed.org)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/*
 * Definition of CCM:
 * http://csrc.nist.gov/publications/nistpubs/800-38C/SP800-38C_updated-July20_2007.pdf
 * RFC 3610 "Counter with CBC-MAC (CCM)"
 *
 * Related:
 * RFC 5116 "An Interface and Algorithms for Authenticated Encryption"
 */


#include <string.h>
#include "crypto.h"
#include "ccm.h"
#include "trace.h"


#undef printf
#define printf 

#define CCM_ENCRYPT 0
#define CCM_DECRYPT 1




/*
 * Macros for common operations.
 * Results in smaller compiled code than static inline functions.
 */

/*
 * Update the CBC-MAC state in y using a block in b
 * (Always using b as the source helps the compiler optimise a bit better.)
 */
#define UPDATE_CBC_MAC                                                      \
    for( i = 0; i < 16; i++ )                                               \
        y[i] ^= b[i];                                                       \
                                                                            \
    if( ( ret = bt_crypto_e( ctx->key, y, y) ) != 0 ) \
        return( ret );

/*
 * Encrypt or decrypt a partial block with CTR
 * Warning: using b for temporary storage! src and dst must not be b!
 * This avoids allocating one more 16 bytes buffer while allowing src == dst.
 */
#define CTR_CRYPT( dst, src, len  )                                            \
    if( ( ret = bt_crypto_e( ctx->key, ctr, b) ) != 0 )  \
        return( ret );                                                         \
                                                                               \
    for( i = 0; i < len; i++ )                                                 \
        dst[i] = src[i] ^ b[i];

/*
 * Authenticated encryption or decryption
 */
static int ccm_auth_crypt( mesh_security_ccm_context *ctx, int mode, size_t length,
                           const unsigned char *iv, size_t iv_len,
                           const unsigned char *add, size_t add_len,
                           const unsigned char *input, unsigned char *output,
                           unsigned char *tag, size_t tag_len )
{
    int ret;
    unsigned char i;
    unsigned char q;
    size_t len_left, olen;
    unsigned char b[16];
    unsigned char y[16];
    unsigned char ctr[16];
    const unsigned char *src;
    unsigned char *dst;

    /*
     * Check length requirements: SP800-38C A.1
     * Additional requirement: a < 2^16 - 2^8 to simplify the code.
     * 'length' checked later (when writing it to the first block)
     */
    if( tag_len < 4 || tag_len > 16 || tag_len % 2 != 0 )
        return( -1 );

    /* Also implies q is within bounds */
    if( iv_len < 7 || iv_len > 13 )
        return( -1 );

    if( add_len > 0xFF00 )
        return( -1 );

    q = 16 - 1 - (unsigned char) iv_len;

    /*
     * First block B_0:
     * 0        .. 0        flags
     * 1        .. iv_len   nonce (aka iv)
     * iv_len+1 .. 15       length
     *
     * With flags as (bits):
     * 7        0
     * 6        add present?
     * 5 .. 3   (t - 2) / 2
     * 2 .. 0   q - 1
     */
    b[0] = 0;
    b[0] |= ( add_len > 0 ) << 6;
    b[0] |= ( ( tag_len - 2 ) / 2 ) << 3;
    b[0] |= q - 1;

    memcpy( b + 1, iv, iv_len );

    for( i = 0, len_left = length; i < q; i++, len_left >>= 8 )
        b[15-i] = (unsigned char)( len_left & 0xFF );

    if( len_left > 0 )
        return( -1 );


    /* Start CBC-MAC with first block */
    memset( y, 0, 16 );
    UPDATE_CBC_MAC;

    /*
     * If there is additional data, update CBC-MAC with
     * add_len, add, 0 (padding to a block boundary)
     */
    if( add_len > 0 )
    {
        size_t use_len;
        len_left = add_len;
        src = add;

        memset( b, 0, 16 );
        b[0] = (unsigned char)( ( add_len >> 8 ) & 0xFF );
        b[1] = (unsigned char)( ( add_len      ) & 0xFF );

        use_len = len_left < 16 - 2 ? len_left : 16 - 2;
        memcpy( b + 2, src, use_len );
        len_left -= use_len;
        src += use_len;

        UPDATE_CBC_MAC;

        while( len_left > 0 )
        {
            use_len = len_left > 16 ? 16 : len_left;

            memset( b, 0, 16 );
            memcpy( b, src, use_len );
            UPDATE_CBC_MAC;

            len_left -= use_len;
            src += use_len;
        }
    }

    /*
     * Prepare counter block for encryption:
     * 0        .. 0        flags
     * 1        .. iv_len   nonce (aka iv)
     * iv_len+1 .. 15       counter (initially 1)
     *
     * With flags as (bits):
     * 7 .. 3   0
     * 2 .. 0   q - 1
     */
    ctr[0] = q - 1;
    memcpy( ctr + 1, iv, iv_len );
    memset( ctr + 1 + iv_len, 0, q );
    ctr[15] = 1;

    /*
     * Authenticate and {en,de}crypt the message.
     *
     * The only difference between encryption and decryption is
     * the respective order of authentication and {en,de}cryption.
     */
    len_left = length;
    src = input;
    dst = output;

    while( len_left > 0 )
    {
        size_t use_len = len_left > 16 ? 16 : len_left;

        if( mode == CCM_ENCRYPT )
        {
            memset( b, 0, 16 );
            memcpy( b, src, use_len );
            UPDATE_CBC_MAC;
        }

        CTR_CRYPT( dst, src, use_len );

        if( mode == CCM_DECRYPT )
        {
            memset( b, 0, 16 );
            memcpy( b, dst, use_len );
            UPDATE_CBC_MAC;
        }

        dst += use_len;
        src += use_len;
        len_left -= use_len;

        /*
         * Increment counter.
         * No need to check for overflow thanks to the length check above.
         */
        for( i = 0; i < q; i++ )
            if( ++ctr[15-i] != 0 )
                break;
    }

    /*
     * Authentication: reset counter and crypt/mask internal tag
     */
    for( i = 0; i < q; i++ )
        ctr[15-i] = 0;

    CTR_CRYPT( y, y, 16 );
    memcpy( tag, y, tag_len );

    return( 0 );
}

/*
 * Authenticated encryption
 */
int mesh_security_ccm_encrypt_and_tag( mesh_security_ccm_context *ctx, size_t length,
                         const unsigned char *iv, size_t iv_len,
                         const unsigned char *add, size_t add_len,
                         const unsigned char *input, unsigned char *output,
                         unsigned char *tag, size_t tag_len )
{
    return( ccm_auth_crypt( ctx, CCM_ENCRYPT, length, iv, iv_len,
                            add, add_len, input, output, tag, tag_len ) );
}

/*
 * Authenticated decryption
 */
int mesh_security_ccm_auth_decrypt( mesh_security_ccm_context *ctx, size_t length,
                      const unsigned char *iv, size_t iv_len,
                      const unsigned char *add, size_t add_len,
                      const unsigned char *input, unsigned char *output,
                      const unsigned char *tag, size_t tag_len )
{
    int ret;
    unsigned char check_tag[16];
    unsigned char i;
    int diff;

    if( ( ret = ccm_auth_crypt( ctx, CCM_DECRYPT, length,
                                iv, iv_len, add, add_len,
                                input, output, check_tag, tag_len ) ) != 0 )
    {
        DBG_BUFFER(MODULE_APP, LEVEL_ERROR, "ccm_auth_crypt return fail ret = %d ", 1, ret);
        return( ret );
    }

    /* Check tag in "constant-time" */
    for( diff = 0, i = 0; i < tag_len; i++ )
        diff |= tag[i] ^ check_tag[i];

    if( diff != 0 )
    {
            DBG_BUFFER(MODULE_APP, LEVEL_ERROR, "check_tag = ", 0);
            MeshShowKey(check_tag, tag_len);
            DBG_BUFFER(MODULE_APP, LEVEL_ERROR, "tag = ", 0);
            MeshShowKey(tag, tag_len);
           return( -1 );
    }

    return( 0 );
}


#if 0
/*
 * Examples 1 to 3 from SP800-38C Appendix C
 */

#define NB_TESTS 3

/*
 * The data is the same for all tests, only the used length changes
 */
static const unsigned char key[] = {
    0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47,
    0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f
};

static const unsigned char iv[] = {
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
    0x18, 0x19, 0x1a, 0x1b
};

static const unsigned char ad[] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
    0x10, 0x11, 0x12, 0x13
};

static const unsigned char msg[] = {
    0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27,
    0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f,
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
};

static const size_t iv_len [NB_TESTS] = { 7, 8,  12 };
static const size_t add_len[NB_TESTS] = { 8, 16, 20 };
static const size_t msg_len[NB_TESTS] = { 4, 16, 24 };
static const size_t tag_len[NB_TESTS] = { 4, 6,  8  };

static const unsigned char res[NB_TESTS][32] = {
    {   0x71, 0x62, 0x01, 0x5b, 0x4d, 0xac, 0x25, 0x5d },
    {   0xd2, 0xa1, 0xf0, 0xe0, 0x51, 0xea, 0x5f, 0x62,
        0x08, 0x1a, 0x77, 0x92, 0x07, 0x3d, 0x59, 0x3d,
        0x1f, 0xc6, 0x4f, 0xbf, 0xac, 0xcd },
    {   0xe3, 0xb2, 0x01, 0xa9, 0xf5, 0xb7, 0x1a, 0x7a,
        0x9b, 0x1c, 0xea, 0xec, 0xcd, 0x97, 0xe7, 0x0b,
        0x61, 0x76, 0xaa, 0xd9, 0xa4, 0x42, 0x8a, 0xa5,
        0x48, 0x43, 0x92, 0xfb, 0xc1, 0xb0, 0x99, 0x51 }
};

int mesh_security_ccm_self_test( int verbose )
{
    mesh_security_ccm_context ctx;
    unsigned char out[32];
    size_t i;
    int ret;

	memcpy(ctx.key, key, 16);

    for( i = 0; i < NB_TESTS; i++ )
    {
        if( verbose != 0 )
            printf( "  CCM-AES #%u: ", (unsigned int) i + 1 );

        ret = mesh_security_ccm_encrypt_and_tag( &ctx, msg_len[i],
                                   iv, iv_len[i], ad, add_len[i],
                                   msg, out,
                                   out + msg_len[i], tag_len[i] );

        if( ret != 0 ||
            memcmp( out, res[i], msg_len[i] + tag_len[i] ) != 0 )
        {
            if( verbose != 0 )
                printf( "failed\n" );

            return( 1 );
        }

        ret = mesh_security_ccm_auth_decrypt( &ctx, msg_len[i],
                                iv, iv_len[i], ad, add_len[i],
                                res[i], out,
                                res[i] + msg_len[i], tag_len[i] );

        if( ret != 0 ||
            memcmp( out, msg, msg_len[i] ) != 0 )
        {
            if( verbose != 0 )
                printf( "failed\n" );

            return( 1 );
        }

        if( verbose != 0 )
           printf( "passed\n" );
    }




    return( 0 );
}



void test_vector1()
{


	mesh_security_ccm_context ctx;
	unsigned char out[32];
	size_t i;
	int ret;




	const unsigned char key[] = {
		0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xc5, 0xc6, 0xc7,
		0xc8, 0xc9, 0xca, 0xcb, 0xcc, 0xcd, 0xce, 0xcf
	};

	 const unsigned char iv[] = {
		0x00, 0x00, 0x00, 0x03, 0x02, 0x01, 0x00, 0xA0,
		0xA1, 0xA2, 0xA3, 0xA4, 0xA5
	};

	 const unsigned char ad[] = {
		0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
		0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
		0x10, 0x11, 0x12, 0x13
	};

	 const unsigned char msg[] = {
		 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
		 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
		 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e
	};

	 const unsigned char res[] = {
		 0x58, 0x8C, 0x97, 0x9A, 0x61, 0xC6, 0x63, 0xD2, 
		 0xF0, 0x66, 0xD0, 0xC2, 0xC0, 0xF9, 0x89, 0x80, 
		 0x6D, 0x5F, 0x6B, 0x61, 0xDA, 0xC3, 0x84, 0x17,
		 0xE8, 0xD1, 0x2C, 0xFD, 0xF9, 0x26, 0xE0
	 };

	memcpy(ctx.key, key, 16);

	ret = mesh_security_ccm_encrypt_and_tag( &ctx, 23,
		iv, 13, ad, 0,
		msg, out,
		out + 23, 8);



	if( ret != 0 ||
		memcmp( out, res, 23 + 8 ) != 0 )
	{
		printf( "failed\n" );
	}

}


/************************************************************************/
/* 

---->[Mesh][NetKey]84,ba,f7,c1,20,e1,d5,f9,b7,42,cd,92,f2,fe,9a,6c 
---->[Mesh][NetworkID]06,c6,ad,5e,1d,ae,7a,fc,4d,de,5d,21,af,2f,fa,30 
---->[Mesh][IVZero]9a,21,ef,c1,92,0e,de,e3,02,c0,43,1c,75,da,b4,c5 
---->[Mesh][NetworkIV]07,a9,0a,3e,41,f4,b7,ac,85,d1,ff,11,3a,10,fa,c9 
---->[Mesh][App_Nonce]80,0f,0c,a1,0f,0b,d1,ff,11,3a,10,fa,c9 
---->[Mesh][CCM_B0]09,80,0f,0c,a1,0f,0b,d1,ff,11,3a,10,fa,c9,00,06 
---->[Mesh][CCM_X1]7d,f5,01,7b,d3,fb,51,28,61,61,ad,9f,7e,a4,58,ff 
---->[Mesh][CCM_B]a0,b1,c2,d3,e4,f5,00,00,00,00,00,00,00,00,00,00 
---->[Mesh][CCM_X]b2,4c,af,7d,fb,e8,a8,e7,6c,f5,b6,d4,75,fa,fb,91 
---->[Mesh][CCM_A0]01,80,0f,0c,a1,0f,0b,d1,ff,11,3a,10,fa,c9,00,00 
---->[Mesh][CCM_S0]80,a0,1b,c1,b9,75,90,eb,54,ed,e0,e2,d2,a5,e6,79 
---->[Mesh][CCM_MIC]32,ec,b4,bc 
---->[Mesh][CCMXOR_A1]01,80,0f,0c,a1,0f,0b,d1,ff,11,3a,10,fa,c9,00,01 
---->[Mesh][CCMXOR_S1]c2,e3,e0,ea,89,48,bd,fe,34,e0,f5,fd,d0,8d,7c,f5 
---->[Mesh][CCMXOR_In]a0,b1,c2,d3,e4,f5 
---->[Mesh][CCMXOR_Out]62,52,22,39,6d,bd 
---->[Mesh][Net_Nonce]cc,0f,0c,a1,0f,0b,d1,ff,11,3a,10,fa,c9 
---->[Mesh][CCM_B0]09,cc,0f,0c,a1,0f,0b,d1,ff,11,3a,10,fa,c9,00,0c 
---->[Mesh][CCM_X1]77,66,41,b2,48,24,da,7c,3b,3a,08,55,a0,d1,ee,7d 
---->[Mesh][CCM_B]1d,c0,62,52,22,39,6d,bd,32,ec,b4,bc,00,00,00,00 
---->[Mesh][CCM_X]49,19,cf,39,de,4b,e9,47,bd,61,f1,67,a6,57,52,66 
---->[Mesh][CCM_A0]01,cc,0f,0c,a1,0f,0b,d1,ff,11,3a,10,fa,c9,00,00 
---->[Mesh][CCM_S0]76,6e,de,10,40,33,f3,04,81,49,ad,2a,45,02,96,27 
---->[Mesh][CCM_MIC]3f,77,11,29 
---->[Mesh][CCMXOR_A1]01,cc,0f,0c,a1,0f,0b,d1,ff,11,3a,10,fa,c9,00,01 
---->[Mesh][CCMXOR_S1]36,37,3c,37,5c,2f,ae,76,65,f4,5b,5b,82,32,f5,e4 
---->[Mesh][CCMXOR_In]1d,c0,62,52,22,39,6d,bd,32,ec,b4,bc 
---->[Mesh][CCMXOR_Out]2b,f7,5e,65,7e,16,c3,cb,57,18,ef,e7 
---->[Mesh][Privacy_Counter]07,a9,0a,3e,41,f4,b7,ac,85,2b,f7,5e,65,7e,16,c3 
---->[Mesh][Privacy_Key]53,db,fc,4b,80,a1,49,b9,c1,aa,74,10,5c,de,d0,e1 
---->[Mesh][PECB]27,93,34,23,a3,38,11,01,bc,07,9e,79,8e,ff,ac,a9 
---->[Mesh][PRI_TTL_SEQ_SRC]cc,0f,0c,a1,0f,0b 
---->[Mesh][oTTLSEQSRC]eb,9c,38,82,ac,33 
---->[Mesh][App_PDU]50,eb,9c,38,82,ac,33,2b,f7,5e,65,7e,16,c3,cb,57, 
->18,ef,e7,3f,77,11,29 
*/
/************************************************************************/

void test_mesh()
{
	mesh_security_ccm_context ctx;
	unsigned char out[32];
	size_t i;
	int ret;

	const unsigned char app_key[] = {
		0x7b, 0x1d, 0x0f, 0x68, 0xef, 0x03, 0xff, 0x78, 
		0x41, 0x09, 0x11, 0xea, 0x1b, 0x6a, 0x4e, 0xfa
	};

	const unsigned char net_key[] = 
	{
		0x84,0xba,0xf7,0xc1,0x20,0xe1,0xd5,0xf9,
		0xb7,0x42,0xcd,0x92,0xf2,0xfe,0x9a,0x6c
	};

	const unsigned char iv[] = {
		0x80,0x0f,0x0c,0xa1,0x0f,0x0b,0xd1,0xff,
		0x11,0x3a,0x10,0xfa,0xc9
	};
/*
	0x9a,0x21,0xef,0xc1,0x92,0x0e,0xde,0xe3,
		0x02,0xc0,0x43,0x1c,0x75,0xda,0xb4,0xc5
*/
	const unsigned char ad[] = {
		0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
		0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
		0x10, 0x11, 0x12, 0x13
	};

	const unsigned char msg[] = {
		0xa0, 0xb1, 0xc2, 0xd3, 0xe4, 0xf5
	};



	const unsigned char res[] = {
		0x58, 0x8C, 0x97, 0x9A, 0x61, 0xC6, 0x63, 0xD2, 
		0xF0, 0x66, 0xD0, 0xC2, 0xC0, 0xF9, 0x89, 0x80, 
		0x6D, 0x5F, 0x6B, 0x61, 0xDA, 0xC3, 0x84, 0x17,
		0xE8, 0xD1, 0x2C, 0xFD, 0xF9, 0x26, 0xE0
	};

	memcpy(ctx.key, app_key, 16);

	ret = mesh_security_ccm_encrypt_and_tag( &ctx, 6,
		iv, 13, ad, 0,
		msg, out,
		out + 6, 4);



	if( ret != 0 ||
		memcmp( out, res, 23 + 8 ) != 0 )
	{
		printf( "failed\n" );
	}

}

#endif



