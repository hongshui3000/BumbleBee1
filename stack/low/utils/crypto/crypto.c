/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file crypto.c
 *  Contains the implementation of Cryptographic functions required by the
 *  Host Controller Firmware.
 *
 *  Many of the functions in this module are not \b reentrant. It means that
 *  the call to any function has to be protected with synchronization
 *  primitives (eg. mutexes, DISABLE_INTERRUPTS), if it has to be called
 *  from an interrupt/another thread of execution.
 *
 * \author Santhosh kumar M
 * \date 2007-05-09
 */


/********************************* Logger *************************/
enum { __FILE_NUM__= 214 };
/* ========================= Include File Section ========================= */
#include "crypto.h"
#include "mpal.h"
#include "ecdh.h"
#include "mem.h"
#include "bz_debug.h"
#include "hci_vendor_defines.h"
#include "bt_fw_hci_spec_defines.h"
#include "lmp.h"

#include "bz_auth_internal.h"

#ifdef _CCH_RTL8723A_B_CUT
// _CCH_ISSC_VENDOR_
extern OS_HANDLE hci_ch_as_task_handle;
#endif

#ifdef _SSP_DHKEY_CALCULATE_NEW_FLOW_ 
extern UINT8 g_ssp_ce_index;
#endif     

/* ====================== Macro Declaration Section ======================= */
#ifndef BZ_ASSERT
#include <assert.h>
#define BZ_ASSERT(c, n)     assert(c)
#endif

#define STATIC static
#define CRYPTO_ASSERT(c, n)     /* null macro */

#define MAX_HASH_INPUT_LEN  192 /* bytes (should be multiple of 64) */


/* ==================== Structure Declaration Section ===================== */
typedef struct _ssp_rng_t
{
    ALIGN(4) u8 xkey[18];
    ALIGN(4) u8 xseed[16];
} ssp_rng_t;

/* ===================== Variable Declaration Section ===================== */
static const u32 ssp_k[] =
{
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1,
    0x923f82a4, 0xab1c5ed5, 0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
    0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786,
    0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147,
    0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
    0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85, 0xa2bfe8a1, 0xa81a664b,
    0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a,
    0x5b9cca4f, 0x682e6ff3, 0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
    0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2
};

static ssp_rng_t ssp_rng;


/* ================== Static Function Prototypes Section ================== */
void minimal_sha256(const u8* msg, const u8 len, OUT u8 digest[32]);
void hmac_sha256(const u8* key, const u8 klen, const u8* msg,
                        const u8 mlen, OUT u8 digest[32]);

/* ===================== Function Definition Section ====================== */
/**
 * Compute sha256 sum of the \a msg of length \a len.
 *
 * \param msg Input message for which sha256 sum has to be computed.
 * \param len Length of the input message \a msg (in bytes).
 * \param digest The SHA256 sum output.
 *
 * \return None.
 *
 * \warning This function is named as minimal_sha256 because it will not work
 *          for input message length greater than #MAX_HASH_INPUT_LEN-8. Since
 *          this function is used only for Secure Simple Pairing(SSP) and SSP
 *          doesn't require length more than 184, #MAX_HASH_INPUT_LEN is
 *          choosen to be 192.
 */
void minimal_sha256(const u8* msg, const u8 len, OUT u8 digest[32])
{
    //ALIGN(4)
    u8 pmsg[MAX_HASH_INPUT_LEN] = {0};  /* pre-processed message */
    u8 *p_pmsg;
    u32 H[8] = { 0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
                 0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19
               };
    u32 plen, i;
    UINT32 iub;

    /* Simple pairing doesn't require len > 184 support */
    CRYPTO_ASSERT(len < (MAX_HASH_INPUT_LEN-8), "Message length should not "
                  "be more than sizeof(pmsg)-8");

    /* Preprocessing */
    memcpy(&pmsg[0], msg, len);
    plen = len + (64 - ((len+8) & 0x3F)) + 8;
    pmsg[len] = (u8)(1<<7);
    st32_msb(&pmsg[plen-4], (len << 3));

    /* for each 512-bit(64-byte) chunk */
    iub = (plen >> 6);
    for (i = 0, p_pmsg = pmsg; i < iub; i++, p_pmsg += 64)
    {
        u32 j, a, b, c, d, e, f, g, h, s0, s1, t1, t2, maj, ch;
        u32 w[64];

        /* Break chunk into sixteen 32-bit big-endian words w[0..15] */
        for (j = 0; j < 16; j++)
        {
            ld32_msb(w[j], &p_pmsg[j << 2]);
        }

        /* Extend the sixteen 32-bit words into sixty-four 32-bit words */
        for (j = 16; j < 64; j++)
        {
            s0 = ror32(w[j-15], 7) ^ ror32(w[j-15], 18) ^ (w[j-15]>>3);
            s1 = ror32(w[j-2], 17) ^ ror32(w[j-2], 19) ^ (w[j-2]>>10);
            w[j] = w[j-16] + s0 + w[j-7] + s1;
        }

        /* Initialize hash value for this chunk */
        a = H[0], b = H[1], c = H[2], d = H[3];
        e = H[4], f = H[5], g = H[6], h = H[7];

        /* Main loop */
        for (j = 0; j < 64; j++)
        {
            s0 = ror32(a, 2) ^ ror32(a, 13) ^ ror32(a, 22);
            maj = (a & b) ^ (a & c) ^ (b & c);
            t2 = s0 + maj;
            s1 = ror32(e, 6) ^ ror32(e, 11) ^ ror32(e, 25);
            ch = (e & f) ^ ((~e) & g);
            t1 = h + s1 + ch + ssp_k[j] + w[j];

            h = g;
            g = f;
            f = e;
            e = d + t1;
            d = c;
            c = b;
            b = a;
            a = t1 + t2;
        } /* end main loop */

        /* Add this chunk's hash to result so far */
        H[0] += a, H[1] += b, H[2] += c, H[3] += d;
        H[4] += e, H[5] += f, H[6] += g, H[7] += h;
    } /* end for(each 512-bit chunk) */

    /* Produce the final result */
    for (i = 0; i < 8; i++)
    {
        st32_msb(&digest[i << 2], H[i]);
    }
}

/**
 * Initializes the random number generator with the given \a seed. This
 * function can be called any number of times to re-initialize with different
 * \a seed.
 *
 * \param seed Seed value used for initialization. Getting this value from a
 *             truly random source would enhance randomness and yield maximum
 *             entropy.
 *
 * \return None.
 *
 * \warning This function is not \b reentrant.
 */
void ssp_rng_init(IN u32 seed)
{
    memcpy(&ssp_rng.xseed[0], &seed, 4);
    memcpy(&ssp_rng.xseed[4], &seed, 4);
    memcpy(&ssp_rng.xseed[8], ssp_rng.xseed, 8);
}

/**
 * Generates a new random number.
 *
 * \param rand Random number output.
 *
 * \return None.
 *
 * \warning This function is not \b reentrant.
 */
void ssp_rng_get(OUT u8 rand[16])
{
    u8 g[32];

    mpal_add_u8(&ssp_rng.xkey[0], 18, ssp_rng.xseed, 16);
    minimal_sha256(ssp_rng.xkey, 16, &g[0]);
    memcpy(rand, g, 16);
    memcpy(ssp_rng.xkey, rand, 16);
    ssp_rng.xkey[16] = ssp_rng.xkey[17] = 0;
}

/**
 * Computes keyed-hash message authentication code(HMAC) of the \a msg of
 * length \a mlen using the \a key of length \a klen. The hash function used
 * is SHA256.
 *
 * \param key Input key for computing hmac.
 * \param klen Length of the \a key (in bytes).
 * \param msg Message for which HMAC has to be computed.
 * \param mlen Length of the \a msg (in bytes).
 * \param digest The computed HMAC output.
 *
 * \return None.
 *
 * \warning It supports key length upto 64 bytes only.
 */
void hmac_sha256(const u8* key, const u8 klen, const u8* msg,
                        const u8 mlen, OUT u8 digest[32])
{
    u8 cmsg[MAX_HASH_INPUT_LEN];   /* concatenated message */
    u32 i;

    CRYPTO_ASSERT(klen <= 64, "Key length should not be greater than 64");
    CRYPTO_ASSERT(mlen < (MAX_HASH_INPUT_LEN-64), "Message length should "
                  "not be more than sizeof(cmsg)-64");

    /* inner_digest = h((k ^ ipad) || msg) */
    memset(cmsg, 0x36, 64);
    for (i = 0; i < klen; i++)
    {
        cmsg[i] = (u8)(0x36 ^ key[i]);  /* (k ^ ipad) */
    }
    memcpy(&cmsg[64], msg, mlen);       /* (k ^ ipad) || msg */
    minimal_sha256(cmsg, 64+mlen, &digest[0]);  /* h((k ^ ipad) || msg) */

    /* outer_digest = h((k ^ opad) || inner_digest) */
    memset(cmsg, 0x5c, 64);
    for (i = 0; i < klen; i++)
    {
        cmsg[i] = (u8)(0x5c ^ key[i]);  /* (k ^ opad) */
    }
    memcpy(&cmsg[64], digest, 32);      /* (k ^ opad) || inner_digest */
    minimal_sha256(cmsg, 64+32, &digest[0]); /* h((k ^ opad)||inner_digest) */
}


/**
 * Computes \a commitment value.
 *
 * \param U Input array U of 192 bits (24 bytes).
 * \param V Input array V of 192 bits (24 bytes).
 * \param X Input array X of 128 bits (16 bytes).
 * \param Z Input value Z.
 * \param commitment The computed output commitment value.
 *
 * \return None.
 */
#ifndef _CCH_SC_ECDH_P256_
void ssp_f1(const u8 U[24], const u8 V[24], const u8 X[16], const u8 Z,
            OUT u8 commitment[16])
{
    u8 cmsg[49];    /* concatenated message */
    u8 digest[32];

    /* commitment_value = f1(U, V, X, Z)
     *      = hmac_sha256x(U || V || Z) / 2**128
     */
    memcpy(&cmsg[0], U, 24);
    memcpy(&cmsg[24], V, 24);
    cmsg[48] = Z;
    hmac_sha256(X, 16, cmsg, 49, &digest[0]);
    memcpy(&commitment[0], digest, 16); /* only 16 most significant bytes */
}
#else
void ssp_f1(const u8 U[MPAL_PRIME_LEN_MAX<<2], const u8 V[MPAL_PRIME_LEN_MAX<<2], const u8 X[16], const u8 Z,
            OUT u8 commitment[16], UINT16 len_P)
{
    u8 cmsg[(MPAL_PRIME_LEN_MAX<<3)+1];    /* concatenated message */
    u8 digest[MPAL_PRIME_LEN_MAX<<2];

    /* commitment_value = f1(U, V, X, Z)
     *      = hmac_sha256x(U || V || Z) / 2**128
     */

    memcpy(&cmsg[0], U, len_P<<2);
    memcpy(&cmsg[len_P<<2], V, len_P<<2);
    cmsg[len_P<<3] = Z;

    hmac_sha256(X, 16, cmsg, ((len_P<<3)+1), &digest[0]);
    memcpy(&commitment[0], digest, 16); /* only 16 most significant bytes */
}

#endif
/**
 * Computes 6 digit Numeric Check Value.
 * \code
 *  numeric_check_value = g(PKax, PKbx, Na, Nb)
 *
 *  where PK denotes Public Key
 *        N denotes Nonce
 *        a denotes Initiator
 *        b denotes Responder
 *        x denotes x-coodinate of PK
 * \endcode
 *
 * \param U Input array U of 192 bits (24 bytes). U is x-coodinate of
 *          initiator's public key (PKax).
 * \param V Input array V of 192 bits (24 bytes). V is x-coodinate of
 *          responder's public key (PKbx).
 * \param X Input array X of 128 bits (16 bytes). X is initiator's Nonce
 *          value (Na).
 * \param Y Input array Y of 128 bits (16 bytes). Y is responder's Nonce
 *          value (Nb).
 * \param numeric_check_value The computed output numeric check value.
 *
 * \return None.
 */

#ifndef _CCH_SC_ECDH_P256_
void ssp_g(const u8 U[24], const u8 V[24], const u8 X[16], const u8 Y[16],
           OUT u32* numeric_check_value)
{
    u8 cmsg[80];
    u8 digest[32];

    /* numeric_check_value = g(U, V, X, Y)
     *      = sha256(U || V || X || Y) mod 2**32
     */
    memcpy(&cmsg[0], U, 24);
    memcpy(&cmsg[24], V, 24);
    memcpy(&cmsg[48], X, 16);
    memcpy(&cmsg[64], Y, 16);
    minimal_sha256(cmsg, 80, &digest[0]);
    ld32_msb(*numeric_check_value, &digest[28]); /* only 4 LSB are used */
}
#else
void ssp_g(const u8 U[MPAL_PRIME_LEN_MAX<<2], const u8 V[MPAL_PRIME_LEN_MAX<<2], const u8 X[16], const u8 Y[16],
           OUT u32* numeric_check_value, UINT16 len_P)
{
    u8 cmsg[(MPAL_PRIME_LEN_MAX<<3)+32];
    u8 digest[32];

    /* numeric_check_value = g(U, V, X, Y)
     *      = sha256(U || V || X || Y) mod 2**32
     */
    memcpy(&cmsg[0], U, (len_P<<2));
    memcpy(&cmsg[(len_P<<2)], V, (len_P<<2));
    memcpy(&cmsg[(len_P<<3)], X, 16);
    memcpy(&cmsg[(len_P<<3)+16], Y, 16);
    minimal_sha256(cmsg, (len_P<<3)+32, &digest[0]);
    ld32_msb(*numeric_check_value, &digest[28]); /* only 4 LSB are used */	
}

#endif
/**
 * Calculates Link Key.
 * \code
 *  link_key = f2(DHkey, N_master, N_slave, "btlk", BD_ADDR_master,
 *                BD_ADDR_slave)
 * \endcode
 *
 * \param W Input array W of 192 bits (24 bytes). W is Diffie-Hellman Key (the
 *          output of p192 function - DHkey).
 * \param N1 Input array N1 of 128 bits (16 bytes). N1 is Nonce of Master
 *           device.
 * \param N2 Input array N2 of 128 bits (16 bytes). N2 is Nonce of Slave
 *           device.
 * \param keyID Input array keyID of 32 bits (4 bytes). KeyID is a fixed value
 *              -- "btlk".
 * \param A1 Input array A1 of 48 bits (6 bytes). A1 is Master's BD Address.
 * \param A2 Input array A2 of 48 bits (6 bytes). A2 is Slave's BD Address.
 * \param link_key The caculated output link key value.
 *
 * \return None.
 */
 #ifndef _CCH_SC_ECDH_P256_
void ssp_f2(const u8 W[24], const u8 N1[16], const u8 N2[16], const u8 keyID[4],
            const u8 A1[6], const u8 A2[6], OUT u8 link_key[16])
{
    u8 cmsg[48];
    u8 digest[32];

    /* link_key = f2(W, N1, N2, keyID, A1, A2)
     *      = hmac_sha256w(N1 || N2 || keyID || A1 || A2) / 2**128
     */
    memcpy(&cmsg[0], N1, 16);
    memcpy(&cmsg[16], N2, 16);
    memcpy(&cmsg[32], keyID, 4);
    memcpy(&cmsg[36], A1, 6);
    memcpy(&cmsg[42], A2, 6);
    hmac_sha256(W, 24, cmsg, 48, &digest[0]);
    memcpy(&link_key[0], &digest[0], 16); /* only 16 most significant bytes */
}
 #else

void ssp_f2(const u8 W[MPAL_PRIME_LEN_MAX<<2], const u8 N1[16], const u8 N2[16], const u8 keyID[4],
            const u8 A1[6], const u8 A2[6], OUT u8 link_key[16], UINT16 len_P)
{
    u8 cmsg[48];
    u8 digest[32];

    /* link_key = f2(W, N1, N2, keyID, A1, A2)
     *      = hmac_sha256w(N1 || N2 || keyID || A1 || A2) / 2**128
     */
    memcpy(&cmsg[0], N1, 16);
    memcpy(&cmsg[16], N2, 16);
    memcpy(&cmsg[32], keyID, 4);
    memcpy(&cmsg[36], A1, 6);
    memcpy(&cmsg[42], A2, 6);
    hmac_sha256(W, (len_P<<2), cmsg, 48, &digest[0]);
    memcpy(&link_key[0], &digest[0], 16); /* only 16 most significant bytes */
}
 #endif

/**
 * Computes \a check_value.
 *
 * \param W Input array W of 192 bits (24 bytes).
 * \param N1 Input array N1 of 128 bits (16 bytes).
 * \param N2 Input array N2 of 128 bits (16 bytes).
 * \param R Input array R of 128 bits (16 bytes).
 * \param IOcap Input array IO Capabilities of 24 bits (3 bytes).
 * \param A1 Input array A1 of 48 bits (6 bytes).
 * \param A2 Input array A2 of 48 bits (6 bytes).
 * \param check_value The computed output check value.
 *
 * \return None.
 */
#ifndef _CCH_SC_ECDH_P256_ 
void ssp_f3(const u8 W[24], const u8 N1[16], const u8 N2[16], const u8 R[16],
            const u8 IOcap[3], const u8 A1[6], const u8 A2[6],
            OUT u8 check_value[16])
{
    u8 cmsg[63];
    u8 digest[32];

    /* check_value = f3(W, N1, N2, R, IOcap, A1, A2)
     *      = hmac_sha256w(N1 || N2 || R || IOcap || A1|| A2) / 2**128
     */
    memcpy(&cmsg[0], N1, 16);
    memcpy(&cmsg[16], N2, 16);
    memcpy(&cmsg[32], R, 16);
    memcpy(&cmsg[48], IOcap, 3);
    memcpy(&cmsg[51], A1, 6);
    memcpy(&cmsg[57], A2, 6);
    hmac_sha256(W, 24, cmsg, 63, &digest[0]);
    memcpy(&check_value[0], &digest[0], 16); /* 16 most significant bytes */
}
#else
void ssp_f3(const u8 W[MPAL_PRIME_LEN_MAX<<2], const u8 N1[16], const u8 N2[16], const u8 R[16],
            const u8 IOcap[3], const u8 A1[6], const u8 A2[6],
            OUT u8 check_value[16], UINT16 len_P)
{
    u8 cmsg[63];
    u8 digest[32];

    /* check_value = f3(W, N1, N2, R, IOcap, A1, A2)
     *      = hmac_sha256w(N1 || N2 || R || IOcap || A1|| A2)
     */
    memcpy(&cmsg[0], N1, 16);
    memcpy(&cmsg[16], N2, 16);
    memcpy(&cmsg[32], R, 16);
    memcpy(&cmsg[48], IOcap, 3);
    memcpy(&cmsg[51], A1, 6);
    memcpy(&cmsg[57], A2, 6);
    hmac_sha256(W, len_P<<2, cmsg, 63, &digest[0]);
    memcpy(&check_value[0], &digest[0], 16); /* 16 most significant bytes */
}
#endif



#ifdef _SPEED_UP_AUTH_INIT_
const UINT8 sim_xkey[18] = {0xb4, 0xae, 0xd8, 0xf8, 0xa3, 0x71, 0x34, 0x9f, 0x8b,
                            0x76, 0x6f, 0xda, 0xdf, 0x56, 0xc3, 0x55, 0x00, 0x00};
const UINT32 sim_priv_tmp[6] = {0x6cb01400, 0x12f3528e, 0xf8d8aeb4, 
                                0x9f3471a3, 0xda6f768b, 0x55c356df};
const UINT32 sim_xy_table[12] = {0x4b2c5a2e, 0xcbf29dcf, 0xce139c74, 0x544cf457, 
                                 0x53d3a948, 0x6f7319c5, 0x5368ebf7, 0xf150aecc,
                                 0x66e20257, 0xd3bc5ca3, 0xc92fa6b7, 0x9e77dcae}; 
#ifdef _CCH_SC_ECDH_P256_	
       
#if 1
// Data Set 1a
const UINT32 sim_priv_tmp_p256_1a[MPAL_PRIME_LEN_P256] = {0xd4f6493f, 0x385fc5a3, 0xe3b3c974, 0x503f10d2,
                                                       0x7b60ff4a, 0x99b740eb, 0xa6b89958, 0xbd1a3ccd};

const UINT32 sim_xy_table_p256_1a[MPAL_PRIME_LEN_P256_X2] = {0xd203b020, 0x2cbe97f2, 0xa7832c5e, 0xb9a5f9e9,
                                                          0x1191f4ef, 0xdbfdf4ac, 0x480103cc, 0xe69d350e,
                                                          0x499c80dc, 0x6deb2a65, 0xbf9a3263, 0x5c15525a,
                                                          0xc2456376, 0x2430ed8f, 0xd08e1c74, 0x8bd28915};

// Data Set 1b
const UINT32 sim_priv_tmp_p256_1b[MPAL_PRIME_LEN_P256] = {0x3d8b1855, 0x9abbf632, 0xfbfc0a90, 0x2ae7d4ee,
                                                       0xc29acb59, 0xfb7c9df1, 0x49dd4f6b, 0xfdc57ff4};

const UINT32 sim_xy_table_p256_1b[MPAL_PRIME_LEN_P256_X2] = {0xf0f0a11e, 0x961daf1f, 0x84225909, 0x004c9ef1,
                                                          0xfd8ab547, 0x9fa61586, 0xb2779055, 0x90a1aa2f,
                                                          0x3ef3554c, 0x37ad9d42, 0x3a705673, 0x6051b89a,
                                                          0x30112d47, 0x76368ee2, 0xf9af895f, 0x4a21b115};

// Data Set 2a
const UINT32 sim_priv_tmp_p256_2a[MPAL_PRIME_LEN_P256] = {0x6916a506, 0x1aa39a3c, 0x5d548460, 0x41b65d0c,
                                                       0xb97285b4, 0xffdd0372, 0xf773acb7, 0x637645d0};

const UINT32 sim_xy_table_p256_2a[MPAL_PRIME_LEN_P256_X2] = {0x7ba4312c, 0x9e807957, 0xeab54cf4, 0x433e5caf,
                                                          0xadfaf8d5, 0xcb94874a, 0x039b7e98, 0xdd785c74,
                                                          0x18129591, 0xbedf9838, 0x40e252cd, 0x1f87438e,
                                                          0x911021d0, 0xd43ebd17, 0x7743f8ea, 0x4f5d7143};
#endif

#if 1
// Data Set 2b
const UINT32 sim_priv_tmp_p256[MPAL_PRIME_LEN_P256] = {0x67a09a52, 0x64cd720d, 0xd42e5097, 0x032b5073,
                                                       0xb503887e, 0xa52908c6, 0x19a2caa3, 0xba305550};

const UINT32 sim_xy_table_p256[MPAL_PRIME_LEN_P256_X2] = {0x3fe465f4, 0x1b3f3df2, 0xc0dfc79d, 0x8175a84d,
                                                          0x66c9db84, 0xec964720, 0xf56c0dcf, 0xcc0065e1,
                                                          0x48d00102, 0x99d8bbbc, 0x24c4efee, 0xc2334e16,
                                                          0x10b0c201, 0x434d6bca, 0xca55a1a8, 0x79b2ecd8};
#endif

#endif

#endif

void ssp_get_ecdh_keypair(OUT ssp_prkey_t priv, OUT ssp_pukey_t* pub)
{
#ifndef _SPEED_UP_AUTH_INIT_
    DIGIT_S X[MPAL_PRIME_LEN] = { 0x82ff1012, 0xf4ff0afd, 0x43a18800,
                                  0x7cbf20eb, 0xb03090f6, 0x188da80e
                                };
    DIGIT_S Y[MPAL_PRIME_LEN] = { 0x1e794811, 0x73f977a1, 0x6b24cdd5,
                                  0x631011ed, 0xffc8da78, 0x07192b95
                                };
    DIGIT_S S[MPAL_PRIME_LEN];

#ifndef FOR_SIMULATION
    u16 len_X = 6, len_Y =6;
#endif

    /* private key: 0 < u < r/2 (currently only 160 bits - can be 191 bits) */
    ssp_rng_get(&priv[0]);
    ssp_rng_get(&priv[8]);
    priv[0] = 0x0;

    /* public key: { private_key * G(x,y) } */
    memcpy_and_swap((u8*)S, (u8*)priv, 24);
    
#ifdef FOR_SIMULATION
    S[0] = 0x51fee357;
    X[0] = 0xf32cc65e;
    Y[0] = 0x75e9cce8;
    S[1] = 0xd86eda0e;
    X[1] = 0x90ec6b7e;
    Y[1] = 0x2a4f9ef6;
    S[2] = 0x5a677184;
    X[2] = 0x2641a1de;
    Y[2] = 0xf0185e6b;
    S[3] = 0xf75dbe11;
    X[3] = 0x78016522;
    Y[3] = 0x4606e195;
    S[4] = 0xf7719dd5;
    X[4] = 0x2bf1ad0a;
    Y[4] = 0x64a0e182;
    S[5] = 0x004708ff;
    X[5] = 0x67195ab4;
    Y[5] = 0x47eb5ca8;

    memcpy_and_swap((u8*)pub->x, (u8*)X, 24);
    memcpy_and_swap((u8*)pub->y, (u8*)Y, 24); 
#else
    mixed_scalar_multiply_state0(S,6,X,&len_X,Y,&len_Y, pub->x, pub->y, FALSE);
#endif


#ifdef _CCH_SC_ECDH_P256_
    DIGIT_S X_P256[MPAL_PRIME_LEN_P256] = { 0xd898c296, 0xf4a13945, 0x2deb33a0, 0x77037d81, 
                                            0x63a440f2, 0xf8bce6e5, 0xe12c4247, 0x6b17d1f2};
    DIGIT_S Y_P256[MPAL_PRIME_LEN_P256] = { 0x37bf51f5, 0xcbb64068, 0x6b315ece, 0x2bce3357, 
                                            0x7c0f9e16, 0x8ee7eb4a, 0xfe1a7f9b, 0x4fe342e2};
#endif

#else    
    /* generate pre-defined key content fast */
    memcpy(ssp_rng.xkey, sim_xkey, 18);
    memcpy(priv, (UINT8*)sim_priv_tmp, 24);    
    memcpy(pub->x, (UINT8*)sim_xy_table, 48);   
#endif
}


#ifdef _CCH_SC_ECDH_P256_	
void ssp_get_ecdh_keypair_p256(OUT ssp_prkey_t_p256 priv, OUT ssp_pukey_t_p256* pub)
{

#ifdef _CCH_SC_ECDH_P256_LOG_XXX  
    if( (otp_str_data.bt_bd_addr[0]&BIT0) == 0)
    {
        memcpy(priv, (UINT8*)sim_priv_tmp_p256_1a, 32);    
        memcpy(pub->x, (UINT8*)sim_xy_table_p256_1a, 64);
    }
    else
    {
        memcpy(priv, (UINT8*)sim_priv_tmp_p256_1b, 32);    
        memcpy(pub->x, (UINT8*)sim_xy_table_p256_1b, 64);
    }	
#else
    memcpy(priv, (UINT8*)sim_priv_tmp_p256, 32);    
    memcpy(pub->x, (UINT8*)sim_xy_table_p256, 64);
#endif		
}
#endif

#ifdef _CCH_SC_ECDH_P256_	
void ssp_p192(const ssp_prkey_t priv, const ssp_pukey_t_max* pub,
              OUT ssp_dhkey_t_max dhkey)
#else
void ssp_p192(const ssp_prkey_t priv, const ssp_pukey_t* pub,
              OUT ssp_dhkey_t dhkey)
#endif
{
    DIGIT_S X[MPAL_PRIME_LEN], Y[MPAL_PRIME_LEN], S[MPAL_PRIME_LEN];
    u16 len_X = 6, len_Y = 6;

    memcpy_and_swap((u8*)X, (u8*)pub->x, 24);
    memcpy_and_swap((u8*)Y, (u8*)pub->y, 24);    
    memcpy_and_swap((u8*)S, (u8*)priv, 24);

    if (!verify_point_on_curve(X, len_X, Y, len_Y))
    {
        BZ_ASSERT(0, "The given public key is not on the elliptic curve");
        ssp_rng_get(&dhkey[3]);

#ifdef _SSP_DHKEY_CALCULATE_NEW_FLOW_ 
        LMP_CONNECTION_ENTITY* ce_ptr;    
        ce_ptr = &lmp_connection_entity[g_ssp_ce_index];
        ce_ptr->dhkey_calculating = FALSE;
#endif
        return;
    }
    mixed_scalar_multiply_state0(S,6,X,&len_X,Y,&len_Y, dhkey, NULL, TRUE);
}

#ifdef _CCH_SC_ECDH_P256_	
void ssp_p256(const ssp_prkey_t_p256 priv, const ssp_pukey_t_max* pub,
              OUT ssp_dhkey_t_max dhkey)
{
    DIGIT_S X[MPAL_PRIME_LEN_P256], Y[MPAL_PRIME_LEN_P256], S[MPAL_PRIME_LEN_P256];
    u16 len_X = MPAL_PRIME_LEN_P256, len_Y = MPAL_PRIME_LEN_P256;

    memcpy_and_swap((u8*)X, (u8*)pub->x, 32);
    memcpy_and_swap((u8*)Y, (u8*)pub->y, 32);    
    memcpy_and_swap((u8*)S, (u8*)priv, 32);

#ifdef _CCH_SC_ECDH_P256_LOG
        RT_BT_LOG(WHITE, YL_DBG_HEX_14, 14, 0xAA, 0xAA,
        pub->x[0],pub->x[1],pub->x[2],pub->x[3],
        pub->y[0],pub->y[1],pub->y[2],pub->y[3],
        priv[0],priv[1],priv[2],priv[3]
        );
#endif

    if (!verify_point_on_curve(X, len_X, Y, len_Y))
    {
        BZ_ASSERT(0, "The given public key is not on the elliptic curve");
        ssp_rng_get(&dhkey[3]);

#ifdef _SSP_DHKEY_CALCULATE_NEW_FLOW_ 
        LMP_CONNECTION_ENTITY* ce_ptr;    
        ce_ptr = &lmp_connection_entity[g_ssp_ce_index];
        ce_ptr->dhkey_calculating = FALSE;
#endif


#ifdef _CCH_SC_ECDH_P256_LOG
        RT_BT_LOG(RED, CCH_DBG_019, 0,0);
        BZ_AUTH_LINK_PARAMS* auth;
        auth = ce_ptr->auth;

        RT_BT_LOG(RED, YL_DBG_HEX_14, 14, 0xAA, auth->len_prime,
        X[0],X[1],X[2],X[3],
        Y[0],Y[1],Y[2],Y[3],
        S[0],S[1],S[2],S[3]
        );
#endif

        return;
    }

#ifdef _CCH_SC_ECDH_P256_LOG	
    LMP_CONNECTION_ENTITY* ce_ptr;    
    ce_ptr = &lmp_connection_entity[g_ssp_ce_index];
    BZ_AUTH_LINK_PARAMS* auth;
    auth = ce_ptr->auth;

    RT_BT_LOG(WHITE, YL_DBG_HEX_14, 14, 0xAA, auth->len_prime,
    X[0],X[1],X[2],X[3],
    Y[0],Y[1],Y[2],Y[3],
    S[0],S[1],S[2],S[3]
    );
#endif
    mixed_scalar_multiply_state0(S,MPAL_PRIME_LEN_P256,X,&len_X,Y,&len_Y, dhkey, NULL, TRUE);
}
#endif

