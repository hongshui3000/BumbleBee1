/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/** 
 * \file crypto.h
 *  Contains the Interface definition of Cryptographic functions required by
 *  the Host Controller Firmware.
 * 
 * \author Santhosh kumar M
 * \date 2007-05-09
 */

#ifndef _CRYPTO_H_
#define _CRYPTO_H_

/* ========================= Include File Section ========================= */
#include "bt_fw_common.h"
#include "btypes.h"
#include "mpal.h"


/* ====================== Macro Declaration Section ======================= */
#define ld32_msb(v, m)  (v) = (((m)[0]<<24) | ((m)[1]<<16) | \
                                ((m)[2]<<8) | (m)[3])
#define st32_msb(m, v)  (m)[0]=(u8)((v)>>24); (m)[1]=(u8)((v)>>16); \
                                (m)[2]=(u8)((v)>>8); (m)[3]=(u8)(v);

#define rol32(r, n)     (((r) << (n)) | ((r) >> (32-(n))))
#define ror32(r, n)     (((r) >> (n)) | ((r) << (32-(n))))

#if 0 /* move to compiler.h */
#ifndef ALIGN
#define ALIGN(n)    __attribute__((aligned(n)))
#endif
#endif

/* ==================== Data Types Declaration Section ==================== */

/** 
 * SSP Private Key type. The private key is represented in binary format with
 * MSB byte ordering.
 */
typedef u8 ssp_prkey_t[24];
/** 
 * SSP Publick Key type. Each coordinate of the public key is represented in
 * binary format with MSB byte ordering.
 */
typedef struct
{
    u8 x[24];           /**< X coordinate (Format(MSB): 23..0) */
    u8 y[24];           /**< Y coordinate (Format(MSB): 47..24) */
} ssp_pukey_t;
/** 
 * SSP Diffie-Hellman Key (generated shared key) type. The dhkey is
 * represented in binary format with MSB byte ordering.
 */
typedef u8 ssp_dhkey_t[24];


#ifdef _CCH_SC_ECDH_P256_
/** 
 * (P256) SSP Private Key type. The private key is represented in binary format with
 * MSB byte ordering.
 */
typedef u8 ssp_prkey_t_p256[MPAL_PRIME_LEN_P256_X2*2];
/** 
 * (P256) SSP Publick Key type. Each coordinate of the public key is represented in
 * binary format with MSB byte ordering.
 */
typedef struct
{
    u8 x[MPAL_PRIME_LEN_P256_X2*2];           /**< X coordinate (Format(MSB): 31..0) */
    u8 y[MPAL_PRIME_LEN_P256_X2*2];           /**< Y coordinate (Format(MSB): 63..32) */

} ssp_pukey_t_p256;
/** 
 * (P256) SSP Diffie-Hellman Key (generated shared key) type. The dhkey is
 * represented in binary format with MSB byte ordering.
 */
typedef u8 ssp_dhkey_t_p256[MPAL_PRIME_LEN_P256_X2*2];



typedef u8 ssp_prkey_t_max[MPAL_PRIME_LEN_MAX_X2*2];

typedef struct
{
    u8 x[MPAL_PRIME_LEN_MAX_X2*2];           /**< X coordinate (Format(MSB): 31..0) */
    u8 y[MPAL_PRIME_LEN_MAX_X2*2];           /**< Y coordinate (Format(MSB): 63..32) */

} ssp_pukey_t_max;

typedef u8 ssp_dhkey_t_max[MPAL_PRIME_LEN_MAX_X2*2];


#endif

#ifdef _CCH_RTL8723A_B_CUT
// _CCH_ISSC_VENDOR_

typedef struct
{
    UINT8   sec_state;
    UINT8   sec_dsa_state;	
    UINT8   padding[3];
    UINT8   salt[2];     
    UINT8   challenge_length;	

    union {  // 64 Byte
        struct {
            UINT8   sec_out2[32]; 
            UINT8   sec_out3[32]; 	
        };
        UINT8   challenge[64]; 
    };


    struct {  // 44 Byte
        union {
            UINT8   sec_out1[40]; 
            struct {
               UINT32 dsa_r[5]; 
               UINT32 dsa_s[5];
           };
        };
        UINT8 mic[4];  			
    };	

    UINT8   rand[4]; 	
    UINT8   mir[48]; 		
    UINT8   mic_error;	


	
} ISSC_SEC_STRUCT;
#endif


/* ================ Exported Variables Declaration Section ================ */


/* ============================= API Section ============================== */
void ssp_rng_init(IN u32 seed);
void ssp_rng_get(OUT u8 rand[16]);

#ifndef _CCH_SC_ECDH_P256_
void ssp_f1(const u8 U[24], const u8 V[24], const u8 X[16], const u8 Z,
        OUT u8 commitment[16]);
void ssp_g(const u8 U[24], const u8 V[24], const u8 X[16], const u8 Y[16],
        OUT u32* numeric_check_value);
void ssp_f2(const u8 W[24], const u8 N1[16], const u8 N2[16], const u8 keyID[4],
        const u8 A1[6], const u8 A2[6], OUT u8 link_key[16]);
void ssp_f3(const u8 W[24], const u8 N1[16], const u8 N2[16], const u8 R[16],
        const u8 IOcap[3], const u8 A1[6], const u8 A2[6],
        OUT u8 check_value[16]);

#else
void ssp_f1(const u8 U[MPAL_PRIME_LEN_MAX<<2], const u8 V[MPAL_PRIME_LEN_MAX<<2], const u8 X[16], const u8 Z,
            OUT u8 commitment[16], UINT16 len_P);

void ssp_g(const u8 U[MPAL_PRIME_LEN_MAX<<2], const u8 V[MPAL_PRIME_LEN_MAX<<2], const u8 X[16], const u8 Y[16],
        OUT u32* numeric_check_value, UINT16 len_P);

void ssp_f2(const u8 W[MPAL_PRIME_LEN_MAX<<2], const u8 N1[16], const u8 N2[16], const u8 keyID[4],
        const u8 A1[6], const u8 A2[6], OUT u8 link_key[16], UINT16 len_P);

void ssp_f3(const u8 W[MPAL_PRIME_LEN_MAX<<2], const u8 N1[16], const u8 N2[16], const u8 R[16],
        const u8 IOcap[3], const u8 A1[6], const u8 A2[6],
        OUT u8 check_value[16], UINT16 len_P);

#endif


void ssp_get_ecdh_keypair(OUT ssp_prkey_t priv, OUT ssp_pukey_t* pub);

#ifndef _CCH_SC_ECDH_P256_	
void ssp_p192(const ssp_prkey_t priv, const ssp_pukey_t* pub,
        OUT ssp_dhkey_t dhkey);
#else
void ssp_p192(const ssp_prkey_t priv, const ssp_pukey_t_max* pub,
              OUT ssp_dhkey_t_max dhkey);
void ssp_get_ecdh_keypair_p256(OUT ssp_prkey_t_p256 priv, OUT ssp_pukey_t_p256* pub);
void ssp_p256(const ssp_prkey_t_p256 priv, const ssp_pukey_t_max* pub,
              OUT ssp_dhkey_t_max dhkey);
#endif
#ifdef _CCH_RTL8723A_B_CUT

#define MPAL_RAND_LOOP 100
#define SEC_DSA_STATE_NUM 10


#endif


#endif /* _CRYPTO_H_ */

