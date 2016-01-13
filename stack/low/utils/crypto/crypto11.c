/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/********************************* Logger *************************/ 
enum { __FILE_NUM__= 999 };

/* ========================= Include File Section ========================= */
#include "btypes.h"
#include "crypto11.h"
#include "mem.h"
#include "compiler.h"
#include "bz_auth_internal_2_1.h"

extern void hmac_sha256(const u8* key, const u8 klen, const u8* msg,
                        const u8 mlen, OUT u8 digest[32]);

/* ====================== Macro Declaration Section ======================= */


/* ==================== Structure Declaration Section ===================== */


/* ===================== Variable Declaration Section ===================== */
/* Polynomial g1 for calculation of encryption key */
static const u8 polynom_g1[15][16] =
{
    {0x1d,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00},

    {0x3f,0x00,0x01,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},

    {0xdb,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00},

    {0xaf,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00},

    {0x39,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00},

    {0x91,0x02,0x00,0x00,0x00,0x00,0x01,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00},

    {0x95,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00},

    {0x1b,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00},

    {0x09,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x01,0x00,0x00,0x00,0x00,0x00,0x00},

    {0x15,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x01,0x00,0x00,0x00,0x00,0x00},

    {0x3b,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x01,0x00,0x00,0x00,0x00},

    {0xdd,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x01,0x00,0x00,0x00},

    {0x9d,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x01,0x00,0x00},

    {0x4f,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x01,0x00},

    {0xe7,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x01},
};

/* Polynomial g2 for calculation of encryption key*/
static const u8 polynom_g2[15][16] =
{
    {0x8f,0xb0,0x6c,0xbf,0x9b,0x8b,0x92,0xcf,0xd4,
        0x18,0xd2,0xab,0xa0,0x75,0xe2,0x00},

    {0xef,0xef,0xf6,0xcf,0x58,0xc2,0x18,0x7f,
        0xb3,0x59,0x76,0x3d,0xf6,0xe3,0x01,0x00},

    {0x8b,0x80,0x19,0x19,0x5a,0x0a,0x03,0xb1,0x3a,
        0x6c,0x6c,0xf6,0xbe,0x01,0x00,0x00},

    {0xd9,0x6a,0x73,0xd3,0x7f,0x46,0x17,0xde,0x69,
        0x99,0xb8,0x6a,0x01,0x00,0x00,0x00},

    {0x47,0x52,0x71,0x55,0xec,0x50,0xda,0x91,0x32,
        0x06,0x63,0x01,0x00,0x00,0x00,0x00},

    {0x11,0x83,0x46,0x54,0xc0,0x6c,0xaa,0x52,0x93,
        0x2c,0x00,0x00,0x00,0x00,0x00,0x00},

    {0x73,0xa0,0xf3,0x79,0xe2,0xfc,0xff,0xf7,0xb3,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00},

    {0x25,0x80,0xec,0xc7,0x5b,0x81,0xab,0xa1,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00},

    {0x4d,0xb0,0xd8,0x11,0x80,0xc9,0x02,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00},

    {0xbb,0xa4,0xf9,0x24,0x8e,0x05,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00},

    {0xd7,0x24,0x60,0xa7,0x0c,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00},

    {0xb9,0x26,0x9c,0x1c,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00},

    {0xe3,0xd9,0x26,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00},

    {0x77,0x43,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00},

    {0x89,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00},
};


/* Global exp45 and log45 tables */
static const u8 SAFER_PLUS_exp45[256] =
{

    0x01, 0x2d, 0xe2, 0x93, 0xbe, 0x45, 0x15, 0xae,

    0x78, 0x03, 0x87, 0xa4, 0xb8, 0x38, 0xcf, 0x3f,

    0x08, 0x67, 0x09, 0x94, 0xeb, 0x26, 0xa8, 0x6b,

    0xbd, 0x18, 0x34, 0x1b, 0xbb, 0xbf, 0x72, 0xf7,

    0x40, 0x35, 0x48, 0x9c, 0x51, 0x2f, 0x3b, 0x55,

    0xe3, 0xc0, 0x9f, 0xd8, 0xd3, 0xf3, 0x8d, 0xb1,

    0xff, 0xa7, 0x3e, 0xdc, 0x86, 0x77, 0xd7, 0xa6,

    0x11, 0xfb, 0xf4, 0xba, 0x92, 0x91, 0x64, 0x83,

    0xf1, 0x33, 0xef, 0xda, 0x2c, 0xb5, 0xb2, 0x2b,

    0x88, 0xd1, 0x99, 0xcb, 0x8c, 0x84, 0x1d, 0x14,

    0x81, 0x97, 0x71, 0xca, 0x5f, 0xa3, 0x8b, 0x57,

    0x3c, 0x82, 0xc4, 0x52, 0x5c, 0x1c, 0xe8, 0xa0,

    0x04, 0xb4, 0x85, 0x4a, 0xf6, 0x13, 0x54, 0xb6,

    0xdf, 0x0c, 0x1a, 0x8e, 0xde, 0xe0, 0x39, 0xfc,

    0x20, 0x9b, 0x24, 0x4e, 0xa9, 0x98, 0x9e, 0xab,

    0xf2, 0x60, 0xd0, 0x6c, 0xea, 0xfa, 0xc7, 0xd9,

    0x00, 0xd4, 0x1f, 0x6e, 0x43, 0xbc, 0xec, 0x53,

    0x89, 0xfe, 0x7a, 0x5d, 0x49, 0xc9, 0x32, 0xc2,

    0xf9, 0x9a, 0xf8, 0x6d, 0x16, 0xdb, 0x59, 0x96,

    0x44, 0xe9, 0xcd, 0xe6, 0x46, 0x42, 0x8f, 0x0a,

    0xc1, 0xcc, 0xb9, 0x65, 0xb0, 0xd2, 0xc6, 0xac,

    0x1e, 0x41, 0x62, 0x29, 0x2e, 0x0e, 0x74, 0x50,

    0x02, 0x5a, 0xc3, 0x25, 0x7b, 0x8a, 0x2a, 0x5b,

    0xf0, 0x06, 0x0d, 0x47, 0x6f, 0x70, 0x9d, 0x7e,

    0x10, 0xce, 0x12, 0x27, 0xd5, 0x4c, 0x4f, 0xd6,

    0x79, 0x30, 0x68, 0x36, 0x75, 0x7d, 0xe4, 0xed,

    0x80, 0x6a, 0x90, 0x37, 0xa2, 0x5e, 0x76, 0xaa,

    0xc5, 0x7f, 0x3d, 0xaf, 0xa5, 0xe5, 0x19, 0x61,

    0xfd, 0x4d, 0x7c, 0xb7, 0x0b, 0xee, 0xad, 0x4b,

    0x22, 0xf5, 0xe7, 0x73, 0x23, 0x21, 0xc8, 0x05,

    0xe1, 0x66, 0xdd, 0xb3, 0x58, 0x69, 0x63, 0x56,

    0x0f, 0xa1, 0x31, 0x95, 0x17, 0x07, 0x3a, 0x28,

};

static const u8 SAFER_PLUS_log45[256] =
{

    0x80, 0x00, 0xb0, 0x09, 0x60, 0xef, 0xb9, 0xfd,

    0x10, 0x12, 0x9f, 0xe4, 0x69, 0xba, 0xad, 0xf8,

    0xc0, 0x38, 0xc2, 0x65, 0x4f, 0x06, 0x94, 0xfc,

    0x19, 0xde, 0x6a, 0x1b, 0x5d, 0x4e, 0xa8, 0x82,

    0x70, 0xed, 0xe8, 0xec, 0x72, 0xb3, 0x15, 0xc3,

    0xff, 0xab, 0xb6, 0x47, 0x44, 0x01, 0xac, 0x25,

    0xc9, 0xfa, 0x8e, 0x41, 0x1a, 0x21, 0xcb, 0xd3,

    0x0d, 0x6e, 0xfe, 0x26, 0x58, 0xda, 0x32, 0x0f,

    0x20, 0xa9, 0x9d, 0x84, 0x98, 0x05, 0x9c, 0xbb,

    0x22, 0x8c, 0x63, 0xe7, 0xc5, 0xe1, 0x73, 0xc6,

    0xaf, 0x24, 0x5b, 0x87, 0x66, 0x27, 0xf7, 0x57,

    0xf4, 0x96, 0xb1, 0xb7, 0x5c, 0x8b, 0xd5, 0x54,

    0x79, 0xdf, 0xaa, 0xf6, 0x3e, 0xa3, 0xf1, 0x11,

    0xca, 0xf5, 0xd1, 0x17, 0x7b, 0x93, 0x83, 0xbc,

    0xbd, 0x52, 0x1e, 0xeb, 0xae, 0xcc, 0xd6, 0x35,

    0x08, 0xc8, 0x8a, 0xb4, 0xe2, 0xcd, 0xbf, 0xd9,

    0xd0, 0x50, 0x59, 0x3f, 0x4d, 0x62, 0x34, 0x0a,

    0x48, 0x88, 0xb5, 0x56, 0x4c, 0x2e, 0x6b, 0x9e,

    0xd2, 0x3d, 0x3c, 0x03, 0x13, 0xfb, 0x97, 0x51,

    0x75, 0x4a, 0x91, 0x71, 0x23, 0xbe, 0x76, 0x2a,

    0x5f, 0xf9, 0xd4, 0x55, 0x0b, 0xdc, 0x37, 0x31,

    0x16, 0x74, 0xd7, 0x77, 0xa7, 0xe6, 0x07, 0xdb,

    0xa4, 0x2f, 0x46, 0xf3, 0x61, 0x45, 0x67, 0xe3,

    0x0c, 0xa2, 0x3b, 0x1c, 0x85, 0x18, 0x04, 0x1d,

    0x29, 0xa0, 0x8f, 0xb2, 0x5a, 0xd8, 0xa6, 0x7e,

    0xee, 0x8d, 0x53, 0x4b, 0xa1, 0x9a, 0xc1, 0x0e,

    0x7a, 0x49, 0xa5, 0x2c, 0x81, 0xc4, 0xc7, 0x36,

    0x2b, 0x7f, 0x43, 0x95, 0x33, 0xf2, 0x6c, 0x68,

    0x6d, 0xf0, 0x02, 0x28, 0xce, 0xdd, 0x9b, 0xea,

    0x5e, 0x99, 0x7c, 0x14, 0x86, 0xcf, 0xe5, 0x42,

    0xb8, 0x40, 0x78, 0x2d, 0x3a, 0xe9, 0x64, 0x1f,

    0x92, 0x90, 0x7d, 0x39, 0x6f, 0xe0, 0x89, 0x30,

};

static const u8 SAFER_PLUS_BTable[16][16] =
{

    {0x46, 0x97, 0xb1, 0xba, 0xa3, 0xb7, 0x10, 0x0a, 0xc5, 0x37, 0xb3, 0xc9,
     0x5a, 0x28, 0xac, 0x64},

    {0xec, 0xab, 0xaa, 0xc6, 0x67, 0x95, 0x58, 0x0d, 0xf8, 0x9a, 0xf6, 0x6e,
     0x66, 0xdc, 0x05, 0x3d},

    {0x8a, 0xc3, 0xd8, 0x89, 0x6a, 0xe9, 0x36, 0x49, 0x43, 0xbf, 0xeb, 0xd4,
     0x96, 0x9b, 0x68, 0xa0},

    {0x5d, 0x57, 0x92, 0x1f, 0xd5, 0x71, 0x5c, 0xbb, 0x22, 0xc1, 0xbe, 0x7b,
     0xbc, 0x99, 0x63, 0x94},

    {0x2a, 0x61, 0xb8, 0x34, 0x32, 0x19, 0xfd, 0xfb, 0x17, 0x40, 0xe6, 0x51,
     0x1d, 0x41, 0x44, 0x8f},

    {0xdd, 0x04, 0x80, 0xde, 0xe7, 0x31, 0xd6, 0x7f, 0x01, 0xa2, 0xf7, 0x39,
     0xda, 0x6f, 0x23, 0xca},

    {0x3a, 0xd0, 0x1c, 0xd1, 0x30, 0x3e, 0x12, 0xa1, 0xcd, 0x0f, 0xe0, 0xa8,
     0xaf, 0x82, 0x59, 0x2c},

    {0x7d, 0xad, 0xb2, 0xef, 0xc2, 0x87, 0xce, 0x75, 0x06, 0x13, 0x02, 0x90,
     0x4f, 0x2e, 0x72, 0x33},

    {0xc0, 0x8d, 0xcf, 0xa9, 0x81, 0xe2, 0xc4, 0x27, 0x2f, 0x6c, 0x7a, 0x9f,
     0x52, 0xe1, 0x15, 0x38},

    {0xfc, 0x20, 0x42, 0xc7, 0x08, 0xe4, 0x09, 0x55, 0x5e, 0x8c, 0x14, 0x76,
     0x60, 0xff, 0xdf, 0xd7},

    {0xfa, 0x0b, 0x21, 0x00, 0x1a, 0xf9, 0xa6, 0xb9, 0xe8, 0x9e, 0x62, 0x4c,
     0xd9, 0x91, 0x50, 0xd2},

    {0x18, 0xb4, 0x07, 0x84, 0xea, 0x5b, 0xa4, 0xc8, 0x0e, 0xcb, 0x48, 0x69,
     0x4b, 0x4e, 0x9c, 0x35},

    {0x45, 0x4d, 0x54, 0xe5, 0x25, 0x3c, 0x0c, 0x4a, 0x8b, 0x3f, 0xcc, 0xa7,
     0xdb, 0x6b, 0xae, 0xf4},

    {0x2d, 0xf3, 0x7c, 0x6d, 0x9d, 0xb5, 0x26, 0x74, 0xf2, 0x93, 0x53, 0xb0,
     0xf0, 0x11, 0xed, 0x83},

    {0xb6, 0x03, 0x16, 0x73, 0x3b, 0x1e, 0x8e, 0x70, 0xbd, 0x86, 0x1b, 0x47,
     0x7e, 0x24, 0x56, 0xf1},

    {0x88, 0x46, 0x97, 0xb1, 0xba, 0xa3, 0xb7, 0x10, 0x0a, 0xc5, 0x37, 0xb3,
     0xc9, 0x5a, 0x28, 0xac},

};

/* ================== Static Function Prototypes Section ================== */
void calculate_PHT(u8 *input);
void exp_and_log(u8 *input);
void xor_and_sum(u8 *input1, u8 *input2, u8 flag);
void calculate_K_dash(u8 *key);
void generate_subkey_vector(const u8 *key,u8 *subkey,u8 n);
void AR_function(const u8 key[16], u8 rand[16], u8 flag);
void shift_base8(u8 *input,u8 *output,u8 len, u8 n);
u8 get_msb_bit(u8 *input, u8 len);
u8 compare(u8 *input1, u8 *input2, u8 len);
void get_modulo_oper( u8 *key, u8 *polynom,u8 len);
void multiply(u8 *key, u8 *polynom,u8 len);
void calculate_kc_dash(INOUT u8 *key, u8 len);


/* ===================== Function Definition Section ====================== */
/**
 * Performs permutation with the given input values.
 *
 * \param input Input values.
 *
 * \return None.
 */
void permute(u8 *input)
{
    u8 temp[16];
    memcpy(&temp[0],&input[0],16);

    input[0]     = temp[8];
    input[1]     = temp[11];
    input[2]     = temp[12];
    input[3]     = temp[15];
    input[4]     = temp[2];
    input[5]     = temp[1];
    input[6]     = temp[6];
    input[7]     = temp[5];
    input[8]     = temp[10];
    input[9]     = temp[9];
    input[10]    = temp[14];
    input[11]    = temp[13];
    input[12]    = temp[0];
    input[13]    = temp[7];
    input[14]    = temp[4];
    input[15]    = temp[3];
}

/**
 * Implements the butter fly function with the given input values.
 *
 * \param input Input values.
 *
 * \return None.
 */
void calculate_PHT(u8 *input)
{
    u8 i;
    u8 temp1,temp2;
    for(i=0;i<16;i=(u8)(i+2))
    {
        temp1 = input[i];
        temp2 = temp1 + input[i+1];
        input[i] = temp1 + temp2;
        input[i+1] = temp2;
    }
}

/**
 * Implementation of exp and log function.
 *
 * \param input Input values.
 *
 * \return None.
 */
void exp_and_log(u8 *input)
{
    u8 i;
    for(i=0;i<16;i++)
    {
        if ((1 << i) & 0x9999) /* speed up and reduce code size by austin */
        {
            input[i] = SAFER_PLUS_exp45[input[i]];
        }
        else
        {
            input[i] = SAFER_PLUS_log45[input[i]];
        }
    }
}

/**
 * Implementation of XOR and SUM function
 *
 * \param input1 Input value 1
 * \param input2 Input value 2
 *
 * \return None.
 * \param flag
 */
void xor_and_sum(u8 *input1, u8 *input2, u8 flag)
{
    u8 i;
    
    /* modify the same logic but speed up and reduce code size by austin */
    if ((flag == 1) || (flag == 2))
    {
        for(i=0;i<16;i++)
        {
            if (((flag == 1) && ((1 << i) & 0x9999)) ||
                ((flag == 2) && !((1 << i) & 0x9999)))                
            {
                /*perform xor operation */
                input1[i] ^= input2[i];
            }
            else
            {
                input1[i] += input2[i];                
            }
        }
    }
}

/**
 * Implementation of K_dash function.
 *
 * \param key Key value.
 *
 * \return None.
 */
void calculate_K_dash(u8 *key)
{
    key[0]  = (u8)((key[0]  + 233) & 0xFF);
    key[1]  = (u8)(key[1]  ^ 229);
    key[2]  = (u8)((key[2]  + 223) & 0xFF);
    key[3]  = (u8)(key[3]  ^ 193);
    key[4]  = (u8)((key[4]  + 179) & 0xFF);
    key[5]  = (u8)(key[5]  ^ 167);
    key[6]  = (u8)((key[6]  + 149) & 0xFF);
    key[7]  = (u8)(key[7]  ^ 131);
    key[8]  = (u8)(key[8]  ^ 233);
    key[9]  = (u8)((key[9]  + 229) & 0xFF);
    key[10] = (u8)(key[10] ^ 223);
    key[11] = (u8)((key[11] + 193) & 0xFF);
    key[12] = (u8)(key[12] ^ 179);
    key[13] = (u8)((key[13] + 167) & 0xFF);
    key[14] = (u8)(key[14] ^ 149);
    key[15] = (u8)((key[15] + 131) & 0xFF);
}

/**
 * Implementation of subkey vector function.
 *
 * \param key Key
 * \param subkey Sub key
 * \param n N
 *
 * \return None.
 */
void generate_subkey_vector(const u8 *key,u8 *subkey,u8 n)
{
    u8 i,j;
    u8 temp[20];
    u8 xor_sum;
    u8 temp1;

    xor_sum = key[0];
    temp[0] = key[0];
    for(i=1;i<16;i++)
    {
        xor_sum = (u8)(xor_sum ^ key[i]);
        temp[i] = key[i];
    }
    temp[16] = xor_sum;
    for(i=1;i<n;i++)
    {
        for(j=0;j<17;j++)
        {
            /* Rotate Left by three bits */
            temp1    = (u8)(temp[j]>>5);
            temp[j]  = (u8)((temp[j] <<3) | temp1);
        }
    }
    /*  Select octets  and add Bias */
    j = (u8)(n-1);
    for(i=0;i<16;i++)
    {
        subkey[i] = temp[j];
        if(n>1)
        {
            subkey[i] = (u8)(subkey[i] + SAFER_PLUS_BTable[n-2][i]);
        }
        j = (u8)((j+1)%17);
    }
}

/**
 * Implementation of SAFER (a.k.a AR_function) function.
 *
 * \param key Key.
 * \param rand Random number.
 * \param flag 0 / 1.
 *
 * \return None.
 */
void AR_function(const u8 key[16], u8 rand[16], u8 flag)
{
    u8 r,i;
    u8 round_one_input[16];
    u8 subkey[16];

    if (flag)
    {
        for(i=0;i<16;i++)
        {
            round_one_input[i] = rand[i];
        } 
    }
    
    for(r=1;r<9;r++)
    {
        if (flag)
        {
            if (r == 3)
            {
                xor_and_sum(rand, round_one_input,1);
            }
        }

        /* Generate subkey K (2*r -1) */
        generate_subkey_vector(key,subkey,(u8)((r << 1) - 1));
        /* Perform xor and sum operation */
        xor_and_sum(rand, subkey,1);
        /* Perform log and exp operation */
        exp_and_log(rand);
        /* Generate subkey K (2*r) */
        generate_subkey_vector(key,subkey,(u8)(r << 1));
        /* Perform xor and sum operation */
        xor_and_sum(rand, subkey,2);
        /* Perform PHT and permute operation three times*/
        for(i=0;i<3;i++)
        {
            /*Perform PHT Operation */
            calculate_PHT(rand);
            /* Perform Permute */
            permute(rand);
        }

        /*Perform PHT Operation once more */
        calculate_PHT(rand);
    }

    /* Generate subkey K (2*r -1) */
    generate_subkey_vector(key,subkey,(u8)((r << 1) - 1));
    /* Perform xor and sum operation */
    xor_and_sum(rand, subkey,1);
}

/**
 * Shifts a array left by len number of bytes.
 *
 * \param input Input array.
 * \param output Output array.
 * \param len Length to be shifted.
 * \param n N.
 *
 * \return None.
 */
void shift_base8(u8 *input,u8 *output,u8 len, u8 n)
{
    u8 count;
    u8 length;
    u8 no_of_indexs;
    u8 no_of_bits;
    u8 temp[20];

    no_of_indexs = (u8)(len >> 3);
    no_of_bits = (u8)(len & 0x07);

    length = (u8)(n - no_of_indexs);

    /* Shift by indexs */
    for( count=0;count <length ;count++ )
    {
      temp[no_of_indexs+count] = input[count];
    }

    /* Shift by bits */
    length = (u8)(8 - no_of_bits);
    output[no_of_indexs] = (u8)(temp[no_of_indexs] << no_of_bits);
    for(count = (u8)(no_of_indexs+1);count < n; count++)
    {
        output[count] = (u8)((temp[count-1]>>length) | (temp[count]<<no_of_bits));
    }
    return;
}

/**
 * Finds MSB bit of an array.
 *
 * \param input Input array.
 * \param len Length of the array.
 *
 * \return MSB of the input array.
 */
u8 get_msb_bit(u8 *input, u8 len)
{
    u8  count,i ;
    u8 result;

    result = (u8)(len << 3); /* (len * 8); */
    for(count = len ; count > 0; count--)
    {
        if (input[count-1] == 0)
        {
            result -= 8;
        }
        else
        {            
            i = 0;
            while (((0x80 >> i) & input[count-1]) == 0)
            {
                i++;
                result--;
            }
            break;
        }
    }
    return result;
}

/**
 * Compares two arrays.
 *
 * \param input1 Input array 1.
 * \param input2 Input array 2.
 * \param len Length to be compared.
 *
 * \return 0, if both inputs are same. 1, if input1 is lesser than input2.
 *         0xff, if input1 is greater than input2.
 */
u8 compare(u8 *input1, u8 *input2, u8 len)
{
    u8 count;
    u8 result=0;

    for( count =len; count > 0 ;count--)
    {
        if(input1[count-1] < input2[count-1])
        {
            result =1;
            return result;
        }

        if(input1[count-1] > input2[count-1])
        {
            result = 0xff;
            return result;
        }
    }
    return result;
}

/**
 * Finds modulo of two arrays.
 *
 * \param key Key.
 * \param polynom Polynomial.
 * \param len Length.
 *
 * \return None.
 */
void get_modulo_oper( u8 *key, u8 *polynom,u8 len)
{
   u8 n1,n2,count;
   u8 result;
   u8 temp[20];
    result = compare(key,polynom,len);

    if(result == 0 ) /*  equal */
    {
        memset(&key[0],0x0,len);
        return;
    }
    else if ( result == 1)
    {
        return;
    }
    else
    {
        memset(&temp[0],0x0,16);
        n2 = get_msb_bit(polynom,len);
        while( result != 1)
        {
            n1 = get_msb_bit( key,len);
            if( n1 > n2 )
            {
                n1 = (u8)(n1 -n2);
                
                shift_base8(polynom,&temp[0],n1,len);
                
                for(count=0;count<16;count++)
                {
                    key[count] = (u8)(key[count] ^ temp[count]);
                }

                /* subtract(key,&temp[0],len); */
                result = compare(key,polynom,len);
            }
            else if(n1 == n2)
            {
                /* subtract(key,polynom,len); */
                for(count=0;count<16;count++)
                {
                    key[count] = (u8)(key[count] ^ polynom[count]);
                }
                return;
            }
            else
            {
                return;
            }
        }
    }
}

/**
 * Multiplies two arrays.
 *
 * \param key Key.
 * \param polynom Polynomial.
 * \param len Length.
 *
 * \return None.
 */
void multiply(u8 *key, u8 *polynom,u8 len)
{
    u8 multiplicant[16];
    u8 multipler[16];
    ALIGN(4) u8 temp[16];
    u8 count,n,result;
    UINT8 *temp_a;
    UINT8 *temp_b;
    u8 index;
    u8 bit_pos;

    result = compare(key,polynom,len);

    if(result <=1)
    {
        temp_a = multiplicant;
        temp_b = multipler;
    }
    else
    {
        temp_a = multipler;
        temp_b = multiplicant;
    }
    
    for(count=0;count<len;count++)
    {
        temp_a[count] = polynom[count];
        temp_b[count] = key[count];
        key[count] =0;
    }

    *(UINT32*)&temp[0] = 0;
    *(UINT32*)&temp[4] = 0;    
    *(UINT32*)&temp[8] = 0;
    *(UINT32*)&temp[12] = 0;
    
    n = get_msb_bit(&multipler[0],len);
    while(n > 0)
    {
        shift_base8(&multiplicant[0],&temp[0],(u8)(n-1),len);

        /* subtract(key,&temp[0],len); */
        for(count=0;count<16;count++)
        {
            key[count] = (u8)(key[count] ^ temp[count]);
        }

        index = (n-1) >> 3;
        bit_pos = (n-1) & 0x07; 
        
        *(UINT32*)&temp[0] = 0;
        *(UINT32*)&temp[4] = 0;    
        *(UINT32*)&temp[8] = 0;
        *(UINT32*)&temp[12] = 0;

        temp[index] = 1 << bit_pos;
        
        /* subtract(&multipler[0],&temp[0],len); */
        multipler[index] ^= temp[index];

        n = get_msb_bit(&multipler[0],len);
    }
    return;
}

/**
 * Calculates the encryption key Kc-dash (truncated or augumented encryption
 * key) from Kc depending on allowed length of encryption key.
 *
 * \param key Encryption Key to be truncated (or augumented) to \a len.
 * \param len Allowed encryption key length.
 *
 * \return None.
 */
void calculate_kc_dash(INOUT u8 *key, u8 len)
{
    u8 polynomg1[16];
    u8 polynomg2[16];

    if (len < 16)
    {
        memcpy(&polynomg1[0], &polynom_g1[len-1][0], 16);
        memcpy(&polynomg2[0], &polynom_g2[len-1][0], 16);
        get_modulo_oper(key, polynomg1, 16);
        multiply(key, polynomg2, 16);
    }
}

/**
 * Computes SRES and ACO.
 *
 * \param key Link key to be authenticated.
 * \param rand Authentication challenge (LMP_AU_RAND content).
 * \param bd_addr Bluetooth Device Address of the device which received the
 *                challenge.
 * \param sres Calculated response output.
 * \param aco Authenticated Ciphering Offset output.
 *
 * \return None.
 */
void lp_E1(const u8 key[16], const u8 rand[16], const u8 bd_addr[6],
        u8 sres[4], u8 aco[12])
{
    register int i;
    u8 lkey[16];     /* local key (modified during calculations) */
    u8 lrand[16];    /* local random number */
    ALIGN(2) u8 addr[16];

    memcpy(&lrand[0], rand, 16);
    memcpy(&lkey[0], key, 16);

    /* addr = U bd_addr[i%6], for i = 0..15 */
    memcpy(&addr[0], bd_addr, 6);
    *(UINT16*)&addr[6] = *(UINT16*)&addr[0]; 
    *(UINT16*)&addr[8] = *(UINT16*)&addr[2];             
    *(UINT16*)&addr[10] = *(UINT16*)&addr[4]; 
    *(UINT16*)&addr[12] = *(UINT16*)&addr[0]; 
    *(UINT16*)&addr[14] = *(UINT16*)&addr[2];

    /* Perform Ar operation first Time */
    AR_function(lkey, lrand, 0);

    /* Modulo 2 Addition of Result from Ar function and Random Number */
    /* Add output from E function and Modulo 2 added Result */
    for(i = 0; i < 16; i++)
    {
        lrand[i] = (u8)((lrand[i] ^ rand[i]) + addr[i]);
    }

    /* Calculate K dash */
    calculate_K_dash(lkey);

    /* Perform Ar dash operation */
    AR_function(lkey, lrand, 1);
    memcpy(sres, lrand, 4);
    memcpy(aco, &lrand[4], 12);
}

/**
 * Computes Unit_Key or Comb_Key.
 *
 * \param rand_num Input random number.
 * \param bd_addr BD_ADDR of the device which generated the \a rand_num.
 * \param key Generated Unit_Key or Comb_Key.
 *
 * \return None.
 */
void lp_E21(const u8 rand[16], const u8 bd_addr[6], u8 key[16])
{
    u8 mrand[16];

    /* modify random number: mrand = rand[0..14] U (rand[15] ^ 6) */
    memcpy(&mrand[0], rand, 16);
    mrand[15] = (u8)(mrand[15] ^ 6);

    /* set initial key: key = (U bd_addr[i%6] for i = 0..15) */
    memcpy(&key[0], bd_addr, 6);
    memcpy(&key[6], bd_addr, 6);
    memcpy(&key[12], bd_addr, 4);

    /* compute final key */
    AR_function(mrand, key, 1);
}

/** 
 * Computes Initialization_Key or Master_link_key.
 * 
 * \param rand Random number (eg. content of LMP_in_rand or LMP_temp_rand).
 * \param pin PIN, Random number, or Link key.
 * \param pin_len Length of \a pin.
 * \param bd_addr BD_ADDR of the device receiving the LMP_in_rand or all zeros
 *                when computing master link key.
 * \param key Computed key.
 * 
 * \return None.
 */
void lp_E22(const u8 rand[16], const u8 pin[16], u8 pin_len,
        const u8 bd_addr[6], u8 key[16])
{
    u8 aug_len;
    u8 aug_pin[16];

    /* Set the initial value of the key to rand */
    memcpy(&key[0], rand, 16);

    /* Augument the PIN */
    memcpy(&aug_pin[0], pin, pin_len);
    aug_len = (u8)(16 - pin_len);
    if (aug_len > 6)
    {
        aug_len = 6;
    }
    memcpy(&aug_pin[pin_len], bd_addr, aug_len);
    pin_len = (u8)(pin_len + aug_len);
    memcpy(&aug_pin[pin_len], &aug_pin[0], 16-pin_len);

    /* Compute key */
    key[15] = (u8)(key[15] ^ pin_len);
    AR_function(aug_pin, key, 1);
}

/**
 * Computes encryption key with the effective length given by \a len.
 *
 * \param key Current Link key.
 * \param randr Encryption Random number (content of LMP_start_encryption).
 * \param aco Authenticated Ciphering Offset (by-product of E1 algorithm
 *            during authentication).
 * \param enc_key The output key (Encryption key).
 * \param len Allowed encryption key length (The \a enc_key will be
 *            manipulated based in this \a len to support varying encryption
 *            key sizes).
 *
 * \return None.
 */
void lp_E3(const u8 key[16], const u8 rand[16], const u8 aco[12],
        u8 enc_key[16], u8 len)
{
    register int i;
    ALIGN(4) u8 temp_aco[16];
    ALIGN(4) u8 tempkey[16];

    memcpy(&tempkey[0], key, 16);   /* copy link key to workspace */
    memcpy(&enc_key[0], rand, 16);  /* set the initial key */

    memcpy(&temp_aco[0], aco, 12);  /* Augument the ACO */
    *(UINT32*)&temp_aco[12] = *(UINT32*)&temp_aco[0]; /* temp_aco[15:12] = [3:0] */

    AR_function(tempkey, enc_key, 0);  /* Perform Ar operation first Time */

    /* Modulo 2 Addition of Result from Ar function and Random Number */
    /* Calculate E function */
    /* Add output from E function and Modulo 2 added Result */
    for(i = 0; i < 16; i++)
    {
        enc_key[i] = (u8)((enc_key[i] ^ rand[i]) + temp_aco[i]);
    }

    calculate_K_dash(tempkey);          /* Calculate K dash */
    AR_function(tempkey, enc_key, 1);   /* Perform Ar dash operation */
    calculate_kc_dash(enc_key, len);
}

#ifdef _CCH_SC_ECDH_P256_ 
/**
 * The AES Encryption Key Generation Function h3.
 * \code
 *  h3(T, keyID, A1, A2, ACO) 
 * \endcode
 *
 * \param T is the 128 bit Bluetooth Link Key derived from f2. (== W)
 * \param keyID Input array keyID of 32 bits (4 bytes). KeyID is a fixed value
 *              -- "btlk".
 * \param A1 Input array A1 of 48 bits (6 bytes). A1 is Master's BD Address.
 * \param A2 Input array A2 of 48 bits (6 bytes). A2 is Slave's BD Address.
 * \param ACO is the 64 bit ACO output from h5
 
 * \param aes_enc_key The caculated output AES encryption keys.
 *
 * \return None.
 */
void lp_h3(const u8 T_orig[16], const u8 keyID[4],
            const u8 A1_orig[6], const u8 A2_orig[6], const u8 ACO[8], OUT u8 aes_enc_key[16])
{
    u8 cmsg[24];
    u8 digest[32];

    /* aes_enc_key = h3(T, keyID, A1, A2, ACO) 
     *      = hmac_sha256t(keyID || A1 || A2 || ACO) / 2^128
     */

    UCHAR T[16];
    UCHAR A1[LMP_BD_ADDR_SIZE];
    UCHAR A2[LMP_BD_ADDR_SIZE];

    memcpy(&T[0], T_orig, 16);
    memcpy(&A1[0], A1_orig, LMP_BD_ADDR_SIZE);
    memcpy(&A2[0], A2_orig, LMP_BD_ADDR_SIZE);

    bz_auth_convert_to_msb(&T[0], 16);
    bz_auth_convert_to_msb(&A1[0], LMP_BD_ADDR_SIZE);
    bz_auth_convert_to_msb(&A2[0], LMP_BD_ADDR_SIZE);	

    memcpy(&cmsg[0], keyID, 4);
    memcpy(&cmsg[4], A1, 6);
    memcpy(&cmsg[10], A2, 6);
    memcpy(&cmsg[16], ACO, 8);	
    hmac_sha256(T, 16, cmsg, 24, &digest[0]);
    memcpy(&aes_enc_key[0], &digest[0], 16); /* only 16 most significant bytes */

#ifdef _CCH_SC_ECDH_P256_LOG
    RT_BT_LOG(RED, YL_DBG_HEX_20, 20, T[0],T[1],T[2],T[3],
    cmsg[0],cmsg[1],cmsg[2],cmsg[3],
    cmsg[4],cmsg[5],cmsg[6],cmsg[7],
    cmsg[10],cmsg[11],cmsg[12],cmsg[13],cmsg[16],cmsg[17],cmsg[18],cmsg[19]);
#endif
	
	
#ifdef _CCH_SC_ECDH_P256_LOG
    RT_BT_LOG(RED, CCH_DBG_157, 4, aes_enc_key[0], aes_enc_key[1], aes_enc_key[2], aes_enc_key[3]);
#endif	

}

/**
 * The Device Authentication Key Generation Function h4.
 * \code
 *  h4(T, KeyID, A1, A2)
 * \endcode
 *
 * \param T is the 128 bit Bluetooth Link Key derived from f2. (== W)
 * \param keyID Input array keyID of 32 bits (4 bytes). KeyID is a fixed value
 *              -- "btlk".
 * \param A1 Input array A1 of 48 bits (6 bytes). A1 is Master's BD Address.
 * \param A2 Input array A2 of 48 bits (6 bytes). A2 is Slave's BD Address.
 
 * \param dev_auth_key The caculated output Device Authentication keys.
 *
 * \return None.
 */
void lp_h4(u8 T_orig[16], u8 keyID[4],
            u8 A1_orig[6], u8 A2_orig[6], OUT u8 dev_auth_key[16])
{
    u8 cmsg[16];
    u8 digest[32];

    /* dev_auth_key = h4(T, keyID, A1, A2) 
     *      = hmac_sha256t(keyID || A1 || A2) / 2^128
     */
    UCHAR T[16];
    UCHAR A1[LMP_BD_ADDR_SIZE];
    UCHAR A2[LMP_BD_ADDR_SIZE];

    memcpy(&T[0], T_orig, 16);
    memcpy(&A1[0], A1_orig, LMP_BD_ADDR_SIZE);
    memcpy(&A2[0], A2_orig, LMP_BD_ADDR_SIZE);

    bz_auth_convert_to_msb(&T[0], 16);
    bz_auth_convert_to_msb(&A1[0], LMP_BD_ADDR_SIZE);
    bz_auth_convert_to_msb(&A2[0], LMP_BD_ADDR_SIZE);	

    memcpy(&cmsg[0], keyID, 4);
    memcpy(&cmsg[4], A1, 6);
    memcpy(&cmsg[10], A2, 6);	
    hmac_sha256(T, 16, cmsg, 16, &digest[0]);
    memcpy(&dev_auth_key[0], &digest[0], 16); /* only 16 most significant bytes */

#ifdef _CCH_SC_ECDH_P256_LOG
    RT_BT_LOG(RED, YL_DBG_HEX_16, 16, T[0],T[1],T[2],T[3],
    cmsg[0],cmsg[1],cmsg[2],cmsg[3],
    cmsg[4],cmsg[5],cmsg[6],cmsg[7],
    cmsg[10],cmsg[11],cmsg[12],cmsg[13]);
#endif
	
#ifdef _CCH_SC_ECDH_P256_LOG
    RT_BT_LOG(RED, CCH_DBG_158, 4,dev_auth_key[0], dev_auth_key[1], dev_auth_key[2], dev_auth_key[3]);
#endif

}

/**
 * The Device Authentication Confirmation Function h5.
 * \code
 *  h5(S, R1, R2)
 * \endcode
 *
 * \param S is the 128 bit Bluetooth Device Authentication Key derived from h4.
 * \param R1 is the 128 bit random number (AU_RAND_M) 
 *           from the master during the Link Manager device authentication sequence
 * \param R2 is the 128 bit random number (AU_RAND_S) 
 *           from the slave during the Link Manager device authentication sequence.
 *
 * \param SRESmaster
 * \param SRESslave
 * \param ACO is the Authentication Ciphering Offset 
 *            used in h3 and as the IV for Encryption Start for the encryption nonce.
 *
 * \return None.
 */
void lp_h5(const u8 S[16], const u8 R1_orig[16], const u8 R2_orig[16], 
            OUT u8 SRESmaster[4], OUT u8 SRESslave[4], OUT u8 ACO[8])
{
    u8 cmsg[32];
    u8 digest[32];

    /* (SRESmaster || SRESslave || ACO) = h5(S, R1, R2)
     *      = hmac_sha256s(R1 || R2) / 2^128
     */

    UCHAR R1[16];
    UCHAR R2[16];

    memcpy(&R1[0], R1_orig, 16);
    memcpy(&R2[0], R2_orig, 16);

    bz_auth_convert_to_msb(&R1[0], 16);
    bz_auth_convert_to_msb(&R2[0], 16);	

    memcpy(&cmsg[0], R1, 16);
    memcpy(&cmsg[16], R2, 16);	
    hmac_sha256(S, 16, cmsg, 32, &digest[0]);
    memcpy(&SRESmaster[0], &digest[0], 4); /* only 4 most significant bytes */
    memcpy(&SRESslave[0], &digest[4], 4);
    memcpy(&ACO[0], &digest[8], 8);	

    bz_auth_convert_to_msb(&SRESmaster[0], 4);
    bz_auth_convert_to_msb(&SRESslave[0], 4);	

#ifdef _CCH_SC_ECDH_P256_LOG
    RT_BT_LOG(RED, CCH_DBG_159, 4,S[0], S[1], S[2], S[3]);
    RT_BT_LOG(RED, YL_DBG_HEX_8, 8,R1[0], R1[1], R1[2], R1[3], R2[0], R2[1], R2[2], R2[3]);
#endif

#ifdef _CCH_SC_ECDH_P256_LOG
    RT_BT_LOG(RED, CCH_DBG_159, 4,ACO[0], ACO[1], ACO[2], ACO[3]);
    RT_BT_LOG(RED, CCH_DBG_159, 4,SRESmaster[0], SRESmaster[1], SRESslave[0], SRESslave[1]);
#endif	

}


#endif

