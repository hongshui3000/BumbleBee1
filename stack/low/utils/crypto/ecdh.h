/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file ecdh.h
 *  Contains the Interface definition of ECDH functions required by
 *  the Cryptographic modules.
 *
 * \author Akshat Kumar
 * \date 2008-03-07
 */

#ifndef _ECDH_H_
#define _ECDH_H_

/* ========================= Include File Section ========================= */
#include "btypes.h"
#include "mpal.h"

/* ============================= API Section ============================== */

//#define MIXED_SCALAR_MULT_BUF_COUNT_MAX     (1 << 2)
#define MIXED_SCALAR_MULT_BUF_COUNT_MAX     (1 << 1)

typedef struct MIXED_SCALAR_MULT_ARGUMENT_S_ {
    UINT8 *key_x;
    UINT8 *key_y;

#ifndef _CCH_SC_ECDH_P256_
    DIGIT_S Qx[MPAL_MAX_LEN];
    DIGIT_S Qy[MPAL_MAX_LEN];
    DIGIT_S Qz[MPAL_MAX_LEN];
    DIGIT_S x[MPAL_PRIME_LEN];
    DIGIT_S y[MPAL_PRIME_LEN];
    DIGIT_S s[MPAL_PRIME_LEN];       
#else
    DIGIT_S Qx[MPAL_PRIME_LEN_MAX_X2];
    DIGIT_S Qy[MPAL_PRIME_LEN_MAX_X2];
    DIGIT_S Qz[MPAL_PRIME_LEN_MAX_X2];
    DIGIT_S x[MPAL_PRIME_LEN_MAX];
    DIGIT_S y[MPAL_PRIME_LEN_MAX];
    DIGIT_S s[MPAL_PRIME_LEN_MAX];  
#endif
	
    UINT16 len_qx;
    UINT16 len_qy;
    UINT16 len_qz;
    UINT16 len_x;
    UINT16 len_y;
    INT16 i;
    INT16 j;
#ifdef _SSP_DHKEY_CALCULATE_NEW_FLOW_
    UINT8 ce_index;
#endif

#ifdef _CCH_SC_ECDH_P256_
    UINT16 len_prime;
#endif
} MIXED_SCALAR_MULT_ARGUMENT_S;

typedef struct MIXED_SCALAR_MULT_MANAGER_S_ {
    MIXED_SCALAR_MULT_ARGUMENT_S argu[MIXED_SCALAR_MULT_BUF_COUNT_MAX];
    UINT8 index;  
} MIXED_SCALAR_MULT_MANAGER_S;

extern MIXED_SCALAR_MULT_MANAGER_S mpal_manager;


#ifdef _CCH_SC_ECDH_P256_
void mpal_mod(INOUT DIGIT_S* A, UINT16 len_A, UINT16 len_P);
void mpal_mod_by_p256(INOUT DIGIT_S* A, UINT16 len_A);
void mpal_inv(INOUT DIGIT_S* N, UINT16 len_N, UINT16 len_P);
#endif


void mixed_scalar_multiply_state0(DIGIT_S * S, u16 len_S, 
                        OUT DIGIT_S * X, OUT u16 * len_X, OUT DIGIT_S * Y, 
                        OUT u16 * len_Y, UINT8 * key_x, UINT8 * key_y, 
                        UINT8 partition);
void mixed_scalar_multiply_state1(UINT8 index, UINT8 partition);
void mixed_scalar_multiply_state2(UINT8 index);


u8 verify_point_on_curve(const DIGIT_S* X, u16 len_X,
        const DIGIT_S* Y, u16 len_Y);

#endif /* _ECDH_H_ */

