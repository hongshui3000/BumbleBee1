/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file mpal.h
 *  Multi-Precision Arithmetic Library(MPAL) interface.
 *
 * \author Santhosh kumar M
 * \date 2007-08-20
 */

#ifndef _MPAL_H_
#define _MPAL_H_

/* ========================= Include File Section ========================= */
#include "btypes.h"
#include "compiler.h"
#include "platform.h"

/* ==================== Data Types Declaration Section ==================== */
typedef s64 SIGNED_BIG_DIGIT;
typedef u64 UNSIGNED_BIG_DIGIT;
typedef u32 DIGIT_S;

/* ====================== Macro Declaration Section ======================= */
#define MPAL_MAX_LEN   16
#define MPAL_PRIME_LEN 6       // 32*6

#ifdef _CCH_SC_ECDH_P256_
#define MPAL_PRIME_LEN_MAX 8
#define MPAL_PRIME_LEN_MAX_X2 16  // MAXX2+4
#define MPAL_PRIME_LEN_P256 8 // 32*8
#define MPAL_PRIME_LEN_P256_X2 16


#define MPAL_INV_LOOP 100
#endif


#define MPAL_BASE                32
#define MPAL_BASE_MASK           0xFFFFFFFF

void mpal_set_number(DIGIT_S* dwA, UINT16 dw_len, UINT32 val);

/* Unsafe, fix this - or use proper Notation. Also depending on sizeof
 * (DIGIT ) multiplier*/
#define MPAL_MACRO_DEF

/* Call MPAL_MACRO_DEF in initialization section, before calling this macro */
INLINE u16 mpal_trim_func(DIGIT_S* A, u16 len);
#define mpal_trim(A,len,out_len) do{out_len = mpal_trim_func((DIGIT_S*)A, len);}while(0)

void convert_to_lsb(u8* bytes, u8 len);
#define convert_to_msb(b, l) convert_to_lsb((b), (l))

void memcpy_and_swap(u8 * dest, u8 * src, u8 len); // added by austin

/* ============================= API Section ============================== */
void mpal_add_u8(INOUT u8* A, u16 len_A, const u8* B, u16 len_B);
void mpal_add(INOUT DIGIT_S* A, u16 len_A, const DIGIT_S* B, u16 len_B);
void mpal_sub(INOUT DIGIT_S* A, u16 len_A, const DIGIT_S* B, u16 len_B);
void mpal_mult(const DIGIT_S* A, u16 len_A, const DIGIT_S* B, u16 len_B,
        OUT DIGIT_S* C);
void mpal_mult_by_left_shift(INOUT DIGIT_S * A, u16 len_A, UINT8 ls, OUT DIGIT_S * C);
void mpal_square(const DIGIT_S * A, u16 len_A, OUT DIGIT_S * C);

s8   mpal_compare(const DIGIT_S* A, u16 len_A, const DIGIT_S* B, u16 len_B);
u16  mpal_right_shift(INOUT DIGIT_S* A,u16 len_A);
void mpal_mod_by_sub(INOUT DIGIT_S* A, u16 len_A, const DIGIT_S* B, u16 len_B);
#endif /* _MPAL_H_ */

