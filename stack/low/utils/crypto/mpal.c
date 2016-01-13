/**
 * \file mpal.c
 *  Multi-Precision Arithmetic Library(MPAL) implementation.
 *
 * \author Akshat Kumar
 * \date 2007-08-20
 */

/* ========================= Include File Section ========================= */
#include "mpal.h"
#include "mem.h"
#include "platform.h"

/* ====================== Macro Declaration Section ======================= */
#ifndef BZ_ASSERT
#include <assert.h>
#define BZ_ASSERT(c, n)     assert(c)
#endif

/* ==================== Structure Declaration Section ===================== */


/* ===================== Variable Declaration Section ===================== */

/* ================== Static Function Prototypes Section ================== */

void mpal_set_number(DIGIT_S* dwA, UINT16 dw_len, UINT32 val)
{
    UINT16 i;

    for (i = 0; i < dw_len; i++)
    {
        dwA[i] = val;
    }    
}


/* ===================== Function Definition Section ====================== */
/**
 * Performs \a A = \a A + \a B.
 *
 * \param A Input argument 1 in LSB format (A[0] should have the LSB and
 *          A[len_A-1] should have the MSB). The final result is stored in A
 *          and it should be big enough to hold the result.
 * \param len_A Length of A.
 * \param B Input argument 2 in LSB format (B[0] should have the LSB and
 *          B[len_B-1] should have the MSB).
 * \param len_B Length of B.
 *
 * \return None.
 */
void mpal_add_u8(INOUT u8* A, u16 len_A, const u8* B, u16 len_B)
{
    register int i;
    u16 temp;

    temp = 0;
    for (i = 0; i < len_B; i++)
    {
        temp += (u16)(A[i] + B[i]);
        A[i] = (u8)(temp & 0xFF);
        temp = (u16)(temp >> 8);
    }

    while (temp && (i < len_A))
    {
        temp += A[i];
        A[i] = (u8)(temp & 0xFF);
        temp = (u16)(temp >> 8);
        i++;
    }

    BZ_ASSERT(temp == 0, "The array A is too small to store the result");
}

/**
 * Performs \a A = \a A + \a B.
 *
 * \param A Input argument 1 in LSB format (A[0] should have the LSB and
 *          A[len_A-1] should have the MSB). The final result is stored in A
 *          and it should be big enough to hold the result.
 * \param len_A Length of A.
 * \param B Input argument 2 in LSB format (B[0] should have the LSB and
 *          B[len_B-1] should have the MSB).
 * \param len_B Length of B.
 *
 * \return None.
 */
void mpal_add(INOUT DIGIT_S* A, u16 len_A, const DIGIT_S* B, u16 len_B)
{
    register int i;
    UNSIGNED_BIG_DIGIT temp;

    temp = 0;
    for (i = 0; i < len_B; i++)
    {
        temp += (UNSIGNED_BIG_DIGIT)A[i] + B[i];
        A[i] = (DIGIT_S)(temp & MPAL_BASE_MASK);
        temp = (DIGIT_S)(temp >> MPAL_BASE);
    }

    while (temp && (i < len_A))
    {
        temp += A[i];
        A[i] = (DIGIT_S)(temp & MPAL_BASE_MASK);
        temp = (DIGIT_S)(temp >> MPAL_BASE);
        i++;
    }
    BZ_ASSERT(temp == 0, "The array A is too small to store the result");
}

/**
 * Performs \a A = \a A - \a B.
 *
 * \param A Input argument 1 in LSB format (A[0] should have the LSB and
 *          A[len_A-1] should have the MSB). The final result is stored in A
 *          and it should be big enough to hold the result.
 * \param len_A Length of A.
 * \param B Input argument 2 in LSB format (B[0] should have the LSB and
 *          B[len_B-1] should have the MSB).
 * \param len_B Length of B.
 *
 * \return None.
 */
void mpal_sub(INOUT DIGIT_S* A, u16 len_A, const DIGIT_S* B, u16 len_B)
{
    register int i;
    SIGNED_BIG_DIGIT temp;
    u16 carry = 0;

    for (i = 0; i < len_B; i++)
    {
        temp = (SIGNED_BIG_DIGIT)A[i] - B[i] - carry;
        if (temp < 0)
        {
            carry = 1;
            temp += ((UNSIGNED_BIG_DIGIT)1 << MPAL_BASE);
        }
        else
        {
            carry = 0;
        }
        A[i]  = (DIGIT_S)(MPAL_BASE_MASK & temp);
    }
    if(carry)
    {
        for(; i < len_A && A[i] == 0; i++)
        {
            A[i] = MPAL_BASE_MASK;
        }
        if(i<len_A)
        {
            A[i]--;
        }
    }
}


/**
 * Performs \a C = \a A * \a B. Note : Is susceptible to buffer overflows
 *
 * \param A Input argument 1 in LSB format (A[0] should have the LSB and
 *          A[len_A-1] should have the MSB).
 * \param len_A Length of A.
 * \param B Input argument 2 in LSB format (B[0] should have the LSB and
 *          B[len_B-1] should have the MSB).
 * \param len_B Length of B.
 * \param C Output argument 3 in LSB format. It should be atleast of size
 *          (len_A + len_B) - if the output is less, it will be padded
 *          with zeroes.
 *
 * \return None.
 */
void mpal_mult(const DIGIT_S* A, u16 len_A, const DIGIT_S* B, u16 len_B,
        OUT DIGIT_S* C)
{
    register int i, j;
    UNSIGNED_BIG_DIGIT temp;
    MPAL_MACRO_DEF;

    mpal_trim(A, len_A, len_A);
    mpal_trim(B, len_B, len_B);
    mpal_set_number(C, len_A + len_B, 0);

    for(i = 0; i < len_A; i++)
    {
        temp = 0;        
        for (j = 0; j < len_B; j++)
        {
            temp += (UNSIGNED_BIG_DIGIT)A[i] * B[j] + C[i+j];
            C[i+j] = (DIGIT_S)(temp & MPAL_BASE_MASK);
            temp = (DIGIT_S)(temp >> MPAL_BASE);
        }
        j += i;
        while (temp && (j < (len_A + len_B)))
        {
            temp += C[j];
            C[j] = (DIGIT_S)(temp & MPAL_BASE_MASK);
            temp = (DIGIT_S)(temp >> MPAL_BASE);
            j++;
        }
        BZ_ASSERT(temp == 0, "This should never happen");
    }
}

/**
 * Compares two given parameters.
 *
 * \param A Input argument 1 in LSB format (A[0] should have the LSB and
 *          A[len_A-1] should have the MSB).
 * \param len_A Length of A.
 * \param B Input argument 2 in LSB format (B[0] should have the LSB and
 *          B[len_B-1] should have the MSB).
 * \param len_B Length of B.
 *
 * \return  An integer less than, equal to, or greater than zero if A is found,
 *          respectively, to be less than, to match, or be greater than B.
 */
s8 mpal_compare(const DIGIT_S* A, u16 len_A, const DIGIT_S* B, u16 len_B)
{
    MPAL_MACRO_DEF;
    register int i;

    mpal_trim(A, len_A, len_A);
    mpal_trim(B, len_B, len_B);

    if(len_A > len_B)
    {
        return 1;
    }
    else if(len_A < len_B)
    {
        return -1;
    }

    for( i = len_A-1; i >= 0; i--)
    {
        if(A[i] > B[i])
        {
            return 1;
        }
        else if(A[i] < B[i])
        {
            return -1;
        }
    }

    return 0;
}


/**
 * Performs A = A >> 1 (non cyclical shift)
 *
 * \param A Input argument 1 in LSB format (A[0] should have the LSB and
 *          A[len_A-1] should have the MSB).
 * \param len_A Length of A.
 *
 * \return New length of the number.
 */
u16 mpal_right_shift(INOUT DIGIT_S* A,u16 len_A)
{
    register int i, iub;

    iub = len_A - 1;    
    for(i = 0; i < iub; i++)
    {
        A[i] = (A[i] >> 1) | ((A[i + 1] & 0x01) << (MPAL_BASE-1)) ;
    }
    A[i] >>= 1;
    if(A[i] == 0)
    {
        len_A--;
    }

    return len_A;
}

void mpal_mult_by_left_shift(INOUT DIGIT_S* A,u16 len_A, UINT8 ls, OUT DIGIT_S* C)
{
    register int i;

    mpal_trim(A, len_A, len_A);    
    C[0] = 0;    

    for (i = 0; i < len_A; i++)
    {
        C[i] |= A[i] << ls;
        C[i + 1] = A[i] >> (MPAL_BASE - ls);
    }
}


void mpal_square(const DIGIT_S* A, u16 len_A, OUT DIGIT_S* C)
{
    register int i, j;
    UNSIGNED_BIG_DIGIT temp;
    UINT16 len;
    UINT8 pre;
    UINT8 next;    

    MPAL_MACRO_DEF;

    mpal_trim(A, len_A, len_A);

    len = len_A << 1;

    mpal_set_number(C, len, 0);

    for(i = 0; i < len_A; i++)
    {
        temp = 0;
        for (j = i + 1; j < len_A; j++)
        {   
            temp += (UNSIGNED_BIG_DIGIT)A[i] * A[j] + C[i+j];
            C[i+j] = (DIGIT_S)(temp & MPAL_BASE_MASK);
            temp = (DIGIT_S)(temp >> MPAL_BASE);
        }
        j += i;
        while (temp && (j < len))
        {
            temp += C[j];
            C[j] = (DIGIT_S)(temp & MPAL_BASE_MASK);
            temp = (DIGIT_S)(temp >> MPAL_BASE);
            j++;
        }
    }

    pre = 0;
    for (i = 0; i < len; i++)
    {
        next = C[i] >> 31;
        C[i] = (C[i] << 1) | pre;
        pre = next;
    }

    for(i = 0; i < len_A; i++)
    {     
        j = i << 1;
        temp = (UNSIGNED_BIG_DIGIT)A[i] * A[i] + C[j];
        C[j] = (DIGIT_S)(temp & MPAL_BASE_MASK);
        temp = (DIGIT_S)(temp >> MPAL_BASE);
        j++;        
        while (temp && (j < len))
        {
            temp += C[j];
            C[j] = (DIGIT_S)(temp & MPAL_BASE_MASK);
            temp = (DIGIT_S)(temp >> MPAL_BASE);
            j++;
        }
    }  
}


/**
 * Performs \a A = \a A % \a B.
 * Modulus is calculated with repeated subtraction - so time
 * complexity is roughly exponential with respect to number of
 * digits.
 *
 * \param A Input argument 1 in LSB format (A[0] should have the LSB and
 *          A[len_A-1] should have the MSB).
 * \param len_A Length of A.
 * \param B Input argument 2 in LSB format (B[0] should have the LSB and
 *          B[len_B-1] should have the MSB).
 * \param len_B Length of B.
 *
 * \return None.
 */
void mpal_mod_by_sub(INOUT DIGIT_S* A, u16 len_A, const DIGIT_S* B, u16 len_B)
{
    MPAL_MACRO_DEF;

    while(mpal_compare(A, len_A, B, len_B) > 0)
    {
        mpal_sub(A, len_A, B, len_B);
        mpal_trim(A, len_A, len_A);
    }
}

void convert_to_lsb(u8* bytes, u8 len)
{
    register int i, j, iub;
    u8 temp;

    iub = len >> 1;
    for (i = 0, j = len-1; i < iub; i++, j--)
    {
        temp = bytes[i];
        bytes[i] = bytes[j];
        bytes[j] = temp;
    }
}

void memcpy_and_swap(u8* dest, u8* src, u8 len)
{
    register int i, j;

    for (i = 0, j = len-1; i < len; i++, j--)
    {
        dest[j] = src[i];
    }
}

INLINE u16 mpal_trim_func(DIGIT_S* A, u16 len)
{
    while(len  && A[--len]==0);
    return (len + 1);
}

