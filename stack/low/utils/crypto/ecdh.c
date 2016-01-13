enum { __FILE_NUM__= 211 };

/**
 * \file ecdh.c
 *  Elliptic Curve Diffie Hellmen Implementation
 *
 *  Contains the implementation of  Elliptic Curve Diffie Hellmen functions
 *  required by the  Host Controller Firmware.
 *
 *  Many of the functions in this module are not \b reentrant. It means that
 *  the call to any function has to be protected with synchronization
 *  primitives (eg. mutexes, DISABLE_INTERRUPTS), if it has to be called
 *  from an interrupt/another thread of execution.
 *
 * \author Akshat Kumar
 * \date 10/10/08
 */

/* ========================= Include File Section ========================= */
#include "crypto.h"
#include "mpal.h"
#include "mem.h"
#include "bz_debug.h"
#include "logger.h"

/* ====================== Macro Declaration Section ======================= */
#ifndef BZ_ASSERT
#include <assert.h>
#define BZ_ASSERT(c, n)     assert(c)
#endif

#include "ecdh.h"
#include "lmp_internal.h"

#include "bz_auth_internal.h"

/* ==================== Structure Declaration Section ===================== */


/* ===================== Variable Declaration Section ===================== */
/** Prime number for field arithmetic on elliptic curve P192. **/
static const DIGIT_S PRIME[MPAL_PRIME_LEN] =
    {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFE, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF};

/** Parameter 'a' of the elliptic curve. It is equivalent to (-3) % Prime **/
static const DIGIT_S  curve_a[MPAL_PRIME_LEN] =
    {0xFFFFFFFC, 0xFFFFFFFF, 0xFFFFFFFE, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF};

static const u16 len_curve_a = MPAL_PRIME_LEN;

/** Parameter 'b' of the elliptic curve. **/
static const DIGIT_S  curve_b[MPAL_PRIME_LEN] =
    {0xc146b9b1, 0xfeb8deec, 0x72243049, 0x0fa7e9ab, 0xe59c80e7, 0x64210519};

static const u16 len_curve_b = MPAL_PRIME_LEN;

#ifdef _CCH_SC_ECDH_P256_
// All LSB first
// Prime number for field arithmetic on elliptic curve P256.
// p256 = 2^256 - 2^224 + 2^192 + 2^96 -1
const DIGIT_S PRIME_P256[MPAL_PRIME_LEN_P256] =
    {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0x0, 0x0, 0x0, 0x1, 0xFFFFFFFF};

// Parameter 'a' of the elliptic curve. It is equivalent to (-3) % Prime
const DIGIT_S  curve_a_p256[MPAL_PRIME_LEN_P256] =
    {0xFFFFFFFC, 0xFFFFFFFF, 0xFFFFFFFF, 0x0, 0x0, 0x0, 0x1, 0xFFFFFFFF};

const u16 len_curve_a_p256 = MPAL_PRIME_LEN_P256;

// Parameter 'b' of the elliptic curve.
const DIGIT_S  curve_b_p256[MPAL_PRIME_LEN_P256] =
    {0x27d2604b, 0x3bce3c3e, 0xcc53b0f6, 0x651d06b0, 0x769886bc, 0xb3ebbd55, 0xaa3a93e7, 0x5ac635d8};

const u16 len_curve_b_p256 = MPAL_PRIME_LEN_P256;

#endif

#ifndef _ENABLE_RETENTION_FLOW_FOR_DLPS_
MIXED_SCALAR_MULT_MANAGER_S mpal_manager;
#else
SECTION_LOW_BSS MIXED_SCALAR_MULT_MANAGER_S mpal_manager;
#endif

/* ================== Static Function Prototypes Section ================== */
void mpal_mod_by_p192(INOUT DIGIT_S* A, u16 len_A);
#ifndef _CCH_SC_ECDH_P256_
void mpal_inv_by_p192(INOUT DIGIT_S* N, u16 len_N);
#endif
void jacobian_point_double(MIXED_SCALAR_MULT_ARGUMENT_S *argu);
void mixed_point_addition(MIXED_SCALAR_MULT_ARGUMENT_S *argu);

#ifdef _CCH_SC_ECDH_P256_
void mpal_mod(INOUT DIGIT_S* A, UINT16 len_A, UINT16 len_P)
{
    if(len_P == 6)
    {
        mpal_mod_by_p192(A, len_A);
    }else if(len_P == 8)
    {
        mpal_mod_by_p256(A, len_A);
    }
}
#endif
/* ===================== Function Definition Section ====================== */
/**
 * Computes modulus of a given number with respect to Prime P192. Uses fast
 * modulo reduction based on property of NIST specified *(pseudo- mersenne)
 * primes.
 *
 * \param A Input number for which the mod will be calculated. The number
 *          The number should be in LSB format. The callee will reset this
 *          number to new value.
 * \param len Length of the input number \a A (in sizeof DIGIT_S).
 *           The length should never be greater then twice the length of prime.
 * \return None.
 *
 * \warning This function will calculate modulus of a number only with respect
 *          to the prime specified for P192 curve - and only for numbers less
 *          than 48 bytes in length ie; 2 * MPAL_PRIME_LEN.
 *          This function is not \b reentrant.
 */
void mpal_mod_by_p192(INOUT DIGIT_S* A, u16 len_A)
{
    DIGIT_S s1[MPAL_PRIME_LEN];
    DIGIT_S s2[MPAL_PRIME_LEN];
    DIGIT_S s3[MPAL_PRIME_LEN];
    DIGIT_S s4[MPAL_PRIME_LEN];
    DIGIT_S scratch[2*MPAL_PRIME_LEN];
    register int temp;
    UINT8 i;  

    MPAL_MACRO_DEF;

    // Put check - if smaller return

    /* Reset static variables */
    
    mpal_trim(A, len_A, len_A);
    BZ_ASSERT(len_A <= 2*MPAL_PRIME_LEN,
            "The array A is to be big to be specially reduced");

    for (i = 0; i < (2*MPAL_PRIME_LEN); i++)
    {
        scratch[i] = (i < len_A) ? A[i] : 0;
    }    

    /* s1 = (c2,c1,c0) ( ci = 64 bit (8 byte)) = 64*3/MPAL_BASE*/
    s1[0] = scratch[0];    
    s1[1] = scratch[1];
    s1[2] = scratch[2];
    s1[3] = scratch[3];
    s1[4] = scratch[4];
    s1[5] = scratch[5];
    
    /* s2 = (0,c3,c3) ( ci = 64 bit (8 byte)) */
    s2[0] = scratch[6];
    s2[1] = scratch[7];
    s2[2] = scratch[6]; 
    s2[3] = scratch[7];
    s2[4] = 0;
    s2[5] = 0;

    /* s3 = (c4,c4,0) ( ci = 64 bit (8 byte)) */  
    s3[0] = 0;
    s3[1] = 0;   
    s3[2] = scratch[8];
    s3[3] = scratch[9];
    s3[4] = scratch[8]; 
    s3[5] = scratch[9];   

    /* s4 = (c5,c5,c5) ( ci = 64 bit (8 byte)) */
    s4[0] = scratch[10];
    s4[1] = scratch[11];
    s4[2] = scratch[10]; 
    s4[3] = scratch[11];    
    s4[4] = scratch[10]; 
    s4[5] = scratch[11];   

    /* Set scratch = s1 + s2 + s3 + s4 */
    /* because s1[5:0] = scratch[5:0], we only need to clear scratch[11:6]*/
    scratch[MPAL_PRIME_LEN+0] = 0;
    scratch[MPAL_PRIME_LEN+1] = 0;
    scratch[MPAL_PRIME_LEN+2] = 0;
    scratch[MPAL_PRIME_LEN+3] = 0;
    scratch[MPAL_PRIME_LEN+4] = 0;
    scratch[MPAL_PRIME_LEN+5] = 0;

    mpal_add(scratch, MPAL_PRIME_LEN+6, s2, MPAL_PRIME_LEN);
    mpal_add(scratch, MPAL_PRIME_LEN+6, s3, MPAL_PRIME_LEN);
    mpal_add(scratch, MPAL_PRIME_LEN+6, s4, MPAL_PRIME_LEN);

    /* Set temp = scratch.length */
    mpal_trim(scratch, MPAL_PRIME_LEN+6, temp);

    /* Now we can find modulus of the number by simply repeated subtraction */
    mpal_mod_by_sub(scratch, temp, PRIME, MPAL_PRIME_LEN);

    /* Set output */   
    mpal_set_number(A, len_A, 0);
    mpal_add(A, MPAL_PRIME_LEN, scratch, MPAL_PRIME_LEN);
}

#ifdef _CCH_SC_ECDH_P256_
void mpal_mod_by_p256(INOUT DIGIT_S* A, UINT16 len_A)
{
//    DIGIT_S s1[MPAL_PRIME_LEN_P256];
    DIGIT_S s2[MPAL_PRIME_LEN_P256];
    DIGIT_S s3[MPAL_PRIME_LEN_P256];
    DIGIT_S s4[MPAL_PRIME_LEN_P256];
    DIGIT_S s5[MPAL_PRIME_LEN_P256];
    DIGIT_S s6[MPAL_PRIME_LEN_P256];
    DIGIT_S s7[MPAL_PRIME_LEN_P256];
    DIGIT_S s8[MPAL_PRIME_LEN_P256];
    DIGIT_S s9[MPAL_PRIME_LEN_P256];
	
    DIGIT_S scratch[MPAL_PRIME_LEN_P256_X2];
    UINT8 length = (MPAL_PRIME_LEN_P256_X2);	
    register int temp;
    UINT8 i;  

    MPAL_MACRO_DEF;

    // Put check - if smaller return

    /* Reset static variables */
    
    mpal_trim(A, len_A, len_A);

	
    BZ_ASSERT(len_A <= MPAL_PRIME_LEN_P256_X2,
            "The array A is to be big to be specially reduced");

    for (i = 0; i < MPAL_PRIME_LEN_P256_X2; i++)
    {
        scratch[i] = (i < len_A) ? A[i] : 0;
    }    

    // s1 = (c7, c6, c5, c4, c3, c2, c1, c0) ( ci = 32 bit (4 byte))
/*    s1[0] = scratch[0];    
    s1[1] = scratch[1];
    s1[2] = scratch[2];
    s1[3] = scratch[3];
    s1[4] = scratch[4];
    s1[5] = scratch[5];
    s1[6] = scratch[6];
    s1[7] = scratch[7];*/

	
    // s2 = (c15, c14, c13, c12, c11, 0, 0, 0) ( ci = 32 bit (4 byte))
    s2[0] = 0;    
    s2[1] = 0;
    s2[2] = 0;
    s2[3] = scratch[11];
    s2[4] = scratch[12];
    s2[5] = scratch[13];
    s2[6] = scratch[14];
    s2[7] = scratch[15];

    // s3 = (0, c15, c14, c13, c12, 0, 0, 0) ( ci = 32 bit (4 byte))
    s3[0] = 0;    
    s3[1] = 0;
    s3[2] = 0;
    s3[3] = scratch[12];
    s3[4] = scratch[13];
    s3[5] = scratch[14];
    s3[6] = scratch[15];
    s3[7] = 0;

    // s4 = (c15, c14, 0, 0, 0, c10, c9, c8) ( ci = 32 bit (4 byte))
    s4[0] = scratch[8];    
    s4[1] = scratch[9];
    s4[2] = scratch[10];
    s4[3] = 0;
    s4[4] = 0;
    s4[5] = 0;
    s4[6] = scratch[14];
    s4[7] = scratch[15];

    // s5 = (c8, c13, c15, c14, c13, c11, c10, c9) ( ci = 32 bit (4 byte))
    s5[0] = scratch[9];    
    s5[1] = scratch[10];
    s5[2] = scratch[11];
    s5[3] = scratch[13];
    s5[4] = scratch[14];
    s5[5] = scratch[15];
    s5[6] = scratch[13];
    s5[7] = scratch[8];


    // s6 = (c10, c8, 0, 0, 0, c13, c12, c11) ( ci = 32 bit (4 byte))
    s6[0] = scratch[11];    
    s6[1] = scratch[12];
    s6[2] = scratch[13];
    s6[3] = 0;
    s6[4] = 0;
    s6[5] = 0;
    s6[6] = scratch[8];
    s6[7] = scratch[10];

    // s7 = (c11, c9, 0, 0, c15, c14, c13, c12) ( ci = 32 bit (4 byte))
    s7[0] = scratch[12];    
    s7[1] = scratch[13];
    s7[2] = scratch[14];
    s7[3] = scratch[15];
    s7[4] = 0;
    s7[5] = 0;
    s7[6] = scratch[9];
    s7[7] = scratch[11];

    // s8 = (c12, 0, c10, c9, c8, c15, c14, c13) ( ci = 32 bit (4 byte))
    s8[0] = scratch[13];    
    s8[1] = scratch[14];
    s8[2] = scratch[15];
    s8[3] = scratch[8];
    s8[4] = scratch[9];
    s8[5] = scratch[10];
    s8[6] = 0;
    s8[7] = scratch[12];

    // s9 = (c13, 0, c11, c10, c9, 0, c15, c14) ( ci = 32 bit (4 byte))
    s9[0] = scratch[14];    
    s9[1] = scratch[15];
    s9[2] = 0;
    s9[3] = scratch[9];
    s9[4] = scratch[10];
    s9[5] = scratch[11];
    s9[6] = 0;
    s9[7] = scratch[13];


	
    /* Set scratch = s1 + s2 + s3 + s4 */
    /* because s1[5:0] = scratch[5:0], we only need to clear scratch[11:6]*/

    UINT8 ind;

    for( ind = 0; ind < (MPAL_PRIME_LEN_P256); ind ++ )
    {
        scratch[MPAL_PRIME_LEN_P256+ind] = 0;
	}


#if 0
    mpal_add(scratch, length, s2, MPAL_PRIME_LEN_P256);
    mpal_add(scratch, length, s2, MPAL_PRIME_LEN_P256);	
    mpal_add(scratch, length, s3, MPAL_PRIME_LEN_P256);
    mpal_add(scratch, length, s3, MPAL_PRIME_LEN_P256);
    mpal_add(scratch, length, s4, MPAL_PRIME_LEN_P256);	
    mpal_add(scratch, length, s5, MPAL_PRIME_LEN_P256);

    mpal_sub(scratch, length, s6, MPAL_PRIME_LEN_P256);
    mpal_sub(scratch, length, s7, MPAL_PRIME_LEN_P256);
    mpal_sub(scratch, length, s8, MPAL_PRIME_LEN_P256);
    mpal_sub(scratch, length, s9, MPAL_PRIME_LEN_P256);
	

    /* Set temp = scratch.length */
    mpal_trim(scratch, length, temp);

    /* Now we can find modulus of the number by simply repeated subtraction */
    mpal_mod_by_sub(scratch, temp, PRIME_P256, MPAL_PRIME_LEN_P256);
#else
    mpal_add(scratch, length, s2, MPAL_PRIME_LEN_P256);
    mpal_add(scratch, length, s2, MPAL_PRIME_LEN_P256);	
    mpal_add(scratch, length, s3, MPAL_PRIME_LEN_P256);
    mpal_add(scratch, length, s3, MPAL_PRIME_LEN_P256);
    mpal_add(scratch, length, s4, MPAL_PRIME_LEN_P256);	
    mpal_add(scratch, length, s5, MPAL_PRIME_LEN_P256);


    mpal_trim(scratch, length, temp);
    mpal_mod_by_sub(scratch, temp, PRIME_P256, MPAL_PRIME_LEN_P256);

	
    mpal_trim(scratch, length, temp);
    if(mpal_compare(scratch, temp, s6, MPAL_PRIME_LEN_P256) < 0)
    {
        mpal_add(scratch, length, PRIME_P256, MPAL_PRIME_LEN_P256);
    }

    mpal_trim(scratch, length, temp);
    if(mpal_compare(scratch, temp, s6, MPAL_PRIME_LEN_P256) <= 0)
    {
        RT_BT_LOG(RED, CCH_DBG_019, 0,0);
    }



    mpal_sub(scratch, length, s6, MPAL_PRIME_LEN_P256);

    mpal_trim(scratch, length, temp);
    if(mpal_compare(scratch, temp, s7, MPAL_PRIME_LEN_P256) < 0)
    {
        mpal_add(scratch, length, PRIME_P256, MPAL_PRIME_LEN_P256);
    }


    mpal_trim(scratch, length, temp);
    if(mpal_compare(scratch, temp, s7, MPAL_PRIME_LEN_P256) <= 0)
    {
        RT_BT_LOG(RED, CCH_DBG_019, 0,0);
    }

    mpal_sub(scratch, length, s7, MPAL_PRIME_LEN_P256);

    mpal_trim(scratch, length, temp);
    if(mpal_compare(scratch, temp, s8, MPAL_PRIME_LEN_P256) < 0)
    {
        mpal_add(scratch, length, PRIME_P256, MPAL_PRIME_LEN_P256);
    }


    mpal_trim(scratch, length, temp);
    if(mpal_compare(scratch, temp, s8, MPAL_PRIME_LEN_P256) <= 0)
    {
        RT_BT_LOG(RED, CCH_DBG_019, 0,0);
    }
	
    mpal_sub(scratch, length, s8, MPAL_PRIME_LEN_P256);

    mpal_trim(scratch, length, temp);
    if(mpal_compare(scratch, temp, s9, MPAL_PRIME_LEN_P256) < 0)
    {
        mpal_add(scratch, length, PRIME_P256, MPAL_PRIME_LEN_P256);
    }


    mpal_trim(scratch, length, temp);
    if(mpal_compare(scratch, temp, s9, MPAL_PRIME_LEN_P256) <= 0)
    {
        RT_BT_LOG(RED, CCH_DBG_019, 0,0);
    }
	
    mpal_sub(scratch, length, s9, MPAL_PRIME_LEN_P256);

    mpal_trim(scratch, length, temp);
    mpal_mod_by_sub(scratch, temp, PRIME_P256, MPAL_PRIME_LEN_P256);
	
#endif
    /* Set output */   
    mpal_set_number(A, len_A, 0);
    mpal_add(A, MPAL_PRIME_LEN_P256, scratch, MPAL_PRIME_LEN_P256);
}
#endif

#ifdef _CCH_SC_ECDH_P256_

// len_P: 6 or 8
void mpal_inv(INOUT DIGIT_S* N, UINT16 len_N, UINT16 len_P)
{

#ifdef _CCH_TEST_SEC_1
    clock_inv_s = BB_read_native_clock();

    DEF_CRITICAL_SECTION_STORAGE;
    MINT_OS_ENTER_CRITICAL();
	
#endif

    DIGIT_S u[MPAL_PRIME_LEN_MAX_X2];
    DIGIT_S v[MPAL_PRIME_LEN_MAX_X2];
    DIGIT_S A[MPAL_PRIME_LEN_MAX_X2];
    DIGIT_S C[MPAL_PRIME_LEN_MAX_X2];
    DIGIT_S  zero[1] = {0};
    u16 len_u, len_v, len_A, len_C;
    UINT8 i;
    s8 result;

    if( (len_P != 6)&&(len_P!=8) )
    {
#ifdef _CCH_TEST_SEC_1
        MINT_OS_EXIT_CRITICAL();   
#endif
        return;
    }
	
    MPAL_MACRO_DEF;

    /* Reset static variables */

    mpal_trim(N, len_N, len_N);

    /* Clear and initialize all scratch buffers (fast init by austin) */  
    for (i = 0; i < MPAL_PRIME_LEN_MAX_X2; i++)
    {
        u[i] = (i < len_N) ? N[i] : 0;        
    }


    mpal_set_number(A, MPAL_PRIME_LEN_MAX_X2, 0);
	
    A[0] = 1;

    mpal_set_number(C, MPAL_PRIME_LEN_MAX_X2, 0);

    mpal_set_number(v, MPAL_PRIME_LEN_MAX_X2, 0);

    if(len_P == 6)
    {
	    for (i = 0; i < len_P; i++)
	    {
	        v[i] = PRIME[i];       
	    }
    }else if(len_P == 8)
	{
	    for (i = 0; i < len_P; i++)
	    {
	        v[i] = PRIME_P256[i];       
	    }  
    }
    len_u = len_N;
    len_v = len_P;
    len_A = 1;
    len_C = 0;

    // tomake : Make a macro isZero
    while (1)        
    {
        result = mpal_compare(u, len_u, zero, 1);
        if (result == 0)
        {
            break;
        }   

        while(((u[0] & 0x01) == 0) && (result != 0))
        {
            len_u = mpal_right_shift(u, len_u);

            if (A[0] & 0x01)
            {
                len_A = (len_P >= len_A) ?
                        (len_P+1) : (len_A + 1);

                if(len_P == 6)
                {
                    mpal_add(A, len_A, PRIME, len_P);
                }else if(len_P == 8)
				{
                    mpal_add(A, len_A, PRIME_P256, len_P);
                }

                mpal_trim(A, len_A, len_A);
            }            
            len_A = mpal_right_shift(A, len_A);
            result = mpal_compare(u, len_u, zero, 1);            
        }        
        
        while(((v[0] & 0x01) == 0) && (mpal_compare(v, len_v, zero, 1) != 0))
        {
            len_v = mpal_right_shift(v, len_v);

            if (C[0] & 0x01)
            {
                len_C = (len_P >= len_C) ?
                        (len_P+1) : (len_C + 1);
                
                if(len_P == 6)
                {
                    mpal_add(C, len_C, PRIME, len_P);
                }else if(len_P == 8)
                {
                    mpal_add(C, len_C, PRIME_P256, len_P);
                }
				
				
                mpal_trim(C, len_C, len_C);
            }
            len_C = mpal_right_shift(C, len_C);
        }

        if (mpal_compare(u, len_u, v, len_v) >= 0)
        {
            mpal_sub(u, len_u, v, len_v);
            mpal_trim(u, len_u, len_u);
            
            while(mpal_compare(A, len_A, C, len_C) < 0)
            {
                if(len_P == 6)
                {
                    mpal_add(A, (len_P<<1), PRIME, len_P);
                }else if(len_P == 8)
				{
                    mpal_add(A, (len_P<<1), PRIME_P256, len_P);
                }
				
                mpal_trim(A, (len_P<<1), len_A);
            }
            mpal_sub(A, len_A, C, len_C);
            mpal_trim(A, len_A, len_A);
        }
        else
        {
            mpal_sub(v, len_v, u, len_u);
            mpal_trim(v, len_v, len_v);

            while(mpal_compare(C, len_C, A, len_A) < 0)
            {
                if(len_P == 6)
                {
                    mpal_add(C, (len_P<<1), PRIME, len_P);
                }else if(len_P == 8)
				{
                    mpal_add(C, (len_P<<1), PRIME_P256, len_P);
                }
				
				
                mpal_trim(C, (len_P<<1), len_C);
            }
            mpal_sub(C, len_C, A, len_A);
            mpal_trim(C, len_C, len_C);
        }
    }

    mpal_set_number(N, len_P, 0);
    mpal_add(N, len_P, C, len_C);


#ifdef _CCH_TEST_SEC_1
    MINT_OS_EXIT_CRITICAL();
    clock_inv_e = BB_read_native_clock();

    RT_BT_LOG(RED, YL_DBG_HEX_2, 2, clock_inv_s, clock_inv_e);

#endif
	
}
#endif

#ifndef _CCH_SC_ECDH_P256_
/**
 * Computes modulo inverse  of a given number with respect to Prime P192. Uses
 * Binary Extended Euclidean algorithm for GCD.
 *
 * \param A Input number for which the inverse will be calculated. The number
 *          The number should be in LSB format. The callee will reset this
 *          number to  new value.
 * \param len Length of the input number \a A (in sizeof DIGIT_S).
 *           The length should never be greater then twice the length of prime.
 * \return None.
 *
 * \warning This function will calculate inverse of a number only with respect
 *          to the prime specified for P192 curve - and only for numbers less
 *          than the prime itself. For numbers bigger than the primes, first
 *          explicity modulo reduce them, and then pass then to calculate the
 *          inverse.
 *          This function is not \b reentrant.
 */
void mpal_inv_by_p192(INOUT DIGIT_S* N, u16 len_N)
{
    DIGIT_S u[MPAL_MAX_LEN];
    DIGIT_S v[MPAL_MAX_LEN];
    DIGIT_S A[MPAL_MAX_LEN];
    DIGIT_S C[MPAL_MAX_LEN];
    DIGIT_S  zero[1] = {0};
    u16 len_u, len_v, len_A, len_C;
    UINT8 i;
    s8 result;
    
    MPAL_MACRO_DEF;

    /* Reset static variables */

    mpal_trim(N, len_N, len_N);

    /* Clear and initialize all scratch buffers (fast init by austin) */  
    for (i = 0; i < MPAL_MAX_LEN; i++)
    {
        u[i] = (i < len_N) ? N[i] : 0;        
    }

    A[0] = 1;
    A[1] = 0;
    A[2] = 0;
    A[3] = 0;
    A[4] = 0;
    A[5] = 0;
    A[6] = 0;
    A[7] = 0;
    A[8] = 0;
    A[9] = 0;
    A[10] = 0;
    A[11] = 0;
    A[12] = 0;
    A[13] = 0;
    A[14] = 0;
    A[15] = 0;

    C[0] = 0;
    C[1] = 0;
    C[2] = 0;
    C[3] = 0;
    C[4] = 0;
    C[5] = 0;
    C[6] = 0;
    C[7] = 0;
    C[8] = 0;
    C[9] = 0;
    C[10] = 0;
    C[11] = 0;
    C[12] = 0;
    C[13] = 0;
    C[14] = 0;
    C[15] = 0;

    v[0] = 0xFFFFFFFF;
    v[1] = 0xFFFFFFFF;
    v[2] = 0xFFFFFFFE;
    v[3] = 0xFFFFFFFF;
    v[4] = 0xFFFFFFFF;    
    v[5] = 0xFFFFFFFF;
    v[6] = 0;
    v[7] = 0;
    v[8] = 0;
    v[9] = 0;
    v[10] = 0;
    v[11] = 0;    
    v[12] = 0;
    v[13] = 0;
    v[14] = 0;
    v[15] = 0;   

    len_u = len_N;
    len_v = MPAL_PRIME_LEN;
    len_A = 1;
    len_C = 0;

    // tomake : Make a macro isZero
    while (1)        
    {
        result = mpal_compare(u, len_u, zero, 1);
        if (result == 0)
        {
            break;
        }   

        while(((u[0] & 0x01) == 0) && (result != 0))
        {
            len_u = mpal_right_shift(u, len_u);

            if (A[0] & 0x01)
            {
                len_A = (MPAL_PRIME_LEN >= len_A) ?
                        (MPAL_PRIME_LEN+1) : (len_A + 1);
                mpal_add(A, len_A, PRIME, MPAL_PRIME_LEN);
                mpal_trim(A, len_A, len_A);
            }            
            len_A = mpal_right_shift(A, len_A);
            result = mpal_compare(u, len_u, zero, 1);            
        }        
        
        while(((v[0] & 0x01) == 0) && (mpal_compare(v, len_v, zero, 1) != 0))
        {
            len_v = mpal_right_shift(v, len_v);

            if (C[0] & 0x01)
            {
                len_C = (MPAL_PRIME_LEN >= len_C) ?
                        (MPAL_PRIME_LEN+1) : (len_C + 1);
                mpal_add(C, len_C, PRIME, MPAL_PRIME_LEN);
                mpal_trim(C, len_C, len_C);
            }
            len_C = mpal_right_shift(C, len_C);
        }

        if (mpal_compare(u, len_u, v, len_v) >= 0)
        {
            mpal_sub(u, len_u, v, len_v);
            mpal_trim(u, len_u, len_u);
            
            while(mpal_compare(A, len_A, C, len_C) < 0)
            {
                mpal_add(A, MPAL_PRIME_LEN + 6, PRIME, MPAL_PRIME_LEN);
                mpal_trim(A, MPAL_PRIME_LEN + 6, len_A);
            }
            mpal_sub(A, len_A, C, len_C);
            mpal_trim(A, len_A, len_A);
        }
        else
        {
            mpal_sub(v, len_v, u, len_u);
            mpal_trim(v, len_v, len_v);

            while(mpal_compare(C, len_C, A, len_A) < 0)
            {
                mpal_add(C, MPAL_PRIME_LEN + 6, PRIME, MPAL_PRIME_LEN);
                mpal_trim(C, MPAL_PRIME_LEN + 6, len_C);
            }
            mpal_sub(C, len_C, A, len_A);
            mpal_trim(C, len_C, len_C);
        }
    }

    mpal_set_number(N, MPAL_PRIME_LEN, 0);
    mpal_add(N, MPAL_PRIME_LEN, C, len_C);
}
#endif
/****************************** Elliptic functions ****************************/

/**
 * Verifies whether given point lies on the elliptic curve - A.
 *
 * \param X     X - co-ordinate of point to check on curve - A .
 * \param X_len Length of the input number \a X(in sizeof DIGIT_S).
 * \param Y     Y - co-ordinate of point to check on curve - A .
 * \param Y_len Length of the input number \a Y(in sizeof DIGIT_S).
 *
 * \return TRUE if point lies on curve, else FALSE.
 *
 * \warning This function checks only if given point lies on the curve - it
 *          doesn't check if it is in correct subgroup.
 *          This function is not \b reentrant.
 */
u8 verify_point_on_curve(const DIGIT_S* X, u16 len_X, 
    const DIGIT_S* Y, u16 len_Y)
{
    MPAL_MACRO_DEF;
    DIGIT_S  zero[1] = {0};
#ifdef _CCH_SC_ECDH_P256_
    DIGIT_S scratch1[MPAL_PRIME_LEN_MAX_X2];
    DIGIT_S scratch2[MPAL_PRIME_LEN_MAX_X2];
#else
    DIGIT_S scratch1[MPAL_MAX_LEN];
    DIGIT_S scratch2[MPAL_MAX_LEN];
#endif

    u16 len_scratch1, len_scratch2;
    UINT8 i;
    

#ifdef _CCH_SC_ECDH_P256_

    UINT8 is_p256 = FALSE;
    UINT16 len_prime = len_X;
    if( len_X == MPAL_PRIME_LEN)
    {
        is_p256 = FALSE;
    }else if( len_X == MPAL_PRIME_LEN_P256)
    {
        is_p256 = TRUE;
    }else
    {
        return FALSE;
    }
    	
#endif


    mpal_trim(X, len_X, len_X);
    mpal_trim(Y, len_Y, len_Y);
    
    /* Check if X,Y E [1,PRIME-1] */
#ifdef _CCH_SC_ECDH_P256_
    if((mpal_compare(X, len_X, zero, 1) <= 0) ||
       ((is_p256 == 0)&&(mpal_compare(X, len_X, PRIME , MPAL_PRIME_LEN) >= 0)) ||
       ((is_p256 == 1)&&(mpal_compare(X, len_X, PRIME_P256 , MPAL_PRIME_LEN_P256) >= 0)))
#else
    if((mpal_compare(X, len_X, zero, 1) <= 0) ||
       (mpal_compare(X, len_X, PRIME , MPAL_PRIME_LEN) >= 0))
#endif
    {
        return FALSE;
    }


#ifdef _CCH_SC_ECDH_P256_
    if((mpal_compare(Y, len_Y, zero, 1) <= 0) ||
       ((is_p256 == 0)&&(mpal_compare(Y, len_Y, PRIME , MPAL_PRIME_LEN) >= 0)) ||
       ((is_p256 == 1)&&(mpal_compare(Y, len_Y, PRIME_P256 , MPAL_PRIME_LEN_P256) >= 0)))
#else
    if((mpal_compare(Y, len_Y, zero, 1) <= 0) ||
       (mpal_compare(Y, len_Y, PRIME , MPAL_PRIME_LEN) >= 0))
#endif
    {
        return FALSE;
    }

    /* Clear all scratch buffers */ 
#ifdef _CCH_SC_ECDH_P256_
    for (i = 0; i < MPAL_PRIME_LEN_MAX_X2; i++)
#else
    for (i = 0; i < MPAL_MAX_LEN; i++)
#endif
    {
        scratch1[i] = 0;
        scratch2[i] = 0;
    }

    /* Check y^2 == x^3 + a*x + b ( mod p192) */
    mpal_square(X, len_X, scratch1);

#ifdef _CCH_SC_ECDH_P256_
    if( is_p256 == 0)
    {
        mpal_mod_by_p192(scratch1, len_X << 1);
        mpal_add(scratch1, len_prime +1, curve_a, len_curve_a);
        mpal_mod_by_p192(scratch1, len_prime + 1);
    }else
    {
        mpal_mod_by_p256(scratch1,  (len_X<<1));
        mpal_add(scratch1, len_prime +1, curve_a_p256 , len_curve_a_p256 );
        mpal_mod_by_p256(scratch1, len_prime + 1);
    }

    mpal_trim(scratch1, len_prime, len_scratch1);
    mpal_mult(scratch1, len_scratch1, X, len_X, scratch2);


    if( is_p256 == 0)
    {
        mpal_mod_by_p192(scratch2, len_X + len_scratch1);
        mpal_add(scratch2, len_prime + 1, curve_b, len_curve_b);
        mpal_mod_by_p192(scratch2, len_prime + 1);
    }else
    {
        mpal_mod_by_p256(scratch2, len_X + len_scratch1);
        mpal_add(scratch2, len_prime + 1, curve_b_p256, len_curve_b_p256);
        mpal_mod_by_p256(scratch2, len_prime + 1);
    }

    mpal_trim(scratch2, len_prime, len_scratch2);

    mpal_set_number(scratch1, MPAL_PRIME_LEN_MAX_X2, 0);
    mpal_square(Y, len_Y, scratch1);   

    if( is_p256 == 0)
    {
        mpal_mod_by_p192(scratch1, len_Y << 1);
    }else
    {
        mpal_mod_by_p256(scratch1, len_Y << 1);
    }

    mpal_trim(scratch1, len_prime, len_scratch1);
    mpal_trim(scratch2, len_prime, len_scratch2);

#else
    mpal_mod_by_p192(scratch1, len_X << 1);
    mpal_add(scratch1, MPAL_PRIME_LEN +1, curve_a, len_curve_a);
    mpal_mod_by_p192(scratch1, MPAL_PRIME_LEN + 1);
    mpal_trim(scratch1, MPAL_PRIME_LEN, len_scratch1);
    mpal_mult(scratch1, len_scratch1, X, len_X, scratch2);
    mpal_mod_by_p192(scratch2, len_X + len_scratch1);
    mpal_add(scratch2, MPAL_PRIME_LEN + 1, curve_b, len_curve_b);
    mpal_mod_by_p192(scratch2, MPAL_PRIME_LEN + 1);
    mpal_trim(scratch2, MPAL_PRIME_LEN, len_scratch2);

    mpal_set_number(scratch1, MPAL_MAX_LEN, 0);

    mpal_square(Y, len_Y, scratch1);   
    mpal_mod_by_p192(scratch1, len_Y << 1);
#endif

#ifdef _CCH_SC_ECDH_P256_
    if (mpal_compare(scratch1, len_scratch1, scratch2, len_scratch2) == 0)    
#else
    if (mpal_compare(scratch1, MPAL_PRIME_LEN, scratch2, len_scratch2) == 0)    
#endif
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

/**
 * Calculates 2*A where A is a point on the P-192 curve. Co-ordinates are
 * specified in jacobian projective co-ordinate system.
 *
 * \return None.
 *
 * \warning This function does not check whether the given point lies
 *          the elliptic curve - although it does return an infinite point
 *          if the input is an explicit infinite point ie; Z == 0.
 *          This function is not \b reentrant.
 */
void jacobian_point_double(MIXED_SCALAR_MULT_ARGUMENT_S *argu)
{
    u16 new_lenx, new_leny, new_lenz;
    u16 len_scratch1, len_scratch2, len_scratch3;
    u16 len_A, len_B, len_C, len_D;
    UINT8 i;

#ifndef _CCH_SC_ECDH_P256_
    DIGIT_S A[MPAL_MAX_LEN];
    DIGIT_S B[MPAL_MAX_LEN];
    DIGIT_S C[MPAL_MAX_LEN];
    DIGIT_S D[MPAL_MAX_LEN];
    DIGIT_S scratch1[MPAL_MAX_LEN];
    DIGIT_S scratch2[MPAL_MAX_LEN];
    DIGIT_S scratch3[MPAL_MAX_LEN];  
#else
    DIGIT_S A[MPAL_PRIME_LEN_MAX_X2];
    DIGIT_S B[MPAL_PRIME_LEN_MAX_X2];
    DIGIT_S C[MPAL_PRIME_LEN_MAX_X2];
    DIGIT_S D[MPAL_PRIME_LEN_MAX_X2];
    DIGIT_S scratch1[MPAL_PRIME_LEN_MAX_X2];
    DIGIT_S scratch2[MPAL_PRIME_LEN_MAX_X2];
    DIGIT_S scratch3[MPAL_PRIME_LEN_MAX_X2]; 
#endif

    DIGIT_S *X = argu->Qx; /* X - co-ordinate of point on curve - A . 
                              Will be set to the X co-ordinate of the 
                              2*A point by callee */
    DIGIT_S *Y = argu->Qy; /* Y - co-ordinate of point on curve - A . 
                              Will be set to the Y co-ordinate of the 
                              2*A point by callee */    
    DIGIT_S *Z = argu->Qz; /* Z - co-ordinate of point on curve - A . 
                              Will be set to the Z co-ordinate of the 
                              2*A point by callee */

    MPAL_MACRO_DEF;

    mpal_trim(X, argu->len_qx, new_lenx);
    mpal_trim(Y, argu->len_qy, new_leny);
    mpal_trim(Z, argu->len_qz, new_lenz);

    if(new_lenz == 1 && Z[0] == 0)
    {
        mpal_set_number(X, new_lenx, 0);
        mpal_set_number(Y, new_leny, 0x01010101);
        //mpal_set_number(Z, new_lenz, 0); /* redundance code */

        argu->len_qx = 1;
        argu->len_qy = 1;
        argu->len_qz = 1;
        return;
    }

#ifndef _CCH_SC_ECDH_P256_
    /* Clear all scratch buffers */
    for (i = 0; i < MPAL_MAX_LEN; i++)
    {
        A[i] = 0;
        B[i] = 0;
        C[i] = 0;
        D[i] = 0;
        scratch1[i] = 0;
        scratch2[i] = 0;
        scratch3[i] = 0;
    }
#else
    /* Clear all scratch buffers */
    for (i = 0; i < MPAL_PRIME_LEN_MAX_X2; i++)
    {
        A[i] = 0;
        B[i] = 0;
        C[i] = 0;
        D[i] = 0;
        scratch1[i] = 0;
        scratch2[i] = 0;
        scratch3[i] = 0;
    }
#endif


    /* scratch2 = 4Y^2, scratch1=Y^2 */
    mpal_square(Y, new_leny, scratch1); 

#ifndef _CCH_SC_ECDH_P256_
    mpal_mod_by_p192(scratch1, new_leny << 1);
    mpal_mult_by_left_shift(scratch1, MPAL_PRIME_LEN, 2, scratch2);  
    mpal_mod_by_p192(scratch2, MPAL_PRIME_LEN+1);
    mpal_trim(scratch2, MPAL_PRIME_LEN+1, len_scratch2);
#else
    mpal_mod(scratch1, new_leny << 1, (argu->len_prime));
    mpal_mult_by_left_shift(scratch1, (argu->len_prime), 2, scratch2); 
    mpal_mod(scratch2, (argu->len_prime)+1, (argu->len_prime));
    mpal_trim(scratch2, (argu->len_prime)+1, len_scratch2);
#endif

 

    /* A = 4XY^2 */
    mpal_mult(scratch2, len_scratch2, X, new_lenx, A);
#ifndef _CCH_SC_ECDH_P256_
    mpal_mod_by_p192(A, len_scratch2+new_lenx);
    mpal_trim(A, MPAL_PRIME_LEN, len_A);
#else
    mpal_mod(A, len_scratch2+new_lenx, (argu->len_prime));
    mpal_trim(A, (argu->len_prime), len_A);
#endif


    /* B = 8Y^4 */
    mpal_mult_by_left_shift(scratch2, len_scratch2, 1, scratch3);   

#ifndef _CCH_SC_ECDH_P256_ 
    mpal_mod_by_p192(scratch3, len_scratch2 +1 );
    mpal_mult(scratch3, MPAL_PRIME_LEN, scratch1, MPAL_PRIME_LEN, B);
    mpal_mod_by_p192(B, 2*MPAL_PRIME_LEN);
    mpal_trim(B, MPAL_PRIME_LEN, len_B);

    mpal_set_number(scratch1, 2*MPAL_PRIME_LEN, 0);
#else
    mpal_mod(scratch3, len_scratch2 +1, (argu->len_prime));
    mpal_mult(scratch3, (argu->len_prime), scratch1, (argu->len_prime), B);
    mpal_mod(B, 2*(argu->len_prime), (argu->len_prime));
    mpal_trim(B, (argu->len_prime), len_B);
    mpal_set_number(scratch1, MPAL_PRIME_LEN_MAX_X2, 0);
#endif

    /* scratch1 = Z^2 */
    mpal_square(Z, new_lenz, scratch1); 
#ifndef _CCH_SC_ECDH_P256_
    mpal_mod_by_p192(scratch1, new_lenz << 1);
    mpal_trim(scratch1, MPAL_PRIME_LEN, len_scratch1);
#else
    mpal_mod(scratch1, new_lenz << 1, (argu->len_prime));
    mpal_trim(scratch1, (argu->len_prime), len_scratch1);
#endif

    /* scratch2 = X-Z^2 */
#ifndef _CCH_SC_ECDH_P256_
    for (i = 0; i < 2*MPAL_PRIME_LEN; i++)
#else
    for (i = 0; i < MPAL_PRIME_LEN_MAX_X2; i++)
#endif
    {
        scratch2[i] = (i < new_lenx) ? X[i] : 0x00000000;
    }

    if(mpal_compare(scratch2, new_lenx, scratch1, len_scratch1) < 0)
    {
#ifndef _CCH_SC_ECDH_P256_  
        mpal_add(scratch2,  MPAL_MAX_LEN, PRIME, MPAL_PRIME_LEN);
#else
        if(argu->len_prime == 6)
        {
            mpal_add(scratch2,  (argu->len_prime), PRIME, (argu->len_prime));
        }else if(argu->len_prime == 8)
        {
            mpal_add(scratch2,  (argu->len_prime), PRIME_P256, (argu->len_prime));
        }
#endif
    }
#ifndef _CCH_SC_ECDH_P256_  
    mpal_sub(scratch2, MPAL_PRIME_LEN + 1, scratch1, len_scratch1);
    mpal_trim(scratch2, MPAL_PRIME_LEN, len_scratch2);
#else
    mpal_sub(scratch2, (argu->len_prime) + 1, scratch1, len_scratch1);
    mpal_trim(scratch2, (argu->len_prime), len_scratch2);
#endif

    /* scratch3 = X + Z^2 */
#ifndef _CCH_SC_ECDH_P256_  
    for (i = 0; i < 2*MPAL_PRIME_LEN; i++)
#else
    for (i = 0; i < MPAL_PRIME_LEN_MAX_X2; i++)
#endif
    {
        scratch3[i] = (i < new_lenx) ? X[i] : 0x00000000;
    }
    mpal_add(scratch3, len_scratch1 + 1, scratch1, len_scratch1);
#ifndef _CCH_SC_ECDH_P256_
    mpal_mod_by_p192(scratch3, len_scratch1 + 2);
    mpal_trim(scratch3, MPAL_PRIME_LEN, len_scratch3);
#else
    mpal_mod(scratch3, len_scratch1 + 2, (argu->len_prime));
    mpal_trim(scratch3, (argu->len_prime), len_scratch3);
#endif


    /*scratch1 = (X-Z^2)(X+Z^2) */
#ifndef _CCH_SC_ECDH_P256_
    mpal_set_number(scratch1, len_scratch1, 0);
#else
    mpal_set_number(scratch1, MPAL_PRIME_LEN_MAX_X2, 0);
#endif

    mpal_mult(scratch3, len_scratch3, scratch2, len_scratch2, scratch1);

#ifndef _CCH_SC_ECDH_P256_
    mpal_mod_by_p192(scratch1, len_scratch2 + len_scratch3);
#else
    mpal_mod(scratch1, len_scratch2 + len_scratch3, (argu->len_prime));
#endif

    /*  C = 3(X-Z^2)(X+Z^2) */
#ifndef _CCH_SC_ECDH_P256_  
    mpal_mult_by_left_shift(scratch1, MPAL_PRIME_LEN, 1, C); 
    mpal_add(C, MPAL_PRIME_LEN + 1, scratch1, MPAL_PRIME_LEN);    
    mpal_mod_by_p192(C, MPAL_PRIME_LEN+1);
    mpal_trim(C, MPAL_PRIME_LEN, len_C);
#else
    mpal_mult_by_left_shift(scratch1, (argu->len_prime), 1, C); 
    mpal_add(C, (argu->len_prime) + 1, scratch1, (argu->len_prime));   
    mpal_mod(C, (argu->len_prime)+1, (argu->len_prime));
    mpal_trim(C, (argu->len_prime), len_C);
#endif

    mpal_set_number(scratch1, len_scratch2+len_scratch3, 0);
    mpal_set_number(scratch2, len_scratch2, 0);
    //mpal_set_number(scratch3, len_scratch3, 0); /* redundant */

    /* D = 2A + C^2 */
    mpal_square(C, len_C, D); 

#ifndef _CCH_SC_ECDH_P256_
    mpal_mod_by_p192(D, len_C << 1);
#else
    mpal_mod(D, len_C << 1, (argu->len_prime));
#endif

    mpal_trim(D, len_C << 1, len_D);

    if(mpal_compare(D, len_D, A, len_A) < 0)
    {
#ifndef _CCH_SC_ECDH_P256_
        len_D = MPAL_PRIME_LEN + 1;
        mpal_add(D, len_D, PRIME, MPAL_PRIME_LEN);
#else
        len_D = (argu->len_prime) + 1;
        if(argu->len_prime == 6)
        {
            mpal_add(D, len_D, PRIME, (argu->len_prime));
        }else if(argu->len_prime == 8)
        {
            mpal_add(D, len_D, PRIME_P256, (argu->len_prime));
        }
#endif
    }

    mpal_sub(D, len_D, A, len_A);
    mpal_trim(D, len_D, len_D);

    if(mpal_compare(D, len_D, A, len_A) < 0)
    {
#ifndef _CCH_SC_ECDH_P256_
        len_D = MPAL_PRIME_LEN + 1;
        mpal_add(D, len_D, PRIME, MPAL_PRIME_LEN);
#else
        len_D = (argu->len_prime) + 1;
        if(argu->len_prime == 6)
        {
            mpal_add(D, len_D, PRIME, (argu->len_prime));
        }else if(argu->len_prime == 8)
        {
            mpal_add(D, len_D, PRIME_P256, (argu->len_prime));
        }
#endif
    }
    mpal_sub(D, len_D, A, len_A);
    mpal_trim(D, len_D, len_D);

    /*  new Z = 2YZ */
    mpal_mult(Y, new_leny, Z, new_lenz, scratch1);

#ifndef _CCH_SC_ECDH_P256_
    mpal_mod_by_p192(scratch1, new_leny + new_lenz);
    mpal_mult_by_left_shift(scratch1, MPAL_PRIME_LEN, 1, scratch2);   
    mpal_mod_by_p192(scratch2, MPAL_PRIME_LEN + 1);
#else
    mpal_mod(scratch1, new_leny + new_lenz, (argu->len_prime));
    mpal_mult_by_left_shift(scratch1, (argu->len_prime), 1, scratch2);  
    mpal_mod(scratch2, (argu->len_prime)+1, (argu->len_prime));
#endif
 
    mpal_set_number(scratch1, new_leny + new_lenz, 0);
    mpal_set_number(Z, new_lenz, 0);
#ifndef _CCH_SC_ECDH_P256_
    mpal_add(Z, MPAL_PRIME_LEN, scratch2, MPAL_PRIME_LEN);
#else
    mpal_add(Z, (argu->len_prime), scratch2, (argu->len_prime));
#endif
    //mpal_set_number(scratch2, MPAL_PRIME_LEN + 1, 0); /* redundant */
#ifndef _CCH_SC_ECDH_P256_
    mpal_trim(Z, MPAL_PRIME_LEN, argu->len_qz);
#else
    mpal_trim(Z, (argu->len_prime), argu->len_qz);
#endif

    /*  new Y = C.(A-D) - B */
    if(mpal_compare(A, len_A, D, len_D) < 0)
    {
#ifndef _CCH_SC_ECDH_P256_
        mpal_add(A, MPAL_PRIME_LEN + 1, PRIME, MPAL_PRIME_LEN);
        len_A = MPAL_PRIME_LEN + 1;
#else
        if(argu->len_prime == 6)
        {
            mpal_add(A, (argu->len_prime) + 1, PRIME, (argu->len_prime));
        }else if(argu->len_prime == 8)
        {
            mpal_add(A, (argu->len_prime) + 1, PRIME_P256, (argu->len_prime));
        }
        len_A = (argu->len_prime) + 1;
#endif
    }
    mpal_sub(A, len_A, D, len_D);
    mpal_trim(A, len_A, len_A);

    mpal_mult(A, len_A, C, len_C, scratch1);

#ifndef _CCH_SC_ECDH_P256_
    mpal_mod_by_p192(scratch1, len_A + len_C);
#else
    mpal_mod(scratch1, len_A + len_C, (argu->len_prime));
#endif

    mpal_trim(scratch1, len_A + len_C, len_scratch1);

    if(mpal_compare(scratch1, len_scratch1, B, len_B) < 0)
    {
#ifndef _CCH_SC_ECDH_P256_
        mpal_add(scratch1, MPAL_PRIME_LEN + 1, PRIME, MPAL_PRIME_LEN);
        len_scratch1 = MPAL_PRIME_LEN + 1;
#else
        if(argu->len_prime == 6)
        {
            mpal_add(scratch1, (argu->len_prime) + 1, PRIME, (argu->len_prime));
        }else if(argu->len_prime == 8)
        {
            mpal_add(scratch1, (argu->len_prime) + 1, PRIME_P256, (argu->len_prime));
        }
        len_scratch1 = (argu->len_prime) + 1;
#endif
    }
    mpal_sub(scratch1, len_scratch1, B, len_B);    
    mpal_set_number(Y, new_leny, 0);
    mpal_add(Y, len_scratch1, scratch1, len_scratch1);
    mpal_trim(Y, len_scratch1, argu->len_qy);

    /* new X = D */
    mpal_set_number(X, new_lenx, 0);
    mpal_add(X, len_D, D, len_D);
    argu->len_qx = len_D;

    /* scratch1 uncleared . Else all cleared .*/
}

/**
 * Calculates J +K,  where J and K are points on the P-192 curve.
 * Co-ordinates for J are  specified in jacobian projective co-ordinate
 * system, while that for K are in affine co-ord system.
 * 
 * \warning This function does not check whether the given points lies
 *          the elliptic curve . Checks need to be added for cases -
 *          1> One point is at infinity
 *          2> One point is inverse of other.
 *          This function is not \b reentrant.
 */
void mixed_point_addition(MIXED_SCALAR_MULT_ARGUMENT_S *argu)
{
    u16 new_lenxj, new_lenyj, new_lenzj, new_lenxk, new_lenyk;
    u16 len_scratch1, len_scratch2;
    u16 len_A, len_B, len_C, len_D, len_C3, len_XC2;
    MPAL_MACRO_DEF;

#ifndef _CCH_SC_ECDH_P256_	
    DIGIT_S A[MPAL_MAX_LEN];
    DIGIT_S B[MPAL_MAX_LEN];
    DIGIT_S C[MPAL_MAX_LEN];
    DIGIT_S D[MPAL_MAX_LEN];
    DIGIT_S C3[MPAL_MAX_LEN];
    DIGIT_S XC2[MPAL_MAX_LEN];            
    DIGIT_S scratch1[MPAL_MAX_LEN];
    DIGIT_S scratch2[MPAL_MAX_LEN];
#else
    DIGIT_S A[MPAL_PRIME_LEN_MAX_X2];
    DIGIT_S B[MPAL_PRIME_LEN_MAX_X2];
    DIGIT_S C[MPAL_PRIME_LEN_MAX_X2];
    DIGIT_S D[MPAL_PRIME_LEN_MAX_X2];
    DIGIT_S C3[MPAL_PRIME_LEN_MAX_X2];
    DIGIT_S XC2[MPAL_PRIME_LEN_MAX_X2];            
    DIGIT_S scratch1[MPAL_PRIME_LEN_MAX_X2];
    DIGIT_S scratch2[MPAL_PRIME_LEN_MAX_X2];
#endif

    DIGIT_S *XJ = argu->Qx;         /* X - co-ordinate of point on curve - J . 
                                       Will be set to the X co-ordinate of the 
                                       J+K point by callee.*/
    DIGIT_S *YJ = argu->Qy;         /* Y - co-ordinate of point on curve - J . 
                                       Will be set to the Y co-ordinate of the 
                                       J+K point by callee.*/
    DIGIT_S *ZJ = argu->Qz;         /* Z - co-ordinate of point on curve - J . 
                                       Will be set to the Z co-ordinate of the 
                                       J+K point by callee.*/
    DIGIT_S *XK = argu->x;          /* X - co-ordinate of point on curve - K */
    DIGIT_S *YK = argu->y;          /* Y - co-ordinate of point on curve - K */
    UINT16 *len_XJ = &argu->len_qx; /* Length of the input number XJ */
    UINT16 *len_YJ = &argu->len_qy; /* Length of the input number YJ */       
    UINT16 *len_ZJ = &argu->len_qz; /* Length of the input number ZJ */
    UINT16 len_XK = argu->len_x;    /* Length of the input number XK */
    UINT16 len_YK = argu->len_y;    /* Length of the input number YK */
    UINT8 i;    

    /* Clear all scratch buffers */
#ifndef _CCH_SC_ECDH_P256_
    for (i = 0; i < MPAL_MAX_LEN; i++)
#else
    for (i = 0; i < MPAL_PRIME_LEN_MAX_X2; i++)
#endif
    {
        A[i] = 0;
        B[i] = 0;
        C[i] = 0;
        D[i] = 0;
        C3[i] = 0;
        XC2[i] = 0;
        scratch1[i] = 0;
        scratch2[i] = 0;      
    }
    
    mpal_trim(XJ, *len_XJ, new_lenxj);
    mpal_trim(YJ, *len_YJ, new_lenyj);
    mpal_trim(ZJ, *len_ZJ, new_lenzj);

    mpal_trim(XK, len_XK, new_lenxk);
    mpal_trim(YK, len_YK, new_lenyk);

    //Check ? - For infinite pnt ? For sum of inverse point ? .   
    mpal_square(ZJ, new_lenzj, scratch1);
#ifndef _CCH_SC_ECDH_P256_
    mpal_mod_by_p192(scratch1, new_lenzj << 1);
    mpal_trim(scratch1, MPAL_PRIME_LEN, len_scratch1);
#else
    mpal_mod(scratch1, new_lenzj << 1,(argu->len_prime));
    mpal_trim(scratch1, MPAL_PRIME_LEN_MAX_X2, len_scratch1);
#endif

    /* A = Xk * Zj^2 */
    mpal_mult(XK, new_lenxk, scratch1, len_scratch1, A);
#ifndef _CCH_SC_ECDH_P256_
    mpal_mod_by_p192(A, new_lenxk + len_scratch1);
    mpal_trim(A, MPAL_PRIME_LEN, len_A);
#else
    mpal_mod(A, new_lenxk + len_scratch1,(argu->len_prime));
    mpal_trim(A, MPAL_PRIME_LEN_MAX_X2, len_A);
#endif

    /*  B = Yk*Zj^3 */
    mpal_mult(scratch1, len_scratch1, YK, new_lenyk, scratch2);
#ifndef _CCH_SC_ECDH_P256_
    mpal_mod_by_p192(scratch2, len_scratch1 + new_lenyk);
    mpal_mult(scratch2, MPAL_PRIME_LEN, ZJ, new_lenzj, B);
    mpal_mod_by_p192(B, MPAL_PRIME_LEN + new_lenzj);
    mpal_trim(B, MPAL_PRIME_LEN, len_B);
#else
    mpal_mod(scratch2, len_scratch1 + new_lenyk,(argu->len_prime));
    mpal_mult(scratch2, (argu->len_prime), ZJ, new_lenzj, B);
    mpal_mod(B, (argu->len_prime) + new_lenzj,(argu->len_prime));
    mpal_trim(B, MPAL_PRIME_LEN_MAX_X2, len_B);
#endif

    /* C = A - Xj */
    for (i = 0; i < len_A; i++)
    {
        C[i] = A[i];
    }
    len_C = len_A;
    if(mpal_compare(A, len_A, XJ, new_lenxj) < 0)
    {
#ifndef _CCH_SC_ECDH_P256_
        len_C = MPAL_PRIME_LEN + 1;
        mpal_add(C, len_C, PRIME, MPAL_PRIME_LEN);
#else
        len_C = (argu->len_prime) + 1;
        if(argu->len_prime == 6)
        {
            mpal_add(C, len_C, PRIME, (argu->len_prime));
        }else if(argu->len_prime == 8)
        {
            mpal_add(C, len_C, PRIME_P256, (argu->len_prime));
        }
#endif
    }
    mpal_sub(C, len_C, XJ, new_lenxj);
    mpal_trim(C, len_C, len_C);

    /* D = B - Yj */
    for (i = 0; i < len_B; i++)
    {
        D[i] = B[i];
    }
    len_D = len_B;
    if(mpal_compare(D, len_D, YJ, new_lenyj) < 0)
    {
#ifndef _CCH_SC_ECDH_P256_
        len_D = MPAL_PRIME_LEN + 1;
        mpal_add(D, len_D, PRIME, MPAL_PRIME_LEN);
#else
        len_D = (argu->len_prime) + 1;
        if(argu->len_prime == 6)
        {
            mpal_add(D, len_D, PRIME, (argu->len_prime));
        }else if(argu->len_prime == 8)
        {
            mpal_add(D, len_D, PRIME_P256, (argu->len_prime));
        }
#endif
    }
    mpal_sub(D, len_D, YJ, new_lenyj);
    mpal_trim(D, len_D, len_D);

    /*  Xj*C^2 and C^3 */
    mpal_square(C, len_C, scratch1);
#ifndef _CCH_SC_ECDH_P256_
    mpal_mod_by_p192(scratch1, len_C << 1);
#else
    mpal_mod(scratch1, len_C << 1,(argu->len_prime));
#endif
    mpal_trim(scratch1, len_C << 1, len_scratch1);

    mpal_mult(C, len_C, scratch1, len_scratch1, C3);

	
#ifndef _CCH_SC_ECDH_P256_
    mpal_mod_by_p192(C3, len_C + len_scratch1);
#else
    mpal_mod(C3, len_C + len_scratch1,(argu->len_prime));
#endif
	
    mpal_trim(C3, len_C + len_scratch1, len_C3);
    mpal_mult(XJ, new_lenxj, scratch1, len_scratch1, XC2);

	
#ifndef _CCH_SC_ECDH_P256_
    mpal_mod_by_p192(XC2, new_lenxj + len_scratch1);
#else
    mpal_mod(XC2, new_lenxj + len_scratch1,(argu->len_prime));
#endif
	
    mpal_trim(XC2, new_lenxj + len_scratch1, len_XC2);

#ifndef _CCH_SC_ECDH_P256_
    mpal_set_number(scratch1, 2*MPAL_PRIME_LEN, 0);
    mpal_set_number(scratch2, 2*MPAL_PRIME_LEN, 0);
#else
    mpal_set_number(scratch1, MPAL_PRIME_LEN_MAX_X2, 0);
    mpal_set_number(scratch2, MPAL_PRIME_LEN_MAX_X2, 0);
#endif

    /* X3 = D^2 - ( X^3 + 2*XJ*C^2) */
    mpal_square(D, len_D, scratch1);

	
#ifndef _CCH_SC_ECDH_P256_
    mpal_mod_by_p192(scratch1, len_D << 1);
#else
    mpal_mod(scratch1, len_D << 1,(argu->len_prime));
#endif
	
    mpal_trim(scratch1, len_D << 1, len_scratch1);

    mpal_mult_by_left_shift(XC2, len_XC2, 1, scratch2);    

	
#ifndef _CCH_SC_ECDH_P256_
    mpal_mod_by_p192(scratch2, len_XC2 + 1);
    mpal_add(scratch2, MPAL_PRIME_LEN+1, C3, len_C3);
    mpal_mod_by_p192(scratch2, MPAL_PRIME_LEN + 2);
    mpal_trim(scratch2, MPAL_PRIME_LEN + 2, len_scratch2);
#else
    mpal_mod(scratch2, len_XC2 + 1,(argu->len_prime));
    mpal_add(scratch2, (argu->len_prime)+1, C3, len_C3);
    mpal_mod(scratch2, (argu->len_prime) + 2,(argu->len_prime));
    mpal_trim(scratch2, (argu->len_prime) + 2, len_scratch2);
#endif

    if(mpal_compare(scratch1, len_scratch1, scratch2, len_scratch2) < 0)
    {
#ifndef _CCH_SC_ECDH_P256_
        len_scratch1 = MPAL_PRIME_LEN + 1;
        mpal_add(scratch1, len_scratch1, PRIME, MPAL_PRIME_LEN);
#else
        len_scratch1 = (argu->len_prime) + 1;
        if(argu->len_prime == 6)
        {
             mpal_add(scratch1, len_scratch1, PRIME, (argu->len_prime));
        }else if(argu->len_prime == 8)
        {
             mpal_add(scratch1, len_scratch1, PRIME_P256, (argu->len_prime));
        }
#endif
    }
    mpal_sub(scratch1, len_scratch1, scratch2, len_scratch2);
    mpal_trim(scratch1, len_scratch1, len_scratch1);
    /*  scratch1 = X3 */
    mpal_set_number(XJ, new_lenxj, 0);
#ifndef _CCH_SC_ECDH_P256_
    mpal_add(XJ, MPAL_PRIME_LEN, scratch1, len_scratch1);
#else
    mpal_add(XJ, (argu->len_prime), scratch1, len_scratch1);
#endif
    *len_XJ = len_scratch1;

    /*  Y3 = D(XJ*c^2  - X3) - Y1*C^3 */
    if(mpal_compare(XC2, len_XC2, scratch1, len_scratch1) < 0)
    {
#ifndef _CCH_SC_ECDH_P256_
        len_XC2 = MPAL_PRIME_LEN + 1;
        mpal_add(XC2, len_XC2, PRIME, MPAL_PRIME_LEN);
#else
        len_XC2 = (argu->len_prime) + 1;
        if(argu->len_prime == 6)
        {
             mpal_add(XC2, len_XC2, PRIME, (argu->len_prime));
        }else if(argu->len_prime == 8)
        {
             mpal_add(XC2, len_XC2, PRIME_P256, (argu->len_prime));
        }
#endif
    }
    mpal_sub(XC2, len_XC2, scratch1, len_scratch1);
    mpal_trim(XC2, len_XC2, len_XC2);

    /*  Clear */
#ifndef _CCH_SC_ECDH_P256_
    mpal_set_number(scratch1, 2*MPAL_PRIME_LEN, 0);
    mpal_set_number(scratch2, 2*MPAL_PRIME_LEN, 0);
#else
    mpal_set_number(scratch1, MPAL_PRIME_LEN_MAX_X2, 0);
    mpal_set_number(scratch2, MPAL_PRIME_LEN_MAX_X2, 0);
#endif

    mpal_mult(D, len_D, XC2, len_XC2, scratch1);
#ifndef _CCH_SC_ECDH_P256_
    mpal_mod_by_p192(scratch1, len_XC2+len_D);
#else
    mpal_mod(scratch1, len_XC2+len_D,(argu->len_prime));
#endif

    mpal_mult(YJ, new_lenyj, C3, len_C3, scratch2);
	
#ifndef _CCH_SC_ECDH_P256_
    mpal_mod_by_p192(scratch2, len_C3+new_lenyj);
#else
    mpal_mod(scratch2, len_C3+new_lenyj,(argu->len_prime));
#endif

#ifndef _CCH_SC_ECDH_P256_
    len_scratch1 = MPAL_PRIME_LEN;
    if(mpal_compare(scratch1, len_scratch1, scratch2, MPAL_PRIME_LEN) < 0)
    {
        len_scratch1++;
        mpal_add(scratch1, len_scratch1, PRIME, MPAL_PRIME_LEN);
    }
#else
    len_scratch1 = (argu->len_prime);
    if(mpal_compare(scratch1, len_scratch1, scratch2, (argu->len_prime)) < 0)
    {
        len_scratch1++;
        if(argu->len_prime == 6)
        {
             mpal_add(scratch1, len_scratch1, PRIME, (argu->len_prime));
        }else if(argu->len_prime == 8)
        {
             mpal_add(scratch1, len_scratch1, PRIME_P256, (argu->len_prime));
        }
    }
#endif

#ifndef _CCH_SC_ECDH_P256_
    mpal_sub(scratch1, len_scratch1, scratch2, MPAL_PRIME_LEN);
#else
    mpal_sub(scratch1, len_scratch1, scratch2, (argu->len_prime));
#endif
    mpal_trim(scratch1, len_scratch1, len_scratch1);
    mpal_set_number(YJ, new_lenyj, 0);
#ifndef _CCH_SC_ECDH_P256_
    mpal_add(YJ, MPAL_PRIME_LEN, scratch1, len_scratch1);
#else
    mpal_add(YJ, (argu->len_prime), scratch1, len_scratch1);
#endif
    *len_YJ = len_scratch1;

    /*  Z3 = Z1 * C */
    mpal_mult(ZJ, new_lenzj, C, len_C, scratch1);

#ifndef _CCH_SC_ECDH_P256_
    mpal_mod_by_p192(scratch1, len_C + new_lenzj);
#else
    mpal_mod(scratch1, len_C + new_lenzj,(argu->len_prime));
#endif
    mpal_trim(scratch1, len_C + new_lenzj, len_scratch1);

    mpal_set_number(ZJ, new_lenzj, 0);
#ifndef _CCH_SC_ECDH_P256_
    mpal_add(ZJ, MPAL_PRIME_LEN, scratch1, len_scratch1);
#else
    mpal_add(ZJ, (argu->len_prime), scratch1, len_scratch1);
#endif

    *len_ZJ = len_scratch1;
}

#ifdef _SSP_DHKEY_CALCULATE_NEW_FLOW_ 
extern UINT8 g_ssp_ce_index;
#endif

/**
 * Calculates S*A where A is a point on the P-192 curve and S is a scalar.
 * Co-ordinates are  specified in affine co-ordinate system for the point X.
 *
 * \param S    Scalar number - S, to be multiplied to point on curve.
 * \param S_len Length of the input number \a S(in sizeof DIGIT_S).
 *              (MUST be the Prime Length)
 * \param X    X - co-ordinate of point on curve - A . Will be set to
 *           the X co-ordinate of the S*A point by callee.
 * \param X_len Length of the input number \a X(in sizeof DIGIT_S).
 * \param Y   Y - co-ordinate of point on curve - A . Will be set to
 *           the Y co-ordinate of the S*A point by callee.
 * \param Y_len Length of the input number \a Y(in sizeof DIGIT_S).
 * \return None.
 *  
 * \warning This function is not \b reentrant.
 */
void mixed_scalar_multiply_state0(DIGIT_S* S, u16 len_S,
        OUT DIGIT_S* X, OUT u16 *len_X, OUT DIGIT_S* Y, OUT u16 *len_Y, 
        UINT8 *key_x, UINT8 *key_y, UINT8 partition)
{
    MPAL_MACRO_DEF;    
    s16 i, j;
    OS_SIGNAL signal;
    UINT8 index;
    UINT8 end = FALSE;
    MIXED_SCALAR_MULT_ARGUMENT_S *argu;

    index = mpal_manager.index;                
    argu = &mpal_manager.argu[index];
    argu->key_x = key_x;
    argu->key_y = key_y;

#ifdef _SSP_DHKEY_CALCULATE_NEW_FLOW_ 
    argu->ce_index = g_ssp_ce_index;
#endif
        
#ifdef _CCH_SC_ECDH_P256_
    argu->len_prime = len_S;
#endif


    mpal_trim(X, *len_X, argu->len_x);
    mpal_trim(Y, *len_Y, argu->len_y);
    mpal_trim(S, len_S, len_S);    

#ifndef _CCH_SC_ECDH_P256_
    for (i = 0; i < MPAL_MAX_LEN; i++)
    {
        argu->Qx[i] = 0;
        argu->Qy[i] = 0;        
        argu->Qz[i] = 0;
    }

    for (i = 0; i < MPAL_PRIME_LEN; i++)
    {
        argu->x[i] = X[i];
        argu->y[i] = Y[i];
        argu->s[i] = S[i];
    }
#else
    for (i = 0; i < MPAL_PRIME_LEN_MAX_X2; i++)
    {
        argu->Qx[i] = 0;
        argu->Qy[i] = 0;        
        argu->Qz[i] = 0;
    }

    for (i = 0; i < MPAL_PRIME_LEN_MAX; i++)
    {
        argu->x[i] = (i < len_S) ? X[i] : 0;   	
        argu->y[i] = (i < len_S) ? Y[i] : 0;   	
        argu->s[i] = (i < len_S) ? S[i] : 0;   			
    }
#endif

    for (i=(len_S-1);i>=0;i--)
    {
        for (j=(MPAL_BASE-1);j>=0;j--)
        {
            if (argu->s[i] & (1 << j))
            {
                mpal_add(argu->Qx, argu->len_x, argu->x, argu->len_x);
                mpal_add(argu->Qy, argu->len_y, argu->y, argu->len_y);
                argu->Qz[0] = 1;
                argu->len_qx = argu->len_x;
                argu->len_qy = argu->len_y;
                argu->len_qz = 1;  

                if (j == 0)
                {
                    if (i == 0)
                    {
                        end = TRUE;
                    }
                    else
                    {
                        argu->i = i - 1;
                        argu->j = (MPAL_BASE-1);                        
                    }
                }
                else
                {
                    argu->i = i;
                    argu->j = j - 1;
                }

                if (end == FALSE)
                {                    
                    if (partition == TRUE)
                    {
                        /* send the signal to start the calculation of DHKey */
                        signal.type = LMP_CALCULATE_DHKEY_PARTIAL_SIGNAL;
                        signal.param = (OS_ADDRESS)((UINT32)index);
#ifdef _SSP_DHKEY_CALCULATE_NEW_FLOW_
                        OS_SEND_SIGNAL_TO_TASK(lmp_task_handle, signal);
#else                        
                        OS_SEND_SIGNAL_TO_TASK_HEAD(lmp_task_handle, signal);
#endif
                    }
                    else
                    {
                        mixed_scalar_multiply_state1(index, FALSE);
                    }
                    mpal_manager.index++;
                    mpal_manager.index &= (MIXED_SCALAR_MULT_BUF_COUNT_MAX - 1);                    
                }
                else
                {
                    mixed_scalar_multiply_state2(index);
                }
                return;
            }
        }
    }    
}

UINT16 g_max_allow_mixed_scalar_mult_state1_count = 1;
void mixed_scalar_multiply_state1(UINT8 index, UINT8 partition)
{
    MIXED_SCALAR_MULT_ARGUMENT_S *argu;
    argu = &mpal_manager.argu[index];       
    UINT16 count = 0;
 
    do 
    {
        jacobian_point_double(argu);
        if (argu->s[argu->i] & (1 << argu->j))
        {
            mixed_point_addition(argu);
        }

        if (argu->j == 0)
        {
            if (argu->i == 0)
            {
                mixed_scalar_multiply_state2(index);
                return;
            }
            else
            {
                argu->i--;
                argu->j = (MPAL_BASE-1);                        
            }
        }
        else
        {
            argu->j--;
        }

        count++;

        if (partition)
        {
            if (count >= g_max_allow_mixed_scalar_mult_state1_count)
            {            
                break;
            }
        }        
    }
    while (1);
    
    /* send the signal to continue the calculation of DHKey */
    OS_SIGNAL signal;
    signal.type = LMP_CALCULATE_DHKEY_PARTIAL_SIGNAL;
    signal.param = (OS_ADDRESS)((UINT32)index);
#ifdef _SSP_DHKEY_CALCULATE_NEW_FLOW_
    OS_SEND_SIGNAL_TO_TASK(lmp_task_handle, signal);
#else    
    OS_SEND_SIGNAL_TO_TASK_HEAD(lmp_task_handle, signal);  
#endif
}

void mixed_scalar_multiply_state2(UINT8 index)
{
    MPAL_MACRO_DEF;
    MIXED_SCALAR_MULT_ARGUMENT_S *argu;
    union {

#ifndef _CCH_SC_ECDH_P256_
		
        struct {
            DIGIT_S scratch1[MPAL_MAX_LEN];
            DIGIT_S scratch2[MPAL_MAX_LEN];
            DIGIT_S scratch3[MPAL_MAX_LEN];
        };
        UINT32 buf[MPAL_MAX_LEN * 3];
#else
		
        struct {
            DIGIT_S scratch1[MPAL_PRIME_LEN_MAX_X2];
            DIGIT_S scratch2[MPAL_PRIME_LEN_MAX_X2];
            DIGIT_S scratch3[MPAL_PRIME_LEN_MAX_X2];
        };
        UINT32 buf[MPAL_PRIME_LEN_MAX_X2 * 3];
#endif
    } set;    
    u16 len_scratch1,  len_scratch2;
    UINT8 i;
    
    argu = &mpal_manager.argu[index];    

#ifndef _CCH_SC_ECDH_P256_
    for (i = 0; i < (MPAL_MAX_LEN*3); i++)
#else
    for (i = 0; i < (MPAL_PRIME_LEN_MAX_X2*3); i++)
#endif
    {
        set.buf[i] = 0x00000000;
    }

    /*  Convert Q to affine co-ordinates */  
    mpal_square(argu->Qz, argu->len_qz, set.scratch1);

#ifndef _CCH_SC_ECDH_P256_
    mpal_mod_by_p192(set.scratch1, argu->len_qz << 1);
#else
    if(argu->len_prime == 6)
    {
        mpal_mod_by_p192(set.scratch1, argu->len_qz << 1);
    }else if(argu->len_prime == 8)
    {
        mpal_mod_by_p256(set.scratch1, argu->len_qz << 1);
    }
#endif

    mpal_trim(set.scratch1, argu->len_qz << 1, len_scratch1);

    mpal_mult(argu->Qz, argu->len_qz, set.scratch1, len_scratch1, set.scratch2);


#ifndef _CCH_SC_ECDH_P256_
    mpal_mod_by_p192(set.scratch2, argu->len_qz+len_scratch1);
#else
    if(argu->len_prime == 6)
    {
        mpal_mod_by_p192(set.scratch2, argu->len_qz+len_scratch1);
    }else if(argu->len_prime == 8)
    {
        mpal_mod_by_p256(set.scratch2, argu->len_qz+len_scratch1);
    }
#endif
	
    mpal_trim(set.scratch2, argu->len_qz+len_scratch1, len_scratch2);

#ifndef _CCH_SC_ECDH_P256_
    mpal_inv_by_p192(set.scratch1, len_scratch1);
    mpal_trim(set.scratch1, MPAL_PRIME_LEN, len_scratch1);
#else
    mpal_inv(set.scratch1, len_scratch1, argu->len_prime);
    mpal_trim(set.scratch1, MPAL_PRIME_LEN_MAX, len_scratch1);
#endif

    mpal_mult(set.scratch1, len_scratch1, argu->Qx, argu->len_qx, set.scratch3);

#ifndef _CCH_SC_ECDH_P256_
    mpal_mod_by_p192(set.scratch3, argu->len_qx+len_scratch1);
#else
    if(argu->len_prime == 6)
    {
        mpal_mod_by_p192(set.scratch3, argu->len_qx+len_scratch1);
    }else if(argu->len_prime == 8)
    {
        mpal_mod_by_p256(set.scratch3, argu->len_qx+len_scratch1);
    }
#endif


    /*  Set X */
    mpal_set_number(argu->x, argu->len_x, 0);
#ifndef _CCH_SC_ECDH_P256_
    mpal_trim(set.scratch3, MPAL_PRIME_LEN, argu->len_x);
#else
    mpal_trim(set.scratch3, MPAL_PRIME_LEN_MAX, argu->len_x);
#endif
    mpal_add(argu->x, argu->len_x, set.scratch3, argu->len_x);
#ifndef _CCH_SC_ECDH_P256_
    mpal_set_number(set.scratch3, MPAL_MAX_LEN, 0);
#else
    mpal_set_number(set.scratch3, MPAL_PRIME_LEN_MAX_X2, 0);
#endif

#ifndef _CCH_SC_ECDH_P256_
    mpal_inv_by_p192(set.scratch2, len_scratch2);
    mpal_trim(set.scratch2, MPAL_PRIME_LEN, len_scratch2);
#else
    mpal_inv(set.scratch2, len_scratch2, argu->len_prime);
    mpal_trim(set.scratch2, MPAL_PRIME_LEN_MAX, len_scratch2);
#endif
    mpal_mult(set.scratch2, len_scratch2, argu->Qy, argu->len_qy, set.scratch3);

#ifndef _CCH_SC_ECDH_P256_
    mpal_mod_by_p192(set.scratch3, argu->len_qy+len_scratch2);
#else
    if(argu->len_prime == 6)
    {
        mpal_mod_by_p192(set.scratch3, argu->len_qy+len_scratch2);
    }else if(argu->len_prime == 8)
    {
        mpal_mod_by_p256(set.scratch3, argu->len_qy+len_scratch2);
    }
#endif

    /*  Set y */
    mpal_set_number(argu->y, argu->len_y, 0);
#ifndef _CCH_SC_ECDH_P256_
    mpal_trim(set.scratch3, MPAL_PRIME_LEN, argu->len_y);
#else
    mpal_trim(set.scratch3, MPAL_PRIME_LEN_MAX, argu->len_y);
#endif
    mpal_add(argu->y, argu->len_y, set.scratch3, argu->len_y);

    if (argu->key_x != NULL)
    {
#ifndef _CCH_SC_ECDH_P256_
        memcpy_and_swap((u8*)argu->key_x, (u8*)argu->x, 24);
#else
        memcpy_and_swap((u8*)argu->key_x, (u8*)argu->x, (argu->len_prime)<<2);
#endif
    }    

    if (argu->key_y != NULL)
    {
#ifndef _CCH_SC_ECDH_P256_    
        memcpy_and_swap((u8*)argu->key_y, (u8*)argu->y, 24);
#else
        memcpy_and_swap((u8*)argu->key_y, (u8*)argu->y, (argu->len_prime)<<2);
#endif
    }       

#ifdef _SSP_DHKEY_CALCULATE_NEW_FLOW_ 
    LMP_CONNECTION_ENTITY* ce_ptr;    
    ce_ptr = &lmp_connection_entity[argu->ce_index];
    ce_ptr->dhkey_calculating = FALSE;
#endif    
}

