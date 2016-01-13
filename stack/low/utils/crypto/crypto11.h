/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/** 
 * \file crypto11.h
 *  Bluetooth 1.1 crypto interface definition.
 * 
 * \author Santhosh kumar M
 * \date 2007-08-28
 *
 *  Add Bluetooth 4.1 secure connection
 * 
 * \author ChiaChun Hung
 * \date 2012-11-08
 */

#ifndef _CRYPTO11_H_
#define _CRYPTO11_H_

/* ========================= Include File Section ========================= */
#include "btypes.h"

/* ====================== Macro Declaration Section ======================= */


/* ==================== Data Types Declaration Section ==================== */


/* ================ Exported Variables Declaration Section ================ */


/* ============================= API Section ============================== */
void lp_E1(const u8 key[16], const u8 rand[16], const u8 bd_addr[6],
        u8 sres[4], u8 aco[12]);
void lp_E21(const u8 rand[16], const u8 bd_addr[6], u8 key[16]);
void lp_E22(const u8 rand[16], const u8 pin[16], u8 pin_len,
        const u8 bd_addr[6], u8 key[16]);
void lp_E3(const u8 key[16], const u8 rand[16], const u8 aco[12],
        u8 enc_key[16], u8 len);


#ifdef _CCH_SC_ECDH_P256_
void lp_h3(const u8 T[16], const u8 keyID[4],
            const u8 A1[6], const u8 A2[6], const u8 ACO[8], OUT u8 aes_enc_key[16]);

void lp_h4(u8 T[16], u8 keyID[4],
            u8 A1[6], u8 A2[6], OUT u8 dev_auth_key[16]);

void lp_h5(const u8 S[16], const u8 R1_orig[16], const u8 R2_orig[16], 
            OUT u8 SRESmaster[4], OUT u8 SRESslave[4], OUT u8 ACO[8]);
#endif


#endif /* _CRYPTO11_H_ */

