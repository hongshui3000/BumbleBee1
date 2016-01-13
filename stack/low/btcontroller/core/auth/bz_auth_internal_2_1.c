/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file bz_auth_internal_2_1.c
 *  BlueWiz Authentication module internal interface(BT2.1) implementation.
 *  This file implements functions which shall only be used within this
 *  authentication module.
 *
 * \author Santhosh kumar M
 * \date 2007-10-23
 */
/********************************* Logger *************************/ 
enum { __FILE_NUM__= 21 };
/********************************* Logger *************************/

/* ========================= Include File Section ========================= */
#include "mem.h"
#include "lmp.h"
#include "bz_debug.h"
#include "crypto.h"
#include "crypto11.h"
#include "bz_auth_states.h"
#include "bz_auth_extern_accessors.h"
#include "bz_auth_hci.h"
#include "bz_auth_internal_2_1.h"

/* ====================== Macro Declaration Section ======================= */


/* ==================== Structure Declaration Section ===================== */


/* ===================== Variable Declaration Section ===================== */


/* ================== Static Function Prototypes Section ================== */
void bz_auth_initialize_oob_params(BZ_AUTH_LINK_PARAMS* auth);
void bz_auth_initiate_oob_method(UINT16 ce_index, LMP_TRAN_ID tid);
void bz_auth_generate_passkey(OUT UINT32* passkey);
void bz_auth_initiate_passkey_entry_method(UINT16 ce_index,
        LMP_TRAN_ID tid);
void bz_auth_initialize_nc_params(BZ_AUTH_LINK_PARAMS* auth);
void bz_auth_initiate_numeric_comparison_method(UINT16 ce_index,
        LMP_TRAN_ID tid);
void bz_auth_initiate_auth_stage1(UINT16 ce_index, LMP_TRAN_ID tid);
void bz_auth_convert_to_pubkey_edtm_format(
        const BZ_AUTH_PUBLIC_KEY* pubkey, UCHAR buf[48]);

#ifndef _CCH_SC_ECDH_P256_	
void bz_auth_convert_to_pubkey_internal_format(const UCHAR buf[48],
        BZ_AUTH_PUBLIC_KEY* pubkey);
#else
void bz_auth_convert_to_pubkey_edtm_format_p256(
        const BZ_AUTH_PUBLIC_KEY_P256* pubkey, UCHAR buf[BZ_AUTH_PUBLIC_KEY_CO_ORDINATE_SIZE_P256*2]);
void bz_auth_convert_to_pubkey_internal_format(const UCHAR buf[BZ_AUTH_PUBLIC_KEY_CO_ORDINATE_SIZE_MAX*2],
        BZ_AUTH_PUBLIC_KEY_MAX* pubkey, UCHAR len_prime);  
#endif


UINT8 bz_auth_determine_auth_method(
        const BZ_AUTH_IO_CAP_DATA* local, const BZ_AUTH_IO_CAP_DATA* remote,
        OUT UINT8* lk_type);    

/* ===================== Function Definition Section ====================== */
/**
 * Generate and send LMP_io_capability_req PDU to the remote device.
 *
 * \param ce_index ACL connection entity index.
 * \param auth Authentication parameter to be associated with the \a ce_index.
 * \param tid Transaction ID to be used for the PDU.
 *
 * \return None.
 */
void bz_auth_send_io_capability_req_pdu(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth, LMP_TRAN_ID tid)
{
    UCHAR param_list[LMP_IO_CAPABILITY_REQ_LEN];

    param_list[0] = LMP_ESCAPE4_OPCODE;
    param_list[2] = LMP_IO_CAPABILITY_REQ_OPCODE;
    param_list[3] = auth->txn_params.ssp_data.local_io_cap.io_cap;
    param_list[4] = auth->txn_params.ssp_data.local_io_cap.oob_data_present;
    param_list[5] = auth->txn_params.ssp_data.local_io_cap.auth_requirements;
    bz_auth_generate_pdu(ce_index, param_list, LMP_IO_CAPABILITY_REQ_LEN, tid);
}

/**
 * Generate and send LMP_io_capability_res PDU to the remote device.
 *
 * \param ce_index ACL connection entity index.
 * \param auth Authentication parameter to be associated with the \a ce_index.
 * \param tid Transaction ID to be used for the PDU.
 *
 * \return None.
 */
void bz_auth_send_io_capability_res_pdu(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth, LMP_TRAN_ID tid)
{
    UCHAR param_list[LMP_IO_CAPABILITY_RES_LEN];

    param_list[0] = LMP_ESCAPE4_OPCODE;
    param_list[2] = LMP_IO_CAPABILITY_RES_OPCODE;
    param_list[3] = auth->txn_params.ssp_data.local_io_cap.io_cap;
    param_list[4] = auth->txn_params.ssp_data.local_io_cap.oob_data_present;
    param_list[5] = auth->txn_params.ssp_data.local_io_cap.auth_requirements;
    bz_auth_generate_pdu(ce_index, param_list, LMP_IO_CAPABILITY_RES_LEN, tid);
}

/**
 * Generate and send LMP_pause_encryption_req PDU to the remote device.
 *
 * \param ce_index ACL connection entity index.
 * \param auth Authentication parameter to be associated with the \a ce_index.
 * \param tid Transaction ID to be used for the PDU.
 *
 * \return None.
 */
void bz_auth_send_pause_encryption_req_pdu(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth, LMP_TRAN_ID tid)
{
#ifdef _SUPPORT_SECURE_CONNECTION_
#ifdef _SECURE_CONN_TEST_LOG
RT_BT_LOG(YELLOW, DAPE_TEST_LOG522, 2,auth->secure_conn_enabled, auth->len_prime);
#endif

    if ((auth->secure_conn_enabled) 
       && (auth->txn_params.auth_role == BZ_AUTH_ROLE_INITIATOR))
    {
        UCHAR param_list[LMP_PAUSE_ENCRYPTION_AES_REQ_LEN];
        param_list[0] = LMP_PAUSE_ENCRYPTION_AES_REQ_OPCODE;

#ifdef _CCH_SC_ECDH_P256_PAUSE_ENC
        auth->sc_use_enc_rand = 1;

        ssp_rng_get(&param_list[2]);
        memcpy(&auth->enc_rand[0], &param_list[2],
            BZ_AUTH_ENC_RAND_SIZE);	

#ifdef _CCH_SC_TEST_20130129_QCA_03
            bz_auth_convert_to_msb(&auth->enc_rand[0], BZ_AUTH_ENC_RAND_SIZE);
#endif
	
#ifdef _CCH_SC_ECDH_P256_LOG
    RT_BT_LOG(BLUE, YL_DBG_HEX_16, 16, 
    auth->enc_rand[0], auth->enc_rand[1], auth->enc_rand[2], auth->enc_rand[3],
    auth->enc_rand[4], auth->enc_rand[5], auth->enc_rand[6], auth->enc_rand[7],
    auth->enc_rand[8], auth->enc_rand[9], auth->enc_rand[10], auth->enc_rand[11],
    auth->enc_rand[12], auth->enc_rand[13], auth->enc_rand[14], auth->enc_rand[15]);
#endif
#ifdef _DAPE_TEST_UPF45_TRY_TO_FIX_ROLE_SW_FROM_MTOS_FAIL
		
#ifdef _CCH_SC_TEST_20130129_QCA_05
        memcpy(&auth->enc_rand_remote[0], &auth->enc_rand[0],
                   BZ_AUTH_ENC_RAND_SIZE);  

#ifdef _DAPE_TEST_CHK_SC_ROLE_SW
RT_BT_LOG(RED, YL_DBG_HEX_16, 16,
auth->enc_rand_remote[0],
auth->enc_rand_remote[1], 
auth->enc_rand_remote[2],
auth->enc_rand_remote[3],
auth->enc_rand_remote[4],
auth->enc_rand_remote[5],
auth->enc_rand_remote[6],
auth->enc_rand_remote[7],
auth->enc_rand_remote[8],
auth->enc_rand_remote[9], 
auth->enc_rand_remote[10],
auth->enc_rand_remote[11],
auth->enc_rand_remote[12],
auth->enc_rand_remote[13],
auth->enc_rand_remote[14],
auth->enc_rand_remote[15]
);
#endif
#endif
#endif		
#endif	
        bz_auth_generate_pdu(ce_index, param_list, 
                LMP_PAUSE_ENCRYPTION_AES_REQ_LEN, tid);
    }
    else
#endif
    {
        UCHAR param_list[LMP_PAUSE_ENCRYPTION_REQ_LEN];
    param_list[0] = LMP_ESCAPE4_OPCODE;
    param_list[2] = LMP_PAUSE_ENCRYPTION_REQ_OPCODE;
    bz_auth_generate_pdu(ce_index, param_list, 
            LMP_PAUSE_ENCRYPTION_REQ_LEN, tid);
}

}

/**
 * Generate and send LMP_resume_encryption_req PDU to the remote device.
 *
 * \param ce_index ACL connection entity index.
 * \param auth Authentication parameter to be associated with the \a ce_index.
 * \param tid Transaction ID to be used for the PDU.
 *
 * \return None.
 */
void bz_auth_send_resume_encryption_req_pdu(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth, LMP_TRAN_ID tid)
{
    UCHAR param_list[LMP_RESUME_ENCRYPTION_REQ_LEN];

    param_list[0] = LMP_ESCAPE4_OPCODE;
    param_list[2] = LMP_RESUME_ENCRYPTION_REQ_OPCODE;
    bz_auth_generate_pdu(ce_index, param_list,
            LMP_RESUME_ENCRYPTION_REQ_LEN, tid);
}

/**
 * Generate and send LMP_keypress_notification PDU to the remote device.
 *
 * \param ce_index ACL connection entity index.
 * \param notification_type Notification type to used as the PDU parameter.
 * \param tid Transaction ID to be used for the PDU.
 *
 * \return None.
 */
void bz_auth_send_keypress_notification_pdu(UINT16 ce_index,
        UCHAR notification_type, LMP_TRAN_ID tid)
{
    UCHAR param_list[LMP_KEYPRESS_NOTIFICATION_LEN];

    param_list[0] = LMP_ESCAPE4_OPCODE;
    param_list[2] = LMP_KEYPRESS_NOTIFICATION_OPCODE;
    param_list[3] = notification_type;
    bz_auth_generate_pdu(ce_index, param_list,
            LMP_KEYPRESS_NOTIFICATION_LEN, tid);
}

/**
 * Disable Secure Simple Pairing feature bits. It may be called during reset
 * to dynamically disable SSP host support.
 *
 * \param None.
 *
 * \return None.
 */
void bz_auth_reset_ssp_ext_feature_bits(void)
{
    register int i;
    UCHAR* features;

    for (i = 0; i < LMP_MAX_PICONETS_SUPPORTED; i++)
    {
        /* Disable extended features support. This is an ugly hack introduced
         * because of the hacks introduced in the spec itself. We may need to
         * move it appropriate place without breaking the /INTERFACES/.
         * Currently SSP is the only feature which uses EXT_FEATURES page, so
         * we can safely disable it. But in future it may need to be reworked.
         */
#if 0 /* It seems that we don't need to unset this bit */
        features = &bz_auth_get_local_features(i)[0];
        features[7] = (UCHAR)(features[7] & (~EXTENDED_FEATURE));
#endif

        /* Disable SSP HOST support */
        features = &bz_auth_get_local_ext_features(i)[0][0];
        features[0] = (UCHAR)(features[0] &
                (~SECURE_SIMPLE_PAIRING_HOST_SUPPORT));
    }

    return;
}

/**
 * Enables the Secure Simple Pairing related feature bits. It also set the
 * #EXTENDED_FEATURE bit. These bits have to be dynamically set by the
 * controller based on the Host's input (In this case, it is
 * Write_Simple_Pairing_Mode(Enabled) should have been issued).
 *
 * \param None.
 *
 * \return None.
 */
void bz_auth_enable_ssp_ext_feature_bits(void)
{
    register int i;
    UCHAR* features;

    for (i = 0; i < LMP_MAX_PICONETS_SUPPORTED; i++)
    {
        /* Enable extended features support */
        features = &bz_auth_get_local_features(i)[0];
        features[7] = (UCHAR)(features[7] | EXTENDED_FEATURE);

        /* Enable SSP HOST support */
        features = &bz_auth_get_local_ext_features(i)[0][0];
        features[0] = (UCHAR)(features[0] | SECURE_SIMPLE_PAIRING_HOST_SUPPORT);
    }

    return;
}

/**
 * Generates new local OOB data(Simple_Pairing_Hash_C,
 * Simple_Pairing_Randomizer_R).
 *
 * \param None.
 *
 * \return None.
 */
void bz_auth_generate_oob_data(void)
{
    BZ_AUTH_OOB_DATA* local;

    local = &bz_auth_dev_params.last_generated_oob_data;

    ssp_rng_get(&local->R[0]);  /* generate Randomizer R */
    /* Here we generate local Simple_Pairing_Hash_C and so we need to use
     * local public key.
     */
#ifndef _CCH_SC_ECDH_P256_     
    ssp_f1(bz_auth_local_pubkey.x, /* PKa/bx */
            bz_auth_local_pubkey.x, /* PKa/bx */
            &local->R[0], /* Ra/Rb -- Simple_Pairing_Randomizer_R */
            0, /* Z */
            &local->C[0]); /* Ca/Cb -- Simple_Pairing_Hash_C */
#else

    ssp_rng_get(&local->R_P256[0]); 
#ifdef TMP_FIX_OOB
// for 001133553311
local->R[15] = 0xce;
local->R[14] = 0xd5;
local->R[13] = 0x18;
local->R[12] = 0x01;
local->R[11] = 0x09;
local->R[10] = 0x11;
local->R[9] = 0xc1;
local->R[8] = 0x89;
local->R[7] = 0x72;
local->R[6] = 0x37;
local->R[5] = 0xe3;
local->R[4] = 0x63;
local->R[3] = 0xf3;
local->R[2] = 0xe9;
local->R[1] = 0x73;
local->R[0] = 0x16;

local->R_P256[15] = 0x0c;
local->R_P256[14] = 0x95;
local->R_P256[13] = 0x45;
local->R_P256[12] = 0x89;
local->R_P256[11] = 0x9d;
local->R_P256[10] = 0xf4;
local->R_P256[9] = 0x1c;
local->R_P256[8] = 0xf5;
local->R_P256[7] = 0xe7;
local->R_P256[6] = 0x0a;
local->R_P256[5] = 0xae;
local->R_P256[4] = 0x4e;
local->R_P256[3] = 0x95;
local->R_P256[2] = 0xc8;
local->R_P256[1] = 0x77;
local->R_P256[0] = 0xbf;

#if 0 // for c0e04c235678
local->R[15] = 0x2e;
local->R[14] = 0x8e;
local->R[13] = 0x5e;
local->R[12] = 0x2e;
local->R[11] = 0xe4;
local->R[10] = 0x56;
local->R[9] = 0x3d;
local->R[8] = 0x66;
local->R[7] = 0xc3;
local->R[6] = 0x07;
local->R[5] = 0xa3;
local->R[4] = 0x3e;
local->R[3] = 0x2c;
local->R[2] = 0x66;
local->R[1] = 0x50;
local->R[0] = 0x46;

local->R_P256[15] = 0x73;
local->R_P256[14] = 0x5b;
local->R_P256[13] = 0x9c;
local->R_P256[12] = 0xca;
local->R_P256[11] = 0x74;
local->R_P256[10] = 0x1e;
local->R_P256[9] = 0xb1;
local->R_P256[8] = 0x9d;
local->R_P256[7] = 0xbf;
local->R_P256[6] = 0x92;
local->R_P256[5] = 0xda;
local->R_P256[4] = 0x41;
local->R_P256[3] = 0x54;
local->R_P256[2] = 0x65;
local->R_P256[1] = 0x2b;
local->R_P256[0] = 0x28;
#endif
#endif

    ssp_f1(bz_auth_local_pubkey.x, /* PKa/bx */
            bz_auth_local_pubkey.x, /* PKa/bx */
            &local->R[0], /* Ra/Rb -- Simple_Pairing_Randomizer_R */
            0, /* Z */
            &local->C[0], 6); /* Ca/Cb -- Simple_Pairing_Hash_C */


    ssp_f1(bz_auth_local_pubkey_p256.x, /* PKa/bx */
            bz_auth_local_pubkey_p256.x, /* PKa/bx */
            &local->R_P256[0], /* Ra/Rb -- Simple_Pairing_Randomizer_R */
            0, /* Z */
            &local->C_P256[0], 8); /* Ca/Cb -- Simple_Pairing_Hash_C */
#endif

}

/**
 * Verifies whether the received OOB hash value is same as the locally
 * generated hash value.
 *
 * \param auth Authentication parameter containing the required input
 *             parameters like remote device's public key.
 * \param ReceivedC Received hash value from the remote device.
 * \param RandomizerR Received randomizer from the remote device.
 *
 * \return TRUE, if both the received value and the computed value are same.
 *         FALSE, otherwise.
 */
BOOLEAN bz_auth_verify_oob_hash_value(BZ_AUTH_LINK_PARAMS* auth,
        const UCHAR ReceivedC[BZ_AUTH_CONFIRM_VALUE_SIZE],
        const UCHAR RandomizerR[BZ_AUTH_OOB_SECRET_NUMBER_SIZE])
{
    UCHAR ComputedC[BZ_AUTH_CONFIRM_VALUE_SIZE];
    BZ_AUTH_SSP_DATA* ssp_data;

    ssp_data = &auth->txn_params.ssp_data;

    /* Here we generate remote Simple_Pairing_Hash_C and so we need to use
     * remote public key.
     */
#ifndef _CCH_SC_ECDH_P256_     
    ssp_f1(ssp_data->remote_pubkey.x, /* PKa/bx */
            ssp_data->remote_pubkey.x, /* PKa/bx */
            RandomizerR, /* Ra/Rb -- Simple_Pairing_Randomizer_R */
            0, /* Z */
            &ComputedC[0]); /* Ca/Cb -- Simple_Pairing_Hash_C */
#else
    ssp_f1(ssp_data->remote_pubkey.x, /* PKa/bx */
            ssp_data->remote_pubkey.x, /* PKa/bx */
            RandomizerR, /* Ra/Rb -- Simple_Pairing_Randomizer_R */
            0, /* Z */
            &ComputedC[0], auth->len_prime); /* Ca/Cb -- Simple_Pairing_Hash_C */
#endif

    
    /* Is ReceivedC == ComputedC */
    if (memcmp(ReceivedC, ComputedC, BZ_AUTH_CONFIRM_VALUE_SIZE) == 0)
    {
        /* Store the Simple_Pairing_Randomizer_R */
        memcpy(&ssp_data->remote_R[0], RandomizerR,
                BZ_AUTH_OOB_SECRET_NUMBER_SIZE);

#ifdef _CCH_SC_ECDH_P256_
        RT_BT_LOG(WHITE, YL_DBG_HEX_16, 16,
        ReceivedC[0],ReceivedC[1],ReceivedC[2],ReceivedC[3], 
        ReceivedC[12],ReceivedC[13],ReceivedC[14],ReceivedC[15],
        ComputedC[0],ComputedC[1],ComputedC[2],ComputedC[3],
        ComputedC[12],ComputedC[13],ComputedC[14],ComputedC[15]);
#endif
		
        return TRUE;
    }
    else
    {
#ifdef _CCH_SC_ECDH_P256_
        RT_BT_LOG(RED, YL_DBG_HEX_16, 16,
        ReceivedC[0],ReceivedC[1],ReceivedC[2],ReceivedC[3], 
        ReceivedC[12],ReceivedC[13],ReceivedC[14],ReceivedC[15],
        ComputedC[0],ComputedC[1],ComputedC[2],ComputedC[3],
        ComputedC[12],ComputedC[13],ComputedC[14],ComputedC[15]);
#endif
        return FALSE;
    }
}

/**
 * Calculates the link key using the DHKEY and other output parameters from
 * the SSP procedures.
 *
 * \param ce_ptr ACL Connection entity pointer.
 * \param auth Authentication parameters associated with the \a ce_ptr.
 *
 * \return None.
 */
void bz_auth_calculate_ssp_link_key(LMP_CONNECTION_ENTITY* ce_ptr,
        BZ_AUTH_LINK_PARAMS* auth)
{
    UCHAR* Nmaster;
    UCHAR* Nslave;
    UCHAR* BD_ADDR_master;
    UCHAR* BD_ADDR_slave;
    BZ_AUTH_SSP_DATA* ssp_data;
    UCHAR A1[LMP_BD_ADDR_SIZE];
    UCHAR A2[LMP_BD_ADDR_SIZE];

    ssp_data = &auth->txn_params.ssp_data;

    if (bz_auth_is_master(ce_ptr))
    {
        Nmaster = ssp_data->local_N;
        Nslave = ssp_data->remote_N;
        BD_ADDR_master = bz_auth_get_local_bd_addr();
        BD_ADDR_slave = bz_auth_get_remote_bd_addr(ce_ptr);
    }
    else
    {
        Nmaster = ssp_data->remote_N;
        Nslave = ssp_data->local_N;
        BD_ADDR_master = bz_auth_get_remote_bd_addr(ce_ptr);
        BD_ADDR_slave = bz_auth_get_local_bd_addr();
    }
    /* Convert BD_ADDR to MSB format */
    memcpy(&A1[0], BD_ADDR_master, LMP_BD_ADDR_SIZE);
    bz_auth_convert_to_msb(&A1[0], LMP_BD_ADDR_SIZE);
    memcpy(&A2[0], BD_ADDR_slave, LMP_BD_ADDR_SIZE);
    bz_auth_convert_to_msb(&A2[0], LMP_BD_ADDR_SIZE);

    /* Calculate link key */
#ifndef _CCH_SC_ECDH_P256_	
    ssp_f2(ssp_data->dhkey, Nmaster, Nslave, (u8*)"btlk", &A1[0], &A2[0],
            &auth->txn_params.link_key[0]);
#else
    ssp_f2(ssp_data->dhkey, Nmaster, Nslave, (u8*)"btlk", &A1[0], &A2[0],
            &auth->txn_params.link_key[0], auth->len_prime);

#endif


    /* 1.1 Crypto algorithms require it in LSB format */
    bz_auth_convert_to_lsb(&auth->txn_params.link_key[0],
            BZ_AUTH_LINK_KEY_SIZE);
    /* Set link key type */
    auth->txn_params.lk_type = ssp_data->lk_type;
}

/**
 * Verifies whether the received DHKey check value and the locally generated
 * check value or same or not.
 *
 * \param ce_ptr ACL connection entity pointer.
 * \param auth Authentication parameters associated with the \a ce_ptr.
 * \param lmp_pdu_ptr LMP PDU packet containing the received DHKey check value.
 *
 * \return TRUE, if the received DHKey check value and the computed check
 *         value are same. FALSE, otherwise.
 */
BOOLEAN bz_auth_verify_dhkey_check_value(LMP_CONNECTION_ENTITY* ce_ptr,
        BZ_AUTH_LINK_PARAMS* auth, LMP_PDU_PKT* lmp_pdu_ptr)
{
    BZ_AUTH_SSP_DATA* ssp_data;
    UCHAR computed_check_value[BZ_AUTH_CHECK_VALUE_SIZE];
    UCHAR remote_io_cap[3];
    UCHAR A1[LMP_BD_ADDR_SIZE];
    UCHAR A2[LMP_BD_ADDR_SIZE];

    ssp_data = &auth->txn_params.ssp_data;

    bz_auth_convert_to_msb(&lmp_pdu_ptr->payload_content[1],
            BZ_AUTH_CHECK_VALUE_SIZE);
    /* ssp_f3() requires the MSB of the io_cap to be "AuthRequirements"... so
     * we need to do the following conversion.
     */
    remote_io_cap[0] = ssp_data->remote_io_cap.auth_requirements;
    remote_io_cap[1] = ssp_data->remote_io_cap.oob_data_present;
    remote_io_cap[2] = ssp_data->remote_io_cap.io_cap;

    /* Convert BD_ADDR to MSB format */
    memcpy(&A1[0], bz_auth_get_remote_bd_addr(ce_ptr), LMP_BD_ADDR_SIZE);
    bz_auth_convert_to_msb(&A1[0], LMP_BD_ADDR_SIZE);
    memcpy(&A2[0], bz_auth_get_local_bd_addr(), LMP_BD_ADDR_SIZE);
    bz_auth_convert_to_msb(&A2[0], LMP_BD_ADDR_SIZE);
    /* Compute remote DHKey Check value(Ea/Eb) that has to be compared with
     * the check value received from the remote device. Here we use remote
     * device's IO_Cap. We put remote device's values first (in case of N and
     * bd_addr) when both local and remote values are required.
     * NOTE: Here we use local_R (Randomizer/Secret), because the OOB method
     *       authentication implicitly depends on the presence of correct
     *       (C,R) values of the remote device. DHKey check is the only place
     *       where the authenticity can be verified (Check the cryptographic
     *       function "f3" table). We must check whether he really has our
     *       Secret value (R), so we use our R value.
     */
#ifndef _CCH_SC_ECDH_P256_     
    ssp_f3(ssp_data->dhkey, /* computed p192() value */
            ssp_data->remote_N, /* Remote LMP_simple_pairing_number(Na/Nb) */
            ssp_data->local_N, /* Our LMP_simple_pairing_number(Na/Nb) */
            ssp_data->local_R, /* Our Randomizer/Secret value(Ra/Rb) */
            &remote_io_cap[0], /* Remote IO_Cap(IOCapA/IOCapB) */
            &A1[0], /* Remote BD_ADDR (in MSB format) */
            &A2[0], /* Local BD_ADDR (in MSB format) */
            &computed_check_value[0]);
#else
    ssp_f3(ssp_data->dhkey, /* computed p192() value */
            ssp_data->remote_N, /* Remote LMP_simple_pairing_number(Na/Nb) */
            ssp_data->local_N, /* Our LMP_simple_pairing_number(Na/Nb) */
            ssp_data->local_R, /* Our Randomizer/Secret value(Ra/Rb) */
            &remote_io_cap[0], /* Remote IO_Cap(IOCapA/IOCapB) */
            &A1[0], /* Remote BD_ADDR (in MSB format) */
            &A2[0], /* Local BD_ADDR (in MSB format) */
            &computed_check_value[0],auth->len_prime);
#endif
    /* Is ComputedEb/Ea == ReceivedEb/Ea */
    if (memcmp(computed_check_value, &lmp_pdu_ptr->payload_content[1],
                BZ_AUTH_CHECK_VALUE_SIZE) == 0)
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

/**
 * Initiates authentication stage2
 *
 * \param ce_index ACL Connection entity index.
 * \param tid Transaction Id.
 *
 * \return None.
 */
void bz_auth_initiate_auth_stage2(UINT16 ce_index, LMP_TRAN_ID tid)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;

    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;

#ifdef _SSP_DHKEY_CALCULATE_NEW_FLOW_
    if (ce_ptr->dhkey_calculating)
    {
        OS_SIGNAL signal;
        signal.type = LMP_CHK_AUTH_STAGE2_SIGNAL;
        signal.param = (OS_ADDRESS)((UINT32)ce_index);
        signal.ext_param = (OS_ADDRESS)((UINT32)tid);
        OS_SEND_SIGNAL_TO_TASK(lmp_task_handle, signal);
        return;
    }
#endif

    if (auth->txn_params.auth_role != BZ_AUTH_ROLE_INITIATOR)
    {
        bz_auth_transition_to_sub_state(auth, RESP_AWAIT_DHKEY_CHECK);
    }
    else
    {
        /* compute Ea and send LMP_Dhkey_Check(Ea) */
        bz_auth_send_dhkey_check_pdu(ce_index, auth, tid);
        bz_auth_transition_to_sub_state(auth, INTR_AWAIT_DHKEY_RESP);
    }
}

/**
 * Initialize the parameters required for OOB procedure. It has to be done
 * before any transaction is initiated for OOB procedure during authentication
 * stage1.
 *
 * \param auth Authentication parameters containing the OOB structure to be
 *             initialized.
 *
 * \return None.
 */
void bz_auth_initialize_oob_params(BZ_AUTH_LINK_PARAMS* auth)
{
    BZ_AUTH_SSP_DATA* ssp_data;

    ssp_data = &auth->txn_params.ssp_data;

    /* Select Na/Nb */
    ssp_rng_get(&ssp_data->local_N[0]);
    /* Set Ra and Rb */
    if (ssp_data->remote_io_cap.oob_data_present)   /* Our OOB Data valid? */
    {
#ifdef _CCH_SC_ECDH_P256_    
        if(auth->len_prime == 6)
        {
#endif
        memcpy(ssp_data->local_R, bz_auth_dev_params.last_generated_oob_data.R,
                BZ_AUTH_SECRET_NUMBER_SIZE);
#ifdef _CCH_SC_ECDH_P256_  
        }
        else if(auth->len_prime == 8)
        {
            memcpy(ssp_data->local_R, bz_auth_dev_params.last_generated_oob_data.R_P256,
                    BZ_AUTH_SECRET_NUMBER_SIZE);
        }
#endif 

    }
    else
    {
        memset(ssp_data->local_R, 0, BZ_AUTH_SECRET_NUMBER_SIZE);
    }
    memset(ssp_data->remote_R, 0, BZ_AUTH_SECRET_NUMBER_SIZE);
    /* Used as Z parameter to ssp_f1() and it should be 0 for
     * NUMERIC_COMPARISON and OOB method.
     */
    ssp_data->ri = 0;
}

/**
 * Initiates oob method.
 *
 * \param ce_index ACL Connection entity index.
 * \param tid Transaction Id.
 *
 * \return None.
 */
void bz_auth_initiate_oob_method(UINT16 ce_index, LMP_TRAN_ID tid)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;

    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;

    /* Generate Na/Nb and Set Ra and Rb */
    bz_auth_initialize_oob_params(auth);
    /* Assume we don't have OOB Data */
    auth->txn_params.ssp_data.is_oob_check_succeeded = TRUE;
#ifdef _SUPPORT_SECURE_CONNECTION_
    auth->txn_params.ssp_data.is_oob256_check_succeeded = TRUE;
#endif    
    if (auth->txn_params.auth_role != BZ_AUTH_ROLE_INITIATOR)
    {
        if (auth->txn_params.ssp_data.local_io_cap.oob_data_present)
        {
            /* We have the remote OOB Data, so lets get it from the host */
            /* send HCI_Remote_OOB_Data_Request_Event */
            bz_auth_generate_remote_oob_data_request_event(ce_index);
            bz_auth_transition_to_sub_state(auth,
                    OOB_RESP_REMOTE_OOB_DATA_REQUESTED);
        }
        else
        {
            /* Lets wait for the SP_Number from the the initiator */
            bz_auth_transition_to_sub_state(auth, OOB_RESP_AWAIT_SP_NUM);
        }
    }
    else    /* We are initiator */
    {
        if (auth->txn_params.ssp_data.local_io_cap.oob_data_present)
        {
            /* We have the remote OOB Data, so lets get it from the host */
            /* send HCI_Remote_OOB_Data_Request_Event */
            bz_auth_generate_remote_oob_data_request_event(ce_index);
            bz_auth_transition_to_sub_state(auth,
                    OOB_INTR_REMOTE_OOB_DATA_REQUESTED);
        }
        else
        {
            /* We don't have the remote OOB data... we got to blindly
             * believe the remote device.
             */
            /* send LMP_Simple_Pairing_Number(Na) */
            bz_auth_send_simple_pairing_number_pdu(ce_index, auth,
                    SELF_DEV_TID);
            bz_auth_transition_to_sub_state(auth, OOB_INTR_SENT_SP_NUM);
        }
    } /* end if (auth_role != BZ_AUTH_ROLE_INITIATOR) */
}

/**
 * Verifies whether the received confirmation value and the locally computed
 * confirmation value are same or not.
 *
 * \param auth Authentication parameters to be used for the computation.
 * \param lmp_pdu_ptr LMP PDU packet containing the received Nonce value
 *                    received from the remote device.
 *
 * \return TRUE, if both the computed and received confirmation values are
 *         same. FALSE, otherwise.
 */
BOOLEAN bz_auth_verify_confirmation_value(BZ_AUTH_LINK_PARAMS* auth,
        LMP_PDU_PKT* lmp_pdu_ptr)
{
    BZ_AUTH_SSP_DATA* ssp_data;
    UCHAR ComputedC[BZ_AUTH_CONFIRM_VALUE_SIZE];

    BZ_ASSERT(bz_auth_get_pdu_opcode(lmp_pdu_ptr)
            == LMP_SIMPLE_PAIRING_NUMBER_OPCODE,
            "Only LMP_simple_pairing_number(Na/Nb) is expected");

    ssp_data = &auth->txn_params.ssp_data;

    /* Store remote SP_Number */
    memcpy(&ssp_data->remote_N[0], &lmp_pdu_ptr->payload_content[1],
            BZ_AUTH_SP_NUMBER_SIZE);
    bz_auth_convert_to_msb(&ssp_data->remote_N[0], BZ_AUTH_SP_NUMBER_SIZE);

    /* Here we are computing the remote device's confirmation value, so we
     * have to use remote device's pubkey first and then local device's
     * pubkey. We should use remote device's SP_Number not ours. ssp_data->ri
     * should be zero for NUMERIC_COMPARISON.
     */
#ifndef _CCH_SC_ECDH_P256_     
    ssp_f1(ssp_data->remote_pubkey.x, bz_auth_local_pubkey.x,
            ssp_data->remote_N, ssp_data->ri, &ComputedC[0]);
#else
    if(auth->len_prime == 6)
    {
        ssp_f1(ssp_data->remote_pubkey.x, bz_auth_local_pubkey.x,
            ssp_data->remote_N, ssp_data->ri, &ComputedC[0], auth->len_prime);
    }
    else if(auth->len_prime == 8)
    {
        ssp_f1(ssp_data->remote_pubkey.x, bz_auth_local_pubkey_p256.x,
            ssp_data->remote_N, ssp_data->ri, &ComputedC[0], auth->len_prime);
    }

#endif
    /* Is ReceivedC(a/b)i == ComputedC(a/b)i */
    if (memcmp(ssp_data->remote_C, ComputedC, BZ_AUTH_CONFIRM_VALUE_SIZE) == 0)
    {
#ifdef _CCH_SC_ECDH_P256_
        RT_BT_LOG(WHITE, YL_DBG_HEX_11, 11, 0x00, 0xBF, auth->len_prime,
        ssp_data->remote_C[0],ssp_data->remote_C[1],ssp_data->remote_C[2],ssp_data->remote_C[3],
        ComputedC[0],ComputedC[1],ComputedC[2],ComputedC[3]);
#endif   
        return TRUE;
    }
    else
    {
#ifdef _CCH_SC_ECDH_P256_
        RT_BT_LOG(RED, YL_DBG_HEX_11, 11, 0x00, 0xBF, auth->len_prime,
        ssp_data->remote_C[0],ssp_data->remote_C[1],ssp_data->remote_C[2],ssp_data->remote_C[3],
        ComputedC[0],ComputedC[1],ComputedC[2],ComputedC[3]);
#endif	
        return FALSE;
    }
}

/**
 * Generates a random passkey key value to be used for the passkey
 * notification event.
 *
 * \param passkey Pointer to store the generated passkey.
 *
 * \return None.
 */
void bz_auth_generate_passkey(OUT UINT32* passkey)
{
    /* Passkey value should be between "000000" and "999999" (decimal)
     * inclusive. (0x00000000 and 0x000F423F in hex).
     */
    *passkey = lmp_generate_random_number();
    *passkey = (*passkey | (lmp_generate_random_number()<<8));
    *passkey = (*passkey | (lmp_generate_random_number()<<16));
    *passkey = (*passkey & 0x000F3FFF);    /* don't exceed 0x000F3FFF */
}

/**
 * Updates the output parameters of authentication stage1 after the completion
 * of the passkey entry procedure.
 *
 * \param auth Authentication parameters to be updated.
 *
 * \return None.
 */
void bz_auth_update_final_pe_params(BZ_AUTH_LINK_PARAMS* auth)
{
    BZ_AUTH_SSP_DATA* ssp_data;

    ssp_data = &auth->txn_params.ssp_data;

    /* Set Ra
     * PASSKEY_ENTRY: The content is memset to zero and local_R[12..15] is set
     *                to the passkey value with MSB of passkey in R[12]
     */
    memset(ssp_data->local_R, 0, BZ_AUTH_SECRET_NUMBER_SIZE);
    /* MSB of passkey is anyway zero, so we need not set it */
    ssp_data->local_R[13] = (UCHAR)(ssp_data->passkey>>16);
    ssp_data->local_R[14] = (UCHAR)(ssp_data->passkey>>8);
    ssp_data->local_R[15] = (UCHAR)ssp_data->passkey;

    /* Set Rb */
    memcpy(&ssp_data->remote_R[0], ssp_data->local_R,
            BZ_AUTH_SECRET_NUMBER_SIZE);
}

/**
 * Generates Simple_Pairing_Number, Z value and sends the
 * LMP_simple_pairing_confirm PDU.
 *
 * \param ce_index ACL connection entity index.
 * \param auth Authentication parameters associated with \a ce_index.
 * \param tid Transaction ID to be used for sending the PDU.
 *
 * \return None.
 */
void bz_auth_send_ith_pe_confirm(UINT16 ce_index, BZ_AUTH_LINK_PARAMS* auth,
        LMP_TRAN_ID tid)
{
    BZ_AUTH_SSP_DATA* ssp_data;

    ssp_data = &auth->txn_params.ssp_data;

    /* Select Na/Nb */
    ssp_rng_get(&ssp_data->local_N[0]);
    /* Set ri */
    /* if (ith bit of passkey is one) then {ri = 0x81} else {ri = 0x80} */
    ssp_data->ri = (UCHAR)(0x80|((ssp_data->passkey>>ssp_data->i)&0x01));
    /* Compute commitment/confirmation value and send it */
    bz_auth_send_simple_pairing_confirm_pdu(ce_index, auth, ssp_data->ri, tid);
    ssp_data->i ++;
}

/**
 * Initiates passkey entry method.
 *
 * \param ce_index ACL Connection entity index.
 * \param tid Transaction ID.
 *
 * \return None.
 */
void bz_auth_initiate_passkey_entry_method(UINT16 ce_index,
        LMP_TRAN_ID tid)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    BZ_AUTH_SSP_DATA* ssp_data;

    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;
    ssp_data = &auth->txn_params.ssp_data;

    /* Initialize PE params */
    ssp_data->i = 0;

    if (auth->txn_params.auth_role != BZ_AUTH_ROLE_INITIATOR)
    {
        if (ssp_data->local_io_cap.io_cap == KEYBOARD_ONLY)
        {
            /* send HCI_User_Passkey_Request_Event */
            bz_auth_generate_user_passkey_request_event(ce_index);
            bz_auth_transition_to_sub_state(auth, PE_RESP_PASSKEY_REQUESTED);
        }
        else
        {
            /* generate passkey and send HCI_User_Passkey_Notification_Event */
            bz_auth_generate_passkey(&ssp_data->passkey);
            bz_auth_generate_user_passkey_notification_event(ce_index,
                    ssp_data->passkey);
            bz_auth_transition_to_sub_state(auth, PE_RESP_AWAIT_SP_CONFIRM);
        }
    }
    else
    {
        if (ssp_data->local_io_cap.io_cap == KEYBOARD_ONLY)
        {
            /* send HCI_User_Passkey_Request_Event */
            bz_auth_generate_user_passkey_request_event(ce_index);
            bz_auth_transition_to_sub_state(auth, PE_INTR_PASS_KEY_REQUESTED);
        }
        else
        {
            /* generate passkey and send HCI_User_Passkey_Notification_Event */
            bz_auth_generate_passkey(&ssp_data->passkey);
            bz_auth_generate_user_passkey_notification_event(ce_index,
                    ssp_data->passkey);

            /* Generate Nai, Calculate Cai and send
             * LMP_Simple_Pairing_Confirm(Cai) [Initiator - so 'a' is used]
             */
            bz_auth_send_ith_pe_confirm(ce_index, auth, tid);
            bz_auth_transition_to_sub_state(auth, PE_INTR_SENT_SP_CONFIRM);
        }
    } /* end if (auth_role != BZ_AUTH_ROLE_INITIATOR) */
}

/**
 * Initialize the parameters required for NC procedure. It has to be done
 * before any transaction is initiated for NC procedure during authentication
 * stage1.
 *
 * \param auth Authentication parameters containing the NC structure to be
 *             initialized.
 *
 * \return None.
 */
void bz_auth_initialize_nc_params(BZ_AUTH_LINK_PARAMS* auth)
{
    BZ_AUTH_SSP_DATA* ssp_data;

    ssp_data = &auth->txn_params.ssp_data;

    /* Select Na/Nb */
    ssp_rng_get(&ssp_data->local_N[0]);
    /* Set Ra = Rb = 0 */
    memset(ssp_data->local_R, 0, BZ_AUTH_SECRET_NUMBER_SIZE);
    memset(ssp_data->remote_R, 0, BZ_AUTH_SECRET_NUMBER_SIZE);
    /* Used as Z parameter to ssp_f1() and it should be 0 for
     * NUMERIC_COMPARISON and OOB method.
     */
    ssp_data->ri = 0;
}

/**
 * Initiates numeric comparison method.
 *
 * \param ce_index ACL Connection entity index.
 * \param tid Transaction Id.
 *
 * \return None.
 */
void bz_auth_initiate_numeric_comparison_method(UINT16 ce_index,
        LMP_TRAN_ID tid)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;

    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;

    /* Select Na/Nb, Set Ra = Rb = 0 */
    bz_auth_initialize_nc_params(auth);

    if (auth->txn_params.auth_role != BZ_AUTH_ROLE_INITIATOR)
    {
        /* Cb = f1(PKbx, PKax, Nb, 0) and send LMP_simple_pairing_confirm(Cb) */
        bz_auth_send_simple_pairing_confirm_pdu(ce_index, auth, 0, tid);
        bz_auth_transition_to_sub_state(auth, NC_RESP_AWAIT_SP_NUM);
    }
    else
    {
        bz_auth_transition_to_sub_state(auth, NC_INTR_AWAIT_SP_CONFIRM);
    }
}

/**
 * Initiates auth state1 transaction.
 *
 * \param ce_index ACL Connection entity index.
 * \param tid Transaction Id.
 *
 * \return None.
 */
void bz_auth_initiate_auth_stage1(UINT16 ce_index, LMP_TRAN_ID tid)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    OS_SIGNAL signal;

    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;

    /* The control is now getting transferred from EDTM module to the
     * authentication module. We have to reset the auth TID timer to its
     * default timeout value.
     */
    bz_auth_start_tid_timer(ce_index, LMP_AUTH_TID_TIMEOUT);

    /* Log the acquired data so far */

    /* Initiate the calculation of DHKey */
    signal.type = LMP_CALCULATE_DHKEY_SIGNAL;
    signal.param = (OS_ADDRESS)auth;
    OS_SEND_SIGNAL_TO_TASK(lmp_task_handle, signal);

#ifdef _SSP_DHKEY_CALCULATE_NEW_FLOW_
    auth->ce_index = ce_index;
    ce_ptr->dhkey_calculating = TRUE;
#endif

    /* Determine Auth Method to use */
    auth->txn_params.ssp_data.auth_method =
        bz_auth_determine_auth_method(&auth->txn_params.ssp_data.local_io_cap,
                &auth->txn_params.ssp_data.remote_io_cap,
                &auth->txn_params.ssp_data.lk_type);
    /* Is anybody using debug ecdh key pair? */

#ifdef _CCH_SC_ECDH_P256_		
    if( auth->len_prime == 6)
    {
#endif    
    if (bz_auth_dev_params.ssp_debug_mode == 0x1
            || (memcmp(&auth->txn_params.ssp_data.remote_pubkey,
                    &bz_auth_local_debug_pubkey,
                    (2*BZ_AUTH_PUBLIC_KEY_CO_ORDINATE_SIZE)) == 0x0))
    {
        auth->txn_params.ssp_data.lk_type =
            BZ_AUTH_LINK_KEY_TYPE_DEBUG_COMBINATION_KEY;
    }
#ifdef _CCH_SC_ECDH_P256_
    }
    else if( auth->len_prime == 8)
    {
#ifdef _SUPPORT_SECURE_CONNECTION_
        if (auth->txn_params.ssp_data.lk_type == 
            BZ_AUTH_LINK_KEY_TYPE_UNAUTHENTICATED_COMBINATION_KEY)
        {
            auth->txn_params.ssp_data.lk_type = 
            BZ_AUTH_LINK_KEY_TYPE_UNAUTHENTICATED_COMBINATION_KEY_P256;
        }
        if (auth->txn_params.ssp_data.lk_type == 
            BZ_AUTH_LINK_KEY_TYPE_AUTHENTICATED_COMBINATION_KEY)
        {
            auth->txn_params.ssp_data.lk_type = 
            BZ_AUTH_LINK_KEY_TYPE_AUTHENTICATED_COMBINATION_KEY_P256;
        }        
#endif
    
        if (bz_auth_dev_params.ssp_debug_mode == 0x1
            || (memcmp(&auth->txn_params.ssp_data.remote_pubkey,
                    &bz_auth_local_debug_pubkey_p256,
                    (2*BZ_AUTH_PUBLIC_KEY_CO_ORDINATE_SIZE_P256)) == 0x0))
        {
            auth->txn_params.ssp_data.lk_type =
                BZ_AUTH_LINK_KEY_TYPE_DEBUG_COMBINATION_KEY;
        }
    }		
#endif 



    switch (auth->txn_params.ssp_data.auth_method)
    {
        case BZ_AUTH_METHOD_NC:
            bz_auth_initiate_numeric_comparison_method(ce_index, tid);
            break;
        case BZ_AUTH_METHOD_PE:
            bz_auth_initiate_passkey_entry_method(ce_index, tid);
            break;
        case BZ_AUTH_METHOD_OOB:
            bz_auth_initiate_oob_method(ce_index, tid);
            break;
        default:
            break;
    }

#ifdef _CCH_SC_ECDH_P256_LOG

RT_BT_LOG(RED, CCH_DBG_151, 32, 
auth->txn_params.ssp_data.remote_pubkey.x[0],auth->txn_params.ssp_data.remote_pubkey.x[1],auth->txn_params.ssp_data.remote_pubkey.x[2],auth->txn_params.ssp_data.remote_pubkey.x[3],
auth->txn_params.ssp_data.remote_pubkey.x[4],auth->txn_params.ssp_data.remote_pubkey.x[5],auth->txn_params.ssp_data.remote_pubkey.x[6],auth->txn_params.ssp_data.remote_pubkey.x[7],
auth->txn_params.ssp_data.remote_pubkey.x[8],auth->txn_params.ssp_data.remote_pubkey.x[11],auth->txn_params.ssp_data.remote_pubkey.x[10],auth->txn_params.ssp_data.remote_pubkey.x[11],
auth->txn_params.ssp_data.remote_pubkey.x[12],auth->txn_params.ssp_data.remote_pubkey.x[13],auth->txn_params.ssp_data.remote_pubkey.x[14],auth->txn_params.ssp_data.remote_pubkey.x[15],
auth->txn_params.ssp_data.remote_pubkey.x[16],auth->txn_params.ssp_data.remote_pubkey.x[17],auth->txn_params.ssp_data.remote_pubkey.x[18],auth->txn_params.ssp_data.remote_pubkey.x[19],
auth->txn_params.ssp_data.remote_pubkey.x[20],auth->txn_params.ssp_data.remote_pubkey.x[21],auth->txn_params.ssp_data.remote_pubkey.x[22],auth->txn_params.ssp_data.remote_pubkey.x[23],
auth->txn_params.ssp_data.remote_pubkey.x[24],auth->txn_params.ssp_data.remote_pubkey.x[25],auth->txn_params.ssp_data.remote_pubkey.x[26],auth->txn_params.ssp_data.remote_pubkey.x[27],
auth->txn_params.ssp_data.remote_pubkey.x[28],auth->txn_params.ssp_data.remote_pubkey.x[29],auth->txn_params.ssp_data.remote_pubkey.x[30],auth->txn_params.ssp_data.remote_pubkey.x[31]
);
#endif
	
}

/**
 * Convert the public key (\a pubkey) from internal storage format to the
 * format required by the EDTM module for transmission.
 *
 * \param pubkey Public key to be converted.
 * \param buf The output public key in the format expected by the EDTM module.
 *
 * \return None.
 */
void bz_auth_convert_to_pubkey_edtm_format(
        const BZ_AUTH_PUBLIC_KEY* pubkey, UCHAR buf[48])
{
    register int i;
    const UCHAR* coord;

    coord = &pubkey->x[23]; /* address of LSB X-coordinate */
    for (i = 0; i < 24; i++, coord--)
    {
        buf[i] = *coord;
    }
    coord = &pubkey->y[23]; /* address of LSB Y-coordinate */
    for (i = 24; i < 48; i++, coord--)
    {
        buf[i] = *coord;
    }
}
#ifdef _CCH_SC_ECDH_P256_	
void bz_auth_convert_to_pubkey_edtm_format_p256(
        const BZ_AUTH_PUBLIC_KEY_P256* pubkey, UCHAR buf[BZ_AUTH_PUBLIC_KEY_CO_ORDINATE_SIZE_P256*2])
{
    register int i;
    const UCHAR* coord;

    coord = &pubkey->x[31]; /* address of LSB X-coordinate */
    for (i = 0; i < 32; i++, coord--)
    {
        buf[i] = *coord;
    }
    coord = &pubkey->y[31]; /* address of LSB Y-coordinate */
    for (i = 32; i < 64; i++, coord--)
    {
        buf[i] = *coord;
    }
}
#endif
/**
 * Converts the given public key (\a buf) from EDTM format to the internal
 * storage format.
 *
 * \param buf Public key in the EDTM format.
 * \param pubkey The output public key in the internal storage format.
 *
 * \return None.
 */
#ifndef _CCH_SC_ECDH_P256_	 
void bz_auth_convert_to_pubkey_internal_format(const UCHAR buf[48],
        BZ_AUTH_PUBLIC_KEY* pubkey)
{
    register int i;
    UCHAR* coord;

    coord = &pubkey->x[23]; /* address of LSB X-coordinate */
    for (i = 0; i < 24; i++, coord--)
    {
        *coord = buf[i];
    }
    coord = &pubkey->y[23]; /* address of LSB Y-coordinate */
    for (i = 24; i < 48; i++, coord--)
    {
        *coord = buf[i];
    }
}
#else
void bz_auth_convert_to_pubkey_internal_format(const UCHAR buf[BZ_AUTH_PUBLIC_KEY_CO_ORDINATE_SIZE_MAX*2],
        BZ_AUTH_PUBLIC_KEY_MAX* pubkey, UCHAR len_prime)
{
    register int i;
    UCHAR* coord;

    coord = &pubkey->x[(len_prime*4) - 1]; /* address of LSB X-coordinate */
    for (i = 0; i < (len_prime*4); i++, coord--)
    {
        *coord = buf[i];
    }
    coord = &pubkey->y[(len_prime*4) - 1]; /* address of LSB Y-coordinate */
    for (i = (len_prime*4); i < (len_prime*8); i++, coord--)
    {
        *coord = buf[i];
    }
}
#endif
/**
 * Handles the completion of sending public key encapsulated pdu's.
 *
 * \param ce_index ACL Connection entity index.
 * \param status completiion status.
 * \param reason reason for failure.
 * \param user_data contains the registered user data.
 *
 * \return None.
 */
void bz_auth_handle_public_key_sent_callback(UINT16 ce_index,
        UINT16 status, UCHAR reason, void* user_data)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    LMP_ENCAPSULATED_DATA* encap_data;

    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;
    encap_data = &auth->txn_params.encap_data;

    if (auth->sub_state != PUBLIC_KEY_EXCHANGE_SENDING_ENCAPSULATED_PDU)
    {
        BZ_ASSERT(0, "We are not expecting public_key_sent_cb at the moment");
        return;
    }
    if (status != HCI_COMMAND_SUCCEEDED)
    {
        bz_auth_handle_simple_pairing_complete(ce_index, (UCHAR)status);
        return;
    }

    if (auth->txn_params.auth_role != BZ_AUTH_ROLE_INITIATOR)
    {
        /* we have received remote public key and sent our public key. */
        /* Initiate auth Stage1 */
        bz_auth_initiate_auth_stage1(ce_index, (LMP_TRAN_ID) encap_data->tid);
        return;
    }
    /* we have sent our public key, now we need to wait for public key from
     * the remote device (We must be initiator here).
     */
    /* use the same tid, major_type, minor_type, payload buf, payload_length */
    lmp_edtm_receive_data(ce_index, encap_data,
            bz_auth_handle_public_key_received_callback, encap_data);
#ifndef _SUPPORT_SECURE_CONNECTION_
#ifdef _CCH_SC_ECDH_P256_
RT_BT_LOG(YELLOW, DAPE_TEST_LOG522, 2,0, auth->len_prime);
#endif
#else
#ifdef _SECURE_CONN_TEST_LOG
RT_BT_LOG(YELLOW, DAPE_TEST_LOG522, 2,auth->secure_conn_enabled, auth->len_prime);
#endif
    if (auth->secure_conn_enabled)
    {
        /* timeout value for 5 pdu's */
        bz_auth_start_tid_timer(ce_index, LMP_AUTH_TID_TIMEOUT * 5);
    }
    else
#endif
    {
        /* timeout value for 4 pdu's */
        bz_auth_start_tid_timer(ce_index, LMP_AUTH_TID_TIMEOUT * 4);
    }
    bz_auth_transition_to_sub_state(auth,
            PUBLIC_KEY_EXCHANGE_RECEIVING_ENCAPSULATED_PDU);
    return;
}

/**
 * Handles the completion of receiving public key encapsulated pdu's.
 *
 * \param ce_index ACL Connection entity index.
 * \param status completiion status.
 * \param reason reason for failure.
 * \param user_data contains the registered user data.
 *
 * \return None.
 */
void bz_auth_handle_public_key_received_callback(UINT16 ce_index,
        UINT16 status, UCHAR reason, void* user_data)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    LMP_ENCAPSULATED_DATA* encap_data;

    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;
    encap_data = &auth->txn_params.encap_data;

    if (auth->sub_state != PUBLIC_KEY_EXCHANGE_RECEIVING_ENCAPSULATED_PDU)
    {
        BZ_ASSERT(0, "We are not expecting public_key_recvd_cb at the moment");
        return;
    }
    if (status != HCI_COMMAND_SUCCEEDED)
    {
        bz_auth_handle_simple_pairing_complete(ce_index, (UCHAR)status);
        return;
    }

    /* We have received the remote public key in EDTM format. Lets convert it
     * into internal representation.
     */
#ifndef _CCH_SC_ECDH_P256_  	
    bz_auth_convert_to_pubkey_internal_format(encap_data->payload,
            &auth->txn_params.ssp_data.remote_pubkey);
#else
    bz_auth_convert_to_pubkey_internal_format(encap_data->payload,
            &auth->txn_params.ssp_data.remote_pubkey, auth->len_prime);

#endif

    if (auth->txn_params.auth_role == BZ_AUTH_ROLE_INITIATOR)
    {
        /* we have sent our public key and received remote public key */
        /* Initiate auth Stage1 */
        bz_auth_initiate_auth_stage1(ce_index, (LMP_TRAN_ID)encap_data->tid);
        return;
    }
    /* we have received remote public key, now we need to send
     * our public key (We must be responder here).
     */
    /* use the same tid, major_type, minor_type, payload_length */
    /* Convert the local public key to EDTM representation (LSB format: X, Y
     * coordinates) and send it.
     */
#ifndef _CCH_SC_ECDH_P256_   
    bz_auth_convert_to_pubkey_edtm_format(&bz_auth_local_pubkey,
            &encap_data->payload[0]);
#else
    if(auth->len_prime == 6)
    {
    bz_auth_convert_to_pubkey_edtm_format(&bz_auth_local_pubkey,
            &encap_data->payload[0]);
    }
    else if(auth->len_prime == 8)
    {
        bz_auth_convert_to_pubkey_edtm_format_p256(&bz_auth_local_pubkey_p256,
            &encap_data->payload[0]);
    }
#endif

    lmp_edtm_send_data(ce_index, encap_data,
            bz_auth_handle_public_key_sent_callback, encap_data);
#ifndef _SUPPORT_SECURE_CONNECTION_
#ifdef _CCH_SC_ECDH_P256_
RT_BT_LOG(YELLOW, DAPE_TEST_LOG522, 2,0, auth->len_prime);
#endif
#else
#if 0
#ifdef _SECURE_CONN_TEST_LOG
RT_BT_LOG(YELLOW, DAPE_TEST_LOG522, 2,auth->secure_conn_enabled, auth->len_prime);
#else
RT_BT_LOG(YELLOW, DAPE_TEST_LOG522, 2,0, auth->len_prime);
#endif
#endif
    if (auth->secure_conn_enabled)
    {
        /* timeout value for 5 pdu's */
        bz_auth_start_tid_timer(ce_index, LMP_AUTH_TID_TIMEOUT * 5);
    }
    else
#endif
    {
        /* timeout value for 4 pdu's */
        bz_auth_start_tid_timer(ce_index, LMP_AUTH_TID_TIMEOUT * 4);
    }
    bz_auth_transition_to_sub_state(auth,
            PUBLIC_KEY_EXCHANGE_SENDING_ENCAPSULATED_PDU);

#ifdef _CCH_SC_ECDH_P256_LOG
RT_BT_LOG(RED, CCH_DBG_151, 32, 
auth->txn_params.ssp_data.remote_pubkey.x[0],auth->txn_params.ssp_data.remote_pubkey.x[1],auth->txn_params.ssp_data.remote_pubkey.x[2],auth->txn_params.ssp_data.remote_pubkey.x[3],
auth->txn_params.ssp_data.remote_pubkey.x[4],auth->txn_params.ssp_data.remote_pubkey.x[5],auth->txn_params.ssp_data.remote_pubkey.x[6],auth->txn_params.ssp_data.remote_pubkey.x[7],
auth->txn_params.ssp_data.remote_pubkey.x[8],auth->txn_params.ssp_data.remote_pubkey.x[11],auth->txn_params.ssp_data.remote_pubkey.x[10],auth->txn_params.ssp_data.remote_pubkey.x[11],
auth->txn_params.ssp_data.remote_pubkey.x[12],auth->txn_params.ssp_data.remote_pubkey.x[13],auth->txn_params.ssp_data.remote_pubkey.x[14],auth->txn_params.ssp_data.remote_pubkey.x[15],
auth->txn_params.ssp_data.remote_pubkey.x[16],auth->txn_params.ssp_data.remote_pubkey.x[17],auth->txn_params.ssp_data.remote_pubkey.x[18],auth->txn_params.ssp_data.remote_pubkey.x[19],
auth->txn_params.ssp_data.remote_pubkey.x[20],auth->txn_params.ssp_data.remote_pubkey.x[21],auth->txn_params.ssp_data.remote_pubkey.x[22],auth->txn_params.ssp_data.remote_pubkey.x[23],
auth->txn_params.ssp_data.remote_pubkey.x[24],auth->txn_params.ssp_data.remote_pubkey.x[25],auth->txn_params.ssp_data.remote_pubkey.x[26],auth->txn_params.ssp_data.remote_pubkey.x[27],
auth->txn_params.ssp_data.remote_pubkey.x[28],auth->txn_params.ssp_data.remote_pubkey.x[29],auth->txn_params.ssp_data.remote_pubkey.x[30],auth->txn_params.ssp_data.remote_pubkey.x[31]
);
#endif

	
    return;
}

/**
 * Initiates public key exchange transaction.
 *
 * \param ce_index ACL Connection entity index.
 * \param tid Transaction Id.
 *
 * \return None.
 */
void bz_auth_initiate_public_key_exchange(UINT16 ce_index, LMP_TRAN_ID tid)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    LMP_ENCAPSULATED_DATA* encap_data;

    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;
    encap_data = &auth->txn_params.encap_data;
    /* Log the acquired data so far */

    /* Setup common input parameters */
    encap_data->tid = bz_auth_get_abs_tid(ce_index, tid);
    encap_data->major_type = 0x1;
    encap_data->minor_type = 0x1;
#ifdef _CCH_SC_TEST_20130129_QCA
    if(auth->secure_conn_enabled)
    {
        encap_data->minor_type = 0x2;
    }
#endif
	

    encap_data->payload = auth->txn_params.ssp_data.edtm_buf;
#ifndef _SUPPORT_SECURE_CONNECTION_
#ifdef _CCH_SC_ECDH_P256_
RT_BT_LOG(YELLOW, DAPE_TEST_LOG522, 2,0, auth->len_prime);
#endif
#else
#if 0
#ifdef _SECURE_CONN_TEST_LOG
RT_BT_LOG(YELLOW, DAPE_TEST_LOG522, 2,auth->secure_conn_enabled, auth->len_prime);
#else
RT_BT_LOG(YELLOW, DAPE_TEST_LOG522, 2,0, auth->len_prime);
#endif
#endif
    if (auth->secure_conn_enabled)
    {
        encap_data->payload_length = 2 * BZ_AUTH_SECURE_CONN_PUBLIC_KEY_CO_ORDINATE_SIZE;
        /* timeout value for 5 pdu's */
        bz_auth_start_tid_timer(ce_index, LMP_AUTH_TID_TIMEOUT * 5);
    }
    else
#endif
    {
        encap_data->payload_length = 2 * BZ_AUTH_PUBLIC_KEY_CO_ORDINATE_SIZE;
    /* timeout value for 4 pdu's */
    bz_auth_start_tid_timer(ce_index, LMP_AUTH_TID_TIMEOUT * 4);
    }
    if (auth->txn_params.auth_role != BZ_AUTH_ROLE_INITIATOR)
    {
        lmp_edtm_receive_data(ce_index, encap_data,
                bz_auth_handle_public_key_received_callback, encap_data);
        bz_auth_transition_to_sub_state(auth,
                PUBLIC_KEY_EXCHANGE_RECEIVING_ENCAPSULATED_PDU);
        return;
    }
    /* Convert the local public key to EDTM representation (LSB format: X, Y
     * coordinates) and send it.
     */

#ifndef _CCH_SC_ECDH_P256_ 
    bz_auth_convert_to_pubkey_edtm_format(&bz_auth_local_pubkey,
            &encap_data->payload[0]);
#else
    if(auth->len_prime == 6)
    {
    bz_auth_convert_to_pubkey_edtm_format(&bz_auth_local_pubkey,
            &encap_data->payload[0]);
    }
    else if(auth->len_prime == 8)
    {
        bz_auth_convert_to_pubkey_edtm_format_p256(&bz_auth_local_pubkey_p256,
            &encap_data->payload[0]);
    }
#endif


    lmp_edtm_send_data(ce_index, encap_data,
            bz_auth_handle_public_key_sent_callback, encap_data);
    bz_auth_transition_to_sub_state(auth,
            PUBLIC_KEY_EXCHANGE_SENDING_ENCAPSULATED_PDU);
    return;
}

/**
 * Initiates the pause encryption action. Upon completion of the pause
 * encryption procedure, the pause_encryption_callback will be invoked.
 *
 * \param ce_index ACL connection entity index.
 * \param auth Authentication parameters associated with the \a ce_index.
 *
 * \return None.
 */
void bz_auth_perform_pause_encryption_action(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth)
{
    LMP_CONNECTION_ENTITY* ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    BZ_ASSERT(auth->txn_params.auth_role == BZ_AUTH_ROLE_INITIATOR,
            "Only initiator can call this function");

    /* send LMP_pause_encryption_req(SELF_TID) */
    bz_auth_send_pause_encryption_req_pdu(ce_index, auth, SELF_DEV_TID);

    /* transition to appropriate sub_state */
    if (bz_auth_is_master(ce_ptr))
    {
        bz_auth_transition_to_sub_state(auth, AWAIT_PAUSE_ENCRYPTION);
    }
    else
    {
        bz_auth_transition_to_sub_state(auth, AWAIT_STOP_ENCRYPTION);
    }
}

/**
 * Initiates the resume encryption action. Upon completion of the resume
 * encryption procedure, the resume_encryption_callback will be invoked.
 *
 * \param ce_index ACL connection entity index.
 * \param auth Authentication parameters associated with the \a ce_index.
 *
 * \return None.
 */
void bz_auth_perform_resume_encryption_action(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth)
{
    LMP_CONNECTION_ENTITY* ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    BZ_ASSERT(auth->txn_params.auth_role == BZ_AUTH_ROLE_INITIATOR,
            "Only initiator can call this function");

    /* do action and transition to appropriate sub_state */
    if (bz_auth_is_master(ce_ptr))
    {
        /* send LMP_start_encryption_req(SELF_TID) */
        bz_auth_send_start_encryption_pdu(ce_index, auth, SELF_DEV_TID);
        bz_auth_transition_to_sub_state(auth, START_ENCRYPTION_SENT);

#ifndef _CCH_SC_ECDH_P256_START_ENC
#ifdef _CCH_SC_ECDH_P256_PAUSE_ENC
        if(auth->secure_conn_enabled)
        {
            BB_write_sc_rxpkcnt_ignore_seqcompare(ce_index, 1);
        }
#endif
#endif		
    }
    else
    {
        /* send LMP_resume_encryption_req(SELF_TID) */
        bz_auth_send_resume_encryption_req_pdu(ce_index, auth, SELF_DEV_TID);
        bz_auth_transition_to_sub_state(auth, AWAIT_START_ENCRYPTION);
    }
}

/**
 * Extracts the IO Capability information from the \a input stream and puts it
 * into \a io_cap structure.
 *
 * \param input Raw IO Cap data (input[0] -> IO_Cap, input[1] ->
 *              OOB_Data_Present/OOB_Auth_Data, input[2] -> Auth_Requirements).
 * \param io_cap Extracted output.
 *
 * \return None.
 */
void bz_auth_extract_io_cap_data(const UCHAR input[3],
        OUT BZ_AUTH_IO_CAP_DATA* io_cap)
{
    io_cap->io_cap = input[0];
    io_cap->oob_data_present = input[1];
    io_cap->auth_requirements = input[2];
}

/**
 * Determines the SSP authentication method to be used from the IO Capabilities
 * of the local and the remote device. It also returns the link key type that
 * would be generated using the selected authentication method.
 *
 * \param local Local device IO capability.
 * \param remote Remote device IO capability.
 * \param lk_type Link key type that would be generated using the selected
 *                authentication method.
 *
 * \return SSP Authentication Method to be used.
 */
UINT8 bz_auth_determine_auth_method(
        const BZ_AUTH_IO_CAP_DATA* local, const BZ_AUTH_IO_CAP_DATA* remote,
        OUT UINT8* lk_type)
{
    *lk_type = BZ_AUTH_LINK_KEY_TYPE_AUTHENTICATED_COMBINATION_KEY;

    if (local->oob_data_present || remote->oob_data_present)
    {
        return BZ_AUTH_METHOD_OOB;
    }

    /* if (Auth_reqA == MITM_Not_Req && Auth_reqB == MITM_Not_Req) */
    if ((local->auth_requirements == NO_MITM_NO_BONDING
                || local->auth_requirements == NO_MITM_DEDICATED_BONDING
                || local->auth_requirements == NO_MITM_GENERAL_BONDING)
            && (remote->auth_requirements == NO_MITM_NO_BONDING
                || remote->auth_requirements == NO_MITM_DEDICATED_BONDING
                || remote->auth_requirements == NO_MITM_GENERAL_BONDING))
    {
        *lk_type = BZ_AUTH_LINK_KEY_TYPE_UNAUTHENTICATED_COMBINATION_KEY;
        return BZ_AUTH_METHOD_NC;
    }

    /* if ((IOCapA == Keyboard_only || IOCapB == Keyboard_only)
                && (IOCapA != NoInpOut && IOCapB != NoInpOut)) */
    if ((local->io_cap == KEYBOARD_ONLY
                || remote->io_cap == KEYBOARD_ONLY)
            && (local->io_cap != NO_INPUT_NO_OUTPUT
                && remote->io_cap != NO_INPUT_NO_OUTPUT))
    {
        return BZ_AUTH_METHOD_PE;
    }
    else
    {
        /* Numeric comparison can yield AUTHENTICATED_LINK_KEY only when
         * both devices have DISPLAY_YES_NO IO capability.
         */
        if (local->io_cap != DISPLAY_YES_NO
                || remote->io_cap != DISPLAY_YES_NO)
        {
            *lk_type = BZ_AUTH_LINK_KEY_TYPE_UNAUTHENTICATED_COMBINATION_KEY;
        }
        return BZ_AUTH_METHOD_NC;
    }
}

/**
 * Generate and send LMP_dhkey_check PDU to the remote device.
 *
 * \param ce_index ACL connection entity index.
 * \param auth Authentication parameter to be associated with the \a ce_index.
 * \param tid Transaction ID to be used for the PDU.
 *
 * \return None.
 */
void bz_auth_send_dhkey_check_pdu(UINT16 ce_index, BZ_AUTH_LINK_PARAMS* auth,
        LMP_TRAN_ID tid)
{
    BZ_AUTH_SSP_DATA* ssp_data;
    UCHAR param_list[LMP_DHKEY_CHECK_LEN];
    UCHAR local_io_cap[3];
    UCHAR A1[LMP_BD_ADDR_SIZE];
    UCHAR A2[LMP_BD_ADDR_SIZE];

    ssp_data = &auth->txn_params.ssp_data;

    /* ssp_f3() requires the MSB of the io_cap to be "AuthRequirements"... so
     * we need to do the following conversion.
     */
    local_io_cap[0] = ssp_data->local_io_cap.auth_requirements;
    local_io_cap[1] = ssp_data->local_io_cap.oob_data_present;
    local_io_cap[2] = ssp_data->local_io_cap.io_cap;

    /* Convert BD_ADDR to MSB format */
    memcpy(&A1[0], bz_auth_get_local_bd_addr(), LMP_BD_ADDR_SIZE);
    bz_auth_convert_to_msb(&A1[0], LMP_BD_ADDR_SIZE);
    memcpy(&A2[0], bz_auth_get_remote_bd_addr(&lmp_connection_entity[ce_index]),
            LMP_BD_ADDR_SIZE);
    bz_auth_convert_to_msb(&A2[0], LMP_BD_ADDR_SIZE);
    /* Compute our DHKey Check value(Ea/Eb) that has to be sent to remote
     * device. Here we use our own IO_Cap. We put our own values first
     * (in case of N and bd_addr) when both local and remote values are
     * required.
     * NOTE: Here we use remote_R (Randomizer/Secret), because the OOB method
     *       authentication implicitly depends on the presence of correct
     *       (C,R) values of the remote device. DHKey check is the only place
     *       where the authenticity can be verified (Check the cryptographic
     *       function "f3" table). We must use remote Secret value to make him
     *       believe we are authentic.
     */

#ifndef _CCH_SC_ECDH_P256_	
    ssp_f3(ssp_data->dhkey, /* computed p192() value */
            ssp_data->local_N, /* Our LMP_simple_pairing_number(Na/Nb) */
            ssp_data->remote_N, /* Remote LMP_simple_pairing_number(Na/Nb) */
            ssp_data->remote_R, /* Remote Randomizer/Secret value(Ra/Rb) */
            &local_io_cap[0], /* Our IO_Caps(IOCapA/IOCapB) */
            &A1[0], /* Local BD_ADDR (in MSB format) */
            &A2[0], /* Remote BD_ADDR (in MSB format) */
            &param_list[2]);
#else
    ssp_f3(ssp_data->dhkey, /* computed p192() value */
            ssp_data->local_N, /* Our LMP_simple_pairing_number(Na/Nb) */
            ssp_data->remote_N, /* Remote LMP_simple_pairing_number(Na/Nb) */
            ssp_data->remote_R, /* Remote Randomizer/Secret value(Ra/Rb) */
            &local_io_cap[0], /* Our IO_Caps(IOCapA/IOCapB) */
            &A1[0], /* Local BD_ADDR (in MSB format) */
            &A2[0], /* Remote BD_ADDR (in MSB format) */
            &param_list[2], auth->len_prime);
#endif
    bz_auth_convert_to_lsb(&param_list[2], BZ_AUTH_CHECK_VALUE_SIZE);
    param_list[0] = LMP_DHKEY_CHECK_OPCODE;
    bz_auth_generate_pdu(ce_index, param_list, LMP_DHKEY_CHECK_LEN, tid);
}

/**
 * Generate and send LMP_simple_pairing_confirm PDU to the remote device.
 *
 * \param ce_index ACL connection entity index.
 * \param auth Authentication parameter to be associated with the \a ce_index.
 * \param tid Transaction ID to be used for the PDU.
 *
 * \return None.
 */
void bz_auth_send_simple_pairing_confirm_pdu(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth, UCHAR Z, LMP_TRAN_ID tid)
{
    UCHAR param_list[LMP_SIMPLE_PAIRING_CONFIRM_LEN];

    /* Whoever sends the LMP_simple_pairing_confirm uses their own pubkey
     * first and then remote device's pubkey. It also uses local SP_Number.
     */
    /* Compute commitment/confirmation value */
#ifndef _CCH_SC_ECDH_P256_	
    ssp_f1(bz_auth_local_pubkey.x /* PKax/PKbx */,
            auth->txn_params.ssp_data.remote_pubkey.x /* PKax/PKbx */,
            auth->txn_params.ssp_data.local_N /* Na/Nb */,
            Z /* Used only in PE_METHOD */, &param_list[2]);
#else
    if( auth->len_prime == 6)
    {
    ssp_f1(bz_auth_local_pubkey.x /* PKax/PKbx */,
            auth->txn_params.ssp_data.remote_pubkey.x /* PKax/PKbx */,
            auth->txn_params.ssp_data.local_N /* Na/Nb */,
            Z /* Used only in PE_METHOD */, &param_list[2],auth->len_prime);
    }
    else if( auth->len_prime == 8)
    {
        ssp_f1(bz_auth_local_pubkey_p256.x /* PKax/PKbx */,
            auth->txn_params.ssp_data.remote_pubkey.x /* PKax/PKbx */,
            auth->txn_params.ssp_data.local_N /* Na/Nb */,
            Z /* Used only in PE_METHOD */, &param_list[2],auth->len_prime);
    }
#endif

    bz_auth_convert_to_lsb(&param_list[2], BZ_AUTH_CONFIRM_VALUE_SIZE);
    param_list[0] = LMP_SIMPLE_PAIRING_CONFIRM_OPCODE;
    bz_auth_generate_pdu(ce_index, param_list, LMP_SIMPLE_PAIRING_CONFIRM_LEN,
            tid);
}

/**
 * Generate and send LMP_simple_pairing_number PDU to the remote device.
 *
 * \param ce_index ACL connection entity index.
 * \param auth Authentication parameter to be associated with the \a ce_index.
 * \param tid Transaction ID to be used for the PDU.
 *
 * \return None.
 */
void bz_auth_send_simple_pairing_number_pdu(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth, LMP_TRAN_ID tid)
{
    UCHAR param_list[LMP_SIMPLE_PAIRING_NUMBER_LEN];

    /* We send our local simple pairing number and we may be initiator or
     * responder, so it is denoted as Na/Nb.
     */
    param_list[0] = LMP_SIMPLE_PAIRING_NUMBER_OPCODE;
    memcpy(&param_list[2], auth->txn_params.ssp_data.local_N /* Na/Nb */,
            BZ_AUTH_SP_NUMBER_SIZE);
    bz_auth_convert_to_lsb(&param_list[2], BZ_AUTH_SP_NUMBER_SIZE);
    bz_auth_generate_pdu(ce_index, param_list, LMP_SIMPLE_PAIRING_NUMBER_LEN,
            tid);
}

/**
 * Generate and send LMP_numeric_comparison_failed PDU to the remote device.
 *
 * \param ce_index ACL connection entity index.
 * \param auth Authentication parameter to be associated with the \a ce_index.
 * \param tid Transaction ID to be used for the PDU.
 *
 * \return None.
 */
void bz_auth_send_numeric_comparison_failed_pdu(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth, LMP_TRAN_ID tid)
{
    UCHAR param_list[LMP_NUMERIC_COMPARISON_FAILED_LEN];

    param_list[0] = LMP_ESCAPE4_OPCODE;
    param_list[2] = LMP_NUMERIC_COMPARISON_FAILED_OPCODE;
    bz_auth_generate_pdu(ce_index, param_list,
            LMP_NUMERIC_COMPARISON_FAILED_LEN, tid);
}

/**
 * Generate and send LMP_passkey_entry_failed PDU to the remote device.
 *
 * \param ce_index ACL connection entity index.
 * \param auth Authentication parameter to be associated with the \a ce_index.
 * \param tid Transaction ID to be used for the PDU.
 *
 * \return None.
 */
void bz_auth_send_passkey_entry_failed_pdu(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth, LMP_TRAN_ID tid)
{
    UCHAR param_list[LMP_PASSKEY_FAILED_LEN];

    param_list[0] = LMP_ESCAPE4_OPCODE;
    param_list[2] = LMP_PASSKEY_FAILED_OPCODE;
    bz_auth_generate_pdu(ce_index, param_list, LMP_PASSKEY_FAILED_LEN, tid);
}

/**
 * Generate and send LMP_oob_failed PDU to the remote device.
 *
 * \param ce_index ACL connection entity index.
 * \param auth Authentication parameter to be associated with the \a ce_index.
 * \param tid Transaction ID to be used for the PDU.
 *
 * \return None.
 */
void bz_auth_send_oob_failed_pdu(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth, LMP_TRAN_ID tid)
{
    UCHAR param_list[LMP_OOB_FAILED_LEN];

    param_list[0] = LMP_ESCAPE4_OPCODE;
    param_list[2] = LMP_OOB_FAILED_OPCODE;
    bz_auth_generate_pdu(ce_index, param_list, LMP_OOB_FAILED_LEN, tid);
}

/**
 * Handles simple pairing complete procedure. The function is called after the
 * [un]successful completion of the simple pairing procedure.
 *
 * \param ce_index ACL Connection entity index.
 * \param status Simple pairing completion status.
 *
 * \return None.
 */
void bz_auth_handle_simple_pairing_complete(UINT16 ce_index, UCHAR status)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;

    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;

    /* send HCI_Simple_Pairing_Complete_Event */
    bz_auth_generate_simple_pairing_complete_event(ce_index, status);
    if (status != HCI_COMMAND_SUCCEEDED)
    {
        bz_auth_handle_auth_completion(ce_index, (UCHAR)status);
        return;
    }
    /* Calculate the link key from dhkey, Na, Nb, BD_ADDR_A, BD_ADDR_B */
    bz_auth_calculate_ssp_link_key(ce_ptr, auth);
    if (auth->txn_params.auth_role == BZ_AUTH_ROLE_INITIATOR)
    {
        /* Challenge the responder (send LMP_au_rand) */
        bz_auth_send_au_rand_pdu(ce_index, auth, SELF_DEV_TID);
#ifdef SECURE_CONN_MUTUAL_AUTH
        if (auth->secure_conn_enabled)
        {
            bz_auth_transition_to_sub_state(auth,
                    SECURE_CONN_MUTUAL_SEND_AU_RAND);
        }
        else
#endif
        {
        bz_auth_transition_to_sub_state(auth,
                INTR_CHALLENGED_REMOTE_HOST_MUTUAL);
        }
    }
    else
    {
        /* Wait for challenge from initiator */
#ifdef SECURE_CONN_MUTUAL_AUTH
        if (auth->secure_conn_enabled)
        {
            bz_auth_transition_to_sub_state(auth,
                    SECURE_CONN_MUTUAL_AWAIT_AU_RAND);
        }
        else
#endif
        {
            bz_auth_transition_to_sub_state(auth, RESP_AWAIT_CHALLENGE_MUTUAL);
        }
    }
}

