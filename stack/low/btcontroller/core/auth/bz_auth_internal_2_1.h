/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file bz_auth_internal_2_1.h
 *  BlueWiz Authentication module internal interface (BT2.1 related) definition.
 *
 * \author Rajan PS, Santhosh kumar M
 * \date 2007-10-23
 */

#ifndef _BZ_AUTH_INTERNAL_2_1_H_
#define _BZ_AUTH_INTERNAL_2_1_H_

/* ========================= Include File Section ========================= */
#include "bz_auth.h"
#include "bz_auth_states.h"
#include "lmp.h"
#include "crypto.h"
#include "bz_auth_internal.h"
#include "lmp_edtm.h"


/**
 * \addtogroup bz_auth_internal Authentication Internal Interface
 * @{ */


/* ====================== Macro Declaration Section ======================= */
/* Authentication Requirements parameter */
#define NO_MITM_NO_BONDING              0x00
#define MITM_NO_BONDING                 0x01
#define NO_MITM_DEDICATED_BONDING       0x02
#define MITM_DEDICATED_BONDING          0x03
#define NO_MITM_GENERAL_BONDING         0x04
#define MITM_GENERAL_BONDING            0x05

/* IO Capability parameter */
#define DISPLAY_ONLY                    0x00
#define DISPLAY_YES_NO                  0x01
#define KEYBOARD_ONLY                   0x02
#define NO_INPUT_NO_OUTPUT              0x03

/* Timer Value to initiate auto-EPR.
 * BZ_AUTH_AUTO_EPR_TIMER_CHECK_VAL defines the interval in which we check for
 * auto epr, we may choose 10 minutes (60*10) = 600.
 * The encryption keys shall be refreshed by the Link Manager at least
 * once every 228 Bluetooth Clocks (about 23.3 hours)
 * So, as a compromize auto-epr will be done every 12 hrs (60*12/10)
 */
#define BZ_AUTH_AUTO_EPR_TIMER_CHECK_VAL            600
#define BZ_AUTH_AUTO_EPR_TIMER_VAL                  72


/* ==================== Data Types Declaration Section ==================== */


/* ================ Exported Variables Declaration Section ================ */


/* ============================= API Section ============================== */
void bz_auth_send_io_capability_req_pdu(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth, LMP_TRAN_ID tid);
void bz_auth_send_io_capability_res_pdu(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth, LMP_TRAN_ID tid);
void bz_auth_send_pause_encryption_req_pdu(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth, LMP_TRAN_ID tid);
void bz_auth_send_resume_encryption_req_pdu(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth, LMP_TRAN_ID tid);
void bz_auth_send_keypress_notification_pdu(UINT16 ce_index,
        UCHAR notification_type, LMP_TRAN_ID tid);
void bz_auth_reset_ssp_ext_feature_bits(void);
void bz_auth_enable_ssp_ext_feature_bits(void);
void bz_auth_generate_oob_data(void);
BOOLEAN bz_auth_verify_oob_hash_value(BZ_AUTH_LINK_PARAMS* auth,
        const UCHAR ReceivedC[BZ_AUTH_CONFIRM_VALUE_SIZE],
        const UCHAR RandomizerR[BZ_AUTH_OOB_SECRET_NUMBER_SIZE]);
void bz_auth_calculate_ssp_link_key(LMP_CONNECTION_ENTITY* ce_ptr,
        BZ_AUTH_LINK_PARAMS* auth);
BOOLEAN bz_auth_verify_dhkey_check_value(LMP_CONNECTION_ENTITY* ce_ptr,
        BZ_AUTH_LINK_PARAMS* auth, LMP_PDU_PKT* lmp_pdu_ptr);
void bz_auth_initiate_auth_stage2(UINT16 ce_index, LMP_TRAN_ID tid);
BOOLEAN bz_auth_verify_confirmation_value(BZ_AUTH_LINK_PARAMS* auth,
        LMP_PDU_PKT* lmp_pdu_ptr);
void bz_auth_update_final_pe_params(BZ_AUTH_LINK_PARAMS* auth);
void bz_auth_send_ith_pe_confirm(UINT16 ce_index, BZ_AUTH_LINK_PARAMS* auth,
        LMP_TRAN_ID tid);
void bz_auth_handle_public_key_sent_callback(UINT16 ce_index, UINT16 status,
        UCHAR reason, void* user_data);
void bz_auth_handle_public_key_received_callback(UINT16 ce_index,
        UINT16 status, UCHAR reason, void* user_data);
void bz_auth_initiate_public_key_exchange(UINT16 ce_index, LMP_TRAN_ID tid);
void bz_auth_perform_pause_encryption_action(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth);
void bz_auth_perform_resume_encryption_action(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth);
void bz_auth_extract_io_cap_data(const UCHAR input[3],
        OUT BZ_AUTH_IO_CAP_DATA* io_cap);
void bz_auth_send_dhkey_check_pdu(UINT16 ce_index, BZ_AUTH_LINK_PARAMS* auth,
        LMP_TRAN_ID tid);
void bz_auth_send_simple_pairing_confirm_pdu(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth, UCHAR Z, LMP_TRAN_ID tid);
void bz_auth_send_simple_pairing_number_pdu(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth, LMP_TRAN_ID tid);
void bz_auth_send_numeric_comparison_failed_pdu(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth, LMP_TRAN_ID tid);
void bz_auth_send_passkey_entry_failed_pdu(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth, LMP_TRAN_ID tid);
void bz_auth_send_oob_failed_pdu(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth, LMP_TRAN_ID tid);
void bz_auth_handle_simple_pairing_complete(UINT16 ce_index, UCHAR status);

/**
 * Local public key. It returns the appropriate public key based on the SSP
 * debug mode.
 */
#define bz_auth_local_pubkey         \
    (bz_auth_permanent_params.pubkey[bz_auth_dev_params.ssp_debug_mode])
/**
 * Local private key. It returns the appropriate private key based on the SSP
 * debug mode.
 */
#define bz_auth_local_prikey         \
    (bz_auth_permanent_params.prikey[bz_auth_dev_params.ssp_debug_mode])
/**
 * Local calculated public key. It is generated on every reset and local to
 * the controller.
 */
#define bz_auth_local_calc_pubkey (bz_auth_permanent_params.pubkey[0])
/**
 * Local calculated private key. It is generated on every reset and local to
 * the controller.
 */
#define bz_auth_local_calc_prikey (bz_auth_permanent_params.prikey[0])
/**
 * Local debug public key. It is provided in the specification.
 */
#define bz_auth_local_debug_pubkey (bz_auth_permanent_params.pubkey[1])
/**
 * Local debug private key. It is provided in the specification.
 */
#define bz_auth_local_debug_prikey (bz_auth_permanent_params.prikey[1])


#ifdef _CCH_SC_ECDH_P256_	

#define bz_auth_local_pubkey_p256         \
    (bz_auth_permanent_params.pubkey_p256[bz_auth_dev_params.ssp_debug_mode])

#define bz_auth_local_prikey_p256         \
    (bz_auth_permanent_params.prikey_p256[bz_auth_dev_params.ssp_debug_mode])

#define bz_auth_local_calc_pubkey_p256 (bz_auth_permanent_params.pubkey_p256[0])

#define bz_auth_local_calc_prikey_p256 (bz_auth_permanent_params.prikey_p256[0])

#define bz_auth_local_debug_pubkey_p256 (bz_auth_permanent_params.pubkey_p256[1])

#define bz_auth_local_debug_prikey_p256 (bz_auth_permanent_params.prikey_p256[1])

#endif



#define bz_auth_convert_to_msb(b, l)    convert_to_msb((b),(l))
#define bz_auth_convert_to_lsb(b, l)    convert_to_lsb((b),(l))

/** @} end: bz_auth_internal */

#endif /* _BZ_AUTH_INTERNAL_2_1_H_ */

