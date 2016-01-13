/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file bz_auth_lmp.c
 *  BlueWiz Authentication module 2.0 LMP interface implementation. It
 *  contains the handlers for 2.0 authentication related LMP PDUs.
 *
 * \author Santhosh kumar M
 * \date 2007-08-10
 */
/********************************* Logger *************************/ 
enum { __FILE_NUM__= 23};
/********************************* Logger *************************/
/* ========================= Include File Section ========================= */
#include "bz_auth.h"
#include "lmp.h"
#include "bz_auth_internal.h"
#include "bz_auth_internal_2_1.h"
#include "bz_debug.h"
#include "bz_auth_extern_accessors.h"
#include "bz_auth_hci.h"
#include "crypto.h"
#include "crypto11.h"
#include "bz_auth_lmp.h"
#include "bb_driver.h"
#ifdef COMPILE_BROADCAST_ENCRYPTION
#include "bz_auth_mlk.h"
#endif
#include "mem.h"

#ifdef _CCH_IOT_RALINK_
#include "lc.h"
#endif

/* ====================== Macro Declaration Section ======================= */


/* ==================== Structure Declaration Section ===================== */


/* ===================== Variable Declaration Section ===================== */


/* ================== Static Function Prototypes Section ================== */

/* ================== For rom code patch function point ================== */

#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8723_RCP_
PF_ROM_CODE_PATCH_FUNC  rcp_bz_auth_handle_au_rand_pdu_subcase = NULL;
PF_ROM_CODE_PATCH_FUNC  rcp_bz_auth_handle_sres_pdu_subcase = NULL;
#endif
#endif


/* ===================== Function Definition Section ====================== */
/**
 * Handles lmp_stop_encryption_req PDU from the remote device.
 *
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \param can_free_pdu Set to TRUE, if the PDU can be freed by the caller.
 *                     FALSE, otherwise.
 *
 * \return None.
 */
void bz_auth_handle_stop_encryption_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index, OUT BOOLEAN* can_free_pdu)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    UCHAR lmp_error_code = PDU_NOT_ALLOWED_ERROR;
    LMP_TRAN_ID tid;
    UCHAR active_tid;

    *can_free_pdu = TRUE;
    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;
    tid = (LMP_TRAN_ID)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr);

    active_tid = bz_auth_get_tid_from_auth_role(ce_ptr, auth);
#ifdef _CCH_IOT_RALINK_			
    if(!g_efuse_lps_setting_3.iot_ralink_tid_no_check )
    {
        if (tid != active_tid)
        {
            bz_auth_send_not_accepted_pdu(ce_index, LMP_STOP_ENCRYPTION_REQ_OPCODE,
                    tid, PDU_NOT_ALLOWED_ERROR);
            return;
        }
    }
#else
    if (tid != active_tid)
    {
        bz_auth_send_not_accepted_pdu(ce_index, LMP_STOP_ENCRYPTION_REQ_OPCODE,
                tid, PDU_NOT_ALLOWED_ERROR);
        return;
    }
#endif
    switch (auth->sub_state)
    {
        case AWAIT_PAUSE_ENCRYPTION_NOT_ACCEPTED:
        case AWAIT_DISABLE_ENCRYPTION_MOD_REQ_NOT_ACCEPTED:
            if (auth->pending_pdu == NULL)
            {
                bz_auth_set_pending_pdu(auth, lmp_pdu_ptr, can_free_pdu);
            }
            return;
        case AWAIT_STOP_ENCRYPTION:
            /* disable encryption for the link */
            bz_auth_disable_link_level_encryption(ce_index, TRUE);
            /* send lmp_accepted(stop_encryption) as unencrypted */
            bz_auth_send_accepted_pdu(ce_index,
                    LMP_STOP_ENCRYPTION_REQ_OPCODE, tid);
            /* Encryption Status(success) */
            bz_auth_handle_disable_enc_completion(ce_index,
                    HCI_COMMAND_SUCCEEDED);

#ifdef _CCH_SC_ECDH_P256_PAUSE_ENC
            if(auth->secure_conn_enabled)
            {
#ifdef _CCH_SC_ECDH_P256_LOG    
                RT_BT_LOG(BLUE, CCH_DBG_155, 0, 0);
#endif

#ifdef _CCH_SC_ECDH_P256_LOG
    RT_BT_LOG(BLUE, YL_DBG_HEX_4, 4, auth->enc_rand[0], auth->enc_rand[1], auth->enc_rand[2], auth->enc_rand[3]);
#endif

#ifdef _CCH_SC_TEST_20130129_QCA_02
            UCHAR temp[BZ_AUTH_ENC_RAND_SIZE];
            memcpy(&temp[0], &auth->enc_rand[0], BZ_AUTH_ENC_RAND_SIZE);
            bz_auth_convert_to_msb(&temp[0], 16);
            BB_write_sc_iv(ce_index, &temp[0]);					
#else

#ifdef _CCH_SC_TEST_20130129_QCA_04
#ifdef _CCH_SC_TEST_20130129_QCA_05
#ifdef _DAPE_TEST_CHK_SC_ROLE_SW
RT_BT_LOG(RED, DAPE_TEST_LOG293, 1,ce_ptr->remote_dev_role);
#endif

            if(ce_ptr->remote_dev_role == MASTER )
            {
                BB_write_sc_iv(ce_index, &auth->enc_rand_remote[8]);
			         }
            else
			{
                BB_write_sc_iv(ce_index, &auth->enc_rand[8]);
			}

#else
            BB_write_sc_iv(ce_index, &auth->enc_rand[8]);
#endif
#else
            BB_write_sc_iv(ce_index, &auth->enc_rand[0]);
#endif
#endif


            }
#endif

			
            return;
        default:
            BZ_ASSERT(0, "Bug in the state machine or remote device is "
                    "misbehaving");
            break;
    }
    bz_auth_send_not_accepted_pdu(ce_index,
            LMP_STOP_ENCRYPTION_REQ_OPCODE, tid, lmp_error_code);
}

/**
 * Handles lmp_encryption_key_size_mask_req PDU from the remote device.
 *
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \param can_free_pdu Set to TRUE, if the PDU can be freed by the caller.
 *                     FALSE, otherwise.
 *
 * \return None.
 */
void bz_auth_handle_encryption_key_size_mask_req_pdu(
        LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index, OUT BOOLEAN* can_free_pdu)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    UCHAR lmp_error_code = HCI_COMMAND_SUCCEEDED;
    LMP_TRAN_ID tid;

    ce_ptr = &lmp_connection_entity[ce_index];

    auth = ce_ptr->auth;
    tid = (LMP_TRAN_ID)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr);

    if (bz_auth_is_master(ce_ptr)
            || tid != MASTER)
    {
        lmp_error_code = PDU_NOT_ALLOWED_ERROR;
    }
    else if ((bz_auth_get_local_features(bz_auth_get_piconet_id(ce_ptr))[2]
                & LMP_BC_ENCRYPTION_FEATURE) == 0)
    {
        lmp_error_code = UNSUPPORTED_REMOTE_FEATURE_ERROR;
    }

    if (lmp_error_code != HCI_COMMAND_SUCCEEDED)
    {
        /* NOTE: No need to start tid timer, so using generic
         *       lmp_send_lmp_not_accepted function.
         */
        lmp_send_lmp_not_accepted(ce_index,
                LMP_ENCRYPTION_KEY_SIZE_MASK_REQ_OPCODE,
                (UCHAR)tid, lmp_error_code);
        return;
    }
    bz_auth_send_encryption_key_size_mask_res_pdu(ce_index, auth, tid);
}

/**
 * Handles lmp_encryption_key_size_mask_res PDU from the remote device.
 *
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \param can_free_pdu Set to TRUE, if the PDU can be freed by the caller.
 *                     FALSE, otherwise.
 *
 * \return None.
 */
void bz_auth_handle_encryption_key_size_mask_res_pdu(
        LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index, OUT BOOLEAN* can_free_pdu)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    UCHAR lmp_error_code = HCI_COMMAND_SUCCEEDED;
    LMP_TRAN_ID tid;
    UINT16 key_size_mask;

    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;
    tid = (LMP_TRAN_ID)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr);

    if (bz_auth_is_slave(ce_ptr)
            || tid != MASTER)
    {
        lmp_error_code = PDU_NOT_ALLOWED_ERROR;
    }

    if (lmp_error_code != HCI_COMMAND_SUCCEEDED)
    {
       lmp_send_lmp_not_accepted(ce_index,
                LMP_ENCRYPTION_KEY_SIZE_MASK_RES_OPCODE,
                (UCHAR)tid, lmp_error_code);
        return;
    }
    /* Store the encryption key size mask of the slave. */
    key_size_mask = (UINT16)(lmp_pdu_ptr->payload_content[2] << 8);
    key_size_mask = (UINT16)(key_size_mask | lmp_pdu_ptr->payload_content[1]);
    auth->key_size_mask = key_size_mask;
}

/**
 * Handles lmp_encryption_key_size_req PDU from the remote device.
 *
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \param can_free_pdu Set to TRUE, if the PDU can be freed by the caller.
 *                     FALSE, otherwise.
 *
 * \return None.
 */
void bz_auth_handle_encryption_key_size_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index, OUT BOOLEAN* can_free_pdu)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    UCHAR lmp_error_code = PDU_NOT_ALLOWED_ERROR;
    LMP_TRAN_ID tid;
    UCHAR enc_key_size;
    UCHAR local_max_enc_key_size;
    UCHAR active_tid;

    *can_free_pdu = TRUE;
    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;
    tid = (LMP_TRAN_ID)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr);
    enc_key_size = lmp_pdu_ptr->payload_content[1];

    active_tid = bz_auth_get_tid_from_auth_role(ce_ptr, auth);
#ifdef _CCH_IOT_RALINK_			
    if(!g_efuse_lps_setting_3.iot_ralink_tid_no_check )
    {
        if (tid != active_tid)
        {
            bz_auth_send_not_accepted_pdu(ce_index,
                    LMP_ENCRYPTION_KEY_SIZE_REQ_OPCODE, tid, PDU_NOT_ALLOWED_ERROR);
            return;
        }
    }
#else
    if (tid != active_tid)
    {
        bz_auth_send_not_accepted_pdu(ce_index,
                LMP_ENCRYPTION_KEY_SIZE_REQ_OPCODE, tid, PDU_NOT_ALLOWED_ERROR);
        return;
    }
#endif
    switch (auth->sub_state)
    {
        case AWAIT_ENABLE_ENCRYPTION_MOD_REQ_NOT_ACCEPTED:
            if (auth->pending_pdu == NULL)
            {
                bz_auth_set_pending_pdu(auth, lmp_pdu_ptr, can_free_pdu);
            }
            return;
        case ENCRYPTION_KEY_SIZE_SENT:
            /* case1: key size should be <= what we sent and => our
             * MIN supported value. otherwize send not accepted
             */
            if ((enc_key_size <
                        bz_auth_local_supported_min_enc_key_size(ce_ptr, auth))
                    || enc_key_size > auth->enc_key_size)
            {
                bz_auth_send_not_accepted_pdu(ce_index,
                        LMP_ENCRYPTION_KEY_SIZE_REQ_OPCODE,
                        tid, UNSUPPORTED_PARAMETER_VALUE_ERROR);
                /* Encryption Status(failed) */
                bz_auth_handle_enable_enc_completion(ce_index,
                        INSUFFICIENT_SECURITY);
                return;
            }

            /* case2: Since our current implementation supports a range
             * of values there is no possibility of negotiation.
             * Negotiation possible only when we support discrete
             * values like 1, 2, 4, 8, 12, 16 etc
             * So send lmp_encryption_key_size with negotiated value is not
             * applicable now.
             */
            /* now the received value is within our limit, send accepted pdu */
            bz_auth_send_accepted_pdu(ce_index,
                    LMP_ENCRYPTION_KEY_SIZE_REQ_OPCODE, tid);
            /* store negotiated enc_key_size */
            auth->enc_key_size = enc_key_size;
            if (bz_auth_is_master(ce_ptr))  /* Are we master? */
            {
                /* send start_encryption_req and calculate encryption_key */
                bz_auth_send_start_encryption_pdu(ce_index, auth, tid);
                bz_auth_transition_to_sub_state(auth, START_ENCRYPTION_SENT);
            }
            else
            {
                /* Wait for start_encryption_request */
                bz_auth_transition_to_sub_state(auth, AWAIT_START_ENCRYPTION);
            }
            return;

        case AWAIT_ENCRYPTION_KEY_SIZE_REQ:
            /* Only slave can get into this state */

            /* case1: key size < our min supported value,
             * we cannot accept it */
            if (enc_key_size <
                    bz_auth_local_supported_min_enc_key_size(ce_ptr, auth))
            {
                bz_auth_send_not_accepted_pdu(ce_index,
                        LMP_ENCRYPTION_KEY_SIZE_REQ_OPCODE,
                        tid, UNSUPPORTED_PARAMETER_VALUE_ERROR);
                /* Encryption Status(failed) */
                bz_auth_handle_enable_enc_completion(ce_index,
                        INSUFFICIENT_SECURITY);
                return;
            }
            local_max_enc_key_size =
                bz_auth_local_supported_max_enc_key_size(auth);
            /* case2: key size to be negotiated */
            if (enc_key_size > local_max_enc_key_size)
            {
                /* store negotiated enc_key_size to be sent */
                auth->enc_key_size = local_max_enc_key_size;
                /* send lmp_encryption_key_size with negotiated value */
                bz_auth_send_encryption_key_size_req_pdu(ce_index, auth, tid);
                return;
            }

            /* case3: key size agreed */
            /* send lmp_accepted */
            auth->enc_key_size = enc_key_size;
            bz_auth_send_accepted_pdu(ce_index,
                    LMP_ENCRYPTION_KEY_SIZE_REQ_OPCODE, tid);
            /* sub state change to AWAIT_START_ENCRYPTION */
            bz_auth_transition_to_sub_state(auth, AWAIT_START_ENCRYPTION);
            return;

        default:
            BZ_ASSERT(0, "Bug in the state machine or remote device is "
                    "misbehaving");
            break;
    }
    bz_auth_send_not_accepted_pdu(ce_index,
            LMP_ENCRYPTION_KEY_SIZE_REQ_OPCODE, tid, lmp_error_code);
}

/**
 * Handles lmp_accepted_ext for security related pdus.
 *
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \param can_free_pdu Set to TRUE, if the PDU can be freed by the caller.
 *                     FALSE, otherwise.
 *
 * \return TRUE, if the PDU was handled. FALSE, otherwise.
 */
BOOLEAN bz_auth_handle_accepted_ext_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index, OUT BOOLEAN* can_free_pdu)
{
    UCHAR pdu_ext_opcode;

    pdu_ext_opcode = lmp_pdu_ptr->payload_content[3];

    switch (pdu_ext_opcode)
    {
        default:
            return FALSE;
    }
}

/**
 * Handles lmp_accepted(lmp_in_rand) PDU from the remote device.
 *
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \param can_free_pdu Set to TRUE, if the PDU can be freed by the caller.
 *                     FALSE, otherwise.
 *
 * \return None.
 */
void bz_auth_handle_in_rand_accepted_pdu(LMP_PDU_PKT* lmp_pdu_ptr,
                    UINT16 ce_index, OUT BOOLEAN* can_free_pdu)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;

    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;
   
    switch (auth->sub_state)
    {
        case INTR_IN_RAND_PDU_SENT:
            bz_auth_return_if_remote_initiated_transaction;
            /* calculate and send  unit_key/comb_key protected by generated
             * init_key
             */
            bz_auth_intr_send_unit_or_comb_key(ce_index, auth);
            break;

        case RESP_IN_RAND_PDU_SENT:
            bz_auth_return_if_self_initiated_transaction;
            /* wait for unit_key/comb_key */
            bz_auth_transition_to_sub_state(auth, RESP_AWAIT_UNIT_OR_COMB_KEY);
            break;

        default:
            BZ_ASSERT(0, "Bug in the state machine or remote device is "
                    "misbehaving");
            break;
    }
}

/**
 * Handles lmp_accepted(lmp_encryption_key_size_req) PDU from the remote device.
 *
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \param can_free_pdu Set to TRUE, if the PDU can be freed by the caller.
 *                     FALSE, otherwise.
 *
 * \return None.
 */
void bz_auth_handle_encryption_key_size_req_accepted_pdu(
        LMP_PDU_PKT* lmp_pdu_ptr, UINT16 ce_index, OUT BOOLEAN* can_free_pdu)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    LMP_TRAN_ID tid;
    UCHAR active_tid;

    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;
    tid = (LMP_TRAN_ID)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr);
    active_tid = bz_auth_get_tid_from_auth_role(ce_ptr, auth);
#ifdef _CCH_IOT_RALINK_			
    if(!g_efuse_lps_setting_3.iot_ralink_tid_no_check )
    {
        if (tid != active_tid)
        {
            BZ_ASSERT(0, "Bug in the tid calculation or remote device is "
                    "misbehaving");
            return;
        }
    }
#else
    if (tid != active_tid)
    {
        BZ_ASSERT(0, "Bug in the tid calculation or remote device is "
                "misbehaving");
        return;
    }
#endif
    switch (auth->sub_state)
    {
        case ENCRYPTION_KEY_SIZE_SENT:
            if (bz_auth_is_master(ce_ptr))
            {
                /* send start_encryption_req pdu
                 * change sub state to START_ENCRYPTION_SENT */
                bz_auth_send_start_encryption_pdu(ce_index, auth, tid);
                bz_auth_transition_to_sub_state(auth, START_ENCRYPTION_SENT);
            }
            else
            {
                 /* change sub state to AWAIT_START_ENCRYPTION */
                bz_auth_transition_to_sub_state(auth, AWAIT_START_ENCRYPTION);
            }
            break;

        default:
            BZ_ASSERT(0, "Bug in the state machine or remote device is "
                    "misbehaving");
            break;
    }
}

/**
 * Handles lmp_accepted(lmp_encryption_mode_req) PDU from the remote device.
 *
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \param can_free_pdu Set to TRUE, if the PDU can be freed by the caller.
 *                     FALSE, otherwise.
 *
 * \return None.
 */
void bz_auth_handle_encryption_mode_req_accepted_pdu(
        LMP_PDU_PKT* lmp_pdu_ptr, UINT16 ce_index, OUT BOOLEAN* can_free_pdu)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    LMP_TRAN_ID tid;
    UCHAR active_tid;

    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;
    tid = (LMP_TRAN_ID)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr);
    active_tid = bz_auth_get_tid_from_auth_role(ce_ptr, auth);
#ifdef _CCH_IOT_RALINK_			
    if(!g_efuse_lps_setting_3.iot_ralink_tid_no_check )
    {
        if (tid != active_tid)
        {
            BZ_ASSERT(0, "Bug in the tid calculation or remote device is "
                    "misbehaving");
            return;
        }
    }
#else
    if (tid != active_tid)
    {
        BZ_ASSERT(0, "Bug in the tid calculation or remote device is "
                "misbehaving");
        return;
    }
#endif
    switch (auth->sub_state)
    {
        case DISABLE_ENCRYPTION_MODE_REQ_SENT:
            if (bz_auth_is_master(ce_ptr))
            {
                /* send stop_encryption_pdu */
                bz_auth_send_stop_encryption_pdu(ce_index, auth, tid);
                /* change sub state to STOP_ENCRYPTION_SENT */
                bz_auth_transition_to_sub_state(auth, STOP_ENCRYPTION_SENT);
            }
            else
            {
                /* change sub state to AWAIT_STOP_ENCRYPTION */
                bz_auth_transition_to_sub_state(auth, AWAIT_STOP_ENCRYPTION);
            }
            break;

        case ENABLE_ENCRYPTION_MODE_REQ_SENT:
            if (bz_auth_is_master(ce_ptr))
            {
                /* store the max supported encryption key size to sent */
                auth->enc_key_size =
                    bz_auth_local_supported_max_enc_key_size(auth);
                /* Send encryption key size req and transition to appropriate
                 * state (ENCRYPTION_KEY_SIZE_SENT).
                 */
                bz_auth_send_encryption_key_size_req_pdu(ce_index, auth, tid);
            }
            else
            {
                 /* change sub state to AWAIT_ENCRYPTION_KEY_SIZE_REQ */
                bz_auth_transition_to_sub_state(auth,
                        AWAIT_ENCRYPTION_KEY_SIZE_REQ);
            }
            break;

        default:
            BZ_ASSERT(0, "Bug in the state machine or remote device is "
                    "misbehaving");
            break;
    }
}

/**
 * Handles lmp_accepted(lmp_stop_encryption_req) PDU from the remote device.
 *
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \param can_free_pdu Set to TRUE, if the PDU can be freed by the caller.
 *                     FALSE, otherwise.
 *
 * \return None.
 */
void bz_auth_handle_stop_encryption_req_accepted_pdu(
        LMP_PDU_PKT* lmp_pdu_ptr, UINT16 ce_index, OUT BOOLEAN* can_free_pdu)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    LMP_TRAN_ID tid;
    UCHAR active_tid;

    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;
    tid = (LMP_TRAN_ID)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr);
    active_tid = bz_auth_get_tid_from_auth_role(ce_ptr, auth);
#ifdef _CCH_IOT_RALINK_			
    if(!g_efuse_lps_setting_3.iot_ralink_tid_no_check )
    {
        if (tid != active_tid)
        {
            BZ_ASSERT(0, "Bug in the tid calculation or remote device is "
                    "misbehaving");
            return;
        }
    }
#else
    if (tid != active_tid)
    {
        BZ_ASSERT(0, "Bug in the tid calculation or remote device is "
                "misbehaving");
        return;
    }
#endif

    switch (auth->sub_state)
    {
        case STOP_ENCRYPTION_SENT:
    	    BB_encryption_control(ce_ptr->am_addr, ce_ptr->phy_piconet_id, 
            BB_ENC_TX_DISBALE_RX_DISABLE);

            /* Stop Encryption status(success) */
            bz_auth_handle_disable_enc_completion(ce_index,
    	       HCI_COMMAND_SUCCEEDED);

#ifdef _CCH_SC_ECDH_P256_PAUSE_ENC
            if(auth->secure_conn_enabled)
            {
                auth->sc_use_enc_rand = 1;    
            }
#endif
            break;
        default:
            BZ_ASSERT(0, "Bug in the state machine or remote device is "
				"misbehaving");
            break;
	}

	return;
}

/**
 * Handles lmp_accepted(lmp_start_encryption_req) PDU from the remote device.
 *
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \param can_free_pdu Set to TRUE, if the PDU can be freed by the caller.
 *                     FALSE, otherwise.
 *
 * \return None.
 */
void bz_auth_handle_start_encryption_req_accepted_pdu(
        LMP_PDU_PKT* lmp_pdu_ptr, UINT16 ce_index, OUT BOOLEAN* can_free_pdu)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    LMP_TRAN_ID tid;
    UCHAR active_tid;

    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;
    tid = (LMP_TRAN_ID)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr);
    active_tid = bz_auth_get_tid_from_auth_role(ce_ptr, auth);

#ifdef _CCH_SC_ECDH_P256_LOG
RT_BT_LOG(WHITE, YL_DBG_HEX_3, 3, tid, active_tid, auth->sub_state);
#endif
	
#ifdef _CCH_IOT_RALINK_			
    if(!g_efuse_lps_setting_3.iot_ralink_tid_no_check )
    {    
        if (tid != active_tid)
        {
            BZ_ASSERT(0, "Bug in the tid calculation or remote device is "
                    "misbehaving");
            return;
        }
    }
#else
    if (tid != active_tid)
    {
        BZ_ASSERT(0, "Bug in the tid calculation or remote device is "
                "misbehaving");
        return;
    }
#endif
    switch (auth->sub_state)
    {
        case START_ENCRYPTION_SENT:
    		BB_encryption_control(ce_ptr->am_addr, 
    			ce_ptr->phy_piconet_id, BB_ENC_TX_ENABLE_RX_ENABLE);

    		/* Start Encryption status(success) */
    		bz_auth_handle_enable_enc_completion(ce_index,
    			HCI_COMMAND_SUCCEEDED);
    		break;
		default:
			BZ_ASSERT(0, "Bug in the state machine or remote device is "
				"misbehaving");
			break;
	}

	return;
}

/**
 * Handles lmp accepted for security related pdus.
 *
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \param can_free_pdu Set to TRUE, if the PDU can be freed by the caller.
 *                     FALSE, otherwise.
 *
 * \return TRUE, if the PDU was handled. FALSE, otherwise.
 */
BOOLEAN bz_auth_handle_accepted_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index, OUT BOOLEAN* can_free_pdu)
{
    UCHAR pdu_opcode;

    pdu_opcode = lmp_pdu_ptr->payload_content[1];

    switch (pdu_opcode)
    {
        case LMP_IN_RAND_OPCODE:
            bz_auth_handle_in_rand_accepted_pdu(lmp_pdu_ptr,
                    ce_index, can_free_pdu);
            break;
        case LMP_ENCRYPTION_KEY_SIZE_REQ_OPCODE:
            bz_auth_handle_encryption_key_size_req_accepted_pdu(lmp_pdu_ptr,
                    ce_index, can_free_pdu);
            break;
        case LMP_ENCRYPTION_MODE_REQ_OPCODE:
            bz_auth_handle_encryption_mode_req_accepted_pdu(lmp_pdu_ptr,
                    ce_index, can_free_pdu);
            break;
        case LMP_START_ENCRYPTION_REQ_OPCODE:
            bz_auth_handle_start_encryption_req_accepted_pdu(lmp_pdu_ptr,
                    ce_index, can_free_pdu);
            break;
        case LMP_STOP_ENCRYPTION_REQ_OPCODE:
            bz_auth_handle_stop_encryption_req_accepted_pdu(lmp_pdu_ptr,
                    ce_index, can_free_pdu);
            break;
#ifdef COMPILE_BROADCAST_ENCRYPTION
        case LMP_USE_SEMI_PERMANENT_KEY_OPCODE:
            bz_auth_handle_use_semi_permanent_key_accepted_pdu(lmp_pdu_ptr,
                    ce_index, can_free_pdu);
            break;
#endif
        case LMP_SIMPLE_PAIRING_CONFIRM_OPCODE:
            /* not expecting  this pdu */
            break;
        case LMP_SIMPLE_PAIRING_NUMBER_OPCODE:
            bz_auth_handle_simple_pairing_number_accepted_pdu(lmp_pdu_ptr,
                    ce_index, can_free_pdu);
            break;
        case LMP_DHKEY_CHECK_OPCODE:
            bz_auth_handle_dhkey_check_accepted_pdu(lmp_pdu_ptr,
                    ce_index, can_free_pdu);
            break;
        default:
            return FALSE;
    }
    return TRUE;
}

/**
 * Handles lmp_not_accepted_ext for security related pdus.
 *
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \param can_free_pdu Set to TRUE, if the PDU can be freed by the caller.
 *                     FALSE, otherwise.
 *
 * \return TRUE, if the PDU was handled. FALSE, otherwise.
 */
BOOLEAN bz_auth_handle_not_accepted_ext_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index, OUT BOOLEAN* can_free_pdu)
{
    UCHAR pdu_ext_opcode;    
    UCHAR lmp_reason;

    pdu_ext_opcode = lmp_pdu_ptr->payload_content[3];
    lmp_reason = lmp_pdu_ptr->payload_content[4];   

    switch (pdu_ext_opcode)
    {
        case LMP_PAUSE_ENCRYPTION_REQ_OPCODE:
            bz_auth_handle_pause_encryption_req_not_accepted_pdu(lmp_pdu_ptr,
                    ce_index, can_free_pdu);
            break;

        case LMP_RESUME_ENCRYPTION_REQ_OPCODE:
            /* So bad, this should not happen! */
            bz_auth_handle_enable_enc_completion(ce_index, lmp_reason);
            break;

        case LMP_IO_CAPABILITY_REQ_OPCODE:
            bz_auth_handle_io_capability_req_not_accepted_pdu(lmp_pdu_ptr,
                    ce_index, can_free_pdu);
            break;

        case LMP_IO_CAPABILITY_RES_OPCODE:
#if 0
            bz_auth_handle_simple_pairing_complete(ce_index, lmp_reason);
#endif
            break;

        default:
            return FALSE;
    }
    return TRUE;   
}

/**
 * Handles lmp_au_rand PDU from the remote device.
 *
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \param can_free_pdu Set to TRUE, if the PDU can be freed by the caller.
 *                     FALSE, otherwise.
 *
 * \return None.
 */
void bz_auth_handle_au_rand_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index, OUT BOOLEAN* can_free_pdu)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    UCHAR lmp_error_code = HCI_COMMAND_SUCCEEDED;
    LMP_TRAN_ID tid;

    *can_free_pdu = TRUE;
    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;
    tid = (LMP_TRAN_ID)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr);
#ifdef _SECURE_CONN_TEST_LOG
RT_BT_LOG(BLUE, DAPE_TEST_LOG525, 6, auth->sub_state,
auth->super_state,
auth->secure_conn_enabled, auth->len_prime,0,0);
#endif

    switch(auth->sub_state)
    {
        case INTR_LINK_KEY_REQUESTED:
            /* First PDU
             *	Store it.
             * else send LMP_not_accepted.
             */

            if (bz_auth_is_self_initiated(ce_ptr, lmp_pdu_ptr)
                    ||	auth->pending_pdu != NULL)
            {
                lmp_error_code = PDU_NOT_ALLOWED_ERROR;
                break;
            }
            else
            {
                bz_auth_set_pending_pdu(auth, lmp_pdu_ptr, can_free_pdu);
            }
            return;

        case INTR_PIN_CODE_REQUESTED:
        case INTR_IN_RAND_PDU_SENT:
        case INTR_AWAIT_REMOTE_IO_CAP_RESPONSE:
            /* LMP_not_accepted, Key Missing */
            lmp_error_code = KEY_MISSING_ERROR;
            break;

#ifdef SECURE_CONN_MUTUAL_AUTH
        case SECURE_CONN_MUTUAL_SEND_AU_RAND:
#ifdef _SECURE_CONN_TEST_LOG
RT_BT_LOG(YELLOW, DAPE_TEST_LOG525, 6,auth->secure_conn_enabled, auth->len_prime, tid,0,0,0);
#endif        	
            bz_auth_transition_to_sub_state(auth, SECURE_CONN_INTR_AWAIT_SRES);
            if (bz_auth_is_master(ce_ptr))
            {
                bz_auth_set_pending_pdu(auth, lmp_pdu_ptr, can_free_pdu);                
            }
            else
            {
                bz_auth_send_sres_pdu(ce_index, auth, lmp_pdu_ptr);
            }
            return;
        case SECURE_CONN_MUTUAL_AWAIT_AU_RAND:
#ifdef _SECURE_CONN_TEST_LOG
RT_BT_LOG(YELLOW, DAPE_TEST_LOG525, 6,auth->secure_conn_enabled, auth->len_prime, tid,0,0,0);
#endif        	
            bz_auth_send_au_rand_pdu(ce_index, auth, tid);
            bz_auth_transition_to_sub_state(auth, SECURE_CONN_INTR_AWAIT_SRES);
            if (bz_auth_is_master(ce_ptr))
            {
                bz_auth_set_pending_pdu(auth, lmp_pdu_ptr, can_free_pdu);                
            }
            else
            {
                bz_auth_send_sres_pdu(ce_index, auth, lmp_pdu_ptr);
            }
            return;            
#endif
        case INTR_CHALLENGED_REMOTE_HOST:
            if (bz_auth_is_master(ce_ptr))
            {
#ifdef SECURE_CONN_MUTUAL_AUTH
#ifdef _SECURE_CONN_TEST_LOG
RT_BT_LOG(YELLOW, DAPE_TEST_LOG525, 6,auth->secure_conn_enabled, auth->len_prime, tid,0,0,0);
#endif

                if ((auth->secure_conn_enabled) && (tid == MASTER_TID))
                {
                    bz_auth_set_pending_pdu(auth, lmp_pdu_ptr, can_free_pdu);
                    bz_auth_transition_to_sub_state(auth, SECURE_CONN_INTR_AWAIT_SRES);
                }
                else
#endif                
                {
                bz_auth_send_not_accepted_pdu(ce_index, LMP_AU_RAND_OPCODE,
                        (LMP_TRAN_ID)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr),
                        LMP_ERROR_TRANSACTION_COLLISION_ERROR);
                }
            }
            else
            {
#ifdef SECURE_CONN_MUTUAL_AUTH
#ifdef _SECURE_CONN_TEST_LOG
RT_BT_LOG(YELLOW, DAPE_TEST_LOG525, 6,auth->secure_conn_enabled, auth->len_prime, tid,0,0,0);
#endif

                if ((auth->secure_conn_enabled) && (tid == SLAVE_TID))
                {
                    bz_auth_send_sres_pdu(ce_index, auth, lmp_pdu_ptr);
                    bz_auth_transition_to_sub_state(auth, SECURE_CONN_INTR_AWAIT_SRES);
                }
                else
#endif           
                {
                    auth->txn_params.auth_role =
                    BZ_AUTH_ROLE_INITIATOR_AS_RESPONDER;
                    bz_auth_set_pending_pdu(auth, lmp_pdu_ptr, can_free_pdu);
                    bz_auth_transition_to_sub_state(auth,
                        INTR_RESP_AWAIT_AU_RAND_PDU_NOT_ACCEPTED);
                }
            }
            return;

        case INTR_RESP_AWAIT_CHALLENGE:
            bz_auth_break_if_self_initiated_transaction;

            /* calculate and send sres */
#ifdef SECURE_CONN_MUTUAL_AUTH
#ifdef _SECURE_CONN_TEST_LOG
RT_BT_LOG(YELLOW, DAPE_TEST_LOG522, 2,auth->secure_conn_enabled, auth->len_prime);
#endif

            if (auth->secure_conn_enabled)
            {
                /* dape: Since we only will go to this state 
                when we are slave, so for simplicity, we dont 
                add the code for master here. */
                bz_auth_send_au_rand_pdu(ce_index, auth, tid);
                bz_auth_send_sres_pdu(ce_index, auth, lmp_pdu_ptr);
                bz_auth_transition_to_sub_state(auth,
                    RESP_AWAIT_CHALLENGED_REMOTE_HOST);
            }
            else
#endif
            {
            bz_auth_send_sres_pdu(ce_index, auth, lmp_pdu_ptr);
            bz_auth_handle_auth_completion(ce_index, HCI_COMMAND_SUCCEEDED);
            }
            return;

        case RESP_AWAIT_CHALLENGE_MUTUAL:
            bz_auth_break_if_self_initiated_transaction;

            /* calculate and send sres */
            bz_auth_send_sres_pdu(ce_index, auth, lmp_pdu_ptr);
            /* send LMP_au_rand */
            bz_auth_send_au_rand_pdu(ce_index, auth, tid);
            bz_auth_transition_to_sub_state(auth,
                    RESP_AWAIT_CHALLENGED_REMOTE_HOST);
            return;

        case INTR_AWAIT_REMOTE_CHALLENGE:

#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8723_RCP_
        if (rcp_bz_auth_handle_au_rand_pdu_subcase != NULL)
        {
            if ( rcp_bz_auth_handle_au_rand_pdu_subcase((void *)&ce_index, auth, lmp_pdu_ptr) )
            {
                return;
            }
        }    
#endif
#endif
            bz_auth_break_if_remote_initiated_transaction;

            /* calculate and send sres */
            bz_auth_send_sres_pdu(ce_index, auth, lmp_pdu_ptr);
			bz_auth_verify_challenge(ce_index, auth, TRUE);
            return;

        case BZ_AUTH_SUB_STATE_IDLE:
            break;

        case ENABLE_ENCRYPTION_MODE_REQ_SENT:
            /* This is a special case which happens mostly during connection
             * when the auth_enable and enc_enable is done at both the sides.
             * It increases the possiblity of collision between authentication
             * and encryption (basically different transaction collision).
             * Ignoring this case might render us to not interop with some
             * of the existing devices (100% we will not be able connect with
             * them).
             * Sample scenario with k750i phone and our HFU device(Slave):
             * HFU:                             k750i:
             * lmp_accepted(conn)  -->
             * lmp_au_rand         -->
             *                     <--  lmp_sres
             * lmp_enc_mode_req    -->
             *                     <--  lmp_au_rand
             * lmp_not_accepted    --> [[[[ This is the problem ]]]]
             *
             * This fix addresses this scenario.
             */
            /* set the link key in transaction params */
            memcpy(&auth->txn_params.link_key[0], auth->link_key,
                    BZ_AUTH_LINK_KEY_SIZE);
            auth->txn_params.lk_type = auth->lk_type;
            /* send LMP_sres */
            bz_auth_send_sres_pdu(ce_index, auth, lmp_pdu_ptr);
            /* update the new link_key, lk_type, aco from txn_params */
            bz_auth_update_auth_params_from_txn_params(auth);
            return;
        default:
		    LMP_LOG_INFO(LOG_LEVEL_HIGH, SOFT_ASSERT_COLON_DIFFERENT_TRANSACTION, 0, 0);
            lmp_error_code = PDU_NOT_ALLOWED_ERROR;
            break;
    }

    bz_auth_return_if_error_or_self_initiated_transaction(LMP_AU_RAND_OPCODE);

    switch (auth->super_state)
    {
        case UNAUTHENTICATED_DURING_CONN:
        case UNAUTHENTICATED:
            auth->txn_params.auth_role = BZ_AUTH_ROLE_RESPONDER;
            bz_auth_set_pending_pdu(auth, lmp_pdu_ptr, can_free_pdu);
            bz_auth_perform_super_state_transition(auth,
                    HCI_AUTH_REQ_OR_AURAND_INRAND_OR_IOCAP_REQ_AUTH_ENABLE);
            /* send HCI_LinkKey_request  */
            bz_auth_enter_link_key_requested_state(ce_index, auth,
                    RESP_LINK_KEY_REQUESTED);
            break;

        case AUTHENTICATED:
        case ENCRYPTED:
        case AUTHENTICATED_DURING_CONN:
        case ENCRYPTED_DURING_CONN:
        case TEMP_AUTHENTICATED:
        case TEMP_ENCRYPTED:
            auth->txn_params.auth_role = BZ_AUTH_ROLE_RESPONDER;
            /* set the link key in transaction params */
            memcpy(&auth->txn_params.link_key[0], auth->link_key,
                    BZ_AUTH_LINK_KEY_SIZE);
            auth->txn_params.lk_type = auth->lk_type;
            /* We won't be upgrading key when TEMP_KEY in use. But we have to
             * copy the ACO value generated during authentication.
             */
            /* send LMP_sres */
            bz_auth_send_sres_pdu(ce_index, auth, lmp_pdu_ptr);
            bz_auth_perform_super_state_transition(auth,
                    HCI_AUTH_REQ_OR_AURAND_INRAND_OR_IOCAP_REQ_AUTH_ENABLE);
            /* indicate auth_status(success) */
            bz_auth_handle_auth_completion(ce_index, HCI_COMMAND_SUCCEEDED);
            return;
        default:
            /* send LMP not accepted. */
            bz_auth_send_not_accepted_pdu(ce_index, LMP_AU_RAND_OPCODE,
                    tid, PDU_NOT_ALLOWED_ERROR);
            return;
    }
    bz_auth_perform_super_state_transition(auth,
            HCI_AUTH_REQ_OR_AURAND_INRAND_OR_IOCAP_REQ_AUTH_ENABLE);
}


/**
 * Handles lmp_not_accepted(lmp_au_rand) PDU from the remote device.
 *
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \param can_free_pdu Set to TRUE, if the PDU can be freed by the caller.
 *                     FALSE, otherwise.
 *
 * \return None.
 */
void bz_auth_handle_au_rand_not_accepted_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index, OUT BOOLEAN* can_free_pdu)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    UCHAR lmp_reason;

    *can_free_pdu = TRUE;
    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;
    lmp_reason = lmp_pdu_ptr->payload_content[2];

    switch (auth->sub_state)
    {
        case INTR_RESP_AWAIT_AU_RAND_PDU_NOT_ACCEPTED:
            bz_auth_return_if_remote_initiated_transaction;

            /* Assume we have not received the remote initiated AU_RAND */
            bz_auth_transition_to_sub_state(auth, INTR_RESP_AWAIT_CHALLENGE);
            if (auth->pending_pdu)
            {
                UCHAR local_can_free_pdu;

                /* It seems that we have already received the REMOTE AU_RAND,
                 * so let's handle it now.
                 */
                bz_auth_handle_au_rand_pdu(auth->pending_pdu, ce_index,
                        &local_can_free_pdu);
                bz_auth_free_pending_pdu(auth);
            }
            return;
        case INTR_AWAIT_AU_RAND_PDU_NOT_ACCEPTED:
            bz_auth_return_if_remote_initiated_transaction;
            if (bz_auth_get_pdu_opcode(auth->pending_pdu) == LMP_IN_RAND_OPCODE)
            {
                bz_auth_generate_pin_code_request_event(ce_index);
                bz_auth_transition_to_sub_state(auth, INTR_PIN_CODE_REQUESTED);
            }
            if (bz_auth_get_pdu_opcode(auth->pending_pdu)
                    == LMP_IO_CAPABILITY_REQ_OPCODE)
            {
                bz_auth_generate_io_cap_request_event(ce_index);
                bz_auth_transition_to_sub_state(auth, INTR_IO_CAP_REQUESTED);
            }
            return;

        case INTR_CHALLENGED_REMOTE_HOST:
            bz_auth_return_if_remote_initiated_transaction;
            if (lmp_reason == KEY_MISSING_ERROR)
            {
                if (bz_auth_is_ssp_allowed(ce_ptr))
                {
                    /* send IO_cap_request_event to host */
                    bz_auth_generate_io_cap_request_event(ce_index);
                    bz_auth_transition_to_sub_state(auth,
                            INTR_IO_CAP_REQUESTED);
                }
                else
                {
                    /* send  HCI_PIN_Code_request event */
                    bz_auth_generate_pin_code_request_event(ce_index);
                    bz_auth_transition_to_sub_state(auth,
                            INTR_PIN_CODE_REQUESTED);
                }
                return;
            }
            else if (lmp_reason == LMP_ERROR_TRANSACTION_COLLISION_ERROR)
            {
                auth->txn_params.auth_role =
                    BZ_AUTH_ROLE_INITIATOR_AS_RESPONDER;
                bz_auth_transition_to_sub_state(auth,
                        INTR_RESP_AWAIT_CHALLENGE);
                return;
            }
            /* auth_status(failure): done at the end */
            break;
        case RESP_AWAIT_CHALLENGED_REMOTE_HOST:
            bz_auth_return_if_self_initiated_transaction;
            /* auth_status(failure): done at the end */
            break;
        default:
            BZ_ASSERT(0, "Either remote device is misbehaving or I "
                    "screwed up the state machine");
            return;
    }
    /* auth_status(failure) */
    bz_auth_handle_auth_completion(ce_index, lmp_reason);
}

/**
 * Handles lmp_not_accepted(lmp_in_rand) PDU from the remote device.
 *
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \param can_free_pdu Set to TRUE, if the PDU can be freed by the caller.
 *                     FALSE, otherwise.
 *
 * \return None.
 */
void bz_auth_handle_in_rand_not_accepted_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index, OUT BOOLEAN* can_free_pdu)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    UCHAR lmp_reason;

    *can_free_pdu = TRUE;
    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;
    lmp_reason = lmp_pdu_ptr->payload_content[2];

    switch (auth->sub_state)
    {
        case INTR_IN_RAND_PDU_SENT:
            bz_auth_return_if_remote_initiated_transaction;
            if (lmp_reason == LMP_ERROR_TRANSACTION_COLLISION_ERROR
                    && bz_auth_is_slave(ce_ptr))
            {
                auth->txn_params.auth_role =
                    BZ_AUTH_ROLE_INITIATOR_AS_RESPONDER;
                bz_auth_transition_to_sub_state(auth,
                        INTR_RESP_AWAIT_IN_RAND_PDU);
                return;
            }
            /* auth_status(failure): done at the end */
            break;
        case RESP_IN_RAND_PDU_SENT:
            bz_auth_return_if_self_initiated_transaction;
            /* auth_status(failure): done at the end */
            break;

        case INTR_RESP_AWAIT_IN_RAND_PDU_NOT_ACCEPTED:
            /* we are actually waiting for lmp_not_accepted of the transaction
             * that we had previously initiated. Since then, we have become
             * BZ_AUTH_ROLE_INITIATOR_AS_RESPONDER, so we are expecting a
             * our own TID here. I hope, you are not confused :)
             */
            bz_auth_return_if_remote_initiated_transaction;
            /* based on the pin_type, either send LMP_in_rand or LMP_accepted,
             * calculate init_key and switch to corresponding sub_state.
             */
            bz_auth_resp_send_in_rand_or_in_rand_accepted(ce_index, auth,
                    auth->pending_pdu);
            /* free the pending pdu */
            bz_auth_free_pending_pdu(auth);
            return;

        default:
            BZ_ASSERT(0, "Either remote device is misbehaving or I "
                    "screwed up the state machine");
            return;
    }

    /* auth_status(failure) */
    bz_auth_handle_auth_completion(ce_index, lmp_reason);
}

/**
 * Handles lmp_not_accepted(lmp_comb_key) PDU from the remote device.
 *
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \param can_free_pdu Set to TRUE, if the PDU can be freed by the caller.
 *                     FALSE, otherwise.
 *
 * \return None.
 */
void bz_auth_handle_comb_key_not_accepted_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index, OUT BOOLEAN* can_free_pdu)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    UCHAR lmp_reason;

    *can_free_pdu = TRUE;
    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;
    lmp_reason = lmp_pdu_ptr->payload_content[2];

    bz_auth_return_if_remote_initiated_transaction;
    switch (auth->sub_state)
    {
        case INTR_CHANGING_KEY_SENT_COMB_KEY:
            /* auth_status(failure): done at the end */
            break;

        case INTR_RESP_CHANGING_KEY_AWAIT_COMB_KEY_NOT_ACCEPTED:
            if (lmp_reason == LMP_ERROR_TRANSACTION_COLLISION_ERROR
                    && bz_auth_is_slave(ce_ptr))
            {
                /* send comb key (unit_key will not be sent here, check
                 * state), compute final link key and transition to
                 * appropriate sub_state.
                 */
                bz_auth_resp_send_unit_or_comb_key(ce_index, auth,
                        auth->pending_pdu);
                bz_auth_free_pending_pdu(auth);
                return;
            }
            /* auth_status(failure): done at the end */
            break;
        default:
            BZ_ASSERT(0, "Either remote device is misbehaving or I "
                    "screwed up the state machine");
            return;
    }

    /* auth_status(failure) */
    bz_auth_handle_auth_completion(ce_index, lmp_reason);
}

/**
 * Handles lmp_not_accepted(lmp_encryption_key_size_req) PDU
 * from the remote device.
 *
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \param can_free_pdu Set to TRUE, if the PDU can be freed by the caller.
 *                     FALSE, otherwise.
 *
 * \return None.
 */
void bz_auth_handle_encryption_key_size_req_not_accepted_pdu(
        LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index, OUT BOOLEAN* can_free_pdu)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    UCHAR lmp_reason;
    UCHAR active_tid;

    *can_free_pdu = TRUE;
    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;
    lmp_reason = lmp_pdu_ptr->payload_content[2];
    active_tid = bz_auth_get_tid_from_auth_role(ce_ptr, auth);

    bz_auth_return_if_different_transaction(active_tid);

    switch (auth->sub_state)
    {
        case ENCRYPTION_KEY_SIZE_SENT:
            /* enc_status(failure): done at the end */
            break;
        default:
            BZ_ASSERT(0, "Either remote device is misbehaving or I "
                    "screwed up the state machine");
            return;
    }

    /* enc_status(failure) */
    bz_auth_handle_enable_enc_completion(ce_index, lmp_reason);
}

/**
 * Handles lmp_not_accepted(lmp_encryption_mode_req) PDU
 * from the remote device.
 *
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \param can_free_pdu Set to TRUE, if the PDU can be freed by the caller.
 *                     FALSE, otherwise.
 *
 * \return None.
 */
void bz_auth_handle_encryption_mode_req_not_accepted_pdu(
        LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index, OUT BOOLEAN* can_free_pdu)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    UCHAR lmp_reason;
    UCHAR active_tid;
    BOOLEAN can_free_pending_pdu = TRUE;

    *can_free_pdu = TRUE;
    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;
    lmp_reason = lmp_pdu_ptr->payload_content[2];
    active_tid = bz_auth_get_tid_from_auth_role(ce_ptr, auth);

    switch (auth->sub_state)
    {
        case ENABLE_ENCRYPTION_MODE_REQ_SENT:
            bz_auth_return_if_different_transaction(active_tid);
            /* enc_status(failure) */
            bz_auth_handle_enable_enc_completion(ce_index, lmp_reason);
            break;

        case DISABLE_ENCRYPTION_MODE_REQ_SENT:
            bz_auth_return_if_different_transaction(active_tid);
            /* enc_status(failure) */
            bz_auth_handle_disable_enc_completion(ce_index, lmp_reason);
            break;

        case AWAIT_DISABLE_ENCRYPTION_MOD_REQ_NOT_ACCEPTED:
            /* we are actually waiting for lmp_not_accepted of the transaction
             * that we had previously initiated. Since then, we have become
             * BZ_AUTH_ROLE_INITIATOR_AS_RESPONDER, so we are expecting a
             * different TID from the current role. The self initiated
             * transaction is getting rejected in this case, so we expect our
             * TID here. I hope, you are not confused :)
             */
            bz_auth_return_if_remote_initiated_transaction;
            bz_auth_transition_to_sub_state(auth, AWAIT_STOP_ENCRYPTION);
            if (auth->pending_pdu != NULL)
            {
                /* process the pending pdu: lmp_stop_encryption_req */
                bz_auth_handle_stop_encryption_req_pdu(auth->pending_pdu,
                    ce_index, &can_free_pending_pdu);
                bz_auth_free_pending_pdu(auth);
            }
            return;

        case AWAIT_ENABLE_ENCRYPTION_MOD_REQ_NOT_ACCEPTED:
            /* we are actually waiting for lmp_not_accepted of the transaction
             * that we had previously initiated. Since then, we have become
             * BZ_AUTH_ROLE_INITIATOR_AS_RESPONDER, so we are expecting a
             * different TID from the current role. The self initiated
             * transaction is getting rejected in this case, so we expect our
             * TID here. I hope, you are not confused :)
             */
            bz_auth_return_if_remote_initiated_transaction;
            bz_auth_transition_to_sub_state(auth,
                    AWAIT_ENCRYPTION_KEY_SIZE_REQ);
            if (auth->pending_pdu != NULL)
            {
                /* process the pending pdu: lmp_encryption_key_size_req */
                bz_auth_handle_encryption_key_size_req_pdu(auth->pending_pdu,
                        ce_index, &can_free_pending_pdu);
                bz_auth_free_pending_pdu(auth);
            }
            return;

        default:
            BZ_ASSERT(0, "Either remote device is misbehaving or I "
                    "screwed up the state machine");
            return;
    }
}


/**
 * Handles lmp_not_accepted for security related pdus.
 *
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \param can_free_pdu Set to TRUE, if the PDU can be freed by the caller.
 *                     FALSE, otherwise.
 *
 * \return TRUE, if the PDU was handled. FALSE, otherwise.
 */
BOOLEAN bz_auth_handle_not_accepted_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index, OUT BOOLEAN* can_free_pdu)
{
    UCHAR pdu_opcode;

    pdu_opcode = lmp_pdu_ptr->payload_content[1];

    switch (pdu_opcode)
    {
        case LMP_AU_RAND_OPCODE:
            bz_auth_handle_au_rand_not_accepted_pdu(lmp_pdu_ptr,
                    ce_index, can_free_pdu);
            break;
        case LMP_IN_RAND_OPCODE:
            bz_auth_handle_in_rand_not_accepted_pdu(lmp_pdu_ptr,
                    ce_index, can_free_pdu);
            break;
        case LMP_COMB_KEY_OPCODE:
            bz_auth_handle_comb_key_not_accepted_pdu(lmp_pdu_ptr,
                    ce_index, can_free_pdu);
            break;
        case LMP_ENCRYPTION_KEY_SIZE_REQ_OPCODE:
            bz_auth_handle_encryption_key_size_req_not_accepted_pdu(
                    lmp_pdu_ptr, ce_index, can_free_pdu);
            break;
        case LMP_ENCRYPTION_MODE_REQ_OPCODE:
            bz_auth_handle_encryption_mode_req_not_accepted_pdu(
                    lmp_pdu_ptr, ce_index, can_free_pdu);
            break;
        case LMP_SIMPLE_PAIRING_CONFIRM_OPCODE:
            bz_auth_handle_simple_pairing_confirm_not_accepted_pdu(
                    lmp_pdu_ptr, ce_index, can_free_pdu);
            break;
        case LMP_SIMPLE_PAIRING_NUMBER_OPCODE:
            bz_auth_handle_simple_pairing_number_not_accepted_pdu(
                    lmp_pdu_ptr, ce_index, can_free_pdu);
            break;
        case LMP_DHKEY_CHECK_OPCODE:
            bz_auth_handle_dhkey_check_not_accepted_pdu(
                    lmp_pdu_ptr, ce_index, can_free_pdu);
            break;
#ifdef COMPILE_BROADCAST_ENCRYPTION
        case LMP_TEMP_RAND_OPCODE:
        case LMP_TEMP_KEY_OPCODE:
            bz_auth_handle_temp_pdu_not_accepted_pdu(lmp_pdu_ptr,
                    ce_index, can_free_pdu);
            break;
        case LMP_USE_SEMI_PERMANENT_KEY_OPCODE:
            bz_auth_handle_use_semi_permanent_key_not_accepted_pdu(lmp_pdu_ptr,
                    ce_index, can_free_pdu);
            break;
#endif /* COMPILE_BROADCAST_ENCRYPTION */
        default:
            return FALSE;
    }
    return TRUE;
}

/**
 * Handles lmp_in_rand PDU from the remote device.
 *
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \param can_free_pdu Set to TRUE, if the PDU can be freed by the caller.
 *                     FALSE, otherwise.
 *
 * \return None.
 */
void bz_auth_handle_in_rand_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index, OUT BOOLEAN* can_free_pdu)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    UCHAR lmp_error_code = HCI_COMMAND_SUCCEEDED;
    LMP_TRAN_ID tid;

    *can_free_pdu = TRUE;
    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;
    tid = (LMP_TRAN_ID)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr);

    switch (auth->sub_state)
    {
        case INTR_LINK_KEY_REQUESTED:
        case INTR_PIN_CODE_REQUESTED:
            /* First PDU, No SSP
             *	Store it.
             * else send LMP_not_accepted.
             */
            if (bz_auth_is_self_initiated(ce_ptr, lmp_pdu_ptr)
                    || auth->pending_pdu != NULL
                    || bz_auth_is_ssp_allowed(ce_ptr))
            {
                lmp_error_code = PDU_NOT_ALLOWED_ERROR;
                break;
            }
            bz_auth_set_pending_pdu(auth, lmp_pdu_ptr, can_free_pdu);
            return;
        case INTR_CHALLENGED_REMOTE_HOST:
            /* First PDU, No SSP
             *	Store it.
             * else send LMP_not_accepted.
             */
            if (bz_auth_is_self_initiated(ce_ptr, lmp_pdu_ptr)
                    || auth->pending_pdu != NULL
                    || bz_auth_is_ssp_allowed(ce_ptr))
            {
                lmp_error_code = PDU_NOT_ALLOWED_ERROR;
                break;
            }
            bz_auth_set_pending_pdu(auth, lmp_pdu_ptr, can_free_pdu);
            bz_auth_transition_to_sub_state(auth,
                    INTR_AWAIT_AU_RAND_PDU_NOT_ACCEPTED);
            return;

        case INTR_IN_RAND_PDU_SENT:
            if (bz_auth_is_remote_initiated(ce_ptr, lmp_pdu_ptr))
            {
                if (bz_auth_is_slave(ce_ptr))   /* are we slave? */
                {
                    bz_auth_set_pending_pdu(auth, lmp_pdu_ptr, can_free_pdu);
                    auth->txn_params.auth_role =
                        BZ_AUTH_ROLE_INITIATOR_AS_RESPONDER;
                    bz_auth_transition_to_sub_state(auth,
                            INTR_RESP_AWAIT_IN_RAND_PDU_NOT_ACCEPTED);
                    return;
                }
                else
                {
                    /* master, same collision */
                    lmp_error_code = LMP_ERROR_TRANSACTION_COLLISION_ERROR;
                }
                break;
            }
            if (bz_auth_is_fixed_pin())     /* do we have fixed pin? */
            {
                /* send LMP_not_accepted(pairing not allowed) */
                lmp_error_code = PAIRING_NOT_ALLOWED_ERROR;
                bz_auth_send_not_accepted_pdu(ce_index, LMP_IN_RAND_OPCODE,
                    tid, lmp_error_code);
                /*
                 * auth_status(failure, pairing not allowed)
                 */
                bz_auth_handle_auth_completion(ce_index, lmp_error_code);
            }
            else
            {
                /* send LMP_accepted and compute init_key */
                bz_auth_send_in_rand_accepted_pdu(ce_index, auth, lmp_pdu_ptr);

                /* Send unit_key or comb_key based on key type */
                bz_auth_intr_send_unit_or_comb_key(ce_index, auth);
            }
            return;

        case INTR_RESP_AWAIT_IN_RAND_PDU:
            bz_auth_break_if_self_initiated_transaction;

            /* based on the pin_type, either send LMP_in_rand or LMP_accepted,
             * calculate init_key and switch to corresponding sub_state.
             */
            bz_auth_resp_send_in_rand_or_in_rand_accepted(ce_index, auth,
                    lmp_pdu_ptr);
            return;

        case BZ_AUTH_SUB_STATE_IDLE:
            break;

        default:
    	    LMP_LOG_INFO(LOG_LEVEL_HIGH, SOFT_ASSERT_COLON_DIFFERENT_TRANSACTION, 0, 0);
            lmp_error_code = PDU_NOT_ALLOWED_ERROR;
            break;
    }

    bz_auth_return_if_error_or_self_initiated_transaction(LMP_IN_RAND_OPCODE);

    lmp_error_code = PDU_NOT_ALLOWED_ERROR;
    switch (auth->super_state)
    {
        case AUTHENTICATED:
        case ENCRYPTED:
        case UNAUTHENTICATED_DURING_CONN:
        case UNAUTHENTICATED:
            if (bz_auth_is_ssp_allowed(ce_ptr))
            {
                bz_auth_send_not_accepted_pdu(ce_index, LMP_IN_RAND_OPCODE,
                    tid, PDU_NOT_ALLOWED_ERROR);
                return;
            }
            auth->txn_params.auth_role = BZ_AUTH_ROLE_RESPONDER;
            bz_auth_set_pending_pdu(auth, lmp_pdu_ptr, can_free_pdu);
            /* send HCI_PIN_Code_request */
            bz_auth_generate_pin_code_request_event(ce_index);
            bz_auth_transition_to_sub_state(auth, RESP_PIN_CODE_REQUESTED);
            break;

        case TEMP_AUTHENTICATED:
        case TEMP_ENCRYPTED:
            /* send LMP_not accepted(pairing not allowed) */
            lmp_error_code = PAIRING_NOT_ALLOWED_ERROR;
        default:
            /* send LMP not accepted. */
            bz_auth_send_not_accepted_pdu(ce_index, LMP_IN_RAND_OPCODE,
                tid, lmp_error_code);
            return;
    }
    bz_auth_perform_super_state_transition(auth,
            HCI_AUTH_REQ_OR_AURAND_INRAND_OR_IOCAP_REQ_AUTH_ENABLE);
}

/**
 * Handles lmp_sres PDU from the remote device.
 *
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \param can_free_pdu Set to TRUE, if the PDU can be freed by the caller.
 *                     FALSE, otherwise.
 *
 * \return None.
 */
void bz_auth_handle_sres_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index, OUT BOOLEAN* can_free_pdu)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    UCHAR lmp_error_code = PDU_NOT_ALLOWED_ERROR;
    LMP_TRAN_ID tid;

    *can_free_pdu = TRUE;
    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;
    tid = (LMP_TRAN_ID)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr);
#ifdef _SECURE_CONN_TEST_LOG
RT_BT_LOG(YELLOW, DAPE_TEST_LOG522, 2,auth->sub_state, auth->len_prime);
#endif
    switch (auth->sub_state)
    {
#ifdef SECURE_CONN_MUTUAL_AUTH
        case SECURE_CONN_MAS_AWAIT_SRES:
            memcpy(&auth->txn_params.remote_sres[0],
                    &lmp_pdu_ptr->payload_content[1], BZ_AUTH_SRES_SIZE);

            bz_auth_send_sres_pdu(ce_index, auth, auth->pending_pdu);
            bz_auth_free_pending_pdu(auth);
         			bz_auth_verify_challenge(ce_index, auth, FALSE);
            return;
        case SECURE_CONN_SLV_AWAIT_SRES:
            memcpy(&auth->txn_params.remote_sres[0],
                    &lmp_pdu_ptr->payload_content[1], BZ_AUTH_SRES_SIZE);
            bz_auth_free_pending_pdu(auth);
         			bz_auth_verify_challenge(ce_index, auth, FALSE);
            return;
            
        case SECURE_CONN_INTR_AWAIT_SRES:
            memcpy(&auth->txn_params.remote_sres[0],
                    &lmp_pdu_ptr->payload_content[1], BZ_AUTH_SRES_SIZE);
	
            if (bz_auth_is_master(ce_ptr))
            {
                bz_auth_send_sres_pdu(ce_index, auth, auth->pending_pdu);
                bz_auth_free_pending_pdu(auth);
            }
            bz_auth_verify_challenge(ce_index, auth, TRUE);
            return;        
#endif
        case INTR_CHALLENGED_REMOTE_HOST:
            bz_auth_break_if_remote_initiated_transaction;
            memcpy(&auth->txn_params.remote_sres[0],
                    &lmp_pdu_ptr->payload_content[1], BZ_AUTH_SRES_SIZE);
			bz_auth_verify_challenge(ce_index, auth, FALSE);
            return;

        case RESP_AWAIT_CHALLENGED_REMOTE_HOST: 
            bz_auth_break_if_self_initiated_transaction;
            memcpy(&auth->txn_params.remote_sres[0],
                    &lmp_pdu_ptr->payload_content[1], BZ_AUTH_SRES_SIZE);
			
			bz_auth_verify_challenge(ce_index, auth, TRUE);
            return;

        case INTR_CHALLENGED_REMOTE_HOST_MUTUAL:

#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8723_RCP_
            if (rcp_bz_auth_handle_sres_pdu_subcase != NULL)
            {
                if ( rcp_bz_auth_handle_sres_pdu_subcase((void *)&ce_index, auth, lmp_pdu_ptr) )
                {
                     return;
                }
            }    
#endif
#endif

            bz_auth_break_if_remote_initiated_transaction;

            //RT_BT_LOG(YELLOW, CCH_DBG_100, 1,ce_ptr->remote_dev_role);
            //RT_BT_LOG(YELLOW, CCH_DBG_100, 1,((UCHAR)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr)));

            /* store remote sres */
            memcpy(&auth->txn_params.remote_sres[0],
                    &lmp_pdu_ptr->payload_content[1], BZ_AUTH_SRES_SIZE);
            bz_auth_transition_to_sub_state(auth, INTR_AWAIT_REMOTE_CHALLENGE);
            return;

        default:
            BZ_ASSERT(0, "Bug in the state machine or remote device is "
                    "misbehaving");
            break;
    }

	/* send LMP not accepted. */
    bz_auth_send_not_accepted_pdu(ce_index, LMP_SRES_OPCODE,
            tid, lmp_error_code);
}

/**
 * Handles lmp_comb_key PDU from the remote device.
 *
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \param can_free_pdu Set to TRUE, if the PDU can be freed by the caller.
 *                     FALSE, otherwise.
 *
 * \return None.
 */
void bz_auth_handle_comb_key_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index, OUT BOOLEAN* can_free_pdu)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    UCHAR lmp_error_code = HCI_COMMAND_SUCCEEDED;
    LMP_TRAN_ID tid;

    *can_free_pdu = TRUE;
    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;
    tid = (LMP_TRAN_ID)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr);

    switch (auth->sub_state)
    {
        case INTR_SENT_COMB_KEY:
            bz_auth_break_if_remote_initiated_transaction;
            
            /* compute final key */
            bz_auth_compute_final_key_from_pdu(ce_index, auth,
                    LMP_COMB_KEY_OPCODE, lmp_pdu_ptr);
            /*  send LMP_au_rand  */
            bz_auth_send_au_rand_pdu(ce_index, auth, tid);
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
            return;
        case INTR_SENT_UNIT_KEY:
            bz_auth_break_if_remote_initiated_transaction;
		
            /* compute final key */
            bz_auth_compute_final_key_from_pdu(ce_index, auth,
                    LMP_UNIT_KEY_OPCODE, lmp_pdu_ptr);
#if 0
            memcpy(&auth->txn_params.link_key[0], bz_auth_dev_params.unit_key,
                    BZ_AUTH_LINK_KEY_SIZE);
#endif
            /*  send LMP_au_rand  */
            bz_auth_send_au_rand_pdu(ce_index, auth, tid);
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
            return;
        case INTR_CHANGING_KEY_SENT_COMB_KEY:
            /* different TID */
            if (bz_auth_is_remote_initiated(ce_ptr, lmp_pdu_ptr))
            {
                if (bz_auth_is_master(ce_ptr))  /* are we master */
                {
                    /* send LMP_not accepted same collision */
                    lmp_error_code = LMP_ERROR_TRANSACTION_COLLISION_ERROR;
                    break;
                }
                else
                {
                    auth->txn_params.auth_role =
                        BZ_AUTH_ROLE_INITIATOR_AS_RESPONDER;
                    bz_auth_set_pending_pdu(auth, lmp_pdu_ptr, can_free_pdu);
                    bz_auth_transition_to_sub_state(auth,
                            INTR_RESP_CHANGING_KEY_AWAIT_COMB_KEY_NOT_ACCEPTED);
                    return;
                }
            }
            /* compute final key */
            bz_auth_compute_final_key_from_pdu(ce_index, auth,
                    LMP_COMB_KEY_OPCODE, lmp_pdu_ptr);
            /*  send LMP_au_rand  */
            bz_auth_send_au_rand_pdu(ce_index, auth, tid);
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
            return;

        case RESP_AWAIT_UNIT_OR_COMB_KEY:
            bz_auth_break_if_self_initiated_transaction;

            /* send comb key or unit key based on local key type,
             * compute the final key and transition to corresponding sub_state.
             */
            bz_auth_resp_send_unit_or_comb_key(ce_index, auth, lmp_pdu_ptr);
            return;

        case BZ_AUTH_SUB_STATE_IDLE:
            break;

        default:
			LMP_LOG_INFO(LOG_LEVEL_HIGH, SOFT_ASSERT_COLON_DIFFERENT_TRANSACTION, 0, 0);
            lmp_error_code = PDU_NOT_ALLOWED_ERROR;
            break;
    }

    bz_auth_return_if_error_or_self_initiated_transaction(LMP_COMB_KEY_OPCODE);

    lmp_error_code = PDU_NOT_ALLOWED_ERROR;
    switch(auth->super_state)
    {
        case AUTHENTICATED:
        case ENCRYPTED:
            if (bz_auth_is_link_using_unit_key(auth))
            {
                bz_auth_send_not_accepted_pdu(ce_index, LMP_COMB_KEY_OPCODE,
                    tid, PDU_NOT_ALLOWED_ERROR);
                return;
            }

            /* Initialize the transaction params with required information:
             * Since the comb_key/unit_key has to be protected with link_key,
             * we must copy the current link key to txn_params.init_key
             * (during pairing we use init_key as current link key and so all
             * the link key creation procedures expect current key in init_key
             * only).
             */
            memcpy(&auth->txn_params.init_key[0], auth->link_key,
                    BZ_AUTH_LINK_KEY_SIZE);

            auth->txn_params.auth_role = BZ_AUTH_ROLE_RESPONDER;
            /* Perform the super state transition first... it is used to
             * determine the link key type.
             */
            bz_auth_perform_super_state_transition(auth,
                    HCI_CHANGE_CONN_LINK_KEY_LMP_UNIT_COMB_KEY);
            /* send comb key (unit key will not be sent here),
             * compute the final key and transition to corresponding sub_state.
             */
            bz_auth_resp_send_unit_or_comb_key(ce_index, auth, lmp_pdu_ptr);
            break;

        default:
            /* send LMP not accepted. */
            bz_auth_send_not_accepted_pdu(ce_index, LMP_COMB_KEY_OPCODE,
                tid, lmp_error_code);
            break;
    }
}

/**
 * Handles lmp_unit_key PDU from the remote device.
 *
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \param can_free_pdu Set to TRUE, if the PDU can be freed by the caller.
 *                     FALSE, otherwise.
 *
 * \return None.
 */
void bz_auth_handle_unit_key_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index, OUT BOOLEAN* can_free_pdu)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    UCHAR lmp_error_code = PDU_NOT_ALLOWED_ERROR;
    LMP_TRAN_ID tid;

    *can_free_pdu = TRUE;
    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;
    tid = (LMP_TRAN_ID)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr);

    switch(auth->sub_state)
    {
        case INTR_SENT_COMB_KEY:
            bz_auth_break_if_remote_initiated_transaction;
			
            /* compute final key */
            bz_auth_compute_final_key_from_pdu(ce_index, auth,
                    LMP_COMB_KEY_OPCODE, lmp_pdu_ptr);
            /*  send LMP_au_rand  */
            bz_auth_send_au_rand_pdu(ce_index, auth, tid);
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
            return;
        case INTR_SENT_UNIT_KEY:
            bz_auth_break_if_remote_initiated_transaction;
			
            /* compute final key */
            bz_auth_compute_final_key_from_pdu(ce_index, auth,
                    LMP_UNIT_KEY_OPCODE, lmp_pdu_ptr);
            /*  send LMP_au_rand  */
            bz_auth_send_au_rand_pdu(ce_index, auth, tid);
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
            return;

        case RESP_AWAIT_UNIT_OR_COMB_KEY:
            bz_auth_break_if_self_initiated_transaction;

            /* send comb key or unit key based on local key type,
             * compute the final key and transition to corresponding sub_state.
             */
            bz_auth_resp_send_unit_or_comb_key(ce_index, auth, lmp_pdu_ptr);
            return;

        default:
            BZ_ASSERT(0, "Bug in the state machine or remote device is "
                    "misbehaving");
            break;
    }

    bz_auth_send_not_accepted_pdu(ce_index, LMP_UNIT_KEY_OPCODE,
                tid, lmp_error_code);
}

/**
 * Handles lmp_encryption_mode_req PDU from the remote device.
 *
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \param can_free_pdu Set to TRUE, if the PDU can be freed by the caller.
 *                     FALSE, otherwise.
 *
 * \return None.
 */
void bz_auth_handle_encryption_mode_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index, OUT BOOLEAN* can_free_pdu)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    UCHAR lmp_error_code = HCI_COMMAND_SUCCEEDED;
    LMP_TRAN_ID tid;
    UCHAR encryption_mode;
    UCHAR active_tid;

    *can_free_pdu = TRUE;
    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;
    tid = (LMP_TRAN_ID)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr);
    encryption_mode = lmp_pdu_ptr->payload_content[1];

    LMP_LOG_INFO(LOG_LEVEL_HIGH, HANDLE_ENC_MODE_REQ, 4, 
        auth->super_state, auth->sub_state, 
        auth->txn_params.auth_role, ce_ptr->remote_dev_role);
    
    if (encryption_mode > 2)
    {
        bz_auth_send_not_accepted_pdu(ce_index, LMP_ENCRYPTION_MODE_REQ_OPCODE,
                tid, INVALID_LMP_PARAMETERS_ERROR);
        return;
    }
#ifndef _TEST_NO_REFUSE_ENC_OFF      
#ifdef _SUPPORT_SECURE_CONNECTION_
#ifdef _SECURE_CONN_TEST_LOG
RT_BT_LOG(YELLOW, DAPE_TEST_LOG525, 6,auth->secure_conn_enabled, 
encryption_mode, auth->len_prime,0,0,0);
#endif

    if ((auth->secure_conn_enabled) && (encryption_mode == 0))
   
    {
        bz_auth_send_not_accepted_pdu(ce_index, LMP_ENCRYPTION_MODE_REQ_OPCODE,
                tid, ENCRYPTION_MODE_NOT_ACCEPTABLE_ERROR);
        return;
    }
#endif
#endif
    //// dape: if encryption_mode >=1, encryption_mode = 1.
    encryption_mode = (UCHAR)(encryption_mode ? 1: 0);
    active_tid = bz_auth_get_tid_from_auth_role(ce_ptr, auth);
    switch (auth->sub_state)
    {
        case BZ_AUTH_SUB_STATE_IDLE:
            break;

        case ENABLE_ENCRYPTION_MODE_REQ_SENT:
            if (encryption_mode == BZ_AUTH_ENCRYPTION_MODE_OFF)
            {
                lmp_error_code = PDU_NOT_ALLOWED_ERROR;
                break;
            }
            bz_auth_break_if_same_transaction(active_tid);
            if (bz_auth_is_master(ce_ptr))
            {
                lmp_error_code = LMP_ERROR_TRANSACTION_COLLISION_ERROR;
            }
            else
            {
                auth->txn_params.auth_role =
                    BZ_AUTH_ROLE_INITIATOR_AS_RESPONDER;
                bz_auth_send_accepted_pdu(ce_index,
                        LMP_ENCRYPTION_MODE_REQ_OPCODE, tid);
                bz_auth_transition_to_sub_state(auth,
                        AWAIT_ENABLE_ENCRYPTION_MOD_REQ_NOT_ACCEPTED);
                return;
            }
            break;

        case AWAIT_ENABLE_ENCRYPTION_MODE_REQ:
            if (encryption_mode == BZ_AUTH_ENCRYPTION_MODE_OFF)
            {
                lmp_error_code = PDU_NOT_ALLOWED_ERROR;
                break;
            }
            /* send lmp_accepted. if master send lmp_encryption_key_size */
            bz_auth_send_response_to_enable_encryption_mode_req_pdu(ce_index,
                    auth, lmp_pdu_ptr);
            return;

        case AWAIT_DISABLE_ENCRYPTION_MODE_REQ:
            if (encryption_mode == BZ_AUTH_ENCRYPTION_MODE_ON)
            {
                lmp_error_code = PDU_NOT_ALLOWED_ERROR;
                break;
            }
            /* send lmp_accepted. if master send lmp_encryption_key_size */
            bz_auth_send_response_to_disable_encryption_mode_req_pdu(ce_index,
                    auth, lmp_pdu_ptr);
            return;
        case DISABLE_ENCRYPTION_MODE_REQ_SENT:
            if (encryption_mode == BZ_AUTH_ENCRYPTION_MODE_ON)
            {
                lmp_error_code = PDU_NOT_ALLOWED_ERROR;
                break;
            }
            bz_auth_break_if_same_transaction(active_tid);
            if (bz_auth_is_master(ce_ptr))
            {
                lmp_error_code = LMP_ERROR_TRANSACTION_COLLISION_ERROR;
            }
            else
            {
                auth->txn_params.auth_role =
                    BZ_AUTH_ROLE_INITIATOR_AS_RESPONDER;
                bz_auth_send_accepted_pdu(ce_index,
                        LMP_ENCRYPTION_MODE_REQ_OPCODE, tid);
                bz_auth_transition_to_sub_state(auth,
                        AWAIT_DISABLE_ENCRYPTION_MOD_REQ_NOT_ACCEPTED);
                return;
            }
            break;

        default:
            LMP_LOG_INFO(LOG_LEVEL_HIGH, SOFT_ASSERT_COLON_DIFFERENT_TRANSACTION, 0, 0);
            lmp_error_code = PDU_NOT_ALLOWED_ERROR;
            break;
    }

    bz_auth_return_if_error_or_self_initiated_transaction(
            LMP_ENCRYPTION_MODE_REQ_OPCODE);

    switch (auth->super_state)
    {
        case AUTHENTICATED_DURING_CONN:
        case AUTHENTICATED:
        case TEMP_AUTHENTICATED:
            if (encryption_mode == BZ_AUTH_ENCRYPTION_MODE_OFF)
            {
                lmp_error_code = PDU_NOT_ALLOWED_ERROR;
                break;
            }
            auth->txn_params.auth_role = BZ_AUTH_ROLE_RESPONDER;
            /* send lmp_accepted. if master also send lmp_encryption_key_size */
            bz_auth_send_response_to_enable_encryption_mode_req_pdu(ce_index,
                    auth, lmp_pdu_ptr);
            break;

        case ENCRYPTED:
        case TEMP_ENCRYPTED:
            if (encryption_mode == BZ_AUTH_ENCRYPTION_MODE_ON)
            {
                lmp_error_code = PDU_NOT_ALLOWED_ERROR;
                break;
            }
            auth->txn_params.auth_role = BZ_AUTH_ROLE_RESPONDER;
            /* send lmp_accepted. if master also send lmp_stop_encryption_req */
            bz_auth_send_response_to_disable_encryption_mode_req_pdu(ce_index,
                    auth, lmp_pdu_ptr);
            break;

        default:
            /* send LMP not accepted, at the end of the function*/
            lmp_error_code = PDU_NOT_ALLOWED_ERROR;
            break;
    }
    if (lmp_error_code != HCI_COMMAND_SUCCEEDED)
    {
        bz_auth_send_not_accepted_pdu(ce_index, LMP_ENCRYPTION_MODE_REQ_OPCODE,
            tid, lmp_error_code);
        return;
    }
    bz_auth_perform_super_state_transition(auth,
            HCI_SET_ENC_OR_ENC_REQ_OR_AUTH_SUCC_ENC_ENABLED);
}

/**
 * Handles lmp_start_encryption_req PDU from the remote device.
 *
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \param can_free_pdu Set to TRUE, if the PDU can be freed by the caller.
 *                     FALSE, otherwise.
 *
 * \return None.
 */
void bz_auth_handle_start_encryption_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index, OUT BOOLEAN* can_free_pdu)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    UCHAR lmp_error_code = HCI_COMMAND_SUCCEEDED;
    LMP_TRAN_ID tid;
    UCHAR active_tid = SLAVE;

    *can_free_pdu = TRUE;
    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;
    tid = (LMP_TRAN_ID)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr);

    active_tid = bz_auth_get_tid_from_auth_role(ce_ptr, auth);
    switch (auth->sub_state)
    {
        case BZ_AUTH_SUB_STATE_IDLE:
            break;

        case AWAIT_START_ENCRYPTION:
#ifdef _CCH_IOT_RALINK_			
            if(!g_efuse_lps_setting_3.iot_ralink_tid_no_check )
            {    
            bz_auth_break_if_different_transaction(active_tid);
            }
#else
            bz_auth_break_if_different_transaction(active_tid);
#endif
            /* generate encryption key */
#ifdef _SUPPORT_SECURE_CONNECTION_
#ifdef _SECURE_CONN_TEST_LOG
#ifdef _CCH_SC_TEST_20130129_MRV_01
RT_BT_LOG(YELLOW, DAPE_TEST_LOG522, 2,auth->secure_conn_enabled, ce_ptr->done_h3);
#else
RT_BT_LOG(YELLOW, DAPE_TEST_LOG522, 2,auth->secure_conn_enabled, auth->len_prime);
#endif
#endif

            if(auth->secure_conn_enabled)
            {
#ifdef _CCH_SC_TEST_20130129_MRV_01
                if(ce_ptr->done_h3 == 0)
                {
	            if( lmp_connection_entity[ce_index].remote_dev_role == MASTER )
	            {       
	                lp_h3(auth->link_key, (u8*)"btak",
	                    ce_ptr->bd_addr, bz_auth_get_local_bd_addr(), auth->aco, &auth->enc_key[0]);	
	            }
                    else
                    {
                        lp_h3(auth->link_key, (u8*)"btak",
	                    bz_auth_get_local_bd_addr(), ce_ptr->bd_addr, auth->aco, &auth->enc_key[0]);
	            }
    	            ce_ptr->done_h3 = 1;
                }
#else
                if( lmp_connection_entity[ce_index].remote_dev_role == MASTER )
                {       
                    lp_h3(auth->link_key, (u8*)"btak",
                        ce_ptr->bd_addr, bz_auth_get_local_bd_addr(), auth->aco, &auth->enc_key[0]);	
                }
                else
                {
                    lp_h3(auth->link_key, (u8*)"btak",
                        bz_auth_get_local_bd_addr(), ce_ptr->bd_addr, auth->aco, &auth->enc_key[0]);
                }
#endif

#ifdef _CCH_SC_ECDH_P256_PAUSE_ENC_
                if(auth->sc_use_enc_rand)
                {
#ifdef _CCH_SC_ECDH_P256_LOG    
                    RT_BT_LOG(BLUE, CCH_DBG_155, 0, 0);
#endif
#ifdef _CCH_SC_ECDH_P256_LOG
                    RT_BT_LOG(BLUE, YL_DBG_HEX_4, 4, auth->enc_rand[0], 
                                         auth->enc_rand[1], auth->enc_rand[2], auth->enc_rand[3]);
#endif
#ifdef _CCH_SC_TEST_20130129_QCA_02
                    UCHAR temp[BZ_AUTH_ENC_RAND_SIZE];
                    memcpy(&temp[0], &auth->enc_rand[0], BZ_AUTH_ENC_RAND_SIZE);
                    bz_auth_convert_to_msb(&temp[0], 16);	
                    BB_write_sc_iv(ce_index, &temp[0]);					
#else
#ifdef _CCH_SC_TEST_20130129_QCA_04
#ifdef _CCH_SC_TEST_20130129_QCA_05
#ifdef _DAPE_TEST_CHK_SC_ROLE_SW
RT_BT_LOG(RED, DAPE_TEST_LOG293, 1,ce_ptr->remote_dev_role);
#endif

                    if(ce_ptr->remote_dev_role == MASTER )
                    {
                        BB_write_sc_iv(ce_index, &auth->enc_rand_remote[8]);
                    }
                    else
                    {
                        BB_write_sc_iv(ce_index, &auth->enc_rand[8]);
                    }
#else
                    BB_write_sc_iv(ce_index, &auth->enc_rand[8]);
#endif
#else
                    BB_write_sc_iv(ce_index, &auth->enc_rand[0]);
#endif
#endif
                }
#endif
#ifdef _SECURE_CONN_TEST_LOG
RT_BT_LOG(WHITE, DAPE_TEST_LOG526, 19,
auth->link_key[0], auth->link_key[1], auth->link_key[2],
auth->link_key[3], auth->link_key[4], auth->link_key[5],
auth->link_key[6], auth->link_key[7], auth->link_key[8],
auth->link_key[9], auth->link_key[10],auth->link_key[11],
auth->link_key[12],auth->link_key[13],auth->link_key[14],
auth->link_key[15],0,0,0);
RT_BT_LOG(WHITE, DAPE_TEST_LOG526, 19,
auth->aco[0], auth->aco[1], auth->aco[2],
auth->aco[3], auth->aco[4], auth->aco[5],
auth->aco[6], auth->aco[7], auth->aco[8],
auth->aco[9], auth->aco[10],auth->aco[11],
0,0,0,0,0,0,0);
RT_BT_LOG(WHITE, DAPE_TEST_LOG526, 19,
auth->enc_key[0], auth->enc_key[1], auth->enc_key[2],
auth->enc_key[3], auth->enc_key[4], auth->enc_key[5],
auth->enc_key[6], auth->enc_key[7], auth->enc_key[8],
auth->enc_key[9], auth->enc_key[10],auth->enc_key[11],
auth->enc_key[12],auth->enc_key[13],auth->enc_key[14],
auth->enc_key[15],0,0,0);
#endif
                
            }
            else
#endif            
            {
                if (bz_auth_is_to_use_temp_key_for_encryption(auth))
                {
                    UCHAR cof[2*LMP_BD_ADDR_SIZE];
                    memcpy(&cof[0], &bz_auth_get_remote_bd_addr(ce_ptr),
                        LMP_BD_ADDR_SIZE);
                    memcpy(&cof[LMP_BD_ADDR_SIZE], &cof[0], LMP_BD_ADDR_SIZE);
                    lp_E3(auth->link_key,
                        &lmp_pdu_ptr->payload_content[1]/* en_rand */,
                        cof, &auth->enc_key[0], auth->enc_key_size);
                }
                else
                {
                    lp_E3(auth->link_key,
                        &lmp_pdu_ptr->payload_content[1]/* en_rand */,
                        auth->aco, &auth->enc_key[0], auth->enc_key_size);
                }
            }
            /* enable encryption for the link */
            bz_auth_enable_link_level_encryption(ce_index, TRUE);
            /* Send lmp_accepted(start_encryption) as encrypted */
            bz_auth_send_accepted_pdu(ce_index,
                    LMP_START_ENCRYPTION_REQ_OPCODE, tid);
            /* Encryption Status(success) */
            bz_auth_handle_enable_enc_completion(ce_index,
                    HCI_COMMAND_SUCCEEDED);
            return;

        default:
            LMP_LOG_INFO(LOG_LEVEL_HIGH, SOFT_ASSERT_COLON_DIFFERENT_TRANSACTION, 0, 0);
            lmp_error_code = PDU_NOT_ALLOWED_ERROR;
            break;
    }
    bz_auth_return_if_error_or_self_initiated_transaction(
            LMP_START_ENCRYPTION_REQ_OPCODE);

    switch (auth->super_state)
    {
        case ENCRYPTED_PAUSED:
        case TEMP_ENCRYPTED_PAUSED:
            /* Master would expect only the LMP_resume_encryption_req, so we
             * should be slave here. We must also be supporting EPR.
             */
            BZ_ASSERT(bz_auth_is_slave(ce_ptr)
                    && bz_auth_is_epr_supported(ce_ptr),
                    "We must be Slave and support EPR here");
            if (bz_auth_is_slave(ce_ptr) && bz_auth_is_epr_supported(ce_ptr))
            {
                auth->txn_params.auth_role = BZ_AUTH_ROLE_RESPONDER;
                bz_auth_transition_to_sub_state(auth, AWAIT_START_ENCRYPTION);
                bz_auth_perform_super_state_transition(auth,
                        RESUMING_ENCRYPTION_REQ);
                bz_auth_handle_start_encryption_req_pdu(lmp_pdu_ptr, ce_index,
                        can_free_pdu);
                return;
            }
            break;

        default:
            break;
    }

    /* send LMP not accepted, at the end of the function*/
    bz_auth_send_not_accepted_pdu(ce_index, LMP_START_ENCRYPTION_REQ_OPCODE,
            tid, PDU_NOT_ALLOWED_ERROR);
}



/**
 * Handles all the security related pdus.
 *
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index  ACL connection entity index.
 *
 * \return TRUE, if the PDU was handled. FALSE, otherwise.
 */
BOOLEAN bz_auth_handle_security_pdus(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index)
{
    BOOLEAN can_free_pdu = TRUE;

    switch (bz_auth_get_pdu_opcode(lmp_pdu_ptr))
    {
        case LMP_ACCEPTED_OPCODE:
            if (!bz_auth_handle_accepted_pdu(lmp_pdu_ptr, ce_index,
                        &can_free_pdu))
            {
                return FALSE;
            }
            break;
        case LMP_NOT_ACCEPTED_OPCODE:
            if (!bz_auth_handle_not_accepted_pdu(lmp_pdu_ptr, ce_index,
                        &can_free_pdu))
            {
                return FALSE;
            }
            break;
        case LMP_AU_RAND_OPCODE:
            bz_auth_handle_au_rand_pdu(lmp_pdu_ptr, ce_index, &can_free_pdu);
            break;
        case LMP_IN_RAND_OPCODE:
            bz_auth_handle_in_rand_pdu(lmp_pdu_ptr, ce_index, &can_free_pdu);
            break;
        case LMP_SRES_OPCODE:
			bz_auth_handle_sres_pdu(lmp_pdu_ptr, ce_index, &can_free_pdu);
            break;
        case LMP_COMB_KEY_OPCODE:
            bz_auth_handle_comb_key_pdu(lmp_pdu_ptr, ce_index, &can_free_pdu);
            break;
        case LMP_UNIT_KEY_OPCODE:
            bz_auth_handle_unit_key_pdu(lmp_pdu_ptr, ce_index, &can_free_pdu);
            break;
#ifdef COMPILE_BROADCAST_ENCRYPTION
        case LMP_TEMP_RAND_OPCODE:
            bz_auth_handle_temp_rand_pdu(lmp_pdu_ptr, ce_index, &can_free_pdu);
            break;
        case LMP_TEMP_KEY_OPCODE:
            bz_auth_handle_temp_key_pdu(lmp_pdu_ptr, ce_index, &can_free_pdu);
            break;
        case LMP_USE_SEMI_PERMANENT_KEY_OPCODE:
            bz_auth_handle_use_semi_permanent_key_pdu(lmp_pdu_ptr, ce_index,
                    &can_free_pdu);
            break;
#endif /* COMPILE_BROADCAST_ENCRYPTION */
        case LMP_ENCRYPTION_MODE_REQ_OPCODE:
            bz_auth_handle_encryption_mode_req_pdu(lmp_pdu_ptr,
                    ce_index, &can_free_pdu);
            break;
        case LMP_ENCRYPTION_KEY_SIZE_REQ_OPCODE:
            bz_auth_handle_encryption_key_size_req_pdu(lmp_pdu_ptr,
                    ce_index, &can_free_pdu);
            break;
        case LMP_START_ENCRYPTION_REQ_OPCODE:
            bz_auth_handle_start_encryption_req_pdu(lmp_pdu_ptr,
                    ce_index, &can_free_pdu);
            break;
        case LMP_STOP_ENCRYPTION_REQ_OPCODE:
            bz_auth_handle_stop_encryption_req_pdu(lmp_pdu_ptr,
                    ce_index, &can_free_pdu);
            break;
        case LMP_ENCRYPTION_KEY_SIZE_MASK_REQ_OPCODE:
            bz_auth_handle_encryption_key_size_mask_req_pdu(lmp_pdu_ptr,
                    ce_index, &can_free_pdu);
            break;
        case LMP_ENCRYPTION_KEY_SIZE_MASK_RES_OPCODE:
            bz_auth_handle_encryption_key_size_mask_res_pdu(lmp_pdu_ptr,
                    ce_index, &can_free_pdu);
            break;
        case LMP_SIMPLE_PAIRING_CONFIRM_OPCODE:
            bz_auth_handle_simple_pairing_confirm_pdu(lmp_pdu_ptr,
                    ce_index, &can_free_pdu);
            break;
        case LMP_SIMPLE_PAIRING_NUMBER_OPCODE:
            bz_auth_handle_simple_pairing_number_pdu(lmp_pdu_ptr,
                    ce_index, &can_free_pdu);
            break;
        case LMP_DHKEY_CHECK_OPCODE:
            bz_auth_handle_dhkey_check_pdu(lmp_pdu_ptr,
                    ce_index, &can_free_pdu);
            break;
#ifdef _SUPPORT_SECURE_CONNECTION_            
        case LMP_PAUSE_ENCRYPTION_AES_REQ_OPCODE:
            bz_auth_handle_pause_encryption_req_pdu(lmp_pdu_ptr,
                ce_index, &can_free_pdu);
            break;
#endif            
        case LMP_ESCAPE4_OPCODE:
            switch (bz_auth_get_ext_pdu_opcode(lmp_pdu_ptr))
            {
                case LMP_ACCEPTED_EXT_OPCODE:
                    if (!bz_auth_handle_accepted_ext_pdu(lmp_pdu_ptr,
                                ce_index, &can_free_pdu))
                    {
                        return FALSE;
                    }
                    break;
                case LMP_NOT_ACCEPTED_EXT_OPCODE:
                    if (!bz_auth_handle_not_accepted_ext_pdu(lmp_pdu_ptr,
                                ce_index, &can_free_pdu))
                    {
                        return FALSE;
                    }
                    break;
                case LMP_PAUSE_ENCRYPTION_REQ_OPCODE:
                    bz_auth_handle_pause_encryption_req_pdu(lmp_pdu_ptr,
                                ce_index, &can_free_pdu);
                    break;
                case LMP_RESUME_ENCRYPTION_REQ_OPCODE:
                    bz_auth_handle_resume_encryption_req_pdu(lmp_pdu_ptr,
                                ce_index, &can_free_pdu);
                    break;
                case LMP_IO_CAPABILITY_REQ_OPCODE:
                    bz_auth_handle_io_capability_req_pdu(lmp_pdu_ptr,
                                ce_index, &can_free_pdu);
                    break;
                case LMP_IO_CAPABILITY_RES_OPCODE:
                    bz_auth_handle_io_capability_res_pdu(lmp_pdu_ptr,
                               ce_index, &can_free_pdu);
                    break;
                case LMP_NUMERIC_COMPARISON_FAILED_OPCODE:
                    bz_auth_handle_numeric_comparison_failed_pdu(lmp_pdu_ptr,
                               ce_index, &can_free_pdu);
                    break;
                case LMP_PASSKEY_FAILED_OPCODE:
                    bz_auth_handle_passkey_failed_pdu(lmp_pdu_ptr,
                               ce_index, &can_free_pdu);
                    break;
                case LMP_OOB_FAILED_OPCODE:
                    bz_auth_handle_oob_failed_pdu(lmp_pdu_ptr,
                               ce_index, &can_free_pdu);
                    break;
                case LMP_KEYPRESS_NOTIFICATION_OPCODE:
                    bz_auth_handle_keypress_notification_pdu(lmp_pdu_ptr,
                               ce_index, &can_free_pdu);
                    break;
#ifdef _SUPPORT_SECURE_CONNECTION_
                case LMP_PING_REQ_OPCODE:
                    bz_auth_handle_ping_req_pdu(lmp_pdu_ptr,
                               ce_index, &can_free_pdu);
                    break;
                case LMP_PING_RES_OPCODE:
                    bz_auth_handle_ping_res_pdu(lmp_pdu_ptr,
                               ce_index, &can_free_pdu);
                    break;
#endif
                default:
                    return FALSE;
            }
            break;
        default:
            return FALSE;
    }
    if (can_free_pdu == TRUE)
    {
        OS_FREE_BUFFER(lmp_pdu_buffer_pool_handle, lmp_pdu_ptr);
    }
    return TRUE;
}


