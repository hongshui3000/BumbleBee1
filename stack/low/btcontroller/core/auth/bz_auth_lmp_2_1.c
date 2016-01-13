/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file bz_auth_lmp_2_1.c
 *  BlueWiz Authentication module 2.1 LMP interface implementation. It
 *  contains the handlers for 2.1 authentication related LMP PDUs.
 *
 * \author Santhosh kumar M
 * \date 2007-08-10
 */
/********************************* Logger *************************/ 
enum { __FILE_NUM__= 24 };
/********************************* Logger *************************/
/* ========================= Include File Section ========================= */
#include "bz_auth.h"
#include "lmp.h"
#include "bz_auth_internal_2_1.h"
#include "bz_debug.h"
#include "bz_auth_extern_accessors.h"
#include "bz_auth_hci.h"
#include "crypto.h"
#include "bz_auth_lmp.h"

#include "mem.h"
#ifdef _CCH_IOT_RALINK_
#include "lc.h"
#endif
/* ====================== Macro Declaration Section ======================= */


/* ==================== Structure Declaration Section ===================== */


/* ===================== Variable Declaration Section ===================== */


/* ================== Static Function Prototypes Section ================== */


/* ===================== Function Definition Section ====================== */
/**
 * Handles lmp_io_capability_req PDU from the remote device.
 *
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \param can_free_pdu Set to TRUE, if the PDU can be freed by the caller.
 *                     FALSE, otherwise.
 *
 * \return None.
 */
void bz_auth_handle_io_capability_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
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
            if (bz_auth_is_during_conn(auth))
            {
                /* during connection */
                lmp_error_code = PDU_NOT_ALLOWED_ERROR;
                break;
            }
            if (!bz_auth_is_ssp_allowed(ce_ptr))
            {
                /* no ssp support */
                lmp_error_code = SECURE_SIMPLE_PAIRING_NOT_SUPPORTED_BY_HOST;
                break;
            }
            /*@fallthrough@*/
        case INTR_IO_CAP_REQUESTED:
            if (bz_auth_is_self_initiated(ce_ptr, lmp_pdu_ptr)
                    ||	auth->pending_pdu != NULL)
            {
                /* not first PDU, and self initiated */
                lmp_error_code = PDU_NOT_ALLOWED_ERROR;
                break;
            }
            bz_auth_set_pending_pdu(auth, lmp_pdu_ptr, can_free_pdu);
            return;
        case INTR_AWAIT_REMOTE_IO_CAP_RESPONSE:
            bz_auth_break_if_self_initiated_transaction;
            if (bz_auth_is_master(ce_ptr))
            {
                lmp_error_code = LMP_ERROR_TRANSACTION_COLLISION_ERROR;
                break;
            }
            else
            {
                if (auth->pending_pdu != NULL)
                {
                    lmp_error_code = PDU_NOT_ALLOWED_ERROR;
                    break;
                }
                bz_auth_set_pending_pdu(auth, lmp_pdu_ptr, can_free_pdu);
            }
            return;

        case INTR_RESP_AWAIT_IO_CAP_REQ:
            bz_auth_extract_io_cap_data(&lmp_pdu_ptr->payload_content[2],
                &auth->txn_params.ssp_data.remote_io_cap);
            /* send HCI_IO_Capability_Response_event */
            bz_auth_generate_io_cap_response_event(ce_index,
                    (UCHAR*)&auth->txn_params.ssp_data.remote_io_cap);
            /* send LMP_io_capability_response */
            bz_auth_send_io_capability_res_pdu(ce_index, auth, tid);
            /* initiate public key exchange */
            bz_auth_initiate_public_key_exchange(ce_index, tid);
            return;
        case BZ_AUTH_SUB_STATE_IDLE:
            break;
        default:
    	    LMP_LOG_INFO(LOG_LEVEL_HIGH, SOFT_ASSERT_COLON_DIFFERENT_TRANSACTION, 0, 0);
            lmp_error_code = PDU_NOT_ALLOWED_ERROR;
            break;
    }

    bz_auth_return_if_error_or_self_initiated_transaction_ext(
            LMP_ESCAPE4_OPCODE, LMP_IO_CAPABILITY_REQ_OPCODE);

    lmp_error_code = PDU_NOT_ALLOWED_ERROR;
    switch (auth->super_state)
    {
        case UNAUTHENTICATED:
        case AUTHENTICATED:
        case ENCRYPTED:
        case TEMP_AUTHENTICATED:
        case TEMP_ENCRYPTED:
            if (bz_auth_is_ssp_allowed(ce_ptr))
            {
                auth->txn_params.auth_role = BZ_AUTH_ROLE_RESPONDER;
                /* send HCI_IO_capability_response_event */
                bz_auth_extract_io_cap_data(&lmp_pdu_ptr->payload_content[2],
                    &auth->txn_params.ssp_data.remote_io_cap);
                bz_auth_generate_io_cap_response_event(ce_index,
                        (UCHAR*)&auth->txn_params.ssp_data.remote_io_cap);

                /* send HCI_IO_capability_request_event */
                bz_auth_generate_io_cap_request_event(ce_index);
                bz_auth_transition_to_sub_state(auth, RESP_IO_CAP_REQUESTED);
                break;
            }
            else
            {
                /* no ssp support */
                lmp_error_code = SECURE_SIMPLE_PAIRING_NOT_SUPPORTED_BY_HOST;
            }
            /*@fallthrough@*/
        default:
            /* send LMP not accepted. */
            bz_auth_send_not_accepted_ext_pdu(ce_index, LMP_ESCAPE4_OPCODE,
                    LMP_IO_CAPABILITY_REQ_OPCODE, tid, lmp_error_code);
            return;
    }
    bz_auth_perform_super_state_transition(auth,
            HCI_AUTH_REQ_OR_AURAND_INRAND_OR_IOCAP_REQ_AUTH_ENABLE);
}

/**
 * Handles lmp_io_capability_res PDU from the remote device.
 *
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \param can_free_pdu Set to TRUE, if the PDU can be freed by the caller.
 *                     FALSE, otherwise.
 *
 * \return None.
 */
void bz_auth_handle_io_capability_res_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index, OUT BOOLEAN* can_free_pdu)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    UCHAR lmp_error_code = PDU_NOT_ALLOWED_ERROR;
    LMP_TRAN_ID tid;

    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;
    tid = (LMP_TRAN_ID)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr);

    switch (auth->sub_state)
    {
        case INTR_AWAIT_REMOTE_IO_CAP_RESPONSE:
            bz_auth_break_if_remote_initiated_transaction;
            bz_auth_extract_io_cap_data(&lmp_pdu_ptr->payload_content[2],
                    &auth->txn_params.ssp_data.remote_io_cap);
            /* send HCI_IO_Capability_Response_event */
            bz_auth_generate_io_cap_response_event(ce_index,
                    (UCHAR*)&auth->txn_params.ssp_data.remote_io_cap);
            /* initiate public key exchange */
            bz_auth_initiate_public_key_exchange(ce_index, tid);
            return;
        default:
            BZ_ASSERT(0, "Bug in the state machine or remote device is "
                    "misbehaving");
            break;
    }
    /* send LMP not accepted. */
    bz_auth_send_not_accepted_ext_pdu(ce_index, LMP_ESCAPE4_OPCODE,
            LMP_IO_CAPABILITY_RES_OPCODE, tid, lmp_error_code);
}

/**
 * Handles lmp_numeric_comparison_failed PDU from the remote device.
 *
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \param can_free_pdu Set to TRUE, if the PDU can be freed by the caller.
 *                     FALSE, otherwise.
 *
 * \return None.
 */
void bz_auth_handle_numeric_comparison_failed_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index, OUT BOOLEAN* can_free_pdu)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    UCHAR lmp_error_code = HCI_COMMAND_SUCCEEDED;
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
            bz_auth_send_not_accepted_ext_pdu(ce_index, LMP_ESCAPE4_OPCODE,
                    LMP_NUMERIC_COMPARISON_FAILED_OPCODE,
                    tid, PDU_NOT_ALLOWED_ERROR);
            return;
        }
    }
#else
    if (tid != active_tid)
    {
        bz_auth_send_not_accepted_ext_pdu(ce_index, LMP_ESCAPE4_OPCODE,
                LMP_NUMERIC_COMPARISON_FAILED_OPCODE,
                tid, PDU_NOT_ALLOWED_ERROR);
        return;
    }
#endif	
    switch(auth->sub_state)
    {
        case NC_RESP_USER_CONFIRM_REQUESTED:
            if (auth->pending_pdu == NULL)
            {
                bz_auth_set_pending_pdu(auth, lmp_pdu_ptr, can_free_pdu);
            }
            else
            {
                lmp_error_code = PDU_NOT_ALLOWED_ERROR;
            }
            break;
        case NC_RESP_AWAIT_DH_KEY_TO_SKIP:
        case RESP_AWAIT_DHKEY_CHECK:
            bz_auth_handle_simple_pairing_complete(ce_index,
                    AUTHENTICATION_FAILURE_ERROR);
            break;
        default:
            BZ_ASSERT(0, "Bug in the state machine or remote device is "
                    "misbehaving");
            lmp_error_code = PDU_NOT_ALLOWED_ERROR;
            break;
    }
    if (lmp_error_code != HCI_COMMAND_SUCCEEDED)
    {
        bz_auth_send_not_accepted_ext_pdu(ce_index, LMP_ESCAPE4_OPCODE,
            LMP_NUMERIC_COMPARISON_FAILED_OPCODE, tid, lmp_error_code);
    }
}

/**
 * Handles lmp_oob_failed PDU from the remote device.
 *
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \param can_free_pdu Set to TRUE, if the PDU can be freed by the caller.
 *                     FALSE, otherwise.
 *
 * \return None.
 */
void bz_auth_handle_oob_failed_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index,
        OUT BOOLEAN* can_free_pdu)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    UCHAR lmp_error_code = HCI_COMMAND_SUCCEEDED;
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
            bz_auth_send_not_accepted_ext_pdu(ce_index, LMP_ESCAPE4_OPCODE,
                    LMP_OOB_FAILED_OPCODE, tid, PDU_NOT_ALLOWED_ERROR);
            return;
        }
    }
#else
    if (tid != active_tid)
    {
        bz_auth_send_not_accepted_ext_pdu(ce_index, LMP_ESCAPE4_OPCODE,
                LMP_OOB_FAILED_OPCODE, tid, PDU_NOT_ALLOWED_ERROR);
        return;
    }
#endif	
    switch(auth->sub_state)
    {
        case OOB_RESP_REMOTE_OOB_DATA_REQUESTED:
            if (auth->pending_pdu != NULL)
            {
                lmp_error_code = PDU_NOT_ALLOWED_ERROR;
                break;
            }
            bz_auth_set_pending_pdu(auth, lmp_pdu_ptr, can_free_pdu);
            break;
        case OOB_RESP_AWAIT_SP_NUM:
        case RESP_AWAIT_DHKEY_CHECK:
            bz_auth_handle_simple_pairing_complete(ce_index,
                    AUTHENTICATION_FAILURE_ERROR);
            return;
        default:
            BZ_ASSERT(0, "Bug in the state machine or remote device is "
                    "misbehaving");
            lmp_error_code = PDU_NOT_ALLOWED_ERROR;
            break;
    }
    bz_auth_send_not_accepted_ext_pdu(ce_index, LMP_ESCAPE4_OPCODE,
            LMP_OOB_FAILED_OPCODE, tid, lmp_error_code);
}

/**
 * Handles lmp_keypress_notification PDU from the remote device.
 *
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \param can_free_pdu Set to TRUE, if the PDU can be freed by the caller.
 *                     FALSE, otherwise.
 *
 * \return None.
 */
void bz_auth_handle_keypress_notification_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index, OUT BOOLEAN* can_free_pdu)
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
            bz_auth_send_not_accepted_ext_pdu(ce_index, LMP_ESCAPE4_OPCODE,
                    LMP_KEYPRESS_NOTIFICATION_OPCODE, tid, PDU_NOT_ALLOWED_ERROR);
            return;
        }
	}
#else
    if (tid != active_tid)
    {
        bz_auth_send_not_accepted_ext_pdu(ce_index, LMP_ESCAPE4_OPCODE,
                LMP_KEYPRESS_NOTIFICATION_OPCODE, tid, PDU_NOT_ALLOWED_ERROR);
        return;
    }
#endif
    switch (auth->sub_state)
    {
        case PE_INTR_PASS_KEY_REQUESTED:
        case PE_INTR_SENT_SP_CONFIRM:
        case PE_RESP_PASSKEY_REQUESTED:
        case PE_RESP_AWAIT_SP_CONFIRM:
        case PE_RESP_AWAIT_SP_CONFIRM_TO_SKIP:
            if (lmp_pdu_ptr->payload_content[2] < 5)
            {
                bz_auth_generate_keypress_notification_event(ce_index,
                        lmp_pdu_ptr->payload_content[2]);
            }
            return;
        default:
            BZ_ASSERT(0, "Bug in the state machine or remote device is "
                    "misbehaving");
            break;
    }
    bz_auth_send_not_accepted_ext_pdu(ce_index, LMP_ESCAPE4_OPCODE,
            LMP_KEYPRESS_NOTIFICATION_OPCODE, tid, PDU_NOT_ALLOWED_ERROR);
}

/**
 * Handles lmp_passkey_failed PDU from the remote device.
 *
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \param can_free_pdu Set to TRUE, if the PDU can be freed by the caller.
 *                     FALSE, otherwise.
 *
 * \return None.
 */
void bz_auth_handle_passkey_failed_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index, OUT BOOLEAN* can_free_pdu)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    UCHAR lmp_error_code = HCI_COMMAND_SUCCEEDED;
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
            bz_auth_send_not_accepted_ext_pdu(ce_index, LMP_ESCAPE4_OPCODE,
                    LMP_PASSKEY_FAILED_OPCODE, tid, PDU_NOT_ALLOWED_ERROR);
            return;
        }
	}
#else
    if (tid != active_tid)
    {
        bz_auth_send_not_accepted_ext_pdu(ce_index, LMP_ESCAPE4_OPCODE,
                LMP_PASSKEY_FAILED_OPCODE, tid, PDU_NOT_ALLOWED_ERROR);
        return;
    }
#endif
    switch(auth->sub_state)
    {
        case PE_RESP_PASSKEY_REQUESTED:
            if (auth->pending_pdu == NULL)
            {
                bz_auth_set_pending_pdu(auth, lmp_pdu_ptr, can_free_pdu);
                return;
            }
            else
            {
                lmp_error_code = PDU_NOT_ALLOWED_ERROR;
            }
        case PE_RESP_AWAIT_SP_CONFIRM:
        case RESP_AWAIT_DHKEY_CHECK:
            bz_auth_handle_simple_pairing_complete(ce_index,
                    AUTHENTICATION_FAILURE_ERROR);
            return;
        default:
            BZ_ASSERT(0, "Bug in the state machine or remote device is "
                    "misbehaving");
            lmp_error_code = PDU_NOT_ALLOWED_ERROR;
            break;
    }
    bz_auth_send_not_accepted_ext_pdu(ce_index, LMP_ESCAPE4_OPCODE,
            LMP_PASSKEY_FAILED_OPCODE, tid, lmp_error_code);
}

/**
 * Handles lmp_not_accepted(lmp_pause_encryption_req) PDU
 * from the remote device.
 *
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \param can_free_pdu Set to TRUE, if the PDU can be freed by the caller.
 *                     FALSE, otherwise.
 *
 * \return None.
 */
void bz_auth_handle_pause_encryption_req_not_accepted_pdu(
        LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index, OUT BOOLEAN* can_free_pdu)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    UCHAR lmp_reason;
    BOOLEAN local_can_free_pdu;

    *can_free_pdu = TRUE;
    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;

    lmp_reason = lmp_pdu_ptr->payload_content[4];

    switch (auth->sub_state)
    {
        case AWAIT_PAUSE_ENCRYPTION_NOT_ACCEPTED:
            BZ_ASSERT(bz_auth_is_slave(ce_ptr), "We must be Slave here");
            bz_auth_return_if_remote_initiated_transaction;
            bz_auth_transition_to_sub_state(auth, AWAIT_STOP_ENCRYPTION);
            if (auth->pending_pdu != NULL)
            {
                bz_auth_handle_stop_encryption_req_pdu(auth->pending_pdu,
                        ce_index, &local_can_free_pdu);
                bz_auth_free_pending_pdu(auth);
            }
            return;

        default:
            bz_auth_handle_disable_enc_completion(ce_index, lmp_reason);
            return;
    }
}

/**
 * Handles lmp_not_accepted(lmp_io_capability_req) PDU
 * from the remote device.
 *
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \param can_free_pdu Set to TRUE, if the PDU can be freed by the caller.
 *                     FALSE, otherwise.
 *
 * \return None.
 */
void bz_auth_handle_io_capability_req_not_accepted_pdu(
        LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index, OUT BOOLEAN* can_free_pdu)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    UCHAR lmp_reason;
    BOOLEAN local_can_free_pdu;

    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;
    lmp_reason = lmp_pdu_ptr->payload_content[4];

    switch (auth->sub_state)
    {
        case INTR_AWAIT_REMOTE_IO_CAP_RESPONSE:
            bz_auth_return_if_remote_initiated_transaction;
            if (auth->pending_pdu != NULL)
            {
                BZ_ASSERT(bz_auth_is_slave(ce_ptr), "We must be slave here -- "
                        "Otherwise we wouldn't have stored this PDU");

                /* we are slave and received LMP_io_capability_req from the
                 * remote device.
                 */
                auth->txn_params.auth_role =
                    BZ_AUTH_ROLE_INITIATOR_AS_RESPONDER;
                bz_auth_transition_to_sub_state(auth,
                        INTR_RESP_AWAIT_IO_CAP_REQ);
                bz_auth_handle_io_capability_req_pdu(auth->pending_pdu,
                        ce_index, &local_can_free_pdu);
                bz_auth_free_pending_pdu(auth);
                return;
            }
            if (bz_auth_is_slave(ce_ptr)
                    && (lmp_reason == LMP_ERROR_TRANSACTION_COLLISION_ERROR))
            {
                auth->txn_params.auth_role =
                    BZ_AUTH_ROLE_INITIATOR_AS_RESPONDER;
                bz_auth_transition_to_sub_state(auth,
                        INTR_RESP_AWAIT_IO_CAP_REQ);
                return;
            }
            bz_auth_handle_simple_pairing_complete(ce_index, lmp_reason);
            return;
        default:
            BZ_ASSERT(0, "Bug in the state machine or remote device is "
                    "misbehaving");
            return;
    }
}

/**
 * Handles lmp_pause_encryption_req PDU from the remote device.
 *
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \param can_free_pdu Set to TRUE, if the PDU can be freed by the caller.
 *                     FALSE, otherwise.
 *
 * \return None.
 */
void bz_auth_handle_pause_encryption_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index, OUT BOOLEAN* can_free_pdu)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    UCHAR lmp_error_code = HCI_COMMAND_SUCCEEDED;
    LMP_TRAN_ID tid;
    UCHAR active_tid;

	tid = (LMP_TRAN_ID)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr);
    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;

    LMP_LOG_INFO(LOG_LEVEL_HIGH, HANDLE_PAUSE, 6, 
                        ce_index, tid,
                        auth->super_state, auth->sub_state,
                        auth->txn_params.auth_role, ce_ptr->remote_dev_role);    

#ifdef _CCH_SC_ECDH_P256_PAUSE_ENC

    if (bz_auth_get_pdu_opcode(lmp_pdu_ptr) == 
                LMP_PAUSE_ENCRYPTION_AES_REQ_OPCODE)
    {
        auth->sc_use_enc_rand = 1;    
        memcpy(&auth->enc_rand[0], &lmp_pdu_ptr->payload_content[1],
                   BZ_AUTH_ENC_RAND_SIZE);  

#ifdef _CCH_SC_TEST_20130129_QCA_03
        bz_auth_convert_to_msb(&auth->enc_rand[0], BZ_AUTH_ENC_RAND_SIZE);

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

		
#ifdef _CCH_SC_ECDH_P256_LOG
    RT_BT_LOG(BLUE, YL_DBG_HEX_16, 16, 
    auth->enc_rand[0], auth->enc_rand[1], auth->enc_rand[2], auth->enc_rand[3],
    auth->enc_rand[4], auth->enc_rand[5], auth->enc_rand[6], auth->enc_rand[7],
    auth->enc_rand[8], auth->enc_rand[9], auth->enc_rand[10], auth->enc_rand[11],
    auth->enc_rand[12], auth->enc_rand[13], auth->enc_rand[14], auth->enc_rand[15]);
#endif	
    }
	
#endif
#ifdef _SUPPORT_VER_4_1_
#ifdef _SUPPORT_PCA_ADJUST
    if (IS_BT41)
    {
        if (bt_pca_manager.pca_updating)
        {
            bz_auth_send_not_accepted_ext_pdu(ce_index,
            LMP_ESCAPE4_OPCODE,
            LMP_PAUSE_ENCRYPTION_REQ_OPCODE, tid,
            PDU_NOT_ALLOWED_ERROR);
            return;
        }    
    }
#endif
#endif

    if (!bz_auth_is_epr_supported(ce_ptr))
    {
      	LMP_LOG_INFO(LOG_LEVEL_HIGH, EPR_NOT_SUPPORTED, 0, 0);
#ifdef _SUPPORT_SECURE_CONNECTION_
        if (bz_auth_get_pdu_opcode(lmp_pdu_ptr) == 
                    LMP_PAUSE_ENCRYPTION_AES_REQ_OPCODE)
        {
            bz_auth_send_not_accepted_pdu(ce_index, 
                    LMP_PAUSE_ENCRYPTION_AES_REQ_OPCODE, tid,
                    UNSUPPORTED_REMOTE_FEATURE_ERROR);
        }
        else
#endif
        {
        bz_auth_send_not_accepted_ext_pdu(ce_index, LMP_ESCAPE4_OPCODE,
                LMP_PAUSE_ENCRYPTION_REQ_OPCODE, tid,
                UNSUPPORTED_REMOTE_FEATURE_ERROR);
        }
        return;
    }

    active_tid = bz_auth_get_tid_from_auth_role(ce_ptr, auth);
    switch (auth->sub_state)
    {
        case BZ_AUTH_SUB_STATE_IDLE:
            break;
        case AWAIT_PAUSE_ENCRYPTION:
            /* We can enter into this state,
             * (1) either if master initiated the transaction or
             * (2) if we are responder of a bigger transaction which requires
             *     pause and resume (e.g. Change_Conn_Link_Key).
             */
            if (bz_auth_is_master(ce_ptr))
            {
#ifdef _CCH_IOT_RALINK_			
                if(!g_efuse_lps_setting_3.iot_ralink_tid_no_check )
                {            
                    if (active_tid != tid)  /* Is it a transaction collision */
                    {
                        /* I am the master, don't dare to collide with me! */
#ifdef _SUPPORT_SECURE_CONNECTION_
                        if (bz_auth_get_pdu_opcode(lmp_pdu_ptr) == 
                                    LMP_PAUSE_ENCRYPTION_AES_REQ_OPCODE)
                        {
                            bz_auth_send_not_accepted_pdu(ce_index, 
                                    LMP_PAUSE_ENCRYPTION_AES_REQ_OPCODE, tid,
                                    LMP_ERROR_TRANSACTION_COLLISION_ERROR);
                        }
                        else
#endif                      
                        {
                        bz_auth_send_not_accepted_ext_pdu(ce_index,
                                LMP_ESCAPE4_OPCODE,
                                LMP_PAUSE_ENCRYPTION_REQ_OPCODE, tid,
                                LMP_ERROR_TRANSACTION_COLLISION_ERROR);
                        }
                        return;
                    }
    	        }
#else
                if (active_tid != tid)  /* Is it a transaction collision */
                {
#ifdef _SUPPORT_SECURE_CONNECTION_
                    if (bz_auth_get_pdu_opcode(lmp_pdu_ptr) == 
                                LMP_PAUSE_ENCRYPTION_AES_REQ_OPCODE)
                    {
                        bz_auth_send_not_accepted_pdu(ce_index, 
                                LMP_PAUSE_ENCRYPTION_AES_REQ_OPCODE, tid,
                                LMP_ERROR_TRANSACTION_COLLISION_ERROR);
                    }
                    else
#endif                
                    {
                    /* I am the master, don't dare to collide with me! */
                    bz_auth_send_not_accepted_ext_pdu(ce_index,
                            LMP_ESCAPE4_OPCODE,
                            LMP_PAUSE_ENCRYPTION_REQ_OPCODE, tid,
                            LMP_ERROR_TRANSACTION_COLLISION_ERROR);
                    }
                    return;
                }
#endif				
                bz_auth_send_stop_encryption_pdu(ce_index, auth, tid);
                bz_auth_transition_to_sub_state(auth, STOP_ENCRYPTION_SENT);
            }
            else
            {
                /* We are slave and we would enter this state only in case of
                 * (2) -- as explained above.
                 */
                bz_auth_send_pause_encryption_req_pdu(ce_index, auth, tid);
                bz_auth_transition_to_sub_state(auth, AWAIT_STOP_ENCRYPTION);
            }
            return;

        case AWAIT_STOP_ENCRYPTION:
            BZ_ASSERT(bz_auth_is_slave(ce_ptr), "Only slave can enter this "
                    "state");
#ifdef _CCH_IOT_RALINK_			
            if(!g_efuse_lps_setting_3.iot_ralink_tid_no_check )
            {    
                if (active_tid != tid)  /* Is it a transaction collision */
                {
                    /* Slave: I obey the Master's order */
                    bz_auth_send_pause_encryption_req_pdu(ce_index, auth, tid);
                    auth->txn_params.auth_role =
                        BZ_AUTH_ROLE_INITIATOR_AS_RESPONDER;
                    bz_auth_transition_to_sub_state(auth,
                            AWAIT_PAUSE_ENCRYPTION_NOT_ACCEPTED);
                }
                else
                {
                    BZ_ASSERT(0, "Master can't send me LMP_pause_enc_req with my "
                            "TID -- he has to send LMP_stop_enc_req :'(");
                }
            }
#else
            if (active_tid != tid)  /* Is it a transaction collision */
            {
                /* Slave: I obey the Master's order */
                bz_auth_send_pause_encryption_req_pdu(ce_index, auth, tid);
                auth->txn_params.auth_role =
                    BZ_AUTH_ROLE_INITIATOR_AS_RESPONDER;
                bz_auth_transition_to_sub_state(auth,
                        AWAIT_PAUSE_ENCRYPTION_NOT_ACCEPTED);
            }
            else
            {
                BZ_ASSERT(0, "Master can't send me LMP_pause_enc_req with my "
                        "TID -- he has to send LMP_stop_enc_req :'(");
            }
#endif			
            return;

        default:
        	LMP_LOG_INFO(LOG_LEVEL_HIGH, SOFT_ASSERT_COLON_DIFFERENT_TRANSACTION, 0, 0);
            lmp_error_code = PDU_NOT_ALLOWED_ERROR;
            break;
    }
#ifdef _SUPPORT_SECURE_CONNECTION_
    if (bz_auth_get_pdu_opcode(lmp_pdu_ptr) == 
                LMP_PAUSE_ENCRYPTION_AES_REQ_OPCODE)
    {
        bz_auth_return_if_error_or_self_initiated_transaction(
                              LMP_PAUSE_ENCRYPTION_AES_REQ_OPCODE);

    }
    else
#endif
    {
    bz_auth_return_if_error_or_self_initiated_transaction_ext(
            LMP_ESCAPE4_OPCODE, LMP_PAUSE_ENCRYPTION_REQ_OPCODE);
    }
    switch (auth->super_state)
    {
        case ENCRYPTED:
        case TEMP_ENCRYPTED:
            auth->txn_params.auth_role = BZ_AUTH_ROLE_RESPONDER;
            if (bz_auth_is_master(ce_ptr))
            {
                bz_auth_send_stop_encryption_pdu(ce_index, auth, tid);
                bz_auth_transition_to_sub_state(auth, STOP_ENCRYPTION_SENT);
            }
            else
            {
                bz_auth_send_pause_encryption_req_pdu(ce_index, auth, tid);
                bz_auth_transition_to_sub_state(auth, AWAIT_STOP_ENCRYPTION);
            }
            break;
        default:
#ifdef _SUPPORT_SECURE_CONNECTION_
            if (bz_auth_get_pdu_opcode(lmp_pdu_ptr) == 
                        LMP_PAUSE_ENCRYPTION_AES_REQ_OPCODE)
            {
                /* send LMP not accepted, at the end of the function*/
                bz_auth_send_not_accepted_pdu(ce_index,
                        LMP_PAUSE_ENCRYPTION_AES_REQ_OPCODE, tid,
                        PDU_NOT_ALLOWED_ERROR);
            }
            else
#endif                
            {
            /* send LMP not accepted, at the end of the function*/
            bz_auth_send_not_accepted_ext_pdu(ce_index, LMP_ESCAPE4_OPCODE,
                    LMP_PAUSE_ENCRYPTION_REQ_OPCODE, tid,
                    PDU_NOT_ALLOWED_ERROR);
            }
            return;
    }
    bz_auth_perform_super_state_transition(auth, PAUSE_ENCRYPTION_REQ);
}

/**
 * Handles lmp_resume_encryption_req PDU from the remote device.
 *
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \param can_free_pdu Set to TRUE, if the PDU can be freed by the caller.
 *                     FALSE, otherwise.
 *
 * \return None.
 */
void bz_auth_handle_resume_encryption_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index, OUT BOOLEAN* can_free_pdu)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    UCHAR lmp_error_code = HCI_COMMAND_SUCCEEDED;
    LMP_TRAN_ID tid;

	tid = (LMP_TRAN_ID)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr);
    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;
    if (!bz_auth_is_epr_supported(ce_ptr))
    {
        bz_auth_send_not_accepted_ext_pdu(ce_index, LMP_ESCAPE4_OPCODE,
                LMP_RESUME_ENCRYPTION_REQ_OPCODE, tid,
                UNSUPPORTED_REMOTE_FEATURE_ERROR);
        return;
    }
    switch (auth->sub_state)
    {
        case BZ_AUTH_SUB_STATE_IDLE:
            break;

        case AWAIT_RESUME_ENCRYPTION:
            BZ_ASSERT(bz_auth_is_master(ce_ptr), "Only master can receive "
                    "LMP_resume_enc_req");
            if (bz_auth_is_master(ce_ptr))
            {
                bz_auth_break_if_self_initiated_transaction;
                bz_auth_send_start_encryption_pdu(ce_index, auth, tid);
                bz_auth_transition_to_sub_state(auth, START_ENCRYPTION_SENT);
                return;
            }
            lmp_error_code = PDU_NOT_ALLOWED_ERROR;
            break;

        default:
	        LMP_LOG_INFO(LOG_LEVEL_HIGH, SOFT_ASSERT_COLON_DIFFERENT_TRANSACTION, 0, 0);
            lmp_error_code = PDU_NOT_ALLOWED_ERROR;
            break;
    }

    bz_auth_return_if_error_or_self_initiated_transaction_ext(
            LMP_ESCAPE4_OPCODE,
            LMP_RESUME_ENCRYPTION_REQ_OPCODE);

    switch (auth->super_state)
    {
        case ENCRYPTED_PAUSED:
        case TEMP_ENCRYPTED_PAUSED:
            /* Slave would expect only the LMP_start_enc_req, so we should
             * be master here.
             */
            BZ_ASSERT(bz_auth_is_master(ce_ptr), "We must be Master here");
            if (bz_auth_is_master(ce_ptr))
            {
                bz_auth_perform_super_state_transition(auth, RESUMING_ENCRYPTION_REQ);
                auth->txn_params.auth_role = BZ_AUTH_ROLE_RESPONDER;
                bz_auth_send_start_encryption_pdu(ce_index, auth, tid);
                bz_auth_transition_to_sub_state(auth, START_ENCRYPTION_SENT);
                break;
            }
            /*@fallthrough@*/
        default:
            /* send LMP not accepted, at the end of the function*/
            bz_auth_send_not_accepted_ext_pdu(ce_index, LMP_ESCAPE4_OPCODE,
                    LMP_RESUME_ENCRYPTION_REQ_OPCODE, tid,
                    PDU_NOT_ALLOWED_ERROR);
            return;
    }
}

/**
 * Handles lmp_simple_pairing_confirm PDU from the remote device.
 *
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \param can_free_pdu Set to TRUE, if the PDU can be freed by the caller.
 *                     FALSE, otherwise.
 *
 * \return None.
 */
void bz_auth_handle_simple_pairing_confirm_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index, OUT BOOLEAN* can_free_pdu)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    BZ_AUTH_SSP_DATA* ssp_data;
    UCHAR lmp_error_code = HCI_COMMAND_SUCCEEDED;
    LMP_TRAN_ID tid;
    UCHAR active_tid;

    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;
    ssp_data = &auth->txn_params.ssp_data;
    tid = (LMP_TRAN_ID)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr);

    active_tid = bz_auth_get_tid_from_auth_role(ce_ptr, auth);
#ifdef _CCH_IOT_RALINK_			
    if(!g_efuse_lps_setting_3.iot_ralink_tid_no_check )
    {
        if (tid != active_tid)
        {
            bz_auth_send_not_accepted_pdu(ce_index,
                    LMP_SIMPLE_PAIRING_CONFIRM_OPCODE, tid, PDU_NOT_ALLOWED_ERROR);
            return;
        }
    }
#else
    if (tid != active_tid)
    {
        bz_auth_send_not_accepted_pdu(ce_index,
                LMP_SIMPLE_PAIRING_CONFIRM_OPCODE, tid, PDU_NOT_ALLOWED_ERROR);
        return;
    }
#endif	
    switch (auth->sub_state)
    {
        case NC_INTR_AWAIT_SP_CONFIRM:
            /* Store Ca */
            memcpy(&ssp_data->remote_C[0], &lmp_pdu_ptr->payload_content[1],
                    BZ_AUTH_CONFIRM_VALUE_SIZE);
            bz_auth_convert_to_msb(&ssp_data->remote_C[0],
                    BZ_AUTH_CONFIRM_VALUE_SIZE);
            /* send pre-generated LMP_Simple_Pairing_Number(Na) */
            bz_auth_send_simple_pairing_number_pdu(ce_index, auth, tid);
            bz_auth_transition_to_sub_state(auth, NC_INTR_SENT_SP_NUM);
            break;

        case PE_INTR_SENT_SP_CONFIRM:
            /* Store Cb */
            memcpy(&ssp_data->remote_C[0], &lmp_pdu_ptr->payload_content[1],
                    BZ_AUTH_CONFIRM_VALUE_SIZE);
            bz_auth_convert_to_msb(&ssp_data->remote_C[0],
                    BZ_AUTH_CONFIRM_VALUE_SIZE);
            /* send the pre-generated LMP_Simple_Pairing_Number(Nai) */
            bz_auth_send_simple_pairing_number_pdu(ce_index, auth, tid);
            bz_auth_transition_to_sub_state(auth, PE_INTR_SENT_SP_NUM);
            break;

        case PE_RESP_AWAIT_SP_CONFIRM:
            /* Store Ca */
            memcpy(&ssp_data->remote_C[0], &lmp_pdu_ptr->payload_content[1],
                    BZ_AUTH_CONFIRM_VALUE_SIZE);
            bz_auth_convert_to_msb(&ssp_data->remote_C[0],
                    BZ_AUTH_CONFIRM_VALUE_SIZE);
            /* Generate Nbi, Calculate Cbi and send
             * LMP_Simple_Pairing_Confirm(Cbi) [Responder - So 'b' is used]
             */
            bz_auth_send_ith_pe_confirm(ce_index, auth, tid);
            bz_auth_transition_to_sub_state(auth, PE_RESP_SENT_SP_CONFIRM);
            break;

        case PE_RESP_AWAIT_SP_CONFIRM_TO_SKIP:
            bz_auth_send_not_accepted_pdu(ce_index,
                LMP_SIMPLE_PAIRING_CONFIRM_OPCODE, tid,
                AUTHENTICATION_FAILURE_ERROR);
            bz_auth_handle_simple_pairing_complete(ce_index,
                    AUTHENTICATION_FAILURE_ERROR);
            return;

        case PE_RESP_PASSKEY_REQUESTED:
            if (auth->pending_pdu == NULL)
            {
                bz_auth_set_pending_pdu(auth, lmp_pdu_ptr, can_free_pdu);
            }
            else
            {
                lmp_error_code = PDU_NOT_ALLOWED_ERROR;
            }
            break;

        default:
            BZ_ASSERT(0, "Bug in the state machine or remote device is "
                    "misbehaving");
            lmp_error_code = PDU_NOT_ALLOWED_ERROR;
            break;
    }

    if (lmp_error_code != HCI_COMMAND_SUCCEEDED)
    {
        bz_auth_send_not_accepted_pdu(ce_index,
                LMP_SIMPLE_PAIRING_CONFIRM_OPCODE, tid, lmp_error_code);
    }
}

/**
 * Handles lmp_simple_pairing_number PDU from the remote device.
 *
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \param can_free_pdu Set to TRUE, if the PDU can be freed by the caller.
 *                     FALSE, otherwise.
 *
 * \return None.
 */
void bz_auth_handle_simple_pairing_number_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index, OUT BOOLEAN* can_free_pdu)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    BZ_AUTH_SSP_DATA* ssp_data;
    UCHAR lmp_error_code = HCI_COMMAND_SUCCEEDED;
    LMP_TRAN_ID tid;
    UCHAR active_tid;

    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;
    ssp_data = &auth->txn_params.ssp_data;
    tid = (LMP_TRAN_ID)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr);

    active_tid = bz_auth_get_tid_from_auth_role(ce_ptr, auth);
#ifdef _CCH_IOT_RALINK_			
    if(!g_efuse_lps_setting_3.iot_ralink_tid_no_check )
    {
        if (tid != active_tid)
        {
            bz_auth_send_not_accepted_pdu(ce_index,
                    LMP_SIMPLE_PAIRING_NUMBER_OPCODE, tid, PDU_NOT_ALLOWED_ERROR);
            return;
        }
    }
#else
    if (tid != active_tid)
    {
        bz_auth_send_not_accepted_pdu(ce_index,
                LMP_SIMPLE_PAIRING_NUMBER_OPCODE, tid, PDU_NOT_ALLOWED_ERROR);
        return;
    }
#endif    
    switch (auth->sub_state)
    {
        case NC_INTR_AWAIT_SP_NUM:
            /* ReceivedCb == ComputedCb */
            if (bz_auth_verify_confirmation_value(auth, lmp_pdu_ptr))
            {
                UINT32 Va;

                /* Calculate Va */
                /* Here we are initiator, so 'a' means local device and 'b'
                 * means remote device.
                 */
#ifndef _CCH_SC_ECDH_P256_                 
                ssp_g(bz_auth_local_pubkey.x /* PKax */,
                        ssp_data->remote_pubkey.x /* PKbx */,
                        ssp_data->local_N  /* Na */,
                        ssp_data->remote_N /* Nb */, &Va);
#else
                if( auth->len_prime == 6 )
                {
                ssp_g(bz_auth_local_pubkey.x /* PKax */,
                        ssp_data->remote_pubkey.x /* PKbx */,
                        ssp_data->local_N  /* Na */,
                        ssp_data->remote_N /* Nb */, &Va, auth->len_prime);
                }
                else if( auth->len_prime == 8 )
                {
                    ssp_g(bz_auth_local_pubkey_p256.x /* PKax */,
                        ssp_data->remote_pubkey.x /* PKbx */,
                        ssp_data->local_N  /* Na */,
                        ssp_data->remote_N /* Nb */, &Va, auth->len_prime);
                }
				
#endif

                Va = Va % 1000000U;
                bz_auth_send_accepted_pdu(ce_index,
                        LMP_SIMPLE_PAIRING_NUMBER_OPCODE, tid);
                /* HCI_User_Confirmation_Request_Event(Va) */
                bz_auth_generate_user_confirmation_request_event(ce_index, Va);
                bz_auth_transition_to_sub_state(auth,
                        NC_INTR_USER_CONFIRM_REQUESTED);
            }
            else
            {
                bz_auth_send_not_accepted_pdu(ce_index,
                        LMP_SIMPLE_PAIRING_NUMBER_OPCODE, tid,
                        AUTHENTICATION_FAILURE_ERROR);
                bz_auth_handle_simple_pairing_complete(ce_index,
                        AUTHENTICATION_FAILURE_ERROR);
                return;
            }
            break;
        case NC_RESP_AWAIT_SP_NUM:
            /* Store remote SP_Number(Na) */
            memcpy(&ssp_data->remote_N[0], &lmp_pdu_ptr->payload_content[1],
                    BZ_AUTH_SP_NUMBER_SIZE);
            bz_auth_convert_to_msb(&ssp_data->remote_N[0],
                    BZ_AUTH_SP_NUMBER_SIZE);

            bz_auth_send_accepted_pdu(ce_index,
                    LMP_SIMPLE_PAIRING_NUMBER_OPCODE, tid);
            /* send pre-generated LMP_Simple_Pairing_Number(Nb) */
            bz_auth_send_simple_pairing_number_pdu(ce_index, auth, tid);
            bz_auth_transition_to_sub_state(auth, NC_RESP_SENT_SP_NUM);
            break;
        case OOB_INTR_AWAIT_SP_NUM:
            /* Do we have remote device's OOB Data? (If yes, we might have
             * already fetched it and checked whether the commitment value
             * (Simple_Pairing_Hash_C) is correct.
             */
#ifdef _SUPPORT_SECURE_CONNECTION_
            if (((auth->secure_conn_enabled) &&
                (!ssp_data->is_oob256_check_succeeded)) ||
                ((!auth->secure_conn_enabled) && 
                 (!ssp_data->is_oob_check_succeeded)))
            {
                bz_auth_send_not_accepted_pdu(ce_index,
                        LMP_SIMPLE_PAIRING_NUMBER_OPCODE, tid,
                        AUTHENTICATION_FAILURE_ERROR);
                bz_auth_handle_simple_pairing_complete(ce_index,
                        AUTHENTICATION_FAILURE_ERROR);
                break;
            }
#else                        
            if (!ssp_data->is_oob_check_succeeded)
            {
                bz_auth_send_not_accepted_pdu(ce_index,
                        LMP_SIMPLE_PAIRING_NUMBER_OPCODE, tid,
                        AUTHENTICATION_FAILURE_ERROR);
                bz_auth_handle_simple_pairing_complete(ce_index,
                        AUTHENTICATION_FAILURE_ERROR);
                break;
            }
#endif            
            /* Store remote SP_Number(Nb) */
            memcpy(&ssp_data->remote_N[0], &lmp_pdu_ptr->payload_content[1],
                    BZ_AUTH_SP_NUMBER_SIZE);
            bz_auth_convert_to_msb(&ssp_data->remote_N[0],
                    BZ_AUTH_SP_NUMBER_SIZE);

            /* Either we didn't have the remote OOB data or his commitment is
             * valid and successfully authenticated. So accept him.
             */
            bz_auth_send_accepted_pdu(ce_index,
                LMP_SIMPLE_PAIRING_NUMBER_OPCODE, tid);
            bz_auth_initiate_auth_stage2(ce_index, tid);
            break;
        case OOB_RESP_REMOTE_OOB_DATA_REQUESTED:
            if (auth->pending_pdu != NULL)
            {
                lmp_error_code = PDU_NOT_ALLOWED_ERROR;
                break;
            }
            bz_auth_set_pending_pdu(auth, lmp_pdu_ptr, can_free_pdu);
            break;
        case OOB_RESP_AWAIT_SP_NUM:
            /* Do we have remote device's OOB Data? (If yes, we might have
             * already fetched it and checked whether the commitment value
             * (Simple_Pairing_Hash_C) is correct.
             */
#ifdef _SUPPORT_SECURE_CONNECTION_
            if (((auth->secure_conn_enabled) &&
                (!ssp_data->is_oob256_check_succeeded)) ||
                ((!auth->secure_conn_enabled) && 
                 (!ssp_data->is_oob_check_succeeded)))
            {
                bz_auth_send_not_accepted_pdu(ce_index,
                        LMP_SIMPLE_PAIRING_NUMBER_OPCODE, tid,
                        AUTHENTICATION_FAILURE_ERROR);
                bz_auth_handle_simple_pairing_complete(ce_index,
                        AUTHENTICATION_FAILURE_ERROR);
                break;
            }
#else
            if (!ssp_data->is_oob_check_succeeded)
            {
                bz_auth_send_not_accepted_pdu(ce_index,
                        LMP_SIMPLE_PAIRING_NUMBER_OPCODE, tid,
                        AUTHENTICATION_FAILURE_ERROR);
                bz_auth_handle_simple_pairing_complete(ce_index,
                        AUTHENTICATION_FAILURE_ERROR);
                break;
            }
#endif            
            
            /* Store remote SP_Number(Nb) */
            memcpy(&ssp_data->remote_N[0], &lmp_pdu_ptr->payload_content[1],
                    BZ_AUTH_SP_NUMBER_SIZE);
            bz_auth_convert_to_msb(&ssp_data->remote_N[0],
                    BZ_AUTH_SP_NUMBER_SIZE);

            bz_auth_send_accepted_pdu(ce_index,
                    LMP_SIMPLE_PAIRING_NUMBER_OPCODE, tid);
            /* send LMP_Simple_Pairing_Number(Nb) */
            bz_auth_send_simple_pairing_number_pdu(ce_index, auth, tid);
            bz_auth_transition_to_sub_state(auth, OOB_RESP_SENT_SP_NUM);
            break;
        case PE_INTR_AWAIT_SP_NUM:
            /* ReceivedCbi != ComputedCbi */
            if (!bz_auth_verify_confirmation_value(auth, lmp_pdu_ptr))
            {
                bz_auth_send_not_accepted_pdu(ce_index,
                    LMP_SIMPLE_PAIRING_NUMBER_OPCODE, tid,
                    AUTHENTICATION_FAILURE_ERROR);
                bz_auth_handle_simple_pairing_complete(ce_index,
                    AUTHENTICATION_FAILURE_ERROR);
                break;
            }
            /* Commitment value is same.. lets accept it */
            bz_auth_send_accepted_pdu(ce_index,
                LMP_SIMPLE_PAIRING_NUMBER_OPCODE, tid);
            if (ssp_data->i == 20)
            {
                /* Ouch.. 20 rounds are successfully over */
                bz_auth_update_final_pe_params(auth);
                bz_auth_initiate_auth_stage2(ce_index, tid);
                break;
            }
            /* Start music... its time for another round :) */
            /* Generate Nai, Calculate Cai and send
             * LMP_Simple_Pairing_Confirm(Cai) [Initiator - so 'a' is used]
             */
            bz_auth_send_ith_pe_confirm(ce_index, auth, tid);
            bz_auth_transition_to_sub_state(auth, PE_INTR_SENT_SP_CONFIRM);
            break;

        case PE_RESP_SENT_SP_CONFIRM:
            /* ReceivedCai != ComputedCai */
            if (!bz_auth_verify_confirmation_value(auth, lmp_pdu_ptr))
            {
                bz_auth_send_not_accepted_pdu(ce_index,
                    LMP_SIMPLE_PAIRING_NUMBER_OPCODE, tid,
                    AUTHENTICATION_FAILURE_ERROR);
                bz_auth_handle_simple_pairing_complete(ce_index,
                    AUTHENTICATION_FAILURE_ERROR);
                break;
            }
            /* Commitment value is same.. lets accept it */
            bz_auth_send_accepted_pdu(ce_index,
                LMP_SIMPLE_PAIRING_NUMBER_OPCODE, tid);
            /* send pre-generated LMP_Simple_Pairing_Number(Nbi) */
            bz_auth_send_simple_pairing_number_pdu(ce_index, auth, tid);
            bz_auth_transition_to_sub_state(auth, PE_RESP_SENT_SP_NUM);
            break;

        default:
            BZ_ASSERT(0, "Bug in the state machine or remote device is "
                    "misbehaving");
            lmp_error_code = PDU_NOT_ALLOWED_ERROR;
            break;
    }
    if (lmp_error_code != HCI_COMMAND_SUCCEEDED)
    {
        bz_auth_send_not_accepted_pdu(ce_index,
                LMP_SIMPLE_PAIRING_NUMBER_OPCODE, tid, lmp_error_code);
    }
}

/**
 * Handles lmp_dhkey_check PDU from the remote device.
 *
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \param can_free_pdu Set to TRUE, if the PDU can be freed by the caller.
 *                     FALSE, otherwise.
 *
 * \return None.
 */
void bz_auth_handle_dhkey_check_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index, OUT BOOLEAN* can_free_pdu)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    UCHAR lmp_error_code = HCI_COMMAND_SUCCEEDED;
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
            bz_auth_send_not_accepted_pdu(ce_index,
                    LMP_DHKEY_CHECK_OPCODE, tid, PDU_NOT_ALLOWED_ERROR);
            return;
        }
    }
#else
    if (tid != active_tid)
    {
        bz_auth_send_not_accepted_pdu(ce_index,
                LMP_DHKEY_CHECK_OPCODE, tid, PDU_NOT_ALLOWED_ERROR);
        return;
    }
#endif	
    switch (auth->sub_state)
    {
        case NC_RESP_USER_CONFIRM_REQUESTED:
            if (auth->pending_pdu == NULL)
            {
                bz_auth_set_pending_pdu(auth, lmp_pdu_ptr, can_free_pdu);
                return;
            }
            break;
        case NC_RESP_AWAIT_DH_KEY_TO_SKIP:
            bz_auth_send_not_accepted_pdu(ce_index,
                LMP_DHKEY_CHECK_OPCODE, tid, AUTHENTICATION_FAILURE_ERROR);
            bz_auth_handle_simple_pairing_complete(ce_index,
                    AUTHENTICATION_FAILURE_ERROR);
            return;
        case INTR_AWAIT_DHKEY_CHECK:
            /* Is ComputedEb != ReceivedEb */
            if (!bz_auth_verify_dhkey_check_value(ce_ptr, auth, lmp_pdu_ptr))
            {
                bz_auth_send_not_accepted_pdu(ce_index, LMP_DHKEY_CHECK_OPCODE,
                        tid, AUTHENTICATION_FAILURE_ERROR);
                bz_auth_handle_simple_pairing_complete(ce_index,
                        AUTHENTICATION_FAILURE_ERROR);
            }
            else
            {
                bz_auth_send_accepted_pdu(ce_index, LMP_DHKEY_CHECK_OPCODE,
                        tid);
                bz_auth_handle_simple_pairing_complete(ce_index,
                        HCI_COMMAND_SUCCEEDED);
            }
            break;
        case RESP_AWAIT_DHKEY_CHECK:
            /* Is ComputedEa != ReceivedEa */
            if (!bz_auth_verify_dhkey_check_value(ce_ptr, auth, lmp_pdu_ptr))
            {
                bz_auth_send_not_accepted_pdu(ce_index, LMP_DHKEY_CHECK_OPCODE,
                        tid, AUTHENTICATION_FAILURE_ERROR);
                bz_auth_handle_simple_pairing_complete(ce_index,
                        AUTHENTICATION_FAILURE_ERROR);
            }
            else
            {
                bz_auth_send_accepted_pdu(ce_index, LMP_DHKEY_CHECK_OPCODE,
                        tid);
                /* Compute Eb and send LMP_Dhkey_Check(Eb) */
                bz_auth_send_dhkey_check_pdu(ce_index, auth, tid);
                bz_auth_transition_to_sub_state(auth, RESP_AWAIT_DHKEY_RESP);
            }
            break;
        default:
            BZ_ASSERT(0, "Bug in the state machine or remote device is "
                    "misbehaving");
            lmp_error_code = PDU_NOT_ALLOWED_ERROR;
            break;
    }
    if (lmp_error_code != HCI_COMMAND_SUCCEEDED)
    {
        bz_auth_send_not_accepted_pdu(ce_index,
                LMP_DHKEY_CHECK_OPCODE, tid, lmp_error_code);
    }


#ifdef _CCH_SC_ECDH_P256_LOG

RT_BT_LOG(RED, CCH_DBG_152, 32, 
auth->txn_params.ssp_data.dhkey[0],auth->txn_params.ssp_data.dhkey[1],auth->txn_params.ssp_data.dhkey[2],auth->txn_params.ssp_data.dhkey[3],
auth->txn_params.ssp_data.dhkey[4],auth->txn_params.ssp_data.dhkey[5],auth->txn_params.ssp_data.dhkey[6],auth->txn_params.ssp_data.dhkey[7],
auth->txn_params.ssp_data.dhkey[8],auth->txn_params.ssp_data.dhkey[9],auth->txn_params.ssp_data.dhkey[10],auth->txn_params.ssp_data.dhkey[11],
auth->txn_params.ssp_data.dhkey[12],auth->txn_params.ssp_data.dhkey[13],auth->txn_params.ssp_data.dhkey[14],auth->txn_params.ssp_data.dhkey[15],
auth->txn_params.ssp_data.dhkey[16],auth->txn_params.ssp_data.dhkey[17],auth->txn_params.ssp_data.dhkey[18],auth->txn_params.ssp_data.dhkey[19],
auth->txn_params.ssp_data.dhkey[20],auth->txn_params.ssp_data.dhkey[21],auth->txn_params.ssp_data.dhkey[22],auth->txn_params.ssp_data.dhkey[23],
auth->txn_params.ssp_data.dhkey[24],auth->txn_params.ssp_data.dhkey[25],auth->txn_params.ssp_data.dhkey[26],auth->txn_params.ssp_data.dhkey[27],
auth->txn_params.ssp_data.dhkey[28],auth->txn_params.ssp_data.dhkey[29],auth->txn_params.ssp_data.dhkey[30],auth->txn_params.ssp_data.dhkey[31]
);

#endif	
}

/**
 * Handles lmp_accepted(lmp_simple_pairing_number) PDU from the remote device.
 *
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \param can_free_pdu Set to TRUE, if the PDU can be freed by the caller.
 *                     FALSE, otherwise.
 *
 * \return None.
 */
void bz_auth_handle_simple_pairing_number_accepted_pdu(
        LMP_PDU_PKT* lmp_pdu_ptr, UINT16 ce_index, OUT BOOLEAN* can_free_pdu)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    BZ_AUTH_SSP_DATA* ssp_data;
    LMP_TRAN_ID tid;
    UCHAR active_tid;

    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;
    ssp_data = &auth->txn_params.ssp_data;
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
        case NC_RESP_SENT_SP_NUM:
        {
            UINT32 Vb;

            /* Calculate Vb */
            /* Here we are responder, so 'a' means remote device and 'b'
             * means local device.
             */
#ifndef _CCH_SC_ECDH_P256_             
            ssp_g(ssp_data->remote_pubkey.x     /* PKax */,
                    bz_auth_local_pubkey.x /* PKbx */,
                    ssp_data->remote_N  /* Na */,
                    ssp_data->local_N   /* Nb */, &Vb);
#else
            if( auth->len_prime == 6 )
            {
            ssp_g(ssp_data->remote_pubkey.x     /* PKax */,
                    bz_auth_local_pubkey.x /* PKbx */,
                    ssp_data->remote_N  /* Na */,
                    ssp_data->local_N   /* Nb */, &Vb, auth->len_prime);
            }
            else if( auth->len_prime == 8 )
            {
                ssp_g(ssp_data->remote_pubkey.x     /* PKax */,
                    bz_auth_local_pubkey_p256.x /* PKbx */,
                    ssp_data->remote_N  /* Na */,
                    ssp_data->local_N   /* Nb */, &Vb, auth->len_prime);
            }
			
#endif
            Vb = Vb % 1000000U;
            /* HCI_User_Confirmation_Request_Event(Vb) */
            bz_auth_generate_user_confirmation_request_event(ce_index, Vb);
            bz_auth_transition_to_sub_state(auth,
                    NC_RESP_USER_CONFIRM_REQUESTED);
            break;
        }
        case NC_INTR_SENT_SP_NUM:
            bz_auth_transition_to_sub_state(auth, NC_INTR_AWAIT_SP_NUM);
            break;
        case OOB_INTR_SENT_SP_NUM:
            bz_auth_transition_to_sub_state(auth, OOB_INTR_AWAIT_SP_NUM);
            break;
        case OOB_RESP_SENT_SP_NUM:
            bz_auth_initiate_auth_stage2(ce_index, tid);
            break;
        case PE_INTR_SENT_SP_NUM:
            bz_auth_transition_to_sub_state(auth, PE_INTR_AWAIT_SP_NUM);
            break;
        case PE_RESP_SENT_SP_NUM:
            if (ssp_data->i == 20)
            {
                /* Ouch.. 20 rounds are successfully over */
                bz_auth_update_final_pe_params(auth);
                bz_auth_initiate_auth_stage2(ce_index, tid);
                break;
            }
            else
            {
                /* Start music... its time for another round :) */
                bz_auth_transition_to_sub_state(auth, PE_RESP_AWAIT_SP_CONFIRM);
            }
            break;
        default:
            BZ_ASSERT(0, "Bug in the state machine or remote device is "
                    "misbehaving");
            break;
    }
}

/**
 * Handles lmp_accepted(lmp_dhkey_check) PDU from the remote device.
 *
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \param can_free_pdu Set to TRUE, if the PDU can be freed by the caller.
 *                     FALSE, otherwise.
 *
 * \return None.
 */
void bz_auth_handle_dhkey_check_accepted_pdu(LMP_PDU_PKT* lmp_pdu_ptr,
        UINT16 ce_index, OUT BOOLEAN* can_free_pdu)
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
        case INTR_AWAIT_DHKEY_RESP:
            bz_auth_transition_to_sub_state(auth, INTR_AWAIT_DHKEY_CHECK);
            break;
        case RESP_AWAIT_DHKEY_RESP:
            bz_auth_handle_simple_pairing_complete(ce_index,
                    HCI_COMMAND_SUCCEEDED);
            break;

        default:
            BZ_ASSERT(0, "Bug in the state machine or remote device is "
                    "misbehaving");
            break;
    }
}

/**
 * Handles lmp_not_accepted(lmp_simple_pairing_confirm) PDU
 * from the remote device.
 *
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \param can_free_pdu Set to TRUE, if the PDU can be freed by the caller.
 *                     FALSE, otherwise.
 *
 * \return None.
 */
void bz_auth_handle_simple_pairing_confirm_not_accepted_pdu(
        LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index, OUT BOOLEAN* can_free_pdu)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    UCHAR lmp_reason;
    LMP_TRAN_ID tid;
    UCHAR active_tid;

    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;
    lmp_reason = lmp_pdu_ptr->payload_content[2];
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
        case PE_INTR_SENT_SP_CONFIRM:
            bz_auth_handle_simple_pairing_complete(ce_index, lmp_reason);
            break;
        default:
            BZ_ASSERT(0, "Either remote device is misbehaving or I "
                    "screwed up the state machine");
    }
}

/**
 * Handles lmp_not_accepted(lmp_simple_pairing_number) PDU
 * from the remote device.
 *
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \param can_free_pdu Set to TRUE, if the PDU can be freed by the caller.
 *                     FALSE, otherwise.
 *
 * \return None.
 */
void bz_auth_handle_simple_pairing_number_not_accepted_pdu(
        LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index, OUT BOOLEAN* can_free_pdu)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    UCHAR lmp_reason;
    LMP_TRAN_ID tid;
    UCHAR active_tid;

    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;
    lmp_reason = lmp_pdu_ptr->payload_content[2];
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
        case NC_RESP_SENT_SP_NUM:
        case NC_INTR_SENT_SP_NUM:
        case OOB_INTR_SENT_SP_NUM:
        case OOB_RESP_SENT_SP_NUM:
        case PE_INTR_SENT_SP_NUM:
        case PE_RESP_SENT_SP_NUM:
            bz_auth_handle_simple_pairing_complete(ce_index,
                    lmp_reason);
            break;
        default:
            BZ_ASSERT(0, "Either remote device is misbehaving or I "
                    "screwed up the state machine");
            break;
    }
}

/**
 * Handles lmp_not_accepted(lmp_dhkey_check) PDU
 * from the remote device.
 *
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \param can_free_pdu Set to TRUE, if the PDU can be freed by the caller.
 *                     FALSE, otherwise.
 *
 * \return None.
 */
void bz_auth_handle_dhkey_check_not_accepted_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index, OUT BOOLEAN* can_free_pdu)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    UCHAR lmp_reason;
    LMP_TRAN_ID tid;
    UCHAR active_tid;

    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;
    lmp_reason = lmp_pdu_ptr->payload_content[2];
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
        case INTR_AWAIT_DHKEY_RESP:
        case RESP_AWAIT_DHKEY_RESP:
            bz_auth_handle_simple_pairing_complete(ce_index,
                    lmp_reason);
            break;
        default:
            BZ_ASSERT(0, "Either remote device is misbehaving or I "
                    "screwed up the state machine");
            break;
    }
}

