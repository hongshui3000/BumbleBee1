/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file bz_auth_hci_2_1.c
 *  BlueWiz Authentication module 2.1 HCI interface implementation. It
 *  contains the handlers for 2.1 authentication related HCI commands and
 *  event generators.
 *
 * \author Santhosh kumar M
 * \date 2007-08-10
 */
/********************************* Logger *************************/ 
enum { __FILE_NUM__= 19 };
/********************************* Logger *************************/

/* ========================= Include File Section ========================= */
#include "bz_auth.h"
#include "lmp.h"
#include "bt_fw_hci.h"
#include "bt_fw_hci_internal.h"
#include "bz_auth_internal_2_1.h"
#include "bz_auth_extern_accessors.h"
#include "bz_debug.h"
#include "bz_auth_hci.h"
#include "bz_auth_lmp_2_1.h"
#include "mem.h"
/* ====================== Macro Declaration Section ======================= */


/* ==================== Structure Declaration Section ===================== */


/* ===================== Variable Declaration Section ===================== */


/* ================== Static Function Prototypes Section ================== */


/* ===================== Function Definition Section ====================== */
/**
 * Generates the IO Capability request event to the host.
 *
 * \param ce_index ACL connection entity index.
 *
 * \return None.
 */
void bz_auth_generate_io_cap_request_event(UINT16 ce_index)
{
    UCHAR event_parameter[HCI_IO_CAPABILITY_REQUEST_EVENT_LEN];
    memcpy(&event_parameter[0], lmp_connection_entity[ce_index].bd_addr,
            LMP_BD_ADDR_SIZE);
    if (!hci_generate_event(HCI_IO_CAPABILITY_REQUEST_EVENT,
            event_parameter, HCI_IO_CAPABILITY_REQUEST_EVENT_LEN))
    {
        hci_send_host_event_masked_signal(ce_index,
                HCI_IO_CAPABILITY_REQUEST_EVENT);
    }
}

/**
 * Generates the IO Capability response event to the host.
 *
 * \param ce_index ACL connection entity index.
 * \param remote_io_cap Received IO capabilities from the remote device. The
 *                      bytes are organised in (io_cap, oob_data_present,
 *                      auth_requirements) format.
 *
 * \return TRUE, if the operation was successful. FALSE, otherwise.
 */
void bz_auth_generate_io_cap_response_event(UINT16 ce_index,
        UCHAR *remote_io_cap)
{
    UCHAR event_parameter[HCI_IO_CAPABILITY_RESPONSE_EVENT_LEN];
    memcpy(&event_parameter[0], lmp_connection_entity[ce_index].bd_addr,
            LMP_BD_ADDR_SIZE);
    event_parameter[6] = remote_io_cap[0];
    event_parameter[7] = remote_io_cap[1];
    event_parameter[8] = remote_io_cap[2];
    hci_generate_event(HCI_IO_CAPABILITY_RESPONSE_EVENT,
            event_parameter, HCI_IO_CAPABILITY_RESPONSE_EVENT_LEN);
}

/**
 * Generates the user confirmation request event to the host.
 *
 * \param ce_index ACL connection entity index.
 * \param numeric_check_value Check value to be shown to the user for
 *                            confirmation.
 *
 * \return None.
 */
void bz_auth_generate_user_confirmation_request_event(UINT16 ce_index,
        UINT32 numeric_check_value)
{
    UCHAR event_parameter[HCI_USER_CONFIRMATION_REQUEST_EVENT_LEN];
    memcpy(&event_parameter[0], lmp_connection_entity[ce_index].bd_addr,
            LMP_BD_ADDR_SIZE);
    bt_fw_ultostr(&event_parameter[6], numeric_check_value, 4);
    if (!hci_generate_event(HCI_USER_CONFIRMATION_REQUEST_EVENT,
            event_parameter, HCI_USER_CONFIRMATION_REQUEST_EVENT_LEN))
    {
        hci_send_host_event_masked_signal(ce_index,
                HCI_USER_CONFIRMATION_REQUEST_EVENT);
    }
}

/**
 * Generates the user passkey request event to the host.
 *
 * \param ce_index ACL connection entity index.
 *
 * \return None.
 */
void bz_auth_generate_user_passkey_request_event(UINT16 ce_index)
{
    UCHAR event_parameter[HCI_USER_PASSKEY_REQUEST_EVENT_LEN];
    memcpy(&event_parameter[0], lmp_connection_entity[ce_index].bd_addr,
            LMP_BD_ADDR_SIZE);
    if (!hci_generate_event(HCI_USER_PASSKEY_REQUEST_EVENT, event_parameter,
            HCI_USER_PASSKEY_REQUEST_EVENT_LEN))
    {
        hci_send_host_event_masked_signal(ce_index,
                HCI_USER_PASSKEY_REQUEST_EVENT);
    }
}

/**
 * Generates the remote OOB data request event to the host.
 *
 * \param ce_index ACL connection entity index.
 *
 * \return None.
 */
void bz_auth_generate_remote_oob_data_request_event(UINT16 ce_index)
{
    UCHAR event_parameter[HCI_REMOTE_OOB_DATA_REQUEST_EVENT_LEN];
    memcpy(&event_parameter[0], lmp_connection_entity[ce_index].bd_addr,
            LMP_BD_ADDR_SIZE);
    if (!hci_generate_event(HCI_REMOTE_OOB_DATA_REQUEST_EVENT, event_parameter,
            HCI_REMOTE_OOB_DATA_REQUEST_EVENT_LEN))
    {
        hci_send_host_event_masked_signal(ce_index,
                HCI_REMOTE_OOB_DATA_REQUEST_EVENT);
    }
}

/**
 * Generates the simple pairing complete event to the host.
 *
 * \param ce_index ACL connection entity index.
 * \param status Status of the simple pairing procedure. It must a valid
 *               bluetooth error code.
 *
 * \return TRUE, if the operation was successful. FALSE, otherwise.
 */
void bz_auth_generate_simple_pairing_complete_event(UINT16 ce_index,
        UCHAR status)
{
    UCHAR event_parameter[HCI_SIMPLE_PAIRING_COMPLETE_EVENT_LEN];
    event_parameter[0] = status;
    memcpy(&event_parameter[1], lmp_connection_entity[ce_index].bd_addr,
            LMP_BD_ADDR_SIZE);
    hci_generate_event(HCI_SIMPLE_PAIRING_COMPLETE_EVENT, event_parameter,
            HCI_SIMPLE_PAIRING_COMPLETE_EVENT_LEN);
}

/**
 * Generates the user passkey notification event to the host.
 *
 * \param ce_index ACL connection entity index.
 * \param passkey Passkey to be notified to the user.
 *
 * \return None.
 */
void bz_auth_generate_user_passkey_notification_event(UINT16 ce_index,
        UINT32 passkey)
{
    UCHAR event_parameter[HCI_USER_PASSKEY_NOTIFICATION_EVENT_LEN];
    memcpy(&event_parameter[0], lmp_connection_entity[ce_index].bd_addr,
            LMP_BD_ADDR_SIZE);
    bt_fw_ultostr(&event_parameter[6], passkey, 4);
    hci_generate_event(HCI_USER_PASSKEY_NOTIFICATION_EVENT, event_parameter,
            HCI_USER_PASSKEY_NOTIFICATION_EVENT_LEN);
}

/**
 * Generates the keypress notification event to the host.
 *
 * \param ce_index ACL connection entity index.
 * \param notification_type Keypress notification type.
 *
 * \return TRUE, if the operation was successful. FALSE, otherwise.
 */
void bz_auth_generate_keypress_notification_event(UINT16 ce_index,
        UCHAR notification_type)
{
    UCHAR event_parameter[HCI_KEYPRESS_NOTIFICATION_EVENT_LEN];
    memcpy(&event_parameter[0], lmp_connection_entity[ce_index].bd_addr,
            LMP_BD_ADDR_SIZE);
    event_parameter[6] = notification_type;
    hci_generate_event(HCI_KEYPRESS_NOTIFICATION_EVENT, event_parameter,
            HCI_KEYPRESS_NOTIFICATION_EVENT_LEN);
}

/**
 * Generates the encryption key refresh complete event to the host.
 *
 * \param ce_index ACL connection entity index.
 * \param status Status of the encryption key refresh operation. It must be a
 *               valid bluetooth error code.
 *
 * \return TRUE, if the operation was successful. FALSE, otherwise.
 */
void bz_auth_generate_encryption_key_refresh_complete_event(UINT16 ce_index,
        UCHAR status)
{
    UCHAR event_parameter[HCI_ENCRYPTION_KEY_REFRESH_COMPLETE_EVENT_LEN];
    UINT16 conn_handle;

    conn_handle = lmp_connection_entity[ce_index].connection_type.connection_handle;

    event_parameter[0] = status;
    event_parameter[1] = (UCHAR)conn_handle;
    event_parameter[2] = (UCHAR)(conn_handle>>8);
#ifdef _CHK_RESTART_ENC
#ifdef _SECURE_CONN_TEST_LOG        
RT_BT_LOG(BLUE, DAPE_TEST_LOG522, 2,status, conn_handle);
#endif
#endif
    hci_generate_event(HCI_ENCRYPTION_KEY_REFRESH_COMPLETE_EVENT,
            event_parameter, HCI_ENCRYPTION_KEY_REFRESH_COMPLETE_EVENT_LEN);
}

/**
 * Handles the HCI refresh encryption command.
 *
 * \param hci_cmd_ptr HCI command packet.
 *
 * \return HCI_COMMAND_SUCCEEDED, if the operation is successfully initiated.
 *         Otherwise, a valid bluetooth error code.
 */
UCHAR bz_auth_handle_refresh_encryption(HCI_CMD_PKT* hci_cmd_ptr)
{
    UINT16 ce_index;
    BZ_AUTH_LINK_PARAMS* auth;

    if (!bz_auth_validate_conn_handle(hci_cmd_ptr, &ce_index))
    {
        return NO_CONNECTION_ERROR;
    }

    hci_generate_command_status_event(hci_cmd_ptr->cmd_opcode,
            HCI_COMMAND_SUCCEEDED);

    /* hurray we have passed the sanity checks, lets proceed :) */
    auth = lmp_connection_entity[ce_index].auth;
    if (!bz_auth_is_epr_supported(&lmp_connection_entity[ce_index]))
    {
        bz_auth_generate_encryption_key_refresh_complete_event(ce_index,
                COMMAND_DISALLOWED_ERROR);
        return HCI_COMMAND_SUCCEEDED;
    }
    switch (auth->super_state)
    {
        case ENCRYPTED:
        case TEMP_ENCRYPTED:
            auth->txn_params.auth_role = BZ_AUTH_ROLE_INITIATOR;
            bz_auth_perform_pause_encryption_action(ce_index, auth);
            break;

        default:
            bz_auth_generate_encryption_key_refresh_complete_event(ce_index,
                    COMMAND_DISALLOWED_ERROR);
            return HCI_COMMAND_SUCCEEDED;
    }

    bz_auth_perform_super_state_transition(auth, HCI_REFRESH_ENCRYPTION_KEY);

    return HCI_COMMAND_SUCCEEDED;
}

/**
 * Handles the HCI keypress notification command.
 *
 * \param hci_cmd_ptr HCI command packet.
 *
 * \return HCI_COMMAND_SUCCEEDED, if the operation is successfully initiated.
 *         Otherwise, a valid bluetooth error code.
 */
UCHAR bz_auth_handle_key_press_notification(HCI_CMD_PKT* hci_cmd_ptr)
{
    BZ_AUTH_LINK_PARAMS* auth;
    UCHAR status;
    LMP_TRAN_ID tid = SELF_DEV_TID;
    UINT16 ce_index;

    /* validate basic parameters necessary to process this command */
    if (!bz_auth_validate_bd_addr(hci_cmd_ptr, &ce_index))
    {
        status = NO_CONNECTION_ERROR;
        ce_index = INVALID_CE_INDEX;
    }
    else
    {
        /* done with sanity checks, now process reply command */
        auth = lmp_connection_entity[ce_index].auth;

        switch (auth->sub_state)
        {
            case PE_RESP_PASSKEY_REQUESTED:
                tid = REMOTE_DEV_TID;
                /*@fallthrough@*/
            case PE_INTR_PASS_KEY_REQUESTED:
                if (hci_cmd_ptr->cmd_parameter[6] < 5)
                {
                    bz_auth_send_keypress_notification_pdu(ce_index,
                            hci_cmd_ptr->cmd_parameter[6], tid);
                    status = HCI_COMMAND_SUCCEEDED;
                }
                else
                {
                    status = INVALID_HCI_COMMAND_PARAMETERS_ERROR;
                }
                break;
            default:
                /* we don't expect the command now, the host is trying ruin our
                 * life. I am intelligent.. hahaha :-D
                 */
                status = COMMAND_DISALLOWED_ERROR;
                break;
        }
    }
    bz_auth_generate_command_complete_event(hci_cmd_ptr, status,
            ce_index);

    return HCI_COMMAND_SUCCEEDED;
}

/**
 * Returns whether IO Capability from the host, at this point, is expected or
 * not.
 *
 * \param auth Authentication parameters of the ACL link.
 *
 * \return TRUE, if the IO Capability is expected. FALSE, otherwise.
 */
BOOLEAN bz_auth_is_io_cap_expected(BZ_AUTH_LINK_PARAMS* auth)
{
    return (BOOLEAN)(auth->sub_state == INTR_IO_CAP_REQUESTED
            || auth->sub_state == RESP_IO_CAP_REQUESTED);
}

/**
 * Returns whether io cap req reply parameters are valid or not.
 *
 * \param hci_cmd_ptr HCI Command Packet.
 *
 * \return TRUE, if the io cap req reply params are valid. FALSE, otherwise.
 */
BOOLEAN bz_auth_is_valid_hci_io_cap_req_reply_params(
        HCI_CMD_PKT* hci_cmd_ptr)
{
    if ((hci_cmd_ptr->cmd_parameter[LMP_BD_ADDR_SIZE] > 0x03)
#ifdef _SUPPORT_SECURE_CONNECTION_
            || (hci_cmd_ptr->cmd_parameter[LMP_BD_ADDR_SIZE+1] > 0x03)
#else    
            || (hci_cmd_ptr->cmd_parameter[LMP_BD_ADDR_SIZE+1] > 0x01)
#endif            
            || (hci_cmd_ptr->cmd_parameter[LMP_BD_ADDR_SIZE+2] > 0x05))
    {
        return FALSE;
    }
    return TRUE;
}

/**
 * Returns whether io cap req neg reply parameters are valid or not.
 *
 * \param hci_cmd_ptr HCI Command Packet.
 *
 * \return TRUE, if the io cap req neg reply params are valid. FALSE, otherwise.
 */
BOOLEAN bz_auth_is_valid_hci_io_cap_req_neg_reply_params(
        HCI_CMD_PKT* hci_cmd_ptr)
{
    UCHAR reason;

    reason = hci_cmd_ptr->cmd_parameter[LMP_BD_ADDR_SIZE];
    if (reason == SECURE_SIMPLE_PAIRING_NOT_SUPPORTED_BY_HOST
            || reason < 0x01 || reason > LAST_ENTRY_BLUETOOTH_ERROR_CODE)
    {
        return FALSE;
    }

    return TRUE;
}

/**
 * Handles IO Capability request reply from the host.
 *
 * \param hci_cmd_ptr HCI Command Packet.
 *
 * \return HCI_COMMAND_SUCCEEDED, if the command was handled. A valid
 *         bluetooth error code, otherwise.
 */
UCHAR bz_auth_handle_io_cap_request_reply(HCI_CMD_PKT* hci_cmd_ptr)
{
    UINT16 ce_index;
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    UINT16 reason;

    reason = perform_sanity_checks_for_reply_commands(hci_cmd_ptr,
            bz_auth_is_io_cap_expected,
            bz_auth_is_valid_hci_io_cap_req_reply_params, &ce_index, &ce_ptr);
    if (reason != HCI_COMMAND_SUCCEEDED)
    {
        return HCI_COMMAND_SUCCEEDED; /* Cmd_Status should not be generated */
    }

    auth = ce_ptr->auth;

    /* received the IO Capability at right time, now store it temporarily
     */
    bz_auth_extract_io_cap_data(&hci_cmd_ptr->cmd_parameter[6],
            &auth->txn_params.ssp_data.local_io_cap);
#ifdef _SUPPORT_SECURE_CONNECTION_
    if (auth->secure_conn_enabled)
    {
        if (auth->txn_params.ssp_data.local_io_cap.oob_data_present < 2)
        {
            //RT_BT_LOG(RED, DAPE_TEST_LOG293, 1,
            //auth->txn_params.ssp_data.local_io_cap.oob_data_present);
            auth->txn_params.ssp_data.local_io_cap.oob_data_present = 0;
        }
        if (auth->txn_params.ssp_data.local_io_cap.oob_data_present >= 2)
        {
            //RT_BT_LOG(GREEN, DAPE_TEST_LOG293, 1,
            //auth->txn_params.ssp_data.local_io_cap.oob_data_present);
            auth->txn_params.ssp_data.local_io_cap.oob_data_present = 1;
        }
    }
#endif

    /* are we responder, then job is pretty simple */
    if (auth->sub_state == RESP_IO_CAP_REQUESTED)
    {
        /* Send LMP_io_capability_res */
        bz_auth_send_io_capability_res_pdu(ce_index, auth, REMOTE_DEV_TID);
        /* Enter into Public Key exchange phase */
        bz_auth_initiate_public_key_exchange(ce_index, REMOTE_DEV_TID);

        return HCI_COMMAND_SUCCEEDED;
    }

    /* we should be initiator to reach here.
     * don't we have any collision? damn good, we can complete straight away
     */
    if (auth->pending_pdu == NULL)
    {
        /* Send LMP_io_capability_req(SELF_DEV_TID) and wait for response */
        bz_auth_send_io_capability_req_pdu(ce_index, auth, SELF_DEV_TID);
        bz_auth_transition_to_sub_state(auth,
                INTR_AWAIT_REMOTE_IO_CAP_RESPONSE);
        return HCI_COMMAND_SUCCEEDED;
    }

    /* being a initiator and handling collisions is pretty tough -- let's see
     * what we can do with the collided pdu.
     */
    if (bz_auth_get_pdu_opcode(auth->pending_pdu) == LMP_ESCAPE4_OPCODE
            && (bz_auth_get_ext_pdu_opcode(auth->pending_pdu)
                == LMP_IO_CAPABILITY_REQ_OPCODE))
    {
        if (bz_auth_is_master(ce_ptr))  /* are we master? */
        {
            /* Initiate our transaction */
            /* send LMP_io_capability_req(MASTER_TID) and wait for response */
            bz_auth_send_io_capability_req_pdu(ce_index, auth, MASTER_TID);
            bz_auth_transition_to_sub_state(auth,
                    INTR_AWAIT_REMOTE_IO_CAP_RESPONSE);
            /* terminate remote transaction */
            /* send lmp_not_accepted(same collision) and free stored pdu */
            bz_auth_send_not_accepted_ext_pdu(ce_index, LMP_ESCAPE4_OPCODE,
                    LMP_IO_CAPABILITY_REQ_OPCODE,
                    (LMP_TRAN_ID) LMP_GET_TRANSACTION_ID(auth->pending_pdu),
                    LMP_ERROR_TRANSACTION_COLLISION_ERROR);
            bz_auth_free_pending_pdu(auth);
        }
        else
        {
            /* AuthRole = InitiatorAsResponder */
            auth->txn_params.auth_role = BZ_AUTH_ROLE_INITIATOR_AS_RESPONDER;
            /* Extract the parameters & Generate IO_Cap_Response_Event */
            bz_auth_extract_io_cap_data(&auth->pending_pdu->payload_content[2],
                    &auth->txn_params.ssp_data.remote_io_cap);
            bz_auth_free_pending_pdu(auth);
            bz_auth_generate_io_cap_response_event(ce_index,
                    (UCHAR*)&auth->txn_params.ssp_data.remote_io_cap);
            /* Send LMP_io_capability_res(MASTER_TID) */
            bz_auth_send_io_capability_res_pdu(ce_index, auth, MASTER_TID);
            /* Enter into Public Key exchange phase */
            bz_auth_initiate_public_key_exchange(ce_index, MASTER_TID);
        }
    }
    else
    {
        BZ_ASSERT(0, "We shouldn't have stored this pdu, or sombody "
                "forgot to make the pending_pdu = NULL");
        bz_auth_free_pending_pdu(auth);
    }

    return HCI_COMMAND_SUCCEEDED;
}

/**
 * Performs IO Capability request negative reply action
 *
 * \param ce_index ACL coneection entity index.
 * \param reason reason for rejection.
 *
 * \return HCI_COMMAND_SUCCEEDED, if the command was handled. A valid
 *         bluetooth error code, otherwise.
 */
UCHAR bz_auth_perform_io_cap_request_neg_reply_action(UINT16 ce_index,
        UINT16 reason)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;

    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;

    if (auth->pending_pdu == NULL)  /* are we in pathetic(collision) state? */
    {
        /* No.. We are safe... Nobody dared to collide with us */
        /* if we are a responder, we need to reject his trasaction with
         * whatever crap error code the host has given us.
         */
        if (auth->sub_state == RESP_IO_CAP_REQUESTED)
        {
            /* send lmp_not_accepted_ext(reason) and free stored in_rand pdu */
            bz_auth_send_not_accepted_ext_pdu(ce_index, LMP_ESCAPE4_OPCODE,
                    LMP_IO_CAPABILITY_REQ_OPCODE, REMOTE_DEV_TID,
                    (UCHAR)reason);
        }
        /* indicate auth_status(failed, reason) */
        bz_auth_handle_simple_pairing_complete(ce_index,
                AUTHENTICATION_FAILURE_ERROR);

        return HCI_COMMAND_SUCCEEDED;
    }
    else if (bz_auth_get_pdu_opcode(auth->pending_pdu) == LMP_ESCAPE4_OPCODE
            && (bz_auth_get_ext_pdu_opcode(auth->pending_pdu)
                == LMP_IO_CAPABILITY_REQ_OPCODE))
    {
        /* There seems to be a collision and we must be initiator here.
         * We don't store LMP_io_capability_req as responder (we just extract
         * the details from the PDU and free it).
         */
        BZ_ASSERT(auth->txn_params.auth_role == BZ_AUTH_ROLE_INITIATOR,
                "We must be initiator here (then only we can have collision)");
        /* send lmp_not_accepted_ext(reason) and free stored in_rand pdu */
        bz_auth_send_not_accepted_ext_pdu(ce_index, LMP_ESCAPE4_OPCODE,
                LMP_IO_CAPABILITY_REQ_OPCODE,
                (LMP_TRAN_ID) (LMP_GET_TRANSACTION_ID(auth->pending_pdu)),
                    (UCHAR)reason);
        bz_auth_free_pending_pdu(auth);
        /* indicate auth_status(failed, reason) */
        bz_auth_handle_simple_pairing_complete(ce_index,
                AUTHENTICATION_FAILURE_ERROR);

        return HCI_COMMAND_SUCCEEDED;
    }
    else
    {
        BZ_ASSERT(0, "The PDU shouldn't have been stored or sombody forgot "
                "to make the pending_pdu = NULL");
        bz_auth_free_pending_pdu(auth);
    }

    return HCI_COMMAND_SUCCEEDED;
}

/**
 * Handles IO Capability request negative reply from the host.
 *
 * \param hci_cmd_ptr HCI Command Packet.
 *
 * \return HCI_COMMAND_SUCCEEDED, if the command was handled. A valid
 *         bluetooth error code, otherwise.
 */
UCHAR bz_auth_handle_io_cap_request_neg_reply(HCI_CMD_PKT* hci_cmd_ptr)
{
    UINT16 ce_index;
    LMP_CONNECTION_ENTITY* ce_ptr;
    UINT16 reason;

    reason = perform_sanity_checks_for_reply_commands(hci_cmd_ptr,
            bz_auth_is_io_cap_expected,
            bz_auth_is_valid_hci_io_cap_req_neg_reply_params,
            &ce_index, &ce_ptr);
    if (reason != HCI_COMMAND_SUCCEEDED)
    {
        return HCI_COMMAND_SUCCEEDED; /* Cmd_Status should not be generated */
    }

    /* Host given reason for rejecting the IO_Cap_Request */
    reason = hci_cmd_ptr->cmd_parameter[6];

    /* received the event at the right time, now proceed further */
    return bz_auth_perform_io_cap_request_neg_reply_action(ce_index, reason);
}

/**
 * Returns whether User Confirmation from the host, at this point,
 * is expected or not.
 *
 * \param auth Authentication parameters of the ACL link.
 *
 * \return TRUE, if the User Confirmation is expected. FALSE, otherwise.
 */
BOOLEAN bz_auth_is_user_confirmation_expected(
        BZ_AUTH_LINK_PARAMS* auth)
{
    return (BOOLEAN)(auth->sub_state == NC_INTR_USER_CONFIRM_REQUESTED
            || auth->sub_state == NC_RESP_USER_CONFIRM_REQUESTED);
}

/**
 * Handles User Confirmation request reply from the host.
 *
 * \param hci_cmd_ptr HCI Command Packet.
 *
 * \return HCI_COMMAND_SUCCEEDED, if the command was handled. A valid
 *         bluetooth error code, otherwise.
 */
UCHAR bz_auth_handle_user_confirmation_request_reply(HCI_CMD_PKT* hci_cmd_ptr)
{
    UINT16 ce_index;
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    UINT16 reason;
    LMP_TRAN_ID tid = SELF_DEV_TID;

    reason = perform_sanity_checks_for_reply_commands(hci_cmd_ptr,
            bz_auth_is_user_confirmation_expected, NULL, &ce_index, &ce_ptr);
    if (reason != HCI_COMMAND_SUCCEEDED)
    {
        return HCI_COMMAND_SUCCEEDED; /* Cmd_Status should not be generated */
    }
    auth = ce_ptr->auth;

    /* received the event at the right time, now proceed further */
    /* remote device rejected us with LMP_numeric_comparison_failed pdu? */
    if (auth->pending_pdu == NULL)
    {
        /* No.. We are safe... remote device didn't dare to reject us */
        if (auth->sub_state == NC_RESP_USER_CONFIRM_REQUESTED)
        {
            tid = REMOTE_DEV_TID;
        }
        /* Let us proceed with auth_stage2 */
        bz_auth_initiate_auth_stage2(ce_index, tid);
    }
    else if (bz_auth_get_pdu_opcode(auth->pending_pdu) == LMP_ESCAPE4_OPCODE
            && (bz_auth_get_ext_pdu_opcode(auth->pending_pdu)
                == LMP_NUMERIC_COMPARISON_FAILED_OPCODE))
    {
        /* There seems to be numeric comparison failed at remote side */
        BZ_ASSERT(auth->txn_params.auth_role != BZ_AUTH_ROLE_INITIATOR,
                "We must be responder here to receive "
                "LMP_numeric_comparison_failed PDU");
        bz_auth_free_pending_pdu(auth);
        /* indicate auth_status(failed, reason) */
        bz_auth_handle_simple_pairing_complete(ce_index,
                AUTHENTICATION_FAILURE_ERROR);
    }
    else if (bz_auth_get_pdu_opcode(auth->pending_pdu)
            == LMP_DHKEY_CHECK_OPCODE)
    {
        UCHAR local_can_free_pdu;

        /* The initiator seems to be faster */
        BZ_ASSERT(auth->txn_params.auth_role != BZ_AUTH_ROLE_INITIATOR,
                "We must be responder here to receive "
                "LMP_dhkey_check PDU (responder can't initiate State2)");

        bz_auth_transition_to_sub_state(auth, RESP_AWAIT_DHKEY_CHECK);

        bz_auth_handle_dhkey_check_pdu(auth->pending_pdu, ce_index,
                &local_can_free_pdu);
        bz_auth_free_pending_pdu(auth);
    }
    else
    {
        BZ_ASSERT(0, "The PDU shouldn't have been stored or sombody forgot "
                "to make the pending_pdu = NULL");
        bz_auth_free_pending_pdu(auth);
    }
    return HCI_COMMAND_SUCCEEDED;
}

/**
 * Performs User Confirmation request negative reply action.
 *
 * \param ce_index ACL connection entity index.
 *
 * \return HCI_COMMAND_SUCCEEDED, if the command was handled. A valid
 *         bluetooth error code, otherwise.
 */
UCHAR bz_auth_perform_user_confirmation_request_neg_reply_action(
        UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;

    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;

    /* remote device rejected us with LMP_numeric_comparison_failed pdu? */
    if (auth->pending_pdu == NULL)
    {
        /* No.. We are safe... remote device didn't dare to reject us */
        if (auth->sub_state == NC_RESP_USER_CONFIRM_REQUESTED)
        {
            /* Let us wait for remote device response to complete(terminate)
             * simple pairing. We are not privileged enough to send
             * LMP_numeric_comparison_failed pdu.. we are responder :'(
             */
            bz_auth_transition_to_sub_state(auth, NC_RESP_AWAIT_DH_KEY_TO_SKIP);
        }
        else
        {
            /* We are initiator here.. We can command the responder :) */
            bz_auth_send_numeric_comparison_failed_pdu(ce_index, auth,
                    SELF_DEV_TID);
            /* indicate auth_status(failed, reason) */
            bz_auth_handle_simple_pairing_complete(ce_index,
                    AUTHENTICATION_FAILURE_ERROR);
        }
    }
    else if (bz_auth_get_pdu_opcode(auth->pending_pdu) == LMP_ESCAPE4_OPCODE
            && (bz_auth_get_ext_pdu_opcode(auth->pending_pdu)
                == LMP_NUMERIC_COMPARISON_FAILED_OPCODE))
    {
        /* There seems to be numeric comparison failed at remote side */
        BZ_ASSERT(auth->txn_params.auth_role != BZ_AUTH_ROLE_INITIATOR,
                "We must be responder here to receive "
                "LMP_numeric_comparison_failed PDU");
        bz_auth_free_pending_pdu(auth);
        /* indicate auth_status(failed, reason) */
        bz_auth_handle_simple_pairing_complete(ce_index,
                AUTHENTICATION_FAILURE_ERROR);
    }
    else if (bz_auth_get_pdu_opcode(auth->pending_pdu)
            == LMP_DHKEY_CHECK_OPCODE)
    {
        UCHAR local_can_free_pdu;

        /* The initiator seems to be faster */
        BZ_ASSERT(auth->txn_params.auth_role != BZ_AUTH_ROLE_INITIATOR,
                "We must be responder here to receive "
                "LMP_dhkey_check PDU (responder can't initiate State2)");
        bz_auth_transition_to_sub_state(auth, NC_RESP_AWAIT_DH_KEY_TO_SKIP);
        bz_auth_handle_dhkey_check_pdu(auth->pending_pdu, ce_index,
                &local_can_free_pdu);
        bz_auth_free_pending_pdu(auth);
    }
    else
    {
        BZ_ASSERT(0, "The PDU shouldn't have been stored or sombody forgot "
                "to make the pending_pdu = NULL");
        bz_auth_free_pending_pdu(auth);
    }

    return HCI_COMMAND_SUCCEEDED;
}

/**
 * Handles User Confirmation request negative reply from the host.
 *
 * \param hci_cmd_ptr HCI Command Packet.
 *
 * \return HCI_COMMAND_SUCCEEDED, if the command was handled. A valid
 *         bluetooth error code, otherwise.
 */
UCHAR bz_auth_handle_user_confirmation_request_neg_reply(
        HCI_CMD_PKT* hci_cmd_ptr)
{
    UINT16 ce_index;
    LMP_CONNECTION_ENTITY* ce_ptr;
    UINT16 reason;

    reason = perform_sanity_checks_for_reply_commands(hci_cmd_ptr,
            bz_auth_is_user_confirmation_expected, NULL, &ce_index, &ce_ptr);
    if (reason != HCI_COMMAND_SUCCEEDED)
    {
        return HCI_COMMAND_SUCCEEDED; /* Cmd_Status should not be generated */
    }
    /* received the event at the right time, now proceed further */
    return bz_auth_perform_user_confirmation_request_neg_reply_action(ce_index);
}

/**
 * Returns whether user passkey from the host, at this point,
 * is expected or not.
 *
 * \param auth Authentication parameters of the ACL link.
 *
 * \return TRUE, if the user passkey is expected. FALSE, otherwise.
 */
BOOLEAN bz_auth_is_user_passkey_expected(BZ_AUTH_LINK_PARAMS* auth)
{
    return (BOOLEAN)(auth->sub_state == PE_INTR_PASS_KEY_REQUESTED
            || auth->sub_state == PE_RESP_PASSKEY_REQUESTED);
}

/**
 * Returns whether user passkey req reply parameters are valid or not.
 *
 * \param hci_cmd_ptr HCI Command Packet.
 *
 * \return TRUE, if the user passkey req reply params are valid.
 * FALSE, otherwise.
 */
BOOLEAN bz_auth_is_valid_hci_user_passkey_reply_params(
        HCI_CMD_PKT* hci_cmd_ptr)
{
    UINT32 passkey;
    BT_FW_EXTRACT_32_BITS(passkey,
            &hci_cmd_ptr->cmd_parameter[LMP_BD_ADDR_SIZE]);

    if (passkey > 0x000F423F)
    {
        return FALSE;
    }
    return TRUE;
}

/**
 * Handles user passkey request from the host.
 *
 * \param hci_cmd_ptr HCI Command Packet.
 *
 * \return HCI_COMMAND_SUCCEEDED, if the command was handled. A valid
 *         bluetooth error code, otherwise.
 */
UCHAR bz_auth_handle_user_passkey_request_reply(HCI_CMD_PKT* hci_cmd_ptr)
{
    UINT16 ce_index;
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    UINT16 reason;

    reason = perform_sanity_checks_for_reply_commands(hci_cmd_ptr,
            bz_auth_is_user_passkey_expected,
            bz_auth_is_valid_hci_user_passkey_reply_params,
            &ce_index, &ce_ptr);
    if (reason != HCI_COMMAND_SUCCEEDED)
    {
        return HCI_COMMAND_SUCCEEDED; /* Cmd_Status should not be generated */
    }
    auth = ce_ptr->auth;

    /* received the event at the right time, now proceed further */

    /* Initialize PE params */
    auth->txn_params.ssp_data.i = 0;
    /* Set passkey */
    BT_FW_EXTRACT_32_BITS(auth->txn_params.ssp_data.passkey,
            &hci_cmd_ptr->cmd_parameter[6]);

    /* Have we received any pdu from the remote device? */
    if (auth->pending_pdu == NULL)
    {
        /* We are damn fast to get the response.. Good Host :) or we are the
         * initiator -- responder can't send any pdu.. haha :)
         */
        if (auth->sub_state == PE_RESP_PASSKEY_REQUESTED)
        {
            /* Wait for initiator to start Stage1 with some PDU
             * (LMP_simple_pairing_confirm or LMP_passkey_entry_failed)
             * Kudos.. we are damn fast.. 8)
             */
            bz_auth_transition_to_sub_state(auth, PE_RESP_AWAIT_SP_CONFIRM);
        }
        else
        {
            /* We are the initiator, its our responsibility to initiate Stage1
             * with some pdu.
             */
            /* Generate Nai, Calculate Cai and send
             * LMP_Simple_Pairing_Confirm(Cai) [Initiator - so 'a' is used]
             */
            bz_auth_send_ith_pe_confirm(ce_index, auth, SELF_DEV_TID);
            bz_auth_transition_to_sub_state(auth, PE_INTR_SENT_SP_CONFIRM);
        }
    }
    else if (bz_auth_get_pdu_opcode(auth->pending_pdu) ==
            LMP_SIMPLE_PAIRING_CONFIRM_OPCODE)
    {
        UCHAR local_can_free_pdu;

        /* Hmmm.. the remote device(initiator) is faster.. :( */
        BZ_ASSERT(auth->txn_params.auth_role != BZ_AUTH_ROLE_INITIATOR,
                "We must be responder here to receive "
                "LMP_simple_pairing_confirm while we are waiting for Passkey");
        /* Pretend as if we are receiving the LMP_simple_pairing_confirm PDU
         * just now.
         */
        bz_auth_transition_to_sub_state(auth, PE_RESP_AWAIT_SP_CONFIRM);
        bz_auth_handle_simple_pairing_confirm_pdu(auth->pending_pdu, ce_index,
                &local_can_free_pdu);
        bz_auth_free_pending_pdu(auth);
    }
    else if (bz_auth_get_pdu_opcode(auth->pending_pdu) == LMP_ESCAPE4_OPCODE
            && (bz_auth_get_ext_pdu_opcode(auth->pending_pdu)
                == LMP_PASSKEY_FAILED_OPCODE))
    {
        /* The remote host (initiator) must have given passkey -ve reply..
         * Poor chap! Didn't have the patience to type the Passkey.
         */
        BZ_ASSERT(auth->txn_params.auth_role != BZ_AUTH_ROLE_INITIATOR,
                "We must be responder here to receive "
                "LMP_passkey_entry_failed pdu");
        bz_auth_free_pending_pdu(auth);
        /* indicate auth_status(failed, reason) */
        bz_auth_handle_simple_pairing_complete(ce_index,
                AUTHENTICATION_FAILURE_ERROR);
    }
    else
    {
        BZ_ASSERT(0, "The PDU shouldn't have been stored or sombody forgot "
                "to make the pending_pdu = NULL");
        bz_auth_free_pending_pdu(auth);
    }

    return HCI_COMMAND_SUCCEEDED;
}

/**
 * Performs user passkey request negative reply action.
 *
 * \param ce_index ACL connection entity index.
 *
 * \return HCI_COMMAND_SUCCEEDED, if the command was handled. A valid
 *         bluetooth error code, otherwise.
 */
UCHAR bz_auth_perform_user_passkey_request_neg_reply_action(UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;

    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;


    /* Have we received any pdu from the remote device? */
    if (auth->pending_pdu == NULL)
    {
        /* We are damn fast to get the response.. Good Host :) or we are the
         * initiator -- responder can't send any pdu.. haha :)
         * But anyway, its a -ve reply :(
         */
        if (auth->sub_state == PE_RESP_PASSKEY_REQUESTED)
        {
            /* We are responder and don't have the privilege to send
             * LMP_passkey_entry_failed PDU :( So let's for initiator to start
             * the Stage1. (But after that we can reject him :))
             */
            bz_auth_transition_to_sub_state(auth,
                    PE_RESP_AWAIT_SP_CONFIRM_TO_SKIP);
        }
        else
        {
            /* We are initiator here.. We can command the responder :) */
            bz_auth_send_passkey_entry_failed_pdu(ce_index, auth, SELF_DEV_TID);
            /* indicate auth_status(failed, reason) */
            bz_auth_handle_simple_pairing_complete(ce_index,
                    AUTHENTICATION_FAILURE_ERROR);
        }
    }
    else if (bz_auth_get_pdu_opcode(auth->pending_pdu) ==
            LMP_SIMPLE_PAIRING_CONFIRM_OPCODE)
    {
        /* Hmmm.. the remote device(initiator) is faster.. :( */
        BZ_ASSERT(auth->txn_params.auth_role != BZ_AUTH_ROLE_INITIATOR,
                "We must be responder here to receive "
                "LMP_simple_pairing_confirm while we are waiting for Passkey");
        /* send lmp_not_accepted(reason) and free stored pdu */
        bz_auth_send_not_accepted_pdu(ce_index,
                LMP_SIMPLE_PAIRING_CONFIRM_OPCODE,
                (LMP_TRAN_ID) LMP_GET_TRANSACTION_ID(auth->pending_pdu),
                AUTHENTICATION_FAILURE_ERROR);
        bz_auth_free_pending_pdu(auth);
        /* indicate auth_status(failed, reason) */
        bz_auth_handle_simple_pairing_complete(ce_index,
                AUTHENTICATION_FAILURE_ERROR);
    }
    else if (bz_auth_get_pdu_opcode(auth->pending_pdu) == LMP_ESCAPE4_OPCODE
            && (bz_auth_get_ext_pdu_opcode(auth->pending_pdu)
                == LMP_PASSKEY_FAILED_OPCODE))
    {
        /* The remote host (initiator) must have given passkey -ve reply..
         * Poor chap! Didn't have the patience to type the Passkey (exactly
         * like our host -- why the hell you start pairing then? -- its ok).
         */
        BZ_ASSERT(auth->txn_params.auth_role != BZ_AUTH_ROLE_INITIATOR,
                "We must be responder here to receive "
                "LMP_passkey_entry_failed pdu");
        bz_auth_free_pending_pdu(auth);
        /* indicate auth_status(failed, reason) */
        bz_auth_handle_simple_pairing_complete(ce_index,
                AUTHENTICATION_FAILURE_ERROR);
    }
    else
    {
        BZ_ASSERT(0, "The PDU shouldn't have been stored or sombody forgot "
                "to make the pending_pdu = NULL");
        bz_auth_free_pending_pdu(auth);
    }

    return HCI_COMMAND_SUCCEEDED;
}

/**
 * Handles user passkey request negative reply from the host.
 *
 * \param hci_cmd_ptr HCI Command Packet.
 *
 * \return HCI_COMMAND_SUCCEEDED, if the command was handled. A valid
 *         bluetooth error code, otherwise.
 */
UCHAR bz_auth_handle_user_passkey_request_neg_reply(HCI_CMD_PKT* hci_cmd_ptr)
{
    UINT16 ce_index;
    LMP_CONNECTION_ENTITY* ce_ptr;
    UINT16 reason;

    reason = perform_sanity_checks_for_reply_commands(hci_cmd_ptr,
            bz_auth_is_user_passkey_expected, NULL, &ce_index, &ce_ptr);
    if (reason != HCI_COMMAND_SUCCEEDED)
    {
        return HCI_COMMAND_SUCCEEDED; /* Cmd_Status should not be generated */
    }

    /* received the event at the right time, now proceed further */
    return bz_auth_perform_user_passkey_request_neg_reply_action(ce_index);
}

/**
 * Returns whether remote oob data reply from the host, at this point,
 * is expected or not.
 *
 * \param auth Authentication parameters of the ACL link.
 *
 * \return TRUE, if the oob data reply is expected. FALSE, otherwise.
 */
BOOLEAN bz_auth_is_remote_oob_data_expected(
        BZ_AUTH_LINK_PARAMS* auth)
{
    return (BOOLEAN)(auth->sub_state == OOB_INTR_REMOTE_OOB_DATA_REQUESTED
            || auth->sub_state == OOB_RESP_REMOTE_OOB_DATA_REQUESTED);
}

/**
 * Handles remote oob data request reply from the host.
 *
 * \param hci_cmd_ptr HCI Command Packet.
 *
 * \return HCI_COMMAND_SUCCEEDED, if the command was handled. A valid
 *         bluetooth error code, otherwise.
 */
UCHAR bz_auth_handle_remote_oob_data_request_reply(HCI_CMD_PKT* hci_cmd_ptr)
{
    UINT16 ce_index;
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    UINT16 reason;
    LMP_TRAN_ID tid = SELF_DEV_TID;
    BOOLEAN check_result;

    reason = perform_sanity_checks_for_reply_commands(hci_cmd_ptr,
            bz_auth_is_remote_oob_data_expected, NULL, &ce_index, &ce_ptr);
    if (reason != HCI_COMMAND_SUCCEEDED)
    {
        return HCI_COMMAND_SUCCEEDED; /* Cmd_Status should not be generated */
    }
    auth = ce_ptr->auth;

    /* received the event at the right time, now proceed further */
    /* Convert to MSB format... All the internal cryptographic functions
     * require the input to be in MSB format.
     */
    bz_auth_convert_to_msb(&hci_cmd_ptr->cmd_parameter[6],
            BZ_AUTH_CONFIRM_VALUE_SIZE);
    bz_auth_convert_to_msb(&hci_cmd_ptr->cmd_parameter[22],
            BZ_AUTH_OOB_SECRET_NUMBER_SIZE);
    /* Check whether the received Simple_Pairing_Hash_C is same as computed
     * Simple_Pairing_Hash_C.
     */
    check_result = bz_auth_verify_oob_hash_value(auth,
            &hci_cmd_ptr->cmd_parameter[6], &hci_cmd_ptr->cmd_parameter[22]);

    /* Store the check result for future use */
    auth->txn_params.ssp_data.is_oob_check_succeeded = check_result;
    if (auth->pending_pdu == NULL)
    {
        if (auth->sub_state == OOB_INTR_REMOTE_OOB_DATA_REQUESTED)
        {
            /* send LMP_Simple_Pairing_Number(Na) */
            bz_auth_send_simple_pairing_number_pdu(ce_index, auth, tid);
            bz_auth_transition_to_sub_state(auth, OOB_INTR_SENT_SP_NUM);
        }
        else /* auth->sub_state == OOB_RESP_REMOTE_OOB_DATA_REQUESTED */
        {
            bz_auth_transition_to_sub_state(auth, OOB_RESP_AWAIT_SP_NUM);
        }
    }
    else if (bz_auth_get_pdu_opcode(auth->pending_pdu) ==
            LMP_SIMPLE_PAIRING_NUMBER_OPCODE)
    {
        BZ_ASSERT(auth->txn_params.auth_role != BZ_AUTH_ROLE_INITIATOR,
                "We must be responder to receive simple pairing number (OOB)");

        tid = (LMP_TRAN_ID)LMP_GET_TRANSACTION_ID(auth->pending_pdu);
        if (check_result)
        {
            /* The authencity of the remote device is verified using OOB. Now
             * store his random number.
             */
            memcpy(&auth->txn_params.ssp_data.remote_N[0],
                    &auth->pending_pdu->payload_content[1],
                    BZ_AUTH_SP_NUMBER_SIZE);
            /* Convert to MSB format (as required by crypto algo) */
            bz_auth_convert_to_msb(&auth->txn_params.ssp_data.remote_N[0],
                    BZ_AUTH_SP_NUMBER_SIZE);
            bz_auth_send_accepted_pdu(ce_index,
                    LMP_SIMPLE_PAIRING_NUMBER_OPCODE, tid);
            bz_auth_free_pending_pdu(auth);
            /* send LMP_Simple_Pairing_Number(Nb) */
            bz_auth_send_simple_pairing_number_pdu(ce_index, auth, tid);
            bz_auth_transition_to_sub_state(auth, OOB_RESP_SENT_SP_NUM);
        }
        else
        {
            bz_auth_send_not_accepted_pdu(ce_index,
                    LMP_SIMPLE_PAIRING_NUMBER_OPCODE, tid,
                    AUTHENTICATION_FAILURE_ERROR);
            bz_auth_free_pending_pdu(auth);
            bz_auth_handle_simple_pairing_complete(ce_index,
                    AUTHENTICATION_FAILURE_ERROR);
        }
    }
    else if (bz_auth_get_ext_pdu_opcode(auth->pending_pdu) ==
            LMP_OOB_FAILED_OPCODE)
    {
        BZ_ASSERT(auth->txn_params.auth_role != BZ_AUTH_ROLE_INITIATOR,
                "We must be responder to receive simple pairing number (OOB)");

        bz_auth_free_pending_pdu(auth);
        bz_auth_handle_simple_pairing_complete(ce_index,
                AUTHENTICATION_FAILURE_ERROR);
    }
    else
    {
        BZ_ASSERT(0, "The PDU shouldn't have been stored or sombody forgot "
                "to make the pending_pdu = NULL");
        bz_auth_free_pending_pdu(auth);
    }

    return HCI_COMMAND_SUCCEEDED;
}

/**
 * Performs remote oob data request negative reply action.
 *
 * \param ce_index ACL connection entity index.
 *
 * \return HCI_COMMAND_SUCCEEDED, if the command was handled. A valid
 *         bluetooth error code, otherwise.
 */
UCHAR bz_auth_perform_remote_oob_data_request_neg_reply_action(UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    LMP_TRAN_ID tid = SELF_DEV_TID;

    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;

    if (auth->pending_pdu == NULL)
    {
        if (auth->sub_state == OOB_INTR_REMOTE_OOB_DATA_REQUESTED)
        {
            bz_auth_send_oob_failed_pdu(ce_index, auth, SELF_DEV_TID);
            bz_auth_handle_simple_pairing_complete(ce_index,
                    AUTHENTICATION_FAILURE_ERROR);
        }
        else
        {
            /* In case of responder, lets treat -ve reply as a OOB check
             * failure. As a responder, we don't need to differentiate between
             * these two because the responder shall not send LMP_oob_failed
             * PDU.
             */
            auth->txn_params.ssp_data.is_oob_check_succeeded = FALSE;
            bz_auth_transition_to_sub_state(auth, OOB_RESP_AWAIT_SP_NUM);
        }
    }
    else if (bz_auth_get_pdu_opcode(auth->pending_pdu) ==
            LMP_SIMPLE_PAIRING_NUMBER_OPCODE
            || bz_auth_get_ext_pdu_opcode(auth->pending_pdu)
                == LMP_OOB_FAILED_OPCODE)
    {
        BZ_ASSERT(auth->txn_params.auth_role != BZ_AUTH_ROLE_INITIATOR,
                "We must be responder to receive simple pairing number (OOB)");
        if (bz_auth_get_pdu_opcode(auth->pending_pdu)
                == LMP_SIMPLE_PAIRING_NUMBER_OPCODE)
        {
            tid = (LMP_TRAN_ID)LMP_GET_TRANSACTION_ID(auth->pending_pdu);
            bz_auth_send_not_accepted_pdu(ce_index,
                    LMP_SIMPLE_PAIRING_NUMBER_OPCODE, tid,
                    AUTHENTICATION_FAILURE_ERROR);
        }
        bz_auth_free_pending_pdu(auth);
        bz_auth_handle_simple_pairing_complete(ce_index,
                AUTHENTICATION_FAILURE_ERROR);
    }
    else
    {
        BZ_ASSERT(0, "The PDU shouldn't have been stored or sombody forgot "
                "to make the pending_pdu = NULL");
        bz_auth_free_pending_pdu(auth);
    }

    return HCI_COMMAND_SUCCEEDED;
}

/**
 * Handles remote oob data request negative reply from the host.
 *
 * \param hci_cmd_ptr HCI Command Packet.
 *
 * \return HCI_COMMAND_SUCCEEDED, if the command was handled. A valid
 *         bluetooth error code, otherwise.
 */
UCHAR bz_auth_handle_remote_oob_data_request_neg_reply(
        HCI_CMD_PKT* hci_cmd_ptr)
{
    UINT16 ce_index;
    LMP_CONNECTION_ENTITY* ce_ptr;
    UINT16 reason;

    reason = perform_sanity_checks_for_reply_commands(hci_cmd_ptr,
            bz_auth_is_remote_oob_data_expected, NULL, &ce_index, &ce_ptr);
    if (reason != HCI_COMMAND_SUCCEEDED)
    {
        return HCI_COMMAND_SUCCEEDED; /* Cmd_Status should not be generated */
    }
    /* received the event at the right time, now proceed further */
    return bz_auth_perform_remote_oob_data_request_neg_reply_action(ce_index);
}

