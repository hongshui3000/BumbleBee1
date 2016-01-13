/***************************************************************************
 Copyright (C) Realtek Semiconductor Corp.
 This module is a confidential and proprietary property of Realtek and
 a possession or use of this module requires written permission of Realtek.
 ***************************************************************************/

/**
 * \file
 *  SECURE CONNECTION HCI Commands implementation.
 *  
 */
#ifdef _SUPPORT_SECURE_CONNECTION_
/********************************* Logger *************************/ 
enum { __FILE_NUM__= 77 };
/********************************* Logger *************************/
/* -----------------Includes----------------------------- */
#include "bt_fw_hci_internal.h"
#include "btc_defines.h"
#include "bt_fw_hci_external_defines.h"
#include "bt_fw_hci_secure_conn_cmds_evts.h"
#include "lmp_internal.h"
#include "mem.h"
#include "bz_auth.h"
#include "bz_auth_internal.h"
#include "bz_auth_hci.h"
#include "bt_secure_conn.h"
#include "bz_auth_internal_2_1.h"
#include "bz_auth_extern_accessors.h"

/* -----------------Global variables--------------------- */
#ifdef _DAPE_TEST_GEN_FAKE_ESCO_DATA
extern UINT8 g_gen_fake_esco_data;
extern HCI_SYNC_DATA_PKT *pgen_synchronous_pkt; 
#endif

/* -----------------Static functions--------------------- */
/**
 * Handles the HCI Read Secure Connection Host Support Command.
 * 
 * \param hci_cmd_ptr HCI Command pointer.
 * 
 * \return HCI Status.
 */
UCHAR hci_handle_remote_oob_ext_data_req_reply(HCI_CMD_PKT *hci_cmd_ptr)
{
    UINT16 ce_index;
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    UINT16 reason;
    LMP_TRAN_ID tid = SELF_DEV_TID;
    BOOLEAN check_result;
    BOOLEAN check_result2 = FALSE;

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
     
    if (auth->secure_conn_enabled)
    {
        bz_auth_convert_to_msb(&hci_cmd_ptr->cmd_parameter[38],
                BZ_AUTH_CONFIRM_VALUE_SIZE);
        bz_auth_convert_to_msb(&hci_cmd_ptr->cmd_parameter[54],
                BZ_AUTH_OOB_SECRET_NUMBER_SIZE);
        check_result2 = bz_auth_verify_oob_hash_value(auth,
                &hci_cmd_ptr->cmd_parameter[38], &hci_cmd_ptr->cmd_parameter[54]);
        auth->txn_params.ssp_data.is_oob256_check_succeeded = check_result2;
    }
    else
    {
        /* Store the check result for future use */
        auth->txn_params.ssp_data.is_oob_check_succeeded = check_result;
    }
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

        if (((auth->secure_conn_enabled)&&(check_result2)) ||
            ((!auth->secure_conn_enabled)&&(check_result)))
       
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
 * Handles the HCI Read Secure Connection Host Support Command.
 * 
 * \param hci_cmd_ptr HCI Command pointer.
 * 
 * \return HCI Status.
 */
UCHAR hci_handle_read_secure_conn_host_support_command(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR ret_error = HCI_COMMAND_SUCCEEDED;   
    return ret_error;
}

/**
 * Handles the HCI Write Secure Connection Host Support Command.
 * 
 * \param hci_cmd_ptr HCI Command pointer.
 * 
 * \return HCI Status.
 */
UCHAR hci_handle_write_secure_conn_host_support_command(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR ret_error = HCI_COMMAND_SUCCEEDED;    
    SECURE_CONN_HCI_WRITE_SECURE_CONN_HOST_SUPPORT_CMD_PKT_S *cmd;
    
    cmd = (SECURE_CONN_HCI_WRITE_SECURE_CONN_HOST_SUPPORT_CMD_PKT_S *)hci_cmd_ptr; 
    
    if ((lmp_self_device_data.lc_cur_dev_state == LC_CUR_STATE_PAGE)
        || (lmp_self_device_data.scan_enable & 0x02))
    {
        ret_error = COMMAND_DISALLOWED_ERROR;
    }
    else
    {
        bz_auth_dev_params.secure_connection_host_enable = cmd->Secure_Conn_Host_Support;
        if (bz_auth_dev_params.secure_connection_host_enable == 0x01)
        {
            bz_auth_enable_sc_ext_feature_bits();
        }
        else
        {
            bz_auth_reset_sc_ext_feature_bits();
        }
    }    
    return ret_error;
}

/**
 * Handles the HCI Read Authentication Payload Timeout Command.
 * 
 * \param hci_cmd_ptr HCI Command pointer.
 * 
 * \return HCI Status.
 */
UCHAR hci_handle_read_auth_payload_timeout(HCI_CMD_PKT *hci_cmd_ptr)
{    
    UINT16 ce_index ;
    UINT16 conn_handle;
    UCHAR ret_error = HCI_COMMAND_SUCCEEDED;    

    SECURE_CONN_HCI_READ_AUTH_PAYLOAD_TO_CMD_PKT_S *cmd;
    
    cmd = (SECURE_CONN_HCI_READ_AUTH_PAYLOAD_TO_CMD_PKT_S *)hci_cmd_ptr; 
    conn_handle = cmd->bConn_handle[0] | (cmd->bConn_handle[1] << 8);

    /* Validate connection handle.*/
    /* Obtain the index corresponding to the connection handle */
    if(LMP_GET_CE_INDEX_FROM_CONN_HANDLE(conn_handle, &ce_index)
            == API_FAILURE)
    {
        ret_error = NO_CONNECTION_ERROR ;
        ce_index = INVALID_CE_INDEX;
    }
    
    return ret_error;
}

/**
 * Handles the HCI Write Authentication Payload Timeout Command.
 * 
 * \param hci_cmd_ptr HCI Command pointer.
 * 
 * \return HCI Status.
 */
UCHAR hci_handle_write_auth_payload_timeout(HCI_CMD_PKT *hci_cmd_ptr)
{
    UINT16 ce_index ;
    UINT16 conn_handle;
    UCHAR ret_error = HCI_COMMAND_SUCCEEDED;    

    SECURE_CONN_HCI_WRITE_AUTH_PAYLOAD_TO_CMD_PKT_S *cmd;
    
    cmd = (SECURE_CONN_HCI_WRITE_AUTH_PAYLOAD_TO_CMD_PKT_S *)hci_cmd_ptr; 
    conn_handle = cmd->bConn_handle[0] | (cmd->bConn_handle[1] << 8);

    /* Validate connection handle.*/
    /* Obtain the index corresponding to the connection handle */
    if(LMP_GET_CE_INDEX_FROM_CONN_HANDLE(conn_handle, &ce_index)
            == API_FAILURE)
    {
        ret_error = NO_CONNECTION_ERROR ;
        ce_index = INVALID_CE_INDEX;

#ifdef _BT4_1_LE_PING_FEATURE_SUPPORT_
        /* check for LE connections */
        if (conn_handle > LL_HCI_MAX_CONNECTION_HANDLE)
        {
            return NO_CONNECTION_ERROR;
        }

        LL_CONN_HANDLE_UNIT *pHandle = ll_fw_search_handle_unit_via_conn_handle(conn_handle);

        if (pHandle == NULL || pHandle->connected == FALSE)
        {
            return NO_CONNECTION_ERROR;
        }

        if (pHandle->encrypted == FALSE)
        {
            return COMMAND_DISALLOWED_ERROR;
        }

        pHandle->auth_payload_timeout = cmd->bAuth_Payload_TO[0] | (cmd->bAuth_Payload_TO[1] << 8);

        if (pHandle->auth_payload_timeout < pHandle->ce_interval * (1 + pHandle->slave_latency))
        {
            return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
        }

        ret_error = HCI_COMMAND_SUCCEEDED;

        llc_ping_req_restart_timer(pHandle->unit_id);
#endif /* _BT4_1_LE_PING_FEATURE_SUPPORT_ */

    }
    else
    {
        LMP_CONNECTION_ENTITY *ce_ptr;
        ce_ptr = &lmp_connection_entity[ce_index];       
        ce_ptr->max_auth_interval = cmd->bAuth_Payload_TO[0] | (cmd->bAuth_Payload_TO[1] << 8);
#ifdef SECURE_CONN_PING_EN
        BZ_AUTH_LINK_PARAMS* auth;
        auth = lmp_connection_entity[ce_index].auth;

        if ((auth->secure_conn_enabled) && (bz_auth_is_link_encrypted(ce_index)))
        {
            //RT_BT_LOG(GREEN, DAPE_TEST_LOG207, 1,BB_read_native_clock());
            bz_auth_start_ping_req_timer(ce_index);
        }
#endif    
#ifdef _FAKE_SECURE_CONN_
        bz_auth_start_ping_req_timer(ce_index);
        RT_BT_LOG(GREEN, DAPE_TEST_LOG207, 1,BB_read_native_clock());
#endif                        
#ifdef _SECURE_CONN_TEST_LOG
RT_BT_LOG(BLUE, DAPE_TEST_LOG525, 6,ce_index, hci_cmd_ptr->cmd_parameter[2],
hci_cmd_ptr->cmd_parameter[3], ce_ptr->max_auth_interval,0,0);
#endif
    }
    return ret_error;
}

/**
 * Handles the HCI Set Reserved LT ADDR Command.
 * 
 * \param hci_cmd_ptr HCI Command pointer.
 * 
 * \return HCI Status.
 */
UCHAR hci_handle_read_local_oob_ext_data(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR ret_error = HCI_COMMAND_SUCCEEDED;    
    return ret_error;
}
/**
 * Validates the parameters of Write_Secure_Connection_Test_Mode_command
 * and updates the self device test mode paramter with value received
 * from the Host.
 *
 * \param hci_cmd_ptr Pointer to the command packet.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_write_secure_conn_test_mode_command(HCI_CMD_PKT *hci_cmd_ptr)
{

    UINT16 ce_index ;
    UINT16 conn_handle;
    UCHAR ret_error = HCI_COMMAND_SUCCEEDED;    

    SECURE_CONN_HCI_WRITE_SC_TEST_MODE_CMD_PKT_S *cmd;
    
    cmd = (SECURE_CONN_HCI_WRITE_SC_TEST_MODE_CMD_PKT_S *)hci_cmd_ptr; 
    conn_handle = cmd->bConn_handle[0] | (cmd->bConn_handle[1] << 8);

    /* Validate connection handle.*/
    /* Obtain the index corresponding to the connection handle */
    if(LMP_GET_CE_INDEX_FROM_CONN_HANDLE(conn_handle, &ce_index)
            == API_FAILURE)
    {
        ret_error = NO_CONNECTION_ERROR ;
        ce_index = INVALID_CE_INDEX;
        return ret_error;
    }
    else
    {
        LMP_CONNECTION_ENTITY *ce_ptr;
        
        ce_ptr = &lmp_connection_entity[ce_index];

        /* Step 1. Check DM1 ACL-U Mode. */
        if (cmd->DM1_ACLU_mode)
        {
            if ((ce_ptr->connection_type.packet_type != 0x3306) ||
            	   (ce_ptr->connection_type.packet_type != 0x330E)) 
            {
                if (ce_ptr->connection_type.packet_type & DM1)
                {
                    ce_ptr->connection_type.packet_type &= (~(DM1));
                    ce_ptr->disable_dm1 = 1;
                }
                //RT_BT_LOG(BLUE, DAPE_TEST_LOG207, 1,
                //	ce_ptr->connection_type.packet_type);
            }
        }
        else
        {
            /* check if DM1 is disabled before. */
            if (ce_ptr->disable_dm1)
            {
                ce_ptr->connection_type.packet_type |= DM1;
                ce_ptr->disable_dm1 = 0;
            }
        }

        /* Step 2. Check eSCO loopback mode. */
        if (cmd->esco_loopback_mode)
        {
            pf_switch_hci_dma_parameter(ENTER_TEST_MODE);
            lmp_self_device_data.test_mode = HCI_REMOTE_LOOPBACK_MODE;

#ifdef _DAPE_TEST_GEN_FAKE_ESCO_DATA
            if (ce_ptr->remote_dev_role == MASTER)
            {
                /* disable gen fake esco data to avoid we chain wrong packet
                   when esco tx (austin) */                           
                if (g_gen_fake_esco_data)
                {
                    if (pgen_synchronous_pkt  != NULL)
                    {
                        dma_tx_fifo_pkt_free((void * )pgen_synchronous_pkt,
                                               HCI_TRANSPORT_SYNC_DATA_PKT_TYPE);
                        pgen_synchronous_pkt = NULL;
                    }
                    g_gen_fake_esco_data = FALSE;
                }
            }
#endif            
        }
        else
        {
            pf_switch_hci_dma_parameter(LEAVE_TEST_MODE);
            lmp_self_device_data.test_mode = HCI_NO_LOOPBACK_MODE;
        }

    }

    return ret_error;
}


/* -----------------External functions------------------- */
/**
 * Handles all the link controller for BR/EDR Secure Connection.
 * This function is called from the hci_task.c if the opcode
 * does not match any of the 1.1/1.2/2.1/3.0/CSA4 Version opcodes.
 *
 * \param hci_cmd_ptr Pointer to HCI_CMD_PKT.
 * \param sent_event  Sets the variable if event is sent.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
 UCHAR hci_handle_secure_conn_link_controller_commands(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR ret_error = UNKNOWN_HCI_COMMAND_ERROR;
    UINT8 sent_cmd_status = FALSE;

    switch(hci_cmd_ptr->cmd_opcode)
    {
    case HCI_REMOTE_OOB_EXT_DATA_REQUEST_REPLY_OPCODE:            
        ret_error = hci_handle_remote_oob_ext_data_req_reply(hci_cmd_ptr);
        break;

    default:
        break;
    }

    if (ret_error == UNKNOWN_HCI_COMMAND_ERROR)
    {
        /* bypass unknown hci command */
        return UNKNOWN_HCI_COMMAND_ERROR;            
    }
    
    if(sent_cmd_status == TRUE)
    {
        hci_generate_command_status_event(hci_cmd_ptr->cmd_opcode, ret_error);        
    }
    else
    {
        hci_generate_command_complete_event(hci_cmd_ptr->cmd_opcode, ret_error,
                0, NULL);
    }

    return HCI_COMMAND_SUCCEEDED;
}
/**
 * Handles all the host controller and baseband for BR/EDR Secure Connection.
 * This function is called from the hci_task.c if the opcode
 * does not match any of the 1.1/1.2/2.1/3.0/CSA4 Version opcodes.
 *
 * \param hci_cmd_ptr Pointer to HCI_CMD_PKT.
 * \param sent_event  Sets the variable if event is sent.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_secure_conn_hc_bb_commands(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR ret_error = HCI_COMMAND_SUCCEEDED;

    switch(hci_cmd_ptr->cmd_opcode)
    {
    case HCI_READ_SECURE_CONN_HOST_SUPPORT_OPCODE:            
        ret_error = hci_handle_read_secure_conn_host_support_command(hci_cmd_ptr);
        break;
        
    case HCI_WRITE_SECURE_CONN_HOST_SUPPORT_OPCODE:            
        ret_error = hci_handle_write_secure_conn_host_support_command(hci_cmd_ptr);
        break;
        
    case HCI_READ_AUTH_PAYLOAD_TIMEOUT_OPCODE:            
        ret_error = hci_handle_read_auth_payload_timeout(hci_cmd_ptr);
        break;
        
    case HCI_WRITE_AUTH_PAYLOAD_TIMEOUT_OPCODE:            
        ret_error = hci_handle_write_auth_payload_timeout(hci_cmd_ptr);
        break;

    case HCI_READ_LOCAL_OOB_EXT_DATA_OPCODE:            
        ret_error = hci_handle_read_local_oob_ext_data(hci_cmd_ptr);
        break;

    default:
        ret_error = UNKNOWN_HCI_COMMAND_ERROR;
        break;
    }

    return ret_error;
}
UCHAR hci_handle_secure_conn_testing_commands(HCI_CMD_PKT *hci_cmd_ptr)
{

    UCHAR ret_error = HCI_COMMAND_SUCCEEDED;

    switch(hci_cmd_ptr->cmd_opcode)
    {
    case HCI_WRITE_SECURE_CONN_TEST_MODE_OPCODE:            
        ret_error = hci_handle_write_secure_conn_test_mode_command(hci_cmd_ptr);
        if (ret_error != HCI_COMMAND_SUCCEEDED)
        {
            BT_FW_HCI_ERR(HCI_HANDLE_WRITE_LOOPBACK_COMMAND_FAILED,0,0);
        }
        break;
        
    default:
        ret_error = UNKNOWN_HCI_COMMAND_ERROR;
        break;
    }

    return ret_error;
}

#endif

