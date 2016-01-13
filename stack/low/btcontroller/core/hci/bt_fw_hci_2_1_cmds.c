/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  BT2_1 HCI Commands implementation.
 *  
 *  \author Muthu Subramanian K
 *  
 */
/********************************* Logger *************************/ 
enum { __FILE_NUM__= 6 };
/********************************* Logger *************************/

/* -----------------Includes----------------------------- */
#include "bt_fw_hci_internal.h"
#include "bt_fw_hci_2_1_cmds.h"
#include "bt_fw_acl_q.h"
#include "lmp_2_1.h"
#include "btc_defines.h"
#include "mem.h"
#ifdef _3DD_FUNCTION_SUPPORT_
#include "bt_3dd.h"
#endif

/* -----------------Global variables--------------------- */

/* -----------------Static functions--------------------- */
/**
 * Parses the eir_data to find the length of the significant
 * part of the EIR Data.
 * 
 * \param eir_data Pointer to the EIR Data.
 * 
 * \return (>240) on Failure (<=240) on Success.
 */
UCHAR hci_get_eir_data_len(UCHAR *eir_data)
{
    /* follow EIR data format of GAP (vol 3). In original FW implementation, 
       it can cause error EIR data amount to remote device - austin  */
    
    UINT8 len;
    UINT16 sign_part_len = 0;

    while (1)
    {
        len = eir_data[sign_part_len];
        if (len == 0)
        {
            /* this is an early termination */
            break;
        }

        /* increase one EIR Data Structure */
        sign_part_len += 1 + len;

        /* avoid to cause overflow in 240 byte EIR data block */
        if (sign_part_len >= MAX_EIR_DATA_LEN)
        {
            sign_part_len = MAX_EIR_DATA_LEN;
            break;
        }
    }

    return (UCHAR)sign_part_len;
}

/**
 * Finds a suitable packet type for EIR data.
 * 
 * \param eir_len EIR Data length.
 * \param fec_req FEC Required?
 * 
 * \return Packet type or 0xFFFF on Error.
 */
UINT16 hci_get_eir_pkt_type(UINT16 eir_len, UCHAR fec_req)
{
    if(fec_req == 0)
    {
        if (eir_len <= 27)
        {
            return BB_DH1_LUT;
        }
        if (eir_len <= 183)
        {
            return BB_DH3_LUT;
        }     
    }
    else
    {
        if (eir_len <= 17)
        {
            return BB_DM1_LUT;
        }
        if (eir_len <= 121)
        {
            return BB_DM3_LUT;
        }
        if (eir_len <= 224)
        {
            return BB_DM5_LUT;
        }
        return (UINT16) 0xFFFF;
    }
    return BB_DH5_LUT;      
}

/**
 * HCI Write EIR Data command handler.
 * 
 * \param hci_cmd_ptr Pointer to the hci_cmd.
 * 
 * \return HCI Error code.
 */
UCHAR hci_handle_write_eir_data(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR *eir_data;
    UINT16 eir_pkt_type;
    UCHAR eir_len;
    UCHAR fec_req;

    fec_req = hci_cmd_ptr->cmd_parameter[0];
    eir_data = &(hci_cmd_ptr->cmd_parameter[1]);

    /* Parameter checks */
    if(fec_req > 1)
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }
    if(hci_cmd_ptr->param_total_length!=(MAX_EIR_DATA_LEN + 1))
    {
        return EXTENDED_INQUIRY_RESPONSE_TOO_LARGE;
    }
    eir_len = hci_get_eir_data_len(eir_data);
    if(eir_len > 240)
    {
        return EXTENDED_INQUIRY_RESPONSE_TOO_LARGE;
    }

    eir_pkt_type = hci_get_eir_pkt_type(eir_len, fec_req);
    if(eir_pkt_type == (UINT16)0xFFFF)
    {
        return EXTENDED_INQUIRY_RESPONSE_TOO_LARGE;
    }
#ifdef _SUPPORT_PCA_ADJUST
    if (IS_BT41 && (bt_pca_manager.pca_clk_req_sent))
    {
        return COMMAND_DISALLOWED_ERROR;
    }
#endif    
    /* Copy to the data structures */
    lmp_self_device_data.eir_data.packet_type = eir_pkt_type;
    lmp_self_device_data.eir_data.length = eir_len;
    lmp_self_device_data.eir_data.fec_req = fec_req;
    memcpy(lmp_self_device_data.eir_data.data, eir_data, eir_len);

    /* Check and Enable EIR */
    hci_check_and_enable_eir_trans();

    return HCI_COMMAND_SUCCEEDED;
}

#ifdef COMPILE_SNIFF_MODE
/**
 * HCI Sniff Subrating command handler.
 * 
 * \param hci_cmd_ptr Pointer to the hci_cmd.
 * 
 * \return HCI Error code.
 */
UCHAR hci_handle_sniff_subrating(HCI_CMD_PKT *hci_cmd_ptr, 
                                        UINT16 *ext_param)
{
    UINT16 ce_index;
    UINT16 conn_handle;
    UINT16 max_latency;
    UINT16 min_remote_timeout;
    UINT16 min_local_timeout;
    UINT16 old_max_latency;
    UINT16 old_rem_timeout;
	LMP_CONNECTION_ENTITY *ce_ptr;
	SSR_DATA *ssr_data_ptr;

    /* Get parameters */
    BT_FW_EXTRACT_16_BITS(conn_handle, &hci_cmd_ptr->cmd_parameter[0]);
    BT_FW_EXTRACT_16_BITS(max_latency, &hci_cmd_ptr->cmd_parameter[2]);
    BT_FW_EXTRACT_16_BITS(min_remote_timeout, &hci_cmd_ptr->cmd_parameter[4]);
    BT_FW_EXTRACT_16_BITS(min_local_timeout, &hci_cmd_ptr->cmd_parameter[6]);

    *ext_param = conn_handle;

    /* Parameter checks */
    if(hci_cmd_ptr->param_total_length != 8)
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    if (LMP_GET_CE_INDEX_FROM_CONN_HANDLE(conn_handle,&ce_index) != API_SUCCESS)
    {
        return NO_CONNECTION_ERROR;
    }
	
	ce_ptr = &lmp_connection_entity[ce_index];
	ssr_data_ptr = &ce_ptr->ssr_data;

    /* Validate Parameters */
    if(max_latency == 0xFFFF || 
       min_remote_timeout == 0xFFFF || min_local_timeout == 0xFFFF)
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    /* Check for lmp_transaction in progress */
	if(ssr_data_ptr->lmp_ssr_state == LMP_SSR_TRANSITION)
	{
		return LMP_ERROR_TRANSACTION_COLLISION_ERROR;
	}

	/* Check for local & remote device features */
	if((lmp_feature_data.feat_page0[5] & SNIFF_SUBRATING) == 0)
	{
		return UNSUPPORTED_FEATURE_OR_PARAMETER_ERROR;
	}

	if((ce_ptr->feat_page0[5] & SNIFF_SUBRATING) == 0)
	{
		return UNSUPPORTED_REMOTE_FEATURE_ERROR;
	}

	old_max_latency = ssr_data_ptr->max_latency;
	old_rem_timeout = ssr_data_ptr->min_remote_timeout;

	/* Store the parameters */
	ssr_data_ptr->max_latency = max_latency;
	ssr_data_ptr->min_remote_timeout = min_remote_timeout;
	ssr_data_ptr->min_local_timeout = min_local_timeout;

    /* Try to initiate ssr_req */
    if(old_max_latency != max_latency ||
       old_rem_timeout != min_remote_timeout)
    {
        lmp_initiate_sniff_subrating(ce_index);
    }

    return HCI_COMMAND_SUCCEEDED;
}
#endif /* COMPILE_SNIFF_MODE */

/**
 * HCI_Write_Inquiry_Trans_Power command handler.
 * 
 * \param hci_cmd_ptr HCI Command Packet Pointer.
 * 
 * \return HCI Error code.
 */
UCHAR hci_handle_write_inq_trans_power(HCI_CMD_PKT *hci_cmd_ptr)
{
    CHAR tx_pwr;
    tx_pwr = (CHAR)(hci_cmd_ptr->cmd_parameter[0]);

    if ((hci_cmd_ptr->param_total_length != 1) ||
       ((tx_pwr < (CHAR) -70) || (tx_pwr > 20)))
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    /* TODO: Because we use fixed power steps, could we set Inquiry Tx Power ? 
       - austin */
       
    return HCI_COMMAND_SUCCEEDED;
} 


UCHAR hci_handle_enhanced_flush_command(HCI_CMD_PKT *hci_cmd_ptr)
{
    UINT16 connection_handle;
    UINT16 ce_index;
    UCHAR status = HCI_COMMAND_SUCCEEDED;
    UCHAR packet_type;
	LMP_CONNECTION_ENTITY *ce_ptr;    

    if (hci_cmd_ptr->param_total_length != 3)
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    BT_FW_EXTRACT_16_BITS(connection_handle, &hci_cmd_ptr->cmd_parameter[0]);
    packet_type = hci_cmd_ptr->cmd_parameter[2];
    if (packet_type != 0)
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    if (LMP_GET_CE_INDEX_FROM_CONN_HANDLE(connection_handle,&ce_index)
                                                         != API_SUCCESS)
    {
        return NO_CONNECTION_ERROR;
    }

    ce_ptr = &lmp_connection_entity[ce_index];

    /* Set as flush (automatically-flushable packets) */
    aclq_acl_enhanced_flush(ce_ptr->am_addr,
                 L_CH_L2CAP_START, ce_ptr->phy_piconet_id);

    ce_ptr->enhanced_flush = TRUE;
	
    return status;
}

/* -----------------External functions------------------- */

/** 
 * Checks and enables/disables EIR (transmit) feature.
 * 
 * \return None.
 */
void hci_check_and_enable_eir_trans(void)
{
    EIR_DATA *eir_data;
    eir_data = &lmp_self_device_data.eir_data;
    
    /* Disable EIR */
    /* move here to avoid inquiry scan early but EIR bit is set and EIR data 
       is not ready in the broadcast tx fifo - austin */
    bb_enable_eir(FALSE);

    if (eir_data->length != 0)
    {
        /* Enable EIR */
        bb_write_eir_data(eir_data->length, eir_data->packet_type, 
                          eir_data->data);
        bb_enable_eir(TRUE);
    }
}

/**
 * Checks and enables/disables EIR Recv feature.
 * 
 * \return None.
 */
void hci_check_and_enable_eir_recv(void)
{
    if (lmp_self_device_data.inquiry_mode == HCI_INQ_RESULT_EVENT_WITH_EIR
#ifdef ENABLE_SCO
		&& lmp_self_device_data.total_no_of_sco_conn == 0 
#endif
#ifdef COMPILE_ESCO
           && lmp_self_device_data.number_of_esco_connections == 0
#endif
      )
    {
        bb_enable_eir_recv(TRUE);
    }
    else
    {
        bb_enable_eir_recv(FALSE);
    }
    return;
}

#ifdef SCO_OVER_HCI
#ifdef COMPILE_ESCO
/**
 * HCI_write_default_errorenous_data_reporting command handler.
 * 
 * \param hci_cmd_ptr HCI Command Packet Pointer.
 * 
 * \return HCI Error code.
 */
UCHAR hci_handle_write_default_erroneous_data_reporting(
        HCI_CMD_PKT *hci_cmd_ptr)
{
    if ((hci_cmd_ptr->param_total_length != 1) ||
        (hci_cmd_ptr->cmd_parameter[0] > 0x1))
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }
    lmp_self_device_data.default_erroneous_data_reporting =
                                (UCHAR) hci_cmd_ptr->cmd_parameter[0];

    if (lmp_self_device_data.default_erroneous_data_reporting)
    {
        lmp_esco_over_codec = FALSE;
#ifdef BZ_2_1_2_CODEC
        sync_link_codec_state = OVER_HCI;
#else
        sync_link_codec_state = OVER_NULL;
#endif
    }
    else
    {   
        //RT_BT_LOG(RED,LC_SYNC_LINKS_293,0,0);
        lmp_esco_over_codec = TRUE;
        sync_link_codec_state = OVER_CODEC;
    }  
    
    return HCI_COMMAND_SUCCEEDED;
}
#endif
#endif //#ifdef SCO_OVER_HCI


/**
 * Handles all the host controller and baseband for the Bluetooth Version 2_1.
 * This function is called from the hci_task.c if the opcode
 * does not match any of the 1.1/1.2 Version opcodes.
 *
 * \param hci_cmd_ptr Pointer to HCI_CMD_PKT.
 * \param sent_event  Sets the variable if event is sent.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_2_1_hc_bb_command(HCI_CMD_PKT *hci_cmd_ptr, 
                                   UCHAR *sent_event_flag)
{
    UCHAR ret_error = HCI_COMMAND_SUCCEEDED;
    
    switch(hci_cmd_ptr->cmd_opcode)
    {
        case HCI_ENHANCED_FLUSH_OPCODE:
            ret_error =
                hci_handle_enhanced_flush_command(hci_cmd_ptr);
            *sent_event_flag = TRUE;
            hci_generate_command_status_event(hci_cmd_ptr->cmd_opcode,
                ret_error);
            break;

        case HCI_READ_EXTENDED_INQUIRY_RESPONSE_OPCODE:
            /* Do nothing - see events.c for implementation */
            break;

        case HCI_WRITE_EXTENDED_INQUIRY_RESPONSE_OPCODE:           
            ret_error = hci_handle_write_eir_data(hci_cmd_ptr);

#ifdef _3DD_FUNCTION_SUPPORT_
#ifdef _SUPPORT_CSB_TRANSMITTER_
            if (ret_error == HCI_COMMAND_SUCCEEDED)
            {
                bt_csb_check_and_set_csb_or_rp_mode();
            }                
#endif 
#endif
            break;

        case HCI_READ_INQUIRY_RES_TRANS_POWER_LEVEL_OPCODE:
            /* Do nothing - see events.c for implementation */
            break;

        case HCI_WRITE_INQUIRY_TRANS_POWER_LEVEL_OPCODE:
            ret_error = hci_handle_write_inq_trans_power(hci_cmd_ptr);
            break;
#ifdef SCO_OVER_HCI
        case HCI_READ_DEFAULT_ERRONEOUS_DATA_REPORTING_OPCODE:
            /* handled at generate command complete function */
            break;
        case HCI_WRITE_DEFAULT_ERRONEOUS_DATA_REPORTING_OPCODE:
            ret_error =
                hci_handle_write_default_erroneous_data_reporting(hci_cmd_ptr);
            break;
#endif //#ifdef SCO_OVER_HCI

        default:
            ret_error = UNKNOWN_HCI_COMMAND_ERROR;
            break;
    }

    return ret_error;
}
/**
 * Handles all the link policy commands for the Bluetooth Version 2_1.
 * This function is called from the hci_task.c if the opcode
 * does not match any of the 1.1/1.2 Version opcodes.
 *
 * \param hci_cmd_ptr Pointer to HCI_CMD_PKT.
 * \param sent_event  Sets the variable if event is sent.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_2_1_link_policy_command(HCI_CMD_PKT *hci_cmd_ptr, 
                                   UCHAR *sent_event_flag)
{
    UCHAR ret_error = HCI_COMMAND_SUCCEEDED;
    UCHAR command_complete_event_flag = FALSE;
    UINT16 ext_param;
    
    switch(hci_cmd_ptr->cmd_opcode)
    {
#ifdef COMPILE_SNIFF_MODE
        case HCI_SNIFF_SUBRATING_OPCODE:
            ret_error = hci_handle_sniff_subrating(hci_cmd_ptr, &ext_param);
            command_complete_event_flag = TRUE;
            break;
#endif

        default:
            ret_error = UNKNOWN_HCI_COMMAND_ERROR;
            command_complete_event_flag = 0x2;
            break;
    }

    if (command_complete_event_flag == FALSE)
    {
        /* Send command status event*/
        hci_generate_command_status_event(hci_cmd_ptr->cmd_opcode, ret_error);
    }
    else if (command_complete_event_flag == TRUE)
    {
        /* Send command complete event*/
        hci_generate_command_complete_event(
                hci_cmd_ptr->cmd_opcode, ret_error, ext_param, NULL);
    }
    else
    {
        /* Generate no event. */
        /* NOTE: command_complete_event_flag is uchar and can be
         * something other than true/false
         */
    }
    
    return ret_error;
}

