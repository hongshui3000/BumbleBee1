/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  Implements the HCI command functions.
 *      It contains
 *          Link control command handling functions,
 *          Link Policy command handling functions,
 *          Host controller and Baseband handling comamnds,
 *          Information command handling functions.
 */

/********************************* Logger *************************/ 
enum { __FILE_NUM__= 8};
/********************************* Logger *************************/

/* ========================= Include File Section ========================= */
#include "bt_fw_hci_internal.h"
#include "bt_fw_acl_q.h"
#include "lmp.h"
#include "lmp_pdu_q.h"
#include "bz_debug.h"
#include "bz_auth.h"
#include "mem.h"
#include "UartPrintf.h"

#include "lmp_2_1.h"

#ifdef LE_MODE_EN
#include "le_ll.h"
#include "le_hci_4_0.h"
#endif
#ifdef _DAPE_TEST_NO_ROLE_SW_WHEN_AU_RAND
#include "bz_auth_internal.h"
#endif
#ifdef _DAPE_NO_ROLE_SW_WHEN_ROLE_SW
UINT16 g_role_switch_status = 0;
#endif                    

/* ==================== Structure declaration Section ===================== */
typedef UCHAR (*MASTER_LINK_KEY_CMD_HANDLER)(UINT16 ce_index,
        LMP_CONNECTION_ENTITY* ce_ptr, void* user_arg);

/* ================== For rom code patch function point ================== */

#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8723_RCP_
PF_ROM_CODE_PATCH_FUNC rcp_hci_handle_sniff_mode_command_get_slot = NULL;
#endif
#ifdef _DAPE_8723_RCP_
PF_ROM_CODE_PATCH_FUNC rcp_hci_handle_accept_conn_func = NULL;
#endif
#endif

/* ===================== Variable Declaration Section ===================== */

#ifdef TEST_MODE
extern UCHAR lc_tci_pause_flag;
extern UINT8 test_mode_sched_acl_pkt;
extern UINT8 test_mode_tx_mode_force_ack;
extern UINT8 test_mode_tx_mode_buf_in_used;
#endif

/* ================== Static Function Prototypes Section ================== */
#ifdef ENABLE_SCO
UCHAR hci_handle_change_sco_conn_pkt_type(UINT16 conn_handle,
        UINT16 sco_ce_index, UINT16 host_pkt_types,
        UCHAR *hci_generate_ccpt_event_flag);
#endif

void hci_queue_conn_packet_type_changed_event(UINT16 ce_index);
extern void hci_generate_read_lmp_handle_complete_event(UCHAR  status, UINT16 cmd_opcode,
												 UINT16 connection_handle, UCHAR lmp_handle);
/* ===================== Function Definition Section ====================== */
/**
 * Extracts and validates the inquiry command parameters.
 *
 * \param hci_cmd_ptr Pointer to HCI_CMD_PKT.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI_Error code.
 *
 */
UCHAR hci_handle_inquiry_command(HCI_CMD_PKT *hci_cmd_ptr)
{
	UINT8 temp_var;

	temp_var = hci_validate_inquiry_cmd_params(hci_cmd_ptr);

	if (temp_var != HCI_COMMAND_SUCCEEDED)
	{
		return ((UCHAR)temp_var);
	}

	/* Update number of responses parameter */
	lmp_num_inq_resp_expected = hci_cmd_ptr ->cmd_parameter[4];

	temp_var = lc_handle_baseband_commands(hci_cmd_ptr);

	if (temp_var != HCI_COMMAND_SUCCEEDED)
	{
		return temp_var;
	}

	hci_generate_command_status_event(
		HCI_INQUIRY_OPCODE, HCI_COMMAND_SUCCEEDED);

	return HCI_COMMAND_SUCCEEDED ;
}

#ifdef COMPILE_PERIODIC_INQUIRY
/**
 * Extracts and validates the inquiry command parameters.
 *
 * \param hci_cmd_ptr Pointer to HCI_CMD_PKT.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_periodic_inquiry_command(HCI_CMD_PKT *hci_cmd_ptr)
{
    UINT32 lap;
    UCHAR  inquiry_length;
    UINT16 periodic_inquiry_max_length;
    UINT16 periodic_inquiry_min_length;
	UCHAR temp;

	temp = hci_validate_periodic_inquiry_cmd_params(hci_cmd_ptr);

	if (temp != HCI_COMMAND_SUCCEEDED)
	{
		return temp;
	}

	/* Extract the Inquiry length */
	inquiry_length = hci_cmd_ptr->cmd_parameter[7];
	
	BT_FW_EXTRACT_16_BITS(periodic_inquiry_max_length,
                          &(hci_cmd_ptr->cmd_parameter[0]));

    BT_FW_EXTRACT_16_BITS(periodic_inquiry_min_length,
                          &(hci_cmd_ptr->cmd_parameter[2]));

    /* Extract LAP from command packet */
    BT_FW_EXTRACT_24_BITS(lap,&hci_cmd_ptr->cmd_parameter[4]);

	/* Update number of responses parameter */
	lmp_num_inq_resp_expected = hci_cmd_ptr ->cmd_parameter[8];

    if (lc_handle_periodic_inquiry(periodic_inquiry_max_length,
           periodic_inquiry_min_length,lap, inquiry_length)!=API_SUCCESS)
    {
        return HARDWARE_FAILURE_ERROR ;
    }

    return HCI_COMMAND_SUCCEEDED ;
}
#endif /* COMPILE_PERIODIC_INQUIRY */


/**
 * Checks if SCO connection occupies full bandwidth.
 * Doesnot check for eSCO connections because 'reserved' slot
 * fullbandwidth is currently not supported.
 *
 * \param None.
 *
 * \return UCHAR TRUE or FALSE
 *
 */
UCHAR hci_is_full_bandwidth(void)
{
    UCHAR full_band = FALSE;

#ifdef ENABLE_SCO
	switch(lmp_self_device_data.total_no_of_sco_conn)
    {
        case 1:
            if (lmp_self_device_data.sco_pkt_type == HV1)
            {
                full_band = TRUE;
            }
            break;

        case 2:
            if (lmp_self_device_data.sco_pkt_type == HV2)
            {
                full_band = TRUE;
            }
            break;

        case 3:
            if (lmp_self_device_data.sco_pkt_type == HV3)
            {
                full_band = TRUE;
            }
            break;

        default:
            break;
    }
#endif

    return full_band;
}

/**
 * Allocates an entry in the connection entity database
 * and assigns the AM address. It extracts the command packet and
 * updates the connection entity database.
 * It then passes the command to the LC module.
 *
 * \param hci_cmd_ptr Pointer to HCI_CMD_PKT.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_create_connection_command(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR am_addr;
    UINT16 ce_index;
    UCHAR index;
    UCHAR temp_bd_addr[LMP_BD_ADDR_SIZE];
    UINT16 temp_packet_type;
    UINT16 temp_page_scan_repetition_mode;
    UINT16 temp_page_scan_mode;
    UINT16 temp_allow_role_switch;
    UINT16 clock_offset;
    UCHAR status;
    LMP_CONNECTION_ENTITY *ce_ptr;

#ifdef TEST_MODE
    switch(lmp_self_device_data.test_mode)
    {
        case HCI_LOCAL_LOOPBACK_MODE: /* Fall through. */
            /* Create Connection not allowed when the devie is
               in Local lpbk mode
             */

        case HCI_DEVICE_UNDER_TEST_MODE:
            /*
             * Create connection not allowed unless master-slave
             * switch is provided. If Role switch is allowed, then
             * the remote device (tester) should be the Master
             * after creating the connection.
             *
             * Currently Create connection is not allowed!!
             */
             return COMMAND_DISALLOWED_ERROR;

        case HCI_REMOTE_LOOPBACK_MODE:
            /* Only one connection should be alive for the Remote
               Loopback mode
             */
                for(index=0; index < LMP_MAX_CE_DATABASE_ENTRIES; index++)
                {
                    if (lmp_connection_entity[index].ce_status != LMP_STANDBY)
                    {
                        return COMMAND_DISALLOWED_ERROR;
                    }
                }
                break;

        default:
                break;
    }

    /* Reset the index */
    index = 0;
#endif /* TEST_MODE */

	status = hci_validate_create_connection_cmd_params(hci_cmd_ptr);
	if (status != HCI_COMMAND_SUCCEEDED)
	{
		return status;
	}

	/* Check if free AM_ADDR available */
	status = lmp_allocate_am_addr(&am_addr, &ce_index);

	if (status != API_SUCCESS)
	{
		RT_BT_LOG(GRAY, BT_FW_HCI_CMDS_295, 0, 0);
		return status;
	}
    
	ce_ptr = &lmp_connection_entity[ce_index];
   
	memcpy(temp_bd_addr, &hci_cmd_ptr->cmd_parameter[0], LMP_BD_ADDR_SIZE);

    RT_BT_LOG(BLUE, MSG_HCI_CREATE_CONNECT, 9, 
                            temp_bd_addr[0], temp_bd_addr[1], 
                            temp_bd_addr[2], temp_bd_addr[3], 
                            temp_bd_addr[4], temp_bd_addr[5], 
                            am_addr, ce_index, ce_ptr->phy_piconet_id);

	index = LMP_BD_ADDR_SIZE ;

	/* Check the Packet type */
	BT_FW_EXTRACT_16_BITS(temp_packet_type, &(hci_cmd_ptr->cmd_parameter[index]));
	index += 2 ;

	/* Page scan repetition mode */
	temp_page_scan_repetition_mode = (UCHAR) hci_cmd_ptr->cmd_parameter[index];
	index++;

	temp_page_scan_mode = (UCHAR) hci_cmd_ptr->cmd_parameter[index];
	index++;

	/*
	* LC module checks Clock offset valid flag(Bit 15), If it is invalid
	* value it programs baseband clock offset register with a value zero.
	*/
	BT_FW_EXTRACT_16_BITS(clock_offset, &(hci_cmd_ptr->cmd_parameter[index]));
	index += 2;

	temp_allow_role_switch = hci_cmd_ptr->cmd_parameter[index];
	index++;

	memcpy(ce_ptr->bd_addr, temp_bd_addr, LMP_BD_ADDR_SIZE);

	ce_ptr->connection_type.packet_type = temp_packet_type;
	ce_ptr->page_scan_repetition_mode = (UCHAR)temp_page_scan_repetition_mode;
	ce_ptr->page_scan_mode = (UCHAR)temp_page_scan_mode ;

	ce_ptr->allow_role_switch = (UCHAR)temp_allow_role_switch ;
	ce_ptr->am_addr = am_addr ;
	ce_ptr->connection_type.link_type = ACL_LINK;

	lc_update_pkts_allowed(ce_index);

#ifdef _CCH_PAGE_CON_
       ce_ptr->connect_reason = CON_REASON_CREATE_CON;
#endif	

	/* If MSB is 1, the clock-offset is valid. */
	if (clock_offset & 0x8000)
	{
		ce_ptr->clock_offset = clock_offset;

		ce_ptr->clock_offset = (UINT16) (ce_ptr->clock_offset & 0x7fff);

		RT_BT_LOG(GRAY, BT_FW_HCI_CMDS_334, 1, ce_ptr->clock_offset);
	}
	else
	{
		ce_ptr->clock_offset = 0;
	}

	status = lc_handle_baseband_commands(hci_cmd_ptr);
	if (status != HCI_COMMAND_SUCCEEDED)
	{
                /* Release AM_ADDR if any error */
	        if (lmp_release_am_addr_ppi(am_addr, ce_ptr->phy_piconet_id) 
                                    != API_SUCCESS)
		{
			RT_BT_LOG(GRAY, BT_FW_HCI_CMDS_352, 0, 0);
		}
		LMP_REMOVE_BD_ADDR_FROM_HASH(temp_bd_addr);
		return status;
	}

	lmp_set_ce_status(ce_index, LMP_PAGING);

	hci_generate_command_status_event(
		HCI_CREATE_CONNECTION_OPCODE, 
		HCI_COMMAND_SUCCEEDED);
    
    return HCI_COMMAND_SUCCEEDED ;
}

/**
 * Interprets the Accept Connection Request command which
 * helps the Link Manager to decide to accept the connection request.
 *
 * \param hci_cmd_ptr Pointer to HCI_CMD_PKT.
 * \param sent_event_flag Set to TRUE if command status event is generated.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_accept_connection_request_command(HCI_CMD_PKT *hci_cmd_ptr,
        UCHAR *sent_event_flag)
{
    UINT16 ce_index;
    UCHAR bd_addr[LMP_BD_ADDR_SIZE];
    UCHAR role;
    LMP_CONNECTION_ENTITY* ce_ptr;
	UCHAR ret_val;
	
	memcpy(&bd_addr[0], &hci_cmd_ptr->cmd_parameter[0], LMP_BD_ADDR_SIZE);
    if (LMP_GET_CE_INDEX_FROM_BD_ADDR(bd_addr, &ce_index) == API_FAILURE)
    {
        return NO_CONNECTION_ERROR;
    }

    role = hci_cmd_ptr->cmd_parameter[6];
    if (role > 1)
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    ce_ptr = &lmp_connection_entity[ce_index];

#ifdef _ROM_CODE_PATCHED_
#ifdef _DAPE_8723_RCP_
    UCHAR return_status;
    if (rcp_hci_handle_accept_conn_func != NULL)
    {
        if (rcp_hci_handle_accept_conn_func((void *)&return_status, 
			            ce_index, hci_cmd_ptr, sent_event_flag))
        {
            return return_status;
        }
    }
#endif
#endif
#ifdef _DAPE_TEST_NO_SW_TO_MASTER
role = SLAVE;
#endif
    switch(ce_ptr->ce_status)
    {
        case LMP_WAITING_FOR_CONN_ACCEPT:
            *sent_event_flag = TRUE;
            hci_generate_command_status_event(hci_cmd_ptr->cmd_opcode,
                    HCI_COMMAND_SUCCEEDED);
            
            OS_DELETE_TIMER(&ce_ptr->conn_accept_timer_handle);

#ifdef COMPILE_ROLE_SWITCH
			if (role == SLAVE)
			{
                /* remain the slave for this connection. The LM will NOT 
                   perform the role switch */
#ifdef _AUTO_ROLE_SW_TO_MASTER_WHEN_SCO
                if ( 
#ifdef ENABLE_SCO
                    (lmp_self_device_data.total_no_of_sco_conn !=0)
#endif
#ifdef COMPILE_ESCO
                    || (lmp_self_device_data.number_of_esco_connections !=0)
#endif
                    )
                {
                    ret_val = lmp_check_for_role_switch_in_scatternet(ce_index);

                    if (ret_val != TRUE)
                    {
                        lmp_accept_host_connection_request_pdu(ce_index);
                    }
#ifdef _DAPE_TEST_NO_ROLE_SW_WHEN_NOT_ALLOWED
                    else
#endif
                    {
                        lmp_start_role_switch_during_conn_as_slave(ce_index, bd_addr);
                    }
                }
                else
                {
                    lmp_accept_host_connection_request_pdu(ce_index);
                }
#else
                lmp_accept_host_connection_request_pdu(ce_index);
#endif
            }
            else
            {
                /* Become the Master for this connection. The LM will perform
                   the role switch */
                ret_val = lmp_check_for_role_switch_in_scatternet(ce_index);
                UCHAR feature;
                feature = ce_ptr->features[0][0];
                if((feature & LMP_SWITCH_FEATURE) == FALSE)
                {
                    ret_val = FALSE;
                }
#ifdef _DAPE_NO_ROLE_SW_WHEN_ROLE_SW
                if (g_role_switch_status != 0)
                {
                    ret_val = FALSE;
                }
#endif
                if (ret_val != TRUE)
                {
                    lmp_accept_host_connection_request_pdu(ce_index);
                }
#ifdef _DAPE_TEST_NO_ROLE_SW_WHEN_NOT_ALLOWED
                else
#endif
                {
                    lmp_start_role_switch_during_conn_as_slave(ce_index, bd_addr);
#ifdef _DAPE_NO_ROLE_SW_WHEN_ROLE_SW
                    g_role_switch_status |= (BIT0 << ce_index);
#endif                    
                }
            }
#else
			lmp_accept_host_connection_request_pdu(ce_index);
#endif
            break;

#ifdef ENABLE_SCO
        case LMP_ADDING_SCO_LINK:
#ifdef COMPILE_SNIFF_MODE
        case LMP_ADDING_SCO_LINK_DURING_SNIFF:
#endif
        {
            UINT16 sco_1_1_pkt_type, sco_1_2_pkt_type;
            UCHAR ret_error;
            UCHAR* cmd_parameter;
            HCI_LINK_TYPE link_type;

            /* Reuse the same HCI_CMD_PKT */
            cmd_parameter = &hci_cmd_ptr->cmd_parameter[0];

            /* Tx Bandwidth */
            cmd_parameter[6] = 0x40;
            cmd_parameter[7] = 0x1F;
            cmd_parameter[9] = cmd_parameter[8] = 0;

            /* RX Bandwidth */
            cmd_parameter[10] = 0x40;
            cmd_parameter[11] = 0x1F;
            cmd_parameter[13] = cmd_parameter[12] = 0;

            /* Max latency */
            cmd_parameter[14] = 0x10;
            cmd_parameter[15] = 0x00;

            /* Content Format */
            cmd_parameter[16] = (UCHAR)
                (lmp_self_device_data.voice_setting & 0xFF);
            cmd_parameter[17] = (UCHAR)
                ((lmp_self_device_data.voice_setting & 0xFF00) >> 8);

            /* Retransmission Effort */
            cmd_parameter[18] = 0;

            /* Packet type */
			sco_1_1_pkt_type = (UINT16)(
				(lmp_feature_data.feat_page0[1] << 2)
				& ALL_HCI_1_1_SCO_PKT_TYPES);

			sco_1_2_pkt_type =
                SCO_PKT_TYPE_1_1_HCI_TO_1_2_HCI(sco_1_1_pkt_type);
            sco_1_2_pkt_type = (UINT16)
                (sco_1_2_pkt_type | HCI_ESCO_EDR_PKT_TYPE_UMASK);
            cmd_parameter[19] = (UCHAR)(sco_1_2_pkt_type);
            cmd_parameter[20] = (UCHAR)(sco_1_2_pkt_type >> 8);

            *sent_event_flag = TRUE;
            ret_error = hci_handle_accept_sync_conn_req_command(hci_cmd_ptr,
                    &ce_index, &link_type, TRUE);
            if (ret_error != HCI_COMMAND_SUCCEEDED)
            {
                hci_generate_connection_complete_event(ret_error,
                        ce_ptr->connection_type.connection_handle,
                        ce_ptr->bd_addr, SCO_LINK,
                        (UCHAR) bz_auth_get_encryption_mode(ce_index));
            }

            return HCI_COMMAND_SUCCEEDED;
        }
#endif /* ENABLE_SCO */

        default:
            return COMMAND_DISALLOWED_ERROR;
    }

    return HCI_COMMAND_SUCCEEDED;
}

/**
 * Interprets the Reject Connection Request command which
 * helps the Link Manager to decide to reject the connection request.
 *
 * \param hci_cmd_ptr Pointer to HCI_CMD_PKT.
 * \param sent_event_flag Set to TRUE if command status event is generated.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 */
UCHAR hci_handle_reject_connection_request_command(HCI_CMD_PKT *hci_cmd_ptr,
        UCHAR *sent_event_flag)
{
    UINT16 ce_index;
    UCHAR bd_addr[LMP_BD_ADDR_SIZE];
    UCHAR reason;
    LMP_CONNECTION_ENTITY* ce_ptr;

    memcpy(&bd_addr[0], &hci_cmd_ptr->cmd_parameter[0], LMP_BD_ADDR_SIZE);
    if (LMP_GET_CE_INDEX_FROM_BD_ADDR(bd_addr, &ce_index) == API_FAILURE)
    {
        X_BZ_ASSERT(0, "Host could be misbehaving");
        return NO_CONNECTION_ERROR;
    }

    reason = hci_cmd_ptr->cmd_parameter[6];
    if ((reason < 0x0D) || (reason > 0x0F))
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    ce_ptr = &lmp_connection_entity[ce_index];
    switch(ce_ptr->ce_status)
    {
         case LMP_WAITING_FOR_CONN_ACCEPT:
            OS_DELETE_TIMER(&ce_ptr->conn_accept_timer_handle);

            lmp_send_lmp_not_accepted(ce_index, LMP_HOST_CONNECTION_REQ_OPCODE,
                    REMOTE_DEV_TID, reason);
            lmp_handle_acl_disconnect(ce_index, reason);
            break;

#ifdef ENABLE_SCO
        case LMP_ADDING_SCO_LINK:
#ifdef COMPILE_SNIFF_MODE
        case LMP_ADDING_SCO_LINK_DURING_SNIFF:
#endif
        {
            HCI_LINK_TYPE link_type;

            *sent_event_flag = TRUE;
            reason = hci_handle_reject_sync_conn_req_command(hci_cmd_ptr,
                    &ce_index, &link_type);
            if (reason != HCI_COMMAND_SUCCEEDED)
            {
                hci_generate_connection_complete_event(reason,
                        ce_ptr->connection_type.connection_handle,
                        ce_ptr->bd_addr, SCO_LINK, 
                        (UCHAR) bz_auth_get_encryption_mode(ce_index));
            }
            break;
        }
#endif /* ENABLE_SCO */

        default:
            BZ_ASSERT(0, "I am not the culprit.. whose problem is this?");
            break;
    }

    return HCI_COMMAND_SUCCEEDED;
}

/**
 * Interprets the Disconnect Command and sets
 * the appropriate registers in the baseband and initiates the
 * Disconnect Process.
 *
 * \param hci_cmd_ptr Pointer to HCI_CMD_PKT.
 * \param sent_event_flag Set to TRUE if command status event is generated.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 */
UCHAR hci_handle_disconnect_command(HCI_CMD_PKT *hci_cmd_ptr,
        UCHAR *sent_event_flag)
{
    UCHAR reason;
    UINT16 conn_handle;
    UINT16 ce_index;
    LMP_CONNECTION_ENTITY *ce_ptr;

    BT_FW_EXTRACT_16_BITS(conn_handle, &hci_cmd_ptr->cmd_parameter[0]);
    reason = hci_cmd_ptr->cmd_parameter[2];
    *sent_event_flag = (UCHAR) FALSE;

    switch(reason)
    {
        case AUTHENTICATION_FAILURE_ERROR:
        case CONNECTION_TERMINATED_USER_ERROR:
        case CONNECTION_TERMINATED_LOW_RESOURCES_ERROR:
        case CONNECTION_TERMINATED_POWER_OFF_ERROR:
        case UNSUPPORTED_REMOTE_FEATURE_ERROR:
        case PAIRING_WITH_UNIT_KEY_NOT_SUPPPORTED_ERROR:
        case UNACCEPTABLE_CONNECTION_INTERVAL:  // After Spec4.1
            break;

        default:
            return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

#ifdef LE_MODE_EN
    if (IS_BT40)
    {
        /* This macro will delete all the le connections - austin */
        if (ll_handle_acl_disconnect(conn_handle, reason, TRUE) == BT_FW_SUCCESS)
        {
            return HCI_COMMAND_SUCCEEDED;
        }
    }
#endif

#ifdef COMPILE_ESCO
    /* This macro will delete all the esco connections */
    if (lmp_handle_esco_disconnect(conn_handle, reason, sent_event_flag)
            == BT_FW_SUCCESS)
    {
        return HCI_COMMAND_SUCCEEDED;
    }
#endif

#ifdef ENABLE_SCO
    if (lmp_handle_sco_disconnect(conn_handle, reason, sent_event_flag)
            == API_SUCCESS)
    {
        return HCI_COMMAND_SUCCEEDED;
    }
#endif /* ENABLE_SCO */

    if (LMP_GET_CE_INDEX_FROM_CONN_HANDLE(conn_handle, &ce_index)
            != API_SUCCESS)
    {
        return NO_CONNECTION_ERROR;
    }

    ce_ptr = &lmp_connection_entity[ce_index];

#ifdef TEST_MODE
    if (ce_ptr->test_mode_info.test_state == TEST_STARTED )
    {
        lc_tci_pause_flag = TRUE;

        test_mode_sched_acl_pkt = FALSE;   
        test_mode_tx_mode_force_ack = FALSE;
        test_mode_tx_mode_buf_in_used = FALSE;
        pf_switch_hci_dma_parameter(DUT_TEST_MODE_FIFO_FREE);

#ifdef _DUT_DELAYED_LOOPBACK_MODE_
        dut_mode_manager.buf[0].ppkt = NULL;
        dut_mode_manager.buf[1].ppkt = NULL;
        dut_mode_manager.work_space = 0;
#endif

        lc_flush_piconet1_tx_fifo();
    }
#endif

    ce_ptr->disconnect_reason = reason;

#ifdef _CCH_PAGE_CON_
    ce_ptr->connect_reason = 0;
#endif	

#ifdef COMPILE_PARK_MODE
    if (ce_ptr->ce_status == LMP_PARK_MODE)
    {
        ce_ptr->low_power_disconnect_state = UNPARK_DISCONNECT;
        hci_handle_exit_park_mode_command(ce_index);

        return HCI_COMMAND_SUCCEEDED;
    }
#endif /* COMPILE_PARK_MODE */

#ifdef COMPILE_SNIFF_MODE
    if (ce_ptr->in_sniff_mode == TRUE)
    {
		hci_handle_exit_sniff_mode_command(ce_index);
        ce_ptr->low_power_disconnect_state = UNSNIFF_DISCONNECT;

        return HCI_COMMAND_SUCCEEDED;
    }
#endif /* COMPILE_PARK_MODE */

#ifdef COMPILE_HOLD_MODE
    if (ce_ptr->ce_status == LMP_HOLD_MODE)
    {
        ce_ptr->low_power_disconnect_state = HOLD_DISCONNECT;
        return HCI_COMMAND_SUCCEEDED;
    }
#endif /* COMPILE_HOLD_MODE */

    if (lmp_handle_acl_disconnect(ce_index, reason) == API_FAILURE)
    {
        return UNSPECIFIED_ERROR;
    }

    return HCI_COMMAND_SUCCEEDED;
}

#if defined(ADD_SCO) && defined(ENABLE_SCO)
/** 
 * Handles HCI_ADD_SCO_CONNECTION_COMMAND from the host.
 * 
 * \param hci_cmd_ptr HCI Command Packet pointer.
 * \param sent_event_flag Set to TRUE if command status event is generated.
 * 
 * \return HCI_COMMAND_SUCCEEDED, if the operation was successful. A valid
 *         error code, otherwise.
 */
UCHAR hci_handle_add_sco_connection_command(HCI_CMD_PKT* hci_cmd_ptr,
        UCHAR* sent_event_flag)
{
    UINT16 sco_1_1_pkt_type, sco_1_2_pkt_type;
    UCHAR ret_error;
    UCHAR* cmd_parameter;
    UINT16 ce_index;
    HCI_LINK_TYPE link_type;
    LMP_CONNECTION_ENTITY* ce_ptr;

    cmd_parameter = &hci_cmd_ptr->cmd_parameter[0];

    /* Packet type (We are going to overwrite it, so process this first) */
    sco_1_1_pkt_type = (UINT16)(cmd_parameter[3] << 8);
    sco_1_1_pkt_type = (UINT16)(sco_1_1_pkt_type | cmd_parameter[2]);
    sco_1_2_pkt_type = SCO_PKT_TYPE_1_1_HCI_TO_1_2_HCI(sco_1_1_pkt_type);
    sco_1_2_pkt_type = (UINT16)(sco_1_2_pkt_type | HCI_ESCO_EDR_PKT_TYPE_UMASK);
    
	/* Tx Bandwidth */ 
	cmd_parameter[2] = 0x40;
	cmd_parameter[3] = 0x1F;
	cmd_parameter[5] = cmd_parameter[4] = 0;

    /* Rx Bandwidth */
	cmd_parameter[6] = 0x40;
	cmd_parameter[7] = 0x1F;
	cmd_parameter[9] = cmd_parameter[8] = 0;

    /* Max latency */
    cmd_parameter[10] = 0x10;
    cmd_parameter[11] = 0x00;

    /* Voice Setting */
    cmd_parameter[12] = (UCHAR)(lmp_self_device_data.voice_setting & 0xFF);
    cmd_parameter[13] = (UCHAR)(lmp_self_device_data.voice_setting >> 8);

    /* Retransmission Effort */
    cmd_parameter[14] = 0;

    /* Set allowed packet types */
    cmd_parameter[15] = (UCHAR)(sco_1_2_pkt_type);
    cmd_parameter[16] = (UCHAR)(sco_1_2_pkt_type >> 8);

	RT_BT_LOG(GREEN, BT_FW_HCI_CMDS_750, 1, lmp_self_device_data.voice_setting);

    *sent_event_flag = TRUE; /* Setup_Sync will send the Cmd_Status_Evt */

    {
        SYNC_LINK_STATUS sync_link;
        ret_error = hci_handle_setup_sync_conn_command(hci_cmd_ptr,
                &ce_index, &link_type, TRUE, &sync_link);
    }

    ce_ptr= &lmp_connection_entity[ce_index];

    if (ret_error != HCI_COMMAND_SUCCEEDED)
    {
        hci_generate_connection_complete_event(ret_error,
                ce_ptr->connection_type.connection_handle,
                ce_ptr->bd_addr, SCO_LINK,
                (UCHAR) bz_auth_get_encryption_mode(ce_index));
    }

    return ret_error;
}
#endif /* ADD_SCO */

#ifdef ENABLE_SCO
UCHAR hci_handle_change_sco_conn_pkt_type(UINT16 conn_handle,
        UINT16 sco_ce_index, UINT16 host_pkt_types,
        UCHAR *hci_generate_ccpt_event_flag)
{
    UINT16 ce_index;
    UINT16 remote_pkt_types;
    UINT16 allowed_pkt_types;
    UCHAR reason;
    LMP_CONNECTION_ENTITY* ce_ptr;
    LMP_SCO_CONNECTION_DATA* sco_ce_ptr;
    LMP_SCO_CONNECTION_DATA* temp_sco_ce_ptr;

    sco_ce_ptr = &lmp_sco_connection_data[sco_ce_index];
    temp_sco_ce_ptr = &lmp_sco_connection_data[LMP_MAX_SCO_CONNECTIONS];
    ce_index = sco_ce_ptr->conn_entity_index;
    ce_ptr = &lmp_connection_entity[ce_index];

    switch (ce_ptr->ce_status)
    {
        case LMP_CONNECTED:
#ifdef COMPILE_SNIFF_MODE
        case LMP_SNIFF_MODE:    /* Fall through */
#endif
            break;
        default:
            return COMMAND_DISALLOWED_ERROR;
    }

    if (sco_ce_ptr->sco_conn_status != SCO_CONNECTED)
    {
        return COMMAND_DISALLOWED_ERROR;
    }

    remote_pkt_types = (UINT16)(ce_ptr->feat_page0[1] << 2);
    allowed_pkt_types = (UINT16)(host_pkt_types & remote_pkt_types
            & ALL_HCI_1_1_SCO_PKT_TYPES);
   
	if (allowed_pkt_types == 0)
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

	if (lmp_self_device_data.total_no_of_sco_conn > 1)
	{
		return COMMAND_DISALLOWED_ERROR;
	}

#ifndef _CCH_ESCO_SCA_
	if ( lc_check_if_device_is_in_scatternet() == TRUE)
	{
		//RT_BT_LOG(GRAY, BT_FW_HCI_CMDS_825, 0, 0);
		/* return UNSPECIFIED_ERROR; */
		return HOST_REJECTED_LIMITED_RESOURCES_ERROR;
	}
#endif

    reason = lmp_generate_sco_params_for_change_pkt_type(sco_ce_index,
            allowed_pkt_types, 8 /* DONT CARE */, 0);
    if (reason != BT_FW_SUCCESS)
    {
        return reason;
    }

    if (temp_sco_ce_ptr->pkt_type == sco_ce_ptr->pkt_type)
    {
        lmp_free_temp_sco_conn_entity();
        *hci_generate_ccpt_event_flag = 0x01;
        return HCI_COMMAND_SUCCEEDED;
    }

    lmp_send_sco_link_request_pdu(ce_index, LMP_MAX_SCO_CONNECTIONS,
            SELF_DEV_TID);
    if (ce_ptr->remote_dev_role == MASTER)
    {
        ce_ptr->lmp_expected_pdu_opcode |= lmp_get_opcode_mask(
                LMP_SCO_LINK_REQ_OPCODE, 0x0);
    }
    sco_ce_ptr->sco_conn_status = SCO_CHG_PARAMS;
    lmp_sco_set_trans_status(sco_ce_ptr, TRS_SELF_INITIATED);

    return HCI_COMMAND_SUCCEEDED;
}
#endif /* ENABLE_SCO */

/**
 * This function changes packet types can be used for a connection
 * that is currently established.
 *
 * \param hci_cmd_ptr Pointer to HCI_CMD_PKT.
 * \param sent_event_flag Set to TRUE if command status event is generated.
 * \param hci_generate_ccpt_event_flag if set ccpt event should be generated.
 * \param hci_max_slots_change_event_flag Set to True if max slots change
 *                                        event is sent to host.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_change_connection_packet_type_command(
        HCI_CMD_PKT *hci_cmd_ptr, UCHAR *sent_event_flag,
        UCHAR *hci_generate_ccpt_event_flag,
        UCHAR *hci_max_slots_change_event_flag)
{
#ifdef ENABLE_SCO
    UINT16 sco_ce_index;
#endif
    UINT16 conn_handle;
    UINT16 packet_type;
    UINT16 ce_index;
    UCHAR handle_cmd = TRUE;
    LMP_CONNECTION_ENTITY *ce_ptr;
    UINT32 new_max_slot;

    *sent_event_flag = FALSE;
    *hci_max_slots_change_event_flag = FALSE;

    BT_FW_EXTRACT_16_BITS(conn_handle, &hci_cmd_ptr->cmd_parameter[0]);
    BT_FW_EXTRACT_16_BITS(packet_type, &hci_cmd_ptr->cmd_parameter[2]);

#ifdef ENABLE_SCO
    /* Check if the command is for a SCO connection. */
    if (lmp_get_sco_ce_index_from_conn_handle(conn_handle, &sco_ce_index)
            == API_SUCCESS)
    {
        UCHAR reason;

        reason = hci_handle_change_sco_conn_pkt_type(conn_handle,
                sco_ce_index, packet_type, hci_generate_ccpt_event_flag);
        return reason;
    }
    else
#endif

    if (LMP_GET_CE_INDEX_FROM_CONN_HANDLE(conn_handle, &ce_index) ==
            API_FAILURE)
    {
        return NO_CONNECTION_ERROR;
    }

    ce_ptr = &lmp_connection_entity[ce_index];

    if (handle_cmd == FALSE)
    {
        *hci_generate_ccpt_event_flag = 0x00;
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    /* Store the packet type in CE. */
    ce_ptr->connection_type.packet_type = packet_type;


#ifndef COMPILE_SINGLE_SLOT_PACKETS_ONLY
    new_max_slot = lmp_calculate_max_slot_for_hci(ce_index);

    {
        if (new_max_slot == 1)
        {
            /* Update packets allowed. */
            lc_update_pkts_allowed(ce_index);

            hci_queue_conn_packet_type_changed_event(ce_index);
            return HCI_COMMAND_SUCCEEDED;
        }
    }

    if ( (ce_ptr->last_accepted_max_slot == 
            LMP_LAST_ACCEPTED_MAX_SLOT_INVALID) || 
                (new_max_slot > ce_ptr->last_accepted_max_slot) )
    {
        /* Send max slot req PDU to remote dev. */
        lmp_send_max_slot_req_pdu(ce_index, (UCHAR) new_max_slot);

        /* Set flag to generate event later. */
        ce_ptr->hci_cmd_bits |= CHANGE_CONN_PKT_TYPE_BIT_MASK;

		HCI_LOG_INFO(LOG_LEVEL_LOW, MAX_SLOT_REQ_QUEUED_FOR_CH_CON_PKT_TYPE_CMD,0,0);
    }
    else
    {
        /* Update packets allowed. */
        lc_update_pkts_allowed(ce_index);

        hci_queue_conn_packet_type_changed_event(ce_index);

		HCI_LOG_INFO(LOG_LEVEL_LOW,NO_MAX_SLOT_REQ_IS_SENT_FOR_CH_CON_PKT_TYPE_CMD,0,0);
    }
#else /* COMPILE_SINGLE_SLOT_PACKETS_ONLY */
    /* Generate connection packet type changed event here. */
    hci_queue_conn_packet_type_changed_event(ce_index);
#endif /* COMPILE_SINGLE_SLOT_PACKETS_ONLY */

    return HCI_COMMAND_SUCCEEDED;
}

/**
 * Queues Connection packet type changed command to hci-event-task.
 *
 * \param ce_index Index to CE for the connection.
 *
 * \return None.
 *
 */
void hci_queue_conn_packet_type_changed_event(UINT16 ce_index)
{
    OS_SIGNAL sig_send;

    sig_send.type = HCI_GEN_CONN_PKT_TYPE_CHGD_EVENT_SIGNAL;
    sig_send.param = (OS_ADDRESS)((UINT32)ce_index);

    if (OS_SEND_SIGNAL_TO_TASK (hci_event_task_handle, sig_send)
                != BT_ERROR_OK )
    {
		HCI_LOG_INFO(LOG_LEVEL_LOW, ERROR_SENDING_SIGNAL,0,0);
    }

    return;
}

/**
 * Gets the user-friendly name of the remote device.
 * It checks the connection exists for the given BD_ADDR, if the
 * connection not exists, it sends the command to the LC module for
 * starting the page.
 *
 * \param hci_cmd_ptr Pointer to HCI_CMD_PKT.
 * \param baseband_cmd_flag Set to TRUE if command status event is generated.
 *                          [Check]
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_remote_name_request_command(HCI_CMD_PKT *hci_cmd_ptr,
        UCHAR * baseband_cmd_flag)
{
    UCHAR am_addr;
    UINT16 ce_index;
    UCHAR index = 0;
    UCHAR temp_bd_addr[LMP_BD_ADDR_SIZE];
    UINT16 clock_offset;
    UCHAR page_scan_rep_mode;
    UCHAR page_scan_mode;
    UCHAR parameter_list[LMP_NAME_REQ_LEN];
    UCHAR status;
    LMP_CONNECTION_ENTITY *ce_ptr;

    memcpy(temp_bd_addr, &hci_cmd_ptr->cmd_parameter[index], LMP_BD_ADDR_SIZE);
    index += LMP_BD_ADDR_SIZE;

    /* Page scan repetition mode */
    page_scan_rep_mode = (UCHAR)hci_cmd_ptr->cmd_parameter[index];
    index++;

    /* Page scan mode */
    page_scan_mode = (UCHAR)hci_cmd_ptr->cmd_parameter[index];
    index++;
#ifdef _DAPE_NO_REMOTE_NAME_DURING_AUTH
    /* if the bd_addr is not our current link, and we are processing auth with
       any other link, then we do not process paging to reduce the connection
       complete time of other link. */
    if (LMP_GET_CE_INDEX_FROM_BD_ADDR(temp_bd_addr, &ce_index) != API_SUCCESS)
    {
        for (ce_index = 0; ce_index < LMP_MAX_CE_DATABASE_ENTRIES; ce_index ++)
        {
            LMP_CONNECTION_ENTITY *ce_ptr;
            ce_ptr = &lmp_connection_entity[ce_index];
            if (ce_ptr->entity_status == ASSIGNED)
            {
                BZ_AUTH_LINK_PARAMS* auth;
                auth = ce_ptr->auth;
                if (auth->sub_state != BZ_AUTH_SUB_STATE_IDLE)
                {
                    return COMMAND_DISALLOWED_ERROR;
                }                          
            }
        }
    }
#endif
    if (page_scan_mode || page_scan_rep_mode > 2)
    {
#if !defined(_CCH_RTL8723A_B_CUT)
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
#else
// _CCH_8723_A_ECO_
        if( g_efuse_lps_setting_3.iot_issc_rmr_par == 1 )
        {
// cch        
// issc 20110906  
// for issc error command parameter
             page_scan_mode = 0;
             page_scan_rep_mode = 0;
        }
        else
        {
            return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
        }	
#endif

    }

    /*
     * LC module checks Clock offset valid flag(Bit 15), If it is invalid
     * value it programs baseband clock offset register with a value zero.
     */
    BT_FW_EXTRACT_16_BITS(clock_offset, &(hci_cmd_ptr->cmd_parameter[index]));
    index += 2;
    if (clock_offset & 0x8000)
    {
        clock_offset = (UINT16) (clock_offset & 0x7FFF);
    }
    else
    {
        clock_offset = 0;
    }

    if (LMP_GET_CE_INDEX_FROM_BD_ADDR(temp_bd_addr, &ce_index) == API_SUCCESS)
    {
        /* Connection already exists send remote name request */
        ce_ptr = &lmp_connection_entity[ce_index];

        parameter_list[0] = LMP_NAME_REQ_OPCODE;
        parameter_list[2] = 0;
        ce_ptr->name_req_name_offset = 0;
#ifdef _REMOTE_NAME_RES_WRONG
        ce_ptr->name_length_offset_zero = 0;
#endif
        lmp_generate_pdu(ce_index, parameter_list, LMP_NAME_REQ_LEN,
                SELF_DEV_TID, LMP_NO_STATE_CHANGE);
        ce_ptr->lmp_expected_pdu_opcode |= lmp_get_opcode_mask(
                LMP_NAME_RES_OPCODE, 0x0);

        ce_ptr->hci_cmd_bits |= REMOTE_NAME_REQ_BIT_MASK;
    }
    else
    {
        /* We don't have a connection, so need to create a temproary BB level
         * connection to get the remote name.
         */
        if (hci_is_full_bandwidth() == TRUE)
        {
            return HOST_REJECTED_LIMITED_RESOURCES_ERROR;
        }

		/* Check if free AM_ADDR available */
		{
			UCHAR page_piconet_id;
			UCHAR ret_val;

			page_piconet_id = lc_allocate_piconet_id_for_paging();
			if (page_piconet_id == SCA_PICONET_INVALID)
			{
				return COMMAND_DISALLOWED_ERROR;
			}

			ret_val = lmp_allocate_am_addr(&am_addr, &ce_index);

			if (ret_val != API_SUCCESS)
			{
				return MAX_NUMBER_OF_CONNECTIONS_ERROR;
			}
		}

#ifdef _DAPE_TEST_NO_REMOTE_NAME_REQ_DURING_ROLE_SW
        if (lmp_mss_state != LMP_MSS_INIT)
        {
            return COMMAND_DISALLOWED_ERROR;
        }
#endif
        /* Allocated a connection entity */
        ce_ptr = &lmp_connection_entity[ce_index];

        ce_ptr->page_scan_repetition_mode = page_scan_rep_mode;
        ce_ptr->page_scan_mode = (UCHAR)page_scan_mode;
        ce_ptr->clock_offset = clock_offset;

        memcpy(ce_ptr->bd_addr, temp_bd_addr, LMP_BD_ADDR_SIZE);

        ce_ptr->am_addr = am_addr;
        ce_ptr->connection_type.link_type = ACL_LINK;

        status = lc_handle_baseband_commands(hci_cmd_ptr);

        if (status != HCI_COMMAND_SUCCEEDED)
        {
            if (lmp_release_am_addr_ppi(am_addr,
    		                        ce_ptr->phy_piconet_id) != API_SUCCESS)
    	    {
                RT_BT_LOG(GRAY, BT_FW_HCI_CMDS_352, 0, 0);
    	    }
	        LMP_REMOVE_BD_ADDR_FROM_HASH(temp_bd_addr);
            return status;
        }

        lmp_set_ce_status(ce_index, LMP_DURING_CONN_REMOTE_NAME_REQ);
    	lmp_self_device_data.device_status = LMP_PAGE;

        ce_ptr->hci_cmd_bits |= REMOTE_NAME_REQ_CON_BIT_MASK;
        *baseband_cmd_flag = TRUE;

#ifdef _CCH_PAGE_CON_
         ce_ptr->connect_reason = CON_REASON_REMOTE_NAME_REQ;
#endif	

		hci_generate_command_status_event(
			HCI_REMOTE_NAME_REQUEST_OPCODE, 
			HCI_COMMAND_SUCCEEDED);
   }

    /* Reset the device name */
    memset(ce_ptr->device_name, 0, LMP_MAX_NAME_LENGTH);

    return HCI_COMMAND_SUCCEEDED;
}


/**
 * Gets a list of the supported features for the remote device
 * identified by the Connection_Handle parameter.
 *
 * \param hci_cmd_ptr Pointer to HCI_CMD_PKT.
 * \param sent_event_flag Set to TRUE if command status event is generated.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_remote_supported_features_command(HCI_CMD_PKT *hci_cmd_ptr,
        UCHAR * sent_event_flag)
{
    UINT16 ce_index;
    UINT16 conn_handle;
    LMP_CONNECTION_ENTITY *ce_ptr;

    BT_FW_EXTRACT_16_BITS(conn_handle, &hci_cmd_ptr->cmd_parameter[0]);
    if (LMP_GET_CE_INDEX_FROM_CONN_HANDLE(conn_handle, &ce_index)
            != API_SUCCESS)
    {
        return NO_CONNECTION_ERROR;
    }

    ce_ptr = &lmp_connection_entity[ce_index];

#ifdef COMPILE_PARK_MODE
    /* Check if the device is in park-state */
    if (ce_ptr->ce_status == LMP_PARK_MODE)
    {
        return COMMAND_DISALLOWED_ERROR;
    }
#endif

    lmp_send_features_req_or_res_pdu(ce_index, LMP_FEATURES_REQ_OPCODE,
            SELF_DEV_TID);
    ce_ptr->lmp_expected_pdu_opcode |= lmp_get_opcode_mask(
                LMP_FEATURES_RES_OPCODE, 0x0);

    ce_ptr->hci_cmd_bits |= REMOTE_SUP_FEA_BIT_MASK;
    ce_ptr->hci_cmd_bits  |= REMOTE_SUP_FEA_TRUE_HOST_BIT_MASK;     

    return HCI_COMMAND_SUCCEEDED;
}

/**
 * Obtains the values for the version information for the
 * remote bluetooth device identified by the Connection_Handle parameter.
 *
 * \param hci_cmd_ptr     Pointer to HCI_CMD_PKT.
 * \param sent_event_flag Set to TRUE if command status event is generated.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_remote_version_information_command(HCI_CMD_PKT *hci_cmd_ptr,
                                                    UCHAR *sent_event_flag)
{
    UINT16 ce_index ;
    UCHAR parameter_list[LMP_VERSION_REQ_LEN];
    UINT16 conn_handle ;
    LMP_CONNECTION_ENTITY *ce_ptr;
    UINT8 ret_error = HCI_COMMAND_SUCCEEDED;
#ifdef LE_MODE_EN
    LL_CONN_HANDLE_UNIT *ll_handle = NULL;
#endif

    BT_FW_EXTRACT_16_BITS(conn_handle, &(hci_cmd_ptr->cmd_parameter[0]));

    do 
    {
        if (LMP_GET_CE_INDEX_FROM_CONN_HANDLE(conn_handle, &ce_index) 
                                                    != API_SUCCESS)
        {
            ret_error = NO_CONNECTION_ERROR;

#ifdef LE_MODE_EN
            if (IS_BT40)
            {
                ret_error = hci_4_0_handle_remote_version_information_command(
                                            conn_handle, &ll_handle);
            }
#endif
            break;
        }

        ce_ptr = &lmp_connection_entity[ce_index];

        switch(ce_ptr->ce_status)
        {
            case LMP_CONNECTED :
               break ;

#ifdef COMPILE_HOLD_MODE
            case LMP_HOLD_MODE :
                break;
#endif

#ifdef COMPILE_SNIFF_MODE
            case LMP_SNIFF_MODE :
               break;
#endif

            default :
                ret_error = COMMAND_DISALLOWED_ERROR;
                break;
        }

        if (ret_error != HCI_COMMAND_SUCCEEDED)
        {
            break;
        }

        /*
         * Check version information of the remote device is in connection
         * entity database.
         *
         * If version info is in connection enttiy(already version info
         * transaction is done) send it to the  host without going for PDU
         * transaction.Otherwise send version request PDU to the remote device.
         */

        parameter_list[0] = LMP_VERSION_REQ_OPCODE;
#ifdef _DAPE_UNIFY_VERSION_FEATURE_UINT
        parameter_list[2] = lmp_self_device_data.lmp_version;
#else
        if (IS_BT42)
        {
            parameter_list[2] = BT_FW_LMP_VERSION_BT42;            
        }
        else if (IS_BT41)
        {
            parameter_list[2] = BT_FW_LMP_VERSION_BT41;            
        }
        else if (IS_BT40)
        {
            parameter_list[2] = BT_FW_LMP_VERSION_BT40;            
        }
        else if (IS_BT30)
        {
            parameter_list[2] = BT_FW_LMP_VERSION_BT30;            
        }
        else
        {
            parameter_list[2] = BT_FW_LMP_VERSION_BT21_PLUS_EDR;
        }
#endif
        parameter_list[3] = LSB(otp_str_data.bt_manufacturer_name);
        parameter_list[4] = MSB(otp_str_data.bt_manufacturer_name);
        parameter_list[5] = LSB(BT_FW_LMP_SUBVERSION);
        parameter_list[6] = MSB(BT_FW_LMP_SUBVERSION);

        lmp_generate_pdu(ce_index, parameter_list, LMP_VERSION_REQ_LEN,
                SELF_DEV_TID, LMP_NO_STATE_CHANGE);

        ce_ptr->lmp_expected_pdu_opcode |= lmp_get_opcode_mask(
                    LMP_VERSION_RES_OPCODE, 0x0);


        ce_ptr->hci_cmd_bits |= REMOTE_VER_INFO_BIT_MASK;

    }
    while (0);
    
#ifdef LE_MODE_EN
    if (!IS_BT40)
    {
        return ret_error;
    }

    /*Send Command status event */
    hci_generate_command_status_event(hci_cmd_ptr->cmd_opcode, ret_error);

    if (ll_handle != NULL)
    {
        if (ll_handle->llc.get_version_ind)
        {
            /*Send remote version information complete Event */
            hci_generate_remote_version_information_complete_event(ret_error, 
                conn_handle, ll_handle->unit_id + LMP_MAX_CE_DATABASE_ENTRIES);
            ll_handle->hci_cmd_bm.remote_version = 0;
        }
        else
        {
            ll_handle->hci_cmd_bm.remote_version = 1;

            if (ret_error == HCI_COMMAND_SUCCEEDED)
            {
                /* check we have already sent an LL_VERSION_IND pdu to the remote device */
                if (!ll_handle->llc.sent_version_ind)
                {
                    if (ll_handle->llc.type != LLC_PROCEDURE_TYPE_NONE)
                    {
                        /* we need to wait the completion of previous LLC procedure */
                        ll_handle->llc.pend_version_ex = TRUE;
                    }
                    else
                    {
                        llc_schedule_version_exchange_procedure(ll_handle);
                    }
                }
            }
        }
    }

    *sent_event_flag = TRUE;
#endif

    return ret_error;
}

/**
 * Allows the Host to read clock offset of remote devices.
 *
 * \param hci_cmd_ptr     Pointer to HCI_CMD_PKT.
 * \param sent_event_flag Set to TRUE if command status event is generated.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_clock_offset_command(HCI_CMD_PKT *hci_cmd_ptr,
                                        UCHAR *sent_event_flag)
{
    UINT16 ce_index ;
    UCHAR parameter_list[LMP_CLKOFFSET_REQ_LEN];
    UINT16 conn_handle;
    LMP_CONNECTION_ENTITY *ce_ptr;

    /*Extract the connection handle */
    BT_FW_EXTRACT_16_BITS(conn_handle, &(hci_cmd_ptr->cmd_parameter[0]));

    if (LMP_GET_CE_INDEX_FROM_CONN_HANDLE(conn_handle, &ce_index) != API_SUCCESS)
    {
         return NO_CONNECTION_ERROR ;
    }

    ce_ptr = &lmp_connection_entity[ce_index];

    if (ce_ptr->remote_dev_role == SLAVE)
    {
		parameter_list[0] = LMP_CLKOFFSET_REQ_OPCODE ;
        switch(ce_ptr->ce_status)
        {
            case LMP_CONNECTED :
                break;

#ifdef COMPILE_HOLD_MODE
            case LMP_HOLD_MODE :
                break;
#endif

#ifdef COMPILE_SNIFF_MODE
            case LMP_SNIFF_MODE :
                break;
#endif

            default :
                return COMMAND_DISALLOWED_ERROR ;
        }
        if (lmp_generate_pdu(ce_index, parameter_list, LMP_CLKOFFSET_REQ_LEN,
                    SELF_DEV_TID, LMP_NO_STATE_CHANGE) != API_SUCCESS)
        {
			return UNSPECIFIED_ERROR;
        }

        ce_ptr->lmp_expected_pdu_opcode |=
                     lmp_get_opcode_mask( LMP_CLKOFFSET_RES_OPCODE, 0x0);


        ce_ptr->hci_cmd_bits |= READ_CLOCK_OFFSET_BIT_MASK;
    }
    else
    {
        hci_generate_command_status_event(hci_cmd_ptr->cmd_opcode,
                        HCI_COMMAND_SUCCEEDED);

        /* Send Read clock offset Complete event */
        hci_generate_clock_offset_complete_event(
                HCI_COMMAND_SUCCEEDED, conn_handle, ce_ptr->clock_offset);

        *sent_event_flag = TRUE ;
    }

    return HCI_COMMAND_SUCCEEDED;
}
/**
 * Allows the Host to read lmp handle of bt controller.
 *
 * \param hci_cmd_ptr     Pointer to HCI_CMD_PKT.
 * \param sent_event_flag Set to TRUE if command status event is generated.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */

UCHAR hci_read_lmp_handle_command(HCI_CMD_PKT *hci_cmd_ptr,
                                        UCHAR *sent_event_flag)
{
    UINT16 ce_index ;
    UINT16 sco_ce_index, esco_ce_index;
    UINT16 conn_handle;
    UCHAR  lmp_handle;

    /*Extract the connection handle */
    BT_FW_EXTRACT_16_BITS(conn_handle, &(hci_cmd_ptr->cmd_parameter[0]));

    /* Check if connection handle is not ACL connection handle*/
    if (LMP_GET_CE_INDEX_FROM_CONN_HANDLE(conn_handle, &ce_index) == API_SUCCESS)
    {
        /*return command not valid for ACL connection */
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }
    /* Check if connection handle specified is for SCO conn */
    else if (!lmp_get_sco_ce_index_from_conn_handle(conn_handle,
                                                  &sco_ce_index))
    {
        /* Sco link connection handle */
        lmp_handle = lmp_sco_connection_data[sco_ce_index].sco_handle;
    }
    /* Check if connection handle specified is for eSCO conn */
    else if (!lmp_get_esco_ce_index_from_conn_handle(conn_handle,
                                                  &esco_ce_index))
    {
        /* Esco link connection handle */
        lmp_handle = lmp_esco_connection_entity[esco_ce_index].esco_handle;
    }
    else
    {
        /* Entered connection handle is not valid */
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    /* Send Read clock offset Complete event */
    hci_generate_read_lmp_handle_complete_event(HCI_COMMAND_SUCCEEDED,
        hci_cmd_ptr->cmd_opcode, conn_handle, lmp_handle);

    *sent_event_flag = TRUE ;
    return HCI_COMMAND_SUCCEEDED;
}

#ifdef COMPILE_HOLD_MODE

/**
 * Inteprets the Hold Mode command packet coming from the
 * the host and sets the connection_entity parameters
 * hold_mode_max_interval and hold_mode_min_interval which is used by
 * the Link Manager to negotiate the Hold Mode Interval with the remote
 * device.
 *
 * \param hci_cmd_ptr Pointer to HCI_CMD_PKT.
 * \param ce_index    Connection Entity Index.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_hold_mode_command(UCHAR *hci_cmd_ptr, UINT16 ce_index)
{
    UINT32 clock_value;
    UCHAR parameter_list[LMP_HOLD_REQ_LEN];
    UINT32 hold_instant ;
    UINT16 tpoll_val ;
    UINT16 hold_max_interval, hold_min_interval ;
    UINT16 remote_feature, local_feature ;
    LMP_CONNECTION_ENTITY* ce_ptr;
    UCHAR temp_var;
	UCHAR phy_piconet_id;

	ce_ptr = &lmp_connection_entity[ce_index];

	remote_feature = ce_ptr->feat_page0[0];

	local_feature = lmp_feature_data.feat_page0[0];

#ifdef COMPILE_CHECK_LOCAL_FEATURES
    if ((local_feature & LMP_HOLD_MODE_FEATURE) == FALSE)
    {
        return UNSUPPORTED_FEATURE_OR_PARAMETER_ERROR;
    }
#endif

    if ((remote_feature & LMP_HOLD_MODE_FEATURE) == FALSE )
    {
        return UNSUPPORTED_REMOTE_FEATURE_ERROR;
    }

    /* Extract the Hold Mode Max Interval */
    BT_FW_EXTRACT_16_BITS(hold_max_interval, &(hci_cmd_ptr[2]));

    /* Extract the Hold Mode Min Interval */
    BT_FW_EXTRACT_16_BITS(hold_min_interval, &(hci_cmd_ptr[4]));

    temp_var = hci_validate_hold_params(hci_cmd_ptr, ce_index);
    if (temp_var != API_SUCCESS)
    {
        return temp_var;
    }

#ifdef COMPILE_NESTED_PAUSE_RESUME
	aclq_mark_am_addr_as_paused(ce_ptr->am_addr, ce_ptr->phy_piconet_id, 
	                                             ACL_PAUSED_HOLD);
#else /* COMPILE_NESTED_PAUSE_RESUME */
	aclq_mark_am_addr_as_paused(ce_ptr->am_addr, ce_ptr->phy_piconet_id);
#endif /* COMPILE_NESTED_PAUSE_RESUME */

    /* Dispatch the LMP_hold PDU */
    if ((ce_ptr->hold_mode_interval_negotiated == TRUE)
        && (ce_ptr->hold_mode_interval <= ce_ptr->hold_neg_max_interval))
    {
        parameter_list[0] = LMP_HOLD_OPCODE;
    }
    else
    {
        parameter_list[0] = LMP_HOLD_REQ_OPCODE;
    }

	phy_piconet_id = ce_ptr->phy_piconet_id;
	lc_get_clock_in_scatternet(&clock_value, phy_piconet_id);

    /* Calculate the hold instant. */
    clock_value >>= 1;
    tpoll_val = (UINT16)(ce_ptr->Tpoll * HOLD_INSTANT_MULTIPLIER );
    hold_instant = tpoll_val + clock_value ;
    hold_instant &= 0xFFFFFFFE;             /* Make it a Master's slot */
    hold_instant &= BT_CLOCK_27_BITS;
    ce_ptr->hold_instant = hold_instant;
    parameter_list[2] = LSB(ce_ptr->hold_mode_interval);
    parameter_list[3] = MSB(ce_ptr->hold_mode_interval);
    bt_fw_ultostr(&parameter_list[4], hold_instant, 4);

    lmp_generate_pdu(ce_index, parameter_list, LMP_HOLD_REQ_LEN,
                     SELF_DEV_TID, LMP_ENTERING_HOLD_MODE);

    if ((ce_ptr->hold_mode_interval_negotiated == TRUE)
            && (ce_ptr->hold_mode_interval <= ce_ptr->hold_neg_max_interval))
    {
        /*
         * If the self device is a Master start the timer for hold instant
         * and then put the device into the hold mode.
         * If the self device is a Slave send LMP_hold,
         * remote device (Master) will send LMP_hold, then put the device
         * into the hold mode.
         */
        if (ce_ptr->remote_dev_role == MASTER)
        {
            ce_ptr->lmp_expected_pdu_opcode |=
                lmp_get_opcode_mask( LMP_HOLD_OPCODE, 0x0);

        }

        ce_ptr->hold_mode_accepted_flag = TRUE ;
    }
    else
    {
        ce_ptr->hold_mode_min_interval = hold_min_interval;
        ce_ptr->hold_mode_max_interval = ce_ptr->hold_mode_interval;
        ce_ptr->lmp_expected_pdu_opcode |= lmp_get_opcode_mask(
                LMP_HOLD_REQ_OPCODE, 0x0);
    }

    return HCI_COMMAND_SUCCEEDED;
}

#endif /* COMPILE_HOLD_MODE */

/**
 * Inteprets the Sniff Mode command packet coming from the
 * the host and sets the connection_entity parameters sniff_attempt,
 * sniff_timeout, sniff_mode_max_interval and sniff_mode_min_interval
 * which is used by the Link Manager to put the remote device into Sniff mode.
 *
 * \param hci_cmd_ptr Pointer to HCI_CMD_PKT.
 * \param ce_index    Connection Entity Index.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
#ifdef COMPILE_SNIFF_MODE
UCHAR hci_handle_sniff_mode_command(UCHAR *hci_cmd_ptr, UINT16 ce_index)
{
    UCHAR timing_control_flags ;
    UCHAR parameter_list[LMP_SNIFF_REQ_LEN];
    UCHAR init_proc ;
    UINT16 sniff_max_interval, sniff_min_interval;
    UINT16 sniff_attempt, sniff_timeout ;
    UINT16 remote_feature;
    LMP_CONNECTION_ENTITY* ce_ptr;
    UCHAR temp_var;
#ifdef COMPILE_CHECK_LOCAL_FEATURES
    UCHAR local_feature;
#endif

	ce_ptr = &lmp_connection_entity[ce_index];

#ifdef COMPILE_CHECK_LOCAL_FEATURES
	local_feature = lmp_feature_data.feat_page0[0];

    if ((local_feature & LMP_SNIFF_MODE_FEATURE) == FALSE)
    {
		HCI_LOG_ERROR(LOG_LEVEL_LOW, SELF_DEVICE_IS_NOT_SUP_THE_SNIFF_MODE_FEATURE,0,0);
        return UNSUPPORTED_FEATURE_OR_PARAMETER_ERROR;
    }

	if ((ce_ptr->link_policy_settings & LMP_LP_MASTER_SNIFF_MODE) == 0)
	{
		HCI_LOG_ERROR(LOG_LEVEL_LOW, SELF_DEVICE_IS_NOT_SUP_THE_SNIFF_MODE_FEATURE,0,0);
		return UNSUPPORTED_FEATURE_OR_PARAMETER_ERROR;
	}
#endif

    remote_feature = ce_ptr->feat_page0[0];

    if ((remote_feature & LMP_SNIFF_MODE_FEATURE) == FALSE )
    {
        return UNSUPPORTED_FEATURE_OR_PARAMETER_ERROR;
    }

    /* Extract the Sniff Max Interval */
    BT_FW_EXTRACT_16_BITS(sniff_max_interval, &(hci_cmd_ptr[2]));

    /* Extract the Sniff Min Interval */
    BT_FW_EXTRACT_16_BITS(sniff_min_interval, &(hci_cmd_ptr[4]));

    /* Extract the Sniff Attempt value */
    BT_FW_EXTRACT_16_BITS(sniff_attempt, &(hci_cmd_ptr[6]));
    /* Extract the Sniff Timeout value */
    BT_FW_EXTRACT_16_BITS(sniff_timeout, &(hci_cmd_ptr[8]));

    temp_var = hci_validate_sniff_params(hci_cmd_ptr, ce_index);

    if (temp_var != API_SUCCESS)
    {
        return temp_var;
    }

    /* Make Sniff interval even */
    ce_ptr->sniff_interval += 0x01;
    ce_ptr->sniff_interval &= (~0x0001U);

    /* Store the Sniff parameters in connection entity */
    ce_ptr->sniff_max_interval = sniff_max_interval;
    ce_ptr->sniff_min_interval = sniff_min_interval;
    ce_ptr->sniff_attempt = sniff_attempt;
    ce_ptr->sniff_timeout = sniff_timeout;

#ifndef _CCH_SLOT_OFFSET_
    /* Calculate the sniff slot offset */
	if (lmp_get_slot_offset(ce_index, &init_proc) != API_SUCCESS)
    {
        return UNSPECIFIED_ERROR;
    }

    if (ce_ptr->remote_dev_role == SLAVE)
    {
        ce_ptr->sniff_slot_offset = lmp_get_dsniff();
        if (ce_ptr->sniff_slot_offset >= ce_ptr->sniff_min_interval)
        {
            lmp_put_dsniff(ce_ptr->sniff_slot_offset);
            ce_ptr->sniff_slot_offset = 0;
        }
    }
    else
    {
        ce_ptr->sniff_slot_offset = 0;
    }
#endif

#ifdef _CCH_SLOT_OFFSET_

#ifdef _CCH_8723_RCP_
// RCP for use another parameter to get slot offset for sniff request from host
// Default Parameter:  can find by ce_index
// Change Parameter: can change sniff_attempt and sniff_timeout

    UCHAR return_status = 0;

#ifdef _ROM_CODE_PATCHED_
    if (rcp_hci_handle_sniff_mode_command_get_slot != NULL)
    {
        rcp_hci_handle_sniff_mode_command_get_slot((void *)&return_status,
			&sniff_attempt, &sniff_timeout, ce_index);
    }
#endif

    if( return_status == 0)
#endif		
    {		

        UINT16 temp_slot_num;

        if (sniff_attempt > 1)
        {
            temp_slot_num = sniff_attempt << 1;
        }                 
        else
        {
            temp_slot_num = 2;
        }
    
        if(sniff_timeout > 0)
        {
            temp_slot_num += 2;
        }         		
	
//    RT_BT_LOG(RED, CCH_DBG_032, 0, 0);

#ifdef _CCH_BQB_LIH_BV_122_
        if (ce_ptr->remote_dev_role == SLAVE)
        {
#endif	
            ce_ptr->sniff_slot_offset = lmp_get_global_slot_offset(ce_index, 0, 
                                            ce_ptr->sniff_interval, temp_slot_num, 0, 0);

            UCHAR lmp_get_global_slot_offset_fail;
            lmp_get_global_slot_offset_fail = 1;

            if ((ce_ptr->sniff_slot_offset) < GLOBAL_SLOT_INTERVAL)
            {
                lmp_get_global_slot_offset_fail = 0;
            }
    
            if (lmp_get_global_slot_offset_fail)
            {
                lmp_put_global_slot_offset(ce_index, 0);
                return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
            }
#ifdef _CCH_BQB_LIH_BV_122_
        }else
        {
            lmp_force_global_slot_offset(ce_index, 0, 
                ce_ptr->sniff_interval, temp_slot_num, 0);
            ce_ptr->sniff_slot_offset = 0;	
        }
#endif

    }


    UINT32 clock_value;
    UINT16 phy_piconet_id;
    
    phy_piconet_id = lmp_connection_entity[ce_index].phy_piconet_id;

    /* Extract the value of the current clock value from the BB registers */
    lc_get_clock_in_scatternet(&clock_value, (UCHAR)phy_piconet_id);

    /* If the MSB ie clock_value(26) bit is 0 follow INIT 1 else follow INIT 2*/
    init_proc = (UCHAR)((clock_value & 0x08000000) >> 27);

#endif

    /* Calculate the timing control flags */
    timing_control_flags = init_proc ? 0x03 : 0;

    ce_ptr->temp_sniff_interval = ce_ptr->sniff_interval;
    ce_ptr->temp_sniff_nego_in_progress = TRUE;
    lmp_send_max_slot_pdu_to_all_devices();

    parameter_list[0] = LMP_SNIFF_REQ_OPCODE;
    parameter_list[2] = timing_control_flags;
    parameter_list[3] = LSB(ce_ptr->sniff_slot_offset);
    parameter_list[4] = MSB(ce_ptr->sniff_slot_offset);
    parameter_list[5] = LSB(ce_ptr->sniff_interval);
    parameter_list[6] = MSB(ce_ptr->sniff_interval);
    parameter_list[7] = LSB(ce_ptr->sniff_attempt);
    parameter_list[8] = MSB(ce_ptr->sniff_attempt);
    parameter_list[9] = LSB(ce_ptr->sniff_timeout);
    parameter_list[10] = MSB(ce_ptr->sniff_timeout);

    lmp_generate_pdu(ce_index, parameter_list, LMP_SNIFF_REQ_LEN,
                     SELF_DEV_TID, LMP_SNIFF_MODE_NEG);

    ce_ptr->lmp_expected_pdu_opcode |= lmp_get_opcode_mask(
            LMP_SNIFF_REQ_OPCODE, 0x0);

    return HCI_COMMAND_SUCCEEDED;
}

/**
 * Inteprets the Exit Sniff Mode command packet coming from
 * the host and brings the specified device into active mode.
 *
 * \param ce_index    Connection Entity Index.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_exit_sniff_mode_command(UINT16 ce_index)
{
    UCHAR parameter_list[LMP_UNSNIFF_REQ_LEN];
    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    /* Queue a LMP_unsniff_req PDU to the other device */
    parameter_list[0] = LMP_UNSNIFF_REQ_OPCODE;

	/* Enter sniff transition mode. */
	if (ce_ptr->remote_dev_role == SLAVE)
	{
		lc_program_exit_sniff_transition_mode(ce_index);

        ce_ptr->cont_poll_count = LC_CONT_POLL_SLOT_COUNT;
		RT_BT_LOG(GRAY, BT_FW_HCI_CMDS_1749, 1, ce_index);
	}

	lmp_generate_pdu(ce_index, parameter_list, LMP_UNSNIFF_REQ_LEN,
		SELF_DEV_TID, LMP_NO_STATE_CHANGE);

	return HCI_COMMAND_SUCCEEDED;
}
#endif /* COMPILE_SNIFF_MODE */

#ifdef COMPILE_PARK_MODE
UCHAR hci_handle_park_mode_command_bottom_half(UINT16 ce_index)
{
    UCHAR parameter_list[LMP_PARK_REQ_LEN];
    UCHAR pm_addr = 0 ;
    UCHAR ar_addr = 0 ;
    LMP_CONNECTION_ENTITY* ce_ptr;
#ifndef AFH_PARK_DISABLE
    UINT16 link_super_timeout;
#endif

	LMP_LOG_INFO(LOG_LEVEL_LOW, RECD_PARK_MODE_COMMAND_FROM_THE_HOST, 0, 0);
    ce_ptr = &lmp_connection_entity[ce_index];

    /*
     * This function generates beacon parameters and puts the
     * generated parameters in the connection entity
     */
    lmp_generate_beacon_parameters(ce_index);

    if (ce_ptr->Tbeacon <=
            (ce_ptr->D_access + (ce_ptr->M_access * ce_ptr->T_access)))
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR ;
    }

    if ((ce_ptr->link_supervision_timeout != 0) &&
       (ce_ptr->link_supervision_timeout < 
            (ce_ptr->Tbeacon << 1) + BT_FW_PARK_BEACON_LSTO_MIN_DIFF))
    {
        return UNSUPPORTED_FEATURE_OR_PARAMETER_ERROR ;
    }

#ifndef MASTER_AFH_PARK
#ifdef COMPILE_AFH_HOP_KERNEL
#ifdef AFH_PARK
#ifdef AFH_PARK_DISABLE
    if ((ce_ptr->remote_dev_role == SLAVE) && (ce_ptr->afh_mode == AFH_ENABLE))
    {
        /* AFH is enabled. Disable it and resume park. */
        ce_ptr->park_pending_for_afh = TRUE;
        lmp_update_map(AFH_DISABLE, ce_index, TRUE);

        ce_ptr->afh_disabled_for_park = TRUE;

        return HCI_COMMAND_SUCCEEDED;
    }
#endif /* AFH_PARK */
#endif /* AFH_PARK_DISABLE */
#endif /* COMPILE_AFH_HOP_KERNEL */
#endif /* MASTER_AFH_PARK */

    /*
     * If we are the Master then allocate a pm address and ar address
     * for this slave
     */
    if (ce_ptr->remote_dev_role == SLAVE)
    {
        ce_ptr->pm_addr = 0;
		if (lmp_get_pm_addr(&pm_addr, ce_ptr->phy_piconet_id)!= API_SUCCESS)
        {
			HCI_LOG_ERROR(LOG_LEVEL_LOW, ERROR_GETTING_PM_ADDRESS,0,0);
        }

        ce_ptr->ar_addr = 0;
		if (lmp_get_ar_addr(&ar_addr, ce_ptr->phy_piconet_id) != API_SUCCESS)
        {
			HCI_LOG_ERROR(LOG_LEVEL_LOW, ERROR_GETTING_AR_ADDRESS,0,0);
        }
    }

    ce_ptr->pm_addr = pm_addr;
    ce_ptr->ar_addr = ar_addr;

	LMP_LOG_INFO(LOG_LEVEL_LOW, ALLOCATED_PM_ADDR_AR_ADDR, 2, pm_addr, ar_addr);

#ifdef COMPILE_NESTED_PAUSE_RESUME
	aclq_mark_am_addr_as_paused(ce_ptr->am_addr, ce_ptr->phy_piconet_id, 
	                            ACL_PAUSED_PARK);
#else /* COMPILE_NESTED_PAUSE_RESUME */
	aclq_mark_am_addr_as_paused(ce_ptr->am_addr, ce_ptr->phy_piconet_id);
#endif /* COMPILE_NESTED_PAUSE_RESUME */

    /*
     * If we are master and parking a slave , then set the supervision timeout
     * for this slave to 0 , temporarly till unpark happens ,
     * This is done so that we donot want to auto park and unpark if
     * supervision timeout expires
     */
#ifndef AFH_PARK_DISABLE
    if (ce_ptr->remote_dev_role == SLAVE)
    {
        if (ce_ptr->link_supervision_timeout != 0)
        {
            parameter_list[0] = LMP_SUPERVISION_TIMEOUT_OPCODE;
            link_super_timeout = 0;
            parameter_list[2] = LSB(link_super_timeout);
            parameter_list[3] = MSB(link_super_timeout);

            lmp_generate_pdu(ce_index, parameter_list, LMP_SUPERVISION_TIMEOUT_LEN,
                    MASTER_TID, LMP_NO_STATE_CHANGE);

            /* As a Master Stop the supervision Timer */
            OS_STOP_TIMER( ce_ptr->supervision_timeout_handle, 0);

            /*
             * Store the current Supervision To in the CE temporarly , restore
             * this after becoming active
             */
            ce_ptr->stored_link_supervision_timeout =
                     ce_ptr->link_supervision_timeout;

            /*
             * Set this flag otherwise supervision timer would be started
             * in ISR poll/null reception.
             */
            ce_ptr->link_supervision_timeout = 0;
        }
    }
#endif /* AFH_PARK_DISABLE */

    /* Send a LMP_Park_Req PDU to the remote device */
    parameter_list[0] = LMP_PARK_REQ_OPCODE;
    /* Timing control flags */
    parameter_list[2] = ce_ptr->timing_control_flag;
    parameter_list[3] = LSB(ce_ptr->Dbeacon);
    parameter_list[4] = MSB(ce_ptr->Dbeacon);
    parameter_list[5] = LSB(ce_ptr->Tbeacon);
    parameter_list[6] = MSB(ce_ptr->Tbeacon);
    parameter_list[7] = ce_ptr->Nbeacon ;
    parameter_list[8] = ce_ptr->Delta_beacon ;
    parameter_list[9] = ce_ptr->pm_addr ;
    parameter_list[10] = ce_ptr->ar_addr ;
    parameter_list[11] = ce_ptr->NB_sleep ;
    parameter_list[12] = ce_ptr->DB_sleep ;
    parameter_list[13] = ce_ptr->D_access ;
    parameter_list[14] = ce_ptr->T_access ;
    parameter_list[15] = ce_ptr->N_acc_slots ;
    parameter_list[16] = ce_ptr->N_poll ;
    parameter_list[17] = (ce_ptr->access_scheme << 4) | ce_ptr->M_access;

    lmp_generate_pdu(ce_index, parameter_list, LMP_PARK_REQ_LEN, SELF_DEV_TID,
            LMP_PARK_MODE_REQ);

    if (ce_ptr->remote_dev_role == MASTER)
    {
        ce_ptr->lmp_expected_pdu_opcode |= lmp_get_opcode_mask(
            LMP_PARK_REQ_OPCODE, 0x0);

    }

    return HCI_COMMAND_SUCCEEDED;
}


/**
 * Handles the Park Mode command packet coming from the
 * the host and sets the connection_entity parameters beacon_max_interval
 * and beacon_min_interval which is used by the Link Manager to put the
 * remote device into Park mode.
 *
 * \param hci_cmd_ptr Pointer to HCI_CMD_PKT.
 * \param ce_index    Connection Entity Index.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_park_mode_command(HCI_CMD_PKT *hci_cmd_ptr, UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
#ifndef AFH_PARK_DISABLE
    UINT16 link_super_timeout;
#endif

#ifdef COMPILE_CHECK_LOCAL_FEATURES
    UCHAR local_feature;
#endif

    UCHAR temp_var;

	LMP_LOG_INFO(LOG_LEVEL_LOW, RECD_PARK_MODE_COMMAND_FROM_THE_HOST, 0, 0);
    ce_ptr = &lmp_connection_entity[ce_index];

#ifdef COMPILE_CHECK_LOCAL_FEATURES
	local_feature = lmp_feature_data.feat_page0[1];

    if ((local_feature & LMP_PARK_MODE_FEATURE) == FALSE)
    {
		HCI_LOG_ERROR(LOG_LEVEL_LOW, SELF_DEVICE_IS_NOT_SUP_THE_PARK_MODE_FEATURE,0,0);
        return UNSUPPORTED_FEATURE_OR_PARAMETER_ERROR;
    }
#endif

    if (lmp_self_device_data.number_of_parked_dev != 0)
    {
		RT_BT_LOG(GRAY, BT_FW_HCI_CMDS_1817, 0, 0);

		return COMMAND_DISALLOWED_ERROR;
     }

    temp_var = hci_validate_park_params(hci_cmd_ptr->cmd_parameter, ce_index);

    if (temp_var != API_SUCCESS)
    {
        return temp_var;
    }

    ce_ptr->park_pending_for_afh = FALSE;
    return hci_handle_park_mode_command_bottom_half(ce_index);
}

/**
 * Handles the Exit Park Mode command packet coming from
 * the host and brings the specified device into active mode.
 *
 * \param ce_index    Connection Entity Index.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_exit_park_mode_command(UINT16 ce_index)
{
	LMP_CONNECTION_ENTITY* ce_ptr;
	UCHAR ret_val;

	ce_ptr = &lmp_connection_entity[ce_index];

	if (ce_ptr->hci_unpark_req_flag == HCI_UNPARK_HOST_INITIATED)
	{
		RT_BT_LOG(GRAY, BT_FW_HCI_CMDS_2022, 0, 0);

		return COMMAND_DISALLOWED_ERROR;
	}

	ce_ptr->hci_unpark_req_flag = HCI_UNPARK_HOST_INITIATED;
	ret_val = lmp_handle_host_initiated_unpark(ce_index);

	return ret_val;
}
#endif /* COMPILE_PARK_MODE */

/**
 * Inteprets the QOS setup command comming
 * from the host  and generates a QOS parameters for the particular
 * connection handle.
 *
 * \param hci_cmd_ptr Pointer to HCI_CMD_PKT.
 * \param ce_index    Connection Entity Index.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_qos_setup_command(HCI_CMD_PKT *hci_cmd_ptr, UINT16 ce_index)
{
    UCHAR service_type ;
    UINT32 token_rate;
    UINT32 peak_bandwidth;
    UINT32 latency;
    UINT32 delay_variation;
    UCHAR parameter_list[LMP_QoS_REQ_LEN];
	LMP_CONNECTION_ENTITY *ce_ptr;

    /* Extract service_type */
    service_type = hci_cmd_ptr->cmd_parameter[3];

    /* Extract the token rate */
    BT_FW_EXTRACT_32_BITS(token_rate, &(hci_cmd_ptr->cmd_parameter[4]));

    /* Extract peak bandwidth */
    BT_FW_EXTRACT_32_BITS(peak_bandwidth, &(hci_cmd_ptr->cmd_parameter[8]));

    /* Extract latency */
    BT_FW_EXTRACT_32_BITS(latency, &(hci_cmd_ptr->cmd_parameter[12]));

    /* Extract delay variation */
    BT_FW_EXTRACT_32_BITS(delay_variation, &(hci_cmd_ptr->cmd_parameter[16]));

    if (service_type > QOS_MAX_SERVICE_TYPE)
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    ce_ptr = &lmp_connection_entity[ce_index];

    ce_ptr->service_type = service_type ;
    ce_ptr->token_rate = token_rate ;
    ce_ptr->peak_bandwidth = peak_bandwidth ;
    ce_ptr->latency = latency;
    ce_ptr->delay_variation = delay_variation;

    /* If service-type is 'no traffic' or 'best-effort',
       all the other params are ignored. */
    if ( (service_type == QOS_NO_TRAFFIC) /*||
            (service_type == QOS_BEST_EFFORT) */)
    {
        hci_queue_QoS_complete_event(ce_index, HCI_COMMAND_SUCCEEDED);

        return HCI_COMMAND_SUCCEEDED;
    }

    if ((token_rate == HCI_DEFAULT_QoS_TOKEN_RATE)
#ifdef ENABLE_SCO
        || (lc_full_bandwidth_flag == TRUE)
#endif
        )
    {
        hci_queue_QoS_complete_event(ce_index, QOS_NOT_SUPPORTED_ERROR);

        return HCI_COMMAND_SUCCEEDED;
    }

	if ( (hci_generate_qos_params(ce_index, QOS_SETUP_DUAL_SIDE)) == 
		API_FAILURE)
    {
        /* QoS parameter validation has failed. */
        return HCI_COMMAND_SUCCEEDED;
    }

    /* Generate LMP_qos_req pdu */
    parameter_list[0] = LMP_QoS_REQ_OPCODE;
    parameter_list[2] = LSB(ce_ptr->qos_tpoll);
    parameter_list[3] = MSB(ce_ptr->qos_tpoll);
    parameter_list[4] = ce_ptr->num_of_BC;

    lmp_generate_pdu(ce_index, parameter_list, LMP_QoS_REQ_LEN, SELF_DEV_TID,
            LMP_NO_STATE_CHANGE);

	ce_ptr->qos_tn_details = QOS_TN_QOS_SETUP;

    return HCI_COMMAND_SUCCEEDED;
}

/**
 * Inteprets the role discovery command coming
 * from the host and determine which role the device is performing for
 * a particular connection handle.
 *
 * \param hci_cmd_ptr Pointer to HCI_CMD_PKT.
 * \param ce_index    Connection Entity Index.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_role_discovery_command(HCI_CMD_PKT *hci_cmd_ptr,
                                        UINT16 *ce_index)
{
    UINT16 connection_handle;

    /* Extract the connection handle */
    BT_FW_EXTRACT_16_BITS(connection_handle, &(hci_cmd_ptr->cmd_parameter[0]));

    /* Validate connection handle. */
    /* Obtain the index corresponding to the connection handle */
    if (LMP_GET_CE_INDEX_FROM_CONN_HANDLE(connection_handle, ce_index)
                                                        == API_FAILURE)
    {
        return NO_CONNECTION_ERROR;
    }

    return HCI_COMMAND_SUCCEEDED ;
}


#ifdef COMPILE_ROLE_SWITCH
UCHAR hci_handle_switch_role_command_bottom_half(UINT16 ce_index)
{
#ifdef COMPILE_AFH_HOP_KERNEL
    /* Check if the AFH timer is running. */
    if (lmp_connection_entity[ce_index].afh_instant_timer_handle != NULL)
    {
        /* Set flag to resume HCI-cmd after AFH-instant. */
        lmp_connection_entity[ce_index].mss_cmd_pending = TRUE;

        return HCI_COMMAND_SUCCEEDED;        
    }
#endif

    if (bz_auth_is_master_link_key_in_use(ce_index))
    {
        return COMMAND_DISALLOWED_ERROR;
    }

    if (bz_auth_is_link_encrypted(ce_index))
    {
        /* before proceeding with the role-switch, we got to either pause or
         * stop the encryption.
         */
        if (bz_auth_pause_encryption(ce_index))
        {
            /* encryption disabling procedure for this link has been started
             * and the Authentication Module will intimate about its
             * completion through "BZ_AUTH_PAUSE_ENC_CB". refer
             * bz_auth_register_pause_encryption_callback() for more details.
             * We got to wait until this procedure is completed and then
             * possibly initiate the role-switch.
             */
			lmp_set_ce_status(ce_index, LMP_ROLE_SWITCHING);
        }
        else
        {
            /* Unable to stop/pause encryption, so abort role-switch procedure
             */
            return UNSPECIFIED_ERROR;
        }
    }
    else
    {
        /* The link is not encrypted, we can start role-switch right away :) */
        return hci_start_role_switch(ce_index);
    }

    return HCI_COMMAND_SUCCEEDED;
}

/**
 * Allows the Host to switch the current role the device is
 * performing for a particular connection with another bluetooth device.
 *
 * \param hci_cmd_ptr Pointer to HCI_CMD_PKT.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_switch_role_command(HCI_CMD_PKT *hci_cmd_ptr, UINT16 *ce_idx)
{
    UINT16 ce_index ;
    UCHAR temp_bd_addr[LMP_BD_ADDR_SIZE];
    LMP_CONNECTION_ENTITY *ce_ptr;
    UCHAR ret_val;

    memcpy(temp_bd_addr, &hci_cmd_ptr->cmd_parameter[0], LMP_BD_ADDR_SIZE);

    ret_val = hci_validate_switch_role_params(hci_cmd_ptr->cmd_parameter);
#ifdef _DAPE_TEST_NO_SW_TO_MASTER
RT_BT_LOG(RED, DAPE_TEST_LOG293, 1,12345);
ret_val = COMMAND_DISALLOWED_ERROR;
#endif
    if (ret_val != API_SUCCESS)
    {
        return ret_val;
    }

    LMP_GET_CE_INDEX_FROM_BD_ADDR(temp_bd_addr, &ce_index);

    ce_ptr = &lmp_connection_entity[ce_index];
#ifdef _DAPE_TEST_NO_ROLE_SW_WHEN_AU_RAND
    BZ_AUTH_LINK_PARAMS* auth;
    auth = ce_ptr->auth;
    if (auth->sub_state != BZ_AUTH_SUB_STATE_IDLE)
    {
        return COMMAND_DISALLOWED_ERROR;
    }
#endif
    *ce_idx = ce_index;
    
    ce_ptr->mss_cmd_pending = FALSE;
    return hci_handle_switch_role_command_bottom_half(ce_index);
}

/**
 * Used to start role switch.
 *
 * \param ce_index Connection Entity Index.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_start_role_switch(UINT16 ce_index)
{
    UCHAR parameter_list[LMP_SWITCH_REQ_LEN];
    UINT32 native_clock;
    UINT32 switch_instant;
    UCHAR remote_dev_role;
    LMP_CONNECTION_ENTITY *ce_ptr;
    UCHAR total_pdus;

    ce_ptr = &lmp_connection_entity[ce_index];

#ifdef COMPILE_NESTED_PAUSE_RESUME
    aclq_mark_am_addr_as_paused(ce_ptr->am_addr, 
                                ce_ptr->phy_piconet_id, ACL_PAUSED_MSS);
#else /* COMPILE_NESTED_PAUSE_RESUME */
    aclq_mark_am_addr_as_paused(ce_ptr->am_addr, ce_ptr->phy_piconet_id);
#endif /* COMPILE_NESTED_PAUSE_RESUME */
#ifndef _ENABLE_COMPILE_NESTED_PAUSE_RESUME_
    ce_ptr->pause_data_transfer = TRUE;
#endif
    /*
     * Switch instant should be atleast 2*Tpoll or 32.
     * (Whichever is greater), safe side we calculate 5*Tpoll.
     */

    total_pdus = 1;

#ifdef _DAPE_TEST_MINIMIZE_ROLE_SW_INSTANT
    total_pdus += pduq_get_no_of_pdus_piconet(ce_ptr->phy_piconet_id);

    switch_instant = (4 + total_pdus) * (ce_ptr->Tpoll);
    /* dape: all other bluetooth dongles send switch_instant 238 slots later. */
    if (switch_instant < 238)
    {
        switch_instant = 238;
    }
    
// dape : Add the following codes in case there is some one in sniff mode. 
// If someone is in sniff mode, then the pdus before lmp_switch_req 
// May take a longer to finish. (Since the acl is blocked before sniff instant)
// So We reserve longer time for the lmp_switch_req to be sent.

#ifdef COMPILE_SNIFF_MODE
    UINT32 sniff_int;
#ifdef _DAPE_SNIFF_PKT_TEST	
    UINT16 sniff_ce_index;
    UINT16 sniff_attempt;	
    sniff_int = lc_get_least_sniff_interval(&sniff_ce_index, &sniff_attempt);
#else
    sniff_int = lc_get_least_sniff_interval();
#endif
    if ((sniff_int != 0 ) && (sniff_int < 50))
    {
        UINT8 i;
        for (i = 0; i <= SCA_PICONET_MAX; i++)
        {
            total_pdus += pduq_get_no_of_pdus_piconet(i);
        } 
        switch_instant = (10 + total_pdus) * (ce_ptr->Tpoll);
        if (switch_instant < 320)
        {
            switch_instant = 320;
        }
    }
#endif

#else
    UINT8 i;
    for (i = 0; i <= SCA_PICONET_MAX; i++)
    {
        total_pdus += pduq_get_no_of_pdus_piconet(i);
    } 

    switch_instant = (10 + total_pdus) * (ce_ptr->Tpoll);
    if (switch_instant < 320)
    {
        switch_instant = 320;
    }
#endif

    if (IS_USE_FOR_BQB)
    {	
        /* avoid always instant passed for BQB test - austin */
        if (switch_instant < 800)
        {
            switch_instant = 800;
        }        
    }

    lc_get_clock_in_scatternet(&native_clock, ce_ptr->phy_piconet_id);

    native_clock >>= 1;
    native_clock += switch_instant ;
    native_clock &= BT_CLOCK_27_BITS ;
    native_clock &= (~(0x01));

    ce_ptr->switch_instant = native_clock;

    remote_dev_role = ce_ptr->remote_dev_role;

    if (remote_dev_role == MASTER)
    {
        lmp_send_slot_offset_pdu(ce_index, SLAVE_TID);
        ce_ptr->cont_poll_count = LC_CONT_POLL_SLOT_COUNT;
    }

    /* Now send the LMP_SWITCH_REQ_OPCODE pdu. */
    parameter_list[0] = LMP_SWITCH_REQ_OPCODE;
    bt_fw_ultostr(&parameter_list[2], native_clock, 4);    

    lmp_generate_pdu(ce_index, parameter_list, LMP_SWITCH_REQ_LEN,
        (LMP_TRAN_ID)(remote_dev_role ^ 0x01), LMP_ROLE_SWITCHING);

    if (ce_ptr->remote_dev_role == SLAVE)
    {
        ce_ptr->lmp_expected_pdu_opcode |= lmp_get_opcode_mask(
            LMP_SLOT_OFFSET_OPCODE, 0x0);
    }

    return HCI_COMMAND_SUCCEEDED;
}
#endif /* COMPILE_ROLE_SWITCH */

/**
 * Inteprets the Write link policy settings command coming
 * from the host and updates the policy settings in the connection entity
 * database. Command complete event is sent to the host for this command
 * with status and connection handle as the return parameters.
 *
 * \param hci_cmd_ptr Pointer to HCI_CMD_PKT.
 * \param ce_index    Connection Entity Index.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_write_link_policy_settings_command(UCHAR *hci_cmd_ptr,
                                                            UINT16 ce_index)
{
    UINT16 link_settings ;

    /* Extract the Link policy settings */
    BT_FW_EXTRACT_16_BITS(link_settings, &(hci_cmd_ptr[2]));

    /* Validate Link policy settings.*/
    if (link_settings > 0x000F)
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    /* Update Link policy settings into connection entity database */
    lmp_connection_entity[ce_index].link_policy_settings = link_settings;

    return HCI_COMMAND_SUCCEEDED;
}

/**
 * Inteprets the Write link policy settings command coming
 * from the host and updates the policy settings in the connection entity
 * database. Command complete event is sent to the host for this command
 * with status and connection handle are the return parameters.
 *
 * \param hci_cmd_ptr Pointer to HCI_CMD_PKT.
 * \param ce_index    Connection Entity Index.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_write_default_link_policy_settings_command(UCHAR *hci_cmd_ptr)
{
    UINT16 link_settings ;

    /* Extract the Link policy settings */
    BT_FW_EXTRACT_16_BITS(link_settings, &(hci_cmd_ptr[0]));

    /* Validate Link policy settings.*/
    if (link_settings > 0x000F)
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    /* Update Link policy settings into connection entity database */
    lmp_self_device_data.default_link_policy_settings = link_settings ;

	return HCI_COMMAND_SUCCEEDED ;
}

/**
* Handles the flow specification command.
*
* \param hci_cmd_ptr Pointer to HCI_CMD_PKT.
* \param ce_index    Connection Entity Index.
*
* \return HCI_COMMAND_SUCCEEDED or HCI error code.
*
*/
UCHAR hci_handle_flow_specification_command(
	HCI_CMD_PKT *hci_cmd_ptr, UINT16 ce_index)
{
	UCHAR flow_direction;
	UCHAR service_type;

	UINT32 token_rate;
	UINT32 token_bucket_rate;
	UINT32 peak_bandwidth;
	UINT32 latency;
	LMP_CONNECTION_ENTITY *ce_ptr;
	UCHAR parameter_list[LMP_QoS_REQ_LEN];

	ce_ptr = &lmp_connection_entity[ce_index];

	flow_direction = hci_cmd_ptr->cmd_parameter[3];

	/* Extract service_type */
	service_type = hci_cmd_ptr->cmd_parameter[4];

	/* Extract the token rate */
	BT_FW_EXTRACT_32_BITS(token_rate, &(hci_cmd_ptr->cmd_parameter[5]));

	/* Extract the token bucket rate */
	BT_FW_EXTRACT_32_BITS(token_bucket_rate, &(hci_cmd_ptr->cmd_parameter[9]));

	/* Extract peak bandwidth */
	BT_FW_EXTRACT_32_BITS(peak_bandwidth, &(hci_cmd_ptr->cmd_parameter[13]));

	/* Extract access latency */
	BT_FW_EXTRACT_32_BITS(latency, &(hci_cmd_ptr->cmd_parameter[17]));

	ce_ptr->service_type = service_type ;
	ce_ptr->token_rate = token_rate ;
	ce_ptr->token_bucket_rate = token_bucket_rate ;
	ce_ptr->peak_bandwidth = peak_bandwidth ;
	ce_ptr->latency = latency;

	/* If service-type is 'no traffic',
	all the other params are ignored. */
	if ( (service_type == QOS_NO_TRAFFIC))
	{
		hci_queue_flow_spec_complete_event(ce_index, flow_direction, 
			HCI_COMMAND_SUCCEEDED);

		return HCI_COMMAND_SUCCEEDED;
	}

	if ((token_rate == HCI_DEFAULT_QoS_TOKEN_RATE)
#ifdef ENABLE_SCO
		|| (lc_full_bandwidth_flag == TRUE)
#endif
		)
	{
		hci_queue_flow_spec_complete_event(ce_index, flow_direction, 
			QOS_REJECTED_ERROR);

		return HCI_COMMAND_SUCCEEDED;
	}


	if ( (hci_generate_qos_params(ce_index, flow_direction)) == API_FAILURE)
	{
		/* Validation has failed. */
		return HCI_COMMAND_SUCCEEDED;
	}

	lmp_calculate_min_tpoll_from_qos_and_flow_spec(ce_index);

	/* Generate LMP_qos_req pdu. */
	parameter_list[0] = LMP_QoS_REQ_OPCODE;
    parameter_list[2] = LSB(ce_ptr->qos_tpoll);
    parameter_list[3] = MSB(ce_ptr->qos_tpoll);
	parameter_list[4] = ce_ptr->num_of_BC;

	lmp_generate_pdu(ce_index, parameter_list, LMP_QoS_REQ_LEN, SELF_DEV_TID,
            LMP_NO_STATE_CHANGE);

	ce_ptr->qos_tn_details = flow_direction;

	return HCI_COMMAND_SUCCEEDED;
}
