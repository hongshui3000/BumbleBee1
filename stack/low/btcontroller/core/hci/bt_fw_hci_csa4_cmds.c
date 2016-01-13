/***************************************************************************
 Copyright (C) Realtek Semiconductor Corp.
 This module is a confidential and proprietary property of Realtek and
 a possession or use of this module requires written permission of Realtek.
 ***************************************************************************/

/**
 * \file
 *  CSA4 HCI Commands implementation.
 *  
 */
#ifdef VER_CSA4
/********************************* Logger *************************/ 
enum { __FILE_NUM__= 75 };
/********************************* Logger *************************/
/* -----------------Includes----------------------------- */
#include "bt_fw_hci_internal.h"
#include "btc_defines.h"
#include "bt_fw_hci_external_defines.h"
#include "bt_fw_hci_csa4_cmds_evts.h"
#include "bt_3dd.h"
#include "lmp_internal.h"
#include "mem.h"

/* -----------------Global variables--------------------- */
#ifdef _CSB_RX_DBG_LOG
extern UINT8 g_csb_rx_dbg_log;
#endif
/* -----------------Static functions--------------------- */

/**
 * Handles the HCI Truncated Page Command.
 * 
 * \param hci_cmd_ptr HCI Command pointer.
 * 
 * \return HCI Status.
 */
UCHAR hci_handle_truncated_page(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR am_addr;
    UINT16 ce_index;
    UCHAR index;
    UCHAR temp_bd_addr[LMP_BD_ADDR_SIZE];
    UINT16 temp_page_scan_repetition_mode;
    UINT16 clock_offset;
    UCHAR status;
    LMP_CONNECTION_ENTITY *ce_ptr;

	status = hci_validate_truncated_page_cmd_params(hci_cmd_ptr);
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

    RT_BT_LOG(BLUE, MSG_CSB_TRUNCATED_PAGE, 9, 
                            temp_bd_addr[5], temp_bd_addr[4], 
                            temp_bd_addr[3], temp_bd_addr[2], 
                            temp_bd_addr[1], temp_bd_addr[0], 
                            am_addr, ce_index, ce_ptr->phy_piconet_id);

	index = LMP_BD_ADDR_SIZE ;

	/* Page scan repetition mode */
	temp_page_scan_repetition_mode = (UCHAR) hci_cmd_ptr->cmd_parameter[index];
	index++;

	/*
	* LC module checks Clock offset valid flag(Bit 15), If it is invalid
	* value it programs baseband clock offset register with a value zero.
	*/
	BT_FW_EXTRACT_16_BITS(clock_offset, &(hci_cmd_ptr->cmd_parameter[index]));
	index += 2;

	memcpy(&ce_ptr->bd_addr, &temp_bd_addr, LMP_BD_ADDR_SIZE);

	//ce_ptr->connection_type.packet_type = temp_packet_type;
	ce_ptr->connection_type.packet_type = 0;
	ce_ptr->page_scan_repetition_mode = (UCHAR)temp_page_scan_repetition_mode;
	//ce_ptr->page_scan_mode = (UCHAR)temp_page_scan_mode ;
    ce_ptr->page_scan_mode = 0;
	//ce_ptr->allow_role_switch = (UCHAR)temp_allow_role_switch ;
    ce_ptr->allow_role_switch = 0;
	ce_ptr->am_addr = am_addr ;
	ce_ptr->connection_type.link_type = ACL_LINK;

	//lc_update_pkts_allowed(ce_index);

#ifdef _CCH_PAGE_CON_
       ce_ptr->connect_reason = CON_REASON_TRUNCATED_PAGE;
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

	//lmp_set_ce_status(ce_index, LMP_TRUNCATED_PAGING);
    /* QQQ: should we set this status to truncated page for
       sending truncated page complete event since the procedure
       when ce_status == LMP_PAGING is not needed here. */
    lmp_set_ce_status(ce_index, LMP_TRUNCATED_PAGING);
    //lmp_set_ce_status(ce_index, LMP_PAGING);
    lmp_self_device_data.device_status = LMP_PAGE;
	//hci_generate_command_status_event(
	//	HCI_TRUNCATED_PAGE_OPCODE, 
	//	HCI_COMMAND_SUCCEEDED);
    
    return HCI_COMMAND_SUCCEEDED ;
}

/**
 * Handles the HCI Truncated Page Cancel Command.
 * 
 * \param hci_cmd_ptr HCI Command pointer.
 * 
 * \return HCI Status.
 */
UCHAR hci_handle_truncated_page_cancel(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR ret_error = UNKNOWN_HCI_COMMAND_ERROR;    

    memcpy(rem_name_cancel_bd_addr, &hci_cmd_ptr->cmd_parameter[0], LMP_BD_ADDR_SIZE);            
    ret_error = lc_kill_paging(rem_name_cancel_bd_addr, CON_REASON_TRUNCATED_PAGE);

    return ret_error;
}

/**
 * Handles the HCI Set Connectionless Slave Broadcast Command.
 * 
 * \param hci_cmd_ptr HCI Command pointer.
 * 
 * \return HCI Status.
 */
UCHAR hci_handle_set_connectionless_slave_broadcast(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR ret_error = INVALID_HCI_COMMAND_PARAMETERS_ERROR; 
    CSB_HCI_SET_CSB_CMD_PKT_S *cmd = (CSB_HCI_SET_CSB_CMD_PKT_S *)hci_cmd_ptr;
    BT_CSA4_BEACON_TX_UINT_S *csb = &bt_3dd_var.csb_tx_param;

    do
    {
        /*----------------------------------------------------*/
        /*         Parser the Command Parameters              */
        /*----------------------------------------------------*/
        if (cmd->Enable > 1)
        {
            /* out of spec range */
            break;
        }

        if ((cmd->LT_Addr == 0) || (cmd->LT_Addr > 7))
        {
            /* out of spec range */            
            break;
        }

        if (!csb->is_reserved_lt_addr ||
            (csb->lt_addr != cmd->LT_Addr))
        {
            /* no pre-allocated lt addr */
            ret_error = NO_CONNECTION_ERROR;  
            break;
        }

        if (cmd->Enable == 0)
        {
            /* If this command is issued with Enable = 0, and the
               synchronization train substate is active, then the Controller 
               shall also exit the synchronization train substate.*/
            
            /* disable synchronization train transmitter */
            bt_csb_driver_stop_sync_train();  

            /* disable connectionless slave broadcast transmitter */
            bt_csb_driver_stop_transmit_beacon();   

            ret_error = HCI_COMMAND_SUCCEEDED;    

            break;
        }

        if (cmd->LPO_Allowed > 1)
        {
            /* out of spec range */
            break;
        }

        if ((cmd->Interval_Max > 0xFFFE) || (cmd->Interval_Max < 0x0002) ||
            (cmd->Interval_Min > 0xFFFE) || (cmd->Interval_Min < 0x0002) ||
            (cmd->Interval_Max & 0x01) || (cmd->Interval_Min & 0x01) ||
            (cmd->Interval_Max < cmd->Interval_Min))
        {
            /* out of spec range or odd value or unreasonable condition */
            break;
        }

        if ((cmd->CSB_supervisionTO > 0xFFFE) ||
            (cmd->CSB_supervisionTO < 0x0002) ||
            (cmd->CSB_supervisionTO & 0x01))
        {
            /* out of spec range or odd value */
            break;
        }

        ret_error = HCI_COMMAND_SUCCEEDED;                        

        /*----------------------------------------------------*/
        /*         Update Local Parameters                    */
        /*----------------------------------------------------*/
        csb->is_allow_lpo = cmd->LPO_Allowed;
        csb->support_packet_type_bm = cmd->Packet_Type;
        csb->support_packet_type_bm ^= ALL_EDR_PACKETS;
        csb->interval_min = cmd->Interval_Min;
        csb->interval_max = cmd->Interval_Max;
        csb->interval = csb->interval_min; /* temp option */
        csb->supervision_timeout = cmd->CSB_supervisionTO;
        
        /* enable connectionless slave broadcast transmitter */
        bt_csb_driver_start_transmit_beacon();
    }
    while (0);
           
    return ret_error;
}

/**
 * Handles the HCI Set Connectionless Slave Broadcast Received Command.
 * 
 * \param hci_cmd_ptr HCI Command pointer.
 * 
 * \return HCI Status.
 */
UCHAR hci_handle_set_connectionless_slave_broadcast_receive(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR ret_error = INVALID_HCI_COMMAND_PARAMETERS_ERROR; 
    CSB_HCI_SET_CSB_RECEIVE_CMD_PKT_S *cmd = (CSB_HCI_SET_CSB_RECEIVE_CMD_PKT_S *)hci_cmd_ptr;
    BT_CSA4_BEACON_RX_UNIT_S *csb = &bt_3dd_var.csb_rx_param;

    do
    {
        /*----------------------------------------------------*/
        /*         Parser the Command Parameters              */
        /*----------------------------------------------------*/
        if (cmd->Enable > 1)
        {
            /* out of spec range */
            break;
        }
        
        if (cmd->Enable == 0)
        {
            /* If this command is issued with Enable = 0, then 
               the Controller shall exit the csb rx substate.*/
            
            /* disable connectionless slave broadcast transmitter */
            bt_csb_driver_stop_rx_beacon();   

            ret_error = HCI_COMMAND_SUCCEEDED;    

            break;
        }

        if (bt_3dd_var.csb_rx_param.enable)
        {
            /* If sync scan is already enabled, the Command
                Disallowed (0x0C) error code shall be returned. */            
            ret_error = COMMAND_DISALLOWED_ERROR;
            break;
        }
        
        if ((cmd->LT_Addr == 0) || (cmd->LT_Addr > 7))
        {
            /* out of spec range */            
            break;
        }

        UINT16 interval = cmd->bInterval[0] | (cmd->bInterval[1] << 8);
        UINT16 stp_rx_to = cmd->bCSB_supervisionTO[0] | 
                                      (cmd->bCSB_supervisionTO[1] << 8);

        if ((interval > 0xFFFE) || (interval < 0x0002) ||
            (interval & 0x01))
        {
            /* out of spec range or odd value or unreasonable condition */
            break;
        }

        if ((stp_rx_to > 0xFFFE) ||
            (stp_rx_to < 0x0002) ||
            (stp_rx_to & 0x01))
        {
            /* out of spec range or odd value */
            break;
        }
        
        UINT16 packet_type;
        UINT16 packet_type_bm;
        packet_type = cmd->bPacket_Type[0] | (cmd->bPacket_Type[1] << 8);
        packet_type_bm = packet_type ^ ALL_EDR_PACKETS;
        
        if ((packet_type_bm & ALL_EDR_PACKETS) & 
            (packet_type_bm & (HOST_DM3 | HOST_DM5 | HOST_DH1 | HOST_DH3 | HOST_DH5)))
        {
            /* EDR and BR pkt are enabled at the same time. */
            CSB_LOG(RED, LOG_LEVEL_HIGH, 
                    MSG_CSB_RX_PKT_TYPE_WRONG, 2, 
                        packet_type, packet_type_bm);
            break;
        }

        ret_error = HCI_COMMAND_SUCCEEDED;                        

        /*----------------------------------------------------*/
        /*         Update Local Parameters                    */
        /*----------------------------------------------------*/
        memcpy(&csb->bd_addr, &cmd->bBD_Addr[0], LMP_BD_ADDR_SIZE);
        csb->lt_addr = cmd->LT_Addr;
        csb->interval = interval;
        memcpy(&csb->clk_offset, &cmd->bClock_Offset[0], 4);
        memcpy(&csb->next_instant, &cmd->bNext_CSB_Clock[0], 4);
        csb->supervision_timeout = stp_rx_to;
        csb->remote_timging_accuracy = cmd->Remote_Timing_Accuracy;
        csb->skip = cmd->Skip;
        csb->support_packet_type_bm = packet_type_bm;       
        memcpy(&csb->ch_map, &cmd->bAFH_Ch_Map[0], 10);
#ifdef _CSB_RX_DBG_LOG
        if (g_csb_rx_dbg_log)
        {
            RT_BT_LOG(BLUE, YL_DBG_HEX_6, 6, csb->bd_addr[0],
            csb->bd_addr[1],
            csb->bd_addr[2],
            csb->bd_addr[3],
            csb->bd_addr[4],
            csb->bd_addr[5]);
        }
#endif

#ifdef _CSB_RX_DBG_LOG
        if (g_csb_rx_dbg_log)
        {
            RT_BT_LOG(BLUE, YL_DBG_HEX_10, 10, csb->ch_map[0],
            csb->ch_map[1],
            csb->ch_map[2],
            csb->ch_map[3],
            csb->ch_map[4],
            csb->ch_map[5],
            csb->ch_map[6],
            csb->ch_map[7],
            csb->ch_map[8],
            csb->ch_map[9]);
        }
#endif

        
        /* enable connectionless slave broadcast transmitter */
        bt_csb_driver_start_rx_beacon();
    }
    while (0);

    return ret_error;
}

/**
 * Handles the HCI Start Synchronization Train Command.
 * 
 * \param hci_cmd_ptr HCI Command pointer.
 * 
 * \return HCI Status.
 */
UCHAR hci_handle_start_synchronization_train_command(HCI_CMD_PKT *hci_cmd_ptr)
{    
    UCHAR ret_error = HCI_COMMAND_SUCCEEDED;    

    do
    {
#ifndef _DAPE_STILL_TX_SYNC_TRAIN_EVEN_NO_BEACON    
        if (!bt_3dd_var.csb_tx_param.enable)
        {
            /* If connectionless slave broadcast mode is not enabled, the Command
                Disallowed (0x0C) error code shall be returned. */            
            ret_error = COMMAND_DISALLOWED_ERROR;
            break;
        }
#endif
        /* enable synchronization train transmitter */
        bt_csb_driver_start_sync_train();
    }
    while (0);
    
    return ret_error;
}

/**
 * Handles the HCI Receive Synchronization Train Command.
 * 
 * \param hci_cmd_ptr HCI Command pointer.
 * 
 * \return HCI Status.
 */
UCHAR hci_handle_receive_synchronization_train_command(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR ret_error = INVALID_HCI_COMMAND_PARAMETERS_ERROR;    
    CSB_HCI_RECEIVE_SYNC_TRAIN_CMD_PKT_S *cmd;
    BT_CSA4_SYNC_TRAIN_RX_UNIT_S *stp = &bt_3dd_var.stp_rx_param;

    cmd = (CSB_HCI_RECEIVE_SYNC_TRAIN_CMD_PKT_S *)hci_cmd_ptr; 
    
    do
    {
        if (bt_3dd_var.stp_rx_param.enable)
        {
            /* If sync scan is already enabled, the Command
                Disallowed (0x0C) error code shall be returned. */            
            ret_error = COMMAND_DISALLOWED_ERROR;
            break;
        }
    
        /*----------------------------------------------------*/
        /*         Parser the Command Parameters              */  
        /*----------------------------------------------------*/           
        UINT16 scan_window = cmd->bSync_Scan_Window[0] | (cmd->bSync_Scan_Window[1] << 8);
        UINT16 scan_interval = cmd->bSync_Scan_Interval[0] | (cmd->bSync_Scan_Interval[1] << 8);
        UINT32 stto = cmd->bSync_scanTO[0] | 
                      (cmd->bSync_scanTO[1] << 8);
        
        if ((scan_window > 0xFFFE) || (scan_window < 0x0022) ||
            (scan_interval > 0xFFFE) || (scan_interval < 0x0002) ||
            (scan_window & 0x01) || (scan_interval & 0x01) ||
            (scan_interval < scan_window))
        {
            /* out of spec range or odd value or unreasonable condition */
            break;
        }

        if ((stto > 0xFFFE) || (stto < 0x0022) || (stto & 0x01))
        {
            /* out of spec range or odd value */
            break;
        }

        /*----------------------------------------------------*/
        /*         Update Local Parameters                    */
        /*----------------------------------------------------*/
        stp->scan_window = scan_window;
        stp->scan_interval = scan_interval;
        stp->timeout = stto;
        stp->scan_map = 7; //(?)
        memcpy(&stp->bd_addr, &cmd->bBD_Addr[0], LMP_BD_ADDR_SIZE);

#ifdef _CSB_RX_DBG_LOG
        if (g_csb_rx_dbg_log)
        {
            RT_BT_LOG(WHITE, YL_DBG_HEX_9, 9, stp->bd_addr[0],
            stp->bd_addr[1],
            stp->bd_addr[2],
            stp->bd_addr[3],
            stp->bd_addr[4],
            stp->bd_addr[5],
            stp->scan_window,
            stp->scan_interval,
            stp->timeout);
            RT_BT_LOG(WHITE, YL_DBG_HEX_4, 4,
            cmd->bSync_Scan_Window[0],
            cmd->bSync_Scan_Window[1],
            cmd->bSync_Scan_Interval[0],
            cmd->bSync_Scan_Interval[1]);
        }
#endif

        /* enable synchronization scan*/
        bt_csb_driver_start_sync_scan();

        ret_error = HCI_COMMAND_SUCCEEDED;          
    }
    while (0);
    return ret_error;
}

/**
 * Handles the HCI Set Reserved LT ADDR Command.
 * 
 * \param hci_cmd_ptr HCI Command pointer.
 * 
 * \return HCI Status.
 */
UCHAR hci_handle_set_reserved_lt_addr_command(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR ret_error = HCI_COMMAND_SUCCEEDED;    
    CSB_HCI_SET_RESERVED_LT_ADDR_CMD_PKT_S *cmd;
    UINT8 index;
    UINT8 lt_addr;
    LMP_AM_ADDR_TO_CE_INDEX_TABLE *table;  
    UINT8 piconet_id;

    cmd = (CSB_HCI_SET_RESERVED_LT_ADDR_CMD_PKT_S *)hci_cmd_ptr;

    do
    {
        /*----------------------------------------------------*/
        /*         Parser the Command Parameters              */
        /*----------------------------------------------------*/
        lt_addr = cmd->LT_Addr;

        if ((lt_addr > 7) || (lt_addr == 0))
        {
            /* out of spec range */
            ret_error = INVALID_HCI_COMMAND_PARAMETERS_ERROR; 
            break;
        }

        /* Check if scans are enabled in baseband.
           These will be temporarily killed. */
        if (lmp_self_device_data.scan_enable != 0x0)
        {
            lc_kill_scan_mode();
        }

        piconet_id = lc_allocate_piconet_id_for_paging();

        if (piconet_id == SCA_PICONET_INVALID)
        {
            ret_error = HOST_REJECTED_LIMITED_RESOURCES_ERROR; 
            break;            
        }

        index = lt_addr - 1;      
        table = &lmp_am_addr_to_ce_index_table_ppi[index][piconet_id];

        if (table->status == ASSIGNED)
        {
            ret_error = ACL_CONNECTION_EXISTS_ERROR; 
            break;            
        }

        /* update piconet info */
        if (lmp_self_device_data.lc_no_of_connections[piconet_id] == 0)
        {
            lc_sca_manager.pnet[piconet_id].master = 1;
            lc_sca_manager.pnet[piconet_id].active = 1;
            lc_sca_manager.master_cnt = 1;
            lc_sca_manager.master_id = piconet_id;
            lmp_self_device_data.lc_no_of_connections[piconet_id] = 1;
#ifdef _SET_CSB_PN_INFO
            OR_val_with_bb_reg(reg_PICONET_INFO[piconet_id], BIT0);
#endif
        }        
        
        bt_3dd_var.csb_tx_param.lt_addr = lt_addr;
        bt_3dd_var.csb_tx_param.piconet_id = piconet_id;
        bt_3dd_var.csb_tx_param.is_reserved_lt_addr = TRUE;
        table->status = ASSIGNED;       
#ifdef _DAPE_FIX_CSB_TX_NO_AFH_WHEN_IN_CAM
        lmp_connection_entity[table->ce_index].am_addr = lt_addr;
        lmp_connection_entity[table->ce_index].phy_piconet_id = piconet_id;
#endif
    }
    while (0);

    lc_check_and_enable_scans_in_scatternet();

    return ret_error;
}

/**
 * Handles the HCI Delete Reserved LT ADDR Command.
 * 
 * \param hci_cmd_ptr HCI Command pointer.
 * 
 * \return HCI Status.
 */
UCHAR hci_handle_delete_reserved_lt_addr_command(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR ret_error = HCI_COMMAND_SUCCEEDED;    
    CSB_HCI_DELETE_RESERVED_LT_ADDR_CMD_PKT_S *cmd;
    UINT8 lt_addr;
    UINT8 piconet_id;
   
    cmd = (CSB_HCI_DELETE_RESERVED_LT_ADDR_CMD_PKT_S *)hci_cmd_ptr; 

    do
    {
        /*----------------------------------------------------*/
        /*         Parser the Command Parameters              */
        /*----------------------------------------------------*/    
        if (bt_3dd_var.csb_tx_param.enable)
        {
            /* If connectionless slave broadcast mode is still active, 
               then the Controller shall return the Command Disallowed (0x0c) 
               error code.*/
            ret_error = COMMAND_DISALLOWED_ERROR;
            break;
        }

        lt_addr = cmd->LT_Addr;
        if ((lt_addr > 7) || (lt_addr == 0))
        {
            /* out of spec range */
            ret_error = INVALID_HCI_COMMAND_PARAMETERS_ERROR; 
            break;
        }

        if ((bt_3dd_var.csb_tx_param.is_reserved_lt_addr) &&
               (bt_3dd_var.csb_tx_param.lt_addr == lt_addr))        
        {
            piconet_id = bt_3dd_var.csb_tx_param.piconet_id;

            if (lc_sca_manager.master_id == piconet_id)
            {            
                /* update piconet info */
                if (lmp_self_device_data.lc_no_of_connections[piconet_id] == 1)
                {
                    lc_sca_manager.pnet[piconet_id].active = 0;
                    lc_sca_manager.master_cnt = 0;
                    lmp_self_device_data.lc_no_of_connections[piconet_id] = 0;

#ifdef _SET_CSB_PN_INFO
                    AND_val_with_bb_reg(reg_PICONET_INFO[piconet_id], (UINT16)(~BIT0));
#endif
                }  
            }

            bt_3dd_var.csb_tx_param.is_reserved_lt_addr = FALSE;
            lmp_put_am_addr_ppi(lt_addr, bt_3dd_var.csb_tx_param.piconet_id);
        }
        else
        {
            /* If the LT_ADDR indicated in the LT_ADDR parameter is not 
               reserved by the BR/EDR Controller, it shall return the Unknown 
               Connection Identifier (0x02) error code.*/               
            ret_error = NO_CONNECTION_ERROR;
            break;
        }
    }
    while (0);
    
    return ret_error;
}

/**
 * Handles the HCI Set Connectionless Slave Broadcast Data Command.
 * 
 * \param hci_cmd_ptr HCI Command pointer.
 * 
 * \return HCI Status.
 */
UCHAR hci_handle_set_connectionless_slave_broadcast_data(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR ret_error = HCI_COMMAND_SUCCEEDED;    
    CSB_HCI_SET_CSB_DATA_CMD_PKT_S *cmd;
    UINT8 lt_addr;
    UINT8 fragment;
    HCI_ACL_DATA_PKT *hci_acl_pkt_ptr; 
    BT_CSA4_BEACON_TX_UINT_S *csb = &bt_3dd_var.csb_tx_param;
    
    cmd = (CSB_HCI_SET_CSB_DATA_CMD_PKT_S *)hci_cmd_ptr; 

    do
    {
        lt_addr = cmd->LT_Addr;
        fragment = cmd->Fragment;
        
        /*----------------------------------------------------*/
        /*         Parser the Command Parameters              */  
        /*----------------------------------------------------*/           
        if ((lt_addr > 7) || (lt_addr == 0) || (fragment > 3) ||
            (cmd->length != (3 + cmd->Data_Length)))
        {
            /* out of spec range */
            ret_error = INVALID_HCI_COMMAND_PARAMETERS_ERROR; 
            break;
        }

        if (!((csb->is_reserved_lt_addr) && (csb->lt_addr == lt_addr)))
        {
            /* If the command is issued without the LT_ADDR reserved, 
               the Unknown Connection Identifier (0X02) error code shall 
               be returned.*/
            ret_error = NO_CONNECTION_ERROR;
            break;
        }

        /* check previous fragment and current fragment */

        if ((fragment == 0) ||      /* Continuation fragment */
            (fragment == 2))        /* Ending fragment */
        {
            if ((csb->fragment > 1) ||
                (csb->pData == NULL) || 
                ((csb->data_len + cmd->Data_Length) > BZDMA_MAX_ACL_PKT_SZ))
            {
                ret_error = INVALID_HCI_COMMAND_PARAMETERS_ERROR; 
                break;
            }            
        }
        else /* Starting fragment or No fragmentation (single fragment) */
        {
            if ((cmd->Data_Length > 0) && (csb->pData == NULL))
            {
                os_reserve_buffer();

                if (OS_ALLOC_BUFFER(tx_table[ACL_DATA_HANDLER_TASK].pool_handle,
                                (OS_ADDRESS *)&hci_acl_pkt_ptr) != BT_ERROR_OK)
                {
                    os_free_reserved_buffer();  

                    ret_error = HOST_REJECTED_LIMITED_RESOURCES_ERROR;
                    break;            
                }     

                csb->pData = (UINT8*)hci_acl_pkt_ptr;
            }

            csb->data_len = 0;
        }
               
        csb->fragment = cmd->Fragment;

        if (cmd->Data_Length > 0)
        {
            hci_acl_pkt_ptr = (HCI_ACL_DATA_PKT *)csb->pData;
            memcpy(&hci_acl_pkt_ptr->hci_acl_data_pkt[csb->data_len], 
                                        cmd->Data, cmd->Data_Length);            
            csb->data_len += cmd->Data_Length;          
        }

        if (csb->enable)
        {
            /* must update broadcast data content in run time */
            // TODO:  move to safe time for update csb tx data
            bt_csb_driver_prepare_beacon_data();
        }        
    }
    while (0);

    return ret_error;
}

/**
 * Handles the HCI Read Synchronization Train Parameters Command.
 * 
 * \param hci_cmd_ptr HCI Command pointer.
 * 
 * \return HCI Status.
 */
UCHAR hci_handle_read_synchronization_train_parameters(HCI_CMD_PKT *hci_cmd_ptr)
{
    return HCI_COMMAND_SUCCEEDED;
}

/**
 * Handles the HCI Write Synchronization Train Parameters Command.
 * 
 * \param hci_cmd_ptr HCI Command pointer.
 * 
 * \return HCI Status.
 */
UCHAR hci_handle_write_synchronization_train_parameters(HCI_CMD_PKT *hci_cmd_ptr)
{ 
    UCHAR ret_error = INVALID_HCI_COMMAND_PARAMETERS_ERROR;    
    CSB_HCI_WRITE_SYNC_TRAIN_PARAMS_CMD_PKT_S *cmd;
    BT_CSA4_SYNC_TRAIN_TX_UNIT_S *stp = &bt_3dd_var.stp_tx_param;
    
    cmd = (CSB_HCI_WRITE_SYNC_TRAIN_PARAMS_CMD_PKT_S *)hci_cmd_ptr; 
    
    do
    {        
        /*----------------------------------------------------*/
        /*         Parser the Command Parameters              */  
        /*----------------------------------------------------*/           
        UINT16 intv_min = cmd->bInterval_Min[0] | (cmd->bInterval_Min[1] << 8);
        UINT16 intv_max = cmd->bInterval_Max[0] | (cmd->bInterval_Max[1] << 8);
        UINT32 stto = cmd->bSync_TrainTO[0] | 
                      (cmd->bSync_TrainTO[1] << 8) |
                      (cmd->bSync_TrainTO[2] << 16) | 
                      (cmd->bSync_TrainTO[3] << 24);
        
        if ((intv_min > 0xFFFE) || (intv_min < 0x0002) ||
            (intv_max > 0xFFFE) || (intv_max < 0x0002) ||
            (intv_min & 0x01) || (intv_max & 0x01) ||
            (intv_max < intv_min))
        {
            /* out of spec range or odd value or unreasonable condition */
            break;
        }

        if ((stto > 0x07FFFFFE) || (stto < 0x00000002) || (stto & 0x01))
        {
            /* out of spec range or odd value */
            break;
        }

        /*----------------------------------------------------*/
        /*         Update Local Parameters                    */
        /*----------------------------------------------------*/
        stp->interval_min = intv_min;
        stp->interval_max = intv_max;
        stp->interval = intv_min; /* current option */
        stp->timeout = stto;
        stp->service_data = cmd->Service_Data;    

        ret_error = HCI_COMMAND_SUCCEEDED;          
    }
    while (0);


    return ret_error;
}

/**
 * Handles the HCI Set Triggered Clock Capture Command.
 * 
 * \param hci_cmd_ptr HCI Command pointer.
 * 
 * \return HCI Status.
 */
UCHAR hci_handle_set_triggered_clock_capture(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR ret_error = INVALID_HCI_COMMAND_PARAMETERS_ERROR;    
    CSB_HCI_SET_TRIGGERED_CLOCK_CAPTURE_CMD_PKT_S *cmd;    
    BT_CSA4_CLOCK_CAPTURE_UNIT_S *pclk_cap;
    UINT16 ce_index = 0;

    cmd = (CSB_HCI_SET_TRIGGERED_CLOCK_CAPTURE_CMD_PKT_S *)hci_cmd_ptr; 
    pclk_cap = &bt_3dd_var.clk_cap;

    do
    {        
        /*----------------------------------------------------*/
        /*         Parser the Command Parameters              */  
        /*----------------------------------------------------*/           
        UINT16 conn_handle = cmd->bConnecton_Handle[0] | 
                            (cmd->bConnecton_Handle[1] << 8);
        
        if ((conn_handle > 0x0EFF) ||
            (cmd->Enable > 1) ||
            (cmd->Which_Clock > 1) ||
            (cmd->LPO_Allowed > 1))
        {
            /* out of spec range */
            break;
        }

        if (cmd->Which_Clock)
        {
            if (LMP_GET_CE_INDEX_FROM_CONN_HANDLE(conn_handle,
                                                  &ce_index)!= API_SUCCESS)
            {
                ret_error = NO_CONNECTION_ERROR;                
                break;
            }            
        }

        /*----------------------------------------------------*/
        /*         Update Local Parameters                    */
        /*----------------------------------------------------*/
        DEF_CRITICAL_SECTION_STORAGE;
        MINT_OS_ENTER_CRITICAL();   
        pclk_cap->conn_handle = conn_handle;
        pclk_cap->enable = cmd->Enable;
        pclk_cap->is_allow_lpo = cmd->LPO_Allowed;
        pclk_cap->num_ext_clk_captures = cmd->Num_Clk_Cap_To_Filter;
        bt_3dd_var.use_piconet_clk = cmd->Which_Clock;
        
        if (bt_3dd_var.use_piconet_clk)
        {
            /* plan to capture piconet clock */            
            pclk_cap->piconet_id = lmp_connection_entity[ce_index].phy_piconet_id;                       
        }
        MINT_OS_EXIT_CRITICAL();         

        ret_error = HCI_COMMAND_SUCCEEDED;          
    }
    while (0);  

    return ret_error;
}

/* -----------------External functions------------------- */
/**
 * Handles all the link controller for the Bluetooth Version CSA4.
 * This function is called from the hci_task.c if the opcode
 * does not match any of the 1.1/1.2/2.1/3.0 Version opcodes.
 *
 * \param hci_cmd_ptr Pointer to HCI_CMD_PKT.
 * \param sent_event  Sets the variable if event is sent.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_csa4_link_controller_commands(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR ret_error = UNKNOWN_HCI_COMMAND_ERROR;
    UINT8 sent_cmd_status = FALSE;

    switch(hci_cmd_ptr->cmd_opcode)
    {
    case HCI_TRUNCATED_PAGE_OPCODE:            
        ret_error = hci_handle_truncated_page(hci_cmd_ptr);
        sent_cmd_status = TRUE;
        break;
        
    case HCI_TRUNCATED_PAGE_CANCEL_OPCODE:            
        ret_error = hci_handle_truncated_page_cancel(hci_cmd_ptr);
        break;
        
    case HCI_SET_CONNECTIONLESS_SLAVE_BROADCAST_OPCODE:            
        ret_error = hci_handle_set_connectionless_slave_broadcast(hci_cmd_ptr);
        break;
        
    case HCI_SET_CONNECTIONLESS_SLAVE_BROADCAST_RECEIVE_OPCODE:            
        ret_error = hci_handle_set_connectionless_slave_broadcast_receive(hci_cmd_ptr);
        break;

    case HCI_START_SYNCHRONIZATION_TRAIN_OPCODE:            
        ret_error = hci_handle_start_synchronization_train_command(hci_cmd_ptr);
        sent_cmd_status = TRUE;
        break;

    case HCI_RECEIVE_SYNCHRONIZATION_TRAIN_OPCODE:            
        ret_error = hci_handle_receive_synchronization_train_command(hci_cmd_ptr);
        sent_cmd_status = TRUE;
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
 * Handles all the host controller and baseband for the Bluetooth Version CSA4.
 * This function is called from the hci_task.c if the opcode
 * does not match any of the 1.1/1.2/2.1/3.0 Version opcodes.
 *
 * \param hci_cmd_ptr Pointer to HCI_CMD_PKT.
 * \param sent_event  Sets the variable if event is sent.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_csa4_hc_bb_commands(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR ret_error = HCI_COMMAND_SUCCEEDED;

    switch(hci_cmd_ptr->cmd_opcode)
    {
    case HCI_SET_RESERVED_LT_ADDR_OPCODE:            
        ret_error = hci_handle_set_reserved_lt_addr_command(hci_cmd_ptr);
        break;
        
    case HCI_DELETE_RESERVED_LT_ADDR_OPCODE:            
        ret_error = hci_handle_delete_reserved_lt_addr_command(hci_cmd_ptr);
        break;
        
    case HCI_SET_CONNECTIONLESS_SLAVE_BROADCAST_DATA_OPCODE:            
        ret_error = hci_handle_set_connectionless_slave_broadcast_data(hci_cmd_ptr);
        break;
        
    case HCI_READ_SYNCHRONIZATION_TRAIN_PARAMETERS_OPCODE:            
        ret_error = hci_handle_read_synchronization_train_parameters(hci_cmd_ptr);
        break;

    case HCI_WRITE_SYNCHRONIZATION_TRAIN_PARAMETERS_OPCODE:            
        ret_error = hci_handle_write_synchronization_train_parameters(hci_cmd_ptr);
        break;

    default:
        ret_error = UNKNOWN_HCI_COMMAND_ERROR;
        break;
    }

    return ret_error;
}

/* -----------------External functions------------------- */
/**
 * Handles all the status for the Bluetooth Version CSA4.
 * This function is called from the hci_task.c if the opcode
 * does not match any of the 1.1/1.2/2.1/3.0 Version opcodes.
 *
 * \param hci_cmd_ptr Pointer to HCI_CMD_PKT.
 * \param sent_event  Sets the variable if event is sent.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_csa4_status_commands(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR ret_error = HCI_COMMAND_SUCCEEDED;

    switch(hci_cmd_ptr->cmd_opcode)
    {
    case HCI_SET_TRIGGERED_CLOCK_CAPTURE_OPCODE:            
        ret_error = hci_handle_set_triggered_clock_capture(hci_cmd_ptr);            
        break;

    default:
        ret_error = UNKNOWN_HCI_COMMAND_ERROR;
        break;
    }

    return ret_error;
}


#endif

