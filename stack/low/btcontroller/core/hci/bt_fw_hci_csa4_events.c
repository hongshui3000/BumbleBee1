/***************************************************************************
 Copyright (C) Realtek Semiconductor Corp.
 This module is a confidential and proprietary property of Realtek and
 a possession or use of this module requires written permission of Realtek.
 ***************************************************************************/

/**
 * \file
 *  HCI events handlers related
 *  to the Bluetooth CSA4 Features.
 *  
 */
#ifdef VER_CSA4
/********************************* Logger *************************/ 
enum { __FILE_NUM__= 76 };
/********************************* Logger *************************/

#include "lmp.h"
#include "lc_internal.h"
#include "bt_fw_hci_csa4_cmds_evts.h"
#include "bt_3dd.h"
#include "bt_fw_hci_spec_defines.h"
#include "bt_fw_hci_internal.h"
#include "mem.h"

/* -----------------Global variables--------------------- */

#ifdef _CSB_RX_DBG_LOG
extern UINT8 g_csb_rx_dbg_log;
#endif
/* -----------------Static functions--------------------- */

/**
 * Prepares the command complete event for Version CSA4 HCI commands.
 * This function is called from the hci_generate_2_1_command_complete_
 * event function in bt_fw_hci_2_1_events.c , this function populates the
 * parameters after the status parameter and returns the length of
 * event packet.
 *
 * \param cmd_opcode Command opcode for which event has to be generated.
 * \param pkt_param  Pointer to the event packet parameter list.
 *
 * \return The total length of all the parameters in the event packet.
 *
 */
UCHAR hci_generate_csa4_command_complete_event( UINT16 cmd_opcode,
                                               UCHAR *pkt_param, 
                                               UINT16 ce_index )
{
    /* Param length is 4 because event_opcode(1),cmd_opcode(2) and status(1)
     * are populated by the calling function
     */
    UCHAR param_length = 4;

    switch(cmd_opcode)
    {
#ifdef _SUPPORT_CSB_RECEIVER_
    case  HCI_TRUNCATED_PAGE_CANCEL_OPCODE:
        *(UINT16*)&pkt_param[4] = lc_paging_bd_addr[0];
        *(UINT16*)&pkt_param[6] = lc_paging_bd_addr[1];            
        *(UINT16*)&pkt_param[8] = lc_paging_bd_addr[2];
        param_length = 10;
        break;

    case HCI_SET_CONNECTIONLESS_SLAVE_BROADCAST_RECEIVE_OPCODE:
        /* TODO */
        param_length = 11;
        break;
        
    case HCI_READ_SYNCHRONIZATION_TRAIN_PARAMETERS_OPCODE:
        htole16(bt_3dd_var.stp_tx_param.interval, &pkt_param[4]);
        htole32(bt_3dd_var.stp_tx_param.timeout, &pkt_param[6]);
        pkt_param[10] = bt_3dd_var.stp_tx_param.service_data;
        param_length = 11;
        break;
#endif        

    case HCI_SET_CONNECTIONLESS_SLAVE_BROADCAST_OPCODE:
        pkt_param[4] = bt_3dd_var.csb_tx_param.lt_addr;
        pkt_param[5] = LSB(bt_3dd_var.csb_tx_param.interval);
        pkt_param[6] = MSB(bt_3dd_var.csb_tx_param.interval);
        param_length = 7;
        break;

    case HCI_SET_RESERVED_LT_ADDR_OPCODE:
    case HCI_DELETE_RESERVED_LT_ADDR_OPCODE:   
    case HCI_SET_CONNECTIONLESS_SLAVE_BROADCAST_DATA_OPCODE:        
        pkt_param[4] = bt_3dd_var.csb_tx_param.lt_addr;
        param_length = 5;
        break;

    case HCI_WRITE_SYNCHRONIZATION_TRAIN_PARAMETERS_OPCODE:
        pkt_param[4] = LSB(bt_3dd_var.stp_tx_param.interval);
        pkt_param[5] = MSB(bt_3dd_var.stp_tx_param.interval);
        param_length = 6;
        break;

    case HCI_SET_TRIGGERED_CLOCK_CAPTURE_OPCODE:
        break;

    default :
        return 0xFF;
    }/* end switch(cmd_opcode) */

    return (UCHAR) (param_length - 4);
}

/**************************************************************************
 * Function     : hci_generate_csa4_triggered_clock_capture_event
 *
 * Description  : This function is used to generate Triggered Clock Capture
 *                Event. The Triggered Clock Capture event is sent to indicate 
 *                that a triggering event has occurred at the specified clock 
 *                and offset value.
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
void hci_generate_csa4_triggered_clock_capture_event(void)
{
    UCHAR event_parameter[9];
    CSB_TRIGGERED_CLOCK_CAPTURE_EVT_PARA_S *param;
    BT_CSA4_CLOCK_CAPTURE_UNIT_S *pclk_cap;

    param = (CSB_TRIGGERED_CLOCK_CAPTURE_EVT_PARA_S *)event_parameter;
    pclk_cap = (BT_CSA4_CLOCK_CAPTURE_UNIT_S *)&bt_3dd_var.clk_cap;

    if (bt_3dd_var.clk_cap.enable)
    {
        param->Connection_Handle = pclk_cap->conn_handle;
        param->Which_Clock = bt_3dd_var.use_piconet_clk;
        param->bClock[0] = pclk_cap->pre_piconet_clk & 0xFF;
        param->bClock[1] = pclk_cap->pre_piconet_clk >> 8;
        param->bClock[2] = pclk_cap->pre_piconet_clk >> 16;
        param->bClock[3] = pclk_cap->pre_piconet_clk >> 24;
        param->bSlot_Offset[0] = LSB(pclk_cap->pre_piconet_clk_us);
        param->bSlot_Offset[1] = MSB(pclk_cap->pre_piconet_clk_us);        
        hci_generate_event(HCI_TRIGGERED_CLOCK_CAPTURE_EVENT,
                event_parameter, 9);
    }
}

/**************************************************************************
 * Function     : hci_generate_csa4_sychronization_train_complete_event
 *
 * Description  : This function is used to generate Synchronization Train 
 *                Complete Event. The Synchronization Train Complete event 
 *                indicates that the Start Synchronization Train command has 
 *                completed.
 *
 * Parameters   : status
 *
 * Returns      : None
 *
 *************************************************************************/
void hci_generate_csa4_sychronization_train_complete_event(UINT8 status)
{
    UCHAR event_parameter[1];
    event_parameter[0] = status;
    hci_generate_event(HCI_SYNCHRONIZATION_TRAIN_COMPLETE_EVENT,
            event_parameter, 1);
}

#ifdef _SUPPORT_CSB_RECEIVER_

/**************************************************************************
 * Function     : hci_generate_csa4_sychronization_train_received_event
 *
 * Description  : This function is used to generate Synchronization Train 
 *                Received Event. The Synchronization Train Received event 
 *                provides information received from a synchronization train 
 *                packet transmitted by a Connectionless Slave Broadcast 
 *                transmitter with the given BD_ADDR.
 *
 * Parameters   : status
 *
 * Returns      : None
 *
 *************************************************************************/
void hci_generate_csa4_sychronization_train_received_event(UINT8 status)
{
    UCHAR event_parameter[29];
    CSB_SYNC_TRAIN_RECEIVED_EVT_PARA_S *param;
    BT_CSA4_SYNC_TRAIN_RX_UNIT_S *psync_train_rx;

    param = (CSB_SYNC_TRAIN_RECEIVED_EVT_PARA_S *)event_parameter;
    psync_train_rx = (BT_CSA4_SYNC_TRAIN_RX_UNIT_S *)&bt_3dd_var.stp_rx_param;

    //if (bt_3dd_var.stp_rx_param.enable)
    {         
        param->Status = status;
        memcpy(&param->bBD_ADDR[0], 
                 &psync_train_rx->bd_addr[0], 6);
        param->bClock_Offset[0] = psync_train_rx->clock_offset & 0xFF;
        param->bClock_Offset[1] = psync_train_rx->clock_offset >> 8;
        param->bClock_Offset[2] = psync_train_rx->clock_offset >> 16;
        param->bClock_Offset[3] = psync_train_rx->clock_offset >> 24;

        memcpy(&param->bAFH_Ch_Map[0],
                 &psync_train_rx->ch_map[0], 10);
        param->LT_Addr = psync_train_rx->csb_lt_addr;
        memcpy(&param->bNext_Broadcast_Inst[0], 
                 &psync_train_rx->next_instant[0], 4);
        memcpy(&param->bCSB_Interval[0], 
                 &psync_train_rx->csb_interval[0], 4);
        param->Service_Data = psync_train_rx->service_data;

#ifdef _CSB_RX_DBG_LOG
        if (g_csb_rx_dbg_log)
        {
            RT_BT_LOG(BLUE, YL_DBG_HEX_20, 20,
            param->bBD_ADDR[0],
            param->bBD_ADDR[1],
            param->bBD_ADDR[2],
            param->bBD_ADDR[3],
            param->bBD_ADDR[4],
            param->bBD_ADDR[5],
            param->bClock_Offset[0],
            param->bClock_Offset[1],
            param->bClock_Offset[2],
            param->bClock_Offset[3],
            param->bAFH_Ch_Map[0],
            param->bAFH_Ch_Map[1],
            param->bAFH_Ch_Map[2],
            param->bAFH_Ch_Map[3],
            param->bAFH_Ch_Map[4],
            param->bAFH_Ch_Map[5],
            param->bAFH_Ch_Map[6],
            param->bAFH_Ch_Map[7],
            param->bAFH_Ch_Map[8],
            param->bAFH_Ch_Map[9]);
            RT_BT_LOG(BLUE, YL_DBG_HEX_9, 9,
            param->Status,
            param->LT_Addr,
            param->bNext_Broadcast_Inst[0],
            param->bNext_Broadcast_Inst[1],
            param->bNext_Broadcast_Inst[2],
            param->bNext_Broadcast_Inst[3],
            param->bCSB_Interval[0],
            param->bCSB_Interval[1],
            param->Service_Data);
        }
#endif
    hci_generate_event(HCI_SYNCHRONIZATION_TRAIN_RECEIVED_EVENT,
            event_parameter, 29);
}
}

/**************************************************************************
 * Function     : hci_generate_csa4_connectionless_slave_broadcast_receive_event
 *
 * Description  : This function is used to generate Connectionless Slave 
 *                Broadcast Receive Event. The Connectionless Slave Broadcast 
 *                Receive event shall be sent by the BR/EDR Controller every 
 *                Connectionless Slave Broadcast Instant on which the
 *                BR/EDR Controller is scheduled to receive a Connectionless 
 *                Slave Broadcast packet.
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
void hci_generate_csa4_connectionless_slave_broadcast_receive_event(void)
{
#if 0
    CSB_CSB_RECEIVED_EVT_PARA_S *param;
    BT_CSA4_BEACON_RX_UNIT_S *pbeacon_rx;
    param = (CSB_CSB_RECEIVED_EVT_PARA_S *)event_parameter;
    pbeacon_rx = (BT_CSA4_BEACON_RX_UNIT_S *)&bt_3dd_var.csb_rx_param;

    {         
        memcpy(&param->bBD_ADDR[0], 
                 &pbeacon_rx->bd_addr[0], 6);
        param->LT_Addr = pbeacon_rx->lt_addr;
                 
        param->bClock_Offset[0] = psync_train_rx->clock_offset & 0xFF;
        param->bClock_Offset[1] = psync_train_rx->clock_offset >> 8;
        param->bClock_Offset[2] = psync_train_rx->clock_offset >> 16;
        param->bClock_Offset[3] = psync_train_rx->clock_offset >> 24;

        memcpy(&param->bAFH_Ch_Map[0],
                 &psync_train_rx->ch_map[0], 10);
        param->LT_Addr = psync_train_rx->csb_lt_addr;
        memcpy(&param->bNext_Broadcast_Inst[0], 
                 &psync_train_rx->next_instant[0], 4);
        memcpy(&param->bCSB_Interval[0], 
                 &psync_train_rx->csb_interval[0], 4);
        param->Service_Data = psync_train_rx->service_data;

    }


    // TODO: implement it after we support CSB receiver
    hci_generate_event(HCI_CONNECTIONLESS_SLAVE_BROADCAST_RECEIVED_EVENT, 18);    
#endif
}
#endif

/**************************************************************************
 * Function     : hci_generate_csa4_connectionless_slave_broadcast_timeout_event
 *
 * Description  : This function is used to generate Connectionless Slave 
 *                Broadcast Timeout Event. On the Connectionless Slave Broadcast 
 *                Receiver, the Connectionless Slave Broadcast Timeout event 
 *                indicates to the Host that the BR/EDR Controller has lost 
 *                synchronization with the Connectionless Slave Broadcast 
 *                because no Connectionless Slave Broadcast packets have been 
 *                received for the timeout interval, CSB_supervisionTO, 
 *                specified in the Set Connectionless Slave Broadcast Receive 
 *                command.
 *                On the Connectionless Slave Broadcast Transmitter, the 
 *                Connectionless Slave Broadcast Timeout event indicates to the 
 *                Host that the BR/EDR Controller has been unable to transmit a 
 *                Connectionless Slave Broadcast packet for the timeout 
 *                interval, CSB_supervisionTO, specified in the Set 
 *                Connectionless Slave Broadcast command.
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
void hci_generate_csa4_connectionless_slave_broadcast_timeout_event(UINT8 rx)
{
    UCHAR event_parameter[7];
    CSB_CSB_TIMEOUT_EVT_PARA_S *param;

    param = (CSB_CSB_TIMEOUT_EVT_PARA_S *)event_parameter;
    if (!rx)
    {
    param->wBD_ADDR[0] = *(UINT16*)&otp_str_data.bt_bd_addr[0];
    param->wBD_ADDR[1] = *(UINT16*)&otp_str_data.bt_bd_addr[2];    
    param->wBD_ADDR[2] = *(UINT16*)&otp_str_data.bt_bd_addr[4];
    param->LT_Addr = bt_3dd_var.csb_tx_param.lt_addr;
    }
    else
    {
        param->wBD_ADDR[0] = *(UINT16*)&bt_3dd_var.csb_rx_param.bd_addr[0];
        param->wBD_ADDR[1] = *(UINT16*)&bt_3dd_var.csb_rx_param.bd_addr[2];    
        param->wBD_ADDR[2] = *(UINT16*)&bt_3dd_var.csb_rx_param.bd_addr[4];
        param->LT_Addr = bt_3dd_var.csb_rx_param.lt_addr;
    }
    hci_generate_event(HCI_CONNECTIONLESS_SLAVE_BROADCAST_TIMEOUT_EVENT,
            event_parameter, 7);
}

#ifdef _SUPPORT_CSB_RECEIVER_

/**************************************************************************
 * Function     : hci_generate_csa4_truncated_page_complete_event
 *
 * Description  : This function is used to generate Truncated Page Complete 
 *                Event. The Truncated Page Complete event indicates to the 
 *                Host that a Truncated Page command completed.
 *
 * Parameters   : status
 *
 * Returns      : None
 *
 *************************************************************************/
void hci_generate_csa4_truncated_page_complete_event(UINT8 status, UINT16 ce_index)
{
    UCHAR event_parameter[7];
    event_parameter[0] = status;
    memcpy(& event_parameter[1],
           lmp_connection_entity[ce_index].bd_addr, LMP_BD_ADDR_SIZE );

    hci_generate_event(HCI_TRUNCATED_PAGE_COMPLETE_EVENT, event_parameter, 7);
}
#endif

/**************************************************************************
 * Function     : hci_generate_csa4_slave_page_response_timeout_event
 *
 * Description  : This function is used to generate Slave Page Response Timeout 
 *                Event. The Slave Page Response Timeout event indicates to the 
 *                Host that a slave page response timeout has occurred in the 
 *                BR/EDR Controller.
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
void hci_generate_csa4_slave_page_response_timeout_event(void)
{
    hci_generate_event(HCI_SLAVE_PAGE_RESPONSE_TIMEOUT_EVENT, NULL, 0);
}

/**************************************************************************
 * Function     : hci_generate_csa4_connectionless_slave_broadcast_channel_map_change_event
 *
 * Description  : This function is used to generate Connectionless Slave 
 *                Broadcast Channel Map Change Event. The Connectionless Slave 
 *                Broadcast Channel Map Change event is sent by the master's 
 *                BR/EDR Controller to the master's Host to indicate that the
 *                master¡¦s BR/EDR Controller has moved to a new AFH channel 
 *                map for the PBD logical link.
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
void hci_generate_csa4_connectionless_slave_broadcast_channel_map_change_event(void)
{
    UCHAR event_parameter[10];
    memcpy(event_parameter, bt_3dd_var.stp_tx_param.ch_map, 10);
    hci_generate_event(HCI_CONNECTIONLESS_SLAVE_BROADCAST_CHANNEL_MAP_CHANGE_EVENT,
            event_parameter, 10);
}

/**************************************************************************
 * Function     : hci_generate_csa4_inquiry_response_notification_event
 *
 * Description  : This function is used to generate Inquiry Response 
 *                Notification Event.TThe Inquiry Response Notification event 
 *                indicates to the Host that the BR/EDR Controller responded to 
 *                an Inquiry message. The LAP parameter in the event indicates 
 *                the LAP used to create the access code received.
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
void hci_generate_csa4_inquiry_response_notification_event(void)
{
    UCHAR event_parameter[4];
    CSB_INQUIRY_RESP_NOTIFICATION_EVT_PARA_S *param;

    param = (CSB_INQUIRY_RESP_NOTIFICATION_EVT_PARA_S *)event_parameter;    
    param->bLAP[0] = lmp_self_device_data.iac_lap[0][0];
    param->bLAP[1] = lmp_self_device_data.iac_lap[0][1];
    param->bLAP[2] = lmp_self_device_data.iac_lap[0][2];
    param->rssi = lc_calculate_log_from_rssi(lc_read_rssi_register());
    hci_generate_event(HCI_INQUIRY_RESPONSE_NOTIFICATION_EVENT,
            event_parameter, 4);
}
#endif

