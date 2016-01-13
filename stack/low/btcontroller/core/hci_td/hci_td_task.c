/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  Implements HCI Transport Driver Task.
 */

/********************************* Logger *************************/ 
enum { __FILE_NUM__= 33};
/********************************* Logger *************************/
 
#include "hci_td.h"
#include "bt_fw_os.h"
#include "platform.h"
#include "bz_debug.h"
#ifdef LE_MODE_EN
#include "le_ll.h"
#include "le_hci_4_0.h"
#endif
#include "bt_fw_acl_q.h"
#include "dma_usb.h"
#include "lc_internal.h"
#include "bt_fw_os.h"
#include "lmp_vendor_defines.h"
#include "bt_fw_hci_internal.h"

#ifdef _DAPE_SEND_ENCRYPTION_COMMAND_DISALLOWED_AFTER_5SEC
TimerHandle_t send_enc_command_disallow_timer = NULL;
UINT16 g_send_enc_command_disallow_conn_handle = 0xFF;
UINT8 g_send_enc_mode = 0;
void send_enc_command_disallow_timer_func(TimerHandle_t timer_handle);
#endif

void hci_handle_data_to_host(HCI_ACL_RX_DATA_PKT *hci_data_pkt);

extern void hci_td_tx_packet_data(UCHAR pkt_type, UCHAR *buffer, UINT16 length,
                                  UCHAR *data_buffer, UINT16 data_length);
extern void hci_generate_command_status_event(UINT16 hci_cmd_opcode, 
                                             UCHAR cmd_status);

#ifdef ENABLE_PLATFORM_TASK_HOOK
extern void pf_task (INT32 signal, void* arg);
#endif

extern LMP_SELF_DEVICE_DATA lmp_self_device_data;
extern TASK_ID  hci_event_task_handle ;

#ifdef HS_USB_SEND_EVENT_REORDER_ISSUE_NEW
BOOLEAN hci_td_check_hci_event_need_in_order_imp(HCI_EVENT_PKT *evt,
        UINT8 *status, UINT16 *conn_handle)
{
    switch (evt->event_opcode)
    {
    case HCI_CONNECTION_COMPLETE_EVENT:
    case HCI_ENCRYPTION_CHANGE_EVENT:
    case HCI_ENCRYPTION_KEY_REFRESH_COMPLETE_EVENT:
        *status = evt->event_parameter[0];
        *conn_handle = letoh16(&evt->event_parameter[1]);
        break;
    default:
        return FALSE;
    }
    return TRUE;
}
BOOLEAN (*hci_td_check_hci_event_need_in_order)(HCI_EVENT_PKT *, UINT8 *, UINT16 *) = hci_td_check_hci_event_need_in_order_imp;
#endif

/** 
 * HCI_TD_TASK signal handler.
 * 
 * \param signal Signal to process.
 *
 * \return SUCCESS always.
 */
UCHAR hci_td_process_signal(OS_SIGNAL *signal)
{
#ifdef _DMA_LOOPBACK_TEST_
	UINT8 *pbuf;
#endif

    switch (signal->type)
    {
        case SIGNAL_BEGIN:
            //TD_INF(log_file,"Begin Signal Received by Transport Driver Task");
			//TD_INF(BEGIN_SIGNAL_RECEIVED_BY_TRANSPORT_DRIVER_TASK,0,0);
            break;

        case HCI_TD_HCI_EVENT_TO_HOST_SIGNAL:
        {
            //TD_INF(log_file, "Received Event Packet to Send to Host \n");
			TD_INF(RECEIVED_EVENT_PACKET_TO_SEND_TO_HOST,0,0);
#ifdef _DMA_LOOPBACK_TEST_
			pbuf = (UINT8 *) signal->param;
			//RT_BT_LOG(GREEN, DMA_DES_ALLOCAT_RX_ADDR, 1, pbuf);
			pbuf++;
			//RT_BT_LOG(GREEN, DMA_DES_ALLOCAT_RX_ADDR, 1, pbuf);
			pbuf[0] = 0xFC;
			signal->param = pbuf;
			signal->length = signal->length - 1;
			//RT_BT_LOG(GREEN, DMA_DES_ALLOCAT_RX_ADDR, 1, pbuf);
			//return BT_FW_SUCCESS;
#endif

            HCI_EVENT_PKT *event_pkt= (HCI_EVENT_PKT*) signal->param;
#ifdef HS_USB_SEND_EVENT_REORDER_ISSUE_NEW
            if ((g_fun_interface_info.b.hci_sel == PACKAGE_USB_USB)
                    || IS_EVENT_REORDER_FOR_ANY_INTERFACE)
            {
                UINT8 status;
                UINT16 conn_handle;
                if (hci_td_check_hci_event_need_in_order(event_pkt, &status,
                        &conn_handle))
                {
                    if (status == HCI_COMMAND_SUCCEEDED)
                    {
                        aclq_rx_wait_queue_get(conn_handle);
                    }
                }
            }
#endif
#ifdef HS_USB_SEND_EVENT_REORDER_ISSUE
            if ((g_fun_interface_info.b.hci_sel == PACKAGE_USB_USB) ||
                IS_EVENT_REORDER_FOR_ANY_INTERFACE)
            {        
                if (event_pkt->event_opcode == HCI_CONNECTION_COMPLETE_EVENT)
                {
                    if (event_pkt->event_parameter[0] == HCI_COMMAND_SUCCEEDED)
                    {
                        /* update recorded connecton handle */
                        hs_recd_conn_handle = event_pkt->event_parameter[1] | 
                                          (event_pkt->event_parameter[2] << 8);                
                    }
                }        

            }
#endif

#ifdef _DAPE_SEND_ENCRYPTION_COMMAND_DISALLOWED_AFTER_5SEC
            {
                if (event_pkt->event_opcode == HCI_ENCRYPTION_CHANGE_EVENT)
                {
                    if (event_pkt->event_parameter[0] == HCI_COMMAND_SUCCEEDED)
                    {
                        if (send_enc_command_disallow_timer != NULL)
                        {
                            OS_STOP_TIMER(send_enc_command_disallow_timer, 0);  
                        }
                    }
                    if (event_pkt->event_parameter[0] == COMMAND_DISALLOWED_ERROR)
                    {
                        g_send_enc_command_disallow_conn_handle = 
                        	        ((event_pkt->event_parameter[2] & 0xF)<<8) | 
                        	          event_pkt->event_parameter[1];
                        g_send_enc_mode = event_pkt->event_parameter[3];
                            
                        if (send_enc_command_disallow_timer!= NULL)
                        {        
                            OS_DELETE_TIMER(&send_enc_command_disallow_timer);
                        }
                
                        /* Create a tid timer */
                        OS_CREATE_TIMER(ONESHOT_TIMER, &send_enc_command_disallow_timer,
                                send_enc_command_disallow_timer_func, NULL, 0);
                        OS_START_TIMER(send_enc_command_disallow_timer, 1000);  

                        OS_FREE_BUFFER(tx_table[HCI_EVENT_HANDLER_TASK].pool_handle,
                                       (UINT8*)event_pkt);
                        break;
                        
                    }
                }
            }
#endif

#if defined(_LE_AUTO_REPORT_RSSI_AND_LOGIN_INOUT_FUNC_) && defined(LE_MODE_EN)
            if (g_host_state)
            {
                UINT8 drop = FALSE;
                UINT8 status = LE_WAKEON_STATE_NOT_IN_RSSI_LIST;                
                UINT8 rtk_rssi_mode_flag_n = FALSE;

#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_
                if (ll_rssi_manager.msft_monitor_rssi_mode != 
                                LE_MSFT_MONITOR_RSSI_VIA_CONN_HANDLE_MODE_RTK)
                {
                    rtk_rssi_mode_flag_n = TRUE;
                }
#endif

                if (rtk_rssi_mode_flag_n)
                {
                    /* do nothing ~~~ */
                }
                else if ((event_pkt->event_opcode == HCI_LE_MEGA_EVENT) && 
                    (event_pkt->event_parameter[0] == HCI_LE_ADVERTISING_REPORT_SUBEVENT)) 
                {
                    if ((ll_manager.conn_unit.connection_cnts == 0) &&
                        (ll_rssi_manager.max_indivadual_cnt > 0) &&
                            (ll_rssi_manager.max_indivadual_cnt !=
                                        ll_rssi_manager.max_bredr_cnt))
                    {                                       
                        LE_ADVERTISING_REPORT_EVENT_PARA_S *pPara;
                        UINT8 i;
                        INT8 rssi;
                        UINT8 offset = 0;
                        UINT8 addr_type;
                        UINT8 *addr;
                        UINT8 len;
                        
                        pPara = (LE_ADVERTISING_REPORT_EVENT_PARA_S *)event_pkt->event_parameter;

                        for (i = 0; i < pPara->Num_Reports; i++)
                        {                        
                            offset++;
                            addr_type = pPara->Event_Type[offset];
                            addr = &pPara->Event_Type[offset + 1];
                            len = pPara->Event_Type[offset + 7];
                            offset += (8 + len);
                            rssi = pPara->Event_Type[offset];
                            offset++;

                            status = rssi_app_le_check_rssi_condition(addr_type, addr, rssi);

                            if (status < LE_WAKEON_STATE_FAIL_IN_RSSI_LIST)
                            {
                                break;
                            }
                        }

                        RT_BT_LOG(YELLOW, MSG_LE_RSSI_S3_RX_PKT_SIM, 1, status);

                        if (status == LE_WAKEON_STATE_FAIL_IN_RSSI_LIST)
                        {
                            drop = TRUE;                  
                        } 
                    }
                }
                else if ((event_pkt->event_opcode == HCI_INQUIRY_RESULT_WITH_RSSI_EVENT) ||
                        (event_pkt->event_opcode == HCI_EXTENDED_INQUIRY_RESULT_EVENT))
                {
                    UINT8 num_res;
                    UINT8 offset = 1;

                    if (event_pkt->event_opcode == HCI_EXTENDED_INQUIRY_RESULT_EVENT)
                    {
                        /* avoid driver's error */                    
                        num_res = 1;
                    }
                    else
                    {
                        num_res = event_pkt->event_parameter[0];
                    }

                    if ((ll_rssi_manager.max_bredr_cnt > 0) &&
                        (LMP_NO_CONNECTION()))
                    {                                                               
                        while (num_res--)
                        {
                            UINT8 *addr = &event_pkt->event_parameter[offset];
                            INT8 rssi = (INT8)event_pkt->event_parameter[offset + 13];

                            status = rssi_app_le_check_rssi_condition(2, addr, rssi);

                            if (status == LE_WAKEON_STATE_PASS_IN_RSSI_LIST)
                            {
                                break;
                            }
                            offset += 14;                        
                        }

                        RT_BT_LOG(YELLOW, MSG_LE_RSSI_S3_RX_PKT_SIM, 1, status);
                        
                        if (status != LE_WAKEON_STATE_PASS_IN_RSSI_LIST)
                        {
                            drop = TRUE;                 
                        } 
                    }       

                }

                if (drop)
                {
                    OS_FREE_BUFFER(tx_table[HCI_EVENT_HANDLER_TASK].pool_handle, 
                                                    (UCHAR *)signal->param);
                    break;
                }
            }

#endif
            
            hci_td_tx_packet(HCI_TRANSPORT_EVENT_PKT_TYPE,
                             (UCHAR *)signal->param,
                             signal->length);
            break;
        }

        case HCI_TD_ACL_DATA_TO_HOST_SIGNAL:
#ifdef _DMA_LOOPBACK_TEST_
			hci_td_tx_packet(HCI_TRANSPORT_ACL_DATA_PKT_TYPE,
							 (UCHAR *)signal->param,
							 signal->length);
#else

#ifdef _DAPE_TEST_FIX_INQUIRY_OTHER_DEVICE
            if (signal->type == HCI_TD_ACL_DATA_TO_HOST_SIGNAL)
            {
                HCI_ACL_RX_DATA_PKT *hci_pkt = (HCI_ACL_RX_DATA_PKT *)signal->param;

                if ((hci_pkt->broadcast_flag != 0) &&
#ifdef LE_MODE_EN
                    (hci_pkt->connection_handle  == (LL_HCI_MAX_CONN_HANDLE+5)))
#else
                    (hci_pkt->connection_handle  == (LMP_MAX_CONN_HANDLES+8+5)))
#endif
                {
                    /* drop false alarm EIR packet */
                    OS_FREE_BUFFER(tx_table[ACL_DATA_HANDLER_TASK].pool_handle,hci_pkt);
                    os_free_reserved_buffer();   

                    /* TODO: flush acl rx fifo or notify host to reset us */
                    if (lmp_self_device_data.number_of_hlc == 0)
                    {
                        UINT16 priority_control1 = BB_read_baseband_register(SCA_PRIORITY_REGISTER2);
                        priority_control1 &= (UINT16)(0xFFEF); 
                        BB_write_baseband_register(SCA_PRIORITY_REGISTER2, priority_control1);
                        lc_flush_rx_fifo();
                        priority_control1 |= (UINT16)(0x0010);
                        BB_write_baseband_register(SCA_PRIORITY_REGISTER2, priority_control1);
                    }	
                    else
                    {
                        /* flush acl rx fifo or notify host to reset us */
#ifdef _DAPE_TEST_SEND_EVENT_WHEN_ACL_FIFO_WRONG				
                        hci_generate_hw_error_event(0x33);
#endif
                    }
                    break;          
                }        
            }
#endif	

#if defined(HS_USB_SEND_EVENT_REORDER_ISSUE) || defined(HS_USB_SEND_EVENT_REORDER_ISSUE_NEW)
            if ((g_fun_interface_info.b.hci_sel == PACKAGE_USB_USB) ||
                IS_EVENT_REORDER_FOR_ANY_INTERFACE)
            {
                OS_SIGNAL sig_send;  
                HCI_ACL_RX_DATA_PKT *acl_pkt = (HCI_ACL_RX_DATA_PKT *) signal->param;

                /* send the acl-u rx packet to the hci_event_task */
                sig_send.type = HCI_DELIVER_ACL_DATA_SIGNAL;
                sig_send.param = acl_pkt;
                OS_SEND_SIGNAL_TO_TASK(hci_event_task_handle, sig_send);       
                break;
            }
#endif

            hci_handle_data_to_host((HCI_ACL_RX_DATA_PKT *)signal->param);
#endif
            break;

#ifdef LE_MODE_EN
        case HCI_TD_LE_ACL_DATA_TO_HOST_SIGNAL:
#ifdef HS_USB_SEND_EVENT_REORDER_ISSUE_NEW
            if ((g_fun_interface_info.b.hci_sel == PACKAGE_USB_USB)
                    || IS_EVENT_REORDER_FOR_ANY_INTERFACE)
            {
                if (ll_manager.conn_unit.l2h_acl_pkt_list.in_procedure)
                {
                    OS_SIGNAL sig_send;
                    sig_send.type = HCI_DELIVER_LE_ACL_DATA_SIGNAL;
                    OS_SEND_SIGNAL_TO_TASK(hci_event_task_handle, sig_send);
                }
                break;
            }
#endif
            if (ll_manager.conn_unit.l2h_acl_pkt_list.in_procedure)
            {
                LL_HCI_ACL_DATA_PKT *acl_pkt;
                LL_HCI_ACL_DATA_PKT *acl_pkt_next;                
                LL_CONN_HANDLE_UNIT *handle;
                UINT32 pool_id = tx_table[LE_ACL_DATA_HANDLER_TASK].pool_handle;
                
                acl_pkt = ll_manager.conn_unit.l2h_acl_pkt_list.pDataHead;
                
                while (acl_pkt != NULL)
                {
                    acl_pkt_next = acl_pkt->next;
                    acl_pkt->next = NULL;

                    handle = ll_fw_search_handle_unit_via_conn_handle(
                                                    acl_pkt->connection_handle);
                    
                    if ((handle != NULL) && handle->connected)
                    {
#ifndef LE_HW_TEST
                        hci_handle_data_to_host((HCI_ACL_RX_DATA_PKT *)acl_pkt);
#else
                        ll_test_receive_rx_data_pkt(acl_pkt);
#endif
                    }
                    else
                    {
                        OS_FREE_BUFFER(pool_id, acl_pkt);

#ifdef _ENABLE_BLE_LL_FLOW_CONTROL_
                        os_release_one_le_reserved_buffer();
#endif    
                    }
                    acl_pkt = acl_pkt_next;
                }

                ll_manager.conn_unit.l2h_acl_pkt_list.pDataHead = NULL;
                ll_manager.conn_unit.l2h_acl_pkt_list.pDataTail = NULL;                
                ll_manager.conn_unit.l2h_acl_pkt_list.pktcnt = 0;
                ll_manager.conn_unit.l2h_acl_pkt_list.in_procedure = FALSE;                
            }           
            break;
#endif /* LE_MODE_EN */

        case HCI_TD_HANDLE_TX_COMPLETED_SIGNAL:
            hci_td_tx_complete((UCHAR *)signal->param, signal->fifo_type);
            break;

        case HCI_TD_SYNCHRONOUS_DATA_TO_HOST_SIGNAL:
            {
                //TD_INF(log_file, "Received Synchrnous Data packet to send to host");
				TD_INF(RECEIVED_SYNCHRNOUS_DATA_PACKET_TO_SEND_TO_HOST,0,0);
                hci_td_tx_packet(HCI_TRANSPORT_SYNC_DATA_PKT_TYPE,
                                 (UCHAR *)signal->param, signal->length);
            }
            break; 

		case HCI_TD_ACL_TO_HOST_TEST_SIGNAL:
			{
				//TD_INF(log_file, "Received Synchrnous Data packet to send to host");
				//TD_INF(RECEIVED_SYNCHRNOUS_DATA_PACKET_TO_SEND_TO_HOST,0,0);
				hci_td_tx_packet(HCI_TRANSPORT_ACL_DATA_PKT_TYPE,
					(UCHAR *)signal->param, signal->length);
			}
			break;

#ifdef ENABLE_PLATFORM_TASK_HOOK
        case PLATFORM_TASK_SIGNAL:
            pf_task((INT32)signal->param, (void *)signal->ext_param);
            break;
#endif /* ENABLE_PLATFORM_TASK_HOOK */

        default:
            //TD_INF(log_file,"Invalid Signal Type received by Transport Task");
			TD_INF(INVALID_SIGNAL_TYPE_RECEIVED_BY_TRANSPORT_TASK,0,0);
            break;
    }
    return BT_FW_SUCCESS;
}

#ifdef ENABLE_PLATFORM_TASK_HOOK
BOOLEAN pf_task_send_signal(INT32 pf_task_signal, void* arg)
{
    OS_SIGNAL signal;

    signal.type = PLATFORM_TASK_SIGNAL;
    signal.param = (OS_ADDRESS)pf_task_signal;
    signal.ext_param = (OS_ADDRESS)arg;

    if (OS_SEND_SIGNAL_TO_TASK(hci_td_task_handle, signal) != BT_ERROR_OK)
    {
        return FALSE;
    }
    else
    {
        return TRUE;
    }
}
#endif /* ENABLE_PLATFORM_TASK_HOOK */

#ifdef _DAPE_SEND_ENCRYPTION_COMMAND_DISALLOWED_AFTER_5SEC
void send_enc_command_disallow_timer_func(TimerHandle_t timer_handle)
{
    UINT16 ce_index;
    if((LMP_GET_CE_INDEX_FROM_CONN_HANDLE(g_send_enc_command_disallow_conn_handle, 
                                                   &ce_index) == API_SUCCESS) &&
                 (lmp_connection_entity[ce_index].entity_status == ASSIGNED))
    {
        UCHAR event_parameter[4];
        event_parameter[0] = COMMAND_DISALLOWED_ERROR;
        event_parameter[1] = (UCHAR)g_send_enc_command_disallow_conn_handle;
        event_parameter[2] = (UCHAR)(g_send_enc_command_disallow_conn_handle>>8);
        event_parameter[3] = (UCHAR)g_send_enc_mode;
        hci_generate_event(HCI_ENCRYPTION_CHANGE_EVENT, event_parameter, 4);
    }
}
#endif


