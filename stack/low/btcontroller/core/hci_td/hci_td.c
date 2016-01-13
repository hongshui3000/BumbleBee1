/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  HCI Transport Driver utility functions.
 */

/********************************* Logger *************************/
enum { __FILE_NUM__= 31 };
/********************************* Logger *************************/

#include "hci_td.h"
#include "bt_fw_hci_internal.h"
#include "bt_fw_os.h"
#include "hci_vendor_defines.h"
#include "lmp_vendor_defines.h"
#include "lmp_internal.h"
#include "lc.h"
#include "platform.h"
#include "bz_debug.h"
#include "mem.h"
#include "UartPrintf.h"
#include "h5.h"

#ifdef PTA_EXTENSION
#include "pta_meter.h"
#endif

#include "bt_fw_acl_q.h"
#include "le_ll.h"

extern POOL_ID hci_cmd_pool_handle;
extern POOL_ID acl_data_pool_handle;
extern OS_HANDLE hci_rx_task_handle;
extern OS_HANDLE lc_rx_task_handle;
extern TASK_ID  hci_event_task_handle;
extern UINT8 bz_isoch_conns;

//20120215 morgan add
extern UINT8 bFtpAccumulateCounter;

#ifdef _LE_SPEED_UP_HCI_ACL_TX_PKT_HANDLE_
LL_HCI_ACL_DATA_PKT *pll_hci_pre_pend_list_head = NULL;
LL_HCI_ACL_DATA_PKT *pll_hci_pre_pend_list_tail = NULL;
#endif

extern void hci_handle_host_data_pkt_in_local_loopback(HCI_ACL_DATA_PKT *);
#if defined(SCO_OVER_HCI) || defined(COMPILE_ESCO)
extern void hci_handle_host_sync_pkt_in_local_loopback(HCI_SYNC_DATA_PKT *);
#endif /* defined(SCO_OVER_HCI) || defined(COMPILE_ESCO) */

extern API_RESULT lmp_get_esco_ce_index_from_conn_handle(UINT16 conn_handle,
        UINT16* ce_index);
API_RESULT send_next_data_packet(UINT16 connection_handle);

/**
 * Forwards the packet to the respective task for it to be processed.
 *
 * \param pkt_type Packet Type (CMD, ACL, SYNC).
 * \param buffer The Packet buffer
 * \param length Length of the buffer.
 *
 * \return None.
 */
void hci_td_handle_received_packet(HCI_TRANSPORT_PKT_TYPE pkt_type,
                                   UCHAR *buffer, UINT16 length, UINT32 fifo_index)
{
    OS_SIGNAL signal;
    HCI_ACL_DATA_PKT *ppkt;
    UINT32 task_id;

    signal.length =  length;
    signal.param = buffer;
    signal.fifo_type = pkt_type;
    signal.fifo_index = fifo_index;

    switch(pkt_type)
    {
    case HCI_TRANSPORT_CMD_PKT_TYPE:
#ifndef _FIX_RACE_CONDITION_ISSUE_OF_NUM_HCI_CMD_PKT_FIELD_
        lmp_self_device_data.num_hci_command_packets--;
#endif
        signal.type = HCI_CMD_RECD_SIGNAL;
        task_id = rx_table[HCI_CMD_HANDLER_TASK].task_handle;
        break;

    case HCI_TRANSPORT_ACL_DATA_PKT_TYPE:
        ppkt = (HCI_ACL_DATA_PKT*)buffer;
        ppkt->ws->flush_time = BB_read_native_clock();

#ifdef TEST_MODE
        if (lmp_self_device_data.test_mode == HCI_LOCAL_LOOPBACK_MODE)
        {
            hci_handle_host_data_pkt_in_local_loopback(ppkt);
            return;
        }
#endif

#ifdef PTA_EXTENSION

        // morgan add 20111026
        /* ACL TX measure */
        if( pta_meter_var.bPtaMeterSwitch )
        {
        	pta_meter_var.dwPtaACLTxCnt += length;
        }

        if (is_wifi_alive)
        {
            /* --------------------------------*/
            /* the flushable flag for A2DP estimation  */
            /* --------------------------------*/
/* morgan add for A2DP estimation */

#ifdef SOURCECODE_TEST_CLOSE_FW_PROFILE_ESTIMATE
#ifdef A2DP_FLUSHABLE
            pta_a2dp_estimation(ppkt);
#endif

#ifdef A2DP_SBC_SYNCWORD
            pta_a2dp_sbc_estimation2(buffer);
#endif

#ifdef FTP_ESTIMATION
            /* morgan add for FTP estimation */
            pta_ftp_estimation(ppkt);
#endif
#endif
        }
#endif

        signal.type = LC_HANDLE_HOST_DATA_PKT;
        task_id = rx_table[ACL_DATA_HANDLER_TASK].task_handle;
        break;

    case HCI_TRANSPORT_SYNC_DATA_PKT_TYPE:
#if defined(SCO_OVER_HCI) || defined(COMPILE_ESCO)
#ifdef TEST_MODE
        if (lmp_self_device_data.test_mode == HCI_LOCAL_LOOPBACK_MODE)
        {
            //chris_text_mode_modify
            hci_handle_host_sync_pkt_in_local_loopback((HCI_SYNC_DATA_PKT *)buffer);
            return;
        }
#endif

#ifdef PTA_EXTENSION /* morgan 20120104 add */
        if (is_wifi_alive)
        {
            if ( pta_meter_var.bPtaMeterSwitch )
            {
                pta_meter_var.dwPtaSCOTxCnt += length;
            }
        }
#endif	/* morgan 20120104 add */

        signal.type = LC_HANDLE_HOST_SYNCHRONOUS_PKT;
        task_id = rx_table[SYNCHRONOUS_DATA_HANDLER_TASK].task_handle;
#endif //#if defined(SCO_OVER_HCI)
        break;

#ifdef LE_MODE_EN
    case HCI_TRANSPORT_LE_ACL_DATA_PKT_TYPE:
        signal.type = LL_HANDLE_HOST_DATA_PKT;
        task_id = rx_table[LE_ACL_DATA_HANDLER_TASK].task_handle;

#ifdef _LE_SPEED_UP_HCI_ACL_TX_PKT_HANDLE_
        {
            LL_HCI_ACL_DATA_PKT *ppkt;
            ppkt = (LL_HCI_ACL_DATA_PKT *)buffer;

            ppkt->next = NULL;

            if (pll_hci_pre_pend_list_head == NULL)
            {
                pll_hci_pre_pend_list_head = ppkt;
                pll_hci_pre_pend_list_tail = ppkt;
                OS_SEND_SIGNAL_TO_TASK(task_id,signal);
                return;
            }
            else
            {
                pll_hci_pre_pend_list_tail->next = ppkt;
                pll_hci_pre_pend_list_tail = ppkt;
            }
        }
#endif
        break;

#endif

    default :
        //HI_DRIVER_TRC2(log_file,"Invalid Packet type Received in hci_td_handle_received_packet UNRECOVERABLE ERROR \n");
        HI_DRIVER_TRC2(HCI_TD_HANDLE_RECEIVED_PACKET_UNRECOVERABLE_ERROR,0,0);
        return;
    }

    OS_SEND_SIGNAL_TO_TASK(task_id,signal);

    return;
}

/**
 * Transmits a complete packet using the configured transport.
 * The packet is first queued in a internal transmit queue of
 * the transport driver and if the transport is free it is
 * written to the transport.See ALSO hci_td_tx_packet.
 *
 * \param pkt_type Packet Type (Event, ACL, SYNC).
 * \param buffer Buffer containing the packet.
 * \param length Length of the buffer.
 *
 * \return None.
 */
void hci_td_tx_packet_data(HCI_TRANSPORT_PKT_TYPE pkt_type, UCHAR *buffer,
                           UINT16 length, UCHAR *data_buffer,
                           UINT16 data_length)
{
    //TD_INF(log_file," Packet Queued to be sent to Host pkt_type = %d Length = %d \n",pkt_type,length);
    TD_INF(PACKET_QUEUED_TO_BE_SENT_TO_HOST_PKT_TYPE_LENGTH,2,pkt_type,length);

    pf_hci_transport_write(HCI_TRANSPORT_ACL_DATA_PKT_TYPE,
                           (UCHAR *)(data_buffer-/*(UCHAR *)*/length),
                           (UINT16)(data_length+length), 0);
    return;
}

/**
 * Transmits a complete packet using the configured transport.
 * The packet is first queued in a internal transmit queue of
 * the transport driver and if the transport is free it is
 * written to the transport.
 *
 * \param pkt_type Packet Type (Event, ACL, SYNC).
 * \param buffer Buffer containing the packet.
 * \param length Length of the buffer.
 *
 * \return None.
 */
void hci_td_tx_packet(HCI_TRANSPORT_PKT_TYPE pkt_type, UCHAR *buffer,
                      UINT16 length)
{
    //TD_INF(log_file," Packet Queued to be sent to Host pkt_type = %d Length = %d \n",pkt_type,length);
    TD_INF(PACKET_QUEUED_TO_BE_SENT_TO_HOST_PKT_TYPE_LENGTH,2,pkt_type,length);
    TD_INF(SENDING_QUEUED_PACKET_TO_HOST_PACKET,0,0);

    pf_hci_transport_write(pkt_type, buffer, length, 0);
    return;
}

/**
 * Callback for successful transmission of packet by UART or USB transport.
 * This function will reset the sate of the transport, free the
 * buffer of the transmitted packet and if there are any pendng
 * HCI packets in the queue they are transmitted.
 *
 * \param buffer Packet buffer (successfully transmitted).
 *
 * \return None.
 */
void hci_td_tx_complete(UCHAR *buffer, UINT8 pkt_type)
{
    OS_SIGNAL signal;
    HCI_EVENT_PKT *event_pkt= (HCI_EVENT_PKT*) buffer;

    /* Free the previously transmitted packet */
    switch(pkt_type)
    {
        case HCI_TRANSPORT_EVENT_PKT_TYPE:
#ifdef _UART_H5 /* for H4 error w1c after event RX */
            // (yilinli) Note: it is executed in ISR due to _FAST_HCI_TD_COMPLETE_ definition //
            /* check interface type outside to reduce unnecesary context
               switch and increase code readability - austin */
            if (g_fun_interface_info.b.bt_interface == UART_INTERFACE)
            {
                if (!g_data_uart_settings_2.h5_en)
                {
                    hci_uart_h4_err_event_complete_check(buffer);
                }
                // TODO: check change baudrate complete event and execute hci_uart_vendor_set_baud(baud_new);

                if (hci_uart_man.baud_new_valid_flag)
                {
                    hci_uart_change_baudrate_event_complete_check(buffer);
                }
#ifdef _8821A_BTON_DESIGN_
                if (hci_uart_man.chg_para_valid_flag)
                {
                    hci_uart_change_parameter_event_complete_check(buffer);
                }
#endif
            }
#endif

#ifdef _SUPPORT_AUTO_DETACH_LINK_
            if (event_pkt->event_opcode == HCI_DISCONNECTION_COMPLETE_EVENT)
            {
                if (auto_detach_link_timer != NULL)
                {
                    if (auto_detach_check_all_link_removed() == TRUE)
                    {
                        /* auto-detach is finished. we can restart the timer to
                           do final hci reset or hw power down after 10 ms */
                        OS_STOP_TIMER(auto_detach_link_timer, 0);
                        OS_START_TIMER(auto_detach_link_timer, 10);
                    }
                }
            }
#endif

#ifdef HS_USB_SEND_EVENT_REORDER_ISSUE_NEW
#ifndef _SUPPORT_3BITS_HCI_SELECTION_
#ifndef _SUPPORT_2BIT_HCI_AND_PCIE_UART_
            if ((g_fun_interface_info.b.hci_sel == COMBO_INF_USB_MULTI)
#else
            if ((g_fun_interface_info.b.hci_sel == PACKAGE_USB_USB)
#endif
#else
            if ((g_fun_interface_info.b.hci_sel == PACKAGE_USB_USB)
#endif

                    || IS_EVENT_REORDER_FOR_ANY_INTERFACE)
            {
                UINT8 status;
                UINT16 conn_handle;
                if (hci_td_check_hci_event_need_in_order(event_pkt, &status,
                        &conn_handle))
                {
                    if (status == HCI_COMMAND_SUCCEEDED)
                    {
                        HCI_ACL_RX_DATA_PKT *acl_pkt;
                        ACL_RX_DATA_WAIT_QUEUE *wq;

                        DEF_CRITICAL_SECTION_STORAGE;
                        MINT_OS_ENTER_CRITICAL();
                        wq = aclq_rx_wait_queue_find(conn_handle);
                        if (wq != NULL)
                        {
                            void (*send)(void *) = wq->send;
                            /* copy the acl-u pkt list in the wait queue then
                               reset the catch */
                            acl_pkt = aclq_rx_wait_queue_dequeue_all(wq);
                            aclq_rx_wait_queue_free(conn_handle);
                            MINT_OS_EXIT_CRITICAL();

                            /* this delay is a must. it is used to cover the worse case
                               (1 ms latency) before usb host to send INT In token */
                            pf_delay(1);

                            if (acl_pkt != NULL)
                            {
                                /* send acl-u pkt from the wait queue */
                                if (send != NULL)
                                {
                                    send(acl_pkt);
                                }
                            }
                        }
                        else
                        {
                            MINT_OS_EXIT_CRITICAL();
                        }
                    }
                }
            }
#endif
#ifdef HS_USB_SEND_EVENT_REORDER_ISSUE
            if ((g_fun_interface_info.b.hci_sel == PACKAGE_USB_USB)
                || IS_EVENT_REORDER_FOR_ANY_INTERFACE)
            {
                if (event_pkt->event_opcode == HCI_CONNECTION_COMPLETE_EVENT)
                {
                    if (event_pkt->event_parameter[0] == HCI_COMMAND_SUCCEEDED)
                    {
                        HCI_ACL_RX_DATA_PKT *acl_pkt;
                        UINT16 conn_handle = event_pkt->event_parameter[1] |
                                            (event_pkt->event_parameter[2] << 8);

                        DEF_CRITICAL_SECTION_STORAGE;
                        MINT_OS_ENTER_CRITICAL();
                        if (conn_handle == hs_recd_conn_handle)
                        {
                            /* copy the acl-u pkt list in the wait queue then
                               reset the catch */
                            acl_pkt = hs_wait_queue_acl_head;
                            hs_wait_queue_acl_head = NULL;
                            hs_wait_queue_acl_tail = NULL;
                            hs_recd_conn_handle = 0xBEEF;
                            MINT_OS_EXIT_CRITICAL();

                            /* this delay is a must. it is used to cover the worse case
                               (1 ms latency) before usb host to send INT In token */
                            pf_delay(1);

                            if (acl_pkt != NULL)
                            {
                                /* send acl-u pkt from the wait queue */
                                aclq_send_acl_pkt_in_wait_queue(acl_pkt);
                            }
                        }
                        else
                        {
                            MINT_OS_EXIT_CRITICAL();
                        }
                    }
                }
            }
#endif

            if (IS_SUPPORT_HCI_LOG_PKT)
            {
                if (dbg_vendor_set_log_complete_event_buf == buffer)
                {
                    /* we get the command complete event for
                       vendor set command */
                    dbg_vendor_set_log_complete_event_buf = NULL;
                }
                else
                {
                    hci_vendor_check_log_data_packet(VENDOR_LOG_PACKET_TYPE_EVENT,
                                                     buffer, TRUE);
                }
            }

            if (OS_FREE_BUFFER(tx_table[HCI_EVENT_HANDLER_TASK].pool_handle,buffer) != BT_ERROR_OK)
            {
                RT_BT_LOG(GRAY, HCI_TD_872, 0, 0);
            }

            break;

#ifdef LE_MODE_EN
        case HCI_TRANSPORT_LE_ACL_DATA_PKT_TYPE:
            signal.type = HCI_HANDLE_DATA_TX_SIGNAL;
            signal.param = (OS_ADDRESS)((UINT32)pkt_type);
            OS_SEND_SIGNAL_TO_TASK(hci_event_task_handle,signal);
            break;
#endif

        case HCI_TRANSPORT_ACL_DATA_PKT_TYPE:
            signal.type = HCI_HANDLE_DATA_TX_SIGNAL;
            signal.param = (OS_ADDRESS)((UINT32)pkt_type);
            OS_SEND_SIGNAL_TO_TASK(hci_event_task_handle,signal);
            break;

        case HCI_TRANSPORT_SYNC_DATA_PKT_TYPE:
            // TODO: For SCO, hci layer can't free buffer. Wait for API BB provided.

            OS_FREE_BUFFER(synchronous_data_to_host_pool_id, buffer);

#ifdef SCO_OVER_HCI
#ifdef _FIX_MULTIPLE_SCO_RECORD_PATH_
            if (bz_isoch_conns > 1)
            {
                signal.type = HCI_SEND_SYNCHRONOUS_PACKET_SIGNAL;
                OS_SEND_SIGNAL_TO_TASK(hci_event_task_handle, signal);
            }
#endif
#endif
            break;

        default : /* Will  never come here */
            //TD_ERR(log_file,"Invalid First Byte");
            TD_ERR(INVALID_FIRST_BYTE,0,0);
            break;
    }

}

/**
 * Copy packet content from hci acl tx data packet to hci acl rx data packet.
 * Then we send a copied packet to host.
 *
 * \param pkt Packet pointer.
 *
 * \return None.
 */
void Send_loopback_DataToHost(HCI_ACL_DATA_PKT *pkt)
{
    HCI_ACL_RX_DATA_PKT *acl_pkt;
    OS_SIGNAL sig_send;

    if(OS_ALLOC_BUFFER(tx_table[ACL_DATA_HANDLER_TASK].pool_handle, (void **)(&acl_pkt))!= BT_ERROR_OK)
    {
        return;
    }
    acl_pkt->acl_data_total_length = pkt->acl_data_total_length;
    acl_pkt->connection_handle = pkt->connection_handle;
    acl_pkt->packet_boundary_flag = 0;
    acl_pkt->broadcast_flag = 0;

    memcpy(acl_pkt->hci_acl_data_pkt,pkt->hci_acl_data_pkt,
           pkt->acl_data_total_length);

    sig_send.type = HCI_DELIVER_ACL_DATA_SIGNAL;
    sig_send.param=acl_pkt;
    OS_SEND_SIGNAL_TO_TASK(hci_event_task_handle,sig_send);
}

#if defined(SCO_OVER_HCI) || defined(COMPILE_ESCO)
/**
 * Copy packet content from hci sync tx data packet to hci sync rx data packet.
 * Then we send a copied packet to host.
 *
 * \param pkt Packet pointer.
 *
 * \return None.
 */
void Send_loopback_SyncDataToHost(HCI_SYNC_DATA_PKT *pkt)
{
    HCI_SYNC_DATA_PKT *sco_pkt;
    OS_SIGNAL sig_send;

    if( OS_ALLOC_BUFFER(synchronous_data_to_host_pool_id,
                        (void **)(&sco_pkt))!= BT_ERROR_OK)
    {
        return;
    }
    sco_pkt->packet_length = pkt->packet_length;
    sco_pkt->connection_handle = (UINT16) pkt->connection_handle;

    memcpy(sco_pkt->hci_sync_data_packet,pkt->hci_sync_data_packet,
           pkt->packet_length);

    sig_send.type = HCI_DELIVER_SYNCHRONOUS_DATA_SIGNAL;
    sig_send.param=sco_pkt;
    OS_SEND_SIGNAL_TO_TASK(hci_event_task_handle,sig_send);
}
#endif /* defined(SCO_OVER_HCI) || defined(COMPILE_ESCO) */

void hci_send_event(UINT16 conn_handle)
{
    UCHAR event_parameter[5];
    /* Set default connection handle */
    event_parameter[0] = 0x1;
    event_parameter[1] = LSB(conn_handle);
    event_parameter[2] = MSB(conn_handle);
    event_parameter[3] = 0x01;
    event_parameter[4] = 0x00;

    hci_generate_event(HCI_NUMBER_OF_COMPLETED_PACKETS_EVENT,
            event_parameter, 5);
}

void hci_handle_host_data_pkt_in_local_loopback(HCI_ACL_DATA_PKT *pkt)
{
    //chris_text_mode_modify
    Send_loopback_DataToHost(pkt);
    hci_send_event((UINT16)pkt->connection_handle);
    dma_tx_fifo_pkt_free((void * )pkt, HCI_TRANSPORT_ACL_DATA_PKT_TYPE);
}

#if defined(SCO_OVER_HCI) || defined(COMPILE_ESCO)
void hci_handle_host_sync_pkt_in_local_loopback(HCI_SYNC_DATA_PKT *pkt)
{
    //chris_text_mode_modify
    Send_loopback_SyncDataToHost(pkt);
    hci_send_event((UINT16)pkt->connection_handle);
    dma_tx_fifo_pkt_free((void * )pkt, HCI_TRANSPORT_SYNC_DATA_PKT_TYPE);
}
#endif /* defined(SCO_OVER_HCI) || defined(COMPILE_ESCO) */





