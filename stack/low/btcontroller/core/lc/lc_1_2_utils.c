/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  Contains BT_Spec1.2 LC module implementation.
 */

/********************************* Logger *************************/
enum { __FILE_NUM__= 37 };
/********************************* Logger *************************/

/* ========================= Include File Section ========================= */
#include "lc.h"
#include "lc_internal.h"
#include "btc_defines.h"
#include "bz_debug.h"
#include "mem.h"
#include "bz_pf_isoch.h"
#include "UartPrintf.h"
#include "bzdma.h"
#include "lc_1_2_internal.h"
#ifdef _DAPE_TEST_GEN_FAKE_ESCO_DATA
UINT8 g_change_flag = 0;
#endif
/* ==================== Sturcture declaration Section ===================== */



/* ===================== Variable Declaration Section ===================== */



/* ================== Static Function Prototypes Section ================== */



/* ===================== Function Definition Section ====================== */

#ifdef COMPILE_AFH_HOP_KERNEL

/**
 * Resets the AFH states, to synchronize with MSS. This function has to
 * be called only from interrupt context. Do not call this function from
 * task context.
 *
 * \param ce_index Index to lmp-connection-entity of the connection.
 *
 * \return None.
 */
void lc_handle_mss_with_afh(UINT16 ce_index)
{
    OS_SIGNAL sig_send;
    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    /* Stop AFH instant timer if running */
    if (ce_ptr->afh_instant_timer_handle != NULL)
    {
        sig_send.type = LMP_DELETE_TIMER_HANDLE_SIGNAL;
        sig_send.param =  (OS_ADDRESS) (ce_ptr->afh_instant_timer_handle);
        OS_SEND_SIGNAL_TO_TASK(lmp_task_handle, sig_send);

        ce_ptr->afh_instant_timer_handle = NULL;
    }
    ce_ptr->waiting_for_set_afh_pdu_ack = FALSE;
    ce_ptr->afh_ch_cl_reporting_mode = AFH_DISABLE;

#ifdef _CCH_IOT_CSR_RS_
    if(ce_ptr->waiting_for_rs_several_times == 0)
    {		
        ce_ptr->waiting_for_rs_several_times = 1;
    }		
//    RT_BT_LOG(YELLOW, CCH_DBG_055, 1,ce_ptr->waiting_for_rs_several_times);
#endif	

    return;
}
#endif /* COMPILE_AFH_HOP_KERNEL */


#if defined(COMPILE_ESCO) || defined(SCO_OVER_HCI)
/**
 * Receives the synchrnous data from the host and queues it in
 * the corresponding connection entity.
 *
 * \param synchronous_pkt Pointer to the synchronous packet.
 *
 * \return None.
 */
void lc_handle_synchronous_data_pkt(HCI_SYNC_DATA_PKT *synchronous_pkt)
{
    UINT16 sync_conn_handle;
    UINT16 sync_ce_index;

    sync_conn_handle = synchronous_pkt->connection_handle;
#ifdef _DAPE_TEST_GEN_FAKE_ESCO_DATA
    // 
    if (g_change_flag)
    {
    g_change_flag = 0;
    RT_BT_LOG(GREEN, YL_DBG_HEX_12, 12, 
    sync_conn_handle,
    synchronous_pkt->packet_length,
    synchronous_pkt->hci_sync_data_packet[0],
    synchronous_pkt->hci_sync_data_packet[1],
    synchronous_pkt->hci_sync_data_packet[2],
    synchronous_pkt->hci_sync_data_packet[3],
    synchronous_pkt->hci_sync_data_packet[4],
    synchronous_pkt->hci_sync_data_packet[5],
    synchronous_pkt->hci_sync_data_packet[6],
    synchronous_pkt->hci_sync_data_packet[7],
    synchronous_pkt->hci_sync_data_packet[8],
    synchronous_pkt->hci_sync_data_packet[9]);
    }
#endif

#ifdef COMPILE_ESCO
    /* Check the Connection Handle is eSCO link or not */
    if (lmp_get_esco_ce_index_from_conn_handle(sync_conn_handle,
            &sync_ce_index) == API_SUCCESS)
    {
        lc_queue_esco_data_pkt(synchronous_pkt, sync_ce_index);
        return;
    }
#endif /* COMPILE_ESCO */

#ifdef SCO_OVER_HCI
    /* Check the Connection Handle is SCO link or not */
    if (lmp_get_sco_ce_index_from_conn_handle(sync_conn_handle,
            &sync_ce_index) == API_SUCCESS)
    {
        BOOLEAN result;
        result = pf_hci_transport_isoch_queue_pkt((void*)synchronous_pkt);
        if (result == FALSE)
        {
            dma_tx_fifo_pkt_free((void * )synchronous_pkt,
                           HCI_TRANSPORT_SYNC_DATA_PKT_TYPE);               
        }
        return;
    }
#endif /* SCO_OVER_HCI */

    /* invalid Connection Handle in the Header of HCI Synchronous Data pkt */

    RT_BT_LOG(GRAY, LC_1_2_UTILS_176, 1, sync_conn_handle);

    dma_tx_fifo_pkt_free((void * )synchronous_pkt,
                             HCI_TRANSPORT_SYNC_DATA_PKT_TYPE);

#ifndef _SCO_PACKET_ERROR_DONT_REPORT_HW_ERR_EVENT_
    UCHAR *pkt_buf;

    if (OS_ALLOC_BUFFER(tx_table[HCI_EVENT_HANDLER_TASK].pool_handle,
                        (void **)(&pkt_buf)) != BT_ERROR_OK)
    {
#ifdef ENABLE_LOGGER_LEVEL_2
        LC_LOG_INFO(LOG_LEVEL_LOW, EVENT_POOL_MEMORY_ALLOCATION_FAILED_FOR_SYNC_LOST_EVENT,0,0);
#endif
        return;
    }

    /* report HW Error Event to host */
    
    pkt_buf[0] = 0x10;
    pkt_buf[1] = 0x01;
    pkt_buf[2] = PF_HCI_TRANSPORT_HW_ERROR_WRONG_PARAM_FOR_SYNC_DATA;
    hci_td_deliver_event_to_host(pkt_buf);
#endif            
}
#endif /* defined(COMPILE_ESCO || SCO_OVER_HCI) */

