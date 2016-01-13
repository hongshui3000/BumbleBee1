enum { __FILE_NUM__= 202 };

#include "le_ll.h"
#include "le_hw_reg.h"
#include "bb_driver.h"
#include "bt_fw_hci_internal.h"
#include "bt_fw_signals.h"
#include "bzdma.h"
#include "lc.h"
#include "lc_internal.h"
#include "le_ll_driver.h"
#include "lmp.h"
#include "mem.h"
#include "le_hci_4_0.h"
#ifdef LE_HW_TEST
#include "le_ll_test.h"
#endif

#include "lmp_vendor_defines.h"
#include "gpio.h"
#include "lmp_internal.h"

#ifdef _DAPE_TEST_CHK_LE_EARLY_INTR
UINT32 g_ll_ticks = 0;
#endif
#ifdef _PAUSE_SCO_FOR_LE_CONN
extern UINT8 lc_sco_pause_status;
#endif
#ifdef _DAPE_TEST_SLOT_OFFSET_FOR_LE
/* to decide if controller has to change all its connections to the same intervals */
/* if IS_LE_CONN_INTV_AUTO_UPDATE_EN    = 1, then all connections's interval =     */
/* (g_le_conn_interval*1.25) ms.                                                   */
/* We have two choice for g_le_conn_interval. If IS_LE_CONN_INTV_DEFAULT_EN = 1,*/
/* then g_le_conn_interval = LE_CONN_INTV_DEFAULT_VALUE. otherwise,                */
/* g_le_conn_interval = the connection_interval of first connection.               */
UCHAR  g_le_conn_interval = 3;
UINT16 g_le_conn_interval_changed = 0;
#endif
#ifdef _DAPE_TEST_NEW_HW_BLOCK_ACL_FOR_LE
UINT8 g_enable_le_block_legacy = 0;
extern UINT8 g_block_legacy_for_le_slot_num;
#endif

UINT8 g_le_event_end_flag_in_this_isr = FALSE;

#ifdef _YL_TEST_MODEM_RX_REPORT
extern UINT32 g_rx_report_last_native_clock;
#endif

#ifdef _ROM_CODE_PATCHED_
PF_ROM_CODE_PATCH_FUNC rcp_ll_hit_conn_adv_isr_func = NULL;
#ifdef _DAPE_8723_RCP_
PF_ROM_CODE_PATCH_FUNC rcp_ll_update_slot_in_ce_end_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_ll_ce_begin_func = NULL;
#endif

#ifdef _CCH_8821_RCP_
PF_ROM_CODE_PATCH_VOID rcp_ll_cleanup_rx_status_ae_end = NULL;
#endif

#ifdef _CCH_8821B_TC_RCP_
PF_ROM_CODE_PATCH_VOID rcp_ll_handle_event_end_interrupt = NULL;
#endif


PF_ROM_CODE_PATCH_FUNC rcp_ll_isr_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_ll_handle_scan_start_intr_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_ll_early_func = NULL;
#endif

#ifdef _MODI_LPS_AFTER_RTL8821B_TC_
extern UINT32 g_lps_timer_counter;
extern UINT32 g_timer2_init_value;
#endif

#ifdef _MODI_LPS_AFTER_RTL8703B_
void record_le_interrupt_status();
#endif

#ifdef USE_NEW_LE_SCHEDULER
#define LE_SCHED_PKT_FIFO_MAX_SIZE  8

//LE_SCHED_PKT_FIFO *le_sched_pkt_fifo = NULL;
LE_SCHED_PKT_FIFO_NODE le_sched_pkt_node_buf[LL_MAX_CONNECTION_UNITS][LE_SCHED_PKT_FIFO_MAX_SIZE];
LE_SCHED_PKT_FIFO le_sched_pkt_fifo[LL_MAX_CONNECTION_UNITS];
void le_sched_pkt_fifo_reset_entry(UINT8 entry)
{
    le_sched_pkt_fifo[entry].size = LE_SCHED_PKT_FIFO_MAX_SIZE;
    le_sched_pkt_fifo[entry].start = 0;
    le_sched_pkt_fifo[entry].end = 0;
    le_sched_pkt_fifo[entry].len = 0;
}

void le_sched_pkt_fifo_init(void)
{
    int i;

#if 0
    le_sched_pkt_fifo = os_malloc(
            sizeof (*le_sched_pkt_fifo) * bzdma_supported_le_link_num,
            MEM_TYPE_DMEM);
    if (le_sched_pkt_fifo == NULL)
    {
        return;
    }

    le_sched_pkt_fifo[0].node = os_malloc(
            sizeof (*le_sched_pkt_fifo[0].node) * bzdma_supported_le_link_num * bzdma_supported_le_max_seg_num,
            MEM_TYPE_DMEM);
    le_sched_pkt_fifo_reset_entry(0);
    for (i = 1; i < bzdma_supported_le_link_num; ++i)
    {
        le_sched_pkt_fifo[i].node = le_sched_pkt_fifo[i - 1].node + bzdma_supported_le_max_seg_num;
        le_sched_pkt_fifo_reset_entry(i);
    }
#endif
    for (i = 0; i < LL_MAX_CONNECTION_UNITS; ++i)
    {
        le_sched_pkt_fifo[i].node = le_sched_pkt_node_buf[i];
        le_sched_pkt_fifo_reset_entry(i);
    }
}

BOOLEAN le_sched_pkt_fifo_enqueue(UINT8 entry, LE_SCHED_PKT_TYPE type, void *pkt)
{
    LE_SCHED_PKT_FIFO * const fifo = &le_sched_pkt_fifo[entry];
    if (fifo->len >= fifo->size)
    {
        return FALSE;
    }
    fifo->node[fifo->end].type = type;
    fifo->node[fifo->end].pkt = pkt;
    fifo->end = (fifo->end + 1) % fifo->size;
    ++fifo->len;
    return TRUE;
}

static inline LE_SCHED_PKT_FIFO_NODE *le_sched_pkt_fifo_front(UINT8 entry)
{
    return &le_sched_pkt_fifo[entry].node[le_sched_pkt_fifo[entry].start];
}

void le_sched_pkt_fifo_dequeue(UINT8 entry)
{
    LE_SCHED_PKT_FIFO * const fifo = &le_sched_pkt_fifo[entry];
    if (fifo->len > 0)
    {
        fifo->start = (fifo->start + 1) % fifo->size;
        --fifo->len;
    }
}

void le_sched_pkt_fifo_flush_entry(UINT8 entry)
{
    if (!le_sched_pkt_fifo_is_empty(entry))
    {
        bzdma_flush_ble_data_ring_fifo(entry, TRUE);

        /* NOTE: Don't reset le_sched_pkt_fifo[entry] before reclaiming
         * underlying resources to avoid entering DLPS.
         */
        UINT8 i;
        UINT8 iter = le_sched_pkt_fifo[entry].start;
        for (i = 0; i < le_sched_pkt_fifo[entry].len; ++i)
        {
            LE_SCHED_PKT_FIFO_NODE *node = &le_sched_pkt_fifo[entry].node[iter];
            switch (node->type)
            {
            case LE_SCHED_PKT_TYPE_CTRL_PDU:
                {
                    LE_ACL_PKT_LIST_MANAGE list;
                    list.pCtrlHead = node->ctrl_pdu;
                    list.pCtrlTail = node->ctrl_pdu;
                    list.pktcnt = 1;
                    llc_append_pdu_list_to_free_list(&list);
                }
                break;
            case LE_SCHED_PKT_TYPE_MISC_PKT:
                {
                    LE_ACL_MISC_PKT_LIST_MANAGE list;
                    list.MiscHeadId = node->misc_pkt->MyNodeId;
                    list.MiscTailId = node->misc_pkt->MyNodeId;
                    list.pktcnt = 1;
                    ll_append_acl_tx_misc_pkt_list_to_list(&list, entry,
                            LL_LINKED_LIST_INSERT_POLICY_TAIL,
                            LL_LINKED_LIST_TYPE_TX_FREE);
                }
                break;
            }
            iter = (iter + 1) % le_sched_pkt_fifo[entry].size;
        }

        if (ll_manager.conn_unit.llc_free_pkt_list.pktcnt > 0)
        {
            llc_free_pdu_in_free_list();
        }
        if (ll_manager.conn_unit.handle[entry].h2l_free_misc_pkt_list.pktcnt > 0)
        {
            ll_free_acl_tx_misc_pkts_in_free_list(entry);
        }
    }

    le_sched_pkt_fifo_reset_entry(entry);
}
#endif /* USE_NEW_LE_SCHEDULER */

/**************************************************************************
 * Function     : ll_pop_all_rx_int_stack
 *
 * Description  : This function is used to (1) clean hw interrupt of event end
 *                and (2) pop all rx status from the hw fifo then copy and push
 *                to sw fifo.
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
SECTION_LE_ISR void ll_pop_all_rx_int_stack(void)
{
    LE_REG_S_INT_MISR reg_misr;
    LE_REG_S_SET reg_union;
    UINT16 bt_int_status_reg;
    FW_BT_RX_INT_UNIT_S *punit;
    UINT16 temp0;
    UINT16 temp1;

#ifdef _USE_NEW_BLE_STACK_OVERFLOW_CHECK_DESIGN_
    temp0 = RD_LE_REG(LE_REG_STATUS_ADV_TIMEOUT_CTRL);
    temp1 = BB_read_baseband_register(ESCO_STOP_RX_REGISTER);
    if (temp0 & BIT15)
    {
        /* ce_end_stack_ovf */
        RT_BT_LOG(RED, MSG_BLE_STACK_OVERFLOW, 2, 0, 1);
    }

    if (temp1 & BIT4)
    {
        /* ce_end_stack_ovf */
        RT_BT_LOG(RED, MSG_BLE_STACK_OVERFLOW, 2, 1, 0);
    }
#endif

    fw_bt_rx_int_fifo.found_legacy = FALSE;

    while (1)
    {
        reg_union.value = RD_LE_REG(LE_REG_STATUS_RX_INT_STACK);
        punit = &fw_bt_rx_int_fifo.unit[fw_bt_rx_int_fifo.wptr];

        if (reg_union.status_rx_int_stack.rx_ind_id)
        {
            /* for BR_EDR */

            bt_int_status_reg = BB_read_baseband_register(INTERRUPT_REGISTER);
            if (!(bt_int_status_reg & (1 << 0)))
            {
#ifdef _USE_NEW_BLE_STACK_OVERFLOW_CHECK_DESIGN_
                break;
#else /* else of #ifndef _USE_NEW_BLE_STACK_OVERFLOW_CHECK_DESIGN_ */
                /* to check any BLE stack is pending (if overflow !!) */
                *(UINT16*)&reg_misr = RD_LE_REG(LE_REG_INT_MISR);
                if (!reg_misr.event_end_int)
                {
                    /* nothing, so we can quit safely !! */
                    break;
                }
                else
                {
                    /* error !! but we still need to pop
                       register to clear interrupt */
#ifdef _DAPE_TEST_NEW_HW_READ_RSSI_STACK
                    BB_read_baseband_register(BB_RSSI_STACK_REG);
#endif
                    BB_read_baseband_register(RECEIVED_PAYLOAD_HEADER_REGISTER);
                    BB_read_baseband_register(RECEIVED_STATUS_REGISTER);

                    RT_BT_LOG(RED, MSG_BLE_STACK_OVERFLOW, 2, 1, 0);

                    continue;
                }
#endif /* end of #ifndef _USE_NEW_BLE_STACK_OVERFLOW_CHECK_DESIGN_ */
            }
            else
            {
                /* In 8821, we need to read RSSI before reading RECEIVED_STATUS_REGISTER. */
#ifdef _DAPE_TEST_NEW_HW_READ_RSSI_STACK
                punit->rssi = BB_read_baseband_register(BB_RSSI_STACK_REG);
                punit->rssi &= (UINT16)(0x003F);
#endif

                /* clear the interrupt */
                temp0 = BB_read_baseband_register(RECEIVED_PAYLOAD_HEADER_REGISTER);
                temp1 = BB_read_baseband_register(RECEIVED_STATUS_REGISTER);
                punit->type = 0;
                fw_bt_rx_int_fifo.found_legacy = TRUE;
            }
        }
        else
        {
            /* for LE */

            *(UINT16*)&reg_misr = RD_LE_REG(LE_REG_INT_MISR);
            if (!reg_misr.event_end_int)
            {
#ifdef _USE_NEW_BLE_STACK_OVERFLOW_CHECK_DESIGN_
                break;
#else /* else of #ifndef _USE_NEW_BLE_STACK_OVERFLOW_CHECK_DESIGN_ */
                /* to check any BR_EDR stack is pending (if overflow !!) */
                bt_int_status_reg = BB_read_baseband_register(INTERRUPT_REGISTER);
                if (!(bt_int_status_reg & (1 << 0)))
                {
                    /* nothing, so we can quit safely !! */
                    break;
                }
                else
                {
                    /* error !! but we still need to pop
                       register to clear interrupt */
                    RD_LE_REG(LE_REG_STATUS_CE_WORD_CNT);
                    RD_LE_REG(LE_REG_STATUS_CE_END_EVENT);

                    RT_BT_LOG(RED, MSG_BLE_STACK_OVERFLOW, 2, 0, 1);

                    continue;
                }
#endif /* end of #ifndef _USE_NEW_BLE_STACK_OVERFLOW_CHECK_DESIGN_ */
            }
            else
            {
#ifdef _NEW_BLE_HW_SPEC_FROM_150320_
                /* read le anch differ rpt registers from stack */
                punit->reg2_value = RD_LE_REG(LE_REG_ANCH_DIFF_RPT0);
                punit->reg3_value = RD_LE_REG(LE_REG_ANCH_DIFF_RPT1);
#endif

                /* clear the interrupt */
                temp0 = RD_LE_REG(LE_REG_STATUS_CE_WORD_CNT);
                temp1 = RD_LE_REG(LE_REG_STATUS_CE_END_EVENT);
                punit->type = 1;
            }
        }

        punit->reg0_value = temp0;
        punit->reg1_value = temp1;

        fw_bt_rx_int_fifo.wptr++;
        fw_bt_rx_int_fifo.wptr &= (MAX_FW_BT_RX_INT_FIFO_UNITS - 1);
        fw_bt_rx_int_fifo.count++;
    }
}


SECTION_LE_ISR void ll_handle_ctrl_pdu_tx_ack_imp(LL_CONN_HANDLE_UNIT *chu,
        LL_CTRL_PDU_PAYLOAD *pdu)
{
    switch (pdu->OpCode)
    {
#ifdef _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_
    case LL_LENGTH_RSP:
        llc_finish_data_length_update_procedure(chu);
        break;
#endif
    default:
        break;
    }
}
void (*ll_handle_ctrl_pdu_tx_ack)(LL_CONN_HANDLE_UNIT *, LL_CTRL_PDU_PAYLOAD *) = ll_handle_ctrl_pdu_tx_ack_imp;

SECTION_LE_ISR void ll_handle_ctrl_pdu_tx_ack_list(LL_CONN_HANDLE_UNIT *chu,
        LE_ACL_PKT_LIST_MANAGE *pdu_list)
{
    LL_CTRL_PDU_PAYLOAD *pdu;
    for (pdu = pdu_list->pCtrlHead; pdu != NULL; pdu = pdu->pNext)
    {
        ll_handle_ctrl_pdu_tx_ack(chu, pdu);
    }
}

#ifdef USE_NEW_LE_SCHEDULER
void ll_handle_tx_ack_pkts(UINT8 entry, UINT8 acked_cnt)
{
    LL_CONN_HANDLE_UNIT *chu = &ll_manager.conn_unit.handle[entry];
    UINT8 i;
    for (i = 0; i < acked_cnt; ++i)
    {
        if (le_sched_pkt_fifo_is_empty(entry))
        {
            LL_LOG_TRACE(RED, LE_MSG_LE_SCHED_PKT_FIFO_EXHAUSTED, 1, entry);
            break;
        }
        LE_SCHED_PKT_FIFO_NODE *node = le_sched_pkt_fifo_front(entry);
        switch(node->type)
        {
        case LE_SCHED_PKT_TYPE_CTRL_PDU:
            if (chu->llc.wait_ack)
            {
                if (chu->llc.wait_sent_pdu_type == node->ctrl_pdu->OpCode)
                {
                    llc_handle_sent_llc_pdu_ack_recd(chu, node->ctrl_pdu->OpCode);
                }
            }
            ll_handle_ctrl_pdu_tx_ack(chu, node->ctrl_pdu);
            {
                LE_ACL_PKT_LIST_MANAGE list;
                list.pCtrlHead = node->ctrl_pdu;
                list.pCtrlTail = node->ctrl_pdu;
                list.pktcnt = 1;
                llc_append_pdu_list_to_free_list(&list);
            }
            break;
        case LE_SCHED_PKT_TYPE_MISC_PKT:
            {
                LE_ACL_MISC_PKT_LIST_MANAGE list;
                list.MiscHeadId = node->misc_pkt->MyNodeId;
                list.MiscTailId = node->misc_pkt->MyNodeId;
                list.pktcnt = 1;
                ll_append_acl_tx_misc_pkt_list_to_list(&list,
                        entry, LL_LINKED_LIST_INSERT_POLICY_TAIL,
                        LL_LINKED_LIST_TYPE_TX_FREE);
            }
            break;
        }
        le_sched_pkt_fifo_dequeue(entry);
    }
}
#endif /* USE_NEW_LE_SCHEDULER */

/**************************************************************************
 * Function     : ll_pre_cleanup_tx_schedule_of_event_end
 *
 * Description  : This function is used to handle the result of scheduled
 *                tx packet list in this connection event. We need to free all
 *                sent packets that receive the acknowledgement. Then we return
 *                remainding sent packets to pending packet list for next
 *                scheduling.
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
#ifdef USE_NEW_LE_SCHEDULER
SECTION_LE_ISR void ll_pre_cleanup_tx_schedule_of_event_end(void)
{
    UINT8 rptr;
    for (rptr = fw_bt_rx_int_fifo.rptr; rptr != fw_bt_rx_int_fifo.wptr;
            rptr = ((rptr + 1) & (MAX_FW_BT_RX_INT_FIFO_UNITS - 1)))
    {
        FW_BT_RX_INT_UNIT_S *punit = &fw_bt_rx_int_fifo.unit[rptr];

        if (!punit->type || punit->ce_end_status.ae_end)
        {
            continue;
        }

        /* le ce end */
        if (punit->ce_end_status.ce_end)
        {
            UINT8 cur_entry = punit->ce_end_status.entry;
            LL_CONN_HANDLE_UNIT *phandle = &ll_manager.conn_unit.handle[cur_entry];
            UINT8 acked_cnt = punit->ce_end_status.tx_pkt_num - phandle->ce_acked_tx_pkt_cnt;

            phandle->ce_acked_tx_pkt_cnt = 0;
            phandle->tx_sched_pkts_one_ce -= punit->ce_end_status.tx_pkt_num;

            ll_manager.event_manager.event_start = 0;
            ll_manager.event_manager.event_end = 1;

            ll_handle_tx_ack_pkts(cur_entry, acked_cnt);

#if defined(_SCHEDULE_BLE_MISC_PACKET_) && defined(_NEW_BZDMA_FROM_V8_)
            /* free LE ACL data misc tx packet */
            if (phandle->h2l_free_misc_pkt_list.pktcnt > 0)
            {
                ll_free_acl_tx_misc_pkts_in_free_list(cur_entry);
            }
#endif

#ifdef _NEW_BZDMA_FROM_V8_
            bzdma_update_fw_rptr_of_ble_data_ring_fifo(cur_entry, acked_cnt,
            TRUE);
#endif

            /* send signal to check any pending llc pdu for schedule */
            if (phandle->is_checking_llc_procedure)
            {
                break;
            }
            phandle->is_checking_llc_procedure = TRUE;

            OS_SIGNAL signal;
            signal.type = LL_CONTROL_SIGNAL;
            signal.param = (OS_ADDRESS) ((UINT32) cur_entry);
            OS_ISR_SEND_SIGNAL_TO_TASK(lmp_task_handle, signal);
        } /* end of if (punit->ce_end_status.ce_end) */
    }

    /* free LLC pdu packet */
    if (ll_manager.conn_unit.llc_free_pkt_list.pktcnt > 0)
    {
        llc_free_pdu_in_free_list();
    }
}
#else /* ! USE_NEW_LE_SCHEDULER */
SECTION_LE_ISR void ll_pre_cleanup_tx_schedule_of_event_end(void)
{
    FW_BT_RX_INT_UNIT_S *punit;
    LL_CONN_HANDLE_UNIT *phandle;
    LE_ACL_PKT_LIST_MANAGE *pctrl_list;
    LE_ACL_PKT_LIST_MANAGE ack_list;
    LE_ACL_PKT_LIST_MANAGE unack_list;
    UINT8 cur_entry;
    UINT8 i;
    UINT8 acked_cnt;
    UINT8 rptr = fw_bt_rx_int_fifo.rptr;
#ifndef _SCHEDULE_BLE_MISC_PACKET_
    LE_ACL_PKT_LIST_MANAGE *pdata_list;
    LL_HCI_ACL_DATA_PKT *resend_pkt;
    LE_ACL_PKT_LIST_MANAGE resend_list;
#endif
    UINT8 resend_lmp;
#ifdef _NEW_BZDMA_FROM_V8_
    UINT8 acked_cnt_tmp;
#endif

    while (fw_bt_rx_int_fifo.wptr != rptr)
    {
        punit = &fw_bt_rx_int_fifo.unit[rptr];

        do
        {
            if (!punit->type)
            {
                break;
            }

            if (punit->ce_end_status.ae_end)
            {
                /* le ae end */
                break;
            }

            if (punit->ce_end_status.ce_end)
            {
                /* le ce end */
                cur_entry = punit->ce_end_status.entry;
                phandle = &ll_manager.conn_unit.handle[cur_entry];
                pctrl_list = &phandle->tx_sched_ctrl_pkt_list;
#ifndef _SCHEDULE_BLE_MISC_PACKET_
                pdata_list = &phandle->tx_sched_data_pkt_list;
#endif
                acked_cnt = punit->ce_end_status.tx_pkt_num;
                if (phandle->tx_sched_pkts_one_ce < acked_cnt)
                {
                    /* to avoid hw's false alarm */
                    acked_cnt = phandle->tx_sched_pkts_one_ce;
                }
                acked_cnt -= phandle->ce_acked_tx_pkt_cnt;

                phandle->ce_acked_tx_pkt_cnt = 0;
                phandle->tx_sched_pkts_one_ce = 0;

                ll_manager.event_manager.event_start = 0;
                ll_manager.event_manager.event_end = 1;

                ll_manager.event_manager.bm_conn_event_window &=
                                                            ~(1 << cur_entry);

#ifdef _NEW_BZDMA_FROM_V8_
                acked_cnt_tmp = acked_cnt;
#endif

#if defined(_SCHEDULE_BLE_MISC_PACKET_) && defined(_NEW_BZDMA_FROM_V8_)
                /* check the status of resend misc acl pkt */
                LE_ACL_MISC_PKT_LIST_MANAGE *pdata_misc_list;
                LL_MISC_ACL_DATA_PKT_NODE *resent_misc_pkt;
                LE_ACL_MISC_PKT_LIST_MANAGE acl_misc_list;

                pdata_misc_list = &phandle->tx_sched_misc_pkt_list;
                resent_misc_pkt = phandle->tx_resent_misc_pkt;

                if ((resent_misc_pkt != NULL) && (acked_cnt > 0))
                {
                    /* append unacked tx packet list to free list */
                    resent_misc_pkt->NextNodeId = LL_MISC_ACL_DATA_PKT_MAX_NODES;
                    acl_misc_list.MiscHeadId = resent_misc_pkt->MyNodeId;
                    acl_misc_list.MiscTailId = resent_misc_pkt->MyNodeId;
                    acl_misc_list.pktcnt = 1;

                    /* all scheduled acl pkts receive ACK */
                    ll_append_acl_tx_misc_pkt_list_to_list(
                                &acl_misc_list, cur_entry,
                                LL_LINKED_LIST_INSERT_POLICY_TAIL,
                                LL_LINKED_LIST_TYPE_TX_FREE);

                    phandle->tx_resent_misc_pkt = NULL;
                    acked_cnt--;
                }
#endif

#ifndef _SCHEDULE_BLE_MISC_PACKET_
                /* check the status of resend acl pkt */
                resend_pkt = phandle->tx_resent_data_pkt;
                resend_lmp = FALSE;

                if ((resend_pkt != NULL) && (acked_cnt > 0))
                {
                    /* append unacked tx packet list to free list */
                    resend_pkt->next = NULL;
                    resend_list.pDataHead = resend_pkt;
                    resend_list.pDataTail = resend_pkt;
                    resend_list.pktcnt = 1;

                    /* all scheduled acl pkts receive ACK */
                    ll_append_acl_tx_pkt_list_to_list(
                        &resend_list, cur_entry,
                        LL_LINKED_LIST_INSERT_POLICY_TAIL,
                        LL_LINKED_LIST_TYPE_TX_FREE);

                    phandle->tx_resent_data_pkt = NULL;
                    acked_cnt--;
                }
#endif

                /* check the status of scheduled LLC ctrl pdu */
                if (pctrl_list->pktcnt > 0)
                {
                    if (acked_cnt >= pctrl_list->pktcnt)
                    {
                        /* check if we need to wait any ack in sent pdu to
                           stop llc procedure */
                        if (phandle->llc.wait_ack)
                        {
                            LL_CTRL_PDU_PAYLOAD *pNode = pctrl_list->pCtrlHead;

                            while (pNode != NULL)
                            {
                                if (phandle->llc.wait_sent_pdu_type == pNode->OpCode)
                                {
                                    llc_handle_sent_llc_pdu_ack_recd(phandle,
                                                                pNode->OpCode);
                                    break;
                                }
                                pNode = pNode->pNext;
                            }
                        }
                        ll_handle_ctrl_pdu_tx_ack_list(phandle, pctrl_list);
                        /* all scheduled llc pkts receive ACK */
                        llc_append_pdu_list_to_free_list(pctrl_list);

                        acked_cnt -= pctrl_list->pktcnt;
                    }
                    else if (acked_cnt == 0)
                    {
                        /* all scheduled llc pkts receive NAK,and we need
                           to retransmit these pkts */
                        llc_append_pdu_list_to_list(pctrl_list,
                                cur_entry, LL_LINKED_LIST_INSERT_POLICY_HEAD,
                                LL_LINKED_LIST_TYPE_TX_PEND);
                        resend_lmp = TRUE;
                    }
                    else
                    {
                        /* partial scheduled llc pkts receive ACK */
                        LL_CTRL_PDU_PAYLOAD *pllc_pkt_tmp;
                        LL_CTRL_PDU_PAYLOAD *pllc_pkt_tmp_pre;

                        /* to separate acked and unacked packet list */
                        pllc_pkt_tmp = pctrl_list->pCtrlHead;
                        pllc_pkt_tmp_pre = pctrl_list->pCtrlHead;
                        for (i = 0; i < acked_cnt; i++)
                        {
                            pllc_pkt_tmp_pre = pllc_pkt_tmp;
                            pllc_pkt_tmp = pllc_pkt_tmp->pNext;

                            /* check if we need to wait any ack in sent pdu to
                               stop llc procedure */
                            if (phandle->llc.wait_ack)
                            {
                                if (phandle->llc.wait_sent_pdu_type ==
                                        pllc_pkt_tmp_pre->OpCode)
                                {
                                    llc_handle_sent_llc_pdu_ack_recd(phandle,
                                                   pllc_pkt_tmp_pre->OpCode);
                                }
                            }

                        }
                        pllc_pkt_tmp_pre->pNext = NULL;

                        ack_list.pCtrlHead = pctrl_list->pCtrlHead;
                        ack_list.pCtrlTail = pllc_pkt_tmp_pre;
                        ack_list.pktcnt = acked_cnt;
                        ll_handle_ctrl_pdu_tx_ack_list(phandle, &ack_list);
                        llc_append_pdu_list_to_free_list(&ack_list);

                        unack_list.pCtrlHead = pllc_pkt_tmp;
                        unack_list.pCtrlTail = pctrl_list->pCtrlTail;
                        unack_list.pktcnt = pctrl_list->pktcnt - acked_cnt;
                        llc_append_pdu_list_to_list(&unack_list,
                                cur_entry, LL_LINKED_LIST_INSERT_POLICY_HEAD,
                                LL_LINKED_LIST_TYPE_TX_PEND);

                        acked_cnt = 0;
                        resend_lmp = TRUE;
                    }

                    /* init tx_sched_ctrl_pkt_list */
                    pctrl_list->pCtrlHead = NULL;
                    pctrl_list->pCtrlTail = NULL;
                    pctrl_list->pktcnt = 0;
                } /* end of if (pctrl_list->pktcnt > 0) */

#if defined(_SCHEDULE_BLE_MISC_PACKET_) && defined(_NEW_BZDMA_FROM_V8_)
                /* check the status of scheduled ACL tx misc data pkts */
                if (pdata_misc_list->pktcnt > 0)
                {
                    if (acked_cnt >= pdata_misc_list->pktcnt)
                    {
                        /* all scheduled acl pkts receive ACK */
                        ll_append_acl_tx_misc_pkt_list_to_list(pdata_misc_list,
                                  cur_entry, LL_LINKED_LIST_INSERT_POLICY_TAIL,
                                  LL_LINKED_LIST_TYPE_TX_FREE);

                        acked_cnt -= pdata_misc_list->pktcnt;
                    }
                    else if (acked_cnt == 0)
                    {
                        if ((phandle->tx_resent_misc_pkt == NULL) &&
                            (resent_misc_pkt == NULL))
                        {
                            /* append oldest unack packet to the resent
                               pkt manager */

                            resent_misc_pkt = &le_tx_misc_node[pdata_misc_list->MiscHeadId];
                            pdata_misc_list->pktcnt--;

                            if (pdata_misc_list->MiscHeadId != pdata_misc_list->MiscTailId)
                            {
                                pdata_misc_list->MiscHeadId = resent_misc_pkt->NextNodeId;

                                /* all scheduled acl pkts receive NAK,and we need
                                   to retransmit these pkts */
                                ll_append_acl_tx_misc_pkt_list_to_list(pdata_misc_list,
                                          cur_entry, LL_LINKED_LIST_INSERT_POLICY_HEAD,
                                          LL_LINKED_LIST_TYPE_TX_PEND);
                            }

                            resent_misc_pkt->NextNodeId = LL_MISC_ACL_DATA_PKT_MAX_NODES;
                            phandle->tx_resent_misc_pkt = resent_misc_pkt;
                        }
                        else
                        {
                            /* all scheduled acl pkts receive NAK,and we need
                               to retransmit these pkts */
                            ll_append_acl_tx_misc_pkt_list_to_list(pdata_misc_list,
                                      cur_entry, LL_LINKED_LIST_INSERT_POLICY_HEAD,
                                      LL_LINKED_LIST_TYPE_TX_PEND);
                        }
                    }
                    else
                    {
                        /* partial scheduled llc pkts receive ACK */
                        UINT8 acl_node_tmp;
                        UINT8 acl_node_tmp_pre;

                        /* separate acked and unacked packet list */
                        acl_node_tmp = pdata_misc_list->MiscHeadId;
                        acl_node_tmp_pre = pdata_misc_list->MiscHeadId;

                        for (i = 0; i < acked_cnt; i++)
                        {
                            acl_node_tmp_pre = acl_node_tmp;
                            acl_node_tmp = le_tx_misc_node[acl_node_tmp].NextNodeId;
                        }
                        le_tx_misc_node[acl_node_tmp_pre].NextNodeId =
                                                            LL_MISC_ACL_DATA_PKT_MAX_NODES;

                        /* append acked tx packet list to free list */
                        acl_misc_list.MiscHeadId = pdata_misc_list->MiscHeadId;
                        acl_misc_list.MiscTailId = acl_node_tmp_pre;
                        acl_misc_list.pktcnt = acked_cnt;
                        ll_append_acl_tx_misc_pkt_list_to_list(
                                            &acl_misc_list, cur_entry,
                                            LL_LINKED_LIST_INSERT_POLICY_TAIL,
                                            LL_LINKED_LIST_TYPE_TX_FREE);

                        if (pdata_misc_list->MiscTailId != acl_node_tmp)
                        {
                            /* if still have any unacked packet, append to
                               pending list */
                            acl_misc_list.MiscHeadId = le_tx_misc_node[acl_node_tmp].NextNodeId;
                            acl_misc_list.MiscTailId = pdata_misc_list->MiscTailId;
                            acl_misc_list.pktcnt = pdata_misc_list->pktcnt -
                                                acked_cnt - 1;
                            ll_append_acl_tx_misc_pkt_list_to_list(
                                        &acl_misc_list, cur_entry,
                                        LL_LINKED_LIST_INSERT_POLICY_HEAD,
                                        LL_LINKED_LIST_TYPE_TX_PEND);
                        }

                        /* append oldest unack packet to the resent
                           pkt manager */
                        resent_misc_pkt = &le_tx_misc_node[acl_node_tmp];
                        resent_misc_pkt->NextNodeId = LL_MISC_ACL_DATA_PKT_MAX_NODES;
                        phandle->tx_resent_misc_pkt = resent_misc_pkt;

                        acked_cnt = 0;
                    }

                    /* init tx_sched_data_pkt_list */
                    pdata_misc_list->DWord = LL_MISC_ACL_PKT_LIST_RESET_VALUE;
                } /* end of if (pdata_list->pktcnt > 0) */
#endif

#ifndef _SCHEDULE_BLE_MISC_PACKET_
                /* check the status of scheduled ACL tx data pkts */
                if (pdata_list->pktcnt > 0)
                {
                    if (acked_cnt >= pdata_list->pktcnt)
                    {
                        /* all scheduled acl pkts receive ACK */
                        ll_append_acl_tx_pkt_list_to_list(pdata_list,
                                  cur_entry, LL_LINKED_LIST_INSERT_POLICY_TAIL,
                                  LL_LINKED_LIST_TYPE_TX_FREE);

                        acked_cnt -= pdata_list->pktcnt;
                    }
                    else if (acked_cnt == 0)
                    {
                        if ((phandle->tx_resent_data_pkt == NULL) &&
                            (resend_lmp == FALSE))
                        {
                            /* append oldest unack packet to the resent
                               pkt manager */
                            resend_pkt = pdata_list->pDataHead;
                            pdata_list->pktcnt--;

                            if (pdata_list->pDataHead != pdata_list->pDataTail)
                            {
                                pdata_list->pDataHead = pdata_list->pDataHead->next;

                                /* all scheduled acl pkts receive NAK,and we need
                                   to retransmit these pkts */
                                ll_append_acl_tx_pkt_list_to_list(pdata_list,
                                          cur_entry, LL_LINKED_LIST_INSERT_POLICY_HEAD,
                                          LL_LINKED_LIST_TYPE_TX_PEND);
                            }

                            resend_pkt->next = NULL;
                            phandle->tx_resent_data_pkt = resend_pkt;
                        }
                        else
                        {
                            /* all scheduled acl pkts receive NAK,and we need
                               to retransmit these pkts */
                            ll_append_acl_tx_pkt_list_to_list(pdata_list,
                                      cur_entry, LL_LINKED_LIST_INSERT_POLICY_HEAD,
                                      LL_LINKED_LIST_TYPE_TX_PEND);
                        }
                    }
                    else
                    {
                        /* partial scheduled llc pkts receive ACK */
                        LL_HCI_ACL_DATA_PKT *pacl_pkt_tmp;
                        LL_HCI_ACL_DATA_PKT *pacl_pkt_tmp_pre;

                        /* to separate acked and unacked packet list */
                        pacl_pkt_tmp = pdata_list->pDataHead;
                        pacl_pkt_tmp_pre = pdata_list->pDataHead;

                        for (i = 0; i < acked_cnt; i++)
                        {
                            pacl_pkt_tmp_pre = pacl_pkt_tmp;
                            pacl_pkt_tmp = pacl_pkt_tmp->next;
                        }
                        pacl_pkt_tmp_pre->next = NULL;

                        ack_list.pDataHead = pdata_list->pDataHead;
                        ack_list.pDataTail = pacl_pkt_tmp_pre;
                        ack_list.pktcnt = acked_cnt;
                        ll_append_acl_tx_pkt_list_to_list(
                            &ack_list, cur_entry,
                            LL_LINKED_LIST_INSERT_POLICY_TAIL,
                            LL_LINKED_LIST_TYPE_TX_FREE);

                        if (pdata_list->pDataTail != pacl_pkt_tmp)
                        {
                            /* if still have any unacked packet, append to
                               pending list */
                            unack_list.pDataHead = pacl_pkt_tmp->next;
                            unack_list.pDataTail = pdata_list->pDataTail;
                            unack_list.pktcnt = pdata_list->pktcnt -
                                                acked_cnt - 1;
                            ll_append_acl_tx_pkt_list_to_list(
                                &unack_list, cur_entry,
                                LL_LINKED_LIST_INSERT_POLICY_HEAD,
                                LL_LINKED_LIST_TYPE_TX_PEND);
                        }

                        /* append oldest unack packet to the resent
                           pkt manager */
                        resend_pkt = pacl_pkt_tmp;
                        resend_pkt->next = NULL;
                        phandle->tx_resent_data_pkt = resend_pkt;

                        acked_cnt = 0;
                    }

                    /* init tx_sched_data_pkt_list */
                    pdata_list->pDataHead = NULL;
                    pdata_list->pDataTail = NULL;
                    pdata_list->pktcnt = 0;
                } /* end of if (pdata_list->pktcnt > 0) */
#endif

#if defined(_SCHEDULE_BLE_MISC_PACKET_) && defined(_NEW_BZDMA_FROM_V8_)
                /* free LE ACL data misc tx packet */
                if (phandle->h2l_free_misc_pkt_list.pktcnt > 0)
                {
                    ll_free_acl_tx_misc_pkts_in_free_list(cur_entry);
                }
#endif

#ifndef _SCHEDULE_BLE_MISC_PACKET_
                /* free LE ACL data tx packet */
                if (phandle->h2l_free_pkt_list.pktcnt > 0)
                {
                    ll_free_acl_tx_pkts_in_free_list(cur_entry);
                }
#endif


#ifdef _NEW_BZDMA_FROM_V8_
                bzdma_update_fw_rptr_of_ble_data_ring_fifo(cur_entry,
                                                           acked_cnt_tmp,
                                                           TRUE);
                bzdma_flush_ble_data_ring_fifo(cur_entry, TRUE);
#endif

                /* send signal to check any pending llc pdu for schedule */
                if (phandle->is_checking_llc_procedure)
                {
                    break;
                }
                phandle->is_checking_llc_procedure = TRUE;

                OS_SIGNAL signal;
                signal.type = LL_CONTROL_SIGNAL;
                signal.param = (OS_ADDRESS)((UINT32)cur_entry);
                OS_ISR_SEND_SIGNAL_TO_TASK(lmp_task_handle, signal);
            } /* end of if (punit->ce_end_status.ce_end) */
        }
        while (0);

        rptr++;
        rptr &= (MAX_FW_BT_RX_INT_FIFO_UNITS - 1);
    }

    /* free LLC pdu packet */
    if (ll_manager.conn_unit.llc_free_pkt_list.pktcnt > 0)
    {
        llc_free_pdu_in_free_list();
    }
}
#endif /* USE_NEW_LE_SCHEDULER */

#ifdef _CCH_8821B_TEST_LE_CONN_LPS
void rcp_ll_update_slot_in_ce_end_func_imp(UINT8 cur_entry)
{
    LL_CONN_HANDLE_UNIT *phandle;
    phandle = &ll_manager.conn_unit.handle[cur_entry];

    if(phandle->ce_count_no_crc_ok > (phandle->slave_latency))
    {
#ifdef _LPS_LOG_EN_
        RT_BT_LOG(RED, YL_DBG_HEX_4, 4, (phandle->ce_count_no_crc_ok), (phandle->ce_interval),
            (phandle->supervision_to), (phandle->slave_latency) );
#endif
        return;
    }

    if(g_lps_timer_counter >0)
    {
        return;
    }


    UINT16 ps_duration = phandle->ce_interval<<1;
    UINT32 ps_clock = (phandle->anchor_point_by_intr>>1) + ps_duration;

    if(ll_manager.conn_unit.master == 0)
    {
        ps_duration = ps_duration*(phandle->slave_latency + 1);
    }


    if((lc_check_lps_for_link(0,1,0) == FALSE) ||
        (ps_duration < otp_str_data.bt_deep_sleep_mode_threshold))
    {
        //RT_BT_LOG(RED, YL_DBG_HEX_3, 3, ps_duration, (phandle->ce_interval),
        //    otp_str_data.bt_deep_sleep_mode_threshold );
        return;
    }


    if(g_efuse_lps_setting_4.lps_use_state)
    {
        lps_period_state_machine_fast(ps_clock, ps_duration);
    }
}
#endif

SECTION_LE_ISR void ll_update_rx_statistics(UINT8 entry)
{
    UINT8 dw_addr;
    UINT32 rd_value;
    LL_CONN_HANDLE_UNIT *phandle = &ll_manager.conn_unit.handle[entry];

#ifdef _DAPE_MOVE_LE_SLAVE_PARAMETER_TO_CAM
    UINT8 cam_entry = 0;
    if (ll_manager.conn_unit.master)
    {
        cam_entry = entry;
    }
    dw_addr = LE_CAM_ENTRY_BASE(cam_entry) + LE_CAM_ADDR_4;
    phandle->conn_counter = ll_driver_read_cam(dw_addr) >> 16;

    dw_addr = LE_CAM_ENTRY_BASE(cam_entry) + LE_CAM_ADDR_14;
    rd_value = ll_driver_read_cam(dw_addr);

    if (rd_value & BIT24)
    {
        /* all rx pkts no crc OK */
        phandle->all_rx_pkt_no_crc_ok = TRUE;
        phandle->ce_count_no_crc_ok++;
    }
    else
    {
        /* at least one rx pkt is crc ok */
        phandle->all_rx_pkt_no_crc_ok = FALSE;
        phandle->ce_count_crc_ok++;
    }

    if (rd_value & BIT25)
    {
        /* at least one rx pkt is mic error */
        phandle->rx_pkt_mic_err = TRUE;
    }
    else
    {
        /* all rx pkts are mic ok */
        phandle->rx_pkt_mic_err = FALSE;
    }

    if (rd_value & BIT26)
    {
        /* all rx pkts are miss */
        phandle->rx_any_pkt_in_ce = FALSE;
        phandle->ce_count_no_any_pkt_received++;
    }
    else
    {
        /* at least one rx pkt is received */
        phandle->rx_any_pkt_in_ce = TRUE;
        phandle->ce_count_rx_any_pkt++;
        phandle->rx_any_in_conn_state = TRUE;
    }
#else
    LE_REG_S_SET reg_union;

    if (ll_manager.conn_unit.master)
    {
        dw_addr = LE_CAM_ENTRY_BASE(entry) + LE_CAM_ADDR_4;
        phandle->conn_counter = ll_driver_read_cam(dw_addr) >> 16;

        dw_addr = LE_CAM_ENTRY_BASE(entry) + LE_CAM_ADDR_14;
        rd_value = ll_driver_read_cam(dw_addr);

        if (rd_value & BIT24)
        {
            /* all rx pkts no crc OK */
            phandle->all_rx_pkt_no_crc_ok = TRUE;
            phandle->ce_count_no_crc_ok++;
        }
        else
        {
            /* at least one rx pkt is crc ok */
            phandle->all_rx_pkt_no_crc_ok = FALSE;
            phandle->ce_count_crc_ok++;
        }

        if (rd_value & BIT25)
        {
            /* at least one rx pkt is mic error */
            phandle->rx_pkt_mic_err = TRUE;
        }
        else
        {
            /* all rx pkts are mic ok */
            phandle->rx_pkt_mic_err = FALSE;
        }

        if (rd_value & BIT26)
        {
            /* all rx pkts are miss */
            phandle->rx_any_pkt_in_ce = FALSE;
            phandle->ce_count_no_any_pkt_received++;
        }
        else
        {
            /* at least one rx pkt is received */
            phandle->rx_any_pkt_in_ce = TRUE;
            phandle->ce_count_rx_any_pkt++;
            phandle->rx_any_in_conn_state = TRUE;
        }
    }
    else
    {
        phandle->conn_counter = RD_LE_REG(LE_REG_CONN_COUNTER);

        reg_union.value = RD_LE_REG(LE_REG_STATUS_RX_INT_STACK);

        if (reg_union.status_rx_int_stack.no_crc_ok)
        {
            /* all rx pkts no crc OK */
            phandle->all_rx_pkt_no_crc_ok = TRUE;
            phandle->ce_count_no_crc_ok++;
        }
        else
        {
            /* at least one rx pkt is crc ok */
            phandle->all_rx_pkt_no_crc_ok = FALSE;
            phandle->ce_count_crc_ok++;
        }

        if (reg_union.status_rx_int_stack.mic_err)
        {
            /* at least one rx pkt is mic error */
            phandle->rx_pkt_mic_err = TRUE;
        }
        else
        {
            /* all rx pkts are mic ok */
            phandle->rx_pkt_mic_err = FALSE;
        }

        if (reg_union.status_rx_int_stack.no_rx_pkt)
        {
            /* all rx pkts are miss */
            phandle->rx_any_pkt_in_ce = FALSE;
            phandle->ce_count_no_any_pkt_received++;
        }
        else
        {
            /* at least one rx pkt is received */
            phandle->rx_any_pkt_in_ce = TRUE;
            phandle->ce_count_rx_any_pkt++;
            phandle->rx_any_in_conn_state = TRUE;
        }
    }
#endif

}


/**************************************************************************
 * Function     : ll_cleanup_rx_status
 *
 * Description  : This function is used to handle rx path from the rx status
 *                and generate different rx tasks in the ISR of the end of
 *                advertising event or connection event
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
SECTION_LE_ISR void ll_cleanup_rx_status(void)
{
    FW_BT_RX_INT_UNIT_S *punit;
    LL_CONN_HANDLE_UNIT *phandle;
    UINT8 cur_entry;
    UINT16 rx_word_cnt;
    OS_SIGNAL sig_send;
    LL_TASK_PARA_U task_param;
#ifndef _DAPE_MOVE_LE_SLAVE_PARAMETER_TO_CAM
    LE_REG_S_SET reg_union;
#endif

#ifdef _NEW_BLE_HW_SPEC_FROM_150320_
    UINT16 rx_pkt_cnt;
#endif

    while (fw_bt_rx_int_fifo.wptr != fw_bt_rx_int_fifo.rptr)
    {
        punit = &fw_bt_rx_int_fifo.unit[fw_bt_rx_int_fifo.rptr];

        do
        {
            if (punit->type)
            {
                rx_word_cnt = (punit->ce_end_status.rx_word_cnt_b16 << 16) |
                              punit->ce_word_cnt_reg_value;

#ifdef _NEW_BLE_HW_SPEC_FROM_150320_
                /* total accumulated rx packet count for this event */
                rx_pkt_cnt = punit->anch_diff_rpt1.ce_end_rx_pkt_cnt;
#endif

                if (punit->ce_end_status.ae_end)
                {
                    ll_manager.event_manager.event_start = 0;
                    ll_manager.event_manager.event_end = 1;

#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8821_RCP_
                    if (rcp_ll_cleanup_rx_status_ae_end != NULL)
                    {
                        rcp_ll_cleanup_rx_status_ae_end();
                    }
#endif
#endif

                    /* le ae end */
                    if (rx_word_cnt < ll_manager.event_manager.cur_rx_word_cnt)
                    {
                        LL_LOG_TRACE(RED, LE_MSG_EVENT_END_RX_WORD_CNT_ERROR, 2,
                                     ll_manager.event_manager.cur_rx_word_cnt, rx_word_cnt);

                        ll_manager.event_manager.cur_rx_word_cnt = 0;
                        break;
                    }

                    rx_word_cnt -= ll_manager.event_manager.cur_rx_word_cnt;
                    ll_manager.event_manager.cur_rx_word_cnt = 0;

                    if (rx_word_cnt == 0)
                    {
                        break;
                    }

                    /* Send the signal to lc_rx_task to pick up the packet*/
                    task_param.Dword = 0;
                    task_param.rx_s.fifo_word_cnt = rx_word_cnt;
                    task_param.rx_s.sub_type = LL_HANDLE_RECD_ADV_PACKET_TYPE;
                    task_param.rx_s.int_src = 1;
                    sig_send.type = LL_HANDLE_RECD_PACKET_SIGNAL;
                    sig_send.param = (OS_ADDRESS)task_param.Dword;
                    OS_ISR_SEND_SIGNAL_TO_TASK (lc_rx_task_handle, sig_send);
                    break;
                } /* end of if (punit->ce_end_status.ae_end) */

                if (punit->ce_end_status.ce_end)
                {
                    /* le ce end */
                    cur_entry = punit->ce_end_status.entry;
                    phandle = &ll_manager.conn_unit.handle[cur_entry];

#ifdef _NEW_BLE_HW_SPEC_FROM_150320_
#ifdef _ENABLE_BLE_LL_FLOW_CONTROL_
                    if (rx_pkt_cnt >= phandle->ce_rx_packet_cnt)
                    {
                        rx_pkt_cnt = rx_pkt_cnt - phandle->ce_rx_packet_cnt;
                    }
                    else
                    {
                        rx_pkt_cnt = 0;
                    }
                    phandle->ce_rx_packet_cnt = 0;
#endif
#endif
                    if (phandle->connected == FALSE)
                    {
                        LL_LOG_TRACE(RED, LE_MSG_CONNECT_NO_ALIVE, 1,
                                     cur_entry);
                        phandle->ce_rx_word_cnt = 0;
                        break;
                    }

                    if (rx_word_cnt < phandle->ce_rx_word_cnt)
                    {
                        LL_LOG_TRACE(RED, LE_MSG_EVENT_END_RX_WORD_CNT_ERROR, 2,
                                     phandle->ce_rx_word_cnt, rx_word_cnt);
                        phandle->ce_rx_word_cnt = 0;
                        break;
                    }

                    rx_word_cnt -= phandle->ce_rx_word_cnt;
                    phandle->ce_rx_word_cnt = 0;

                    /* update some rx status */
                    ll_update_rx_statistics(cur_entry);

                    if (ll_manager.conn_unit.master)
                    {

                        if ((LLC_START_ENC_STATE_BEGIN ==
                                phandle->encrypt_blk.start_enc_state) &&
                                !phandle->install_key)
                        {
                            ll_driver_install_encryption_info(LL_MASTER,
                                  cur_entry, (UINT16*)phandle->encrypt_blk.iv_m,
                                  (UINT16*)phandle->encrypt_blk.sesskey);

                            phandle->install_key = 1;
                        }
                    }
                    else
                    {
                        if ((LLC_START_ENC_STATE_BEGIN ==
                            phandle->encrypt_blk.start_enc_state) &&
                            !phandle->install_key && phandle->long_term_key_got)
                        {
                            phandle->encrypt_blk.start_enc_state =
                                LLC_START_ENC_STATE_S2M_START_ENC_REQ;

                            ll_driver_install_encryption_info(LL_SLAVE,
                                  cur_entry, (UINT16*)phandle->encrypt_blk.iv_m,
                                  (UINT16*)phandle->encrypt_blk.sesskey);

                            phandle->install_key = 1;

                            /* send LL_START_ENC_REQ to remote device */
                            LL_CTRL_PDU_PAYLOAD *pTxPkt;
                            pTxPkt = llc_generate_ll_start_enc_req();
                            llc_append_pdu_to_tx_list(pTxPkt, cur_entry);
                        }
                    }

#if defined(_NEW_BLE_HW_SPEC_FROM_150320_) && defined(_LE_WAKE_FROM_SLAVE_LATENCY_)
                    /* if I am slave, notify hw to wakeup from slave latency */
                    if (ll_manager.conn_unit.en_wake_from_slave_latency &&
                                        !ll_manager.conn_unit.master &&
                                    (phandle->slave_latency != 0) &&
                                    (phandle->all_rx_pkt_no_crc_ok == FALSE))
                    {
                        if ((phandle->tx_pend_data_pkt_list.pktcnt > 0)
                                || (phandle->tx_pend_ctrl_pkt_list.pktcnt > 0)
#ifdef USE_NEW_LE_SCHEDULER
                                || (phandle->tx_pend_misc_pkt_list.pktcnt > 0)
#else
#ifdef _SCHEDULE_BLE_MISC_PACKET_
                                || (phandle->tx_pend_misc_pkt_list.pktcnt > 0)
                                || (phandle->tx_resent_misc_pkt != NULL)
#else
                                || (phandle->tx_resent_data_pkt != NULL)
#endif
#endif
                           )
                        {
                            ll_driver_wake_from_slave_latency();
                        }
                    }
#endif

                    /* check MIC error */
                    if (phandle->rx_pkt_mic_err)
                    {
                        phandle->kill_link_reason = LL_KILL_REASON_MIC_ERROR;

                        /* need to kill le connection */
                        ll_driver_kill_connection(cur_entry);
                    }

#ifdef _DAPE_TEST_NEW_HW_READ_RSSI_STACK
                    if (phandle->rx_any_pkt_in_ce)
                    {
                        phandle->last_rssi = punit->rssi;
                    }
#ifdef _YL_MODEM_RSSI_MAPPING
                    // TODO: need lc_fine_tune_rssi_offset(); ?? NO
#endif
#endif

#ifdef _LE_AUTO_REPORT_RSSI_AND_LOGIN_INOUT_FUNC_
                    do {
#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_
                        if (ll_rssi_manager.msft_monitor_rssi_mode ==
                                LE_MSFT_MONITOR_RSSI_VIA_CONN_HANDLE_MODE_NONE)
                        {
                            break;
                        }

                        if (ll_rssi_manager.msft_monitor_rssi_mode ==
                                LE_MSFT_MONITOR_RSSI_VIA_CONN_HANDLE_MODE_MSFT)
                        {
                            if (ll_rssi_manager.bm_used_handle & (1 << cur_entry))
                            {
                                UINT8 index = cur_entry + 16;
                                INT8 cur_rssi = lc_calculate_log_from_rssi(phandle->last_rssi);
                                rssi_app_msft_update_conn_rssi_info(index, cur_rssi);
                            }
                            break;
                        }
#endif
                        if ((ll_rssi_manager.max_indivadual_cnt > 0) &&
                            (ll_rssi_manager.bm_used_handle & (1 << cur_entry)))
                        {
                            UINT8 index = ll_rssi_manager.handle[cur_entry];
                            LL_CONN_HANDLE_RSSI_UNIT *pentry;

                            pentry = &ll_rssi_manager.entry[index];
                            pentry->rssi_count++;
                            pentry->rssi_sum += lc_calculate_log_from_rssi(phandle->last_rssi);

                            if (pentry->rssi_count == 128)
                            {
                                OS_SIGNAL signal;
                                signal.param = (OS_ADDRESS)(UINT32)(index | (cur_entry << 8) |
                                                                   (pentry->rssi_count << 16));
                                signal.ext_param = (OS_ADDRESS)(UINT32)(pentry->rssi_sum);
                                signal.type = CH_AS_TASK_LE_RSSI;
                                OS_ISR_SEND_SIGNAL_TO_TASK(hci_ch_as_task_handle, signal);

                                pentry->rssi_count = 0;
                                pentry->rssi_sum = 0;
                            }
                        }
                    }
                    while (0);
#endif

#ifdef _PAUSE_SCO_FOR_LE_CONN
                    if (phandle->pause_sco ==1)
                    {
                        if (!(phandle->ce_count_rx_any_pkt) || (phandle->conn_counter < 6)
#ifdef _DAPE_AUTO_CONN_UPDT_WHEN_SLOT_CONFLICT_AFTER_CONN_UPDT
                        || ((phandle->conn_counter - phandle->updated_conn_counter < 6)
                        &&(phandle->updated_conn_counter != 0))
#endif
                         )
                        {
                            phandle->conn_created = 1;
                        }
                        bb_pause_sco(FALSE);
                        bb_pause_esco(FALSE);
                        phandle->pause_sco = 0;
                        lc_sco_pause_status &= ~BIT7;
                    }
#endif

#ifdef _DAPE_NO_TRX_WHEN_LE_SEND_CONN_REQ_HW_TIMER
                    if (IS_TIMER3_FOR_LE_DUAL_WHCK)
                    {
                        if (phandle->conn_counter >= 8)
                        {
                            timer_on_off(TIMER3_ID, 0, 0);
                            bb_switch_scheduler_to_legacy();
                            RT_BT_LOG(BLUE, DAPE_TEST_LOG207, 1,BB_read_native_clock());
                        }
                    }
#endif

#ifdef _DAPE_TEST_NEW_HW_BLOCK_ACL_FOR_LE
                    if ((phandle->conn_counter >= 8) && (g_enable_le_block_legacy == 1))
                    {
                        ll_driver_block_legacy_slot_for_le(g_block_legacy_for_le_slot_num);
                        g_enable_le_block_legacy = 0;
                    }
#endif

                    /* this scope is used to do the process which is
                       need after LE connection update. */
                    if (phandle->conn_update_le_slot_updated == FALSE)
                    {
#ifndef _DAPE_AUTO_CONN_UPDT_WHEN_SLOT_CONFLICT_AFTER_CONN_UPDT
#ifdef _CCH_SLOT_OFFSET_
#ifdef _DAPE_TEST_SLOT_OFFSET_FOR_LE
                        ll_fw_compute_slot_offset_and_occupy(cur_entry,
                                                ll_manager.conn_unit.master);
#endif
#endif
                        ll_fw_get_slot_offset(cur_entry,
                                              ll_manager.conn_unit.master);
#else
                        if ((LE_AUTO_UPDT_WHEN_SLOT_CONFLICT_AFTER_CONN_UPDT) &&
                            (phandle->self_updt == 0))
                        {
                            ll_fw_get_update_slot(cur_entry, 1);
                        }
                        else
                        {
                            phandle->self_updt = 0;
                            ll_fw_get_update_slot(cur_entry, 2);

                            /*ll_fw_compute_slot_offset_and_occupy(cur_entry,
                                                ll_manager.conn_unit.master);
                            ll_fw_get_slot_offset(cur_entry,
                                              ll_manager.conn_unit.master);*/
                        }
                        phandle->updated_conn_counter = phandle->conn_counter;
#endif
                        phandle ->conn_update_le_slot_updated = TRUE;
                    }
                    else
                    {
                        if ((phandle->le_slot_updated == FALSE) &&
                            (phandle->conn_counter >= 2) &&
                            /* If
                            (ll_manager.conn_unit.conn_updt_entry == cur_entry)
                            this means that this entry is doing connection update and
                            we should not do connection update now.
                            So we should to connection update while
                            this entry is not doing connection update. */
                            (ll_manager.conn_unit.conn_updt_entry != cur_entry)
                            )
                        {
                            ll_fw_get_update_slot(cur_entry, 0);
                            phandle->le_slot_updated = TRUE;
                        }
                    }

#ifdef _ROM_CODE_PATCHED_
#ifdef _DAPE_8723_RCP_
                   if (rcp_ll_update_slot_in_ce_end_func != NULL)
                   {
                       rcp_ll_update_slot_in_ce_end_func((void*)(&cur_entry));
                   }
#endif
#endif

#ifdef _CCH_8821B_TEST_LE_CONN_LPS
                   rcp_ll_update_slot_in_ce_end_func_imp(cur_entry);
#endif


                    if ((phandle->sup_timer != NULL) &&
                            (!phandle->all_rx_pkt_no_crc_ok))
                    {
                        /* restart the supervision timer */
                        ll_start_timer(cur_entry, LL_TIMER_TYPE_SUPERVISION);
                        phandle->sup_timeout_restart = TRUE;
                    }

                    if (rx_word_cnt > 0)
                    {
                        /* Send the signal to lc_rx_task to pick up the packet */
                        task_param.Dword = 0;
                        task_param.rx_s.fifo_word_cnt = rx_word_cnt;
                        task_param.rx_s.sub_type = LL_HANDLE_RECD_DATA_PACKET_TYPE;
                        task_param.rx_s.int_src = 1;
                        task_param.rx_s.conn_entry = cur_entry;
                        sig_send.type = LL_HANDLE_RECD_PACKET_SIGNAL;
                        sig_send.param = (OS_ADDRESS)task_param.Dword;
                        OS_ISR_SEND_SIGNAL_TO_TASK(lc_rx_task_handle, sig_send);

#ifdef _NEW_BLE_HW_SPEC_FROM_150320_
#ifdef _ENABLE_BLE_LL_FLOW_CONTROL_
                        os_occupy_le_reserve_buffers_from_isr(rx_pkt_cnt);
#endif
#endif
                    }

#ifdef _NEW_BLE_HW_SPEC_FROM_150320_
#ifdef _ENABLE_BLE_LL_FLOW_CONTROL_
                    if (os_read_occupied_le_reserved_buffers() >=
                         (LL_POLL_HCI_MAX_RX_ACL_PKT_CNT - g_le_flow_stop_threshold_pkt_cnt))
                    {
                        if (!g_le_enter_flow_stop_flag)
                        {
                            ll_driver_set_le_flow_stop(TRUE);
                            g_le_enter_flow_stop_flag = TRUE;

                            LL_LOG_TRACE(YELLOW, MSG_BLE_HW_FLOW_STOP, 3,
                                            os_read_occupied_le_reserved_buffers(),
                                            rx_pkt_cnt, cur_entry);

                        }
                    }
                    else
                    {
                        if (g_le_enter_flow_stop_flag)
                        {
                            ll_driver_set_le_flow_stop(FALSE);
                            g_le_enter_flow_stop_flag = FALSE;

                            LL_LOG_TRACE(YELLOW, MSG_BLE_HW_FLOW_GO, 3,
                                            os_read_occupied_le_reserved_buffers(),
                                            rx_pkt_cnt, cur_entry);
                        }
                    }
#endif
#endif
                    break;
                } /* end of if (punit->ce_end_status.ce_end) */

                /* Here is an exception case */
                /* Send the signal to lc_rx_task to pick up the packet*/
                if (rx_word_cnt > 0)
                {
                    task_param.Dword = 0;
                    task_param.rx_s.fifo_word_cnt = rx_word_cnt;
                    task_param.rx_s.sub_type = LL_HANDLE_RECD_EXCEPTION_TYPE;
                    task_param.rx_s.int_src = 1;
                    sig_send.type = LL_HANDLE_RECD_PACKET_SIGNAL;
                    sig_send.param = (OS_ADDRESS)task_param.Dword;
                    OS_ISR_SEND_SIGNAL_TO_TASK ( lc_rx_task_handle, sig_send );
                }
            }
            else
            {
                /* legacy pcd_rx done */
                BT_RX_INT_ARGUMENT rx_argu;
                rx_argu.payload_hdr = punit->rx_payload_reg_value;
                rx_argu.packet_hdr = punit->rx_status_reg_value;
#ifdef _DAPE_TEST_NEW_HW_READ_RSSI_STACK
                rx_argu.rssi = punit->rssi;
#endif
                BB_handle_full_rx_interrupt(&rx_argu);
            }
        }
        while (0);

        fw_bt_rx_int_fifo.rptr++;
        fw_bt_rx_int_fifo.rptr &= (MAX_FW_BT_RX_INT_FIFO_UNITS - 1);
        fw_bt_rx_int_fifo.count--;
    }
}

/**************************************************************************
 * Function     : ll_handle_tx_ack_threshold_interrupt
 *
 * Description  : This function is used to handle tx ack threshold interrupt.
 *                The threshold is depend on word length via register settings.
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
#ifdef USE_NEW_LE_SCHEDULER
SECTION_LE_ISR void ll_handle_tx_ack_threshold_interrupt(UINT8 conn_entry)
{
    LL_CONN_HANDLE_UNIT *phandle;
    LE_REG_S_SET reg_union;
    UINT8 acked_cnt;

    reg_union.value = RD_LE_REG(LE_REG_STATUS_TX_THR_STATUS);
    acked_cnt = reg_union.status_tx_thres.acked_tx_pkt_cnt;

    if (acked_cnt == 0)
    {
        /* No receive any acknowledgaement for current tx scheduled packets */
        return;
    }

    if (ll_manager.event_manager.event_end)
    {
        /* do not handle in ce event end */
        return;
    }

#ifdef _LE_NO_TRX_THRESHOLD_INT_FOR_MULTI_CE_OPTION_
    if (IS_NO_LE_TX_THRESHOLD_FOR_MULTI_CE)
    {
        LE_REG_S_SET reg_union2;
        reg_union2.value = RD_LE_REG(LE_REG_CE_TX_STATUS);

        /* do not handle tx ack threshold interrupt for multiple ce */

        if ((reg_union2.ce_tx_status.data_tx) &&
            (ll_manager.conn_unit.connection_cnts > 1))
        {
            return;
        }
    }
#endif

    phandle = &ll_manager.conn_unit.handle[conn_entry];
    phandle->ce_acked_tx_pkt_cnt += acked_cnt;

    ll_handle_tx_ack_pkts(conn_entry, acked_cnt);

    /* free LLC pdu packet */
    if (ll_manager.conn_unit.llc_free_pkt_list.pktcnt > 0)
    {
#ifdef PATCH_FIX_UNACKED_PACKET_NOT_SCHEDULE_IN_BZDMA_DBG_LOG
        patch_llc_free_pdu_in_free_list();
#else
        llc_free_pdu_in_free_list();
#endif
    }

#if defined(_SCHEDULE_BLE_MISC_PACKET_) && defined(_NEW_BZDMA_FROM_V8_)
    /* free LE ACL data misc tx packet */
    if (phandle->h2l_free_misc_pkt_list.pktcnt > 0)
    {
        ll_free_acl_tx_misc_pkts_in_free_list(conn_entry);
    }
#endif

#ifdef _NEW_BZDMA_FROM_V8_
    if (acked_cnt > 0)
    {
        bzdma_update_fw_rptr_of_ble_data_ring_fifo(conn_entry, acked_cnt, FALSE);
    }
#endif
}
#else /* ! USE_NEW_LE_SCHEDULER */
SECTION_LE_ISR void ll_handle_tx_ack_threshold_interrupt(UINT8 conn_entry)
{
    LL_CONN_HANDLE_UNIT *phandle;
    LE_ACL_PKT_LIST_MANAGE *pctrl_list;
    LE_ACL_PKT_LIST_MANAGE ack_list;
    LE_REG_S_SET reg_union;
    UINT8 acked_cnt;
    UINT8 i;    
#ifndef _SCHEDULE_BLE_MISC_PACKET_
    LE_ACL_PKT_LIST_MANAGE *pdata_list;
    LL_HCI_ACL_DATA_PKT *resent_pkt;
    LE_ACL_PKT_LIST_MANAGE resend_list;
#endif
#ifdef _NEW_BZDMA_FROM_V8_
    UINT8 acked_cnt_tmp;
#endif

    reg_union.value = RD_LE_REG(LE_REG_STATUS_TX_THR_STATUS);

    acked_cnt = reg_union.status_tx_thres.acked_tx_pkt_cnt;

    if (acked_cnt == 0)
    {
        /* No receive any acknowledgaement for current tx scheduled packets */
        return;
    }

#ifdef _NEW_BZDMA_FROM_V8_
    acked_cnt_tmp = acked_cnt;
#endif

    if (ll_manager.event_manager.event_end)
    {
        /* do not handle in ce event end */
        return;
    }

#ifdef _LE_NO_TRX_THRESHOLD_INT_FOR_MULTI_CE_OPTION_
    if (IS_NO_LE_TX_THRESHOLD_FOR_MULTI_CE)
    {
        LE_REG_S_SET reg_union2;
        reg_union2.value = RD_LE_REG(LE_REG_CE_TX_STATUS);

        /* do not handle tx ack threshold interrupt for multiple ce */

        if ((reg_union2.ce_tx_status.data_tx) &&
            (ll_manager.conn_unit.connection_cnts > 1))
        {
            return;
        }
    }
#endif

    phandle = &ll_manager.conn_unit.handle[conn_entry];
    phandle->ce_acked_tx_pkt_cnt += acked_cnt;
    pctrl_list = &phandle->tx_sched_ctrl_pkt_list;
#ifndef _SCHEDULE_BLE_MISC_PACKET_
    pdata_list = &phandle->tx_sched_data_pkt_list;
    resent_pkt = phandle->tx_resent_data_pkt;    
#endif

#if defined(_SCHEDULE_BLE_MISC_PACKET_) && defined(_NEW_BZDMA_FROM_V8_)
    LE_ACL_MISC_PKT_LIST_MANAGE *pdata_misc_list;
    LL_MISC_ACL_DATA_PKT_NODE *resent_misc_pkt;
    LE_ACL_MISC_PKT_LIST_MANAGE acl_misc_list;

    pdata_misc_list = &phandle->tx_sched_misc_pkt_list;
    resent_misc_pkt = phandle->tx_resent_misc_pkt;

    if ((resent_misc_pkt != NULL) && (acked_cnt > 0))
    {
        /* append unacked tx packet list to free list */
        resent_misc_pkt->NextNodeId = LL_MISC_ACL_DATA_PKT_MAX_NODES;
        acl_misc_list.MiscHeadId = resent_misc_pkt->MyNodeId;
        acl_misc_list.MiscTailId = resent_misc_pkt->MyNodeId;
        acl_misc_list.pktcnt = 1;

        /* all scheduled acl pkts receive ACK */
        ll_append_acl_tx_misc_pkt_list_to_list(
                    &acl_misc_list, conn_entry,
                    LL_LINKED_LIST_INSERT_POLICY_TAIL,
                    LL_LINKED_LIST_TYPE_TX_FREE);

        phandle->tx_resent_misc_pkt = NULL;

        acked_cnt--;
    }
#endif

#ifndef _SCHEDULE_BLE_MISC_PACKET_
    if ((resent_pkt != NULL) && (acked_cnt > 0))
    {
        /* append unacked tx packet list to free list */
        resent_pkt->next = NULL;
        resend_list.pDataHead = resent_pkt;
        resend_list.pDataTail = resent_pkt;
        resend_list.pktcnt = 1;

        /* all scheduled acl pkts receive ACK */
        ll_append_acl_tx_pkt_list_to_list(
            &resend_list, conn_entry,
            LL_LINKED_LIST_INSERT_POLICY_TAIL,
            LL_LINKED_LIST_TYPE_TX_FREE);


        phandle->tx_resent_data_pkt = NULL;

        acked_cnt--;
    }
#endif

    /* check the status of scheduled LLC ctrl pdu */
    if ((pctrl_list->pktcnt > 0)  && (acked_cnt > 0))
    {
        LL_CTRL_PDU_PAYLOAD *pNode;

        if (acked_cnt >= pctrl_list->pktcnt)
        {
            /* check if we need to wait any ack in sent pdu to
               stop llc procedure */
            if (phandle->llc.wait_ack)
            {
                pNode = pctrl_list->pCtrlHead;

                while (pNode != NULL)
                {
                    if (phandle->llc.wait_sent_pdu_type == pNode->OpCode)
                    {
                        llc_handle_sent_llc_pdu_ack_recd(phandle,
                                                            pNode->OpCode);
                        break;
                    }
                    pNode = pNode->pNext;
                }
            }

            ll_handle_ctrl_pdu_tx_ack_list(phandle, pctrl_list);
            /* all scheduled llc pkts receive ACK */
            llc_append_pdu_list_to_free_list(pctrl_list);
            acked_cnt -= pctrl_list->pktcnt;

            /* init tx_sched_ctrl_pkt_list */
            pctrl_list->pCtrlHead = NULL;
            pctrl_list->pCtrlTail = NULL;
            pctrl_list->pktcnt = 0;
        }
        else
        {
            /* partial scheduled llc pkts receive ACK */
            LL_CTRL_PDU_PAYLOAD *pllc_pkt_tmp;
            LL_CTRL_PDU_PAYLOAD *pllc_pkt_tmp_pre;

            /* separate acked and unacked packet list */
            pllc_pkt_tmp = pctrl_list->pCtrlHead;
            pllc_pkt_tmp_pre = pctrl_list->pCtrlHead;
            for (i = 0; i < acked_cnt; i++)
            {
                pllc_pkt_tmp_pre = pllc_pkt_tmp;
                pllc_pkt_tmp = pllc_pkt_tmp->pNext;

                /* check if we need to wait any ack in sent pdu to
                   stop llc procedure */
                if (phandle->llc.wait_ack)
                {
                    if (phandle->llc.wait_sent_pdu_type ==
                            pllc_pkt_tmp_pre->OpCode)
                    {
                        llc_handle_sent_llc_pdu_ack_recd(phandle,
                                                    pllc_pkt_tmp_pre->OpCode);
                    }
                }
            }
            pllc_pkt_tmp_pre->pNext = NULL;

            /* append acked tx packet list to free list */
            ack_list.pCtrlHead = pctrl_list->pCtrlHead;
            ack_list.pCtrlTail = pllc_pkt_tmp_pre;
            ack_list.pktcnt = acked_cnt;
            ll_handle_ctrl_pdu_tx_ack_list(phandle, &ack_list);
            llc_append_pdu_list_to_free_list(&ack_list);

            /* update tx_sched_ctrl_pkt_list */
            pctrl_list->pCtrlHead = pllc_pkt_tmp;
            pctrl_list->pktcnt -= acked_cnt;

            acked_cnt = 0;
        }

        /* free LLC pdu packet */
        if (ll_manager.conn_unit.llc_free_pkt_list.pktcnt > 0)
        {
            llc_free_pdu_in_free_list();
        }
    } /* end of if (pctrl_list->pktcnt > 0) */

#if defined(_SCHEDULE_BLE_MISC_PACKET_) && defined(_NEW_BZDMA_FROM_V8_)
    /* check the status of scheduled ACL tx data pkts */
    if ((pdata_misc_list->pktcnt > 0) && (acked_cnt > 0))
    {
        if (acked_cnt >= pdata_misc_list->pktcnt)
        {
            /* all scheduled acl pkts receive ACK */
            ll_append_acl_tx_misc_pkt_list_to_list(
                        pdata_misc_list, conn_entry,
                        LL_LINKED_LIST_INSERT_POLICY_TAIL,
                        LL_LINKED_LIST_TYPE_TX_FREE);

            acked_cnt -= pdata_misc_list->pktcnt;

            /* init tx_sched_data_pkt_list */
            pdata_misc_list->DWord = LL_MISC_ACL_PKT_LIST_RESET_VALUE;
        }
        else
        {
            /* partial scheduled llc pkts receive ACK */
            UINT8 acl_node_tmp;
            UINT8 acl_node_tmp_pre;

            /* separate acked and unacked packet list */
            acl_node_tmp = pdata_misc_list->MiscHeadId;
            acl_node_tmp_pre = pdata_misc_list->MiscHeadId;

            for (i = 0; i < acked_cnt; i++)
            {
                acl_node_tmp_pre = acl_node_tmp;
                acl_node_tmp = le_tx_misc_node[acl_node_tmp].NextNodeId;
            }
            le_tx_misc_node[acl_node_tmp_pre].NextNodeId =
                                                LL_MISC_ACL_DATA_PKT_MAX_NODES;

            /* append acked tx packet list to free list */
            acl_misc_list.MiscHeadId = pdata_misc_list->MiscHeadId;
            acl_misc_list.MiscTailId = acl_node_tmp_pre;
            acl_misc_list.pktcnt = acked_cnt;
            ll_append_acl_tx_misc_pkt_list_to_list(
                                &acl_misc_list, conn_entry,
                                LL_LINKED_LIST_INSERT_POLICY_TAIL,
                                LL_LINKED_LIST_TYPE_TX_FREE);

            /* update tx_sched_data_pkt_list */
            pdata_misc_list->MiscHeadId = acl_node_tmp;
            pdata_misc_list->pktcnt -= acked_cnt;

            acked_cnt = 0;
        }
    } /* end of if (pdata_list->pktcnt > 0) */
#endif

#ifndef _SCHEDULE_BLE_MISC_PACKET_
    /* check the status of scheduled ACL tx data pkts */
    if ((pdata_list->pktcnt > 0) && (acked_cnt > 0))
    {
        if (acked_cnt >= pdata_list->pktcnt)
        {
            /* all scheduled acl pkts receive ACK */
            ll_append_acl_tx_pkt_list_to_list(
                pdata_list, conn_entry,
                LL_LINKED_LIST_INSERT_POLICY_TAIL,
                LL_LINKED_LIST_TYPE_TX_FREE);

            acked_cnt -= pdata_list->pktcnt;

            /* init tx_sched_data_pkt_list */
            pdata_list->pDataHead = NULL;
            pdata_list->pDataTail = NULL;
            pdata_list->pktcnt = 0;
        }
        else
        {
            /* partial scheduled llc pkts receive ACK */
            LL_HCI_ACL_DATA_PKT *pacl_pkt_tmp;
            LL_HCI_ACL_DATA_PKT *pacl_pkt_tmp_pre;

            /* separate acked and unacked packet list */
            pacl_pkt_tmp = pdata_list->pDataHead;
            pacl_pkt_tmp_pre = pdata_list->pDataHead;

            for (i = 0; i < acked_cnt; i++)
            {
                pacl_pkt_tmp_pre = pacl_pkt_tmp;
                pacl_pkt_tmp = pacl_pkt_tmp->next;
            }
            pacl_pkt_tmp_pre->next = NULL;

            /* append acked tx packet list to free list */
            ack_list.pDataHead = pdata_list->pDataHead;
            ack_list.pDataTail = pacl_pkt_tmp_pre;
            ack_list.pktcnt = acked_cnt;
            ll_append_acl_tx_pkt_list_to_list(
                &ack_list, conn_entry,
                LL_LINKED_LIST_INSERT_POLICY_TAIL,
                LL_LINKED_LIST_TYPE_TX_FREE);

            /* update tx_sched_data_pkt_list */
            pdata_list->pDataHead = pacl_pkt_tmp;
            pdata_list->pktcnt -= acked_cnt;
        }
    } /* end of if (pdata_list->pktcnt > 0) */
#endif

#if defined(_SCHEDULE_BLE_MISC_PACKET_) && defined(_NEW_BZDMA_FROM_V8_)
    /* free LE ACL data misc tx packet */
    if ((phandle->h2l_free_misc_pkt_list.pktcnt > 0))
    {
        ll_free_acl_tx_misc_pkts_in_free_list(conn_entry);
    }
#endif

#ifndef _SCHEDULE_BLE_MISC_PACKET_
    /* free LE ACL data tx packet */
    if (phandle->h2l_free_pkt_list.pktcnt > 0)
    {
        ll_free_acl_tx_pkts_in_free_list(conn_entry);
    }
#endif

#ifdef _NEW_BZDMA_FROM_V8_
    if (acked_cnt_tmp > 0)
    {
        bzdma_update_fw_rptr_of_ble_data_ring_fifo(conn_entry,
                                                   acked_cnt_tmp,
                                                   FALSE);
    }
#endif
}
#endif /* USE_NEW_LE_SCHEDULER */

SECTION_LE_ISR void ll_handle_rx_test_mode_in_isr(UINT16 word_cnt, UINT16 crc_err)
{
    if (!crc_err)
    {
        ll_manager.test_unit.num_of_rx_pkts++;
    }
    else
    {
        ll_manager.test_unit.num_of_rx_pkts_crc_err++;
    }

    lc_flush_rx_fifo();
    return;
}

SECTION_LE_ISR void ll_handle_rx_threshold_interrupt(UINT8 conn_entry)
{
    LE_REG_S_SET reg_union;
    LE_REG_S_SET reg_union2;
    UINT16 fifo_word_cnt;
    UINT8 crc_err;
    OS_SIGNAL sig_send;
    LL_TASK_PARA_U task_param;
#ifdef _NEW_BLE_HW_SPEC_FROM_150320_
    UINT16 rx_pkt_cnt;
#endif

    reg_union.value = RD_LE_REG(LE_REG_STATUS_RX_THRES_STATUS);
    reg_union2.value = RD_LE_REG(LE_REG_CE_RX_STATUS);

    fifo_word_cnt = reg_union.status_rx_thres_status.fifo_word_cnt;
    crc_err = reg_union2.ce_rx_status.crc_err;

#ifdef _NEW_BLE_HW_SPEC_FROM_150320_
    LE_REG_S_SET rx_thres_ctrl = {
            .value = RD_LE_REG(LE_REG_STATUS_RX_THRES_CTRL)
    };
    rx_pkt_cnt = ((reg_union.status_rx_thres_status.pkt_cnt << 7)
            | rx_thres_ctrl.status_rx_thres_ctrl.pkt_cnt);
#endif

#ifdef _YL_TEST_MODEM_RX_REPORT
    {
        UINT32 rx_report_native_clock = BB_read_native_clock();
        UINT16 rx_report_array[16];
        if ( ((rx_report_native_clock-g_rx_report_last_native_clock)>=1600)
             || (rx_report_native_clock<g_rx_report_last_native_clock))
        {
            g_rx_report_last_native_clock = rx_report_native_clock;
  #ifdef _YL_TEST_MODEM_RX_REPORT_BT_GPIO
            SET_BT_GPIO_OUTPUT_HIGH(0);
  #endif
            rx_report_array[10] = RTK_READ_MODEM_REG_PI(2, TRANS_MODEM_REG(0x7e));
            rx_report_array[0] = RTK_READ_MODEM_REG_PI(0, TRANS_MODEM_REG(0x68));
            rx_report_array[1] = RTK_READ_MODEM_REG_PI(0, TRANS_MODEM_REG(0x6a));
            rx_report_array[2] = RTK_READ_MODEM_REG_PI(0, TRANS_MODEM_REG(0x6c));
            rx_report_array[3] = RTK_READ_MODEM_REG_PI(0, TRANS_MODEM_REG(0x6e));
            rx_report_array[4] = RTK_READ_MODEM_REG_PI(0, TRANS_MODEM_REG(0x70));
            rx_report_array[5] = RTK_READ_MODEM_REG_PI(2, TRANS_MODEM_REG(0x64));
            rx_report_array[6] = RTK_READ_MODEM_REG_PI(2, TRANS_MODEM_REG(0x76));
            rx_report_array[7] = RTK_READ_MODEM_REG_PI(2, TRANS_MODEM_REG(0x78));
            rx_report_array[8] = RTK_READ_MODEM_REG_PI(2, TRANS_MODEM_REG(0x7a));
            rx_report_array[9] = RTK_READ_MODEM_REG_PI(2, TRANS_MODEM_REG(0x7c));
            rx_report_array[11] = RTK_READ_MODEM_REG_PI(0, TRANS_MODEM_REG(0x7e));
  #ifdef _YL_TEST_MODEM_RX_REPORT_BT_GPIO
            SET_BT_GPIO_OUTPUT_LOW(0);
  #endif
            RT_BT_LOG(YELLOW, MDM_LOG_024, 16, reg_union.value, reg_union2.value, fifo_word_cnt, crc_err,
                        rx_report_array[0],                        rx_report_array[1],
                        rx_report_array[2],                        rx_report_array[3],
                        rx_report_array[4],                        rx_report_array[5],
                        rx_report_array[6],                        rx_report_array[7],
                        rx_report_array[8],                        rx_report_array[9],
                        rx_report_array[10],                        rx_report_array[11]);

        }
    }
#endif

    if (fifo_word_cnt == 0)
    {
        /* No receive any pkts in the rxfifo */
        return;
    }

    if (ll_manager.cur_mode == LL_MODE_RX_TEST)
    {
        ll_handle_rx_test_mode_in_isr(fifo_word_cnt, crc_err);
#ifdef _YL_NEW_MODEM_SRAM_DEBUG
        {
            if (g_modem_sram_debug_en && g_modem_sram_debug_le_trig_en)
            {
                UINT8 is_le_crc_trigered = (g_modem_sram_debug_le_trig_crc_ok ? (crc_err==0) : (crc_err!=0));
    RT_BT_LOG(WHITE, DAPE_TEST_LOG525, 6,0x8888, is_le_crc_trigered, 
        g_modem_sram_debug_le_trig_crc_ok, crc_err, ll_manager.test_unit.num_of_rx_pkts,
       ll_manager.test_unit.num_of_rx_pkts_crc_err );
                if (is_le_crc_trigered && (((ll_manager.test_unit.num_of_rx_pkts!=0) && (crc_err==0))
                    || ((ll_manager.test_unit.num_of_rx_pkts_crc_err != 0) && (crc_err)))
                   )
                {
                    rtl8821_btrf_modem_sram_debug_set_en(0);
                    g_modem_sram_debug_captured_flag = 1;
                }
            }
        }
#endif
        return;
    }

    if (reg_union2.ce_rx_status.adv_rx)
    {
        if (g_le_event_end_flag_in_this_isr)
        {
            /* do not handle when multi-flag are pending
               (rx threshold + event end) */
            return;
        }

        ll_manager.event_manager.event_type = LL_CHANNEL_TYPE_ADVERTISING;
    }

    if (reg_union2.ce_rx_status.data_rx)
    {
        if (ll_manager.event_manager.event_end)
        {
            /* do not handle after ce event end */
            return;
        }

        ll_manager.event_manager.event_type = LL_CHANNEL_TYPE_DATA;

#ifdef _LE_NO_TRX_THRESHOLD_INT_FOR_MULTI_CE_OPTION_
        if (IS_NO_LE_RX_THRESHOLD_FOR_MULTI_CE &&
            (ll_manager.conn_unit.connection_cnts > 1))
        {
            /* do not handle tx ack threshold interrupt for multiple ce */
            return;
        }
#endif

#ifdef _DAPE_LE_CE_ENTRY_REPORT_IN_RX_TH_AFTER_2801
        LE_REG_S_LE_EXT_MISR ll_reg;
        *(UINT16*)&ll_reg = RD_LE_REG(LE_REG_EXT_MISR);
        if (ll_reg.rxth_entry_report != ll_manager.event_manager.entry)
        {
            RT_BT_LOG(YELLOW, DAPE_TEST_LOG549, 2,
                      ll_reg.rxth_entry_report,
                      ll_manager.event_manager.entry);
            return;
        }
#endif
    }

    /* parse SRAM payload content after move data from acl rxfifo to SRAM */
    if (ll_manager.event_manager.event_type == LL_CHANNEL_TYPE_ADVERTISING)
    {
        ll_manager.event_manager.cur_rx_word_cnt += fifo_word_cnt;

        /* Send the signal to lc_rx_task to pick up the packet*/
        task_param.Dword = 0;
        task_param.rx_s.fifo_word_cnt = fifo_word_cnt;
        task_param.rx_s.sub_type = LL_HANDLE_RECD_ADV_PACKET_TYPE;
        task_param.rx_s.int_src = 0;

#ifdef _NEW_BLE_HW_SPEC_FROM_150320_
        ll_manager.event_manager.cur_rx_packet_cnt = rx_pkt_cnt;
#endif
    }
    else
    {
        ll_manager.conn_unit.handle[conn_entry].ce_rx_word_cnt += fifo_word_cnt;

        /* Send the signal to lc_rx_task to pick up the packet*/
        task_param.Dword  = 0;
        task_param.rx_s.fifo_word_cnt = fifo_word_cnt;
        task_param.rx_s.sub_type = LL_HANDLE_RECD_DATA_PACKET_TYPE;
        task_param.rx_s.conn_entry = conn_entry;
        task_param.rx_s.int_src = 0;

#ifdef _NEW_BLE_HW_SPEC_FROM_150320_
#ifdef _ENABLE_BLE_LL_FLOW_CONTROL_
        if (rx_pkt_cnt > ll_manager.conn_unit.handle[conn_entry].ce_rx_packet_cnt)
        {
            UINT8 cur_rx_pkt_cnt = rx_pkt_cnt - ll_manager.conn_unit.handle[conn_entry].ce_rx_packet_cnt;
            os_occupy_le_reserve_buffers_from_isr(cur_rx_pkt_cnt);
            if (os_read_occupied_le_reserved_buffers() >=
                    (LL_POLL_HCI_MAX_RX_ACL_PKT_CNT - g_le_flow_stop_threshold_pkt_cnt))
            {
                if (!g_le_enter_flow_stop_flag)
                {
                    ll_driver_set_le_flow_stop(TRUE);
                    g_le_enter_flow_stop_flag = TRUE;

                    LL_LOG_TRACE(YELLOW, MSG_BLE_HW_FLOW_STOP, 3,
                            os_read_occupied_le_reserved_buffers(),
                            cur_rx_pkt_cnt, conn_entry);

                }
            }
        }
#endif

        ll_manager.conn_unit.handle[conn_entry].ce_rx_packet_cnt = rx_pkt_cnt;
#endif
    }

    sig_send.type = LL_HANDLE_RECD_PACKET_SIGNAL;
    sig_send.param = (OS_ADDRESS)task_param.Dword;
    OS_ISR_SEND_SIGNAL_TO_TASK ( lc_rx_task_handle, sig_send );
}

#ifndef USE_NEW_LE_SCHEDULER
SECTION_LE_ISR void ll_handle_miss_previous_ce(UINT8 entry_id)
{
    LL_EVENT_MANAGE *event_manager;
    LL_CONN_HANDLE_UNIT *handle;

    event_manager = &ll_manager.event_manager;
    event_manager->event_start = 0;
    event_manager->event_end = 1;

    /* do resource free handling for previous entry */
    handle = &ll_manager.conn_unit.handle[entry_id];

    if (!handle->connected)
    {
        /* avoid this handle is disconnected */
        return;
    }

    handle->ce_acked_tx_pkt_cnt = 0;
    handle->tx_sched_pkts_one_ce = 0;

    if (handle->tx_sched_ctrl_pkt_list.pktcnt > 0)
    {
        /* all scheduled llc pkts receive NAK,and we need
        to retransmit these pkts */
        llc_append_pdu_list_to_list(&handle->tx_sched_ctrl_pkt_list,
                                    entry_id,
                                    LL_LINKED_LIST_INSERT_POLICY_HEAD,
                                    LL_LINKED_LIST_TYPE_TX_PEND);

        handle->tx_sched_ctrl_pkt_list.pCtrlHead = NULL;
        handle->tx_sched_ctrl_pkt_list.pCtrlTail = NULL;
        handle->tx_sched_ctrl_pkt_list.pktcnt = 0;
    }

#ifndef _SCHEDULE_BLE_MISC_PACKET_
    if (handle->tx_sched_data_pkt_list.pktcnt > 0)
    {
        /* all scheduled le acl pkts receive NAK,and we need
           to retransmit these pkts */
        ll_append_acl_tx_pkt_list_to_list(&handle->tx_sched_data_pkt_list,
                                          entry_id,
                                          LL_LINKED_LIST_INSERT_POLICY_HEAD,
                                          LL_LINKED_LIST_TYPE_TX_PEND);

        handle->tx_sched_data_pkt_list.pCtrlHead = NULL;
        handle->tx_sched_data_pkt_list.pCtrlTail = NULL;
        handle->tx_sched_data_pkt_list.pktcnt = 0;
    }
#else
    if (handle->tx_sched_misc_pkt_list.pktcnt > 0)
    {
        /* all scheduled le acl pkts receive NAK,and we need
           to retransmit these pkts */
        ll_append_acl_tx_misc_pkt_list_to_list(&handle->tx_sched_misc_pkt_list,
                                          entry_id,
                                          LL_LINKED_LIST_INSERT_POLICY_HEAD,
                                          LL_LINKED_LIST_TYPE_TX_PEND);
        handle->tx_sched_misc_pkt_list.DWord = LL_MISC_ACL_PKT_LIST_RESET_VALUE;
    }
#endif
}
#endif /* ! USE_NEW_LE_SCHEDULER */

/**************************************************************************
 * Function     : ll_handle_event_begin_early_interrupt
 *
 * Description  : This function is used to handle ISR of early interrupt of
 *                the start of advertising event or connection event (the early
 *                offset can be adjusted by register)
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
SECTION_LE_ISR void ll_handle_event_begin_early_interrupt(void)
{
#ifdef _DAPE_TEST_CHK_LE_EARLY_INTR
reset_vendor_counter();
#endif
    LE_REG_S_SET reg_union;
    LL_EVENT_MANAGE *event_manager;
    LL_CONN_HANDLE_UNIT *handle;
    UINT8 entry_id;

    event_manager = &ll_manager.event_manager;

    /* read LE_REG_STATUS_CE_BEGIN_STATUS to clear interrupt */
    reg_union.value = RD_LE_REG(LE_REG_STATUS_CE_BEGIN_STATUS);

    if (reg_union.status_ce_begin_status.ce_begin)
    {
        entry_id = reg_union.status_ce_begin_status.entry;

#ifndef _NEW_BZDMA_FROM_V8_
        UINT32 reg_value;
        UINT8 tx_entry_type = BZDMA_TX_ENTRY_TYPE_LE_DATA_BURST0;
        UINT8 chk_bzdma = TRUE;

        do
        {
            if (!IS_TOGGLE_LL_DATA_CMD)
            {
                break;
            }

            if (ll_manager.event_manager.used_txcmd_cnt == 2)
            {
                UINT8 pre_tx_entry_type;
                pre_tx_entry_type = BZDMA_TX_ENTRY_TYPE_LE_DATA_BURST0 +
                                 !ll_manager.event_manager.cur_txcmd_id;

                /* check bzdma for le burst is busy or not ? */

                reg_value = RD_U32_BZDMA_REG(BZDMA_REG_TX_CMD(pre_tx_entry_type));
                if (reg_value & BIT31)
                {
                    bzdma_invalid_txcmd(pre_tx_entry_type, 0, 0);
                    ll_handle_miss_previous_ce(event_manager->pre_tx_entry);
                }
                else
                {
                    bzdma_release_tx_entry(pre_tx_entry_type);
                }
                ll_manager.event_manager.used_txcmd_cnt--;
            }

            if (ll_manager.event_manager.used_txcmd_cnt == 1)
            {
                tx_entry_type = BZDMA_TX_ENTRY_TYPE_LE_DATA_BURST0 +
                                ll_manager.event_manager.cur_txcmd_id;
            }

            if (ll_manager.event_manager.used_txcmd_cnt == 0)
            {
                chk_bzdma = FALSE;
            }
        }
        while (0);

#ifdef _ROM_CODE_PATCHED_
        if (rcp_ll_early_func != NULL)
        {
            rcp_ll_early_func((void*)&reg_union.value, &chk_bzdma);
        }
#endif

        if (chk_bzdma)
        {
            /* check bzdma for le burst is busy or not ? */
            reg_value = RD_U32_BZDMA_REG(BZDMA_REG_TX_CMD(tx_entry_type));
            if (reg_value & BIT31)
            {
                if (IS_TOGGLE_LL_DATA_CMD)
                {
                    if (entry_id == event_manager->cur_tx_entry)
                    {
                        /* same entry */
                        bzdma_invalid_txcmd(tx_entry_type, 0, 0);
                        ll_handle_miss_previous_ce(event_manager->cur_tx_entry);
                        ll_manager.event_manager.used_txcmd_cnt--;
                    }
                }
                else
                {
                    /* original code flow */
                    bzdma_invalid_txcmd(tx_entry_type, 0, 0);
                    ll_handle_miss_previous_ce(event_manager->entry);
                    ll_manager.event_manager.used_txcmd_cnt--;
                }
            }
            else
            {
                bzdma_release_tx_entry(tx_entry_type);
                ll_manager.event_manager.used_txcmd_cnt--;
            }
        }

        if (ll_manager.event_manager.used_txcmd_cnt == 0)
        {
            ll_manager.event_manager.cur_txcmd_id = 0;
        }
#endif

#ifdef _ROM_CODE_PATCHED_
        if (rcp_ll_early_func != NULL)
        {
            if (rcp_ll_early_func((void*)&reg_union.value, entry_id))
            {
                return;
            }
        }
#endif

#ifndef USE_NEW_LE_SCHEDULER
#if defined(_NEW_BZDMA_FROM_V8_) && defined(_NEW_BZDMA_FROM_V8_CE_MISSING_PROCESS_)
        if (event_manager->bm_conn_event_window & (1 << entry_id))
        {
            /* missing previous ce now !! */
            /* this process is used to avoid le acl-u pause or terminating */
            bzdma_flush_ble_data_ring_fifo(entry_id, TRUE);
            ll_handle_miss_previous_ce(entry_id);
        }
#endif /* end of #ifndef _NEW_BZDMA_FROM_V8_ */
#endif /* ! USE_NEW_LE_SCHEDULER */

        event_manager->event_start = 1;
        event_manager->event_end = 0;
#ifndef USE_NEW_LE_SCHEDULER
        event_manager->bm_conn_event_window |= 1 << entry_id;
#endif

#ifdef LE_HW_TEST
        ll_test_unit.le_ce_cnt++;
#endif

        /* TODO: HW will schedule the connection entry in the following
           connection event. FW shall prepare burst LE tx data pkts via
           BZDMA tx command. */
        event_manager->entry = entry_id;
        event_manager->event_type = LL_CHANNEL_TYPE_DATA;
        handle = &ll_manager.conn_unit.handle[entry_id];

        /* do not scheduler any packet after LMP_TERMINATE_IND pdu is sent */
        if (handle->llc.pend_terminate_ind)
        {
            /* LL_TERMINATE_IND pdu is sending */
            if (!handle->terminate_timer_run)
            {
                /* schedule possible resent packet */
                ll_driver_data_channel_schedule_bzdma(entry_id, TRUE);

                ll_start_timer(entry_id, LL_TIMER_TYPE_TERMINATE);
                handle->terminate_timer_run = TRUE;

#ifdef USE_FREERTOS
                OS_STOP_TIMER(handle->sup_timer, NULL);
#else
                /* delete any running supervision timer */
                if (handle->sup_timer != NULL)
                {
                    OS_DELETE_TIMER(&handle->sup_timer);
                }
#endif
            }
            return;
        }

        /* start to scheduler burst packet for this connection entity */
        ll_driver_data_channel_schedule_bzdma(entry_id, FALSE);

#ifdef _DAPE_MOVE_LE_SLAVE_PARAMETER_TO_CAM
        UINT8 cam_entry = 0;
        if (ll_manager.conn_unit.master)
        {
            cam_entry = entry_id;
        }
        UINT8 dw_addr = LE_CAM_ENTRY_BASE(cam_entry) + LE_CAM_ADDR_4;
        handle->conn_counter = ll_driver_read_cam(dw_addr) >> 16;

#else
        /* update current connection counter */
        if (ll_manager.conn_unit.master)
        {
            UINT8 dw_addr = LE_CAM_ENTRY_BASE(entry_id) + LE_CAM_ADDR_4;
            handle->conn_counter = ll_driver_read_cam(dw_addr) >> 16;
        }
        else
        {
            handle->conn_counter = RD_LE_REG(LE_REG_CONN_COUNTER);
        }
#endif

#ifdef _CCH_SLOT_OFFSET_
#ifdef _DAPE_TEST_SLOT_OFFSET_FOR_LE
        handle->anchor_point_by_intr = BB_read_native_clock() + 1;
#endif
#endif

#ifdef _PAUSE_SCO_FOR_LE_CONN
        /* pause sco for the start of connection and the
           first 6 intervals after connection update is completed. */
        if ((PAUSE_SCO_FOR_LE) &&
            ((handle->conn_created)
#ifdef _DAPE_AUTO_CONN_UPDT_WHEN_SLOT_CONFLICT_AFTER_CONN_UPDT
            ||((handle->conn_counter - handle->updated_conn_counter < 6)
                 &&(handle->updated_conn_counter != 0))
#endif
            ))
        {
#ifdef _DAPE_GET_LEGACY_SLOT_OFST_AVOID
            UCHAR avoid = 0;
            UCHAR slot_ofst_avoid = 0xFF;
            lmp_get_avoid_slot_offset(&avoid, &slot_ofst_avoid);
            if ((avoid == 1) && ((slot_ofst_avoid == handle->slot_offset)
#ifdef _PAUSE_SCO_FOR_LE_SLV
                || (ll_manager.conn_unit.master == 0)
#endif
            ))
#endif
            {
                handle->conn_created = 0;
                bb_pause_sco(TRUE);
                bb_pause_esco(TRUE);
                handle->pause_sco = 1;
                lc_sco_pause_status |= BIT7;
            }
        }
#endif

#ifdef _ROM_CODE_PATCHED_
#ifdef _DAPE_8723_RCP_
        if (rcp_ll_ce_begin_func != NULL)
        {
            if (rcp_ll_ce_begin_func((void *)&entry_id))
            {
                return;
            }
        }
#endif
#endif

    }
    else
    {
        event_manager->event_start = 1;
        event_manager->event_end = 0;
        event_manager->event_type = LL_CHANNEL_TYPE_ADVERTISING;
    }

#ifdef _DAPE_TEST_CHK_LE_EARLY_INTR
    UINT32 ticks = read_vendor_counter_no_display();
    if (g_ll_ticks < ticks )
    {
        g_ll_ticks = ticks;  /* g_ll_ticks is UINT32 global variable */
        RT_BT_LOG(RED, DAPE_TEST_LOG293, 1,g_ll_ticks);
    }
#endif

}

#ifdef FIX_LE_HW_NO_BD_ADDR_CHECK
void ll_send_early_disconnection_complete_event(void *param)
{
    HCI_DISCONNECTION_COMPLETE_EVT_PARAM evt_param;
    LL_TASK_PARA_U lmp_param = { .Dword = (UINT32) param };
    UINT16 conn_handle = ll_manager.conn_unit.handle[lmp_param.lmp_s.conn_entry].conn_handle;
    evt_param.status = lmp_param.lmp_s.status;
    evt_param.conn_handle[0] = LSB(conn_handle);
    evt_param.conn_handle[1] = MSB(conn_handle);
    evt_param.reason = lmp_param.lmp_s.reason;
    hci_generate_event(HCI_DISCONNECTION_COMPLETE_EVENT, &evt_param,
            sizeof (evt_param));
}

void ll_early_kill_connection_with_existed_bd_addr(UINT8 conn_entry)
{
    int i;
    int existed_conn_ent = -1;
    LL_CONN_HANDLE_UNIT *chu, *chu2kill;

    chu = &ll_manager.conn_unit.handle[conn_entry];
    for (i = 0; i < LL_MAX_CONNECTION_UNITS; ++i)
    {
        if (i == conn_entry
                || !(ll_manager.conn_unit.bmActiveHandle & (1 << i))
                || !ll_manager.conn_unit.handle[i].connected)
        {
            continue;
        }
        chu2kill = &ll_manager.conn_unit.handle[i];
        if (chu->remote_info.addr_type == chu2kill->remote_info.addr_type
                && memcmp(chu->remote_info.addr, chu2kill->remote_info.addr,
                        sizeof (chu->remote_info.addr)) == 0)
        {
            existed_conn_ent = i;
            break;
        }
    }

    if (existed_conn_ent >= 0)
    {
        LL_TASK_PARA_U lmp_param = { .Dword = 0 };
        lmp_param.lmp_s.conn_entry = existed_conn_ent;
        lmp_param.lmp_s.status = HCI_COMMAND_SUCCEEDED;
        lmp_param.lmp_s.reason = CONNECTION_TERMINATED_LOCAL_HOST_ERROR;
        OS_SIGNAL sig = {
                .type = LMP_GENERIC_SIGNAL,
                .param = ll_send_early_disconnection_complete_event,
                .ext_param = (void *) lmp_param.Dword
        };
        OS_SEND_SIGNAL_TO_TASK(lmp_task_handle, sig);

        chu2kill->early_killed = TRUE;
        ll_manager.conn_unit.handle[existed_conn_ent].kill_link_reason =
                LL_KILL_REASON_EXISTED_BD_ADDR;
        ll_driver_kill_connection(existed_conn_ent);
    }
}
#endif /* FIX_LE_HW_NO_BD_ADDR_CHECK */

#ifdef _ROM_CODE_PATCHED_
BOOLEAN (*rcp_ll_handle_conn_interrupt)(LE_REG_S_STATUS_CONN_STATUS *, UINT8 *) = NULL;
#endif
/**************************************************************************
 * Function     : ll_handle_conn_interrupt
 *
 * Description  : This function is used to handle ISR of initiating state or
 *                connected state change
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
SECTION_LE_ISR void ll_handle_conn_interrupt(void)
{
    LE_REG_S_SET reg_union;
    UINT8 conn_entry;
    UINT8 hw_conn_ent;
    UINT8 conn_num;
    OS_SIGNAL sig_send;
    LL_TASK_PARA_U task_param;
    LL_CONN_HANDLE_UNIT *handle;
    UINT8 del_timer = FALSE;

    /* read LE_REG_STATUS_CONN_STATUS to clear interrupt */
    reg_union.value = RD_LE_REG(LE_REG_STATUS_CONN_STATUS);
#ifdef _ROM_CODE_PATCHED_
    if (rcp_ll_handle_conn_interrupt != NULL)
    {
        if (rcp_ll_handle_conn_interrupt(&reg_union.status_conn_status,
                &del_timer))
        {
            return;
        }
    }
#endif

    conn_num = reg_union.status_conn_status.conn_num;
    hw_conn_ent = reg_union.status_conn_status.entry;
    handle = &ll_manager.conn_unit.handle[hw_conn_ent];

    switch (reg_union.status_conn_status.term_reason)
    {
        case LE_CONN_TERM_REASON_CONNECT_SUCCESS:
            if (reg_union.status_conn_status.role == LL_MASTER)
            {
                UINT16 bm_conn_entry = RD_LE_REG(LE_REG_STATUS_CAM_VALID);
                conn_entry = ll_manager.initiator_unit.entry_id;

                if (!ll_manager.initiator_unit.enable ||
                        (conn_entry != hw_conn_ent) ||
                        !(bm_conn_entry & (1 << conn_entry)))
                {
                    LL_LOG_TRACE(RED, LE_MSG_MASTER_CONN_SUCCESS_ERROR, 4,
                                 ll_manager.initiator_unit.enable, conn_entry,
                                 hw_conn_ent, bm_conn_entry);
                    break;
                }

                /* initiator state -> master role. The first function fill the
                   BD_ADDR of remote device. */
                ll_fw_fill_remote_conn_entry_param(LL_MASTER, conn_entry);
                ll_manager.initiator_unit.enable = 0;
                ll_manager.conn_unit.enable = 1;
                ll_manager.conn_unit.master = 1;
                ll_manager.conn_unit.connection_cnts = conn_num;

#ifdef FIX_LE_HW_NO_BD_ADDR_CHECK
                ll_early_kill_connection_with_existed_bd_addr(conn_entry);
#endif

                /* dape added for record the first connection. */
                if (ll_manager.conn_unit.connection_cnts == 1)
                {
                    ll_manager.conn_unit.first_connection_entry = conn_entry;
                }
#ifdef _DAPE_NO_TRX_WHEN_LE_SEND_CONN_REQ_HW_TIMER
                if (IS_TIMER3_FOR_LE_DUAL_WHCK)
                {
                    UINT16 conn_win_size_ofst = RD_LE_REG(LE_REG_CONN_UPD_WIN_SIZE) >> 8;
                    UINT16 conn_win_ofst = RD_LE_REG(LE_REG_CONN_WIN_OFFSET);
                    /* first pkt = conn_win_size_ofst(ms) + conn_win_ofst(ms) + 1.25ms*/
                    turn_on_timer3_for_le(conn_win_size_ofst, (conn_win_ofst + 1));
                    RT_BT_LOG(BLUE, DAPE_TEST_LOG207, 1, BB_read_native_clock());
                }
#endif
                ll_handle_master_connection_complete_process_second_part(hw_conn_ent);
#ifdef LE_HW_TEST
                ll_test_trigger_one_shot_event(LL_TEST_CASE_START_ENCRPT, 0);
#endif

                /* create and start the supervision timer */
                ll_start_timer(conn_entry, LL_TIMER_TYPE_SUPERVISION);

            }
            else
            {
                if (!ll_manager.conn_unit.enable ||
                        ll_manager.conn_unit.master || (conn_num != 1))
                {
                    LL_LOG_TRACE(RED, LE_MSG_SALVE_CONN_SUCCESS_ERROR, 3,
                                 conn_num, ll_manager.conn_unit.enable,
                                 ll_manager.conn_unit.master);
                    break;
                }
                ll_manager.conn_unit.connection_cnts = conn_num;

                /* advertising state -> slave role */
                ll_fw_fill_remote_conn_entry_param(LL_SLAVE, 0);

                if (ll_manager.conn_unit.handle[0].recv_conn_req)
                {
                    /* generate connection complete event to host */
                    task_param.Dword = 0;
                    task_param.lmp_s.sub_type = LL_TASK_HANDLE_CONN_COMP;
                    task_param.lmp_s.conn_entry = 0;
                    task_param.lmp_s.status = HCI_COMMAND_SUCCEEDED;
                    sig_send.type = LL_GEN_HCI_EVENT_SIGNAL;
                    sig_send.param = (OS_ADDRESS)task_param.Dword;
                    OS_SEND_SIGNAL_TO_TASK (lmp_task_handle, sig_send);

                    /* create and start the supervision timer */
                    ll_start_timer(0, LL_TIMER_TYPE_SUPERVISION);
                }
            }
            break;

        case LE_CONN_TERM_REASON_USER_DISCONNET:
            /* send a Disconnection Complete event to the host if the command
               is from the host or firmware */
            if (handle->connected)
            {
                task_param.Dword = 0;
                task_param.lmp_s.sub_type = LL_TASK_HANDLE_DISCONN;
                task_param.lmp_s.conn_entry = hw_conn_ent;
                task_param.lmp_s.status = HCI_COMMAND_SUCCEEDED;
                if (handle->llc.loc_term_from_host)
                {
                    task_param.lmp_s.reason = CONNECTION_TERMINATED_LOCAL_HOST_ERROR;
                }
                else
                {
                    task_param.lmp_s.reason = handle->llc.loc_terminate_reason;
                }
                sig_send.type = LL_GEN_HCI_EVENT_SIGNAL;
                sig_send.param = (OS_ADDRESS)task_param.Dword;
                OS_SEND_SIGNAL_TO_TASK (lmp_task_handle, sig_send);

                del_timer = TRUE;
            }

            LL_LOG_TRACE(RED, LE_MSG_USER_DISCONNECT_CMD, 2,
                         hw_conn_ent, ll_manager.conn_unit.handle[hw_conn_ent].connected);
            break;

        case LE_CONN_TERM_REASON_USER_CANCEL:
            LL_LOG_TRACE(RED, LE_MSG_KILL_CONNECTION_INFO, 3,
                         hw_conn_ent, handle->connected, handle->kill_link_reason);

            if (!handle->connected)
            {
                break;
            }

            /* generate subitem for disconnect */
            task_param.Dword = 0;
            task_param.lmp_s.sub_type = LL_TASK_HANDLE_DISCONN;
            task_param.lmp_s.conn_entry = hw_conn_ent;
            task_param.lmp_s.status = HCI_COMMAND_SUCCEEDED;

            switch (handle->kill_link_reason)
            {
                case LL_KILL_REASON_SUPERVISION_TIMEOUT:
                    /* the link shall be disconnected as supervision timeout */
                    if (!handle->sup_timeout_restart)
                    {
                        task_param.lmp_s.reason = CONNECTION_FAILED_TO_BE_ESTABLISHED;
                    }
                    else
                    {
                        task_param.lmp_s.reason = CONNECTION_TIMEOUT_ERROR;
                    }
                    break;

                case LL_KILL_REASON_LLC_TIMEOUT:
                    /* send a Disconnection Complete event after LLC procedure response
                       timeout */
                    task_param.lmp_s.reason = LL_RESPONSE_TIMEOUT_ERROR;
                    break;

                case LL_KILL_REASON_UPDT_PAST:
                    /* send a Disconnection Complete event after update time was past */
                    task_param.lmp_s.reason = INSTANT_PASSED_ERROR;
                    break;

#ifdef FIX_LE_HW_NO_BD_ADDR_CHECK
                case LL_KILL_REASON_EXISTED_BD_ADDR:
                    /* Disconnection Complete Event is already sent (early_killed == TRUE).
                     * No need to set reason.
                     */
                    break;
#endif

                default:
                    /* send a Disconnection Complete event after receive MIC error pkt*/
                    task_param.lmp_s.reason = CONNECTION_TERMINATED_DUE_TO_MIC_FAILURE;
                    break;
            }

            sig_send.type = LL_GEN_HCI_EVENT_SIGNAL;
            sig_send.param = (OS_ADDRESS)task_param.Dword;
            OS_SEND_SIGNAL_TO_TASK (lmp_task_handle, sig_send);

#ifdef LE_ISSUE_INSTR_IN_ORDER_ON_COMPLT
            {
                LE_INSTR_ENTITY instr = ll_driver_issue_next_kill_connection();
                if (instr.conn.conn_entry != hw_conn_ent)
                {
                    LL_LOG_TRACE(RED, LE_MSG_KILL_CONN_ENT_MISMATCH, 2,
                            instr.conn.conn_entry, hw_conn_ent);
                }
            }
#endif
            del_timer = TRUE;
            break;

        case LE_CONN_TERM_REASON_SIX_FAILURE:
            /* send a Disconnection Complete event after six retry timeout */
            if (handle->connected)
            {
                task_param.Dword = 0;
                task_param.lmp_s.sub_type = LL_TASK_HANDLE_DISCONN;
                task_param.lmp_s.conn_entry = hw_conn_ent;
                task_param.lmp_s.status = HCI_COMMAND_SUCCEEDED;
                if (handle->rx_any_in_conn_state)
                {
                    task_param.lmp_s.reason = CONNECTION_TIMEOUT_ERROR;
                }
                else
                {
                    task_param.lmp_s.reason = CONNECTION_FAILED_TO_BE_ESTABLISHED;
                }
                sig_send.type = LL_GEN_HCI_EVENT_SIGNAL;
                sig_send.param = (OS_ADDRESS)task_param.Dword;
                OS_SEND_SIGNAL_TO_TASK (lmp_task_handle, sig_send);

                del_timer = TRUE;
            }
            else
            {
                LL_LOG_TRACE(RED, LE_MSG_SIX_TIMEOUT_EXCEPTION, 3,
                             hw_conn_ent, ll_manager.adv_unit.enable,
                             ll_manager.initiator_unit.enable);
            }
            break;

        case LE_CONN_TERM_REASON_RX_TERM_IND:
            /* handle LL_TERMINATE_IND pkt in RX FIFO */
            if (handle->connected)
            {
                del_timer = TRUE;
            }
            break;

        default:
            break;
    }

    /* This Block handles the disconnection needed procedure. */
    /* delete supervision timer or llc procedure timer */
    if (del_timer == TRUE)
    {
#ifdef USE_FREERTOS
        OS_STOP_TIMER(handle->llc_timer, NULL);
        OS_STOP_TIMER(handle->sup_timer, NULL);
#else
        /* free llc timer */
        if (handle->llc_timer != NULL)
        {
            OS_DELETE_TIMER(&handle->llc_timer);
        }

        /* free supervision timer */
        if (handle->sup_timer != NULL)
        {
            OS_DELETE_TIMER(&handle->sup_timer);
        }
#endif

#ifdef _CCH_SLOT_OFFSET_
#ifdef _DAPE_TEST_SLOT_OFFSET_FOR_LE
        lmp_disconnect_global_slot_offset(hw_conn_ent + LL_HCI_MIN_CONN_HANDLE);
#endif
#endif
#ifdef _DAPE_SET_ESCO_RETRY_PRIORITY_HIGHER_WHILE_NO_FULL_BW
    /* If esco is not full bandwidth, then we can set the
       priority of esco retry window to high.*/
    /*if ()&&
    ((lc_esco_window[esco_lt_addr] == TRUE))
    {
        UINT16 reg;
        reg = BB_read_baseband_register(0x5E);
        BB_write_baseband_register(0x5E, reg|BIT14);
        reg = BB_read_baseband_register(0x120);
        BB_write_baseband_register(0x120, reg | BIT8);
    }*/

#endif

    }
    else
    {
        UINT16 reg;

        reg = BB_read_baseband_register(0x5E);
        BB_write_baseband_register(0x5E, reg&(~BIT14));
        reg = BB_read_baseband_register(0x120);
        BB_write_baseband_register(0x120, reg & (~BIT8));
    }

    LL_LOG_TRACE(GRAY, LE_MSG_QUIT_CONN, 2,
                 reg_union.status_conn_status.term_reason, reg_union.value);
}

#ifdef _ROM_CODE_PATCHED_
BOOLEAN (*rcp_ll_handle_adv_interrupt)(LE_REG_S_STATUS_ADV_STATUS *) = NULL;
#endif
/**************************************************************************
 * Function     : ll_handle_adv_interrupt
 *
 * Description  : This function is used to handle ISR of advertising state
 *                change (scanning state -> standby or connected state)
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
SECTION_LE_ISR void ll_handle_adv_interrupt(void)
{
    LE_REG_S_SET reg_union;
    OS_SIGNAL sig_send;
    LL_TASK_PARA_U task_param;

    /* read LE_REG_STATUS_ADV_STATUS to clear interrupt */
    reg_union.value = RD_LE_REG(LE_REG_STATUS_ADV_STATUS);
#ifdef _ROM_CODE_PATCHED_
    if (rcp_ll_handle_adv_interrupt != NULL)
    {
        if (rcp_ll_handle_adv_interrupt(&reg_union.status_adv_status))
        {
            return;
        }
    }
#endif

    switch (reg_union.status_adv_status.term_reason)
    {
        case LE_ADV_TERM_REASON_USER_CANCEL:
#ifndef _MODI_LPS_AFTER_RTL8821B_TC_
            ll_manager.adv_unit.enable = 0;
#endif
            break;

        case LE_ADV_TERM_REASON_CONN_CREATED:
            if (ll_manager.adv_unit.enable)
            {
                ll_manager.conn_unit.enable = 1;
                ll_manager.conn_unit.master = 0;
                ll_manager.adv_unit.enable = 0;
            }
            break;

        case LE_ADV_TERM_REASON_DIRECT_ADV_TIMEOUT:
            if (ll_manager.adv_unit.enable)
            {
                /* send a LE Connection Complete event with reason code 0x3C
                   to the host */
                task_param.Dword = 0;
                task_param.lmp_s.sub_type = LL_TASK_HANDLE_CONN_COMP;
                task_param.lmp_s.conn_entry = 0;
                task_param.lmp_s.status = DIRECTED_ADVERTISING_TIMEOUT;
                sig_send.type = LL_GEN_HCI_EVENT_SIGNAL;
                sig_send.param = (OS_ADDRESS)task_param.Dword;
                OS_SEND_SIGNAL_TO_TASK (lmp_task_handle, sig_send);
            }
            break;

        default:
            ll_manager.adv_unit.enable = 0;
            break;
    }

#ifdef _NEW_BZDMA_FROM_V8_
    bzdma_invalid_ble_txcmd(BZDMA_LEONLY_TX_ENTRY_TYPE_ADV0);
#else
    bzdma_invalid_txcmd(BZDMA_TX_ENTRY_TYPE_LE_ADV, 0, 0);
#endif

#ifdef _MODI_LPS_AFTER_RTL8821B_TC_
    if(reg_union.status_adv_status.term_reason != LE_ADV_TERM_REASON_USER_CANCEL)
    {
        LL_LOG_TRACE(GRAY, LE_MSG_QUIT_ADV, 2,
                 reg_union.status_adv_status.term_reason, reg_union.value);

        lmp_update_lps_para();
    }
#else
    LL_LOG_TRACE(GRAY, LE_MSG_QUIT_ADV, 2,
                 reg_union.status_adv_status.term_reason, reg_union.value);
#endif
}

/**************************************************************************
 * Function     : ll_handle_scan_interrupt
 *
 * Description  : This function is used to handle ISR of scanning state change
 *                (scanning state -> standby)
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
SECTION_LE_ISR void ll_handle_scan_interrupt(void)
{
#ifndef _MODI_LPS_AFTER_RTL8821B_TC_
    /* TODO: HW notify FW that it has quited the scanning state */
    LL_LOG_TRACE(GRAY, LE_MSG_QUIT_SCAN, 0, 0);
#endif

}

/**************************************************************************
 * Function     : ll_handle_update_interrupt
 *
 * Description  : This function is used to handle ISR of connection update or
 *                channel map update
 *
 * Parameters   : type: Connection Update or Channel Map Update
 *
 * Returns      : None
 *
 *************************************************************************/
SECTION_LE_ISR void ll_handle_update_interrupt_imp(UINT8 type)
{
    UINT8 entry;
    LL_CONN_HANDLE_UNIT *handle;

#ifdef _DAPE_FIX_HW_NO_LE_SCAN_N_CONN_UPDT_SAME_TIME
    if ((type == LL_UPDATE_REASON_CONN_PARAM) ||
        (type == LL_UPDATE_REASON_CH_MAP))
    {
        if (ll_manager.scan_unit.enable)
        {
#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_LE_
            ll_driver_enable_scanning(FALSE);
#else
            ll_driver_enable_scanning();
#endif
        }
        else
        {
            ll_driver_disable_scanning();
            ll_driver_clear_all_duplicated_flags_in_list(LL_WHITE_LIST_TYPE);
            ll_driver_clear_all_duplicated_flags_in_list(LL_BLACK_LIST_TYPE);
        }
        RT_BT_LOG(BLUE, DAPE_TEST_LOG541, 4, ll_manager.scan_unit.enable,
                   ll_manager.conn_unit.conn_updt_entry,
                   ll_manager.conn_unit.chm_updt_entry,
                   ll_manager.scan_unit.filter_duplicate);
    }
#endif
    if ((type == LL_UPDATE_REASON_STOP_SUPERVISION_TIMER) ||
            (type == LL_UPDATE_REASON_CONN_PARAM))
    {
        /* connection update */
        entry = ll_manager.conn_unit.conn_updt_entry;

        if (entry != LL_MAX_CONNECTION_UNITS)
        {
            LL_CONN_UPDT_BLK *pConnUpdt;
            LL_TASK_PARA_U task_param;
            OS_SIGNAL sig_send;
            UINT8 set_updt = TRUE;

            pConnUpdt = &ll_manager.conn_unit.handle[entry].conn_updt_blk;
            handle = &ll_manager.conn_unit.handle[entry];

            if (type == LL_UPDATE_REASON_STOP_SUPERVISION_TIMER)
            {
#ifdef USE_FREERTOS
                OS_STOP_TIMER(handle->sup_timer, NULL);
#else
                if (handle->sup_timer != NULL)
                {
                    /* stop the timer if we have the body of supervision timer.
                       this can avoid the supervision timeout when sync
                       transmit window in update procedure */
                    OS_DELETE_TIMER(&handle->sup_timer);
                }
#endif
#ifdef _DAPE_NO_TRX_WHEN_LE_CONN_UPDT
                if (IS_TIMER3_FOR_LE_DUAL_WHCK)
                {
                    UINT16 conn_win_size_ofst = RD_LE_REG(LE_REG_CONN_UPD_WIN_SIZE) >> 8;
                    UINT16 conn_win_ofst = RD_LE_REG(LE_REG_CONN_UPD_WIN_OFFSET);
                    /* this interrupt comes at ce_end of the instant before
                       Update instant. So besides win_offset & win_size offset,
                       we still need to add an ce_interval. */
                    turn_on_timer3_for_le(conn_win_size_ofst, conn_win_ofst+handle->ce_interval-4);
                    RT_BT_LOG(YELLOW, DAPE_TEST_LOG213, 2, conn_win_size_ofst, conn_win_ofst);
                }
#endif
#ifdef _DAPE_TEST_NEW_HW_BLOCK_ACL_FOR_LE
                {
                    ll_driver_block_legacy_slot_for_le(7);
                    g_enable_le_block_legacy = 1;
                }
#endif
#ifdef _DAPE_SKIP_HW_LE_FAKE_UPDT_INTR
                /* dape added 20121121. HW will introduce this interrupt
                   whenever the event end is AE-end or CE-end.
                   Only the first one is correct, which happens in CE end.
                   So we mask this interrupt at the time we receive the
                   first interrupt. and enable it again after interrupt-13
                   happens. */
                ll_manager.int_imr |= BIT11;
                WR_LE_REG(LE_REG_INT_IMR, ll_manager.int_imr);
#endif
                return;
            }


                //ll_fw_check_all_slave_interval();

                handle ->conn_update_le_slot_updated = FALSE;
#ifdef _DAPE_PAUSE_SCO_WHEN_CONN_UPDT
                if (LE_AUTO_UPDT_WHEN_SLOT_CONFLICT_AFTER_CONN_UPDT)
                {
                  handle->conn_created = 0;
                  bb_pause_sco(TRUE);
                  bb_pause_esco(TRUE);
                  handle->pause_sco = 1;
                  lc_sco_pause_status |= BIT7;
                  //ll_fw_get_update_slot(entry, 1);
                }
#endif
#ifdef _DAPE_SKIP_HW_LE_FAKE_UPDT_INTR
                ll_manager.int_imr &= (~BIT11);
                WR_LE_REG(LE_REG_INT_IMR, ll_manager.int_imr);
#endif

            /* we shall not return connection update complete event to
               host if no parameters are changed */
            if ((handle->ce_interval == pConnUpdt->ce_interval) &&
                    (handle->slave_latency == pConnUpdt->slave_latency) &&
                    (handle->supervision_to == pConnUpdt->supervision_to)
#ifdef _DAPE_TEST_SEND_CONN_UPT_COMPLETE_EVT_WHEN_HOST_REQ
                     && (handle->hci_cmd_bm.conn_updt == 0)
#endif
                    )
            {
                set_updt = FALSE;
            }


#ifdef _DAPE_PRINT_MORE_LOG_INFO
            RT_BT_LOG(GREEN, DAPE_TEST_LOG569, 12, BB_read_native_clock(),
            entry, ll_manager.conn_unit.master,
            handle->ce_interval, pConnUpdt->ce_interval,
            handle->ce_length, pConnUpdt->ce_length,
            handle->slave_latency, pConnUpdt->slave_latency,
            handle->supervision_to, pConnUpdt->supervision_to,
            handle->conn_counter);
#endif
            if (ll_manager.conn_unit.master)
            {
                handle->ce_length = pConnUpdt->ce_length;
            }
            else
            {
                if (pConnUpdt->slave_latency > 0)
                {
                    /* enable slave latency */
                    LE_REG_S_SET reg;
                    reg.value = RD_LE_REG(LE_REG_SLAVE_WIN_WIDENING_L);
                    reg.slave_win_widen_l.sub_en = TRUE;
                    WR_LE_REG(LE_REG_SLAVE_WIN_WIDENING_L, reg.value);
                }
            }
            handle->ce_interval = pConnUpdt->ce_interval;
            handle->slave_latency = pConnUpdt->slave_latency;
            handle->supervision_to = pConnUpdt->supervision_to;

            handle->llc.type = LLC_PROCEDURE_TYPE_NONE;
            ll_fw_check_all_slave_interval();
            /* restart the supervision timer and update the timeout value */
            ll_start_timer(entry, LL_TIMER_TYPE_SUPERVISION);

            /* TODO: turn next connection entity */
            ll_manager.conn_unit.conn_updt_entry = LL_MAX_CONNECTION_UNITS;

            if (set_updt == TRUE)
            {
#ifdef _DAPE_TEST_SEND_CONN_UPT_COMPLETE_EVT_WHEN_HOST_REQ
                if (handle->hci_cmd_bm.conn_updt)
                {
                    handle->hci_cmd_bm.conn_updt = 0;
                }
#endif
                /* generate connection update complete event to host */
                task_param.Dword = 0;
                task_param.lmp_s.sub_type = LL_TASK_HANDLE_CONN_UPDT_COMP;
                task_param.lmp_s.conn_entry = entry;
                task_param.lmp_s.status = HCI_COMMAND_SUCCEEDED;
                sig_send.type = LL_GEN_HCI_EVENT_SIGNAL;
                sig_send.param = (OS_ADDRESS)task_param.Dword;
                OS_SEND_SIGNAL_TO_TASK (lmp_task_handle, sig_send);
            }
        }
    }
    else
    {
        /* channel map update */
        entry = ll_manager.conn_unit.chm_updt_entry;

        if (entry != LL_MAX_CONNECTION_UNITS)
        {
            handle = &ll_manager.conn_unit.handle[entry];
            handle->bmChMap[0] = ll_manager.conn_unit.updt_ch_map[0];
            handle->bmChMap[1] = ll_manager.conn_unit.updt_ch_map[1];
            handle->bmChMap[2] = ll_manager.conn_unit.updt_ch_map[2];
            handle->bmChMap[3] = ll_manager.conn_unit.updt_ch_map[3];
            handle->bmChMap[4] = ll_manager.conn_unit.updt_ch_map[4];
            handle->llc.type = LLC_PROCEDURE_TYPE_NONE;

            /* TODO: turn next connection entity */
            ll_manager.conn_unit.chm_updt_entry = LL_MAX_CONNECTION_UNITS;

            LL_CONNECTION_UNIT *conn = &ll_manager.conn_unit;
            LE_REG_S_SET reg;
            if ((!conn->master)&&(handle->slave_latency !=0))
            {
                /* enable slave latency */
                reg.value = RD_LE_REG(LE_REG_SLAVE_WIN_WIDENING_L);
                reg.slave_win_widen_l.sub_en = TRUE;
                WR_LE_REG(LE_REG_SLAVE_WIN_WIDENING_L, reg.value);
            }

        }
    }
}
void (*ll_handle_update_interrupt)(UINT8) = ll_handle_update_interrupt_imp;

#ifdef _DAPE_LE_MULTI_SLAVE_METHOD_2
const UINT16 nth_interval[LL_MAX_CONNECTION_UNITS] = {0, 4, 2, 6, 1, 5, 3, 7};
#endif

SECTION_LE_ISR void ll_handle_hit_connected_adv_int_no_link_policy(void)
{
    LL_INITIATOR_UNIT *initiator = &ll_manager.initiator_unit;
    UINT8 conn_entry_id;
    UINT32 cur_clock;
    UINT16 slot_offset_for_le_conn;
    UINT16 conn_win_size_offset;
    UINT16 cur_slot_offset;
    UINT16 slot_min;
    UINT16 slot_max;
    UINT16 conn_intv_slot;
    UINT16 temp;
    UINT8 state = 0;

    //UINT16 count_down = 888;

    conn_entry_id = initiator->entry_id;

    cur_clock = BB_read_native_clock();
    conn_intv_slot = initiator->conn_interval << 1;
    if ((conn_intv_slot > GLOBAL_SLOT_INTERVAL) ||
        (conn_intv_slot == 0))
    {
        conn_intv_slot = GLOBAL_SLOT_INTERVAL;
    }

    cur_slot_offset = (cur_clock >> 1) % conn_intv_slot;

    /* slot_min & slot_max: unit:625us. the slot offset range of tx_win */
    /* start from tx_win + 1.25 ms and stops 1.25ms before tx_win ends*/
    /*slot_min = (cur_slot_offset & ~0x01) + 4 +
               (initiator->tx_win_offset << 1) +
               (LL_CONN_TX_WIN_SIZE_OFFSET * 2);*/
    slot_min = (cur_slot_offset & ~0x01) + 2 +
               (initiator->tx_win_offset << 1) +
               (LL_CONN_TX_WIN_SIZE_OFFSET * 2);

    ///// for bitfile after 0325 /////
    UINT16 adv_hit_counter_n;
    adv_hit_counter_n = ll_fw_get_adv_hit_counter_n();

    if ((cur_slot_offset & 1) && (adv_hit_counter_n < 502 ))
    {
        slot_min = slot_min + 2;
    }
    slot_max = slot_min + (initiator->tx_win_size << 1) -
                        (LL_CONN_TX_WIN_SIZE_OFFSET * 2) - 2;
    slot_offset_for_le_conn = lmp_get_global_slot_offset(
                                  conn_entry_id + LL_HCI_MIN_CONN_HANDLE, 0,
                                  conn_intv_slot, 2, slot_min, slot_max);

    if (slot_offset_for_le_conn >= GLOBAL_SLOT_INTERVAL)
    {
        /* To Do: When slot_offset_for_le_conn >= GLOBAL_SLOT_INTERVAL,
           It means slot is overlapping. We should choose an appropriate value
            For anchor point. - dape */
        //RT_BT_LOG(RED, DAPE_TEST_LOG417, 2,slot_offset_for_le_conn, slot_min);
        slot_offset_for_le_conn = slot_min;
        state |= BIT0;
    }

    temp = slot_min % conn_intv_slot;
    // dape modified 20121105
    /*conn_win_size_offset = slot_offset_for_le_conn - temp;
    if (slot_offset_for_le_conn < temp)
    {
        conn_win_size_offset += conn_intv_slot;
    }*/
    if (slot_offset_for_le_conn < temp)
    {
        conn_win_size_offset = slot_offset_for_le_conn - temp + conn_intv_slot;
    }
    else
    {
       conn_win_size_offset = slot_offset_for_le_conn - temp;
    }
    //temp = (conn_win_size_offset >> 1) + LL_CONN_TX_WIN_SIZE_OFFSET;
    /* dape modified 2012/11/05, no need to add LL_CONN_TX_WIN_SIZE_OFFSET
       here because we add this before in the parameter "slot_min".*/
    temp = (conn_win_size_offset >> 1);
    if (temp >=  initiator->tx_win_size)
    {
        //RT_BT_LOG(RED, DAPE_TEST_LOG415, 2, conn_win_size_offset >> 1,
        //                                    initiator->tx_win_size);
        conn_win_size_offset = 0;
        temp = LL_CONN_TX_WIN_SIZE_OFFSET;
        state |= BIT1;
    }

    ll_driver_fill_conn_win_size_offset(temp);

    ll_manager.conn_unit.current_slot_offset_for_le = slot_offset_for_le_conn;

    RT_BT_LOG(BLUE, DAPE_TEST_LOG416, 11, cur_slot_offset,
              slot_offset_for_le_conn,
              cur_clock, initiator->conn_interval,
              conn_win_size_offset,
              initiator->tx_win_offset,
              slot_min, slot_max,
              temp,adv_hit_counter_n,
              state);
    /*RT_BT_LOG(BLUE, DAPE_TEST_LOG416, 11, cur_slot_offset,
              slot_offset_for_le_conn,
              cur_clock, initiator->conn_interval,
              conn_win_size_offset,
              initiator->tx_win_offset,
              slot_min, slot_max,
              (slot_min % conn_intv_slot),
              adv_hit_counter_n,
              state);*/
}

SECTION_LE_ISR void ll_handle_hit_connected_adv_int_any_link_policy(void)
{
    LL_INITIATOR_UNIT *initiator = &ll_manager.initiator_unit;
    UINT32 anchor_point_slot_max = 0;
    UINT32 anchor_point_slot_min = 0xFFFFFFFF;
    UINT32 next_anchor_point_slot_min = 0xFFFFFFFF;
    UINT32 cur_clock;
    UINT32 cur_clock_slot;
    UINT32 con_win_start_clk_slot;
    UINT32 con_win_end_clk_slot;
    UINT16 conn_win_size_offset = 1; /* don't change the initial value; */
    UINT8  num_le_conn = 0;
#ifdef _DAPE_LE_MULTI_SLAVE_METHOD_2
    UINT16 next_anchor_distance = ll_manager.conn_unit.anchor_distance_min;
    UINT32 next_anchor_point = 0;
    UINT32 next_anchor_point_offset = 0;
    UINT16 conn_intv_slot;
    UINT16 cur_clock_slot_offset = 0;
    UINT16 empty_entry = LL_MAX_CONNECTION_UNITS;
    UINT8 ce_interval_same = ll_manager.conn_unit.ce_interval_same;
#endif
    cur_clock = BB_read_native_clock();
    cur_clock_slot = cur_clock>>1;
    ///// for bitfile after 0325 /////
    UINT16 adv_hit_counter_n;
    adv_hit_counter_n = ll_fw_get_adv_hit_counter_n();
    /* Calculate the start of conn_window. */
    con_win_start_clk_slot = cur_clock_slot + 4 - (cur_clock_slot&1) +
                             ((initiator->tx_win_offset) << 1);

    ///// for bitfile after 0325 /////
    if ((cur_clock_slot&1) && (adv_hit_counter_n < 502))
    {
        con_win_start_clk_slot = con_win_start_clk_slot + 2;
    }

    ll_fw_get_all_anchor_point(&anchor_point_slot_max,
        &anchor_point_slot_min, &next_anchor_point_slot_min,
        &num_le_conn, &empty_entry, cur_clock_slot);


    con_win_end_clk_slot = con_win_start_clk_slot + ((initiator->tx_win_size - 1) << 1);
    conn_intv_slot = initiator->conn_interval << 1;

    if (ce_interval_same)
    {
        cur_clock_slot_offset = cur_clock_slot % conn_intv_slot;
    }

#ifdef _DAPE_LE_MULTI_SLAVE_METHOD_2
    UCHAR tx_win_offset_has_to_change = 0;
    UINT16 tx_win_offset = initiator->tx_win_offset;
    UINT32 ce_interval_slot_min = ll_manager.conn_unit.ce_interval_min << 1;
    //UINT8 dape_test = 0;

    ll_manager.conn_unit.tx_win_offset = initiator->tx_win_offset;
    if (ce_interval_same)
    {
        next_anchor_distance = nth_interval[empty_entry] * (ce_interval_slot_min >> 3);
        next_anchor_point_offset = ll_manager.conn_unit.first_slot_offset_for_le + next_anchor_distance;
        next_anchor_point = cur_clock_slot + (next_anchor_point_offset - cur_clock_slot_offset);

        if (next_anchor_point_offset < cur_clock_slot_offset)
        {
            next_anchor_point += conn_intv_slot;
        }
        ll_manager.conn_unit.current_slot_offset_for_le = next_anchor_point_offset % conn_intv_slot;
    }
    else
    {
        UINT32 free_space;
        if (next_anchor_point_slot_min > anchor_point_slot_max)
        {
            free_space = next_anchor_point_slot_min - anchor_point_slot_max;
            next_anchor_distance = ll_fw_get_gcd(ll_fw_get_gcd(ll_fw_get_gcd(ce_interval_slot_min,
                                                 initiator->conn_interval << 1),
                                                 free_space),
                                                 ll_manager.conn_unit.anchor_distance_min) >> 1;
            if (next_anchor_distance == 0)
            {
                next_anchor_distance = 4;
            }
        }
        else
        {
            next_anchor_distance = 4;
        }
        ll_manager.conn_unit.anchor_distance_min = next_anchor_distance;
        next_anchor_point = anchor_point_slot_min - (UINT32)next_anchor_distance;
        if (next_anchor_distance & 0x01)
        {
            next_anchor_point--;
        }
    }
    if (num_le_conn != 0)
    {
        /* Check if next_anchor_point is in the tx_win_size */
        if ((con_win_start_clk_slot <= next_anchor_point) &&
            (next_anchor_point <= con_win_end_clk_slot))
        {
            /* next_anchor_point is inside the tx_win_size, so we only need to change con_win_size_offset*/
            conn_win_size_offset = (next_anchor_point - con_win_start_clk_slot) >> 1;
            //dape_test = 1;
        }
        else
        {
            tx_win_offset_has_to_change = 1;
            if (next_anchor_point < con_win_start_clk_slot)
            {
                /* next_anchor_point is earlier than the tx_win_size. we have to change tx_win_offset */
                if (initiator->tx_win_offset >
                    ((con_win_start_clk_slot - next_anchor_point) >> 1))
                {
                    conn_win_size_offset = 1;
                    tx_win_offset = initiator->tx_win_offset -
                                    ((con_win_start_clk_slot - next_anchor_point) >> 1)
                                    - conn_win_size_offset;
                    //dape_test = 2;
                }
                else
                {
                    next_anchor_point = next_anchor_point + (UINT32)conn_intv_slot;
                    conn_win_size_offset = 1;
                    tx_win_offset = initiator->tx_win_offset
                                    + ((next_anchor_point - con_win_start_clk_slot) >> 1)
                                    - conn_win_size_offset;
                    //dape_test = 5;
                    if (tx_win_offset > initiator->conn_interval)
                    {
                        conn_win_size_offset = tx_win_offset - initiator->conn_interval;
                        tx_win_offset = initiator->conn_interval;
                        //dape_test = 6;
                        if (conn_win_size_offset >= initiator->tx_win_size)
                        {
                            conn_win_size_offset = initiator->tx_win_size - 1;
                            //dape_test = 8;
                        }
                    }
                }
            }
            else
            {
                /* next_anchor_point is dehind the tx_win. we have to change tx_win_offset */
                conn_win_size_offset = 1;
                tx_win_offset = initiator->tx_win_offset
                                + ((next_anchor_point - con_win_start_clk_slot) >> 1)
                                - conn_win_size_offset;
                //dape_test = 3;
                if (tx_win_offset > initiator->conn_interval)
                {
                    conn_win_size_offset = tx_win_offset - initiator->conn_interval;
                    tx_win_offset = initiator->conn_interval;
                    //dape_test = 4;
                    if (conn_win_size_offset >= initiator->tx_win_size)
                    {
                        conn_win_size_offset = initiator->tx_win_size - 1;
                        //dape_test = 7;
                    }
                }
            }
        }
    }
#endif
    /* set conn_win_offset */
    if (tx_win_offset_has_to_change)
    {
        WR_LE_REG(LE_REG_CONN_WIN_OFFSET, tx_win_offset);
        ll_manager.conn_unit.tx_win_offset = tx_win_offset;
    }
    ll_driver_fill_conn_win_size_offset(conn_win_size_offset);

    /*            RT_BT_LOG(BLUE, DAPE_TEST_LOG426, 9, dape_test, anchor_point_slot_max,
                          next_anchor_point_slot_min,
                          ce_interval_slot_min, conn_win_size_offset, tx_win_offset,
                          next_anchor_distance, next_anchor_point, ce_interval_same );

                RT_BT_LOG(BLUE, DAPE_TEST_LOG424, 9, num_le_conn, ll_manager.conn_unit.handle[0].anchor_point_slot,
                         ll_manager.conn_unit.handle[1].anchor_point_slot, cur_clock, cur_clock_slot, adv_hit_counter_n,
                         con_win_start_clk_slot, ll_manager.conn_unit.anchor_distance_min, anchor_point_slot_min);

                RT_BT_LOG(BLUE, DAPE_TEST_LOG428, 5, empty_entry, cur_clock_slot_offset,
                         ll_manager.conn_unit.first_slot_offset_for_le, next_anchor_point_offset, conn_intv_slot);
    */
}

SECTION_LE_ISR void ll_handle_hit_connected_adv_interrupt(void)
{
#ifdef _ROM_CODE_PATCHED_
    if (rcp_ll_hit_conn_adv_isr_func != NULL)
    {
        if (rcp_ll_hit_conn_adv_isr_func(NULL))
		{
            return;
		}
    }
#endif
#ifdef _DAPE_TEST_NEW_HW_BLOCK_ACL_FOR_LE
    ll_driver_block_legacy_slot_for_le(7);
    g_enable_le_block_legacy = 1;
#endif
#ifdef _CCH_SLOT_OFFSET_
#ifdef _DAPE_TEST_SLOT_OFFSET_FOR_LE
    if (ll_manager.conn_unit.connection_cnts == 0)
    {
        ll_handle_hit_connected_adv_int_no_link_policy();
        return;
    }
#endif
#endif

    ll_handle_hit_connected_adv_int_any_link_policy();
#ifdef _DAPE_PRINT_MORE_LOG_INFO
        LL_LOG_TRACE(YELLOW, DAPE_TEST_LOG545, 2,
                     RD_LE_REG(LE_REG_CONN_UPD_WIN_SIZE),
                            RD_LE_REG(LE_REG_CONN_WIN_OFFSET));
#endif
}
#ifdef _DAPE_EN_8821_MP_LE_SCAN_INTR
/**************************************************************************
 * Function     : ll_handle_scan_start_interrupt
 *
 * Description  : This function is used to handle ISR of scan start interrupt.
 *
 * Parameters   : type. BIT0:initiator; BIT1:scanner
 *
 * Returns      : None
 *
 *************************************************************************/
//// dape: why ISR has no prototype?
SECTION_LE_ISR void ll_handle_scan_start_interrupt(UINT8 type)
{
#ifdef _ROM_CODE_PATCHED_
    if (rcp_ll_handle_scan_start_intr_func != NULL)
    {
        if (rcp_ll_handle_scan_start_intr_func((void *)&type))
        {
            return;
        }
    }
#endif
    /* turn on timer 3 immediately. */
    timer_on_off(TIMER3_ID, 1, 1);
}
#endif

/* LE Controller Interrupt Priority (High -> Low):
   (1) conn_int = adv_int = scan_int > ce_end_int (in stack) > ce_early_int >
       rx_threshold = tx_ack_threshold
   (2) if rx_threshold is occurred, need check rx int stack to avoid acl
       rxfifo out of order */
/**************************************************************************
 * Function     : ll_interrupt_handler
 *
 * Description  : This function is used to handle ISR of LL hardware controller
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
SECTION_LE_ISR void ll_interrupt_handler(void)
{
    LE_REG_S_INT_MISR reg_misr;
#ifdef _DAPE_EN_8821_MP_LE_SCAN_INTR
    LE_REG_S_LE_EXT_MISR reg_ext_misr;

    *(UINT16*)&reg_ext_misr = RD_LE_REG(LE_REG_EXT_MISR);
#endif
    *(UINT16*)&reg_misr = RD_LE_REG(LE_REG_INT_MISR);

    g_le_event_end_flag_in_this_isr = FALSE;

#ifdef _ROM_CODE_PATCHED_
    if (rcp_ll_isr_func != NULL)
    {
#ifndef _DAPE_EN_8821_MP_LE_SCAN_INTR
        if (rcp_ll_isr_func((void *)&reg_misr))
#else
        if (rcp_ll_isr_func((void *)&reg_misr, &reg_ext_misr))
#endif
        {
            return;
        }
    }
#endif

    *(UINT16*)&reg_misr &= ~ll_manager.int_imr;

    /* handle advertising state interrupt if it is present */
    if (reg_misr.adv_int)
    {
        ll_handle_adv_interrupt();
    }

    /* handle the interrupt when hit the connected adv pdu if it is present */
    if (reg_misr.hit_conn_adv)
    {
        /* write one to clear interrupt */
        WR_LE_REG(LE_REG_INT_MISR, BIT15);

        /* TODO: check clock and calculate the offset of tx window */
        ll_handle_hit_connected_adv_interrupt();

#ifdef _DAPE_PRINT_MORE_LOG_INFO
        LL_LOG_TRACE(YELLOW, DAPE_TEST_LOG545, 2,
                     RD_LE_REG(LE_REG_CONN_UPD_WIN_SIZE),
                            RD_LE_REG(LE_REG_CONN_WIN_OFFSET));
#endif

#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_
        ll_driver_store_tx_resolvable_private_address();
#endif
    }

    /* handle connected or initiating state interrupt if it is present */
    if (reg_misr.conn_int)
    {
#ifdef _MODI_LPS_AFTER_RTL8821B_TC_
        g_lps_timer_counter = g_timer2_init_value;
#endif
        ll_handle_conn_interrupt();
    }

    /* handle AE or CE end interrupt if it is present */
    if (reg_misr.event_end_int)
    {

#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8821B_TC_RCP_
        if (rcp_ll_handle_event_end_interrupt != NULL)
        {
            rcp_ll_handle_event_end_interrupt();
        }
        else
#endif
#endif
        {

#ifdef _MODI_LPS_AFTER_RTL8703B_
            record_le_interrupt_status();
#endif
            ll_pop_all_rx_int_stack();
            ll_pre_cleanup_tx_schedule_of_event_end();
            ll_cleanup_rx_status();

#ifdef _MODI_LPS_AFTER_RTL8703B_
            ll_program_ae_sm_mode();
#endif
			g_le_event_end_flag_in_this_isr = TRUE;
        }
    }

    /* handle connection update interrupt in the ce end before updated instant */
    if (reg_misr.conn_updt_ce_e)
    {
        /* write one to clear interrupt */
        WR_LE_REG(LE_REG_INT_MISR, BIT11);
        ll_handle_update_interrupt(LL_UPDATE_REASON_STOP_SUPERVISION_TIMER);
    }

    /* handle connection update interrupt in the start of tx window */
    if (reg_misr.conn_updt_tw_s)
    {
        /* write one to clear interrupt */
        WR_LE_REG(LE_REG_INT_MISR, BIT13);
        ll_handle_update_interrupt(LL_UPDATE_REASON_CONN_PARAM);
    }

    /* handle channel map update interrupt in the ce start if it is present */
    if (reg_misr.chm_updt_ce_s)
    {
        /* write one to clear interrupt */
        WR_LE_REG(LE_REG_INT_MISR, BIT12);
        ll_handle_update_interrupt(LL_UPDATE_REASON_CH_MAP);

#ifdef _DAPE_NO_TRX_WHEN_LE_CONN_UPDT
        if (IS_TIMER3_FOR_LE_DUAL_WHCK)
        {
            timer_on_off(TIMER3_ID, 0, 0);
            bb_switch_scheduler_to_legacy();
            RT_BT_LOG(BLUE, DAPE_TEST_LOG207, 1,BB_read_native_clock());
        }
#endif
    }

    /* handle AE or CE early interrupt before anchor point if it is present */
    if (reg_misr.event_begin_int)
    {
        ll_handle_event_begin_early_interrupt();
    }

    /* handle the threshold of tx ack interrupt if it is present */
    if (reg_misr.tx_thr_int)
    {
        ll_handle_tx_ack_threshold_interrupt(ll_manager.event_manager.entry);
    }

    /* handle the threshold of rx fifo interrupt if it is present */
    if (reg_misr.rx_th_int)
    {
        ll_handle_rx_threshold_interrupt(ll_manager.event_manager.entry);
    }

    /* handle tx packet done interrupt if it is present */
    if (reg_misr.pkt_tx)
    {
        /* write one to clear interrupt */
        WR_LE_REG(LE_REG_INT_MISR, BIT3);
    }

    /* handle rx packet done interrupt if it is present */
    if (reg_misr.pkt_rx)
    {
        /* write one to clear interrupt */
        WR_LE_REG(LE_REG_INT_MISR, BIT2);
    }

    /* handle scanning state interrupt if it is present */
    if (reg_misr.scan_int)
    {
        /* write one to clear interrupt */
        WR_LE_REG(LE_REG_INT_MISR, BIT7);
        ll_handle_scan_interrupt();
    }

#ifdef _DAPE_EN_8821_MP_LE_SCAN_INTR
    if (reg_ext_misr.conn_early)
    {
        /* write one to clear interrupt */
        WR_LE_REG(LE_REG_EXT_MISR, ll_manager.ext_int_imr | BIT0);

        ll_handle_scan_start_interrupt(BIT0);
        /*RT_BT_LOG(WHITE, DAPE_TEST_LOG525, 6,
        BB_read_native_clock(),
                    ll_manager.initiator_unit.enable,
                    ll_manager.initiator_unit.scan_interval,
                    ll_manager.initiator_unit.scan_window,
                    ll_manager.ext_int_imr,
                    *(UINT16*)&reg_ext_misr);*/
    }

    if (reg_ext_misr.scan_early)
    {
        /* write one to clear interrupt */
        WR_LE_REG(LE_REG_EXT_MISR, ll_manager.ext_int_imr | BIT1);

        ll_handle_scan_start_interrupt(BIT1);

        /*RT_BT_LOG(BLUE, DAPE_TEST_LOG525, 6,
        BB_read_native_clock(),
                        ll_manager.scan_unit.enable,
                    ll_manager.scan_unit.interval,
                    ll_manager.scan_unit.window,
                        RD_LE_REG(LE_REG_SCAN_INTERVAL),
                    *(UINT16*)&reg_ext_misr);*/
    }

#endif

#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_
    if (reg_ext_misr.scan_req)
    {
        /* write one to clear interrupt */
        WR_LE_REG(LE_REG_EXT_MISR, ll_manager.ext_int_imr | BIT5);

        ll_driver_store_tx_resolvable_private_address();
    }
#endif /* _BT4_2_PRIVACY_FEATURE_SUPPORT_ */

}
#ifdef _MODI_LPS_AFTER_RTL8703B_
void record_le_interrupt_status()
{
    // Need read out before 0xCC (hw read out and clear)
    // Read 0xC0 Bit[14:13]: 01:scan window end 10: initiator window end 11: adv window end
    // use this to minus window and add interval to get next window start
    sleep_mode_param.le_interrupt_status = RD_LE_REG(LE_REG_STATUS_ADV_TIMEOUT_CTRL);
}

void ll_program_ae_sm_mode()
{
    FW_BT_RX_INT_UNIT_S *punit;
    UINT8 rptr = fw_bt_rx_int_fifo.rptr;
    punit = &fw_bt_rx_int_fifo.unit[rptr];

    if(g_lps_timer_counter >0)
    {
        return;
    }

    if(lc_check_lps_for_link(0,1,0) == FALSE)
    {
        return;
    }

   /*
    * Include Scan/Initiator/Adv Window end interrupt
    */
    if (punit->ce_end_status.ae_end)
    {
        UINT16 reg_u16 = sleep_mode_param.le_interrupt_status;

        if( (reg_u16&0x6000) == 0x2000 )    // 01:scan window
        {
            sleep_mode_param.scan_end_flag |= BIT3;
        }

        if( (reg_u16&0x6000) == 0x6000 )    // 11: adv window
        {
#ifdef _MODI_LPS_AFTER_RTL8703B_
            sleep_mode_param.le_adv_count ++;

            if(sleep_mode_param.le_adv_count < sleep_mode_param.le_adv_num)
            {
                return;
            }
#endif
            sleep_mode_param.scan_end_flag |= BIT2;
        }

        if(( (reg_u16&0x6000) == 0x4000 )&&(ll_manager.initiator_unit.enable))
        {

            UINT32 ps_clock = BB_read_native_clock();
            UINT16 ps_duration = (ll_manager.initiator_unit.scan_interval)- (ll_manager.initiator_unit.scan_window);
            ps_clock = (ps_clock>>1) + ps_duration;

            if(g_efuse_lps_setting_4.lps_use_state)
            {
                lps_period_state_machine_fast(ps_clock, ps_duration);
            }
        }
#ifdef _MODI_LPS_STATE_WITH_INTR_
        else
        {
            if(g_efuse_rsvd_2.lps_state_with_intr)
            {
                sleep_mode_param.lps_lpm_lps_mode = LPS_SNIFF_DSM;
                lc_post_lps_stste_signal(LPS_SNIFF_DSM);
            }
            else
            {
                if( lc_check_lps_for_idle(1))
                {
                    UINT8 state = sleep_mode_param.lps_period_state;
                    UINT8 bitmap = sleep_mode_param.lps_period_state_bitmap;
                    UINT8 count = sleep_mode_param.lps_period_count;
                    if(!sleep_mode_param.lps_task_flag)		 // 20110310
                    {
                        lps_period_state_machine(state, bitmap, count, LPS_TIMER2_WITH_SCAN, 0, 0);
                    }
                }

             }

        }

#endif
    }
    return;
}
#endif
