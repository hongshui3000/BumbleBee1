/***************************************************************************
Copyright (C) MindTree Ltd.
This module is a confidential and proprietary property of MindTree and
a possession or use of this module requires written permission of MindTree.
***************************************************************************/

/**
* \file
*  Contains the implementation of LC Rx Task and LC Tx Task. It also has the
*  signal handlers of the respective tasks.
*/

/********************************* Logger *************************/
enum { __FILE_NUM__= 42 };
/********************************* Logger *************************/


/* ========================= Include File Section ========================= */
#include "lc_internal.h"
#include "bt_fw_hci.h"
#include "lmp.h"
#include "lmp_internal.h"
#include "bt_fw_os.h"
#include "bt_fw_globals.h"
#include "vendor.h"
#include "lc.h"
#include "led_debug.h"
#include "hci_td.h"
#include "bz_debug.h"
#include "mem.h"
#include "UartPrintf.h"
#include "lc_2_1.h"
#include "lmp_2_1.h"
#include "bz_auth_internal_2_1.h"

#ifdef PTA_EXTENSION
#include "pta_meter.h"
#endif

#ifdef SCO_OVER_HCI
#include "bz_fw_isoch.h"
#endif//#ifdef SCO_OVER_HCI

#include "bt_fw_acl_q.h"
#include "lmp_pdu_q.h"
#include "bzdma.h"

#ifdef LE_MODE_EN
#include "le_ll.h"
#include "le_hci_4_0.h"
#include "le_hw_reg.h"
#include "le_ll_driver.h"
#endif

#ifdef _RESET_HW_LINK_INFO_TO_INIT_STATE_
#include "bb_driver.h"
#include "bz_auth.h"
#include "bz_auth_internal.h"
#endif

#include "gpio.h"

#ifdef _3DD_FUNCTION_SUPPORT_
#include "bt_3dd.h"
#include "hci_vendor_defines.h"
#endif

#ifdef _DAPE_ACL_PRI_HIGHER_THAN_INQ_WHEN_BR
extern UCHAR g_acl_priority_high_than_inq;
#endif
#ifdef _DAPE_TEST_FOR_HID_SCO
extern UINT8 lc_sco_pause_status;
#endif
#ifndef _ENABLE_COMPILE_NESTED_PAUSE_RESUME_
extern UCHAR lc_pause_schedule_pkts;
#endif
extern HCI_ACL_DATA_PKT *acl_q_head;

#ifdef _DAPE_TEST_NEW_HW_BLOCK_ACL_FOR_LE
extern UINT8 g_enable_le_block_legacy;
extern UINT8 lc_le_pause_status;
extern UINT8 g_block_legacy_for_le_slot_num;
#endif
#ifdef _DAPE_TEST_SEND_ZERO_LEN_PKT_BY_VENDOR_CMD
extern UINT8 g_send_zero_len;
#endif

#ifdef _BRUCE_FIX_DROP_ZERO_LEN_DM1_WHEN_KEEP_RETRY
UINT8 temp_lut_id[4] = {0};
#endif

#ifdef _LE_SPEED_UP_HCI_ACL_TX_PKT_HANDLE_
extern LL_HCI_ACL_DATA_PKT *pll_hci_pre_pend_list_head;
extern LL_HCI_ACL_DATA_PKT *pll_hci_pre_pend_list_tail;
#endif

extern OS_HANDLE isr_extended_task_handle;
/**PATCH_PENDING_ENCRY_WHILE_ROLE_SWITCH*/
#ifdef _PENDING_ENCRY_WHILE_ROLE_SWITCH
extern UINT8 g_need_pend_encry;
extern UINT8 g_encry_pending;
extern OS_SIGNAL g_signal_ptr_encry;
#endif
#ifndef _RTL8723B_DUT_MODE_
/* from bit 0 to bit 510 is a cycle of pseudo-random sequence */
const UINT8 ALIGN(4) prbs9_bite_stream_bytes[64 + 4] = {
    0xff, 0xc1, 0xfb, 0xe8,
    0x4c, 0x90, 0x72, 0x8b,
    0xe7, 0xb3, 0x51, 0x89,
    0x63, 0xab, 0x23, 0x23,
    0x02, 0x84, 0x18, 0x72,
    0xaa, 0x61, 0x2f, 0x3b,
    0x51, 0xa8, 0xe5, 0x37,
    0x49, 0xfb, 0xc9, 0xca,
    0x0c, 0x18, 0x53, 0x2c,
    0xfd, 0x45, 0xe3, 0x9a,
    0xe6, 0xf1, 0x5d, 0xb0,
    0xb6, 0x1b, 0xb4, 0xbe,
    0x2a, 0x50, 0xea, 0xe9,
    0x0e, 0x9c, 0x4b, 0x5e,
    0x57, 0x24, 0xcc, 0xa1,
    0xb7, 0x59, 0xb8, 0x07,
    0x00, 0x00, 0x00, 0x00
};
#endif

/* ================== Static Function Prototypes Section ================== */
extern UCHAR lmp_calculate_sniff_subrate(UINT16 ce_index);
extern void lc_check_and_cleanup_bd_addr_regs_on_disc(UCHAR phy_piconet_id);
#ifdef  COMPILE_ROLE_SWITCH
void lc_handle_role_switch_complete_in_scatternet(void);
#endif

void lc_handle_and_check_sw_scheduler(UINT8 phy_piconet_id);

/* ================== For rom code patch function point ================== */
#ifdef _ROM_CODE_PATCHED_
#ifdef _DAPE_8723_RCP_
PF_ROM_CODE_PATCH_FUNC rcp_lc_kill_scan_status_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_lc_start_page_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_lc_start_inq_func = NULL;
#endif
#endif
/* ============== Global or Extern Variable Prototypes Section ============== */
#if defined(TEST_MODE)
extern UINT8 test_mode_sched_acl_pkt;
extern UINT8 test_mode_tx_mode_force_ack;
extern UINT8 test_mode_tx_mode_buf_in_used;
extern UINT8 test_mode_rx_int_cnt;
extern UINT8 test_mode_tx_int_cnt;

#ifdef _DUT_DELAYED_LOOPBACK_MODE_
extern DUT_MODE_MANAGER dut_mode_manager;
#endif

#ifdef _RTL8723B_DUT_MODE_
extern UINT16 test_mode_tx_mode_reg_catch;
#endif
#endif
#ifdef _CSB_RX_DBG_LOG
extern UINT8 g_csb_rx_dbg_log;
#endif
LMP_FHS_PKT_RECD  *prev_fhs_ptr_for_eir = NULL;

/* ===================== Function Definition Section ====================== */
/**
* Services the singal posted to lc-tx-task.
*
* \param signal_ptr Pointer to the signal being serviced.
*
* \return None.
*/
void lc_tx_task(OS_SIGNAL *signal_ptr)
{
    UCHAR   schd_read_index;
    UINT16 ce_index;
    UINT32 temp32;
    UCHAR phy_piconet_id;

    switch(signal_ptr->type)
    {
        case SIGNAL_BEGIN:
            break;

        case SIGNAL_END :
            lc_shutdown();
            break;

#ifdef COMPILE_SNIFF_MODE
        case LC_HANDLE_OPTIMIZE_DATA_TX_DURING_SNIFF:
            ce_index = (UCHAR)((UINT32) signal_ptr->param);
            lc_optimize_data_transfer_during_sniff(ce_index);
            break;
#endif

        case LC_HANDLE_ACK_RECD_SIGNAL:
            schd_read_index = (UCHAR)((UINT32) signal_ptr->param);
            phy_piconet_id = (UCHAR) (((UINT32)signal_ptr->ext_param) >> 30);
            temp32 =  ((UINT32)signal_ptr->ext_param) & 0x0fffffff;
            lc_handle_ack_received(schd_read_index, phy_piconet_id, temp32);
            break;

        case LC_HANDLE_START_SCAN_MODE_SIGNAL:
            lc_start_write_scan_mode(lmp_self_device_data.scan_enable);
            break;

        case LC_HANDLE_HOST_DATA_PKT:
            {
#ifdef _MONITOR_ACLU_TRX_THROUGHPUT_
                HCI_ACL_DATA_PKT *acl_tail = acl_q_tail;
#endif
                HCI_ACL_DATA_PKT *acl_pkt;
                acl_pkt = (HCI_ACL_DATA_PKT *)signal_ptr->param;
                lc_handle_host_data_pkt(acl_pkt);

#ifdef _MONITOR_ACLU_TRX_THROUGHPUT_
                if (acl_tail != acl_q_tail)
                {
                    /* this means we enqueue one new hci acl packet */
                    UINT16 handle = acl_pkt->connection_handle;
                    hci_conn_handle_aclu_measure[handle - 1] +=
                                            acl_pkt->acl_data_total_length;
                }
#endif
            }
            break;

#ifdef LE_MODE_EN
        case LL_HANDLE_HOST_DATA_PKT:
#ifdef _LE_SPEED_UP_HCI_ACL_TX_PKT_HANDLE_
            {
                LL_HCI_ACL_DATA_PKT *ppkt_list;
                LL_HCI_ACL_DATA_PKT *ppkt;

            	DEF_CRITICAL_SECTION_STORAGE;

                MINT_OS_ENTER_CRITICAL();
                ppkt_list = pll_hci_pre_pend_list_head;
                pll_hci_pre_pend_list_head = NULL;
                pll_hci_pre_pend_list_tail = NULL;
                MINT_OS_EXIT_CRITICAL();

                while (ppkt_list != NULL)
                {
                    /* cut the node */
                    ppkt = ppkt_list;
                    ppkt_list = ppkt_list->next;
                    ppkt->next = NULL;

                    /* process the ll hci packet */
                    ll_handle_host_data_pkt(ppkt);
                }
            }
#else
            ll_handle_host_data_pkt((LL_HCI_ACL_DATA_PKT *)signal_ptr->param);
#endif
            break;
#endif

#if defined(COMPILE_ESCO) || defined(SCO_OVER_HCI)
        case LC_HANDLE_HOST_SYNCHRONOUS_PKT:
            {
                HCI_SYNC_DATA_PKT *synchronous_pkt;
                synchronous_pkt = (HCI_SYNC_DATA_PKT*)signal_ptr->param;
                lc_handle_synchronous_data_pkt(synchronous_pkt);
            }
            break;
#endif /* COMPILE_ESCO || SCO_OVER_HCI */

        case LC_RESUME_DATA_TRANSFER_SIGNAL:
            /* Invoke all piconets: Check For SCATTERNET */
            phy_piconet_id = (UCHAR) (UINT32)((void*)signal_ptr->param);
            if (phy_piconet_id == SCA_PICONET_INVALID)
            {
                UINT8 i;
                for (i = 0; i <= SCA_PICONET_MAX; i++)
                {
                    if (lc_sca_manager.pnet[i].active)
                    {
                        /* only resume active piconets */
                        lc_invoke_scheduler(i);
                    }
                }
            }
            else
            {
                lc_invoke_scheduler(phy_piconet_id);
            }
            break;

        case LC_HANDLE_RESCHEDULE:
            phy_piconet_id = (UCHAR)((UINT32)signal_ptr->param);
            temp32 = (UINT32)signal_ptr->ext_param;
            lc_reschedule(phy_piconet_id, (UCHAR)temp32);
            break;

        case LC_HANDLE_MARK_ACTIVE:
            {
                UCHAR am_addr;

                am_addr = (UCHAR)( (UINT32)(signal_ptr->param) );
                phy_piconet_id = (UCHAR) (UINT32) ((void*)(signal_ptr->ext_param));

                aclq_mark_am_addr_as_active(am_addr, phy_piconet_id);
                if (lc_piconet_scheduler[phy_piconet_id].lc_allowed_pkt_cnt)
                {
                    lc_invoke_scheduler(phy_piconet_id);
                }
            }
            break;

        case LC_QUEUE_PDU_SIGNAL:
#ifdef _PENDING_ENCRY_WHILE_ROLE_SWITCH
            {
                LMP_PDU_PKT* lmp_pkt;
                UINT8 lmp_opcode;
                UINT8 lmp_opcode2;

                lmp_pkt = (LMP_PDU_PKT*)signal_ptr->param;
                lmp_opcode = lmp_pkt->payload_content[0] >> 1;
                lmp_opcode2 = lmp_pkt->payload_content[1];

                if((lmp_opcode == LMP_ENCRYPTION_MODE_REQ_OPCODE) && (g_need_pend_encry == TRUE))
                {
                    pduq_dequeue_pdu(lmp_pkt);
                    memcpy(&g_signal_ptr_encry, signal_ptr, sizeof(OS_SIGNAL));
                    g_encry_pending = TRUE;
                    return;
                }
                if((lmp_opcode == LMP_SLOT_OFFSET_OPCODE) || (lmp_opcode == LMP_SWITCH_REQ_OPCODE))
                {
                    //need to role swtich, pendig encry
                    g_need_pend_encry = TRUE;
                }
                if((lmp_opcode == LMP_NOT_ACCEPTED_OPCODE) && (lmp_opcode2 == LMP_SWITCH_REQ_OPCODE) \
                    && (g_need_pend_encry == TRUE))
                {
					phy_piconet_id = (UCHAR)((UINT32)(signal_ptr->ext_param));
                    lc_handle_and_check_sw_scheduler(phy_piconet_id);
                    g_need_pend_encry = FALSE;
                    //encry mode
                    if (g_encry_pending)
                    {
                        LMP_PDU_PKT* pdu_pkt;
                        pdu_pkt = (LMP_PDU_PKT *)g_signal_ptr_encry.param;
                        pduq_queue_pdu(pdu_pkt, pdu_pkt ->piconet,  pdu_pkt->ce_index);
						phy_piconet_id = (UCHAR)((UINT32)(g_signal_ptr_encry.ext_param));
						lc_handle_and_check_sw_scheduler(phy_piconet_id);
                        g_encry_pending = FALSE;
                    }
                    return;
                }
            }
#endif
            phy_piconet_id = (UCHAR)((UINT32)(signal_ptr->ext_param));
            lc_handle_and_check_sw_scheduler(phy_piconet_id);
            break;

#ifdef COMPILE_DYNAMIC_POLLING
        case LC_DYNAMIC_PROGRAM_TPOLL:
            ce_index = (UCHAR)((UINT32)(signal_ptr->param));
            lc_program_dynamic_tpoll(ce_index);
            break;
#endif

#ifdef COMPILE_ROLE_SWITCH
        case LC_SWITCH_ROLE_COMPLETE_SIGNAL:
            lmp_handle_role_change_complete();
            break;
#endif

        case LC_PDU_SENT_SIGNAL:
            lmp_handle_pdu_sent((LMP_PDU_PKT *)signal_ptr->param);
            break;

        default:
#ifdef ENABLE_LOGGER_LEVEL_2
            LC_LOG_ERROR(LOG_LEVEL_LOW, LC_TX_TASK_INVALID_SIGNAL_RECEIVED,0,0);
#endif

            break;
    }/* end switch */

    return;
}

/**
* Services the singal posted to lc-rx-task.
*
* \param signal_ptr Pointer to the singal being serviced.
*
* \return None.
*/
void lc_rx_task(OS_SIGNAL *signal_ptr)
{
#ifdef COMPILE_ESCO
    UINT16 packet_length;
    UINT16 status;
#endif

    switch (signal_ptr->type)
    {
    case SIGNAL_BEGIN:
        break;

    case SIGNAL_END:
        break;

    case LC_HLC_SIGNAL:
        lc_handle_lc_hlc_signal(
            (UCHAR)((UINT32)signal_ptr->param),
            (UCHAR)((UINT32)signal_ptr->ext_param) );
        break;

    case LC_HANDLE_RECD_PACKET_SIGNAL:
        if (lc_handle_received_packet_in_scatternet())
        {
            if ((UCHAR)((UINT32)signal_ptr->param))
            {
                os_free_reserved_buffer();
            }
        }
        break;

#ifdef SCO_OVER_HCI
    case LC_HANDLE_RX_SCO_DATA_SIGNAL:
        lc_handle_sco_rx_packet((int)signal_ptr->param);
        break;

    case LC_HANDLE_RX_SCO_ERR_DATA_SIGNAL:
    {
        UINT16 sco_ce_index = (UINT16)((UINT32)signal_ptr->param);
        UCHAR  data_status = (UCHAR)((UINT32)signal_ptr->ext_param);

        bz_isoch_send_data_to_host(
            lmp_sco_connection_data[sco_ce_index].sco_conn_handle,
            lmp_sco_connection_data[sco_ce_index].erroneous_data_reporting,
            lmp_sco_connection_data[sco_ce_index].fifo_num,
            lmp_sco_connection_data[sco_ce_index].pkt_length,
            data_status);
    }
    break;

#endif /* SCO_OVER_HCI */

#ifdef COMPILE_ESCO
    case LC_HANDLE_ESCO_PACKET :
        packet_length = (UINT16)((UINT32)signal_ptr->param);
        status = (UINT16)((UINT32)signal_ptr->ext_param);
        lc_handle_esco_rx_packet(packet_length,status);

#ifdef _BRUCE_DEBUG_PORT
        UINT16 read;
        read=BB_read_baseband_register(0x25c); // 0x25c[2]=0; g_plc_pkt_miss
        read &= (UINT16)(~BIT2);
        BB_write_baseband_register(0x25c,read);
#endif
#ifdef _BRUCE_DEBUG_PORT
        read=BB_read_baseband_register(0x25c); // 0x25c[0]=0; g_plc_crc_err
        read &=(UINT16)(~BIT0);
        BB_write_baseband_register(0x25c,read);
#endif

        break;
#endif

#ifdef COMPILE_SNIFF_MODE
    case LC_EXIT_SSR:
    {
        UINT16 ce_index = (UINT16)((UINT32)signal_ptr->param);
        if (lmp_connection_entity[ce_index].ssr_data.lc_ssr_state ==
                LC_SSR_SUBRATE)
        {
            LC_LOG_INFO(LOG_LEVEL_HIGH,EXITING_SSR_MODE_BB_LMP,0,0);
            lc_exit_ssr_mode(ce_index);

            /* Disabling SSR */
            if (lmp_connection_entity[ce_index].ssr_data.lmp_ssr_state
                    == LMP_SSR_IDLE)
            {
                /* Reset SSR parameters */
                lmp_reset_ssr_parameters(ce_index);
            }
        }
    }
    break;

    case LC_ENTER_SSR:
    {
        UINT16 ce_index = (UINT16)((UINT32)signal_ptr->param);

        SSR_DATA *ssr_data_ptr;

        ssr_data_ptr = &lmp_connection_entity[ce_index].ssr_data;

        if (ssr_data_ptr->lc_ssr_state != LC_SSR_SUBRATE &&
                ssr_data_ptr->lmp_ssr_state != LMP_SSR_TRANSITION &&
                ssr_data_ptr->lmp_ssr_state != LMP_SSR_IDLE)
        {
            LC_LOG_INFO(LOG_LEVEL_HIGH,ENTERING_SSR_MODE_BB,0,0);
            lc_enter_ssr_mode(ce_index);
        }
    }
    break;

    case LC_LMP_ENTER_SSR:
    {
        UINT16 ce_index = (UINT16)((UINT32)signal_ptr->param);

        SSR_DATA *ssr_data_ptr;

        ssr_data_ptr = &lmp_connection_entity[ce_index].ssr_data;

        if (ssr_data_ptr->lc_ssr_state != LC_SSR_SUBRATE)
        {
            RT_BT_LOG(GRAY, LC_TASK_426, 0, 0);

            /**
            * State is changed after changing the prev_pkt_time.
            * This is to avoid use of critical section.
            */
            if (ssr_data_ptr->prev_pkt_time == FORCE_FLUSH_CLOCK)
            {
                ssr_data_ptr->prev_pkt_time =
                    ssr_data_ptr->ssr_instant;
            }

            ssr_data_ptr->lmp_ssr_state = LMP_SSR_ACTIVE;
        }
    }
    break;
#endif

#ifdef POWER_SAVE_FEATURE
#ifndef _REDUCE_LPS_AFTER_RTL8703B_
    case LC_ENTER_SM_MODE:
    {
        UINT32 wakeup_instant = (UINT32)(signal_ptr->param);
        UCHAR piconet_id = (UCHAR)(UINT32)(signal_ptr->ext_param);
        LC_PROGRAM_SM_MODE(wakeup_instant, piconet_id);
    }
    break;
    case LC_ENTER_DSM_MODE:
    {
        UINT32 wakeup_instant = (UINT32)(signal_ptr->param);
        UCHAR piconet_id = (UCHAR)(UINT32)(signal_ptr->ext_param);
        LC_PROGRAM_DSM_MODE(wakeup_instant, piconet_id);
    }
    break;
#endif
#if defined(_CCH_LPS_) && defined(_YL_LPS)
    case LC_ENTER_LPS_MODE:
    {
#ifdef _REDUCE_USED_STACK_SIZE_IN_DLPS_MODE_
        /* post the handler to another task to reduce stack size */
        OS_SIGNAL signal;
        memcpy(&signal, signal_ptr, sizeof(OS_SIGNAL));
        signal.type = ISR_EXT_ENTER_LPS_MODE;
        OS_ISR_SEND_SIGNAL_TO_TASK(isr_extended_task_handle, signal);
#else
        UINT32 wakeup_instant = (UINT32)(signal_ptr->param);
        UCHAR lps_mode =  (UCHAR)((UINT32)(signal_ptr->ext_param) >> 8);
        UCHAR piconet_id = (UCHAR)((UINT32)(signal_ptr->ext_param) & 0xff);
        UINT8 ret_value;
        ret_value = LC_PROGRAM_LPS_MODE(lps_mode, wakeup_instant, piconet_id);
        if( ret_value != API_SUCCESS)
        {
#ifdef _MODI_LPS_AFTER_RTL8821B_TC_

            if( ret_value == API_FAILURE)
            {
                lc_check_lps_for_resume();
            }
#else
            lc_check_and_enable_scans_in_scatternet();
#endif
        }
#endif
    }
    break;
#endif

#ifdef _MODI_LPS_STATE_WITH_INTR_
    case LC_ENTER_LPS_STATE:
    {
        UINT8 state = sleep_mode_param.lps_period_state;
        UINT8 bitmap = sleep_mode_param.lps_period_state_bitmap;
        UINT8 count = sleep_mode_param.lps_period_count;
        UINT32 wakeup_instance = sleep_mode_param.lps_lpm_wakeup_instance;

        lps_period_state_machine(state, bitmap, count, LPS_TIMER2_WITH_SCAN, 0, wakeup_instance);
    }
    break;
#endif
#endif /* POWER_SAVE_FEATURE */

#ifdef COMPILE_ROLE_SWITCH
    case LC_SWITCH_ROLE_WAIT_TO_SIGNAL:
        //LC_LOG_INFO(LOG_LEVEL_HIGH, "One shot timer fired");
        lmp_check_and_switch_role_in_scatternet();
        break;
#endif

#ifdef LE_MODE_EN
    case LL_HANDLE_RECD_PACKET_SIGNAL:
        ll_handle_rxfifo_in_task((void*)&signal_ptr->param);
        break;
#endif

    default :
#ifdef ENABLE_LOGGER_LEVEL_2
        LC_LOG_ERROR(LOG_LEVEL_LOW, LC_RX_TASK_INVALID_SIGNAL_RECD,0,0);
#endif
        break;

    }
}


/**
* Handles the hardware level connection. Also starts the supervision
* timeout timer.
*
* \param am_addr Logical transport address of the connection.
* \param piconet_id The piconet ID of the connection.
*
* \return None.
*/
void lc_handle_lc_hlc_signal(UCHAR am_addr, UCHAR phy_piconet_id)
{
    OS_SIGNAL sig_send;
    UINT16 ce_index;
    LMP_CONNECTION_ENTITY *ce_ptr;

    //RT_BT_LOG(GRAY, LC_TASK_497, 0, 0);

    /* Generate LMP_HARDWARE_LVL_CONN_RECD_SIGNAL for lmp_task */
    sig_send.type = LMP_HARDWARE_LVL_CONN_RECD_SIGNAL;
    sig_send.param = (OS_ADDRESS)((UINT32)am_addr);
    sig_send.ext_param = (OS_ADDRESS)((UINT32)(phy_piconet_id));

    OS_SEND_SIGNAL_TO_TASK (lmp_task_handle, sig_send);

    if (LMP_GET_CE_INDEX_FROM_AM_ADDR_PPI(am_addr,
                                         (UINT16)((UINT32) phy_piconet_id),
                                         &ce_index) != API_SUCCESS)
    {
        RT_BT_LOG(GRAY, LC_TASK_516, 1, am_addr);
        return;
    }

    ce_ptr = &lmp_connection_entity[ce_index];

    /* Do not enable supervision timer if this is bt_3dd_ce_index*/
#ifdef _SUPPORT_CSB_RECEIVER_
    if (!((bt_3dd_var.csb_rx_param.enable) &&
         ce_index == (bt_3dd_var.csb_rx_param.ce_index)))
#endif
    {
    if (ce_ptr->link_supervision_timeout != 0)
    {
        if (OS_START_TIMER(ce_ptr->supervision_timeout_handle,
                          (UINT16)(SLOT_VAL_TO_TIMER_VAL(LMP_SUPERVISION_TIMER_RESOLUTION)))
                != BT_ERROR_OK)
        {
#ifdef ENABLE_LOGGER_LEVEL_2
            LC_LOG_ERROR(LOG_LEVEL_LOW, OS_START_TIMER_FAILED_N,0,0);
#endif
        }
    }
    }
    ce_ptr->supervision_timeout_flag = TRUE;

    //stop_flag = 1;
    lc_check_and_enable_scans_in_scatternet();

#ifdef COMPILE_DYNAMIC_POLLING
    lc_update_dynamic_polling();
#endif

    //RT_BT_LOG(GRAY, LC_TASK_562, 0, 0);

#ifdef ENABLE_LOGGER
    UINT8 lc_no_of_conns = 0;
    UINT8 mid = 0;

    if (lc_sca_manager.master_cnt)
    {
        mid = lc_sca_manager.master_id;
        lc_no_of_conns = lmp_self_device_data.lc_no_of_connections[mid];
    }

    RT_BT_LOG(GRAY, MSG_LC_CREATE_HLC, 12, lc_sca_manager.master_cnt,
              mid, lc_no_of_conns, lc_sca_manager.bm_slave,
              phy_piconet_id, am_addr,
              ce_ptr->bd_addr[5], ce_ptr->bd_addr[4], ce_ptr->bd_addr[3],
              ce_ptr->bd_addr[2], ce_ptr->bd_addr[1], ce_ptr->bd_addr[0]);
#endif

    return;
}


/**
* Kills the hardware level connection. If the connection is in sniff mode,
* it exits to active. If the scheduler already has some packets for this
* connection, then these packets will be be flushed from the baseband and
* removed from the scheduler. Also disables AFH for the connection. Stops
* the tpoll timer if the device is master. Checks and retrieves scans.
*
* \param am_addr Logical transport address of the connection
*        piconet_id : Physical piconet ID of the packet.
*
* \return None.
*/
void lc_kill_hardware_level_conn_in_scatternet(UINT16 ce_index)
{
    UCHAR lut_index;
    UINT16 read;
    UCHAR num_of_acl_conn;
#ifdef TEST_MODE
    UINT16 sco_pkt_type_reg;
#endif
    UCHAR* no_of_con;
    unsigned int i;
    volatile UINT32 new_master_clock, master_clock;
    LMP_CONNECTION_ENTITY *ce_ptr;

    UCHAR phy_piconet_id;
    UCHAR am_addr;

    DEF_CRITICAL_SECTION_STORAGE;

    ce_ptr = &lmp_connection_entity[ce_index];
    phy_piconet_id = ce_ptr->phy_piconet_id;
    am_addr = ce_ptr->am_addr;

    lut_index = lc_get_lut_index_from_phy_piconet_id(
                    am_addr, phy_piconet_id);

    LC_EXIT_SM_MODE();

    BB_write_baseband_register(
        lut_ex_table[lut_index].lower_lut_address, LC_INVALID_PACKET_TYPE);

#ifdef COMPILE_SNIFF_MODE
    /* Kill Sniff before killing HLC, LC need not be sure that
    * the baseband is in sniff mode here.
    */
    if (ce_ptr->in_sniff_mode == TRUE)
    {
        ce_ptr->in_sniff_mode = FALSE;

        lc_exit_sniff_mode(ce_index);

        lc_sniff_cont_count[lut_index - 1] = 0;

#ifdef DETECT_SNIFF_OVERLAP
        if (lmp_self_device_data.no_of_sniff_connections != 0)
        {
            lmp_self_device_data.no_of_sniff_connections--;
        }
#endif
    }
#endif

    BB_stop_tpoll(am_addr, phy_piconet_id);

    if (lmp_self_device_data.number_of_hlc > 0)
    {
        lmp_self_device_data.number_of_hlc--;
    }
    else
    {
        BZ_ASSERT(0, "No of HLC is 0.");
    }

    num_of_acl_conn = lmp_self_device_data.number_of_hlc;

#ifdef COMPILE_PARK_MODE
    if ((lmp_self_device_data.number_of_parked_dev == 0) &&
            (num_of_acl_conn == 0))
#else /* COMPILE_PARK_MODE */
    if (num_of_acl_conn == 0)
#endif /* COMPILE_PARK_MODE */
    {
        /* Spin for clock 2 transition - multislot packets not considered */
        lc_get_clock_in_scatternet((UINT32*)&master_clock, phy_piconet_id);
    }

    master_clock >>= 2;

    new_master_clock = 0;

    i = 0;

    while( (master_clock + 1) >= new_master_clock)
    {
        //Marc add, for compiler
        if (i++ >= 10)
        {
            break;
        }
        lc_get_clock_in_scatternet((UINT32*)&new_master_clock, phy_piconet_id);
        new_master_clock >>= 2;
    }

#ifdef COMPILE_AFH_HOP_KERNEL
    BB_disable_afh(am_addr, phy_piconet_id);
#endif

    BB_encryption_control(am_addr, phy_piconet_id, BB_ENC_TX_DISBALE_RX_DISABLE);

    /* De-activate the Upper LUT for this am address ,
    * The effect is the same as killing hardware level cnnection
    * Kill HLC cannot be instructed to Bseband because we cant be sure
    * as to which piconet we are in.
    */
    read = BB_read_baseband_register(lut_ex_table[lut_index].upper_lut_address);

    /* Now clear the active bit. */
    read &= ~ACTIVE_BIT;

    /* Initialize arqn = 0 */
    read &= ~0x0004;

    /* Initialize seqn = 0 */
    read &= ~0x0002;

    /* Initialize flow = 1 */
    read |= 0x0001;

#ifdef _RESET_HW_LINK_INFO_TO_INIT_STATE_
    /* Initialize ptt = 0 */
    read &= ~0x8000;
#endif

    MINT_OS_ENTER_CRITICAL();
    /* Issue kill-send-packet instruction for this device. */
    BB_write_baseband_register(CONNECTOR_REGISTER,
                               (UCHAR) ( (am_addr << 5) | (phy_piconet_id << 11)) );
    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_KILL_SEND_PACKET);

    {
        UINT16 temp_var1;

        temp_var1 = (UINT16) ((am_addr << 5) | CONNECTOR_REGISTER_FOR_ACL_BIT);
        temp_var1 &= (~CONNECTOR_REGISTER_MAKE_BIT);

        BB_write_baseband_register(
            CONNECTOR_REGISTER, (temp_var1 | ((phy_piconet_id << 11))));
        BB_write_baseband_register(INSTRUCTION_REGISTER, BB_EXECUTE);

        BB_write_baseband_register(
            lut_ex_table[lut_index].upper_lut_address, read);
    }
    MINT_OS_EXIT_CRITICAL();

    /* Reset lut_ex_table */
    lc_reset_lut_ex_table(lut_index);

    BB_modify_xtol_in_scatternet(0x00, phy_piconet_id);

    bzdma_invalid_txcmd(BZDMA_TX_ENTRY_TYPE_PICONET0 + phy_piconet_id,
                        am_addr, 0);

#ifdef TEST_MODE
    /* Restore the hop scheme to 79 hops. */
    sco_pkt_type_reg = BB_read_baseband_register(SCO_PACKET_TYPE_REGISTER);
    sco_pkt_type_reg &= 0xff1f;
    sco_pkt_type_reg |= 0x20;
    BB_write_baseband_register(SCO_PACKET_TYPE_REGISTER, sco_pkt_type_reg);

    /* Restore the three bits in SCO pkt type register. */
    AND_val_with_bb_reg(ENCRYPTION_ENABLE_REGISTER, 0xF7BF);

    lc_tci_pause_flag = FALSE;

    if (lmp_self_device_data.test_mode != HCI_NO_LOOPBACK_MODE)
    {
#if defined(_DUT_DELAYED_LOOPBACK_MODE_)
        pf_switch_hci_dma_parameter(DUT_TEST_MODE_FIFO_FREE);
#endif
        lmp_self_device_data.test_mode = HCI_DEVICE_UNDER_TEST_MODE;
    }

    test_mode_rx_int_cnt = 0;
    test_mode_tx_int_cnt = 0;
    test_mode_sched_acl_pkt = FALSE;
    test_mode_tx_mode_force_ack = FALSE;
    test_mode_tx_mode_buf_in_used = FALSE;

#ifdef _DUT_DELAYED_LOOPBACK_MODE_
    dut_mode_manager.buf[0].ppkt = NULL;
    dut_mode_manager.buf[1].ppkt = NULL;
    dut_mode_manager.work_space = 0;
#endif

#endif /* TEST_MODE */

#ifdef BROADCAST_DATA
    if ( (lmp_self_device_data.number_of_hlc == 0)
#ifdef COMPILE_PARK_MODE
            && (lmp_self_device_data.number_of_parked_dev == 0)
#endif
       )
    {
        BB_encryption_control(BC_AM_ADDR, phy_piconet_id,
                              BB_ENC_TX_DISBALE_RX_DISABLE);

        bzdma_invalid_txcmd(BZDMA_TX_ENTRY_TYPE_BROADCAST, 0, 0);
    }
#endif

    no_of_con = & (lmp_self_device_data.lc_no_of_connections[phy_piconet_id]);

    if ( (*no_of_con) != 0)
    {
        (*no_of_con)--;
    }

    if ( (*no_of_con) == 0)
    {
        /* Last connection is disconnected. */
        OS_SIGNAL sig_send;

        MINT_OS_ENTER_CRITICAL();
        /* Disable NBC */
        BB_disable_NBC(phy_piconet_id);
        MINT_OS_EXIT_CRITICAL();

        sig_send.type = LMP_LAST_CON_DISCONNECTED_SIGNAL;
        OS_SEND_SIGNAL_TO_TASK(lmp_task_handle, sig_send);
    }

    lc_update_scatternet_state_deletion(phy_piconet_id);

    /* Clear the piconet-info-register, if this is the last
    connection in the piconet. */

    if (lmp_self_device_data.lc_no_of_connections[phy_piconet_id] == 0)
    {
        BZ_REG_S_PICONET_INFO lcl_temp_var;
        UINT16 reg_offset = PICONET2_INFO_REGISTER;

        if (phy_piconet_id <= SCA_PICONET_MAX)
        {
            reg_offset = reg_PICONET_INFO[phy_piconet_id];
        }
        *(UINT16*)&lcl_temp_var = BB_read_baseband_register(reg_offset);
        lcl_temp_var.master = FALSE;
        lcl_temp_var.lt_addr = 0;
        BB_write_baseband_register(reg_offset, *(UINT16*)&lcl_temp_var);

        /* all share piconet 1 tx fifo now */
#ifdef _CCH_IOT_FTP_PAGE_
        //RT_BT_LOG(BLUE, CCH_DBG_079, 1,phy_piconet_id);
#else
        lc_flush_piconet1_tx_fifo(); // can remove
        //RT_BT_LOG(BLUE, CCH_DBG_080, 1,phy_piconet_id);
#endif
    }

#ifdef _CCH_SLOT_OFFSET_
    lmp_disconnect_global_slot_offset(ce_index);
#endif

#ifdef _DAPE_TEST_FOR_HID_SCO
	    if ((lc_sco_pause_status & (BIT0 << phy_piconet_id)) != 0 )
	    {
	        lc_sco_pause_status &= ~(BIT0 << phy_piconet_id);
	        if (lc_sco_pause_status == 0)
	        {
	            bb_pause_sco(FALSE);
#ifdef COMPILE_ESCO
#ifdef _DAPE_TEST_NEW_HW_PAUSE_ESCO_WHEN_RETX
                    bb_pause_esco(FALSE);
#endif
#endif
            }
        }
#endif

#ifdef LE_MODE_EN
#ifdef _DAPE_NEW_HW_NO_BLOCK_LEGACY_FOR_LE_WHEN_RETX
        //if ((g_enable_le_block_legacy == 0) && (NO_BLOCK_LEGACY_FOR_LE_WHEN_RETX))
        if (g_enable_le_block_legacy == 0)
        {
            if ((lc_le_pause_status & (BIT0 << phy_piconet_id)) != 0 )
            {
                lc_le_pause_status &= ~(BIT0 << phy_piconet_id);
                if (lc_le_pause_status == 0)
                {
                    ll_driver_block_legacy_slot_for_le(g_block_legacy_for_le_slot_num);
                }
            }
        }
#endif
#endif

#ifdef COMPILE_DYNAMIC_POLLING
    lc_update_dynamic_polling();
#endif
    lc_check_and_cleanup_bd_addr_regs_on_disc(phy_piconet_id);

    lc_check_and_enable_scans_in_scatternet();

#ifdef ENABLE_LOGGER
    UINT8 lc_no_of_conns = 0;
    UINT8 mid = 0;

    if (lc_sca_manager.master_cnt)
    {
        mid = lc_sca_manager.master_id;
        lc_no_of_conns = lmp_self_device_data.lc_no_of_connections[mid];
    }

    RT_BT_LOG(GRAY, MSG_LC_KILL_HLC, 12,
              lc_sca_manager.master_cnt, mid, lc_no_of_conns,
              lc_sca_manager.bm_slave, phy_piconet_id, am_addr,
              ce_ptr->bd_addr[5], ce_ptr->bd_addr[4], ce_ptr->bd_addr[3],
              ce_ptr->bd_addr[2], ce_ptr->bd_addr[1], ce_ptr->bd_addr[0]);
#endif

#ifdef _LPS_FOR_8821_
    sleep_mode_param.lps_table_instance[ce_index] = 0;
#endif


    /* TODO: shall we consider to program Tx gain be default value ? - austin */

    return;
}

#ifdef _FREE_FHS_PKT_WHEN_KILL_INQUIRY_
void lc_free_fhs_pkt_buffer(void)
{
    if (prev_fhs_ptr_for_eir != NULL)
    {
        /* because we stop inquiry flow, we shall free the
           fhs pkt buffer - austin */
        OS_FREE_BUFFER(lmp_fhs_pkt_buffer_pool_handle,
                          prev_fhs_ptr_for_eir);
        prev_fhs_ptr_for_eir = NULL;
    }
}
#endif


#ifdef COMPILE_PERIODIC_INQUIRY
/**
* Programs inquiry to the baseband. Also calculates Ninq based on the
* number of SCO connections. Calculates and programs inq parity bits.
*
* \param max_val Max value ofinq received from the host
* \param min_val Min value ofinq received from the host
* \param lap Lower address part for inquiry
* \param length Duration of the inquiry process
*
* \return API_SUCCESS, if the operation is successful, API_FAILURE otherwise.
*/
API_RESULT lc_handle_periodic_inquiry(UINT16 max_val , UINT16 min_val,
                                      UINT32 lap, UCHAR inq_len)
{
    UCHAR parity_bits[5];
    UINT32 timeout_value;
    UINT16 temp_reg;
#ifdef ENABLE_SCO
    UINT16 SCO_packet_type;
    UCHAR  number_of_sco_connection = 0;
#endif

    /* Either of paging/periodic-inq will be supported. */
    if (lmp_self_device_data.lc_cur_dev_state == LC_CUR_STATE_PAGE)
    {
        return API_FAILURE;
    }

    if (lmp_self_device_data.lc_cur_dev_state != LC_CUR_STATE_IDLE)
    {
        lc_kill_scan_mode();
    }

    /* Kill inquiry */
#ifdef _DAPE_TEST_NEW_HW_INSTRUCTION_KILL_SCAN

#ifdef _DAPE_TEST_NEW_HW_INSTRUCTION_KILL_SCAN_FOR_SAVE
    lc_kill_inquiry_scan_instruction();
#else
    lc_kill_inquiry_instruction();
#endif
#else
    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_KILL_INQUIRY);
#ifdef _FREE_FHS_PKT_WHEN_KILL_INQUIRY_
    lc_free_fhs_pkt_buffer();
#endif
#endif

    lmp_self_device_data.device_status = LMP_PERIODIC_INQUIRY;

    lc_periodic_inq_max_interval = max_val ;
    lc_periodic_inq_min_interval = min_val ;

    lc_periodic_inquiry_interval = inq_len * LC_INQUNIT_TO_SLOT_CONV_FACTOR;



#ifdef VER_2_1
    lc_write_bc_pwr_level(lmp_self_device_data.inq_id_tx_pwr);
#endif

    /* Configure all the registers required for periodic inquiry */
    /* Configure inquiry giac_diac_registers */

    temp_reg = (UINT16) (lap & 0xFFFF) ;
    BB_write_baseband_register(GIAC_DAIC_REGISTER1,temp_reg);

    temp_reg = (UINT16) ( (lap >> 16) & 0xFF) ;
    BB_write_baseband_register_lower_octet(GIAC_DAIC_REGISTER2,temp_reg);

    /* Set GIAC BIT 8 */
    OR_val_with_bb_reg(GIAC_DAIC_REGISTER2,0x0100);

    lc_generate_parity_bits(lap, &parity_bits[0]);

    /* Here Firmware Configure constant value for general inquiry
    parity bits */
    BT_FW_EXTRACT_16_BITS_WA(temp_reg, &(parity_bits[0]));
    BB_write_baseband_register(INQUIRY_PARITY_BITS_REGISTER1, temp_reg);
    BT_FW_EXTRACT_16_BITS_WA(temp_reg, &(parity_bits[2]));
    BB_write_baseband_register(INQUIRY_PARITY_BITS_REGISTER2, temp_reg);
    BB_WRITE_INQ_PARITY_BITS((*((UINT16 *)(&parity_bits[4]))));

    /* Configure Inquiry length. */
    /* Convert length in number of slots */
    BB_write_baseband_register(INQUIRY_TIMEOUT_REGISTER,
                               (UINT16) lc_periodic_inquiry_interval);

#ifdef ENABLE_SCO
    SCO_packet_type = lmp_self_device_data.sco_pkt_type;
    number_of_sco_connection = lmp_self_device_data.total_no_of_sco_conn;


#ifdef _CCH_INQ_PAG_SCAN_LEG_

    UINT16 n_inquiry_val = 0x0100;

    number_of_sco_connection += lmp_self_device_data.number_of_esco_connections;

    switch(number_of_sco_connection)
    {
        case 0 : /* 256 */
            n_inquiry_val = 0x0100;
            break;

        case 1 : /* > 512 */
            if ((lmp_self_device_data.number_of_esco_connections > 0) ||(SCO_packet_type == HV3) )
            {  /* SCO and Esco are not allowed together. */
                n_inquiry_val = 0x0200;
            }
            else if (SCO_packet_type == HV2)
            {
                n_inquiry_val = 0x0300;
            }
            break;

        case 2 : /* 768 */
            n_inquiry_val = 0x0300;
            break;

        default :
             n_inquiry_val = 0x0300;
    }


    BB_write_baseband_register(N_INQUIRY_REGISTER, n_inquiry_val);


#else
    if (SCO_packet_type != BB_HV3)
    {
        UINT16 n_inquiry_val;
        switch(number_of_sco_connection)
        {
            case 0 : /* 256 */
                n_inquiry_val = 0x0100;
                break;

            case 1 : /* > 512 */
                n_inquiry_val = 0x0200;
                break;

            case 2 : /* 768 */
                n_inquiry_val = 0x0300;
                break;

            default :
                n_inquiry_val = 0x0000;
                break ;
        }
        BB_write_baseband_register(N_INQUIRY_REGISTER, n_inquiry_val);
    }

#endif

#else /* ENABLE_SCO */
    BB_write_baseband_register(N_INQUIRY_REGISTER, 0x0100);
#endif /* ENABLE_SCO */

    /* If periodic inquiry is already running , Stop and Delete the timer */
    OS_DELETE_TIMER(&periodic_inquiry_timer);

    if (OS_CREATE_TIMER(ONESHOT_TIMER, &periodic_inquiry_timer,
            lc_start_periodic_inquiry, NULL, 0) != BT_ERROR_OK )
    {
#ifdef ENABLE_LOGGER_LEVEL_2
        LC_LOG_ERROR(LOG_LEVEL_LOW, START_PERIODIC_INQ_FAILED_TO_CREATE_TIMER,0,0);
#endif

        return API_FAILURE;
    }

    /* Set lmp_num_inq_resp_received to zero */
    lmp_num_inq_resp_received = 0 ;
    /* Start timer for periodic inquiry unit of lc_periodic_inq_max_interval
    * and lc_periodic_inq_min_interval is 1.28 sec to convert it into msec
    * we are multiplying it by 1.28* 1000 = 1280
    */
    lc_periodic_inq_max_interval = lc_periodic_inq_max_interval * 1280;
    lc_periodic_inq_min_interval = lc_periodic_inq_min_interval * 1280;
    timeout_value =
        (lc_periodic_inq_max_interval + lc_periodic_inq_min_interval) / 2;

#ifdef ENABLE_LOGGER_LEVEL_2
    LC_LOG_INFO(LOG_LEVEL_LOW, STARTING_PERIODIC_TMER_FOR_MILLI_SECONDS,1,
                timeout_value);
#endif

    if (OS_START_TIMER(periodic_inquiry_timer, timeout_value) != BT_ERROR_OK )
    {
#ifdef ENABLE_LOGGER_LEVEL_2
        LC_LOG_ERROR(LOG_LEVEL_LOW, START_PER_INQ_FAILED_TO_START_TIMER,0,0);
#endif

        return API_FAILURE;
    }

    /* Invoke Inquiry assuming that all other
    * registers are already configured */

#ifdef ENABLE_LOGGER_LEVEL_2
    LC_LOG_INFO(LOG_LEVEL_LOW, STARTING_PERIODIC_INQUIRY,0,0);
#endif
#ifndef _DAPE_TEST_NEW_HW_BLOCK_ACL_FOR_LE
#ifdef LE_MODE_EN
#ifdef _DAPE_TEST_BLOCK_LEGACY_FOR_LE
    /* dape test for LE slave + inquiry */
    ll_driver_block_legacy_for_le(TRUE);
#endif
#endif
#endif
    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_INQUIRY);

    lmp_periodic_inquiry = TRUE ;

    lc_set_lc_cur_device_state(LC_CUR_STATE_INQUIRY);

    return API_SUCCESS ;
}

/**
* Starts one round of inquiry in the baseband. Software timer expiry
* triggers this.
*
* \param timer_handle Periodic inquiry timer handle
* \param param parameters that the timer task can pass, NULL
*              in this case
*
* \return None.
*/
void lc_start_periodic_inquiry(TimerHandle_t timer_handle)
{
    UINT32 timeout_value;
    API_RESULT status;

    /* Kill IS or PS if it is in progress. */
    lc_kill_scan_mode();

    /* Set the LMP state as PERIODIC INQUIRY */
    lmp_self_device_data.device_status = LMP_PERIODIC_INQUIRY;
    /* Set lmp_num_inq_resp_received to zero */
    lmp_num_inq_resp_received = 0 ;
    /* Start timer for periodic inquiry */
    timeout_value = (lc_periodic_inq_max_interval + lc_periodic_inq_min_interval) / 2;

    status = OS_START_TIMER(periodic_inquiry_timer, timeout_value);
    if ( status != BT_ERROR_OK )
    {
        return ;
    }
    lmp_periodic_inquiry = TRUE ;

#ifdef COMPILE_HOLD_MODE
    if ( (lmp_self_device_data.number_of_connections_in_hold_mode != 0) &&
            (lmp_self_device_data.hold_mode_activity !=
             HCI_HOLD_MODE_ACTIVITY_DEFAULT) )
    {
        if (lmp_self_device_data.hold_mode_activity &
                HCI_HOLD_MODE_ACTIVITY_SUSPEND_PER_INQ)
        {
            /* Do not program periodic inq in hold mode. */
            RT_BT_LOG(GRAY, LC_TASK_1404, 0, 0);
            return;
        }
    }
#endif
#ifndef _DAPE_TEST_NEW_HW_BLOCK_ACL_FOR_LE
#ifdef LE_MODE_EN
#ifdef _DAPE_TEST_BLOCK_LEGACY_FOR_LE
    /* dape test for LE slave + inquiry */
    ll_driver_block_legacy_for_le(TRUE);
#endif
#endif
#endif
    BB_write_baseband_register(INQUIRY_TIMEOUT_REGISTER,
                               (UINT16) lc_periodic_inquiry_interval);

    BB_write_baseband_register(INSTRUCTION_REGISTER,BB_INQUIRY);

    lc_set_lc_cur_device_state(LC_CUR_STATE_INQUIRY);

#ifdef COMPILE_DYNAMIC_POLLING
    lc_update_dynamic_polling();
#endif

    return;
}


/**
* Kills the inquiry process in the baseband. Also retrieves scans.
*
* \param None.
*
* \return API_SUCCESS, if the operation is successful, API_FAILURE otherwise.
*/
API_RESULT lc_kill_periodic_inquiry(void)
{
    lc_periodic_inq_max_interval = 0;
    lc_periodic_inq_min_interval = 0;
    lc_periodic_inquiry_interval = 0;
    /* Set lmp_num_inq_resp_received to zero */
    lmp_num_inq_resp_received = 0 ;

    /* Kill inquiry */
#ifdef _DAPE_TEST_NEW_HW_INSTRUCTION_KILL_SCAN

#ifdef _DAPE_TEST_NEW_HW_INSTRUCTION_KILL_SCAN_FOR_SAVE
    lc_kill_inquiry_scan_instruction();
#else
    lc_kill_inquiry_instruction();
#endif
#else
    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_KILL_INQUIRY);
#ifdef _FREE_FHS_PKT_WHEN_KILL_INQUIRY_
    lc_free_fhs_pkt_buffer();
#endif
#endif

    OS_DELETE_TIMER(& periodic_inquiry_timer);


#ifndef _DAPE_TEST_NEW_HW_BLOCK_ACL_FOR_LE
#ifdef LE_MODE_EN
#ifdef _DAPE_TEST_BLOCK_LEGACY_FOR_LE
    /* dape test for LE slave + inquiry */
    ll_driver_block_legacy_for_le(FALSE);
#endif
#endif
#endif
#ifdef _DAPE_TEST_KILL_PERIODIC_INQ_IF_TIMER_RUNNING
    if (lmp_self_device_data.device_status != LMP_PAGE)
#endif
    {
        lmp_self_device_data.device_status = LMP_IDLE;

        lc_set_lc_cur_device_state(LC_CUR_STATE_IDLE);
    }
    lc_check_and_enable_scans_in_scatternet();

    return API_SUCCESS ;
}
#endif /* COMPILE_PERIODIC_INQUIRY */

/**
* Programs inquiry procedure to the baseband.
*
* \param cmd_buffer Pointer to the HCI command packet received
*
* \return HCI_COMMAND_SUCCEEDED on successful execution.
*         COMMAND_DISALLOWED_ERROR on failure.
*/
UCHAR lc_start_inquiry(HCI_CMD_PKT *cmd_buffer)
{
#ifdef COMPILE_DIAC
    UCHAR parity_bits[5];
#endif
    UINT32 lap;
    UINT16 temp_reg;
    UINT16 inquiry_length;
#if !defined(USE_SINGLE_CONNECTION) && defined(ENABLE_SCO)
    UINT16 SCO_packet_type;
    UCHAR number_of_sco_connection=0;
#endif

//#ifdef BZ_2_1_2
//    UINT16 n_inquiry_val = 0x0000;
//#else
    UINT16 n_inquiry_val = 0x0100;
//#endif

    UINT16 sco_ce_index;
/*
#ifdef _ROM_CODE_PATCHED_
#ifdef _DAPE_8723_RCP_
    if (rcp_lc_start_inq_func != NULL)
    {
        UINT8 result;

        if (rcp_lc_start_inq_func((void *)cmd_buffer, &result))
        {
            return result;
        }
    }
#endif
#endif
*/
    lc_write_bc_pwr_level(lmp_self_device_data.inq_id_tx_pwr);

    /* Extract LAP from command packet */
    lap = cmd_buffer->cmd_parameter[2];
    lap <<= 8;
    lap |= cmd_buffer->cmd_parameter[1];
    lap <<= 8;
    lap |= cmd_buffer->cmd_parameter[0];

#ifdef ENABLE_LOGGER_LEVEL_2
    LC_LOG_INFO(LOG_LEVEL_LOW, INQUIRY_RECD_LAP,1,lap);
#endif

    /*Configure inquiry giac_diac_registers */
    BT_FW_EXTRACT_16_BITS(temp_reg, &(cmd_buffer->cmd_parameter[0]));
    BB_write_baseband_register(GIAC_DAIC_REGISTER1,temp_reg);
    temp_reg = cmd_buffer->cmd_parameter[2];
    BB_write_baseband_register_lower_octet(GIAC_DAIC_REGISTER2, temp_reg);

    /* Check for lap either its for general inquiry or dedicated inquiry */
#ifdef COMPILE_DIAC
    if (lap == LC_GEN_INQUIRY_LAP) /* General inquiry */
#endif /* COMPILE_DIAC */
    {
        /* Here Firmware Configure constant value for general inquiry
        parity bits */
        BB_write_baseband_register(INQUIRY_PARITY_BITS_REGISTER1, LC_GIACP1);
        BB_write_baseband_register(INQUIRY_PARITY_BITS_REGISTER2, LC_GIACP2);
        BB_WRITE_INQ_PARITY_BITS(LC_GIACP3);
    }
#ifdef COMPILE_DIAC
    else /* if (lap == LC_GEN_INQUIRY_LAP) */
    {
#ifdef ENABLE_LOGGER_LEVEL_2
        LC_LOG_INFO(LOG_LEVEL_LOW, PROGRAMING_DIAC_INQUIRY,0,0);
#endif

        /* Generate parity bits corresponding to lap */
        lc_generate_parity_bits(lap, &parity_bits[0]);

#ifdef ENABLE_LOGGER_LEVEL_2
        LC_LOG_INFO(LOG_LEVEL_LOW, PRTY_BITS,5,parity_bits[0],
                    parity_bits[1],parity_bits[2],parity_bits[3],parity_bits[4]);
#endif

        /* Configure calculated value of parity bits */
        BT_FW_EXTRACT_16_BITS_WA(temp_reg, &(parity_bits[0]));
        BB_write_baseband_register(INQUIRY_PARITY_BITS_REGISTER1, temp_reg);
        BT_FW_EXTRACT_16_BITS_WA(temp_reg, &(parity_bits[2]));
        BB_write_baseband_register(INQUIRY_PARITY_BITS_REGISTER2, temp_reg);
        BB_WRITE_INQ_PARITY_BITS((*((UINT16 *) &parity_bits[4])));
    }
#endif /* COMPILE_DIAC */

    /* Configure Inquiry length */
    inquiry_length = cmd_buffer->cmd_parameter[3];

    /* Length should be less then 30 where unit of length is 1.28 secs
    * Base  Unit is 2 slots , converting length in 2 slots
    */
    inquiry_length = (UINT16) (inquiry_length * LC_INQUNIT_TO_SLOT_CONV_FACTOR);
    BB_write_baseband_register(INQUIRY_TIMEOUT_REGISTER, inquiry_length);

#ifndef USE_SINGLE_CONNECTION
#ifdef ENABLE_SCO
    SCO_packet_type = lmp_self_device_data.sco_pkt_type;
    number_of_sco_connection = lmp_self_device_data.total_no_of_sco_conn;

    if ((SCO_packet_type == HV1) && (number_of_sco_connection > 0))
    {
#ifdef ENABLE_LOGGER_LEVEL_2
        LC_LOG_ERROR(LOG_LEVEL_LOW, SCO_LINK_WITH_HV1_EXISTS,0,0);
#endif
        return COMMAND_DISALLOWED_ERROR;
    }

    /* If there is a HV3 connection in slave piconet, then assume
    that there are 2 HV3 connections. */
    for (temp_reg = 0; temp_reg < LMP_MAX_SCO_CONNECTIONS; temp_reg++)
    {
        if (lmp_sco_connection_data[temp_reg].sco_conn_status == SCO_CONNECTED)
        {
            sco_ce_index = lmp_sco_connection_data[temp_reg].conn_entity_index;

            if (lmp_connection_entity[sco_ce_index].remote_dev_role == MASTER)
            {
                if (number_of_sco_connection == 1)
                {
                    number_of_sco_connection++;
                    RT_BT_LOG(GRAY, LC_TASK_1591, 1, sco_ce_index);
                }
            }
        }
    }

#ifdef _CCH_INQ_PAG_SCAN_LEG_

    number_of_sco_connection += lmp_self_device_data.number_of_esco_connections;

    switch(number_of_sco_connection)
    {
        case 0 : /* 256 */
            n_inquiry_val = 0x0100;
            break;

        case 1 : /* > 512 */
            if ( (lmp_self_device_data.number_of_esco_connections > 0)  || (SCO_packet_type == HV3) )
            {  /* SCO and Esco are not allowed together. */
                n_inquiry_val = 0x0200;
            }
            else if (SCO_packet_type == HV2)
            {
                n_inquiry_val = 0x0300;
            }
            break;

        case 2 : /* 768 */
            n_inquiry_val = 0x0300;
            break;

        default :
             return COMMAND_DISALLOWED_ERROR;
    }

//    RT_BT_LOG(GRAY, CCH_DBG_123, 1,n_inquiry_val);

#else
    switch(number_of_sco_connection)
    {
        case 0 : /* 256 */
            n_inquiry_val = 0x0100;
            break;

        case 1 : /* > 512 */
            if (SCO_packet_type == HV3)
            {
                n_inquiry_val = 0x0200;
            }
            else if (SCO_packet_type == HV2)
            {
                n_inquiry_val = 0x0300;
            }
            break;

        case 2 : /* 768 */
            if (SCO_packet_type == HV3)
            {
                n_inquiry_val = 0x0300;
            }
            else if (SCO_packet_type == HV2)
            {
#ifdef ENABLE_LOGGER_LEVEL_2
                LC_LOG_ERROR(LOG_LEVEL_LOW, TWO_HV2_EXISTS,0,0);
#endif
                return COMMAND_DISALLOWED_ERROR;
            }
            break;

        default :
#ifdef ENABLE_LOGGER_LEVEL_2
            LC_LOG_ERROR(LOG_LEVEL_LOW, INVALID_NUMBER_OF_SCO_CONNECTIONS,1,
                         number_of_sco_connection);
#endif
            return COMMAND_DISALLOWED_ERROR;
    }

#endif

#endif
#endif /* USE_SINGLE_CONNECTION */
#ifndef _DAPE_TEST_NEW_HW_BLOCK_ACL_FOR_LE
#ifdef LE_MODE_EN
#ifdef _DAPE_TEST_BLOCK_LEGACY_FOR_LE
    /* dape test for LE slave + inquiry */
    ll_driver_block_legacy_for_le(TRUE);
#endif
#endif
#endif
#ifdef _DAPE_ACL_PRI_HIGHER_THAN_INQ_WHEN_BR
    //if (g_acl_priority_high_than_inq)
    //{
        BZ_REG_S_PRI_CTRL2 pri_ctrl2;
        *(UINT16*)&pri_ctrl2 = BB_read_baseband_register(SCA_PRIORITY_REGISTER3);

        if (g_acl_priority_high_than_inq)
        {
            //pri_ctrl2.inquiry_acl_pri = FALSE;
            pri_ctrl2.inq_high_than_acl_def = FALSE;
        }
        else
        {
            //pri_ctrl2.inquiry_acl_pri = TRUE;
            pri_ctrl2.inq_high_than_acl_def = TRUE;
        }
        BB_write_baseband_register(SCA_PRIORITY_REGISTER3, *(UINT16*)&pri_ctrl2);
    //}
#endif

    BB_write_baseband_register(N_INQUIRY_REGISTER, n_inquiry_val);
#ifdef _ROM_CODE_PATCHED_
#ifdef _DAPE_8723_RCP_
    if (rcp_lc_start_inq_func != NULL)
    {
        UINT8 result;

        if (rcp_lc_start_inq_func((void *)cmd_buffer, &result))
        {
            return result;
        }
    }
#endif
#endif

    BB_write_baseband_register(INSTRUCTION_REGISTER,BB_INQUIRY);

    lc_set_lc_cur_device_state(LC_CUR_STATE_INQUIRY);

#ifdef COMPILE_DYNAMIC_POLLING
    lc_update_dynamic_polling();
#endif

    return HCI_COMMAND_SUCCEEDED;
}


const UINT16 n_page_arr[3][3]= { {0x0001, 0x0080, 0x0100},
                            {0x0002, 0x0100, 0x0200},
                            {0x0003, 0x0180, 0x0300}
};

/**
* Programs baseband to start paging.
*
* \param cmd_buffer Pointer to HCI command packet
*
* \return HCI_COMMAND_SUCCEEDED, On success.
*/
UCHAR lc_start_paging(HCI_CMD_PKT *cmd_buffer)
{
    UINT32 lap;
    UINT16 temp_reg;
    UINT16 read;
    UINT16 ce_index;
    UCHAR am_addr;
    UINT16 n_page;
    UCHAR scan_parameter;
    UCHAR num_sco_links;
    UCHAR lut_index;
    UINT16 cmd_opcode;
    UCHAR page_scan_rep_mode;

    LMP_CONNECTION_ENTITY* ce_ptr;

    UCHAR piconet_id;
    UINT8 idx;

#ifdef ENABLE_SCO
    UINT16 sco_ce_index = 0;
#endif

    UINT16 lcl_remote_dev_bd_addr_register1;
    UINT16 lcl_remote_dev_bd_addr_register2;
    UINT16 lcl_remote_dev_bd_addr_register3;
    UINT16 lcl_remote_dev_parity_bits_register1;
    UINT16 lcl_remote_dev_parity_bits_register2;

    DEF_CRITICAL_SECTION_STORAGE;
/*
#ifdef _ROM_CODE_PATCHED_
#ifdef _DAPE_8723_RCP_
    if (rcp_lc_start_page_func != NULL)
    {
        UINT8 result;

        if (rcp_lc_start_page_func((void *)cmd_buffer, &result))
        {
            return result;
        }
    }
#endif
#endif
*/
    cmd_opcode = cmd_buffer->cmd_opcode;

    piconet_id = lc_allocate_piconet_id_for_paging();

    idx = piconet_id;

    if (piconet_id == SCA_PICONET_INVALID)
    {
        RT_BT_LOG(RED, PICONET_ID_CHECK_MSG, 1, piconet_id);
        return HOST_REJECTED_LIMITED_RESOURCES_ERROR;
    }

    /* Take am_addr from CE corresponding to BD_ADDR */
    LMP_GET_CE_INDEX_FROM_BD_ADDR((UCHAR *)&cmd_buffer->cmd_parameter[0],
                                  (UINT16*)&ce_index);

    if (ce_index == INVALID_CE_INDEX)
    {
        return NO_CONNECTION_ERROR;
    }

    lcl_remote_dev_bd_addr_register1 = reg_PICONET_BD_ADDR1[idx];
    lcl_remote_dev_bd_addr_register2 = reg_PICONET_BD_ADDR2[idx];
    lcl_remote_dev_bd_addr_register3 = reg_PICONET_BD_ADDR3[idx];

    lcl_remote_dev_parity_bits_register1 = reg_PICONET_PARITY_BITS1[idx];
    lcl_remote_dev_parity_bits_register2 = reg_PICONET_PARITY_BITS2[idx];

    /* Extract LAP from command packet */
    lap = cmd_buffer->cmd_parameter[2];
    lap <<= 8;
    lap |= cmd_buffer->cmd_parameter[1];
    lap <<= 8;
    lap |= cmd_buffer->cmd_parameter[0];

    MINT_OS_ENTER_CRITICAL();

    /* Configure remote device BD_ADDR */
    BT_FW_EXTRACT_16_BITS(temp_reg, &(cmd_buffer->cmd_parameter[0]));
    BB_write_baseband_register(lcl_remote_dev_bd_addr_register1,temp_reg);
    lc_paging_bd_addr[0] = temp_reg;

    BT_FW_EXTRACT_16_BITS(temp_reg, &(cmd_buffer->cmd_parameter[2]));
    BB_write_baseband_register(lcl_remote_dev_bd_addr_register2, temp_reg);
    lc_paging_bd_addr[1] = temp_reg;

    BT_FW_EXTRACT_16_BITS(temp_reg, &(cmd_buffer->cmd_parameter[4]));
    BB_write_baseband_register(lcl_remote_dev_bd_addr_register3, temp_reg);
    lc_paging_bd_addr[2] = temp_reg;

    /* Calculate Parity bits of remote device */
    lc_generate_parity_bits(lap, (UINT8*)lc_paging_parity_bits);

    /* Configure remote device parity bits */
    BB_write_baseband_register(lcl_remote_dev_parity_bits_register1,
                               lc_paging_parity_bits[0]);

    BB_write_baseband_register(lcl_remote_dev_parity_bits_register2,
                               lc_paging_parity_bits[1]);

    BB_WRITE_REMOTE_PARITY_BITS(lc_paging_parity_bits[2] & 0xff, piconet_id);

    /* Program class of device */
    temp_reg = (UINT16) lmp_self_device_data.class_of_device;
    BB_write_baseband_register(CLASS_OF_DEVICE_REGISTER1, temp_reg);

    temp_reg = (UCHAR) (lmp_self_device_data.class_of_device >> 16 );
    BB_write_baseband_register_lower_octet(CLASS_OF_DEVICE_REGISTER2, temp_reg);

#ifndef _DAPE_TEST_CORRECT_PAGE_SCAN_PARAM_IN_FHS
    scan_parameter = cmd_buffer->cmd_parameter[8];
#else
    if (cmd_opcode == HCI_CREATE_CONNECTION_OPCODE)
    {
        scan_parameter = cmd_buffer->cmd_parameter[8];
    }
    else
    {
        scan_parameter = cmd_buffer->cmd_parameter[6];
    }
#endif
    scan_parameter <<= 2;
    scan_parameter |= 0x02;
    scan_parameter <<= 4;
    BB_write_baseband_register_upper_octet(FHS_PARAMETER_REGISTER,
                                           scan_parameter);

    ce_ptr = &lmp_connection_entity[ce_index];

    ce_ptr->remote_dev_role = SLAVE;

    BB_write_baseband_register(CLOCK_OFFSET_REGISTER, ce_ptr->clock_offset);

    am_addr = ce_ptr->am_addr;

#ifdef COMPILE_AFH_HOP_KERNEL
    /* Disable AFH. */
    BB_disable_afh(am_addr, piconet_id);
#endif
#ifndef _DAPE_TEST_NEW_HW_BLOCK_ACL_FOR_LE
#ifdef LE_MODE_EN
#ifdef _DAPE_TEST_BLOCK_LEGACY_FOR_LE
    /* dape test for LE slave + inquiry */
    ll_driver_block_legacy_for_le(TRUE);
#endif
#endif
#endif
    lc_set_lc_cur_connecting_am_addr(am_addr);

#ifdef ENABLE_SCO
#ifdef _DAPE_TEST_NEW_HW_SCO_SLV_PRIORITY_HIGHER_THAN_PAGE
    BOOLEAN sco_slv = FALSE;
#endif
    num_sco_links = lmp_self_device_data.total_no_of_sco_conn;

    /* If there is a HV3 connection in slave piconet, then assume
    that there are 2 HV3 connections. */
    for(temp_reg = 0; temp_reg < LMP_MAX_SCO_CONNECTIONS; temp_reg++)
    {
        if (lmp_sco_connection_data[temp_reg].sco_conn_status == SCO_CONNECTED)
        {
            sco_ce_index = lmp_sco_connection_data[temp_reg].conn_entity_index;

            if (lmp_connection_entity[sco_ce_index].remote_dev_role == MASTER)
            {
#ifdef _DAPE_TEST_NEW_HW_SCO_SLV_PRIORITY_HIGHER_THAN_PAGE
                sco_slv = TRUE;
#endif
                if (num_sco_links == 1)
                {
                    num_sco_links++;
                    RT_BT_LOG(GRAY, LC_TASK_1859, 1, num_sco_links);
                }
            }
        }
    }
#if defined (_DAPE_TEST_NEW_HW) && defined (_DAPE_TEST_NEW_HW_SCO_SLV_PRIORITY_HIGHER_THAN_PAGE)
{
        /* If we are sco slave, then priority[0] should be 0 to keep sco
           reserved slot sending sco. */
        BZ_REG_S_PRI_CTRL pri_ctrl;
        *(UINT16*)&pri_ctrl = BB_read_baseband_register(SCA_PRIORITY_REGISTER);
        pri_ctrl.page_rsp_high_than_sco = (~sco_slv);
        BB_write_baseband_register(SCA_PRIORITY_REGISTER, *(UINT16*)&pri_ctrl);
//RT_BT_LOG(BLUE, DAPE_TEST_LOG213, 2,SCA_PRIORITY_REGISTER,
//    BB_read_baseband_register(SCA_PRIORITY_REGISTER));
    }
#endif
#endif /* ENABLE_SCO */

#ifdef _CCH_INQ_PAG_SCAN_LEG_
    num_sco_links += lmp_self_device_data.number_of_esco_connections;

    if(num_sco_links >= 2)
    {
        num_sco_links = 2;
    }

#endif

    temp_reg = (UINT16) ((am_addr << 5) | (piconet_id << 11));

#ifdef _SUPPORT_CSB_RECEIVER_
    if (cmd_opcode == HCI_TRUNCATED_PAGE_OPCODE)
    {
        temp_reg |= BIT13;
    }
    else
    {
        temp_reg &= (~BIT13);
    }
#endif
#ifdef _CSB_RX_DBG_LOG
    if (g_csb_rx_dbg_log)
    {
        RT_BT_LOG(RED, DAPE_TEST_LOG207, 1,temp_reg);
    }
#endif

    BB_write_baseband_register(CONNECTOR_REGISTER, temp_reg);

    lc_paging_piconet_id = piconet_id;
#if 0
if( FALSE )
{
    if (cmd_opcode == HCI_CREATE_CONNECTION_OPCODE)
    {
        /* Page scan repetition mode of the remote device */
        page_scan_rep_mode = cmd_buffer->cmd_parameter[8];

        /* Take packet types allowed by host from the command packet */
        BT_FW_EXTRACT_16_BITS(pkt_types, &(cmd_buffer->cmd_parameter[6]));

        lc_update_pkts_allowed(ce_index);
        //this flag close, let the priority issue can be changed by patch
#ifdef _DAPE_ACL_PRI_HIGHER_THAN_PAGE_WHEN_REMOTE_NAME_REQ
        BZ_REG_S_PRI_CTRL pri_ctrl;
        *(UINT16*)&pri_ctrl = BB_read_baseband_register(SCA_PRIORITY_REGISTER);
        pri_ctrl.page_high_than_acl_def = TRUE;
        pri_ctrl.page_high_than_acl = TRUE;
        BB_write_baseband_register(SCA_PRIORITY_REGISTER, *(UINT16*)&pri_ctrl);
#endif

    }
    else
    {
        /* Page scan repetition mode of the remote device */
        page_scan_rep_mode = cmd_buffer->cmd_parameter[6];
        //this flag close, let the priority issue can be changed by patch
#ifdef _DAPE_ACL_PRI_HIGHER_THAN_PAGE_WHEN_REMOTE_NAME_REQ
        BZ_REG_S_PRI_CTRL pri_ctrl;
        *(UINT16*)&pri_ctrl = BB_read_baseband_register(SCA_PRIORITY_REGISTER);
        pri_ctrl.page_high_than_acl_def = FALSE;
        pri_ctrl.page_high_than_acl = FALSE;
        BB_write_baseband_register(SCA_PRIORITY_REGISTER, *(UINT16*)&pri_ctrl);
#endif

    }
}
#endif
    /* avoid exception */
    if (page_scan_rep_mode > 2)
    {
        page_scan_rep_mode = 2;
    }

    n_page = n_page_arr[num_sco_links][page_scan_rep_mode];

    BB_write_baseband_register(N_PAGE_REGISTER, n_page);

    /* Configure page timeout in the baseband. */
    BB_write_baseband_register(PAGE_TIMEOUT_REGISTER,
                            (UINT16) (lmp_self_device_data.page_timeout >> 1));

    /* Now program this piconet_id to the baseband. */
    OR_val_with_bb_reg(reg_PICONET_INFO[idx], SCA_PICONET_MASTER);

    lut_index = am_addr;

    lut_ex_table[lut_index].index_in_CE = ce_index;
    lut_ex_table[lut_index].lower_lut_address = reg_MASTER_LOWER_LUT[am_addr];
    lut_ex_table[lut_index].upper_lut_address = reg_MASTER_UPPER_LUT[am_addr];

    read = BB_read_baseband_register(lut_ex_table[lut_index].upper_lut_address);

    /* In the upper LUT clear the ARQN and SEQN bits. Set the Flow bit. */
    read &= 0xFFF8;
    read |= 0x0001;
    BB_write_baseband_register(lut_ex_table[lut_index].upper_lut_address, read);

#ifdef _ROM_CODE_PATCHED_
#ifdef _DAPE_8723_RCP_
    if (rcp_lc_start_page_func != NULL)
    {
        UINT8 result;

        if (rcp_lc_start_page_func((void *)cmd_buffer, &result))
        {
            MINT_OS_EXIT_CRITICAL();
            return result;
        }
    }
#endif
#endif

    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_PAGE);

    lc_set_lc_cur_device_state(LC_CUR_STATE_PAGE);

#ifdef COMPILE_DYNAMIC_POLLING
    lc_update_dynamic_polling();
#endif

    MINT_OS_EXIT_CRITICAL();

    RT_BT_LOG(GRAY, MSG_PAGING_REM_DEV, 6,
        BB_read_native_clock(),
        lc_paging_bd_addr[2], lc_paging_bd_addr[1], lc_paging_bd_addr[0],
        am_addr, piconet_id);
#ifdef _CSB_RX_DBG_LOG
    if (g_csb_rx_dbg_log)
    {
        RT_BT_LOG(WHITE, DAPE_TEST_LOG213, 2,
            BB_read_baseband_register(PAGE_TIMEOUT_REGISTER),
            BB_read_baseband_register(CONNECTOR_REGISTER));
    }
#endif
    return HCI_COMMAND_SUCCEEDED;
}

void lc_handle_received_fhs_pkt_in_role_switch(LMP_FHS_PKT_RECD  *fhs_pkt_buffer,
                                                         UINT8 am_addr)
{
    ALIGN(2) UCHAR remote_bd_addr[6]; /* These 2 arrays have to be at aligned addr. */
    ALIGN(2) UCHAR parity_bits[6];    /* So, do not move these down ! */
    UINT16 mss_rem_dev_add1;
    UINT16 mss_rem_dev_add2;
    UINT16 mss_rem_dev_add3;
    UINT16 mss_rem_par_bits_add1;
    UINT16 mss_rem_par_bits_add2;
    UINT32 lap;

    DEF_CRITICAL_SECTION_STORAGE;

#ifdef COMPILE_ROLE_SWITCH
    if (lmp_mss_state == LMP_MSS_INIT)
    {
        RT_BT_LOG(GRAY, LC_TASK_2175, 0, 0);
    }

    MINT_OS_ENTER_CRITICAL();

    AND_val_with_bb_reg_isr(
        lmp_role_switch_data.old_upper_lut_address,
        (UINT16)(~ACTIVE_BIT) );

    OR_val_with_bb_reg_isr(
        lmp_role_switch_data.new_upper_lut_address , ACTIVE_BIT);

    lmp_role_switch_data.new_am_addr = am_addr;
    lmp_slave_use_am_addr_ppi(am_addr,
                              lmp_role_switch_data.new_piconet_id,
                              lmp_role_switch_data.ce_index);

    if (lmp_mss_state != LMP_MSS_INIT)
    {
        lmp_set_mss_state(LMP_MSS_M_TO_S_FHS_RECD);
    }

    BB_write_baseband_register(CONNECTOR_REGISTER,
                       (lmp_role_switch_data.new_am_addr << 5) |
                       (lmp_role_switch_data.new_piconet_id << 11));
    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_EXECUTE);

    UINT8 idx = lmp_role_switch_data.new_piconet_id;
    mss_rem_dev_add1 = reg_PICONET_BD_ADDR1[idx];
    mss_rem_dev_add2 = reg_PICONET_BD_ADDR2[idx];
    mss_rem_dev_add3 = reg_PICONET_BD_ADDR3[idx];

    mss_rem_par_bits_add1 = reg_PICONET_PARITY_BITS1[idx];
    mss_rem_par_bits_add2 = reg_PICONET_PARITY_BITS2[idx];

    /* Extract remote bd_addr */
    lmp_extract_bd_addr_from_fhs_packet(fhs_pkt_buffer->fhs_pkt, remote_bd_addr);

    /* Configure remote device BD_ADDR */

    BB_write_baseband_register(mss_rem_dev_add1, *(UINT16*)&remote_bd_addr[0]);
    BB_write_baseband_register(mss_rem_dev_add2, *(UINT16*)&remote_bd_addr[2]);
    BB_write_baseband_register(mss_rem_dev_add3, *(UINT16*)&remote_bd_addr[4]);

    lap = (remote_bd_addr[2] << 16) |
          (remote_bd_addr[1] << 8) | remote_bd_addr[0];

    lc_generate_parity_bits(lap, &parity_bits[0]);
    lc_slave_parity_bits[0] = *(UINT16*)&parity_bits[0];
    lc_slave_parity_bits[1] = *(UINT16*)&parity_bits[2];
    lc_slave_parity_bits[2] = (*(UINT16*)&parity_bits[4]) & 0xff;

    BB_write_baseband_register(mss_rem_par_bits_add1, *(UINT16*)&parity_bits[0]);
    BB_write_baseband_register(mss_rem_par_bits_add2, *(UINT16*)&parity_bits[2]);
    BB_WRITE_REMOTE_PARITY_BITS(parity_bits[4], idx);

    MINT_OS_EXIT_CRITICAL();

#ifdef SCNET_DEBUG
    RT_BT_LOG(GRAY, LC_TASK_2267, 21,
          mss_rem_dev_add1, mss_rem_dev_add2, mss_rem_dev_add3,
          remote_bd_addr[0], remote_bd_addr[1], remote_bd_addr[2],
          remote_bd_addr[3], remote_bd_addr[4], remote_bd_addr[5],
          parity_bits[0], parity_bits[1], parity_bits[2],
          parity_bits[3], parity_bits[4], parity_bits[5],
          lmp_role_switch_data.old_piconet_id,
          lmp_role_switch_data.new_piconet_id,
          lmp_role_switch_data.old_am_addr,
          lmp_role_switch_data.new_am_addr,
          lmp_role_switch_data.old_lut_index,
          lmp_role_switch_data.new_lut_index
         );
#endif

    if (OS_FREE_BUFFER(lmp_fhs_pkt_buffer_pool_handle,
                        fhs_pkt_buffer) != BT_ERROR_OK)
    {
#ifdef ENABLE_LOGGER_LEVEL_2
        LC_LOG_ERROR(LOG_LEVEL_LOW, "OS free buffer failed.");
#endif
    }
#endif /* COMPILE_ROLE_SWITCH */
}

void lc_handle_received_fhs_pkt_in_page_resp(LMP_FHS_PKT_RECD  *fhs_pkt_buffer,
                                                        UINT8 am_addr, UINT8 piconet_id)
{
    ALIGN(2) UCHAR remote_bd_addr[6]; /* These 2 arrays have to be at aligned addr. */
    ALIGN(2) UCHAR parity_bits[6];    /* So, do not move these down ! */
    UCHAR lcl_lut_index;
    UCHAR pid;
    UINT16 temp;
    UINT32 lap;

    DEF_CRITICAL_SECTION_STORAGE;

    MINT_OS_ENTER_CRITICAL();

    if (lc_pagescan_piconet_id == SCA_PICONET_INVALID)
    {
        /* Invalid FHS packet. Dropping the packet. */

        if (OS_FREE_BUFFER(lmp_fhs_pkt_buffer_pool_handle,
                          fhs_pkt_buffer) != BT_ERROR_OK)
        {
#ifdef ENABLE_LOGGER_LEVEL_2
            LC_LOG_ERROR(LOG_LEVEL_LOW, "OS free buffer failed.");
#endif
        }
        MINT_OS_EXIT_CRITICAL();

        RT_BT_LOG(GRAY, LC_TASK_2303, 0, 0);
        return;
    }

    lc_set_lc_cur_connecting_am_addr(am_addr);

    pid = lc_pagescan_piconet_id;

    temp = (UINT16)(((fhs_pkt_buffer->am_addr) << 1) | SCA_PICONET_SLAVE);
    OR_val_with_bb_reg(reg_PICONET_INFO[pid], temp);

    lcl_lut_index = LC_SCA_SLAVE_1_LUT + pid;

    lut_ex_table[lcl_lut_index].lower_lut_address = reg_SCA_SLAVE_LOWER_LUT[pid];
    lut_ex_table[lcl_lut_index].upper_lut_address = reg_SCA_SLAVE_UPPER_LUT[pid];

    /* Arqn = 0, Seqn = 0, Flow = 1 */
    temp = BB_read_baseband_register(reg_SCA_SLAVE_UPPER_LUT[pid]);
    temp &= 0xFFF9;
    temp |= 0x0001;
    BB_write_baseband_register(reg_SCA_SLAVE_UPPER_LUT[pid], temp);

    lmp_handle_page_scan_fhs_pkt(fhs_pkt_buffer,
                            pid, lcl_lut_index,
                            remote_bd_addr);

    /* Configure connector register and remote parity bits */
    temp = (am_addr << 5) | (pid << 11) ;
    BB_write_baseband_register(CONNECTOR_REGISTER, temp);

    /* Configure remote device BD_ADDR */
    lc_slave_bd_addr[0] = *(UINT16*)&remote_bd_addr[0];
    BB_write_baseband_register(reg_PICONET_BD_ADDR1[pid], lc_slave_bd_addr[0]);

    lc_slave_bd_addr[1] = *(UINT16*)&remote_bd_addr[2];
    BB_write_baseband_register(reg_PICONET_BD_ADDR2[pid], lc_slave_bd_addr[1]);

    lc_slave_bd_addr[2] = *(UINT16*)&remote_bd_addr[4];
    BB_write_baseband_register(reg_PICONET_BD_ADDR3[pid], lc_slave_bd_addr[2]);

    lap = (remote_bd_addr[2] << 16) | lc_slave_bd_addr[0];
    lc_generate_parity_bits(lap, &parity_bits[0]);
    lc_slave_parity_bits[0] = *(UINT16*)&parity_bits[0];
    lc_slave_parity_bits[1] = *(UINT16*)&parity_bits[2];
    lc_slave_parity_bits[2] = (*(UINT16*)&parity_bits[4]) & 0x00ff;

    BB_write_baseband_register(
        reg_PICONET_PARITY_BITS1[pid], lc_slave_parity_bits[0]);
    BB_write_baseband_register(
        reg_PICONET_PARITY_BITS2[pid], lc_slave_parity_bits[1]);

    BB_WRITE_REMOTE_PARITY_BITS(parity_bits[4], (UCHAR) piconet_id);

    if (OS_FREE_BUFFER(lmp_fhs_pkt_buffer_pool_handle,
                        fhs_pkt_buffer) != BT_ERROR_OK)
    {
#ifdef ENABLE_LOGGER_LEVEL_2
        LC_LOG_ERROR(LOG_LEVEL_LOW, "OS free buffer failed");
#endif
    }
    MINT_OS_EXIT_CRITICAL();

#ifdef SCNET_DEBUG
    RT_BT_LOG(GRAY, LC_TASK_0380_4, 1, pid);
    RT_BT_LOG(GRAY, LC_TASK_2397, 1, temp);
#endif
}

void lc_handle_received_fhs_pkt_in_inquiry_resp(LMP_FHS_PKT_RECD  *fhs_pkt_buffer,
                                                          PKT_HEADER *ppkt)
{
    OS_SIGNAL sig_send;
    BZ_REG_S_RX_STATUS *rx_status = (BZ_REG_S_RX_STATUS *)&ppkt->packet_header;

    /* Scenario one for Inquiry response */

#ifdef COMPILE_INQ_RES_EVENT_WITH_RSSI
    if (lmp_self_device_data.inquiry_mode == HCI_INQ_RESULT_EVENT_WITH_RSSI
        || lmp_self_device_data.inquiry_mode == HCI_INQ_RESULT_EVENT_WITH_EIR
       )
    {
        ppkt->rssi = lc_calculate_log_from_rssi( (UINT16)ppkt->rssi);

        fhs_pkt_buffer->rssi = (CHAR) ppkt->rssi;
    }
#endif /* COMPILE_INQ_RES_EVENT_WITH_RSSI */

    if (prev_fhs_ptr_for_eir != NULL)
    {
        RT_BT_LOG(GRAY, LC_TASK_2490, 0, 0);
        OS_FREE_BUFFER(lmp_fhs_pkt_buffer_pool_handle,
                            prev_fhs_ptr_for_eir);
        prev_fhs_ptr_for_eir = NULL;
    }

    /* Expecting EIR */
    if (rx_status->eir && ((fhs_pkt_buffer->fhs_pkt[7] & 0x4) != 0))
    {
        prev_fhs_ptr_for_eir = fhs_pkt_buffer;
    }
    else
    {
        /* Force EIR bit to zero - Not expecting EIR Data */
        fhs_pkt_buffer->fhs_pkt[7] &= 0xFB;

        fhs_pkt_buffer->clk = ppkt->clk;

        /* Generate a signal "FHS PACKET RECEIVED" to LMP task */
        sig_send.type = LMP_FHS_PKT_RECD_SIGNAL;
        sig_send.param= (OS_ADDRESS)fhs_pkt_buffer;
        sig_send.ext_param = (OS_ADDRESS)SCA_PICONET_INVALID;
        OS_SEND_SIGNAL_TO_TASK(lmp_task_handle, sig_send);
    }

#ifdef SCNET_DEBUG
    RT_BT_LOG(GRAY, LC_TASK_2525, 18,
              fhs_pkt_buffer->fhs_pkt[0],
              fhs_pkt_buffer->fhs_pkt[1],
              fhs_pkt_buffer->fhs_pkt[2],
              fhs_pkt_buffer->fhs_pkt[3],
              fhs_pkt_buffer->fhs_pkt[4],
              fhs_pkt_buffer->fhs_pkt[5],
              fhs_pkt_buffer->fhs_pkt[6],
              fhs_pkt_buffer->fhs_pkt[7],
              fhs_pkt_buffer->fhs_pkt[8],
              fhs_pkt_buffer->fhs_pkt[9],
              fhs_pkt_buffer->fhs_pkt[10],
              fhs_pkt_buffer->fhs_pkt[11],
              fhs_pkt_buffer->fhs_pkt[12],
              fhs_pkt_buffer->fhs_pkt[13],
              fhs_pkt_buffer->fhs_pkt[14],
              fhs_pkt_buffer->fhs_pkt[15],
              fhs_pkt_buffer->fhs_pkt[16],
              fhs_pkt_buffer->fhs_pkt[17]
             );
#endif
}

void lc_handle_received_fhs_pkt(PKT_HEADER *ppkt, UINT8 lt_addr, UINT8 piconet_id)
{
    LMP_FHS_PKT_RECD  *fhs_pkt_buffer;
    UINT8 am_addr;

    /* Allocate buffer for FHS packet */
    if (OS_ALLOC_BUFFER(lmp_fhs_pkt_buffer_pool_handle,
                            (void**)(&fhs_pkt_buffer)) != BT_ERROR_OK)
    {
#ifndef BZDMA_DONT_WAIT_RXCMD_COMPLETE
        BB_read_baseband_RX_FIFO_and_flush(18, TRUE);
#else
        BB_read_baseband_RX_FIFO_and_flush(18, FALSE);
#endif
        RT_BT_LOG(GRAY, LC_TASK_2149, 0, 0);
        return;
    }

    /* copy fhs content from acl rx fifo */
    BB_read_baseband_RX_FIFO(fhs_pkt_buffer->fhs_pkt, 18, TRUE);

    /* Scenario two for page response and master slave switch */
    if (lmp_self_device_data.lc_cur_dev_state != LC_CUR_STATE_INQUIRY)
    {
        /* Extract am_addr from FHS packet */
        am_addr = (UCHAR) (fhs_pkt_buffer->fhs_pkt[14] & 0x07);
        fhs_pkt_buffer->am_addr = am_addr;

#ifdef COMPILE_ROLE_SWITCH
        if (lt_addr != BC_AM_ADDR)
        {
            lc_handle_received_fhs_pkt_in_role_switch(fhs_pkt_buffer, am_addr);
        }
        else
#endif
        {
            lc_handle_received_fhs_pkt_in_page_resp(fhs_pkt_buffer, am_addr, piconet_id);
        }
    }
    else
    {
        //======= 20120109 morgan add ============
#ifdef PTA_PROFILE_ESTIMATION_OVER_INQUIRY
        fnPtaFhsParser(fhs_pkt_buffer->fhs_pkt);
#endif
        //===================

        lc_handle_received_fhs_pkt_in_inquiry_resp(fhs_pkt_buffer, ppkt);
    }
}

void lc_handle_received_lmp_pkt(UINT16 payload_length, UINT8 am_addr, UINT8 phy_piconet_id)
{
    LMP_PDU_PKT  *lmp_pkt;
    UINT16 ce_index;
    OS_SIGNAL sig_send;

    if (payload_length == 0)
    {
        /* Zero Byte lmp is received don't send it to LMP module
        * No need to read Baseband FIFO
        */
#ifdef ENABLE_LOGGER_LEVEL_2
        LC_LOG_ERROR(LOG_LEVEL_LOW,"Empty lmp-pkt recvd");
#endif

        return;
    }

    if (OS_ALLOC_BUFFER(lmp_pdu_buffer_pool_handle,(void **)(&lmp_pkt))
                                                            != BT_ERROR_OK)
    {
#ifndef BZDMA_DONT_WAIT_RXCMD_COMPLETE
        BB_read_baseband_RX_FIFO_and_flush(payload_length, TRUE);
#else
        BB_read_baseband_RX_FIFO_and_flush(payload_length, FALSE);
#endif
        RT_BT_LOG(GRAY, LC_TASK_2584, 0, 0);
        return;
    }

#ifndef BZDMA_DONT_WAIT_RXCMD_COMPLETE
    BB_read_baseband_RX_FIFO(lmp_pkt->payload_content,payload_length, TRUE);
#else
    BB_read_baseband_RX_FIFO(lmp_pkt->payload_content,payload_length, FALSE);
#endif

    ce_index = 0x0;
    if (am_addr != BC_AM_ADDR)
    {
        if (LMP_GET_CE_INDEX_FROM_AM_ADDR_PPI(am_addr, phy_piconet_id,
                                             &ce_index) == API_FAILURE)
        {
            ce_index = INVALID_CE_INDEX;
        }
    }

    lmp_pkt->am_addr = (UCHAR) am_addr;
    lmp_pkt->pdu_length = (UCHAR) payload_length;
    lmp_pkt->ce_index = ce_index;

    /* Generate a signal "LMP PACKET RECEIVED" to LMP task */
    sig_send.type = LMP_PDU_RECD_SIGNAL;
    sig_send.param = lmp_pkt;
    sig_send.ext_param = (OS_ADDRESS)((UINT32)phy_piconet_id);
    OS_SEND_SIGNAL_TO_TASK(lmp_task_handle, sig_send);
}

UINT8 lc_handle_received_l2cap_pkt(UINT16 payload_length, UINT8 am_addr, UINT8 phy_piconet_id, UINT8 l_ch)
{
    UINT16 ce_index;
    HCI_ACL_DATA_PKT *acl_pkt;
    OS_SIGNAL sig_send;

    if (payload_length == 0)
    {
        /* Zero Byte l2cap data is coming Do not send it to host
        * No need to read Baseband FIFO
        */
        return TRUE;
    }

    if (am_addr != BC_AM_ADDR)
    {
        /* Unicast */

        if (LMP_GET_CE_INDEX_FROM_AM_ADDR_PPI(am_addr, phy_piconet_id,
                                             &ce_index) == API_FAILURE)
        {
#ifndef BZDMA_DONT_WAIT_RXCMD_COMPLETE
            BB_read_baseband_RX_FIFO_and_flush(payload_length, TRUE);
#else
            BB_read_baseband_RX_FIFO_and_flush(payload_length, FALSE);
#endif

#ifdef _DAPE_TEST_ACL_RX_FIFO_DBG
            //gpio_one_pull_high(1);
            //gpio_one_pull_low(1);
            //SET_BT_GPIO_OUTPUT_3_0(0x1);
            //SET_BT_GPIO_OUTPUT_3_0(0);

            UINT32 dape_reg;
            dape_reg = RD_U32_BZDMA_REG(BZDMA_REG_ACL_RXFIFO_PTR);
            RT_BT_LOG(RED, DAPE_TEST_LOG525, 6,BZDMA_REG_ACL_RXFIFO_PTR,
				dape_reg, am_addr, phy_piconet_id, payload_length, 0);
#endif
            /* This case cant happen, we recd a packet when
            there is no connection. Anyway, free the reserve
            buffer and exit. */
            return TRUE;
        }
    }
#ifndef BROADCAST_DATA
    else
    {
        /* broadcast */

        /* Flush the rx fifo. */
#ifndef BZDMA_DONT_WAIT_RXCMD_COMPLETE
        BB_read_baseband_RX_FIFO_and_flush(payload_length, TRUE);
#else
        BB_read_baseband_RX_FIFO_and_flush(payload_length, FALSE);
#endif
        return TRUE;
    }
#endif /* BROADCAST_DATA */

#ifdef TEST_MODE
    //chris_text_mode_modify
    if (lmp_self_device_data.test_mode == HCI_REMOTE_LOOPBACK_MODE)
    {
        if ( (OS_ALLOC_BUFFER(rx_table[ACL_DATA_HANDLER_TASK].pool_handle,
                          (void **)&acl_pkt))!= BT_ERROR_OK)
        {
            /* Alloc buffer should never fail. */
#ifndef BZDMA_DONT_WAIT_RXCMD_COMPLETE
            BB_read_baseband_RX_FIFO_and_flush(payload_length, TRUE);
#else
            BB_read_baseband_RX_FIFO_and_flush(payload_length, FALSE);
#endif
            RT_BT_LOG(GRAY, LC_TASK_2675, 0, 0);

            return TRUE;
        }
    }
    else
#endif
    {
        if ( (OS_ALLOC_BUFFER(tx_table[ACL_DATA_HANDLER_TASK].pool_handle,
                          (void **)&acl_pkt))!= BT_ERROR_OK)
        {
            /* Alloc buffer should never fail because of FLOW_REDESIGN */
#ifndef BZDMA_DONT_WAIT_RXCMD_COMPLETE
            BB_read_baseband_RX_FIFO_and_flush(payload_length, TRUE);
#else
            BB_read_baseband_RX_FIFO_and_flush(payload_length, FALSE);
#endif
            RT_BT_LOG(GRAY, LC_TASK_2689, 2,
                      os_get_reserved_buffers(),
                      OS_get_free_buffer(
                          tx_table[ACL_DATA_HANDLER_TASK].pool_handle));

            return TRUE;
        }
    }

    acl_pkt->acl_data_total_length = payload_length;

#ifdef BROADCAST_DATA
    if (am_addr == BC_AM_ADDR)
    {
        UINT8 bc_flag = 0x01;

        acl_pkt->packet_boundary_flag = (UINT16) l_ch;
        if (lmp_self_device_data.bc_conn_handle == 0xFFFF)
        {
#ifndef LE_MODE_EN
            lmp_self_device_data.bc_conn_handle = LMP_MAX_CONN_HANDLES+5;
#else
            lmp_self_device_data.bc_conn_handle = LL_HCI_MAX_CONN_HANDLE+5;
#endif
        }
        acl_pkt->connection_handle = lmp_self_device_data.bc_conn_handle;

        /* Parse through the lmp-CE, get the fist connection handle as slave. */
        {
            UCHAR temp;
            UCHAR valid_flag = FALSE;
            LMP_CONNECTION_ENTITY *ce_ptr;

            for (temp = 0; temp < LMP_MAX_CE_DATABASE_ENTRIES; temp++)
            {
                ce_ptr = &lmp_connection_entity[temp];
                if (ce_ptr->entity_status == ASSIGNED)
                {
                    if ((ce_ptr->remote_dev_role == MASTER) &&
                        (ce_ptr->phy_piconet_id == phy_piconet_id))
                    {
                        acl_pkt->connection_handle = (UINT16)
                               ce_ptr->connection_type.connection_handle;
                        valid_flag = TRUE;

#ifdef COMPILE_PARK_MODE
                        /* Check if the device is in park-state */
                        if (ce_ptr->ce_status == LMP_PARK_MODE)
                        {
                            bc_flag = 0x02;
                        }
#endif
                        break;
                    }
                }
            }

            if (valid_flag != TRUE)
            {
                /* This is not possible: We are not slave, and still
                received broadcast data. */
            }
        }

        acl_pkt->broadcast_flag = bc_flag;
#ifdef SECURE_CONN_BROADCAST_CHK
        RT_BT_LOG(BLUE, DAPE_TEST_LOG584, 0,0);
#endif
    }
    else
#endif /* BROADCAST_DATA */
    {
        LMP_CONNECTION_ENTITY *ce_ptr;
        ce_ptr = &lmp_connection_entity[ce_index];

        acl_pkt->connection_handle = ce_ptr->connection_type.connection_handle;

        acl_pkt->packet_boundary_flag = (UINT16) l_ch;

        acl_pkt->broadcast_flag = 0x00;
    }

#ifdef TEST_MODE
    UINT16 more_len = 0;
    if (lmp_self_device_data.test_mode == HCI_REMOTE_LOOPBACK_MODE)
    {
        /* because we have reduced hci acl data packet size, so need to
           check the length range of remote loopback data packet */
        if (payload_length > otp_str_data.bt_read_buffer_size)
        {
            more_len = payload_length - otp_str_data.bt_read_buffer_size;
            payload_length = otp_str_data.bt_read_buffer_size;
        }
    }
#endif /* end of #ifdef TEST_MODE */

#ifndef BZDMA_DONT_WAIT_RXCMD_COMPLETE
    BB_read_baseband_RX_FIFO(acl_pkt->hci_acl_data_pkt, payload_length, TRUE);
#else
    BB_read_baseband_RX_FIFO(acl_pkt->hci_acl_data_pkt, payload_length, FALSE);
#endif

#ifdef TEST_MODE
    if (lmp_self_device_data.test_mode == HCI_REMOTE_LOOPBACK_MODE)
    {
        if (more_len > 0)
        {
            /* flush more data */
            // TODO: maybe we can consider reassemble more data in multi-packets (austin)
#ifndef BZDMA_DONT_WAIT_RXCMD_COMPLETE
            BB_read_baseband_RX_FIFO_and_flush(more_len, TRUE);
#else
            BB_read_baseband_RX_FIFO_and_flush(more_len, FALSE);
#endif
        }

        /* Send the Signal to LC Task for loopback to remote device */
        sig_send.type = LC_HANDLE_HOST_DATA_PKT;
        sig_send.param = acl_pkt;
        if (OS_SEND_SIGNAL_TO_TASK(rx_table[ACL_DATA_HANDLER_TASK].task_handle, sig_send) !=
                BT_ERROR_OK)
        {
#ifdef ENABLE_LOGGER_LEVEL_2
            LC_LOG_ERROR(LOG_LEVEL_LOW, "OS send signal to task failed\n");
#endif
        }
        return TRUE;
    }
    else
#endif
    {
        sig_send.type = HCI_TD_ACL_DATA_TO_HOST_SIGNAL;
        sig_send.param = acl_pkt;
        if (OS_SEND_SIGNAL_TO_TASK(hci_td_task_handle,sig_send) !=
                BT_ERROR_OK)
        {
#ifdef ENABLE_LOGGER_LEVEL_2
            LC_LOG_ERROR(LOG_LEVEL_LOW, "OS send signal to task failed\n");
#endif
            return TRUE;
        }
    }
    return FALSE;
}

UINT8 lc_handle_received_pkt_after_pre_fhs_pkt(PKT_HEADER *ppkt)
{
    UINT16 payload_length;
    UINT8 *buffer = NULL;
    UINT8 err = FALSE;
    UINT8 good_user_data = FALSE;
    OS_SIGNAL sig_send;
    BZ_REG_S_RX_PL_HDR *pl_hdr = (BZ_REG_S_RX_PL_HDR *)&ppkt->payload_header;
    BZ_REG_S_RX_STATUS *rx_status = (BZ_REG_S_RX_STATUS *)&ppkt->packet_header;
    UINT8 result = FALSE;

    if (rx_status->eir && (rx_status->pkt_type != BB_FHS))
    {
        payload_length = pl_hdr->len;

        buffer = (UCHAR *)prev_fhs_ptr_for_eir->recv_data;
#ifndef _DAPE_TEST_CLEAR_ACL_RXFIFO_WHEN_EIR_LENGTH_WRONG
        /* Defensive code - may not be required */
        if (payload_length > MAX_EIR_DATA_LEN)
        {
            err = TRUE;
            payload_length = MAX_EIR_DATA_LEN;
        }
#endif

#ifdef _CCH_SC_ECDH_P256_MIC_FLAG
        if ((!rx_status->crc_err) && (!pl_hdr->mic_err) && (!rx_status->hec_err))
#else
        if (!rx_status->crc_err && !rx_status->hec_err)
#endif
        {
#ifdef _DAPE_TEST_CLEAR_ACL_RXFIFO_WHEN_EIR_LENGTH_WRONG
            /* Defensive code - may not be required */
            if (rx_status->lt_addr != 0)
            {
                err = TRUE;
                prev_fhs_ptr_for_eir->recv_length = 0;
                /* Force EIR bit to zero - Not expecting EIR Data */
                prev_fhs_ptr_for_eir->fhs_pkt[7] &= 0xFB;
                good_user_data = TRUE;
            }
            else
#endif
            {
                /* EIR packet without error */
#ifndef BZDMA_DONT_WAIT_RXCMD_COMPLETE
                BB_read_baseband_RX_FIFO(buffer, payload_length, TRUE);
#else
                BB_read_baseband_RX_FIFO(buffer, payload_length, FALSE);
#endif
                /* UCHAR because EIR supports max of 240 bytes only */
                prev_fhs_ptr_for_eir->recv_length = (UCHAR) payload_length;
            }
        }
        else
        {
            err = TRUE;

            prev_fhs_ptr_for_eir->recv_length = 0;

            /* Force EIR bit to zero - Not expecting EIR Data */
            prev_fhs_ptr_for_eir->fhs_pkt[7] &= 0xFB;
        }

        if (err)
        {
            RT_BT_LOG(RED, MSG_EIR_LEN_MISMATCH, 3,
                           ppkt->packet_header, ppkt->payload_header,
                           RD_U32_BZDMA_REG(BZDMA_REG_ACL_RXFIFO_PTR));
        }

        if (good_user_data == FALSE)
        {
            result = TRUE;
        }
    }
    else
    {
        /* if we do not receive EIR pkt after fhs pkt (with EIR bit),
           we can only notify the fhs information to host */
        prev_fhs_ptr_for_eir->recv_length = 0;

#ifdef _CCH_SC_ECDH_P256_MIC_FLAG
        if (rx_status->crc_err || pl_hdr->mic_err || rx_status->hec_err || !rx_status->rx_pkt )
#else
        if (rx_status->crc_err || rx_status->hec_err || !rx_status->rx_pkt )
#endif
        {
            //RT_BT_LOG(RED, CCH_DBG_126, 0,0);E;
            result = TRUE;
        }
    }

    sig_send.type = LMP_FHS_PKT_RECD_SIGNAL;
    sig_send.param= (OS_ADDRESS)prev_fhs_ptr_for_eir;
    sig_send.ext_param = (OS_ADDRESS)SCA_PICONET_INVALID;
    OS_SEND_SIGNAL_TO_TASK(lmp_task_handle, sig_send);

    prev_fhs_ptr_for_eir = NULL;

    return result;
}


/**
* Decodes the packet received, after reading the packet from the baseband
* FIFO, and sends the packet to LMP modulre or L2CAP handling.
*
* \param None.
*
* \return None.
*/
UINT8 lc_handle_received_packet_in_scatternet(void)
{
    PKT_HEADER pkt;
    UINT16 payload_length;
    UCHAR pkt_type;
    UCHAR am_addr;
    UCHAR l_ch;
    UCHAR phy_piconet_id;
    UINT8 good_packet;

    DEF_CRITICAL_SECTION_STORAGE;

    MINT_OS_ENTER_CRITICAL();

    /* potential race condition -- must in critical section (austin) */

    /* Take headers from the top of queue */
    if (lc_rx_pkt_header_q.read_pointer != lc_rx_pkt_header_q.write_pointer)
    {
        /* clone rx packet header content */
        memcpy(&pkt, &lc_rx_pkt_header_q.pkt_header_queue[lc_rx_pkt_header_q.read_pointer],
                sizeof(PKT_HEADER));

        /* Update read pointer of queue */
        lc_rx_pkt_header_q.read_pointer++;
        lc_rx_pkt_header_q.read_pointer &= (LC_MAX_NUMBER_OF_HEADERS - 1);

        MINT_OS_EXIT_CRITICAL();

    }
    else
    {
        MINT_OS_EXIT_CRITICAL();

        /* There was no packet in circular buffer */
        RT_BT_LOG(GRAY, LC_TASK_2013, 0, 0);
        return TRUE;
    }

    BZ_REG_S_RX_PL_HDR *pl_hdr = (BZ_REG_S_RX_PL_HDR *)&pkt.payload_header;
    BZ_REG_S_RX_STATUS *rx_status = (BZ_REG_S_RX_STATUS *)&pkt.packet_header;

    /* Extract different parameters from the headers queue */
    pkt_type = rx_status->pkt_type;
    phy_piconet_id = (pl_hdr->pid_h << 1) | rx_status->pid_l;
    am_addr = rx_status->lt_addr;
#ifdef _CCH_SC_ECDH_P256_MIC_FLAG
    if (rx_status->rx_pkt && !rx_status->hec_err && !rx_status->crc_err && !pl_hdr->mic_err)
#else
    if (rx_status->rx_pkt && !rx_status->hec_err && !rx_status->crc_err)
#endif
    {
        good_packet = TRUE;
    }
    else
    {
        good_packet = FALSE;
    }
#ifdef _SUPPORT_CSB_RECEIVER_
    if (good_packet && (am_addr == 0) &&
#ifndef _DAPE_TEST_FOR_BCM_CSB_IN_UPF45
        (pkt_type == BB_DM3) && (pl_hdr->len == 28))

#else
  (pkt_type == BB_DM3))
#endif
    {
        bt_3dd_handle_received_sync_train_packet();
        return FALSE;
    }
#endif
#if defined(_3DD_FUNCTION_SUPPORT_) && !defined(_DO_NOT_SUPPORT_RP_)
    /* Check AN packet indication from 3DG to host */
    if (IS_SUPPORT_3DG_APP && !bt_3dd_var.is_csb_mode)
    {
        if (good_packet && rx_status->eir && (am_addr == 1) &&
            (pkt_type == BB_DM1) && (pl_hdr->len == 17))
        {
            bt_3dd_handle_received_assocation_notification_packet();
            return FALSE;
        }
    }
#endif

    if (prev_fhs_ptr_for_eir == NULL)
    {
        /* filter any error conditions */
        if (!good_packet)
        {
            return TRUE;
        }

#ifdef _DAPE_TEST_FILTER_EIR_MORE
        /* get eir indication now but no received any fhs packet early, so
           we shall drop it */
        if (rx_status->eir && (pkt_type != BB_FHS))
        {
            /* EIR packet without error */
#ifndef BZDMA_DONT_WAIT_RXCMD_COMPLETE
            BB_read_baseband_RX_FIFO_and_flush(pl_hdr->len, TRUE);
#else
            BB_read_baseband_RX_FIFO_and_flush(pl_hdr->len, FALSE);
#endif
            //RT_BT_LOG(RED, DAPE_TEST_LOG525, 6, *pl_hdr, *rx_status, prev_fhs_ptr_for_eir, 0,0,0);
            return TRUE;
        }
#endif
    }
    else
    {
        /* EIR Packet Handler */
        if (lc_handle_received_pkt_after_pre_fhs_pkt(&pkt))
        {
            return TRUE;
        }
    }
#ifdef _SUPPORT_CSB_RECEIVER_
    UINT16 ce_index;
    LMP_GET_CE_INDEX_FROM_AM_ADDR_PPI(
                   (UCHAR) am_addr, phy_piconet_id, &ce_index);

    if ((bt_3dd_var.csb_rx_param.enable) &&
        (bt_3dd_var.csb_rx_param.ce_index == ce_index))
    {
            bt_3dd_handle_received_beacon_packet(pl_hdr->len, am_addr,
                phy_piconet_id, pkt_type, pl_hdr->llid);
            return FALSE;
    }

#endif
#ifdef SECURE_CONN_BROADCAST_CHK
    if (am_addr == 0)
    {
        RT_BT_LOG(YELLOW, DAPE_TEST_LOG525, 6,
        am_addr,
        pkt.flush_flag, pkt_type,  0,0,0);
    }
#endif
    /* handle flushable received packet */
    if (pkt.flush_flag == TRUE)
    {
#ifndef BZDMA_DONT_WAIT_RXCMD_COMPLETE
        BB_read_baseband_RX_FIFO_and_flush(pl_hdr->len, TRUE);
#else
        BB_read_baseband_RX_FIFO_and_flush(pl_hdr->len, FALSE);
#endif
#ifdef SECURE_CONN_BROADCAST_CHK
    RT_BT_LOG(RED, DAPE_TEST_LOG525, 6,
    am_addr,
    pkt.flush_flag, pkt_type,  0,0,0);
#endif
        return TRUE;
    }

    /* check received FHS packet */
    if (pkt_type == BB_FHS)
    {
        lc_handle_received_fhs_pkt(&pkt, am_addr, phy_piconet_id);
        return TRUE;
    }

    /* check LLID and handle the packet */
    l_ch = pl_hdr->llid;
    payload_length = pl_hdr->len;
#ifdef _CSB_RX_DBG_LOG
    if (g_csb_rx_dbg_log)
    {
    UINT32 cur_clk;
    lc_get_clock_in_scatternet(&cur_clk, phy_piconet_id);
    RT_BT_LOG(GREEN, DAPE_TEST_LOG525, 6,l_ch, payload_length,
    am_addr, phy_piconet_id, BB_read_native_clock(),cur_clk);
    }
#endif

    if (l_ch == L_CH_LMP)
    {
        /* handle lmp packet */
        lc_handle_received_lmp_pkt(payload_length, am_addr, phy_piconet_id);
        return TRUE;
    }
    else if ((l_ch == L_CH_L2CAP_START) || (l_ch == L_CH_L2CAP_CONT))
    {
        /* handle l2cap packet */
        return lc_handle_received_l2cap_pkt(payload_length, am_addr, phy_piconet_id, l_ch);
    }
    else
    {
        /* handle exception packet */
        if (payload_length != 0)
        {
#ifndef BZDMA_DONT_WAIT_RXCMD_COMPLETE
            BB_read_baseband_RX_FIFO_and_flush(payload_length, TRUE);
#else
            BB_read_baseband_RX_FIFO_and_flush(payload_length, FALSE);
#endif
        }

#ifdef _DAPE_TEST_ACL_RX_FIFO_DBG
        //gpio_one_pull_high(1);
        //gpio_one_pull_low(1);
        //SET_BT_GPIO_OUTPUT_3_0(0x1);
        //SET_BT_GPIO_OUTPUT_3_0(0);

        UINT32 dape_reg;
        dape_reg = RD_U32_BZDMA_REG(BZDMA_REG_ACL_RXFIFO_PTR);
        RT_BT_LOG(RED, DAPE_TEST_LOG525, 6,BZDMA_REG_ACL_RXFIFO_PTR,
			      dape_reg, pl_hdr, rx_status, 0, 0);
#endif
        RT_BT_LOG(GRAY, LC_TASK_2785, 1, l_ch);
        return TRUE;
    }
}

/**
* Checks if flush-TO has occured for the L2CAP packet.
*
* \param acl_pkt Pointer to ACL packet received to be checked
* \param ce_ptr Pointer to lmp-connection-entity, to check if
*               the host has enabled auto-flush-to
*
* \return TRUE, if the packet has to be dropped, FALSE otherwise.
*/
UCHAR lc_is_pkt_timeout(HCI_ACL_DATA_PKT *pacl_pkt, LMP_CONNECTION_ENTITY *ce_ptr)
{
    INT32 cur_clock;
    INT32 pkt_clock;
    INT32 flush_timeout;
    INT32 diff;
    UCHAR status = FALSE;
    enum pkt_flush_status
    {
        CHECK_FLUSH = 0,  /* Check for timeout - flushable */
        FORCE_FLUSH,      /* If set flush continue packets */
        NEVER_FLUSH       /* non-flushable packet          */
    };

    HCI_ACL_DATA_PKT_WS *acl_pkt = pacl_pkt->ws;

#ifdef _DAPE_TEST_SEND_ZERO_LEN_PKT_BY_VENDOR_CMD
    if (g_send_zero_len)
    {
        acl_pkt->flush_time = FORCE_FLUSH_CLOCK;
        pacl_pkt->packet_boundary_flag = L_CH_L2CAP_START;
    }
#endif

    DEF_CRITICAL_SECTION_STORAGE;

    switch(pacl_pkt->packet_boundary_flag)
    {
        case L_CH_L2CAP_NON_FLUSH:
            ce_ptr->flush_continue_pkts = NEVER_FLUSH;
            break;

        case L_CH_L2CAP_START:
            ce_ptr->flush_continue_pkts = CHECK_FLUSH;
            break;

        default:
            break;
    }

    pkt_clock = acl_pkt->flush_time;
    if (pkt_clock == FORCE_FLUSH_CLOCK)
    {
        ce_ptr->flush_continue_pkts = (UCHAR)FORCE_FLUSH;
        return TRUE;
    }

    switch(ce_ptr->flush_continue_pkts)
    {
        case NEVER_FLUSH:
            return FALSE;

        case FORCE_FLUSH:
            /* L_CH_L2CAP_CONT pkts - flush */
            return TRUE;

        case CHECK_FLUSH:
        default:
            break;
    }

    /* Check if timeout flag is set */
    flush_timeout = (INT32) (ce_ptr->flush_timeout << 1);
    if (flush_timeout == 0)
    {
        return FALSE;
    }

    LC_EXIT_SM_MODE();

    MINT_OS_ENTER_CRITICAL();
    cur_clock = (INT32)BB_read_baseband_register(NATIVE_CLOCK2_REGISTER);
    cur_clock = cur_clock << 16;
    cur_clock |= (INT32)BB_read_baseband_register(NATIVE_CLOCK1_REGISTER);

    /* Only 28 bits are valid */
    cur_clock = (INT32)((UINT32)cur_clock & 0x0FFFFFFFU);

    if (cur_clock < pkt_clock)
    {
        /* Increase cur_clock (to nullify the effects of clock wrap-around) */
        cur_clock = (INT32) ((UINT32)cur_clock | 0x10000000U);
    }
    diff = cur_clock - pkt_clock;

    if (diff > flush_timeout)
    {
        /* Set flag to flush other continue pkts also */
        ce_ptr->flush_continue_pkts = (UCHAR)FORCE_FLUSH;
        status = TRUE;
    }
    MINT_OS_EXIT_CRITICAL();

#if 0
    RT_BT_LOG(GRAY, MSG_DBG_FLUSH_TIMEOUT, 5, pkt_clock,
                    cur_clock, diff, flush_timeout, status);
#endif

    return status;
}

void lc_check_aclq_in_flow_control(UINT8 phy_piconet_id, UINT8 ce_index)
{
    LMP_CONNECTION_ENTITY *ce_ptr = &lmp_connection_entity[ce_index];
    HCI_ACL_DATA_PKT *ppkt = acl_q_head;
    HCI_ACL_DATA_PKT_WS *pkt;

    while (ppkt != NULL)
    {
        pkt = ppkt->ws;

        if ((pkt->am_addr == ce_ptr->am_addr) &&
            (pkt->phy_piconet_id == phy_piconet_id))
        {
            if (lc_is_pkt_timeout(ppkt, ce_ptr) == TRUE)
            {
                if ((pkt->acl_pkt_tx_failed == 0x0) &&
                    (pkt->ackd_length >= (pkt->tx_length1 + pkt->tx_length2)))
                {
                    /* There is no pending fragment */
                    /* So we can flush this pkt safely */
                    pkt->in_transmission = TRUE;
                    aclq_ack_acl_pkt(ppkt,
                        (ppkt->acl_data_total_length - pkt->ackd_length),
                         0x1, TRUE);
                }
                else
                {
                    if (pkt->acl_pkt_tx_failed == 0x1)
                    {
                        pkt->acl_pkt_nbc_old_count = 0x0;
                        pkt->acl_pkt_tx_failed = 0x0;
                    }
                }
            }
        }
        ppkt = ppkt->next;
    }
}

/**
* Flushes the ACLU packet from the baseband and updates the LC scheduler
* accordingly. It also calls LC scheduler function to schedule new packets.
* Make sure that this function is called only from interrupt context. This
* function should not be called from task context.
*
* \param phy_piconet_id Physical piconet ID of the connection.
*
* \return None.
*/
void lc_reschedule(UCHAR phy_piconet_id, UCHAR lut_id)
{
    LC_PICONET_SCHEDULER *piconet_schd;
    LC_SCHEDULED_PKT *schd;
    UCHAR am_addr;
    UCHAR piconet_id = phy_piconet_id;
    UINT8 read_idx = 0;
    LC_SCHEDULED_PKT *schd1;

    DEF_CRITICAL_SECTION_STORAGE;

    piconet_schd =  &lc_piconet_scheduler[piconet_id];

    MINT_OS_ENTER_CRITICAL();

    read_idx = piconet_schd->rptr;

    schd = &piconet_schd->lc_scheduled_pkt_info[read_idx];

    if (schd->tx_status != LC_TX_SENT)
    {
        MINT_OS_EXIT_CRITICAL();
        UINT32 cur_clk;
        lc_get_clock_in_scatternet(&cur_clk, phy_piconet_id);
        LC_LOG_ERROR(LOG_LEVEL_HIGH,DUMMY_NBC_TIMEOUT,1,cur_clk);
        return;
    }

    am_addr = schd->selected_am_addr;

    /* invalid bzdma tx command and entry */
    if (am_addr == 0)
    {
        bzdma_invalid_txcmd(0, 0, 0);
    }
    else
    {
        bzdma_invalid_txcmd(phy_piconet_id + BZDMA_TX_ENTRY_TYPE_PICONET0,
                            am_addr, 0);
    }

	schd1 = &piconet_schd->lc_scheduled_pkt_info[!read_idx];

	if (schd1->tx_status != LC_TX_IDLE)
    {
		if (schd1->pkt_src == LC_PDU)
		{
			pduq_reset_pdu_wo_failed((LMP_PDU_PKT *)(schd1->packet_ptr));
		}
		else
		{
			aclq_reset_acl_pkt_wo_failed(
				(HCI_ACL_DATA_PKT *)(schd1->packet_ptr), phy_piconet_id);
		}

		if (piconet_schd->lc_allowed_pkt_cnt != LC_MAX_SCH_INFO)
		{
			piconet_schd->lc_allowed_pkt_cnt++;
		}

		piconet_schd->rptr++;

        schd1->tx_status = LC_TX_IDLE;
	}

    if (schd->tx_status != LC_TX_IDLE)
    {
        if (schd->pkt_src == LC_PDU)
        {
            pduq_reset_nbc_pdu((LMP_PDU_PKT *)(schd->packet_ptr));
        }
        else
        {
            LMP_CONNECTION_ENTITY *ce_ptr;
            ce_ptr = &lmp_connection_entity[schd->ce_index];

            /* Set reschedule flag from queued packet */
            if (aclq_is_zero_length_pkt(
                        (HCI_ACL_DATA_PKT*) schd->packet_ptr,
                        phy_piconet_id) == TRUE)
            {
                ce_ptr->aclq_resch_flag = RESCHEDULE_FLAG_ZERO_LENGTH;

#ifdef _BRUCE_FIX_DROP_ZERO_LEN_DM1_WHEN_KEEP_RETRY
                //record this lut_id
                temp_lut_id[phy_piconet_id] = lut_id;
#endif
            }
            else
            {
#ifdef _BRUCE_FIX_DROP_ZERO_LEN_DM1_WHEN_KEEP_RETRY
                //record this lut_id
                temp_lut_id[phy_piconet_id] = 0;
#endif

                /* Check may not be required */
                if (am_addr != BC_AM_ADDR)
                {
                    /**
                     *  Reschedule_flag:
                     * 0 - Normal
                     * 1 - ACL Reschdule
                     * 2 - zero length reschedule
                     */
                    ce_ptr->aclq_resch_flag = RESCHEDULE_FLAG_ACL;
                    piconet_id = ce_ptr->phy_piconet_id;
                }
                else
                {
                    piconet_id = 0;
                }

                aclq_reset_nbc_acl_pkt((HCI_ACL_DATA_PKT *)(schd->packet_ptr),
                           piconet_id,
                           schd->frag_len, schd->pkt_type_lut);
            }
        }

        if (piconet_schd->lc_allowed_pkt_cnt != LC_MAX_SCH_INFO)
        {
            piconet_schd->lc_allowed_pkt_cnt++;
        }

        piconet_schd->rptr++;

        schd->tx_status = LC_TX_IDLE;
    }

    MINT_OS_EXIT_CRITICAL();

    /* if remote device request to enter flow control, we need to check
       any pending flushable acl-u pkts are expired or not in the queue then
       response flush event to host - austin */
    if (lut_ex_table[lut_id].bb_flow == BB_STOP)
    {
        lc_check_aclq_in_flow_control(phy_piconet_id,
                                      lut_ex_table[lut_id].index_in_CE);
    }

#ifdef _BRUCE_FIX_DROP_ZERO_LEN_DM1_WHEN_KEEP_RETRY
                /*
                LMP_CONNECTION_ENTITY *ce_ptr;
                ce_ptr = &lmp_connection_entity[schd->ce_index];
                LC_SCHEDULED_PKT *schd1;
                schd1 = &piconet_schd->lc_scheduled_pkt_info[!piconet_schd->rptr];
                RT_BT_LOG(GRAY,BRUCE_DBG_9,9,piconet_schd->rptr,piconet_schd->wptr,
                    schd->tx_status,schd1->tx_status,schd->ce_index,schd1->ce_index,ce_ptr->aclq_resch_flag,
                    piconet_schd->lc_allowed_pkt_cnt,
                    22);*/
#endif
    lc_invoke_scheduler(piconet_id);
}

/**
* Signals LMP-PDU/L2CAP module about the ACK received.
*
* \param schd_read_index Read-index variable of the scheduler that
*                        contains details of the packet.
*        piconet_id : Physical piconet ID of the packet.
*
* \return None.
*/
void lc_handle_ack_received(UINT8 schd_read_index,
                            UCHAR phy_piconet_id, UINT32 debug_clock)
{
    UINT16 ce_index;
    LC_SCHEDULED_PKT       *ackd;
    UCHAR pdu_opcode;
    LMP_PDU_PKT *lmp_pkt;
    OS_SIGNAL sig_send ;
    LC_PICONET_SCHEDULER  *piconet_schd;

#ifdef TEST_MODE
    UINT16 sco_pkt_type_reg;
    UINT16 channel_reg;
    UINT16 rx_frequency;
    UCHAR hopping_mode;
#endif /* TEST_MODE */

    LMP_CONNECTION_ENTITY *ce_ptr;

    DEF_CRITICAL_SECTION_STORAGE;

    piconet_schd =  &lc_piconet_scheduler[phy_piconet_id];

    /* Get scheduler from schd_read_index */
    ackd = &piconet_schd->lc_scheduled_pkt_info[schd_read_index];

    ce_index = ackd->ce_index;

    if ( piconet_schd->lc_allowed_pkt_cnt != LC_MAX_SCH_INFO)
    {
        piconet_schd->lc_allowed_pkt_cnt++;
    }

    ce_ptr = &lmp_connection_entity[ce_index];

    UINT8 lut_id;
    lut_id = lc_get_lut_index_from_phy_piconet_id(ce_ptr->am_addr,
                                                  phy_piconet_id);

    if (lut_ex_table[lut_id].bb_flow == BB_STOP)
    {
        lc_check_aclq_in_flow_control(phy_piconet_id, ce_index);
    }

    if (ackd->packet_ptr == NULL)
    {
        lc_invoke_scheduler(phy_piconet_id);
        return;
    }

    if ( (ce_index != INVALID_CE_INDEX) &&
            (ce_ptr->test_mode_info.dut_burst_data_flag == TRUE) )
    {
        if (ackd->pkt_src == LC_L2CAP)
        {
            aclq_ack_acl_pkt((HCI_ACL_DATA_PKT *)(ackd->packet_ptr),
                             ackd->frag_len, 0x0, FALSE);
        }
    }
    else if (ackd->pkt_src == LC_L2CAP)
    {
#ifdef TEST_MODE
        if (lmp_self_device_data.host_enable_test_mode == TRUE)
        {
            /* Do not generate num-complete events if the
            device is in DUT mode. */

            aclq_ack_acl_pkt((HCI_ACL_DATA_PKT *)(ackd->packet_ptr),
                             ackd->frag_len, 0x0, FALSE);
            lc_invoke_scheduler(phy_piconet_id);
            return;
        }
#endif /* TEST_MODE */

        aclq_ack_acl_pkt((HCI_ACL_DATA_PKT *)(ackd->packet_ptr),
                         ackd->frag_len, 0x1, FALSE);

        lc_invoke_scheduler(phy_piconet_id);

        return;
    }

    if (ackd->pkt_src == LC_PDU)
    {
        lmp_pkt = (LMP_PDU_PKT *)(ackd->packet_ptr);

        pdu_opcode = (UCHAR) (lmp_pkt->payload_content[0] >> 1);

#ifdef TEST_MODE
        /*
        * Program the baseband to test mode.
        * This is the ack for Lmp_Accepted pdu (for Lmp_Test_Control pdu).
        */
        /* Added the condition to check the lc_test_mode_ack_recd == FALSE.
        * Otherwise, as these data structures are not freed anywhere even
        * after exiting from the test mode, this code will be executed for
        * all the subsequent acks for accepted till these data structures are
        * not updated.
        */

        if ((pdu_opcode == LMP_MAX_POWER_OPCODE) ||
            (pdu_opcode == LMP_MIN_POWER_OPCODE))
        {
            lc_tci_pause_flag = FALSE;
        }

        if ((pdu_opcode == LMP_NOT_ACCEPTED_OPCODE) &&
            (lc_test_mode_ack_recd == FALSE))
        {
            lc_tci_pause_flag = FALSE;
        }

        if ((pdu_opcode == LMP_ACCEPTED_OPCODE) &&
            (lc_test_mode_ack_recd == FALSE))
        {

            /*
            * Added the condition:
            * lmp_connection_entity[ce_index].test_mode_info.test_state == TEST_ACTIVATED,
            * Without this, if we exit from the test mode immediately after entering the
            * test mode, that is without doing any tx or loopback tests, this code will not
            * and the data structures are not updated. So, these will fail when trying to
            * enter test mode second time.
            */
            if ((lmp_self_device_data.test_mode == HCI_DEVICE_UNDER_TEST_MODE) &&
                    (ce_ptr->test_mode_info.test_state == TEST_STARTED)/* ||
                    (ce_ptr->test_mode_info.test_state == TEST_ACTIVATED)*/ )
            {

                lc_test_mode_ack_recd = TRUE;

                lc_tci_pause_flag = FALSE;
                /*
                * Enable Whitening if the test is ACL/SCO Packets without whitening test scenarios.
                * Disable Whitening for the other test scenarios excep Pause Mode Test.
                */

                MT8852B_DBG(GREEN, MT8852B_MSG_TEST_CTRL, 9,
                            ce_ptr->test_mode_info.tc_params.test_scenario,
                            ce_ptr->test_mode_info.tc_params.hopping_mode,
                            ce_ptr->test_mode_info.tc_params.tx_frequency,
                            ce_ptr->test_mode_info.tc_params.rx_frequency,
                            ce_ptr->test_mode_info.tc_params.power_control_mode,
                            ce_ptr->test_mode_info.tc_params.poll_period,
                            ce_ptr->test_mode_info.tc_params.packet_type_in_pdu,
                            ce_ptr->test_mode_info.tc_params.pkt_desc,
                            ce_ptr->test_mode_info.tc_params.num_packets);

                if (ce_ptr->test_mode_info.tc_params.test_scenario
                        != TEST_MODE_PAUSE_TEST_MODE)
                {
                    if ((ce_ptr->test_mode_info.tc_params.test_scenario ==
                            TEST_MODE_ACL_PKTS_WITHOUT_WHITENING) ||
                            (ce_ptr->test_mode_info.tc_params.test_scenario ==
                            TEST_MODE_SCO_PKTS_WITHOUT_WHITENING))
                    {
                        /* TEST_MODE_DISABLE_DATA_WHITENING */
                        /*
                        * disable Data whitening. This has to be done when we send the
                        * LMP_TEST_CONTROL pdu.
                        */
                        OR_val_with_bb_reg(SCO_PACKET_TYPE_REGISTER, 0x0080);
                    }
                    else
                    {
                        /* TEST_MODE_ENABLE_DATA_WHITENING */
                        /*
                        * Enable Data Whitening. Has to be done after Test Control is
                        * complete. Or, on reset. Writes a zero, and enables the Whitening block
                        */
                        AND_val_with_bb_reg(SCO_PACKET_TYPE_REGISTER, 0xff7f);
                    }

                    if (ce_ptr->test_mode_info.tc_params.test_scenario
                            == TEST_MODE_EXIT_TEST_MODE )
                    {
                        /*
                        * Once we get the lc level ack for Exit test mode, go back to the
                        * original state.
                        * This is IMPORTANT. If this is not done, the system will not be in
                        * a correct state, and subsequent connections may not go through
                        * properly.
                        */
                        UINT16 sco_pkt_type_reg;

                        /* TEST_MODE_ENABLE_DATA_WHITENING */
                        /*
                        * Enable Data Whitening. Has to be done after Test Control is
                        * complete. Or, on reset. Writes a zero, and enables the Whitening block
                        */
                        AND_val_with_bb_reg(SCO_PACKET_TYPE_REGISTER, 0xff7f);

                        MINT_OS_ENTER_CRITICAL();
                        /* Switch to 79 hops. */
                        sco_pkt_type_reg = BB_read_baseband_register(SCO_PACKET_TYPE_REGISTER);
                        sco_pkt_type_reg  &= (~0x0060);
                        sco_pkt_type_reg |= 0x20;
                        BB_write_baseband_register(SCO_PACKET_TYPE_REGISTER, sco_pkt_type_reg);

                        AND_val_with_bb_reg(ENCRYPTION_ENABLE_REGISTER,DISABLE_SCO_PKT_LOOPBACK_TX);
                        AND_val_with_bb_reg(ENCRYPTION_ENABLE_REGISTER, (UINT16) (~0x0220));
                        /* Also do this on reset. */
                        AND_val_with_bb_reg(ENCRYPTION_ENABLE_REGISTER,0xfebf);

#ifdef _DUT_DELAYED_LOOPBACK_MODE_
                        dut_mode_manager.buf[0].ppkt = NULL;
                        dut_mode_manager.buf[1].ppkt = NULL;
                        dut_mode_manager.work_space = 0;
#endif

                        test_mode_sched_acl_pkt = FALSE;
                        test_mode_tx_mode_force_ack = FALSE;
                        test_mode_tx_mode_buf_in_used = FALSE;
                        pf_switch_hci_dma_parameter(DUT_TEST_MODE_FIFO_FREE);
                        pf_switch_hci_dma_parameter(LEAVE_TEST_MODE);

                        MINT_OS_EXIT_CRITICAL();

                        ce_ptr->test_mode_info.test_state = TEST_INITIALIZED;

                        /* Reset the test mode, as we are exiting from the test mode.*/
                        lmp_self_device_data.test_mode =
                            lmp_self_device_data.stored_test_mode_state;

                        return;
                    }

                     /* Check if scans are enabled in baseband.
                       These will be temporarily killed. - austin */
                    if ((lc_pagescan_piconet_id != SCA_PICONET_INVALID)
                            || (lc_paging_piconet_id != SCA_PICONET_INVALID)
                       )
                    {
                        lc_kill_scan_mode();
                    }

                    /* Program the hopping Mode */
                    hopping_mode = ce_ptr->test_mode_info.tc_params.hopping_mode;
                    switch(hopping_mode)
                    {
                    case TEST_MODE_SINGLE_FREQUENCY_MODE:
                        MINT_OS_ENTER_CRITICAL();
                        /*
                        * First pgm the number of hops
                        */
                        sco_pkt_type_reg = BB_read_baseband_register(SCO_PACKET_TYPE_REGISTER);

                        /* Isolate the two bits that are used for hop mode */
                        HOP_BITS_ISOLATE(sco_pkt_type_reg);
                        sco_pkt_type_reg |= TEST_MODE_BASEBAND_HOP_1;

                        BB_write_baseband_register(SCO_PACKET_TYPE_REGISTER, sco_pkt_type_reg);
                        MINT_OS_EXIT_CRITICAL();

                        channel_reg = (UINT16)(ce_ptr->test_mode_info.tc_params.tx_frequency);
                        rx_frequency = (UINT16)(ce_ptr->test_mode_info.tc_params.rx_frequency);
                        rx_frequency = (UINT16)(rx_frequency << 8);
                        channel_reg = (UINT16)(rx_frequency | channel_reg);

#ifdef ENABLE_LOGGER_LEVEL_2
                        RT_BT_LOG(GRAY, LC_TASK_4136, 1, channel_reg);
                        RT_BT_LOG(GRAY, LC_TASK_4137, 1, sco_pkt_type_reg);
                        RT_BT_LOG(GRAY, LC_TASK_4138, 0, 0);
#endif

                        BB_write_baseband_register(CHANNEL_REGISTER, channel_reg);

                        break;

                    case TEST_MODE_HOP_79:
                        MINT_OS_ENTER_CRITICAL();
                        sco_pkt_type_reg = BB_read_baseband_register(SCO_PACKET_TYPE_REGISTER);
                        HOP_BITS_ISOLATE(sco_pkt_type_reg);
                        sco_pkt_type_reg |= TEST_MODE_BASEBAND_HOP_79;
                        BB_write_baseband_register(SCO_PACKET_TYPE_REGISTER, sco_pkt_type_reg);
                        MINT_OS_EXIT_CRITICAL();

#ifdef ENABLE_LOGGER_LEVEL_2
                        RT_BT_LOG(GRAY, LC_TASK_4154, 1, sco_pkt_type_reg);
#endif
                        break;

                    default:
#ifdef ENABLE_LOGGER_LEVEL_2
                        /* Control must not come here */
                        RT_BT_LOG(GRAY, LC_TASK_4161, 0, 0);
#endif
                        break;

                    } /* End switch(hopping_mode) */

#ifdef COMPILE_ESCO
                    if ((ce_ptr->test_mode_info.tc_params.pkt_desc ==
                            PACKET_TYPE_DESCRIPTION_BR_eSCO) ||
                            (ce_ptr->test_mode_info.tc_params.pkt_desc ==
                             PACKET_TYPE_DESCRIPTION_EDR_eSCO))
                    {
                        BB_write_baseband_register(ENCRYPTION_ENABLE_REGISTER, 0x0220);
                    }
#endif

 #ifdef _DUT_DELAYED_LOOPBACK_MODE_
                    if (dut_mode_manager.buf[0].ppkt == NULL)
                    {
                        OS_ALLOC_BUFFER(tx_table[ACL_DATA_HANDLER_TASK].pool_handle,
                                        (void **)(&dut_mode_manager.buf[0].ppkt));
                    }
                    if (dut_mode_manager.buf[1].ppkt == NULL)
                    {
                        OS_ALLOC_BUFFER(tx_table[ACL_DATA_HANDLER_TASK].pool_handle,
                                        (void **)(&dut_mode_manager.buf[1].ppkt));
                    }
                    dut_mode_manager.work_space = 0;
#endif

                    lc_is_tx_test_mode = FALSE;

                    switch(ce_ptr->test_mode_info.tc_params.test_scenario)
                    {
                    case TEST_MODE_TRANSMITTER_TEST_ZERO_PATTERN:
                        /* Fall through */
                    case TEST_MODE_TRANSMITTER_TEST_ONE_PATTERN:
                        /* Fall through */
                    case TEST_MODE_TRANSMITTER_TEST_1010_PATTERN:
                        /* Fall through */
                    case TEST_MODE_PSEUDORANDOM_BIT_SEQUENCE:
                        /* Fall through */
                    case TEST_MODE_TRANSMITTER_TEST_11110000_PATTERN:

#ifdef ENABLE_LOGGER_LEVEL_2
                        /* Generate Test Burst Data and Tx to the Test Control Master */
                        RT_BT_LOG(GRAY, LC_TASK_4203, 0, 0);
#endif
                        lc_is_tx_test_mode = TRUE;

                        /* TEST_MODE_DISABLE_DATA_WHITENING */
                        /*
                        * disable Data whitening. This has to be done when we send the
                        * LMP_TEST_CONTROL pdu.
                        */
                        OR_val_with_bb_reg(SCO_PACKET_TYPE_REGISTER, 0x0080);

                        lc_generate_and_send_dut_burst_data(ce_index, phy_piconet_id);

                        test_mode_tx_mode_buf_in_used = TRUE;
                        test_mode_tx_mode_force_ack = TRUE;
                        break;

                    case TEST_MODE_ACL_PKTS_WITHOUT_WHITENING:
                        /* Fall thru */
                    case TEST_MODE_SCO_PKTS_WITHOUT_WHITENING:
                        /* Fall thru */
                    case TEST_MODE_CLOSED_LOOPBACK_ACL_PKTS:
                        /* Fall thru */
                    case TEST_MODE_CLOSED_LOOPBACK_SCO_PKTS:
                        /* From now on loop back any data received */
                        lmp_self_device_data.test_mode = HCI_DUT_LOOPBACK_MODE;
                        OR_val_with_bb_reg(ENCRYPTION_ENABLE_REGISTER,0x0100);
                        AND_val_with_bb_reg(ENCRYPTION_ENABLE_REGISTER, (0xFFBF) );

#ifdef _RTL8723B_DUT_MODE_
                        /* enable loopback mode */
//                        BB_write_baseband_register(RADIO_SELECT_REGISTER, 0x0127);
#endif
                        break;

                    case TEST_MODE_EXIT_TEST_MODE:
                        /* Fall thru */
                    case TEST_MODE_PAUSE_TEST_MODE:
                        /* Do nothing */
                        break;

                    default:
                        break;
                    } /* End switch() */
                }
                else /* TEST_MODE_PAUSE_TEST_MODE */
                {
                    /* Restore PTT bit in the LUT. */
                    if (ce_ptr->ptt_status == LMP_PTT_ENABLED)
                    {
                        lc_enable_ptt_bit(ce_index);
                    }
                    else
                    {
                        lc_disable_ptt_bit(ce_index);
                    }
                } /* End if (test_scenario != TEST_MODE_PAUSE_TEST_MODE) */
            }
        } /* End if ( pdu_opcode == LMP_ACCEPTED_OPCODE ) */

#ifdef _RTL8723B_DUT_MODE_
        if ((ce_ptr->test_mode_info.test_state == TEST_STARTED) &&
            (ce_ptr->test_mode_info.tc_params.test_scenario != TEST_MODE_PAUSE_TEST_MODE))
        {
            if (lc_tci_pause_flag == FALSE)
            {
                if (lmp_self_device_data.test_mode == HCI_DUT_LOOPBACK_MODE)
                {
                    /* enable loopback mode */
                    BB_write_baseband_register(RADIO_SELECT_REGISTER, 0x0127);
                }
                else if (lmp_self_device_data.test_mode == HCI_DEVICE_UNDER_TEST_MODE)
                {
                    /* enable tx mode */
                    BB_write_baseband_register(RADIO_SELECT_REGISTER,
                                               test_mode_tx_mode_reg_catch);
                }
            }
        }
#endif


#endif /* TEST_MODE */

        sig_send.type = LMP_PDU_ACK_RECD_SIGNAL;
        sig_send.param =  (OS_ADDRESS) lmp_pkt ;

        sig_send.ext_param = (OS_ADDRESS)(debug_clock | (phy_piconet_id << 30));

        if ((OS_SEND_SIGNAL_TO_TASK(lmp_task_handle, sig_send ) != BT_ERROR_OK))
        {
#ifdef ENABLE_LOGGER_LEVEL_2
            RT_BT_LOG(GRAY, LC_TASK_4281, 0, 0);
#endif
        }

        lc_invoke_scheduler(phy_piconet_id);
        return;

    } /* switch */

}

/**
* Validates the parameters of the ACL packet received from the host,
* and queues it in the baseband if there is a free scheduler entry
* available.
*
* \param pkt Pointer to the ACL packet received from the host.
*
* \return None.
*/
void lc_handle_host_data_pkt(HCI_ACL_DATA_PKT *pkt)
{
    UINT16 connection_handle;
    UCHAR packet_boundry_flag;
    UCHAR broad_cast_flag;
    UINT16 ce_index = INVALID_CE_INDEX;
    API_RESULT result;
    UCHAR drop_reason = 0;
    UCHAR *pkt_buf = NULL;
    UINT8 piconet_id;

    /* Extract boundry flag and broadcast flag for the packet */
    packet_boundry_flag = (UCHAR)pkt->packet_boundary_flag;
    broad_cast_flag = (UCHAR)pkt->broadcast_flag;

    /*Extract connection handle from the packet */
    connection_handle = (UINT16) pkt->connection_handle;

    do
    {
        /* Check for BC flag */
        if ( (broad_cast_flag > LC_PNET_BC_FLAG))
        {
            drop_reason |= BIT0;
            break;
        }
#ifndef _DAPE_TEST_NEW_HCI_ACL_SPEC
        /* Check for PB flag */
        if (packet_boundry_flag == L_CH_LMP)
        {
            drop_reason |= BIT1;
            break;
        }
#endif
        /* Check for packet size */
        if (pkt->acl_data_total_length > otp_str_data.bt_read_buffer_size)
        {
            drop_reason |= BIT3;
            break;
        }

        /* Get the ce index from connection handle */
        result = LMP_GET_CE_INDEX_FROM_CONN_HANDLE(connection_handle, &ce_index);

        if (result == API_SUCCESS)
        {
            if (ce_index < LMP_MAX_CE_DATABASE_ENTRIES)
            {
                if (((broad_cast_flag == LC_ACTIVE_BC_FLAG) ||
                	(broad_cast_flag == LC_PNET_BC_FLAG)) &&
                	(lmp_connection_entity[ce_index].ce_status != LMP_STANDBY))
                {
                /* If its a broadcast packet and connection entity is assigned for
                * this packet it means host is trying to send broadcast data at
                * ptop connection handle
                */
                    /* host must use different connection handle for
                       broadcast */
                    drop_reason |= BIT6;
                    break;
                }
            }
            else
            {
                drop_reason |= BIT2;
                break;
            }
        }

        if (aclq_queue_acl_pkt(pkt) == API_SUCCESS)
        {
            piconet_id = pkt->ws->phy_piconet_id;

            if (lc_piconet_scheduler[piconet_id].lc_allowed_pkt_cnt != 0)
            {
                lc_invoke_scheduler(piconet_id);
            }
        }
        else
        {
            drop_reason |= BIT5;
            break;
        }
    }
    while (0);

    if (drop_reason != 0)
    {
        /* Free the packet buffer and send H\W error event to host */
        dma_tx_fifo_pkt_free((void *)pkt,
                                 HCI_TRANSPORT_ACL_DATA_PKT_TYPE);
#ifdef _DAPE_TEST_DONT_SEND_ERR_EVENT_TO_HOST
        if (IS_HARDWARE_ERR_EVENT_TO_HOST_WHEN_HOST_ACL)
#endif
        {
            /* generate and send hw error event */
            OS_ALLOC_BUFFER(tx_table[HCI_EVENT_HANDLER_TASK].pool_handle,
                            (void **)(&pkt_buf));
            pkt_buf[0] = 0x10;
            pkt_buf[1] = 0x01;
            pkt_buf[2] = PF_HCI_TRANSPORT_HW_ERROR_WRONG_PARAM_FOR_ACL_DATA;

            hci_td_deliver_event_to_host(pkt_buf);
            //RT_BT_LOG(RED, EVENT_SENT_TO_HCI_TD_TASK_EVENT_TYPE_PARAM_LENGTH,2,pkt_buf[2],pkt_buf[1]);
        }
        RT_BT_LOG(RED, MSG_ACL_HOST_DATA_PKT_ERR, 1, drop_reason);
    }

    return;
}
#ifdef _DAPE_TEST_NEW_HW_INSTRUCTION_KILL_SCAN
void lc_kill_inquiry_instruction(void)
{
    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_KILL_INQUIRY);

#ifdef _FREE_FHS_PKT_WHEN_KILL_INQUIRY_
    lc_free_fhs_pkt_buffer();
#endif
}

void lc_kill_inquiry_scan_instruction(void)
{

    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_KILL_INQUIRY_SCAN);

#ifdef _DAPE_TEST_NEW_HW_INSTRUCTION_KILL_SCAN_FOR_SAVE
    /* dape: add this two command in 8821 to keep the fw behavior the
       same with 8723a, which has lots of IOT test.
       maybe we should modify it in 8723B. */
    lc_kill_inquiry_instruction();
#endif

}

void lc_kill_page_scan_instruction(void)
{
    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_KILL_PAGE_SCAN);
    lc_pagescan_piconet_id = SCA_PICONET_INVALID;

#ifdef _DAPE_TEST_NEW_HW_INSTRUCTION_KILL_SCAN_FOR_SAVE
    /* dape: add this two command in 8821 to keep the fw behavior the
       same with 8723a, which has lots of IOT test.
       maybe we should modify it in 8723B. */
    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_KILL_PAGE);
    lc_paging_piconet_id = SCA_PICONET_INVALID;
#endif
}
#endif

/**
* Kills the scans in the baseband.
*
* \param None.
*
* \return None.
*/
void lc_kill_scan_mode(void)
{
#ifdef _ROM_CODE_PATCHED_
#ifdef _DAPE_8723_RCP_
    if (rcp_lc_kill_scan_status_func != NULL)
    {
        UINT8 return_status;
        rcp_lc_kill_scan_status_func((void *)&return_status);
        if(return_status)
        {
            return;
        }
    }
#endif
#endif

    DEF_CRITICAL_SECTION_STORAGE;

    MINT_OS_ENTER_CRITICAL();

    /* Kill all scan procedures in the baseband */

#ifdef _DAPE_TEST_NEW_HW_INSTRUCTION_KILL_SCAN
    lc_kill_inquiry_scan_instruction();
    lc_kill_page_scan_instruction();
#else

#ifndef _DAPE_TEST_NEW_HW_INSTRUCTION
    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_KILL_PAGE);
    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_KILL_INQUIRY);
#else
    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_KILL_PAGE_SCAN);
    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_KILL_INQUIRY_SCAN);

#ifdef _DAPE_TEST_NEW_HW_INSTRUCTION_KILL_SCAN_FOR_SAVE
    /* dape: add this two command in 8821 to keep the fw behavior the
       same with 8723a, which has lots of IOT test.
       maybe we should modify it in 8723B. */
    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_KILL_PAGE);
    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_KILL_INQUIRY);
#endif
    //RT_BT_LOG(BLUE, DAPE_TEST_LOG293, 1, lmp_self_device_data.lc_cur_dev_state);
#endif

    lc_pagescan_piconet_id = SCA_PICONET_INVALID;
    lc_paging_piconet_id = SCA_PICONET_INVALID;
#endif

/* (josh) When role switch, bit 13 of PAGE_SCAN_WINDOW_REGISTER might be set to 1 to trigger
 * slot_counter_interrupt. However, following codes will set bit 13 to 0, as a result,
 * role switch will not know how to continue. Hence disable the following codes.
 * You can also refer to jira BTFWHW-70.
 */
#if 0
#ifdef _MODI_LPS_AFTER_RTL8703B_
    sleep_mode_param.scan_end_flag = 0;

    BZ_REG_S_INQ_SCAN_WINDOW isw;
    *(UINT16*)&isw = BB_read_baseband_register(INQ_SCAN_WINDOW_REGISTER);
    isw.is_int_en = 0;
    BB_write_baseband_register(INQ_SCAN_WINDOW_REGISTER, *(UINT16*)&isw);


    BZ_REG_S_PAGE_SCAN_WINDOW psw;
    *(UINT16*)&psw = BB_read_baseband_register(PAGE_SCAN_WINDOW_REGISTER);
    psw.ps_int_en = 0;
    BB_write_baseband_register(PAGE_SCAN_WINDOW_REGISTER, *(UINT16*)&psw);
#endif
#endif

#ifdef _DAPE_TEST_KILL_SCAN_STATUS_MODIFY
    do
    {
        if ((lmp_self_device_data.lc_cur_dev_state == LC_CUR_STATE_PAGE) &&
            (lc_cur_connecting_am_addr != INVALID_AM_ADDR))
        {
            UINT16 ce_index;
            LMP_CONNECTION_ENTITY* ce_ptr;

            UCHAR am_addr = 0xFF;
            UCHAR phy_piconet_id;
            UINT32 cur_clk;
            UCHAR paging_bd_addr[6];

            paging_bd_addr[0] = lc_paging_bd_addr[0] & 0xFF;
            paging_bd_addr[1] = (lc_paging_bd_addr[0]>>8) & 0xFF;
            paging_bd_addr[2] = lc_paging_bd_addr[1] & 0xFF;
            paging_bd_addr[3] = (lc_paging_bd_addr[1]>>8) & 0xFF;
            paging_bd_addr[4] = lc_paging_bd_addr[2] & 0xFF;
            paging_bd_addr[5] = (lc_paging_bd_addr[2]>>8) & 0xFF;

            if (API_FAILURE == LMP_GET_CE_INDEX_FROM_BD_ADDR((UCHAR *)&paging_bd_addr[0],
                                      (UINT16*)&ce_index))
            {
                break;
            }

            ce_ptr = &lmp_connection_entity[ce_index];
            phy_piconet_id = ce_ptr->phy_piconet_id;
            lc_get_clock_in_scatternet(&cur_clk, phy_piconet_id);
            lc_set_lc_cur_device_state(LC_CUR_STATE_IDLE);
            am_addr = lc_cur_connecting_am_addr;
            if (((lc_sca_manager.master_cnt) && (lc_sca_manager.master_id != phy_piconet_id))
    			||(lc_sca_manager.master_cnt==0))
            {
                AND_val_with_bb_reg(reg_PICONET_INFO[phy_piconet_id], SCA_PICONET_SLAVE);
            }
            lc_set_lc_cur_connecting_am_addr(INVALID_AM_ADDR);
            /* dape: function call instead of sending task to avoid
               lmp_self_device_data.device_status is no longer PAGE anymore.
               ex: periodic inq->kill page.*/
            lmp_handle_page_timeout(am_addr,phy_piconet_id);

#if 0
            RT_BT_LOG(YELLOW, DAPE_TEST_LOG206, 12,
                        cur_clk,
                        BB_read_baseband_register(PAGE_TIMEOUT_REGISTER),
                        am_addr, phy_piconet_id,
                        paging_bd_addr[0],
                        paging_bd_addr[1],
                        paging_bd_addr[2],
                        paging_bd_addr[3],
                        paging_bd_addr[4],
                        paging_bd_addr[5],
                        ce_index,
                        BB_read_baseband_register(reg_PICONET_INFO[phy_piconet_id]));
#endif
        }

        if (lmp_self_device_data.lc_cur_dev_state == LC_CUR_STATE_INQUIRY)
        {
            OS_SIGNAL sig_send;
            sig_send.type = LMP_HANDLE_INQUIRY_TIMEOUT_SIGNAL;
            sig_send.param = (void *)NULL;
            OS_ISR_SEND_SIGNAL_TO_TASK (lmp_task_handle, sig_send );
#if 0
            UINT32 cur_clk;
            cur_clk = BB_read_native_clock();
            RT_BT_LOG(YELLOW, ITO_MSG_INQUIRY_TIMEOUT, 2,
                cur_clk, BB_read_baseband_register(INQUIRY_TIMEOUT_REGISTER));
#endif
    	}
    }
    while (0);
#endif
    MINT_OS_EXIT_CRITICAL();

#ifdef SCNET_DEBUG
    RT_BT_LOG(GRAY, LC_TASK_4493, 0, 0);
#endif

    return;
}

#ifdef COMPILE_ROLE_SWITCH
/**
* Processes the successful MSS completion signal from the baseband.
* Starts/stops tpoll based on the new role. Also resets the
* lc-state-machine for MSS.
*
* \param None.
*
* \return None.
*/
void lc_handle_role_switch_complete_in_scatternet()
{
    UINT16 ce_index;
    LMP_CONNECTION_ENTITY *ce_ptr;
    UINT16 piconet_status = PICONET1_INFO_REGISTER;
    UCHAR old_am_addr;
    UCHAR new_am_addr;
    UCHAR old_lut_index;
    UCHAR new_lut_index;
    UCHAR old_piconet_id;
    UCHAR new_piconet_id;
    LUT_EXTENSION_TABLE *ex_lut;

    DEF_CRITICAL_SECTION_STORAGE;

    ce_index = lmp_role_switch_data.ce_index;
    old_am_addr = lmp_role_switch_data.old_am_addr;
    new_am_addr = lmp_role_switch_data.new_am_addr;

    old_piconet_id = lmp_role_switch_data.old_piconet_id;
    new_piconet_id = lmp_role_switch_data.new_piconet_id;

    old_lut_index = lmp_role_switch_data.old_lut_index;
    new_lut_index = lmp_role_switch_data.new_lut_index;

    /* This function resets the LC data structures. */

    ce_ptr = &lmp_connection_entity[ce_index];

    if (new_piconet_id <= SCA_PICONET_MAX)
    {
        piconet_status = reg_PICONET_INFO[new_piconet_id];
    }
    else
    {
        RT_BT_LOG(RED, PICONET_ID_CHECK_MSG, 1, new_piconet_id);
    }
#ifndef _ENABLE_COMPILE_NESTED_PAUSE_RESUME_
    lc_pause_schedule_pkts = 0x0;
#endif
    MINT_OS_ENTER_CRITICAL();
    /* ce_ptr->remote_dev_role still shows the old role. */
    if (ce_ptr->remote_dev_role == SLAVE)
    {
        /* new role:slave; old-role:master */

        BB_write_baseband_register(piconet_status,
            (UINT16)(((new_am_addr) << 1) | (ce_ptr->remote_dev_role ^ 1)));

        BB_write_baseband_register(INSTRUCTION_REGISTER, BB_EXECUTE);
    }
    else
    {
        /* new role:master; old-role: slave */

        /* Write Zero am_addr for master case. */
        BB_write_baseband_register(piconet_status, (ce_ptr->remote_dev_role ^ 1));
    }

    BB_write_baseband_register(CONNECTOR_REGISTER, (UINT16)
                       ( ((new_am_addr) << 5) | (new_piconet_id << 11) ));
    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_EXECUTE);

    OR_val_with_bb_reg(
        lut_ex_table[new_lut_index].upper_lut_address, ACTIVE_BIT);
    ex_lut = &lut_ex_table[new_lut_index];

    lc_init_seqn_scatternet(lmp_role_switch_data.new_am_addr,
                            ex_lut, lmp_role_switch_data.new_piconet_id);

    MINT_OS_EXIT_CRITICAL();

    /* Update the scatternet status. */
    lc_update_scatternet_status_after_mss(ce_index);

#ifdef COMPILE_AFH_HOP_KERNEL
    /* Stop the OS timer, and delete it if it is running. */
    if (ce_ptr->afh_instant_timer_handle != NULL)
    {
        OS_DELETE_TIMER( & ce_ptr->afh_instant_timer_handle);
    }
#endif

    aclq_requeue_all_pkts();
    pduq_requeue_all_pdus();

    /* Release the old-lut-index and initialise the new-lut-index. */
    lc_reset_lut_ex_table(old_lut_index);

    lc_update_ce_index_to_lut_extn_tbl(ce_index, new_lut_index);

    /* Update the pkt type in LUT after MSS. */
    BB_write_baseband_register(lut_ex_table[old_lut_index].lower_lut_address,
                               LC_INVALID_PACKET_TYPE);

#ifndef _CCH_IOT_CSR_NO_NULL_
    BB_write_baseband_register(lut_ex_table[new_lut_index].lower_lut_address,
                               LC_SLAVE_DEFAULT_PACKET_TYPE);
#else
    BB_write_baseband_register(lut_ex_table[new_lut_index].lower_lut_address,
                               LC_INVALID_PACKET_TYPE);
#endif

    BB_start_tpoll(new_am_addr, ce_ptr->Tpoll, new_piconet_id);

    BB_disable_afh(new_am_addr, new_piconet_id);

    LC_RESET_PTT_BIT_IN_LUT(lut_ex_table[old_lut_index].upper_lut_address);

    if (ce_ptr->ptt_status == LMP_PTT_ENABLED)
    {
        LC_SET_PTT_BIT_IN_LUT(lut_ex_table[new_lut_index].upper_lut_address);
    }
    else
    {
        LC_RESET_PTT_BIT_IN_LUT(lut_ex_table[new_lut_index].upper_lut_address);
    }

    /* Clear active bit in old piconet. */
    AND_val_with_bb_reg(lut_ex_table[old_lut_index].upper_lut_address, (UINT16)(~ACTIVE_BIT));

    /* Set active bit in new piconet. */
    OR_val_with_bb_reg(lut_ex_table[new_lut_index].upper_lut_address, ACTIVE_BIT);

#ifdef _RESET_HW_LINK_INFO_TO_INIT_STATE_
    /* 1. install key
       2. copy encrption state from old am_addr/piconet
          to new am_addr/piconet */
    if (ce_ptr->enc_opcode != BB_ENC_TX_DISBALE_RX_DISABLE)
    {
        BZ_AUTH_LINK_PARAMS* auth = ce_ptr->auth;
#ifdef _CCH_SC_ECDH_P256_
        if(auth->secure_conn_enabled)
        {
#ifdef _CCH_SC_ECDH_P256_
            BB_write_sc_cam_ini(ce_index);
#endif
            BB_write_sc_encryption_keys(ce_index,
                             ce_ptr->auth->enc_key);
#ifdef _CCH_SC_ECDH_P256_PAUSE_ENC
            if(auth->sc_use_enc_rand)
            {
#ifdef _CCH_SC_ECDH_P256_LOG
                RT_BT_LOG(BLUE, CCH_DBG_155, 0, 0);
#endif

#ifdef _CCH_SC_ECDH_P256_LOG
    RT_BT_LOG(BLUE, YL_DBG_HEX_4, 4, auth->enc_rand[0], auth->enc_rand[1], auth->enc_rand[2], auth->enc_rand[3]);
#endif

#ifdef _CCH_SC_TEST_20130129_QCA_02
            UCHAR temp[BZ_AUTH_ENC_RAND_SIZE];
            memcpy(&temp[0], &auth->enc_rand[0], BZ_AUTH_ENC_RAND_SIZE);
            bz_auth_convert_to_msb(&temp[0], 16);
            BB_write_sc_iv(ce_index, &temp[0]);
#else
#ifdef _CCH_SC_TEST_20130129_QCA_04
#ifdef _CCH_SC_TEST_20130129_QCA_05
#ifdef _DAPE_TEST_CHK_SC_ROLE_SW
RT_BT_LOG(RED, DAPE_TEST_LOG293, 1,ce_ptr->remote_dev_role);
#endif

            if(ce_ptr->remote_dev_role == MASTER )
            {
                BB_write_sc_iv(ce_index, &auth->enc_rand_remote[8]);
            }
            else
            {
                BB_write_sc_iv(ce_index, &auth->enc_rand[8]);
            }
#else
            BB_write_sc_iv(ce_index, &auth->enc_rand[8]);
#endif
#else
            BB_write_sc_iv(ce_index, &auth->enc_rand[0]);
#endif
#endif

            }
            else
            {
                RT_BT_LOG(RED, CCH_DBG_019, 0, 0);
            }
#endif
        }
        else
#endif
        {
            BB_write_encryption_keys(new_am_addr, new_piconet_id, auth->enc_key);
        }
    }

    BB_encryption_control(new_am_addr, new_piconet_id, ce_ptr->enc_opcode);

    /* must check am_addr and piconet_id to avoid no change anything */
    if ((new_am_addr != old_am_addr) || (new_piconet_id != old_piconet_id))
    {
        /* reset encrption state for old am_addr/piconet */
        BB_encryption_control(old_am_addr, old_piconet_id,
                          BB_ENC_TX_DISBALE_RX_DISABLE);
    }
#endif

    if (lmp_self_device_data.lc_no_of_connections[old_piconet_id] == 0)
    {
        lc_reset_lc_piconet_scheduler(old_piconet_id);
    }

#if 0
    ///// dape added for check lmp & os timer after role switch ////
    UINT16 pdu_timer_running = ce_ptr->pdu_response_timer_running;
    UINT32 os_timer_running = FALSE;

    os_timer_running = OS_IS_TIMER_RUNNING(ce_ptr->lmp_response_timer_handle);
    RT_BT_LOG(YELLOW, DAPE_TEST_LOG439, 6,
              pdu_timer_running, os_timer_running,
              ce_ptr->sent_pdu,
              ce_ptr->sent_lmp_pdu_opcode,
              ce_ptr->sent_lmp_pdu_ext_opcode,
              ce_ptr->lmp_expected_pdu_opcode);
    /////////////////////////////////////////////////////////////////
#endif

    return;
}

#endif /* COMPILE_ROLE_SWITCH */

#ifdef TEST_MODE

/**
* Composes packets for transmitter test.
*
* \param ce_index Index to lmp-connection-entity of the connection.
* \param piconet_id Physical piconet ID of the connection.
*
* \return None.
*/
void lc_generate_and_send_dut_burst_data(UINT16 ce_index, UCHAR piconet_id)
{
    UINT16 burst_data_len;
    UCHAR test_scenario;
    UCHAR test_pattern = 0;
    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    burst_data_len = ce_ptr->test_mode_info.tc_params.num_packets;
    if (burst_data_len == 0)
    {
        /* Need not send Burst Data */
#ifdef ENABLE_LOGGER_LEVEL_2
        LC_LOG_INFO(LOG_LEVEL_LOW, BURST_DATA_LENGTH_IS_ZERO_DATA_NOT_GENERATED,0,0);
#endif

        return;
    }

    test_scenario = ce_ptr->test_mode_info.tc_params.test_scenario;
    ce_ptr->test_mode_info.dut_burst_data_flag = TRUE;

#ifdef _RTL8723B_DUT_MODE_
    /* Form the test pattern based on test scenario */
    switch(test_scenario)
    {
        case TEST_MODE_TRANSMITTER_TEST_ZERO_PATTERN:
            test_pattern = 0;
            break;

        case TEST_MODE_TRANSMITTER_TEST_ONE_PATTERN:
            test_pattern = 4;
            break;

        case TEST_MODE_TRANSMITTER_TEST_1010_PATTERN:
            /* This sequence can work with Arnitsu's MT8852B and R&S's CBT */
            test_pattern = 1;
            break;

        case TEST_MODE_PSEUDORANDOM_BIT_SEQUENCE:
            test_pattern = 3;
            break;

        case TEST_MODE_TRANSMITTER_TEST_11110000_PATTERN:
            /* This sequence can work with Arnitsu's MT8852B and R&S's CBT */
            test_pattern = 2;
            break;

        default:
#ifdef ENABLE_LOGGER_LEVEL_2
            LC_LOG_INFO(LOG_LEVEL_LOW, INVALID_TEST_SCENARIO,0,0);
            LC_LOG_INFO(LOG_LEVEL_LOW, UNABLE_TO_FILL_TEST_PATTERN_FOR_BURST_DATA,0,0);
#endif
            return;
    }

    /* update the catch of register content */
    test_mode_tx_mode_reg_catch = 0x0147 | (test_pattern << 9);

    /* enable tx mode */
//    BB_write_baseband_register(RADIO_SELECT_REGISTER,
//                               test_mode_tx_mode_reg_catch);

#else /* else of #ifdef _RTL8723B_DUT_MODE_ */

    /* Form the test pattern based on test scenario */
    switch(test_scenario)
    {
        case TEST_MODE_TRANSMITTER_TEST_ZERO_PATTERN:
            test_pattern = 0x00;
            break;

        case TEST_MODE_TRANSMITTER_TEST_ONE_PATTERN:
            test_pattern = 0xff;
            break;

        case TEST_MODE_TRANSMITTER_TEST_1010_PATTERN:
            /* This sequence can work with Arnitsu's MT8852B and R&S's CBT */
            test_pattern = 0x55;
            break;

        case TEST_MODE_PSEUDORANDOM_BIT_SEQUENCE:
            break;

        case TEST_MODE_TRANSMITTER_TEST_11110000_PATTERN:
            /* This sequence can work with Arnitsu's MT8852B and R&S's CBT */
            test_pattern = 0x0f;
            break;

        default:
#ifdef ENABLE_LOGGER_LEVEL_2
            LC_LOG_INFO(LOG_LEVEL_LOW, INVALID_TEST_SCENARIO,0,0);
            LC_LOG_INFO(LOG_LEVEL_LOW, UNABLE_TO_FILL_TEST_PATTERN_FOR_BURST_DATA,0,0);
#endif
            return;
    }

#ifdef _DUT_DELAYED_LOOPBACK_MODE_
    UINT8 *dest = dut_mode_manager.buf[0].ppkt->hci_acl_data_pkt;
#else
    UINT8 *dest = lmp_test_mode_data_buf;
#endif

    /* do not memory copy once to generate test patterns */
    ce_ptr->test_mode_info.tc_params.test_pattern = test_pattern;

    /* Fill the pseudo random bit sequence pattern */
    if (test_scenario == TEST_MODE_PSEUDORANDOM_BIT_SEQUENCE)
    {
        UINT16 i;
        UINT8 j;
        UINT32 *prbs9_dw;
        UINT8 shift_right;
        UINT32 left_part;
        UINT16 iub;

        prbs9_dw = (UINT32*)prbs9_bite_stream_bytes;

        /* fast PBRS9 payload generator - austin */
        shift_right = 0;
        j = 0;
        iub = burst_data_len + 4;

        for (i = 0; i < iub; i += 4)
        {
            if (shift_right != 0)
            {
                /* memory copy after bit 0 ~ 511 all shift right N */
                *(UINT32*)&dest[i] = prbs9_dw[j] >> shift_right;
                left_part = prbs9_dw[j + 1] & ((1 << shift_right) - 1);
                *(UINT32*)&dest[i] |= left_part << (32 - shift_right);
            }
            else
            {
                *(UINT32*)&dest[i] = prbs9_dw[j];
            }

            j++;

            if (j == 16)
            {
                /* bit copy from next bit stream  (bit 0 ~ N) */
                shift_right++;
                left_part = prbs9_dw[0] & ((1 << shift_right) - 1);
                *(UINT32*)&dest[i] |= left_part << (32 - shift_right);
                j = 0;
            }
        }
    }
    else
    {
        memset(dest, test_pattern, burst_data_len);
    }

#ifdef ENABLE_LOGGER_LEVEL_2
    LC_LOG_INFO(LOG_LEVEL_LOW, WRITING_BURST_DATA_TO_FIFO,0,0);
#endif

#endif /* end of #ifdef _RTL8723B_DUT_MODE_ */
 }

#endif /* TEST_MODE */

/**
* Sets the value of lc_cur_device_state.
*
* \param new_state The new state of the device.
*
* \return None.
*/
INLINE void lc_set_lc_cur_device_state(UCHAR new_state)
{
    lmp_self_device_data.lc_cur_dev_state = new_state;

    return;
}

#ifdef COMPILE_SNIFF_MODE

/*  lc_sniff_sw_timer-handler
*
* \param timer_handle Timer handle.
* \param index ACL connection entity index.
*
* \return None.
*/
void lc_sniff_mode_timer_handler(TimerHandle_t timer_handle)
{
    UINT16 ce_index;
    OS_SIGNAL sig_send;
    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_index = (UINT16)((UINT32)pvTimerGetTimerID(timer_handle));
    ce_ptr = &lmp_connection_entity[ce_index];

    aclq_mark_am_addr_as_active(ce_ptr->am_addr, ce_ptr->phy_piconet_id);

    sig_send.type = LC_RESUME_DATA_TRANSFER_SIGNAL;
    sig_send.param = (void*)((UINT32)(ce_ptr->phy_piconet_id));

    OS_SEND_SIGNAL_TO_TASK(lc_tx_task_handle, sig_send);
}

/**
* Start the lc_sniff_mode_timer. Timeout handler: lc_sniff_mode_timer_handler.
*
* \param ce_index Connection entity index.
*
* \return None.
*/
API_RESULT lc_start_sniff_mode_timer(UINT16 ce_index, UINT16 duration)
{
    LMP_CONNECTION_ENTITY *ce_ptr;
    UINT16 timer_val;
    ce_ptr = &lmp_connection_entity[ce_index];

    timer_val = SLOT_VAL_TO_TIMER_VAL(duration);

    if (timer_val < 10)
    {
        timer_val = 10;
    }

    if (ce_ptr->sniff_sw_timer_handle == NULL)
    {
        if (OS_CREATE_TIMER(ONESHOT_TIMER, &ce_ptr->sniff_sw_timer_handle,
                lc_sniff_mode_timer_handler, (void *)((UINT32)ce_index),
                (UINT16) timer_val) != BT_ERROR_OK)
        {
            return API_FAILURE;
        }
    }

    if (OS_START_TIMER(ce_ptr->sniff_sw_timer_handle, timer_val) != BT_ERROR_OK)
    {
        return API_FAILURE;
    }

    return API_SUCCESS;
}

/**
* Optmizes data transfer during sniff mode, by marking this connection as
* BAD, till the timer fires.
*
* \param ce_index Connection entity index.
*
* \return None.
*/
void lc_optimize_data_transfer_during_sniff(UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    UINT32 sniff_interval;

    ce_ptr = &lmp_connection_entity[ce_index];
    sniff_interval = ce_ptr->sniff_interval;

    if (ce_ptr->ssr_data.lc_ssr_state == LC_SSR_SUBRATE)
    {
        sniff_interval *= lmp_calculate_sniff_subrate(ce_index);
    }

    if ((ce_ptr->remote_dev_role == SLAVE) &&
            sniff_interval > LMP_MIN_SNIFF_INTERVAL_TO_SKIP_DATA_TX_OPT)
    {
        if (lc_start_sniff_mode_timer(ce_index,
                 (sniff_interval - ((ce_ptr->sniff_attempt) * 2) -
                  LMP_MIN_SNIFF_INTERVAL_TO_SKIP_DATA_TX_OPT)) == API_SUCCESS)
        {
            aclq_mark_am_addr_as_failed(ce_ptr->am_addr, ce_ptr->phy_piconet_id);
        }
    }

    return;
}
#endif

void lc_handle_and_check_sw_scheduler(UINT8 phy_piconet_id)
{
    if (lc_piconet_scheduler[phy_piconet_id].lc_allowed_pkt_cnt)
    {
        lc_invoke_scheduler(phy_piconet_id);
    }
}

