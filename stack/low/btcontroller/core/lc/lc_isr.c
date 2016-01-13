/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  Contains functions to handle all baseband interrupts.
 */

/********************************* Logger *************************/
enum { __FILE_NUM__= 39 };
/********************************* Logger *************************/

/* ========================= Include File Section ========================= */
#include "lc_internal.h"
#include "lmp.h"
#include "lc.h"
#include "bt_fw_os.h"
#ifdef _CCH_LPS_
#include "mint_os_timer_internal.h"
#endif
#include "led_debug.h"
#include "bz_debug.h"
#ifdef COMPILE_CHANNEL_ASSESSMENT
#include "lmp_ch_assessment.h"
#endif /* COMPILE_CHANNEL_ASSESSMENT */
#include "hci_vendor_defines.h"
#include "vendor.h"
#include "UartPrintf.h"
#include "lmp_2_1.h"
#include "bt_fw_acl_q.h"
#include "timer.h"
#include "bzdma.h"
#include "mem.h"
#ifdef LE_MODE_EN
#include "le_ll.h"
#include "le_ll_driver.h"
#endif
#ifdef _SUPPORT_SECURE_CONNECTION_
#include "bz_auth_internal_2_1.h"
#endif
#ifdef _3DD_FUNCTION_SUPPORT_
#include "bt_3dd.h"
#endif
#include "gpio.h"
#include "bb_driver.h"
#ifdef PTA_EXTENSION
#include "pta_meter.h"
#endif
#include "bz_auth_internal.h"
#include "mint_os_buffer_internal.h"
#include "lmp_vendor_defines.h"
#include "bb_driver_4_1.h"

/* ===================== Variable Declaration Section ===================== */
#ifdef _DAPE_TEST_CHK_CSB_SYNC_TX
extern UCHAR g_afh_map[2][LMP_AFH_MAP_SIZE];
#endif

#ifdef _DAPE_ACL_PRI_HIGHER_THAN_INQ_WHEN_BR
UCHAR g_acl_priority_high_than_inq = 0;
#endif
#if defined (_TEST_PI_WRITE_PAGE_3_BY_VENDOR_CMD) || defined (_TEST_ADAPTIVITY_FUNC_2)
extern UINT8 g_enable_pi_read_after_tx;
#endif

#ifdef _ROM_CODE_PATCHED_
PF_ROM_CODE_PATCH_FUNC rcp_lc_log_data_for_channel_assessment = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_lc_sniff_att_start_intr_head_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_lc_sniff_att_start_intr_end_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_lc_sniff_att_end_intr_head_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_lc_sniff_att_end_intr_end_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_lc_sniff_cont_count_modify_and_supto_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_rx_interrupt_get_esco_ce_index_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_rx_interrupt_valid_ce_index_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_rx_interrupt_2nd_half_func = NULL;
#ifdef _DAPE_8723_RCP_
PF_ROM_CODE_PATCH_FUNC rcp_lc_send_packet_for_le_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_bb_tx_intr_no_pkt_scheduled_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_dual_mode_switch_in_tx_func = NULL;
#endif
PF_ROM_CODE_PATCH_FUNC rcp_BB_handle_psd_end_intr_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_new_afh_fill_backup_reg_and_send_signal_func = NULL;
#ifdef _SUPPORT_SECURE_CONNECTION_
PF_ROM_CODE_PATCH_FUNC rcp_rx_interrupt_secure_conn_func = NULL;
#endif
PF_ROM_CODE_PATCH_VOID rcp_bb_rx_interrupt = NULL;
#endif
#ifdef _DAPE_TEST_NEW_HW_BLOCK_ACL_FOR_LE
extern UINT8 g_enable_le_block_legacy;
UINT8 lc_le_pause_status = 0x00;
UINT8 lc_le_pause_status_for_sniff = 0x00;
UINT8 lc_le_pause_times = 0;
extern UINT8 g_block_legacy_for_le_slot_num;
#endif
#ifdef _CSB_RX_DBG_LOG
extern UINT8 g_csb_rx_dbg_log;
#endif

extern OS_POOL_MGR  pool_mgr;
extern UCHAR g_rf_test_mode_channel;
UINT8 lc_sco_pause_status = 0x00;
UINT8 lc_esco_pause_times = 0;
#ifdef _DAPE_EN_8821_MP_MODIFY_SLV_SNIFF_TIMEOUT
UINT8 lc_sniff_slv_send_pkt = 0x00;
#endif
#ifndef _ENABLE_COMPILE_NESTED_PAUSE_RESUME_
extern UCHAR lc_pause_schedule_pkts;
#endif
#ifdef _SECURE_CONN_REFRESH_KEY_WHEN_CONTINUOUS_MIC
extern UINT8 g_sc_refresh_key_when_continuous_mic_err;
#endif
#if defined(TEST_MODE)
UINT8 test_mode_sched_acl_pkt = FALSE;
UINT8 test_mode_rx_int_cnt = 0;
UINT8 test_mode_tx_int_cnt = 0;
UINT8 test_mode_tx_mode_force_ack = FALSE;
UINT8 test_mode_tx_mode_buf_in_used = FALSE;
LMP_PDU_PKT *test_mode_lmp_pdu = NULL;
UINT16 lc_is_tx_test_mode = FALSE;

#ifdef _RTL8723B_DUT_MODE_
UINT16 test_mode_tx_mode_reg_catch;
#endif

#endif

#ifdef COMPILE_SNIFF_MODE
UINT32 lc_sniff_cont_count[LC_MAX_NUM_OF_LUT_EX_TABLES] = {0};

#ifdef _CCH_RTL8723A_B_CUT
// _CCH_ECO_LPS_
UINT32 lc_sniff_window_rx_ind[LC_MAX_NUM_OF_LUT_EX_TABLES] = {0};
UINT32 lc_sniff_sup_count[LC_MAX_NUM_OF_LUT_EX_TABLES] = {0};
#endif

#endif /* COMPILE_SNIFF_MODE */
extern UINT32 bm_task_queue_full;
#ifdef _DAPE_TEST_ACL_RX_FIFO_DBG
extern UINT32 dape_acl_rx_rdptr;
extern UINT32 dape_acl_rx_wrptr;
#endif
#define BT0380_DBG_ALLOW_MAX_COUNT        10

LMP_SCO_CONNECTION_DATA *sco_check_entry = NULL;

#ifdef COMPILE_CHANNEL_ASSESSMENT
/**
 * Used by master to decide whether to process an access code interrupt or
 *  not. If last TX pkt is NULL, the interrupt is not processed.
 */
UCHAR lmp_last_tx_pkt_type = 0xff;

/* used by master to record the connection entry of previous tx pkt - austin */
UINT8 lmp_last_tx_conn_entry = 0;
#endif
extern void lmp_afh_la_handle_access_code_miss_interrupt();

#ifdef _YL_TEST_MODEM_RX_REPORT
UINT32 g_rx_report_last_native_clock = 0;
#endif

#ifdef _CCH_LPS_
extern OS_TIMER timer_pool[OS_MAX_TIMER_BUCKETS]; /* the pool of the timer */
extern UINT32 g_lps_timer_counter;
#endif
#ifdef _SECURE_CONN_REFRESH_KEY_BY_VENDOR_CMD
extern UINT8 g_refresh_key_en;
#endif
#ifdef _CSB_CHK_RX_PKT_BY_PCD_REG_IN_8821B_TEST_CHIP
UINT8 g_beacon_rx_ing = 0;
#endif

/* ================== Static Function Prototypes Section ================== */
SECTION_ISR_LOW void lc_log_data_for_channel_assessment(
                        BB_RX_PARAMS *rx_param, UINT8 is_esco);
#ifdef _SUPPORT_CSB_RECEIVER_
SECTION_ISR_LOW void BB_handle_hlc_interrupt_in_scatternet(UCHAR phy_piconet_id);
#else
SECTION_ISR_LOW void BB_handle_hlc_interrupt_in_scatternet(void);
#endif
#ifdef COMPILE_PARK_MODE
SECTION_ISR_LOW void BB_handle_park_imode_interrupts(UINT16 mode_sts_reg,
        UCHAR phy_piconet_id);

#endif

#ifdef COMPILE_ROLE_SWITCH
SECTION_ISR_LOW void lmp_handle_switch_newconn_timeout_in_scatternet(void);
#endif

#ifdef COMPILE_ESCO
SECTION_ISR void BB_handle_start_of_esco_interrupt(UINT16 read);
SECTION_ISR void BB_handle_end_of_esco_interrupt(void);
#endif

#ifdef COMPILE_PARK_MODE
SECTION_ISR_LOW void lc_park_setup_lc_level_conn_during_unpark_procedure();
INLINE void lc_park_clear_lc_level_conn_during_unpark_procedure(UCHAR lut_index);
#endif

#if defined(_NEW_BZDMA_FROM_V7_) && (defined(BZDMA_USE_NEW_SCO_TXCMD) || \
                                    defined(_USE_ONE_NEW_BZDMA_TXCMD_FOR_SCO_))
void lc_handle_sco_txcmd_ito_interrupt(UINT16 status);
#endif

/* ================== For rom code patch function point ================== */
#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8723_RCP_
PF_ROM_CODE_PATCH_FUNC rcp_BB_handle_tx_interrupt_lps_reset = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_BB_handle_rx_interrupt_lps_reset = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_BB_handle_end_of_sniff_attemp_interrupt = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_set_packet_type_in_dont_wait_ack_rx = NULL;
#endif
#ifdef _CCH_8821_RCP_
PF_ROM_CODE_PATCH_FUNC rcp_bb_scan_end_intr = NULL;
#endif
#endif

/* ===================== Static Function Definition Section =============== */
UINT8 generate_new_afh_signal = FALSE;
UINT8 afh_tmp_rx_channel = 0;
UINT8 afh_tmp_weight = 0;
UINT16 afh_tmp_read_rssi = 0;

LC_RTK_PKT_RECD_S new_afh_rx_info;

#ifdef _INQRES_DBG_CCH_
extern const UINT8 afh_is_tx[79];
#endif

#ifdef TEST_MODEM_PSD
extern ALIGN(16) SECTION_PATCH_SBSS MODEM_PSD_REPORT_ELEMENT_S_TYPE patch_g_modem_psd_report_array[79];
#endif

void new_afh_fill_backup_reg_and_send_signal(void)
{
    OS_SIGNAL sig_send;

#ifdef _ROM_CODE_PATCHED_
    if (rcp_new_afh_fill_backup_reg_and_send_signal_func != NULL)
    {
        if (rcp_new_afh_fill_backup_reg_and_send_signal_func(NULL))
        {
            return;
        }
    }
#endif

#ifdef _SSP_DHKEY_CALCULATE_NEW_FLOW_
	if ((new_afh_rx_info.access_code_miss == FALSE) && (new_afh_rx_info.hec_err == FALSE))
	{
	    if (lmp_connection_entity[new_afh_rx_info.ce_index].dhkey_calculating)
	    {
	        /* because the link is calculating DHKey, we can not service
	           hci_ch_as_task temporarily for our task scheduler's capability.
	           Thereforce, we do not send signals here */
	        return;
	    }
	}
#endif


#ifdef _NEW_MODEM_PI_ACCESS_BY_VENDOR_REG_
#if defined(_CCH_CFOE_RETRY_) || defined(_INQRES_DBG_CCH_) || defined(_YL_TEST_MODEM_RX_REPORT)
    new_afh_rx_info.backup_status_rpt0 = RTK_READ_MODEM_REG_PI(0, TRANS_MODEM_REG(0x68));
    new_afh_rx_info.backup_status_rpt1 = RTK_READ_MODEM_REG_PI(0, TRANS_MODEM_REG(0x6A));
#else
    new_afh_rx_info.backup_status_rpt0 = 0;
    new_afh_rx_info.backup_status_rpt1 = 0;
#endif
    new_afh_rx_info.backup_status_rpt2 = RTK_READ_MODEM_REG_PI(0, TRANS_MODEM_REG(0x6C));
    new_afh_rx_info.backup_status_rpt3 = RTK_READ_MODEM_REG_PI(0, TRANS_MODEM_REG(0x6E));
    new_afh_rx_info.backup_status_rpt4 = RTK_READ_MODEM_REG_PI(0, TRANS_MODEM_REG(0x70));
#else
    new_afh_rx_info.backup_status_rpt0 = lc_read_rf_rx_backup_status_rpt_reg(0);
    new_afh_rx_info.backup_status_rpt1 = lc_read_rf_rx_backup_status_rpt_reg(1);
    new_afh_rx_info.backup_status_rpt2 = lc_read_rf_rx_backup_status_rpt_reg(2);
    new_afh_rx_info.backup_status_rpt3 = lc_read_rf_rx_backup_status_rpt_reg(3);
    new_afh_rx_info.backup_status_rpt4 = lc_read_rf_rx_backup_status_rpt_reg(4);
#endif
    memcpy(&sig_send, &new_afh_rx_info, sizeof(new_afh_rx_info));
    OS_ISR_SEND_SIGNAL_TO_TASK (hci_ch_as_task_handle, sig_send);
}

#ifdef COMPILE_SNIFF_MODE
/**
 * Calculates ssr_to value.
 *
 * \param ce_index CE Index.
 *
 * \return SSR Timeout.
 */
UINT16 lc_calculate_ssr_to(UINT16 ce_index)
{
    UINT16 rem_min_timeout;
    UINT16 ssr_to;
    UINT16 lcl_sniff_interval;
    UINT16 temp;

    lcl_sniff_interval = lmp_connection_entity[ce_index].sniff_interval;
    ssr_to = lmp_connection_entity[ce_index].ssr_data.min_local_timeout;
    rem_min_timeout = lmp_connection_entity[ce_index].ssr_data.rem_min_timeout;

    if (ssr_to < rem_min_timeout)
    {
        ssr_to = rem_min_timeout;
    }

    temp = lcl_sniff_interval * LMP_MIN_SSR_TSNIFF_MUL;
    if (ssr_to < temp)
    {
        ssr_to = temp;
    }

    return ssr_to;
}

/**
 * Enters subrating mode. Checks for the instant also.
 * Called from sniff-start-of-attempt interrupt.
 *
 * \param ce_index CE Index.
 *
 * \return None.
 */
void lc_check_enter_ssr(UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY *ce_ptr;
    UINT32 cur_clock;
    LMP_SSR_STATUS lcl_lmp_ssr_state;
    LC_SSR_STATUS lcl_lc_ssr_state;
    UINT32 lcl_ssr_instant;
    UCHAR lcl_enter_exit_ssr;
    UCHAR lcl_neg_in_progress;
    UINT32 lcl_prev_pkt_time;

    ce_ptr = &lmp_connection_entity[ce_index];
    lcl_lmp_ssr_state = (LMP_SSR_STATUS) ce_ptr->ssr_data.lmp_ssr_state;
    lcl_lc_ssr_state = (LC_SSR_STATUS) ce_ptr->ssr_data.lc_ssr_state;
    lcl_ssr_instant = ce_ptr->ssr_data.ssr_instant;
    lcl_enter_exit_ssr = ce_ptr->ssr_data.enter_exit_ssr;
    lcl_neg_in_progress = ce_ptr->ssr_data.neg_in_progress;
    lcl_prev_pkt_time = ce_ptr->ssr_data.prev_pkt_time;

    /* Check states */
    if (((lcl_lmp_ssr_state != LMP_SSR_ACTIVE) &&
         (lcl_lmp_ssr_state != LMP_SSR_TRANSITION)) ||
         (lcl_ssr_instant == FORCE_FLUSH_CLOCK) ||
         (lcl_neg_in_progress != FALSE))
    {
        return;
    }

    if ((lcl_lc_ssr_state == LC_SSR_IDLE) && (lcl_enter_exit_ssr == 0))
    {

        lc_get_clock_in_scatternet(&cur_clock, ce_ptr->phy_piconet_id);

        cur_clock >>= 1;

        /* Check if this is SSR instant */
        if (lcl_lmp_ssr_state == LMP_SSR_TRANSITION)
        {
            if (lcl_lc_ssr_state == LC_SSR_IDLE)
            {
                if (lcl_ssr_instant <= ((cur_clock+2) & BT_CLOCK_27_BITS))
                {
                    LC_LOG_INFO(LOG_LEVEL_HIGH,ENT_SSR_FLAGED_1,1,cur_clock);
                    lcl_enter_exit_ssr = 3;
                }
            }
        }
        else if ((lcl_prev_pkt_time != FORCE_FLUSH_CLOCK) &&
                 (lcl_lc_ssr_state  == LC_SSR_IDLE))
        {
            /* Check for SSR timeout */
            UINT32 ssr_to;
            INT32 lc_get_clock_diff = 0x0;

            ssr_to = lc_calculate_ssr_to(ce_index);

            lc_get_clock_diff = cur_clock - lcl_prev_pkt_time;
            if (lc_get_clock_diff < 0)
            {
                lc_get_clock_diff += (BT_CLOCK_27_BITS + 1);
            }
            if (lc_get_clock_diff >= ssr_to)
            {
                LC_LOG_INFO(LOG_LEVEL_HIGH,CUR_TO,2,cur_clock,ssr_to);
                lcl_enter_exit_ssr = 1;
            }
        }
    }

    ce_ptr->ssr_data.enter_exit_ssr = lcl_enter_exit_ssr;

    return;
}

/**
 * Exits subrating mode. Should be called when a crc packet
 * is received.
 *
 * \param ce_index CE Index.
 *
 * \return None.
 */
void lc_check_exit_ssr(UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY *ce_ptr = &lmp_connection_entity[ce_index];
    UINT32 cur_clock;
    SSR_DATA *ssr_data_ptr;

    if (ce_index == INVALID_CE_INDEX)
    {
        return;
    }

    ssr_data_ptr = &ce_ptr->ssr_data;

    if (ssr_data_ptr->lc_ssr_state == LC_SSR_SUBRATE)
    {
        LC_LOG_INFO(LOG_LEVEL_HIGH,EXT_SSR_FLGD,0,0);
        ssr_data_ptr->enter_exit_ssr = 2;
    }

    if (ssr_data_ptr->lmp_ssr_state == LMP_SSR_ACTIVE)
    {
        /* Reset timeout counter */
        lc_get_clock_in_scatternet(&cur_clock, ce_ptr->phy_piconet_id);

        cur_clock >>= 1;
        ssr_data_ptr->prev_pkt_time = cur_clock;
    }
    return;
}

/**
 * Used the check if tsniff needs to be changed (ssr/sniff)
 * and signals enter or exit ssr.
 *
 * \param ce_index CE Index.
 *
 * \return None.
 */
void signal_ssr_enter_exit(UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY *ce_ptr = &lmp_connection_entity[ce_index];
    OS_SIGNAL sig_send;
    UCHAR state;

    if (ce_index == INVALID_CE_INDEX)
    {
        return;
    }
    state = ce_ptr->ssr_data.enter_exit_ssr;
    ce_ptr->ssr_data.enter_exit_ssr = 0;

    switch(state)
    {
        default:
        case 0:     /* Do Nothing */
            return;

        case 1:     /* Signal Enter */
            LC_LOG_INFO(LOG_LEVEL_HIGH,SSR_ENTER_SIGNALLED,0,0);
            sig_send.type = LC_ENTER_SSR;
            if (ce_ptr->ssr_data.lc_ssr_state == LC_SSR_SUBRATE)
            {
                return;
            }
            break;

        case 2:     /* Signal Exit */
            LC_LOG_INFO(LOG_LEVEL_HIGH,SSR_EXIT_SIGNALLED,0,0);
            sig_send.type = LC_EXIT_SSR;
            if (ce_ptr->ssr_data.lc_ssr_state != LC_SSR_SUBRATE)
            {
                return;
            }
            break;
        case 3:     /* LMP Transition to ACTIVE */
            LC_LOG_INFO(LOG_LEVEL_HIGH, LMP_SSR_ACTIVE_SIGNALLED,0,0);
            sig_send.type = LC_LMP_ENTER_SSR;
            break;
    }

    sig_send.param = (void *)((UINT32)ce_index);
    if (OS_ISR_SEND_SIGNAL_TO_TASK(lc_rx_task_handle, sig_send) != BT_ERROR_OK )
    {
        ;
    }

    return;
}
#endif /* COMPILE_SNIFF_MODE */

/* ===================== Function Definition Section ====================== */

/**
 * Reads RSSI value from the baseband.
 *
 * \param None.
 *
 * \return rssi The RSSI value read from the baseband register.
 */
UINT16 lc_read_rssi_register(void)
{
    UINT16 rssi;

    rssi = BB_read_baseband_register(RADIO_SELECT_REGISTER);
    rssi &= 0x3F; /* only bit[5:0] are valid */
#ifdef _YL_MODEM_RSSI_MAPPING
//    rssi = lc_modem_rssi_mapping(rssi);
#endif
    return (rssi);
}

SECTION_ISR_LOW void BB_handle_slot_counter_interrupt_new(void)
{

#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8821_RCP_
    UCHAR return_status = 0;
    if (rcp_bb_scan_end_intr != NULL)
    {
        if ( rcp_bb_scan_end_intr((void*)(&return_status)))
        {
            return;
        }
    }
#endif
#endif

#ifndef _REDUCE_LPS_AFTER_RTL8703B_
    if( g_efuse_lps_setting_4.lps_use_intr )
    {
#ifdef _LPS_WITH_MULTI_LINK_
        UINT16 reg_int;

        reg_int = BB_read_baseband_register(PAGE_SCAN_INTERVAL_REGISTER);


        UINT16 ce_index;
        ce_index = lmp_role_switch_data.ce_index;
        if ((lmp_connection_entity[ce_index].ce_status == LMP_ROLE_SWITCHING) ||
            ( lmp_connection_entity[ce_index].ce_status == LMP_CONNECTION_ROLE_SWITCH))
        {
            bb_handle_slot_counter_intr();
            return;
        }

        if( (g_efuse_lps_setting_2.timer2_lps_on) &&
            (( reg_int&BIT14)||( reg_int&BIT15 )))
        {
           bb_handle_scan_end_intr(reg_int);
        }
#endif
    }
    else
#endif
    {

#ifdef _LPS_FOR_8821_
#ifdef _MODI_LPS_AFTER_RTL8703B_
        UINT8 enter_lps_flag = 0;

        if( g_lps_timer_counter == 0)
        {
            enter_lps_flag = lc_check_lps_for_idle(1);
        }

        if(ll_manager.adv_unit.enable || ll_manager.scan_unit.enable ||
           ll_manager.initiator_unit.enable || ll_manager.conn_unit.enable)
        {
            enter_lps_flag = 0;
        }
#endif

        UINT16 reg_int;

        reg_int = BB_read_baseband_register(PAGE_SCAN_INTERVAL_REGISTER);

        if( (g_efuse_lps_setting_2.timer2_lps_on) &&
            (( reg_int&BIT14)||( reg_int&BIT15 )))
        {
            if( reg_int&BIT14)
            {
               //RT_BT_LOG(BLUE, CCH_DBG_033, 1, sleep_mode_param.scan_end_flag);
                sleep_mode_param.scan_end_flag |= BIT1;

#ifdef _MODI_LPS_STATE_WITH_INTR_
                if(g_efuse_rsvd_2.lps_state_with_intr)
               	{
                    lc_post_lps_stste_signal(LPS_TIMER2_WITH_SCAN);
                }
                else
#endif
                {
#ifdef _MODI_LPS_AFTER_RTL8703B_
                    if(enter_lps_flag && (lmp_self_device_data.scan_enable == 3) )
                    {
                        lps_kill_inq_scan();
                        lps_period_state_enter_lps(LPS_TIMER2_WITH_SCAN);
                    }
#endif
                }
            }
            if( reg_int&BIT15)
            {
                //RT_BT_LOG(BLUE, YL_DBG_HEX_2, 2, sleep_mode_param.scan_end_flag, BB_read_native_clock());

                sleep_mode_param.scan_end_flag |= BIT0;
#ifdef _MODI_LPS_STATE_WITH_INTR_
                if(g_efuse_rsvd_2.lps_state_with_intr)
               	{
                    lc_post_lps_stste_signal(LPS_TIMER2_WITH_SCAN);
                }
                else
#endif
                {
#ifdef _MODI_LPS_AFTER_RTL8703B_
                    if(enter_lps_flag && (lmp_self_device_data.scan_enable == 2) )
                    {
                        lps_kill_page_scan();
                        lps_period_state_enter_lps(LPS_TIMER2_WITH_SCAN);
                    }
#endif
                }
           }
        }
#endif


#ifdef _DAPE_TEST_NO_ROLE_SW_AFTER_DETACH
        UINT16 ce_index;
        ce_index = lmp_role_switch_data.ce_index;

        /* dape 20150818: these two logics can be merged together without 
           making XBAR error if we check ce_index first. (comment by Austin)*/
        if ( (ce_index >= LMP_MAX_CE_DATABASE_ENTRIES) || 
            (lmp_connection_entity[ce_index].entity_status == UNASSIGNED))
        {
            /* Clear the interrupt. */
            BB_read_baseband_register(PAGE_SCAN_INTERVAL_REGISTER);

            /* Clear enable bit. */
            AND_val_with_bb_reg_macro(PAGE_SCAN_WINDOW_REGISTER, (~BIT13));		
            return;
        }
#endif
        bb_handle_slot_counter_intr();


    }
}

#ifdef _ROM_CODE_PATCHED_
UINT8 (*rcp_baseband_interrupt_handler)(UINT16 *int_stat_reg) = NULL;
#endif

SECTION_ISR void baseband_interrupt_handler(void)
{
    UINT16 int_stat_reg;
    UINT16 read = 0;
    UINT16 lcl_lc_current_interrupt_mask;
    UINT16 int_stat_valid_reg;

#ifdef ESCO_DISC_DEBUG
    OS_SIGNAL end_esco_window_signal;
#endif /* ESCO_DISC_DEBUG */

    int_stat_reg =  BB_read_baseband_register(INTERRUPT_REGISTER);

    lcl_lc_current_interrupt_mask = lc_current_interrupt_mask;

#if defined(TEST_MODE)
    if (lc_tci_pause_flag == TRUE)
    {
        test_mode_tx_int_cnt = 0;
        test_mode_rx_int_cnt = 0;
        test_mode_tx_mode_force_ack = FALSE;
        test_mode_tx_mode_buf_in_used = FALSE;
#ifdef _DUT_DELAYED_LOOPBACK_MODE_
        dut_mode_manager.work_space = 0;
#endif
    }
#endif

    for ( ; int_stat_reg & (~lcl_lc_current_interrupt_mask);
            int_stat_reg = BB_read_baseband_register(INTERRUPT_REGISTER))
    {
#ifdef POWER_SAVE_FEATURE
        if ((sleep_mode_param.bb_sm_sts != BB_NORMAL_MODE) &&
            !(int_stat_reg & 0x4000))
        {
            /* Ignore values read after sleep mode has been enabled. */
            return;
        }
#endif

#ifdef _ENABLE_BTON_POWER_SAVING_
        //if (int_stat_reg & 0x4000)
        //{
        //    RT_BT_LOG(RED,LMP_2_1_393,0,0);
        //}
#endif

#ifdef _ROM_CODE_PATCHED_
        if (rcp_baseband_interrupt_handler != NULL)
        {
            if (rcp_baseband_interrupt_handler(&int_stat_reg))
            {
                continue;
            }
        }
#endif
        int_stat_valid_reg =  int_stat_reg & (~lcl_lc_current_interrupt_mask);

#ifdef _CCH_LPS_
        if (BB_EXIT_POWER_CTRL_INT(int_stat_reg))
        {
#ifdef _YL_LPS
            LPS_DBG_LOG(YELLOW, LPS_LOG_013, 1, int_stat_reg);
#endif
            /* Handle sleep mode interrupt */
            LC_HANDLE_SM_INTR();
        }
#endif

#ifdef LE_MODE_EN
        if (IS_BT40 && BB_LE_MODE_INT(int_stat_valid_reg))
        {
            ll_interrupt_handler();
            if (fw_bt_rx_int_fifo.found_legacy)
            {
                /* clear finished rx interrupt flag */
                int_stat_valid_reg &= ~0x0001;
                fw_bt_rx_int_fifo.found_legacy = FALSE;
            }
        }
#endif

        if (BB_ITO_INT(int_stat_valid_reg))
        {
            BB_handle_ito_interrupt();
        }

        if (BB_TX_INT(int_stat_valid_reg))
        {
            BB_handle_tx_interrupt();
        }

        if (BB_RX_INT(int_stat_valid_reg))
        {
#ifdef _ROM_CODE_PATCHED_
            if (rcp_bb_rx_interrupt != NULL)
            {
                rcp_bb_rx_interrupt();
            }
            else
#endif
            {
#ifdef _DAPE_CORRECT_INTR_HANDLER_BASED_ON_HW_BEHAVIOR
                ll_pop_all_rx_int_stack();
                ll_pre_cleanup_tx_schedule_of_event_end();
                ll_cleanup_rx_status();
#else
                BB_handle_full_rx_interrupt(NULL);
#endif
            }
        }

        if (BB_HLC_INT(int_stat_valid_reg))
        {
#ifdef _SUPPORT_CSB_RECEIVER_
            UINT16 mode_status_reg;
            UCHAR phy_piconet_id;
            mode_status_reg = BB_read_baseband_register(MODE_STATUS_REGISTER);
            phy_piconet_id = ((mode_status_reg >> 15) << 1) |
                             ((mode_status_reg >> 13) & 0x01);

            BB_handle_hlc_interrupt_in_scatternet(phy_piconet_id);
#else
            BB_handle_hlc_interrupt_in_scatternet();
#endif
        }

        if (BB_MODE_STAT_INT(int_stat_valid_reg))
        {
            BB_handle_imode_interrupt();
        }

        if (BB_ACCESS_CODE_MISS_INT(int_stat_valid_reg))
        {
            BB_handle_access_code_miss_interrupt();

            if (generate_new_afh_signal)
            {
                new_afh_fill_backup_reg_and_send_signal();
            }
        }

#ifdef _BRUCE_DEBUG_PORT
        UINT16 read2;
        read2=BB_read_baseband_register(0x25c); // SCO 0x25c[2]=0; g_plc_pkt_miss
        read2 &=(UINT16)~(BIT2);
        BB_write_baseband_register(0x25c,read2);
#endif

        /* Read the SNIFF_ESCO_START_STATUS_REGISTER here , because if Sniff
        interrupt and esco instant occur simultaneously then the register
        will be cleared after the first read.
        */
#if defined(COMPILE_SNIFF_MODE) || defined(COMPILE_ESCO)
        if ((BB_SNIFF_INT(int_stat_valid_reg)) ||
            (BB_ESCO_START_OF_WINDOW_INT(int_stat_valid_reg)))
        {
            read = BB_read_baseband_register(SNIFF_ESCO_START_STATUS_REGISTER);
        }
#endif

#ifdef COMPILE_SNIFF_MODE
        if (BB_SNIFF_INT(int_stat_valid_reg))
        {
            BB_handle_sniff_interrupt(read);
        }
#endif /* COMPILE_SNIFF_MODE */

#ifdef COMPILE_ESCO
        if (BB_ESCO_START_OF_WINDOW_INT(int_stat_valid_reg))
        {
            BB_handle_start_of_esco_interrupt(read);

#ifdef _IMPROVE_PCM_MUTE_CTRL_TIMING_
            //if (otp_str_data.hci_excodec_state)
            if(hci_excodec_state)
            {
                lc_pcm_mute_control();
            }
#endif
        }

        if (BB_ESCO_END_OF_WINDOW_INT(int_stat_valid_reg))
        {
            BB_handle_end_of_esco_interrupt();
        }
#endif

        /* Handle the slot count expied interrupt */
        if (BB_SLOT_COUNTER_INT(int_stat_valid_reg))
        {
            BB_handle_slot_counter_interrupt_new();
        }
    }

    return;
}

#ifdef COMPILE_ESCO
/**
 * Handles start of ESCO window interrupt from the baseband.
 *
 esco start and sniff status register
 * \param read: the read content of register
 *              (esco start and sniff status register, 0x5C)
 *
 * \return None.
 */
SECTION_ISR void BB_handle_start_of_esco_interrupt(UINT16 read)
{
    UCHAR count;
    UCHAR lt_addr;
    UINT16 ce_index;

    read = (UINT16) (read >> 7) & 0x7F;/* 0x5C[13:7]: the bitmap of eSCO[6:0] */

    for (count = 0; count < LMP_MAX_ESCO_CONN_ENTITIES; count++)
    {
        if (read & 0x0001)
        {
            lt_addr = lmp_esco_connection_entity[count].lt_addr;
            ce_index = lmp_esco_connection_entity[count].ce_index;

            /* Handling the start of esco window interrupt */
            lc_handle_esco_instant_interrupt_new(lt_addr, ce_index);
        }
        read >>= 1;
        if (read == 0)
        {
            break;
        }
    }

#ifdef _BRUCE_DEBUG_PORT
    read=BB_read_baseband_register(0x25c); // 0x25c[3]=0; g_plc_HW_pkt_miss
    read &= (UINT16)(~BIT3);
    BB_write_baseband_register(0x25c,read);
#endif

}

/**
 * Handles end of ESCO window interrupt from the baseband.
 *
 * \param None.
 *
 * \return None.
 */
SECTION_ISR void BB_handle_end_of_esco_interrupt(void)
{
    UCHAR count;
    UCHAR lt_addr;
    UINT16 ce_index;
    UCHAR logical_piconet_id;
    UINT16 read;

    read = BB_read_baseband_register(ESCO_END_STATUS_REGISTER) & 0x7F;

    for (count = 0; count < LMP_MAX_ESCO_CONN_ENTITIES; count++)
    {
        if (read & 0x0001)
        {
            lt_addr = lmp_esco_connection_entity[count].lt_addr;
            ce_index = lmp_esco_connection_entity[count].ce_index;
            logical_piconet_id = lmp_connection_entity[ce_index].phy_piconet_id;

            /* Handling end of esco window interrupt */
            lc_handle_esco_window_expiry_interrupt_new(lt_addr,logical_piconet_id);
        }
        read >>= 1;
        if (read == 0)
        {
            break;
        }
    }
}
#endif /* COMPILE_ESCO */

#ifdef COMPILE_SNIFF_MODE
SECTION_ISR void BB_handle_end_of_sniff_attemp_interrupt(UINT8 lut_index)
{
    UINT8 ce_index;
    LMP_CONNECTION_ENTITY *ce_ptr;
    UINT8 lcl_ce_status;
    UINT32 lcl_sniff_interval;

    #ifdef _YL_LPS
    lps_gpio_one_pull_low(LPS_GPIO_SNIFF);
    LPS_DBG_LOG(GRAY, LPS_LOG_019, 1, lut_index);
    #endif

    ce_index = lut_ex_table[lut_index].index_in_CE;

#ifdef _ROM_CODE_PATCHED_
    /* Reserved com code patch by yilinli:
      * e.g. read and record old tolerance value (for modifing wakeup instant)
      */
    if (rcp_lc_sniff_att_end_intr_head_func != NULL)
    {
        if ( rcp_lc_sniff_att_end_intr_head_func((void*)(&lut_index)) )
        {
            return;
        }
    }
#endif

    ce_ptr = &lmp_connection_entity[ce_index];

#ifdef _CSB_RX_DBG_LOG
    if (g_csb_rx_dbg_log)
{

UINT8 pid = ce_ptr->phy_piconet_id;
UINT32 cur_clk;
lc_get_clock_in_scatternet(&cur_clk, pid);
RT_BT_LOG(GRAY, MSG_CSB_SNIFF_END, 11,
cur_clk,
BB_read_native_clock(),
pid,
BB_read_baseband_register(TPOLL_HOLD_SNIFF_INTERVAL_REGISTER),
BB_read_baseband_register(SNIFF_SLOT_OFFSET_INTERVAL_REGISTER),
BB_read_baseband_register(X_VALUE_FOR_TOLERANCE_REGISTER),
BB_read_baseband_register(reg_PICONET_INFO[pid]),
BB_read_baseband_register(reg_SCA_SLAVE_UPPER_LUT[pid]),
BB_read_baseband_register(reg_SCA_SLAVE_LOWER_LUT[pid]),
(bt_3dd_var.stp_rx_param.clock_offset >> 16),
((bt_3dd_var.stp_rx_param.clock_offset) & ((UINT16)(0xFFFF))));
}
#endif

    if ((ce_index == INVALID_CE_INDEX)||( lut_index == 0) || (ce_ptr->sniff_interval == 0))
    {
        return;
    }


#ifdef _YL_LPS
    ce_ptr->sniff_xtol_prev = ce_ptr->sniff_xtol;
#endif


    lcl_ce_status = ce_ptr->ce_status;
    lcl_sniff_interval = ce_ptr->sniff_interval;




#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8723_RCP_
    if (rcp_BB_handle_end_of_sniff_attemp_interrupt != NULL)
    {
        if ( rcp_BB_handle_end_of_sniff_attemp_interrupt((void*)(&lcl_ce_status), ce_index) )
        {
            return;
        }
    }
#endif
#endif


#ifndef _CCH_RTL8723A_B_CUT

    if ( !( (lcl_ce_status == LMP_SNIFF_MODE) ||
            (lcl_ce_status == LMP_SNIFF_MODE_NEG) ||
            (lcl_ce_status == LMP_ADDING_SCO_LINK_DURING_SNIFF) ) )
    {
        /* Drop the sniff interrupt. */
        RT_BT_LOG(GRAY, LC_ISR_945, 2, ce_index, ce_ptr->ce_status);
        return;
    }

#else
// _CCH_8723_A_ECO_
    // cch
    // 20110728 (fw bug)
    // for IOT ISSC eSCO + Sniff/Unsniff ce_status error

    if ( !( (lcl_ce_status == LMP_SNIFF_MODE) ||
            (lcl_ce_status == LMP_SNIFF_MODE_NEG) ||
            (lcl_ce_status == LMP_ADDING_SCO_LINK_DURING_SNIFF) ||
            (lcl_ce_status == LMP_ADDING_ESCO_LINK) ||
            (lcl_ce_status == LMP_CHANGING_ESCO_PARAMS) ||
            (lcl_ce_status == LMP_ESCO_DISCONNECTING) ||
            (lcl_ce_status == LMP_ACL_ESCO_DISCONNECTING) ) )
    {
#ifdef _SUPPORT_CSB_RECEIVER_
        if (!(bt_3dd_var.csb_rx_param.enable && (ce_index == bt_3dd_var.csb_rx_param.ce_index)))
#endif
        {
        /* Drop the sniff interrupt. */
        RT_BT_LOG(GRAY, LC_ISR_945, 2, ce_index, ce_ptr->ce_status);
        return;
        }
    }


#endif


#ifdef _YL_LPS
    UINT16 min_sniff_int_for_xtol = (10 << g_efuse_lps_setting_1.min_sniff_int_for_xtol);
    if (lcl_sniff_interval > min_sniff_int_for_xtol)
#else
    if (lcl_sniff_interval > LC_MIN_SNIFF_INT_FOR_XTOL)
#endif
    {
#ifdef _YL_RTL8723A_B_CUT
        // TODO: removed or not?
#endif
        ce_ptr->sniff_xtol = lc_program_tolerance(lcl_sniff_interval, ce_index);
    }

#ifdef _DAPE_TEST_CHG_SNIFF_ATTEMPT
//RT_BT_LOG(BLUE, DAPE_TEST_LOG293, 1, ce_ptr->sniff_xtol);
    BB_modify_xtol_in_scatternet(6, ce_ptr->phy_piconet_id);
    UINT16 dape_reg_timeout = BB_read_baseband_register(SNIFF_TIMEOUT_REGISTER);
    UINT16 dape_reg_attempt = BB_read_baseband_register(SNIFF_ATTEMPT_REGISTER);
    //RT_BT_LOG(BLUE, DAPE_TEST_LOG525, 6, lut_index, 6, dape_reg_timeout, dape_reg_attempt, 0,0);
#endif

    ce_ptr->cont_poll_count = 0;

    /* Try initiating lmp_SSR_Req */
    if (ce_ptr->ssr_data.lmp_ssr_state == LMP_SSR_IDLE)
    {
        OS_SIGNAL signal;

        signal.type = LMP_INITIATE_SSR_REQ;
        signal.param = (OS_ADDRESS)((UINT32)ce_index);
        OS_ISR_SEND_SIGNAL_TO_TASK(lmp_task_handle, signal);
    }

    /* Re-start the Sup-to. */
    if (lmp_sup_timeout_var[ce_index] == 0)
    {
        /* At least one packet is received in the sniff slots. */
        if ((lmp_sup_var_timeout_var[ce_index] == 0xFF) &&
                (ce_ptr->link_supervision_timeout != 0) )
        {
            lmp_send_os_start_timer_signal(
                ce_ptr->supervision_timeout_handle,
                (UINT16)(SLOT_VAL_TO_TIMER_VAL(
                             LMP_SUPERVISION_TIMER_RESOLUTION)));

            lmp_sup_var_timeout_var[ce_index] = 0;
        }

        lc_sniff_cont_count[lut_index - 1] = 0;
    }
    else
    {
        /* No packet is received in the sniff slots. If this is the
        only connection, then increase xtoll value programmed
        to the baseband, */

#ifndef _MODI_LPS_AFTER_RTL8703B_
        UINT32 temp;
#endif
        lc_sniff_cont_count[lut_index - 1]++;

#ifdef _ROM_CODE_PATCHED_
        /* reserved rom code patch by yilinli, for
          * e.g.
          *    1. modify lc_sniff_cont_count[lut_index - 1] to make sniff xtol more reasonalbe
          *    2. modify lc_sniff_cont_count[lut_index - 1] to make sniff tick supervision timeout reasonable
          *    3. process the supervision timeout by sniff tick
          */
        if (rcp_lc_sniff_cont_count_modify_and_supto_func != NULL)
        {
            if(rcp_lc_sniff_cont_count_modify_and_supto_func((void*)&lut_index, ce_index))
            {
                return;
            }
        }
#endif

#ifdef _YL_LPS // for supervision timeout
/*      TO BE REPLACED BY PATCH
        if ((ce_ptr->link_supervision_timeout) && ( g_efuse_lps_setting_2.sniff_lps_on) )
        {
            if(g_efuse_lps_setting_3.sniff_lps_sup_to)
            {
                temp = (ce_ptr->link_supervision_timeout)/(ce_ptr->sniff_interval);
                if(lc_sniff_cont_count[lut_index - 1] > temp)
                {
                    RT_BT_LOG( GRAY, LPS_LOG_031, 3, ce_ptr->link_supervision_timeout, ce_ptr->sniff_interval, lc_sniff_cont_count[lut_index - 1] );

                    OS_SIGNAL signal_sup_to;
                    signal_sup_to.type = LMP_SUPERVISION_TIMEOUT_SIGNAL;
                    signal_sup_to.param = (OS_ADDRESS)((UINT32)ce_index);
                    if(OS_SEND_SIGNAL_TO_TASK(lmp_task_handle, signal_sup_to)
                            != BT_ERROR_OK)
                    {
                        LMP_ERR(LOG_LEVEL_HIGH, OS_SEND_SIGNAL_TO_TASK_FAILED,0,0);
                    }
                }

            }
        }
*/
#endif

#ifndef _MODI_LPS_AFTER_RTL8703B_
#ifdef _YL_RTL8723A_B_CUT
      // TODO: lc_sniff_cont_count has some BUGs
#endif
        temp = lc_sniff_cont_count[lut_index - 1];

#ifdef _YL_LPS
#ifdef _YL_RTL8723A_B_CUT
      // TODO: (X*(temp+1) > Y) OR  (X*temp > Y)
      if ( (lcl_sniff_interval * temp) > min_sniff_int_for_xtol)
#else
      if ( (lcl_sniff_interval) > min_sniff_int_for_xtol)
#endif
#else
        if ( (lcl_sniff_interval * temp) > LC_MIN_SNIFF_INT_FOR_XTOL)
#endif

        {
            ce_ptr->sniff_xtol = lc_program_tolerance((UINT16)(lcl_sniff_interval *
                                          (temp + 1)), ce_index);
            #if 0 // force xtol = 2; for DSM Slave Test
                BB_modify_xtol_in_scatternet(2, ce_ptr->phy_piconet_id);
            #endif
        }
#endif

    }
    signal_ssr_enter_exit(ce_index);

    if (lmp_self_device_data.next_next_sniff_instant != 0xFFFFFFFF)
    {
        lmp_self_device_data.next_sniff_instant =
            lmp_self_device_data.next_next_sniff_instant;
    }

    if (ce_ptr->next_next_instant_in_nat_clk != 0xFFFFFFFF)
    {
        ce_ptr->next_instant_in_nat_clk = ce_ptr->next_next_instant_in_nat_clk;

        lc_update_next_sniff_instant();
    }

#ifdef _LPS_FOR_8821_
    sleep_mode_param.lps_table_ing_bitmap &= (UINT32)(~ (1<<ce_index));
//    RT_BT_LOG(GRAY, LPS_LOG_044, 2,ce_index, sleep_mode_param.lps_table_ing_bitmap);
#endif

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
    // Move LC_PROGRAM_SNIFF_SM_MODE to the end
    if (ce_ptr->low_power_disconnect_state == UNSNIFF_DISCONNECT)
#else
    if (ce_ptr->low_power_disconnect_state != UNSNIFF_DISCONNECT)
    {
        // TODO: shall take tolerance/role/recv_valid_pkt into account!
        /* sub-optimal solution:
          * Slave role: wakeup_instant -= (tol(k)-tol(k-1)) or not required to adjust
          * Master role: wakeup_instatnt += 0 ????
          */
#ifdef _CCH_LPS_
        if( g_efuse_lps_setting_2.sniff_lps_on )
        {
#endif
            LC_PROGRAM_SNIFF_SM_MODE(ce_index);
#ifdef _CCH_LPS_
        }
#endif

    }
    else
#endif
    {
        lmp_pend_acl_disconnect_from_isr(ce_index, ce_ptr->disconnect_reason);
    }

    /* Policy to improve FTP RX while in sniff (HID). */
    {
        UCHAR num_luts = 0;
        UINT16 temp_ce_index =0;

        for (num_luts = 0; num_luts < LC_MAX_NUM_OF_LUT_EX_TABLES; num_luts++)
        {
            temp_ce_index = lut_ex_table[num_luts].index_in_CE;
            if (( temp_ce_index != INVALID_CE_INDEX) &&
               ( lmp_connection_entity[temp_ce_index].remote_dev_role == SLAVE))
            {
                if (lc_cont_crc_rx_cnt[num_luts] != 0)
                {
                    /* Write default packet in new am_addr */
                    lc_check_and_update_pkt_in_lut(
                        LC_INVALID_PACKET_TYPE,
                        LC_MASTER_DEFAULT_PACKET_TYPE,
                        lut_ex_table[num_luts].lower_lut_address);
                }
            }
        }

        /* trigger optimize data transfer during sniff */
        OS_SIGNAL sig_send;
        sig_send.type = LC_HANDLE_OPTIMIZE_DATA_TX_DURING_SNIFF;
        sig_send.param = (void*)((UINT32)ce_index);

        OS_SEND_SIGNAL_TO_TASK(lc_tx_task_handle, sig_send);
    }

    lc_send_packet_in_scatternet( (ce_ptr->phy_piconet_id) ^ 0x1);

#ifndef IS_BTSOC
#ifdef _CCH_LPS_ // for lmp response timer
    if (ce_ptr->pdu_response_timer_running == TRUE)
    {
        OS_TIMER    *timer;

        timer = &timer_pool[ce_ptr->lmp_response_timer_handle];

        timer->sniff_tick_timer = timer->sniff_tick_timer + ce_ptr->sniff_interval;

        if(  (timer->sniff_tick_timer) >= LMP_RESPONSE_TIMEOUT_SLOT)
        {
            if(g_efuse_lps_setting_3.sniff_lps_lmp_to)
            {
                lmp_response_timeout_handler(ce_ptr->lmp_response_timer_handle,  (void *)((UINT32) ce_index) );
            }

            timer->sniff_tick_timer = 0;
        }
    }
#endif

#ifdef LPS_AFH_INSTANT_TIMER_SNIFF_TIC
    if (lc_lps_double_check_dsm_cond(LPS_SNIFF_DSM) == API_SUCCESS)
    {
        if (OS_IS_TIMER_RUNNING(ce_ptr->afh_instant_timer_handle))
        {
            OS_TIMER    *timer;
            timer = &timer_pool[ce_ptr->afh_instant_timer_handle];
            if (timer->timeout_function == lmp_handle_afh_instant_timer)
            {
                timer->sniff_tick_timer = timer->sniff_tick_timer + ce_ptr->sniff_interval;
                UINT32 timer_value_slots = ((UINT32) timer->timer_value)*16;
                if (timer->sniff_tick_timer >= timer_value_slots)
                {
                      unsigned short dummy;
                      OS_STOP_TIMER(ce_ptr->afh_instant_timer_handle, &dummy);
                      lmp_handle_afh_instant_timer(ce_ptr->afh_instant_timer_handle,  (void *)((UINT32) ce_index) );
                }
            }
        }
    }
#endif
#endif

#ifdef _SUPPORT_CSB_RECEIVER_
#ifdef _CSB_RX_SKIP_ENABLE
    UINT8 csb_rx_skip = bt_3dd_var.csb_rx_param.skip;
    if(( lc_sniff_window_rx_ind[lut_index - 1] == 0) ||
       (bt_3dd_var.csb_rx_param.crc_err))
    {
        /* no pkt is received during this sniff anchor point.
           Or the beacon received is crc err. we disable skip. */
        csb_rx_skip = 0;
    }
#ifdef _CSB_CHK_RX_PKT_BY_PCD_REG_IN_8821B_TEST_CHIP
    if((bt_3dd_var.csb_rx_param.enable) &&
        (ce_index == bt_3dd_var.csb_rx_param.ce_index))
    {
        g_beacon_rx_ing = BB_read_baseband_register(PCD_HIDDEN_STATUS_REGISTER) & (UINT16)(0x000F);
        if (g_beacon_rx_ing)
        {
            RT_BT_LOG(RED, DAPE_TEST_LOG440, 1, g_beacon_rx_ing);
        }
    }
#endif

    /* Re-program skip register and do not enable sniff start interrupt. */
    BB_write_baseband_register(BB_CSB_RX_SKIP_REG,
    csb_rx_skip | (ce_ptr->phy_piconet_id << 8) |
    (3 << 10));
#endif
#endif
#ifdef LPS_NEW_SNIFF_XTOL_UPDATE
    UINT32 inactive_interval;

    inactive_interval = ce_ptr->sniff_interval*(lc_sniff_sup_count[lut_index - 1]+1);
#ifdef _SUPPORT_CSB_RECEIVER_
    if (ce_index == bt_3dd_var.csb_rx_param.ce_index)
    {
            inactive_interval = ce_ptr->sniff_interval*
                  (bt_3dd_var.csb_rx_param.skip +
                  lc_sniff_sup_count[lut_index - 1]+1);
    }

#endif
    inactive_interval = MIN(inactive_interval,65535);
    min_sniff_int_for_xtol = (10 << g_efuse_lps_setting_1.min_sniff_int_for_xtol);
    if ( inactive_interval > min_sniff_int_for_xtol)
    {
        ce_ptr->sniff_xtol = lc_program_tolerance((UINT16)inactive_interval, ce_index);

        if (IS_USE_FOR_BQB && (ce_ptr->sniff_xtol == 4))
        {
            /* modify xtol setting in BQB mode */
            ce_ptr->sniff_xtol = 6;
            BB_modify_xtol_in_scatternet(6, ce_ptr->phy_piconet_id);
        }
    }
    else
    {
        ce_ptr->sniff_xtol = 0;
        BB_modify_xtol_in_scatternet(0, ce_ptr->phy_piconet_id);
    }
#endif
#ifdef _CCH_RTL8723A_B_CUT
// _CCH_ECO_LPS_
    UINT32 temp;
    LPS_DBG_LOG(RED, XTOL_PROGRAMMED, 1,ce_ptr->sniff_xtol);

    if( lc_sniff_window_rx_ind[lut_index - 1] )
    {
        lc_sniff_sup_count[lut_index - 1] = 0;
    }else
    {
        lc_sniff_sup_count[lut_index - 1] ++;
#ifdef _CCH_PATCH_DUMMY_LOG_
        RT_BT_LOG( BLUE, LPS_LOG_031, 3, ce_ptr->link_supervision_timeout, ce_ptr->sniff_interval, lc_sniff_sup_count[lut_index - 1] );
#endif
    }

    if ((ce_ptr->link_supervision_timeout) && ( g_efuse_lps_setting_2.sniff_lps_on) )
    {
        if(g_efuse_lps_setting_3.sniff_lps_sup_to)
        {
            temp = (ce_ptr->link_supervision_timeout)/(ce_ptr->sniff_interval);
            if(lc_sniff_sup_count[lut_index - 1] > temp)
            {
                RT_BT_LOG( GRAY, LPS_LOG_031, 3, ce_ptr->link_supervision_timeout, ce_ptr->sniff_interval, lc_sniff_sup_count[lut_index - 1] );

                OS_SIGNAL signal_sup_to;
                signal_sup_to.type = LMP_SUPERVISION_TIMEOUT_SIGNAL;
                signal_sup_to.param = (OS_ADDRESS)((UINT32)ce_index);
                if(OS_SEND_SIGNAL_TO_TASK(lmp_task_handle, signal_sup_to)
                        != BT_ERROR_OK)
                {
                    LMP_ERR(LOG_LEVEL_HIGH, OS_SEND_SIGNAL_TO_TASK_FAILED,0,0);
                }
            }

        }
    }

#endif
#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
    if (ce_ptr->low_power_disconnect_state != UNSNIFF_DISCONNECT)
    {
        // TODO: shall take tolerance/role/recv_valid_pkt into account!
        /* sub-optimal solution:
          * Slave role: wakeup_instant -= (tol(k)-tol(k-1)) or not required to adjust
          * Master role: wakeup_instatnt += 0 ????
          */

        if( g_efuse_lps_setting_2.sniff_lps_on )
        {

            LC_PROGRAM_SNIFF_SM_MODE(ce_index);
        }
    }
#endif

#ifdef _ROM_CODE_PATCHED_
    /* Reserved com code patch by yilinli:
      * e.g.
      */
    if (rcp_lc_sniff_att_end_intr_end_func != NULL)
    {
        rcp_lc_sniff_att_end_intr_end_func((void*)(&lut_index));
    }
#endif
}

SECTION_ISR void BB_handle_start_of_sniff_attemp_interrupt(UINT8 lut_index)
{
    UINT16 lower_lut_addr;
    UINT8 ce_index;
    LMP_CONNECTION_ENTITY *ce_ptr;
#ifdef _SUPPORT_CSB_RECEIVER_
#ifdef _CSB_RX_DBG_LOG
    if (g_csb_rx_dbg_log)
    {
        UINT8 i;
        UINT8 cam_offset = CAM_ADDR_OFFSET_LAYER2 + lut_index*CAM_ADDR_OFFSET_LAYER2_SIZE + CAM_ADDR_OFFSET_AFH;
        UINT32 afh_map_cam[3];
        for (i = 0; i < 3; i ++)
        {
            afh_map_cam[i] = BB_read_sc_cam( (cam_offset + i));
        }
        RT_BT_LOG(WHITE, MSG_CSB_SNIFF_START, 5, lut_index,
        cam_offset, afh_map_cam[0], afh_map_cam[1], afh_map_cam[2]);
#if 0
        {
        UINT8 pid = 0;
        RT_BT_LOG(WHITE, YL_DBG_HEX_20, 20,
        BB_read_native_clock(),
        pid,
        BB_read_baseband_register(0xC4),
        BB_read_baseband_register(reg_PICONET_BD_ADDR1[pid]),
        BB_read_baseband_register(reg_PICONET_BD_ADDR2[pid]),
        BB_read_baseband_register(reg_PICONET_BD_ADDR3[pid]),
        BB_read_baseband_register(0x16),
        BB_read_baseband_register(reg_PICONET_PARITY_BITS1[pid]),
        BB_read_baseband_register(reg_PICONET_PARITY_BITS2[pid]),
        BB_read_baseband_register(TPOLL_HOLD_SNIFF_INTERVAL_REGISTER),
        BB_read_baseband_register(SNIFF_SLOT_OFFSET_INTERVAL_REGISTER),
        BB_read_baseband_register(X_VALUE_FOR_TOLERANCE_REGISTER),
        BB_read_baseband_register(reg_PICONET_INFO[pid]),
        ((bt_3dd_var.stp_rx_param.clock_offset) & ((UINT16)(0xFFFF))),
        (bt_3dd_var.stp_rx_param.clock_offset >> 16),
        BB_read_baseband_register(reg_SCA_SLAVE_UPPER_LUT[pid]),
        BB_read_baseband_register(reg_SCA_SLAVE_LOWER_LUT[pid]),
        BB_read_baseband_register(AFH_CHANNEL_MAP_EN_REG),
        bt_3dd_var.stp_rx_param.clock_offset,
        BB_read_native_clock - (bt_3dd_var.stp_rx_param.clock_offset)
        );
        }
#endif
}
#endif
#endif
#ifdef _YL_LPS
    lps_gpio_one_pull_high(LPS_GPIO_SNIFF);
    LPS_DBG_LOG(RED, LPS_LOG_018, 1, lut_index);
#endif

    ce_index = lut_ex_table[lut_index].index_in_CE;

    if (ce_index == INVALID_CE_INDEX)
    {
        return;
    }

#ifdef _ROM_CODE_PATCHED_
    /* Reserved com code patch by yilinli:
      * e.g.
      *    1. start LPO calibratioin
      *    (2. xtol_prev[ce_index] = xtol[ce_index])
      */
    if (rcp_lc_sniff_att_start_intr_head_func != NULL)
    {
        if ( rcp_lc_sniff_att_start_intr_head_func((void*)(&lut_index)) )
        {
            return;
        }
    }
#endif

    ce_ptr = &lmp_connection_entity[ce_index];
    lower_lut_addr = lut_ex_table[lut_index].lower_lut_address;
#ifdef _SUPPORT_CSB_RECEIVER_
    if (bt_3dd_var.csb_rx_param.enable && (ce_index == bt_3dd_var.csb_rx_param.ce_index))
    {
        lc_get_clock_in_scatternet(&bt_3dd_var.csb_rx_param.clk_rx, ce_ptr->phy_piconet_id);
#ifdef _CSB_CAL_CLK_OFST_WHEN_RX_BEACON_IN_8821B_TEST_CHIP
        UINT32 native_clk = BB_read_native_clock();
        bt_3dd_var.csb_rx_param.clk_offset =
            (native_clk - bt_3dd_var.csb_rx_param.clk_rx) & (UINT32)(0x0FFFFFFF);
#endif
        bt_3dd_var.csb_rx_param.clk_rx += (ce_ptr->sniff_xtol + 1);
    }
#endif
#ifdef _CCH_RTL8723A_B_CUT
// _CCH_ECO_LPS_
    lc_sniff_window_rx_ind[lut_index - 1] = 0;
#endif


#if defined(POWER_SAVE_FEATURE)
    UINT16 pkt_type;

    if (ce_ptr->remote_dev_role == MASTER)
    {
        lc_check_and_update_pkt_in_lut(LC_INVALID_PACKET_TYPE,
                                       LC_SLAVE_DEFAULT_PACKET_TYPE,
                                       lower_lut_addr);
    }
    pkt_type = BB_read_baseband_register((UINT16) lower_lut_addr);

    if ((pkt_type == LC_INVALID_PACKET_TYPE) ||
        (pkt_type == LC_MASTER_DEFAULT_PACKET_TYPE) ||
        (pkt_type == LC_SLAVE_DEFAULT_PACKET_TYPE))
    {
        lc_check_enter_ssr( (UINT16) ce_index);
    }
#ifdef _DAPE_TEST_FIX_HW_SCO_NAK_SNIFF_BUG
    if (EN_FW_FIX_SCO_NAK_HID)
    {
        if (ce_ptr->should_ack)
        {
            UINT16 upper_lut_addr;
            upper_lut_addr = lut_ex_table[lut_index].upper_lut_address;
            UINT16 upper_lut = BB_read_baseband_register(upper_lut_addr);
            if ((upper_lut & BIT2) == 0)
            {
                BB_write_baseband_register(upper_lut_addr, (upper_lut|0x0004));
            }
            ce_ptr->should_ack = 0;
        }
    }
#endif
    UINT32 curr_clock;
    curr_clock = BB_read_native_clock();
    curr_clock = curr_clock + (ce_ptr->sniff_interval << 1);
    ce_ptr->next_next_instant_in_nat_clk = curr_clock;
    lmp_self_device_data.next_next_sniff_instant = curr_clock;
    lc_update_next_sniff_instant();

    /* Send packet required here, for master, attempt = 1. */
    lc_send_packet_in_scatternet(ce_ptr->phy_piconet_id);

#ifdef _LPS_BY_NATIVE_CLK_
    ce_ptr->sniff_last_instant = BB_read_native_clock();
    sleep_mode_param.lps_table_ing_bitmap |= (UINT32) (1<<ce_index);
#else
    lc_get_clock_in_scatternet(&(ce_ptr->sniff_last_instant),
                               ce_ptr->phy_piconet_id);
#endif
    ce_ptr->sniff_attempt_for_afh_count = 0;


#endif /* defined(POWER_SAVE_FEATURE) */

#ifdef _ROM_CODE_PATCHED_
    /* Reserved com code patch by yilinli:
      * e.g.
      */
    if (rcp_lc_sniff_att_start_intr_end_func != NULL)
    {
        rcp_lc_sniff_att_start_intr_end_func((void*)(&lut_index));
    }
#endif
}

/**
 * Handles sniff mode interrupt from the baseband.
 *
 * \param sniff_interrupt_status The status of sniff interrupt register.
 *
 * \return None.
 */
SECTION_ISR void BB_handle_sniff_interrupt(UINT16 sniff_interrupt_status)
{
    UCHAR lut_index;

#ifdef _ROBUST_READ_SNIFF_INDICATION_
    /* the following hande is more robust because the end and start of sniffer
       attempt indications in LUT register are read clear so it is a dangerous
       design. (fw can often modify that register via write after read)
       - austin */

    UINT16 bm_sniff_start;
    UINT16 bm_sniff_end;

    bm_sniff_start = BB_read_baseband_register(BB_SNIFF_START_REG) & 0x0FFE;
    bm_sniff_end = BB_read_baseband_register(BB_SNIFF_END_REG) & 0x0FFE;

    /* handle possible the end of sniff attemp interrupt */
    lut_index = 1;
    while (bm_sniff_end != 0)
    {
        if (bm_sniff_end & (1 << lut_index))
        {
            BB_handle_end_of_sniff_attemp_interrupt(lut_index);
            bm_sniff_end &= ~(1 << lut_index);
        }
        lut_index++;
    }

    /* handle possible the beginning of sniff attemp interrupt */
    lut_index = 1;
    while (bm_sniff_start != 0)
    {
        if (bm_sniff_start & (1 << lut_index))
        {
            BB_handle_start_of_sniff_attemp_interrupt(lut_index);
            bm_sniff_start &= ~(1 << lut_index);
        }
        lut_index++;
    }

#else

    UINT16 upper_lut_content;
    UINT32 count = 0;
    UINT16 temp_var;
    UINT16 lcl_sniff_interrupt_status;

    /* compose the bitmap of the entry in sniff */
    temp_var = (UINT16) (sniff_interrupt_status >> 7);
    temp_var &= 0x0180;
    lcl_sniff_interrupt_status = (sniff_interrupt_status & 0x7f) | (temp_var);

    while (lcl_sniff_interrupt_status != 0)
    {
        if ((1 << count) & lcl_sniff_interrupt_status)
        {
            lut_index = (UCHAR) (count + 1);

            /* read clear */
            upper_lut_content = BB_read_baseband_register
                                (lut_ex_table[lut_index].upper_lut_address);

            if (upper_lut_content & SNIFF_END_OF_ATTEMPT)
            {
                BB_handle_end_of_sniff_attemp_interrupt(lut_index);
            }

            if (upper_lut_content & SNIFF_START_OF_ATTEMPT)
            {
                BB_handle_start_of_sniff_attemp_interrupt(lut_index);
            }

            lcl_sniff_interrupt_status &= ~(1 << count);
        }
        count++;
    }
#endif

    return;
}
#endif /* COMPILE_SNIFF_MODE */

/**
 * Handles access code miss interrupt from the baseband.
 *
 * \param None.
 *
 * \return None.
 */
SECTION_ISR_LOW void BB_handle_access_code_miss_interrupt(void)
{
    UINT8 is_sco_miss = FALSE;
    UCHAR sco_num = 0xff;

    UINT16 ac_int_status;
    ac_int_status = BB_read_baseband_register(0xFC);

    generate_new_afh_signal = FALSE;

#ifdef SCO_OVER_HCI
    if (ac_int_status & BIT6) /* Is it generated for SCO receive slot? */
    {
        UINT16 fifo_config;

        /* To get the SCO number */
        fifo_config = BB_read_baseband_register(SYNC_FIFO_CONFIG_REGISTER);
        {
            if (fifo_config & BIT6)
            {
                sco_num = 0;
            }
            else if (fifo_config & BIT7)
            {
                sco_num = 1;
            }
            else if (fifo_config & BIT8)
            {
                sco_num = 2;
            }
            if (sco_num != 0xFF)
            {
                lc_handle_sco_erroneous_data(sco_num, ERR_LOST_DATA);
                is_sco_miss = TRUE;
            }
        }
    }
#endif /* SCO_OVER_HCI */

#ifdef _BRUCE_TEST_PLC_PKT_STATUS
    if (is_sco_miss) /* Is it generated for SCO receive slot? */
    {
        plc_pkt_status_var.g_plc_pkt_miss++;   //SCO

#ifdef _BRUCE_DEBUG_PORT
        UINT16 read;
        read=BB_read_baseband_register(0x25c); //SCO 0x25c[2]=1; g_plc_pkt_miss
        read |=(UINT16)BIT2;
        BB_write_baseband_register(0x25c,read);
#endif

    }
#endif



#ifdef COMPILE_CHANNEL_ASSESSMENT
    if (lmp_last_tx_pkt_type == BB_NULL)
    {
        /* Don't process interrupt. Reset pkt type. */
        lmp_last_tx_pkt_type = 0xff;

        /* since we sent NULL pkt in previous tx slot, we shall not
           expect we can receive any pkt - austin */
        return;
    }

    if (lmp_last_tx_pkt_type != 0xff)
    {
        if (otp_str_data.rtk_afh_mechanism_enable)
        {
            lmp_afh_la_handle_access_code_miss_interrupt();

            new_afh_rx_info.is_from_wifi = FALSE;

#ifndef _AFH_ACL_IGNORE_PKT_MISS_
            new_afh_rx_info.is_sco_rx = (sco_num < 3) ? TRUE : FALSE;
            if (is_sco_miss)
            {
                new_afh_rx_info.ce_index =
                    lmp_sco_connection_data[sco_num].conn_entity_index;
            }
            else
            {
                new_afh_rx_info.ce_index = lmp_last_tx_conn_entry;
            }
#else
            new_afh_rx_info.is_sco_rx = FALSE;
            new_afh_rx_info.ce_index = lmp_last_tx_conn_entry;
#endif
        }
    }
    lmp_last_tx_pkt_type = 0xff;
#endif /* COMPILE_CHANNEL_ASSESSMENT */
    return;
}

#ifdef COMPILE_PARK_MODE
/**
 * Park Connection Interrupt. This interrupt is generated
 * when the device has just come out of park.
 *
 * \param None.
 *
 * \return None.
 */
SECTION_ISR_LOW void lc_park_conn_intr(void)
{
    UINT16 address;
    UINT16 ce_index = 0;
    LMP_CONNECTION_ENTITY *ce_ptr;
    OS_SIGNAL signal_send;

    if (lmp_unpark_ce_index != INVALID_CE_INDEX)
    {
        ce_index = lmp_unpark_ce_index;
        ce_ptr = &lmp_connection_entity[ce_index];
    }
    else
    {
        /* Might not be needed */
        /* Looping all ce_indexs and finding the one with PARK_MODE
           can be a solution */

        for (ce_index = 0; ce_index < LMP_MAX_CE_DATABASE_ENTRIES; ce_index++)
        {
            ce_ptr = &lmp_connection_entity[ce_index];

            if ((ce_ptr->entity_status == ASSIGNED) &&
                (ce_ptr->ce_status == LMP_PARK_MODE))
            {
                break;
            }
        }

        if (ce_index == LMP_MAX_CE_DATABASE_ENTRIES)
        {
            ce_ptr = &lmp_connection_entity[0];
        }
    }

#ifdef ENABLE_LOGGER_LEVEL_2
    LC_LOG_INFO(LOG_LEVEL_LOW, PARK_CONNECTION_INTR,0,0);
#endif

    if (ce_ptr->remote_dev_role == MASTER)
    {
        lc_update_scatternet_state_addition(SCA_SLAVE, ce_ptr->phy_piconet_id);
    }
    else
    {
        lc_update_scatternet_state_addition(SCA_MASTER, ce_ptr->phy_piconet_id);
    }

    lmp_unpark_ce_index = INVALID_CE_INDEX ;
    lc_num_unpark_req_trials = 0;
    ce_ptr->unpark_req_flag = LMP_UNPARK_IDLE;

    if (lmp_self_device_data.number_of_parked_dev != 0)
    {
        lmp_self_device_data.number_of_parked_dev--;
    }

    /* Check number of parked devices, if there are no devices in
     * park mode then kill the beacon
     */
    if (lmp_self_device_data.number_of_parked_dev == 0 )
    {
        lc_kill_beacon(ce_index);

        /* Write slave default packet type to Broadcast LUT */
        address = MASTER_BROADCAST_LOWER_LUT_ADDR;
        BB_write_baseband_register(address, LC_SLAVE_DEFAULT_PACKET_TYPE);
    }
    else
    {
        bb_flush_broadcast_fifo();
    }

    lmp_self_device_data.number_of_hlc++;

    BB_modify_xtol_in_scatternet(0x0, ce_ptr->phy_piconet_id);

    {
        UINT16 addr = PICONET1_INFO_REGISTER;
        UINT16 val;

        if (ce_ptr->phy_piconet_id <= SCA_PICONET_MAX)
        {
            addr = reg_PICONET_INFO[ce_ptr->phy_piconet_id];
        }

        if (ce_ptr->remote_dev_role == SLAVE)
        {
            val = 0x01;
        }
        else
        {
            val = (ce_ptr->am_addr << 1);
        }

        BB_write_baseband_register(addr, val);

        RT_BT_LOG(GRAY, LC_ISR_1408, 3, addr, val, ce_ptr->phy_piconet_id);
    }

    if (ce_ptr->remote_dev_role == SLAVE)
    {
        lc_handle_unpark(ce_index);
    }
    else
    {
        /* Send LMP_Accepted PDU. */
        lmp_send_lmp_accepted(ce_index, lmp_self_device_data.unpark_op_code,
                              (UCHAR)MASTER_TID, LMP_CONNECTED);

        /* Reset xtol */
        BB_modify_xtol_in_scatternet(0x0, ce_ptr->phy_piconet_id);

        /* Send Mode change event */
        hci_generate_mode_change_event(HCI_COMMAND_SUCCEEDED,
                                       ce_index,
                                       LP_ACTIVE_MODE,
                                       0x00);

        if (ce_ptr->low_power_disconnect_state == UNPARK_DISCONNECT)
        {
            ce_ptr->low_power_disconnect_state = IDLE_STATE;
            /* Kill ACL connection */
            if (lmp_pend_acl_disconnect_from_isr(ce_index, ce_ptr->disconnect_reason)
                    == API_FAILURE)
            {
#ifdef ENABLE_LOGGER_LEVEL_2
                LC_LOG_ERROR(LOG_LEVEL_LOW,LMP_HANDLE_ACL_DISCONNECT_FAILED,0,0);
#endif
            }
        }
        else
        {
            signal_send.type = LMP_LLC_CON_SETUP_COMPLETE;
            signal_send.param = (OS_ADDRESS)((UINT32)ce_index);

            if (OS_ISR_SEND_SIGNAL_TO_TASK(lmp_task_handle, signal_send)
                    != BT_ERROR_OK)
            {
#ifdef ENABLE_LOGGER_LEVEL_2
                LC_LOG_INFO(LOG_LEVEL_LOW, OS_SEND_SIGNAL_TO_TASK_FAILED,0,0);
#endif
            }
        }

        /* Release pm_addr. */
        lmp_put_pm_addr(ce_ptr->pm_addr, ce_ptr->phy_piconet_id);

        slave_ce_index = INVALID_CE_INDEX;
    }

    BB_start_tpoll(ce_ptr->am_addr, ce_ptr->Tpoll, ce_ptr->phy_piconet_id);

    lc_slave_init_unpark_pending = FALSE;

    if (ce_ptr->remote_dev_role == MASTER)
    {
        ce_ptr->hci_unpark_req_flag = HCI_UNPARK_IDLE;
    }

    lc_check_and_enable_scans_in_scatternet();

    lmp_self_device_data.lc_no_of_connections[ce_ptr->phy_piconet_id]++;

    ce_ptr->auto_unpark_cnt = 0xFFFF;

    return;
}

/**
 * Sets up LC level connection during unpark procedure. When IUT
 * is master, the park-HLC interrupt may be generated on receiving
 * a CRC packet from the slave. In this case, as the  priority of
 * rx-interrupt is higher than the imde-interrupt(park-HLC), IUT
 * will drop the received CRC packet. This will lead to rx-FIFO
 * corruption. So, we temporarily need to fool LC that there is a
 * connection on this am_addr. This function is called on setting
 * the active bit in the baseband. If the unpark procedure fails,
 * then this will be cleared on park-NCTO.
 *
 * \param None.
 *
 * \return None.
 */
SECTION_ISR_LOW void lc_park_setup_lc_level_conn_during_unpark_procedure(void)
{
    UCHAR lut_index;
    UINT16 ce_index;
    LMP_CONNECTION_ENTITY *ce_ptr;
    UCHAR am_addr;
    LUT_EXTENSION_TABLE *lut_ptr;

    if (lmp_unpark_ce_index != INVALID_CE_INDEX)
    {
        ce_index = lmp_unpark_ce_index;
    }
    else
    {
        return;
    }

    ce_ptr = &lmp_connection_entity[ce_index];

    am_addr = ce_ptr->am_addr;

    if (ce_ptr->remote_dev_role == SLAVE)
    {
        lut_index = am_addr;
    }
    else
    {
        if (ce_ptr->phy_piconet_id <= SCA_PICONET_MAX)
        {
            lut_index = LC_SCA_SLAVE_1_LUT + ce_ptr->phy_piconet_id;
        }
        else
        {
            return;
        }
    }

    lut_ptr = &lut_ex_table[lut_index];
    lut_ptr->index_in_CE = ce_index;

    return;
}

/**
 * Clears the temporarily set up LC level connection for the unpark
 * connection. This function is called on park-NCTO.
 *
 * \param lut_index The index to lut_ex_table data structure.
 *
 * \return None.
 */
INLINE void lc_park_clear_lc_level_conn_during_unpark_procedure(UCHAR lut_index)
{
    lut_ex_table[lut_index].index_in_CE = INVALID_CE_INDEX;
}

#endif

/**
 * Handles sco indication in rx interrupt from the baseband.
 *
 * \param None.
 *
 * \return None.
 */
SECTION_ISR void BB_handle_sco_in_rx_interrupt(UINT8 miss, UINT32 lc_rx_pkt_type)
{
#if defined(SCO_OVER_HCI)
    UCHAR sco_num;
    UINT16 fifo_config;

    /* To get the SCO number */
    fifo_config = BB_read_baseband_register(SYNC_FIFO_CONFIG_REGISTER);

    if (fifo_config & BIT6)
    {
        sco_num = 0;
    }
    else if (fifo_config & BIT7)
    {
        sco_num = 1;
    }
    else if (fifo_config & BIT8)
    {
        sco_num = 2;
    }
    else
    {
        /* invalid sco number */
        return;
    }

    if (miss)
    {
        lc_handle_sco_erroneous_data(sco_num, ERR_LOST_DATA);
    }
    else
    {
        if (lc_rx_pkt_type & BB_RX_ISR_HV1_HV2_HV3_DV)
        {
            lc_handle_sco_rx_interrupt(sco_num);
        }
        else
        {
            /* No data received.*/
            lc_handle_sco_erroneous_data(sco_num, ERR_LOST_DATA);
        }
    }
#endif /* defined(SCO_OVER_HCI) */
}

SECTION_ISR_LOW void BB_handle_id_indication_in_rx_interrupt(void)
{
    /* Received ID after sending FHS packet For Role Switch */
    if (lmp_mss_state == LMP_MSS_S_TO_M_SENT_FHS_PKT)
    {
        UINT8 old_lut_index;
        UINT8 new_lut_index;
        UINT16 ce_index;
        UINT16 read;

        old_lut_index = lmp_role_switch_data.old_lut_index;
        new_lut_index = lmp_role_switch_data.new_lut_index;

        ce_index = lmp_role_switch_data.ce_index;

#ifdef COMPILE_AFH_HOP_KERNEL
        /* AFH is disabled after FHS-ID. */
        lmp_connection_entity[ce_index].afh_mode = AFH_DISABLE;

        BB_disable_afh(lmp_role_switch_data.old_am_addr,
                       lmp_connection_entity[ce_index].phy_piconet_id);

#endif /* COMPILE_AFH_HOP_KERNEL */

        /* Make the old am_address deactive */
        AND_val_with_bb_reg_macro(
                            lut_ex_table[old_lut_index].upper_lut_address,
                            (~ACTIVE_BIT));

        OR_val_with_bb_reg_macro(
                            lut_ex_table[new_lut_index].upper_lut_address,
                            ACTIVE_BIT);

        BB_write_baseband_register(
                            lut_ex_table[old_lut_index].lower_lut_address,
                            LC_INVALID_PACKET_TYPE);

        /* Write default packet in new am_addr */
        BB_write_baseband_register(
                            lut_ex_table[new_lut_index].lower_lut_address,
                            LC_MASTER_DEFAULT_PACKET_TYPE);

        read = (UINT16) ((lmp_role_switch_data.new_am_addr << 5) |
                         (lmp_role_switch_data.new_piconet_id << 11));

        BB_write_baseband_register(CONNECTOR_REGISTER, read);

        lmp_set_mss_state(LMP_MSS_S_TO_M_SENT_FHS_RECD_ID);
    }
    else /* put unpark indication flag check. */
    {
        OS_SIGNAL ar_sig_send;
        UCHAR ar_addr;

        /* Received ID packet , For slave initiated unparking */
        ar_addr = (UCHAR) BB_read_baseband_register(
                      ACCESS_REQUEST_ADDRESS_REGISTER);
        ar_sig_send.type = LMP_SLAVE_UNPARK_REQ_RECD_SIGNAL;
        ar_sig_send.param = (OS_ADDRESS)((UINT32)ar_addr);
        ar_sig_send.ext_param = (OS_ADDRESS)SCA_PICONET_FIRST;

        OS_ISR_SEND_SIGNAL_TO_TASK ( lmp_task_handle, ar_sig_send);
    }
}

#ifdef TEST_MODE
SECTION_ISR UINT8 BB_handle_test_mode_in_rx_interrupt(BB_RX_PARAMS *rx_param,
                                                    UINT8 rx_ack,
                                                    UINT16 *lut_low_fix_content)
{
    LUT_EXTENSION_TABLE *ex_lut = &lut_ex_table[rx_param->lut_index];
    UINT16 ce_index = rx_param->ce_index;
    UINT16 read;
    UINT16 address;
    UINT8 am_addr;
    UINT8 rx_seqn;
    UINT16 lut_contents = 0;
    UINT8 rx_pkt_type;
    UINT32 lcl_rx_pkt_type;
    UINT16 payload_hdr = rx_param->payload_header;
    BZ_REG_S_RX_PL_HDR *pl_hdr = (BZ_REG_S_RX_PL_HDR *)&payload_hdr;
    BZ_REG_S_RX_STATUS *rx_status = &rx_param->rx_status;

    rx_pkt_type = rx_status->pkt_type;
    lcl_rx_pkt_type = 1 << rx_pkt_type;
    am_addr = rx_status->lt_addr;
    rx_seqn = rx_status->seqn;

    if ((lmp_self_device_data.test_mode == HCI_DUT_LOOPBACK_MODE) ||
        (lmp_self_device_data.test_mode == HCI_DEVICE_UNDER_TEST_MODE))
    {
        /* If the tests enters loopback mode or tx mode */

#ifndef _RTL8723B_DUT_MODE_
#ifdef _DUT_DELAYED_LOOPBACK_MODE_
        if ((rx_pkt_type == BB_POLL) &&
            (lc_is_tx_test_mode == FALSE) &&
            (dut_mode_manager.test_data_begin == TRUE) &&
            (dut_mode_manager.rx_buf_id != dut_mode_manager.tx_buf_id))
        {
            lcl_rx_pkt_type &= ~BB_RX_ISR_POLL;
        }
#endif
#endif

        if ((lc_is_tx_test_mode == TRUE) &&
            (test_mode_tx_mode_force_ack == FALSE))
        {
            /* In tx mode, do not force ack */

            if (test_mode_lmp_pdu != NULL)
            {
                /* In Tx Mode */
                BB_write_baseband_TX_FIFO_scatternet(
                                (UINT8*)test_mode_lmp_pdu->payload_content,
                                test_mode_lmp_pdu->pdu_length,
                                am_addr, 0);

                read = BB_read_baseband_register(SCA_SLAVE1_LOWER_LUT + 2);
                read |= 0x0004;
                BB_write_baseband_register((SCA_SLAVE1_LOWER_LUT + 2), read);

                lut_contents = (BB_DM1 << 12) | (0x03 << 10) |
                                test_mode_lmp_pdu->pdu_length;

                BB_write_baseband_register(SCA_SLAVE1_LOWER_LUT, lut_contents);

                *lut_low_fix_content = lut_contents;
            }
        }

        if ((!(lcl_rx_pkt_type & (BB_RX_ISR_POLL_NULL_FHS)) &&
               (lc_is_tx_test_mode == FALSE)) ||
               ((lcl_rx_pkt_type & BB_RX_ISR_POLL) &&
                (test_mode_tx_mode_force_ack == TRUE) &&
                (lc_is_tx_test_mode == TRUE)))
        {
            /* In loopback mode, do not receive poll/null/fhs packets or */
            /* In tx mode, receive poll pkt and force ack */
            UCHAR is_sco_pkt = FALSE;
            UINT8 is_esco_aux_pkt = FALSE;
            TEST_CONTROL_PARAMS *tcp;
            UINT8 crc_error;
            UINT16 num_of_bytes;
            UINT16 lut_pkt_type = 0;

            tcp = &lmp_connection_entity[ce_index].test_mode_info.tc_params;
            if (lc_is_tx_test_mode == TRUE)
            {
                rx_pkt_type = tcp->packet_type_in_pdu;
            }

            /* Change this to lcl_rx_pkt_type. */
            if (
#ifdef COMPILE_ESCO
                ((tcp->pkt_desc != PACKET_TYPE_DESCRIPTION_BR_eSCO) &&
                (tcp->pkt_desc != PACKET_TYPE_DESCRIPTION_EDR_eSCO)) &&
#endif /* end of COMPILE_ESCO */
                (lcl_rx_pkt_type & BB_RX_ISR_SCO_VOICE_PKT))
            {
                is_sco_pkt = TRUE;
            }

#ifdef _CCH_SC_ECDH_P256_MIC_FLAG
            crc_error = (rx_status->crc_err) || (pl_hdr->mic_err) ;
#else
            crc_error = rx_status->crc_err;
#endif

            num_of_bytes = pl_hdr->len;

            /* allow retransmit */
            AND_val_with_bb_reg_isr(ENCRYPTION_ENABLE_REGISTER, (0xFFBF));

            if (lc_is_tx_test_mode == TRUE)
            {
                crc_error = 1;
                num_of_bytes = tcp->num_packets;
                payload_hdr = 0;
            }

            /* Check for pdu. */
            if (((pl_hdr->llid != L_CH_LMP) && (ex_lut->bb_flow != 0)) &&
                (lc_tci_pause_flag == FALSE))
            {
                /* disallow retransmit */
                OR_val_with_bb_reg_isr(ENCRYPTION_ENABLE_REGISTER, 0x0040); //added by wzl 20100507

#ifdef _DUT_DELAYED_LOOPBACK_MODE_
                dut_mode_manager.test_data_begin = TRUE;
#endif

                switch (rx_pkt_type)
                {
                case BB_DM1:
                case BB_DH1:
                case BB_DM3:
                case BB_DH3:
                case BB_DM5:
                case BB_DH5:
                    lut_pkt_type = rx_pkt_type << 12;
                    break;

                case BB_HV1:
                    num_of_bytes = 10;
                    lut_contents = 0x580a;
                    break;

                case BB_HV2:
#ifdef COMPILE_ESCO
                    if (tcp->pkt_desc == PACKET_TYPE_DESCRIPTION_EDR_eSCO)
                    {
                        /* 2-EV3 packet. */
                        num_of_bytes = tcp->num_packets;
                        lut_contents = (UINT16) (0x6800 | num_of_bytes);
                        lut_pkt_type = BB_HV2_LUT;
                        is_esco_aux_pkt =  TRUE;
                    }
                    else
#endif /* end of COMPILE_ESCO */
                    {
                        /* HV2 packet. */
                        num_of_bytes = 20;
                        lut_contents = 0x6814;
                    }
                    break;

                case BB_EV4: /* And 2-EV5 packet */
                    num_of_bytes = tcp->num_packets;
                    lut_contents = (UINT16) (0xc800 | num_of_bytes);
                    lut_pkt_type = BB_EV4_LUT;
                    BB_write_baseband_register(0x60, 0x0f00);
                    is_esco_aux_pkt =  TRUE;
                    break;

                case BB_EV5: /* And 3-EV5 packet */
                    num_of_bytes = tcp->num_packets;
                    lut_contents = (UINT16) (0xd800 | num_of_bytes);
                    lut_pkt_type = BB_EV5_LUT;
                    BB_write_baseband_register(0x60, 0x0f00);
                    is_esco_aux_pkt =  TRUE;
                    break;

                case BB_HV3:
#ifdef COMPILE_ESCO
                    if ((tcp->pkt_desc == PACKET_TYPE_DESCRIPTION_BR_eSCO) ||
                        (tcp->pkt_desc == PACKET_TYPE_DESCRIPTION_EDR_eSCO))
                    {
                        /* EV3 packet & 3-EV3*/
                        num_of_bytes = tcp->num_packets;
                        lut_contents = (UINT16) (0x7800 | num_of_bytes);
                        lut_pkt_type = BB_HV3_LUT;
                        is_esco_aux_pkt =  TRUE;
                    }
                    else
#endif /* end of COMPILE_ESCO */
                    {
                        /* HV3 packet */
                        num_of_bytes = 30;
                        lut_contents = 0x781e;
                    }
                    break;

                case BB_DV:
                    if (tcp->pkt_desc == PACKET_TYPE_DESCRIPTION_EDR_ACL)
                    {
                        /* 3-DH1 packet. */
                        /* Nothing more to be done here. */
                    }
                    else
                    {
                        /* DV packet. */
                        num_of_bytes += 10;
                    }

                    lut_pkt_type = BB_DV_LUT;
                    break;

                    /* add by austin */
                case BB_AUX1:
                    lut_pkt_type = BB_AUX1_LUT;
                    crc_error = 0; /* no CRC in AUX1 pkt */
                    is_esco_aux_pkt =  TRUE;
                    break;

                default:
                    break;
                } /* switch (rx_pkt_type) */

                if (is_sco_pkt == FALSE)
                {
                    if (pl_hdr->llid == L_CH_LMP)
                    {
                        pl_hdr->llid = L_CH_L2CAP_START;
                    }
                    lut_contents = (UINT16) ( lut_pkt_type |
                            (pl_hdr->llid << 10) | num_of_bytes);
                }

                BB_write_baseband_register(CONNECTOR_REGISTER, am_addr << 5);

                address = SCA_SLAVE1_LOWER_LUT;

                /* Write ARQN = 1 for Tx Packet in Tx mode */
                /* Write ARQN = 1 for eSCO/Aux1 Tx Packets in DUT mode */
                read = BB_read_baseband_register(address + 2);
                read |= 0x0004;
                if (tcp->pkt_desc & BIT1)
                {
                    read |= 0x8000; /* EDR */
                }
                else
                {
                    read &= 0x7FFF; /* basic rate */
                }
                BB_write_baseband_register((UINT16) (address + 2), read);

                /* start to handle tx mode or loopback mode */

#ifndef _RTL8723B_DUT_MODE_
                UINT8 rx_after_tx = FALSE;
                UINT8 *tbuf = NULL;
                UINT8 idx;

                if (num_of_bytes > 0)
                {
                    test_mode_sched_acl_pkt = TRUE;

#ifdef _DUT_DELAYED_LOOPBACK_MODE_
                    if (lc_is_tx_test_mode == FALSE)
                    {
                        /* This is in Lookback Mode */
                        rx_after_tx = TRUE;

                        if ((dut_mode_manager.rx_buf_id == dut_mode_manager.tx_buf_id) &&
                            (dut_mode_manager.rx_buf_id == 0))
                        {
                            dut_mode_manager.buf[0].len = num_of_bytes;
                            dut_mode_manager.buf[0].lut_type = lut_pkt_type;
                            dut_mode_manager.buf[0].lut_lower = lut_contents;
                            lut_pkt_type = BB_NULL_LUT;
                            lut_contents = 0; /* send NULL */
                        }
                        else
                        {
                            idx = dut_mode_manager.tx_buf_id;
                            tbuf = dut_mode_manager.buf[idx].ppkt->hci_acl_data_pkt;

                            BB_write_baseband_TX_FIFO_scatternet(tbuf,
                                            dut_mode_manager.buf[idx].len,
                                            am_addr, 0);

                            dut_mode_manager.tx_buf_id++;

                            idx = dut_mode_manager.rx_buf_id;
                            dut_mode_manager.buf[idx].lut_lower = lut_contents;
                            dut_mode_manager.buf[idx].lut_type = lut_pkt_type;
                            dut_mode_manager.buf[idx].len = num_of_bytes;
                        }
                    }
                    else
                    {
                        /* In Tx Mode */
                        idx = dut_mode_manager.tx_buf_id;
                        tbuf = dut_mode_manager.buf[idx].ppkt->hci_acl_data_pkt;

                        /* copy data from sram to tx fifo */
                        BB_write_baseband_TX_FIFO_scatternet(tbuf, num_of_bytes,
                                                             am_addr, 0);
                    }
#endif
                }
                else
                {
                    if (lc_is_tx_test_mode == FALSE)
                    {
                        test_mode_sched_acl_pkt = TRUE;
                    }

#ifdef _DUT_DELAYED_LOOPBACK_MODE_
                    if ((lc_is_tx_test_mode == FALSE) &&
                        (dut_mode_manager.rx_buf_id != dut_mode_manager.tx_buf_id))
                    {
                        idx = dut_mode_manager.tx_buf_id;
                        tbuf = dut_mode_manager.buf[idx].ppkt->hci_acl_data_pkt;

                        lut_pkt_type = dut_mode_manager.buf[idx].lut_type;
                        lut_contents = dut_mode_manager.buf[idx].lut_lower;

                        /* copy data from sram to tx fifo */
                        BB_write_baseband_TX_FIFO_scatternet(tbuf,
                                                dut_mode_manager.buf[idx].len,
                                                am_addr, 0);

                        test_mode_sched_acl_pkt = TRUE;
                        dut_mode_manager.test_data_begin = FALSE;
                        dut_mode_manager.tx_buf_id++;
                    }
#endif
                }
#else /* else of #ifndef _RTL8723B_DUT_MODE_ */
                if ((num_of_bytes > 0) || (lc_is_tx_test_mode == FALSE))
                {
                    test_mode_sched_acl_pkt = TRUE;
                }
#endif /* end of #ifndef _RTL8723B_DUT_MODE_ */

                /* here to write LUT register */
                if ((tcp->pkt_desc != PACKET_TYPE_DESCRIPTION_EDR_ACL) &&
                    (lut_pkt_type == BB_DV_LUT))
                {
                    BB_write_baseband_register(address, lut_contents - 10);
                }
                else
                {
                    BB_write_baseband_register(address, lut_contents);
                }

#ifndef _RTL8723B_DUT_MODE_
#ifdef _DUT_DELAYED_LOOPBACK_MODE_
                if (rx_after_tx)
                {
                    UINT8 idx;
                    idx = dut_mode_manager.rx_buf_id;
                    tbuf = dut_mode_manager.buf[idx].ppkt->hci_acl_data_pkt;

                    /* move data from rx fifo to sram */
                    BB_read_baseband_RX_FIFO(tbuf, num_of_bytes, TRUE);
                    dut_mode_manager.rx_buf_id++;
                }

#ifdef _MT8852B_DBG_
                if (test_mode_sched_acl_pkt &&
                    !lc_is_tx_test_mode &&
                    (test_mode_rx_int_cnt < BT0380_DBG_ALLOW_MAX_COUNT))
                {
                    if (rx_after_tx)
                    {
                        MT8852B_DBG(GREEN, MT8852B_MSG_RX_LB_TEST, 7,
                            test_mode_rx_int_cnt, rx_param->packet_header,
                            payload_hdr, num_of_bytes, tbuf[0],
                            dut_mode_manager.rx_buf_id,
                            dut_mode_manager.tx_buf_id);
                    }
                    else
                    {
                        MT8852B_DBG(GREEN, MT8852B_MSG_RX_LB_TEST, 7,
                                test_mode_rx_int_cnt, rx_param->packet_header,
                                payload_hdr,
                                num_of_bytes, 0, 0, 0);
                    }
                    test_mode_rx_int_cnt++;
                }
#endif
#endif /* end of #ifdef _DUT_DELAYED_LOOPBACK_MODE_ */
#endif /* end of #ifndef _RTL8723B_DUT_MODE_ */

                if (!(lcl_rx_pkt_type & BB_RX_ISR_NO_CRC_DATA_PKT) &&
                     (!crc_error))
                {
                    ex_lut->remote_seqn_bit = rx_seqn;
                }

#ifdef COMPILE_ESCO
                /* If scenario_is_esco-loopback; can be either BR or EDR,
                   and no crc error, update seqn. */

                if ((tcp->pkt_desc == PACKET_TYPE_DESCRIPTION_BR_eSCO) ||
                    (tcp->pkt_desc == PACKET_TYPE_DESCRIPTION_EDR_eSCO))
                {
                    if ((lcl_rx_pkt_type & BB_RX_ISR_ESCO_DATA_PKT) &&
                        (!crc_error))
                    {
                        ex_lut->remote_seqn_bit = rx_seqn;
                    }
                }
#endif /* end of COMPILE_ESCO */

                /* Restart supervision timer var. */
                lmp_sup_timeout_var[ce_index] = 0;
                return TRUE;
            }
            else if ((pl_hdr->llid != L_CH_LMP) || crc_error)
            {
                lc_flush_rx_fifo();

                lc_send_packet_in_scatternet(rx_param->phy_piconet_id);

                if (!(lcl_rx_pkt_type & BB_RX_ISR_VOICE_DATA_PKT) && !crc_error)
                {
                    ex_lut->remote_seqn_bit = rx_seqn;
                }

                /* Restart supervision timer var. */
                lmp_sup_timeout_var[ce_index] = 0;

                if (!(rx_ack == BB_ACK))
                {
                    return TRUE;
                }
            }
        }
    }

    /* add by austin for Tx mode */
    if ((lmp_self_device_data.test_mode == HCI_DEVICE_UNDER_TEST_MODE) &&
        (lc_is_tx_test_mode == TRUE))
    {
        if (test_mode_tx_mode_force_ack)
        {
            if (pl_hdr->llid == L_CH_LMP)
            {
                /* start LMP procedure */
#ifdef _MT8852B_DBG_
                test_mode_rx_int_cnt = 0;
                test_mode_tx_int_cnt = 0;
#endif
                test_mode_tx_mode_force_ack = FALSE;
            }
        }
    }

    return FALSE;
}
#endif /* TEST_MODE */

SECTION_ISR_LOW void BB_handle_wait_ack_in_rx_interrupt(BB_RX_PARAMS *rx_param)
{
    UINT8 phy_piconet_id = rx_param->phy_piconet_id;
    LC_PICONET_SCHEDULER *pico_schd = &lc_piconet_scheduler[phy_piconet_id];
    UINT8 cur_rptr = rx_param->cur_schd_rptr;
    LC_SCHEDULED_PKT *schd = rx_param->schd;
    UINT8 lut_index = rx_param->lut_index;
    LMP_CONNECTION_ENTITY *ce_ptr = &lmp_connection_entity[schd->ce_index];
    BZ_REG_S_RX_STATUS *rx_status = &rx_param->rx_status;
    UINT8 my_role = (ce_ptr->remote_dev_role == SLAVE) ? MASTER : SLAVE;
    UINT8 basic_logic;
    UINT16 lc_packet_tye;
    UINT8 is_ret_2_cur_esco = FALSE;

    basic_logic = ((rx_param->lc_baseband_flow == BB_GO) &&
              (lc_cont_crc_rx_cnt[lut_index] == 0) &&
              (lc_cont_crc_tx_cnt[lut_index] == 0) &&
              (rx_status->flow != 0)) ? TRUE : FALSE;

	/* Update read pointer of scheduled packets. */
    pico_schd->rptr++;
    schd = &pico_schd->lc_scheduled_pkt_info[pico_schd->rptr];

    if (my_role == MASTER)
    {
        lc_packet_tye = LC_MASTER_DEFAULT_PACKET_TYPE;

        if (basic_logic)
        {
#ifdef COMPILE_SNIFF_MODE
            if (ce_ptr->cont_poll_count == 0)
            {
                lc_packet_tye = LC_INVALID_PACKET_TYPE;
            }
#endif /* end of COMPILE_SNIFF_MODE */
            is_ret_2_cur_esco = TRUE;
        }
        else
        {
            if ((rx_status->flow == 0) && !IS_USE_FOR_BQB)
            {
                lc_packet_tye = LC_INVALID_PACKET_TYPE;
            }
        }
    }
    else
    {
        BB_disable_NBC(phy_piconet_id);
        lc_packet_tye = LC_SLAVE_DEFAULT_PACKET_TYPE;
#ifdef _DAPE_EN_8821_MP_MODIFY_SLV_SNIFF_TIMEOUT
        if (ce_ptr->in_sniff_mode == TRUE)
        {
            lc_sniff_slv_send_pkt &= ~(BIT0 << phy_piconet_id);
            if (lc_sniff_slv_send_pkt == 0)
            {
                UINT16 reg21c = BB_read_baseband_register(0x21c);
                reg21c |= BIT6;
                BB_write_baseband_register(0x21c, reg21c);
            }
        }
#endif

    }

#ifdef _CCH_IOT_CSR_RS_
    if (ce_ptr->waiting_for_rs_several_times == 1)
    {
        lc_packet_tye = LC_MASTER_DEFAULT_PACKET_TYPE;
//    RT_BT_LOG(YELLOW, CCH_DBG_055, 1,ce_ptr->waiting_for_rs_several_times);
    }
#endif

    BB_write_baseband_register(lut_ex_table[lut_index].lower_lut_address, lc_packet_tye);

    if (is_ret_2_cur_esco)
    {
        LC_RETURN_TO_CURRENT_ESCO();
    }

    if (
#ifdef COMPILE_PARK_MODE
        (lmp_self_device_data.number_of_parked_dev == 0) ||
#endif /* end of COMPILE_PARK_MODE */
        (schd->selected_am_addr != BC_AM_ADDR))
    {
        lc_send_packet_in_scatternet(phy_piconet_id);
    }

    UINT32 clk;
    OS_SIGNAL ack_sig_send;

#ifdef ENABLE_LOGGER_LEVEL_2
    RT_BT_LOG(GRAY, LC_ISR_2741, 0, 0);
#endif /* end of ENABLE_LOGGER_LEVEL_2 */

    ack_sig_send.type = LC_HANDLE_ACK_RECD_SIGNAL;
    ack_sig_send.param = (void *)((UINT32)cur_rptr);

    /* ack_sig_send.ext_param = (void *)phy_piconet_id; */
    lc_get_clock_in_scatternet(&clk, ce_ptr->phy_piconet_id);

    clk = clk & 0x0fffffff;
    clk = clk | (phy_piconet_id << 30);
    ack_sig_send.ext_param = (void *)clk;

    if (OS_ISR_SEND_SIGNAL_TO_TASK(lc_tx_task_handle, ack_sig_send)
            != BT_ERROR_OK)
    {
#ifdef ENABLE_LOGGER_LEVEL_2
        RT_BT_LOG(GRAY, LC_ISR_2748, 0, 0);
#endif /* end of ENABLE_LOGGER_LEVEL_2 */
    }
}


SECTION_ISR_LOW void BB_handle_dont_wait_ack_in_rx_interrupt(BB_RX_PARAMS *rx_param)
{
    LMP_CONNECTION_ENTITY *ce_ptr = &lmp_connection_entity[rx_param->ce_index];
    UINT8 lut_index = rx_param->lut_index;
    BZ_REG_S_RX_STATUS *rx_status = &rx_param->rx_status;
    UINT8 basic_logic;
    UINT8 my_role = (ce_ptr->remote_dev_role == SLAVE) ? MASTER : SLAVE;
    UINT16 lc_packet_tye;
    UINT8 is_ret_2_cur_esco = FALSE;

    LC_PICONET_SCHEDULER *piconet_sched;
    piconet_sched = &lc_piconet_scheduler[ce_ptr->phy_piconet_id];

    basic_logic = ((rx_param->lc_baseband_flow == BB_GO) &&
                  (lc_cont_crc_rx_cnt[lut_index] == 0) &&
                  (lc_cont_crc_tx_cnt[lut_index] == 0) &&
                  (rx_status->flow != 0)) ? TRUE : FALSE;

#ifdef _ROM_CODE_PATCHED_
    if (rcp_set_packet_type_in_dont_wait_ack_rx != NULL)
    {
        if (rcp_set_packet_type_in_dont_wait_ack_rx(rx_param, basic_logic))
        {
            return;
        }
    }
#endif

    if (my_role == MASTER)
    {
        lc_packet_tye = LC_MASTER_DEFAULT_PACKET_TYPE;

        if (basic_logic)
        {
#ifdef COMPILE_SNIFF_MODE
            if (ce_ptr->cont_poll_count == 0)
            {
                if (rx_param->schd->tx_status != LC_TX_READY)
                {
                    lc_packet_tye = LC_INVALID_PACKET_TYPE;
                }
            }
#endif /* end of COMPILE_SNIFF_MODE */
            is_ret_2_cur_esco = TRUE;
        }
        else
        {
            if ((rx_status->flow == 0) && !IS_USE_FOR_BQB)
            {
                lc_packet_tye = LC_INVALID_PACKET_TYPE;
            }
        }
    }
    else
    {
        lc_packet_tye = LC_SLAVE_DEFAULT_PACKET_TYPE;
    }

#ifdef _CCH_IOT_CSR_RS_
    if (ce_ptr->waiting_for_rs_several_times == 1)
    {
        lc_packet_tye = LC_MASTER_DEFAULT_PACKET_TYPE;
        //RT_BT_LOG(YELLOW, CCH_DBG_055, 1,ce_ptr->waiting_for_rs_several_times);
    }
#endif

    BB_write_baseband_register(lut_ex_table[lut_index].lower_lut_address,
                               lc_packet_tye);

    if (is_ret_2_cur_esco)
    {
        LC_RETURN_TO_CURRENT_ESCO();
    }
}

SECTION_ISR_LOW void BB_handle_unicast_nbc_ito_interrupt(UINT8 phy_piconet_id,
                                                   UINT32 cur_clk)
{
    UCHAR lcl_lut_index;
    UINT16 lut_lower_content;
    UINT16 lut_upper_content;
    UINT8 retry_cnt;
    OS_SIGNAL sig_send;
    LC_PICONET_SCHEDULER *piconet_schd;
    LC_SCHEDULED_PKT *schd;

    piconet_schd = &lc_piconet_scheduler[phy_piconet_id];

    schd = &piconet_schd->lc_scheduled_pkt_info[piconet_schd->rptr];

    /* get retry count */
    retry_cnt = BB_read_baseband_register(M_ACCESS_REGISTER) >> 12;

    /* Retry to timeout: release corresponding bzdma tx command */
    bzdma_release_tx_entry(phy_piconet_id + BZDMA_TX_ENTRY_TYPE_PICONET0);

    BB_disable_NBC(phy_piconet_id);

    /* Write Null pkt into LUT here. */
    lcl_lut_index = lc_get_lut_index_from_phy_piconet_id(
                        schd->selected_am_addr, phy_piconet_id);

    lut_lower_content = BB_read_baseband_register(
        lut_ex_table[lcl_lut_index].lower_lut_address);
    lut_upper_content = BB_read_baseband_register(
        lut_ex_table[lcl_lut_index].upper_lut_address);

    LMP_CONNECTION_ENTITY *ce_ptr;
    UINT8 ce_index = lut_ex_table[lcl_lut_index].index_in_CE;
    ce_ptr = &lmp_connection_entity[ce_index];

    if (ce_ptr->remote_dev_role == MASTER)
    {
        BB_write_baseband_register(
            lut_ex_table[lcl_lut_index].lower_lut_address,
            LC_SLAVE_DEFAULT_PACKET_TYPE);
    }
    else
    {
        BB_write_baseband_register(
            lut_ex_table[lcl_lut_index].lower_lut_address,
            LC_INVALID_PACKET_TYPE);
    }

#if defined(ENABLE_SCO) && (!defined(_CCH_NO_PAUSE_SCO_) || defined(_DAPE_TEST_FOR_HID_SCO))
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

    sig_send.type = LC_HANDLE_RESCHEDULE;
    sig_send.param = (void *)((UINT32)phy_piconet_id);
    sig_send.ext_param = (void *)((UINT32)lcl_lut_index);
    OS_SEND_SIGNAL_TO_TASK (lc_tx_task_handle, sig_send);

// 20120907 : morgan move to lower location ( this is a bug )
/*
//20120109 morgan add for TX retry counter
#ifdef PTA_EXTENSION
    if( pta_meter_var.bPtaMeterSwitch )
    {
        pta_meter_var.dwTXRetryCnt+= 1;
    }
#endif
*/

#if 0
    if (LMP_GET_CE_INDEX_FROM_AM_ADDR_PPI(schd->selected_am_addr,
       (UINT16)((UINT32) phy_piconet_id), &ce_index) == API_SUCCESS)
    {
        BB_disable_NBC(phy_piconet_id);

    }
#endif

#ifdef _CCH_CFOE_RETRY_
	UINT32 modem_reg = 0;

	INT16 RPT_CFOE_sync =0;
	INT16 RPT_CFOE_trailer =0;

#ifdef _NEW_MODEM_PI_ACCESS_BY_VENDOR_REG_
    modem_reg = RTK_READ_MODEM_REG_PI(0, TRANS_MODEM_REG(0x6A));
#else
    modem_reg =  lc_read_rf_rx_backup_status_rpt_reg(1); //0x6a
#endif
	RPT_CFOE_sync = (modem_reg & 0x01FF);
	if (RPT_CFOE_sync>256)
		RPT_CFOE_sync = RPT_CFOE_sync- 512;


#ifdef _NEW_MODEM_PI_ACCESS_BY_VENDOR_REG_
    modem_reg = RTK_READ_MODEM_REG_PI(0, TRANS_MODEM_REG(0x6C));
#else
    modem_reg =  lc_read_rf_rx_backup_status_rpt_reg(2); //0x6c
#endif
	RPT_CFOE_trailer = (modem_reg & 0x01FF);
	if (RPT_CFOE_trailer>256)
		RPT_CFOE_trailer = RPT_CFOE_trailer - 512;

	RT_BT_LOG(BLUE, CCH_DBG_049, 2,RPT_CFOE_sync,RPT_CFOE_trailer);
#endif

    if (cur_clk != 0xFFFFFFFF)
    {

//20120109 morgan add for TX retry counter
#ifdef PTA_EXTENSION
    if( pta_meter_var.bPtaMeterSwitch == 1)
    {
        pta_meter_var.dwTXRetryCnt+= 1;
    }
#endif


        RT_BT_LOG(YELLOW, ITO_MSG_RETX_TIMEOUT, 11,
            cur_clk, retry_cnt, phy_piconet_id, schd->selected_am_addr,
            lcl_lut_index, lut_upper_content, lut_lower_content,
            schd->rx_count, schd->flow_ctrl_count,
            schd->fw_sent_time, schd->fw_sent_time_us);
#ifdef _DAPE_TEST_CHK_CLK
    UINT8 i;

    UINT8 cam_offset = CAM_ADDR_OFFSET_LAYER2 + lcl_lut_index*CAM_ADDR_OFFSET_LAYER2_SIZE + CAM_ADDR_OFFSET_AFH;
    UINT32 afh_map_cam[3];

    for (i = 0; i < 3; i ++)
    {
        afh_map_cam[i] = BB_read_sc_cam( (cam_offset + i));
    }

RT_BT_LOG(WHITE, YL_DBG_HEX_10, 10,
ce_index, cam_offset,
afh_map_cam[0], afh_map_cam[1], afh_map_cam[2],
BB_read_native_clock(),
BB_read_baseband_register(0x22),
BB_read_baseband_register(0x24),
BB_read_baseband_register(0x26),
BB_read_baseband_register(CHANNEL_REGISTER));
#endif

    }
}

#ifdef _DAPE_TEST_ACL_RX_FIFO_DBG
SECTION_ISR void BB_check_acl_rx_fifo(BZ_REG_S_RX_PL_HDR *pl_hdr,
                                            UINT16 payload_header,
                                            UINT16 packet_header)
{
    if (pl_hdr->len)
    {
        UINT32 dape_reg;
        UINT32 dape_acl_rx_rdptr_current = 0;
        UINT32 dape_acl_rx_wrptr_current = 0;
        UINT32 dape_clk;
        UINT32 packet_len;
        UINT32 payload_packet_len = pl_hdr->len;
        dape_reg = RD_U32_BZDMA_REG(BZDMA_REG_ACL_RXFIFO_PTR);
        dape_acl_rx_rdptr_current = (dape_reg >> 16) & 0x7FF;
        dape_acl_rx_wrptr_current = dape_reg & 0x7FF;

        if ((dape_acl_rx_rdptr_current != dape_acl_rx_wrptr_current)
             && (lc_rx_pkt_header_q.write_pointer == lc_rx_pkt_header_q.read_pointer))
        {
            if (dape_acl_rx_wrptr_current < dape_acl_rx_rdptr_current)
            {
                packet_len = ((dape_acl_rx_wrptr_current -
                               dape_acl_rx_rdptr_current + 2048)*2 + 4);
            }
            else
            {
                packet_len = ((dape_acl_rx_wrptr_current -
                                       dape_acl_rx_rdptr_current)*2 + 4);
            }
            if ((payload_packet_len + payload_packet_len % 2) != packet_len)
            {
                // gpio_one_pull_high(1);
                // gpio_one_pull_low(1);
                //SET_BT_GPIO_OUTPUT_3_0(0x1);
                //SET_BT_GPIO_OUTPUT_3_0(0);
                RT_BT_LOG(RED, DAPE_TEST_LOG500, 9, payload_header,
                          packet_header,
                          dape_acl_rx_rdptr_current, dape_acl_rx_wrptr_current,
                          dape_acl_rx_rdptr, dape_acl_rx_wrptr,
                          dape_reg,
                          (payload_packet_len + payload_packet_len % 2),
                          packet_len);
            }
        }
    }
}
#endif
#ifdef _SUPPORT_SECURE_CONNECTION_
SECTION_ISR void BB_handle_secure_conn_rx_interrupt(BB_RX_PARAMS *rx_param, UINT8 good_pkt)
{
    LMP_CONNECTION_ENTITY *ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    BZ_REG_S_RX_STATUS *rx_status = &rx_param->rx_status;
    BZ_REG_S_RX_PL_HDR *pl_hdr = &rx_param->pl_hdr;
    UINT16 ce_index = rx_param->ce_index;
    UINT32 cur_clock;
    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;

#ifdef _ROM_CODE_PATCHED_
    if (rcp_rx_interrupt_secure_conn_func != NULL)
    {
        if (rcp_rx_interrupt_secure_conn_func((void*)&rx_param, good_pkt))
        {
            return;
        }
    }
#endif

    lc_get_clock_in_scatternet(&cur_clock, rx_param->phy_piconet_id);

#ifdef _CCH_SC_ECDH_P256_
    if (((pl_hdr->mic_err) && (!rx_status->crc_err))
#ifdef _SECURE_CONN_REFRESH_KEY_BY_VENDOR_CMD
          || (g_refresh_key_en)
#endif
       )
    {
#ifdef _SECURE_CONN_REFRESH_KEY_WHEN_CONTINUOUS_MIC
        ce_ptr->mic_err_cnt ++;
#ifdef _SECURE_CONN_REFRESH_KEY_BY_VENDOR_CMD
        if (g_refresh_key_en)
        {
            ce_ptr->mic_err_cnt = g_refresh_key_en;
            g_refresh_key_en = 0;
        }
#endif
        if (g_sc_refresh_key_when_continuous_mic_err)
        {
            if (ce_ptr->mic_err_cnt == 3)
            {
                /* Start the refresh encryption procedure */
                auth->txn_params.auth_role = BZ_AUTH_ROLE_INITIATOR;
                bz_auth_perform_pause_encryption_action(ce_index, auth);
                bz_auth_perform_super_state_transition(auth,
                        HCI_REFRESH_ENCRYPTION_KEY);
            }
            if (ce_ptr->mic_err_cnt == 4)
            {
                UINT8 reason = HOST_REJECTED_SECURITY_REASONS_ERROR;
                ce_ptr->disconnect_reason = reason;
#ifdef _CCH_PAGE_CON_
                ce_ptr->connect_reason = 0;
#endif
                if (lmp_pend_acl_disconnect_from_isr(ce_index, reason) == API_SUCCESS)
                {
                    RT_BT_LOG(RED, SECURE_CONN_LOG3, 1, cur_clock);
                }

            }
#endif
        }
#ifdef _CCH_SC_ECDH_P256_LOG
                UINT32 tx_msb, tx_lsb, rx_msb, rx_lsb, daycounter;
                BB_read_sc_count(ce_index, &tx_msb, &tx_lsb, &rx_msb, &rx_lsb, &daycounter);
                RT_BT_LOG(RED, SECURE_CONN_LOG4, 18,
                cur_clock, ce_ptr->mic_err_cnt,ce_index,
                rx_param->phy_piconet_id,
                rx_param->am_addr, pl_hdr->len, pl_hdr->llid, lmp_connection_entity[ce_index].ptt_status,
                rx_status->hec_err,
                rx_status->crc_err,
                *rx_status, *pl_hdr,
                tx_msb,  tx_lsb, rx_msb, rx_lsb, daycounter);
#endif
#ifndef _DAPE_NO_RETURN_WHEN_MIC_ERR
                return;
#endif
            }
#if defined (_CCH_SC_ECDH_P256_LOG_RX) || defined (_SECURE_CONN_REFRESH_KEY_WHEN_CONTINUOUS_MIC)
            else if((pl_hdr->len > 0)&&(good_pkt == TRUE))
            {
#ifdef _SUPPORT_CSB_RECEIVER_
                if(!bt_3dd_var.csb_rx_param.enable)
#endif
                {
#ifdef _CCH_SC_ECDH_P256_LOG_RX
                    UINT32 tx_msb, tx_lsb, rx_msb, rx_lsb, daycounter;
                    BB_read_sc_count(ce_index, &tx_msb, &tx_lsb, &rx_msb, &rx_lsb, &daycounter);

                    RT_BT_LOG(GRAY, SECURE_CONN_LOG5, 16,
                    cur_clock, ce_index,
                    rx_param->phy_piconet_id,
                    rx_param->am_addr, pl_hdr->len, pl_hdr->llid, lmp_connection_entity[ce_index].ptt_status,
                    ce_ptr->mic_err_cnt,
                    *rx_status, *pl_hdr,
                    tx_msb, tx_lsb, rx_msb, rx_lsb, daycounter);
#endif
                }
#ifdef _SECURE_CONN_REFRESH_KEY_WHEN_CONTINUOUS_MIC
                ce_ptr->mic_err_cnt = 0;
#endif
            }
#endif
#endif
        }
#endif
SECTION_ISR UINT8 BB_handle_crc_pkt_in_rx_interrupt(BB_RX_PARAMS *rx_param,
                                                             UINT8 flush)
{
    LUT_EXTENSION_TABLE *ex_lut = &lut_ex_table[rx_param->lut_index];
    UCHAR *lcl_rx_seqn_bit;
    BZ_REG_S_RX_STATUS *rx_status = &rx_param->rx_status;

#ifdef _CCH_SC_ECDH_P256_MIC_FLAG
    BZ_REG_S_RX_PL_HDR *pl_hdr = &rx_param->pl_hdr;
    UINT8 crc_error = (rx_status->crc_err) | (pl_hdr->mic_err) ;
#else
    UINT8 crc_error = rx_status->crc_err;
#endif
    UINT8 eir_recv = rx_status->eir;
    UINT8 pkt_received = FALSE;
    UINT8 explicit_ack = FALSE;
    PKT_HEADER *ppkt;
    UINT16 read_old;
    UINT16 read_new;
    UINT8 wptr = lc_rx_pkt_header_q.write_pointer;

    ppkt = &lc_rx_pkt_header_q.pkt_header_queue[wptr];

    if (rx_status->pkt_type == BB_FHS)
    {
        /* Seqn bit is not valid for Inq-response packets */
#ifdef COMPILE_INQ_RES_EVENT_WITH_RSSI
        if (lmp_self_device_data.lc_cur_dev_state == LC_CUR_STATE_INQUIRY)
        {
            /* Store RSSI to send with Inquiry result */
            ppkt->clk = (BB_read_baseband_register(NATIVE_CLOCK1_REGISTER)) |
                  ((BB_read_baseband_register(NATIVE_CLOCK2_REGISTER)) << 16);
        }
#endif /* end of COMPILE_INQ_RES_EVENT_WITH_RSSI */
    }
    else if (eir_recv == 0)
    {
        /* not EIR pkt */

        if ((rx_status->lt_addr == BC_AM_ADDR) &&
            (lc_sca_get_piconet_role(rx_param->phy_piconet_id) == SLAVE))
        {
            lcl_rx_seqn_bit = &ex_lut->bc_seqn_bit;
#ifdef SECURE_CONN_BROADCAST_CHK
            RT_BT_LOG(YELLOW, DAPE_TEST_LOG552, 2,crc_error, eir_recv);
#endif
        }
        else
        {
            lcl_rx_seqn_bit = &ex_lut->remote_seqn_bit;
        }

        read_old = BB_read_baseband_register(ex_lut->upper_lut_address);
        read_new = read_old;

        /* Check for toggling of received packet */
        if (rx_status->seqn == (*lcl_rx_seqn_bit))
        {
            /* Duplicate packet. Ignore Payload; Send ACK even if CRC error
            * check for crc error
            */
            if (crc_error)
            {
                /* Send explict ack, If it is not CRC Error BB will send
                * ACK automatically
                */
                read_new |= 0x0004;
            }
            explicit_ack = TRUE;
            flush = TRUE;

            read_new |= 0x0001;
        }

        if (!crc_error)
        {
            /* Update remote seqn bit in extension lut to keep track of remote
            * seqn bit on the bases of am_addr
            */
            if (flush != TRUE)
            {
                /* Seqn bit is not valid for FHS packets */
                *lcl_rx_seqn_bit = rx_status->seqn;
            }
        }

#ifndef _DAPE_TEST_FIX_HW_SCO_NAK_SNIFF_BUG
        if ((flush == TRUE) && (explicit_ack == FALSE))
        {
            read_new &= 0xFFFB;
        }
#else
        LMP_CONNECTION_ENTITY *ce_ptr;
        ce_ptr = &lmp_connection_entity[rx_param->ce_index];
        if ((flush == TRUE) && (explicit_ack == FALSE))
        {
            read_new &= 0xFFFB;
            ce_ptr->should_ack = 0;
            //RT_BT_LOG(WHITE, DAPE_TEST_LOG213, 2,BB_read_native_clock(), ce_index);

        }
        else
        {
            if (!crc_error)
            {
                ce_ptr->should_ack = 1;
            }
            //RT_BT_LOG(WHITE, DAPE_TEST_LOG213, 2,BB_read_native_clock(), ce_index);
        }
#endif

        if (read_new != read_old)
        {
            /* modify update content */
            BB_write_baseband_register(ex_lut->upper_lut_address, read_new);
        }

#ifdef COMPILE_SNIFF_MODE
        lc_check_exit_ssr(rx_param->ce_index);
#endif /* end of COMPILE_SNIFF_MODE */
    }

    if (!crc_error || eir_recv) /** EIR Data */
    {
#ifdef _DAPE_TEST_ACL_RX_FIFO_DBG
        BB_check_acl_rx_fifo(&rx_param->pl_hdr,
                                rx_param->payload_header,
                                rx_param->packet_header);
#endif

        /* Queue the packet in the headers queue */
        ppkt->packet_header = rx_param->packet_header;
        ppkt->payload_header = rx_param->payload_header;
        ppkt->flush_flag = flush;
        ppkt->ce_index = rx_param->ce_index;

#ifndef _DAPE_TEST_NEW_HW_READ_RSSI_STACK
        ppkt->rssi = (INT16) lc_read_rssi_register();
#else
        ppkt->rssi = rx_param->rssi;
#endif
        pkt_received = TRUE;

        /* queue packet and update write pointer of queue */
        lc_rx_pkt_header_q.write_pointer =
                            (wptr + 1) & (LC_MAX_NUMBER_OF_HEADERS - 1);
    }
#ifdef SECURE_CONN_BROADCAST_CHK
    if (rx_status->lt_addr == BC_AM_ADDR)
    {
        RT_BT_LOG(YELLOW, DAPE_TEST_LOG522,2, crc_error,eir_recv);
    }
#endif
    return pkt_received;
}

SECTION_ISR void BB_handle_fhs_and_eir_in_rx_interrupt(BB_RX_PARAMS *rx_param)
{
    BZ_REG_S_RX_PL_HDR *pl_hdr = &rx_param->pl_hdr;
    BZ_REG_S_RX_STATUS *rx_status = &rx_param->rx_status;
    UCHAR pkt_received = FALSE;

    if (rx_status->pkt_type == BB_FHS)
    {
        /* if HW receive FHS pkt, it can show length = 0 in the register.
           so FW fill the actual length filed to help handle pkt */
        pl_hdr->len = 18;
    }
 #ifdef _DAPE_TEST_ACL_RX_FIFO_DBG
    BB_check_acl_rx_fifo(pl_hdr, rx_param->payload_header, rx_param->packet_header);
#endif

    pkt_received = BB_handle_crc_pkt_in_rx_interrupt(rx_param, FALSE);

    if (pkt_received)
    {
        OS_SIGNAL rx_sig_send;
#ifdef _DONT_RESERVE_ACL_PACKET_FOR_EIR_PKT_
        UINT32 rsvd_buf = FALSE;
#endif

        /* Send the signal to lc_rx_task to pick up the packet*/
        rx_sig_send.type = LC_HANDLE_RECD_PACKET_SIGNAL;
#ifdef _DONT_RESERVE_ACL_PACKET_FOR_EIR_PKT_
        rx_sig_send.param = (void*)(rsvd_buf);
#endif
        OS_ISR_SEND_SIGNAL_TO_TASK ( lc_rx_task_handle, rx_sig_send );
    }
}

SECTION_ISR void BB_handle_rx_interrupt_in_sniff(UINT8 lut_index, UINT8 ce_index)
{
#ifdef COMPILE_SNIFF_MODE
    if ((lut_index != INVALID_LUT_INDEX) && (ce_index != INVALID_CE_INDEX))
    {
        /* Sniff + Receive optimization : Stop polling near any sniff instant */
        UINT16 curr_pkt_type;
        curr_pkt_type = BB_read_baseband_register(
                                lut_ex_table[lut_index].lower_lut_address);

        if (curr_pkt_type == LC_MASTER_DEFAULT_PACKET_TYPE)
        {
            if (lmp_self_device_data.num_of_sniff_connections != 0)
            {
                if (lmp_connection_entity[ce_index].in_sniff_mode == FALSE)
                {
                    if (lmp_self_device_data.next_sniff_instant != 0xFFFFFFFF)
                    {
                        UINT32 curr_clock;

                        curr_clock = BB_read_native_clock();

                        if ((lmp_self_device_data.next_sniff_instant -
                           curr_clock) < LC_MIN_SLOTS_BEFORE_SNIFF_STOP_POLLING)
                        {
                            /* Write invalid pkt type in LUT. */
                            BB_write_baseband_register(
                                lut_ex_table[lut_index].lower_lut_address,
                                LC_INVALID_PACKET_TYPE);
                        }
                    }
                }
            }
        }

        if (lmp_connection_entity[ce_index].cont_poll_count != 0)
        {
            lmp_connection_entity[ce_index].cont_poll_count--;
        }
    }
#endif /* end of COMPILE_SNIFF_MODE */

}

SECTION_ISR void BB_handle_rx_packet(BB_RX_PARAMS *rx_param)
{
    OS_SIGNAL rx_sig_send;
    UINT32 rsvd_buf = FALSE;
    BZ_REG_S_RX_PL_HDR *pl_hdr = &rx_param->pl_hdr;
    BZ_REG_S_RX_STATUS *rx_status = &rx_param->rx_status;

    do
    {
#ifdef _DONT_RESERVE_ACL_PACKET_FOR_EIR_PKT_
        if ((rx_status->eir) && (rx_status->lt_addr ==0))
        {
            break;
        }
#endif

#ifdef _DONT_RESERVE_ACL_PACKET_FOR_UNEXPECTED_CASE_
#ifdef _CCH_SC_ECDH_P256_MIC_FLAG
        if ((rx_status->crc_err || pl_hdr->mic_err || rx_status->hec_err || !rx_status->rx_pkt) ||
            ((rx_status->pkt_type == BB_DM1) && (pl_hdr->llid == L_CH_LMP)))
#else
        if ((rx_status->crc_err || rx_status->hec_err || !rx_status->rx_pkt) ||
            ((rx_status->pkt_type == BB_DM1) && (pl_hdr->llid == L_CH_LMP)))
#endif
        {
            /* 1. Do not reserve acl buffer when receive invalid or
                  error status
               2. Because LMP has additional buffer, do not reserve acl
                  buffer for LMP packet
               ** This process can do an early check and avoid fw to enter
                  flow control easy - austin */
            break;
        }
#endif
        rsvd_buf = TRUE;

        os_reserve_buffer();
        if ((rx_param->lc_baseband_flow == BB_GO) &&
             (os_get_reserved_buffers() >=
              (BT_FW_TOTAL_ACL_PKTS_TO_HOST - BT_FW_ACL_RX_FLOW_ZERO_DIFF)))
        {
            UCHAR phy_piconet_id;
            phy_piconet_id = rx_status->pid_l | (pl_hdr->pid_h << 1);
            UINT32 cur_clock;
            lc_get_clock_in_scatternet(&cur_clock, phy_piconet_id);

            BB_write_flow_stop();
            RT_BT_LOG(GREEN, LC_ISR_3118, 3, cur_clock,
                                             phy_piconet_id,
                                             os_get_reserved_buffers());
        }
    }
    while (0);

    /* Send the signal to lc_rx_task to pick up the packet*/
    rx_sig_send.type = LC_HANDLE_RECD_PACKET_SIGNAL;
    rx_sig_send.param = (void*)(rsvd_buf);
    OS_ISR_SEND_SIGNAL_TO_TASK ( lc_rx_task_handle, rx_sig_send );
}

SECTION_ISR void BB_check_rx_ack_with_scheduler(BB_RX_PARAMS *rx_param,
                                                       UINT8 *waiting_for_ack,
                                                       UINT8 *rx_ack)
{
    LC_SCHEDULED_PKT *schd = rx_param->schd;
    BZ_REG_S_RX_STATUS *rx_status = &rx_param->rx_status;
    LMP_CONNECTION_ENTITY *ce_ptr;
    UINT32 lcl_rx_pkt_type = rx_param->lcl_rx_pkt_type;
    UINT8 phy_piconet_id = rx_param->phy_piconet_id;
    UINT8 lut_index = rx_param->lut_index;

   /* Handle the received ACK from remote device. */
    if ((schd->tx_status == LC_TX_SENT) && (rx_param->ce_index == schd->ce_index))
    {
        *waiting_for_ack = TRUE;

        ce_ptr = &lmp_connection_entity[rx_param->ce_index];

        if (schd->selected_am_addr == rx_param->am_addr)
        {
            schd->rx_count++;

            if (!rx_status->flow)
            {
                schd->flow_ctrl_count++;
            }

#ifdef _RATE_ADAPTION_SAMPLE_
            if (ce_ptr->ptt_status == LMP_PTT_ENABLED)
            {
                if (lcl_rx_pkt_type & BB_RX_ISR_CRC_2M_PKT)
                {
                    ce_ptr->send_2m_edr_ack_count++;
                }
                else if (lcl_rx_pkt_type & BB_RX_ISR_CRC_3M_PKT)
                {
                    ce_ptr->send_3m_edr_ack_count++;
                }
            }
#endif
        }

        /* We are Expecting an ack, Extracting ack from Received arqn bit */
        if (((rx_param->am_addr != BC_AM_ADDR) ||
            (ce_ptr->remote_dev_role == SLAVE)) &&
            rx_status->arqn &&
            ((lc_waiting_for_crc_pkt_ack[phy_piconet_id] == TRUE) ||
             ((lc_waiting_for_crc_pkt_ack[phy_piconet_id] == FALSE) &&
              !(RD_U32_BZDMA_REG(BZDMA_REG_TX_CMD(
               BZDMA_TX_ENTRY_TYPE_PICONET0 + phy_piconet_id)) & BIT31))))
        {
            *rx_ack = BB_ACK;

            schd->tx_status = LC_TX_IDLE;

            lc_cont_crc_tx_cnt[lut_index] += 2;

            /* we can release tx entry after receive ACK */
            bzdma_release_tx_entry(BZDMA_TX_ENTRY_TYPE_PICONET0 + phy_piconet_id);

            BB_disable_NBC(phy_piconet_id);

            LC_RETURN_TO_CURRENT_ESCO();

            lc_waiting_for_crc_pkt_ack[phy_piconet_id] = FALSE;

#if defined(ENABLE_SCO) && (!defined(_CCH_NO_PAUSE_SCO_) || defined(_DAPE_TEST_FOR_HID_SCO))
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
        }
    } /* end of if (schd->tx_status == LC_TX_SENT) */

    /* Update hardware flow bits. */
    lut_ex_table[lut_index].bb_flow = rx_status->flow;
}

SECTION_ISR UINT8 BB_adjust_hw_schduler_after_check_ack(BB_RX_PARAMS *rx_param,
                                                       UINT8 waiting_for_ack, UINT8 rx_ack,
                                                       UINT16 lut_low_fix_content)
{
    UINT8 phy_piconet_id = rx_param->phy_piconet_id;
    UINT8 am_addr = rx_param->am_addr;

    if (waiting_for_ack == TRUE)
    {
        if (rx_ack == BB_ACK)
        {
            BB_handle_wait_ack_in_rx_interrupt(rx_param);
        }
        else
        {
            if ((rx_param->rx_status.flow == 0) && !IS_USE_FOR_BQB)
            {
#ifdef _DAPE_TEST_CHK_LUT
                if (lc_sca_manager.pnet[phy_piconet_id].active && (am_addr != BC_AM_ADDR))
#else
                if (am_addr != BC_AM_ADDR)
#endif
                {
                    UINT8 invalid_quit;

                    invalid_quit = bzdma_invalid_txcmd(
                                phy_piconet_id + BZDMA_TX_ENTRY_TYPE_PICONET0,
                                am_addr, 0);

                    if (!invalid_quit)
                    {
                        lc_waiting_for_crc_pkt_ack[phy_piconet_id] = FALSE;
                        BB_handle_unicast_nbc_ito_interrupt(phy_piconet_id,
                                                            0xFFFFFFFF);
                    }
                    else
                    {
                        RT_BT_LOG(RED, CCH_DBG_082, 2,phy_piconet_id, am_addr);
                    }
                }

#ifdef _CCH_TXRX_TRI_
                gpio_one_pull_high(7);
                gpio_one_pull_low(7);
#endif
            }
        }
    }
    else
    {
        /* After a response to a NON CRC packet, Write invalid pkt type
         * to stop transmission to this slave */
#ifdef _DAPE_TEST_CHK_LUT
        if ((rx_param->ce_index != INVALID_CE_INDEX) &&
            lc_sca_manager.pnet[phy_piconet_id].active)
#else
        if (rx_param->ce_index != INVALID_CE_INDEX)
#endif
        {
            BB_handle_dont_wait_ack_in_rx_interrupt(rx_param);
        }

#ifdef TEST_MODE
        if (lut_low_fix_content != 0xFFFF)
        {
            BB_write_baseband_register(lut_ex_table[rx_param->lut_index].lower_lut_address,
                                       lut_low_fix_content);
        }

        if ((lc_tci_pause_flag == TRUE) &&
             (!(rx_param->lcl_rx_pkt_type & (BB_RX_ISR_POLL_NULL_FHS))) &&
               (rx_param->pl_hdr.llid != L_CH_LMP))
        {
            return TRUE;
        }
#endif /* end of TEST_MODE */

        /*==============================*/
        /* Send Packet Here             */
        /*==============================*/
        if (
#ifdef COMPILE_PARK_MODE
            (lmp_self_device_data.number_of_parked_dev == 0) ||
#endif /* end of COMPILE_PARK_MODE */
            (rx_param->schd->selected_am_addr != BC_AM_ADDR))
        {
            lc_send_packet_in_scatternet(phy_piconet_id);
        }
    } /* else waiting_for_ack == TRUE */
    return FALSE;
}

#ifdef _YL_TEST_MODEM_RX_REPORT
void BB_YL_TEST_MODEM_RX_REPORT_in_rx_interrupt(BB_RX_PARAMS *rx_param)
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

        RT_BT_LOG(YELLOW, MDM_LOG_023, 16,
                (rx_param->rx_channel << 8) | rx_param->tx_channel,
                rx_param->rssi, rx_param->packet_header, rx_param->payload_header,
                    rx_report_array[0],                        rx_report_array[1],
                    rx_report_array[2],                        rx_report_array[3],
                    rx_report_array[4],                        rx_report_array[5],
                    rx_report_array[6],                        rx_report_array[7],
                    rx_report_array[8],                        rx_report_array[9],
                    rx_report_array[10],                        rx_report_array[11]);
    }
}
#endif

#ifdef _YL_NEW_MODEM_SRAM_DEBUG
void BB_YL_NEW_MODEM_SRAM_DEBUG_in_rx_interrupt(BZ_REG_S_RX_STATUS *rx_status)
{
    if (g_modem_sram_debug_en && g_modem_sram_debug_xdh5_trig_en)
    {
        if (g_modem_sram_debug_captured_flag == 0)
        {
            UINT8 is_xDH5 = (rx_status->pkt_type == BB_DH5) ||
                        (rx_status->pkt_type == BB_2_DH5) ||
                        (rx_status->pkt_type == BB_3_DH5);
            UINT8 is_xdh5_crc_trigered = (g_modem_sram_debug_xdh5_trig_crc_ok ? (rx_status->crc_err==0) : rx_status->crc_err);

            if ( (g_modem_sram_debug_xdh5_trig_en && is_xdh5_crc_trigered && is_xDH5) )
            {
                rtl8821_btrf_modem_sram_debug_set_en(0);
                g_modem_sram_debug_captured_flag = 1;
//                RT_BT_LOG(YELLOW, MDM_LOG_018, 4, rxtx_channel, rx_param.rssi, rx_param.packet_header, rx_param.payload_header);
            }
        }
    }
    if (g_modem_sram_debug_en && g_modem_sram_debug_xdh5_xdh3_3dh1_error_log_en)
    {
        UINT8 is_xDH5 = (rx_status->pkt_type == BB_DH5) ||
                        (rx_status->pkt_type == BB_2_DH5) ||
                        (rx_status->pkt_type == BB_3_DH5);
        UINT8 is_xDH3 = (rx_status->pkt_type == BB_DH3) ||
                        (rx_status->pkt_type == BB_2_DH3) ||
                        (rx_status->pkt_type == BB_3_DH3);
        UINT8 is_3DH1 = (rx_status->pkt_type == BB_3_DH1);
        if ((rx_status->crc_err) && (is_xDH5 || is_xDH3 || is_3DH1))
        {
//            RT_BT_LOG(YELLOW, MDM_LOG_019, 5, rxtx_channel, rx_param.rssi, rx_param.packet_header, rx_param.payload_header, BB_read_baseband_register(0x94));
        }
    }
}
#endif

void bt_csb_rx_control_supervision_timer_task(void *no_arg, uint32_t enabled)
{
    bt_csb_rx_control_supervision_timer((UINT8) enabled);
}

/**
 * Handles packet received interrupt from the baseband. This function
 * decodes the packet, and signals appropriate layer. It also decodes
 * and processes packet header details like am-addr, SEQN, ARQN and
 * flow bits.
 *
 * \param None.
 *
 * \return  None.
 */
SECTION_ISR void BB_handle_rx_interrupt(BT_RX_INT_ARGUMENT *argu)
{
    LC_SCHEDULED_PKT *schd;
    LUT_EXTENSION_TABLE *ex_lut;
    UINT16 payload_header;
    UINT16 packet_header;
    UINT16 ce_index;
    UCHAR am_addr;
    UCHAR lut_index;
    UCHAR phy_piconet_id;
    UCHAR flush = FALSE;
    UCHAR rx_ack = BB_NAK;
    UCHAR waiting_for_ack = FALSE;
    UCHAR pkt_received = FALSE;
    UCHAR lc_baseband_flow;
    UINT16 rxtx_channel;
    LC_PICONET_SCHEDULER *piconet_schd;
    BB_RX_PARAMS rx_param;

#ifdef TEST_MODE
    UINT16 lut_low_fix_content = 0xFFFF;
#endif /* end of TEST_MODE */

    UINT32 lcl_rx_pkt_type;

#ifdef COMPILE_ESCO
    UINT16 esco_ce_index;
#endif /* end of COMPILE_ESCO */

    UINT16 rssi;

    UINT8 good_pkt = FALSE;
#ifdef _SUPPORT_CSB_RECEIVER_
    UINT16 clk_report0 = 0;
    UINT16 clk_report1 = 0;
#endif

    generate_new_afh_signal = FALSE;

#ifdef LE_MODE_EN
    if (argu != NULL)
    {
        payload_header = argu->payload_hdr;
        packet_header = argu->packet_hdr;
#ifdef _DAPE_TEST_NEW_HW_READ_RSSI_STACK
        rssi = argu->rssi;
#endif
    }
    else
#endif
    {
        /* In 8821, we need to read RSSI before reading RECEIVED_STATUS_REGISTER. */
#ifdef _DAPE_TEST_NEW_HW_READ_RSSI_STACK
        rssi = BB_read_baseband_register(BB_RSSI_STACK_REG);
        rssi &= (UINT16)(0x003F);
#endif

#ifdef _SUPPORT_CSB_RECEIVER_
        /* Warning!!! Do Not Move This operation after reading packet header!!! */
        clk_report0 = BB_read_baseband_register(BB_CSB_RX_SYNC_CLK_OFST_REP0_REG);
        clk_report1 = BB_read_baseband_register(BB_CSB_RX_SYNC_CLK_OFST_REP1_REG);
#endif

        /* In 0380, we need to read RECEIVED_PAYLOAD_HEADER_REGISTER register more
           early than RECEIVED_STATUS_REGISTER for meet HW rx interrupt stack
           constraint (or payload_hdr value will be invalid) - austin */
        payload_header = BB_read_baseband_register(RECEIVED_PAYLOAD_HEADER_REGISTER);

        /* Read the packet header */
        packet_header = BB_read_baseband_register(RECEIVED_STATUS_REGISTER);
    }
#ifdef _CSB_RX_DBG_LOG
    if (g_csb_rx_dbg_log)
    {
        RT_BT_LOG(BLUE, DAPE_TEST_LOG525, 6, payload_header, packet_header,
        clk_report0,clk_report1,clk_report1&BIT12,BB_read_native_clock());
    }
#endif

#ifdef _YL_LPS
    lps_gpio_one_pull_high(LPS_GPIO_RX);
//    LPS_DBG_LOG(YELLOW, LPS_LOG_021, 1, packet_header);
    lps_gpio_one_pull_low(LPS_GPIO_RX);
#endif

    BZ_REG_S_RX_PL_HDR *pl_hdr = (BZ_REG_S_RX_PL_HDR *)&payload_header;
    BZ_REG_S_RX_STATUS *rx_status = (BZ_REG_S_RX_STATUS *)&packet_header;

    rxtx_channel = BB_read_baseband_register(CHANNEL_REGISTER);

#ifndef _DAPE_TEST_NEW_HW_READ_RSSI_STACK
    rssi = lc_read_rssi_register();
#endif
#ifdef _CCH_SC_ECDH_P256_MIC_FLAG
    if (rx_status->rx_pkt && !rx_status->hec_err && !rx_status->crc_err
        && !pl_hdr->mic_err)
#else
    if (rx_status->rx_pkt && !rx_status->hec_err && !rx_status->crc_err)
#endif
    {
        good_pkt = TRUE;
    }

    /* Get received packet type */
    lcl_rx_pkt_type = 0x01 << rx_status->pkt_type;

    /* Extract am_addr from received packet header */
    am_addr = rx_status->lt_addr;

    /* Extract physical and logical piconet ids from the packet header and
       information with f\w */
    phy_piconet_id = rx_status->pid_l | (pl_hdr->pid_h << 1);

    /* fill rx parameters (part1) */
    rx_param.payload_header = payload_header;
    rx_param.packet_header = packet_header;
    rx_param.rssi = rssi;
    rx_param.rx_channel = rxtx_channel >> 8;
    rx_param.tx_channel = rxtx_channel & 0xff;
    rx_param.lcl_rx_pkt_type = lcl_rx_pkt_type;
    rx_param.am_addr = am_addr;
    rx_param.phy_piconet_id = phy_piconet_id;

#ifdef _YL_TEST_MODEM_RX_REPORT
    BB_YL_TEST_MODEM_RX_REPORT_in_rx_interrupt(&rx_param);
#endif

#ifdef _YL_NEW_MODEM_SRAM_DEBUG
    BB_YL_NEW_MODEM_SRAM_DEBUG_in_rx_interrupt(rx_status);
#endif

#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8723_RCP_
    if (rcp_BB_handle_rx_interrupt_lps_reset != NULL)
    {
        if (rcp_BB_handle_rx_interrupt_lps_reset((void *)&rx_param))
        {
            return;
        }
    }
#endif
#endif

    /* Check if received pkt has HEC error */
    if (rx_status->hec_err)
    {
        /*
         * Two cases where HEC error packet has to be processed, not dropped:
         * 1. ID indication.
         * 2. This is an EIR packet.
         */
        if ((rx_status->ar_id == FALSE) || (rx_status->eir == FALSE))
        {
#if defined(SCO_OVER_HCI)
            if (pl_hdr->sco_rx)
            {

#ifdef _BRUCE_TEST_PLC_PKT_STATUS
                plc_pkt_status_var.g_plc_hec_err++;   //SCO
#endif

#ifdef _BRUCE_DEBUG_PORT
                UINT16 read;
                read=BB_read_baseband_register(0x25c); //SCO 0x25c[1]=1; g_plc_hec_err
                read |=(UINT16)BIT1;
                BB_write_baseband_register(0x25c,read);
#endif

                /* generated for SCO receive slot? */
                BB_handle_sco_in_rx_interrupt(TRUE, 0);
            }
#endif

            /* this case is hec error */

            /* For Local assessment. */
            lc_log_data_for_channel_assessment(&rx_param, FALSE);
            return;
        }
    }

#if defined (COMPILE_PARK_MODE) || defined(COMPILE_ROLE_SWITCH)
    if (rx_status->ar_id)
    {
        /* handle possible events of ID indication
           (slave unpark or role switch) */
        BB_handle_id_indication_in_rx_interrupt();
        return;
    }
#endif /* defined (COMPILE_PARK_MODE) || defined(COMPILE_ROLE_SWITCH) */

    lut_index = lc_get_lut_index_from_phy_piconet_id(am_addr, phy_piconet_id);
    ex_lut = &lut_ex_table[lut_index];

    /* Initialize the ce_index. */
    ce_index = ex_lut->index_in_CE;

    /* fill rx parameters (part2) */
    rx_param.lut_index = lut_index;
    rx_param.ce_index = ce_index;

    if (((rx_status->pkt_type == BB_FHS) || rx_status->eir) &&
        (rx_status->lt_addr == 0))
    {
        /* speed up the rx interrupt handling - austin */
        BB_handle_fhs_and_eir_in_rx_interrupt(&rx_param);
        return;
    }

#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_
    if (ce_index != INVALID_CE_INDEX)
    {
        if (ll_rssi_manager.msft_monitor_rssi_mode == LE_MSFT_MONITOR_RSSI_VIA_CONN_HANDLE_MODE_MSFT)
        {
            if (ll_rssi_manager.bm_used_legacy_handle & (1 << ce_index))
            {
                INT8 cur_rssi = lc_calculate_log_from_rssi(rssi);
                rssi_app_msft_update_conn_rssi_info(ce_index, cur_rssi);
            }
        }
    }
#endif

    /*============================================*/
    /*          To Check SCO and eSCO Rx          */
    /*============================================*/
    do
    {
#ifdef SCO_OVER_HCI
        /* Check SCO Rx */
        if (pl_hdr->sco_rx)
        {
#ifdef _BRUCE_TEST_PLC_PKT_STATUS
            plc_pkt_status_var.g_plc_tot_num++;   //SCO
            UINT16 read;
            //0x25A[15:0]  is the mse that hw obtained.
            read=BB_read_baseband_register(0x25A);
            plc_pkt_status_var.g_plc_pkt_mse += read;  //SCO
            //RT_BT_LOG(GREEN,YL_DBG_DEC_1,1,read);
#endif

            /* Is it generated for SCO receive slot */
            BB_handle_sco_in_rx_interrupt(FALSE, lcl_rx_pkt_type);
            break;
        }
#endif /* SCO_OVER_HCI */

#ifdef COMPILE_ESCO
        /* Check eSCO Rx */
        if (lmp_get_esco_ce_index_from_am_addr(am_addr, phy_piconet_id,
                                              &esco_ce_index) == API_SUCCESS)
        {
            ce_index = lmp_esco_connection_entity[esco_ce_index].ce_index;
            lmp_sup_timeout_var[ce_index] = 0;

            rx_param.ce_index = ce_index;

            /* For Local assessment. */
            lc_log_data_for_channel_assessment(&rx_param, TRUE);

#ifdef _ROM_CODE_PATCHED_
            /* rcp reserved by yilinli for
              * e.g.
              *    1. set rx_indicator = 1 to indicate packet received
              *        a. clear rx_indicator = 0 at sniff attempt start
              *        b. we can check rx_indicator at sniff attempt end to check if packet received at sniff mode
              *        note: shall co-operate with rcp_rx_interrupt_valid_ce_index_func() )
              *    2. etc.
              */
            if (rcp_rx_interrupt_get_esco_ce_index_func != NULL)
            {
                if (rcp_rx_interrupt_get_esco_ce_index_func((void*)&rx_param, ce_index))
                {
                    return;
                }
            }
#endif

            /* Move the one which is originally located in lc_handle_esco_rx_interrupt(). */
#ifdef _DAPE_TEST_NEW_HW_READ_RSSI_STACK
#ifdef COMPILE_RSSI_REPORTING
            lc_calculate_and_store_rssi(rssi, ce_index);
#endif
#endif

#ifdef _CCH_SC_ECDH_P256_MIC_FLAG
            lc_handle_esco_rx_interrupt(am_addr, phy_piconet_id, packet_header, payload_header);
#else
            lc_handle_esco_rx_interrupt(am_addr, phy_piconet_id, packet_header);
#endif

#ifdef _BRUCE_DEBUG_PORT
            UINT16 read;
            read=BB_read_baseband_register(0x25c); // 0x25c[1]=0; g_plc_hec_err
            read &= (UINT16)(~BIT1);
            BB_write_baseband_register(0x25c,read);
#endif

            return;
        }
#endif /* COMPILE_ESCO */
    }
    while (0);
    /*========= End of SCO and eSCO Check =========*/


    /*======================================================*/
    /*          To Check CSB Receiver and Transmitter       */
    /*======================================================*/
#ifdef _SUPPORT_CSB_RECEIVER_
    if ((clk_report1&BIT12) && (bt_3dd_var.stp_rx_param.enable))
    //if ((bt_3dd_var.stp_rx_param.enable))
    {
        if ((good_pkt == TRUE) && (am_addr == 0) &&
#ifndef _DAPE_TEST_FOR_BCM_CSB_IN_UPF45
            (rx_status->pkt_type == BB_DM3) && (pl_hdr->len == 28))
#else
              (rx_status->pkt_type == BB_DM3))
#endif
        {
            bt_3dd_var.stp_rx_param.clock_offset =
                ((clk_report0 | (clk_report1 <<16)) & (UINT32)(0x0FFFFFFF));
#ifdef _CSB_RX_DBG_LOG
            if (g_csb_rx_dbg_log)
            {
                RT_BT_LOG(BLUE, DAPE_TEST_LOG525, 6,
                BB_read_native_clock(),
                clk_report1, clk_report0,
                bt_3dd_var.stp_rx_param.clock_offset,0,0);
            }
#endif
            bt_3dd_checkin_sync_train_packet(&rx_param);
            return;
        }
    }
#endif
#if defined(_3DD_FUNCTION_SUPPORT_) && !defined(_DO_NOT_SUPPORT_RP_)
    if (IS_SUPPORT_3DG_APP && !bt_3dd_var.is_csb_mode)
    {
        if ((good_pkt == TRUE) && rx_status->eir && (am_addr == 1) &&
            (rx_status->pkt_type == BB_DM1) && (pl_hdr->len == 17))
        {
            bt_3dd_checkin_association_notification_packet(&rx_param);
            return;
        }
    }
#endif
    /*========= End of CSB Receiver and Transmitter =========*/


#ifdef _DAPE_NO_BLOCK_LEGACY_WEHN_SLV_EXIT_SNIFF
    if ((lc_le_pause_status_for_sniff & (BIT0 << phy_piconet_id)) != 0)
    {
        ll_driver_block_legacy_slot_for_le(g_block_legacy_for_le_slot_num);
        lc_le_pause_status &= ~(BIT0 << phy_piconet_id);
        lc_le_pause_status_for_sniff &= ~(BIT0 << phy_piconet_id);
    }
#endif


    if (ce_index != INVALID_CE_INDEX)
    {
        LMP_CONNECTION_ENTITY *ce_ptr;
        ce_ptr = &lmp_connection_entity[ce_index];

        BZ_AUTH_LINK_PARAMS* auth;
        auth = ce_ptr->auth;

#ifdef _CCH_SC_ECDH_P256_START_ENC
        if(lmp_connection_entity[ce_index].send_start_enc)
        {
           // if(lmp_connection_entity[ce_index].auth.secure_conn_enabled)
	    /* Configure to transmit unencrypted and receive encrypted */
            BB_encryption_control(am_addr, phy_piconet_id,
                    BB_ENC_TX_DISABLE_RX_ENABLE);
            lmp_connection_entity[ce_index].send_start_enc = 0;

#ifdef _CCH_SC_ECDH_P256_PAUSE_ENC
            if(auth->secure_conn_enabled)
            {
                BB_write_sc_rxpkcnt_ignore_seqcompare(ce_index, 1);
            }
#endif

        }
#endif

#ifdef _CCH_SC_ECDH_P256_STOP_ENC
        if(lmp_connection_entity[ce_index].send_stop_enc)
        {
            lmp_connection_entity[ce_index].send_stop_enc = 0;
            bz_auth_disable_link_level_encryption(ce_index, FALSE);
            /* Configure to transmit encrypted and receive unencrypted */
            BB_encryption_control(ce_ptr->am_addr, ce_ptr->phy_piconet_id,
                BB_ENC_TX_ENABLE_RX_DISABLE);


#ifdef _CCH_SC_ECDH_P256_PAUSE_ENC

            if(auth->secure_conn_enabled)
            {
                auth->sc_use_enc_rand = 1;
#ifdef _CCH_SC_ECDH_P256_LOG
                RT_BT_LOG(BLUE, CCH_DBG_155, 0, 0);
#endif
            }

#endif

        }
#endif


    }

#ifdef _SUPPORT_SECURE_CONNECTION_
    if (ce_index != INVALID_CE_INDEX)
    {
        if (lmp_connection_entity[ce_index].auth->secure_conn_enabled)
        {
            BB_handle_secure_conn_rx_interrupt(&rx_param, good_pkt);
        }
    }
#endif

    /*===================================*/
    /* Update and Check Current Flow Bit */
    /*===================================*/
    lc_baseband_flow = BB_get_current_flow();

    if (lc_baseband_flow == BB_STOP)
    {
        /* Write arqn 0. */
        AND_val_with_bb_reg_macro(ex_lut->upper_lut_address, ~BIT2);
        flush = TRUE;
    }

    /* Get Number of free packets to decide for s\w flow control */
    if ((os_get_reserved_buffers() >= (BT_FW_TOTAL_ACL_PKTS_TO_HOST -
                                      BT_FW_ACL_RX_FLOW_ZERO_DIFF)))
    {
        BB_write_flow_stop();
        UINT32 cur_clock;
        lc_get_clock_in_scatternet(&cur_clock, phy_piconet_id);

#ifndef _CCH_SC_ECDH_P256_MARK_LOG
        RT_BT_LOG(GREEN, LC_ISR_2104, 4, cur_clock, phy_piconet_id,
                                        os_get_reserved_buffers(),
                                        pool_mgr.pool_array[3].free_pools);
#endif

    }
    else
    {
        BB_write_flow_go();
    }
    lc_baseband_flow = BB_get_current_flow();

    piconet_schd = &lc_piconet_scheduler[phy_piconet_id];
    schd = &piconet_schd->lc_scheduled_pkt_info[piconet_schd->rptr];

    /* fill rx parameters (part3) */
    rx_param.lc_baseband_flow = lc_baseband_flow;
    rx_param.schd = schd;
    rx_param.cur_schd_rptr = piconet_schd->rptr;

#ifdef _ROM_CODE_PATCHED_
    if (rcp_rx_interrupt_2nd_half_func != NULL)
    {
        if (rcp_rx_interrupt_2nd_half_func((void*)&rx_param))
        {
            return;
        }
    }
#endif

    if (ce_index != INVALID_CE_INDEX)
    {
        BB_check_rx_ack_with_scheduler(&rx_param, &waiting_for_ack, &rx_ack);
    }

    /* For Local assessment. */
    lc_log_data_for_channel_assessment(&rx_param, FALSE);

#ifdef TEST_MODE
    if ((ce_index != INVALID_CE_INDEX) &&
        (lmp_connection_entity[ce_index].test_mode_info.test_state == TEST_STARTED))
    {
        if (BB_handle_test_mode_in_rx_interrupt(&rx_param, rx_ack,
                                                &lut_low_fix_content))
        {
            return;
        }
    }
#endif /* TEST_MODE */

    if (lut_index != 0)
    {
        if (lcl_rx_pkt_type & (BB_RX_ISR_CRC_PKT) )
        {
#ifdef SECURE_CONN_PING_EN
            if (OS_IS_TIMER_RUNNING(lmp_connection_entity[ce_index].en_ping_req_timer))
            {
                OS_STOP_TIMER(lmp_connection_entity[ce_index].en_ping_req_timer,0);
                OS_START_TIMER(lmp_connection_entity[ce_index].en_ping_req_timer,
                               lmp_connection_entity[ce_index].max_auth_interval*5);
            }
            if (OS_IS_TIMER_RUNNING(lmp_connection_entity[ce_index].send_max_auth_timeout_timer))
            {
                OS_STOP_TIMER(lmp_connection_entity[ce_index].send_max_auth_timeout_timer,0);
                OS_START_TIMER(lmp_connection_entity[ce_index].send_max_auth_timeout_timer,
                               lmp_connection_entity[ce_index].max_auth_interval*10);
            }
#endif
#ifdef COMPILE_DYNAMIC_POLLING
            if ((ce_index != INVALID_CE_INDEX) &&
                (lmp_connection_entity[ce_index].remote_dev_role == SLAVE) &&
                (lmp_connection_entity[ce_index].no_inactive_tpolls != 0))
            {
                OS_SIGNAL tpoll_sig_send;

                lmp_connection_entity[ce_index].no_inactive_tpolls = 0;
                /* Send signal to program tpoll */
                tpoll_sig_send.type = LC_DYNAMIC_PROGRAM_TPOLL;
                tpoll_sig_send.param = (OS_ADDRESS)((UINT32)ce_index);

                if (OS_ISR_SEND_SIGNAL_TO_TASK(lc_tx_task_handle, tpoll_sig_send)
                        != BT_ERROR_OK)
                {
#ifdef ENABLE_LOGGER_LEVEL_2
                    LC_LOG_INFO(LOG_LEVEL_LOW, ERROR_SENDING_SIGNAL_TO_LC_RX_TASK_N,0,0);
#endif /* end of ENABLE_LOGGER_LEVEL_2 */
                }
            }
#endif /* end of COMPILE_DYNAMIC_POLLING */

#ifdef _CCH_SC_ECDH_P256_MIC_FLAG
            if ( (!rx_status->crc_err) && (!pl_hdr->mic_err) && (!rx_status->hec_err) && (!rx_status->eir))
#else
            if (!rx_status->crc_err && !rx_status->hec_err && !rx_status->eir)
#endif
            {
                lc_cont_crc_rx_cnt[lut_index] += 2;
            }
        }
        else
        {
            lc_cont_crc_rx_cnt[lut_index] >>= 1;
        }
    }

    /* For ack received We should be dependent on baseband registers e.g LUT
    * If baseband is in middle of retransmission gets flow as STOP is received
    * from Remote device, baseband is sending default packet type to remote
    * device and restoring CRC packet after flow as GO comes
    */
    if (BB_adjust_hw_schduler_after_check_ack(&rx_param, waiting_for_ack,
                                              rx_ack, lut_low_fix_content))
    {
#ifdef _CSB_RX_DBG_LOG
        if (g_csb_rx_dbg_log)
        {
            RT_BT_LOG(YELLOW, DAPE_TEST_LOG293, 1,lut_index);
        }
#endif

        return;
    }

    if (lcl_rx_pkt_type & BB_RX_ISR_CRC_AND_FHS_AND_AUX1_PKT)
    {
        pkt_received = BB_handle_crc_pkt_in_rx_interrupt(&rx_param, flush);
    }
    else
    {
        /* If received packet type is not a CRC packet then set NAK
        * check if seqn has toggled
        */
        if ((am_addr != BC_AM_ADDR) &&
            (rx_status->seqn != ex_lut->remote_seqn_bit))
        {
            /* Set NAK in the LUT. Arqn bit will be set by BB when it will
            *  receive a correct crc packet
            */
            AND_val_with_bb_reg_macro(ex_lut->upper_lut_address, 0xFFFB);
        }
    }
#ifdef _ROM_CODE_PATCHED_
    /* rcp reserved by yilinli for
      * e.g.
      *    1. set rx_indicator = 1 to indicate packet received
      *        a. clear rx_indicator = 0 at sniff attempt start
      *        b. we can check rx_indicator at sniff attempt end to check if packet received at sniff mode
      *        note: shall co-operate with rcp_rx_interrupt_get_esco_ce_index_func
      *    2. etc.
      */
    if (rcp_rx_interrupt_valid_ce_index_func != NULL)
    {
        if (rcp_rx_interrupt_valid_ce_index_func((void*)&rx_param, pkt_received))
        {
            return;
        }
    }
#endif

    /* Restart supervision timer var. */
    if (ce_index != INVALID_CE_INDEX)
    {
        LMP_CONNECTION_ENTITY *ce_ptr;

        ce_ptr = &lmp_connection_entity[ce_index];

#ifdef LE_MODE_EN
#ifdef _DAPE_NEW_HW_NO_BLOCK_LEGACY_FOR_LE_WHEN_RETX
        if ((ce_ptr->remote_dev_role == MASTER) && (g_enable_le_block_legacy == 0) &&
        	(NO_BLOCK_LEGACY_FOR_LE_WHEN_RETX) && (NO_BLOCK_LEGACY_FOR_LE_WHEN_LEGACY_SLV))
        {
            lc_le_pause_status |= (BIT0 << phy_piconet_id);
    	    ll_driver_block_legacy_slot_for_le(1);
        }
#endif
#endif

        lmp_sup_timeout_var[ce_index] = 0;

        if( ce_ptr->in_sniff_mode)
        {
            lc_sniff_window_rx_ind[lut_index - 1] = 1;
        }
#ifdef _SUPPORT_CSB_RECEIVER_
        if((bt_3dd_var.csb_rx_param.enable) &&
            (ce_index == bt_3dd_var.csb_rx_param.ce_index))
        {
#ifdef _CSB_CHK_RX_PKT_BY_PCD_REG_IN_8821B_TEST_CHIP
            if (g_beacon_rx_ing)
            {
                lc_sniff_sup_count[lut_index - 1] --;
                if (!rx_status->crc_err)
                {
                    BB_write_baseband_register(BB_CSB_RX_SKIP_REG,
                    (bt_3dd_var.csb_rx_param.skip) | (ce_ptr->phy_piconet_id << 8) |
                    (3 << 10));
                }
                g_beacon_rx_ing = 0;
            }

#endif
            lc_sniff_window_rx_ind[lut_index - 1] = 1;

            if (!rx_status->crc_err)
            {
                BaseType_t high_pri_task_woken = pdFALSE;
                /* restart CSB receiver supervision timer */
                xTimerPendFunctionCallFromISR(
                        bt_csb_rx_control_supervision_timer_task, NULL, TRUE,
                        &high_pri_task_woken);
                portYIELD_FROM_ISR(high_pri_task_woken);
                bt_3dd_var.csb_rx_param.crc_err = 0;
            }
            else
            {
                bt_3dd_var.csb_rx_param.crc_err = 1;
            }
        }
#endif

        if (ex_lut->bb_flow != 0)
        {
            /**
            * Mark as active should not be done here. Since this in
            * interrupt, it will affect the scheduling - it may schedule out
            * of order.
            * Best case is to not queue too many signals to mark it as
            * active, but in normal cases maximum of 2 signals were queued.
            * Alternative solutions might be better.
            */
            if (aclq_is_am_addr_failed(ce_ptr->am_addr, ce_ptr->phy_piconet_id))
            {
                OS_SIGNAL ma_sig_send;
                ma_sig_send.type = LC_HANDLE_MARK_ACTIVE;
                ma_sig_send.param = (void*)((UINT32) (ce_ptr->am_addr));
                ma_sig_send.ext_param = (void*)((UINT32) (ce_ptr->phy_piconet_id));
                OS_ISR_SEND_SIGNAL_TO_TASK(lc_tx_task_handle, ma_sig_send);
            }
        }
    }

    /* Send packet received signal if a CRC packet is received */
    if (pkt_received == TRUE)
    {
        BB_handle_rx_packet(&rx_param);
    }

#ifdef COMPILE_SNIFF_MODE
    /* Sniff + Receive optimization : Stop polling near any sniff instant */
    BB_handle_rx_interrupt_in_sniff(lut_index, ce_index);
#endif /* end of COMPILE_SNIFF_MODE */

#ifdef COMPILE_RSSI_REPORTING
    lc_calculate_and_store_rssi(rssi, ce_index);
#endif /* COMPILE_RSSI_REPORTING */

    return;
}

SECTION_ISR void BB_handle_full_rx_interrupt(BT_RX_INT_ARGUMENT *argu)
{
    BB_handle_rx_interrupt(argu);

#ifdef _BRUCE_DEBUG_PORT
    UINT16 read;
    read=BB_read_baseband_register(0x25c); //SCO  0x25c[1]=0; g_plc_hec_err
    read &=(UINT16)~(BIT1);
    BB_write_baseband_register(0x25c,read);
#endif

    if (generate_new_afh_signal)
    {
        if (otp_str_data.rtk_afh_mechanism_enable)
        {
            new_afh_fill_backup_reg_and_send_signal();
        }
    }
}

#ifdef COMPILE_PARK_MODE
SECTION_ISR_LOW void BB_handle_park_new_conn_to_in_ito_interrupt(void)
{
    UINT16 lcl_upper_lut_address;
    UINT8 phy_piconet_id;
    UINT8 lut_index;
    OS_SIGNAL sig_send;
    UINT16 temp;

    /* No changes to BB. Program LMP to return to park-state. */
    UINT16 temp_var;

    //RT_BT_LOG(GRAY, LC_ISR_3296, 0, 0);

    for (temp_var = 0; temp_var < LMP_MAX_CE_DATABASE_ENTRIES; temp_var++)
    {
        LMP_CONNECTION_ENTITY *ce_ptr;

        ce_ptr = &lmp_connection_entity[temp_var];

        if ((ce_ptr->entity_status == ASSIGNED) &&
            (ce_ptr->ce_status == LMP_PARK_MODE))
        {
            phy_piconet_id = ce_ptr->phy_piconet_id;

            if (ce_ptr->remote_dev_role == SLAVE)
            {
                lut_index = ce_ptr->am_addr;
            }
            else
            {
                lut_index = LC_SCA_SLAVE_1_LUT;

                if (ce_ptr->phy_piconet_id <= SCA_PICONET_FOURTH)
                {
                    lut_index = LC_SCA_SLAVE_1_LUT + ce_ptr->phy_piconet_id;
                }
            }

            lcl_upper_lut_address = lut_ex_table[lut_index].upper_lut_address;

            /* Clear the active bit in the upper lut. */
            AND_val_with_bb_reg_macro(lcl_upper_lut_address,
                                      (UINT16)(~ACTIVE_BIT));

            if (ce_ptr->remote_dev_role == SLAVE)
            {
                lc_park_clear_lc_level_conn_during_unpark_procedure(
                    ce_ptr->am_addr);
            }

            RT_BT_LOG(GRAY, LC_ISR_3364, 2, temp_var, ce_ptr->am_addr);

            break;
        }
    } /* end of for(temp_var = 0; temp_var < LMP_MAX_CE_DATA ... */
    bb_flush_broadcast_fifo();

    lc_slave_init_unpark_pending = FALSE;

    //RT_BT_LOG(GRAY, LC_ISR_3374, 0, 0);

    /* Post signal to lmp-task. */
    sig_send.type = LMP_PARK_NCTO_SIGNAL;
    sig_send.param = (void *)((UINT32)temp_var);
    temp = (UINT16) OS_ISR_SEND_SIGNAL_TO_TASK (lmp_task_handle, sig_send );

    if (temp != BT_ERROR_OK )
    {
#ifdef ENABLE_LOGGER_LEVEL_2
        LC_LOG_ERROR(LOG_LEVEL_LOW,
                     "Error sending signal to lmp_task Error = %d \n", temp);
#endif
    }
}
#endif /* #ifdef COMPILE_PARK_MODE */

#ifdef  COMPILE_ROLE_SWITCH
void BB_handle_mss_success_ito_interrupt_bottom_half(void *no_arg,
        uint32_t no_arg2)
{
    OS_SIGNAL sig_send;
    /* Let hw change role faster, so we move the function here - austin */
    lc_handle_role_switch_complete_in_scatternet();

    sig_send.type = LC_SWITCH_ROLE_COMPLETE_SIGNAL;
    OS_SEND_SIGNAL_TO_TASK(lc_tx_task_handle, sig_send);

    RT_BT_LOG(GRAY, LC_ISR_3449, 0, 0);
}

SECTION_ISR_LOW void BB_handle_mss_success_ito_interrupt(void)
{
    UCHAR new_lut_index;
    UCHAR old_lut_index;
    LUT_EXTENSION_TABLE *ex_lut;

    new_lut_index = lmp_role_switch_data.new_lut_index;
    old_lut_index = lmp_role_switch_data.old_lut_index;

    ex_lut = &lut_ex_table[new_lut_index];
    ex_lut->index_in_CE = lmp_role_switch_data.ce_index;

    AND_val_with_bb_reg_macro(CONNECTOR_REGISTER, (~BB_SET_TDD_BIT) );

    AND_val_with_bb_reg_macro(
        lut_ex_table[old_lut_index].upper_lut_address, (~ACTIVE_BIT));

    LC_HANDLE_MSS_WITH_AFH(lmp_role_switch_data.ce_index);

#ifdef _DAPE_TEST_CHG_KEY_WHEN_ROLE_SW
#ifdef _CCH_SC_TEST_20130129_MRV_01
    lmp_connection_entity[lmp_role_switch_data.ce_index].done_h3 = 0;

    RT_BT_LOG(YELLOW, DAPE_TEST_LOG293, 1,
   	lmp_connection_entity[lmp_role_switch_data.ce_index].done_h3);
#endif
#endif

#ifdef _DAPE_SC_TEST_CORRECT_DAYCOUNTER_FOR_UPF45
#ifdef _CCH_SC_TEST_20130201_ESCO_INI_FLAG
    {
        BZ_AUTH_LINK_PARAMS* auth;
        auth = lmp_connection_entity[lmp_role_switch_data.ce_index].auth;

        if(auth->secure_conn_enabled)
       	{
            auth->not_first_esco = 0;
        }
    }
#endif

#endif

    {
        BaseType_t high_pri_task_woken = pdFALSE;
        xTimerPendFunctionCallFromISR(
                BB_handle_mss_success_ito_interrupt_bottom_half, NULL, 0,
                &high_pri_task_woken);
        portYIELD_FROM_ISR(high_pri_task_woken);
    }
}
#endif


#ifdef _NEW_MODEM_PSD_SCAN_

void BB_handle_psd_end_intr(void)
{
#ifdef TEST_MODEM_PSD
{

    //===================================================================
    /* To move from Modem Record SRAM to PSD SRAM */
    //MODEM_REG_S_TYPE modem_reg;

    RT_BT_LOG(GRAY, MDM_LOG_016, 1, BB_read_native_clock());

    RT_BT_LOG(BLUE, YL_DBG_HEX_6, 6,
	     rtk_read_modem_radio_reg(TRANS_MODEM_REG(0x14), TYPE_MODEM),
	     rtk_read_modem_radio_reg(TRANS_MODEM_REG(0x3C), TYPE_MODEM),
//            RTK_READ_RF_REG(0x01),
            -0xffff,
            BB_read_baseband_register(BB_PSD_END_START),
            BB_read_baseband_register(BB_PSD_STEP_MODE),
            BB_read_baseband_register(BB_PSD_TIMEOUT));

    UINT8 ii;
    for (ii = 0; ii<=78; ii=ii+39)
    {
        RT_BT_LOG(WHITE, YL_DBG_HEX_11, 11,
                    ii,
                    patch_g_modem_psd_report_array[ii].psd_avg_dc,
                    patch_g_modem_psd_report_array[ii].psd_avg_pos,
                    patch_g_modem_psd_report_array[ii].psd_avg_neg,
                    (patch_g_modem_psd_report_array[ii].psd_avg_dfir_msb<<16) + patch_g_modem_psd_report_array[ii].psd_avg_dfir_lsb,
                    patch_g_modem_psd_report_array[ii].mp_gain_idx,
                    patch_g_modem_psd_report_array[ii].agc_ste,
                    patch_g_modem_psd_report_array[ii].lna_idx_use,
                    patch_g_modem_psd_report_array[ii].gain_idx_use,
                    patch_g_modem_psd_report_array[ii].psd_clip_ratio_start,
                    patch_g_modem_psd_report_array[ii].psd_clip_ratio_end);
    }

    for (ii = 0; ii<=78; )
    {
        RT_BT_LOG(BLUE, YL_DBG_DEC_10, 10,
            patch_g_modem_psd_report_array[ii].mp_gain_idx,
            patch_g_modem_psd_report_array[ii+1].mp_gain_idx,
            patch_g_modem_psd_report_array[ii+2].mp_gain_idx,
            patch_g_modem_psd_report_array[ii+3].mp_gain_idx,
            patch_g_modem_psd_report_array[ii+4].mp_gain_idx,
            patch_g_modem_psd_report_array[ii+5].mp_gain_idx,
            patch_g_modem_psd_report_array[ii+6].mp_gain_idx,
            patch_g_modem_psd_report_array[ii+7].mp_gain_idx,
            patch_g_modem_psd_report_array[ii+8].mp_gain_idx,
            patch_g_modem_psd_report_array[ii+9].mp_gain_idx);
        ii = ii + 10;
    }

    for (ii = 0; ii<=78; )
    {
        RT_BT_LOG(YELLOW, YL_DBG_DEC_10, 10,
            patch_g_modem_psd_report_array[ii].psd_avg_dc,
            patch_g_modem_psd_report_array[ii+1].psd_avg_dc,
            patch_g_modem_psd_report_array[ii+2].psd_avg_dc,
            patch_g_modem_psd_report_array[ii+3].psd_avg_dc,
            patch_g_modem_psd_report_array[ii+4].psd_avg_dc,
            patch_g_modem_psd_report_array[ii+5].psd_avg_dc,
            patch_g_modem_psd_report_array[ii+6].psd_avg_dc,
            patch_g_modem_psd_report_array[ii+7].psd_avg_dc,
            patch_g_modem_psd_report_array[ii+8].psd_avg_dc,
            patch_g_modem_psd_report_array[ii+9].psd_avg_dc);
        ii = ii + 10;
    }

    #if 0
    for (ii = 0; ii<=78; )
    {
        RT_BT_LOG(BLUE, YL_DBG_DEC_10, 10,
            patch_g_modem_psd_report_array[ii].psd_avg_pos,
            patch_g_modem_psd_report_array[ii+1].psd_avg_pos,
            patch_g_modem_psd_report_array[ii+2].psd_avg_pos,
            patch_g_modem_psd_report_array[ii+3].psd_avg_pos,
            patch_g_modem_psd_report_array[ii+4].psd_avg_pos,
            patch_g_modem_psd_report_array[ii+5].psd_avg_pos,
            patch_g_modem_psd_report_array[ii+6].psd_avg_pos,
            patch_g_modem_psd_report_array[ii+7].psd_avg_pos,
            patch_g_modem_psd_report_array[ii+8].psd_avg_pos,
            patch_g_modem_psd_report_array[ii+9].psd_avg_pos);
        ii = ii + 10;
    }


    for (ii = 0; ii<=78; )
    {
        RT_BT_LOG(YELLOW, YL_DBG_DEC_10, 10,
            patch_g_modem_psd_report_array[ii].psd_avg_neg,
            patch_g_modem_psd_report_array[ii+1].psd_avg_neg,
            patch_g_modem_psd_report_array[ii+2].psd_avg_neg,
            patch_g_modem_psd_report_array[ii+3].psd_avg_neg,
            patch_g_modem_psd_report_array[ii+4].psd_avg_neg,
            patch_g_modem_psd_report_array[ii+5].psd_avg_neg,
            patch_g_modem_psd_report_array[ii+6].psd_avg_neg,
            patch_g_modem_psd_report_array[ii+7].psd_avg_neg,
            patch_g_modem_psd_report_array[ii+8].psd_avg_neg,
            patch_g_modem_psd_report_array[ii+9].psd_avg_neg);
        ii = ii + 10;
    }
    #endif

    #if 1
    for (ii = 0; ii<=78; )
    {
        RT_BT_LOG(BLUE, YL_DBG_DEC_10, 10,
            (patch_g_modem_psd_report_array[ii].psd_avg_dfir_msb<<16) + patch_g_modem_psd_report_array[ii].psd_avg_dfir_lsb,
            (patch_g_modem_psd_report_array[ii+1].psd_avg_dfir_msb<<16) + patch_g_modem_psd_report_array[ii+1].psd_avg_dfir_lsb,
            (patch_g_modem_psd_report_array[ii+2].psd_avg_dfir_msb<<16) + patch_g_modem_psd_report_array[ii+2].psd_avg_dfir_lsb,
            (patch_g_modem_psd_report_array[ii+3].psd_avg_dfir_msb<<16) + patch_g_modem_psd_report_array[ii+3].psd_avg_dfir_lsb,
            (patch_g_modem_psd_report_array[ii+4].psd_avg_dfir_msb<<16) + patch_g_modem_psd_report_array[ii+4].psd_avg_dfir_lsb,
            (patch_g_modem_psd_report_array[ii+5].psd_avg_dfir_msb<<16) + patch_g_modem_psd_report_array[ii+5].psd_avg_dfir_lsb,
            (patch_g_modem_psd_report_array[ii+6].psd_avg_dfir_msb<<16) + patch_g_modem_psd_report_array[ii+6].psd_avg_dfir_lsb,
            (patch_g_modem_psd_report_array[ii+7].psd_avg_dfir_msb<<16) + patch_g_modem_psd_report_array[ii+7].psd_avg_dfir_lsb,
            (patch_g_modem_psd_report_array[ii+8].psd_avg_dfir_msb<<16) + patch_g_modem_psd_report_array[ii+8].psd_avg_dfir_lsb,
            (patch_g_modem_psd_report_array[ii+9].psd_avg_dfir_msb<<16) + patch_g_modem_psd_report_array[ii+9].psd_avg_dfir_lsb);
        ii = ii + 10;
    }
    #endif

    #if 1
    for (ii = 0; ii<=78; )
    {
        RT_BT_LOG(YELLOW, YL_DBG_DEC_10, 10,
            modem_psd_psdavg2dbm(patch_g_modem_psd_report_array[ii].psd_avg_dc, patch_g_modem_psd_report_array[ii].mp_gain_idx),
            modem_psd_psdavg2dbm(patch_g_modem_psd_report_array[ii+1].psd_avg_dc, patch_g_modem_psd_report_array[ii+1].mp_gain_idx),
            modem_psd_psdavg2dbm(patch_g_modem_psd_report_array[ii+2].psd_avg_dc, patch_g_modem_psd_report_array[ii+2].mp_gain_idx),
            modem_psd_psdavg2dbm(patch_g_modem_psd_report_array[ii+3].psd_avg_dc, patch_g_modem_psd_report_array[ii+3].mp_gain_idx),
            modem_psd_psdavg2dbm(patch_g_modem_psd_report_array[ii+4].psd_avg_dc, patch_g_modem_psd_report_array[ii+4].mp_gain_idx),
            modem_psd_psdavg2dbm(patch_g_modem_psd_report_array[ii+5].psd_avg_dc, patch_g_modem_psd_report_array[ii+5].mp_gain_idx),
            modem_psd_psdavg2dbm(patch_g_modem_psd_report_array[ii+6].psd_avg_dc, patch_g_modem_psd_report_array[ii+6].mp_gain_idx),
            modem_psd_psdavg2dbm(patch_g_modem_psd_report_array[ii+7].psd_avg_dc, patch_g_modem_psd_report_array[ii+7].mp_gain_idx),
            modem_psd_psdavg2dbm(patch_g_modem_psd_report_array[ii+8].psd_avg_dc, patch_g_modem_psd_report_array[ii+8].mp_gain_idx),
            modem_psd_psdavg2dbm(patch_g_modem_psd_report_array[ii+9].psd_avg_dc, patch_g_modem_psd_report_array[ii+9].mp_gain_idx));
        ii = ii + 10;
    }

    #endif



    //===================================================================

    return;
}

#else

    // TODO: (yl) To be completed
    // TODO:
    // TODO:
    /********************************************************************************
     * (yilinli)
     * 1. check if the (tdm) psd scan is finished on-time (or done in timer expiry?)
     *    if NOT, do some priority fine-tune or use the channel classification (?)
     * 2. record the bluewiz psd scan result
     * 3. if scan_tdm_state == RUN, set the scan_tdm_state = END
     * 4. ...
     ********************************************************************************/

#ifdef _ROM_CODE_PATCHED_
    if (rcp_BB_handle_psd_end_intr_func != NULL)
    {
        if ( rcp_BB_handle_psd_end_intr_func(NULL) )
        {
            return;
        }
    }
#endif

#ifdef _YL_TEST_NEW_MODEM_PSD_SCAN_FOR_AFH_
    {
        UINT8 ch;
        UINT8 rpt_ii = 0;
        modem_psd_scan_man.scan_tdm_state = MODEM_PSD_SCAN_TDM_STATE_END;
        // Alternative: can be moved to background TASK //
        for (ch = modem_psd_scan_man.scan_tdm_ch_start; ch <= modem_psd_scan_man.scan_tdm_ch_stop; ch+= modem_psd_scan_man.scan_tdm_ch_step)
        {

            if (g_efuse_modem_psd_setting_1.psd_scan_3ch_mode)
            {
                // TODO:
                modem_psd_result_man.psd_dbm_add200_signle[ch-1] = modem_psd_psdavg2dbm(g_modem_psd_report_array[rpt_ii].psd_avg_neg, g_modem_psd_report_array[rpt_ii].mp_gain_idx);
                modem_psd_result_man.psd_dbm_add200_signle[ch] = modem_psd_psdavg2dbm(g_modem_psd_report_array[rpt_ii].psd_avg_dc, g_modem_psd_report_array[rpt_ii].mp_gain_idx);
                modem_psd_result_man.psd_dbm_add200_signle[ch+1] = modem_psd_psdavg2dbm(g_modem_psd_report_array[rpt_ii].psd_avg_pos, g_modem_psd_report_array[rpt_ii].mp_gain_idx);
            }
            else
            {
                modem_psd_result_man.psd_dbm_add200_signle[ch] = modem_psd_psdavg2dbm(g_modem_psd_report_array[rpt_ii].psd_avg_dc, g_modem_psd_report_array[rpt_ii].mp_gain_idx);
            }

#ifdef _YL_TEST_NEW_MODEM_PSD_SCAN_FOR_AFH_LOG_
//            if ((modem_psd_scan_man.scan_tdm_cnt==1) && (modem_psd_scan_man.scan_period_cnt==1))
//            if (ch>=70 && ch<=78)
            {

//                RT_BT_LOG(YELLOW, YL_DBG_DEC_4, 4,
//                            ch,
//                            rpt_ii,
//                            g_modem_psd_report_array[rpt_ii].psd_avg_dc,
//                            modem_psd_result_man.psd_dbm_add200_signle[ch]);
            }
#endif
            rpt_ii++;
        }
    }
#endif
#endif

}
#endif

SECTION_ISR_LOW void BB_handle_broadcast_nbc_ito_interrupt(UINT8 phy_piconet_id,
                                                      UINT32 cur_clk)
{
    UINT8 pkt_type;
    LC_PICONET_SCHEDULER *pico_sched;
    pico_sched =  &lc_piconet_scheduler[phy_piconet_id];

    pkt_type = BB_read_baseband_register(PICONET_LOOK_UP_TABLE_BASE_ADDR) >> 12;

    if ((1 << pkt_type) & BB_RX_ISR_DM_DH_PKT)
    {
        UINT32 lcl_clk;
        LC_SCHEDULED_PKT *schd;
        OS_SIGNAL sig_send;

#ifdef ENABLE_LOGGER_LEVEL_2
        RT_BT_LOG(GRAY, LC_ISR_3573, 1, pkt_type);
        RT_BT_LOG(GRAY, LC_ISR_3574, 0, 0);
#endif

        BB_write_baseband_register(PICONET_LOOK_UP_TABLE_BASE_ADDR,
                                   LC_SLAVE_DEFAULT_PACKET_TYPE);
#ifdef _SUPPORT_VER_4_1_
#ifdef _SUPPORT_PCA_ADJUST
        /* this broadcast pkt is clk_request pdu. */
        if (IS_BT41 && (bt_pca_manager.pca_clk_req_sent) && (pkt_type == BB_DM1))
        {
            bt_pca_manager.pca_clk_req_nbc_to = 1;
            RT_BT_LOG(YELLOW, DAPE_TEST_LOG575, 3,
             cur_clk, phy_piconet_id,
             BB_read_baseband_register(RE_TRANSMISSION_COUNT_REGISTER) & 0xFF);

        }
        else
#endif
#endif
        {
        sig_send.type = LC_HANDLE_ACK_RECD_SIGNAL;
        sig_send.param = (void *)((UINT32)pico_sched->rptr);;
        lcl_clk = (cur_clk & 0x0fffffff) | (phy_piconet_id << 30);

        sig_send.ext_param = (void *)lcl_clk;

        schd = &pico_sched->lc_scheduled_pkt_info[pico_sched->rptr];

        /* Reset status of previously sent packet */
        schd->tx_status = LC_TX_IDLE;

        pico_sched->rptr++;

        if ( OS_SEND_SIGNAL_TO_TASK ( lc_tx_task_handle, sig_send )
                != BT_ERROR_OK )
        {
#ifdef ENABLE_LOGGER_LEVEL_2
            LC_LOG_ERROR(LOG_LEVEL_LOW,
                         "Error sending signal to lmp_task");
#endif
        }

        RT_BT_LOG(YELLOW, ITO_MSG_NBC_TIMEOUT, 6,
             cur_clk,
             BB_read_baseband_register(RE_TRANSMISSION_COUNT_REGISTER) & 0xFF,
             schd->rx_count, schd->flow_ctrl_count,
             schd->fw_sent_time, schd->fw_sent_time_us);
        }

    } /* end of if ( (pkt_type == BB_DM1) || (pkt_type ... ) */
}

#ifdef COMPILE_HOLD_MODE
SECTION_ISR_LOW void BB_handle_hold_mode_ito_interrupt(UINT8 start)
{
    UCHAR i;
    UINT16 lcl_upper_lut;
    UINT16 ce_index;
    OS_SIGNAL sig_send;

    for (i = 0; i < LC_MAX_NUM_OF_LUT_EX_TABLES; i++)
    {
        ce_index = lut_ex_table[i].index_in_CE;

        if (ce_index != INVALID_CE_INDEX)
        {
            lcl_upper_lut = BB_read_baseband_register(
                                lut_ex_table[i].upper_lut_address);

            if (lcl_upper_lut & LC_UPPER_LUT_HOLD_MODE)
            {
                if (start)
                {
                    sig_send.type = LMP_HOLD_INSTANT_SIGNAL;
                }
                else
                {
                    sig_send.type = LMP_COMPLETE_HOLD_MODE_SIGNAL;

                    /* Enabling retx window of esco links on this ACL link */
                    LC_RESTORE_ESCO_RETX(ce_index);
                }

                sig_send.param = (OS_ADDRESS)((UINT32)ce_index);
                OS_SEND_SIGNAL_TO_TASK(lmp_task_handle, sig_send);

                /* This Hold-bit is not clear-on-read.
                   LC has to explicitly clear. */
                AND_val_with_bb_reg_macro(lut_ex_table[i].upper_lut_address,
                                            (UINT16) (~LC_UPPER_LUT_HOLD_MODE));
            }
        }
    }
}
#endif /* end of #ifdef COMPILE_HOLD_MODE */

/**
 * Handles timeout interrupt from the baseband.
 *
 * \param None.
 *
 * \return None.
 */
SECTION_ISR_LOW void BB_handle_ito_interrupt(void)
{
    UCHAR am_addr = 0xFF;
    UINT16 timeout_status;
    OS_SIGNAL sig_send;
    UCHAR phy_piconet_id;
    UINT32 cur_clk;

    timeout_status = BB_read_baseband_register(TIMEOUT_STATUS_REGISTER);
    phy_piconet_id = (UCHAR) (timeout_status >> 14);

    lc_get_clock_in_scatternet(&cur_clk, phy_piconet_id);

#ifdef _CSB_RX_DBG_LOG
    if (g_csb_rx_dbg_log)
    {
RT_BT_LOG(WHITE, DAPE_TEST_LOG207, 1,timeout_status);
    }
#endif

#if defined(_NEW_BZDMA_FROM_V7_) && (defined(BZDMA_USE_NEW_SCO_TXCMD) || defined(_USE_ONE_NEW_BZDMA_TXCMD_FOR_SCO_))
    /* this function is existed from rtl8821a */
    if (timeout_status & 0x1C00)
    {
        lc_handle_sco_txcmd_ito_interrupt(timeout_status);
    }
#endif

#ifdef COMPILE_PARK_MODE
    if (timeout_status & BB_ITO_PARK_NCTO)
    {
        BB_handle_park_new_conn_to_in_ito_interrupt();
    }
#endif

    if (timeout_status & BB_ITO_INQUIRY_TIMEOUT)
    {
        sig_send.type = LMP_HANDLE_INQUIRY_TIMEOUT_SIGNAL;
        sig_send.param = (void *)NULL;
        OS_ISR_SEND_SIGNAL_TO_TASK (lmp_task_handle, sig_send );
        RT_BT_LOG(YELLOW, ITO_MSG_INQUIRY_TIMEOUT, 2,
            cur_clk, BB_read_baseband_register(INQUIRY_TIMEOUT_REGISTER)*2);
    }

    if (timeout_status & BB_ITO_PAGE_TIMEOUT)
    {
        if ((lmp_self_device_data.lc_cur_dev_state == LC_CUR_STATE_PAGE) &&
            (lc_cur_connecting_am_addr != INVALID_AM_ADDR))
        {
            lc_set_lc_cur_device_state(LC_CUR_STATE_IDLE);

            am_addr = lc_cur_connecting_am_addr;

            lc_set_lc_cur_connecting_am_addr(INVALID_AM_ADDR);

            sig_send.type = LMP_HANDLE_PAGE_TIMEOUT_SIGNAL;
            sig_send.param = (void *)((UINT32)am_addr);
            sig_send.ext_param = (void *)((UINT32)(phy_piconet_id));

            OS_SEND_SIGNAL_TO_TASK (lmp_task_handle, sig_send);

            RT_BT_LOG(YELLOW, ITO_MSG_PAGE_TIMEOUT, 4,
                        cur_clk,
                        BB_read_baseband_register(PAGE_TIMEOUT_REGISTER)<<1,
                        am_addr, phy_piconet_id);
        }
    }

#ifdef COMPILE_ROLE_SWITCH
    if (timeout_status & BB_ITO_MSS_SUCCESS)
    {
        BB_handle_mss_success_ito_interrupt();
    }
#endif

    if (timeout_status & BB_ITO_NC_TIMEOUT)
    {
#ifdef COMPILE_ROLE_SWITCH
        /* For MSS. */
        if (lmp_mss_state != LMP_MSS_INIT)
        {
            am_addr = lmp_role_switch_data.old_am_addr;
            lmp_handle_switch_newconn_timeout_in_scatternet();
        }
        else /* For page-scan. */
#endif /* COMPILE_ROLE_SWITCH */
        {
            if ((lmp_self_device_data.lc_cur_dev_state != LC_CUR_STATE_PAGE) &&
                (lc_cur_connecting_am_addr != INVALID_AM_ADDR))
            {
                am_addr = lc_cur_connecting_am_addr;

                sig_send.type = LMP_HANDLE_PAGESCAN_NCTO_SIGNAL;
                sig_send.param = (void *)((UINT32)am_addr);
                sig_send.ext_param = (void *)((UINT32)phy_piconet_id);

                OS_SEND_SIGNAL_TO_TASK(lmp_task_handle, sig_send);

                lc_set_lc_cur_connecting_am_addr(INVALID_AM_ADDR);
            }
        }

        RT_BT_LOG(YELLOW, LC_ISR_3455, 3, am_addr, phy_piconet_id, cur_clk);
    } /* if (timeout_status & BB_ITO_NC_TIMEOUT) */

    if (timeout_status & BB_ITO_NBC_TIMEOUT)
    {
        LC_PICONET_SCHEDULER *piconet_schd;
        UINT8 rptr;

        piconet_schd = &lc_piconet_scheduler[phy_piconet_id];

        lc_waiting_for_crc_pkt_ack[phy_piconet_id] = FALSE;

        rptr = piconet_schd->rptr;

        am_addr = piconet_schd->lc_scheduled_pkt_info[rptr].selected_am_addr;

        if (piconet_schd->lc_scheduled_pkt_info[rptr].tx_status == LC_TX_SENT)
        {
            if (am_addr != BC_AM_ADDR)
            {
                BB_handle_unicast_nbc_ito_interrupt(phy_piconet_id, cur_clk);
            }
            else
            {
                BB_handle_broadcast_nbc_ito_interrupt(phy_piconet_id, cur_clk);
            }
        }
#ifdef _SUPPORT_VER_4_1_
#ifdef _SUPPORT_PCA_ADJUST
        else
        {
            /* this broadcast pkt is clk_request pdu. */
            if (IS_BT41 && bt_pca_manager.pca_clk_req_sent)
            {
                BB_handle_broadcast_nbc_ito_interrupt(phy_piconet_id, cur_clk);
            }
        }
#endif
#endif

    } /* end of if (timeout_status & BB_ITO_NBC_TIMEOUT) */

#ifdef COMPILE_HOLD_MODE
    if (timeout_status & (BB_ITO_HOLD_START_TIMEOUT | BB_ITO_HOLD_END_TIMEOUT))
    {
        UINT8 start;
        start = (timeout_status & BB_ITO_HOLD_START_TIMEOUT) ? TRUE : FALSE;
        BB_handle_hold_mode_ito_interrupt(start);
    }
#endif /* COMPILE_HOLD_MODE */

    if (timeout_status & BB_PAGERESP_TIMEOUT)
    {
#ifdef VER_CSA4
#ifdef _SUPPORT_CSB_TRANSMITTER_
        if (IS_BT_CSA4)
        {
            hci_generate_csa4_slave_page_response_timeout_event();
        }
#endif
#endif

        RT_BT_LOG(YELLOW, ITO_MSG_PAGE_RESPONSE_TIMEOUT, 2,
                                            cur_clk, phy_piconet_id);
    }

    if (timeout_status & BB_ITO_UNPARK_REQ)
    {
        RT_BT_LOG(YELLOW, ITO_MSG_UNPARK_REQ, 0, 0);
    }

    return;
}

/**
 * Handles imode interrupt from the baseband.
 *
 * \param None.
 *
 * \return None.
 */
SECTION_ISR void BB_handle_imode_interrupt(void)
{
    UINT16 mode_sts_reg;
    UCHAR phy_piconet_id;

#ifdef ENABLE_SCO
    UCHAR sco_num = 0xFF;
#endif
    mode_sts_reg = BB_read_baseband_register(MODE_STATUS_REGISTER);

    phy_piconet_id = ((mode_sts_reg >> 15) << 1) |
                     ((mode_sts_reg >> 13) & 0x01);

#ifdef _CSB_RX_DBG_LOG
    if (g_csb_rx_dbg_log)
    {
            RT_BT_LOG(YELLOW, DAPE_TEST_LOG213, 2,
                    (mode_sts_reg & BB_IMODE_TRUNCATED_PAGE_DONE),
                    mode_sts_reg);
    }
#endif

#ifdef _SUPPORT_PCA_ADJUST
    if (mode_sts_reg & BB_IMODE_CLK_ADJ_INSTANT)
    {
        UINT32 clk;
        UINT16 clk_us;
        lc_get_high_dpi_clock_in_scatternet(&clk,
            &clk_us, phy_piconet_id);

        /* this is used for all us shown in clk_adj is shown in a count-down format. */
        if (624 >= clk_us)
        {
            clk_us = 624 - clk_us;
        }

        RT_BT_LOG(BLUE, DAPE_TEST_LOG566, 6, clk,
            phy_piconet_id, clk_us,
            bt_pca_manager.instant,
            bt_pca_manager.clk_adj_slots,
            bt_pca_manager.clk_adj_us);

        bt_pca_manager.pca_updating = 0;

#ifdef _DAPE_TEST_FIX_PCA_NO_TX
        lc_stop_tpoll_on_all_connections_except(16);
        lc_start_tpoll_on_all_connections();
#endif

        if (bt_pca_manager.pca_clk_req_sent == 1)
        {
            /* instant passed. change clk_adj_mode to 1. */
            bzdma_tx_buf[phy_piconet_id][10] = 1;
            /* We don't have to re-send pdu here since tx interrupt will still
               happen and fw will re-send pdu there. */
        }
    }
#endif

#ifdef _SUPPORT_CSB_RECEIVER_
    if (mode_sts_reg & BB_IMODE_TRUNCATED_PAGE_DONE)
    {
        UINT16 ce_index;
#ifdef _CSB_RX_DBG_LOG
        if (g_csb_rx_dbg_log)
        {
            RT_BT_LOG(BLUE, DAPE_TEST_LOG522, 2,
                        lc_cur_connecting_am_addr,
                        phy_piconet_id);
        }
#endif
        if (LMP_GET_CE_INDEX_FROM_AM_ADDR_PPI(lc_cur_connecting_am_addr,
                phy_piconet_id, &ce_index)
                != API_SUCCESS)
        {
            VER_ERR(PAGE_TIMEOUT_EXTRACTION_OF_CE_INDEX_FAILED,2,
                           lc_cur_connecting_am_addr, phy_piconet_id);
            return;
        }

        {
            OS_SIGNAL sig_send;
            UCHAR am_addr = lc_cur_connecting_am_addr;
            lc_set_lc_cur_device_state(LC_CUR_STATE_IDLE);

            lc_set_lc_cur_connecting_am_addr(INVALID_AM_ADDR);

            sig_send.type = LMP_HANDLE_PAGE_TIMEOUT_SIGNAL;
            sig_send.param = (void *)((UINT32)am_addr);
            sig_send.ext_param = (void *)((UINT32)(phy_piconet_id));

            OS_SEND_SIGNAL_TO_TASK (lmp_task_handle, sig_send);

            UINT32 cur_clk = BB_read_native_clock();
            RT_BT_LOG(GRAY, MSG_CSB_TRUNCATED_PAGE_RX_ID, 1,
                        cur_clk);
        }
        lmp_set_ce_status(ce_index, LMP_TRUNCATED_PAGE_COPMLETE);

        hci_generate_csa4_truncated_page_complete_event(HCI_COMMAND_SUCCEEDED, ce_index);

    }
#endif

#ifdef ENABLE_SCO
    if (mode_sts_reg & 0x0080)
    {
        /* Start of SCO 1 Slot Interrupt */
        sco_num = 0;
    }
    else if (mode_sts_reg & 0x0100)
    {
        /* Start of SCO 2 Slot Interrupt */
        sco_num = 1;
    }
    else if (mode_sts_reg & 0x0200)
    {
        /* Start of SCO 3 Slot interrupt */
        sco_num = 2;
    }

    if (sco_num < 0xFF)
    {
#ifdef SCO_OVER_HCI
        /* it is an early interrupt then fw can send host voice data to sco tx
            fifo now */
        lc_handle_sco_instant_interrupt(sco_num);

#ifdef _IMPROVE_PCM_MUTE_CTRL_TIMING_
	//if (otp_str_data.hci_excodec_state)
        if(hci_excodec_state)
        {
            lc_pcm_mute_control();
        }
#endif
#endif /* SCO_OVER_HCI */
    }
#endif /* ENABLE_SCO */

#ifdef COMPILE_PARK_MODE
    BB_handle_park_imode_interrupts(mode_sts_reg, phy_piconet_id);
#endif /* COMPILE_PARK_MODE */

    if (mode_sts_reg & BB_RANDOM_START_TIMEOUT)
    {
#ifdef VER_1_1_SPECIFIC
        lc_handle_random_start_intr();
#endif
    }

#ifdef _NEW_MODEM_PSD_SCAN_
    if (mode_sts_reg & BB_IMODE_PSD_END)
    {
        BB_handle_psd_end_intr();
    }
#endif

    return ;
}

#ifdef TEST_MODE
SECTION_ISR void BB_handle_test_mode_in_tx_interrupt(UINT16 lc_tx_status,
                                         LUT_EXTENSION_TABLE *ex_lut)
{
    UINT16 lut_header;
    UINT8 am_addr;
    UINT8 tx_pkt_type;
    BZ_REG_S_TX_STATUS *reg_tx_status;

    reg_tx_status = (BZ_REG_S_TX_STATUS *)&lc_tx_status;
    am_addr = reg_tx_status->lt_addr;
    tx_pkt_type = reg_tx_status->pkt_type;

    if (lc_is_tx_test_mode != TRUE)
    {
        /* If the transmitted packet is a loopback packet, flush this packet. */
        /* Check if the pkt transmitted is a PDU.
           For PDU, we do need to retransmit. */

        BB_write_baseband_register(0x60, 0x0);

        lut_header = BB_read_baseband_register(ex_lut->lower_lut_address);
        if (((lut_header >> 10) & (0x03) ) != L_CH_LMP)
        {
            BB_write_baseband_register(ex_lut->lower_lut_address,
                                       LC_SLAVE_DEFAULT_PACKET_TYPE);

            test_mode_sched_acl_pkt = FALSE;

#ifndef _RTL8723B_DUT_MODE_

#ifdef _MT8852B_DBG_
            UINT8 *buf_ptr;
            buf_ptr = NON_DCATCH_PTR(
                          Bzdma_Manager.TxEntSta[2].pTxDesc[0].start_addr);
#endif
            bzdma_invalid_txcmd(BZDMA_TX_ENTRY_TYPE_PICONET0, am_addr, 0);

#ifdef _MT8852B_DBG_
#if 1
            if (test_mode_tx_int_cnt < BT0380_DBG_ALLOW_MAX_COUNT)
            {
                MT8852B_DBG(GRAY,MT8852B_MSG_TX_LB_TEST, 8,
                            test_mode_tx_int_cnt, lc_tx_status, lut_header,
                            BB_read_baseband_register(ex_lut->upper_lut_address),
                            buf_ptr[0], buf_ptr[1], buf_ptr[2], buf_ptr[3]);
                test_mode_tx_int_cnt++;
            }
#else
            if ((tx_pkt_type == BB_NULL) && (lut_header != 0))
            {
                MT8852B_DBG(RED, MT8852B_MSG_TX_LAG_LB_TEST, 4,
                            test_mode_tx_int_cnt, lc_tx_status, lut_header,
                            BB_read_baseband_register(ex_lut->upper_lut_address));
                test_mode_tx_int_cnt++;
            }
#endif
#endif
#endif /* end of #ifndef _RTL8723B_DUT_MODE_ */
        }
    }
    else if ((lmp_self_device_data.test_mode == HCI_DEVICE_UNDER_TEST_MODE) &&
             (lc_is_tx_test_mode == TRUE))
    {
        /* If the transmitted packet is a Tx test packet, flush this packet. */

        UINT8 state = 0;
        BZ_REG_S_LUT_LOWER *p_lut_low;

        BB_write_baseband_register(0x60, 0x0);

        lut_header = BB_read_baseband_register(ex_lut->lower_lut_address);
        p_lut_low = (BZ_REG_S_LUT_LOWER *)&lut_header;

        if (p_lut_low->llid != L_CH_LMP)
        {
            /* Not LMP PDU */

            BB_write_baseband_register(ex_lut->lower_lut_address,
                                       LC_SLAVE_DEFAULT_PACKET_TYPE);

#ifndef _RTL8723B_DUT_MODE_
            bzdma_invalid_txcmd(BZDMA_TX_ENTRY_TYPE_PICONET0, am_addr, 0);
#endif

            test_mode_sched_acl_pkt = FALSE;
            test_mode_tx_mode_buf_in_used = FALSE;

            state |= BIT(0);
        }
        else
        {
            /* LMP PDU */

            state |= BIT(2);

            if (test_mode_lmp_pdu != NULL)
            {
                state |= BIT(3);

                if (test_mode_tx_mode_force_ack == FALSE)
                {
                    if (tx_pkt_type == BB_DM1)
                    {
#ifdef _RTL8723B_DUT_MODE_
                        /* re-enable tx mode */
                        BB_write_baseband_register(RADIO_SELECT_REGISTER,
                                                  test_mode_tx_mode_reg_catch);
#endif

                        OS_FREE_BUFFER(lmp_pdu_buffer_pool_handle,
                                       (void*)test_mode_lmp_pdu);

                        test_mode_tx_mode_force_ack = TRUE;
                        test_mode_lmp_pdu = NULL;

                        BB_write_baseband_register(ex_lut->lower_lut_address,
                                                   LC_SLAVE_DEFAULT_PACKET_TYPE);

                        bzdma_invalid_txcmd(BZDMA_TX_ENTRY_TYPE_PICONET0, am_addr, 0);

                        test_mode_sched_acl_pkt = FALSE;
                        test_mode_tx_mode_buf_in_used = FALSE;
                        state |= BIT(4);
                    }
                    state |= BIT(5);
                }
            }
        }

#ifdef _MT8852B_DBG_
#ifndef _RTL8723B_DUT_MODE_
        UINT8 *buf_ptr;
        buf_ptr = NON_DCATCH_PTR(
                      Bzdma_Manager.TxEntSta[2].pTxDesc[0].start_addr);
        if (test_mode_tx_int_cnt < BT0380_DBG_ALLOW_MAX_COUNT)
        {
            MT8852B_DBG(GRAY,MT8852B_MSG_TX_TX_TEST, 8,
                        test_mode_tx_int_cnt, lc_tx_status, lut_header,
                        BB_read_baseband_register(ex_lut->upper_lut_address),
                        buf_ptr[0], buf_ptr[1], buf_ptr[2], state);
            test_mode_tx_int_cnt++;
        }
#endif

        if ((tx_pkt_type == BB_NULL) && ((lut_header & 0xF3FF) != 0))
        {
            MT8852B_DBG(RED, MT8852B_MSG_TX_LAG_TX_TEST, 4,
                        test_mode_tx_int_cnt, lc_tx_status, lut_header,
                        BB_read_baseband_register(ex_lut->upper_lut_address));
            test_mode_tx_int_cnt++;
        }
#endif
    }
}
#endif

#ifdef _INQRES_DBG_CCH_
void BB_handle_tx_int_debug_inqres(void)
{
    UINT16 channel_tx;
    UINT16 channel_rx;
    UINT32 native_clk;
    UINT8  channel_unpair;

    UCHAR modem_addr;
    UINT32 modem_reg;
    UINT16 RPT_AGC_idx;
    UINT16 RPT_gain_idx;
    UINT16 bz_gain_idx;

    channel_unpair = 0;

    native_clk = BB_read_native_clock();
    channel_tx = (BB_read_baseband_register(CHANNEL_REGISTER) & 0x00FF);
    channel_rx = ( (BB_read_baseband_register(CHANNEL_REGISTER) & 0xFF00)>>8 );

    modem_addr = 0x35;
#ifdef _NEW_MODEM_PI_ACCESS_    // to avoid run-time access page-conflicts //
    modem_reg = RTK_READ_MODEM_REG_PI(MODEM_PI_PAGE_0, modem_addr);
#else
    modem_reg = rtk_read_modem_radio_reg(modem_addr, TYPE_MODEM);
#endif
    RPT_gain_idx = (modem_reg & 0x7E00)>> 9;

    modem_addr = 0x38;
#ifdef _NEW_MODEM_PI_ACCESS_    // to avoid run-time access page-conflicts //
    modem_reg = RTK_READ_MODEM_REG_PI(MODEM_PI_PAGE_0, modem_addr);
#else
    modem_reg = rtk_read_modem_radio_reg(modem_addr, TYPE_MODEM);
#endif
    RPT_AGC_idx = (modem_reg & 0xFC00)>> 10;

#ifdef _YL_MODEM_RSSI_MAPPING
    bz_gain_idx = (BB_read_baseband_register(RADIO_SELECT_REGISTER) );
//    bz_gain_idx = lc_modem_rssi_mapping(bz_gain_idx);
#else
    bz_gain_idx = (BB_read_baseband_register(0xD8) );
#endif

    if (afh_is_tx[channel_rx] != channel_tx)
    {
        channel_unpair = 1;
    }

    RT_BT_LOG(RED, CCH_DBG_040, 7,channel_rx,channel_tx,native_clk,
                channel_unpair,RPT_gain_idx,RPT_AGC_idx, bz_gain_idx);

    INT16 RPT_CFOE_sync =0;
    INT16 RPT_CFOE_trailer =0;

#ifdef _NEW_MODEM_PI_ACCESS_BY_VENDOR_REG_
    modem_reg = RTK_READ_MODEM_REG_PI(0, TRANS_MODEM_REG(0x6A));
#else
    modem_reg = lc_read_rf_rx_backup_status_rpt_reg(1); //0x6a
#endif
    RPT_CFOE_sync = (modem_reg & 0x01FF);
    if (RPT_CFOE_sync>256)
        RPT_CFOE_sync = RPT_CFOE_sync- 512;

#ifdef _NEW_MODEM_PI_ACCESS_BY_VENDOR_REG_
    modem_reg = RTK_READ_MODEM_REG_PI(0, TRANS_MODEM_REG(0x6C));
#else
    modem_reg = lc_read_rf_rx_backup_status_rpt_reg(2); //0x6c
#endif
    RPT_CFOE_trailer = (modem_reg & 0x01FF);
    if (RPT_CFOE_trailer>256)
        RPT_CFOE_trailer = RPT_CFOE_trailer - 512;

    RT_BT_LOG(BLUE, CCH_DBG_049, 2,RPT_CFOE_sync,RPT_CFOE_trailer);
}
#endif


void bt_csb_control_supervision_timer_task(void *no_arg1, uint32_t enabled)
{
    bt_csb_control_supervision_timer((UINT8) enabled);
}

/**
 * Handles packet-transmission completion interrupt from the baseband.
 *
 * \param None.
 *
 * \return None.
 */
SECTION_ISR void BB_handle_tx_interrupt(void)
{
    UINT16 ce_index;
    UCHAR am_addr;
    UCHAR tx_pkt_type;
    UCHAR phy_piconet_id;
    UCHAR lcl_lc_waiting_for_crc_pkt_ack = 0xff;
    UCHAR lut_index;
    LMP_CONNECTION_ENTITY *ce_ptr;
    UINT32 lcl_tx_pkt_type;
    UCHAR send_pkt_flag = FALSE;
    LC_SCHEDULED_PKT *schd;
    UINT16 lc_tx_status;
    BZ_REG_S_TX_STATUS *reg_tx_status;

    /* Get the am address for which the packet was transmitted */
    lc_tx_status = BB_read_baseband_register(TRANSMIT_STATUS_REGISTER);
    reg_tx_status = (BZ_REG_S_TX_STATUS *)&lc_tx_status;
    tx_pkt_type = reg_tx_status->pkt_type;
#ifdef _TEST_ADAPTIVITY_FUNC_2
    if (g_enable_pi_read_after_tx)
    {
        UINT32 lbt_dbg_signal;
        lbt_dbg_signal = RTK_READ_MODEM_REG_PI(MODEM_PI_PAGE_0, TRANS_MODEM_REG(0x7E));
        UINT8 rf_mode_lbt = (lbt_dbg_signal >> 11) & 0xF;
        UINT8 rif_tx_gain_lbt = (lbt_dbg_signal >> 3);

        UINT32 reg5C = RTK_READ_MODEM_REG_PI(MODEM_PI_PAGE_3, TRANS_MODEM_REG(0x5C));
        UINT8 reg_tx_gain_set_manu_en = (reg5C >> 6) & BIT0;
        UINT8 reg_tx_gain_set_manu_val_7_0 = (reg5C >> 7) & 0xFF;


        RT_BT_LOG(WHITE, YL_DBG_HEX_8, 8,
        BB_read_native_clock(), BB_read_baseband_register(0xC4),
        reg5C, reg_tx_gain_set_manu_en, reg_tx_gain_set_manu_val_7_0,
        lbt_dbg_signal, rf_mode_lbt, rif_tx_gain_lbt);
    }

#endif

#ifdef _YL_LPS_1
    lps_gpio_one_pull_high(LPS_GPIO_TX);
    LPS_DBG_LOG(GREEN, LPS_LOG_023, 1, lc_tx_status);
    lps_gpio_one_pull_low(LPS_GPIO_TX);
#endif

#ifdef COMPILE_CHANNEL_ASSESSMENT
    lmp_last_tx_pkt_type = tx_pkt_type;
#endif /* COMPILE_CHANNEL_ASSESSMENT */

    phy_piconet_id = (reg_tx_status->pid_h << 1) | reg_tx_status->pid_l;

    am_addr = reg_tx_status->lt_addr;

    LC_PICONET_SCHEDULER *piconet_schd;
    piconet_schd = &lc_piconet_scheduler[phy_piconet_id];
    schd = &piconet_schd->lc_scheduled_pkt_info[piconet_schd->rptr];

    lut_index = lc_get_lut_index_from_phy_piconet_id(am_addr, phy_piconet_id);

#ifdef COMPILE_CHANNEL_ASSESSMENT
    lmp_last_tx_conn_entry = lut_ex_table[lut_index].index_in_CE;
#endif

#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8723_RCP_

    if (rcp_BB_handle_tx_interrupt_lps_reset != NULL)
    {
        if (rcp_BB_handle_tx_interrupt_lps_reset((void *)&lc_tx_status))
        {
            return;
        }
    }
#endif
#endif

#ifdef VER_CSA4
    if (IS_BT_CSA4)
    {
#ifdef _DAPE_TEST_CHK_CSB_SYNC_TX
#if 0
    UINT8 i;

    UINT8 cam_offset = CAM_ADDR_OFFSET_LAYER2 + lut_index*CAM_ADDR_OFFSET_LAYER2_SIZE + CAM_ADDR_OFFSET_AFH;
    UINT32 afh_map_cam[3];

    for (i = 0; i < 3; i ++)
    {
        afh_map_cam[i] = BB_read_sc_cam( (cam_offset + i));
    }
LMP_GET_CE_INDEX_FROM_AM_ADDR_PPI(am_addr, phy_piconet_id,
                                         &ce_index);
//RT_BT_LOG(WHITE, DAPE_TEST_LOG525, 6, ce_index, lut_index,
//	cam_offset, afh_map_cam[0], afh_map_cam[1], afh_map_cam[2]);

    LMP_CONNECTION_ENTITY *ce_ptr;
    ce_ptr = &lmp_connection_entity[ce_index];
if ((ce_ptr->afh_mode == AFH_ENABLE) && (ce_index == 0))
{
if ((g_afh_map[ce_index][6]!= (afh_map_cam[0]& 0xFF)) ||
   (g_afh_map[ce_index][7]!= ((afh_map_cam[0]>>8)& 0xFF)) ||
   (g_afh_map[ce_index][8]!= ((afh_map_cam[0]>>16)& 0xFF)) ||
   (g_afh_map[ce_index][9]!= ((afh_map_cam[0]>>24)& 0xFF)) ||
   (g_afh_map[ce_index][2]!= ((afh_map_cam[1]>>0)& 0xFF)) ||
   (g_afh_map[ce_index][3]!= ((afh_map_cam[1]>>8)& 0xFF)) ||
   (g_afh_map[ce_index][4]!= ((afh_map_cam[1]>>16)& 0xFF)) ||
   (g_afh_map[ce_index][5]!= ((afh_map_cam[1]>>24)& 0xFF)) ||
   (g_afh_map[ce_index][0]!= ((afh_map_cam[2]>>0)& 0xFF)) ||
   (g_afh_map[ce_index][1]!= ((afh_map_cam[2]>>8)& 0xFF)) )


{


RT_BT_LOG(RED, YL_DBG_HEX_20, 20,
BB_read_native_clock(),
lc_tx_status,ce_index,lut_index,
	phy_piconet_id, am_addr,
BB_read_baseband_register(AFH_CHANNEL_MAP_EN_REG),
afh_map_cam[0],afh_map_cam[1],afh_map_cam[2],
g_afh_map[ce_index][0], g_afh_map[ce_index][1], g_afh_map[ce_index][2],
g_afh_map[ce_index][3], g_afh_map[ce_index][4], g_afh_map[ce_index][5],
g_afh_map[ce_index][6], g_afh_map[ce_index][7], g_afh_map[ce_index][8],
g_afh_map[ce_index][9]
);
}
else
{

RT_BT_LOG(BLUE, YL_DBG_HEX_20, 20,
BB_read_native_clock(),
lc_tx_status,ce_index,lut_index,
	phy_piconet_id, am_addr,
BB_read_baseband_register(AFH_CHANNEL_MAP_EN_REG),
afh_map_cam[0],afh_map_cam[1],afh_map_cam[2],
g_afh_map[ce_index][0], g_afh_map[ce_index][1], g_afh_map[ce_index][2],
g_afh_map[ce_index][3], g_afh_map[ce_index][4], g_afh_map[ce_index][5],
g_afh_map[ce_index][6], g_afh_map[ce_index][7], g_afh_map[ce_index][8],
g_afh_map[ce_index][9]
);
}

g_afh_map[ce_index][6]= (afh_map_cam[0]& 0xFF);
g_afh_map[ce_index][7]= ((afh_map_cam[0]>>8)& 0xFF);
g_afh_map[ce_index][8]= ((afh_map_cam[0]>>16)& 0xFF);
g_afh_map[ce_index][9]= ((afh_map_cam[0]>>24)& 0xFF);
g_afh_map[ce_index][2]= ((afh_map_cam[1]>>0)& 0xFF);
g_afh_map[ce_index][3]= ((afh_map_cam[1]>>8)& 0xFF);
g_afh_map[ce_index][4]= ((afh_map_cam[1]>>16)& 0xFF);
g_afh_map[ce_index][5]= ((afh_map_cam[1]>>24)& 0xFF);
g_afh_map[ce_index][0]= ((afh_map_cam[2]>>0)& 0xFF);
g_afh_map[ce_index][1]= ((afh_map_cam[2]>>8)& 0xFF);






}
#endif
#endif
#ifdef _SUPPORT_CSB_TRANSMITTER_
        BT_CSA4_BEACON_TX_UINT_S *pcsb = &bt_3dd_var.csb_tx_param;

        if (pcsb->enable)
        {
#ifdef _NEW_3DD_HW_
            if (reg_tx_status->Is3ddBea)
#else
            if ((pcsb->is_reserved_lt_addr &&
                (pcsb->lt_addr == am_addr)) &&
                (pcsb->select_tx_packet_type == tx_pkt_type))
#endif
            {
                BaseType_t high_pri_task_woken = pdFALSE;
                /* restart CSB supervision timer */
                xTimerPendFunctionCallFromISR(
                        bt_csb_control_supervision_timer_task, NULL, TRUE,
                        &high_pri_task_woken);
                portYIELD_FROM_ISR(high_pri_task_woken);
                return;
            }
        }
#endif
    }
#endif

    /* release bzdma tx entry when AUX1 pkt is sent */
    if (tx_pkt_type == BB_AUX1)
    {
        UINT8 txcmd_id;
        if (am_addr == BC_AM_ADDR)
        {
            txcmd_id = BZDMA_TX_ENTRY_TYPE_BROADCAST;
        }
        else
        {
            txcmd_id = BZDMA_TX_ENTRY_TYPE_PICONET0 + phy_piconet_id;
        }
        bzdma_release_tx_entry(txcmd_id);
    }

#ifdef _SUPPORT_VER_4_1_
#ifdef _SUPPORT_PCA_ADJUST
    if (IS_BT41 && (bt_pca_manager.pca_clk_req_sent) &&
        (bt_pca_manager.pca_clk_req_nbc_to) &&
        (tx_pkt_type == BB_POLL))
    {
        bt_pca_manager.tpoll_cnt_after_clk_req ++;
        if (bt_pca_manager.tpoll_cnt_after_clk_req > bt_pca_manager.clk_req_retry_th)
        {
            BB_prepare_clk_adj_broadcast(phy_piconet_id);
            bt_pca_manager.pca_clk_req_nbc_to = 0;
            bt_pca_manager.tpoll_cnt_after_clk_req = 0;
        }
    }
#endif
#endif

#ifdef COMPILE_AFH_HOP_KERNEL
    if (reg_tx_status->tx_id)
    {
        if (lmp_mss_state &
                (LMP_MSS_M_TO_S_FHS_RECD | LMP_MSS_M_TO_S_WAITING_FOR_FHS) )
        {
            ce_index = lmp_role_switch_data.ce_index;

            lmp_connection_entity[ce_index].afh_mode = AFH_DISABLE;
            BB_disable_afh(lmp_role_switch_data.old_am_addr,
                           lmp_connection_entity[ce_index].phy_piconet_id);
        }

#ifdef COMPILE_PARK_MODE
        if (lmp_self_device_data.number_of_parked_dev != 0)
        {
            RT_BT_LOG(GRAY, LC_ISR_4187, 0, 0);
        }
#endif

        return;
    }
#endif

#ifdef COMPILE_ESCO
    UINT16 esco_ce_index;
    if (lmp_get_esco_ce_index_from_am_addr(am_addr, phy_piconet_id,
                                          &esco_ce_index) == API_SUCCESS)
    {
        if (lc_esco_window[am_addr] == TRUE)
        {
            lc_esco_pkt_tx[am_addr] = TRUE;
        }
        return;
    }
#endif /* COMPILE_ESCO */

    if (tx_pkt_type == BB_FHS)
    {
        /* For Role-switch. */
        if (lmp_mss_state != LMP_MSS_INIT)
        {
            return;
        }

        /* During paging. */
        if (lmp_self_device_data.lc_cur_dev_state == LC_CUR_STATE_PAGE)
        {
            return;
        }

        /* The remaining case is during Inq-scan. */
        lc_handle_inq_fhs_sent_intr();
        RT_BT_LOG(GRAY, LC_ISR_4230, 1, BB_read_native_clock());

#ifdef VER_CSA4
#ifdef _SUPPORT_CSB_TRANSMITTER_
        if (IS_BT_CSA4)
        {
            hci_generate_csa4_inquiry_response_notification_event();
        }
#endif
#endif

#ifdef _INQRES_DBG_CCH_
        {
            OS_SIGNAL sig_send;

            /* Send signal to debug */
            sig_send.type = ISR_EXT_INQ_RES_LOG;
            OS_ISR_SEND_SIGNAL_TO_TASK(isr_extended_task_handle, sig_send);
        }
#endif

        return;
    }

    /* Get the connection entity from the am address */
    if (LMP_GET_CE_INDEX_FROM_AM_ADDR_PPI(am_addr, phy_piconet_id,
                                         &ce_index) == API_FAILURE)
    {
        return;
    }

    ce_ptr = &lmp_connection_entity[ce_index];

#ifdef _ROM_CODE_PATCHED_
#ifdef _DAPE_8723_RCP_
    if (rcp_dual_mode_switch_in_tx_func != NULL)
    {
        if (rcp_dual_mode_switch_in_tx_func((void *)schd, ce_index, lc_tx_status))
        {
            return;
        }
    }
#endif
#endif

#ifdef _DAPE_TEST_FIX_HW_SCO_NAK_SNIFF_BUG
    if (ce_ptr->should_ack)
    {
        ce_ptr->should_ack = 0;
    }
#endif
    lcl_tx_pkt_type = 0x01 << (tx_pkt_type);

    if (schd->tx_status == LC_TX_SENT)
    {
        if (schd->ce_index != INVALID_CE_INDEX)
        {
            if ((schd->selected_am_addr == am_addr) &&
                (schd->pkt_type_lut == (tx_pkt_type << 12)))
            {
                schd->tx_count++;

#ifdef _RATE_ADAPTION_SAMPLE_
                if (ce_ptr->ptt_status == LMP_PTT_ENABLED)
                {
                    if (lcl_tx_pkt_type & BB_RX_ISR_CRC_2M_PKT)
                    {
                        ce_ptr->send_2m_edr_tx_count++;
                    }
                    else if (lcl_tx_pkt_type & BB_RX_ISR_CRC_3M_PKT)
                    {
                        ce_ptr->send_3m_edr_tx_count++;
                    }
                }
#endif

#ifdef ENABLE_SCO
#ifdef _DAPE_TEST_FOR_HID_SCO
                if (schd->ce_index != ce_index)
                {
                    RT_BT_LOG(RED, MSG_CE_INDEX_MISMATCH, 2,
                                    ce_index, schd->ce_index);
                }

                if ((schd->tx_count>=4) &&
                    (lmp_self_device_data.total_no_of_sco_conn != 0) &&
                  (lc_get_master_piconet_id() != phy_piconet_id) &&
                  (ce_ptr->no_of_sco_connections == 0))
                {
                    bb_pause_sco(TRUE);

                    lc_sco_pause_status |= (BIT0 << phy_piconet_id);
                }
#ifdef COMPILE_ESCO
#ifdef _DAPE_TEST_NEW_HW_PAUSE_ESCO_WHEN_RETX
                // NUM_RETX_TO_PAUSE_ESCO default:2
                if ((schd->tx_count >= (NUM_RETX_TO_PAUSE_ESCO*2)) &&
                    (lmp_self_device_data.number_of_esco_connections != 0))
                {
                    UINT16 i;
                    UINT16 pause_esco = TRUE;
                    /* Check if the piconet of esco is the same as current ce_index.
                       If yes, then we shouldn't pause esco.
                       If no, then we can pause esco. */
                    for (i = 0; i < LMP_MAX_ESCO_CONN_ENTITIES; i ++)
                    {
                        if(lmp_esco_connection_entity[i].entity_status == ASSIGNED)
                        {
                            UINT16 tmp_ce_index;
                            tmp_ce_index = lmp_esco_connection_entity[i].ce_index;
                            if(lmp_connection_entity[tmp_ce_index].phy_piconet_id ==
                                phy_piconet_id)
                            {
                                pause_esco = FALSE;
                                break;
                            }
                        }
                    }
                    if (pause_esco)
                    {
                        UINT16 read_16;
                        BZ_REG_S_PRI_CTRL2 *pri_ctrl_reg2;
                        pri_ctrl_reg2 = (BZ_REG_S_PRI_CTRL2 *)&read_16;
                        read_16 = BB_read_baseband_register(SCA_PRIORITY_REGISTER3);
                        if (pri_ctrl_reg2->pause_esco == FALSE)
                        {
                            lc_esco_pause_times ++;
                            // PAUSE_ESCO_INTERVAL default: 1
                            if((lc_esco_pause_times % ((PAUSE_ESCO_INTERVAL * 2)+1)) == 0)
                            {
                                bb_pause_esco(TRUE);
                                lc_sco_pause_status |= (BIT0 << phy_piconet_id);
                            }
                        }
                    }
                }
#endif
#endif

#ifdef LE_MODE_EN
#ifdef _DAPE_NEW_HW_NO_BLOCK_LEGACY_FOR_LE_WHEN_RETX
                // NO_BLOCK_LEGACY_FOR_LE_WHEN_RETX default: 0
                if (NO_BLOCK_LEGACY_FOR_LE_WHEN_RETX &&
                    (((schd->tx_count>=4) && (ll_manager.conn_unit.connection_cnts))
                   || ((ce_ptr->remote_dev_role == MASTER)&& (g_enable_le_block_legacy == 0))))
                {
                    lc_le_pause_times ++;
                    // PAUSE_LE_INTERVAL default: 0
                    if((lc_le_pause_times % ((PAUSE_LE_INTERVAL * 2)+1)) == 0)
                    {
                        ll_driver_block_legacy_slot_for_le(1);
                        lc_le_pause_status |= (BIT0 << phy_piconet_id);
                    }
                }
#endif
#endif
#endif
#endif
            }
        }
    }

    if (ce_ptr->remote_dev_role != MASTER)
    {
        lcl_lc_waiting_for_crc_pkt_ack = FALSE;
    }

    if ((lcl_tx_pkt_type & (BB_RX_ISR_CRC_PKT)) && (am_addr != BC_AM_ADDR))
    {
        lcl_lc_waiting_for_crc_pkt_ack = TRUE;
    }

    if (lcl_lc_waiting_for_crc_pkt_ack != 0xFF)
    {
        lc_waiting_for_crc_pkt_ack[phy_piconet_id] =
            lcl_lc_waiting_for_crc_pkt_ack;
    }

#ifdef TEST_MODE
    if (((lmp_self_device_data.test_mode == HCI_DUT_LOOPBACK_MODE) ||
         (lmp_self_device_data.test_mode == HCI_DEVICE_UNDER_TEST_MODE)) &&
       (ce_ptr->test_mode_info.test_state == TEST_STARTED))
    {
        BB_handle_test_mode_in_tx_interrupt(lc_tx_status,
                                            &lut_ex_table[lut_index]);
    }
#endif /* TEST_MODE */

#ifdef COMPILE_SNIFF_MODE
    if (lcl_tx_pkt_type & (BB_RX_ISR_CRC_PKT) )
    {
        lc_check_exit_ssr(ce_index);
    }
#endif

    if (lmp_self_device_data.lc_no_of_connections[phy_piconet_id] != 0)
    {
        send_pkt_flag = lc_send_packet_in_scatternet(phy_piconet_id);
    }
#ifdef _DAPE_TEST_CHK_LUT
    if ((lut_index != 0)
        && (lc_sca_manager.pnet[phy_piconet_id].active == 1))
#else
    if (lut_index != 0)
#endif
    {
        if (lcl_tx_pkt_type & (BB_RX_ISR_CRC_PKT) )
        {
        }
        else
        {
            lc_cont_crc_tx_cnt[lut_index] >>= 2;

#ifdef COMPILE_DYNAMIC_POLLING
            if ((ce_ptr->remote_dev_role == SLAVE) &&
                    (lc_cont_crc_rx_cnt[lut_index] == 0) )
            {
                OS_SIGNAL sig_send;

                ce_ptr->no_inactive_tpolls++;
                /* Send signal to program tpoll */
                sig_send.type = LC_DYNAMIC_PROGRAM_TPOLL;
                sig_send.param = (OS_ADDRESS)((UINT32)ce_index);

                if (OS_ISR_SEND_SIGNAL_TO_TASK(lc_tx_task_handle, sig_send)
                        != BT_ERROR_OK)
                {
#ifdef ENABLE_LOGGER_LEVEL_2
                    RT_BT_LOG(GRAY, LC_ISR_4346, 0, 0);
#endif
                }
            }
#endif /* COMPILE_DYNAMIC_POLLING */
        }
    }

#ifdef _ROM_CODE_PATCHED_
#ifdef _DAPE_8723_RCP_
    if (rcp_bb_tx_intr_no_pkt_scheduled_func != NULL)
    {
        if (rcp_bb_tx_intr_no_pkt_scheduled_func((void *)&ce_index, lc_tx_status,
			                                send_pkt_flag))
        {
            return;
        }
    }
#endif
#endif
#if defined (_DAPE_ACL_PRI_HIGHER_THAN_INQ_WHEN_BR) || defined (_DAPE_BLOCK_LEGACY_LESS_SLOT_WHEN_BR)
    /* If lc_le_pause_status != 0 it means we want to pause le so we should not
      block acl too many slots. */
    if ((lc_le_pause_status == 0) && (g_enable_le_block_legacy == 0))

    {
        if (ce_ptr->ptt_status != LMP_PTT_ENABLED)
        {
#ifdef _DAPE_ACL_PRI_HIGHER_THAN_INQ_WHEN_BR        
            g_acl_priority_high_than_inq = 1;
#endif
#ifdef LE_MODE_EN
#ifdef _DAPE_BLOCK_LEGACY_LESS_SLOT_WHEN_BR
            ll_driver_block_legacy_slot_for_le(BLOCK_NUM_OF_LEGACY_WHEN_BR);
#endif
#endif
        }
        else
        {
#ifdef _DAPE_ACL_PRI_HIGHER_THAN_INQ_WHEN_BR
            g_acl_priority_high_than_inq = 0;
#endif
#ifdef LE_MODE_EN
#ifdef _DAPE_BLOCK_LEGACY_LESS_SLOT_WHEN_BR
            ll_driver_block_legacy_slot_for_le(g_block_legacy_for_le_slot_num);
#endif
#endif
        }
    }
#endif
    if ( ((tx_pkt_type == BB_NULL) && (send_pkt_flag == FALSE))
#ifdef _DAPE_TEST_SET_INVALID_PKT_WHEN_PAGE
          || ((tx_pkt_type == BB_POLL) && (send_pkt_flag == FALSE) &&
              (lmp_self_device_data.lc_cur_dev_state != LC_CUR_STATE_IDLE))
#endif
	)
    {
        if (ce_ptr->remote_dev_role == SLAVE)
        {
            if (!(RD_U32_BZDMA_REG(BZDMA_REG_TX_CMD(2 + phy_piconet_id)) & BIT31))
            {
                if (lut_index != 0)
                {
                    /* Change NULL to INVALID. */
                   BB_write_baseband_register(
                       lut_ex_table[lut_index].lower_lut_address,
                       LC_INVALID_PACKET_TYPE);
                   lc_cont_crc_rx_cnt[lut_index] = 0;
                   lc_cont_crc_tx_cnt[lut_index] = 0;
                }
            }
        }
    }

    if ((lmp_self_device_data.number_of_hlc == 1) && ce_ptr->in_sniff_mode)
    {
        ce_ptr->sniff_attempt_for_afh_count++;
    }
    return;
}


/**
* Handles slot counter interrupt from the baseband. This interrupt is
* not used currently.
*
* \param None.
*
* \return None.
*/
SECTION_ISR_LOW void bb_handle_slot_counter_intr(void)
{
    OS_SIGNAL sig_send;
    //LMP_LOG_INFO(LOG_LEVEL_HIGH, "One shot timer fired for.");

    /* Clear the interrupt. */
    BB_read_baseband_register(PAGE_SCAN_INTERVAL_REGISTER);

    /* Clear enable bit. */
    AND_val_with_bb_reg_macro(PAGE_SCAN_WINDOW_REGISTER, (~BIT13));

    sig_send.type = LC_SWITCH_ROLE_WAIT_TO_SIGNAL;

    if (OS_ISR_SEND_SIGNAL_TO_TASK(lc_rx_task_handle, sig_send)
            != BT_ERROR_OK)
    {
#ifdef ENABLE_LOGGER_LEVEL_2
        LC_LOG_INFO(LOG_LEVEL_LOW,
                    "OS send signal to task failed.");
#endif
    }
}

#ifdef _LPS_WITH_MULTI_LINK_
/**
* Handles inquiry/page scan end interrupt from the baseband.
*
* \param None.
*
* \return None.
*/
SECTION_ISR_LOW void bb_handle_scan_end_intr(UINT16 reg_int)
{
    UINT32 clock;
    UINT8 enter_lps_flag = 0;

    clock = BB_read_native_clock();
    clock = clock>>1;


    if( g_lps_timer_counter == 0)
    {
        enter_lps_flag = lc_check_lps_for_link( 0, 0, 1);
    }

#ifdef _WA_8821_SCAN_CNT_STOP_WHEN_LPS_

    sleep_mode_param.lps_table_instance[18] = clock + lmp_self_device_data.scan_interval_min;

    if( enter_lps_flag)
    {
        if( reg_int&BIT15 )
        {
            if(lmp_self_device_data.scan_enable&0x1)
            {
                BB_write_baseband_register(INSTRUCTION_REGISTER, BB_KILL_PAGE_SCAN);
                BB_write_baseband_register(INSTRUCTION_REGISTER, BB_KILL_INQUIRY_SCAN);
                lc_retrieve_inq_scan();
                //RT_BT_LOG(BLUE, LPS_LOG_041, 3, 1, clock, sleep_mode_param.lps_table_instance[18]);
            }else
            {
                sleep_mode_param.wakeup_scan_enable = 0;
                lc_post_lps_mode_signal(LPS_WITH_WAKEUP_INSTANCE, 0, 0);
                //RT_BT_LOG(GRAY, LPS_LOG_041, 3, 1, clock, sleep_mode_param.lps_table_instance[18]);
            }
        }else if( reg_int&BIT14 )
        {
            sleep_mode_param.wakeup_scan_enable = 0;
            lc_post_lps_mode_signal(LPS_WITH_WAKEUP_INSTANCE, 0, 0);
            //RT_BT_LOG(GRAY, LPS_LOG_041, 3, 2, clock, sleep_mode_param.lps_table_instance[18]);
        }
    }


#else
    if( reg_int&BIT15 )
    {
            UINT32 reg;

           // ps_end_intr
           BB_write_baseband_register(SCAN_ANCHOR_SEL, 0);
           reg = BB_read_baseband_register(SCAN_ANCHOR);
           reg = ((reg & (UINT32)(0xFFFF)) << 1)| (clock & (0xFFFF0000));

           reg = reg + lmp_self_device_data.page_scan_interval - (lmp_self_device_data.page_scan_window<<1) -2;
           sleep_mode_param.lps_table_instance[18] = reg;
#ifdef PATCH_LPS_LOG_EN_TEST_1
           LPS_CCH_LOG(GRAY, CCH_SEC_018, 4, 1, reg_int, clock, reg);
           LPS_CCH_LOG(GRAY, CCH_DBG_148, 4, g_lps_timer_counter, sleep_mode_param.lps_table_instance[18], sleep_mode_param.lps_table_instance[19], sleep_mode_param.lps_task_flag);
#endif

#ifdef _CCH_8821_TEST_
           if( g_lps_timer_counter == 0)
           {
               BB_write_baseband_register(INSTRUCTION_REGISTER, BB_KILL_PAGE_SCAN);
               if(lmp_self_device_data.scan_enable&0x1)
               {
                   lc_retrieve_inq_scan();
               }else
               {
                   lc_post_lps_mode_signal(LPS_WITH_WAKEUP_INSTANCE);
               }
           }

#else
           if( g_lps_timer_counter == 0)
           {
               lc_post_lps_mode_signal(LPS_WITH_WAKEUP_INSTANCE, 0,0);
           }

#endif
    }
    else if( reg_int&BIT14 )
    {
            UINT32 reg;

           // is_end_intr
           BB_write_baseband_register(SCAN_ANCHOR_SEL, 1);
           reg = BB_read_baseband_register(SCAN_ANCHOR);
           reg = ((reg & (UINT32)(0xFFFF)) << 1)| (clock & (0xFFFF0000));

           reg = reg + lmp_self_device_data.inquiry_scan_interval - (lmp_self_device_data.inquiry_scan_window<<1) -2;
           sleep_mode_param.lps_table_instance[19] = reg;


#ifdef PATCH_LPS_LOG_EN_TEST
           LPS_CCH_LOG(GRAY, CCH_SEC_018, 4, 0, reg_int, clock, reg);
#endif
           if( g_lps_timer_counter == 0)
           {
               BB_write_baseband_register(INSTRUCTION_REGISTER, BB_KILL_INQUIRY_SCAN);
               lc_post_lps_mode_signal(LPS_WITH_WAKEUP_INSTANCE, 0 ,0 );
           }
    }
#endif


}
#endif

UINT8 g_bb_max_re_trans_var = BB_MAX_RE_TRANS;

/**
* Checks and issues send-packet instruction to the baseband in flow GO state.
* Also updates the scheduler states.
*
* \param phy_piconet_id Physical piconet ID of the connection
* \param am_addr lt address of the connection
* \param *schd the pointer of software scheduled packet entry
* \param *ex_lut the pointer of lut table extension
*
* \return TRUE, if the send-packet is issued, FALSE otherwise.
*/
UINT8 lc_send_packet_in_flow_go(UINT8 phy_piconet_id, UINT8 am_addr,
                                        LC_SCHEDULED_PKT* schd,
                                        LUT_EXTENSION_TABLE *ex_lut)
{
    UINT16 lut_contents;
    LMP_PDU_PKT* lmp_pdu_ptr;
    UINT32 curr_clock;
    UINT16 curr_clock_us;
    UINT32 lcl_tx_pkt_type;
    LMP_CONNECTION_ENTITY *ce_ptr;

    lcl_tx_pkt_type = 1 << (schd->pkt_type_lut >> 12);
    ce_ptr = &lmp_connection_entity[schd->ce_index];

#ifndef _DAPE_TEST_NEW_HW_BLOCK_ACL_FOR_LE
#ifdef _DAPE_TEST_NO_PKT_WHEN_LE
    if ((BLOCK_LEGACY_WHEN_LE)&&
        (ll_manager.conn_unit.connection_cnts != 0))
    {
        if (lcl_tx_pkt_type &
            (BB_RX_ISR_2_DH3 | BB_RX_ISR_3_DH3 | BB_RX_ISR_2_DH5 | BB_RX_ISR_3_DH5))
        {
            UINT32 anchor_point_slot_max = 0;
            UINT32 anchor_point_slot_min = 0xFFFFFFFF;
            UINT32 next_anchor_point_slot_min = 0xFFFFFFFF;
            UINT8  num_le_conn = 0;
            UINT16 empty_entry = LL_MAX_CONNECTION_UNITS;
            UINT32 cur_clk, cur_clock;
            UINT32 cur_clock_slot;

            cur_clock = BB_read_native_clock();
            cur_clock_slot = cur_clock>>1;

            ll_fw_get_all_anchor_point(&anchor_point_slot_max,
                                        &anchor_point_slot_min, &next_anchor_point_slot_min,
                                        &num_le_conn, &empty_entry, cur_clock_slot);

            lc_get_clock_in_scatternet(&cur_clk, phy_piconet_id);

            if (lcl_tx_pkt_type & (BB_RX_ISR_3_DH5 | BB_RX_ISR_2_DH5))
            {
                if (((cur_clk + 10 + (BLOCK_SLOT_NUM_5SLOT*2))>>1) >= anchor_point_slot_min)
                {
                    return FALSE;
                }
            }

            if (lcl_tx_pkt_type & (BB_RX_ISR_3_DH3 | BB_RX_ISR_2_DH3))
            {
                if (((cur_clk + 6 + (BLOCK_SLOT_NUM_3SLOT*2))>>1) >= anchor_point_slot_min)
                {
                    return FALSE;
                }
            }
        }
    }
#endif
#endif

    if (schd->txdesc_used_cnt > 0)
    {
        BB_dma_write_baseband_TX_FIFO(schd->txdesc, schd->txdesc_used_cnt,
                                am_addr, phy_piconet_id, 0, 0);
    }

    lut_contents = schd->lch | schd->frag_len | schd->pkt_type_lut;
    OR_val_with_bb_reg_macro(ex_lut->upper_lut_address,
                        (BB_get_current_flow() << BB_L2CAP_FLOW_BIT_POS));

#ifdef BROADCAST_DATA
    if (am_addr == BC_AM_ADDR)
    {
        UCHAR nb_val = 0;
        UCHAR lcl_num_broadcast_retran;
        UCHAR lcl_lmp_current_nb;
#ifdef COMPILE_PARK_MODE
        UCHAR lcl_number_of_parked_dev;
#endif

        lcl_num_broadcast_retran = lmp_self_device_data.num_broadcast_retran;
#ifdef COMPILE_PARK_MODE
        lcl_number_of_parked_dev = lmp_self_device_data.number_of_parked_dev;
#endif
        lcl_lmp_current_nb = lmp_current_nb;

        if ((lcl_lmp_current_nb > lcl_num_broadcast_retran)
#ifdef COMPILE_PARK_MODE
                && (lcl_number_of_parked_dev > 0)
#endif
          )
        {
            if (lcl_lmp_current_nb != 0)
            {
                nb_val = (UCHAR) (lcl_lmp_current_nb - 1);
            }
        }
        else
        {
            if (lcl_num_broadcast_retran != 0)
            {
                nb_val = (UCHAR) (lcl_num_broadcast_retran - 1);
            }
        }
        nb_val++;
        BB_write_baseband_register_lower_octet(
            RE_TRANSMISSION_COUNT_REGISTER,
            nb_val);
    } /* end of if (am_addr == BC_AM_ADDR) */
#endif /* BROADCAST_DATA */

    BB_write_baseband_register(ex_lut->lower_lut_address, lut_contents);

    schd->tx_status = LC_TX_SENT;

    lmp_pdu_ptr = (LMP_PDU_PKT*)schd->packet_ptr;
    if ((schd->lch == BB_LMP_PKT) &&   /* Don't reorder the conditions */
        (lmp_pdu_ptr->pdu_sent == FALSE))
    {
        OS_SIGNAL signal;

        signal.type = LC_PDU_SENT_SIGNAL;
        signal.param = schd->packet_ptr;
        OS_ISR_SEND_SIGNAL_TO_TASK(lc_tx_task_handle, signal);
        lmp_pdu_ptr->pdu_sent = TRUE;   /* PDU sent at least once */
    }

#ifdef BROADCAST_DATA
    if (am_addr == BC_AM_ADDR)
    {
        BB_disable_NBC(phy_piconet_id);
    }
    else
#endif
    {
        UCHAR nbc_timeout = g_bb_max_re_trans_var;

#ifdef COMPILE_SNIFF_MODE
        if (ce_ptr->in_sniff_mode == TRUE)
        {
            UINT32 diff;

            curr_clock = BB_read_native_clock();

            diff = lc_get_clock_diff(ce_ptr->next_instant_in_nat_clk,
                                     curr_clock, 0);

            if ((diff > LC_MAX_SLOTS_TO_FORCE_NBC_FOR_SNIFF_PKT) &&
                (diff < (ce_ptr->sniff_interval << 1)))
            {
                /* Enable NBC with 0 or 1 timeout */
                nbc_timeout = 0x1;
            }

#ifdef _DAPE_EN_8821_MP_MODIFY_SLV_SNIFF_TIMEOUT
            if (ce_ptr->remote_dev_role == MASTER)
            {
                UINT16 reg21c = BB_read_baseband_register(0x21c);
                reg21c &= ~BIT6;
                BB_write_baseband_register(0x21c, reg21c);
                lc_sniff_slv_send_pkt |= (BIT0 << phy_piconet_id);
            }
#endif

        }
#endif

        /* Enable NBC when we are MASTER */
        if (lmp_self_device_data.lc_no_of_connections[phy_piconet_id] == 1)
        {
            /* Disable slot based NBC */
            BB_enable_NBC(phy_piconet_id, nbc_timeout, FALSE);
        }
        else
        {
            /* Enable slot based NBC */
            BB_enable_NBC(phy_piconet_id, nbc_timeout, TRUE);
        }
    }

    BB_write_baseband_register(CONNECTOR_REGISTER,
                    (UINT16) ( (am_addr << 5) | (phy_piconet_id << 11)) ) ;

#ifndef _DONT_USE_BZ_SEND_PACKET_INSTRUCTION
    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_SEND_PACKET);
#endif

    /* help moniter air situation for data transfer - austin */
    lc_get_high_dpi_clock_in_scatternet(&curr_clock, &curr_clock_us,
                                        phy_piconet_id);

    schd->tx_count = 0;
    schd->rx_count = 0;
    schd->flow_ctrl_count = 0;
    schd->fw_sent_time = curr_clock;
    schd->fw_sent_time_us = curr_clock_us;

#ifdef _RATE_ADAPTION_SAMPLE_
    if (ce_ptr->ptt_status == LMP_PTT_ENABLED)
    {
        if (lcl_tx_pkt_type & BB_RX_ISR_CRC_2M_PKT)
        {
            ce_ptr->send_2m_edr_pkt_count++;
        }
        else if (lcl_tx_pkt_type & BB_RX_ISR_CRC_3M_PKT)
        {
            ce_ptr->send_3m_edr_pkt_count++;
        }
    }
#endif

#ifdef ENABLE_SCO
#ifndef _CCH_NO_PAUSE_SCO_
    /* Check if multi-slot packet. Note that this relies on all
       multislot pkt_types being greater than 0xA000. */
    UINT16 max_packet_type;

    if (lc_check_if_device_is_in_scatternet())
    {
        max_packet_type = 0xA000; /* 3 slots */
    }
    else
    {
        max_packet_type = 0xe000; /* 5 slots */
    }

    if (((schd->pkt_type_lut) >= max_packet_type) ||
            ( (lmp_self_device_data.total_no_of_sco_conn != 0) &&
              (lc_get_master_piconet_id() != phy_piconet_id) &&
              (ce_ptr->no_of_sco_connections == 0)))
    {
        BZ_REG_S_PRI_CTRL pri;

        *(UINT16*)&pri = BB_read_baseband_register(SCA_PRIORITY_REGISTER);
        pri.pause_sco = TRUE;
        lc_sco_pause_status |= (BIT0 << phy_piconet_id);
        BB_write_baseband_register(SCA_PRIORITY_REGISTER, *(UINT16*)&pri);
    }
#endif
#endif

    return TRUE;
}


/**
* Checks and issues send-packet instruction to the baseband. Also
* updates the scheduler states.
*
* \param piconet_id Physical piconet ID of the connection
*
* \return TRUE, if the send-packet is issued, FALSE otherwise.
*/
SECTION_ISR UCHAR  lc_send_packet_in_scatternet(UCHAR phy_piconet_id)
{
    LC_SCHEDULED_PKT *schd;
    LUT_EXTENSION_TABLE *ex_lut;
    UCHAR lut_index;
    UCHAR am_addr;
    UCHAR send_pkt_issued_flag = FALSE;

    if (lmp_mss_state != LMP_MSS_INIT)
    {
        return FALSE;
    }

    /* Construct LUT contents and update LUT. */
    LC_PICONET_SCHEDULER *piconet_schd;
    piconet_schd = &lc_piconet_scheduler[phy_piconet_id];
    schd = &piconet_schd->lc_scheduled_pkt_info[piconet_schd->rptr];

    if (schd->tx_status != LC_TX_READY)
    {
#ifdef _CCH_SC_TEST_20130201_BCM_LOG__
RT_BT_LOG(GRAY, CCH_DBG_033, 1,phy_piconet_id);
#endif
        return FALSE;
    }

#ifdef _ROM_CODE_PATCHED_
#ifdef _DAPE_8723_RCP_
    if (rcp_lc_send_packet_for_le_func != NULL)
    {
        UINT8 send_pkt = FALSE;
        if(rcp_lc_send_packet_for_le_func((void*)schd, &send_pkt, phy_piconet_id))
        {
            return send_pkt;
        }
    }
#endif
#endif

    am_addr = schd->selected_am_addr;
    lut_index = lc_get_lut_index_from_phy_piconet_id(am_addr, phy_piconet_id);
    ex_lut = &lut_ex_table[lut_index];

    /* Check H\W and S\W flow bit for LMP and L2CAP packet */
    if ((schd->tx_status == LC_TX_READY) && (ex_lut->bb_flow != 0))
    {
        send_pkt_issued_flag = lc_send_packet_in_flow_go(phy_piconet_id,
                                            am_addr, schd, ex_lut);
    }
    else if (IS_USE_FOR_BQB)
    {
        /*
        * BQB Fix: Tester sends Flow 0 and then queues a acl packet.
        * This code re-schedules the packet - so that 'flush' can be checked.
        */
        if ((ex_lut->bb_flow == 0) && (schd->tx_status == LC_TX_READY) &&
            (schd->lch != BB_LMP_PKT) &&
          !((piconet_schd->rptr != piconet_schd->wptr) &&
    		(piconet_schd->lc_allowed_pkt_cnt == 0)))
        {
            OS_SIGNAL sig_send;

            /* in 0380, hw shares one acl tx fifo in all piconets, and HW
               scheduler can flush the tx fifo before sent acl unicast tx pkt.
               so FW don't flush the tx fifo */

            /*
            * Mark as Sent: Otherwise the packet
            * might get 'sent' before re-scheduled
            */
            schd->tx_status = LC_TX_SENT;
            sig_send.type = LC_HANDLE_RESCHEDULE;
            sig_send.param = (void *)((UINT32)phy_piconet_id);
            sig_send.ext_param = (void *)((UINT32)lut_index);

            OS_SEND_SIGNAL_TO_TASK (lc_tx_task_handle, sig_send);
        }
    } /* end of ((schd->tx_status == LC_TX_READY) && (ex_lut->bb_flow != 0))*/

    return send_pkt_issued_flag;
}

#ifdef COMPILE_PARK_MODE
/**
 * Handles park mode interrupts. This function decides if the park
 * interrupt is for start of beacon or end of beacon and processes
 * them acordingly.
 *
 * \param mode_sts_reg The interrupt status register read from baseband.
 * \param logical_piconet_id The logical piconet ID of the connection.
 * \param phy_piconet_id The physical piconet ID of the connection.
 *
 * \return None.
 */
SECTION_ISR_LOW void BB_handle_park_imode_interrupts(UINT16 mode_sts_reg,
                                        UCHAR phy_piconet_id)
{
    UCHAR am_addr;
    UINT16 read;
    LC_PICONET_SCHEDULER *piconet_schd;
    LC_SCHEDULED_PKT *schd;
    UINT32 i;
    UINT16 lut_contents;
    UCHAR lut_index;
    UINT16 address;

    LMP_CONNECTION_ENTITY *ce_ptr = NULL;

    if (mode_sts_reg & BB_PARK_MODE_IND)
    {
        lc_park_conn_intr();
    }

    if (mode_sts_reg & BB_START_BEACON_IND)
    {
        RT_BT_LOG(GRAY, LC_ISR_5021, 0, 0);

        /* Program to open receive window if we are slave */
        if (slave_ce_index != INVALID_CE_INDEX)
        {
            ce_ptr = &lmp_connection_entity[slave_ce_index];

            if (ce_ptr->remote_dev_role == MASTER)
            {
                lc_program_tolerance(ce_ptr->Tbeacon, slave_ce_index);
            }
        }

        do
        {
            if (lmp_unpark_ce_index != INVALID_CE_INDEX)
            {
                ce_ptr = &lmp_connection_entity[lmp_unpark_ce_index];
            }
            else
            {
                RT_BT_LOG(GRAY, LC_ISR_INVALID, 0, 0);
                break;
            }

            if ((ce_ptr->remote_dev_role == MASTER) &&
                ((ce_ptr->unpark_req_flag == LMP_UNPARK_HOST_INITIATED) ||
                (ce_ptr->unpark_req_flag == LMP_UNPARK_AUTO_CONTROLLER_INITIATED)) &&
                (lc_start_of_beacon == TRUE))
            {
                lc_start_of_beacon = FALSE;

                lc_unpark_req(lmp_unpark_ce_index);

                RT_BT_LOG(GRAY, LC_ISR_5085, 1, ce_ptr->ar_addr);

                lc_num_unpark_req_trials++;
            }
            else if ((lc_start_of_beacon == TRUE) &&
                    (ce_ptr->remote_dev_role == SLAVE))
            {
                lc_start_of_beacon = FALSE;

                /* Load Broadcast PDU to the Broadcast am_address LUT */
                /* Deliver the LMP PDU to LC module for transmission */
                if (ce_ptr->unpark_req_flag == LMP_UNPARK_READY)
                {
                    /* Activate the new am_addr in LUT and write the default packet
                    type. */
                    am_addr = (UCHAR) ce_ptr->am_addr;

                    if (ce_ptr->remote_dev_role == MASTER)
                    {
                        lut_index = LC_SCA_SLAVE_1_LUT;

                        if (ce_ptr->phy_piconet_id <= SCA_PICONET_FOURTH)
                        {
                            /* Idle_slave configuration. */
                            lut_index = LC_SCA_SLAVE_1_LUT + ce_ptr->phy_piconet_id;
                        }
                    }
                    else
                    {
                        lut_index = am_addr;
                    }

                    lc_update_ce_index_to_lut_extn_tbl(lmp_unpark_ce_index,
                                                       lut_index);

                    address = lut_ex_table[lut_index].lower_lut_address;

                    BB_write_baseband_register(address,
                                               LC_MASTER_DEFAULT_PACKET_TYPE);

                    /* Make this Am Address active */
                    address += 2;
                    read = BB_read_baseband_register(address);
                    /* Initialize arqn = 0 seqn = 0 and flow = 1 bits */
                    read &= 0xFFF8;
                    read |= ACTIVE_BIT;
                    read |= 0x0001;

                    {
                        BZ_REG_S_PICONET_INFO read_val;
                        UINT16 read_addr;

                        /* Update Piconet info register. */
                        if (ce_ptr->phy_piconet_id <= SCA_PICONET_MAX)
                        {
                            read_addr = reg_PICONET_INFO[ce_ptr->phy_piconet_id];
                        }
                        else
                        {
                            read_addr = PICONET2_INFO_REGISTER;
                            RT_BT_LOG(RED, PICONET_ID_CHECK_MSG, 1,
                                            ce_ptr->phy_piconet_id);
                        }

                        *(UINT16*)&read_val = BB_read_baseband_register(read_addr);
                        read_val.lt_addr = 0;
                        read_val.master = TRUE;
                        BB_write_baseband_register(read_addr, *(UINT16*)&read_val);
                    }

#ifdef MASTER_AFH_PARK
                    if (ce_ptr->afh_mode == AFH_ENABLE)
                    {
                        BB_write_afh_map(am_addr, ce_ptr->phy_piconet_id,
                                         ce_ptr->afh_map);
                    }
#endif

#ifndef PARK_MASTER_DOES_NOT_SEND_POLL
                    BB_write_baseband_register(address, read);
#endif

                    lc_num_unpark_req_trials++;

#ifndef PARK_MASTER_DOES_NOT_SEND_POLL
                    lc_exit_beacon(lmp_unpark_ce_index);
#endif

                    lc_park_setup_lc_level_conn_during_unpark_procedure();

                    RT_BT_LOG(GRAY, LC_ISR_5187, 3, am_addr, lut_index, address);
                } /* end of if (ce_ptr->unpark_req_flag == LMP_UNPARK_READY) */

                RT_BT_LOG(GRAY, LC_ISR_5191, 0, 0);
            } /* end of else if ((lmp_unpark_ce_index != INVALID_CE_I.... */

            if (lmp_connection_entity[lmp_unpark_ce_index].link_supervision_timeout != 0)
            {
                lc_num_unpark_req_trials = 0;
            }

            if (lc_num_unpark_req_trials > LMP_MAX_UNPARK_TRAILS)
            {
                OS_SIGNAL signal;

#ifdef ENABLE_LOGGER_LEVEL_2
                RT_BT_LOG(GRAY, LC_ISR_5209, 0, 0);
#endif
                am_addr = (UCHAR) ce_ptr->am_addr ;
                if (ce_ptr->remote_dev_role == SLAVE)
                {
                    /* Deactivate the LUT and write Invalid Packet */
                    lut_index = am_addr;

                    address = lut_ex_table[lut_index].lower_lut_address;

                    BB_write_baseband_register(address, LC_INVALID_PACKET_TYPE);
                    /* Make this Am Address deactive */
                    address += 2;
                    AND_val_with_bb_reg_macro(address, ~(ACTIVE_BIT));

                    /* Write default packet type to Broadcast LUT. */
                    address = (PICONET_LOOK_UP_TABLE_BASE_ADDR );
                    read = BB_read_baseband_register(address);
                    BB_write_baseband_register(address,
                                               LC_SLAVE_DEFAULT_PACKET_TYPE);
                }

                lc_num_unpark_req_trials = 0;

                ce_ptr->unpark_req_flag = LMP_UNPARK_IDLE;

                {
                    UCHAR lcl_lut_index;

                    if (ce_ptr->remote_dev_role == SLAVE)
                    {
                        lcl_lut_index = ce_ptr->am_addr;
                    }
                    else
                    {
                        if (ce_ptr->phy_piconet_id <= SCA_PICONET_MAX)
                        {
                            lcl_lut_index = LC_SCA_SLAVE_1_LUT + ce_ptr->phy_piconet_id;
                        }
                        else
                        {
                            lcl_lut_index = LC_SCA_SLAVE_1_LUT;
                            RT_BT_LOG(RED, PICONET_ID_CHECK_MSG, 1, ce_ptr->phy_piconet_id);
                        }
                    }

                    lut_ex_table[lcl_lut_index].index_in_CE = INVALID_CE_INDEX;
                }

                signal.type = LMP_ABNORMAL_EXIT_PARK_SIGNAL;
                signal.param = (OS_ADDRESS)((UINT32)lmp_unpark_ce_index);
                signal.ext_param = (OS_ADDRESS)UNSPECIFIED_ERROR;
                OS_SEND_SIGNAL_TO_TASK(lmp_task_handle, signal);

                lmp_unpark_ce_index = INVALID_CE_INDEX;
            } /* end of if (lc_num_unpark_req_trials > LMP_MAX_UNPARK_TRAILS && .. */
        }
        while (0);
    } /* end of if (mode_sts_reg & BB_START_BEACON_IND) */

#if 0
    if (mode_sts_reg & BB_START_ACCESS_WINDOW_IND)
    {
        RT_BT_LOG(GRAY, LC_ISR_5320, 0, 0);
    } /* end of if (mode_sts_reg & BB_START_ACCESS_WINDOW_IND) */
#endif

    if (mode_sts_reg & BB_END_ACCESS_WINDOW_IND)
    {
        LMP_CONNECTION_ENTITY *ce_ptr;

        lc_start_of_beacon = TRUE;

        /*
        If NB was changed on receiving SET_BC_SCAN_WINDOW, revert it back
        to default value
        */

        if (slave_ce_index != INVALID_CE_INDEX)
        {
            ce_ptr = &lmp_connection_entity[slave_ce_index];

            if (ce_ptr->remote_dev_role == MASTER)
            {
                if (ce_ptr->bc_scan_window == TRUE)
                {
                    ce_ptr->bc_scan_window = FALSE;
                    LC_SET_BROADCAST_SCAN_WINDOW(ce_ptr->Nbeacon);
                }
            }
        }

        RT_BT_LOG(GRAY, LC_ISR_5380, 0, 0);

        if (lc_num_unpark_req_trials == 0)
        {
            lmp_select_unpark_device();
        }

        piconet_schd =  &lc_piconet_scheduler[phy_piconet_id];

        schd = &piconet_schd->lc_scheduled_pkt_info[piconet_schd->rptr];

        for (i = 0; i < LMP_MAX_CE_DATABASE_ENTRIES; i++)
        {
            LMP_CONNECTION_ENTITY *ce_ptr;

            ce_ptr = &lmp_connection_entity[i];

            if ((ce_ptr->ce_status == LMP_PARK_MODE) &&
                    (ce_ptr->auto_unpark_cnt != 0xffff) &&
                    (ce_ptr->unpark_req_flag == LMP_UNPARK_IDLE))
            {

                if (ce_ptr->auto_unpark_cnt != 0)
                {
                    ce_ptr->auto_unpark_cnt--;
                }

                if ((ce_ptr->auto_unpark_cnt == 0) &&
                        (lc_slave_init_unpark_pending == FALSE))
                {
                    UCHAR ret_val;
                    lmp_unpark_ce_index = (UINT16) i;

                    ret_val = lmp_handle_master_unpark( (UINT16) i);
                    if (ret_val != HCI_COMMAND_SUCCEEDED)
                    {
                        RT_BT_LOG(GRAY, LC_ISR_5421, 1, ret_val);
                    }
                    else
                    {
                        ce_ptr->unpark_req_flag =
                            LMP_UNPARK_AUTO_CONTROLLER_INITIATED;
                    }

                    ce_ptr->supto_auto_repark = TRUE;

                    ce_ptr->hci_unpark_req_flag =
                        HCI_UNPARK_AUTO_CONTROLLER_INITIATED;

                    RT_BT_LOG(GRAY, LC_ISR_5436, 0, 0);

                    /* Only one at a time */
                    break;
                }
            }
        }

        if ((lmp_unpark_ce_index != INVALID_CE_INDEX) &&
                (!((schd->selected_am_addr == BC_AM_ADDR) &&
                   (schd->tx_status != LC_TX_IDLE))))
        {
            LMP_CONNECTION_ENTITY *ce_ptr;

            ce_ptr = &lmp_connection_entity[lmp_unpark_ce_index];

            if (((((ce_ptr->unpark_req_flag == LMP_UNPARK_HOST_INITIATED) ||
                     (ce_ptr->unpark_req_flag == LMP_UNPARK_AUTO_CONTROLLER_INITIATED) ||
                     (ce_ptr->unpark_req_flag == LMP_UNPARK_READY)) &&
                    (ce_ptr->remote_dev_role == SLAVE)) ||
                    (ce_ptr->auto_unpark_cnt == 0)) &&
                    (lc_slave_init_unpark_pending == FALSE))
            {
#ifdef ENABLE_LOGGER_LEVEL_2
                RT_BT_LOG(GRAY, LC_ISR_5459, 1, lc_num_unpark_req_trials);
#endif
                /* It means packet can be written in fifo, Write fifo */
                LMP_PDU_PKT *ppkt = &ce_ptr->lmp_pdu_pkt;

                /* copy data from dmem to sram */
                memcpy(bzdma_tx_buf[phy_piconet_id],
                       ppkt->payload_content, ppkt->pdu_length);
                BB_write_baseband_TX_FIFO_scatternet (
                    bzdma_tx_buf[phy_piconet_id],
                    ppkt->pdu_length, BC_AM_ADDR, phy_piconet_id);

                address = (UINT16) (PICONET_LOOK_UP_TABLE_BASE_ADDR);

                lut_contents = (BB_DM1 << 12) | (0x03 << 10) |
                                ce_ptr->lmp_pdu_pkt.pdu_length;
                UINT16 read = BB_read_baseband_register(
                                PICONET_LOOK_UP_TABLE_BASE_ADDR + 2);
                read &= 0xE1FF;
                read |= MAX_RADIO_TX_POWER << 9;

#ifdef _SUPPORT_LBT_FUNC_FOR_RF_ADAPTIVITY_
                if (rtl8723_btrf_check_and_enable_lbt(MAX_RADIO_TX_POWER))
                {
                    read |= BIT12;
                }
#endif

                BB_write_baseband_register(address + 2, read);
                BB_write_baseband_register(address, lut_contents);

                LC_SET_2M_TX_POWER(0,MAX_RADIO_TX_POWER);
                LC_SET_3M_TX_POWER(0,MAX_RADIO_TX_POWER);

                ce_ptr->unpark_req_flag = LMP_UNPARK_READY;

                RT_BT_LOG(GRAY, LC_ISR_5498, 0, 0);
            }
        }
        else
        {
            if (schd->selected_am_addr == BC_AM_ADDR)
            {
                UCHAR master_piconet_id;

                master_piconet_id = lc_get_master_piconet_id();

                if (master_piconet_id != SCA_PICONET_INVALID)
                {
                    lc_send_packet_in_scatternet(master_piconet_id);
                }
            }
        }

        if (lc_slave_init_unpark_pending == TRUE)
        {
            lmp_handle_slave_unpark_req(INVALID_AR_ADDR, INVALID_PICONET_ID);
        }

    } /* end of if (mode_sts_reg & BB_END_ACCESS_WINDOW_IND) */
}
#endif

/**
 * Handles the HCL interrupt from the baseband.
 *
 * \param None.
 *
 * \return None.
 */
#ifdef _SUPPORT_CSB_RECEIVER_
SECTION_ISR_LOW void BB_handle_hlc_interrupt_in_scatternet(UCHAR phy_piconet_id)
#else
SECTION_ISR_LOW void BB_handle_hlc_interrupt_in_scatternet()
#endif
{
    OS_SIGNAL sig_send;
    API_RESULT result;
    UCHAR am_addr;
    UINT16 ce_index;
    UCHAR lut_index;
    UCHAR self_device_role;
    LUT_EXTENSION_TABLE *ex_lut;
    UINT8 pre_dev_state;
    UCHAR new_piconet_id;
#ifndef _SUPPORT_CSB_RECEIVER_
    UINT16 mode_status_reg;
    UCHAR phy_piconet_id;
    mode_status_reg = BB_read_baseband_register(MODE_STATUS_REGISTER);
    phy_piconet_id = ((mode_status_reg >> 15) << 1) |
                     ((mode_status_reg >> 13) & 0x01);
#endif

    am_addr = lc_cur_connecting_am_addr;

    lc_set_lc_cur_connecting_am_addr(INVALID_AM_ADDR);

#ifdef ENABLE_LOGGER_LEVEL_2
    /* Need to change this for mt_custom_radio_2.
    The value 0xf8f8 written in radio-initialisation is overwritten here. */
#endif

    pre_dev_state = lmp_self_device_data.lc_cur_dev_state;

    if (lmp_self_device_data.lc_cur_dev_state == LC_CUR_STATE_PAGE)
    {
        new_piconet_id = lc_paging_piconet_id;
        lc_paging_piconet_id = SCA_PICONET_INVALID;

        /* Update scatternet_state here. */
        new_piconet_id =
                lc_update_scatternet_state_addition(SCA_MASTER, new_piconet_id);

        if (new_piconet_id != phy_piconet_id)
        {
            RT_BT_LOG(YELLOW, MSG_PAGING_PICONET_DIFF, 2,
                            phy_piconet_id, new_piconet_id);
        }

        lut_index = lc_get_lut_index_from_phy_piconet_id(am_addr, phy_piconet_id);

        ex_lut = &lut_ex_table[lut_index];

        BB_write_baseband_register(CONNECTOR_REGISTER,
                         (UINT16)( (am_addr << 5) | (phy_piconet_id << 11) ) );

        BB_write_baseband_register(ex_lut->lower_lut_address,
                                   LC_MASTER_DEFAULT_PACKET_TYPE);

        lc_set_lc_cur_device_state(LC_CUR_STATE_IDLE);
        self_device_role = MASTER;
    }
    else
    {
        new_piconet_id = lc_pagescan_piconet_id;
        lc_pagescan_piconet_id = SCA_PICONET_INVALID;

        /* Update scatternet_state here. */
        new_piconet_id =
                lc_update_scatternet_state_addition(SCA_SLAVE, new_piconet_id);

        if (new_piconet_id != phy_piconet_id)
        {
            RT_BT_LOG(YELLOW, MSG_PAGESCAN_PICONET_DIFF, 2,
                                            phy_piconet_id, new_piconet_id);
        }

        lut_index = LC_SCA_SLAVE_1_LUT + new_piconet_id;

        BB_write_baseband_register(CONNECTOR_REGISTER,
                          (UINT16)(am_addr << 5) | (new_piconet_id << 11));

        ex_lut = &lut_ex_table[lut_index];

        BB_write_baseband_register(ex_lut->lower_lut_address,
                                   LC_SLAVE_DEFAULT_PACKET_TYPE);
#ifdef _CSB_RX_DBG_LOG
        if (g_csb_rx_dbg_log)
        {
            RT_BT_LOG(WHITE, DAPE_TEST_LOG525, 6,
            am_addr, new_piconet_id,
            ((am_addr << 5) | (new_piconet_id << 11)),
            BB_read_baseband_register(CONNECTOR_REGISTER),0,0);
        }
#endif
        self_device_role = SLAVE;
        /*
        Check if there is a master conncetion in the same am_addr already.
        Otherwise, clear the corresponding active bit of lut:1-7. */
        {
            UCHAR master_piconet_id;
            UCHAR active_bit = TRUE;

            master_piconet_id = lc_get_master_piconet_id();

            if (master_piconet_id == SCA_PICONET_INVALID)
            {
                active_bit = FALSE;
            }
            else
            {
                UCHAR lcl_lut_index;
                lcl_lut_index = lc_get_lut_index_from_phy_piconet_id(
                                    am_addr, master_piconet_id);

                if (lut_ex_table[lcl_lut_index].index_in_CE == INVALID_CE_INDEX)
                {
                    active_bit = FALSE;
                }
            }

            if (active_bit == FALSE)
            {
                UINT16 lcl_address, lcl_value;
                lcl_address = 0x70 + (am_addr << 2) + 2;

                lcl_value = BB_read_baseband_register(lcl_address);

                lcl_value = (UINT16) ( lcl_value & (~ACTIVE_BIT));

                BB_write_baseband_register(lcl_address, lcl_value);

                RT_BT_LOG(GRAY, LC_ISR_5683, 1, lcl_address);
            }
        }
    }

#ifndef _RESET_HW_LINK_INFO_TO_INIT_STATE_
    OR_val_with_bb_reg_macro(ex_lut->upper_lut_address,(ACTIVE_BIT | FLOW_BIT));
#else
    UINT16 reg_value;
    reg_value = BB_read_baseband_register(ex_lut->upper_lut_address);

    /* Initialize flow = 1 and active bit */
    reg_value |= ACTIVE_BIT | FLOW_BIT;

    /* Initialize arqn = 0, seqn = 0, ptt = 0 */
    /* (NOTE: (20140305 cch) seqn is useless by writed here) */
    reg_value &= ~0x8006;

    BB_write_baseband_register(ex_lut->upper_lut_address, reg_value);
#endif

    RT_BT_LOG(WHITE, MSG_BB_HLC_ISR, 8,
                    pre_dev_state, lmp_self_device_data.lc_cur_dev_state,
                    phy_piconet_id, new_piconet_id, am_addr, self_device_role,
                    BB_read_baseband_register(ex_lut->lower_lut_address),
                    BB_read_baseband_register(ex_lut->upper_lut_address));

    phy_piconet_id = new_piconet_id;

    if ((result = LMP_GET_CE_INDEX_FROM_AM_ADDR_PPI(
                   (UCHAR) am_addr, phy_piconet_id, &ce_index)) == API_SUCCESS)
    {
#ifdef _SUPPORT_CSB_RECEIVER_
        /* If CSB scan is not enabled or ce_index != csb scan ce_index,
           then start tpoll.*/
        if ((!bt_3dd_var.csb_rx_param.enable) ||
                 (bt_3dd_var.csb_rx_param.ce_index != ce_index))
#endif
        {
        BB_start_tpoll((UCHAR)am_addr,
                       lmp_connection_entity[ce_index].Tpoll, phy_piconet_id);
        }
    }
    else
    {
        RT_BT_LOG(GRAY, LC_ISR_5701, 0, 0);
        lc_update_scatternet_state_deletion(phy_piconet_id);

        return;
    }

    lc_init_seqn_scatternet((UCHAR)am_addr, ex_lut, phy_piconet_id);

    lmp_self_device_data.number_of_hlc++;

    sig_send.type = LC_HLC_SIGNAL;
    sig_send.param = (OS_ADDRESS)((UINT32)(am_addr));

    sig_send.ext_param = (OS_ADDRESS)((UINT32)(phy_piconet_id));

    if (OS_SEND_SIGNAL_TO_TASK (lc_rx_task_handle, sig_send) != BT_ERROR_OK)
    {
#ifdef ENABLE_LOGGER_LEVEL_2
        LC_LOG_ERROR(LOG_LEVEL_LOW, "Error sending signal to lmp_task.");
#endif
    }

    lmp_self_device_data.lc_no_of_connections[phy_piconet_id]++;

#ifdef _USE_SNIFF_NO_ACL_COUNT_REGISTER
#ifdef COMPILE_SNIFF_MODE
    /* Reduce sniff no-acl to 3 temporarily. */
    BB_write_baseband_register(SNIFF_NO_ACL_COUNT_REGISTER, 0x03);
    lmp_self_device_data.no_acl_reduced_flag = TRUE;
    lmp_self_device_data.no_acl_ce_index = ce_index;
#endif
#endif

#ifdef _RESET_HW_LINK_INFO_TO_INIT_STATE_
    /* reset possible encrption state */
    BB_encryption_control(am_addr, phy_piconet_id, BB_ENC_TX_DISBALE_RX_DISABLE);
#endif
#ifdef _DAPE_TEST_FIX_NEW_PLATFORM_MUTE_ACL004_DISCONN
#ifdef _CCH_SC_ECDH_P256_
    BB_write_sc_cam_ini(ce_index);
#endif
#endif

    return;
}

extern UCHAR g_lc_scan_slot_timer_in_use;
#ifdef COMPILE_ROLE_SWITCH
/**
* Resets the role switch data structures and brings back the system to its
* old state.
*
* \param None.
*
* \return None.
*/
SECTION_ISR_LOW void lmp_handle_switch_newconn_timeout_in_scatternet(void)
{
    UCHAR old_am_addr;
    UCHAR new_am_addr;
    UCHAR old_piconet_id;
    UCHAR new_piconet_id;
    UCHAR old_lut_index;
    UCHAR new_lut_index;
    UINT16 ce_index;
    UINT16 read;
    UINT16 reg_addr;
    LMP_CONNECTION_ENTITY *ce_ptr;

    old_am_addr = lmp_role_switch_data.old_am_addr;
    new_am_addr = lmp_role_switch_data.new_am_addr;

    old_piconet_id = lmp_role_switch_data.old_piconet_id;
    new_piconet_id = lmp_role_switch_data.new_piconet_id;

    old_lut_index = lmp_role_switch_data.old_lut_index;
    new_lut_index = lmp_role_switch_data.new_lut_index;

    ce_index = lmp_role_switch_data.ce_index ;
    ce_ptr = &lmp_connection_entity[ce_index];
#ifndef _ENABLE_COMPILE_NESTED_PAUSE_RESUME_
    lc_pause_schedule_pkts = 0x0;
#endif
    lc_cont_crc_tx_cnt[old_lut_index] = 0x00;

#ifdef COMPILE_AFH_HOP_KERNEL
    ce_ptr->afh_mode = lmp_role_switch_data.old_afh_mode;
    if (lmp_role_switch_data.old_afh_mode == AFH_ENABLE)
    {
        BB_write_afh_map(old_am_addr, old_piconet_id, ce_ptr->afh_map);
    }
    else
    {
        BB_disable_afh(old_am_addr, old_piconet_id);
    }
#endif

    switch(lmp_mss_state)
    {
        case LMP_MSS_M_TO_S_WAITING_FOR_FHS:
            AND_val_with_bb_reg_macro(CONNECTOR_REGISTER, (~BB_SET_TDD_BIT));

            BB_write_baseband_register(
                        lut_ex_table[old_lut_index].lower_lut_address,
                        LC_MASTER_DEFAULT_PACKET_TYPE);

            break;

        case LMP_MSS_M_TO_S_FHS_RECD:
            AND_val_with_bb_reg_macro(CONNECTOR_REGISTER, (~BB_SET_TDD_BIT));

            lmp_slave_unuse_am_addr_ppi(new_am_addr, new_piconet_id);

            BB_write_baseband_register(CONNECTOR_REGISTER,
                          ((old_am_addr << 5) | (old_piconet_id << 11) ) );
            break;

        case LMP_MSS_S_TO_M_SENT_FHS_RECD_ID:
            /* Make the new am_address deactive. */
            BB_write_baseband_register(
                lut_ex_table[new_lut_index].lower_lut_address,
                LC_INVALID_PACKET_TYPE);
            AND_val_with_bb_reg_macro(
                lut_ex_table[new_lut_index].upper_lut_address,
                (UINT16) (~ACTIVE_BIT));

            /* Make the old am_address active. */
            OR_val_with_bb_reg_macro(lut_ex_table[old_lut_index].upper_lut_address,
                                     ACTIVE_BIT);

            /* Write default pkt in LUT. */
            BB_write_baseband_register(
                lut_ex_table[old_lut_index].lower_lut_address,
                LC_SLAVE_DEFAULT_PACKET_TYPE);

            read = BB_read_baseband_register(CONNECTOR_REGISTER);
            read &= 0xE71F ;
            read = (UINT16)(read | (old_am_addr << 5) | (old_piconet_id << 11) );
            BB_write_baseband_register(CONNECTOR_REGISTER, read);

            reg_addr = reg_PICONET_INFO[old_piconet_id];

            BZ_REG_S_PICONET_INFO reg_pn_info;
            *(UINT16*)&reg_pn_info = BB_read_baseband_register(reg_addr);
            reg_pn_info.lt_addr = old_am_addr;
            reg_pn_info.master = FALSE;
            BB_write_baseband_register(reg_addr, *(UINT16*)&reg_pn_info);

            lmp_slave_unuse_am_addr_ppi(new_am_addr, new_piconet_id);

            break;

        case LMP_MSS_S_TO_M_SENT_FHS_PKT :
            lmp_slave_unuse_am_addr_ppi(new_am_addr, new_piconet_id);
            AND_val_with_bb_reg_macro(CONNECTOR_REGISTER, (~BB_SET_TDD_BIT));
            break;

        default :
            /* Invalid State. */
            lmp_set_mss_state(LMP_MSS_INIT);
            return;
    }

    lmp_set_mss_state(LMP_MSS_INIT);

    /* Set ACTIVE_BIT in old LUT. */
    OR_val_with_bb_reg_macro(lut_ex_table[old_lut_index].upper_lut_address,
                             ACTIVE_BIT);

    /* Clear ACTIVE_BIT in new LUT. */
    AND_val_with_bb_reg_macro(lut_ex_table[new_lut_index].upper_lut_address,
                              (~ACTIVE_BIT) );

#if 0
    BB_start_tpoll(old_am_addr, ce_ptr->Tpoll, old_piconet_id);
#else
    lc_start_tpoll_on_all_connections();
#endif

#ifdef COMPILE_AFH_HOP_KERNEL
    if (lmp_role_switch_data.old_afh_mode == AFH_ENABLE)
    {
        BB_write_afh_map(old_am_addr, old_piconet_id, ce_ptr->afh_map);
        ce_ptr->afh_mode = AFH_ENABLE;
    }
    else
    {
        BB_disable_afh(old_am_addr, old_piconet_id);
        ce_ptr->afh_mode = AFH_DISABLE;
    }
#endif

    LC_RESET_PTT_BIT_IN_LUT(lut_ex_table[new_lut_index].upper_lut_address);

    if (ce_ptr->ptt_status == LMP_PTT_ENABLED)
    {
        LC_SET_PTT_BIT_IN_LUT(lut_ex_table[old_lut_index].upper_lut_address);
    }
    else
    {
        LC_RESET_PTT_BIT_IN_LUT(lut_ex_table[old_lut_index].upper_lut_address);
    }

    lmp_slave_use_am_addr_ppi(old_am_addr, old_piconet_id, ce_index);

    lmp_handle_role_switch_failure(ce_index, ROLE_SWITCH_FAILED);

    if (g_lc_scan_slot_timer_in_use == 0x1)
    {
        g_lc_scan_slot_timer_in_use = 0x0;

        /* Configure & Retrieve the scan */
        lc_handle_scan_mode_command();
    }
    else
    {
        lc_check_and_enable_scans_in_scatternet();
    }

#ifdef _USE_SNIFF_NO_ACL_COUNT_REGISTER
#ifdef COMPILE_SNIFF_MODE
    /* Restore sniff-no-acl. */
    {
        OS_SIGNAL signal;

        signal.type = LMP_SNIFF_NO_ACL_RESTORE_SIGNAL;
        OS_SEND_SIGNAL_TO_TASK(lmp_task_handle, signal);

        lmp_self_device_data.no_acl_reduced_flag = FALSE;
        lmp_self_device_data.no_acl_ce_index = 0xFF;
    }
#endif
#endif

    return;
}

#endif /* COMPILE_ROLE_SWITCH */

/**
* Sets the state of lmp_mss_state variable.
*
* \param status The new state of the variable.
*
* \return None.
*/
INLINE void lmp_set_mss_state(UCHAR status)
{
    lmp_mss_state = status;

    return;
}

#ifdef COMPILE_SNIFF_MODE
/**
* Calculates the nearest sniff anchor point, after checking all the
* connections in sniff mode. This is used to avoid starting of transmission
* of a CRC packet a few slots before the sniff anchor point. Make sure that
* this function is called from interrupt context, for in critical section
* from tasks. All these instants are in native clock domain, so will not
* match with the negotiated LMP values for slave connections.
*
* \param None.
*
* \return None.
*/
SECTION_ISR_LOW void lc_update_next_sniff_instant(void)
{
    UINT32 count;
    LMP_CONNECTION_ENTITY *ce_ptr;

    UINT32 least_instant = 0xFFFFFFFF;
    UINT32 lcl_instant;

    for (count = 0; count < LMP_MAX_CE_DATABASE_ENTRIES; count++)
    {
        ce_ptr = &lmp_connection_entity[count];

        if ((ce_ptr->entity_status == ASSIGNED) &&
                (ce_ptr->in_sniff_mode == TRUE))
        {
            lcl_instant = ce_ptr->next_instant_in_nat_clk;

            if (lcl_instant != 0xFFFFFFFF)
            {
                if (least_instant > lcl_instant)
                {
                    least_instant = lcl_instant;
                }
            }
        }
    }

    if (least_instant != 0xFFFFFFFF)
    {
        lmp_self_device_data.next_sniff_instant = least_instant;
    }

    return;

}
#endif /* COMPILE_SNIFF_MODE */

SECTION_ISR_LOW void lc_log_data_for_channel_assessment(BB_RX_PARAMS *rx_param, UINT8 is_esco)
{
    UCHAR am_addr;
    UCHAR piconet_id;
    UINT16 ce_index = 0;
    UINT32 lcl_rx_pkt_type;
    UCHAR weightage = 0;
    BZ_REG_S_RX_STATUS *rx_status = &rx_param->rx_status;
    BZ_REG_S_RX_PL_HDR *pl_hdr = &rx_param->pl_hdr;

    /* todo: Decide whether to process here, or to post-signal and process
     * in lower priority task. */

#ifdef _ROM_CODE_PATCHED_
    if (rcp_lc_log_data_for_channel_assessment != NULL)
    {
        if ( rcp_lc_log_data_for_channel_assessment((void*)rx_param, is_esco))
        {
            return;
        }
    }
#endif

    do
    {
        if (is_esco)
        {
            lcl_rx_pkt_type = 0x01 << rx_status->pkt_type;
            if (lcl_rx_pkt_type & BB_RX_ISR_ESCO_DATA_PKT)
            {
#ifdef _CCH_SC_ECDH_P256_MIC_FLAG
                if ((!rx_status->crc_err) && (!pl_hdr->mic_err))
#else
                if (!rx_status->crc_err)
#endif
                {
                    weightage = AFH_GOOD_CRC_PKT_WEIGHT;
                }
                else
                {
                    /* Recd crc error pkt. */
                    weightage = AFH_CRC_ERROR_PKT_WEIGHT;
                }
                break;
            }

            if (lcl_rx_pkt_type & (BB_RX_ISR_POLL | BB_RX_ISR_NULL))
            {
                /* Recd correct poll-null pkt. */
                weightage = AFH_GOOD_POLL_NULL_PKT_WEIGHT;
                break;
            }
            return;
        }

        /* ignore invalid rx status or receive ID packet or EIR packet */
        if (!rx_status->rx_pkt || rx_status->ar_id || rx_status->eir)
        {
            break;
        }

        /* Check for HEC-error. */
        if (rx_status->hec_err)
        {
            /* HEC error code. */
            weightage = AFH_HEC_ERROR_PKT_WEIGHT;

            /* we should break now.
               following information is not reliable - austin  */

#ifdef _BRUCE_TEST_PLC_PKT_STATUS
            plc_pkt_status_var.g_plc_hec_err++;
#ifdef _BRUCE_DEBUG_PORT
            UINT16 read;
            read=BB_read_baseband_register(0x25c); // 0x25c[1]=1; g_plc_hec_err
            read |= (UINT16)BIT1;
            BB_write_baseband_register(0x25c,read);
#endif
#endif

            break;
        }

        am_addr = rx_param->am_addr;
        piconet_id = rx_param->phy_piconet_id;
        lcl_rx_pkt_type = rx_param->lcl_rx_pkt_type;

        /* Check for a valid connection. If there is no connection for
         * these parameters, then ignore the data. */
        if (LMP_GET_CE_INDEX_FROM_AM_ADDR_PPI(am_addr, piconet_id, &ce_index)
                != API_SUCCESS)
        {
            return;
        }

        if (lcl_rx_pkt_type & BB_RX_ISR_CRC_PKT)
        {

#ifdef _CCH_SC_ECDH_P256_MIC_FLAG
            if ((!rx_status->crc_err) && (!pl_hdr->mic_err))
#else
            if (!rx_status->crc_err)
#endif
            {
                weightage = AFH_GOOD_CRC_PKT_WEIGHT;
            }
            else
            {
                /* Recd crc error pkt. */
                weightage = AFH_CRC_ERROR_PKT_WEIGHT;
            }
            break;
        }

        if (pl_hdr->sco_rx)
        {
            /* Recd sco pkt in sco slot */
            weightage = AFH_SCO_PKT_WEIGHT;
            break;
        }

        if (lcl_rx_pkt_type & (BB_RX_ISR_POLL | BB_RX_ISR_NULL))
        {
            /* Recd correct poll-null pkt. */
            weightage = AFH_GOOD_POLL_NULL_PKT_WEIGHT;
            break;
        }
    }
    while (0);

    if (weightage != 0)
    {
        lmp_update_channel_quality(rx_param->rx_channel, rx_param->rssi, weightage);
        new_afh_rx_info.ce_index = ce_index;
        new_afh_rx_info.is_from_wifi = FALSE;
        new_afh_rx_info.is_sco_rx = (pl_hdr->sco_rx) ? TRUE : FALSE;
    }
}

