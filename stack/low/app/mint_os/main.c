/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file main.c
 *  Application main program.
 *
 * \author Santhosh kumar M
 * \date 2008-04-09
 */

enum { __FILE_NUM__= 100 };

/* ========================= Include File Section ========================= */
#include "FreeRTOS.h"
#include "task.h"
#include "bz_bluetooth.h"
#include "platform.h"
#include "bt_fw_os.h"
#include "UartPrintf.h"
#include "uart.h"
#include "mem.h"
#include "led_debug.h"
#include "logger.h"
#include "bt_fw_config.h"
#include "lmp_defines.h"
#include "bt_fw_hci_internal.h"
#include "initial.h"
#include "bzdma.h"
#include "mint_os_buffer_internal.h"
#include "rlx4181.h"
#include "timer.h"
#include "power_control.h"
#include "dma_usb.h"
#ifdef LE_MODE_EN
#include "le_ll.h"
#endif
#ifdef _ENABLE_MAILBOX_
#include "mailbox.h"
#endif
#ifdef _SUPPORT_WL_BT_SCOREBOARD_
#include "scoreboard.h"
#endif
#ifdef _ENABLE_EFUSE_
#include "efuse.h"
#endif
#include "gpio.h"
#include "bz_fw_isoch.h"
#include "spi.h"
#include "pta.h"
#include "h5.h"
#include "platform.h"
#include "hci_vendor_defines.h"
#ifdef _DAPE_TEST_LE_DBG
#include "le_ll_driver.h"
#endif

#include "crypto.h"

#ifdef PTA_EXTENSION
#include "pta_meter.h"
#endif
#include "lc_internal.h"

#include "bt_3dd.h"
#ifdef CONFIG_TV_FEATURE
#include "tv.h"
#endif
#ifdef CONFIG_FM
#include "fm.h"
#endif

#if defined(_BT_ONLY_) || defined(_NEW_BTON_DESIGN_AFTER_RTL8821B_TC_)
#include "new_io.h"
#include "efuse.h"
#endif

#include "plc.h"

#ifdef MWS_ENABLE
#include "mws_imp.h"
#include "mws.h"
#endif
#ifdef _SUPPORT_TIMER1_SHUTDOWN_LOG_
extern UINT16 g_u16shutdown_log_timer_cnt;
#endif

#ifdef TEST_MODEM_PSD
#include "lmp_ch_assessment.h"
#endif
#ifdef _FIX_CROSSBAR_ERROR_DEBUG_
#include "mint_os_timer_internal.h"
#include "bz_utils.h"
extern SECTION_SRAM OS_TIMER dbg_os_timer;
#endif

#ifdef _SUPPORT_INFO_FROM_SYSTEM_ON_
#include "system_on.h"
extern UINT8 g_chip_id;
extern UINT8 g_chip_ver;
#ifdef _SUPPORT_BT_CTRL_FM_
UINT8 g_u8support_fm = 0;
#endif
#endif

/* ====================== Macro Declaration Section ======================= */


/* ==================== Structure Declaration Section ===================== */


/* ===================== Variable Declaration Section ===================== */
#ifdef _FONGPIN_TEST_MUX_DEV_WAKEUP_HOST_BY_GPIO13_
extern BTON_POW_CTRL_REG_S_TYPE pow_ctrl_int_rec;
#endif

extern UINT32 bm_task_queue_full;
extern UINT16 task_sig_lost_history[OS_TASK_SIG_LOST_MAX_CNT];
extern UINT8 task_sig_lost_cnt;

#ifdef _SUPPORT_AUTO_DETACH_LINK_
extern OS_SIGNAL catch_os_signal;
#endif

#ifdef _ENABLE_MAILBOX_
UINT8 is_wifi_alive = TRUE;
#else
UINT8 is_wifi_alive = FALSE;
#endif

UINT32 cpu_clk;

#ifndef IS_BTSOC
#ifdef _ROM_CODE_PATCHED_
UINT32 rom_code_patch_start_address = ROM_CODE_PAGE_START_ADDRESS;
UINT8 rom_code_patch_block_index = 0;
#endif
#endif

UINT32 g_timer2_init_value = 0;

#ifdef _BRUCE_WDG_TIMEOUT_DEBUG
SECTION_SRAM UINT8 patch_in_isr;
SECTION_SRAM UINT8 patch_isr_num;
SECTION_SRAM UINT8 patch_in_cpu_idle;
SECTION_SRAM UINT16 patch_signal_id;
SECTION_SRAM UINT32 patch_isr_cause;
SECTION_SRAM UINT32 patch_isr_cnts_during_one_callback;
SECTION_SRAM UINT32 patch_last_epc;
SECTION_SRAM UINT32 patch_ra;
#endif

#ifdef _RTK8723_UART_INIT_ // for test only
BAUDRATE_FORMAT_TYPE  hci_uart_reset_init_RTL8723(UINT16 re_init_flag);
#endif

#if defined(_IS_ASIC_) && defined(_OTP_ADD_SYS_INIT_PARAM_)
extern void sys_init_by_otp(void);
#endif

#ifdef _CCH_ISSC_VENDOR_TEST_
extern ISSC_SEC_STRUCT issc_sec;
#endif

void dbg_msg_trigger_one_shot_event(UINT32 log_msg_type, UINT32 time_ms);

#ifdef _INQRES_DBG_CCH_
const UINT8 afh_is_tx[79] = {
     50, 255,  58, 255,  64, 255,  72, 255,  66, 255,   /* CH 0~9 */
     74, 255, 255, 255, 255, 255, 255, 255, 255, 255,   /* CH 10~19 */
    255, 255, 255, 255, 255, 255, 255,  12, 255,  20,   /* CH 20~29 */
    255,  14, 255,  22, 255,  28, 255,  36, 255,  30,   /* CH 30~39 */
    255,  38, 255,  16, 255,  24, 255,  18, 255,  26,   /* CH 40~49 */
    255,  32, 255,  40, 255,  34, 255,  42, 255,  44,   /* CH 50~59 */
    255,  52, 255,  46, 255,  54, 255,  60, 255,  68,   /* CH 60~69 */
    255,  62, 255,  70, 255,  48, 255,  56, 255};       /* CH 70~78 */
#endif

extern OS_POOL_MGR                 pool_mgr;

TimerHandle_t dbg_tid_timer = NULL;

#ifdef _ROM_CODE_PATCHED_
PF_ROM_CODE_PATCH_VOID rcp_main_hci_uart_init_func = NULL;
PF_ROM_CODE_PATCH_VOID rcp_hw_reg_reinit_func = NULL;
#endif

#ifndef _MOVE_SOME_FW_SRAM_SIGNATURES_TO_BTON_REG_
SECTION_SRAM UINT32 g_sram_rom_code_patch;
SECTION_SRAM UINT32 g_sram_patch_end_event;
SECTION_SRAM UINT32 g_sram_check_wdg_to_reason;
SECTION_SRAM UINT32 g_sram_check_h5_linkreset;
#endif

#ifdef _YL_RTL8723A_B_CUT
SECTION_SRAM UINT32 g_sram_patch_uart_status;
#endif

#ifdef _YL_TEST_NEW_MODEM_SRAM_DEBUG
#define DBG_MSG_EXPIRE_TIME     5000
#else
#define DBG_MSG_EXPIRE_TIME     10000
#endif

#ifdef _ROM_CODE_PATCHED_
PF_ROM_CODE_PATCH_FUNC rcp_dbg_msg_timer_callback = NULL;
#endif


#ifdef TEST_MODEM_PSD
TIMER_ID psd_test_timer = OS_INVALID_HANDLE;
#define PSD_TEST_TEIMER_EXPIRE_TIME     10000
UINT32 dbg_count_psd = 0;
#define _TEST_YS_MODEM_PSD

 // #ifdef _TEST_YS_MODEM_PSD
       ALIGN(16) SECTION_SRAM MODEM_PSD_REPORT_ELEMENT_S_TYPE patch_g_modem_psd_report_array[79];
 // #endif
void psd_test_timer_trigger_one_shot_event(UINT32 log_msg_type, UINT32 time_ms);
#endif


UINT32 g_dbg_count = 0;
void dbg_msg_timer_callback(TimerHandle_t timer_handle)
{
    UINT8 log_msg_type = (UINT8)((UINT32)pvTimerGetTimerID(timer_handle));

    /* free software timer */
    if (dbg_tid_timer != NULL)
    {
        OS_DELETE_TIMER(&dbg_tid_timer);
    }

#ifdef _ROM_CODE_PATCHED_
    if (rcp_dbg_msg_timer_callback != NULL)
    {
        if (rcp_dbg_msg_timer_callback((void *)log_msg_type))
        {
            return;
        }
    }
#endif

    switch (log_msg_type)
    {
        case 0:
            break;

    case 1:
        dbg_msg_trigger_one_shot_event(1, DBG_MSG_EXPIRE_TIME);

        bz_auth_handle_timeout_for_auto_epr(DBG_MSG_EXPIRE_TIME/1000);

#ifndef _DAPE_MODEM_SRAM_DBG_VER2
#ifdef _YL_NEW_MODEM_SRAM_DEBUG
        rtl8821_btrf_modem_sram_debug_test_log(g_modem_sram_debug_array);
#endif
#endif

        if (g_dbg_count & 0x01)
        {
#ifdef _RTK_PTA_TEST_
            pta_test_in_dbg_timer_callback();
#endif
#ifdef _DAPE_CHK_WHCK_EIR
            lmp_self_device_data.page_scan_interval = 0x200;
            lmp_self_device_data.page_scan_window = 0x12;
            /* (dape 20131018) do not make inq scan interval too short which will
            result in failure in WHCK P2P hct2.4.01.*/
            lmp_self_device_data.inquiry_scan_interval = 0x600;
            lmp_self_device_data.inquiry_scan_window = 0x12;
            lmp_self_device_data.inquiry_scan_type = HCI_INTERLACED_INQ_SCAN;
            RT_BT_LOG(WHITE, DAPE_TEST_LOG525, 6,
                BB_read_baseband_register(PAGE_SCAN_INTERVAL_REGISTER),
                BB_read_baseband_register(PAGE_SCAN_WINDOW_REGISTER),
                BB_read_baseband_register(INQ_SCAN_INTERVAL_REGISTER),
                BB_read_baseband_register(INQ_SCAN_WINDOW_REGISTER),
                lmp_self_device_data.inquiry_scan_type,0
                );
#endif

            RT_BT_LOG(GRAY, MAIN_MSG_KEEP_ALIVE2, 18, g_dbg_count >> 1,
                                                    OS_get_free_buffer(0),
                                                    OS_get_free_buffer(1),
                                                    OS_get_free_buffer(2),
                                                    OS_get_free_buffer(3),
                                                    OS_get_free_buffer(4),
                                                    OS_get_free_buffer(5),
                                                    OS_get_free_buffer(6),
                                                    OS_get_free_buffer(7),
                                                    OS_get_free_buffer(8),
                                                    OS_get_free_buffer(9),
                                                    OS_get_free_buffer(10),
                                                    OS_get_free_buffer(11),
                                                    OS_get_free_buffer(12),
                                                    BB_read_instruction_status(0),
                                                    BB_read_instruction_status(1),
                                                    BB_read_instruction_status(2),
                                                    BB_read_instruction_status(3));

#if 1
            UINT32 sche_info[4][3];
            UINT8 i;
            LC_PICONET_SCHEDULER *lcps;

            for (i = 0; i < 4; i++)
            {
                lcps = &lc_piconet_scheduler[i];

                /* === piconet scheduler queue info ===
                   bit[0]: wptr
                   bit[1]: rptr
                   bit[3:2]: lc_allowed_pkt_cnt
                   bit[5:4]: donot_schedule_pkt
                   bot[7:6]: wait_fpr_clear_pkt_count */
                sche_info[i][0] = lcps->wptr |
                                   (lcps->rptr << 1) |
                                   (lcps->lc_allowed_pkt_cnt << 2) |
                                   (lcps->donot_schedule_pkt << 4) |
                                   (lcps->wait_for_clear_pkt_count << 6);

                /* === piconet scheduler pkt0 info ===
                   bit[3:0]: ce_index
                   bit[7:4]: tx_sttaus
                   bit[11:8]: (lch >> 8)
                   bit[31:12]: fw_sent_time */
                sche_info[i][1] =
                          (lcps->lc_scheduled_pkt_info[0].ce_index & 0x0f) |
                          (lcps->lc_scheduled_pkt_info[0].tx_status << 4) |
                          ((lcps->lc_scheduled_pkt_info[0].lch >> 8) << 8) |
                          (lcps->lc_scheduled_pkt_info[0].fw_sent_time << 12);

                /* === piconet scheduler pkt1 info ===
                   bit[3:0]: ce_index
                   bit[7:4]: tx_sttaus
                   bit[11:8]: (lch >> 8)
                   bit[31:12]: fw_sent_time */
                sche_info[i][2] =
                          (lcps->lc_scheduled_pkt_info[1].ce_index & 0x0f) |
                          (lcps->lc_scheduled_pkt_info[1].tx_status << 4) |
                          ((lcps->lc_scheduled_pkt_info[1].lch >> 8) << 8) |
                          (lcps->lc_scheduled_pkt_info[1].fw_sent_time << 12);
            }

            RT_BT_LOG(GRAY, MSG_LOG_SCHEDULE_LOG_NEW, 14,
                        sche_info[0][0], sche_info[0][1], sche_info[0][2],
                        sche_info[1][0], sche_info[1][1], sche_info[1][2],
                        sche_info[2][0], sche_info[2][1], sche_info[2][2],
                        sche_info[3][0], sche_info[3][1], sche_info[3][2],
                        RD_U32_BZDMA_REG(BZDMA_REG_TX_CMD(2)),
                        RD_U32_BZDMA_REG(BZDMA_REG_TX_CMD(3)));
#endif
            RT_BT_LOG(GRAY, DAPE_TEST_LOG558, 2,
                        RD_U32_BZDMA_REG(BZDMA_REG_ACL_RXFIFO_PTR),
                        RD_U32_BZDMA_REG(BZDMA_REG_SCO_RXFIFO_PTR));

            if (bm_task_queue_full != 0)
            {
                RT_BT_LOG(RED, MSG_TASK_QUEUE_FULL_BM, 10,
                            bm_task_queue_full,
                            task_sig_lost_cnt,
                            task_sig_lost_history[0],
                            task_sig_lost_history[1],
                            task_sig_lost_history[2],
                            task_sig_lost_history[3],
                            task_sig_lost_history[4],
                            task_sig_lost_history[5],
                            task_sig_lost_history[6],
                            task_sig_lost_history[7]);

                task_sig_lost_cnt = 0;
                memset(task_sig_lost_history, 0, OS_TASK_SIG_LOST_MAX_CNT*2);
                bm_task_queue_full = 0;
            }

#ifdef _UART_BAUD_ESTIMATE_
            if (g_fun_interface_info.b.bt_interface == UART_INTERFACE)
            {
                hci_uart_baud_est_log();
            }
#endif


#if defined(_UART_H5) && defined(_YL_H5_TEST)
            hci_uart_h5_test_at_keep_alive_event();
#endif
#if 0
#ifdef _YL_NEW_MODEM_SRAM_DEBUG
            rtl8821_btrf_modem_sram_debug_test_log(g_modem_sram_debug_array);
#endif
#endif
#ifdef _YL_TEST_NEW_MODEM_PI_ACCESS_
            lc_modem_pi_access_test_log(g_dbg_count);
#endif

#ifdef _YL_TEST_UART_BAUD_ESTIMATE_
            hci_uart_baud_est_test_log(g_dbg_count);
#endif


#ifdef _CCH_ISSC_VENDOR_TEST_

            // Send Signal to Background Task
            UINT8 state = 1;

            OS_SIGNAL signal;
            signal.param = (OS_ADDRESS)( (UINT32) state);

            signal.type = CH_AS_TASK_ISSC_SEC;
            OS_ISR_SEND_SIGNAL_TO_TASK(hci_ch_as_task_handle, signal);

            memset(&issc_sec, 0, sizeof(ISSC_SEC_STRUCT));

#endif
        }
        else
        {
#ifdef LE_MODE_EN
#ifdef _DAPE_PRINT_MORE_LOG_INFO
            if (IS_BT40)
            {
                RT_BT_LOG(GRAY, DAPE_TEST_LOG543, 13,
                        sleep_mode_param.lps_wakeup_count,
                        RD_LE_REG(LE_REG_INST_RPT),

                        ll_manager.conn_unit.connection_cnts,
                        ll_manager.conn_unit.bmActiveHandle,

                        ll_manager.adv_unit.enable,
                        ll_manager.adv_unit.interval,

                        ll_manager.scan_unit.enable,
                        ll_manager.scan_unit.interval,
                        ll_manager.scan_unit.window,
                        ll_manager.scan_unit.filter_duplicate,

                        ll_manager.initiator_unit.enable,
                        ll_manager.initiator_unit.scan_interval,
                        ll_manager.initiator_unit.scan_window);

                //UINT8 entry = ll_manager.event_manager.entry;

                RT_BT_LOG( GRAY, DAPE_TEST_LOG539, 9,
                        ll_manager.conn_unit.master,
                        ll_manager.conn_unit.handle[0].ce_interval,
                        ll_manager.conn_unit.handle[0].conn_counter,
                        ll_manager.conn_unit.handle[0].ce_count_restart_sup,
                        ll_manager.conn_unit.handle[0].supervision_to,
                        ll_manager.conn_unit.handle[1].ce_interval,
                        ll_manager.conn_unit.handle[1].conn_counter,
                        ll_manager.conn_unit.handle[1].ce_count_restart_sup,
                        ll_manager.conn_unit.handle[1].supervision_to);

            }
#endif
#endif
        }

        g_dbg_count++;
        break;

    default:
        break;
    }
}

void dbg_msg_trigger_one_shot_event(UINT32 log_msg_type, UINT32 time_ms)
{
    if (dbg_tid_timer != NULL)
    {
        OS_DELETE_TIMER(&dbg_tid_timer);
    }

    /* Create a tid timer */
    OS_CREATE_TIMER(ONESHOT_TIMER, &dbg_tid_timer, dbg_msg_timer_callback,
                    (void *)((UINT32)log_msg_type), 0);

    OS_START_TIMER(dbg_tid_timer, time_ms);
}


#ifdef TEST_MODEM_PSD

void psd_test_timer_callback(TIMER_ID timer_handle, void *index)
{
    //static UINT32 dbg_count_psd = 0;
    UINT8 log_msg_type = (UINT8)((UINT32)index);

    /* free software timer */
    if (psd_test_timer != OS_INVALID_HANDLE)
    {
        OS_DELETE_TIMER(&psd_test_timer);
    }
    dbg_count_psd++;
    psd_test_timer_trigger_one_shot_event(1, PSD_TEST_TEIMER_EXPIRE_TIME);

    //===================================================================
    /* To Add Log and PSD test FW here execute every 10s (PSD_TEST_TEIMER_EXPIRE_TIME (ms))*/

       lc_psd_set_psd_en(0);
       lc_psd_bluewiz_set_parameters(0, 1, 78, PSD_PARA_HALF_SLOT_MODE, 0x3FF);
       //lc_psd_bluewiz_set_parameters(0, 1, 10, PSD_PARA_HALF_SLOT_MODE, 0x3FF);
       lc_psd_set_psd_en(1);
       RT_BT_LOG(GRAY, MDM_LOG_015, 1, BB_read_native_clock());

    //===================================================================

}

void psd_test_timer_trigger_one_shot_event(UINT32 log_msg_type, UINT32 time_ms)
{
    if (psd_test_timer != OS_INVALID_HANDLE)
    {
        OS_DELETE_TIMER(&psd_test_timer);
    }

    /* Create a tid timer */
    OS_CREATE_TIMER((UCHAR)ONESHOT_TIMER, &psd_test_timer,
                    (OS_TIMEOUT_HANDLER)psd_test_timer_callback,
                    (void *)((UINT32)log_msg_type), 0);

    OS_START_TIMER(psd_test_timer, time_ms);
}




#endif

void dbg_rtk_set_debug_port(void)
{
#ifdef LA_DEBUG_PORT_EN
    VENDOR_WRITE(0x30, (VENDOR_READ(0x30)&BIT31) | 0x40);
#ifdef _DAPE_TEST_CSB_RX_DBG
    VENDOR_WRITE(0x30, (VENDOR_READ(0x30)&BIT31) | 0x63);
#endif
#ifdef _TEST_ADAPTIVITY_FUNC_ 
    VENDOR_WRITE(0x30, 0xA0);
#endif
#ifdef _DAPE_TEST_CSB_TX_AFH_DBG
    WR_16BIT_IO(BB_BASE_ADDR, 0x1FE, 0x17);
#else
    WR_16BIT_IO(BB_BASE_ADDR, 0x1FE, 0x14);
#endif
    VENDOR_WRITE(0x32C, 0x02);
#ifdef _YL_H5_UART_DBG_PORT_EN // for debug port
    VENDOR_WRITE(0x30, (VENDOR_READ(0x30)&BIT31) | 0xc0);
    UART_DWORD_WRITE(0x1c, _YL_H5_UART_DBG_PORT);
#endif
#ifdef _YL_H5_DMA_DBG_PORT_EN // for debug port
    VENDOR_WRITE(0x30, (VENDOR_READ(0x30)&BIT31) | 0xe0);
    DMA_DWORD_WRITE(0x80, _YL_H5_DMA_DBG_PORT);
#endif
#ifdef _DAPE_TEST_LE_DBG
    WR_16BIT_IO(BB_BASE_ADDR, 0x1FE, 0x24);
    UINT16 le_reg_ce = RD_LE_REG(0xCE);
    le_reg_ce &= (UINT16)(0x00FF);
    WR_LE_REG(0xCE, le_reg_ce);
#endif
#if defined(_YL_TEST_MODEM_RX_REPORT_BT_GPIO) || \
    defined(_YL_TEST_MODEM_PI_TIMING_BY_BT_GPIO) || \
    defined(_YL_TEST_MODEM_PI_TIMING_BY_BT_GPIO_2) || \
    defined(_DAPE_TEST_BT_ONLY_GPIO_TRIGGER)
    SET_BT_GPIO_MODE();
    SET_BT_GPIO_OUTPUT_3_0(0);
#endif

#ifdef _TEST_SC_BITFILE_DBG_PORT
    VENDOR_WRITE(0x30, 0x40);
    WR_16BIT_IO(BB_BASE_ADDR, 0x1FE, 0x39);
    RT_BT_LOG(YELLOW, CCH_DBG_033, 1, 0x39);
#endif
#endif
}

void pta_function_set(void)
{
#ifdef _RTK_PTA_TEST_
    pta_test_in_main_func();
#endif

#ifdef _ENABLE_RTK_PTA_
    pta_init();
#endif

#ifdef PTA_EXTENSION
    fnPtaHciMeterInit();
#endif
}

void sys_clock_setting(void)
{
    UINT32 U32_dat;

    //Set Uart clock to 40MHz
    UINT32 tmp_val = VENDOR_READ(0x150);
    tmp_val &= 0xfffffffe;
    VENDOR_WRITE(0x150, tmp_val);

#ifndef _BOOT_32K_SWITCH_40M_
    VENDOR_WRITE(0, VENDOR_00_VAL1);
    cpu_clk = 40000000;
#else
    /* 3:40MHz;2:20MHz;1:10MHz;0:5MHz */
#ifdef IS_BTSOC
    cpu_clk = 20000000;
#else
    U32_dat = VENDOR_READ(0x0) & 0x03; //SYS_CLK
    cpu_clk = 40000000 >> (3 - U32_dat);
#endif
#endif /* end of #ifndef _BOOT_32K_SWITCH_40M_ */
}

#ifndef IS_BTSOC
#ifdef _ENHANCED_CPU_SLEEP_WITH_GATED_40MHZ_CLK_
void enhanced_cpu_sleep_func(void)
{
    UINT32 temp;
    UINT8 enh_cpu_sleep = FALSE;

    DEF_CRITICAL_SECTION_STORAGE;

    if (IS_ENHANCED_CPU_SLEEP_MODE)
    {
        MINT_OS_ENTER_CRITICAL();

        do
        {
            UINT32 dword;

            /* check lps, sm, dsm mode */
            if (sleep_mode_param.bb_sm_sts != BB_NORMAL_MODE)
            {
                break;
            }

            /* check bzmda tx */
            if (Bzdma_Manager.bmFreeTxEnt != BZDMA_DEFAULT_TX_FREE_ENTRY_BM)
            {
                break;
            }

            /* check bzdma rx */
            dword = RD_U32_BZDMA_REG(BZDMA_REG_INT_STATUS);
            if (dword & BIT31)
            {
                /* still busy */
                break;
            }

            /* check hci dma tx (NC) */
            /* check hci dma rx (NC, new hw can help take care this) */

            enh_cpu_sleep = TRUE;
        }
        while (0);

        if (enh_cpu_sleep)
        {
            temp = VENDOR_READ(0x00);
            VENDOR_WRITE(0x00, temp | BIT11);
            rlx4081sleep();
            VENDOR_WRITE(0x00, temp & ~BIT11);
        }
        else
        {
            rlx4081sleep();
        }
        MINT_OS_EXIT_CRITICAL();
    }
    else
    {
        rlx4081sleep();
    }
}
#endif
#endif

#ifdef _SUPPORT_MEMORY_TEST_SIMULATION_ONLY_
void mem_test_sim_func(void)
{
    /* to toggle GPIOs to check memory (rom/ram/dmem) in post-sim */
    UINT32 reg_value = VENDOR_READ(0xA0);
    UINT8 i;

    GpioInit();
    reg_value >>= 29;

    bt_led_control(g_wpan_led_num, OUTPUT_HIGH);

    pf_delay(3);

    /* LED toggle behavior :
       toggle 0 time  = rom pass, ram pass, dmem pass
       toggle 1 time  = rom fail, ram pass, dmem pass
       toggle 2 times = rom pass, ram fail, dmem pass
       toggle 3 times = rom fail, ram fail, dmem pass
       toggle 4 times = rom pass, ram pass, dmem fail
       toggle 5 times = rom fail, ram pass, dmem fail
       toggle 6 times = rom pass, ram fail, dmem fail
       toggle 7 times = rom fail, ram fail, dmem fail
     */

    for (i = 0; i < reg_value; i++)
    {
        bt_led_control(g_wpan_led_num, OUTPUT_LOW);
        pf_delay_us(3);
        bt_led_control(g_wpan_led_num, OUTPUT_HIGH);
        pf_delay_us(3);
    }
    pf_delay(3);
}
#endif



#ifdef TEST_MODEM_PSD
void test_modem_psd_init_func(void)
{
    psd_test_timer_trigger_one_shot_event(1, PSD_TEST_TEIMER_EXPIRE_TIME);
    //==========================================================================
    /* some PSD initialization here */

	UINT32 addr_temp;
       UINT32 ii;

        g_efuse_modem_psd_setting_1.d16 = otp_str_data.efuse_modem_psd_setting_1_d16;
        g_efuse_modem_psd_setting_2.d16 = otp_str_data.efuse_modem_psd_setting_2_d16;

       pf_sram_modem_access_enable();   //VENDOR_WRITE(0x30, VENDOR_READ(0x30) | BIT31);

       addr_temp = (UINT32)(patch_g_modem_psd_report_array);
       addr_temp &= 0xFFFF;
       addr_temp = (addr_temp>>3);

	RT_BT_LOG(WHITE, YL_DBG_HEX_1, 1, (UINT32)(patch_g_modem_psd_report_array));
       for (ii = 0; ii<79; ii++)
       {
           patch_g_modem_psd_report_array[ii].d32_array[0]=0xffffffff;
           patch_g_modem_psd_report_array[ii].d32_array[1]=0xffffffff;
           patch_g_modem_psd_report_array[ii].d32_array[2]=0xffffffff;
           patch_g_modem_psd_report_array[ii].d32_array[3]=0xffffffff;
       }

	/* set agc */
	MODEM_REG_S_TYPE modem_reg;
	/* switch to modem page 2 */
       rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x00), TYPE_MODEM, 0x02);

       #if 0
       // fix mp_gain_idx for test
       rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x44), TYPE_MODEM, 0x63cf);
       //rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x44), TYPE_MODEM, 0x401F);
       #endif

       modem_reg.d16 = rtk_read_modem_radio_reg(TRANS_MODEM_REG(0x56), TYPE_MODEM);
       modem_reg.p2reg56.reg_psd_sram_report_addr_base = addr_temp;
       modem_reg.p2reg56.reg_psd_agc_time = 3; //psd_agc_time;    //when reg_psd_agc_time is 0~6: reg_psd_cal_dly should >= reg_psd_agc_time
       rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x56), TYPE_MODEM, modem_reg.d16);

       modem_reg.d16 = rtk_read_modem_radio_reg(TRANS_MODEM_REG(0x58), TYPE_MODEM);
        modem_reg.p2reg58.reg_psd_iq2pwr_opt = 0;
       modem_reg.p2reg58.reg_psd_cal_dly = 3; //psd_cal_dly;
       modem_reg.p2reg58.reg_psd_fix_gain = 0; //psd_fix_gain;
       rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x58), TYPE_MODEM, modem_reg.d16);

       modem_reg.d16 = rtk_read_modem_radio_reg(TRANS_MODEM_REG(0x5a), TYPE_MODEM);
#ifdef _FPGA_WITH_RLE0379_RFE_
       modem_reg.p2reg5a.reg_btm_if_freq_psd = MODEM_PSD_IF_2P5;
#else
       modem_reg.p2reg5a.reg_btm_if_freq_psd = MODEM_PSD_IF_1P4;
#endif
       rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x5a), TYPE_MODEM, modem_reg.d16);

       modem_reg.d16 = rtk_read_modem_radio_reg(TRANS_MODEM_REG(0x5c), TYPE_MODEM);
        modem_reg.p2reg5c.reg_psd_deltaf = 51;  // 1MHz (default)
        modem_reg.p2reg5c.reg_if_bw_psd = 0;
       rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x5c), TYPE_MODEM, modem_reg.d16);

       modem_reg.d16 = rtk_read_modem_radio_reg(TRANS_MODEM_REG(0x5e), TYPE_MODEM);
       modem_reg.p2reg5e.reg_psd_ini_gain_mp = 15; //psd_ini_gain_mp;
       rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x5e), TYPE_MODEM, modem_reg.d16);

       /* switch to modem page 0 */
       rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x00), TYPE_MODEM, 0x00);


    //==========================================================================
}
#endif

void update_bt_efuse_dependancy_valid_content(void)
{
    if (!IS_BT41)
    {
        /* disable MWS -- IS_BT_MWS */
        otp_str_data.bt_func_support_policy &= ~BIT14;

        /* disable secure connection -- IS_BT_SECURE_CON */
        otp_str_data.bt_func_support_policy &= ~BIT13;

        /* disable CSA4 -- IS_BT_CSA4 */
        otp_str_data.bt_func_support_policy &= ~BIT12;
    }
}

void low_stack_init_variable_from_efuse(void)
{
    /*----------------------------------*/
    /* update sw parameters for hci DMA */
    /*----------------------------------*/
#ifdef _NEW_BZDMA_FROM_V8_
    bzdma_supported_le_link_num = LL_MAX_CONNECTION_UNITS;
#ifdef _NEW_BZDMA_FROM_V8_PROGRAMMABLE_CONFIG_SETTING_
    bzdma_supported_le_max_seg_num = 1 << NEW_V8_BZDMA_SUPPORT_MAX_SEGNUM_GRADE_IN_ONE_LINK;
    bzdma_supported_le_max_frag_num = NEW_V8_BZDMA_SUPPORT_MAX_FRAGNUM_IN_ONE_SEGMENT;
    if (bzdma_supported_le_max_frag_num <= 2)
    {
        bzdma_supported_le_max_frag_num = 2;
    }
    else
    {
        bzdma_supported_le_max_frag_num &= ~0x01; /* avoid odd */
    }
#endif
#endif
}

void low_stack_task(void *no_param)
{
    for (;;)
    {
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500)) > 0)
        {
            OS_process_task_signal();
        }
    }
}

xTaskHandle low_task_handle = NULL;
void low_stack_setup_task(void)
{
    xTaskCreate(low_stack_task, "low_stack_task", 512, NULL,
            configMAX_PRIORITIES - 1, &low_task_handle);
}

/*****************************************************************/
/*                      0380 Main Code                           */
/*****************************************************************/
void low_stack_init(void)
{
#ifdef _SUPPORT_MEMORY_TEST_SIMULATION_ONLY_
    mem_test_sim_func();
#endif

#ifndef IS_BTSOC
#ifdef _ENABLE_EFUSE_
    efuse_init();
#endif
#endif

    update_bt_efuse_dependancy_valid_content();

#if !defined(_BT_ONLY_)
#ifdef _SUPPORT_INFO_FROM_SYSTEM_ON_
        g_chip_id = getHwId();
        g_chip_ver = getchipver();
#ifdef _SUPPORT_BT_CTRL_FM_
        if(g_chip_id == CHIP_ID_8703B)
        {
            g_u8support_fm = 1;
        }
#endif
    //#ifdef _FONGPIN_TEST_HCI_SELECTION_
        //RT_BT_LOG(GREEN, PAGE0_REG_HW_INFO, 1, g_chip_id, g_chip_ver);
    //#endif
        //update_lmp_version();
#endif
#endif

    sys_clock_setting();

    low_stack_init_variable_from_efuse();

#ifndef IS_BTSOC
#ifdef _ROM_CODE_PATCHED_
    /* check rom code has patched ? */
    if (IS_ROM_CODE_PATCHED)
    {
        PF_ROM_CODE_PATCH_VOID func;
        func = (PF_ROM_CODE_PATCH_VOID)(ROM_CODE_PAGE_START_ADDRESS + 1); /* mips16 */
        func();
    }
#endif
#endif

#if defined(_IS_ASIC_) && defined(_OTP_ADD_SYS_INIT_PARAM_)
    sys_init_by_otp();
#endif

    //---MINT OS init---
    OS_init(NULL);

    //---LOGGER Module init---
#ifdef ENABLE_LOGGER
    bz_logger_init();

#if defined(_BT_ONLY_) && !defined(_IS_ASIC_)
    {
        /* enable log uart pin mux */
        UINT8 temp = RD_8BIT_SYSON_IO(0x66);
        WR_8BIT_SYSON_IO(0x66, temp | BIT5);
    }
#endif
#endif//ENABLE_LOGGER

    RT_BT_LOG(YELLOW, MSG_CPU_REBOOT, 0, 0);

#ifdef _SUPPORT_INFO_FROM_SYSTEM_ON_
    RT_BT_LOG(GREEN, PAGE0_REG_HW_INFO, 2, g_chip_id, g_chip_ver);
#endif

    //---bluetooth modules init---
    bz_bluetooth_init(); /* pf_hci_transport_init() must before GpioInit() and power_init()*/

#ifdef _SUPPORT_POLLING_BASED_LPM_L1_
    // force lpm NYET first
    set_lpm_l1_token_reponse(L1_NYET);    
#endif

    //---platform HW timer init---
#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
    pf_timer_init(FALSE);
#else
    pf_timer_init();
#endif

    //---GPIO init for radio_switch---
#if defined(_GPIO_POWER_SEQUENCE_ENABLE)
#ifdef _8821A_BTON_DESIGN_
    EFUSE_POW_SETTING_4_S efuse_pow_setting_4;
    *(UINT8*)&efuse_pow_setting_4 = otp_str_data.efuse_pow_setting_4_d8;
    if (efuse_pow_setting_4.fast_gpio_power_on_check2b==0)
#endif
    {
        GpioInit();
    }

#endif//RADIO_SWITCH_SUPPORT

#ifdef _NEW_PLATFORM_XB_INTR_EN
    {
        UINT8 temp = VENDOR_BYTE_READ(0x37);
        VENDOR_BYTE_WRITE(0x37, temp | BIT0);
    }
#endif

#ifdef MINICARD_BT_LED_CONTROL
    BT_LED_WPAN_INIT();
    BT_LED_WPAN_OFF(); /* must after GpioInit() */
#endif

#ifdef _SUPPORT_POWERON_ENABLE_LOG_MUX_
    if(IS_LOG_ENABLED_WHEN_POWER_ON)
    {
#ifdef _SUPPORT_BTGPIO10_AS_LOG1_
        if(SET_LOG_FROM_BTLOG1)
        {
#ifdef _SUPPORT_ENABLE_BTLOG1_WRITE_PAGE0_MUX_        
            if(IS_ENABLE_BTLOG1_WRITE_PAGE0_MUX)
            {
                UINT8 u8tmp;            
                u8tmp = RD_8BIT_SYSON_IO(0x67);
                u8tmp|= BIT2;
                WR_8BIT_SYSON_IO(0x67, u8tmp);
            }
#endif            
            SWITCH_BTON_GPIO_TO_OFF_DOMAIN(BTON_GPIOMAP_10);
            off_domain_log_control(OUTPUT_LOG1);
        }
        else
#endif            
        {
            SWITCH_BTON_GPIO_TO_OFF_DOMAIN(BTON_GPIOMAP_08);
            off_domain_log_control(OUTPUT_LOG0);        
        }


        
        #ifdef _SUPPORT_TIMER1_SHUTDOWN_LOG_
        g_u16shutdown_log_timer_cnt = 0;
        #endif

        
    }
    else
    {
        //DISABLE_OFF_DOMAIN_LOG_OUTPUT();
		SWITCH_BTON_GPIO_TO_ON_DOMAIN(BTON_GPIOMAP_09);
        off_domain_log_control(LOG_OFF);
    }
#endif

    //--- bzdma module init ---
    bzdma_init();

#ifdef _ENABLE_MAILBOX_
    mailbox_init();
#endif

#ifdef _SUPPORT_WL_BT_SCOREBOARD_
    if (IS_USE_SCOREBOARD)
    {
        scoreboard_init();
    }
#endif

#if defined(_RTK8723_UART_INIT_) && defined(_RTK8723_INTERFACE_INFO_)
    if (g_fun_interface_info.b.bt_interface == UART_INTERFACE)
    {
    /* Reserved com code patch by yilinli:
      * e.g.
      *   1. HCI UART reset/init conditions
      *   2. HCI UART H5 reset/init/restore
      *   3. etc.
      */
#ifdef _ROM_CODE_PATCHED_
        if (rcp_main_hci_uart_init_func != NULL)
        {
            rcp_main_hci_uart_init_func();
        }
        else
#endif
        {
#ifdef _YL_RTL8723A_B_CUT
            hci_uart_reset_init_RTL8723(0); /* for H4 */
#else
            if ( !( (IS_ROM_CODE_PATCHED) && (IS_FW_TRIG_WDG_TO) && (IS_SEND_PATCH_END) ) )
            {
                hci_uart_reset_init_RTL8723(0); // PATCH WDG Timeout: do NOT execute hci_uart_reset_init_RTL8723(0)
                                                 // conditions for executing hci_uart_reset_init_RTL8723(0):
                                                 //     a. BT reset by BT_DISn
                                                 //     b. Power-On Reset
                                                 //     c. Other FW WDG Timeoutt (e.g.: infinite loop, H5 link reset, etc.)
            }
#endif
        }
    }
#endif

#ifdef _ENABLE_BTON_POWER_SAVING_
#ifdef _8821A_BTON_DESIGN_
    if (efuse_pow_setting_4.fast_gpio_power_on_check2b==0)
#endif
    {
        power_init(); /* must after GpioInit() */
    }
#endif

#ifdef _SPIC_TEST_
    spi_test_func();
#endif

    //--- enable debug poprt ----
    dbg_rtk_set_debug_port();

    //--- enable pta function ---
    pta_function_set();

#ifdef _PLC_FUNC_ENABLE_
    plc_init();
#endif


#ifdef MWS_ENABLE
    if (IS_BT_MWS)
    {
        mws_init();
    }
#endif

#ifdef CONFIG_TV_FEATURE
    tv_app_init();
#endif

#ifdef CONFIG_FM
#ifdef _SUPPORT_INFO_FROM_SYSTEM_ON_
    if (g_u8support_fm)
#endif
    {
        fm_init();
    }
#endif

#ifdef _ROM_CODE_PATCHED_
    /* we can re-init or change any hw registers via patch here - austin */
    if (rcp_hw_reg_reinit_func != NULL)
    {
        rcp_hw_reg_reinit_func();
    }
#endif

#ifdef _BRUCE_WDG_TIMEOUT_DEBUG

#ifdef _MOVE_SOME_FW_SRAM_SIGNATURES_TO_BTON_REG_
    if ((!IS_FW_TRIG_WDG_TO) && (IS_PWL_ON))
#else
    if (!IS_FW_TRIG_WDG_TO)
#endif

    {
        RT_BT_LOG(RED, MSG_BT_WDG_TIMEOUT_DBG, 8,
                  patch_in_isr, patch_isr_num, patch_in_cpu_idle,
                  patch_signal_id, patch_isr_cause,
                  patch_isr_cnts_during_one_callback, patch_last_epc, patch_ra);
#ifdef _FIX_CROSSBAR_ERROR_DEBUG_  
         print_timer_info();
#endif

#ifdef _MONITOR_WDG_TIMEOUT_THROUGH_HCI_EVENT
        if (IS_SEND_EVT_WHEN_WDG_TO)
        {
            HCI_VENDOR_WATCHDOG_REBOOT_EVT_PARAM evt_param = {
                    .subevt_code = HCI_VENDOR_WATCHDOG_REBOOT_SUBEVENT,
                    .reserved = 0,
                    .in_isr = patch_in_isr,
                    .isr_num = patch_isr_num,
                    .in_cpu_idle = patch_in_cpu_idle,
                    .signal_id = patch_signal_id,
                    .isr_cause = patch_isr_cause,
                    .isr_cnts_during_one_callback = patch_isr_cnts_during_one_callback,
                    .last_epc = patch_last_epc,
                    .os_timer_handle  = dbg_os_timer.handle,
                    .os_timer_cal_table_index = dbg_os_timer.cal_table_index,
                    .os_timer_timer_count = dbg_os_timer.timer_count,
                    .os_timer_timer_value = dbg_os_timer.timer_value,
                    .os_timer_timeout_function = (UINT32) dbg_os_timer.timeout_function,
                    .os_timer_timer_type = dbg_os_timer.timer_type,
                    .os_timer_args = (UINT32)dbg_os_timer.args,
                    .os_timer_next = (UINT32) dbg_os_timer.next,
                    .os_timer_state = dbg_os_timer.state,
                    .os_timer_sniff_tick_timer = dbg_os_timer.sniff_tick_timer
            };
            hci_generate_event(HCI_VENDOR_SPECIFIC_EVENT, &evt_param, sizeof (evt_param));
        }
#endif
    }

    patch_in_isr = 0;
    patch_isr_num = 0xff;
    patch_in_cpu_idle = 0;
    patch_signal_id = 0xffff;
    patch_isr_cause = 0;
    patch_isr_cnts_during_one_callback = 0;
    patch_last_epc = 0;
#endif

    //---log "MAIN()" ---
    RT_BT_LOG(GREEN, MSG_FW_MAIN_SHOWTIME, 9,
                      fw_lmp_sub_version, fw_hci_sub_version, cpu_clk,
                      otp_str_data.bt_bd_addr[5], otp_str_data.bt_bd_addr[4],
                      otp_str_data.bt_bd_addr[3], otp_str_data.bt_bd_addr[2],
                      otp_str_data.bt_bd_addr[1], otp_str_data.bt_bd_addr[0]);

    //---enable interrupts---
    NVIC_SetPriority(BTMAC_IRQn, configMAX_SYSCALL_INTERRUPT_PRIORITY);
    NVIC_EnableIRQ(BTMAC_IRQn);
#ifndef IS_BTSOC
    Rlx4181EnableInterrupt();
#endif

    dbg_msg_trigger_one_shot_event(1, DBG_MSG_EXPIRE_TIME);

#ifdef TEST_MODEM_PSD
    test_modem_psd_init_func();
#endif

#ifndef IS_BTSOC
#ifdef _ROM_CODE_PATCHED_
    if (IS_ROM_CODE_PATCHED)
    {
        if (IS_FW_TRIG_WDG_TO)
        {
            if (IS_SEND_PATCH_END)
            {
                /* notify host that we are ready after finish the patched
                   procedure and do a silence reset */
                rom_code_patch_block_index = GET_PATCH_END_INDEX;
                hci_vendor_cmd_generate_event(HCI_VENDOR_DOWNLOAD,
                                              HCI_COMMAND_SUCCEEDED, NULL);
                ERASE_PATCH_END_EVENT_SIGNATURE;
            }
            ERASE_FW_TRIG_WDG_TO_SIGNATURE;
        }
    }
#endif
#endif
    if (IS_H5_LINKRESET)
    {
        ERASE_H5_LINKRESET_SIGNATURE;
    }

#ifdef _MOVE_SOME_FW_SRAM_SIGNATURES_TO_BTON_REG_
#ifdef _BRUCE_WDG_TIMEOUT_DEBUG
    /* sign the signature to the sram */
    SIGN_PWL_ON_SIGNATURE;
#endif
#endif

#ifdef _SUPPORT_AUTO_DETACH_LINK_
    memset(&catch_os_signal, 0, sizeof(OS_SIGNAL));
#endif

#ifdef _YL_TEST_NEW_MODEM_PSD_SCAN_FOR_AFH_
    /* Manually trigger lmp_init_assessment_at_connection() to initiate PSD-Scan procedures */
    lmp_init_assessment_at_connection();
#endif

#ifdef _BRUCE_MODEM_ADC_DAC_TIMING
    //rtk_write_modem_radio_reg(0x00, 0x01, 0);// switch to page0
    /*(bruce) default 0x18=0x690F
          0x18[4]:ADC timing ; 0x18[5]:DAC timing */
    RTK_WRITE_MODEM_REG(0x00,0);
    UINT16 data = rtk_read_modem_radio_reg(TRANS_MODEM_REG(0x18), TYPE_MODEM);
    RT_BT_LOG(RED,YL_DBG_HEX_1,1,data);
    data |=0x10;
    data |=0x20;
    //data &=0xDF;
    //data &=0xEF;
    //rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x18), 0x01, data);
    RT_BT_LOG(RED,YL_DBG_HEX_1,1,data);
    RTK_WRITE_MODEM_REG(TRANS_MODEM_REG(0x18),data);
#endif

    low_stack_setup_task();
}

