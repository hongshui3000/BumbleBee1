/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  Contains LC module initialization and shutdown routines. It also has
 *  utility functions and low power mode functions.
 */

/********************************* Logger *************************/
enum { __FILE_NUM__= 36 };
/********************************* Logger *************************/

/* ========================= Include File Section ========================= */
#define LC
#include "lc_internal.h"
#include "bt_fw_os.h"
#include "vendor.h"
#include "lc.h"
#include "platform.h"
#include "bz_debug.h"
#include "UartPrintf.h"
#include "bz_fw_isoch.h"
#include "mem.h"
#include "mailbox.h"

#include "bt_fw_acl_q.h"

#include "lmp_2_1.h"
#include "lc_2_1.h"

#ifdef VER_3_0
#include "lmp_3_0.h"
#endif

#include "lmp_pdu_q.h"
#include "led_debug.h"

#ifdef LE_MODE_EN
#include "le_hw_reg.h"
#endif

#include "lmp_internal.h"
#include "gpio.h"

#include "le_ll_driver.h"
#include "power_control.h"

#ifdef _BT_ONLY_
#include "new_io.h"
#endif
#include "le_hci_4_0.h"
#if defined(_ENABLE_RETENTION_FLOW_FOR_DLPS_)
#include "bz_auth_lmp.h"
#include "ecdh.h"
#include "h5.h"
#include "pta.h"
#include "pta_meter.h"
//#include "fm.h"
#endif

#ifdef _SUPPORT_BT_CTRL_FM_
#include "fm.h"
#endif

#ifdef MWS_ENABLE
#include "mws.h"
#include "lmp_ch_assessment.h"
#include "mws_imp.h"
#endif

#ifdef _SUPPORT_CSB_TRANSMITTER_
#include "bt_3dd.h"
#endif
#ifdef CONFIG_TV_POWERON_LPS
#include "tv.h"
#endif

/* ==================== Macro declaration Section ===================== */
#ifdef _YL_LPS
EFUSE_LPS_SETTING_1_S g_efuse_lps_setting_1;
EFUSE_LPS_SETTING_2_S g_efuse_lps_setting_2;
EFUSE_LPS_SETTING_3_S g_efuse_lps_setting_3;

#ifdef _CCH_RTL8723A_B_CUT
// _CCH_ECO_LPS_
EFUSE_LPS_SETTING_4_S g_efuse_lps_setting_4;
#endif

#ifdef _LPS_FOR_8821_
EFUSE_LPS_SETTING_5_S g_efuse_lps_setting_5;
#endif

#if defined(_CCH_SNIFF_NEG_TIMEOUT_) || defined(_CCH_RETENTION_FLOW_FOR_DLPS_)
EFUSE_RSVD_2_S g_efuse_rsvd_2;
#endif

extern UINT32 lc_sniff_cont_count[LC_MAX_NUM_OF_LUT_EX_TABLES];

#ifdef _CCH_RTL8723A_B_CUT
// _CCH_ECO_LPS_
extern UINT32 lc_sniff_window_rx_ind[LC_MAX_NUM_OF_LUT_EX_TABLES];
extern UINT32 lc_sniff_sup_count[LC_MAX_NUM_OF_LUT_EX_TABLES];
#endif



#else
#define SLEEP_MODE_GUARD_INTERVAL     4
#endif

#ifdef _CCH_LPS_
#ifdef ENABLE_LOGGER
extern OS_HANDLE bz_logger_task_handle;
#endif
extern OS_HANDLE hci_ch_as_task_handle;
extern UINT8 dape_task_queue[OS_TOTAL_TASKS];
extern UINT8 dape_task_queue_max[OS_TOTAL_TASKS];

extern UINT32 g_lps_timer_counter;
extern UINT32 g_timer2_init_value;
#endif

#ifdef _DAPE_TEST_DISABLE_DM1_FOR_CCPT_BY_VENDOR_CMD
extern UINT8 g_disable_dm1;
#endif

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_

#define DLPS_BACKUP_BLUEWIZ_REG_SIZE 5

const UINT16 dlps_backup_bluewiz_reg_addr[DLPS_BACKUP_BLUEWIZ_REG_SIZE] = {
CORRELATOR_THRESOLD_REGISTER,
VOICE_SETTING_REGISTER,
0x190,  // PTA_BT_ACL_TO_CTL
0x19E,  // PTA_BT_SCO_RX_PRE_DET_CTRL
0x1A6   // PTA_BT_STAISTICS_CTRL
};

UINT16 dlps_backup_bluewiz_reg_buf[DLPS_BACKUP_BLUEWIZ_REG_SIZE];


#define DLPS_BACKUP_LE_REG_SIZE 18
const UINT16 dlps_backup_le_reg_addr[DLPS_BACKUP_LE_REG_SIZE] = {
LE_REG_SLAVE_WIN_WIDENING_L,
LE_REG_SLAVE_CH_MAP_H,
LE_REG_INT_CE_EARLY_INT_TIME,
LE_REG_INT_IMR,
LE_REG_RANDOM_SEED_L,
LE_REG_RANDOM_SEED_H,
LE_REG_STATUS_CAM_VALID,
LE_REG_DEVA_REMOTE_L,
LE_REG_DEVA_REMOTE_M,
LE_REG_DEVA_REMOTE_H,
LE_REG_DEVA_RANDOM_LOCAL_L,
LE_REG_DEVA_RANDOM_LOCAL_M,
LE_REG_DEVA_RANDOM_LOCAL_H,
LE_REG_DEVA_PUBLIC_LOCAL_L,
LE_REG_DEVA_PUBLIC_LOCAL_M,
LE_REG_DEVA_PUBLIC_LOCAL_H,
LE_REG_DEVA_TYPE,
LE_REG_RX_PKT_ERR_CTRL
};

UINT16 dlps_backup_le_reg_buf[DLPS_BACKUP_LE_REG_SIZE];


#define DLPS_BACKUP_DWGPIO_REG_SIZE 9

const UINT16 dlps_backup_dwgpio_reg_addr[DLPS_BACKUP_DWGPIO_REG_SIZE] = {
GPIO_PORTA_DATA_REG,
GPIO_PORTA_DATA_DREC_REG,
GPIO_INT_POLARITY_REG,
GPIO_INT_TYPE_LEVEL_REG,
GPIO_DEBOUNCE_REG,
GPIO_PORTA_EOI_REG,
GPIO_LS_SYNC,
GPIO_INT_MASK_REG,
GPIO_INT_EN_REG
};

UINT32 dlps_backup_dwgpio_reg_buf[DLPS_BACKUP_DWGPIO_REG_SIZE];

#if defined(_CHECK_STACK_SIZE_B4_ENTER_DLPS_) && defined(_ENABLE_RETENTION_FLOW_FOR_DLPS_)
UINT32 g_dmem_end_addr[1];
#endif

extern unsigned char _intr_stack[];
extern OS_HANDLE mailbox_handle;
extern UCHAR mailbox_task(OS_SIGNAL *signal_ptr);
extern OS_SIGNAL catch_os_signal;
extern void dbg_msg_trigger_one_shot_event(UINT32 log_msg_type, UINT32 time_ms);
extern void lmp_init_connection_entity_on_power_on(UINT16 ce_index);

#include "mint_os_timer_internal.h"
extern OS_TIMER timer_pool[OS_MAX_TIMER_BUCKETS];
#endif

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
#define WRITE_DLPS_MODE(enable)\
{ \
    UINT8 u8data; \
    u8data = VENDOR_BYTE_READ(BTON_DLPS_CONTROL_REGISTER); \
    u8data &= (~BIT0); \
    u8data |= enable; \
    VENDOR_BYTE_WRITE(BTON_DLPS_CONTROL_REGISTER, u8data); \
}
#endif
/* ==================== Structure declaration Section ===================== */

/* ===================== Variable Declaration Section ===================== */

/* ================== For rom code patch function point ================== */
#ifdef _ROM_CODE_PATCHED_
PF_ROM_CODE_PATCH_FUNC rcp_lc_start_sniff_head_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_lc_start_sniff_end_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_lc_calculate_tolerance_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_lc_program_sniff_sm_mode_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_lc_sniff_sm_tune_wakeup_instant_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_lc_handle_sm_intr_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_lc_program_lps_mode_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_lc_program_lps_mode_end_func = NULL;

#ifdef _CCH_8723_RCP_
PF_ROM_CODE_PATCH_FUNC rcp_lc_exit_sniff_mode = NULL;
PF_ROM_CODE_PATCH_VOID rcp_lc_baseband_reset = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_lc_kill_paging = NULL;
#ifndef _DISABLE_HOLD_MODE_
//PF_ROM_CODE_PATCH_FUNC rcp_lc_program_hold_sm_mode = NULL;
#endif
#ifndef _DISABLE_PARK_MODE_
//PF_ROM_CODE_PATCH_FUNC rcp_lc_program_park_sm_mode = NULL;
#endif
PF_ROM_CODE_PATCH_FUNC rcp_lps_period_state_machine = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_lps_period_state_enter_lps = NULL;
#endif

#ifdef _CCH_8821_RCP_
PF_ROM_CODE_PATCH_FUNC rcp_lc_check_lps_for_link = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_lc_check_lps_for_idle = NULL;
PF_ROM_CODE_PATCH_VOID rcp_lc_check_lps_for_resume = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_lc_lps_double_check_dsm_cond = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_lc_lps_double_check_dsm_cond_after_clr_sts = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_lc_check_lps_task_queue = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_lc_program_lps_upd_val = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_lc_program_lps_check_cal = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_lc_program_lps_upd_par = NULL;
#endif

#ifdef _CCH_8821B_TC_RCP_
PF_ROM_CODE_PATCH_FUNC rcp_lps_period_state_machine_fast = NULL;
#endif

#ifdef _CCH_8703B_RCP_
PF_ROM_CODE_PATCH_VOID rcp_dlps_save_cpu_to_partial_on_then_enter_deep_lps_mode = NULL;
PF_ROM_CODE_PATCH_VOID rcp_dlps_save_io_mem_to_partial_on = NULL;
PF_ROM_CODE_PATCH_VOID rcp_dlps_restore_io_mem_from_partial_on = NULL;
PF_ROM_CODE_PATCH_VOID rcp_dlps_restore_io_mem_from_partial_on_after_clk_rdy = NULL;

PF_ROM_CODE_PATCH_FUNC rcp_lps_period_state_machine_mid = NULL;
#endif

PF_ROM_CODE_PATCH_FUNC rcp_lc_get_least_sniff_interval = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_lc_update_pkts_allowed = NULL;

#endif

LUT_EXTENSION_TABLE lut_ex_table[LC_MAX_NUM_OF_LUT_EX_TABLES];

const UINT16 reg_PICONET_BD_ADDR1[4] = {
    0x018, //PICONET1_BD_ADDR1_REGISTER
    0x110, //PICONET2_BD_ADDR1_REGISTER
    0x14A, //PICONET3_BD_ADDR1_REGISTER
    0x156  //PICONET4_BD_ADDR1_REGISTER
};

const UINT16 reg_PICONET_BD_ADDR2[4] = {
    0x01A, //PICONET1_BD_ADDR2_REGISTER
    0x112, //PICONET2_BD_ADDR2_REGISTER
    0x14C, //PICONET3_BD_ADDR2_REGISTER
    0x158, //PICONET4_BD_ADDR2_REGISTER
};

const UINT16 reg_PICONET_BD_ADDR3[4] = {
    0x01C, //PICONET1_BD_ADDR3_REGISTER
    0x114, //PICONET2_BD_ADDR3_REGISTER
    0x14E, //PICONET3_BD_ADDR3_REGISTER
    0x15A, //PICONET4_BD_ADDR3_REGISTER
};

const UINT16 reg_PICONET_PARITY_BITS1[4] = {
    0x01E, //PICONET1_PARITY_BITS_REGISTER1
    0x116, //PICONET2_PARITY_BITS_REGISTER1
    0x150, //PICONET3_PARITY_BITS_REGISTER1
    0x15C  //PICONET4_PARITY_BITS_REGISTER1
};

const UINT16 reg_PICONET_PARITY_BITS2[4] = {
    0x020, //PICONET1_PARITY_BITS_REGISTER2
    0x118, //PICONET2_PARITY_BITS_REGISTER2
    0x152, //PICONET3_PARITY_BITS_REGISTER2
    0x15E  //PICONET4_PARITY_BITS_REGISTER2
};

const UINT16 reg_PICONET_PARITY_BITS3[4] = {
    0x016, //PICONET1_PARITY_BITS_REGISTER3
    0x11A, //PICONET2_PARITY_BITS_REGISTER3
    0x154, //PICONET3_PARITY_BITS_REGISTER3
    0x160  //PICONET4_PARITY_BITS_REGISTER3
};

const UINT16 reg_SCA_SLAVE_LOWER_LUT[4] = {
    0x100, //SCA_SLAVE1_LOWER_LUT_ADDR
    0x104, //SCA_SLAVE2_LOWER_LUT_ADDR
    0x13E, //SCA_SLAVE3_LOWER_LUT_ADDR
    0x142  //SCA_SLAVE4_LOWER_LUT_ADDR
};

const UINT16 reg_SCA_SLAVE_UPPER_LUT[4] = {
    0x102, //SCA_SLAVE1_UPPER_LUT_ADDR
    0x106, //SCA_SLAVE2_UPPER_LUT_ADDR
    0x140, //SCA_SLAVE3_UPPER_LUT_ADDR
    0x144  //SCA_SLAVE4_UPPER_LUT_ADDR
};

const UINT16 reg_PICONET_INFO[4] = {
    0x108, //PICONET1_INFO_REGISTER
    0x10A, //PICONET2_INFO_REGISTER
    0x146, //PICONET3_INFO_REGISTER
    0x148  //PICONET4_INFO_REGISTER
};

const UINT8 reg_MASTER_LOWER_LUT[8] = {
    0x70, 0x74, 0x78, 0x7C, 0x80, 0x84, 0x88, 0x8C
};

const UINT8 reg_MASTER_UPPER_LUT[8] = {
    0x72, 0x76, 0x7A, 0x7E, 0x82, 0x86, 0x8A, 0x8E
};
#ifdef _DAPE_SLV_SUPTO_IMPROVE
const UINT16 reg_SLAVE_TPOLL_MASK_REG[4] = {
    0x226, //SLAVE_TPOLL_MASK_REG
    0x228, //SLAVE_TPOLL_MASK_REG
    0x22a, //SLAVE_TPOLL_MASK_REG
    0x22c  //SLAVE_TPOLL_MASK_REG
};
#endif
/* Use for manage the scatternet */
LC_SCA_MANAGER_S lc_sca_manager;

/* The LMP Buffer in SRAM (use for BZDMA) */
ALIGN(4) SECTION_SRAM UINT8 bzdma_tx_buf[LMP_MAX_PICONETS_SUPPORTED][BZDMA_ACL_TX_BUF_SIZE];

/* The Maximum RTK Radio Tx Step Index */
UINT8 max_rtk_radio_tx_step_index = 0x06;
UINT8 default_rtk_radio_tx_step_index = 0x06;

LC_CUR_SCATTERNET_STATE lc_current_scatternet_state;

/* Command and Event Task handles */
TASK_ID  lc_tx_task_handle;
TASK_ID  lc_rx_task_handle;

/* Pool handles  HOST ===> HOST CONTROLLER DIRECTION. */
POOL_ID acl_data_pool_id;

#if defined(SCO_OVER_HCI) || defined(COMPILE_ESCO)
POOL_ID synchronous_data_pool_id;
#endif /*  defined(SCO_OVER_HCI) || defined(COMPILE_ESCO) */

/* Pool handles  HOST CONTROLLER ===> HOST  DIRECTION. */
POOL_ID acl_data_to_host_pool_id;

#if defined(SCO_OVER_HCI) || defined(COMPILE_ESCO)
POOL_ID synchronous_data_to_host_pool_id;
#endif /* defined(SCO_OVER_HCI) || defined(COMPILE_ESCO) */

/* Pool handles  HOST CONTROLLER ===> REMOTE DEVICE DIRECTION. */
POOL_ID synchronous_data_to_remote_id;

UINT8 lc_cont_crc_rx_cnt[LC_MAX_NUM_OF_LUT_EX_TABLES] = {0};
UINT8 lc_cont_crc_tx_cnt[LC_MAX_NUM_OF_LUT_EX_TABLES] = {0};

#ifdef COMPILE_PARK_MODE
UCHAR lc_start_of_beacon = FALSE;
UINT16 lmp_unpark_ce_index = INVALID_CE_INDEX;
UCHAR lc_beacon_running_flag = FALSE;
UINT16 slave_ce_index = INVALID_CE_INDEX;
#endif

UINT16 lc_power_ctr_config;

#ifdef POWER_SAVE_FEATURE
LC_SLEEP_MODE_PARAM sleep_mode_param;
UINT16 bb_write_baseband_register_func_imp_count;
UINT8 g_lps_scan_status_stay_count = 0;
#endif

UINT16 lc_current_interrupt_mask = 0xFFFF;
UINT16 lc_current_interrupt_mask2 = 0xFFFF;
UINT16 lc_var_esco_fifo_flush_length_and_tx_pkt_len_reg = 0;

UINT16 programmable_rand = MAX_RAND;

TimerHandle_t random_backoff_timer = NULL;

UCHAR random_backoff_status = FALSE;

UINT16 lmp_sup_timeout_var[LMP_MAX_CE_DATABASE_ENTRIES] = {0};
UCHAR lmp_sup_var_timeout_var[LMP_MAX_CE_DATABASE_ENTRIES] = {0};

UINT16 lc_num_unpark_req_trials = 0;

#ifdef ENABLE_SCO
UCHAR lc_full_bandwidth_flag;       /**< TRUE, literally when the whole
                                         b/w is occupied (i.e., The entire
                                         physical channel is occupied). FALSE,
                                         otherwise. */
UCHAR lc_acl_full_bandwidth_flag;   /**< TRUE, when the available ACL bandwidth
                                         (depends active/sniff modes) is
                                         completely occupied by synchronous
                                         links. FALSE, otherwise. */
#endif /* ENABLE_SCO */

UINT8 lc_waiting_for_crc_pkt_ack[LMP_MAX_PICONETS_SUPPORTED] = {FALSE};

UCHAR lc_cur_connecting_am_addr = INVALID_AM_ADDR;

UINT16 lc_paging_bd_addr[3];
UINT16 lc_paging_parity_bits[3];
UINT16 lc_slave_bd_addr[3];
UINT16 lc_slave_parity_bits[3];

#ifdef COMPILE_ESCO
ESCO_SCHEDULER esco_scheduler;


UCHAR lc_esco_stop_reception[8] = {FALSE,FALSE,FALSE,FALSE,FALSE,FALSE,FALSE,FALSE};

UCHAR lc_esco_window[8] = {FALSE,FALSE,FALSE,FALSE,FALSE,FALSE,FALSE,FALSE};
UCHAR lc_esco_pkt_tx[8] = {FALSE,FALSE,FALSE,FALSE,FALSE,FALSE,FALSE,FALSE};

/* Indicates if a packet is stored in the global esco buffer */
UCHAR global_esco_buf_valid[8] = {FALSE,FALSE,FALSE,FALSE,FALSE,FALSE,FALSE,FALSE};
/* Indicates if the data stored in the global buffer has CRC error */
UCHAR global_esco_data_correct[8] = {FALSE,FALSE,FALSE,FALSE,FALSE,FALSE,FALSE,FALSE};

#ifdef _ESCO_NEW_SCHEDULER_FOR_FULL_BW_
UCHAR lc_esco_window_overlap[8] = {FALSE,FALSE,FALSE,FALSE,FALSE,FALSE,FALSE,FALSE};
#endif
#endif

#ifdef TEST_MODE
UCHAR lc_test_mode_ack_recd;
UCHAR lc_tci_pause_flag;

#ifdef _DUT_DELAYED_LOOPBACK_MODE_
DUT_MODE_MANAGER dut_mode_manager;
#endif

#endif /* TEST_MODE */

UCHAR lc_is_tpoll_started[MAX_NO_OF_LUT_INDICES];

UINT8 lc_baseband_has_reset = FALSE;

extern LMP_FHS_PKT_RECD  *prev_fhs_ptr_for_eir;

/*following two global variables can be modified in patch code*/
UINT16 lc_min_sniff_interval_allow_5_slot_pkt = 25;
UINT16 lc_min_sniff_interval_allow_3_slot_pkt = 11;

/* ================== Static Function Prototypes Section ================== */
UCHAR lc_calculate_tolerance(UINT16 inactive_slots, UINT16 ce_index);

#ifdef DETECT_SNIFF_OVERLAP
void lc_check_for_overlapping_sniff_windows(void);
#endif

void lc_set_2m_tx_power (UINT8 lut_idx,UINT8 txgain_idx);
void lc_set_3m_tx_power (UINT8 lut_idx,UINT8 txgain_idx);

extern void lmp_handle_page_resp_timeout(UCHAR am_addr, UCHAR phy_piconet_id);

/* ===================== Function Definition Section ====================== */
#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
void lc_baseband_partial_reset(UINT8 dlps_flow)
#else
void lc_baseband_partial_reset(void)
#endif
{
    UINT32 lap;
    UCHAR parity_bits[5];
    UINT32 i;
    UINT16 temp_val;
    UINT16 nat_clk1 = 0;
    UINT16 nat_clk2 = 0;
    UINT8 default_power_index;
    UINT16 temp_buf[22];

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
    if(dlps_flow == FALSE)
#endif
    {
#if 0
#ifdef FOR_SIMULATION
        for(i = 0; i < 3000; i++); // to avoid native_clock read 0 when polling reset (1----->0->2---->3)
#else
        pf_delay(50);              /* we need to wait so long ? (about 50 ms) - austin  */
#endif
#endif

        nat_clk1 = BB_read_baseband_register(NATIVE_CLOCK1_REGISTER);
        nat_clk2 = BB_read_baseband_register(NATIVE_CLOCK2_REGISTER);

        /* save bluewiz phy settings */
        if (lc_baseband_has_reset)
        {
            /* save some phy relative register settings to memory */
            lc_save_bluewiz_phy_settings(temp_buf);
        }

        /* Reset the baseband */
        BB_write_baseband_register(INSTRUCTION_REGISTER, BB_RESET);

        /*
         * Spin till the reset instruction is completely executed
         * in the baseband. This is when the clock in
         * native-clock-1-register reads 0.
         */
        while(1)
        {
            if (BB_read_baseband_register(NATIVE_CLOCK1_REGISTER) == 0)
            {
                break;
            }
        }
    }

    /* Select sys clock */
    /* bit 2 should be 0(or with 0x0000) for 13mhz */
    /* bit 2 should be 1(or with 0x0004) for 16mhz */
    lc_power_ctr_config = (UINT16) (otp_str_data.bw_rf_radio_sys_clk_sel << 3);

    /* Select low power clock */
    /* bit 3 should be 1(or with 0x0008) for 32khz */
    /* bit 3 should be 0(or with 0x0000) for 32.768khz */
    lc_power_ctr_config |= otp_str_data.bw_rf_low_clk_frac << 2;

    /* Osc start up delay */
    lc_power_ctr_config |= otp_str_data.bw_rf_osc_delay << 8;

    BB_write_baseband_register(POWER_CONTROL_REGISTER, lc_power_ctr_config);

    BB_write_baseband_register(INTERRUPT_MASK_REGISTER, 0xFFFF);
    lc_current_interrupt_mask = 0xFFFF;
    lc_var_esco_fifo_flush_length_and_tx_pkt_len_reg = 0x0000;

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
    if(dlps_flow == FALSE)
#endif
    {
        BB_write_baseband_register(NATIVE_CLOCK1_REGISTER, nat_clk1);
        BB_write_baseband_register(NATIVE_CLOCK2_REGISTER, nat_clk2);
    }

    //RT_BT_LOG(GREEN, LC_318, 1, lc_power_ctr_config);
#ifdef _YL_LPS
    LPS_DBG_LOG(GRAY, LPS_LOG_002, 3, otp_str_data.bw_rf_radio_sys_clk_sel, otp_str_data.bw_rf_low_clk_frac, otp_str_data.bw_rf_osc_delay);
#endif

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
    if(dlps_flow == FALSE)
#endif
    {
        /* Self device BD_ADDR*/
        BB_write_baseband_register(LOCAL_BD_ADDR1_REGISTER,
              otp_str_data.bt_bd_addr[0] | (otp_str_data.bt_bd_addr[1] << 8));

        BB_write_baseband_register(LOCAL_BD_ADDR2_REGISTER,
              otp_str_data.bt_bd_addr[2] | (otp_str_data.bt_bd_addr[3] << 8));

        BB_write_baseband_register(LOCAL_BD_ADDR3_REGISTER,
             otp_str_data.bt_bd_addr[4] | (otp_str_data.bt_bd_addr[5] << 8));
    }

    /*
    * Enable Data Whitening. Has to be done after Test Control is
    * complete. Or, on reset. Writes a zero, and enables the Whitening block
    */
    AND_val_with_bb_reg(SCO_PACKET_TYPE_REGISTER,0xff7f);

    /* disable all possible loopback paths in voice settings */
#ifdef _HW_AUTO_MUTE_
    UINT16 reg_val;
    reg_val = BB_read_baseband_register(VOICE_SETTING_REGISTER);
    reg_val &= ((g_efuse_lps_setting_3.hw_auto_mute_threshold<<4)|
                (g_efuse_lps_setting_3.hw_auto_mute_enable<<1));
    BB_write_baseband_register(VOICE_SETTING_REGISTER, reg_val);
#else
    BB_write_baseband_register(VOICE_SETTING_REGISTER, 0x00);
#endif

    /* reset pcm convert mapping table */
    BB_write_baseband_register(BB_CODEC_CODE_TABLE_REG, 0x00);

    /* reset sco/esco convert mapping table */
    BB_write_baseband_register(BB_SCO_CONV_TABLE_REG, 0x00);

    /* disable fifo control settings */
    BB_write_baseband_register(SYNC_FIFO_CONFIG_REGISTER, 0x00);

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
    if(dlps_flow == FALSE)
#endif
    {
        /* Extract LAP forself device */
        lap = otp_str_data.bt_bd_addr[2];
        lap <<= 8;
        lap |= otp_str_data.bt_bd_addr[1];
        lap <<= 8;
        lap |= otp_str_data.bt_bd_addr[0];

        /* Calculate Parity bits of remote device */
        lc_generate_parity_bits(lap, &parity_bits[0]);

        /* Self Device parity bits */
        BB_write_baseband_register(LOCAL_PARITY_BITS_REGISTER1,
                               (*((UINT16 *) &parity_bits[0])));

        BB_write_baseband_register(LOCAL_PARITY_BITS_REGISTER2,
                               (*((UINT16 *) &parity_bits[2])));

        BB_WRITE_LOCAL_PARITY_BITS((*((UINT16 *) &parity_bits[4])));

        BB_write_baseband_register(CLASS_OF_DEVICE_REGISTER1,
                  (UINT16) (lmp_self_device_data.class_of_device & 0xffff));
        BB_write_baseband_register_lower_octet(CLASS_OF_DEVICE_REGISTER2,
                  (UINT16)((lmp_self_device_data.class_of_device & 0x00ff0000) >> 16));
    }

    /* Write Hop in SCO Pkt Type Reg */
    BB_write_baseband_register(SCO_PACKET_TYPE_REGISTER,LC_DEFAULT_HOP);
    BB_set_sniff_priority_flag();

    temp_val = BB_read_baseband_register(INQ_RESP_FREEZ_REGISTER);
    temp_val |= 0x0400;
    BB_write_baseband_register(INQ_RESP_FREEZ_REGISTER, temp_val);

#ifdef _RF_CONFIG_PROTECTION_
    lc_rf_ctrl_stack = 0;
    lc_rf_off_mask = 0;
#endif

#ifdef _RTL8723B_DUT_MODE_
    /* exit dut mode */
    BB_write_baseband_register(RADIO_SELECT_REGISTER, 0x0107);
#endif

    /* init or load bluewiz phy settings */
    if (lc_baseband_has_reset)
    {
        /* load some phy relative register settings from memory */
        lc_load_bluewiz_phy_settings(temp_buf);
    }
    else
    {
        lc_update_radio_timings();
        lc_baseband_has_reset = TRUE;
    }

    BB_write_baseband_register_lower_octet(RE_TRANSMISSION_COUNT_REGISTER, 10);

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
    if(dlps_flow == FALSE)
#endif
    {
        default_power_index = RADIO_POWER_VAL;
        temp_val = default_power_index << 9;

        for(i = 0; i < 8; i++)
        {
            /* to initialize tx gain index for 2M/3M modulation */
            LC_SET_2M_TX_POWER(i, default_power_index);
            LC_SET_3M_TX_POWER(i, default_power_index);

            /* update new tx power and disable PTT and disable lbt_en */
            Update_val_with_bb_reg(reg_MASTER_UPPER_LUT[i],
                               temp_val, (0x07 << 9) | BIT15 | BIT12);

#ifdef _SUPPORT_LBT_FUNC_FOR_RF_ADAPTIVITY_
            if (rtl8723_btrf_check_and_enable_lbt(default_power_index))
            {
                Update_val_with_bb_reg(reg_MASTER_UPPER_LUT[i],
                                   BIT12, BIT12);
            }
#endif
        }

        /* To initialize tx gain index (1M/2M/3M) for each slave LUT. */
        for (i = 0; i < 4; i++)
        {
            LC_SET_2M_TX_POWER(LC_SCA_SLAVE_1_LUT + i, default_power_index);
            LC_SET_3M_TX_POWER(LC_SCA_SLAVE_1_LUT + i, default_power_index);

            /* update new tx power and disable PTT and disable lbt_en */
            Update_val_with_bb_reg(reg_SCA_SLAVE_UPPER_LUT[i],
                               temp_val, (0x07 << 9) | BIT15 | BIT12);

#ifdef _SUPPORT_LBT_FUNC_FOR_RF_ADAPTIVITY_
            if (rtl8723_btrf_check_and_enable_lbt(default_power_index))
            {
                Update_val_with_bb_reg(reg_SCA_SLAVE_UPPER_LUT[i],
                                   BIT12, BIT12);
            }
#endif
        }

#ifdef ENABLE_SCO
        BB_clear_slave_full_bandwidth_flag();
#endif

        BB_clear_single_slave_full_bandwidth_flag();
        BB_clear_multi_slave_full_bandwidth_flag();
    }

    /* Enable baseband Interrupts */
    lc_current_interrupt_mask &= (~BB_INTR1_MASK_ALL);
    BB_write_baseband_register(INTERRUPT_MASK_REGISTER,
                                lc_current_interrupt_mask);
#ifdef ENABLE_SCO
    lc_current_interrupt_mask2 &= (~BB_INTR2_MASK_ALL);
    BB_write_baseband_register(INTERRUPT_MASK_REGISTER2,
                               lc_current_interrupt_mask2);
#endif

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
    if(dlps_flow == FALSE)
#endif
    {
        BB_reset_variables();
    }
    OR_val_with_bb_reg(SCO_PACKET_TYPE_REGISTER, BIT12);
#ifdef _DAPE_TEST_BLOCK_ACL_FOR_ESCO
    OR_val_with_bb_reg(ESCO_STOP_RX_REGISTER, BIT11);
#endif
}

void lc_init_priority_settings(void)
{
    if (otp_str_data.bt_priority_low == 0x0000)
    {
        BZ_REG_S_PRI_CTRL *pri_ctrl_reg;
        pri_ctrl_reg = (BZ_REG_S_PRI_CTRL *)&otp_str_data.bt_priority_low;

        /* configure default priority control settings */
#ifdef _DAPE_TEST_NEW_HW
#ifndef _DAPE_TEST_NEW_HW_PAGE_RESP_PRIORITY_HIGHER_THAN_SCO
        pri_ctrl_reg->page_rsp_high_than_sco = FALSE;
#else
        pri_ctrl_reg->page_rsp_high_than_sco = TRUE;
#endif
#else
#ifndef _INQ_PAGE_RESP_PRIORITY_HIGHER_THAN_SCO
        pri_ctrl_reg->inq_page_rsp_high_than_sco = FALSE;
#else
        pri_ctrl_reg->inq_page_rsp_high_than_sco = TRUE;
#endif
#endif
#ifndef _CCH_TPOLL_SCO_PRI_
        pri_ctrl_reg->tpoll_high_than_sco_pn0 = TRUE;
        pri_ctrl_reg->tpoll_high_than_sco_pn1 = TRUE;
#else
        pri_ctrl_reg->tpoll_high_than_sco_pn0 = FALSE;
        pri_ctrl_reg->tpoll_high_than_sco_pn1 = FALSE;
#endif
        pri_ctrl_reg->en_esco_force_nak_pn0 = TRUE;
        pri_ctrl_reg->en_esco_force_nak_pn1 = TRUE;
#ifndef _PAGE_PRIORITY_HIGHER
        pri_ctrl_reg->page_high_than_acl = FALSE;
        pri_ctrl_reg->page_high_than_acl_def = FALSE;
#else
        pri_ctrl_reg->page_high_than_acl = TRUE;
        pri_ctrl_reg->page_high_than_acl_def = TRUE;

#endif
#ifndef _CCH_NO_PAUSE_SCO_
        pri_ctrl_reg->pause_sco = TRUE;
#else
        pri_ctrl_reg->pause_sco = FALSE;
#endif
        pri_ctrl_reg->scan_inquiry_high_than_acl = FALSE;
        pri_ctrl_reg->lpm_preempt_pn0 = TRUE;
        pri_ctrl_reg->lpm_preempt_pn1 = TRUE;
        pri_ctrl_reg->tpoll_lpm_pri_pn0 = TRUE;
        pri_ctrl_reg->tpoll_lpm_pri_pn1 = TRUE;
#ifdef _DAPE_TEST_BLOCK_ACL_FOR_ESCO
        pri_ctrl_reg->block_acl_for_esco = TRUE;
#endif
    }

    if (otp_str_data.bt_priority_high == 0x0000)
    {
        BZ_REG_S_PRI_CTRL1 *pri_ctrl_reg1;
        pri_ctrl_reg1 = (BZ_REG_S_PRI_CTRL1 *)&otp_str_data.bt_priority_high;

        /* configure default priority control 1 settings */
        pri_ctrl_reg1->acl_pause = 7;
        pri_ctrl_reg1->slave_crc_pri = TRUE;
        pri_ctrl_reg1->mask_txrx_timing = TRUE;
        pri_ctrl_reg1->en_esco_force_nak_pn2 = TRUE;
        pri_ctrl_reg1->en_esco_force_nak_pn3 = TRUE;
        pri_ctrl_reg1->lpm_preempt_pn2 = TRUE;
        pri_ctrl_reg1->lpm_preempt_pn3 = TRUE;
        pri_ctrl_reg1->tpoll_lpm_pri_pn2 = TRUE;
        pri_ctrl_reg1->tpoll_lpm_pri_pn3 = TRUE;
#ifndef _CCH_TPOLL_SCO_PRI_
        pri_ctrl_reg1->tpoll_high_than_sco_pn2 = TRUE;
        pri_ctrl_reg1->tpoll_high_than_sco_pn3 = TRUE;
#else
        pri_ctrl_reg1->tpoll_high_than_sco_pn2 = FALSE;
        pri_ctrl_reg1->tpoll_high_than_sco_pn3 = FALSE;
#endif
        pri_ctrl_reg1->legacy_high_than_le = FALSE;
#ifdef _DAPE_NO_BLOCK_LE_BEFORE_SCO
        pri_ctrl_reg1->sco_sup_le = FALSE;
#else
        pri_ctrl_reg1->sco_sup_le = TRUE;
#endif
    }

#ifdef _DAPE_TEST_NEW_HW
    if ((otp_str_data.bt_priority_3 & 0xFF) == 0x00)
    {
        BZ_REG_S_PRI_CTRL2 *pri_ctrl_reg2;
        pri_ctrl_reg2 = (BZ_REG_S_PRI_CTRL2 *)&otp_str_data.bt_priority_3;
        /* configure default priority control 1 settings */

#ifdef _DAPE_TEST_NEW_HW_LE_UNCONN_STATE_HIGHER_THAN_LEGACY_UNCONN
        pri_ctrl_reg2->le_adv_pri = TRUE;
#else
        pri_ctrl_reg2->le_adv_pri = FALSE;
#endif
#ifndef _DAPE_TEST_PAUSE_ESCO_FOREVER
        pri_ctrl_reg2->pause_esco = FALSE;
#else
        pri_ctrl_reg2->pause_esco = TRUE;
#endif
#ifdef _DAPE_TEST_NEW_HW_INQ_PRIORITY_HIGHER_THAN_ACL
        pri_ctrl_reg2->inquiry_acl_pri = TRUE;
        pri_ctrl_reg2->inq_high_than_acl_def = TRUE;
#else
        pri_ctrl_reg2->inquiry_acl_pri = FALSE;
        pri_ctrl_reg2->inq_high_than_acl_def = FALSE;
#endif

#ifdef _DAPE_TEST_NEW_HW_INQ_RESP_PRIORITY_LOWER_THAN_SCO
        pri_ctrl_reg2->inq_rsp_high_than_sco = FALSE;
#else
        pri_ctrl_reg2->inq_rsp_high_than_sco = TRUE;
#endif
#ifdef _DAPE_TEST_NEW_HW_LE_CONN_STATE_HIGHER_THAN_LEGACY_ACL
        pri_ctrl_reg2->le_conn_high_than_page_inq = TRUE;
#else
        pri_ctrl_reg2->le_conn_high_than_page_inq = FALSE;
#endif
        pri_ctrl_reg2->le_6_interval_lower = FALSE;
    }
#endif
    BB_write_baseband_register(SCA_PRIORITY_REGISTER,
                                otp_str_data.bt_priority_low);
    BB_write_baseband_register(SCA_PRIORITY_REGISTER2,
                                otp_str_data.bt_priority_high);
#ifdef _DAPE_TEST_NEW_HW
    BB_write_baseband_register(SCA_PRIORITY_REGISTER3,
                                ((otp_str_data.bt_priority_3)&((UINT16)(0xFF))));
#endif
#ifdef _DAPE_NEW_HW_AFTER_8821MP
    if (!IS_USE_FOR_BQB)
    {
        /* 20120821 enable for dual mode(LE master + FTP rx slave).
           0x21c[7] = 1 is good for LE + FTP rx slave. This is also original bluewiz's
           design. Chinwen adds this option for 3dd beacon problem found in VERA.
           If 3dd doesn't have beacon problem then we should enable this.*/
        BZ_REG_S_OPT_REG opt_reg;
        *(UINT16*)&opt_reg = BB_read_baseband_register(OPT_REGISTER);

        opt_reg.slv_blk_tx = TRUE;
#ifdef _DAPE_EN_8821_MP_MODIFY_SLV_SNIFF_TIMEOUT
            /* this bit makes sniff slave not to count sco pkt into sniff timeout.
               So we should enable this bit most of the time and disable it when
               sniff slave wants to send pkt. After receiving ack of that pkt,
               enable this bit again if there is no sniff slave sending pkt. */
        opt_reg.slv_sniff_opt = TRUE;
#endif
#ifdef _DAPE_ENABLE_HW_FIXED_BUGS
        /* enables all vera-found-HW-fixed-bugs to improve IOT. */
        opt_reg.esco_bugfix = FALSE;
        opt_reg.bug_fixed1 = TRUE;
        opt_reg.bug_fix2 = FALSE;
        opt_reg.bug_fixed3 = TRUE;
#endif
#ifdef _DAPE_HW_SINCE_8821B_MP
#ifdef _DAPE_CHG_ESCO_EARLY_INTR_TIME
        opt_reg.delay_esco_early_intr = TRUE;
#endif
#endif
        BB_write_baseband_register(OPT_REGISTER, *(UINT16*)&opt_reg);
    }
#endif
}

void lc_init_lps_settings(void)
{
#ifdef _YL_LPS
    *(UINT32*)&g_efuse_lps_setting_1 = otp_str_data.efuse_lps_setting_1_d32;
    *(UINT32*)&g_efuse_lps_setting_2 = otp_str_data.efuse_lps_setting_2_d32;
    *(UINT32*)&g_efuse_lps_setting_3 = otp_str_data.efuse_lps_setting_3_d32;

#ifdef _CCH_RTL8723A_B_CUT
// _CCH_ECO_LPS_
    *(UINT8*)&g_efuse_lps_setting_4 = otp_str_data.efuse_lps_setting_4_d8;
#endif

#ifdef _8821A_BTON_DESIGN_
    *(UINT8*)&g_efuse_lps_setting_5 = otp_str_data.efuse_lps_setting_5_d8;
#endif


#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
    *(UINT16*)&g_efuse_rsvd_2 = otp_str_data.EFuse_Rsvd_2;
#endif

#ifdef _YL_LPS_EFUSE_FORCE
    g_efuse_lps_setting_1.dsm_guard_interval = 6; // 3b, 7: 16; else: 2+x
    g_efuse_lps_setting_1.sm_guard_interval = 2; // 2b, 3: 16; else: 2+x
    g_efuse_lps_setting_1.min_sm_interval = 1; // 2b, 2^(x+1)
    g_efuse_lps_setting_1.min_dsm_interval = 1; // 2b, 2^(x+1)
    g_efuse_lps_setting_1.dsm_drift_scaling = 0; // 2b, DSM_DRIFT/2^x ????
    g_efuse_lps_setting_1.delay625us_before_bton_lps_req = 1; // 1b,
    g_efuse_lps_setting_1.new_tol_cal = 0; // 1b,
    g_efuse_lps_setting_1.sniff_tol_adjust = 1; // 1b
    g_efuse_lps_setting_1.min_sniff_int_for_xtol = 3; // 2b; 10*2^x slots
    g_efuse_lps_setting_1.lps_task_mask = 0x01ff;           // 16 bit; OS_TASK Mask

    g_efuse_lps_setting_2.lps_pri = 0;                      // 2b; 0: CH_AS_TASK_PRI, else: LC_RX_TASK_PRI
    g_efuse_lps_setting_2.timer2_lps_on = 0;                    // 1b; enable
    g_efuse_lps_setting_2.sniff_lps_on = 1;                       // 1b; enable

    // Timer2 (DSM)
    g_efuse_lps_setting_2.timer2_mode_sel = 1;          // 1b; (0) orig (1) enable cch function
                                                                                    // there is  (efuse_pow_para.b.lps_timer_en) for enable timer2 to enter lps "LPS_TIMER"
    g_efuse_lps_setting_2.timer2_scan_en = 1;           // 1b; enable

#ifndef _MODI_LPS_AFTER_RTL8821B_TC_
    g_efuse_lps_setting_2.timer2_kill_scan_delay_ms = 1;    // 2b; ( 2^(x+1) ) msec
#else
    g_efuse_lps_setting_2.le_lps_enable = 0;
    g_efuse_lps_setting_2.adjust_lps_scan_interval = 0;
#endif

    g_efuse_lps_setting_2.timer2_mode0_opt = 0;

    g_efuse_lps_setting_2.sniff_mode_sel = 1;           // 1b; (0) orig (1) enable cch function
    g_efuse_lps_setting_2.sniff_scan_en = 1;            // 1b; enable

#ifndef _MODI_LPS_AFTER_RTL8821B_TC_
    // Sniff + SM
    g_efuse_lps_setting_2.sm_scan_per = 6;          // 4b; ( sniff interval >>x) as period
    g_efuse_lps_setting_2.sm_scan_per_min = 2;      // 2b; (2^x) period
    // Sniff + DSM
    g_efuse_lps_setting_2.dsm_scan_per = 8;         // 4b; ( sniff interval >>x) as period
    g_efuse_lps_setting_2.dsm_scan_per_min = 1;     // 2b; (2^x) period
#else
    g_efuse_lps_setting_2.scan_lps_min_period = 2;
    g_efuse_lps_setting_2.lc_sniff_sup_max = 4;
    g_efuse_lps_setting_2.lps_nowakeup_when_no_scan = 0;
#endif

    g_efuse_lps_setting_2.force_exit_sm_en = 1;               // 1b; enable

    g_efuse_lps_setting_2.issc_nope_en = 0;             // 1b; issc return nope
    g_efuse_lps_setting_2.issc_nope_num = 0;            // 2b; (1+x)
    g_efuse_lps_setting_2.timer2_no_scan_wakeup_en = 1; // 1b;

    g_efuse_lps_setting_3.timer2_lps_isr_mask = 0x30c;  // 16b //0x004 for uart0 only
    g_efuse_lps_setting_3.timer2_reset_at_uart_isr_en = 1;  // 1b
    g_efuse_lps_setting_3.sniff_lps_lmp_to = 1;  // 1b;
    g_efuse_lps_setting_3.sniff_lps_sup_to = 1;  // 1b;

    g_efuse_lps_setting_3.iot_sco_can_overlap = 0;         // 1b

    g_efuse_lps_setting_3.lc_init_radio_osc_delay_opt = 1; // 1b (1); 1: use EFUSE (recommended)
    g_efuse_lps_setting_3.sniff_wakeup_xtol_opt = 2; // 2b (0); 0: original, 1: wakeup_inst -= tol, 2: wakeup_inst -= (tol-tol_prev)(recommended)

    /* set both to 1 can only help ralink IOT but affect BQB test cases */
#ifndef _CCH_IOT_RALINK_
    g_efuse_lps_setting_3.iot_ralink_tid_no_check = 0;          // 1b
#else
    g_efuse_lps_setting_3.iot_ralink_tid_no_check = 1;          // 1b
#endif

    g_efuse_lps_setting_3.iot_ralink_send_fea_req_later = 1;           // 1b

#ifdef _CCH_RTL8723A_B_CUT

#ifdef _HW_AUTO_MUTE_
    g_efuse_lps_setting_3.hw_auto_mute_enable = 1;
    g_efuse_lps_setting_3.hw_auto_mute_threshold = 1;
#else
// _CCH_8723_A_ECO_
    g_efuse_lps_setting_3.iot_sco_noise_pcm_send0 = 1;        // 1b;
    g_efuse_lps_setting_3.iot_sco_noise_no_sco_count = 0;     // 2b; (2<<(x<<2))no sco input  0 for per-packet
#endif

    g_efuse_lps_setting_3.iot_issc_inq_rssi = 1;              // 1b; enable
    g_efuse_lps_setting_3.iot_issc_rmr_par = 1;               // 1b; enable
    g_efuse_lps_setting_3.esco_nego_by_other = 1;             // 1b; enable

    g_efuse_lps_setting_3.use_ext_32k = 0;                     // 1b; use external 32k
    otp_str_data.bw_rf_low_clk_frac = 1;                     //EFUSE[FE] 0x1 //32K(1) or 32.768K(0)
#endif

#ifdef _CCH_RTL8723A_B_CUT
// _CCH_ECO_LPS_
    g_efuse_lps_setting_4.lps_chk_g_host_wake_bt = 0;        // 1b;
    g_efuse_lps_setting_4.lps_use_state=1;                   // 1b;
    g_efuse_lps_setting_4.whql_test_2sco=0;                  // 1b;
    g_efuse_lps_setting_4.lps_stop_afh_timer=0;              // 1b;
#ifdef _LPS_FOR_8821_
    g_efuse_lps_setting_4.lps_use_state_fast_lps=1;                   // 1b;
    g_efuse_lps_setting_4.lps_use_intr=0;                   // 1b;
#endif
#endif


#ifdef _8821A_BTON_DESIGN_
    g_efuse_lps_setting_4.lps_isr_mask_check_isr_switch = 1;
    g_efuse_lps_setting_4.lps_pow_ctrl_intr_check_isr_switch = 1;

#ifdef _2801_BTON_DESIGN_
    g_efuse_lps_setting_5.lps_check_ext_32k_exist = 1;
#ifdef LPS_NEW_CAL_MODE
    g_efuse_lps_setting_5.lps_use_new_cal = 0;
#else
    g_efuse_lps_setting_5.lps_use_new_cal = 1;
#endif
    g_efuse_lps_setting_5.lps_use_new_cal_fw_mode = 0;
#ifdef LPS_NEW_CAL_MODE
    g_efuse_lps_setting_5.lps_use_new_cal_ini_by_old = 1;  // set lps_use_new_cal = 0 first
#endif
#endif

    g_efuse_lps_setting_5.disable_never_enter_lps_when_usb_active = 0;
    g_efuse_lps_setting_5.lps_scan_protect_time = 3;
#endif

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
    g_efuse_rsvd_2.enable_deep_lps_mode = 0;
    g_efuse_rsvd_2.dlps_guard_time = 0;
    g_efuse_rsvd_2.le_link_lps_enable = 0;
    g_efuse_rsvd_2.lps_state_with_intr = 0;
#endif

#ifdef _CCH_SNIFF_NEG_TIMEOUT_
    g_efuse_rsvd_2.sniff_neg_time = 0;
    g_efuse_rsvd_2.enable_sniff_neg = 0;
    g_efuse_rsvd_2.rsvd_for_sco_slot = 0;
#endif


#endif

#ifdef _YL_LPS_EFUSE_FORCE
    g_efuse_lps_setting_5.disable_never_enter_lps_when_usb_active = 1;
    g_efuse_lps_setting_2.timer2_lps_on = 1;                    // 1b; enable
    otp_str_data.bt_deep_sleep_mode_threshold = 0x80;
#ifdef _MODI_LPS_AFTER_RTL8821B_TC_
    g_efuse_lps_setting_2.le_lps_enable = 1;
    g_efuse_lps_setting_2.adjust_lps_scan_interval = 1;
    g_efuse_lps_setting_2.lps_nowakeup_when_no_scan = 0;
#endif

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
    g_efuse_rsvd_2.enable_deep_lps_mode = 1;
    g_efuse_rsvd_2.le_link_lps_enable = 0;
    g_efuse_rsvd_2.lps_state_with_intr = 1;
#endif

#endif

#ifdef _DLPS_SIMU_TEST_
    g_efuse_lps_setting_5.disable_never_enter_lps_when_usb_active = 1;
    g_efuse_lps_setting_2.timer2_lps_on = 1;                    // 1b; enable
    g_efuse_rsvd_2.lps_state_with_intr = 1;
    otp_str_data.power_seq_param |= 1;
    sleep_mode_param.lps_log_enable = 1;
#ifdef _DLPS_SIMU_EN_      
    g_efuse_rsvd_2.enable_deep_lps_mode = 1;
#endif    
#endif

#ifdef LPS_NEW_CAL_MODE
#ifdef _NEW_BTON_DESIGN_AFTER_RTL8821B_TC_

    // Move LPS ini to bton_32k_cal_ini

    BTON_5C_REG_S_TYPE bton_5c;
    bton_5c.d32 = VENDOR_READ(BTON_5C_REG);
    bton_5c.TEMP_COMP = 1;
    VENDOR_WRITE(BTON_5C_REG, bton_5c.d32);

    POW_OPTION_AND_32K_CTRL_S_TYPE bton_32k_reg;
    bton_32k_reg.d32 = VENDOR_READ(BTON_POW_OPTION_AND_32K_CTRL);
    bton_32k_reg.b.reserved_0_13 &= 0x3FFE;
    bton_32k_reg.b.reserved_0_13 |= 0;   // CLKGEN0 // 8723B: Only Bit 0, 8821a" Only Bit[12:11]
    VENDOR_WRITE(BTON_POW_OPTION_AND_32K_CTRL, bton_32k_reg.d32);

#else

    // Choose Use New 32k Calibration
    if(g_efuse_lps_setting_5.lps_use_new_cal)
    {
        BTON_32K_CTRL_REG_S_TYPE bton_32k_ctrl;
        BTON_32K_CSR0_REG_S_TYPE bton_32k_csr0;

        bton_32k_ctrl.d32 = VENDOR_READ(BTON_32K_CTRL_REG);
        bton_32k_csr0.d32 = VENDOR_READ(BTON_32K_CSR0_REG);
        bton_32k_ctrl.OSC32K_DBG_SEL = 1;
        bton_32k_ctrl.cnt_read_mode = 1;    // Enable report
        bton_32k_csr0.inc_mode = 0;
        bton_32k_csr0.num_32k_cyc = 3; // 16(0),32(1),64(2), 128(3) x 256 cycle option

        VENDOR_WRITE(BTON_32K_CTRL_REG, bton_32k_ctrl.d32);
        VENDOR_WRITE(BTON_32K_CSR0_REG, bton_32k_csr0.d32);
    }
    else
    {
        BTON_32K_CSR0_REG_S_TYPE bton_32k_csr0;
        BTON_32K_TEST_REG_S_TYPE bton_32k_test;
        BTON_32K_CTRL_REG_S_TYPE bton_32k_ctrl;


        bton_32k_csr0.d32 = VENDOR_READ(BTON_32K_CSR0_REG);
        bton_32k_test.d32 = VENDOR_READ(BTON_32K_TEST_REG);
        bton_32k_ctrl.d32 = VENDOR_READ(BTON_32K_CTRL_REG);

	RT_BT_LOG(BLUE, YL_DBG_HEX_9, 9, bton_32k_csr0.d32, bton_32k_test.d32, bton_32k_ctrl.d32,
            bton_32k_csr0.kt_lim,
            bton_32k_csr0.criter0, bton_32k_csr0.criter1,
            bton_32k_csr0.num_32k_cyc, bton_32k_csr0.num_settle,
            bton_32k_test.center_cnt_fref);

        bton_32k_csr0.num_settle = 1;
        bton_32k_csr0.criter0 = 1;
        bton_32k_csr0.criter1 = 3;
        bton_32k_csr0.num_32k_cyc = 11; // 11 30000
        bton_32k_csr0.kt_lim = 0x20;
        bton_32k_csr0.RCAL_h = 0;

        VENDOR_WRITE(BTON_32K_CSR0_REG, bton_32k_csr0.d32);

        bton_32k_test.tm_RCAL_32k = 0x1800;
        bton_32k_test.center_cnt_fref = 30000;

        VENDOR_WRITE(BTON_32K_TEST_REG, bton_32k_test.d32);

//********************
        bton_32k_ctrl.cnt_read_mode = 1;    // Enable report
//********************

        bton_32k_ctrl.OSC32K_RES_COMP = 1;   // (0227) orignal: 1   (0301) 5920 test: 0
        VENDOR_WRITE(BTON_32K_CTRL_REG, bton_32k_ctrl.d32);

//********************
        POW_OPTION_AND_32K_CTRL_S_TYPE bton_32k_reg;
        bton_32k_reg.d32 = VENDOR_READ(BTON_POW_OPTION_AND_32K_CTRL);

        bton_32k_reg.b.reserved_0_13 &= 0x3FFE;
        bton_32k_reg.b.reserved_0_13 |= 0;   // CLKGEN0 // 8723B: Only Bit 0, 8821a" Only Bit[12:11]
        VENDOR_WRITE(BTON_POW_OPTION_AND_32K_CTRL, bton_32k_reg.d32);
//********************

    }
#endif
#endif

    otp_str_data.efuse_pow_setting_1_d8 |= BIT6; // set lop_sop_low_power_setting_en = 1 //
    otp_str_data.power_seq_param |= BIT3; // set efuse_pow_option_en = 1 //
    otp_str_data.power_option &= ~(BIT10|BIT11|BIT12); // set lop_ackf, sop_ackf, sop_erck = 0 //

    if(g_efuse_lps_setting_3.use_ext_32k)
    {
        otp_str_data.power_option &= ~(BIT13); // set sop_pdn_32k_ckgen = 0
    }
	else
    {
        otp_str_data.power_option |= (BIT13); // set sop_pdn_32k_ckgen = 1
    }


    if (g_efuse_lps_setting_2.timer2_mode_sel)
    {
        /* record timer2 init value here to reduce some routine logic */
        g_timer2_init_value = otp_str_data.lps_mode_max_time;
    }



#endif
}

/**
 * Resets the baseband layer. It also initialises the radio.
 * Also loads the one time info like device BD-ADDR and parity bits.
 * It enables the appropriate interrupts.
 *
 * \param None.
 *
 * \return API_SUCCESS, if the operation is successful.
 */
API_RESULT lc_baseband_reset(void)
{
    UINT8 i;

    DEF_CRITICAL_SECTION_STORAGE;

    /* Disable All interrupts */
    BB_write_baseband_register(INTERRUPT_MASK_REGISTER, 0xFFFF);
    BB_write_baseband_register(INTERRUPT_MASK_REGISTER2, 0xFFFF);

    MINT_OS_ENTER_CRITICAL();
    for(i = 0; i < 8; i++)
    {
        /* Issue make=0, acl=1 and execute instruction to each slave */
        BB_write_baseband_register(CONNECTOR_REGISTER,
                                   (UINT16) ((i << 5) | 0x08) );
        BB_write_baseband_register(INSTRUCTION_REGISTER, BB_EXECUTE);

        /* Write Invalid Pkt in all LUTs. */
        BB_write_baseband_register(reg_MASTER_LOWER_LUT[i],
                                   LC_INVALID_PACKET_TYPE );

        /* Clear the active bit. */
        AND_val_with_bb_reg(reg_MASTER_UPPER_LUT[i],
                            (UINT16) (~ACTIVE_BIT));
    }

    for (i = 0; i < 4; i++)
    {
        /* Write NULL Pkt in all LUTs. */
        BB_write_baseband_register(reg_SCA_SLAVE_LOWER_LUT[i],
                                   LC_SLAVE_DEFAULT_PACKET_TYPE);
        /* Clear the active bit. */
        AND_val_with_bb_reg(reg_SCA_SLAVE_UPPER_LUT[i],
                            (UINT16) (~ACTIVE_BIT));
    }

    MINT_OS_EXIT_CRITICAL();

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
    lc_baseband_partial_reset(FALSE);
#else
    lc_baseband_partial_reset();
#endif

    lc_init_priority_settings();

#ifdef _YL_LPS
    lc_init_lps_settings();
#endif

#ifdef _2801_BTON_DESIGN_
    LC_INIT_SLEEP_MODE_PARAM();
#endif

#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8723_RCP_
    if (rcp_lc_baseband_reset != NULL)
    {
        rcp_lc_baseband_reset();
    }
#endif
#endif

    return API_SUCCESS;
}
#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
API_RESULT dlps_lc_baseband_reset(void)
{

    /* Disable All interrupts */
    BB_write_baseband_register(INTERRUPT_MASK_REGISTER, 0xFFFF);
    BB_write_baseband_register(INTERRUPT_MASK_REGISTER2, 0xFFFF);

    lc_baseband_partial_reset(TRUE);


    return API_SUCCESS;
}
#endif

/**
 * Resets the LC layer. It also creates the LC related timers.
 * Resets the LC layer variables, and state machines.
 *
 * \param None.
 *
 * \return None.
 */
void lc_module_reset(void)
{
    API_RESULT status;

    /* To enable external bb signals thro GPIO */
#ifdef POWER_SAVE_FEATURE
    pf_clear_dsm_exit();
#endif

    LC_EXIT_SM_MODE();

    lc_kill_scan_mode();

    if(periodic_inquiry_timer != NULL)
    {
        status = OS_DELETE_TIMER(& periodic_inquiry_timer);
        if(status != BT_ERROR_OK)
        {
#ifdef ENABLE_LOGGER_LEVEL_2
            LC_LOG_ERROR(LOG_LEVEL_LOW, UNABLE_TO_DELETE_PERI_INQ_TIMER, 0,0);
#endif
        }
    }

    lc_reset_global_variables();

    aclq_init();
    pduq_init();

#ifdef SCO_OVER_HCI
    bz_isoch_init_q();
#endif /* SCO_OVER_HCI */

    return;
}


/**
 * Initializes the LC layer. This function also resets the LC level
 * variables. It installs the baseband interrupt handler. Also
 * initialises the radio.
 *
 * \param None.
 *
 * \return API_SUCCESS, if the operation is successful, API_FAILURE otherwise.
 */
API_RESULT lc_init(void)
{
    UINT16 random_number;
    API_RESULT status = BT_ERROR_OK;

    /* To enable external bb signals through GPIO */
#ifdef POWER_SAVE_FEATURE
    pf_clear_dsm_exit();
#endif
    lc_reset_global_variables();

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
    lc_init_radio(FALSE);
#else
    lc_init_radio();
#endif

#ifdef _DAPE_TEST_SHUTDN_RF_WHEN_RESET
    rtk_write_modem_radio_reg(0x00, TYPE_RF, 0x00);
#endif

#ifdef VER_3_0
    if (IS_BT30)
    {
        lmp_init_epc();
    }
#endif

#if 0 /* we have same delay in lc_baseband_reset function */
#ifdef FOR_SIMULATION
    UINT32 i;
    for(i = 0; i < 3000; i++); // to avoid native_clock read 0 when polling reset (1----->0->2---->3)
#else
    pf_delay(50);                  /* we need to wait so long ? (about 50 ms) - austin  */
#endif
#endif

    random_number = lc_generate_rand_number(MAX_RAND);

    /* Create a timer for random backoff */
    if (random_backoff_timer == NULL)
    {
        status = OS_CREATE_TIMER(ONESHOT_TIMER, &random_backoff_timer,
                lc_handle_random_backoff_timeout, NULL,
                SLOT_VAL_TO_TIMER_VAL(random_number));
    }

    if( status != BT_ERROR_OK )
    {
#ifdef ENABLE_LOGGER_LEVEL_2
        LC_LOG_INFO(LOG_LEVEL_LOW, RANDOM_BACKOFF_TIMER_CREATION_FAILED,0,0);
#endif
        return API_FAILURE;
    }

    lc_module_reset();
    lc_baseband_reset();

    return API_SUCCESS;
}
#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
API_RESULT dlps_lc_init(void)
{

    /* To enable external bb signals through GPIO */
#ifdef POWER_SAVE_FEATURE
    pf_clear_dsm_exit();
#endif

    lc_init_radio(TRUE);


#ifdef _DAPE_TEST_SHUTDN_RF_WHEN_RESET
    rtk_write_modem_radio_reg(0x00, TYPE_RF, 0x00);
#endif

    dlps_lc_baseband_reset();

    return API_SUCCESS;
}
#endif
/**
 * Shuts down the LC layer. This function deletes the LC layer tasks.
 * It also deletes the buffer pools synchronous connections. It also
 * releases all the timers locked by LC.
 *
 * \param None.
 *
 * \return API_SUCCESS, if the operation is successful, API_FAILURE otherwise.
 */
API_RESULT lc_shutdown(void)
{
#ifndef _DONT_SUPPORT_OS_SHUTDOWN_

    API_RESULT result;

    /* Delete all the created tasks */
    result = OS_DELETE_TASK(lc_tx_task_handle);
    if(result!= BT_ERROR_OK)
    {
#ifdef ENABLE_LOGGER_LEVEL_2
        LC_LOG_ERROR(LOG_LEVEL_LOW, LC_SHUTDOWN_UNABLE_TO_DELETE_LC_TX_TASK_HANDLE,0,0);
#endif
        return (API_FAILURE);
    }

    result = OS_DELETE_TASK(lc_rx_task_handle);
    if(result!= BT_ERROR_OK)
    {
#ifdef ENABLE_LOGGER_LEVEL_2
        LC_LOG_ERROR(LOG_LEVEL_LOW,LC_SHUTDOWN_UNABLE_TO_DELETE_LC_RX_TASK_HANDLE,0,0);
#endif
        return (API_FAILURE);
    }

    /*
     * ACL Buffer in the host==> host controller direction. When there
     * is ACL data that arrives from the host, THIS buffer is used.
     * Will be ALLOCATED by the Transport driver, and FREED by the
     * MindTree task once the data is consumed.
     */
    result = OS_DELETE_POOL(acl_data_pool_id);

    if( result != BT_ERROR_OK )
    {
#ifdef ENABLE_LOGGER_LEVEL_2
        LC_LOG_ERROR(LOG_LEVEL_LOW,FAILED_TO_DELETE_ACL_DATA_BUFFER_POOL,0,0);
#endif
        return(API_FAILURE);
    }

#if defined(SCO_OVER_HCI) || defined(COMPILE_ESCO)
    /*
     * SCO Buffer in the host==> host controller direction.
     * ALLOCED by Transport driver, FREED by MindTree task system.
     */
    result = OS_DELETE_POOL(synchronous_data_pool_id);

    if(result != BT_ERROR_OK)
    {
#ifdef ENABLE_LOGGER_LEVEL_2
        LC_LOG_ERROR(LOG_LEVEL_LOW, FAILED_TO_DELETE_SCO_DATA_BUFFER_POOL,0,0);
#endif
        return(API_FAILURE);
    }
#endif /* defined(SCO_OVER_HCI) || defined(COMPILE_ESCO) */

    /*
     * ACL Buffer in the HOST CONTROLLER ==> HOST direction. Will
     * be ALLOCED by the MindTree task system, when there is data
     * available from the baseband.
     * FREED by the Transport driver, once the data is consumed.
     */
    result = OS_DELETE_POOL(acl_data_to_host_pool_id);

    if(result != BT_ERROR_OK)
    {
#ifdef ENABLE_LOGGER_LEVEL_2
        LC_LOG_ERROR(LOG_LEVEL_LOW, FAILED_TO_DELETE_ACL_DATA_BUF_POOL_TO_HOST, 0,0);
#endif

        return(API_FAILURE);
    }

#if defined(SCO_OVER_HCI) || defined(COMPILE_ESCO)
    /*
     * SCO Buffer in the HOST CONTROLLER ==> HOST direction.
     * ALLOCED by the MindTree task system, FREED by the Transport driver.
     */
    result = OS_DELETE_POOL(synchronous_data_to_host_pool_id);

    if(result != BT_ERROR_OK)
    {
#ifdef ENABLE_LOGGER_LEVEL_2
        LC_LOG_ERROR(LOG_LEVEL_LOW, FAILED_TO_DELETE_SCO_DATA_BUF_POOL_TO_HOST,0,0);
#endif

        return(API_FAILURE);
    }
#endif /* defined(SCO_OVER_HCI) || defined(COMPILE_ESCO) */

#endif /* ifndef _DONT_SUPPORT_OS_SHUTDOWN_ */

    return API_SUCCESS;
}


/**
 * Programs baseband for core protocols. This function checks the current
 * state of the baseband and programs inquiry or paging.
 *
 * \param cmd_buffer HCI command buffer pointer.
 *
 * \return HCI_COMMAND_SUCCEEDED, if the operation is successful, the
 *         appropriate HCI layer eror code otherwise.
 */
UCHAR lc_handle_baseband_commands(HCI_CMD_PKT *cmd_buffer)
{
    UCHAR status = HCI_COMMAND_SUCCEEDED;

    UCHAR (*f)(HCI_CMD_PKT*);

#ifdef COMPILE_PERIODIC_INQUIRY
    if (lmp_periodic_inquiry == TRUE)
    {
        if (lc_kill_periodic_inquiry()!= API_SUCCESS)
        {
            BT_FW_HCI_ERR(HCI_PERIODIC_INQUIRY_MODE_COMMAND_FAILED,0,0);
        }
        else
        {
            lmp_self_device_data.device_status = LMP_IDLE ;
            hci_baseband_cmd_pending = FALSE;

            /* Reset the inquiry result table */
            memset(lmp_inquiry_result_data, 0,
                   LMP_MAX_INQUIRY_RESULT_DATA *sizeof(LMP_INQUIRY_RESULT_DATA));

            lmp_num_inq_resp_received = 0 ;
        }

        lmp_periodic_inquiry = FALSE;

        RT_BT_LOG(GREEN, BB_MSG_KILL_PERIODIC_INQUIRY, 0, 0);
        //	hci_generate_command_complete_event(cmd_opcode, HCI_COMMAND_SUCCEEDED, 0);
    }
#endif

    /* Check if the baseband is not already in Inquiry or paging. */
    if ((lmp_self_device_data.lc_cur_dev_state &
            (LC_CUR_STATE_INQUIRY | LC_CUR_STATE_PAGE) ) != 0)
    {
            RT_BT_LOG(RED, LC_774, 4, lmp_self_device_data.lc_cur_dev_state,
                            			lc_paging_bd_addr[0],
                            			lc_paging_bd_addr[1],
                            			lc_paging_bd_addr[2]);
        return COMMAND_DISALLOWED_ERROR;
    }

    LC_EXIT_SM_MODE();

    /* Check if scans are enabled in baseband.
       These will be temporarily killed. */
    if (lmp_self_device_data.scan_enable != 0x0)
    {
        lc_kill_scan_mode();
    }

    switch (cmd_buffer->cmd_opcode)
    {
        case HCI_INQUIRY_OPCODE:
            f = lc_start_inquiry;

#ifdef MINICARD_BT_LED_CONTROL
            BT_LED_WPAN_ON();
#endif
            break;

        case HCI_CREATE_CONNECTION_OPCODE: /* Fall through. */
        case HCI_REMOTE_NAME_REQUEST_OPCODE:
#ifdef _SUPPORT_CSB_RECEIVER_
        case HCI_TRUNCATED_PAGE_OPCODE:
#endif
            f = lc_start_paging;

#ifdef MINICARD_BT_LED_CONTROL
             BT_LED_WPAN_ON();
#endif
            break;

        default :
#ifdef ENABLE_LOGGER_LEVEL_2
            LC_LOG_ERROR(LOG_LEVEL_LOW,LC_INVALID_COMMAND,0,0);
#endif
            //RT_BT_LOG(GRAY, LC_813, 0, 0);

            return UNSPECIFIED_ERROR;
    }

    status = (*f)(cmd_buffer);

    return status;
}


/**
 * Queues LMP-PDUs at the baseband. This function queues the packet pointer
 * in the LMP-connection-entity. It then checks the status of the ACL
 * scheduler and writes the payload into the baseband tx-fifo.
 *
 * \param ce_index Index to the lmp-connection-entity
 *        pkt Pointer to the LMP_PDU packet that is to
 *        be queued at the baseband.
 *        piconet_id : Physical piconet ID of the packet.
 *
 * \return API_SUCCESS, if the operation is successful, API_FAILURE otherwise.
 */
API_RESULT lc_handle_lmp_pdus(UINT16 ce_index, LMP_PDU_PKT *pkt,
                              UCHAR phy_piconet_id)
{
    OS_SIGNAL signal;

    signal.type = LC_QUEUE_PDU_SIGNAL;
    signal.param = (OS_ADDRESS)((UINT32) (pkt));
    signal.ext_param = (OS_ADDRESS)((UINT32) (phy_piconet_id));

    OS_SEND_SIGNAL_TO_TASK(lc_tx_task_handle, signal);

    pduq_queue_pdu(pkt,phy_piconet_id, ce_index);

    return API_SUCCESS;
}


/**
 * Kills inquiry process in the baseband. It also resets the LC-state
 * machine to idle state.
 *
 * \param None.
 *
 * \return None.
 */
void lc_kill_inquiry(void)
{
    UINT32 cur_clk;
    UINT32 tar_clk;

    cur_clk = BB_read_native_clock();
    cur_clk >>= 2;

    tar_clk = 0;

    /* wait 1.28ms here */
    while( (cur_clk + 1) >= tar_clk)
    {
        tar_clk = BB_read_native_clock();
        tar_clk >>= 2;
    }
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

    lc_set_lc_cur_device_state(LC_CUR_STATE_IDLE);

    if(lmp_self_device_data.device_status == LMP_INQUIRY)
    {
        lmp_self_device_data.device_status = LMP_IDLE;
    }

    lc_check_and_enable_scans_in_scatternet();

    /* Write default tx-level on BC-AM-ADDR */
    lc_write_bc_pwr_level(RADIO_POWER_VAL);

#ifdef COMPILE_DYNAMIC_POLLING
    lc_update_dynamic_polling();
#endif
#ifndef _DAPE_TEST_NEW_HW_BLOCK_ACL_FOR_LE
#ifdef LE_MODE_EN
#ifdef _DAPE_TEST_BLOCK_LEGACY_FOR_LE
    /* dape test for LE slave + inquiry */
    ll_driver_block_legacy_for_le(FALSE);
#endif
#endif
#endif
    return;
}


#ifdef _CCH_PAGE_CON_
/**
 * Kills paging process in the baseband. It also resets the LC-state
 * machine to idle state.
 *
 * \param bd_addr to be removed.
 *
 * \return None.
 */
API_RESULT lc_kill_paging(UCHAR *bd_addr, UCHAR reason)
{
    UINT16 ce_index;
    LMP_CONNECTION_ENTITY* ce_ptr;
    UCHAR is_paging_bd_addr;

#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8723_RCP_
    UCHAR return_status = 0;

    if (rcp_lc_kill_paging != NULL)
    {
        if ( rcp_lc_kill_paging((void *)&return_status, bd_addr, reason) )
        {
            return return_status;
        }
    }
#endif
#endif

    is_paging_bd_addr =
            (( lc_paging_bd_addr[2] == ((bd_addr[5]<<8) | bd_addr[4]) ) &&
             ( lc_paging_bd_addr[1] == ((bd_addr[3]<<8) | bd_addr[2]) ) &&
             ( lc_paging_bd_addr[0] == ((bd_addr[1]<<8) | bd_addr[0]) )) ?
             TRUE : FALSE;

    if(is_paging_bd_addr)
    {
        if (LMP_GET_CE_INDEX_FROM_BD_ADDR(bd_addr, &ce_index) != API_SUCCESS)
        {
            return HCI_COMMAND_SUCCEEDED;
        }

        ce_ptr = &lmp_connection_entity[ce_index];

        if (ce_ptr->hci_cmd_bits & REMOTE_NAME_REQ_CON_BIT_MASK)
        {
            RT_BT_LOG(GRAY, CCH_DBG_116, 1, ce_ptr->hci_cmd_bits);
            /* Name Request was issued on an existing ACL link */
            hci_generate_remote_name_request_complete_event(
                CONNECTION_TERMINATED_LOCAL_HOST_ERROR, ce_index);
            ce_ptr->hci_cmd_bits &= (~REMOTE_NAME_REQ_CON_BIT_MASK);
        }

        if (ce_ptr->connect_reason == reason)
        {


            if(( reason == CON_REASON_CREATE_CON ) && ( ce_ptr->ce_status == LMP_CONNECTED) )
            {  // Create Connection Complete
               // DONOT Kill Paging and DONOT Generate Connection Complete Event
               return ACL_CONNECTION_EXISTS_ERROR;

            }
            else
            {

               {
                   if( ce_ptr->ce_status == LMP_BB_HL_CONNECTED)
                   {
                       // 8821 need to add this
                       // ce_ptr->connect_reason = 0;
                       // Not Yet Create Connection Complete
                       // But HLC Already
                       lc_kill_hardware_level_conn_in_scatternet(ce_index);
                   }
                   else if( reason == CON_REASON_CREATE_CON )
                   {
                       // Create Connection still in paging state
                       // Kill Paging and Generate Connection Complete Event

                       hci_generate_connection_complete_event(
                           (UCHAR)NO_CONNECTION_ERROR,
                           ce_ptr->connection_type.connection_handle,
                           ce_ptr->bd_addr,
                           ce_ptr->connection_type.link_type,
                          (UCHAR) bz_auth_get_encryption_mode(ce_index));
                       hci_baseband_cmd_pending = FALSE;
                   }
               }

                LMP_REMOVE_BD_ADDR_FROM_HASH(ce_ptr->bd_addr);
                lc_update_scatternet_state_deletion(ce_ptr->phy_piconet_id);

                // BB_KILL_PAGE will kill page scan at the same time
                BB_write_baseband_register(INSTRUCTION_REGISTER, BB_KILL_PAGE);

                lc_set_lc_cur_device_state(LC_CUR_STATE_IDLE);
                lmp_self_device_data.device_status = LMP_IDLE;

                lc_check_and_enable_scans_in_scatternet();

 //               RT_BT_LOG(GRAY, CCH_DBG_116, 1, ce_ptr->am_addr);
 //               RT_BT_LOG(GRAY, CCH_DBG_106, 2, ce_index, lmp_connection_entity[ce_index].entity_status);

                if (lmp_release_am_addr_ppi(ce_ptr->am_addr,
                                        ce_ptr->phy_piconet_id) != API_SUCCESS)
                {
                   lmp_release_entity_to_ce_database(ce_index);
                    LMP_ERR(LOG_LEVEL_HIGH, LMP_RELEASE_AM_ADDR_FAILED,0,0);
                    return HCI_COMMAND_SUCCEEDED;
                }

                lmp_assigned_ce_index--;
                //RT_BT_LOG(GRAY, CCH_DBG_116, 1, 0xff);
                return HCI_COMMAND_SUCCEEDED;
            }
        }
    }

    if(reason == CON_REASON_CREATE_CON)
    {  // CON_REASON_CREATE_CON
        return NO_CONNECTION_ERROR;
    }
    else
    {  // CON_REASON_REMOTE_NAME_REQ
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }
}
#endif

/**
 * Updates packet type in the LUT, ifthe current packet type is
 * same as param1.
 *
 * \param current_val Current contents of LUT.
 * \param current_val New contents to be updated in LUT.
 * \param current_val Address of LUT
 *
 * \return None.
 */
void lc_check_and_update_pkt_in_lut(UINT16 current_val,
                                    UINT16 new_val, UINT16 address)
{
    UINT16 read;
    DEF_CRITICAL_SECTION_STORAGE;

    MINT_OS_ENTER_CRITICAL();

    read = BB_read_baseband_register(address);

    if(read == current_val)
    {
        BB_write_baseband_register(address, new_val);
    }

    MINT_OS_EXIT_CRITICAL();

    return;
}

#ifdef COMPILE_HOLD_MODE
/**
 * Programs the hold mode to the baseband. This will be called
 * when LMP level negotiation of hold mode parameters is completed.
 *
 * \param ce_index Index to lmp-connection-entity database for the connection
 *
 * \return API_SUCCESS on successful programming.
 */
API_RESULT lc_start_hold_mode(UINT16 ce_index)
{
    UCHAR am_addr;
    UCHAR tolerance = 0;
    UINT16 hold_interval;

    LMP_CONNECTION_ENTITY *ce_ptr;
    LUT_EXTENSION_TABLE *ex_lut;
    UCHAR lut_index;
    UINT32 hold_offset;
    UCHAR instant_status;
    UINT32 cur_clk;

    DEF_CRITICAL_SECTION_STORAGE;

    ce_ptr = &lmp_connection_entity[ce_index];
    am_addr = ce_ptr->am_addr;
    hold_interval = ce_ptr->hold_mode_interval;

    lut_index = lc_get_lut_index_from_phy_piconet_id(
                    am_addr, ce_ptr->phy_piconet_id);

    ex_lut = &lut_ex_table[lut_index];

    if( (ce_ptr->remote_dev_role == MASTER) &&
            (hold_interval > LC_MIN_HOLD_INT_FOR_XTOL) )
    {
        /* Program x_val_for_tolerance. */
        //RT_BT_LOG(GRAY, LC_1015, 0, 0);

        tolerance = lc_program_tolerance(hold_interval, ce_index);
        hold_interval = (UINT16) (hold_interval - (tolerance >> 1));
        hold_interval += 1;
    }

    MINT_OS_ENTER_CRITICAL();
    lc_get_clock_in_scatternet(&cur_clk, ce_ptr->phy_piconet_id);

    cur_clk++;
    instant_status =
        lc_check_for_clock_wrap_around(cur_clk, (ce_ptr->hold_instant << 1) );

    /* Hold instant passed. */
    if(instant_status == BT_CLOCK_MORE_THAN_12_HRS_AWAY)
    {
        MINT_OS_EXIT_CRITICAL();

        hci_generate_mode_change_event(INSTANT_PASSED_ERROR, ce_index,
                                       LP_ACTIVE_MODE, 0);

        return API_FAILURE;
    }

    BB_stop_tpoll(am_addr, ce_ptr->phy_piconet_id);

    if(instant_status == BT_CLOCK_CLK_WRAP_AROUND_CASE)
    {
        /* Offset, instant etc. are 1-27 bits.
            So, use BT_CLOCK_27_BITS mask here. */
        hold_offset = ((ce_ptr->hold_instant | (BT_CLOCK_27_BITS + 1))) + (tolerance >> 1);
        LMP_LOG_INFO(LOG_LEVEL_HIGH, CLOCK_WRAP, 0, 0);
    }
    else
    {
        /* CAUTION : The tolerance value is to be added to the Hold instant.
         * This is because baseband will automatically subtract tolerance
         * from programmed hold value. Hence, we compensate for that in fw.
         */
        hold_offset = ce_ptr->hold_instant + (tolerance >> 1);
    }

//	LMP_LOG_INFO(LOG_LEVEL_HIGH, HOLD_OFFSET_PROGRAMMED, 1, hold_offset);

    BB_write_baseband_register(
        (UCHAR) SNIFF_SLOT_OFFSET_INTERVAL_REGISTER, (UINT16)(hold_offset));

    BB_write_baseband_register(CONNECTOR_REGISTER,
               (UINT16) ((am_addr << 5) | (ce_ptr->phy_piconet_id << 11)) );

    BB_write_baseband_register(TPOLL_HOLD_SNIFF_INTERVAL_REGISTER,
                               hold_interval);

    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_HOLD);

    MINT_OS_EXIT_CRITICAL();

    UINT8 state = 0;

    if(instant_status == BT_CLOCK_MORE_THAN_12_HRS_AWAY)
    {
        state = 1;
    }
    else if(instant_status == BT_CLOCK_CLK_WRAP_AROUND_CASE)
    {
        /* Offset, instant etc. are 1-27 bits.
        So, use BT_CLOCK_27_BITS mask here. */
        state = 2;
    }

    /* Disabling retransmission window of esco links on this ACL link */
    LC_DISABLE_ESCO_RETX(ce_index);

    RT_BT_LOG(GRAY, LC_1099, 2, hold_offset, state);

    if (ce_ptr->link_supervision_timeout != 0)
    {
        /* Re-start the SUP TO. */
        if(OS_START_TIMER(ce_ptr->supervision_timeout_handle,
             (UINT16)(SLOT_VAL_TO_TIMER_VAL(LMP_SUPERVISION_TIMER_RESOLUTION)))
              != BT_ERROR_OK)
        {
#ifdef ENABLE_LOGGER_LEVEL_2
            LC_LOG_INFO(LOG_LEVEL_LOW,UNABLE_TO_START_SUPERVISION_TIMEOUT_FOR_AM_ADDR,1,am_addr);
#endif
        }
    }

    /* Transition mode, make sure that LUT has POLL packet, as master. */
    if(ce_ptr->remote_dev_role == SLAVE)
    {
        lc_check_and_update_pkt_in_lut(LC_INVALID_PACKET_TYPE,
                LC_MASTER_DEFAULT_PACKET_TYPE, ex_lut->lower_lut_address);

        ce_ptr->cont_poll_count = LC_CONT_POLL_SLOT_COUNT;
    }

    LC_LOG_INFO(LOG_LEVEL_LOW,OFF_INST_INTERV,3,
        hold_offset,ce_ptr->hold_instant,hold_interval);
    return API_SUCCESS;
}
#endif /* COMPILE_HOLD_MODE */

#ifdef COMPILE_SNIFF_MODE

/**
 * Programs the sniff mode to the baseband. This will be called
 * when LMP level negotiation of sniff mode parameters is completed.
 *
 * \param ce_index Index to lmp-connection-entity database for the connection
 * \param ssr_flag   Flag - 0 => Entering sniff
 *                          1 => Entering ssr
 *                          2 => Exiting ssr
 *
 * \return API_SUCCESS on successful programming.
 */
API_RESULT lc_start_sniff_mode(UINT16 ce_index, UCHAR ssr_flag)
{
    UINT16 Dsniff,Tsniff;
    UINT16 sniff_attempt,sniff_timeout;
    UCHAR am_addr;
    UINT32 master_clock;
    UINT16 offset;
    UINT16 delta_offset;
    UINT32 new_dsniff;
    UCHAR con_am_addr;
    UCHAR lut_index;
    UCHAR phy_piconet_id;
    UCHAR rem_dev_role;
    LMP_CONNECTION_ENTITY *ce_ptr;
    LUT_EXTENSION_TABLE *lut_ptr;

#ifdef _ROM_CODE_PATCHED_
    /* Reserved com code patch by yilinli:
      * e.g.
      *   1. issue FW direct LPO calibratioin: e.g. bton_32k_cal_en() (bton_32k_cal_en())
      *   2. set HW calbration: e.g. hw_thermal/timer_cal_func()
      *   3. FW tolerance record reset OR
      *   (4. xtol[ce_index]=0; xtol_prev[ce_index]=0;)
      */
    if (rcp_lc_start_sniff_head_func != NULL)
    {
        if ( rcp_lc_start_sniff_head_func((void*)(&ce_index), &ssr_flag ) )
        {
            return API_SUCCESS;
        }
    }
#endif

    DEF_CRITICAL_SECTION_STORAGE;

    ce_ptr = &lmp_connection_entity[ce_index];
    rem_dev_role = ce_ptr->remote_dev_role;

    Dsniff = ce_ptr->sniff_slot_offset;
    Tsniff = ce_ptr->sniff_interval;
    sniff_attempt = ce_ptr->sniff_attempt;
    sniff_timeout = ce_ptr->sniff_timeout;
    am_addr       = ce_ptr->am_addr;

#ifdef _YL_LPS
    ce_ptr->sniff_xtol = 0;
    ce_ptr->sniff_xtol_prev = 0;
#endif

    phy_piconet_id = ce_ptr->phy_piconet_id;
    lut_index = lc_get_lut_index_from_phy_piconet_id(am_addr, phy_piconet_id);

    lut_ptr = &lut_ex_table[lut_index];

    /* Stop Tpoll timer */
    //RT_BT_LOG(GRAY, LC_1197, 0, 0);

    BB_stop_tpoll(am_addr, ce_ptr->phy_piconet_id);

    if(ssr_flag != 0)
    {
        /* Sniff <-> SSR transition, program INVALID_PACKET_TYPE */
        if(rem_dev_role == SLAVE)
        {
            lc_check_and_update_pkt_in_lut(LC_MASTER_DEFAULT_PACKET_TYPE,
                     LC_INVALID_PACKET_TYPE, lut_ptr->lower_lut_address);
        }
    }

    MINT_OS_ENTER_CRITICAL();

    lc_get_clock_in_scatternet(&master_clock, phy_piconet_id);

    /* Calculate and program Dsniff, Tsniff. */
    master_clock >>= 1;
    master_clock += 1;

    /* MSB is also made to zero -> timing control flags */
    master_clock &= BT_CLOCK_26_BITS;

    if(ce_ptr->ssr_data.tsniff == 0 && ssr_flag == 1)
    {
        MINT_OS_EXIT_CRITICAL();

        LC_LOG_ERROR(LOG_LEVEL_LOW, SSR_TSNIFF_IS_ZERO,0,0);
        return API_FAILURE;
    }

    if((master_clock != 0) && (Tsniff != 0))
    {
        offset = (UINT16) (master_clock % Tsniff);
    }
    else
    {
        offset = 0;
    }

    delta_offset = (UINT16) (Tsniff - offset);
    new_dsniff = (delta_offset + Dsniff);
    if(new_dsniff > Tsniff)
    {
        new_dsniff = (new_dsniff - Tsniff);
    }

    if(ssr_flag == 1)
    {
        UINT32 ssr_instant = ce_ptr->ssr_data.ssr_instant & BT_CLOCK_26_BITS;
        INT32 diff;
        Tsniff = ce_ptr->ssr_data.tsniff;

        if(master_clock > ssr_instant)
        {
            diff = master_clock - ssr_instant;
        }
        else
        {
            diff = BT_CLOCK_27_BITS - ssr_instant + master_clock + 1;
        }

//		LC_LOG_INFO(LOG_LEVEL_LOW,DFF_MC_INS,3,diff,master_clock,ssr_instant);

        /* ssr_instant + ((diff/Tsniff) + 1)*Tsniff */
        new_dsniff = (diff/Tsniff);
        new_dsniff = (new_dsniff + 1)*Tsniff + ssr_instant;
        new_dsniff = new_dsniff - master_clock;
        new_dsniff &= BT_CLOCK_26_BITS;
    }

    con_am_addr = am_addr;
    con_am_addr <<= 5;

    BB_write_baseband_register(CONNECTOR_REGISTER,
                               (con_am_addr | (ce_ptr->phy_piconet_id << 11)));

#ifdef COMPILE_SNIFF_REDESIGN
    INT32 tar_clk;

    tar_clk = master_clock + new_dsniff;

#ifdef BZ_2_1_2_HID_FTP
    while (new_dsniff < LC_MIN_DSNIFF_VALUE)
    {
        tar_clk += Tsniff;
        new_dsniff += Tsniff;
    }
#else
    if(new_dsniff < LC_MIN_DSNIFF_VALUE)
    {
        tar_clk += Tsniff;
    }
#endif

    ce_ptr->next_instant_in_nat_clk = ((tar_clk - master_clock) << 1)
                                      + BB_read_native_clock();

    BB_write_baseband_register(TPOLL_HOLD_SNIFF_INTERVAL_REGISTER, Tsniff);

    BB_write_baseband_register(SNIFF_SLOT_OFFSET_INTERVAL_REGISTER,
                               (UINT16) tar_clk);

    if(rem_dev_role == MASTER)
    {
        /* IUT is slave. */

        BB_write_baseband_register(SNIFF_TIMEOUT_REGISTER, sniff_timeout);
        BB_write_baseband_register(SNIFF_ATTEMPT_REGISTER, sniff_attempt);
#ifdef _DAPE_EN_8821_MP_MODIFY_SLV_SNIFF_TIMEOUT
        /* this bit makes sniff slave not to count sco pkt into sniff timeout.
           So we should enable this bit most of the time and disable it when
           sniff slave wants to send pkt. After receiving ack of that pkt,
           enable this bit again if there is no sniff slave sending pkt. */
        UINT16 reg21c = BB_read_baseband_register(0x21c);
        reg21c |= BIT6;
        BB_write_baseband_register(0x21c, reg21c);
#endif
    }
    else
    {
        /* IUT is master. */
#ifndef _DAPE_SHORTEN_MASTER_SNIFF_TIMEOUT_FOR_LE_CONN
#ifndef _DAPE_TEST_NEW_HW_SNIFF_MASTER_OK
        BB_write_baseband_register(SNIFF_ATTEMPT_REGISTER,
                                   sniff_attempt + sniff_timeout);
        BB_write_baseband_register(SNIFF_TIMEOUT_REGISTER, 0);
#else
        BB_write_baseband_register(SNIFF_TIMEOUT_REGISTER, sniff_timeout);
        BB_write_baseband_register(SNIFF_ATTEMPT_REGISTER, sniff_attempt);
#endif
#else
        if (SHORTEN_MASTER_SNIFF_TIMEOUT_VAL)
        {
            if (sniff_timeout)
            {
                BB_write_baseband_register(SNIFF_TIMEOUT_REGISTER, 1);
            }
            else
            {
                BB_write_baseband_register(SNIFF_TIMEOUT_REGISTER, 0);
            }
            BB_write_baseband_register(SNIFF_ATTEMPT_REGISTER, sniff_attempt);
        }
        else
        {
            BB_write_baseband_register(SNIFF_TIMEOUT_REGISTER, sniff_timeout);
            BB_write_baseband_register(SNIFF_ATTEMPT_REGISTER, sniff_attempt);
        }
#endif
    }

    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_SNIFF);
#ifndef _CCH_DISABLE_SNIFF_TRAN_
    /* Program sniff transition mode. */
    if ( (ce_ptr->remote_dev_role == MASTER) && (ssr_flag == 0) )
    {
        BB_write_baseband_register(INSTRUCTION_REGISTER,
                                   BB_ENTER_SNIFF_TRAN_MODE);
	    RT_BT_LOG(BLUE, CCH_DBG_043, 0,0);
    }
#endif
    if(ssr_flag == 0)
    {
        if(rem_dev_role == SLAVE)
        {
            lc_cont_crc_rx_cnt[lut_index] = 0;
            lc_cont_crc_tx_cnt[lut_index] = 0;
        }
    }
    MINT_OS_EXIT_CRITICAL();

    lc_update_next_sniff_instant();
    BB_set_sniff_priority_flag();

    LMP_LOG_INFO(LOG_LEVEL_HIGH, SNIFF_REDESIGN, 3,
                            (UINT16)tar_clk, tar_clk, (tar_clk << 1) );

    new_dsniff += LC_BUF_SNIFF_OFFSET_VALUE_FOR_TOL;

#else /* COMPILE_SNIFF_REDESIGN */
    BB_write_baseband_register(TPOLL_HOLD_SNIFF_INTERVAL_REGISTER, Tsniff);

    if(new_dsniff < LC_MIN_DSNIFF_VALUE)
    {
        new_dsniff = new_dsniff + Tsniff - 1;
    }

    BB_write_baseband_register(SNIFF_SLOT_OFFSET_INTERVAL_REGISTER,
                               (UINT16) new_dsniff);

    BB_write_baseband_register(SNIFF_TIMEOUT_REGISTER, sniff_timeout);

    BB_write_baseband_register(SNIFF_ATTEMPT_REGISTER, sniff_attempt);

    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_SNIFF);

    MINT_OS_EXIT_CRITICAL();

    LC_LOG_INFO(LOG_LEVEL_HIGH,SNIFF_PGMED_CONNECTOR,2,
                (con_am_addr | (ce_ptr->phy_piconet_id << 11) ), new_dsniff);
#endif /* COMPILE_SNIFF_REDESIGN */

    /* Sniff transition mode, program POLL/NULL in LUT. */
    if(ssr_flag == 0)
    {
        if(rem_dev_role == SLAVE)
        {
#if 1
            lc_check_and_update_pkt_in_lut(LC_INVALID_PACKET_TYPE,
                     LC_MASTER_DEFAULT_PACKET_TYPE, lut_ptr->lower_lut_address);
            ce_ptr->cont_poll_count = LC_CONT_POLL_SLOT_COUNT;
#else
            BB_write_baseband_register(lut_ptr->lower_lut_address, LC_INVALID_PACKET_TYPE);
            ce_ptr->cont_poll_count = 0;
#endif
            //RT_BT_LOG(GRAY, LC_1379, 0, 0);
        }
        else
        {
            if(new_dsniff > LC_MIN_SNIFF_INT_FOR_XTOL)
            {
                //RT_BT_LOG(GRAY, LC_1385, 0, 0);
                lc_program_tolerance((UINT16)new_dsniff, ce_index);
            }
            lc_check_and_update_pkt_in_lut(LC_INVALID_PACKET_TYPE,
                     LC_SLAVE_DEFAULT_PACKET_TYPE, lut_ptr->lower_lut_address);
        }
    }

    if(rem_dev_role == MASTER)
    {
        /**
         * SSR needs tolerance programming.
         */
        if((ssr_flag != 0) && (new_dsniff > LC_MIN_SNIFF_INT_FOR_XTOL) )
        {
            //RT_BT_LOG(GRAY, LC_1400, 0, 0);
            lc_program_tolerance((UINT16)new_dsniff, ce_index);

            /* Update packet type to invalid */
            lc_check_and_update_pkt_in_lut(LC_SLAVE_DEFAULT_PACKET_TYPE,
                      LC_INVALID_PACKET_TYPE, lut_ptr->lower_lut_address);
        }
    }

#ifdef DETECT_SNIFF_OVERLAP
    if(lmp_self_device_data.no_of_sniff_connections > 1)
    {
        lc_check_for_overlapping_sniff_windows();
    }
#endif

    {
        OS_SIGNAL signal;

        signal.type = LMP_ENTER_SNIFF_MODE;
        signal.param = (void*)((UINT32)Tsniff);

        OS_SEND_SIGNAL_TO_TASK(lmp_task_handle, signal);
    }

    RT_BT_LOG(YELLOW, CCH_DBG_046, 3,Tsniff, sniff_attempt, sniff_timeout);

    if( lut_index >= 1)
    {
#ifdef _YL_LPS
        lc_sniff_cont_count[lut_index - 1] = 0;
#endif

#ifdef _CCH_RTL8723A_B_CUT
// _CCH_ECO_LPS_
        lc_sniff_window_rx_ind[lut_index - 1] = 0;
        lc_sniff_sup_count[lut_index - 1] = 0;
#endif
    }
#ifdef _ROM_CODE_PATCHED_
    /* Reserved com code patch by yilinli:
      * e.g. parameters update, extra LPS processing, tolerance recording, etc.
      */
    if (rcp_lc_start_sniff_end_func != NULL)
    {
        if ( rcp_lc_start_sniff_end_func((void*)(&ce_index), &ssr_flag ) )
        {
            return API_SUCCESS;
        }
    }
#endif

    return API_SUCCESS;
}


/**
 * Programs the baseband to exit from sniff mode. This will be called
 * when LMP level negotiation of exit-sniff omde is completed successfully.
 *
 * \param am_addr Logical transport address of the connection
 * \param piconet_id Physical piconet ID of the connection.
 *
 * \return API_SUCCESS on success.
 */
API_RESULT lc_exit_sniff_mode(UINT16 ce_index)
{
    UCHAR lut_index;
    LMP_CONNECTION_ENTITY *ce_ptr;

    UCHAR am_addr;
    UCHAR phy_piconet_id;


#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8723_RCP_
    if (rcp_lc_exit_sniff_mode != NULL)
    {
        if ( rcp_lc_exit_sniff_mode((void*)(&ce_index)) )
        {
            return API_SUCCESS;
        }
    }
#endif
#endif

    am_addr = lmp_connection_entity[ce_index].am_addr;
    phy_piconet_id = lmp_connection_entity[ce_index].phy_piconet_id;

    LC_EXIT_SM_MODE();

    lut_index = lc_get_lut_index_from_phy_piconet_id(am_addr, phy_piconet_id);

    ce_ptr = &lmp_connection_entity[ce_index];

    BB_modify_xtol_in_scatternet(0x00, ce_ptr->phy_piconet_id);


#ifdef _CCH_RTL8723A_B_CUT
// _CCH_ECO_LPS_
    if(g_efuse_lps_setting_4.lps_stop_afh_timer)
    {
        lmp_start_regular_sw_timers();
    }
#endif

#ifdef _CCH_SLOT_OFFSET_
    lmp_put_global_slot_offset(ce_index, 0);
#endif

    bb_kill_sniff_in_scatternet(am_addr, phy_piconet_id, FALSE);

    BB_start_tpoll(am_addr, ce_ptr->Tpoll, phy_piconet_id);

    if(ce_ptr->remote_dev_role == SLAVE)
    {
#ifndef _CCH_SLOT_OFFSET_
        lmp_put_dsniff(ce_ptr->sniff_slot_offset);
#endif
        lc_check_and_update_pkt_in_lut(LC_INVALID_PACKET_TYPE,
                                   LC_MASTER_DEFAULT_PACKET_TYPE,
                                   lut_ex_table[lut_index].lower_lut_address);
    }

    if(lmp_self_device_data.num_of_sniff_connections)
    {
        lmp_self_device_data.num_of_sniff_connections--;
    }

    RT_BT_LOG(GRAY, LC_1524, 0, 0);

    {
        OS_SIGNAL signal;

        signal.type = LMP_EXIT_SNIFF_MODE;

        OS_SEND_SIGNAL_TO_TASK(lmp_task_handle, signal);
    }

    return API_SUCCESS;
}
#endif /* COMPILE_SNIFF_MODE */

#ifdef COMPILE_PARK_MODE

/**
 * Programs the baseband to start beacon for park state. This will be called
 * when LMP level negotiation of exit-sniff omde is completed successfully.
 *
 * \param ce_index Index to lmp-connection-entity database for the connection
 *
 * \return API_SUCCESS on success.
 */
API_RESULT lc_start_beacon(UINT16 ce_index)
{
    UCHAR am_addr;
    LMP_CONNECTION_ENTITY *ce_ptr;
    UCHAR phy_piconet_id;

    ce_ptr = &lmp_connection_entity[ce_index];
    am_addr = ce_ptr->am_addr;

    LC_LOG_INFO(LOG_LEVEL_HIGH, STARTING_THE_BEACON,0,0);

    /*
       NOTE: Flush only lmp packets.
    */
    lc_clear_queues(ce_index, FALSE);

    phy_piconet_id = ce_ptr->phy_piconet_id;
    lc_kill_hardware_level_conn_in_scatternet(ce_index);

#ifdef PARK_ACTIVE_BIT_PATCH
    /* Clear the active bit in LUT:1-7 */
    {
        UINT16 addr, val;

        addr = reg_MASTER_UPPER_LUT[am_addr];
        val = BB_read_baseband_register(addr);
        val = val & (~ACTIVE_BIT);
        BB_write_baseband_register(addr, val);
    }
#endif

    /* Initialize seqn bit variables for BC lut*/
    lut_ex_table[BC_AM_ADDR].remote_seqn_bit = BB_REMOTE_INIT_SEQN_VAL;

    /* Program beacon to the Baseband */
    if(lc_beacon_running_flag != TRUE)
    {
        lc_program_beacon(ce_index);
        lc_beacon_running_flag = TRUE;
    }

    /* Re-start the SUP to. */
    if (ce_ptr->link_supervision_timeout != 0)
    {
        if(OS_START_TIMER(ce_ptr->supervision_timeout_handle,
              (UINT16)(SLOT_VAL_TO_TIMER_VAL(LMP_SUPERVISION_TIMER_RESOLUTION)))
                != BT_ERROR_OK)
        {
#ifdef ENABLE_LOGGER_LEVEL_2
            LC_LOG_INFO(LOG_LEVEL_LOW,UNABLE_TO_START_SUPERVISION_TIMEOUT_FOR_AM_ADDR,1,am_addr);
#endif
        }
    }

#ifdef COMPILE_AFH_HOP_KERNEL
    if(ce_ptr->afh_mode == AFH_ENABLE
#ifndef MASTER_AFH_PARK
            && ce_ptr->remote_dev_role == MASTER
#endif
      )
    {
        BB_write_afh_map(BC_AM_ADDR ,ce_ptr->phy_piconet_id, ce_ptr->afh_map);
    }
#endif /* COMPILE_AFH_HOP_KERNEL */

    return API_SUCCESS;
}

/**
 * Kills the beacon in the baseband. This function will be called when the
 * device needs to move to active mode from the parked state.
 *
 * \param am_addr The logical transport address of the connection.
 *
 * \return None.
 */
void lc_exit_beacon(UINT16 ce_index)
{
    DEF_CRITICAL_SECTION_STORAGE;

    UCHAR am_addr;
    UCHAR phy_piconet_id;
    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];
    am_addr = ce_ptr->am_addr;
    phy_piconet_id = ce_ptr->phy_piconet_id;

    MINT_OS_ENTER_CRITICAL();

    BB_write_baseband_register( CONNECTOR_REGISTER,
                            (UINT16) (am_addr << 5) | (phy_piconet_id << 11) );

    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_EXIT_BEACON);

    lc_beacon_running_flag = FALSE;

    MINT_OS_EXIT_CRITICAL();

    return;
}


/**
 * Initiates the process of moving from parked state to active mode.
 * In this state, the baseband will be ib the transition state between
 * parked state and active mode.
 *
 * \param ce_index Index to LMP connection entity.
 *
 * \return None.
 */
void lc_kill_beacon(UINT16 ce_index)
{
    UINT16 pic_addr = 0xff;
    BZ_REG_S_PICONET_INFO val;
    LMP_CONNECTION_ENTITY *ce_ptr = &lmp_connection_entity[ce_index];

    *(UINT16*)&val = 0;

    DEF_CRITICAL_SECTION_STORAGE;

    //RT_BT_LOG(GRAY, LC_1766, 0, 0);

    if (ce_ptr->phy_piconet_id <= SCA_PICONET_MAX)
    {
        /* As there will be only one connection in parked state,
        we can update xtol register for both piconets safely here. */
        BB_write_baseband_register(X_VALUE_FOR_TOLERANCE_REGISTER, 0x0000);
        BB_write_baseband_register(X_VALUE_FOR_TOLERANCE1_REGISTER, 0x0000);

        pic_addr = reg_PICONET_INFO[ce_ptr->phy_piconet_id];

        *(UINT16*)&val = BB_read_baseband_register(pic_addr);
        val.park_config = 0;

        MINT_OS_ENTER_CRITICAL();

        BB_write_baseband_register(pic_addr, *(UINT16*)&val);

        BB_write_baseband_register(CONNECTOR_REGISTER, 0);
        BB_write_baseband_register(INSTRUCTION_REGISTER, BB_KILL_BEACON);

        lc_beacon_running_flag = FALSE;

        MINT_OS_EXIT_CRITICAL();

        lc_check_and_enable_scans_in_scatternet();
    }

    RT_BT_LOG(GRAY, LC_1810, 2, pic_addr, *(UINT16*)&val);

    return;
}

/**
 * Moves the baseband from parked state to active mode. This function is
 * called when park mode exit procedure is completed.
 *
 * \param ce_index Index to lmp-connection-entity database for the connection
 *
 * \return None.
 */
void lc_handle_unpark(UINT16 ce_index)
{
    UCHAR lut_index;
    UCHAR phy_piconet_id;
    UCHAR am_addr;
    LMP_CONNECTION_ENTITY *ce_ptr;
    LUT_EXTENSION_TABLE *lut_ptr;

#ifdef ENABLE_LOGGER_LEVEL_2
    UINT16 read;
#endif

    DEF_CRITICAL_SECTION_STORAGE;

    LC_EXIT_SM_MODE();
    ce_ptr = &lmp_connection_entity[ce_index];

    /* Kill all scan before starting unpark procedure */
    lc_kill_scan_mode();

    am_addr = ce_ptr->am_addr;

    phy_piconet_id = ce_ptr->phy_piconet_id;

#ifdef COMPILE_NESTED_PAUSE_RESUME
    aclq_resume_am_addr(am_addr, phy_piconet_id, ACL_PAUSED_PARK);
#else /* COMPILE_NESTED_PAUSE_RESUME */
    aclq_resume_am_addr(am_addr, phy_piconet_id);
#endif /* COMPILE_NESTED_PAUSE_RESUME */

    if(ce_ptr->remote_dev_role == SLAVE)
    {
        lut_index = am_addr;
    }
    else
    {
        if(ce_ptr->phy_piconet_id <= SCA_PICONET_MAX)
        {
            lut_index = LC_SCA_SLAVE_1_LUT + ce_ptr->phy_piconet_id;
        }
        else
        {
            lut_index = LC_SCA_SLAVE_1_LUT;
            RT_BT_LOG(RED, PICONET_ID_CHECK_MSG, 1, ce_ptr->phy_piconet_id);
        }
    }

    lc_reset_lut_ex_table(lut_index);

    lut_ptr = &lut_ex_table[lut_index];
    lut_ptr->index_in_CE = ce_index;

    if (bz_auth_is_link_encrypted(ce_index))
    {
#ifdef ENABLE_LOGGER_LEVEL_2
        RT_BT_LOG(GRAY, LC_1904, 0, 0);
#endif
        bz_auth_enable_link_level_encryption(ce_index, TRUE);
    }

#ifdef ENABLE_LOGGER_LEVEL_2
    RT_BT_LOG(GRAY, LC_1910, 2, lut_index,ce_index);
#endif

    /* Make this Am Address Active */
#ifdef ENABLE_LOGGER_LEVEL_2
    RT_BT_LOG(GRAY, LC_1915, 1, am_addr);
#endif

    OR_val_with_bb_reg(lut_ptr->upper_lut_address, ACTIVE_BIT);

    MINT_OS_ENTER_CRITICAL();
    BB_write_baseband_register(CONNECTOR_REGISTER,
                        ((am_addr << 5) | (ce_ptr->phy_piconet_id << 11)));
    lc_init_seqn_scatternet(am_addr, lut_ptr, phy_piconet_id);
    MINT_OS_EXIT_CRITICAL();

    if(ce_ptr->remote_dev_role == SLAVE)
    {
        BB_write_baseband_register(lut_ptr->lower_lut_address,
                                   LC_MASTER_DEFAULT_PACKET_TYPE);

#ifdef ENABLE_LOGGER_LEVEL_2
        RT_BT_LOG(GRAY, LC_1936, 1, ce_ptr->am_addr);
#endif
    }
    else
    {
        /* Start sending default packet */
        BB_write_baseband_register(lut_ptr->lower_lut_address,
                                   LC_SLAVE_DEFAULT_PACKET_TYPE);

#ifdef COMPILE_AFH_HOP_KERNEL
        if(ce_ptr->afh_mode == AFH_ENABLE)
        {
            BB_write_afh_map(am_addr, ce_ptr->phy_piconet_id, ce_ptr->afh_map);
        }
        lmp_unpark_ce_index = ce_index;
#endif

        lc_exit_beacon(ce_index);

#ifdef ENABLE_LOGGER_LEVEL_2
        RT_BT_LOG(GRAY, LC_1965, 1, am_addr);
#endif
    }

    BB_start_tpoll((UCHAR)am_addr, ce_ptr->Tpoll, ce_ptr->phy_piconet_id);

#ifdef POWER_CONTROL
    ce_ptr->dec_pow_pdu_drop_flag = FALSE;
    ce_ptr->inc_pow_pdu_drop_flag = FALSE;
#endif /* POWER_CONTROL */

#ifdef ENABLE_LOGGER_LEVEL_2
    read = BB_read_baseband_register(lut_ptr->upper_lut_address);

    RT_BT_LOG(GRAY, LC_1987, 2, am_addr, read);

    RT_BT_LOG(GRAY, LC_1989, 1, lut_ptr->upper_lut_address);
#endif

    if(ce_ptr->ptt_status == LMP_PTT_ENABLED)
    {
        lc_enable_ptt_bit(ce_index);
    }
    return;
}


/**
 * Programs slave initiated unpark.
 *
 * \param ar_addr The access request address for which the unpark needs
 *                to be intiated.
 *
 * \return None.
 */
void lc_unpark_req(UINT16 ce_index)
{
    UCHAR ar_addr;
    UCHAR phy_piconet_id;
    DEF_CRITICAL_SECTION_STORAGE;

    LC_EXIT_SM_MODE();

    ar_addr = lmp_connection_entity[ce_index].ar_addr;
    phy_piconet_id = lmp_connection_entity[ce_index].phy_piconet_id;

    MINT_OS_ENTER_CRITICAL();
    BB_write_baseband_register(CONNECTOR_REGISTER,
                       (UINT16) ((BC_AM_ADDR << 5) | (phy_piconet_id << 11)));

    BB_write_baseband_register(ACCESS_REQUEST_ADDRESS_REGISTER, ar_addr);
    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_UNPARK_REQUEST);
    MINT_OS_EXIT_CRITICAL();

    RT_BT_LOG(GRAY, LC_2042, 1, ar_addr);

    return;
}
#endif /* COMPILE_PARK_MODE */

void lc_get_high_dpi_native_clock(UINT32 *bb_clock_slot,
                                    UINT32 *bb_clock_us)
{
    UINT16 temp1;
    UINT16 temp2;
    UINT16 temp3;
    UINT16 count_down;

    /* Initialize to native clock. */
    temp1 = NATIVE_CLOCK1_REGISTER;
    temp2 = NATIVE_CLOCK2_REGISTER;
    temp3 = NATIVE_CLK_COUNDOWN_US_REG;

    *bb_clock_slot = (BB_read_baseband_register(temp2) << 16) |
                    (BB_read_baseband_register(temp1));

    if (bb_clock_us != NULL)
    {
        count_down = BB_read_baseband_register(temp3) & 0x3FF;
        if (count_down > 624)
        {
            count_down -= 624;
        }
        *bb_clock_us = 624 - count_down;
    }
    return;
}

void lc_get_high_dpi_clock_in_scatternet(UINT32 *bb_clock_slot,
                                    UINT16 *bb_clock_us,
                                    UCHAR phy_piconet_id)
{
    UINT16 temp1;
    UINT16 temp2;
    UINT16 temp3;
    UINT16 count_down;
    UCHAR lut_index;

    lut_index = lc_get_lut_index_from_phy_piconet_id(
                    INVALID_AM_ADDR, phy_piconet_id);

    /* Initialize to native clock. */
    temp1 = NATIVE_CLOCK1_REGISTER;
    temp2 = NATIVE_CLOCK2_REGISTER;
    temp3 = NATIVE_CLK_COUNDOWN_US_REG;

    if(lut_index == LC_SCA_SLAVE_1_LUT)
    {
        /* Read piconet1 clock. */
        temp1 = PICONET_CLOCK1_REGISTER;
        temp2 = PICONET_CLOCK2_REGISTER;
        temp3 = PICONET1_CLK_COUNDOWN_US_REG;
    }
    else if(lut_index == LC_SCA_SLAVE_2_LUT)
    {
        /* Read piconet2 clock. */
        temp1 = PICONET2_CLOCK1_REGISTER;
        temp2 = PICONET2_CLOCK2_REGISTER;
        temp3 = PICONET2_CLK_COUNDOWN_US_REG;
    }
    else if (lut_index == LC_SCA_SLAVE_3_LUT)
    {
        /* Read piconet3 clock. */
        temp1 = PICONET3_CLOCK1_REGISTER;
        temp2 = PICONET3_CLOCK2_REGISTER;
        temp3 = PICONET3_CLK_COUNDOWN_US_REG;
    }
    else if (lut_index == LC_SCA_SLAVE_4_LUT)
    {
        /* Read piconet4 clock. */
        temp1 = PICONET4_CLOCK1_REGISTER;
        temp2 = PICONET4_CLOCK2_REGISTER;
        temp3 = PICONET4_CLK_COUNDOWN_US_REG;
    }

    *bb_clock_slot = (BB_read_baseband_register(temp2) << 16) |
                    (BB_read_baseband_register(temp1));

    if (bb_clock_us != NULL)
    {
        count_down = BB_read_baseband_register(temp3) & 0x3FF;
        if (count_down > 624)
        {
            count_down -= 624;
        }
        *bb_clock_us = 624 - count_down;
    }
    return;
}

/**
* Reads the baseband clock when the device is in scatternet.
*
* \param native_clock The variable that will contain the read xalue.
* \param piconet_id The physical piconet ID of the connection.
*
* \return None.
*/
void lc_get_clock_in_scatternet(UINT32 *native_clock,
                                UCHAR phy_piconet_id)
{
    lc_get_high_dpi_clock_in_scatternet(native_clock, NULL, phy_piconet_id);
}

#ifdef _CCH_RTL8723A_B_CUT
// _CCH_ECO_LPS_
#if 0
UINT32 lc_get_clock_by_pid(UCHAR phy_piconet_id)
{
    UINT32 clock;
    lc_get_high_dpi_clock_in_scatternet(&clock, NULL, phy_piconet_id);
    return clock;
}
#endif
#endif

/**
 * Clears the FIFO and the queues (pdu and l2cap)
 *
 * \param ce_index Index to lmp-connection-entity database for the connection
 * \param acl_flag ACL-Q is also cleared if TRUE.
 *
 * \return None.
 */
/* simplify the fw flow by austin for 0380 */
void lc_clear_queues(UINT16 ce_index, UCHAR acl_flag)
{
    LC_SCHEDULED_PKT *schd;
    LC_PICONET_SCHEDULER *piconet_schd;
    UCHAR lut_index;
    UCHAR am_addr;
    UCHAR phy_piconet_id;
    UCHAR flush_bb_fifo = FALSE;
    LMP_CONNECTION_ENTITY *ce_ptr;
    LC_SCHEDULED_PKT *schd1;
    UINT8 reload_fifo = FALSE;

    DEF_CRITICAL_SECTION_STORAGE;

#ifdef COMPILE_AFH_HOP_KERNEL
    lmp_delete_afh_timer(ce_index);
#endif

    ce_ptr = &lmp_connection_entity[ce_index];

    /* Find am_addr and physical piconet id of the connection */

    phy_piconet_id = ce_ptr->phy_piconet_id;

    am_addr = ce_ptr->am_addr;

    lut_index = lc_get_lut_index_from_phy_piconet_id(
                    ce_ptr->am_addr, phy_piconet_id);

    MINT_OS_ENTER_CRITICAL();
    /* Free all the data buffers and LMP packet buffers */
    lc_free_lmp_pdus(ce_index, TRUE);

    piconet_schd =  &lc_piconet_scheduler[phy_piconet_id];

    schd = &piconet_schd->lc_scheduled_pkt_info[piconet_schd->rptr];
    schd1 = &piconet_schd->lc_scheduled_pkt_info[!piconet_schd->rptr];

    if (piconet_schd->lc_allowed_pkt_cnt != LC_MAX_SCH_INFO)
    {
        UCHAR num_of_acl_conn;

        num_of_acl_conn = lmp_self_device_data.number_of_acl_conn;

        if ((ce_ptr->remote_dev_role == MASTER) || (num_of_acl_conn == 0) )
        {
            flush_bb_fifo = TRUE;
        }
        else
        {
            switch (piconet_schd->lc_allowed_pkt_cnt)
            {
                case 0:
                    if ((schd->selected_am_addr == am_addr) ||
                        (schd1->selected_am_addr == am_addr))
                    {
                        flush_bb_fifo = TRUE;

                        if ((schd->selected_am_addr != am_addr) ||
                            (schd1->selected_am_addr != am_addr))
                        {
                            reload_fifo = TRUE;
                        }
                    }
                    break;

                case 1:
                    if(schd->selected_am_addr == am_addr)
                    {
                        flush_bb_fifo = TRUE;
                    }
                    break;

                default:
                    break;
            }
        }

        if (flush_bb_fifo)
        {
            if (!reload_fifo)
            {
                schd->tx_status = LC_TX_IDLE;
                schd->txdesc_used_cnt = 0;
                schd->packet_ptr = NULL;
                schd1->tx_status = LC_TX_IDLE;
                schd1->txdesc_used_cnt = 0;
                schd1->packet_ptr = NULL;
                piconet_schd->rptr = 0;
                piconet_schd->wptr = 0;
                piconet_schd->lc_allowed_pkt_cnt = LC_MAX_SCH_INFO;
            }
            else
            {
                if (schd->selected_am_addr == am_addr)
                {
                    /* schedule 1 need be reloaded */
                    schd1->tx_status = LC_TX_READY;
                    piconet_schd->wptr = piconet_schd->rptr;
                    piconet_schd->rptr++;

                    schd->tx_status = LC_TX_IDLE;
                    schd->txdesc_used_cnt = 0;
                    schd->packet_ptr = NULL;
                }
                else
                {
                    /* schedule 0 need be reloaded */
                    schd->tx_status = LC_TX_READY;
                    piconet_schd->wptr = !piconet_schd->rptr;

                    schd1->tx_status = LC_TX_IDLE;
                    schd1->txdesc_used_cnt = 0;
                    schd1->packet_ptr = NULL;
                }
                piconet_schd->lc_allowed_pkt_cnt = 1;
            }
        } /* end of if(flush_bb_fifo) */
    } /* end of if (piconet_schd->lc_allowed_pkt_cnt != LC_MAX_SCH_INFO) */

    if (acl_flag == TRUE)
    {
        aclq_clear_all_pkts_am_addr(am_addr, FALSE, ce_ptr->phy_piconet_id);
    }

    MINT_OS_EXIT_CRITICAL();
    RT_BT_LOG(GRAY, LC_2373, 1, am_addr);
}

/**
 * Handles scan mode written by the host through hci_write_scan_mode command.
 *
 * \param None.
 *
 * \return API_SUCCESS on successful programming.
 */
API_RESULT lc_handle_scan_mode_command(void)
{
    OS_SIGNAL sig_send;

    sig_send.type = LC_HANDLE_START_SCAN_MODE_SIGNAL;
    OS_SEND_SIGNAL_TO_TASK(lc_tx_task_handle, sig_send);

    return API_SUCCESS;
}

#ifdef COMPILE_CQDDR
/**
 * Returns the intersection of host given packet types and remote device
 * preferred pkt types.
 *
 * \param pref_pkt_type Remote device preferred pkt types
 * \param host_pkt_type Host given packet types
 *
 * \return None.
 */
UINT16 lc_get_cqddr_pkt_type(UINT16 pref_pkt_type,
        UINT16 host_pkt_type)
{
    UINT16 edr_pkt_type;
    UINT16 br_pkt_type;

    edr_pkt_type = (pref_pkt_type | host_pkt_type) & ALL_EDR_PACKETS;
    br_pkt_type = (pref_pkt_type & host_pkt_type) & ALL_BR_PACKETS;

    return (edr_pkt_type | br_pkt_type);
}
#endif /* COMPILE_CQDDR */

#ifdef COMPILE_SNIFF_MODE
/**
 * Calculates the least sniff interval of all the connections.
 *
 * \param None.
 * \return sniff_interval The least value of sniff interval for all the
 *        connections. If there are no connections in sniff mode, then 0
 *        will be returned.
 *
 */
#ifdef _DAPE_SNIFF_PKT_TEST
UINT32 lc_get_least_sniff_interval(UINT16 *sniff_ce_index, UINT16 *sniff_attempt)
#else
UINT32 lc_get_least_sniff_interval(void)
#endif
{
    UINT8 i;
    LMP_CONNECTION_ENTITY *ce_ptr;
    UINT16 least_interval = 0xFFFF;

#ifdef _ROM_CODE_PATCHED_
    UINT16 rcp_sniff_ce_index = 0xFF;
    UINT16 rcp_sniff_attempt = 0;
    if (rcp_lc_get_least_sniff_interval != NULL)
    {
        rcp_lc_get_least_sniff_interval((void*)(&rcp_sniff_ce_index), (&rcp_sniff_attempt));
#ifdef _DAPE_SNIFF_PKT_TEST
        *sniff_ce_index = rcp_sniff_ce_index;
        *sniff_attempt = rcp_sniff_attempt;
#endif
        if(rcp_sniff_ce_index != 0xFF)
        {
            return lmp_connection_entity[rcp_sniff_ce_index].sniff_interval;
        }
        else
        {
            /*no connection is in sniff mode*/
            return 0;
        }
    }
#endif

#ifdef _DAPE_SNIFF_PKT_TEST
    *sniff_ce_index = 0xff;
    *sniff_attempt = 0;
#endif

    for(i = 0; i < LMP_MAX_CE_DATABASE_ENTRIES; i++)
    {
        ce_ptr = &lmp_connection_entity[i];
#ifdef BZ_2_1_2_HID_FTP
        if(ce_ptr->entity_status == ASSIGNED)
        {
            if(ce_ptr->in_sniff_mode == TRUE)
            {
                if(ce_ptr->sniff_interval < least_interval)
                {
                    least_interval = ce_ptr->sniff_interval;
#ifdef _DAPE_SNIFF_PKT_TEST
                    *sniff_ce_index = i;
                    *sniff_attempt = ce_ptr->sniff_attempt;
#endif
                }
            }
            else
            {
                if( (ce_ptr->temp_sniff_interval != 0xffff) &&
                        (ce_ptr->temp_sniff_interval < least_interval) )
                {
                    least_interval = ce_ptr->sniff_interval;
                }
            }
        }
#else
        if( (ce_ptr->entity_status == ASSIGNED) &&
                (ce_ptr->in_sniff_mode == TRUE) )
        {
            if(ce_ptr->sniff_interval < least_interval)
            {
                least_interval = ce_ptr->sniff_interval;
            }
        }
#endif
    }

    if(least_interval == 0xFFFF)
    {
        /*no connection is in sniff mode*/
        return 0;
    }
    else
    {
        /*return the minimum least_interval*/
        return least_interval;
    }
}
#endif

/**
 * Updates the packet types according the PKT_TPYE_INFO parameter.
 *
 * \param ce_index Index to lmp-connection-entity database for the connection
 * \param pkt_type_ptr The data structure that contains the packet types
                       to be updated.
 *
 * \return None.
 */
void lc_update_pkts_allowed(UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY *ce_ptr;
    PKT_ALLOWED *pkts_allowed_ptr;
    UINT16 lcl_status;
    UINT16 *lcl_pkt_type;
#ifdef COMPILE_CQDDR
    UINT16 pref_pkt_type;
#endif

    UCHAR local_feature;

    local_feature = lmp_feature_data.feat_page0[0];

    ce_ptr = &lmp_connection_entity[ce_index];
    pkts_allowed_ptr = &(ce_ptr->pkts_allowed);

    lcl_pkt_type = & (ce_ptr->connection_type.packet_type);

#ifdef COMPILE_CQDDR
    if (!IS_USE_FOR_MUTE)
    {
        pref_pkt_type = lc_get_cqddr_pkt_type(ce_ptr->preferred_cqddr_pkt_type,
                                              ce_ptr->connection_type.packet_type);

        /* Use remote device preferred packets if possible */
        if (pref_pkt_type != ALL_EDR_PACKETS)
        {
            lcl_pkt_type = &pref_pkt_type;
        }
    }
#endif /* COMPILE_CQDDR */

    lcl_status = pkts_allowed_ptr->status;

    /* Step1. Disable all packets. */
    lcl_status &= ~( (1 << LC_DV_INDEX) | (1 << LC_DH1_INDEX) |
                     (1 << LC_DM3_INDEX) | (1 << LC_DH3_INDEX) |
                     (1 << LC_DM5_INDEX) | (1 << LC_DH5_INDEX) );

    /* Disable EDR packets. */
    lcl_status &= ( ~( (1 << LC_2_DH1_INDEX) | (1 << LC_3_DH1_INDEX) |
                       (1 << LC_2_DH3_INDEX) | (1 << LC_3_DH3_INDEX) |
                       (1 << LC_2_DH5_INDEX) | (1 << LC_3_DH5_INDEX) ) );

    /* Step2. Enable, based on host given packets. */
    if (*lcl_pkt_type & DH1)
    {
        /* DH1 is enabled. */
        lcl_status |= (1 << LC_DH1_INDEX);
    }

    if (! (*lcl_pkt_type & EDR_2DH1))
    {
        /* 2-DH1 is enabled. */
        lcl_status |= (1 << LC_2_DH1_INDEX);
    }

    if (! (*lcl_pkt_type & EDR_3DH1))
    {
        /* 3-DH1 is enabled. */
        lcl_status |= (1 << LC_3_DH1_INDEX);
    }

#ifndef COMPILE_SINGLE_SLOT_PACKETS_ONLY
    if(local_feature & LMP_THREE_SLOT_PACKET_FEATURE) // basic rate 3 slot
    {
        if (*lcl_pkt_type & DM3)
        {
            /* DM3 is enabled. */
            lcl_status |= (1 << LC_DM3_INDEX);
        }

        if (*lcl_pkt_type & DH3)
        {
            /* DH3 is enabled. */
            lcl_status |= (1 << LC_DH3_INDEX);
        }
    }

    if(local_feature & LMP_FIVE_SLOT_PACKET_FEATURE) // basic rate 5 slot
    {
        if (*lcl_pkt_type & DM5)
        {
            /* DM5 is enabled. */
            lcl_status |= (1 << LC_DM5_INDEX);
        }

        if (*lcl_pkt_type & DH5)
        {
            /* DH5 is enabled. */
            lcl_status |= (1 << LC_DH5_INDEX);
        }
    }

    local_feature = lmp_feature_data.feat_page0[4];

    if (local_feature & LMP_EDR_3_SLOT_PACKET_FEATURE)
    {
        if (! (*lcl_pkt_type & EDR_2DH3))
        {
            /* 2-DH3 is enabled. */
            lcl_status |= (1 << LC_2_DH3_INDEX);
        }

        if (! (*lcl_pkt_type & EDR_3DH3))
        {
            /* 3-DH3 is enabled. */
            lcl_status |= (1 << LC_3_DH3_INDEX);
        }
    }

    local_feature = lmp_feature_data.feat_page0[5];

    if (local_feature & LMP_EDR_5_SLOT_PACKET_FEATURE)
    {
        if (! (*lcl_pkt_type & EDR_2DH5))
        {
            /* 2-DH5 is enabled. */
            lcl_status |= (1 << LC_2_DH5_INDEX);
        }
        if (! (*lcl_pkt_type & EDR_3DH5))
        {
            /* 3-DH5 is enabled. */
            lcl_status |= (1 << LC_3_DH5_INDEX);
        }
    }

    if (ce_ptr->last_accepted_max_slot != LMP_LAST_ACCEPTED_MAX_SLOT_INVALID)
    {
        /* Step3. Enable, based on last-max-slot-recd */
        if (ce_ptr->last_accepted_max_slot == LMP_LAST_ACCEPTED_MAX_SLOT_5)
        {
            /* Do nothing. All packets are allowed. */
        }
        else if (ce_ptr->last_accepted_max_slot == LMP_LAST_ACCEPTED_MAX_SLOT_3)
        {
            /* Disable 5 slot packets. */
            lcl_status &= ~( (1 << LC_DM5_INDEX) |
                             (1 << LC_DH5_INDEX) |
                             (1 << LC_2_DH5_INDEX) |
                             (1 << LC_3_DH5_INDEX) );
        }
        else
        {
            /* Disable 3 and 5 slot packets. */
            lcl_status &= ~( (1 << LC_DM3_INDEX) |
                             (1 << LC_DH3_INDEX) |
                             (1 << LC_DM5_INDEX) |
                             (1 << LC_DH5_INDEX) |
                             (1 << LC_2_DH3_INDEX) |
                             (1 << LC_3_DH3_INDEX) |
                             (1 << LC_2_DH5_INDEX) |
                             (1 << LC_3_DH5_INDEX) );
        }
    }

    if (ce_ptr->last_recd_max_slot != LMP_MAX_SLOT_INVALID)
    {

        /* Step4. Enable, based on last max-slot-req-sent */
        if(ce_ptr->last_recd_max_slot == LMP_MAX_SLOT_5)
        {
            /* Do nothing. All packets are allowed. */
        }
        else if(ce_ptr->last_recd_max_slot == LMP_MAX_SLOT_3)
        {
            /* Disable 5 slot packets. */
            lcl_status &= ~( (1 << LC_DM5_INDEX) |
                             (1 << LC_DH5_INDEX) |
                             (1 << LC_2_DH5_INDEX) |
                             (1 << LC_3_DH5_INDEX) );
        }
        else
        {
            /* Disable 3 and 5 slot packets. */
            lcl_status &= ~( (1 << LC_DM3_INDEX) |
                             (1 << LC_DH3_INDEX) |
                             (1 << LC_DM5_INDEX) |
                             (1 << LC_DH5_INDEX) |
                             (1 << LC_2_DH3_INDEX) |
                             (1 << LC_3_DH3_INDEX) |
                             (1 << LC_2_DH5_INDEX) |
                             (1 << LC_3_DH5_INDEX) );
        }

    }

#ifdef COMPILE_SNIFF_MODE
    /* Step5. Sniff mode check. */
    if (ce_ptr->in_sniff_mode == TRUE)
    {
        /* Disable 3 and 5 slot packets. */
        lcl_status &= ~( (1 << LC_DM3_INDEX) |
                         (1 << LC_DH3_INDEX) |
                         (1 << LC_DM5_INDEX) |
                         (1 << LC_DH5_INDEX) |
                         (1 << LC_2_DH3_INDEX) |
                         (1 << LC_3_DH3_INDEX) |
                         (1 << LC_2_DH5_INDEX) |
                         (1 << LC_3_DH5_INDEX) );
    }
#endif /* COMPILE_SNIFF_MODE */

#endif /* COMPILE_SINGLE_SLOT_PACKETS_ONLY */

#ifdef ENABLE_SCO
    /* Step6. Synch connections. */
    /* If HV2 or HV3 connections are present, enable only single slot packets. */
    if (((lmp_self_device_data.total_no_of_sco_conn != 0) &&
            (lmp_self_device_data.sco_pkt_type != HV1))
            || (lmp_self_device_data.adding_new_sco_conn != 0)
#ifdef COMPILE_ESCO
            || (lmp_self_device_data.number_of_esco_connections != 0)
#ifdef _DAPE_TEST_SEND_MAX_SLOT_BEFORE_ESCO_CREATED
            || (lmp_self_device_data.adding_new_esco_conn != 0)
#endif
#endif
       )
    {
        if (!lc_check_if_device_is_in_scatternet() &&
                (lmp_self_device_data.sco_pkt_type == HV3))
        {
            /* Disable 5 slot packets. */
            lcl_status &= ~( (1 << LC_DM5_INDEX) |
                             (1 << LC_DH5_INDEX) |
                             (1 << LC_2_DH5_INDEX) |
                             (1 << LC_3_DH5_INDEX) );
        }
        else
        {
            /* Disable 3 and 5 slot packets. */
            lcl_status &= ~( (1 << LC_DM3_INDEX) |
                             (1 << LC_DH3_INDEX) |
                             (1 << LC_DM5_INDEX) |
                             (1 << LC_DH5_INDEX) |
                             (1 << LC_2_DH3_INDEX) |
                             (1 << LC_3_DH3_INDEX) |
                             (1 << LC_2_DH5_INDEX) |
                             (1 << LC_3_DH5_INDEX) );
        }

        if(lc_full_bandwidth_flag == TRUE)
        {
            /* Allow only DM1 packets */
            lcl_status &= ~( (1 << LC_DH1_INDEX) |
                             (1 << LC_2_DH1_INDEX) |
                             (1 << LC_3_DH1_INDEX) );
        }
    }

    /* Step7. For enabling DV. */
    if ( (lmp_self_device_data.total_no_of_sco_conn != 0) &&
            (lmp_self_device_data.sco_pkt_type == HV1) )
    {
        /* Disable all packet types except DV. */
        lcl_status &= ~( (1 << LC_DM1_INDEX) |
                         (1 << LC_DH1_INDEX) |
                         (1 << LC_DM3_INDEX) |
                         (1 << LC_DH3_INDEX) |
                         (1 << LC_DM5_INDEX) |
                         (1 << LC_DH5_INDEX) |
                         (1 << LC_2_DH1_INDEX) |
                         (1 << LC_3_DH1_INDEX) |
                         (1 << LC_2_DH3_INDEX) |
                         (1 << LC_3_DH3_INDEX) |
                         (1 << LC_2_DH5_INDEX) |
                         (1 << LC_3_DH5_INDEX) );

        /* Enable DV packet. */
        lcl_status |= (1 << LC_DV_INDEX) ;
    }
#endif /* ENABLE_SCO */

#ifdef COMPILE_SNIFF_MODE
    /* Step8. If there is any connection in sniff mode, with very small
     * interval, then only single/three slot packets will be allowed on
     * all the connections. */
    {
        UINT32 sniff_int;

#ifdef _DAPE_SNIFF_PKT_TEST
        UINT16 sniff_ce_index;
        UINT16 sniff_attempt;
        UCHAR sniff_piconet;
        UCHAR current_piconet;
        LMP_CONNECTION_ENTITY *sniff_ce_ptr;
        //UINT16 sniff_slot;

        sniff_int = lc_get_least_sniff_interval(&sniff_ce_index, &sniff_attempt);
        if ((sniff_int!=0) && (sniff_attempt ==0))
        {
            sniff_attempt = 1;
        }
#ifdef _DAPE_TEST_FIX_SNIFF_PKT_TEST_ACCESS_ERR
        if (sniff_ce_index == INVALID_CE_INDEX)
        {
            //RT_BT_LOG(RED, DAPE_TEST_LOG293, 1,sniff_int);
            sniff_ce_index = ce_index;
        }
#endif
        sniff_ce_ptr = &lmp_connection_entity[sniff_ce_index];
        sniff_piconet = sniff_ce_ptr->phy_piconet_id;
        current_piconet = ce_ptr->phy_piconet_id;

#else
        sniff_int = lc_get_least_sniff_interval();
#endif
#ifdef _DAPE_SNIFF_PKT_TEST
        if((sniff_int != 0) &&
           (sniff_int < LC_MIN_SNIFF_INTERVAL_ALLOW_5_SLOT_PKT) &&
           (current_piconet != sniff_piconet))
        {
            /* Disable 5 slot packets. */
            lcl_status &= ~( (1 << LC_DM5_INDEX) |
                             (1 << LC_DH5_INDEX) |
                             (1 << LC_2_DH5_INDEX) |
                             (1 << LC_3_DH5_INDEX) );

            if(sniff_int < LC_MIN_SNIFF_INTERVAL_ALLOW_3_SLOT_PKT)
            {
                /* Disable 3 slot packets. */
                lcl_status &= ~( (1 << LC_DM3_INDEX) |
                                 (1 << LC_DH3_INDEX) |
                                 (1 << LC_2_DH3_INDEX) |
                                 (1 << LC_3_DH3_INDEX) );
            }
        }
        else
        {
            if((sniff_int != 0) && (current_piconet == sniff_piconet)
		  	&& (sniff_int < (5*2 + 4 + sniff_attempt*2)))
            {
                /* Disable 5 slot packets. */
                lcl_status &= ~( (1 << LC_DM5_INDEX) |
                               (1 << LC_DH5_INDEX) |
                               (1 << LC_2_DH5_INDEX) |
                               (1 << LC_3_DH5_INDEX) );

                if(sniff_int < (3*2 + 2 + sniff_attempt*2))
                {
                    /* Disable 3 slot packets. */
                    lcl_status &= ~( (1 << LC_DM3_INDEX) |
                                   (1 << LC_DH3_INDEX) |
                                   (1 << LC_2_DH3_INDEX) |
                                   (1 << LC_3_DH3_INDEX) );
                }
            }
        }
#else
        if((sniff_int != 0) &&
           (sniff_int < LC_MIN_SNIFF_INTERVAL_ALLOW_5_SLOT_PKT))
        {
            /* Disable 5 slot packets. */
            lcl_status &= ~( (1 << LC_DM5_INDEX) |
                             (1 << LC_DH5_INDEX) |
                             (1 << LC_2_DH5_INDEX) |
                             (1 << LC_3_DH5_INDEX) );

            if(sniff_int < LC_MIN_SNIFF_INTERVAL_ALLOW_3_SLOT_PKT)
            {
                /* Disable 3 slot packets. */
                lcl_status &= ~( (1 << LC_DM3_INDEX) |
                                 (1 << LC_DH3_INDEX) |
                                 (1 << LC_2_DH3_INDEX) |
                                 (1 << LC_3_DH3_INDEX) );
            }
        }
#endif
    }
#endif
#ifdef _DAPE_TEST_DISABLE_DM1_FOR_CCPT_BY_VENDOR_CMD
    if (!g_disable_dm1)
    {
        /* Step9. Defensive move, enable DM1. */
        lcl_status |= (1 << LC_DM1_INDEX) ;
    }
#endif
    pkts_allowed_ptr->status = lcl_status;

    BB_update_sniff_max_slot();

#ifdef _ROM_CODE_PATCHED_
    if (rcp_lc_update_pkts_allowed != NULL)
    {
        rcp_lc_update_pkts_allowed((void*)(&ce_index));
    }
#endif

    return;
}

/**
 * Resets the particular lut-ex-table entry.
 *
 * \param am_addr The Logical transport address of the connection.
 * \param ce_index Index to lmp-connection-entity database for the connection
 * \param piconet_id phy_piconet_id of the connection
 *
 * \return None.
 */
INLINE void lc_update_ce_index_to_lut_extn_tbl(UINT16 ce_index, UCHAR lut_index)
{
    lc_reset_lut_ex_table(lut_index);

    lut_ex_table[lut_index].index_in_CE = ce_index;

    return;
}

/**
 * Programs the calculated tolerance value to the baseband, for a
 * particular connection.
 *
 * \param inactive_slots The number of slots for which the connection
 *                       will be in low power mode.
 * \param ce_index Index to lmp-connection-entity database for the connection
 *
 * \return API_SUCCESS on success.
 */
UCHAR lc_program_tolerance(UINT16 inactive_slots, UINT16 ce_index)
{
    UINT16 tolerance;

    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    if(ce_ptr->remote_dev_role == SLAVE)
    {
        /* Need not program xtol as master. */
        return 0;
    }

    tolerance = (UINT16) lc_calculate_tolerance(inactive_slots, ce_index);

    BB_modify_xtol_in_scatternet(tolerance, ce_ptr->phy_piconet_id);

/*
#ifdef _YL_LPS
    ce_ptr->sniff_xtol = tolerance;
#endif
*/

#ifdef _YL_LPS_LOG_
    LPS_DBG_LOG(GRAY, LC_2934, 1, tolerance);
#else
#ifdef SCNET_DEBUG
    RT_BT_LOG(GRAY, LC_2934, 1, tolerance);
#endif
#endif

    return (UCHAR)tolerance;
}

#ifdef COMPILE_DYNAMIC_POLLING
/**
* Disable dynamic polling for self device.
*
* \param Nothing
*
* \return Nothing
*/
void lc_disable_dynamic_polling(void)
{
    UCHAR count;
    UINT16 ce_index;
    LMP_CONNECTION_ENTITY* ce_ptr;
    DEF_CRITICAL_SECTION_STORAGE;

    if (lmp_self_device_data.enable_dynamic_polling == FALSE)
    {
        /* Dynamic polling already disabled */
        return;
    }

    lmp_self_device_data.enable_dynamic_polling = FALSE;

    /* Restart default tpoll on all connections in which Tpoll is running. */
    for(count = 0; count< MAX_NO_OF_LUT_INDICES; count++)
    {
        ce_index = lut_ex_table[count].index_in_CE;
        if(ce_index != INVALID_CE_INDEX)
        {
            ce_ptr = &lmp_connection_entity[ce_index];
            MINT_OS_ENTER_CRITICAL();
            if(lc_is_tpoll_started[count] == TRUE)
            {
                BB_stop_tpoll(ce_ptr->am_addr, ce_ptr->phy_piconet_id);
                BB_start_tpoll(ce_ptr->am_addr, ce_ptr->Tpoll
                               ,ce_ptr->phy_piconet_id);
#ifdef _DAPE_TEST_SHORTEN_TPOLL_WHILE_PAGE_INQ
                if (lmp_self_device_data.lc_cur_dev_state != LC_CUR_STATE_IDLE)
                {
                    BZ_REG_S_PRI_CTRL pri_ctrl;
                    BZ_REG_S_PRI_CTRL2 pri_ctrl2;

                    *(UINT16*)&pri_ctrl = BB_read_baseband_register(SCA_PRIORITY_REGISTER);
                    *(UINT16*)&pri_ctrl2 = BB_read_baseband_register(SCA_PRIORITY_REGISTER3);
                    if ((pri_ctrl.page_high_than_acl_def == TRUE)||
                    (pri_ctrl2.inq_high_than_acl_def == TRUE))
                    {
                        BB_start_tpoll(ce_ptr->am_addr, (ce_ptr->Tpoll)>>1
                                       ,ce_ptr->phy_piconet_id);
                        //RT_BT_LOG(RED, DAPE_TEST_LOG293, 1,(ce_ptr->Tpoll)>>1);
                    }
                }
#endif
            }
            MINT_OS_EXIT_CRITICAL();
        }
    }

    //RT_BT_LOG(GRAY, LC_2999, 0, 0);
}

/**
* Update dynamic polling parameters for self device.
*
* \param Nothing
*
* \return Nothing
*/
void lc_update_dynamic_polling(void)
{
    if ((lmp_self_device_data.number_of_hlc > 1) ||
        (lmp_self_device_data.lc_cur_dev_state != LC_CUR_STATE_IDLE))
    {
        lc_disable_dynamic_polling();
        return;
    }

    lmp_self_device_data.enable_dynamic_polling = TRUE;

    //RT_BT_LOG(GRAY, LC_3023, 0, 0);

    switch(lmp_self_device_data.number_of_hlc)
    {
        case 0: /* Fall through. */
        case 1:
            lmp_self_device_data.tpoll_inactive_transition = 8;
            lmp_self_device_data.tpoll_step = 4;
            lmp_self_device_data.min_tpoll = 4;
            break;
        default:
            BZ_ASSERT(0,"Currently we disable dynamic polling for multiple connections.");
            /* Defensive - disable dynamic polling */
            lc_disable_dynamic_polling();
            return;

    }
}


/**
* Programs the dynamic tolerance value to the baseband, based on the
* activity on a particular connection.
*
* \param ce_index Index to lmp-connection-entity database for the connection
*
* \return Nothing
*/
void lc_program_dynamic_tpoll(UINT16 ce_index)
{
    UCHAR lut_index;
    UCHAR tpoll_inactive_transition =
        lmp_self_device_data.tpoll_inactive_transition;
    UCHAR tpoll_step = lmp_self_device_data.tpoll_step;
    UCHAR min_tpoll = lmp_self_device_data.min_tpoll;
    LMP_CONNECTION_ENTITY* ce_ptr;
    DEF_CRITICAL_SECTION_STORAGE;

    ce_ptr = &lmp_connection_entity[ce_index];

    if((ce_ptr == NULL) ||
            (lmp_self_device_data.enable_dynamic_polling == FALSE))
    {
        return;
    }

    if(ce_ptr->no_inactive_tpolls == 0)
    {
        if(ce_ptr->current_tpoll == min_tpoll)
        {
            return;
        }
        /* Program min_tpoll value */
        ce_ptr->current_tpoll = min_tpoll;
    }
    else if(ce_ptr->no_inactive_tpolls > tpoll_inactive_transition)
    {
        /* Increase Tpoll value. */
        if(ce_ptr->current_tpoll < ce_ptr->Tpoll)
        {
            ce_ptr->current_tpoll += tpoll_step;

            if(ce_ptr->Tpoll >= 12)
            {
                /* Maximum value of Tpoll can be the qos Tpoll val */
                if(ce_ptr->current_tpoll > (ce_ptr->Tpoll - 8))
                {
                    ce_ptr->current_tpoll = ce_ptr->Tpoll - 8;
                }
            }
            else
            {
                /* Maximum value of Tpoll can be the qos Tpoll val */
                if(ce_ptr->current_tpoll > ce_ptr->Tpoll)
                {
                    ce_ptr->current_tpoll = ce_ptr->Tpoll;
                }
            }
            /* Reset to non-zero, so as to not start next tpoll from min. */
            ce_ptr->no_inactive_tpolls = 1;
        }
        else
        {
            return;
        }
    }
    else
    {
        return;
    }

    lut_index = lc_get_lut_index_from_ce_index(ce_index);

    MINT_OS_ENTER_CRITICAL();
    if (lc_is_tpoll_started[lut_index] == TRUE)
    {
        /* Program Dynamic Tpoll value */
        BB_stop_tpoll(ce_ptr->am_addr, ce_ptr->phy_piconet_id);
        BB_start_tpoll(ce_ptr->am_addr, ce_ptr->current_tpoll,
                       ce_ptr->phy_piconet_id);
    }
    MINT_OS_EXIT_CRITICAL();

}
#endif

/**
 * Calculates the tolerance value to the baseband, for a
 * particular connection.
 *
 * \param inactive_clots The number of slots for which the connection
 *                       will be in low power mode.
 * \param ce_index Index to lmp-connection-entity database for the connection
 *
 * \return API_SUCCESS on success.
 */
 #ifdef LPS_NEW_CAL_XTOL
 // TODO: (yl) the following is the RTL8723A B-CUT Patch Code for reference, to be improved
UCHAR lc_calculate_tolerance(UINT16 inactive_slots, UINT16 ce_index)
{
    UINT32 low_power_time;
    UINT16 tolerance = 0;
    UCHAR max_remote_drift = 250;

    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    if(ce_ptr->remote_dev_role != MASTER)
    {
        /* Don't program if we are Master */
        return 0;
    }

#ifdef _ROM_CODE_PATCHED_
    /* Reserved com code patch by yilinli:
      * e.g.
      *    1. totally replace the lc_calculate_tolerance() function
      */
    if (rcp_lc_calculate_tolerance_func != NULL)
    {
        if (rcp_lc_calculate_tolerance_func((void*)(&ce_index), &inactive_slots, &tolerance))
        {
            return tolerance;
        }
    }
#endif

    if (ce_ptr->remote_max_drift != 0xff)
    {
        max_remote_drift = ce_ptr->remote_max_drift;
    }

#define XTOL_MAX_SLOT 62
//    if (g_efuse_lps_setting_1.new_tol_cal)
    {
        low_power_time = (inactive_slots*(sleep_mode_param.lps_drift_ppm_for_xtol + max_remote_drift)); /* 1e6*625/2^20 = 596.05 */
//        RT_BT_LOG(GREEN, DMA_UART_002_DBG_DEC, 1, low_power_time);
        {


#ifdef _LPS_MODIFY_XTOL
            if( inactive_slots > ce_ptr->sniff_interval)
            {
                low_power_time += ((inactive_slots - ce_ptr->sniff_interval)*750); // up to -1000 ppm
            }
#endif

            if(low_power_time == 0)
            {
                tolerance = 0;
            }
            else
            {
                /* 625/1024/1024 = 0.0006256
                 * drift(us) = drift_ppm*(inactive_slots*625)/10^6 */
                if (low_power_time >= BIT26)
                {
//                    low_power_time = (low_power_time>>20)*656;
//                    tolerance = (low_power_time+(DSM_DRIFT>>g_efuse_lps_setting_1.dsm_drift_scaling)); /* ~ ceil(x) */
                    low_power_time = (low_power_time>>20)*656;
                }
                else if (low_power_time >= BIT20)
                {
//                    low_power_time = ((low_power_time>>14)*656)>>6;
//                    tolerance = (low_power_time+(DSM_DRIFT>>g_efuse_lps_setting_1.dsm_drift_scaling)); /* ~ ceil(x) */
                    low_power_time = ((low_power_time>>14)*656)>>6;
                }
                else
                {
//                    low_power_time =  low_power_time*656;
//                    tolerance = ((low_power_time>>20)+(DSM_DRIFT>>g_efuse_lps_setting_1.dsm_drift_scaling)); /* ~ ceil(x) */
                    low_power_time =  (low_power_time*656)>>20;
                }

                /* If we are slave , then open the receive window , to
                 * accomodate the drift.
                 * A constant k is added to power power time to accomodate for
                 * switching between low power clock and sysclk in DSM mode.
                 *
                 * tolerance(half-slots) = ceil( (drift(us) + DSM_DRIFT) / 312us )
                 */
                tolerance = (low_power_time + (DSM_DRIFT>>g_efuse_lps_setting_1.dsm_drift_scaling) ) / 312 + 1;
//                RT_BT_LOG(RED, DMA_UART_002_DBG_DEC, 1, tolerance);

#ifdef _8821A_BTON_DESIGN_ /* for RTL8821A and later */ // TODO: (yl) to be confirm with DD for the resolution of xtol
                UINT16 xtol_resol;
                if ( ( (ce_ptr->in_sniff_mode == TRUE) || g_efuse_lps_setting_2.sniff_xtol_scaling_for_park_hold) && (tolerance != 0) )
                {
                    xtol_resol = 4 >> (g_efuse_lps_setting_2.sniff_xtol_scaling);
                }
                else
                {   /* PARK/HOLD mode should be modified for finer xtol resolution */
                    xtol_resol = 4;
                }

                UINT16 tolerance_frac = tolerance % xtol_resol;
                if (tolerance_frac)
                {
                    tolerance = (tolerance - tolerance_frac) + xtol_resol; /* unconditionaly carry-in form 2 LSBs */
                }

#else  /* for RTL8723A */
                if (tolerance & 0x0003)
                {
                    tolerance = (tolerance & 0xfffc)+4; /* unconditionaly carry-in form 2 LSBs */
                }
#endif
                tolerance = MIN( tolerance, XTOL_MAX_SLOT*2);
            }
        }
        return (UCHAR) tolerance;
    }
}
#else
UCHAR lc_calculate_tolerance(UINT16 inactive_slots, UINT16 ce_index)
{
    UINT32 low_power_time,drift;
    UINT16 tolerance = 0;
    UCHAR max_remote_drift = 250;
    UCHAR total_drift;
    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    if(ce_ptr->remote_dev_role != MASTER)
    {
        /* Don't program if we are Master */
        return 0;
    }

#ifdef _ROM_CODE_PATCHED_
    /* Reserved com code patch by yilinli:
      * e.g.
      *    1. totally replace the lc_calculate_tolerance() function
      */
    if (rcp_lc_calculate_tolerance_func != NULL)
    {
        return rcp_lc_calculate_tolerance_func((void*)(&ce_index), &inactive_slots );
    }
#endif

    if (ce_ptr->remote_max_drift != 0xff)
    {
        max_remote_drift = ce_ptr->remote_max_drift;
    }

#ifdef _YL_LPS__TO_BE_REPLACED_BY_PATCH
     // TODO: BUG?   tolerance = ceil(inactive_slots*total_drift*10/1000000+DSM_DRIFT/625)*2
     // TODO: (unit: 0.5 slots, but LSB and MSB has no effects)
#define XTOL_MAX_SLOT 62
    if (g_efuse_lps_setting_1.new_tol_cal)
    {
         ...
    }
    else
#else
    if(1)
#endif
    {
        total_drift = (UCHAR) ((LMP_MAX_DRIFT + max_remote_drift) / 10);

        /* If we are slave , then open the receive window , to
         * accomodate the drift.
         * A constant k is added to power power time to accomodate for
         * switching between low power clock and sysclk in DSM mode.
         */
        low_power_time =  (inactive_slots * 625)/10 + (DSM_DRIFT * 1000);
        if(low_power_time > 0)
        {
            drift = (low_power_time * total_drift)/10;
            tolerance = (UINT16) ((drift/625)/1000); /* drift_slots = drift_ppm*lps_time_us/625/1000 + ??? */
            if((tolerance == 0) && (low_power_time > 2000))
            {
                tolerance = 1;
            }
        }
        else
        {
            tolerance = 0;
        }

        if(tolerance > 0)
        {
            tolerance = (UINT16) (tolerance << 1);
            if (tolerance & 0x03)
            {
                    tolerance = (UINT16) ( ((tolerance >> 2) + 1) << 2); // TODO:  make it even number of slots  WHY?
            }
        }
    }

#ifdef COMPILE_SNIFF_MODE
    /* Adjust tolerance to make sure that there are at least 2 attempts. */
    // TODO:  WHY?
#ifdef _YL_LPS__TO_BE_REPLACED_BY_PATCH
        if ( (ce_ptr->in_sniff_mode == TRUE) && (tolerance != 0) && g_efuse_lps_setting_1.sniff_tol_adjust )
#else
        if ( (ce_ptr->in_sniff_mode == TRUE) && (tolerance != 0) )
#endif
        {
            if( (ce_ptr->sniff_attempt / tolerance ) < 2 )
            {
                tolerance = (UINT16) (ce_ptr->sniff_attempt >> 1);
                tolerance = (UINT16) ((tolerance + 3) & (~0x0003) );

#ifdef _YL_LPS
                LPS_DBG_LOG(LOG_LEVEL_LOW, MODIFIED_XTOL, 0, 0);
#endif
#ifdef ENABLE_LOGGER_LEVEL_2
                LMP_LOG_INFO(LOG_LEVEL_LOW, MODIFIED_XTOL, 0, 0);
#endif

                /* If the tolerance has become 0, make it 4. */
                if (tolerance == 0)
                {
                    tolerance = 4;
                }
            }
        }
#endif

    return (UCHAR)tolerance;
}
#endif


#ifdef POWER_SAVE_FEATURE

/**
 * Initializes power save feature parameters.
 *
 * \param None.
 *
 * \return None.
 */
void lc_init_sleep_mode_param(void)
{
    sleep_mode_param.bb_sm_sts = BB_NORMAL_MODE;
    sleep_mode_param.bb_sm_wakeup_inst = LC_SM_INVALID_INSTANT;

#ifdef _CCH_RTL8723A_B_CUT
// _CCH_ECO_LPS_

    memset(&sleep_mode_param, 0x0, sizeof(sleep_mode_param));
#ifdef LPS_NEW_CAL_XTOL
    sleep_mode_param.lps_drift_ppm_for_xtol = LMP_MAX_DRIFT;
#endif
    memset(&lc_sniff_window_rx_ind, 0x0, sizeof(LC_MAX_NUM_OF_LUT_EX_TABLES));
    memset(&lc_sniff_sup_count, 0x0, sizeof(LC_MAX_NUM_OF_LUT_EX_TABLES));
    bb_write_baseband_register_func_imp_count = 0;
    g_lps_scan_status_stay_count = 0;

#ifdef _ENABLE_BTON_POWER_SAVING_
#ifdef _2801_BTON_DESIGN_
    bton_32k_cal_ini();
#endif
    bton_32k_cal_en();
#endif

#endif

#ifdef _MODI_LPS_AFTER_RTL8703B_
    sleep_mode_param.lps_32k_cal_max = LPS_32K_UNLOCK_MAX_TIME;
#endif

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
    sleep_mode_param.run_dlps_flow = g_efuse_rsvd_2.enable_deep_lps_mode;
#endif

    return;
}
#ifndef _REDUCE_LPS_AFTER_RTL8703B_
/**
 * Programs sleep mode to the baseband.
 *
 * \param wakeup_instant The wakeup instant by which sleep mode should have
 *                       terminated. Note that allowance for the tolerance
 *                       value should have been taken by the callee,
 * \param piconet_id The physical piconet ID of the connection.
 *
 * \return None.
 */
void lc_program_sm_mode(UINT32 wakeup_instant, UCHAR piconet_id)
{

#if 0
    UINT32 clock;
    UINT32 diff;
    DEF_CRITICAL_SECTION_STORAGE;

    /* Check the configuration bit. */
    if (!IS_ENABLE_POWER_SAVE)
    {
        RT_BT_LOG(GRAY, LC_3342, 0, 0);
        return;
    }

    /* In scatternet, if scans are enabled, we cant program sleep mode. */
#ifdef _YL_LPS_DSM_IGNORE_SCAN_ENABEL_CONDITION
    if(0)
#else
    if( (lmp_self_device_data.scan_enable != 0) && IS_SUPPORT_SCATTERNET)
#endif
    {
        RT_BT_LOG(GRAY, LC_3357, 0, 0);
        return;
    }

    /* Check if there is only one active connection.
    Otherwise, make sure that all the connections are parked. */
    if (
#ifdef ENABLE_SCO
        (lmp_self_device_data.total_no_of_sco_conn == 0) &&
#endif
#ifdef COMPILE_ESCO
        (lmp_self_device_data.number_of_esco_connections == 0) &&
#endif
        ((lmp_self_device_data.number_of_hlc == 1) ||
         ( (lmp_self_device_data.number_of_hlc == 0)
#ifdef COMPILE_PARK_MODE
           && (lmp_self_device_data.number_of_parked_dev != 0)
#endif
         ) ) )
    {
#ifdef _YL_LPS
        UINT16 sm_guard_interval;
        if (g_efuse_lps_setting_1.sm_guard_interval == 3)
        {
            sm_guard_interval = 16;
        }
        else
        {
            sm_guard_interval = 2 + g_efuse_lps_setting_1.sm_guard_interval;
        }
        wakeup_instant = (wakeup_instant - sm_guard_interval);
#else
        wakeup_instant = (wakeup_instant - SLEEP_MODE_GUARD_INTERVAL);
#endif

        /* Check if wakeup instant is in the future. */

        MINT_OS_ENTER_CRITICAL();
        // TODO: double check the LPS conditions

        lc_get_clock_in_scatternet(&clock, piconet_id);
        clock = clock >> 1;
        diff = (wakeup_instant | BIT27) - clock;
        diff = diff & ~BIT27;

#ifdef _YL_LPS
#endif
#ifndef _YL_LPS
#define MINIMUM_SLEEP_INTERVAL 2  /* mindtree version */
        if(diff <= MINIMUM_SLEEP_INTERVAL || diff > 0xFFFF)
#else
        UINT16 min_sm_interval = (2 << (g_efuse_lps_setting_1.min_sm_interval+1));
        if(diff <= (min_sm_interval) || diff > 0xFFFF)
#endif
        {
            MINT_OS_EXIT_CRITICAL();
            return;
        }

        sleep_mode_param.bb_sm_wakeup_inst = wakeup_instant;
        BB_write_baseband_register(WAKEUP_INSTANT_REGISTER,
                                   (UINT16)wakeup_instant);

        BB_write_baseband_register(POWER_CONTROL_REGISTER,
                                   (lc_power_ctr_config | BB_PROGRAM_SM_MODE));
        sleep_mode_param.bb_sm_sts = BB_SLEEP_MODE;
        MINT_OS_EXIT_CRITICAL();
        RT_BT_LOG(GRAY, LC_3410, 1, wakeup_instant);
    }
#endif
    return;
}


/**
 * Programs deep sleep mode to the baseband.
 *
 * \param wakeup_instant The wakeup instant by which sleep mode should have
 *                       terminated. Note that allowance for the tolerance
 *                       value should have been taken by the callee,
 * \param piconet_id The physical piconet ID of the connection.
 *
 * \return None.
 */
void lc_program_dsm_mode(UINT32 wakeup_instant, UCHAR piconet_id)
{
#if 0
#ifdef _YL_LPS
    UINT32 clock;
    UINT32 diff;
#endif

    DEF_CRITICAL_SECTION_STORAGE;
    UINT16 osc_delay;

#ifdef _YL_LPS
    LPS_DBG_LOG(GRAY, LPS_LOG_004, 2, wakeup_instant, piconet_id);
#endif

    /* Check the configuration bit. */
    if (!IS_ENABLE_POWER_SAVE)
    {
#ifdef _YL_LPS
        LPS_DBG_LOG(RED, LPS_LOG_005, 0, 0);
#endif
        return;
    }

    // TODO: how to saving pow when scan_enable ! =0 ??
    /* In scatternet, if scans are enabled, we cant program sleep mode. */
    if(lmp_self_device_data.scan_enable != 0)
    {
        RT_BT_LOG(GRAY, LC_3447, 0, 0);
#ifdef _YL_LPS
        LPS_DBG_LOG(RED, LPS_LOG_006, 0, 0);
#endif
        return;
    }

    // TODO: Need also check LE status !!!
    // TODO: Should not go into LPS mode when there is important tasks or HCI data to send?
    // TODO: need also consider HCI traffic status
    // TODO: need also review the Timer2 sleep conditions !!
    /* Check if there is only one active connection.
    Otherwise, make sure that all the connections are parked. */
    if (
#ifdef ENABLE_SCO
        (lmp_self_device_data.total_no_of_sco_conn == 0) &&
#endif
#ifdef COMPILE_ESCO
        (lmp_self_device_data.number_of_esco_connections == 0) &&
#endif
        ((lmp_self_device_data.number_of_hlc == 1) ||
         ( (lmp_self_device_data.number_of_hlc == 0)
#ifdef COMPILE_PARK_MODE
           && (lmp_self_device_data.number_of_parked_dev != 0)
#endif
         ) ) )
    {
        lps_gpio_one_pull_high(LPS_GPIO_PGM_SNIFF_DSM);
        osc_delay = otp_str_data.bw_rf_osc_delay;

#ifdef _YL_LPS
        // TODO: wakeup_instant should also take tolerance into account ?? seems not
        UINT16 dsm_guard_interval;
        if (g_efuse_lps_setting_1.dsm_guard_interval == 7)
        {
            dsm_guard_interval = 16;
        }
        else
        {
            dsm_guard_interval = 2 + g_efuse_lps_setting_1.dsm_guard_interval;
        }
        wakeup_instant = (wakeup_instant - osc_delay)
                         - dsm_guard_interval;
#else
        wakeup_instant = (wakeup_instant - osc_delay)
                         - SLEEP_MODE_GUARD_INTERVAL;
#endif

#ifdef _YL_LPS_ENABLE_DSM_DIFF_CHECK
        MINT_OS_ENTER_CRITICAL();
        // TODO: double check the LPS conditions
        lc_get_clock_in_scatternet(&clock, piconet_id);
        clock = clock >> 1;
        diff = (wakeup_instant | BIT27) - clock;
        diff = diff & ~BIT27;


        // TODO: What is the MIN for 8723???
        // TODO: configured by Efuse 2 ~ 5

        /* Note: original Mindtree did NOT check this condidtion */
        UINT16 min_dsm_interval = (2 << (g_efuse_lps_setting_1.min_dsm_interval+1));
        if(diff <= (min_dsm_interval) || diff > 0xFFFF)
        {
            MINT_OS_EXIT_CRITICAL();
            lps_gpio_one_pull_low(LPS_GPIO_PGM_SNIFF_DSM);
    #ifdef _YL_LPS
            LPS_DBG_LOG(RED, LPS_LOG_007, 2, diff, min_dsm_interval);
    #endif
            return;
        }
#endif
        lps_gpio_one_pull_high(LPS_GPIO_EXECUTE_LPS);

#ifdef _YL_LPS_NEW_PRG_DSM
    #ifndef _YL_LPS_ENABLE_DSM_DIFF_CHECK
        MINT_OS_ENTER_CRITICAL();
    #endif
        sleep_mode_param.bb_sm_wakeup_inst = wakeup_instant;
        sleep_mode_param.bb_sm_piconet_id = piconet_id;
        execute_lps_mode_procedure_6128(1, wakeup_instant);
        sleep_mode_param.bb_sm_sts = BB_DEEP_SLEEP_MODE;
        MINT_OS_EXIT_CRITICAL();


    #if 0// _YL_LPS_FAKE_COMBO_PLATFORM_40M_RECOVERY // for test only
        UINT16 bb_pow_ctrl_reg;
        BT_SYS_CTRL_REG_S_TYPE bt_sys_reg;
        while(1)
        {
            bb_pow_ctrl_reg = BB_read_baseband_register(POWER_CONTROL_REGISTER);
            if(bb_pow_ctrl_reg & 0x0001)
            {
                    bt_sys_reg.d32 = VENDOR_READ(BT_SYS_CTRL_REG);
                    bt_sys_reg.b.cpu_40mhz_en =1;
                    VENDOR_WRITE(BT_SYS_CTRL_REG, bt_sys_reg.d32);
                    lc_handle_sm_intr(); // TODO: is this valid call??
                    break;
            }
        }
    #endif

        lps_gpio_one_pull_low(LPS_GPIO_EXECUTE_LPS);
        lps_gpio_one_pull_low(LPS_GPIO_PGM_SNIFF_DSM);
#else

//    #ifdef _YL_LPS_ENABLE_DSM_DIFF_CHECK
//        MINT_OS_EXIT_CRITICAL();
//    #endif

        sleep_mode_param.bb_sm_wakeup_inst = wakeup_instant;
        BB_write_baseband_register(WAKEUP_INSTANT_REGISTER,
                                   (UINT16)(wakeup_instant));
    #ifndef _YL_LPS_ENABLE_DSM_DIFF_CHECK
        MINT_OS_ENTER_CRITICAL();
    #endif
        BB_write_baseband_register(POWER_CONTROL_REGISTER,
                                   (lc_power_ctr_config | BB_PROGRAM_DSM_MODE));
        sleep_mode_param.bb_sm_sts = BB_DEEP_SLEEP_MODE;

        lps_gpio_one_pull_low(LPS_GPIO_EXECUTE_LPS);
        MINT_OS_EXIT_CRITICAL();
        lps_gpio_one_pull_low(LPS_GPIO_PGM_SNIFF_DSM);
#endif
//        LMP_LOG_INFO(LOG_LEVEL_HIGH, DSM_CLK_OSC, 2,
//                     sleep_mode_param.bb_sm_wakeup_inst, osc_delay);
    }
#ifdef _YL_LPS
    else
    {
        LPS_DBG_LOG(RED, LPS_LOG_010, 3,
                            lmp_self_device_data.total_no_of_sco_conn,
                            lmp_self_device_data.number_of_esco_connections,
                            lmp_self_device_data.number_of_hlc);
    }
#endif
#endif
    return;
}
#endif

#if defined(_CCH_RTL8723A_B_CUT) && defined(_CCH_LPS_USING_STATE_)
// _CCH_ECO_LPS_

void lps_period_state_machine(UINT8 state, UINT8 state_bitmap, UINT8 count, UINT8 lps_mode, UCHAR piconet_id, UINT32 wakeup_instant)
{

#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8723_RCP_
    if (rcp_lps_period_state_machine != NULL)
    {
        if ( rcp_lps_period_state_machine((void*)(&state), state_bitmap, &count, lps_mode, piconet_id, wakeup_instant) )
        {
            return;
        }
    }
#endif
#endif
    UCHAR temp = sleep_mode_param.lps_lpm_lps_mode;
    if( lc_lps_double_check_dsm_cond(temp) == API_FAILURE)
    {
        return;
    }

    if(g_lps_timer_counter>0)
    {
        sleep_mode_param.lps_period_state = LPS_PERIOD_NULL;
        return;
    }

    INT8 state_ind = 0;
    UINT8 count_temp;
    UINT16 reg_val;

    if( temp > 3)
    {
        return;   // 20120310
    }else if(( temp == LPS_TIMER2_WO_SCAN)&&(lmp_self_device_data.scan_enable != 0))
    {
        return;
    }

    if( (lmp_self_device_data.scan_enable != 0)&&(g_efuse_lps_setting_2.timer2_scan_en ==0) )
    {
        return;
    }

    if(lmp_self_device_data.scan_enable == 1)
    {
        return;
    }

    if( state > LPS_PERIOD_NULL)
    {
        sleep_mode_param.lps_period_state = LPS_PERIOD_NULL;
        state = LPS_PERIOD_NULL;
        return;
    }

    sleep_mode_param.lps_procedure_start = 1;
    sleep_mode_param.lps_period_count ++;
//    RT_BT_LOG(BLUE, CCH_LPS_001, 4,count, state, state_bitmap, BB_read_native_clock());


    #ifdef _LPS_LOG_EN_TEST_
    // Only use for test scan report
    reg_val = BB_read_baseband_register(VOICE_SETTING_REGISTER);
    if(reg_val & BIT12)
    {
        RT_BT_LOG(RED, CCH_DBG_019, 0,0);
    }
    #endif


    if(lps_mode < LPS_TIMER2_WITH_SCAN)
    {
  //      sleep_mode_param.lps_lpm_end_flag =  1;   // 20120310
        sleep_mode_param.lps_period_state = LPS_PERIOD_NULL;
        sleep_mode_param.lps_lpm_lps_mode = lps_mode;
        sleep_mode_param.lps_lpm_pid = piconet_id;
        sleep_mode_param.lps_lpm_wakeup_instance = wakeup_instant;
    }



    UINT8 status = TRUE;

    if(sleep_mode_param.lps_lpm_lps_mode < LPS_TIMER2_WITH_SCAN)
    {
        status = lc_check_lps_for_link(0,0,1);
    }else
    {
        if(sleep_mode_param.lps_lpm_lps_mode == LPS_TIMER2_WITH_SCAN)
        {
            status = lc_check_lps_for_idle(1);
        }else if(sleep_mode_param.lps_lpm_lps_mode == LPS_TIMER2_WO_SCAN)
        {
            status = lc_check_lps_for_idle(0);
        }
    }


    if( status == FALSE)
    {
        RT_BT_LOG(RED, YL_DBG_HEX_4, 4, lps_mode, state, state_bitmap, state_ind);
        return;
    }

    if(sleep_mode_param.lps_lpm_end_flag ==0)
    {
        sleep_mode_param.lps_lpm_lps_mode = lps_mode;
    }else
    {
        lps_mode = sleep_mode_param.lps_lpm_lps_mode;
    }


    count_temp = 0;



    if( state > 0 )
    {

        for( state_ind=LPS_PERIOD_NULL; state_ind>=state; state_ind-- )
        {
            if( (state_bitmap >>state_ind) & 0x01 )
            {
                 count_temp += sleep_mode_param.lps_period_state_num[state_ind];
            }
        }

        if( count < count_temp )
        {
            return;
        }


        for( state_ind=(state-1); state_ind>0; state_ind-- )
        {
            if( (state_bitmap >>state_ind)&0x01 )
            {
                 break;
            }
        }

    }else
    {
        state_ind = 0;
    }

#ifdef _LPS_LOG_EN_TEST_
    RT_BT_LOG(BLUE, CCH_LPS_001, 4,count, state_ind, state_bitmap, lps_mode);
#endif

#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8703B_RCP_
    if (rcp_lps_period_state_machine_mid != NULL)
    {
        if ( rcp_lps_period_state_machine_mid((void*)(&state), state_ind) )
        {
            return;
        }
    }
#endif
#endif

    reg_val = BB_read_baseband_register(VOICE_SETTING_REGISTER);

    if( state != 0 )
    {
        // exit each state
        if( state == LPS_PERIOD_INQ_SCAN)
        {  // Inquiry Scan Need to Kill Scan During Scan Response =1 (for Start Inquiry Scan Immediately)

#ifdef _LPS_FOR_8821_
            if( ((sleep_mode_param.scan_end_flag&BIT1) == 0)&&
                 (g_lps_scan_status_stay_count < (g_efuse_lps_setting_5.lps_scan_protect_time + 1)) )
            {
                g_lps_scan_status_stay_count ++;
                if( sleep_mode_param.lps_period_count >0 )
                {
                    sleep_mode_param.lps_period_count --;
                }

                LPS_CCH_LOG(BLUE, YL_DBG_HEX_2, 2,g_lps_scan_status_stay_count, sleep_mode_param.scan_end_flag);

                return;
            }
#endif

            g_lps_scan_status_stay_count = 0;

            #ifdef _LPS_LOG_EN_TEST_
            if(reg_val & BIT12)
            {
                LPS_CCH_LOG(BLUE, CCH_DBG_106, 2, 0,0);
            }
           #endif
//            lc_kill_scan_mode();


#ifdef _MODI_LPS_AFTER_RTL8703B_
            lps_kill_inq_scan();
#else

#ifdef _LPS_FOR_8821_
            sleep_mode_param.scan_end_flag = 0;
            BZ_REG_S_INQ_SCAN_WINDOW isw;

            *(UINT16*)&isw = BB_read_baseband_register(INQ_SCAN_WINDOW_REGISTER);
            isw.is_int_en = 0;
            BB_write_baseband_register(INQ_SCAN_WINDOW_REGISTER, *(UINT16*)&isw);
#endif

#ifdef _DAPE_TEST_NEW_HW_INSTRUCTION
            BB_write_baseband_register(INSTRUCTION_REGISTER, BB_KILL_INQUIRY_SCAN);
#else
            BB_write_baseband_register(INSTRUCTION_REGISTER, BB_KILL_INQUIRY);
#ifdef _FREE_FHS_PKT_WHEN_KILL_INQUIRY_
            lc_free_fhs_pkt_buffer();
#endif
#endif
#endif
        }else if( state == LPS_PERIOD_PAG_SCAN)
        {

#ifdef _LPS_FOR_8821_

            if( ((sleep_mode_param.scan_end_flag&BIT0) == 0)&&
                 (g_lps_scan_status_stay_count < (g_efuse_lps_setting_5.lps_scan_protect_time + 1)) )
            {
                g_lps_scan_status_stay_count ++;
                if( sleep_mode_param.lps_period_count >0 )
                {
                    sleep_mode_param.lps_period_count --;
                }
                //#ifdef _LPS_LOG_EN_TEST_
                //RT_BT_LOG(BLUE, YL_DBG_HEX_2, 2,g_lps_scan_status_stay_count, sleep_mode_param.scan_end_flag);
                //#endif
                return;
            }
#endif

//            lc_kill_scan_mode();
            // need to check scan status

#ifndef _LPS_FOR_8821_  // Only can use in 8723a b cut
            if( (reg_val & BIT12)&&(g_lps_scan_status_stay_count < LPS_SCAN_PROTECT_TIME) )
            {
                g_lps_scan_status_stay_count ++;
                if( sleep_mode_param.lps_period_count >0 )
                {
                    sleep_mode_param.lps_period_count --;
                }
                #ifdef _LPS_LOG_EN_TEST_
                LPS_CCH_LOG(BLUE, CCH_DBG_106, 2, 1,g_lps_scan_status_stay_count);
                #endif
                return;
            }
#endif
            g_lps_scan_status_stay_count = 0;

#ifdef _MODI_LPS_AFTER_RTL8703B_
            lps_kill_inq_scan();
#else
#ifdef _LPS_FOR_8821_
            sleep_mode_param.scan_end_flag = 0;
            BZ_REG_S_PAGE_SCAN_WINDOW psw;

            *(UINT16*)&psw = BB_read_baseband_register(PAGE_SCAN_WINDOW_REGISTER);
            psw.ps_int_en = 0;
            BB_write_baseband_register(PAGE_SCAN_WINDOW_REGISTER, *(UINT16*)&psw);
#endif

#ifdef _DAPE_TEST_NEW_HW_INSTRUCTION
            BB_write_baseband_register(INSTRUCTION_REGISTER, BB_KILL_PAGE_SCAN);
#else
            BB_write_baseband_register(INSTRUCTION_REGISTER, BB_KILL_PAGE);
#endif
//            lc_pagescan_piconet_id = SCA_PICONET_INVALID;
#endif

        }
#ifdef LE_MODE_EN
        else if (IS_BT40)
        {
            if( state == LPS_PERIOD_LE_ADV)
            {
                if(((sleep_mode_param.scan_end_flag & BIT2) == 0))
                {
                    if(g_lps_scan_status_stay_count < (g_efuse_lps_setting_5.lps_scan_protect_time + 1))
                    {
                        g_lps_scan_status_stay_count ++;
                        if( sleep_mode_param.lps_period_count >0 )
                        {
                            sleep_mode_param.lps_period_count --;
                        }

                        LPS_CCH_LOG(BLUE, YL_DBG_HEX_2, 2,g_lps_scan_status_stay_count, sleep_mode_param.scan_end_flag);

                        return;
                    }
                }
                g_lps_scan_status_stay_count = 0;
#ifdef _MODI_LPS_AFTER_RTL8703B_
                sleep_mode_param.le_adv_count = 0;
#endif
                ll_driver_disable_advertising();
            }
            else if( state == LPS_PERIOD_LE_SCAN)
            {
                if (((sleep_mode_param.scan_end_flag & BIT3) == 0))
                {
                    if(g_lps_scan_status_stay_count < (g_efuse_lps_setting_5.lps_scan_protect_time + 1))
                    {
                        g_lps_scan_status_stay_count ++;
                        if( sleep_mode_param.lps_period_count >0 )
                        {
                            sleep_mode_param.lps_period_count --;
                        }

                        LPS_CCH_LOG(BLUE, YL_DBG_HEX_2, 2,g_lps_scan_status_stay_count, sleep_mode_param.scan_end_flag);

                        return;
                    }
                }
                g_lps_scan_status_stay_count = 0;
                ll_driver_disable_scanning();
            }
#ifdef CONFIG_TV_POWERON_LPS
            else if (state == LPS_PERIOD_RSVD1)
            {
                if (((sleep_mode_param.scan_end_flag & BIT3) == 0))
                {
                    if(g_lps_scan_status_stay_count < (g_efuse_lps_setting_5.lps_scan_protect_time + 1))
                    {
                        g_lps_scan_status_stay_count ++;
                        if( sleep_mode_param.lps_period_count >0 )
                        {
                            sleep_mode_param.lps_period_count --;
                        }

                        LPS_CCH_LOG(BLUE, YL_DBG_HEX_2, 2,g_lps_scan_status_stay_count, sleep_mode_param.scan_end_flag);

                        return;
                    }
                }
                g_lps_scan_status_stay_count = 0;
                ll_driver_disable_scanning();
            }
#endif
        }
#endif
    }



    sleep_mode_param.lps_period_state = state_ind;

    if(state_ind == 0)
    {

        lmp_stop_regular_sw_timers();

#if 0
#ifdef _LPS_STOP_REG_TIMER_
    /* stop afh corresponding sw timers */
    OS_STOP_TIMER(la_period_timer, 0);
    OS_STOP_TIMER(la_classify_timer, 0);

    /* stop background 20sec timer */
    OS_STOP_TIMER(dbg_tid_timer, 0);

#ifdef _IS_ASIC_
    if (otp_str_data.EFuse_ThermalUpdateInterval != 0)
    {
        OS_STOP_TIMER(g_rtl8723_btrf_thermal_value_timer, 0);
    }
#endif
#endif
#endif

#ifdef _LPS_LOG_EN_TEST_
        LPS_CCH_LOG(BLUE, CCH_DBG_033, 1,bb_write_baseband_register_func_imp_count);
#endif
        // enter LPS
        sleep_mode_param.lps_period_count = 0;
        sleep_mode_param.lps_period_state = LPS_PERIOD_NULL;
        //sleep_mode_param.lps_lpm_end_flag =  0;

        if((lps_mode >= LPS_TIMER2_WITH_SCAN)
	    &&(sleep_mode_param.lps_period_state_bitmap == 0) )
        {
            lps_mode = LPS_TIMER2_WO_SCAN;
        }
        lps_period_state_enter_lps(lps_mode);
    }else
    {
        sleep_mode_param.scan_end_flag = 0;

        // enter each state
        if(state_ind == LPS_PERIOD_INQ_SCAN)
        {

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
            lc_write_scan_mode(BIT0);
#endif
            lc_kill_scan_mode();

#ifdef _LPS_FOR_8821_
            // need to place after lc_kill_scan_mode
            sleep_mode_param.scan_end_flag = 0;

            BZ_REG_S_INQ_SCAN_WINDOW isw;

            *(UINT16*)&isw = BB_read_baseband_register(INQ_SCAN_WINDOW_REGISTER);
            isw.is_int_en = 1;
            BB_write_baseband_register(INQ_SCAN_WINDOW_REGISTER, *(UINT16*)&isw);
#endif

            lc_retrieve_inq_scan();
        }
        else if(state_ind == LPS_PERIOD_PAG_SCAN)
        {

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
            lc_write_scan_mode(BIT1);
#endif
            lc_kill_scan_mode();

#ifdef _LPS_FOR_8821_
            // need to place after lc_kill_scan_mode
            sleep_mode_param.scan_end_flag = 0;

            BZ_REG_S_PAGE_SCAN_WINDOW psw;

            *(UINT16*)&psw = BB_read_baseband_register(PAGE_SCAN_WINDOW_REGISTER);
            psw.ps_int_en = 1;
            BB_write_baseband_register(PAGE_SCAN_WINDOW_REGISTER, *(UINT16*)&psw);
#endif
            lc_retrieve_page_scan();
        }
#ifdef LE_MODE_EN
        else if (IS_BT40)
        {
            if( state_ind == LPS_PERIOD_LE_ADV)
            {
#ifdef _MODI_LPS_AFTER_RTL8703B_
                sleep_mode_param.le_adv_count = 0;
#endif

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_LE_
                ll_driver_enable_advertising(TRUE);
#else
                ll_driver_enable_advertising();
#endif
            }
            else if( state_ind == LPS_PERIOD_LE_SCAN)
            {
#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_LE_
                ll_driver_enable_scanning(TRUE);
#else
                ll_driver_enable_scanning();
#endif
            }
#ifdef CONFIG_TV_POWERON_LPS
            else if (state_ind == LPS_PERIOD_RSVD1)
            {
                tv_poweron_enable_le_scan(&tv_poweron.scan, TRUE);
            }
#endif
        }
#endif
    }

    return;
}


void lps_period_state_enter_lps(UINT8 lps_mode)
{

#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8723_RCP_
    if (rcp_lps_period_state_enter_lps != NULL)
    {
        if ( rcp_lps_period_state_enter_lps((void*)(&lps_mode)) )
        {
            return;
        }
    }
#endif
#endif


#ifdef _MODI_LPS_AFTER_RTL8703B_
    sleep_mode_param.lps_period_count = 0;
    sleep_mode_param.lps_period_state = LPS_PERIOD_NULL;
#endif

    if(sleep_mode_param.lps_task_flag)
    {   // Already have LPS task in Task Queue

        UINT8 ii;
        INT32 start_index;
        UINT8 queue_index = LC_RX_TASK_PRI;
        INT32 queue_task = LC_ENTER_LPS_MODE;

#ifdef COMPILE_CHANNEL_ASSESSMENT
	    if( g_efuse_lps_setting_2.lps_pri == 0 )
	    {
	        queue_index = CH_AS_TASK_PRI;
            queue_task = CH_AS_TASK_ENTER_LPS_MODE;
	    }
#endif


        start_index = queue_mgr.queue_array[ queue_index ].start_index;

        for (ii = 0; ii< (queue_mgr.queue_array[ queue_index ].q_length); ii++)
        {

            if (queue_mgr.queue_array[ queue_index ].item_array[start_index].type == queue_task)
            {
                LPS_CCH_LOG(YELLOW, YL_DBG_HEX_1, 1,queue_index, queue_mgr.queue_array[ queue_index ].q_length);
                return;
            }

            start_index ++;
            if (start_index == queue_mgr.queue_array[ queue_index ].max_items)
            {
                start_index = 0x00;
            }
        }

    }else
    {
        sleep_mode_param.lps_task_flag = 1;
    }


    switch(lps_mode)
    {
        case LPS_SNIFF_SM :
            lc_post_lps_mode_signal(LPS_SNIFF_SM, sleep_mode_param.lps_lpm_wakeup_instance, sleep_mode_param.lps_lpm_pid);
            break ;

        case LPS_SNIFF_DSM :
            lc_post_lps_mode_signal(LPS_SNIFF_DSM, sleep_mode_param.lps_lpm_wakeup_instance, sleep_mode_param.lps_lpm_pid);
            break ;

        case LPS_TIMER2_WITH_SCAN :
            lc_post_lps_mode_signal(LPS_TIMER2_WITH_SCAN, 0, 0);
            break ;

        case LPS_TIMER2_WO_SCAN :
            lc_post_lps_mode_signal(LPS_TIMER2_WO_SCAN, 0, 0);
            break ;

        default :
            sleep_mode_param.lps_task_flag = 0;
            break;
    }


}

#ifdef _MODI_LPS_AFTER_RTL8821B_TC_
void lps_period_state_machine_fast(UINT32 ps_clock, UINT16 ps_duration)
{

#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8821B_TC_RCP_
    if (rcp_lps_period_state_machine_fast != NULL)
    {
        if ( rcp_lps_period_state_machine_fast((void*)(&ps_clock), ps_duration) )
        {
            return;
        }
    }
#endif
#endif


// _CCH_ECO_LPS_
    if(g_efuse_lps_setting_4.lps_use_state)
    {


        sleep_mode_param.lps_lpm_end_flag = 1;
        sleep_mode_param.lps_lpm_wakeup_instance = ps_clock;
        sleep_mode_param.lps_lpm_lps_mode = LPS_SNIFF_DSM;

        sleep_mode_param.lps_link_interval = ps_duration;

        if(g_efuse_lps_setting_4.lps_use_state_fast_lps)
        {
            sleep_mode_param.lps_link_interval = ps_duration;

            UINT8 enter_lps_now = 0;

            if( lmp_self_device_data.scan_enable
#ifdef LE_MODE_EN
                || (IS_BT40 && (ll_manager.adv_unit.enable || ll_manager.scan_unit.enable))
#endif
                )
            {
                if(lmp_self_device_data.lps_sniff_scan_en == 0)
                {
                    if( lmp_self_device_data.scan_enable&1 )
                    {

#ifdef _MODI_LPS_AFTER_RTL8703B_
                        lps_kill_inq_scan();
#else

#ifdef _DAPE_TEST_NEW_HW_INSTRUCTION
                        BB_write_baseband_register(INSTRUCTION_REGISTER, BB_KILL_INQUIRY_SCAN);
#else
                        BB_write_baseband_register(INSTRUCTION_REGISTER, BB_KILL_INQUIRY);
#ifdef _FREE_FHS_PKT_WHEN_KILL_INQUIRY_
                        lc_free_fhs_pkt_buffer();
#endif
#endif
#endif
                    }
                    if( lmp_self_device_data.scan_enable&2 )
                    {
                        g_lps_scan_status_stay_count = 0;

#ifdef _MODI_LPS_AFTER_RTL8703B_
                        lps_kill_page_scan();
#else

#ifdef _DAPE_TEST_NEW_HW_INSTRUCTION
                        BB_write_baseband_register(INSTRUCTION_REGISTER, BB_KILL_PAGE_SCAN);
#else
                        BB_write_baseband_register(INSTRUCTION_REGISTER, BB_KILL_PAGE);
#endif
                        //lc_pagescan_piconet_id = SCA_PICONET_INVALID;
#endif
                    }

#ifdef LE_MODE_EN
                    if (IS_BT40)
                    {
                        if(ll_manager.adv_unit.enable)
                        {
                            ll_driver_disable_advertising();
                        }

                        if(ll_manager.scan_unit.enable)
                        {
                            ll_driver_disable_scanning();
                        }
                    }
#endif
                    enter_lps_now = 1;
                }
            }

            if((lmp_self_device_data.scan_enable == 0)
#ifdef LE_MODE_EN
                && (!IS_BT40 || ((ll_manager.adv_unit.enable ==0) && (ll_manager.scan_unit.enable==0)))
#endif
                )
            {
                lmp_self_device_data.lps_sniff_scan_en = 0;
                enter_lps_now = 1;
            }

#ifdef _LPS_LOG_EN_1
                RT_BT_LOG(BLUE, YL_DBG_HEX_5, 5,enter_lps_now, lmp_self_device_data.lps_sniff_scan_en, lmp_self_device_data.lps_sniff_scan_count,
                                                  sleep_mode_param.lps_period_state_bitmap, ps_clock);
#endif

            if( enter_lps_now)
            {
                lps_period_state_enter_lps(LPS_SNIFF_DSM);
            }else
            {
                UINT8 state = sleep_mode_param.lps_period_state;
                UINT8 bitmap = sleep_mode_param.lps_period_state_bitmap;
                UINT8 count = sleep_mode_param.lps_period_count;

#ifdef _MODI_LPS_STATE_WITH_INTR_
                if(g_efuse_rsvd_2.lps_state_with_intr)
               	{
                    sleep_mode_param.lps_lpm_lps_mode = LPS_SNIFF_DSM;
                    lc_post_lps_stste_signal(LPS_SNIFF_DSM);
                }
                else
#endif
                {
                    lps_period_state_machine(state, bitmap, count, LPS_SNIFF_DSM, 0, ps_clock);
                }

            }
        }

    }
}
#endif

#ifdef _MODI_LPS_AFTER_RTL8703B_
void lps_kill_inq_scan()
{
    sleep_mode_param.scan_end_flag = 0;

    BZ_REG_S_INQ_SCAN_WINDOW isw;

    *(UINT16*)&isw = BB_read_baseband_register(INQ_SCAN_WINDOW_REGISTER);
    isw.is_int_en = 0;
    BB_write_baseband_register(INQ_SCAN_WINDOW_REGISTER, *(UINT16*)&isw);

    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_KILL_INQUIRY_SCAN);
}


void lps_kill_page_scan()
{
    sleep_mode_param.scan_end_flag = 0;

    BZ_REG_S_PAGE_SCAN_WINDOW psw;

    *(UINT16*)&psw = BB_read_baseband_register(PAGE_SCAN_WINDOW_REGISTER);
    psw.ps_int_en = 0;
    BB_write_baseband_register(PAGE_SCAN_WINDOW_REGISTER, *(UINT16*)&psw);

    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_KILL_PAGE_SCAN);
    //lc_pagescan_piconet_id = SCA_PICONET_INVALID;
}


#endif

UINT8  lc_check_lps_for_link(UCHAR is_LEGACY, UCHAR is_LE, UCHAR not_sure)
{
#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8821_RCP_
    UCHAR return_status = 0;
    if (rcp_lc_check_lps_for_link != NULL)
    {
        if ( rcp_lc_check_lps_for_link((void*)(&return_status), is_LEGACY, is_LE, not_sure) )
        {
            return return_status;
        }
    }
#endif
#endif

#ifdef _MODI_LPS_AFTER_RTL8821B_TC_
    if (IS_USE_FOR_BQB ||
        (lmp_self_device_data.test_mode == HCI_DUT_LOOPBACK_MODE) ||
        (lmp_self_device_data.test_mode == HCI_DEVICE_UNDER_TEST_MODE) ||
        (lmp_self_device_data.host_enable_test_mode))
    {
        /* no enter lps mode in BQB or DUT test mode */
        return FALSE;
    }
#endif

#ifdef _SUPPORT_CSB_TRANSMITTER_
    BT_CSA4_SYNC_TRAIN_TX_UNIT_S *stp = &bt_3dd_var.stp_tx_param;
    BT_CSA4_BEACON_TX_UINT_S *pcsb = &bt_3dd_var.csb_tx_param;

    if((pcsb->enable)||(stp->enable))
    {
        return FALSE;
    }
#endif

#ifdef _MODI_LPS_AFTER_RTL8821B_TC_
    if(g_efuse_lps_setting_2.adjust_lps_scan_interval)
    {
        if((sleep_mode_param.lps_period_state_bitmap !=0) &&
           (sleep_mode_param.lps_period_interval < (g_efuse_lps_setting_2.scan_lps_min_period<<4)))
        {
            return FALSE;
        }
    }

    if (IS_BT40)
    {
        if(g_efuse_lps_setting_2.le_lps_enable == 0)
        {
            if(ll_manager.adv_unit.enable || ll_manager.scan_unit.enable ||
               ll_manager.initiator_unit.enable || ll_manager.conn_unit.enable
#ifdef CONFIG_TV_POWERON_LPS
               || tv_poweron.scan.enable
#endif
               )
            {
                return FALSE;
            }
        }

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_LE_
        if(g_efuse_rsvd_2.le_link_lps_enable == 0)
        {
            if(ll_manager.initiator_unit.enable || ll_manager.conn_unit.enable)
            {
                return FALSE;
            }
        }

        if(g_efuse_rsvd_2.enable_deep_lps_mode)
        {
            if( ll_manager.initiator_unit.enable)
            {
#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
                sleep_mode_param.run_dlps_flow = 0;
#endif
            }
        }
#endif

    }
#endif

    if (lmp_self_device_data.scan_enable != 0)
    {
        if(!( g_efuse_lps_setting_2.timer2_mode_sel &&
             g_efuse_lps_setting_2.timer2_scan_en))
        {
            return FALSE;
        }

        if(lmp_self_device_data.scan_enable == 1)
        {
            return FALSE;
        }

        if ((lmp_self_device_data.scan_enable&1)&&
            (lmp_self_device_data.inquiry_scan_interval < otp_str_data.bt_deep_sleep_mode_threshold))
        {
            return FALSE;
        }

        if ((lmp_self_device_data.scan_enable&2)&&
            (lmp_self_device_data.page_scan_interval < otp_str_data.bt_deep_sleep_mode_threshold))
        {
            return FALSE;
        }
    }

#ifdef LE_MODE_EN
    if (IS_BT40)
    {
        if((ll_manager.adv_unit.enable != 0)||(ll_manager.scan_unit.enable != 0)
#ifdef CONFIG_TV_POWERON_LPS
                || tv_poweron.scan.enable
#endif
                )
        {
            if(!( g_efuse_lps_setting_2.timer2_mode_sel &&
                g_efuse_lps_setting_2.timer2_scan_en))
            {
                return FALSE;
            }

            if ((ll_manager.adv_unit.enable)&&
                (ll_manager.adv_unit.interval < otp_str_data.bt_deep_sleep_mode_threshold))
            {//RT_BT_LOG(GRAY, CCH_DBG_106, 2,ll_manager.adv_unit.interval,otp_str_data.bt_deep_sleep_mode_threshold );
                return FALSE;
            }

            if ((ll_manager.scan_unit.enable)&&
                (ll_manager.scan_unit.interval < otp_str_data.bt_deep_sleep_mode_threshold))
            {//RT_BT_LOG(GRAY, CCH_DBG_106, 2,ll_manager.scan_unit.interval,otp_str_data.bt_deep_sleep_mode_threshold );
                return FALSE;
            }
#ifdef CONFIG_TV_POWERON_LPS
            if (tv_poweron.scan.enable
                    && (tv_poweron.scan.interval < otp_str_data.bt_deep_sleep_mode_threshold))
            {//RT_BT_LOG(GRAY, CCH_DBG_106, 2,ll_manager.scan_unit.interval,otp_str_data.bt_deep_sleep_mode_threshold );
                return FALSE;
            }
#endif
        }
    }
#endif

    if (lmp_self_device_data.lc_cur_dev_state != LC_CUR_STATE_IDLE)
    {
        return FALSE;
    }

    if(lmp_self_device_data.number_of_hlc > 1)
    {
        return FALSE;
    }

#ifdef COMPILE_PARK_MODE
    if (lmp_self_device_data.number_of_parked_dev != 0)
    {
        return FALSE;
    }
#endif

#ifdef COMPILE_HOLD_MODE
    if (lmp_self_device_data.number_of_connections_in_hold_mode != 0)
    {
        return FALSE;
    }
#endif

#ifdef ENABLE_SCO
    if(lmp_self_device_data.total_no_of_sco_conn > 0)
    {
        return FALSE;
    }
#endif

#ifdef COMPILE_ESCO
    if(lmp_self_device_data.number_of_esco_connections > 0)
    {
        return FALSE;
    }
#endif

#ifdef LE_MODE_EN
    if (IS_BT40)
    {
        UINT8 link_count = 0;

        link_count = lmp_self_device_data.number_of_hlc;

        if(ll_manager.initiator_unit.enable)
        {
            link_count += 1;
        }

        if(ll_manager.conn_unit.enable)
        {
            link_count += ll_manager.conn_unit.connection_cnts ;
        }

        if(link_count >1)
        {
            return FALSE;
        }
    }
#endif


    if(lmp_self_device_data.number_of_hlc > 0)
    {
        // Legacy Cannot enter LPS with any Link not in Sniff mode
        UINT8 ce_index;
        LMP_CONNECTION_ENTITY *ce_ptr;

        for(ce_index = 0; ce_index < LMP_MAX_CE_DATABASE_ENTRIES; ce_index++)
        {
            ce_ptr = &lmp_connection_entity[ce_index];

            if( (ce_ptr->entity_status == ASSIGNED) &&
                ( (ce_ptr->ce_status != LMP_SNIFF_MODE)||
                  ((ce_ptr->ce_status == LMP_SNIFF_MODE)&&(ce_ptr->sniff_interval<=otp_str_data.bt_deep_sleep_mode_threshold))
                  ) )
            {
                return FALSE;
            }
#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
            // Store Last Eeqn
            if(ce_ptr->entity_status == ASSIGNED)
            {
                LUT_EXTENSION_TABLE *ex_lut;

                UCHAR lut_index = lc_get_lut_index_from_ce_index(ce_index);

                if( lut_index == 0)
                {
                    return FALSE;
                }

                ex_lut = &lut_ex_table[lut_index];
                UINT16 reg_value;
                reg_value = BB_read_baseband_register(ex_lut->upper_lut_address);
                ce_ptr->dlps_last_seqn = ((reg_value&BIT1)>>1);
            }
#endif
        }
    }

#ifdef LE_MODE_EN
    if (IS_BT40)
    {
        if(ll_manager.initiator_unit.enable)
        {
            if(ll_manager.initiator_unit.scan_interval <= otp_str_data.bt_deep_sleep_mode_threshold )
            {
                return FALSE;
            }
        }

        if(ll_manager.conn_unit.enable)
        {
            UINT8 cur_entry;
            LL_CONN_HANDLE_UNIT *phandle;

            for(cur_entry = 0; cur_entry < LL_MAX_CONNECTION_UNITS; cur_entry++)
            {
                phandle = &ll_manager.conn_unit.handle[cur_entry];
                if((phandle->connected)&&
                   (phandle->ce_interval < (otp_str_data.bt_deep_sleep_mode_threshold>>1) ))
                {
                    return FALSE;
                }
            }

            UINT8 entry;
            //[for Channel map update]
            entry = ll_manager.conn_unit.chm_updt_entry;
            if ((entry != LL_MAX_CONNECTION_UNITS)&&
                (ll_manager.conn_unit.handle[entry].connected))
            {
                return FALSE;
            }
            //[for Connection update]
            entry = ll_manager.conn_unit.conn_updt_entry;
            if ((entry  != LL_MAX_CONNECTION_UNITS)&&
                (ll_manager.conn_unit.handle[entry].connected))
            {
                return FALSE;
            }
            
        }


    }
#endif
    return TRUE;
}

UINT8  lc_check_lps_for_idle(UCHAR with_scan)
{
#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8821_RCP_
    UCHAR return_status = 0;
    if (rcp_lc_check_lps_for_idle != NULL)
    {
        if ( rcp_lc_check_lps_for_idle((void*)(&return_status), with_scan) )
        {
            return return_status;
        }
    }
#endif
#endif

#ifdef _MODI_LPS_AFTER_RTL8821B_TC_
    if (IS_USE_FOR_BQB ||
        (lmp_self_device_data.test_mode == HCI_DUT_LOOPBACK_MODE) ||
        (lmp_self_device_data.test_mode == HCI_DEVICE_UNDER_TEST_MODE) ||
        (lmp_self_device_data.host_enable_test_mode))
    {
        /* no enter lps mode in BQB or DUT test mode */
        return FALSE;
    }
#endif

#ifdef _SUPPORT_CSB_TRANSMITTER_
    BT_CSA4_SYNC_TRAIN_TX_UNIT_S *stp = &bt_3dd_var.stp_tx_param;
    BT_CSA4_BEACON_TX_UINT_S *pcsb = &bt_3dd_var.csb_tx_param;

    if((pcsb->enable)||(stp->enable))
    {
        return FALSE;
    }
#endif

#ifdef _MODI_LPS_AFTER_RTL8821B_TC_
    if(g_efuse_lps_setting_2.adjust_lps_scan_interval)
    {
        if((sleep_mode_param.lps_period_state_bitmap !=0) &&
           (sleep_mode_param.lps_period_interval < (g_efuse_lps_setting_2.scan_lps_min_period<<4)))
        {
            return FALSE;
        }
    }

    if (IS_BT40)
    {
        if(g_efuse_lps_setting_2.le_lps_enable == 0)
        {
            if(ll_manager.adv_unit.enable || ll_manager.scan_unit.enable ||
               ll_manager.initiator_unit.enable || ll_manager.conn_unit.enable
#ifdef CONFIG_TV_POWERON_LPS
               || tv_poweron.scan.enable
#endif
               )
            {
                return FALSE;
            }
        }
    }
#endif

    if (lmp_self_device_data.scan_enable != 0)
    {

        if(lmp_self_device_data.scan_enable == 1)
        {
            return FALSE;
        }

        if ((lmp_self_device_data.scan_enable&1)&&
            (lmp_self_device_data.inquiry_scan_interval < otp_str_data.bt_deep_sleep_mode_threshold))
        {
            return FALSE;
        }

        if ((lmp_self_device_data.scan_enable&2)&&
            (lmp_self_device_data.page_scan_interval < otp_str_data.bt_deep_sleep_mode_threshold))
        {
            return FALSE;
        }
    }

#ifdef LE_MODE_EN
    if (IS_BT40)
    {
        if ((ll_manager.adv_unit.enable)&&
            (ll_manager.adv_unit.interval < otp_str_data.bt_deep_sleep_mode_threshold))
        {//RT_BT_LOG(GRAY, CCH_DBG_106, 2,ll_manager.adv_unit.interval,otp_str_data.bt_deep_sleep_mode_threshold );
            return FALSE;
        }

        if ((ll_manager.scan_unit.enable)&&
            (ll_manager.scan_unit.interval < otp_str_data.bt_deep_sleep_mode_threshold))
        {//RT_BT_LOG(GRAY, CCH_DBG_106, 2,ll_manager.scan_unit.interval,otp_str_data.bt_deep_sleep_mode_threshold );
            return FALSE;
        }

#ifdef CONFIG_TV_POWERON_LPS
        if ((tv_poweron.scan.enable)&&
            (tv_poweron.scan.interval < otp_str_data.bt_deep_sleep_mode_threshold))
        {//RT_BT_LOG(GRAY, CCH_DBG_106, 2,ll_manager.scan_unit.interval,otp_str_data.bt_deep_sleep_mode_threshold );
            return FALSE;
        }
#endif

        if ((ll_manager.adv_unit.enable != 0)&&
            (ll_manager.adv_unit.hci_pdu_type == LL_HCI_ADV_TYPE_ADV_DIRECT_IND))
        {
            return FALSE;
        }
    }
#endif


    if ((lmp_self_device_data.number_of_hlc != 0) ||
#ifdef COMPILE_PARK_MODE
        (lmp_self_device_data.number_of_parked_dev != 0) ||
#endif
        (lmp_self_device_data.lc_cur_dev_state != LC_CUR_STATE_IDLE))
    {
        return FALSE;
    }


    UINT8 ce_index;
    for(ce_index = 0; ce_index < LMP_MAX_CE_DATABASE_ENTRIES; ce_index++)
    {
        if((lmp_connection_entity[ce_index].ce_status == LMP_PAGE_SCAN)&&(lmp_connection_entity[ce_index].entity_status == ASSIGNED))
        {
            LPS_CCH_LOG(RED, YL_DBG_HEX_3, 3,ce_index, lmp_connection_entity[ce_index].ce_status, lmp_connection_entity[ce_index].entity_status);
            return FALSE;
        }
    }


    if (lmp_self_device_data.scan_enable != 0)
    {
        if(with_scan)
        {
            if(!( g_efuse_lps_setting_2.timer2_mode_sel &&g_efuse_lps_setting_2.timer2_scan_en))
            {
                return FALSE;
            }
        }else
        {
            return FALSE;
        }
    }

#ifdef LE_MODE_EN
    if (IS_BT40)
    {
        if (ll_manager.initiator_unit.enable || ll_manager.conn_unit.enable)
        {
            return FALSE;
        }

        if ((ll_manager.adv_unit.enable != 0)||(ll_manager.scan_unit.enable != 0)
#ifdef CONFIG_TV_POWERON_LPS
                || tv_poweron.scan.enable
#endif
                )
        {
            if(with_scan)
            {
                if(!( g_efuse_lps_setting_2.timer2_mode_sel &&g_efuse_lps_setting_2.timer2_scan_en))
                {
                    return FALSE;
                }
            }else
            {
                return FALSE;
            }
        }

    }
#endif

    return TRUE;
}

void lc_check_lps_for_resume(void)
{
#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8821_RCP_
    if (rcp_lc_check_lps_for_resume != NULL)
    {
        rcp_lc_check_lps_for_resume();
    }
#endif
#endif

    sleep_mode_param.lps_procedure_start = 0;
    sleep_mode_param.lps_lpm_end_flag = 0;

    lc_check_and_enable_scans_in_scatternet();

    if( g_efuse_lps_setting_4.lps_use_intr == 0)
    {
#ifdef LE_MODE_EN
        if (IS_BT40)
        {
            // need to resume LE adv and scan
            if(ll_manager.adv_unit.enable == 1)
            {
#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_LE_
                ll_driver_enable_advertising(FALSE);
#else
                ll_driver_enable_advertising();
#endif
            }
            if(ll_manager.scan_unit.enable == 1)
            {
#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_LE_
                ll_driver_enable_scanning(FALSE);
#else
                ll_driver_enable_scanning();
#endif
            }
#ifdef CONFIG_TV_POWERON_LPS
            if (tv_poweron.scan.enable)
            {
                tv_poweron_enable_le_scan(&tv_poweron.scan, FALSE);
            }
#endif
            // RT_BT_LOG(GRAY, CCH_DBG_106, 2,ll_manager.adv_unit.enable, ll_manager.scan_unit.enable);
        }
#endif
        // cause there are many reasons to change adv_enable, need to update lps parameter
        lmp_update_lps_para();
    }

#ifdef _LPS_STOP_REG_TIMER_
    lmp_start_regular_sw_timers();
#endif


}

UINT8  lc_check_lps_task_queue()
{
    // return API_SUCCESS(0) for ok for this lps procedure
    // return API_RETRY(0x0f) for re-call this lps task
    // return API_FAILURE(0xff) for give up this lps task
#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8821_RCP_
    UCHAR return_status = 0;
    if (rcp_lc_check_lps_task_queue != NULL)
    {
        if ( rcp_lc_check_lps_task_queue((void*)(&return_status)))
        {
            return return_status;
        }
    }
#endif
#endif

    UINT16 check_id;

    for(check_id=0; check_id<OS_TOTAL_TASKS; check_id++)
    {
        if( ( (1<<check_id) & g_efuse_lps_setting_1.lps_task_mask ) && ( queue_mgr.queue_array[ check_id ].q_length != 0) )
        {
            RT_BT_LOG(RED, LPS_LOG_024, 2, check_id, queue_mgr.queue_array[ check_id ].q_length);
            return API_RETRY;
        }
    }

#if 1 // reduce same logic process
    if (dma_man.evt_available_num != (EVT_RX_DES_NUM - 1))
    {
        return API_FAILURE;
    }

    if (dma_man.acl_available_num != (ACL_RX_DES_NUM - 1))
    {
        return API_FAILURE;
    }

    if (dma_man.sco_available_num != (SCO_RX_DES_NUM - 1))
    {
        return API_FAILURE;
    }
#else
    for (check_id = 0; check_id < ACL_RX_DES_NUM; check_id++)
    {
        if(dma_acl_rx_des[check_id].rx_des_dw1.own != 0)
        {
            return API_FAILURE;
        }
    }
    for (check_id = 0; check_id < SCO_RX_DES_NUM; check_id++)
    {
        if(dma_sco_rx_des[check_id].rx_des_dw1.own != 0)
        {
            return API_FAILURE;
        }
    }
    for (check_id = 0; check_id < EVT_RX_DES_NUM; check_id++)
    {
        if(dma_evt_rx_des[check_id].rx_des_dw1.own != 0)
        {
            return API_FAILURE;
        }
    }
#endif

    LC_PICONET_SCHEDULER *piconet_schd;
#ifdef _MODI_LPS_AFTER_RTL8703B_
    LC_SCHEDULED_PKT *schd;
#endif
    for (check_id = 0; check_id < LMP_MAX_PICONETS_SUPPORTED; check_id++)
    {
        piconet_schd = &lc_piconet_scheduler[check_id];
#ifdef _MODI_LPS_AFTER_RTL8703B_
        schd = &piconet_schd->lc_scheduled_pkt_info[piconet_schd->rptr];

        if((piconet_schd->lc_allowed_pkt_cnt != LC_MAX_SCH_INFO)
           &&(lmp_self_device_data.lc_no_of_connections[check_id]!=0))
        {
            RT_BT_LOG(YELLOW, YL_DBG_HEX_2, 2,check_id ,piconet_schd->lc_allowed_pkt_cnt);
            return API_FAILURE;
        }

        if (schd->tx_status != LC_TX_IDLE)
        {
            RT_BT_LOG(YELLOW, YL_DBG_HEX_3, 2,check_id ,piconet_schd->lc_allowed_pkt_cnt, schd->tx_status);
            return API_FAILURE;
        }
#else
        if((piconet_schd->lc_allowed_pkt_cnt == 0)
           &&(lmp_self_device_data.lc_no_of_connections[check_id]!=0))
        {
            RT_BT_LOG(YELLOW, CCH_DBG_106, 2,check_id ,piconet_schd->lc_allowed_pkt_cnt);
            return API_FAILURE;
        }
#endif
    }


    return API_SUCCESS;
}

UINT8 lc_lps_double_check_dsm_cond(UCHAR lps_mode)
{  // return API_FAILURE will NOT enter DSM
#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8821_RCP_
    UCHAR return_status = 0;
    if (rcp_lc_lps_double_check_dsm_cond != NULL)
    {
        if ( rcp_lc_lps_double_check_dsm_cond((void*)(&return_status), lps_mode) )
        {
            return return_status;
        }
    }
#endif
#endif
#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
    sleep_mode_param.run_dlps_flow = g_efuse_rsvd_2.enable_deep_lps_mode;
#endif
    if(!(g_efuse_lps_setting_2.timer2_lps_on))
    {
        return API_FAILURE;
    }

#ifdef LPS_NEVER_ENTER_LPS_WHEN_USB_ACTIVE
    if(g_efuse_lps_setting_5.disable_never_enter_lps_when_usb_active == 0)
    {
        if (g_fun_interface_info.b.bt_interface == USB_INTERFACE)
        {
            if (usb_read_suspend_status() == 0)
            {
                return API_FAILURE;
            }
        }
    }
#endif

    if( ( lps_mode == LPS_SNIFF_SM ) || (lps_mode == LPS_SNIFF_DSM) )
    {

        if( !lc_check_lps_for_link( 0, 0, 1))
        {
            return API_FAILURE;
        }

    }else if( lps_mode == LPS_TIMER2_WITH_SCAN )
    {
        if( !lc_check_lps_for_idle(1))
        {
            return API_FAILURE;
        }
    }else if( lps_mode == LPS_TIMER2_WO_SCAN )
    {
        if( !lc_check_lps_for_idle(0))
        {
            return API_FAILURE;
        }
    }

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
    // Check if FW exist
    if(g_efuse_rsvd_2.enable_deep_lps_mode)
    {
#ifdef _SUPPORT_BT_CTRL_FM_
        if( fm_ctrl.enable || fm_ctrl.hw_enable)
        {
            sleep_mode_param.run_dlps_flow = 0;
        }
#endif

    }

#endif

    return API_SUCCESS;
}


UINT8 lc_lps_double_check_dsm_cond_after_clr_sts(UCHAR lps_mode)
{  // return API_FAILURE will NOT enter DSM
#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8821_RCP_
    UCHAR return_status = 0;
    if (rcp_lc_lps_double_check_dsm_cond_after_clr_sts != NULL)
    {
        if ( rcp_lc_lps_double_check_dsm_cond_after_clr_sts((void*)(&return_status), lps_mode) )
        {
            return return_status;
        }
    }
#endif
#endif
    return API_SUCCESS;
}


UINT8 check_host_wake_bt()
{  // return 1, can not enter lps mode
   // should refer to get_gpio_host_wake_bt(), suspend, ...
    return API_FAILURE;
}

void lc_program_lps_upd_val(UCHAR lps_mode, UINT32 *wakeup_instant, UINT32 *scan_interval)
{
#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8821_RCP_
    if (rcp_lc_program_lps_upd_val != NULL)
    {
        if(rcp_lc_program_lps_upd_val((void *)(&wakeup_instant), &scan_interval, &lps_mode))
        {
            return;
        }
    }
#endif
#endif

    UINT8 osc_delay;
    UINT32 wakeup_instant_val = *wakeup_instant;
    /* Check if there is only one active connection.
    Otherwise, make sure that all the connections are parked. */

    osc_delay = otp_str_data.bw_rf_osc_delay;

    // TODO: wakeup_instant should also take tolerance into account ?? seems not
    if( lps_mode == LPS_SNIFF_SM)
    { // sleep mode
        UINT16 sm_guard_interval;
        if (g_efuse_lps_setting_1.sm_guard_interval == 3)
        {
            sm_guard_interval = 16;
        }
        else
        {
            sm_guard_interval = 2 + g_efuse_lps_setting_1.sm_guard_interval;
        }
        (*wakeup_instant) = (wakeup_instant_val - sm_guard_interval);
    }
    else if( lps_mode == LPS_SNIFF_DSM)
    { // deep sleep mode
        UINT16 dsm_guard_interval;
        if (g_efuse_lps_setting_1.dsm_guard_interval == 7)
        {
            dsm_guard_interval = 16;
        }
        else
        {
            dsm_guard_interval = 2 + g_efuse_lps_setting_1.dsm_guard_interval;
        }

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
        if(sleep_mode_param.run_dlps_flow)
        {
            dsm_guard_interval = dsm_guard_interval + g_efuse_rsvd_2.dlps_guard_time;
        }
#endif

        (*wakeup_instant) = (wakeup_instant_val - osc_delay - dsm_guard_interval);

    }
    else if( lps_mode == LPS_TIMER2_WITH_SCAN)
    { // timer2 with scan
        if(g_efuse_lps_setting_4.lps_use_state)
        {
#ifdef _MODI_LPS_AFTER_RTL8703B_
            (*scan_interval) = (UINT32)sleep_mode_param.lps_period_interval - osc_delay;
#else
            (*scan_interval) = (UINT32)sleep_mode_param.lps_period_interval;
#endif
        }
    }
    else if( lps_mode == LPS_TIMER2_WO_SCAN)
    { // timer2 without scan
#ifdef _DLPS_SIMU_TEST_
        (*scan_interval) = 0x50;
#else
        (*scan_interval) = 0xFF00;
#endif
    }

    if( (*scan_interval) == 0 )
    {
        (*scan_interval) = 0xFF00;
    }

}

UINT8 lc_program_lps_check_cal(UCHAR lps_mode)
{
    // return API_SUCCESS(0) for ok for this lps procedure
    // return API_RETRY(0x0f) for re-call this lps task
    // return API_FAILURE(0xff) for give up this lps task

#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8821_RCP_
    if (rcp_lc_program_lps_check_cal != NULL)
    {
        UINT8 return_status;
        if (rcp_lc_program_lps_check_cal((void *)&return_status, lps_mode))
        {
            return return_status;
        }
    }
#endif
#endif

    UINT8 cal_lock;

    cal_lock = bton_32k_cal_chk_lock();


    if((cal_lock == 1)||(( lps_mode >= LPS_TIMER2_WITH_SCAN )&&(cal_lock == 2)))
    {
        sleep_mode_param.lps_32k_cal_count = 0;
        return API_SUCCESS;
    }


    if(cal_lock != 0)
    {
        bton_32k_cal_en();
    }


    sleep_mode_param.lps_32k_cal_count ++;
    LPS_CCH_LOG(BLUE, CCH_DBG_144, 3, lps_mode, sleep_mode_param.lps_32k_cal_count, cal_lock);

#ifdef _MODI_LPS_AFTER_RTL8703B_
    if( sleep_mode_param.lps_32k_cal_count >= sleep_mode_param.lps_32k_cal_max)
#else
    if( sleep_mode_param.lps_32k_cal_count >= LPS_32K_UNLOCK_MAX_TIME)
#endif
    {
        // add for resume lps
        RT_BT_LOG(RED, CCH_DBG_144, 3, lps_mode, sleep_mode_param.lps_32k_cal_count, cal_lock);
        sleep_mode_param.lps_32k_cal_count = 0;
        return API_FAILURE;
    }

    return API_RETRY;


}

void lc_program_lps_upd_par(UCHAR lps_mode)
{
#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8821_RCP_
    if (rcp_lc_program_lps_upd_par != NULL)
    {
        rcp_lc_program_lps_upd_par((void *)&lps_mode);
        return;
    }
#endif
#endif

#ifdef LPS_NEW_CAL_MODE

    UCHAR lps_period_occur = 0;

    if( (lmp_self_device_data.scan_enable&BIT1) ||
        (ll_manager.adv_unit.enable) ||
        (ll_manager.scan_unit.enable) )
    {
        lps_period_occur = 1;
    }

    if( lps_period_occur &&
        ((lmp_self_device_data.number_of_hlc == 1) ||
        ((ll_manager.conn_unit.enable)&&(ll_manager.conn_unit.connection_cnts == 1))))
#else
    if( lmp_self_device_data.scan_enable && (lmp_self_device_data.number_of_hlc != 0))
#endif
    {
        UINT32 temp_u32;
        temp_u32 = (UINT32) lmp_self_device_data.lps_sniff_scan_count +sleep_mode_param.lps_link_interval;

        if( temp_u32 < sleep_mode_param.lps_period_interval)
        {
           lmp_self_device_data.lps_sniff_scan_count = temp_u32;
           lmp_self_device_data.lps_sniff_scan_en = 0;
        }else
        {
            lmp_self_device_data.lps_sniff_scan_count = 0;
            lmp_self_device_data.lps_sniff_scan_en = 1;
        }
    }else
    {
        lmp_self_device_data.lps_sniff_scan_count = 0;
        lmp_self_device_data.lps_sniff_scan_en = 0;
    }

    sleep_mode_param.lps_lpm_lps_mode = 0xFF;   // 20120310

#ifdef _LPS_LOG_EN_1
        RT_BT_LOG(GRAY, YL_DBG_HEX_8, 8, lmp_self_device_data.lps_sniff_scan_en, lmp_self_device_data.lps_sniff_scan_count,
                                         sleep_mode_param.lps_link_interval, sleep_mode_param.lps_period_interval,
                                         lps_period_occur, lmp_self_device_data.scan_enable,
                                         ll_manager.adv_unit.enable, (ll_manager.scan_unit.enable));
#endif

}

UINT8 lc_program_lps_check_diff(UCHAR lps_mode, UINT32 wakeup_instant, UINT32 clock)
{
    UINT32 diff;
    diff = (wakeup_instant | BIT27) - clock;
    diff = diff & ~BIT27;

    // TODO: What is the MIN for 8723???
    // TODO: configured by Efuse 2 ~ 5

    /* Note: original Mindtree did NOT check this condidtion */

    if( lps_mode == LPS_SNIFF_SM )
    { // sleep mode

#ifdef _YL_LPS
#endif
#ifndef _YL_LPS
#define MINIMUM_SLEEP_INTERVAL 2  /* mindtree version */
        if(diff <= MINIMUM_SLEEP_INTERVAL || diff > 0xFFFF)
#else
        UINT16 min_sm_interval = (2 << (g_efuse_lps_setting_1.min_sm_interval+1));
        if(diff <= (min_sm_interval) || diff > 0xFFFF)
#endif
        {
            return API_FAILURE;
        }
    }
    else
    { // deep sleep mode
        UINT16 min_dsm_interval = (2 << (g_efuse_lps_setting_1.min_dsm_interval+1));
        if(diff <= (min_dsm_interval) || diff > 0xFFFF)
        {
            lps_gpio_one_pull_low(LPS_GPIO_PGM_SNIFF_DSM);
#ifdef _LPS_LOG_EN_
            RT_BT_LOG(GRAY, LPS_LOG_047, 3, clock, wakeup_instant, lps_mode);
            LPS_CCH_LOG(RED, LPS_LOG_007, 2, diff, min_dsm_interval);
#endif
            return API_FAILURE;
        }
    }
    return API_SUCCESS;
}

void lc_program_lps_execute_lps(UCHAR lps_mode, UINT32 wakeup_instant, UCHAR piconet_id)
{

#ifdef _LPS_LOG_EN_1
    UINT32 nat_clock;
    nat_clock = (BB_read_baseband_register(NATIVE_CLOCK2_REGISTER) << 16) |
                 BB_read_baseband_register(NATIVE_CLOCK1_REGISTER);
    LPS_CCH_LOG(GRAY, LPS_LOG_045, 2, nat_clock>>1, wakeup_instant);
#endif

    if( lps_mode == LPS_SNIFF_SM)
    { // sleep mode
        sleep_mode_param.bb_sm_wakeup_inst = wakeup_instant;
        BB_write_baseband_register(WAKEUP_INSTANT_REGISTER,
                                   (UINT16)wakeup_instant);

        BB_write_baseband_register(POWER_CONTROL_REGISTER,
                                   (lc_power_ctr_config | BB_PROGRAM_SM_MODE));
        sleep_mode_param.bb_sm_sts = BB_SLEEP_MODE;
    }
    else if(( lps_mode == LPS_SNIFF_DSM ) || ( lps_mode == LPS_TIMER2_WITH_SCAN) ||
            ( lps_mode == LPS_WITH_WAKEUP_INSTANCE))
    { // deep sleep mode or timer2 with scan
        sleep_mode_param.bb_sm_wakeup_inst = wakeup_instant;
        sleep_mode_param.bb_sm_piconet_id = piconet_id;
#ifdef _ENABLE_BTON_POWER_SAVING_
        execute_lps_mode_procedure_6128(1, wakeup_instant);
#endif
        sleep_mode_param.bb_sm_sts = BB_DEEP_SLEEP_MODE;
    }

    if(( lps_mode == LPS_TIMER2_WO_SCAN)||
       (lps_mode == LPS_WO_WAKEUP_INSTANCE))
    {
        if( g_efuse_lps_setting_2.timer2_no_scan_wakeup_en )
        { // deep sleep mode or timer2 with scan
            sleep_mode_param.bb_sm_wakeup_inst = wakeup_instant;
            sleep_mode_param.bb_sm_piconet_id = piconet_id;
#ifdef _ENABLE_BTON_POWER_SAVING_
            execute_lps_mode_procedure_6128(1, wakeup_instant);
#endif
            sleep_mode_param.bb_sm_sts = BB_DEEP_SLEEP_MODE;
        }
        else
        { // timer2 without scan
#ifdef _ENABLE_BTON_POWER_SAVING_
            execute_lps_mode_procedure_6128(0, 0);
#endif
            sleep_mode_param.bb_sm_sts = BB_DEEP_SLEEP_MODE;
        }
    }
}

#endif

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
void dlps_save_cpu_to_partial_on_then_enter_deep_lps_mode(void)
{

#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8703B_RCP_
    if (rcp_dlps_save_cpu_to_partial_on_then_enter_deep_lps_mode != NULL)
    {
        rcp_dlps_save_cpu_to_partial_on_then_enter_deep_lps_mode();
        return;
    }
#endif
#endif

#ifdef _ENABLE_RETENTION_FLOW_FOR_DLPS_
    /* set the signature bit before enter dlps mode (austin) */
    SIGN_ENTER_DLPS_MODE_SIGNATURE;
    SIGN_ENTER_DLPS_MODE_FLAG;  /* for debug use */
#endif

#ifdef _YL_LPS_COMBO_PLATFORM
    BTON_POW_CTRL_REG_S_TYPE pow_ctrl;
    //set BTON LPS mode request
    pow_ctrl.d32 = VENDOR_READ(BTON_POW_CTRL_REG);
#ifdef LPS_MODIFY_POW_CTRL_W1C
    bton_pow_ctrl_avoid_int_sts_w1c(&(pow_ctrl));
#endif
    pow_ctrl.b.lps_mode_en = 1;
    pow_ctrl.b.wifi_onoff_intr_sts = 0; /* to avoid falsely W1C wifi_onoff_intr_sts */
    VENDOR_WRITE(BTON_POW_CTRL_REG,pow_ctrl.d32);
#endif

    // Enter Deep LPS mode here! Right Now!!
}
#endif /* end of #ifdef _CCH_RETENTION_FLOW_FOR_DLPS_ */

#if defined(_CCH_LPS_) && defined(_YL_LPS)

UINT8 lc_program_lps_mode_stage1(UCHAR lps_mode, UINT32 wakeup_instant, UCHAR piconet_id)
{
    UINT32 clock;
    UINT32 scan_interval = 0;


#ifdef _LPS_WITH_MULTI_LINK_
    UINT32 diff;
    UINT8 wakeup_index = 0xFF;
    UINT8 check_cal_lock_flag = 1;
#endif

#ifdef _MODI_LPS_AFTER_RTL8821B_TC_
    lc_program_lps_upd_par(lps_mode);
#endif

    if(!(g_efuse_lps_setting_2.timer2_lps_on))
    {
        return API_FAILURE;
    }

    if(g_efuse_lps_setting_4.lps_chk_g_host_wake_bt != 0)
    {
        if(check_host_wake_bt())
        {  // if g_host_wake_bt == 1, can not enter lps mode
            return API_FAILURE;
        }
    }

    if(lmp_self_device_data.scan_enable == 1)
    {
        return API_FAILURE;
    }

    if(( ( lps_mode == LPS_SNIFF_SM ) || (lps_mode == LPS_SNIFF_DSM) )
       &&(sleep_mode_param.lps_lpm_end_flag == 0))
    {
        return API_FAILURE;
    }


    DEF_CRITICAL_SECTION_STORAGE;

    UINT8 retry_lps = 0;
    retry_lps = lc_check_lps_task_queue();
    if(retry_lps == API_RETRY)
    {
#ifdef _MODI_LPS_AFTER_RTL8703B_
        lps_period_state_enter_lps(lps_mode);
#else
        lc_post_lps_mode_signal(lps_mode,wakeup_instant, piconet_id);
#endif
        return API_RETRY;
    }
    else if(retry_lps == API_FAILURE)
    {
        return API_FAILURE;
    }

    if( lc_lps_double_check_dsm_cond(lps_mode) == API_FAILURE)
    {
        return API_FAILURE;
    }


    if(g_efuse_lps_setting_4.lps_use_intr == 0)
    {
        //RT_BT_LOG(GRAY, LPS_LOG_004, 2, wakeup_instant, piconet_id);
        // TODO: Need also check LE status !!!
        // TODO: Should not go into LPS mode when there is important tasks or HCI data to send?
        // TODO: need also consider HCI traffic status
        // TODO: need also review the Timer2 sleep conditions !!

        lc_program_lps_upd_val(lps_mode, &wakeup_instant, &scan_interval);
    }

#ifdef _YL_LPS_COMBO_PLATFORM
#ifdef LPS_MODIFY_POW_CTRL_W1C
    bton_clr_wakeup_sts();
#endif
#endif

    MINT_OS_ENTER_CRITICAL();

    // yilinli, 2013/08/20, added for more flexible escape check //
    // e.g. clr sts ==> check GPIO13 ==> enter LPS //
    if (lc_lps_double_check_dsm_cond_after_clr_sts(lps_mode) == API_FAILURE)
    {
        MINT_OS_EXIT_CRITICAL();
        return API_FAILURE;
    }

    if(g_lps_timer_counter >0)
    {
        MINT_OS_EXIT_CRITICAL();
        return API_FAILURE;
    }

#ifdef _LPS_BY_NATIVE_CLK_
    //8821  LPS Already Use Native to Sleep by HW
    clock = BB_read_native_clock();
#else
    UINT16 reg;
    reg = BB_read_baseband_register(VOICE_SETTING_REGISTER);

    if( (lmp_self_device_data.number_of_hlc >0) &&
        (lc_sca_manager.pnet[piconet_id].active == 1) &&
        (lc_sca_manager.pnet[piconet_id].master == 0))
    {  // Only Legacy Sniff Slave use piconet clock to sleep
        reg &= (~BIT13);
        BB_write_baseband_register(VOICE_SETTING_REGISTER, reg);

        lc_get_clock_in_scatternet(&clock, piconet_id);
    }else
    {
        reg |= BIT13;
        BB_write_baseband_register(VOICE_SETTING_REGISTER, reg);

        clock = BB_read_native_clock();
    }
#endif
    clock = clock >> 1;

#ifndef _REDUCE_LPS_AFTER_RTL8703B_
    if(g_efuse_lps_setting_4.lps_use_intr)
    {
#ifdef _LPS_WITH_MULTI_LINK_
        UINT16 osc_delay;
        check_cal_lock_flag = 1 ;

        if( lps_mode == LPS_WITH_WAKEUP_INSTANCE)
        {
            if( !lc_check_lps_for_link( 0, 0, 1))
            {
                MINT_OS_EXIT_CRITICAL();
                RT_BT_LOG(RED, CCH_DBG_106, 2, wakeup_instant, clock);
                return API_FAILURE;
            }

            UINT32 instance_diff = 0;
            diff = 0xFFFF;
            UINT8 table_index = 0;
            UINT32 scan_diff = 0xFFFF;

            for( table_index = 0; table_index < LPS_TABLE_NUM; table_index ++)
            {
                if( table_index < LMP_MAX_CE_DATABASE_ENTRIES )
                {
                    if(!((lmp_connection_entity[table_index].entity_status == ASSIGNED)&&
                          (lmp_connection_entity[table_index].ce_status == LMP_SNIFF_MODE)&&
                          (lmp_connection_entity[table_index].sniff_interval > otp_str_data.bt_deep_sleep_mode_threshold )))
                    {
                        sleep_mode_param.lps_table_ing_bitmap &= (UINT32)(~(1<<table_index));
                        continue;
                    }

                }else if( table_index < (LMP_MAX_CE_DATABASE_ENTRIES + LL_MAX_CONNECTION_UNITS) )
                {
                    continue;
                }
#ifdef _WA_8821_SCAN_CNT_STOP_WHEN_LPS_
                else if( table_index == 18 )
                {
                    if(lmp_self_device_data.scan_enable== 0)
                    {
                        sleep_mode_param.lps_table_ing_bitmap &= (UINT32)(~(1<<table_index));
                        continue;
                    }
                }
#else
                else if( table_index == 18 )
                {
                    if(!(lmp_self_device_data.scan_enable&0x2))
                    {
                        continue;
                    }
                }else if( table_index == 19 )
                {
                    if(!(lmp_self_device_data.scan_enable&0x1))
                    {
                        continue;
                    }
                }
#endif
                else if( table_index == 20 )
                {
                    if(ll_manager.adv_unit.enable == 0)
                    {
                        sleep_mode_param.lps_table_ing_bitmap &= (UINT32)(~(1<<table_index));
                        continue;
                    }
                }else if( table_index == 21 )
                {
                    if(ll_manager.scan_unit.enable == 0)
                    {
                        sleep_mode_param.lps_table_ing_bitmap &= (UINT32)(~(1<<table_index));
                        continue;
                    }
                }


                wakeup_instant = sleep_mode_param.lps_table_instance[table_index] ;

                instance_diff = (wakeup_instant | BIT27) - clock;
                instance_diff = instance_diff & ~BIT27;

                if(table_index == 18)
                {
                    scan_diff = instance_diff;
                }

                if( diff > instance_diff )
                {
                    diff = instance_diff;
                    wakeup_index = table_index;

                }
#ifdef PATCH_LPS_LOG_EN_TEST_1
                LPS_CCH_LOG(GRAY, CCH_DBG_148, 4, 0, table_index, diff, instance_diff);
#endif
            }


            if( wakeup_index >= LPS_TABLE_NUM)
            {
                MINT_OS_EXIT_CRITICAL();
                return API_FAILURE;
            }

            if(sleep_mode_param.lps_table_ing_bitmap != 0)
            {
                MINT_OS_EXIT_CRITICAL();
                RT_BT_LOG(BLUE, LPS_LOG_044, 2, sleep_mode_param.wakeup_scan_enable ,sleep_mode_param.lps_table_ing_bitmap);
                return API_FAILURE;
            }

#ifdef PATCH_LPS_LOG_EN_TEST_1
                LPS_CCH_LOG(GRAY, CCH_DBG_148, 4,sleep_mode_param.lps_table_instance[0], sleep_mode_param.lps_table_instance[1],
                sleep_mode_param.lps_table_instance[2], sleep_mode_param.lps_table_instance[3]);
                LPS_CCH_LOG(GRAY, CCH_DBG_148, 4,sleep_mode_param.lps_table_instance[16], sleep_mode_param.lps_table_instance[17],
                sleep_mode_param.lps_table_instance[18], sleep_mode_param.lps_table_instance[19]);
#endif

            osc_delay = otp_str_data.bw_rf_osc_delay;

            if( wakeup_index == 18)
            {
                sleep_mode_param.wakeup_scan_enable = 1;
            }else if( scan_diff<( diff + 20 ) )
            {
                sleep_mode_param.wakeup_scan_enable = 1;
            }

            wakeup_instant = sleep_mode_param.lps_table_instance[wakeup_index];
            sleep_mode_param.lps_table_instance_index = wakeup_index;

            UINT16 dsm_guard_interval;
            if (g_efuse_lps_setting_1.dsm_guard_interval == 7)
            {
                dsm_guard_interval = 16;
            }
            else
            {
                dsm_guard_interval = 2 + g_efuse_lps_setting_1.dsm_guard_interval;
            }

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
            if(sleep_mode_param.run_dlps_flow)
            {
                dsm_guard_interval = dsm_guard_interval + g_efuse_rsvd_2.dlps_guard_time;
            }
#endif

            wakeup_instant = (wakeup_instant - osc_delay)
                                       - dsm_guard_interval;

            if(lc_program_lps_check_diff(lps_mode, wakeup_instant, clock) == API_FAILURE)
            {
                MINT_OS_EXIT_CRITICAL();
                return API_FAILURE;
            }

            if( lc_check_lps_for_idle(1))
            {
                check_cal_lock_flag = 0;
            }

        }else
        {
            if( !lc_check_lps_for_idle(0))
            {
                MINT_OS_EXIT_CRITICAL();
                RT_BT_LOG(RED, CCH_DBG_106, 2, wakeup_instant, clock);
                return API_FAILURE;
            }
            check_cal_lock_flag = 0;
            wakeup_instant = clock  + 0xFF00;
            wakeup_instant = wakeup_instant & BITMASK_27;
        }
#ifdef PATCH_LPS_LOG_EN_TEST_1
        LPS_CCH_LOG(GRAY, LPS_LOG_041, 3, sleep_mode_param.wakeup_scan_enable, clock, wakeup_instant);
#endif
// 20110830 add
        UINT8 cal_lock = 0;
#ifdef _ENABLE_BTON_POWER_SAVING_
        cal_lock = bton_32k_cal_chk_lock();
#endif

        if( (!cal_lock)&& check_cal_lock_flag )
        {
#ifdef _MODI_LPS_AFTER_RTL8703B_
            if( sleep_mode_param.lps_32k_cal_count >= sleep_mode_param.lps_32k_cal_max)
#else
            if( sleep_mode_param.lps_32k_cal_count >= LPS_32K_UNLOCK_MAX_TIME)
#endif
            {
                MINT_OS_EXIT_CRITICAL();

	             // add for resume lps
                RT_BT_LOG(RED, CCH_DBG_144, 3, lps_mode, sleep_mode_param.lps_32k_cal_count, cal_lock);
                sleep_mode_param.lps_32k_cal_count = 0;
                return API_FAILURE;
            }

            MINT_OS_EXIT_CRITICAL();

            sleep_mode_param.lps_32k_cal_count ++ ;

#ifdef _MODI_LPS_AFTER_RTL8703B_
            lps_period_state_enter_lps(lps_mode);
#else
            lc_post_lps_mode_signal(lps_mode, 0, 0);
#endif

            RT_BT_LOG(BLUE, CCH_DBG_144, 3, lps_mode, sleep_mode_param.lps_32k_cal_count, cal_lock);

            return API_FAILURE;
        }
        sleep_mode_param.lps_32k_cal_count = 0;
#endif
    }// end of if(g_efuse_lps_setting_4.lps_use_intr)
    else
#endif
    {
        if(( lps_mode == LPS_TIMER2_WO_SCAN )&&(lmp_self_device_data.scan_enable!=0))
        {
            MINT_OS_EXIT_CRITICAL();
            return API_FAILURE;
        }

        if(( lps_mode == LPS_TIMER2_WITH_SCAN) || ( lps_mode == LPS_TIMER2_WO_SCAN) )
        {
            wakeup_instant = clock  + scan_interval;
            wakeup_instant = wakeup_instant & BITMASK_27;
        }

        if(lc_program_lps_check_diff(lps_mode, wakeup_instant, clock) == API_FAILURE)
        {
            sleep_mode_param.lps_lpm_end_flag = 0;
            MINT_OS_EXIT_CRITICAL();
            return API_FAILURE;
        }

#ifdef _CCH_RTL8723A_B_CUT
        UINT8 cal_lock = 0;
#ifdef _ENABLE_BTON_POWER_SAVING_
        cal_lock = lc_program_lps_check_cal(lps_mode);
#endif

#ifdef _MODI_LPS_AFTER_RTL8703B_
        UINT8 cal_again;
        cal_again = lc_check_lps_task_queue();
        cal_lock |= cal_again;
#endif
        if(cal_lock == API_RETRY)
        {
            MINT_OS_EXIT_CRITICAL();

            sleep_mode_param.lps_lpm_end_flag = 1;
            sleep_mode_param.lps_lpm_lps_mode = lps_mode;
#ifdef _MODI_LPS_AFTER_RTL8703B_
            lps_period_state_enter_lps(lps_mode);
#endif
            return API_RETRY;

        }else if(cal_lock == API_FAILURE)
        {
            MINT_OS_EXIT_CRITICAL();
            return API_FAILURE;
        }
#ifndef _MODI_LPS_AFTER_RTL8821B_TC_
        // move to top
        lc_program_lps_upd_par(lps_mode);
#endif
#endif
    }// end of (!g_efuse_lps_setting_4.lps_use_intr)

    sleep_mode_param.clock_temp2 = BB_read_native_clock();

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
    WRITE_DLPS_MODE(sleep_mode_param.run_dlps_flow);
#endif

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
    // Store Mem
    dlps_save_io_mem_to_partial_on();
#endif

    sleep_mode_param.lps_lpm_end_flag = 0;
    lc_program_lps_execute_lps(lps_mode, wakeup_instant, piconet_id);

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
    if(!sleep_mode_param.run_dlps_flow)
#endif
    {
        MINT_OS_EXIT_CRITICAL();
    }
#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
    LPS_DBG_LOG(GRAY, YL_DBG_HEX_3, 3, g_efuse_rsvd_2.enable_deep_lps_mode, clock<<1, wakeup_instant<<1);
    LPS_DBG_LOG(GRAY, LC_3410, 1, wakeup_instant);

#ifndef IS_BTSOC
    if(sleep_mode_param.run_dlps_flow)
    {
        DisableIntForDLPS();
    }
#endif
#endif

#ifdef _ROM_CODE_PATCHED_
    /* reserved rom code patch by yilinli
      * e.g.
      *    0. something need to be execute outside the critical section
      *    1. wait suitbable sleep_mode_param.bb_sm_sts = NORMAL (to block CPU background tasks)
      *    2. execute lc_exit_sm_mode when timeout
      */
    if (rcp_lc_program_lps_mode_end_func != NULL)
    {
        UINT8 return_status;
        if(rcp_lc_program_lps_mode_end_func((void *)&return_status, &lps_mode, &wakeup_instant))
        {
            return return_status;
        }
    }
#endif

    return API_SUCCESS;
}

void lc_program_lps_mode_stage2(void)
{

#ifdef _ENABLE_RETENTION_FLOW_FOR_DLPS_
        // Move from enter_lps_mode_6128

        if(sleep_mode_param.run_dlps_flow)
        {
#ifndef IS_BTSOC
            //DisableIntForDLPS();    // 20140211 dont need
            WDG_TIMER_DISABLE;
            BTON_WDG_TIMER_DISABLE;

#if defined(_CHECK_STACK_SIZE_B4_ENTER_DLPS_)
        UINT32 sp_cur = *(UINT32*)&_intr_stack[104];
            if (sp_cur < (g_dmem_end_addr[0] - 0x3ff))
            {
                RT_BT_LOG(YELLOW, MSG_SP_CHECK, 2, (g_dmem_end_addr[0] - 0x3ff), sp_cur); 
                Rlx4181EnableInterrupt();
                return;
            }
#endif

            EnterDlpsHandler();
            Rlx4181EnableInterrupt();
#endif
        }
        else
#endif
#ifdef LPS_PROTECT_DSM_BB_ACCESS // YL TEST 2
        {

#ifdef _ENABLE_RETENTION_FLOW_FOR_DLPS_
            SIGN_ENTER_LPS_MODE_FLAG;  /* for debug use */
#endif

#ifdef LPS_NO_WAKEUP_WHEN_NO_SCAN
#ifdef _MODI_LPS_AFTER_RTL8821B_TC_
            if(( lc_check_lps_for_idle(0) == TRUE )&&(g_efuse_lps_setting_2.lps_nowakeup_when_no_scan))
#else
            if( lc_check_lps_for_idle(0) == TRUE )
#endif
            {
                WDG_TIMER_DISABLE;
                BTON_WDG_TIMER_DISABLE;
            }
            else
#endif
            {
                WDG_TIMER_RESTART;  /* avoid watch dog timer timeout and avoid no watch dog timeout */
                BTON_WDG_TIMER_RESTART;
            }


#ifndef IS_BTSOC
            while(sleep_mode_param.bb_sm_sts != BB_NORMAL_MODE)
            {
//            pf_delay_us(1);
                rlx4081sleep();  /* wait any hw interrupt to wakeup 4081 */
            }
#endif
        }
#endif
    }

/**
 * Programs deep sleep mode to the baseband.
 *
 * \param lps_mode (0) sleep mode. (1) deep sleep mode (2) timer2 with scan (3) timer2 without scan
 * \param wakeup_instant The wakeup instant by which sleep mode should have
 *                       terminated. Note that allowance for the tolerance
 *                       value should have been taken by the callee,
 * \param piconet_id The physical piconet ID of the connection.
 *
 * \return None.
 */
UINT8 lc_program_lps_mode(UCHAR lps_mode, UINT32 wakeup_instant, UCHAR piconet_id)
{
    sleep_mode_param.lps_task_flag = 0;
    sleep_mode_param.wakeup_scan_enable = 0;

    /* Check the configuration bit. */
    if (!IS_ENABLE_POWER_SAVE)
    {
        LPS_DBG_LOG(RED, LPS_LOG_005, 0, 0);
        return API_FAILURE;
    }

    if(sleep_mode_param.bb_sm_sts != BB_NORMAL_MODE)
    {
        LPS_DBG_LOG(RED, LPS_LOG_028, 0, 0);
        return API_FAILURE;
    }

#ifdef _ROM_CODE_PATCHED_
    /* Reserved com code patch by yilinli:
      * e.g.
      * 1. check LPO calibration status (e.g. check_fw_lpo_cal_done())
      *     a. done: go ahead
      *     b. else: giveup or wait or create timer or create task again
      * 2. modify function
      * 3. etc.
      */
    UCHAR return_status;
    if (rcp_lc_program_lps_mode_func != NULL)
    {
        if(rcp_lc_program_lps_mode_func((void *)&return_status, &lps_mode, &wakeup_instant, &piconet_id))
        {
            return return_status;
        }
    }
#endif
    UINT8 return_value = lc_program_lps_mode_stage1(lps_mode, wakeup_instant, piconet_id);

    if ( return_value == API_SUCCESS)
    {
        lc_program_lps_mode_stage2();
        return API_SUCCESS;
    }
    else
    {
        return return_value;
    }
}

#endif


/**
 * Forces baseband to move out of power save feature to active mode.
 *
 * \param None.
 *
 * \return None.
 */
void lc_exit_sm_mode(void)
{
    UINT32 timer_val;
    UINT16 div_val;

    if (sleep_mode_param.bb_sm_sts == BB_SLEEP_MODE)
    {
        sleep_mode_param.bb_sm_sts = BB_NORMAL_MODE;

        BB_write_baseband_register(POWER_CONTROL_REGISTER,
                                   (lc_power_ctr_config & 0xFFFC));
    }
    else if (sleep_mode_param.bb_sm_sts == BB_DEEP_SLEEP_MODE)
    {
        LMP_LOG_INFO(LOG_LEVEL_HIGH, DSM_EXIT_SET,0,0);

        pf_set_dsm_exit();

        for(div_val = 0; div_val < 2000; div_val++)
        {
            if (sleep_mode_param.bb_sm_sts == BB_NORMAL_MODE)
            {
                break;
            }
            else
            {
                for(timer_val = 0; timer_val < (0x3000/20); timer_val++);
            }
        }

        LMP_LOG_INFO(LOG_LEVEL_HIGH,DSM_EXIT_CLEAR_MODE, 1,
                     sleep_mode_param.bb_sm_sts);

        pf_clear_dsm_exit();
    }

    return;
}


/**
 * Services the sleep mode interrupt from the baseband.
 *
 * \param int_stat_reg The value of the interrupt status register.
 *
 * \return None.
 */
 // TODO: need to differentiate DSM or SM modes
 // TODO: when wakeup from GPIO or HCI, this function would be executed?
 // TODO: lc_exit_sm_mode() trigger which interrupt"s"??
 // TODO: can we set sleep_mode_param.bb_sm_sts = BB_NORMAL_MODE externally???
void lc_handle_sm_intr(void)
{
    UINT32 read;
#ifdef _YL_LPS
#ifdef _ROM_CODE_PATCHED_
    /* Reserved com code patch by yilinli:
      * e.g. extra wakeup processing, native clock offset tuning, etc.
      */
    if (rcp_lc_handle_sm_intr_func != NULL)
    {
        if ( rcp_lc_handle_sm_intr_func(NULL) )
        {
            return;
        }
    }
#endif



    // TODO: Note: both SM and DSM would trigger this interrupt
  #ifdef _YL_LPS_NEW_PRG_DSM
  #ifndef _YL_LPS_COMBO_PLATFORM
    // TODO: should not check sleep_mode_param.bb_sm_sts == BB_DEEP_SLEEP_MODE to avoid dead-lock?
    if(sleep_mode_param.bb_sm_sts == BB_DEEP_SLEEP_MODE)
    {
#ifdef _ENABLE_BTON_POWER_SAVING_
        execute_exit_lps_mode_procedure_6128();
#endif
    }
  #endif
  #endif

    lps_gpio_one_pull_high(LPS_GPIO_EXIT_DSM);
    lps_gpio_one_pull_low(LPS_GPIO_EXIT_DSM);
    lps_gpio_one_pull_low(LPS_GPIO_DSM_PERIOD);
#endif

    sleep_mode_param.bb_sm_sts = BB_NORMAL_MODE;
#ifdef _MODI_LPS_AFTER_RTL8821B_TC_
    //sleep_mode_param.lps_procedure_start = 0;
    sleep_mode_param.lps_lpm_end_flag = 0;
#endif

    UINT8 run_dlps_flow_temp = 0;

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
    // to clear po_intr
    read = BB_read_baseband_register(POWER_STATUS_REGISTER);
#ifdef _YL_LPS
    LPS_DBG_LOG(GRAY, LPS_LOG_001, 1, read);
#endif

// After 32k disable
    dlps_restore_io_mem_from_partial_on_after_clk_rdy();
    run_dlps_flow_temp = sleep_mode_param.run_dlps_flow;

#endif

#ifdef _ENABLE_BTON_POWER_SAVING_
#ifdef LPS_PROTECT_DSM_BB_ACCESS // YL TEST 2
    rlx4081_isr_sw_unmask();
#endif
#endif

#ifdef _CCH_RTL8723A_B_CUT
#ifdef _ENABLE_BTON_POWER_SAVING_
// _CCH_ECO_LPS_
    bton_32k_cal_en();
#endif
#endif

#ifdef PATCH_LPS_RST_CLK_PHASE
    UINT16 lc_power_ctr_config_0;
    UINT16 lc_power_ctr_config_9;
    lc_power_ctr_config_0 = (UINT16) (otp_str_data.bw_rf_radio_sys_clk_sel << 3);
    lc_power_ctr_config_0 |= otp_str_data.bw_rf_low_clk_frac << 2;
    lc_power_ctr_config_0 |= otp_str_data.bw_rf_osc_delay << 8;

    lc_power_ctr_config_0 &= 0xff07;
    lc_power_ctr_config_9 = (lc_power_ctr_config_0 | 0x0048);

    BB_write_baseband_register(POWER_CONTROL_REGISTER, lc_power_ctr_config_0);
    pf_delay_us(7); // Chinwen need at least 6.4usec
    BB_write_baseband_register(POWER_CONTROL_REGISTER, lc_power_ctr_config_9);
#endif

    UINT32 temp_patch_clock1 = sleep_mode_param.clock_temp1;
    sleep_mode_param.clock_temp1 = BB_read_native_clock();
    sleep_mode_param.lps_wakeup_count ++;

    if(sleep_mode_param.lps_log_enable)
    {
        RT_BT_LOG(GRAY, LPS_LOG_042, 8,
        sleep_mode_param.lps_wakeup_count,
        sleep_mode_param.clock_temp2-temp_patch_clock1,
        sleep_mode_param.clock_temp1-temp_patch_clock1,
        run_dlps_flow_temp,
        sleep_mode_param.lps_period_state_bitmap,
        sleep_mode_param.bb_sm_wakeup_inst,
        temp_patch_clock1,
        0);
    }

#ifdef _LPS_LOG_EN_1

//  #ifdef _YL_LPS_LOG_
    // Note: for debug only
    UINT32 clock;

#ifdef _LPS_FOR_8821_
    clock = BB_read_native_clock();
#else
    lc_get_clock_in_scatternet(&clock, sleep_mode_param.bb_sm_piconet_id);
#endif

    RT_BT_LOG(GRAY, LPS_LOG_040, 3,
              clock>>1,
              sleep_mode_param.bb_sm_wakeup_inst, sleep_mode_param.run_dlps_flow);

#endif

#ifdef SCNET_DEBUG
    LMP_LOG_INFO(LOG_LEVEL_HIGH, WOKEUP_FROM_SLEEPMODE, 0, 0);
#endif

#ifndef _CCH_RETENTION_FLOW_FOR_DLPS_ // move earlier
    // to clear po_intr
    read = BB_read_baseband_register(POWER_STATUS_REGISTER);
#ifdef _YL_LPS
    LPS_DBG_LOG(GRAY, LPS_LOG_001, 1, read);
#endif
#endif

    sleep_mode_param.lps_lpm_end_flag =  0;
#ifdef _CCH_LPS_USING_STATE_FAST_
    if( g_efuse_lps_setting_4.lps_use_state_fast_lps )
    {
        if( g_lps_timer_counter == 0)
        {
            EFUSE_POW_PARA_S_TYPE efuse_pow_para;
            efuse_pow_para.d16 = otp_str_data.power_seq_param;

            if (g_efuse_lps_setting_2.timer2_mode_sel )
            {
                if(g_efuse_lps_setting_4.lps_chk_g_host_wake_bt != 0)
                {
                    if(g_host_wake_bt)
                    {  // if g_host_wake_bt == 1, can not enter lps mode
                        RT_BT_LOG(RED, CCH_DBG_019, 0,0);
                        return;
                    }
                }

                if ((efuse_pow_para.b.lps_timer_en) &&
                    (g_efuse_lps_setting_2.timer2_lps_on))
                {
                    if(g_efuse_lps_setting_4.lps_use_state)
                    {
#ifndef _MODI_LPS_AFTER_RTL8703B_
                        if(lmp_self_device_data.scan_enable)
#endif
                        {
                            if( lc_check_lps_for_idle(1))
                            {
                                #ifdef _CCH_NEED_TO_CHK_                        // need to move to task handle
                                #endif
                                UINT8 state = sleep_mode_param.lps_period_state;
                                UINT8 bitmap = sleep_mode_param.lps_period_state_bitmap;
                                UINT8 count = sleep_mode_param.lps_period_count;
                                sleep_mode_param.lps_lpm_lps_mode = LPS_TIMER2_WITH_SCAN;

                                if(!sleep_mode_param.lps_task_flag)		 // 20110310
                                {
#ifdef _MODI_LPS_STATE_WITH_INTR_
                                    if(g_efuse_rsvd_2.lps_state_with_intr)
                       	            {
                                        lc_post_lps_stste_signal(LPS_TIMER2_WITH_SCAN);
                                    }
                                    else
#endif
                                    {
                                        lps_period_state_machine(state, bitmap, count, LPS_TIMER2_WITH_SCAN, 0, 0);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
#endif


    return;
}


/**
 * Calculates the number of free slots for which the baseband will
 * enter power save mode.
 *
 * \param num_sleep_slots The number of slots, as calculated in this function.
 *
 * \return None.
 */
void lc_calculate_num_park_sleep_slots(UINT16 *num_sleep_slots)
{
#ifdef COMPILE_PARK_MODE
    UINT16 ce_index;
    LMP_CONNECTION_ENTITY *ce_ptr;

    *num_sleep_slots = 0;

    for(ce_index = 0; ce_index < LMP_MAX_CE_DATABASE_ENTRIES; ce_index++)
    {
        ce_ptr = &lmp_connection_entity[ce_index];

        if (ce_ptr->ce_status == LMP_PARK_MODE)
        {
            break;
        }
    }

    if(ce_index != LMP_MAX_CE_DATABASE_ENTRIES)
    {
        *num_sleep_slots = ce_ptr->Tbeacon - ce_ptr->D_access -
                           (ce_ptr->M_access * ce_ptr->T_access);
    }
#endif

    return;

}
#ifndef _REDUCE_LPS_AFTER_RTL8703B_
/**
 * Calls signal to program sleep mode.
 *
 * \param wakeup_instant The wakeup instant by which sleep mode should have
 *                       terminated.
 * \param piconet_id The physical piconet ID of the connection.
 *
 * \return API_SUCCESS, if the operation is successful, API_FAILURE otherwise.
 */
API_RESULT lc_post_sm_mode_signal(UINT32 wakeup_instant, UCHAR piconet_id)
{
    OS_SIGNAL signal;
    signal.type = LC_ENTER_SM_MODE;
    signal.param = (OS_ADDRESS)(wakeup_instant);
    signal.ext_param = (OS_ADDRESS)((UINT32) (piconet_id));
    OS_ISR_SEND_SIGNAL_TO_TASK(lc_rx_task_handle, signal);

    return API_SUCCESS;
}

/**
 * Calls signal to program deep sleep mode.
 *
 * \param wakeup_instant The wakeup instant by which sleep mode should have
 *                       terminated.
 * \param piconet_id The physical piconet ID of the connection.
 *
 * \return API_SUCCESS, if the operation is successful, API_FAILURE otherwise.
 */
API_RESULT lc_post_dsm_mode_signal(UINT32 wakeup_instant, UCHAR piconet_id)
{
    OS_SIGNAL signal;
    signal.type = LC_ENTER_DSM_MODE;
    signal.param = (OS_ADDRESS)(wakeup_instant);
    signal.ext_param = (OS_ADDRESS)((UINT32) (piconet_id));
    OS_ISR_SEND_SIGNAL_TO_TASK(lc_rx_task_handle, signal);

    return API_SUCCESS;
}
#endif
#if defined(_CCH_LPS_) && defined(_YL_LPS)
API_RESULT lc_post_lps_mode_signal(UCHAR lps_mode, UINT32 wakeup_instant, UCHAR piconet_id)
{
    OS_SIGNAL signal;
    signal.param = (OS_ADDRESS)(wakeup_instant);
    signal.ext_param = (OS_ADDRESS)((UINT32) ( (lps_mode<<8) +  piconet_id));


    LPS_DBG_LOG(BLUE, LPS_LOG_025, 3,lps_mode, wakeup_instant, piconet_id);

    if( g_efuse_lps_setting_2.lps_pri == 0 )
    {
#ifdef COMPILE_CHANNEL_ASSESSMENT
        signal.type = CH_AS_TASK_ENTER_LPS_MODE;
        if (OS_ISR_SEND_SIGNAL_TO_TASK(hci_ch_as_task_handle, signal) != BT_ERROR_OK )
        {
            sleep_mode_param.lps_task_flag = 0;
            return API_FAILURE;
        }
#endif
    }
    else
    {
        signal.type = LC_ENTER_LPS_MODE;
        if (OS_ISR_SEND_SIGNAL_TO_TASK(lc_rx_task_handle, signal) != BT_ERROR_OK )
        {
            sleep_mode_param.lps_task_flag = 0;
            return API_FAILURE;
        }
    }

    sleep_mode_param.lps_procedure_start = 1;
    sleep_mode_param.lps_task_flag = 1;

    return API_SUCCESS;
}
#endif

#ifdef _MODI_LPS_STATE_WITH_INTR_
API_RESULT lc_post_lps_stste_signal(UCHAR lps_mode)
{
    OS_SIGNAL signal;
    signal.type = LC_ENTER_LPS_STATE;
    if (OS_ISR_SEND_SIGNAL_TO_TASK(lc_rx_task_handle, signal) != BT_ERROR_OK )
    {
        return API_FAILURE;
    }

    return API_SUCCESS;
}
#endif

#ifdef _YL_LPS
#ifndef _REDUCE_LPS_AFTER_RTL8703B_
API_RESULT lc_post_dsm_mode_signal_test(UINT32 sleep_duration_slot, UCHAR piconet_id)
{
    OS_SIGNAL signal;
    UINT32 clock;
    UINT32 slot_ind;
    UINT32 wakeup_instant;

    lc_get_clock_in_scatternet(&clock, piconet_id);
    slot_ind = clock >> 1;
    wakeup_instant = slot_ind + sleep_duration_slot;
    wakeup_instant = wakeup_instant & BITMASK_27;

    LPS_DBG_LOG(GRAY, LPS_LOG_003, 4, sleep_duration_slot, piconet_id, slot_ind, wakeup_instant);


    signal.type = LC_ENTER_DSM_MODE;
    signal.param = (OS_ADDRESS)(wakeup_instant);
    signal.ext_param = (OS_ADDRESS)((UINT32) (piconet_id));
    lmp_self_device_data.number_of_hlc = 1;
    OS_ISR_SEND_SIGNAL_TO_TASK(lc_rx_task_handle, signal);

    return API_SUCCESS;
}
#endif
#endif
/**
 * Programs sleep mode or deep sleep mode to the baseband in sniff state.
 *
 * \param interval The duration for which the baseband will be in
 *                 power save mode.
 *
 * \return None.
 */
void lc_program_sniff_sm_mode(UINT16 ce_index)
{
    UINT32 ps_clock;
    UINT16 ps_duration;
    LMP_CONNECTION_ENTITY *ce_ptr;
    UCHAR cur_pic_id;
    UCHAR lut_index;


    ce_ptr = &lmp_connection_entity[ce_index];

#ifdef _LPS_FOR_8821_
    cur_pic_id = ce_ptr->phy_piconet_id;
#else
    cur_pic_id = lc_get_connected_piconet_id();
#endif

    if(cur_pic_id == SCA_PICONET_INVALID)
    {
        return;
    }

#ifdef _ROM_CODE_PATCHED_
    /* Reserved com code patch by yilinli:
      * e.g.
      *    1.
      */
    if (rcp_lc_program_sniff_sm_mode_func != NULL)
    {
        if ( rcp_lc_program_sniff_sm_mode_func((void*)(&ce_index)) )
        {
            return;
        }
    }
#endif


    lut_index = lc_get_lut_index_from_ce_index(ce_index);

    if(lut_index == 0)
    {
        return;
    }


    if( !lc_check_lps_for_link( 1, 0, 0))
   {
        g_lps_timer_counter = (g_timer2_init_value);
        return;
   }


#ifdef COMPILE_SNIFF_MODE
    // TODO: Is sniff_last_instant always correctly updated?
    ps_clock = ce_ptr->sniff_last_instant;
#ifdef _YL_LPS
    if (g_efuse_lps_setting_3.sniff_wakeup_xtol_opt==1)
    {
        ps_clock -= (ce_ptr->sniff_xtol );
    }
    else if (g_efuse_lps_setting_3.sniff_wakeup_xtol_opt==2)
    {
        ps_clock -= (ce_ptr->sniff_xtol - ce_ptr->sniff_xtol_prev);
    }
#endif
    ps_clock >>= 1;

    /* Note that we do not have to subtract tolerance here as
     * sniff_last_instant is the clock value at last sniff start
     * interrupt, rather than the actual sniff instant .*/
    ps_clock = ps_clock + ce_ptr->sniff_interval;
    ps_clock = ps_clock & BITMASK_27;

    ps_duration = ce_ptr->sniff_interval;

#ifdef _ROM_CODE_PATCHED_
    /* Reserved com code patch by yilinli:
      * e.g.
      * 1. read new tolerance value (e.g. by BB_read_xtol_in_scatternet())
      * 2. modify ps_clock(wakeup_instant) according to old and new tolerance
      * 3. etc.
      */
    if (rcp_lc_sniff_sm_tune_wakeup_instant_func != NULL)
    {
        if ( rcp_lc_sniff_sm_tune_wakeup_instant_func((void*)(&ce_index), &ps_clock, &ps_duration) )
        {
            return;
        }
    }
#endif

#ifdef _CCH_LPS_

    if( g_efuse_lps_setting_2.sniff_mode_sel)
    {
        if((lmp_self_device_data.scan_enable != 0) && (!g_efuse_lps_setting_2.sniff_scan_en) )
        {
            return;
        }

#ifdef RTL8723A_B_CUT

#ifdef _LPS_LOG_EN_TEST_
        if((lc_sniff_sup_count[lut_index - 1] == 0)&&(ps_duration > otp_str_data.bt_deep_sleep_mode_threshold) )
        {
            LPS_CCH_LOG(YELLOW, LPS_LOG_021, 1, lut_index);
        }

        if((lc_sniff_sup_count[lut_index - 1] != 0)&&(ps_duration > otp_str_data.bt_deep_sleep_mode_threshold) )
        {
            RT_BT_LOG(BLUE, LPS_LOG_044, 3,lut_index, lc_sniff_sup_count[lut_index - 1], ps_clock<<1);
        }
#endif
#ifdef _MODI_LPS_AFTER_RTL8821B_TC_
        if((g_lps_timer_counter > 0)||(lc_sniff_sup_count[lut_index - 1] > g_efuse_lps_setting_2.lc_sniff_sup_max))
#else
        if((g_lps_timer_counter > 0)||(lc_sniff_sup_count[lut_index - 1] > 2))
#endif
        {
            return;
        }

#endif

    }

#endif

    if (ps_duration >= otp_str_data.bt_deep_sleep_mode_threshold)
    {

#ifdef _CCH_LPS_
        if( g_efuse_lps_setting_2.sniff_mode_sel )
        {
#ifndef _REDUCE_LPS_AFTER_RTL8703B_
            if( g_efuse_lps_setting_4.lps_use_intr)
            {
                sleep_mode_param.lps_table_instance[ce_index] = ps_clock;

                if(sleep_mode_param.wakeup_scan_enable)
                {
                    RT_BT_LOG(BLUE, LPS_LOG_046, 2,sleep_mode_param.wakeup_scan_enable, sleep_mode_param.lps_table_ing_bitmap);
                    return;
                }
                lc_post_lps_mode_signal(LPS_WITH_WAKEUP_INSTANCE, 0, 0);
            }else
#endif
            {

                if( lc_lps_double_check_dsm_cond(LPS_SNIFF_DSM) == API_FAILURE)
                {
                    return;
                }

#if defined(_CCH_RTL8723A_B_CUT) && defined(_CCH_LPS_USING_STATE_)
// _CCH_ECO_LPS_
            if(g_efuse_lps_setting_4.lps_use_state)
            {
#ifndef _CCH_LPS_USING_STATE_FAST_
                sleep_mode_param.lps_lpm_end_flag = 1;
                sleep_mode_param.lps_lpm_wakeup_instance = ps_clock;
                sleep_mode_param.lps_lpm_lps_mode = LPS_SNIFF_DSM;
#else
#ifdef _MODI_LPS_AFTER_RTL8821B_TC_
                lps_period_state_machine_fast(ps_clock, ps_duration);
#else

                sleep_mode_param.lps_lpm_end_flag = 1;
                sleep_mode_param.lps_lpm_wakeup_instance = ps_clock;
                sleep_mode_param.lps_lpm_lps_mode = LPS_SNIFF_DSM;

                if(g_efuse_lps_setting_4.lps_use_state_fast_lps)
                {
                    sleep_mode_param.lps_link_interval = ps_duration;

                    UINT8 enter_lps_now = 0;

                    if ((lmp_self_device_data.number_of_hlc != 1) ||
                        (lmp_self_device_data.lc_cur_dev_state != LC_CUR_STATE_IDLE))
                    {
                        return;
                    }

                    if( lmp_self_device_data.scan_enable
#ifdef LE_MODE_EN
                        || (IS_BT40 && (ll_manager.adv_unit.enable || ll_manager.scan_unit.enable))
#endif
                        )
                    {
                        if(lmp_self_device_data.lps_sniff_scan_en == 0)
                        {
                            if( lmp_self_device_data.scan_enable&1 )
                            {
#ifdef _DAPE_TEST_NEW_HW_INSTRUCTION
                                BB_write_baseband_register(INSTRUCTION_REGISTER, BB_KILL_INQUIRY_SCAN);
#else
                                BB_write_baseband_register(INSTRUCTION_REGISTER, BB_KILL_INQUIRY);
#ifdef _FREE_FHS_PKT_WHEN_KILL_INQUIRY_
                                lc_free_fhs_pkt_buffer();
#endif
#endif
                            }
                            if( lmp_self_device_data.scan_enable&2 )
                            {
                                g_lps_scan_status_stay_count = 0;
#ifdef _DAPE_TEST_NEW_HW_INSTRUCTION
                                BB_write_baseband_register(INSTRUCTION_REGISTER, BB_KILL_PAGE_SCAN);
#else
                                BB_write_baseband_register(INSTRUCTION_REGISTER, BB_KILL_PAGE);
#endif
                                lc_pagescan_piconet_id = SCA_PICONET_INVALID;
                            }

#ifdef LE_MODE_EN
                            if (IS_BT40)
                            {
                                if(ll_manager.adv_unit.enable)
                                {
                                    ll_driver_disable_advertising();
                                }

                                if(ll_manager.scan_unit.enable)
                                {
                                    ll_driver_disable_scanning();
                                }
                            }
#endif
                            enter_lps_now = 1;
                        }
                    }

                    if((lmp_self_device_data.scan_enable == 0)
#ifdef LE_MODE_EN
                        && (!IS_BT40 || ((ll_manager.adv_unit.enable ==0) && (ll_manager.scan_unit.enable==0)))
#endif
                        )
                    {
                        lmp_self_device_data.lps_sniff_scan_en = 0;
                        enter_lps_now = 1;
                    }

                    if( enter_lps_now)
                    {
                        lps_period_state_enter_lps(LPS_SNIFF_DSM);
                    }else
                    {
                        UINT8 state = sleep_mode_param.lps_period_state;
                        UINT8 bitmap = sleep_mode_param.lps_period_state_bitmap;
                        UINT8 count = sleep_mode_param.lps_period_count;
                        lps_period_state_machine(state, bitmap, count, LPS_SNIFF_DSM, cur_pic_id, ps_clock);

                    }
                }
#endif
#endif
	    }
#else
            if(lmp_self_device_data.scan_enable != 0)
            {
                UINT16 lps_sniff_scan_interval;

                lps_sniff_scan_interval = (ps_duration >> g_efuse_lps_setting_2.dsm_scan_per);

                if( lps_sniff_scan_interval < g_efuse_lps_setting_2.dsm_scan_per_min )
                {
                    lps_sniff_scan_interval = g_efuse_lps_setting_2.dsm_scan_per_min;
                }

                if( lmp_self_device_data.lps_sniff_scan_count == 0)
                {  // start scan here
                    lc_check_and_enable_scans_in_scatternet();
                }else
                {  // enter deep sleep mode here

                    lc_kill_scan_mode();
                    lc_post_lps_mode_signal(1,ps_clock, cur_pic_id);
                }

                if( lmp_self_device_data.lps_sniff_scan_count < lps_sniff_scan_interval)
                {
                    lmp_self_device_data.lps_sniff_scan_count ++;
                }else
                {
                    lmp_self_device_data.lps_sniff_scan_count = 0;
                }

            }else
            {
                lc_post_lps_mode_signal(1,ps_clock, cur_pic_id);
            }
#endif
        }
        }
#else
        lc_post_dsm_mode_signal(ps_clock, cur_pic_id);
#endif
        LPS_DBG_LOG(GRAY, LPS_LOG_015, 5, ps_clock, ps_duration, cur_pic_id, ce_ptr->sniff_last_instant, ce_ptr->sniff_interval);
    }
#ifndef _REDUCE_LPS_AFTER_RTL8703B_
    else if(ps_duration > otp_str_data.bt_sleep_mode_threshold)
    {


#ifdef _CCH_LPS_

        if( g_efuse_lps_setting_2.sniff_mode_sel )
        {

#if defined(_CCH_RTL8723A_B_CUT) && defined(_CCH_LPS_USING_STATE_)
// _CCH_ECO_LPS_
            if(g_efuse_lps_setting_4.lps_use_state)
            {
                sleep_mode_param.lps_lpm_end_flag = 1;
                sleep_mode_param.lps_lpm_wakeup_instance = ps_clock;
                sleep_mode_param.lps_lpm_lps_mode = LPS_SNIFF_SM;
            }
#else
            if(lmp_self_device_data.scan_enable != 0)
            {
                UINT16 lps_sniff_scan_interval;

                lps_sniff_scan_interval = (ps_duration >> g_efuse_lps_setting_2.sm_scan_per);

                if( lps_sniff_scan_interval < g_efuse_lps_setting_2.sm_scan_per_min )
                {
                    lps_sniff_scan_interval = g_efuse_lps_setting_2.sm_scan_per_min;
                }

                if( lmp_self_device_data.lps_sniff_scan_count == 0)
                {  // start scan here
                    lc_check_and_enable_scans_in_scatternet();
                }else
                {  // enter sleep mode here

                    lc_kill_scan_mode();
                    lc_post_lps_mode_signal(0,ps_clock, cur_pic_id);
                }

                if( lmp_self_device_data.lps_sniff_scan_count < lps_sniff_scan_interval)
                {
                    lmp_self_device_data.lps_sniff_scan_count ++;
                }else
                {
                    lmp_self_device_data.lps_sniff_scan_count = 0;
                }

            }else
            {
                lc_post_lps_mode_signal(0,ps_clock, cur_pic_id);
            }
#endif
        }
#ifndef _REDUCE_LPS_AFTER_RTL8703B_
	 else
        {
            lc_post_sm_mode_signal(ps_clock, cur_pic_id);
        }
#endif
#else
        lc_post_sm_mode_signal(ps_clock, cur_pic_id);
#endif


        LPS_DBG_LOG(GRAY, LPS_LOG_016, 3, ps_clock, ps_duration, cur_pic_id, ce_ptr->sniff_last_instant, ce_ptr->sniff_interval);
    }
#endif
    else
    {
        LPS_DBG_LOG(GRAY, LPS_LOG_017, 3, ps_clock, ps_duration, cur_pic_id, ce_ptr->sniff_last_instant, ce_ptr->sniff_interval);
    }
#endif /* COMPILE_SNIFF_MODE */

    return;
}

#ifndef _DISABLE_HOLD_MODE_
/**
 * Programs sleep mode or deep sleep mode to the baseband in hold state.
 *
 * \param interval The duration for which the baseband will be in
 *                 power save mode.
 *
 * \return None.
 */
void lc_program_hold_sm_mode(UINT16 hold_interval, UINT32 hold_instant)
{
#ifndef _REDUCE_LPS_AFTER_RTL8703B_
    UINT32 wakeup_instant;
    UINT16 tolerance;
#endif
    UCHAR cur_pic_id;

    cur_pic_id = lc_get_connected_piconet_id();

    if(cur_pic_id == SCA_PICONET_INVALID)
    {
        return;
    }

#ifdef _ROM_CODE_PATCHED_REDUCE_
#ifdef _CCH_8723_RCP_
    if (rcp_lc_program_hold_sm_mode != NULL)
    {
        if ( rcp_lc_program_hold_sm_mode((void*)(&hold_interval), &hold_instant) )
        {
            return;
        }
    }
#endif
#endif
#ifndef _REDUCE_LPS_AFTER_RTL8703B_
    if (cur_pic_id <= SCA_PICONET_SECOND)
    {
        tolerance = BB_read_baseband_register(X_VALUE_FOR_TOLERANCE_REGISTER);
        if (cur_pic_id == SCA_PICONET_FIRST)
        {
            tolerance &= 0xFF;
        }
        else
        {
            tolerance >>= 8;
        }
    }
    else
    {
        tolerance = BB_read_baseband_register(X_VALUE_FOR_TOLERANCE1_REGISTER);
        if (cur_pic_id == SCA_PICONET_THIRD)
        {
            tolerance &= 0xFF;
        }
        else
        {
            tolerance >>= 8;
        }
    }
    tolerance = tolerance >> 1;

    /* Sleep mode should be over xtol/2 slots before waking up from hold. */
    wakeup_instant = (hold_instant + hold_interval - tolerance) & BITMASK_27;

    if (hold_interval > otp_str_data.bt_deep_sleep_mode_threshold)
    {
        lc_post_dsm_mode_signal(wakeup_instant, cur_pic_id);
    }
    else if(hold_interval > otp_str_data.bt_sleep_mode_threshold)
    {
        lc_post_sm_mode_signal(wakeup_instant, cur_pic_id);
    }
#endif

    return;
}
#endif /* end of #ifndef _DISABLE_HOLD_MODE_ */

#ifndef _DISABLE_PARK_MODE_
/**
 * Programs sleep mode or deep sleep mode to the baseband in park state.
 *
 * \param interval The duration for which the baseband will be in
 *                 power save mode.
 *
 * \return None.
 */
void lc_program_park_sm_mode(UINT16 interval)
{
#ifndef _REDUCE_LPS_AFTER_RTL8703B_
    UINT32 ps_clock;
#endif
    UCHAR cur_pic_id;

    cur_pic_id = lc_get_connected_piconet_id();

    if(cur_pic_id == SCA_PICONET_INVALID)
    {
        return;
    }

#ifdef _ROM_CODE_PATCHED_REDUCE_
#ifdef _CCH_8723_RCP_
    if (rcp_lc_program_park_sm_mode != NULL)
    {
        if ( rcp_lc_program_park_sm_mode((void*)(&interval)) )
        {
            return;
        }
    }
#endif
#endif

#ifndef _REDUCE_LPS_AFTER_RTL8703B_
    lc_get_clock_in_scatternet(&ps_clock, SCA_PICONET_FIRST);

    ps_clock >>= 1;
    ps_clock = (ps_clock + interval - 2) & BITMASK_27;

    if (interval > otp_str_data.bt_deep_sleep_mode_threshold)
    {
        lc_post_dsm_mode_signal(ps_clock, cur_pic_id);
    }
    else if(interval > otp_str_data.bt_sleep_mode_threshold)
    {
        lc_post_sm_mode_signal(ps_clock, cur_pic_id);
    }
#endif
    return;
}
#endif /* end of #ifndef _DISABLE_PARK_MODE_ */
#endif /* POWER_SAVE_FEATURE */

/**
 * Enables PTT in the baseband.
 *
 * \param ce_index Index to lmp-connection-entity.
 *
 * \return None.
 */
void lc_enable_ptt_bit(UINT16 ce_index)
{
    UCHAR am_addr;
    UINT16 lut_address;
    UCHAR lut_index;
    UCHAR phy_piconet_id;
    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];
    am_addr = ce_ptr->am_addr;

    /* Calculate the lut-address. */
    phy_piconet_id = ce_ptr->phy_piconet_id;
    lut_index = lc_get_lut_index_from_phy_piconet_id(
                    am_addr, phy_piconet_id);
    lut_address = lut_ex_table[lut_index].upper_lut_address;

    /* Set the PTT bit in LUT. */
    LC_SET_PTT_BIT_IN_LUT(lut_address);

#ifdef SCNET_DEBUG
    RT_BT_LOG(GRAY, LC_3876, 3, am_addr ,phy_piconet_id,lut_address);
#endif /* SCNET_DEBUG */

    return;
}


/**
 * Disables PTT in the baseband.
 *
 * \param ce_index Index to lmp-connection-entity.
 *
 * \return None.
 */

void lc_disable_ptt_bit(UINT16 ce_index)
{
    UCHAR am_addr;
    UINT16 lut_address;
    UCHAR lut_index;
    UCHAR phy_piconet_id;
    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];
    am_addr = ce_ptr->am_addr;

    /* Calculate the lut-address. */
    phy_piconet_id = ce_ptr->phy_piconet_id;
    lut_index = lc_get_lut_index_from_phy_piconet_id(
                    am_addr, phy_piconet_id);
    lut_address = lut_ex_table[lut_index].upper_lut_address;

    LC_RESET_PTT_BIT_IN_LUT(lut_address);

#ifdef SCNET_DEBUG
    RT_BT_LOG(GRAY, LC_3923, 3, am_addr ,phy_piconet_id, lut_address);
#endif /* SCNET_DEBUG */

    return;
}

#ifdef POWER_CONTROL
/**
 * Reads the current power level from the LUT.
 *
 * \param am_addr The physical transport address of the connection.
 *
 * \return read The val read from the LUT.
 */
UINT16 lc_get_current_power_value(UCHAR am_addr, UCHAR phy_piconet_id)
{
    UINT16 address;
    UINT16 read;
    UCHAR lut_index;

    lut_index = lc_get_lut_index_from_phy_piconet_id(am_addr, phy_piconet_id);
    address = lut_ex_table[lut_index].upper_lut_address;

    read = BB_read_baseband_register(address);
    read >>= 9;
    read &= 0x7;

    return read;
}

/**
* Programs power level in LUT in scatternet.
*
* \param am_addr The logical transport address of the connection.
* \param tx_gain_value The value of gain to be programmed in LUT.
* \param piconet_id The Piconet_ID of the connection.
* \param piconet_id The ce_index of the connection.
*
* \return None.
*/
void lc_program_power_level_in_lut_for_scatternet(UCHAR am_addr,
        UINT16 tx_gain_value, UINT16 ce_index)
{
    UINT16 address;
    UINT16 read;
    UINT16 lcl_tx_gain_val;
    UCHAR lut_index;
    UCHAR phy_piconet_id;
    DEF_CRITICAL_SECTION_STORAGE;

    phy_piconet_id = lmp_connection_entity[ce_index].phy_piconet_id;

    lut_index = lc_get_lut_index_from_phy_piconet_id(am_addr, phy_piconet_id);
    address = lut_ex_table[lut_index].upper_lut_address;

    MINT_OS_ENTER_CRITICAL();
    read = BB_read_baseband_register(address);
    read = (UINT16)(read & 0xC1FF);
    lcl_tx_gain_val = (UINT16) (tx_gain_value << 9);
    read = (UINT16)(read | lcl_tx_gain_val);

#ifdef _SUPPORT_LBT_FUNC_FOR_RF_ADAPTIVITY_
    if (rtl8723_btrf_check_and_enable_lbt(tx_gain_value))
    {
        /* enable lbt_en_lut bit */
        read |= BIT12;
    }
#endif

    BB_write_baseband_register(address, read);

    /* add by austin to updata tx gain index for different modulation */
    LC_SET_2M_TX_POWER(lut_index, tx_gain_value);
    LC_SET_3M_TX_POWER(lut_index, tx_gain_value);

    MINT_OS_EXIT_CRITICAL();

    return;
}


/**
 * Programs tx-power level to the baseband, depending on the
 * type of radio selected.
 *
 * \param
 *
 * \return None.
 */

void lc_program_tx_power(UCHAR am_addr, UINT16 ce_index, UINT32 change_type, UINT16 tx_gain_value)
{
    //	UINT16 tx_gain_value;
    UINT32 temp_var = FALSE;
    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    //	tx_gain_value = (UINT16) lc_get_current_power_value(am_addr,
    //		ce_ptr->phy_piconet_id);


    if (change_type == LC_TX_POWER_LEVEL_INCREMENT)
    {
        temp_var = LC_CHECK_FOR_INCR_POWER_LEVEL((UCHAR)tx_gain_value);

        if(temp_var == TRUE)
        {
            /* We are not already at max-power, we can inc tx-power level */
            tx_gain_value = LC_INCR_POWER_LEVEL((UCHAR)tx_gain_value);
        }
    }
    else if (change_type == LC_TX_POWER_LEVEL_DECREMENT)
    {
        temp_var = LC_CHECK_FOR_DECR_POWER_LEVEL((UCHAR)tx_gain_value);

        if(temp_var == TRUE)
        {
            /* We are not already at min-power, we can dec tx-power level */
            tx_gain_value = LC_DECR_POWER_LEVEL((UCHAR)tx_gain_value);
        }
    }

    /* Program the updated power to LUT. */
    if(temp_var == TRUE)
    {
        lc_program_power_level_in_lut_for_scatternet(
            am_addr, tx_gain_value, ce_index);

        RT_BT_LOG(GREEN, LC_PROGRAM_TX_POWER_1, 2, ce_index, tx_gain_value);
    }

    return;
}
#endif /* POWER_CONTROL */


/**
* Programs Sniff transition mode, while moving from sniff mode to
*  active mode.
*
* \param ce_index Index to CE.
*
* \return None.
*/
void lc_program_exit_sniff_transition_mode(UINT16 ce_index)
{
#ifndef _CCH_DISABLE_SNIFF_TRAN_
    LMP_CONNECTION_ENTITY *ce_ptr;
    DEF_CRITICAL_SECTION_STORAGE;

    ce_ptr = &lmp_connection_entity[ce_index];

    LC_EXIT_SM_MODE();

    MINT_OS_ENTER_CRITICAL();

    BB_write_baseband_register(CONNECTOR_REGISTER,
                               ( (ce_ptr->am_addr << 5) |
                               (ce_ptr->phy_piconet_id << 11) ) );

    BB_write_baseband_register(INSTRUCTION_REGISTER,
                               BB_ENTER_SNIFF_TRAN_MODE_DURING_EXIT_SNIFF_MASTER);
    MINT_OS_EXIT_CRITICAL();

    //RT_BT_LOG(GRAY, LC_4152, 0, 0);

    return;
#endif
}

/**
 * Kills sniff transition mode for the connection.
 *
 * \param ce_index Index to CE.
 *
 * \return None.
 */
void lc_program_kill_sniff_transition_mode(UINT16 ce_index)
{
#ifndef _CCH_DISABLE_SNIFF_TRAN_
    LMP_CONNECTION_ENTITY *ce_ptr;
    DEF_CRITICAL_SECTION_STORAGE;

    ce_ptr = &lmp_connection_entity[ce_index];

    LC_EXIT_SM_MODE();

    MINT_OS_ENTER_CRITICAL();
    BB_write_baseband_register(CONNECTOR_REGISTER,
                               ((ce_ptr->am_addr << 5) |
                               (ce_ptr->phy_piconet_id << 11)));

    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_EXIT_SNIFF_TRAN_MODE);
    MINT_OS_EXIT_CRITICAL();

    //LMP_LOG_INFO(LOG_LEVEL_HIGH, "0x8c programmed.");

    return;
#endif
}

#ifdef DETECT_SNIFF_OVERLAP
void lc_check_for_overlapping_sniff_windows(void)
{
    return;
}
#endif

#ifndef _REDUCE_LPS_AFTER_RTL8703B_
UINT8 lc_check_link_idle_state(void)
{
    UINT8 i;

    if ((lmp_self_device_data.lc_cur_dev_state != LC_CUR_STATE_IDLE) ||
#ifndef _CCH_LPS_
        (lmp_self_device_data.scan_enable != 0))
#else
        ( (lmp_self_device_data.scan_enable != 0) &&
        (!( g_efuse_lps_setting_2.timer2_mode_sel &&
        g_efuse_lps_setting_2.timer2_scan_en))))
#endif
    {
        return FALSE;
    }

#ifndef _CCH_LPS_
    for (i = 0; i < LMP_MAX_CONN_HANDLES; i++)
#else
    for (i = 0; i < LMP_MAX_CE_DATABASE_ENTRIES; i++)
#endif
    {
        if (lmp_connection_entity[i].ce_status != LMP_STANDBY)
        {
            return FALSE;
        }
    }

#ifdef LE_MODE_EN
    if (IS_BT40)
    {
        if (ll_manager.adv_unit.enable ||
            ll_manager.scan_unit.enable ||
            ll_manager.initiator_unit.enable ||
            ll_manager.conn_unit.enable ||
            (ll_manager.conn_unit.bmActiveHandle != 0))
        {
            return FALSE;
        }
    }
#endif

    return TRUE;
}
#endif

/* Set 2M/3M Tx Power with LUT register - add by austin */
void lc_set_2m_tx_power (UINT8 lut_idx,UINT8 txgain_idx)
{
    UINT16 temp_var;
    UINT16 reg_var;
    UINT32 shift_left = lut_idx;

    if (shift_left > 11)
    {
        return;
    }
    if (shift_left < 5)
    {
        reg_var = TX_EDR_GAIN0_REG;
    }
    else if (shift_left < 10)
    {
        shift_left -= 5;
        reg_var = TX_EDR_GAIN1_REG;
    }
    else
    {
        shift_left -= 10;
        reg_var = TX_EDR_GAIN2_REG;
    }
    shift_left = shift_left * 3;
    temp_var = BB_read_baseband_register(reg_var);
    temp_var &= ~(0x07 << shift_left);
    temp_var |= (txgain_idx & 0x7) << shift_left;
    BB_write_baseband_register(reg_var, temp_var);
}

void lc_set_3m_tx_power (UINT8 lut_idx,UINT8 txgain_idx)
{
    UINT16 temp_var;
    UINT16 reg_var;
    UINT32 shift_left = lut_idx;

    if (shift_left > 11)
    {
        return;
    }
    if (shift_left < 3)
    {
        shift_left += 2;
        reg_var = TX_EDR_GAIN2_REG;
    }
    else if (shift_left < 8)
    {
        shift_left -= 3;
        reg_var = TX_EDR_GAIN3_REG;
    }
    else
    {
        shift_left -= 8;
        reg_var = TX_EDR_GAIN4_REG;
    }
    shift_left = shift_left * 3;
    temp_var = BB_read_baseband_register(reg_var);
    temp_var &= ~(0x07 << shift_left);
    temp_var |= (txgain_idx & 0x7) << shift_left;
    BB_write_baseband_register(reg_var, temp_var);
}

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
void dlps_save_io_mem_to_partial_on(void)
{
#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8703B_RCP_
    if (rcp_dlps_save_io_mem_to_partial_on != NULL)
    {
        rcp_dlps_save_io_mem_to_partial_on();
        return;
    }
#endif
#endif


    if(sleep_mode_param.run_dlps_flow)
    {
        dlps_save_mem();

        // Bluewiz and modem HW backup
        dlps_save_bluewiz_and_modem();
        // Bluewiz Register
        dlps_save_bluewiz_req(dlps_backup_bluewiz_reg_buf, dlps_backup_le_reg_buf);

        // GPIO
        dlps_save_dwgpio_reg(dlps_backup_dwgpio_reg_buf);

        // Scae H5 Status
        UINT32 uart_status_d32 = (UINT32)UART_DWORD_READ(DMA_UART_H5_INTSTS);
        SIGN_PATCH_UART_STATUS(uart_status_d32);
    }
}

void dlps_restore_io_mem_from_partial_on(void)
{
#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8703B_RCP_
    if (rcp_dlps_restore_io_mem_from_partial_on != NULL)
    {
        rcp_dlps_restore_io_mem_from_partial_on();
        return;
    }
#endif
#endif

    sleep_mode_param.dlps_restore_flow = TRUE;

    dlps_restore_main(); // Restore DWGPIO here
    dlps_restore_mem();

}

void dlps_restore_io_mem_from_partial_on_after_clk_rdy(void)
{

#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8703B_RCP_
    if (rcp_dlps_restore_io_mem_from_partial_on_after_clk_rdy != NULL)
    {
        rcp_dlps_restore_io_mem_from_partial_on_after_clk_rdy();
        return;
    }
#endif
#endif

    if(sleep_mode_param.dlps_restore_flow)
    {
    
        dlps_restore_modem();

        dlps_lc_init();
        ll_hw_init(TRUE);

        // Bluewiz and modem HW restore
        dlps_restore_bluewiz();

        // Bluewiz Register
        dlps_restore_bluewiz_req(dlps_backup_bluewiz_reg_buf, dlps_backup_le_reg_buf);

        // Set Legacy Seqn
        dlps_restore_legacy_seqn();

        //hci_td_init();
        //---platform HW timer init---
        pf_timer_init(TRUE);

        //--- bzdma module init ---
        bzdma_init();

#ifndef IS_BTSOC
        //---enable interrupts---
        Rlx4181EnableInterrupt();
#endif
    }

    sleep_mode_param.dlps_restore_flow = FALSE;
    lmp_connection_entity_store_index = 0xFF;

}

void dlps_restore_main(void)
{

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

#ifdef _LPS_LOG_EN_
    LPS_DBG_LOG(BLUE, LPS_LOG_051, 0, 0);
#endif

{   //---bluetooth modules init---
    //bz_bluetooth_init(); /* pf_hci_transport_init() must before GpioInit() and power_init()*/

    //hci_td_init();
    {

        //TD_INF(log_file,"Initializing Transport Driver \n");
        TD_INF(INITIALIZING_TRANSPORT_DRIVER,0,0);

        RxPktFunc = hci_td_handle_received_packet;
        TxCompFunc = hci_td_handle_tx_completed;
        pf_hci_transport_write = S3C2410Usb_Dma;
    	pf_tx_fifo_order_free = dma_tx_fifo_order_free;
        pf_switch_hci_dma_parameter = switch_hci_dma_setting;
        pf_interface_wake_up_interrupt = wake_up_interrupt_switch;
        pf_enable_usb_interface_sco_filter = enable_filter_sco_connection;

        pf_hci_transport_init();

#ifdef _8821A_BTON_DESIGN_
        /* fast_gpio_power_on_check option is to be tested */
        EFUSE_POW_SETTING_4_S efuse_pow_setting_4;
        *(UINT8*)&efuse_pow_setting_4 = otp_str_data.efuse_pow_setting_4_d8;
        if (efuse_pow_setting_4.fast_gpio_power_on_check2b==1)
        {
            dlps_restore_dwgpio_reg(dlps_backup_dwgpio_reg_buf);
            power_init();
        }
#endif

        //TD_INF(log_file,"Transport Driver Initialized Successfully");
	    TD_INF(TRANSPORT_DRIVER_INITIALIZED_SUCCESSFULLY,0,0);
	}

        /* To enable external bb signals through GPIO */
#ifdef POWER_SAVE_FEATURE
        pf_clear_dsm_exit();
#endif
}

    //---GPIO init for radio_switch---
#if defined(_GPIO_POWER_SEQUENCE_ENABLE)
#ifdef _8821A_BTON_DESIGN_
    EFUSE_POW_SETTING_4_S efuse_pow_setting_4;
    *(UINT8*)&efuse_pow_setting_4 = otp_str_data.efuse_pow_setting_4_d8;
    if (efuse_pow_setting_4.fast_gpio_power_on_check2b==0)
#endif
    {
        dlps_restore_dwgpio_reg(dlps_backup_dwgpio_reg_buf);
    }
#endif//RADIO_SWITCH_SUPPORT


#ifdef _NEW_PLATFORM_XB_INTR_EN
    {
        UINT8 temp = VENDOR_BYTE_READ(0x37);
        VENDOR_BYTE_WRITE(0x37, temp | BIT0);
    }
#endif


#ifdef _ENABLE_MAILBOX_
    // mailbox_init();
    //---Create Mailbox task---
    if(OS_CREATE_TASK( &mailbox_handle, MAILBOX_TASK_NAME,
                   MAILBOX_TASK_PRI,
                   (OS_TASK_FUNCTION)mailbox_task,
                   MAILBOX_TASK_Q_SIZE,
                   NAILBOX_TASK_BUSY_PERIOD) != BT_ERROR_OK)
    {
    }
#endif

    if (g_fun_interface_info.b.bt_interface == UART_INTERFACE)
    {
        {
            dlps_restore_uart();
        }
    }


#ifdef _ENABLE_BTON_POWER_SAVING_
#ifdef _8821A_BTON_DESIGN_
    if (efuse_pow_setting_4.fast_gpio_power_on_check2b==0)
#endif
    {
        power_init(); /* must after GpioInit() */
    }
#endif

// #define DBG_MSG_EXPIRE_TIME     10000
//    dbg_msg_trigger_one_shot_event(1, 10000);

    if (IS_ROM_CODE_PATCHED)
    {
        if (IS_FW_TRIG_WDG_TO)
        {
            if (IS_SEND_PATCH_END)
            {
                ERASE_PATCH_END_EVENT_SIGNATURE;
            }
            ERASE_FW_TRIG_WDG_TO_SIGNATURE;
        }
    }
    if (IS_H5_LINKRESET)
    {
        ERASE_H5_LINKRESET_SIGNATURE;
    }

#ifdef _SUPPORT_AUTO_DETACH_LINK_
    memset(&catch_os_signal, 0, sizeof(OS_SIGNAL));
#endif


}

void dlps_save_bluewiz_and_modem(void)
{
    BTON_BT_RETENTION_REG_TYPE retention_reg;

    retention_reg.modem_retention = 0xc3;
    retention_reg.bluewiz_retention = 0xc3;
    VENDOR_WRITE(BTON_BT_RETENTION_REG, retention_reg.d32);


    UINT16 loop_ind = 0;
    while(1)
    {
        retention_reg.d32 = VENDOR_READ(BTON_BT_RETENTION_REG);
        if( ((retention_reg.d32&BIT0) == 0)&&((retention_reg.d32&BIT16) == 0) )
        {
            break;
        }

        if (loop_ind > 2000)
        {
            RT_BT_LOG(GREEN, YL_DBG_HEX_2, 2, 1, loop_ind);

            retention_reg.bluewiz_retention = 0xa5;
            retention_reg.modem_retention = 0xa5;
            VENDOR_WRITE(BTON_BT_RETENTION_REG, retention_reg.d32);

            break;
        }
	    loop_ind ++;
    }
//    RT_BT_LOG(GREEN, YL_DBG_HEX_2, 2, 1, loop_ind);

}

void dlps_restore_modem(void)
{
    BTON_BT_RETENTION_REG_TYPE retention_reg;

    retention_reg.modem_retention = 0x3c;
    VENDOR_WRITE(BTON_BT_RETENTION_REG, retention_reg.d32);


    UINT16 loop_ind = 0;
    while(1)
    {
        retention_reg.d32 = VENDOR_READ(BTON_BT_RETENTION_REG);
        if((retention_reg.d32&BIT0) == 0)
        {
            break;
        }

        if (loop_ind > 2000)
        {
            RT_BT_LOG(GREEN, YL_DBG_HEX_2, 2, 1, loop_ind);

            retention_reg.modem_retention = 0xa5;

            VENDOR_WRITE(BTON_BT_RETENTION_REG, retention_reg.d32);

            break;
        }
	    loop_ind ++;
    }
//    RT_BT_LOG(GREEN, YL_DBG_HEX_2, 2, 2, loop_ind);
}


void dlps_restore_bluewiz(void)
{
    BTON_BT_RETENTION_REG_TYPE retention_reg;

    retention_reg.bluewiz_retention = 0x3c;
    VENDOR_WRITE(BTON_BT_RETENTION_REG, retention_reg.d32);


    UINT16 loop_ind = 0;
    while(1)
    {
        retention_reg.d32 = VENDOR_READ(BTON_BT_RETENTION_REG);
        if((retention_reg.d32&BIT17) == 0)
        {
            break;
        }

        if (loop_ind > 2000)
        {
            RT_BT_LOG(GREEN, YL_DBG_HEX_2, 2, 1, loop_ind);

            retention_reg.bluewiz_retention = 0xa5;

            VENDOR_WRITE(BTON_BT_RETENTION_REG, retention_reg.d32);

            break;
        }
	    loop_ind ++;
    }
//    RT_BT_LOG(GREEN, YL_DBG_HEX_2, 2, 2, loop_ind);
}



void dlps_save_mem(void)
{
#ifdef _ENABLE_RETENTION_FLOW_FOR_DLPS_
    // lmp_connection_entity and bz_auth_link_params
    if(lmp_self_device_data.number_of_hlc > 0)
    {
        UINT8 ce_index;
        LMP_CONNECTION_ENTITY *ce_ptr, *ce_ptr_0;

	    ce_ptr_0 = &lmp_connection_entity[0];
        for(ce_index = 0; ce_index < LMP_MAX_CE_DATABASE_ENTRIES; ce_index++)
        {
            ce_ptr = &lmp_connection_entity[ce_index];

            if(ce_ptr->entity_status == ASSIGNED)
            {
                lmp_connection_entity_store_index = ce_index;
                memcpy(&lmp_connection_entity_bton, (void *)ce_ptr, sizeof(LMP_CONNECTION_ENTITY));
                if(ce_index > 0)
                {
                    memcpy( (void *)ce_ptr_0->auth, (void *)ce_ptr->auth, sizeof(BZ_AUTH_LINK_PARAMS));
                }
            }
        }
    }
    else
    {
        lmp_connection_entity_store_index = 0xFF;
    }
#endif

}

void dlps_restore_mem(void)
{
#ifdef _ENABLE_RETENTION_FLOW_FOR_DLPS_
    // lmp_connection_entity and bz_auth_link_params and mpal_manager
    UINT16 ce_index;
    for (ce_index = 0; ce_index < LMP_MAX_CE_DATABASE_ENTRIES; ce_index++)
    {
        lmp_init_connection_entity_on_power_on(ce_index);
    }

    if((lmp_self_device_data.number_of_hlc > 0)&&(lmp_connection_entity_store_index < LMP_MAX_CE_DATABASE_ENTRIES))
    {
        ce_index = lmp_connection_entity_store_index;
        LMP_CONNECTION_ENTITY *ce_ptr, *ce_ptr_0;

        ce_ptr = &lmp_connection_entity[ce_index];
        ce_ptr_0 = &lmp_connection_entity[0];

        memcpy((void *)ce_ptr, &lmp_connection_entity_bton, sizeof(LMP_CONNECTION_ENTITY));
        if(ce_index > 0)
        {
            memcpy((void *)ce_ptr->auth, (void *)ce_ptr_0->auth, sizeof(BZ_AUTH_LINK_PARAMS));
        }
    }

    for (ce_index = 0; ce_index < LMP_MAX_CE_DATABASE_ENTRIES; ce_index++)
    {
        if(ce_index != lmp_connection_entity_store_index)
        {
            bz_auth_init_linkparams(ce_index);
        }
    }

    memset(&mpal_manager, 0, sizeof(mpal_manager));
#endif

}

void dlps_save_bluewiz_req(UINT16 *buf, UINT16 *buf_le)
{
    UINT8 i;
    for (i = 0; i < DLPS_BACKUP_BLUEWIZ_REG_SIZE; i++)
    {
        buf[i] = BB_read_baseband_register(dlps_backup_bluewiz_reg_addr[i]);
    }

    for (i = 0; i < DLPS_BACKUP_LE_REG_SIZE; i++)
    {
        buf_le[i] = RD_LE_REG(dlps_backup_le_reg_addr[i]);
    }

}


void dlps_restore_bluewiz_req(UINT16 *buf, UINT16 *buf_le)
{
    UINT8 i;
    for (i = 0; i < DLPS_BACKUP_BLUEWIZ_REG_SIZE; i++)
    {
        BB_write_baseband_register(dlps_backup_bluewiz_reg_addr[i], buf[i]);
    }

    for (i = 0; i < DLPS_BACKUP_LE_REG_SIZE; i++)
    {
        WR_LE_REG(dlps_backup_le_reg_addr[i], buf_le[i]);
    }

    UINT16 ce_index;
    if((lmp_self_device_data.number_of_hlc > 0)&&(lmp_connection_entity_store_index < LMP_MAX_CE_DATABASE_ENTRIES))
    {
        ce_index = lmp_connection_entity_store_index;
        LMP_CONNECTION_ENTITY *ce_ptr;

        ce_ptr = &lmp_connection_entity[ce_index];
        BB_encryption_control(ce_ptr->am_addr, ce_ptr->phy_piconet_id, ce_ptr->enc_opcode);
    }

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_LE_
    LL_CONN_HANDLE_UNIT *pHandle;
    UINT8 entry;

    entry = ll_manager.conn_unit.chm_updt_entry;
    if ((entry != LL_MAX_CONNECTION_UNITS)&&
        (ll_manager.conn_unit.handle[entry].connected))
    {

        LL_CH_MAP_BLK *pChMap;

        pHandle = &ll_manager.conn_unit.handle[entry];
        pChMap = &ll_manager.conn_unit.handle[entry].ch_map_blk;

        /* TODO: fire channel update command and further handling */
    	if(ll_manager.conn_unit.master)
        {
            ll_driver_channel_map_update(entry, pHandle->ch_map_blk.instant);
        }
        else
        {
            ll_driver_channel_map_update(entry, pChMap->instant);
        }

    }
    entry = ll_manager.conn_unit.conn_updt_entry;
    if ((entry  != LL_MAX_CONNECTION_UNITS)&&
        (ll_manager.conn_unit.handle[entry].connected))
    {
        LL_CONN_UPDT_BLK *pConnUpdt;

        pHandle = &ll_manager.conn_unit.handle[entry];
        pConnUpdt = &ll_manager.conn_unit.handle[entry].conn_updt_blk;

        if(ll_manager.conn_unit.master)
        {
            ll_driver_connection_update(entry, pHandle->conn_updt_blk.instant);
        }
        else
        {
            ll_driver_connection_update(entry, pConnUpdt->instant);
        }
    }


#endif

}


void dlps_save_dwgpio_reg(UINT32 *buf)
{
    UINT8 i;
    for (i = 0; i < DLPS_BACKUP_DWGPIO_REG_SIZE; i++)
    {
        buf[i] = GPIO_READ(dlps_backup_dwgpio_reg_addr[i]);
    }
}

void dlps_restore_dwgpio_reg(UINT32 *buf)
{
    UINT8 i;
    for (i = 0; i < DLPS_BACKUP_DWGPIO_REG_SIZE; i++)
    {
        GPIO_WRITE(dlps_backup_dwgpio_reg_addr[i], buf[i]);
    }
}

void dlps_restore_legacy_seqn(void)
{

    if(lmp_self_device_data.number_of_hlc > 0)
    {
        UINT8 ce_index;
        LMP_CONNECTION_ENTITY *ce_ptr;

        for(ce_index = 0; ce_index < LMP_MAX_CE_DATABASE_ENTRIES; ce_index++)
        {
            ce_ptr = &lmp_connection_entity[ce_index];

            if(ce_ptr->entity_status == ASSIGNED)
            {

                LUT_EXTENSION_TABLE *ex_lut;

                UCHAR lut_index = lc_get_lut_index_from_ce_index(ce_index);

                if( lut_index == 0)
                {
                    return;
                }

                ex_lut = &lut_ex_table[lut_index];
                UINT16 reg_value;
                reg_value = BB_read_baseband_register(ex_lut->upper_lut_address);

                if((ce_ptr->dlps_last_seqn) == 0)
                {
                    //lc_init_seqn_scatternet((UCHAR), ex_lut, );

                    UINT16 temp_reg;

                    temp_reg = (UINT16)( ((ce_ptr->am_addr) << 5) | ((ce_ptr->phy_piconet_id) << 11) );

                    BB_write_baseband_register(CONNECTOR_REGISTER, temp_reg);

                    /* Make seqn bit in the baseband lut. */
                    temp_reg = BB_read_baseband_register(ex_lut->upper_lut_address);
                    temp_reg &= 0xfffd;
                    BB_write_baseband_register(ex_lut->upper_lut_address,temp_reg);

                    /* Issue opcode to initilize seqn in the baseband. */
                    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_SEQN_INIT);

                }
            }
        }
    }
}


void dlps_restore_uart(void)
{
    hci_uart_reset_init_RTL8723(1); /* for H4 */
}

#endif
