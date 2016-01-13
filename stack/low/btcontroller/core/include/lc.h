/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  Contains the LC module external API.
 */

/** \addtogroup lc_external
 *   @{ */
#ifndef _H_LC_
#define _H_LC_

#include "lc_internal.h"

#define LC_TX_POWER_LEVEL_INCREMENT                        0x00
#define LC_TX_POWER_LEVEL_DECREMENT                        0x01

#ifdef VER_3_0
#define RF_NUM_MODULATIONS             3

#endif

#define SCA_PICONET_ID_MASTER_PICONET                      0x00
#define SCA_PICONET_ID_SLAVE1_PICONET                      0x01
#define SCA_PICONET_ID_SLAVE2_PICONET                      0x02

#define SCA_PICONET_FIRST                                  0x00
#define SCA_PICONET_SECOND                                 0x01
#define SCA_PICONET_SECOND_SLAVE                           0x02
#define SCA_PICONET_THIRD                                  0x02
#define SCA_PICONET_FOURTH                                 0x03
#define SCA_PICONET_INVALID                                0xFF

#define SCA_SLAVE1_LOWER_LUT                              0x100
#define SCA_SLAVE1_UPPER_LUT                              0x102
#define SCA_SLAVE2_LOWER_LUT                              0x104
#define SCA_SLAVE2_UPPER_LUT                              0x106

#define LC_SCA_SLAVE_1_LUT                                 0x08
#define LC_SCA_SLAVE_2_LUT                                 0x09
#define LC_SCA_SLAVE_3_LUT                                 10
#define LC_SCA_SLAVE_4_LUT                                 11

#define SCA_PICONET_MASTER                                 0x01
#define SCA_PICONET_SLAVE                                  0x00

#define LC_CONT_POLL_SLOT_COUNT                            0x20

extern const UINT16 reg_PICONET_BD_ADDR1[4];
extern const UINT16 reg_PICONET_BD_ADDR2[4];
extern const UINT16 reg_PICONET_BD_ADDR3[4];
extern const UINT16 reg_PICONET_PARITY_BITS1[4];
extern const UINT16 reg_PICONET_PARITY_BITS2[4];
extern const UINT16 reg_PICONET_PARITY_BITS3[4];
extern const UINT16 reg_SCA_SLAVE_LOWER_LUT[4];
extern const UINT16 reg_SCA_SLAVE_UPPER_LUT[4];
extern const UINT16 reg_PICONET_INFO[4];
extern const UINT8 reg_MASTER_LOWER_LUT[8];
extern const UINT8 reg_MASTER_UPPER_LUT[8];
#ifdef _DAPE_SLV_SUPTO_IMPROVE
extern const UINT16 reg_SLAVE_TPOLL_MASK_REG[4];
#endif

#ifdef _BRUCE_TEST_PLC_PKT_STATUS
typedef struct PLC_PKT_STATUS_ {
UINT16 g_plc_tot_num;
UINT32 g_plc_pkt_mse;
UINT32 g_plc_pkt_mse_avg;
UINT16 g_plc_print_clk;
UINT16 g_plc_hec_err;
UINT16 g_plc_crc_err;  
UINT16 g_plc_pkt_miss;
UINT16 g_plc_HW_pkt_miss;

#ifdef _BRUCE_RECORD_ERASECNT_STATUS
UINT8  g_record_cnt;
UINT8  g_record_flag;
UINT8  g_burst_error_flag;
UINT16 g_plc_pkt_miss_erasecnt_2;
UINT16 g_plc_pkt_miss_erasecnt_3;
UINT16 g_plc_pkt_miss_erasecnt_5;
UINT16 g_plc_pkt_miss_burst_error; 
#endif

}PLC_CHECK_PKT_STATUS;
PLC_CHECK_PKT_STATUS plc_pkt_status_var;
#endif

extern UCHAR lc_pagescan_piconet_id;
extern UCHAR lc_paging_piconet_id;
extern void lc_retrieve_inq_scan(void);
extern void lc_retrieve_page_scan(void);
extern void lc_check_and_enable_scans_in_scatternet(void);

UCHAR lc_get_lut_index_from_phy_piconet_id(
	UCHAR am_addr, UCHAR phy_piconet_id);

UCHAR lc_get_lut_index_from_phy_piconet_id_for_disable_afh(
	UCHAR am_addr, UCHAR phy_piconet_id);

#ifdef COMPILE_PARK_MODE
#define INVALID_AR_ADDR                                    0xff
#endif /* COMPILE_PARK_MODE */
#define INVALID_PICONET_ID                                 0xff

#define LC_GET_PHASE_OFFSET_VALUE() \
    BB_read_baseband_register(PHASE_OFFSET_REGISTER);

#define LC_WRITE_PHASE_OFFSET_VALUE(value) \
    BB_write_baseband_register(PHASE_OFFSET_REGISTER, value );

#define LC_START_MS_SWITCH()\
    BB_write_baseband_register(INSTRUCTION_REGISTER,BB_MASTER_SLAVE_SWITCH);

#define LC_SET_TDD_BIT()\
    lc_set_tdd_bit();

#define LC_SET_BROADCAST_SCAN_WINDOW(new_Nbeacon)                              \
{                                                                              \
    BB_write_baseband_register_upper_octet(                                    \
        BEACON_PACKET_REPETETION_NO_REGISTER, new_Nbeacon);                    \
}

#ifdef _YL_LPS
typedef struct EFUSE_LPS_SETTING_1_S_ {
    UINT32 dsm_guard_interval:3;                    // [2:0] 3b, 7: 16; else: 2+x
    UINT32 sm_guard_interval:2;                     // [4:3] 2b, 3: 16; else: 2+x
    UINT32 min_sm_interval:2;                       // [6:5] 2b, 2^(x+1)
    UINT32 min_dsm_interval:2;                      // [8:7] 2b, 2^(x+1)
    UINT32 dsm_drift_scaling:2;                     // [10:9] 2b, DSM_DRIFT/2^x ????
    UINT32 delay625us_before_bton_lps_req:1;        // [11] 1b,  
    UINT32 new_tol_cal:1;                           // [12] 1b, 
    UINT32 sniff_tol_adjust:1;                      // [13] 1b
    UINT32 min_sniff_int_for_xtol:2;                // [15:14] 2b; 10*2^x slots
    UINT32 lps_task_mask:16;                        // [31:16] 16 bit; OS_TASK Mask 0x01ff   
} EFUSE_LPS_SETTING_1_S;

typedef struct EFUSE_LPS_SETTING_2_S_ {	
    UINT32 lps_pri:2; //g_efuse_lps_pri = 2;        // [1:0] 2b; 0: CH_AS_TASK_PRI, else: LC_RX_TASK_PRI   
    UINT32 timer2_lps_on:1;                         // [2] 1b; enable
    UINT32 sniff_lps_on:1;                          // [3] 1b; enable
     
    // Timer2 (DSM)
    UINT32 timer2_mode_sel:1;                       // [4] 1b; (0) orig (1) enable cch function
    UINT32 timer2_scan_en:1;                        // [5] 1b; enable
#ifdef _YL_RTL8723A_B_CUT
#ifdef _8821A_BTON_DESIGN_
    UINT32 lps_enable_hci_dma_wakeup_isr:1;         // [6] default: 0, 1: enable HCI DMA WAKEUP ISR when entering LPS, 0 is recommended
    UINT32 sniff_xtol_scaling:2;                    // [8:7] default: 0, if(SNIFF), xtol resolution = (4 >> x); RTL8821A: 1
    UINT32 sniff_xtol_scaling_for_park_hold:1;      // [9] default: 0, 1: also apply sniff_xtol_scaling to PARK/HOLD modes
#else
    UINT32 hci_reset_init_power_var:1;                                               // 1b; (default: 0);  1: initialize power-related variable at HCI_RESET
    UINT32 power_on_check_init_power_var:1;                                    // 1b; (default: 0);  1: initialize power-related variable at REBOOT gpio_power_on_check 
    UINT32 gpio_host_wake_bt_en:1;                                                  // 1b; (default: 0);  1: enable GPIO output
    UINT32 gpio_host_wake_bt_polarity:1;                                           // 1b; (default: 0); 1: active high, 0: active low
#endif
#else
    UINT32 timer2_scan_win:2;     //g_efuse_lps_timer2_scan_win = 2;           // 2b; ( 2^x ) 10ms
    UINT32 timer2_scan_stop:2;    //g_efuse_lps_timer2_scan_stop = 3;          // 2b; scan_interval << x
#endif    

#ifndef _MODI_LPS_AFTER_RTL8821B_TC_   
    UINT32 timer2_kill_scan_delay_ms:2; //g_efuse_lps_timer2_kill_scan_delay_ms = 1; 	// 2b; ( 2^(x+1) ) msec
#else
    UINT32 le_lps_enable:1;                         // [10] 1b; (0) disable (1) enable
    UINT32 adjust_lps_scan_interval:1;                // [11] 1b; (0) disable (1) enable
#endif

    UINT32 timer2_mode0_opt:1;                      // [12] 1b; (0) orig (1) chris timer2 with yilin function
	
    UINT32 sniff_mode_sel:1;                        // [13] 1b; (0) orig (1) enable cch function
    UINT32 sniff_scan_en:1;                         // [14] 1b; enable

#ifndef _MODI_LPS_AFTER_RTL8821B_TC_   
    
    // Sniff + SM
    UINT32 sm_scan_per:4;          //g_efuse_lps_sm_scan_per = 6;	               // 4b; ( sniff interval >>x) as period
    UINT32 sm_scan_per_min:2;   //g_efuse_lps_sm_scan_per_min = 2;	        // 2b; (2^x) period
    // Sniff + DSM
    UINT32 dsm_scan_per:4;        //g_efuse_lps_dsm_scan_per = 6;              // 4b; ( sniff interval >>x) as period
    UINT32 dsm_scan_per_min:2; //g_efuse_lps_dsm_scan_per_min = 2;       // 2b; (2^x) period
#else
    UINT32 scan_lps_min_period:4;                   // [18:15] 4b; default 2 (unit: 10msec)       
    UINT32 lc_sniff_sup_max:4;                      // [22:19] 4b; default 4 (unit: no receive packet times)
    UINT32 lps_nowakeup_when_no_scan:1;             // [23] 1b; enable
    UINT32 rsvd_24:3;                               // [26:24] 3b
#endif
	
    UINT32 force_exit_sm_en:1;                      // [27] 1b; enable

    UINT32 issc_nope_en:1;                          // [28] 1b; enable
    UINT32 issc_nope_num:2;                         // [30:29] 2b; return (x+1)
	
    UINT32 timer2_no_scan_wakeup_en:1;              // [31] 1b; timer2 with no scan enable need to wakeup enable
} EFUSE_LPS_SETTING_2_S;

typedef struct EFUSE_LPS_SETTING_3_S_ {
    UINT32 timer2_lps_isr_mask:16;        // [15:0] 16b; isr mask
    UINT32 timer2_reset_at_uart_isr_en:1; // [16]   1b;
    UINT32 sniff_lps_lmp_to:1;            // [17]   1b; enable
    UINT32 sniff_lps_sup_to:1;            // [18]   1b; enable

    UINT32 iot_sco_can_overlap:1;         // [19]   1b; sco slot can overlap to sniff

    UINT32 lc_init_radio_osc_delay_opt:1; // [20]   1b; 1(recommended): use EFUSE value instead of DEFINE
    UINT32 sniff_wakeup_xtol_opt:2;       // [22:21]   2b (0); 0: original, 1: wakeup_inst -= tol, 2(recommended): wakeup_inst -= (tol-tol_prev)   

    UINT32 iot_ralink_tid_no_check:1;     // [23]
    UINT32 iot_ralink_send_fea_req_later:1;// [24]

#ifdef _CCH_RTL8723A_B_CUT

#ifdef _HW_AUTO_MUTE_
    UINT32 hw_auto_mute_enable:1;        // [25] 1b;
    UINT32 hw_auto_mute_threshold:1;     // [26] 1b;
    UINT32 rsvd:1;                       // [27] 1b;
#else
// _CCH_8723_A_ECO_
    UINT32 iot_sco_noise_pcm_send0:1;        // 1b; 
    UINT32 iot_sco_noise_no_sco_count:2;     // 2b; (2<<(x<<2))no sco input  0 for per-packet
#endif    

    UINT32 iot_issc_inq_rssi:1;          // [28] 1b; enable   
    UINT32 iot_issc_rmr_par:1;           // [29] 1b; enable   
    UINT32 esco_nego_by_other:1;         // [30] 1b; enable   
    UINT32 use_ext_32k:1;                // [31] 1b
#else

    UINT32 rsvd:7; 
#endif

} EFUSE_LPS_SETTING_3_S;

#ifdef _CCH_RTL8723A_B_CUT
typedef struct EFUSE_LPS_SETTING_4_S_ {

// _CCH_ECO_LPS_
    UINT8 lps_chk_g_host_wake_bt:1;             // [0] 1: lps_chk_g_host_wake_bt = 0;                    // 1b;
    UINT8 lps_use_state:1;                      // [1] 1: lps_use_state = 0;                    // 1b;
    UINT8 whql_test_2sco:1;                     // [2] 1: whql_test_2sco = 0;                    // 1b;
    UINT8 lps_stop_afh_timer:1;                 // [3] 1: lps_stop_afh_timer = 0;                    // 1b;
#ifdef _LPS_FOR_8821_
    UINT8 lps_use_state_fast_lps:1;             // [4] 1: lps_use_state_fast_lps = 0;                    // 1b;
    UINT8 lps_use_intr:1;                       // [5] 1: lps_use_intr = 0;                    // 1b;   1: enable
#ifdef _8821A_BTON_DESIGN_    
    UINT8 lps_isr_mask_check_isr_switch:1;      // [6] 1: pow_ctrl_intr_handler(){ check isr_switch before checking lps_isr_mask and lps_mode_en = 0 } 
    UINT8 lps_pow_ctrl_intr_check_isr_switch:1; // [7] 1: save CPU resource by checking isr_switch before LPS-wakeup check
#else
    UINT8 rsvd:3; 
#endif    
#else
    UINT8 rsvd:4; 
#endif    
} EFUSE_LPS_SETTING_4_S;
#endif


#ifdef _8821A_BTON_DESIGN_
typedef struct EFUSE_LPS_SETTING_5_S_ {
#ifdef _2801_BTON_DESIGN_     
    UINT8 lps_check_ext_32k_exist:1;  // [0] 1: check enable  (from 8723b and 2801)
    UINT8 lps_use_new_cal:1;          // [1] 1: use new calibration (from 8723b and 2801)   
    UINT8 lps_use_new_cal_fw_mode:1;  // [2] 1: use new calibration fw manual mode(from 8723b and 2801)  
#ifdef LPS_NEW_CAL_MODE  
    UINT8 lps_use_new_cal_ini_by_old:1;//[3] 1: use new calibration but ini by old (from 8723b and 2801)
#else
    UINT8 rsvd:1;
#endif
#else
    UINT8 rsvd:4; 
#endif
    UINT8 disable_never_enter_lps_when_usb_active:1;    // [4]   1: disable_never_enter_lps_when_usb_active = 0; // 1b;   
    UINT8 lps_scan_protect_time:3;                      // [7:5] 3: lps_scan_protect_time = 0; // (x + 1)*10msec
} EFUSE_LPS_SETTING_5_S;
#endif


extern EFUSE_LPS_SETTING_1_S g_efuse_lps_setting_1;
extern EFUSE_LPS_SETTING_2_S g_efuse_lps_setting_2;
extern EFUSE_LPS_SETTING_3_S g_efuse_lps_setting_3;

#ifdef _CCH_RTL8723A_B_CUT
extern EFUSE_LPS_SETTING_4_S g_efuse_lps_setting_4;
#endif

#ifdef _8821A_BTON_DESIGN_
extern EFUSE_LPS_SETTING_5_S g_efuse_lps_setting_5;
#endif

#if defined(_CCH_SNIFF_NEG_TIMEOUT_) || defined(_CCH_RETENTION_FLOW_FOR_DLPS_)
extern EFUSE_RSVD_2_S g_efuse_rsvd_2;
#endif

#endif
#ifdef _YL_LPS_GPIO
#define lps_gpio_one_pull_high(index) gpio_one_pull_high(index)
#define lps_gpio_one_pull_low(index) gpio_one_pull_low(index)

#define LPS_GPIO_PGM_SNIFF_DSM 0
#define LPS_GPIO_EXECUTE_LPS 1
#define LPS_GPIO_EXIT_DSM 3 
#define LPS_GPIO_TX 4
#define LPS_GPIO_RX 5
#define LPS_GPIO_SNIFF 6
#define LPS_GPIO_DSM_PERIOD 7
#else
#define lps_gpio_one_pull_high(index) 
#define lps_gpio_one_pull_low(index) 

#define LPS_GPIO_PGM_SNIFF_DSM 0
#define LPS_GPIO_EXECUTE_LPS 1
#define LPS_GPIO_EXIT_DSM 3 
#define LPS_GPIO_TX 4
#define LPS_GPIO_RX 5
#define LPS_GPIO_SNIFF 6
#define LPS_GPIO_DSM_PERIOD 7
#endif

#ifdef COMPILE_PARK_MODE
extern UINT32 lc_slave_init_unpark_pending;
extern void lmp_handle_slave_unpark_req(UCHAR ar_addr, UCHAR piconet_id);
#endif

#ifdef COMPILE_PARK_MODE
extern UINT16 slave_ce_index;
extern UINT16 lc_num_unpark_req_trials;
extern UCHAR lc_start_of_beacon;
extern UINT16 lmp_unpark_ce_index;
#endif

#ifdef TEST_MODE
extern UCHAR lc_tci_pause_flag;
extern UINT16 lc_is_tx_test_mode;
extern UINT8 test_mode_sched_acl_pkt;
#endif

#ifdef COMPILE_ESCO
extern UCHAR lc_esco_window[8];
extern LMP_ESCO_CONNECTION_ENTITY 
       lmp_esco_connection_entity[LMP_MAX_ESCO_CONN_ENTITIES];        
#endif

#ifdef POWER_SAVE_FEATURE
extern UINT16 lc_power_ctr_config;
#endif

#ifdef COMPILE_ESCO
extern LMP_ESCO_CONNECTION_ENTITY 
       lmp_esco_connection_entity[LMP_MAX_ESCO_CONN_ENTITIES];
#endif

#ifdef COMPILE_AFH_HOP_KERNEL
extern OS_HANDLE hci_ch_as_task_handle;
#endif

#ifdef COMPILE_ESCO
extern POOL_ID synchronous_data_to_host_pool_id;
extern OS_HANDLE hci_event_task_handle;
extern UCHAR num_esco_links_over_codec;
extern UCHAR lmp_esco_over_codec;
#endif

/*
 * Pool handles  HOST ===> HOST CONTROLLER DIRECTION.
 */
extern POOL_ID             acl_data_pool_id;
#if defined(SCO_OVER_HCI) || defined(COMPILE_ESCO)
extern POOL_ID             synchronous_data_pool_id;
#endif /* SCO_OVER_HCI || COMPILE_ESCO */

/*
 * Pool handles  HOST CONTROLLER ===> HOST  DIRECTION.
 */

extern POOL_ID             acl_data_to_host_pool_id;
#if defined(SCO_OVER_HCI) || defined(COMPILE_ESCO)
extern POOL_ID             synchronous_data_to_host_pool_id;
#endif /* SCO_OVER_HCI || COMPILE_ESCO */

#ifdef _IS_ASIC_
extern INT8 g_rtl8723_btrf_txpower_track_n_old;
extern INT8 g_rtl8723_btrf_cfo_track_n_old;
extern INT8 g_rtl8723_btrf_rxgain_track_n_old;
extern UINT8 g_rtl8723_btrf_thermal_old;
#ifndef RTL8723A_B_CUT
extern INT8 g_rtl8723_btrf_lok_track_n_old;
#endif
extern TIMER_ID g_rtl8723_btrf_thermal_value_timer;
extern INT8 g_rtl8723_btrf_txpower_track_extra_delta;
#endif

#ifdef POWER_CONTROL
void lc_program_tx_power(UCHAR am_addr,UINT16 ce_index,UINT32 change_type,UINT16 tx_gain_value);
#endif

#ifdef ENABLE_SCO
extern UCHAR lc_full_bandwidth_flag;
extern UCHAR lc_acl_full_bandwidth_flag;
#endif

extern UCHAR lc_is_tpoll_started[MAX_NO_OF_LUT_INDICES];

UCHAR lc_handle_baseband_commands(HCI_CMD_PKT *cmd_buffer);

API_RESULT lc_handle_lmp_pdus(UINT16 ce_index, LMP_PDU_PKT *packet, 
                              UCHAR piconet_id);

API_RESULT lc_init(void);
#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
API_RESULT dlps_lc_init(void);
#endif

void lc_kill_inquiry(void);

#ifdef _CCH_PAGE_CON_
API_RESULT lc_kill_paging(UCHAR *bd_addr, UCHAR reason);
#endif	


API_RESULT lc_baseband_reset(void);
#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
API_RESULT dlps_lc_baseband_reset(void);
#endif

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
void lc_baseband_partial_reset(UINT8 dlps_flow);
#else
void lc_baseband_partial_reset(void);
#endif

void lc_module_reset(void);

API_RESULT lc_kill_sco_connection(UINT16 sco_ce_index);

API_RESULT lc_handle_connect_sco(UCHAR am_addr,UINT16 sco_ce_index,
								 UCHAR phy_piconet_id);

API_RESULT lc_start_sniff_mode(UINT16 ce_index, UCHAR ssr_flag);

API_RESULT lc_start_hold_mode(UINT16 ce_index);

API_RESULT lc_exit_sniff_mode(UINT16 ce_index);

void lc_get_clock_in_scatternet(UINT32 *native_clock, UCHAR phy_piconet_id);

#ifdef _CCH_RTL8723A_B_CUT
// _CCH_ECO_LPS_
#if 0
UINT32 lc_get_clock_by_pid(UCHAR phy_piconet_id);
#endif
#endif

void lc_get_high_dpi_native_clock(UINT32 *bb_clock_slot,
                                    UINT32 *bb_clock_us);

void lc_get_high_dpi_clock_in_scatternet(UINT32 * bb_clock_slot,
                                    UINT16 * bb_clock_us,UCHAR phy_piconet_id);

API_RESULT lc_handle_scan_mode_command(void);

void lc_handle_unpark(UINT16 ce_index);

API_RESULT lc_start_beacon(UINT16 ce_index);

void lc_kill_beacon(UINT16 ce_index);

void lc_exit_beacon(UINT16 ce_index);

void lc_unpark_req(UINT16 ce_index);

INLINE void lc_update_ce_index_to_lut_extn_tbl(UINT16 ce_index, UCHAR lut_index);

void lc_tx_task(OS_SIGNAL *signal_ptr);

#ifdef _FREE_FHS_PKT_WHEN_KILL_INQUIRY_
void lc_free_fhs_pkt_buffer(void);
#endif

#ifdef COMPILE_PERIODIC_INQUIRY
API_RESULT lc_handle_periodic_inquiry(UINT16 max_val , UINT16 min_val,
                                          UINT32 lap, UCHAR inq_len);

API_RESULT lc_kill_periodic_inquiry(void);
#endif /* COMPILE_PERIODIC_INQUIRY */

void lc_invoke_scheduler(UCHAR piconet_id);

extern void hci_generate_flush_occured_event(UINT16 conn_handle);

void lc_enable_ptt_bit(UINT16 ce_index);
void lc_disable_ptt_bit(UINT16 ce_index);

extern void lmp_select_unpark_device(void);

UCHAR lc_sca_get_piconet_role(UCHAR phy_piconet_id);

extern void lc_check_and_update_pkt_in_lut(UINT16 old_pkt_type, 
										   UINT16 new_pkt_type, UINT16 address);

extern UCHAR lc_check_for_clock_wrap_around(UINT32 cur_clock, 
											UINT32 future_clock);

extern UINT16 lc_get_expect_transfer_time(UINT16 conn_handle, 
                                                  UINT8 retry_index, 
                                                  UINT8  pkt_type);

UINT16 lc_get_current_power_value(UCHAR am_addr, UCHAR phy_piconet_id);

UINT32 lc_get_nslots_to_first_instant(UINT16 Tinterval, UINT16 Dphase, 
        UCHAR tcf, UCHAR piconet_id, UINT16* nslots_to_instant);
UINT32 lc_spin_until_clk1_transition(UCHAR piconet_id);

UCHAR lc_allocate_piconet_id_for_mss(UINT16 ce_index);
UCHAR lc_allocate_lut_index_for_mss(UINT16 ce_index, 
									UCHAR new_piconet_id, UCHAR new_am_addr);
UCHAR lc_allocate_am_addr_for_mss(UINT16 ce_index, UCHAR new_piconet_id);
void lc_update_addresses_in_lut_ex_table_for_mss(void);
UCHAR lc_allocate_piconet_id_for_paging(void);
INLINE UCHAR lc_get_no_of_piconets_connected(void);
INLINE void lc_set_lc_cur_device_state(UCHAR new_state);
UCHAR lc_check_if_device_is_in_scatternet(void);
void lc_program_exit_sniff_transition_mode(UINT16 ce_index);
void lc_program_kill_sniff_transition_mode(UINT16 ce_index);
#ifdef COMPILE_PARK_MODE
void lc_park_setup_lc_level_conn_during_unpark_procedure();
INLINE void lc_park_clear_lc_level_conn_during_unpark_procedure(UCHAR lut_index);
#endif

void lc_program_power_level_in_lut_for_scatternet(UCHAR am_addr, 
                               UINT16 tx_gain_value, UINT16 ce_index);

void lc_start_tpoll_on_all_connections(void);
void lc_stop_tpoll_on_all_connections_except(UINT32 ce_index);

#if defined(_CCH_RTL8723A_B_CUT) && defined(_CCH_LPS_USING_STATE_)
void lc_check_lps_for_resume(void);
UINT8 lc_check_lps_for_idle(UCHAR with_scan);
UINT8 lc_check_lps_for_link(UCHAR is_LEGACY, UCHAR is_LE, UCHAR not_sure);
UINT8 lc_check_lps_task_queue();
UINT8 lc_lps_double_check_dsm_cond(UCHAR lps_mode);
#endif

#ifdef ENABLE_PCM_ONLY_WHEN_SCO_LINK_CREATED
void lc_pcm_enable_control(UINT8 enable);
#endif

void lc_handle_and_check_sw_scheduler(UINT8 phy_piconet_id);

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
void dlps_save_io_mem_to_partial_on();
void dlps_restore_io_mem_from_partial_on();
void dlps_restore_io_mem_from_partial_on_after_clk_rdy();

void dlps_restore_main();
void dlps_save_bluewiz_and_modem(void);
void dlps_restore_modem(void);
void dlps_restore_bluewiz(void);
void dlps_save_mem(void);
void dlps_restore_mem(void);
void dlps_save_bluewiz_req(UINT16 *buf, UINT16 *buf_le);
void dlps_restore_bluewiz_req(UINT16 *buf, UINT16 *buf_le);
void dlps_save_dwgpio_reg(UINT32 *buf);
void dlps_restore_dwgpio_reg(UINT32 *buf);
void dlps_restore_legacy_seqn(void);
void dlps_restore_uart(void);
#endif

#endif /* _H_LC_ */

/** @} end: lc_external */

