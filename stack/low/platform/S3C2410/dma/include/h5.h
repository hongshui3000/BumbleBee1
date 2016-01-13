/***************************************************************************
 Copyright (C) RealTek Ltd.
 This module is a confidential and proprietary property of RealTek and
 a possession or use of this module requires written permission of RealTek.
 ***************************************************************************/

#ifndef __H5_H__
#define __H5_H__

#ifdef _RTK8723_UART_INIT_
#define DMA_UART_DLH_OFF                                        0x04
#define DMA_UART_DLL_OFF                                         0x00
#define DMA_UART_LINE_CTL_REG_OFF                       0x0c // struct: DMA_UART_LINE_CTL_REG_S
#ifndef _UART_H5
#define DMA_UART_LINE_CTL_REG_DLAB0                   0x03
#define DMA_UART_LINE_CTL_REG_DLAB1                   0x83
#endif
#ifdef _8821A_NEW_UART_DESIGN_
#define DMA_UART_REG1C                                  0x1c // struct: DMA_UART_REG1C_S
#endif
#endif

#ifdef _UART_H5
#define DMA_UART_H5_CTL0                                        0x24 // struct: DMA_UART_H5_CTL0_REG_S
#define DMA_UART_H5_CTL1                                        0x28 // struct: DMA_UART_H5_CTL1_REG_S
#define DMA_UART_H5_INTSTS                                    0x2c // struct: DMA_UART_H5_INTSTS_REG_S
#define DMA_UART_H5_INTEN                                      0x30 // struct: DMA_UART_H5_INTEN_REG_S
#ifdef _UART_BAUD_ESTIMATE_
#define DMA_UART_TOGGLE_MON_CTRL                            0x34 // struct: DMA_UART_TOGGLE_MON_CTRL_REG_S
#endif

#ifdef LE_MODE_EN 
    #define DMA_ISR_H5_TRX_ACTIVE_CHECK_MASK0 0x5B // refer to  ISR_STATUS_TYPE; to check if HCI TRX is active (receive valid packet or acked)
    #define DMA_ISR_H5_TRX_ACTIVE_CHECK_MASK1 0x5F // refer to  ISR_STATUS_TYPE; to check if HCI TRX is active (receive valid packet or acked)
    #define DMA_ISR_H5_GO_SLEEP_CHECK_MASK 0x7F // refer to ISR_STATUS_TYPE; to check if HCI TRX is ready to sleep or not (any pending TX or Unexpected RX status)
#else
    #define DMA_ISR_H5_TRX_ACTIVE_CHECK_MASK0 0x1B
    #define DMA_ISR_H5_TRX_ACTIVE_CHECK_MASK1 0x1F
    #define DMA_ISR_H5_GO_SLEEP_CHECK_MASK 0x3F
#endif
#endif
#ifdef _RTK8723_UART_INIT_
#ifdef _8821A_NEW_UART_DESIGN_
typedef union DMA_UART_REG1C_S{
    UINT32 d32;
    struct
    {
        UINT32 d16              :16; //bit0~15, used for other functions
        UINT32 ovsr_adj         :11; //bit16~26
        UINT32 rsvd             :5; //bit27~31
    }b;
}DMA_UART_REG1C_S_TYPE;
#endif
#endif

#ifdef _UART_H5
#define H5_STATE_UNINIT 0
#define H5_STATE_INIT 1
#define H5_STATE_ACTIVE 3
typedef union H5_HW_STATE_TYPE_{
    UINT16 d16;
    struct
    {        
        UINT16 tx_seq_num           :3; //bit0~2
        UINT16 tx_ack_num           :3; //bit3~5
        UINT16 ln_state             :2; //bit6~7
        UINT16 sliding_wsize        :3; //bit8~10
        UINT16 oof_fctrl            :1; //bit11
        UINT16 data_chec_en         :1; //bit12
        UINT16 reserved_13_15       :3; //bit13~15
    }b;
}H5_HW_STATE_TYPE;
typedef union H5_BTON_TYPE_{
    UINT16 d16;
    struct
    {        
        UINT16 h5_hw_state_d16      :13; //bit0~12
        UINT16 h5_sleep_msg_state   :1; //bit13
        UINT16 reserved_14_15       :2; //bit14~15
    }b;
}H5_BTON_TYPE;

typedef union DMA_UART_LINE_CTL_REG_S{
    UINT8 d8;
    struct
    {
        UINT8 word_len               :2; //bit0~1
        UINT8 stop_num               :1; //bit2
        UINT8 parity_en              :1; //bit3
        UINT8 parity_even            :1; //bit4
        UINT8 stick_parity           :1; //bit5
        UINT8 set_break              :1; //bit6
        UINT8 dlab                   :1; //bit7
    }b;
}DMA_UART_LINE_CTL_REG_S_TYPE;

typedef union DMA_UART_H5_CTL0_REG_S{
    UINT32 d32;
    struct
    {
        UINT32 active_sync           :1; //bit0
        UINT32 poll_wake             :1; //bit1
        UINT32 send_sleep            :1; //bit2
        UINT32 send_break            :1; //bit3
        UINT32 clr_unack_pkt         :1; //bit4
        UINT32 go_park               :1; //bit5
        UINT32 go_idle               :1; //bit6, FORCE the H5 Stop and Park (recommended instead of go_park)
        UINT32 force_no_oof          :1; //bit7
        UINT32 host_to_val           :2; //bit8~9
        UINT32 rsvd_10_31            :22; //bit10~31
    }b;
}DMA_UART_H5_CTL0_REG_S_TYPE;

typedef union DMA_UART_H5_CTL1_REG_S{
    UINT32 d32;
    struct
    {
        UINT32 wake_to_val             :2; //bit0~1
        UINT32 ack_to_val              :2; //bit2~3
        UINT32 sync_to_val             :3; //bit4~6
        UINT32 h5_en                   :1; //bit7
        UINT32 resend_to_val           :3; //bit8~10
        UINT32 retry_limit             :3; //bit11~13
        UINT32 force_oof_ctrl          :1; //bit14
        UINT32 ignore_sync             :1; //bit16
        UINT32 rsvd_16_31              :16; //bit16~31
    }b;
}DMA_UART_H5_CTL1_REG_S_TYPE;

typedef union DMA_UART_H5_INTSTS_REG_S{
    UINT32 d32;
    struct
    {
        UINT32 link_reset_int           :1; //bit0
        UINT32 rx_wakeup_int            :1; //bit1
        UINT32 rx_woken_int             :1; //bit2
        UINT32 rx_sleep_int             :1; //bit3
        UINT32 slip_err_int             :1; //bit4
        UINT32 poll_wake_int            :1; //bit5
        UINT32 retry_alarm_int          :1; //bit6
        UINT32 retry_fail_int           :1; //bit7
        UINT32 park_ok_int              :1; //bit8
        UINT32 link_est_int             :1; //bit9
#ifdef _UART_BAUD_ESTIMATE_
        UINT32 reserved_10_14           :5; //bit10~14
        UINT32 uart_falling_cnt_intr_sts:1; //bit15
#else
        UINT32 reserved_10_15           :6; //bit10~15
#endif        
        UINT32 h5_current_st_d16        :16; //bit16~31 refer to H5_HW_STATE_TYPE
    }b;
}DMA_UART_H5_INTSTS_REG_S_TYPE;

typedef union DMA_UART_H5_INTEN_REG_S{
    UINT32 d32;
    struct
    {
        UINT32 link_reset_int_en       :1; //bit0
        UINT32 rx_wakeup_int_en        :1; //bit1
        UINT32 rx_woken_int_en         :1; //bit2
        UINT32 rx_sleep_int_en         :1; //bit3
        UINT32 slip_err_int_en         :1; //bit4
        UINT32 poll_wake_int_en        :1; //bit5
        UINT32 retry_alarm_int_en      :1; //bit6
        UINT32 retry_fail_int_en       :1; //bit7
        UINT32 park_ok_int_en          :1; //bit8
        UINT32 link_est_int_en         :1; //bit9
#ifdef _UART_BAUD_ESTIMATE_
        UINT32 reserved_10_14          :5; //bit10~14
        UINT32 uart_falling_cnt_intr_en:1; //bit15
#else
        UINT32 reserved_10_15          :6; //bit10~15
#endif        
        UINT32 h5_backup_st_d16        :16; //bit16~31 refer to H5_HW_STATE_TYPE
    }b;
}DMA_UART_H5_INTEN_REG_S_TYPE;

#ifdef _UART_BAUD_ESTIMATE_
typedef union DMA_UART_TOGGLE_MON_CTRL_REG_S{
    UINT32 d32;
    struct
    {
        UINT32 uart_min_falling_space:12;   //[11:0]
        UINT32 uart_min_low_period:12;      //[23:12]
        UINT32 uart_falling_cnt_th:6;       //[29:24]
        UINT32 mon_data_valid:1;            //[30] cleared at mon_en rising edge, set as 1 after the first falling space cnt update
        UINT32 uart_toggle_mon_en:1;        //[31]
    }b;
}DMA_UART_TOGGLE_MON_CTRL_REG_S_TYPE;
#endif

#endif

#ifdef _RTK8723_UART_INIT_
typedef union DMA_UART_SOFEWARE_RESET_REG_S{
    UINT8 d8;
    struct
    {
        UINT8 uart_hci_sel     :1; //bit0
        UINT8 uart_hci_en      :1; //bit1
        UINT8 reserved_2       :1; //bit2, reserved?
        UINT8 uart_hci_rst     :1; //bit3
        UINT8 pcm_en           :1; //bit4
        UINT8 reserved_5_7     :3; //bit5~7, reserved?
    }b;
}DMA_UART_SOFEWARE_RESET_REG_S_TYPE;


typedef union BAUDRATE_FORMAT_{
    UINT32 d32;
    struct
    {
#ifdef _8821A_NEW_UART_DESIGN_
        UINT32 d24:28;              //[27:0]
        UINT32 rsvd:4;              //[31:28]
#else
        UINT32 d24              :24; //bit0~23
        UINT32 rsvd8            :8; //bit24~31
#endif    
    }b;	
    struct
    {
        UINT32 uart_divisor:12;     //[11:0]
        UINT32 uart_ovsr:4;         //[15:12], Note: uart_ovsr = (real_ovsr - 5) !!!
#ifdef _8821A_NEW_UART_DESIGN_        
        UINT32 uart_ovsr_adj:11;    //[26:16], bit-wise ovsr adjustment {stop,par,d[7:0],start} 
        UINT32 uart_clk_sel:1;      //[27] 1: 120MHz (from BT_PLL), 0: 40MHz
        UINT32 rsvd:4;              //[31:28]
#else
        UINT32 uart_pll_d        :5; //bit16~20, 
        UINT32 uart_pll_sel      :2; //bit21~22, 0: DIV=4, 1: DIV=5, 2, DIV=9
        UINT32 uart_clk_sel      :1; //bit23
        UINT32 rsvd8             :8; //bit24~31
#endif
    }b2;	
}BAUDRATE_FORMAT_TYPE;
#ifdef _8821A_BTON_DESIGN_
#define HCI_UART_CLK_SEL_1_RATIO 3 // This define indicate the clock rate ratio of {uart_clk_sel = 1}/{uart_clk_sel = 0}
#endif
#endif


#ifdef _UART_H5
typedef struct HCI_UART_MAN_TYPE_ {
    union {
        struct {
            UINT32 h5_sleep_msg_state:1; //2 shall only be changed by hci_uart_h5_set_sleep_msg_state()
                                                               //2 need to be saved to BTON; 1: SLEEP received/sent; 0: 
                                                               //2 need to be recovered from sleep state
            UINT32 h5_retry_alarm_state:1;
            UINT32 h5_retry_fail_state:1;
            UINT32 h4_err_event_wait_rx_state:1; // TODO: move to other place?
            UINT32 h4_err_event_wait_recov_timer_state:1; // TODO: move to other place?
            UINT32 baud_new_valid_flag:1; // TODO: move to other place?
            UINT32 chg_para_valid_flag:1;
            UINT32 reserved:25;
        };
        UINT32 d32;
    };
    UINT32 h5_trx_active_mask;
    BAUDRATE_FORMAT_TYPE baud_new; // TODO: move to other place?    
    //TIMER_ID h5_poll_wake_timer_id;
    UCHAR h4_hw_err_event_code; 
#ifdef _UART_BAUD_ESTIMATE_
    UINT8 baud_est_h5_state_record;
    UINT8 baud_sram_recov_en;                       // the flag to determine the action of FW UART BAUD ESTIMATE
#endif    
//    UINT8 h5_link_est_intablished; 
//    UINT8 btdma_triggered; 
//    UINT16 link_reset_cnt;
//    H5_HW_STATE_TYPE h5_state;
} HCI_UART_MAN_TYPE;

// TODO: To complete the coding
#ifdef _UART_BAUD_ESTIMATE_  // TODO: to be moved to SRAM
typedef struct HCI_UART_MAN_SRAM_TYPE_ {
    UINT8 hci_uart_baud_est_state;
    UINT8 hci_uart_baud_est_intr_cnt;
    UINT8 hci_vendor_uart_sync_received; // record if hci_vendor_uart_sync_received for FW UART BAUD ESTIMATE actions
    UINT8 hci_vendor_set_baud_received;  // 
    UINT8 hci_vendor_set_baud_executed;  // 
    UINT8 baud_est_original_baud_det_done;
    UINT16 est_min_low_period_record;
    UINT16 est_min_falling_space_record;
//    UINT32 hci_uart_man_sram_valid_signature;
    BAUDRATE_FORMAT_TYPE baud_current_setting;      // to record the Baudrate setting in SRAM, to allow recover after WDG Timeout
                                                    // may be used when FW UART BAUD ESTIMATE is enabled
} HCI_UART_MAN_SRAM_TYPE;
#define HCI_UART_MAN_SRAM_VALID_SIGNATURE   0x6A10983E
#define HCI_UART_BAUD_EST_ST_START          0                
#define HCI_UART_BAUD_EST_ST_TRIAL          1
#define HCI_UART_BAUD_EST_ST_STOP           2
#endif    

#endif

#ifdef _RTK8723_UART_INIT_

/* The Structure of RLT8723 HCI DATA UART Settings (size = 4 bytes) 
   (Default Settings: 0x19CAC558) */

typedef struct RLT8723_DATA_UART_SETTINGS_S_ {
    UINT32 chk_bton_en:1;          // bit[0], (0) 1: use bton/baudrate_record if bton/baudrate_record!=0
    UINT32 wr_bton_en:1;           // bit[1], (0) 1: enable write to bton/baudrate_record
    UINT32 baud_det_en:1;          // bit[2], (0) 1: enable baud rate detection funcation
    UINT32 baud_default_en:1;      // bit[3], (0) when goto_check_default: 1: use default(RTL8723:115200), 0: use efuse baudrate
#ifdef _8821A_BTON_DESIGN_ 
    UINT32 baud_mon_log_en:1;                       // bit[4], (1)
    UINT32 rsvd0:1;                                 // bit[5], (0)
    UINT32 uart_set_flow_control_after_hci_en:1;    // bit[6], (1) (1 is preferred) to keep the flow control setting as "STOP" before HCI MODE is ready, it is important to H4
    UINT32 rsvd1:1;                                 // bit[7], (0)
    UINT32 baud_est_baud_det_fail_redet_en:1;       // [8] (1)
    UINT32 baud_est_use_original_buad_det_only:1;   // [9] (0)
    UINT32 baud_est_use_original_buad_det_once:1;   // [10] (1)
    UINT32 baud_record_at_rx_vendor_sync:1;         // [11] (0)
    UINT32 hci_uart_mcr_rtsn:1;                     // [12] (0) valid when HW FLOW CONTROL OFF; Note: PIN RTSN = ~RTS
    UINT32 baud_est_baud_det_trial_redet_en:1;      // [13] (0)
    UINT32 baud_est_delay_before_h5_state_check:1;  //[14] (1) 
    UINT32 buad_est_bypass_mon_valid_check:1;       // [15] (1)
    UINT32 baud_record_at_h5_link_est:1;            // [16] (0)
    UINT32 baud_det_init_with_reset:1;              // [17], (1)
    UINT32 baud_est_wr_bton_en_at_h5link_uartsync:1;// [18], (0) 
    UINT32 h5_sign_linkrst_signature:1;             // [19] (1)  
    UINT32 h5_linkrst_sign_fw_trig_wdg:1;           // [20] (0) SIGN_FW_TRIG_WDD_TIMEOUT when H5 link_reset_int WDD Timeout
#else
    UINT32 baud_det_timeout:5;     // bit[8:4], (21) timeout_loops = 2^x, depends on host requirements
    UINT32 baud_det_timeout_en:1;  // bit[9], (0) depends on host requirements    UINT32 pll_ckrdy_timeout:5;    // bit[14:10], (17) timeout_loops = 2^x, ~ 10ms is desired
    UINT32 pll_ckrdy_timeout:5;    // bit[14:10], (17) timeout_loops = 2^x
    UINT32 pll_ckrdy_timeout_en:1; // bit[15], (1) should be 1 
    UINT32 pll_ldo_stable_loops:5; // bit[20:16], (10) timeout_loops = 2^x*10, 100us is preferred
#endif    
    UINT32 h5_set_g_host_state:1;  // bit[21], (0) 1: set g_host_state when setting sleep_msg_state (to enable HCI C2H gating function)
    UINT32 tune_pll_sel_en:1;      // bit[22], (1) 1: enable pll_sel tuning at SPI mode
#ifdef _8821A_BTON_DESIGN_   
    UINT32 h4_err_intr_en:1;                        // [23], (1) 1: enable h4 error interrupt
#else
    UINT32 afe_turn_off_en:1;      // bit[23], (1) 1: enable turn UART PLL/LDO off when clk_sel = 0 and spi_mode = 0
#endif    
    UINT32 en0_before_set_en:1;    // bit[24], (1) 1(recommended): disable uart before UART initialization setting in uart_set_baud_clk_RTL8723(); to avoid glitch effects
    UINT32 err_recov_en:1;         // bit[25], (0) 1b, 1: call hci_uart_reset_init_RTL8723(re_init_flag = 1) when uart error
    UINT32 baud_redet_en:1;        // bit[26], (0) valid when baud_det_en = 1, (when re_init_flag = 1) 1: detect baudrate 0: no re-detection
#ifdef _8821A_BTON_DESIGN_   
    UINT32 baud_est_stop_at_rx_vendor_sync:1;       // [27], (1)
    UINT32 baud_mon_resume_every_log:1;             // [28], (1)
    UINT32 uart_h5_go_sleep_backup_en:1;            // [29], (0) 1: enable h5 status backup when H5 go sleep
#else
    UINT32 det_wait_en:1;          // bit[27], (1, but 0 is preferred) (when re_init_flag = 0) 1: wait for detection results, 0: bypass the waiting loops
    UINT32 redet_wait_en:1;        // bit[28], (1, but 0 is preferred) (when re_init_flag = 1) 1: wait for detection results, 0: bypass the waiting loops; valid when g_hci_uart_baud_redet_en = 1
    UINT32 ck_timeout_afe_off_en:1;// bit[29], (0) 1: turn AFE off when wait PLL CKRDY timeout 
#endif    
    UINT32 err_event_err_code_opt:1; // bit[30], (0) 1b;1: error code = HARDWARE_FAILURE_ERROR_RTK_H4 to distinguish with other Hardware Error Codes
    UINT32 h5_linkfail_clr_unack:1;  // bit[31], (0) 1b; 1: clr_unack_pkt() when link_fail_int
} RLT8723_DATA_UART_SETTINGS_S;

#ifdef _UART_H5
typedef struct RLT8723_DATA_UART_SETTINGS_2_S_ {
    UINT32 h5_en:1;                       //bit[0], (0) 1b
    UINT32 h5_active_sync:1;              //bit[1], (0) 1b; 1: do not wait HOST SYNC message before sending the first SYNC in UNINITIALIZED STATE
    UINT32 h5_ignore_sync:1;              //bit[2], (1) 1b; 1: ignore SYNC at ACTIVE STATE
    UINT32 h5_force_oof_ctrl:1;           //bit[3], (0) 1b; force out-of-frame software flow control ON (XON/XOFF)
    UINT32 h5_force_no_oof:1;             //bit[4], (0) 1b; force out-of-frame software flow control OFF (XON/XOFF)
    UINT32 h5_retry_limit:3;              //bit[7:5], (7) 3b; LinkFailure: 2^(x+1) retries; LinkAlarm: 2^(x) retries
    UINT32 h5_resend_to_val:3;            //bit[10:8], (3) 3b; 0~5; resend(retry) period: 2^(x+9) T_char; 115200: T_char ~ 95.5us; 921600: T_char ~ 11.9us
    UINT32 h5_sync_to_val:3;              //bit[13:11], (3) 3b; 0~4; link establishment period; 2^(2x+6) T_char; 115200: T_char ~ 95.5us; 921600: T_char ~ 11.9us
    UINT32 h5_ack_to_val:2;               //bit[15:14], (0) 2b; 0~3; pure ACK wait time: 2^(2x+2) T_char; 115200: T_char ~ 95.5us; 921600: T_char ~ 11.9us
    UINT32 h5_wake_to_val:2;              //bit[17:16], (1) 2b; WAKEUP period;  2^(2x+2) T_char; 115200: T_char ~ 95.5us; 921600: T_char ~ 11.9us
    UINT32 h5_host_to_val:2;              //bit[19:18], (0) 2b; HCI TX abort timeout(to avoid hangup bugs); 2^(2x+10) characters; 3: never timeout
    UINT32 h5_int_en:10;                  //bit[29:20], (0x2ff) 10b;(BIT0|BIT1|BIT2|BIT3|BIT4|BIT5|BIT6|BIT7 |BIT9); // 16b; No PARK_OK_INT
//    UINT32 h5_linkreset_restart:1;      //bit[30], (0), 1b; valid when h5_ignore_sync = 0; 1: execute HCI RESET and DMA INIT when link_reset_int
    UINT32 h5_lowpow_sleep_msg_wait_en:1; //bit[30], (0), 1b; valid when h5_lowpow_sleep_msg_en = 1; 0: exit hci_uart_h5_send_sleep_msg_wait_done() without waiting done
    UINT32 h5_retry_state_clr_when_trx:1; //bit[31] (0), 1b; 1: clear h5_retry_alarm_state and h5_retry_fail_state when HCI DMA TRX is active    
} RLT8723_DATA_UART_SETTINGS_2_S;
typedef struct RLT8723_DATA_UART_SETTINGS_3_S_ {
    UINT32 parity_en:1;                    //bit[0], (0) 1b
    UINT32 parity_even:1;                  //bit[1], (0) 1b
    UINT32 hw_fctrl_on:1;                  //bit[2], (1) 1b        
    UINT32 long_break_duration:3;          //bit[5:3], (0) 3b; 2^(x)*10ms
    UINT32 h5_poll_wake_duration:3;        //bit[8:6], (5) 3b; 7: persistent poll; else: 2^(2x)*10ms
    UINT32 h5_retry_alarm_opt:2;           //bit[10:9], (0) 2b; 0: nothing, 1: sent short break, 2: send long break; 3: poll_wake
    UINT32 h5_retry_fail_opt:2;            //bit[12:11], (0) 2b; 0: nothing, 1: sent short break, 2: send long break; 3: poll_wake
    UINT32 h5_lowpow_wakeup_opt:2;         //bit[14:13], (0) 2b; // TODO:  if(sleep_msg_state==1 && RX is to be send to HCI DMA), 0: nothing, 1: sent short break, 2: send long break; 3: poll_wake
    UINT32 h5_lowpow_sleep_msg_en:1;       //bit[15], (0) 1b; 1: send_sleep_msg_and_wait_done() before go to sleep // TODO: need extra efuse for LPS mode?
    UINT32 h5_scounrel_force_en:1;         //bit[16], (0) 1b
    UINT32 h5_scounrel_force_value:1;      //bit[17], (0) 1b; valid when force_en = 1, 0: reliable, 1: unreliable   
    UINT32 h5_resend_time_adapt_en:1;      //bit[18], (1) 1b; enable retry_limit adaption according to baudrate and resend_target; Should not be enabled when "baud_det_en & (det_wait_en = 0 | det_timeout_en)" ==> Baudrate may be strange
    UINT32 h5_resend_target:2;             //bit[20:19], (0) 2b; valid when resend_time_adapt_en = 1; target resend timing for adaption = 250ms/2^x
    UINT32 h5_resend_time_adapt_max:3;     //bit[23:21], (5) 3b; the maximum adapted resend_to_val;
    UINT32 h5_resend_time_adapt_min:3;     //bit[26:24], (3) 3b; the minimum adapted resend_to_val
    UINT32 h5_retry_limit_adapt_en:1;      //bit[27], (1) 1b; valid when resend_time_adapt_en = 1; enable retry_limit adaption according baud and resend_to_val
    UINT32 h5_trx_active_mask_opt:1;       //bit[28], (0) 1b; to clear sleep_mst_state: 0: CMD/ACL/EVT, 1: CMD/ACL/EVT + SCO TX
    UINT32 err_send_event_delayw1c_opt:3;  //bit[31:29], (0) 3b; 0: no HW Err Event + N0 delayed W1C, 1: HW Err Event + N0 delayed W1C, else: HW Err Event + delayed W1C with delay = 2^(x)*2.5ms
} RLT8723_DATA_UART_SETTINGS_3_S;


#ifdef _UART_BAUD_ESTIMATE_
typedef union EFUSE_BAUD_EST_SETTINGS_1_S_{ 
    UINT16 d16;
    struct 
    {
        UINT16 baud_mon_en:1;                      // [0] (0)
        UINT16 baud_est_en:1;                      // [1] (0)
        UINT16 baud_recov_at_fw_trig_wdg:1;        // [2] (1)
        UINT16 baud_recov_at_h5_linkreset:1;       // [3] (1)
        UINT16 baud_recov_at_reinit:1;             // [4] (0) ???????
        UINT16 baud_recov_at_other_wdg:1;          // [5] (0) ??
        UINT16 baud_recov_at_state_stop_postset:1; // [6] (1)
//        UINT16 baud_est_en_at_fw_trig_wdg:1;     // [2]
        UINT16 baud_recov_at_state_stop_preset:1;  // [7] (0)
//        UINT16 baud_recov_after_set_baud:1;      // [7]
        UINT16 baud_est_restart_at_h5_linkreset:1; // [8] (0)
        // TODO: resolution requirement?
        UINT16 exec_baud_est_init_post_at_reinit:1;// [9] (0==>1)
        UINT16 baud_est_update_at_h5_initialized:1;      // [10] (1)
        UINT16 baud_est_restart_at_baud_sram_recov_0:1;  // [11] (1)
        UINT16 baud_est_ovsrx8_grid:2;             // [13:12] (0) ovsr search grid = 2^(x)/8; 1/4 or 1/8 is preferred
        UINT16 min_low_falling_udfl_th_opt:2;      // [15:14] (2)
    };
} EFUSE_BAUD_EST_SETTINGS_1_S;
typedef union EFUSE_BAUD_EST_SETTINGS_2_S_{ 
    UINT16 d16;
    struct 
    {
        UINT16 baud_est_falling_cnt_th_1st:4;      // [3:0] (3) uart_falling_cnt_th = (baud_est_toggle_th*4)
        UINT16 baud_est_combine_opt:2;             // [5:4] (1) 0: by min_low_period, 1: by min_falling_space, 2: by average, ....
        UINT16 baud_est_combine_opt_both_udfl:2;   // [7:6] (1) 
        UINT16 min_falling_ovfl_chg_to_min_low:1;  // [8] (1)
        UINT16 min_low_udfl_chg_to_min_falling:1;  // [9] (1)
        UINT16 ignore_invalid_falling_low_ratio:1; // [10] (0)
        UINT16 est_bias_value:3;                   // [13:11] (0)  Unit: 1/4 ovsr_div
        UINT16 est_bias_sign:1;                    // [14] (0) 0: add, 1: sub        
        UINT16 baud_det_finetune_by_exhaust_est:1; // [15] (0)
    };
} EFUSE_BAUD_EST_SETTINGS_2_S;
typedef union EFUSE_BAUD_EST_SETTINGS_3_S_{ 
    UINT16 d16;
    struct 
    {
        UINT16 det_allow_table:12;                 // [11:0] (0xFFF)
        UINT16 baud_est_ovsr_low_bound:3;          // [14:12] (4) Estimation OVSR Lower Bound = x+5
        UINT16 w1c_at_fallint_cnt_intr_end:1;      // [12] (1)
    };
} EFUSE_BAUD_EST_SETTINGS_3_S;
typedef union EFUSE_BAUD_EST_SETTINGS_4_S_{ 
    UINT16 d16;
    struct 
    {
        UINT16 baud_est_opt:2;                      // [1:0] (1) 0: exhaustive search, 1: detection, 2: detection, if err>(est/2^(1+x)) ==> exhaustive search
        UINT16 det_err_chg_to_est_th:3;             // [4:2] (3) valid when baud_est_opt = 2, if(err/ovsr_div_x8_est >= 1/2^x){ use exhaustive search }
        UINT16 baud_est_delay:3;                    // [7:5] (2) 0: no delay, 1~7: delay 2^(x-1)ms; It should not affect the normal reception by changing baudrate during charcter
        UINT16 hci_uart_baud_est_h5init_retry_th:3; // [10:8] (3) intr_cnt_th for update baud rate at "H5 INIT State" or "R8723A-Det Mode Idle State"
        UINT16 baud_est_by_low_period_at_h5_initialized:1; // [11] (0)
        UINT16 baud_est_falling_cnt_th_2nd:4;       // [15:12] (14) TRAIL State uart_falling_cnt_th = (baud_est_toggle_th*4)
    };
} EFUSE_BAUD_EST_SETTINGS_4_S;
#endif

#endif

#ifdef _8821A_NEW_UART_DESIGN_
  #if 1 //(_HCI_UART_CLK_FREQ == 40000000)
    //according to BAUD_FORMAT_TYPE, coresponding to 40MHz crystal
    // format: refer to BAUDRATE_FORMAT_TYPE
    //                                       divisor|   ovsr<<12     | ovsr_adj<<16 | hci_clk_sel<<27
    #define BAUD_FORMAT_D24_1200             ((2589) | ((12-5) << 12) | (0x7F7<<16))     // clk=40MHz, divisor  = 2082, ovsr = 12-5
    #define BAUD_FORMAT_D24_9600             ((271)  | ((15-5) << 12) | (0x24A<<16))    // clk=40MHz, divisor = 260, ovsr = 15-5
    #define BAUD_FORMAT_D24_57600            ((55)   | ((12-5) << 12) | (0x5AD<<16))    // clk=40MHz, divisor = 63, ovsr = 12-5
    #define BAUD_FORMAT_D24_115200           ((20)   | ((17-5) << 12) | (0x252<<16))    // clk=40MHz, divisor = 29, ovsr = 17-5
    #define BAUD_FORMAT_D24_115200_NO_ADJ    ((29)   | ((12-5) << 12))                  // clk=40MHz, divisor = 29, ovsr = 12-5
    #define BAUD_FORMAT_D24_230400           ((10)   | ((17-5) << 12) | (0x252<<16))    // clk=40MHz, divisor = 11, ovsr = 17-5
    #define BAUD_FORMAT_D24_460800           ((5)    | ((17-5) << 12) | (0x252<<16))    // clk=40MHz, divisor = 8, ovsr = 17-5
    #define BAUD_FORMAT_D24_921600           ((4)    | ((10-5) << 12) | (0x3F7<<16))    // clk=40MHz, divisor = 4, ovsr = 10-5
    #define BAUD_FORMAT_D24_921600_DET       ((4)    | ((11-5) << 12) | (0x000<<16))    // clk=40MHz, divisor = 4, ovsr = 11-5
    #define BAUD_FORMAT_D24_1500000          ((2)    | ((13-5) << 12) | (0x492<<1)6)    // clk=40MHz, divisor = 1, ovsr = 13-5 
    #define BAUD_FORMAT_D24_3000000          ((1)    | ((13-5) << 12) | (0x492<<16))    // clk=40MHz, divisor = 1, ovsr = 13-5 
    #define BAUD_FORMAT_D24_3250000          ((1)    | ((12-5) << 12) | (0x112<<16))    // clk=40MHz, divisor = 1, ovsr = 12-5 
    #define BAUD_FORMAT_D24_4000000          ((1)    | ((10-5) << 12) | (0x000<<16))    // clk=40MHz, divisor = 1, ovsr = 10-5 
    #define BAUD_FORMAT_D24_8000000          ((1)    | ((5-5)  << 12) | (0x000<<16))    // clk=40MHz, divisor = 1, ovsr = 5-5 
  #else
    #if (_HCI_UART_CLK_FREQ == 120000000)
        //according to BAUD_FORMAT_TYPE, coresponding to 40MHz crystal
        // format: refer to BAUDRATE_FORMAT_TYPE
        //                                                               divisor   |    ovsr<<12   | pll_d<<16 | pll_sel<<21 | clk_sel<<23
        to be modified!
        #define BAUD_FORMAT_D24_1200             ((2589) | ((12-5) << 12) | (0x7F7<<16)   // clk=120MHz, divisor  = 2082, ovsr = 16-5
        #define BAUD_FORMAT_D24_9600               ((271) | ((15-5) << 12)) | (0x24A<<16)    // clk=120MHz, divisor = 260, ovsr = 16-5
        #define BAUD_FORMAT_D24_57600               ((55) | ((12-5) << 12)) | (0x5AD<<16)   // clk=120MHz, divisor = 63, ovsr = 11-5
        #define BAUD_FORMAT_D24_115200             ((20) | ((17-5) << 12)) | (0x252<<16)  // clk=120MHz, divisor = 29, ovsr = 12-5
        #define BAUD_FORMAT_D24_230400             ((10) | ((17-5) << 12)) | (0x252<<16)  // clk=120MHz, divisor = 11, ovsr = 16-5
        #define BAUD_FORMAT_D24_460800             ((5)  | ((17-5) << 12)) | (0x252<<16)    // clk=120MHz, divisor = 8, ovsr = 11-5
        #define BAUD_FORMAT_D24_921600             ((4)   | ((10-5) << 12)) | (0x3F7<<16)    // clk=120MHz, divisor = 4, ovsr = 11-5
        #define BAUD_FORMAT_D24_921600_DET         ((4)   | ((11-5) << 12)) | (0x000<<16)    // clk=120MHz, divisor = 4, ovsr = 11-5
        #define BAUD_FORMAT_D24_3000000           ((1)   | ((13-5) << 12)) | (0x492<<16)    // clk=120MHz, divisor = 1, ovsr = 13-5 /* YL20110328 */
   #else
        unexpected _HCI_UART_CLK_FREQ setting!     
   #endif
  #endif  
#else
    //according to BAUD_FORMAT_TYPE, coresponding to 40MHz crystal
    // format: {clk_sel, pll_sel[1:0], pll_d[4:0], (real_ovsr-5), divisor[11:0]}, refer to BAUD_FORMAT_TYPE
    //                                                               divisor   |    ovsr<<12   | pll_d<<16 | pll_sel<<21 | clk_sel<<23
    #define BAUD_FORMAT_D24_1200           ((2083)   | ((16-5) << 12))  // clk=40MHz, divisor  = 2082, ovsr = 16-5
    #define BAUD_FORMAT_D24_9600               ((260) | ((16-5) << 12))   // clk=40MHz, divisor = 260, ovsr = 16-5
    #define BAUD_FORMAT_D24_57600               ((63) | ((11-5) << 12))  // clk=40MHz, divisor = 63, ovsr = 11-5
    #define BAUD_FORMAT_D24_115200             ((29) | ((12-5) << 12))  // clk=40MHz, divisor = 29, ovsr = 12-5
    #define BAUD_FORMAT_D24_230400             ((11) | ((16-5) << 12))  // clk=40MHz, divisor = 11, ovsr = 16-5
    #define BAUD_FORMAT_D24_460800             ((8)   | ((11-5) << 12))    // clk=40MHz, divisor = 8, ovsr = 11-5
    #define BAUD_FORMAT_D24_921600             ((4)   | ((11-5) << 12))    // clk=40MHz, divisor = 4, ovsr = 11-5
    #define BAUD_FORMAT_D24_921600_DET         ((4)   | ((11-5) << 12))    // clk=40MHz, divisor = 4, ovsr = 11-5
    #define BAUD_FORMAT_D24_921600_HS      ((10)  | ((13-5) << 12) | (22<<16) | (0<<21) | (1<<23))   // clk = 20MHz*24/4, ovsr = 13-5, divisor = 10
    #define BAUD_FORMAT_D24_3000000           ((1)   | ((13-5) << 12))    // clk=40MHz, divisor = 1, ovsr = 13-5 /* YL20110328 */
#endif
#endif

#ifdef _UART_H5
/* HCI UART H4/H5 APIs */
API_RESULT hci_uart_lps_clock_off_procedure(void); /* check if UART interface internally */
API_RESULT hci_uart_lps_clock_on_procedure(void); /* check if UART interface internally */
API_RESULT hci_uart_send_long_break(void); /* check if UART interface internally */
API_RESULT hci_uart_h5_send_short_break(void); /* check if UART && H5 inteface internally */
API_RESULT hci_uart_h5_go_sleep(void); /* check if UART && H5 inteface internally */
API_RESULT hci_uart_h5_check_sleep_before_hci_rx(void); /* check if UART && H5 inteface internally */
API_RESULT hci_uart_h4_err_event_complete_check(UCHAR *buffer); /* check if UART && H4 interface internally */
API_RESULT hci_uart_change_baudrate_event_complete_check(UCHAR *buffer);  /* check if UART inteface internally */
API_RESULT hci_uart_change_parameter_event_complete_check(UCHAR *buffer);

// hci_uart_h5_poll_wake_callback: need any calling to do the error handling?
API_RESULT hci_uart_h5_go_lps(void); /* check if UART && H5 inteface internally */

// TODO: (block HCI RX when H4 Error Recovery is not finished? i.e. hci_uart_man.h4_err_event_wait_recov_timer_state = 0) ??


/* Other Utilities for internal use*/
API_RESULT hci_uart_h5_send_sleep_msg_wait_done(void);
API_RESULT hci_uart_h5_send_sleep_msg(void);
void hci_uart_h5_backup(void);
void hci_uart_h5_restore(void);
API_RESULT hci_uart_h5_poll_wake(void);
// TODO: to be verified and ...
void hci_uart_h5_clr_unack_pkt(void);
API_RESULT hci_uart_vendor_set_baud(BAUDRATE_FORMAT_TYPE baud_new);
UINT8 hci_uart_read_cts_in_status(void);
void hci_uart_set_h4_error_interrupt_en(UINT8 value);
#ifdef _8821A_BTON_DESIGN_
void hci_uart_set_mcr_rtsn(UINT8 value);
void hci_uart_baud_record_to_bton(UINT32 baud_d32);
#endif
void hci_uart_h5_isr(UINT32 hci_dma_isr_status_d32);

#ifdef _UART_BAUD_ESTIMATE_
void hci_uart_set_toggle_mon_en(UINT8 val);
void hci_uart_set_falling_cnt_intr_en(UINT8 val);
void hci_uart_set_falling_cnt_th(UINT16 val);
void hci_uart_falling_cnt_isr(UINT32 hci_dma_isr_status_d32);
#endif

#ifdef _YL_H5_TEST
void hci_uart_h5_test_at_keep_alive_event(void);
#endif
#ifdef _YL_TEST_UART_BAUD_ESTIMATE_
void hci_uart_baud_est_test_log(UINT32 dbg_count);
#endif
#endif

#ifdef _UART_BAUD_ESTIMATE_
void hci_uart_baud_est_log(void);
#endif

#ifdef _RTK8723_UART_INIT_
BAUDRATE_FORMAT_TYPE  hci_uart_reset_init_RTL8723(UINT16 re_init_flag);
BAUDRATE_FORMAT_TYPE hci_uart_read_hw_setting(void);
#ifdef _8821A_BTON_DESIGN_
UINT32 get_uint32_from_uint8_array(UINT8 *uint8_array);
void trun_on_off_uart_spi_pll(UINT8 on_off);
#else
void trun_on_off_uart_spi_pll(UINT8 on_off, BAUDRATE_FORMAT_TYPE uart_pll_ctrl);
#endif
#endif

#ifdef _RTK8723_UART_INIT_
#define PLL_ON                                                 1
#define PLL_OFF                                               0
#define PLL_CKRDY_TIMEOUT_LOOPS                 (1 << g_data_uart_settings.pll_ckrdy_timeout) /* YL20110328 */
#define PLL_LDO_STABLE_LOOPS                    (1 << g_data_uart_settings.pll_ldo_stable_loops) /* YL20110328 */
/* 921600Hz for rate detection */
#define BUAD_FORMAT_DETECTION                   BAUD_FORMAT_D24_921600_DET 
 /* default rate: 115200 */  
#define BUAD_FORMAT_DEFAULT                     BAUD_FORMAT_D24_115200_NO_ADJ  

/* setting of UART_STS_REG for baudrate detection, corresponding to 921600 at 40MHz crystal */
#define UART_STS_REG_BAUD_DET_EN0             0x60
#define UART_STS_REG_BAUD_DET_EN1             0x68

/* initialization of BAUD_DET_TIMEOUT_LOOPS */
#ifdef _8821A_BTON_DESIGN_
#define BAUD_DET_TIMEOUT_LOOPS                  (1 << (g_data_uart_settings.baud_det_timeout+16)) /* YL20110328 */
#else
#define BAUD_DET_TIMEOUT_LOOPS                  (1 << g_data_uart_settings.baud_det_timeout) /* YL20110328 */
#endif
#endif

#ifdef _UART_H5
#define H5_WAKEUP_OPT_NONE 0
#define H5_WAKEUP_OPT_SHORT_BREAK 1
#define H5_WAKEUP_OPT_LONG_BREAK 2
#define H5_WAKEUP_OPT_POLL_WAKE 3
#endif

#ifdef _RTK8723_UART_INIT_
extern RLT8723_DATA_UART_SETTINGS_S g_data_uart_settings;
#ifdef _UART_H5
extern RLT8723_DATA_UART_SETTINGS_2_S g_data_uart_settings_2;
extern RLT8723_DATA_UART_SETTINGS_3_S g_data_uart_settings_3;
#ifdef _UART_BAUD_ESTIMATE_
extern EFUSE_BAUD_EST_SETTINGS_1_S g_efuse_baud_est_setting_1;
extern EFUSE_BAUD_EST_SETTINGS_2_S g_efuse_baud_est_setting_2;
extern EFUSE_BAUD_EST_SETTINGS_3_S g_efuse_baud_est_setting_3;
extern EFUSE_BAUD_EST_SETTINGS_4_S g_efuse_baud_est_setting_4;
#endif
#endif
#endif

#ifdef _UART_H5
// TODO: To be included into EFUSE
extern HCI_UART_MAN_TYPE hci_uart_man;
#ifdef _UART_BAUD_ESTIMATE_
extern SECTION_SRAM HCI_UART_MAN_SRAM_TYPE hci_uart_man_sram;
#endif
#endif

#endif

