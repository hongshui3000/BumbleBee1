/***************************************************************************
 Copyright (C) Realtek
 This module is a confidential and proprietary property of Realtek and
 a possession or use of this module requires written permission of Realtek.
 ***************************************************************************/

/**
 * \file
 *  bt_3dd.h (3D Display Definition)
 *
 * \author 
 *  austin <austin_chen@realtek.com>, (C) 2012
 */

/** \addtogroup 3D Display Driver Module
 *  @{ */
#ifndef __3DD_H__
#define __3DD_H__

#include "DataType.h"
#include "bt_fw_os.h"
#include "lc_internal.h"
#include "otp.h"

/* for some other 3DD/HOGP ECO define */ 
#define ENABLE_REPORT_3DD_SYNC_TOGGLE
#define _SET_SYNC_FRAM_FOR_ECO_VERSION
#define ENABLE_SET_3DD_PARAM
//#define FPGA_TEST
//#define BT_3DD_FPGA_TEST
//#define SUPPORT_3DD_SYNC_RPOFILE_RP_V101
// for 3DD test use
//#define D3D_GITTER_TEST

/* 3dd sync train parameters */
#define BT3DD_SYNC_TRAIN_PERIOD_DEFAULT         64 /* unit : 1.25 ms */

/* 3dd linkless broadcast (beacon) parameters */
#define BT3DD_BEACON_PERIOD_DEFAULT             64 /* unit : 1.25 ms */

/* 3dd Class of Device (CoD) */
#define BT3DD_COD_DWORD_I                       0x0008043C
#define BT3DD_COD_DWORD_II                      0x0000043C

/* CSA4 CSB Synchronization Train Maximum Delay (slot) */
#define BT_CSB_SYNC_TRAIN_MAX_RANDOM_DELAY      15

/* CSA4 CSB Default Reserved LT Address */
#define BT_CSB_RESERVED_LT_ADDRESS_DEFAULT         1

/* CSA4 CSB Synchronization Train Timeout Parameters (units : 625 us) */
#define BT_CSB_SYNC_TRAIN_TIMEOUT_DEFAULT       0x0002EE00 // 120 sec (spec)
#define BT_CSB_SYNC_TRAIN_TIMEOUT_MIN           0x00000002
#define BT_CSB_SYNC_TRAIN_TIMEOUT_MAX           0x7FFFFFFE

/* CSA4 CSB Synchronization Train Interval Parameters (units : 625 us) */
#define BT_CSB_SYNC_TRAIN_INTERVAL_DEFAULT          0x0080 // 80 ms (rtk)
#define BT_CSB_SYNC_TRAIN_INTERVAL_MIN              0x0020
#define BT_CSB_SYNC_TRAIN_INTERVAL_MAX              0xFFFE

/* CSA4 CSB Synchronization Scan Timeout Parameters (units : 625 us) */
#define BT_CSB_SYNC_SCAN_TIMEOUT_DEFAULT            0x2000 // 5.12 sec (spec)
#define BT_CSB_SYNC_SCAN_TIMEOUT_MIN                0x0022
#define BT_CSB_SYNC_SCAN_TIMEOUT_MAX                0xFFFE

/* CSA4 CSB Broadcast_Supervision Timeout Parameters (units : 625 us) */
#define BT_CSB_BEACON_SUPERVISION_TIMEOUT_DEFAULT   0x2000 // 5.12 sec (spec)
#define BT_CSB_BEACON_SUPERVISION_TIMEOUT_MIN       0x0002
#define BT_CSB_BEACON_SUPERVISION_TIMEOUT_MAX       0xFFFE

/* CSA4 CSB Broadcast_Interval Parameters (units : 625 us) */
#define BT_CSB_BEACON_INTERVAL_DEFAULT              0x0080 // 80 ms (rtk)
#define BT_CSB_BEACON_INTERVAL_MIN                  0x0002
#define BT_CSB_BEACON_INTERVAL_MAX                  0xFFFE

/* CSA4 CSB Broadcast Instant Offset */
#define BT_CSB_BEACON_OFFSET                        1

/* CSA4 CSB Synchronization Train Packet Period (units : 1.25 ms) */
#define BT_CSB_SYNC_TRAIN_PACKET_PERIOD             1

/**
 * @struct BT_EFUSE_TV_3DD_GROUP_
 * @brief TV 3DD EFuse settings.
 *
 * Refer to EFuse[0x187-0x174] and otp_str_data.EFuse_TV_3DD_Group.
 */
typedef struct BT_EFUSE_TV_3DD_GROUP_ {
    /** EFUSE[0x174][0] Flag to turn on 3D sync debouncing. */
    UINT8 enable_3dd_gpio_deboubcing : 1;
    /** EFUSE[0x174][1] */
    UINT8 enable_3d_sync_schmitt : 1;
    /** EFUSE[0x174][2] */
    UINT8 version_3DSP : 1;
    /** EFUSE[0x174][7:3] Fast recovery time. */
    UINT8 fast_recover_times : 5;

    /** EFUSE[0x175][0] Left first or not. */
    UINT8 L_R_first : 1;
    /** EFUSE[0x175][2:1] Calculation window. */
    UINT8 calculation_window : 2;
    /** EFUSE[0x175][4:3] */
    UINT8 period_delay : 2;
    /** EFUSE[0x175][7:5] */
    UINT8 reserved : 3;

    /** EFUSE[0x176] */
    INT8  lsod;
    /** EFUSE[0x177] */
    INT8  lscd;
    /** EFUSE[0x178] */
    INT8  rsod;
    /** EFUSE[0x179] */
    INT8  rscd;
    /** EFUSE[0x17A] */
    UINT8 period_diff_limit;
    /** EFUSE[0x17B] */
    UINT8 shutter_open_delay;
    /** EFUSE[0x17C] */
    UINT8 shutter_close_delay;

    /** EFUSE[0x17D][3:0] */
    UINT8 rejection_times : 4;
    /** EFUSE[0x17D][7:4] */
    UINT8 toggle_detection_window : 4;
    /** EFUSE[0x17E] */
    UINT8 toggle_detection_delay;
    /** EFUSE[0x17F] */
    UINT8 period_significant_diff;
    /** EFUSE[0x180][3:0] */
    UINT8 coeff_gain : 4;
    /** EFUSE[0x180][7:4] */
    UINT8 coeff_avg : 4;
    /** EFUSE[0x181] */
    UINT8 period_significant_diff_long_term;
} BT_EFUSE_TV_3DD_GROUP;

#define efuse_tv_3dd ((BT_EFUSE_TV_3DD_GROUP *) otp_str_data.EFuse_TV_3DD_Group)

/*==========================================================*/
/* SW Structure for 3DD Association Unit                    */
/*==========================================================*/
typedef struct BT_3DD_ASSOCIATION_S_ {
} BT_3DD_ASSOCIATION_S;

/*==========================================================*/
/* SW Structure for 3DD Synchronization Train Unit          */
/*==========================================================*/
typedef struct BT_3DD_SYNC_TRAIN_S_ {
    UINT16 interval;    
} BT_3DD_SYNC_TRAIN_S;

/*==========================================================*/
/* SW Structure for 3DD Beacon Unit                         */
/*==========================================================*/
typedef struct BT_3DD_BEACON_S_ {
    UINT16 is_2d_mode:1;
    UINT16 dual_video_stream_mode:1; 
    UINT16 rsvd:14;
    UINT16 interval; /* beacon interval */     
    UINT16 lsod; /* left shuntter open delay (us) */
    UINT16 lscd; /* left shuntter close delay (us) */    
    UINT16 rsod; /* right shuntter open delay (us) */
    UINT16 rscd; /* right shuntter close delay (us) */      
} BT_3DD_BEACON_S;

/*==========================================================*/
/* SW Structure for CSA4 Clock Capture Unit                 */
/*==========================================================*/
typedef struct BT_CSA4_CLOCK_CAPTURE_UNIT_S_ {
    UINT8 enable:1;                /* enable or disable clock capture */
    UINT8 is_allow_lpo:1;          /* allow to enter low power mode or not ? */
    UINT8 rsvd:6;                  /* reserved */
    UINT8 piconet_id;              /* my piconet id */
    UINT8 num_ext_clk_captures;    /* number of triggered clock captures filtered 
                                      between sending an event to host */
    UINT8 cur_ext_clk_captures;    /* current number of triggered clock captures */                                                
    UINT32 pre_piconet_clk;        /* previous piconet clock */ 
    UINT16 pre_piconet_clk_us;     /* previous piconet clock in us */
    UINT16 conn_handle;            /* recorded connection handle */    
} BT_CSA4_CLOCK_CAPTURE_UNIT_S;

/*==========================================================*/
/* SW Structure for CSA4 Sync Train Transmitter Unit        */
/*==========================================================*/
typedef struct BT_CSA4_SYNC_TRAIN_TX_UNIT_S_ {
    UINT8 enable:1;                /* enable or disable sync train */   
    UINT8 rsvd:7;                  /* reserved */
    UINT8 service_data;            /* service data of packet body */
    UINT8 ch_map[10];              /* current channel map */
    UINT8 num_of_next_beacon_inst; /* display number of next beacon instant */
    UINT8 stp2stp_duration;        /* the duration between two sync train packet 
                                      (unit: slot pair) */
    UINT16 interval_min;           /* minimum stp interval (unit: slot) */
    UINT16 interval_max;           /* maximum stp interval (unit: slot) */
    UINT16 interval;               /* current stp interval (unit: slot) */
    UINT32 timeout;                /* duration for sending sync train (unit: slot) */
} BT_CSA4_SYNC_TRAIN_TX_UNIT_S;

/*==========================================================*/
/* SW Structure for CSA4 Beacon Transmitter Unit            */
/*==========================================================*/
typedef struct BT_CSA4_BEACON_TX_UNIT_S_ {
    UINT8 enable:1;                /* enable or disable csb */
    UINT8 is_allow_lpo:1;          /* allow to enter low power mode or not ? */    
    UINT8 is_reserved_lt_addr:1;   /* is reserved lt addr ??*/
    UINT8 fragment:2;              /* 0: continue, 1: start, 2: end, 3: no fragment */     
    UINT8 rsvd:3;                  /* reserved */
    UINT8 lt_addr;                 /* reserved lt addr for csb */ 
    UINT8 piconet_id;              /* the piconet id of reserved lt addr */
    UINT8 select_tx_packet_type;   /* current select packet type */
    UINT16 data_len;               /* length of payload body field */           
    UINT16 support_packet_type_bm; /* support packet type */
    UINT16 interval_min;           /* minimum csb interval (unit: slot) */ 
    UINT16 interval_max;           /* maximum csb interval (unit: slot) */
    UINT16 interval;               /* current csb interval (unit: slot) */
    UINT16 supervision_timeout;    /* csb supervision timeout (unit: slot) */
    UINT8 *pData;                  /* the pointer of data buffer */    
    INT32 ext_frame_period_us;    /* the period of external frame sync (us) */     /* 20140417 morgan add from 8761a-dcut*/
} BT_CSA4_BEACON_TX_UINT_S;
#ifdef _SUPPORT_CSB_RECEIVER_

/*==========================================================*/
/*   SW Structure for CSA4 Sync Train Receiver Unit         */
/*==========================================================*/
typedef struct BT_CSA4_SYNC_TRAIN_RX_UNIT_S_ {
    UINT8 bd_addr[LMP_BD_ADDR_SIZE];
    UINT16 timeout;                /* duration for scanning sync train (unit: slot) */
    UINT16 scan_window;            /* current st scan window (unit: slot) */
    UINT16 scan_interval;          /* current st scan interval (unit: slot) */
    UINT8 enable:1;
               /* enable */
    UINT8 scan_map:3;              /* scan map */
    UINT8 rsvd:4;                  /* reserved */
    /* below are used when sync train is received.*/
    UINT8 csb_lt_addr;             /* the lt_addr of beacon in sync-train.*/
#ifdef _DAPE_TEST_CSB_RX_SELF_CALC_CLK_OFST_BY_VENDOR_CMD
    UINT32 master_clk;             /* the master clock when rx sync train */
#endif    
    UINT32 clock_offset;           /* native clock minus rx clock. [27:0]*/
    UINT8 ch_map[10];              /* afh map informarion in sync-train */
    UINT8 next_instant[4];         /* the clock of next csb instant in sync-train. */
    UINT8 csb_interval[2];           /* the csb interval in sync-train. (unit: slot) */
    UINT8 service_data;            /* the service data in sync-train. */
} BT_CSA4_SYNC_TRAIN_RX_UNIT_S;
/*==========================================================*/
/*   SW Structure for CSA4 Beacon Receiver Unit         */
/*==========================================================*/
typedef struct BT_CSA4_BEACON_RX_UNIT_S_ {
    UINT8 enable:1;                  /* enable or disable csb rx*/
    UINT8 rsvd:7;                    /* reserved */    
    UINT8 lt_addr;                   /* lt addr for csb */ 
    UINT16 ce_index;
    UINT8 bd_addr[LMP_BD_ADDR_SIZE];
    UINT16 interval;               /* current csb interval (unit: slot) */
    UINT32 clk_offset;             /* the clock offset */
    UINT32 next_instant;           /* the clock of next csb instant. */
    UINT16 supervision_timeout;    /* csb supervision timeout (unit: slot) */
    UINT8 remote_timging_accuracy; /* timing accuracy of remote csb tx device. */ 
    UINT8 skip;                    /* number of csb instants to skip after success rx.*/ 
    UINT16 support_packet_type_bm;               /* rx pkt type. */ 
    UINT8 ch_map[10];              /* current channel map */
    UINT32 clk_rx;             /* the clock of beacon-rx-ed. (for sending event) */
    UINT32 crc_err;             /* if the beacon crc error or not. */
} BT_CSA4_BEACON_RX_UNIT_S;
#endif
/*==========================================================*/
/* SW Structure for 3DD Application Manager Unit            */
/*==========================================================*/
typedef struct BT_3DD_MANAGER_S_ {    
    UINT8 enable:1;                 /* enable or not */
    UINT8 is_csb_mode:1;            /* support csb or rp mode now */
    UINT8 use_piconet_clk:1;        /* capture piconet clock  */
    UINT8 rsvd:5;                   /* reserved */

    /* use for native clock or rp application */
    UINT8 ext_clk_fraction;         /* frame sync fragction (1/256 us) */    
    UINT16 ext_clk_period;          /* frame sync period (us) */

    UINT16 ext_clk_edge_counts;     /* external clock capture counter */    

    UINT16 bbclk_us;                /* previous bb clock in usec */
    UINT32 bbclk;                   /* previous bb clock */

    /* TBD: use union to share memory ?? */

    /*-------------------------*/
    /* reference protocol part */
    /*-------------------------*/
    BT_3DD_ASSOCIATION_S assoc;
    BT_3DD_SYNC_TRAIN_S synctrain;
    BT_3DD_BEACON_S beacon;   

    /*-------------------------------------*/
    /* connectionless slave broadcast part */
    /*-------------------------------------*/
    BT_CSA4_CLOCK_CAPTURE_UNIT_S clk_cap;
    BT_CSA4_SYNC_TRAIN_TX_UNIT_S stp_tx_param;
    BT_CSA4_BEACON_TX_UINT_S csb_tx_param;       
#ifdef _SUPPORT_CSB_RECEIVER_
    BT_CSA4_SYNC_TRAIN_RX_UNIT_S stp_rx_param;
    BT_CSA4_BEACON_RX_UNIT_S csb_rx_param;   
#endif
    /* dape: we share these two timers with CSB rx. */
    //TIMER_ID stp_timer;                 /* synchronization train timer */    
    //TIMER_ID csb_trx_supervision_timer;  /* csb trx supervision timer */    
} BT_3DD_MANAGER_S;

extern BT_3DD_MANAGER_S bt_3dd_var;

/**
 * @struct CO_EXIST_CTRL_REG_
 * WLAN co-existence and 3D sync time-domain debouncing control register.
 */
typedef struct CO_EXIST_CTRL_REG_
{
    /** [3:0] Dummy. */
    UINT32 dummy : 4;
    /** [7:4] Reserved. */
    UINT32 reserved : 4;
    /** [11:8] WLAN co-existence mode selection.
     *
     * 0: Two wire co-existence, BT_PRI, WLAN_ACT.\n
     * 1: Three wire co-existence, BT_PRI, BT_ACT, WLAN_ACT.\n
     * 2: Four wire co-existence, BT_PRI, BT_ACT, BT_OOB, WLAN_ACT.\n
     * 3: RTK co-existence, WLAN_ACT/DATA, BT_ACT/CLK.\n
     * 4: No co-existence mechanism.
     */
    UINT32 ce_mode : 4;
    /** [12] Enable 3D Sync time domain debouncing. */
    UINT32 deb_3d_sync_en : 1;
    /** [15:13] Reserved. */
    UINT32 reserved1 : 3;
    /** [31:16] Debouncing window.
     *
     * Default: 0xfff0.\n
     * Range: 0x0002~0xfffe
     */
    UINT32 deb_win : 16;
} CO_EXIST_CTRL_REG;

#define CO_EXIST_CTRL_REG_MMIO VENDOR_REG_ADDR(0x14C)

static inline CO_EXIST_CTRL_REG co_exist_ctrl_reg_read()
{
    return RD_REG_MMIO(CO_EXIST_CTRL_REG, CO_EXIST_CTRL_REG_MMIO);
}

static inline void co_exist_ctrl_reg_write(CO_EXIST_CTRL_REG reg)
{
    WR_REG_MMIO(CO_EXIST_CTRL_REG, CO_EXIST_CTRL_REG_MMIO, reg);
}

extern void (*bt_3dd_driver_init)(void);
void bt_3dd_driver_retrieve_inq_scan(void);
void bt_3dd_driver_update_transmit_beacon_parameters(void);
UINT32 bt_3dd_driver_get_sync_native_clock(UINT16 *offset);
void bt_3dd_driver_set_3d_sync_debouncing(UINT16 window);
void bt_3dd_driver_unset_3d_sync_debouncing();


// 20131128 morgan add for 3D only FPGA test
// 20140417 morgan copy to 8821
#ifdef _3DD_ONLY_SUPPORT
UCHAR pat3d_sim_hci_handle_set_connectionless_slave_broadcast(UCHAR an_status);
UCHAR pat3d_sim_hci_handle_start_synchronization_train_command(UCHAR an_status);
UCHAR pat3d_sim_hci_handle_set_connectionless_slave_broadcast_data(HCI_CMD_PKT *hci_cmd_ptr,UCHAR mode);
#endif
void fn_set_shutter_delay(void);
void fn_host_set_shutter_delay(INT16 bt_shutter_lsod,INT16 bt_shutter_lscd, INT16 bt_shutter_rsod, INT16 bt_shutter_rscd);
void pat3d_generate_3dd_gpio_sync_toggle_event(UINT8 status);

#ifdef _NEW_3DD_HW_
void bt_3dd_driver_set_ext_frame_sync_period(void);
#endif

#if !defined(_SUPPORT_RP_VIA_CSB_SPEC_) && !defined(_DO_NOT_SUPPORT_RP_)
void bt_3dd_driver_association_enable(void);
void bt_3dd_driver_association_disable(void);
void bt_3dd_driver_start_sync_train(void);
void bt_3dd_driver_stop_sync_train(void);
void bt_3dd_driver_start_transmit_beacon(void);
void bt_3dd_driver_stop_transmit_beacon(void);
#ifdef _3DD_FUNCTION_TEST_
void bt_3dd_test_func(void);
#endif
#endif

#if !defined(_DO_NOT_SUPPORT_RP_)
void bt_3dd_checkin_association_notification_packet(BB_RX_PARAMS *pparam);
UINT8 bt_3dd_handle_received_assocation_notification_packet(void);
#endif

#ifdef _SUPPORT_CSB_TRANSMITTER_
/* dm1 < dh1 < 2dh1 < 3dh1 < dm3 < dh3 < 
   dm5 < dh5 < 2dh3 < 3dh3 < 2dh5 < 3dh5 */
extern const UINT8 g_host_hci_packet_type_bitnum_sort_by_maxlen[12];
extern const UINT8 g_host_hci_packet_type_ptt_sort_by_maxlen[12];

UINT32 bt_csb_driver_get_ext_frame_sync_clock(UINT8 native_clk, UINT8 piconet, UINT16 * offset);
void bt_csb_sync_train_timeout_callback(TimerHandle_t timer_handle);
void bt_csb_control_sync_train_timer(UINT8 enable);
void bt_csb_driver_start_sync_train(void);
void bt_csb_driver_stop_sync_train(void);
void bt_csb_tx_supervision_callback(TimerHandle_t timer_handle);
void bt_csb_control_supervision_timer(UINT8 enable);   
UINT8 bt_csb_decide_broadcast_packet_type(UINT16 *length, UINT16 *pkt_type_lut, UINT8 *pkt_type_table);
UINT8 bt_csb_driver_prepare_beacon_data(void);
void bt_csb_driver_start_transmit_beacon(void);
void bt_csb_driver_stop_transmit_beacon(void);
void bt_csb_driver_handle_ext_clock_capture_by_edge_trigger(void);
void bt_csb_update_host_afh_ch_classification(UINT8 * map);
void bt_csb_check_and_set_csb_or_rp_mode(void);
#endif
#ifdef _SUPPORT_CSB_RECEIVER_
UCHAR hci_validate_truncated_page_cmd_params(HCI_CMD_PKT *hci_cmd_ptr);
void bt_csb_driver_start_sync_scan(void);
void bt_csb_control_sync_scan_timer(UINT8 enable);
void bt_csb_sync_scan_timeout_callback(TimerHandle_t timer_handle);
void bt_csb_driver_stop_sync_scan(void);
void bt_csb_driver_start_rx_beacon(void);
void bt_csb_driver_stop_rx_beacon(void);
void bt_csb_rx_control_supervision_timer(UINT8 enable);
void bt_csb_rx_supervision_callback(TimerHandle_t timer_handle);
void bt_3dd_checkin_sync_train_packet(BB_RX_PARAMS *pparam);
UINT8 bt_3dd_handle_received_sync_train_packet(void);
void bt_3dd_handle_received_beacon_packet(UINT16 payload_length, UINT8 am_addr, UINT8 phy_piconet_id, UCHAR pkt_type, UCHAR llid);
#endif

#endif /* __3DD_H__ */

