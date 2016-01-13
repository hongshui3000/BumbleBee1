/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  Contains channel assessment and channel map generation related interface.
 */

/** \addtogroup lmp_internal
 *   @{ */
#ifndef _LMP_CH_ASSESSMENT_H_
#define _LMP_CH_ASSESSMENT_H_

#ifdef COMPILE_CHANNEL_ASSESSMENT

#include "lmp_internal.h"
#include "logger.h"

#define LMP_MAX_CHANNELS                        79
#define LMP_CH_ASS_1_SEC_CLK0_TICKS             3200

/* The structue for New RTL AFH */
typedef struct LC_RTK_PKT_RECD_S_ {
    UINT16 channel_id:7;        /* chhanel index (0 ~ 78) */
    UINT16 access_code_miss:1;  /* the access code is miss ? */
    UINT16 hec_err:1;           /* the HEC error ? */
    UINT16 crc_err:1;           /* the CRC error ? */
    UINT16 ce_index:4;          /* the index of connection entry */
    UINT16 is_from_wifi:1;      /* the source is from wifi */
    UINT16 is_sco_rx:1;         /* the packet is from sco reserved slot */
    UINT16 rssi;                /* the rssi value */
    UINT16 backup_status_rpt0;  /* read value from backup status report reg0 */
    UINT16 backup_status_rpt1;  /* read value from backup status report reg1 */
    UINT16 signal_type;         /* the type of this signal */
    UINT16 backup_status_rpt2;  /* read value from backup status report reg2 */
    UINT16 backup_status_rpt3;  /* read value from backup status report reg3 */
    UINT16 backup_status_rpt4;  /* read value from backup status report reg4 */
} LC_RTK_PKT_RECD_S;

//##################################################################//
//---------------------------------------------------Begin--Jr. Neil defination----
//-----------------------------------//
// add by Jr.Neil--2010-0806                           //
// for RTK AFH mechanism using                        //
// Define channel assessment information struct //
//-----------------------------------//
//#define RTK_AFH_MECHANISM_ENABLE
//#define LC_RTK_AFH_SIGNAL              (FIRST_CH_AS_TASK_SIGNAL + 5)

//#define WE_ARE_MASTER 0x01
//========= Need to define the constansts into efuse==========//

/*
#define THRESHOLD0                   0          // 0.8
#define THRESHOLD1                   0          // 0.5
#define THRESHOLD2                   0          //  0.1
#define THRESHOLD3                   0          //0.65

#define THRESHOLD4                     0
#define THRESHOLD5                     0
*/
#define WEIGHT_GOOD                  0x08
#define WEIGHT_UNKI                  0x05
#define WEIGHT_UNKII                 0x01
#define WEIGHT_BAD                   0x08

//Add by Neil Chen--2011--09--21
#define RSSI_VARIATION               0x05


/*
#define MAX_AFH_EXECUTE_TIMES    40       // Normal AFH Mechanism execute times
#define AFH_EXECUTE_TIMES_I      30
#define AFH_EXECUTE_TIMES_II     20
#define AFH_EXECUTE_TIMES_III    10
#define MIN_AFH_EXECUTE_TIMES    5
*/
//default

#define INITIAL                     0xDD
#define USING_PSD                   0x2F
#define INITIAL_TIMES               0x01
//#define MAX_AFH_EXECUTE_TIMES       199       // Normal AFH Mechanism execute times //5Min to do
//#define AFH_EXECUTE_TIMES_I         99
//#define AFH_EXECUTE_TIMES_II        59
//#define AFH_EXECUTE_TIMES_III       19
//#define AFH_EXECUTE_TIMES_IV        9
//#define MIN_AFH_EXECUTE_TIMES       2
#define MAX_AFH_EXECUTE_TIMES_RECOVERY  9

//#define SCORE_THRESHOLD             0x0      // to classify the good channel or bad channel
#define MIN_AFH_CHANNELS            20       // BT spec requirment
#define MIN_CHANNELS_BY_RTK         28       // to decide  good_channels < MIN_CHANNELS_BY_RTK to RECOVERY STATE 
//#define WLAN_GAP_TH                 10       // WLAN GAP
#define MIN_AFH_CHANNELS_TIMES      0x5 //ff

#define SCORE_THRESHOLD             otp_str_data.score_threshold
#define WLAN_GAP_TH                 otp_str_data.wlan_gap_threshold
#define MAX_AFH_EXECUTE_TIMES       otp_str_data.max_afh_execute_times
#define MIN_AFH_EXECUTE_TIMES       otp_str_data.min_afh_execute_times
#define AFH_EXECUTE_TIMES_I         otp_str_data.afh_execute_times_I
#define AFH_EXECUTE_TIMES_II        otp_str_data.afh_execute_times_II
#define AFH_EXECUTE_TIMES_III       otp_str_data.afh_execute_times_III
#define AFH_EXECUTE_TIMES_IV        otp_str_data.afh_execute_times_IV

#define MAILBOX_READ                1
#define MAILBOX_WRITE               2
#define MAILBOX_AFH                 3
#define DW_READ_WRITE_REG           1
#define W_READ_WRITE_REG            2

#ifndef RTL8723A_B_CUT
#define SCORE_THRESHOLD_I           -300
#define SCORE_THRESHOLD_II          -600
#define SCORE_THRESHOLD_SCO         -100
#else
extern INT16 afh_score_threshold_1;
extern INT16 afh_score_threshold_2;
extern INT16 afh_score_threshold_sco;

#define SCORE_THRESHOLD_I           afh_score_threshold_1
#define SCORE_THRESHOLD_II          afh_score_threshold_2
#define SCORE_THRESHOLD_SCO         afh_score_threshold_sco
#endif

//#define ACL_MODE 0x11

//#define RTK_AFH_USING_SLAVE_REPORT 0x0          // don't using slave report now
//============================================End//

#define GOOD_CHANNEL_BY_SLAVE              0x01
#define UNKNOWN_CHANNEL_BY_SLAVE           0x00
#define BAD_CHANNEL_BY_SLAVE               0x03
// for AFH_STATE PARAM USE
#define NORMAL                             10
#define RECOVERY                           22
#define HOLD						30

#define HOST_NOT_SPECIFY                   0x20
//#define HOST_BAD_CHANNEL                 0x1F

#define WIFI_RECOVER			1
#define RANDOM_RECOVER			0
typedef struct CLASSIFY_THRESHOLD_NODE
{
    UINT16  CRC_threshold0;
    UINT16  CRC_threshold1;
    UINT16  CRC_threshold2;
    UINT16  SC_threshold0;
    UINT16  SC_threshold1;
} CLASSIFY_THRESHOLD_NODE;


typedef struct HEADER_QUALITY_REG6E
{
    UINT16  bit3_bit0:4;
    UINT16  bit14_bit4:11;
    UINT16  bit15:1;
} HEADER_QUALITY_REG6E;

typedef struct HEADER_QUALITY_REG6C
{
    UINT16  bit8_bit0:9;
    UINT16  bit15_bit9:7;
} HEADER_QUALITY_REG6C;

typedef struct ACCAGC_QUALITY
{
    UINT16  rpt_syncmax_bit0: 1;              // Check BT modem's register description register reg6a
    UINT16  rpt_syncmax_bit1: 1;              // Check BT modem's register description register reg6e
    UINT16  rpt_syncmax_bit6_bit2:5;       // Check BT modem's register description register reg70
    UINT16  reserved: 3;
    UINT16  rpt_agc_idx:6;             // check BT modem register description register reg70
} ACCAGC_QUALITY;


typedef struct AVERAGE_PKT_DEF
{
    UCHAR avg_total_pkts;
    UCHAR avg_bad_pkts;
    UCHAR avg_sco_total_pkts;
    UCHAR avg_sco_bad_pkts;	
	
} AVERAGE_PKT_DEF;

typedef struct LC_RTK_AFH_BOTTOM_HALF_PARAM_S_ {
    UINT32 piconet_id;          /* the piconet id */
    UINT32 rsvd;                /* reserved */
    UINT16 signal_type;         /* the type of this signal  */
    AVERAGE_PKT_DEF pkt_def;    /* the packet defined block */
} RTK_AFH_BOTTOM_HALF_PARAM_S;


// this struct for recovery monitor using
typedef struct RECOVERY_MON_REG
{
    UCHAR channel0;
    UCHAR channel1;
    UCHAR channel2;
  	
    UCHAR sel_bank;    // to check the bank sel or not
    UCHAR wlan_nbi;    // 1: wlan, 0: NBI
    UCHAR downtoup;  // 0: try path is down to  up ; 1: try path is up to down	
} RECOVERY_MON_REG;

typedef struct RSSI_AGC_TYPE
{
    UINT32 rssi_count;
    UINT32 rssi_pkts;	

}RSSI_AGC_TYPE;

// to let cal channel seperate...Add by NeilChen...2011-04-15
typedef struct CAL_PARA_DEF_NODE
{
    UCHAR    avg_total_pkts;
    UCHAR    avg_bad_pkts;
    UCHAR    avg_sco_total_pkts;
    UCHAR    avg_sco_bad_pkts;
    UCHAR    channel_pkts_threshold0;
    UCHAR    channel_pkts_threshold1;
    UCHAR    channel_pkts_threshold2;
    UCHAR    acl_th0_slave;
    UCHAR    acl_th1_slave;
    UCHAR    acl_th2_slave;
}CAL_PARA_DEF_NODE;


typedef struct AFH_CH_ASSESSMENT_NODE
{
    UCHAR total_pkts;           // for ACL using
    UCHAR sco_total_pkts;       // for SCO/ eSCO using
    UCHAR num_good_pkts;
    UCHAR num_unknownI_pkts;
    UCHAR num_unknownII_pkts;
    UCHAR num_bad_pkts;
    UCHAR sco_unknownI_pkts;
    UCHAR sco_unknownII_pkts;
    UCHAR sco_bad_pkts;
    UCHAR agc_idx;    
    UINT16 sco_header_quality;	
    UINT16 header_quality;
} AFH_CH_ASSESSMENT_NODE;

typedef struct HOST_CHANNEL_MASK
{
    UCHAR host_ch_status;
}HOST_CHANNEL_MASK;

typedef struct CHANNEL_CLASSIFY_NODE
{
    INT16   score;
    INT16   sco_score;
    UINT16 header_quality; 
    UINT16 sco_header_quality;	
    UCHAR agc_idx;
} CHANNEL_CLASSIFY_NODE;

//########################
//###############################


typedef struct AFH_LOCAL_ASSESSMENT_NODE
{
    /* Host ch status 0x0 => BAD, 0x1 => UNKNOWN */
    UCHAR host_ch_status;
    /* The assessment value, 0(BAD) ~ 255 (GOOD) */
    UCHAR assessment_value;

    UCHAR ch_bad_status;

    UCHAR n_pkts;

    UINT32 average_rssi;
} AFH_LOCAL_ASSESSMENT_NODE;



#ifdef _NEW_MODEM_PSD_SCAN_

typedef union _EFUSE_MODEM_PSD_SETTING_1_S_{ 
    UINT16 d16;
    struct 
    {
        UINT16 rtk_afh_modem_psd_enable:1;              //[0] (1)
        UINT16 afh_map_gen_instant_opt:1;               //[1] (1)
        UINT16 psd_timers_start_opt:2;                  //[3:2] (1) 0: no start, 1: as old afh, 2: if any active link with AFH
        UINT16 psd_scan_3ch_mode:1;                     //[4] (1) 0: 1-channel mode, 1: 3-channel mode (need also to set EN_BPF_PSD related registers)
        UINT16 rom_code_init_en:1;                      //[5] (1)
        UINT16 force_channel_classify_on:1;             // TODO: [6] (0) 
        UINT16 lmp_gen_afh_map_use_modem_psd_only:1;    // TODO: [7] (1)
        UINT16 lmp_gen_afh_map_at_other_case:1;         // TODO: [8] (0)
        UINT16 rsvd:7;                                  //[15:9]
    };
} EFUSE_MODEM_PSD_SETTING_1_S;
typedef union _EFUSE_MODEM_PSD_SETTING_2_S_{ 
    UINT16 d16;
    struct 
    {
        UINT16 rsvd:16;                      // [15:0]
    };
} EFUSE_MODEM_PSD_SETTING_2_S;

extern EFUSE_MODEM_PSD_SETTING_1_S g_efuse_modem_psd_setting_1;
extern EFUSE_MODEM_PSD_SETTING_2_S g_efuse_modem_psd_setting_2;

#define MODEM_PSD_SCAN_STATE_INIT 0
#define MODEM_PSD_SCAN_STATE_RUN 1
#define MODEM_PSD_SCAN_STATE_END 2
#define MODEM_PSD_SCAN_TDM_STATE_INIT 0
#define MODEM_PSD_SCAN_TDM_STATE_RUN 1
#define MODEM_PSD_SCAN_TDM_STATE_END 2

typedef struct _MODEM_PSD_SCAN_MANAGER_S_ 
{
#if 0
    TIMER_ID scan_period_timer; // = OS_INVALID_HANDLE;
    UINT16 scan_period_timeout_ms; // = 500; // 600ms (?)
    TIMER_ID scan_tdm_timer; // = OS_INVALID_HANDLE; //for time-diversity, interlacing PSD Scan is desired?
    UINT16 scan_tdm_timeout_ms; // = 20; // 20ms
    TIMER_ID afh_map_gen_timer; // = oS_INVALD_HANDLE; (optionally used by efuse_modem_psd_setting_1.modem_psd_afh_map_gen_instant_opt = 1)
    UINT16 afh_map_gen_timerout_ms; // = 2500;
#endif

    UINT8 scan_period_state; // = MODEM_PSD_SCAN_STATE_INIT;
    UINT8 scan_period_cnt;   // = 0; // to count how many times the scan_period_timer is experienced
    UINT8 scan_period_cnt_th;// = 4;
    UINT8 scan_tdm_state;    // = MODEM_PSD_SCAN_TDM_STATE_INIT; 
    UINT8 scan_tdm_number;   // = 1; // number of tdm scans per psd_scan_period; 1: no diversity
    UINT8 scan_tdm_cnt;      // = 0;
    UINT8 scan_tdm_ch_start; // = 0;
    UINT8 scan_tdm_ch_step;   // = 1;
    UINT8 scan_tdm_ch_stop;   //  = 78;
}MODEM_PSD_SCAN_MANAGER_TYPE;

#define MODEM_PSD_DBM_OFFSE 200
#define MODEM_PSD_AVG_DBM_OFFSET 0

#define MODEM_PSD_INVALID_DBM_SUB200 0x00
typedef struct _MODEM_PSD_RESULT_MANAGER_S_ 
{
    UINT8 psd_update_cnt_within_la_period;
    UINT8 psd_dbm_add200_signle[LMP_MAX_CHANNELS];      // to save the instaneous value of psd result
    UINT8 psd_dbm_add200_max_hold[LMP_MAX_CHANNELS];
    UINT8 psd_dbm_add200_max_hold_d[LMP_MAX_CHANNELS];  // the latched max_hod value
    UINT8 piconet_rssi_dbm_add200[MAX_PICONET_CNT];     // MODEM_PSD_INVALID_DBM_SUB200: not intended to be tested for generating .psd_afh_map
    UINT8 afh_map[MAX_PICONET_CNT][LMP_AFH_MAP_SIZE];
    UINT8 afh_map_launch_cnt_latch[MAX_PICONET_CNT];    // latch afh_map_launch_cnt when indiviual MAPs are updated
    UINT8 afh_map_launch_cnt_latch_d[MAX_PICONET_CNT];  // the latched .afh_map_launch_cnt_latch to check if any update
    UINT8 afh_map_launch_cnt;                           // cyclic counter from 0~255, to distinguish between different updates for making sure the MAP validity
}MODEM_PSD_RESULT_MANAGER_TYPE;
#endif

#ifdef _NEW_MODEM_PSD_SCAN_
extern TimerHandle_t scan_period_timer;
extern UINT16 scan_period_timeout_ms; // 600ms (?)
extern TimerHandle_t scan_tdm_timer; //for time-diversity, interlacing PSD Scan is desired?
extern UINT16 scan_tdm_timeout_ms; // 20ms
extern TimerHandle_t afh_map_gen_timer; // = oS_INVALD_HANDLE; (optionally used by efuse_modem_psd_setting_1.modem_psd_afh_map_gen_instant_opt = 1)
extern UINT16 afh_map_gen_timerout_ms;
#endif

#define AFH_GOOD_CRC_PKT_WEIGHT          132 //170 //130
#define AFH_GOOD_POLL_NULL_PKT_WEIGHT    131 //169 //129 
#define AFH_CRC_ERROR_PKT_WEIGHT         128
#define AFH_HEC_ERROR_PKT_WEIGHT         127
#define AFH_ACCESS_ERROR_PKT_WEIGHT      129
#define AFH_OUTBAND_ACCESS_ERROR_PKT_WEIGHT (AFH_ACCESS_ERROR_PKT_WEIGHT - 1)
#define AFH_SCO_PKT_WEIGHT               130

/* The ch with above this value will be considered as GOOD Channel */
#define AFH_GOOD_CH_THRESHOLD_VALUE      128

/* The ch with below this value will be marked as bad */
#define AFH_BAD_CHANNEL_THRESHOLD        (AFH_GOOD_CH_THRESHOLD_VALUE -1)

/* If detected more ch as bad than this limit initiate Auto AFH */
#define AFH_AUTO_GENERATION_BAD_CH_COUNT 18

#define AFH_CH_RSSI_DBM_TOLERANCE         0
#define AFH_STEP_VALUE_FOR_CH_RECOVERY   1
#define AFH_MIN_CHANNELS_ADDITION         0

#define AFH_HOST_BAD_CHANNEL        0
#define AFH_HOST_UNKNOWN_CHANNEL    1
#define AFH_LA_MIN_VALUE            0
#define AFH_LA_MID_VALUE            AFH_GOOD_CH_THRESHOLD_VALUE
#define AFH_LA_MAX_VALUE            AFH_GOOD_CRC_PKT_WEIGHT

#define AFH_BAD_CHANNEL_AS_VALUE        0
#define AFH_GOOD_CHANNEL_AS_VALUE       AFH_GOOD_CRC_PKT_WEIGHT
#define AFH_UNKNOWN_CHANNEL_AS_VALUE    AFH_GOOD_POLL_NULL_PKT_WEIGHT

#define AFH_UNUSED_CHANNEL_AS_VALUE     0
#define AFH_USED_CHANNEL_AS_VALUE       AFH_GOOD_CRC_PKT_WEIGHT

#ifdef RTK_AFH_MECHANISM_DEBUG
#define RTK_AFH_LOG(color, log_str_index, para_num, ...) \
   LOG(color, __FILE_NUM__, __LINE__, log_str_index, para_num, "A", __VA_ARGS__)
#else
#define RTK_AFH_LOG(color, log_str_index, para_num, ...)
#endif

extern LMP_SELF_DEVICE_DATA     lmp_self_device_data;
extern UCHAR afh_host_classification[LMP_AFH_MAP_SIZE];

extern UINT8 generate_new_afh_signal;
extern LC_RTK_PKT_RECD_S new_afh_rx_info;

extern const UINT8 afh_ahs79_channel_map[LMP_AFH_MAP_SIZE];
extern UINT8 g_rtk_afh_wlan_psd_internal_enable;
#ifdef _NEW_MODEM_PSD_SCAN_
extern UINT8 g_rtk_afh_modem_psd_internal_enable;
#endif

extern TimerHandle_t la_period_timer;
extern UINT32 afh_la_cycle_period;

/** One shot timer that runs for mailbox assessment duration. */
extern TimerHandle_t la_mailbox_timer;
extern UINT32 la_mailbox_one_shot_duration;

// 20120911 morgan add for wifi set channel and BW data store for mailbox report
extern UCHAR afh_wifi_set_ch_bw[LMP_AFH_MAP_SIZE]; // set to global
//extern UCHAR afh_bt_psd_map[LMP_AFH_MAP_SIZE];	// add for bt psd mailbox report
extern UCHAR afh_bt_psd_map[LMP_MAX_PICONETS_SUPPORTED][LMP_AFH_MAP_SIZE];	// add for bt psd mailbox report
extern UCHAR afh_map_psd[LMP_AFH_MAP_SIZE];
// 20100927 morgan add for afh channel recover range
extern UINT8 bBelowLow_good_channel;
extern UINT8 bLow_good_channel;
extern UINT8 bLowMiddle_good_channel;
extern UINT8 bMiddle_good_channel;
extern UINT8 bLowUpper_good_channel;
extern UINT8 bUpper_good_channel;
extern UINT8 bFullRandomRecover;
extern UINT8 bFastFullOn;
// add by NeilChen
/* Periodic timer that runs for Channel Classify period. */
extern TimerHandle_t la_classify_timer;
extern UINT32 afh_channel_classify_period;
extern RSSI_AGC_TYPE bt_afh_rssi;

extern AFH_CH_ASSESSMENT_NODE afh_ch_status[LMP_MAX_CHANNELS];
extern HOST_CHANNEL_MASK afh_ch_host[LMP_MAX_CHANNELS];
extern CHANNEL_CLASSIFY_NODE afh_ch_quality[LMP_MAX_CHANNELS];
extern UCHAR afh_map_last[LMP_AFH_MAP_SIZE];
//Neil Chen---2011--06--13
extern UCHAR unknow_map[LMP_AFH_MAP_SIZE];
extern UCHAR unknow_map_II[LMP_AFH_MAP_SIZE];
extern UCHAR unk_channel_num;
extern UCHAR unkII_channel_num;
// for support several PICONETS Support
extern UCHAR afh_state[LMP_MAX_PICONETS_SUPPORTED];      // define for AFH state of Normal or Recovery ..
extern UCHAR g_afh_times[ LMP_MAX_PICONETS_SUPPORTED];       // afh mechanism execute times
extern UCHAR afh_recovery_cnt [LMP_MAX_PICONETS_SUPPORTED];    //  AFH recovery state execute counter
extern UCHAR afh_recovery_cnt1[LMP_MAX_PICONETS_SUPPORTED];     //
extern UCHAR afh_recovery_cnt2[LMP_MAX_PICONETS_SUPPORTED];     // check counter
extern UCHAR afh_recovery_cnt3[LMP_MAX_PICONETS_SUPPORTED];
extern UCHAR afh_recovery_cnt4[LMP_MAX_PICONETS_SUPPORTED];
//UCHAR num_unknow_channel;
extern RECOVERY_MON_REG mon_reg[LMP_MAX_PICONETS_SUPPORTED];   // for recovery using monitor
extern UCHAR all_on_map_cnt[LMP_MAX_PICONETS_SUPPORTED];

//Add by Neil Chen---2011-05--04 
extern CAL_PARA_DEF_NODE channel_cal_parameter;
extern UCHAR afh_piconet;  // to indeicate the slave piconet idx

//NeilChen--2011-09-20 
extern UINT16 rssi_result;	
extern UINT16 rssi_result_last;
//============Add By NeilChen for WLAN PSD===========2011-0307-------
extern RSSI_AGC_TYPE bt_afh_rssi;
extern UCHAR afh_map_psd[LMP_AFH_MAP_SIZE];
extern UCHAR wifiwtmailbox;
//====================================================

extern UINT8 g_rtk_afh_wlan_psd_internal_enable;

#ifdef RTL8723A_B_CUT
extern INT16 afh_score_threshold_1;
extern INT16 afh_score_threshold_2;
extern INT16 afh_score_threshold_sco;
#endif

void lmp_update_host_afh_ch_classification(UCHAR* host_ch_cl);
void lmp_generate_afh_map(UCHAR piconet_id, UCHAR* afh_map);
void init_channel_assessment_module(UINT8 load_default_type);
void lmp_update_channel_quality(UCHAR channel_id, UINT16 rssi, UCHAR weightage);

void rtk_afh_bottom_half_callback(void * argument);

#ifdef _ENABLE_MAILBOX_
UINT8 pf_os_trigger_mailbox_task(UINT16 type, UINT32 data_1, UINT32 data_2);
#endif
void lmp_handle_channel_classify_timer_expiry_II(TimerHandle_t timer_handle);
void lmp_wifi_host_irq(UINT32 data0,UINT32 data1);
UCHAR lmp_generate_afh_map_for_device_RTK(void);
void lmp_cal_afh_channel_quality_RTK(UCHAR piconet_id) ;
void lmp_cal_afh_channel_quality_RTK_II(UCHAR piconet_id);

#if 0 //unused logic
#ifdef RTK_AFH_USING_SLAVE_REPORT
void lmp_ch_la_update_la_with_ch_cl_or_afh_map_RTK(UCHAR* afh_map, UCHAR piconet_id, UCHAR role);
#endif
#endif

void lmp_generate_afh_map_for_AFH_RECOVERY(UCHAR piconet_id, UCHAR* afh_map);
void lmp_generate_afh_map_for_AFH_FULL_RANDOM_RECOVERY(UCHAR piconet_id, UCHAR* afh_map);

void lmp_generate_afh_map_from_PGR_RTK(UCHAR piconet_id, UCHAR* afh_map);
void lmp_classify_channel_quality_RTK(LC_RTK_PKT_RECD_S *recd_sig);

#endif /* COMPILE_CHANNEL_ASSESSMENT */
void lmp_afh_state_reset(UINT8 piconet_id);

#endif /* _LMP_CH_ASSESSMENT_H_ */

/** @} end: lmp_internal */
