/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  Contains Channel assessment and channel map generation implementation.
 *  Channel assessment and local assessment are synonymous.
 */

/********************************* Logger *************************/
enum { __FILE_NUM__= 50 };
/********************************* Logger *************************/

/* ========================= Include File Section ========================= */
#include "lmp_ch_assessment.h"
#ifdef PROFILE_CALC
#include "profiler.h"
#endif

#include "bt_fw_acl_q.h"
#include "mem.h"
#include "lmp_2_1.h"
#include "mailbox.h"
#include "lc.h"
#include "crypto.h"
#include "pta_meter.h"
#include "lmp_vendor_defines.h"

#ifdef COMPILE_CHANNEL_ASSESSMENT

#ifdef _ROM_CODE_PATCHED_
PF_ROM_CODE_PATCH_FUNC rcp_lmp_handle_channel_assess_timer_expiry_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_lmp_handle_channel_classify_timer_expiry_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_lmp_handle_channel_classify_timer_expiry_II_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_lmp_generate_afh_map_for_device_RTK_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_lmp_generate_afh_map_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_lmp_generate_afh_map_post_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_lmp_generate_afh_map_for_device_RTK_post_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_modem_psd_man_refresh_at_la_period_timer_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_modem_psd_man_init_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_lmp_generate_afh_map_by_modem_psd_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_lmp_generate_afh_map_check_and_handle_few_channels_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_lmp_classify_channel_quality_RTK_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_lmp_cal_afh_channel_quality_RTK_generator_func = NULL;
// morgan add 20120823 for recover mechanism
PF_ROM_CODE_PATCH_FUNC rcp_lmp_generate_afh_recovery_subroutine_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_lmp_generate_afh_map_for_AFH_RECOVERY_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_lmp_generate_afh_map_front_half_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_lmp_generate_afh_map_from_PGR_RTK_func = NULL;
//PF_ROM_CODE_PATCH_FUNC rcp_lmp_decide_no_wifi_RECOVERY_func= NULL;
PF_ROM_CODE_PATCH_FUNC rcp_lmp_generate_afh_map_for_AFH_RECOVERY_part1_func = NULL;
//NeilChen 20121119
PF_ROM_CODE_PATCH_FUNC rcp_lmp_generate_afh_map_by_psd_recovery_func = NULL;
#ifndef _BT_ONLY_
PF_ROM_CODE_PATCH_FUNC rcp_lmp_generate_afh_map_by_wlan_psd_func = NULL;
#endif
PF_ROM_CODE_PATCH_FUNC rcp_lmp_modem_psd_scan_period_timer_expiry = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_lmp_modem_psd_scan_tdm_timer_expiry = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_lmp_modem_psd_afh_map_gen_timer_expiry = NULL;

PF_ROM_CODE_PATCH_FUNC rcp_lmp_piconet_start_afh_map_updation = NULL;
#endif

UINT32 g_last_afh_update = 0x0;

OS_HANDLE hci_ch_as_task_handle; /**< Channel Assessment Task Handle */

/* AFH_Host_Channel_Classification */
/* 79 1 bit field. */
/* 0 => bad, 1 => unknown */
/* Bit 79 reserved, set 0 */
UCHAR afh_host_classification[LMP_AFH_MAP_SIZE];

//add by Neil to store host last information
UCHAR afh_host_classification_last[LMP_AFH_MAP_SIZE];


const UINT8 afh_ahs79_channel_map[LMP_AFH_MAP_SIZE] = 
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F};

const UINT8 afh_map_psd_table[16] =
{
    0x00, 0x03, 0x0C, 0x0F, 0x30 ,0x33, 0x3C, 0x3F,
    0xC0, 0xC3 ,0xCC, 0xCF, 0xF0, 0xF3, 0xFC, 0xFF
};

//#############################################################//
//===============================================//
// add by Jr.Neil--2010-0806                     //
// for RTK AFH mechanism using                   //
// Define channel assessment information struct  //
//=================================Begin=========//

//UCHAR afh_execute_times[LMP_MAX_PICONETS_SUPPORTED];
AFH_CH_ASSESSMENT_NODE afh_ch_status[LMP_MAX_CHANNELS];
HOST_CHANNEL_MASK afh_ch_host[LMP_MAX_CHANNELS];
CHANNEL_CLASSIFY_NODE afh_ch_quality[LMP_MAX_CHANNELS];
UCHAR afh_map_last[LMP_AFH_MAP_SIZE];
//Neil Chen---2011--06--13
UCHAR unknow_map[LMP_AFH_MAP_SIZE];
UCHAR unknow_map_II[LMP_AFH_MAP_SIZE];
UCHAR unk_channel_num;
UCHAR unkII_channel_num;
// for support several PICONETS Support
UCHAR afh_state[LMP_MAX_PICONETS_SUPPORTED];      // define for AFH state of Normal or Recovery ..
UCHAR afh_sub_state[LMP_MAX_PICONETS_SUPPORTED];      // used for separate wifi recover or random recover
UCHAR g_afh_times[ LMP_MAX_PICONETS_SUPPORTED];       // afh mechanism execute times
UCHAR afh_recovery_cnt [LMP_MAX_PICONETS_SUPPORTED];    //  AFH recovery state execute counter
UCHAR afh_recovery_cnt1[LMP_MAX_PICONETS_SUPPORTED];     //
UCHAR afh_recovery_cnt2[LMP_MAX_PICONETS_SUPPORTED];     // check counter
UCHAR afh_recovery_cnt3[LMP_MAX_PICONETS_SUPPORTED];
UCHAR afh_recovery_cnt4[LMP_MAX_PICONETS_SUPPORTED];
//UCHAR num_unknow_channel;
RECOVERY_MON_REG mon_reg[LMP_MAX_PICONETS_SUPPORTED];   // for recovery using monitor
UCHAR all_on_map_cnt[LMP_MAX_PICONETS_SUPPORTED];

//Add by Neil Chen---2011-05--04 
CAL_PARA_DEF_NODE channel_cal_parameter;
UCHAR afh_piconet = 0xFF;  // to indeicate the slave piconet idx

//NeilChen--2011-09-20 
UINT16 rssi_result = 0;	
UINT16 rssi_result_last = 0;
//============Add By NeilChen for WLAN PSD===========2011-0307-------
RSSI_AGC_TYPE bt_afh_rssi;
UCHAR afh_map_psd[LMP_AFH_MAP_SIZE];
UCHAR wifiwtmailbox = 0;
UINT8 g_wifi_cnt = 0;
UINT8 g_is_use_wifi_cnt = FALSE;
//====================================================
// 20100927 morgan add for afh channel recover range
#ifdef _DAPE_AFH_RECOVER_FASTER
UINT8 bBelowLow_good_channel = 30;
#else
UINT8 bBelowLow_good_channel = 24;
#endif
UINT8 bLow_good_channel = 32;
UINT8 bLowMiddle_good_channel = 40;
UINT8 bMiddle_good_channel = 46;
UINT8 bLowUpper_good_channel = 52;
UINT8 bUpper_good_channel = 59;
UINT8 bFullRandomRecover = 0;
UINT8 bFastFullOn = 1;
// 20120911 morgan add for wifi set channel and BW data store for mailbox report
UCHAR afh_wifi_set_ch_bw[LMP_AFH_MAP_SIZE]; // set to global
UCHAR afh_bt_psd_map[LMP_MAX_PICONETS_SUPPORTED][LMP_AFH_MAP_SIZE];	// add for bt psd mailbox report
UCHAR afh_bt_hold_state_map[LMP_MAX_PICONETS_SUPPORTED][LMP_AFH_MAP_SIZE];
//====================================================
UINT8 g_rtk_afh_wlan_psd_internal_enable = 0;

#ifdef RTL8723A_B_CUT
INT16 afh_score_threshold_1 = -300;
INT16 afh_score_threshold_2 = -600;
INT16 afh_score_threshold_sco = -100;
#endif

const UINT8 byte_mask2[8] = {0xFC, 0xFC, 0xF3, 0xF3,
                             0xCF, 0xCF, 0x3F, 0x3F
                            };

extern OS_HANDLE isr_extended_task_handle;

/* ===================== Variable Declaration Section ===================== */
/** Periodic timer that runs for assessment period. */
TimerHandle_t la_period_timer = NULL;
UINT32 afh_la_cycle_period = AFH_LA_CYCLE_PERIOD;

/** One shot timer that runs for mailbox assessment duration. */
TimerHandle_t la_mailbox_timer = NULL;
UINT32 la_mailbox_one_shot_duration = 200; /*0.2 sec */

// add by NeilChen
/* Periodic timer that runs for Channel Classify period. */
TimerHandle_t la_classify_timer = NULL;
UINT32 afh_channel_classify_period = 2500;    // 2.6 Sec 

UINT8 afh_repeat_check_count = 0; /* this repeat counter is used to 
                                     avoid protocol dead-lock */
#ifdef _NEW_MODEM_PSD_SCAN_

//RSSI_AGC_TYPE bt_afh_rssi_pn[MAX_PICONET_CNT];
EFUSE_MODEM_PSD_SETTING_1_S g_efuse_modem_psd_setting_1;
EFUSE_MODEM_PSD_SETTING_2_S g_efuse_modem_psd_setting_2;
UINT8 g_rtk_afh_modem_psd_internal_enable = 0;

MODEM_PSD_SCAN_MANAGER_TYPE modem_psd_scan_man;
MODEM_PSD_RESULT_MANAGER_TYPE modem_psd_result_man;

#endif

//#################################################################//
//============ Add the function declaration by Jr. Neil =======  2010--08--23--
//#################################################################
//#####################################################
//
//#######################################################

/* ===================== Function Definition Section ====================== */
#ifdef COMPILE_RSSI_REPORTING

/**
 * Calculates and stores RSSI value in LMP-CE.
 *
 * \param val Value of RSSI reported by baseband.
 *
 * \return None.
 */
void lmp_calc_rssi(UINT32 val)
{
    UINT16 ce_index;
    UINT16 rssi;
    
    ce_index = (UINT16) ( (val >> 16) & 0xffff);

    /* If LM level connection is not yet setup, do not process RSSI. */
    if (lmp_connection_entity[ce_index].setup_complete_status ==
            CONN_COMPLETE_EVENT)
    {
        rssi = (UINT16) (val & 0xffff);
        lc_calculate_and_store_rssi(rssi, ce_index);
    }

    return;
}
#endif

/**
 * Services messages posted to lmp_ch_as_task.
 *
 * \param signal_ptr Pointer to the signal that is serviced.
 *
 * \return None.
 */
void hci_ch_as_task(OS_SIGNAL *signal_ptr)
{   
    LC_RTK_PKT_RECD_S *rtk_pkt_blk;

    switch(signal_ptr->type)
    {
        case LC_NEW_PKT_RECD_SIGNAL:
            rtk_pkt_blk = (LC_RTK_PKT_RECD_S *)signal_ptr;
            // need to handle how to get the status from per rx descriptor..... Jr. Neil

            //====================================================
            //=========== Add the Debug Infor =====================
            //============================2010-09-20-Jr. Neil=========
            // The new function created by Jr. Neil for RTK AFH Mechanism--2010-08-11--
            lmp_classify_channel_quality_RTK(rtk_pkt_blk);
            break;
            
#if defined(_CCH_LPS_) && defined(_YL_LPS)
        case CH_AS_TASK_ENTER_LPS_MODE:
            {
                UINT32 wakeup_instant = (UINT32)(signal_ptr->param);
                UCHAR lps_mode =  (UCHAR)((UINT32)(signal_ptr->ext_param) >> 8);
                UCHAR piconet_id = (UCHAR)((UINT32)(signal_ptr->ext_param) & 0xff);
                UINT8 ret_value;

                if(sleep_mode_param.bb_sm_sts == BB_NORMAL_MODE)
                {
                    // for g_efuse_lps_setting_4.lps_use_intr, only lps_mode is used
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
                        RT_BT_LOG(BLUE, LPS_LOG_052, 1, ret_value);
                    }     

//                  LPS_DBG_LOG(BLUE, LPS_LOG_025, 3,lps_mode, wakeup_instant, piconet_id);               

                }

            }
            break;
#endif

#if defined(_LE_AUTO_REPORT_RSSI_AND_LOGIN_INOUT_FUNC_) && defined(LE_MODE_EN)
        case CH_AS_TASK_LE_RSSI:
            {
                UINT8 index;
                UINT16 rssi_count;
                INT32 rssi_sum;            
                index = (UINT8)((UINT32)signal_ptr->param);
                rssi_count = (UINT16)(((UINT32)signal_ptr->param) >> 16);
                rssi_sum = (INT32)signal_ptr->ext_param;            
                rssi_app_vendor_check_and_send_rssi_event(1, index, 
                                                        rssi_count, rssi_sum);          
            }
            break;
#endif

        default:
            break;
    }
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//============All Use Functions==== RTK AFH Mechanims and MindTree both used this functions
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

UINT8 lmp_check_connection_entity_in_afh(UINT8 ce_index, UINT8 piconet_id)
{
    LMP_CONNECTION_ENTITY *ce_ptr;   

    if (!lc_sca_manager.pnet[piconet_id].active)
    {
        return FALSE;
    }

    ce_ptr = &lmp_connection_entity[ce_index];

    do
    {
        if ((ce_ptr->ce_status != LMP_STANDBY) &&
            (ce_ptr->phy_piconet_id == piconet_id) &&
            (ce_ptr->afh_mode == AFH_ENABLE) )
        {
#ifdef COMPILE_HOLD_MODE
            if (ce_ptr->ce_status == LMP_HOLD_MODE)
            {
                /*
                 * implicitly disabled in hold-mode.
                 * No further processing will be done.
                 */
                break;
            }
#endif
#ifdef COMPILE_PARK_MODE
            if (ce_ptr->ce_status == LMP_PARK_MODE)
            {
                /*
                 * implicitly disabled in park-mode.
                 * No further processing will be done.
                 */
                break;
            }
#endif

            return TRUE;
        }
    }
    while (0);

    return FALSE;
}

void lmp_piconet_start_afh_map_updation(UCHAR piconet_id, UCHAR* afh_map)
{
    UINT8 ce_index;

    if (!lc_sca_manager.pnet[piconet_id].active)
    {
        return;
    }

    /* Check the following:
       1. Connection exists.
       2. AFH is enabled. */

    for (ce_index = 0; ce_index < LMP_MAX_CE_DATABASE_ENTRIES; ce_index++)
    {
        if (lmp_check_connection_entity_in_afh(ce_index, piconet_id))
        {
#ifdef _ROM_CODE_PATCHED_
            if (rcp_lmp_piconet_start_afh_map_updation != NULL)
            {
                if (rcp_lmp_piconet_start_afh_map_updation(&ce_index, piconet_id))
                {
                    continue;
                }
            }
#endif
#ifdef TMP_PATCH_AFH_ONLY_ONE_CHANNEL
            RT_BT_LOG(RED, DAPE_TEST_LOG293, 1, 8888);
            memset(afh_map, 0x00, 10);   

            UINT16 reg_value;
            reg_value = BB_read_baseband_register(SCO_PACKET_TYPE_REGISTER);
            reg_value |= TEST_MODE_BASEBAND_HOP_1;
            BB_write_baseband_register(SCO_PACKET_TYPE_REGISTER, reg_value);
   
            afh_map[0] = 1;
            UINT16 ch_num;                                      
            ch_num = 0;
            BB_write_baseband_register(0xC4, (ch_num << 8) | ch_num);
#endif

            lmp_start_afh_map_updation(afh_map,ce_index,AFH_ENABLE);
        }
    }
    return;
}

UCHAR lmp_get_any_afh_enabled_connections(UCHAR piconet_id)
{
    UINT8 ce_index;

    if (!lc_sca_manager.pnet[piconet_id].active)
    {
        return FALSE;
    }

    /* Check the following:
       1. Connection exists.
       2. AFH is enabled. */

    for (ce_index = 0; ce_index < LMP_MAX_CE_DATABASE_ENTRIES; ce_index++)
    {
        if (lmp_check_connection_entity_in_afh(ce_index, piconet_id))
        {
            return TRUE;
        }
    }
    return FALSE;
}


#ifdef _NEW_MODEM_PSD_SCAN_
void modem_psd_man_init(void)
{
    // TODO:

#ifdef _ROM_CODE_PATCHED_
    if (rcp_modem_psd_man_init_func != NULL)
    {
        if (rcp_modem_psd_man_init_func(NULL))
        {
            return;
        }
    }   
#endif

    UINT8 ii;

    // modem_psd_scan_man //
    /* mark by austin. it can not reset from sw reset and cause timer leakage 
       (move from strcture to another global variables */
#if 0       
    modem_psd_scan_man.scan_period_timer = OS_INVALID_HANDLE;
    modem_psd_scan_man.scan_period_timeout_ms = 500;
    modem_psd_scan_man.scan_tdm_timer = OS_INVALID_HANDLE;
    modem_psd_scan_man.scan_tdm_timeout_ms = 100;
    modem_psd_scan_man.afh_map_gen_timer = OS_INVALID_HANDLE;
    modem_psd_scan_man.afh_map_gen_timerout_ms = 2500;
#endif

    modem_psd_scan_man.scan_period_state = MODEM_PSD_SCAN_STATE_INIT; // TODO: need this?
    modem_psd_scan_man.scan_period_cnt = 0; // to count how many times the scan_period_timer is experienced
    modem_psd_scan_man.scan_period_cnt_th = 4;
    modem_psd_scan_man.scan_tdm_state = MODEM_PSD_SCAN_TDM_STATE_INIT; // TODO: need this?
    modem_psd_scan_man.scan_tdm_number = 1; 
    modem_psd_scan_man.scan_tdm_cnt = 0;
//    modem_psd_scan_man.scan_tdm_ch_start = 0; // to be calculate at each tdm scan //
    modem_psd_scan_man.scan_tdm_ch_step = (g_efuse_modem_psd_setting_1.psd_scan_3ch_mode) ? 3 : 1;
//    modem_psd_scan_man.scan_tdm_ch_stop = 78; // to be calculate at each tdm scan //

#ifdef _YL_TEST_NEW_MODEM_PSD_SCAN_FOR_AFH_
    if (g_modem_psd_report_entry_num == 0)
    {
        RT_BT_LOG(RED, MDM_LOG_081, 0, 0);
    }
    else
    {
        if (g_efuse_modem_psd_setting_1.psd_scan_3ch_mode == 0) // 1 ch / scan
        {
            // ENTRY NUM = 9/27/79 ==> tdm number = 9/3/1 //
            // Note: TDM NUMBER can be further increased for detection diversity //
            modem_psd_scan_man.scan_tdm_number = 79/g_modem_psd_report_entry_num+((79%g_modem_psd_report_entry_num) != 0);
        }
        else // 3 ch's / scan
        {
            // ENTRY NUM = 9/27/79 ==> tdm number = 3/1/1 //
            // Note: TDM NUMBER can be further increased for detection diversity //
            modem_psd_scan_man.scan_tdm_number = 27/g_modem_psd_report_entry_num+((27%g_modem_psd_report_entry_num) != 0);
        }
    }
    modem_psd_scan_man.scan_tdm_ch_step = modem_psd_scan_man.scan_tdm_number*(1+g_efuse_modem_psd_setting_1.psd_scan_3ch_mode*2);
#endif    

    // modem_psd_result_man //
    memset(&modem_psd_result_man, 0, sizeof(MODEM_PSD_RESULT_MANAGER_TYPE));    
    //modem_psd_result_man.psd_update_cnt_within_la_period;
    //modem_psd_result_man.psd_dbm_add200_signle[LMP_MAX_CHANNELS];
    //modem_psd_result_man.psd_dbm_add200_max_hold[LMP_MAX_CHANNELS];
    //modem_psd_result_man.psd_dbm_add200_max_hold_d[LMP_MAX_CHANNELS]; 
    //modem_psd_result_man.piconet_rssi_dbm_add200[MAX_PICONET_CNT]; 
    //modem_psd_result_man.afh_map[MAX_PICONET_CNT][LMP_AFH_MAP_SIZE];
    //modem_psd_result_man.afh_map_launch_cnt_latch[MAX_PICONET_CNT]; 
    //modem_psd_result_man.afh_map_launch_cnt_latch_d[MAX_PICONET_CNT]; 
    modem_psd_result_man.afh_map_launch_cnt = 0x40;
    //modem_psd_result_man.psd_update_cnt_within_la_period = 0; // same as scan_period_cnt ?? //

    for (ii=0; ii<LMP_MAX_PICONETS_SUPPORTED; ii++)
    {
        memcpy(modem_psd_result_man.afh_map[ii], afh_ahs79_channel_map, LMP_AFH_MAP_SIZE);
    }

    
}


void modem_psd_man_refresh_at_la_period_timer(void)
{
    // TODO: 

    UINT8 ii;

#ifdef _ROM_CODE_PATCHED_
    if (rcp_modem_psd_man_refresh_at_la_period_timer_func != NULL)
    {
        if (rcp_modem_psd_man_refresh_at_la_period_timer_func(NULL))
        {
            return;
        }
    }   
#endif
    // modem_psd_scan_man //
    modem_psd_scan_man.scan_period_state = MODEM_PSD_SCAN_STATE_INIT; // TODO: need this?    
    modem_psd_scan_man.scan_period_cnt = 0;
    modem_psd_scan_man.scan_tdm_state = MODEM_PSD_SCAN_TDM_STATE_INIT; // TODO: need this?
    modem_psd_scan_man.scan_tdm_cnt = 0;
    // to latch the g_efuse_modem_psd_setting_1.psd_scan_3ch_mode during psd_scan //
//    modem_psd_scan_man.scan_tdm_ch_start = 0; // to be calculate at each tdm scan //
#ifndef _YL_TEST_NEW_MODEM_PSD_SCAN_FOR_AFH_
    modem_psd_scan_man.scan_tdm_ch_step = (g_efuse_modem_psd_setting_1.psd_scan_3ch_mode) ? 3 : 1;
#endif    
//    modem_psd_scan_man.scan_tdm_ch_stop = 78; // to be calculate at each tdm scan //

    // modem_psd_result_man //
    memcpy(modem_psd_result_man.psd_dbm_add200_max_hold_d, modem_psd_result_man.psd_dbm_add200_max_hold, LMP_MAX_CHANNELS);
    memset(modem_psd_result_man.psd_dbm_add200_max_hold, 0, LMP_MAX_CHANNELS);
    
    modem_psd_result_man.psd_update_cnt_within_la_period = 0;
    modem_psd_result_man.afh_map_launch_cnt++;
    for (ii = 0; ii<MAX_PICONET_CNT; ii++)
    {
        if (modem_psd_result_man.afh_map_launch_cnt_latch[ii] == modem_psd_result_man.afh_map_launch_cnt)
        {
            modem_psd_result_man.afh_map_launch_cnt_latch_d[ii] = modem_psd_result_man.afh_map_launch_cnt+1;
            modem_psd_result_man.afh_map_launch_cnt_latch[ii] = modem_psd_result_man.afh_map_launch_cnt+1;
        }
        else
        {
            modem_psd_result_man.afh_map_launch_cnt_latch_d[ii] = modem_psd_result_man.afh_map_launch_cnt_latch[ii];
        }
    }
}




/*=======================================================================================================
 * la_period_timer --> immediate modem_psd_scan_period_timer(10ms) --> immediate modem_psd_scan_tdm_timer(10ms)
 *      |                   |                                            |
 *      |                   |----> modem_psd_scan_period_timer           |----> modem_psd_scan_tdm_timer
 *      |                             |                                            ...
 *      |                             |----> modem_psd_scan_period_timer           ...
 *      |                                       |
 *      |                                       |----> modem_psd_scan_period_timer --> immediate modem_psd_scan_tdm_timer_func
 *      |                                                                                 |
 *      |                                                                                   ...
 *      |                                                                                 |----> last modem_psd_scan_tdm_timer: 
 *      |                                                                                            if (modem_psd_afh_map_gen_instant_opt == 0)  modem_psd_afh_map_gen_timer
 *      |
 *      |------------------------------- (modem_psd_afh_map_gen_instant_opt = 1) --------------------> modem_psd_afh_map_gen_timer
 *
 *
 * modem_psd_scan_period_timer: timer for launch each scan of whole 79 channels
 * modem_psd_scan_tdm_timer: timer for time-diversity scan; e.g. 
 *                           a. 1:9:73 then 4:9:76 then 7:9:79 (3 channels per calculation)
 *                           b. 0:4:78 then 1:4:76 then 2:4:77 then 3:4:78 (1 channel per calculation)
 * NOTE: the time for calculating the AFH MAP is to be tested and can be moved when it is too time-consuming
 *======================================================================================================*/
 
void lmp_modem_psd_scan_period_timer_expiry(TimerHandle_t timer_handle)
{
    // TODO: need to check if the period-scan is finished, then decide if PAUSE_ACL or promote the PRIORITY, re-issue or Giveup//
#ifdef _ROM_CODE_PATCHED_
    if (rcp_lmp_modem_psd_scan_period_timer_expiry != NULL)
    {
        if ( rcp_lmp_modem_psd_scan_period_timer_expiry(timer_handle) )
        {
            return;
        }
    }   
#endif    
#ifdef _YL_TEST_NEW_MODEM_PSD_SCAN_FOR_AFH_
    // to issue the timer expiry function "IMMEDIATELY" //

    if (modem_psd_scan_man.scan_period_state != MODEM_PSD_SCAN_STATE_RUN)
    {
        modem_psd_scan_man.scan_tdm_cnt = 0;
        modem_psd_scan_man.scan_period_state = MODEM_PSD_SCAN_STATE_RUN;
        
        OS_START_TIMER(scan_tdm_timer, 10);        
        
        modem_psd_scan_man.scan_period_cnt++;

        if (modem_psd_scan_man.scan_period_cnt > 1)
        { 
            UINT8 ch;
            for (ch = 0; ch<79; ch++)
            {
                modem_psd_result_man.psd_dbm_add200_max_hold[ch] = 
                        MAX(modem_psd_result_man.psd_dbm_add200_max_hold[ch],modem_psd_result_man.psd_dbm_add200_signle[ch]);
            }
        }
        
        if (modem_psd_scan_man.scan_period_cnt < modem_psd_scan_man.scan_period_cnt_th)
        {
            OS_START_TIMER(scan_period_timer, scan_period_timeout_ms);
        }
        
      #ifdef _YL_TEST_NEW_MODEM_PSD_SCAN_FOR_AFH_LOG_
    //    RT_BT_LOG(YELLOW, MDM_LOG_041, 1, modem_psd_scan_man.scan_period_cnt);
      #endif
    }
    else
    {
        // TODO: Execption Processing is improtant !!!

    }
    
#endif
}

void lmp_modem_psd_scan_tdm_timer_expiry(TimerHandle_t timer_handle)
{
    // TODO: need to check if the tdm-scan is finished, then decide if PAUSE_ACL or promote the PRIORITY, or Giveup//
#ifdef _ROM_CODE_PATCHED_
    if (rcp_lmp_modem_psd_scan_tdm_timer_expiry != NULL)
    {
        if (rcp_lmp_modem_psd_scan_tdm_timer_expiry(timer_handle))
        {
            return;
        }
    }   
#endif    
#ifdef _YL_TEST_NEW_MODEM_PSD_SCAN_FOR_AFH_
    modem_psd_scan_man.scan_tdm_ch_start = g_efuse_modem_psd_setting_1.psd_scan_3ch_mode 
                                        + modem_psd_scan_man.scan_tdm_cnt*(g_efuse_modem_psd_setting_1.psd_scan_3ch_mode ? 3 : 1);
    modem_psd_scan_man.scan_tdm_ch_stop = modem_psd_scan_man.scan_tdm_ch_start + 
                                                        (g_modem_psd_report_entry_num - 1)*
                                                        modem_psd_scan_man.scan_tdm_ch_step;
    if (modem_psd_scan_man.scan_tdm_ch_stop>78)
    {
        modem_psd_scan_man.scan_tdm_ch_stop -= modem_psd_scan_man.scan_tdm_ch_step;
    }
    lc_psd_bluewiz_set_parameters(modem_psd_scan_man.scan_tdm_ch_start, modem_psd_scan_man.scan_tdm_ch_step, modem_psd_scan_man.scan_tdm_ch_stop, PSD_PARA_HALF_SLOT_MODE, 0x3FF);
    lc_psd_set_psd_en(0);
    lc_psd_set_psd_en(1); // to issue the BLuewiz PSD Scan Command, which will result in BB_handle_psd_end_intr() //

    if (modem_psd_scan_man.scan_tdm_state != MODEM_PSD_SCAN_TDM_STATE_RUN)
    {
        modem_psd_scan_man.scan_tdm_cnt++;
        modem_psd_scan_man.scan_tdm_state = MODEM_PSD_SCAN_TDM_STATE_RUN;
        if (modem_psd_scan_man.scan_tdm_cnt < modem_psd_scan_man.scan_tdm_number)
        {
            OS_START_TIMER(scan_tdm_timer, scan_tdm_timeout_ms);
        }
        else
        {
            modem_psd_scan_man.scan_period_state = MODEM_PSD_SCAN_STATE_END;
        }                
    }
    else
    { 
        // TODO: Execption Processing is improtant !!!
      #ifdef _YL_TEST_NEW_MODEM_PSD_SCAN_FOR_AFH_LOG_
        RT_BT_LOG(RED, MDM_LOG_044, 1, modem_psd_scan_man.scan_tdm_state);
      #endif
        if (modem_psd_scan_man.scan_tdm_cnt < modem_psd_scan_man.scan_tdm_number)
        {
            OS_START_TIMER(scan_tdm_timer, scan_tdm_timeout_ms/4); // to check again soon //
        }
    }

  #ifdef _YL_TEST_NEW_MODEM_PSD_SCAN_FOR_AFH_LOG_
    if (modem_psd_scan_man.scan_period_cnt == 1)
    {
//        if (modem_psd_scan_man.scan_tdm_cnt == modem_psd_scan_man.scan_tdm_number)
        {
//            RT_BT_LOG(YELLOW, MDM_LOG_042, 4, modem_psd_scan_man.scan_tdm_cnt, modem_psd_scan_man.scan_tdm_ch_start, modem_psd_scan_man.scan_tdm_ch_step, modem_psd_scan_man.scan_tdm_ch_stop);
        }
    }
  #endif
    
#endif
}

void lmp_modem_psd_afh_map_gen_timer_expiry(TimerHandle_t timer_handle)
{
    // TODO: need to check if the ALL period-scan's are finished, then decide if Giveup or Used previous results or ... FW max-hold/Average, ...//
    // TODO: Execption Processing is improtant !!!
#ifdef _ROM_CODE_PATCHED_
    if (rcp_lmp_modem_psd_afh_map_gen_timer_expiry != NULL)
    {
        if ( rcp_lmp_modem_psd_afh_map_gen_timer_expiry(timer_handle) )
        {
            return;
        }
    }   
#endif    
#ifdef _YL_TEST_NEW_MODEM_PSD_SCAN_FOR_AFH_
  #ifdef _YL_TEST_NEW_MODEM_PSD_SCAN_FOR_AFH_LOG_
    {
        UINT8 ch;
        for (ch = 0; ch<79; ch+=20)
        {
            RT_BT_LOG(GREEN, MDM_LOG_045, 21, ch,  
                    modem_psd_result_man.psd_dbm_add200_signle[ch],
                    modem_psd_result_man.psd_dbm_add200_signle[ch+1],
                    modem_psd_result_man.psd_dbm_add200_signle[ch+2],
                    modem_psd_result_man.psd_dbm_add200_signle[ch+3],
                    modem_psd_result_man.psd_dbm_add200_signle[ch+4],
                    modem_psd_result_man.psd_dbm_add200_signle[ch+5],
                    modem_psd_result_man.psd_dbm_add200_signle[ch+6],
                    modem_psd_result_man.psd_dbm_add200_signle[ch+7],
                    modem_psd_result_man.psd_dbm_add200_signle[ch+8],
                    modem_psd_result_man.psd_dbm_add200_signle[ch+9],
                    modem_psd_result_man.psd_dbm_add200_signle[ch+10],
                    modem_psd_result_man.psd_dbm_add200_signle[ch+11],
                    modem_psd_result_man.psd_dbm_add200_signle[ch+12],
                    modem_psd_result_man.psd_dbm_add200_signle[ch+13],
                    modem_psd_result_man.psd_dbm_add200_signle[ch+14],
                    modem_psd_result_man.psd_dbm_add200_signle[ch+15],
                    modem_psd_result_man.psd_dbm_add200_signle[ch+16],
                    modem_psd_result_man.psd_dbm_add200_signle[ch+17],
                    modem_psd_result_man.psd_dbm_add200_signle[ch+18],
                    modem_psd_result_man.psd_dbm_add200_signle[ch+19]);
        }
        for (ch = 0; ch<79; ch+=20)
        {
            RT_BT_LOG(WHITE, MDM_LOG_046, 21, ch, 
                    modem_psd_result_man.psd_dbm_add200_max_hold[ch],
                    modem_psd_result_man.psd_dbm_add200_max_hold[ch+1],
                    modem_psd_result_man.psd_dbm_add200_max_hold[ch+2],
                    modem_psd_result_man.psd_dbm_add200_max_hold[ch+3],
                    modem_psd_result_man.psd_dbm_add200_max_hold[ch+4],
                    modem_psd_result_man.psd_dbm_add200_max_hold[ch+5],
                    modem_psd_result_man.psd_dbm_add200_max_hold[ch+6],
                    modem_psd_result_man.psd_dbm_add200_max_hold[ch+7],
                    modem_psd_result_man.psd_dbm_add200_max_hold[ch+8],
                    modem_psd_result_man.psd_dbm_add200_max_hold[ch+9],
                    modem_psd_result_man.psd_dbm_add200_max_hold[ch+10],
                    modem_psd_result_man.psd_dbm_add200_max_hold[ch+11],
                    modem_psd_result_man.psd_dbm_add200_max_hold[ch+12],
                    modem_psd_result_man.psd_dbm_add200_max_hold[ch+13],
                    modem_psd_result_man.psd_dbm_add200_max_hold[ch+14],
                    modem_psd_result_man.psd_dbm_add200_max_hold[ch+15],
                    modem_psd_result_man.psd_dbm_add200_max_hold[ch+16],
                    modem_psd_result_man.psd_dbm_add200_max_hold[ch+17],
                    modem_psd_result_man.psd_dbm_add200_max_hold[ch+18],
                    modem_psd_result_man.psd_dbm_add200_max_hold[ch+19]);
        }
    }
  #endif

#endif   
}

#endif

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//==============================================End All Use Functions====//
//
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/**
 * Handles the timer expiry of channel assessmet period timer. Stops
 * host classification update timer and calls common timer expiry handler
 * to calculate channel map/classification.
 *
 * \param timer_handle Handle of AFH timer expired.
 * \param arg Arguements passed to the timer module.
 *
 * \return None.
 */
void lmp_handle_channel_assess_timer_expiry(TimerHandle_t timer_handle)
{
//================ Note============Jr. Neil ====2010-09-20=====//
// PS. lmp_handle_channel_assess_timer_expiry is the timing counter for AFH mechanis
//       this timing counter is repeat and repeat.......
//======================================================

#ifdef _YL_TEST_NEW_MODEM_PSD_SCAN_FOR_AFH_LOG_
//    RT_BT_LOG(YELLOW, MDM_LOG_040, 0, 0);
#endif

#ifdef _ROM_CODE_PATCHED_
    if (rcp_lmp_handle_channel_assess_timer_expiry_func != NULL)
    {
        if ( rcp_lmp_handle_channel_assess_timer_expiry_func(timer_handle) )
        {
            return;
        }
    }   
#endif
    //lmp handle channel assess timer expiry
    RTK_AFH_LOG(GREEN,AFH_HANDLE_CHANNEL_ASSESS_TIMER,1,afh_la_cycle_period );

//##############################################################
// need to check -----
//========Add RTK_AFH_MECHANISM Branch by Jr. Neil =======//

// Add by neil Chen to check 2 task for AFH

//lmp_cal_afh_channel_quality_RTK(piconet_id,&avg_pkt_def);     //cal every channel PGR/PLR/....etc
    if (lmp_generate_afh_map_for_device_RTK() == 0x0) // this means no any connection
    {
        bt_afh_rssi.rssi_pkts = 0x0;
        bt_afh_rssi.rssi_count = 0x0;	

        // to end the AFH recovery state
        //---to reset the parameters
        // Add by Jr. Neil ---2010--09--20---

        /* speed up process time */
        memset(afh_ch_status, 0, sizeof(AFH_CH_ASSESSMENT_NODE)*LMP_MAX_CHANNELS);
        memset(afh_ch_quality, 0, sizeof(CHANNEL_CLASSIFY_NODE)*LMP_MAX_CHANNELS);
    }
//############################################

//=====================================================
// add by Neil Chen to start the channel classify timer
//---------------------------------

    OS_START_TIMER(la_period_timer, afh_la_cycle_period);

#ifdef _NEW_MODEM_PSD_SCAN_ 
    if ( (!g_rtk_afh_wlan_psd_internal_enable) || 
        (g_efuse_modem_psd_setting_1.force_channel_classify_on &&
         otp_str_data.rtk_afh_bt_psd_enable))
#else
    if (!g_rtk_afh_wlan_psd_internal_enable)
#endif    
    {
        OS_START_TIMER(la_classify_timer, afh_channel_classify_period);
    }	

#ifdef _NEW_MODEM_PSD_SCAN_ 
    // TODO: patch_lmp_handle_channel_assess_timer_expiry in RTL8723A
    if (g_rtk_afh_modem_psd_internal_enable && otp_str_data.rtk_afh_bt_psd_enable)
    {
        // Note: Re-start modem_psd timers in lmp_generate_afh_map_for_device_RTK() //
        modem_psd_man_refresh_at_la_period_timer(); // TODO: need to handle the case of AFH --> STOP --> re-init AFH
    }
#endif
    
//---------------------

    return;
}

void lmp_handle_afh_start_timer_bottom_half(void *no_arg, uint32_t no_arg2)
{
    OS_CREATE_TIMER(ONESHOT_TIMER, &la_mailbox_timer,
            lmp_handle_channel_classify_timer_expiry_II, NULL, 0);
    OS_START_TIMER(la_mailbox_timer, la_mailbox_one_shot_duration);
}

UINT8 lmp_handle_afh_start_timer(UINT8 piconet)
{
    if (lmp_get_any_afh_enabled_connections(piconet))
    {
        lmp_cal_afh_channel_quality_RTK(piconet);
        afh_piconet = piconet;
        // call back to do lmp cal II part
        if (la_mailbox_timer == NULL)
        {
            if (IN_ISR())
            {
                BaseType_t high_pri_task_woken = pdFALSE;
                xTimerPendFunctionCallFromISR(
                        lmp_handle_afh_start_timer_bottom_half, NULL, 0,
                        &high_pri_task_woken);
                portYIELD_FROM_ISR(high_pri_task_woken);
                return TRUE;
            }

            OS_CREATE_TIMER(ONESHOT_TIMER, &la_mailbox_timer,
                    lmp_handle_channel_classify_timer_expiry_II, NULL, 0);
        }

        OS_START_TIMER(la_mailbox_timer, la_mailbox_one_shot_duration);     
        return TRUE;
    }
    return FALSE;
}

//=======================================================
/**
 * This function should be invoked when a set host channel
 * classification Command received from the host.
 * This function update the host provided channel classification
 * variable and update local assessesment data structure accordingly.
 * \param host_ch_cl : Host provided channel classification
 *                     in spec defined format.
 *
 * \return None.
 */
//##########################################################
// Need to check  how to use the function into the design
//########################################### Jr. Neil--2010-09-20

// Add by Neil Chen to handle channel classify function by periodic timer
void lmp_handle_channel_classify_timer_expiry(TimerHandle_t timer_handle)
{
    UCHAR i;	

#ifdef _ROM_CODE_PATCHED_
    if (rcp_lmp_handle_channel_classify_timer_expiry_func != NULL)
    {
        if ( rcp_lmp_handle_channel_classify_timer_expiry_func(timer_handle) )
        {
            return;
        }
    }   
#endif	
    RTK_AFH_LOG(GREEN,CHANNEL_CLASSIFY_TIMER,0,0);	

    // have connection need to do channel classification	
    if (lmp_self_device_data.number_of_acl_conn)
    {
        //========================================================
        // need to note that this function code should only support one piconet //---add by Jr. Neil Chen 2010-08-13
        //==========================================================
        if (lc_sca_manager.master_cnt)// has master piconet
        {
            i = lc_sca_manager.master_id;		
            lmp_handle_afh_start_timer(i);
        }
        else   // no master only slave
        {
            for (i = 0; i < LMP_MAX_PICONETS_SUPPORTED; i++)
            {
               if(lc_sca_manager.pnet[i].active)
               {
                    if (lmp_handle_afh_start_timer(i))
                    {
                        break;
                    }                      
               }
            }		   	     
        }   
    }
}

void lmp_handle_channel_classify_timer_expiry_II(TimerHandle_t timer_handle)
{
#ifdef _ROM_CODE_PATCHED_
    if (rcp_lmp_handle_channel_classify_timer_expiry_II_func != NULL)
    {
        if (rcp_lmp_handle_channel_classify_timer_expiry_II_func(timer_handle))
        {
            return;
        }
    }   
#endif

    RTK_AFH_LOG(GREEN,CHANNEL_CLASSIFY_TIMER,0,0);	
    lmp_cal_afh_channel_quality_RTK_II(afh_piconet);
}
/******************************************
* purpose :  update host channel 
* method :  update afh_ch_host by afh_channel_classification
* copy 
*
*   called by wifi and hci 
*    Save host classification in global variable 
*    lmp_update_host_afh_ch_classification(&afh_map[0]);
*
*
*	hci_handle_1_2_hc_bb_command
*	HCI_SET_AFH_HOST_CHANNEL_CLASSIFICATION_OPCODE
*	hci_handle_set_afh_host_channel_classification_command
*
*
*    to update the host information
*    mailbox command by wifi
*    WIFI_CHANNEL_AND_BANDWIDTH_CMD
*    lmp_update_host_afh_ch_classification(&wifi_host[0]);
* 
*	( wifi will notify "is wifi used" , the used BW and channels )
*	( they shall be masked in channel classification : afh_host_classification )
*	( and afh_ch_host )
*    memcpy afh_map or wifi_host to afh_host_classification
*    then 
*    remap afh_host_classification to afh_ch_host 
*    the value will re-calculate by 2 conjection channels 
*    only bit "1 1" map to "1 1" otherwise map to " 0 0"
*    
******************************************/

void lmp_update_host_afh_ch_classification(UCHAR* host_ch_cl)
{
    UCHAR byte_index,bit_index, ch_index;

    memcpy(&afh_host_classification[0], host_ch_cl, LMP_AFH_MAP_SIZE);

    // report the host classification information to check--Jr. Neil
    RT_BT_LOG(WHITE, LMP_CH_ASSESSMENT_358, 10,
              afh_host_classification[0],
              afh_host_classification[1],
              afh_host_classification[2],
              afh_host_classification[3],
              afh_host_classification[4],
              afh_host_classification[5],
              afh_host_classification[6],
              afh_host_classification[7],
              afh_host_classification[8],
              afh_host_classification[9]);

    for (ch_index = 0; ch_index < (LMP_MAX_CHANNELS - 2); ch_index+=2)
    {
        byte_index = ch_index >> 3;
        bit_index = ch_index & 0x07;
        // host_ch_status
        // HOST specify the channel as bad channel  ---Jr. Neil
        
        if (((afh_host_classification[byte_index] >> bit_index) & 0x03) != 0x03)
        {
            afh_ch_host[ch_index].host_ch_status = AFH_HOST_BAD_CHANNEL;
            afh_ch_host[ch_index+1].host_ch_status = AFH_HOST_BAD_CHANNEL;

//201200907 morgan add
// here can start to update afh_host_classification
//			afh_host_classification[byte_index] 

        }
        else    //Host not specify the channel as bad channel, .. so unknown channel --Jr. Neil
        {
            afh_ch_host[ch_index].host_ch_status = AFH_HOST_UNKNOWN_CHANNEL;
            afh_ch_host[ch_index+1].host_ch_status = AFH_HOST_UNKNOWN_CHANNEL;
            // need to do .....
        }
    }

    /* check channel 78 */
    if (((afh_host_classification[9] >> 6) & 0x01 ) == AFH_HOST_BAD_CHANNEL)
    {
        afh_ch_host[78].host_ch_status = AFH_HOST_BAD_CHANNEL;
    }
    else
    {
        afh_ch_host[78].host_ch_status = AFH_HOST_UNKNOWN_CHANNEL;
    }

    //========Interrupt by host and restart the AFH Instant ==========Jr. Neil====//
    if (la_period_timer == NULL)
    {
        OS_CREATE_TIMER(ONESHOT_TIMER, &la_period_timer,
                lmp_handle_channel_assess_timer_expiry, 0, afh_la_cycle_period);
    }
    
    /* Restart the timer */
    //OS_STOP_TIMER(la_period_timer, 0);
    OS_START_TIMER(la_period_timer, afh_la_cycle_period);

    return;

}


/**
 * Updates the channel quality with the RSSI value.
 *
 * \param channel_id The index of the channel 0-78
 * \param rssi The RSSI indiator of the channel.
 * \param pkt_type The pkt_type received on the channel.

 * \return None.
 */
void lmp_update_channel_quality(UCHAR channel_id, UINT16 rssi, UCHAR weightage)
{
    generate_new_afh_signal = TRUE;
    new_afh_rx_info.channel_id = channel_id;
    new_afh_rx_info.rssi = rssi;
    new_afh_rx_info.signal_type = LC_NEW_PKT_RECD_SIGNAL;

    switch (weightage)
    {
        case AFH_ACCESS_ERROR_PKT_WEIGHT:
            new_afh_rx_info.access_code_miss = TRUE;
            new_afh_rx_info.hec_err = TRUE;
            new_afh_rx_info.crc_err = TRUE;
            break;

        case AFH_HEC_ERROR_PKT_WEIGHT:
            new_afh_rx_info.access_code_miss = FALSE;
            new_afh_rx_info.hec_err = TRUE;
            new_afh_rx_info.crc_err = TRUE;
            break;

        case AFH_GOOD_POLL_NULL_PKT_WEIGHT:
        case AFH_GOOD_CRC_PKT_WEIGHT:
        case AFH_SCO_PKT_WEIGHT:
            new_afh_rx_info.access_code_miss = FALSE;
            new_afh_rx_info.hec_err = FALSE;
            new_afh_rx_info.crc_err = FALSE;
            break;

        case AFH_CRC_ERROR_PKT_WEIGHT:
            new_afh_rx_info.access_code_miss = FALSE;
            new_afh_rx_info.hec_err = FALSE;
            new_afh_rx_info.crc_err = TRUE;
            break;

        default:
            /* no such input */
            generate_new_afh_signal = FALSE;
            break;
    }
}

#ifdef _NEW_MODEM_PSD_SCAN_

//void lmp_update_modem_psd_data(void)
//{
//  // TODO: to be completed
//}

//#################################################################
// function : lmp_wifi_host_irq
// modify : 
// history : austin modify process speed. : ( 20120912 modify some bug of shift operation)
// NOTE: this should "smoothly" turn-off the modem_psd_function when e.g. Test Mode or ...  //
//####################################################################
void lmp_runtime_stop_modem_psd_scan(void)
{
    g_rtk_afh_modem_psd_internal_enable = 0;
    lc_psd_set_psd_en(0); // to stop the bluewiz scheduling as soon as possible //
    // TODO: any other actions required ?? e.g. modem_psd_scan_man, modem_psd_result_man

}
//#################################################################
// function : lmp_wifi_host_irq
// Create : 
// modify : 
// history : austin modify process speed. : ( 20120912 modify some bug of shift operation)
// NOTE: this should "smoothly" turn-off the modem_psd_function when e.g. Test Mode or ... //
//####################################################################
void lmp_runtime_restart_modem_psd_scan(void)
{
//    g_rtk_afh_modem_psd_internal_enable = 1;
    g_rtk_afh_modem_psd_internal_enable = g_efuse_modem_psd_setting_1.rtk_afh_modem_psd_enable;
    modem_psd_man_init();
    // lc_psd_set_psd_en(1); // to be restarted at la_period_timer ... //
    // TODO: any other actions required ?? e.g. modem_psd_scan_man, modem_psd_result_man
}

#endif

//#################################################################
// function : lmp_wifi_host_irq
//
// author:  Neil Chen 
// purpose : update wifi channel and BW to host channel classification
// modify : 
// history : austin modify process speed. : ( 20120912 modify some bug of shift operation)
// 
//####################################################################
void lmp_wifi_host_irq(UINT32 data0,UINT32 data1)
{

    UCHAR wifi_connect, wifi_ch,wifi_bw;
    UCHAR wifi_host[LMP_AFH_MAP_SIZE];
    CHAR mask_min;
    UCHAR mask_max;
    UCHAR freq; 
    
    wifi_connect = (data0 >> 16) & 0xFF;
    wifi_ch = (data0 >> 24);
    wifi_bw = (data1 & 0xFF);
 	 
#ifdef RTK_AFH_MECHANISM_DEBUG
       //RT_BT_LOG(YELLOW,WIFI_INFO_AFH,3,data0,data1,0);	
	RT_BT_LOG(YELLOW,WIFI_INFO_AFH,3,wifi_connect,wifi_ch,wifi_bw);	
#endif

    /* enable all channel to be default map */
    memcpy(wifi_host, afh_ahs79_channel_map, 10);  

    if ((wifi_connect == 1) && (wifi_bw != 0)) // 1: WIFI is using; 0: WIFI isn't using
    {
        /* mask some channels to be zeros from wifi's ireq */
        
        freq = 5 + (5 * wifi_ch);
        mask_min = freq - (wifi_bw >> 1);
        mask_max = freq + (wifi_bw >> 1);	

        if (mask_min & 0x01) // odd part issue
        {
            mask_min--;  
        }

        mask_min = MAX(mask_min, 0);
        mask_max = MIN(mask_max, LMP_MAX_CHANNELS);

        /* faster process (min loops) by austin */

        UINT8 offset_min;
        UINT8 offset_min_mod;
        UINT8 offset_max;
        UINT8 offset_max_mod;

        offset_min = mask_min >> 3;
        offset_max = mask_max >> 3;        
        offset_min_mod = mask_min & 0x07;        
        offset_max_mod = mask_max & 0x07;

        if (offset_max_mod > 0)
        {
            /* zero left half part :
               i.e. 1111111 ---> 00001111 */            
            wifi_host[offset_max] >>= (offset_max_mod + 1);
            wifi_host[offset_max] <<= (offset_max_mod + 1);
        }

        if (offset_min == offset_max)
        {           
            /* fill left half part  :
               i.e. 00001111 ---> 11001111 */
            wifi_host[offset_min] |= (1 << offset_min_mod) - 1;
        }
        else
        {
            /* zero right half part :
               i.e. 11111111 ---> 11110000 */
            wifi_host[offset_min] = (1 << offset_min_mod) - 1;

            /* zeros masked part */
            UINT8 index;
            for (index = offset_min + 1; index < offset_max; index++)
            {
                wifi_host[index] = 0;
            }            
        }

#ifdef RTK_AFH_MECHANISM_DEBUG
        RTK_AFH_LOG(YELLOW,MASK_INFO,2,mask_min,mask_max);  
        //RTK_AFH_LOG(YELLOW,WIFI_INFO_AFH,3,wifi_connect,wifi_ch,wifi_bw); 
#endif
    }
    
#ifdef RTK_AFH_MECHANISM_DEBUG	
    //RTK_AFH_LOG(GREEN,GENERATE_AFH_MAP_FOR_RECOVERY,10,
    RTK_AFH_LOG(GREEN,AFH_WIFI_SET_CH_BW,10,
                  wifi_host[0],
                  wifi_host[1],
                  wifi_host[2],
                  wifi_host[3],
                  wifi_host[4],
                  wifi_host[5],
                  wifi_host[6],
                  wifi_host[7],
                  wifi_host[8],
                  wifi_host[9]);
#endif

	memcpy( afh_wifi_set_ch_bw,wifi_host,10);	
    // to update the host information
    lmp_update_host_afh_ch_classification(&wifi_host[0]);
}


//#################################################################
// function : lmp_generate_afh_map_by_modem_psd
// create : YiLin 20120901 
// author:  
// purpose : 
// modify : 
// history : 
// 
//##################################################################

#ifdef _NEW_MODEM_PSD_SCAN_
void lmp_generate_afh_map_by_modem_psd(UCHAR piconet_id, UCHAR* afh_map_modem_psd)
{
    // TODO: to be completed
#ifdef _ROM_CODE_PATCHED_
    if (rcp_lmp_generate_afh_map_by_modem_psd_func != NULL)
    {
        if ( rcp_lmp_generate_afh_map_by_modem_psd_func((void*)afh_map_modem_psd, piconet_id) )
        {
            return;
        }
    }   
#endif

}

void lmp_generate_afh_map_by_psd_recovery(UCHAR piconet_id, UCHAR* afh_map_modem_psd)
{
	    // TODO: to be completed
#ifdef _ROM_CODE_PATCHED_
    if (rcp_lmp_generate_afh_map_by_psd_recovery_func != NULL)
    {
        if ( rcp_lmp_generate_afh_map_by_psd_recovery_func((void*)afh_map_modem_psd, piconet_id) )
        {
            return;
        }
    }   
#endif
}

#endif
//#################################################################
// function : lmp_generate_afh_map
// create :  Neil Chen
// author:  
// purpose : 
// modify : Morgan, YiLin
// history : 20120910
//  1. add set bt channel quality afh mode ( or bt psd mode )
//  2. add lmp_generate_afh_map_by_modem_psd
//  3. add patch 
//##################################################################

void lmp_generate_afh_map(UCHAR piconet_id, UCHAR* afh_map)
{   
#ifdef _NEW_MODEM_PSD_SCAN_
    UCHAR afh_map_modem_psd[LMP_AFH_MAP_SIZE];
#endif

#ifdef _ROM_CODE_PATCHED_
    if (rcp_lmp_generate_afh_map_func != NULL)
    {
        if ( rcp_lmp_generate_afh_map_func((void*)afh_map, piconet_id) )
        {
            return;
        }
    }   
#endif

#if 0 //unused logic
#ifdef RTK_AFH_USING_SLAVE_REPORT
    if (otp_str_data.rtk_afh_using_slave_report)
    {
        UCHAR slave_ch_assessment[LMP_MAX_CHANNELS];
        lmp_ch_la_update_la_with_ch_cl_or_afh_map_RTK(
                                    &slave_ch_assessment[0],
                                    piconet_id,
                                    lc_sca_get_piconet_role(piconet_id));
    }
#endif
#endif

    //Display the generate the AFH map for device RTK
	RTK_AFH_LOG(RED,AFH_DISPLAY_MAP_STATE,3,piconet_id, afh_state[piconet_id],lmp_self_device_data.afh_channel_assessment_mode);
	
// 20120917 morgan add some mode setting here
	// RECOVERY FSM
	if ( bBtPsdMode == 0 )
	{
#ifdef _DAPE_GROUP_AFH_RESET_TO_FUNCTION //// dape move to lmp_afh_state_reset(piconet_id);
            lmp_afh_state_reset(piconet_id);

#else
            // chang state              
            afh_state[piconet_id] = INITIAL; 
            
            //g_afh_times[piconet_id] = 0;		// wifi use this variable			
            // reset state
            afh_recovery_cnt[piconet_id] = 0;			        	
            afh_recovery_cnt1[piconet_id] = 0;
            afh_recovery_cnt2[piconet_id] = 0;
            afh_recovery_cnt3[piconet_id] = 0;
            afh_recovery_cnt4[piconet_id] = 0;
            all_on_map_cnt[piconet_id] = 0;			

#endif
	}

    if (lmp_self_device_data.afh_channel_assessment_mode == AFH_ENABLE)  //=====check AFH enable or not ---Jr. Neil
    {
#ifdef _NEW_MODEM_PSD_SCAN_
        if (g_rtk_afh_modem_psd_internal_enable && (otp_str_data.rtk_afh_bt_psd_enable==1))
        {
            lmp_generate_afh_map_by_modem_psd(piconet_id, afh_map_modem_psd);
            if (g_efuse_modem_psd_setting_1.lmp_gen_afh_map_use_modem_psd_only) // TODO: 
            {
                memcpy(afh_map, afh_map_modem_psd, LMP_AFH_MAP_SIZE);
                return;
            }
        }
#endif

        {      
            UINT8 i;
            if(afh_state[piconet_id] == INITIAL)   // Add Initial State to avoid the connection problem
            {
                for (i = 0; i < LMP_AFH_MAP_SIZE; i++)
                {
                    afh_map[i] = (afh_ahs79_channel_map[i] & afh_host_classification[i]);
                }

    			if ( bBtPsdMode ) // 20120912 morgan add for set all on + host_classification option
    			{						
                    g_afh_times[piconet_id]++;
                    if (g_afh_times[piconet_id] > INITIAL_TIMES)
                    {
                        afh_state[piconet_id] = NORMAL;
                        g_afh_times[piconet_id] = 0x0;
                        OS_START_TIMER(la_classify_timer, afh_channel_classify_period); // add by Neil Chen to start the channel classify timer
                    }  	   		
                }
    		}			
    		//20120926 morgan add 		
    		else if ( afh_state[piconet_id] == HOLD )
    		{								
    			memcpy(afh_map,&afh_bt_hold_state_map[piconet_id][0], 10);
    			if (!pta_meter_var.bPtaMeterSwitch || 
                     (( pta_meter_var.dwPtaACLRxCntStore + pta_meter_var.dwPtaACLTxCntStore) >= 5 ) ||
    			     ((pta_meter_var.dwPtaSCORxCntStore + pta_meter_var.dwPtaSCOTxCntStore ) >= 5 ))
    			{
    				afh_state[piconet_id] = NORMAL;				
    				RTK_AFH_LOG(BLUE,AFH_SET_STATE,1,afh_state[piconet_id] );		
    			}						
    		}		
            else
            {
    			// here add the hold state check
    			//20120926 morgan add 
    			if(bFullRandomRecover)
    			{			
                    if (pta_meter_var.bPtaMeterSwitch && 
                        (( pta_meter_var.dwPtaACLRxCntStore + pta_meter_var.dwPtaACLTxCntStore) < 5 ) && 
                        ((pta_meter_var.dwPtaSCORxCntStore + pta_meter_var.dwPtaSCOTxCntStore ) < 5 ))
                    {
                    	afh_state[piconet_id] = HOLD;
                    	memcpy(afh_map,afh_map_last, 10);
                    	memcpy(afh_bt_hold_state_map[piconet_id],afh_map_last, 10);
                    	RTK_AFH_LOG(BLUE,AFH_SET_STATE,1,afh_state[piconet_id] );
                    }		
    			}
    			
                if (afh_state[piconet_id] == RECOVERY)
                {
#ifdef _NEW_MODEM_PSD_SCAN_
        		    if (g_rtk_afh_modem_psd_internal_enable && (otp_str_data.rtk_afh_bt_psd_enable==2))
        		    {
        		        RTK_AFH_LOG(GREEN,BTPSD_RECOVERY,0,0);	
        		        lmp_generate_afh_map_by_psd_recovery(piconet_id, afh_map_modem_psd);
        		    }
                    else
#endif
                    {
                        if(bFullRandomRecover)
                        {
                            lmp_generate_afh_map_for_AFH_FULL_RANDOM_RECOVERY(piconet_id, afh_map);		        
                        }
                        else
                        {									
                            lmp_generate_afh_map_for_AFH_RECOVERY(piconet_id, afh_map);
                        }
                    }
                }
                else if (afh_state[piconet_id] == NORMAL)
                {                
                    //Display the generate the AFH map for device RTK
                    	//RTK_AFH_LOG(YELLOW,AFHSTATE_NORMAL,0,0); 
                    lmp_generate_afh_map_from_PGR_RTK(piconet_id, afh_map);
                }
            }
		
         }			  
    }			  
    else   // generate map by host information
    {
        memcpy(afh_map, afh_host_classification, 10);   

        RTK_AFH_LOG(RED,AFH_MAP_BY_HOST,10,
                          afh_map[0],
                          afh_map[1],
                          afh_map[2],
                          afh_map[3],
                          afh_map[4],
                          afh_map[5],
                          afh_map[6],
                          afh_map[7],
                          afh_map[8],
                          afh_map[9]);		
    }
#ifdef _NEW_MODEM_PSD_SCAN_
#ifdef _ROM_CODE_PATCHED_
    /*******************************************************************
     * (yilinli)
     * To decide: use channel classification or modem_psd or hybrid
     *
     *******************************************************************/
    if (rcp_lmp_generate_afh_map_post_func != NULL)
    {
        rcp_lmp_generate_afh_map_post_func((void*)afh_map, piconet_id, afh_map_modem_psd);
    }   
#endif
#endif
	
}

//#################################################################
// function : lmp_generate_afh_map_for_device_RTK
// create :  Neil Chen
// author:  
// purpose : to generate a AFH map
//	this function will be used by RTK AFH Mechanism
// modify : Morgan, YiLin
// history : 20120910
//	1. add re-initial when No connection or connection is now in disconnection state
//	2. add modem psd info
//##################################################################

UCHAR lmp_generate_afh_map_for_device_RTK(void)
{
    UCHAR i;
    UCHAR afh_map[LMP_AFH_MAP_SIZE];    
    UINT8 any_active_link_with_afh = 0;
    UINT8 any_active_link_with_old_afh = 0;
    
#ifdef _ROM_CODE_PATCHED_
    UINT8 result = 0x01;

    if (rcp_lmp_generate_afh_map_for_device_RTK_func != NULL)
    {
        if (rcp_lmp_generate_afh_map_for_device_RTK_func(afh_map))
        {
            return result;
        }
    }   
#endif

#ifdef _YL_TEST_NEW_MODEM_PSD_SCAN_FOR_AFH_LOG_
//    RT_BT_LOG(WHITE, MDM_LOG_047, 0, 0);
#endif


   //Display the generate the AFH map for device RTK
    if (LMP_NO_CONNECTION())
    {
        /* No connection in the system. Not doing local assessment. */
#ifndef _YL_TEST_NEW_MODEM_PSD_SCAN_FOR_AFH_
        return 0x1;
#endif    
    }

    g_last_afh_update = BB_read_native_clock();    // need to modify the afh information by different indicator--Jr. Neil

    //========================================================
    // need to note that this function code should only support one piconet //---add by Jr. Neil Chen 2010-08-13
    //==========================================================

    for (i=0; i< LMP_MAX_PICONETS_SUPPORTED; i++)
    {
        if (lmp_get_any_afh_enabled_connections(i))
        {
            //Display the generate the AFH map for device RTK
            RTK_AFH_LOG(GREEN,GENERATE_AFH_MAP_FOR_DEVICE,0,0);

            //if MAster to generate AFH map by RTK mechanism
            if (!lc_sca_manager.pnet[i].active)
            {
                continue;
            }

            any_active_link_with_afh = 1;
            if (lc_sca_manager.pnet[i].master)
            {
                lmp_generate_afh_map(i, &afh_map[0]);         // is the bug of the afh_map[0]? should be i???
                any_active_link_with_old_afh = 1;
            }
            else
            {
                // if slave , only one piconet to generate AFH map by RTK mechanism to check the performance by 8852B
                // if slave , scatter net ---slave not to generate afh map
                // Only one link and 8852B case
                if ((lc_sca_manager.slave_cnt == 1) && 
                    (lc_sca_manager.master_cnt == 0))
                {                    
                    lmp_generate_afh_map(i, &afh_map[0]);
                    any_active_link_with_old_afh = 1;
                }
                else
                {                     
#ifdef _NEW_MODEM_PSD_SCAN_
                    if (g_efuse_modem_psd_setting_1.lmp_gen_afh_map_at_other_case &&
                        otp_str_data.rtk_afh_bt_psd_enable)
                    {
                        lmp_generate_afh_map(i, &afh_map[0]);
                    }
                    else
#endif
                    {
                        memcpy(afh_map, afh_ahs79_channel_map, 10);   
                    }
                }
            }
#ifdef _DAPE_TEST_SET_AFH_ALL_ON 
            memcpy(afh_map, afh_ahs79_channel_map, 10);   

#endif
            lmp_piconet_start_afh_map_updation(i, &afh_map[0]);


            // To check the AFH execute times , if epxression , then go into the AFH recovery Stage
            // DM check each piconet good channels to decide the AFH state
 
            //Display the generate the AFH map for device RTK
            RTK_AFH_LOG(RED,GENERATE_AFH_MAP_STATE,2,i, afh_state[i]); 
        }    
	    // 20120912 morgan add for set BT psd to initial state
    	// clear BT PSD state 
    	else
    	{
#ifdef _DAPE_GROUP_AFH_RESET_TO_FUNCTION
            lmp_afh_state_reset(i);
            g_afh_times[i] = 0;
#else    	
            afh_state[i]=INITIAL;
            g_afh_times[i] = 0;
            afh_recovery_cnt[i] = 0;
            afh_recovery_cnt1[i] = 0;
            afh_recovery_cnt2[i] = 0;
            afh_recovery_cnt3[i] = 0;
            afh_recovery_cnt4[i] = 0;
            all_on_map_cnt[i] = 0;	
#endif            
    	}
    }
#ifdef _NEW_MODEM_PSD_SCAN_

#ifdef _ROM_CODE_PATCHED_
    if (rcp_lmp_generate_afh_map_for_device_RTK_post_func != NULL)
    {
        if (rcp_lmp_generate_afh_map_for_device_RTK_post_func(&result, any_active_link_with_old_afh, any_active_link_with_afh))
        {
            return result;
        }
    }   
#endif

    if (g_rtk_afh_modem_psd_internal_enable && otp_str_data.rtk_afh_bt_psd_enable)
    {
        if ( ((g_efuse_modem_psd_setting_1.psd_timers_start_opt==1) && any_active_link_with_old_afh) ||
              ((g_efuse_modem_psd_setting_1.psd_timers_start_opt==2) && any_active_link_with_afh) 
#ifdef _YL_TEST_NEW_MODEM_PSD_SCAN_FOR_AFH_    
              || (g_efuse_modem_psd_setting_1.psd_timers_start_opt==3)
#endif
              )
        {

#ifdef _YL_TEST_NEW_MODEM_PSD_SCAN_FOR_AFH_LOG_
//            RT_BT_LOG(WHITE, MDM_LOG_048, 0, 0);
#endif        
            // to issue the timer expiry function "IMMEDIATELY" //
            OS_START_TIMER(scan_period_timer, 10);
            if (g_efuse_modem_psd_setting_1.afh_map_gen_instant_opt)
            {
                OS_START_TIMER(afh_map_gen_timer, afh_map_gen_timerout_ms);
            }
        }
    }
#endif

    return 0x0;
}


//#################################################################
// function : lmp_generate_afh_map_check_and_handle_few_channels
// create :  Neil Chen
// author:  
// purpose : to generate a AFH map when good channel is under 20 
// modify : Morgan, YiLin
// history : 20120910
//	1. add re-initial when No connection or connection is now in disconnection state
//	2. add modem psd info
//##################################################################
UINT8 lmp_generate_afh_map_check_and_handle_few_channels(
                        UINT8 num_good_channel, UINT8 recovery,
                        UINT8 *afh_map, UINT8 piconet_id)
{
//========================================//
// To protect the good channel >= Min AFH channels (20)
//========================================//

    UINT8 rand_num = 0;
    UINT8 ch_index;
    UINT8 bit_index;
    UINT8 byte_index;
    UINT8 case_cnt;
    UINT8 loop;

#ifdef _ROM_CODE_PATCHED_
    if (rcp_lmp_generate_afh_map_check_and_handle_few_channels_func != NULL)
    {
        if (rcp_lmp_generate_afh_map_check_and_handle_few_channels_func((void*)afh_map,
                    &num_good_channel, recovery, piconet_id))
        {
            return num_good_channel;
        }
    }   
#endif

    if (num_good_channel >= MIN_AFH_CHANNELS)
    {
        all_on_map_cnt[piconet_id] = 0x0;
        return num_good_channel;
    }    	

    for (loop = 0; loop < 2; loop++)
    {       
        UCHAR i;
        UINT8 *tmp_unk_channel = NULL;
        UINT8 *tmp_unk_map = NULL;

        for(i = 0; i < MIN_AFH_CHANNELS; i++)
        { 
            if (unk_channel_num > 0)
            {    
                tmp_unk_channel = &unk_channel_num;
                tmp_unk_map = unknow_map;
            }
            else if (unkII_channel_num > 0)
            {
                tmp_unk_channel = &unkII_channel_num;
                tmp_unk_map = unknow_map_II;
            }
            else
            {
                /* we can break soon */
                break;
            }
            
            if (loop == 0)
            {
                rand_num = lc_generate_rand_number(*tmp_unk_channel);                  
            }
            else
            {
                rand_num++;
                if(rand_num > 9)
                {
                    rand_num -= 10;
                }                
            }    
            
            ch_index = tmp_unk_map[rand_num]; 
            
            if (ch_index != 0xFF) 		   
            {
                bit_index = ch_index & 0x07;
                byte_index = ch_index >> 3;                

                /* check even channel */
                if ((afh_map[byte_index] &  (1 << bit_index)) == 0)
                {
                    num_good_channel++;
                    afh_map[byte_index] |= (1 << bit_index);
                }

                /* check odd channel */
                if (ch_index != (LMP_MAX_CHANNELS-1))
                {
                    if ((afh_map[byte_index] &  (1 << (bit_index + 1))) == 0)
                    {
                        num_good_channel++;
                        afh_map[byte_index] |= (1 << (bit_index + 1));
                    }
                }
                (*tmp_unk_channel)--;
            }             

            if (loop == 0)
            {
                break;
            }
        }  

        if (tmp_unk_channel == NULL)
        {
            /* no any unknown group */            
            for (case_cnt = 0; case_cnt < 10; case_cnt++)
            {
                unknow_map[case_cnt] = 0xFF;
                unknow_map_II[case_cnt] = 0xFF;
            }
            break;
        }
        else
        {
            if (num_good_channel >= MIN_AFH_CHANNELS) 
            {
                /* chnanel is enough */
                break;
            }
        }
    }
    
    //20120927 morgan add for fast all-on process
    UINT8 must_recover_soon = FALSE;

#ifdef FULL_RANDOM_RECOVER
    if( bFastFullOn)
    {
        if (num_good_channel <=14)
        {     
            must_recover_soon = TRUE;               
        }  
    } 
#endif
    
    // No any add by above mechanism so using random methodology
    UINT8 recovery_loop;

    /* In loop 0, we search unknwon channel group to fill non-enough channel map,
       if it is unlucky to enter loop 1,
       In loop 1, we search bad channel group to fill non-enough channel map and
       recover some bad channel groip to unknwon channel group - austin */
       
    for (recovery_loop = 0; recovery_loop < 2; recovery_loop++)
    {
        if (num_good_channel < MIN_AFH_CHANNELS) 
        {
            UINT8 i;	
            ch_index = lc_generate_rand_number(39);
            ch_index = (ch_index<<1);   // rand_num =< 78           
            
            for (i = 0; i < 40; i++)
            {                
                ch_index = ch_index+2;
                  // to aovid big than LMP_MAX_CHANNELS
                if (ch_index > LMP_MAX_CHANNELS)
                {
                    ch_index = 0;    // let ch 80 is ch 0  // due to add 2 by every step
                }		

                if (afh_ch_host[ch_index].host_ch_status == AFH_HOST_BAD_CHANNEL)
                {
                    if (recovery_loop == 0)
                    {                        
                        continue;
                    }    

                    /* force bad channel to be unknown channel in loop 1 */
                    afh_ch_host[ch_index].host_ch_status = AFH_HOST_UNKNOWN_CHANNEL;
                    if (ch_index != (LMP_MAX_CHANNELS -1))
                    {
                        afh_ch_host[ch_index + 1].host_ch_status = AFH_HOST_UNKNOWN_CHANNEL;                        
                    }
                }
                else
                {
                    if (recovery_loop == 1)
                    {                        
                        continue;
                    }                        
                }
                
                byte_index = (ch_index >> 3);
                bit_index = ch_index & 0x07;

                if ((afh_map[byte_index] & (1 << bit_index))==0)
                {
                    afh_map[byte_index] |= (1 << bit_index);
                    num_good_channel++;
                }

                if ((afh_map[byte_index] & (1 << (bit_index + 1)))==0)
                {                        
                    if (ch_index != (LMP_MAX_CHANNELS-1))
                    {
                        afh_map[byte_index] |= 1 << (bit_index + 1);
                        num_good_channel++;
                    }
                }
                
                if (num_good_channel >= MIN_AFH_CHANNELS)
                {
                    break;
                }           
            }          
        }
    }
    
    all_on_map_cnt[piconet_id]++;	

    if (recovery)
    {
        if ((all_on_map_cnt[piconet_id] >= MIN_AFH_CHANNELS_TIMES) ||
            must_recover_soon)
        {
#ifdef _DAPE_GROUP_AFH_RESET_TO_FUNCTION
            lmp_afh_state_reset(piconet_id);
            g_afh_times[piconet_id] = 0;
#else        
            afh_state[piconet_id]=INITIAL;
            g_afh_times[piconet_id] = 0;
            afh_recovery_cnt[piconet_id] = 0;
            afh_recovery_cnt1[piconet_id] = 0;
            afh_recovery_cnt2[piconet_id] = 0;
            afh_recovery_cnt3[piconet_id] = 0;
            afh_recovery_cnt4[piconet_id] = 0;
            all_on_map_cnt[piconet_id] = 0;	
#endif            
        }    
    }  

    return num_good_channel;
}

//#################################################################
// function : lmp_generate_afh_map_front_half
// create :  Neil Chen
// author:  
// purpose : to generate AFH map in BT PSD case 
// modify : Morgan, YiLin
// history : 20120910
//	1. add store afh_bt_psd_map for debug mailbox use
//	2. add patch backdoor
//##################################################################
UINT8 lmp_generate_afh_map_front_half(UCHAR* afh_map)
{
    UCHAR ch_index;
    UCHAR byte_index;
    UCHAR bit_index;
    UCHAR num_good_channel = 0;

    memset(afh_map, 0, LMP_AFH_MAP_SIZE);

#ifdef _ROM_CODE_PATCHED_
    if (rcp_lmp_generate_afh_map_front_half_func != NULL)
    {
        if (rcp_lmp_generate_afh_map_front_half_func((void*)afh_map, &num_good_channel))
        {
            return num_good_channel;
        }
    }   
#endif

    for (ch_index = 0; ch_index < LMP_MAX_CHANNELS; ch_index += 2)
    {
        bit_index = ch_index & 0x07;
        byte_index = ch_index >> 3;
    	
	// 20120907 morgan 	
        // handle host last information
        // compare last time and this time  
        // if last time is not good ( classify to "0", this time is good  (AFH_HOST_UNKNOWN_CHANNEL)
        // set the afh_map = "1" good channel
        // increase num_good_channel
        if ((afh_host_classification_last[byte_index] & (1<<bit_index)) == 0)
        {
            if(afh_ch_host[ch_index].host_ch_status == AFH_HOST_UNKNOWN_CHANNEL)
            {
                if (ch_index == (LMP_MAX_CHANNELS - 1))
                {
                    afh_map[9] |= (1 << 6);
                    num_good_channel++;
                }
                else
                {
                    afh_map[byte_index] |= (3 << bit_index);  
                    num_good_channel += 2;	
                }
            }
            continue;
        }
        if ((afh_map_last[byte_index] & (1 << bit_index)) == 0)
        {
            continue;
        }
        if (afh_ch_host[ch_index].host_ch_status == AFH_HOST_BAD_CHANNEL)
        {
            continue;
        }
        
        // Master Channel Testing
        // Only ACL mode ---> Only check ACL SCORE
        // To check ch_index

	// ==== 20120907 morgan ====
	// condition : if last time channel classify to :"1"
	// measure channel quality
	// the quality is calculated by the below function  ( lmp_cal_afh_channel_quality_RTK_generator)

	// afh_ch_quality.score
	// and the pocedure 
	// lmp_handle_channel_classify_timer_expiry
	// lmp_handle_afh_start_timer
       // lmp_cal_afh_channel_quality_RTK
	// lmp_cal_afh_channel_quality_RTK_generator(0, 10, piconet_id); lmp_cal_afh_channel_quality_RTK_II will call
	// lmp_cal_afh_channel_quality_RTK_generator
	

// ====== ACL ======		
        if ((afh_ch_status[ch_index].total_pkts > 0) &&
            (afh_ch_status[ch_index].sco_total_pkts == 0))
        {
            if (afh_ch_quality[ch_index].score >= SCORE_THRESHOLD)
            {
                if (ch_index == (LMP_MAX_CHANNELS-1))
                {
                    afh_map[9] |= (1 << 6);
                    num_good_channel++;
                    break;
                }  
                
                // To check ch_index+1
                if (afh_ch_quality[ch_index+1].score >= SCORE_THRESHOLD)
                {
                    if ((afh_ch_status[ch_index+1].total_pkts > 0) &&
                        (afh_ch_status[ch_index+1].sco_total_pkts == 0))
                    {
                        afh_map[byte_index] |= (3 << bit_index);                       
                        num_good_channel += 2;
                    }
                }
                // To check ch index+1 for Unk Group
                else if(afh_ch_quality[ch_index+1].score >= SCORE_THRESHOLD_I)
                {
                    if(unk_channel_num < 10)
                    {
                        unknow_map[unk_channel_num] = ch_index;
                        unk_channel_num++;	 
                    }			 
                }
                else if(afh_ch_quality[ch_index+1].score >= SCORE_THRESHOLD_II)
                {
                    if(unkII_channel_num < 10)
                    {                         
                        unknow_map_II[unkII_channel_num] = ch_index;
                        unkII_channel_num++;	 
                    }	   
                }		
            }
            else if(afh_ch_quality[ch_index].score >= SCORE_THRESHOLD_I)
            {
                if(afh_ch_quality[ch_index+1].score >= SCORE_THRESHOLD_I)
                {
                    if(unk_channel_num < 10)
                    {
                        unknow_map[unk_channel_num] = ch_index;
                        unk_channel_num++;	   
                    }	   
                }
                else if(afh_ch_quality[ch_index+1].score >= SCORE_THRESHOLD_II)	
                {
                    if(unkII_channel_num < 10)
                    {
                        unknow_map_II[unkII_channel_num] = ch_index;
                        unkII_channel_num++;	   
                    }	   
                }		   
            }
            else if(afh_ch_quality[ch_index].score >= SCORE_THRESHOLD_II)
            {
                if(afh_ch_quality[ch_index+1].score >= SCORE_THRESHOLD_II)
                {
                    if(unkII_channel_num < 10)
                    {
                        unknow_map_II[unkII_channel_num] = ch_index;
                        unkII_channel_num++;		   
                    }	   
                }			
            }	   
        }
#ifndef _AFH_USE_SCO_SIMULATE_ACL_PKT_
// ====== SCO ======			
        else if (afh_ch_status[ch_index].sco_total_pkts > 0) //SCO PART
    	{
            if (afh_ch_quality[ch_index].score >=SCORE_THRESHOLD_SCO)
            {
                if (ch_index == (LMP_MAX_CHANNELS-1))
                {
                    afh_map[9] |= (1 << 6);
                    num_good_channel++;
                    break;
                }
                
                // To check ch_index+1
                if (afh_ch_quality[ch_index+1].score >= SCORE_THRESHOLD_SCO)
                {
                    afh_map[byte_index] |= (3 << bit_index);                       
                    num_good_channel += 2;
                }
                // To check ch index+1 for Unk Group
                else if(afh_ch_quality[ch_index+1].score >= SCORE_THRESHOLD_I)
                {
                    if(unk_channel_num < 10)
                    {
                        unknow_map[unk_channel_num] = ch_index;
                        unk_channel_num++;	 
                    }			 
                }
                else if(afh_ch_quality[ch_index+1].score >= SCORE_THRESHOLD_II)
                {
                    if(unkII_channel_num < 10)
                    {                     
                        unknow_map_II[unkII_channel_num] = ch_index;
                        unkII_channel_num++;	 
                    }	   
                }		
            }
            else    // 
            {
                if(afh_ch_quality[ch_index].sco_score >= SCORE_THRESHOLD_I)
                {
                    if(afh_ch_quality[ch_index+1].sco_score >= SCORE_THRESHOLD_I)
                    {
                        if(unk_channel_num < 10)
                        {
                            unknow_map[unk_channel_num] = ch_index;
                            unk_channel_num++;	   
                        }	   
                    }
                    else if(afh_ch_quality[ch_index+1].sco_score >= SCORE_THRESHOLD_II)	
                    {
                        if(unkII_channel_num < 10)
                        {
                            unknow_map_II[unkII_channel_num] = ch_index;
                            unkII_channel_num++;	   
                        }	   
                    }		   
                }
                else if(afh_ch_quality[ch_index].sco_score >= SCORE_THRESHOLD_II)
                {
                    if(afh_ch_quality[ch_index+1].sco_score >= SCORE_THRESHOLD_II)
                    {
                        if(unkII_channel_num < 10)
                        {
                            unknow_map_II[unkII_channel_num] = ch_index;
                            unkII_channel_num++;		   
                        }	   
                    }			
                }		   
            }		 
        }	
#endif
    }    

	// memcpy(afh_bt_psd_map, afh_map, LMP_AFH_MAP_SIZE);
    
    return num_good_channel;
}

//================Add by Jr. Neil =============
/**
 * Generate afh map from local Packet Good Ratio.
 *
 * \param afh_map The map will be updated into this data-structure.
 * \param afh_threshold The afh threshold value for GOOD channel.
 * \param afh_threshold could be from efuse ....
 *
 * \return None.
 */
// this function for AFH Normal state using

void lmp_generate_afh_map_from_PGR_RTK(UCHAR piconet_id, UCHAR* afh_map)
{
    UCHAR num_good_channel;
    
#ifdef _ROM_CODE_PATCHED_
    if (rcp_lmp_generate_afh_map_from_PGR_RTK_func != NULL)
    {
        if ( rcp_lmp_generate_afh_map_from_PGR_RTK_func((void*)afh_map, piconet_id) )
        {
            return;
        }
    }   
#endif
	
    num_good_channel = lmp_generate_afh_map_front_half(afh_map);

    memcpy(afh_bt_psd_map[piconet_id], afh_map, LMP_AFH_MAP_SIZE);

   // 20120829 morgan modify set AFH manual
#ifdef MANUAL_SET_AFH

    afh_map[0] = 0xFF;
    afh_map[1] = 0xFF;
    afh_map[2] = 0xFF;
    afh_map[3] = 0xFF;
    afh_map[4] = 0xFF;
    afh_map[5] = 0x00;
    afh_map[6] = 0x00;
    afh_map[7] = 0x00;
    afh_map[8] = 0x00;
    afh_map[9] = 0x00;

	
    RT_BT_LOG(RED,GENERATE_AFH_MAP_FROM_PGR_RTK,10,
              afh_map[0],
              afh_map[1],
              afh_map[2],
              afh_map[3],
              afh_map[4],
              afh_map[5],
              afh_map[6],
              afh_map[7],
              afh_map[8],
              afh_map[9]);

    memcpy(afh_map_last, afh_map, 10);
    num_good_channel = 40;
    //memcpy(afh_host_classification_last, afh_host_classification, 10);	
    return;
#endif
	
    RTK_AFH_LOG(YELLOW,NUM_OF_GOOD_CHANNELS,1, num_good_channel);

//------------ Add by Jr. Neil ---2011/01/06------------
// Add the dynmaic try channel mechanism to improve the FER
//-------------------------------------------------
    if (num_good_channel >= bUpper_good_channel)//71
    {
        RTK_AFH_LOG(YELLOW,NORMALCNT,1,g_afh_times[piconet_id]);

        if (g_afh_times[piconet_id] >= MAX_AFH_EXECUTE_TIMES)
        {
            afh_state[piconet_id] = RECOVERY;
            g_afh_times[piconet_id] = 0;
        }
        else
        {
            g_afh_times[piconet_id]++;
        }
        afh_recovery_cnt[piconet_id] = 0;
        afh_recovery_cnt1[piconet_id] = 0;
        afh_recovery_cnt2[piconet_id] = 0;
        afh_recovery_cnt3[piconet_id] = 0;
        afh_recovery_cnt4[piconet_id] = 0;
    }
    else if ((num_good_channel >= bLowUpper_good_channel) && (num_good_channel < bUpper_good_channel))//59 71
    {
        RTK_AFH_LOG(YELLOW,NORMALCNT,1,afh_recovery_cnt[piconet_id]);

        if ((g_afh_times[piconet_id] >= MAX_AFH_EXECUTE_TIMES)||
            (afh_recovery_cnt[piconet_id] >= AFH_EXECUTE_TIMES_I))
        {
            afh_state[piconet_id] = RECOVERY;
            g_afh_times[piconet_id] = 0;
            afh_recovery_cnt[piconet_id] = 0;
        }
        else
        {
            afh_recovery_cnt[piconet_id]++;
            g_afh_times[piconet_id]++;
        }
        afh_recovery_cnt1[piconet_id] = 0;
        afh_recovery_cnt2[piconet_id] = 0;
        afh_recovery_cnt3[piconet_id] = 0;
        afh_recovery_cnt4[piconet_id] = 0;
    }
    else if ((num_good_channel >=bMiddle_good_channel) && (num_good_channel <bLowUpper_good_channel)) //47 59
    {
        RTK_AFH_LOG(YELLOW,NORMALCNT,1,afh_recovery_cnt1[piconet_id]);

        if ((g_afh_times[piconet_id] >= MAX_AFH_EXECUTE_TIMES)||
            (afh_recovery_cnt[piconet_id] >= AFH_EXECUTE_TIMES_I)||
            (afh_recovery_cnt1[piconet_id] >= AFH_EXECUTE_TIMES_II))
        {
            afh_state[piconet_id] = RECOVERY;
            g_afh_times[piconet_id] = 0;
            afh_recovery_cnt[piconet_id] = 0;
            afh_recovery_cnt1[piconet_id] = 0;
        }
        else
        {
            afh_recovery_cnt1[piconet_id]++;
            afh_recovery_cnt[piconet_id]++;
            g_afh_times[piconet_id]++;
        }
        afh_recovery_cnt2[piconet_id] = 0;
        afh_recovery_cnt3[piconet_id] = 0;
        afh_recovery_cnt4[piconet_id] = 0;
    }
    else if ((num_good_channel >=bLowMiddle_good_channel) && (num_good_channel <bMiddle_good_channel))// 37 47
    {
        RTK_AFH_LOG(YELLOW,NORMALCNT,1,afh_recovery_cnt2[piconet_id]);

        if ((g_afh_times[piconet_id] >= MAX_AFH_EXECUTE_TIMES)||
            (afh_recovery_cnt[piconet_id] >= AFH_EXECUTE_TIMES_I)||
            (afh_recovery_cnt1[piconet_id] >= AFH_EXECUTE_TIMES_II)||
            (afh_recovery_cnt2[piconet_id]>= AFH_EXECUTE_TIMES_III))
        {
            afh_state[piconet_id] = RECOVERY;
            g_afh_times[piconet_id] = 0;
            afh_recovery_cnt[piconet_id] = 0;
            afh_recovery_cnt1[piconet_id] = 0;
            afh_recovery_cnt2[piconet_id] = 0;
        }
        else
        {
            afh_recovery_cnt2[piconet_id]++;
            afh_recovery_cnt1[piconet_id]++;
            afh_recovery_cnt[piconet_id]++;
            g_afh_times[piconet_id]++;
        }
        afh_recovery_cnt3[piconet_id] = 0;
        afh_recovery_cnt4[piconet_id] = 0;
    }
    else if ((num_good_channel >=bLow_good_channel) && (num_good_channel <bLowMiddle_good_channel)) //27 37
    {
        RTK_AFH_LOG(YELLOW,NORMALCNT,1,afh_recovery_cnt2[piconet_id]);

        if ((g_afh_times[piconet_id] >= MAX_AFH_EXECUTE_TIMES)||
            (afh_recovery_cnt[piconet_id] >= AFH_EXECUTE_TIMES_I)||
            (afh_recovery_cnt1[piconet_id] >= AFH_EXECUTE_TIMES_II)||
            (afh_recovery_cnt2[piconet_id]>= AFH_EXECUTE_TIMES_III)||
            (afh_recovery_cnt3[piconet_id]>= AFH_EXECUTE_TIMES_IV))
        {
            afh_state[piconet_id] = RECOVERY;
            g_afh_times[piconet_id] = 0;
            afh_recovery_cnt[piconet_id] = 0;
            afh_recovery_cnt1[piconet_id] = 0;
            afh_recovery_cnt2[piconet_id] = 0;
            afh_recovery_cnt3[piconet_id] = 0;
        }
        else
        {
            afh_recovery_cnt3[piconet_id]++;
            afh_recovery_cnt2[piconet_id]++;
            afh_recovery_cnt1[piconet_id]++;
            afh_recovery_cnt[piconet_id]++;
            g_afh_times[piconet_id]++;
        }
        afh_recovery_cnt4[piconet_id] = 0;
    }
    else if ((num_good_channel >=bBelowLow_good_channel) && (num_good_channel <bLow_good_channel))// 20 27
    {
        RTK_AFH_LOG(YELLOW,NORMALCNT,1,afh_recovery_cnt3[piconet_id]);

        if ((afh_recovery_cnt4[piconet_id] >= MIN_AFH_EXECUTE_TIMES)||
            (afh_recovery_cnt3[piconet_id] >= AFH_EXECUTE_TIMES_IV))
        {
            afh_state[piconet_id] = RECOVERY;
            afh_recovery_cnt3[piconet_id] = 0;
            afh_recovery_cnt4[piconet_id] = 0;
        }
        else
        {
            afh_recovery_cnt4[piconet_id]++;
            afh_recovery_cnt3[piconet_id]++;
        }
        g_afh_times[piconet_id] = 0;
        afh_recovery_cnt[piconet_id] = 0;
        afh_recovery_cnt1[piconet_id] = 0;
        afh_recovery_cnt2[piconet_id] = 0;
            
    }
    else     // num good channels < BT Spec requirement
    {
        afh_state[piconet_id] = RECOVERY;
        g_afh_times[piconet_id] = 0;
        afh_recovery_cnt[piconet_id] = 0;
        afh_recovery_cnt1[piconet_id] = 0;
        afh_recovery_cnt2[piconet_id] = 0;
        afh_recovery_cnt3[piconet_id] = 0;
        afh_recovery_cnt4[piconet_id] = 0;
    }
    
    //========================================//
    // To protect the good channel >= Min AFH channels (20)
    //========================================//
    num_good_channel = lmp_generate_afh_map_check_and_handle_few_channels(
                                num_good_channel, FALSE, afh_map, piconet_id);

    RTK_AFH_LOG(YELLOW,NUM_OF_GOOD_CHANNELS,1, num_good_channel);
    
    //lmp handle channel assess timer expiry
    RTK_AFH_LOG(RED,GENERATE_AFH_MAP_FROM_PGR_RTK,10,
              afh_map[0],
              afh_map[1],
              afh_map[2],
              afh_map[3],
              afh_map[4],
              afh_map[5],
              afh_map[6],
              afh_map[7],
              afh_map[8],
              afh_map[9]);

    memcpy(afh_map_last, afh_map, 10);
    memcpy(afh_host_classification_last, afh_host_classification, 10);	

    return;

}


void lmp_generate_afh_recovery_subroutine(UINT8 piconet_id, UINT8 *afh_map, 
                                    UINT8 *mask_min, UINT8 *mask_max, UINT8 i,
                                    UINT8 *num_good_channel)
{
    UINT8 case_cnt;
    UINT8 byte_index;
    UINT8 bit_index;
    UINT8 *mask;    
    
#ifdef _ROM_CODE_PATCHED_
    if (rcp_lmp_generate_afh_recovery_subroutine_func != NULL)
    {
        if ( rcp_lmp_generate_afh_recovery_subroutine_func((void*)afh_map, piconet_id,mask_min,mask_max, i, num_good_channel) )
        {
            return;
        }
    }   
#endif
    
    //=========2 WLAN select 1========//
    if (mon_reg[piconet_id].downtoup==0xFF)
    {
        case_cnt =lc_generate_rand_number(99);
        if (case_cnt >=50)  // downto up
        {
            mask_min[i]+=2;
            mon_reg[piconet_id].downtoup =0x00;
            mask = &mask_min[i];
        }
        else  // up to down
        {
            if (mask_max[i]==(LMP_MAX_CHANNELS-1))
            {
                mask_max[i]-=2;
            }
            else
            {
                mask_max[i]-=3;
            }
            mon_reg[piconet_id].downtoup = 0x01;
            mask = &mask_max[i];
        }
    }
    else if (mon_reg[piconet_id].downtoup==0x00)
    {
        mask_min[i]+=2;
        mask = &mask_min[i];
    }
    else
    {
        if (mask_max[i]==(LMP_MAX_CHANNELS-1))
        {
            mask_max[i]-=2;
        }
        else
        {
            mask_max[i]-=3;
        }
        mask = &mask_max[i];        
    }

    byte_index = *mask >> 3;
    bit_index = *mask & 0x07;

    mon_reg[piconet_id].channel0 = (byte_index<<4) | bit_index;

    /* bit index is even - austin */
    afh_map[byte_index] |= 3 << (bit_index & ~0x1);

    if (afh_map[byte_index] !=afh_map_last[byte_index])
    {
        *num_good_channel+=2;
    }
    mon_reg[piconet_id].sel_bank = 1;
    mon_reg[piconet_id].wlan_nbi=0x01; // to set wlan recovery    
}

void lmp_generate_afh_map_for_AFH_RECOVERY_cnt_0(UCHAR piconet_id, UINT8 num_good_channel)
{
    UCHAR ch_index;
    UCHAR case_cnt;    
    UCHAR bit_index;
    UCHAR byte_index;
    UCHAR wlan_num;    
    UCHAR mask_min[4] = {0xFF, 0xFF, 0xFF, 0xFF};
    UCHAR mask_max[4] = {0xFF, 0xFF, 0xFF, 0xFF};

    //------ WLAN estimator -------//
    // it will estimate how much wlan interferense in the BT channel
    // Only est at the step 0    

    case_cnt = 0;
    wlan_num = 0;

// 20120912 morgan modify :
// recover modify to 2 recover state :
// 1. wifi state
// 2. random state
    if (num_good_channel < 70)
    {
        // detect how much wlan interference in BT Channels
        for (ch_index = 0; ch_index < LMP_MAX_CHANNELS; ch_index++)
        {
            if (afh_ch_host[ch_index].host_ch_status == AFH_HOST_BAD_CHANNEL)
            {
                if (mask_min[case_cnt >> 1]!=0xFF)  // it means that the wlan contain host mask
                {
                    if((ch_index-mask_min[case_cnt >> 1]-1) >=WLAN_GAP_TH)
                    {
                        mask_max[case_cnt>>1] = ch_index -1;
                        wlan_num++;
                        case_cnt++;
                    }
                    else
                    {
                        mask_max[case_cnt>>1] = 0xFF;
                        mask_min[case_cnt>>1] = 0xFF;         
                    }
                }
                continue;
            }
            if (ch_index == (LMP_MAX_CHANNELS-1))   //To check Channel #78
            {                    
                if ((afh_map_last[9] & (1<<6))==0)
                {
                    if (case_cnt & 0x01)
                    {
                        UINT8 index = case_cnt >> 1;
                                                    
                        if (mask_max[index]==0xFF)
                        {
                            mask_max[index]=78;
                            if ((mask_max[index]-mask_min[index]) >= WLAN_GAP_TH)
                            {
                                wlan_num++;
                            }
                            else
                            {
                                mask_max[index] = 0xFF;
                                mask_min[index] = 0xFF;
                            }
                        }                           
                    }                        
                }
                break;
            }

            bit_index = ch_index & 0x07;
            byte_index = ch_index >> 3;

            if ((afh_map_last[byte_index] & (1<<bit_index))==0)
            {
                if (!(case_cnt & 0x01))
                {
                    if (mask_min[case_cnt >> 1]==0xFF)
                    {
                        mask_min[case_cnt >> 1]= ch_index;
                        case_cnt++;
                    }                        
                }    
                continue;
            }
            else
            {
                if (case_cnt & 0x01)
                {
                    UINT8 index = case_cnt >> 1;
                                                
                    if (mask_max[index]==0xFF)
                    {
                        mask_max[index]=ch_index-1;
                        if ((mask_max[index]-mask_min[index]) >=WLAN_GAP_TH)
                        {
                            case_cnt++;
                            wlan_num++;
                        }
                        else
                        {
                            mask_max[index] = 0xFF;
                            mask_min[index] = 0xFF;
                            case_cnt--;
                        }

                        if (index == 3)
                        {
                            /* original case_cnt is 7 */
                            case_cnt = 8;
                            // Stop the for loop
                            break;
                        }
                    }                        
                }           
                continue;
            }
        }

        g_wifi_cnt = wlan_num;

        if ( wlan_num >= 1 )
        {
            afh_sub_state[piconet_id] = WIFI_RECOVER;
        }
        else
        {
            afh_sub_state[piconet_id] = RANDOM_RECOVER;
        }
        afh_recovery_cnt[piconet_id] ++;
    }

    // end the detect step
    RTK_AFH_LOG(YELLOW,WLAN_EST_MIN,4, mask_min[0],mask_min[1],mask_min[2],mask_min[3]);
    RTK_AFH_LOG(YELLOW,WLAN_EST_MAX,4, mask_max[0],mask_max[1],mask_max[2],mask_max[3]);
}

void lmp_generate_afh_map_for_AFH_RECOVERY_cnt_1(UCHAR piconet_id, UINT8 *pnum_good_channel, UCHAR* afh_map)
{
    UCHAR ch_index;  
    UCHAR bit_index;
    UCHAR byte_index;
    UCHAR wlan_num;    
    UCHAR i;
    UCHAR rand_num;    
    UCHAR mask_min[4] = {0xFF, 0xFF, 0xFF, 0xFF};
    UCHAR mask_max[4] = {0xFF, 0xFF, 0xFF, 0xFF};
    UINT8 num_good_channel = *pnum_good_channel;

    wlan_num = g_is_use_wifi_cnt ? g_wifi_cnt : 0;

    // to decide the recovery pah by wlan in BT channels or not
    // ====== WLAN interference channel recover =========
    //if (wlan_num >= 1)
	switch( afh_sub_state[piconet_id])
	{
	case WIFI_RECOVER :
    {
        UINT8 recovery = TRUE;
        i = lc_generate_rand_number(98);  
        switch(wlan_num)
        {
        case 2:
            i = (i + 1)/50;
            break;

        case 3:
            i = (i + 1)/33;
            break;

        case 4:
            i = (i + 1)/25;            
            break;

        default:              
            if (num_good_channel >61)   // random to try
            {
                if (i < 50)
                {
                    // To Stop
                    afh_recovery_cnt[piconet_id]+=10;
                    mon_reg[piconet_id].sel_bank = 0;
                    recovery = FALSE;
                    break;
                }
            }               
            break;
        }

        if (recovery)
        {
            lmp_generate_afh_recovery_subroutine(piconet_id,
                         afh_map, mask_min, mask_max, 0, &num_good_channel);                 
        }

        RTK_AFH_LOG(YELLOW,AFH_SUB_RECOVER_WIFI,1,0);
        afh_recovery_cnt[piconet_id]++;
    }
    break;
//========= end of wifi interference channel recover =======

        //============ if no WLAN interference , also need to decide the trial channels=======
    //else  // random to try
    case RANDOM_RECOVER:
    {
        for(i = 0; i < 10; i++)
        {
            rand_num = lc_generate_rand_number(78);
            byte_index = rand_num >> 3;
            bit_index = rand_num & 0x07;

            ch_index = (UCHAR)(afh_map[byte_index] | byte_mask2[bit_index]);

            if (ch_index !=0xFF)
            {
                mon_reg[piconet_id].channel0 = (byte_index<<4) | bit_index;
                if (rand_num == 78)
                {
                    afh_map[9] |= 1 << 6;
                    mon_reg[piconet_id].wlan_nbi= 0xFF;
                    if (afh_map[9] != afh_map_last[9])
                    {
                        num_good_channel++;
                    }
                    break;
                }
                else
                {
                    /* bit index is even - austin */
                    afh_map[byte_index] |= 3 << (bit_index & ~0x1);

                    if (afh_map[byte_index] != afh_map_last[byte_index])
                    {
                        num_good_channel += 2;
                    }
                    mon_reg[piconet_id].wlan_nbi= 0xFF;
                    break;
                }
            }
        }

        //RTK_AFH_LOG(RED,WLAN_NBI_SEL,1,mon_reg[piconet_id].wlan_nbi);					
        RTK_AFH_LOG(YELLOW,NUM_OF_TEST,2,byte_index,bit_index);
        RTK_AFH_LOG(YELLOW,AFH_SUB_RECOVER_RANDOM,0,0);
        afh_recovery_cnt[piconet_id]++;
		
    }
	break;
	//========= end of random recover =======

	default:
		break;
    }

    *pnum_good_channel = num_good_channel;
}

void lmp_generate_afh_map_for_AFH_RECOVERY_cnt_even(UCHAR piconet_id, UINT8 *pnum_good_channel, UCHAR* afh_map)
{
    UCHAR ch_index;  
    UCHAR bit_index = 0;
    UCHAR byte_index;   
    UCHAR i;
    UCHAR rand_num;    
    
    UINT8 num_good_channel = *pnum_good_channel;

    if (mon_reg[piconet_id].wlan_nbi == 0x01) // wifi recover
    {
        if (mon_reg[piconet_id].sel_bank ==1)
        {
            // first to check the try channels
            byte_index = mon_reg[piconet_id].channel0 >> 4;
            bit_index = mon_reg[piconet_id].channel0 & 0x0f;
            ch_index = (UCHAR)(afh_map[byte_index] | byte_mask2[bit_index]);

            if (ch_index==0xFF)   // try chanenl sucessful
            {
                // Do the next step of the recovery state
                if (mon_reg[piconet_id].downtoup==0x00)   // down to up path
                {
                    if (bit_index==0)  // to take the byte index issue
                    {
                        if (byte_index >=1)
                        {
                            byte_index -=1;
                            afh_map[byte_index] |= 3 << 6;
                            if (afh_map[byte_index] != afh_map_last[byte_index])
                            {
                                num_good_channel+=2;
                            }
                            byte_index +=1;
                        }
                    }
                    else
                    {
                        afh_map[byte_index] |= 3 << (bit_index-2);
                        if (afh_map[byte_index] != afh_map_last[byte_index])
                        {
                            num_good_channel+=2;
                        }
                    }

                    if (afh_recovery_cnt[piconet_id] != 8)
                    {
                        if (afh_recovery_cnt[piconet_id] ==4)
                        {
                            if (bit_index == 6)
                            {
                                if (byte_index < 9)
                                {
                                    byte_index +=1;
                                    afh_map[byte_index] |= 3 << 0;
                                    if (afh_map[byte_index] !=afh_map_last[byte_index])
                                    {
                                        num_good_channel+=2;
                                    }
                                    byte_index -=1;
                                }
                            }
                            else
                            {
                                afh_map[byte_index] |= 3 << (bit_index+2);
                                if (afh_map[byte_index] !=afh_map_last[byte_index])
                                {
                                    num_good_channel+=2;
                                }
                            }
                            bit_index +=6;
                        }
                        else
                        {
                            bit_index +=4;
                        }

                        if (bit_index >7)
                        {
                            byte_index +=1;
                            bit_index -=8;
                        }
                        if (byte_index <=9)
                        {
                            afh_map[byte_index] |= 3 << bit_index;
                            if (afh_map[byte_index] != afh_map_last[byte_index])
                            {
                                num_good_channel+=2;
                            }
                            mon_reg[piconet_id].channel0 = (byte_index<<4) | bit_index;
                        }
                    }
                    
                    RTK_AFH_LOG(YELLOW,NUM_OF_TEST,2,byte_index,bit_index);

                    afh_recovery_cnt[piconet_id]++;
                }
                //=====================================
                else   // up to down path
                {
                    if (bit_index==6)  // to take the byte index issue
                    {
                        if (byte_index <9)
                        {
                            byte_index +=1;
                            afh_map[byte_index] |= 3 << 0;
                            if (afh_map[byte_index] != afh_map_last[byte_index])
                            {
                                num_good_channel+=2;
                            }
                            byte_index -=1;

                        }
                    }
                    else
                    {
                        if ((byte_index==9) && (bit_index==4))//Channel #78 Case need to take care
                        {
                            afh_map[byte_index] |= 1 << (bit_index+2);
                            if (afh_map[byte_index] !=afh_map_last[byte_index])
                            {
                                num_good_channel+=1;
                            }
                        }
                        else
                        {
                            afh_map[byte_index] |= 3 << (bit_index+2);
                            if (afh_map[byte_index] !=afh_map_last[byte_index])
                            {
                                num_good_channel+=2;
                            }
                        }
                    }
                    //mon_reg[piconet_id].channel0 +=4;
                    if (afh_recovery_cnt[piconet_id] !=8)
                    {
                        if (afh_recovery_cnt[piconet_id] ==4)
                        {
                            if (bit_index==0)  // to take the byte index issue
                            {
                                if (byte_index >=1)
                                {
                                    byte_index -=1;
                                    afh_map[byte_index] |= 3 << 6;
                                    if (afh_map[byte_index] != afh_map_last[byte_index])
                                    {
                                        num_good_channel+=2;
                                    }
                                    byte_index +=1;
                                }
                            }
                            else
                            {
                                afh_map[byte_index] |= 3 << (bit_index-2);
                                if (afh_map[byte_index] != afh_map_last[byte_index])
                                {
                                    num_good_channel+=2;
                                }
                            }
                            
                            if (bit_index <6)
                            {
                                if (byte_index >= 1)
                                {
                                    bit_index=bit_index+8-6;
                                    byte_index -=1;
                                }
                            }
                            else
                            {
                                bit_index-=6;
                            }
                        }
                        else // 2 or 6
                        {
                            if (bit_index>=4)
                            {
                                bit_index -=4;
                            }
                            else
                            {
                                if (byte_index >=1)
                                {
                                    bit_index+=8;
                                    byte_index -=1;
                                    bit_index -=4;
                                }
                            }
                        }
                        afh_map[byte_index] |= 3 << bit_index;
                        if (afh_map[byte_index] !=afh_map_last[byte_index])
                        {
                            num_good_channel+=2;
                        }
                        mon_reg[piconet_id].channel0 = (byte_index<<4) | bit_index;
                    }

                    RTK_AFH_LOG(YELLOW,NUM_OF_TEST,2,byte_index,bit_index);

                    afh_recovery_cnt[piconet_id]++;
                }
            }
            else
            {
                // to change the try path , down to up or up to down
                if (afh_recovery_cnt[piconet_id] ==2)
                {
                    if (mon_reg[piconet_id].downtoup==0x00)
                    {
                        mon_reg[piconet_id].downtoup=0x01;
                    }
                    else
                    {
                        mon_reg[piconet_id].downtoup=0x00;
                    }
                }
                // stop the recovery state
                // if num_good < 25 , not stop the recovery state
                if (num_good_channel < 25)
                {
                    afh_recovery_cnt[piconet_id] =0;
                }
                else
                {
                    afh_recovery_cnt[piconet_id] +=10;
                }

                RTK_AFH_LOG(YELLOW,CHK_TO_VAR,1,mon_reg[piconet_id].downtoup);
            }
        }
        else if (mon_reg[piconet_id].sel_bank ==0)
        {
            // STOP the RECOVERY State
            afh_recovery_cnt[piconet_id] +=10;  // STOP
        }
    }
    else if (mon_reg[piconet_id].wlan_nbi ==0xFF) // NBI // random recover
    {
        afh_recovery_cnt[piconet_id]++;
        for( i=0; i<10; i++)
        {
            rand_num = lc_generate_rand_number(78);

            if(afh_ch_host[rand_num].host_ch_status==AFH_HOST_BAD_CHANNEL)
            {
                continue;
            }
            byte_index = (rand_num>>3);
            bit_index = bit_index & 0x07;

            ch_index = (UCHAR)(afh_map[byte_index]|byte_mask2[bit_index]);
            
            if (ch_index !=0xFF)
            {
                mon_reg[piconet_id].channel0 = (byte_index<<4) | bit_index;
                if (rand_num==78)
                {
                    afh_map[9] |= 1 << 6;
                    mon_reg[piconet_id].wlan_nbi= 0xFF;
                    if (afh_map[9] != afh_map_last[9])
                    {
                        num_good_channel+=1;
                    }
                    break;
                }
                else
                {
                    /* bit index is even - austin */
                    afh_map[byte_index] |= 3 << (bit_index & ~0x1);

                    if (afh_map[byte_index] !=afh_map_last[byte_index])
                    {
                        num_good_channel+=2;
                    }
                    mon_reg[piconet_id].wlan_nbi= 0xFF;
                    break;
                }
            }
        }
        
        RTK_AFH_LOG(YELLOW,NUM_OF_TEST,2,byte_index,bit_index);
    }
    else // error break mechanism
    {
        afh_recovery_cnt[piconet_id]+=10;
    }

    RTK_AFH_LOG(YELLOW,AFH_SUB_RECOVER_STATE,1,afh_sub_state[piconet_id]);

    *pnum_good_channel = num_good_channel;
}


// this function for AFH recovery state using
void lmp_generate_afh_map_for_AFH_RECOVERY(UCHAR piconet_id, UCHAR* afh_map)
{
    UCHAR bit_index;
    UCHAR num_good_channel;
    UCHAR i;
    
//---------Normal detect function as the AFH normal state ------------//

    num_good_channel = lmp_generate_afh_map_front_half(afh_map);
    memcpy(&afh_bt_psd_map[piconet_id][0], afh_map, LMP_AFH_MAP_SIZE);

//---------------------------------------------------end the normal measurement
//===============================================
    memcpy(afh_map_last, afh_map, 10);   
//============================================

    RTK_AFH_LOG(YELLOW,RECOVERYCNT,1,afh_recovery_cnt[piconet_id]);

    do
    {
        // ===== 20120827 morgan add patch ===================================
#ifdef _ROM_CODE_PATCHED_
        if (rcp_lmp_generate_afh_map_for_AFH_RECOVERY_part1_func != NULL)
        {
            if ( rcp_lmp_generate_afh_map_for_AFH_RECOVERY_part1_func((void*)afh_map, 
                                                &num_good_channel, piconet_id) )
            {           
                break; // break of switch case 0xff
            }
        } 
#endif	

        if (afh_recovery_cnt[piconet_id] == 0)
        {
            lmp_generate_afh_map_for_AFH_RECOVERY_cnt_0(piconet_id, num_good_channel);
        }
        else if ( afh_recovery_cnt[piconet_id] == 1 )
        {
            // to decide the recovery pah by wlan in BT channels or not
            lmp_generate_afh_map_for_AFH_RECOVERY_cnt_1(piconet_id, &num_good_channel, afh_map);
        }
        else if ((afh_recovery_cnt[piconet_id] & 0x01) == 0)
        {
            // === 20140828 ===
            // the case is 2 4 6 8
            // === austin ===        
            lmp_generate_afh_map_for_AFH_RECOVERY_cnt_even(piconet_id, &num_good_channel, afh_map);
        }	
        else
        {
            // === 20120907 ===
            // the case is 1 3 5 7 9
            // === morgan ===
        	//20120912 morgan add for recover mode	
            afh_recovery_cnt[piconet_id]++;
        }
    }
    while (0);

    RTK_AFH_LOG(YELLOW,NUM_OF_GOOD_CHANNELS,1, num_good_channel);   

    num_good_channel = 0;

    // Add by Neil Chen---20110530-----
    for (i = 0; i < LMP_AFH_MAP_SIZE; i++)
    {        
        afh_map[i] = (afh_map[i] & afh_host_classification[i]);

        for (bit_index = 0; bit_index < 8; bit_index++)
        {
            if (afh_map[i] & (0x01 << bit_index))
            {               
                num_good_channel++;              
            }
        }
    }

    // RECOVERY FSM
    if (afh_recovery_cnt[piconet_id] >= MAX_AFH_EXECUTE_TIMES_RECOVERY)
    {
        if (num_good_channel < 23) // original is 20
        {
            afh_recovery_cnt[piconet_id] = 0;
            afh_state[piconet_id] = RECOVERY;
        }
        afh_recovery_cnt[piconet_id] = 0;
        afh_state[piconet_id] = NORMAL;
        g_afh_times[piconet_id] = 0;
        afh_recovery_cnt1[piconet_id] = 0;
        afh_recovery_cnt2[piconet_id] = 0;
        afh_recovery_cnt3[piconet_id] = 0;
        afh_recovery_cnt4[piconet_id] = 0;
        all_on_map_cnt[piconet_id] = 0;
    }
    
    //========================================//
    // To protect the good channel >= Min AFH channels (20)
    //========================================//
    num_good_channel = lmp_generate_afh_map_check_and_handle_few_channels(
                                num_good_channel, TRUE, afh_map, piconet_id);

    memcpy(afh_map_last, afh_map, 10);
    memcpy(afh_host_classification_last, afh_host_classification, 10);	
    RTK_AFH_LOG(RED,NUM_OF_GOOD_CHANNELS,1, num_good_channel);

    //lmp handle channel assess timer expiry
    RTK_AFH_LOG(GREEN,GENERATE_AFH_MAP_FOR_RECOVERY,10,
              afh_map[0],
              afh_map[1],
              afh_map[2],
              afh_map[3],
              afh_map[4],
              afh_map[5],
              afh_map[6],
              afh_map[7],
              afh_map[8],
              afh_map[9]);

    return;

}
//==== full random recover ====
// ==== 20120919 morgan ====
#ifdef FULL_RANDOM_RECOVER
// this function for AFH recovery state using
void lmp_generate_afh_map_for_AFH_FULL_RANDOM_RECOVERY(UCHAR piconet_id, UCHAR* afh_map)
{

    UCHAR bit_index = 0;
    UCHAR num_good_channel;
    CHAR byte_index;
    UCHAR rand_num;
    UINT8 AFH_find_flag =0;
    UINT8 i;
    rand_num = 0x0;


#ifdef _ROM_CODE_PATCHED_
    if (rcp_lmp_generate_afh_map_for_AFH_RECOVERY_func != NULL)
    {
        if ( rcp_lmp_generate_afh_map_for_AFH_RECOVERY_func((void*)afh_map, piconet_id) )
        {
            return;
        }
    }   
#endif
//---------Normal detect function as the AFH normal state ------------//
    num_good_channel = lmp_generate_afh_map_front_half(afh_map);
  memcpy(afh_bt_psd_map[piconet_id], afh_map, LMP_AFH_MAP_SIZE);
//---------------------------end the normal measurement
//====================================
    memcpy(afh_map_last, afh_map, 10);   
//====================================
	RTK_AFH_LOG(GREEN,NUM_OF_GOOD_CHANNELS,1, num_good_channel);   
    RTK_AFH_LOG(YELLOW,RECOVERYCNT,1,afh_recovery_cnt[piconet_id]);

 // Add by Neil Chen---20110530-----
        num_good_channel=0;
    for (i = 0; i < LMP_AFH_MAP_SIZE; i++)
    {        
        afh_map[i] = (afh_map[i] & afh_host_classification[i]);

        for (bit_index = 0; bit_index < 8; bit_index++)
        {
            if (afh_map[i] & (0x01 << bit_index))
            {               
                num_good_channel++;              
            }
        }
    }
    RTK_AFH_LOG(YELLOW,NUM_OF_GOOD_CHANNELS,1, num_good_channel);   

    if ( afh_recovery_cnt[piconet_id] <= 9)             
    {
        RTK_AFH_LOG(YELLOW,AFH_SUB_RECOVER_STATE,1,afh_sub_state[piconet_id]);
        rand_num = lc_generate_rand_number(76);
        RTK_AFH_LOG(BLUE,AFH_GEN_RANDOM_NUMBER,1,rand_num);
        // find up or find down						 					
        byte_index = rand_num >> 3;
        bit_index = rand_num & 0x07;
        if( bit_index >= 6 )
        {
            bit_index = 0;
            byte_index++;
        }			
        if( byte_index <10 )
        {				  
            while(1)
            {
                if (   ( (afh_host_classification[byte_index] >> bit_index)&0x01) != 0 ) // =1 ( not wifi channel )
                {				  		
                    if (   ( (afh_map[byte_index] >> bit_index)&0x01) != 1 ) // =0 ( bad channel )
                    {
                        afh_map[byte_index] |= (1 << bit_index);
                        afh_map[byte_index] |= (1 << (bit_index+1));
                        mon_reg[piconet_id].wlan_nbi= 0xFF;	                       
                        num_good_channel+=2;	
                        AFH_find_flag = 1;
                        // show log
                        RTK_AFH_LOG(YELLOW,AFH_TRY_RECOVER_CH,3,0,((byte_index*8)+bit_index),((byte_index*8)+bit_index+1));
                        break;
                    }
                }				
                //===============
                if( bit_index >= 6 )
                {
                    bit_index = 0;
                    byte_index++;
                    if( byte_index >=10 )
                    {
                        break;
                    }
                }
                else if (( bit_index & 0x01) == 0x01)
                {
                    bit_index++;
                }										
                else
                {
                    bit_index+=2;
                }															
            }
        }

        // ==== up to down =====
        if ( AFH_find_flag == 0 )
        {
            byte_index = rand_num >> 3;
            bit_index = rand_num & 0x07;			
            if( bit_index <= 1 )
            {
                bit_index = 7;
                byte_index--;
            }			
            if( byte_index <10 )
            {				  
                while(1)
                {
                    if (   ( (afh_host_classification[byte_index] >> bit_index)&0x01) != 0 ) // =1 ( not wifi channel )
                    {			  						  
                        if (   ( (afh_map[byte_index] << (7-bit_index))&0x80) != 0x80 ) // =0 ( bad channel )
                        {
                            afh_map[byte_index] |= (1 << (7-bit_index));
                            afh_map[byte_index] |= (1 << (7-bit_index-1));
                            mon_reg[piconet_id].wlan_nbi= 0xFF;	                       
                            num_good_channel+=2;	
                            // show log
                            RTK_AFH_LOG(YELLOW,AFH_TRY_RECOVER_CH,3,1,((byte_index*8)+bit_index),((byte_index*8)+bit_index-1));
                            break;
                        }
                    }

                    if( bit_index <= 1 )
                    {
                        bit_index = 7;
                        byte_index--;
                        if( byte_index <0 )
                        {
                            // no update
                            RTK_AFH_LOG(YELLOW,AFH_TRY_RECOVER_CH,3,2,0,0);
                            break;
                        }
                    }
                    else if (( bit_index & 0x01) == 0x00)
                    {
                        bit_index--;
                    }										
                    else
                    {
                        bit_index-=2;
                    }
                }
            }
        }
        if (num_good_channel > 40)
        {
            afh_recovery_cnt[piconet_id] += 2;
        }
        else
        {
            afh_recovery_cnt[piconet_id]++;
        }
        // ==== up to down ending ====						
    }

    RTK_AFH_LOG(BLUE,NUM_OF_GOOD_CHANNELS,1, num_good_channel);   

    // RECOVERY FSM
    if (afh_recovery_cnt[piconet_id] >= MAX_AFH_EXECUTE_TIMES_RECOVERY)
    {
        if (num_good_channel < 22) // original is 20
        {
            afh_recovery_cnt[piconet_id] = 0;
            afh_state[piconet_id] = RECOVERY;
        }
        afh_recovery_cnt[piconet_id] = 0;
        afh_state[piconet_id] = NORMAL;
        g_afh_times[piconet_id] = 0;
        afh_recovery_cnt1[piconet_id] = 0;
        afh_recovery_cnt2[piconet_id] = 0;
        afh_recovery_cnt3[piconet_id] = 0;
        afh_recovery_cnt4[piconet_id] = 0;
        all_on_map_cnt[piconet_id] = 0;
    }    
    //========================================//
    // To protect the good channel >= Min AFH channels (20)
    //========================================//
    num_good_channel = lmp_generate_afh_map_check_and_handle_few_channels(
                                num_good_channel, TRUE, afh_map, piconet_id);

    memcpy(afh_map_last, afh_map, 10);
    memcpy(afh_host_classification_last, afh_host_classification, 10);	
    RTK_AFH_LOG(RED,NUM_OF_GOOD_CHANNELS,1, num_good_channel);

    //lmp handle channel assess timer expiry
    RTK_AFH_LOG(GREEN,GENERATE_AFH_MAP_FOR_RECOVERY,10,
              afh_map[0],
              afh_map[1],
              afh_map[2],
              afh_map[3],
              afh_map[4],
              afh_map[5],
              afh_map[6],
              afh_map[7],
              afh_map[8],
              afh_map[9]);
    return;
}
#endif
// update channel map from slave ch classification map--Jr. Neil

void lmp_update_ch_as_from_slave_ch_cl_map_RTK(UCHAR* classified_map, UCHAR* slave_ch_assessment)
{
    UCHAR byte_index;
    UCHAR bit_index;
    UCHAR ch_index;
    UCHAR ch_pair_status;
    UCHAR ch_status[4] = {UNKNOWN_CHANNEL_BY_SLAVE, /* Unknown channel */
                          GOOD_CHANNEL_BY_SLAVE,    /* Good channel */
                          BAD_CHANNEL_BY_SLAVE,     /* Reserved Value */
                          BAD_CHANNEL_BY_SLAVE
                         };    /* Bad channel. */

    /* channel 0th ~ 77th channels */
    for (ch_index = 0; ch_index < (LMP_MAX_CHANNELS -2); ch_index += 2)
    {
        byte_index = ch_index >> 3;
        bit_index = ch_index & 0x07;
        ch_pair_status = (classified_map[byte_index] >> (bit_index)) & 0x03;

        slave_ch_assessment[ch_index] = ch_status[ch_pair_status];
        slave_ch_assessment[ch_index + 1] = ch_status[ch_pair_status];
    }

    /* 78th channel */
    if (((classified_map[9] >> 6) & 0x01) == 0x1)
    {
        /* Good Channel */
        slave_ch_assessment[ch_index] = GOOD_CHANNEL_BY_SLAVE;
    }
    else
    {
        /* Bad channel */
        slave_ch_assessment[ch_index] = BAD_CHANNEL_BY_SLAVE;
    }
// need to do ---
}


#if 0 //unused logic
/**
 * Updates the Channel assessment algorithm with last received
 * channel-classification/afh map.
 *
 * \param piconet_id piconet id.
 * \param role piconet id role (MASTER/SLAVE)
 * \Add the function refer to slave's channel result --Jr. Neil--2010--08--17--
 * \ need to do more option in the future--How to use the slave's channel result--Jr. Neil--2010-08-17
 * \return None.
 */
void  lmp_ch_la_update_la_with_ch_cl_or_afh_map_RTK(UCHAR* slave_ch_assessment, UCHAR piconet_id, UCHAR role)
{
    UINT32 ce_index;
    LMP_CONNECTION_ENTITY *ce_ptr;

    /* Check the following:
       1. Connection exists.
       2. AFH is enabled. */
    for(ce_index = 0; ce_index < LMP_MAX_CE_DATABASE_ENTRIES; ce_index++)
    {
        ce_ptr = &lmp_connection_entity[ce_index];

        if ( (ce_ptr->ce_status != LMP_STANDBY) &&
                (ce_ptr->phy_piconet_id == piconet_id) &&
                (ce_ptr->afh_mode == AFH_ENABLE) )
        {
#ifdef COMPILE_HOLD_MODE
            if (ce_ptr->ce_status == LMP_HOLD_MODE)
            {
                /*
                 * implicitly disabled in hold-mode.
                 * No further processing will be done.
                 */
                continue;
            }
#endif
#ifdef COMPILE_PARK_MODE
            if (ce_ptr->ce_status == LMP_PARK_MODE)
            {
                /*
                 * implicitly disabled in park-mode.
                 * No further processing will be done.
                 */
                continue;
            }
#endif

            /* Call update the map function with ce_ptr map. */
            if (role == MASTER)
            {
                if ((ce_ptr->remote_dev_role == SLAVE ) &&
                        (ce_ptr->new_ch_cl_map == 0x1))
                {
                    lmp_update_ch_as_from_slave_ch_cl_map_RTK(
                        ce_ptr->last_recd_ch_cl_map, slave_ch_assessment);         // to request slave report the ch classification result---Jr. Neil

                }
            }
        }
    }
    return;
}
#endif

void lmp_cal_afh_channel_quality_RTK_generator(UINT8 ch_min, 
                                                            UINT8 ch_max,
                                                            UINT8 piconet_id)
{
    UINT16   total_pkts;            // to calculate the total packets of ACL type
    UINT16   sco_total_pkts;     // to calculate the total packets of SCO type
    UINT16   good_pkts;
    UINT16   unknownI_pkts;
    UINT16   sco_unknownI_pkts;   //need to know that in SCO only could check HEC
    UINT16   unknownII_pkts;
    UINT16   sco_unknownII_pkts;
    UINT16   bad_pkts;
    UINT16   sco_bad_pkts;
    UCHAR   byte_index,bit_index, ch_index;
    UCHAR   temp;
    UCHAR   temp_sco;
    UCHAR    avg_total_pkts;
    UCHAR    avg_sco_total_pkts;
    UCHAR    avg_sco_bad_pkts;
    UCHAR  channel_pkts_threshold0;
    UCHAR  channel_pkts_threshold1;
    UCHAR  channel_pkts_threshold2;
    UCHAR  acl_th0_slave;
    UCHAR  acl_th1_slave;
    UCHAR  acl_th2_slave;
    UCHAR    avg_bad_pkts;

#ifdef _ROM_CODE_PATCHED_
    if (rcp_lmp_cal_afh_channel_quality_RTK_generator_func != NULL)
    {
        if (rcp_lmp_cal_afh_channel_quality_RTK_generator_func((void*)&piconet_id, ch_min, ch_max))
        {
            return;
        }
    }   
#endif
    acl_th0_slave = channel_cal_parameter.acl_th0_slave;
    acl_th1_slave = channel_cal_parameter.acl_th1_slave;
    acl_th2_slave = channel_cal_parameter.acl_th2_slave;
    channel_pkts_threshold0 = channel_cal_parameter.channel_pkts_threshold0;
    channel_pkts_threshold1 = channel_cal_parameter.channel_pkts_threshold1;
    channel_pkts_threshold2 = channel_cal_parameter.channel_pkts_threshold2;
    avg_bad_pkts = channel_cal_parameter.avg_bad_pkts;
    avg_total_pkts = channel_cal_parameter.avg_total_pkts;
    avg_sco_bad_pkts = channel_cal_parameter.avg_sco_bad_pkts;
    avg_sco_total_pkts = channel_cal_parameter.avg_sco_total_pkts;	
#ifdef _DAPE_AFH_SET_CH_GOOD_AT_INIT
    if (afh_state[piconet_id] == INITIAL)
    {
        memset(afh_ch_status, 0, sizeof(AFH_CH_ASSESSMENT_NODE)*LMP_MAX_CHANNELS);
    }
#endif
    for (ch_index = ch_min; ch_index < ch_max; ch_index++)
    {
        //host to specify the channel status
        if (afh_ch_host[ch_index].host_ch_status == AFH_HOST_BAD_CHANNEL)
        {
            continue;
        }

        bit_index = ch_index & 0x07;
        byte_index = ch_index >> 3;
        
#ifdef _DAPE_AFH_SET_CH_GOOD_AT_INIT
        if (afh_state[piconet_id] == INITIAL)
        {
            afh_ch_quality[ch_index].score = 1000;
            afh_ch_quality[ch_index].header_quality= 1;        	
            continue;
        }
#endif
        //recovery mechanism

        if ((afh_map_last[byte_index] & (1 << bit_index))==0)
        {
            afh_ch_quality[ch_index].score = -1000;
            afh_ch_quality[ch_index].header_quality=200;
            continue;

        }
        total_pkts = afh_ch_status[ch_index].total_pkts;
        good_pkts  = afh_ch_status[ch_index].num_good_pkts;
        bad_pkts    = afh_ch_status[ch_index].num_bad_pkts;
        unknownI_pkts = afh_ch_status[ch_index].num_unknownI_pkts;
        unknownII_pkts = afh_ch_status[ch_index].num_unknownII_pkts;
        sco_total_pkts = afh_ch_status[ch_index].sco_total_pkts;
        sco_bad_pkts    = afh_ch_status[ch_index].sco_bad_pkts;
        sco_unknownI_pkts = afh_ch_status[ch_index].sco_unknownI_pkts;
        sco_unknownII_pkts = afh_ch_status[ch_index].sco_unknownII_pkts;

        temp = total_pkts - bad_pkts;
        temp_sco = sco_total_pkts - sco_bad_pkts;

        if ((total_pkts >=1) && (sco_total_pkts ==0))  // Only ACL type
        {
            if (temp>0)
            {
                /* reference score */
                afh_ch_quality[ch_index].score = 
                        (good_pkts<<4)-(unknownI_pkts<<2)-(unknownII_pkts<<3);

                if ((avg_bad_pkts >=3) && (bad_pkts < avg_bad_pkts))
                {
                    afh_ch_quality[ch_index].score -= (bad_pkts-avg_bad_pkts)<<3;
                }
                else
                {
                    afh_ch_quality[ch_index].score -= (bad_pkts-avg_bad_pkts)<<4;
                }
                afh_ch_quality[ch_index].header_quality = (afh_ch_status[ch_index].header_quality/temp);
            }
            else //  temp = total_pkts - bad_pkts; 
            {
                afh_ch_quality[ch_index].header_quality=200;
                afh_ch_quality[ch_index].score = -1000;
            }
#ifndef _AFH_USE_SCO_SIMULATE_ACL_PKT_
            afh_ch_quality[ch_index].sco_score =1000 ;
            afh_ch_quality[ch_index].sco_header_quality=1;
#endif
        }
#ifndef _AFH_USE_SCO_SIMULATE_ACL_PKT_
        else if ((sco_total_pkts >=1) && (total_pkts ==0))   // Only SCO Type
        {
            if (temp_sco > 0)
            {
                if (sco_bad_pkts <= channel_pkts_threshold0)
                {
                    afh_ch_quality[ch_index].sco_score  = 1000;
                }
                else
                {
                    afh_ch_quality[ch_index].sco_score  = -1000;
                }
                afh_ch_quality[ch_index].sco_header_quality = (afh_ch_status[ch_index].sco_header_quality/temp_sco);
            }
            else
            {
                if (sco_bad_pkts <= channel_pkts_threshold2)
                {
                    afh_ch_quality[ch_index].sco_header_quality=10;
                    afh_ch_quality[ch_index].sco_score = 20;
                }
                else
                {
                    afh_ch_quality[ch_index].sco_header_quality=200;
                    afh_ch_quality[ch_index].sco_score = -1000;
                }
            }
            // ACL Part
            afh_ch_quality[ch_index].score =1000 ;
            afh_ch_quality[ch_index].header_quality=1;
        }
        else if ((sco_total_pkts >=1) && (total_pkts >=1))    // Mixed Mode
        {
            if ((temp>0) && (temp_sco >0))
            {
                //ACL Part
                /* reference score */
                afh_ch_quality[ch_index].score = 
                        (good_pkts<<4)-(unknownI_pkts<<2)-(unknownII_pkts<<3);

                if ((avg_bad_pkts >=3) && (bad_pkts < avg_bad_pkts))
                {
                    afh_ch_quality[ch_index].score -= (bad_pkts-avg_bad_pkts)<<3;
                }
                else
                {
                    afh_ch_quality[ch_index].score -= (bad_pkts-avg_bad_pkts)<<4;
                }
                afh_ch_quality[ch_index].header_quality = (afh_ch_status[ch_index].header_quality/temp);

                // SCO Part
                if (sco_bad_pkts <= channel_pkts_threshold0)
                {
                    afh_ch_quality[ch_index].sco_score  = 1000;
                }
                else
                {
                    afh_ch_quality[ch_index].sco_score  = -1000;
                }
                afh_ch_quality[ch_index].sco_header_quality = (afh_ch_status[ch_index].sco_header_quality/temp_sco);
            }
            else if ((temp > 0) && (temp_sco ==0))
            {
                // ACL part
                /* reference score */
                afh_ch_quality[ch_index].score = 
                        (good_pkts<<4)-(unknownI_pkts<<2)-(unknownII_pkts<<3);

                if ((avg_bad_pkts >=3) && (bad_pkts < avg_bad_pkts))
                {
                    afh_ch_quality[ch_index].score -= (bad_pkts-avg_bad_pkts)<<3;
                }
                else
                {
                    afh_ch_quality[ch_index].score -= (bad_pkts-avg_bad_pkts)<<4;
                }
                afh_ch_quality[ch_index].header_quality = (afh_ch_status[ch_index].header_quality/temp);

                // SCO Part
                if (sco_bad_pkts <= channel_pkts_threshold2)
                {
                    afh_ch_quality[ch_index].sco_header_quality=10;
                    afh_ch_quality[ch_index].sco_score = 20;
                }
                else
                {
                    afh_ch_quality[ch_index].sco_header_quality=200;
                    afh_ch_quality[ch_index].sco_score = -1000;
                }
            }
            else if ((temp_sco > 0) && (temp ==0))
            {
                // SCO Part
                if (sco_bad_pkts <= channel_pkts_threshold0)
                {
                    afh_ch_quality[ch_index].sco_score  = 1000;
                }
                else
                {
                    afh_ch_quality[ch_index].sco_score  = -1000;
                }
                afh_ch_quality[ch_index].sco_header_quality = (afh_ch_status[ch_index].sco_header_quality/temp_sco);
                // ACL Part
                afh_ch_quality[ch_index].header_quality=200;
                afh_ch_quality[ch_index].score = -1000;
            }
            else  // ACL and SCO all zero
            {
                // SCO Part
                if (sco_bad_pkts <= channel_pkts_threshold2)
                {
                    afh_ch_quality[ch_index].sco_header_quality=10;
                    afh_ch_quality[ch_index].sco_score = 20;
                }
                else
                {
                    afh_ch_quality[ch_index].sco_header_quality=200;
                    afh_ch_quality[ch_index].sco_score = -1000;
                }
                // ACL Part
                afh_ch_quality[ch_index].header_quality=200;
                afh_ch_quality[ch_index].score = -1000;
            }
        }
#endif
        else  // ACL and SCO total Packet all zeros
        {
#ifndef _DAPE_AFH_NO_SET_SCORE_LOW_WHEN_NO_RX            
        
            // TODO
            afh_ch_quality[ch_index].score = -1000;
            afh_ch_quality[ch_index].header_quality=200;
#ifndef _AFH_USE_SCO_SIMULATE_ACL_PKT_
            afh_ch_quality[ch_index].sco_score = -1000 ;
            afh_ch_quality[ch_index].sco_header_quality=200;
#endif
#endif
        }
        
        if (lc_sca_manager.pnet[piconet_id].master !=TRUE)
        {
            if (afh_ch_status[ch_index].total_pkts < acl_th0_slave) //small than 1/2
            {
                afh_ch_quality[ch_index].score -=300;
            }
            else if (afh_ch_status[ch_index].total_pkts < acl_th1_slave)  //3/4
            {
                afh_ch_quality[ch_index].score -=160;
            }
            else if (afh_ch_status[ch_index].total_pkts < acl_th2_slave)  //7/8
            {
                afh_ch_quality[ch_index].score -=80;
            }
            if (temp >0)
            {
                if (afh_ch_quality[ch_index].header_quality<=4)
                {
                    afh_ch_quality[ch_index].score +=200;
                }
                else if (afh_ch_quality[ch_index].header_quality<=8)
                {
                    //afh_ch_quality[ch_index].score= afh_ch_quality[ch_index].score;
                }
                else if (afh_ch_quality[ch_index].header_quality<=18)
                {
                    afh_ch_quality[ch_index].score -=500;
                }
                else if (afh_ch_quality[ch_index].header_quality<=50)
                {
                    afh_ch_quality[ch_index].score -=800;
                }
                else
                {
                    afh_ch_quality[ch_index].score -=1000;
                }

            }
        }
        if (lc_sca_manager.pnet[piconet_id].master ==TRUE)
        {
            if (temp >0)
            {
                if (afh_ch_quality[ch_index].header_quality<=4)
                {
                    afh_ch_quality[ch_index].score +=200;
                }
                else if (afh_ch_quality[ch_index].header_quality<=10)
                {
                    //afh_ch_quality[ch_index].score= afh_ch_quality[ch_index].score;
                }
                else if (afh_ch_quality[ch_index].header_quality<=50)
                {
                    afh_ch_quality[ch_index].score -=100;
                }
                else if (afh_ch_quality[ch_index].header_quality<=100)
                {
                    afh_ch_quality[ch_index].score -=200;
                }
                else
                {
                    afh_ch_quality[ch_index].score -=500;
                }
            }

#ifndef _AFH_USE_SCO_SIMULATE_ACL_PKT_
            if (temp_sco >0)
            {
                if (afh_ch_quality[ch_index].sco_header_quality<=4)
                {
                    afh_ch_quality[ch_index].sco_score +=200;
                }
                else if (afh_ch_quality[ch_index].sco_header_quality<=10)
                {
                    //afh_ch_quality[ch_index].sco_score= afh_ch_quality[ch_index].sco_score;
                }
                else if (afh_ch_quality[ch_index].sco_header_quality<=50)
                {
                    afh_ch_quality[ch_index].sco_score -=100;
                }
                else if (afh_ch_quality[ch_index].sco_header_quality<=100)
                {
                    afh_ch_quality[ch_index].sco_score -=200;
                }
                else
                {
                    afh_ch_quality[ch_index].sco_score -=500;
                }

            }
#endif            
        }
    }

#ifdef AFH_EXTENSION_DEBUG
	if( ch_index == LMP_MAX_CHANNELS )
	{
		UINT8 ww =0;
		for( ww=0;ww<7;ww++)
		{
		    RTK_AFH_LOG(GREEN,AFH_DISPALY_BT_AFH_CH_QTY,11,	
			ww,
			afh_ch_quality[ww*10+0].score,
			afh_ch_quality[ww*10+1].score,
			afh_ch_quality[ww*10+2].score,
			afh_ch_quality[ww*10+3].score,
			afh_ch_quality[ww*10+4].score,
			afh_ch_quality[ww*10+5].score,
			afh_ch_quality[ww*10+6].score,
			afh_ch_quality[ww*10+7].score,
			afh_ch_quality[ww*10+8].score,
			afh_ch_quality[ww*10+9].score);
		}

		    RTK_AFH_LOG(GREEN,AFH_DISPALY_BT_AFH_CH_QTY,11,	
			7,
			afh_ch_quality[70].score,
			afh_ch_quality[71].score,
			afh_ch_quality[72].score,
			afh_ch_quality[73].score,
			afh_ch_quality[74].score,
			afh_ch_quality[75].score,
			afh_ch_quality[76].score,
			afh_ch_quality[77].score,
			afh_ch_quality[78].score,0);	
	}
#endif	
}


/**
 * Generates the AFH map for a particular piconet. The map is
 * calcuated from the Pkt loss Ratio
 * The implementation nof this interface can be moved to RAM so that we
 * cusomize for different RF support.
 *
 *
 * \param piconet_id piconet id.
 * \param afh_map Pointer to the AFH map.
 *
 * \return None.
 */
void lmp_cal_afh_channel_quality_RTK(UCHAR piconet_id)     // this function design by RTK Jr. Neil
{
    UINT16   total_pkts;            // to calculate the total packets of ACL type
    UINT16   sco_total_pkts;     // to calculate the total packets of SCO type
    UINT16   bad_pkts;
    UINT16   sco_bad_pkts;
    UCHAR   byte_index,bit_index, ch_index;
    UCHAR   used_channel;
    UCHAR   sco_used_channel;
    UINT32   sum_total_pkts;
    UINT32   sum_bad_pkts;
    UINT32   sum_sco_total_pkts;
    UINT32   sum_sco_bad_pkts;
    UCHAR    avg_total_pkts;
    UCHAR    avg_bad_pkts;
    UCHAR    avg_sco_total_pkts;
    UCHAR    avg_sco_bad_pkts;
    UCHAR  channel_pkts_threshold0;
    UCHAR  channel_pkts_threshold1;
    UCHAR  channel_pkts_threshold2;
    UCHAR  acl_th0_slave;
    UCHAR  acl_th1_slave;
    UCHAR  acl_th2_slave;

    sum_total_pkts = 0;
    sum_bad_pkts =0;
    sum_sco_total_pkts = 0;
    sum_sco_bad_pkts =0;
    used_channel = 0;
    sco_used_channel = 0;
    sco_total_pkts = 0;
    total_pkts = 0;	

    for (ch_index = 0; ch_index < LMP_MAX_CHANNELS; ch_index++)
    {
        if (afh_ch_host[ch_index].host_ch_status == AFH_HOST_BAD_CHANNEL)
        {
            continue;
        }

        bit_index = ch_index & 0x07;
        byte_index = ch_index >> 3;

        if ((afh_map_last[byte_index] & (1<<bit_index)) == 0)
        {
            continue;
        }

        total_pkts = afh_ch_status[ch_index].total_pkts;
        sco_total_pkts = afh_ch_status[ch_index].sco_total_pkts;

        bad_pkts = afh_ch_status[ch_index].num_bad_pkts;
        sco_bad_pkts = afh_ch_status[ch_index].sco_bad_pkts;

#ifndef _AFH_USE_SCO_SIMULATE_ACL_PKT_
        if ((total_pkts >=1) && (sco_total_pkts >=1))
        {
            sum_total_pkts += total_pkts;
            sum_sco_total_pkts += sco_total_pkts;
            sum_bad_pkts += bad_pkts;
            sum_sco_bad_pkts += sco_bad_pkts;

            sco_used_channel++;
            used_channel++;
        }
        else if ((sco_total_pkts >=1) && (total_pkts ==0))
        {
            sum_sco_bad_pkts += sco_bad_pkts;
            sum_sco_total_pkts += sco_total_pkts;
            sco_used_channel++;
        }
        else
#endif
        {
            if ((total_pkts >=1) && (sco_total_pkts ==0))
            {
                sum_total_pkts +=total_pkts;
                sum_bad_pkts +=bad_pkts;
                used_channel++;
            }
        }
    }

    if (used_channel > 0)
    {
        avg_bad_pkts = (sum_bad_pkts/used_channel);
        avg_total_pkts = (sum_total_pkts/used_channel);
    }
    else
    {
        avg_bad_pkts = 0;
        avg_total_pkts = 0;
    }

#ifndef _AFH_USE_SCO_SIMULATE_ACL_PKT_    
    if (sco_used_channel > 0)
    {
        avg_sco_bad_pkts= (sum_sco_bad_pkts/sco_used_channel);
        avg_sco_total_pkts= (sum_sco_total_pkts/sco_used_channel);
    }
    else
#endif        
    {
        avg_sco_bad_pkts = 0;
        avg_sco_total_pkts =0;
    }

    // to show the total pkts and bad pkts
    RTK_AFH_LOG(YELLOW,NUM_OF_CHANNEL_AVG,1, avg_total_pkts);
    RTK_AFH_LOG(YELLOW,SCO_OF_CHANNEL_AVG,1, sco_total_pkts);
    RTK_AFH_LOG(RED, NUM_OF_AVG_BAD_PKTS,1, avg_bad_pkts);
    RTK_AFH_LOG(RED, SCO_OF_AVG_BAD_PKTS,1, avg_sco_bad_pkts);

// for SCO mode using
    if (avg_sco_bad_pkts <=4)
    {
        channel_pkts_threshold0 = 30;
        channel_pkts_threshold1 = 10;
        channel_pkts_threshold2 = 9;

    }
#ifndef _AFH_USE_SCO_SIMULATE_ACL_PKT_
    else
    {
        channel_pkts_threshold0 = avg_sco_bad_pkts*6;
        channel_pkts_threshold1 = avg_sco_bad_pkts*4;
        channel_pkts_threshold2 = avg_sco_bad_pkts*2;
    }
#endif

    acl_th0_slave = (avg_total_pkts>>1);  // 1/2
    acl_th1_slave = ((avg_total_pkts*3)>>2);
    acl_th2_slave = ((avg_total_pkts*7)>>3);

    //========= NeilChen=========2011--04--15======//
    // save channel cal parameter
    //========================
    channel_cal_parameter.acl_th0_slave = acl_th0_slave;
    channel_cal_parameter.acl_th1_slave = acl_th1_slave;
    channel_cal_parameter.acl_th2_slave = acl_th2_slave;
    channel_cal_parameter.channel_pkts_threshold0 = channel_pkts_threshold0;
    channel_cal_parameter.channel_pkts_threshold1 = channel_pkts_threshold1;
    channel_cal_parameter.channel_pkts_threshold2 = channel_pkts_threshold2;
    channel_cal_parameter.avg_bad_pkts = avg_bad_pkts;
    channel_cal_parameter.avg_total_pkts = total_pkts;
    channel_cal_parameter.avg_sco_bad_pkts = avg_sco_bad_pkts;
    channel_cal_parameter.avg_sco_total_pkts = avg_sco_total_pkts;	
    //==========================================	

    lmp_cal_afh_channel_quality_RTK_generator(0, 10, piconet_id);
    
    return;
}

//#################################################
//     Add by Neil Chen to Divide Cal Channel to 2 function 
//     I: to check CH#0~38     II: to check CH#39~78
//#################################################
void lmp_cal_afh_channel_quality_RTK_II(UCHAR piconet_id)     // this function design by RTK Jr. Neil
{
    lmp_cal_afh_channel_quality_RTK_generator(10, LMP_MAX_CHANNELS, piconet_id);
    
    return;
}



//===========  Add functions by Jr. Neil for RTK AFH Mechanism ==========//
void lmp_classify_channel_quality_RTK(LC_RTK_PKT_RECD_S *recd_sig)
{
    UCHAR   channel_id;
    UINT16  channel_hq;
    UINT16  hq1,hq2,hq3;
    UINT16  agc_idx;

#ifdef _ROM_CODE_PATCHED_
    if (rcp_lmp_classify_channel_quality_RTK_func != NULL)
    {
        if (rcp_lmp_classify_channel_quality_RTK_func((void*)recd_sig))
        {
            return;
        }
    }   
#endif
    
    channel_id = recd_sig->channel_id;
    
    // need to add more function ---2010--08--26---

    //ACCAGC_QUALITY bt_bb_reg70;

    // Header Quality Calculate
    hq1 = recd_sig->backup_status_rpt3 & 0x0f;
    hq2 = (recd_sig->backup_status_rpt3 >> 4) & 0x7ff;
    hq3 =  recd_sig->backup_status_rpt2 >> 9;

    agc_idx= (recd_sig->backup_status_rpt4 >> 10);
#ifdef _YL_MODEM_RSSI_MAPPING
    agc_idx = lc_modem_rssi_mapping(agc_idx);
    // agc_idx = MAX(0,(INT8)agc_idx+(otp_str_data.efuse_rssi0_pin_dBm_div2+45));
#endif

    if (hq2 > 0)
    {
        channel_hq = ((hq1 << 7) | hq3) / hq2;
    }
    else
    {
        channel_hq = 10;
    }

//=============  Add by Neil Chen to add the distingish part for SCO and ACL ==========//

    /* use SCO pkt to simulate ACL pkt and CRC indication */
#ifndef _AFH_USE_SCO_SIMULATE_ACL_PKT_
    if (!IS_NO_AFH_SCO_SIMULATE_ACL_PKT)
#endif
    {
        if (recd_sig->is_sco_rx)
        {
            if (recd_sig->hec_err == FALSE)
            {
                recd_sig->crc_err = (channel_hq > 1) ? TRUE : FALSE;            
            }
            recd_sig->is_sco_rx = FALSE;
        }
    }

#ifndef _AFH_USE_SCO_SIMULATE_ACL_PKT_
    if (recd_sig->is_sco_rx)
    {
        afh_ch_status[channel_id].sco_total_pkts++;
        if (recd_sig->access_code_miss == TRUE)
        {
            afh_ch_status[channel_id].sco_bad_pkts++;
        }
        else
        {
            //========== access code sync but hec check error =========//
            if (recd_sig->hec_err == TRUE)
            {
                afh_ch_status[channel_id].sco_unknownII_pkts++;
            }
            else  // HEC coorect , SCO only could check the HEC, couldn't check CRC
            {
                afh_ch_status[channel_id].sco_unknownI_pkts++;
            }
            afh_ch_status[channel_id].sco_header_quality += channel_hq;            

            bt_afh_rssi.rssi_count += agc_idx;
            bt_afh_rssi.rssi_pkts++;
        }
    }
    else
#endif        
    {
        afh_ch_status[channel_id].total_pkts++;
        if (recd_sig->access_code_miss == TRUE)
        {
            afh_ch_status[channel_id].num_bad_pkts++;
        }
        else
        {
            //========== access code sync but hec check error =========//
            if (recd_sig->hec_err == TRUE)
            {
                afh_ch_status[channel_id].num_unknownII_pkts++;
            }
            else
            {
                if (recd_sig->crc_err == TRUE)       // HEC CORRECT but CRC failure
                {
                    afh_ch_status[channel_id].num_unknownI_pkts++;
                }
                else      //CRC correct
                {
                    afh_ch_status[channel_id].num_good_pkts++;
                }
            }
            afh_ch_status[channel_id].header_quality += channel_hq;            
            
            bt_afh_rssi.rssi_count += agc_idx;
            bt_afh_rssi.rssi_pkts++;

            if (bt_afh_rssi.rssi_pkts == 512)
            {
                /* avoid overflow -- austin */
                bt_afh_rssi.rssi_count = agc_idx;                
                bt_afh_rssi.rssi_pkts = 1;
            }              
        }
    }
}

//############################################################

//##############################################################
//     End RTK AFH Mechanism Functions
//#################################################

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//============All Use Functions==== RTK AFH Mechanims and MindTree both used this functions
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//======== Handle the Access Code Miss Interrupt ===========//
void lmp_afh_la_handle_access_code_miss_interrupt()
{
    UINT16 channel;
    channel = BB_read_baseband_register(CHANNEL_REGISTER);
    channel = ((channel >> 8) & 0x7F);

    lmp_update_channel_quality(channel, 0,
                               AFH_ACCESS_ERROR_PKT_WEIGHT);
}

/**
 * Initializes the local assessment.
 *
 * \param None.
 *
 * \return None.
 */
void init_channel_assessment_module(UINT8 load_default_type)
{    
    UINT8 i;
    UINT8 temp;

    //====Init the AFH map last=====all using//
     for (i = 0; i < LMP_AFH_MAP_SIZE; i++)
    {
        afh_host_classification[i] = afh_ahs79_channel_map[i];
        afh_host_classification_last[i] = afh_ahs79_channel_map[i];	
        afh_map_psd[i] = afh_ahs79_channel_map[i];
        afh_map_last[i] = afh_ahs79_channel_map[i];	
        unknow_map[i] = 0xFF;
        unknow_map_II[i] = 0xFF;    // default is no channel
    }
    wifiwtmailbox = 0x0;
    unk_channel_num=0x0;
    unkII_channel_num=0x0;	      
    //bt_afh_rssi->avg_rssi = 0x0;
    bt_afh_rssi.rssi_pkts = 0x0;
    bt_afh_rssi.rssi_count = 0x0;
    rssi_result = 0x0;	
    rssi_result_last = 0;    
    afh_piconet = 0xFF;

#ifdef RTL8723A_B_CUT
    afh_score_threshold_sco = otp_str_data.trk_afh_score_thres_sco << 2;
    afh_score_threshold_1 = otp_str_data.trk_afh_score_thres_i << 4;
    afh_score_threshold_2 = otp_str_data.trk_afh_score_thres_ii << 4;
#endif

    afh_repeat_check_count = 0;


#ifdef _NEW_MODEM_PSD_SCAN_    
    if (load_default_type)
    {
    }
    else
    {
        // TODO:
    }

    if (otp_str_data.rtk_afh_bt_psd_enable)
    {        
        g_rtk_afh_modem_psd_internal_enable = g_efuse_modem_psd_setting_1.rtk_afh_modem_psd_enable;
        // TODO: Init  modem_psd_scan_man and  modem_psd_result_man ??
        //    TODO: 

        modem_psd_man_init();
    }
#endif
   
    temp = load_default_type ? INITIAL : NORMAL;

    for (i = 0x0; i < LMP_MAX_PICONETS_SUPPORTED; i++)
    {
        afh_state[i] = temp;
        g_afh_times[i] = 0;
        afh_recovery_cnt[i] = 0;
        afh_recovery_cnt1[i] = 0;
        afh_recovery_cnt2[i] = 0;
        afh_recovery_cnt3[i] = 0;
        afh_recovery_cnt4[i] = 0;
        mon_reg[i].channel0 = 0xFF;
        mon_reg[i].channel1 = 0xFF;
        mon_reg[i].channel2 = 0xFF;
        mon_reg[i].sel_bank = 0x00;
        mon_reg[i].wlan_nbi = 0x01; // to set wlan recovery
        mon_reg[i].downtoup =0x00;  // default is down to up
        //afh_execute_times[i] = MAX_AFH_EXECUTE_TIMES;  // initial the afh execute times for using
    }

    /* speed up process time */
    memset(afh_ch_status, 0, sizeof(AFH_CH_ASSESSMENT_NODE)*LMP_MAX_CHANNELS);
    memset(afh_ch_quality, 0, sizeof(CHANNEL_CLASSIFY_NODE)*LMP_MAX_CHANNELS);
    memset(afh_ch_host, AFH_HOST_UNKNOWN_CHANNEL, 
                            sizeof(HOST_CHANNEL_MASK)*LMP_MAX_CHANNELS);    // to init as unknow channel by host

    //show channel ID for channel hopping Sequence
    RTK_AFH_LOG(GREEN,AFH_INIT_CHANNEL_ASSESSMENT_MODULE,0,0);
    
}

#ifdef _NEW_MODEM_PSD_SCAN_
TimerHandle_t scan_period_timer = NULL; // Handler: lmp_modem_psd_scan_period_timer_expiry()
UINT16 scan_period_timeout_ms = 500; // 600ms (?)
TimerHandle_t scan_tdm_timer = NULL; // Handler: lmp_modem_psd_scan_tdm_timer_expiry();   for time-diversity, interlacing PSD Scan is desired?
UINT16 scan_tdm_timeout_ms = 40; // 20ms
TimerHandle_t afh_map_gen_timer = NULL; // Handler: lmp_modem_psd_afh_map_gen_timer_expiry()  (optionally used by efuse_modem_psd_setting_1.modem_psd_afh_map_gen_instant_opt = 1)
UINT16 afh_map_gen_timerout_ms = 2500;
#endif

/**
 * Initializes the local assessment. This function will be called after the
 * first LM connection is established, and AFH can be enabled for the
 * connection.
 *
 * \param None.
 *
 * \return None.
 */
void lmp_init_assessment_at_connection()
{
    /* Create and start timer */
    if (la_period_timer == NULL)
    {
        OS_CREATE_TIMER(ONESHOT_TIMER, &la_period_timer,
                lmp_handle_channel_assess_timer_expiry, 0, afh_la_cycle_period);
    }

    
    // add by Neil Chen---2011---04--13--- 	
    if (la_classify_timer == NULL)
    {
        OS_CREATE_TIMER(ONESHOT_TIMER, &la_classify_timer,
                lmp_handle_channel_classify_timer_expiry, 0, afh_channel_classify_period);
    }
    //---------------------------------------------------------------------------

#ifdef _BT_ONLY_
#ifdef _NEW_MODEM_PSD_SCAN_
    if (otp_str_data.rtk_afh_bt_psd_enable)
    {
        if (scan_period_timer == NULL)
        {
            OS_CREATE_TIMER(ONESHOT_TIMER, &scan_period_timer,
                    lmp_modem_psd_scan_period_timer_expiry, 0, scan_period_timeout_ms);
        }
        if (scan_tdm_timer == NULL)
        {
            OS_CREATE_TIMER(ONESHOT_TIMER, &scan_tdm_timer,
                    lmp_modem_psd_scan_tdm_timer_expiry, 0, scan_tdm_timeout_ms);
        }
        if (afh_map_gen_timer == NULL)
        {
            OS_CREATE_TIMER(ONESHOT_TIMER, &afh_map_gen_timer,
                    lmp_modem_psd_afh_map_gen_timer_expiry, 0, afh_map_gen_timerout_ms);
        }
    }
#endif
#endif

    //lmp handle channel assess timer expiry
    RTK_AFH_LOG(GREEN,AFH_HANDLE_CHANNEL_ASSESS_TIMER,1,afh_la_cycle_period );

    OS_START_TIMER(la_period_timer, afh_la_cycle_period);
    return;
}

void lmp_afh_state_reset(UINT8 piconet_id)
{
    afh_state[piconet_id] = INITIAL; 
    
    //g_afh_times[piconet_id] = 0;		// wifi use this variable			
    // reset state
    afh_recovery_cnt[piconet_id] = 0;			        	
    afh_recovery_cnt1[piconet_id] = 0;
    afh_recovery_cnt2[piconet_id] = 0;
    afh_recovery_cnt3[piconet_id] = 0;
    afh_recovery_cnt4[piconet_id] = 0;
    all_on_map_cnt[piconet_id] = 0;			

}
#endif /* COMPILE_CHANNEL_ASSESSMENT */

