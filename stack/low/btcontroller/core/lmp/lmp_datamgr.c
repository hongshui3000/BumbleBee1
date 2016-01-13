/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  Contains the code that implements Self device database initialization,
 *  Connection entity database initialization and hash functions to access
 *  CE database.
 */

/********************************* Logger *************************/
enum { __FILE_NUM__= 51 };
/********************************* Logger *************************/


/* ========================= Include File Section ========================= */
#include "lmp_internal.h"
#include "bt_fw_globals.h"
#include "btc_defines.h"
#include "bz_debug.h"
#include "bz_auth.h"

#include "vendor.h"
#include "mem.h"
#ifdef COMPILE_CHANNEL_ASSESSMENT
#include "lmp_ch_assessment.h"
#endif /* COMPILE_CHANNEL_ASSESSMENT */

#include "otp.h"
#include "le_ll.h"
#include "le_ll_driver.h"
#include "gpio.h"

#ifdef _CCH_SC_ECDH_P256_
#include "bz_auth_internal.h"
#endif

#ifdef CONFIG_TV_POWERON_LPS
#include "tv.h"
#endif

#ifdef _DAPE_TEST_FOR_HID_SCO
extern UINT8 lc_sco_pause_status;
#endif
#ifdef _DAPE_EN_8821_MP_MODIFY_SLV_SNIFF_TIMEOUT
extern UINT8 lc_sniff_slv_send_pkt;
#endif
#ifdef _DAPE_TEST_NEW_HW_BLOCK_ACL_FOR_LE
extern UINT8 g_enable_le_block_legacy;
extern UINT8 lc_le_pause_status;
extern UINT8 g_block_legacy_for_le_slot_num;
#endif
#ifdef _DAPE_SLV_SUPTO_IMPROVE
UINT8 g_slv_tpoll_disabled = 0xFF;
TimerHandle_t en_slv_tpoll_timer = NULL;
void en_slv_tpoll_callback(TimerHandle_t timer_handle);
#endif
/* ==================== Structure declaration Section ===================== */



/* ===================== Variable Declaration Section ===================== */
//{{added by guochunxia
UINT16 defined_voice_setting = VOICE_SETTINGS_CVSD_MASK;
//}}

/** Inquiry result table */
LMP_INQUIRY_RESULT_DATA lmp_inquiry_result_data[LMP_MAX_INQUIRY_RESULT_DATA];

/** Self device data  */
LMP_SELF_DEVICE_DATA lmp_self_device_data;

#ifndef _RTL8723B_DUT_MODE_
#if defined(TEST_MODE) && !defined(_DUT_DELAYED_LOOPBACK_MODE_)
SECTION_SRAM UINT8 lmp_test_mode_data_buf[1024];
#endif
#endif

#ifdef _DAPE_TEST_AUTO_CONN
SECTION_SRAM LEGACY_WHITE_LIST conn_white_list[LEGACY_MAX_WHITE_LIST];
SECTION_SRAM UINT8 num_of_white_list_device;
#endif

ALIGN(4) SECTION_SRAM UINT8 lmp_self_eir_data_buf[MAX_EIR_DATA_LEN];

#ifdef _MONITOR_ACLU_TRX_THROUGHPUT_
HCI_CONN_HANDLE_ACLU_MEASURE_UNIT hci_conn_handle_aclu_measure;
#endif

/** Piconet Specific Information database */
LMP_FEATURE_DATA lmp_feature_data;

/** ACL Connection Entity database */
#ifndef _ENABLE_RETENTION_FLOW_FOR_DLPS_
LMP_CONNECTION_ENTITY lmp_connection_entity[LMP_MAX_CE_DATABASE_ENTRIES];
#else
UINT16 lmp_connection_entity_store_index;
LMP_CONNECTION_ENTITY lmp_connection_entity_bton;
SECTION_LOW_BSS LMP_CONNECTION_ENTITY lmp_connection_entity[LMP_MAX_CE_DATABASE_ENTRIES];
#endif

#ifdef ENABLE_SCO
/** SCO Connection entity database */
LMP_SCO_CONNECTION_DATA lmp_sco_connection_data[LMP_MAX_SCO_CONN_ENTRIES];
#endif

/** Host Information database */
LMP_HOST_INFO_DATA lmp_host_info_data;

/** AM Addr to CE index mapping table */
LMP_AM_ADDR_TO_CE_INDEX_TABLE
lmp_am_addr_to_ce_index_table_ppi[LC_MAX_AM_ADDR][LMP_MAX_PICONETS_SUPPORTED];

/** Conn handle to CE index mapping table */
LMP_CONN_HANDLE_TO_CE_INDEX_TABLE
lmp_ch_to_ce_index_table[LMP_MAX_CONN_HANDLES];

/** Piconet ID table */
UINT16 piconet_id_table[LMP_MAX_PICONETS_SUPPORTED];

/** Role switch information table */
LMP_ROLE_SWITCH_PARAMS lmp_role_switch_data;

UCHAR lmp_num_packets_timer_flag;
UINT16 lmp_assigned_ce_index = 0;

/** Index to inquiry data table */
UCHAR lmp_inquiry_data_index;
UCHAR lmp_event_filters_written;

/**
 * Master Slave Switch State. Once the role switch is agreed set this variable
 * to indicate waiting for FHS packet.
 */
UCHAR lmp_mss_state;

/**PATCH_PENDING_ENCRY_WHILE_ROLE_SWITCH*/
#ifdef _PENDING_ENCRY_WHILE_ROLE_SWITCH
UINT8 g_need_pend_encry;
UINT8 g_encry_pending;
OS_SIGNAL g_signal_ptr_encry;
#endif
#ifdef _STOP_AFH_TIMER_WHILE_ROLE_SWITCH
UINT8 g_stop_afh_timer;
#endif

const UINT16 g_lc_pkt_type_lut[16] = {
    BB_DV_LUT,
    BB_2_DH1_LUT,
    BB_3_DH1_LUT,
    BB_DM1_LUT,
    BB_DH1_LUT,
    BB_ERR_PKT,
    BB_ERR_PKT,
    BB_ERR_PKT,
    BB_2_DH3_LUT,
    BB_3_DH3_LUT,
    BB_DM3_LUT,
    BB_DH3_LUT,
    BB_2_DH5_LUT,
    BB_3_DH5_LUT,
    BB_DM5_LUT,
    BB_DH5_LUT
};

const UINT16 g_lc_pkt_type_max_len[16] = {
    LC_MAX_NUMBER_OF_BYTES_IN_DV_PACKET,
    LC_MAX_NUMBER_OF_BYTES_IN_2_DH1_PACKET,
    LC_MAX_NUMBER_OF_BYTES_IN_3_DH1_PACKET,
    LC_MAX_NUMBER_OF_BYTES_IN_DM1_PACKET,
    LC_MAX_NUMBER_OF_BYTES_IN_DH1_PACKET,
    0,
    0,
    0,
    LC_MAX_NUMBER_OF_BYTES_IN_2_DH3_PACKET,
    LC_MAX_NUMBER_OF_BYTES_IN_3_DH3_PACKET,
    LC_MAX_NUMBER_OF_BYTES_IN_DM3_PACKET,
    LC_MAX_NUMBER_OF_BYTES_IN_DH3_PACKET,
    LC_MAX_NUMBER_OF_BYTES_IN_2_DH5_PACKET,
    LC_MAX_NUMBER_OF_BYTES_IN_3_DH5_PACKET,
    LC_MAX_NUMBER_OF_BYTES_IN_DM5_PACKET,
    LC_MAX_NUMBER_OF_BYTES_IN_DH5_PACKET
};

#ifdef COMPILE_ESCO
extern LMP_ESCO_CONN_HANDLE_TO_CE_INDEX_TABLE
lmp_esco_ch_to_ce_index_table[LMP_MAX_ESCO_CONN_HANDLES];
#endif /* COMPILE_ESCO */

extern LC_PICONET_SCHEDULER lc_piconet_scheduler[LMP_MAX_PICONETS_SUPPORTED];

/* ================== Static Function Prototypes Section ================== */
void lmp_init_connection_entity_on_power_on(UINT16 ce_index);


/* ================== For rom code patch function point ================== */
#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8723_RCP_
PF_ROM_CODE_PATCH_VOID rcp_lmp_update_lps_para = NULL;
#endif
PF_ROM_CODE_PATCH_FUNC rcp_sup_to_pwa_func = NULL;
#endif

/* ===================== Function Definition Section ====================== */
/**
 * Initialize all the LMP module global data structures and variables. The
 * main data structures initialized are #lmp_self_device_data,
 * #lmp_connection_entity.
 *
 * \param None.
 *
 * \return None.
 */
void lmp_initialize_host_controller(void)
{
    UINT16 index = 0;

    /* Initializing the self device strucure */
    lmp_init_device_attributes();

    /* Initializing the Connection Entities database */
    for (index = 0; index < LMP_MAX_CE_DATABASE_ENTRIES; index++)
    {
        lmp_init_connection_entity_on_power_on(index);
    }

    /* Initializing piconet specific datas*/
    lmp_init_piconet_database();

    /* Initializing the inquiry result table */
    for(index = 0; index < LMP_MAX_INQUIRY_RESULT_DATA; index++)
    {
        memset(&lmp_inquiry_result_data[index],0,
               sizeof(LMP_INQUIRY_RESULT_DATA));
    }

    /* Initializing the data base access tables */
    lmp_init_db_acces_tables();

    /* Inititalizing Host information structure. */
    memset(&lmp_host_info_data,0 , sizeof(LMP_HOST_INFO_DATA));

    /* Initializing the SCO connection data table*/
#ifdef ENABLE_SCO
    lmp_init_sco_connection_table();
#endif /* ENABLE_SCO */

    if (IS_USE_FOR_BQB)
    {
        afh_la_cycle_period = 10000;

#ifndef _IS_ASIC_
        /* in asic, we change bt efuse options for flexibility */
        otp_str_data.bt_func_support_policy |= 0x0020;     // set bqb mode for special process
        otp_str_data.bt_func_support_policy &= ~0x40000000;// disable pta meter
        otp_str_data.bt_func_support_policy_ext &= ~BIT15; // disable SHORTEN_MASTER_SNIFF_TIMEOUT_VAL
        otp_str_data.bt_global_slot_min_sniff_intv = 0x12; // set bt_global_slot_min_sniff_intv
        otp_str_data.bt_le_fw_policy &= ~(BIT27 | BIT28);  // let le advertising report event be one device only
#endif
    }

#if defined(ENABLE_SCO) || defined(COMPILE_ESCO)
#ifdef SCO_OVER_HCI

/*  Updated by Wallice Su for PCM Enhancement. 2013/07/08  */
    hci_excodec_state = otp_str_data.hci_excodec_state & 0x0F;

#if 0
    if ((otp_str_data.hci_excodec_state == 0x1) ||
        (otp_str_data.hci_excodec_state == 0x2)) //Over CODEC
#else
    if ((hci_excodec_state == 0x1) || (hci_excodec_state == 0x2)) //Over CODEC
#endif
    {

        if (hci_excodec_state == 0x1)
        {
           RT_BT_LOG(RED,LC_SYNC_LINKS_293,0,0);
        }
        else if (hci_excodec_state == 0x2)
        {
           RT_BT_LOG(RED,MSG_CODEC_I2S_0,0,0);
        }

        sync_link_codec_state = OVER_CODEC; /* NOT_APPLICABLE; */
        lmp_esco_over_codec = TRUE;

        scoconv = otp_str_data.scoconv;
        pcmifctrl1 = otp_str_data.pcm_if_ctrl1;
        pcmifctrl2 = otp_str_data.pcm_if_ctrl2;
        pcmifctrl3 = otp_str_data.pcm_if_ctrl3;
        pcmconvert = otp_str_data.pcm_convert;

	/*  Updated by Wallice Su for PCM Enhancement. 2013/07/08  */
//        pcm_ex_codec_format_8bit = otp_str_data.pcm_ex_codec_format_8bit;
        pcm_ex_codec_format_8bit = (otp_str_data.pcm_if_ctrl1 >> 0x06) & 0x01;
//        pcm_ex_codec_format = otp_str_data.pcm_ex_codec_format;
        pcm_ex_codec_format = (otp_str_data.hci_excodec_state >> 0x04) & 0xF;
    }
    else //Over HCI
    {
        RT_BT_LOG(RED,MSG_LC_SCO_OVER_HCI,0,0);
        sync_link_codec_state = OVER_HCI;//OVER_CODEC; /* NOT_APPLICABLE; */
        lmp_esco_over_codec = FALSE;
    }
/*  End Updated by Wallice Su.  2011/01/03  */

#endif /* SCO_OVER_HCI */

#ifdef ENABLE_PCM_ONLY_WHEN_SCO_LINK_CREATED
    lc_pcm_enable_control(0);
#endif

    sync_link_codec_type = PCM_CODEC;   /* NOT_APPLICABLE; */  // deleted by guochunxia 20090519
    pcm_codec_availability = TRUE;
    uda_codec_availability = TRUE;
#endif /* ENABLE_SCO || COMPILE_ESCO */

#ifndef _RTL8821A_
    g_global_slot_min_allow_sniff_int = 30;
#else
    g_global_slot_min_allow_sniff_int = otp_str_data.bt_global_slot_min_sniff_intv;
#endif

    init_cache_table();
    lmp_init_glob_vars();
    INIT_ESCO_MAPPING_TABLES();
    INIT_ESCO_CONNECTION_DATA();

#ifdef COMPILE_CHANNEL_ASSESSMENT
    init_channel_assessment_module(TRUE);
#endif

    bz_auth_reset();
    bz_auth_register_auth_completed_callback(lmp_auth_completed_callback, NULL);
    bz_auth_register_pause_encryption_callback(lmp_pause_encryption_callback,
            NULL);
    bz_auth_register_resume_encryption_callback(lmp_resume_encryption_callback,
            NULL);

    lmp_edtm_init();

    return;
}

/**
 * Initialize the piconet database.
 *
 * \param None.
 *
 * \return None.
 */
void lmp_init_piconet_database(void)
{
    /* zeroize all bytes in supported feature pages */
    memset((void *)(&lmp_feature_data), 0, sizeof(lmp_feature_data));

    /* initialize page 0 setting */
    memcpy(lmp_feature_data.feat_page0,
            otp_str_data.bt_master_piconet_fea, BTC_FEATURES_LEN);

    /* zeroize possible invalid bit in unsupported spec */

#ifdef VER_3_0
    if (!IS_BT30)
    {
        /* disbale EPC */
        lmp_feature_data.feat_page0[7] &= ~BIT2;
    }
#endif

#ifdef LE_MODE_EN
    if (!IS_BT40)
    {
        /* disable LE supported (controller) */
        lmp_feature_data.feat_page0[4] &= ~BIT6;

        /* disable simultaneous LE and BR/EDR to same device
           capable (controller) */
        lmp_feature_data.feat_page0[6] &= ~BIT1;
    }
#endif

    /* initialize page 1 setting */

#ifdef _SUPPORT_EXT_FEATURES_PAGE_2_
    /* initialize page 2 setting */
#ifdef VER_CSA4
    if (IS_BT_CSA4)
    {
#ifdef _SUPPORT_CSB_TRANSMITTER_
        lmp_feature_data.features[2][0] |= CSB_MASTER; /* CSB master */
        lmp_feature_data.features[2][0] |= SYNCHRONIZATION_TRAIN_MASTER; /* STP master */
        lmp_feature_data.features[2][0] |= INQ_NOTIFICATION_EVT; /* inquiry response
                                                    notification event */
#endif

#ifdef _SUPPORT_CSB_RECEIVER_
        lmp_feature_data.features[2][0] |= CSB_SLAVE; /* CSB slave */
        lmp_feature_data.features[2][0] |= SYNCHRONIZATION_TRAIN_SCAN_SLAVE; /* STP slave */
#endif
    }
#endif /* end of VER_CSA4 */

#ifdef _SUPPORT_SECURE_CONNECTION_
    if (IS_BT_SECURE_CONN)
    {
        lmp_feature_data.features[2][1] |= SECURE_CONNECTION_CONTROLLER_SUPPORT; /* secure connections (controller support) */
        lmp_feature_data.features[2][1] |= SUPPORT_PING; /* Ping */
    }
#endif
#ifdef _SUPPORT_VER_4_1_
    if (IS_BT41)
    {
#ifdef _SUPPORT_PCA_ADJUST
        lmp_feature_data.features[2][0] |= COARSE_CLOCK_ADJUSTMENT; /* Coarse Clock Adjustment. */
#endif
    }
#endif

#endif /* end of _SUPPORT_EXT_FEATURES_PAGE_2_ */

#ifdef COMPILE_FEATURE_REQ_EXT
    /* update max features page */
    lmp_get_max_features_page();
#endif
#ifdef _DAPE_TEST_DISABLE_AFH
    lmp_feature_data.feat_page0[4] &= (~AFH_CAPABLE_SLAVE);
    lmp_feature_data.feat_page0[4] &= (~AFH_CLASSIFICATION_SLAVE);
    lmp_feature_data.feat_page0[5] &= (~AFH_CAPABLE_MASTER);
    lmp_feature_data.feat_page0[5] &= (~AFH_CLASSIFICATION_MASTER);
#endif

    /* end of initialize all feature page */
}

/**
 * Initialize the cache table (#bd_addr_cache).
 *
 * \param None.
 *
 * \return None.
 */
void init_cache_table(void)
{
    UCHAR index;

    for(index = 0; index < (MAX_CACHE - 1); index++)
    {
        memset(&bd_addr_cache[index],0,sizeof(CACHE_TABLE));
        bd_addr_cache[index].state = CACHE_FREE;
        bd_addr_cache[index].next_index = (UCHAR)(index + 1);
        bd_addr_cache[index].prev_index = (UCHAR)(index - 1);
        bd_addr_cache[index].cur_index = index;
    }

    /* Make the last node point to an invalid index */
    memset(&bd_addr_cache[index],0,sizeof(CACHE_TABLE));
    bd_addr_cache[index].next_index = INVALID_INDEX;
    bd_addr_cache[index].prev_index = (UCHAR)(index - 1);
    bd_addr_cache[index].cur_index = index;

    bd_addr_cache_free = 0;
    bd_addr_cache_start = 0;
}

/**
 * Initialize all the LMP hash tables (Connection entity database access
 * tables).
 *
 * \param None.
 *
 * \return None.
 */
void lmp_init_db_acces_tables(void)
{
    UINT16 index;

    memset(&lmp_am_addr_to_ce_index_table_ppi[0][0], 0,
           (sizeof(LMP_AM_ADDR_TO_CE_INDEX_TABLE)
            * LC_MAX_AM_ADDR
            * LMP_MAX_PICONETS_SUPPORTED));

    for(index = 0; index < LMP_MAX_CONN_HANDLES; index++)
    {
        lmp_ch_to_ce_index_table[index].status = UNASSIGNED ;
    }

    for(index = 0 ; index < LMP_MAX_PICONETS_SUPPORTED ; index++)
    {
        piconet_id_table[index] = UNASSIGNED ;
    }
}

#ifdef ENABLE_SCO
/**
 * Initialize the sco connection table (#lmp_sco_connection_data).
 *
 * \param None.
 *
 * \return None.
 */
void lmp_init_sco_connection_table(void)
{
    UCHAR index = 0;

    /*Initializing the SCO connection data */
    for(index = 0; index < LMP_MAX_SCO_CONN_ENTRIES; index++)
    {
        lmp_init_sco_connection_entity(index);
    }
}

/**
 * Initialize a single SCO connection entity entry.
 *
 * \param index Index of the #lmp_sco_connection_data to be initialized.
 *
 * \return None.
 */
void lmp_init_sco_connection_entity(const UCHAR index)
{
    LMP_SCO_CONNECTION_DATA *sco_ce_ptr = NULL;

    BZ_ASSERT(index < LMP_MAX_SCO_CONN_ENTRIES, "Array index out of bound");

    sco_ce_ptr = &lmp_sco_connection_data[index];

    sco_ce_ptr->sco_handle = (UCHAR)(index+1);

    /* Pre assinged SCO handles */
    sco_ce_ptr->sco_conn_handle = (UINT16)(index + LMP_MAX_CE_DATABASE_ENTRIES
                                           + LMP_MAX_BC_CONNECTIONS + 1);
    sco_ce_ptr->sco_conn_status = SCO_FREE;
    lmp_sco_set_trans_status(sco_ce_ptr, TRS_INVALID);
    sco_ce_ptr->conn_entity_index = 0xff ;
    sco_ce_ptr->pkt_type = 0x00;
    sco_ce_ptr->sco_number = (UCHAR)index;
    sco_ce_ptr->Tsco = 0x00;
    sco_ce_ptr->Dsco = 0x00;
    sco_ce_ptr->time_control_flags = 0xff;
    sco_ce_ptr->air_mode = 0xff;
    sco_ce_ptr->gen_conn_complete = FALSE; /* default Sync_Conn_Complete_Evt */

    sco_ce_ptr->host_allowed_packets = 0;

    sco_ce_ptr->codec_type = NOT_APPLICABLE;

    sco_ce_ptr->fifo_num = SYNC_FIFO1;

#ifdef SCO_OVER_HCI
    sco_ce_ptr->sch_valid_flag = FALSE;
    sco_ce_ptr->bb_pkt_type = 0;
    sco_ce_ptr->pkt_length = 0;
    sco_ce_ptr->codec_state = NOT_APPLICABLE;

#endif /* SCO_OVER_HCI */


#ifdef _CCH_SCO_OUTPUT_ZERO_WHEN_NO_RX_
    sco_ce_ptr->sco_no_rx_count = 0;
    sco_ce_ptr->sco_no_rx_force_output_zero_en = 0;
#endif


}

/**
 * Free the resources occupied by the SCO connection pointed by \a
 * sco_ce_index. This function correctly handles the order in which the
 * resources have to be freed.
 *
 * \param sco_ce_index The index of the SCO connection under consideration.
 *
 * \return None.
 */
void lmp_sco_connection_data_cleanup_after_detach(int sco_ce_index)
{
    LMP_SCO_CONNECTION_DATA* sco_ce_ptr;
    sco_ce_ptr = &lmp_sco_connection_data[sco_ce_index];

#ifdef _CCH_SLOT_OFFSET_
    lmp_put_global_slot_offset(sco_ce_ptr->conn_entity_index, sco_ce_index  + (LMP_MAX_CE_DATABASE_ENTRIES<<1) );
#endif
    if (lmp_self_device_data.total_no_of_sco_conn == 0)
    {
        lmp_self_device_data.sco_pkt_type = 0x00 ;
    }
#ifdef _DAPE_ALLOW_SYNC_LINK_FOR_WIN8
    /* to prevent that there is another sco connecting in the same ce.*/
    UINT8 ii = 0;
    UINT8 ce_sco_no_reset = 0;
    LMP_SCO_CONNECTION_DATA* sco_ce_ptr_other;
    for (ii = 0; ii< 3; ii++)
    {
        sco_ce_ptr_other = &lmp_sco_connection_data[ii];
        if ((sco_ce_ptr_other->sco_conn_status == SCO_CONNECTING) &&
            (sco_ce_ptr_other->conn_entity_index ==
            sco_ce_ptr->conn_entity_index))
        {
            ce_sco_no_reset = 1;
        }
    }
    if (!ce_sco_no_reset)
#endif
    /* reset the sco and codec status */
    {
    lmp_connection_entity[sco_ce_ptr->conn_entity_index].sco_status_byte = 0;
    lmp_connection_entity[sco_ce_ptr->conn_entity_index].codec_status_byte = 0;
    }
#ifdef SCO_OVER_HCI
    if (sco_ce_ptr->codec_state == OVER_CODEC)
    {
#endif /* SCO_OVER_HCI */
        /* Free the codec ('global resources') */
        if (sco_ce_ptr->codec_type == PCM_CODEC)
        {
            pcm_codec_availability = TRUE;
        }
        else
        {
            uda_codec_availability = TRUE;
        }
#ifdef SCO_OVER_HCI
    }
    else    /* sco_ce_ptr->codec_state != OVER_CODEC */
    {
    } /* end if (sco_ce_ptr->codec_state == OVER_CODEC) */
#endif /* SCO_OVER_HCI */

    lmp_init_sco_connection_entity((UCHAR)sco_ce_index);
}
#endif /* ENABLE_SCO */


/**
 * Initialize all the global variables used by LMP module.
 *
 * \param None.
 *
 * \return None.
 */
void lmp_init_glob_vars(void)
{
    /* Initializing the Global varialbes */
    lmp_num_inq_resp_expected = 0 ;
    lmp_num_inq_resp_received = 0 ;
    lmp_event_filters_written = 0;
    lmp_inquiry_data_index = 0;

    lmp_set_mss_state(LMP_MSS_INIT);
    /**PATCH_PENDING_ENCRY_WHILE_ROLE_SWITCH*/
#ifdef _PENDING_ENCRY_WHILE_ROLE_SWITCH
	g_need_pend_encry = FALSE;
	g_encry_pending = FALSE;
#endif
#ifdef _STOP_AFH_TIMER_WHILE_ROLE_SWITCH
    g_stop_afh_timer = FALSE;
#endif

    lmp_num_packets_timer_flag = 0;
    lmp_assigned_ce_index =0 ;
    bc_ce_index = 0; /* Broadcast ce_index */

    lmp_role_switch_data.ce_index = 0xFF;
#ifdef COMPILE_AFH_HOP_KERNEL
    lmp_role_switch_data.old_afh_mode = AFH_DISABLE;
#endif
    lmp_role_switch_data.old_am_addr = INVALID_AM_ADDR;
    lmp_role_switch_data.new_am_addr = INVALID_AM_ADDR;
    lmp_role_switch_data.old_lut_index = INVALID_LUT_INDEX;
    lmp_role_switch_data.new_lut_index = INVALID_LUT_INDEX;

    lmp_periodic_inquiry = FALSE;

    return;
}

/**
 * Reset all the LMP timers.
 *
 * \param ce_index Connection entity index of the ACL connection.
 *
 * \return None.
 */
void lmp_reset_timers(UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY *ce_ptr;
    ce_ptr = &lmp_connection_entity[ce_index];

#ifdef COMPILE_PARK_MODE
    if(ce_ptr->park_mode_timer_handle != NULL)
    {
        OS_DELETE_TIMER( & ce_ptr->park_mode_timer_handle);
    }
#endif /* COMPILE_PARK_MODE */

#ifdef COMPILE_SNIFF_MODE
    if(ce_ptr->sniff_sw_timer_handle != NULL)
    {
        OS_DELETE_TIMER( & ce_ptr->sniff_sw_timer_handle);
    }
#endif /* COMPILE_SNIFF_MODE */

    if(ce_ptr->conn_accept_timer_handle != NULL)
    {
        OS_DELETE_TIMER( & ce_ptr->conn_accept_timer_handle);
    }

    if(ce_ptr->detach_connection_timer_handle != NULL)
    {
        OS_DELETE_TIMER( & ce_ptr->detach_connection_timer_handle);
    }

    if(ce_ptr->supervision_timeout_handle != NULL)
    {
        OS_DELETE_TIMER( & ce_ptr->supervision_timeout_handle);
    }

    if(ce_ptr->lmp_response_timer_handle != NULL)
    {
        OS_DELETE_TIMER(& ce_ptr->lmp_response_timer_handle);
    }

#ifdef COMPILE_AFH_HOP_KERNEL
    if(ce_ptr->afh_instant_timer_handle != NULL)
    {
        OS_DELETE_TIMER( & ce_ptr->afh_instant_timer_handle);
    }
#endif

#ifdef SECURE_CONN_PING_EN
    if(ce_ptr->en_ping_req_timer != NULL)
    {
        OS_DELETE_TIMER( & ce_ptr->en_ping_req_timer);
    }
    if(ce_ptr->send_max_auth_timeout_timer != NULL)
    {
        OS_DELETE_TIMER( & ce_ptr->send_max_auth_timeout_timer);
    }
#endif
    bz_auth_reset_auth_timers(ce_index);

    return;
}

#ifdef _CCH_LPS_
void lmp_update_lps_para(void)
{

#ifndef _REDUCE_LPS_AFTER_RTL8703B_
    if( g_efuse_lps_setting_4.lps_use_intr )
    {
        UINT16 temp_reg;
        UINT16 temp_reg_min = 0xFFFF;


        if( lmp_self_device_data.scan_enable&0x01)
        {
            temp_reg = (lmp_self_device_data.inquiry_scan_window<<1) -2;

            if( lmp_self_device_data.inquiry_scan_interval > temp_reg )
            {
                temp_reg_min = lmp_self_device_data.inquiry_scan_interval - temp_reg;
            }
        }


        if( lmp_self_device_data.scan_enable&0x02)
        {
            temp_reg = (lmp_self_device_data.page_scan_window<<1) -2;

            if( lmp_self_device_data.page_scan_interval > temp_reg )
            {
                temp_reg = lmp_self_device_data.page_scan_interval - temp_reg;
                if( temp_reg_min > temp_reg)
                {
                    temp_reg_min = temp_reg;
                }
            }
        }

        lmp_self_device_data.scan_interval_min = temp_reg_min;


    }else
#endif
    {
        sleep_mode_param.lps_period_state_bitmap = 0;

#ifdef _CCH_RTL8723A_B_CUT
// _CCH_ECO_LPS_
        if(lmp_self_device_data.scan_enable & 0x01)
        {// Reduse Inquiry Scan Window (for Start Inquiry Scan Immediately)
            sleep_mode_param.lps_period_state_bitmap |= BIT1;

            if( lmp_self_device_data.interlaced_inq_scan )
            {
                sleep_mode_param.lps_period_state_num[LPS_PERIOD_INQ_SCAN]
                    = ((lmp_self_device_data.inquiry_scan_window) >>3) + 1;
            }else
            {
                sleep_mode_param.lps_period_state_num[LPS_PERIOD_INQ_SCAN]
                    = ((lmp_self_device_data.inquiry_scan_window) >>4) + 1;
            }

            if( sleep_mode_param.lps_period_state_num[LPS_PERIOD_INQ_SCAN] ==  0)
            {
                sleep_mode_param.lps_period_state_num[LPS_PERIOD_INQ_SCAN] = 1;
            }

            sleep_mode_param.lps_period_state_interval[LPS_PERIOD_INQ_SCAN]
	    		= lmp_self_device_data.inquiry_scan_interval;
        }else
        {
            sleep_mode_param.lps_period_state_bitmap &= ~BIT1;
        }

        if(lmp_self_device_data.scan_enable & 0x02)
        {// Reduse Page Scan Window (for Start Page Scan Immediately)
            sleep_mode_param.lps_period_state_bitmap |= BIT2;

            if( lmp_self_device_data.interlaced_page_scan )
            {
                sleep_mode_param.lps_period_state_num[LPS_PERIOD_PAG_SCAN]
                    = ((lmp_self_device_data.page_scan_window) >>3) + 1;
            }else
            {
                sleep_mode_param.lps_period_state_num[LPS_PERIOD_PAG_SCAN]
                    = ((lmp_self_device_data.page_scan_window) >>4) + 1;
            }

            sleep_mode_param.lps_period_state_interval[LPS_PERIOD_PAG_SCAN]
	    		= lmp_self_device_data.page_scan_interval;
        }else
        {
            sleep_mode_param.lps_period_state_bitmap &= ~BIT2;
        }

#ifdef LE_MODE_EN
        if (IS_BT40)
        {
            if (ll_manager.adv_unit.enable)
            {
                sleep_mode_param.lps_period_state_bitmap |= BIT3;
#ifdef _MODI_LPS_AFTER_RTL8821B_TC_
                UINT16 adv_h2h_period;
                // set adv_h2h_period (unit: 625us), adv_chm, filter policy, packet type
                adv_h2h_period = ll_fw_compute_advertiser_h2h_min_duration();
                adv_h2h_period = LL_DRIVER_TRANSLATE_625US_UNIT(adv_h2h_period);
                adv_h2h_period = ((adv_h2h_period*3)>>3) + 1;
                sleep_mode_param.lps_period_state_num[LPS_PERIOD_LE_ADV] = adv_h2h_period;
#else
                sleep_mode_param.lps_period_state_num[LPS_PERIOD_LE_ADV]
    			    = 4;
#endif
                sleep_mode_param.lps_period_state_interval[LPS_PERIOD_LE_ADV]
    	    		= ll_manager.adv_unit.interval;

#ifdef _MODI_LPS_AFTER_RTL8703B_
                UINT8 ch_count;
                sleep_mode_param.le_adv_num = 0;
                for(ch_count=0; ch_count<3; ch_count++)
                {
                    if(ll_manager.adv_unit.ch_map&(1<<ch_count))
                        sleep_mode_param.le_adv_num ++;
                }
#endif

            }
            else
            {
                sleep_mode_param.lps_period_state_bitmap &= ~BIT3;
            }


            if (ll_manager.scan_unit.enable)
            {
                sleep_mode_param.lps_period_state_bitmap |= BIT4;
                sleep_mode_param.lps_period_state_num[LPS_PERIOD_LE_SCAN]
        			= (ll_manager.scan_unit.window >>4) + 1;

                sleep_mode_param.lps_period_state_interval[LPS_PERIOD_LE_SCAN]
    		    	= ll_manager.scan_unit.interval;
            }
            else
            {
                sleep_mode_param.lps_period_state_bitmap &= ~BIT4;
            }

#ifdef CONFIG_TV_POWERON_LPS
            if (tv_poweron.scan.enable)
            {
                sleep_mode_param.lps_period_state_bitmap |= BIT6;
                sleep_mode_param.lps_period_state_num[LPS_PERIOD_RSVD1] =
                        (tv_poweron.scan.window >> 4) + 1;
                sleep_mode_param.lps_period_state_interval[LPS_PERIOD_RSVD1] =
                        tv_poweron.scan.interval;
            }
            else
            {
                sleep_mode_param.lps_period_state_bitmap &= ~BIT6;
            }
#endif
        }
#endif

        UINT8 state_ind;
        UINT16 interval_temp = 0xffff;
        UINT16 state_sum = 0;

        for( state_ind=1; state_ind<LPS_PERIOD_STATE_NUM; state_ind++ )
        {
            if( (sleep_mode_param.lps_period_state_bitmap  >>state_ind) & 0x01 )
            {
                state_sum += sleep_mode_param.lps_period_state_num[state_ind];
                if( interval_temp > sleep_mode_param.lps_period_state_interval[state_ind])
                {
                    interval_temp = sleep_mode_param.lps_period_state_interval[state_ind];
                }
            }
        }

        sleep_mode_param.lps_period_interval = interval_temp;

#ifdef _MODI_LPS_AFTER_RTL8821B_TC_
        if(g_efuse_lps_setting_2.adjust_lps_scan_interval)
        {
    		if(sleep_mode_param.lps_period_interval > (state_sum<<4))
            {
                sleep_mode_param.lps_period_interval = sleep_mode_param.lps_period_interval - (state_sum<<4);
            }
        }
#endif

#endif

    }

#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8723_RCP_
// RCP for change LPS Parameter or add some lost initial value

    if (rcp_lmp_update_lps_para != NULL)
    {
        rcp_lmp_update_lps_para();

    }
#endif
#endif


}
#endif

/**
 * Initialize the self device attributes (lmp_self_device_data).
 *
 * \param None.
 *
 * \return None.
 */
void lmp_init_device_attributes(void)
{
    UCHAR index ;

    /*
     * Initializing self device data structure.
     */
    memset((void *)(&lmp_self_device_data), 0, sizeof(lmp_self_device_data));
#ifdef _DAPE_UNIFY_VERSION_FEATURE_UINT
    if (IS_BT50)
    {
        lmp_self_device_data.lmp_version = BT_FW_LMP_VERSION_BT50;            
        lmp_self_device_data.hci_version = BT_FW_HCI_VERSION_BT50;
    }
    if (IS_BT42)
    {
        lmp_self_device_data.lmp_version = BT_FW_LMP_VERSION_BT42;            
        lmp_self_device_data.hci_version = BT_FW_HCI_VERSION_BT42;
    }
    else if (IS_BT41)
    {
        lmp_self_device_data.lmp_version = BT_FW_LMP_VERSION_BT41;            
        lmp_self_device_data.hci_version = BT_FW_HCI_VERSION_BT41;
    }
    else if (IS_BT40)
    {
        lmp_self_device_data.lmp_version = BT_FW_LMP_VERSION_BT40;            
        lmp_self_device_data.hci_version = BT_FW_HCI_VERSION_BT40;
    }
    else if (IS_BT30)
    {
        lmp_self_device_data.lmp_version = BT_FW_LMP_VERSION_BT30;            
        lmp_self_device_data.hci_version = BT_FW_HCI_VERSION_BT30;
    }
    else
    {
        lmp_self_device_data.lmp_version = BT_FW_LMP_VERSION_BT21_PLUS_EDR;            
        lmp_self_device_data.hci_version = BT_FW_LMP_VERSION_BT21_PLUS_EDR;
    }
#endif
    lmp_self_device_data.class_of_device = otp_str_data.bt_class_of_device;

    lmp_self_device_data.opt_page_scan_flag = FALSE;

    /* memor copy security key */
    memcpy(lmp_self_device_data.security_key, otp_str_data.bt_security_key, 16);

    /* Mandatory Page Scan Mode */
    /* Both Inquiry and Page Scan Disabled */
    lmp_self_device_data.scan_enable = 0x00;
    lmp_self_device_data.page_scan_mode = 0x00;
    lmp_self_device_data.page_scan_period_mode = 0x02;
    lmp_self_device_data.page_scan_repetition_mode = 0x01;

    lmp_self_device_data.page_scan_interval = otp_str_data.bt_page_scan_interval;

    lmp_self_device_data.page_scan_window = otp_str_data.bt_page_scan_window;

    lmp_self_device_data.inquiry_scan_interval = otp_str_data.bt_inquiry_scan_interval;

    lmp_self_device_data.inquiry_scan_window = otp_str_data.bt_inquiry_scan_window;

    lmp_self_device_data.conn_accept_timeout = otp_str_data.bt_conn_accept_timeout;
    lmp_self_device_data.page_timeout = otp_str_data.bt_page_timeout;

    lmp_self_device_data.voice_setting = defined_voice_setting; //VOICE_SETTINGS_CVSD_MASK;
    lmp_self_device_data.lmp_air_mode =
        lmp_convert_air_mode((UCHAR)(defined_voice_setting&AIR_MODE_MASK),
                             HCI_LAYER, LMP_LAYER);//LMP_CVSD;

    lmp_self_device_data.input_data_format = INPUT_DATA_FORMAT_MASK;
    lmp_self_device_data.device_status = LMP_IDLE;

    lmp_self_device_data.default_link_policy_settings = (UINT16)
            ((lmp_feature_data.feat_page0[0] & POLICY_SETTINGS_MASK0)>>5);
    lmp_self_device_data.default_link_policy_settings = (UINT16)
            (lmp_self_device_data.default_link_policy_settings |
             ((lmp_feature_data.feat_page0[1] & POLICY_SETTINGS_MASK1)<<3));

    /* No DIAC scan by default.. by default GIAC scan is allowed.*/
    lmp_self_device_data.diac_lap = FALSE ;

    /* Default value is null terminated string */

    for(index = 0; ((index <LMP_MAX_NAME_LENGTH) && (otp_str_data.bt_local_name[index]!='\0')) ; index++)
    {
        lmp_self_device_data.local_name[index] = otp_str_data.bt_local_name[index];
    }

    if(index != LMP_MAX_NAME_LENGTH)
    {
        lmp_self_device_data.local_name[index] = 0x0;
    }
    lmp_self_device_data.local_name_len = index ;

    /* Default event_mask is 0x 00 00 1F FF FF FF FF FF */
    for(index = 0; index < 5; index++)
    {
        lmp_self_device_data.event_mask[index] = 0xFF;
    }
    lmp_self_device_data.event_mask[5] = 0x1F;
    lmp_self_device_data.event_mask[6] = 0x00;
    lmp_self_device_data.event_mask[7] = 0x00;

#ifdef TEST_MODE
    /* Init Test Mode */
    lmp_self_device_data.test_mode = HCI_NO_LOOPBACK_MODE;
    lmp_self_device_data.stored_test_mode_state = HCI_NO_LOOPBACK_MODE;
    lmp_self_device_data.host_enable_test_mode = FALSE;
#endif /* TEST_MODE */

    lmp_initialize_event_filters();

    /* Maximum number of commands the host can send.
     * Keeping one extra command buffer for number of completed packets
     * command from the host. Even in the worst case, number of completed packets
     * command can not be rejected.*/

    lmp_self_device_data.num_hci_command_packets = (BT_FW_CMD_BUFFERS - 1);
    lmp_self_device_data.sco_pkt_type = 0x00 ;

#ifdef ENABLE_SCO
    lmp_self_device_data.total_no_of_sco_conn = 0x00 ;
    lmp_self_device_data.adding_new_sco_conn = 0x00 ;
#endif
#ifdef COMPILE_ESCO
#ifdef _DAPE_TEST_SEND_MAX_SLOT_BEFORE_ESCO_CREATED
	lmp_self_device_data.adding_new_esco_conn = 0x00;
#endif
#endif
    lmp_self_device_data.number_of_acl_conn = 0x00 ;
    lmp_self_device_data.number_of_hlc = 0x00 ;

    lmp_self_device_data.num_broadcast_retran = LC_MAX_RETX_LIMIT ;

    lmp_self_device_data.sco_flow_control_enable = 0x00 ;
    lmp_self_device_data.num_supported_iac = 1 ;

    lmp_self_device_data.flow_control_hc_to_host = 0x00 ;
#ifdef COMPILE_PARK_MODE
    lmp_self_device_data.number_of_parked_dev = 0x00;
#endif /* COMPILE_PARK_MODE */

    for(index = 0; index < LMP_MAX_IAC_LAPS; index++)
    {
        lmp_self_device_data.iac_lap[index][0] = 0x33;
        lmp_self_device_data.iac_lap[index][1] = 0x8b;
        lmp_self_device_data.iac_lap[index][2] = 0x9e;
    }

#ifdef BROADCAST_DATA
    lmp_self_device_data.asb_num_of_completed_packets = 0;
    lmp_self_device_data.psb_num_of_completed_packets = 0;
    lmp_self_device_data.bc_conn_handle = 0xFFFF;
    lmp_self_device_data.park_bc_conn_handle = 0xFFFF;
#endif

    lmp_self_device_data.lc_dsniffs = 0;
#ifdef COMPILE_DYNAMIC_POLLING
    lmp_self_device_data.enable_dynamic_polling = TRUE;
    lmp_self_device_data.tpoll_inactive_transition = 8;
    lmp_self_device_data.tpoll_step = 4;
    lmp_self_device_data.min_tpoll = 4;
#endif

#ifdef COMPILE_HOLD_MODE
    lmp_self_device_data.hold_mode_activity = HCI_HOLD_MODE_ACTIVITY_DEFAULT;
    lmp_self_device_data.number_of_connections_in_hold_mode = 0;
#endif

    lc_set_lc_cur_device_state(LC_CUR_STATE_IDLE);

    INIT_1_2_SELF_DEVICE_DATA;
    INIT_2_1_SELF_DEVICE_DATA;
#ifdef VER_3_0
    if (IS_BT30)
    {
        INIT_3_0_SELF_DEVICE_DATA;
    }
#endif
    lmp_self_device_data.num_of_sniff_connections = 0;
    lmp_self_device_data.next_sniff_instant = 0xFFFFFFFF;
    lmp_self_device_data.next_next_sniff_instant = 0xFFFFFFFF;

#ifdef _USE_SNIFF_NO_ACL_COUNT_REGISTER
#ifdef COMPILE_SNIFF_MODE
    lmp_self_device_data.no_acl_reduced_flag = FALSE;
    lmp_self_device_data.no_acl_ce_index = 0xFF;
#endif
#endif

#ifdef _CCH_SLOT_OFFSET_

    for(index=0; index<GLOBAL_SLOT_USE_NUM; index++)
    {
    	lmp_self_device_data.global_slot_use_ce_index[index]=0xff;
    	lmp_self_device_data.global_slot_use_acl_ce_index[index]=0xff;
    	lmp_self_device_data.global_slot_use_interval[index]=0xff;
    	lmp_self_device_data.global_slot_use_slot_offset[index]=0xff;
    	lmp_self_device_data.global_slot_use_remote_slot[index]=0xff;
    	lmp_self_device_data.global_slot_use_slot_num[index]=0xff;
    	lmp_self_device_data.global_slot_interval[index]=0xff;
    }

#endif


#ifdef _CCH_LPS_
    lmp_update_lps_para();
#endif
#ifdef _SUPPORT_VER_4_1_
#ifdef _SUPPORT_PCA_ADJUST
    memset(&bt_pca_manager, 0, sizeof(BT_PCA_MANAGER_S));
    bt_pca_manager.clk_adj_id = 0xFF;
    bt_pca_manager.clk_req_retry_th = 5;
    bt_pca_manager.clk_offset_for_update = 128;
#endif
#endif
    //RT_BT_LOG(GREEN, LMP_DATAMGR_791, 1, lmp_self_device_data.voice_setting);

    return;
}

/**
 * Initialize the event filters.
 *
 * \param None.
 *
 * \return None.
 */
void lmp_initialize_event_filters(void)
{
    UCHAR index ;

    /* By default after power-on or reset, no filters are set, and the
     * auto accept flag is off.(An incomming connections are not automatically
     * accepted.
     */
    for (index=0; index < LMP_MAX_EVENT_FILTERS; index++)
    {
        lmp_self_device_data.event_filter[index].filter_type = 0x00;
        lmp_self_device_data.event_filter[index].filter_condition_type = 0x00;
        lmp_self_device_data.event_filter[index].class_of_device = 0x000000;
        lmp_self_device_data.event_filter[index].class_of_device_mask = 0x00;
        memset(lmp_self_device_data.event_filter[index].bd_addr, 0,
               LMP_BD_ADDR_SIZE);
        /* Auto accept the connection (Auto accept is OFF) */
        lmp_self_device_data.event_filter[index].auto_accept_flag = 0x00;
    }

    lmp_event_filters_written = 0;
}

/**
 * Initialize the inquiry result event filters.
 *
 * \param None.
 *
 * \return None.
 */
void lmp_initialize_inquiry_result_event_filters(void)
{
    UCHAR index ;

    for (index=0; index < LMP_MAX_EVENT_FILTERS; index++)
    {
        if(lmp_self_device_data.event_filter[index].filter_type ==
                INQUIRY_RESULT_FILTER)
        {
            lmp_self_device_data.event_filter[index].filter_type = 0x00;
            lmp_self_device_data.event_filter[index].filter_condition_type=0x00;
            lmp_self_device_data.event_filter[index].class_of_device = 0x000000;
            lmp_self_device_data.event_filter[index].class_of_device_mask=0x00;
            memset(lmp_self_device_data.event_filter[index].bd_addr, 0,
                   LMP_BD_ADDR_SIZE);
        }
    }
}

/**
 * Initializes the connection entity elements except timers.
 *
 * \param ce_index ACL Connection entity index.
 *
 * \return None.
 */
void lmp_init_connection_entity_on_power_on(UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY *ce_ptr;
    PKT_ALLOWED *pkt_allow;

    ce_ptr = &lmp_connection_entity[ce_index];
    pkt_allow = &ce_ptr->pkts_allowed;

    memset((void *)ce_ptr, 0, sizeof(LMP_CONNECTION_ENTITY));

    /* Check out the initialization values */
    lmp_set_ce_status(ce_index, LMP_STANDBY);
    ce_ptr->conn_accept_timer_handle = NULL;
    ce_ptr->detach_connection_timer_handle = NULL;
    ce_ptr->supervision_timeout_handle = NULL;
    ce_ptr->afh_instant_timer_handle = NULL;
#ifdef SECURE_CONN_PING_EN
    ce_ptr->en_ping_req_timer = NULL;
    ce_ptr->send_max_auth_timeout_timer = NULL;
    /* 0x0BB8 => 30 sec. */
    ce_ptr->max_auth_interval = 0xBB8;
#ifdef _TMP_SET_SHORT_AUTH_INTERVAL
    ce_ptr->max_auth_interval = 0x0100;
#endif
#endif


    ce_ptr->afh_ce_status = LMP_AFH_DISABLED;
    ce_ptr->temp_ce_status = LMP_STANDBY;
    ce_ptr->entity_status = UNASSIGNED;
    ce_ptr->lmp_expected_pdu_opcode = BIT_MASK_NO_PDU;

    /* Initialize remote device features */
    ce_ptr->requested_ext_page = 0;

    pkt_allow->status = 0;

    ce_ptr->am_addr = 0xFF;
    ce_ptr->class_of_device = 0xFFFFFF;
    ce_ptr->remote_dev_role = 0xFF ;
    ce_ptr->hc_num_of_completed_packets = 0x0000;
    ce_ptr->optional_page_scheme = MANDATORY_PAGING_SCHEME;
    ce_ptr->optional_page_setting = PAGING_SCHEME_SETTING_R0;

    /*Connection handle is fixed to a connection entity */
    ce_ptr->connection_type.connection_handle = (UINT16)(ce_index + 1);

    ce_ptr->connection_type.packet_type = 0xCC18;
    ce_ptr->connection_type.link_type = INVALID_LINK_TYPE;

    ce_ptr->remote_max_drift = 0xff;

#ifdef COMPILE_HOLD_MODE
    ce_ptr->hold_mode_max_interval = otp_str_data.bt_hold_max_interval;
    ce_ptr->hold_mode_min_interval = otp_str_data.bt_hold_min_interval;

    ce_ptr->hold_mode_interval = 0X00;
    ce_ptr->hold_instant = 0X00;

    ce_ptr->hold_max_interval_negotiated = 0X00;
    ce_ptr->hold_min_interval_negotiated = 0X00;
    ce_ptr->hold_mode_accepted_flag = 0X00;
    ce_ptr->hold_neg_max_interval = 0x00;

#endif /* COMPILE_HOLD_MODE */

#ifdef COMPILE_SNIFF_MODE
    ce_ptr->in_sniff_mode = FALSE;

    ce_ptr->sniff_max_interval = otp_str_data.bt_sniff_max_interval;
    ce_ptr->sniff_min_interval = otp_str_data.bt_sniff_min_interval;
    ce_ptr->sniff_interval = 0X00;
    ce_ptr->temp_sniff_interval = 0xFFFF;

    ce_ptr->sniff_slot_offset = 0X00;
    ce_ptr->sniff_attempt = 0X00;
    ce_ptr->sniff_attempt_for_afh_count = 0X00;
    ce_ptr->sniff_timeout = 0X00;


#ifdef _CCH_SNIFF_NEG_TIMEOUT_
    ce_ptr->sniff_neg_count = 0x00;
#endif

    ce_ptr->sniff_sw_timer_handle = NULL;
#ifdef POWER_SAVE_FEATURE
    ce_ptr->sniff_last_instant = 0x00;
#endif
    ce_ptr->sniff_max_interval_negotiated = 0X00;
    ce_ptr->sniff_min_interval_negotiated = 0X00;

    ce_ptr->next_instant_in_nat_clk = 0xFFFFFFFF;
    ce_ptr->next_next_instant_in_nat_clk = 0xFFFFFFFF;

#endif /* COMPILE_SNIFF_MODE */

#ifdef COMPILE_PARK_MODE
    ce_ptr->beacon_max_interval = 0X00;
    ce_ptr->beacon_min_interval = 0X00;

    ce_ptr->park_mode_negotiated = 0X00;
    ce_ptr->park_mode_timer_handle = NULL;

    ce_ptr->Dbeacon = LMP_DBEACON ;
    ce_ptr->Tbeacon = LMP_TBEACON ;
    ce_ptr->Nbeacon  = LMP_NBEACON ;
    ce_ptr->Delta_beacon = LMP_DELTA_BEACON;

    ce_ptr->unpark_req_flag = LMP_UNPARK_IDLE;

    ce_ptr->hci_unpark_req_flag = HCI_UNPARK_IDLE;

    memset(&ce_ptr->lmp_pdu_pkt,0, sizeof(LMP_PDU_PKT));

    ce_ptr->auto_unpark_cnt = 0xFFFF;
    ce_ptr->supto_auto_repark = FALSE;
    ce_ptr->bc_scan_window = FALSE;
#endif /* COMPILE_PARK_MODE */

    ce_ptr->stored_link_supervision_timeout = 0;
    /* Link supervision timeout*/

    ce_ptr->link_supervision_timeout = otp_str_data.bt_supervision_timeout;
    ce_ptr->supervision_timeout_flag = FALSE ;
    /* Link Policy settings. */

    ce_ptr->link_policy_settings = otp_str_data.bt_def_link_policy_settings;

    /*
     * Default values for QoS_setup.
     * Ref : BT-Specification-1.1 L2CAP Section, page 300
     * for default values and the meaning.
     */

    /* This field indicates the level of service required.
     * Best effort is the default value,
     */
    ce_ptr->service_type = 0x01;

    /*
     * The value of this field represents the rate at which traffic credits are
     * granted in bytes per second.The value 0x00000000 indicates no token rate
     * is specified.This is the default value and implies indifference to token
     * rate. The value 0xFFFFFFFF represents a wild card matching the maximum
     * token rate available.
     */
    ce_ptr->token_rate  =  0x00000000;

    ce_ptr->token_bucket_rate = 0x00000000;

    /*
     * The value of this field, expressed in bytes per second, limits how fast
     * packets may be sent back-to-back from applications.The value of
     * 0x00000000 states that the maximum bandwidth is unknown, which is the
     * default value.
     */
    ce_ptr->peak_bandwidth = 0x00000000;

    /*
     * The value of this field represents the maximum acceptable delay between
     * transmission of a bit by the sender and its initial transmission over
     * the air, expressed in microseconds. The precise interpretation of this
     * number depends on the level of guarantee specified in the Class of
     * Service. The value 0xFFFFFFFF represents a do not care and is the
     * default value.
     */
    ce_ptr->latency  =  0xffffffff;

    /*
     * The value of this field is the difference, in microseconds, between the
     * maximum and minimum possible delay that a packet will experience. This
     * value is used by applications to determine the amount of buffer space
     * needed at the receiving side in order to restore the original data
     * transmission pattern.The value 0xFFFFFFFF represents a do not care and
     * is the default value.
     */
    ce_ptr->delay_variation = 0xffffffff;

    ce_ptr->qos_tpoll = otp_str_data.bt_t_poll;

    ce_ptr->qos_setup_tpoll = HCI_INVALID_TPOLL;
    ce_ptr->flow_tx_tpoll = HCI_INVALID_TPOLL;
    ce_ptr->flow_rx_tpoll = HCI_INVALID_TPOLL;

#ifdef COMPILE_PARK_MODE
    /* Parked mode address */
    ce_ptr->pm_addr = 0XFF;
    ce_ptr->ar_addr = 0XFF ;
#endif /* COMPILE_PARK_MODE */

    /* Store Tpoll value in milli seconds. Default value is 40 slots */
    ce_ptr->Tpoll = otp_str_data.bt_t_poll;

#ifdef COMPILE_DYNAMIC_POLLING
    ce_ptr->current_tpoll = 4; /* Replace this by define */
    ce_ptr->no_inactive_tpolls = 0;
#endif

    ce_ptr->num_of_BC = LC_MAX_RETX_LIMIT ;

    ce_ptr->setup_complete_status = BEGIN_SETUP ;
    ce_ptr->clock_offset = 0xFFFF;

#ifdef TEST_MODE
    ce_ptr->test_mode_info.test_state = TEST_INITIALIZED;

    ce_ptr->test_mode_info.tc_params.packet_type_in_pdu = BB_NULL;
    ce_ptr->test_mode_info.tc_params.test_scenario = TEST_MODE_EXIT_TEST_MODE;
    ce_ptr->test_mode_info.tc_params.hopping_mode = TEST_MODE_HOP_79;
#endif /* TEST_MODE */

    ce_ptr->lmp_response_timer_handle = NULL;

    ce_ptr->clock_offset = 0 ;
    ce_ptr->slot_offset = 0 ;
    ce_ptr->rem_manuf_name = 0 ;
    ce_ptr->lmp_subversion = 0 ;

#ifdef COMPILE_ROLE_SWITCH
    ce_ptr->mss_completion_status = 0;
#endif
    ce_ptr->is_enc_paused = FALSE;
    ce_ptr->allow_role_switch = FALSE ;
    ce_ptr->role_switch_accepted_flag = FALSE ;
    ce_ptr->disconnect_reason = 0 ;
    ce_ptr->name_req_name_offset = 0 ;
#ifdef _REMOTE_NAME_RES_WRONG
    ce_ptr->name_length_offset_zero = 0;
#endif
    ce_ptr->sent_pdu = 0 ;

    memset(ce_ptr->device_name, 0 ,LMP_MAX_NAME_LENGTH);
    ce_ptr->lmp_version = 0 ;

    ce_ptr->tx_max_slot = 1 ;
    ce_ptr->temp_tx_max_slot = 1 ;
    ce_ptr->old_tx_max_slot = 1 ;
    ce_ptr->rx_max_slot = 1 ;
    ce_ptr->temp_rx_max_slot = 1 ;
    ce_ptr->old_rx_max_slot = 1 ;

    ce_ptr->page_scan_repetition_mode = 0 ;
    ce_ptr->page_scan_period_mode = 0 ;
    ce_ptr->page_scan_mode = 0 ;
    ce_ptr->pdu_response_timer_running = FALSE ;

#ifdef COMPILE_HOLD_MODE
    ce_ptr->hold_mode_interval_negotiated = FALSE ;
#endif /* COMPILE_HOLD_MODE */

#ifdef COMPILE_PARK_MODE
    ce_ptr->NB_sleep = 0 ;
    ce_ptr->DB_sleep = 0 ;
    ce_ptr->D_access = 0 ;
    ce_ptr->T_access = 0 ;
    ce_ptr->N_acc_slots = 0 ;
    ce_ptr->N_poll = 0 ;
    ce_ptr->M_access = 0 ;
    ce_ptr->access_scheme = 0 ;
    ce_ptr->timing_control_flag = 0 ;
#endif /* COMPILE_PARK_MODE */

#ifdef COMPILE_CQDDR
    ce_ptr->sent_autorate_pdu = FALSE;
    ce_ptr->received_auto_rate_pdu = FALSE;
    ce_ptr->preferred_cqddr_pkt_type = ALL_EDR_PACKETS;
#endif /* COMPILE_CQDDR */

    ce_ptr->flush_timeout = 0 ;
    ce_ptr->is_last_sent_zero_l_l2cap = 0x1;

    ce_ptr->aclq_resch_flag = RESCHEDULE_FLAG_NONE;
    ce_ptr->aclq_start_flush_time = (UINT32)0x0;

    ce_ptr->flush_running = FALSE;
    ce_ptr->failed_contact_counter = 0 ;
    ce_ptr->transmit_power_level = 0 ;
    ce_ptr->link_quality = 0 ;
    ce_ptr->detach_timer_state = IDLE_STATE ;

    ce_ptr->low_power_disconnect_state= IDLE_STATE ;

#if defined(POWER_CONTROL) && defined(COMPILE_RSSI_REPORTING)
    ce_ptr->rssi = 0 ;
    ce_ptr->power_ctrl_resp = LMP_MID_POWER;
#ifdef OPTIONAL_PAGING
    ce_ptr->rssi_meas_flag = TRUE ;
#endif
#endif /* POWER_CONTROL */

#ifdef POWER_CONTROL
    ce_ptr->dec_pow_pdu_drop_flag = FALSE;
    ce_ptr->inc_pow_pdu_drop_flag = FALSE;
    ce_ptr->power_ctrl_pdu_sent_flag = FALSE;
#endif /* POWER_CONTROL */

    ce_ptr->out_standing_data_to_host = 0;

    ce_ptr->hci_cmd_bits = 0;

    ce_ptr->host_con_req_rx_flag = FALSE;
    ce_ptr->paging_completed_flag = FALSE;

#ifdef COMPILE_SNIFF_MODE
    ce_ptr->cont_poll_count = 0;
#endif

#ifdef COMPILE_PARK_MODE
    ce_ptr->pm_addr = 0;
    ce_ptr->ar_addr = 0;
#endif

#ifndef COMPILE_SINGLE_SLOT_PACKETS_ONLY
    ce_ptr->last_max_slot_req_sent = LMP_LAST_MAX_SLOT_REQ_SENT_INVALID;
    ce_ptr->last_accepted_max_slot = LMP_LAST_ACCEPTED_MAX_SLOT_INVALID;
    ce_ptr->last_recd_max_slot = LMP_MAX_SLOT_INVALID;
    ce_ptr->last_max_slot_sent = LMP_MAX_SLOT_INVALID;
#endif /* COMPILE_SINGLE_SLOT_PACKETS_ONLY */

    /* Reset the number of samples here. */
    ce_ptr->rssi_samples_count = 0;
    ce_ptr->rssi = 0;
    ce_ptr->rssi_value_accumulate= 0;

    INIT_1_2_CONNECTION_ENTITY(ce_index);
    INIT_2_1_CONNECTION_ENTITY(ce_index);
#ifdef VER_3_0
    if (IS_BT30)
    {
        INIT_3_0_CONNECTION_ENTITY(ce_index);
    }
#endif
    bz_auth_init_linkparams(ce_index);
    lmp_edtm_init_linkparams(ce_index);
#ifndef _ENABLE_COMPILE_NESTED_PAUSE_RESUME_
    ce_ptr->pause_data_transfer = FALSE;
#endif
    ce_ptr->no_of_sco_connections = 0;
    ce_ptr->no_of_esco_connections = 0;
    ce_ptr->sco_status_byte = 0;
    ce_ptr->codec_status_byte = 0;

#ifdef _INI_SECURE_CONN_
    BZ_AUTH_LINK_PARAMS* auth;
    auth = ce_ptr->auth;

#ifndef _INI_SECURE_CONN_8_
    auth->secure_conn_enabled = 0;
    auth->len_prime = 6;
#else
    auth->secure_conn_enabled = 1;
    auth->len_prime = 8;
#endif

#ifdef _CCH_SC_ECDH_P256_PAUSE_ENC
    auth->sc_use_enc_rand = 0;
#endif
#ifdef _CCH_SC_TEST_20130201_ESCO_INI_FLAG
    auth->not_first_esco = 0;
#endif
#endif

}

/**
 * Initialize a single connection entity database entry.
 *
 * \param ce_index Connection entity index of the ACL connection.
 *
 * \return None.
 */
void lmp_init_connection_entity(UINT16 ce_index)
{
    /* Reset the timers of this connection entity...*/
    lmp_reset_timers(ce_index);

    lmp_init_connection_entity_on_power_on(ce_index);

    return;
}

/**
 * Extract the FHS packet's contents into the inquiry result table.
 *
 * \param lmp_fhs_pkt_recd The pointer to the FHS packet.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_extract_fhs_packet_to_inquiry_table(
    LMP_FHS_PKT_RECD *lmp_fhs_pkt_recd)
{
    UCHAR index ;
    UCHAR temp_byte ;
    UCHAR temp_bd_addr[LMP_BD_ADDR_SIZE] ;
    UINT32 slave_clock = 0;
    UINT16 clock_offset = 0;
    UINT32 master_clock =0;

    LMP_INQUIRY_RESULT_DATA *lcl_ptr;

    lmp_extract_bd_addr_from_fhs_packet(&lmp_fhs_pkt_recd->fhs_pkt[0],
                                        &temp_bd_addr[0]);

    /* Check the BD_ADDR already exists */
    if (lmp_self_device_data.inquiry_mode != HCI_INQ_RESULT_EVENT_WITH_RSSI
        && lmp_self_device_data.inquiry_mode != HCI_INQ_RESULT_EVENT_WITH_EIR)
    {
        index = 0;
        while(index < LMP_MAX_INQUIRY_RESULT_DATA)
        {
            if(memcmp(lmp_inquiry_result_data[index].bd_addr,
                      temp_bd_addr,
                      LMP_BD_ADDR_SIZE) == 0)
            {
                /* Already BD_ADDR exists */
                return BT_FW_ERROR;
            }
            index++;
        }
    }

    lcl_ptr = &lmp_inquiry_result_data[lmp_inquiry_data_index];

    /* Copy the BD address to inquiry table */
    memcpy((UCHAR *)&lcl_ptr->bd_addr[0], temp_bd_addr,  LMP_BD_ADDR_SIZE);

    /* Class of Device */
    lcl_ptr->class_of_device = (lmp_fhs_pkt_recd->fhs_pkt[13] << 16) |
                               (lmp_fhs_pkt_recd->fhs_pkt[12] << 8) |
                               (lmp_fhs_pkt_recd->fhs_pkt[11] << 0);

    /* Extracting the Page Scan Repetition Mode */
    temp_byte = lmp_fhs_pkt_recd->fhs_pkt[7];
    lcl_ptr->page_scan_repetition_mode = (UCHAR)((temp_byte & 0x30) >> 4);

    /* Page Scan Period Mode */
    lcl_ptr->page_scan_period_mode = (UCHAR)((temp_byte & 0xc0)>>6);

    /* Page Scan Mode */
    temp_byte = lmp_fhs_pkt_recd->fhs_pkt[17];
    lcl_ptr->page_scan_mode = (UCHAR)((temp_byte >> 5) & 0x07);
    slave_clock = (lmp_fhs_pkt_recd->fhs_pkt[17]) &0x1F ;
    slave_clock <<= 8 ;
    slave_clock |= (lmp_fhs_pkt_recd->fhs_pkt[16]);
    slave_clock <<= 8 ;
    slave_clock |= (lmp_fhs_pkt_recd->fhs_pkt[15]);
    slave_clock <<= 5 ;
    slave_clock |= (lmp_fhs_pkt_recd->fhs_pkt[14]) >> 3;

    slave_clock = (slave_clock & 0x7FFF);

    /* Derecement by 1, this is for interrupt delay. */
    lmp_fhs_pkt_recd->clk--;
    lmp_fhs_pkt_recd->clk = lmp_fhs_pkt_recd->clk & 0xfffffff;

    master_clock = lmp_fhs_pkt_recd->clk;
    master_clock = ((master_clock >> 2) & 0x7FFF);

    /* Calculating the clock offset */
    clock_offset = (UINT16)((slave_clock - master_clock) & (~0x8000));

    lcl_ptr->clock_offset = clock_offset;

#ifdef COMPILE_RSSI_REPORTING
    lcl_ptr->rssi = lmp_fhs_pkt_recd->rssi;
#endif /* COMPILE_RSSI_REPORTING */

    return BT_FW_SUCCESS;
}

/**
 * Allocate a free connection handle.
 *
 * \param conn_handle The new connection handle returned.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_allocate_conn_handle(UINT16* conn_handle)
{
    UCHAR index ;

    for(index = (UCHAR)(lmp_assigned_ce_index+1);
            index <= (LMP_MAX_CE_DATABASE_ENTRIES+LMP_MAX_BC_CONNECTIONS); index++)
    {
#ifdef COMPILE_ESCO
        if((lmp_ch_to_ce_index_table[index-1].status == UNASSIGNED) &&
                (lmp_esco_ch_to_ce_index_table[index-1].status == UNASSIGNED))
#else
        if(lmp_ch_to_ce_index_table[index-1].status == UNASSIGNED)
#endif
        {
            lmp_ch_to_ce_index_table[index-1].status = ASSIGNED ;
            lmp_assigned_ce_index = index ;
            *conn_handle = index ;
            return BT_FW_SUCCESS ;
        }
    }
    for(index = 1 ; index <= lmp_assigned_ce_index ; index++)
    {
#ifdef COMPILE_ESCO
        if((lmp_ch_to_ce_index_table[index-1].status == UNASSIGNED) &&
                (lmp_esco_ch_to_ce_index_table[index-1].status == UNASSIGNED))
#else
        if(lmp_ch_to_ce_index_table[index-1].status == UNASSIGNED)
#endif
        {
            lmp_ch_to_ce_index_table[index-1].status = ASSIGNED ;
            lmp_assigned_ce_index = index ;
            *conn_handle = index ;
            return BT_FW_SUCCESS ;
        }
    }

    return BT_FW_ERROR ;
}

/**
 * Release the connection handle provided (\a conn_handle).
 *
 * \param conn_handle Connection handle to be released.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_release_conn_handle(UINT16 conn_handle)
{
    if ((conn_handle < 1) ||
        (conn_handle > (LMP_MAX_CE_DATABASE_ENTRIES + LMP_MAX_BC_CONNECTIONS)))
    {
        return BT_FW_ERROR ;
    }

    lmp_ch_to_ce_index_table[conn_handle-1].status = UNASSIGNED ;

    return BT_FW_SUCCESS ;
}

/*
 * Handles the intermediate
 * (LMP_SUPERVISION_TIMER_RESOLUTION) timeouts and if
 * the intermediate timeouts add upto "supervision_timeout" then sends
 * LMP_SUPERVISION_TIMEOUT_SIGNAL so that Supervision Timeout can be handled.
 *
 * It was introduced to avoid restarting the timer in the interrupts.
 */
void __sup_to_pwa(TimerHandle_t timer_handle)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    OS_SIGNAL signal_send;
    UINT16 ce_index;
    UINT16 sup_to;
    UINT16 temp_sup_timeout_var;
    DEF_CRITICAL_SECTION_STORAGE;

    ce_index = (UINT16)((UINT32)pvTimerGetTimerID(timer_handle));
    ce_ptr = &lmp_connection_entity[ce_index];
    sup_to = ce_ptr->link_supervision_timeout;

    MINT_OS_ENTER_CRITICAL();
    lmp_sup_timeout_var[ce_index]++;
    temp_sup_timeout_var = lmp_sup_timeout_var[ce_index];
    MINT_OS_EXIT_CRITICAL();

    if (temp_sup_timeout_var >
            ((sup_to >> LMP_SUPERVISION_TIMER_RESOLUTION_EXPONENT) + 1))
    {
        signal_send.type = LMP_SUPERVISION_TIMEOUT_SIGNAL;
        signal_send.param = (OS_ADDRESS)((UINT32)ce_index);

        if(OS_SEND_SIGNAL_TO_TASK(lmp_task_handle, signal_send)
                != BT_ERROR_OK)
        {
            LMP_ERR(LOG_LEVEL_HIGH, OS_SEND_SIGNAL_TO_TASK_FAILED,0,0);
        }

        MINT_OS_ENTER_CRITICAL();
        lmp_sup_timeout_var[ce_index] = 0;
        MINT_OS_EXIT_CRITICAL();

#ifdef COMPILE_ROLE_SWITCH
        if(ce_ptr->ce_status == LMP_ROLE_SWITCHING)
        {
            RT_BT_LOG(GRAY, LMP_DATAMGR_1572, 0, 0);
        }
#endif

    }
    else
    {
        if (sup_to != 0)
        {
            if(OS_START_TIMER(ce_ptr->supervision_timeout_handle,
              (UINT16)(SLOT_VAL_TO_TIMER_VAL(LMP_SUPERVISION_TIMER_RESOLUTION)))
                    != BT_ERROR_OK)
            {
                LMP_ERR(LOG_LEVEL_LOW,
                        UNABLE_TO_START_SUPERVISION_TIMEOUT_FOR_AM_ADDR,
                        1,ce_ptr->am_addr);
            }

            MINT_OS_ENTER_CRITICAL();
            lmp_sup_var_timeout_var[ce_index] = 0xFF;
            MINT_OS_EXIT_CRITICAL();
        }
    }

    /* Logger will  display intermediate to after 2^MAX_SIZE_OF_INTERMEDIATE_TIMEOUTS_TO_LOG timeouts
    ie; After 2^(MAX_SIZE_OF_INTERMEDIATE_TIMEOUTS_TO_LOG + LMP_SUPERVISION_TIMER_RESOLUTION_EXPONENT) slots */
#define MAX_SIZE_OF_INTERMEDIATE_TIMEOUTS_TO_LOG 2//6

    UINT16 size_of_intermediate_to;
    sup_to = (sup_to >> LMP_SUPERVISION_TIMER_RESOLUTION_EXPONENT);
    size_of_intermediate_to = (1 << MAX_SIZE_OF_INTERMEDIATE_TIMEOUTS_TO_LOG);

    if  (((temp_sup_timeout_var % size_of_intermediate_to) == 0) ||
            (temp_sup_timeout_var == sup_to))
    {
        /* NOTE: Current_intermediate_to can go upto [(sup_to)/
        * size_of_intermediate_to] - which may be equal or less
        * than 2^LMP_SUPERVISION_TIMER_RESOLUTION_EXPONENT.
        */
        LMP_CONNECTION_ENTITY *ce_ptr;
        UINT8 lut_index;
        UINT16 lower_lut_address = 0xffff;
        UINT16 lower_lut_contents = 0xffff;
        UINT16 upper_lut_address = 0xffff;
        UINT16 upper_lut_contents = 0xffff;
        LC_SCHEDULED_PKT  *schd_info;
        UINT8 am_addr;
        UINT8 pid;
        UINT16 val_piconet_info;
        UINT32 clk;

        ce_ptr = &lmp_connection_entity[ce_index];

        am_addr = ce_ptr->am_addr;
        pid = ce_ptr->phy_piconet_id;

        lc_get_clock_in_scatternet(&clk, pid);

        val_piconet_info = BB_read_baseband_register(reg_PICONET_INFO[pid]);

        lut_index = lc_get_lut_index_from_phy_piconet_id(am_addr, pid);

        LC_PICONET_SCHEDULER *piconet_schd;
        piconet_schd = &lc_piconet_scheduler[pid];
        schd_info = &piconet_schd->lc_scheduled_pkt_info[piconet_schd->rptr];

        if(lut_index != 0xff)
        {
            lower_lut_address = lut_ex_table[lut_index].lower_lut_address;
            upper_lut_address = lut_ex_table[lut_index].upper_lut_address;

            lower_lut_contents = BB_read_baseband_register(lower_lut_address);
            upper_lut_contents = BB_read_baseband_register(upper_lut_address);
        }

#ifdef _ROM_CODE_PATCHED_
        if (rcp_sup_to_pwa_func != NULL)
        {
            if (rcp_sup_to_pwa_func((void *)&ce_index))
            {
                return;
            }
        }
#endif
#ifdef _DAPE_TEST_FOR_HID_SCO
        if ((lc_sco_pause_status & (BIT0 << pid)) != 0 )
        {
            if (ce_ptr->remote_dev_role == MASTER)
            {
                lc_sco_pause_status &= ~(BIT0 << pid);
    	    }
            if ((lc_sco_pause_status == 0) &&
                   (ce_ptr->remote_dev_role == MASTER))
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
            if ((lc_le_pause_status & (BIT0 << pid)) != 0 )
            {
                lc_le_pause_status &= ~(BIT0 << pid);
                if (lc_le_pause_status == 0)
                {
                    ll_driver_block_legacy_slot_for_le(g_block_legacy_for_le_slot_num);
                }
            }
        }
#endif
#endif
#ifdef _DAPE_EN_8821_MP_MODIFY_SLV_SNIFF_TIMEOUT
        if ((lc_sniff_slv_send_pkt & (BIT0 << pid)) != 0 )
        {
            lc_sniff_slv_send_pkt &= ~(BIT0 << pid);
            if (lc_sniff_slv_send_pkt == 0)
            {
                UINT16 reg21c = BB_read_baseband_register(0x21c);
                reg21c |= BIT6;
                BB_write_baseband_register(0x21c, reg21c);
            }
        }
#endif
#ifdef _DAPE_SLV_SUPTO_IMPROVE
        /* if we are slave, then we can disable tpoll for some time for
           other HW schedule such as page scan. */
        if ((lut_index >= 8) && (ce_ptr->remote_dev_role == MASTER))
        {
            UINT16 slv_tpoll_mask_reg = reg_SLAVE_TPOLL_MASK_REG[lut_index - 8];
            UINT16 slv_tpoll_mask_val =
                                  BB_read_baseband_register(slv_tpoll_mask_reg);
            BB_write_baseband_register(slv_tpoll_mask_reg, slv_tpoll_mask_val | BIT12);
            g_slv_tpoll_disabled = lut_index;
            /* SW timer */
            if (en_slv_tpoll_timer!= NULL)
            {
                OS_DELETE_TIMER(&en_slv_tpoll_timer);
            }
            /* Create a tid timer */
            OS_CREATE_TIMER(ONESHOT_TIMER, &en_slv_tpoll_timer,
                    en_slv_tpoll_callback, 0, 0);
            OS_START_TIMER(en_slv_tpoll_timer,
                (UINT16)(SLOT_VAL_TO_TIMER_VAL(LMP_SUPERVISION_TIMER_RESOLUTION)));
        }
#endif
        RT_BT_LOG(GRAY, MSG_SUPTER_TO, 14,
          clk, temp_sup_timeout_var, sup_to,
          pid, am_addr, ce_index, ce_ptr->ce_status,
          lut_index, lower_lut_contents, upper_lut_contents,
          val_piconet_info,
          schd_info->tx_status,
          schd_info->pkt_type_lut >> 12,
          schd_info->pkt_src);
#ifdef _DAPE_TEST_CHK_LE_MASTER_LEGACY_SLV_LEGACY_DISCONNECT
          SET_BT_GPIO_OUTPUT_HIGH(0);
          SET_BT_GPIO_OUTPUT_LOW(0);
#endif
#ifdef _DAPE_TEST_CHK_NEW_PLATFORM_DISCONN
        lc_get_clock_in_scatternet(&clk, pid);
        RT_BT_LOG(WHITE, DAPE_TEST_LOG213, 2,clk, BB_read_baseband_register(0xc4));
#endif
#ifdef _DAPE_TEST_CHK_LEGACY_CONNECTION_DISCONN
        lc_get_clock_in_scatternet(&clk, pid);
        RT_BT_LOG(WHITE, DAPE_TEST_LOG213, 2,clk, BB_read_baseband_register(0xc4));
          SET_BT_GPIO_OUTPUT_HIGH(0);
          SET_BT_GPIO_OUTPUT_LOW(0);

#endif

    }

    return;
}

/**
 * Allocate a free entry from the connection entity database
 * (#lmp_connection_entity).
 *
 * \param None.
 *
 * \return New connection entity index, if the operation is successful.
 *         BT_FW_ERROR, otherwise.
 */
UINT16 lmp_allocate_entity_from_ce_database(void)
{
    UINT16 ce_index = 0;
    UINT16 conn_handle;
    LMP_CONNECTION_ENTITY *ce_ptr;

    for(ce_index = 0; ce_index < LMP_MAX_CE_DATABASE_ENTRIES; ce_index++)
    {
        ce_ptr = &lmp_connection_entity[ce_index];

        if(ce_ptr->entity_status == UNASSIGNED)
        {
            if(lmp_allocate_conn_handle(&conn_handle) != BT_FW_SUCCESS)
            {
                LMP_ERR(log_file, "No free connection handles");
                return BT_FW_ERROR;
            }
            ce_ptr->entity_status = ASSIGNED;

            lmp_ch_to_ce_index_table[conn_handle-1].ce_index = ce_index;

            /* Initialize connection handle */
            ce_ptr->connection_type.connection_handle = conn_handle;
            /* Initialize link policy settings */
            ce_ptr->link_policy_settings =
                lmp_self_device_data.default_link_policy_settings;

            /* Initialize Timers */
            OS_CREATE_TIMER(ONESHOT_TIMER, &ce_ptr->supervision_timeout_handle,
                __sup_to_pwa, (void *)((UINT32)ce_index), 0);

            OS_CREATE_TIMER(ONESHOT_TIMER, &ce_ptr->lmp_response_timer_handle,
                lmp_response_timeout_handler, (void *)((UINT32)ce_index), 0);

            bz_auth_create_auth_timers(ce_index);

            return ce_index;
        }
    }

    return BT_FW_ERROR;
}

/**
 * Release the connection entity index (\a ce_index) to the
 * #lmp_connection_entity database.
 *
 * \param ce_index Connection entity index of the entry to be freed.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_release_entity_to_ce_database(UINT16 ce_index)
{
    UINT16 conn_handle;

    if(lmp_connection_entity[ce_index].entity_status != UNASSIGNED)
    {
        conn_handle = lmp_connection_entity[ce_index].
                      connection_type.connection_handle;
        lmp_release_conn_handle(conn_handle);
        lmp_init_connection_entity(ce_index);
        lmp_connection_entity[ce_index].entity_status = UNASSIGNED;
        return BT_FW_SUCCESS;
    }
    return BT_FW_ERROR;
}
#ifdef _DAPE_SLV_SUPTO_IMPROVE
void en_slv_tpoll_callback(TimerHandle_t timer_handle)
{
    UINT16 slv_tpoll_mask_val;
    UINT16 slv_tpoll_mask_reg;
    if (g_slv_tpoll_disabled != 0xFF)
    {
        slv_tpoll_mask_reg = reg_SLAVE_TPOLL_MASK_REG[g_slv_tpoll_disabled - 8];
        slv_tpoll_mask_val = BB_read_baseband_register(slv_tpoll_mask_reg);
        BB_write_baseband_register(slv_tpoll_mask_reg, slv_tpoll_mask_val & (~BIT12));
        g_slv_tpoll_disabled = 0xFF;
    }
}

#endif

