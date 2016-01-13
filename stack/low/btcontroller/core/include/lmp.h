/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  Contains the LMP module external API.
 */

/** \addtogroup lmp_external
 *   @{ */
#ifndef __LMP_H__
#define __LMP_H__

#include "lmp_defines.h"
#include "lmp_1_2_defines.h"
#include "lmp_sco.h"            /* Includes the public SCO interface */

#define SEND_NUM_COMPLETED_SIGNAL_TO_HOST(ce_index)                     \
    lmp_handle_num_packets_timer(ce_index);

#define LMP_LAST_ACCEPTED_MAX_SLOT_5                          5
#define LMP_LAST_ACCEPTED_MAX_SLOT_3                          3
#define LMP_LAST_ACCEPTED_MAX_SLOT_1                          1
#define LMP_LAST_ACCEPTED_MAX_SLOT_INVALID                 0xFF

#define LMP_MAX_SLOT_5                                        5
#define LMP_MAX_SLOT_3                                        3
#define LMP_MAX_SLOT_1                                        1
#define LMP_MAX_SLOT_INVALID                               0xFF

#define LMP_LAST_MAX_SLOT_REQ_SENT_5_SLOT                     1
#define LMP_LAST_MAX_SLOT_REQ_SENT_3_SLOT                     2
#define LMP_LAST_MAX_SLOT_REQ_SENT_INVALID                 0xFF

#ifdef COMPILE_NESTED_PAUSE_RESUME
#define ACL_RESUME                                       0x0000
#define ACL_PAUSED_PARK                                  0x0001
#define ACL_PAUSED_AFH                                   0x0002
#define ACL_PAUSED_MSS                                   0x0004
#define ACL_PAUSED_ENCRYPTION                            0x0008
#define ACL_PAUSED_HOLD                                  0x0010
#define ACL_PAUSED_DETACH                                0x0020
#define ACL_PAUSED_ALL                                   0xFFFF
#endif /* COMPILE_NESTED_PAUSE_RESUME */

#define LMP_FEATURE_MASK_BIT(page, byte, bit, on)  ((on) ? (1 << (bit)) : 0) 

#ifndef ON
#define ON      1
#endif

#ifndef OFF
#define OFF     0
#endif

enum PCM_CODEC_TYPE_ {
    PCM_CODEC_TYPE_LINEAR = 0,
    PCM_CODEC_TYPE_ULAW,
    PCM_CODEC_TYPE_ALAW,
    PCM_CODEC_TYPE_CVSD,
    PCM_CODEC_TYPE_MSBC
}; 

/*----------------------------------------------------------------------*/
/*         Supported Feature in Feature Mask Definition - austin        */
/*----------------------------------------------------------------------*/
// [Page 0 Byte 0]
#define FMD_00_3_SLOT_PACKETS                LMP_FEATURE_MASK_BIT(0, 0, 0, ON)
#define FMD_01_5_SLOT_PACKETS                LMP_FEATURE_MASK_BIT(0, 0, 1, ON)
#define FMD_02_ENCRYPTION                    LMP_FEATURE_MASK_BIT(0, 0, 2, ON)
#define FMD_03_SLOT_OFFSET                   LMP_FEATURE_MASK_BIT(0, 0, 3, ON)
#define FMD_04_TIMING_ACCURACY               LMP_FEATURE_MASK_BIT(0, 0, 4, ON)
#ifdef COMPILE_ROLE_SWITCH
#define FMD_05_ROLE_SWITCH                   LMP_FEATURE_MASK_BIT(0, 0, 5, ON)
#else
#define FMD_05_ROLE_SWITCH                   LMP_FEATURE_MASK_BIT(0, 0, 5, OFF)
#endif
#define FMD_06_HOLD_MODE                     LMP_FEATURE_MASK_BIT(0, 0, 6, ON)
#define FMD_07_SNIFF_MODE                    LMP_FEATURE_MASK_BIT(0, 0, 7, ON)

// [Page 0 Byte 1]
#define FMD_08_PARK_STATE                    LMP_FEATURE_MASK_BIT(0, 1, 0, ON)
#define FMD_09_POWER_CONTROL_REQUEST         LMP_FEATURE_MASK_BIT(0, 1, 1, ON)
#ifdef _DAPE_TEST_FOR_MUTE
// Nish turn off CQDDR for running MUTE
#define FMD_10_CQDDR                         LMP_FEATURE_MASK_BIT(0, 1, 2, OFF)
#else
#define FMD_10_CQDDR                         LMP_FEATURE_MASK_BIT(0, 1, 2, ON)
#endif

#define FMD_11_SCO_LINK                      LMP_FEATURE_MASK_BIT(0, 1, 3, ON)
#define FMD_12_HV2_PACKETS                   LMP_FEATURE_MASK_BIT(0, 1, 4, ON)
#define FMD_13_HV3_PACKETS                   LMP_FEATURE_MASK_BIT(0, 1, 5, ON)
#define FMD_14_U_LAW_LOG_SYNCHRONOUS_DATA    LMP_FEATURE_MASK_BIT(0, 1, 6, ON)
#define FMD_15_A_LAW_LOG_SYNCHRONOUS_DATA    LMP_FEATURE_MASK_BIT(0, 1, 7, ON)

// [Page 0 Byte 2]
#define FMD_16_CVSD_SYNCHRONOUS_DATA         LMP_FEATURE_MASK_BIT(0, 2, 0, ON)
#define FMD_17_PAGING_PARAMETER_NEGOTIATION  LMP_FEATURE_MASK_BIT(0, 2, 1, ON)
#define FMD_18_POWER_CONTROL                 LMP_FEATURE_MASK_BIT(0, 2, 2, ON)
#define FMD_19_TRANSPARENT_SYNCHRONOUS_DATA  LMP_FEATURE_MASK_BIT(0, 2, 3, ON)
#define FMD_20_FLOW_CTRL_LAG_LEAST_SIGN_BIT  LMP_FEATURE_MASK_BIT(0, 2, 4, ON)
#define FMD_21_FLOW_CTRL_LAG_MIDDLE_BIT      LMP_FEATURE_MASK_BIT(0, 2, 5, ON)
#define FMD_22_FLOW_CTRL_LAG_MOST_SIGN_BIT   LMP_FEATURE_MASK_BIT(0, 2, 6, ON)
#define FMD_23_BROADCAST_ENCRYPTION          LMP_FEATURE_MASK_BIT(0, 2, 7, ON)

// [Page 0 Byte 3]
#define FMD_24_RESERVED                      LMP_FEATURE_MASK_BIT(0, 3, 0, OFF)
#define FMD_25_EDR_ACL_2M_MODE               LMP_FEATURE_MASK_BIT(0, 3, 1, ON)
#define FMD_26_EDR_ACL_3M_MODE               LMP_FEATURE_MASK_BIT(0, 3, 2, ON)
#define FMD_27_ENHANCED_INQUIRY_SCAN         LMP_FEATURE_MASK_BIT(0, 3, 3, ON)
#define FMD_28_INTERLACED_INQUIRY_SCAN       LMP_FEATURE_MASK_BIT(0, 3, 4, ON)
#define FMD_29_INTERLACED_PAGE_SCAN          LMP_FEATURE_MASK_BIT(0, 3, 5, ON)
#define FMD_30_RSSI_WITH_INQUIRY_RESULTS     LMP_FEATURE_MASK_BIT(0, 3, 6, ON)
#define FMD_31_ESCO_LINK_EV3_PACKETS         LMP_FEATURE_MASK_BIT(0, 3, 7, ON)

// [Page 0 Byte 4]
#define FMD_32_EV4_PACKETS                   LMP_FEATURE_MASK_BIT(0, 4, 0, ON)
#define FMD_33_EV5_PACKETS                   LMP_FEATURE_MASK_BIT(0, 4, 1, ON)
#define FMD_34_RESERVED                      LMP_FEATURE_MASK_BIT(0, 4, 2, OFF)
#ifdef _ENABLE_AFH_FUNCTION_
#define FMD_35_AFH_CAPABLE_SLAVE             LMP_FEATURE_MASK_BIT(0, 4, 3, ON)
#define FMD_36_AFH_CLASSIFICATION_SLAVE      LMP_FEATURE_MASK_BIT(0, 4, 4, ON)
#else
#define FMD_35_AFH_CAPABLE_SLAVE             LMP_FEATURE_MASK_BIT(0, 4, 3, OFF)
#define FMD_36_AFH_CLASSIFICATION_SLAVE      LMP_FEATURE_MASK_BIT(0, 4, 4, OFF)
#endif
#define FMD_37_BR_EDR_NOT_SUPPORTED          LMP_FEATURE_MASK_BIT(0, 4, 5, OFF)
#ifdef LE_MODE_EN
#define FMD_38_LE_SUPPORTED_CONTROLLER       LMP_FEATURE_MASK_BIT(0, 4, 6, ON)
#else
#define FMD_38_LE_SUPPORTED_CONTROLLER       LMP_FEATURE_MASK_BIT(0, 4, 6, OFF)
#endif
#define FMD_39_3_SLOT_EDR_ACL_PACKETS        LMP_FEATURE_MASK_BIT(0, 4, 7, ON)

// [Page 0 Byte 5]
#define FMD_40_5_SLOT_EDR_ACL_PACKETS        LMP_FEATURE_MASK_BIT(0, 5, 0, ON)
#define FMD_41_SNIFF_SUBRATING               LMP_FEATURE_MASK_BIT(0, 5, 1, ON)
#define FMD_42_PAUSE_ENCRYPTION              LMP_FEATURE_MASK_BIT(0, 5, 2, ON)
#ifdef _ENABLE_AFH_FUNCTION_
#define FMD_43_AFH_CAPABLE_MASTER            LMP_FEATURE_MASK_BIT(0, 5, 3, ON)
#define FMD_44_AFH_CLASSIFICATION_MASTER     LMP_FEATURE_MASK_BIT(0, 5, 4, ON)
#else
#define FMD_43_AFH_CAPABLE_MASTER            LMP_FEATURE_MASK_BIT(0, 5, 3, OFF)
#define FMD_44_AFH_CLASSIFICATION_MASTER     LMP_FEATURE_MASK_BIT(0, 5, 4, OFF)
#endif
#define FMD_45_EDR_ESCO_2M_MODE              LMP_FEATURE_MASK_BIT(0, 5, 5, ON)
#define FMD_46_EDR_ESCO_3M_MODE              LMP_FEATURE_MASK_BIT(0, 5, 6, ON)
#define FMD_47_3_SLOT_EDR_ESCO_PACKETS       LMP_FEATURE_MASK_BIT(0, 5, 7, ON)

// [Page 0 Byte 6]
#define FMD_48_EXTENDED_INQUIRY_RESPONSE     LMP_FEATURE_MASK_BIT(0, 6, 0, ON)
#ifdef LE_MODE_EN
#define FMD_49_SIMU_LE_AND_BR_EDR_CONTROLLER LMP_FEATURE_MASK_BIT(0, 6, 1, ON)
#else
#define FMD_49_SIMU_LE_AND_BR_EDR_CONTROLLER LMP_FEATURE_MASK_BIT(0, 6, 1, OFF)
#endif
#define FMD_50_RESERVED                      LMP_FEATURE_MASK_BIT(0, 6, 2, OFF)
#define FMD_51_SECURE_SIMPLE_PARING          LMP_FEATURE_MASK_BIT(0, 6, 3, ON)
#define FMD_52_ENCAPSULATED_PDU              LMP_FEATURE_MASK_BIT(0, 6, 4, ON)
#define FMD_53_ERRONEOUS_DATA_REPORTING      LMP_FEATURE_MASK_BIT(0, 6, 5, ON)
#define FMD_54_NON_FLUSHABLE_PB_FLAG         LMP_FEATURE_MASK_BIT(0, 6, 6, ON)
#define FMD_55_RESERVED                      LMP_FEATURE_MASK_BIT(0, 6, 7, OFF)

// [Page 0 Byte 7]
#define FMD_56_SUPERVISION_TO_CHANGED_EVENT  LMP_FEATURE_MASK_BIT(0, 7, 0, ON)
#define FMD_57_IQUIRY_TX_POWER_LEVEL         LMP_FEATURE_MASK_BIT(0, 7, 1, ON)
#ifdef VER_3_0
#define FMD_58_ENHANCED_POWER_CONTROL        LMP_FEATURE_MASK_BIT(0, 7, 2, ON)
#else
#define FMD_58_ENHANCED_POWER_CONTROL        LMP_FEATURE_MASK_BIT(0, 7, 2, OFF)
#endif
#define FMD_59_RESERVED                      LMP_FEATURE_MASK_BIT(0, 7, 3, OFF)
#define FMD_60_RESERVED                      LMP_FEATURE_MASK_BIT(0, 7, 4, OFF)
#define FMD_61_RESERVED                      LMP_FEATURE_MASK_BIT(0, 7, 5, OFF)
#define FMD_62_RESERVED                      LMP_FEATURE_MASK_BIT(0, 7, 6, OFF)
#define FMD_63_EXTENDED_FEATURES             LMP_FEATURE_MASK_BIT(0, 7, 7, ON)

// [Page 1 Byte 0]
#define FMD_64_SECURE_SIMPLE_PAIRING_HOST    LMP_FEATURE_MASK_BIT(1, 0, 0, OFF)
#define FMD_65_LE_SUPPORTED_HOST             LMP_FEATURE_MASK_BIT(1, 0, 1, OFF)
#define FMD_66_SIMU_LE_AND_BR_EDR_HOST       LMP_FEATURE_MASK_BIT(1, 0, 2, OFF)
#ifdef _SUPPORT_SECURE_CONNECTION_
#define FMD_67_SECURE_CONN_HOST              LMP_FEATURE_MASK_BIT(1, 0, 3, OFF)
#endif

// [Page 2 Byte 1]
#ifdef _SUPPORT_SECURE_CONNECTION_
#define FMD_136_SECURE_CONN_CONTROLLER       LMP_FEATURE_MASK_BIT(2, 1, 0, ON)
#define FMD_137_PING                         LMP_FEATURE_MASK_BIT(2, 1, 1, ON)
#endif

/*----------------------------------------------------------------------------*/

#define FMD_PAGE0_BYTE0 \
    FMD_00_3_SLOT_PACKETS | FMD_01_5_SLOT_PACKETS | FMD_02_ENCRYPTION | \
    FMD_03_SLOT_OFFSET | FMD_04_TIMING_ACCURACY  | FMD_05_ROLE_SWITCH | \
    FMD_06_HOLD_MODE | FMD_07_SNIFF_MODE

#define FMD_PAGE0_BYTE1 \
    FMD_08_PARK_STATE | FMD_09_POWER_CONTROL_REQUEST | FMD_10_CQDDR | \
    FMD_11_SCO_LINK | FMD_12_HV2_PACKETS  | FMD_13_HV3_PACKETS | \
    FMD_14_U_LAW_LOG_SYNCHRONOUS_DATA | FMD_15_A_LAW_LOG_SYNCHRONOUS_DATA

#define FMD_PAGE0_BYTE2 \
    FMD_16_CVSD_SYNCHRONOUS_DATA | FMD_17_PAGING_PARAMETER_NEGOTIATION | \
    FMD_18_POWER_CONTROL | FMD_19_TRANSPARENT_SYNCHRONOUS_DATA | \
    FMD_20_FLOW_CTRL_LAG_LEAST_SIGN_BIT | FMD_21_FLOW_CTRL_LAG_MIDDLE_BIT | \
    FMD_22_FLOW_CTRL_LAG_MOST_SIGN_BIT | FMD_23_BROADCAST_ENCRYPTION

#define FMD_PAGE0_BYTE3 \
    FMD_24_RESERVED | FMD_25_EDR_ACL_2M_MODE | FMD_26_EDR_ACL_3M_MODE | \
    FMD_27_ENHANCED_INQUIRY_SCAN | FMD_28_INTERLACED_INQUIRY_SCAN  | \
    FMD_29_INTERLACED_PAGE_SCAN | FMD_30_RSSI_WITH_INQUIRY_RESULTS | \
    FMD_31_ESCO_LINK_EV3_PACKETS

#define FMD_PAGE0_BYTE4 \
    FMD_32_EV4_PACKETS | FMD_33_EV5_PACKETS | FMD_34_RESERVED | \
    FMD_35_AFH_CAPABLE_SLAVE | FMD_36_AFH_CLASSIFICATION_SLAVE  | \
    FMD_37_BR_EDR_NOT_SUPPORTED | FMD_38_LE_SUPPORTED_CONTROLLER | \
    FMD_39_3_SLOT_EDR_ACL_PACKETS

#define FMD_PAGE0_BYTE5 \
    FMD_40_5_SLOT_EDR_ACL_PACKETS | FMD_41_SNIFF_SUBRATING | \
    FMD_42_PAUSE_ENCRYPTION | FMD_43_AFH_CAPABLE_MASTER | \
    FMD_44_AFH_CLASSIFICATION_MASTER  | FMD_45_EDR_ESCO_2M_MODE | \
    FMD_46_EDR_ESCO_3M_MODE | FMD_47_3_SLOT_EDR_ESCO_PACKETS

#define FMD_PAGE0_BYTE6 \
    FMD_48_EXTENDED_INQUIRY_RESPONSE | FMD_49_SIMU_LE_AND_BR_EDR_CONTROLLER | \
    FMD_50_RESERVED | FMD_51_SECURE_SIMPLE_PARING | FMD_52_ENCAPSULATED_PDU | \
    FMD_53_ERRONEOUS_DATA_REPORTING | FMD_54_NON_FLUSHABLE_PB_FLAG | \
    FMD_55_RESERVED

#define FMD_PAGE0_BYTE7 \
    FMD_56_SUPERVISION_TO_CHANGED_EVENT | FMD_57_IQUIRY_TX_POWER_LEVEL | \
    FMD_58_ENHANCED_POWER_CONTROL | FMD_59_RESERVED | FMD_60_RESERVED | \
    FMD_61_RESERVED | FMD_62_RESERVED | FMD_63_EXTENDED_FEATURES
#ifdef _SUPPORT_SECURE_CONNECTION_
#define FMD_PAGE1_BYTE0 \
    FMD_64_SECURE_SIMPLE_PAIRING_HOST | FMD_65_LE_SUPPORTED_HOST | \
    FMD_66_SIMU_LE_AND_BR_EDR_HOST | FMD_67_SECURE_CONN_HOST
#else
#define FMD_PAGE1_BYTE0 \
    FMD_64_SECURE_SIMPLE_PAIRING_HOST | FMD_65_LE_SUPPORTED_HOST | \
    FMD_66_SIMU_LE_AND_BR_EDR_HOST
#endif
#ifdef _SUPPORT_SECURE_CONNECTION_
#define FMD_PAGE2_BYTE0 \
    FMD_136_SECURE_CONN_CONTROLLER| FMD_137_PING
#endif
#define BB_SET_TDD_BIT                                   0x0100

#define PDU_TIMER_AS_WAIT_FOR_CLEAR_PKT    0x10
#define TIME_WAIT_FOR_CLEAR_PKT            10 /* 10 ms */
#define MAX_LC_SCHEDULE_PKT_CLEAR_COUNT    20 /* => 200 ms */

#ifdef COMPILE_AFH_HOP_KERNEL
extern UINT16 num_of_tx_pkt;
extern UINT16 num_of_rx_pkt;
#endif

/*LC module use this PDU buffer pool handle to deallocate the buffer */
extern POOL_ID lmp_pdu_buffer_pool_handle ;
extern POOL_ID lmp_fhs_pkt_buffer_pool_handle ;

extern TASK_ID lmp_task_handle ;

/* Inquiry result table */
extern LMP_INQUIRY_RESULT_DATA 
                    lmp_inquiry_result_data[LMP_MAX_INQUIRY_RESULT_DATA];

/* Self device data table */
extern LMP_SELF_DEVICE_DATA lmp_self_device_data ;

/** ACL Connection Entity database */
#ifndef _ENABLE_RETENTION_FLOW_FOR_DLPS_
extern LMP_CONNECTION_ENTITY lmp_connection_entity[LMP_MAX_CE_DATABASE_ENTRIES];
#else
extern UINT16 lmp_connection_entity_store_index;
extern LMP_CONNECTION_ENTITY lmp_connection_entity_bton;
extern SECTION_LOW_BSS LMP_CONNECTION_ENTITY lmp_connection_entity[LMP_MAX_CE_DATABASE_ENTRIES];
#endif

extern LMP_CONN_HANDLE_TO_CE_INDEX_TABLE
        lmp_ch_to_ce_index_table[LMP_MAX_CONN_HANDLES] ;

#ifdef ENABLE_SCO
extern LMP_SCO_CONNECTION_DATA
        lmp_sco_connection_data[LMP_MAX_SCO_CONN_ENTRIES];
#endif

extern UCHAR lmp_num_inq_resp_received;
/* LC module before writing the inquiry command to the baseband
 * will update this variable.
 */
extern UCHAR lmp_num_inq_resp_expected ;
extern LMP_ROLE_SWITCH_PARAMS lmp_role_switch_data ;
extern UCHAR lmp_periodic_inquiry ;

#define LMP_I_AM_MASTER()                                                    \
    ((lc_scatternet_table[MASTER_PICONET].lc_piconet_role == PICONET_MASTER) \
    ? 1 : 0)

#define LMP_NO_CONNECTION()                                                  \
    ((lmp_self_device_data.number_of_acl_conn == 0) ? 1 : 0)

#define LMP_I_AM_SLAVE()                                                     \
    ((lc_scatternet_table[MASTER_PICONET].lc_piconet_role == PICONET_SLAVE)  \
    ? 1 : 0)

#define LMP_REMOVE_BD_ADDR_FROM_HASH               \
                    lmp_remove_bd_addr_from_hash

#define LMP_GET_CE_INDEX_FROM_BD_ADDR              \
                        lmp_get_CE_index_from_BD_ADDR

#define LMP_GET_CE_INDEX_FROM_AM_ADDR_PPI          \
                        lmp_get_CE_index_from_AM_ADDR_PPI

#define LMP_GET_CE_INDEX_FROM_CONN_HANDLE          \
                        lmp_get_CE_index_from_conn_handle

#define LMP_GET_CE_INDEX_FROM_AR_ADDR              \
                        lmp_get_CE_index_from_AR_ADDR

#define LMP_GET_CE_INDEX_FROM_PM_ADDR              \
                        lmp_get_CE_index_from_PM_ADDR

void lmp_set_ce_status(UINT16 ce_index, UINT8 new_ce_status);

/** 
 * Returns TRUE if the device has multple acl connections, and FALSE
 * otherwise.
 * 
 * \param None.
 *
 * \return TRUE or FALSE.
 */
#define lmp_have_multiple_acls() (lmp_self_device_data.number_of_acl_conn > 1)

extern LMP_HOST_INFO_DATA   lmp_host_info_data ;
extern UCHAR                lmp_event_filters_written ;
extern UCHAR                lmp_num_packets_timer_flag ;
extern UCHAR                lmp_mss_state ;
extern LMP_FEATURE_DATA lmp_feature_data;
extern UINT16 bc_ce_index ; /* Broadcast ce_index */
extern UINT16 lmp_assigned_ce_index;

extern UINT16 lmp_sup_timeout_var[LMP_MAX_CE_DATABASE_ENTRIES];
extern UCHAR lmp_sup_var_timeout_var[LMP_MAX_CE_DATABASE_ENTRIES];

#if defined(ENABLE_SCO) || defined(COMPILE_ESCO)

#ifdef SCO_OVER_HCI
extern UCHAR sync_link_codec_state;
#endif /* SCO_OVER_HCI */

extern UCHAR sync_link_codec_type;
extern UCHAR pcm_codec_availability;
extern UCHAR uda_codec_availability;

extern UCHAR pcm_ex_codec_format;
extern UCHAR pcm_ex_codec_format_8bit;
extern UINT16 pcmifctrl1;
extern UINT16 pcmifctrl2;
extern UCHAR scoconv;
extern UINT16 pcmconvert;

/*  Added by Wallice for PCM Enhancement.  2013/07/08  */
extern UCHAR hci_excodec_state;
extern UINT16 pcmifctrl3;
/*  End Added by Wallice for PCM Enhancement.  2013/07/08  */

#endif /* defined(ENABLE_SCO) || defined(COMPILE_ESCO) */

#ifdef COMPILE_CHANNEL_ASSESSMENT
extern TimerHandle_t la_period_timer;
#endif

extern OS_HANDLE lmp_task_handle ;
extern UINT16 lmp_unpark_ce_index;
extern UCHAR lmp_current_nb;

void  lmp_extract_bd_addr_from_fhs_packet(UCHAR *lmp_fhs_pkt_recd,
                                         UCHAR *bd_addr);

#ifdef TEST_MODE
extern UCHAR lc_test_mode_ack_recd;
extern UCHAR lc_tci_pause_flag;

#if !defined(_DUT_DELAYED_LOOPBACK_MODE_)
extern SECTION_SRAM UINT8 lmp_test_mode_data_buf[1024];
#endif

#endif

#ifdef POWER_SAVE_FEATURE
extern UINT16 lc_power_ctr_config;
#endif

/* LMP API's prototypes */

API_RESULT lmp_init(void);

API_RESULT lmp_shutdown(void);

API_RESULT lmp_module_reset(void);

UCHAR lmp_allocate_am_addr(UCHAR *am_addr, UINT16 *ce_index);

API_RESULT lmp_release_am_addr_ppi(UCHAR am_addr, UINT16 phy_piconet_id);

API_RESULT lmp_get_CE_index_from_BD_ADDR(UCHAR *bd_addr, UINT16* ce_index);

API_RESULT lmp_get_CE_index_from_PM_ADDR(UCHAR pm_addr, UINT16* ce_index,
                                            UCHAR piconet_id);

API_RESULT lmp_get_CE_index_from_AR_ADDR(UCHAR ar_addr, UINT16* ce_index,
                                            UCHAR piconet_id);

API_RESULT lmp_get_CE_index_from_AM_ADDR_PPI(UCHAR am_addr, UCHAR piconet_id,
                                            UINT16* ce_index);

API_RESULT lmp_get_CE_index_from_conn_handle(UINT16 conn_handle,
                                                UINT16* ce_index);

void lmp_remove_bd_addr_from_hash(UCHAR *bd_addr);

API_RESULT lmp_generate_pdu_impl(UINT16 ce_index, UCHAR* param_list,
        UCHAR pdu_len, LMP_TRAN_ID tran_id, UINT8 ce_status,
        UCHAR use_dm1);

/**
 * Generate LMP pdu and deliver it to the remote device. It simply queues the
 * pdu to the LC module and it in turn delivers the PDU to remote device.
 *
 * \param ce_index The index of the Connection entity database corresponding
 *                 to the link.
 * \param param_list LMP PDU parameter list. Element0 will be the opcode,
 *                   Element1 will be our role(Master/Slave), Element2 and
 *                   the rest will be specific to the PDU to be sent.
 * \param pdu_len Length of the PDU (Use predefined macros like
 *                #LMP_ACCEPTED_LEN - if length has to be hard coded, then it
 *                should be Bluetooth spec defined length of that PDU plus
 *                one).
 * \param tid Transaction ID to be used for this PDU.
 * \param ce_status The status with which the main LM state machine has to be
 *                  updated.
 *
 * \return API_SUCCESS, if the operation is successful. API_FAILURE, otherwise.
 */
#define lmp_generate_pdu(ce_index, param_list, pdu_len, tid, ce_status)  \
    lmp_generate_pdu_impl(ce_index, param_list, pdu_len, tid, ce_status, FALSE)

/**
 * Generate LMP pdu and deliver it to the remote device using DM1 packet type.
 * It simply queues the pdu to the LC module and it in turn delivers the PDU
 * to remote device.
 *
 * \param ce_index The index of the Connection entity database corresponding
 *                 to the link.
 * \param param_list LMP PDU parameter list. Element0 will be the opcode,
 *                   Element1 will be our role(Master/Slave), Element2 and
 *                   the rest will be specific to the PDU to be sent.
 * \param pdu_len Length of the PDU (Use predefined macros like
 *                #LMP_ACCEPTED_LEN - if length has to be hard coded, then it
 *                should be Bluetooth spec defined length of that PDU plus
 *                one).
 * \param tid Transaction ID to be used for this PDU.
 * \param ce_status The status with which the main LM state machine has to be
 *                  updated.
 *
 * \return API_SUCCESS, if the operation is successful. API_FAILURE, otherwise.
 */
#define lmp_generate_pdu_dm1(ce_index, param_list, pdu_len, tid, ce_status)  \
    lmp_generate_pdu_impl(ce_index, param_list, pdu_len, tid, ce_status, TRUE)

API_RESULT lmp_get_pm_addr(UCHAR *pm_addr, UINT16 phy_piconet_id);

API_RESULT lmp_put_pm_addr(UCHAR pm_addr, UINT16 phy_piconet_id);

API_RESULT lmp_get_ar_addr(UCHAR *ar_addr, UINT16 phy_piconet_id);

API_RESULT lmp_put_ar_addr(UCHAR ar_addr, UINT16 phy_piconet_id);

#ifdef COMPILE_SNIFF_MODE
#ifndef _CCH_SLOT_OFFSET_
API_RESULT lmp_get_slot_offset(UINT16 ce_index, UCHAR *init_proc);
#endif
#endif /* COMPILE_SNIFF_MODE */

API_RESULT lmp_add_to_device_cache(DEVICE_INFO device_info);

API_RESULT lmp_get_device_info(UCHAR *bd_addr, DEVICE_INFO *device_info);

void LMP_Task(OS_SIGNAL *signal_ptr);

void lmp_initialize_event_filters(void);

UCHAR lmp_release_conn_handle(UINT16 conn_handle);

void lmp_send_lmp_accepted_impl(UINT16 ce_index, UCHAR opcode, UCHAR trans_id,
        UINT8 ce_status, UCHAR use_dm1);

/**
 * Sends LMP_accepted to the remote device.
 *
 * \param ce_index ACL connection entity index of the connection on which the
 *                 PDU has to be sent.
 * \param opcode Opcode for which this LMP_accepted is being sent.
 * \param tid Transaction ID.
 * \param ce_status Connection entity status (Main state machine status) to
 *                  which the main state machine has to be switched.
 * \return None.
 */
#define lmp_send_lmp_accepted(ce_index, opcode, tid, ce_status)     \
    lmp_send_lmp_accepted_impl(ce_index, opcode, tid, ce_status, FALSE)

/**
 * Sends LMP_accepted to the remote device using DM1 packet type.
 *
 * \param ce_index ACL connection entity index of the connection on which the
 *                 PDU has to be sent.
 * \param opcode Opcode for which this LMP_accepted is being sent.
 * \param tid Transaction ID.
 * \param ce_status Connection entity status (Main state machine status) to
 *                  which the main state machine has to be switched.
 * \return None.
 */
#define lmp_send_lmp_accepted_dm1(ce_index, opcode, tid, ce_status) \
    lmp_send_lmp_accepted_impl(ce_index, opcode, tid, ce_status, TRUE)

void lmp_send_lmp_not_accepted(UINT16 ce_index,UCHAR opcode,
                                   UCHAR trans_id,UCHAR reason);

void lmp_decide_to_send_conn_complete_evt(UINT16 ce_index);

void lmp_generate_beacon_parameters(UINT16 ce_index);
#if 0
#ifdef COMPILE_ROLE_SWITCH
UCHAR lmp_handle_role_switch_fhs_pkt(LMP_FHS_PKT_RECD *lmp_fhs_pkt_recd);
#endif
#endif
void lmp_handle_num_packets_timer(UINT16 ce_index);

void lmp_initialize_inquiry_result_event_filters(void);

void lmp_disconnect_links(UINT16 ce_index, UCHAR reason);

#ifdef COMPILE_ESCO
void lmp_disconnect_esco_links(UINT16 ce_index, UCHAR reason);
#endif

void lmp_cleanup_after_acl_detach_if_no_scheduled_pkt(UINT16 ce_index);
void lmp_cleanup_after_acl_detach(UINT16 ce_index);

API_RESULT lmp_pend_acl_disconnect_from_isr(UINT16 ce_index, UCHAR reason);
API_RESULT lmp_handle_acl_disconnect(UINT16 ce_index, UCHAR reason);
void lmp_disconnect_links(UINT16 ce_index, UCHAR reason);
void lmp_init_connection_entity(UINT16 ce_index);

UCHAR lmp_handle_page_scan_fhs_pkt(LMP_FHS_PKT_RECD * lmp_fhs_pkt_recd,
                    UCHAR phy_piconet_id,UCHAR lut_index,UCHAR * bd_addr);

#ifdef _ENABLE_COMPILE_NESTED_PAUSE_RESUME_
UCHAR lmp_handle_hold_mode_req_complete(UINT16 ce_index, UCHAR transaction_id);
#else
UCHAR lmp_handle_mode_req_complete(UINT16 ce_index, UCHAR transaction_id);
#endif

#ifndef _CCH_SLOT_OFFSET_
UINT16 lmp_get_dsniff(void);
void lmp_put_dsniff(UINT16 dsniff);
#endif

#ifdef _CCH_SLOT_OFFSET_
void global_slot_offset_log();
#ifdef _DAPE_GET_LEGACY_SLOT_OFST_AVOID
void lmp_get_avoid_slot_offset(UCHAR *avoid, UCHAR *slot_ofst);
#endif
UINT16 lmp_get_global_slot_offset(UINT16 ce_index, UINT16 is_esco_ce_index, UINT16 interval, UINT16 slot_num, UINT16 slot_min, UINT16 slot_max);
void lmp_put_global_slot_offset(UINT16 ce_index, UINT16 is_esco_ce_index);
void lmp_disconnect_global_slot_offset(UINT16 ce_index);
UCHAR lmp_find_global_slot_index(UINT16 ce_index, UINT16 is_esco_ce_index);
UCHAR lmp_force_global_slot_offset(UINT16 ce_index, UINT16 is_esco_ce_index, UINT16 interval, UINT16 slot_num, UINT16 slot_val);
UCHAR lmp_cal_m2s_clock_offset(UINT16 ce_index, UINT16 interval , UINT16 use_interval, UCHAR opt_cal_offset);
UCHAR lmp_update_global_slot_offset(UINT16 ce_index, UINT16 is_esco_ce_index);
#endif

void lmp_send_os_start_timer_signal(TimerHandle_t timerid, UINT32 value);

#ifdef COMPILE_PARK_MODE
UCHAR lmp_handle_master_unpark(UINT16 ce_index);
void lmp_start_park_mode_timer(UINT16 ce_index);
void lmp_set_auto_unpark_cnt(UINT16 ce_index);
#endif

#ifdef COMPILE_SNIFF_MODE
API_RESULT lmp_start_sniff_mode(UINT16 ce_index);

API_RESULT lmp_exit_sniff_mode(UINT16 ce_index);
#endif

#ifndef COMPILE_SINGLE_SLOT_PACKETS_ONLY
void lmp_send_max_slot_pdu(UINT16 ce_index);
void lmp_send_max_slot_req_pdu(UINT16 ce_index, UCHAR max_slot);
void lmp_send_max_slot_pdu_to_all_devices(void);
void lmp_send_max_slot_req_pdu_to_all_devices(void);
UCHAR lmp_calculate_max_slot_for_hci(UINT16 ce_index);
UINT32 lmp_calculate_max_slot(UINT32 ce_index);
#endif

#ifdef COMPILE_CQDDR
void lmp_send_auto_rate_pdu(UINT16 ce_index);
void lmp_calculate_and_send_preferred_rate_pdu(UINT16 ce_index);
#endif

void lmp_handle_last_conn_disconnected(void);
UCHAR lmp_generate_random_number(void);
void lmp_send_features_req_or_res_pdu(UINT16 ce_index, UCHAR opcode,
                                                    LMP_TRAN_ID tid);

#ifdef COMPILE_ROLE_SWITCH
void lmp_start_role_switch_during_conn_as_slave(UINT16 ce_index,
        UCHAR *bd_addr);
void lmp_send_slot_offset_pdu(UINT16 ce_index, LMP_TRAN_ID tid);
void lmp_handle_role_switch_failure(UINT16 ce_index, UCHAR reason);
#endif

void lmp_slave_use_am_addr_ppi(UCHAR am_addr, UINT16 phy_piconet_id,
        UINT16 acl_ce_index);

void lmp_slave_unuse_am_addr_ppi(UCHAR am_addr, UINT16 phy_piconet_id);

#ifdef COMPILE_ESCO
void lmp_slave_use_esco_am_addr_ppi(UCHAR esco_am_addr, UINT16 phy_piconet_id,
        UINT16 esco_ce_index);

void lmp_slave_unuse_esco_am_addr_ppi(UCHAR esco_am_addr, UINT16 phy_piconet_id);

#endif /* COMPILE_ESCO */

UCHAR lc_get_master_piconet_id(void);

#ifdef COMPILE_ROLE_SWITCH
UCHAR lmp_check_for_role_switch_in_scatternet(UINT16 ce_index);
#endif

#if defined(ENABLE_SCO) || defined(COMPILE_SNIFF_MODE)
void lmp_determine_full_bandwidth(void);
#endif
void lmp_accept_host_connection_request_pdu(UINT16 ce_index);
void lmp_handle_mss_completion_after_connection(UINT16 ce_index);
void lmp_auth_completed_callback(UINT16 ce_index, UINT16 status,
        UCHAR reason, void* user_data);
void lmp_pause_encryption_callback(UINT16 ce_index, UINT16 status,
        UINT8 auth_role, UINT8 enc_proc, void* user_data);
void lmp_resume_encryption_callback(UINT16 ce_index, UINT16 status,
        UINT8 auth_role, UINT8 enc_proc, void* user_data);

INLINE void lmp_set_mss_state(UCHAR status);

#ifdef COMPILE_ESCO
UCHAR lmp_check_for_allowing_new_esco_connection(void);
#endif

#ifdef COMPILE_PARK_MODE
UCHAR lmp_handle_host_initiated_unpark(UINT16 ce_index);
#endif

#ifdef RTL8723A_B_CUT
void lmp_stop_regular_sw_timers(void);
void lmp_start_regular_sw_timers(void);
#endif


#endif /* __LMP_H__ */

/** @} end: lmp_external */
