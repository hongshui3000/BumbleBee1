/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  Contains the LMP module internal interface.
 */

/** \addtogroup lmp_internal
 *   @{ */
#ifndef __LMP_INTERNAL_H__
#define __LMP_INTERNAL_H__

/*
 *  OS_include files
 */
#include "bt_fw_common.h"
#include "bt_fw_globals.h"

/*
 * Module header files
 */
#include "bt_fw_hci.h"
#include "lc.h"


#include "lmp_defines.h"
#include "lmp_1_2_defines.h"
#include "lmp_sco_internal.h"       /* Includes the Internal SCO interface */


/*
 * Extern variables
 */

extern UINT32 afh_la_cycle_period;

extern TASK_ID lmp_task_handle ;
extern POOL_ID lmp_pdu_buffer_pool_handle ;
extern POOL_ID lmp_fhs_pkt_buffer_pool_handle ;

/* Inquiry result table */
extern LMP_INQUIRY_RESULT_DATA lmp_inquiry_result_data
                            [LMP_MAX_INQUIRY_RESULT_DATA];

/* Self device data table */
extern LMP_SELF_DEVICE_DATA lmp_self_device_data ;
extern ALIGN(4) SECTION_SRAM UINT8 lmp_self_eir_data_buf[MAX_EIR_DATA_LEN];

/*Connection entity database */
extern LMP_CONNECTION_ENTITY  lmp_connection_entity[LMP_MAX_CE_DATABASE_ENTRIES] ;

extern UCHAR lmp_inquiry_data_index ;

extern UCHAR lmp_mss_state ;
extern UCHAR lmp_num_inq_resp_expected ;
extern UCHAR lmp_num_inq_resp_received ;

extern const UINT16 g_lc_pkt_type_lut[16];
extern const UINT16 g_lc_pkt_type_max_len[16];

extern LMP_AM_ADDR_TO_CE_INDEX_TABLE
    lmp_am_addr_to_ce_index_table_ppi[LC_MAX_AM_ADDR][LMP_MAX_PICONETS_SUPPORTED];

extern LMP_CONN_HANDLE_TO_CE_INDEX_TABLE    \
        lmp_ch_to_ce_index_table[LMP_MAX_CONN_HANDLES] ;

#ifdef ENABLE_SCO
extern LMP_SCO_CONNECTION_DATA
        lmp_sco_connection_data[LMP_MAX_SCO_CONN_ENTRIES];
#endif

extern UCHAR lmp_num_packets_timer_flag ;

extern LMP_ROLE_SWITCH_PARAMS lmp_role_switch_data ;
extern UINT16 lmp_assigned_ce_index ;

/*Device Info data */
extern CACHE_TABLE bd_addr_cache[MAX_CACHE];

extern UCHAR bd_addr_cache_start ;
extern UCHAR bd_addr_cache_free ;

extern UINT16 lmp_park_bd_addr_ce_index ;

extern UCHAR lmp_periodic_inquiry;

extern LMP_FEATURE_DATA lmp_feature_data;

/*
 * Function prototypes
 */
void LMP_Task(OS_SIGNAL *signal_ptr);
void lmp_initialize_host_controller(void);
#ifdef _CCH_LPS_
extern TimerHandle_t la_classify_timer;
extern OS_QUEUE_MGR queue_mgr;
extern TimerHandle_t dbg_tid_timer;
void lmp_update_lps_para(void);
#endif
void lmp_init_device_attributes(void);
void lmp_init_connection_entity(UINT16  ce_index);
void lmp_initialize_event_filters(void);
void lmp_response_timeout_handler(TimerHandle_t timer_handle);
void lmp_detach_connection_timer_handler(TimerHandle_t timer_handle);
void lmp_stop_connection_timers(UINT16 ce_index);
#ifdef COMPILE_PARK_MODE
void lmp_abnormal_exit_park_n_cleanup(UINT16 ce_index, UCHAR reason);
#endif /* PARK_MODE */
void lmp_reset_timers(UINT16 ce_index );

UINT8 lmp_get_max_features_page(void);

UCHAR lmp_release_conn_handle(UINT16 conn_handle);

UCHAR lmp_extract_fhs_packet_to_inquiry_table(
                                LMP_FHS_PKT_RECD *lmp_fhs_pkt_recd);
UCHAR lmp_handle_not_accepted_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index);

UCHAR lmp_handle_incoming_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UCHAR piconet_id);

UCHAR lmp_handle_hardware_level_connection (UCHAR am_addr, 
                                        UCHAR phy_piconet_id);
UCHAR lmp_handle_fhs_packet(LMP_FHS_PKT_RECD *lmp_fhs_pkt_recd, 
                                                UCHAR piconet_id);
UCHAR lmp_handle_page_timeout(UCHAR am_addr, UCHAR phy_piconet_id);
void lmp_handle_slave_unpark_req(UCHAR am_addr, UCHAR piconet_id);

UCHAR lmp_handle_inquiry_timeout(void);

void lmp_create_unpark_req_pdu(UINT16 ce_index, UCHAR init_timing);
UCHAR lmp_release_entity_to_ce_database(UINT16 ce_index);
UINT16 lmp_allocate_entity_from_ce_database(void);
UCHAR lmp_handle_inquiry_fhs_pkt(LMP_FHS_PKT_RECD *lmp_fhs_pkt_recd);

UCHAR lmp_handle_page_scan_fhs_pkt(LMP_FHS_PKT_RECD * lmp_fhs_pkt_recd,
                                UCHAR phy_piconet_id,
                                UCHAR lut_index,UCHAR * bd_addr);
BOOLEAN lmp_handle_accepted_pdu (LMP_PDU_PKT *lmp_pdu_ptr,
                                UINT16 ce_index);
BOOLEAN lmp_handle_not_accepted_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
                                    UINT16 ce_index);
UCHAR lmp_handle_features_response_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
                                            UINT16 ce_index);
UCHAR lmp_handle_host_connection_request_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
                                            UINT16 ce_index);
UCHAR lmp_handle_setup_complete_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index);
UCHAR lmp_handle_detach_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index);
UCHAR lmp_handle_features_request_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
                                        UINT16 ce_index);

void lmp_extract_fhs_packet_to_ce(LMP_FHS_PKT_RECD *lmp_fhs_pkt_recd,
                                        UINT16 ce_index);
UCHAR lmp_validate_hold_parms(UCHAR trans_id, UINT16 ce_index, 
                        UINT16 hold_interval, UCHAR *start_timer_flag);

UCHAR lmp_handle_hold_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index);
UCHAR lmp_handle_hold_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index);
UCHAR lmp_handle_start_hold_mode(UINT16 ce_index);
void lmp_handle_hold_instant_signal(UINT16 ce_index);
UCHAR lmp_handle_complete_hold_mode(UINT16 ce_index);
UCHAR lmp_handle_hold_req_accepted(UINT16 ce_index);
UCHAR lmp_handle_sniff_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index);
UCHAR lmp_handle_unsniff_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index);
UCHAR lmp_handle_sniff_req_accepted(UINT16 ce_index);
UCHAR lmp_handle_unsniff_req_accepted(UINT16 ce_index);
void lmp_park_mode_timer_handler(TimerHandle_t timer_handle);
void lmp_generate_beacon_parameters(UINT16 ce_index);
UCHAR lmp_handle_sniff_req_not_accepted(UCHAR reason, UINT16 ce_index);

UCHAR lmp_handle_park_req_accepted(UCHAR transaction_id, UINT16 ce_index);

void lmp_handle_park_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index);

UCHAR lmp_update_beacon_parameters(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index);
UCHAR lmp_validate_beacon_parameters(LMP_PDU_PKT *lmp_pdu_ptr);
void lmp_conn_accept_timeout_timer_handler(TimerHandle_t timer_handle) ;
void lmp_power_control_handler(TimerHandle_t timer_handle);
void lmp_send_power_ctrl_pdu(UINT16 ce_index, UCHAR transaction_id);
API_RESULT lmp_get_CE_index_from_conn_handle(UINT16 conn_handle,
                                                            UINT16* ce_index);
API_RESULT lmp_get_CE_index_from_AM_ADDR_PPI(UCHAR am_addr, UCHAR piconet_id,
                                                            UINT16* ce_index);
UCHAR lmp_handle_stop_pdu_response_time(UINT16 ce_index, LMP_PDU_PKT *pdu_pkt);
void lmp_init_glob_vars(void);
void lmp_init_db_acces_tables(void);
UCHAR lmp_handle_name_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index);
UCHAR lmp_handle_name_resp_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index);
UCHAR lmp_handle_max_slot_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index);
UCHAR lmp_handle_version_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index);
UCHAR lmp_handle_clockoffset_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index);
UCHAR lmp_handle_version_resp_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index);
UCHAR lmp_handle_clockoffset_resp_pdu(LMP_PDU_PKT *lmp_pdu_ptr,UINT16 ce_index);
UCHAR lmp_handle_max_slot_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index);
UCHAR lmp_handle_supervision_timeout_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
                                                            UINT16 ce_index);
UCHAR lmp_handle_slot_offset_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index);


void lmp_send_lmp_not_accepted(UINT16 ce_index,UCHAR opcode,
                    UCHAR trans_id,UCHAR reason);

UCHAR lmp_handle_hold_req_accepted(UINT16 ce_index);
UCHAR lmp_handle_host_connection_req_accepted(UINT16 ce_index);
UCHAR lmp_handle_park_req_accepted(UCHAR transaction_id, UINT16 ce_index);
UCHAR lmp_handle_unsniff_req_accepted(UINT16 ce_index);
UCHAR lmp_handle_sniff_req_accepted(UINT16 ce_index);
UCHAR lmp_handle_park_req_not_accepted(UCHAR reason, UINT16 ce_index);
UCHAR lmp_handle_unsniff_req_not_accepted(UCHAR reason, UINT16 ce_index);
UCHAR lmp_handle_hold_req_not_accepted(UCHAR trans_id, UCHAR reason,
                                                            UINT16 ce_index);
UCHAR lmp_handle_host_connection_req_not_accepted(UCHAR reason,
                                                         UINT16 ce_index);
UCHAR lmp_handle_max_slot_req_accepted(UCHAR transaction_id, UINT16 ce_index);
UCHAR lmp_handle_max_slot_req_not_accepted(LMP_PDU_PKT *lmp_pdu_ptr,
                                                          UINT16 ce_index);
UCHAR lmp_handle_switch_req_not_accepted(LMP_PDU_PKT *lmp_pdu_ptr,
                                                        UINT16 ce_index);
UCHAR lmp_handle_switch_req_accepted(UCHAR transaction_id, UINT16 ce_index);

void lmp_check_and_switch_role_in_scatternet(void);
void lmp_switch_role_in_scatternet(UINT16 ce_index);

UCHAR lmp_handle_switch_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index);

void lmp_handle_role_change_complete(void);

UCHAR lmp_handle_unpark_PM_ADDR_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr, 
                                                        UINT16 phy_piconet_id);

UCHAR lmp_handle_unpark_BD_ADDR_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr);
UCHAR lmp_handle_unpark_req_accepted(UCHAR transaction_id,
                                                        UINT16 ce_index);
UCHAR lmp_handle_set_broadcast_scan_window(LMP_PDU_PKT *lmp_pdu_ptr,
                                                        UINT16 ce_index);
UCHAR lmp_handle_modify_beacon(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index);
void lmp_handle_num_packets_timer(UINT16 ce_index);
void lmp_supervision_timeout_handler(UINT16 ce_index);
void lmp_init_piconet_database(void);
#ifdef OPTIONAL_PAGING
UCHAR lmp_handle_page_scan_mode_accepted(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index);
UCHAR lmp_handle_page_mode_accepted(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index);
UCHAR lmp_handle_page_scan_mode_req_not_accepted(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index);
UCHAR lmp_handle_page_mode_req_not_accepted(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index);
#endif

#ifndef _CCH_REMOVE_USELESS_FUNC_
UCHAR lmp_send_page_mode_pdu(UINT16 ce_index);
#endif
UCHAR lmp_handle_page_mode_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index);
API_RESULT lmp_add_to_device_cache(DEVICE_INFO device_info);
API_RESULT lmp_get_device_info(UCHAR *bd_addr, DEVICE_INFO *device_info);
void init_cache_table(void);
UCHAR lmp_handle_page_scan_mode_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index);

#ifdef COMPILE_CQDDR
API_RESULT lmp_handle_preferred_data_rate(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index);
API_RESULT lmp_handle_auto_data_rate(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index);
#endif
#if 0
UCHAR lmp_handle_role_switch_fhs_pkt(LMP_FHS_PKT_RECD *lmp_fhs_pkt_recd);
#endif
UCHAR lmp_handle_qos(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index);
UCHAR lmp_handle_qos_req(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index);
UCHAR lmp_handle_qos_req_accepted(UINT16 ce_index);
UCHAR lmp_handle_qos_req_not_accepted(UCHAR reason, UINT16 ce_index);

UCHAR lmp_handle_timing_accuracy_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index);
UCHAR lmp_handle_timing_accuracy_res_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index);
UCHAR lmp_handle_start_pdu_response_time(UINT16 ce_index, 
        UCHAR* parameter_list);
UCHAR lmp_handle_pdu_ack_recd(LMP_PDU_PKT *lmp_pdu_pkt, UCHAR piconet_id, UINT32 debug_clock);
void lmp_handle_pdu_sent(LMP_PDU_PKT* lmp_pdu_pkt);
UCHAR lmp_validate_sniff_parms(UCHAR trans_id, UCHAR timing_ctl_flag, 
                    UINT16 ce_index, UINT16 sniff_slot_offset, UINT16 Tsniff,
                    UINT16 sniff_attempt,UINT16 sniff_timeout);

void lmp_initialize_inquiry_result_event_filters(void);
void lmp_extract_bd_addr_from_fhs_packet(UCHAR *lmp_fhs_pkt_recd,
                                         UCHAR *bd_addr);

#ifdef POWER_CONTROL
UCHAR lmp_handle_incr_power_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index);
UCHAR lmp_handle_decr_power_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index);
UCHAR lmp_handle_min_power_pdu(UINT16 ce_index);
UCHAR lmp_handle_max_power_pdu(UINT16 ce_index);
#endif

#ifdef TEST_MODE
UCHAR lmp_handle_test_activate_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index);
UCHAR lmp_handle_test_control_opcode(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index);
#endif /* TEST_MODE */
API_RESULT lmp_get_CE_index_from_BD_ADDR(UCHAR *bd_addr, UINT16* ce_index);

API_RESULT lmp_get_CE_index_from_conn_handle(UINT16 conn_handle, UINT16* ce_index);

API_RESULT lmp_get_CE_index_from_AR_ADDR(UCHAR ar_addr, UINT16* ce_index, UCHAR piconet_id);

API_RESULT lmp_get_CE_index_from_PM_ADDR(UCHAR pm_addr, UINT16* ce_index, UCHAR piconet_id);

UCHAR lmp_allocate_am_addr(UCHAR *am_addr, UINT16 *ce_index);

API_RESULT lmp_release_am_addr_ppi(UCHAR am_addr, UINT16 phy_piconet_id);

API_RESULT lmp_get_pm_addr(UCHAR *pm_addr, UINT16 phy_piconet_id);
API_RESULT lmp_put_pm_addr(UCHAR pm_addr, UINT16 phy_piconet_id);

API_RESULT lmp_get_am_addr_ppi(UCHAR *am_addr, UINT16 phy_piconet_id);

API_RESULT lmp_put_am_addr_ppi(UCHAR am_addr, UINT16 phy_piconet_id);

API_RESULT lmp_get_ar_addr(UCHAR *ar_addr, UINT16 phy_piconet_id);

API_RESULT lmp_put_ar_addr(UCHAR ar_addr, UINT16 phy_piconet_id);

UCHAR lmp_generate_lmp_ptt_req_pdu(UINT16 ce_index, UCHAR pkt_type_table);

#ifdef ESCO_DISC_DEBUG
int lmp_handle_end_of_esco_window(UINT16 esco_lt_addr, UCHAR logical_piconet_id);
#endif /* ESCO_DISC_DEBUG */
UCHAR lmp_convert_air_mode(UCHAR air_mode, UCHAR from, UCHAR to);

#ifdef COMPILE_AFH_HOP_KERNEL
void lmp_send_ch_cl_pdu(UCHAR *afh_map,UINT16 ce_index);
void lmp_make_map_for_ch_cl_pdu(UCHAR *afh_map, UCHAR *classified_map);
#endif

void lmp_continue_conn_proc_after_feature_exchange(UINT16 ce_index);

/** 
 * Checks whether the given \a mangled_ce_index is ACL connection entity
 * index or not.
 * 
 * \param mangled_ce_index Mangled connection entity index to be checked.
 *                         Refer #lmp_am2ce_mangle_acl_ce_index to find out
 *                         about mangling ACL connection entity index.
 * 
 * \return TRUE, if the \a mangled_ce_index is ACL connection entity index.
 *         FALSE, otherwise.
 */
#define lmp_am2ce_is_acl_ce_index(mangled_ce_index)                 \
    ((mangled_ce_index) < LMP_MAX_CE_DATABASE_ENTRIES)

/** 
 * Mangles ACL connection entity index such a way that it can be
 * differentiated from other connection entity indices.
 * 
 * \param acl_ce_index ACL connection entity index to be mangled.
 * 
 * \return Mangled ACL connection entiy index.
 */
#define lmp_am2ce_mangle_acl_ce_index(acl_ce_index) (acl_ce_index)

/** 
 * Demangles the given \a mangled_acl_ce_index.
 * 
 * \param mangled_acl_ce_index Mangled ACL connection enity index.
 * 
 * \return Demangled ACL connection entity index.
 */
#define lmp_am2ce_demangle_acl_ce_index(mangled_acl_ce_index)       \
    (mangled_acl_ce_index)

#ifdef COMPILE_ESCO
/** 
 * Checks whether the given \a mangled_ce_index is eSCO connection entity
 * index or not.
 * 
 * \param mangled_ce_index Mangled connection entity index to be checked.
 *                         Refer #lmp_am2ce_mangle_esco_ce_index to find out
 *                         about mangling eSCO connection entity index.
 * 
 * \return TRUE, if the \a mangled_ce_index is eSCO connection entity index.
 *         FALSE, otherwise.
 */
#define lmp_am2ce_is_esco_ce_index(mangled_ce_index)                \
    ((mangled_ce_index) >= LMP_MAX_CE_DATABASE_ENTRIES)

/** 
 * Mangles eSCO connection entity index such a way that it can be
 * differentiated from other connection entity indices.
 * 
 * \param esco_ce_index eSCO connection entity index to be mangled.
 * 
 * \return Mangled eSCO connection entiy index.
 */
#define lmp_am2ce_mangle_esco_ce_index(esco_ce_index)               \
    ((UINT16)((esco_ce_index)+LMP_MAX_CE_DATABASE_ENTRIES))

/** 
 * Demangles the given \a mangled_esco_ce_index.
 * 
 * \param mangled_esco_ce_index Mangled eSCO connection enity index.
 * 
 * \return Demangled eSCO connection entity index.
 */
#define lmp_am2ce_demangle_esco_ce_index(mangled_esco_ce_index)     \
    ((UINT16)((mangled_esco_ce_index)-LMP_MAX_CE_DATABASE_ENTRIES))
#endif /* COMPILE_ESCO */

void lmp_decide_and_update_max_slot_req(UINT16 ce_index);

#endif  /* __LMP_INTERNAL_H__ */

/** @} end: lmp_internal */
