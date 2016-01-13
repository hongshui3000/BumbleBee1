/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

#ifndef __LMP_VENDOR_DEFINES_H__
#define __LMP_VENDOR_DEFINES_H__

#include "bt_fw_globals.h"
#include "hci_vendor_defines.h"
#include "le_ll.h"

#if defined(_LE_AUTO_REPORT_RSSI_AND_LOGIN_INOUT_FUNC_) && defined(LE_MODE_EN)
enum LE_WAKEON_CHECK_STATE_ {
    LE_WAKEON_STATE_PASS_IN_RSSI_LIST = 0,
    LE_WAKEON_STATE_NOT_IN_RSSI_LIST = 1,       
    LE_WAKEON_STATE_FAIL_IN_RSSI_LIST = 2,
};

#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_ 
#define LE_MSFT_MON_RSSI_MAX_SUPPORT_IRK_CNT        20
#define LE_MSFT_MON_RSSI_MAX_SUPPORT_BD_ADDR_CNT    20
#define LE_MSFT_MON_RSSI_MAX_SUPPORT_PATTERN_CNT    10
#define LE_MSFT_MON_RSSI_MAX_SUPPORT_UUID_CNT       20
#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_FINAL_SPEC_
#define LE_MSFT_MON_RSSI_MAX_SUPPORT_CONDITION_CNT  20
#endif

enum LE_MSFT_MONITOR_RSSI_FLAGS {
    LE_MSFT_MON_RSSI_FLAG_NONE = 0,
    LE_MSFT_MON_RSSI_FLAG_PATTERN_DATA = 1,
    LE_MSFT_MON_RSSI_FLAG_UUID = 2,
    LE_MSFT_MON_RSSI_FLAG_IRK = 3,
    LE_MSFT_MON_RSSI_FLAG_BDADDR = 4,      
} ;    

enum LE_MSFT_MONITOR_RSSI_VIA_CONN_HANDLE_MODE_SET {    
    LE_MSFT_MONITOR_RSSI_VIA_CONN_HANDLE_MODE_NONE = 0,
    LE_MSFT_MONITOR_RSSI_VIA_CONN_HANDLE_MODE_RTK = 1,
    LE_MSFT_MONITOR_RSSI_VIA_CONN_HANDLE_MODE_MSFT = 2,        
} ;    

enum LE_MSFT_MONITOR_RSSI_STATE_SET {  
    LE_MSFT_MONITOR_RSSI_STATE_INIT = 0,
    LE_MSFT_MONITOR_RSSI_STATE_OUT_RANGE = 1,
    LE_MSFT_MONITOR_RSSI_STATE_IN_RANGE = 2, 
} ;

typedef struct LL_MSFT_HANDLE_RSSI_PATTERN_UNIT_ {
#ifndef _SUPPORT_MSFT_BT_HCI_EXTENSION_FINAL_SPEC_
    UINT8 next_id;  
#endif
    UINT8 ad_type;
    UINT8 offset;
    UINT8 length;
    UINT8 data[29];
} LL_MSFT_HANDLE_RSSI_PATTERN_UNIT;

typedef struct LL_MSFT_HANDLE_RSSI_UUID_UNIT_ {
    UINT8 type;
    union {
        UINT8 data[16];
        UINT16 wdata[8];
        UINT32 dwdata[4];
    };
} LL_MSFT_HANDLE_RSSI_UUID_UNIT;

typedef struct LL_MSFT_HANDLE_RSSI_UUID_COMPARE_SET_ {
    UINT8 uuid_16_cnt;
    UINT8 uuid_32_cnt;
    UINT8 uuid_128_cnt;    
    UINT16 uuid_16[7];      /* max 7 set */
    UINT32 uuid_32[5];      /* max 5 set */
    UINT8  uuid_128_u1[16]; /* max 1 set */
} LL_MSFT_HANDLE_RSSI_UUID_COMPARE_SET;

typedef struct LL_MSFT_HANDLE_RSSI_BDADDR_UNIT_ {
    UINT8 type;
    UINT8 addr[6];
} LL_MSFT_HANDLE_RSSI_BDADDR_UNIT;
#endif

typedef struct LL_CONN_HANDLE_RSSI_UNIT_ {
    INT8 min_rssi_thres;
    INT8 max_rssi_thres;

    UINT16 rssi_count;
    INT32 rssi_sum;
    UINT8 addr_type;
    UINT8 addr[6];

#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_
    UINT8 notified:1;
    UINT8 start_periodic_timer:1;
    UINT8 rsvd:6;
    UINT8 sample_period;
    UINT8 sample_counter_of_tick;
    UINT16 min_rssi_thres_intv;
    UINT16 rssi_thres_counter;
    UINT16 conn_handle;
    UINT8 rssi_level;
    UINT8 last_sent_rssi_event;
    INT8 last_rssi;
#endif   
} LL_CONN_HANDLE_RSSI_UNIT;

#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_
typedef struct LL_ADV_RSSI_UNIT_ {
    UINT8 mon_state:2; 
    UINT8 notified:1;
    UINT8 start_periodic_timer:1;
#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_FINAL_SPEC_
    UINT8 condition_type:4;
#else
    UINT8 rsvd:4;
#endif
        
    INT8 min_rssi_thres;
    INT8 max_rssi_thres;
    INT8 last_rssi;
    UINT16 min_rssi_thres_intv;     
    UINT16 rssi_thres_counter; 

    UINT8 sample_period;
    UINT8 sample_counter_of_tick;    
    UINT16 pkt_cnts;
    INT32 pkt_rssi_sum;  

    LE_HW_ADVERTISING_CH_RX_PKT_S loc_adv;
} LL_ADV_RSSI_UNIT;
#endif

#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_FINAL_SPEC_
typedef struct MS_BT_HCI_EXT_ADV_CONDTYPE_UNIT_ {
    UINT8 next_id;
    
    union {
        LL_MSFT_HANDLE_RSSI_PATTERN_UNIT le_adv_pat;
        UINT8 le_adv_irk[16];
        LL_MSFT_HANDLE_RSSI_UUID_UNIT le_adv_uuid;
        LL_MSFT_HANDLE_RSSI_BDADDR_UNIT le_adv_bd_addr;
    };
    
} MS_BT_HCI_EXT_ADV_CONDTYPE_UNIT;
#endif

typedef struct LL_CONN_HANDLE_RSSI_MANAGER_ {
    UINT8 handle[8];
    UINT8 legacy_handle[16];
    LL_CONN_HANDLE_RSSI_UNIT entry[24];
    UINT16 bm_used_legacy_handle;
    UINT8 bm_used_handle;
    UINT8 max_indivadual_cnt;
    UINT8 max_bredr_cnt;

#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_ 
    UINT8 msft_monitor_rssi_mode;
    UINT8 le_adv_monttor_filter_enable;
    UINT8 le_adv_monitor_type; /* note: it is a bitmap */

    UINT8 le_adv_monitor_entry_count;
    UINT8 le_adv_free_monitor_head;    
    UINT8 le_adv_free_monitor_tail;
    UINT8 le_adv_free_monitor_unit_cnt;

#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_FINAL_SPEC_ 
    UINT32 bm_le_adv_monitor_entry[5]; /* [0] mean overall */
#else
    UINT32 bm_le_adv_monitor_entry;
#endif
    
#ifndef _SUPPORT_MSFT_BT_HCI_EXTENSION_FINAL_SPEC_    
    LL_ADV_RSSI_UNIT adv_entry[LE_MSFT_MON_RSSI_MAX_SUPPORT_IRK_CNT];
    union {
        LL_MSFT_HANDLE_RSSI_PATTERN_UNIT le_adv_pat[LE_MSFT_MON_RSSI_MAX_SUPPORT_PATTERN_CNT];
        UINT8 le_adv_irk[LE_MSFT_MON_RSSI_MAX_SUPPORT_IRK_CNT][16];
        LL_MSFT_HANDLE_RSSI_UUID_UNIT le_adv_uuid[LE_MSFT_MON_RSSI_MAX_SUPPORT_UUID_CNT];        
        LL_MSFT_HANDLE_RSSI_BDADDR_UNIT le_adv_bd_addr[LE_MSFT_MON_RSSI_MAX_SUPPORT_BD_ADDR_CNT];
    } ;
#else
    LL_ADV_RSSI_UNIT adv_entry[LE_MSFT_MON_RSSI_MAX_SUPPORT_CONDITION_CNT];
    MS_BT_HCI_EXT_ADV_CONDTYPE_UNIT ms_adv_unit[LE_MSFT_MON_RSSI_MAX_SUPPORT_CONDITION_CNT];
#endif    
#endif
} LL_CONN_HANDLE_RSSI_MANAGER;

extern LL_CONN_HANDLE_RSSI_MANAGER ll_rssi_manager;
void rssi_app_le_send_hci_vendor_rssi_event(UINT16 handle, INT8 rssi);
void rssi_app_le_bind_rssi_info(void);
UINT8 rssi_app_le_check_rssi_condition(UINT8 addr_type, UINT8 *rem_addr, INT8 rssi);
void rssi_app_vendor_check_and_send_rssi_event(
                                UINT8 mode, UINT8 index, 
                                UINT16 rssi_count, INT32 rssi_sum);

#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_
void rssi_app_msft_reset_le_adv_pattern(void);
UINT8 rssi_app_msft_get_rssi_monitor_entry(UINT8 type, UINT8 num_of_patterns);
void rssi_app_msft_free_rssi_monitor_entry(UINT8 type, UINT8 handle_id);
void rssi_app_msft_update_adv_rssi_info(UINT8 handle_id, UINT8 *pHdr, INT8 rssi);
void rssi_app_msft_update_conn_rssi_info(UINT8 handle_id, INT8 rssi);
UINT8 rssi_app_msft_check_rssi_monitor_condition(UINT8 *pHdr, INT8 rssi);
void rssi_app_msft_check_conn_thres_counter_then_send_event(UINT8 handle_id);
void rssi_app_msft_check_adv_thres_counter(UINT8 handle_id);
void rssi_app_msft_check_adv_counter_then_send_event(UINT8 handle_id);
void rssi_app_msft_check_all_counter_every_100ms(void);

void hci_generate_vs_msft_rssi_event(UINT8 status, UINT16 conn_handle, INT8 rssi);
void hci_generate_vs_msft_le_monitor_event(UINT8 addr_type,UINT8 * paddr,
                                      UINT16 mon_handle,UINT8 state);
#endif
#endif

#ifdef _SUPPORT_AUTO_DETACH_LINK_ 

/* the definition for the steta machine of auto detech function */
#define AUTO_DETACH_PROCESS_HCI_RESET_CMD      0
#define AUTO_DETACH_PROCESS_HW_RADIO_OFF       1
#define AUTO_DETACH_QUEUE_HCI_RESET_CMD        2
#ifdef CONFIG_TV_POWERON
#define AUTO_DETACH_TV_OFF_CMD                 3
#endif

extern TimerHandle_t auto_detach_link_timer;
extern UINT8 auto_detach_link_current_mode;
extern OS_SIGNAL catch_os_signal;
UINT8 auto_detach_check_all_link_removed(void);
UINT8 auto_detach_terminate_all_remote_links(void);
UINT8 auto_detach_pend_terminate_all_remote_links_from_isr(void);
void auto_detach_link_callback(TimerHandle_t timer_handle);
void auto_detach_enable_link_timer(UINT8 mode, UINT16 time_ms);
extern void lc_start_write_scan_mode(UCHAR scan_enable);
#endif

/*
 * Error injection start values in the packet. The logic is simple.
 * First 72 bits are the Channel Access Code.
 * Next 56 bits are the header, which is FEC 1/3 coded.
 * After 128 bits come the payload header, and actual payload.
 * 
 * REQUIREMENT FOR ERROR INSERTION:
 *
 * For introducing an FEC error 2/3, introduction of 
 * just 1 bit error is sufficient.
 *
 * For introducing FEC 1/3, CRC/HEC error, minimum of 3 CONSECUTIVE bits of
 * the error mask need to be set.
 */
#define		HEC_ERROR_START				285
#define		FEC_1_3_ERROR_START			310
#define		FEC_2_3_ERROR_START			250
#define		CRC_ERROR_START				250


/*
 * Default error mask that is programmed to error count register.
 * Note that these values are not cast in stone. They are just picked up.
 * The values can be easily changed to suit the needs.
 */
#define HEC_ERROR_MASK					0x0077
#define CRC_ERROR_MASK					0x0077
#define FEC_PACKET_HEADER_ERROR_MASK	0xf000
#define FEC_PAYLOAD_ERROR_MASK			0x0003
#define RANDOM_ERROR_MASK				0x0000	

/*
 * These values are used for inserting Header, Payload errors.
 * The values indicate the start location of the bits in the packet.
 */
#define PACKET_HEADER_START_INDEX	72   /* 72 bits */
#define	PACKET_PAYLOAD_START_INDEX	126  /* 126 bits */

#endif /* __LMP_VENDOR_DEFINES_H__ */
