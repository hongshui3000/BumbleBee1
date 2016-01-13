/***************************************************************************
 Copyright (C) RealTek Ltd.
 This module is a confidential and proprietary property of RealTek and
 a possession or use of this module requires written permission of RealTek.
 ***************************************************************************/

#ifndef __PTA_H__
#define __PTA_H__

#include "platform.h"
#include "DataType.h"
#include "bt_fw_os.h"
#include "lc.h"

#ifdef _ENABLE_RTK_PTA_

//fw recorder maximum profile
#define MAX_RECORD_PROFILE              8

//profile index
#define HID_PROFILE                     1
#define A2DP_PROFILE                    2
#define FTP_PROFILE                     3
#define PAN_PROFILE                     4
#define HANDSFREE_PROFILE               5
#define HANDSET_PROFILE                 6

//role define
#define PTA_MASTER_ROLE                 1
#define PTA_SLAVE_ROLE                  2
#define PTA_SCANNET_ROLE                3

//define PTA profile recoder struct
typedef struct PTA_PROFILE_RECROD_S{
    UINT8   profile;
    UINT16  con_handle;
    UINT8   status;
    UINT32  max_transfer_time;
}PTA_PROFILE_RECROD_S_TYPE;

typedef struct PTA_MANAGE_S{
    UINT8 link_num;
    UINT8 reserved[3];    
    PTA_PROFILE_RECROD_S_TYPE  profile_info[MAX_RECORD_PROFILE];
}PTA_MANAGE_S_TYPE;

typedef union PTA_TDMA_PARA_S{
    UINT32 d32;
    struct
    {
        UINT32 bt_retry_index       :8;
        UINT32 bt_pkt_type          :8;
        UINT32 reserved_8_31        :16;
    }b;
}PTA_TDMA_PARA_S_TYPE;

typedef struct PTA_TDMA_MANAGE_S{
    UINT32 wlan_active_time;
    UINT32 bt_request_active_time;
    UINT32 bt_max_active_time;
    PTA_TDMA_PARA_S_TYPE tdma_para;   
}PTA_TDMA_MANAGE_S_TYPE;

#ifndef _DISABLE_OBSOLETE_MAILBOX_CMD_
extern PTA_MANAGE_S_TYPE pta_manage;
extern PTA_TDMA_MANAGE_S_TYPE pta_tdma_manage;
#endif
extern UINT32 pta_counter;
extern UINT8 tdma_state;
extern UINT8 traditional_tdma;

void pta_init(void);
UINT8 pta_search_exist_con_handle(UINT16 con_handle);
UINT8 pta_search_available_index();
void pta_profile_handle(UINT8 handle_index, UINT8 is_on);
void pta_get_bt_transfer_time();
void pta_execule_tdma_procedure();
void pta_notify_wifi();
void pta_tdma_control(UINT8 is_on);
BOOLEAN pta_profile_manage(UINT8 *parameter);
void pta_scan_bt_controller_role();
void pta_main_switch(UINT8 is_on);
void update_pta_manager();

#ifdef _RTK_PTA_TEST_
void pta_test_in_dbg_timer_callback(void);
void pta_test_in_main_func(void);
void pta_test_in_hci_reset_func(void);
#endif

#endif
#endif

