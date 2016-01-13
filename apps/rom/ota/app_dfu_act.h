/*
 *  Routines to access hardware
 *
 *  Copyright (c) 2014 Realtek Semiconductor Corp.
 *
 *  This module is a confidential and proprietary property of RealTek and
 *  possession or use of this module requires written permission of RealTek.
 */


#ifndef _APP_DFU_ACT_H_
#define  _APP_DFU_ACT_H_

#include "rtl_types.h"
#include "app_dfu_int.h"


enum
{
    DFU_ACT_INIT_UPPERSTACK,
    DFU_ACT_INIT_DFU_SERVICE,
    DFU_ACT_INIT_ENABLE_ADV,
    DFU_ACT_PROCESS_ADV_ENABLE_EVT,
    DFU_ACT_PROCESS_LINK_CONNECTED_EVT,
    DFU_ACT_PROCESS_NOTIFICATION_ENABLE_EVT,
    DFU_ACT_NOTIFY_RX_FW_CP_REQ,
    DFU_ACT_NOTIFY_FW_RX_CMPL,
    DFU_ACT_NOTIFY_VALID,
    DFU_ACT_RESET_AND_ACTIVATE,
    DFU_ACT_SYSTEM_RESET,
    DFU_SM_NO_ACTION
};

#define     DFU_SUBACT_RX_FW_CP_REQ_REPORT_TARGET_INFO 0x01
#define     DFU_SUBACT_RX_FW_CP_REQ_START_DFU                    0x02



void dfu_act_process_evt_init_upperstack(TDFU_CB *pDfuCb,
        TDFU_INIT_DATA *pDfuData);


void dfu_act_process_evt_upperstack_active(TDFU_CB *pDfuCb,
        TDFU_INIT_DATA *pDfuData);


void dfu_act_process_evt_dfu_service_registered(TDFU_CB *pDfuCb,
        TDFU_INIT_DATA *pDfuData);



void    dfu_act_process_evt_adv_enabled(TDFU_CB *pDfuCb,
                                        TDFU_INIT_DATA *pDfuData);


void dfu_act_process_evt_link_connected(TDFU_CB *pDfuCb,
                                        TDFU_INIT_DATA *pDfuData);


void dfu_act_process_evt_enable_notification(TDFU_CB *pDfuCb,
        TDFU_INIT_DATA *pDfuData);



void dfu_act_notify_rx_cp_req(TDFU_CB *pDfuCb,
                              TDFU_INIT_DATA *pDfuData);

void dfu_act_notify_report_target_image_information(TDFU_CB *pDfuCb,
        TDFU_INIT_DATA *pDfuData);

void dfu_act_notify_start_dfu(TDFU_CB *pDfuCb,
                              TDFU_INIT_DATA *pDfuData);



void dfu_act_notify_fw_rx_cmpl(TDFU_CB *pDfuCb,
                               TDFU_INIT_DATA *pDfuData);

void dfu_act_notify_valid(TDFU_CB *pDfuCb,
                          TDFU_INIT_DATA *pDfuData);

void dfu_act_reset_and_activate(TDFU_CB *pDfuCb,
                                TDFU_INIT_DATA *pDfuData);

void    dfu_act_system_reset(TDFU_CB *pDfuCb,
                             TDFU_INIT_DATA *pDfuData);

void dfu_act_send_notification(TDFU_CB *pDfuCb,
                               uint8_t* pData,
                               int dataLen);

#endif
