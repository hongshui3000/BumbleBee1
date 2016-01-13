/*
 *  Routines to access hardware
 *
 *  Copyright (c) 2014 Realtek Semiconductor Corp.
 *
 *  This module is a confidential and proprietary property of RealTek and
 *  possession or use of this module requires written permission of RealTek.
 */


#ifndef _APP_DFU_INT_H_
#define  _APP_DFU_INT_H_
#include "rtl_types.h"
#include "blueapi_types.h"


#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <timers.h>

#include "dfu_service.h"



/* DFU events */
enum
{
    /* these events are handled by the state machine */
    DFU_EVT_INIT_REG_UPPERSTACK = 0x00,
    DFU_EVT_UPPERSTACK_ACTIVE,
    DFU_EVT_DFU_SERVICE_REGISTERED,
    DFU_EVT_ADV_ENABLED,
    DFU_EVT_LINK_CONNECTED,
    DFU_EVT_NOTIFICATION_ENABLED,
    DFU_EVT_RX_FW_CP_REQ,
    DFU_EVT_RX_FW_CMPL,
    DFU_EVT_VALID_FW,
    DFU_EVT_ACTIVE_RESET,
    DFU_EVT_SYSTEM_RESET,
    DFU_EVT_LINK_DISCONNECTED,
    DFU_MAX_EVT
};
typedef UINT8 TDFU_EVENT;

/* DFU state */
enum
{
    DFU_ST_IDLE,
    DFU_ST_INITIALIZING_UPPERSTACK,
    DFU_ST_INITIALIZING_DFU_SERVICE,
    DFU_ST_INITIALIZING_ENABLE_ADV,
    DFU_ST_WAIT4_CON,
    DFU_ST_CONNECTED,
    DFU_ST_WAIT_FW,
    DFU_ST_FW_RX_CMPL,
    DFU_ST_FW_VALIDATE,
    DFU_ST_WAIT_FOR_ACTIVE_RESET,
    DFU_ST_MAX
};
typedef UINT8 TDFU_STATE;



typedef union
{
    UINT8 *pInitData;
} TDFU_INIT_DATA;

#define TIMER_ID_DFU_TOTAL                          1
#define TIMER_ID_DFU_WAIT4_CON                  2
#define TIMER_ID_DFU_IMAGE_TRANSFER         3

typedef struct
{

    xTaskHandle DfuAppTaskHandle;
    xQueueHandle dfu_QueueHandleEvent;
    xQueueHandle dfu_QueueHandleMessage;
    xTimerHandle  dfu_totalTimerHandle;
    UINT32           dfu_TimeOutValue_total;
    xTimerHandle  dfu_wait4_connTimerHandle;
    UINT32           dfu_TimeOutValue_wait4_conn;
    xTimerHandle  dfu_image_transferTimerHandle;
    UINT32           dfu_TimeOutValue_image_transfer;


    TDFU_STATE state;
    UINT8     RemoteBd[BLUE_API_BD_SIZE];        /* current remote BD */
    BOOL             RemoteBdValid;
    UINT16         local_MDL_ID;
    BOOL local_MDL_ID_Valid;
    UINT16         wDsPoolId;       /* buffer pool id for connection related  */
    UINT16         wDsDataOffset;   /* data offset in downstream data buffer  */
    UINT16         wDsCredits;      /* downstream (tx) credits for WriteCommand */
    /* and Notifications                        */
    UINT16         wMTUSize;        /* ATT layer MTU size */

    void * pServiceHandle;
    void * pRequestHandle;
    TGATTAttrIdxCCCD AttrIdxCCCD;

    UINT32 nMaxKernelImageSizeSupport;
    UINT32 nMaxUserImageSizeSupport;
    UINT32 nCurOffSet;
    UINT32 nImageTotalLength;

    //

    UINT16 nOrigKernelImageVersion;
    UINT16 nOrigUserImageVersion;
    
    //parameter for start dfu procedure
    UINT16 nOffSet;
    UINT16 nSignature;
    UINT16 nImageVersion;
    UINT16 nCRC16;
    UINT16 nImageLength;
    UINT16 nReserved;

    UINT16 nTotalErrorCount;

    UINT8   nCurNotificationAct;


} TDFU_CB;

typedef void (*TDFU_ACT)(TDFU_CB* pDfuCb, TDFU_INIT_DATA *pDfuInitdata);

const char * dfu_get_event_name(TDFU_EVENT dfu_evt);
const char * dfu_get_state_name(TDFU_STATE dfu_state);
void dfu_sm_event(TDFU_CB *pDfuCb, TDFU_EVENT event, void*pData);


void dfu_BlueAPICallback(PBlueAPI_UsMessage pMsg);

#endif
