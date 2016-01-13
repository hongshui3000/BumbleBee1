/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       btglib.c
* @brief     
* @details   
*
* @author   	gordon
* @date      	2015-06-25
* @version	v0.1
*/

#include <btglib.h>
#include <hci_api.h>
#include <l2c_api.h>
#include <sdp_api.h>
#include <gatt_api.h>
#include <upper_stack_global.h>

#define TRACE_MODULE_ID     MID_BT_L2C
/**
* @brief  send connect response to the queue set by hTargetQueue
* 
* @param  hTargetQueue:  target Queue ID
* @param  SrcTaskName: 
* @param  cid: 
* @param  channel
* @param  status: 
* @param  pExt
*
*/
void btgSendConResp(uint8_t hTargetQueue, uint16_t cid, uint16_t status, PBtConRespPSpecifc pExt)
{
    TBtConRespPSpecifc TExt;

    if (pExt == NULL)
    {
        memset(&TExt, 0, sizeof(TBtConRespPSpecifc));
        pExt = &TExt;
    }

    if (hTargetQueue == l2cQueueID)
    {
        l2cHandleL2C_CON_RESP(cid, status, pExt);
    }
    else if (hTargetQueue == gattQueueID)
    {
        gattHandleUpConnectResp(cid, status);
    }
}

/**
* @brief  send disconnect req to the queue set by hTargetQueue
* 
* @param  hTargetQueue:  target Queue ID
* @param  SrcTaskName: 
* @param  cid: 
* @param  channel
* @param  cause: 
* @param  holdLink
* @return  blueFaceStatus
*
*/
void btgSendDiscReq(uint8_t hTargetQueue, uint16_t cid, uint16_t cause, BOOL holdLink)
{
	if(hTargetQueue == hciQueueID) /**< send to hci queue*/
	{
		HCI_TRACE_PRINTF_2(HCI_TRACE_MASK_TRACE,"hciHandleUpDisconnectReq CID %x status %x", cid, cause);
		hciCommandDisconnect(cid, (uint8_t) cause);
		hciLaterEntry();
	}
    else if(hTargetQueue == l2cQueueID) /**< send l2cap queue*/
	{
		l2cHandleBTG_DISC_REQ(cid, holdLink);
	}
    else if(hTargetQueue == sdpQueueID) /**< send sdp queue*/
	{
		sdpHandleUpDiscReq(cid, holdLink);
	}
    else if(hTargetQueue == gattQueueID)
	{
		gattHandleUpDisconnectReq(cid, cause);
	}
}
