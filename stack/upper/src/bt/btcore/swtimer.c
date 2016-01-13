/**
*****************************************************************
*  Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************
* @file  swtimer.c
* @brief  bt software module
* @details   
*
* @author  kyle
* @date  2015-10-29
* @version  v0.1
*/

#include <gatt_api.h>
#include <l2c_api.h>
#include <btsm_api.h>
#include <sdp_api.h>
#include <blueapi_def.h>
#include <upper_stack_global.h>
#include <swtimer.h>
#include <os_timer.h>

uint8_t swQueueID;
MESSAGE_T swMessage;

void swTimerCallBack(void * xTimer)
{
    MESSAGE_T Message;

    osGetTimerID(&xTimer, &Message.MData.Timer.QueueID,
        &Message.MData.Timer.TimerID, &Message.MData.Timer.TimerChannel);

    osMessageSend(swQueueID, &Message);

    osDeleteTimer(&xTimer);
}

void swTimerEntry(void  * dataSegment)
{
    if (osMessageReceive(swQueueID, &swMessage) == 0)
    {
        if(swMessage.MData.Timer.QueueID == blueAPIQueueID)
        {
            blueAPI_TimerTimeout(pBlueAPIData,
                (TBlueAPI_MCLTimerID)swMessage.MData.Timer.TimerID, swMessage.MData.Timer.TimerChannel);
        }
        else if(swMessage.MData.Timer.QueueID == btsmQueueID)
        {
            btsmHandleTimerExpired(swMessage.MData.Timer.TimerID, swMessage.MData.Timer.TimerChannel);
        }
        else if(swMessage.MData.Timer.QueueID == gattQueueID)
        {
            attTimerExpired(swMessage.MData.Timer.TimerID,
                (swMessage.MData.Timer.TimerChannel != 0) ? TRUE : FALSE);
        }
        else if(swMessage.MData.Timer.QueueID == l2cQueueID)
        {
            l2cHandleTIMER_EXPIRED(swMessage.MData.Timer.TimerID, swMessage.MData.Timer.TimerChannel);
        }
        else if(swMessage.MData.Timer.QueueID == sdpQueueID)
        {
            sdpHandleTimer((uint8_t)swMessage.MData.Timer.TimerChannel);
        }
    }
}

int swInit(void)
{
    if (osCoroutineCreate(&swQueueID, swTimerEntry, NULL))
    {
        return(FALSE);
    }

    return(TRUE);
}
