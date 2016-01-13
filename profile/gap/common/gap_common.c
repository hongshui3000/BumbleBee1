enum { __FILE_NUM__ = 0 };
/**
*****************************************************************************************
*     Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
  * @file    gap_common.c
  * @brief   This file provides the GAP role common functions, used by all GAP roles.
  * @details
  * @author  Ethan
  * @date    11-Aug-2015
  * @version v0.1
  ******************************************************************************
  * @attention
  * <h2><center>&copy; COPYRIGHT 2015 Realtek Semiconductor Corporation</center></h2>
  ******************************************************************************
  */

/** Add Includes here **/
#include <string.h>
#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include <blueapi.h>
#include <trace.h>
#include <gap.h>
#include <bee_message.h>
#include <common_defs.h>

extern xQueueHandle hEventQueueHandle;
extern xQueueHandle hMessageQueueHandle;
extern xQueueHandle hIoQueueHandle;
TBlueAPIAppHandle tAppHandle;

/**
  * @brief  Callback function should be register to upper stack to send message to application.
  * @param  pMsg - message sent from upper stack.
  * @retval none
  */
void GAP_BlueAPIMessageCB(PBlueAPI_UsMessage pmsg)
{
    unsigned char event = BLUEAPI_MSG_EVENT;

    if (xQueueSend(hMessageQueueHandle, &pmsg, 0) == errQUEUE_FULL)
    {
        blueAPI_BufferRelease(pmsg);
    }
    else if (xQueueSend(hEventQueueHandle, &event, 0) == errQUEUE_FULL)
    {

    }
}

/******************************************************************
 * @fn          GAP_SendBtMsgToApp
 * @brief      send BEE_IO_MSG to app task.            
 * @param    pBeeMsgBlk  - pointer to BEE_IO_MSG message
 *
 * @return     void
 */
void GAP_SendBtMsgToApp(BEE_IO_MSG *pmsg)
{
    portBASE_TYPE result;
    uint8_t event = 0;

    result = xQueueSend(hIoQueueHandle, pmsg, 0xFFFF);
    if (result != pdPASS)
    {
        DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "GAP_SendBtMsgToApp fail2", 1, result);
    }

    event = EVENT_NEWIODRIVER_TO_APP;
    result = xQueueSend(hEventQueueHandle, &event, 0xFFFF);
    if (result != pdPASS)
    {
        DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "GAP_SendBtMsgToApp fail", 1, result);
    }
}

/******************************************************************
 * @fn          peripheralStateNotificationCB
 * @brief      callback to send state change message to app task.      
 * @param    newState
 *
 * @return     void
 */
void GAP_StateNotificationCB(gaprole_States_t new_state)
{
    BEE_IO_MSG bee_io_msg;
    BT_STACK_MSG bt_gap_msg;

#ifdef GAP_DEBUG_FLAG
    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "GAP_StateNotificationCB newState=0x%x", 1, new_state);
#endif

    bee_io_msg.IoType = BT_STATUS_UPDATE;
    bee_io_msg.subType = BT_MSG_TYPE_CONN_STATE_CHANGE;
    bt_gap_msg.msgData.gapConnStateChange.newState = new_state;

    memcpy(&bee_io_msg.parm, &bt_gap_msg, sizeof(bee_io_msg.parm));

    GAP_SendBtMsgToApp(&bee_io_msg);
}

/**
  * @brief  Make upper stack ready to use, by sending the register request.
  * @retval TRUE - start successful.
  *         FALSE - start failed.
  */
bool GAP_StartBtStack(void)
{
    bool Result = true;
    if (
        blueAPI_RegisterReq( (void *)&tAppHandle,
                             (void *)GAP_BlueAPIMessageCB)
    )
    {
#ifdef GAP_DEBUG_FLAG
        DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "GAP_StartBtStack", 0);
#endif
    }
    else
    {
        Result = false;
    }

    return( Result );
}

#if 0
bool blueAPI_ReadRSSI(uint16_t local_MDL_ID)
{
    TApiBufUserDefined ApiBufUserDefined;
    ApiBufUserDefined.Type = API_TYPE_READ_RSSI_CMD;
    ApiBufUserDefined.p.ApiBufVendorCmd.opCode = 0x1405;
    ApiBufUserDefined.p.ApiBufVendorCmd.len = 2;
    LE_WORD2EXTRN(ApiBufUserDefined.p.ApiBufVendorCmd.para, local_MDL_ID);
    return blueAPI_UserDefined(&ApiBufUserDefined);
}

bool blueAPI_ReadPatchVersion(uint16_t * PatchVersion)
{
    TApiBufUserDefined ApiBufUserDefined;
    ApiBufUserDefined.Type = API_TYPE_READ_PATCH_VERSION;
    ApiBufUserDefined.p.pPatchVersion = PatchVersion;
    return blueAPI_UserDefined(&ApiBufUserDefined);     
}
#endif

