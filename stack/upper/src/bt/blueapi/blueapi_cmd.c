/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       blueapi_cmd.c
* @brief     bluetooth api for send event to app
* @details   
*
* @author   gordon
* @date      2015-07-08
* @version  v0.1
*/

#include <flags.h>
#include <rtl_types.h>
#include <blueface.h>
#include <btglbdef.h>
#include <bt_api.h>
#include <blueapi_types.h>
#include <os_pool.h>
#include <sdp_code.h>
#include <blueapi_api.h>
#include <blueapi_def.h>
#include <blueapi_osif.h>
#include <l2c_api.h>
#include <gatt_api.h>
#include <btsm_api.h>
#include <hci_api.h>
#include <sdp_api.h>
#include <btsend.h>
#include <hci_api.h>
#include <btglib.h>
#include <upper_stack_global.h>

/* File Id for COM_InternalEventInfo */
#define BLUE_API_SOURCE_FILE_ID     0x01
extern const char pBlueAPIDumpPrefix[];
#define TRACE_MODULE_ID  MID_BT_HDP

/**
 * @brief  send event to app
 *
 * @param  pBlueAPIdata:
 * @param  pOldMsg: old message, we can reuse it(avoid release and alloc again)
 * @param  pMsg:  need to send msg
 *
 * @return  
 *
 */
BOOL blueAPI_TgtSendEvent(PBlueAPI_App pBlueAPIApp, PBlueAPI_DsMessage pOldMsg, PBlueAPI_UsMessage pMsg)
{
    uint16_t length;
    PBlueAPI_UsMessage pBLUE_APIMsg;

    if (pOldMsg != NULL)
    {  
        osBufferRelease(pOldMsg);
        pOldMsg = NULL;
    } 

    if (pBlueAPIApp == NULL || !pBlueAPIApp->used)
    {
        BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE,
            "!!blueAPI_TgtSendEvent: no app registered for cmd[0x%X]", pMsg->Command);

        return TRUE;
    }

    if ((pMsg->Command == blueAPI_EventGATTAttributeWriteCommandInfo)
        || (pMsg->Command == blueAPI_EventGATTAttributeUpdateRsp)
        || (pMsg->Command == blueAPI_EventGATTAttributeWriteInd)
        || (pMsg->Command == blueAPI_EventGATTDiscoveryInd)
        || (pMsg->Command == blueAPI_EventGATTAttributeReadRsp)
        || (pMsg->Command == blueAPI_EventGATTAttributeInd)
        || (pMsg->Command == blueAPI_EventGATTAttributeNotificationInfo)
        || (pMsg->Command == blueAPI_EventGATTCCCDInfo)
        || (pMsg->Command == blueAPI_EventGATTAttributePrepareWriteInd)
        || (pMsg->Command == blueAPI_EventGATTAttributePrepareWriteRsp)
        || (pMsg->Command == blueAPI_EventLEDataInd))
    {
        /* data is already in osBuffer */
        pBLUE_APIMsg = pMsg;
        length = pMsg->Length;
    }
    else
    {
        /* alloc new osBuffer */
        if (osBufferGet(pBlueAPIData->usBlueAPIPoolID, sizeof(TBlueAPI_UsMessage),(PVOID *)&pBLUE_APIMsg))
        {
            BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_ERROR,
                "!!blueAPI_TgtSendEvent: failed to get buffer for cmd[0x%X]", pMsg->Command);

            DebuggerBreak();
            return TRUE;    /* error */
        }

        length = sizeof(TBlueAPI_UsMessage);
        memcpy(pBLUE_APIMsg, pMsg, length);
    }

    pBLUE_APIMsg->Length = length;

    BLUEAPI_TRACE_BINARY_UPSTREAM(TRACE_PKT_TYPE_BLUEAPI_OSIF_US, length, (uint8_t *)pBLUE_APIMsg);

    if (pBlueAPIApp->MDHmsgHandlerCallback != (PVOID)0)
    {
        PBlueAPI_CallBack MDHmsgHandlerCallback = (PBlueAPI_CallBack)pBlueAPIApp->MDHmsgHandlerCallback;

        MDHmsgHandlerCallback(pBLUE_APIMsg);
    }

    return FALSE;
}

/**
 * @brief  send pairable mode rsp to app
 *
 * @param  pReqMsg
 * @param  cause
 *
 * @return  
 *
 */
void blueAPI_Send_PairableModeSetRsp(PBlueAPI_DsMessage pReqMsg, TBlueAPI_Cause cause)
{
    TBlueAPI_UsMessage msg;

    msg.Command = blueAPI_EventPairableModeSetRsp;

    msg.p.PairableModeSetRsp.cause = cause;

    BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE, "blueAPI:>>> PairableModeSetRsp cause[%d]",
        msg.p.PairableModeSetRsp.cause);

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, pReqMsg, &msg);
}

void blueAPI_Send_UserAuthorizationReqInd(uint8_t * remote_BD, uint8_t outgoing, uint16_t psm, uint16_t server_channel, uint16_t uuid)
{
    TBlueAPI_UsMessage msg;

    msg.Command = blueAPI_EventUserAuthorizationReqInd;

    msg.p.UserAuthorizationReqInd.outgoing          = outgoing;
    msg.p.UserAuthorizationReqInd.psm               = psm;
    msg.p.UserAuthorizationReqInd.server_channel    = server_channel;
    msg.p.UserAuthorizationReqInd.uuid              = uuid;

    memcpy(msg.p.UserAuthorizationReqInd.remote_BD, remote_BD, 6);

    BLUEAPI_TRACE_PRINTF_5(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:>>> UserAuthorizationReqInd outgoing[%d] psm[0x%x] server_channel[%d] uuid[0x%x] BD[%s]",
        outgoing, psm, server_channel, uuid, BLUEAPI_TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE, remote_BD));

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

/**
 * @brief  sdp auth response
 *
 * @param  pReqMsg
 * @param  remote_BD
 * @param  cause
 *
 * @return  
 *
 */
void blueAPI_Send_AuthRsp(PBlueAPI_DsMessage pReqMsg, uint8_t * remote_BD, TBlueAPI_Cause cause)
{
    TBlueAPI_UsMessage msg;

    msg.Command = blueAPI_EventAuthRsp;
    memcpy(msg.p.AuthRsp.remote_BD, remote_BD, 6);
    msg.p.AuthRsp.cause = cause;

    BLUEAPI_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_TRACE,
                             "blueAPI:>>> AuthRsp BD[%s] cause[%d]",
                             TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE, msg.p.AuthRsp.remote_BD),
                             msg.p.AuthRsp.cause
                             );

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, pReqMsg, &msg);
}

void blueAPI_Send_UserAuthRequestInd(uint8_t * remote_BD)
{
    TBlueAPI_UsMessage msg;

    msg.Command = blueAPI_EventUserAuthRequestInd;
    memcpy(msg.p.UserAuthRequestInd.remote_BD, remote_BD, BD_ADDR_SIZE);

    BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE, "blueAPI:>>> UserAuthRequestInd BD[%s]",
        BLUEAPI_TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE, msg.p.UserAuthRequestInd.remote_BD));

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

/**
 * @brief    blueapi send auth result request indicate
 *
 * @param  remote_BD
 * @param  remote_BD_Type
 * @param  keyType
 * @param  restartHandle
 *
 * @return
 *
 */
void blueAPI_Send_AuthResultRequestInd(uint8_t *remote_BD,
    TBlueAPI_RemoteBDType remote_BD_Type, TBlueAPI_LinkKeyType keyType, uint16_t restartHandle)
{
    TBlueAPI_UsMessage msg;

    msg.Command = blueAPI_EventAuthResultRequestInd;
    memcpy(msg.p.AuthResultRequestInd.remote_BD, remote_BD, BD_ADDR_SIZE);
    msg.p.AuthResultRequestInd.remote_BD_Type  = remote_BD_Type;
    msg.p.AuthResultRequestInd.keyType         = keyType;
    msg.p.AuthResultRequestInd.restartHandle   = restartHandle;

    BLUEAPI_TRACE_PRINTF_4(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:>>> AuthResultRequestInd BD[%s] BDType[%d] keyType[%d] restart[0x%x]",
        TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE,msg.p.AuthResultRequestInd.remote_BD),
        msg.p.AuthResultRequestInd.remote_BD_Type, msg.p.AuthResultRequestInd.keyType,
        msg.p.AuthResultRequestInd.restartHandle);

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

/**
 * @brief  blueapi send user passkey request request response
 *
 * @param  cause
 *
 * @return  
 *
 */
void blueAPI_Send_UserPasskeyReqReplyRsp(TBlueAPI_Cause cause)
{
    TBlueAPI_UsMessage msg;

    msg.Command = blueAPI_EventUserPasskeyReqReplyRsp;
    msg.p.UserPasskeyReqReplyRsp.cause = cause;

    BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE, "blueAPI:>>> UserPasskeyReqReplyRsp cause[%d]",
        msg.p.UserPasskeyReqReplyRsp.cause);

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

void blueAPI_Send_UserConfirmationReqInd(uint8_t * remote_BD, uint32_t displayValue)
{
    TBlueAPI_UsMessage msg;
    
    msg.Command = blueAPI_EventUserConfirmationReqInd;
    memcpy(msg.p.UserConfirmationReqInd.remote_BD,remote_BD,6);
    msg.p.UserConfirmationReqInd.displayValue = displayValue;

    BLUEAPI_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:>>> UserConfirmationReqInd BD[%s] displayValue[%d]",
        BLUEAPI_TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE, msg.p.UserConfirmationReqInd.remote_BD),
        msg.p.UserConfirmationReqInd.displayValue);

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

void blueAPI_Send_UserPasskeyReqInd(uint8_t * remote_BD)
{
    TBlueAPI_UsMessage msg;

    msg.Command = blueAPI_EventUserPasskeyReqInd;

    memcpy(msg.p.UserPasskeyReqInd.remote_BD, remote_BD, 6);

    BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE, "blueAPI:>>> UserPasskeyReqInd BD[%s]",
        BLUEAPI_TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE, msg.p.UserPasskeyReqInd.remote_BD));

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

/**
 * @brief  blueapi send blueAPI_EventKeypressNotificationRsp to app
 *
 * @param  cause
 *
 * @return  
 *
 */
void blueAPI_Send_KeypressNotificationRsp(TBlueAPI_Cause cause)
{
    TBlueAPI_UsMessage msg;

    msg.Command = blueAPI_EventKeypressNotificationRsp;
    msg.p.KeypressNotificationRsp.cause = cause;

    BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE, "blueAPI:>>> KeypressNotificationRsp cause[%d]",
        msg.p.KeypressNotificationRsp.cause);

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

void blueAPI_Send_UserPasskeyNotificationInfo(uint8_t *remote_BD, uint32_t displayValue)
{
    TBlueAPI_UsMessage msg;

    msg.Command = blueAPI_EventUserPasskeyNotificationInfo;

    memcpy(msg.p.UserPasskeyNotificationInfo.remote_BD, remote_BD, 6);
    msg.p.UserPasskeyNotificationInfo.displayValue = displayValue;

    BLUEAPI_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:>>> UserPasskeyNotificationInfo BD[%s] displayValue[%d]",
        BLUEAPI_TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE, msg.p.UserPasskeyNotificationInfo.remote_BD),
        msg.p.UserPasskeyNotificationInfo.displayValue);

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

void blueAPI_Send_KeypressNotificationInfo(uint8_t *remote_BD, uint8_t eventType)
{
    TBlueAPI_UsMessage msg;

    msg.Command = blueAPI_EventKeypressNotificationInfo;

    memcpy(msg.p.KeypressNotificationInfo.remote_BD, remote_BD, 6);
    msg.p.KeypressNotificationInfo.eventType = (TBlueAPI_SSPKeyEvent)eventType;

    BLUEAPI_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:>>> KeypressNotificationInfo BD[%s] eventType[%d]",
        BLUEAPI_TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE, msg.p.KeypressNotificationInfo.remote_BD),
        msg.p.KeypressNotificationInfo.eventType);

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

/**
 * @brief  send remote oob data request to app
 *
 * @param  remote_BD: bdaddr
 *
 * @return  
 *
 */
void blueAPI_Send_RemoteOOBDataReqInd(uint8_t *remote_BD)
{
    TBlueAPI_UsMessage msg;

    msg.Command = blueAPI_EventRemoteOOBDataReqInd;

    memcpy(msg.p.RemoteOOBDataReqInd.remote_BD, remote_BD, 6);

    BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE, "blueAPI:>>> RemoteOOBDataReqInd BD[%s]",
        TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE,msg.p.RemoteOOBDataReqInd.remote_BD));

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

void blueAPI_Send_LegacyRemoteOOBDataReqInd(uint8_t *remote_BD)
    {
    TBlueAPI_UsMessage msg;

    msg.Command = blueAPI_EventLegacyRemoteOOBDataReqInd;

    memcpy(msg.p.LegacyRemoteOOBDataReqInd.remote_BD, remote_BD, 6);

    BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE, "blueAPI:>>> LegacyRemoteOOBDataReqInd BD[%s]",
        TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE, msg.p.RemoteOOBDataReqInd.remote_BD));

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}
/**
 * @brief  blueapi send read local oob data response to app
 *
 * @param  cause
 * @param  pC
 * @param  pR
 *
 * @return
 *
 */
void blueAPI_Send_LocalOOBDataRsp(TBlueAPI_Cause cause, uint8_t *pC, uint8_t *pR)
{
    TBlueAPI_UsMessage msg;

    msg.Command = blueAPI_EventLocalOOBDataRsp;
    msg.p.LocalOOBDataRsp.cause = cause;
    memcpy(&msg.p.LocalOOBDataRsp.C[0], pC, 16);
    memcpy(&msg.p.LocalOOBDataRsp.R[0], pR, 16);

    BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE, "blueAPI:>>> LocalOOBDataRsp cause[%d]", cause);

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

/**
 * @brief   blueapi send auth result indicator
 *
 * @param  remote_BD
 * @param  remote_BD_Type
 * @param  linkKeyLength
 * @param  linkKey
 * @param  keyType
 * @param  cause
 *
 * @return
 *
 */
void blueAPI_Send_AuthResultInd(uint8_t * remote_BD, TBlueAPI_RemoteBDType remote_BD_Type,
    uint8_t linkKeyLength, uint8_t *linkKey, TBlueAPI_LinkKeyType keyType,TBlueAPI_Cause cause)
{
    TBlueAPI_UsMessage msg;

    msg.Command = blueAPI_EventAuthResultInd;

    memcpy(msg.p.AuthResultInd.remote_BD, remote_BD, 6);
    msg.p.AuthResultInd.remote_BD_Type = remote_BD_Type;
    msg.p.AuthResultInd.linkKeyLength  = linkKeyLength;

    if (linkKey != NULL)
    {  
        memcpy(msg.p.AuthResultInd.linkKey, linkKey, linkKeyLength);
    }
    else
    {  
        memset(msg.p.AuthResultInd.linkKey, 0x00, linkKeyLength);
    }

    msg.p.AuthResultInd.keyType = keyType;
    msg.p.AuthResultInd.cause   = cause;

    BLUEAPI_TRACE_PRINTF_5(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:>>> AuthResultInd BD[%s] BDType[%d] keyLen[%d] keyType[0x%X] cause[%d]",
        TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE, msg.p.AuthResultInd.remote_BD),
        msg.p.AuthResultInd.remote_BD_Type, msg.p.AuthResultInd.linkKeyLength,
        msg.p.AuthResultInd.keyType, msg.p.AuthResultInd.cause);

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

/****************************************************************************/
/* void blueAPI_Send_AuthDeleteRsp                                          */
/****************************************************************************/
void blueAPI_Send_AuthDeleteRsp(PBlueAPI_DsMessage pReqMsg, uint8_t * remote_BD,
    TBlueAPI_RemoteBDType remote_BD_Type, TBlueAPI_Cause cause)
{
    TBlueAPI_UsMessage msg;

    msg.Command = blueAPI_EventAuthDeleteRsp;

    memcpy(msg.p.AuthDeleteRsp.remote_BD, remote_BD, 6);
    msg.p.AuthDeleteRsp.remote_BD_Type = remote_BD_Type;
    msg.p.AuthDeleteRsp.cause          = cause;

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, pReqMsg, &msg);
}

/****************************************************************************/
/* void blueAPI_Send_AuthListRsp                                            */
/****************************************************************************/
void blueAPI_Send_AuthListRsp(uint8_t *remote_BD, TBlueAPI_RemoteBDType remote_BD_Type,
    TBlueAPI_Cause cause)
{
    TBlueAPI_UsMessage msg;

    msg.Command = blueAPI_EventAuthListRsp;

    memcpy(msg.p.AuthListRsp.remote_BD, remote_BD, 6);
    msg.p.AuthListRsp.remote_BD_Type = remote_BD_Type;
    msg.p.AuthListRsp.cause          = cause;

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

/**
 * @brief    blueapi send auth list info to app
 *
 * @param    remote_BD:
 * @param    remote_BD_Type
 * @param    keyType
 * @param    AppData
 * @param    Remote_DeviceName
 *
 * @return
 *
 */
void blueAPI_Send_AuthListInfo(uint8_t *remote_BD, TBlueAPI_RemoteBDType remote_BD_Type,
    TBlueAPI_LinkKeyType keyType, uint32_t AppData, uint8_t * Remote_DeviceName)
{
    TBlueAPI_UsMessage msg;

    msg.Command = blueAPI_EventAuthListInfo;

    memcpy(msg.p.AuthListInfo.remote_BD, remote_BD, 6);
    msg.p.AuthListInfo.remote_BD_Type  = remote_BD_Type;
    msg.p.AuthListInfo.keyType         = keyType;
    msg.p.AuthListInfo.AppData         = AppData;
    memcpy(msg.p.AuthListInfo.Remote_DeviceName, Remote_DeviceName, BLUE_API_DEVICE_NAME_LENGTH);

    BLUEAPI_TRACE_PRINTF_5(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:>>> AuthListInfo BD[%s] BDType[%d] keyType[%d] AppData[0x%X] Name[%s]",
        TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE, msg.p.AuthListInfo.remote_BD),
        msg.p.AuthListInfo.remote_BD_Type, msg.p.AuthListInfo.keyType, msg.p.AuthListInfo.AppData,
        BTRACE_RAMDATA1(BLUEAPI_TRACE_MASK_TRACE, msg.p.AuthListInfo.Remote_DeviceName));

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

/**
 * @brief  send device config set rsp
 *
 * @param  pReqMsg:  req msg, may used again
 * @param  opCode
 * @param  cause: request result
 *
 * @return  
 *
 */
void blueAPI_Send_DeviceConfigSetRsp(PBlueAPI_DsMessage pReqMsg,
    TBlueAPI_DeviceConfigOpcode opCode, TBlueAPI_Cause cause)
{
    TBlueAPI_UsMessage msg;

    BLUEAPI_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:>>> DeviceConfigSetRsp opCode[%d] cause[%d]", opCode, cause);

    msg.Command                     = blueAPI_EventDeviceConfigSetRsp;
    msg.p.DeviceConfigSetRsp.opCode = opCode;
    msg.p.DeviceConfigSetRsp.cause  = cause;

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, pReqMsg, &msg);
}

/**
 * @brief  send acl config response to app
 *
 * @param  pReqMsg:
 * @param  remote_BD
 * @param  remote_BD_Type 
 * @param  opCode
 * @param  cause
 *
 * @return  
 *
 */
void blueAPI_Send_ACLConfigRsp(PBlueAPI_DsMessage pReqMsg, uint8_t *remote_BD,
    TBlueAPI_RemoteBDType remote_BD_Type, TBlueAPI_ACLConfigOpcode opCode, TBlueAPI_Cause cause)
{
    TBlueAPI_UsMessage msg;

    BLUEAPI_TRACE_PRINTF_4(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:>>> ACLConfigRsp BD[%s] BDtype[%d] opCode[%d] cause[%d]",
        TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE, remote_BD), remote_BD_Type, opCode, cause);

    msg.Command = blueAPI_EventACLConfigRsp;

    memcpy(msg.p.ACLConfigRsp.remote_BD, remote_BD, BLUE_API_BD_SIZE);
    msg.p.ACLConfigRsp.remote_BD_Type = remote_BD_Type;
    msg.p.ACLConfigRsp.opCode         = opCode;
    msg.p.ACLConfigRsp.cause          = cause;

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, pReqMsg, &msg);
}

/**
 * @brief  send connect mdl info
 *
 * @param  pLinkContext
 *
 * @return  
 *
 */
void blueAPI_Send_ConnectMDLInfo(PBlueAPI_LinkDescriptor pLinkContext)
{
    TBlueAPI_UsMessage msg;

    msg.Command                           = blueAPI_EventConnectMDLInfo;
    msg.p.ConnectMDLInfo.local_MDL_ID     = pLinkContext->pMDL->local_MDL_ID;
    msg.p.ConnectMDLInfo.dsPoolID         = pLinkContext->dsPoolID;
    msg.p.ConnectMDLInfo.dsDataOffset     = BT_DS_WRITE_OFFSET_COUNT;
    msg.p.ConnectMDLInfo.maxTPDUSize      = pLinkContext->pMDL->maxTPDUSize;
    msg.p.ConnectMDLInfo.maxTPDUdsCredits = pLinkContext->pMDL->dsCredits;
    msg.p.ConnectMDLInfo.linkConfigType   = pLinkContext->linkConfigType;

    /* if credit based flowctrl was not requested, don't send internal values upstream */
    if (pLinkContext->pMDL->maxUsCredits == 0)
    {
        msg.p.ConnectMDLInfo.maxTPDUdsCredits = 0;
    }

    blueAPI_MDLSubFlags(pLinkContext->pMDL, BLUE_API_MDL_FLAG_NOT_ACTIVATED);

    BLUEAPI_TRACE_PRINTF_6(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:>>> ConnectMDLInfo local_MDL[%d] poolID[%d] offset[%d] maxTPDU[%d] confType[%d] usCred[%d] dsCred[%d]",
        pLinkContext->pMDL->local_MDL_ID, pLinkContext->dsPoolID, BT_DS_WRITE_OFFSET_COUNT,
        msg.p.ConnectMDLInfo.maxTPDUSize, pLinkContext->linkConfigType, msg.p.ConnectMDLInfo.maxTPDUdsCredits);

    pLinkContext->MDLConnected = TRUE;
    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

/**
 * @brief  send delete MDL info to app
 *
 * @param  pMDL
 *
 * @return
 *
 */
void blueAPI_Send_DeleteMDLInfo(PBlueAPI_MDL pMDL)
{
    TBlueAPI_UsMessage msg;

    msg.Command                      = blueAPI_EventDeleteMDLInfo;
    msg.p.DeleteMDLInfo.local_MDL_ID = pMDL->local_MDL_ID;

    BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE, "blueAPI:>>> DeleteMDLInfo local_MDL[%d]",
        pMDL->local_MDL_ID);

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

#if SEND_MCL_STATUS_INFO
/**
 * @brief   send mcl status info
 *
 * @param  pBD
 * @param  localMCL_ID
 * @param  status
 *
 * @return  point to comapp
 *
 */
void blueAPI_Send_MCLStatusInfo(uint8_t *pBD, uint16_t localMCL_ID,
    TBlueAPI_MCLStatus status)
{
    TBlueAPI_UsMessage msg;

    msg.Command                      = blueAPI_EventMCLStatusInfo;
    msg.p.MCLStatusInfo.local_MCL_ID = localMCL_ID;
    msg.p.MCLStatusInfo.status       = status;

    memcpy(msg.p.MCLStatusInfo.remote_BD, pBD, BD_ADDR_SIZE);

    BLUEAPI_TRACE_PRINTF_3(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:>>> MCLStatusInfo BD[%s] local_MCL[%d] status[%d]",
        TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE,pBD), localMCL_ID, status);

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}
#endif

/**
 * @brief  send acl status info to app
 *
 * @param  pBD: bd address
 * @param  remote_BD_type: remote bd type
 * @param  status 
 * @param  pParam: param
 *
 * @return  
 *
 */
void blueAPI_Send_ACLStatusInfo(uint8_t *pBD, TBlueAPI_RemoteBDType remote_BD_type,
    TBlueAPI_ACLStatus status, PBlueAPI_ACLStatusParam pParam)
{
    TBlueAPI_UsMessage msg;

    msg.Command                        = blueAPI_EventACLStatusInfo;
    msg.p.ACLStatusInfo.status         = status;
    msg.p.ACLStatusInfo.remote_BD_type = remote_BD_type;
    memcpy(msg.p.ACLStatusInfo.remote_BD, pBD, BD_ADDR_SIZE);

    BLUEAPI_TRACE_PRINTF_3(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:>>> ACLStatusInfo BD[%s] bdType[%d] status[%d]",
        TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE, pBD), remote_BD_type, status);

    if (pParam != NULL)
    {
        memcpy(&msg.p.ACLStatusInfo.p, pParam, sizeof(TBlueAPI_ACLStatusParam));
    }
    else
    {
        memset(&msg.p.ACLStatusInfo.p, 0x00, sizeof(TBlueAPI_ACLStatusParam));
    }

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

/**
 * @brief  send create MDL Indicate
 *
 * @param  pMCL: 
 * @param  pMDL: 
 *
 * @return
 *
 */
void blueAPI_Send_CreateMDLInd(PBlueAPI_MCL pMCL, PBlueAPI_MDL pMDL)
{
    TBlueAPI_UsMessage msg;

    msg.Command                       = blueAPI_EventCreateMDLInd;
    msg.p.CreateMDLInd.local_MDL_ID   = pMDL->local_MDL_ID;
    msg.p.CreateMDLInd.remote_BD_type = pMDL->remoteBdType;
    memcpy(msg.p.CreateMDLInd.remote_BD, pMDL->remoteBd, BD_ADDR_SIZE);

    BLUEAPI_TRACE_PRINTF_3(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:>>> CreateMDLInd BD[%s] bdType[%d] local_MDL[%d]",
        TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE, pMDL->remoteBd),
        pMDL->remoteBdType, pMDL->local_MDL_ID);

    /* save pending US COM context */
    if (pMCL)
    {  
        pMCL->US_CommandInProgress = (TBlueAPI_Command)msg.Command;
        memcpy(&pMCL->US_CommandData, &msg.p, sizeof(TBlueAPI_UsCommandData));
    }
    else
    {
        BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE, "!!==>: no MCL found");
    }

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

/**
* @brief  blueapid send connect MDL response
*
* @param  pReqMsg
* @param  pMCL
* @param  pBD
* @param  bdType
* @param  local_MDL_ID
* @param  cause
*
* @return  
*
*/
void blueAPI_Send_ConnectMDLRsp(PBlueAPI_DsMessage pReqMsg, PBlueAPI_MCL pMCL,
    uint8_t *pBD, TBlueAPI_RemoteBDType bdType, uint16_t local_MDL_ID, TBlueAPI_Cause cause)
{
    TBlueAPI_UsMessage msg;

    msg.Command                         = blueAPI_EventConnectMDLRsp;
    msg.p.ConnectMDLRsp.local_MDL_ID    = local_MDL_ID;
    msg.p.ConnectMDLRsp.cause           = cause;
    msg.p.ConnectMDLRsp.remote_BD_type  = bdType;

    memcpy(msg.p.ConnectMDLRsp.remote_BD, pBD, BD_ADDR_SIZE);

    BLUEAPI_TRACE_PRINTF_4(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:>>> ConnectMDLRsp BD[%s] bdType[%d] local_MDL[%d] cause[%d]",
        TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE, pBD), bdType, local_MDL_ID, cause);

    /* clear pending DS COM context */
    if (pMCL)
    {
        pMCL->DS_CommandInProgress = blueAPI_EventIdle;
        pMCL->DS_CommandMsg = NULL;
    }
    else
    {
        BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE, "!!==>: no MCL found");
    }

    if (pReqMsg == NULL)
    {
        BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE, "!!==>: pReqMsg is NULL");
    }

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, pReqMsg, &msg);
}

/**
 * @brief  send disconnect mdl indicate
 *
 * @param  pMCL
 * @param  pMDL
 * @param  cause
 *
 * @return  
 *
 */
void blueAPI_Send_DisconnectMDLInd(PBlueAPI_MCL pMCL, PBlueAPI_MDL pMDL,
    TBlueAPI_Cause cause)
{
    TBlueAPI_UsMessage msg;
    PBlueAPI_LinkDescriptor pLinkContext;

    pLinkContext = blueAPI_ChannelFindByLocal_MDL_ID(pMDL->local_MDL_ID);

    msg.Command                         = blueAPI_EventDisconnectMDLInd;
    msg.p.DisconnectMDLInd.local_MDL_ID = pMDL->local_MDL_ID;
    msg.p.DisconnectMDLInd.cause        = cause;

    BLUEAPI_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:>>> DisconnectMDLInd local_MDL[%d] cause[%d]", pMDL->local_MDL_ID, cause);

    /* save pending US COM context */
    if (pMCL)
    {  
        pMCL->US_CommandInProgress = (TBlueAPI_Command)msg.Command;
        memcpy(&pMCL->US_CommandData, &msg.p, sizeof(TBlueAPI_UsCommandData));
    }
    else
    {
        BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE, "!!==>: no MCL found");
    }

    if (pLinkContext)
    {
        pLinkContext->MDLConnected = FALSE;
    }

    /* delete flag from MDL */
    blueAPI_MDLSubFlags(pMDL, BLUE_API_MDL_FLAG_DISCONNECT_IND);

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

/**
 * @brief  send mdl disconnect response
 *
 * @param  pReqMsg
 * @param  pMCL
 * @param  local_MDL_ID
 * @param  cause
 *
 * @return  
 *
 */
void blueAPI_Send_DisconnectMDLRsp(PBlueAPI_DsMessage pReqMsg,
    PBlueAPI_MCL pMCL, uint16_t local_MDL_ID, TBlueAPI_Cause cause)
{
    TBlueAPI_UsMessage msg;

    msg.Command                         = blueAPI_EventDisconnectMDLRsp;
    msg.p.DisconnectMDLRsp.local_MDL_ID = local_MDL_ID;
    msg.p.DisconnectMDLRsp.cause        = cause;

    BLUEAPI_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:>>> DisconnectMDLRsp local_MDL[%d] cause[%d]", local_MDL_ID, cause);

    /* clear pending DS COM context */
    if (pMCL)
    {  
        pMCL->DS_CommandInProgress = blueAPI_EventIdle;
        pMCL->DS_CommandMsg = NULL;
    }
    else
    {
        BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE, "!!==>: no MCL found");
    }

    if (pReqMsg == NULL)
    {
        BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE, "!!==>: pReqMsg is NULL");
    }

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, pReqMsg, &msg);
}

void blueAPI_Send_RegisterRsp(PBlueAPI_App pBlueAPIApp, PBlueAPI_DsMessage pReqMsg,
    TBlueAPI_Cause cause)
{
    TBlueAPI_UsMessage msg = {0};

    msg.Command             = blueAPI_EventRegisterRsp;
    msg.p.RegisterRsp.cause = cause;

    blueAPI_TgtSendEvent(pBlueAPIApp, pReqMsg, &msg);
}

/**
 * @brief  blueapi send mtu size info
 *
 * @param  local_MDL_ID
 * @param  mtuSize
 *
 * @return  
 *
 */
void blueAPI_Send_GATTMtuSizeInfo(uint16_t local_MDL_ID, uint16_t mtuSize)
{
    TBlueAPI_UsMessage msg;

    BLUEAPI_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:>>> GATTMtuSizeInfo local_MDL[%d] mtuSize[%d]", local_MDL_ID, mtuSize);

    msg.Command = blueAPI_EventGATTMtuSizeInfo;

    msg.p.GATTMtuSizeInfo.local_MDL_ID = local_MDL_ID;
    msg.p.GATTMtuSizeInfo.mtuSize      = mtuSize;

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

/**
 * @brief  blueapi send blueAPI_EventGATTServiceRegisterRsp
 *
 * @param  serviceHandle
 * @param  subCause
 *
 * @return  
 *
 */
void blueAPI_Send_GATTServiceRegisterRsp(PVOID serviceHandle, uint16_t subCause)
{
    TBlueAPI_UsMessage msg;  
    TBlueAPI_Cause   cause = blueAPI_GATTConvertCause(subCause);

    BLUEAPI_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:>>> GATTServiceRegisterRsp srvHandle[0x%X]  subcause[0x%x]", serviceHandle, subCause);

    msg.Command                                = blueAPI_EventGATTServiceRegisterRsp;
    msg.p.GATTServiceRegisterRsp.serviceHandle = serviceHandle;
    msg.p.GATTServiceRegisterRsp.cause    = cause;
    msg.p.GATTServiceRegisterRsp.subCause = subCause;

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

/**
 * @brief  blueapi send gatt attribute update response
 *
 * @param  pBuffer
 * @param  offset
 * @param  serviceHandle
 * @param  requestHandle
 * @param  attribIndex
 * @param  subCause
 * @param  count
 *
 * @return
 *
 */
BOOL blueAPI_Send_GATTAttributeUpdateRsp(PVOID pBuffer, uint16_t offset,
    PVOID serviceHandle, PVOID requestHandle, uint16_t attribIndex, uint16_t subCause, uint16_t count)
{
    TBlueAPI_Cause cause;
    PBlueAPI_UsMessage pMsg = (PBlueAPI_UsMessage)pBuffer;
    PBlueAPI_GATTAttributeUpdateRsp pGATTAttribUpdateRsp = &pMsg->p.GATTAttributeUpdateRsp;

    pMsg->Command = blueAPI_EventGATTAttributeUpdateRsp;
    pMsg->Length  = offsetof(TBlueAPI_UsMessage, p.GATTAttributeUpdateRsp.list);
    pGATTAttribUpdateRsp->serviceHandle = serviceHandle;
    cause = blueAPI_GATTConvertCause(subCause);
    pGATTAttribUpdateRsp->cause = cause;
    pGATTAttribUpdateRsp->subCause = subCause;

    BLUEAPI_TRACE_PRINTF_3(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:>>> GATTAttributeUpdateRsp srvHandle[0x%X], cause[0x%x],  subcause[0x%x]",
        serviceHandle, cause, subCause);

    pGATTAttribUpdateRsp->attribIndex = attribIndex;
    pGATTAttribUpdateRsp->requestHandle = requestHandle;
    pGATTAttribUpdateRsp->count = count;
    pGATTAttribUpdateRsp->gap = offset - offsetof(TBlueAPI_UsMessage, p.GATTAttributeUpdateRsp.list);

    if (count > 0)
    {
        pMsg->Length += pGATTAttribUpdateRsp->gap + count*sizeof(TGATT_ATTRIB_UPDATE_LIST_ELEMENT);
    }

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, pMsg);

    return FALSE;
}

void blueAPI_Send_GATTAttributeUpdateStatusInd(uint16_t wAttribIndex,
    PVOID pServiceHandle, PVOID pReqHandle, uint8_t bdType, uint8_t *RemoteBd, uint16_t subCause)
{
    TBlueAPI_UsMessage msg;
    TBlueAPI_Cause   cause = blueAPI_GATTConvertCause(subCause);

    msg.Command = blueAPI_EventGATTAttributeUpdateStatusInd;
    msg.p.GATTAttributeUpdateStatusInd.serviceHandle  = pServiceHandle;
    msg.p.GATTAttributeUpdateStatusInd.requestHandle  = pReqHandle;
    msg.p.GATTAttributeUpdateStatusInd.attribIndex    = wAttribIndex;
    msg.p.GATTAttributeUpdateStatusInd.cause          = cause;
    msg.p.GATTAttributeUpdateStatusInd.subCause       = subCause;
    msg.p.GATTAttributeUpdateStatusInd.remote_BD_Type = (TBlueAPI_RemoteBDType)bdType;

    memcpy(msg.p.GATTAttributeUpdateStatusInd.remote_BD, RemoteBd,
        sizeof(msg.p.GATTAttributeUpdateStatusInd.remote_BD));

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

/**
 * @brief  send GATTAttributeRead indicate
 *
 * @param  serviceHandle
 * @param  attribIndex
 * @param  readOffset
 * @param  local_MDL_ID
 *
 * @return
 *
 */
void blueAPI_Send_GATTAttributeReadInd(PVOID serviceHandle, uint16_t attribIndex,
    uint16_t readOffset, uint16_t local_MDL_ID)
{
    TBlueAPI_UsMessage msg;

    BLUEAPI_TRACE_PRINTF_3(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:>>> GATTAttributeReadInd srvHandle[0x%X] attribIndex[%d] readOffset[%d]",
        serviceHandle, attribIndex, readOffset);

    msg.Command = blueAPI_EventGATTAttributeReadInd;
    msg.p.GATTAttributeReadInd.local_MDL_ID  = local_MDL_ID;
    msg.p.GATTAttributeReadInd.serviceHandle = serviceHandle;
    msg.p.GATTAttributeReadInd.attribIndex   = attribIndex;
    msg.p.GATTAttributeReadInd.readOffset    = readOffset;

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

/**
 * @brief  send blueAPI_EventGATTAttributeWriteCommandInfo/blueAPI_EventGATTAttributeWriteInd
 *
 * @param  pBuffer
 * @param  offset
 * @param  type
 * @param  serviceHandle
 * @param  attribIndex
 * @param  attribLength
 * @param  handle
 * @param  writeOffset
 * @param  local_MDL_ID
 *
 * @return
 *
 */
BOOL blueAPI_Send_GATTAttributeWriteInd(PVOID pBuffer, uint16_t offset, uint16_t type, 
    PVOID serviceHandle, uint16_t attribIndex, uint16_t attribLength, uint16_t handle, 
    uint16_t writeOffset, uint16_t local_MDL_ID)
{
    PBlueAPI_UsMessage pMsg = (PBlueAPI_UsMessage)pBuffer;
    PBlueAPI_GATTAttributeWriteInd  pGATTAttribWriteInd = &pMsg->p.GATTAttributeWriteInd;

    if (offset < offsetof(TBlueAPI_UsMessage, p.GATTAttributeWriteInd.data))
    {
        return TRUE ;
    }
                 

	switch( type )
	{
	default:
        break;
	case GATT_WRITE_TYPE_CMD:
		pMsg->Command = blueAPI_EventGATTAttributeWriteCommandInfo;
		break;
	case GATT_WRITE_TYPE_REQ:
		pMsg->Command = blueAPI_EventGATTAttributeWriteInd;
		break;
    case GATT_WRITE_TYPE_PREP:
        pMsg->Command = blueAPI_EventGATTAttributePrepareWriteInd;
        break;

	}


    pMsg->Length  = offsetof(TBlueAPI_UsMessage, p.GATTAttributeWriteInd.data);
    pGATTAttribWriteInd->local_MDL_ID  = local_MDL_ID;
    pGATTAttribWriteInd->serviceHandle = serviceHandle;
    pGATTAttribWriteInd->attribIndex   = attribIndex;
    pGATTAttribWriteInd->attribLength  = attribLength;
    pGATTAttribWriteInd->writeOffset   = writeOffset;
    pGATTAttribWriteInd->gap = offset - offsetof(TBlueAPI_UsMessage, p.GATTAttributeWriteInd.data);
    pMsg->Length += pGATTAttribWriteInd->gap + pGATTAttribWriteInd->attribLength;

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, pMsg);

    return FALSE;
}

/**
 * @brief  blueAPI_EventGATTCCCDInfo
 *
 * @param  pBuffer
 * @param  offset
 * @param  serviceHandle
 * @param  count
 * @param  local_MDL_ID
 *
 * @return
 *
 */
BOOL blueAPI_Send_GATTCCCDInfo(PVOID pBuffer, uint16_t offset, PVOID serviceHandle,
    uint16_t count, uint16_t local_MDL_ID)
{
    PBlueAPI_UsMessage pMsg = (PBlueAPI_UsMessage)pBuffer;
    PBlueAPI_GATTCCCDInfo pGATTCCCDInfo = &pMsg->p.GATTCCCDInfo;

    BLUEAPI_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:>>> GATTCCCDInfo srvHandle[0x%X] count[%d]", serviceHandle, count);

    pMsg->Command = blueAPI_EventGATTCCCDInfo;
    pMsg->Length  = offsetof(TBlueAPI_UsMessage, p.GATTCCCDInfo.data);

    pGATTCCCDInfo->local_MDL_ID  = local_MDL_ID;
    pGATTCCCDInfo->serviceHandle = serviceHandle;
    pGATTCCCDInfo->count         = count;
    pGATTCCCDInfo->gap           = offset - offsetof(TBlueAPI_UsMessage,p.GATTCCCDInfo.data);

    pMsg->Length += pGATTCCCDInfo->gap + pGATTCCCDInfo->count * 2*sizeof(uint16_t);

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, pMsg);

    return FALSE;
}

/**
 * @brief  blueapi send gatt discovery response
 *
 * @param  pReqMsg: 
 * @param  local_MDL_ID
 * @param  discoveryType
 * @param  subCause
 *
 * @return
 *
 */
void blueAPI_Send_GATTDiscoveryRsp(PBlueAPI_DsMessage pReqMsg, uint16_t local_MDL_ID,
    TBlueAPI_GATTDiscoveryType discoveryType, uint16_t subCause)
{
    TBlueAPI_UsMessage msg;
    TBlueAPI_Cause cause = blueAPI_GATTConvertCause(subCause);

    BLUEAPI_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:>>> GATTDiscoveryRsp, cause[0x%x]  subcause[0x%x]", cause, subCause);

    msg.Command = blueAPI_EventGATTDiscoveryRsp;
    msg.p.GATTDiscoveryRsp.local_MDL_ID  = local_MDL_ID;
    msg.p.GATTDiscoveryRsp.discoveryType = discoveryType;
    msg.p.GATTDiscoveryRsp.cause         = cause;
    msg.p.GATTDiscoveryRsp.subCause      = subCause;

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, pReqMsg, &msg);
}

/**
 * @brief  send GATT_DISCOVERY_IND.
 *
 * @param  pBuffer
 * @param  offset
 * @param  type
 * @param  elementCount
 * @param  elementLength
 * @param  subCause
 * @param  local_MDL_ID
 *
 * @return  
 *
 */
BOOL blueAPI_Send_GATTDiscoveryInd(PVOID pBuffer, uint16_t offset, uint16_t type,
    uint16_t elementCount, uint16_t elementLength, uint16_t subCause, uint16_t local_MDL_ID)
{
    TBlueAPI_Cause cause;
    TBlueAPI_GATTDiscoveryType discoveryType;
    PBlueAPI_UsMessage pMsg = (PBlueAPI_UsMessage)pBuffer;
    PBlueAPI_GATTDiscoveryInd pGATTDiscoveryInd = &pMsg->p.GATTDiscoveryInd;

    cause = blueAPI_GATTConvertCause(subCause);

    switch (type)
    {
    default:
        assert(FALSE);
        return TRUE;

    case GATT_TYPE_DISCOVERY_PSRV_ALL:
        discoveryType = blueAPI_GATTDiscoveryServices;
        break;

    case GATT_TYPE_DISCOVERY_PSRV_UUID:
        discoveryType = blueAPI_GATTDiscoveryServiceByUUID;
        break;

    case GATT_TYPE_DISCOVERY_CHAR_ALL:
        discoveryType = blueAPI_GATTDiscoveryCharacteristics;
        break;

    case GATT_TYPE_DISCOVERY_CHAR_DESCR:
        discoveryType = blueAPI_GATTDiscoveryCharacDescriptors;
        break;

    case GATT_TYPE_DISCOVERY_RELATION:
        discoveryType = blueAPI_GATTDiscoveryRelationship;
        break;
    }

    BLUEAPI_TRACE_PRINTF_4(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:>>> GATTDiscoveryInd type[%d], cause[0x%x],  subcause[0x%x] count[%d]",
        discoveryType, cause, subCause, elementCount);

    pMsg->Command = blueAPI_EventGATTDiscoveryInd;
    pMsg->Length  = offsetof(TBlueAPI_UsMessage, p.GATTDiscoveryInd.list);
    pGATTDiscoveryInd->local_MDL_ID = local_MDL_ID;
    pGATTDiscoveryInd->discoveryType = discoveryType;
    pGATTDiscoveryInd->cause         = cause; 
    pGATTDiscoveryInd->subCause      = subCause;
    pGATTDiscoveryInd->elementCount  = elementCount;
    pGATTDiscoveryInd->elementLength = elementLength;
    pGATTDiscoveryInd->gap = offset - offsetof(TBlueAPI_UsMessage, p.GATTDiscoveryInd.list);

    pMsg->Length += pGATTDiscoveryInd->gap +
        (pGATTDiscoveryInd->elementCount * pGATTDiscoveryInd->elementLength);

    /* forward list elements without Blueface (packed struct !!) to BlueAPI */
    /* conversion. this is OK as long as the list element structs comprise  */
    /* only WORDs and uint8_t[] arrays of even size. if not manual conversion  */
    /* (into larger buffer :-( ..) is required !!!                          */
    /* XXXXMJMJ emergency exit: */
    if ((sizeof(TGATT_GENERIC_ELEMENT_16) != sizeof(TBlueAPI_GATTGenericElement16)) ||
        (sizeof(TGATT_GENERIC_ELEMENT_128) != sizeof(TBlueAPI_GATTGenericElement128)) ||
        (sizeof(TGATT_PSRV_ALL_ELEMENT_16) != sizeof(TBlueAPI_GATTServicesElement16)) ||
        (sizeof(TGATT_PSRV_ALL_ELEMENT_128) != sizeof(TBlueAPI_GATTServicesElement128)) ||
        (sizeof(TGATT_PSRV_UUID_ELEMENT) != sizeof(TBlueAPI_GATTServiceByUUIDElement)) ||
        (sizeof(TGATT_RELATION_ELEMENT_16) != sizeof(TBlueAPI_GATTRelationshipElement16)) ||
        (sizeof(TGATT_RELATION_ELEMENT_128) != sizeof(TBlueAPI_GATTRelationshipElement128)) ||
        (sizeof(TGATT_CHAR_ALL_ELEMENT_16) != sizeof(TBlueAPI_GATTCharacteristicsElement16)) ||
        (sizeof(TGATT_CHAR_ALL_ELEMENT_128) != sizeof(TBlueAPI_GATTCharacteristicsElement128)) ||
        (sizeof(TGATT_CHAR_DESCR_ELEMENT_16) != sizeof(TBlueAPI_GATTCharacDescriptorsElement16)) ||
        (sizeof(TGATT_CHAR_DESCR_ELEMENT_128) != sizeof(TBlueAPI_GATTCharacDescriptorsElement128)))
    {
        assert(FALSE);
        return TRUE;
    }

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, pMsg);

    return FALSE;
}

/**
 * @brief  gatt attribute read response
 *
 * @param  pReqMsg
 * @param  pBuffer
 * @param  offset
 * @param  readType
 * @param  readOffset
 * @param  totalLength
 * @param  attribLength
 * @param  nbrOfHandles
 * @param  subCause
 * @param  local_MDL_ID
 *
 * @return
 *
 */
BOOL blueAPI_Send_GATTAttributeReadRsp(PBlueAPI_DsMessage pReqMsg, PVOID pBuffer,
    uint16_t offset, TBlueAPI_GATTReadType readType, uint16_t readOffset, uint16_t totalLength,
    uint16_t attribLength, uint16_t nbrOfHandles, uint16_t subCause, uint16_t local_MDL_ID)
{
    TBlueAPI_Cause cause;
    PBlueAPI_UsMessage pMsg;
    PBlueAPI_GATTAttributeReadRsp pGATTAttributeReadRsp;

    if (pBuffer != NULL)
    {
        pMsg = (PBlueAPI_UsMessage)pBuffer;
    }
    else
    {
        if (osBufferGet(pBlueAPIData->usBlueAPIPoolID, sizeof(TBlueAPI_UsMessage), (PVOID *)&pMsg))
        {
            BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_ERROR,
                "!!blueAPI_Send_GATTAttributeReadRsp: failed to get buffer for cmd[0x%X]",
                pMsg->Command);

            DebuggerBreak();
            return TRUE;
        }
    }
    cause = blueAPI_GATTConvertCause(subCause);

    BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:>>> GATTAttributeReadRsp  subcause[0x%x]",subCause);

    pMsg->Command = blueAPI_EventGATTAttributeReadRsp;
    pMsg->Length  = offsetof(TBlueAPI_UsMessage, p.GATTAttributeReadRsp.handlesData);
    pGATTAttributeReadRsp               = &pMsg->p.GATTAttributeReadRsp;
    pGATTAttributeReadRsp->local_MDL_ID = local_MDL_ID;
    pGATTAttributeReadRsp->readType     = readType;
    pGATTAttributeReadRsp->cause        = cause;
    pGATTAttributeReadRsp->subCause     = subCause;

    if (pBuffer == NULL)
    {
        /* local rejection of request */
        pGATTAttributeReadRsp->readOffset   = 0;
        pGATTAttributeReadRsp->totalLength  = 0;
        pGATTAttributeReadRsp->attribLength = 0;
        pGATTAttributeReadRsp->nbrOfHandles = 0;
        pGATTAttributeReadRsp->gap          = 0;
    }
    else
    {
        if (offset < offsetof(TBlueAPI_UsMessage,p.GATTAttributeReadRsp.handlesData))
        {
            /* BlueAPI msg header does not fit .. */
            assert(FALSE);
            return TRUE;
        }

        pGATTAttributeReadRsp->readOffset   = readOffset;
        pGATTAttributeReadRsp->totalLength  = totalLength;
        pGATTAttributeReadRsp->attribLength = attribLength;
        pGATTAttributeReadRsp->nbrOfHandles = nbrOfHandles;
        pGATTAttributeReadRsp->gap =
            offset - offsetof(TBlueAPI_UsMessage,p.GATTAttributeReadRsp.handlesData);

        pMsg->Length += pGATTAttributeReadRsp->gap + pGATTAttributeReadRsp->totalLength;
    }

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, pReqMsg, pMsg);

    return FALSE;
}

/**
 * @brief  send gatt attribute write resonse   
 *
 * @param  pReqMsg
 * @param  local_MDL_ID
 * @param  writeType
 * @param  subCause
 * @param  length
 *
 * @return
 *
 */
BOOL blueAPI_Send_GATTAttributeWriteRsp(PBlueAPI_DsMessage pReqMsg,
    uint16_t local_MDL_ID, TBlueAPI_GATTWriteType writeType, uint16_t subCause, uint16_t length)
{
    TBlueAPI_UsMessage msg;
    PBlueAPI_UsMessage pMsg; 
    TBlueAPI_Cause cause = blueAPI_GATTConvertCause(subCause);
    BOOL releaseBuffer = TRUE;

    BLUEAPI_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:>>> GATTAttribWriteRsp cause is [0x%x] subcause[0x%x]", cause, subCause);

    pMsg = &msg;
    pMsg->Length = offsetof(TBlueAPI_UsMessage, p.GATTAttributeWriteRsp.data);
    pMsg->p.GATTAttributeWriteRsp.attribLength = 0;
    pMsg->p.GATTAttributeWriteRsp.gap          = 0;

    pMsg->Command = blueAPI_EventGATTAttributeWriteRsp;
    pMsg->p.GATTAttributeWriteRsp.local_MDL_ID = local_MDL_ID;
    pMsg->p.GATTAttributeWriteRsp.writeType    = writeType;
    pMsg->p.GATTAttributeWriteRsp.cause        = cause;
    pMsg->p.GATTAttributeWriteRsp.subCause     = subCause;

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, pReqMsg, pMsg);

    return releaseBuffer;
}

BOOL blueAPI_Send_GATTAttributePrepareWriteRsp(PBlueAPI_DsMessage pReqMsg,
    PVOID pBuffer, uint16_t offset, uint16_t attribLength, uint16_t writeOffset, uint16_t subCause,  
    uint16_t local_MDL_ID)
{
    PBlueAPI_UsMessage pMsg = NULL;
    TBlueAPI_Cause cause = blueAPI_GATTConvertCause(subCause);
    PBlueAPI_GATTAttributePrepareWriteRsp pGATTAttribPrepareWriteRsp = NULL;
    
    if (pReqMsg != NULL)
    {
        /* local rejection of request */
        pMsg = (PBlueAPI_UsMessage)pReqMsg;
    }
    else
    {
        /* from lower layer (=> pBfMsg != NULL) */
        pMsg = (PBlueAPI_UsMessage)pBuffer;
    }


    pMsg->Command = blueAPI_EventGATTAttributePrepareWriteRsp;
    pMsg->Length = offsetof(TBlueAPI_UsMessage, p.GATTAttributePrepareWriteRsp.data);
    pGATTAttribPrepareWriteRsp = &pMsg->p.GATTAttributePrepareWriteRsp;
    pGATTAttribPrepareWriteRsp->local_MDL_ID = local_MDL_ID;
    pGATTAttribPrepareWriteRsp->cause        = cause;
    pGATTAttribPrepareWriteRsp->subCause     = subCause;

    if (pReqMsg != NULL)
    {
        pGATTAttribPrepareWriteRsp->writeOffset  = 0;
        pGATTAttribPrepareWriteRsp->attribLength = 0;
        pGATTAttribPrepareWriteRsp->gap          = 0;

    }
    else
    {
        pGATTAttribPrepareWriteRsp->writeOffset  = writeOffset;
        pGATTAttribPrepareWriteRsp->attribLength = attribLength;
        pGATTAttribPrepareWriteRsp->gap =
            offset - offsetof(TBlueAPI_UsMessage, p.GATTAttributePrepareWriteRsp.data);
        pMsg->Length += pGATTAttribPrepareWriteRsp->gap + pGATTAttribPrepareWriteRsp->attribLength;
    }

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, pMsg);

    return  FALSE;
}

void blueAPI_Send_GATTExecuteWriteRsp(PBlueAPI_DsMessage pReqMsg, 
    uint16_t local_MDL_ID, uint16_t subCause)
{
    TBlueAPI_UsMessage msg = {0};
    TBlueAPI_Cause   cause = blueAPI_GATTConvertCause(subCause);

    msg.Command = blueAPI_EventGATTAttributeExecuteWriteRsp;
    msg.p.GATTAttributeExecuteWriteRsp.local_MDL_ID = local_MDL_ID;
    msg.p.GATTAttributeExecuteWriteRsp.cause        = cause;
    msg.p.GATTAttributeExecuteWriteRsp.subCause     = subCause;

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, pReqMsg, &msg);
}

void blueAPI_Send_GATTExecuteWriteInd(uint16_t local_MDL_ID, uint8_t flags)
{
    TBlueAPI_UsMessage msg = {0};

    msg.Command = blueAPI_EventGATTAttributeExecuteWriteInd;
    msg.p.GATTAttributeExecuteWriteInd.local_MDL_ID  = local_MDL_ID;
    msg.p.GATTAttributeExecuteWriteInd.flags = flags;

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

/**
 * @brief  blueapi blueAPI_EventGATTAttributeNotificationInfo / blueAPI_EventGATTAttributeInd
 *
 * @param  pBuffer
 * @param  offset
 * @param  Notify
 * @param  attribHandle
 * @param  attribLength
 * @param  local_MDL_ID
 *
 * @return  
 *
 */
BOOL blueAPI_Send_GATTAttributeNotifInd(PVOID pBuffer, uint16_t offset, BOOL Notify,
    uint16_t attribHandle, uint16_t attribLength, uint16_t local_MDL_ID)
{
    PBlueAPI_UsMessage pMsg = (PBlueAPI_UsMessage)pBuffer;
    PBlueAPI_GATTAttributeInd pGATTAttributeInd = &pMsg->p.GATTAttributeInd;

    BLUEAPI_TRACE_PRINTF_3(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:>>> GATTAttributeNotifInd notif[%d] attribHandle[0x%X] attribLength[%d]",
        Notify, attribHandle, attribLength);

    if (Notify)
    {
        pMsg->Command = blueAPI_EventGATTAttributeNotificationInfo;
    }
    else
    {
        pMsg->Command = blueAPI_EventGATTAttributeInd;
    }

    pMsg->Length  = offsetof(TBlueAPI_UsMessage, p.GATTAttributeInd.data);
    pGATTAttributeInd->local_MDL_ID = local_MDL_ID;
    pGATTAttributeInd->attribHandle = attribHandle;
    pGATTAttributeInd->attribLength = attribLength;
    pGATTAttributeInd->gap = offset - offsetof(TBlueAPI_UsMessage, p.GATTAttributeInd.data);

    pMsg->Length += pGATTAttributeInd->gap + pGATTAttributeInd->attribLength;

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, pMsg);

    return FALSE;
}

/**
 * @brief  send gatt sdp discovery response
 *
 * @param  pReqMsg
 * @param  cause
 *
 * @return  
 *
 */
void blueAPI_Send_GATTSDPDiscoveryRsp(PBlueAPI_DsMessage pReqMsg,TBlueAPI_Cause cause)
{
    TBlueAPI_UsMessage msg;

    BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE, "blueAPI:>>> GATTSDPDiscoveryRsp cause[%d]",cause);

    msg.Command = blueAPI_EventGATTSDPDiscoveryRsp;
    msg.p.GATTSDPDiscoveryRsp.cause = cause;

    blueAPI_SDP_DS_ContextClear(pBlueAPIData, pReqMsg);

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, pReqMsg, &msg);
}

/**
 * @brief  send blueAPI_EventGATTSDPDiscoveryInd
 *
 * @param  pBlueAPIdata
 * @param  serviceHandle
 * @param  remote_BD
 * @param  remote_GATT_UUID
 * @param  remote_GATT_StartHandle
 * @param  remote_GATT_EndHandle
 *
 * @return  
 *
 */
void blueAPI_Send_GATTSDPDiscoveryInd(PBlueAPI_Data pBlueAPIdata,
    uint32_t serviceHandle, uint8_t *remote_BD, uint16_t remote_GATT_UUID,
    uint16_t remote_GATT_StartHandle, uint16_t remote_GATT_EndHandle)
{
    TBlueAPI_UsMessage msg;

    BLUEAPI_TRACE_PRINTF_4(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:>>> GATTSDPDiscoveryInd handle[0x%X] UUID[0x%X] Start[0x%X] End[0x%X]",
        serviceHandle, remote_GATT_UUID, remote_GATT_StartHandle, remote_GATT_EndHandle);

    msg.Command                                       = blueAPI_EventGATTSDPDiscoveryInd;
    msg.p.GATTSDPDiscoveryInd.serviceHandle           = serviceHandle;
    msg.p.GATTSDPDiscoveryInd.remote_GATT_UUID        = remote_GATT_UUID;
    msg.p.GATTSDPDiscoveryInd.remote_GATT_StartHandle = remote_GATT_StartHandle;
    msg.p.GATTSDPDiscoveryInd.remote_GATT_EndHandle   = remote_GATT_EndHandle;

    memcpy(msg.p.GATTSDPDiscoveryInd.remote_BD, remote_BD, BD_ADDR_SIZE);

    pBlueAPIdata->SM_SDP_US_Command = blueAPI_EventGATTSDPDiscoveryInd;
    pBlueAPIdata->SM_SDP_US_Handle  = serviceHandle;

    blueAPI_TgtSendEvent(pBlueAPIdata->pActiveApp, NULL, &msg);
}

/**
 * @brief  send blueAPI_EventGATTSecurityRsp
 *
 * @param  local_MDL_ID
 * @param  keyType
 * @param  keySize
 * @param  cause
 *
 * @return  
 *
 */
void blueAPI_Send_GATTSecurityRsp(uint16_t local_MDL_ID, TBlueAPI_KeyType keyType,
    uint8_t keySize, TBlueAPI_Cause cause)
{
    TBlueAPI_UsMessage msg;

    BLUEAPI_TRACE_PRINTF_4(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:>>> GATTSecurityRsp MDL[%d] keyType[%d] keySize[%d] cause[%d]",
        local_MDL_ID, keyType, keySize, cause);

    msg.Command                        = blueAPI_EventGATTSecurityRsp;
    msg.p.GATTSecurityRsp.local_MDL_ID = local_MDL_ID;
    msg.p.GATTSecurityRsp.keyType      = keyType;
    msg.p.GATTSecurityRsp.keySize      = keySize;
    msg.p.GATTSecurityRsp.cause        = cause;

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

/**
* @brief		blueapi send gatt server store indicator
*
* @param	pBlueAPIdata:
* @param	opCode: 
* @param	remote_BD
* @param	remote_BD_Type
* @param	restartHandle
* @param	dataLength
* @param	data
*
* @return
*
*/
void blueAPI_Send_GATTServerStoreInd(TBlueAPI_GATTStoreOpCode opCode,
    uint8_t *remote_BD, TBlueAPI_RemoteBDType remote_BD_Type, uint16_t restartHandle,
    uint8_t dataLength, uint8_t *data)
{
    TBlueAPI_UsMessage msg;

    BLUEAPI_TRACE_PRINTF_5(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:>>> GATTServerStoreInd op[%d] bd[%s] bdType[%d] restart[0x%x] datalen[%d]",
        opCode, TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE, remote_BD), remote_BD_Type,
        restartHandle, dataLength);

    msg.Command = blueAPI_EventGATTServerStoreInd;
    msg.p.GATTServerStoreInd.opCode = opCode;
    memcpy(msg.p.GATTServerStoreInd.remote_BD, remote_BD, BD_ADDR_SIZE);
    msg.p.GATTServerStoreInd.remote_BD_Type = remote_BD_Type;
    msg.p.GATTServerStoreInd.restartHandle = restartHandle;
    msg.p.GATTServerStoreInd.dataLength = dataLength;
    memcpy(msg.p.GATTServerStoreInd.data, data, dataLength);

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

void blueAPI_Send_LEScanInfo(uint8_t eventType, uint8_t addressType, uint8_t *address,
    uint8_t rssi, uint8_t dataLength, uint8_t *data)
{
    TBlueAPI_UsMessage msg;

    BLUEAPI_TRACE_PRINTF_5(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:>>> LEScanInfo BD[%s] BDtype[%d] advType[%d] rssi[%d] length[%d]",
        BLUEAPI_TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE, address), addressType, eventType, rssi,
        dataLength);

    msg.Command = blueAPI_EventLEScanInfo;
    msg.p.LEScanInfo.remote_BD_type = (TBlueAPI_RemoteBDType)(addressType);
    msg.p.LEScanInfo.advType = (TBlueAPI_LEAdvEventType)(eventType);
    msg.p.LEScanInfo.rssi = rssi;
    msg.p.LEScanInfo.dataLength = dataLength;
    memcpy(msg.p.LEScanInfo.remote_BD, address, BD_ADDR_SIZE);
    memcpy(msg.p.LEScanInfo.data, data, dataLength);

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

/**
 * @brief  send inquiry response to app
 *
 * @param  pReqMsg
 * @param  cancelInquiry
 * @param  cause
 *
 * @return
 *
 */
void blueAPI_Send_InquiryRsp( PBlueAPI_DsMessage pReqMsg, BOOL cancelInquiry,
    TBlueAPI_Cause cause)
{
    TBlueAPI_UsMessage msg;

    BLUEAPI_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_TRACE, "blueAPI:>>> InquiryRsp cancel[%d] cause[%d]",
        cancelInquiry, cause);

    msg.Command                    = blueAPI_EventInquiryRsp;
    msg.p.InquiryRsp.cancelInquiry = cancelInquiry;
    msg.p.InquiryRsp.cause         = cause;

    blueAPI_SDP_DS_ContextClear(pBlueAPIData, pReqMsg);

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, pReqMsg, &msg);
}

/**
 * @brief  send inquiry device info to app
 * 
 * @param  remote_BD
 * @param  remote_Device_Class
 * @param  remote_RSSI
 * @param  remote_Device_Name
 *
 * @return  
 *
 */
void blueAPI_Send_InquiryDeviceInfo(uint8_t *remote_BD, uint32_t remote_Device_Class,
    uint8_t remote_RSSI, uint8_t * remote_Device_Name)
{
    TBlueAPI_UsMessage msg;
    uint16_t          nameLen;

    BLUEAPI_TRACE_PRINTF_4(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:>>> InquiryDeviceInfo BD[%s] Class[0x%X] RSSI[%d] Name[%s]",
        TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE, remote_BD), remote_Device_Class, remote_RSSI,
        BTRACE_RAMDATA1(BLUEAPI_TRACE_MASK_TRACE, remote_Device_Name));

    msg.Command = blueAPI_EventInquiryDeviceInfo;
    msg.p.InquiryDeviceInfo.remote_Device_Class = remote_Device_Class;
    msg.p.InquiryDeviceInfo.remote_RSSI = remote_RSSI;

    memcpy(msg.p.InquiryDeviceInfo.remote_BD, remote_BD, BD_ADDR_SIZE);
    nameLen = strlen((char *)remote_Device_Name);
    memcpy(msg.p.InquiryDeviceInfo.remote_Device_Name, remote_Device_Name, nameLen);
    msg.p.InquiryDeviceInfo.remote_Device_Name[nameLen] = 0;

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

/**
 * @brief  send device name response
 *
 * @param  pReqMsg: 
 * @param  remote_BD
 * @param  remote_Device_Name
 * @param  cause
 *
 * @return
 *
 */
void blueAPI_Send_DeviceNameRsp(PBlueAPI_DsMessage pReqMsg, uint8_t *remote_BD,
    uint8_t *remote_Device_Name, TBlueAPI_Cause cause)
{
    TBlueAPI_UsMessage msg;
    uint16_t nameLen;

    BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE, "blueAPI:>>> DeviceNameRsp cause[%d]", cause);

    msg.Command = blueAPI_EventDeviceNameRsp;
    msg.p.DeviceNameRsp.cause = cause;

    if (remote_BD != NULL)
    {  
        memcpy(msg.p.DeviceNameRsp.remote_BD, remote_BD, BD_ADDR_SIZE);
    }
    else
    {  
        memset(msg.p.DeviceNameRsp.remote_BD, 0x00, BD_ADDR_SIZE);
    }

    nameLen = strlen((char *)remote_Device_Name);
    if (nameLen >= BLUE_API_DEVICE_NAME_LENGTH)
    {  
        nameLen = BLUE_API_DEVICE_NAME_LENGTH -1;
    }

    memcpy(msg.p.DeviceNameRsp.remote_Device_Name, remote_Device_Name, nameLen);
    msg.p.DeviceNameRsp.remote_Device_Name[nameLen] = '\0';

    blueAPI_SDP_DS_ContextClear(pBlueAPIData, pReqMsg);

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, pReqMsg, &msg);
}

/**
 * @brief  send did device indicate
 *
 * @param  pBlueAPIdata
 * @param  serviceHandle
 * @param  remote_BD
 * @param  remote_VendorID
 * @param  remote_VendorID_Source
 * @param  remote_ProductID
 * @param  remote_Version
 * @param  remote_Device_Name
 *
 * @return
 *
 */
void blueAPI_Send_DIDDeviceInd(PBlueAPI_Data pBlueAPIdata, uint32_t serviceHandle,
    uint8_t *remote_BD, uint16_t remote_VendorID, uint16_t remote_VendorID_Source,
    uint16_t remote_ProductID, uint16_t remote_Version, uint8_t * remote_Device_Name)
{
    TBlueAPI_UsMessage msg;
    uint16_t          nameLen;

    BLUEAPI_TRACE_PRINTF_7(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:>>> DIDDeviceInd handle[0x%X] BD[%s] V_ID[0x%X] P_ID[0x%X] Vers[0x%X] src[%d] Name[%s]%c",
        serviceHandle, TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE, remote_BD), remote_VendorID,
        remote_ProductID, remote_Version, remote_VendorID_Source,
        BTRACE_RAMDATA1(BLUEAPI_TRACE_MASK_TRACE, remote_Device_Name));

    msg.Command                               = blueAPI_EventDIDDeviceInd;
    msg.p.DIDDeviceInd.serviceHandle          = serviceHandle;
    msg.p.DIDDeviceInd.remote_VendorID        = remote_VendorID;
    msg.p.DIDDeviceInd.remote_VendorID_Source = remote_VendorID_Source;
    msg.p.DIDDeviceInd.remote_ProductID       = remote_ProductID;
    msg.p.DIDDeviceInd.remote_Version         = remote_Version;

    memcpy(msg.p.DIDDeviceInd.remote_BD, remote_BD, BD_ADDR_SIZE);

    nameLen = strlen((char *)remote_Device_Name);
    if (nameLen >= BLUE_API_DEVICE_NAME_LENGTH)
    {  
        nameLen = BLUE_API_DEVICE_NAME_LENGTH -1;
    }

    memcpy(msg.p.DIDDeviceInd.remote_Device_Name, remote_Device_Name, nameLen);
    msg.p.DIDDeviceInd.remote_Device_Name[nameLen] = '\0';

    pBlueAPIdata->SM_SDP_US_Command = blueAPI_EventDIDDeviceInd;
    pBlueAPIdata->SM_SDP_US_Handle  = serviceHandle;

    blueAPI_TgtSendEvent(pBlueAPIdata->pActiveApp, NULL, &msg);
}

/**
 * @brief  blueapi sende radio mode set rsp
 *
 * @param  cause
 *
 * @return  
 *
 */
void blueAPI_Send_RadioModeSetRsp(TBlueAPI_Cause cause)
{
   TBlueAPI_UsMessage msg;

   BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE, "blueAPI:>>> RadioModeSetRsp cause[%d]", cause);

   msg.Command = blueAPI_EventRadioModeSetRsp;
   msg.p.RadioModeSetRsp.cause = cause;

   blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

/**
 * @brief  send SDP discovey response
 *
 * @param  pBlueAPIdata
 * @param  pReqMsg
 * @param  cause
 *
 * @return  
 *
 */
void blueAPI_Send_SDPDiscoveryRsp(PBlueAPI_Data pBlueAPIdata, PBlueAPI_DsMessage pReqMsg, TBlueAPI_Cause cause)
{
    TBlueAPI_UsMessage msg;

    BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE, "blueAPI:>>> SDPDiscoveryRsp cause[%d]", cause);

    msg.Command = blueAPI_EventSDPDiscoveryRsp;
    msg.p.SDPDiscoveryRsp.cause = cause;

    blueAPI_SDP_DS_ContextClear(pBlueAPIdata, pReqMsg);

    blueAPI_TgtSendEvent(pBlueAPIdata->pActiveApp, pReqMsg, &msg);
}

/**
 * @brief  send SDP end point indicate
 * 
 * @param  pBlueAPIdata
 * @param  serviceHandle
 * @param  remote_BD
 * @param  remote_MDEP_ID
 * @param  remote_MDEP_DataType
 * @param  remote_MDEP_Description
 *
 * @return  
 *
 */
void blueAPI_Send_SDPEndpointInd(PBlueAPI_Data pBlueAPIdata, uint32_t serviceHandle,
    uint8_t *remote_BD, uint8_t remote_MDEP_ID, TBlueAPI_MDEPDataType remote_MDEP_DataType,
    uint8_t *remote_MDEP_Description, uint16_t remoteversion, uint16_t supportedfeatures)
{
    TBlueAPI_UsMessage msg;
    uint16_t          descLen;

    BLUEAPI_TRACE_PRINTF_7(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:>>> SDPEndpointInd handle[0x%X] BD[%s] ID[%d] Type[0x%X] name[%s] remoteversion[0x%x] supportedfeatures[0x%x]",
        serviceHandle, TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE, remote_BD), remote_MDEP_ID,
        remote_MDEP_DataType, BTRACE_RAMDATA1(BLUEAPI_TRACE_MASK_TRACE, remote_MDEP_Description),
        remoteversion, supportedfeatures);

    msg.Command                               = blueAPI_EventSDPEndpointInd;
    msg.p.SDPEndpointInd.serviceHandle        = serviceHandle;
    msg.p.SDPEndpointInd.remote_MDEP_ID       = remote_MDEP_ID;
    msg.p.SDPEndpointInd.remote_MDEP_DataType = remote_MDEP_DataType;

    memcpy(msg.p.SDPEndpointInd.remote_BD, remote_BD, BD_ADDR_SIZE);

    descLen = strlen((char *)remote_MDEP_Description);
    if (descLen >= BLUE_API_MDEP_NAME_LENGTH)
    {  
        descLen = BLUE_API_MDEP_NAME_LENGTH -1;
    }
    memcpy(msg.p.SDPEndpointInd.remote_MDEP_Description, remote_MDEP_Description, descLen);
    msg.p.SDPEndpointInd.remote_MDEP_Description[descLen] = '\0';

    msg.p.SDPEndpointInd.remoteversion = remoteversion;
    msg.p.SDPEndpointInd.supportedfeatures = supportedfeatures;

    pBlueAPIdata->SM_SDP_US_Command = blueAPI_EventSDPEndpointInd;
    pBlueAPIdata->SM_SDP_US_Handle  = serviceHandle;

    blueAPI_TgtSendEvent(pBlueAPIdata->pActiveApp, NULL, &msg);
}

/**
 * @brief  send act info to app
 *
 * @param  pBlueAPIdata
 * @param  pBlueAPIApp
 * @param  cause: 
 *
 * @return  
 *
 */
void blueAPI_Send_ActInfo(PBlueAPI_Data pBlueAPIdata, PBlueAPI_App pBlueAPIApp, TBlueAPI_Cause cause)
{
    TBlueAPI_UsMessage msg;

    msg.Command = blueAPI_EventActInfo;

    memcpy(msg.p.ActInfo.local_BD, pBlueAPIdata->local_BD, BD_ADDR_SIZE);

    msg.p.ActInfo.cause        = cause;
    msg.p.ActInfo.systemPoolID = pBlueAPIdata->systemPoolID;
    BLUEAPI_TRACE_PRINTF_4(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:>>> ActInfo App[0x%X] BD[%s] cause[%d] sysPool[0x%X]",
        pBlueAPIApp, TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE, msg.p.ActInfo.local_BD),
        msg.p.ActInfo.cause, msg.p.ActInfo.systemPoolID);

    TRACE_DUMP_SYSTEM(pBlueAPIdata);

    /* *NOT* using pBlueAPIdata->pActiveApp here, since we must inform all Apps */
    blueAPI_TgtSendEvent(pBlueAPIApp, NULL, &msg);
}

/**
 * @brief  send act info to all applications
 *
 * @param  pBlueAPIdata: 
 * @param  cause: 
 * @param  force: if send to applications which has sended
 *
 * @return  
 *
 */
void blueAPI_SendToAll_ActInfo(PBlueAPI_Data pBlueAPIdata, TBlueAPI_Cause cause, BOOL force)
{
    PBlueAPI_App pBlueAPIApp;

    pBlueAPIApp = &pBlueAPIdata->appDescriptorTable;
    if ((pBlueAPIApp->used) && ((!pBlueAPIApp->actInfoSent) || force))
    {
        blueAPI_Send_ActInfo(pBlueAPIdata, pBlueAPIApp, cause);
        pBlueAPIApp->actInfoSent = TRUE;
    }
}

#if SEND_INT_EVENT_INFO
/**
 * @brief  send internal event info to app
 *
 * @param  pBlueAPIdata:
 * @param  eventType: event type
 * @param  eventInfo:  event info
 * @param  cause:
 *
 * @return  
 *
 */
void blueAPI_Send_InternalEventInfo(TBlueAPI_InternalEventType eventType, uint32_t eventInfo, TBlueAPI_Cause cause)
{
    TBlueAPI_UsMessage msg;

    msg.Command                        = blueAPI_EventInternalEventInfo;
    msg.p.InternalEventInfo.eventType  = eventType;
    msg.p.InternalEventInfo.eventInfo  = eventInfo;
    msg.p.InternalEventInfo.cause      = cause;

    BLUEAPI_TRACE_PRINTF_3(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:>>> InternalEventInfo type[%d] info[0x%X] cause[%d]",
        msg.p.InternalEventInfo.eventType, msg.p.InternalEventInfo.eventInfo,
        msg.p.InternalEventInfo.cause);

    TRACE_DUMP_SYSTEM(pBlueAPIData); /* internalEventInfo */

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}
#endif

void blueAPI_Send_LEScanRsp(uint16_t subCause)
{
    TBlueAPI_UsMessage msg;
    TBlueAPI_Cause cause;

    cause = blueAPI_GATTConvertCause(subCause);

    msg.Command = blueAPI_EventLEScanRsp;
    msg.p.LEScanRsp.cause = cause;

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

/**
 * @brief   blueapi send le adv response
 *
 * @param  pBlueAPIdata
 * @param  cause
 *
 * @return  
 *
 */
void blueAPI_Send_LEAdvertiseRsp(TBlueAPI_LEAdvMode advMode, TBlueAPI_Cause cause)
{
    TBlueAPI_UsMessage msg;

    BLUEAPI_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:>>> LEAdvertiseRsp advMode[%d] cause[%d]", advMode, cause);

    msg.Command                   = blueAPI_EventLEAdvertiseRsp;
    msg.p.LEAdvertiseRsp.advMode  = advMode;
    msg.p.LEAdvertiseRsp.cause    = cause;

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

/**
 * @brief   blueapi send le adv param set response
 *
 * @param  cause
 *
 * @return  
 *
 */
void blueAPI_Send_LEAdvertiseParameterSetRsp(TBlueAPI_Cause cause)
{
    TBlueAPI_UsMessage msg;

    BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:>>> LEAdvertiseParameterSetRsp cause[%d]", cause);

    msg.Command = blueAPI_EventLEAdvertiseParameterSetRsp;
    msg.p.LEAdvertiseParameterSetRsp.cause = cause;

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

/**
 * @brief  blueapi send le advertise data set response
 *
 * @param  dataType
 * @param  cause
 *
 * @return  
 *
 */
void blueAPI_Send_LEAdvertiseDataSetRsp(TBlueAPI_LEDataType dataType,
    TBlueAPI_Cause cause)
{
    TBlueAPI_UsMessage msg;

    BLUEAPI_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:>>> LEAdvertiseDataSetRsp dataType[%d] cause[%d]", dataType, cause);

    msg.Command = blueAPI_EventLEAdvertiseDataSetRsp;
    msg.p.LEAdvertiseDataSetRsp.dataType = dataType;
    msg.p.LEAdvertiseDataSetRsp.cause = cause;

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

 /**
  * @brief  blueapi send modify whitelist response to app
  *
  * @param  operation
  * @param  cause
  *
  * @return
  *
  */
void blueAPI_Send_LEModifyWhitelistRsp(TBlueAPI_LEWhitelistOp operation,
    TBlueAPI_Cause cause)
{
    TBlueAPI_UsMessage msg;

    BLUEAPI_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:>>> LEModifyWhitelistRsp op[%d] cause[%d]", operation, cause);

    msg.Command = blueAPI_EventLEModifyWhitelistRsp;
    msg.p.LEModifyWhitelistRsp.operation = operation;
    msg.p.LEModifyWhitelistRsp.cause = cause;

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

/**
 * @brief  blueapi send le connect update response
 *
 * @param  local_MDL_ID
 * @param  subCause
 *
 * @return  
 *
 */
void blueAPI_Send_LEConnectionUpdateRsp(uint16_t local_MDL_ID, uint16_t subCause)
{
    TBlueAPI_UsMessage msg;

    TBlueAPI_Cause cause;

    cause = blueAPI_GATTConvertCause(subCause);

    BLUEAPI_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:>>> LEConnectionUpdateRsp MDL[%d] cause[%d]", local_MDL_ID, cause);

    msg.Command = blueAPI_EventLEConnectionUpdateRsp;
    msg.p.LEConnectionUpdateRsp.local_MDL_ID = local_MDL_ID;
    msg.p.LEConnectionUpdateRsp.cause = cause;

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

/**
 * @brief  blueapi send blueAPI_EventLEConnectionUpdateInd
 *
 * @param  local_MDL_ID
 * @param  pParam
 *
 * @return  
 *
 */
void blueAPI_Send_LEConnectionUpdateInd(uint16_t local_MDL_ID,
    TGATT_LE_CONNECTION_UPDATE_PARAM *pParam)
{
    TBlueAPI_UsMessage msg;

    BLUEAPI_TRACE_PRINTF_5(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:>>> LEConnectionUpdateInd MDL[%d] connInterval[%d-%d] latency[%d] supervision[%d]",
        local_MDL_ID, pParam->connIntervalMin, pParam->connIntervalMax, pParam->connLatency,
        pParam->supervisionTimeout);

    msg.Command                                     = blueAPI_EventLEConnectionUpdateInd;
    msg.p.LEConnectionUpdateInd.local_MDL_ID        = local_MDL_ID;
    msg.p.LEConnectionUpdateInd.connIntervalMin     = pParam->connIntervalMin;
    msg.p.LEConnectionUpdateInd.connIntervalMax     = pParam->connIntervalMax;
    msg.p.LEConnectionUpdateInd.connLatency         = pParam->connLatency;
    msg.p.LEConnectionUpdateInd.supervisionTimeout  = pParam->supervisionTimeout;

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

/**
 * @brief  blueapi send le connection paramter info
 *
 * @param  pLinkContext
 *
 * @return  
 *
 */
void blueAPI_Send_LEConnectionParameterInfo(PBlueAPI_LinkDescriptor pLinkContext)
{
    TBlueAPI_UsMessage msg;

    BLUEAPI_TRACE_PRINTF_4(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:>>> LEConnectionParameterInfo MDL[%d] connInterval[%d] latency[%d] supervision[%d]",
        pLinkContext->pMDL->local_MDL_ID, pLinkContext->LEConnectionInterval,
        pLinkContext->LEConnectionLatency, pLinkContext->LESupervisionTimeout);

    msg.Command                                        = blueAPI_EventLEConnectionParameterInfo;
    msg.p.LEConnectionParameterInfo.local_MDL_ID       = pLinkContext->pMDL->local_MDL_ID;
    msg.p.LEConnectionParameterInfo.connInterval       = pLinkContext->LEConnectionInterval;
    msg.p.LEConnectionParameterInfo.connLatency        = pLinkContext->LEConnectionLatency;
    msg.p.LEConnectionParameterInfo.supervisionTimeout = pLinkContext->LESupervisionTimeout;

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

void blueAPI_Send_SetRandomAddressRsp(uint16_t subCause)
{
    TBlueAPI_UsMessage msg = {0};
    TBlueAPI_Cause  cause = blueAPI_GATTConvertCause(subCause);

    BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:>>> SetRandomAddressRsp subCause 0x%x", subCause);

    msg.Command = blueAPI_EventSetRandomAddressRsp;
    msg.p.SetRandomAddressRsp.cause = cause;
    msg.p.SetRandomAddressRsp.subCause = subCause;

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

void blueAPI_Send_SDPRegisterRsp(TBlueAPI_Cause cause)
{
    TBlueAPI_UsMessage msg;

    BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:>>> SdpRegisterRsp cause[%d]", cause);

    msg.Command = blueAPI_EventSDPRegisterRsp;
    msg.p.SDPRegisterRsp.cause = cause;

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

void blueAPI_Send_L2cSecurityRegisterRsp(uint16_t psm, uint16_t server_channel,
    uint8_t outgoing, uint8_t active, uint16_t uuid, uint16_t cause)
{
    TBlueAPI_UsMessage msg;

    BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE, "blueAPI:>>> L2cSecurityRegisterRsp cause[%d]",
        cause);

    msg.Command = blueAPI_EventL2cSecurityRegisterRsp;
    msg.p.L2cSecurityRegisterRsp.psm = psm;
    msg.p.L2cSecurityRegisterRsp.server_channel = server_channel;
    msg.p.L2cSecurityRegisterRsp.outgoing = outgoing;
    msg.p.L2cSecurityRegisterRsp.active = active;
    msg.p.L2cSecurityRegisterRsp.uuid = uuid;
    msg.p.L2cSecurityRegisterRsp.cause= cause;

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

void blueAPI_Send_L2cConInd(uint16_t cid, uint8_t usQueueID, uint8_t *remote_bd)
{
    TBlueAPI_UsMessage msg;

    msg.Command = blueAPI_EventL2cConInd;
    msg.p.L2cConInd.cid = cid;
    msg.p.L2cConInd.usQueueID = usQueueID;

    memcpy(msg.p.L2cConInd.remote_BD, remote_bd, BD_ADDR_SIZE);

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

void blueAPI_Send_L2cConActInd(uint16_t cid, uint16_t localUsMTU, uint16_t remoteMTU,
    uint16_t dsPoolID, uint16_t usQueueID, uint8_t* remoteBD)
{
    TBlueAPI_UsMessage msg;

    msg.Command = blueAPI_EventL2cConActInd;
    msg.p.L2cConActInd.cid = cid;
    msg.p.L2cConActInd.localUsMTU = localUsMTU;
    msg.p.L2cConActInd.remoteMTU = remoteMTU;
    msg.p.L2cConActInd.dsPoolID = dsPoolID;
    msg.p.L2cConActInd.usQueueID = usQueueID;

    memcpy(msg.p.L2cConActInd.remoteBd, remoteBD, 6);

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

void blueAPI_Send_L2cDataInd(MESSAGE_T *pmsg, uint16_t usQueueID)
{
    TBlueAPI_UsMessage msg;

    msg.Command                 = blueAPI_EventL2cDataInd;
    msg.p.L2cDataInd.cid        = pmsg->MData.DataCBChan.Channel;
    msg.p.L2cDataInd.dataOffset = pmsg->MData.DataCBChan.Offset;
    msg.p.L2cDataInd.length     = pmsg->MData.DataCBChan.Length;
    msg.p.L2cDataInd.buf        = pmsg->MData.DataCBChan.BufferAddress;
    msg.p.L2cDataInd.flag       = pmsg->MData.DataCB.Flag;
    msg.p.L2cDataInd.usQueueID  = usQueueID;

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

void blueAPI_Send_L2cDiscInd(uint16_t cid, uint16_t usQueueID, uint16_t cause)
{
    TBlueAPI_UsMessage msg;

    msg.Command = blueAPI_EventL2cDiscInd;
    msg.p.L2cDiscInd.cid = cid;
    msg.p.L2cDiscInd.usQueueID = usQueueID;
    msg.p.L2cDiscInd.cause = cause;

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

void blueAPI_Send_L2cConRsp(uint16_t cid, uint16_t usQueueID, uint8_t *remoteBd,
    uint16_t status)
{
    TBlueAPI_UsMessage msg;

    msg.Command = blueAPI_EventL2cConRsp;
    msg.p.L2cConRsp.cid = cid;
    msg.p.L2cConRsp.usQueueID = usQueueID;
    msg.p.L2cConRsp.status = status;

    memcpy(msg.p.L2cConRsp.remote_BD, remoteBd, BD_ADDR_SIZE);

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

void blueAPI_Send_L2cDiscRsp(uint16_t usQueueID, uint16_t cid, uint16_t reslut)
{
    TBlueAPI_UsMessage msg;

    msg.Command = blueAPI_EventL2cDiscRsp;
    msg.p.L2cDiscRsp.cid = cid;
    msg.p.L2cDiscRsp.usQueueID = usQueueID;
    msg.p.L2cDiscRsp.status = reslut;

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

/**
 * @brief  rfcomm  authetication response
 *
 * @param  bd
 * @param  ref
 * @param  channelId
 * @param  uuid
 * @param  outgoing
 * @param  cause
 *
 * @return
 *
 */
void blueAPI_Send_RFCAuthenticationRsp(TBdAddr bd, uint16_t ref, uint16_t channelId,
    uint8_t outgoing, uint16_t cause)
{
    TBlueAPI_UsMessage msg;

    msg.Command = blueAPI_EventRFCAuthenticationRsp;
    msg.p.RFCAuthenticaionRsp.channel = ref;
    msg.p.RFCAuthenticaionRsp.dlci = channelId;
    msg.p.RFCAuthenticaionRsp.outgoing = outgoing;
    msg.p.RFCAuthenticaionRsp.cause = cause;

    memcpy(msg.p.RFCAuthenticaionRsp.bd, bd, BD_ADDR_SIZE);

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

#if (F_BT_SCO)
/**
 * @brief  send sco connection Indicate
 *
 * @param  remote_bd
 *
 * @return
 *
 */
void blueAPI_Send_SCOConInd(LPCBYTE remote_BD)
{
    TBlueAPI_UsMessage msg;

    msg.Command = blueAPI_EventSCOConInd;

    memcpy(msg.p.SCOConInd.remote_BD, remote_BD, BD_ADDR_SIZE);

    BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE, "blueAPI:>>> SCOConInd BD[%s]",
        BLUEAPI_TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE, remote_BD));

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

/**
 * @brief  send sco connection response
 *
 * @param  remote_BD
 * @param  status
 *
 * @return
 *
 */
void blueAPI_Send_SCOConRsp(LPCBYTE remote_BD, uint8_t status)
{
    TBlueAPI_UsMessage msg;

    msg.Command = blueAPI_EventSCOConRsp;
    memcpy(msg.p.SCOConRsp.remote_BD, remote_BD, BD_ADDR_SIZE);
    msg.p.SCOConRsp.status = status;

    BLUEAPI_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_TRACE, "blueAPI:>>> SCOConRsp BD[%s] status[%d]",
        BLUEAPI_TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE,remote_BD), status);

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

/**
 * @brief  send sco connection indicate
 *
 * @param  remote_BD
 * @param  airMode
 *
 * @return
 *
 */
void blueAPI_Send_SCOConActInd(LPCBYTE remote_BD, uint8_t airMode)
{
    TBlueAPI_UsMessage msg;

    msg.Command = blueAPI_EventSCOConActInd;
    msg.p.SCOConActInd.airMode = airMode;

    memcpy(msg.p.SCOConActInd.remote_BD, remote_BD, BD_ADDR_SIZE);

    BLUEAPI_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_TRACE, "blueAPI:>>> SCOConActInd BD[%s] airMode[%d]",
        BLUEAPI_TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE, remote_BD), airMode);

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

/**
 * @brief  send sco disconnect indicate
 *
 * @param  pBlueAPIdata:
 *
 * @return
 *
 */
void blueAPI_Send_SCODiscInd(LPCBYTE remote_BD)
{
    TBlueAPI_UsMessage msg;

    msg.Command = blueAPI_EventSCODiscInd;
    memcpy(msg.p.SCODiscInd.remote_BD, remote_BD, BD_ADDR_SIZE);

    BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE, "blueAPI:>>> SCODisconnect BD[%s]",
        BLUEAPI_TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE, remote_BD));

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}
#endif

#if F_BT_LE_BT41_SUPPORT
void blueAPI_Send_CreateLEDataChannelRsp(uint16_t local_MDL_ID, uint16_t channel,
    uint16_t subCause)
{
    TBlueAPI_UsMessage msg = {0};
    TBlueAPI_Cause cause = blueAPI_L2CAPConvertCause(subCause);

    msg.Command = blueAPI_EventCreateLEDataChannelRsp;
    msg.p.CreateLEDataChannelRsp.local_MDL_ID = local_MDL_ID;
    msg.p.CreateLEDataChannelRsp.channel = channel;
    msg.p.CreateLEDataChannelRsp.cause = cause;
    msg.p.CreateLEDataChannelRsp.subCause = subCause;

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

void blueAPI_Send_CreateLEDataChannelInd (uint16_t local_MDL_ID, uint16_t channel)
{
    TBlueAPI_UsMessage msg = {0};

    msg.Command = blueAPI_EventCreateLEDataChannelInd;
    msg.p.CreateLEDataChannelInd.local_MDL_ID = local_MDL_ID;
    msg.p.CreateLEDataChannelInd.channel = channel;

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

void blueAPI_Send_DisconnectLEDataChannelRsp (uint16_t local_MDL_ID,
    uint16_t channel, uint16_t subCause)
{
    TBlueAPI_UsMessage msg = {0};
    TBlueAPI_Cause cause = blueAPI_L2CAPConvertCause(subCause);

    msg.Command = blueAPI_EventDisconnectLEDataChannelRsp;
    msg.p.DisconnectLEDataChannelRsp.local_MDL_ID = local_MDL_ID;
    msg.p.DisconnectLEDataChannelRsp.channel = channel;
    msg.p.DisconnectLEDataChannelRsp.cause = cause;
    msg.p.DisconnectLEDataChannelRsp.subCause= subCause;

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

void blueAPI_Send_DisconnectLEDataChannelInd(uint16_t local_MDL_ID, 
    uint16_t channel, uint16_t subCause)
{
    TBlueAPI_UsMessage msg = {0};
    TBlueAPI_Cause cause = blueAPI_L2CAPConvertCause(subCause);

    msg.Command = blueAPI_EventDisconnectLEDataChannelInd;
    msg.p.DisconnectLEDataChannelInd.local_MDL_ID = local_MDL_ID;
    msg.p.DisconnectLEDataChannelInd.channel = channel;
    msg.p.DisconnectLEDataChannelInd.cause = cause;
    msg.p.DisconnectLEDataChannelInd.subCause = subCause;

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

void blueAPI_Send_SendLEFlowControlCreditRsp(uint16_t local_MDL_ID,
    uint16_t channel, uint16_t subCause)
{
    TBlueAPI_UsMessage msg = {0};
    TBlueAPI_Cause cause = blueAPI_L2CAPConvertCause(subCause);

    msg.Command = blueAPI_EventSendLEFlowControlCreditRsp;
    msg.p.SendLEFlowControlCreditRsp.local_MDL_ID = local_MDL_ID;
    msg.p.SendLEFlowControlCreditRsp.channel = channel;
    msg.p.SendLEFlowControlCreditRsp.cause = cause;
    msg.p.SendLEFlowControlCreditRsp.subCause = subCause;

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

void blueAPI_Send_LEDataRsp(uint16_t local_MDL_ID, uint16_t channel, uint16_t subCause)
{
    TBlueAPI_UsMessage msg = {0};
    TBlueAPI_Cause cause = blueAPI_L2CAPConvertCause(subCause);

    msg.Command = blueAPI_EventLEDataRsp;
    msg.p.LEDataRsp.local_MDL_ID = local_MDL_ID;
    msg.p.LEDataRsp.channel = channel;
    msg.p.LEDataRsp.cause = cause;
    msg.p.LEDataRsp.subCause = subCause;

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

BOOL blueAPI_Send_LEDataInd(PVOID pBuffer, uint16_t offset, uint16_t channel,
    uint16_t length, uint16_t local_MDL_ID)
{
    PBlueAPI_UsMessage pMsg = (PBlueAPI_UsMessage)pBuffer;
    PBlueAPI_LEDataInd  pLEDataInd = &pMsg->p.LEDataInd;

    pMsg->Command = blueAPI_EventLEDataInd;
    pMsg->Length = offset + length;

    pLEDataInd->local_MDL_ID = local_MDL_ID;
    pLEDataInd->channel = channel;
    pLEDataInd->valueLength = length;
    pLEDataInd->gap = offset - offsetof(TBlueAPI_UsMessage, p.LEDataInd.data);

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, pMsg);

    return FALSE;
}

void blueAPI_Send_LEDataChannelParameterInfo(uint16_t local_MDL_ID, uint16_t channel, 
    uint16_t remote_mtu, uint16_t remote_mps, uint16_t remote_initial_credits, uint16_t tx_size)
{
    TBlueAPI_UsMessage msg = {0};
    
    msg.Command = blueAPI_EventLEDataChannelParameterInfo;
    msg.p.LEDataChannelParameterInfo.local_MDL_ID = local_MDL_ID;
    msg.p.LEDataChannelParameterInfo.channel = channel;
    msg.p.LEDataChannelParameterInfo.remote_mtu = remote_mtu;
    msg.p.LEDataChannelParameterInfo.remote_mps = remote_mps;
    msg.p.LEDataChannelParameterInfo.remote_initial_credits = remote_initial_credits;
    msg.p.LEDataChannelParameterInfo.maxDsCredits = tx_size;

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

void blueAPI_Send_LEDataChannelCreditsAlertInfo(uint16_t local_MDL_ID,
    uint16_t channel)
{
    TBlueAPI_UsMessage msg = {0};
    
    msg.Command = blueAPI_EventLEDataChannelCreditsAlertInfo;
    msg.p.LEDataChannelCreditsAlertInfo.local_MDL_ID = local_MDL_ID;
    msg.p.LEDataChannelCreditsAlertInfo.channel = channel;

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}

void blueAPI_Send_LEPsmSecuritySetRsp(TBlueAPI_Cause cause)
{
    TBlueAPI_UsMessage msg = {0};

    msg.Command = blueAPI_EventLEPsmSecuritySetRsp;
    msg.p.LEPsmSecuritySetRsp.cause = cause;

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, NULL, &msg);
}
#endif

void blueAPI_Handle_ConnectMDLReq(PBlueAPI_Data pBlueAPIdata, PBlueAPI_DsMessage pReqMsg)
{
   PBlueAPI_ConnectMDLReq      pConnectMDLReq = &pReqMsg->p.ConnectMDLReq;
   PBlueAPI_MCL                pMCL           = NULL;
   PBlueAPI_MDL                pMDL;
   uint16_t                        remote_Data_PSM    = 0;
   TBlueAPI_Cause              cause              = blueAPI_CauseSuccess;

   switch (pConnectMDLReq->linkConfigType)
   {
     case blueAPI_LinkConfigGATT:
       remote_Data_PSM    = BLUEFACE_PSM_GATT;
       if ((pConnectMDLReq->remote_BD_Type != blueAPI_RemoteBDTypeClassic)
           && (pConnectMDLReq->remote_BD_Type != blueAPI_RemoteBDTypeLEPublic)
           && (pConnectMDLReq->remote_BD_Type != blueAPI_RemoteBDTypeLERandom)
           && (pConnectMDLReq->remote_BD_Type != blueAPI_RemoteBDTypeLEResolved)
          )
       {
         cause = blueAPI_CauseInvalidParameter;
       }
       break;

     default:
       cause = blueAPI_CauseInvalidParameter;
       break;
   }

   do
   {
     if (cause != blueAPI_CauseSuccess)
       break;

     /* search for MCL */
     pMCL = blueAPI_MCLFindByBD(pConnectMDLReq->remote_BD,
                                blueAPI_LinkConfigType2MCLType(pConnectMDLReq->linkConfigType));

     /* no MCL found */
     if (pMCL == NULL)
     {
       pMCL = blueAPI_MCLAllocate(pConnectMDLReq->remote_BD,
                                  pConnectMDLReq->remote_BD_Type,
                                  blueAPI_LinkConfigType2MCLType(pConnectMDLReq->linkConfigType)
                                  );
       /* allocation failed */
       if (pMCL == NULL)
       {
         cause = blueAPI_CauseResourceError;
         break;
       }
     }

     pBlueAPIdata->pActiveMCL = pMCL;

     if (blueAPI_MCLInTransition(pMCL))
     {
       /* queue message */
       if (blueAPI_QueueCOMCommand(pMCL, pReqMsg))
       {
         BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE,
                                   "!!blueAPI: can not queue COM command 0x%x",
                                   pReqMsg->Command
                                   );

         blueAPI_Send_ConnectMDLRsp(
                                    pReqMsg,
                                    NULL, /* !!! */
                                    pConnectMDLReq->remote_BD,
                                    pConnectMDLReq->remote_BD_Type,
                                    0,
                                    blueAPI_CauseResourceError
                                    );
       }
       return;
     }

     BLUEAPI_TRACE_PRINTF_5(BLUEAPI_TRACE_MASK_TRACE,
                               "blueAPI:<<< ConnectMDLReq BD[%s] bdType[%d] loc_MDEP_ID[%d] com_PSM[0x%X] data_PSM[0x%X] confType[0x%x]",
                               BLUEAPI_TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE, pConnectMDLReq->remote_BD),
                               pConnectMDLReq->remote_BD_Type,
                               pConnectMDLReq->local_MDEP_ID,
                               remote_Data_PSM,
                               pConnectMDLReq->linkConfigType
                               );

     if (pMCL->mclType == blueAPI_MCLType_GATT)
     {
       /* fake ctrl connected message to app */
       if (pMCL->state != blueAPI_DS_DataConnected)
       {
         blueAPI_MCLSetState(pMCL, blueAPI_DS_ControlConnected);
       }
     }

       pMDL = blueAPI_MDLAllocate(pMCL, pConnectMDLReq->linkConfigType);

       if (pMDL == NULL)
       {
         cause = blueAPI_CauseResourceError;
         break;
       }

       /* set pending DS COM context                                   */
	   pMCL->DS_CommandInProgress = (TBlueAPI_Command)pReqMsg->Command;
	   pMCL->DS_CommandMsg        = pReqMsg;

	   blueAPI_Send_CreateMDLInd(pMCL, pMDL);
   } while (0);

   if (cause != blueAPI_CauseSuccess)
   {
     blueAPI_Send_ConnectMDLRsp(
                                pReqMsg,
                                pMCL,
                                pConnectMDLReq->remote_BD,
                                pConnectMDLReq->remote_BD_Type,
                                0,
                                cause
                                );
   }
}

/****************************************************************************/
/* void blueAPI_Handle_DisconnectMDLReq                                    */
/* (                                                                        */
/*    PBlueAPI_Data                  pBlueAPIdata                                */
/*    PBlueAPI_Message               pReqMsg                                   */
/* )                                                                        */
/****************************************************************************/

void blueAPI_Handle_DisconnectMDLReq(PBlueAPI_Data pBlueAPIdata, PBlueAPI_DsMessage pReqMsg)
{
   PBlueAPI_DisconnectMDLReq pDisconnectMDLReq = &pReqMsg->p.DisconnectMDLReq;
   PBlueAPI_LinkDescriptor        pLinkContext;
   PBlueAPI_MCL                   pMCL              = NULL;
   PBlueAPI_MDL                   pMDL;

   BLUEAPI_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_TRACE,
                             "blueAPI:<<< DisconnectMDLReq local_mdl[%d] cause[0x%x]",
                             pDisconnectMDLReq->local_MDL_ID,
                             pDisconnectMDLReq->cause
                             );

   pLinkContext = blueAPI_ChannelFindByLocal_MDL_ID(pDisconnectMDLReq->local_MDL_ID);

   if(pLinkContext)
   {  pMDL = pLinkContext->pMDL;
      pMCL = pLinkContext->pMCL;

      pBlueAPIdata->pActiveMCL = pMCL;

      if(blueAPI_MCLInTransition(pMCL))
      {
        if ((pMCL->DS_CommandInProgress == blueAPI_EventConnectMDLReq) &&
            (pMCL->state == blueAPI_DS_DataConnecting) &&
            (pLinkContext->linkState == blueAPI_Connecting)
           )
        {
          BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE,
                                    "blueAPI: ABORT outgoing connect on local_mdl[%d]",
                                    pDisconnectMDLReq->local_MDL_ID
                                    );

          blueAPI_ChannelDisconnect(pBlueAPIdata, pLinkContext, TRUE);
          /* handle as disconnect during connect */
          pLinkContext->linkState = blueAPI_Connecting;

          blueAPI_Send_DisconnectMDLRsp(
                                        pReqMsg,
                                        NULL,
                                        pDisconnectMDLReq->local_MDL_ID,
                                        blueAPI_CauseSuccess
                                        );
        }
        /* queue message */
        else if (blueAPI_QueueCOMCommand(pMCL, pReqMsg))
         {
            blueAPI_Send_DisconnectMDLRsp(
                                           pReqMsg,
                                           NULL,
                                           pDisconnectMDLReq->local_MDL_ID,
                                           blueAPI_CauseResourceError
                                           );
         }
         return;
      }

      if (pDisconnectMDLReq->cause == blueAPI_CauseConnectionPaused)
      {

          blueAPI_Send_DisconnectMDLRsp(
                                        pReqMsg,
                                        pMCL,
                                        pDisconnectMDLReq->local_MDL_ID,
                                        blueAPI_CauseInvalidParameter
                                        );
           return;
      }

      switch(pDisconnectMDLReq->cause)
      {  case blueAPI_CauseSuccess: /*-----------------------------------------*/
            /* legacy support, legcy application might use success instead  */
            /* of blueAPI_CauseConnectionDisconnect... patch message           */
            pDisconnectMDLReq->cause = blueAPI_CauseConnectionDisconnect;
         case blueAPI_CauseConnectionDisconnect: /*----------------------------*/
         case blueAPI_CauseConnectionPaused: /*--------------------------------*/
            /* set pending DS COM context                                   */
            pMCL->DS_CommandInProgress = (TBlueAPI_Command)pReqMsg->Command;
						pMCL->DS_CommandMsg        = pReqMsg;

            blueAPI_Send_DisconnectMDLInd(pMCL, pMDL, pDisconnectMDLReq->cause);
            break;
         default: /*--------------------------------------------------------*/
            blueAPI_Send_DisconnectMDLRsp(
                                           pReqMsg,
                                           pMCL,
                                           pDisconnectMDLReq->local_MDL_ID,
                                           blueAPI_CauseInvalidParameter
                                           );
            break;
      }
   } else /* 'disconnect' idle MDL... just delete                           */
   {
      BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE,
                                "blueAPI: link for Local_MDL_ID[%d] not found",
                                pDisconnectMDLReq->local_MDL_ID
                                );

      pMDL = blueAPI_MDLFindByLocal_MDL_ID(pBlueAPIdata,pDisconnectMDLReq->local_MDL_ID);

      if(pMDL&&pDisconnectMDLReq->cause==blueAPI_CauseConnectionDisconnect)
      {
         BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE,
                                   "blueAPI: delete Idle MDL"
                                   );

         blueAPI_Send_DeleteMDLInfo(pMDL);
         blueAPI_MDLRelease(pMDL);
         blueAPI_Send_DisconnectMDLRsp(
                                        pReqMsg,
                                        pMCL,
                                        pDisconnectMDLReq->local_MDL_ID,
                                        blueAPI_CauseSuccess
                                        );
      } else
      {
         BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE,
                                   "!!blueAPI: MDL for Local_MDL_ID[%d] not found",
                                   pDisconnectMDLReq->local_MDL_ID
                                   );

         blueAPI_Send_DisconnectMDLRsp(
                                        pReqMsg,
                                        pMCL,
                                        pDisconnectMDLReq->local_MDL_ID,
                                        blueAPI_CauseInvalidParameter
                                        );
      }
   }
}

/****************************************************************************/
/* void blueAPI_Handle_DisconnectMDLConf                                   */
/* (                                                                        */
/*    PBlueAPI_Data                   pBlueAPIdata                               */
/*    PBlueAPI_DisconnectMDLConf pDisconnectMDLConf                       */
/* )                                                                        */
/****************************************************************************/
void blueAPI_Handle_DisconnectMDLConf(PBlueAPI_Data pBlueAPIdata, PBlueAPI_DisconnectMDLConf pDisconnectMDLConf)
{
   PBlueAPI_LinkDescriptor pLinkContext = blueAPI_ChannelFindByLocal_MDL_ID(pDisconnectMDLConf->local_MDL_ID);
   PBlueAPI_MDL            pMDL         = blueAPI_MDLFindByLocal_MDL_ID(pBlueAPIdata,pDisconnectMDLConf->local_MDL_ID);
   PBlueAPI_MCL            pMCL;

   BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE,
                            "blueAPI:<<< DisconnectMDLConf local_mdl[%d]",
                            pDisconnectMDLConf->local_MDL_ID
                            );

   if(!pMDL)
   {
      BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE,
                                "!!blueAPI: unknown locMDL_ID[%d]",
                                pDisconnectMDLConf->local_MDL_ID
                                );
#if SEND_INT_EVENT_INFO /*app do not care of event info!!*/
      blueAPI_Send_InternalEventInfo(blueAPI_InternalEventInvalidDisconnectMDLConf,
                                      BLUE_API_GENERATE_EVENT_ID,
                                      blueAPI_CauseInvalidParameter
                                      );
#endif
      return;
   }

   pMCL = blueAPI_MCLFindByBD(pMDL->remoteBd,
                              blueAPI_LinkConfigType2MCLType(pMDL->linkConfigType)
                              );

   if(!pMCL)
   {
      BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE,
                                "!!blueAPI: MCL not found => panic!"
                                );
#if SEND_INT_EVENT_INFO /*app do not care of event info!!*/
      blueAPI_Send_InternalEventInfo(blueAPI_InternalEventInvalidDisconnectMDLConf,
                                      BLUE_API_GENERATE_EVENT_ID,
                                      blueAPI_CauseInvalidParameter
                                      );
#endif
      return;
   }

   if (pMCL->DS_CommandInProgress == blueAPI_EventDisconnectMDLReq)
   {
     /* App initiated a disconnect                                         */
      PBlueAPI_DisconnectMDLReq pDisconnectMDLReq = &pMCL->DS_CommandMsg->p.DisconnectMDLReq;

      /* check if App response is valid                                     */
      if (pDisconnectMDLConf->local_MDL_ID != pDisconnectMDLReq->local_MDL_ID)
      {
#if SEND_INT_EVENT_INFO
        blueAPI_Send_InternalEventInfo(blueAPI_InternalEventInvalidDisconnectMDLConf,
                                         BLUE_API_GENERATE_EVENT_ID,
                                         blueAPI_CauseInvalidParameter
                                         );
#endif
         return;
      }

/* (F_BLUE_API_RECONNECT_SUPPORT) */
      {
        if (pLinkContext->linkState != blueAPI_Disconnecting)
        {
          blueAPI_ChannelDisconnect(pBlueAPIdata, pLinkContext, FALSE);
        }
        else
        {
          btApiBT_DISC_RESP(pLinkContext->handle);
          blueAPI_ChannelRelease(pBlueAPIdata, pLinkContext);

          blueAPI_Send_DisconnectMDLRsp(
                                        pMCL->DS_CommandMsg,
                                        pMCL,
                                        pMDL->local_MDL_ID,
                                        blueAPI_CauseSuccess
                                        );

          switch (pMCL->mclType)
          {
            case blueAPI_MCLType_GATT:
              /* RFCOMM, GATT, L2CAP have no reconnect, delete the MDL now */
              blueAPI_Send_DeleteMDLInfo( pMDL);
              blueAPI_MDLRelease(pMDL);
              blueAPI_MCLSetState(pMCL, blueAPI_DS_Idle);
              break;

            default:
              break;
          }
        }
      }
   }
   else if (pMCL->US_CommandInProgress == blueAPI_EventDisconnectMDLInd)
   {  /* remode side initiated a disconnect                                 */
      PBlueAPI_DisconnectMDLInd pDisconnectInd = (PBlueAPI_DisconnectMDLInd)&pMCL->US_CommandData;

      /* check if App response is valid                                     */
      if (pDisconnectMDLConf->local_MDL_ID != pDisconnectInd->local_MDL_ID)
      {
 #if SEND_INT_EVENT_INFO
        blueAPI_Send_InternalEventInfo(blueAPI_InternalEventInvalidDisconnectMDLConf,
                                         BLUE_API_GENERATE_EVENT_ID,
                                         blueAPI_CauseInvalidParameter
                                         );
 #endif
         return;
      }

      /* if there is a data channel => disconnect it                        */
      if (pLinkContext)
      {  if (pLinkContext->linkState != blueAPI_Disconnecting)
         {  blueAPI_ChannelDisconnect(pBlueAPIdata, pLinkContext, FALSE);
         } else
         {  /* terminate BT_DISCONNECT  from remote device                  */
            btApiBT_DISC_RESP(pLinkContext->handle);
            blueAPI_ChannelRelease(pBlueAPIdata,pLinkContext);

            switch (pMCL->mclType)
            {
              case blueAPI_MCLType_GATT:
                /* RFCOMM, GATT, L2CAP have no reconnect, delete the MDL now */
                blueAPI_Send_DeleteMDLInfo(pMDL);
                blueAPI_MDLRelease(pMDL);
                blueAPI_MCLSetState(pMCL, blueAPI_DS_Idle);
                break;

              default:
                break;
            }
         }
      } else if (pDisconnectInd->cause == blueAPI_CauseConnectionDisconnect)
      {  /* no data channel left ==> local handling                         */
         blueAPI_Send_DeleteMDLInfo(pMDL);
         blueAPI_MDLRelease(pMDL);
         pMDL = NULL;
      }
   } else
   {
#if SEND_INT_EVENT_INFO
    blueAPI_Send_InternalEventInfo(blueAPI_InternalEventInvalidDisconnectMDLConf,
                                      BLUE_API_GENERATE_EVENT_ID,
                                      blueAPI_CauseInvalidState
                                      );
#endif
      return;
   }

   blueAPI_MCL_US_ContextClear(pMCL);

   pBlueAPIdata->pActiveMCL = pMCL;
}

/* F_BLUE_API_RECONNECT_SUPPORT */
/**
* @brief  create mdl confirm
*
* @param  pBuffer:
* @param  blueAPIHandle: 
* @param  local_MDL_ID: 
* @param  linkConfigType
* @param  maxTPDUusCredits
* @param  cause
*
* @return 
*
*/
bool blueAPI_IntlCreateMDLConf(PBlueAPI_Data pBlueAPIdata,
                           uint16_t        local_MDL_ID,
                           TBlueAPI_LinkConfigType linkConfigType,
                           uint8_t         maxTPDUusCredits,
                           TBlueAPI_Cause  cause
                          )
{
	PBlueAPI_LinkDescriptor    pLinkContext = NULL;
	PBlueAPI_MDL               pMDL         = blueAPI_MDLFindByLocal_MDL_ID(pBlueAPIdata, local_MDL_ID);
	TBlueAPI_Cause             check_cause  = blueAPI_CauseAccept;
	PBlueAPI_MCL               pMCL         = NULL;
    uint16_t             check_MDL_ID;

	BLUEAPI_TRACE_PRINTF_4(BLUEAPI_TRACE_MASK_TRACE,
	                         "blueAPI:<<< CreateMDLConf local_mdl[%d] confType[%d] usCred[%d] cause[%d]",
	                         local_MDL_ID,
	                         linkConfigType,
	                         maxTPDUusCredits,
	                         cause
	                         );

	if(!pMDL)
	{
		BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE,
		                        "!!blueAPI: unknown locMDL_ID[%d]",
		                        local_MDL_ID
		                        );
#if SEND_INT_EVENT_INFO
		blueAPI_Send_InternalEventInfo(blueAPI_InternalEventInvalidCreateMDLConf,
		                              BLUE_API_GENERATE_EVENT_ID,
		                              blueAPI_CauseInvalidParameter
		                              );
#endif
		return FALSE;
	}

	pMCL = blueAPI_MCLFindByBD(pMDL->remoteBd,
	                          blueAPI_LinkConfigType2MCLType(pMDL->linkConfigType)
	                          );

	if(!pMCL)
	{
		BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE,
		                        "!!blueAPI: unknown MCL bd[%s]",
		                        BLUEAPI_TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE, pMDL->remoteBd)
		                        );
#if SEND_INT_EVENT_INFO
		blueAPI_Send_InternalEventInfo(blueAPI_InternalEventInvalidCreateMDLConf,
		                              BLUE_API_GENERATE_EVENT_ID,
		                              blueAPI_CauseInvalidParameter
		                              );
#endif
		return FALSE;
	}

    check_MDL_ID = pMCL->US_CommandData.CreateMDLInd.local_MDL_ID;

	if (local_MDL_ID != check_MDL_ID)
	{  
#if SEND_INT_EVENT_INFO   
	    blueAPI_Send_InternalEventInfo(blueAPI_InternalEventInvalidCreateMDLConf,
	                                  BLUE_API_GENERATE_EVENT_ID,
	                                  blueAPI_CauseInvalidParameter
	                                  );
#endif
	    return FALSE;
	}

	/* check if config type from App is ok                                   */
	if (cause == blueAPI_CauseAccept)
	{
	    if (pMCL->mclType != blueAPI_LinkConfigType2MCLType(linkConfigType))
	    {
	        BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE,
	                                 "blueAPI: App responded with wrong Config type..."
	                                 );

	        check_cause = blueAPI_CauseInvalidParameter;
	    }
	    else
	    {
		    if (pMDL->linkConfigType != blueAPI_LinkConfigHDPDontCare)
		    {  
		        if(linkConfigType!=pMDL->linkConfigType)
		        {
		            BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE,
		                                   "blueAPI: App responded with wrong Config type..."
		                                   );

		            check_cause=blueAPI_CauseInvalidParameter;
		        }
		    }
	    }
	}


	switch(pMCL->DS_CommandInProgress)
	{
	case blueAPI_EventConnectMDLReq: 
		blueAPI_MCL_US_ContextClear(pMCL);

		if(cause == blueAPI_CauseAccept)
		{ 
       
			if(check_cause==blueAPI_CauseAccept) /* config type valid => continue    */
               
			{  
				pMDL->linkConfigType = linkConfigType;
				/* start new data channel connection */
				if(pMCL->state != blueAPI_DS_Idle) /* still an active MCL */
				{  
			    	pLinkContext = blueAPI_ChannelAllocate(pMCL,
				                                      blueAPI_SubRoleInitiator,
				                                      pMDL->linkConfigType
				                                      );
				    if (pLinkContext)
				    {  
				        pLinkContext->pMDL                = pMDL;
				        pLinkContext->pMDL->maxUsCredits  = maxTPDUusCredits;

				        blueAPI_ChannelConnect(pBlueAPIdata, pMCL, pLinkContext);
				    } else /* no linkcontext created... silent discard MDL */
				    {
				        BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE,
				                               "blueAPI: out of channel context..."
				                               );

				        check_cause=blueAPI_CauseResourceError;
				    }
				} 
				else /* MCL disconnected while create...   */
				{
					BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE,
					                        "blueAPI: MCL already disconnected..."
					                        );

					check_cause=blueAPI_CauseConnectionDisconnect;
				}             
			} /* end if cause == blueAPI_CauseAccept                        */
			if (check_cause != blueAPI_CauseAccept)
			{
				switch (pMCL->mclType)
				{
				case blueAPI_MCLType_GATT:
					blueAPI_Send_ConnectMDLRsp(
					                         pMCL->DS_CommandMsg,
					                         pMCL,
					                         pMDL->remoteBd,
					                         pMDL->remoteBdType,
					                         pMDL->local_MDL_ID,
					                         check_cause
					                         );
					blueAPI_Send_DeleteMDLInfo(pMDL);
					blueAPI_MDLRelease(pMDL);
					break;

				default:
					break;
				}
			} /* end if cause == blueAPI_CauseAccept                           */
		}
        else /* rejected by application...                               */
        {
           switch (pMCL->mclType)
           {
             case blueAPI_MCLType_GATT:
               blueAPI_Send_ConnectMDLRsp(
                                          pMCL->DS_CommandMsg,
                                          pMCL,
                                          pMDL->remoteBd,
                                          pMDL->remoteBdType,
                                          0,
                                          blueAPI_CauseReject
                                          );
               blueAPI_MDLRelease(pMDL);
               break;
             default:
               break;
           }
         }
		break;
	default:
#if SEND_INT_EVENT_INFO
		blueAPI_Send_InternalEventInfo(blueAPI_InternalEventInvalidCreateMDLConf,
		                             BLUE_API_GENERATE_EVENT_ID,
		                             blueAPI_CauseInvalidState
		                             );
#endif
		blueAPI_MCL_US_ContextClear(pMCL);
		break;
	    case blueAPI_EventIdle:
	    {  /* incoming request  */
			switch(pMCL->US_CommandInProgress)
			{  
	        case blueAPI_EventCreateMDLInd:
				blueAPI_MCL_US_ContextClear(pMCL);
				pLinkContext = blueAPI_ChannelFindByLocal_MDL_ID(pMDL->local_MDL_ID);

				if((pMCL->state!=blueAPI_DS_Idle) && (!pLinkContext))
				{
					BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE,
					                        "!!blueAPI: no link found for locMDL_ID[%d] => panic!",
					                        pMDL->local_MDL_ID
					                        );
#if SEND_INT_EVENT_INFO 
					blueAPI_Send_InternalEventInfo(blueAPI_InternalEventInvalidCreateMDLConf,
					                              BLUE_API_GENERATE_EVENT_ID,
					                              blueAPI_CauseInvalidState
					                             );
#endif
					return FALSE;
				}

				switch(cause)
				{  
				case blueAPI_CauseAccept:
					if(check_cause == blueAPI_CauseAccept)/*config type valid => conti*/
					{  
		                if(pMCL->state != blueAPI_DS_Idle)/* still an active MCL*/
		                {  /* adopt config type to App response   */
		                    pMDL->linkConfigType = linkConfigType;

		                    blueAPI_ChannelSetConfiguration(pBlueAPIdata, pLinkContext, pMDL->linkConfigType);
		                    pLinkContext->pMDL->maxUsCredits  = maxTPDUusCredits;

		                    switch (pMCL->mclType)
		                    {
							case blueAPI_MCLType_GATT:
								blueAPI_MCLSetState(pMCL, blueAPI_DS_DataListening);
								blueAPI_ChannelAccept(pBlueAPIdata, pLinkContext, TRUE);
								break;				   
		                    /*F_BLUE_API_HDP_SUPPORT*/
		                     default:
		                      break;
		                    }
		                    //                  TRACE_DUMP_LINK(pLinkContext,pBlueAPIDumpPrefix);
		                } else /* MCL disconnected while create...          */
			            {  
				            blueAPI_Send_DeleteMDLInfo(pMDL);

				            switch (pMCL->mclType)
				            {
							case blueAPI_MCLType_GATT:	 	
								blueAPI_ChannelAccept(pBlueAPIdata, pLinkContext, FALSE);
								break;
							default:
								break;
				            }
				            blueAPI_MDLRelease(pMDL);
						}
		            } 
					else /* config type not valid => cancel              */
					{  
						blueAPI_Send_DeleteMDLInfo(pMDL);
	                    switch (pMCL->mclType)
	                    {
						case blueAPI_MCLType_GATT:		  	
							blueAPI_ChannelAccept(pBlueAPIdata, pLinkContext, FALSE);
							break;
						default:
							break;
	                    }
	                    blueAPI_MDLRelease(pMDL);

	                    if(pMCL->state != blueAPI_DS_Idle)/* still an active MCL*/
	                    {  /* otherwhise it is done while CC disconnect...  */
							blueAPI_ChannelRelease(pBlueAPIdata, pLinkContext);
	                    }
					}
				break;
	              default: /*                                               */
	                 BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE,
	                                           "blueAPI: invalid cause!!"
	                                           );
#if SEND_INT_EVENT_INFO
	                 blueAPI_Send_InternalEventInfo(blueAPI_InternalEventInvalidCreateMDLConf,
	                                                 BLUE_API_GENERATE_EVENT_ID,
	                                                 blueAPI_CauseInvalidParameter
	                                                 );
                 case blueAPI_CauseReject: /*                                 */
                     if(pMCL->state!=blueAPI_DS_Idle)/* still an active MCL   */
                     {

                        switch(pMCL->mclType)
                        {
                          case blueAPI_MCLType_GATT:
                            blueAPI_ChannelAccept(pBlueAPIdata, pLinkContext, FALSE);
                            break;
                          default:
                            break;
                        }
                     }

                     blueAPI_ChannelRelease(pBlueAPIdata, pLinkContext);
                     blueAPI_Send_DeleteMDLInfo(pMDL);
                     blueAPI_MDLRelease(pMDL);
                     break;
#endif
				} /* switch pCreateMDLConf->cause */
				break;
	        default: 
#if SEND_INT_EVENT_INFO/*app not used!!!*/						
	           blueAPI_Send_InternalEventInfo(blueAPI_InternalEventInvalidCreateMDLConf,
	                                           BLUE_API_GENERATE_EVENT_ID,
	                                           blueAPI_CauseInvalidState
	                                           );
#endif						
	           blueAPI_MCL_US_ContextClear(pMCL);
	           break;
	        } /* end case US_COM_CommandInProgress */
	  } /* case blueAPI_Event_No_Event             */
	} /* end case DS_COM_CommandInProgress       */

	pBlueAPIdata->pActiveMCL = pMCL; 
	return TRUE;

}

/****************************************************************************/
/* void blueAPI_Handle_CreateMDLConf                                        */
/* (                                                                        */
/*    PBlueAPI_Data               pBlueAPIdata                              */
/*    PBlueAPI_CreateMDLConf pCreateMDLConf                                 */
/* )                                                                        */
/****************************************************************************/
void blueAPI_Handle_CreateMDLConf(PBlueAPI_Data pBlueAPIdata, PBlueAPI_CreateMDLConf pCreateMDLConf)
{
    blueAPI_IntlCreateMDLConf(pBlueAPIdata, pCreateMDLConf->local_MDL_ID, 
                                     blueAPI_LinkConfigGATT, pCreateMDLConf->maxTPDUusCredits,
                                     pCreateMDLConf->cause);
}

/****************************************************************************/
/* void blueAPI_Handle_RegisterReq                                          */
/****************************************************************************/
void blueAPI_Handle_RegisterReq(PBlueAPI_Data pBlueAPIdata, PBlueAPI_DsMessage pReqMsg)
{
   PBlueAPI_RegisterReq pRegisterReq = &pReqMsg->p.RegisterReq;
   TBlueAPI_Cause       cause        = blueAPI_CauseSuccess;
   PBlueAPI_App         pMedApp      = NULL;


   pMedApp = blueAPI_AllocCOMApp(pBlueAPIdata);
   if (pMedApp == NULL)
   {
        cause = blueAPI_CauseResourceError;
   }

   if (cause != blueAPI_CauseSuccess)
   {
     TBlueAPI_App medAppTemp;

     /* we have no valid appcontext for the response, so we build one from scratch */
     medAppTemp.MDHmsgHandlerCallback   = pRegisterReq->MDHmsgHandlerCallback;
     medAppTemp.pBlueAPIdata            = pBlueAPIdata;

     blueAPI_Send_RegisterRsp(
                              &medAppTemp,
                              pReqMsg,
                              cause
                              );
     return;
   }

   pMedApp->MDHmsgHandlerCallback   = pRegisterReq->MDHmsgHandlerCallback;

   blueAPI_Send_RegisterRsp(
                             pMedApp,
                             pReqMsg,
                             blueAPI_CauseSuccess
                             );

   if (((pBlueAPIdata->state & BLUE_API_STATE_READY) == BLUE_API_STATE_READY) ||
       (pBlueAPIdata->stateCause != blueAPI_CauseSuccess)
      )
   {
     blueAPI_SendToAll_ActInfo(pBlueAPIdata, pBlueAPIdata->stateCause, FALSE);
   }
}

/****************************************************************************/
/* void blueAPI_Handle_ReleaseReq                                           */
/****************************************************************************/
void blueAPI_Handle_ReleaseReq(PBlueAPI_DsMessage pReqMsg)
{
    TBlueAPI_UsMessage msg;

    BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE, "blueAPI:<<< ReleaseReq");
    msg.Command = blueAPI_EventReleaseRsp;
    msg.p.ReleaseRsp.cause = blueAPI_CauseSuccess;
    pBlueAPIData->appDescriptorTable.used = FALSE;

    blueAPI_TgtSendEvent(pBlueAPIData->pActiveApp, pReqMsg, &msg);
}

/****************************************************************************/
/* void blueAPI_Handle_InquiryReq                                           */
/****************************************************************************/
void blueAPI_Handle_InquiryReq(PBlueAPI_Data pBlueAPIdata, PBlueAPI_DsMessage pReqMsg)
{
    uint8_t iac = IAC_GIAC_IDX;

    BLUEAPI_TRACE_PRINTF_3(BLUEAPI_TRACE_MASK_TRACE,
                            "blueAPI:<<< InquiryReq limited[%d] cancel[%d] timeout[0x%x]",
                            pReqMsg->p.InquiryReq.limitedInquiry,
                            pReqMsg->p.InquiryReq.cancelInquiry,
                            pReqMsg->p.InquiryReq.InquiryTimeout
                            );

    if ((pBlueAPIdata->SM_SDP_DS_Command == blueAPI_EventIdle) &&
        (pReqMsg->p.InquiryReq.cancelInquiry == FALSE)
        )
    {
        blueAPI_SDP_DS_ContextSet(pBlueAPIdata, pReqMsg);

        if (pReqMsg->p.InquiryReq.limitedInquiry)
        {
            iac = IAC_LIAC_IDX;
        }

        if (pReqMsg->p.InquiryReq.InquiryTimeout <= 0x30)
        {
            btApiBT_HCI_INQUIRY_REQ(pBlueAPIdata->AppHandle,
                            pReqMsg->p.InquiryReq.InquiryTimeout,
                            BT_DS_PDU_L2C_BYTE_COUNT / (BD_ADDR_SIZE + BT_CLASS_SIZE) - 1,
                            iac           /* idxIAC     */
                            );
        }
        else
        {
            blueAPI_Send_InquiryRsp(pReqMsg,
                                    pReqMsg->p.InquiryReq.cancelInquiry,
                                    blueAPI_CauseInvalidParameter
                                    );
        }
    }
    else if ((pBlueAPIdata->SM_SDP_DS_Command == blueAPI_EventInquiryReq) &&
            (pReqMsg->p.InquiryReq.cancelInquiry == TRUE)
            )
    {
        btApiBT_HCI_INQUIRY_REQ(pBlueAPIdata->AppHandle, 0, 0, iac);
        pBlueAPIdata->SM_SDP_DS_CommandMsg->p.InquiryReq.cancelInquiry = TRUE;
        blueAPI_Send_InquiryRsp(pReqMsg, TRUE, blueAPI_CauseSuccess);
    }
    else
    {
        blueAPI_Send_InquiryRsp(pReqMsg,
                                pReqMsg->p.InquiryReq.cancelInquiry,
                                blueAPI_CauseInvalidState
                                );
    }
}

/****************************************************************************/
/* void blueAPI_Handle_DeviceNameReq                                        */
/****************************************************************************/
void blueAPI_Handle_DeviceNameReq(PBlueAPI_Data pBlueAPIdata, PBlueAPI_DsMessage pReqMsg)
{
    PBlueAPI_DeviceNameReq pDeviceNameReq = &pReqMsg->p.DeviceNameReq;

    BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE,
                             "blueAPI:<<< DeviceNameReq BD[%s]",
                             BLUEAPI_TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE, pDeviceNameReq->remote_BD)
                             );

    if (pBlueAPIdata->SM_SDP_DS_Command == blueAPI_EventIdle)
    {
        blueAPI_SDP_DS_ContextSet(pBlueAPIdata, pReqMsg);
        btApiBT_HCI_NAME_REQ(pBlueAPIdata->AppHandle, pDeviceNameReq->remote_BD);
    }
    else
    {
        blueAPI_Send_DeviceNameRsp(pReqMsg, NULL, (uint8_t *)"", blueAPI_CauseInvalidState);
    }
}

/****************************************************************************/
/* void blueAPI_Handle_DIDDeviceConf                                        */
/****************************************************************************/
void blueAPI_Handle_DIDDeviceConf(PBlueAPI_Data pBlueAPIdata, PBlueAPI_DIDDeviceConf pDIDDeviceConf)
{
   BLUEAPI_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_TRACE,
                             "blueAPI:<<< DIDDeviceConf handle[0x%X] cause[0x%x]",
                             pDIDDeviceConf->serviceHandle,
                             pDIDDeviceConf->cause
                             );

   if ((pBlueAPIdata->SM_SDP_US_Command == blueAPI_EventDIDDeviceInd) &&
       (pBlueAPIdata->SM_SDP_US_Handle == pDIDDeviceConf->serviceHandle)
      )
   {
      pBlueAPIdata->SM_SDP_US_Command = blueAPI_EventIdle;
      pBlueAPIdata->SM_SDP_US_Handle  = 0x00000000;

      /* store first upstream error                                         */
      if ((pDIDDeviceConf->cause != blueAPI_CauseSuccess) &&
          (pBlueAPIdata->SM_SDP_DS_Cause == blueAPI_CauseSuccess)
         )
      {  pBlueAPIdata->SM_SDP_DS_Cause = pDIDDeviceConf->cause;
      }

      /* continue SDP actions                                               */
      switch(pBlueAPIdata->SM_SDP_DS_Command)
      {
         case blueAPI_EventSDPDiscoveryReq:
         {  
            PBlueAPI_SDPDiscoveryReq pProgress = &pBlueAPIdata->SM_SDP_DS_CommandMsg->p.SDPDiscoveryReq;

            pBlueAPIdata->SM_SDP_ServiceUUID = (uint16_t)pProgress->remote_MDEP_DataType;
            blueAPI_Send_BT_SDP_SEARCH_REQ(pBlueAPIdata, pBlueAPIdata->SM_SDP_ServiceUUID, 0);
            break;
         }

         case blueAPI_EventGATTSDPDiscoveryReq: /*----------------------------*/
         {
            PBlueAPI_GATTSDPDiscoveryReq pProgress = &pBlueAPIdata->SM_SDP_DS_CommandMsg->p.GATTSDPDiscoveryReq;
            pBlueAPIdata->SM_SDP_ServiceUUID = UUID_ATT;
            blueAPI_Send_BT_SDP_SEARCH_REQ(pBlueAPIdata, UUID_ATT, pProgress->remote_GATT_UUID);
            break;
         }
         default: /*--------------------------------------------------------*/
#if SEND_INT_EVENT_INFO
            blueAPI_Send_InternalEventInfo(blueAPI_InternalEventInvalidDIDDeviceConf,
                                           BLUE_API_GENERATE_EVENT_ID,
                                           blueAPI_CauseInvalidState
                                           );
#endif
            break;
      }
   } else /* message was not expected                                       */
   {
#if SEND_INT_EVENT_INFO
   	blueAPI_Send_InternalEventInfo(blueAPI_InternalEventInvalidDIDDeviceConf,
                                     BLUE_API_GENERATE_EVENT_ID,
                                     blueAPI_CauseInvalidState
                                     );
#endif
   }
}

/*(F_BLUE_API_HDP_SUPPORT)*/

/****************************************************************************/
/* void blueAPI_Handle_RadioModeSetReq                                      */
/****************************************************************************/
void blueAPI_Handle_RadioModeSetReq(PBlueAPI_Data pBlueAPIdata,PBlueAPI_DsMessage pReqMsg)
{
    PBlueAPI_RadioModeSetReq pRadioModeSetReq = (PBlueAPI_RadioModeSetReq)&pReqMsg->p;
    TBlueAPI_BRPageScanMode  pageMode;

    BLUEAPI_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_TRACE,
                            "blueAPI:<<< RadioModesetReq mode[%d] limited[%d]",
                            pRadioModeSetReq->localRadioMode,
                            pRadioModeSetReq->limitedDiscoverable
                            );

    switch(pRadioModeSetReq->localRadioMode)
    {
    case blueAPI_RadioVisibleConnectable:
        pageMode = blueAPI_BRScanModeInquiryAndPageScan;
        break;

    case blueAPI_RadioVisible:
        pageMode = blueAPI_BRScanModeInquiryScan;
        break;

    case blueAPI_RadioConnectable:
        pageMode = blueAPI_BRScanModePageScan;
        break;

    case blueAPI_RadioNonDiscoverable:
        pageMode = blueAPI_BRScanModeNoScan;
        break;

    case blueAPI_RadioOff:
        hciDeactReq();
        blueAPI_Send_RadioModeSetRsp(blueAPI_CauseSuccess);
        return;

    case blueAPI_RadioDeepSleep:
    default:
        blueAPI_Send_RadioModeSetRsp(blueAPI_CauseInvalidParameter);
        return;
    }

    pBlueAPIdata->ConfigParameter.limitedDiscovery = pRadioModeSetReq->limitedDiscoverable;
    btApiBT_PAGEMODE_REQ((uint8_t)pageMode);
}

/****************************************************************************/
/* void blueAPI_Handle_SDPDiscoveryReq                                      */
/****************************************************************************/
void blueAPI_Handle_SDPDiscoveryReq(PBlueAPI_Data pBlueAPIdata, PBlueAPI_DsMessage pReqMsg)
{
    PBlueAPI_SDPDiscoveryReq pSDPDiscoveryReq = &pReqMsg->p.SDPDiscoveryReq;
    TBlueAPI_Cause cause = blueAPI_CauseSuccess;

    BLUEAPI_TRACE_PRINTF_3(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:<<< SDPDiscoveryReq BD[%s] dataType[0x%X] did[%X]",
        BLUEAPI_TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE, pSDPDiscoveryReq->remote_BD),
        pSDPDiscoveryReq->remote_MDEP_DataType, pSDPDiscoveryReq->remote_DID_Discovery);

    if (pBlueAPIdata->SM_SDP_DS_Command == blueAPI_EventIdle)
    {
        pBlueAPIdata->SM_SDP_DS_Cause = blueAPI_CauseSuccess;
        memset(&pBlueAPIdata->SM_SDP_Data, 0x00, sizeof(TBlueAPI_SDPParserData));

        cause = blueAPI_SDP_Connect(pBlueAPIdata, pReqMsg, pSDPDiscoveryReq->remote_BD);
    }
    else
    {
        cause = blueAPI_CauseInvalidState;
    }

    if (cause == blueAPI_CauseSuccess)
    {
        blueAPI_SDP_DS_ContextSet(pBlueAPIdata, pReqMsg);
    }
    else
    {
        blueAPI_Send_SDPDiscoveryRsp(pBlueAPIdata, pReqMsg, cause);
    }
}

/****************************************************************************/
/* void blueAPI_Handle_SDPEndpointConf                                      */
/****************************************************************************/
void blueAPI_Handle_SDPEndpointConf(PBlueAPI_Data pBlueAPIdata, PBlueAPI_SDPEndpointConf pSDPEndpointConf)
{
    BLUEAPI_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_TRACE,
                             "blueAPI:<<< SDPEndpointConf handle[0x%X] cause[0x%x]",
                             pSDPEndpointConf->serviceHandle,
                             pSDPEndpointConf->cause
                             );

    if (pBlueAPIdata->SM_SDP_DS_Command == blueAPI_EventSDPDiscoveryReq &&
       pBlueAPIdata->SM_SDP_US_Command == blueAPI_EventSDPEndpointInd &&
       pBlueAPIdata->SM_SDP_US_Handle == pSDPEndpointConf->serviceHandle)
    {  
        pBlueAPIdata->SM_SDP_US_Command = blueAPI_EventIdle;
        pBlueAPIdata->SM_SDP_US_Handle  = 0x00000000;

        /* store first upstream error */
        if (pSDPEndpointConf->cause != blueAPI_CauseSuccess &&
            pBlueAPIdata->SM_SDP_DS_Cause == blueAPI_CauseSuccess)
        {
            pBlueAPIdata->SM_SDP_DS_Cause = pSDPEndpointConf->cause;
        }

        pBlueAPIdata->serviceHandleIndex++;
        blueAPI_SDPGetNextService(pBlueAPIdata);
    }
    else
    {
#if SEND_INT_EVENT_INFO	
        blueAPI_Send_InternalEventInfo(blueAPI_InternalEventInvalidSPPEndpointConf,
                                     BLUE_API_GENERATE_EVENT_ID,
                                     blueAPI_CauseInvalidState
                                     );
#endif
    }
}

/****************************************************************************/
/* void blueAPI_Handle_GATTServiceRegisterReq                               */
/* (                                                                        */
/*    PBlueAPI_Data          pBlueAPIdata                                   */
/*    PBlueAPI_Message       pMsg                                           */
/* )                                                                        */
/****************************************************************************/
BOOL blueAPI_Handle_GATTServiceRegisterReq(PBlueAPI_Data pBlueAPIdata, PBlueAPI_DsMessage pMsg)
{
    PBlueAPI_GATTServiceRegisterReq pGATTServiceRegisterReq = &pMsg->p.GATTServiceRegisterReq;
    PVOID pService = pGATTServiceRegisterReq->pService;
    uint16_t nbrOfAttrib = pGATTServiceRegisterReq->nbrOfAttrib;

    BLUEAPI_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:<<< GATTServiceRegisterReq nbrOfAttrib[%d] pService[0x%X] ",
        pGATTServiceRegisterReq->nbrOfAttrib, pGATTServiceRegisterReq->pService);

    gattHandleGATT_SERVICE_REGISTER_REQ(nbrOfAttrib, (uint8_t *)pService);

    return TRUE;
}

/****************************************************************************/
/* void blueAPI_Handle_GATTAttributeUpdateReq                               */
/* (                                                                        */
/*    PBlueAPI_Data        pBlueAPIdata                                     */
/*    PBlueAPI_Message     pMsg                                             */
/* )                                                                        */
/****************************************************************************/
BOOL blueAPI_Handle_GATTAttributeUpdateReq(PBlueAPI_DsMessage pMsg)
{
    BOOL msgHandled = FALSE;
    uint8_t *pBuffer = (uint8_t *)pMsg;
    PBlueAPI_GATTAttributeUpdateReq pGATTAttributeUpdateReq = &pMsg->p.GATTAttributeUpdateReq;

    BLUEAPI_TRACE_PRINTF_3(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:<<< GATTAttributeUpdateReq srvHandle[0x%X] reqHandle[0x%X] attribIndex[%d]",
        pGATTAttributeUpdateReq->serviceHandle, pGATTAttributeUpdateReq->requestHandle,
        pGATTAttributeUpdateReq->attribIndex);

    if (pGATTAttributeUpdateReq->attribLength == 0)
    {
        /* req. without payload => no connections exists */
        msgHandled = TRUE;
        pBuffer    = NULL;
    }

    if (btApiGATT_ATTRIB_UPDATE_REQ(pGATTAttributeUpdateReq->requestHandle,
            pGATTAttributeUpdateReq->serviceHandle, pGATTAttributeUpdateReq->attribIndex,
            pGATTAttributeUpdateReq->attribLength, pGATTAttributeUpdateReq->gap, pBuffer)
            != blueFaceNoError)
    {
        msgHandled = TRUE;
    }

    return msgHandled;
}

/****************************************************************************/
/* void blueAPI_Handle_GATTAttributeUpdateStatusConf                        */
/* (                                                                        */
/*    PBlueAPI_Data        pBlueAPIdata                                     */
/*    PBlueAPI_Message     pMsg                                             */
/* )                                                                        */
/****************************************************************************/
void blueAPI_Handle_GATTAttributeUpdateStatusConf(PBlueAPI_Data pBlueAPIdata,
                                                  PBlueAPI_DsMessage pMsg)
{
}

/****************************************************************************/
/* void blueAPI_Handle_GATTAttributeReadConf                                */
/* (                                                                        */
/*    PBlueAPI_Data        pBlueAPIdata                                     */
/*    PBlueAPI_Message     pMsg                                             */
/* )                                                                        */
/****************************************************************************/
BOOL blueAPI_Handle_GATTAttributeReadConf(PBlueAPI_DsMessage pMsg)
{
    PBlueAPI_LinkDescriptor pLinkContext;
#if SEND_INT_EVENT_INFO
    TBlueAPI_Cause cause = blueAPI_CauseSuccess;
#endif
    BOOL msgHandled = FALSE;
    uint8_t *pBuffer = (uint8_t *)pMsg;
    PBlueAPI_GATTAttributeReadConf pGATTAttributeReadConf = &pMsg->p.GATTAttributeReadConf;

    BLUEAPI_TRACE_PRINTF_3(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:<<< GATTAttributeReadConf local_mdl[%d] srvHandle[0x%X] attribIndex[%d]",
        pGATTAttributeReadConf->local_MDL_ID, pGATTAttributeReadConf->serviceHandle,
        pGATTAttributeReadConf->attribIndex);

    pLinkContext = blueAPI_ChannelFindByLocal_MDL_ID(pGATTAttributeReadConf->local_MDL_ID);

    if ((pLinkContext != NULL) &&
        (pLinkContext->linkState == blueAPI_Connected) && pLinkContext->MDLConnected)
    {
        if (pGATTAttributeReadConf->attribLength == 0)
        {
          /* req. without payload */
          msgHandled = TRUE;
          pBuffer    = NULL;
        }
        if (btApiGATT_ATTRIB_READ_RESP(pLinkContext->handle,
                pGATTAttributeReadConf->serviceHandle, pGATTAttributeReadConf->subCause,
                pGATTAttributeReadConf->attribLength, pGATTAttributeReadConf->gap, pBuffer)
            != blueFaceNoError)
        {
            msgHandled = TRUE;
#if SEND_INT_EVENT_INFO
            cause = blueAPI_CauseResourceError;
#endif
            assert(FALSE);
        }
    }
    else
    {
        msgHandled = TRUE;
#if SEND_INT_EVENT_INFO
        cause = blueAPI_CauseInvalidParameter;
#endif
    }

    if (msgHandled && (pBuffer != NULL))
    {
#if SEND_INT_EVENT_INFO
        /* error */
        blueAPI_Send_InternalEventInfo(blueAPI_InternalEventInvalidGATTAttributeReadConf,
            BLUE_API_GENERATE_EVENT_ID, cause);
#endif
    }

    return msgHandled;
}

/****************************************************************************/
/* void blueAPI_Handle_GATTAttributeWriteConf                               */
/* (                                                                        */
/*    PBlueAPI_Data        pBlueAPIdata                                     */
/*    PBlueAPI_Message     pMsg                                             */
/* )                                                                        */
/****************************************************************************/
void blueAPI_Handle_GATTAttributeWriteConf(PBlueAPI_DsMessage pMsg)
{
    PBlueAPI_LinkDescriptor pLinkContext;
    PBlueAPI_GATTAttributeWriteConf pGATTAttributeWriteConf = &pMsg->p.GATTAttributeWriteConf;

    BLUEAPI_TRACE_PRINTF_3(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:<<< GATTAttributeWriteConf local_mdl[%d] srvHandle[0x%X] attribIndex[%d]",
        pGATTAttributeWriteConf->local_MDL_ID, pGATTAttributeWriteConf->serviceHandle,
        pGATTAttributeWriteConf->attribIndex);

    pLinkContext = blueAPI_ChannelFindByLocal_MDL_ID(pGATTAttributeWriteConf->local_MDL_ID);
    if ((pLinkContext != NULL) &&
        (pLinkContext->linkState == blueAPI_Connected) && pLinkContext->MDLConnected)
    {
        btApiGATT_ATTRIB_WRITE_RESP(pLinkContext->handle, pGATTAttributeWriteConf->subCause, 0, 0, NULL);
    }
    else
    {
        /* error */
#if SEND_INT_EVENT_INFO
        blueAPI_Send_InternalEventInfo(blueAPI_InternalEventInvalidGATTAttributeWriteConf,
            BLUE_API_GENERATE_EVENT_ID, blueAPI_CauseInvalidParameter);
#endif
    }
}

BOOL blueAPI_Handle_GATTAttributePrepareWriteConf(PBlueAPI_DsMessage pMsg)
{
    PBlueAPI_LinkDescriptor pLinkContext;
    TBlueAPI_Cause cause = blueAPI_CauseSuccess;
    BOOL msgHandled = FALSE;
    uint8_t * pBuffer = (uint8_t *)pMsg;
    PBlueAPI_GATTAttributePrepareWriteConf pGATTAttributePrepareWriteConf =
        &pMsg->p.GATTAttributePrepareWriteConf;

    pLinkContext = blueAPI_ChannelFindByLocal_MDL_ID(pGATTAttributePrepareWriteConf->local_MDL_ID);

    if ((pLinkContext != NULL) &&
        (pLinkContext->linkState == blueAPI_Connected) && pLinkContext->MDLConnected)
    {
        if (pGATTAttributePrepareWriteConf->attribLength == 0)
        {
            /* req. without payload */
            msgHandled = TRUE;
            pBuffer    = NULL;
        }
        if (btApiGATT_ATTRIB_WRITE_RESP(pLinkContext->handle, pGATTAttributePrepareWriteConf->subCause,
            pGATTAttributePrepareWriteConf->attribLength, pGATTAttributePrepareWriteConf->gap,
            pBuffer) != blueFaceNoError)
        {
            msgHandled = TRUE;
            cause  = blueAPI_CauseResourceError;
            assert(FALSE);
        }
    }
    else
    {
        msgHandled = TRUE;
        cause = blueAPI_CauseInvalidParameter;
    }

    if (cause != blueAPI_CauseSuccess)
    {
#if SEND_INT_EVENT_INFO
        blueAPI_Send_InternalEventInfo(blueAPI_InternalEventInvalidGATTAttributePrepareWriteConf,
            BLUE_API_GENERATE_EVENT_ID, cause);
#endif
    }

    return msgHandled;
}

void blueAPI_Handle_GATTAttributeExecuteWriteConf(PBlueAPI_DsMessage pMsg)
{
    PBlueAPI_LinkDescriptor pLinkContext;
    PBlueAPI_GATTAttributeExecuteWriteConf pGATTExecuteWriteConf =
        &pMsg->p.GATTAttributeExecuteWriteConf;

    pLinkContext = blueAPI_ChannelFindByLocal_MDL_ID(pGATTExecuteWriteConf->local_MDL_ID);
    if ((pLinkContext != NULL) &&
        (pLinkContext->linkState == blueAPI_Connected) && pLinkContext->MDLConnected)
    {
        btApiGATT_EXECUTE_WRITE_RESP(pLinkContext->handle, pGATTExecuteWriteConf->subCause,
            pGATTExecuteWriteConf->handle);
    }
    else
    {
        /* error */
#if SEND_INT_EVENT_INFO
        blueAPI_Send_InternalEventInfo(blueAPI_InternalEventInvalidGATTExecuteWriteConf,
            BLUE_API_GENERATE_EVENT_ID, blueAPI_CauseInvalidParameter);
#endif
    }
}

void blueAPI_Handle_GATTAttributeExecuteWriteReq(PBlueAPI_DsMessage pMsg)
{
    PBlueAPI_LinkDescriptor pLinkContext;
    uint16_t cause = GATT_SUCCESS;
    PBlueAPI_GATTAttributeExecuteWriteReq pGATTAttributeExecuteWriteReq =
        &pMsg->p.GATTAttributeExecuteWriteReq;

    pLinkContext = blueAPI_ChannelFindByLocal_MDL_ID(pGATTAttributeExecuteWriteReq->local_MDL_ID);

    if ((pLinkContext != NULL) &&
        (pLinkContext->linkState == blueAPI_Connected) && pLinkContext->MDLConnected)
    {
        if (btApiGATT_EXECUTE_WRITE_REQ(pLinkContext->handle,
            pGATTAttributeExecuteWriteReq->flags) != blueFaceNoError)
        {
            cause = GATT_ERR_OUT_OF_RESOURCE;
        }
    }
    else
    {
        if (pLinkContext != NULL)
        {
            cause = GATT_ERR_ILLEGAL_STATE;
        }
        else
        {
            cause = GATT_ERR_ILLEGAL_PARAMETER;
        }
    }

    if (cause != GATT_SUCCESS)
    {
        /* error */
        blueAPI_Send_GATTExecuteWriteRsp(NULL, pGATTAttributeExecuteWriteReq->local_MDL_ID, cause);
    }
}

/****************************************************************************/
/* void blueAPI_Handle_GATTDiscoveryReq                                     */
/* (                                                                        */
/*    PBlueAPI_Data        pBlueAPIdata                                     */
/*    PBlueAPI_Message     pMsg                                             */
/* )                                                                        */
/****************************************************************************/
BOOL blueAPI_Handle_GATTDiscoveryReq(PBlueAPI_DsMessage pMsg)
{
    PBlueAPI_LinkDescriptor pLinkContext;
    uint16_t type = GATT_TYPE_UNDEFINED;
    uint16_t cause = GATT_SUCCESS;
    PBlueAPI_GATTDiscoveryReq pGATTDiscoveryReq = &pMsg->p.GATTDiscoveryReq;

    BLUEAPI_TRACE_PRINTF_4(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:<<< GATTDiscoveryReq local_mdl[%d] type[%d] startHandle[0x%X] endHandle[0x%X]",
        pGATTDiscoveryReq->local_MDL_ID, pGATTDiscoveryReq->discoveryType,
        pGATTDiscoveryReq->startHandle, pGATTDiscoveryReq->endHandle);

    pLinkContext = blueAPI_ChannelFindByLocal_MDL_ID(pGATTDiscoveryReq->local_MDL_ID);

    if ((pLinkContext != NULL) &&
      (pLinkContext->linkState == blueAPI_Connected) && pLinkContext->MDLConnected)
    {
        /* convert discoveryType from BlueAPI to Blueface value */
        type = pGATTDiscoveryReq->discoveryType;
        if (type == GATT_TYPE_UNDEFINED)
        {
            cause = GATT_ERR_ILLEGAL_PARAMETER;
        }
    }
    else
    {
        if (pLinkContext != NULL)
        {
            cause = GATT_ERR_ILLEGAL_STATE;
        }
        else
        {
            cause = GATT_ERR_ILLEGAL_PARAMETER;
        }
    }

    if (cause != GATT_SUCCESS)
    {
        blueAPI_Send_GATTDiscoveryRsp(pMsg, pGATTDiscoveryReq->local_MDL_ID,
            pGATTDiscoveryReq->discoveryType, cause);

        return FALSE;
    }
    else
    {
        btApiGATT_DISCOVERY_REQ(pLinkContext->handle, type, pGATTDiscoveryReq->startHandle,
            pGATTDiscoveryReq->endHandle, pGATTDiscoveryReq->UUID16, pGATTDiscoveryReq->UUID128);
    }
    return TRUE;
}

/****************************************************************************/
/* void blueAPI_Handle_GATTDiscoveryConf                                    */
/* (                                                                        */
/*    PBlueAPI_Data        pBlueAPIdata                                     */
/*    PBlueAPI_Message     pMsg                                             */
/* )                                                                        */
/****************************************************************************/
void blueAPI_Handle_GATTDiscoveryConf(PBlueAPI_DsMessage pMsg)
{
    PBlueAPI_LinkDescriptor pLinkContext;
    PBlueAPI_GATTDiscoveryConf pGATTDiscoveryConf = &pMsg->p.GATTDiscoveryConf;

    BLUEAPI_TRACE_PRINTF_4(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:<<< GATTDiscoveryConf local_mdl[%d] type[%d] startHandle[0x%X] endHandle[0x%X]",
        pGATTDiscoveryConf->local_MDL_ID, pGATTDiscoveryConf->discoveryType,
        pGATTDiscoveryConf->startHandle, pGATTDiscoveryConf->endHandle);

    pLinkContext = blueAPI_ChannelFindByLocal_MDL_ID(pGATTDiscoveryConf->local_MDL_ID);

    if ((pLinkContext != NULL) &&
        (pLinkContext->linkState == blueAPI_Connected) && pLinkContext->MDLConnected)
    {
        btApiGATT_DISCOVERY_RESP(pLinkContext->handle,
            pGATTDiscoveryConf->discoveryType, pGATTDiscoveryConf->startHandle,
            pGATTDiscoveryConf->endHandle);
    }
    else
    {
        /* error */
#if SEND_INT_EVENT_INFO
        blueAPI_Send_InternalEventInfo(blueAPI_InternalEventInvalidGATTDiscoveryConf,
            BLUE_API_GENERATE_EVENT_ID, blueAPI_CauseInvalidParameter);
#endif
    }
}

/****************************************************************************/
/* void blueAPI_Handle_GATTAttributeReadReq                                 */
/* (                                                                        */
/*    PBlueAPI_Data        pBlueAPIdata                                     */
/*    PBlueAPI_Message     pMsg                                             */
/* )                                                                        */
/****************************************************************************/
BOOL blueAPI_Handle_GATTAttributeReadReq(PBlueAPI_DsMessage pMsg)
{
    PBlueAPI_LinkDescriptor pLinkContext;
    uint16_t type = GATT_TYPE_UNDEFINED;
    uint16_t cause = GATT_SUCCESS;
    PBlueAPI_GATTAttributeReadReq pGATTAttributeReadReq = &pMsg->p.GATTAttributeReadReq;

    BLUEAPI_TRACE_PRINTF_4(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:<<< GATTAttributeRead local_mdl[%d] type[%d] startHandle[0x%X] endHandle[0x%X]",
        pGATTAttributeReadReq->local_MDL_ID, pGATTAttributeReadReq->readType,
        pGATTAttributeReadReq->startHandle, pGATTAttributeReadReq->endHandle);

    pLinkContext = blueAPI_ChannelFindByLocal_MDL_ID(pGATTAttributeReadReq->local_MDL_ID);

    if ((pLinkContext != NULL) &&
        (pLinkContext->linkState == blueAPI_Connected) && pLinkContext->MDLConnected)
    {
        /* convert readType from BlueAPI to Blueface value */
        switch (pGATTAttributeReadReq->readType)
        {
        default:
            cause = GATT_ERR_ILLEGAL_PARAMETER;
            break;

        case blueAPI_GATTReadTypeBasic:
            if (pGATTAttributeReadReq->readOffset == 0)
            {
              type = GATT_READ_TYPE_BASIC;
            }
            else
            {
              type = GATT_READ_TYPE_BLOB;
            }
            break;

        case blueAPI_GATTReadTypeByUUID:
            type = GATT_READ_TYPE_TYPE;
            break;
        }
    }
    else
    {
        if (pLinkContext != NULL)
        {
            cause = GATT_ERR_ILLEGAL_STATE;
        }
        else
        {
            cause = GATT_ERR_ILLEGAL_PARAMETER;
        }
    }

    if (cause != GATT_SUCCESS)
    {
        blueAPI_Send_GATTAttributeReadRsp(pMsg, NULL, 0, pGATTAttributeReadReq->readType,
            0, 0, 0, 0, cause, pGATTAttributeReadReq->local_MDL_ID);
        return FALSE;
    }
    else
    {
        if (type == GATT_READ_TYPE_TYPE)
        {
            btApiGATT_ATTRIB_READ_REQ_UUID(pLinkContext->handle,
                pGATTAttributeReadReq->startHandle, pGATTAttributeReadReq->endHandle,
                pGATTAttributeReadReq->UUID16, pGATTAttributeReadReq->UUID128);
        }
        else
        {
            btApiGATT_ATTRIB_READ_REQ(pLinkContext->handle, pGATTAttributeReadReq->readOffset,
                1, &pGATTAttributeReadReq->startHandle);
        }
    }
    return TRUE;
}

/****************************************************************************/
/* void blueAPI_Handle_GATTAttributeWriteReq                                */
/* (                                                                        */
/*    PBlueAPI_Data        pBlueAPIdata                                     */
/*    PBlueAPI_Message     pMsg                                             */
/* )                                                                        */
/****************************************************************************/
void blueAPI_Handle_GATTAttributeWriteReq(PBlueAPI_DsMessage pMsg)
{
    PBlueAPI_LinkDescriptor pLinkContext;
    uint16_t cause = GATT_SUCCESS;
    uint8_t * pBuffer = (uint8_t *)pMsg;
    PBlueAPI_GATTAttributeWriteReq pGATTAttributeWriteReq = &pMsg->p.GATTAttributeWriteReq;
    uint16_t type;

    BLUEAPI_TRACE_PRINTF_3(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:<<< GATTAttributeWriteReq local_mdl[%d] attribHandle[0x%X] attribLength[%d]",
        pGATTAttributeWriteReq->local_MDL_ID, pGATTAttributeWriteReq->attribHandle,
        pGATTAttributeWriteReq->attribLength);

    pLinkContext = blueAPI_ChannelFindByLocal_MDL_ID(pGATTAttributeWriteReq->local_MDL_ID);

    switch (pGATTAttributeWriteReq->writeType)
    {
        case blueAPI_GATTWriteTypeCommand:
            type = GATT_WRITE_TYPE_CMD;
            break;
        case 3:
            type = GATT_WRITE_TYPE_PREP;
            break;
        default:
            type = GATT_WRITE_TYPE_REQ;
            break;
    }

    if ((pLinkContext != NULL) && (pLinkContext->linkState == blueAPI_Connected)
        && pLinkContext->MDLConnected)
    { 
        if (btApiGATT_ATTRIB_WRITE_REQ(pLinkContext->handle, type,
            pGATTAttributeWriteReq->attribHandle, pGATTAttributeWriteReq->attribLength,
            pGATTAttributeWriteReq->writeOffset, pGATTAttributeWriteReq->gap, pBuffer)
            != blueFaceNoError)
        {
            cause = GATT_ERR_OUT_OF_RESOURCE;
        }
    }
    else
    {
        if (pLinkContext != NULL)
        {
            cause = GATT_ERR_ILLEGAL_STATE;
        }
        else
        {
            cause = GATT_ERR_ILLEGAL_PARAMETER;
        }
    }

    if ( cause != GATT_SUCCESS )
    {
        /* error */
        blueAPI_Send_GATTAttributeWriteRsp(pMsg, pGATTAttributeWriteReq->local_MDL_ID,
                                           (TBlueAPI_GATTWriteType)pGATTAttributeWriteReq->writeType,
                                           cause, 0);
    }

}

/****************************************************************************/
/* void blueAPI_Handle_GATTAttributeConf                                    */
/* (                                                                        */
/*    PBlueAPI_Data        pBlueAPIdata                                     */
/*    PBlueAPI_Message     pMsg                                             */
/* )                                                                        */
/****************************************************************************/
void blueAPI_Handle_GATTAttributeConf(PBlueAPI_DsMessage pMsg)
{
    PBlueAPI_LinkDescriptor pLinkContext;
    PBlueAPI_GATTAttributeConf pGATTAttributeConf = &pMsg->p.GATTAttributeConf;

    BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE, "blueAPI:<<< GATTAttributeConf");

    pLinkContext = blueAPI_ChannelFindByLocal_MDL_ID(pGATTAttributeConf->local_MDL_ID);

    if ((pLinkContext != NULL) && (pLinkContext->linkState == blueAPI_Connected)
        && pLinkContext->MDLConnected)
    {
        LPbtLink pLink = (LPbtLink)(pLinkContext->handle);
        gattHandleGATT_ATTRIB_INDICATION_RESP(pLink->handle);
    }
    else
    {
        /* error */
#if SEND_INT_EVENT_INFO
        blueAPI_Send_InternalEventInfo(blueAPI_InternalEventInvalidGATTAttributeConf,
            BLUE_API_GENERATE_EVENT_ID, blueAPI_CauseInvalidParameter);
#endif
    }
}

/****************************************************************************/
/* blueAPI_Handle_GATTSDPDiscoveryReq                                       */
/****************************************************************************/
void blueAPI_Handle_GATTSDPDiscoveryReq(PBlueAPI_DsMessage pReqMsg)
{
    PBlueAPI_GATTSDPDiscoveryReq pDiscoveryReq = &pReqMsg->p.GATTSDPDiscoveryReq;
    TBlueAPI_Cause cause = blueAPI_CauseSuccess;

    BLUEAPI_TRACE_PRINTF_3(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:<<< GATTSDPDiscoveryReq BD[%s] UUID[0x%X] did[%X]",
        BLUEAPI_TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE, pDiscoveryReq->remote_BD),
        pDiscoveryReq->remote_GATT_UUID, pDiscoveryReq->remote_DID_Discovery);

    if (pBlueAPIData->SM_SDP_DS_Command == blueAPI_EventIdle)
    {
        /* start GATT discovery */
        pBlueAPIData->SM_SDP_DS_Cause = blueAPI_CauseSuccess;
        memset(&pBlueAPIData->SM_SDP_Data, 0x00, sizeof(TBlueAPI_SDPParserData));
        cause = blueAPI_SDP_Connect(pBlueAPIData, pReqMsg, pDiscoveryReq->remote_BD);
    }
    else
    {
        cause = blueAPI_CauseInvalidState;
    }

    if (cause == blueAPI_CauseSuccess)
    {
        blueAPI_SDP_DS_ContextSet(pBlueAPIData, pReqMsg);
    }
    else
    {
        blueAPI_Send_GATTSDPDiscoveryRsp(pReqMsg, cause);
    }
} /* blueAPI_Handle_GATTSDPDiscoveryReq */

/****************************************************************************/
/* blueAPI_Handle_GATTSDPDiscoveryConf                                      */
/****************************************************************************/
void blueAPI_Handle_GATTSDPDiscoveryConf(PBlueAPI_DsMessage pMsg)
{
    PBlueAPI_GATTSDPDiscoveryConf pServiceConf = &pMsg->p.GATTSDPDiscoveryConf;

    BLUEAPI_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:<<< GATTSDPDiscoveryConf handle[0x%X] cause[0x%x]",
        pServiceConf->serviceHandle, pServiceConf->cause);

    if (pBlueAPIData->SM_SDP_DS_Command == blueAPI_EventGATTSDPDiscoveryReq &&
        pBlueAPIData->SM_SDP_US_Command == blueAPI_EventGATTSDPDiscoveryInd &&
        pBlueAPIData->SM_SDP_US_Handle == pServiceConf->serviceHandle)
    {
        pBlueAPIData->SM_SDP_US_Command = blueAPI_EventIdle;
        pBlueAPIData->SM_SDP_US_Handle  = 0x00000000;

        /* store first upstream error */
        if (pServiceConf->cause != blueAPI_CauseSuccess &&
            pBlueAPIData->SM_SDP_DS_Cause == blueAPI_CauseSuccess)
        {
            pBlueAPIData->SM_SDP_DS_Cause = pServiceConf->cause;
        }

        pBlueAPIData->serviceHandleIndex++;
        blueAPI_SDPGetNextService(pBlueAPIData);
    }
    else
    {
#if SEND_INT_EVENT_INFO
        blueAPI_Send_InternalEventInfo(blueAPI_InternalEventInvalidGATTSDPDiscoveryConf,
            BLUE_API_GENERATE_EVENT_ID, blueAPI_CauseInvalidState);
#endif
    }
} /* blueAPI_Handle_GATTSDPDiscoveryConf */


/****************************************************************************/
/* void blueAPI_Handle_GATTSecurityReq                                      */
/* (                                                                        */
/*    PBlueAPI_Data        pBlueAPIdata                                     */
/*    PBlueAPI_Message     pMsg                                             */
/* )                                                                        */
/****************************************************************************/
void blueAPI_Handle_GATTSecurityReq(
                                           PBlueAPI_DsMessage pMsg)
{
    PBlueAPI_MCL             pMCL;
    PBlueAPI_GATTSecurityReq pSecReq      = &pMsg->p.GATTSecurityReq;

    BLUEAPI_TRACE_PRINTF_3(BLUEAPI_TRACE_MASK_TRACE,
                            "blueAPI:<<< GATTSecurityReq MDL[%d] requirements[0x%x] minKeySize[%d]",
                            pSecReq->local_MDL_ID,
                            pSecReq->requirements,
                            pSecReq->minKeySize
                            );

    pMCL = blueAPI_MCLFindByLocal_MDL_ID(pSecReq->local_MDL_ID);
    if ((pMCL != NULL) && (pMCL->mclType == blueAPI_MCLType_GATT))
    {
        btsmSendMsgAuthenticationReq(
                                    pMCL->bd,
                                    pSecReq->requirements,
                                    pSecReq->minKeySize
                                    );
    }
    else
    {
        blueAPI_Send_GATTSecurityRsp(
                                    pSecReq->local_MDL_ID,
                                    (TBlueAPI_KeyType)0x00,
                                    0x00,
                                    blueAPI_CauseInvalidParameter
                                    );
    }
}

/****************************************************************************/
/* void blueAPI_Handle_GATTServerStoreConf                                   */
/****************************************************************************/
void blueAPI_Handle_GATTServerStoreConf(PBlueAPI_DsMessage pMsg)
{
    PBlueAPI_GATTServerStoreConf pStoreConf = &pMsg->p.GATTServerStoreConf;
    PDEVICE_DATA_IND pDeviceDataInd = &pBlueAPIData->pDSCurrentMsg->p.deviceDataIndication;
    PDEVICE_DATA pDeviceData = &pDeviceDataInd->deviceData;
    BOOL completed = TRUE;
    uint16_t status  = BTSEC_SUCCESS;

    BLUEAPI_TRACE_PRINTF_6(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:<<< GATTServerStoreConf op[%d] bd[%s] bdType[%d] restart[0x%x] datalen[%d] cause[0x%x]",
        pStoreConf->opCode, BLUEAPI_TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE, pStoreConf->remote_BD),
        pStoreConf->remote_BD_Type, pStoreConf->restartHandle, pStoreConf->dataLength, 
        pStoreConf->cause);

    switch (pStoreConf->opCode)
    {
    case blueAPI_GATTStoreOpGetCCCBits:
    case blueAPI_GATTStoreOpSetCCCBits:
    case blueAPI_GATTStoreOpGetAllCCCBits:
    case blueAPI_GATTStoreOpDeleteAllCCCBits:
        if ((pBlueAPIData->pDSCurrentMsg == NULL) ||
            (pDeviceData->dataType != DEVICE_DATA_TYPE_GATT_CCC_BITS))
        {
#if SEND_INT_EVENT_INFO
            blueAPI_Send_InternalEventInfo(blueAPI_InternalEventInvalidSecurityConf,
                BLUE_API_GENERATE_EVENT_ID, blueAPI_CauseInvalidParameter);
#endif
            return;
        }
        break;

    case blueAPI_GATTStoreOpGetSrvChg:
    case blueAPI_GATTStoreOpSetSrvChg:
    case blueAPI_GATTStoreOpSetSrvChgFlag:
        if ((pBlueAPIData->pDSCurrentMsg == NULL) ||
            (pDeviceData->dataType != DEVICE_DATA_TYPE_GATT_SRV_CHG_HANDLE))
        {
#if SEND_INT_EVENT_INFO
            blueAPI_Send_InternalEventInfo(blueAPI_InternalEventInvalidSecurityConf,
                BLUE_API_GENERATE_EVENT_ID, blueAPI_CauseInvalidParameter);
#endif
            return;
        }
        break;

    default:
#if SEND_INT_EVENT_INFO
        blueAPI_Send_InternalEventInfo(blueAPI_InternalEventInvalidSecurityConf,
            BLUE_API_GENERATE_EVENT_ID, blueAPI_CauseInvalidParameter);
#endif
        return;
    }

    pDeviceData->restartHandle = pStoreConf->restartHandle;

    switch (pStoreConf->opCode)
    {
    case blueAPI_GATTStoreOpGetCCCBits:
    case blueAPI_GATTStoreOpGetSrvChg:
        if (pStoreConf->cause != blueAPI_CauseSuccess)
        {
            status = BTSEC_ERR_UNSPECIFIED;
        }

        if (pStoreConf->opCode == blueAPI_GATTStoreOpGetCCCBits)
        {
            pDeviceData->elementCount = pStoreConf->dataLength/sizeof(TDEVICE_DATA_ELEMENT_GATT_CCC_BITS);
        }
        else
        {
            pDeviceData->elementCount = 1;
        }

        memcpy(pDeviceData->p.data, pStoreConf->data, pStoreConf->dataLength);

        btApiDEVICE_DATA_GET_RESP(pDeviceDataInd->handle, status, pDeviceData);
        break;

    case blueAPI_GATTStoreOpSetCCCBits:
    case blueAPI_GATTStoreOpSetSrvChg:
    case blueAPI_GATTStoreOpDeleteAllCCCBits:
    case blueAPI_GATTStoreOpSetSrvChgFlag:
        if (pStoreConf->cause != blueAPI_CauseSuccess)
        {
            status = BTSEC_ERR_UNSPECIFIED;
        }

        pDeviceData->elementCount = 0;

        btApiDEVICE_DATA_SET_RESP(pDeviceDataInd->handle, status, pDeviceData);
        break;

    case blueAPI_GATTStoreOpGetAllCCCBits:
        if (pStoreConf->cause != blueAPI_CauseSuccess)
        {
            status = BTSEC_ERR_UNSPECIFIED;

            btApiDEVICE_DATA_GET_RESP(pDeviceDataInd->handle, status, pDeviceData);
        }
        else
        {
            PDEVICE_DATA_ELEMENT_GATT_CCC_BITS pCCCsearch;
            uint16_t idx;

            /* check if requested CCC bit is set for this BD */
            for (idx = 0; idx < pStoreConf->dataLength; idx += sizeof(TDEVICE_DATA_ELEMENT_GATT_CCC_BITS))
            {
                pCCCsearch = (PDEVICE_DATA_ELEMENT_GATT_CCC_BITS)&pStoreConf->data[idx];

                if ((pCCCsearch->attHandle == pBlueAPIData->dsCCCSearchHandle) &&
                    (pCCCsearch->cccBits != 0x0000))
                {
                    if (pDeviceData->elementCount < DEVICE_DATA_ELEMENT_GATT_CCC_BITS_REV_MAX)
                    {
                        PDEVICE_DATA_ELEMENT_GATT_CCC_BITS_REV pCCCRev;

                        pCCCRev = &pDeviceData->p.cccBitRev[pDeviceData->elementCount++];
                        pCCCRev->bdType = pStoreConf->remote_BD_Type;
                        memcpy(pCCCRev->bd, pStoreConf->remote_BD, BD_ADDR_SIZE);
                    }
                }
            }

            /* no matching CCC bits yet, return to application */
            if ((pDeviceData->elementCount == 0) && (pStoreConf->restartHandle != 0x0000))
            {
                blueAPI_Send_GATTServerStoreInd(blueAPI_GATTStoreOpGetAllCCCBits,
                    pDeviceDataInd->deviceData.bd,
                    (TBlueAPI_RemoteBDType)pDeviceDataInd->deviceData.bdType,
                    pStoreConf->restartHandle, 0, NULL);

                completed = FALSE;
            }
            else
            {
                /* adjust response type */
                pDeviceData->dataType = DEVICE_DATA_TYPE_GATT_CCC_REV_BITS;

                btApiDEVICE_DATA_GET_RESP(pDeviceDataInd->handle, status, pDeviceData);
            }
        }
        break;

    default:
        break;
    }

    if (completed)
    {
        blueAPIStore_QueueTrigger(pBlueAPIData, TRUE);
    }
} /* blueAPI_Handle_GATTServerStoreConf */

/****************************************************************************/
/* void blueAPI_Handle_PairableModeSetReq                                   */
/****************************************************************************/
void blueAPI_Handle_PairableModeSetReq(PBlueAPI_DsMessage pReqMsg)
{
    PBlueAPI_PairableModeSetReq pPairableModeSetReq = &pReqMsg->p.PairableModeSetReq;

    BLUEAPI_TRACE_PRINTF_5(BLUEAPI_TRACE_MASK_TRACE,
                             "blueAPI:<<< PairableModeSetReq pairable[%d] btMode[%d] AuthReq[%d] ioCap[%d] OOB[%d]",
                             pPairableModeSetReq->enablePairableMode,
                             pPairableModeSetReq->BluetoothMode,
                             pPairableModeSetReq->AuthRequirements,
                             pPairableModeSetReq->IOCapabilities,
                             pPairableModeSetReq->remoteOOBDataPresent
                             );

    if ((pPairableModeSetReq->BluetoothMode <= 2) &&
        (pPairableModeSetReq->AuthRequirements <= 5) &&
        (pPairableModeSetReq->IOCapabilities <= 4)
        )
    {
        TSEC_CONF_DEVICE_SECURITY deviceSecurity;

        deviceSecurity.pairable_mode   = (uint8_t)pPairableModeSetReq->enablePairableMode;
        deviceSecurity.authenSettings  = (uint8_t)pPairableModeSetReq->AuthRequirements;
        deviceSecurity.io_capabilities = (uint8_t)pPairableModeSetReq->IOCapabilities;
        deviceSecurity.oob_present	   = (uint8_t)pPairableModeSetReq->remoteOOBDataPresent;
        deviceSecurity.bt_mode         = pPairableModeSetReq->BluetoothMode;

        btsmHandleConfigurationReq(BLUEFACE_CONF_DEVICE_SECURITY,
                                   (uint8_t *)&deviceSecurity,
                                   pPairableModeSetReq->isSetBluetoothMode
                                   );

        blueAPI_Send_PairableModeSetRsp(pReqMsg, blueAPI_CauseSuccess);
    }
    else
    {
        blueAPI_Send_PairableModeSetRsp(pReqMsg, blueAPI_CauseInvalidParameter);
    }
}

/****************************************************************************/
/* void blueAPI_Handle_AuthReq                                              */
/****************************************************************************/
void blueAPI_Handle_AuthReq(PBlueAPI_DsMessage pReqMsg)
{
    PBlueAPI_AuthReq pAuthReq = &pReqMsg->p.AuthReq;
    TBlueAPI_Cause cause= blueAPI_CauseSuccess;

    BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE, "blueAPI:<<< AuthReq BD[%s]",
        BLUEAPI_TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE, pAuthReq->remote_BD));

    if ((pBlueAPIData->SM_SEC_DS_CommandMsg == NULL) &&
        (pBlueAPIData->SM_SDP_DS_CommandMsg == NULL))
    {
        cause = blueAPI_SDP_Connect(pBlueAPIData, pReqMsg, pAuthReq->remote_BD);
        if (cause == blueAPI_CauseSuccess)
        {
            blueAPI_SDP_DS_ContextSet(pBlueAPIData, pReqMsg);
        }
    }
    else
    {
        cause = blueAPI_CauseInvalidState;
    }

    if (cause == blueAPI_CauseSuccess)
    {
        pBlueAPIData->SM_SEC_DS_CommandMsg = pReqMsg;
    }
    else
    {
        blueAPI_Send_AuthRsp(pReqMsg, pAuthReq->remote_BD, cause);
    }
}


/****************************************************************************/
/* void blueAPI_Handle_UserAuthRequestConf                                  */
/****************************************************************************/
void blueAPI_Handle_UserAuthRequestConf(PBlueAPI_UserAuthRequestConf pUserAuthRequestConf)
{
    BLUEAPI_TRACE_PRINTF_3(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:<<< UserAuthRequestConf BD[%s] authsize[%d] cause[0x%x]",
        BLUEAPI_TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE,pUserAuthRequestConf->remote_BD),
        pUserAuthRequestConf->AuthCodeLength, pUserAuthRequestConf->cause);

    if (pUserAuthRequestConf->cause == blueAPI_CauseNotSupported)
    {
        BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE, "!!blueAPI: terminated");
        return;
    }
    {
        uint16_t cause = HCI_SUCCESS;

        if ((pUserAuthRequestConf->cause != blueAPI_CauseSuccess) &&
            (pUserAuthRequestConf->cause != blueAPI_CauseAccept))
        {
            cause = HCI_ERR | HCI_ERR_KEY_MISSING;
        }

        btApiBT_HCI_KEY_RESP(BLUEFACE_LOCALPIN, cause, pUserAuthRequestConf->AuthCodeLength,
            pUserAuthRequestConf->AuthCode, pUserAuthRequestConf->remote_BD);
    }
}

/****************************************************************************/
/* void blueAPI_Handle_AuthResultRequestConf                                */
/****************************************************************************/
void blueAPI_Handle_AuthResultRequestConf(PBlueAPI_AuthResultRequestConf pAuthResultRequestConf)
{
    BLUEAPI_TRACE_PRINTF_6(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:<<< AuthResultRequestConf BD[%s] BDType[%d] keyType[%d] keyLen[%d] restart[0x%x] cause[0x%x]",
        BLUEAPI_TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE, pAuthResultRequestConf->remote_BD),
        pAuthResultRequestConf->remote_BD_Type, pAuthResultRequestConf->keyType,
        pAuthResultRequestConf->linkKeyLength, pAuthResultRequestConf->restartHandle,
        pAuthResultRequestConf->cause);

    if (pAuthResultRequestConf->cause == blueAPI_CauseNotSupported)
    {
        BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE, "!!blueAPI: terminated");
        return;
    }

    switch (pAuthResultRequestConf->keyType)
    {
    case blueAPI_LinkKeyTypeCombination:
    case blueAPI_LinkKeyTypeUnauthenticated:
    case blueAPI_LinkKeyTypeAuthenticated:
    {
        uint16_t cause = HCI_SUCCESS;
        uint16_t keyType = BLUEFACE_LINKKEY;

        if (pAuthResultRequestConf->remote_BD_Type == blueAPI_RemoteBDTypeClassic)
        {
            if ((pAuthResultRequestConf->cause != blueAPI_CauseSuccess) &&
                (pAuthResultRequestConf->cause != blueAPI_CauseAccept))
            {
                cause = HCI_ERR | HCI_ERR_KEY_MISSING;
            }
            else if (pAuthResultRequestConf->keyType == blueAPI_LinkKeyTypeAuthenticated)
            {
                keyType = BLUEFACE_LINKKEY_MITM;
            }

            btApiBT_HCI_KEY_RESP(keyType, cause, LINK_KEY_SIZE,
                pAuthResultRequestConf->linkKey, pAuthResultRequestConf->remote_BD);
        }
        else
        {
#if SEND_INT_EVENT_INFO
            blueAPI_Send_InternalEventInfo(blueAPI_InternalEventInvalidSecurityConf,
                BLUE_API_GENERATE_EVENT_ID, blueAPI_CauseUnspecified);
#endif
        }
    }
        break;

    case blueAPI_LinkKeyTypeLELocalLTK:
    case blueAPI_LinkKeyTypeLERemoteLTK:
    case blueAPI_LinkKeyTypeLELocalIRK:
    case blueAPI_LinkKeyTypeLERemoteIRK:
        if (pBlueAPIData->pDSCurrentMsg != NULL)
        {
            PDEVICE_DATA_IND pDeviceDataInd = &pBlueAPIData->pDSCurrentMsg->p.deviceDataIndication;
            uint16_t status = BTSEC_SUCCESS;

            memcpy(pDeviceDataInd->deviceData.bd, pAuthResultRequestConf->remote_BD, BLUE_API_BD_SIZE);
            pDeviceDataInd->deviceData.bdType = pAuthResultRequestConf->remote_BD_Type;
            pDeviceDataInd->deviceData.dataType = pAuthResultRequestConf->keyType;
            pDeviceDataInd->deviceData.restartHandle = pAuthResultRequestConf->restartHandle;
            pDeviceDataInd->deviceData.elementCount = (pAuthResultRequestConf->linkKeyLength > 0) ? 1 : 0;

            memcpy(&pDeviceDataInd->deviceData.p.data, pAuthResultRequestConf->linkKey,
                pAuthResultRequestConf->linkKeyLength);

            if ((pAuthResultRequestConf->cause != blueAPI_CauseSuccess) &&
                (pAuthResultRequestConf->cause != blueAPI_CauseAccept))
            {
                status = BTSEC_ERR_UNSPECIFIED;
            }

            btApiDEVICE_DATA_GET_RESP(pDeviceDataInd->handle, status, &pDeviceDataInd->deviceData);

            blueAPIStore_QueueTrigger(pBlueAPIData, TRUE);
        }
        else
        {
#if SEND_INT_EVENT_INFO
            blueAPI_Send_InternalEventInfo(blueAPI_InternalEventInvalidSecurityConf,
                BLUE_API_GENERATE_EVENT_ID, blueAPI_CauseUnspecified);
#endif
        }
        break;

    default:
#if SEND_INT_EVENT_INFO
        blueAPI_Send_InternalEventInfo(blueAPI_InternalEventInvalidSecurityConf,
            BLUE_API_GENERATE_EVENT_ID, blueAPI_CauseUnspecified);
#endif
        break;
    }
}

/****************************************************************************/
/* void blueAPI_Handle_UserConfirmationReqConf                              */
/****************************************************************************/
void blueAPI_Handle_UserConfirmationReqConf( PBlueAPI_UserConfirmationReqConf pUserConfirmationReqConf)
{
   BLUEAPI_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_TRACE,
                             "blueAPI:<<< UserConfirmationReqConf BD[%s] cause[0x%x]",
                             BLUEAPI_TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE, pUserConfirmationReqConf->remote_BD),
                             pUserConfirmationReqConf->cause
                             );

   if (pUserConfirmationReqConf->cause == blueAPI_CauseNotSupported)
   {
      BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE,
                                "!!blueAPI: terminated"
                                );
      return;
   }

   btsmSendLHciUserConfReqResp(pUserConfirmationReqConf->remote_BD, (pUserConfirmationReqConf->cause == blueAPI_CauseAccept) ? 0 : 1);
}

void blueAPI_Handle_UserAuthorizationReqConf(PBlueAPI_UserAuthorizationReqConf pUserAuthorizationReqConf)
{
    TSEC_CONF_AUTHORIZE auth;

    BLUEAPI_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_TRACE,
                         "blueAPI:<<< UserAuthorizationReqConf BD[%s] cause[%x]",
                         BLUEAPI_TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE, pUserAuthorizationReqConf->remote_BD),
                         pUserAuthorizationReqConf->cause
                         );

    if (pUserAuthorizationReqConf->cause == blueAPI_CauseNotSupported)
    {
        BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE, "!!blueAPI: terminated");
        return;
    }

    memcpy(auth.bd, pUserAuthorizationReqConf->remote_BD, BD_ADDR_SIZE);
    auth.active = (pUserAuthorizationReqConf->cause == blueAPI_CauseAccept) ? 0 : 1;

    btsmHandleConfigurationReq(BLUEFACE_CONF_AUTHORIZE, (uint8_t *)&auth, FALSE);
}

/****************************************************************************/
/* void blueAPI_Handle_UserPasskeyReqConf                                   */
/****************************************************************************/
void blueAPI_Handle_UserPasskeyReqConf(PBlueAPI_UserPasskeyReqConf pUserPasskeyReqConf)
{
   BLUEAPI_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_TRACE,
                             "blueAPI:<<< UserPasskeyReqConf BD[%s] cause[0x%x]",
                             BLUEAPI_TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE, pUserPasskeyReqConf->remote_BD),
                             pUserPasskeyReqConf->cause
                             );

   if (pUserPasskeyReqConf->cause == blueAPI_CauseNotSupported)
   {
      BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE,
                                "!!blueAPI: terminated"
                                );
      return;
   }
   btsmHandleUserPasskeyReqConf(pUserPasskeyReqConf->remote_BD, 
                                (pUserPasskeyReqConf->cause == blueAPI_CauseAccept) ? 0 : 1,
                                0);
}

/****************************************************************************/
/* blueAPI_Handle_UserPasskeyReqReplyReq                                    */
/****************************************************************************/
void blueAPI_Handle_UserPasskeyReqReplyReq( PBlueAPI_UserPasskeyReqReplyReq pUserPasskeyReqReplyReq)
{
   BLUEAPI_TRACE_PRINTF_3(BLUEAPI_TRACE_MASK_TRACE,
                             "blueAPI:<<< UserPasskeyReqReplyReq BD[%s] passkey[%d] cause[0x%x]",
                             BLUEAPI_TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE, pUserPasskeyReqReplyReq->remote_BD),
                             pUserPasskeyReqReplyReq->passKey,
                             pUserPasskeyReqReplyReq->cause
                             );

   btsmHandleUserPasskeyReqReplyReq(pUserPasskeyReqReplyReq->remote_BD,
                                              (pUserPasskeyReqReplyReq->cause == blueAPI_CauseAccept) ? 0 : 1,
                                              pUserPasskeyReqReplyReq->passKey);
}

/****************************************************************************/
/* blueAPI_Handle_KeypressNotificationReq                                   */
/****************************************************************************/
void blueAPI_Handle_KeypressNotificationReq(PBlueAPI_KeypressNotificationReq pKeypressNotificationReq)
{
   BLUEAPI_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_TRACE,
                             "blueAPI:<<< KeypressNotificationReq BD[%s] eventType[%d]",
                             BLUEAPI_TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE, pKeypressNotificationReq->remote_BD),
                             pKeypressNotificationReq->eventType
                             );

   btsmHandleKeypressNotificationReq(pKeypressNotificationReq->remote_BD,
                                              (uint8_t)pKeypressNotificationReq->eventType
                                              );
}

/****************************************************************************/
/* blueAPI_Handle_RemoteOOBDataReqConf                                      */
/****************************************************************************/
void blueAPI_Handle_RemoteOOBDataReqConf( PBlueAPI_RemoteOOBDataReqConf pRemoteOOBDataReqConf)
{
   BLUEAPI_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_TRACE,
                             "blueAPI:<<< RemoteOOBDataReqConf BD[%s] cause[0x%x]",
                             BLUEAPI_TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE, pRemoteOOBDataReqConf->remote_BD),
                             pRemoteOOBDataReqConf->cause
                             );

   if (pRemoteOOBDataReqConf->cause == blueAPI_CauseNotSupported)
   {
      BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE,
                                "!!blueAPI: terminated"
                                );
      return;
   }
   btsmHandleRemoteOOBDataReqConf(pRemoteOOBDataReqConf->remote_BD,
                                  pRemoteOOBDataReqConf->C,
                                  NULL,
                                  (pRemoteOOBDataReqConf->cause == blueAPI_CauseAccept) ? 0 : 1
                                  );
}

void blueAPI_Handle_LegacyRemoteOOBDataReqConf(PBlueAPI_LegacyRemoteOOBDataReqConf pLegacyRemoteOOBDataReqConf)
{
   BLUEAPI_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_TRACE,
                             "blueAPI:<<< LegacyRemoteOOBDataReqConf BD[%s] cause[0x%x]",
                             BLUEAPI_TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE, pLegacyRemoteOOBDataReqConf->remote_BD),
                             pLegacyRemoteOOBDataReqConf->cause
                             );

   if (pLegacyRemoteOOBDataReqConf->cause == blueAPI_CauseNotSupported)
   {
      BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE,
                                "!!blueAPI: terminated"
                                );
      return;
   }
   btsmHandleRemoteOOBDataReqConf(pLegacyRemoteOOBDataReqConf->remote_BD,
                                  pLegacyRemoteOOBDataReqConf->C,
                                  pLegacyRemoteOOBDataReqConf->R,
                                  (pLegacyRemoteOOBDataReqConf->cause == blueAPI_CauseAccept) ? 0 : 1
                                  );
}
/****************************************************************************/
/* blueAPI_LocalOOBDataReq                                                  */
/****************************************************************************/
void blueAPI_Handle_LocalOOBDataReq(void)
{
   BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE,"blueAPI:<<< LocalOOBDataReq");

   blueAPI_Send_BLUEFACE_CONF_READ_LOCAL_OOB_DATA();
}

/****************************************************************************/
/* PBlueAPI_MCL blueAPI_Handle_AuthResultConf                               */
/****************************************************************************/
void blueAPI_Handle_AuthResultConf(PBlueAPI_AuthResultConf pAuthResultConf)
{
    BLUEAPI_TRACE_PRINTF_3(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:<<< AuthResultConf BD[%s] BDType[%d] cause[0x%x]",
        BLUEAPI_TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE, pAuthResultConf->remote_BD),
        pAuthResultConf->remote_BD_Type, pAuthResultConf->cause);

    switch(pAuthResultConf->cause)
    {
    case blueAPI_CauseSuccess:
        switch (pBlueAPIData->ConfigParameter.StoreBondMode)
        {
        case blueAPI_StoreBondModeNVStore:
        case blueAPI_StoreBondModeRAMStore:
            blueAPIStore_SetPeerInfo(pBlueAPIData, pAuthResultConf->remote_BD,
                pAuthResultConf->remote_BD_Type, NULL, 0, NULL);
            break;

        default:
            break;
        }

        if (pBlueAPIData->pDSCurrentMsg != NULL)
        {
            PDEVICE_DATA_IND pDeviceDataInd = &pBlueAPIData->pDSCurrentMsg->p.deviceDataIndication;

            if ((pAuthResultConf->remote_BD_Type == pDeviceDataInd->deviceData.bdType) &&
                (memcmp(pAuthResultConf->remote_BD, pDeviceDataInd->deviceData.bd, BLUE_API_BD_SIZE) == 0))
            {
                pDeviceDataInd->deviceData.elementCount = 0;

                btApiDEVICE_DATA_SET_RESP(pDeviceDataInd->handle, BTSEC_SUCCESS, &pDeviceDataInd->deviceData);

                blueAPIStore_QueueTrigger(pBlueAPIData, TRUE);
            }
            else
            {
#if SEND_INT_EVENT_INFO
                blueAPI_Send_InternalEventInfo(blueAPI_InternalEventInvalidSecurityConf,
                    BLUE_API_GENERATE_EVENT_ID, blueAPI_CauseInvalidParameter);
#endif
            }
        }
        break;

    case blueAPI_CauseNotSupported:
        BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE, "!!blueAPI: terminated");
        break;

    default:
#if SEND_INT_EVENT_INFO
        blueAPI_Send_InternalEventInfo(blueAPI_InternalEventInvalidSecurityConf,
            BLUE_API_GENERATE_EVENT_ID, blueAPI_CauseInvalidParameter);
#endif
        break;
    }
}

/****************************************************************************/
/* void blueAPI_Handle_AuthDeleteReq                                        */
/****************************************************************************/
void blueAPI_Handle_AuthDeleteReq(PBlueAPI_DsMessage pReqMsg)
{
    PBlueAPI_AuthDeleteReq pAuthDeleteReq = &pReqMsg->p.AuthDeleteReq;
    TBlueAPI_Cause cause = blueAPI_CauseInvalidState;

    BLUEAPI_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_TRACE, "blueAPI:<<< AuthDeleteReq BD[%s] BDType[%d]",
        BLUEAPI_TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE, pAuthDeleteReq->remote_BD),
        pAuthDeleteReq->remote_BD_Type);

    if (pBlueAPIData->SM_SEC_DS_CommandMsg == NULL)
    {
        BOOL found = blueAPIStore_DeletePeer(pBlueAPIData, pAuthDeleteReq->remote_BD,
            pAuthDeleteReq->remote_BD_Type);

        cause = (found) ? blueAPI_CauseSuccess : blueAPI_CauseInvalidParameter;
    }

    blueAPI_Send_AuthDeleteRsp(pReqMsg, pAuthDeleteReq->remote_BD, pAuthDeleteReq->remote_BD_Type, cause);
}

/****************************************************************************/
/* void blueAPI_Handle_AuthListReq                                          */
/* (                                                                        */
/*    PBlueAPI_Data                     pBlueAPIdata                        */
/*    PBlueAPI_AuthListReq         pAuthListReq                             */
/* )                                                                        */
/****************************************************************************/
void blueAPI_Handle_AuthListReq(PBlueAPI_AuthListReq pAuthListReq)
{
    BOOL found;

    BLUEAPI_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_TRACE, "blueAPI:<<< AuthListReq BD[%s] BDType[%d]",
        BLUEAPI_TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE, pAuthListReq->remote_BD),
        pAuthListReq->remote_BD_Type);

    found = blueAPIStore_SendAuthList(pBlueAPIData, pAuthListReq->remote_BD,
        pAuthListReq->remote_BD_Type);

    blueAPI_Send_AuthListRsp( pAuthListReq->remote_BD, pAuthListReq->remote_BD_Type,
        found ? blueAPI_CauseSuccess : blueAPI_CauseInvalidParameter);
}

/****************************************************************************/
/* void blueAPI_HandleEventDeviceConfigSetReq                               */
/****************************************************************************/
void blueAPI_Handle_DeviceConfigSetReq(PBlueAPI_DsMessage pReqMsg)
{
    PBlueAPI_DeviceConfigSetReq pDeviceConfigSetReq = &pReqMsg->p.DeviceConfigSetReq;
    BOOL sendResp = TRUE;
    TBlueAPI_Cause cause = blueAPI_CauseSuccess;

    do
    {
        if (pBlueAPIData->SM_SEC_DS_CommandMsg != NULL)
        {
            cause = blueAPI_CauseInvalidState;
            break;
        }

        switch (pDeviceConfigSetReq->opCode)
        {
        case blueAPI_DeviceConfigDevice:
        {
            uint32_t classOfDevice = pDeviceConfigSetReq->p.dev.classOfDevice;

            BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE,
                "blueAPI:<<< DeviceConfigSetReq opCode[blueAPI_DeviceConfigDevice] COD[0x%X]",
                classOfDevice);

            pBlueAPIData->ConfigParameter.ClassOfDevice = classOfDevice;
            if (pBlueAPIData->ConfigParameter.limitedDiscovery)
            {
                classOfDevice |= HCI_SERVICE_CLASS_LIMITED_DISCOVERABLE_MODE;
            }
            hciHandleUpWriteClassOfDevice((uint8_t *)&classOfDevice);

            blueAPI_SetDeviceName(pBlueAPIData, pDeviceConfigSetReq->p.dev.deviceName);
        }
            break;

        case blueAPI_DeviceConfigDID:
            BLUEAPI_TRACE_PRINTF_4(BLUEAPI_TRACE_MASK_TRACE,
                "blueAPI:<<< DeviceConfigSetReq opCode[blueAPI_DeviceConfigDID] VID[0x%X] PID[0x%X] PV[0x%X] IDsrc[%d]",
                pDeviceConfigSetReq->p.did.vendorID, pDeviceConfigSetReq->p.did.productID,
                pDeviceConfigSetReq->p.did.productVersion,pDeviceConfigSetReq->p.did.vendorIDSource);

            sdpHandleConfigDID(pDeviceConfigSetReq->p.did.vendorID,
                               pDeviceConfigSetReq->p.did.productID,
                               pDeviceConfigSetReq->p.did.productVersion,
                               pDeviceConfigSetReq->p.did.vendorIDSource
                               );
            break;

        case blueAPI_DeviceConfigExtraEIR:
            sdpHandleExtraEIRdata((char *)pDeviceConfigSetReq->p.extraEIR.pdata);
            break;

        case blueAPI_DeviceConfigSecurity:
            BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE,
                "blueAPI:<<< DeviceConfigSetReq opCode[blueAPI_DeviceConfigSecurity] fixVal[%d]",
                pDeviceConfigSetReq->p.security.leFixedDisplayValue);

            {
                uint32_t fixedValue = pDeviceConfigSetReq->p.security.leFixedDisplayValue;

                if (fixedValue & BLUE_API_USE_LE_FIXED_DISPLAY_VALUE)
                {
                    fixedValue &= ~(BLUE_API_USE_LE_FIXED_DISPLAY_VALUE);
                }
                else
                {
                    fixedValue = 0xFFFFFFFF;
                }

                pBlueAPIData->SM_SEC_DS_CommandMsg = pReqMsg;
                blueAPI_Send_BLUEFACE_CONF_LE_SSP_PARAMETER(pBlueAPIData, fixedValue);
                sendResp = FALSE;
            }
            break;

        case blueAPI_DeviceConfigStore: 
            BLUEAPI_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_TRACE,
                "blueAPI:<<< DeviceConfigSetReq opCode[blueAPI_DeviceConfigStore] bMode[%d] bSize[%d]",
                pDeviceConfigSetReq->p.store.storeBondMode, pDeviceConfigSetReq->p.store.storeBondSize);

            if ((pDeviceConfigSetReq->p.store.storeBondMode <= blueAPI_StoreBondModeExtStore) &&
                (pDeviceConfigSetReq->p.store.storeBondSize >= 1) &&
                (pDeviceConfigSetReq->p.store.storeBondSize <= pBlueAPIData->DataStore.peerCount)
                )
            {
                blueAPIStore_ChangeMode(pBlueAPIData,
                                        pDeviceConfigSetReq->p.store.storeBondMode,
                                        pDeviceConfigSetReq->p.store.storeBondSize,
                                        FALSE
                                        );
            }
            else
            {
                cause = blueAPI_CauseInvalidParameter;
            }
            break;

        case blueAPI_DeviceConfigPagescan:
            BLUEAPI_TRACE_PRINTF_4(BLUEAPI_TRACE_MASK_TRACE,
                "blueAPI:<<< DeviceConfigSetReq opCode[blueAPI_DeviceConfigPagescan] repMode[%d] scanType[%d] repInterval[%d] repWindow[%d]",
                pDeviceConfigSetReq->p.pagescan.repMode, pDeviceConfigSetReq->p.pagescan.scanType,
                pDeviceConfigSetReq->p.pagescan.repInterval, pDeviceConfigSetReq->p.pagescan.repWindow);

            if (((pDeviceConfigSetReq->p.pagescan.repMode <= 2) || (pDeviceConfigSetReq->p.pagescan.repMode == 0xFF)) &&
                (pDeviceConfigSetReq->p.pagescan.scanType <= 1) &&
                ( 0x12 <= pDeviceConfigSetReq->p.pagescan.repInterval && pDeviceConfigSetReq->p.pagescan.repInterval <= 0x1000) &&
                ( 0x11 <= pDeviceConfigSetReq->p.pagescan.repWindow && pDeviceConfigSetReq->p.pagescan.repWindow <= 0x1000) &&
                (pDeviceConfigSetReq->p.pagescan.repWindow <= pDeviceConfigSetReq->p.pagescan.repInterval) &&
                (pDeviceConfigSetReq->p.pagescan.pageTimeout)
                )
            {
                hciHandleConfPageScanActivityEx(pDeviceConfigSetReq->p.pagescan.repMode,
                                                pDeviceConfigSetReq->p.pagescan.scanType,
                                                pDeviceConfigSetReq->p.pagescan.repInterval,
                                                pDeviceConfigSetReq->p.pagescan.repWindow
                                                );
                hciHandleConfPageCharacteristic(pDeviceConfigSetReq->p.pagescan.repMode,
                                                pDeviceConfigSetReq->p.pagescan.pageTimeout
                                                );
            }
            else
            {
                cause = blueAPI_CauseInvalidParameter;
            }
            break;

        case blueAPI_DeviceConfigInquiryscan:
            BLUEAPI_TRACE_PRINTF_3(BLUEAPI_TRACE_MASK_TRACE,
                "blueAPI:<<< DeviceConfigSetReq opCode[blueAPI_DeviceConfigInquiryscan] scanType[%d] interval[%d] window[%d]",
                pDeviceConfigSetReq->p.inquiryscan.scanType, pDeviceConfigSetReq->p.inquiryscan.interval,
                pDeviceConfigSetReq->p.inquiryscan.window);

            if ((pDeviceConfigSetReq->p.inquiryscan.scanType <= 1) &&
                (0x12 <= pDeviceConfigSetReq->p.inquiryscan.interval && pDeviceConfigSetReq->p.inquiryscan.interval <= 0x1000) &&
                (0x11 <= pDeviceConfigSetReq->p.inquiryscan.window && pDeviceConfigSetReq->p.inquiryscan.window <= 0x1000) &&
                (pDeviceConfigSetReq->p.inquiryscan.window <= pDeviceConfigSetReq->p.inquiryscan.interval) &&
                ((pDeviceConfigSetReq->p.inquiryscan.interval & 0x01) == 0)
                )
            {
                hciHandleConfInquiryScanActivityEx(pDeviceConfigSetReq->p.inquiryscan.scanType,
                                                   pDeviceConfigSetReq->p.inquiryscan.interval,
                                                   pDeviceConfigSetReq->p.inquiryscan.window
                                                   );
            }
            else
            {
                cause = blueAPI_CauseInvalidParameter;
            }
            break;

        case blueAPI_DeviceConfigLinkpolicy:
            BLUEAPI_TRACE_PRINTF_3(BLUEAPI_TRACE_MASK_TRACE,
                "blueAPI:<<< DeviceConfigSetReq opCode[blueAPI_DeviceConfigLinkpolicy] deviceRole[%d] linkPolicy[%d] supervisionTimeout[%d]",
                pDeviceConfigSetReq->p.linkpolicy.deviceRole, pDeviceConfigSetReq->p.linkpolicy.linkPolicy,
                pDeviceConfigSetReq->p.linkpolicy.supervisionTimeout);

            if ((pDeviceConfigSetReq->p.linkpolicy.deviceRole <= 4) &&
                (pDeviceConfigSetReq->p.linkpolicy.linkPolicy < 0x10 )
                )
            {
                hciHandleConfLinkPolicyDefault(pDeviceConfigSetReq->p.linkpolicy.deviceRole,
                                               pDeviceConfigSetReq->p.linkpolicy.linkPolicy,
                                               pDeviceConfigSetReq->p.linkpolicy.supervisionTimeout
                                               );
            }
            else
            {
                cause = blueAPI_CauseInvalidParameter;
            }
            break;

        case blueAPI_DeviceConfigDeviceName:
            {
            uint16_t deviceNameLen = 0;
            BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE,
                "blueAPI:<<< DeviceConfigSetReq opCode[blueAPI_DeviceConfigDeviceName] ");

            deviceNameLen = (uint16_t)strlen((char *)pDeviceConfigSetReq->p.device.deviceName);
            gattHandleCONFIG_GAP_DEVICE_NAME(pDeviceConfigSetReq->p.device.deviceName, deviceNameLen);
            }
            break;

        case blueAPI_DeviceConfigAppearance: 
            BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE,
                                    "blueAPI:<<< DeviceConfigSetReq opCode[blueAPI_DeviceConfigAppearance] appearance[0x%x]",
                                    pDeviceConfigSetReq->p.appearance.appearance
                                    );
            gattHandleCONFIG_GAP_APPEARANCE(pDeviceConfigSetReq->p.appearance.appearance);
            break;

        case blueAPI_DeviceConfigPerPrefConnParam: 
            BLUEAPI_TRACE_PRINTF_4(BLUEAPI_TRACE_MASK_TRACE,
                "blueAPI:<<< DeviceConfigSetReq opCode[blueAPI_DeviceConfigPerPrefConnParam] connIntervalMin[0x%x] connIntervalMax[0x%x] slaveLatency[0x%x] supervisionTimeout[0x%x]",
                pDeviceConfigSetReq->p.conn.connIntervalMin, pDeviceConfigSetReq->p.conn.connIntervalMax,
                pDeviceConfigSetReq->p.conn.slaveLatency, pDeviceConfigSetReq->p.conn.supervisionTimeout);

            gattHandleCONFIG_GAP_PER_PREF_CONN_PARAM(pDeviceConfigSetReq->p.conn.connIntervalMin,
                pDeviceConfigSetReq->p.conn.connIntervalMax, pDeviceConfigSetReq->p.conn.slaveLatency,
                pDeviceConfigSetReq->p.conn.supervisionTimeout);
            break;

        case blueAPI_DeviceConfigInquirymode:
            BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE,
                "blueAPI:<<< DeviceConfigSetReq opCode[blueAPI_DeviceConfigInquirymode] mode[%d]",
                pDeviceConfigSetReq->p.inquirymode.mode);

            if (pDeviceConfigSetReq->p.inquirymode.mode <= 2)
            {
                hciHandleConfInquiryMode(pDeviceConfigSetReq->p.inquirymode.mode);
            }
            else
            {
                cause = blueAPI_CauseInvalidParameter;
            }
            break;

        default:
            cause = blueAPI_CauseInvalidParameter;
            break;
        }
    } while (0);

    if (sendResp)
    {
        blueAPI_Send_DeviceConfigSetRsp(pReqMsg, pDeviceConfigSetReq->opCode, cause);
    }

} /* end of blueAPI_HandleEventDeviceConfigSetReq */

/****************************************************************************/
/* void blueAPI_Handle_ACLConfigReq                                         */
/****************************************************************************/
void blueAPI_Handle_ACLConfigReq(PBlueAPI_DsMessage pReqMsg)
{
    PBlueAPI_ACLConfigReq pACLConfigReq = &pReqMsg->p.ACLConfigReq;
    TBlueAPI_Cause cause = blueAPI_CauseInvalidParameter;
    bool send_rsp = true;

    if (pBlueAPIData->SM_SEC_DS_CommandMsg == NULL)
    {
        switch (pACLConfigReq->opCode)
        {
        case blueAPI_ACLConfigLinkpolicy:
            if (pACLConfigReq->remote_BD_Type == blueAPI_RemoteBDTypeClassic)
            {
                BLUEAPI_TRACE_PRINTF_5(BLUEAPI_TRACE_MASK_TRACE,
                    "blueAPI:<<< ACLConfigReq BD[%s] BDtype[%d] opCode[blueAPI_ACLConfigLinkpolicy] deviceRole[%d] linkPolicy[%d] supervisionTimeout[%d]",
                    BLUEAPI_TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE, pACLConfigReq->remote_BD),
                    pACLConfigReq->remote_BD_Type, pACLConfigReq->p.linkpolicy.deviceRole,
                    pACLConfigReq->p.linkpolicy.linkPolicy,
                    pACLConfigReq->p.linkpolicy.supervisionTimeout);

                if ((pACLConfigReq->p.linkpolicy.deviceRole <= 4) &&
                    (pACLConfigReq->p.linkpolicy.linkPolicy <= 0x10))
                {
                    hciHandleConfLinkPolicy(pACLConfigReq->remote_BD,
                        pACLConfigReq->p.linkpolicy.deviceRole,
                        pACLConfigReq->p.linkpolicy.linkPolicy,
                        pACLConfigReq->p.linkpolicy.supervisionTimeout);

                    cause = blueAPI_CauseSuccess;
                }
            }
            break;

        case blueAPI_ACLConfigSniffmode:
            if (pACLConfigReq->remote_BD_Type == blueAPI_RemoteBDTypeClassic)
            {
                BLUEAPI_TRACE_PRINTF_6(BLUEAPI_TRACE_MASK_TRACE,
                    "blueAPI:<<< ACLConfigReq BD[%s] BDtype[%d] opCode[blueAPI_ACLConfigSniffmode] min[%d] max[%d] att[%d] timeout[%d]",
                    BLUEAPI_TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE, pACLConfigReq->remote_BD),
                    pACLConfigReq->remote_BD_Type, pACLConfigReq->p.sniffmode.minInterval,
                    pACLConfigReq->p.sniffmode.maxInterval, pACLConfigReq->p.sniffmode.sniffAttempt,
                    pACLConfigReq->p.sniffmode.sniffTimeout);

                pBlueAPIData->SM_SEC_DS_CommandMsg = pReqMsg;
                send_rsp = hciHandleUpSniffReq(pACLConfigReq->remote_BD,
                    pACLConfigReq->remote_BD_Type, pACLConfigReq->p.sniffmode.maxInterval,
                    pACLConfigReq->p.sniffmode.minInterval, pACLConfigReq->p.sniffmode.sniffAttempt,
                    pACLConfigReq->p.sniffmode.sniffTimeout);
            }
            break;

        default:
            break;
        }
    }
    else
    {
        cause = blueAPI_CauseInvalidState;
    }

    if (send_rsp == true)
    {
        blueAPI_Send_ACLConfigRsp(pReqMsg, pACLConfigReq->remote_BD, pACLConfigReq->remote_BD_Type,
            pACLConfigReq->opCode, cause);
    }
} /* blueAPI_Handle_ACLConfigReq */

/****************************************************************************
 * blueAPI_Handle_LEAdvertiseReq()
 ****************************************************************************/
void blueAPI_Handle_LEAdvertiseReq(PBlueAPI_LEAdvertiseReq pAdvReq)
{
    BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:<<< LEAdvertiseReq mode[%d]", pAdvReq->advMode);

    switch (pAdvReq->advMode)
    {
    case blueAPI_LEAdvModeDisabled:
        gattHandleGATT_LE_ADVERTISE_REQ(GATT_LE_ADVERTISE_OPCODE_AD_DISABLE, 0, NULL);
        break;

    case blueAPI_LEAdvModeEnabled:
        gattHandleGATT_LE_ADVERTISE_REQ(GATT_LE_ADVERTISE_OPCODE_AD_ENABLE, 0, NULL);
        break;

    case blueAPI_LEAdvModeDirectedHighDuty:
        gattHandleGATT_LE_ADVERTISE_REQ(GATT_LE_ADVERTISE_OPCODE_AD_DIRECTED,0,NULL);
        break;

    default:
        blueAPI_Send_LEAdvertiseRsp(pAdvReq->advMode, blueAPI_CauseInvalidParameter);
        break;
    }
}

/****************************************************************************
 * blueAPI_Handle_LEAdvertiseParameterSetReq()
 ****************************************************************************/
void blueAPI_Handle_LEAdvertiseParameterSetReq(PBlueAPI_LEAdvertiseParameterSetReq pAdvParamReq)
{
    TGATT_LE_ADVERTISE_PARAMETER advParam;

    BLUEAPI_TRACE_PRINTF_5(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:<<< LEAdvertiseParameterSetReq type[%d] scanWL[%d] connWL[%d] interval[%d]-[%d]",
        pAdvParamReq->advType, pAdvParamReq->filterScanReq, pAdvParamReq->filterConnectReq,
        pAdvParamReq->minAdvInterval, pAdvParamReq->maxAdvInterval);

    if ((pAdvParamReq->advType == blueAPI_LEAdvTypeDirectedLowDuty) ||
        (pAdvParamReq->advType == blueAPI_LEAdvTypeDirectedHighDuty))
    {
        if (pAdvParamReq->remote_BD_type & BLUEFACE_BDTYPE_LE_MASK)
        {
            advParam.bdType = pAdvParamReq->remote_BD_type;
            memcpy(advParam.bd, pAdvParamReq->remote_BD, BD_ADDR_SIZE);
        }
        else
        {
            blueAPI_Send_LEAdvertiseParameterSetRsp(blueAPI_CauseInvalidParameter);
            return;
        }
    }
    else
    {
        advParam.bdType = 0;
        memset(advParam.bd,0,BD_ADDR_SIZE);
    }

    advParam.advType          = pAdvParamReq->advType;
    advParam.advChannelMap    = LE_ADVERTISING_CHANNEL_ALL;
    advParam.minAdvInterval   = pAdvParamReq->minAdvInterval;
    advParam.maxAdvInterval   = pAdvParamReq->maxAdvInterval;
    advParam.filterPolicy     = (pAdvParamReq->filterScanReq) ? 0x01 : 0x00;
    advParam.filterPolicy    |= (pAdvParamReq->filterConnectReq) ? 0x02 : 0x00;
    advParam.local_bdType     = pAdvParamReq->local_BD_type;

    btsmUpdateLocalBdType(pAdvParamReq->local_BD_type);
    gattHandleGATT_LE_ADVERTISE_REQ(GATT_LE_ADVERTISE_OPCODE_AD_PARAMETER,
        sizeof(TGATT_LE_ADVERTISE_PARAMETER),(uint8_t *)&advParam);
}

/****************************************************************************
 * blueAPI_Handle_LEAdvertiseDataSetReq()
 ****************************************************************************/
void blueAPI_Handle_LEAdvertiseDataSetReq(PBlueAPI_LEAdvertiseDataSetReq pAdvDataReq)
{
    uint8_t opCode = GATT_LE_ADVERTISE_OPCODE_AD_DATA;

    BLUEAPI_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:<<< LEAdvertiseDataSetReq type[%d] length[%d]",
        pAdvDataReq->dataType, pAdvDataReq->dataLength);

    if (pAdvDataReq->dataType == blueAPI_LEDataTypeScanResponseData)
    {
        opCode = GATT_LE_ADVERTISE_OPCODE_SC_RSP;
    }

    gattHandleGATT_LE_ADVERTISE_REQ(opCode, pAdvDataReq->dataLength, pAdvDataReq->pDataBuffer);
}

/****************************************************************************
 * blueAPI_Handle_LEScanReq()
 ****************************************************************************/
void blueAPI_Handle_LEScanReq(PBlueAPI_LEScanReq pScanReq)
{
    BLUEAPI_TRACE_PRINTF_5(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:<<< LEScanReq mode[%d] interval[%d] window[%d] filterPolicy[%d] filterDups[%d]",
        pScanReq->scanMode, pScanReq->scanInterval, pScanReq->scanWindow, pScanReq->filterPolicy,
        pScanReq->filterDuplicates);

    btApiGATT_LE_SCAN_REQ((pScanReq->scanMode > 0), (pScanReq->scanMode > 1),
        pScanReq->scanInterval, pScanReq->scanWindow, pScanReq->filterPolicy,
        pScanReq->local_BD_Type, pScanReq->filterDuplicates);
}

/****************************************************************************
 * blueAPI_Handle_LEModifyWhitelistReq()
 ****************************************************************************/
void blueAPI_Handle_LEModifyWhitelistReq(PBlueAPI_LEModifyWhitelistReq pModWhitelistReq)
{
    BLUEAPI_TRACE_PRINTF_3(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:<<< LEModifyWhitelistReq op[%d] BD[%s] BDtype[%d]", pModWhitelistReq->operation,
        BLUEAPI_TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE, pModWhitelistReq->remote_BD),
        pModWhitelistReq->remote_BD_type);

    /* only allow LE address types */
    if (pModWhitelistReq->remote_BD_type & BLUEFACE_BDTYPE_LE_MASK)
    {
        gattHandleGATT_LE_MODIFY_WHITELIST_REQ(pModWhitelistReq->operation,
            pModWhitelistReq->remote_BD, pModWhitelistReq->remote_BD_type);
    }
    else
    {
        blueAPI_Send_LEModifyWhitelistRsp(pModWhitelistReq->operation, blueAPI_CauseInvalidParameter);
    }
}

/****************************************************************************
 * blueAPI_Handle_LEConnectionUpdateReq()
 ****************************************************************************/
void blueAPI_Handle_LEConnectionUpdateReq( PBlueAPI_LEConnectionUpdateReq pUpdateReq)
{
    PBlueAPI_LinkDescriptor pLinkContext = blueAPI_ChannelFindByLocal_MDL_ID(pUpdateReq->local_MDL_ID);

    BLUEAPI_TRACE_PRINTF_5(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:<<< LEConnectionUpdateReq MDL[%d] connInterval[%d-%d] latency[%d] supervision[%d]",
        pUpdateReq->local_MDL_ID, pUpdateReq->connIntervalMin, pUpdateReq->connIntervalMax,
        pUpdateReq->connLatency, pUpdateReq->supervisionTimeout);

    if (pLinkContext != NULL)
    {
        btApiGATT_LE_CONNECTION_UPDATE_REQ(pLinkContext->handle, pUpdateReq->connIntervalMin,
            pUpdateReq->connIntervalMax, pUpdateReq->connLatency, pUpdateReq->supervisionTimeout, 0, 0);
    }
    else
    {
        blueAPI_Send_LEConnectionUpdateRsp(pUpdateReq->local_MDL_ID, GATT_ERR_ILLEGAL_PARAMETER);
    }
}

/****************************************************************************
 * blueAPI_Handle_LEConnectionUpdateConf()
 ****************************************************************************/
void blueAPI_Handle_LEConnectionUpdateConf( PBlueAPI_LEConnectionUpdateConf pUpdateConf)
{
    PBlueAPI_LinkDescriptor pLinkContext = blueAPI_ChannelFindByLocal_MDL_ID(pUpdateConf->local_MDL_ID);

    BLUEAPI_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI:<<< LEConnectionUpdateConf MDL[%d] cause[0x%x]",
        pUpdateConf->local_MDL_ID, pUpdateConf->cause);

    if (pLinkContext != NULL)
    {
        LPbtLink pLink = (LPbtLink)(pLinkContext->handle);
        gattHandleGATT_LE_CONNECTION_UPDATE_RESP(pLink->handle,
            (pUpdateConf->cause == blueAPI_CauseAccept) ? GATT_SUCCESS : GATT_ERR);
    }
    else
    {
#if SEND_INT_EVENT_INFO
        blueAPI_Send_InternalEventInfo(blueAPI_InternalEventInvalidLEConnectionUpdateConf,
            BLUE_API_GENERATE_EVENT_ID, blueAPI_CauseInvalidParameter);
#endif
    }
}

void blueAPI_Handle_SetRandomAddressReq(PBlueAPI_SetRandomAddressReq pSetRandomAddrReq)
{
    uint16_t cause = GATT_SUCCESS;

    if (btApiGATT_SET_RANDOM_ADDRESS_REQ(pSetRandomAddrReq->random_BD) != blueFaceNoError)
    {
        cause = GATT_ERR_OUT_OF_RESOURCE;
    }

    if (cause != GATT_SUCCESS)
    {
        blueAPI_Send_SetRandomAddressRsp(cause);
    }
}

void blueAPI_Handle_SDPRegisterReq(PBlueAPI_SDPRegisterReq pSDPRegisterReq)
{
    BOOL ret = sdpHandleUpRegReq(pSDPRegisterReq->buf, pSDPRegisterReq->offset);

    if(ret == FALSE)
    {
        blueAPI_Send_SDPRegisterRsp(blueAPI_CauseResourceError);
    }
    else
    {
        blueAPI_Send_SDPRegisterRsp(blueAPI_CauseSuccess);
    }
}

void blueAPI_Handle_L2cProtocolRegisterReq(PBlueAPI_L2cProtocolRegisterReq pL2cProtocolRegisterReq)
{
    l2cUSendListenReq(pL2cProtocolRegisterReq->psm, pL2cProtocolRegisterReq->listenQueue, pL2cProtocolRegisterReq->action);
}

void blueAPI_Handle_L2cConReq(PBlueAPI_L2cConReq pL2cConReq)
{
    LP_L2C_Conf_para pConfPara;

    pConfPara = commonPrepareL2cConfPara(pL2cConReq->mtuSize, BT_US_PDU_L2C_BYTE_COUNT, 0);

    if(pConfPara == NULL)
    {
        return;
    }

    l2cUSendConReq(pL2cConReq->usQueueID, pL2cConReq->remoteBd,
                    pL2cConReq->psm, pL2cConReq->uuid, pL2cConReq->mtuSize, pConfPara);
}

void blueAPI_Handle_L2cConConf(PBlueAPI_L2cConConf pL2cConConf)
{
    TBtConRespPSpecifc  ConRespExt;
    LP_L2C_Conf_para    pConfPara;

    pConfPara = commonPrepareL2cConfPara(BT_US_PDU_L2C_BYTE_COUNT, BT_US_PDU_L2C_BYTE_COUNT, 0);
    if(pConfPara == 0)
    {
        return;
    }
    ConRespExt.l2cap.statusDetails = 0;
    ConRespExt.l2cap.pConfPara = pConfPara;

    l2cHandleL2C_CON_RESP(pL2cConConf->cid, pL2cConConf->status, &ConRespExt);
}

void blueAPI_Handle_L2cDataReq(PBlueAPI_L2cDataReq pL2cDataReq)
{
    MESSAGE_T msg;

    msg.MData.DataCBChan.BufferAddress  = pL2cDataReq->buf;
    msg.MData.DataCBChan.Offset         = pL2cDataReq->writeOffset;
    msg.MData.DataCBChan.Flag           = DATA_CB_RELEASE;
    msg.MData.DataCBChan.Channel        = pL2cDataReq->cid;
    msg.MData.DataCBChan.Length         = pL2cDataReq->length;

    l2cHandleL2C_DATA_REQ(&msg, FALSE);
}

void blueAPI_Handle_L2cDiscReq(PBlueAPI_L2cDiscReq pL2cDiscReq)
{
    l2cHandleBTG_DISC_REQ(pL2cDiscReq->cid, FALSE);
}

void blueAPI_Handle_L2cDiscConf(PBlueAPI_L2cDiscConf pL2cDiscConf)
{
    l2cHandleL2C_DISC_RESP(pL2cDiscConf->cid);
}

void blueAPI_Handle_L2cSecurityRegisterReq(PBlueAPI_L2cSecurityRegisterReq pL2cSecurityRegReq)
{
    uint16_t cause = HCI_SUCCESS;
    TSEC_CONF_SECURITY  serviceSecurity;

    serviceSecurity.psm             = pL2cSecurityRegReq->psm;
    serviceSecurity.serverChannel   = pL2cSecurityRegReq->server_channel;
    serviceSecurity.outgoing        = pL2cSecurityRegReq->outgoing;
    serviceSecurity.active          = pL2cSecurityRegReq->active;
    serviceSecurity.uuid            = pL2cSecurityRegReq->uuid;
    serviceSecurity.authenSetting   = pL2cSecurityRegReq->authentication;
    serviceSecurity.authorize       = pL2cSecurityRegReq->authorize;
    serviceSecurity.encryption      = pL2cSecurityRegReq->encryption;
    serviceSecurity.mitm            = pL2cSecurityRegReq->mitm;

    cause = btsmHandleConfigurationReq(BLUEFACE_CONF_SECURITY, (uint8_t *)&serviceSecurity, FALSE);

    blueAPI_Send_L2cSecurityRegisterRsp(pL2cSecurityRegReq->psm,
                                        pL2cSecurityRegReq->server_channel,
                                        pL2cSecurityRegReq->outgoing,
                                        pL2cSecurityRegReq->active,
                                        pL2cSecurityRegReq->uuid,
                                        cause
                                        );
}

void blueAPI_Handle_RFCAuthenticationReq(PBlueAPI_RFCAuthenticationReq pRFCAuthenticationReq)
{
     btsmSendMsgAuthenticationInd(pRFCAuthenticationReq->bd,
                                  pRFCAuthenticationReq->channel,
                                  pRFCAuthenticationReq->dlci,
                                  pRFCAuthenticationReq->uuid,
                                  pRFCAuthenticationReq->outgoing,
                                  pRFCAuthenticationReq->active,
                                  SECMAN_SOURCE_RFCOMM,
                                  BLUEFACE_CON_TYPE_BR_EDR
                                  );
}

#if (F_BT_SCO)
/**
 * @brief  handle sco connection request
 *
 * @param  pSCOConReq
 *
 * @return
 *
 */
void blueAPI_Handle_SCOConReq(PBlueAPI_SCOConReq pSCOConReq)
{
    TCON_REQ_SCO conReqSCO;
    uint8_t status;

    BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE, "blueAPI:<<< blueAPI_SCOConnectReq");

    conReqSCO.transmitBandwidth = pSCOConReq->txBandwidth;
    conReqSCO.receiveBandwidth = pSCOConReq->rxBandwidth;
    conReqSCO.maxLatency = pSCOConReq->maxLatency;
    conReqSCO.content = pSCOConReq->voiceSetting;
    conReqSCO.retry = pSCOConReq->retransEffort;
    conReqSCO.packetType = pSCOConReq->packetType;

    status = btApiBT_CON_REQ(pBlueAPIData->AppHandle,
                             NULL,
                             pSCOConReq->remote_BD,
                             0,
                             BLUEFACE_PSM_SCO,
                             0,
                             sizeof(TCON_REQ_SCO),
                             (uint8_t *)&conReqSCO
                             );

    if (status != blueFaceNoError)
    {
        blueAPI_Send_SCOConRsp(pSCOConReq->remote_BD, status);
    }
}

/**
 * @brief  handle sco connection confirm
 *
 * @param  pSCOConConf
 *
 * @return
 *
 */
void blueAPI_Handle_SCOConConf(PBlueAPI_SCOConConf pSCOConConf)
{
    TCON_RESP_SCO conRespSCO;
    LPbtLink link;

    BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE, "blueAPI:<<< SCOConnectionConf cause[%d]", pSCOConConf->cause);

    conRespSCO.txBandwidth   = pSCOConConf->txBandwidth;
    conRespSCO.rxBandwidth   = pSCOConConf->rxBandwidth;
    conRespSCO.maxLatency    = pSCOConConf->maxLatency;
    conRespSCO.voiceSetting  = pSCOConConf->voiceSetting;
    conRespSCO.retransEffort = pSCOConConf->retransEffort;
    conRespSCO.packetType    = pSCOConConf->packetType;

    link = blueFaceFindLinkByBDandPSM((LPCBYTE)pSCOConConf->remote_BD, BLUEFACE_PSM_SCO);

    if (link)
    {
        if (pSCOConConf->cause == blueAPI_CauseAccept)
        {
            btApiBT_CON_RESP((BLINKHANDLE)link, NULL, BLUEFACE_CON_ACCEPT,
                sizeof(TCON_RESP_SCO),(uint8_t *)&conRespSCO);
        }
        else
        {
            btApiBT_CON_RESP((BLINKHANDLE)link, NULL, BLUEFACE_CON_REJECT,
                sizeof(TCON_RESP_SCO), (uint8_t *)&conRespSCO);
        }
    }
    else
    {
        BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_ERROR, "!!!SCOConnectionConf can not find link");
    }
}

/**
 * @brief  handle sco disconnect request
 *
 * @param  pSCODiscReq
 *
 * @return
 *
 */
void blueAPI_Handle_SCODiscReq(PBlueAPI_SCODiscReq pSCODiscReq)
{
    LPbtLink link;

    BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE, "blueAPI:<<< blueAPI_SCODisconnectReq");

    link = blueFaceFindLinkByBDandPSM((LPCBYTE)pSCODiscReq->remote_BD, BLUEFACE_PSM_SCO);

    if (link)
    {
        btApiBT_DISC_REQ((BLINKHANDLE)link, HCI_ERR_OTHER_END_TERMINATE_13, 0);
    }
    else
    {
        BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_ERROR, "!!!blueAPI:<<< SCOConnectionConf can not find link");
    }
}
#endif

#if F_BT_LE_BT41_SUPPORT
uint16_t blueAPI_CheckConnection(PBlueAPI_LinkDescriptor pLinkContext)
{
    uint16_t            cause = L2CAP_NO_CAUSE;
    
    if (pLinkContext != NULL)
    {
        switch (pLinkContext->linkState)
        {
        case blueAPI_Connected:
            if (pLinkContext->MDLConnected)
            {
                break;
            }
        /* no break */
        case blueAPI_Disconnecting:
        case blueAPI_Disconnected:
            if (pLinkContext->pMDL && pLinkContext->pMDL->pendingCause != blueAPI_CauseSuccess)
            {
                cause = pLinkContext->pMDL->pendingCause;
            }
            else
            {
                cause = L2CAP_ERR_ILLEGAL_STATE | L2CAP_ERR;
            }
            break;

        default:
            cause = L2CAP_ERR_ILLEGAL_PARAMETER | L2CAP_ERR;
            break;
        }
    }
    else
    {
        cause = L2CAP_ERR_ILLEGAL_PARAMETER | L2CAP_ERR;
    }
    return cause;
}

void blueAPI_Handle_CreateLEDataChannelReq(PBlueAPI_CreateLEDataChannelReq pCreateReq)
{
    PBlueAPI_LinkDescriptor pLinkContext = blueAPI_ChannelFindByLocal_MDL_ID(pCreateReq->local_MDL_ID);
    uint16_t            cause = L2CAP_NO_CAUSE;

    cause = blueAPI_CheckConnection(pLinkContext);
    if (cause == L2CAP_NO_CAUSE)
    {
        LPbtLink   pLink = pLinkContext->handle;
        uint16_t handle = gattGetLEConnectionHandle(pLink->handle);
        cause = l2cLEHandle_CreateLEDataChannelReq(handle,
                                            pCreateReq->le_psm,
                                            pCreateReq->mtu,
                                            pCreateReq->mps,
                                            pCreateReq->initialCredits,
                                            pCreateReq->creditsIncrease);
    }
    if (cause != L2CAP_NO_CAUSE)
    {
        blueAPI_Send_CreateLEDataChannelRsp(pCreateReq->local_MDL_ID, 0, cause);
    }
}

void blueAPI_Handle_CreateLEDataChannelConf(PBlueAPI_CreateLEDataChannelConf pCreateConf)
{
    PBlueAPI_LinkDescriptor pLinkContext = blueAPI_ChannelFindByLocal_MDL_ID(pCreateConf->local_MDL_ID);
    uint16_t          cause = L2CAP_NO_CAUSE;
    
    cause = blueAPI_CheckConnection(pLinkContext);
    if (cause == L2CAP_NO_CAUSE)
    {
        if (pCreateConf->cause == blueAPI_CauseAccept)
            pCreateConf->cause = blueAPI_CauseSuccess;
        l2cLEHandle_CreateLEDataChannelConf(pCreateConf->channel,
                                            pCreateConf->mtu,
                                            pCreateConf->mps,
                                            pCreateConf->initialCredits,
                                            pCreateConf->creditsIncrease,
                                            pCreateConf->cause);

    }
    else
    {
#if SEND_INT_EVENT_INFO
        blueAPI_Send_InternalEventInfo(blueAPI_InternalEventInvalidCreateLEDataChannelConf,
                                       BLUE_API_GENERATE_EVENT_ID,
                                       blueAPI_CauseInvalidParameter
                                      );
#endif
    }
}

void blueAPI_Handle_DisconnectLEDataChannelReq(PBlueAPI_DisconnectLEDataChannelReq pDiscReq)
{
    PBlueAPI_LinkDescriptor pLinkContext = blueAPI_ChannelFindByLocal_MDL_ID(pDiscReq->local_MDL_ID);
    uint16_t            cause = L2CAP_NO_CAUSE;
    
    cause = blueAPI_CheckConnection(pLinkContext);
    if (cause == L2CAP_NO_CAUSE)
    {
        cause = l2cLEHandle_DisconnectLEDataChannelReq(pDiscReq->channel);
    }
    if (cause != L2CAP_NO_CAUSE)
    {
        blueAPI_Send_DisconnectLEDataChannelRsp(pDiscReq->local_MDL_ID, 0, cause);
    }
}

void blueAPI_Handle_DisconnectLEDataChannelConf(PBlueAPI_DisconnectLEDataChannelConf pDiscConf)
{
    PBlueAPI_LinkDescriptor pLinkContext = blueAPI_ChannelFindByLocal_MDL_ID(pDiscConf->local_MDL_ID);
    uint16_t          cause = L2CAP_NO_CAUSE;
    
    cause = blueAPI_CheckConnection(pLinkContext);
    if (cause == L2CAP_NO_CAUSE)
    {
        l2cHandleL2C_DISC_RESP(pDiscConf->channel);
    }
    else
    {
#if SEND_INT_EVENT_INFO
        blueAPI_Send_InternalEventInfo(blueAPI_InternalEventInvalidDisconnectLEDataChannelConf,
                                       BLUE_API_GENERATE_EVENT_ID,
                                       blueAPI_CauseInvalidParameter
                                      );
#endif
    }
}

void blueAPI_Handle_SendLEFlowControlCreditReq(PBlueAPI_SendLEFlowControlCreditReq pSendReq)
{
    PBlueAPI_LinkDescriptor pLinkContext = blueAPI_ChannelFindByLocal_MDL_ID(pSendReq->local_MDL_ID);
    uint16_t            cause = L2CAP_NO_CAUSE;
    
    cause = blueAPI_CheckConnection(pLinkContext);
    if (cause == L2CAP_NO_CAUSE)
    {
        cause = l2cLEHandle_SendLEFlowControlCreditReq(pSendReq->channel,
                                               pSendReq->credits);
    }
    if (cause != L2CAP_NO_CAUSE)
    {
        blueAPI_Send_SendLEFlowControlCreditRsp(pSendReq->local_MDL_ID, pSendReq->channel, cause);
    }
}        
       
void blueAPI_Handle_LEDataReq(PBlueAPI_DsMessage pMsg)
{
    uint8_t *               pBuffer      = (uint8_t *)pMsg;
    PBlueAPI_LEDataReq   pDataReq = (PBlueAPI_LEDataReq)&pMsg->p;
    PBlueAPI_LinkDescriptor pLinkContext = blueAPI_ChannelFindByLocal_MDL_ID(pDataReq->local_MDL_ID);
    uint16_t            cause = L2CAP_NO_CAUSE;
    
    cause = blueAPI_CheckConnection(pLinkContext);
    if (cause == L2CAP_NO_CAUSE)
    {
        cause = l2cLEHandle_LEDataReq(pDataReq->channel,
                              pBuffer,
                              pDataReq->gap,
                              pDataReq->valueLength
                              );
    }
    if (cause != L2CAP_NO_CAUSE)
    {
        blueAPI_Send_LEDataRsp(pDataReq->local_MDL_ID, pDataReq->channel, cause);
        osBufferRelease((uint8_t *)pMsg);
    }
}

void blueAPI_Handle_LEDataConf(PBlueAPI_LEDataConf pDataConf)
{
    PBlueAPI_LinkDescriptor pLinkContext = blueAPI_ChannelFindByLocal_MDL_ID(pDataConf->local_MDL_ID);
    uint16_t          cause = L2CAP_NO_CAUSE;
    cause = blueAPI_CheckConnection(pLinkContext);
    if (cause == L2CAP_NO_CAUSE)
    {
        if (pDataConf->cause == blueAPI_CauseAccept)
            pDataConf->cause = blueAPI_CauseSuccess;
        l2cLEHandle_LEDataConf(pDataConf->channel,
                               pDataConf->cause);
    }
    else
    {
#if SEND_INT_EVENT_INFO
        blueAPI_Send_InternalEventInfo(blueAPI_InternalEventInvalidLEDataConf,
                                       BLUE_API_GENERATE_EVENT_ID,
                                       blueAPI_CauseInvalidParameter
                                      );
#endif
    } 
}

void blueAPI_Handle_LEPsmSecuritySetReq(PBlueAPI_DsMessage pReqMsg)
{
    PBlueAPI_LEPsmSecuritySetReq pSecSetReq = &pReqMsg->p.LEPsmSecuritySetReq;
    uint16_t cause = HCI_SUCCESS;
    TSEC_CONF_LE_SECURITY serviceSecurity;
    TBlueAPI_Cause param_cause;

    serviceSecurity.psm     = pSecSetReq->le_psm;
    serviceSecurity.active  = pSecSetReq->active;
    serviceSecurity.secMode = pSecSetReq->secMode;
    serviceSecurity.keySize = pSecSetReq->keySize;
    
    cause = btsmHandleConfigurationReq(BLUEFACE_CONF_LE_SECURITY,
                   (uint8_t *)&serviceSecurity, FALSE);
    param_cause = (cause == HCI_SUCCESS) ? blueAPI_CauseSuccess : blueAPI_CauseInvalidParameter; 
    blueAPI_Send_LEPsmSecuritySetRsp(param_cause);
}
#endif


/****************************************************************************
 ******************   End of Secutity Management part ***********************
 ***************************************************************************/
BOOL blueAPI_Handle_Command(PBlueAPI_DsMessage pMsg)
{
    PBlueAPI_DsCommandData pCommandData = &pMsg->p;
    BOOL msgHandled = TRUE;
    PBlueAPI_MCL pMCL;
    uint16_t loop;

    pBlueAPIData->SM_Active++;
    pBlueAPIData->pActiveMCL = NULL;

    switch (pMsg->Command)
    {
    case blueAPI_EventConnectMDLReq:
        blueAPI_Handle_ConnectMDLReq(pBlueAPIData, pMsg);
        msgHandled = FALSE;
        break;

    case blueAPI_EventCreateMDLConf:
        blueAPI_Handle_CreateMDLConf(pBlueAPIData,(PBlueAPI_CreateMDLConf)pCommandData);
        break;

    case blueAPI_EventDisconnectMDLReq:
        blueAPI_Handle_DisconnectMDLReq(pBlueAPIData, pMsg);
        msgHandled = FALSE;
        break;

    case blueAPI_EventDisconnectMDLConf:
        blueAPI_Handle_DisconnectMDLConf(pBlueAPIData,(PBlueAPI_DisconnectMDLConf)pCommandData);
        break;

    case blueAPI_EventRegisterReq:
        blueAPI_Handle_RegisterReq(pBlueAPIData, pMsg);
        msgHandled = FALSE;
        break;

    case blueAPI_EventReleaseReq:
        blueAPI_Handle_ReleaseReq(pMsg);
        msgHandled = FALSE;
        break;

    case blueAPI_EventInquiryReq:
        blueAPI_Handle_InquiryReq(pBlueAPIData, pMsg);
        msgHandled = FALSE;
        break;

    case blueAPI_EventDeviceNameReq:
        blueAPI_Handle_DeviceNameReq(pBlueAPIData, pMsg);
        msgHandled = FALSE;
        break;

    case blueAPI_EventDIDDeviceConf:
        blueAPI_Handle_DIDDeviceConf(pBlueAPIData, (PBlueAPI_DIDDeviceConf)pCommandData);
        break;

    case blueAPI_EventRadioModeSetReq:
        blueAPI_Handle_RadioModeSetReq(pBlueAPIData, pMsg);
        break;

    case blueAPI_EventSDPDiscoveryReq:
        blueAPI_Handle_SDPDiscoveryReq(pBlueAPIData, pMsg);
        msgHandled = FALSE;
        break;

    case blueAPI_EventSDPEndpointConf:
        blueAPI_Handle_SDPEndpointConf(pBlueAPIData, (PBlueAPI_SDPEndpointConf)pCommandData);
        break;

    case blueAPI_EventDeviceConfigSetReq:
        blueAPI_Handle_DeviceConfigSetReq(pMsg);
        msgHandled = FALSE;
        break;

    case blueAPI_EventACLConfigReq:
        blueAPI_Handle_ACLConfigReq(pMsg);
        msgHandled = FALSE;
        break;

    /**************************   Start of GATT part ****************************/
    case blueAPI_EventGATTServiceRegisterReq:
        msgHandled = blueAPI_Handle_GATTServiceRegisterReq(pBlueAPIData, pMsg);
        break;

    case blueAPI_EventGATTAttributeUpdateReq:
        msgHandled = blueAPI_Handle_GATTAttributeUpdateReq(pMsg);
        break;

    case blueAPI_EventGATTAttributeUpdateStatusConf:
        blueAPI_Handle_GATTAttributeUpdateStatusConf(pBlueAPIData, pMsg);
        break;

    case blueAPI_EventGATTAttributeReadConf:
        msgHandled = blueAPI_Handle_GATTAttributeReadConf(pMsg);
        break;

    case blueAPI_EventGATTAttributeWriteConf:
        blueAPI_Handle_GATTAttributeWriteConf(pMsg);
        break;

    case blueAPI_EventGATTAttributePrepareWriteConf:
        msgHandled = blueAPI_Handle_GATTAttributePrepareWriteConf(pMsg);
        break;

    case blueAPI_EventGATTAttributeExecuteWriteConf:
        blueAPI_Handle_GATTAttributeExecuteWriteConf(pMsg);
        break;

    case blueAPI_EventGATTAttributeExecuteWriteReq:
        blueAPI_Handle_GATTAttributeExecuteWriteReq(pMsg);
        break;    

    case blueAPI_EventGATTDiscoveryReq:
        msgHandled = blueAPI_Handle_GATTDiscoveryReq(pMsg);
        break;

    case blueAPI_EventGATTDiscoveryConf:
        blueAPI_Handle_GATTDiscoveryConf(pMsg);
        break;

    case blueAPI_EventGATTAttributeReadReq:
        msgHandled = blueAPI_Handle_GATTAttributeReadReq(pMsg);
        break;

    case blueAPI_EventGATTAttributeWriteReq:
    case blueAPI_EventGATTAttributePrepareWriteReq:
        blueAPI_Handle_GATTAttributeWriteReq(pMsg);
        msgHandled = FALSE;
        break;

    case blueAPI_EventGATTAttributeConf:
        blueAPI_Handle_GATTAttributeConf(pMsg);
        break;

    case blueAPI_EventGATTSDPDiscoveryReq:
        blueAPI_Handle_GATTSDPDiscoveryReq(pMsg);
        msgHandled = FALSE;
        break;

    case blueAPI_EventGATTSDPDiscoveryConf:
        blueAPI_Handle_GATTSDPDiscoveryConf(pMsg);
        break;

    case blueAPI_EventGATTSecurityReq:
        blueAPI_Handle_GATTSecurityReq(pMsg);
        break;

    case blueAPI_EventGATTServerStoreConf:
        blueAPI_Handle_GATTServerStoreConf(pMsg);
        break;
    /**************************  End of GATT part ***************************/

    /*******************  Start of Secutity Management part *********************/
    case blueAPI_EventPairableModeSetReq:
        blueAPI_Handle_PairableModeSetReq(pMsg);
        msgHandled = FALSE;
        break;

    case blueAPI_EventAuthReq:
        blueAPI_Handle_AuthReq(pMsg);
        msgHandled = FALSE;
        break;

    case blueAPI_EventUserAuthRequestConf:
        blueAPI_Handle_UserAuthRequestConf((PBlueAPI_UserAuthRequestConf)pCommandData);
        break;

    case blueAPI_EventAuthResultRequestConf:
        blueAPI_Handle_AuthResultRequestConf((PBlueAPI_AuthResultRequestConf)pCommandData);
        break;

    case blueAPI_EventUserConfirmationReqConf:
        blueAPI_Handle_UserConfirmationReqConf((PBlueAPI_UserConfirmationReqConf)pCommandData);
        break;

    case blueAPI_EventUserAuthorizationReqConf:
        blueAPI_Handle_UserAuthorizationReqConf((PBlueAPI_UserAuthorizationReqConf)pCommandData);
        break;

    case blueAPI_EventUserPasskeyReqConf:
        blueAPI_Handle_UserPasskeyReqConf((PBlueAPI_UserPasskeyReqConf)pCommandData);
        break;

    case blueAPI_EventUserPasskeyReqReplyReq:
        blueAPI_Handle_UserPasskeyReqReplyReq((PBlueAPI_UserPasskeyReqReplyReq)pCommandData);
        break;

    case blueAPI_EventKeypressNotificationReq:
        blueAPI_Handle_KeypressNotificationReq((PBlueAPI_KeypressNotificationReq)pCommandData);
        break;

    case blueAPI_EventRemoteOOBDataReqConf:
        blueAPI_Handle_RemoteOOBDataReqConf((PBlueAPI_RemoteOOBDataReqConf)pCommandData);
        break;

    case blueAPI_EventLegacyRemoteOOBDataReqConf:
        blueAPI_Handle_LegacyRemoteOOBDataReqConf((PBlueAPI_LegacyRemoteOOBDataReqConf)pCommandData);
        break;

    case blueAPI_EventLocalOOBDataReq:
        blueAPI_Handle_LocalOOBDataReq();
        break;

    case blueAPI_EventAuthResultConf:
        blueAPI_Handle_AuthResultConf((PBlueAPI_AuthResultConf)pCommandData);
        break;

    case blueAPI_EventAuthDeleteReq:
        blueAPI_Handle_AuthDeleteReq(pMsg);
        msgHandled = FALSE;
        break;

    case blueAPI_EventAuthListReq:
        blueAPI_Handle_AuthListReq((PBlueAPI_AuthListReq)pCommandData);
        break;
    /*******************  End of Security Management part *****************/

    case blueAPI_EventLEAdvertiseReq:
        blueAPI_Handle_LEAdvertiseReq((PBlueAPI_LEAdvertiseReq)pCommandData);
        break;

    case blueAPI_EventLEAdvertiseParameterSetReq:
        blueAPI_Handle_LEAdvertiseParameterSetReq((PBlueAPI_LEAdvertiseParameterSetReq)pCommandData);
        break;

    case blueAPI_EventLEAdvertiseDataSetReq:
        blueAPI_Handle_LEAdvertiseDataSetReq((PBlueAPI_LEAdvertiseDataSetReq)pCommandData);
        break;

    case blueAPI_EventLEScanReq:
        blueAPI_Handle_LEScanReq((PBlueAPI_LEScanReq)pCommandData);
        break;

    case blueAPI_EventLEModifyWhitelistReq:
        blueAPI_Handle_LEModifyWhitelistReq((PBlueAPI_LEModifyWhitelistReq)pCommandData);
        break;

    case blueAPI_EventLEConnectionUpdateReq:
        blueAPI_Handle_LEConnectionUpdateReq((PBlueAPI_LEConnectionUpdateReq)pCommandData);
        break;

    case blueAPI_EventLEConnectionUpdateConf:
        blueAPI_Handle_LEConnectionUpdateConf((PBlueAPI_LEConnectionUpdateConf)pCommandData);
        break;

    case blueAPI_EventSetRandomAddressReq:
        blueAPI_Handle_SetRandomAddressReq((PBlueAPI_SetRandomAddressReq)pCommandData);
        break;

#if F_BT_LE_BT41_SUPPORT  
    case blueAPI_EventCreateLEDataChannelReq:
        blueAPI_Handle_CreateLEDataChannelReq((PBlueAPI_CreateLEDataChannelReq)pCommandData);
        break;

    case blueAPI_EventCreateLEDataChannelConf:
        blueAPI_Handle_CreateLEDataChannelConf((PBlueAPI_CreateLEDataChannelConf)pCommandData);
        break;

    case blueAPI_EventDisconnectLEDataChannelReq:
        blueAPI_Handle_DisconnectLEDataChannelReq((PBlueAPI_DisconnectLEDataChannelReq)pCommandData);
        break;

    case blueAPI_EventDisconnectLEDataChannelConf:
        blueAPI_Handle_DisconnectLEDataChannelConf((PBlueAPI_DisconnectLEDataChannelConf)pCommandData);
        break;

    case blueAPI_EventSendLEFlowControlCreditReq:
        blueAPI_Handle_SendLEFlowControlCreditReq((PBlueAPI_SendLEFlowControlCreditReq)pCommandData);
        break;

    case blueAPI_EventLEDataReq:
        blueAPI_Handle_LEDataReq(pMsg);
        msgHandled = FALSE;
        break;

    case blueAPI_EventLEDataConf:
        blueAPI_Handle_LEDataConf((PBlueAPI_LEDataConf)pCommandData);
        break;

    case blueAPI_EventLEPsmSecuritySetReq:
        blueAPI_Handle_LEPsmSecuritySetReq(pMsg);
        break;
#endif

    case blueAPI_EventSDPRegisterReq:
        blueAPI_Handle_SDPRegisterReq((PBlueAPI_SDPRegisterReq)pCommandData);
        break;

    case blueAPI_EventL2cProtocolRegisterReq:
        blueAPI_Handle_L2cProtocolRegisterReq((PBlueAPI_L2cProtocolRegisterReq)pCommandData);
        break;

    case blueAPI_EventL2cConReq:
        blueAPI_Handle_L2cConReq((PBlueAPI_L2cConReq)pCommandData);
        break;

    case blueAPI_EventL2cConConf:
        blueAPI_Handle_L2cConConf((PBlueAPI_L2cConConf)pCommandData);
        break;

    case blueAPI_EventL2cDataReq:
        blueAPI_Handle_L2cDataReq((PBlueAPI_L2cDataReq)pCommandData);
        break;

    case blueAPI_EventL2cDiscReq:
        blueAPI_Handle_L2cDiscReq((PBlueAPI_L2cDiscReq)pCommandData);
        break;

    case blueAPI_EventL2cDiscConf:
        blueAPI_Handle_L2cDiscConf((PBlueAPI_L2cDiscConf)pCommandData);
        break;

    case blueAPI_EventL2cSecurityRegisterReq:
        blueAPI_Handle_L2cSecurityRegisterReq((PBlueAPI_L2cSecurityRegisterReq)pCommandData);
        break;

    case blueAPI_EventRFCAuthenticationReq:
        blueAPI_Handle_RFCAuthenticationReq((PBlueAPI_RFCAuthenticationReq)pCommandData);
        break;

#if (F_BT_SCO)
    case blueAPI_EventSCOConReq:
        blueAPI_Handle_SCOConReq((PBlueAPI_SCOConReq)pCommandData);
        break;

    case blueAPI_EventSCOConConf:
        blueAPI_Handle_SCOConConf((PBlueAPI_SCOConConf)pCommandData);
        break;

    case blueAPI_EventSCODiscReq:
        blueAPI_Handle_SCODiscReq((PBlueAPI_SCODiscReq)pCommandData);
        break;
#endif

    default:
        BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE, "!!blueAPI: unhandled COM event (0x%x)", pMsg->Command);
        TRACE_DUMP_SYSTEM(pBlueAPIData); /* error detected */
        break;
    }

    pBlueAPIData->SM_Active--;

    if (msgHandled)
    {  
        osBufferRelease((uint8_t *)pMsg);
    }

    if(pBlueAPIData->pActiveMCL)
    {
        /* check active MCL for idle or retrigger */
        blueAPI_CheckMCLDisconnect(pBlueAPIData, pBlueAPIData->pActiveMCL);
    }
    else
    {
        /* last command did not address anny MCL that was valid at execution  */
        /* time but might have been valid at receiption time and deferred.    */
        /* e.g.: two discReqs in a row address the same valid MDL, but after  */
        /* the first discReq is executed, the second addresses a no longer    */
        /* valid MDL so no MCL context can be found                           */
        for (loop = 0; loop < otp_str_data.gEfuse_UpperStack_s.num_link_don; loop++)
        {
            pMCL = &pBlueAPIData->pMCLDescriptorTableDon[loop];

            if (pMCL->state == blueAPI_DS_Idle)
            {
                blueAPI_CheckMCLDisconnect(pBlueAPIData, pMCL);
            }
        }

        for (loop = 0; loop < otp_str_data.gEfuse_UpperStack_s.num_link_doff; loop++)
        {
            pMCL = &pBlueAPIData->pMCLDescriptorTableDoff[loop];

            if (pMCL->state == blueAPI_DS_Idle)
            {
                blueAPI_CheckMCLDisconnect(pBlueAPIData, pMCL);
            }
        }
    }

    switch (pMsg->Command)
    {
    case blueAPI_EventConnectMDLReq:
    case blueAPI_EventCreateMDLConf:
    case blueAPI_EventDisconnectMDLReq:
    case blueAPI_EventDisconnectMDLConf:
        BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE, "blueAPI: exit COM command [0x%x]", pMsg->Command);
        break;

    default:
        break;
    }

    return msgHandled;
}

