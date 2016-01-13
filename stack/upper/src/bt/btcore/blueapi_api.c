/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       blueapi_api.c
* @brief     bluetooth api func
* @details   
*
* @author   gordon
* @date      2015-07-08
* @version  v0.1
*/

#include <flags.h>
#include <os_message.h>
#include <os_pool.h>
#include <blueface.h>
#include <blueapi_types.h>
#include <blueapi.h>
#include <blueapi_def.h>
#include <blueapi_osif.h>
#include <btsend.h>
#include <sdp_api.h>
#include <l2c_api.h>
#include <upper_stack_global.h>
#include <sdplib.h>
#include <trace_binary.h>

#define TRACE_MODULE_ID     MID_BLUEFACE

static const uint8_t zeros[32] = {0};

/**
 * @brief  send message to blueapi task
 *
 * @param  pBuffer
 * @param  pMsg:  need to send msg
 *
 * @return  
 *
 */
bool blueAPI_SendMessage(PVOID pBuffer, PBlueAPI_DsMessage pMsg)
{
    uint16_t length = sizeof(TBlueAPI_DsMessage);
    MESSAGE_T msg;
    PBlueAPI_DsMessage pBLUE_APIMsg = NULL;

    if ((pMsg->Command == blueAPI_EventGATTAttributeUpdateReq)
        || (pMsg->Command == blueAPI_EventGATTAttributeReadConf)
        || (pMsg->Command == blueAPI_EventGATTAttributeWriteReq)
        || (pMsg->Command == blueAPI_EventGATTAttributePrepareWriteConf)
        || (pMsg->Command == blueAPI_EventGATTAttributePrepareWriteReq) 
        || (pMsg->Command == blueAPI_EventLEDataReq))
    {
        length = pMsg->Length;
    }
    else
    {
        pMsg->Length = length;
    }

    BLUEAPI_TRACE_BINARY_DOWNSTREAM(TRACE_PKT_TYPE_BLUEAPI_OSIF_DS, length, (uint8_t *)pMsg);

    if (pBuffer == NULL)
    {
        if (osBufferGet(pBlueAPIData->dsBlueAPIPoolID, length, (PVOID *) &pBLUE_APIMsg))
        {
            BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE,
                "!!blueAPI_PutMessage: failed to get buffer for cmd[0x%X]", pMsg->Command);

            DebuggerBreak();
            return FALSE;
        }
    }
    else
    {
        pBLUE_APIMsg = (PBlueAPI_DsMessage)pBuffer;
    }

    if (pBLUE_APIMsg != pMsg)
    {
        memcpy(pBLUE_APIMsg, pMsg, length);
    }

    msg.Command                    = BLUE_API_MSG;
    msg.MData.DataCB.Flag          = DATA_CB_RELEASE;
    msg.MData.DataCB.Offset        = 0;
    msg.MData.DataCB.Length        = length;
    msg.MData.DataCB.BufferAddress = (PVOID)pBLUE_APIMsg;

    if (osMessageSend(blueAPIQueueID, &msg))
    {
        BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_ERROR,
            "!!blueAPI_PutMessage: failed to send message cmd[0x%X]", pBLUE_APIMsg->Command);

        osBufferRelease((PVOID)msg.MData.DataCB.BufferAddress);
        return FALSE;
    }

    return TRUE;
}

/**
* @brief  get os buffer
*
* @param  dsPoolID
* @param  dsTPDUSize
* @param  dsDataOffset
* @param  ppBuffer
*
* @return 
*
*/
TBlueAPI_Cause blueAPI_BufferGet(uint16_t dsPoolID, uint16_t dsTPDUSize,
    uint16_t dsDataOffset, void **ppBuffer)
{
    if ((dsDataOffset < BT_DS_WRITE_OFFSET_COUNT) || (dsTPDUSize > BT_DS_PDU_L2C_BYTE_COUNT))
    { 
        return blueAPI_CauseInvalidParameter;
    }

    if (osBufferGet(dsPoolID, dsTPDUSize + dsDataOffset, ppBuffer))
    { 
        return blueAPI_CauseResourceError;
    }

    return blueAPI_CauseSuccess;
}

/**
* @brief  release os buffer
*
* @param  pBuffer
*
* @return  
*
*/
TBlueAPI_Cause blueAPI_BufferRelease(PVOID pBuffer)
{
    if (osBufferRelease(pBuffer))
    { 
        return blueAPI_CauseInvalidParameter;
    }

    return blueAPI_CauseSuccess;
}

/**
 * @brief  internal blueAPI_ConnectIntlMDLReq().
 *
 *  This is a detail description of this export function. Write
 *  the detail description of this function here.
 *
 * @param remote_BD remote bluetooth device address.
 * @param remote_BD_Type remote bluetooth device address type.
 * @param local_BD_Type
 * @param local_MDEP_ID
 * @param linkConfigType
 * @param pParam pointer to the buffe of connection paramters.
 * @return write the description of the return value of this function here.
 * @retval 0 successfully
 * @retval 1 failed
 */
bool blueAPI_ConnectIntlMDLReq(uint8_t *remote_BD, TBlueAPI_RemoteBDType remote_BD_Type,
    TBlueAPI_LocalBDType local_BD_Type, uint8_t local_MDEP_ID, TBlueAPI_LinkConfigType linkConfigType,
    PBlueAPI_ConnectMDLReqParam pParam)
{
    TBlueAPI_DsMessage msg = {0};
    int parLength;

    msg.Command = blueAPI_EventConnectMDLReq;

    msg.p.ConnectMDLReq.local_MDEP_ID = local_MDEP_ID;
    msg.p.ConnectMDLReq.linkConfigType = linkConfigType;
    msg.p.ConnectMDLReq.remote_BD_Type = remote_BD_Type;
    msg.p.ConnectMDLReq.local_BD_Type = local_BD_Type;

    if (remote_BD == NULL)
    {
        remote_BD = (uint8_t *)zeros;
    }

    switch (linkConfigType)
    {
    case blueAPI_LinkConfigGATT:
      parLength = sizeof(TBlueAPI_ConnectMDLReqGATT);
      break;

    default:
      parLength = 0;
      break;
    }

    /* copy BD and parameter */
    memcpy(msg.p.ConnectMDLReq.remote_BD, remote_BD, BLUE_API_BD_SIZE);
    memcpy(&msg.p.ConnectMDLReq.p, pParam, parLength);

    return blueAPI_SendMessage(NULL, &msg);
}

/**
 * @brief  connect gatt mdl request
 *
 * @param  remote_BD: 
 * @param  local_MDEP_ID
 * @param  scanInterval
 * @param  scanWindow
 * @param  scanTimeout
 * @param  connIntervalMin
 * @param  connIntervalMax
 * @param  connLatency
 * @param  supervisionTimeout
 *
 * @return  bool
 *
 */
bool blueAPI_ConnectGATTMDLReq(uint8_t * remote_BD, TBlueAPI_RemoteBDType remote_BD_Type,
    TBlueAPI_LocalBDType local_BD_Type, uint16_t scanInterval, uint16_t scanWindow,
    uint16_t scanTimeout, uint16_t connIntervalMin, uint16_t connIntervalMax, uint16_t connLatency,
    uint16_t supervisionTimeout, uint16_t CE_Length)
{
    TBlueAPI_ConnectMDLReqParam Param;

    Param.gatt.scanInterval = scanInterval;
    Param.gatt.scanWindow = scanWindow;
    Param.gatt.scanTimeout = scanTimeout;
    Param.gatt.connIntervalMin = connIntervalMin;
    Param.gatt.connIntervalMax = connIntervalMax;
    Param.gatt.connLatency = connLatency;
    Param.gatt.supervisionTimeout = supervisionTimeout;
    Param.gatt.CE_Length = CE_Length;

    return blueAPI_ConnectIntlMDLReq(remote_BD, remote_BD_Type, local_BD_Type, 0,
        blueAPI_LinkConfigGATT, &Param);
}

bool blueAPI_CreateMDLConf(uint16_t local_MDL_ID, uint8_t maxTPDUusCredits,
    TBlueAPI_Cause cause)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventCreateMDLConf;

    msg.p.CreateMDLConf.local_MDL_ID = local_MDL_ID;
    msg.p.CreateMDLConf.maxTPDUusCredits = maxTPDUusCredits;
    msg.p.CreateMDLConf.cause = cause;

    return blueAPI_SendMessage(NULL, &msg);
}

/**
 * @brief  disconnect mdl request
 *
 * @param  local_MDL_ID: 
 * @param  cause
 *
 * @return 
 *
 */
bool blueAPI_DisconnectMDLReq(uint16_t local_MDL_ID, TBlueAPI_Cause cause)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventDisconnectMDLReq;

    msg.p.DisconnectMDLReq.local_MDL_ID = local_MDL_ID;
    msg.p.DisconnectMDLReq.cause = cause;

    return blueAPI_SendMessage(NULL, &msg); 
}

/**
 * @brief  gatt get disconnect indicate confirm
 *
 * @param  pBuffer
 * @param  blueAPIHandle
 * @param  local_MDL_ID
 *
 * @return
 *
 */
bool blueAPI_DisconnectMDLConf(uint16_t local_MDL_ID)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventDisconnectMDLConf;

    msg.p.DisconnectMDLConf.local_MDL_ID = local_MDL_ID;

    return blueAPI_SendMessage(NULL, &msg);
}

/**
 * @brief  gatt service register request
 *
 * @param  nbrOfAttrib
 * @param  pService
 *
 * @return
 *
 */
bool blueAPI_GATTServiceRegisterReq(uint16_t nbrOfAttrib, void * pService)
{
   TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventGATTServiceRegisterReq;

    msg.p.GATTServiceRegisterReq.nbrOfAttrib = nbrOfAttrib;
    msg.p.GATTServiceRegisterReq.pService = pService;

    return blueAPI_SendMessage(NULL, &msg);
}

/**
 * @brief  gatt service release request
 *
 * @param  serviceHandle
 *
 * @return
 *
 */
bool blueAPI_GATTServiceReleaseReq(void * serviceHandle)
{
    UNUSED_PARAMETER(serviceHandle);

    /* not yet implemented */
    return FALSE;
}

/**
 * @brief  gatt attribute update request
 *
 * @param  pBuffer:
 * @param  serviceHandle
 * @param  requestHandle
 * @param  attribIndex
 * @param  attribLength
 * @param  offset
 *
 * @return
 *
 */
bool blueAPI_GATTAttributeUpdateReq(void * pBuffer, void * serviceHandle,
    void * requestHandle, uint16_t attribIndex, uint16_t attribLength, uint16_t offset)
{
    TBlueAPI_DsMessage msg = {0};
    PBlueAPI_DsMessage pMsg;

    if (attribLength > 0)
    {
        /* data is supplied in pBuffer */
        assert(offset >= offsetof(TBlueAPI_DsMessage, p.GATTAttributeUpdateReq.data));

        if ((pBuffer == NULL) || (offset < offsetof(TBlueAPI_DsMessage, p.GATTAttributeUpdateReq.data)||
            attribLength>otp_str_data.gEfuse_UpperStack_s.att_max_mtu_size - 3))
        {
            BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_ERROR,
                "blueAPI_GATTAttributeUpdateReq: invalid parameters");

            return FALSE;
        }

        pMsg = (PBlueAPI_DsMessage)pBuffer;
        pMsg->Length = offset + attribLength;
    }
    else
    {
        pMsg = &msg;
        offset = 0;
        pMsg->Length = sizeof(TBlueAPI_DsMessage);
    }

    pMsg->Command = blueAPI_EventGATTAttributeUpdateReq;

    pMsg->p.GATTAttributeUpdateReq.requestHandle = requestHandle;
    pMsg->p.GATTAttributeUpdateReq.attribIndex = attribIndex;
    pMsg->p.GATTAttributeUpdateReq.serviceHandle = serviceHandle;
    pMsg->p.GATTAttributeUpdateReq.attribLength = attribLength;
    pMsg->p.GATTAttributeUpdateReq.gap = offset;

    return blueAPI_SendMessage(pBuffer, pMsg);
}

/**
 * @brief  gatt attribute update status confirm
 *
 * @param  serviceHandle
 * @param  requestHandle
 * @param  attribIndex
 *
 * @return
 *
 */
bool blueAPI_GATTAttributeUpdateStatusConf(void * serviceHandle,
    void * requestHandle, uint16_t attribIndex)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventGATTAttributeUpdateStatusConf;

    msg.p.GATTAttributeUpdateStatusConf.serviceHandle = serviceHandle;
    msg.p.GATTAttributeUpdateStatusConf.requestHandle = requestHandle;
    msg.p.GATTAttributeUpdateStatusConf.attribIndex = attribIndex;

    return blueAPI_SendMessage(NULL, &msg);
}

/**
 * @brief  gatt attribute read confirm
 *
 * @param  pBuffer
 * @param  local_MDL_ID
 * @param  serviceHandle
 * @param  cause
 * @param  subCause
 * @param  attribIndex
 * @param  attribLength
 * @param  offset
 *
 * @return
 *
 */
bool blueAPI_GATTAttributeReadConf(void * pBuffer, uint16_t local_MDL_ID,
    void * serviceHandle, TBlueAPI_Cause cause, uint16_t subCause,
    uint16_t attribIndex, uint16_t attribLength, uint16_t offset)
{
    TBlueAPI_DsMessage msg = {0};
    PBlueAPI_DsMessage pMsg;

    if (attribLength > 0)
    {
        /* data is supplied in pBuffer */
        assert(offset >= offsetof(TBlueAPI_DsMessage, p.GATTAttributeReadConf.data));

        if ((pBuffer == NULL) || (offset < offsetof(TBlueAPI_DsMessage, p.GATTAttributeReadConf.data)||
            attribLength > otp_str_data.gEfuse_UpperStack_s.att_max_mtu_size - 1))
        {
            BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_ERROR,
                "!!blueAPI_GATTAttributeReadConf: invalid parameters");

            return FALSE;
        }

        pMsg = (PBlueAPI_DsMessage)pBuffer;
        pMsg->Length = offset + attribLength;
    }
    else
    {
        pMsg = &msg;
        offset = 0;
        pMsg->Length = sizeof(TBlueAPI_DsMessage);
    }

    pMsg->Command = blueAPI_EventGATTAttributeReadConf;

    pMsg->p.GATTAttributeReadConf.local_MDL_ID = local_MDL_ID;
    pMsg->p.GATTAttributeReadConf.serviceHandle = serviceHandle;
    pMsg->p.GATTAttributeReadConf.cause = cause;
    pMsg->p.GATTAttributeReadConf.subCause = subCause;
    pMsg->p.GATTAttributeReadConf.attribIndex = attribIndex;
    pMsg->p.GATTAttributeReadConf.attribLength = attribLength;
    pMsg->p.GATTAttributeReadConf.gap = offset;

    return blueAPI_SendMessage(pBuffer, pMsg);
}

/**
 * @brief  gatt attribute write confirm
 *
 * @param  local_MDL_ID
 * @param  serviceHandle
 * @param  cause
 * @param  subCause
 * @param  attribIndex
 *
 * @return
 *
 */
bool blueAPI_GATTAttributeWriteConf(uint16_t local_MDL_ID, void * serviceHandle,
    TBlueAPI_Cause cause, uint16_t subCause, uint16_t attribIndex)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventGATTAttributeWriteConf;

    msg.p.GATTAttributeWriteConf.local_MDL_ID = local_MDL_ID;
    msg.p.GATTAttributeWriteConf.serviceHandle = serviceHandle;
    msg.p.GATTAttributeWriteConf.cause = cause;
    msg.p.GATTAttributeWriteConf.subCause = subCause;
    msg.p.GATTAttributeWriteConf.attribIndex = attribIndex;

    return blueAPI_SendMessage(NULL, &msg);
}

bool blueAPI_GATTAttributePrepareWriteConf(void * pBuffer, uint16_t local_MDL_ID,
    void * serviceHandle, TBlueAPI_Cause cause, uint16_t subCause,
    uint16_t attribIndex, uint16_t attribLength, uint16_t offset)
{
    TBlueAPI_DsMessage msg;
    PBlueAPI_DsMessage pMsg;

    if (attribLength > 0)
    {
        /* data is supplied in pBuffer */
        assert(offset >= offsetof(TBlueAPI_DsMessage, p.GATTAttributePrepareWriteConf.data));

        if ((pBuffer == NULL) || (offset < offsetof(TBlueAPI_DsMessage, p.GATTAttributePrepareWriteConf.data)))
        {
            BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_ERROR,
                "!!blueAPI_GATTAttributePrepareWriteConf: invalid parameters");

            return FALSE;
        }

        pMsg = (PBlueAPI_DsMessage)pBuffer;
        pMsg->Length = offset + attribLength;
    }
    else
    {
        pMsg = &msg;
        offset = 0;
        pMsg->Length = sizeof(TBlueAPI_DsMessage);
    }

    pMsg->Command = blueAPI_EventGATTAttributePrepareWriteConf;

    pMsg->p.GATTAttributePrepareWriteConf.local_MDL_ID = local_MDL_ID;
    pMsg->p.GATTAttributePrepareWriteConf.serviceHandle = serviceHandle;
    pMsg->p.GATTAttributePrepareWriteConf.cause = cause;
    pMsg->p.GATTAttributePrepareWriteConf.subCause = subCause;
    pMsg->p.GATTAttributePrepareWriteConf.attribIndex = attribIndex;
    pMsg->p.GATTAttributePrepareWriteConf.attribLength = attribLength;
    pMsg->p.GATTAttributePrepareWriteConf.gap = offset;

    return blueAPI_SendMessage(pBuffer, pMsg);
}

bool blueAPI_GATTAttributeExecuteWriteConf(uint16_t local_MDL_ID, TBlueAPI_Cause cause,
    uint16_t subCause, uint16_t handle)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventGATTAttributeExecuteWriteConf;

    msg.p.GATTAttributeExecuteWriteConf.local_MDL_ID = local_MDL_ID;
    msg.p.GATTAttributeExecuteWriteConf.cause = cause;
    msg.p.GATTAttributeExecuteWriteConf.subCause = subCause;
    msg.p.GATTAttributeExecuteWriteConf.handle = handle;

    return blueAPI_SendMessage(NULL, &msg);
}

bool blueAPI_GATTAttributePrepareWriteReq(void * pBuffer, uint16_t local_MDL_ID,
    uint16_t attribHandle, uint16_t attribLength, uint16_t writeOffset, uint16_t offset)
{
    PBlueAPI_DsMessage  pMsg;

    /* data is supplied in pBuffer */
    assert(offset >= offsetof(TBlueAPI_DsMessage, p.GATTAttributeWriteReq.data));
    if ((attribLength == 0) || (pBuffer == NULL) ||
        (offset < offsetof(TBlueAPI_DsMessage, p.GATTAttributeWriteReq.data)||
        (attribLength > otp_str_data.gEfuse_UpperStack_s.att_max_mtu_size - 5)))
    {
        BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_ERROR,
            "!!blueAPI_GATTAttributePrepareWriteReq: invalid parameters");

        return FALSE;
    }

    pMsg = (PBlueAPI_DsMessage)pBuffer;

    pMsg->Command = blueAPI_EventGATTAttributePrepareWriteReq;
    pMsg->Length  = offset + attribLength;

    pMsg->p.GATTAttributeWriteReq.local_MDL_ID = local_MDL_ID;
    pMsg->p.GATTAttributeWriteReq.writeType = 3;
    pMsg->p.GATTAttributeWriteReq.attribHandle = attribHandle;
    pMsg->p.GATTAttributeWriteReq.attribLength = attribLength;
    pMsg->p.GATTAttributeWriteReq.writeOffset = writeOffset;
    pMsg->p.GATTAttributeWriteReq.gap = offset;

    return blueAPI_SendMessage(pBuffer, pMsg);
}

bool blueAPI_GATTAttributeExecuteWriteReq(uint16_t local_MDL_ID, uint8_t flags)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventGATTAttributeExecuteWriteReq;

    msg.p.GATTAttributeExecuteWriteReq.local_MDL_ID = local_MDL_ID;
    msg.p.GATTAttributeExecuteWriteReq.flags = flags;

    return blueAPI_SendMessage(NULL, &msg);
}
      
/**
 * @brief  gatt discovery request
 *
 * @param  local_MDL_ID
 * @param  discoveryType
 * @param  startHandle
 * @param  endHandle
 * @param  UUID16
 * @param  pUUID128
 *
 * @return
 *
 */
bool blueAPI_GATTDiscoveryReq(uint16_t local_MDL_ID, TBlueAPI_GATTDiscoveryType discoveryType,
    uint16_t startHandle, uint16_t endHandle, uint16_t UUID16, uint8_t * pUUID128)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventGATTDiscoveryReq;

    if (pUUID128 == NULL)
    {
        pUUID128 = (uint8_t *)zeros;
    }

    msg.p.GATTDiscoveryReq.local_MDL_ID = local_MDL_ID;
    msg.p.GATTDiscoveryReq.discoveryType = discoveryType;
    msg.p.GATTDiscoveryReq.startHandle = startHandle;
    msg.p.GATTDiscoveryReq.endHandle = endHandle;
    msg.p.GATTDiscoveryReq.UUID16 = UUID16;
    memcpy(msg.p.GATTDiscoveryReq.UUID128, pUUID128, sizeof(msg.p.GATTDiscoveryReq.UUID128));

    return blueAPI_SendMessage(NULL, &msg);
}

/**
 * @brief  gatt discovery confirm
 *
 * @param  pBuffer:
 * @param  blueAPIHandle: 
 * @param  local_MDL_ID
 * @param  discoveryType
 * @param  startHandle
 * @param  endHandle
 *
 * @return
 *
 */
bool blueAPI_GATTDiscoveryConf(uint16_t local_MDL_ID, TBlueAPI_GATTDiscoveryType discoveryType,
    uint16_t startHandle, uint16_t endHandle)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventGATTDiscoveryConf;

    msg.p.GATTDiscoveryConf.local_MDL_ID = local_MDL_ID;
    msg.p.GATTDiscoveryConf.discoveryType = discoveryType;
    msg.p.GATTDiscoveryConf.startHandle = startHandle;
    msg.p.GATTDiscoveryConf.endHandle = endHandle;

    return blueAPI_SendMessage(NULL, &msg);
}

/**
 * @brief  gatt attribute read request
 *
 * @param  pBuffer:
 * @param  blueAPIHandle: 
 * @param  local_MDL_ID
 * @param  readType
 * @param  readOffset
 * @param  startHandle
 * @param  endHandle
 * @param  UUID16
 * @param  pUUID128
 *
 * @return
 *
 */
bool blueAPI_GATTAttributeReadReq(uint16_t local_MDL_ID, TBlueAPI_GATTReadType readType,
    uint16_t readOffset, uint16_t startHandle, uint16_t endHandle,
    uint16_t UUID16, uint8_t * pUUID128)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventGATTAttributeReadReq;

    if (pUUID128 == NULL)
    {
        pUUID128 = (uint8_t *)zeros;
    }

    msg.p.GATTAttributeReadReq.local_MDL_ID = local_MDL_ID;
    msg.p.GATTAttributeReadReq.readType = readType;
    msg.p.GATTAttributeReadReq.readOffset = readOffset;
    msg.p.GATTAttributeReadReq.startHandle = startHandle;
    msg.p.GATTAttributeReadReq.endHandle = endHandle;
    msg.p.GATTAttributeReadReq.UUID16 = UUID16;
    memcpy(msg.p.GATTAttributeReadReq.UUID128, pUUID128, sizeof(msg.p.GATTAttributeReadReq.UUID128));

    return blueAPI_SendMessage(NULL, &msg);
}

/**
 * @brief  gatt attribute write request
 *
 * @param  pBuffer:
 * @param  local_MDL_ID
 * @param  writeType
 * @param  attribHandle
 * @param  attribLength
 * @param  writeOffset
 * @param  offset
 *
 * @return
 *
 */
bool blueAPI_GATTAttributeWriteReq(void * pBuffer, uint16_t local_MDL_ID,
    TBlueAPI_GATTWriteType writeType, uint16_t attribHandle, uint16_t attribLength, uint16_t offset)
{
    PBlueAPI_DsMessage pMsg = {0};

    /* data is supplied in pBuffer */
    assert(offset >= offsetof(TBlueAPI_DsMessage, p.GATTAttributeWriteReq.data));

    if ((attribLength == 0) || (pBuffer == NULL) ||
        (offset < offsetof(TBlueAPI_DsMessage, p.GATTAttributeWriteReq.data) ||
        attribLength > otp_str_data.gEfuse_UpperStack_s.att_max_mtu_size - 3))
    {
        BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_ERROR,
            "!!blueAPI_GATTAttributeWriteReq: invalid parameters");

        return FALSE;
    }

    pMsg = (PBlueAPI_DsMessage)pBuffer;

    pMsg->Command = blueAPI_EventGATTAttributeWriteReq;
    pMsg->Length = offset + attribLength;

    pMsg->p.GATTAttributeWriteReq.local_MDL_ID = local_MDL_ID;
    pMsg->p.GATTAttributeWriteReq.writeType = writeType;
    pMsg->p.GATTAttributeWriteReq.attribHandle = attribHandle;
    pMsg->p.GATTAttributeWriteReq.attribLength = attribLength;
    pMsg->p.GATTAttributeWriteReq.writeOffset = 0;
    pMsg->p.GATTAttributeWriteReq.gap = offset;

    return blueAPI_SendMessage(pBuffer, pMsg);
}

/**
 * @brief  gatt attribute confirm
 *
 * @param  local_MDL_ID
 *
 * @return
 *
 */
bool blueAPI_GATTAttributeConf(uint16_t local_MDL_ID)
{
   TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventGATTAttributeConf;

    msg.p.GATTAttributeConf.local_MDL_ID = local_MDL_ID;

    return blueAPI_SendMessage(NULL, &msg);
}

/**
 * @brief  gatt sdp discovery request
 *
 * @param  remote_BD
 * @param  remote_GATT_UUID
 * @param  remote_DID_Discovery
 *
 * @return
 *
 */
bool blueAPI_GATTSDPDiscoveryReq(uint8_t * remote_BD,
    uint16_t remote_GATT_UUID, bool remote_DID_Discovery)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventGATTSDPDiscoveryReq;

    msg.p.GATTSDPDiscoveryReq.remote_GATT_UUID = remote_GATT_UUID;
    msg.p.GATTSDPDiscoveryReq.remote_DID_Discovery = remote_DID_Discovery;

    memcpy(msg.p.GATTSDPDiscoveryReq.remote_BD, remote_BD, BLUE_API_BD_SIZE);

    return blueAPI_SendMessage(NULL, &msg);   
}

/**
 * @brief  gatt sdp discovery confirm
 *
 * @param  serviceHandle
 * @param  cause
 *
 * @return
 *
 */
bool blueAPI_GATTSDPDiscoveryConf(uint32_t serviceHandle, TBlueAPI_Cause cause)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventGATTSDPDiscoveryConf;

    msg.p.GATTSDPDiscoveryConf.serviceHandle = serviceHandle;
    msg.p.GATTSDPDiscoveryConf.cause = cause;

    return blueAPI_SendMessage(NULL, &msg);
}

/**
 * @brief  blueapi gatt request security
 *
 * @param  local_MDL_ID: 
 * @param  requirements
 * @param  minKeySize
 *
 * @return  
 *
 */
bool blueAPI_GATTSecurityReq(uint16_t local_MDL_ID, uint16_t requirements, uint8_t minKeySize)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventGATTSecurityReq;

    msg.p.GATTSecurityReq.local_MDL_ID = local_MDL_ID;
    msg.p.GATTSecurityReq.requirements = requirements;
    msg.p.GATTSecurityReq.minKeySize = minKeySize;

    return blueAPI_SendMessage(NULL, &msg);
}

/**
 * @brief  gatt sever store confirm
 *
 * @param  opCode: 
 * @param  remote_BD
 * @param  remote_BD_Type
 * @param  restartHandle
 * @param  dataLength
 * @param  data
 * @param  cause
 *
 * @return  
 *
 */
bool blueAPI_GATTServerStoreConf(TBlueAPI_GATTStoreOpCode opCode, uint8_t * remote_BD,
    TBlueAPI_RemoteBDType remote_BD_Type, uint16_t restartHandle, uint8_t dataLength,
    uint8_t * data, TBlueAPI_Cause cause)
{
    TBlueAPI_DsMessage msg = {0};

    if(dataLength > 32)
    {
        BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_ERROR,
            "!!blueAPI_GATTServerStoreConf: dataLength is invalid(%d)", dataLength);

        return false;
    }
    
    msg.Command = blueAPI_EventGATTServerStoreConf;

    msg.p.GATTServerStoreConf.opCode = opCode;
    memcpy(msg.p.GATTServerStoreConf.remote_BD, remote_BD, BLUE_API_BD_SIZE);
    msg.p.GATTServerStoreConf.remote_BD_Type = remote_BD_Type;
    msg.p.GATTServerStoreConf.restartHandle = restartHandle;
    msg.p.GATTServerStoreConf.dataLength = dataLength;
    memcpy(msg.p.GATTServerStoreConf.data, data, dataLength);
    msg.p.GATTServerStoreConf.cause = cause;

    return blueAPI_SendMessage(NULL, &msg);
}

/**
 * @brief  inquiry
 *
 * @param  limitedInquiry
 * @param  cancelInquiry
 * @param  timeout
 *
 * @return
 *
 */
bool blueAPI_InquiryReq(bool limitedInquiry, bool cancelInquiry, uint8_t timeout)
{
    TBlueAPI_DsMessage msg = {0};
    msg.Command = blueAPI_EventInquiryReq;

    msg.p.InquiryReq.limitedInquiry = limitedInquiry;
    msg.p.InquiryReq.cancelInquiry = cancelInquiry;
    msg.p.InquiryReq.InquiryTimeout = timeout;

    return blueAPI_SendMessage(NULL, &msg);
}

/**
 * @brief  device config device(cod/name) set request
 *
 * @param  classOfDevice
 * @param  deviceName
 *
 * @return
 *
 */
bool blueAPI_DeviceConfigDeviceSetReq(uint32_t classOfDevice, uint8_t * deviceName)
{
    TBlueAPI_DsMessage msg = {0};
    uint16_t deviceNameLen;

    msg.Command = blueAPI_EventDeviceConfigSetReq;

    msg.p.DeviceConfigSetReq.opCode = blueAPI_DeviceConfigDevice;
    msg.p.DeviceConfigSetReq.p.dev.classOfDevice = classOfDevice;

    deviceNameLen = (uint16_t)strlen((char *)deviceName);
    if (deviceNameLen > BLUE_API_DEVICE_NAME_LENGTH-1)
    {
        deviceNameLen = BLUE_API_DEVICE_NAME_LENGTH-1;
    }

    memcpy(msg.p.DeviceConfigSetReq.p.dev.deviceName, deviceName, deviceNameLen);
    msg.p.DeviceConfigSetReq.p.dev.deviceName[deviceNameLen] = 0x00;

    return blueAPI_SendMessage(NULL, &msg);
}

/**
 * @brief  device config DID set request
 *
 * @param  vendorID
 * @param  vendorIDSource
 * @param  productID
 * @param  productVersion
 *
 * @return
 *
 */
bool blueAPI_DeviceConfigDIDSetReq(uint16_t vendorID, uint16_t vendorIDSource,
    uint16_t productID, uint16_t productVersion)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventDeviceConfigSetReq;

    msg.p.DeviceConfigSetReq.opCode = blueAPI_DeviceConfigDID;
    msg.p.DeviceConfigSetReq.p.did.vendorID = vendorID;
    msg.p.DeviceConfigSetReq.p.did.vendorIDSource = vendorIDSource;
    msg.p.DeviceConfigSetReq.p.did.productID = productID;
    msg.p.DeviceConfigSetReq.p.did.productVersion = productVersion;

    return blueAPI_SendMessage(NULL, &msg);
}

bool blueAPI_DeviceConfigExtraEIRSetReq(uint8_t *pdata)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventDeviceConfigSetReq;

    msg.p.DeviceConfigSetReq.opCode = blueAPI_DeviceConfigExtraEIR;
    msg.p.DeviceConfigSetReq.p.extraEIR.pdata = pdata;

    return blueAPI_SendMessage(NULL, &msg);
}

/**
 * @brief  device config security set request
 *
 * @param  pBuffer
 * @param  blueAPIHandle
 * @param  storeBondMode
 * @param  storeBondSize
 * @param  leFixedDisplayValue
 *
 * @return
 *
 */
bool blueAPI_DeviceConfigSecuritySetReq(uint32_t leFixedDisplayValue)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventDeviceConfigSetReq;

    msg.p.DeviceConfigSetReq.opCode = blueAPI_DeviceConfigSecurity;
    msg.p.DeviceConfigSetReq.p.security.leFixedDisplayValue = leFixedDisplayValue;

    return blueAPI_SendMessage(NULL, &msg);
}

bool blueAPI_DeviceConfigStoreSetReq(TBlueAPI_StoreBondModes storeBondMode, uint8_t storeBondSize)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventDeviceConfigSetReq;

    msg.p.DeviceConfigSetReq.opCode = blueAPI_DeviceConfigStore;
    msg.p.DeviceConfigSetReq.p.store.storeBondMode = storeBondMode;
    msg.p.DeviceConfigSetReq.p.store.storeBondSize = storeBondSize;

    return blueAPI_SendMessage(NULL, &msg);
}

/**
 * @brief  device config pagescan param set request
 *
 * @param  pBuffer
 * @param  blueAPIHandle
 * @param  scanType
 * @param  repMode
 * @param  repInterval
 * @param  repWindow
 * @param  pageTimeout
 *
 * @return
 *
 */
bool blueAPI_DeviceConfigPagescanSetReq(TBlueAPI_BRPageScanType scanType,
    TBlueAPI_BRPageScanRepMode repMode, uint16_t repInterval,
    uint16_t repWindow, uint16_t pageTimeout)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventDeviceConfigSetReq;

    msg.p.DeviceConfigSetReq.opCode = blueAPI_DeviceConfigPagescan;
    msg.p.DeviceConfigSetReq.p.pagescan.scanType = scanType;
    msg.p.DeviceConfigSetReq.p.pagescan.repMode = repMode;
    msg.p.DeviceConfigSetReq.p.pagescan.repInterval = repInterval;
    msg.p.DeviceConfigSetReq.p.pagescan.repWindow = repWindow;
    msg.p.DeviceConfigSetReq.p.pagescan.pageTimeout = pageTimeout;

    return blueAPI_SendMessage(NULL, &msg);
}

/**
 * @brief  device config inquiry scan param set request
 *
 * @param  scanType
 * @param  interval
 * @param  window
 *
 * @return
 *
 */
bool blueAPI_DeviceConfigInquiryscanSetReq(TBlueAPI_BRInquiryScanType scanType,
    uint16_t interval, uint16_t window)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventDeviceConfigSetReq;

    msg.p.DeviceConfigSetReq.opCode = blueAPI_DeviceConfigInquiryscan;
    msg.p.DeviceConfigSetReq.p.inquiryscan.scanType = scanType;
    msg.p.DeviceConfigSetReq.p.inquiryscan.interval = interval;
    msg.p.DeviceConfigSetReq.p.inquiryscan.window = window;

    return blueAPI_SendMessage(NULL, &msg);
}

/**
 * @brief  device config inquiry mode param set request
 *
 * @param  mode
 *
 * @return
 *
 */
bool blueAPI_DeviceConfigInquiryModeReq(TBlueAPI_BRInquiryMode mode)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventDeviceConfigSetReq;

    msg.p.DeviceConfigSetReq.opCode = blueAPI_DeviceConfigInquirymode;
    msg.p.DeviceConfigSetReq.p.inquirymode.mode = mode;

    return blueAPI_SendMessage(NULL, &msg);
}

/**
 * @brief  device config link policy set request
 *
 * @param  linkPolicy
 * @param  deviceRole
 * @param  supervisionTimeout
 *
 * @return
 *
 */
bool blueAPI_DeviceConfigLinkpolicySetReq(TBlueAPI_BRLinkPolicy linkPolicy,
    TBlueAPI_BRDeviceRole deviceRole, uint16_t supervisionTimeout)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventDeviceConfigSetReq;

    msg.p.DeviceConfigSetReq.opCode = blueAPI_DeviceConfigLinkpolicy;
    msg.p.DeviceConfigSetReq.p.linkpolicy.linkPolicy = linkPolicy;
    msg.p.DeviceConfigSetReq.p.linkpolicy.deviceRole = deviceRole;
    msg.p.DeviceConfigSetReq.p.linkpolicy.supervisionTimeout = supervisionTimeout;

    return blueAPI_SendMessage(NULL, &msg);
}

bool blueAPI_DeviceConfigDeviceNameSetReq(uint8_t * deviceName)
{    
    TBlueAPI_DsMessage msg = {0};
    uint16_t deviceNameLen;

    msg.Command = blueAPI_EventDeviceConfigSetReq;

    msg.p.DeviceConfigSetReq.opCode = blueAPI_DeviceConfigDeviceName;

    deviceNameLen = (uint16_t)strlen((char *)deviceName);
    if (deviceNameLen > BLUE_API_DEVICE_NAME_LENGTH - 1)
    {
        deviceNameLen = BLUE_API_DEVICE_NAME_LENGTH - 1;
    }

    memcpy(msg.p.DeviceConfigSetReq.p.device.deviceName, deviceName, deviceNameLen);
    msg.p.DeviceConfigSetReq.p.device.deviceName[deviceNameLen] = 0x00;

    return blueAPI_SendMessage(NULL, &msg);
}

bool blueAPI_DeviceConfigAppearanceSetReq(uint16_t appearance)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventDeviceConfigSetReq;

    msg.p.DeviceConfigSetReq.opCode = blueAPI_DeviceConfigAppearance;

    msg.p.DeviceConfigSetReq.p.appearance.appearance = appearance;

    return blueAPI_SendMessage(NULL, &msg);
}

bool blueAPI_DeviceConfigPerPrefConnParamSetReq(uint16_t connIntervalMin,
    uint16_t connIntervalMax, uint16_t slaveLatency, uint16_t supervisionTimeout)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventDeviceConfigSetReq;

    msg.p.DeviceConfigSetReq.opCode = blueAPI_DeviceConfigPerPrefConnParam;

    msg.p.DeviceConfigSetReq.p.conn.connIntervalMin = connIntervalMin;
    msg.p.DeviceConfigSetReq.p.conn.connIntervalMax = connIntervalMax;
    msg.p.DeviceConfigSetReq.p.conn.slaveLatency = slaveLatency;
    msg.p.DeviceConfigSetReq.p.conn.supervisionTimeout = supervisionTimeout;

    return blueAPI_SendMessage(NULL, &msg);
}                                               

/**
 * @brief  blueapi acl config link config request
 *
 * @param  remote_BD
 * @param  linkPolicy
 * @param  deviceRole
 * @param  supervisionTimeout
 *
 * @return 
 *
 */
bool blueAPI_ACLConfigLinkpolicyReq(uint8_t * remote_BD, TBlueAPI_BRLinkPolicy linkPolicy,
    TBlueAPI_BRDeviceRole deviceRole, uint16_t supervisionTimeout)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventACLConfigReq;

    msg.p.ACLConfigReq.opCode = blueAPI_ACLConfigLinkpolicy;
    memcpy(msg.p.ACLConfigReq.remote_BD, remote_BD, BLUE_API_BD_SIZE);
    msg.p.ACLConfigReq.remote_BD_Type = blueAPI_RemoteBDTypeClassic;
    msg.p.ACLConfigReq.p.linkpolicy.linkPolicy = linkPolicy;
    msg.p.ACLConfigReq.p.linkpolicy.deviceRole = deviceRole;
    msg.p.ACLConfigReq.p.linkpolicy.supervisionTimeout = supervisionTimeout;

    return blueAPI_SendMessage(NULL, &msg);
}

/**
 * @brief  blueapi acl config sniff mode request
 *
 * @param  remote_BD
 * @param  minInterval
 * @param  maxInterval
 * @param  sniffAttempt
 * @param  sniffTimeout
 * @param  maxLatency
 * @param  minRemoteTimeout
 * @param  minLocalTimeout
 *
 * @return 
 *
 */
bool blueAPI_ACLConfigSniffmodeReq(uint8_t * remote_BD, uint16_t minInterval,
    uint16_t maxInterval, uint16_t sniffAttempt, uint16_t sniffTimeout, uint16_t maxLatency,
    uint16_t minRemoteTimeout, uint16_t minLocalTimeout)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventACLConfigReq;

    msg.p.ACLConfigReq.opCode = blueAPI_ACLConfigSniffmode;
    memcpy(msg.p.ACLConfigReq.remote_BD, remote_BD, BLUE_API_BD_SIZE);
    msg.p.ACLConfigReq.remote_BD_Type = blueAPI_RemoteBDTypeClassic;
    msg.p.ACLConfigReq.p.sniffmode.minInterval = minInterval;
    msg.p.ACLConfigReq.p.sniffmode.maxInterval = maxInterval;
    msg.p.ACLConfigReq.p.sniffmode.sniffAttempt = sniffAttempt;
    msg.p.ACLConfigReq.p.sniffmode.sniffTimeout = sniffTimeout;
    msg.p.ACLConfigReq.p.sniffmode.maxLatency = maxLatency;
    msg.p.ACLConfigReq.p.sniffmode.minRemoteTimeout = minRemoteTimeout;
    msg.p.ACLConfigReq.p.sniffmode.minLocalTimeout = minLocalTimeout;

    return blueAPI_SendMessage(NULL, &msg);
}

/**
 * @brief  get device name
 *
 * @param  remote_BD
 *
 * @return
 *
 */
bool blueAPI_DeviceNameReq(uint8_t * remote_BD)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventDeviceNameReq;

    memcpy(msg.p.DeviceNameReq.remote_BD, remote_BD, BLUE_API_BD_SIZE);

    return blueAPI_SendMessage(NULL, &msg);
}

/**
* @brief  DID Device confirm
*
* @param  serviceHandle
* @param  cause
*
* @return 
*
*/
bool blueAPI_DIDDeviceConf(uint32_t serviceHandle, TBlueAPI_Cause cause)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventDIDDeviceConf;

    msg.p.DIDDeviceConf.serviceHandle = serviceHandle;
    msg.p.DIDDeviceConf.cause = cause;

    return blueAPI_SendMessage(NULL, &msg);	
}

/**
 * @brief  set page mode
 * @param  localRadioMode
 * @param  limitedDiscoverable
 *
 * @return
 *
 */
bool blueAPI_RadioModeSetReq(TBlueAPI_RadioMode localRadioMode, bool limitedDiscoverable)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventRadioModeSetReq;

    msg.p.RadioModeSetReq.localRadioMode = localRadioMode;
    msg.p.RadioModeSetReq.limitedDiscoverable = limitedDiscoverable;

    return blueAPI_SendMessage(NULL, &msg);
}

/**
 * @brief  SDP discovery
 *
 * @param  remote_BD: 
 * @param  remote_MDEP_DataType
 * @param  remote_DID_Discovery
 *
 * @return
 *
 */
bool blueAPI_SDPDiscoveryReq(uint8_t * remote_BD,
    TBlueAPI_MDEPDataType remote_MDEP_DataType, bool remote_DID_Discovery)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventSDPDiscoveryReq;

    msg.p.SDPDiscoveryReq.remote_MDEP_DataType = remote_MDEP_DataType;
    msg.p.SDPDiscoveryReq.remote_DID_Discovery = remote_DID_Discovery;

    memcpy(msg.p.SDPDiscoveryReq.remote_BD, remote_BD, BLUE_API_BD_SIZE);

    return blueAPI_SendMessage(NULL, &msg);
}

/**
 * @brief  SDP endpoint confirm
 *
 * @param  serviceHandle: 
 * @param  cause
 *
 * @return
 *
 */
bool blueAPI_SDPEndpointConf(uint32_t serviceHandle, TBlueAPI_Cause cause)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventSDPEndpointConf;

    msg.p.SDPEndpointConf.serviceHandle = serviceHandle;
    msg.p.SDPEndpointConf.cause = cause;

    return blueAPI_SendMessage(NULL, &msg);
}

/**
 * @brief  register to blueapi
 *
 * @param  appHandle
 * @param  MDHmsgHandlerCallback: event callback
 *
 * @return
 *
 */
bool blueAPI_RegisterReq(TBlueAPIAppHandle appHandle, void * MDHmsgHandlerCallback)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventRegisterReq;

    msg.p.RegisterReq.apiVersion = BLUEAPI_API_VERSION;
    msg.p.RegisterReq.MDHmsgHandlerCallback = MDHmsgHandlerCallback;

    return blueAPI_SendMessage(NULL, &msg);
}

/**
 * @brief  release req
 *
 * @param  void
 *
 * @return
 *
 */
bool blueAPI_ReleaseReq(void)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventReleaseReq;

    return blueAPI_SendMessage(NULL, &msg);
}

/**
 * @brief  set pairing mode
 *
 * @param  enablePairableMode
 * @param  BluetoothMode
 * @param  AuthRequirements
 * @param  IOCapabilities
 * @param  remoteOOBDataPresent
 *
 * @return
 *
 */
bool blueAPI_ExtendPairableModeSetReq(bool enablePairableMode,
    TBlueAPI_BluetoothMode BluetoothMode, TBlueAPI_AuthRequirements AuthRequirements,
    TBlueAPI_IOCapabilities IOCapabilities, bool remoteOOBDataPresent)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventPairableModeSetReq;

    msg.p.PairableModeSetReq.enablePairableMode = enablePairableMode;
    msg.p.PairableModeSetReq.BluetoothMode = BluetoothMode;
    msg.p.PairableModeSetReq.AuthRequirements = AuthRequirements;
    msg.p.PairableModeSetReq.IOCapabilities = IOCapabilities;
    msg.p.PairableModeSetReq.remoteOOBDataPresent = remoteOOBDataPresent;
    msg.p.PairableModeSetReq.isSetBluetoothMode = TRUE;  

    return blueAPI_SendMessage(NULL, &msg);
}

bool blueAPI_PairableModeSetReq(bool enablePairableMode, TBlueAPI_AuthRequirements AuthRequirements,
    TBlueAPI_IOCapabilities IOCapabilities, bool remoteOOBDataPresent)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventPairableModeSetReq;

    msg.p.PairableModeSetReq.enablePairableMode = enablePairableMode;
    msg.p.PairableModeSetReq.BluetoothMode = blueAPI_BTMode21Disabled;
    msg.p.PairableModeSetReq.AuthRequirements = AuthRequirements;
    msg.p.PairableModeSetReq.IOCapabilities = IOCapabilities;
    msg.p.PairableModeSetReq.remoteOOBDataPresent = remoteOOBDataPresent;
    msg.p.PairableModeSetReq.isSetBluetoothMode = FALSE;  

    return blueAPI_SendMessage(NULL, &msg);
}

/**
 * @brief  auth request
 *
 * @param  remote_BD
 *
 * @return
 *
 */
bool blueAPI_AuthReq(uint8_t * remote_BD)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventAuthReq;

    memcpy(msg.p.AuthReq.remote_BD, remote_BD, BLUE_API_BD_SIZE);

    return blueAPI_SendMessage(NULL, &msg);
}

/**
 * @brief  user auth request confirm
 *
 * @param  remote_BD
 * @param  AuthCodeLength
 * @param  AuthCode
 * @param  cause
 *
 * @return
 *
 */
bool blueAPI_UserAuthRequestConf(uint8_t * remote_BD, uint8_t AuthCodeLength,
    uint8_t * AuthCode, TBlueAPI_Cause cause)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventUserAuthRequestConf;

    if (remote_BD == NULL)
    {
        remote_BD = (uint8_t *)zeros;
    }

    if (AuthCode == NULL)
    {
        AuthCode = (uint8_t *)zeros;
    }

    if (AuthCodeLength > 16)
    {
        AuthCodeLength = 16;
    }

    memcpy(msg.p.UserAuthRequestConf.remote_BD, remote_BD, BLUE_API_BD_SIZE);
    msg.p.UserAuthRequestConf.AuthCodeLength = AuthCodeLength;
    memcpy(msg.p.UserAuthRequestConf.AuthCode, AuthCode, AuthCodeLength);
    msg.p.UserAuthRequestConf.cause = cause;

    return blueAPI_SendMessage(NULL, &msg);
}

/**
 * @brief  auth result request confirm
 *
 * @param  remote_BD
 * @param  remote_BD_Type
 * @param  linkKeyLength
 * @param  linkKey
 * @param  keyType
 * @param  restartHandle
 * @param  cause
 *
 * @return
 *
 */
bool blueAPI_AuthResultRequestConf(uint8_t * remote_BD, TBlueAPI_RemoteBDType remote_BD_Type,
    uint8_t linkKeyLength, uint8_t * linkKey, TBlueAPI_LinkKeyType keyType,
    uint16_t restartHandle, TBlueAPI_Cause cause)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventAuthResultRequestConf;

    if (remote_BD == NULL)
    {
        remote_BD = (uint8_t *)zeros;
    }

    if (linkKey == NULL)
    {
        linkKey = (uint8_t *)zeros;
    }

    if (linkKeyLength > sizeof(msg.p.AuthResultRequestConf.linkKey))
    {
        linkKeyLength = sizeof(msg.p.AuthResultRequestConf.linkKey);
    }

    memcpy(msg.p.AuthResultRequestConf.remote_BD, remote_BD, BLUE_API_BD_SIZE);
    msg.p.AuthResultRequestConf.remote_BD_Type = remote_BD_Type;
    msg.p.AuthResultRequestConf.linkKeyLength = linkKeyLength;
    memcpy(msg.p.AuthResultRequestConf.linkKey, linkKey, linkKeyLength);
    msg.p.AuthResultRequestConf.keyType = keyType;
    msg.p.AuthResultRequestConf.restartHandle = restartHandle;
    msg.p.AuthResultRequestConf.cause = cause;

    return blueAPI_SendMessage(NULL, &msg);
}

/****************************************************************************
 * blueAPI_UserAuthorizationReqConf()
 ****************************************************************************/
bool blueAPI_UserAuthorizationReqConf(uint8_t *remote_BD, TBlueAPI_Cause cause)
{
    TBlueAPI_DsMessage msg;

    msg.Command = blueAPI_EventUserAuthorizationReqConf;

    if (remote_BD == NULL)
    {
        remote_BD = (uint8_t *)zeros;
    }

    memcpy(msg.p.UserAuthorizationReqConf.remote_BD, remote_BD, BLUE_API_BD_SIZE);
    msg.p.UserAuthorizationReqConf.cause = cause;

    return blueAPI_SendMessage(NULL, &msg);
}

/**
 * @brief  user confirm request confirm 
 *
 * @param  remote_BD
 * @param  cause
 *
 * @return  
 *
 */
bool blueAPI_UserConfirmationReqConf(uint8_t * remote_BD, TBlueAPI_Cause cause)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventUserConfirmationReqConf;

    if (remote_BD == NULL)
    {
        remote_BD = (uint8_t *)zeros;
    }

    memcpy(msg.p.UserConfirmationReqConf.remote_BD, remote_BD, BLUE_API_BD_SIZE);
    msg.p.UserConfirmationReqConf.cause = cause;

    return blueAPI_SendMessage(NULL, &msg);
}

/**
 * @brief  user passkey request confirm 
 *         confirm stack app layer has recved user passkey request
 *
 * @param  remote_BD
 * @param  cause
 *
 * @return  
 *
 */
bool blueAPI_UserPasskeyReqConf(uint8_t * remote_BD, TBlueAPI_Cause cause)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventUserPasskeyReqConf;

    if (remote_BD == NULL)
    {
        remote_BD = (uint8_t *)zeros;
    }

    memcpy(msg.p.UserPasskeyReqConf.remote_BD, remote_BD, BLUE_API_BD_SIZE);
    msg.p.UserPasskeyReqConf.cause = cause;

    return blueAPI_SendMessage(NULL, &msg);
}

/**
 * @brief  user passkey request reply request
 *
 * @param  remote_BD
 * @param  passKey
 * @param  cause
 *
 * @return  
 *
 */
bool blueAPI_UserPasskeyReqReplyReq(uint8_t * remote_BD, uint32_t passKey, TBlueAPI_Cause cause)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventUserPasskeyReqReplyReq;

    memcpy(msg.p.UserPasskeyReqReplyReq.remote_BD, remote_BD, BLUE_API_BD_SIZE);
    msg.p.UserPasskeyReqReplyReq.passKey = passKey;
    msg.p.UserPasskeyReqReplyReq.cause   = cause;

    return blueAPI_SendMessage(NULL, &msg);
}

/**
 * @brief  key press notification request
 *
 * @param  remote_BD
 * @param  eventType
 *
 * @return  
 *
 */
bool blueAPI_KeypressNotificationReq(uint8_t * remote_BD, TBlueAPI_SSPKeyEvent eventType)
{
    TBlueAPI_DsMessage msg = {0};
    msg.Command = blueAPI_EventKeypressNotificationReq;

    memcpy(msg.p.KeypressNotificationReq.remote_BD, remote_BD, BLUE_API_BD_SIZE);
    msg.p.KeypressNotificationReq.eventType = eventType;

    return blueAPI_SendMessage(NULL, &msg);
}

/**
 * @brief  remote oob data reqeust confirm
 *          app layer response remot oob data request
 *
 * @param  remote_BD
 * @param  pC: simple pairing hash C
 * @param  pR: simple pairing Randomizer
 * @param  cause
 *
 * @return  
 *
 */
bool blueAPI_LegacyRemoteOOBDataReqConf(uint8_t * remote_BD,
    uint8_t * pC, uint8_t * pR, TBlueAPI_Cause cause)
{
    TBlueAPI_DsMessage msg = {0};
    msg.Command = blueAPI_EventLegacyRemoteOOBDataReqConf;

    if (remote_BD == NULL)
    {
        remote_BD = (uint8_t *)zeros;
    }

    if (pC == NULL)
    {
        pC = (uint8_t *)zeros;
    }
    
    if (pR == NULL)
    {
        pR = (uint8_t *)zeros;
    }

    memcpy(msg.p.LegacyRemoteOOBDataReqConf.remote_BD, remote_BD, BLUE_API_BD_SIZE);
    memcpy(msg.p.LegacyRemoteOOBDataReqConf.C, pC, 16);
    memcpy(msg.p.LegacyRemoteOOBDataReqConf.R, pR, 16);

    msg.p.LegacyRemoteOOBDataReqConf.cause = cause;

    return blueAPI_SendMessage(NULL, &msg);
}

bool blueAPI_RemoteOOBDataReqConf(uint8_t * remote_BD, uint8_t * pC, TBlueAPI_Cause cause)
{
    TBlueAPI_DsMessage msg = {0};
    msg.Command = blueAPI_EventRemoteOOBDataReqConf;

    if (remote_BD == NULL)
    {
        remote_BD = (uint8_t *)zeros;
    }

    if (pC == NULL)
    {
        pC = (uint8_t *)zeros;
    }

    memcpy(msg.p.RemoteOOBDataReqConf.remote_BD, remote_BD, BLUE_API_BD_SIZE);
    memcpy(msg.p.RemoteOOBDataReqConf.C, pC, 16);

    msg.p.RemoteOOBDataReqConf.cause = cause;

    return blueAPI_SendMessage(NULL, &msg);
}

/**
 * @brief  read local oob data reqeust 
 *
 * @param
 *
 * @return  
 *
 */
bool blueAPI_LocalOOBDataReq(void)
{ 
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventLocalOOBDataReq;

    return blueAPI_SendMessage(NULL, &msg);
}

/**
 * @brief  auth result confirm
 *
 * @param  remote_BD
 * @param  remote_BD_Type
 * @param  AppData
 * @param  cause
 *
 * @return  
 *
 */
bool blueAPI_AuthResultConf(uint8_t * remote_BD, TBlueAPI_RemoteBDType remote_BD_Type, TBlueAPI_Cause cause)
{
    TBlueAPI_DsMessage msg = {0};
    msg.Command = blueAPI_EventAuthResultConf;

    if (remote_BD == NULL)
    {
        remote_BD = (uint8_t *)zeros;
    }

    memcpy(msg.p.AuthResultConf.remote_BD, remote_BD, BLUE_API_BD_SIZE);
    msg.p.AuthResultConf.remote_BD_Type = remote_BD_Type;
    msg.p.AuthResultConf.cause = cause;

    return blueAPI_SendMessage(NULL, &msg);
}

/**
 * @brief  auth delete request
 *
 * @param  remote_BD
 * @param  remote_BD_Type
 *
 * @return  
 *
 */
bool blueAPI_AuthDeleteReq(uint8_t * remote_BD, TBlueAPI_RemoteBDType remote_BD_Type)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventAuthDeleteReq;

    memcpy(msg.p.AuthDeleteReq.remote_BD, remote_BD, BLUE_API_BD_SIZE);
    msg.p.AuthDeleteReq.remote_BD_Type = remote_BD_Type;

    return blueAPI_SendMessage(NULL, &msg);
}

/**
 * @brief  auth list request
 *
 * @param  remote_BD
 * @param  remote_BD_Type
 *
 * @return  
 *
 */
bool blueAPI_AuthListReq(uint8_t * remote_BD, TBlueAPI_RemoteBDType  remote_BD_Type)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventAuthListReq;

    if (remote_BD == NULL)
    {
        remote_BD = (uint8_t *)zeros;
    }

    memcpy(msg.p.AuthListReq.remote_BD, remote_BD, BLUE_API_BD_SIZE);
    msg.p.AuthListReq.remote_BD_Type = remote_BD_Type;

    return blueAPI_SendMessage(NULL, &msg);
}

/**
 * @brief  enable/disable le advertise
 *
 * @param  advMode
 *
 * @return  
 *
 */
bool blueAPI_LEAdvertiseReq(TBlueAPI_LEAdvMode advMode)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventLEAdvertiseReq;

    msg.p.LEAdvertiseReq.advMode = advMode;

    return blueAPI_SendMessage(NULL, &msg);
}

/**
 * @brief  set le advertise parameter
 *
 * @param  advType
 * @param  filterScanReq
 * @param  filterConnectReq
 * @param  minAdvInterval
 * @param  maxAdvInterval
 * @param  local_BD_type
 * @param  remote_BD
 * @param  remote_BD_type
 *
 * @return  
 *
 */
bool blueAPI_LEAdvertiseParameterSetReq(TBlueAPI_LEAdvType advType,
    TBlueAPI_LEFilterPolicy filterScanReq, TBlueAPI_LEFilterPolicy filterConnectReq,
    uint16_t minAdvInterval, uint16_t maxAdvInterval, TBlueAPI_LocalBDType local_BD_type,
    uint8_t * remote_BD, TBlueAPI_RemoteBDType remote_BD_type)
{   
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventLEAdvertiseParameterSetReq;

    if (remote_BD == NULL)
    {
        remote_BD = (uint8_t *)zeros;
    }

    msg.p.LEAdvertiseParameterSetReq.advType = advType;
    msg.p.LEAdvertiseParameterSetReq.filterScanReq = filterScanReq;
    msg.p.LEAdvertiseParameterSetReq.filterConnectReq = filterConnectReq;
    msg.p.LEAdvertiseParameterSetReq.minAdvInterval = minAdvInterval;
    msg.p.LEAdvertiseParameterSetReq.maxAdvInterval = maxAdvInterval;
    msg.p.LEAdvertiseParameterSetReq.remote_BD_type = remote_BD_type;
    msg.p.LEAdvertiseParameterSetReq.local_BD_type = local_BD_type;
    memcpy(msg.p.LEAdvertiseParameterSetReq.remote_BD, remote_BD, BLUE_API_BD_SIZE);

    return blueAPI_SendMessage(NULL, &msg);
}

/**
 * @brief  le adv data set request
 *
 * @param  dataType
 * @param  dataLength
 * @param  data
 *
 * @return  
 *
 */
bool blueAPI_LEAdvertiseDataSetReq(TBlueAPI_LEDataType dataType, uint8_t dataLength, uint8_t * data)
{
    TBlueAPI_DsMessage msg = {0};

    if (dataLength > 31)
    {
        dataLength = 31;
    }

    msg.Command = blueAPI_EventLEAdvertiseDataSetReq;

    msg.p.LEAdvertiseDataSetReq.dataType = dataType;
    msg.p.LEAdvertiseDataSetReq.dataLength = dataLength;
    msg.p.LEAdvertiseDataSetReq.pDataBuffer = NULL;

    osBufferGet(pBlueAPIData->dsBlueAPIPoolID, dataLength, (PVOID *)&msg.p.LEAdvertiseDataSetReq.pDataBuffer);

    memcpy(msg.p.LEAdvertiseDataSetReq.pDataBuffer, data, dataLength);

    return blueAPI_SendMessage(NULL, &msg);
}

/**
 * @brief  le scan request
 *
 * @param  scanMode
 * @param  scanInterval
 * @param  scanWindow
 * @param  filterPolicy
 * @param  filterDuplicates
 *
 * @return  
 *
 */
bool blueAPI_LEScanReq(TBlueAPI_LEScanMode scanMode, uint16_t scanInterval, uint16_t scanWindow,
    TBlueAPI_LEFilterPolicy filterPolicy, TBlueAPI_LocalBDType local_BD_Type, uint8_t filterDuplicates)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventLEScanReq;

    msg.p.LEScanReq.scanMode = scanMode;
    msg.p.LEScanReq.scanInterval = scanInterval;
    msg.p.LEScanReq.scanWindow = scanWindow;
    msg.p.LEScanReq.filterPolicy = filterPolicy;
    msg.p.LEScanReq.local_BD_Type = local_BD_Type;
    msg.p.LEScanReq.filterDuplicates = filterDuplicates;

    return blueAPI_SendMessage(NULL, &msg);
}

/**
 * @brief  modify white list request
 *
 * @param  operation
 * @param  remote_BD
 * @param  remote_BD_Type
 *
 * @return  
 *
 */
bool blueAPI_LEModifyWhitelistReq(TBlueAPI_LEWhitelistOp operation, uint8_t * remote_BD,
    TBlueAPI_RemoteBDType remote_BD_type)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventLEModifyWhitelistReq;

    if (remote_BD == NULL)
    {
        remote_BD = (uint8_t *)zeros;
    }

    msg.p.LEModifyWhitelistReq.operation = operation;
    msg.p.LEModifyWhitelistReq.remote_BD_type = remote_BD_type;
    memcpy(msg.p.LEModifyWhitelistReq.remote_BD, remote_BD, BLUE_API_BD_SIZE);

    return blueAPI_SendMessage(NULL, &msg);
}

/**
 * @brief  le connection update request
 *
 * @param  local_MDL_ID
 * @param  connIntervalMin
 * @param  connIntervalMax
 * @param  connLatency
 * @param  supervisionTimeout
 *
 * @return  
 *
 */
bool blueAPI_LEConnectionUpdateReq(uint16_t local_MDL_ID, uint16_t connIntervalMin,
    uint16_t connIntervalMax, uint16_t connLatency, uint16_t supervisionTimeout)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventLEConnectionUpdateReq;

    msg.p.LEConnectionUpdateReq.local_MDL_ID = local_MDL_ID;
    msg.p.LEConnectionUpdateReq.connIntervalMin = connIntervalMin;
    msg.p.LEConnectionUpdateReq.connIntervalMax = connIntervalMax;
    msg.p.LEConnectionUpdateReq.connLatency = connLatency;
    msg.p.LEConnectionUpdateReq.supervisionTimeout = supervisionTimeout;

    return blueAPI_SendMessage(NULL, &msg);
}

/**
 * @brief  le connection update confirm
 *
 * @param  local_MDL_ID
 * @param  cause
 *
 * @return  
 *
 */
bool blueAPI_LEConnectionUpdateConf(uint16_t local_MDL_ID, TBlueAPI_Cause cause)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventLEConnectionUpdateConf;

    msg.p.LEConnectionUpdateConf.local_MDL_ID = local_MDL_ID;
    msg.p.LEConnectionUpdateConf.cause = cause;

    return blueAPI_SendMessage(NULL, &msg);
}

bool blueAPI_SetRandomAddressReq(uint8_t * random_BD)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventSetRandomAddressReq;

    if (random_BD == NULL)
    {
        return false;
    }

    memcpy(msg.p.SetRandomAddressReq.random_BD, random_BD, BLUE_API_BD_SIZE);

    return blueAPI_SendMessage(NULL, &msg);
}

bool blueAPI_SDPRegister(void* buf, uint8_t offset)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventSDPRegisterReq;

    msg.p.SDPRegisterReq.buf = buf;
    msg.p.SDPRegisterReq.offset = offset;

    return blueAPI_SendMessage(NULL, &msg);
}

bool blueAPI_L2cProtocolRegister(uint16_t psm, uint16_t listenQueue, uint8_t action)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventL2cProtocolRegisterReq;

    msg.p.L2cProtocolRegisterReq.psm = psm;
    msg.p.L2cProtocolRegisterReq.listenQueue = listenQueue;
    msg.p.L2cProtocolRegisterReq.action = action;

    return blueAPI_SendMessage(NULL, &msg);
}

bool blueAPI_L2cConReq(uint16_t psm, uint16_t uuid, uint8_t* remoteBd, uint16_t usQueueID, uint16_t mtuSize)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventL2cConReq;

    msg.p.L2cConReq.psm = psm;
    msg.p.L2cConReq.uuid = uuid;
    msg.p.L2cConReq.usQueueID = usQueueID;
    msg.p.L2cConReq.mtuSize = mtuSize;

    memcpy(msg.p.L2cConReq.remoteBd, remoteBd, BLUE_API_BD_SIZE);

    return blueAPI_SendMessage(NULL, &msg);
}

bool blueAPI_L2cConConf(uint16_t status, uint16_t cid)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventL2cConConf;

    msg.p.L2cConConf.status = status;
    msg.p.L2cConConf.cid = cid;

    return blueAPI_SendMessage(NULL, &msg);
}

bool blueAPI_L2cDataReq(void* buf, uint8_t writeOffset, uint16_t cid, uint16_t length)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventL2cDataReq;

    msg.p.L2cDataReq.buf = buf;
    msg.p.L2cDataReq.writeOffset = writeOffset;
    msg.p.L2cDataReq.cid = cid;
    msg.p.L2cDataReq.length = length;

    return blueAPI_SendMessage(NULL, &msg);
}

bool blueAPI_L2cDiscConf(uint16_t cid)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventL2cDiscConf;

    msg.p.L2cDiscConf.cid = cid;

    return blueAPI_SendMessage(NULL, &msg);
}

bool blueAPI_L2cDiscReq(uint16_t cid)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventL2cDiscReq;

    msg.p.L2cDiscReq.cid = cid;

    return blueAPI_SendMessage(NULL, &msg);
}

bool blueAPI_L2cSecurityRegister(uint8_t active, uint8_t outgoing, uint8_t psm, uint16_t server_channel,
    uint16_t uuid, bool authentication, bool authorize, bool encryption, bool mitm)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventL2cSecurityRegisterReq;

    msg.p.L2cSecurityRegisterReq.active = active;
    msg.p.L2cSecurityRegisterReq.outgoing = outgoing;
    msg.p.L2cSecurityRegisterReq.psm = psm;
    msg.p.L2cSecurityRegisterReq.server_channel = server_channel;
    msg.p.L2cSecurityRegisterReq.uuid = uuid;
    msg.p.L2cSecurityRegisterReq.authentication = authentication;
    msg.p.L2cSecurityRegisterReq.authorize = authorize;
    msg.p.L2cSecurityRegisterReq.encryption = encryption;
    msg.p.L2cSecurityRegisterReq.mitm = mitm;

    return blueAPI_SendMessage(NULL, &msg);
}

bool blueAPI_RFCAuthenticationReq(uint8_t * bd, uint16_t channel, uint16_t dlci,
    uint16_t uuid, uint8_t outgoing, uint8_t active)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventRFCAuthenticationReq;

    msg.p.RFCAuthenticationReq.channel = channel;
    msg.p.RFCAuthenticationReq.dlci = dlci;
    msg.p.RFCAuthenticationReq.uuid = uuid;
    msg.p.RFCAuthenticationReq.outgoing = outgoing;
    msg.p.RFCAuthenticationReq.active = active;

    memcpy(msg.p.RFCAuthenticationReq.bd, bd, BLUE_API_BD_SIZE);

    return blueAPI_SendMessage(NULL, &msg);
}

/**
 * @brief   Count DES sequence length from format describing string
 *    <>  : Sequence w. 8 bit length field
 *    []  : Sequence w. 16 bit length field
 *    {}  : Sequence w. 32 bit length field
 *    U   : UUID - 16, a 16 bit value follows
 *    2U  : same as U
 *    4U: : UUID - 32, a 16 bit value follows
 *    6U  : UUID - 128, a 16 bit value follows, that is expanded to a 128 bit uuid
 *    8U    UUID - 128, a pointer to a 16 byte field follows, that is used as a 128 bit uuid
 *    Y   : UINT - 128 a pointer to a 16 byte field follows,
 *    X   : UINT - 64  use 2 parameters upper 32 bits and lower 32 bits
 *    L   : UINT - 32
 *    I   : UINT - 16
 *    B   : UINT - 8
 *    y   : SINT - 128 a pointer to a 16 byte field follows,
 *    x   : SINT - 64  use 2 parameters upper 32 bits and lower 32 bits
 *    l   : SINT - 32
 *    i   : SINT - 16
 *    b   : SINT - 8
 *    O   : BOOL
 *    S   : String
 *    R   : URL
 *
 * @param format
 *
 * @return  length of des sequence or 0 when format error
 *
 */
uint32_t blueAPI_SDPRecordLength(uint8_t *format, ...)
{
    uint8_t tos = 0;
    uint32_t length = 0;
    uint32_t sl = 0;
    VA_LIST marker;
    uint8_t field_length = 0;
    char ch;
    LPSTR s;
    BOOL more = TRUE;

    VA_START(marker, format);

    while (more)
    {
        ch = *format++;
        switch (ch)
        {
        case 0:
            more = FALSE;
            break;

        case ' ':       /* tolerate formatting characters */
            break;

        case '2':
        case '4':
        case '6':
        case '8':
            /* set fieldLength for next operator */
            field_length = ch - '0';
            break;

        case '<':      /* Push actual adress to stack */
            if (tos >= SDP_CREATE_DES_MAX_NESTING)
            {
                length = 0; /* reset result to nothing */
                more = FALSE;
                break;
            }
            tos++;
            length += 2;  /* Sequence Header Length is 2 Bytes */
            break;

        case '[':      /* Push actual adress to stack */
            if (tos >= SDP_CREATE_DES_MAX_NESTING)
            {
                length = 0; /* reset result to nothing */
                more = FALSE;
                break;
            }
            tos++;
            length += 3;  /* Sequence Header Length is 3 Bytes in this case */
            break;

        case '{':      /* Push actual adress to stack */
            if (tos >= SDP_CREATE_DES_MAX_NESTING)
            {
                length = 0; /* reset result to nothing */
                more = FALSE;
                break;
            }
            tos++;
            length += 5;  /* Sequence Header Length is 5 Bytes in this case */
            break;

        case '>':      /* Pop adress from stack and generate correct header */
        case ']':
        case '}':
            if (tos == 0)
            {
                length = 0; /* reset result to nothing */
                more = FALSE;
                break;
            }
            --tos;
            break;

        case 'U':      /* Generate UUID-16/32/128 from following word */
            switch (field_length)
            {
            case 0:
            case 2:
                /* 16 bit UUID */
                VA_ARG(marker, int);
                length += 3;   /* 1 for header, 2 for UUID */
                break;
            case 4:
                /* 32 bit UUID */
                VA_ARG(marker, int);
                length += 5;   /* 1 for header, 4 for UUID */
                break;
            case 6:
                /* 128 bit UUID, expanded from 32 bit value */
                VA_ARG(marker, int);
                length += 17;  /* 1 for header, 16 for UUID */
                break;
            case 8:
                /* 128 bit UUID, expanded from pointer to 16 byte field */
                VA_ARG(marker, LPSTR);
                length += 17;  /* 1 for header, 16 for UUID */
                break;
            default:
                assert(FALSE);  /* unknown field length */
                break;
            } /* switch */
            field_length = 0;    /* reset field length tp default */
            break;

        case 'Y':       /* Generate 16 bytes UINT from following pointer */
        case 'y':       /* Generate 16 bytes SINT from following pointer */
            VA_ARG(marker, LPSTR);
            length += 17;  /* 1 for header, 8 for UINT-64 */
            break;

        case 'X':      /* Generate UINT-64 from following 2 dwords */
        case 'x':      /* Generate SINT-64 from following 2 dwords */
            VA_ARG(marker, LONG);
            VA_ARG(marker, LONG);
            length += 9;  /* 1 for header, 8 for UINT-64 */
            break;

        case 'L':      /* Generate UINT-32 from following dword */
        case 'l':      /* Generate SINT-32 from following dword */
            VA_ARG(marker, LONG);
            length += 5;   /* 1 for header, 4 for UINT-32 */
            break;

        case 'I':      /* Generate UINT-16 from following word */
        case 'i':      /* Generate SINT-16 from following word */
            VA_ARG(marker, int);
            length += 3;   /* 1 for header, 2 for UINT-16 */
            break;

        case 'B':      /* Generate UINT-8 from following word */
        case 'b':      /* Generate UINT-8 from following word */
            VA_ARG(marker, int);
            length += 2;   /* 1 for header, 1 for UINT-8 */
            break;

        case 'O':      /* Generate BOOL */
            VA_ARG(marker, int);
            length += 2;   /* 1 for header, 1 for BOOL */
            break;

        case 'S':      /* Insert a string */
        case 'R':      /* Insert a URL */
            s = VA_ARG(marker, LPSTR);
            sl = (uint32_t)(strlen(s)+1); /* including trailing zero */
            if (sl < 0x100)
            {
                length += 2+sl;    /* 2 for header, sl for string */
            }
            else if (sl < 0x10000)
            {
                length += 3+sl;    /* 3 for header, sl for string */
            }
            else
            {
                length += 5+sl;    /* 5 for header, sl for string */
            }
            break;

        default:
            length = 0; /* reset result to nothing */
            more = FALSE;
            break;
        } /* switch */
    } /* while */
    VA_END(marker);

    return length;
}

#if (F_BT_SCO)
/**
 * @brief  SCO/eSCO connection request
 *
 * @param  remote_BD
 * @param  txBandwidth
 * @param  rxBandwidth
 * @param  maxLatency
 * @param  voiceSetting
 * @param  retransEffort
 * @param  packetType
 *
 * @return
 *
 */
bool blueAPI_SCOConReq(uint8_t *remote_BD, uint32_t txBandwidth, uint32_t rxBandwidth,
    uint16_t maxLatency, uint16_t voiceSetting, uint8_t  retransEffort, uint16_t packetType)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventSCOConReq;

    msg.p.SCOConReq.txBandwidth = txBandwidth;
    msg.p.SCOConReq.rxBandwidth = rxBandwidth;
    msg.p.SCOConReq.maxLatency = maxLatency;
    msg.p.SCOConReq.voiceSetting = voiceSetting;
    msg.p.SCOConReq.retransEffort = retransEffort;
    msg.p.SCOConReq.packetType = packetType;

    memcpy(msg.p.SCOConReq.remote_BD, remote_BD, BLUE_API_BD_SIZE);

    return blueAPI_SendMessage(NULL, &msg);
}

/**
 * @brief  SCO/eSCO connection confirm
 *
 * @param  remote_BD
 * @param  txBandwidth
 * @param  rxBandwidth
 * @param  maxLatency
 * @param  voiceSetting
 * @param  retransEffort
 * @param  packetType
 * @param  cause
 *
 * @return
 *
 */
bool blueAPI_SCOConConf(uint8_t *remote_BD, uint32_t txBandwidth, uint32_t rxBandwidth,
    uint16_t maxLatency, uint16_t voiceSetting, uint8_t  retransEffort,
    uint16_t packetType, TBlueAPI_Cause cause)
{

    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventSCOConConf;

    msg.p.SCOConConf.txBandwidth = txBandwidth;
    msg.p.SCOConConf.rxBandwidth = rxBandwidth;
    msg.p.SCOConConf.maxLatency = maxLatency;
    msg.p.SCOConConf.voiceSetting = voiceSetting;
    msg.p.SCOConConf.retransEffort = retransEffort;
    msg.p.SCOConConf.packetType = packetType;
    msg.p.SCOConConf.cause = cause;

    memcpy(msg.p.SCOConConf.remote_BD, remote_BD, BLUE_API_BD_SIZE);

    return blueAPI_SendMessage(NULL, &msg);
}

/**
 * @brief  SCO/eSCO disconnect request
 *
 * @param  remote_BD
 *
 * @return
 *
 */
bool blueAPI_SCODiscReq(uint8_t * remote_BD)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventSCODiscReq;

    memcpy(msg.p.SCODiscReq.remote_BD, remote_BD, BLUE_API_BD_SIZE);

    return blueAPI_SendMessage(NULL, &msg);
}
#endif

#if F_BT_LE_BT41_SUPPORT
bool blueAPI_CreateLEDataChannelReq(uint16_t local_MDL_ID, uint16_t le_psm, uint16_t mtu,
    uint16_t mps, uint16_t initialCredits, uint16_t creditsIncrease)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventCreateLEDataChannelReq;

    msg.p.CreateLEDataChannelReq.local_MDL_ID = local_MDL_ID;
    msg.p.CreateLEDataChannelReq.le_psm = le_psm;
    msg.p.CreateLEDataChannelReq.mtu = mtu;
    msg.p.CreateLEDataChannelReq.mps = mps;
    msg.p.CreateLEDataChannelReq.initialCredits = initialCredits;
    msg.p.CreateLEDataChannelReq.creditsIncrease = creditsIncrease;

    return blueAPI_SendMessage(NULL, &msg);
}

bool blueAPI_CreateLEDataChannelConf(uint16_t local_MDL_ID, uint16_t channel, uint16_t mtu,
    uint16_t mps, uint16_t initialCredits,  uint16_t creditsIncrease, TBlueAPI_Cause cause)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventCreateLEDataChannelConf;

    msg.p.CreateLEDataChannelConf.local_MDL_ID = local_MDL_ID;
    msg.p.CreateLEDataChannelConf.channel = channel;
    msg.p.CreateLEDataChannelConf.mtu = mtu;
    msg.p.CreateLEDataChannelConf.mps = mps;
    msg.p.CreateLEDataChannelConf.initialCredits = initialCredits;
    msg.p.CreateLEDataChannelConf.creditsIncrease = creditsIncrease;
    msg.p.CreateLEDataChannelConf.cause = cause;

    return blueAPI_SendMessage(NULL, &msg);
}

bool blueAPI_DisconnectLEDataChannelReq(uint16_t local_MDL_ID, uint16_t channel)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventDisconnectLEDataChannelReq;

    msg.p.DisconnectLEDataChannelReq.local_MDL_ID = local_MDL_ID;
    msg.p.DisconnectLEDataChannelReq.channel = channel;

    return blueAPI_SendMessage(NULL, &msg);
}

bool blueAPI_DisconnectLEDataChannelConf(uint16_t local_MDL_ID, uint16_t channel, TBlueAPI_Cause cause)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventDisconnectLEDataChannelConf;

    msg.p.DisconnectLEDataChannelConf.local_MDL_ID = local_MDL_ID;
    msg.p.DisconnectLEDataChannelConf.cause = cause;
    msg.p.DisconnectLEDataChannelConf.channel = channel;

    return blueAPI_SendMessage(NULL, &msg);
}

bool blueAPI_SendLEFlowControlCreditReq(uint16_t local_MDL_ID, uint16_t channel, uint16_t credits)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventSendLEFlowControlCreditReq;

    msg.p.SendLEFlowControlCreditReq.local_MDL_ID = local_MDL_ID;
    msg.p.SendLEFlowControlCreditReq.channel = channel;
    msg.p.SendLEFlowControlCreditReq.credits = credits;

    return blueAPI_SendMessage(NULL, &msg);
}

bool blueAPI_LEDataReq(void * pBuffer, uint16_t local_MDL_ID, uint16_t channel,
    uint16_t valueLength, uint16_t offset)
{
    PBlueAPI_DsMessage pMsg;

    assert(offset >= offsetof(TBlueAPI_DsMessage, p.LEDataReq.data));

    if ((valueLength == 0) || (pBuffer == NULL) ||
        (offset < offsetof(TBlueAPI_DsMessage, p.LEDataReq.data)))
    {
        BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_ERROR, "!!blueAPI_LEDataReq: invalid parameters");
        return FALSE;
    }

    pMsg = (PBlueAPI_DsMessage)pBuffer;

    if (((uint32_t)pMsg) & 0x01)
    {
        assert(offset > offsetof(TBlueAPI_DsMessage, p.LEDataReq.data));

        pMsg = (PBlueAPI_DsMessage)(((uint8_t *)pMsg) + 1);
        offset--;
    }

    pMsg->Command = blueAPI_EventLEDataReq;
    pMsg->Length  = offset + valueLength;

    pMsg->p.LEDataReq.local_MDL_ID = local_MDL_ID;
    pMsg->p.LEDataReq.channel = channel;
    pMsg->p.LEDataReq.valueLength = valueLength;
    pMsg->p.LEDataReq.gap = offset;

    return blueAPI_SendMessage(pBuffer, pMsg);

}

bool blueAPI_LEDataConf(uint16_t local_MDL_ID, uint16_t channel, TBlueAPI_Cause cause)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventLEDataConf;

    msg.p.LEDataConf.local_MDL_ID = local_MDL_ID;
    msg.p.LEDataConf.channel = channel;
    msg.p.LEDataConf.cause = cause;

    return blueAPI_SendMessage(NULL, &msg);
}

bool blueAPI_LEPsmSecuritySetReq(uint16_t le_psm, bool active,
    TBlueAPI_LESecurityMode secMode, uint8_t keySize)
{
    TBlueAPI_DsMessage msg = {0};

    msg.Command = blueAPI_EventLEPsmSecuritySetReq;

    if (le_psm > 0xff)
    {
        return FALSE;
    }

    msg.p.LEPsmSecuritySetReq.le_psm = le_psm;
    msg.p.LEPsmSecuritySetReq.active = active;
    msg.p.LEPsmSecuritySetReq.secMode = secMode;
    msg.p.LEPsmSecuritySetReq.keySize = keySize;

    return blueAPI_SendMessage(NULL, &msg);
}
#endif

bool blueAPI_DataRamPoolInit(uint8_t * pBufferDataOff, uint16_t size)
{
#if 0
    if(pPatch_upper_stack_blueAPI_AppPoolInit)
    {
        return pPatch_upper_stack_blueAPI_AppPoolInit(pBufferDataOff, size);
    }
#endif
    return FALSE;
}

uint16_t blueAPI_DataRamPoolCreate(uint16_t poolElementSize, uint16_t poolElementCount)
{
    uint8_t retVal = 0;
    
    if (osPoolCreate(&retVal, RAM_TYPE_DATA_OFF, TRUE, poolElementSize, poolElementCount, 4))
    {
        return 0xFFFF; /* error */
    }
    else
    {
        return retVal; /* OK */
    }
}

/****************************************************************************/
/* uint16_t blueAPI_DataRamPoolExtend                                                  */
/* (                                                                        */
/*    uint16_t       poolID                                                     */
/*    uint16_t       poolElementSize                                            */
/*    uint16_t       poolElementCount                                           */
/* )                                                                        */
/****************************************************************************/
bool blueAPI_DataRamPoolExtend(uint16_t poolID, uint16_t poolElementSize, uint16_t poolElementCount)
{
    if (osPoolExtend(poolID, poolElementSize, poolElementCount))
    {
        return TRUE; /* error */
    }
    else
    {
        return FALSE; /* OK    */
    }
}

/****************************************************************************/
/* uint8_t * blueAPI_DataRamPoolBufferGet                                               */
/* (                                                                        */
/*    uint16_t       AppPoolID                                                  */
/*    uint16_t       len                                                        */
/* )                                                                        */
/****************************************************************************/
void * blueAPI_DataRamPoolBufferGet(uint16_t AppPoolID, uint16_t len)
{
    uint8_t * lpBuffer = NULL;
#if 0    
    if(pPatch_upper_stack_blueAPI_AppBufferAlloc)
    {
        return pPatch_upper_stack_blueAPI_AppBufferAlloc(AppPoolID, len);
    }
#endif
    if (osBufferGet(AppPoolID, len, (PVOID  *)&lpBuffer))
    {
        BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE,
                               "!!blueAPI: no buffer len[%d] in blueAPI_DataRamPoolBufferGet",
                               len
                              );
    }

    //DBG_DIRECT("buffer alloc id=%x \n",lpBuffer);
    return lpBuffer;
}

/****************************************************************************/
/* void blueAPI_DataRamPoolBufferRelease                                               */
/* (                                                                        */
/*   uint8_t *     pBuffer                                                     */
/* )                                                                        */
/****************************************************************************/
void blueAPI_DataRamPoolBufferRelease(uint16_t AppPoolID, void * pBuffer)
{
#if 0
    if(pPatch_upper_stack_blueAPI_AppBufferRelease)
    {
        return pPatch_upper_stack_blueAPI_AppBufferRelease(pBuffer);
    }
#endif
    osBufferRelease(pBuffer);
}

