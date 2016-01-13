/**
********************************************************************************************************
Copyright (c) 2015,  Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       btsend.c
* @brief     bt control application
* @details   
*
* @author   	gordon
* @date      	2015-07-02
* @version	v0.1
*/

#include <flags.h>
#include <os_message.h>
#include <os_pool.h>
#include <os_queue.h>
#include <blueface.h>

#include <btcommon.h>
#include <btman.h>
#include <btsend.h>
#include <btglib.h>
#include <gatt_api.h>
#include <trace_binary.h>
#include <l2c_api.h>
#include <blueapi_osif.h>
#include <blueapi_def.h>
#include <blueapi_api.h>
#include <bt_api.h>
#include <sdp_code.h>
#include <btsm_api.h>
#include <hci_api.h>
#include <btman.h>
#include <blueapi_osif.h>
#include <blueapi_def.h>
#include <upper_stack_global.h>

#define TRACE_MODULE_ID     MID_BLUEFACE

/**************************************************************************/
/* Message Generators Downstream (BT STACK)                               */
/**************************************************************************/
/**
* @brief  common prepare l2cap confirm param
*
* @param  pool:
* @param  mtuSize: 
* @param  maxPDUSize
* @param  reqId
*
* @return  
*
*/
LP_L2C_Conf_para commonPrepareL2cConfPara(uint16_t mtuSize, uint16_t maxPDUSize, uint16_t reqId)
{
	LP_L2C_Conf_para pConfPara;

	if (osBufferGet(BTSystemPoolID, sizeof(T_L2C_Conf_para), (PVOID)&pConfPara))
	{
	    RFCOMM_TRACE_PRINTF_1(RFCOMM_TRACE_MASK_ERROR,"!!RFC: commonPrepareL2cConfPara out of memory size %d", sizeof(T_L2C_Conf_para));
	    return (LP_L2C_Conf_para)0;
	}

	memset((PVOID)pConfPara, 0, sizeof(T_L2C_Conf_para));

	pConfPara->reqId 					= reqId;
	pConfPara->mtuSize                  = mtuSize;
	pConfPara->validMask.FlowControl    = 1;
	pConfPara->flowControl.mode         = L2C_MODE_BASIC;
	pConfPara->flowControl.maxPDUSize   = maxPDUSize;

	return pConfPara;
}

/**
* @brief  send bt connect confirm
*
* @param  bLinkContext
* @param  bLinkHandle
* @param  cause
*
* @return  
*
*/
void blueFaceSendBT_CON_CONF(BLINKCONTEXT bLinkContext, BLINKHANDLE bLinkHandle, uint16_t cause)
{
    PBlueAPI_LinkDescriptor pLinkContext = (PBlueAPI_LinkDescriptor)(bLinkContext);

    BLUEAPI_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_TRACE, "blueAPI: BT_CON_CONF link[0x%X] cause(%d)",
        pLinkContext, cause);

    if (!cause)
    {  
        pLinkContext->handle = bLinkHandle;
    }
    else
    {
        /* fake a disconnect to handle this...  */
        TBlueAPI__BT__LinkCloseInd BT__LinkCloseInd;

        BT__LinkCloseInd.cause        = blueAPI_CauseConnectionLost;
        BT__LinkCloseInd.pLinkContext = pLinkContext;

        blueAPI_Handle_BT_Event(pBlueAPIData, blueAPI_Event_BT__LinkCloseInd,
            (PBlueAPI__BT_Event)&BT__LinkCloseInd);
    }
}

/**
* @brief  blueface send bt connect indicate
*
* @param  link
*
* @return  
*/
void blueFaceSendBT_CON_IND(LPbtLink link)
{
    BLINKHANDLE bLinkHandle = (BLINKHANDLE)link;
    uint16_t psm = link->psm;
    uint8_t * bd = (uint8_t *)link->bd;
    BOOL accept = FALSE;
    PBlueAPI_LinkDescriptor pLinkContext = NULL;
    PBlueAPI_MCL pMCL = NULL;
    PBlueAPI_MDL pMDL = NULL;

    if (psm == BLUEFACE_PSM_GATT)
    {
        TBlueAPI_RemoteBDType bdType;
        TBlueAPI_LinkConfigType linkConfigType;

        bdType = (TBlueAPI_RemoteBDType)link->connIndParams.gatt.bdType;
        linkConfigType = blueAPI_LinkConfigGATT;

        BLUEAPI_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_TRACE,
            "blueAPI: BT_CON_IND BD[%s] psm[0x%4.4X]",
            BLUEAPI_TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE, bd),psm);

        pMCL = blueAPI_MCLFindByBD(bd, blueAPI_LinkConfigType2MCLType(linkConfigType));

        if (!pMCL)
        {
            /* remote PSMs are not known */
            pMCL = blueAPI_MCLAllocate(bd, bdType, blueAPI_LinkConfigType2MCLType(linkConfigType));
            if (pMCL)
            {
                /* fake ctrl connected message to app */
                blueAPI_MCLSetState(pMCL, blueAPI_DS_ControlConnected);
            }
            else
            {
                BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE, "blueAPI: BT_CON_IND no free MCL");
            }
        }
        else if (blueAPI_MCLInTransition(pMCL))
        {
            pMCL = NULL;
            BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE, "blueAPI: BT_CON_IND while MCL busy");
        }

        if (pMCL)
        {
            /* => pMDEP != NULL */
            pMDL = blueAPI_MDLAllocate(pMCL, linkConfigType);

            if (pMDL)
            {  
                pLinkContext = blueAPI_ChannelAllocate(pMCL, blueAPI_SubRoleAcceptor,
                    pMDL->linkConfigType);

                if (!pLinkContext)
                {
                    BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE,
                        "blueAPI: BT_CON_IND no free linkContext");

                    blueAPI_MDLRelease(pMDL);
                }
                else
                { 
                    pLinkContext->pMDL = pMDL;
                    pLinkContext->handle = bLinkHandle;
                    pLinkContext->linkState = blueAPI_Connecting;

                    blueAPI_Send_CreateMDLInd(pMCL, pMDL);

                    /* not a real accept, just don't reject now */
                    accept = TRUE;
                }
            }
            else
            {
                BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE, "blueAPI: BT_CON_IND no free MDL");
            }
        }

        if (!accept)
        {
            btApiBT_CON_RESP(bLinkHandle, NULL, BLUEFACE_CON_REJECT, 0, NULL);
        }
    }
    else /* unknown PSM */
    {
        BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE, "!!blueAPI: BT_CON_IND --> reject ");

        btApiBT_CON_RESP(bLinkHandle, NULL, BLUEFACE_CON_REJECT, 0, NULL);
    }
} /* blueFaceSendBT_CON_IND */

/**
 * @brief  send common connection active indicate
 *
 * @param  bLinkContext:
 * @param  frameSize:
 * @param  poolId
 * @param  param
 *
 * @return
 */
void  blueFaceSendBT_CON_ACT_IND(BLINKCONTEXT bLinkContext, uint16_t frameSize, uint16_t poolId, uint16_t *param)
{
    PBlueAPI_LinkDescriptor pLinkContext = (PBlueAPI_LinkDescriptor)(bLinkContext);
    PBlueAPI_MCL pMCL = pLinkContext->pMCL;
    PBlueAPI_MDL pMDL = pLinkContext->pMDL;
    TBlueAPI__BT__LinkOpenInd BT__LinkOpenInd;

    BT__LinkOpenInd.pLinkContext = pLinkContext;
    BT__LinkOpenInd.cause        = blueAPI_CauseSuccess;

    if (pLinkContext && pMCL)
    {
        pLinkContext->dsPoolID  = poolId;
        pLinkContext->linkState = blueAPI_Connected;

        if (pMCL->mclType == blueAPI_MCLType_GATT)
        {
            TCON_ACT_IND_GATT* gatt = (TCON_ACT_IND_GATT *)param;
            BLUEAPI_TRACE_PRINTF_5(BLUEAPI_TRACE_MASK_TRACE,
                                   "blueAPI: BT_CON_ACT_IND link[0x%X] dsPool(0x%X) conInt(%d) conLat(%d) svto(%d)",
                                   pLinkContext,
                                   pLinkContext->dsPoolID,
                                   gatt->connInterval,
                                   gatt->connLatency,
                                   gatt->supervisionTimeout
                                   );

            pMDL->maxTPDUSize = frameSize; /* = MTU size */
            pMDL->dsCredits   = (uint8_t)gatt->credits;

            /* in case of connection via whitelist remoteBd is not known before: */
            memcpy(pMDL->remoteBd, gatt->bd, sizeof(TBdAddr));
            memcpy(pMCL->bd, gatt->bd, sizeof(TBdAddr));
            pMDL->remoteBdType = (TBlueAPI_RemoteBDType)gatt->bdType;
            pMCL->bdType = pLinkContext->pMDL->remoteBdType;

            if (pMCL->bdType != blueAPI_RemoteBDTypeClassic)
            {
                blueAPI_TimerStop(pMCL, blueAPI_TimerID_LEScanTO);

                pLinkContext->LEConnectionInterval = gatt->connInterval;
                pLinkContext->LEConnectionLatency  = gatt->connLatency;
                pLinkContext->LESupervisionTimeout = gatt->supervisionTimeout;
            }
        }

        blueAPI_Handle_BT_Event(pBlueAPIData, blueAPI_Event_BT__LinkOpenInd, (PBlueAPI__BT_Event)&BT__LinkOpenInd);
    }
    else
    {
        BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE, "!!blueAPI: BT_CON_ACT_IND unknown context");
    }
}

/**
* @brief  send disconnect indicate to app
*
* @param  app:
* @param  bLinkContext
* @param  cause: disconnect reason
*
* @return
*
*/
void blueFaceSendBT_DISC_IND(BLINKCONTEXT bLinkContext, uint16_t cause)
{
    PBlueAPI_LinkDescriptor    pLinkContext = (PBlueAPI_LinkDescriptor)(bLinkContext);
    TBlueAPI__BT__LinkCloseInd BT__LinkCloseInd;

    BLUEAPI_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_TRACE, "blueAPI: BT_DISC_IND link[0x%X] cause(0x%X)",
        pLinkContext, cause);

    switch (pLinkContext->linkState)
    {  
    case blueAPI_Connecting:      /* our connect failed */
        break;

    case blueAPI_Connected:       /* connection died */
        pLinkContext->linkState = blueAPI_Disconnecting;
        break;

    case blueAPI_Disconnecting:   /* both sides disconnected */
        break;

    default:
#if SEND_INT_EVENT_INFO
        blueAPI_Send_InternalEventInfo(blueAPI_InternalEventInvalidRemoteEvent,
                                     BLUE_API_GENERATE_EVENT_ID,
                                     blueAPI_CauseInvalidState
                                     );
#endif
        break;
    }

    switch(cause)
    {  
    case 0: 
        BT__LinkCloseInd.cause = blueAPI_CauseSuccess;
        break;

    case HCI_ERR|HCI_ERR_PAGE_TIMEOUT: 
    case HCI_ERR|HCI_ERR_CONNECTION_TIMEOUT: 
    case HCI_ERR|HCI_ERR_HOST_TIMEOUT:
    case HCI_ERR|HCI_ERR_NOCONNECTION: 
    case HCI_ERR|HCI_ERR_CONNECTION_FAILED_TO_BE_ESTABLISHED: 
    case RFCOMM_ERR|RFCOMM_ERR_TIMEOUT: 
        BT__LinkCloseInd.cause = blueAPI_CauseConnectionLost;
        break;

    case L2CAP_ERR|L2CAP_ERR_REFUS_INV_PSM: 
    case L2CAP_ERR|L2CAP_ERR_REFUS_NO_RESOURCE: 
    case L2CAP_ERR|L2CAP_ERR_TIMEOUT_EXTERNAL: 
    case RFCOMM_ERR|RFCOMM_ERR_REJECTED: 
        BT__LinkCloseInd.cause = blueAPI_CauseConnectionDisconnect;
        break;

    case L2CAP_ERR|L2CAP_ERR_REFUS_SEC_BLOCK: 
    case HCI_ERR|HCI_ERR_AUTHENTICATION_FAILED:
        BT__LinkCloseInd.cause = blueAPI_CauseAuthenticationFailed;
        break;

    case HCI_ERR|HCI_ERR_COMMAND_DISALLOWED: 
        BT__LinkCloseInd.cause = blueAPI_CauseInvalidState;
        break;

    case SECMAN_ERR|SECMAN_ERR_LE_BD_NOT_RESOLVED: 
        BT__LinkCloseInd.cause = blueAPI_CauseAddressNotResolved;
        break;

    default: 
        BT__LinkCloseInd.cause = blueAPI_CauseUnspecified;
        break;
    }

    BT__LinkCloseInd.pLinkContext = pLinkContext;

    blueAPI_Handle_BT_Event(pBlueAPIData, blueAPI_Event_BT__LinkCloseInd, (PBlueAPI__BT_Event)&BT__LinkCloseInd);
} /* blueFaceSendBT_DISC_IND */

/**
* @brief  blueface send sdp search confirm
*
* @param  bLinkContext
* @param  handles
* @param  count
* @param  offset
*
* @return  
*
*/
void blueFaceSendBT_SDP_SEARCH_CONF(BLINKCONTEXT bLinkContext, uint32_t* handles, uint16_t count, uint16_t offset)
{
    uint8_t rb[DATACB_INIT_OFFSET];
    LPblueFaceMsg pmsg;
    uint16_t headerSize;

    BLUEAPI_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI_Handle_BT_SDP_SEARCH_CONF: uuid=0x%x totalHandles=%d",
        pBlueAPIData->SM_SDP_ServiceUUID, count);

    if (handles == NULL)
    {
        offset  = DATACB_INIT_OFFSET;
        handles = (uint32_t*) (rb + offset);
    } /* no data pointer */

    headerSize = (uint16_t)offsetof(TblueFaceMsg, p.sdpSearchConfirmation.handles);
    assert(offset >= headerSize);

    pmsg            = (LPblueFaceMsg)((uint8_t *)handles - offset);
    pmsg->length    = (uint16_t)(offset + count*sizeof(uint32_t));

    pmsg->p.sdpSearchConfirmation.bLinkContext = bLinkContext;
    pmsg->p.sdpSearchConfirmation.totalHandles = count;
    pmsg->p.sdpSearchConfirmation.gap          = (uint8_t) (offset - headerSize);

    pBlueAPIData->pSearchConf         = &pmsg->p.sdpSearchConfirmation;
    pBlueAPIData->serviceHandleIndex  = 0;

    blueAPI_SDPGetNextService(pBlueAPIData);
}

/**
 * @brief  blueface sdp attribute confirm
 * 
 * @param  app
 * @param  bLinkContext
 * @param  avList
 * @param  length
 * @param  offset
 * @param  tc
 *
 * @return  
 *
 */
void blueFaceSendBT_SDP_ATTRIBUTE_CONF(BLINKCONTEXT bLinkContext,
    uint8_t *avList, uint16_t length, uint16_t offset, TsdpChanDesc *tc)
{
    LPblueFaceMsg pmsg;  
    uint32_t *pHandles = (uint32_t*)((uint8_t *)pBlueAPIData->pSearchConf->handles +
        pBlueAPIData->pSearchConf->gap);
    uint32_t serviceHandle = pHandles[pBlueAPIData->serviceHandleIndex];
    BOOL getNextService = FALSE;

    uint8_t rb[DATACB_INIT_OFFSET];
    uint16_t headerSize;
    /* Send a SDP_SEARCH_CONF upstream to the application, use the incoming message, add
        blueFace header in front and sent to application
        */

    if (avList == NULL)
    {
        /* no data pointer given, allocate memory from the pool */
        offset = DATACB_INIT_OFFSET;
        avList = rb+offset;
    } /* no data pointer */

    headerSize = (uint16_t)(offsetof(TblueFaceMsg, p.sdpAttributeConfirmation.avList));
    pmsg = (LPblueFaceMsg)(avList - offset);
    pmsg->length = offset + length;
    pmsg->p.sdpAttributeConfirmation.bLinkContext = bLinkContext;
    pmsg->p.sdpAttributeConfirmation.totalLen = length;
    pmsg->p.sdpAttributeConfirmation.gap = (uint8_t)(offset - headerSize);

    BLUEAPI_TRACE_PRINTF_4(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI_Handle_BT_SDP_ATTRIBUTE_CONF: uuid=0x%x handle=0x%x attrRange=0x%x len=%d",
        pBlueAPIData->SM_SDP_ServiceUUID, serviceHandle, pBlueAPIData->attributeRange, length);

    switch (pBlueAPIData->SM_SDP_ServiceUUID)
    {
    case UUID_PNPINFORMATION: 
        blueAPI_SDPDecodeDIPAttr(pBlueAPIData, serviceHandle,
            pmsg->p.sdpAttributeConfirmation.avList + pmsg->p.sdpAttributeConfirmation.gap, length);

        pBlueAPIData->serviceHandleIndex++;
        getNextService = TRUE;
        break;

    case UUID_ATT:
        blueAPI_SDPDecodeGATTAttr(pBlueAPIData, serviceHandle,
            pmsg->p.sdpAttributeConfirmation.avList + pmsg->p.sdpAttributeConfirmation.gap, length);
        break;

    default:
        blueAPI_SDPDecodeRFCOMMAttr(pBlueAPIData, serviceHandle,
            pmsg->p.sdpAttributeConfirmation.avList + pmsg->p.sdpAttributeConfirmation.gap, length);
        break;
    }

    if(tc && tc->respp)
    {
        osBufferRelease(tc->respp);
        tc->respp    = NULL;
        tc->nextCode = 0;
    }

    if(getNextService)
    {
        blueAPI_SDPGetNextService(pBlueAPIData);
    }
}

/**
* @brief  blueface send name confirm
*
* @param  app:
* @param  bd:  bdaddr
* @param  name: device name
* @param  cause: error code
*
* @return
*
*/
void blueFaceSendBT_HCI_NAME_CONF(LPCBYTE bd, uint8_t *name, uint16_t cause)
{
    TBlueAPI_Cause param_cause = blueAPI_CauseSuccess;
    uint16_t nameLen;
    uint8_t rb[32];
  
    if (name == NULL)
    {
        /* no data pointer given, allocate memory from the pool */
        name = rb + sizeof(rb) - 1; /* point to last byte in buffer */
        *name = 0;                  /* generate the empty string */
    } /* no data pointer */

    nameLen = strlen((char *)name);
  
    if (cause != HCI_SUCCESS)
    {
        param_cause = blueAPI_CauseConnectionLost;
        name = (uint8_t *)"";
        nameLen = 0;
    }
  
    blueAPIStore_SetPeerInfo(pBlueAPIData, (uint8_t *)bd, BLUEFACE_BDTYPE_BR_EDR, name, nameLen, NULL);
  
    switch (pBlueAPIData->SM_SDP_DS_Command)
    {
    case blueAPI_EventDeviceNameReq:
        blueAPI_Send_DeviceNameRsp(pBlueAPIData->SM_SDP_DS_CommandMsg, (uint8_t *)bd, name,param_cause);
        break;

    case blueAPI_EventSDPDiscoveryReq:
    case blueAPI_EventGATTSDPDiscoveryReq:
        if (nameLen >= BLUE_API_DEVICE_NAME_LENGTH)
        {  
            nameLen = BLUE_API_DEVICE_NAME_LENGTH -1;
        }

        memcpy(pBlueAPIData->SM_SDP_Data.DID_Device_Name, name, nameLen);
        pBlueAPIData->SM_SDP_Data.DID_Device_Name[nameLen] = '\0';
        if ((pBlueAPIData->SM_SDP_DS_Command == blueAPI_EventSDPDiscoveryReq)
            || (pBlueAPIData->SM_SDP_DS_Command == blueAPI_EventGATTSDPDiscoveryReq))
        {
            blueAPI_Send_DIDDeviceInd(pBlueAPIData,
                                      pBlueAPIData->SM_SDP_Data.DID_ServiceHandle,
                                      (uint8_t *)bd,
                                      pBlueAPIData->SM_SDP_Data.DID_VendorID,
                                      pBlueAPIData->SM_SDP_Data.DID_VendorID_Source,
                                      pBlueAPIData->SM_SDP_Data.DID_ProductID,
                                      pBlueAPIData->SM_SDP_Data.DID_Version,
                                      pBlueAPIData->SM_SDP_Data.DID_Device_Name
                                      );
        }
        break;

    default:
        break;
    }
} /* blueFaceSendBT_HCI_NAME_CONF */

/**
 * @brief  blueface send auth confirm
 *
 * @param  cause
 *
 * @return  
 *
 */
void blueFaceSendBT_HCI_AUTH_CONF(uint16_t cause)
{
    BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE, "blueAPI_Handle_BT_HCI_AUTH_CONF: cause:0x%x", cause);

    switch (cause)
    {
    case HCI_SUCCESS: 
        pBlueAPIData->SM_SDP_DS_Cause = blueAPI_CauseSuccess;
        break;

    case HCI_ERR | HCI_ERR_PARING_NOT_ALLOWED:
        pBlueAPIData->SM_SDP_DS_Cause = blueAPI_CauseReject;
        break;

    case HCI_ERR | HCI_ERR_AUTHENTICATION_FAILED: 
        pBlueAPIData->SM_SDP_DS_Cause = blueAPI_CauseAuthenticationFailed;
        break;

    default: 
        pBlueAPIData->SM_SDP_DS_Cause = blueAPI_CauseUnspecified;
        break;
    }

    /* authentication done, disconnect SDP link */
    blueAPI_ChannelDisconnect(pBlueAPIData, pBlueAPIData->pSDPLinkContext, FALSE);
}

/**
 * @brief  blueface send bt hci key indicate
 *
 * @param  bLinkContext
 * @param  keyRequestType
 * @param  bd
 *
 * @return
 *
 */
void blueFaceSendBT_HCI_KEY_IND(uint16_t keyRequestType, LPCBYTE bd)
{
    TDEVICE_DATA_ELEMENT_BREDR_LINKKEY linkkey;
    uint16_t cause = HCI_ERR | HCI_ERR_KEY_MISSING;
    BOOL sendResp = TRUE;

    BLUEAPI_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI_Handle_BT_HCI_KEY_IND: bd:%s reqType:0x%x",
        BLUEAPI_TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE, bd), keyRequestType);

    linkkey.keyType = (uint8_t)keyRequestType;

    switch (keyRequestType)
    {
    case BLUEFACE_LINKKEY:
        switch (pBlueAPIData->ConfigParameter.StoreBondMode)
        {
        case blueAPI_StoreBondModeNVStore:
        case blueAPI_StoreBondModeRAMStore:
            /* do not use stored key during outgoing authreq (force pairing) */
            if ((pBlueAPIData->SM_SEC_DS_CommandMsg == NULL) ||
                (pBlueAPIData->SM_SEC_DS_CommandMsg->Command != blueAPI_EventAuthReq) ||
                (memcmp(pBlueAPIData->SM_SEC_DS_CommandMsg->p.AuthReq.remote_BD, bd, BD_ADDR_SIZE) != 0)
               )
            {
                if (blueAPIStore_GetBRLinkKey(pBlueAPIData, (uint8_t *)bd, &linkkey))
                {
                    cause = HCI_SUCCESS;
                }
            }
            break;

        case blueAPI_StoreBondModeExtStore:
            blueAPI_Send_AuthResultRequestInd((uint8_t *)bd, blueAPI_RemoteBDTypeClassic,
                blueAPI_LinkKeyTypeRequestBR, 0x0000);
            sendResp = FALSE;
            break;

        default:
            break;
        }
        break;

    case BLUEFACE_LOCALPIN: 
    case BLUEFACE_REMOTEPIN: 
        blueAPI_Send_UserAuthRequestInd((uint8_t *)bd);
        sendResp = FALSE;
        break;

    default:
        break;
    }

    if (sendResp)
    {
        btApiBT_HCI_KEY_RESP(
            (linkkey.keyType == BLUEFACE_KEYTYPE_AUTHENTICATED) ? BLUEFACE_LINKKEY_MITM : BLUEFACE_LINKKEY,
            cause, LINK_KEY_SIZE, linkkey.key, (uint8_t *)bd);
    }
}

/**
 * @brief  blueface send bt hci new link key indicate
 *
 * @param  bLinkContext: 
 * @param  bd
 * @param  linkKey
 * @param  keyType
 *
 * @return
 *
 */
void blueFaceSendBT_HCI_NEWKEY_IND(BLINKCONTEXT bLinkContext, LPCBYTE bd,
    LPCBYTE linkKey, uint8_t keyType)
{
    TDEVICE_DATA_ELEMENT_BREDR_LINKKEY linkkey;
    uint32_t appData = 0x00000000;

    BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE, "blueAPI_Handle_BT_HCI_NEWKEY_IND: keyType:0x%x",
        keyType);

    switch (pBlueAPIData->ConfigParameter.StoreBondMode)
    {
    case blueAPI_StoreBondModeNVStore: 
    case blueAPI_StoreBondModeRAMStore: 
        linkkey.keyType = keyType;
        memcpy(linkkey.key, linkKey, LINK_KEY_SIZE);
        blueAPIStore_SetBRLinkKey(pBlueAPIData, (uint8_t *)bd, &linkkey);
        blueAPIStore_GetPeerInfo(pBlueAPIData, (uint8_t *)bd, BLUEFACE_BDTYPE_BR_EDR, NULL, 0, &appData);
        /* no break */

    case blueAPI_StoreBondModeExtStore: 
        blueAPI_Send_AuthResultInd((uint8_t *)bd, blueAPI_RemoteBDTypeClassic, LINK_KEY_SIZE,
            (uint8_t *)linkKey, (TBlueAPI_LinkKeyType)keyType, blueAPI_CauseSuccess);
        break;

    default: 
        break;
    }
}

/**
* @brief  send active indicate to app
*
* @param  app: application
* @param  bd: local bdaddr
* @param  cause: status
*
* @return  
*
*/
void blueFaceSendBT_ACT_IND(TCBdAddr bd, uint16_t cause)
{
    BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE, "blueAPI: BT_ACT_IND cause: 0x%04x", cause);

    blueAPIStore_Init(pBlueAPIData);

    /* store local BD */
    memcpy(pBlueAPIData->local_BD, bd, BD_ADDR_SIZE);

    pBlueAPIData->state |= BLUE_API_STATE_BF_ACT_IND;

    if (cause == blueFaceNoError)
    {
        pBlueAPIData->systemPoolID = BTSystemPoolID;
        blueAPI_SendToAll_ActInfo(pBlueAPIData, blueAPI_CauseSuccess, TRUE);
    }
    else
    {
        pBlueAPIData->systemPoolID = 0;

        switch (cause)
        {
        case (HCI_ERR|HCI_ERR_TIMEOUT): 
            pBlueAPIData->stateCause = blueAPI_CauseInitTimeout;
            break;

        case (HCI_ERR|HCI_ERR_OUTOFSYNC): 
            pBlueAPIData->stateCause = blueAPI_CauseInitOutofSync;
            break;

        case (HCI_ERR|HCI_ERR_INFOPAGE_MISSING): 
            pBlueAPIData->stateCause = blueAPI_CauseInitHardwareFailure;
            break;

        default: 
            pBlueAPIData->stateCause = blueAPI_CauseUnspecified;
            break;
        }

        /* BF returned an error => inform all registered Apps */
        /* allow sending a second ActInfo (caused by RadioModeSetReq/RadioOff) */
        blueAPI_SendToAll_ActInfo(pBlueAPIData, pBlueAPIData->stateCause, TRUE);
    }
}

/**
 * @brief  blueface send bt config indicate mode change
 *
 * @param  pModeChangeInd
 *
 * @return  
 *
 */
void blueFaceSendBT_CONF_IND_Mode_Change(LPHCI_MODECHANGE_IND pModeChangeInd)
{
#if (BLUEAPI_ENABLE_ACL_INFO & BLUE_API_ENABLE_ACL_INFO_CONNECTED_SNIFF)        
    TBlueAPI_ACLStatusParam aclStatusParam;
#endif

    switch (pModeChangeInd->mode)
    {
#if (BLUEAPI_ENABLE_ACL_INFO & BLUE_API_ENABLE_ACL_INFO_CONNECTED_ACTIVE)
    case 0: /* active mode */
        blueAPI_Send_ACLStatusInfo(pModeChangeInd->bd, blueAPI_RemoteBDTypeClassic,
            blueAPI_ACLConnectedActive, NULL);
        break;
#endif

#if (BLUEAPI_ENABLE_ACL_INFO & BLUE_API_ENABLE_ACL_INFO_CONNECTED_SNIFF)
    case 2: /* sniff mode */
        aclStatusParam.sniff.interval = pModeChangeInd->interval;
        blueAPI_Send_ACLStatusInfo(pModeChangeInd->bd, blueAPI_RemoteBDTypeClassic,
            blueAPI_ACLConnectedSniff, &aclStatusParam);
        break;
#endif

    default:
        break;
    }

    if (pBlueAPIData->SM_SEC_DS_CommandMsg != NULL &&
        pBlueAPIData->SM_SEC_DS_CommandMsg->Command == blueAPI_EventACLConfigReq)
    {
        PBlueAPI_ACLConfigReq pACLConfigReq = &pBlueAPIData->SM_SEC_DS_CommandMsg->p.ACLConfigReq;
        TBlueAPI_Cause cause = blueAPI_CauseSuccess;

        if (pModeChangeInd->status == HCI_SUCCESS)
        {
            if (pACLConfigReq->p.sniffmode.maxLatency != 0)
            {
                blueAPI_Send_BLUEFACE_CONF_SNIFF_SUBRATING(pBlueAPIData, pACLConfigReq->remote_BD,
                    pACLConfigReq->p.sniffmode.maxLatency, pACLConfigReq->p.sniffmode.minRemoteTimeout,
                    pACLConfigReq->p.sniffmode.minLocalTimeout);

                blueAPI_Send_ACLConfigRsp(pBlueAPIData->SM_SEC_DS_CommandMsg, pACLConfigReq->remote_BD,
                    pACLConfigReq->remote_BD_Type, blueAPI_ACLConfigSniffmode, blueAPI_CauseSuccess);

                pBlueAPIData->SM_SEC_DS_CommandMsg = NULL;
                return;
            }
        }
        else
        {
            cause = blueAPI_CauseUnspecified;
        }

        blueAPI_Send_ACLConfigRsp(pBlueAPIData->SM_SEC_DS_CommandMsg, pACLConfigReq->remote_BD,
            pACLConfigReq->remote_BD_Type, blueAPI_ACLConfigSniffmode, cause);

        /*we need to fix it*/
        pBlueAPIData->SM_SEC_DS_CommandMsg = NULL;
    }
}

/**
* @brief  blueface find and check link
*
* @param  pBTA:
* @param  cid: 
* @param  pTxt
*
* @return  
*
*/
LPbtLink blueFaceFindAndCheckLink(PBTA pBTA, uint16_t cid, const char *pTxt)
{
    LPbtLink       pLink;

    pLink = blueFaceFindLinkByHandle(cid, BLUEFACE_PSM_GATT);    
    if (pLink==NULL)
    {
              BLUEFACE_TRACE_PRINTF_1(BLUEFACE_TRACE_MASK_ERROR,"!!BTMAN: cannot find cid %X",  cid);
    }
    else if (pLink->app==NULL)
    {
        /* there is no application to control the link: kill everything immediately */
        BLUEFACE_TRACE_PRINTF_1(BLUEFACE_TRACE_MASK_TRACE,
                "BTMAN: no app for cid %X", cid);
        blueFaceDeallocateLink(pLink);
        pLink=NULL;
    } /* no app */

    return(pLink);
}

/**
* @brief  blueface send BT device data indicate
*
* @param  app:
* @param  pmsg
*
* @return  
*
*/
void blueFaceSendBT_DEVICE_DATA_IND(MESSAGE_T * pmsg)
{
    LPblueFaceMsg pblueFaceMsg;
    BOOL retVal;

    if ((pBTA->secApp != NULL) && pBTA->secApp->security)
    {

    }
    else
    {
        osBufferRelease((LPblueFaceMsg)pmsg->MData.DataCB.BufferAddress);
        return;
    }

#if CHECK_API_PARAM
    /* use buffer for message setup */
    if (pmsg->MData.DataCB.Offset != blueFaceMsgSize)
    {
        assert(FALSE);
    }
    else
#endif        
    {
        pblueFaceMsg = (LPblueFaceMsg)pmsg->MData.DataCB.BufferAddress;

        if (pmsg->Command == DEVICE_DATA_GET_IND)
        {
            pblueFaceMsg->command = BT_DEVICE_DATA_GET_IND;
        }
        else
        {
            pblueFaceMsg->command = BT_DEVICE_DATA_SET_IND;
        }
        /* other parameters are already setup properly! */
    }

    retVal = blueAPIStore_QueueIn(pBlueAPIData, pblueFaceMsg);
    blueAPIStore_QueueTrigger(pBlueAPIData, FALSE);

    if (retVal)
    {
        osBufferRelease(pblueFaceMsg);
    }
}

