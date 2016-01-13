/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       bt_api.c
* @brief     Bluetooth L2 message handler
* @details   
*
* @author   	gordon
* @date      	2015-07-08
* @version	v0.1
*/

#include <flags.h>
#include <blueface.h>
#include <bt_api.h>
#include <bluemgr.h>
#include <os_pool.h>
#include <btman.h>
#include <btsend.h>
#include <btglib.h>
#include <btcommon.h>
#include <gatt_api.h>
#include <btsm_api.h>
#include <upper_stack_global.h>
#include <trace_binary.h>
#include <hci_api.h>
#include <sdp_api.h>
#include <hci_code.h>
#include <blueface.h>
#include <blueapi_api.h>

#define TRACE_MODULE_ID     MID_BLUEFACE

/**
* @brief  btapi prepare l2cap confirm param
* 
* @param  app
* @param  pBfConf
*
* @return  
*
*/
LP_L2C_Conf_para btApiBT_PrepareL2capConfPara(LPbtApplication app, LPCONF_L2CAP pBfConf)
{
	LP_L2C_Conf_para pConfPara;
	LPFlowControl    pflc;
	pflc = &pBfConf->flc;

	pConfPara = commonPrepareL2cConfPara(pBfConf->mtuSize,pflc->maxPDUSize,0);
	if(pConfPara == 0)
	{
	    assert(FALSE);
	    return (LP_L2C_Conf_para)0;
	}

	pConfPara->validMask.FlushTO = 1;
	pConfPara->FlushTO = pBfConf->flushTO;

	if (pBfConf->flow.flags==0xff)
	{
	}
	else
	{
	    LPFlowSpec pflow;

	    /* this is a configuration request with QoS */
	    pConfPara->validMask.Qos = 1;

	    pflow = &pBfConf->flow;
	    pConfPara->flowSpec.flags          = pflow->flags;
	    pConfPara->flowSpec.serviceType    = pflow->serviceType;
	    pConfPara->flowSpec.tokenRate      = pflow->tokenRate;
	    pConfPara->flowSpec.tokenBucket    = pflow->tokenBucket;
	    pConfPara->flowSpec.peakBandwidth  = pflow->peakBandwidth;
	    pConfPara->flowSpec.latency        = pflow->latency;
	    pConfPara->flowSpec.delayVariation = pflow->delayVariation;
	}

	pConfPara->flowControl.mode                  = pBfConf->mode;
	pConfPara->flowControl.txWindowSize          = pflc->txWindowSize;
	pConfPara->flowControl.maxTransmit           = pflc->maxTransmit;
	pConfPara->flowControl.retransmissionTimeout = pflc->retransmissionTimeout;
	pConfPara->flowControl.monitorTimeout        = pflc->monitorTimeout;
	pConfPara->fcs         = pBfConf->fcs;
	pConfPara->maxDsBuffer = pBfConf->maxDsBuffer;

	return pConfPara;
} /* blueFacePrepareL2capConfPara */
 
/**
  * @brief  btapi send sdp attribute request
  * 
  * @param  bLinkHandle
  * @param  maxLen
  * @param  serviceHandle
  * @param  lenAttrib
  * @param  pAttrib
  *
  * @return
  *
  */
blueFaceStatus btApiBT_SDP_ATTRIBUTE_REQ(BLINKHANDLE bLinkHandle, uint16_t maxLen,
    uint32_t serviceHandle, uint16_t lenAttrib, uint8_t *pAttrib)
{
    LPbtLink pLink = bLinkHandle;

#if CHECK_API_PARAM
    if (BLUEFACE_CHECK_LINK_VALID())
    {
        BLUEFACE_TRACE_PRINTF_1(BLUEFACE_TRACE_MASK_ERROR,
            "!!blueFaceHandleSdpAttributeReq, ill linkhandle %p", pLink);
    }
    else if (pLink->state != link_Connected || pLink->psm != BLUEFACE_PSM_SDP)
    {
        BLUEFACE_TRACE_PRINTF_1(BLUEFACE_TRACE_MASK_ERROR,
            "!!blueFaceHandleSdpAttributeReq, syntax / state error in link descriptor %lx", pLink);
    }
    /* send message to local sdp instance */
    else
#endif
    {
        if(sdpHandleUpAttributeReq(pLink->handle, serviceHandle, maxLen, lenAttrib, pAttrib) == 0)
        {
            return blueFaceNoError;   /* every thing ok just return */
        }
    }

    /* we get here only in case of an error, so give empty reply */
    blueFaceSendBT_SDP_ATTRIBUTE_CONF(pLink->context, NULL, 0, 0, NULL);

    return blueFaceNoError;
}

/**
 * @brief  bt api send sdp search request
 *
 * @param  bLinkHandle
 * @param  maxHandles
 * @param  lenUuid
 * @param  pUuid
 *
 * @return  
 *
 */
blueFaceStatus btApiBT_SDP_SEARCH_REQ(BLINKHANDLE bLinkHandle, uint16_t maxHandles,
    uint16_t lenUuid, uint8_t *pUuid)
{
    LPbtLink pLink = bLinkHandle;

#if CHECK_API_PARAM
    if (BLUEFACE_CHECK_LINK_VALID())
    {
        BLUEFACE_TRACE_PRINTF_1(BLUEFACE_TRACE_MASK_ERROR,
            "!!blueFaceHandleSdpSearchReq, ill linkhandle %p", pLink);
    }
    else if (pLink->state != link_Connected || pLink->psm != BLUEFACE_PSM_SDP)
    {
        BLUEFACE_TRACE_PRINTF_1(BLUEFACE_TRACE_MASK_ERROR,
            "!!blueFaceHandleSdpSearchReq, syntax / state error in link descriptor %lx", pLink);
    }
    /* send message to local sdp instance */
    else
#endif
    {
        if (sdpHandleUpSearchReq(pLink->handle, maxHandles, lenUuid, pUuid) == 0)
        {
            return blueFaceNoError;   /* ewery thing ok just return */
        }
    }

    /* we get here only in case of an error, so give empty reply */
    blueFaceSendBT_SDP_SEARCH_CONF(pLink->context, NULL, 0, 0);

    return blueFaceNoError;  
}

/**
 * @brief  blueapi page mode req
 *
 * @param  pageMode: page mode
 *
 * @return  
 *
 */
blueFaceStatus btApiBT_PAGEMODE_REQ(uint8_t pageMode)
{
    hciHandleUpWriteScanEnable(pageMode, SAVE_PARAM);
    hciLaterEntry();

    return blueFaceNoError;
}

/**
 * @brief  btapi send connect response
 * 
 * @param  appHandle
 * @param  bLinkHandle
 * @param  bLinkContext
 * @param  accept
 * @param  lenPara
 * @param  para
 *
 * @return  
 *
 */
blueFaceStatus btApiBT_CON_RESP(BLINKHANDLE bLinkHandle, BLINKCONTEXT bLinkContext,
    uint8_t accept, uint16_t lenPara, uint8_t *para)
{
    TCON_RESP conf_rsp;
    LPbtLink pLink = bLinkHandle;

    if (para)
    {
        memcpy((PVOID)&conf_rsp.p, (PVOID)para, lenPara);
    }

#if CHECK_API_PARAM
    if (BLUEFACE_CHECK_LINK_VALID())
    {
        return blueFaceIllParameter;
    }
#endif    

    if (pLink->state == link_Incoming_Disc)
    {
        if (accept == BLUEFACE_CON_ACCEPT)
        {
            pLink->context = bLinkContext;

#if (F_BT_SCO)
            if (pLink->psm == BLUEFACE_PSM_SCO)
            {
                blueAPI_Send_SCODiscInd(pLink->bd);
                blueFaceDeallocateLink(pLink);
            }
            else
            {
                blueFaceSendBT_DISC_IND(pLink->context, 0);
            }
#else
            blueFaceSendBT_DISC_IND(pLink->context, 0 /* no better cause available here */);
#endif
            return blueFaceNoError;
        }
        /* ignore code or reject code on disconnected link, kill descriptor */
        blueFaceDeallocateLink(pLink);
        return blueFaceNoError;
    } /* link already disconnecting */

    /* if this is a ignore code, this application did not want the call.
           we look for the next application in sequence
        */
    if (accept == BLUEFACE_CON_IGNORE && pLink->psm != BLUEFACE_PSM_L2CAP)
    {
        blueFaceHandleConInd(pLink, pBlueAPIData->AppHandle);
        return blueFaceNoError;
    }

    if (accept == BLUEFACE_CON_ALERT)
    {
        /* link goes into alerting state: we remember the context
               (if the link goes down we need if for the disc_ind,,,)
             */
        pLink->context = bLinkContext;
        pLink->alerting = TRUE; /* link is in alerting state now: can be disc_ind... */
        return blueFaceNoError;
    }

    switch (pLink->psm)
    {
    case BLUEFACE_PSM_GATT:
        /* check for correct link state, SDP incoming connections do not run over blueFace IF */
        if (pLink->state != link_Incoming)
        {
            return blueFaceNoError;
        }
        if (accept == BLUEFACE_CON_REJECT)
        {
            btgSendConResp(blueFacePSM2hQueue(pLink->psm), pLink->handle, accept, NULL);
            blueFaceDeallocateLink(pLink);
        }
        else
        {
            pLink->context = bLinkContext;   /* remember application context */
            btgSendConResp(blueFacePSM2hQueue(pLink->psm), pLink->handle, accept, NULL);
        }

        break;

#if (F_BT_SCO)
    case BLUEFACE_PSM_SCO:
        if (accept == BLUEFACE_CON_REJECT)
        {
            blueFaceDeallocateLink(pLink);
        }
        else
        {
            pLink->state = link_Incoming_Acp;
        }
        hciHandleUpSCOConConf(pLink->bd, accept, (PCON_RESP_SCO)para);
        break;
#endif

    default:
        break;
    } 

    return blueFaceNoError;
}

/**
 * @brief  disconnect response
 *
 * @param  bLinkHandle
 *
 * @return  
 *
 */
blueFaceStatus btApiBT_DISC_RESP(BLINKHANDLE bLinkHandle)
{
    LPbtLink pLink = bLinkHandle;
    
#if CHECK_API_PARAM
    if (BLUEFACE_CHECK_LINK_VALID())
    {
         BLUEFACE_TRACE_PRINTF_1(BLUEFACE_TRACE_MASK_ERROR,
            "!!blueFaceHandleDiscResp, ill linkhandle %lx", pLink);
         return blueFaceIllParameter;
    }
#endif

    if (pLink)
    {
        if (pLink->psm == BLUEFACE_PSM_GATT)
        {
            gattHandleDisconnected(pLink->handle, 0, FALSE);
        }

        blueFaceDeallocateLink(pLink);
    }

    return blueFaceNoError;
}

/**
 * @brief  auth request
 *
 * @param  bLinkHandle
 *
 * @return  
 *
 */
blueFaceStatus btApiBT_AUTH_REQ(BLINKHANDLE bLinkHandle)
{
    LPbtLink pLink = bLinkHandle;

#if CHECK_API_PARAM
    if (BLUEFACE_CHECK_LINK_VALID())
    {
        BLUEFACE_TRACE_PRINTF_1(BLUEFACE_TRACE_MASK_ERROR,
            "!!blueFaceHandleHciAuthReq, ill linkhandle %lx", pLink);

        blueFaceSendBT_HCI_AUTH_CONF(BLUEFACE_ERR | BF_ILL_PARAMETER);
        return blueFaceIllParameter;
    }
#endif

    /* remember the context given by the application (used in auth_conf) */
    pLink->activeAuth = TRUE;

    /* send authentication request downstream */
    btsmSendMsgHciAuthReq(pLink->bd, pLink->selfIndex);
    return blueFaceNoError;
}

/**
 * @brief  btapi send key response to hci
 *
 * @param  keyRequestType
 * @param  cause
 * @param  keyLength
 * @param  pKey
 * @param  bd
 *
 * @return
 *
 */
blueFaceStatus btApiBT_HCI_KEY_RESP(uint16_t keyRequestType, uint16_t cause,
    uint16_t keyLength, uint8_t *pKey, uint8_t *bd)
{
    btsmHandleBT_HCI_KEY_RESP(bd,
        (uint8_t)(keyRequestType==BLUEFACE_LINKKEY?LINK_KEY_REQUEST: 
        (keyRequestType==BLUEFACE_LINKKEY_MITM?LINK_KEY_REQUEST_MITM:PIN_CODE_REQUEST)),
        cause, pKey, keyLength);
    return blueFaceNoError;
}

/**
* @brief  handle connect request
*
* @param  app:
* @param  pmsg: 
*
* @return  
*
*/
void btApiBT_HandleConReq(LPbtApplication app, LPblueFaceMsg pmsg)
{
    LPbtLink lp;
    uint16_t  status = BLUEFACE_ERR | BF_SUCCESS;

    /* allocate new descriptor for new connection */
    lp = blueFaceAllocateLink();

    if (lp == NULL) /* no descriptor, no connection */
    {
#if (F_BT_SCO)
        if (pmsg->p.connectRequest.psm == BLUEFACE_PSM_SCO)
        {
            blueAPI_Send_SCOConRsp(pmsg->p.connectRequest.bd, BLUEFACE_ERR | BF_NO_RESOURCES);
        }
        else
        {
            blueFaceSendBT_CON_CONF(pmsg->p.connectRequest.bLinkContext, NULL, BLUEFACE_ERR | BF_NO_RESOURCES);
        }
#else
        blueFaceSendBT_CON_CONF(pmsg->p.connectRequest.bLinkContext, NULL, BLUEFACE_ERR | BF_NO_RESOURCES);
#endif
        return;
    }

    pBTA->activeLink     = lp;
    lp->app              = app;
    lp->context          = pmsg->p.connectRequest.bLinkContext;
    lp->psm              = pmsg->p.connectRequest.psm;
    memcpy(lp->bd,(PVOID)pmsg->p.connectRequest.bd, BD_ADDR_SIZE);

    lp->state            = link_Connecting;
    lp->encryptState     = ENCRYPT_IDLE;
    lp->outgoing         = TRUE;

    switch (lp->psm)
    {
    case BLUEFACE_PSM_SDP:
        sdpHandleUpConReq((uint8_t *)pmsg->p.connectRequest.bd, pmsg->p.connectRequest.frameSize);
        break;

    case BLUEFACE_PSM_GATT:
        gattHandleUpConnectReq(pmsg->p.connectRequest.p.gatt.bdType, 
                               (uint8_t *)pmsg->p.connectRequest.bd,
                               (PCON_REQ_GATT)&pmsg->p.connectRequest.p.gatt);
        break;

#if (F_BT_SCO)
    case BLUEFACE_PSM_SCO:
        hciHandleUpSCOConReq((uint8_t *)pmsg->p.connectRequest.bd,
                            &(pmsg->p.connectRequest.p.sco));
        break;
#endif

    default:
        status = BLUEFACE_ERR | BF_PSM_NOT_IMPLEMENTED;
        break;
    } /* switch */

    if(status != (BLUEFACE_ERR | BF_SUCCESS))         /* in case of error */
    {
        blueFaceDeallocateLink(lp);
        lp = NULL;
        blueFaceReleaseActiveLinkSemaphore(); //      pBTA->activeLink = NULL;
    }

#if (F_BT_SCO)
    if (pmsg->p.connectRequest.psm != BLUEFACE_PSM_SCO)
    {
        blueFaceSendBT_CON_CONF(pmsg->p.connectRequest.bLinkContext, (BLINKHANDLE)lp, status);
    }
#else
    blueFaceSendBT_CON_CONF(pmsg->p.connectRequest.bLinkContext, (BLINKHANDLE)lp, status);
#endif
} /* blueFaceHandleConReq */

/**
* @brief  connect channel
*
* @param  appHandle:
* @param  context: 
* @param  bd: 
* @param  frameSize
* @param  psm
* @param  uuid
* @param  lenPara
* @param  para
*
* @return  
*
*/
blueFaceStatus btApiBT_CON_REQ(BAPPHANDLE appHandle, BLINKCONTEXT context, uint8_t *bd,            uint16_t frameSize,
    uint16_t psm, uint16_t uuid, uint16_t lenPara, uint8_t *para)
{
    PblueFaceMsg pmsg;
    BOOL flag = DATA_CB_RELEASE;
    MESSAGE_T lmsg;

    if (osBufferGet(BTSystemPoolID, offsetof(TblueFaceMsg, p.connectRequest.p) + lenPara, (void *)&pmsg))
    {
        BLUEFACE_TRACE_PRINTF_0(BLUEFACE_TRACE_MASK_ERROR, "!!!btApiBT_CON_REQ, fail to get buffer");
        return blueFaceNoResources;
    }

    pmsg->length = offsetof(TblueFaceMsg,p.connectRequest.p) + lenPara;
    pmsg->p.connectRequest.bLinkContext = context;
    pmsg->p.connectRequest.frameSize = frameSize;
    pmsg->p.connectRequest.psm = psm;
    pmsg->p.connectRequest.uuid = uuid;
    if (bd)
    {
        memcpy((PVOID)pmsg->p.connectRequest.bd,(PVOID)bd, BD_ADDR_SIZE);
    }
    else
    {
        memset((PVOID)pmsg->p.connectRequest.bd, 0, BD_ADDR_SIZE);
    }

    if (para)
    {
        memcpy((PVOID)&pmsg->p.connectRequest.p, (PVOID) para, lenPara);
    }

    if (pBTA->activeLink)
    {
        lmsg.MData.BlueFace.MessageReq.DataCB.BufferAddress = (uint8_t *)pmsg;
        lmsg.MData.BlueFace.MessageReq.ApplHandle = appHandle;
        osMessageSend(pBTA->btReqQueueID, &lmsg);
        flag &= ~DATA_CB_RELEASE;
    }
    else
    {
        btApiBT_HandleConReq((LPbtApplication)appHandle, pmsg);
    }

    if (flag & DATA_CB_RELEASE)
    {
        osBufferRelease(pmsg);
    }

    return blueFaceNoError;
}

/**
* @brief  send disconnect req to related queue ID
* 
* @param  appHandle:  
* @param  bLinkHandle: 
* @param  cause: disconnect reason
* @param  holdLink
* @return  blueFaceStatus
*
*/
blueFaceStatus btApiBT_DISC_REQ(BLINKHANDLE bLinkHandle, uint16_t cause, uint16_t holdLink)
{
    LPbtLink pLink = bLinkHandle;
    BOOL param_holdlink = FALSE;

    switch (pLink->state)
    {
    case link_Connecting:
        if (pLink->handle == 0xffff)
        {
            pLink->delayedDisc = TRUE;
            return blueFaceNoError;
        }
        break;

    case link_Disconnecting:
        /* already disconnecting, nothing to do */
        return blueFaceNoError;

    case link_Connected:
        /* normally condition, just break*/
        break;

    default:
        break;
    }

    pLink->state = link_Disconnecting;

    switch (pLink->psm)
    {
    case BLUEFACE_PSM_SDP:
        param_holdlink = holdLink;
        break;

    case BLUEFACE_PSM_GATT:
        break;

#if (F_BT_SCO)
    case BLUEFACE_PSM_SCO:
        break;
#endif

    default:
        pLink->psm = 0;       /* invalidate the PSM */
        break;
    }

    if( pLink->psm != 0)
    {
        btgSendDiscReq(blueFacePSM2hQueue(pLink->psm), pLink->handle, cause, param_holdlink);
    }

    return blueFaceNoError;
}

/**
 * @brief  send inquiry req
 *
 * @param  appHandle
 * @param  TimeOut
 * @param  MaxBd
 * @param  classMask
 * @param  classValue
 * @param  idxIAC
 *
 * @return
 *
 */
blueFaceStatus btApiBT_HCI_INQUIRY_REQ(BAPPHANDLE appHandle, uint16_t TimeOut, uint16_t MaxBd, uint8_t idxIAC)
{
    LPbtApplication  app = appHandle;

    if (app == app->bta->activeApp && MaxBd == 0)
    {
        hciHandleUpInquiryReq(0, 0, GIAC_IDX);
        return blueFaceNoError;
    }

    app->bta->activeApp = app;
    hciHandleUpInquiryReq(TimeOut, MaxBd, idxIAC);

    return blueFaceNoError;
}

/**
 * @brief  handle device name request
 *
 * @param  appHandle:
 * @param Bd: 
 *
 * @return
 *
 */
blueFaceStatus btApiBT_HCI_NAME_REQ(BAPPHANDLE appHandle, uint8_t *bd)
{
    LPbtApplication  app = appHandle;

    if (app->bta->activeApp)
    {
        BLUEFACE_TRACE_PRINTF_2(BLUEFACE_TRACE_MASK_ERROR,
                                "!!blueFaceHandleHciNameReq: already active app %lx active %lx",
                                app, app->bta->activeApp
                                );
        blueFaceSendBT_HCI_NAME_CONF((PBYTE)bd, NULL, BLUEFACE_ERR | BF_BUSY);
        return blueFaceNoError;
    }

    app->bta->activeApp = app;
    hciHandleUpNameReq(bd);
    return blueFaceNoError;
}

/**
* @brief  btapi device data response
*
* @param  command
* @param  handle
* @param  status
* @param  pDeviceData
*
* @return  
*
*/
blueFaceStatus btApiDEVICE_DATA_RESP(TblueFaceEvent command, PVOID handle,
    uint16_t status, PDEVICE_DATA pDeviceData)
{
    TDeviceDataResp deviceDataResp;

    deviceDataResp.handle = handle;
    deviceDataResp.status = status;
    deviceDataResp.deviceData = *pDeviceData;

    /* determine receiver of msg based on data type */
    switch (deviceDataResp.deviceData.dataType & DEVICE_DATA_TYPE_MASK)
    {
    case DEVICE_DATA_TYPE_GATT:
        if(command == BT_DEVICE_DATA_GET_RESP)
        {
            gattHandleDEVICE_DATA_RESP(DEVICE_DATA_GET_RESP, &deviceDataResp);
        }
        else
        {
            gattHandleDEVICE_DATA_RESP(DEVICE_DATA_SET_RESP, &deviceDataResp);
        }
        break;

    case DEVICE_DATA_TYPE_SECMAN:
        if(command == BT_DEVICE_DATA_GET_RESP)
        {
            btsmHandleDEVICE_DATA_GET_RESP(&deviceDataResp);

        }
        else
        {
            btsmHandleDEVICE_DATA_SET_RESP(&deviceDataResp);
        }
        break;

    default:
        break;
    }

    return blueFaceNoError;
}

/**
 * @brief  btapi device data get response
 *
 * @param  handle
 * @param  status
 * @param  pDeviceData
 *
 * @return  
 *
 */
blueFaceStatus btApiDEVICE_DATA_GET_RESP(PVOID handle, uint16_t status, PDEVICE_DATA pDeviceData)
{
    return (btApiDEVICE_DATA_RESP(BT_DEVICE_DATA_GET_RESP, handle, status, pDeviceData));
}

/**
 * @brief  btapi device data set response
 *
 * @param  handle
 * @param  status
 * @param  pDeviceData
 *
 * @return  
 *
 */
blueFaceStatus btApiDEVICE_DATA_SET_RESP(PVOID handle, uint16_t status, PDEVICE_DATA pDeviceData)
{
    return(btApiDEVICE_DATA_RESP(BT_DEVICE_DATA_SET_RESP, handle, status, pDeviceData));
}

#if CHECK_API_PARAM
/**
 * @brief  check if link valid
 *
 * @param  app:
 * @param  pLink 
 *
 * @return
 *
 */
BOOL btApiGATT_LinkValid(LPbtApplication app, LPbtLink pLink)
{
    /* check for correct link descriptor */
    if (BLUEFACE_CHECK_LINK_VALID())
    {
        return(FALSE);
    }

    if (!pLink->used || pLink->app != app)
    {
        return(FALSE);
    }
    return(TRUE);
}
#endif

/**
  * @brief  btapi gatt attribute update request
  *
  * @param  reqHandle: 
  * @param  serviceHandle
  * @param  attribIndex: index of attrib. in service structure
  * @param  length:
  * @param  offset: offset of attrib. data in pBuffer 
  * @param  pBuffer
  *
  * @return
  *
  */
blueFaceStatus btApiGATT_ATTRIB_UPDATE_REQ(PVOID reqHandle, PVOID serviceHandle,
    uint16_t attribIndex, uint16_t length, uint16_t offset, uint8_t *pBuffer)
{
    if (length > 0)
    {
        /* data is supplied in (OSIF) pBuffer */
        if (pBuffer == NULL)
        {
            return blueFaceNoResources;
        }
    }
    else
    {
        /* no data supplied => pBuffer MUST be NULL !!!! */
        if (pBuffer != NULL)
        {
            return blueFaceIllParameter;
        }

        if (osBufferGet(BTSystemPoolID, BT_DS_WRITE_OFFSET_COUNT, (void *)&pBuffer))
        {
            return blueFaceNoResources;
        }

        offset = BT_DS_WRITE_OFFSET_COUNT;
    }

    gattHandleGATT_ATTRIB_UPDATE_REQ(pBuffer,reqHandle, serviceHandle, attribIndex, length, offset);

    return blueFaceNoError;;   /* msg buffer will not be released by callers */
}

/**
  * @brief  read server application supplied attribute value response
  *
  * @param  bLinkHandle: 
  * @param  serviceHandle
  * @param  cause
  * @param  length
  * @param  offset
  * @param  pBuffer
  *
  * @return
  *
  */
blueFaceStatus btApiGATT_ATTRIB_READ_RESP(BLINKHANDLE bLinkHandle,
    PVOID serviceHandle, uint16_t cause, uint16_t length, uint16_t offset, uint8_t *pBuffer)
{
    LPbtLink pLink = bLinkHandle;
    PGATTAttribReadRespParam pParam;

#if CHECK_API_PARAM
    /* check for correct link descriptor */
    if (!btApiGATT_LinkValid(app, pLink))
    {
        return( blueFaceIllParameter);
    }
#endif

    if (length > 0)
    {
        /* data is supplied in (OSIF) pBuffer */
        if (pBuffer == NULL)
        {
            return blueFaceNoResources;
        }
    }
    else
    {
        /* no data supplied => pBuffer MUST be NULL !!!! */
        if (pBuffer != NULL)
        {
            return blueFaceIllParameter;
        }

        if (osBufferGet(BTSystemPoolID, BT_DS_WRITE_OFFSET_COUNT, (void *)&pBuffer))
        {
            return blueFaceNoResources;
        }

        offset = BT_DS_WRITE_OFFSET_COUNT;
    }

    pParam = (PGATTAttribReadRespParam)pBuffer;
    pParam->serviceHandle = serviceHandle;
    pParam->offset = offset - (offsetof(TGATTAttribReadRespParam, data));

    gattHandleGATT_ATTRIB_READ_RESP(pLink->handle, cause, length, pParam);

    return blueFaceNoError;   /* msg buffer will not be released by callers */
}

/**
 * @brief   write server application supplied attribute value response 
 *
 * @param  appHandle
 * @param  bLinkHandle
 * @param  serviceHandle
 * @param  cause
 * @param  type
 * @param  length
 * @param  offset
 * @param  pBuffer
 *
 * @return  
 *
 */
blueFaceStatus btApiGATT_ATTRIB_WRITE_RESP(BLINKHANDLE bLinkHandle, uint16_t cause,
    uint16_t length, uint16_t offset, uint8_t *pBuffer)
{
    LPbtLink pLink = bLinkHandle;

    gattHandleGATT_ATTRIB_WRITE_RESP(pLink->handle, cause, length, offset, pBuffer);

    return blueFaceNoError;   /* msg buffer will not be released by callers */
}

blueFaceStatus btApiGATT_EXECUTE_WRITE_RESP(BLINKHANDLE bLinkHandle, uint16_t cause, uint16_t handle)
{
    LPbtLink pLink = bLinkHandle;
    gattHandleGATT_EXECUTE_WRITE_RESP(pLink->handle, cause, handle);
    return blueFaceNoError;   
}

blueFaceStatus btApiGATT_EXECUTE_WRITE_REQ(BLINKHANDLE bLinkHandle, uint8_t flags)
{
    LPbtLink pLink = bLinkHandle;
    gattHandleGATT_EXECUTE_WRITE_REQ(pLink->handle, flags);
    return blueFaceNoError; 
}

/**
  * @brief   gatt discovery request
  *
  * @param  bLinkHandle
  * @param  type
  * @param  startingHandle
  * @param  endingHandle
  * @param  uuid16
  * @param  pUuid128
  *
  * @return  
  *
  */
blueFaceStatus btApiGATT_DISCOVERY_REQ(BLINKHANDLE bLinkHandle, uint16_t type,
    uint16_t startingHandle, uint16_t endingHandle, uint16_t uuid16, uint8_t *pUuid128)  
{
    TGATTDiscoveryReq tDiscoveryReq;
    LPbtLink pLink = bLinkHandle;

    tDiscoveryReq.cid = pLink->handle;
    tDiscoveryReq.type = type;
    /* all discovery req. share the handle parameters: */
    tDiscoveryReq.p.common.startingHandle = startingHandle;
    tDiscoveryReq.p.common.endingHandle = endingHandle;

    switch (type)
    {
    case GATT_TYPE_DISCOVERY_PSRV_UUID:
        if (uuid16 != 0)
        {
            tDiscoveryReq.p.psrvUUID.uuidType = UUID_TYPE_16;
            tDiscoveryReq.p.psrvUUID.uuid.uuid16 = uuid16;
        }
        else if (pUuid128 != NULL)
        {
            tDiscoveryReq.p.psrvUUID.uuidType = UUID_TYPE_128;
            tDiscoveryReq.p.psrvUUID.uuid.pUuid128 = pUuid128;
        }
        break;
    
    default:
        break;
    }

    gattHandleGATT_DISCOVERY_REQ(&tDiscoveryReq);

    return blueFaceNoError;
}

/**
 * @brief  Discover services, relationships or characteristics  
 *
 * @param  bLinkHandle: 
 * @param  type: type of search
 * @param  startingHandle: 
 * @param  endingHandle
 * @param  uuid16: 16 bit UUID
 * @param  pUuid128: 128 bit UUID
 *
 * @return
 *
 */
blueFaceStatus btApiGATT_DISCOVERY_RESP(BLINKHANDLE bLinkHandle,
    uint16_t type, uint16_t startingHandle, uint16_t endingHandle)
{
    LPbtLink pLink = bLinkHandle;

    gattHandleGATT_DISCOVERY_RESP(pLink->handle, type, startingHandle, endingHandle);

    return blueFaceNoError;
}

/**
 * @brief   GATT "Read Request", "Read Blob Request" and "Read Multiple Request" 
 *
 * @param  appHandle:
 * @param  bLinkHandle: 
 * @param  valueOffset
 * @param  nbrOfHandles
 * @param  pHandles
 *
 * @return
 *
 */
blueFaceStatus btApiGATT_ATTRIB_READ_REQ(BLINKHANDLE bLinkHandle,
    uint16_t valueOffset, uint16_t nbrOfHandles, LPWORD pHandles)
{
    TGATTAttribReadReq tgattAttribReadReq;
    LPbtLink pLink = bLinkHandle;

    tgattAttribReadReq.cid = pLink->handle;
    if (valueOffset == 0)
    {
        tgattAttribReadReq.type = GATT_READ_TYPE_BASIC;
        tgattAttribReadReq.p.basic.handle = *pHandles;       
    }
    else
    {
        tgattAttribReadReq.type = GATT_READ_TYPE_BLOB;
        tgattAttribReadReq.p.blob.handle  = *pHandles;
        tgattAttribReadReq.p.blob.valueOffset  = valueOffset;      
    }

    gattHandleGATT_ATTRIB_READ_REQ(&tgattAttribReadReq);

    return blueFaceNoError;
}

/**
 * @brief  btapi attribute read request uuid
 *
 * @param  appHandle:
 * @param  bLinkHandle: 
 * @param  startingHandle
 * @param  endingHandle
 * @param  uuid16
 * @param  pUuid128
 *
 * @return
 *
 */
blueFaceStatus btApiGATT_ATTRIB_READ_REQ_UUID(BLINKHANDLE bLinkHandle,
    uint16_t startingHandle, uint16_t endingHandle, uint16_t uuid16, uint8_t *pUuid128)
{
    TGATTAttribReadReq tgattAttribReadReq;
    LPbtLink pLink = bLinkHandle;

    tgattAttribReadReq.cid = pLink->handle;
    tgattAttribReadReq.type = GATT_READ_TYPE_TYPE;
    tgattAttribReadReq.p.type.startingHandle = startingHandle;
    tgattAttribReadReq.p.type.endingHandle   = endingHandle;
    if ( uuid16 != 0 )
    {
        tgattAttribReadReq.p.type.uuidType = UUID_TYPE_16;
        tgattAttribReadReq.p.type.uuid.uuid16 = uuid16;
    }
    else if ( pUuid128 != NULL )
    {      
        tgattAttribReadReq.p.type.uuidType = UUID_TYPE_128;
        tgattAttribReadReq.p.type.uuid.pUuid128 = pUuid128; 
    }

    gattHandleGATT_ATTRIB_READ_REQ(&tgattAttribReadReq);

    return blueFaceNoError;
}

/**
 * @brief  Client write attribute value(s)       
 *
 * @param  appHandle
 * @param  bLinkHandle: 
 * @param  type
 * @param  handle
 * @param  length
 * @param  writeOffset
 * @param  offset
 * @param  pBuffer
 *
 * @return
 *
 */
blueFaceStatus btApiGATT_ATTRIB_WRITE_REQ(BLINKHANDLE bLinkHandle, uint16_t type,
    uint16_t handle, uint16_t length, uint16_t writeOffset, uint16_t offset, uint8_t *pBuffer)
{
    PGATTAttribWriteReqParam pParam;
    LPbtLink pLink = bLinkHandle;

#if CHECK_API_PARAM
    /* check for correct link descriptor */
    if ( !btApiGATT_LinkValid(app, pLink) )
    {
        return( blueFaceIllParameter);
    }
#endif
    pParam = (PGATTAttribWriteReqParam)pBuffer;

    pParam->offset = offset - (offsetof(TGATTAttribWriteReqParam, data));
    pParam->writeOffset = writeOffset;

    gattHandleGATT_ATTRIB_WRITE_REQ(pLink->handle, type, handle, length, pParam);

    return blueFaceNoError;   /* msg buffer will not be released by callers */
}

/**
 * @brief  btapi le scan request
 *
 * @param  enable
 * @param  scanType
 * @param  scanInterval
 * @param  scanWindow
 * @param  filterPolicy
 * @param  filterDuplicates
 *
 * @return  
 *
 */
blueFaceStatus btApiGATT_LE_SCAN_REQ(uint8_t enable, uint8_t scanType, uint16_t scanInterval,
    uint16_t scanWindow, uint8_t filterPolicy, uint8_t localBdType, uint8_t filterDuplicates)
{
    TGATTLEScanReq TScanParam;

    TScanParam.enable            = enable;
    TScanParam.scanType          = scanType;
    TScanParam.scanInterval      = scanInterval;
    TScanParam.scanWindow        = scanWindow;
    TScanParam.filterPolicy      = filterPolicy;
    TScanParam.localBdType       = localBdType;
    TScanParam.filterDuplicates  = filterDuplicates;

    gattHandleGATT_LE_SCAN_REQ(&TScanParam);

    return blueFaceNoError;
}

/**
 * @brief  btapi le connection update request
 *
 * @param  bLinkHandle
 * @param  connIntervalMin
 * @param  connIntervalMax
 * @param  connLatency
 * @param  supervisionTimeout
 * @param  minimumCELength
 * @param  maximumCELength
 *
 * @return  
 *
 */
blueFaceStatus btApiGATT_LE_CONNECTION_UPDATE_REQ(BLINKHANDLE bLinkHandle,
    uint16_t connIntervalMin, uint16_t connIntervalMax, uint16_t connLatency,
    uint16_t supervisionTimeout, uint16_t minimumCELength, uint16_t maximumCELength)
{
    LPbtLink pLink = bLinkHandle;
    TGATTLEConnectionUpdateParam tUpdateReqPram;

    tUpdateReqPram.connIntervalMin = connIntervalMin;
    tUpdateReqPram.connIntervalMax = connIntervalMax;
    tUpdateReqPram.connLatency = connLatency;
    tUpdateReqPram.supervisionTimeout = supervisionTimeout;
    tUpdateReqPram.minimumCELength = minimumCELength;
    tUpdateReqPram.maximumCELength = maximumCELength;

    gattHandleGATT_LE_CONNECTION_UPDATE_REQ(pLink->handle, &tUpdateReqPram);

    return blueFaceNoError;
}

blueFaceStatus btApiGATT_SET_RANDOM_ADDRESS_REQ(uint8_t * bd)
{
    btsmHandleLE_SET_RANDOM_ADDRESS(bd);
    hciCommandLESetRandomAddressCommand(bd);
    hciLaterEntry();
    return blueFaceNoError;
}
