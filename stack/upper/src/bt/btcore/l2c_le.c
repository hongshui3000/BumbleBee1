/**
*****************************************************************
*	Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************
* @file       l2c_le.c
* @brief     Bluetooth L2CAP Layer (Low Energy)
* @details   
*
* @author   	gordon
* @date      	2015-07-09
* @version	v0.1
*/

#include <flags.h>
#include <os_message.h>
#include <os_pool.h>
#include <os_timer.h>
#include <message.h>
#include <l2c.h>
#include <l2c_api.h>
#include <l2c_le.h>
#include <hci_api.h>
#include <bt_msg.h>
#include <gatt_api.h>
#include <hci_code.h>
#include <btsm_api.h>
#include <upper_stack_global.h>
#include <swtimer.h>
#include <blueapi_api.h>

#define TRACE_MODULE_ID     MID_BT_L2C

#if F_BT_LE_BT41_SUPPORT
void l2cFragmentLEData(P_L2CAP_CHANNEL pChan);
#endif
/**
* @brief   Search channel (LCid)
*
* @param  LCid
*
* @return  
*
*/
BOOL l2cLESearchLCid(uint16_t LCid)
{
	switch (LCid)
	{
	case CID_ATTRIBUTE_PROTOCOL:
	case CID_SECURITY_MANAGER:
	case CID_SIGNALING_LE:
		return TRUE;

	default:
		/* support only GATT and SecurityManager */
		DebuggerBreak();
		break;
	}

	return(FALSE);
}

/**
* @brief   Check connection update parameter
*
* @param  pParam
*
* @return  
*
*/
uint16_t l2cConnectionUpdateParametersCheck(PGATTLEConnectionUpdateParam pParam)
{
    if ((pParam->connIntervalMin >= 6 && pParam->connIntervalMin <= 3200) &&
        (pParam->connIntervalMax >= 6 && pParam->connIntervalMax <= 3200) &&
        (pParam->connIntervalMax >= pParam->connIntervalMin) &&
        (pParam->connLatency < 500) &&
        (pParam->supervisionTimeout >= 10 && pParam->supervisionTimeout <= 3200))
    {
        return(L2CAP_NO_CAUSE);                          /* ok. */
    }
    else
    {
        return(L2CAP_ERR | L2CAP_ERR_ILLEGAL_PARAMETER); /* wrong parameter */
    }
}

/**
* @brief  l2cap send connection param update response
*
* @param  pChan
* @param  pHciDesc
* @param  result
*
* @return  
*
*/
void l2cSendL2CAP_CONNECTION_PARAMETER_UPDATE_RESPONSE(P_L2CAP_CHANNEL pChan, P_ACLPOOLDESC pHciDesc, uint16_t result)
{
	uint8_t ResultBuf[2];

	SHORT2CHAR(ResultBuf, result);

	L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_TRACE,
	                        "L2C<- L2CAP_CONNECTION_PARAMETER_UPDATE_RESPONSE result %X", result);
	l2cSendL2CAPMessage(pChan,  pHciDesc, L2CAP_CONNECTION_PARAMETER_UPDATE_RESPONSE, ResultBuf, sizeof(ResultBuf), 0 );
}

/**
* @brief  Handle L2CAP_CONNECTION_PARAMETER_UPDATE_REQUEST     
* 
* @param  pHciDesc
* @param  pPar
*
* @return  
*
*/
void l2cLEHandleL2CAP_CONNECTION_PARAMETER_UPDATE_REQUEST(P_ACLPOOLDESC pHciDesc, uint8_t * pPar)
{
	TGATTLEConnectionUpdateParam tParam;

	tParam.connIntervalMin	= CHAR2SHORT(pPar);
	tParam.connIntervalMax	= CHAR2SHORT(pPar + 2);
	tParam.connLatency		= CHAR2SHORT(pPar + 4);
	tParam.supervisionTimeout = CHAR2SHORT(pPar + 6);
    tParam.minimumCELength = 0;
    tParam.maximumCELength = 0;
	
	if (l2cConnectionUpdateParametersCheck(&tParam))
	{ /* reject parameters */
		l2cSendL2CAP_CONNECTION_PARAMETER_UPDATE_RESPONSE(NULL, pHciDesc, LE_CONNECTION_UPDATE_REJECT);
	}
	else	
	{ /* send upstream and wait for response */
        gattHandleL2CAP_CONNECTION_PARAMETER_UPDATE_REQUEST(pHciDesc->handle, &tParam);
	}
}

/**
* @brief  l2cap handle connection parameter update request
* 
* @param  hciDesc
* @param  pPar
*
* @return  
*
*/
void l2cHandleL2CAP_CONNECTION_PARAMETER_UPDATE_REQUEST(P_ACLPOOLDESC pHciDesc, uint8_t * pPar)
{
    L2CAP_TRACE_PRINTF_4(L2CAP_TRACE_MASK_TRACE,
                            "L2C-> L2CAP L2CAP_CONNECTION_PARAMETER_UPDATE_REQUEST interval min %d max %d slave latency %d timeout %d",
                            CHAR2SHORT(pPar + 0), CHAR2SHORT(pPar + 2), CHAR2SHORT(pPar + 4), CHAR2SHORT(pPar + 6));

    if (pHciDesc->role != LE_ROLE_MASTER)
    {
        l2cSendL2CAP_COMMAND_REJECT(NULL, pHciDesc, L2CAP_CMDREJ_NOT_UNDERSTOOD, 0, 0);
        return;
    }

    l2cLEHandleL2CAP_CONNECTION_PARAMETER_UPDATE_REQUEST(pHciDesc, pPar);
}


/**
* @brief  l2cap handle connection parameter update response
* 
* @param  hciDesc
* @param  pPar
*
* @return  
*
*/
void l2cHandleL2CAP_CONNECTION_PARAMETER_UPDATE_RESPONSE(P_ACLPOOLDESC pHciDesc, uint8_t * pPar)
{
    L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_TRACE,
                            "L2C-> L2CAP L2CAP_CONNECTION_PARAMETER_UPDATE_RESPONSE result %d",
                            CHAR2SHORT(pPar + 0));

    if (pHciDesc->role == LE_ROLE_SLAVE)
    {
        l2cLEStopRTXTimeout(pHciDesc);
        gattHandleLEConnectionUpdateConf(pHciDesc->handle,(CHAR2SHORT(pPar)) ? (HCI_ERR | HCI_ERR_UNACCEPTABLE_CONNECTION_INTERVAL)
	                                       : HCI_SUCCESS );
    }

    pHciDesc->SigIdSent   = 0;
}




/**
* @brief   l2cap send le data indicate
*
* @param  pHciDesc
* @param  LCid
* @param  pMsg
*
* @return  
*
*/
void l2cLESendDataInd(P_ACLPOOLDESC pHciDesc, uint16_t LCid, MESSAGE_P pMsg)
{
	LPLEMessage pLEMsg;
  	BOOL ReleaseBuffer = TRUE;

	switch (LCid)
	{
	case CID_ATTRIBUTE_PROTOCOL:
	case CID_SECURITY_MANAGER:
		break;

	default:
		DebuggerBreak();
		if (pMsg->MData.DataCB.Flag & DATA_CB_RELEASE)
		{
			osBufferRelease((PVOID)pMsg->MData.DataCB.BufferAddress);
		}
		return;

	}
	pLEMsg = (LPLEMessage)(pMsg->MData.DataCB.BufferAddress +
	                 pMsg->MData.DataCB.Offset - offsetof(TLEMessage,p));
	pLEMsg->msgType = LE_DATA;
	pLEMsg->handle  = pHciDesc->handle;
	pLEMsg->channel = LCid;
	pLEMsg->status  = L2CAP_NO_CAUSE;

	if (pLEMsg->channel == CID_SECURITY_MANAGER)
	{
		btsmHandleSMPMessage(pLEMsg, pMsg->MData.DataCB.Length + offsetof(TLEMessage,p));
	}
    else
	{
		gattHandleAttData(pLEMsg->handle, pMsg, FALSE);
        ReleaseBuffer = FALSE;
	}

	if(ReleaseBuffer)
	{
		osBufferRelease(pMsg->MData.DataCB.BufferAddress);
	}
}

/**
* @brief  gatt handle le data request
*
* @param  pmsg
*
* @return  
*
*/
uint16_t l2cLEHandleDataReq(MESSAGE_P pmsg, uint16_t channel, uint16_t handle)
{
	uint16_t        RCid;
    P_ACLPOOLDESC pHciDesc = NULL;

	switch (channel)
	{
	case CID_ATTRIBUTE_PROTOCOL:
	case CID_SECURITY_MANAGER:
		RCid = channel;
		break;

	default:
		if (pmsg->MData.DataCB.Flag & DATA_CB_RELEASE)
		{
			osBufferRelease((PVOID)pmsg->MData.DataCB.BufferAddress);
		}
		/* support only GATT and SecurityManager */
		DebuggerBreak();
		return(L2CAP_ERR | L2CAP_ERR_ILLEGAL_PARAMETER);
	}
    if (FALSE == gL2cDsFragmentationSupport)
    {
	    l2cSendL2CAP_PDU(handle, pmsg,
	                     pmsg->MData.DataCB.Length,
	                     RCid,
	                     TRUE);
    }
    else
    {
        pHciDesc = l2cSearchHciDescByHciHandle(handle);
        if (pHciDesc != NULL)
        {
            pmsg->MData.DataCBChan.Channel = RCid;
            osMessageSend( pHciDesc->dsBufQueue, pmsg);
            l2cFragmentDATA_REQ(pHciDesc);
        }
        else
        {
            L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_ERROR,
                                 "!!l2cLEHandleDataReq pHciDesc is NULL ");
            DebuggerBreak();
        }
    }

	return(L2CAP_NO_CAUSE);
}

/**
* @brief  Handle l2cap connection update response 
* 
* @param  handle
* @param  pParam
* @param  status
*
* @return  
*
*/
void l2cLEHandleConnectionUpdateResp(uint16_t handle, PGATTLEConnectionUpdateParam pParam, uint8_t status)
{
	P_ACLPOOLDESC pHciDesc;
	pHciDesc = l2cSearchHciDescByHciHandle(handle);		//randy--615

	if ((pHciDesc != (P_ACLPOOLDESC)0) && (pHciDesc->role == LE_ROLE_MASTER))
	{
		l2cSendL2CAP_CONNECTION_PARAMETER_UPDATE_RESPONSE(NULL, pHciDesc, status);

		if (status == LE_CONNECTION_UPDATE_ACCEPT)
		{   
			if (!l2cConnectionUpdateParametersCheck(pParam))	  	
			{
                hciCommandLEConnectionUpdate(handle, pParam->connIntervalMin, 
                                         pParam->connIntervalMax, pParam->connLatency, pParam->supervisionTimeout,
	                                     pParam->minimumCELength, pParam->maximumCELength);
				hciLaterEntry();
				return; /* do not release buffer */
			}
		}
	}

	return;
}

/**
* @brief  send l2cap connect update request msg
*
* @param  pL2CChannel
* @param  pParam
*
* @return  
*
*/
uint16_t l2cLEHandleConnectionUpdateReq(uint16_t handle, PGATTLEConnectionUpdateParam pParam)
{
	P_ACLPOOLDESC pHciDesc;
    pHciDesc = l2cSearchHciDescByHciHandle(handle);

	if ((pHciDesc == (P_ACLPOOLDESC)0) || (pHciDesc->handle == 0x0000))
	{
        return (L2CAP_ERR | L2CAP_ERR_ILLEGAL_PARAMETER);
	}
	else
	{
		uint16_t Status;

		Status = l2cConnectionUpdateParametersCheck(pParam);
		if (Status)
		{
			return(Status);
		}

		if (pHciDesc->role == LE_ROLE_MASTER)
		{
            hciCommandLEConnectionUpdate(handle, pParam->connIntervalMin, 
                                         pParam->connIntervalMax, pParam->connLatency, pParam->supervisionTimeout,
	                                     pParam->minimumCELength, pParam->maximumCELength);
			hciLaterEntry();		//randy ---615
		}
		else
		{
			if (pHciDesc->SigIdSent == 0)    /* no response pending */
			{
                uint8_t mbuf[HCI_MAX_SHORT_CMD_SIZE];
		        uint16_t pos = 0;
				SHORT2CHAR(mbuf+pos, pParam->connIntervalMin); pos += 2;
				SHORT2CHAR(mbuf+pos, pParam->connIntervalMax); pos += 2;
				SHORT2CHAR(mbuf+pos, pParam->connLatency); pos += 2;
				SHORT2CHAR(mbuf+pos, pParam->supervisionTimeout); pos += 2;
				l2cSendL2CAPMessage(NULL, pHciDesc, L2CAP_CONNECTION_PARAMETER_UPDATE_REQUEST, mbuf, pos, 0 );	
			}
			else	/* waiting for response */
			{
				return(L2CAP_ERR | L2CAP_ERR_PENDING);
			}
			hciLaterEntry();		//randy--615	 
		}
	}
    return(L2CAP_NO_CAUSE);
}

/**
* @brief  handle le connection complete event
*
* @param  Handle
* @param  role
* @param  Status
* @param  peerAddress
*
* @return  
*
*/
void l2cLEHandleHCI_LE_CONNECTION_COMPLETE_EVENT(uint16_t Handle, uint8_t role, uint16_t Status, uint8_t * peerAddress)
{
    P_ACLPOOLDESC pHciDesc;

    if (Status == 0)
    {
        pHciDesc = l2cGetHciDesc((LPCBdAddr)peerAddress, BT_CONNECTION_TYPE_LE);
        if (pHciDesc == (P_ACLPOOLDESC)0) /* no resource */
        {
            hciCommandDisconnect(Handle, HCI_ERR_OTHER_END_TERMINATE_14);
            hciLaterEntry();        //randy----618
            return;
        }

        pHciDesc->handle = Handle;
        pHciDesc->role	 = role;
    }
}

/**
* @brief  l2cap handle le disconnect indicate
*
* @param  handle
*
* @return  
*
*/
void l2cLEHandleHCI_DISCONNECTION_COMPLETE(uint16_t handle)
{
	P_ACLPOOLDESC pHciDesc;

#if F_BT_LE_BT41_SUPPORT
	P_L2CAP_CHANNEL pChan;

    while((pChan = l2cSearchHciHandle(handle)) != NULL)	/* there are still channels on this HCI handle */
	{    
		switch (pChan->State)
		{
	    case l2cStateWaitForL2C_DISC_RESP:
	        l2cDeleteChannel(pChan, TRUE);
	        break;

	    default:
	        l2cStopRTXTimeout(pChan);
            l2cSendL2C_DISC_IND(pChan, 0);
			l2cChangeState(pChan, l2cStateClosed);
	        break;
		}
	}
#endif

	pHciDesc = l2cSearchHciDescByHciHandle(handle);
	if (pHciDesc != (P_ACLPOOLDESC)0)
	{
		if (pHciDesc->SigIdSent != 0)
		{
		    l2cLEStopRTXTimeout(pHciDesc);
		    pHciDesc->SigIdSent = 0;
		}
		l2cFreeHciDesc(pHciDesc);
	}
}

/**
* @brief  Handle RTX timeout
*
* @param  pHciDesc
*
* @return  
*
*/
void l2cLEStartRTXTimeout(P_ACLPOOLDESC pHciDesc, uint16_t secs)
{
    if (pHciDesc->RTXTimerHandle)
    {
        osDeleteTimer(&(pHciDesc->RTXTimerHandle));
    }
    osStartTimer(&(pHciDesc->RTXTimerHandle), l2cQueueID, L2CAP_RTX_TID, pHciDesc->SigIdSent, secs*1000, swTimerCallBack);
}

void l2cLEStopRTXTimeout(P_ACLPOOLDESC pHciDesc)
{
	osDeleteTimer(&(pHciDesc->RTXTimerHandle));
}

void l2cLEHandleTimeout(P_ACLPOOLDESC pHciDesc)
{
    uint8_t        pPar[2];
      /* fake response for upper layers */
    SHORT2CHAR(pPar, LE_CONNECTION_UPDATE_REJECT);
    l2cHandleL2CAP_CONNECTION_PARAMETER_UPDATE_RESPONSE(pHciDesc, pPar);  
    
	hciCommandDisconnect(pHciDesc->handle, HCI_ERR_UNACCEPTABLE_CONNECTION_INTERVAL);
	hciLaterEntry();
}
#if F_BT_LE_BT41_SUPPORT
void l2cSend_CreateLEDataChannelRsp(uint16_t channel, uint16_t handle, uint16_t cause)
{
    uint16_t local_MDL_ID = blueAPI_GetMdlID(handle);

    blueAPI_Send_CreateLEDataChannelRsp(local_MDL_ID, channel, cause);
}

void l2cSend_CreateLEDataChannelInd(uint16_t channel, uint16_t handle)
{
    uint16_t local_MDL_ID = blueAPI_GetMdlID(handle);    
    blueAPI_Send_CreateLEDataChannelInd(local_MDL_ID, channel);
}

void l2cSend_DisconnectLEDataChannelInd (uint16_t channel, uint16_t handle, uint16_t cause)
{
    uint16_t local_MDL_ID = blueAPI_GetMdlID(handle);
    blueAPI_Send_DisconnectLEDataChannelInd(local_MDL_ID, channel, cause);
}

void l2cSend_DisconnectLEDataChannelRsp(uint16_t channel, uint16_t handle, uint16_t cause)
{
    uint16_t local_MDL_ID = blueAPI_GetMdlID(handle);    
    blueAPI_Send_DisconnectLEDataChannelRsp(local_MDL_ID, channel, cause);
}


void l2cSend_LEDataRsp(uint16_t handle, uint16_t channel, uint16_t cause)
{
        uint16_t local_MDL_ID = blueAPI_GetMdlID(handle);
    blueAPI_Send_LEDataRsp(local_MDL_ID, channel, cause);
}

void l2cSend_LEDataChannelParameterInfo(uint16_t handle, uint16_t channel, uint16_t mtu, uint16_t mps, uint16_t credits)
{
    uint16_t local_MDL_ID = blueAPI_GetMdlID(handle);
    uint16_t tx_size;

    if(otp_str_data.gEfuse_UpperStack_s.le_data_max_tx_wsize == 0)
	{
		tx_size = 1;
	}
	else
	{
		tx_size = otp_str_data.gEfuse_UpperStack_s.le_data_max_tx_wsize;
	}
    blueAPI_Send_LEDataChannelParameterInfo(local_MDL_ID, channel, mtu, mps, credits, tx_size);
}

void l2cSend_LEDataChannelCreditsAlertInfo(uint16_t handle, uint16_t channel)
{
    uint16_t local_MDL_ID = blueAPI_GetMdlID(handle);
    blueAPI_Send_LEDataChannelCreditsAlertInfo(local_MDL_ID, channel);
}
void l2cSendL2CAP_LE_CREDIT_BASED_CONNECTION_REQUEST(P_L2CAP_CHANNEL pChan)
{
    uint8_t ConReqPara[10];

    SHORT2CHAR(ConReqPara    , pChan->le_psm);
    SHORT2CHAR(ConReqPara + 2, pChan->LocalCid);
    SHORT2CHAR(ConReqPara + 4, pChan->LocalMtu);
    SHORT2CHAR(ConReqPara + 6, pChan->LocalMps);
    SHORT2CHAR(ConReqPara + 8, pChan->LocalInitialCredits);


    L2CAP_TRACE_PRINTF_5(L2CAP_TRACE_MASK_TRACE,
                         "L2C<- L2CAP_LE_CREDIT_BASED_CONNECTION_REQUEST le_psm 0x%x scid 0x%x mtu 0x%x mps 0x%x initialCredits 0x%x",
                         pChan->le_psm, pChan->LocalCid, pChan->LocalMtu, pChan->LocalMps, pChan->LocalInitialCredits);

    l2cSendL2CAPMessage(pChan, NULL, L2CAP_LE_CREDIT_BASED_CONNECTION_REQUEST, ConReqPara, sizeof(ConReqPara), 0 );
}

void l2cSendL2CAP_LE_CREDIT_BASED_CONNECTION_RESPONSE(P_ACLPOOLDESC hciDesc, P_L2CAP_CHANNEL pChan, uint16_t dcid, uint16_t mtu, uint16_t mps,
        uint16_t initialCredits, uint16_t result)
{
    uint8_t ConRespPara[10];

    SHORT2CHAR(ConRespPara    , dcid);
    SHORT2CHAR(ConRespPara + 2, mtu);
    SHORT2CHAR(ConRespPara + 4, mps);
    SHORT2CHAR(ConRespPara + 6, initialCredits);
    SHORT2CHAR(ConRespPara + 8, result);


    L2CAP_TRACE_PRINTF_5(L2CAP_TRACE_MASK_TRACE,
                         "L2C<- L2CAP_LE_CREDIT_BASED_CONNECTION_RESPONSE dcid 0x%x mtu 0x%x mps 0x%x initialCredits 0x%x result 0x%x ",
                         dcid, mtu, mps, initialCredits, result);

    l2cSendL2CAPMessage(pChan, NULL, L2CAP_LE_CREDIT_BASED_CONNECTION_RESPONSE, ConRespPara, sizeof(ConRespPara), 0 );
}

void l2cSendL2CAP_LE_FLOW_CONTROL_CREDIT(P_L2CAP_CHANNEL pChan, uint16_t credits)
{
    uint8_t FlowPara[4];

    L2CAP_TRACE_PRINTF_3(L2CAP_TRACE_MASK_TRACE,
                         "L2C<- L2CAP_LE_FLOW_CONTROL_CREDIT dcid 0x%X scid 0x%X creditd %d",
                         pChan->RemoteCid, pChan->LocalCid, credits);

    SHORT2CHAR(FlowPara,     pChan->LocalCid);
    SHORT2CHAR(FlowPara + 2, credits);

    l2cSendL2CAPMessage(pChan, NULL, L2CAP_LE_FLOW_CONTROL_CREDIT, FlowPara, sizeof(FlowPara), 0 );
}

void l2cHandleL2CAP_LE_CREDIT_BASED_CONNECTION_REQUEST(P_ACLPOOLDESC hciDesc, T_L2CAP_Command *l2cap_cmd, uint8_t * pPar)
{
    P_L2CAP_CHANNEL     pChan;
    T_L2CAP_LE_ConReq_para L2CAP_ConReq;

    L2CAP_ConReq.le_psm = CHAR2SHORT(pPar);
    pPar += 2;
    L2CAP_ConReq.scid = CHAR2SHORT(pPar);
    pPar += 2;
    L2CAP_ConReq.mtu = CHAR2SHORT(pPar);
    pPar += 2;
    L2CAP_ConReq.mps = CHAR2SHORT(pPar);
    pPar += 2;
    L2CAP_ConReq.initialCredits = CHAR2SHORT(pPar);

    hciDesc->ReqIdRcv = l2cap_cmd->id;

    if ((L2CAP_ConReq.le_psm == 0) || (L2CAP_ConReq.le_psm > LE_PSM_MAX))
    {
        L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_ERROR,
                             "!!l2cHandleL2CAP_LE_CREDIT_BASED_CONNECTION_REQUEST(): PSM invalid or unsupported !");
        l2cSendL2CAP_LE_CREDIT_BASED_CONNECTION_RESPONSE(hciDesc, NULL, 0, 0, 0, 0, LE_CREDIT_ERR_LE_PSM_NOT_SUPPORTED);
        return;
    }

    if ((pChan = l2cCheckRcid(hciDesc, L2CAP_ConReq.scid )) == NULL)
    {
        L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_ERROR,
                             "!!l2cHandleL2CAP_LE_CREDIT_BASED_CONNECTION_REQUEST(): couldn't create new channel");

        l2cSendL2CAP_LE_CREDIT_BASED_CONNECTION_RESPONSE(hciDesc, NULL, 0, 0, 0, 0, LE_CREDIT_ERR_NO_RESOURCES_AVAILABLE);
        return;
    }

    pChan->ReqIdRcv = l2cap_cmd->id;
    if (pChan->State == l2cStateClosed)
    {
        pChan->le_psm = L2CAP_ConReq.le_psm;
        pChan->RemoteMtu = L2CAP_ConReq.mtu;
        pChan->RemoteMps = L2CAP_ConReq.mps;
        pChan->RemoteInitialCredits = L2CAP_ConReq.initialCredits;
        pChan->role = terminate;
        l2cSendSECMAN_AUTHENTICATION_IND(pChan,0, 1);
        l2cChangeState(pChan, l2cStateWaitForSECMAN_AUTHENTICATION_RESP);
        return;
    }
    else
    {
        switch (pChan->State)
        {
        case l2cStateOpen:
            l2cSendL2CAP_LE_CREDIT_BASED_CONNECTION_RESPONSE(NULL, pChan, pChan->LocalCid, pChan->LocalMtu,
                    pChan->LocalMps, pChan->LocalInitialCredits, LE_CREDIT_CONNECTION_SUCCESS);
            break;

        case l2cStateWaitForL2C_CON_LE_DATA_CHANNEL_RESP:
        case l2cStateWaitForL2C_DISC_RESP:
        case l2cStateWaitForL2CAP_LE_CREDIT_BASED_CONNECTION_RESPONSE:
        case l2cStateWaitForL2CAP_DISCONNECTION_RESPONSE:
            L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_ERROR,
                                 "!!l2cHandleL2CAP_LE_CREDIT_BASED_CONNECTION_REQUEST(): invalid state %d", pChan->State);

            break;

        default:
            break;
        }
    }
}

void l2cHandleL2CAP_LE_CREDIT_BASED_CONNECTION_RESPONSE(P_ACLPOOLDESC hciDesc, T_L2CAP_Command *l2cap_cmd, uint8_t * pPar)
{
    P_L2CAP_CHANNEL     pChan;
    T_L2CAP_LE_ConResp_para L2CAP_ConResp;
    uint16_t                    cause;

    L2CAP_ConResp.dcid = CHAR2SHORT(pPar);
    pPar += 2;
    L2CAP_ConResp.mtu = CHAR2SHORT(pPar);
    pPar += 2;
    L2CAP_ConResp.mps = CHAR2SHORT(pPar);
    pPar += 2;
    L2CAP_ConResp.initialCredits = CHAR2SHORT(pPar);
    pPar += 2;
    L2CAP_ConResp.result = CHAR2SHORT(pPar);


    if ((pChan = l2cSearchCmdId(l2cap_cmd->id)) == NULL)
    {
        L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_ERROR,
                             "!!l2cHandleSigCmd(): couldn't find l2cap channel");
        return;
    }

    if (L2CAP_ConResp.result != LE_CREDIT_CONNECTION_SUCCESS)
    {
        switch (L2CAP_ConResp.result)
        {
        case LE_CREDIT_ERR_LE_PSM_NOT_SUPPORTED:
            cause = L2CAP_ERR_REFUS_INV_PSM | L2CAP_ERR;
            break;
        case LE_CREDIT_ERR_NO_RESOURCES_AVAILABLE:
            cause = L2CAP_ERR_REFUS_NO_RESOURCE | L2CAP_ERR;
            break;
        case LE_CREDIT_ERR_INSUFFICIENT_AUTHENTICATION:
            cause = L2CAP_ERR_INSUFFICIENT_AUTHENTICATION | L2CAP_ERR;
            break;
        case LE_CREDIT_ERR_INSUFFICIENT_AUTHORIZATION:
            cause = L2CAP_ERR_INSUFFICIENT_AUTHORIZATION | L2CAP_ERR;
            break;
        case LE_CREDIT_ERR_INSUFFICIENT_ENCRYPTION_KEY_SIZE:
            cause = L2CAP_ERR_INSUFFICIENT_KEY_SIZE | L2CAP_ERR;
            break;
        case LE_CREDIT_ERR_INSUFFICIENT_ENCRYPTION :
            cause = L2CAP_ERR_INSUFFICIENT_ENCRYPTION | L2CAP_ERR;
            break;
		case LE_CREDIT_ERR_INVAlID_SOURCE_CID :
            cause = L2CAP_ERR_INVAlID_SOURCE_CID | L2CAP_ERR;
            break;
		case LE_CREDIT_ERR_SOURCE_CID_ALREADY_ALLOCATED :
            cause = L2CAP_ERR_SOURCE_CID_ALREADY_ALLOCATED | L2CAP_ERR;
            break;
        default:
            cause = L2CAP_ERR_OUT_OF_RANGE | L2CAP_ERR;
            break;
        }
        l2cSend_CreateLEDataChannelRsp(0, hciDesc->handle,
                                            cause);
        l2cChangeState( pChan, l2cStateClosed);
        return;
    }
    pChan->RemoteCid = L2CAP_ConResp.dcid;
    pChan->RespIdRcv = l2cap_cmd->id;

    switch (pChan->State)
    {
    case l2cStateWaitForL2CAP_LE_CREDIT_BASED_CONNECTION_RESPONSE:
        pChan->RemoteMtu = L2CAP_ConResp.mtu;
        pChan->RemoteMps = L2CAP_ConResp.mps;
        pChan->RemoteInitialCredits = L2CAP_ConResp.initialCredits;
        l2cSend_CreateLEDataChannelRsp(pChan->LocalCid, hciDesc->handle,
                                            L2CAP_NO_CAUSE);
        l2cChangeState( pChan, l2cStateOpen);
        break;

    default:
        L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_ERROR,
                             "!!l2cHandleL2CAP_LE_CREDIT_BASED_CONNECTION_RESPONSE(): invalid state %d", pChan->State);
        break;
    }
}

void l2cHandleL2CAP_LE_FLOW_CONTROL_CREDIT(P_ACLPOOLDESC hciDesc, T_L2CAP_Command *l2cap_cmd, uint8_t * pPar)
{
    P_L2CAP_CHANNEL     pChan;
    T_L2CAP_LE_FlowCtrol_para LE_FlowCtr;
    unsigned int credit1 = 0;
    unsigned int credit2 = 0;

    LE_FlowCtr.cid = CHAR2SHORT(pPar);
    pPar += 2;
    LE_FlowCtr.credits = CHAR2SHORT(pPar);

    pChan = l2cSearchRcid(hciDesc, LE_FlowCtr.cid);
    if (pChan == NULL)
    {
        L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_TRACE,
                             "l2cHandleL2CAP_LE_FLOW_CONTROL_CREDIT: not find the channel");
    }
    else
    {
        if (pChan->State == l2cStateOpen)
        {
            L2CAP_TRACE_PRINTF_2(L2CAP_TRACE_MASK_TRACE,
                                 "l2cHandleL2CAP_LE_FLOW_CONTROL_CREDIT Credit %d remoteCredits %d",
                                 LE_FlowCtr.credits, pChan->RemoteInitialCredits);
            credit1 = pChan->RemoteInitialCredits;
            credit2 = LE_FlowCtr.credits;
            credit1 = credit1 + credit2;
            if (credit1 > 65535)
            {
                L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_TRACE,
                                     "l2cHandleL2CAP_LE_FLOW_CONTROL_CREDIT: credits exceeds 65535,disconnect the channel");
                l2cSend_DisconnectLEDataChannelInd(pChan->LocalCid, hciDesc->handle , L2CAP_ERR_CREDITS_EXCEED_RANGE | L2CAP_ERR);
                l2cSendL2CAP_DISCONNECTION_REQUEST(pChan);
                l2cChangeState(pChan, l2cStateWaitForL2CAP_DISCONNECTION_RESPONSE);
            }
            else
            {
                pChan->RemoteInitialCredits = credit1;

                l2cFragmentLEData(pChan);
            }
        }
        else
        {
            L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_ERROR,
                                 "!!l2cHandleL2CAP_LE_FLOW_CONTROL_CREDIT: invalid state %d", pChan->State);
        }
    }

}

uint16_t l2cLEHandle_CreateLEDataChannelReq(uint16_t handle, uint16_t le_psm, uint16_t mtu, uint16_t mps,
                                        uint16_t initialCredits, uint16_t creditsIncrease)
{
    P_L2CAP_CHANNEL pNewChan;
    P_ACLPOOLDESC pHciDesc;
    uint16_t cause = L2CAP_NO_CAUSE;

    pHciDesc = l2cSearchHciDescByHciHandle(handle);
    if ( (mps < 23)
         || (mtu < 23)
         || (le_psm == 0)
         || (pHciDesc == NULL)
       )
    {
        cause = L2CAP_ERR_ILLEGAL_PARAMETER | L2CAP_ERR;
    }

    if (cause == L2CAP_NO_CAUSE)
    {
        pNewChan = l2cChannelCreate(0, (LPCBdAddr)pHciDesc->remote_bd, BT_CONNECTION_TYPE_LE);
        if (pNewChan != NULL)
        {
            pNewChan->le_psm = le_psm;
            pNewChan->LocalMtu = mtu;
            pNewChan->LocalMps = mps;
            pNewChan->LocalInitialCredits = initialCredits;
			pNewChan->InitialCredits = initialCredits;
            pNewChan->creditsIncrease = creditsIncrease;
            pNewChan->role = originate;
            l2cSendL2CAP_LE_CREDIT_BASED_CONNECTION_REQUEST(pNewChan);
            pNewChan->State = l2cStateWaitForL2CAP_LE_CREDIT_BASED_CONNECTION_RESPONSE;
            l2cInsertChannel(pNewChan);
        }
        else
        {
            cause = L2CAP_ERR_NO_RESOURCE | L2CAP_ERR;
        }

    }
    return cause;

}

void l2cLEHandle_CreateLEDataChannelConf(uint16_t channel, uint16_t mtu, uint16_t mps, uint16_t initialCredits,
                                         uint16_t creditsIncrease, uint16_t cause)
{
    P_L2CAP_CHANNEL pChan;

    pChan = l2cSearchLcid(channel);
    if (pChan)
    {
        if (cause != L2CAP_NO_CAUSE)
        {
            l2cSendL2CAP_LE_CREDIT_BASED_CONNECTION_RESPONSE(NULL, pChan, 0, 0,
                    0, 0, LE_CREDIT_ERR_NO_RESOURCES_AVAILABLE);
            l2cChangeState(pChan, l2cStateClosed);
            return;
        }
        if (pChan->State == l2cStateWaitForL2C_CON_LE_DATA_CHANNEL_RESP)
        {
            pChan->LocalMtu = mtu;
            pChan->LocalMps = mps;
            pChan->LocalInitialCredits = initialCredits;
			pChan->InitialCredits = initialCredits;
            pChan->creditsIncrease = creditsIncrease;
            l2cSendL2CAP_LE_CREDIT_BASED_CONNECTION_RESPONSE(NULL, pChan, pChan->LocalCid, pChan->LocalMtu,
                    pChan->LocalMps, pChan->LocalInitialCredits, LE_CREDIT_CONNECTION_SUCCESS);
            l2cChangeState(pChan, l2cStateOpen);
        }
        else
        {
            L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_ERROR, "!!l2cHandleL2C_CON_LE_DATA_CHANNEL_RESP: invalid state %d.", pChan->State);
            l2cChangeState(pChan, l2cStateClosed);
        }
    }
    else
    {
        L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_ERROR, "!!l2cHandleL2C_CON_LE_DATA_CHANNEL_RESP:the channel is not exist.");
    }
}

uint16_t l2cLEHandle_DisconnectLEDataChannelReq(uint16_t channel)
{
    P_L2CAP_CHANNEL pChan;
    uint16_t cause = L2CAP_NO_CAUSE;

    pChan = l2cSearchLcid(channel);
    if (pChan != NULL)
    {
        switch (pChan->State)
        {
        case l2cStateOpen:
            l2cSendL2CAP_DISCONNECTION_REQUEST(pChan);
            l2cChangeState(pChan, l2cStateWaitForL2CAP_DISCONNECTION_RESPONSE);
            pChan->WaitDisRsp = TRUE;
            break;
        case l2cStateClosed:
            l2cSend_DisconnectLEDataChannelRsp(channel,
                                                 pChan->pHciDesc->handle, L2CAP_NO_CAUSE);
            break;
        case l2cStateWaitForL2CAP_DISCONNECTION_RESPONSE:
            cause = L2CAP_ERR_BUSY | L2CAP_ERR;
            break;
        default:
            L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_ERROR, "!!l2cHandleL2C_DISC_LE_DATA_CHANNEL_REQ: invalid state %d.", pChan->State);
            l2cChangeState(pChan, l2cStateClosed);
            cause = L2CAP_ERR_ILLEGAL_STATE | L2CAP_ERR;
            break;
        }
    }
    else
    {
        cause = L2CAP_ERR_ILLEGAL_PARAMETER | L2CAP_ERR;
        L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_ERROR, "!!l2cHandleL2C_DISC_LE_DATA_CHANNEL_REQ:failed cause 0x%x.", cause);
    }
    return cause;
}

uint16_t l2cLEHandle_SendLEFlowControlCreditReq(uint16_t channel, uint16_t credits)
{
    P_L2CAP_CHANNEL pChan;
    uint16_t cause = L2CAP_NO_CAUSE;

    pChan = l2cSearchLcid(channel);
    if (pChan == NULL)
    {
        L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_ERROR, "!!l2cHandleL2C_LE_FLOW_CONTROL_CREDIT_REQ:the channel is not exist.");
        cause = L2CAP_ERR_ILLEGAL_PARAMETER | L2CAP_ERR;
        return cause;
    }
    if (pChan->State == l2cStateOpen)
    {
        l2cSendL2CAP_LE_FLOW_CONTROL_CREDIT(pChan, credits);
        pChan->LocalInitialCredits += credits;
    }
    else
    {
        L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_ERROR, "!!l2cHandleL2C_LE_FLOW_CONTROL_CREDIT_REQ:the state is not open");
        cause = L2CAP_ERR_ILLEGAL_STATE | L2CAP_ERR;
    }
    return cause;
}
void l2cLEHandle_LEDataConf(uint16_t channel, uint16_t cause)
{
}

void l2cLEDATACallBack(uint32_t handle)
{
    P_L2CAP_CHANNEL pChan = (P_L2CAP_CHANNEL)handle;


    L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_TRACE,
                         "L2C-> l2cLEDATACallBack() cid 0x%X", pChan->LocalCid);
    l2cFragmentLEData(pChan);
}

void l2cLESduBufferCallback(uint32_t handle)
{
    P_L2CAP_CHANNEL  pChan = (P_L2CAP_CHANNEL)handle;
    L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_TRACE, "l2cLESduBufferCallback channel = 0x%x", pChan->LocalCid);
    if (pChan->State == l2cStateOpen)
    {
        l2cSend_LEDataRsp(pChan->pHciDesc->handle, pChan->LocalCid, L2CAP_NO_CAUSE);
		l2cFragmentLEData(pChan);
    }
	else
	{
		L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_ERROR, "l2cLESduBufferCallback invalid state 0x%x", pChan->State);
	}
}
void l2cFragmentLEData(P_L2CAP_CHANNEL pChan)
{
    uint16_t dsMTU;
    uint16_t bytesToAlloc;
    MESSAGE_T msg;
    uint8_t * myBuf;
    BOOL firstPak;
    uint16_t ACLpayloadSize;
	PL2CTxData pTxData = NULL;

	L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_TRACE, "l2cFragmentLEData");

    if (pChan->RemoteInitialCredits == 0)
    {
        L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_TRACE, "l2cFragmentLEData: RemoteInitialCredits = 0");
        return;
    }

    if (pChan->State != l2cStateOpen)
    {
    	L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_ERROR, "l2cFragmentLEData: channel not OPEN");
        if(pChan->dsMessage.MData.DataCBChan.BufferAddress != NULL)
        {
        	osBufferRelease(pChan->dsMessage.MData.DataCBChan.BufferAddress);
        	pChan->dsMessage.MData.DataCBChan.BufferAddress = NULL;
        }
        return;
    }

    if (pChan->dsMessage.MData.DataCBChan.BufferAddress == NULL)
    {
    	if(pChan->TxDataQueue.Count > 0)
		{
		    L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_TRACE, "l2cFragmentLEData TxDataQueue.Count = %d", pChan->TxDataQueue.Count);
		    pTxData = osQueueOut( &pChan->TxDataQueue );
			if(pTxData != NULL)
			{
				pChan->dsMessage.MData.DataCBChan.BufferAddress = pTxData->pBuffer;
            	pChan->dsMessage.MData.DataCBChan.Offset = pTxData->offset;
            	pChan->dsMessage.MData.DataCBChan.Length = pTxData->length;
            	pChan->dsMessage.MData.DataCBChan.Flag = DATA_CB_RELEASE;
            	pChan->dsMessage.MData.DataCBChan.TotalLength = pTxData->length;
            	pChan->dsMessage.MData.DataCBChan.Channel = pChan->RemoteCid;
				if (osBufferCallBackSet(pChan->dsMessage.MData.DataCBChan.BufferAddress, l2cLESduBufferCallback, (uint32_t)pChan) != 0)
    			{
        			L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_ERROR, "l2cFragmentLEData: osBufferCallBackSet failed");
        			osBufferRelease(pChan->dsMessage.MData.DataCBChan.BufferAddress);
        			pChan->dsMessage.MData.DataCBChan.BufferAddress = NULL;
					osQueueIn( &pChan->TxDataQueueFree, pTxData );
        			return;
    			}
				osQueueIn( &pChan->TxDataQueueFree, pTxData );
			}
		}
		else
			return;
    }

    if (pChan->dsMessage.MData.DataCBChan.TotalLength == pChan->dsMessage.MData.DataCBChan.Length)
    {
        firstPak = TRUE;
    }
    else
    {
        firstPak = FALSE;
    }

    if (firstPak)
    {
        dsMTU = pChan->RemoteMps - L2CAP_SDU_LENGTH_FIELD_SIZE;
        bytesToAlloc   = pChan->RemoteMps + L2C_WRITE_OFFSET;
        ACLpayloadSize = pChan->RemoteMps;
        if (pChan->dsMessage.MData.DataCBChan.TotalLength > dsMTU)
        {
            if (osBufferGetCallBack(DownstreamPoolID, bytesToAlloc, (void  *)&myBuf,
                                    l2cLEDATACallBack, (uint32_t)pChan))
            {
                L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_TRACE,
                                     "l2cFragmentLEData Could not allocate memory %d ", bytesToAlloc);
                return;
            }

            L2CAP_TRACE_PRINTF_3(L2CAP_TRACE_MASK_TRACE,
                                 "L2C-> l2cFragmentLEData first LE-frame on HCI chan 0x%X  RemoteCID 0x%x length %d",
                                 pChan->pHciDesc->handle,
                                 pChan->dsMessage.MData.DataCBChan.Channel,
                                 dsMTU);

            msg.MData.DataCB.BufferAddress = myBuf;
            msg.MData.DataCB.Offset        = L2C_WRITE_OFFSET;
            msg.MData.DataCB.Flag          = pChan->dsMessage.MData.DataCB.Flag | DATA_CB_RELEASE;
            msg.MData.DataCB.Length        = ACLpayloadSize;
            SHORT2CHAR(msg.MData.DataCB.BufferAddress + msg.MData.DataCB.Offset,
                       pChan->dsMessage.MData.DataCBChan.TotalLength);      /* write 16 bits SDU length */

            memcpy(myBuf + L2C_WRITE_OFFSET + L2CAP_SDU_LENGTH_FIELD_SIZE,
                   pChan->dsMessage.MData.DataCB.BufferAddress + pChan->dsMessage.MData.DataCB.Offset, dsMTU);

            if (0)
            {
                l2cSendL2CAP_PDU(pChan->pHciDesc->handle, &msg,
                                 msg.MData.DataCB.Length,
                                 pChan->RemoteCid,
                                 TRUE);
            }
            else
            {
                msg.MData.DataCBChan.Channel = pChan->RemoteCid;
                osMessageSend(pChan->pHciDesc->dsBufQueue, &msg);
                l2cFragmentDATA_REQ(pChan->pHciDesc);
            }

            pChan->dsMessage.MData.DataCB.Offset += dsMTU;
            pChan->dsMessage.MData.DataCB.Length -= dsMTU;
        }
        else
        {
            pChan->dsMessage.MData.DataCBChan.Offset -= L2CAP_SDU_LENGTH_FIELD_SIZE;
            pChan->dsMessage.MData.DataCBChan.Length += L2CAP_SDU_LENGTH_FIELD_SIZE;

            SHORT2CHAR(pChan->dsMessage.MData.DataCBChan.BufferAddress + pChan->dsMessage.MData.DataCBChan.Offset,
                       pChan->dsMessage.MData.DataCBChan.TotalLength);      /* write 16 bits SDU length */
            if (0)
            {
                l2cSendL2CAP_PDU(pChan->pHciDesc->handle, &pChan->dsMessage,
                                 pChan->dsMessage.MData.DataCBChan.Length,
                                 pChan->RemoteCid,
                                 TRUE);
            }
            else
            {
                osMessageSend(pChan->pHciDesc->dsBufQueue, &pChan->dsMessage);
                l2cFragmentDATA_REQ(pChan->pHciDesc);
            }
            pChan->dsMessage.MData.DataCBChan.BufferAddress = NULL;
        }
        pChan->RemoteInitialCredits--;
        return;
    }

    dsMTU = pChan->RemoteMps;
    bytesToAlloc   = dsMTU + L2C_WRITE_OFFSET;
    ACLpayloadSize = dsMTU;

    if (pChan->dsMessage.MData.DataCB.Length > ACLpayloadSize)  /* needs fragmentation */
    {
        if (osBufferGetCallBack(DownstreamPoolID, bytesToAlloc, (void  *)&myBuf,
                                l2cLEDATACallBack, (uint32_t)pChan))
        {
            L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_TRACE,
                                 "l2cFragmentLEData Could not allocate memory %d retrying", bytesToAlloc);
            return;
        }

        L2CAP_TRACE_PRINTF_3(L2CAP_TRACE_MASK_TRACE,
                             "L2C-> l2cFragmentLEData subsequent LE-frame on HCI chan 0x%X  RemoteCID 0x%x length %d",
                             pChan->pHciDesc->handle,
                             pChan->dsMessage.MData.DataCBChan.Channel,
                             ACLpayloadSize);

        msg.MData.DataCB.BufferAddress = myBuf;
        msg.MData.DataCB.Offset        = L2C_WRITE_OFFSET;
        msg.MData.DataCB.Flag          = pChan->dsMessage.MData.DataCB.Flag | DATA_CB_RELEASE;
        msg.MData.DataCB.Length        = ACLpayloadSize;

        memcpy(myBuf + L2C_WRITE_OFFSET,
               pChan->dsMessage.MData.DataCB.BufferAddress + pChan->dsMessage.MData.DataCB.Offset, ACLpayloadSize);
        if (0)
        {
            l2cSendL2CAP_PDU(pChan->pHciDesc->handle, &msg,
                             msg.MData.DataCB.Length,
                             pChan->RemoteCid,
                             TRUE);
        }
        else
        {
        	msg.MData.DataCBChan.Channel = pChan->RemoteCid;
            osMessageSend(pChan->pHciDesc->dsBufQueue, &msg);
            l2cFragmentDATA_REQ(pChan->pHciDesc);
        }

        pChan->dsMessage.MData.DataCB.Offset += ACLpayloadSize;
        pChan->dsMessage.MData.DataCB.Length -= ACLpayloadSize;
    }
    else
    {
        L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_TRACE,
                             "L2C-> l2cFragmentLEData last LE-frame  length %d",
                             pChan->dsMessage.MData.DataCB.Length);
        if (0)
        {
            l2cSendL2CAP_PDU(pChan->pHciDesc->handle, &pChan->dsMessage,
                             pChan->dsMessage.MData.DataCB.Length,
                             pChan->RemoteCid,
                             TRUE);
        }
        else
        {
			osMessageSend(pChan->pHciDesc->dsBufQueue, &pChan->dsMessage);
            l2cFragmentDATA_REQ(pChan->pHciDesc);
        }
        pChan->dsMessage.MData.DataCBChan.BufferAddress = NULL;
    }
    pChan->RemoteInitialCredits--;
}
 


void l2cHandleDownstreamLEFrame(P_L2CAP_CHANNEL pChan, uint8_t * pBuffer, uint16_t offset, uint16_t valueLength)
{
    L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_TRACE,"l2cHandleDownstreamLEFrame");
    pChan->dsMessage.MData.DataCBChan.BufferAddress = pBuffer;
    pChan->dsMessage.MData.DataCBChan.Offset = offset;
    pChan->dsMessage.MData.DataCBChan.Length = valueLength;
    pChan->dsMessage.MData.DataCBChan.Flag = DATA_CB_RELEASE;
    pChan->dsMessage.MData.DataCBChan.TotalLength = valueLength;
    pChan->dsMessage.MData.DataCBChan.Channel = pChan->RemoteCid;

    if (osBufferCallBackSet(pChan->dsMessage.MData.DataCBChan.BufferAddress, l2cLESduBufferCallback, (uint32_t)pChan) != 0)
    {
        L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_ERROR, "l2cHandleDownstreamLEFrame: osBufferCallBackSet failed");
        osBufferRelease(pChan->dsMessage.MData.DataCBChan.BufferAddress);
        pChan->dsMessage.MData.DataCBChan.BufferAddress = NULL;
        return;
    }

    l2cFragmentLEData(pChan);
}

uint16_t l2cLEHandle_LEDataReq(uint16_t channel, uint8_t * pBuffer, uint16_t offset, uint16_t valueLength)
{
    P_L2CAP_CHANNEL pChan;
    uint16_t status = L2CAP_NO_CAUSE;
	PL2CTxData pTxData = NULL;


    L2CAP_TRACE_PRINTF_2(L2CAP_TRACE_MASK_TRACE,
                         "L2C-> l2cHandle_L2C_LE_DATA_REQ() handle 0x%X length %d",
                         channel, valueLength);


    pChan = l2cSearchLcid(channel);

    if (pChan != NULL)
    {
        if (valueLength > (pChan->RemoteMtu))
        {
            status = L2CAP_ERR_INVAILD_PDU | L2CAP_ERR;
            return status;
        }

        if (pChan->State == l2cStateOpen)
        {
            if ((pChan->dsMessage.MData.DataCBChan.BufferAddress != NULL) || (pChan->TxDataQueue.Count > 0))
            {
                L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_TRACE,
                                     "L2C-> l2cHandleL2C_LE_DATA_REQ() pak in progress");
				pTxData = osQueueOut( &pChan->TxDataQueueFree );
                if ( pTxData != NULL )
                {
                    pTxData->pBuffer = pBuffer;
					pTxData->offset = offset;
					pTxData->length = valueLength;
                    
					osQueueIn( &pChan->TxDataQueue, pTxData );
                }
                else
                {
                    L2CAP_TRACE_PRINTF_0(GATT_TRACE_MASK_ERROR,
                            "!!l2cHandleL2C_LE_DATA_REQ:  Queue is full");
                   	status = L2CAP_ERR_QUEUE_IS_FULL | L2CAP_ERR;
                }
            }
            else
            {
                l2cHandleDownstreamLEFrame(pChan, pBuffer, offset, valueLength);
            }
        }
        else
        {
            L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_ERROR,
                                 "!!L2C handle_L2C_LE_DATA_REQ channel not OPEN!");
            status = L2CAP_ERR_LE_CHANNEL_NOT_OPEN | L2CAP_ERR;
        }
    }
    else
    {
        /* no channel exists */
        L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_ERROR,
                             "!!L2C handle_L2C_LE_DATA_REQ no channel discarding data!");
        status = L2CAP_ERR_ILLEGAL_PARAMETER | L2CAP_ERR;
    }

    return status;
}

void l2cCloseLEChannel(P_L2CAP_CHANNEL pChan, uint16_t cause)
{
	L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_TRACE, "l2cCloseChannel(): cid 0x%x ", pChan->LocalCid);

    switch (pChan->State)
    {
    case l2cStateClosed:
        break;
    case l2cStateOpen:
        {
            l2cSend_DisconnectLEDataChannelInd(pChan->LocalCid, pChan->pHciDesc->handle, cause);
            l2cSendL2CAP_DISCONNECTION_REQUEST(pChan);
            l2cChangeState(pChan, l2cStateWaitForL2CAP_DISCONNECTION_RESPONSE);
        }
        break;

    default:
        l2cChangeState(pChan, l2cStateClosed);
        break;
    }
}

void l2cHandleUpstreamLEFrame(P_L2CAP_CHANNEL pChan, MESSAGE_P pMsg)
{
    uint16_t   BufferLength;
    uint8_t *    pBuffer;
    uint16_t   length;

    L2CAP_TRACE_PRINTF_2(L2CAP_TRACE_MASK_TRACE,
                         "l2cHandleUpstreamLEFrame:cid = 0x%x credits = %d", pChan->LocalCid, pChan->LocalInitialCredits);

    if (pChan->LocalInitialCredits == 0)
    {
        l2cCloseLEChannel(pChan, L2CAP_ERR_CREDITS_LACK | L2CAP_ERR);
        return;
    }
    pChan->LocalInitialCredits--;
    if (pChan->usLEFrames.pd == NULL)
    {
        int sduLength;

        sduLength = CHAR2SHORT(pMsg->MData.DataCBChan.BufferAddress + pMsg->MData.DataCBChan.Offset);
        if (sduLength > pChan->LocalMtu)
        {
            L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_ERROR, "SDU length field value exceeds the receiver's MTU, sdu length = %d", sduLength);
            l2cCloseLEChannel(pChan, L2CAP_ERR_INVAILD_PDU | L2CAP_ERR);
            return;
        }
        else
        {
            pMsg->MData.DataCBChan.Offset += L2CAP_SDU_LENGTH_FIELD_SIZE;
            BufferLength = L2CAP_US_WRITE_OFFSET + sduLength;
            if (osBufferGet(UpstreamPoolID, BufferLength, (PVOID *)&pBuffer))
            {
                L2CAP_TRACE_PRINTF_1(HCI_TRACE_MASK_ERROR, "!!l2cHandleUpstreamLEFrame:buffer get failed. no mem %d ", BufferLength);
                l2cCloseLEChannel(pChan, L2CAP_ERR_NO_RESOURCE | L2CAP_ERR);
                return ;
            }

            pChan->usLEFrames.expected = sduLength;
            pChan->usLEFrames.len = pMsg->MData.DataCBChan.Length - L2CAP_SDU_LENGTH_FIELD_SIZE;
            pChan->usLEFrames.offset = L2CAP_US_WRITE_OFFSET;
            pChan->usLEFrames.pd = pBuffer;
            memcpy((pChan->usLEFrames.pd + pChan->usLEFrames.offset),
                   (pMsg->MData.DataCBChan.BufferAddress + pMsg->MData.DataCBChan.Offset),
                   pChan->usLEFrames.len);
        }
    }
    else
    {
        length = pMsg->MData.DataCBChan.Length;
        if ((pChan->usLEFrames.len + length) > pChan->usLEFrames.expected)
        {
            L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_ERROR,
                                 "the sum of the payload lengths for the LE-frames exceeds the specified SDU length");
            l2cCloseLEChannel(pChan, L2CAP_ERR_INVAILD_PDU | L2CAP_ERR);
            return;
        }
        else
        {
            memcpy((pChan->usLEFrames.pd + pChan->usLEFrames.len + pChan->usLEFrames.offset),
                   (pMsg->MData.DataCBChan.BufferAddress + pMsg->MData.DataCBChan.Offset),
                   length);
            pChan->usLEFrames.len += length;
        }
    }

    if (pChan->usLEFrames.len == pChan->usLEFrames.expected)
    {
        uint16_t local_MDL_ID = blueAPI_GetMdlID(pChan->pHciDesc->handle);
        blueAPI_Send_LEDataInd(pChan->usLEFrames.pd, pChan->usLEFrames.offset,
                               pChan->LocalCid, pChan->usLEFrames.len, local_MDL_ID);

        pChan->usLEFrames.pd = NULL;
        pChan->usLEFrames.expected = pChan->usLEFrames.len = 0;       
    }

	if(pChan->creditsIncrease != 0)
	{
		if(pChan->LocalInitialCredits == pChan->creditsIncrease)
		{
			l2cSendL2CAP_LE_FLOW_CONTROL_CREDIT(pChan, (pChan->InitialCredits - pChan->LocalInitialCredits));
			pChan->LocalInitialCredits = pChan->InitialCredits;
		}
	}
	else
	{
		if (pChan->LocalInitialCredits == 0) 
    	{
    		l2cSend_LEDataChannelCreditsAlertInfo(pChan->pHciDesc->handle, pChan->LocalCid);
    	}
	}

}

#endif
