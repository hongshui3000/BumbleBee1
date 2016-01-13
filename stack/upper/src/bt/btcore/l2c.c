/**
*****************************************************************
*	Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************
* @file       l2c.c
* @brief     Bluetooth L2CAP Layer
* @details   
*
* @author   	gordon
* @date      	2015-07-09
* @version	v0.1
*/

#include <flags.h>
#include <btcommon.h>
#include <l2c.h>
#include <l2c_le.h>
#if F_BT_BREDR_SUPPORT
#include <l2c_br.h>
#endif
#include <l2c_api.h>
#include <btglib.h>
#include <btsm_api.h>
#include <gatt_api.h>
#include <sdp_api.h>
#include <hci_api.h>
#include <hci_code.h>
#include <swtimer.h>
#include <upper_stack_global.h>
#include <blueapi_api.h>
#include <os_pool.h>
#include <os_mem.h>
#include <os_timer.h>
#include <os_intr.h>

#define TRACE_MODULE_ID     MID_BT_L2C

#if defined(F_RAM_COUNTING)       /* define Instance and other Data global to get size from map/listing */
TL2C l2cInstanceRAM;
T_ACLPOOLDESC   l2cACLDescRAM;
T_L2CAP_CHANNEL l2cChanRAM;
#endif  /* defined(F_RAM_COUNTING) */

/* special stuff for qualification testcase stimulation */

void l2cFragmentDATA_REQ(P_ACLPOOLDESC pHciDesc);


BOOL L2CAP_ISREQUEST(uint8_t cmd_code)
{
    if (cmd_code == L2CAP_LE_FLOW_CONTROL_CREDIT)
        return FALSE;
    if (cmd_code % 2)
        return FALSE;
    else
        return TRUE;
}

/**
* @brief  get l2cap command identifier
*
* @param
*
* @return  command identifier
*
*/
uint8_t l2cGetL2capCmdID(void)
{
    pL2c->L2cap_next_cmdID++;
    if(pL2c->L2cap_next_cmdID == 0)
	{
    	pL2c->L2cap_next_cmdID++;
	}
	
    return pL2c->L2cap_next_cmdID;
}
/**
* @brief	l2cap change state and do some new state conf
*
* @param	pChan:
* @param	NewState
*
* @return
*
*/
#if (F_BT_BREDR_SUPPORT)||(F_BT_LE_BT41_SUPPORT)
void l2cChangeState(P_L2CAP_CHANNEL pChan, TL2cState NewState)
{
	L2CAP_TRACE_PRINTF_2(L2CAP_TRACE_MASK_TRACE,
	                      "l2cChangeState: %d -> %d", pChan->State, NewState);

	if (pChan->State != NewState)
	{
		switch (NewState)
		{
		case l2cStateClosed:
#if F_BT_LE_BT41_SUPPORT
            if(pChan->pHciDesc->conType == BT_CONNECTION_TYPE_LE)
            {
                (void)l2cDeleteChannel(pChan, FALSE);
            }
#endif
#if F_BT_BREDR_SUPPORT
            if(pChan->pHciDesc->conType != BT_CONNECTION_TYPE_LE)
            {
    			if(pChan->OpenL2cDiscConf == FALSE)
    			{
    				(void)l2cDeleteChannel(pChan, FALSE);
    			}
    			else
    			{
    				if(pChan->pHciDesc->uses == 1)
    				{
    					btgSendDiscReq(hciQueueID, pChan->pHciDesc->handle, HCI_ERR_OTHER_END_TERMINATE_13, FALSE);
    				}
    			}
            }
#endif
			break;
#if F_BT_BREDR_SUPPORT
		case l2cStateConfig:
			pChan->ConfReqReady = pChan->ConfRspReady = FALSE;
			l2cStartCONFIGTimeout(pChan, 2 * L2CAP_RTX_TIME);   /* start timeout */
			break;

		case l2cStateWaitForL2CAP_CONNECTION_RESPONSE:
		{
			L2CAP_TRACE_PRINTF_2(L2CAP_TRACE_MASK_TRACE,
			                        "L2C<- L2CAP ConReq psm %X scid %x", pChan->psm, pChan->LocalCid);

			l2cSendL2CAP_INFORMATION_REQUEST(pChan, L2CAP_INFOTYPE_EXTENDED_FEATURES_SUPPORT);
			break;
		}
#endif
#if F_BT_LE_BT41_SUPPORT
        case l2cStateOpen:
            if(pChan->pHciDesc->conType == BT_CONNECTION_TYPE_LE)
            {
                l2cSend_LEDataChannelParameterInfo(pChan->pHciDesc->handle,
                                                  pChan->LocalCid, pChan->RemoteMtu, 
                                                  pChan->RemoteMps, pChan->RemoteInitialCredits);
            }
            break;
#endif                
		default:
			break;
		}
	}
	pChan->State = NewState;
}



/**
* @brief		start l2cap t/rx timeout
*
* @param	id
* @param	secs
*
* @return
*
*/
void l2cStartRTXTimeout(P_L2CAP_CHANNEL pChan, uint16_t secs)
{
    if (pChan->RTXTimerHandle)
    {
        osDeleteTimer(&(pChan->RTXTimerHandle));
    }
    osStartTimer(&(pChan->RTXTimerHandle), l2cQueueID, L2CAP_RTX_TID, pChan->SigIdSent, secs*1000, swTimerCallBack);
}

/**
* @brief	stop t/rx timeout
*
* @param	id
*
* @return
*
*/
void l2cStopRTXTimeout(P_L2CAP_CHANNEL pChan)
{
	osDeleteTimer(&(pChan->RTXTimerHandle));
}

void l2cSendL2C_DISC_IND(PC_L2CAP_CHANNEL pChan, uint16_t status)
{
#if F_BT_LE_BT41_SUPPORT
    if(pChan->pHciDesc->conType == BT_CONNECTION_TYPE_LE)
    {
        l2cSend_DisconnectLEDataChannelInd(pChan->LocalCid, pChan->pHciDesc->handle, status); 
        return;
    }
#endif
#if F_BT_BREDR_SUPPORT
    if(pChan->usQueueID == gattQueueID)
    {
        gattHandleL2cDisconnectInd(pChan->LocalCid, status);
    }
    else if(pChan->usQueueID == sdpQueueID)
    {
        sdpHandleL2cDiscInd(pChan->LocalCid, status);
    }
    else if(pChan->usQueueID < OS_FIRST_QUEUE_ID)
    {
        blueAPI_Send_L2cDiscInd(pChan->LocalCid, pChan->usQueueID, status);
	}
#endif
}

/**
* @brief  handle l2cap disconnect response
*
* @param  lcid: local channel ID
*
* @return  
*
*/

void l2cHandleL2C_DISC_RESP(uint16_t lcid)
{
    P_L2CAP_CHANNEL pChan;

    L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_TRACE,
                            "L2C-> l2cHandleL2C_DISC_RESP lcid 0x%X ",
                            lcid);

    pChan = l2cSearchLcid(lcid);
    if(pChan != NULL && pChan->State == l2cStateWaitForL2C_DISC_RESP)  /* only if we still knew the channel */
    {
        /* Outgoing, no L2CAP_CONNECTION_RESP yet (e.g. authenticaion failure) */
        if (pChan->RemoteCid == 0)
        {
#if F_BT_BREDR_SUPPORT
            if(pChan->pHciDesc->conType != BT_CONNECTION_TYPE_LE)
            {
                pChan->ACLDiscImmediate = TRUE;
                (void)l2cDeleteChannel(pChan, FALSE);
            } 
#endif
            l2cChangeState(pChan, l2cStateClosed);
        }
        else
        {
            l2cSendL2CAP_DISCONNECTION_RESPONSE(pChan);
			l2cChangeState(pChan, l2cStateClosed);
        }
    }
}

void l2cSendSECMAN_AUTHENTICATION_IND(P_L2CAP_CHANNEL pChan, uint8_t outgoing, uint8_t active)
{
#if F_BT_LE_BT41_SUPPORT
    if(pChan->pHciDesc->conType == BT_CONNECTION_TYPE_LE)	
    {
	    btsmSendMsgAuthenticationInd(pChan->pHciDesc->remote_bd, pChan->LocalCid, pChan->le_psm, pChan->pHciDesc->handle, outgoing, active, SECMAN_SOURCE_L2CAP, BLUEFACE_CON_TYPE_LE);
    }
#endif
#if F_BT_BREDR_SUPPORT
    if(pChan->pHciDesc->conType != BT_CONNECTION_TYPE_LE)	
    {
        btsmSendMsgAuthenticationInd(pChan->pHciDesc->remote_bd, pChan->LocalCid, pChan->psm, pChan->uuid, outgoing, active, SECMAN_SOURCE_L2CAP, BLUEFACE_CON_TYPE_BR_EDR);
    }
#endif
    pChan->authenticationActive = 1;
}
/**
* @brief	 l2cap handle secmanager authentication response
*
* @param	bd:
* @param	ref: 
* @param	channelId
* @param	uuid
* @param	outgoing
* @param	cause
*
* @return
*
*/
void l2cHandleSECMAN_AUTHENTICATION_RESP( TBdAddr bd,
																  uint16_t    ref,
																  uint16_t    channelId,
																  uint16_t    uuid,
																  uint8_t    outgoing,
																  uint16_t    cause)
{
    P_L2CAP_CHANNEL            pChan;

    L2CAP_TRACE_PRINTF_6(L2CAP_TRACE_MASK_TRACE,
                   "L2C-> SECMAN_AUTHENTICATION_RESP bd %s CID %X PSM %X UUID %X outgoing %d cause %x",
                   TRACE_BDADDR1(L2CAP_TRACE_MASK_TRACE, bd),
                   ref,
                   channelId,
                   uuid,
                   outgoing,
                   cause);

    pChan = l2cSearchLcid(ref);
    if (pChan == (P_L2CAP_CHANNEL)0)
    {
         L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_ERROR,
                                 "!!l2cHandleSECMAN_AUTHENTICATION_RESP: CID %x not found",
                                 ref);
         return;
    }

    pChan->authenticationActive = 0;

    switch (pChan->State)
    {
#if F_BT_BREDR_SUPPORT
	case l2cStateClosed:
	case l2cStateWaitForL2CAP_CONNECTION_RESPONSE:
	    if (outgoing)
	    {
	        if (cause)
	        {
				l2cChangeState(pChan, l2cStateWaitForL2C_DISC_RESP);
				l2cSendL2C_DISC_IND(pChan, cause);
	        }
	        else
	        {
	            uint8_t ConReqPara[4];

	            SHORT2CHAR(ConReqPara,     pChan->psm);
	            SHORT2CHAR(ConReqPara + 2, pChan->LocalCid);

	            L2CAP_TRACE_PRINTF_2(L2CAP_TRACE_MASK_TRACE,
	                                    "L2C<- L2CAP ConReq psm %X scid %x", pChan->psm, pChan->LocalCid);

	            l2cSendL2CAPMessage(pChan, NULL, L2CAP_CONNECTION_REQUEST, ConReqPara, sizeof(ConReqPara), 0 );
	            l2cChangeState(pChan, l2cStateWaitForL2CAP_CONNECTION_RESPONSE);
	        }
	    }
	    else
	    {
	        if (cause)
	        {
	            btgSendDiscReq(hciQueueID, pChan->pHciDesc->handle, HCI_ERR_AUTHENTICATION_FAILED, FALSE);
	            (void)l2cDeleteChannel(pChan, FALSE);
	        }
	        else
	        {
				/*state change before*/
	            l2cChangeState(pChan, l2cStateWaitForL2C_CON_RESP);
		
				if (pChan->usQueueID == gattQueueID)
				{
					gattHandleL2cConnectInd(pChan->LocalCid, pChan->psm, pChan->pHciDesc->remote_bd);
				}
				else if (pChan->usQueueID == sdpQueueID)
				{
					sdpHandleL2cConInd(pChan->LocalCid);
				}
                else if(pChan->usQueueID < OS_FIRST_QUEUE_ID)
				{
                    blueAPI_Send_L2cConInd(pChan->LocalCid, pChan->usQueueID, pChan->pHciDesc->remote_bd);
				}
	        }
	    }
	    break;
#endif
#if F_BT_LE_BT41_SUPPORT        
        case l2cStateWaitForSECMAN_AUTHENTICATION_RESP:
            if (cause)
            {
                l2cSendL2CAP_LE_CREDIT_BASED_CONNECTION_RESPONSE(NULL, pChan, 0, 0, 0, 0, cause);
                l2cChangeState(pChan, l2cStateClosed);
            }
            else
            {
                l2cSend_CreateLEDataChannelInd(pChan->LocalCid, pChan->pHciDesc->handle);
                l2cChangeState(pChan, l2cStateWaitForL2C_CON_LE_DATA_CHANNEL_RESP);
            }

            break;
#endif
        default:
            break;
    }
}

void l2cRemoveChan(P_L2CAP_CHANNEL pChan, BOOL HCIDisconnected)
{
	l2cStopRTXTimeout(pChan);
#if F_BT_BREDR_SUPPORT
	l2cStopCONFIGTimeout(pChan);
#endif
	(void)l2cDeleteChannel(pChan, HCIDisconnected);
} 
#endif
/**
* @brief  send l2cap PDU to hci layer
*
* @param  HciHandle:
* @param  pMsg: 
* @param  L2CapSize:
* @param  cid:
* @param  startPak: 
*
* @return  
*
*/
void l2cSendL2CAP_PDU(uint16_t HciHandle, MESSAGE_P pMsg, uint16_t L2CapSize, uint16_t cid, BOOL startPak)
{
	uint8_t * pPdu;

	pPdu = pMsg->MData.DataCBChan.BufferAddress + pMsg->MData.DataCB.Offset;

	/*l2cap header format:
	 *|length(2byte)|channle ID(2byte)|info payload|*/
	if(startPak) 							/* insert L2C header before payload */
	{                          
	    pPdu = pPdu - L2CAP_HDR_LENGTH;     /* pos ptr 4 bytes back for L2c header */
	    SHORT2CHAR(pPdu, L2CapSize);        /* write L2Cap packet length */
	    SHORT2CHAR(pPdu + 2, cid);          /* and channel ID */

	    pMsg->MData.DataCBChan.Length += L2CAP_HDR_LENGTH;
	    pMsg->MData.DataCBChan.Offset -= L2CAP_HDR_LENGTH;
	}

	pMsg->MData.DataCBChan.Flag &= ~(DATA_CB_BLOCK_FIRST | DATA_CB_BLOCK_MIDDLE);

	if (startPak)
	{
  		pMsg->MData.DataCBChan.Flag |= DATA_CB_BLOCK_FIRST;
	}
	else
	{
  		pMsg->MData.DataCBChan.Flag |= DATA_CB_BLOCK_MIDDLE;
	}

	pMsg->MData.DataCBChan.Channel = HciHandle;

	pMsg->Command = HCI_DATA_REQ;
	osMessageSend(hciQueueID, pMsg);
}

/**
* @brief  send l2cap msg
*
* @param  pChan: l2cap channel
* @param  pHciDesc: hci descriptor
* @param  cmd_code: msg cmd
* @param  para_buf: param buf
* @param  length: bug length
* @param  timeout: timeout(sec)
*
* @return  
*
*/
void l2cSendL2CAPMessage(P_L2CAP_CHANNEL pChan, P_ACLPOOLDESC pHciDesc, uint8_t cmd_code, LPCBYTE para_buf, uint16_t length, uint16_t timeout)
{
    MESSAGE_T 	msg;
    uint8_t * 		myBuf;
    uint8_t * 		pCmd;
    LPCmdBuf 	pCmdBuf = NULL;
    uint16_t     	channel;
    uint16_t needed_bytes = L2CAP_SIGNAL_MTU + L2C_WRITE_OFFSET;

	/*l2cap command format:
	* |Code(1byte)|Identifier(1byte)|Length(2byte)|data|*/
    assert((length + L2CAP_HDR_LENGTH) < L2CAP_SIGNAL_MTU);

    if(osBufferGet(BTSystemPoolID, needed_bytes, (void  *)&myBuf))
    {
		L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_ERROR,
		                    "!!: l2cSendL2CAPMessage Could not allocate memory %X", needed_bytes);
		return;
    }

	/*if channel is not complete, we use pHciDesc instead*/
#if (F_BT_BREDR_SUPPORT)||(F_BT_LE_BT41_SUPPORT)
    if(pChan != NULL)
    {
		assert(pHciDesc == NULL);
		pHciDesc = pChan->pHciDesc;
    }
    else
#endif
    {
		assert(pHciDesc != NULL);
    }

    pCmd = myBuf + L2C_WRITE_OFFSET;
    *pCmd = cmd_code;

	/*check if it is request command*/
    if(L2CAP_ISREQUEST(cmd_code)) 
	{
		if ((pHciDesc->conType == BT_CONNECTION_TYPE_LE) && (pChan == NULL))
		{
            pHciDesc->SigIdSent = *(pCmd + 1) = l2cGetL2capCmdID();

            if (timeout)
    		{
    			l2cLEStartRTXTimeout(pHciDesc, timeout);
    		}
    		else
    		{
    			l2cLEStartRTXTimeout(pHciDesc, L2CAP_RTX_TIME);
    		}

		}
#if (F_BT_BREDR_SUPPORT)||(F_BT_LE_BT41_SUPPORT)
		else
		{
		    assert(pChan != NULL);
		    pChan->SigIdSent = *(pCmd + 1) = l2cGetL2capCmdID();
#if (BT_L2C_MAX_CMD_RETRIES_COUNT != 0)
		    pChan->ReqCmdRetryCount = BT_L2C_MAX_CMD_RETRIES_COUNT;
		    pCmdBuf = &pChan->lastReqSend;
#else 
		    pChan->lastReqSend = cmd_code;
#endif
    		if (timeout)
    		{
    			l2cStartRTXTimeout(pChan, timeout);
    		}
    		else
    		{
    			l2cStartRTXTimeout(pChan, L2CAP_RTX_TIME);
    		}
		}
#endif
	}
#if F_BT_LE_BT41_SUPPORT
    else if (cmd_code == L2CAP_LE_FLOW_CONTROL_CREDIT)
    {
        *(pCmd + 1) = l2cGetL2capCmdID();
    }
#endif
	else 
	{
#if (F_BT_BREDR_SUPPORT)||(F_BT_LE_BT41_SUPPORT)
		if(pChan != NULL)
		{
			*(pCmd + 1) = pChan->ReqIdRcv;			/* send response with ID of the Request */
			pCmdBuf = &pChan->lastRspSend;//for bredr
		}
		else
#endif
		{
			*(pCmd + 1) = pHciDesc->ReqIdRcv;			/* send response with ID of the Request */
		}
    }
    SHORT2CHAR(pCmd + 2, length);
    if(length != 0)
    {
        memcpy((pCmd + L2CAP_HDR_LENGTH), para_buf, length);
    }

    if(pCmdBuf != NULL)  //for bredr       /* only if we have a retry buffer */
    {
		pCmdBuf->len = (uint16_t)(L2CAP_HDR_LENGTH + length);    /* save the pdu for later use */
		memcpy(pCmdBuf->buf, pCmd, pCmdBuf->len);
    }

    msg.MData.DataCB.BufferAddress = myBuf;
    msg.MData.DataCB.Offset        = L2C_WRITE_OFFSET;
    msg.MData.DataCB.Flag          = DATA_CB_RELEASE;
    msg.MData.DataCB.Length        = length + L2CAP_HDR_LENGTH;


    if (pHciDesc->conType == BT_CONNECTION_TYPE_LE)
	{
    	channel = CID_SIGNALING_LE;
	}
#if F_BT_BREDR_SUPPORT
	else
	{
		channel = CID_SIGNALING_BR;
	}
#endif

    if (FALSE == gL2cDsFragmentationSupport)
    {
        l2cSendL2CAP_PDU(pHciDesc->handle, &msg, msg.MData.DataCB.Length, channel, TRUE);
    }
    else
    {
        msg.MData.DataCBChan.Channel = channel;
        osMessageSend( (uint16_t)pHciDesc->dsPrioBufQueue, &msg);
        l2cFragmentDATA_REQ((P_ACLPOOLDESC)pHciDesc);
    }

}

/**
* @brief  resend last sent l2cap msg
*
* @param  pChan: l2cap channel
* @param  pLastCmd
*
* @return  
*
*/
#if (F_BT_BREDR_SUPPORT)||(F_BT_LE_BT41_SUPPORT)
void l2cReSendL2CAPMessage(PC_L2CAP_CHANNEL pChan, LPCCmdBuf pLastCmd)
{
    MESSAGE_T msg;
    uint8_t * myBuf;
    uint8_t * pCmd;
    uint16_t   channel;
    uint16_t needed_bytes = L2CAP_SIGNAL_MTU + L2C_WRITE_OFFSET;

    if (osBufferGet(BTSystemPoolID, needed_bytes, (void  *)&myBuf)) {
        L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_ERROR,
                                "!!: l2cReSendL2CAPMessage Could not allocate memory %X", needed_bytes);
        return;
    }

    pCmd = myBuf + L2C_WRITE_OFFSET;
    memcpy(pCmd, pLastCmd->buf, pLastCmd->len);
    msg.MData.DataCB.BufferAddress = myBuf;
    msg.MData.DataCB.Offset        = L2C_WRITE_OFFSET;
    msg.MData.DataCB.Flag          = DATA_CB_RELEASE;
    msg.MData.DataCB.Length        = pLastCmd->len;

    L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_TRACE,
                            "l2cReSendL2CAPMessage() id 0x%X", *(pCmd + 1));

    if (*pCmd == L2CAP_CONNECTION_PARAMETER_UPDATE_REQUEST ||
        *pCmd == L2CAP_CONNECTION_PARAMETER_UPDATE_RESPONSE)
	{
    	channel = CID_SIGNALING_LE;
	}
    else
	{
    	channel = CID_SIGNALING_BR;
	}

    if (FALSE == gL2cDsFragmentationSupport)
    {
        l2cSendL2CAP_PDU(pChan->pHciDesc->handle, &msg, msg.MData.DataCB.Length, channel, TRUE);
    }
    else
    {
        msg.MData.DataCBChan.Channel = channel;
        osMessageSend( (uint16_t)pChan->pHciDesc->dsPrioBufQueue, &msg);
        l2cFragmentDATA_REQ(pChan->pHciDesc);
    }

    if(L2CAP_ISREQUEST(*pCmd)) 
	{
        l2cStartRTXTimeout((P_L2CAP_CHANNEL)pChan, L2CAP_RTX_TIME);
    }
}
#endif

/**
* @brief  send l2cap reject cmd
* 
* @param  pChan
* @param  pHciDesc
* @param  reason
* @param  para1
* @param  para2
*
* @return  
*
*/
void l2cSendL2CAP_COMMAND_REJECT(P_L2CAP_CHANNEL pChan, P_ACLPOOLDESC pHciDesc, uint16_t reason, uint16_t para1, uint16_t para2)
{
	uint8_t mbuf[6];
	PBYTE pbuf = mbuf;

	SHORT2CHAR(pbuf, (uint16_t)reason);        /* reason code */
	pbuf += 2;

	switch(reason)
	{
	case L2CAP_CMDREJ_NOT_UNDERSTOOD:
	    break;
	case L2CAP_CMDREJ_MTU_EXCEEDED:
	    SHORT2CHAR(pbuf, para1);           /* 1. para: max MTU */
	    pbuf += 2;
	    break;
	case L2CAP_CMDREJ_INVALID_CID:
	    SHORT2CHAR(pbuf, para1);           /* 1. para: local CID */
	    pbuf += 2;
	    SHORT2CHAR(pbuf, para2);           /* 2. para: remote CID */
	    pbuf += 2;
	    break;
	default:
	  assert(FALSE);
	  break;
	}

	L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_TRACE,"L2C<- L2CAP_COMMAND_REJECT");
	l2cSendL2CAPMessage(pChan,  pHciDesc, L2CAP_COMMAND_REJECT, mbuf, (uint16_t)(pbuf - mbuf), 0 );
}

/**
* @brief  l2cap send disconnect req msg
* 
* @param  pChan
*
*/
#if (F_BT_BREDR_SUPPORT)||(F_BT_LE_BT41_SUPPORT)
void l2cSendL2CAP_DISCONNECTION_REQUEST(P_L2CAP_CHANNEL pChan)
{
    uint8_t DiscPara[4];
	
    L2CAP_TRACE_PRINTF_2(L2CAP_TRACE_MASK_TRACE,
                                  "L2C<- L2CAP_DISCONNECTION_REQUEST dcid 0x%X scid 0x%X ",
                                  pChan->RemoteCid, pChan->LocalCid);

    SHORT2CHAR(DiscPara,     pChan->RemoteCid);
    SHORT2CHAR(DiscPara + 2, pChan->LocalCid);

    l2cSendL2CAPMessage(pChan, NULL, L2CAP_DISCONNECTION_REQUEST, DiscPara, sizeof(DiscPara), 0 );
}

void l2cSendL2CAP_DISCONNECTION_RESPONSE(P_L2CAP_CHANNEL pChan)
{
    uint8_t DiscPara[4];

    L2CAP_TRACE_PRINTF_2(L2CAP_TRACE_MASK_TRACE,
                          "L2C<- L2CAP_DISCONNECTION_RESPONSE dcid 0x%X scid 0x%X ",
                          pChan->LocalCid, pChan->RemoteCid);
    SHORT2CHAR(DiscPara,     pChan->LocalCid);
    SHORT2CHAR(DiscPara + 2, pChan->RemoteCid);
    l2cSendL2CAPMessage(pChan,  NULL, L2CAP_DISCONNECTION_RESPONSE, DiscPara, sizeof(DiscPara), 0 );
}

/**
* @brief  l2cap handle disconnect request msg
* 
* @param  hciDesc
* @param  l2cap_cmd
* @param  pPar
*
* @return  
*
*/
void l2cHandleL2CAP_DISCONNECTION_REQUEST(P_ACLPOOLDESC hciDesc, T_L2CAP_Command *l2cap_cmd, uint8_t * pPar)
{
    P_L2CAP_CHANNEL    pChan;
    T_L2CAP_DiscR_para L2CAP_DiscR;

    L2CAP_DiscR.dcid = CHAR2SHORT(pPar);
    L2CAP_DiscR.scid = CHAR2SHORT(pPar + 2);

    L2CAP_TRACE_PRINTF_2(L2CAP_TRACE_MASK_TRACE,
                            "L2C-> L2CAP DISCONNECT REQ dcid 0x%X scid 0x%X",
                            L2CAP_DiscR.dcid, L2CAP_DiscR.scid);

    if((pChan = l2cSearchLcid(L2CAP_DiscR.dcid)) != NULL)
    {
        if(L2CAP_DiscR.scid != pChan->RemoteCid)
    	{
        	return;                /* if scid do not match discard the cmd see spec page 285 */
    	}
    }
    else                           /* channel not found */
    {
        hciDesc->ReqIdRcv = l2cap_cmd->id;
        l2cSendL2CAP_COMMAND_REJECT(NULL, hciDesc, L2CAP_CMDREJ_INVALID_CID, L2CAP_DiscR.dcid, L2CAP_DiscR.scid);
        return;
    }

    if(l2cap_cmd->id == pChan->ReqIdRcv) 
	{        /* it is a repeatet request because same cmd ID */
        /* so we will send again the last response */
        l2cReSendL2CAPMessage(pChan, &pChan->lastRspSend);
        return;                                                                /* thats all */
    }
    pChan->ReqIdRcv  = l2cap_cmd->id;

    switch (pChan->State)
    {
	case l2cStateOpen:
	case l2cStateConfig:
		l2cStopRTXTimeout(pChan);
#if F_BT_BREDR_SUPPORT
		l2cStopCONFIGTimeout(pChan);   /* stop timeout */
#endif
		l2cChangeState(pChan, l2cStateWaitForL2C_DISC_RESP);
		l2cSendL2C_DISC_IND(pChan, 0);
		break;

	case l2cStateWaitForL2CAP_DISCONNECTION_RESPONSE:
		l2cChangeState(pChan, l2cStateWaitForL2C_DISC_RESP);
		l2cSendL2C_DISC_IND(pChan, 0);
		break;

	default:
		break;
    }
}

/**
* @brief  l2cap handle disconnect response
* 
* @param  l2cap_cmd
* @param  pPar
*
* @return  
*
*/
void l2cHandleL2CAP_DISCONNECTION_RESPONSE(T_L2CAP_Command *l2cap_cmd, uint8_t * pPar)
{
    P_L2CAP_CHANNEL    pChan;
    T_L2CAP_DiscR_para L2CAP_DiscR;

    L2CAP_DiscR.dcid = CHAR2SHORT(pPar);
    L2CAP_DiscR.scid = CHAR2SHORT(pPar + 2);

    L2CAP_TRACE_PRINTF_2(L2CAP_TRACE_MASK_TRACE,
                            "L2C-> L2CAP DISCONNECT RESP dcid 0x%X scid 0x%X",
                            L2CAP_DiscR.dcid, L2CAP_DiscR.scid);

    if((pChan = l2cSearchLcid(L2CAP_DiscR.scid)) == NULL)
	{
    	return;                                /* if channel not found just ignore the response */
	}
	
    if(L2CAP_DiscR.dcid != pChan->RemoteCid)
	{
    	return;                /* if dcid does not match discard the response see spec page 285 */
	}
    pChan->RespIdRcv = l2cap_cmd->id;

    switch (pChan->State)
    {
	case l2cStateWaitForL2CAP_DISCONNECTION_RESPONSE:
#if F_BT_LE_BT41_SUPPORT
        if(pChan->pHciDesc->conType == BT_CONNECTION_TYPE_LE)
        {
            if(pChan->WaitDisRsp)
            {
                l2cSend_DisconnectLEDataChannelRsp(pChan->LocalCid, pChan->pHciDesc->handle, L2CAP_NO_CAUSE);
                pChan->WaitDisRsp = FALSE;
            }
        }
#endif
#if F_BT_BREDR_SUPPORT
        if(pChan->pHciDesc->conType != BT_CONNECTION_TYPE_LE)
        {
            l2cTearDownChan(pChan);
        }
#endif
		l2cChangeState(pChan, l2cStateClosed);
		break;

	case l2cStateWaitForL2C_DISC_RESP:
#if F_BT_LE_BT41_SUPPORT
        if(pChan->pHciDesc->conType == BT_CONNECTION_TYPE_LE)
        {
            l2cChangeState(pChan, l2cStateClosed);
        }
#endif
#if F_BT_BREDR_SUPPORT
        if(pChan->pHciDesc->conType != BT_CONNECTION_TYPE_LE)
        {
            l2cTearDownChan(pChan);
        }
#endif
		break;

	default:
		break;
    }
}
#endif
/**
* @brief  l2cap handle command reject 
* 
* @param  pChan
* @param  pPar
*
* @return  
*
*/
void l2cHandleL2CAP_COMMAND_REJECT(P_L2CAP_CHANNEL pChan, uint8_t * pPar)
{
    uint16_t reason;

    reason = CHAR2SHORT(pPar);


    L2CAP_TRACE_PRINTF_4(L2CAP_TRACE_MASK_TRACE,
                            "L2C-> L2CAP COMMAND REJECT reason %d par1 0x%X par2 0x%X",
                            reason, CHAR2SHORT(pPar + 2), CHAR2SHORT(pPar + 4), 0);

    if (pChan != NULL)
    {
#if F_BT_LE_BT41_SUPPORT
        if(pChan->pHciDesc->conType == BT_CONNECTION_TYPE_LE)
        {
            uint16_t cause = L2CAP_ERR_ILLEGAL_PARAMETER | L2CAP_ERR;
            if (reason == 0)
                cause = L2CAP_ERR_CMD_NOT_UNDERSTOOD | L2CAP_ERR;
            if (pChan->State == l2cStateWaitForL2CAP_LE_CREDIT_BASED_CONNECTION_RESPONSE)
            {
                l2cSend_CreateLEDataChannelRsp(0, pChan->pHciDesc->handle, cause);
                l2cChangeState(pChan, l2cStateClosed);
            }
        }
#endif
#if F_BT_BREDR_SUPPORT
        if(pChan->pHciDesc->conType != BT_CONNECTION_TYPE_LE)
        {
            if (reason == L2CAP_CMDREJ_NOT_UNDERSTOOD &&
                (pChan->L2capInformationRequestSent))
            {
                l2cCheckMode(pChan);  /* simulate info resp */
            }
        }
#endif
    }
}

/**
* @brief  l2cap handle signal command packet
* 
* @param  hciDesc
* @param  pCmd_pak
* @param  cmd_pak_len
*
* @return  
*
*/
int l2cHandleSigCmd(P_ACLPOOLDESC hciDesc, uint8_t * pCmd_pak, uint16_t cmd_pak_len)
{
	T_L2CAP_Command l2cap_cmd;
#if (F_BT_BREDR_SUPPORT)||(F_BT_LE_BT41_SUPPORT)
    P_L2CAP_CHANNEL pChan = NULL;
#endif
    uint8_t * pPar;

    l2cap_cmd.code   = *pCmd_pak;
    l2cap_cmd.id     = *(pCmd_pak + 1);
    l2cap_cmd.length = CHAR2SHORT(pCmd_pak + 2) ;

    L2CAP_TRACE_PRINTF_4(L2CAP_TRACE_MASK_TRACE,
                            "L2C-> l2cap command %d ID 0x%X length %d HCI handle 0x%X",
                            (l2cap_cmd.code), l2cap_cmd.id, l2cap_cmd.length, hciDesc->handle);

    pPar = pCmd_pak + 4;                /* parameters start 4 byte behind L2Cap header */

    if(l2cap_cmd.id == 0) 
	{
        L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_ERROR,
                                "!!L2C-> commandpacket with invalid command ID 0, discarding it");
        return 0;                                                /* so discard silently */
    }
    if(L2CAP_ISREQUEST(l2cap_cmd.code))		/* it is a request */
	{
#if F_BT_BREDR_SUPPORT
		if(cmd_pak_len > L2CAP_SIGNAL_MTU)
#else
        if(cmd_pak_len > L2CAP_SIGNAL_MTU_LE)
#endif
		{
			L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_ERROR,
			                        "!!l2cHandleSigCmd() packet size (%d) exeeds MTU_sig !",cmd_pak_len);

			hciDesc->ReqIdRcv = l2cap_cmd.id;
			l2cSendL2CAP_COMMAND_REJECT(NULL, hciDesc, L2CAP_CMDREJ_MTU_EXCEEDED, L2CAP_SIGNAL_MTU, 0);
			return -1;
		}

		switch(l2cap_cmd.code)
		{
#if F_BT_BREDR_SUPPORT
		case L2CAP_CONNECTION_REQUEST:
		case L2CAP_CONFIGURE_REQUEST:
#if !F_BT_LE_BT41_SUPPORT
		case L2CAP_DISCONNECTION_REQUEST:
#endif
		case L2CAP_ECHO_REQUEST:
		case L2CAP_INFORMATION_REQUEST:       /* received a INFO_REQUEST answer with INFORMATION RESPONSE */
		    if (hciDesc->signalingCID != CID_SIGNALING_BR)
		    {
		        l2cSendL2CAP_COMMAND_REJECT(NULL, hciDesc, L2CAP_CMDREJ_INVALID_CID, hciDesc->signalingCID, 0);
		        return 0;
		    }
		    break;
#endif
		case L2CAP_CONNECTION_PARAMETER_UPDATE_REQUEST:
 #if F_BT_LE_BT41_SUPPORT
        case L2CAP_LE_CREDIT_BASED_CONNECTION_REQUEST:
 #endif
		    if (hciDesc->signalingCID != CID_SIGNALING_LE)
		    {
		        l2cSendL2CAP_COMMAND_REJECT(NULL, hciDesc, L2CAP_CMDREJ_INVALID_CID, hciDesc->signalingCID, 0);
		        return 0;
		    }
		    break;

		default:
		    break;
		} /* switch(l2cap_cmd.code) */
    } 
    else if (l2cap_cmd.code != L2CAP_LE_FLOW_CONTROL_CREDIT)
     	/* it is a response */
    {
#if (F_BT_BREDR_SUPPORT)||(F_BT_LE_BT41_SUPPORT)
        pChan = l2cSearchCmdId(l2cap_cmd.id);
        if(pChan == NULL)
        {
#endif
    		if (hciDesc->conType == BT_CONNECTION_TYPE_LE)
    		{
    		    if (hciDesc->SigIdSent != l2cap_cmd.id)                  /* we have not send a command with this cmd Id */
    		    {
    		        L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_ERROR,
    		                                "!L2C-> response with invalid command ID, discarding it");
                    switch (l2cap_cmd.code)
        			{
        				case L2CAP_COMMAND_REJECT:
    					case L2CAP_CONNECTION_PARAMETER_UPDATE_RESPONSE:
#if F_BT_LE_BT41_SUPPORT
    					case L2CAP_LE_CREDIT_BASED_CONNECTION_RESPONSE:
    					case L2CAP_DISCONNECTION_RESPONSE:/* so discard silently */
#endif
    						break;
    					default:
           				    L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_ERROR,
                                 "!!l2cHandleSigCmd() INVALID L2CAP command code 0x%X !!!!!!!!", l2cap_cmd.code);

            				hciDesc->ReqIdRcv = l2cap_cmd.id;
            				l2cSendL2CAP_COMMAND_REJECT(NULL,hciDesc, L2CAP_CMDREJ_NOT_UNDERSTOOD, 0, 0);		
    				}
    		        return 0;                                            /* so discard silently */
    		    }
                l2cLEStopRTXTimeout(hciDesc);	/*stop timer*/
    		}
            else
            {
                L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_ERROR,
		                                "!L2C-> response with invalid command ID, discarding it");
		        return 0;     
            }
#if (F_BT_BREDR_SUPPORT)||(F_BT_LE_BT41_SUPPORT)
        }
		else
		{
            l2cStopRTXTimeout(pChan);	/*stop timer*/
		}
#endif
    }

    switch(l2cap_cmd.code) 
	{
#if F_BT_BREDR_SUPPORT
    case L2CAP_CONNECTION_REQUEST:
        l2cHandleL2CAP_CONNECTION_REQUEST(hciDesc, &l2cap_cmd, pPar);
        break;

    case L2CAP_CONNECTION_RESPONSE:
        l2cHandleL2CAP_CONNECTION_RESPONSE(&l2cap_cmd, pPar);
        break;

    case L2CAP_CONFIGURE_REQUEST:
        l2cHandleL2CAP_CONFIGURE_REQUEST(hciDesc, &l2cap_cmd, pPar);
        break;

    case L2CAP_CONFIGURE_RESPONSE:
        l2cHandleL2CAP_CONFIGURE_RESPONSE(&l2cap_cmd, pPar);
        break;

    case L2CAP_ECHO_REQUEST:
        l2cHandleL2CAP_ECHO_REQUEST(hciDesc, &l2cap_cmd);
        return 0;
#if F_BT_L2C_ENHANCED_CONFORMANCE
    case L2CAP_ECHO_RESPONSE:
        l2cHandleL2CAP_ECHO_RESPONSE(pChan, &l2cap_cmd);
        return 0;
#endif
    case L2CAP_INFORMATION_REQUEST:       /* received a INFO_REQUEST answer with INFORMATION RESPONSE */
        l2cHandleL2CAP_INFORMATION_REQUEST(hciDesc, &l2cap_cmd, pPar);
        return 0;

    case L2CAP_INFORMATION_RESPONSE:
        l2cHandleL2CAP_INFORMATION_RESPONSE(pChan, hciDesc, &l2cap_cmd, pPar);
        break;
#endif

#if (F_BT_BREDR_SUPPORT)||(F_BT_LE_BT41_SUPPORT)
    case L2CAP_DISCONNECTION_REQUEST:
        l2cHandleL2CAP_DISCONNECTION_REQUEST(hciDesc, &l2cap_cmd, pPar);
        break;

    case L2CAP_DISCONNECTION_RESPONSE:
        l2cHandleL2CAP_DISCONNECTION_RESPONSE(&l2cap_cmd, pPar);
        break;
#endif        
    case L2CAP_COMMAND_REJECT:
#if (F_BT_BREDR_SUPPORT)||(F_BT_LE_BT41_SUPPORT)
        l2cHandleL2CAP_COMMAND_REJECT(pChan, pPar);
#else
        l2cHandleL2CAP_COMMAND_REJECT(NULL, pPar);
#endif
        break;

    case L2CAP_CONNECTION_PARAMETER_UPDATE_REQUEST:
        hciDesc->ReqIdRcv = l2cap_cmd.id;
        l2cHandleL2CAP_CONNECTION_PARAMETER_UPDATE_REQUEST(hciDesc, pPar);
        break;

    case L2CAP_CONNECTION_PARAMETER_UPDATE_RESPONSE:
        l2cHandleL2CAP_CONNECTION_PARAMETER_UPDATE_RESPONSE(hciDesc, pPar);
        break;
#if F_BT_LE_BT41_SUPPORT
    case L2CAP_LE_CREDIT_BASED_CONNECTION_REQUEST:
        hciDesc->ReqIdRcv = l2cap_cmd.id;
        l2cHandleL2CAP_LE_CREDIT_BASED_CONNECTION_REQUEST(hciDesc, &l2cap_cmd, pPar);
        break;

    case L2CAP_LE_CREDIT_BASED_CONNECTION_RESPONSE:
        l2cHandleL2CAP_LE_CREDIT_BASED_CONNECTION_RESPONSE(hciDesc, &l2cap_cmd, pPar);
        break;

    case L2CAP_LE_FLOW_CONTROL_CREDIT:
        l2cHandleL2CAP_LE_FLOW_CONTROL_CREDIT(hciDesc, &l2cap_cmd, pPar);
        break;
#endif
    default:
        L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_ERROR,
                                "!!l2cHandleSigCmd() INVALID L2CAP command code 0x%X !!!!!!!!",l2cap_cmd.code);

        hciDesc->ReqIdRcv = l2cap_cmd.id;
        l2cSendL2CAP_COMMAND_REJECT(NULL, hciDesc, L2CAP_CMDREJ_NOT_UNDERSTOOD, 0, 0);
        return 0;
    } /* switch(l2cap_cmd.code) */

#if (F_BT_BREDR_SUPPORT)||(F_BT_LE_BT41_SUPPORT)
    if(pChan == NULL) 
	{ 
#endif
        /* we did not find the channel */
        if(L2CAP_ISREQUEST(l2cap_cmd.code)) 
		{        /* it is a request */
            hciDesc->ReqIdRcv = l2cap_cmd.id;
        }
#if (F_BT_BREDR_SUPPORT)||(F_BT_LE_BT41_SUPPORT)
    }
#endif
    return 0;
}

/**
* @brief  l2cap handle signal command packet
* 
* @param  hciDesc
* @param  pCmd_pak
* @param  cmd_pak_len
*
* @return  
*
*/
void l2cHandleSigCmdPacket(P_ACLPOOLDESC hciDesc, uint8_t * pCmd_pak, uint16_t cmd_pak_len)
{
	uint16_t cmdLength;
	uint16_t offset = 0;

	cmdLength = CHAR2SHORT(pCmd_pak + 2);

	if(cmd_pak_len < cmdLength + 4)
    {
	    L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_ERROR,
	                            "L2C-> command packet too short, discarded");
	    return;
	}

	if(cmd_pak_len > cmdLength + 4)
    {
	    L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_TRACE,
	                            "L2C-> l2cHandleSigCmdPacket(), multiple commands in packet");
	}
	else
	{
	    L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_TRACE,
	                            "L2C-> l2cHandleSigCmdPacket(), single command in packet");
	}

	while(offset < cmd_pak_len) 
	{
	    cmdLength = CHAR2SHORT(pCmd_pak + offset + 2);
	    if(l2cHandleSigCmd(hciDesc, pCmd_pak + offset, cmd_pak_len) == -1) 
		{
	        return;
	    }
	    offset += (cmdLength + 4);
	}
}

/**
* @brief  l2cap handle hci data
* 
* @param
*
* @return  
*
*/
void l2cHandleHCI_DATA_IND(void)
{
    MESSAGE_T     Msg;
    MESSAGE_P     pMsg;
    P_ACLPOOLDESC pHciDesc;
    T_L2C_HEADER  Pdu_hdr;
    uint8_t *        pPdu_hdr;
    uint16_t          hci_handle;
    uint16_t          data_length;

    pMsg = &pL2c->Message;
    pPdu_hdr = (uint8_t *)(pMsg->MData.DataCBChan.BufferAddress + pMsg->MData.DataCB.Offset);

    data_length = pMsg->MData.DataCBChan.Length;
    hci_handle  = pMsg->MData.DataCBChan.Channel;

    pHciDesc = l2cSearchHciDescByHciHandle(hci_handle);
    if(pHciDesc == NULL)
    {
        L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_ERROR,
                                "!! HCI_DATA_IND on unknown channel");
        return;
    }

    if(pMsg->MData.DataCB.Flag & DATA_CB_BLOCK_MIDDLE)  /* continuation of L2CAP packet */
	{ 
		pMsg->MData.DataCB.Flag &= ~(DATA_CB_BLOCK_MIDDLE);

		if(pHciDesc->usPakOpen.len != 0) /* channel with open packet found */
		{ 
			int bytesFree = (pHciDesc->usPakOpen.expected != 0) ? pHciDesc->usPakOpen.expected : (BT_US_PDU_L2C_BYTE_COUNT + 4);
			bytesFree = bytesFree - pHciDesc->usPakOpen.len;
			if(bytesFree < data_length)
			{
				L2CAP_TRACE_PRINTF_2(L2CAP_TRACE_MASK_ERROR,
				                      "!!L2C Data overshots announced / MAX len expected %d got %d",
				                      pHciDesc->usPakOpen.expected, pHciDesc->usPakOpen.len + data_length);
				data_length = (uint16_t)bytesFree;    /* cut down to maximum free space in buffer to avoid overflow */
			}
			memcpy((pHciDesc->usPakOpen.pd + pHciDesc->usPakOpen.len + pHciDesc->usPakOpen.offset),
			       (pMsg->MData.DataCB.BufferAddress + pMsg->MData.DataCB.Offset),
			       data_length);
			pHciDesc->usPakOpen.len += data_length;

			L2CAP_TRACE_PRINTF_4(L2CAP_TRACE_MASK_TRACE,
			                        "L2C fragmented us data packet expect %d adding %d now have %d",
			                        pHciDesc->usPakOpen.expected, data_length, pHciDesc->usPakOpen.len, 0);

		}   
		else /* there nothing we can do with this packet, no first packet found */
		{
			L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_ERROR,
			                        "!!L2C No Channel for continuation packet");
			return;           
		}
    }
    else
    {                                              /* start of L2CAP packet */
        if(pHciDesc->usPakOpen.len != 0) 	/*waiting for continue l2cap packet*/
		{
			L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_ERROR,
			                        "!!L2C Start of Pak while waiting for continuation");
			osBufferRelease(pHciDesc->usPakOpen.pd);
        }

        pHciDesc->usPakOpen.len      = data_length;
        pHciDesc->usPakOpen.expected = 0;             /* we do not yet know how much to expect */
        pHciDesc->usPakOpen.offset = pMsg->MData.DataCB.Offset;
        pHciDesc->usPakOpen.pd     = pMsg->MData.DataCB.BufferAddress;
        pMsg->MData.DataCB.Flag   &= ~DATA_CB_RELEASE;    /* do not release */
    }   /* start of L2CAP packet */

    if(pHciDesc->usPakOpen.expected == 0)     /* we do not yet know how much to expect */
    {
		if(pHciDesc->usPakOpen.len < L2CAP_HDR_LENGTH)
		{
			return;         /* thats all for now */
		}
		else    /* there should be enough bytes to get the header */
		{
			pPdu_hdr = pHciDesc->usPakOpen.pd + pHciDesc->usPakOpen.offset;
			Pdu_hdr.length  = CHAR2SHORT(pPdu_hdr);
			pPdu_hdr += 2;
			Pdu_hdr.cid     = CHAR2SHORT(pPdu_hdr);
			pHciDesc->usPakOpen.expected = Pdu_hdr.length + L2CAP_HDR_LENGTH;

			if (pHciDesc->conType == BT_CONNECTION_TYPE_LE)  /* LE with fixed channels, used also handle */
			{
				/* check for LE needed ????? */
				pHciDesc->usPakOpen.isLeChannel = l2cLESearchLCid(Pdu_hdr.cid);
#if F_BT_LE_BT41_SUPPORT
                if (pHciDesc->usPakOpen.isLeChannel == FALSE)
                {
                    pHciDesc->usPakOpen.pChan = l2cSearchLcid(Pdu_hdr.cid);
                    if ((pHciDesc->usPakOpen.pChan != NULL) && (Pdu_hdr.length >
                            (pHciDesc->usPakOpen.pChan->LocalMps)))
                    {
                    	osBufferRelease(pHciDesc->usPakOpen.pd);
                        pHciDesc->usPakOpen.pd = NULL;
                        pHciDesc->usPakOpen.expected = pHciDesc->usPakOpen.len = 0;
                        l2cCloseLEChannel(pHciDesc->usPakOpen.pChan, L2CAP_ERR_INVAILD_PDU | L2CAP_ERR);
                    }
                }
#endif
			}
#if F_BT_BREDR_SUPPORT
			else
			{
				pHciDesc->usPakOpen.pChan = l2cSearchLcid(Pdu_hdr.cid);

				if (Pdu_hdr.length > BT_US_PDU_L2C_BYTE_COUNT           /* signaled length > MAX MTU */
				 || (pHciDesc->usPakOpen.pChan != NULL && Pdu_hdr.length >
				    (pHciDesc->usPakOpen.pChan->LocalUsMtu + pHciDesc->usPakOpen.pChan->ExtFLength))) /* or > actual MTU */
				{
					uint8_t cmdid     = *(pHciDesc->usPakOpen.pd + pHciDesc->usPakOpen.offset + L2CAP_HDR_LENGTH + 1);

					L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_ERROR,
					                      "!! L2C DATA IND length > Max/Local MTU discarding data");
					osBufferRelease(pHciDesc->usPakOpen.pd);
					pHciDesc->usPakOpen.pd = NULL;
					pHciDesc->usPakOpen.expected = pHciDesc->usPakOpen.len = 0;

					if (Pdu_hdr.cid == CID_SIGNALING_BR)          /* packet contains L2CAP signaling info */
					{
						pHciDesc->ReqIdRcv = cmdid;
						l2cSendL2CAP_COMMAND_REJECT(NULL, pHciDesc, L2CAP_CMDREJ_MTU_EXCEEDED, L2CAP_SIGNAL_MTU, 0); /* response to remote side */
					}
					if (pHciDesc->usPakOpen.pChan != NULL)
					{     /* in case of Pdu_hdr.length > BT_US_MTU_L2C_BYTE_COUNT we may not have a valid channel */

					}
					return;
				} /* if( signaled length > MAX MTU ) */
			}
#endif
		}
    }

    if(pHciDesc->usPakOpen.expected != 0)
    {                                               /* we know the length for the packet    */
		if(pHciDesc->usPakOpen.expected <= pHciDesc->usPakOpen.len)
		{                                                           /* whole paket received */
			Pdu_hdr.length  = CHAR2SHORT(pHciDesc->usPakOpen.pd + pHciDesc->usPakOpen.offset);
			Pdu_hdr.cid     = CHAR2SHORT(pHciDesc->usPakOpen.pd + pHciDesc->usPakOpen.offset + 2);
#if F_BT_BREDR_SUPPORT
			if((Pdu_hdr.cid == CID_SIGNALING_BR)           /* packet contains L2CAP signaling info */
			|| (Pdu_hdr.cid == CID_SIGNALING_LE)
			   )
#else
            if(Pdu_hdr.cid == CID_SIGNALING_LE)
#endif
			{
				pHciDesc->signalingCID = Pdu_hdr.cid;      /* save */
				l2cHandleSigCmdPacket(pHciDesc,
				    pHciDesc->usPakOpen.pd + pHciDesc->usPakOpen.offset + L2CAP_HDR_LENGTH, Pdu_hdr.length);
			}
			else if(Pdu_hdr.cid == CID_CONNECTIONLESS)
			{      /* packet adresses connectionless data channel */
		    	L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_ERROR,
		                            "!!L2C Connection less data not supported !!");
			}
			else	 /* connection oriented channel, data channel*/
			{                            
			    if (pHciDesc->conType == BT_CONNECTION_TYPE_LE)
			    {
			        if (pHciDesc->usPakOpen.isLeChannel)
			        {
			            if(pHciDesc->usPakOpen.expected >= pHciDesc->usPakOpen.len)
			            {         /* signaled length greater or equal than received */
			                Msg.MData.DataCBChan.Flag          = DATA_CB_RELEASE;
			                Msg.MData.DataCBChan.Offset        = pHciDesc->usPakOpen.offset + L2CAP_HDR_LENGTH;
			                Msg.MData.DataCBChan.Length        = Pdu_hdr.length;
			                Msg.MData.DataCBChan.BufferAddress = pHciDesc->usPakOpen.pd;

			                l2cLESendDataInd(pHciDesc, Pdu_hdr.cid, &Msg);
			                pHciDesc->usPakOpen.pd = NULL;    /* set to NULL so buffer is not released */
			            }
			        }
#if F_BT_LE_BT41_SUPPORT
                    else
                    {
                        P_L2CAP_CHANNEL pChan = pHciDesc->usPakOpen.pChan;

                        if (pChan != NULL)
                        {
                            if (pChan->State == l2cStateOpen)
                            {
                                if (pHciDesc->usPakOpen.expected < pHciDesc->usPakOpen.len)
                                {
                                    /* signaled length less than received */
                                    L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_ERROR, "!!L2C HCI len > L2CAP len");
                                }
                                else
                                {
                                    L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_TRACE, "Receive LE frame");
                                    Msg.MData.DataCBChan.Flag        = DATA_CB_RELEASE;
                                    Msg.MData.DataCBChan.Channel     = pChan->LocalCid;
                                    Msg.MData.DataCBChan.Offset      = pHciDesc->usPakOpen.offset + L2CAP_HDR_LENGTH;
                                    Msg.MData.DataCBChan.Length      = Pdu_hdr.length;
                                    Msg.MData.DataCBChan.TotalLength = Pdu_hdr.length;
                                    Msg.MData.DataCBChan.BufferAddress = pHciDesc->usPakOpen.pd;

                                    l2cHandleUpstreamLEFrame(pChan, &Msg);
                                }
                            }
                            else
                            {
                                L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_ERROR, "!!L2C COC DATA IND channel not open");
                            }
                        }
                        else   /* no Channel for that CID found, discard the packet */
                        {
                            L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_ERROR, "!!L2C COC DATA IND unknown CID");
                        }
                    }
#endif
                }   /* connection oriented channel */
#if F_BT_BREDR_SUPPORT
			    else
			    {
			        P_L2CAP_CHANNEL pChan = pHciDesc->usPakOpen.pChan;

			        if(pChan != NULL) 
					{
			            if (pChan->State == l2cStateOpen || pChan->State == l2cStateConfig) 
						{
			              	if(pHciDesc->usPakOpen.expected < pHciDesc->usPakOpen.len)
							{         /* signaled length less than received */                       
							}
			              	else  /* recombination complete, we have a full PDU */
			              	{
								Msg.MData.DataCBChan.Flag        = DATA_CB_RELEASE;
								Msg.MData.DataCBChan.Channel     = pChan->LocalCid;
								Msg.MData.DataCBChan.Offset      = pHciDesc->usPakOpen.offset + L2CAP_HDR_LENGTH;
								Msg.MData.DataCBChan.Length      = Pdu_hdr.length;
								Msg.MData.DataCBChan.TotalLength = Pdu_hdr.length;
								Msg.MData.DataCBChan.BufferAddress = pHciDesc->usPakOpen.pd;
								
#if F_BT_L2C_ENHANCED_FEATURE_SUPPORT //champion
								if (pChan->Mode != L2CAP_MODE_BASIC)
								{

									uint16_t fcs   = 0;
									uint16_t inFcs = 0;

									if (pChan->Fcs)
									{
									  inFcs = CHAR2SHORT(&Msg.MData.DataCBChan.BufferAddress[Msg.MData.DataCBChan.Offset + (Pdu_hdr.length - L2CAP_FCS_LENGTH)]);
									  fcs = btxfcs(LFSR_LOAD_VALUE, Msg.MData.DataCBChan.BufferAddress + pHciDesc->usPakOpen.offset, (uint16_t)(Pdu_hdr.length - L2CAP_FCS_LENGTH + L2CAP_HDR_LENGTH));
									}

									if (inFcs != fcs)
									{
									  L2CAP_TRACE_PRINTF_2(L2CAP_TRACE_MASK_ERROR,
									                          "!!L2C wrong FCS %4.4x received (%4.4x)", inFcs, fcs);
									}
									else
									{
									  l2cHandleUpstreamPacket(pChan, &Msg);

									  pHciDesc->usPakOpen.pd = NULL;    /* set to NULL so buffer is not released */
									}

									L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_ERROR,
									                      "!!l2cHandleHCI_DATA_IND--pChan->Mode != L2CAP_MODE_BASIC");

								}
								else
#endif									
								{
									L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_TRACE,
										                        "L2C<- L2C_DATA_IND length %d", Pdu_hdr.length);							
									if(pChan->usQueueID == gattQueueID)
									{
										gattHandleAttData(0, &Msg, TRUE);
									}
                                    else if(pChan->usQueueID == sdpQueueID)
									{
										sdpHandleLDataInd(pHciDesc->usPakOpen.pd + pHciDesc->usPakOpen.offset + L2CAP_HDR_LENGTH, pChan->LocalCid, Pdu_hdr.length);
										osBufferRelease(pHciDesc->usPakOpen.pd);
									}
                                    else if(pChan->usQueueID < OS_FIRST_QUEUE_ID)
									{
										blueAPI_Send_L2cDataInd(&Msg, pChan->usQueueID);
									}
									pHciDesc->usPakOpen.pd = NULL;    /* set to NULL so buffer is not released */
								}
			              }
			            } 
			            else 
						{        /* this channel is not in CONFIG or OPEN state */
			                L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_ERROR,
			                                        "!!L2C COC DATA IND channel not open");
			            }
			        } /* channel found */
			        else 
					{ /* no Channel for that CID found, discard the packet */
			            L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_ERROR,
			                                    "!!L2C COC DATA IND unknown CID");
			        }

			    } /*  if (pHciDesc->conType == BT_CONNECTION_TYPE_LE) */
#endif
			}   /* connection oriented channel */
			if((pHciDesc->usPakOpen.pd) != NULL) 
			{
				osBufferRelease(pHciDesc->usPakOpen.pd);
			}
			pHciDesc->usPakOpen.pd = NULL;
			pHciDesc->usPakOpen.expected = pHciDesc->usPakOpen.len = 0;
		}   /* we got a complete packet */
    } /* we know a length for the packet    */
}




/*=========================================================================
 *
 *  l2cHCIDATACallBack()
 *
 *  Purpose:
 *      CallBack function for Downstream fragmentation pool
 *
 *  Parameters:
 *      [handle] : pointer to the descriptor of the HCI channel
 *
 *  Returns:    none
 *
 *  Note :
 *
 *=======================================================================*/

void l2cHCIDATACallBack(uint32_t handle)
{
    P_ACLPOOLDESC pHciDesc = (P_ACLPOOLDESC)handle;

    L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_TRACE,
                            "L2C-> l2cHCIDATACallBack() HciDesc 0x%X", pHciDesc);
    l2cFragmentDATA_REQ(pHciDesc);
}

/*=========================================================================
 *
 *  l2cFragmentDATA_REQ()
 *
 *  Purpose:
 *      performs the fragmentation of downstream data packets if needed
 *
 *  Parameters:
 *      [pHciDesc] : pointer to the descriptor of the HCI channel
 *
 *  Returns:    none
 *
 *  Note :
 *
 *=======================================================================*/

void l2cFragmentDATA_REQ(P_ACLPOOLDESC pHciDesc)
{
    uint16_t dsMTU;
    uint16_t bytesToAlloc;
    MESSAGE_T msg;
    uint8_t * myBuf;
    BOOL firstPak;
    uint16_t ACLpayloadSize;
    int             s;

#if F_BT_BREDR_SUPPORT
    if (pHciDesc->conType == BT_CONNECTION_TYPE_LE)
      dsMTU = pL2c->leDsMtu;
    else
      dsMTU = pL2c->dsMtu;
#else
    dsMTU = pL2c->leDsMtu;
#endif

    bytesToAlloc   = dsMTU + L2C_WRITE_OFFSET;
    ACLpayloadSize = dsMTU;

    for(;;)
    {
        ACLpayloadSize = dsMTU;
        s = osInterruptDisable();
        if(pHciDesc->dsMessage.MData.DataCB.BufferAddress != NULL)
        {
            L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_TRACE,
                                    "L2C-> l2cFragmentDATA_REQ() pak in progress");
        }
        else
        {
            uint16_t elemCount;
            uint16_t prioElemCount;

            osMessageQueueElementCountGet(pHciDesc->dsPrioBufQueue, &prioElemCount);

            if(prioElemCount == 0)
            {
              osMessageQueueElementCountGet(pHciDesc->dsBufQueue, &elemCount);

              if(elemCount == 0)
              {
                  osInterruptEnable(s);
                  return;
              }
            }
            else
            {

            }

            if(osMessageReceive((uint8_t) (prioElemCount ? pHciDesc->dsPrioBufQueue : pHciDesc->dsBufQueue),
                                &pHciDesc->dsMessage) != 0)
            {
                osInterruptEnable(s);
                return;
            }
            pHciDesc->dsMessage.MData.DataCB.Flag &= ~DATA_CB_MORE; /* clear the more flag - start of pak */
        }

        firstPak = pHciDesc->dsMessage.MData.DataCB.Flag & DATA_CB_MORE ? FALSE : TRUE;
        if(firstPak)
        {
            ACLpayloadSize -= L2CAP_HDR_LENGTH; /* the first packet has to carry L2CAP header */
        }

        if(pHciDesc->dsMessage.MData.DataCB.Length > ACLpayloadSize) { /* needs fragmentation */
            if(osBufferGetCallBack(pL2c->dsAclPoolID, bytesToAlloc, (void  *)&myBuf,
                                    l2cHCIDATACallBack, (uint32_t)pHciDesc)) {
                L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_TRACE,
                                        "l2cFragmentDATA_REQ Could not allocate memory %d retrying", bytesToAlloc);
                return;
            }

            L2CAP_TRACE_PRINTF_3(L2CAP_TRACE_MASK_TRACE,
                                    "L2C-> l2cFragmentDATA_REQ() pak chunk on HCI chan 0x%X  RemoteCID 0x%x length %d",
                                    pHciDesc->handle,
                                    pHciDesc->dsMessage.MData.DataCBChan.Channel,
                                    ACLpayloadSize);

            msg.MData.DataCB.BufferAddress = myBuf;
            msg.MData.DataCB.Offset        = L2C_WRITE_OFFSET;
            msg.MData.DataCB.Flag          = pHciDesc->dsMessage.MData.DataCB.Flag | DATA_CB_RELEASE;
            msg.MData.DataCB.Length        = ACLpayloadSize;

            memcpy(myBuf + L2C_WRITE_OFFSET,
                pHciDesc->dsMessage.MData.DataCB.BufferAddress + pHciDesc->dsMessage.MData.DataCB.Offset, ACLpayloadSize);

            l2cSendL2CAP_PDU(pHciDesc->handle, &msg,
                                    pHciDesc->dsMessage.MData.DataCB.Length,
                                    pHciDesc->dsMessage.MData.DataCBChan.Channel,
                                    firstPak);

            pHciDesc->dsMessage.MData.DataCB.Offset += ACLpayloadSize;
            pHciDesc->dsMessage.MData.DataCB.Length -= ACLpayloadSize;
            pHciDesc->dsMessage.MData.DataCB.Flag   |= DATA_CB_MORE; /* set the more flag - continuation pak */
        }
        else
        {
#if 1
            l2cSendL2CAP_PDU(pHciDesc->handle, &pHciDesc->dsMessage,
                                    pHciDesc->dsMessage.MData.DataCB.Length,
                                    pHciDesc->dsMessage.MData.DataCBChan.Channel,
                                    firstPak);
            pHciDesc->dsMessage.MData.DataCB.BufferAddress = NULL;
#endif
#if 0
            THandle handle;
            handle.pHandle = pHciDesc;
            bytesToAlloc   = pHciDesc->dsMessage.MData.DataCB.Length + L2C_WRITE_OFFSET;

            if (osBufferGetCallBack(pL2c->dsAclPoolID, bytesToAlloc, (void  *)&myBuf,
                                    l2cHCIDATACallBack, handle))
            {
                L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_TRACE,
                                     "l2cFragmentDATA_REQ Could not allocate memory %d retrying", bytesToAlloc);
                osInterruptEnable(s);
                return;
            }
            L2CAP_TRACE_PRINTF_3(L2CAP_TRACE_MASK_TRACE,
                                 "L2C-> l2cFragmentDATA_REQ() last pak chunk on HCI chan 0x%X  RemoteCID 0x%x length %d",
                                 pHciDesc->handle,
                                 pHciDesc->dsMessage.MData.DataCBChan.Channel,
                                 pHciDesc->dsMessage.MData.DataCB.Length);
            memcpy(myBuf + L2C_WRITE_OFFSET,
                   pHciDesc->dsMessage.MData.DataCB.BufferAddress + pHciDesc->dsMessage.MData.DataCB.Offset, pHciDesc->dsMessage.MData.DataCB.Length);

            msg.MData.DataCB.BufferAddress = myBuf;
            msg.MData.DataCB.Offset        = L2C_WRITE_OFFSET;
            msg.MData.DataCB.Flag          = pHciDesc->dsMessage.MData.DataCB.Flag | DATA_CB_RELEASE;
            msg.MData.DataCB.Length        = pHciDesc->dsMessage.MData.DataCB.Length;
        
            l2cSendL2CAP_PDU(pHciDesc->handle, &msg,
                             pHciDesc->dsMessage.MData.DataCB.Length,
                             pHciDesc->dsMessage.MData.DataCBChan.Channel,
                             firstPak);
            
            osBufferRelease(pHciDesc->dsMessage.MData.DataCB.BufferAddress);
            pHciDesc->dsMessage.MData.DataCB.BufferAddress = NULL;
#endif
        }
        osInterruptEnable(s);
    } /* while(1) */
}


/**
* @brief	 l2cap handle hci disconnect indicate
*
* @param	handle: 
* @param	cause:
* @param	conType:
*
* @return
*
*/
#if F_BT_BREDR_SUPPORT
void l2cHandleHciDiscInd(uint16_t handle, uint16_t cause, uint8_t conType)
{
	P_L2CAP_CHANNEL pChan;
	P_ACLPOOLDESC pHciDesc;

	L2CAP_TRACE_PRINTF_2(L2CAP_TRACE_MASK_TRACE,
	                    "L2C-> HCI_DISC_IND handle 0x%X cause %d",
	                    handle, cause);

	while((pChan = l2cSearchHciHandle(handle)) != NULL)	/* there are still channels on this HCI handle */
	{    
		switch (pChan->State)
		{
	    case l2cStateClosed:
	        if(pChan->OpenL2cDiscConf == TRUE)
	        {
	            l2cSendL2C_DISC_CONF(pChan->usQueueID, pChan->LocalCid, pChan->status);
	        }
	        l2cRemoveChan(pChan, TRUE);
	        break;

	    case l2cStateWaitForL2C_DISC_RESP:
	        l2cDeleteChannel(pChan, TRUE);
	        break;

	    default:
	        l2cStopRTXTimeout(pChan);
	        l2cStopCONFIGTimeout(pChan);   /* stop timeout */
			l2cChangeState(pChan, l2cStateClosed);
	        l2cSendL2C_DISC_IND(pChan, cause);
	        break;
		}
	}

	pHciDesc = l2cSearchHciDescByHciHandle(handle);
	if(pHciDesc != NULL)
	{
		L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_TRACE,"Close Hci channel");
		l2cFreeHciDesc(pHciDesc);
	}
	else
	{
		L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_TRACE,"Hci channel not existent");
	}
} /* l2cHandleHciDiscInd */
#endif
/**
* @brief   l2cap handle timeout
*
* @param  pChan
*
* @return  
*
*/
#if (F_BT_BREDR_SUPPORT)||(F_BT_LE_BT41_SUPPORT)
void l2cHandleTimeout(P_L2CAP_CHANNEL pChan)
{
	L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_ERROR,
	                      "!!L2CAP time out in %d", pChan->State);

	switch (pChan->State)
	{
	case l2cStateWaitForL2CAP_DISCONNECTION_RESPONSE:
#if F_BT_LE_BT41_SUPPORT
        if(pChan->pHciDesc->conType == BT_CONNECTION_TYPE_LE)
        {
            if(pChan->WaitDisRsp)
            {
                l2cSend_DisconnectLEDataChannelRsp(pChan->LocalCid, pChan->pHciDesc->handle, L2CAP_ERR_TIMEOUT_EXTERNAL | L2CAP_ERR);
                pChan->WaitDisRsp = FALSE;
            }

            l2cChangeState(pChan, l2cStateClosed);
        }
#endif
#if F_BT_BREDR_SUPPORT
        if(pChan->pHciDesc->conType != BT_CONNECTION_TYPE_LE)
        {
    		pChan->status = L2CAP_ERR_TIMEOUT_EXTERNAL | L2CAP_ERR;
    		pChan->OpenL2cDiscConf = pChan->ACLDiscImmediate;
    		if(pChan->ACLDiscImmediate == FALSE || pChan->pHciDesc->uses != 1)
    		{
    			l2cSendL2C_DISC_CONF(pChan->usQueueID, pChan->LocalCid, pChan->status);
    		}
    		else
    		{
    			pChan->OpenL2cDiscConf = TRUE;
    		}
        }
#endif
		break;
#if F_BT_BREDR_SUPPORT
	case l2cStateWaitForL2CAP_CONNECTION_RESPONSE:
		switch(pChan->lastReqSend)
		{
		case L2CAP_CONNECTION_REQUEST:            /* connection request timed out - this is an error close connection etc */
			l2cChangeState(pChan, l2cStateClosed);
			l2cSendL2C_DISC_IND(pChan, L2CAP_ERR_TIMEOUT_EXTERNAL | L2CAP_ERR);

		default:
			assert(FALSE);
		}
		break;
#endif
#if F_BT_LE_BT41_SUPPORT
    case l2cStateWaitForL2CAP_LE_CREDIT_BASED_CONNECTION_RESPONSE:
        switch (pChan->lastReqSend)
        {
        case L2CAP_LE_CREDIT_BASED_CONNECTION_REQUEST:
            l2cSend_CreateLEDataChannelRsp(0, pChan->pHciDesc->handle, L2CAP_ERR_TIMEOUT_EXTERNAL | L2CAP_ERR);
            l2cChangeState(pChan, l2cStateClosed);
            break;
        }
        break;
#endif
	case l2cStateClosed:
		l2cRemoveChan(pChan, FALSE);
		break;

	default:
		l2cStopRTXTimeout(pChan);
#if F_BT_BREDR_SUPPORT
		l2cStopCONFIGTimeout(pChan);   /* stop timeout */
#endif
		l2cSendL2C_DISC_IND(pChan, L2CAP_ERR_TIMEOUT_EXTERNAL | L2CAP_ERR);

		l2cSendL2CAP_DISCONNECTION_REQUEST(pChan);
		break;
	}
}
#endif
/**
* @brief  l2cap timer expired
*
* @param  TimerID:
* @param  TimerChannel
*
* @return  
*
*/
void l2cHandleTIMER_EXPIRED(uint8_t TimerID, uint16_t TimerChannel)
{
#if (F_BT_BREDR_SUPPORT)||(F_BT_LE_BT41_SUPPORT)
    P_L2CAP_CHANNEL pChan;
#endif
	switch(TimerID) 
	{
    case L2CAP_RTX_TID:
		L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_TRACE,
		                        "l2cHandleTIMER_EXPIRED() L2CAP_RTX_TID on channel 0x%X",
		                        TimerChannel);
#if (F_BT_BREDR_SUPPORT)||(F_BT_LE_BT41_SUPPORT)
		pChan = l2cSearchCmdId(TimerChannel);
        if(pChan != NULL) 
		{
#if F_BT_BREDR_SUPPORT
            if (pChan->L2capInformationRequestSent)
            {
                l2cCheckMode(pChan);  /* simulate info resp */
            }
            else
#endif
            {
#if (BT_L2C_MAX_CMD_RETRIES_COUNT != 0)  /*[*/
                if(pChan->ReqCmdRetryCount--) 
				{
                    l2cReSendL2CAPMessage(pChan, &pChan->lastReqSend);
                }
                else
                {
                    l2cHandleTimeout(pChan);
                }
#else 
                l2cHandleTimeout(pChan);
#endif
            }
        }
        else
#endif
        {
			P_ACLPOOLDESC pHciDesc;

			pHciDesc = l2cSearchHciDescBySigIdSent(TimerChannel);
			if (pHciDesc != (P_ACLPOOLDESC)0)
			{
				l2cLEHandleTimeout(pHciDesc);
			}
        }

        break;
#if F_BT_BREDR_SUPPORT
    case L2CAP_CONFIG_TID:
        L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_TRACE,
                                "l2cHandleTIMER_EXPIRED() L2CAP_CONFIG_TID on channel0x%X",
                                TimerChannel);
        pChan = l2cSearchLcid(TimerChannel);
        if(pChan != NULL) 
		{
            l2cHandleTimeout(pChan);
        }
        break;
    case CLOSEDELAY_TID:
        L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_TRACE,
                                "l2cHandleTIMER_EXPIRED() L2CAP_CLOSEDELAY_TID on channel0x%X",
                                TimerChannel);
        l2cRemoveHCI(NULL, TimerChannel);
        break;
#if F_BT_L2C_ENHANCED_FEATURE_SUPPORT
    case L2CAP_MONITOR_TID:
        L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_TRACE,
                                "l2cHandleTIMER_EXPIRED() L2CAP_MONITOR_TID on channel0x%X",
                                TimerChannel);
        pChan = l2cSearchLcid(TimerChannel);
        if (pChan != NULL)
		{
			l2cHandleTIMER_EXPIREDMonitor(pChan);
		}
        break;
    case L2CAP_RETRANSMISSION_TID:
        L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_TRACE,
                                "l2cHandleTIMER_EXPIRED() L2CAP_RETRANSMISSION_TID on channel0x%X",
                                TimerChannel);
        pChan = l2cSearchLcid(TimerChannel);
        if (pChan != NULL)
		{
			l2cHandleTIMER_EXPIREDRetransmission(pChan);
		}
        break;
#endif
#if defined(L2CAP_ACK_TID)
    case L2CAP_ACK_TID:
        L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_TRACE,
                                "l2cHandleTIMER_EXPIRED() L2CAP_ACK_TID on channel0x%X",
                                TimerChannel);
        pChan = l2cSearchLcid(TimerChannel);
        if (pChan != NULL)
          l2cHandleTIMER_EXPIREDAck(pChan);
        break;
#endif
#endif
    default:
        L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_ERROR,
                                "!!l2cHandleTIMER_EXPIRED() got UNKNOWN Timer ID 0x%X !!!!",
                                TimerID);
        break;
    }

} /* l2cHandleTIMER_EXPIRED() */

void l2cHandleHciInitCompleted(uint16_t leDsAclSize, uint16_t status, uint16_t aclSize)
{
    L2CAP_TRACE_PRINTF_3(L2CAP_TRACE_MASK_TRACE,
                        "L2C-> HCI_RESET_IND status %X max pak size %d le size %d",
                        status, aclSize, leDsAclSize);
    gL2cDsFragmentationSupport = FALSE;
    
    if(status == 0)
    {
#if F_BT_BREDR_SUPPORT
        pL2c->dsMtu = aclSize;
        if ((BT_DS_PDU_L2C_BYTE_COUNT + L2CAP_HDR_LENGTH) > pL2c->dsMtu)
        {
            gL2cDsFragmentationSupport = TRUE;
        }
#endif        
        pL2c->leDsMtu = leDsAclSize; 
        if ((otp_str_data.gEfuse_UpperStack_s.att_max_mtu_size  + L2CAP_HDR_LENGTH) > pL2c->leDsMtu) 
        {
            gL2cDsFragmentationSupport = TRUE;
        }

#if F_BT_LE_BT41_SUPPORT
        if((otp_str_data.gEfuse_UpperStack_s.num_l2c_chan_don != 0)||(otp_str_data.gEfuse_UpperStack_s.num_l2c_chan_doff != 0))
        {
            gL2cDsFragmentationSupport = TRUE;
        }
#endif

        if (TRUE == gL2cDsFragmentationSupport)
		{
			unsigned i;
			for(i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_don; i++)    /* now search for a free descriptor */
			{

				if(osMessageQueueCreate(&pL2c->pAclDescDon[i].dsBufQueue))
				{
				  L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_ERROR,
				                          "!!: l2cHandleHCI_RESET_IND() Could not create queue");
				  assert(FALSE);
				}
			    else
			    {
					if(osMessageQueueCreate(&pL2c->pAclDescDon[i].dsPrioBufQueue))
					{
						L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_ERROR,
						                        "!!: l2cHandleHCI_RESET_IND() Could not create Prio queue");
						assert(FALSE);
					}
					else
					{
						L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_TRACE,
						                        "l2cHandleHCI_RESET_IND() created new DSqueue");
					}
			    }
				
			} /* for all ACL descriptors */
            for(i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_doff; i++)    /* now search for a free descriptor */
			{
				if(osMessageQueueCreate(&pL2c->pAclDescDoff[i].dsBufQueue))
				{
				  L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_ERROR,
				                          "!!: l2cHandleHCI_RESET_IND() Could not create queue");
				  assert(FALSE);
				}
			    else
			    {
					if(osMessageQueueCreate(&pL2c->pAclDescDoff[i].dsPrioBufQueue))
					{
						L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_ERROR,
						                        "!!: l2cHandleHCI_RESET_IND() Could not create Prio queue");
						assert(FALSE);
					}
					else
					{
						L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_TRACE,
						                        "l2cHandleHCI_RESET_IND() created new DSqueue");
					}
			    }

			} /* for all ACL descriptors */
		}
    }
}

/**
* @brief  l2cap entry
* 
* @param  lpL2C
*
* @return  
*
*/
void  l2cEntry(TL2C  *lpL2C)
{
    PL2C   pL2C = (PL2C)(lpL2C);

    if (osMessageReceive(l2cQueueID, &pL2C->Message) == 0) 
    {
        switch (pL2C->Message.Command)
        {
        /* handle hci data*/
        case HCI_DATA_IND:
            l2cHandleHCI_DATA_IND();
            if (pL2C->Message.MData.DataCB.Flag & DATA_CB_RELEASE)
            {
                osBufferRelease(pL2C->Message.MData.DataCB.BufferAddress);
            }
            break;

        default:
            L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_ERROR,
                                "!!l2cEntry() UNKNOWN message command 0x%X !!!!!!!!",
                                pL2C->Message.Command);
        break;
        } /* switch */
    } /* if */
} /* l2cEntry */

/**
* @brief  init l2cap 
* 
* @param DataSegment: pL2c memory
* @param TaskName: task name
*
* @return  init result
*
*/
BOOL l2cInit(void)
{
    pL2c = osMemoryClearAllocate(RAM_TYPE_DATA_ON, sizeof(TL2C));
    
    pL2c->dsAclPoolID = DownstreamPoolID;
    
    pL2c->pAclDescDon = osMemoryAllocate(RAM_TYPE_DATA_ON, sizeof(T_ACLPOOLDESC)*otp_str_data.gEfuse_UpperStack_s.num_link_don);
    pL2c->pAclDescDoff = osMemoryAllocate(RAM_TYPE_DATA_OFF, sizeof(T_ACLPOOLDESC)*otp_str_data.gEfuse_UpperStack_s.num_link_doff);

#if (F_BT_BREDR_SUPPORT)||(F_BT_LE_BT41_SUPPORT)
    pL2c->L2cap_next_cmdID = 1;
    pL2c->pLLDataChanDon = osMemoryAllocate(RAM_TYPE_DATA_ON, sizeof(T_L2CAP_CHANNEL)*otp_str_data.gEfuse_UpperStack_s.num_l2c_chan_don);
    pL2c->pLLDataChanDoff = osMemoryAllocate(RAM_TYPE_DATA_OFF, sizeof(T_L2CAP_CHANNEL)*otp_str_data.gEfuse_UpperStack_s.num_l2c_chan_doff);
#endif

#if F_BT_BREDR_SUPPORT
    pL2c->uplDesc = osMemoryAllocate(RAM_TYPE_DATA_ON, sizeof(TupperlayerDesc)*otp_str_data.gEfuse_UpperStack_s.num_data_psm);
#endif

    osCoroutineCreate(&l2cQueueID, (TCoroutineEntry)l2cEntry, pL2c);
    
#if F_BT_LE_BT41_SUPPORT
    l2cInitChannelList();
#endif
#if F_BT_BREDR_SUPPORT
#if F_BT_L2C_ENHANCED_FEATURE_SUPPORT   //champion
    {
        int  i;

        for (i = 0; i < L2CAP_MAX_ENHANCED_FEATURE_CHANNELS; i++)
        {
            osMessageQueueCreate(&pL2c->ExtF[i].TxQueueID);
            osMessageQueueCreate(&pL2c->ExtF[i].SentQueueID);
            osMessageQueueCreate(&pL2c->ExtF[i].RxQueueID);
        }

        if (osPoolCreate(&pL2c->SFramePool, RAM_TYPE_DATA_OFF, TRUE,
            L2CAP_SFRAME_LENGTH + L2CAP_SFRAME_OFFSET, L2CAP_SFRAME_BUFFER, 4))
        {
            DebuggerBreak();
            return(FALSE);
        }
    }
#endif
#endif    
    return(TRUE);
} /* l2cInit */

#if !F_BT_BREDR_SUPPORT 

void l2cHandleBTG_DISC_REQ(uint16_t cid, BOOL holdLink)
{
}
void l2cHandleL2C_CON_RESP(uint16_t cid, uint16_t status, PBtConRespPSpecifc p)
{
}

void l2cHandleL2C_DATA_REQ(MESSAGE_P  pmsg, BOOL prio)
{
}
uint16_t l2cUSendListenReq(uint16_t psm, uint8_t listenerQueue, uint8_t action)
{
	return 0;
}
void l2cUSendConReq(uint8_t RequesterQueue, uint8_t * remoteBd, uint16_t psm, uint16_t uuid, uint16_t frameSize,LP_L2C_Conf_para pConfPara)
{
}

#endif

#if !((F_BT_BREDR_SUPPORT)||(F_BT_LE_BT41_SUPPORT))
void l2cHandleL2C_DISC_RESP(uint16_t lcid)
{
}
#endif

