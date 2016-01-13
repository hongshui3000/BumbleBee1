/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       hci_le.c
* @brief     HCI Protocol Layer (Low Energy)
* @details   
*
* @author  	gordon
* @date      	2015-07-09
* @version	v0.1
*/

#include <flags.h>
#include <hci_code.h>
#include <os_pool.h>
#include <btcommon.h>
#include <hci.h>
#include <hci_br.h>
#include <hci_api.h>
#include <btsm_api.h>
#include <l2c_api.h>
#include <blueapi_api.h>
#include <upper_stack_global.h>

#define TRACE_MODULE_ID     MID_BT_HCI

/**
* @brief   send HCI_WRITE_EXTENDED_INQUIRY_RESPONSEc ommand
*
* @param  fec: if need fec
* @param  extendedResult: extended inq response data
*
* @return  
*
*/
void hciCommandWriteExtendedInquiryResponse(uint8_t fec, uint8_t * extendedResult)
{
    uint8_t * mbuf;
    uint16_t   pos    = pHCI->WriteOffset;
    uint8_t   length = BLUEFACE_EXTENDED_INQUIRY_RESPONSE_SIZE +1;

    if (osBufferGet(BTSystemPoolID, (uint16_t)(pHCI->WriteOffset + HCI_OFFSET + length), (PVOID  *)&mbuf))
    {
		assert(FALSE);
		return;
    }

    mbuf[pos++] = CMD_PKT;
    SHORT2CHAR(mbuf+pos, HCI_WRITE_EXTENDED_INQUIRY_RESPONSE); pos += 2;
    mbuf[pos++] = length;

    mbuf[pos++] = fec;
    memcpy(mbuf+pos,extendedResult,BLUEFACE_EXTENDED_INQUIRY_RESPONSE_SIZE);
    pos += BLUEFACE_EXTENDED_INQUIRY_RESPONSE_SIZE;

    hciSendDsMessageFromBuffer(pHCI->hciQueueID, pHCI->WriteOffset, mbuf, (uint16_t)(pos - pHCI->WriteOffset));
}

/**
* @brief  send HCI_SNIFF_MODE command
*
* @param  handle: ALC handle
* @param  max: min sniff interval
* @param  min: max sniff interval
* @param  attempt: attempt
* @param  timeOut: timeout
*
* @return  
*
*/
void hciCommandSniffMode(uint16_t handle, uint16_t max, uint16_t min, uint16_t attempt, uint16_t timeout)
{
    uint8_t mbuf[HCI_MAX_SHORT_CMD_SIZE];
    uint16_t pos = 0;

    SHORT2CHAR(mbuf+pos, handle);                            pos += 2;
    SHORT2CHAR(mbuf+pos, max);                               pos += 2;
    SHORT2CHAR(mbuf+pos, min);                               pos += 2;
    SHORT2CHAR(mbuf+pos, attempt);                           pos += 2;
    SHORT2CHAR(mbuf+pos, timeout);                           pos += 2;

    hciSendDsCommand(pHCI->hciTransQueueID, HCI_SNIFF_MODE, mbuf, pos);
}  

/**
* @brief  send HCI_SNIFF_SUBRATING command
*
* @param  handle: ALC handle
* @param  maxLatency: max latency
* @param  minRemoteTimeout: remote min timeout
* @param  minLocalTimeout: local min timout
*
* @return  
*
*/ 
void hciCommandSniffSubrating(uint16_t handle, uint16_t maxLatency, uint16_t minRemoteTimeout, uint16_t minLocalTimeout)
{
	uint8_t mbuf[HCI_MAX_SHORT_CMD_SIZE];
	uint16_t pos = 0;

	SHORT2CHAR(mbuf+pos, handle);           pos += 2;
	SHORT2CHAR(mbuf+pos, maxLatency);       pos += 2;
	SHORT2CHAR(mbuf+pos, minRemoteTimeout); pos += 2;
	SHORT2CHAR(mbuf+pos, minLocalTimeout);  pos += 2;

	hciSendDsCommand(pHCI->hciQueueID, HCI_SNIFF_SUBRATING, mbuf, pos);
}

void hciCommandIoCapabilityRequestReply(uint8_t * bd, uint8_t capability,
                                               uint8_t oob_data_present, uint8_t auth_requirements)
{
    uint8_t mbuf[HCI_MAX_SHORT_CMD_SIZE];
    uint16_t pos = 0;

    memcpy(mbuf+pos, bd, BD_ADDR_SIZE);  pos += BD_ADDR_SIZE;
    mbuf[pos++] = capability;
    mbuf[pos++] = oob_data_present;
    mbuf[pos++] = auth_requirements;

    hciSendDsCommand(pHCI->hciQueueID, HCI_IO_CAPABILITY_REQUEST_REPLY, mbuf, pos);
}

/**
* @brief  send HCI_CREATE_CONNECTION command
*
* @param  bd_addr: 
* @param  packetType
* @param  pageScanRepMode
* @param  pageScanMode
* @param  clockOffset
*
* @return  
*
*/
void hciCommandConnectReq(uint8_t * bd_addr, uint16_t packetType,
                          uint8_t pageScanRepMode, uint8_t pageScanMode, uint16_t clockOffset)
{
	uint8_t mbuf[HCI_MAX_SHORT_CMD_SIZE];
	uint16_t pos = 0;

	memcpy(mbuf + pos, bd_addr, BD_ADDR_SIZE); pos += BD_ADDR_SIZE;
	SHORT2CHAR(mbuf + pos, packetType); pos += 2;
	mbuf[pos++] = pageScanRepMode;
	mbuf[pos++] = pageScanMode;
	SHORT2CHAR(mbuf + pos, clockOffset); pos += 2;
	if ( pHCI->piconetType == HCI_PICONET_MASTER_REQUIRED
        || pHCI->piconetType == HCI_PICONET_SLAVE_REQUIRED /* SLAVE_REQUIRED - prevent switching initiated by remote site before connection complete */
	    || (pHCI->capabilities & HCI_ENABLE_ROLE_SWITCH_NOT_SUPPORTED))
	{
    	mbuf[pos++] =  0;  /* do not allow role switch */
	}
	else
	{
    	mbuf[pos++] =  1;  /* allow role switch */
	}

	hciSendDsCommand(pHCI->hciTransQueueID, HCI_CREATE_CONNECTION, mbuf, pos);
}

/**
* @brief  send HCI_INQUIRY command
*
* @param  TimeOut: 
* @param  maxHdl: if inquiry cancel
* @param  idxIAC
*
* @return
*
*/
void hciCommandInquiry(uint8_t idxIAC, uint8_t timeout, uint8_t maxresult)
{
    uint8_t mbuf[HCI_MAX_SHORT_CMD_SIZE];
    uint16_t pos = 0;

	if(idxIAC == 0) 						
	{
		mbuf[pos++] = 0x33;
	}
    else if(idxIAC == 1)	/*limit discovery*/
	{
		mbuf[pos++] = 0x00;
	}
	mbuf[pos++] = 0x8b;
	mbuf[pos++] = 0x9e;
    mbuf[pos++] = timeout;
    mbuf[pos++] = maxresult;

    hciSendDsCommand(pHCI->hciTransQueueID, HCI_INQUIRY, mbuf, pos);
}

/**
* @brief  send HCI_LINK_KEY_REQUEST_REPLY command 
*
* @param  bd: bdaddr
* @param  key: link key
*
* @return  
*
*/
void hciCommandLinkKeyRequestReply(uint8_t * bd, uint8_t * key)
{
    uint8_t mbuf[HCI_MAX_SHORT_CMD_SIZE];
    uint16_t pos = 0;

    memcpy(mbuf+pos, bd, BD_ADDR_SIZE);                   pos += BD_ADDR_SIZE;
    memcpy(mbuf+pos, key, LINK_KEY_SIZE);                 pos += LINK_KEY_SIZE;

    hciSendDsCommand(pHCI->hciQueueID, HCI_LINK_KEY_REQUEST_REPLY, mbuf, pos);
}

/**
* @brief  send HCI_PIN_CODE_REQUEST_REPLY command 
*
* @param  bd: bdaddr
* @param  key: pin
* @param  length: pin length
*
* @return  
*
*/
void hciCommandPinCodeRequestReply(uint8_t * bd, uint8_t * key, uint8_t length)
{
    uint8_t mbuf[HCI_MAX_SHORT_CMD_SIZE];
    uint16_t pos = 0;

    memcpy(mbuf+pos, bd, BD_ADDR_SIZE);                   pos += BD_ADDR_SIZE;
    mbuf[pos++] = length;
    memcpy(mbuf+pos, key, LINK_KEY_SIZE);                 pos += LINK_KEY_SIZE;

    hciSendDsCommand(pHCI->hciQueueID, HCI_PIN_CODE_REQUEST_REPLY, mbuf, pos);
}

/**
* @brief  send HCI_USER_PASSKEY_REQUEST_REPLY command
*
* @param  bd
* @param  value
*
* @return  
*
*/
void hciCommandUserPasskeyRequestReply(uint8_t * bd, uint32_t value)
{
	uint8_t mbuf[HCI_MAX_SHORT_CMD_SIZE];
	uint16_t pos = 0;

	memcpy(mbuf+pos, bd, BD_ADDR_SIZE);  pos += BD_ADDR_SIZE;
	LONG2CHAR(mbuf+pos, value);          pos += 4;

	hciSendDsCommand(pHCI->hciQueueID, HCI_USER_PASSKEY_REQUEST_REPLY, mbuf, pos);
}

/**
* @brief  send HCI_REMOTE_OOB_DATA_REQUEST_REPLY command
*
* @param  bd
* @param  C
* @param  R
*
* @return  
*
*/
void hciCommandRemoteOOBDataRequestReply(uint8_t * bd, uint8_t * C, uint8_t * R)
{
    uint8_t mbuf[BD_ADDR_SIZE + 16 + 16];
    uint16_t pos = 0;

    memcpy(mbuf+pos, bd, BD_ADDR_SIZE);  pos += BD_ADDR_SIZE;
    memcpy(mbuf+pos, C, 16);             pos += 16;
    memcpy(mbuf+pos, R, 16);             pos += 16;

    hciSendDsCommand(pHCI->hciQueueID, HCI_REMOTE_OOB_DATA_REQUEST_REPLY, mbuf, pos);
}

void hciCommandChangeLocalName(uint8_t * localName, uint16_t len)
{
    uint8_t * mbuf;
    uint16_t   pos = pHCI->WriteOffset;
    uint8_t   llen = NAME_LENGTH;

    /* check if short is possible */
    if (pHCI->capabilities & HCI_SHORT_SET_LOCAL_NAME_SUPPORTED)
    {
        llen = (uint8_t) len;
    }

    if (osBufferGet(BTSystemPoolID, (uint16_t)(pHCI->WriteOffset + HCI_OFFSET + llen), (PVOID  *)&mbuf))
    {
      assert(FALSE);
      return;
    }

    mbuf[pos++] = CMD_PKT;
    SHORT2CHAR(mbuf+pos, HCI_CHANGE_LOCAL_NAME); pos += 2;
    mbuf[pos++] = llen;                   /* length */

    if (localName)
    {
        memcpy(mbuf+pos,localName, len);  /* copy only provided length */
    }
    pos += llen;                          /* packet length is always full length */

    hciSendDsMessageFromBuffer(pHCI->hciQueueID, pHCI->WriteOffset, mbuf, (uint16_t)(pos - pHCI->WriteOffset));
}

/**
* @brief		send HCI_REMOTE_NAME_REQUEST command
*
* @param	bd: 
* @param	pageScanRepMode
* @param	pageScanMode
* @param	clockOffset
*
* @return
*
*/
void hciCommandRemoteNameRequest(uint8_t * bd, uint8_t pageScanRepMode, uint8_t pageScanMode, uint16_t clockOffset)
{
    uint8_t mbuf[HCI_MAX_SHORT_CMD_SIZE];
    uint16_t pos = 0;

    memcpy(mbuf+pos, bd, BD_ADDR_SIZE);            pos += BD_ADDR_SIZE;
    mbuf[pos++] = pageScanRepMode;
    mbuf[pos++] = pageScanMode;
    SHORT2CHAR(mbuf+pos, clockOffset);             pos += 2;

    hciSendDsCommand(pHCI->hciTransQueueID, HCI_REMOTE_NAME_REQUEST, mbuf, pos);
}

/**
* @brief  send HCI_WRITE_CURRENT_IAC_LAP command
*
* @param  numCurrentIAC: 
*
* @return  
*
*/
void hciCommandWriteCurrentIACs(uint8_t numCurrentIAC)
{
	uint8_t mbuf[HCI_MAX_SHORT_CMD_SIZE]; /* check if number of suppoprted IAC fits into the buffer */
	uint16_t pos = 0;

	mbuf[pos++] = numCurrentIAC;                        /* num of IACs */

	/*only support 1 && 2*/
	mbuf[pos++] = 0x33;
	mbuf[pos++] = 0x8b;
	mbuf[pos++] = 0x9e;

	if(numCurrentIAC == 2)
	{
		mbuf[pos++] = 0x00;
		mbuf[pos++] = 0x8b;
		mbuf[pos++] = 0x9e;
	}

	hciSendDsCommand(pHCI->hciQueueID, HCI_WRITE_CURRENT_IAC_LAP, mbuf, pos);
}

void hciCommandHostBufferSize(uint16_t aclLen, uint8_t scoLen, uint16_t aclCnt, uint16_t scoCnt)
{
    uint8_t mbuf[HCI_MAX_SHORT_CMD_SIZE];
    uint16_t pos = 0;

    SHORT2CHAR(mbuf+pos, aclLen); pos += 2;
    mbuf[pos++] = scoLen;
    SHORT2CHAR(mbuf+pos, aclCnt); pos += 2;
    SHORT2CHAR(mbuf+pos, scoCnt); pos += 2;

    hciSendDsCommand(pHCI->hciQueueID, HCI_HOST_BUFFER_SIZE, mbuf, pos);
}

void hciCommandHostNumberOfCompletedPackets(uint16_t handle, uint16_t count)
{
    uint8_t mbuf[HCI_MAX_SHORT_CMD_SIZE];
    uint16_t pos = 0;

    mbuf[pos++] = 1;     /* nbr of handles */
    SHORT2CHAR(mbuf+pos, handle); pos += 2;
    SHORT2CHAR(mbuf+pos, count);  pos += 2;

    HCI_TRACE_PRINTF_2(HCI_TRACE_MASK_TRACE,"HCI: HOST_NUM hdl %d cnt %d", handle,count);

    /* Send Message directly in Downstream Queue (no flow control, p.663) */
    hciSendDsCommand(pHCI->dsQueueID, HCI_HOST_NUMBER_OF_COMPLETED_PACKETS, mbuf, pos);
}


/**
* @brief		Lookup BD in remDev cache
*
* @param	bd: 
*
* @return	if found then return entry, else return NULL
*
*/
ThciRemDevice * hciLookupRemDev(uint8_t * bd)
{
    ThciRemDevice * rp = NULL;
    int             i;

    for (i = 0; i < HCI_MAX_REMDEV && rp == NULL; i++)
    {
        if (memcmp(bd, pHCI->remDev[i].bd, BD_ADDR_SIZE) == 0)
        {
            rp = &pHCI->remDev[i];
        }
    }
    HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_TRACE, "hciLookupRemDev, desc found: %lx", rp);

    return rp;
}

void hciHandleUpL2cConnectReq(TBdAddr remote_bd)
{
	uint16_t packetType = HCI_PACKET_TYPE_DM1 | HCI_PACKET_TYPE_DH1;
	ThciRemDevice * rp;
	/* remember the type of link requested, so we can handle the answer properly */
	pHCI->linkType = HCI_LINKTYPE_ACL;

	/* search for bd in remdev cache, if found than we have some addit information
	regarding / describing the timing of the remote device, which will result in faster
	connection setup times
	*/
	rp = hciLookupRemDev(remote_bd);

	if(rp != NULL)
	{
		hciCommandConnectReq(
							remote_bd,
							packetType,
							rp->pageScanRepMode,
							rp->pageScanMode,
							(uint16_t)(rp->clockOffset | 0x8000) /* set clock valid bit */
							);
	}
	else
	{
		hciCommandConnectReq(
							remote_bd,
							packetType,
							(uint8_t)(pHCI->scanRepetitionModeReq == 0xFF ? 2 : pHCI->scanRepetitionModeReq),
							0,
							0
							);
	}
	hciLaterEntry();
}

/**
* @brief  send HCI_WRITE_SCAN_ENABLE command
*
* @param  pHCI:
* @param  enable: scan mode
* @param  parAction: if save to nv
*
* @return  
*
*/
void hciHandleUpWriteScanEnable(uint8_t enable, uint8_t parAction)
{
	uint8_t mbuf[HCI_MAX_SHORT_CMD_SIZE];
	uint16_t pos = 0;

	if( pHCI->crrScanEnable == pHCI->saveScanEnable && parAction == RESTORE_PARAM)
	{
		return;
	}

	HCI_TRACE_PRINTF_3(HCI_TRACE_MASK_TRACE, "hciHandleUpWriteScanEnable: enable %X, crrScanEn %X saveScanEn %X ",
	                         enable,
	                         pHCI->crrScanEnable,
	                         pHCI->saveScanEnable);

	switch(parAction)
	{
	case NO_ACTION_PARAM:
		pHCI->internScanEnable = TRUE;
		break;

	case SAVE_PARAM:
		pHCI->saveScanEnable   = enable;
		pHCI->internScanEnable = FALSE;
		break;

	case RESTORE_PARAM:
		enable                 = pHCI->saveScanEnable;
		pHCI->internScanEnable = TRUE;
		break;

    default:
        break;
	}

	pHCI->crrScanEnable  = enable;
	mbuf[pos++]          = enable; /* len */
	hciSendDsCommand(pHCI->hciTransQueueID, HCI_WRITE_SCAN_ENABLE, mbuf, pos);

}

/**
* @brief  send HCI_WRITE_CLASS_OF_DEVICE command
*
* @param  ptr: 
*
* @return  
*
*/
void hciHandleUpWriteClassOfDevice(uint8_t * ptr)
{
    uint8_t mbuf[HCI_MAX_SHORT_CMD_SIZE];
    uint16_t pos = 0;
    uint32_t*  devClassReq = (uint32_t*)ptr;

    HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_TRACE, "hciHandleUpWriteClassOfDevice: %X", devClassReq);

    mbuf[pos++] = (uint8_t)(*devClassReq>>0);
    mbuf[pos++] = (uint8_t)(*devClassReq>>8);
    mbuf[pos++] = (uint8_t)(*devClassReq>>16);

    hciSendDsCommand(pHCI->hciQueueID, HCI_WRITE_CLASS_OF_DEVICE, mbuf, pos);
    hciLaterEntry();
}

void hciHandleUpNameReq(uint8_t * bd)
{
	ThciRemDevice * rp;

	HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_TRACE, "hciHandleUpNameReq: bd %s", TRACE_BDADDR1(HCI_TRACE_MASK_TRACE, bd));

	rp = hciLookupRemDev(bd);
	if (rp)
	{
		hciCommandRemoteNameRequest((uint8_t *)bd, rp->pageScanRepMode, rp->pageScanMode,
	   		                        (uint16_t)(rp->clockOffset | 0x8000) /* set clock valid bit */);
	}
	else
	{
		hciCommandRemoteNameRequest((uint8_t *)bd,
									(uint8_t)(pHCI->scanRepetitionModeReq == 0xFF ? 2 : pHCI->scanRepetitionModeReq),
									0, 0);
	}
	hciLaterEntry();
}

bool hciHandleUpSniffReq(LPCBYTE bd, uint8_t bd_type, uint16_t maxSniff, uint16_t minSniff, uint16_t attempt, uint16_t timeout)
{
	ThciHandleDesc * lh = hciFindBd((uint8_t *)bd, BT_CONNECTION_TYPE_BR_ACL);
	
	HCI_TRACE_PRINTF_6(HCI_TRACE_MASK_TRACE,"hciHandleUpSniffReq: bd %s max %d min %d att %d tim %d hdl %d",
                        TRACE_BDADDR1(HCI_TRACE_MASK_TRACE, bd),
                        maxSniff,
                        minSniff,
                        attempt,
                        timeout,
                        lh?lh->handle:0);

    if ((0x0002 <= minSniff <= 0xFFFE) && (0x0002 <= maxSniff <= 0xFFFE) &&
        (minSniff & 0x01 == 0) && (maxSniff & 0x01 == 0) && (minSniff <= maxSniff) &&
        (0x0001 <= attempt <= 0x7FFF) && (timeout <= 0x7FFF)
        )
    {
        if ((lh == NULL) ||
    		((lh->link_policy & HCI_LINK_POLICY_ENABLE_SNIFF) == 0x00) ||
    		((lh->nModeState == HCI_MODE_STATE_ACTIVE) && (maxSniff == 0))
    	   )
    	{
    		THCI_MODECHANGE_IND tmc;

    		tmc.bLinkContext = NULL;
    		memcpy((PVOID)tmc.bd, bd, BD_ADDR_SIZE);
    		if ((lh == NULL) || (maxSniff))
    		{
    			tmc.status		= hciStatus(HCI_ERR_COMMAND_DISALLOWED);
    		}
    		else
    		{
    			tmc.status		= hciStatus(HCI_SUCCESS);
    		}
    		tmc.mode		  = 0; /* active mode */
    		tmc.interval	  = maxSniff;

    		blueAPI_Handle_HCI_MODE_CHANGE_IND(&tmc);
    	}
    	else
    	{
    	    if ((lh->supervisionTimeout) && (maxSniff >= lh->supervisionTimeout))
            {
                return true;
            }
    		/* be sure there are even values - see BT1.2 specification */
    		minSniff &= ~0x0001;
    		maxSniff &= ~0x0001;

    		if ( maxSniff )
    		{
    			if ( lh->nModeState == HCI_MODE_STATE_ACTIVE )
    			{
    				hciCommandSniffMode(lh->handle, maxSniff, minSniff, attempt, timeout);
    				lh->nModeState = HCI_MODE_STATE_SNIFF_PENDING;
    			}
    			else
    			{
    				lh->sniffMax	   = maxSniff;
    				lh->sniffMin	   = minSniff;
    				lh->sniffAttempt = attempt;
    				lh->sniffTimeout = timeout;
    				if ( lh->nModeState == HCI_MODE_STATE_SNIFF_PENDING )
    				{
    					lh->nModeState = HCI_MODE_STATE_NEW_SNIFF_PENDING;
    				}
    				else
    				{ 
    					hciCommandWordParameter(HCI_EXIT_SNIFF_MODE, lh->handle);
    					lh->nModeState = HCI_MODE_STATE_WAKE_UP_SNIFF_PENDING;
    				}
    			}
    		}
    		else
    		{ 
    			hciCommandWordParameter(HCI_EXIT_SNIFF_MODE, lh->handle);
    			lh->nModeState = HCI_MODE_STATE_WAKE_UP_PENDING;
    		}
    	}
    	hciLaterEntry();
        return false;
    }
    else
    {
        return true;
    }
}

void hciHandleUpInquiryReq(uint8_t timeOut, uint16_t maxHdl, uint8_t idxIAC)
{
	uint16_t size = (uint16_t)(sizeof(TinqBuf) + (BD_ADDR_SIZE + BT_CLASS_SIZE) * (maxHdl) + pHCI->ReadOffset);
	uint8_t * p;

	timeOut = timeOut ? timeOut : 8;

	/*inquiry cancel*/
	if (maxHdl == 0)
	{
		HCI_TRACE_PRINTF_0(HCI_TRACE_MASK_TRACE, "hciHandleUpListenReq: cancel command");
		hciCommandNoParameter(HCI_INQUIRY_CANCEL);
		return;
	}
	if (osBufferGet(UpstreamPoolID, size, (void  *)&p))
	{
		HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_ERROR, "!!hciHandleUpListenReq: no memory maxHdl %X", maxHdl);
		assert(FALSE);
		return;
	}
	pHCI->inqBuf		 = (LPinqBuf)(p + pHCI->ReadOffset);
	pHCI->inqBuf->status = 0;			/* not exactly necessary */
	pHCI->inqBuf->count  = 0;			/* strictly required */
	pHCI->inqBuf->maxHdl = maxHdl;

	hciCommandInquiry(idxIAC, timeOut, 0xff /* give all results.... */);
	hciLaterEntry();
}

/**
* @brief  handle upstream auth request
* 
* @param  bd
* @param  id
*
* @return  
*
*/
void hciHandleUpAuthReq(TBdAddr bd, uint8_t id)
{
    ThciHandleDesc * lh;

    HCI_TRACE_PRINTF_2(HCI_TRACE_MASK_TRACE,"hciHandleUpAuthReq: bd %s id %x",TRACE_BDADDR1(HCI_TRACE_MASK_TRACE, bd),id);

    lh = hciFindBd(bd, BT_CONNECTION_TYPE_BR_ACL);
    if (!lh)
    {
        btsmHandleHciAuthConf(bd, id, hciStatus(HCI_ERR_ILLEGAL_HANDLE));
        return;
    }

    /* remember id of request */
    pHCI->tid = id;
    lh->tid   = id;

    if (hciAuthenticationOnLinkEncrypted(lh, HCI_EA_AUTH_REQ))
    {
        // if link is encrypted switch off encryption and after that send auth req
        return;
    }

    /* bd translated into handle, request action from controller */
    hciCommandWordParameter(HCI_AUTHENTICATION_REQUESTED, lh->handle);
}

/**
* @brief  send hci encryption request
* 
* @param  bd
* @param  id
* @param  enable
*
* @return  
*
*/
void hciHandleUpEncryptReq(TBdAddr bd, uint8_t enable)
{
    ThciHandleDesc * lh;

    HCI_TRACE_PRINTF_2(HCI_TRACE_MASK_TRACE,"hciHandleUpEncryptReq: bd %s en %x",
                        TRACE_BDADDR1(HCI_TRACE_MASK_TRACE, bd), enable);

    lh = hciFindBd(bd, BT_CONNECTION_TYPE_BR_ACL);
    if (!lh)
    {
        hciEncryptInd(bd, hciStatus(HCI_ERR_ILLEGAL_HANDLE), 0);
        return;
    }

    if ( hciAuthenticationOnLinkEncrypted( lh, HCI_EA_ENCRYPT_REQ) )
    {
        return;
    }

    memcpy(pHCI->remoteBdAddr, bd, BD_ADDR_SIZE);
    /* bd translated into handle, request action from controller */
    hciCommandWordByteParameter(HCI_SET_CONNECTION_ENCRYPTION, lh->handle, enable);
    hciLaterEntry();
} /* hciHandleUpEncryptReq */



/**
* @brief  send write extended inquiry response command
*
* @param  eirResponse:
*
* @return  
*
*/
void hciHandleUpWriteExtendedInquiryResponse(LPHCI_CONF_EXTENDED_INQUIRY_RESPONSE eirResponse)
{
	if (pHCI->hciVersion >= HCI_VERSION_21)
	{
		hciCommandWriteExtendedInquiryResponse(eirResponse->fec, eirResponse->extendedResult);
		pHCI->wEIRp = (uint8_t *)eirResponse; 	/* save the pointer for release by command complete from HCI, will call callback func */
        hciLaterEntry();
    }
    else
    {
        osBufferRelease(eirResponse);
    }
}

void hciHandleConfLinkPolicy(uint8_t * remote_BD,
									TBlueAPI_BRDeviceRole role,
									TBlueAPI_BRLinkPolicy link_policy,
									uint16_t link_supervision_timeout)
{
	ThciHandleDesc	*pHd;
	pHd = hciFindBd(remote_BD, BT_CONNECTION_TYPE_BR_ACL);

	if (!pHd)
	{
		HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_ERROR, "!!hciHandleUpConfigureReq. HCI_CONF_LINK_POLICY_LINK, unknown link handle for %s",
							TRACE_BDADDR1(HCI_TRACE_MASK_ERROR,(uint8_t *)remote_BD));
		return;
	}

	pHd->piconetType = role;
	pHd->link_policy = link_policy;
    pHd->supervisionTimeout = link_supervision_timeout;

	if ( pHd->currentRole == HCI_ROLE_MASTER )
	{
		hciCommandWordWordParameter(HCI_WRITE_LINK_SUPERVISION_TIMEOUT, pHd->handle, link_supervision_timeout);
	}
	hciChangeLinkPolicyAndRole(pHd);
	hciLaterEntry();
}

void hciHandleConfLinkPolicyDefault(TBlueAPI_BRDeviceRole role,
											TBlueAPI_BRLinkPolicy link_policy,
											uint16_t link_supervision_timeout)
{	
	int i;
	uint8_t oldPicoNetType;
    ThciHandleDesc * lh;

	oldPicoNetType				   = pHCI->piconetType;
	pHCI->piconetType			   = role;
	pHCI->link_policy			   = link_policy;
	pHCI->link_supervision_timeout = link_supervision_timeout;

	for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_don; i++)
	{
		lh = &pHCI->phciDescDon[i];
		if (lh->used && oldPicoNetType == lh->piconetType)
		{
			lh->piconetType = pHCI->piconetType;  /* copy piconet type to specific link */
			hciChangeLinkPolicyAndRole(lh);
		}
	}

	for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_doff; i++)
	{
		lh = &pHCI->phciDescDoff[i];
		if (lh->used && oldPicoNetType == lh->piconetType)
		{
			lh->piconetType = pHCI->piconetType;  /* copy piconet type to specific link */
			hciChangeLinkPolicyAndRole(lh);
		}
	}
}

void hciHandleConfPageCharacteristic(TBlueAPI_BRPageScanRepMode  scanRepetitionMode,
												uint16_t pageTimeout)
{
	if (scanRepetitionMode < 3)
	{
		pHCI->scanRepetitionModeReq = scanRepetitionMode;
	}
	if (pageTimeout == 0)
	{
		pHCI->pageTimeout = BT_HCI_PAGE_TIMEOUT_DEFAULT_COUNT;	/*default*/
	}
	else
	{
		pHCI->pageTimeout = pageTimeout;
	}
	hciCommandWordParameter(HCI_WRITE_PAGE_TIMEOUT, pHCI->pageTimeout);
	hciLaterEntry();
}

void hciHandleConfPageScanActivityEx(TBlueAPI_BRPageScanRepMode  scanRepetitionMode,
												TBlueAPI_BRPageScanType scanType,
												uint16_t scanInterval, uint16_t scanWindow)
{
	BOOL bSend = TRUE;

	switch (scanRepetitionMode)
	{
	case 0x00: /* 11.25 ms */
		pHCI->pageScanActivity.interval = 0x0012;
		pHCI->pageScanActivity.window   = 0x0012;
		break;

	case 0x01: /*  1.28 s */
		pHCI->pageScanActivity.interval = 0x0800;
		pHCI->pageScanActivity.window   = 0x0012;
		break;

	case 0x02: /*  2.56 s */
		pHCI->pageScanActivity.interval = 0x1000;
		pHCI->pageScanActivity.window   = 0x0012;
		break;

	case 0xFF:
		pHCI->pageScanActivity.interval = (scanInterval & ~0x0001); /* only even values allowed - see BT1.2 specification */
		pHCI->pageScanActivity.window   = scanWindow;
		break;

	default:
		bSend = FALSE;
		HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_ERROR, "!!hciHandleUpConfigureReq(HCI_CONF_PAGE_SCAN_ACTIVITY), unknown repetition mode 0x%.2X", scanRepetitionMode);
		break;
	}

	if (bSend)
	{
		hciCommandWordWordParameter(HCI_WRITE_PAGE_SCAN_ACTIVITY,
		                          pHCI->pageScanActivity.interval,
		                          pHCI->pageScanActivity.window);
		hciLaterEntry();
	}

	if (scanType < 2)
	{
		hciCommandByteParameter(HCI_WRITE_PAGE_SCAN_TYPE, scanType);
		hciLaterEntry();
	}
	
}

void hciHandleConfSniffSubrating(uint8_t * remote_BD, uint16_t maxLatency,
                                          uint16_t minRemoteTimeout, uint16_t minLocalTimeout)
{
	ThciHandleDesc *pHd;

	pHd = hciFindBd(remote_BD, BT_CONNECTION_TYPE_BR_ACL);
	if (pHd == NULL)
	{
		HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_ERROR,
						  "!!hciHandleUpConfigureReq HCI_CONF_SNIFF_SUBRATING, unknown link handle for %s",
						  TRACE_BDADDR1(HCI_TRACE_MASK_ERROR, remote_BD));
		return;
	}

	hciCommandSniffSubrating(pHd->handle,
						   maxLatency,
						   minRemoteTimeout,
						   minLocalTimeout);
	hciLaterEntry();
}

void hciHandleConfInquiryScanActivityEx(TBlueAPI_BRInquiryScanType scanType,
												uint16_t scanInterval, uint16_t scanWindow)
{
	hciCommandWordWordParameter(HCI_WRITE_INQUIRY_SCAN_ACTIVITY, scanInterval, scanWindow);
	hciLaterEntry();

	if (scanType < 2)
	{
		hciCommandByteParameter(HCI_WRITE_INQUIRY_SCAN_TYPE, scanType);
		hciLaterEntry();
	}
}

void hciHandleConfInquiryMode(uint8_t mode)
{
	hciCommandByteParameter(HCI_WRITE_INQUIRY_MODE, mode);
	hciLaterEntry();
}

/**
* @brief  hci encrypt indicate
*
* @param  bd: 
* @param  status: 
* @param  enable
*
* @return  
*
*/
void hciEncryptInd(uint8_t * bd, uint16_t status, uint8_t enable)
{
	ThciHandleDesc *hd;

	hd = hciFindBd(bd, BT_CONNECTION_TYPE_BR_ACL);

	if (hd)
	{
	    if ( status )
	    {
			if ( hd->encryptionStatus == HCI_ENCRYPTION_STATE_OFF_WHILE_AUTHENTICATION_PENDING )
			{
				btsmHandleHciAuthConf(hd->bd, hd->tid, status);
			}
	    }
	    if ( hciAuthenticationOnLinkEncrypted( hd, enable ?HCI_EA_ENCRYPT_ON_IND :HCI_EA_ENCRYPT_OFF_IND) )
	    {
	        return;
	    }
	}
	else
	{
	    HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_ERROR,"!!HCI: hciEncryptInd: could not find desc for bd %s", TRACE_BDADDR1(HCI_TRACE_MASK_ERROR, bd));
	}

	btsmHandleHciEncryptInd(bd, status, enable);

}

/**
* @brief  link enctrypted
*
* @param  hd
* @param  eAEvent: 
*
* @return  
*
*/
BOOL hciAuthenticationOnLinkEncrypted(ThciHandleDesc *hd, THciEncryptionAuthenticationEvent eAEvent)
{
    if(hd)
    {
        HCI_TRACE_PRINTF_2(HCI_TRACE_MASK_TRACE, "hciAuthenticationOnLinkEncrypted(%i) in state %i", eAEvent, hd->encryptionStatus);
    }
    else
    {
        HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_TRACE, "! hciAuthenticationOnLinkEncrypted(%i): illegal link descriptor", eAEvent);
    }

    switch (eAEvent)
    {
    case HCI_EA_ENCRYPT_ON_IND:
        hd->encryptionStatus = HCI_ENCRYPTION_STATE_ON;
        break;

    case HCI_EA_ENCRYPT_OFF_IND:
        if (hd->encryptionStatus == HCI_ENCRYPTION_STATE_OFF_WHILE_AUTHENTICATION_PENDING)
        {
            hciCommandWordParameter(HCI_AUTHENTICATION_REQUESTED, hd->handle);
        }
        else if (hd->encryptionStatus == HCI_ENCRYPTION_STATE_ON_AFTER_AUTHENTICATION)
        {
            HCI_TRACE_PRINTF_0(HCI_TRACE_MASK_ERROR, "!hciAuthenticationOnLinkEncrypted: cannot enable encryption after authentication");
            hd->encryptionStatus = HCI_ENCRYPTION_STATE_OFF;
        }
        break;

    case HCI_EA_ENCRYPT_REQ:
        if (hd->encryptionStatus != HCI_ENCRYPTION_STATE_OFF)
        {
            return TRUE;
        }
        break;

    case HCI_EA_AUTH_REQ:
        if (hd->encryptionStatus == HCI_ENCRYPTION_STATE_ON)
        {
			hciCommandWordByteParameter(HCI_SET_CONNECTION_ENCRYPTION, hd->handle, HCI_LINK_LEVEL_ENCRYPTION_OFF);
            hd->encryptionStatus = HCI_ENCRYPTION_STATE_OFF_WHILE_AUTHENTICATION_PENDING;
            return TRUE;;
        }
        else if (hd->encryptionStatus != HCI_ENCRYPTION_STATE_OFF)
        {
            return TRUE;
        }
        break;

    case HCI_EA_AUTH_CONF_IND:
        if (hd->encryptionStatus == HCI_ENCRYPTION_STATE_OFF_WHILE_AUTHENTICATION_PENDING)
        {
			hciCommandWordByteParameter(HCI_SET_CONNECTION_ENCRYPTION, hd->handle, HCI_LINK_LEVEL_ENCRYPTION_ON);
            hd->encryptionStatus = HCI_ENCRYPTION_STATE_ON_AFTER_AUTHENTICATION;
        }
        break;

    default:
        break;
    }

    return FALSE;
}

/**
* @brief  change link policy and role
*
* @param  pHd: hci handle des
*
* @return  
*
*/
void hciChangeLinkPolicyAndRole(ThciHandleDesc *pHd)
{
	BOOL bChangeRole = (pHd->currentRole == HCI_ROLE_MASTER && (pHd->piconetType == HCI_PICONET_SLAVE_PREFERRED  || pHd->piconetType == HCI_PICONET_SLAVE_REQUIRED))
	                 || (pHd->currentRole == HCI_ROLE_SLAVE  && (pHd->piconetType == HCI_PICONET_MASTER_PREFERRED || pHd->piconetType == HCI_PICONET_MASTER_REQUIRED));

	if ( bChangeRole )
	{
		hciCommandWordWordParameter(HCI_WRITE_LINK_POLICY_SETTINGS, pHd->handle, (uint16_t)(pHd->link_policy | HCI_LINK_POLICY_ENABLE_SWITCH) );
	}
	else if ( pHd->piconetType == HCI_PICONET_MASTER_REQUIRED || pHd->piconetType == HCI_PICONET_SLAVE_REQUIRED )
	{
		hciCommandWordWordParameter(HCI_WRITE_LINK_POLICY_SETTINGS, pHd->handle, (uint16_t)(pHd->link_policy & ~HCI_LINK_POLICY_ENABLE_SWITCH) );
	}
	else
	{
		hciCommandWordWordParameter(HCI_WRITE_LINK_POLICY_SETTINGS, pHd->handle, pHd->link_policy );
	}

	if (bChangeRole)
	{
		hciCommandBDAddrByteParameter(HCI_SWITCH_ROLE, pHd->bd, (uint8_t)(pHd->piconetType == HCI_PICONET_SLAVE_PREFERRED || pHd->piconetType == HCI_PICONET_SLAVE_REQUIRED));
	}
	else
	{
		pHd->switchRoleTries = 0;
		blueAPI_Handle_HCI_ROLE_CHANGE((uint8_t)(pHd->currentRole ?ROLE_CHANGE_TO_SLAVE :ROLE_CHANGE_TO_MASTER), pHd->bd, pHd->bdType);
	}
} /* hciChangeLinkPolicyAndRole */

/**
* @brief  hci change connect packet type
*
* @param  handle: ACL handle
*
* @return  
*
*/
void hciChangeConnPacketType(uint16_t handle)
{
#if (BT_HCI_FIXED_PACKET_TYPES_COUNT != 0)
    uint16_t packetType  =  BT_HCI_FIXED_PACKET_TYPES_COUNT;
#else  
    uint16_t packetType  =  HCI_PACKET_TYPE_DM1 |
                        HCI_PACKET_TYPE_NO_2DH1 | HCI_PACKET_TYPE_NO_3DH1 |
                        HCI_PACKET_TYPE_NO_2DH3 | HCI_PACKET_TYPE_NO_3DH3 |
                        HCI_PACKET_TYPE_NO_2DH5 | HCI_PACKET_TYPE_NO_3DH5;

#if (BT_MAX_PACKET_SIZE > HCI_DM1_PAYLOAD)
    packetType |= HCI_PACKET_TYPE_DH1;
#endif

/* BR packet types */
#if (BT_MAX_PACKET_SIZE > HCI_DH1_PAYLOAD)
    packetType |= HCI_PACKET_TYPE_DM3;
#if (BT_MAX_PACKET_SIZE > HCI_DM3_PAYLOAD)
    packetType |= HCI_PACKET_TYPE_DH3;
#if (BT_MAX_PACKET_SIZE > HCI_DH3_PAYLOAD)
    packetType |= HCI_PACKET_TYPE_DM5;
#if (BT_MAX_PACKET_SIZE > HCI_DM5_PAYLOAD)
    packetType |= HCI_PACKET_TYPE_DH5;
#endif  /* > HCI_DM5_PAYLOAD */
#endif  /* > HCI_DH3_PAYLOAD */
#endif  /* > HCI_DM3_PAYLOAD */
#endif  /* > HCI_DH1_PAYLOAD */

/* EDR 2Mbps packet types */
#if (BT_MAX_PACKET_SIZE > HCI_DH1_PAYLOAD)
    packetType &= ~(HCI_PACKET_TYPE_NO_2DH1);
#if (BT_MAX_PACKET_SIZE > HCI_2DH1_PAYLOAD)
    packetType &= ~(HCI_PACKET_TYPE_NO_2DH3);
#if (BT_MAX_PACKET_SIZE > HCI_2DH3_PAYLOAD)
    packetType &= ~(HCI_PACKET_TYPE_NO_2DH5);
#endif  /* > HCI_2DH3_PAYLOAD */
#endif  /* > HCI_2DH1_PAYLOAD */
#endif  /* > HCI_DH1_PAYLOAD */

/* EDR 3Mbps packet types */
#if (BT_MAX_PACKET_SIZE > HCI_DH1_PAYLOAD)
    packetType &= ~(HCI_PACKET_TYPE_NO_3DH1);
#if (BT_MAX_PACKET_SIZE > HCI_3DH1_PAYLOAD)
    packetType &= ~(HCI_PACKET_TYPE_NO_3DH3);
#if (BT_MAX_PACKET_SIZE > HCI_3DH3_PAYLOAD)
    packetType &= ~(HCI_PACKET_TYPE_NO_3DH5);
#endif  /* > HCI_2DH3_PAYLOAD */
#endif  /* > HCI_2DH1_PAYLOAD */
#endif  /* > HCI_DH1_PAYLOAD */

    /* disable packetTypes if feature is not supported */
    if (!(pHCI->localFeatures[0] & LMP_FEAT_3SLOT))
      packetType &= ~(HCI_PACKET_TYPE_DM3 | HCI_PACKET_TYPE_DH3);

    if (!(pHCI->localFeatures[0] & LMP_FEAT_5SLOT))
      packetType &= ~(HCI_PACKET_TYPE_DM5 | HCI_PACKET_TYPE_DH5);

    if (!(pHCI->localFeatures[4] & LMP_FEAT_3SLOT_EDR))
      packetType |= HCI_PACKET_TYPE_NO_2DH3 | HCI_PACKET_TYPE_NO_3DH3;

    if (!(pHCI->localFeatures[5] & LMP_FEAT_5SLOT_EDR))
      packetType |= HCI_PACKET_TYPE_NO_2DH5 | HCI_PACKET_TYPE_NO_3DH5;

    if (!(pHCI->localFeatures[3] & LMP_FEAT_EDR_2MBPS))
      packetType |= HCI_PACKET_TYPE_NO_2DH1 | HCI_PACKET_TYPE_NO_2DH3 | HCI_PACKET_TYPE_NO_2DH5;

    if (!(pHCI->localFeatures[3] & LMP_FEAT_EDR_3MBPS))
      packetType |= HCI_PACKET_TYPE_NO_3DH1 | HCI_PACKET_TYPE_NO_3DH3 | HCI_PACKET_TYPE_NO_3DH5;

#endif
    HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_TRACE, "hciChangeConnPacketType: packet type 0x%x", packetType);

    hciCommandWordWordParameter(HCI_CHANGE_CONNECTION_PACKET_TYPE, handle, packetType);
} /* hciChangeConnPacketType */

/**
* @brief  handle hci listen confirm
* 
* @param  status
*
* @return  
*
*/
void hciListenCnf(uint16_t status)
{
    int i;
    uint8_t * dst,*src;
	
#if CHECK_API_PARAM	
    if (!pHCI->inqBuf)         /* No Memory: No Info */
    {
        HCI_TRACE_PRINTF_0(HCI_TRACE_MASK_ERROR, "!!hciListenCnf: no mem allocated");
        return;
    }
#endif		

    /* copy the class of devices section immediately after the last bd */
    dst = pHCI->inqBuf->bd + BD_ADDR_SIZE * pHCI->inqBuf->count;
    src = pHCI->inqBuf->bd + BD_ADDR_SIZE * pHCI->inqBuf->maxHdl;
    for (i = 0; i < pHCI->inqBuf->count * BT_CLASS_SIZE; i++)
	{
    	*dst++ = *src++;
	}

	blueAPI_Handle_HCI_LISTEN_CONF(status);
	osBufferRelease((uint8_t *)(pHCI->inqBuf) - pHCI->ReadOffset);
    pHCI->inqBuf = NULL;
} /* hciListenCnf */

#if (F_BT_SCO)
/**
 * @brief  send add SCO connection command to downstream
 *
 * @param  pHCI
 * @param  handle
 * @param  packetType
 *
 * @return
 *
 */
void hciCommandAddScoConnection(uint16_t handle, uint16_t packetTypes)
{
    uint8_t mbuf[4];
    uint16_t pos = 0;
    SHORT2CHAR(mbuf+pos, handle);      pos += 2;      /* acl connection handle */
    SHORT2CHAR(mbuf+pos, packetTypes); pos += 2;      /* packet type */
    hciSendDsCommand(pHCI->hciQueueID, HCI_ADD_SCO_CONNECTION, mbuf, pos);
}

/**
 * @brief  send SetupSynchronousConnection command to downstream
 *
 * @param  pHCI
 * @param  handle
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
void hciCommandSetupSynchronousConnection(
                                                            uint16_t  handle,
                                                            uint32_t txBandwidth,
                                                            uint32_t rxBandwidth,
                                                            uint16_t  maxLatency,
                                                            uint16_t  voiceSetting,
                                                            uint8_t  retransEffort,
                                                            uint16_t  packetType
                                                            )
{
    uint8_t mbuf[17];
    uint16_t pos = 0;

    HCI_TRACE_PRINTF_7(HCI_TRACE_MASK_TRACE,
                        "hciCommandSetupSynchronousConnection: hdl %X, txB %i, rxB %i, maxL %i, voiceSet 0x%X, retxEff %i, pckType 0x%X",
                        handle,
                        txBandwidth,
                        rxBandwidth,
                        maxLatency,
                        voiceSetting,
                        retransEffort,
                        packetType
                        );

    SHORT2CHAR(mbuf+pos, handle);       pos += 2;     /* acl connection handle */
    LONG2CHAR(mbuf+pos, txBandwidth);   pos += 4;     /* transmit bandwidth in octets per second */
    LONG2CHAR(mbuf+pos, rxBandwidth);   pos += 4;     /* receive bandwidth in octets per second  */
    SHORT2CHAR(mbuf+pos, maxLatency);   pos += 2;     /* max latency */
    SHORT2CHAR(mbuf+pos, voiceSetting); pos += 2;     /* voice setting */
    mbuf[pos++] = retransEffort;                      /* retransmission effort */
    SHORT2CHAR(mbuf+pos, packetType);   pos += 2;     /* packet type */

    hciSendDsCommand(pHCI->hciQueueID, HCI_SETUP_SYNCHRONOUS_CONNECTION, mbuf, pos);
}

/**
 * @brief  send accept Synchronous Connection command to downstream
 *
 * @param  pHCI
 * @param  bd
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
void hciCommandAcceptSynchronousConnectionRequest(
                                                                        uint8_t * bd,
                                                                        uint32_t txBandwidth,
                                                                        uint32_t rxBandwidth,
                                                                        uint16_t  maxLatency,
                                                                        uint16_t  contentFormat,
                                                                        uint8_t  retransEffort,
                                                                        uint16_t  packetType
                                                                        )
{
    uint8_t mbuf[21];
    uint16_t pos = 0;

    memcpy(mbuf+pos, bd, BD_ADDR_SIZE);     pos += BD_ADDR_SIZE;
    LONG2CHAR(mbuf+pos, txBandwidth);       pos += 4;   /* transmit bandwidth in octets per second */
    LONG2CHAR(mbuf+pos, rxBandwidth);       pos += 4;   /* receive bandwidth in octets per second  */
    SHORT2CHAR(mbuf+pos, maxLatency);       pos += 2;   /* max latency */
    SHORT2CHAR(mbuf+pos, contentFormat);    pos += 2;   /* content format */
    mbuf[pos++] = retransEffort;                        /* retransmission effort */
    SHORT2CHAR(mbuf+pos, packetType);       pos += 2;   /* packet type */

    hciSendDsCommand(pHCI->hciQueueID, HCI_ACCEPT_SYNCHRONOUS_CONNECTION_REQUEST, mbuf, pos);
}

/**
 * @brief  hci SCO connection response
 *
 * @param  bd
 * @param  devClass
 * @param  linktype
 *
 * @return
 *
 */
void hciHandleUpSCOConConf(TBdAddr bd, uint16_t accept, PCON_RESP_SCO pSCO_RESP)
{
    switch (accept)
    {
    case BLUEFACE_CON_ACCEPT: /* Accept Connection */
        /* save the BD in question in remoteBdAddr, it might be useful for early ACL packets */
        memcpy(pHCI->remoteBdAddr, bd, BD_ADDR_SIZE);

        if (pHCI->hciVersion > 1 && !(pHCI->capabilities & HCI_SYNCHRONOUS_CONNECTION_COMMAND_NOT_SUPPORTED)) /* sco or esco & bt1.2 */
        {
            /* accept incoming (e)sco connection with parameter set received from BLUEFACE_CONF_eSCO_DEFAULT */
            /* HCI spec says: it is allowed to enable packet types that are not supported by the local device, so we do not check / modify */
            hciCommandAcceptSynchronousConnectionRequest(
                            bd,                         /* bd address (for sco type invalid) */
                            pSCO_RESP->txBandwidth,     /* txBandwidth in octets per second */
                            pSCO_RESP->rxBandwidth,     /* rxBandwidth in octets per second (for sco type invalid) */
                            pSCO_RESP->maxLatency,      /* maximum latency in milliseconds - don not care*/
                            pSCO_RESP->voiceSetting,    /* voice setting*/
                            pSCO_RESP->retransEffort,   /* retransmission effort: at last one retransmission, optimize for pwr saving*/
                            pSCO_RESP->packetType       /* packet type*/
                            );
        }
        else
        {
            hciCommandBDAddrByteParameter(HCI_ACCEPT_CONNECTION_REQUEST, bd, 1 /* first remain slave */);
        }
        break;

    case BLUEFACE_CON_REJECT: /* Reject with default reject code */
        if (pHCI->hciVersion > 1 && !(pHCI->capabilities & HCI_SYNCHRONOUS_CONNECTION_COMMAND_NOT_SUPPORTED)) /* sco or esco & bt1.2 */
        {
            hciCommandBDAddrByteParameter(HCI_REJECT_SYNCHRONOUS_CONNECTION_REQUEST, bd, HCI_ERR_HOST_REJECTED_0D);
        }
        else
        {
            hciCommandBDAddrByteParameter(HCI_REJECT_CONNECTION_REQUEST, bd, HCI_ERR_HOST_REJECTED_0D);
        }
        break;

    default: /* Reject with specified rejection code */
        if (pHCI->hciVersion > 1 && !(pHCI->capabilities & HCI_SYNCHRONOUS_CONNECTION_COMMAND_NOT_SUPPORTED)) /* sco or esco & bt1.2 */
        {
            hciCommandBDAddrByteParameter(HCI_REJECT_SYNCHRONOUS_CONNECTION_REQUEST, bd, accept);
        }
        else
        {
            hciCommandBDAddrByteParameter(HCI_REJECT_CONNECTION_REQUEST, bd, accept);
        }
        break;
    } /* switch */
}

/**
 * @brief  handle SCO connection request
 *
 * @param  bd
 * @param  conSCOReq
 *
 * @return
 *
 */
void hciHandleUpSCOConReq(uint8_t * bd, PCON_REQ_SCO pSCOConReq)
{
    ThciHandleDesc * lh = NULL;

    memcpy(pHCI->remoteBdAddr, bd, BD_ADDR_SIZE);

    lh = hciFindBd(bd, BT_CONNECTION_TYPE_BR_ACL);
    if (!lh)
    {
        blueAPI_Handle_SCO_CON_CONF(bd, hciStatus(HCI_ERR_ILLEGAL_HANDLE));
        return;
    }

    HCI_TRACE_PRINTF_3(HCI_TRACE_MASK_TRACE,
                       "hciHandleUpSCOConReq: bd %s using handle 0x%X packetType 0x%X",
                       TRACE_BDADDR1(HCI_TRACE_MASK_TRACE, bd),
                       lh->handle,
                       pSCOConReq->packetType
                       );

    /* remember the linktype, is required for COMMAND_STATUS evaluation */
    pHCI->linkType = HCI_LINKTYPE_SCO;

    if (pHCI->hciVersion > 1
        && !(pHCI->capabilities & HCI_SYNCHRONOUS_CONNECTION_COMMAND_NOT_SUPPORTED)
        && (pSCOConReq->transmitBandwidth != 0 || pSCOConReq->receiveBandwidth != 0)
        )
    {
        /* HCI spec says: it is allowed to enable packet types that are not supported by the local device, so we do not check / modify */
        hciCommandSetupSynchronousConnection(
                    lh->handle,
                    pSCOConReq->transmitBandwidth,  /* txBandwidth in octets per second */
                    pSCOConReq->receiveBandwidth,   /* rxBandwidth in octets per second */
                    pSCOConReq->maxLatency,         /* maximum latency in milliseconds  - don not care */
                    pSCOConReq->content,            /* voice setting */
                    pSCOConReq->retry,              /* retransmission effort: at last one retransmission, optimize for pwr saving */
                    pSCOConReq->packetType          /* packet type */
        );
    }
    else
    {
        if((pHCI->localFeatures[0] & LMP_FEAT_HV3) == 0x00)
        {
            pSCOConReq->packetType &= ~(HCI_PACKET_TYPE_HV3 >> 5); /* convert blueface bit definition into hci */
        }
        if( (pHCI->localFeatures[0] & LMP_FEAT_HV2) == 0x00 )
        {
            pSCOConReq->packetType &= ~(HCI_PACKET_TYPE_HV2 >> 5);
        }
        /* convert packet type from setup synchronous (new) to add sco connection (old, deprecated) */
        hciCommandAddScoConnection(lh->handle, (pSCOConReq->packetType & 7) << 5);
    }
}

/**
 * @brief  when disconnect a non SCO connection, check if there are any related (to same BD) SCO links 
 *
 * @param  hd
 * @param  handle
 * @param  status
 *
 * @return
 *
 */
void hciDiscAllScoByAcl(ThciHandleDesc *hd, uint16_t handle, uint8_t status)
{
    /* if we disconnect a non sco connection, we check if there are any
        related (to same BD) SCO links. All such links are implicitely disconnected
        THANKS to PHILIPS BB (R3.3)
        */
    int i;
    ThciHandleDesc *lh;

    if (hd->conType == BT_CONNECTION_TYPE_BR_SCO)
    {
        blueAPI_Handle_SCO_DISC(handle, status);
    }
    else if (hd->conType == BT_CONNECTION_TYPE_BR_ACL)
    {
        for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_don; i++)
        {
            lh = &pHCI->phciDescDon[i];
            if ((lh->used) && (lh->handle != handle) && (memcmp(lh->bd, hd->bd, BD_ADDR_SIZE)==0))
            {
                HCI_TRACE_PRINTF_2(HCI_TRACE_MASK_ERROR,"!!HCI: disconnecting sco %x from link %x",lh->handle, handle);

                if(lh->conType == BT_CONNECTION_TYPE_BR_SCO)
                {
                    blueAPI_Handle_SCO_DISC(lh->handle, status);
                    hciRemoveHandle(lh->handle);
                }
            }
        }

        for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_doff; i++)
        {
            lh = &pHCI->phciDescDoff[i];
            if ((lh->used) && (lh->handle != handle) && (memcmp(lh->bd, hd->bd, BD_ADDR_SIZE)==0))
            {
                HCI_TRACE_PRINTF_2(HCI_TRACE_MASK_ERROR,"!!HCI: disconnecting sco %x from link %x",lh->handle, handle);

                if(lh->conType == BT_CONNECTION_TYPE_BR_SCO)
                {
                    blueAPI_Handle_SCO_DISC(lh->handle, status);
                    hciRemoveHandle(lh->handle);
                }
            }
        }
    }
}
#endif

