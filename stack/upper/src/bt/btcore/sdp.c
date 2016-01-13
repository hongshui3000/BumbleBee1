/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file 	  sdp.c
* @brief   SDP Layer
* @details	 
*
* @author  gordon
* @date 	   2015-06-23
* @version	   v0.1
*/

#include <flags.h>
#include <os_message.h>
#include <os_pool.h>
#include <os_timer.h>
#include <os_mem.h>
#include <sdp.h>
#include <sdp_code.h>
#include <sdplib.h>
#include <btcommon.h>
#include <l2c_api.h>
#include <hci_api.h>
#include <btsend.h>
#include <trace_binary.h>
#include <swtimer.h>
#include <sdp_api.h>
#include <upper_stack_global.h>
#include <blueapi_api.h>

#define TRACE_MODULE_ID     MID_BT_SDP

#define X_SDP_QUALIFICATION_SUPPORT 0  /* Experimental, leave off */
#define X_SDP_LOOPBACK              0  /* Experimental, leave off */

#if defined(UPF_TEST)
extern int btTestCase;
#endif

#if (X_SDP_QUALIFICATION_SUPPORT)
int sdpTestCase = 0;
#endif

#if defined(F_RAM_COUNTING)      /* define Instance and other Data global to get size from map/listing */
TSDP sdpInstanceRAM;
TsdpChanDesc sdpChanRAM;
#endif  /* defined(F_RAM_COUNTING) */

#if BT_SDP_CLIENT_BUFFER_COUNT
#define BT_US_SDP_BUFFER_SIZE  (BT_SDP_CLIENT_BUFFER_BYTE_COUNT + BT_US_WRITE_OFFSET_COUNT + ACL_HDR_LENGTH + L2CAP_HDR_LENGTH)
#endif

/**
* @brief  init sdp
* 
* @param DataSegment: pSDP memory
* @param TaskName: task name
*
* @return  init result
*
*/
BOOL sdpInit(void)
{
    int i;

    pSDP = osMemoryAllocate(RAM_TYPE_DATA_ON, sizeof(TSDP));
    
    pSDP->readOffset  = DATACB_INIT_OFFSET;
    pSDP->writeOffset = (BT_L1_HCI_DS_OFFSET + ACL_HDR_LENGTH + L2CAP_HDR_LENGTH);

    if (osPoolCreate(&pSDP->usSDPAttrDataPool, RAM_TYPE_DATA_OFF, TRUE, BT_US_SDP_BUFFER_SIZE,
        BT_SDP_CLIENT_BUFFER_COUNT, 4))
    {
        DebuggerBreak();
        return(FALSE);
    }

#if 0
    for (i=0;i<BT_SDP_MAX_CHANNELS_COUNT;i++)
    {
        pSDP->channels[i].chan     = 0;
        pSDP->channels[i].incoming = FALSE;
        pSDP->channels[i].used     = FALSE;
        pSDP->channels[i].self     = (uint8_t)i;
        pSDP->channels[i].cmdp     = NULL;
        pSDP->channels[i].respp    = NULL;
        pSDP->channels[i].flag     = 0;
        pSDP->channels[i].nextCode = 0;
        pSDP->channels[i].mtuSize  = 0;
    }
#else
    pSDP->pchannelsDon = osMemoryAllocate(RAM_TYPE_DATA_ON, sizeof(TsdpChanDesc)*otp_str_data.gEfuse_UpperStack_s.num_link_don);
    pSDP->pchannelsDoff = osMemoryAllocate(RAM_TYPE_DATA_OFF, sizeof(TsdpChanDesc)*otp_str_data.gEfuse_UpperStack_s.num_link_doff);

    for (i=0;i<otp_str_data.gEfuse_UpperStack_s.num_link_don;i++)
    {
        pSDP->pchannelsDon[i].chan     = 0;
        pSDP->pchannelsDon[i].incoming = FALSE;
        pSDP->pchannelsDon[i].used     = FALSE;
        pSDP->pchannelsDon[i].self     = (uint8_t)i;
        pSDP->pchannelsDon[i].cmdp     = NULL;
        pSDP->pchannelsDon[i].respp    = NULL;
        pSDP->pchannelsDon[i].flag     = 0;
        pSDP->pchannelsDon[i].nextCode = 0;
        pSDP->pchannelsDon[i].mtuSize  = 0;
    }

    for (i=0;i<otp_str_data.gEfuse_UpperStack_s.num_link_doff;i++)
    {
        pSDP->pchannelsDoff[i].chan     = 0;
        pSDP->pchannelsDoff[i].incoming = FALSE;
        pSDP->pchannelsDoff[i].used     = FALSE;
        pSDP->pchannelsDoff[i].self     = (uint8_t)i;
        pSDP->pchannelsDoff[i].cmdp     = NULL;
        pSDP->pchannelsDoff[i].respp    = NULL;
        pSDP->pchannelsDoff[i].flag     = 0;
        pSDP->pchannelsDoff[i].nextCode = 0;
        pSDP->pchannelsDoff[i].mtuSize  = 0;
    }
#endif

    for (i = 0; i < BT_SDP_SERVER_MAX_SERVICES; i++)
	{
		pSDP->services[i] = NULL;
	}
	
    /* create SDP database main record and enter as service zero */
    (void)sdpCreateDes(pSDP->mainRecord,
				        "< I<U> I<I> >",
				        SDP_ATTR_SERVICECLASSIDLIST, UUID_SERVICE_DISCOVERY_SERVER,
				        SDP_ATTR_VERSIONNUMBERLIST, 0x0100);

    pSDP->services[0] = pSDP->mainRecord;
    pSDP->offsets [0] = 0;

    pSDP->eirRunning   = FALSE;
    pSDP->eirPending   = FALSE;
    pSDP->shortName[0] = 0;      /* name is empty */
    pSDP->extraEIRData = NULL;

	osCoroutineCreate(&sdpQueueID, (TCoroutineEntry)NULL, (void *)pSDP);
	l2cUSendListenReq(PSM_SDP, sdpQueueID, 1);
    return(TRUE);
}

/**
* @brief  sdp service search request
*
* @param  pSDP: 
* @param  chan
* @param  id
* @param  sid
* @param  maxcnt
* @param  cstate
* @param  cstateLen
*
* @return  
*
*/
void sdpServiceSearchRequest(uint16_t chan, uint16_t id, LPCBYTE sid, uint16_t sidlen, uint16_t maxcnt, LPCBYTE cstate, uint8_t cstateLen)
{
    uint8_t * mybuf;
    uint16_t op = pSDP->writeOffset;

    uint16_t len = 2 + sidlen + 1 + cstateLen;
    if (osBufferGet(DownstreamPoolID, (uint16_t)(op + len + SDP_HEADER_LENGTH), (void  *)&mybuf))
    {
        SDP_TRACE_PRINTF_0(SDP_TRACE_MASK_ERROR, "!!sdpServiceSearchRequest: no memory (no request)");
        DebuggerBreak();
        return;
    }
	/*sdp header:
	 |PDU ID(1BYTE) | Transaction ID(2 Byte) | param Length(2 Byte)*/
    mybuf[op++] = SDP_SERVICE_SEARCH_REQUEST;
    NETSHORT2CHAR(mybuf+op, id);     op += 2;
    NETSHORT2CHAR(mybuf+op, len);    op += 2;
    memcpy(mybuf+op, sid, sidlen);   op += sidlen;
    NETSHORT2CHAR(mybuf+op, maxcnt); op += 2;       /* total service records */
    mybuf[op++] = cstateLen;                          /* cont. state = empty*/
    if (cstateLen)
	{
		memcpy(mybuf+op, cstate, cstateLen); op += cstateLen;
	}

    sdpSend(chan, mybuf, op);
}

/**
* @brief  handle sdp service search response
*
* @param  chan
* @param  id
* @param  maxIndex
* @param  hbuf
* @param  hcnt
* @param  follow
*
* @return  
*
*/
void sdpServiceSearchResponse(uint16_t chan, uint16_t id, uint16_t maxIndex, PCUINT32 hbuf, uint16_t hcnt, uint16_t follow)
{
    uint8_t * mybuf;
    uint16_t op = pSDP->writeOffset;
    uint16_t dlen,i;

    /* compute destination len */
    dlen = (uint16_t)(5 + 4 * hcnt);
    if (follow)
	{
		dlen += 2;
	}
    if (osBufferGet(DownstreamPoolID, (uint16_t)(op + dlen + SDP_HEADER_LENGTH), (void  *)&mybuf))
    {
        SDP_TRACE_PRINTF_0(SDP_TRACE_MASK_ERROR, "!!sdpServiceSearchResponse: no memory (no answer)");
        DebuggerBreak();
        return;
    }
    mybuf[op++] = SDP_SERVICE_SEARCH_RESPONSE;
    NETSHORT2CHAR(mybuf+op, id);        op += 2;
    NETSHORT2CHAR(mybuf+op, dlen);      op += 2; /* len */
    NETSHORT2CHAR(mybuf+op, maxIndex);  op += 2; /* total service records */
    NETSHORT2CHAR(mybuf+op, hcnt);      op += 2; /* current service records */
    for (i = 0; i < hcnt; i++)
    {
        NETLONG2CHAR (mybuf+op, hbuf[i]);
        op += 4;
    }
    if (follow) 
	{
        mybuf[op++] = 2;                            /* cont. state = 2 bytes */
        NETSHORT2CHAR(mybuf+op, follow);   op += 2;  /* next state */
    }
    else
	{
    	mybuf[op++] = 0;                            /* cont. state = empty */
	}

    sdpSend(chan, mybuf, op);
}

/**
* @brief  sdp send service attribute request
* 
* @param  pSDP
* @param  chan
* @param  id
* @param  handle
* @param  aid
* @param  aidlen
* @param  maxcnt
* @param  cstate
* @param  cstateLen
*
* @return  
*
*/
void sdpServiceAttributeRequest(uint16_t chan, uint16_t id, uint32_t handle, LPCBYTE aid, uint16_t aidlen, uint16_t maxcnt, LPCBYTE cstate, uint8_t cstateLen)
{
    uint8_t * mybuf;
    uint16_t op = pSDP->writeOffset;

    uint16_t len = 6 + aidlen + 1 + cstateLen;

    if (osBufferGet(DownstreamPoolID, (uint16_t)(op + len + SDP_HEADER_LENGTH), (void  *)&mybuf))
    {
        SDP_TRACE_PRINTF_0(SDP_TRACE_MASK_ERROR, "!!sdpServiceAttributeRequest: no memory (no request)");
        DebuggerBreak();
        return;
    }
    mybuf[op++] = SDP_SERVICE_ATTRIBUTE_REQUEST;
    NETSHORT2CHAR(mybuf+op, id);      op += 2;
    NETSHORT2CHAR(mybuf+op, len);     op += 2;
    NETLONG2CHAR(mybuf+op, handle);   op += 4;
    NETSHORT2CHAR(mybuf+op, maxcnt);  op += 2;       /* max attrib byte count */

    memcpy(mybuf+op, aid, aidlen);    op += aidlen;

    mybuf[op++] = cstateLen;                          /* cont. state = empty*/
    if (cstateLen)
	{
   		memcpy(mybuf+op, cstate, cstateLen); 
		op += cstateLen;
	}

    sdpSend(chan, mybuf, op);
}

/**
* @brief  sdpCode could be either SDP_SERVICE_ATTRIBUTE_RESPONSE or SDP_SERVICE_SEARCH_ATTRIBUTE_RESPONSE
* 
* @param  pSDP
* @param  chan
* @param  id
* @param  sdpPduId
* @param  aid
* @param  aLen
* @param  cState
*
* @return  
*
*/
void sdpServiceAttributeResponse(uint16_t chan, uint16_t id, uint8_t sdpPduId, LPCBYTE aid, uint16_t aLen, uint16_t cState)
{
    uint8_t * mybuf;
    uint16_t op        = pSDP->writeOffset;
    uint8_t cstateLen = 0;
    uint16_t len;

    if (cState)
	{
    	cstateLen = 2;
	}

    len = 2 + aLen + 1 + cstateLen;

    if (osBufferGet(DownstreamPoolID, (uint16_t)(op + len + SDP_HEADER_LENGTH), (void  *)&mybuf))
    {
        SDP_TRACE_PRINTF_0(SDP_TRACE_MASK_ERROR, "!!sdpServiceAttributeResponse: no memory (no response)");
        DebuggerBreak();
        return;
    }

    /* SDP header */
    mybuf[op++] = sdpPduId;
    NETSHORT2CHAR(mybuf+op, id);       op += 2;
    NETSHORT2CHAR(mybuf+op, len);      op += 2;

    NETSHORT2CHAR(mybuf+op, aLen);     op += 2;       /* max attrib byte count */
    memcpy(mybuf+op, aid, aLen);       op += aLen;

    mybuf[op++] = cstateLen;                          /* cont. state = empty*/
    if (cstateLen)
    {
        NETSHORT2CHAR(mybuf+op, cState); op += 2;
    }

    sdpSend(chan, mybuf, op);
}

/**
* @brief  sdp send service search
* 
* @param  pSDP
* @param  chan
* @param  id
* @param  sid
* @param  sidlen
* @param  maxcnt
* @param  aid
* @param  aidlen
* @param  cstate
* @param  cstateLen
*
* @return  
*
*/
void sdpServiceSearchAttributeRequest(uint16_t chan, uint16_t id,
												LPCBYTE sid, uint16_t sidlen,
												uint16_t maxcnt,
												LPCBYTE aid, uint16_t aidlen,
												LPCBYTE cstate, uint8_t cstateLen)
{
    uint8_t * mybuf;
    uint16_t op = pSDP->writeOffset;
    uint16_t len = sidlen + 2 + aidlen + 1 + cstateLen;

    if (osBufferGet(DownstreamPoolID, (uint16_t)(op + len + SDP_HEADER_LENGTH), (void  *)&mybuf))
    {
        SDP_TRACE_PRINTF_0(SDP_TRACE_MASK_ERROR, "!!sdpServiceAttributeRequest: no memory (no request)");
        DebuggerBreak();
        return;
    }
    mybuf[op++] = SDP_SERVICE_SEARCH_ATTRIBUTE_REQUEST;
    NETSHORT2CHAR(mybuf+op, id);        op += 2;

#if (X_SDP_QUALIFICATION_SUPPORT)
    if (sdpTestCase >= 720 && sdpTestCase < 730 /* SSA/BI02C */)
    {
        len         = 0x50; /* use value as provided in UPF 4 CAT 2 SDP Test 07 */
        debugOut(dbError,"!!patching sdp parametersize to %d",len);
        sdpTestCase = 0;
    }
#endif

    NETSHORT2CHAR(mybuf+op, len);      op += 2;
    memcpy(mybuf+op, sid, sidlen);     op += sidlen;  /* service ids */

#if defined (UPF_TEST)
    if (btTestCase == 3) 
	{
        maxcnt = 0x045;         /* limit max attrib byte count for this testcase */
    }
#endif  /* defined (UPF_TEST) */

    NETSHORT2CHAR(mybuf+op, maxcnt);   op += 2;       /* max attrib byte count */
    memcpy(mybuf+op, aid, aidlen);     op += aidlen;  /* attribute lists */
    mybuf[op++] = cstateLen;                          /* cont. state = empty*/
    if (cstateLen)
	{
		memcpy(mybuf+op, cstate, cstateLen);
		op += cstateLen;
	}

    sdpSend(chan, mybuf, op);
}

/**
* @brief  sdp handle data indicate
* 
* @param  p
* @param  chan
* @param  length
*
* @return  
*
*/
void sdpHandleLDataInd(uint8_t * p, uint16_t chan, uint16_t length)
{
    uint8_t code;
    uint16_t id;
    uint16_t len;
    uint16_t pos=0;
    TsdpChanDesc * cp;

    /* iterate over all sdp commands in this l2cap pdu */
    while (pos < length)
    {
        code = p[pos++];
        id   = NETCHAR2SHORT(p+pos); pos += 2;
        len  = NETCHAR2SHORT(p+pos); pos += 2;
        cp   = sdpGetChannel(chan);

        if (!cp)   /* channel not found */
    	{
       		break; 
    	}
        SDP_TRACE_PRINTF_6(SDP_TRACE_MASK_TRACE, "SDP: DATA_IND  pos %d chan %d code 0x%X id %d len %d incoming %d",
            				pos, chan, code, id, len, cp->incoming);

        if (cp->incoming)                                         /* Server Mode */
        {
            /* new on UPF4: restart the timer #1 on each SDP transaction */
            sdpResetTimer(1, cp);
            sdpSetTimer(1, cp, SDP_TIMEOUT_SERVER);

            /* this is the server part: check if pdu is within l2cap pdu */
            if ((pos+len) > length)
            {
                SDP_TRACE_PRINTF_3(SDP_TRACE_MASK_ERROR, "!!SDP: DATA_IND  len error pos %d len %d l2cap %d",
                    				pos, len, length);
                sdpError(chan, id, SDP_INVALID_PDU_SIZE);
                break;
            }

            switch (code)
            {
            case SDP_SERVICE_SEARCH_REQUEST:
                sdpHandleServiceSearchRequest(cp, p+pos, id, len);
                break;
            case SDP_SERVICE_ATTRIBUTE_REQUEST:
                sdpHandleServiceAttributeRequest(cp, p+pos, id, len);
                break;
            case SDP_SERVICE_SEARCH_ATTRIBUTE_REQUEST:
                sdpHandleServiceSearchAttributeRequest(cp, p+pos, id, len);
                break;
            default:
                SDP_TRACE_PRINTF_1(SDP_TRACE_MASK_TRACE, "SDP-SERVER: unhandled code %X", code);
                sdpError(chan, id, SDP_INVALID_REQUEST_SYNTAX);
				break;
            } /* switch */
        } /* if */
        else                                                     /* Client Mode */
        {
            /* this is the client part: check if pdu is identical to l2cap pdu */
            if ((pos+len) != length)
            {
                SDP_TRACE_PRINTF_3(SDP_TRACE_MASK_ERROR, "!!SDP-CLIENT: ill len pos %d len %d lg %d",
                    				pos, len, length);
				/* 05.04.2002 KM: new after UPF 7.5: we received a not welfromed reply to our request. we do
				   not isue a silent discard (and wait for timeout), but generate an empty answer to app layer,
				   thus sdp will procedd directly, without timeout
				*/
                sdpCloseLink(cp, (uint16_t)(SDP_ERR | SDP_INVALID_RESPONSE_SYNTAX), 0 );
                break;
            }

            switch (code)
            {
            case SDP_SERVICE_SEARCH_RESPONSE:
                sdpHandleServiceSearchResponse(cp, p+pos, id, len);
                break;
            case SDP_SERVICE_ATTRIBUTE_RESPONSE:
                sdpHandleServiceAttributeResponse(cp, p+pos, id, len);
                break;
            case SDP_SERVICE_SEARCH_ATTRIBUTE_RESPONSE:
                sdpHandleServiceSearchAttributeResponse(cp, p+pos, id, len);
                break;
            case SDP_ERROR_RESPONSE:
            {
                uint16_t status;
                status = NETCHAR2SHORT(p+pos); pos += 2;
                SDP_TRACE_PRINTF_1(SDP_TRACE_MASK_TRACE, "SDP-CLIENT: SDP_ERROR_RESPONSE %X", status);
                sdpResetTimer(2, cp);
                sdpCloseLink(cp, (uint16_t)(SDP_ERR | status), 0);
            }
                break;
            default:
                SDP_TRACE_PRINTF_1(SDP_TRACE_MASK_TRACE,"SDP-CLIENT: unhandled code %X", code);
                /* Close the link (application will get disc_ind) */
                sdpResetTimer(2, cp);
                sdpCloseLink(cp, (uint16_t)(SDP_ERR | SDP_INVALID_RESPONSE_SYNTAX), 0);
				break;
            } /* switch */
        } /* else */
        pos += len; /* skip forward to next pdu within this l2cap pdu */
    } /* while */	
}

/**
* @brief  sdp attribute confirm
* 
* @param  tc
* @param  pSDP
*
* @return  
*
*/
void sdpAttributeConf(TsdpChanDesc *tc)
{

	LPbtLink           pLink;
	LPsdpAttributeConf attributeConf = (LPsdpAttributeConf) (tc->respp + pSDP->readOffset);
	uint8_t               handle        = (uint8_t) tc->self;

	if (tc->flag & DATA_CB_RELEASE)
	{
		osBufferRelease(tc->cmdp);
	}
	tc->cmdp     = NULL;

	pLink = blueFaceFindLinkByHandle(handle, BLUEFACE_PSM_SDP);
	if (pLink == NULL)
	{
		osBufferRelease(tc->respp);
		tc->respp	 = NULL;
		tc->nextCode = 0;
		SDP_TRACE_PRINTF_1(SDP_TRACE_MASK_ERROR, "!!SDP_ATTRIBUTE_CONF: cannot find handle 0x%X", handle);
	}
	else
	{
		/* send message via BAPI to application */
		blueFaceSendBT_SDP_ATTRIBUTE_CONF(pLink->context,
											attributeConf->aList,
											attributeConf->byteCount,
											pSDP->readOffset, tc);
	}
}

/**
* @brief  get sdp chan descriptor by l2cap channel id(cid)
* 
* @param  pSDP
* @param  chan
*
* @return  
*
*/
TsdpChanDesc * sdpGetChannel(uint16_t chan)
{
    int i;
	
#if 0	
    for (i = 0; i < BT_SDP_MAX_CHANNELS_COUNT; i++) 
	 {
        if (pSDP->channels[i].used && pSDP->channels[i].chan == chan)
    	{
        	return &pSDP->channels[i];
    	}
    }
#else
	 for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_don; i++) 
	 {
        if (pSDP->pchannelsDon[i].used && pSDP->pchannelsDon[i].chan == chan)
    	{
        	return &pSDP->pchannelsDon[i];
    	}
    }
	 for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_doff; i++) 
	 {
        if (pSDP->pchannelsDoff[i].used && pSDP->pchannelsDoff[i].chan == chan)
    	{
        	return &pSDP->pchannelsDoff[i];
    	}
    }
#endif

    SDP_TRACE_PRINTF_1(SDP_TRACE_MASK_ERROR, "!!sdpGetChannel: cannot find descriptor chan %X", chan);
    return NULL;
}

/**
* @brief  get sdp channel descriptor by bdaddr
*
* @param  pSDP:
* @param  bd: 
*
* @return  
*
*/
TsdpChanDesc * sdpGetChannelByBd(LPCBYTE bd)
{
	int i;
	TsdpChanDesc * lch;

#if 0
	for (i = 0; i < BT_SDP_MAX_CHANNELS_COUNT; i++)
	{
		lch = &pSDP->channels[i];
		if (lch->used && memcmp(lch->bd, bd, BD_ADDR_SIZE) == 0 && !lch->incoming)
		{
			return lch;
		}
	}
#else		
	for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_don; i++)
	{
		lch = &pSDP->pchannelsDon[i];
		if (lch->used && memcmp(lch->bd, bd, BD_ADDR_SIZE) == 0 && !lch->incoming)
		{
			return lch;
		}
	}
	for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_doff; i++)
	{
		lch = &pSDP->pchannelsDoff[i];
		if (lch->used && memcmp(lch->bd, bd, BD_ADDR_SIZE) == 0 && !lch->incoming)
		{
			return lch;
		}
	}
#endif

	return NULL;
}

/**
* @brief  allocate sdp channel
*
* @param  pSDP:
* @param  chan: 
* @param  incoming
*
* @return  allocated channel
*
*/
TsdpChanDesc * sdpAllocateChannel(uint16_t chan, BOOL incoming)
{
	int i;

#if 0	
	for (i = 0; i < BT_SDP_MAX_CHANNELS_COUNT; i++)
	{
		if (!pSDP->channels[i].used)
		{
			pSDP->channels[i].used     = TRUE;
			pSDP->channels[i].chan     = chan;
			pSDP->channels[i].incoming = incoming;
			pSDP->channels[i].mtuSize  = 0;
			return &pSDP->channels[i];
		}
	}
#else	
	for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_don; i++)
	{
		if (!pSDP->pchannelsDon[i].used)
		{
			pSDP->pchannelsDon[i].used     = TRUE;
			pSDP->pchannelsDon[i].chan     = chan;
			pSDP->pchannelsDon[i].incoming = incoming;
			pSDP->pchannelsDon[i].mtuSize  = 0;
            pSDP->pchannelsDon[i].TimerHandle = 0;
			return &pSDP->pchannelsDon[i];
		}
	}
	for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_doff; i++)
	{
		if (!pSDP->pchannelsDoff[i].used)
		{
			pSDP->pchannelsDoff[i].used     = TRUE;
			pSDP->pchannelsDoff[i].chan     = chan;
			pSDP->pchannelsDoff[i].incoming = incoming;
			pSDP->pchannelsDoff[i].mtuSize  = 0;
            pSDP->pchannelsDoff[i].TimerHandle = 0;
			return &pSDP->pchannelsDoff[i];
		}
	}
#endif

	SDP_TRACE_PRINTF_1(SDP_TRACE_MASK_ERROR, "!!sdpAllocateChannel: cannot find free descriptor for chan %X", chan);
	return NULL;
}

/**
* @brief make space for new element in eirP.
*		eirP points to partial, but syntactical correct EIR
*   		of 240 (BLUEFACE_EXTENDED_INQUIRY_RESPONSE_SIZE) bytes.
*
*   		On function entry,  <size> defines the number of bytes required for new entry (2/4/16 bytes)
*
*  		On function return, free space for new element is directly at the beginning of the correct section and
*   		pointer to length field of that section is returned. the length field already updated.
*
*   		Truncates lists if required and indicates this in the type field (list is only partial...).
*
* @param  eirP: eir data point
* @param  size
*
* @return  
*
*/
uint8_t * sdpEIRmakeRoom(uint8_t * eirP, int size)
{

	uint8_t * p16;
	uint8_t * p32;
	uint8_t * p128;
	uint8_t * pe;
	uint8_t * p;
	uint8_t * loc;

	while (1)
	{
		/* update local pointers into EIR structure */
		p16   = eirP + eirP[0] + 1;
		p32   = p16  + p16[0] + 1;
		p128  = p32  + p32[0] + 1;
		pe    = p128 + p128[0] + 1;
		pe[0] = 0; /* insert terminator */

		/* stop if there is enough room for new element ... */
		if ((pe + size) < (eirP + BLUEFACE_EXTENDED_INQUIRY_RESPONSE_SIZE))
		{
			break;
		}
		/* new item does not fit, throw away something: Throw away any 128 bit elements, then
		any 32 bit elements, then any 16 bit elements. */
		if (p128[0] > 1)
		{
			/* at least one long UUID128, throw the last one away (always sufficient for one new entry */
			p128[0] -= 16;
			p128[1]  = SDP_EIR_UUID128_MORE;      /* show that list is truncated */
		} /* 128 bit not empty */
		else if (p32[0] > 1)
		{
			/* no entry in uuid128 list, at least one entry in uuid32 list, delete last enry */
			uint8_t * p128n;
			p32[0]  -= 4;
			p32[1]   = SDP_EIR_UUID32_MORE;       /* show that list is truncated */
			p128n    = p32 + p32[0] + 1;
			p128n[0] = 1;                         /* UUID128 list is and was empty */
			p128n[1] = p128[1];                   /* copy type field (partial / complete) from 128 bit list */
		} /* 32 bit not empty */
		else
		{
			/* no entry in uuid128 + uuid32 list, at least one entry in uuid16 list, delete last entry */
			uint8_t * p32n;
			uint8_t * p128n;
			p16[0]  -= 2;
			p16[1]   = SDP_EIR_UUID16_MORE;       /* show that list is truncated */
			p32n     = p16  + p16[0] + 1;
			p128n    = p32n + p32n[0] + 1;
			p32n[0]  = 1;                         /* UUID32 list is and was empty */
			p32n[1]  = p32[1];                   /* copy type field (partial / complete) from 32 bit list */
			p128n[0] = 1;                         /* UUID128 list is and was empty */
			p128n[1] = p128[1];                   /* copy type field (partial / complete) from 128 bit list */
		} /* 16 bit not empty */
	} /* while (1) */

	/* enough space for new element is now available and all the pointers p16, p32 and p128 are up to date */
	if (size == 2)
	{
		loc = p16;
	}
	else if (size == 4)
	{
		loc = p32;
	}
	else
	{
		loc = p128;
	}

	/* move all bytes up by <size> positions */
	p = pe-1;
	while (p >= loc+2) /* loc+2 is first data byte after type field */
	{
		p[size] = p[0];
		p--;
	}
	loc[0] += size;   /* update the length field of this section */
	return loc;
} /* sdpEIRmakeRoom */

/**
* @brief Insert data to EIR data
* 		eirP points to valid, partial EIR record
*		insert bytes according to <mode> (SDP_EIR_UUID16 / 32 / 128)  from val or uuidP at the proper entry
*		within eirP, if not already present within this EIR (check for duplicates).
*		For 16 bit and 32 bit entries, <val> contains the UUID, for 128 bit entries, uuidP points to the UUID.
*		If there is not enough room for new entry, restructure the EIR (delete some entries) and reflect the
*		fact of the deletion in the type field /show this entry is now only partial).
*		Deletions occur in the sequence: delete from -128 bit entries, delete from -32 bit entries, delete from -16 bit
*		entries.
*
* @param  eirP: eir data point
* @param  val: value
* @param  uuidP
* @param  mode: uuid type
*
* @return  
*
*/
void sdpEIRInsert(uint8_t * eirP, uint32_t val, uint8_t * uuidP, int mode)
{
	uint8_t * p16  = eirP + eirP[0] + 1;  /* point to length field of 16 bit list */
	uint8_t * p32  = p16  + p16[0] + 1;   /* point to length field of 32 bit list */
	uint8_t * p128 = p32  + p32[0] + 1;   /* point to length field of 128 bit list */
	uint8_t * pe   = p128 + p128[0] + 1;  /* point to first free byte at end of list (terminator) */
	uint8_t * p;
	int    i;
	switch (mode)
	{
	case SDP_EIR_UUID16:
		/* search for duplicates */
		p = p16+2; /* position at start of sequence, after length and type */
		while (p<p32)
		{
			uint32_t lval = CHAR2SHORT(p);
			if (lval == val)
			{
		   		return; /* entry already exists --> all done, stop operation NOW */
			}
			p += 2; /* position to next entry */
		}
		p = sdpEIRmakeRoom(eirP, 2);
		SDP_TRACE_PRINTF_1(SDP_TRACE_MASK_ERROR, "sdpEIRInsert: insert UUID16 %X", val);
		/* enter in UUID16 list */
		SHORT2CHAR(p+2, val );
		break;

	case SDP_EIR_UUID32:
		p = p32+2; /* position at start of sequence, after length and type */
		while (p<p128)
		{
			uint32_t lval = CHAR2LONG(p);
			if (lval == val)
			{
					return; /* entry already exists --> all done,  stop operation NOW */
			}
			p += 4; /* position to next entry */
		}
		p = sdpEIRmakeRoom(eirP,4);
		SDP_TRACE_PRINTF_1(SDP_TRACE_MASK_ERROR, "sdpEIRInsert: insert UUID32 %X", val);
		/* enter in UUID32 l?st */
		LONG2CHAR(p+2, val );
		break;

	case SDP_EIR_UUID128:
		/* check for duplicates:  .... */
		p = p128 + 2;   /* position at start of sequence, after length and type */
		while (p < pe)
		{
			/* compare UUID at uuidP with the uuid located at p, p is reversed !! */
			BOOL identical = TRUE;
			for (i=0;i<16;i++)
			{
				if (p[i] != uuidP[15-i])
				{
					identical = FALSE;
				}
			}
			if (identical)
			{
				return; /* entry already exists --> all done,  stop operation NOW */
			}
			p += 16; /* skip to next UUID entry within the list */
		} /* while */
		p = sdpEIRmakeRoom(eirP, 16);
		SDP_TRACE_PRINTF_1(SDP_TRACE_MASK_ERROR, "sdpEIRInsert: insert UUID128 %X", uuidP);
		/* enter in UUID128 l?st: UUID comes from memory in NETWORK byte order, goes into packet in INTEL byte order */
		for (i=0;i<16;i++)
		{
			p[2+i] = uuidP[15-i];
		}
		break;

	default:
		break;
	} /* switch */
} /* sdpEIRInsert */

/**
* @brief  called when a EIR operation is finished
*		when the buffer is released
* 		check if there is a pending operation and start is operation if required
*
* @param  handle
*
* @return  
*
*/
void sdpBufferCallback(uint32_t handle)
{
	PSDP pSDP = (PSDP)handle;

	pSDP->eirRunning = FALSE; /* last operation completed */
	if (pSDP->eirPending)
	{
		/* if there is a pending operation, clear pending flag and start that operation */
		pSDP->eirPending = FALSE;
		sdpNewEIR();
	}
} /* sdpBufferCallback */

/**
* @brief   Generate a new Extended Inquiry Response
*		 send it (via L2CAP) to HCI Layer
*   		Call this function when anything related / contained in the EIR structure has changed.
*   	EIR contains:
*	     - local device friendly name (may be shortened)
*	     - UUID list of local services (service class uuid list) (may be shortended, truncated)
*	     - (optional) transmit power level of inquiry response
*	     - (optional) manufacturer specific date
*	     - (optional flag byte (no flags currently defined)
*
* @param  pSDP
*
* @return  
*
*/
void sdpNewEIR(void)
{
	uint8_t * eirP; 		/* points to buffer of 240 byte length */
	uint8_t * idListP; 	/* pointer to id List  */
	uint8_t * lp;
	uint8_t * uuidP;
	uint32_t  uuid;
	int i;    			/* index into list of SDP service records */
	int aix;
	int nl,pos;

	LPHCI_CONF_EXTENDED_INQUIRY_RESPONSE eirResponse;

	if (pSDP->eirRunning)
	{
		/* if there is already a eir operation running, wait for this operation to complete and just indicate this
		pending operation. This operation will be completed upon callback */
		pSDP->eirPending = TRUE;
		return;
	}

	/* allocate memory for EIR operation */
	if (osBufferGet(BTSystemPoolID, 1 + BLUEFACE_EXTENDED_INQUIRY_RESPONSE_SIZE, (PVOID) &eirResponse))
	{
		SDP_TRACE_PRINTF_1(SDP_TRACE_MASK_TRACE, "!!sdpnewEIR: no memory, len %d", 1 + BLUEFACE_EXTENDED_INQUIRY_RESPONSE_SIZE);
		DebuggerBreak();
		return;
	}

	eirResponse->fec = 0; 						/* FEC not required */
	eirP = &eirResponse->extendedResult[0];
	pSDP->eirRunning = TRUE;

	/* insert callback into buffer (for flow control purposes) */
	osBufferCallBackSet(eirResponse, sdpBufferCallback, (uint32_t)pSDP);

	/* prepare basic record: all uuid types present by default, but maybe empty (and always complete) */
	memset(eirP, 0, BLUEFACE_EXTENDED_INQUIRY_RESPONSE_SIZE);

	/* insert short name first (might be the complete name) */
	nl = strlen((char  *)pSDP->shortName);
	eirP[0] = (uint8_t)(nl+1);
	eirP[1] = (nl >= ((int)sizeof(pSDP->shortName)-1))? SDP_EIR_NAME_SHORT : SDP_EIR_NAME;
	memcpy(eirP+2, pSDP->shortName, nl); /* non zero terminated friendly name string */
	pos = nl + 2;

	/* next insert empty records for each UUID type */
	eirP[pos++] = 1; /* length */
	eirP[pos++] = SDP_EIR_UUID16;
	eirP[pos++] = 1; /* length */
	eirP[pos++] = SDP_EIR_UUID32;
	eirP[pos++] = 1; /* length */
	eirP[pos++] = SDP_EIR_UUID128;
	eirP[pos++] = 0; /* length = terminator */

	/* collect all relevant UUIDs (all sizes) from all defined services */
	for (i = 0; i < BT_SDP_SERVER_MAX_SERVICES; i++)
	{
		lp = pSDP->services[i];
		if (lp)
		{
			/* lp points to SDP record with attribute / value pairs : goto service class id list  */
			idListP = sdpFindAttribute(lp, NULL, SDP_ATTR_SERVICECLASSIDLIST);
			if (idListP) /* service class id list found in this record */
			{
			   /* this list consist of UUID of mixed and arbitrary sizes. get minimum size and put into corresponding list */
			   aix = 0;
			   while (1)
			   {
			       aix++; /* index start at one */
			       uuidP = sdpAccessElement(idListP, NULL, aix);
			       if (uuidP == NULL)
			       {
						break; /* reached end of list */
			       }
			       uuid = sdpGetDValue(uuidP, NULL);
			       if (uuid < 0x10000)
			       {
						/* 16 bit uuid */
						sdpEIRInsert(eirP, uuid, NULL, SDP_EIR_UUID16);
			       }
			       else if (uuid != 0xffffffff)
			       {
						/* 32 bit uuid */
						sdpEIRInsert(eirP, uuid, NULL, SDP_EIR_UUID32);
			       }
			       else
			       {
						/* 128 bit uuid */
						uint8_t typ;
						uint16_t len;
						sdpEIRInsert(eirP, 0 /* not used */, sdpDecodeElement(uuidP, NULL, &len, &typ), SDP_EIR_UUID128);
			       }
			   } /* while (1) */
			} /* idListP (serviceclassidlist exists) */
		} /* lp (service exists */
	} /* for */

    if (pSDP->VendorID || pSDP->ProductID || pSDP->ProductVersion || pSDP->IDSource)
    {
        uint8_t * p16  = eirP + eirP[0] + 1;  /* point to length field of 16 bit list */
        uint8_t * p32  = p16  + p16[0] + 1;   /* point to length field of 32 bit list */
        uint8_t * p128 = p32  + p32[0] + 1;   /* point to length field of 128 bit list */
        uint8_t * pe   = p128 + p128[0] + 1;  /* point to first free byte at end of list (terminator) */
        pe[0] = 0x09;
        pe[1] = 0x10;
        SHORT2CHAR(pe+2, pSDP->IDSource);
        SHORT2CHAR(pe+4, pSDP->VendorID);
        SHORT2CHAR(pe+6, pSDP->ProductID);
        SHORT2CHAR(pe+8, pSDP->ProductVersion);
    }

    if (pSDP->extraEIRData)
    {
        uint8_t * p16  = eirP + eirP[0] + 1;  /* point to length field of 16 bit list */
        uint8_t * p32  = p16  + p16[0] + 1;   /* point to length field of 32 bit list */
        uint8_t * p128 = p32  + p32[0] + 1;   /* point to length field of 128 bit list */
        uint8_t * pid  = p128 + p128[0] + 1;  /* point to first free byte at end of list (terminator) */
        uint8_t * pe   = pid  + pid[0] + 1;
        uint8_t   length = strlen(pSDP->extraEIRData);
        if (length+(pe-eirP) > BLUEFACE_EXTENDED_INQUIRY_RESPONSE_SIZE)
        {
            length = BLUEFACE_EXTENDED_INQUIRY_RESPONSE_SIZE - (pe-eirP);
        }
        memcpy(pe, pSDP->extraEIRData, length);
    }

	hciHandleUpWriteExtendedInquiryResponse(eirResponse);
} /* sdpNewEIR */

/**
* @brief  send sdp msg to peer entry
*
* @param  pSDP: 
* @param  chan
* @param  buf
* @param  pos
*
* @return  
*
*/
void sdpSend(uint16_t chan, uint8_t * buf, uint16_t pos)
{
    MESSAGE_T msg;
    TsdpChanDesc * tc = sdpGetChannel(chan);

    if (tc && ((pos-pSDP->writeOffset) > tc->mtuSize))
    {
		/* message too large for this link */
		SDP_TRACE_PRINTF_4(SDP_TRACE_MASK_TRACE, "!!sdpSend: message to long chan %d mtu %d len %d",
		  					chan, tc->inMtuSize, pos-pSDP->writeOffset, 0);
		/* terminate the transaction */
		sdpCloseLink(tc, 0, 0);
		return ;
    }

    msg.MData.DataCBChan.BufferAddress  = buf;
    msg.MData.DataCBChan.Offset         = pSDP->writeOffset;
    msg.MData.DataCBChan.Flag           = DATA_CB_RELEASE;
    msg.MData.DataCBChan.Channel        = chan;
    msg.MData.DataCBChan.Length         = pos - pSDP->writeOffset;

	l2cHandleL2C_DATA_REQ(&msg, FALSE);
}

/**
* @brief  send SDP_ERROR_RESPONSE to Peer Entity
*
* @param  pSDP: 
* @param  chan
* @param  id
* @param  error
*
* @return  
*
*/
void sdpError(uint16_t chan, uint16_t id, uint8_t error)
{
	uint8_t * mybuf;
	uint16_t op = pSDP->writeOffset;
	if (osBufferGet(DownstreamPoolID, (uint16_t)(op + 2 + SDP_HEADER_LENGTH), (void  *)&mybuf))
	{
	    SDP_TRACE_PRINTF_1(SDP_TRACE_MASK_ERROR, "!!sdpError: could not send error message %X", error);
	    DebuggerBreak();
	    return;
	}
	mybuf[op++] = SDP_ERROR_RESPONSE;
	NETSHORT2CHAR(mybuf+op, id);    op += 2;
	NETSHORT2CHAR(mybuf+op, 2);     op += 2; /* len */
	NETSHORT2CHAR(mybuf+op, (uint16_t)error); op += 2;
	sdpSend(chan, mybuf, op);
}

/**
* @brief  create Data Element Sequence specific header at buf+bp, return new bp, typ,len describe element
* 
* @param  p
* @param  pe
*
* @return  
*
*/
uint16_t sdpGetValue(uint8_t * p, uint8_t * pe)
{
	uint16_t len;
	uint8_t typ;
	uint8_t * lp;

	if (!p)              /* we wanna be on the very save side */
	{
		return 0xffff;
	}
	lp = sdpDecodeElement(p, pe, &len, &typ);
	if (!lp)
	{
   		return 0xffff;
	}

	switch (len)
	{
	case 0:
	   return 0;
	case 1:
	   return *lp;
	case 2:
	   return NETCHAR2SHORT(lp);
	case 4:
	   return (uint16_t)(NETCHAR2LONG(lp));
	default:
	   return 0xffff;   /* safety exit */
	}
}

/**
* @brief  handle sdp service search request
*
* @param  pSDP: 
* @param  tc
* @param  buf
* @param  id
* @param  blen
*
* @return  
*
*/
void sdpHandleServiceSearchRequest(CONST TsdpChanDesc * tc, uint8_t * buf, uint16_t id, uint16_t blen)
{
    uint8_t typ;
    uint8_t err;
    uint16_t len;
    uint16_t index;
    uint8_t cstatelen;
    uint16_t maxServiceCnt;
    uint16_t maxIndex;
    uint16_t maxServiceThisPDU;
    uint8_t * pl;
    uint8_t * be = buf + blen;
    uint16_t hcnt   = 0;
    uint16_t follow = 0;
    uint32_t hbuf[SDP_MAX_HDL];

    SDP_TRACE_PRINTF_2(SDP_TRACE_MASK_TRACE, "SDP: SDP_SERVICE_SEARCH_REQUEST chan %X id %X", tc->chan, id );

    if (blen < 5) /* at least: des (2), maxserv (2), cstate (1) */
    {
        sdpError(tc->chan, id, SDP_INVALID_REQUEST_SYNTAX);
        return;
    }
    pl = sdpDecodeElement(buf, buf+blen, &len, &typ);
    if (!pl)
    {
        sdpError(tc->chan, id, SDP_INVALID_REQUEST_SYNTAX);
        return;
    }
    pl += len; /* point to next element after the sequence */

    if ((pl + 3) > be) /* at least: maxserv (2), cstate (1) */
    {
        sdpError(tc->chan, id, SDP_INVALID_REQUEST_SYNTAX);
        return;
    }

    maxServiceCnt = NETCHAR2SHORT(pl); pl += 2;
    cstatelen     = *pl++;

    if (cstatelen==0)
    {
        index = 0;
    }
    else if (cstatelen==2)
    {
        index = NETCHAR2SHORT(pl); pl += 2;
    }
    else
    {
        sdpError(tc->chan, id, SDP_INVALID_REQUEST_SYNTAX);
        return;
    }

    if (pl != be)
    {
        sdpError(tc->chan, id, SDP_INVALID_REQUEST_SYNTAX);
        return;
    }

    /* limit the number of responses to generate according to MTU size this channel !! */
    assert(tc->mtuSize - (SDP_HEADER_LENGTH + 2 + 2 + 1 + 2) >= 0);
    maxServiceThisPDU = ((uint16_t)(tc->mtuSize - (SDP_HEADER_LENGTH + 2 + 2 + 1 + 2))) / sizeof(uint32_t);
    if (maxServiceThisPDU > SDP_MAX_HDL)
	{
    	maxServiceThisPDU = SDP_MAX_HDL;
	}

    err = sdpDbServiceSearch(
							buf, buf+blen,              /* items to search for */
							index, maxServiceCnt,       /* startindex for items to return, absolute maximum */
							hbuf,                       /* buffer for handles */
							maxServiceThisPDU,          /* max of items to return in this buffer */
							&maxIndex);                 /* total no of items found */

    if (err != SDP_SUCCESS)
    {
        sdpError(tc->chan, id, err);
        return;
    }

    /* preparations for the answer */
    if (index < maxIndex)                   /* some entrys generated */
    {
        hcnt = maxIndex - index;            /* num of entrys above index */
        if (hcnt > maxServiceThisPDU)       /* more than could fit -> follow state exists */
        {
            follow = index + maxServiceThisPDU; /* set correct follow state */
            hcnt   = maxServiceThisPDU;     /* show correct number of handles */
        }
    }

    sdpServiceSearchResponse(tc->chan, id, maxIndex, (PCUINT32)hbuf, hcnt, follow);
} /* sdpHandleServiceSearchRequest */

/**
* @brief  sdp handle service search response
* 
* @param  pSDP
* @param  tc
* @param  buf
* @param  id
* @param  blen
*
* @return  
*
*/
void sdpHandleServiceSearchResponse(TsdpChanDesc * tc, uint8_t * buf, uint16_t id, uint16_t blen)
{
    uint16_t totalServiceRecords;
    uint16_t currentServiceRecords;
    uint8_t * handleList;
    uint8_t cstateLen = 100; /* some illegal value */
    uint8_t * cstate = buf;
    uint16_t i;
    uint16_t bp = 0;

    LPsdpSearchConf searchConf = (LPsdpSearchConf) (tc->respp + pSDP->readOffset);
    LPsdpSearchReq searchReq   = (LPsdpSearchReq)  (tc->cmdp + tc->offset);

    if (tc->id != id || tc->nextCode != SDP_SERVICE_SEARCH_RESPONSE)
    {
        SDP_TRACE_PRINTF_2(SDP_TRACE_MASK_ERROR, "!!sdpHandleServiceSearchResponse: wrong id %X (%X) or unexpected answer", id, tc->id);
        sdpCloseLink(tc, SDP_ERR | SDP_INVALID_RESPONSE_SYNTAX, 0);
        return;
    }

    totalServiceRecords   = NETCHAR2SHORT(buf+bp); bp += 2;
    currentServiceRecords = NETCHAR2SHORT(buf+bp); bp += 2;

    handleList            = buf+bp;
    bp += (uint16_t)(currentServiceRecords * sizeof(uint32_t));
    if (bp < blen)
    {
        cstateLen             = buf[bp++];
        cstate                = buf+bp;
        bp                   += cstateLen;
    }

    SDP_TRACE_PRINTF_8(SDP_TRACE_MASK_TRACE, "SDP: SDP_SERVICE_SEARCH_RESPONSE chan %X id %X total %X current %X cstateLen %X",
        				tc->chan, id, totalServiceRecords, currentServiceRecords, cstateLen,0 ,0, 0);

    if (bp != blen
        || cstateLen > 16
        || searchConf == NULL
        || searchReq == NULL
        || totalServiceRecords > searchReq->maxHandles
        || (tc->index + currentServiceRecords) > totalServiceRecords
        )
    {
        SDP_TRACE_PRINTF_0(SDP_TRACE_MASK_ERROR, "!!sdpHandleServiceSearchResponse: Syntax Error in Response");
        sdpCloseLink(tc, SDP_ERR | SDP_INVALID_RESPONSE_SYNTAX, 0);
        return;
    }

    for (i = 0; i < currentServiceRecords; i++)
	{
    	searchConf->handles[i + tc->index] = NETCHAR2LONG(handleList + i * sizeof(uint32_t));
	}
    tc->index += currentServiceRecords;

    if (cstateLen) /* more data waits for us */
    {
        LPsdpSearchReq pSearchReq = (LPsdpSearchReq) (tc->cmdp + tc->offset);
        sdpServiceSearchRequest(tc->chan, ++tc->id,
								pSearchReq->uuidList, pSearchReq->uuidLen,
								pSearchReq->maxHandles,
								cstate, cstateLen);
    }
    else
    {
   
        searchConf->totalHandles   = tc->index;
        searchConf->linkid         = tc->self;

        if (tc->flag & DATA_CB_RELEASE)
    	{
        	osBufferRelease(tc->cmdp);
    	}
        tc->cmdp     = NULL;

        sdpSearchConf(tc);
        sdpResetTimer(2, tc);
    }
}

/**
* @brief  sdp handle SDP_SERVICE_ATTRIBUTE_REQUEST
* 
* @param  pSDP
* @param  tc
* @param  buf
* @param  id
* @param  blen
*
* @return  
*
*/
void sdpHandleServiceAttributeRequest(CONST TsdpChanDesc * tc, uint8_t * buf, uint16_t id, uint16_t blen)
{
    uint32_t handle;
    uint16_t  maxByteCount; /* max data requested from application */
    uint16_t  maxPayload;   /* max Payload this PDU */
    uint8_t * attrList;
    uint8_t * pl;          /* Payload of first level sequence */
    uint16_t index = 0;     /* Incoming continuation state */
    uint16_t lbp = 0;       /* index into lbuf */
    uint8_t * be = buf + blen; /* end of buffer pointer */

    uint8_t * cstateLenP;
    uint8_t typ;
    uint16_t len;
    uint16_t newIndex;      /* Outgoing continuation state */

    if (blen < 9)
    {
        sdpError(tc->chan, id, SDP_INVALID_REQUEST_SYNTAX);
        return;
    }
    handle       = NETCHAR2LONG(buf);  buf += 4;
    maxByteCount = NETCHAR2SHORT(buf); buf += 2;
    attrList     = buf;
    pl = sdpDecodeElement(attrList, be, &len, &typ);
    if (!pl || typ != SDP_TYP_SEQUENCE || len == 0 || maxByteCount < 7)
    {
        sdpError(tc->chan, id, SDP_INVALID_REQUEST_SYNTAX);
        return;
    }
    buf        = pl + len;
    cstateLenP = buf++;

    if (cstateLenP >= be)
    {
        sdpError(tc->chan, id, SDP_INVALID_REQUEST_SYNTAX);
        return;
    }

    if (*cstateLenP == 0)
	{
		index = 0;
	}
    else if (*cstateLenP == 2)
    {
        index = NETCHAR2SHORT(buf); buf += 2;
    }
    else
    {
        sdpError(tc->chan, id, SDP_INVALID_REQUEST_SYNTAX);
        return;
    }

    if (buf != be)
    {
        sdpError(tc->chan, id, SDP_INVALID_REQUEST_SYNTAX);
        return;
    }

    SDP_TRACE_PRINTF_8(SDP_TRACE_MASK_TRACE, "SDP: SDP_SERVICE_ATTRIBUTE_REQUEST chan %X id %X handle %lX bytec %X cstateLen %X",
        				tc->chan, id, handle, maxByteCount, *cstateLenP, 0 ,0, 0);

    /* Limit transfer amount according to mtu size and local buffer size :
       2 bytes for size, 3 bytes for cstate */
    maxPayload = tc->mtuSize - (SDP_HEADER_LENGTH + 2 + 3);

    /* now limit it to the maxByteCount value requested by the client */
    if(maxPayload > maxByteCount)
    {
        maxPayload = maxByteCount;
    }

    /* check if handle is correct, allow for handle 0 */
    if (handle && (
        (handle <= SDP_SERVICE_OFFSET) ||
        (handle >= (SDP_SERVICE_OFFSET + BT_SDP_SERVER_MAX_SERVICES)) ||
        (pSDP->services[handle-SDP_SERVICE_OFFSET] == NULL)
        )
        )
    {
        sdpError(tc->chan, id, SDP_INVALID_SERVICE_RECORD_HANDLE);
        return;
    }

    /* create attribute list, do not delete empty DES sequences */
    lbp = sdpCreateAttributeList(pSDP->bufTmp, maxPayload, index, 0, 0xFFFF, attrList, handle, FALSE);

    if (lbp == 0xffff)   /* Error occured in sdpCreateAttributeList */
    {
        sdpError(tc->chan, id, SDP_INVALID_REQUEST_SYNTAX);
        return;
    }

    /* compute valid length this PDU */
    if (lbp <= index)     /* no data for pdu (should not happen) */
    {
        len      = 0;
        newIndex = 0;
    }
    else if (lbp > (index + maxPayload)) /* still data for next pdu */
    {
        len      = maxPayload;
        newIndex = index + maxPayload;
    }
    else /* this pdu partially filled */
    {
        len      = lbp - index;
        newIndex = 0;
    }
    sdpServiceAttributeResponse(tc->chan, id, SDP_SERVICE_ATTRIBUTE_RESPONSE, pSDP->bufTmp, len, newIndex);
} /* sdpHandleServiceAttributeRequest */

/**
* @brief  sdp handle service attribute response
* 
* @param  pSDP
* @param  tc
* @param  buf
* @param  id
* @param  blen
*
* @return  
*
*/
void sdpHandleServiceAttributeResponse(TsdpChanDesc * tc, uint8_t * buf, uint16_t id, uint16_t blen)
{
    uint16_t  byteCount;
    uint8_t * aList;
    uint8_t * cstateLenP;
    uint8_t * cstate;
    uint16_t  bp;            /* current position in incoming pdu */
    uint16_t  ea;            /* ending address */

    LPsdpAttributeReq  attributeReq  = (LPsdpAttributeReq)  (tc->cmdp + tc->offset);
    LPsdpAttributeConf attributeConf = (LPsdpAttributeConf) (tc->respp + pSDP->readOffset);

    bp         = 0;

    /* Get parameters off the PDU */
    byteCount  = NETCHAR2SHORT(buf + bp); bp += 2;
    aList      = buf+bp;                bp += byteCount;
    cstateLenP = buf+bp;                bp += 1;
    cstate     = buf+bp;

    ea         = tc->index + byteCount;

    /* test if the incoming pdu is syntactical correct ... */
    if (tc->id != id
        || tc->nextCode != SDP_SERVICE_ATTRIBUTE_RESPONSE
        || bp > blen
        || *cstateLenP > 16 || (bp + *cstateLenP) > blen
        )
    {
        SDP_TRACE_PRINTF_0(SDP_TRACE_MASK_ERROR, "!!sdpHandleServiceAttributeResponse: Syntax Error");
        sdpCloseLink(tc, SDP_ERR | SDP_INVALID_RESPONSE_SYNTAX, 0);
        return;
    }

    /* we check if the total answer will fit into our preallocated block */
#if (X_SDP_QUALIFICATION_SUPPORT == 0)
	if (ea > attributeReq->maxCount)
	{
	    SDP_TRACE_PRINTF_2(SDP_TRACE_MASK_ERROR, "!!sdpHandleServiceAttributeResponse: too much Data total %d allow %d", ea, attributeReq->maxCount);
	    sdpCloseLink(tc, SDP_ERR | SDP_INSUFFICIENT_RESOURCES, 0);
	    return;
	}
#endif

    /* Reassemble data into attributeConf and update the index */
    memcpy(attributeConf->aList + tc->index, aList, byteCount);
    tc->index = ea;

    if (tc->segmentationMode == 0 && *cstateLenP)
    {
        /* Continuation state exists and we are in normal reassembly mode: request more data */
        sdpServiceAttributeRequest(tc->chan, ++tc->id,
								attributeReq->handle,
								attributeReq->aList, attributeReq->aLen,
								(uint16_t)min(attributeReq->maxCount, tc->inMtuSize - SDP_RESPONSE_OVERHEAD),
								cstate, *cstateLenP
								);
    }
    else
    {
        tc->continuationLen = *cstateLenP;
        if (*cstateLenP)
        {
          /* continuation state exists, but we are not in reassembly mode: deliver data to user and remember
             the continuation state for the next operation, max length is already checked
          */
          memcpy(tc->continuationState, cstate, *cstateLenP);
        }

        attributeConf->byteCount = tc->index;
        sdpAttributeConf(tc);
        sdpResetTimer(2, tc);
    }

} /* sdpHandleServiceAttributeResponse */

/**
* @brief  sdp handle service search attribute request
* 
* @param  pSDP
* @param  tc
* @param  buf
* @param  id
* @param  blen
*
* @return  
*
*/
void sdpHandleServiceSearchAttributeRequest(CONST TsdpChanDesc * tc, uint8_t * buf, uint16_t id, uint16_t blen)
{
    uint16_t maxByteCount;  /* max size of outgoing data after reassembly */
    uint16_t maxPayload;    /* max size of payload before reassembly */
    uint16_t index = 0;     /* Incoming continuation state */
    uint16_t newIndex = 0;  /* Outgoing continuation state */
    uint16_t lbp = 0;       /* index into lbuf */
    uint8_t * uuidList;
    uint8_t * uuidEnd;
    uint8_t * attrList;
    uint8_t  cstateLen;
    uint8_t * pl;
    uint8_t * be = buf + blen;   /* end of buffer pointer */
    uint8_t typ;
    uint16_t len;
    uint32_t lhandle;
    uint8_t tbuf[3];

    /* Arguments are: ServiceSearchPattern (UUID List), maxByteCount, AttrIdList, cState */

    if (blen < 7)
    {
        sdpError(tc->chan, id, SDP_INVALID_REQUEST_SYNTAX);
        return;
    }

    pl = sdpDecodeElement(buf, buf+blen, &len, &typ);
    if (!pl || typ != SDP_TYP_SEQUENCE || len == 0)
    {
        sdpError(tc->chan, id, SDP_INVALID_REQUEST_SYNTAX);
        return;
    }
    uuidList     = buf;
    buf          = pl + len;
    uuidEnd      = buf;

    if ((buf + 5) > be) /* at least to follow: maxbytecnt (2), el seq. (2), cstate (1) */
    {
        sdpError(tc->chan, id, SDP_INVALID_REQUEST_SYNTAX);
        return;
    }
    maxByteCount = NETCHAR2SHORT(buf); buf += 2;

    pl = sdpDecodeElement(buf, buf+blen, &len, &typ);
    if (!pl || typ != SDP_TYP_SEQUENCE || len == 0 || maxByteCount < 7)
    {
        sdpError(tc->chan, id, SDP_INVALID_REQUEST_SYNTAX);
        return;
    }
    attrList  = buf;
    buf       = pl + len;
    if ((buf + 1) > be)
    {
        sdpError(tc->chan, id, SDP_INVALID_REQUEST_SYNTAX);
        return;
    }
    cstateLen = *buf++;

    SDP_TRACE_PRINTF_4(SDP_TRACE_MASK_TRACE, "SDP: SDP_SERVICE_SEARCH_ATTRIBUTE_REQUEST chan %X id %X bytec %X cstateLen %X",
        				tc->chan, id, maxByteCount, cstateLen  );

    if ((buf + cstateLen) != be)
    {
        sdpError(tc->chan, id, SDP_INVALID_REQUEST_SYNTAX);
        return;
    }

    if (cstateLen == 0)
	{
    	index = 0;
	}
    else if (cstateLen == 2)
    {
        index = NETCHAR2SHORT(buf); buf += 2;
    }
    else
    {
        sdpError(tc->chan, id, SDP_INVALID_REQUEST_SYNTAX);
        return;
    }

    /* Limit transfer amount to mtu size and local buffer size :
       2 bytes for size, 3 bytes for cstate */
    maxPayload = tc->mtuSize - (SDP_HEADER_LENGTH + 2 + 3);

    /* now limit it to the maxByteCount value requested by the client */
    if(maxPayload > maxByteCount)
    {
        maxPayload = maxByteCount;
    }

    /* create initial main level header (artificial length) */
    lbp = sdpDesHeader(pSDP->bufTmp,lbp,SDP_TYP_SEQUENCE_WORD,0);

    /* Search all handles for attribute list */
    for (lhandle=SDP_SERVICE_OFFSET; lhandle < SDP_SERVICE_OFFSET + BT_SDP_SERVER_MAX_SERVICES; lhandle++)
    {
        uint8_t res;

        if (pSDP->services[lhandle-SDP_SERVICE_OFFSET] == NULL)
            continue;

        res = sdpDbServiceSearchList(uuidList, uuidEnd, lhandle);

        if (res == SDP_NOT_FOUND)
    	{
        	continue;
    	}

        if (res!= SDP_SUCCESS)
        {
            sdpError(tc->chan, id, res);
            return;
        }

        /* create attribute list, do delete empty DES sequences */
        lbp = sdpCreateAttributeList(pSDP->bufTmp, maxPayload, index, lbp, 0xFFFF , attrList, lhandle, TRUE);

        if (lbp == 0xffff)   /* Error occured in sdpCreateAttributeList */
        {
            sdpError(tc->chan, id, SDP_INVALID_REQUEST_SYNTAX);
            return;
        }
    } /* for */

    /* insert header with correct length in front */
    (void)sdpDesHeader(tbuf, 0, SDP_TYP_SEQUENCE_WORD, (uint16_t)(lbp-3));
    sdpMemCpy(pSDP->bufTmp, 0, tbuf, 3, index, BT_SDP_SERVER_BUFFER_BYTE_COUNT);

    /* compute valid length this PDU */
    if (lbp <= index)     /* no data for pdu (should not happen) */
    {
        len      = 0;
        newIndex = 0;
    }
    else if (lbp > (index + maxPayload)) /* still data for next pdu */
    {
        len      = maxPayload;
        newIndex = index + maxPayload;
    }
    else /* this pdu partially filled */
    {
        len      = lbp - index;
        newIndex = 0;
    }

    /* now everything is prepared, send the response ! */
    sdpServiceAttributeResponse(tc->chan, id, SDP_SERVICE_SEARCH_ATTRIBUTE_RESPONSE, pSDP->bufTmp, len, newIndex);
} /* sdpHandleServiceSearchAttributeRequest */

/**
* @brief  sdp handle SDP_SERVICE_SEARCH_ATTRIBUTE_RESPONSEe
* 
* @param  pSDP
* @param  tc
* @param  buf
* @param  id
* @param  blen
*
* @return  
*
*/
void sdpHandleServiceSearchAttributeResponse(TsdpChanDesc * tc, uint8_t * buf, uint16_t id, uint16_t blen)
{
    uint16_t  byteCount;
    uint8_t * aList;
    uint8_t * cstateLenP;
    uint8_t * cstate;
    uint16_t  bp;            /* current position in incoming pdu */
    uint16_t  ea;            /* ending address */

    LPsdpSearchAttributeReq  sattributeReq  = (LPsdpSearchAttributeReq)  (tc->cmdp + tc->offset);
    LPsdpSearchAttributeConf sattributeConf = (LPsdpSearchAttributeConf) (tc->respp + pSDP->readOffset);

    bp = 0;

    /* Get parameters off the PDU */
    byteCount  = NETCHAR2SHORT(buf+bp); bp += 2;
    aList      = buf+bp;                bp += byteCount;
    cstateLenP = buf+bp;                bp += 1;
    cstate     = buf+bp;

    ea = tc->index + byteCount;

    /* test if the incoming pdu is syntactical correct ... */
    if (tc->id != id
        || tc->nextCode != SDP_SERVICE_SEARCH_ATTRIBUTE_RESPONSE
        || bp > blen
        || *cstateLenP > 16 || (bp + *cstateLenP) > blen
        )
    {
        SDP_TRACE_PRINTF_0(SDP_TRACE_MASK_ERROR, "!!sdpHandleServiceSearchAttributeResponse: Syntax Error");
        sdpCloseLink(tc, SDP_ERR | SDP_INVALID_RESPONSE_SYNTAX, 0);
        return;
    }

#if (X_SDP_QUALIFICATION_SUPPORT==0)
/* we check if the total answer will fit into our preallocated block */
    if (ea > sattributeReq->maxCount)
    {
        SDP_TRACE_PRINTF_2(SDP_TRACE_MASK_ERROR, "!!sdpHandleServiceSearchAttributeResponse: too much Data total %d allow %d",ea,sattributeReq->maxCount);
        sdpCloseLink(tc, SDP_ERR | SDP_INSUFFICIENT_RESOURCES, 0);
        return;
    }
#endif

    /* Reassemble data into attributeConf and update the index */
    memcpy(sattributeConf->aList+tc->index, aList, byteCount);
    tc->index = ea;

    if (*cstateLenP)
    {
        /* Continuation state exists: request more data */
        sdpServiceSearchAttributeRequest(tc->chan, ++tc->id,
		            sattributeReq->lists, sattributeReq->sLen,
		            (uint16_t)min(sattributeReq->maxCount, tc->inMtuSize - SDP_RESPONSE_OVERHEAD),   /* -5 for sdp header, -2 for AttrListByteCount, -17 for continuation state */
		            sattributeReq->lists + sattributeReq->sLen, sattributeReq->aLen,
		            cstate, *cstateLenP
		            );
    }
    else
    {
        sattributeConf->byteCount = tc->index;
        sattributeConf->linkid    = tc->self;
        if (tc->flag & DATA_CB_RELEASE)
    	{
        	osBufferRelease(tc->cmdp);
    	}

        tc->cmdp     = NULL;     
        sdpResetTimer(2, tc);
        tc->respp    = NULL;
        tc->nextCode = 0;
    }
} /* sdpHandleServiceAttributeResponse */

/**
* @brief  set sdp timer
* 
* @param  pSDP
* @param  timerID: timer ID
* @param  channel: timer channel
* @param  seconds: timeout
*
* @return  
*
*/
void sdpSetTimer(uint8_t timerID, TsdpChanDesc *cp, uint16_t seconds)
{
    osStartTimer(&(cp->TimerHandle), sdpQueueID, timerID, cp->self, seconds*1000, swTimerCallBack);
}

/**
* @brief  reset sdp timer
* 
* @param  pSDP
* @param  timerID: timer ID
* @param  channel: timer channel
*
* @return  
*
*/
void sdpResetTimer(uint8_t timerID, TsdpChanDesc *cp)
{
	osDeleteTimer(&(cp->TimerHandle));
}

/**
* @brief  sdp handle timer
* 
* @param  chan
*
* @return  
*
*/
void sdpHandleTimer(uint8_t chan)
{
    TsdpChanDesc * cp;
	
    /* Channel is index into Channel Descriptor Table */
    if((chan>=(otp_str_data.gEfuse_UpperStack_s.num_link_don+0x10))&&(chan>=otp_str_data.gEfuse_UpperStack_s.num_link_doff))
    {
        SDP_TRACE_PRINTF_1(SDP_TRACE_MASK_ERROR, "!!sdpHandleTimer: ill channel id %x", chan);
        return;
    }
#if 0		
    cp = &pSDP->channels[chan];
#else	
#if 0
    if(chan&0x10)
        cp = &pSDP->pchannelsDon[chan-0x10];
    else
        cp = &pSDP->pchannelsDoff[chan];
#else
	cp = &pSDP->pchannelsDon[chan];
#endif
#endif		
    if (!cp->used)
    {
        SDP_TRACE_PRINTF_1(SDP_TRACE_MASK_ERROR, "!!sdpHandleTimer: channel not in use %x", chan);
        return;
    }

    if (cp->incoming)
    {
        l2cHandleBTG_DISC_REQ(cp->chan, FALSE);
		/* not here: only at disc_conf         sdpCloseChannel(pSDP, cp); */
    }
    else
    {
        sdpCloseLink(cp, SDP_ERR | SDP_ERR_TIMEOUT, 0);
    }
}

/**
* @brief  Create list of requested attribute values (attrList) for handle into buf. buf represents
*  		only a fraction of a larger virtual buffer. The current
*   		logical position is bpos, buf describes buffer portion from index to index+bufsize-1,
*   		thus buf[0] is bufferposition[index] etc. return new logical position (or 0xffff on error)
*
* @param  pSDP: Instance data
* @param  buf: pointer into virtual buffer fragment
* @param  bufsize: size of virttual buffer fragment
* @param  index: startposition of virtual buuffer fragment
* @param  bpos: logical start position
* @param  maxByteCount: size of virtual buffer
* @param  attrList: pointer to DES of attribute values / attribute ranges
* @param  handle: handle to SDP database entry to refer to
* @param  deleteEmpty: flag to delete empty sequences (no empty DES generated
*
* @return  
*
*/
uint16_t sdpCreateAttributeList(             
						        uint8_t * buf,             
						        uint16_t bufsize,           
						        uint16_t index,             
						        uint16_t bpos,              
						        uint16_t maxByteCount,      
						        uint8_t * attrList,        
						        uint32_t handle,           
						        uint8_t deleteEmpty        
        						)
{
    uint16_t len;
    uint8_t typ;
    uint16_t lbp;
    uint8_t * element;
    uint8_t * epl;
    uint8_t * spl;
    uint8_t lbuf[3];

    /* save space for initial header (1 byte header + 2 byte length field) */
    lbp     = bpos+3;

    /* go into attribute list */
    element = sdpDecodeElement(attrList, NULL, &len, &typ);
    epl     = element + len;
    while (element < epl)
    {
        uint16_t attr_low;
        uint16_t attr_high;
        uint16_t attribute;

        spl = sdpDecodeElement(element, epl, &len, &typ);

        /* allowed types are: uint16 (attribute) or uint32 (attribute range) */
        if (!spl || typ != SDP_TYP_UINT || ((len != 2) && (len != 4)))
		{
			return 0xFFFF;
		}

        if (len==2)
        {   /* single attribute value */
            attr_low  = NETCHAR2SHORT(spl);
            attr_high = attr_low;   /* simulate single value range */
            SDP_TRACE_PRINTF_1(SDP_TRACE_MASK_TRACE, "SDP: found single attribute (handle %lX)", handle);
        } /* len == 2 */
        else
        {   /* attribute range */
            attr_low  = NETCHAR2SHORT(spl);
            attr_high = NETCHAR2SHORT(spl+2);
            SDP_TRACE_PRINTF_2(SDP_TRACE_MASK_TRACE, "SDP: found attrib range %X-%X", attr_low, attr_high);
            if (attr_high > SDP_LAST_ATTRIBUTE)   /* limit the range ... */
        	{
            	attr_high = SDP_LAST_ATTRIBUTE;
        	}
        } /* len == 4 */

        attribute = attr_low;
        while (attribute <= attr_high)
        {
            uint16_t tmp;
            uint16_t aLen;
            uint16_t nextAttribute;
            uint8_t * aVal = sdpDbSearchAttribute(handle, attribute, &aLen, &nextAttribute);

            if (aVal)   /* try to insert aval into list */
            {
                /* required space is: 3 bytes for attribute id (incl. header)
                   plus alen bytes for attribute value */
                if ((lbp + 3 + aLen) > maxByteCount)
            	{
                	break;              /* stop working */
            	}

                /* generate header and attrib value in local buffer */
                tmp = sdpDesHeader(lbuf,0,SDP_TYP_UINT,2);
                NETSHORT2CHAR(lbuf+tmp,attribute); tmp += 2;

                /* transfer into virtualized buffer */
                sdpMemCpy(buf, lbp, lbuf, tmp, index, bufsize);  lbp += tmp;

                /* transfer attribute value into virtualized buffer */
                sdpMemCpy(buf, lbp, aVal, aLen, index, bufsize); lbp += aLen;
            }

            /* switch to next attribute in sequence, if ==0, then the end is reached */
            if (!nextAttribute)
        	{
            	break;
        	}

            attribute = nextAttribute;
        } /* while */

        element = spl + len; /* skip to next element in this sequence */
    } /* while */

    /* insert header with correct length in front */
    len = (uint16_t)(lbp-(bpos+3));

    /* if nothing generated, do not even return a empty sequence (return nothing) -- if requested */
    if (!len && deleteEmpty)
	{
    	return bpos;
	}

    (void)sdpDesHeader(lbuf, 0, SDP_TYP_SEQUENCE_WORD, len);
    sdpMemCpy(buf, bpos, lbuf, 3, index, bufsize);

    return lbp;

} /* sdpCreateAttributeList */

/**
* @brief  copy some memory. (sbuf,scnt) describes source memory area. dix is the start pos
*   		in the destination buffer. dbuf describes part of destination buffer that starts at dindex and
*   		is dsize bytes long. all data in source range not fitting into destination buffer is cut off
*   		(not copied).
*
* @param  dbuf
* @param  dix
* @param  sbuf
* @param  scnt
* @param  dindex
* @param  dsize
*
* @return  
*
*/
void sdpMemCpy(uint8_t * dbuf, uint16_t dix, LPCBYTE sbuf, uint16_t scnt, uint16_t dindex, uint16_t dsize)
{
    uint16_t tmp;

    if ((dix+scnt) <= dindex)       /* everything is left of dbuf: noop */
	{
    	return;
	}

    if (dix >= (dindex + dsize))    /* everything to the right of dbuf: noop */
	{
    	return;
	}

    if (dix < dindex)               /* some left part to chop off */
    {
        tmp   = dindex - dix;       /* size to chop off */
        sbuf += tmp;                /* remove left part */
        scnt -= tmp;
        dix  += tmp;
    }

    if ((dix + scnt) > (dindex + dsize))         /* some right part to chop off */
    {
        tmp   = (dix + scnt) - (dindex + dsize); /* size to chop off */
        scnt -= tmp;
    }
    memcpy(dbuf+dix-dindex, sbuf, scnt);

} /* sdpMemCpy */

/**
* @brief  sdp search confirm
* 
* @param  tc
* @param  pSDP
*
* @return  
*
*/
void sdpSearchConf(TsdpChanDesc * tc)
{
    LPbtLink pLink;
    LPsdpSearchConf searchConf = (LPsdpSearchConf)(tc->respp + pSDP->readOffset);
    uint8_t handle    = (uint8_t)tc->self;
    uint32_t *handles = searchConf->handles;
    uint16_t count    = searchConf->totalHandles;

    /*release data first*/
    osBufferRelease(tc->respp);
    tc->respp    = NULL;
    tc->nextCode = 0;

    pLink = blueFaceFindLinkByHandle(handle, BLUEFACE_PSM_SDP);
    if (pLink==NULL)
    {
        SDP_TRACE_PRINTF_1(SDP_TRACE_MASK_ERROR, "!!SDP_SEARCH_CONF: cannot find handle %X", handle);
    }
    else
    {
        /* send message via blueFace to application */
        blueFaceSendBT_SDP_SEARCH_CONF(pLink->context, handles, count, pSDP->readOffset);
    }
}

/**
* @brief   Close the Link and generate DiscInd to upper layer 
* 
* @param  pSDP
* @param  tc: 
* @param  status: 
* @param  holdLink: if hold acl link
*
*/
void sdpCloseLink(TsdpChanDesc * tc, uint16_t status, uint16_t holdLink)
{
    if (status==0 || tc->respp == NULL || tc->nextCode == 0) 
    {
        l2cHandleBTG_DISC_REQ(tc->chan, holdLink);
    }
    else
    {
        switch (tc->nextCode)
        {
        case SDP_SERVICE_SEARCH_ATTRIBUTE_RESPONSE:
			/*nothing todo!*/
            break;
        case SDP_SERVICE_ATTRIBUTE_RESPONSE:
        {
            LPsdpAttributeConf attributeConf = (LPsdpAttributeConf)(tc->respp + pSDP->readOffset);
            attributeConf->byteCount   = 0;
            sdpAttributeConf(tc);
        }
            break;
        case SDP_SERVICE_SEARCH_RESPONSE:
        {
            LPsdpSearchConf searchConf = (LPsdpSearchConf)(tc->respp + pSDP->readOffset);
            searchConf->linkid         = tc->self;
            searchConf->totalHandles   = 0;
            sdpSearchConf(tc);
        }
            break;
        default:
            assert(FALSE);
			break;
        } 
        sdpResetTimer(2, tc);
    } /* else */
    sdpCleanClient(tc);
} /* sdpCloseLink */

/**
* @brief   sdp clean client point
* 
* @param  pSDP
* @param  tc: 
*
*/
void sdpCleanClient(TsdpChanDesc * tc)
{
    if (tc->cmdp)
    {
        osBufferRelease(tc->cmdp);
        tc->cmdp = NULL;
    }
    if (tc->respp)
    {
        osBufferRelease(tc->respp);
        tc->respp = NULL;
    }
}

/**
* @brief   sdp handle config device name
* 
* @param  pName
* @param  len
*
* @return  
*/
void sdpHandleCONFIG_GAP_DEVICE_NAME(uint8_t * pName, uint16_t len)
{
	/* received a copy of the message that sets a new friendly name on hci layer.
	We copy the contents of the message to local instance storage,
	for EIR (extended inquiry response) handling.
	The copy is space-limited, in order to save RAM memory.
	*/
	uint16_t   length = len;

	if (length > sizeof(pSDP->shortName) - 1)
	{
		length = sizeof(pSDP->shortName) - 1;
	}

	memcpy(pSDP->shortName, pName, length);
	pSDP->shortName[length] = 0;
	sdpNewEIR();
}

/**
 * @brief   sdp handle config device ID in EIR data
 *
 * @param  VendorID
 * @param  ProductID
 * @param  ProductID
 * @param  ProductID
 *
 * @return
 */
void sdpHandleConfigDID(uint16_t VendorID, uint16_t ProductID, uint16_t ProductVersion, uint16_t IDSource)
{
    pSDP->VendorID = VendorID;
    pSDP->ProductID = ProductID;
    pSDP->ProductVersion = ProductVersion;
    pSDP->IDSource = IDSource;
    sdpNewEIR();
}

/**
 * @brief   sdp handle config extra EIR data
 *
 * @param  buf
 *
 * @return
 */
void sdpHandleExtraEIRdata(char *pdata)
{
    pSDP->extraEIRData = pdata;
    SDP_TRACE_PRINTF_1(SDP_TRACE_MASK_ERROR,"!!SDP: extraEIRData is %p", pSDP->extraEIRData);
    sdpNewEIR();
}

/**
* @brief  send sdp connect req
*
* @param  bd
* @param  mtuSize
*
* @return  
*
*/
void sdpHandleUpConReq(LPCBYTE bd, uint16_t mtuSize)
{
	TsdpChanDesc   * tc;
	LP_L2C_Conf_para pConfPara;

	/* check if there is already a SDP channel to exactly this device: avoid double client SDP sessions */
	tc = sdpGetChannelByBd(bd);
	if (tc)
	{
		SDP_TRACE_PRINTF_1(SDP_TRACE_MASK_ERROR,"!!SDP: SDP_CON_REQ exists bd %s", TRACE_BDADDR1(SDP_TRACE_MASK_ERROR, bd));
		blueAPI_Handle_SDP_CON_CONF(SDP_ERR | SDP_INSUFFICIENT_RESOURCES, 0);
		return;
	} /* bd already exists */

	/* try to allocate a channel for this outgoing connection (might fail!)*/
	tc = sdpAllocateChannel(0 /* no channel id available */, FALSE);
	if (!tc)
	{
		SDP_TRACE_PRINTF_1(SDP_TRACE_MASK_ERROR, "!!SDP: SDP_CON_REQ busy bd %s", TRACE_BDADDR1(SDP_TRACE_MASK_ERROR, bd));
		blueAPI_Handle_SDP_CON_CONF(SDP_ERR | SDP_INSUFFICIENT_RESOURCES, 0);
		return;
	} /* channel allocation failed */

	SDP_TRACE_PRINTF_4(SDP_TRACE_MASK_TRACE,"SDP: SDP_CON_REQ desc %X bd %s mtu %X ", tc, TRACE_BDADDR1(SDP_TRACE_MASK_TRACE, bd), mtuSize, 0);

	memcpy(tc->bd, bd, BD_ADDR_SIZE);

	tc->mtuSize = BT_US_PDU_L2C_BYTE_COUNT;
	pConfPara = commonPrepareL2cConfPara(tc->mtuSize, tc->mtuSize, 0);
	if(pConfPara == 0)
	{
		SDP_TRACE_PRINTF_1(SDP_TRACE_MASK_ERROR, "!!SDP: SDP_CON_REQ out of memory %s", TRACE_BDADDR1(SDP_TRACE_MASK_ERROR, bd));
		blueAPI_Handle_SDP_CON_CONF(SDP_ERR | SDP_INSUFFICIENT_RESOURCES, 0);
		DebuggerBreak();
		return;
	}

    l2cUSendConReq(sdpQueueID, tc->bd, PSM_SDP, UUID_SDP /* UUID */, 0 /* framesize */, pConfPara);
}

/**
* @brief   sdp handle disconnect req
*
* @param  cid
* @param  holdLink
*
* @return  
*
*/
void sdpHandleUpDiscReq(uint16_t cid, uint16_t holdLink)
{
#if 0
	TsdpChanDesc *tc = &pSDP->channels[cid];
#else
    TsdpChanDesc *tc;
    if((cid < otp_str_data.gEfuse_UpperStack_s.num_link_don) && (pSDP->pchannelsDon[cid].used))
    {
       tc = &pSDP->pchannelsDon[cid];
    }
    else if((cid < otp_str_data.gEfuse_UpperStack_s.num_link_doff) && (pSDP->pchannelsDoff[cid].used))
    {
    	tc = &pSDP->pchannelsDoff[cid];
    }
    else
    {
        return;
    }
#endif
	SDP_TRACE_PRINTF_2(SDP_TRACE_MASK_TRACE, "SDP_DISC_REQ linkid %d hold %d", cid, holdLink);
	sdpResetTimer(2, tc);
	sdpCloseLink(tc, 0, holdLink); 
}

/**
* @brief   sdp handle search req
*
* @param  linkid
* @param  maxHandles
* @param  uuidLen
* @param  uuidList
*
* @return  
*
*/
uint16_t sdpHandleUpSearchReq(uint16_t linkid, uint16_t maxHandles, uint16_t uuidLen, LPCBYTE uuidList)
{
	TsdpChanDesc *tc ;
	uint16_t len;
	LPsdpSearchReq searchReq;
	uint16_t mySize = (uint16_t)(offsetof(TsdpSearchReq, uuidList) + uuidLen);

	if((linkid < otp_str_data.gEfuse_UpperStack_s.num_link_don) && (pSDP->pchannelsDon[linkid].used))
	{
		tc = &pSDP->pchannelsDon[linkid];
	}
	else if((linkid<otp_str_data.gEfuse_UpperStack_s.num_link_doff )&&(pSDP->pchannelsDoff[linkid].used))
	{
		tc = &pSDP->pchannelsDoff[linkid];
	}
	else
	{
		return 0;
	}

	if (osBufferGet(BTSystemPoolID, (offsetof(TsdpSearchReq, uuidList) + uuidLen), (PVOID) &searchReq))
	{
		SDP_TRACE_PRINTF_1(SDP_TRACE_MASK_ERROR, "!!sdpHandleUpSearchReq: no memory, len %d", mySize);
		assert(FALSE);
		return BLUEFACE_ERR | BF_NO_RESOURCES;
	}

	searchReq->linkid		= linkid;
	searchReq->maxHandles	= maxHandles;
	searchReq->uuidLen		= uuidLen;
	memcpy(searchReq->uuidList, uuidList, uuidLen);

	SDP_TRACE_PRINTF_4(SDP_TRACE_MASK_TRACE, "SDP: SDP_SEARCH_REQ linkid %d uuidlist %X maxHandles %X",
						linkid, uuidList, maxHandles, 0);

	tc->cmdp		= (uint8_t *)searchReq;
	tc->offset		= 0;
	tc->flag		= DATA_CB_RELEASE;
	tc->index		= 0; /* Starting index */
	tc->nextCode	= SDP_SERVICE_SEARCH_RESPONSE;

	len = (uint16_t)(pSDP->readOffset + sizeof(TsdpSearchConf) + maxHandles*4 /* 4 bytes per handle (not:sizeof(uint32_t)) ! */);
	if (osBufferGet(UpstreamPoolID, len, (void  *)&tc->respp))
	{
		SDP_TRACE_PRINTF_1(SDP_TRACE_MASK_ERROR,"!!sdpHandleUpSearchReq: NO MEMORY, len %d", len);
		sdpCloseLink(tc, SDP_ERR | SDP_INSUFFICIENT_RESOURCES, 0);
		DebuggerBreak();
		return 0;
	}

	sdpServiceSearchRequest(tc->chan, ++tc->id,
							uuidList, uuidLen,
							maxHandles,
							NULL, 0
							);
	sdpSetTimer(2, tc, SDP_TIMEOUT_CLIENT);

	return BLUEFACE_ERR | BF_SUCCESS;
}

/**
* @brief  sdp handle attribute request
* 
* @param  linkid
* @param  handle
* @param  byteCnt
* @param  aLen
* @param  aList
*
* @return  
*
*/
uint16_t sdpHandleUpAttributeReq(uint16_t linkid, uint32_t handle, uint16_t byteCnt, uint16_t aLen, LPCBYTE aList)
{
	TsdpChanDesc *tc;

#if BT_SDP_CLIENT_BUFFER_COUNT
	uint16_t upperLimit = (uint16_t)(BT_SDP_CLIENT_BUFFER_BYTE_COUNT - sizeof(TsdpAttributeConf));
#else
	uint16_t upperLimit = (uint16_t)(BT_US_PDU_L2C_BYTE_COUNT - sizeof(TsdpAttributeConf));
#endif
	uint16_t len;

	LPsdpAttributeReq attributeReq;
	uint16_t mySize = (uint16_t)(offsetof(TsdpAttributeReq, aList) + aLen);

	if((linkid < otp_str_data.gEfuse_UpperStack_s.num_link_don) && (pSDP->pchannelsDon[linkid].used))
	{
		tc = &pSDP->pchannelsDon[linkid];
	}
	else if((linkid<otp_str_data.gEfuse_UpperStack_s.num_link_doff )&&(pSDP->pchannelsDoff[linkid].used))
	{
		tc = &pSDP->pchannelsDoff[linkid];
	}
	else
	{
		return 0;   
	}

	if (osBufferGet(BTSystemPoolID, mySize, (PVOID) &attributeReq))
	{
		SDP_TRACE_PRINTF_1(SDP_TRACE_MASK_ERROR, "!!sdpHandleUpAttributeReq: no memory, len %d", mySize);
		assert(FALSE);
		/*lint -e{527} the return may be reachable */
		return BLUEFACE_ERR | BF_NO_RESOURCES;
	}

	attributeReq->linkid	= linkid;
	attributeReq->handle	= handle;
	attributeReq->maxCount	= byteCnt;
	attributeReq->aLen		= aLen;
	memcpy(attributeReq->aList, aList, aLen);

	SDP_TRACE_PRINTF_4(SDP_TRACE_MASK_TRACE,
						"SDP: SDP_ATTRIBUTE_REQ linkid %d handle %lX alist %X bytecnt %X",
						linkid, handle, aList, byteCnt);

	tc->cmdp		= (uint8_t *)attributeReq;
	tc->offset		= 0;
	tc->flag		= DATA_CB_RELEASE;
	tc->index		= 0; /* Starting index */
	tc->nextCode	= SDP_SERVICE_ATTRIBUTE_RESPONSE;

	if (attributeReq->maxCount == 0xffff || attributeReq->maxCount == 0xfffe) /* unsegmented operation, first segment oder follow segment */
	{
		SDP_TRACE_PRINTF_2(SDP_TRACE_MASK_TRACE,"SDP: SDP_ATTRIBUTE_REQ, segmentation mode 0x%04x maxCount %d", byteCnt, upperLimit);
		if (attributeReq->maxCount == 0xffff) /* first segment: start with empty continuation state */
		{
			tc->continuationLen  = 0;
			tc->segmentationMode = 1;
		}
		else
		{
			tc->segmentationMode = 2;
		}
		attributeReq->maxCount = upperLimit;
	}
	else
	{
		tc->segmentationMode = 0;
		tc->continuationLen  = 0;
	}
	if (attributeReq->maxCount == 0 || attributeReq->maxCount > upperLimit)
	{
		attributeReq->maxCount = upperLimit;
		SDP_TRACE_PRINTF_1(SDP_TRACE_MASK_TRACE,"SDP: SDP_ATTRIBUTE_REQ, using default bytecnt of %d", byteCnt);
	}

	len = (uint16_t)(pSDP->readOffset + sizeof(TsdpAttributeConf) + attributeReq->maxCount);
	if (osBufferGet(pSDP->usSDPAttrDataPool, len, (void  *)&tc->respp))
	{
		SDP_TRACE_PRINTF_1(SDP_TRACE_MASK_ERROR,"!!sdpHandleUpAttributeReq: NO MEMORY, len %d",len);
		sdpCloseLink(tc, SDP_ERR | SDP_INSUFFICIENT_RESOURCES, 0);
		DebuggerBreak();
		return 0;
	}

	/* if we are not in reassembly mode and the continuation state is already empty, then we have
	already reached the end of the SDP answer from the remote side: directly signal an empty packet to UL */
	if ((tc->segmentationMode == 2) && (tc->continuationLen == 0))
	{
		LPsdpAttributeConf attributeConf = (LPsdpAttributeConf) (tc->respp + pSDP->readOffset);
		attributeConf->byteCount = tc->index; /* always zero: empty answer */
		attributeConf->linkid	 = tc->self;

		sdpAttributeConf(tc);
	}
	else
	{
		sdpServiceAttributeRequest(tc->chan, ++tc->id,
									handle,
									aList, aLen,
									(uint16_t)min(attributeReq->maxCount, tc->inMtuSize - SDP_RESPONSE_OVERHEAD),
									tc->continuationState, tc->continuationLen
									);
		sdpSetTimer(2, tc, SDP_TIMEOUT_CLIENT);
	}

	return BLUEFACE_ERR | BF_SUCCESS;
}

/**
* @brief  sdp handle connect confirm
* 
* @param  remote_bd
* @param  cid
* @param  status
*
* @return  
*
*/
void sdpHandleL2cConConf(TBdAddr remote_bd, uint16_t cid, uint16_t status)
{
	TsdpChanDesc * tc;
	SDP_TRACE_PRINTF_4(SDP_TRACE_MASK_TRACE, "SDP: L2C_CON_CONF, chan 0x%X result 0x%X bd %s",
						cid, status, TRACE_BDADDR1(SDP_TRACE_MASK_TRACE, remote_bd),0);

	tc = sdpGetChannelByBd(remote_bd);
	if (!tc)
	{
		SDP_TRACE_PRINTF_1(SDP_TRACE_MASK_ERROR, "!!sdpHandleL2cConConf: bd %s not found",
							TRACE_BDADDR1(SDP_TRACE_MASK_ERROR, remote_bd));
		return;
	}

	/* check for connection pending message status */
	if (status == (L2CAP_ERR | 0x0001))
	{
		SDP_TRACE_PRINTF_1(SDP_TRACE_MASK_TRACE, "sdpHandleL2cConConf: cid %x: connection pending", cid);
		return;
	}

	tc->chan = cid; /*l2cap channel id*/
	blueAPI_Handle_SDP_CON_CONF(status, tc->self);
	if (status)
	{
		sdpCleanClient(tc);
		tc->used = FALSE;
	}
}

/**
* @brief  sdp handle connect act indication
* 
* @param  cid
* @param  status
* @param  mtuSize
*
* @return  
*
*/
void sdpHandleL2cConActInd(uint16_t cid, uint16_t status, uint16_t mtuSize)
{
	TsdpChanDesc * tc;
	SDP_TRACE_PRINTF_2(SDP_TRACE_MASK_TRACE,"SDP: L2C_CON_ACT_IND, chan 0x%X status 0x%X", cid, status);

	if (status == (L2CAP_ERR | L2CAP_ERR_PENDING))
	{
		SDP_TRACE_PRINTF_1(SDP_TRACE_MASK_TRACE, "sdpHandleL2cConActInd : cid 0x%x: connection pending", cid);
		return;
	}

	tc = sdpGetChannel(cid);
	if (!tc)
	{
		return;
	}

	tc->inMtuSize = mtuSize;

	SDP_TRACE_PRINTF_1(SDP_TRACE_MASK_TRACE, "SDP: L2C_CON_ACT_IND chan 0x%X: READY", tc->chan);

	if (!tc->incoming)
	{
		blueAPI_Handle_SDP_CON_ACT_IND(tc->inMtuSize, tc->self);
	}
	else if (tc->incoming)
	{
		sdpSetTimer(1, tc, SDP_TIMEOUT_SERVER);
	}
}

/**
* @brief  sdp handle disconnect indicate
* 
* @param  localCID
* @param  status
*
* @return  
*
*/
void sdpHandleL2cDiscInd(uint16_t localCID, uint16_t status)
{
	TsdpChanDesc * tc;

	tc = sdpGetChannel(localCID);
	SDP_TRACE_PRINTF_4(SDP_TRACE_MASK_TRACE,"SDP:L2C_DISC_IND chan %X desc %X status %X", localCID, tc, status, 0);

	l2cHandleL2C_DISC_RESP(localCID);
	if (!tc)
	{
		return;
	}
	if (!tc->incoming)
	{
		blueAPI_Handle_SDP_DISC_IND(status, tc->self);
		sdpCleanClient(tc);
		tc->used = FALSE;
	}
	else
	{
		sdpResetTimer(1, tc);
		tc->used = FALSE;
	}
}

/**
* @brief  sdp handle disconnect confirm
* 
* @param  localCID
* @param  result
*
* @return  
*
*/
void sdpHandleL2cDiscConf(uint16_t LocalCid, uint16_t result)
{
	TsdpChanDesc * tc = sdpGetChannel(LocalCid);
	SDP_TRACE_PRINTF_4(SDP_TRACE_MASK_TRACE,"SDP: L2C_DISC_CONF chan %x status %x tc %lx", LocalCid, result, tc, 0);
	
	if (!tc)
	{
		SDP_TRACE_PRINTF_1(SDP_TRACE_MASK_ERROR,"!!SDP: L2C_DISC_CONF chan %x not found", LocalCid);
		return;
	}
	if (!tc->incoming)
	{
		blueAPI_Handle_SDP_DISC_IND(result, tc->self);
		sdpCleanClient(tc);
		tc->used = FALSE;
	}
	else
	{
		tc->used = FALSE;
	}
}

/**
* @brief  sdp handle connect indicate
* 
* @param  localCID
*
* @return  
*
*/
void sdpHandleL2cConInd(uint16_t localCID)
{
	TsdpChanDesc * chanDesc = NULL;
	uint16_t acceptCall = L2CAP_ERR_REFUS_NO_RESOURCE;
	TBtConRespPSpecifc ConRespExt;
	LP_L2C_Conf_para	pConfPara;

	SDP_TRACE_PRINTF_1(SDP_TRACE_MASK_TRACE, "SDP: L2C_CON_IND, chan %X", localCID);

	/* try to allocate a channel for this incoming connection (might fail!)*/
	chanDesc = sdpAllocateChannel(localCID, TRUE);

	/* accept call only if channel was allocated */
	if (chanDesc)
	{
		acceptCall = L2CAP_CONNECTION_ACCEPT;
		chanDesc->mtuSize = BT_SDP_SERVER_BUFFER_BYTE_COUNT;
	}

	pConfPara = commonPrepareL2cConfPara(BT_US_PDU_L2C_BYTE_COUNT, BT_US_PDU_L2C_BYTE_COUNT, 0);
	if(pConfPara == 0)
	{
		SDP_TRACE_PRINTF_0(SDP_TRACE_MASK_ERROR, "!!SDP: sdpHandleL2cConInd out of memory");
		DebuggerBreak();
		return;
	}
	ConRespExt.l2cap.statusDetails = 0;
	ConRespExt.l2cap.pConfPara	= pConfPara;

	l2cHandleL2C_CON_RESP(localCID, acceptCall, &ConRespExt);
}

/**
* @brief  SDP handle confirm req(register)
*
* @param  BufferAddress: msg addr
* @param  offset
* @param  length
*
* @return
*
*/
BOOL sdpHandleUpRegReq(void *buf, uint16_t offset)
{
    uint16_t i;

	/* the database might change */
	pSDP->dbState++;

    for (i = 0; i < BT_SDP_SERVER_MAX_SERVICES; i++)
    {
        if (pSDP->services[i] == NULL)
        {
            pSDP->services[i] = buf;
            pSDP->offsets [i] = offset;
            sdpNewEIR(); /* update EIR due to change in services */
            break;
        } 
    }

    if (i == BT_SDP_SERVER_MAX_SERVICES)
    {
        return FALSE;
    }
    else
    {
        return TRUE;
    }
}
