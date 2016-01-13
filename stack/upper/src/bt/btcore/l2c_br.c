/**
*****************************************************************
*	Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************
* @file       l2c_br.c
* @brief     Bluetooth L2CAP Layer (Low Energy)
* @details   
*
* @author   	gordon
* @date      	2015-07-09
* @version	v0.1
*/

#include <flags.h>
#include <os_pool.h>
#include <os_timer.h>
#include <l2c.h>
#include <l2c_br.h>
#include <l2c_api.h>
#include <l2cchman.h>
#include <hci_api.h>
#include <sdp_api.h>
#include <gatt_api.h>
#include <blueapi_api.h>
#include <hci_code.h>
#include <upper_stack_global.h>
#include <swtimer.h>
#include <btglib.h>

#define TRACE_MODULE_ID     MID_BT_L2C

void l2cHandleL2CAP_CONFIGURE_REQUESTinStateConfigOrOpen(P_L2CAP_CHANNEL pChan);
void l2cSendL2C_CON_ACT_IND(PC_L2CAP_CHANNEL pChan, uint16_t usQueueID, uint16_t cid, uint16_t status);

/**
* @brief	 start config timeout
*
* @param	id
* @param     ticks
*
* @return
*
*/
void l2cStartCONFIGTimeout(P_L2CAP_CHANNEL pChan, uint16_t secs)
{
    if (pChan->CONFIGTimerHandle)
    {
        osDeleteTimer(&(pChan->CONFIGTimerHandle));
    }
    osStartTimer(&(pChan->CONFIGTimerHandle), l2cQueueID, L2CAP_CONFIG_TID, pChan->LocalCid, secs*1000, swTimerCallBack);
}

/**
* @brief	l2cap stop config timeout
*
* @param	id
*
* @return
*
*/
void l2cStopCONFIGTimeout(P_L2CAP_CHANNEL pChan)
{
	osDeleteTimer(&(pChan->CONFIGTimerHandle));
}


/****************************************************************************/
#if CHECK_API_PARAM
/**
* @brief  check if confirm param legal
*
* @param  pConfPara:
*
* @return  
*
*/
BOOL l2cCheckConfPara(LP_L2C_Conf_para pConfPara)
{
	if (pConfPara->flowControl.mode == 0)
	{
	    assert(FALSE);
	    return FALSE;
	}

	if (pConfPara->validMask.FlushTO &&
	    pConfPara->FlushTO != FLUSHTIMEOUT_DEFAULT && pConfPara->FlushTO > 1279 /* = 0x7FF * 0.625 ms */)
	{
    	pConfPara->FlushTO = 1279;                  /* max. value */
	}

	if (pConfPara->validMask.FlushTO == 0 ||
	    (pConfPara->validMask.FlushTO && pConfPara->FlushTO == 0))
	    pConfPara->FlushTO = FLUSHTIMEOUT_DEFAULT;

	if (pConfPara->flowControl.mode == L2C_MODE_BASIC)  /* basic mode only */
	{
	    if (pConfPara->mtuSize == 0)
	    {
	        pConfPara->mtuSize = BT_US_PDU_L2C_BYTE_COUNT;
	    }
	    if (pConfPara->mtuSize < L2CAP_SIGNAL_MTU)   /* MTU size shall larger than minimum */
	    {
	        assert(FALSE);
	        return FALSE;
	    }

	    pConfPara->flowControl.maxPDUSize = pConfPara->mtuSize;
	}
	return(TRUE);
}
#endif
/**
* @brief  get lecap mode
* 
* @param  Mode
*
* @return  
*
*/
uint8_t l2cGetL2capMode(uint16_t Mode)
{
#if F_BT_L2C_ENHANCED_FEATURE_SUPPORT

    uint8_t L2capMode;

    switch (Mode)
    {
        case L2C_MODE_BASIC:
            L2capMode = L2CAP_MODE_BASIC;
            break;
#if F_BT_L2C_ENHANCED_FEATURE_SUPPORT			
        case L2C_MODE_ENHANCED_RETRANSMISSION:
            L2capMode = L2CAP_MODE_ENHANCED_RETRANSMISSION;
            break;
        case L2C_MODE_STREAMING:
            L2capMode = L2CAP_MODE_STREAMING;
            break;
#endif			
        default:
            L2capMode = 0xFF;
            break;
    }

    return(L2capMode);
#else
	if(Mode == L2C_MODE_BASIC)
	{
		return L2CAP_MODE_BASIC;
	}else
	{
		return 0xFF;
	}
#endif
}
	    
/**
* @brief  get next mode
* 
* @param  pChan
* @param  ModeBits
* @param  ModeAct
*
* @return  
*
*/
uint16_t l2cGetNextMode(P_L2CAP_CHANNEL pChan, uint16_t ModeBits, uint16_t ModeAct)
{
#if F_BT_L2C_ENHANCED_FEATURE_SUPPORT
    int  L2capMode;
    uint16_t Mode;

    if (ModeAct == 0)        /* first time */
    {
        L2capMode = (sizeof(ModeBits) * 8) - 1;
    }
    else
        L2capMode = l2cGetL2capMode(ModeAct) - 1;

    for (; L2capMode >= 0 ; L2capMode--)
    {
        if (ModeBits & (1 << L2capMode))
        {
          if (L2capMode == L2CAP_MODE_BASIC)
            break;
#if F_BT_L2C_ENHANCED_FEATURE_SUPPORT

          if (L2capMode == L2CAP_MODE_ENHANCED_RETRANSMISSION &&
              pChan->pHciDesc->remoteExtendedFeatures & L2CAP_EX_FEATURE_ENHANCED_RETRANS)
            break;

          if (L2capMode == L2CAP_MODE_STREAMING &&
              pChan->pHciDesc->remoteExtendedFeatures & L2CAP_EX_FEATURE_STREAMING)
            break;
#endif
        }
    }

    if (L2capMode < 0)
    {
        Mode = 0;
    }
    else
    {
        Mode = (uint16_t)(1 << L2capMode);
    }
    return (Mode);
#else
    return L2C_MODE_BASIC;
#endif

}

/**
* @brief  get first mode for negotiation
* 
* @param  pChan
*
* @return  
*
*/
void l2cGetNextCurrentMode(P_L2CAP_CHANNEL pChan)
{
#if F_BT_L2C_ENHANCED_FEATURE_SUPPORT
    pChan->CurrentMode = l2cGetNextMode(pChan, pChan->ConfParaLocal.flowControl.mode, pChan->CurrentMode);
    if (pChan->CurrentMode != 0)
    {
        pChan->RespMode = pChan->CurrentMode;
        if (pChan->CurrentMode == L2C_MODE_BASIC)
    	{
        	pChan->ConfParaLocal.mtuSize = pChan->ConfParaLocal.flowControl.maxPDUSize;
    	}
    }
#else
	pChan->RespMode = pChan->CurrentMode = L2C_MODE_BASIC;
	pChan->ConfParaLocal.mtuSize = pChan->ConfParaLocal.flowControl.maxPDUSize;
#endif
}

/**
* @brief  check l2cap mode
* 
* @param  pChan
*
* @return  
*
*/
void l2cCheckMode(P_L2CAP_CHANNEL pChan)
{
	pChan->L2capInformationRequestSent = FALSE;
	if (pChan->SendL2capConfigureRequest == TRUE)
	{
		pChan->SendL2capConfigureRequest = FALSE;
		l2cGetNextCurrentMode(pChan);            
		if (pChan->CurrentMode != 0)
		{
		    if (pChan->SendL2capConfigureResponse == TRUE)
		    {
		        pChan->SendL2capConfigureResponse = FALSE;
		        l2cHandleL2CAP_CONFIGURE_REQUESTinStateConfigOrOpen(pChan);
		    }
		    l2cSendL2CAP_CONFIGURE_REQUEST(pChan);
		    l2cChangeState(pChan, l2cStateConfig);
		}
#if F_BT_L2C_ENHANCED_FEATURE_SUPPORT		
		else  /* mode is not allowed */
		{
		    l2cSendL2CAP_DISCONNECTION_REQUEST(pChan);
		    l2cChangeState(pChan, l2cStateWaitForL2CAP_DISCONNECTION_RESPONSE);
		}
#endif		
	}
}

/**
* @brief  l2cap send HCI_WRITE_AUTOMATIC_FLUSH_TIMEOUT command
* 
* @param  pHciDesc
*
* @return  
*
*/
void l2cSendLHciWriteAutomaticFlushTimeout(P_ACLPOOLDESC pHciDesc)
{
    uint16_t timeout;

    switch (pHciDesc->FlushTO)
    {
		case 0xFFFF:
			timeout = 0;               /* no automatic flush */
			break;
		case 1:
			timeout = 1;               /* no retransmission */
			break;
		default:
			timeout = (uint16_t)(((pHciDesc->FlushTO * 8) / 5) & 0x7FF);
			break;
    }
    L2CAP_TRACE_PRINTF_2(L2CAP_TRACE_MASK_TRACE,
                                  "L2C<- HCI_CONF_AUTOMATIC_FLUSH_TIMEOUT %d (%d)",
                                  pHciDesc->FlushTO, timeout);

	hciCommandWordWordParameter(HCI_WRITE_AUTOMATIC_FLUSH_TIMEOUT,
								pHciDesc->handle,
								timeout);
	hciLaterEntry();

} /* l2cSendLHciWriteAutomaticFlushTimeout */
/**
* @brief  l2cap send configure request
* 
* @param  pChan
*
* @return  
*
*/
void l2cSendL2CAP_CONFIGURE_REQUEST(P_L2CAP_CHANNEL pChan)
{
	P_L2C_Conf_para pConfPara = &pChan->ConfParaLocal;
	PBYTE            mbuf = pL2c->mbuf;
	PBYTE            pBuf = mbuf;
	int16_t            flags = 0;              /* flags field: C bit clear */	
	
#if F_BT_L2C_ENHANCED_FEATURE_SUPPORT
	uint8_t             L2capMode;

	L2capMode = l2cGetL2capMode(pChan->CurrentMode);
#endif	
	SHORT2CHAR(pBuf, pChan->RemoteCid);
	pBuf += 2;

#if (BT_L2C_MAX_CMD_RETRIES_COUNT == 0)
	pChan->lastConfigFlagSend = flags;
#endif  /* (BT_L2C_MAX_CMD_RETRIES_COUNT == 0) */
	SHORT2CHAR(pBuf, flags);
	pBuf += 2;

	L2CAP_TRACE_PRINTF_2(L2CAP_TRACE_MASK_TRACE,
	                              "L2C<- L2CAP CONFIG REQ dcid 0x%X flags 0x%X",
	                              pChan->RemoteCid, flags);

	pChan->LocalMtu = pConfPara->mtuSize;
#if F_BT_L2C_ENHANCED_FEATURE_SUPPORT		
	if (L2capMode == L2CAP_MODE_BASIC)
#endif		
	{
		if (pChan->LocalMtu > BT_US_PDU_L2C_BYTE_COUNT)
		{
			pChan->LocalMtu = BT_US_PDU_L2C_BYTE_COUNT;
		}
	}
	pChan->LocalUsMtu = pChan->LocalMtu;   /* initial equals to requested mtu */

	if (pChan->LocalMtu != MTU_DEFAULT)
	{                                           /* only if its not the default MTU */
		*pBuf++ = L2CAP_CONFIG_PARA_MTU;
		*pBuf++ = 2;
		SHORT2CHAR(pBuf, pChan->LocalMtu);
		pBuf += 2;
	}
#if F_BT_L2C_ENHANCED_FEATURE_SUPPORT	//champion
	{
	    if (L2capMode == L2CAP_MODE_STREAMING &&
	        pChan->ConfParaLocal.validMask.FlushTO &&
	        pChan->ConfParaLocal.FlushTO != FLUSHTIMEOUT_DEFAULT)
	    {
	        *pBuf++ = L2CAP_CONFIG_PARA_FLUSH_TIMEOUT;
	        *pBuf++ = 0x02;
	        SHORT2CHAR(pBuf, pChan->ConfParaLocal.FlushTO);
	        pBuf += 2;
	    }
	}
	/* (F_BT_L2C_ENHANCED_FEATURE_SUPPORT) */

	/* (F_BT_L2C_QOS_SUPPORT) */

	if (L2capMode != L2CAP_MODE_BASIC && pConfPara->validMask.Qos == 1)
	{
	    L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_TRACE,
	        "!!!l2cSendL2CAP_CONFIGURE_REQUEST: L2CAP_CONFIG_PARA_QOS + L2CAP_CONFIG_PARA_FCandR not implemented");
	    assert(FALSE);
	}

	if (L2capMode != L2CAP_MODE_BASIC &&
	    pConfPara->validMask.Qos == 0)
	{
	    if (((L2capMode == L2CAP_MODE_ENHANCED_RETRANSMISSION &&
	          pChan->pHciDesc->remoteExtendedFeatures & L2CAP_EX_FEATURE_ENHANCED_RETRANS)) ||
	        ((L2capMode == L2CAP_MODE_STREAMING &&
	          pChan->pHciDesc->remoteExtendedFeatures & L2CAP_EX_FEATURE_STREAMING)))
	    {
	        uint16_t MPS;                 /* max. size of payload data */

	        MPS = pConfPara->flowControl.maxPDUSize;

	        if (MPS > (BT_US_PDU_L2C_BYTE_COUNT - (L2CAP_CONTROL_LENGTH +
	                                               L2CAP_FCS_LENGTH     +
	                                               L2CAP_SDU_LENGTH_FIELD_SIZE)))
	        {
	            MPS = BT_US_PDU_L2C_BYTE_COUNT -
	                  (L2CAP_CONTROL_LENGTH + L2CAP_FCS_LENGTH + L2CAP_SDU_LENGTH_FIELD_SIZE);
	        }

	        *pBuf++ = L2CAP_CONFIG_PARA_FCandR;
	        *pBuf++ = 9;
	        *pBuf++ = L2capMode;
	        *pBuf++ = pConfPara->flowControl.txWindowSize;
	        *pBuf++ = pConfPara->flowControl.maxTransmit;
	        SHORT2CHAR(pBuf, 0);          /* retransmission timeout */
            pBuf += 2;
	        SHORT2CHAR(pBuf, 0);          /* monitor timeout */
	        pBuf += 2;
	        SHORT2CHAR(pBuf, MPS);
	        pBuf += 2;

	        pChan->RespTxWindowSize = pConfPara->flowControl.txWindowSize; /* setup with local TWS */

	        L2CAP_TRACE_PRINTF_6(L2CAP_TRACE_MASK_TRACE,
	                                      "L2C<- L2CAP_CONFIG_PARA_FCandR mode %d txw %d maxtx %d retranst %d mont %d MPS %d",
	                                      L2capMode, pConfPara->flowControl.txWindowSize,
	                                      pConfPara->flowControl.maxTransmit, 0,
	                                      0, MPS);
	    }
	}

	/* F_BT_L2C_ENHANCED_CONFORMANCE */
	if ((L2capMode == L2CAP_MODE_ENHANCED_RETRANSMISSION ||
	     L2capMode == L2CAP_MODE_STREAMING) &&
	    pChan->pHciDesc->remoteExtendedFeatures & L2CAP_EX_FEATURE_OPTIONAL_FCS)
	{

	    *pBuf++ = L2CAP_CONFIG_PARA_FCS;
	    *pBuf++ = 1;
	    *pBuf++ = pChan->ConfParaLocal.fcs;
	}

#endif

	l2cSendL2CAPMessage(pChan,  NULL, L2CAP_CONFIGURE_REQUEST, mbuf, (uint16_t)(pBuf - mbuf), 0);
}

/**
* @brief  send l2cap config response to peer
* 
* @param  pChan
* @param  flags
* @param  result
* @param  options
* @param  optLen
*
* @return  
*
*/
void l2cSendL2CAP_CONFIGURE_RESPONSE(P_L2CAP_CHANNEL pChan, uint16_t flags, uint16_t result, PCBYTE options, uint16_t optLen)
{
    PBYTE mbuf = pL2c->mbuf;
    PBYTE pbuf = mbuf;

    SHORT2CHAR(pbuf, pChan->RemoteCid);                /* remote CID */
    pbuf += 2;
    SHORT2CHAR(pbuf, flags);                           /* flags */
    pbuf += 2;
    SHORT2CHAR(pbuf, result);                          /* result */
    pbuf += 2;

    if(options != NULL)
    {
		memcpy(pbuf, options, optLen);
		pbuf += optLen;
    }
    L2CAP_TRACE_PRINTF_4(L2CAP_TRACE_MASK_TRACE,
                                  "L2C<- L2CAP CONFIG RESP scid 0x%X flags 0x%x result 0x%X",
                                  pChan->RemoteCid, flags, result, 0);

    l2cSendL2CAPMessage(pChan,  NULL, L2CAP_CONFIGURE_RESPONSE, mbuf, (uint16_t)(pbuf - mbuf), 0 );
}

/**
* @brief  l2cap send connect response msg
* 
* @param  pChan
* @param  pHciDesc
* @param  lcid
* @param  rcid
* @param  result
* @param  status
*
* @return  
*
*/
void l2cSendL2CAP_CONNECTION_RESPONSE(P_L2CAP_CHANNEL pChan, P_ACLPOOLDESC pHciDesc, uint16_t lcid, uint16_t rcid,
                                    uint16_t result, uint16_t status)
{
    uint8_t ConRespPara[8];

    SHORT2CHAR(ConRespPara    , lcid);
    SHORT2CHAR(ConRespPara + 2, rcid);
    SHORT2CHAR(ConRespPara + 4, result);
    SHORT2CHAR(ConRespPara + 6, status);
	
    L2CAP_TRACE_PRINTF_4(L2CAP_TRACE_MASK_TRACE,
                                  "L2C<- L2CAP CONRESP dcid %X scid %X result %X status %X",
                                  lcid, rcid, result, status);
	
    l2cSendL2CAPMessage(pChan,  pHciDesc, L2CAP_CONNECTION_RESPONSE, ConRespPara, sizeof(ConRespPara), 0 );
}

/**
* @brief  send l2cap info request msg
*
* @param  pChan:
* @param  infoType:
*
* @return  
*
*/
void l2cSendL2CAP_INFORMATION_REQUEST(P_L2CAP_CHANNEL pChan, uint16_t infoType)
{
    uint8_t mbuf[4];
    PBYTE pbuf = mbuf;
	
    L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_TRACE, "L2C<- L2CAP_INFO_REQUEST");

    SHORT2CHAR(pbuf, infoType);
    pbuf += 2;

    l2cSendL2CAPMessage(pChan, NULL, L2CAP_INFORMATION_REQUEST, mbuf, (uint16_t)(pbuf - mbuf), L2CAP_INFO_RESP_RTX_TIME );

    pChan->L2capInformationRequestSent = TRUE;

}

/**
* @brief  send l2cap echo request msg
*
* @param  pChan: 
*
* @return  
*
*/
#if F_BT_L2C_ENHANCED_CONFORMANCE
void l2cSendL2CAP_ECHO_REQUEST(P_L2CAP_CHANNEL pChan)
{
    L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_TRACE, "L2C<- L2CAP_ECHO_REQUEST");

    l2cSendL2CAPMessage(pChan,  NULL, L2CAP_ECHO_REQUEST,
                                 NULL, 0,
                                 0);
}
#endif

/**
* @brief  decode l2cap config request param
* 
* @param  pChan
* @param  cmdId
* @param  flags
* @param  pPar
* @param  ParLength
*
* @return  
*
*/

int l2cDecodeConfigReqParas(P_L2CAP_CHANNEL pChan, uint8_t cmdId, uint16_t flags, LPCBYTE pPar, uint16_t ParLength)
{
    uint16_t subLen;

    while(ParLength >= 2)
    {
        uint8_t type = *pPar++;
        subLen = (uint16_t)*pPar++;
        ParLength -= 2;
        switch(type)
        {
        case L2CAP_CONFIG_PARA_MTU:
            pChan->RemoteMtu = CHAR2SHORT(pPar);
            pPar += 2;
            ParLength -= 2;
            L2CAP_TRACE_PRINTF_2(L2CAP_TRACE_MASK_TRACE,
                                    "L2C<- L2CAP_CONFIG_PARA_MTU len %d MTU %d", subLen, pChan->RemoteMtu);

            pChan->ConfParaRemote.mtuSize = pChan->RemoteMtu;
            if (pChan->ConfParaLocal.validMask.FlowControl == 0)
            {
                pChan->ConfParaRemote.flowControl.maxPDUSize = pChan->RemoteMtu;
            }
            break;
        case L2CAP_CONFIG_PARA_FLUSH_TIMEOUT:
            pChan->RemoteFlushTimeout = CHAR2SHORT(pPar);
            pPar += 2;
            ParLength -= 2;
            L2CAP_TRACE_PRINTF_2(L2CAP_TRACE_MASK_TRACE,
                                          "L2C<- L2CAP_CONFIG_PARA_FLUSH_TIMEOUT len %d FlushTimeout %d",
                                          subLen, pChan->RemoteFlushTimeout);

            pChan->ConfParaRemote.validMask.FlushTO = 1;
            pChan->ConfParaRemote.FlushTO           = pChan->RemoteFlushTimeout;
            break;
        case L2CAP_CONFIG_PARA_QOS:
            pPar += subLen;
            ParLength -= subLen;
            break;
        case L2CAP_CONFIG_PARA_FCandR:
            if (subLen == 9 && ParLength >= 9)
            {
               uint8_t L2capMode;
               uint16_t tmp = 0;

               pChan->ConfParaRemote.validMask.FlowControl = 1;
               L2capMode                                   = *(pPar++);
               pChan->ConfParaRemote.flowControl.mode      = (1 << L2capMode);
               if (L2capMode == L2CAP_MODE_BASIC)
               {
                 pPar += 8;                /* ignore parameters */
               }
               else
               {
                 pChan->ConfParaRemote.flowControl.txWindowSize          = *(pPar++);
                 pChan->ConfParaRemote.flowControl.maxTransmit           = *(pPar++);
                 pPar += 4;
                 tmp                                                     = CHAR2SHORT(pPar); pPar += 2;
                 pChan->ConfParaRemote.flowControl.maxPDUSize = min(tmp, BT_DS_PDU_L2C_BYTE_COUNT);
               }

               L2CAP_TRACE_PRINTF_7(L2CAP_TRACE_MASK_TRACE,
                                             "L2C<- L2CAP_CONFIG_PARA_FCandR len %d mode %d txw %d maxtx %d retranst %d mont %d MPS %d",
                                             subLen, L2capMode, pChan->ConfParaRemote.flowControl.txWindowSize,
                                             pChan->ConfParaRemote.flowControl.maxTransmit, pChan->ConfParaRemote.flowControl.retransmissionTimeout,
                                             pChan->ConfParaRemote.flowControl.monitorTimeout, tmp);
            }
            else
            {
               L2CAP_TRACE_PRINTF_2(L2CAP_TRACE_MASK_TRACE,
                         "!!L2C<- L2CAP_CONFIG_PARA_FCandR wrong length slen %d plen %d", subLen, ParLength);
               pPar += subLen;
            }
             ParLength -= subLen;
            break;
        case L2CAP_CONFIG_PARA_FCS:

            if (subLen == 1 && ParLength >= 1)
            {
               pChan->ConfParaRemote.fcs = *(pPar++);

               L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_TRACE,
                                       "L2C<- L2CAP_CONFIG_PARA_FCS FCS %d", pChan->ConfParaRemote.fcs);
            }
            else
            {
               L2CAP_TRACE_PRINTF_2(L2CAP_TRACE_MASK_TRACE,
                           "!!L2C<- L2CAP_CONFIG_PARA_FCS wrong length slen %d plen %d", subLen, ParLength);
               pPar += subLen;
            }
            ParLength -= subLen;
            break;
        default:                        /* invalid Config type */
            if((type & 0x80) == 0)                /* option must be recognized */
            {
                uint8_t mbuf[10];
                PBYTE pbuf = mbuf;

                L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_TRACE,
                                        "!!L2C<- L2CAP_CONFIG_PARA unknown mandatory Option 0x%X", type);
                *pbuf = type;
                pbuf++;

                pChan->ReqIdRcv        = cmdId;
                l2cSendL2CAP_CONFIGURE_RESPONSE(pChan, flags, L2CAP_CFGRSP_UNKNOWN_OPTIONS, mbuf, (uint16_t)(pbuf - mbuf) );
                return L2CAP_CFGRSP_REJECTED;
            } else
            {                                                /* ignore option */
                L2CAP_TRACE_PRINTF_2(L2CAP_TRACE_MASK_TRACE,
                                        "L2C<- L2CAP_CONFIG_PARA unknown Option ignored 0x%X %d", type, subLen);
                pPar += subLen;
                ParLength -= subLen;
            }
            break;
        }
    }
    return 0;
}

/**
* @brief  decode l2cap config response param
* 
* @param  pChan
* @param  pPar
* @param  ParLength
*
* @return  
*
*/
void l2cDecodeConfigRespParas(P_L2CAP_CHANNEL pChan, LPCBYTE pPar, uint16_t ParLength)
{
    uint16_t subLen;

    while(ParLength >= 2)
    {
        uint8_t type = *pPar++;
        subLen = (uint16_t)*pPar++;
        ParLength -= 2;
        switch(type)
        {
		case L2CAP_CONFIG_PARA_MTU:
			pChan->LocalUsMtu = CHAR2SHORT(pPar);
			pPar += 2;
			ParLength -= 2;
			L2CAP_TRACE_PRINTF_2(L2CAP_TRACE_MASK_TRACE,
			                              "L2C-> L2CAP_CONFIG_PARA_MTU len %d MTU %d",
			                              subLen, pChan->LocalUsMtu);
			if(pChan->LocalUsMtu > pChan->LocalMtu)
			{
			    L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_ERROR,
			                            "!!L2C-> !!outgoing MTU of remote device > local MTU!!");
			    pChan->LocalUsMtu = pChan->LocalMtu;
			}
			break;
#if F_BT_L2C_ENHANCED_FEATURE_SUPPORT 			
        case L2CAP_CONFIG_PARA_FLUSH_TIMEOUT:
            {
#if (L2CAP_TRACE_VERBOSITY_COUNT<0 || L2CAP_TRACE_VERBOSITY_COUNT>=2)
                uint16_t tmp;
                tmp = CHAR2SHORT(pPar);
                pPar += 2;
                ParLength -= 2;
                L2CAP_TRACE_PRINTF_2(L2CAP_TRACE_MASK_TRACE,
                                        "L2C-> L2CAP_CONFIG_PARA_FLUSH_TIMEOUT len %d Flushtimeout %d ignored",
                                        subLen, tmp);
#endif
            }
            break;

        case L2CAP_CONFIG_PARA_QOS:
            pPar += subLen;
            ParLength -= subLen;
            break;
        case L2CAP_CONFIG_PARA_FCandR:
            if (subLen == 9 && ParLength >= 9)
            {
 #if F_BT_L2C_ENHANCED_FEATURE_SUPPORT //champion
               uint8_t L2capMode;

               pChan->ConfParaLocal.validMask.FlowControl = 1;
               L2capMode                                              = *(pPar++);
               pChan->RespMode                                        = (1 << L2capMode);
               pChan->RespTxWindowSize                                = *(pPar++);
               pChan->ConfParaLocal.flowControl.maxTransmit           = *(pPar++);
               pChan->ConfParaRemote.flowControl.retransmissionTimeout = CHAR2SHORT(pPar); pPar += 2;
               pChan->ConfParaRemote.flowControl.monitorTimeout        = CHAR2SHORT(pPar); pPar += 2;
               pChan->RespMaxPDUSize                                    = CHAR2SHORT(pPar); pPar += 2;

               L2CAP_TRACE_PRINTF_7(L2CAP_TRACE_MASK_TRACE,
                                             "L2C-> L2CAP_CONFIG_PARA_FCandR len %d mode %d txw %d maxtx %d retranst %d mont %d mps %d",
                                             subLen, L2capMode, pChan->ConfParaLocal.flowControl.txWindowSize,
                                             pChan->ConfParaLocal.flowControl.maxTransmit, pChan->ConfParaLocal.flowControl.retransmissionTimeout,
                                             pChan->ConfParaLocal.flowControl.monitorTimeout, pChan->ConfParaLocal.flowControl.maxPDUSize);

#endif
            }
            else
            {
               L2CAP_TRACE_PRINTF_2(L2CAP_TRACE_MASK_TRACE,
                          "!!L2C-> L2CAP_CONFIG_PARA_FCandR wrong length slen %d plen %d", subLen, ParLength);
               pPar += subLen;
            }
             ParLength -= subLen;
            break;
        case L2CAP_CONFIG_PARA_FCS:
            if (subLen == 1 && ParLength >= 1)
            {
               L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_TRACE,
                                       "L2C-> L2CAP_CONFIG_PARA_FCS FCS %d", *(pPar));
               pPar++;
            }
            else
            {
               L2CAP_TRACE_PRINTF_2(L2CAP_TRACE_MASK_TRACE,
                           "!!L2C-> L2CAP_CONFIG_PARA_FCS wrong length slen %d plen %d", subLen, ParLength);
               pPar += subLen;
            }
            ParLength -= subLen;
            break;
        default:
            break;
#else
		default:
			pPar += subLen;
			ParLength -= subLen;
			break;
#endif
        }
    }
}   
       


/**
* @brief  handle l2cap config request in config or open state
* 
* @param  pChan
*
* @return  
*
*/
void l2cHandleL2CAP_CONFIGURE_REQUESTinStateConfigOrOpen(P_L2CAP_CHANNEL pChan)
{
    uint8_t            mbuf[L2CAP_SIGNAL_MTU];
    PBYTE           pBuf = mbuf;
    uint16_t            Status = L2CAP_CFGRSP_SUCCESS;
    uint16_t            NextMode;
    uint16_t            tmp;
    uint8_t            L2capMode;

    memset((PVOID)mbuf, 0, sizeof(mbuf));

    NextMode = pChan->CurrentMode;
#if F_BT_L2C_ENHANCED_CONFORMANCE
    if (pChan->CurrentMode != pChan->ConfParaRemote.flowControl.mode)
    {
        if (pChan->CurrentMode > pChan->ConfParaRemote.flowControl.mode)
        {
            if (pChan->ConfParaLocal.flowControl.mode & pChan->ConfParaRemote.flowControl.mode)
            {
                NextMode = pChan->ConfParaRemote.flowControl.mode;
            }
            else
            {
                Status = L2CAP_CFGRSP_UNACCEPTABLE_PARA;
                NextMode = l2cGetNextMode(pChan, pChan->ConfParaLocal.flowControl.mode, pChan->ConfParaRemote.flowControl.mode);
                pChan->ConfParaRemote.flowControl.mode = NextMode;
            }

            if (NextMode == 0)
            {
                l2cSendL2CAP_DISCONNECTION_REQUEST(pChan);
                l2cChangeState(pChan, l2cStateWaitForL2CAP_DISCONNECTION_RESPONSE);
                return;
            }
        }
        else
        {
            Status = L2CAP_CFGRSP_UNACCEPTABLE_PARA;
            NextMode                               = pChan->CurrentMode;
            pChan->ConfParaRemote.flowControl.mode = pChan->CurrentMode;
        }
    }

    if (NextMode)
      L2capMode = l2cGetL2capMode(NextMode);
    else
#endif		
      L2capMode = L2CAP_MODE_BASIC;

    if (Status == L2CAP_CFGRSP_UNACCEPTABLE_PARA)
    {
        if (pChan->UnacceptableMode == pChan->ConfParaRemote.flowControl.mode)
        {
            l2cSendL2CAP_DISCONNECTION_REQUEST(pChan);
            l2cChangeState(pChan, l2cStateWaitForL2CAP_DISCONNECTION_RESPONSE);
            return;
        }
        pChan->UnacceptableMode = pChan->ConfParaRemote.flowControl.mode;
    }

    *pBuf = L2CAP_CONFIG_PARA_MTU;
    pBuf++;
    *pBuf = 2;
    pBuf++;
#if (BT_DS_MTU_L2C_BYTE_COUNT)
    tmp = min(pChan->RemoteMtu, BT_DS_MTU_L2C_BYTE_COUNT);
#else
    tmp = pChan->RemoteMtu;
#endif
    if (L2capMode == L2CAP_MODE_BASIC)
    {
		tmp = min(tmp, BT_DS_PDU_L2C_BYTE_COUNT);
    }

    if (tmp < L2CAP_SIGNAL_MTU)
    {
        Status = L2CAP_CFGRSP_UNACCEPTABLE_PARA;
        tmp    = L2CAP_SIGNAL_MTU;    /* minimum MTU size */

        pChan->RemoteMtu = tmp;
    }

/* F_BT_L2C_ENHANCED_CONFORMANCE */

    SHORT2CHAR(pBuf, tmp);    /* MTU */
    pBuf += 2;

/* (F_BT_L2C_ENHANCED_CONFORMANCE) */

    if (L2capMode != L2CAP_MODE_BASIC ||
        (L2capMode == L2CAP_MODE_BASIC && Status != L2CAP_CFGRSP_SUCCESS))
    {
        uint32_t extendedFeatures = L2CAP_EX_FEATURE_ENHANCED_RETRANS | L2CAP_EX_FEATURE_STREAMING  |  L2CAP_EX_FEATURE_OPTIONAL_FCS;

        if (pChan->ConfParaRemote.validMask.FlowControl &&
            pChan->pHciDesc->remoteExtendedFeatures & extendedFeatures)
        {
            *pBuf++ = L2CAP_CONFIG_PARA_FCandR;
            *pBuf++ = 9;
            *pBuf++ = L2capMode;
			
#if F_BT_L2C_ENHANCED_CONFORMANCE
            if (L2capMode == L2CAP_MODE_ENHANCED_RETRANSMISSION)
            {
                if (pChan->ConfParaRemote.flowControl.txWindowSize > pChan->MaxDsBuffer)
                  pChan->ConfParaRemote.flowControl.txWindowSize = pChan->MaxDsBuffer;  /* we don't have so much buffers */

                *pBuf++ = pChan->ConfParaRemote.flowControl.txWindowSize;
                *pBuf++ = pChan->ConfParaRemote.flowControl.maxTransmit;
                if (Status == L2CAP_CFGRSP_SUCCESS)
                {
                    SHORT2CHAR(pBuf, pChan->ConfParaLocal.flowControl.retransmissionTimeout);
                    pBuf += 2;
                    SHORT2CHAR(pBuf, pChan->ConfParaLocal.flowControl.monitorTimeout);
                    pBuf += 2;
                }
                else
                {
                    SHORT2CHAR(pBuf, pChan->ConfParaRemote.flowControl.retransmissionTimeout);
                    pBuf += 2;
                    SHORT2CHAR(pBuf, pChan->ConfParaRemote.flowControl.monitorTimeout);
                    pBuf += 2;
                }
            }
            else
#endif
            {
                pBuf += 6;
            }
            SHORT2CHAR(pBuf, pChan->ConfParaRemote.flowControl.maxPDUSize);
            pBuf += 2;

            L2CAP_TRACE_PRINTF_6(L2CAP_TRACE_MASK_TRACE,
                                    "L2C<- L2CAP_CONFIG_Resp mode %d txw %d maxtx %d retranst %d mont %d MPS %d",
                                    L2capMode, pChan->ConfParaRemote.flowControl.txWindowSize,
                                    pChan->ConfParaRemote.flowControl.maxTransmit, pChan->ConfParaRemote.flowControl.retransmissionTimeout,
                                    pChan->ConfParaRemote.flowControl.monitorTimeout, pChan->ConfParaRemote.flowControl.maxPDUSize);
        }
    }

/* F_BT_L2C_ENHANCED_CONFORMANCE */

    //@@@@ in case of neg result or wildcard paras add config para
    if (Status == L2CAP_CFGRSP_SUCCESS)
    {
        pChan->ConfRspReady = TRUE;
    }

    l2cSendL2CAP_CONFIGURE_RESPONSE(pChan, 0, Status, mbuf, (uint16_t)(pBuf - mbuf) );
/* F_BT_L2C_ENHANCED_CONFORMANCE */

    if( pChan->ConfReqReady && pChan->ConfRspReady) 
	{
        if (pChan->State == l2cStateOpen)
        {
            l2cSendL2CAP_CONFIGURE_REQUEST(pChan);
            l2cChangeState(pChan, l2cStateConfig);
            return;
        }

        l2cStopCONFIGTimeout(pChan);   /* stop timeout */
        pChan->Mode = l2cGetL2capMode(NextMode);
#if F_BT_L2C_ENHANCED_FEATURE_SUPPORT 
        if (pChan->Mode == L2CAP_MODE_BASIC)
          l2cDeleteExtFData(pChan);
        else
          l2cStateIsOpened(pChan);
#endif
        if (pChan->pHciDesc->FlushTO != pChan->ConfParaLocal.FlushTO)
        {
			pChan->pHciDesc->FlushTO = pChan->ConfParaLocal.FlushTO;
			l2cSendLHciWriteAutomaticFlushTimeout(pChan->pHciDesc);
        }
/* (F_BT_L2C_ENHANCED_FEATURE_SUPPORT) */
        l2cSendL2C_CON_ACT_IND(pChan, pChan->usQueueID, pChan->LocalCid, 0);

        l2cChangeState(pChan, l2cStateOpen);
        return;
    }
}

/**
* @brief  l2cap handle connect request msg
* 
* @param  hciDesc
* @param  l2cap_cmd
* @param  pPar
*
* @return  
*
*/
void l2cHandleL2CAP_CONNECTION_REQUEST(P_ACLPOOLDESC hciDesc, T_L2CAP_Command *l2cap_cmd, uint8_t * pPar)
{
	P_L2CAP_CHANNEL     pChan;   
	T_L2CAP_ConReq_para L2CAP_ConReq;
	uint16_t                psmIdx;

	L2CAP_ConReq.psm  = CHAR2SHORT(pPar);
	L2CAP_ConReq.scid = CHAR2SHORT(pPar + 2);

	L2CAP_TRACE_PRINTF_2(L2CAP_TRACE_MASK_TRACE,
	                        "L2C-> L2CAP ConReq psm %X scid %X", L2CAP_ConReq.psm, L2CAP_ConReq.scid);
	/*find psm idx*/
	for(psmIdx = 0;
	    (psmIdx < otp_str_data.gEfuse_UpperStack_s.num_data_psm) && pL2c->uplDesc[psmIdx].psm != L2CAP_ConReq.psm;
	    psmIdx++)
	      ;

	hciDesc->ReqIdRcv = l2cap_cmd->id;

	if((L2CAP_ConReq.psm != 0) && (psmIdx < otp_str_data.gEfuse_UpperStack_s.num_data_psm))  /* found a registered client */
	{
		L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_TRACE,
		                      "l2cHandleSigCmd(): found UPL Desc index %d", psmIdx);

		if((pChan = l2cCheckRcid(hciDesc, L2CAP_ConReq.scid )) == NULL) 
		{
			/* we are not able to create a new channel */
			L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_ERROR,
			                      "!!l2cHandleSigCmd(): couldn't create new channel");

			l2cSendL2CAP_CONNECTION_RESPONSE(NULL, hciDesc, 0, 0, L2CAP_ERR_REFUS_NO_RESOURCE, 0);
			return;
	  	}
		pChan->usQueueID     = pL2c->uplDesc[psmIdx].queueID;
		pChan->psm           = pL2c->uplDesc[psmIdx].psm;
		pChan->role          = terminate;                         /* in this case we are the terminator */
	}
	else  /* it is an invalid or unsupported PSM */
	{
		L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_ERROR,
		                      "!!l2cHandleSigCmd(): PSM invalid or unsupported !");

		l2cSendL2CAP_CONNECTION_RESPONSE(NULL, hciDesc, 0, 0, L2CAP_ERR_REFUS_INV_PSM, 0);
		return;
	}

	if(l2cap_cmd->id == pChan->ReqIdRcv)         /* it is a repeatet request because same cmd ID */
	{
	    /* so we will send again the last response */
	    l2cReSendL2CAPMessage(pChan, &pChan->lastRspSend);
	    return;                                                                /* thats all */
	}

	pChan->ReqIdRcv = l2cap_cmd->id;

	switch (pChan->State)
	{
	case l2cStateClosed:
		/* send always connection pending */
		l2cSendL2CAP_CONNECTION_RESPONSE(pChan, NULL, pChan->LocalCid, pChan->RemoteCid,
		                                                     L2CAP_ERR_PENDING, 0 /* no further informastion */);
		l2cStopCONFIGTimeout(pChan);     /* stop the timeout */
		l2cSendSECMAN_AUTHENTICATION_IND(pChan, 0 /*outgoing*/, 1 /* active */);
		break;

	default:
		break;
	}
}

/**
* @brief  l2cap handle connect response
* 
* @param  l2cap_cmd
* @param  pPar
*
* @return  
*
*/
void l2cHandleL2CAP_CONNECTION_RESPONSE(T_L2CAP_Command *l2cap_cmd, uint8_t * pPar)
{
    P_L2CAP_CHANNEL      pChan;
    T_L2CAP_ConResp_para L2CAP_ConResp;

    L2CAP_ConResp.dcid   = CHAR2SHORT(pPar);
    L2CAP_ConResp.scid   = CHAR2SHORT(pPar + 2);
    L2CAP_ConResp.result = CHAR2SHORT(pPar + 4);
    L2CAP_ConResp.status = CHAR2SHORT(pPar + 6);

    L2CAP_TRACE_PRINTF_4(L2CAP_TRACE_MASK_TRACE,
                            "L2C-> L2CAP CONRESP dcid %X scid %X result %X status %X",
                            L2CAP_ConResp.dcid, L2CAP_ConResp.scid, L2CAP_ConResp.result, L2CAP_ConResp.status);

    if(!L2CAP_ConResp.result)                 /* positive response */
    {
        if((pChan = l2cSearchLcid(L2CAP_ConResp.scid)) == NULL)
        {
            L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_ERROR,
                                    "!!l2cHandleSigCmd(): couldn't find l2cap channel");
            return;
        }
        pChan->RemoteCid = L2CAP_ConResp.dcid;
    }
    else 
	{                                                        /* negative connect response */
        if(L2CAP_ConResp.result == L2CAP_ERR_PENDING)
        {
            return;
        }
        else
        {
            if((pChan = l2cSearchCmdId(l2cap_cmd->id)) == NULL)
            {
                L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_ERROR,
                                        "!!l2cHandleSigCmd(): couldn't find l2cap channel");
                return;
            }
        }
    }
    pChan->RespIdRcv = l2cap_cmd->id;

    switch (pChan->State)
    {
	case l2cStateWaitForL2CAP_CONNECTION_RESPONSE:
		if (L2CAP_ConResp.result != 0)   /* if its an error result mark it with our layer mask */
		{
		    L2CAP_ConResp.result = (L2CAP_ConResp.result & ~BLUEFACE_ERR_SRC_MASK) | L2CAP_ERR;
		}

		if(L2CAP_ConResp.result == 0)
		{
			l2cGetNextCurrentMode(pChan);            /* get first mode for negotiation */
			/* F_BT_L2C_ENHANCED_CONFORMANCE */
			if (pChan->CurrentMode != 0)
			{
				l2cSendL2CAP_CONFIGURE_REQUEST(pChan);
				l2cChangeState(pChan, l2cStateConfig);
			}
#if F_BT_L2C_ENHANCED_FEATURE_SUPPORT			
			else  /* mode is not allowed */
			{
				l2cSendL2CAP_DISCONNECTION_REQUEST(pChan);
				l2cChangeState(pChan, l2cStateWaitForL2CAP_DISCONNECTION_RESPONSE);
			}
#endif			
		}
		else if(L2CAP_ConResp.result == (L2CAP_ERR_PENDING | L2CAP_ERR))
		{
		    l2cStartRTXTimeout(pChan, L2CAP_ERTX_TIME);
		}
		else
		{
			l2cChangeState(pChan, l2cStateClosed);
		    l2cSendL2C_DISC_IND(pChan, L2CAP_ConResp.result);
		}
		break;

      default:
        break;
    }
}

/**
* @brief  handle remote l2cap config request msg
* 
* @param  hciDesc
* @param  l2cap_cmd
* @param  pPar
*
* @return  
*
*/
void l2cHandleL2CAP_CONFIGURE_REQUEST(P_ACLPOOLDESC hciDesc, T_L2CAP_Command *l2cap_cmd, uint8_t * pPar)
{
    P_L2CAP_CHANNEL     pChan;
    T_L2CAP_Config_para L2CAP_Config;

    L2CAP_Config.dcid  = CHAR2SHORT(pPar);
    pPar += 2;
    L2CAP_Config.flags = CHAR2SHORT(pPar);
    pPar += 2;

    L2CAP_TRACE_PRINTF_4(L2CAP_TRACE_MASK_TRACE,
                            "L2C-> L2CAP CONFIG REQ dcid 0x%X flags 0x%X options_len %d",
                            L2CAP_Config.dcid, L2CAP_Config.flags, l2cap_cmd->length - 4, 0);

    pChan = l2cSearchLcid(L2CAP_Config.dcid);

    if(pChan == NULL) /* channel not found */
    {
        l2cSendL2CAP_COMMAND_REJECT(NULL, hciDesc, L2CAP_CMDREJ_INVALID_CID, L2CAP_Config.dcid, 0);
    }
    else
    {
		if( l2cDecodeConfigReqParas(pChan, l2cap_cmd->id, L2CAP_Config.flags, pPar,
		  (uint16_t)(l2cap_cmd->length - 4)) )
		{        /* error return */
			return;                /* just go one do not inform ULP */
		}

		if( L2CAP_Config.flags & 0x01 )                /* not yet the final config frame */
		{
			pChan->ReqIdRcv  = l2cap_cmd->id;
			l2cSendL2CAP_CONFIGURE_RESPONSE(pChan, 0x01, L2CAP_CFGRSP_SUCCESS, NULL, 0);
			return;                /* just go one until final received */
		}
    }

    if(l2cap_cmd->id == pChan->ReqIdRcv)        /* it is a repeatet request because same cmd ID */
    {
        /* so we will send again the last response */
        l2cReSendL2CAPMessage(pChan, &pChan->lastRspSend);
        return;                                                                /* thats all */
    }
    pChan->ReqIdRcv  = l2cap_cmd->id;

    switch (pChan->State)
    {
	case l2cStateConfig:
	case l2cStateOpen:
		l2cHandleL2CAP_CONFIGURE_REQUESTinStateConfigOrOpen(pChan);
		break;

	case l2cStateWaitForL2C_CON_RESP:
		if (pChan->L2capInformationRequestSent)
		{
			pChan->SendL2capConfigureResponse = TRUE;
		}
		break;

	default:
		break;
    }
}

/**
* @brief  handle l2cap configure response
* 
* @param  l2cap_cmd
* @param  pPar
*
* @return  
*
*/
void l2cHandleL2CAP_CONFIGURE_RESPONSE(T_L2CAP_Command *l2cap_cmd, uint8_t * pPar)
{
    P_L2CAP_CHANNEL         pChan;
    T_L2CAP_ConfigResp_para L2CAP_ConfigResp;

    L2CAP_ConfigResp.scid   = CHAR2SHORT(pPar);
    L2CAP_ConfigResp.flags  = CHAR2SHORT(pPar + 2);
    L2CAP_ConfigResp.result = CHAR2SHORT(pPar + 4);

    l2cap_cmd->length -= 6;

    L2CAP_TRACE_PRINTF_4(L2CAP_TRACE_MASK_TRACE,
                            "L2C-> L2CAP CONFIG RESP scid 0x%X flags 0x%x result 0x%X",
                            L2CAP_ConfigResp.scid, L2CAP_ConfigResp.flags, L2CAP_ConfigResp.result, 0);

    if((pChan = l2cSearchLcid(L2CAP_ConfigResp.scid)) == NULL)
    {
        return;       /* if channel not found just ignore the response */
    }

    l2cDecodeConfigRespParas(pChan, pPar + 6, l2cap_cmd->length);

    if( L2CAP_ConfigResp.flags & 0x01 ) /* not yet the final config frame */
    {
		uint8_t txCommand;
		uint16_t txFlags;

#if (BT_L2C_MAX_CMD_RETRIES_COUNT != 0) 
		txCommand = *pChan->lastReqSend.buf;
		txFlags = CHAR2SHORT(pChan->lastReqSend.buf + 6);
#else 
		txCommand = pChan->lastReqSend;
		txFlags = pChan->lastConfigFlagSend;
#endif  

		L2CAP_TRACE_PRINTF_2(L2CAP_TRACE_MASK_TRACE,
		                      "L2C lastcmd 0x%X lastflags 0x%x",
		                      txCommand, txFlags);

		if((txCommand == L2CAP_CONFIGURE_REQUEST) && ((txFlags & 0x01) == 0))   /* we did not sent a continuation flag */
		{                                                                   /* errata 2263 request to send out a CONFIG_REQUEST */
			uint8_t buf[4];        /* space for config cmd */
			PBYTE pBuf = buf;

			SHORT2CHAR(pBuf, pChan->RemoteCid);
			pBuf += 2;
			SHORT2CHAR(pBuf, 0);                    /* flags field: C bit clear */
			pBuf += 2;

			l2cSendL2CAPMessage(pChan,  NULL, L2CAP_CONFIGURE_REQUEST, buf, (uint16_t)(pBuf - buf), 0 );
		}

		return;       /* just go one until final received */
    }

    pChan->RespIdRcv = l2cap_cmd->id;

    switch (pChan->State)
    {
	case l2cStateConfig:
		switch (L2CAP_ConfigResp.result)
		{
		default:
		case L2CAP_CFGRSP_SUCCESS:
			pChan->ConfReqReady = TRUE;
#if F_BT_L2C_ENHANCED_CONFORMANCE
			if (pChan->ConfParaRemote.validMask.FlowControl == 0 &&
			  pChan->CurrentMode != L2C_MODE_BASIC)
			  break;
#endif
			if (pChan->CurrentMode == pChan->ConfParaRemote.flowControl.mode)
			{
				assert(pChan->role != undef);

				if( pChan->ConfRspReady )       /* request from remote side done */
				{
					l2cStopCONFIGTimeout(pChan);   /* stop timeout */

					if (pChan->State == l2cStateOpen)
					{
						l2cChangeState(pChan, l2cStateConfig);
						return;
					}
					pChan->Mode = l2cGetL2capMode(pChan->CurrentMode);
#if F_BT_L2C_ENHANCED_FEATURE_SUPPORT 
					if (pChan->Mode == L2CAP_MODE_BASIC)
					l2cDeleteExtFData(pChan);
					else
					l2cStateIsOpened(pChan);
#endif
					if (pChan->pHciDesc->FlushTO != pChan->ConfParaLocal.FlushTO)
					{
						pChan->pHciDesc->FlushTO = pChan->ConfParaLocal.FlushTO;
	                    l2cSendLHciWriteAutomaticFlushTimeout(pChan->pHciDesc);
					}
					l2cSendL2C_CON_ACT_IND(pChan, pChan->usQueueID, pChan->LocalCid, 0);

					l2cChangeState(pChan, l2cStateOpen);
					return;
			  }
			  break;
		}

        case L2CAP_CFGRSP_UNACCEPTABLE_PARA:
            if (!(pChan->ConfParaLocal.flowControl.mode & pChan->RespMode))
            {
/* F_BT_L2C_ENHANCED_CONFORMANCE */
                {
                    if (pChan->RespMode > pChan->CurrentMode)
                        pChan->CurrentMode = 0;   /* negotiation imposible */
                    else
                        pChan->CurrentMode = l2cGetNextMode(pChan, pChan->ConfParaLocal.flowControl.mode, pChan->RespMode);
                }
            }
            else
            {
                pChan->CurrentMode = pChan->RespMode;
            }	
            if (pChan->CurrentMode == 0)
            {
                l2cSendL2CAP_DISCONNECTION_REQUEST(pChan);
                l2cChangeState(pChan, l2cStateWaitForL2CAP_DISCONNECTION_RESPONSE);
                return;
            }
            else
            {
/* F_BT_L2C_ENHANCED_CONFORMANCE */
                if (pChan->ConfParaLocal.mtuSize > pChan->LocalUsMtu)
            	{
                	pChan->ConfParaLocal.mtuSize = pChan->LocalUsMtu;
            	}
                l2cSendL2CAP_CONFIGURE_REQUEST(pChan);
            }
            break;

/* F_BT_L2C_ENHANCED_CONFORMANCE */
      }
      break;

	default:
		break;
    }
}

/**
* @brief  l2cap handle echo request
* 
* @param  hciDesc
* @param  l2cap_cmd
*
* @return  
*
*/
void l2cHandleL2CAP_ECHO_REQUEST(P_ACLPOOLDESC hciDesc, T_L2CAP_Command *l2cap_cmd)
{
	char echoData[] = "Sto//mann Germany";

#if (L2CAP_TRACE_VERBOSITY_COUNT >= 0) && (L2CAP_TRACE_VERBOSITY_COUNT < 2)
	UNUSED_PARAMETER(l2cap_cmd);
#endif

    L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_TRACE,"L2C-> L2CAP_ECHO_REQUEST");

    L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_TRACE, "L2C<- L2CAP_ECHO_RESPONSE");

    l2cSendL2CAPMessage(NULL, hciDesc, L2CAP_ECHO_RESPONSE, (uint8_t *)echoData, sizeof(echoData), 0 );
}

#if F_BT_L2C_ENHANCED_CONFORMANCE
/**
* @brief  l2cap handle echo response
* 
* @param  pChan
* @param  l2cap_cmd
*
* @return  
*
*/
void l2cHandleL2CAP_ECHO_RESPONSE(P_L2CAP_CHANNEL pChan, T_L2CAP_Command *l2cap_cmd)
{
#if (L2CAP_TRACE_VERBOSITY_COUNT >= 0) && (L2CAP_TRACE_VERBOSITY_COUNT < 2)
    UNUSED_PARAMETER(l2cap_cmd);
#endif

    L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_TRACE,
                            "L2C-> L2CAP_ECHO_RESPONSE length %d",
                            l2cap_cmd->length);

    if(pChan->role == pingreq) 
	{    /* the sole porpuse for this channel was a EchoReq */
        (void)l2cDeleteChannel(pChan, FALSE);
    }
}
#endif

/**
* @brief  l2cap handle L2CAP_INFO_REQ
* 
* @param  hciDesc
* @param  l2cap_cmd
* @param  pPar
*
* @return  
*
*/
void l2cHandleL2CAP_INFORMATION_REQUEST(P_ACLPOOLDESC hciDesc, T_L2CAP_Command *l2cap_cmd, uint8_t * pPar)
{
    uint16_t infoType = CHAR2SHORT(pPar);
    uint8_t InfoResp[8];
    uint16_t respLen = 4;

    L2CAP_TRACE_PRINTF_2(L2CAP_TRACE_MASK_TRACE,
                            "L2C-> L2CAP_INFORMATION_REQUEST length %d infotype 0x%X",
                            l2cap_cmd->length, infoType);

    SHORT2CHAR(InfoResp, infoType);

    if(infoType == L2CAP_INFOTYPE_EXTENDED_FEATURES_SUPPORT)
    {
		uint32_t localExtendedFeatures = 0;                  /* all feature bits 0 */

		SHORT2CHAR(InfoResp + 2, 0x0000);                /* respond: success */
		LONG2CHAR(InfoResp + 4, localExtendedFeatures);
		respLen = 8;
    }
    else
    {
		SHORT2CHAR(InfoResp + 2, 0x0001);                /* respond: not supported */
    }

    hciDesc->ReqIdRcv = l2cap_cmd->id;
    L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_TRACE, "L2C<- L2CAP_INFORMATION_RESPONSE");
    l2cSendL2CAPMessage(NULL, hciDesc, L2CAP_INFORMATION_RESPONSE,
                        InfoResp, respLen,
                        0);
}

/**
* @brief  l2cap handle L2CAP_INFO_RES
* 
* @param  pChan
* @param  hciDesc
* @param  l2cap_cmd
* @param  pPar
*
* @return  
*
*/
void l2cHandleL2CAP_INFORMATION_RESPONSE(P_L2CAP_CHANNEL pChan, P_ACLPOOLDESC hciDesc, T_L2CAP_Command *l2cap_cmd, uint8_t * pPar)
{
   	uint16_t infoType = CHAR2SHORT(pPar);
    uint16_t result = CHAR2SHORT(pPar + 2);

    L2CAP_TRACE_PRINTF_4(L2CAP_TRACE_MASK_TRACE,
                            "L2C-> L2CAP_INFORMATION_RESPONSE length %d infotyp %d result 0x%x",
                            l2cap_cmd->length, infoType, result, 0);

    if(result == 0)
    {
		switch(infoType)
		{
			case L2CAP_INFOTYPE_EXTENDED_FEATURES_SUPPORT:
				hciDesc->remoteExtendedFeatures = CHAR2LONG(pPar + 4);
				L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_TRACE,
				                        "L2CAP_INFOTYPE_EXTENDED_FEATURES_MASK 0x%lx",
				                        hciDesc->remoteExtendedFeatures);
				break;
		}
    }
    l2cCheckMode(pChan);

    switch (pChan->State)
    {
	case l2cStateWaitForL2CAP_CONNECTION_RESPONSE:
		l2cSendSECMAN_AUTHENTICATION_IND(pChan, 1 /*outgoing*/, 1 /* active */);
		break;

	default:
		break;
    }
}


/**
* @brief  send l2cap connection active indicate to up layer
* 
* @param  pChan
* @param  usQueueID
* @param  cid
* @param  status
*
* @return  
*
*/
void l2cSendL2C_CON_ACT_IND(PC_L2CAP_CHANNEL pChan, uint16_t usQueueID, uint16_t cid, uint16_t status)
{
	T_L2C_Conf_para tConfPara;
	P_L2C_Conf_para pConfPara = &tConfPara;
	TCON_ACT_IND_L2CAP l2cap;
	LPFlowSpec		 pflow;
	LPFlowControl	 pflc; 

    L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_TRACE,
                            "L2C<- L2C_CON_ACT_IND cid 0x%X", cid);

    memcpy((PVOID)&tConfPara, (PVOID)&pChan->ConfParaRemote, sizeof(T_L2C_Conf_para));

	L2CAP_TRACE_PRINTF_6(L2CAP_TRACE_MASK_TRACE,
	                           "L2C<- L2C_CON_ACT_IND type %d tokrate %ld tokbucket %ld peak %ld latency %ld delay %ld",
	                           pConfPara->flowSpec.serviceType, pConfPara->flowSpec.tokenRate,
	                           pConfPara->flowSpec.tokenBucket, pConfPara->flowSpec.peakBandwidth,
	                           pConfPara->flowSpec.latency, pConfPara->flowSpec.delayVariation);

	l2cap.lcid				  = cid;
	l2cap.result			  = status;
	l2cap.status			  = 0;
	l2cap.conf.mode = pConfPara->flowControl.mode;
	l2cap.mtuSize	   = pChan->LocalUsMtu;
	l2cap.txWindowSize = 0;	/* max Rx window size */

	if (pChan->ConfParaRemote.flowControl.maxPDUSize)
	{
		pConfPara->mtuSize = min(pChan->RemoteMtu, pChan->ConfParaRemote.flowControl.maxPDUSize);
	}
	else
	{
		pConfPara->mtuSize = pChan->RemoteMtu;
	}

	l2cap.maxPDUSize   = pConfPara->mtuSize; 	/* incoming PDU size */
	l2cap.conf.mtuSize = pConfPara->mtuSize;

	if (pConfPara->validMask.FlushTO)
	{
		l2cap.conf.flushTO = pConfPara->FlushTO;
	}
	else
	{
		l2cap.conf.flushTO = 0xffff;
	}

	pflow = &l2cap.conf.flow;
	if (pConfPara->validMask.Qos)
	{
		pflow->flags		  = pConfPara->flowSpec.flags;
		pflow->serviceType	  = pConfPara->flowSpec.serviceType;
		pflow->tokenRate	  = pConfPara->flowSpec.tokenRate;
		pflow->tokenBucket	  = pConfPara->flowSpec.tokenBucket;
		pflow->peakBandwidth  = pConfPara->flowSpec.peakBandwidth;
		pflow->latency		  = pConfPara->flowSpec.latency;
		pflow->delayVariation = pConfPara->flowSpec.delayVariation;
	}
	else
	{
		pflow->flags		  = 0;
		pflow->serviceType	  = 0x01; /* best effort */
		pflow->tokenRate	  = 0;
		pflow->tokenBucket	  = 0;
		pflow->peakBandwidth  = 0;
		pflow->latency		  = 0xffffffff;
		pflow->delayVariation = 0xffffffff;
	}

	pflc = &l2cap.conf.flc;

	pflc->txWindowSize = 0;
	pflc->maxTransmit = 0;
	pflc->retransmissionTimeout = 0;
	pflc->monitorTimeout = 0;
	pflc->maxPDUSize = pConfPara->mtuSize;

	if(usQueueID == gattQueueID)
	{
		gattHandleL2cConnectActInd(cid, status, pConfPara->mtuSize);
	}
	else if (usQueueID == sdpQueueID)
	{
		sdpHandleL2cConActInd(cid, status, pConfPara->mtuSize);
	}
    else if(usQueueID < OS_FIRST_QUEUE_ID)
	{
		blueAPI_Send_L2cConActInd(cid, pChan->LocalUsMtu, pChan->RemoteMtu, DownstreamPoolID, usQueueID, pChan->pHciDesc->remote_bd);
	}
}


/**
* @brief  send l2cap connect confirm
*
* @param  usQueueID: 
* @param  remoteBd
* @param  channel
* @param  reqId
* @param  cid
* @param  status
*
* @return  
*
*/
void l2cSendL2C_CON_CONF(uint16_t usQueueID, TBdAddr remote_bd, uint16_t reqId, uint16_t cid, uint16_t status)
{
	if(usQueueID == gattQueueID)
	{
		gattHandleL2cConnectConf(reqId, cid, status);
	}
	else if(usQueueID == sdpQueueID)
	{
		sdpHandleL2cConConf(remote_bd, cid, status);
	}
    else if(usQueueID < OS_FIRST_QUEUE_ID)
	{
		blueAPI_Send_L2cConRsp(cid, usQueueID, remote_bd, status);
	}
}

/**
* @brief  remove pending l2cap channel
*
* @param  pChan: 
* @param  pPar
*
* @return  
*
*/
void l2cCallbackRemovePending(P_L2CAP_CHANNEL pChan, PCVOID pPar)
{
	L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_TRACE,
	                    "l2cCallbackRemovePending() chan 0x%X HCIhandle 0x%x",
						pChan->LocalCid);
	assert(pChan != NULL);

	if(pChan != pPar)
	{                     /* not the new one */
		if(pChan->role == originate) 
		{  /* only if we are the initior of the connection */
			l2cSendL2C_DISC_IND(pChan, L2CAP_NO_CAUSE | L2CAP_ERR);
		}
		(void)l2cDeleteChannel(pChan, FALSE);
	}
}

/**
* @brief  create acl link
*
* @param  pChan: 
*
* @return  
*
*/
void l2cCreateHCIConnection(P_L2CAP_CHANNEL pChan)
{
	L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_TRACE,
	                      "onL2C_CON_REQinCLOSED() HCI state %d", pChan->pHciDesc->state);

	switch(pChan->pHciDesc->state)
	{
	case aclStateAllocated:	/*ACL Link have not created*/
		pChan->pHciDesc->state = aclStateOpening;
		hciHandleUpL2cConnectReq(pChan->pHciDesc->remote_bd);
		break;

	case aclStateOpening:	/*already opening, nothing todo*/
		break;

	case aclStateInuse:		/*ACL link created, just create l2cap link*/
		l2cChangeState(pChan, l2cStateWaitForL2CAP_CONNECTION_RESPONSE);
		break;

	case aclStateClosePending:
		l2cSearchBdCallback(pChan->pHciDesc->remote_bd, l2cCallbackRemovePending, pChan);    /* remove all channels waiting on ACT_IND */
		pChan->pHciDesc->state = aclStateOpening;
		break;

	default:
	  assert(FALSE);
	  break;
	}
}


/**
* @brief  send listen req to l2cap
* 
* @param psm: Protocol Service Multiplexer to listen for
* @param listenerQueue: where L2cap sends messages for the listend psm
* @param action: 0: diable 1= enable listen	
*
* @return  init result
*
*/
uint16_t 
l2cUSendListenReq(uint16_t psm, uint8_t listenerQueue, uint8_t action)
{
	uint16_t descIdx;
    uint16_t foundSlot = otp_str_data.gEfuse_UpperStack_s.num_data_psm;
    uint16_t freeSlot  = otp_str_data.gEfuse_UpperStack_s.num_data_psm;

	uint16_t status = 0;

	L2CAP_TRACE_PRINTF_2(L2CAP_TRACE_MASK_TRACE,
	                        "L2C-> registering PSM %d with queue 0x%X",
	                        psm, listenerQueue);

	for(descIdx = 0; descIdx < otp_str_data.gEfuse_UpperStack_s.num_data_psm; descIdx++)
	{
	    if(pL2c->uplDesc[descIdx].psm == psm)       /* found the slot */
	    {
	        foundSlot = descIdx;
	    }
	    if(pL2c->uplDesc[descIdx].psm == 0)       /* found a free slot */
	    {
	        freeSlot = descIdx;
	    }
	}

	if(action == 0)      /* deregistration */
	{
	    if(foundSlot != otp_str_data.gEfuse_UpperStack_s.num_data_psm)
	    {
	        pL2c->uplDesc[foundSlot].psm     = 0;
	        pL2c->uplDesc[foundSlot].queueID = 0;
	        L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_TRACE,"L2C-> removed registration");
	    }
	    else
	    {
	        status = L2CAP_ERR_REFUS_INV_PSM;
	    }
	}
	else				/* registration */
	{
	    if(foundSlot != otp_str_data.gEfuse_UpperStack_s.num_data_psm)
	    {
	        L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_ERROR,
	                                "!!L2C-> duplicate Registration");
	        status = L2CAP_ERR_REFUS_INV_PSM;
	    }
	    else if(freeSlot != otp_str_data.gEfuse_UpperStack_s.num_data_psm)
	    {
	        pL2c->uplDesc[freeSlot].psm       = psm;
	        pL2c->uplDesc[freeSlot].queueID   = listenerQueue;
	    }
	    else
	    {
	        L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_ERROR,
	                                "!!L2C-> No free Slot for Registration");
	        status = L2CAP_ERR_REFUS_NO_RESOURCE;
	    }
	} /* registration */

    return status;
}

/**
* @brief  send l2cap connect request
*
* @param  targetQueue:
* @param  SrcTaskName: 
* @param  remoteBd
* @param  psm
* @param  uuid
* @param  frameSize
* @param  pConfPara
*
* @return  
*
*/
void 
l2cUSendConReq(uint8_t RequesterQueue, uint8_t * remoteBd, uint16_t psm, uint16_t uuid, uint16_t frameSize, LP_L2C_Conf_para pConfPara)
{
    P_L2CAP_CHANNEL  pNewChan;
    uint16_t reqId = pConfPara->reqId;

#if F_BT_L2C_ENHANCED_FEATURE_SUPPORT
    uint16_t mode  = pConfPara->flowControl.mode;
#endif

    L2CAP_TRACE_PRINTF_4(L2CAP_TRACE_MASK_TRACE,
                                  "L2C-> L2C_CON_REQ  bd %s psm %d ReqId %d fsize %x",
                                  TRACE_BDADDR1(L2CAP_TRACE_MASK_TRACE, remoteBd),
                                  psm, reqId, 
                                  frameSize);

#if CHECK_API_PARAM
    if (mode == 0)
    {
        l2cSendL2C_CON_CONF(RequesterQueue, remoteBd, reqId, 0 /* cid */,
              L2CAP_ERR_ILLEGAL_PARAMETER | L2CAP_ERR);
        osBufferRelease((PVOID)pConfPara);
        return;
    }

    if (l2cCheckConfPara(pConfPara) == FALSE)
    {
        l2cSendL2C_CON_CONF(RequesterQueue, remoteBd, reqId, 0 /* cid */,
              L2CAP_ERR_ILLEGAL_PARAMETER | L2CAP_ERR);
        osBufferRelease((PVOID)pConfPara);
        return;
    }
#endif
    /* check for valid PSM, must be odd and least significant bit of most sig octet must be 0 */
	if((psm & 0x0101) == 0x01)           
	{
#if F_BT_L2C_ENHANCED_FEATURE_SUPPORT
		P_L2CAP_CHANNEL_EXT pExtF = (P_L2CAP_CHANNEL_EXT)0;
		if (mode & (L2C_MODE_ENHANCED_RETRANSMISSION | L2C_MODE_STREAMING))
		{
			pExtF = l2cGetFreeExtFData();
			if (pExtF == (P_L2CAP_CHANNEL_EXT)0)
			{
				l2cSendL2C_CON_CONF(RequesterQueue, remoteBd, reqId, 0 /* cid */,
				  L2CAP_ERR_REFUS_NO_RESOURCE | L2CAP_ERR);
				osBufferRelease((PVOID)pConfPara);
				return;
			}
		}
#endif

		/* check if we already have a connection to that device */
		pNewChan = l2cChannelCreate(0, /*lint -e(545)*/ (LPCBdAddr)remoteBd, BT_CONNECTION_TYPE_BR_ACL);

		if(pNewChan != NULL ) 
		{
#if F_BT_L2C_ENHANCED_FEATURE_SUPPORT   //champion
			pNewChan->pExtF = pExtF;
#endif
			l2cSendL2C_CON_CONF(RequesterQueue, remoteBd, reqId, pNewChan->LocalCid /* cid */, 0);
			l2cInsertChannel(pNewChan);
			pNewChan->role = originate;	/* in this case we are the originator */
			pNewChan->psm  = psm;		/* store PSM for this request for later use */
			pNewChan->uuid = uuid;  	/* store UUID for this request for SecMan */
			pNewChan->usQueueID     = RequesterQueue;
			pNewChan->LocalDsMtu    = frameSize;
			pNewChan->MaxDsBuffer   = pConfPara->maxDsBuffer;
			pNewChan->ConfParaLocal = *pConfPara;

			memcpy(pNewChan->pHciDesc->remote_bd, remoteBd, sizeof(TBdAddr)); 	 

			if (pNewChan->State == l2cStateClosed)
			{
				l2cCreateHCIConnection(pNewChan);
			}
		}
		else 
		{
			l2cSendL2C_CON_CONF(RequesterQueue, remoteBd, reqId, 0 /* cid */,
			  L2CAP_ERR_REFUS_NO_RESOURCE | L2CAP_ERR);

#if F_BT_L2C_ENHANCED_FEATURE_SUPPORT
			if (pExtF != NULL)
			{
				pExtF->Used = FALSE;
			}
#endif
		}
    }
    else  /* it is an invalid or unsupported PSM */
    {
		l2cSendL2C_CON_CONF(RequesterQueue, remoteBd, reqId, 0 /* cid */,
		 				 L2CAP_ERR_REFUS_INV_PSM | L2CAP_ERR);
    }
    if (pConfPara)
	{
		osBufferRelease((PVOID)pConfPara);
	}
}

/**
* @brief  l2cap handle disconnect request from up layer
* 
* @param  cid
* @param  holdLink:  if disconnect immediatly
*
*/
void l2cHandleBTG_DISC_REQ(uint16_t cid, BOOL holdLink)
{
    P_L2CAP_CHANNEL pChan;

    L2CAP_TRACE_PRINTF_2(L2CAP_TRACE_MASK_TRACE,
                            "L2C-> handle_BTG_DISC_REQ lcid 0x%X HoldLink %d",
                            cid,
                            holdLink);

    pChan = l2cSearchLcid(cid);

	if(pChan != NULL) 
	{
		pChan->ACLDiscImmediate = holdLink == 0 ? TRUE : FALSE;  /* shall delay the disconnect ?? */

		switch (pChan->State)
		{
		case l2cStateOpen:
			l2cSendL2CAP_DISCONNECTION_REQUEST(pChan);
			l2cChangeState(pChan, l2cStateWaitForL2CAP_DISCONNECTION_RESPONSE);
			break;

		case l2cStateConfig:
			l2cStopCONFIGTimeout(pChan);   /* stop timeout */
			l2cSendL2CAP_DISCONNECTION_REQUEST(pChan);
			l2cChangeState(pChan, l2cStateWaitForL2CAP_DISCONNECTION_RESPONSE);
			break;

		case l2cStateWaitForL2CAP_CONNECTION_RESPONSE:
			l2cTearDownChan(pChan);
			l2cChangeState(pChan, l2cStateClosed);
			break;

		case l2cStateClosed:
			l2cSendL2C_DISC_CONF(pChan->usQueueID, pChan->LocalCid, 0);
			switch(pChan->pHciDesc->state)
			{
			case aclStateOpening:
			  pChan->pHciDesc->state = aclStateClosePending;
			  break;

			default:
			  L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_TRACE,
			                          "onBTG_DISC_REQinCLOSED aclState %d",
			                          pChan->pHciDesc->state);
			  assert(FALSE);
			  break;
			}
			break;

		default:
			break;
		}
    }
}

/**
* @brief  handle l2cap connect response
* 
* @param  cid: Identifier for this connection
* @param  status: result of the connect indication
* @param  p: protocol specific part
*
* @return  
*
*/
void l2cHandleL2C_CON_RESP(uint16_t cid, uint16_t status, PBtConRespPSpecifc p)
{
    LP_L2C_Conf_para pConfPara = p->l2cap.pConfPara;
    P_L2CAP_CHANNEL pChan;

    L2CAP_TRACE_PRINTF_4(L2CAP_TRACE_MASK_TRACE,
                            "L2C-> L2C_CON_RESP lcid 0x%X status 0x%X details 0x%X mode %4.4x",
                            cid,
                            status, p->l2cap.statusDetails,
                            (pConfPara!=NULL)?pConfPara->flowControl.mode:0);

    pChan = l2cSearchLcid(cid);
    if (pChan == (P_L2CAP_CHANNEL)0)
    {
		assert(FALSE);
		return;
    }

    if (status == L2CAP_CONNECTION_ACCEPT)
    {
#if CHECK_API_PARAM     
        if (pConfPara == (LP_L2C_Conf_para)0 ||
            l2cCheckConfPara(pConfPara) == FALSE)
        {
            if (pConfPara != (LP_L2C_Conf_para)0)
        	{
          		osBufferRelease((PVOID)pConfPara);
        	}
            return;
        }
#endif			
#if F_BT_L2C_ENHANCED_FEATURE_SUPPORT /*won't have this mode*/
        if (pConfPara->flowControl.mode & (L2C_MODE_ENHANCED_RETRANSMISSION | L2C_MODE_STREAMING))
        {
#if F_BT_L2C_ENHANCED_FEATURE_SUPPORT	//champion
            P_L2CAP_CHANNEL_EXT pExtF;

            pExtF = l2cGetFreeExtFData();
            if (pExtF == (P_L2CAP_CHANNEL_EXT)0)
            {
                osBufferRelease((PVOID)pConfPara);

                l2cSendL2CAP_CONNECTION_RESPONSE(pChan,  NULL, pChan->LocalCid, pChan->RemoteCid, L2CAP_ERR_REFUS_NO_RESOURCE, 0);
                (void)l2cDeleteChannel(pChan, FALSE);

                return;
            }

            pChan->pExtF = pExtF;
#else	
       		L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_ERROR,
                                                  "!!l2cHandleL2C_CON_RESP--pChan->Mode != L2CAP_MODE_BASIC");
#endif
        }
#endif
        if (l2cCheckFlushTO(pChan, pConfPara->FlushTO))
        {
#if F_BT_L2C_ENHANCED_FEATURE_SUPPORT  /*just support basic mode*/       
            if (pConfPara->flowControl.mode & L2C_MODE_STREAMING && pConfPara->flowControl.mode != L2C_MODE_STREAMING)
            {
                pConfPara->flowControl.mode &=~L2C_MODE_STREAMING;
                pConfPara->FlushTO           = FLUSHTIMEOUT_DEFAULT;
            }
            else
#endif				
            {
                osBufferRelease((PVOID)pConfPara);

                l2cSendL2CAP_CONNECTION_RESPONSE(pChan,  NULL, pChan->LocalCid, pChan->RemoteCid, L2CAP_ERR_REFUS_NO_RESOURCE, 0);
                (void)l2cDeleteChannel(pChan, FALSE);

                return;
            }
        }

        pChan->MaxDsBuffer = pConfPara->maxDsBuffer;
    }

    if (pConfPara != (LP_L2C_Conf_para)0)
    {
		memcpy((PVOID)&pChan->ConfParaLocal, (PVOID)pConfPara, sizeof(T_L2C_Conf_para));
		osBufferRelease((PVOID)pConfPara);
    }

    switch (pChan->State)
    {
	case l2cStateWaitForL2C_CON_RESP :
		l2cSendL2CAP_CONNECTION_RESPONSE(pChan,  NULL, pChan->LocalCid, pChan->RemoteCid,
		                                 status,
		                                 (uint16_t)(status == L2CAP_ERR_PENDING ? p->l2cap.statusDetails : 0));

        if(status == 0) 
		{                /* connect succedded */
            if (pChan->L2capInformationRequestSent)
            {
                pChan->SendL2capConfigureRequest = TRUE;  /* wait for response before sending new request */
            }
            else
            {
                l2cGetNextCurrentMode(pChan);            /* get first mode for negotiation */
                if (pChan->CurrentMode != 0)
                {
                    l2cSendL2CAP_CONFIGURE_REQUEST(pChan);
                    l2cChangeState(pChan, l2cStateConfig);
                }
#if F_BT_L2C_ENHANCED_FEATURE_SUPPORT					
                else  /* mode is not allowed */
                {
                    l2cSendL2CAP_DISCONNECTION_REQUEST(pChan);
                    l2cChangeState(pChan, l2cStateWaitForL2CAP_DISCONNECTION_RESPONSE);
                }
#endif
            }
        }
        else if(status != L2CAP_ERR_PENDING) 
		{
            l2cChangeState(pChan, l2cStateClosed);
        }
        break;

	default:
        break;
    }
}

/**
* @brief	send l2cap disconnect confirm
*
* @param	usQueueID: upstream queue
* @param	LocalCid: 
* @param	result: disconnect reason
*
* @return
*
*/
void l2cSendL2C_DISC_CONF(uint16_t usQueueID, uint16_t LocalCid, uint16_t result)
{
	if(usQueueID == gattQueueID)
	{
		gattHandleDisconnected(LocalCid, result, TRUE);
	}
    else if (usQueueID == sdpQueueID)
	{
		sdpHandleL2cDiscConf(LocalCid, result);
	}
    else if(usQueueID < OS_FIRST_QUEUE_ID)
    {
        blueAPI_Send_L2cDiscRsp(usQueueID, LocalCid, result);
	}
}

/**
* @brief  handles a Data request packet from the upper layer 
*
* @param  pmsg: 
* @param  prio
*
* @return  
*
*/
void l2cHandleL2C_DATA_REQ(MESSAGE_P  pmsg, BOOL prio)
{
    P_L2CAP_CHANNEL pChan;
    BOOL releasePak = FALSE;
	

    L2CAP_TRACE_PRINTF_2(L2CAP_TRACE_MASK_TRACE,
                            "L2C-> handle_L2C_DATA_REQ() lcid 0x%X length %d",
                            pmsg->MData.DataCBChan.Channel, pmsg->MData.DataCB.Length);

    pChan = l2cSearchLcid(pmsg->MData.DataCBChan.Channel);

    if(pChan != NULL)
    {
        assert(pmsg->MData.DataCBChan.Length <= (pChan->RemoteMtu + pChan->ExtFLength));

        if (pChan->State == l2cStateOpen || pChan->State == l2cStateConfig)
		{
#if F_BT_L2C_ENHANCED_FEATURE_SUPPORT
            if (pChan->Mode != L2CAP_MODE_BASIC)
            {
                l2cHandleDownstreamData(pChan, pmsg);
            }
            else
#endif				
            {
                if (FALSE == gL2cDsFragmentationSupport)
                {
                    l2cSendL2CAP_PDU(pChan->pHciDesc->handle, pmsg,
                                        pmsg->MData.DataCB.Length,
                                        pChan->RemoteCid,
                                        TRUE);
                }
                else
                {
                    pmsg->MData.DataCBChan.Channel = pChan->RemoteCid;
                    osMessageSend((uint16_t) ((prio) ? pChan->pHciDesc->dsPrioBufQueue : pChan->pHciDesc->dsBufQueue), pmsg);
                    l2cFragmentDATA_REQ(pChan->pHciDesc);
                }
            }
        }
        else
		{
            L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_ERROR,
                                    "!!L2C handle_L2C_DATA_REQ channel not OPEN!");
            releasePak = TRUE;
        }
    }
    else 
	{      /* no channel exists */
        L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_ERROR,
                                "!!L2C handle_L2C_DATA_REQ no channel discarding data!");
        releasePak = TRUE;
    }
    if((releasePak == TRUE) && (pmsg->MData.DataCBChan.Flag & DATA_CB_RELEASE))
    {
        osBufferRelease(pmsg->MData.DataCBChan.BufferAddress);
    }
} /* handle_L2C_DATA_REQ() */

/**
* @brief  hci connect active callbck indicate
*
* @param  pChan
* @param  pPar
*
* @return  
*
*/
void l2cCallbackOnHCI_ACT_IND(P_L2CAP_CHANNEL pChan, PCVOID pPar)
{
	ThciConActInd * pCon_act_ind = (ThciConActInd *)pPar;

	L2CAP_TRACE_PRINTF_2(L2CAP_TRACE_MASK_TRACE,
	                        "l2cCallbackOnHCI_ACT_IND() chan 0x%X HCIhandle 0x%x",
	                        pChan->LocalCid, pCon_act_ind->handle);
	assert(pChan != NULL);
	
	/**  we found the channel, and HCI is not already in use 
	  * (this shall prevent that all chanels are discarded on an HCI_CON_ACT_IND
	  * with same bd as existing channel and negativ status) bug#000324 */
	if(pChan != NULL && pChan->pHciDesc->state != aclStateInuse)
	{ 
		if(pCon_act_ind->status) 
		{          /* acl connect fail*/
			if(pChan->role == originate)
			{  /* only if we are the initior of the connection */
				l2cSendL2C_DISC_IND(pChan, pCon_act_ind->status);
			}
			(void)l2cDeleteChannel(pChan, TRUE);
		}
		else 
		{
			pChan->pHciDesc->handle = pCon_act_ind->handle;                /* now we have got an HCI handle */

			if(pChan->pHciDesc->state == aclStateClosePending)
			{
				if(pChan->role == originate)
				{  	/* only if we are the initior of the connection */
					l2cSendL2C_DISC_IND(pChan, L2CAP_NO_CAUSE | L2CAP_ERR);
				}
				(void)l2cDeleteChannel(pChan, FALSE);
			}
			else 
			{
				pChan->pHciDesc->state  = aclStateInuse;

				if (pChan->State == l2cStateClosed)
				{
					assert(pChan->role != undef);

					if(pChan->role == originate) 
					{                /* only if we are the initior of the connection */
					    l2cChangeState(pChan, l2cStateWaitForL2CAP_CONNECTION_RESPONSE);
					}
#if F_BT_L2C_ENHANCED_CONFORMANCE					
					else if(pChan->role == pingreq)
					{
					    l2cSendL2CAP_ECHO_REQUEST(pChan);
					}
#endif					
					else if(pChan->role == inforeq)
					{
					    l2cSendL2CAP_INFORMATION_REQUEST(pChan, L2CAP_INFOTYPE_EXTENDED_FEATURES_SUPPORT);
					    pChan->role = undef;
					}
					else if(pChan->role == terminate)
					{
						l2cSendL2CAP_INFORMATION_REQUEST(pChan, L2CAP_INFOTYPE_EXTENDED_FEATURES_SUPPORT);
						l2cStartCONFIGTimeout(pChan, L2CAP_ERTX_TIME);   /* start timeout */
					}
				}
			}
		}
	} /* channel found */
}

/**
* @brief  l2cap handle hci connect active indicate
*
* @param  status: status
* @param  handle: ACL handle
* @param  bd: bdaddr
*
* @return  
*
*/
void l2cHandleHciConActInd(uint16_t status, uint16_t handle, uint8_t * bd)
{
	ThciConActInd  TConActInd;
	memcpy(TConActInd.bd, bd, BD_ADDR_SIZE);
    TConActInd.handle = handle;
    TConActInd.status = status;

	l2cSearchBdCallback(bd, l2cCallbackOnHCI_ACT_IND, &TConActInd);
}

/**
* @brief  l2cap handle hci connection indicate
* 
* @param  bd
* @param  devClass
* @param  linktype
*
* @return  
*
*/
void l2cHandleHciConInd(uint8_t * bd, uint8_t * devClass, uint8_t linktype)
{
	P_L2CAP_CHANNEL pNewChan;

	L2CAP_TRACE_PRINTF_5(L2CAP_TRACE_MASK_TRACE,
								  "L2C-> HCI_CON_IND bd %s devclass %x %x %x linktype %x",
								  TRACE_BDADDR1(L2CAP_TRACE_MASK_TRACE, bd),
								  devClass[0],
								  devClass[1],
								  devClass[2],
								  linktype);
	
	pNewChan = l2cChannelCreate(0, /*lint -e(545)*/ (LPCBdAddr)bd, BT_CONNECTION_TYPE_BR_ACL);
	if( pNewChan != NULL) 
	{
		l2cInsertChannel(pNewChan);
		pNewChan->role = terminate; 				/* in this case we are the terminator */
		hciCommandBDAddrByteParameter(HCI_ACCEPT_CONNECTION_REQUEST, bd, 1 /* first remain slave */);		
	}else
	{
		hciCommandBDAddrByteParameter(HCI_REJECT_CONNECTION_REQUEST,  bd, HCI_ERR_HOST_REJECTED_0D);
	}
	hciLaterEntry();
}

/**
* @brief	 l2cap handle hci connect confirm
*		sucess nothing to do
*
* @param	bd: 
* @param	status:
*
* @return
*
*/
void l2cHandleHciConCnf(uint8_t * bd, uint16_t status)
{
    P_L2CAP_CHANNEL pChan;

    L2CAP_TRACE_PRINTF_2(L2CAP_TRACE_MASK_TRACE,
                            "L2C-> HCI_CON_CNF Status %x bd %s",
                            status, TRACE_BDADDR1(L2CAP_TRACE_MASK_TRACE, bd));

    pChan = l2cSearchBdCallback(bd, NULL, NULL);

    assert(pChan != NULL);

    if(pChan != NULL) 
    {                
        if(status) 
        {     
            /* we are allways the initior of the connection */
            l2cSendL2C_DISC_IND(pChan, status );
            (void)l2cDeleteChannel(pChan, TRUE);
        }
    }
}

/**
* @brief  l2cap channel disconnected
* 
* @param  pChan
*
* @return  
*
*/
void l2cTearDownChan(P_L2CAP_CHANNEL pChan)
{
	pChan->status = 0;
	if(pChan->ACLDiscImmediate == FALSE || pChan->pHciDesc->uses != 1)
	{
		l2cSendL2C_DISC_CONF(pChan->usQueueID, pChan->LocalCid, pChan->status);
	}
	else
	{
		pChan->OpenL2cDiscConf = TRUE;	/*need to send disconnect req to hci layer*/
	}
}

/**
* @brief  disconnect hci acl link
*
* @param  remoteBd: pointer to the HCI descriptor 
				may be NULL than CloseId is used instead
* @param  CloseId: used to identify the HCI descriptor
*
* @return  
*
*/
void l2cRemoveHCI(P_ACLPOOLDESC pHciDesc, uint8_t CloseId)
{
    if(pHciDesc == NULL)    /* we do not have the descriptor yet */
    {
		pHciDesc = l2cSearchHciCloseId(CloseId);
    }

    if(pHciDesc != NULL)
    {
        L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_TRACE,
                                "l2c_remove_HCI() Hci channel removed");

        L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_TRACE, "L2C<- HCI_DISC_REQ");

        btgSendDiscReq(hciQueueID, pHciDesc->handle, HCI_ERR_OTHER_END_TERMINATE_13, FALSE);
        l2cFreeHciDesc(pHciDesc);
    }
    else
    {
        L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_TRACE,
                                "l2c_remove_HCI() channel in use again");
    }
}
