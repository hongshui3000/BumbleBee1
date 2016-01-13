/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       gatt_br.c
* @brief     gatt BR/EDR specific routines
* @details   
*
* @author   	gordon
* @date      	2015-07-09
* @version	v0.1
*/

#include <flags.h>
#include <gattdef.h>
#include <os_pool.h>
#include <btcommon.h>
#include <l2c_api.h>
#include <btsend.h>
#include <sdp_code.h>
#include <sdplib.h>
#include <sdp_api.h>
#include <blueapi_api.h>
#include <upper_stack_global.h>

#define TRACE_MODULE_ID     MID_BT_L2C
/**
* @brief  send L2C_CON_REQ
*
* @param  pL2CChannel:
*
* @return  
*
*/
int gattSendL2C_CON_REQ(PGATTL2CChannel pL2CChannel )
{
	LP_L2C_Conf_para pConfPara;
	int              iReturn;

	  GATT_TRACE_PRINTF_3(GATT_TRACE_MASK_TRACE,
	                 "GATT: gattSendL2C_CON_REQ / reqId=%d, BD=%s PSM=0x%x",
	                  pL2CChannel->wNbr,
	                  TRACE_BDADDR1(GATT_TRACE_MASK_TRACE, pL2CChannel->RemoteBd),
	                  pL2CChannel->psm);

	/* prepare L2C configuration parameters */
	pConfPara = commonPrepareL2cConfPara(pL2CChannel->wMTUSizeMax,pL2CChannel->wMTUSizeMax, pL2CChannel->wNbr);
	if (pConfPara != (LP_L2C_Conf_para)0)
	{
		l2cUSendConReq(gattQueueID, pL2CChannel->RemoteBd, pL2CChannel->psm, UUID_GATT /* UUID */, 0 /* frameSize */, pConfPara);
		gattL2CChangeState( pL2CChannel, gattL2CStateConConfPending );
		iReturn = 0;
	}
	else
	{
		iReturn = -1;
	}

	return( iReturn );
}

/**
* @brief  send L2C_CON_RESP
*
* @param  pL2CChannel:
* @param  wStatus:
* @param  cid:
*
* @return  
*
*/
int gattSendL2C_CON_RESP( PGATTL2CChannel pL2CChannel,
                                                      uint16_t wStatus, uint16_t cid )
{
	TBtConRespPSpecifc ConRespExt;
	int                iReturn   = 0;
	LP_L2C_Conf_para   pConfPara = NULL;

	GATT_TRACE_PRINTF_1(GATT_TRACE_MASK_TRACE,
	                              "GATT: gattSendL2C_CON_RESP / cid=0x%x", cid );

	if ( wStatus == L2CAP_CONNECTION_ACCEPT )  /* => pL2CChannel != NULL */
	{
		/* prepare L2C configuration parameters */
		pConfPara = commonPrepareL2cConfPara(pL2CChannel->wMTUSizeMax,pL2CChannel->wMTUSizeMax, pL2CChannel->wNbr);
		if (pConfPara == (LP_L2C_Conf_para)0)
		{
			wStatus = L2CAP_ERR_REFUS_NO_RESOURCE;
			iReturn = -1;
		}
	}

	/* send rsp to L2CAP */
	ConRespExt.l2cap.statusDetails = 0;
	ConRespExt.l2cap.pConfPara     = pConfPara;

	l2cHandleL2C_CON_RESP(cid, wStatus, &ConRespExt);                                 
	return( iReturn );
}

/**
* @brief  send L2C_DISC_REQ
*
* @param  pL2CChannel:
* @param  wStatus
*
* @return  
*
*/
void gattSendL2C_DISC_REQ( PGATTL2CChannel pL2CChannel, uint16_t wStatus )
{
	GATT_TRACE_PRINTF_2(GATT_TRACE_MASK_TRACE,
	                          "GATT: gattSendL2C_DISC_REQ / cid=0x%x, cause=0x%x",
	                          pL2CChannel->cid, wStatus );

	gattL2CChangeState( pL2CChannel, gattL2CStateDiscPending );
	l2cHandleBTG_DISC_REQ(pL2CChannel->cid, FALSE);
}

/**
* @brief  send l2cap data request
*
* @param  pL2CChannel:
* @param  pBuffer
* @param  wLength
* @param  wRxOffset
*
* @return  
*
*/

void gattSendL2C_DATA_REQ( PGATTL2CChannel pL2CChannel,
                           uint8_t * pBuffer, uint16_t wLength, uint16_t wRxOffset )
{
	MESSAGE_T Message;
    
	Message.MData.DataCBChan.BufferAddress  = pBuffer;
	Message.MData.DataCBChan.Offset         = wRxOffset;
	Message.MData.DataCBChan.Flag           = DATA_CB_RELEASE;
	Message.MData.DataCBChan.Channel        = pL2CChannel->cid;
	Message.MData.DataCBChan.Length         = wLength;

	l2cHandleL2C_DATA_REQ(&Message, FALSE);
}

void gattHandleL2cConnectConf(uint16_t reqId, uint16_t cid, uint16_t status)
{
	PGATTL2CChannel pL2CChannel;

    GATT_TRACE_PRINTF_3(GATT_TRACE_MASK_TRACE,
                            "GATT:l2cSendL2C_CON_CONF   / reqId=%d, num_link_don=0x%x, num_link_doff=0x%x",
                            reqId, 
                            otp_str_data.gEfuse_UpperStack_s.num_link_don, 
                            otp_str_data.gEfuse_UpperStack_s.num_link_doff);
    if(reqId>= 0x10)
    {
        pL2CChannel = &pGATT->pL2CChannelDoff[reqId-0x10];
    }
    else
    {
        pL2CChannel = &pGATT->pL2CChannelDon[reqId];
    }


	if ( status == L2CAP_NO_CAUSE )
    {
        pL2CChannel->cid = cid;
        gattL2CChangeState( pL2CChannel, gattL2CStateConnecting );
    }
    else
    {
        /* local L2CAP error */
        gattL2CChangeState( pL2CChannel, gattL2CStateIdle );
    }

    /* inform LL interface */
    gattLLConnectConf( pL2CChannel, status );
}

void gattHandleL2cConnectInd(uint16_t LocalCid, uint16_t psm, uint8_t * remote_bd)
{
    PGATTL2CChannel    pL2CChannel;
	uint16_t			   wStatus	 = L2CAP_ERR_REFUS_NO_RESOURCE;
				
		GATT_TRACE_PRINTF_3(GATT_TRACE_MASK_TRACE,
					   "GATT: gattHandleL2C_CON_IND / reqId=%d, psm=0x%x, cid=0x%x",
					   TRACE_BDADDR1(GATT_TRACE_MASK_TRACE, remote_bd),
					   psm,
					   LocalCid);
	
	if ( (pL2CChannel = gattL2CChannelAllocate( FALSE )) != NULL )
	{
		pL2CChannel->cid = LocalCid;
		pL2CChannel->psm = psm;
		memcpy( pL2CChannel->RemoteBd, remote_bd, sizeof(TBdAddr) );

		pL2CChannel->bdType = BLUEFACE_BDTYPE_BR_EDR;

		wStatus = L2CAP_CONNECTION_ACCEPT;
		gattL2CChangeState( pL2CChannel, gattL2CStateConnecting );
	
	}
	
	if ( wStatus == L2CAP_CONNECTION_ACCEPT )
	{
		/* inform LL interface */
		gattLLConnectInd( pL2CChannel );
	}
	else
	{
		/* reject call */
		gattSendL2C_CON_RESP( pL2CChannel, wStatus, LocalCid );

		/* cleanup, no DISC_IND !!! */
		gattL2CChannelFree( pL2CChannel );
	}
}

void gattHandleL2cConnectActInd(uint16_t cid, uint16_t status, uint16_t mtuSize)
{
    PGATTL2CChannel pL2CChannel;

    GATT_TRACE_PRINTF_2(GATT_TRACE_MASK_TRACE,
    				"GATT: gattHandleL2C_CON_ACT_IND / status=0x%x, cid=0x%x",
    				status, cid);

    if ( status == (L2CAP_ERR | L2CAP_ERR_PENDING) )
    {
        GATT_TRACE_PRINTF_1(GATT_TRACE_MASK_TRACE,
        "GATT: cid=0x%x connection pending", cid);
    }
    else
    {

        pL2CChannel = gattL2CChannelFindHandle( FALSE, cid );
        if ( pL2CChannel != NULL )
        {
            if ( status == L2CAP_NO_CAUSE )
            {
                /* OK, connected */
                gattL2CChangeState( pL2CChannel, gattL2CStateConnected );
                pL2CChannel->wMTUSize = mtuSize;

                /* inform LL interface */
                gattLLConnected( pL2CChannel, 0 );
            }
        }
    }
}

void gattHandleL2cDisconnectInd( uint16_t LocalCid, uint16_t status)
{
	PGATTL2CChannel   pL2CChannel;
	
	GATT_TRACE_PRINTF_2(GATT_TRACE_MASK_TRACE,
					   "GATT: gattHandleL2C_DISC_IND / status=0x%x, lcid=0x%x",
					   status, LocalCid);
	
	pL2CChannel = gattL2CChannelFindHandle( FALSE,LocalCid );
	if ( pL2CChannel != NULL )
	{
		gattL2CChangeState( pL2CChannel, gattL2CStateDisconnecting );

		/* inform LL interface */
		gattLLDisconnectInd( pL2CChannel, status );
	}
	else
	{
		/* oops, what is this ??? send rsp anyhow */
        l2cHandleL2C_DISC_RESP(LocalCid);
	}
}

/**
* @brief  free deferred msg descriptor
*
* @param  pDeferredMsg:
*
* @return  
*
*/
void gattDeferredMsgFree( PGATTDeferredMsg pDeferredMsg )
{
	osQueueDelete( &pGATT->DeferredMsgQueue, pDeferredMsg );
	osQueueIn( &pGATT->DeferredMsgQueueFree, pDeferredMsg );
}

/**
* @brief  send GATT_SERVICE_REGISTER_CONF and free message descriptor
*
* @param  pDeferredMsg:
* @param  wCause
*
* @return  
*
*/
void gattSrvRegConfFreeMsg( PGATTDeferredMsg pDeferredMsg, uint16_t wCause )
{
	/* free msg descriptor */
	gattDeferredMsgFree(pDeferredMsg);
	gattSendGATT_SERVICE_REGISTER_CONF(wCause, pDeferredMsg->pGATTService);
}

void gattHandleSDP_CONFIGURE_CONF(uint16_t status, uint32_t reference, uint32_t context)
{
    PGATTDeferredMsg	pDeferredMsg;
	PGATTService		 pGATTService;
	int				 iReturn;
	uint16_t				 wCause = GATT_ERR_OUT_OF_RESOURCE;

	/* restore applic. service register REQ (built-in service: NULL ..) */
	pDeferredMsg = (PGATTDeferredMsg)reference;
	pGATTService = (PGATTService)context;
    
    if ( status == SDP_SUCCESS )
    {
    	/* check if more SDP records must be created */
    	iReturn = gattSDPCreateDes( pGATTService, (PVOID)pDeferredMsg );
    	if ( iReturn > 0 )
    	{
    	  /* yes, wait for next SDP rsp */
    	  return;
    	}
    	else
    	{
    		if ( iReturn == 0 )
    		{
    			/* no more .. */
    			pGATTService->Sdp.State = SdpStateRegistered;
    			wCause = GATT_SUCCESS;

    			/* [ */
    			if ( pDeferredMsg != NULL )
    			{
    				/* send CONF */
    				gattSrvRegConfFreeMsg( pDeferredMsg, wCause );
    			}

    			/* check for more queued reqs .. */
    			iReturn = 0;
    			while ( (iReturn == 0) && (pGATT->DeferredMsgQueue.Count != 0) )
    			{
    				/* application service register req. was received while		*/
    				/* creation of SDP records for previous services was still in */
    				/* progress, continue with application services now .. :		*/
    				pDeferredMsg = (PGATTDeferredMsg)pGATT->DeferredMsgQueue.First;
    				pGATTService = pDeferredMsg->pGATTService;
    				iReturn = gattSDPCreateDes( pGATTService, (PVOID)pDeferredMsg );
    				if ( iReturn > 0 )
    				{
    					/* yes, wait for next SDP rsp */
    					return;
    				}
    				else
    				{
    					if ( iReturn == 0 )
    					{
    						/* no need to create SDP record, send immediate CONF */
    						gattSrvRegConfFreeMsg( pDeferredMsg, wCause );
    					}
    					else
    					{
    						/* error */
    						wCause = GATT_ERR_OUT_OF_RESOURCE;
    					}
    				}
    			}
    		}
    		else
    		{
    			wCause = GATT_ERR_OUT_OF_RESOURCE;
    		}
    	}
    }

    if ( wCause != GATT_SUCCESS )
    {
    	/* XXXXMJMJ what about SDP records that have been created already ?? */
    	/* XXXXMJMJ SDP_RELEASE_REQ with handle=0 ???? */

#if (!GATT_NO_BUILTIN_SERVICE)
    	if ( pDeferredMsg == NULL )
    	{
    		/* built-in services */
    		gattServiceFree( pGATTService );
    	}
#endif

    	/* flush pending requests */
    	while ( pGATT->DeferredMsgQueue.Count != 0 )
    	{
    		pDeferredMsg = (PGATTDeferredMsg)pGATT->DeferredMsgQueue.First;
    		pGATTService = pDeferredMsg->pGATTService;

    		gattSrvRegConfFreeMsg( pDeferredMsg, wCause );
    		gattServiceFree( pGATTService );
    	}

    }

}

void gattSendSDP_CONFIGURE_REQ(uint8_t * pDesBuffer, uint16_t wLength, uint16_t wOffset)
{
	uint32_t context   = ((uint32_t*)pDesBuffer)[0];
	uint32_t reference = ((uint32_t*)pDesBuffer)[1];

    BOOL ret = sdpHandleUpRegReq((uint8_t*)pDesBuffer+wOffset, wOffset);

    if (ret == TRUE)
    {
        gattHandleSDP_CONFIGURE_CONF(0, reference, context);
    }
    else
    {
        osBufferRelease(pDesBuffer);
        gattHandleSDP_CONFIGURE_CONF(SDP_ERR | SDP_INSUFFICIENT_RESOURCES, reference, context);
    }
}

/**
* @brief create SDP record for primary services that are accessible over BR/EDR.
* 
* @param pGATTService
* @param pHandle
*
* @return 
* @retval > 0 : creation of SDP record was initiated.
* @retval  0   : no need to create SDP record.
* @retval < 0 : error.
*
*/
int gattSDPCreateDes( PGATTService pGATTService, PVOID pHandle )
{
	int     i, iCnt, iSize, iReturn;
	uint16_t    wStartHandleSearch, wEndHandleSearch;
	uint8_t *  pList;
	BOOL    Stop;

	iReturn = 0;
	Stop    = FALSE;

	if ( pGATTService->Sdp.Last )   /* nothing more to do ... */
	{
		pGATTService->Sdp.Last = FALSE;
		return( iReturn );
	}

	/* allocate temp. buffer for search result list. large enough */
	/* to store 2 <starthandle,endHandle,128-bit UUID> elements:  */
	iSize = 2 * (2 * sizeof(uint16_t) + UUID_128BIT_SIZE);
	if ( osBufferGet(BTSystemPoolID, iSize,
	                                        (PVOID  *)&pList) != 0 )
	{
		assert(FALSE);
		return( -1 );
	}

	/* use generic search routine: */
	wStartHandleSearch = pGATTService->Sdp.wHandleNext;
	wEndHandleSearch   = pGATTService->wHandleStart +   /* largest handle .. */
	                                  pGATTService->wAttribCnt - 1;

	if ( wStartHandleSearch > wEndHandleSearch )
	{
		/* OK, ready */
	}
	else
	{
		do
		{
			gattAttributesSearch( TRUE, NULL, FALSE, GATT_TYPE_DISCOVERY_PSRV_ALL, TRUE,
			                  &wStartHandleSearch, wEndHandleSearch,
			                  0, NULL,
			                  &iCnt,
			                  pList,
			                  &iSize /* size of list element on return */,
			                  -1 /* forced list element size, -1: not yet defined */);
			if ( iCnt > 0 )
			{
				int     iUUIDSize;
				uint16_t    wUUID16;
				uint8_t *  pUUID128;
				uint16_t    wStartHandleSrv, wEndHandleSrv;
				PAttrib pAttrib;
				uint8_t *  pListEntry = pList;

				if ( iSize == (2*sizeof(uint16_t)+UUID_16BIT_SIZE) )
				{
					iUUIDSize = UUID_16BIT_SIZE;
				}
				else
				{
					iUUIDSize = UUID_128BIT_SIZE;
				}

				/* create SDP for first entry in search result list */
				/* with ATTRIB_FLAG_BR_EDR bit set:                 */
				for ( i = 0; i < iCnt; i++ )
				{
					wStartHandleSrv    = LE_EXTRN2WORD(pListEntry);
					pListEntry        += sizeof(uint16_t);
					wEndHandleSrv      = LE_EXTRN2WORD(pListEntry);

					if ( wEndHandleSrv == 0xFFFF )
					{
						wStartHandleSearch     = wEndHandleSrv;
						pGATTService->Sdp.Last = TRUE;
					}
					else
					{
						wStartHandleSearch = wEndHandleSrv + 1;
					}

					pListEntry        += sizeof(uint16_t);
					wUUID16            = LE_EXTRN2WORD(pListEntry);
					pUUID128           = pListEntry;
					pListEntry        += iUUIDSize;

					if ( (pAttrib = gattHandleToAttribute( wStartHandleSrv )) != NULL )
					{
						if ( pAttrib->wFlags & ATTRIB_FLAG_BR_EDR )
						{
						    uint8_t *      pDesBuffer;
						    uint16_t        wLength;

						    /*--- setup SDP descriptor ---*/
						    if ( iUUIDSize == UUID_16BIT_SIZE )
							{
								wLength = GATT_SDP_BUFFER_SIZE_16;
							}
						    else
							{
								wLength = GATT_SDP_BUFFER_SIZE_128;
							}
						    if ( osBufferGet(BTSystemPoolID, wLength + 2*sizeof(uint32_t),
						                      (PVOID  *)&pDesBuffer) != 0 )
						    {
								assert(FALSE);
								iReturn = -1;
								Stop    = TRUE;
								break;
						    }
                            //XXXXMJMJ cross check with BT_SDP_SERVER_MAX_SERVICES_COUNT ???
						    if ( iUUIDSize == UUID_16BIT_SIZE )
						    {
								wLength  = (uint16_t)sdpCreateDes( &pDesBuffer[2*sizeof(uint32_t)],
								    "[ I<U> I<<UI><UII>> I<U>]",
								    /*1*/ SDP_ATTR_SERVICECLASSIDLIST, wUUID16,
								    /*2*/ SDP_ATTR_PROTOCOLDESCRIPTORLIST, UUID_L2CAP, PSM_ATT,
								                                           UUID_ATT,
								                                           wStartHandleSrv,
								                                           wEndHandleSrv,
								    /*3*/ SDP_ATTR_BROWSEGROUPLIST, UUID_PUBLIC_BROWSE_GROUP
								                     );
						    }
						    else
						    {
								wLength  = (uint16_t)sdpCreateDes( &pDesBuffer[2*sizeof(uint32_t)],
								    "[ I<8U> I<<UI><UII>> I<U>]",
								    /*1*/ SDP_ATTR_SERVICECLASSIDLIST, pUUID128,
								    /*2*/ SDP_ATTR_PROTOCOLDESCRIPTORLIST, UUID_L2CAP, PSM_ATT,
								                                           UUID_ATT,
								                                           wStartHandleSrv,
								                                           wEndHandleSrv,
								    /*3*/ SDP_ATTR_BROWSEGROUPLIST, UUID_PUBLIC_BROWSE_GROUP
								                     );
						    }

						    /* insert application and transaction handle at the start of the message: */
						    ((uint32_t*)pDesBuffer)[0] = (uint32_t)pGATTService;
						    ((uint32_t*)pDesBuffer)[1] = (uint32_t)pHandle;
							
						    /* save ending handle for continued search */
						    pGATTService->Sdp.wHandleNext = wStartHandleSearch;
						    pGATTService->Sdp.State = SdpStateRegistering;
						    gattSendSDP_CONFIGURE_REQ(pDesBuffer, wLength, 2*sizeof(uint32_t));
							
						    iReturn = 1;
						    Stop    = TRUE;
						    break;
					  	}
					}
				}
			}
		}
		while( !Stop && (iCnt > 0) );  /* continue search reusing buffer pList .. */
	}

	osBufferRelease( pList );
	return( iReturn );
}
					   

