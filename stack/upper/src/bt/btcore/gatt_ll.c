/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       gatt_ll.c
* @brief     gatt Logical Link/Lower Layer interfacing
* @details   
*
* @author   	gordon
* @date      	2015-07-09
* @version	v0.1
*/

#include <flags.h>
#include <os_pool.h>
#include <gattdef.h>
#include <attdef.h>
#include <blueapi_api.h>
#include <btcommon.h>
#include <l2c_api.h>
#include <btman.h>
#include <upper_stack_global.h>

#define TRACE_MODULE_ID     MID_BT_GATT

/**
* @brief  gatt link layer connect req
*
* @param  pL2CChannel:
* @param  pConPara
*
* @return  
*
*/
int gattLLConnectReq( PGATTL2CChannel pL2CChannel, PCON_REQ_GATT pConPara)
{
	int iReturn;

	if ( pL2CChannel->bdType & BLUEFACE_BDTYPE_LE_MASK )
	{
		pL2CChannel->OutgoingCall = TRUE;
		iReturn = gattSendLE_CREATE_CONNECTION( pL2CChannel, pConPara );
	}
#if F_BT_BREDR_SUPPORT
	else
	{
		pL2CChannel->psm = PSM_ATT;

		iReturn = gattSendL2C_CON_REQ( pL2CChannel );
	}
#endif
	return( iReturn );
}

/**
* @brief  local confirmation of connection setup
*
* @param  pL2CChannel
* @param  wStatus
*
* @return  
*
*/
void gattLLConnectConf( PGATTL2CChannel pL2CChannel,
                        uint16_t wStatus )
{
	/* send deferred CONF to applic. */
	blueAPI_Handle_GATT_CON_CONF( wStatus, pL2CChannel->cid);

	if ( wStatus != L2CAP_NO_CAUSE )
	{
		/* local L2CAP error, free channel */
		gattL2CChannelFree( pL2CChannel );
	}
}

/**
* @brief  setup incoming connection
*
* @param  pL2CChannel:
*
* @return  
*
*/
void gattLLConnectInd( PGATTL2CChannel pL2CChannel )
{
	gattSendGATT_CON_IND( pL2CChannel->cid, &pL2CChannel->RemoteBd,
	                                              pL2CChannel->bdType);
}

/**
* @brief  applic. incoming connection response
*
* @param  pL2CChannel:
* @param  wStatus
*
* @return  
*
*/
void gattLLConnectResp( PGATTL2CChannel pL2CChannel, uint16_t wStatus )
{
	if ( pL2CChannel->bdType & BLUEFACE_BDTYPE_LE_MASK )
	{
		/* state must be gattL2CStateConnected disregarding wStatus value, */
		/* needed for gattLLDisconnectReq() call:                          */
		gattL2CChangeState( pL2CChannel, gattL2CStateConnected );

		if ( (wStatus == L2CAP_NO_CAUSE) || (wStatus == L2CAP_CONNECTION_ACCEPT) )
		{
			gattLLConnected( pL2CChannel, 0 ); /* triggers GATT_CON_ACT_IND .. */
		}
		else
		{
			/* disconnect */
			gattLLDisconnectReq( pL2CChannel, wStatus );
		}
	}
#if F_BT_BREDR_SUPPORT
	else
	{
		gattSendL2C_CON_RESP( pL2CChannel, wStatus, pL2CChannel->cid );

		if ( (wStatus != L2CAP_NO_CAUSE) && (wStatus != L2CAP_CONNECTION_ACCEPT) )
		{
			/* call not accepted, free channel (no DISC_IND !! ..) */
			gattL2CChannelFree( pL2CChannel );
		}
	}
#endif
}

/**
* @brief  gatt link layer connected
*
* @param  pL2CChannel:
* @param  iLevel
*
* @return  
*
*/
void gattLLConnected( PGATTL2CChannel pL2CChannel, int iLevel )
{
	if ( iLevel > 0 )
	{
		gattSendGATT_CON_ACT_IND( pL2CChannel );
	}
	else
	{
		/* for LE links exchange max. MTU size */
		if ( (pL2CChannel->bdType & BLUEFACE_BDTYPE_LE_MASK) && pL2CChannel->OutgoingCall )
		{
			gattServerConfiguration( pL2CChannel );
		}

		/* get CCC bits from BlueAPI/store if bond for BD exists */
		gattSendDEVICE_DATA_IND( DEVICE_DATA_GET_IND,
		              DEVICE_DATA_TYPE_GATT_CCC_BITS, FALSE, pL2CChannel, 0, NULL );
	}
}

/**
* @brief  (local) connection shutdown initiated
*
* @param  pL2CChannel:
* @param  wStatus
*
* @return  
*
*/
void gattLLDisconnectReq( PGATTL2CChannel pL2CChannel, uint16_t wStatus )
{
	if ( pL2CChannel->bdType & BLUEFACE_BDTYPE_LE_MASK )
	{
		if ( pL2CChannel->L2CState == gattL2CStateConnected )
		{
			gattSendLE_DISCONNECT( pL2CChannel, 0 /*wStatus*/ );
		}
		else
		{
			if ( (pL2CChannel->L2CState == gattL2CStateDiscPending) ||
			   (pL2CChannel->L2CState == gattL2CStateDisconnecting)
			 )
			{
				/* ignore req. */
			}
			else
			{
				gattSendLE_CREATE_CONNECTION_CANCEL( pL2CChannel );
			}
		}
	}
#if F_BT_BREDR_SUPPORT
	else
	{
		gattSendL2C_DISC_REQ( pL2CChannel, wStatus );
	}
#endif
}

/**
* @brief  (remote) connection shutdown initiated
*
* @param  pL2CChannel:
* @param  wStatus
*
* @return  
*
*/
void gattLLDisconnectInd( PGATTL2CChannel pL2CChannel, uint16_t wStatus )
{
	blueAPI_Handle_GATT_DISC_IND( wStatus, pL2CChannel->cid );
}
	
/**
* @brief   connection shutdown complete
*
* @param  pL2CChannel:
* @param  wStatus
* @param  DiscConf
* @param  ForceCleanup
*
* @return  
*
*/
void gattLLDisconnected( PGATTL2CChannel pL2CChannel, uint16_t wStatus,
                         BOOL DiscConf, BOOL ForceCleanup )
{
	BOOL Cleanup = ForceCleanup;

	if ( DiscConf )      /* can be TRUE only for BR/EDR */
	{
		/* BTMAN converts DISC_CONF into DISC_IND !!!!! */
		blueAPI_Handle_GATT_DISC_CONF( wStatus, pL2CChannel->cid );
	}
	else
	{
		if ( pL2CChannel->bdType & BLUEFACE_BDTYPE_LE_MASK )
		{
		  /* no rsp needed */
		}
		else if ( pL2CChannel->L2CState == gattL2CStateDisconnecting )
		{
			/* DISC_IND was rcvd before (=> DISC_RESP is not fwd if local side */
			/* initiated shutdown thru DISC_REQ)                               */
			l2cHandleL2C_DISC_RESP( pL2CChannel->cid);
		}

		Cleanup = TRUE;
	}

	if ( Cleanup )
	{
		attTimerStop( pL2CChannel, FALSE );
		attTimerStop( pL2CChannel, TRUE );
		gattL2CChannelFree( pL2CChannel );
	}
}

/**
* @brief  set buffer callback
*
* @param  CallbackFunction:
* @param  pL2CChannel
* @param  pBuffer
*
* @return  
*
*/
int gattLLBufferCallbackSet( TBufferCallBack CallbackFunction,
                                   PGATTL2CChannel pL2CChannel, uint8_t * pBuffer )
{
    return osBufferCallBackSet(pBuffer, CallbackFunction, (uint32_t)pL2CChannel);
}

/**
* @brief  gatt link layer buffer get
*
* @param  iLength:
*
* @return  
*
*/
uint8_t * gattLLBufferGet( int iLength )
{
	uint8_t * pBuffer;

	if ( osBufferGet(DownstreamPoolID, pGATT->wRxOffset+iLength, (void  *)&pBuffer ) )
	{
		    GATT_TRACE_PRINTF_2(GATT_TRACE_MASK_ERROR,
		                     "!!GATT: gattLLBufferGet / no buffer, iLength=%d, wRxOffset=%d",
		                     iLength, pGATT->wRxOffset);
		assert(FALSE);
		pBuffer = NULL;
	}
	else
	{
		pBuffer += pGATT->wRxOffset;
	}

	return( pBuffer );
}

/**
* @brief  send data packet
*
* @param  pL2CChannel:
*
* @return  
*
*/
void gattLLDataSend( PGATTL2CChannel pL2CChannel,
                                uint8_t * pBuffer, int iLength, uint16_t wOffset )
{
	/* pBuffer points to payload */
	pBuffer -= wOffset;

	if ( pL2CChannel->bdType & BLUEFACE_BDTYPE_LE_MASK )
	{
		gattSendLE_DATA( pL2CChannel, pBuffer, iLength, wOffset );
	}
#if F_BT_BREDR_SUPPORT
	else
	{
		gattSendL2C_DATA_REQ( pL2CChannel, pBuffer, iLength, wOffset );
	}
#endif
}

/**
* @brief  gatt data packet received
*
* @param  pL2CChannel:
* @param  pBuffer
* @param  wLength
* @param  wOffset
*
* @return  
*
*/
BOOL gattLLDataReceived( PGATTL2CChannel pL2CChannel,
                         uint8_t * pBuffer, uint16_t wLength, uint16_t wOffset )
{
	BOOL  ReleaseBuffer = TRUE;

	if ( (pL2CChannel != NULL) &&
	   (pL2CChannel->L2CState != gattL2CStateDiscPending) &&
	   (pL2CChannel->L2CState != gattL2CStateDisconnecting)
	 )
	{
		ReleaseBuffer = attDataReceived( pL2CChannel, pBuffer+wOffset, wLength );
	}

	return( ReleaseBuffer );
}

void gattHandleUpConnectReq(uint8_t bdType, uint8_t * bd, PCON_REQ_GATT pConPara)
{
    PGATTL2CChannel 		pL2CChannel;
	uint16_t					wCause	= GATT_ERR_OUT_OF_RESOURCE;

    /* allocate L2CAP channel */
	if ( (pL2CChannel = gattL2CChannelAllocate( !(bdType == BLUEFACE_BDTYPE_BR_EDR) )) != NULL )
	{
		memcpy( pL2CChannel->RemoteBd, bd, sizeof(TBdAddr) );

		pL2CChannel->bdType = bdType;

		/* init. connection setup, CONF is deferred until */
		/* lower layer ops have completed 				*/
		if ( gattLLConnectReq( pL2CChannel, pConPara) == 0 )
		{
			wCause = GATT_SUCCESS;
		}
	}

	if ( wCause != GATT_SUCCESS )
	{
		/* send immediate -rsp */
		blueAPI_Handle_GATT_CON_CONF( wCause, 0 /* cid */);
	}
}

void gattHandleUpConnectResp(uint16_t cid, uint16_t status)
{
    PGATTL2CChannel 	pL2CChannel;
	uint16_t				wStatus;
	
	/* trigger lower layer rsp */
	pL2CChannel = gattL2CChannelFindHandle( FALSE,
										 cid );
	if ( pL2CChannel != NULL )
	{
		wStatus = status == 0 ?
		  L2CAP_CONNECTION_ACCEPT : L2CAP_ERR_REFUS_NO_RESOURCE; 
		pL2CChannel->wConRespStatus = wStatus;
		gattLLConnectResp( pL2CChannel, wStatus );
	}
}

void gattHandleUpDisconnectReq(uint16_t cid, uint16_t cause)
{
    PGATTL2CChannel  pL2CChannel;

	GATT_TRACE_PRINTF_2(GATT_TRACE_MASK_TRACE,"GATT: DISC_REQ / cause=0x%x, cid=0x%x",
							  cause,
							  cid);

	/* trigger lower layer rsp */
	pL2CChannel = gattL2CChannelFindHandle( FALSE,
									  cid );
	if ( pL2CChannel != NULL )
	{
		gattLLDisconnectReq( pL2CChannel, cause );
	}
}

void gattHandleDisconnected( uint16_t cid, uint16_t result, BOOL DiscConf)
{
    PGATTL2CChannel   pL2CChannel;

	GATT_TRACE_PRINTF_2(GATT_TRACE_MASK_TRACE,
				   "GATT: gattHandleL2C_DISC_CONF / status=0x%x, lcid=0x%x",
				   result, cid);

	pL2CChannel = gattL2CChannelFindHandle( FALSE, cid );
	if ( pL2CChannel != NULL )
	{
		/* no L2C_DISC_IND follows !!!!! */
		gattLLDisconnected( pL2CChannel, result, DiscConf, FALSE );
	}
}

/**
* @brief  send GATT_CON_IND
*
* @param  pL2CChannel:
*
* @return  
*
*/
void gattSendGATT_CON_IND( uint16_t cid, TBdAddr *pBd, uint8_t bdType )
{
    TBtConInd tconInd;

    tconInd.cid            = cid;
    tconInd.channel        = 0; /* upper layer does not see direction bit of dlci */
    tconInd.p.gatt.bdType  = bdType;
    tconInd.p.gatt.mtuSize = 0;
    if ( pBd != NULL )
    {
        memcpy(tconInd.bd, pBd, sizeof(TBdAddr));
    } 
    blueFaceHandleBtConInd_2(BLUEFACE_PSM_GATT, &tconInd);
}

/**
* @brief  gatt send connect active indicate
*
* @param  pL2CChannel:
*
* @return  
*
*/
void gattSendGATT_CON_ACT_IND( PGATTL2CChannel pL2CChannel )
{
    TCON_ACT_IND_GATT gatt;

	gatt.bdType 			= pL2CChannel->bdType;
	gatt.credits			= otp_str_data.gEfuse_UpperStack_s.gatt_max_tx_wsize;
	gatt.connInterval		= pL2CChannel->conParam.connIntervalMin;
	gatt.connLatency		= pL2CChannel->conParam.connLatency;
	gatt.supervisionTimeout = pL2CChannel->conParam.supervisionTimeout;
	memcpy(gatt.bd, pL2CChannel->RemoteBd, sizeof(TBdAddr));

    blueAPI_Handle_GATT_CON_ACT_IND(pL2CChannel->cid, pL2CChannel->wMTUSize, (uint16_t*)&gatt); 
}


void gattHandleAttData(uint16_t handle, MESSAGE_P pMsg, BOOL isL2c)
{
	PGATTL2CChannel pL2CChannel;
	BOOL			DoNotDeferMsg;
    BOOL			ReleaseBuffer = TRUE;
    uint8_t * pData = pMsg->MData.DataCBChan.BufferAddress + pMsg->MData.DataCBChan.Offset;

    if(isL2c)
    {
	    pL2CChannel = gattL2CChannelFindHandle( FALSE, pMsg->MData.DataCBChan.Channel);
    }
    else
    {
        pL2CChannel = gattLEChannelFind( handle );
    }

	if (pL2CChannel != NULL)
	{
		/* the GATT server may not yet be ready when the first ATT PDU is received:  */
		/* defer message if it is not ATT_EXCHANGE_MTU_REQUEST, ATT_WRITE_COMMAND .. */
		/* queue size = 1 is sufficient since all client requests must be responded  */
		/* before another request can be received (window size = 1).				 */
		if ( pL2CChannel->Ready )
		{
			DoNotDeferMsg = TRUE;
		}
		else
		{
			DoNotDeferMsg = !gattDeferMsgIfChannelNotReady( pData );
		}

		if ( DoNotDeferMsg ||(pL2CChannel->DeferredMsgLL.pBuffer == NULL) )
		{
			if ( !DoNotDeferMsg && !pL2CChannel->Ready )
			{
				/* defer handling of data until fully connected */
				/* and all connection data (CCCD ..) is setup	*/
				gattDeferredMsgLLSave( pL2CChannel, pMsg);
				return;
			}
			else
			{
				pGATT->pBuffer = pMsg->MData.DataCB.BufferAddress;
				ReleaseBuffer  = gattLLDataReceived( pL2CChannel,
					  pMsg->MData.DataCBChan.BufferAddress,
					  pMsg->MData.DataCB.Length,
					  pMsg->MData.DataCBChan.Offset);
			}
		}
		else
		{
			/* should no happen due to window size 1 for reqs and inds, discard data */
			GATT_TRACE_PRINTF_0(GATT_TRACE_MASK_TRACE, "!! GATT: discard LE_DATA !!");
			assert(FALSE);
		}
	}

    if(ReleaseBuffer)
	{
		osBufferRelease(pMsg->MData.DataCB.BufferAddress);
	}
}

