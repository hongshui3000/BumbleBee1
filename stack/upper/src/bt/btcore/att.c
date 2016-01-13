/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file        att.c
* @brief      ATT (Attribute Protocol)
* @details   
*
* @author   	gordon
* @date      	2015-07-08
* @version	v0.1
*/

#include <flags.h>
#include <gattdef.h>
#include <attdef.h>
#include <btglbdef.h>
#include <swtimer.h>
#include <os_timer.h>
#include <upper_stack_global.h>
#define TRACE_MODULE_ID     MID_BT_GATT
/**
* @brief  start att timer
*
* @param  pL2CChannel:
* @param  ServerTimer
*
* @return  
*
*/
void attTimerStart( PGATTL2CChannel pL2CChannel,
                           BOOL ServerTimer )
{
#if (GATT_ATT_TIMER_DISABLED)
	UNUSED_PARAMETER(pL2CChannel);
#else
	BOOL     * pTimerStarted = NULL;
    void    ** pTimerHandle = NULL;

	if ( ServerTimer )
	{
		pTimerStarted = &pL2CChannel->pRClient->TimerStarted;
        pTimerHandle = &pL2CChannel->pRClient->timerHandle;
	}
	else
	{
		pTimerStarted = &pL2CChannel->pClient->TimerStarted;
        pTimerHandle = &pL2CChannel->pClient->timerHandle;
	}

	if ( !(*pTimerStarted) )
	{
		GATT_TRACE_PRINTF_2(GATT_TRACE_MASK_ERROR,
		                    "GATT: attTimerStart / timer(srv=%d) started, BD=%s",
		                    ServerTimer, TRACE_BDADDR1(GATT_TRACE_MASK_ERROR, pL2CChannel->RemoteBd));
        *pTimerHandle = 0;
        osStartTimer(pTimerHandle, gattQueueID, (uint8_t)pL2CChannel->wNbr, ServerTimer ? 1 : 0, ATT_TRANSACTION_TIMEOUT_VALUE, swTimerCallBack);
		*pTimerStarted = TRUE;
	}
#endif	
}

/**
* @brief  stop att timer
*
* @param  pGATT:
* @param  pL2CChannel:
* @param  ServerTimer
*
* @return  
*
*/
void attTimerStop(PGATTL2CChannel pL2CChannel, BOOL ServerTimer )
{
	BOOL     * pTimerStarted = NULL;
    void    ** pTimerHandle = NULL;

	if ( ServerTimer )
	{
		pTimerStarted = &pL2CChannel->pRClient->TimerStarted;
        pTimerHandle = &pL2CChannel->pRClient->timerHandle;
	}
	else
	{
		pTimerStarted = &pL2CChannel->pClient->TimerStarted;
        pTimerHandle = &pL2CChannel->pClient->timerHandle;
	}

	if ( *pTimerStarted )
	{
		GATT_TRACE_PRINTF_2(GATT_TRACE_MASK_ERROR,
		                    "GATT: attTimerStop / timer(srv=%d) stopped, BD=%s",
		                    ServerTimer, TRACE_BDADDR1(GATT_TRACE_MASK_ERROR, pL2CChannel->RemoteBd));
        osDeleteTimer(pTimerHandle);
		*pTimerStarted = FALSE;
	}
}

/**
* @brief  att timer expiration handle
*
* @param  pL2CChannel:
* @param  ServerTimer
*
* @return  
*
*/
void attTimerExpired(uint8_t TimerID, BOOL ServerTimer )
{
	BOOL     * pTimerStarted = NULL;
    PGATTL2CChannel pL2CChannel = NULL;
	GATT_TRACE_PRINTF_2(GATT_TRACE_MASK_ERROR,
	                  "!!GATT: attTimerExpired / timer(srv=%d) expired, BD=%s",
	                  ServerTimer, TRACE_BDADDR1(GATT_TRACE_MASK_ERROR, pL2CChannel->RemoteBd));

    if (TimerID >= 0x10)
    {
        pL2CChannel = &pGATT->pL2CChannelDoff[TimerID - 0x10];
    }
    else
    {
        pL2CChannel = &pGATT->pL2CChannelDon[TimerID];
    }

	if ( ServerTimer )
	{
		pTimerStarted = &pL2CChannel->pRClient->TimerStarted;
	}
	else
	{
		pTimerStarted = &pL2CChannel->pClient->TimerStarted;
	}

	if ( *pTimerStarted )
	{
		/* ATT transaction timeout has occurred on this bearer channel:  */
		/* no more PDUs may be sent! establishment of a new bearer resp. */
		/* the shutdown of the current one is left to higher layers!     */
		*pTimerStarted          = FALSE;
		pL2CChannel->ATTTimeout = TRUE;

		if ( ServerTimer )
		{
			/* must be ATT_HANDLE_VALUE_INDICATION timeout */
			gattHandleAttribConf( pL2CChannel, GATT_ERR_TIMEOUT );
		}
		else
		{
			gattHandleATTErrorResponseOrTimeout( pL2CChannel, TRUE,
			                          pL2CChannel->pClient->CurrProc.bOpCode,
			                          0, 0 );
			pL2CChannel->pClient->CurrProc.bOpCode = 0;
		}

	}
}

/**
* @brief   execute (tx) buffer callback
*
* @param  Handle
*
* @return  
*
*/
void attBufferCallback( uint32_t Handle )
{
	PGATTL2CChannel pL2CChannel = (PGATTL2CChannel)Handle;

	if ( pL2CChannel->wUsed &&
	   (pL2CChannel->L2CState == gattL2CStateConnected)
	 )
	{
		PGATTTxData pTxData;

		pTxData = osQueueOut( &pL2CChannel->TxDataQueue );
		if ( pTxData != NULL )
		{
			switch( pTxData->TxDataType )
			{
			default:
				break;

			case TxTypeNotification:
				pTxData->p.AttribUpdData.cause = GATT_SUCCESS;
				gattSendGATT_ATTRIB_UPDATE_CONF( &pTxData->p.AttribUpdData, 0, NULL );
				break;

			case TxTypeWriteCommand:
				gattSendGATT_ATTRIB_WRITE_CONF( GATT_SUCCESS, pL2CChannel->cid,
				                              GATT_WRITE_TYPE_CMD, 0, NULL );
				break;
			}
		  	osQueueIn( &pL2CChannel->TxDataQueueFree, pTxData );

#if (GATT_USE_LL_WSIZE_EQ_1)
			/* more data to send? */
			if ( pL2CChannel->TxDataQueue.Count > 0 )
			{
				uint16_t     wOffset = 0;
				uint16_t     wLength = 0;
				uint8_t *   pBuffer = NULL;

				pTxData = (PGATTTxData)pL2CChannel->TxDataQueue.First;

				switch( pTxData->TxDataType )
				{
				default:
					break;

				case TxTypeNotification:
					pBuffer = pTxData->p.AttribUpdData.pBuffer;
					wOffset = pTxData->p.AttribUpdData.offset;
					wLength = pTxData->p.AttribUpdData.length;
					break;

				case TxTypeWriteCommand:
					pBuffer = pTxData->p.AttribWriteData.pBuffer;
					wOffset = pTxData->p.AttribWriteData.offset;
					wLength = pTxData->p.AttribWriteData.length;
					break;

				}

				if ( pBuffer != NULL )
				{
					/* continue with transfer */
					if ( gattLLBufferCallbackSet(attBufferCallback,
					                                               pL2CChannel, pBuffer ) )
					{
						assert(FALSE);
					}
					gattLLDataSend( pL2CChannel, pBuffer + wOffset, wLength, wOffset );
				}
			}
#endif
		}
	}
}

/**
* @brief  setup MTU size based on value received from peer
*
* @param  pL2CChannel:
* @param  wPeerRxMTU
*
* @return  
*
*/
void attSetupMTUSize( PGATTL2CChannel pL2CChannel,
                                                              uint16_t wPeerRxMTU )
{
	uint16_t  wMTUSizePrev = pL2CChannel->wMTUSize;

	if ( wPeerRxMTU < pL2CChannel->wMTUSizeMax )
	{
		if ( wPeerRxMTU > ATT_MTU_SIZE_LE_DEFAULT )
		{
	  		pL2CChannel->wMTUSize = wPeerRxMTU;
		}
	}
	else
	{
		pL2CChannel->wMTUSize = pL2CChannel->wMTUSizeMax;
	}

	/* if MTU changed inform application */
	if ( wMTUSizePrev != pL2CChannel->wMTUSize )
	{
		gattSendGATT_MTU_SIZE_IND( pL2CChannel->wMTUSize, pL2CChannel->cid );
	}
}

/**
* @brief  send att generic pdu
*
* @param  pL2CChannel:
*
* @return  
*
*/
void attSendATT_GENERIC_PDU( PGATTL2CChannel pL2CChannel, uint8_t  bOpcode, int iLength, 
                                 uint8_t parm1, uint8_t parm2, uint16_t parm3,
                                 int param4, uint8_t * param5)
{
	if ( !pL2CChannel->ATTTimeout )
	{
		uint8_t *      pBuffer;

		if ( (pBuffer = gattLLBufferGet( iLength )) != NULL )
		{
			switch(bOpcode)
			{
			default : /*generic*/
				/* setup PDU */
				((LPATTPDUGeneric)pBuffer)->Opcode = bOpcode;
			break;
			case ATT_ERROR_RESPONSE:
				{
					((LPATTErrorResp)pBuffer)->Opcode	 = ATT_ERROR_RESPONSE;
					((LPATTErrorResp)pBuffer)->ReqOpcode = parm1;
					((LPATTErrorResp)pBuffer)->ErrorCode = parm2;
					LE_WORD2EXTRN( ((LPATTErrorResp)pBuffer)->Handle, parm3);
				}
				break;
			case ATT_EXCHANGE_MTU_REQUEST:
				{
					/* setup PDU */
					pL2CChannel->pClient->CurrProc.bMTUReqCnt++;			
					((LPATTExMTUReq)pBuffer)->Opcode = ATT_EXCHANGE_MTU_REQUEST;
					LE_WORD2EXTRN( ((LPATTExMTUReq)pBuffer)->ClientRxMTU, pL2CChannel->wMTUSizeMax );
				}
				break;
			case ATT_EXCHANGE_MTU_RESPONSE:
				{
					/* setup PDU */
					((LPATTExMTUResp)pBuffer)->Opcode = ATT_EXCHANGE_MTU_RESPONSE;
					LE_WORD2EXTRN( ((LPATTExMTUResp)pBuffer)->ServerRxMTU, parm3);
				}
				break;
			case ATT_READ_BY_GROUP_TYPE_RESPONSE:
				{
					/* setup PDU */
					((LPATTReadGroupTypeResp)pBuffer)->Opcode = ATT_READ_BY_GROUP_TYPE_RESPONSE;
					((LPATTReadGroupTypeResp)pBuffer)->Length = param4;
					memcpy( ((LPATTReadGroupTypeResp)pBuffer)->AttDataList, param5, iLength - offsetof(TATTReadGroupTypeResp, AttDataList));
				}
				break;
			case ATT_FIND_BY_TYPE_VALUE_RESPONSE:
				{
					((LPATTFindTypeValueResp)pBuffer)->Opcode = ATT_FIND_BY_TYPE_VALUE_RESPONSE;
					memcpy( ((LPATTFindTypeValueResp)pBuffer)->HandlesInfoList, param5, param4);
				}
				break;
			case ATT_FIND_INFO_RESPONSE:
				{
					/* setup PDU */
					((LPATTFindInfoResp)pBuffer)->Opcode = ATT_FIND_INFO_RESPONSE;
					if ( param4 == (sizeof(uint16_t) + UUID_16BIT_SIZE) )
					{
						((LPATTFindInfoResp)pBuffer)->Format = ATT_FIND_INFO_RESP_FORMAT_UUID_16BIT;
					}
					else
					{
				  		((LPATTFindInfoResp)pBuffer)->Format = ATT_FIND_INFO_RESP_FORMAT_UUID_128BIT;
					}
					memcpy( ((LPATTFindInfoResp)pBuffer)->InfoData, param5, iLength - offsetof(TATTFindInfoResp, InfoData));
				}
				break;
			case ATT_READ_RESPONSE:
			case ATT_READ_BLOB_RESPONSE:
				{
					/* setup PDU */
					((LPATTPDUGeneric)pBuffer)->Opcode = bOpcode;
					memcpy( ((LPATTPDUGeneric)pBuffer)->Param, param5, param4 );
				}
				break;
			}

			gattLLDataSend( pL2CChannel, pBuffer, iLength, pGATT->wRxOffset );
		}
	}
}

/**
* @brief		send ATT_ERROR_RESPONSE
*
* @param	pL2CChannel
* @param	bReqOpcode
* @param	wHandle
* @param	bErrorCode
*
* @return
*
*/
void attSendATT_ERROR_RESPONSE( PGATTL2CChannel pL2CChannel,
                                             uint8_t  bReqOpcode,
                                             uint16_t  wHandle,
                                             uint8_t  bErrorCode )
{
	attSendATT_GENERIC_PDU(pL2CChannel, ATT_ERROR_RESPONSE, sizeof(TATTErrorResp), bReqOpcode, bErrorCode, wHandle, 0, NULL);
}

/**
* @brief		att handle att error response
*
* @param	pL2CChannel
* @param	bErrorCode
*
* @return
*
*/
void attHandleATT_ERROR_RESPONSE( PGATTL2CChannel pL2CChannel, LPATTErrorResp pErrorResp )
{
	if ( pErrorResp->ReqOpcode == ATT_EXCHANGE_MTU_REQUEST )
	{
		if ( pL2CChannel->pClient->CurrProc.bMTUReqCnt > 0 )
		{
			pL2CChannel->pClient->CurrProc.bMTUReqCnt--;
		}
	}
	else
	{
		attTimerStop( pL2CChannel, FALSE );
		pL2CChannel->pClient->CurrProc.bOpCode = 0;

		gattHandleATTErrorResponseOrTimeout( pL2CChannel, FALSE,
		                        pErrorResp->ReqOpcode, pErrorResp->ErrorCode,
		                        LE_EXTRN2WORD(pErrorResp->Handle) );
	}
}

/**
* @brief  send ATT_EXCHANGE_MTU_REQUEST.
*
* @param  pL2CChannel:
*
* @return  
*
*/
void attSendATT_EXCHANGE_MTU_REQUEST( PGATTL2CChannel pL2CChannel )
{
	attSendATT_GENERIC_PDU(pL2CChannel, ATT_EXCHANGE_MTU_REQUEST, sizeof(TATTExMTUReq), 0, 0, 0, 0, NULL);
}

/**
* @brief  att handle ATT_EXCHANGE_MTU_RESPONSE
*
* @param  pL2CChannel:
*
* @return  
*
*/
void attHandleATT_EXCHANGE_MTU_RESPONSE( PGATTL2CChannel pL2CChannel, LPATTExMTUResp pExMTUResp )
{
	if ( pL2CChannel->pClient->CurrProc.bMTUReqCnt > 0 )
	{
		pL2CChannel->pClient->CurrProc.bMTUReqCnt--;

		/* setup MTU size */
		attSetupMTUSize(pL2CChannel, LE_EXTRN2WORD(pExMTUResp->ServerRxMTU) );
	}
}

/**
* @brief  att send generic request
*
* @param  pL2CChannel:
* @param  iLength
* @param  opcode
* @param  wStartingHandle
* @param  wEndingHandle
* @param  pRequestbuf
* @param  iRequestLength
*
* @return  
*
*/
uint16_t attSendGenericRequest(PGATTL2CChannel pL2CChannel, int iLength,
                                            uint8_t opcode, uint16_t   wStartingHandle,
                                            uint16_t   wEndingHandle,
                                            uint8_t * pRequestbuf,
                                            int    iRequestLength)
{
	uint16_t  wStatus = GATT_SUCCESS;

	if ( pL2CChannel->ATTTimeout )
	{
		wStatus = GATT_ERR_LINK_DEACTIVATED;
	}
	else
	{
		uint8_t * pBuffer;
		
		if ( (pBuffer = gattLLBufferGet( iLength )) != NULL )
		{
			pL2CChannel->pClient->CurrProc.bOpCode	= opcode;
			switch (opcode)
			{
			case ATT_READ_BY_GROUP_TYPE_REQUEST:
			  	{
				    LPATTReadGroupTypeReq  pReadGroupTypeReq;
					/* setup PDU */
					pReadGroupTypeReq		  = (LPATTReadGroupTypeReq)pBuffer;
					pReadGroupTypeReq->Opcode = opcode;
					LE_WORD2EXTRN( pReadGroupTypeReq->StartingHandle, wStartingHandle );
					LE_WORD2EXTRN( pReadGroupTypeReq->EndingHandle, wEndingHandle );
					memcpy( pReadGroupTypeReq->AttGroupType, pRequestbuf, iRequestLength );
				}
				break;
			case ATT_FIND_BY_TYPE_VALUE_REQUEST:
				{
			    LPATTFindTypeValueReq  pFindTypeValueReq;
				pFindTypeValueReq		  = (LPATTFindTypeValueReq)pBuffer;
				pFindTypeValueReq->Opcode = pL2CChannel->pClient->CurrProc.bOpCode;
				LE_WORD2EXTRN( pFindTypeValueReq->StartingHandle, wStartingHandle );
				LE_WORD2EXTRN( pFindTypeValueReq->EndingHandle, wEndingHandle );
				LE_WORD2EXTRN( pFindTypeValueReq->AttType, GATT_UUID_PRIMARY_SERVICE );
				memcpy( pFindTypeValueReq->AttValue, pRequestbuf, iRequestLength );
				}
			break;
			case ATT_READ_BY_TYPE_REQUEST:
				{
			    LPATTReadTypeReq  pReadTypeReq;
				pReadTypeReq		 = (LPATTReadTypeReq)pBuffer;
				pReadTypeReq->Opcode = pL2CChannel->pClient->CurrProc.bOpCode;
				LE_WORD2EXTRN( pReadTypeReq->StartingHandle, wStartingHandle );
				LE_WORD2EXTRN( pReadTypeReq->EndingHandle, wEndingHandle );
				memcpy( pReadTypeReq->AttType, pRequestbuf, iRequestLength );
				}
			break;
			case ATT_FIND_INFO_REQUEST:
				{
			    LPATTFindInfoReq  pFindInfoReq;
				pFindInfoReq		 = (LPATTFindInfoReq)pBuffer;
				pFindInfoReq->Opcode = pL2CChannel->pClient->CurrProc.bOpCode;
				LE_WORD2EXTRN( pFindInfoReq->StartingHandle, wStartingHandle );
				LE_WORD2EXTRN( pFindInfoReq->EndingHandle, wEndingHandle );
				}
			break;
			case ATT_READ_REQUEST:
				{
			    LPATTReadReq      pReadReq;
				pReadReq		 = (LPATTReadReq)pBuffer;
				pReadReq->Opcode = pL2CChannel->pClient->CurrProc.bOpCode;
				LE_WORD2EXTRN( pReadReq->AttHandle, wStartingHandle );
				}
			break;
			case ATT_READ_BLOB_REQUEST:
				{
			    LPATTReadBlobReq      pReadBlobReq;
				pReadBlobReq		 = (LPATTReadBlobReq)pBuffer;
				pReadBlobReq->Opcode = pL2CChannel->pClient->CurrProc.bOpCode;
				LE_WORD2EXTRN( pReadBlobReq->AttHandle, wStartingHandle );
				LE_WORD2EXTRN( pReadBlobReq->ValueOffset, wEndingHandle );
				}
			}
			gattLLDataSend( pL2CChannel, pBuffer, iLength, pGATT->wRxOffset );
			attTimerStart( pL2CChannel, FALSE );
		}
	}

	return( wStatus );
	
}

/**
* @brief		send ATT_READ_BY_GROUP_TYPE_REQUEST
*
* @param	pL2CChannel: 
* @param	wStartingHandle:
* @param	wEndingHandle
* @param	pAttGroupType
* @param	iAttGroupTypeLength
*
* @return
*
*/
uint16_t attSendATT_READ_BY_GROUP_TYPE_REQUEST( PGATTL2CChannel pL2CChannel,
                                            uint16_t   wStartingHandle,
                                            uint16_t   wEndingHandle,
                                            uint8_t * pAttGroupType,
                                            int    iAttGroupTypeLength )
{
	return attSendGenericRequest(pL2CChannel, offsetof(TATTReadGroupTypeReq, AttGroupType) + iAttGroupTypeLength,
							ATT_READ_BY_GROUP_TYPE_REQUEST, wStartingHandle, wEndingHandle,
							pAttGroupType, iAttGroupTypeLength);
}

/**
* @brief  att handle generic response
*
* @param  pL2CChannel:
* @param  pReadGroupTypeResp
* @param  iLength
*
* @return  
*
*/
void attHandleGenericResponse(PGATTL2CChannel pL2CChannel, int iCount, int iSize, uint8_t * pList)
{
	attTimerStop( pL2CChannel, FALSE );
	pL2CChannel->pClient->CurrProc.bOpCode = 0;
	
	gattHandleGenericResponse( pL2CChannel, iCount, iSize, pList);
}

/**
* @brief  att handle ATT_READ_BY_GROUP_TYPE_RESPONSE
*
* @param  pL2CChannel:
* @param  pReadGroupTypeResp
* @param  iLength
*
* @return  
*
*/
void attHandleATT_READ_BY_GROUP_TYPE_RESPONSE(PGATTL2CChannel        pL2CChannel,
                                    LPATTReadGroupTypeResp pReadGroupTypeResp,
                                    int                    iLength )
{
	attHandleGenericResponse(pL2CChannel,(iLength - offsetof(TATTReadGroupTypeResp,AttDataList)) / pReadGroupTypeResp->Length, pReadGroupTypeResp->Length, 
										pReadGroupTypeResp->AttDataList);
}

/**
* @brief  att send ATT_FIND_BY_TYPE_VALUE_REQUEST
*
* @param  pL2CChannel:
* @param  pReadGroupTypeResp
* @param  iLength
*
* @return  
*
*/
uint16_t attSendATT_FIND_BY_TYPE_VALUE_REQUEST( PGATTL2CChannel pL2CChannel,
                                            uint16_t            wStartingHandle,
                                            uint16_t            wEndingHandle,
                                            int             iUUIDLength,
                                            uint8_t *          pUUID)
{
	return attSendGenericRequest(pL2CChannel, offsetof(TATTFindTypeValueReq,AttValue) + iUUIDLength,
							ATT_FIND_BY_TYPE_VALUE_REQUEST, wStartingHandle, wEndingHandle,
							pUUID, iUUIDLength);
}

/**
* @brief  att handle ATT_FIND_BY_TYPE_VALUE_RESPONSE
*
* @param  pL2CChannel:
* @param  pFindTypeValueResp
* @param  iLength
*
* @return  
*
*/
void attHandleATT_FIND_BY_TYPE_VALUE_RESPONSE( PGATTL2CChannel        pL2CChannel,
                                    LPATTFindTypeValueResp pFindTypeValueResp,
                                    int                    iLength )
{
	attHandleGenericResponse(pL2CChannel, 2*sizeof(uint16_t), (iLength - offsetof(TATTFindTypeValueResp,HandlesInfoList)) / (2*sizeof(uint16_t)),
										pFindTypeValueResp->HandlesInfoList);

}

/**
* @brief		send ATT_READ_BY_TYPE_REQUEST
*
* @param	pL2CChannel: 
* @param	wStartingHandle
* @param	wEndingHandle: 
* @param	iUUIDLength
* @param	pUUID
*
* @return
*
*/
uint16_t attSendATT_READ_BY_TYPE_REQUEST( PGATTL2CChannel pL2CChannel,
                                      uint16_t            wStartingHandle,
                                      uint16_t            wEndingHandle,
                                      int             iUUIDLength,
                                      uint8_t *          pUUID)
{
	return attSendGenericRequest(pL2CChannel, offsetof(TATTReadTypeReq,AttType) + iUUIDLength,
						ATT_READ_BY_TYPE_REQUEST, wStartingHandle, wEndingHandle,
						pUUID, iUUIDLength);
}

/**
* @brief		handle ATT_READ_BY_TYPE_RESPONSE
*
* @param	pL2CChannel: 
* @param	pReadTypeResp
* @param	iLength
*
* @return
*
*/
void attHandleATT_READ_BY_TYPE_RESPONSE( PGATTL2CChannel   pL2CChannel,
                                    LPATTReadTypeResp pReadTypeResp,
                                    int               iLength )
{
	int  iCount;       	/* nbr. of list elements */
	int  iTruncated;   	/* nbr. of bytes that have been truncated in last */
	                 	/* handle/value pair     */

	attTimerStop( pL2CChannel, FALSE );
	pL2CChannel->pClient->CurrProc.bOpCode = 0;

	iCount     = (iLength - offsetof(TATTReadTypeResp,AttDataList)) /
	                                                   pReadTypeResp->Length;
	/* last handle/value pair may have been truncated due to MTU size limits: */
	iTruncated = (iLength - offsetof(TATTReadTypeResp,AttDataList)) %
	                                                   pReadTypeResp->Length;
	if ( iTruncated != 0 )
	{
		iTruncated = pReadTypeResp->Length - iTruncated;
		iCount++;
	}

	gattHandleReadByTypeResponse( pL2CChannel, iCount,
	                                            pReadTypeResp->Length,
	                                            iTruncated,
	                                            pReadTypeResp->AttDataList );
}

/**
* @brief		send ATT_FIND_INFO_REQUEST
*
* @param	pL2CChannel: 
* @param	wStartingHandle
* @param	wEndingHandle: 
*
* @return
*
*/
uint16_t attSendATT_FIND_INFO_REQUEST( PGATTL2CChannel pL2CChannel,
                                   uint16_t            wStartingHandle,
                                   uint16_t            wEndingHandle )
{
	return attSendGenericRequest(pL2CChannel, sizeof(TATTFindInfoReq),
						ATT_FIND_INFO_REQUEST, wStartingHandle, wEndingHandle,
						NULL, 0);
}

/**
* @brief		handle ATT_FIND_INFO_RESPONSE
*
* @param	pL2CChannel: 
* @param	pFindInfoResp
* @param	iLength: 
*
* @return
*
*/
void attHandleATT_FIND_INFO_RESPONSE( PGATTL2CChannel   pL2CChannel,
                                             LPATTFindInfoResp pFindInfoResp,
                                             int               iLength )
{
	int  iCount;  /* nbr. of list elements */
	int  iSize;   /* size of list elements */

	iCount = iLength - offsetof(TATTFindInfoResp,InfoData);
	if ( pFindInfoResp->Format == ATT_FIND_INFO_RESP_FORMAT_UUID_16BIT )
	{
		iSize = (sizeof(uint16_t)+UUID_16BIT_SIZE);
	}
	else
	{
		iSize = (sizeof(uint16_t)+UUID_128BIT_SIZE);
	}
	iCount /= iSize;

	attHandleGenericResponse(pL2CChannel, iCount, iSize,
									 pFindInfoResp->InfoData );
}

/**
* @brief		send ATT_READ_REQUEST
*
* @param	pL2CChannel
* @param	wHandle
*
* @return
*
*/
uint16_t attSendATT_READ_REQUEST( PGATTL2CChannel pL2CChannel,
                              uint16_t            wHandle )
{
	return attSendGenericRequest(pL2CChannel, sizeof(TATTReadReq),
						ATT_READ_REQUEST, wHandle, 0,
						NULL, 0);
}

/**
* @brief		handle ATT_READ_RESPONSE
*
* @param	pL2CChannel: 
* @param	pReadResp
* @param	iLength: 
*
* @return
*
*/
void attHandleATT_READ_RESPONSE( PGATTL2CChannel   pL2CChannel,
                                        LPATTReadResp     pReadResp,
                                        int               iLength )
{
	attTimerStop( pL2CChannel, FALSE );
	pL2CChannel->pClient->CurrProc.bOpCode = 0;

	gattHandleReadResponse( pL2CChannel, GATT_READ_TYPE_BASIC,
	                                iLength - offsetof(TATTReadResp,AttValue),
	                                pReadResp->AttValue );
}

/**
* @brief		send ATT_READ_BLOB_REQUEST
*
* @param	pL2CChannel
* @param	wHandle
* @param	wValueOffset
*
* @return
*
*/
uint16_t attSendATT_READ_BLOB_REQUEST( PGATTL2CChannel pL2CChannel,
                                   uint16_t            wHandle,
                                   uint16_t            wValueOffset )
{
	return attSendGenericRequest(pL2CChannel, sizeof(TATTReadBlobReq),
						ATT_READ_BLOB_REQUEST, wHandle, wValueOffset,
						NULL, 0);
}

/**
* @brief		handle ATT_READ_BLOB_RESPONSE
*
* @param	pL2CChannel
* @param	pReadBlobResp
* @param	iLength
*
* @return
*
*/
void attHandleATT_READ_BLOB_RESPONSE( PGATTL2CChannel     pL2CChannel,
                                        LPATTReadBlobResp   pReadBlobResp,
                                        int                 iLength )
{
	attTimerStop( pL2CChannel, FALSE );
	pL2CChannel->pClient->CurrProc.bOpCode = 0;

	gattHandleReadResponse( pL2CChannel, GATT_READ_TYPE_BLOB,
	                          iLength - offsetof(TATTReadBlobResp,PartAttValue),
	                          pReadBlobResp->PartAttValue );
}

/**
* @brief		send ATT_WRITE_REQUEST, ATT_WRITE_COMMAND
*
* @param	pL2CChannel: 
* @param	Cmd
* @param	wHandle
* @param	pBuffer
* @param	pwOffset
* @param	pwLength
*
* @return
*
*/
uint16_t attSendATT_WRITE_REQ_CMD( PGATTL2CChannel pL2CChannel,
                               BOOL            Cmd,
                               uint16_t            wHandle,
                               uint8_t *          pBuffer,
                               LPWORD          pwOffset,
                               LPWORD          pwLength
                             )
{
	uint16_t  wStatus = GATT_SUCCESS;

	if ( pL2CChannel->ATTTimeout )
	{
		wStatus = GATT_ERR_LINK_DEACTIVATED;
	}
	else
	{
		LPATTWriteReq  pWriteReq;
		int            iLength = offsetof(TATTWriteReq,AttValue) + (*pwLength);
		uint8_t           bOpCode = Cmd ? ATT_WRITE_COMMAND : ATT_WRITE_REQUEST;

		/* setup PDU */
		(*pwLength) += offsetof(TATTWriteReq,AttValue);
		(*pwOffset) -= offsetof(TATTWriteReq,AttValue);
		pWriteReq    = (LPATTWriteReq)(pBuffer + (*pwOffset));
		pWriteReq->Opcode = bOpCode;
		LE_WORD2EXTRN( pWriteReq->AttHandle, wHandle );

		if ( Cmd )
		{
#if (GATT_USE_LL_WSIZE_EQ_1)
			/* check if other write commands (or notifications) are queued   */
			/* (current write command is not yet queued by calling routine!) */
			if ( pL2CChannel->TxDataQueue.Count > 0 )
			{
				/* yes, transfer continues in buffer callback */
				return( wStatus );
			}
#endif

			/* callback triggers GATT_WRITE_CONF msg */
			if ( gattLLBufferCallbackSet( attBufferCallback, pL2CChannel, pBuffer ) )
			{
				assert(FALSE);
			}
		}

		gattLLDataSend( pL2CChannel, pBuffer+(*pwOffset), iLength, *pwOffset );
		if ( Cmd )
		{
		  /* no response from peer */
		}
		else
		{
			pL2CChannel->pClient->CurrProc.bOpCode = bOpCode;
			attTimerStart( pL2CChannel, FALSE );
		}
	}

	return( wStatus );
}

/**
* @brief		handle ATT_WRITE_RESPONSE
*
* @param	pL2CChannel: 
* @param	pWriteResp
* @param	iLength
*
* @return
*
*/
void attHandleATT_WRITE_RESPONSE(PGATTL2CChannel  pL2CChannel,
                                        LPATTWriteResp   pWriteResp,
                                        int              iLength )
{
	attTimerStop( pL2CChannel, FALSE );
	pL2CChannel->pClient->CurrProc.bOpCode = 0;

	gattHandleWriteResponse( pL2CChannel, GATT_WRITE_TYPE_REQ, 0, NULL );
}

/**
* @brief		att handle ATT_HANDLE_VALUE_NOTIFICATION / ATT_HANDLE_VALUE_INDICATION
*
* @param	pL2CChannel: 
* @param	pHandleValueInd
* @param	iLength
*
* @return
*
*/
void attHandleATT_HANDLE_VALUE_NOTIF_IND( PGATTL2CChannel      pL2CChannel,
                                        LPATTHandleValueInd  pHandleValueInd,
                                        int                  iLength )
{
	iLength -= offsetof(TATTHandleValueInd, AttValue);
	gattHandleNotifInd( pL2CChannel,
	               pHandleValueInd->Opcode == ATT_HANDLE_VALUE_NOTIFICATION,
	               LE_EXTRN2WORD(pHandleValueInd->AttHandle),
	               iLength, pHandleValueInd->AttValue );
}

/**
* @brief		att send ATT_EXCHANGE_MTU_RESPONSE
*
* @param	pL2CChannel: 
* @param	wRxMTUSize
*
* @return
*
*/
void attSendATT_EXCHANGE_MTU_RESPONSE( PGATTL2CChannel pL2CChannel,
                                              uint16_t            wRxMTUSize)
{
	attSendATT_GENERIC_PDU(pL2CChannel, ATT_EXCHANGE_MTU_RESPONSE, sizeof(TATTExMTUResp), 0, 0, wRxMTUSize,0, NULL);
}

/**
* @brief		att handle ATT_EXCHANGE_MTU_REQUEST
*
* @param	pL2CChannel: 
* @param	pExMTUReq
* @param	iLength
*
* @return
*
*/
uint32_t attHandleATT_EXCHANGE_MTU_REQUEST( PGATTL2CChannel pL2CChannel,
                                                LPATTExMTUReq   pExMTUReq,
                                                int             iLength )
{
	if ( iLength < sizeof(TATTExMTUReq) )
	{
		return( ATT_ERR_INVALID_PDU );
	}

	/* send rsp with own rx MTU size */
	attSendATT_EXCHANGE_MTU_RESPONSE( pL2CChannel, pL2CChannel->wMTUSizeMax );

	/* setup MTU size */
	attSetupMTUSize( pL2CChannel, LE_EXTRN2WORD(pExMTUReq->ClientRxMTU) );

	return( ATT_OK );
}

/**
* @brief		handle ATT_READ_BY_GROUP_TYPE_REQUEST
*
* @param	pL2CChannel: 
* @param	pReadGroupTypeReq
* @param	iLength
*
* @return
*
*/
uint32_t attHandleATT_READ_BY_GROUP_TYPE_REQUEST( PGATTL2CChannel        pL2CChannel,
                                     LPATTReadGroupTypeReq  pReadGroupTypeReq,
                                     int                    iLength )
{
	uint16_t   wStartingHandle;
	uint16_t   wEndingHandle;
	uint32_t  dwATTError = ATT_ERR_INVALID_PDU;

	/* attribute group type must be 16 or 128 bit UUID */
	iLength -= offsetof(TATTReadGroupTypeReq,AttGroupType);
	if ( (iLength == UUID_16BIT_SIZE) || (iLength == UUID_128BIT_SIZE) )
	{
		wStartingHandle = LE_EXTRN2WORD( pReadGroupTypeReq->StartingHandle );
		wEndingHandle   = LE_EXTRN2WORD( pReadGroupTypeReq->EndingHandle );

		if ( (wStartingHandle == 0) || (wStartingHandle > wEndingHandle) )
		{
			dwATTError = ATT_ERR_INVALID_HANDLE;
			ATT_HANDLE2DWORD(dwATTError, wStartingHandle);
		}
		else
		{
			dwATTError = gattHandleReadByGroupTypeRequest( pL2CChannel,
			                 wStartingHandle, wEndingHandle,
			                 iLength, pReadGroupTypeReq->AttGroupType );
		}
	}

	return( dwATTError );
}

/**
* @brief		send ATT_READ_BY_GROUP_TYPE_REQUEST
*
* @param	pL2CChannel: 
* @param	iCnt: nbr. of list elements
* @param	iSize: size of list element
* @param	pList: list of results
*
* @return
*
*/
void attSendATT_READ_BY_GROUP_TYPE_RESPONSE(
                                PGATTL2CChannel  pL2CChannel,
                                int    iCnt,     
                                int    iSize,    
                                uint8_t * pList )   
{
	attSendATT_GENERIC_PDU(pL2CChannel, ATT_READ_BY_GROUP_TYPE_RESPONSE, offsetof(TATTReadGroupTypeResp, AttDataList) + iCnt*iSize, 0, 0, 0,iSize,pList);
}

/**
* @brief		handle ATT_FIND_BY_TYPE_VALUE_REQUEST
*
* @param	pL2CChannel: 
* @param	pFindTypeValueReq
* @param	iLength
*
* @return
*
*/
uint32_t attHandleATT_FIND_BY_TYPE_VALUE_REQUEST( PGATTL2CChannel        pL2CChannel,
                                     LPATTFindTypeValueReq  pFindTypeValueReq,
                                     int                    iLength )
{
	uint16_t   wStartingHandle;
	uint16_t   wEndingHandle;
	uint16_t   wUUID;
	uint32_t  dwATTError = ATT_ERR_INVALID_PDU;

	if ( iLength >= sizeof(TATTFindTypeValueReq) )
	{
		wStartingHandle = LE_EXTRN2WORD( pFindTypeValueReq->StartingHandle );
		wEndingHandle   = LE_EXTRN2WORD( pFindTypeValueReq->EndingHandle );

		if ( (wStartingHandle == 0) || (wStartingHandle > wEndingHandle) )
		{
			dwATTError = ATT_ERR_INVALID_HANDLE;
			ATT_HANDLE2DWORD(dwATTError, wStartingHandle);
		}
		else
		{
			wUUID = LE_EXTRN2WORD( pFindTypeValueReq->AttType );
			dwATTError = gattHandleFindByTypeValueRequest( pL2CChannel,
			                 wStartingHandle, wEndingHandle, wUUID,
			                 iLength-offsetof(TATTFindTypeValueReq,AttValue),
			                 pFindTypeValueReq->AttValue);
		}
	}

	return( dwATTError );
}

/**
* @brief		send ATT_FIND_BY_TYPE_VALUE_RESPONSE
*
* @param	pL2CChannel: 
* @param	iCnt: nbr. of list elements
* @param	iSize: size of list element
* @param	pList: list of results
*
* @return
*
*/
void attSendATT_FIND_BY_TYPE_VALUE_RESPONSE(
                                PGATTL2CChannel  pL2CChannel,
                                int    iCnt,    
                                int    iSize,   
                                uint8_t * pList )   
{
	attSendATT_GENERIC_PDU(pL2CChannel, ATT_FIND_BY_TYPE_VALUE_RESPONSE, offsetof(TATTFindTypeValueResp, HandlesInfoList) + iCnt*iSize, 0, 0, 0,iCnt*iSize ,pList);
}

/**
* @brief		handle ATT_READ_BY_TYPE_REQUEST
*
* @param	pL2CChannel: 
* @param	pReadTypeReq
* @param	iLength
*
* @return
*
*/
uint32_t attHandleATT_READ_BY_TYPE_REQUEST( PGATTL2CChannel  pL2CChannel,
                                     LPATTReadTypeReq pReadTypeReq,
                                     int              iLength )
{
	uint16_t   wStartingHandle;
	uint16_t   wEndingHandle;
	uint32_t  dwATTError = ATT_ERR_INVALID_PDU;

	/* attribute type must be 16 or 128 bit UUID */
	iLength -= offsetof(TATTReadTypeReq,AttType);
	if ( (iLength == UUID_16BIT_SIZE) || (iLength == UUID_128BIT_SIZE) )
	{
		wStartingHandle = LE_EXTRN2WORD( pReadTypeReq->StartingHandle );
		wEndingHandle   = LE_EXTRN2WORD( pReadTypeReq->EndingHandle );

		if ( (wStartingHandle == 0) || (wStartingHandle > wEndingHandle) )
		{
			dwATTError = ATT_ERR_INVALID_HANDLE;
			ATT_HANDLE2DWORD(dwATTError, wStartingHandle);
		}
		else
		{
			dwATTError = gattHandleReadByTypeRequest( pL2CChannel,
			                 wStartingHandle, wEndingHandle,
			                 iLength, pReadTypeReq->AttType);
		}
	}

	return( dwATTError );
}

/**
* @brief		send ATT_READ_BY_TYPE_RESPONSE
*
* @param	pL2CChannel
* @param	iCnt
* @param	iSize
* @param	pList
*
* @return
*
*/
void attSendATT_READ_BY_TYPE_RESPONSE(
                                PGATTL2CChannel  pL2CChannel,
                                int    iCnt,     /* nbr. of list elements */
                                int    iSize,    /* size of list element  */
                                uint8_t * pList )   /* list of results       */
{
	if ( !pL2CChannel->ATTTimeout )
	{
		uint8_t *      pBuffer;
		int         iLength, iDelta;

		iLength = offsetof(TATTReadTypeResp, AttDataList) + iCnt*iSize;
		iDelta  = iLength - pL2CChannel->wMTUSize;
		if ( iDelta > 0 )
		{
			/* last list element has been truncated due to limited MTU size */
			iLength -= iDelta;
		}
		else
		{
			iDelta = 0;
		}

		if ( (pBuffer = gattLLBufferGet( iLength )) != NULL )
		{
			/* setup PDU */
			((LPATTReadTypeResp)pBuffer)->Opcode = ATT_READ_BY_TYPE_RESPONSE;
			((LPATTReadTypeResp)pBuffer)->Length = iSize;
			memcpy( ((LPATTReadTypeResp)pBuffer)->AttDataList, pList, iCnt*iSize - iDelta );

			gattLLDataSend( pL2CChannel, pBuffer, iLength, pGATT->wRxOffset );
		}
	}
}

/**
* @brief		handle ATT_FIND_INFO_REQUEST
*
* @param	pL2CChannel
* @param	pFindInfoReq
* @param	iLength
*
* @return
*
*/
uint32_t attHandleATT_FIND_INFO_REQUEST( PGATTL2CChannel  pL2CChannel,
                                     LPATTFindInfoReq pFindInfoReq,
                                     int              iLength )
{
	uint16_t   wStartingHandle;
	uint16_t   wEndingHandle;
	uint32_t  dwATTError = ATT_ERR_INVALID_PDU;

	if ( iLength == sizeof(TATTFindInfoReq) )
	{
		wStartingHandle = LE_EXTRN2WORD( pFindInfoReq->StartingHandle );
		wEndingHandle   = LE_EXTRN2WORD( pFindInfoReq->EndingHandle );

		if ( (wStartingHandle == 0) || (wStartingHandle > wEndingHandle) )
		{
			dwATTError = ATT_ERR_INVALID_HANDLE;
			ATT_HANDLE2DWORD(dwATTError, wStartingHandle);
		}
		else
		{
			dwATTError = gattHandleFindInfoRequest( pL2CChannel,
			                                 wStartingHandle, wEndingHandle );
		}
	}

	return( dwATTError );
}

/**
* @brief		send ATT_FIND_INFO_RESPONSE
*
* @param	pL2CChannel
* @param	iCnt
* @param	iSize
* @param	pList
*
* @return
*
*/
void attSendATT_FIND_INFO_RESPONSE(
                                PGATTL2CChannel  pL2CChannel,
                                int    iCnt,     /* nbr. of list elements */
                                int    iSize,    /* size of list element  */
                                uint8_t * pList )   /* list of results       */
{
	attSendATT_GENERIC_PDU(pL2CChannel, ATT_FIND_INFO_RESPONSE, offsetof(TATTFindInfoResp, InfoData) + iCnt*iSize, 0, 0, 0,iSize ,pList);
}

/**
* @brief		handle ATT_READ_REQUEST
*
* @param	pL2CChannel
* @param	pReadReq
* @param	iLength
*
* @return
*
*/
uint32_t attHandleATT_READ_REQUEST( PGATTL2CChannel  pL2CChannel,
                                     LPATTReadReq     pReadReq,
                                     int              iLength )
{
	uint16_t   wHandle;
	uint32_t  dwATTError = ATT_ERR_INVALID_PDU;

	if ( iLength == sizeof(TATTReadReq) )
	{
		wHandle = LE_EXTRN2WORD( pReadReq->AttHandle );

		if ( wHandle == 0 )
		{
			dwATTError = ATT_ERR_INVALID_HANDLE;
			ATT_HANDLE2DWORD(dwATTError, wHandle);
		}
		else
		{
			dwATTError = gattHandleReadRequest( pL2CChannel,
			                            GATT_READ_TYPE_BASIC, (uint8_t *)pReadReq );
		}
	}

	return( dwATTError );
}

/**
* @brief		handle ATT_READ_BLOB_REQUEST
*
* @param	pL2CChannel
* @param	pReadBlobReq
* @param	iLength
*
* @return
*
*/
uint32_t attHandleATT_READ_BLOB_REQUEST(PGATTL2CChannel    pL2CChannel,
                                     LPATTReadBlobReq   pReadBlobReq,
                                     int                iLength )
{
	uint16_t   wHandle;
	uint32_t  dwATTError = ATT_ERR_INVALID_PDU;

	if ( iLength == sizeof(TATTReadBlobReq) )
	{
		wHandle = LE_EXTRN2WORD( pReadBlobReq->AttHandle );

		if ( wHandle == 0 )
		{
			dwATTError = ATT_ERR_INVALID_HANDLE;
			ATT_HANDLE2DWORD(dwATTError, wHandle);
		}
		else
		{
			dwATTError = gattHandleReadRequest( pL2CChannel,
			                            GATT_READ_TYPE_BLOB, (uint8_t *)pReadBlobReq );
		}
	}

	return( dwATTError );
}

/**
* @brief		send ATT_READ_RESPONSE, ATT_READ_BLOB_RESPONSE
*
* @param	pL2CChannel: 
* @param	bOpCode
* @param	iSize:  size of attribute value 
* @param	pValue: attribute value
*
* @return
*
*/
void attSendATT_READx_RESPONSE( PGATTL2CChannel  pL2CChannel,
                                uint8_t   bOpCode,
                                int    iSize,    
                                uint8_t * pValue )  
{
	attSendATT_GENERIC_PDU(pL2CChannel, bOpCode, offsetof(TATTPDUGeneric, Param) + iSize, 0, 0, 0,iSize ,pValue);
}
	
/**
* @brief		handle ATT_WRITE_REQUEST, ATT_WRITE_COMMAND, ATT_PREPARE_WRITE_REQUEST
*
* @param	pL2CChannel: 
* @param	pWriteReq
* @param	iLength 
* @param	pReleaseBuffer
*
* @return
*
*/
uint32_t attHandleATT_WRITE_REQ_CMD( PGATTL2CChannel pL2CChannel,
                                                LPATTWriteReq   pWriteReq,
                                                int             iLength,
                                                BOOL          * pReleaseBuffer )
{
	uint16_t   wHandle;
	uint16_t   wType;
	int    iValueLength = iLength - offsetof(TATTWriteReq,AttValue);
	uint8_t * pAttValue    = pWriteReq->AttValue;
	uint16_t   wWriteOffset = 0;
	uint32_t  dwATTError   = ATT_OK;

	*pReleaseBuffer = TRUE;

	switch( pWriteReq->Opcode )
	{
	default:
	case ATT_WRITE_REQUEST:
		wType = GATT_WRITE_TYPE_REQ;
		break;
	case ATT_WRITE_COMMAND:
		wType = GATT_WRITE_TYPE_CMD;
		break;
	}

	wHandle = LE_EXTRN2WORD( pWriteReq->AttHandle );

	if ( wHandle == 0 )
	{
		dwATTError = ATT_ERR_INVALID_HANDLE;
		ATT_HANDLE2DWORD(dwATTError, wHandle);
	}
	else
	{
		dwATTError = gattHandleWriteReqCmd( pL2CChannel, wType, wHandle,
		                                    iValueLength,
		                                    wWriteOffset,
		                                    pAttValue,
		                                    pReleaseBuffer );
	}

	/* for ATT_WRITE_COMMAND no response at all is sent */
	if ( wType == GATT_WRITE_TYPE_CMD )
	{
		dwATTError = ATT_OK;
	}

	return( dwATTError );
}


uint32_t attHandleATT_PREPARE_WRITE_REQ( PGATTL2CChannel pL2CChannel,
                                      LPATTPrepareWriteReq   pPrepWriteReq,
                                      int             iLength,
                                      BOOL          * pReleaseBuffer )
{
    uint16_t   wHandle;
    int    iValueLength = iLength - offsetof(TATTPrepareWriteReq, PartAttValue);
    uint8_t * pAttValue    = pPrepWriteReq->PartAttValue;
    uint16_t   wWriteOffset = 0;
    uint32_t  dwATTError   = ATT_OK;
	
	*pReleaseBuffer = TRUE;

    wHandle = LE_EXTRN2WORD( pPrepWriteReq->AttHandle );
    wWriteOffset = LE_EXTRN2WORD( pPrepWriteReq->ValueOffset );

    if ( wHandle == 0 )
    {
        dwATTError = ATT_ERR_INVALID_HANDLE;
        ATT_HANDLE2DWORD(dwATTError, wHandle);
    }
    else
    {
        dwATTError = gattHandlePrepareWriteReq( pL2CChannel, wHandle,
                                                iValueLength,
                                                wWriteOffset,
                                                pAttValue,
                                                pReleaseBuffer);
    }

    return ( dwATTError );
}

uint32_t attHandleATT_EXECUTE_WRITE_REQ( PGATTL2CChannel pL2CChannel,
                                      LPATTExecuteWriteReq   pExecWriteReq,
                                      int             iLength )
{
    uint32_t  dwATTError   = ATT_OK;
    uint8_t   flags;

    flags = pExecWriteReq->Flags;

    dwATTError = gattHandleExecuteWriteReq( pL2CChannel, flags);

    return ( dwATTError );
}

void attHandleATT_PREPARE_WRITE_RESPONSE(PGATTL2CChannel  pL2CChannel,
        LPATTPrepareWriteResp   pPrepWriteResp,
        int              iLength )
{
    uint16_t valueOffset;
    attTimerStop(pL2CChannel, FALSE );
    pL2CChannel->pClient->CurrProc.bOpCode = 0;
    valueOffset = LE_EXTRN2WORD( pPrepWriteResp->ValueOffset);
    iLength -= offsetof(TATTPrepareWriteResp, PartAttValue);

    gattHandlePrepareWriteResponse(pL2CChannel, GATT_WRITE_TYPE_PREP, valueOffset, iLength, pPrepWriteResp->PartAttValue);
}


void attHandleATT_EXECUTE_WRITE_RESPONSE(PGATTL2CChannel  pL2CChannel,
        LPATTExecuteWriteResp   pExecWriteResp,
        int              iLength )
{
    attTimerStop(pL2CChannel, FALSE );
    pL2CChannel->pClient->CurrProc.bOpCode = 0;

    gattHandleExecuteWriteResponse(pL2CChannel);
}

uint16_t attSendATT_PREPARE_WRITE_REQ( PGATTL2CChannel pL2CChannel,
                                   uint16_t            wWriteOffset,
                                   uint16_t            wHandle,
                                   uint8_t *          pBuffer,
                                   LPWORD          pwOffset,
                                   LPWORD          pwLength
                                 )

{
    uint16_t  wStatus = GATT_SUCCESS;

    if ( pL2CChannel->ATTTimeout )
    {
        wStatus = GATT_ERR_LINK_DEACTIVATED;
    }
    else
    {
        LPATTPrepareWriteReq  pPrepWriteReq;
        int            iLength = offsetof(TATTPrepareWriteReq, PartAttValue) + (*pwLength);
        uint8_t           bOpCode = ATT_PREPARE_WRITE_REQUEST;

        /* setup PDU */
        (*pwLength) += offsetof(TATTPrepareWriteReq, PartAttValue);
        (*pwOffset) -= offsetof(TATTPrepareWriteReq, PartAttValue);
        pPrepWriteReq    = (LPATTPrepareWriteReq)(pBuffer + (*pwOffset));
        pPrepWriteReq->Opcode = bOpCode;
        LE_WORD2EXTRN( pPrepWriteReq->AttHandle, wHandle );
        LE_WORD2EXTRN( pPrepWriteReq->ValueOffset, wWriteOffset );

        gattLLDataSend( pL2CChannel, pBuffer + (*pwOffset), iLength, *pwOffset );

        pL2CChannel->pClient->CurrProc.bOpCode = bOpCode;
        attTimerStart( pL2CChannel, FALSE );
    }

    return ( wStatus );
}

void attSendATT_PREPARE_WRITE_RESPONSE( PGATTL2CChannel  pL2CChannel,
                                        uint16_t   wHandle,
                                        uint16_t   wWriteOffset,
                                        int    iSize,
                                        uint8_t * pValue )
{
    if ( !pL2CChannel->ATTTimeout )
    {
        uint8_t *      pBuffer;
        LPATTPrepareWriteResp  pPrepWriteResp;
        int            iLength = offsetof(TATTPrepareWriteResp, PartAttValue) + iSize;

        if ( (pBuffer = gattLLBufferGet( iLength )) != NULL )
        {
            /* setup PDU */
            pPrepWriteResp = (LPATTPrepareWriteResp)pBuffer;
            pPrepWriteResp->Opcode = ATT_PREPARE_WRITE_RESPONSE;
            LE_WORD2EXTRN( pPrepWriteResp->AttHandle, wHandle );
            LE_WORD2EXTRN( pPrepWriteResp->ValueOffset, wWriteOffset );
            memcpy(pPrepWriteResp->PartAttValue, pValue, iSize );

            gattLLDataSend( pL2CChannel, pBuffer, iLength, pGATT->wRxOffset );
        }

    }
}

uint16_t attSendATT_EXECUTE_WRITE_REQ( PGATTL2CChannel pL2CChannel,
                                   uint8_t            bFlags
                                 )
{
    uint16_t  wStatus = GATT_SUCCESS;

    if ( pL2CChannel->ATTTimeout )
    {
        wStatus = GATT_ERR_LINK_DEACTIVATED;
    }
    else
    {
        uint8_t *            pBuffer;
        LPATTExecuteWriteReq pExeWriteReq;
        int               iLength;

        iLength = sizeof(TATTExecuteWriteReq);
        if ( (pBuffer = gattLLBufferGet( iLength )) != NULL )
        {
            /* setup PDU */
            pL2CChannel->pClient->CurrProc.bOpCode = ATT_EXECUTE_WRITE_REQUEST;

            pExeWriteReq         = (LPATTExecuteWriteReq)pBuffer;
            pExeWriteReq->Opcode = pL2CChannel->pClient->CurrProc.bOpCode;
            pExeWriteReq->Flags = bFlags;

            gattLLDataSend( pL2CChannel, pBuffer, iLength, pGATT->wRxOffset );
            attTimerStart( pL2CChannel, FALSE );
        }
    }

    return ( wStatus );
}

void attSendATT_EXECUTE_WRITE_RESPONSE( PGATTL2CChannel pL2CChannel )
{
    attSendATT_GENERIC_PDU( pL2CChannel, ATT_EXECUTE_WRITE_RESPONSE, offsetof(TATTPDUGeneric, Param),0,0,0,0,0 );
}


/**
* @brief		send ATT_HANDLE_VALUE_NOTIFICATION / ATT_HANDLE_VALUE_INDICATION
*
* @param	pL2CChannel: 
* @param	Notify
* @param	pBuffer
* @param	pwOffsetPayload:
* @param	wHandle
* @param	piLength
* @param	pValue
*
* @return
*
*/
int  attSendATT_HANDLE_VALUE_NOTIF_IND( PGATTL2CChannel  pL2CChannel,
                                 BOOL Notify,
                                 uint8_t * pBuffer, LPWORD pwOffsetPayload,
                                 uint16_t wHandle, int * piLength, uint8_t * pValue )
{
	uint16_t wOffset;
	int  iReturn     = -1;
	BOOL ReuseBuffer = (pBuffer != NULL) ? TRUE : FALSE; /* reuse applic. buffer */

	if ( pL2CChannel->ATTTimeout )
	{
		iReturn = 1;        /* link deactivated due to preceding timeout */
	}
	else
	{
		int     iSize;
		uint8_t *  pBuffer0 = pBuffer;

		iSize = offsetof( TATTHandleValueInd, AttValue) + (*piLength);

		/* pBuffer points to start of ATT PDU: */
		if ( ReuseBuffer )
		{
			assert(*pwOffsetPayload >= offsetof( TATTHandleValueInd, AttValue ));
			pBuffer = pBuffer + *pwOffsetPayload -
			                   offsetof( TATTHandleValueInd, AttValue );
			*pwOffsetPayload -= offsetof( TATTHandleValueInd, AttValue );
			wOffset = *pwOffsetPayload;
		}
		else
		{
			pBuffer = gattLLBufferGet( iSize );
			wOffset = pGATT->wRxOffset;
		}

		if ( pBuffer != NULL )
		{
			/* setup PDU */
			((LPATTHandleValueInd)pBuffer)->Opcode =
			  Notify ? ATT_HANDLE_VALUE_NOTIFICATION : ATT_HANDLE_VALUE_INDICATION;
			LE_WORD2EXTRN( ((LPATTHandleValueInd)pBuffer)->AttHandle, wHandle );
			if ( ReuseBuffer )
			{
				if ( Notify )
				{
#if (GATT_USE_LL_WSIZE_EQ_1)
					/* check if other notifications (or write commands) are queued  */
					/* (current notification is not yet queued by calling routine!) */
					if ( pL2CChannel->TxDataQueue.Count > 0 )
					{
						/* yes, transfer continues in buffer callback */
						*piLength = iSize;
						return( 0 );
					}
#endif
					/* callback triggers GATT_ATTRIB_UPDATE_CONF msg (for indications */
					/* CONF is generated immediately ..)                              */
					if ( gattLLBufferCallbackSet( attBufferCallback,
					                                             pL2CChannel, pBuffer0 ) )
					{
						assert(FALSE);
					}
				}
		  	}
			else
			{
				memcpy( ((LPATTHandleValueInd)pBuffer)->AttValue, pValue, *piLength );
			}

			gattLLDataSend( pL2CChannel, pBuffer, iSize, wOffset );
			if ( !Notify )
			{
				attTimerStart( pL2CChannel, TRUE );
			}
			*piLength = iSize;
			iReturn   = 0;
		}
		else
		{
			iReturn = -1;
		}
	}
	return( iReturn );
}

/**
* @brief		handle ATT_HANDLE_VALUE_CONFIRMATION
*
* @param	pL2CChannel: 
* @param	pHandleValueConf
* @param	iLength
*
* @return
*
*/
uint32_t attHandleATT_HANDLE_VALUE_CONFIRMATION( PGATTL2CChannel        pL2CChannel,
                                       LPATTHandleValueConf   pHandleValueConf,
                                       int                    iLength )
{
	UNUSED_PARAMETER(pHandleValueConf);
	UNUSED_PARAMETER(iLength);

	attTimerStop(pL2CChannel, TRUE );

	gattHandleAttribConf( pL2CChannel, GATT_SUCCESS );
	return( ATT_OK );
}

/**
* @brief  handle receive data packet.
*
* @param  pL2CChannel:
* @param  pBuffer
* @param  iLength
*
* @return  
*
*/
BOOL attDataReceived( PGATTL2CChannel pL2CChannel,
                                                uint8_t * pBuffer, int iLength )
{
	uint8_t       bOpcode;
	uint32_t      dwATTError;
	BOOL       ReleaseBuffer = TRUE;

	bOpcode    = ((LPATTPDU)pBuffer)->Opcode;
	bOpcode   &= bOpcode & ATT_OPCODE_MASK;
	dwATTError = ATT_OK;


	/* requests handled disregarding role */
	switch( bOpcode )
	{
	default:
		break;

	case ATT_EXCHANGE_MTU_REQUEST:
		dwATTError = attHandleATT_EXCHANGE_MTU_REQUEST( pL2CChannel,
		                                              (LPATTExMTUReq)pBuffer,
		                                              iLength );
		break;
	case ATT_READ_BY_GROUP_TYPE_REQUEST:
		dwATTError = attHandleATT_READ_BY_GROUP_TYPE_REQUEST( pL2CChannel,
		                                         (LPATTReadGroupTypeReq)pBuffer,
		                                         iLength );
		break;
	case ATT_FIND_BY_TYPE_VALUE_REQUEST:
		dwATTError = attHandleATT_FIND_BY_TYPE_VALUE_REQUEST( pL2CChannel,
		                                         (LPATTFindTypeValueReq)pBuffer,
		                                         iLength );
		break;
	case ATT_READ_BY_TYPE_REQUEST:
		dwATTError = attHandleATT_READ_BY_TYPE_REQUEST( pL2CChannel,
		                                         (LPATTReadTypeReq)pBuffer,
		                                         iLength );
		break;
	case ATT_FIND_INFO_REQUEST:
		dwATTError = attHandleATT_FIND_INFO_REQUEST( pL2CChannel,
		                                            (LPATTFindInfoReq)pBuffer,
		                                            iLength );
		break;
	case ATT_READ_REQUEST:
		dwATTError = attHandleATT_READ_REQUEST( pL2CChannel,
		                                            (LPATTReadReq)pBuffer,
		                                            iLength );
		break;
	case ATT_READ_BLOB_REQUEST:
		dwATTError = attHandleATT_READ_BLOB_REQUEST( pL2CChannel,
		                                            (LPATTReadBlobReq)pBuffer,
		                                            iLength );
		break;
	case ATT_WRITE_REQUEST:
	case ATT_WRITE_COMMAND:
		dwATTError = attHandleATT_WRITE_REQ_CMD( pL2CChannel,
		                                            (LPATTWriteReq)pBuffer,
		                                            iLength,
		                                            &ReleaseBuffer );
		break;

    case ATT_PREPARE_WRITE_REQUEST:
        dwATTError = attHandleATT_PREPARE_WRITE_REQ( pL2CChannel,
                     (LPATTPrepareWriteReq)pBuffer,
                     iLength,
                     &ReleaseBuffer );
        break;        
    case ATT_EXECUTE_WRITE_REQUEST:
        dwATTError = attHandleATT_EXECUTE_WRITE_REQ( pL2CChannel,
                     (LPATTExecuteWriteReq)pBuffer,
                     iLength );
        break;

	case ATT_HANDLE_VALUE_CONFIRMATION:
		dwATTError = attHandleATT_HANDLE_VALUE_CONFIRMATION( pL2CChannel,
		                                          (LPATTHandleValueConf)pBuffer,
		                                          iLength );
		break;

	case ATT_READ_MULTI_REQUEST:
		dwATTError = ATT_ERR_UNSUPPORTED_REQUEST;
		break;
	case ATT_ERROR_RESPONSE:
		attHandleATT_ERROR_RESPONSE( pL2CChannel,
		                                          (LPATTErrorResp)pBuffer );
		break;

	case ATT_EXCHANGE_MTU_RESPONSE:
		attHandleATT_EXCHANGE_MTU_RESPONSE( pL2CChannel,
		                                          (LPATTExMTUResp)pBuffer );
		break;

	case ATT_READ_BY_GROUP_TYPE_RESPONSE:
		attHandleATT_READ_BY_GROUP_TYPE_RESPONSE( pL2CChannel,
		                                      (LPATTReadGroupTypeResp)pBuffer,
		                                      iLength );
		break;

	case ATT_FIND_BY_TYPE_VALUE_RESPONSE:
		attHandleATT_FIND_BY_TYPE_VALUE_RESPONSE( pL2CChannel,
		                                      (LPATTFindTypeValueResp)pBuffer,
		                                      iLength );
		break;

	case ATT_READ_BY_TYPE_RESPONSE:
		attHandleATT_READ_BY_TYPE_RESPONSE( pL2CChannel,
		                                      (LPATTReadTypeResp)pBuffer,
		                                      iLength );
		break;

	case ATT_FIND_INFO_RESPONSE:
		attHandleATT_FIND_INFO_RESPONSE( pL2CChannel,
		                                      (LPATTFindInfoResp)pBuffer,
		                                      iLength );
		break;

	case ATT_READ_RESPONSE:
		attHandleATT_READ_RESPONSE( pL2CChannel,
		                                      (LPATTReadResp)pBuffer,
		                                      iLength );
		break;

	case ATT_READ_BLOB_RESPONSE:
		attHandleATT_READ_BLOB_RESPONSE( pL2CChannel,
		                                      (LPATTReadBlobResp)pBuffer,
		                                      iLength );
		break;

	case ATT_WRITE_RESPONSE:
		attHandleATT_WRITE_RESPONSE( pL2CChannel,
		                                      (LPATTWriteResp)pBuffer,
		                                      iLength );
		break;
    
    case ATT_PREPARE_WRITE_RESPONSE:
        attHandleATT_PREPARE_WRITE_RESPONSE( pL2CChannel,
                                             (LPATTPrepareWriteResp)pBuffer,
                                             iLength );
        break;

    case ATT_EXECUTE_WRITE_RESPONSE:
        attHandleATT_EXECUTE_WRITE_RESPONSE( pL2CChannel,
                                             (LPATTExecuteWriteResp)pBuffer,
                                             iLength );
        break;
     
	case ATT_HANDLE_VALUE_NOTIFICATION:
	case ATT_HANDLE_VALUE_INDICATION:
		attHandleATT_HANDLE_VALUE_NOTIF_IND( pL2CChannel,
		                                      (LPATTHandleValueInd)pBuffer,
		                                      iLength );
		ReleaseBuffer = FALSE;
		break;
	}

	if ( dwATTError )
	{
		attSendATT_ERROR_RESPONSE( pL2CChannel,
		                                  ((LPATTPDU)pBuffer)->Opcode,
		                                  ATT_DWORD2HANDLE(dwATTError),
		                                  ATT_DWORD2ERRCODE(dwATTError) );
	}

	return( ReleaseBuffer );
}

