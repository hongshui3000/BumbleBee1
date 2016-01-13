/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       gattmsg.c
* @brief     (OSIF) message handling
* @details   
*
* @author   	gordon
* @date      	2015-07-09
* @version	v0.1
*/

#include <flags.h>
#include <os_pool.h>
#include <os_intr.h>
#include <gattdef.h>
#include <blueapi_api.h>
#include <hci_code.h>
#include <hci_api.h>
#include <btsend.h>
#include <l2c_api.h>
#include <att_code.h>
#include <attdef.h>
#include <gatt_api.h>
#include <upper_stack_global.h>

#define TRACE_MODULE_ID     MID_BT_HCI
/**
* @brief		send GATT_SERVICE_REGISTER_CONF
*
* @param	wApplHandle: 
* @param	wCause
* @param	pGATTService
* @param	bServiceChanged
*
* @return
*
*/
void gattSendGATT_SERVICE_REGISTER_CONF( uint16_t         wCause,
                                                PGATTService pGATTService)
{
#if 0
	if ( (wCause == GATT_SUCCESS) && bServiceChanged )
	{
		/* delete all CCCDs for all BDs in keystore */
		gattSendDEVICE_DATA_IND( pGATT, DEVICE_DATA_SET_IND,
		                             DEVICE_DATA_TYPE_GATT_CCC_BITS, TRUE,
		                             NULL, 0, NULL );
		/* set ServiceChanged indication flag for all BDs in keystore */
		gattSendDEVICE_DATA_IND( pGATT, DEVICE_DATA_SET_IND,
		                             DEVICE_DATA_TYPE_GATT_SRV_CHG_HANDLE, FALSE,
		                             NULL, 0, NULL );
	}
#endif
    blueAPI_Handle_GATT_SERVICE_REGISTER_CONF(wCause, pGATTService);
}

/**
* @brief		 send GATT_ATTRIB_UPDATE_CONF
*
* @param	pAttribUpdData: 
* @param	iCount
* @param	pElement
*
* @return
*
*/
void gattSendGATT_ATTRIB_UPDATE_CONF( PGATTAttribUpdData pAttribUpdData,
                          int  iCount,
                          PDEVICE_DATA_ELEMENT_GATT_CCC_BITS_REV pElement )
{
	PGATTService          pGATTService;
	int                   i, iOffset, iLength;
	uint8_t *                pBuffer;
	PGATT_ATTRIB_UPDATE_LIST_ELEMENT pElementDst;

	pGATTService = pAttribUpdData->pGATTService;


	/* assure that all msg headers of upper layers fit into buffer: */
	iOffset = gGattUsWriteOffset;

	iLength = iCount*sizeof(TGATT_ATTRIB_UPDATE_LIST_ELEMENT);
	if ( osBufferGet(UpstreamPoolID, iOffset+iLength, (PVOID)&pBuffer) )
	{
		GATT_TRACE_PRINTF_1(GATT_TRACE_MASK_ERROR,
		             "!!GATT: gattSendGATT_ATTRIB_UPDATE_CONF / out of memory, size=%d",
		             iOffset+iLength);
		assert(FALSE);
		return;
	}

	/* append list if BDs */
	pElementDst = (PGATT_ATTRIB_UPDATE_LIST_ELEMENT)(pBuffer+iOffset);
	for ( i = 0; i < iCount; i++ )
	{
		memcpy( &pElementDst[i].bd, &pElement[i].bd, sizeof(TBdAddr) );
		pElementDst[i].bdType = pElement[i].bdType;
	}

    if( !blueAPI_Send_GATTAttributeUpdateRsp(pBuffer, iOffset, pGATTService, 
                                             pAttribUpdData->reqHandle,
                                             pAttribUpdData->attribIndex,
                                             pAttribUpdData->cause,
                                             iCount) )
	{
	    return;
	}
	osBufferRelease((PVOID)pBuffer);   
}

/**
* @brief  send blueAPI_EventGATTAttributeUpdateStatusInd
*
* @param  pRClient:
* @param  wStatus
*
* @return  
*
*/
void gattSendGATT_ATTRIB_UPDATE_STATUS_IND( PGATTRClient pRClient,
                                                         uint16_t wStatus )
{
    blueAPI_Send_GATTAttributeUpdateStatusInd(pRClient->CurrInd.wAttribIndex,
	                                           pRClient->CurrInd.pServiceHandle,
	                                           pRClient->CurrInd.pReqHandle, 
	                                           pRClient->pL2CChannel->bdType,
	                                           (uint8_t *)&pRClient->pL2CChannel->RemoteBd,
                                               wStatus
                                               );
}




/**
* @brief  on reception of DEVICE_DATA_GET_RESP channel is ready and deferred
* 		CON_ACT_IND may be sent.
*
* @param  pL2CChannel
* @return  
*
*/
void gattSetChannelReady( PGATTL2CChannel pL2CChannel )
{
	if ( !pL2CChannel->Ready )
	{
		pL2CChannel->Ready = TRUE;
		gattLLConnected( pL2CChannel, 1 );
	}
}

/**
* @brief  gatt send device data indicate
*
* @param  wCommand:
* @param  bDataType
* @param  Delete
* @param  pL2CChannel
* @param  wHandle
* @param  pAttribUpdData
*
* @return  
*
*/
int gattSendDEVICE_DATA_IND( uint16_t wCommand, uint8_t bDataType,
                                BOOL Delete, PGATTL2CChannel pL2CChannel,
                                uint16_t wHandle, PGATTAttribUpdData pAttribUpdData )
{
	MESSAGE_T        Message;
	uint8_t *           pBuffer;
	PDeviceDataInd   pDeviceDataInd;
	int              iLength, iOffset;
	int              iCount  = 0;
	int              iReturn = 0;

	if ( wCommand == DEVICE_DATA_SET_IND )
	{
		if ( bDataType == DEVICE_DATA_TYPE_GATT_CCC_BITS )
		{
			if ( pL2CChannel == NULL )
			{
				/* delete all CCCDs for all BDs in keystore */
				iCount = -1;
			}
			else
			{
				/* count BDs CCC bits in local table, if req. increase buffer size */
				iCount = gattCCCBitsOp( CCCBitsCount, pL2CChannel, NULL );
				if ( iCount <= 0 )
				{
					return( iReturn );
				}
			}
		}
		else
		{
			/* bDataType == DEVICE_DATA_TYPE_GATT_SRV_CHG_HANDLE */
			iCount = 1;
		}
	}
	else
	{
		if ( pL2CChannel != NULL )
		{
			pL2CChannel->DevDataGetRspPending = TRUE;
		}
	}

	iLength = offsetof(TblueFaceMsg, p.deviceDataIndication) + sizeof(TDEVICE_DATA_IND);
	if ( osBufferGet(BTSystemPoolID, iLength, (PVOID)&pBuffer) )
	{
		GATT_TRACE_PRINTF_1(GATT_TRACE_MASK_ERROR,
		             "!!GATT: gattSendDEVICE_DATA_IND / out of memory, size=%d",
		             iLength);
		assert(FALSE);
		iReturn = -1;
	}
	else
	{
		/* leave space for Blueface header */
		iOffset        = offsetof(TblueFaceMsg,p.deviceDataIndication);
		pDeviceDataInd = (PDeviceDataInd)(pBuffer + iOffset);
		pDeviceDataInd->deviceData.dataType = bDataType;
		pDeviceDataInd->deviceData.restartHandle = 0x0000;

		/* setup BD */
		if ( pL2CChannel == NULL )
		{
			memset( pDeviceDataInd->deviceData.bd, 0, sizeof(TBdAddr) );
			pDeviceDataInd->deviceData.bdType   = BLUEFACE_BDTYPE_ANY;
		}
		else
		{
			pDeviceDataInd->deviceData.bdType   = pL2CChannel->bdType;
			memcpy( pDeviceDataInd->deviceData.bd, &pL2CChannel->RemoteBd, sizeof(TBdAddr) );
		}

		pDeviceDataInd->deviceData.elementCount  = 0;

		if ( bDataType == DEVICE_DATA_TYPE_GATT_CCC_BITS )
		{
			PDEVICE_DATA_ELEMENT_GATT_CCC_BITS  pElement;

			pElement = &pDeviceDataInd->deviceData.p.cccBit[0];
			pElement->cccBits = 0;

			if ( wCommand == DEVICE_DATA_GET_IND )
			{
				if ( wHandle == 0 )
				{
					/* get all CCCDs for given BD */
					pElement->attHandle    = 0;
					pDeviceDataInd->handle = (PVOID)pL2CChannel;
				}
				else
				{
					/* get all BDs for given CCCD */
					pDeviceDataInd->deviceData.elementCount  = 1;

					pElement->attHandle    = wHandle;
					pDeviceDataInd->handle = (PVOID)pAttribUpdData;
				}
		  	}
			else
			{
				if ( iCount >= 0 )
				{
					/* append BDs CCC bits in local table to list */
					if ( iCount > 0 )
					{
						pDeviceDataInd->deviceData.elementCount  = iCount;
						gattCCCBitsOp( CCCBitsAppend, pL2CChannel, (uint8_t *)pElement );
					}
					pDeviceDataInd->handle = (PVOID)pL2CChannel->pRClient;
				}
			}
		}
		else
		{
			/* bDataType == DEVICE_DATA_TYPE_GATT_SRV_CHG_HANDLE */
			if ( wCommand == DEVICE_DATA_SET_IND )
			{
				PDEVICE_DATA_ELEMENT_GATT_SRV_CHG_HANDLE  pElement;

				pElement = &pDeviceDataInd->deviceData.p.srvChg;
				pElement->attHandle = wHandle;
				pElement->indicate  = 0;

				pDeviceDataInd->deviceData.elementCount  = Delete ? 0 : 1;
			}
			pDeviceDataInd->handle = (PVOID)pL2CChannel;
		}

		Message.Command        = wCommand;
		Message.MData.DataCB.BufferAddress = pBuffer;
		Message.MData.DataCB.Offset        = iOffset;
		Message.MData.DataCB.Length        = iLength;
		Message.MData.DataCB.Flag          = DATA_CB_RELEASE;

		blueFaceSendBT_DEVICE_DATA_IND(&Message);

	}

	return( iReturn );
}

/**
* @brief  gatt handle DEVICE_DATA_GET_RESP and DEVICE_DATA_SET_RESP
*
* @param  BufferAddress:
* @param  offset
*
* @return  
*
*/
void gattHandleDEVICE_DATA_RESP(uint16_t cmd,  PDeviceDataResp	pDeviceDataResp)
{
	if ( cmd == DEVICE_DATA_GET_RESP )
	{
		int 				i;
		PGATTL2CChannel 	pL2CChannel    = NULL;

		PGATTAttribUpdData	pAttribUpdData = NULL;
		uint16_t				wCause = GATT_SUCCESS;	/* for GATT_ATTRIB_UPDATE_CONF */
		int 				iCount = 0;
		PDEVICE_DATA_ELEMENT_GATT_CCC_BITS_REV pElementRev = NULL;

		GATT_TRACE_PRINTF_5(GATT_TRACE_MASK_TRACE,
						   "GATT: DEVICE_DATA_GET_RESP, status=0x%x, type=0x%x, BD=%s, count=%d, restart=0x%x",
						   pDeviceDataResp->status, pDeviceDataResp->deviceData.dataType,
						   TRACE_BDADDR1(GATT_TRACE_MASK_TRACE, pDeviceDataResp->deviceData.bd),
						   pDeviceDataResp->deviceData.elementCount,
						   pDeviceDataResp->deviceData.restartHandle
						   );

        for ( i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_don; i++ )
		{
			if ( &pGATT->pL2CChannelDon[i] == (PGATTL2CChannel)pDeviceDataResp->handle )
			{
				pL2CChannel = (PGATTL2CChannel)pDeviceDataResp->handle;
				goto TagFound;
			}
		}
        
        for ( i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_doff; i++ )
		{
			if ( &pGATT->pL2CChannelDoff[i] == (PGATTL2CChannel)pDeviceDataResp->handle )
			{
				pL2CChannel = (PGATTL2CChannel)pDeviceDataResp->handle;
				break;
			}
		}

TagFound:
		if ( pL2CChannel == NULL )
		{
			/* list of BDs was requested, setup GATT_ATTRIB_UPDATE_CONF rsp */
			pAttribUpdData = (PGATTAttribUpdData)pDeviceDataResp->handle;
		} /* else list of handles/CCCD or ServiceChanged characteristic */
		  /* handle/indication flag was requested */
		else
		{
			pL2CChannel->DevDataGetRspPending = FALSE;
			if ( pL2CChannel->L2CState != gattL2CStateConnected )
			{
				/* disconnect race condition */
				if ( pL2CChannel->L2CState == gattL2CStateIdle )
				{
					gattL2CChannelFree( pL2CChannel );
				}
				return;
			}
		}

		if ( pDeviceDataResp->status == BTSEC_SUCCESS )
		{
			/* => pDeviceDataResp->deviceData.elementCount > 0 */
			if ( pL2CChannel != NULL )				   /* connection exists and no more  */
			{
				gattSetChannelReady( pL2CChannel ); /* DEVICE_DATA_GET_IND ops needed */
			}

			if ( pDeviceDataResp->deviceData.dataType == DEVICE_DATA_TYPE_GATT_CCC_BITS )
			{
				if ( pL2CChannel != NULL )
				{
					/* list of handles/CCC bits was requested */
					/* => pDeviceDataResp->deviceData.elementLength == 2*sizeof(uint16_t) */
					int  i;
					PDEVICE_DATA_ELEMENT_GATT_CCC_BITS pElement;

					/* write CCC bits to local table too and sent values to application */
					/* if configured accordingly										  */
					pElement = &pDeviceDataResp->deviceData.p.cccBit[0];

					gattSendGATT_CCCD_IND( pL2CChannel, FALSE,
								  pDeviceDataResp->deviceData.elementCount, pElement );

					for ( i=0; i<pDeviceDataResp->deviceData.elementCount; i++ )
					{
						if ( gattCCCBitsSet( pL2CChannel, FALSE, pElement->attHandle,
																	   pElement->cccBits ) < 0 )
						{
							assert(FALSE);
						}
						pElement++;
					}
				}
			}
			else if (pDeviceDataResp->deviceData.dataType == DEVICE_DATA_TYPE_GATT_CCC_REV_BITS )
			{

				/* list of BDs was requested and BDs that configured the CCCD */
				/* for notifications/indications were found ..				  */
				/* => pDeviceDataResp->deviceData.elementLength == sizeof(TBdAddr)+sizeof(uint8_t) */

				wCause		= GATT_ERR_NOTIF_IND_CFG;
				iCount		= pDeviceDataResp->deviceData.elementCount;
				pElementRev = &pDeviceDataResp->deviceData.p.cccBitRev[0];

			}
			else
			{
				/* dataType == DEVICE_DATA_TYPE_GATT_SRV_CHG_HANDLE */
				/* send ServiceChanged indication to client 		*/
				PDEVICE_DATA_ELEMENT_GATT_SRV_CHG_HANDLE pElement;

				pElement = &pDeviceDataResp->deviceData.p.srvChg;
				if ( pElement->indicate )
				{
					PAttrib  pAttrib;
                    BOOL	 ReleaseBuffer;

					pAttrib = gattHandleToAttribute( pElement->attHandle );
					if ( pAttrib != NULL )
					{
						if ( gattAttribNotifInd( pL2CChannel,
											GATT_CLIENT_CHAR_CONFIG_INDICATE,
											pElement->attHandle, pAttrib, NULL, &ReleaseBuffer) == 0 )
						{
							/* OK, indication was sent */
							pL2CChannel->pRClient->CurrInd.ConfPending	= TRUE;
							pL2CChannel->pRClient->CurrInd.ServiceChanged = TRUE;
							pL2CChannel->pRClient->CurrInd.wApplHandle	= 0;
						}
						else
						{
							/* no buffer .. */
							assert(FALSE); 
						}
				  	}
				}
			}
		}
		else
		{
			if ( pDeviceDataResp->deviceData.dataType == DEVICE_DATA_TYPE_GATT_CCC_BITS )
			{
				/* nothing found resp. error */
				if ( pL2CChannel != NULL )
				{
					/* get ServiceChanged characteristic handle/indication flag */
					gattSendDEVICE_DATA_IND( DEVICE_DATA_GET_IND,
					  DEVICE_DATA_TYPE_GATT_SRV_CHG_HANDLE, FALSE, pL2CChannel, 0, NULL );
				}

				if ( pAttribUpdData != NULL )
				{
					/* list of BDs was requested */
					wCause = GATT_ERR_NOTIF_IND_NOT_CFG;
				}
			}
			else
			{
				/* dataType == DEVICE_DATA_TYPE_GATT_SRV_CHG_HANDLE */
				if ( pL2CChannel != NULL )					 /* connection exists and no more  */
				{
			  		gattSetChannelReady( pL2CChannel ); /* DEVICE_DATA_GET_IND ops needed */
				}
			}
		}

		if ( pAttribUpdData != NULL )
		{
			/* => pL2CChannel == NULL, no connection exists.	   */
			/* setup GATT_ATTRIB_UPDATE_CONF rsp for application */
			pAttribUpdData->cause   = wCause;
			pAttribUpdData->pBuffer = NULL;
			gattSendGATT_ATTRIB_UPDATE_CONF( pAttribUpdData, iCount, pElementRev );
		}

		/* if LL rx data handling was deferred do it now .. */
		if ( (pL2CChannel != NULL) &&
			 (pL2CChannel->DeferredMsgLL.pBuffer != NULL) && pL2CChannel->Ready
		   )
		{
			gattDeferredMsgLLProcess( pL2CChannel );
		}

	}
	else
	{
		/* DEVICE_DATA_SET_RESP */
		GATT_TRACE_PRINTF_5(GATT_TRACE_MASK_TRACE,
							"GATT: DEVICE_DATA_SET_RESP, status=0x%x, type=0x%x, BD=%s, count=%d, restart=0x%x",
							pDeviceDataResp->status, pDeviceDataResp->deviceData.dataType,
							TRACE_BDADDR1(GATT_TRACE_MASK_TRACE, pDeviceDataResp->deviceData.bd),
							pDeviceDataResp->deviceData.elementCount,
							pDeviceDataResp->deviceData.restartHandle
							);

#if (GATT_CCC_BITS_TO_BTSEC_AT_CONN_END)
		/* NOP (pDeviceDataResp->handle may already be invalid !!!) */
#else
		/* resume/finalize write operation */
		gattWriteResumeBTSEC( (PGATTRClient)pDeviceDataResp->handle,
													   pDeviceDataResp->status );
#endif
	}
}

/**
* @brief		send GATT_ATTRIB_READ_IND
*
* @param	pL2CChannel
*
* @return
*
*/
void gattSendGATT_ATTRIB_READ_IND( PGATTL2CChannel pL2CChannel )
{
    uint16_t mdl_id = 0;
	PGATTTransRead      pTransRead     = &pL2CChannel->pRClient->CurrTrans.p.read;
	uint16_t				wAttribIndex   = GATT_ATTR_HANDLE_IDX_GET(pTransRead->wHandle);

    mdl_id = blueAPI_Check_LE_Link(pL2CChannel->cid);
    if(mdl_id != 0)
    {
        blueAPI_Send_GATTAttributeReadInd(pTransRead->pGATTService, wAttribIndex,
		                                  pTransRead->wReadOffset, mdl_id);
    }
}

/**
* @brief		send GATT_ATTRIB_WRITE_IND
*
* @param	pRClient: 
* @param	wType
* @param	iLength 
* @param	wWriteOffset
* @param	pAttValue
*
* @return
*
*/
void gattSendGATT_ATTRIB_WRITE_IND( PGATTRClient pRClient,
                                                 uint16_t         wType,
                                                 int          iLength,
                                                 uint16_t         writeOffset,
                                                 uint8_t *       pAttValue )
{
    uint16_t mdl_id = 0;
    BOOL release = TRUE;
	PGATTTransWrite 	 pTransWrite	 = &pRClient->CurrTrans.p.write;

    mdl_id = blueAPI_Check_LE_Link(pRClient->pL2CChannel->cid);
	

    if(mdl_id != 0)
    {
        release = blueAPI_Send_GATTAttributeWriteInd(pGATT->pBuffer,
                                        pAttValue - pGATT->pBuffer,
                                        wType,
                                        pTransWrite->pGATTService,
	                                    GATT_ATTR_HANDLE_IDX_GET(pTransWrite->wHandle),
	                                    iLength,
	                                    pTransWrite->wHandle,  
	                                    writeOffset, 
                                        mdl_id
                                       );
        if(!release)
	    {
            
	        return;
	    }
    }
    osBufferRelease(pGATT->pBuffer);
}

void gattSendGATT_ATTRIB_PREPARE_WRITE_CONF( uint16_t status, uint16_t cid,
        uint16_t valueOffset, int length, uint8_t * pPartAttValue )
{
    uint16_t     mdl_id = 0;
    int      iOffset = gGattUsWriteOffset;
    uint8_t *   pBuffer;

    mdl_id = blueAPI_Check_LE_Link(cid);

    if(mdl_id)
    {
        if ( osBufferGet(UpstreamPoolID, iOffset+length, (PVOID)&pBuffer) )
		{
			GATT_TRACE_PRINTF_1(GATT_TRACE_MASK_ERROR,
			           "!!GATT: gattSendGATT_CCCD_IND / out of memory, size=%d",
			           iOffset+length);
			assert(FALSE);
			return;
		}
		else
		{
            if (pPartAttValue != NULL)
            {
                memcpy((PVOID)&pBuffer[iOffset], (PVOID)pPartAttValue, length);
            }
            blueAPI_Send_GATTAttributePrepareWriteRsp(NULL, pBuffer,
                                                      iOffset,
                                                      length,
                                                      valueOffset,
                                                      status,
                                                      mdl_id);
		}


    }
}

void gattSendGATT_EXECUTE_WRITE_IND( PGATTL2CChannel pL2CChannel,
                                     uint8_t            bFlags )
{
    uint16_t mdl_id = 0;

    mdl_id = blueAPI_Check_LE_Link(pL2CChannel->cid);
    if(mdl_id)
    {
        blueAPI_Send_GATTExecuteWriteInd(mdl_id, bFlags);
        
    }
}

void gattSendGATT_EXECUTE_WRITE_CONF( uint16_t status, uint16_t cid )
{
    uint16_t mdl_id = 0;

    mdl_id = blueAPI_Check_LE_Link(cid);
    if(mdl_id)
    {
        blueAPI_Send_GATTExecuteWriteRsp(NULL, mdl_id, status);        
    }
}

/**
* @brief		send GATT_CCCD_IND
*
* @param	pL2CChannel: 
* @param	Client
* @param	iCount 
* @param	pElement
*
* @return
*
*/
void gattSendGATT_CCCD_IND( PGATTL2CChannel pL2CChannel, BOOL Client,
                            int iCount, PDEVICE_DATA_ELEMENT_GATT_CCC_BITS pElement )
{
	int                  iOffset;
    uint16_t                 mdl_id = 0;

	/* assure that all msg headers of upper layers fit into buffer: */
	iOffset = gGattUsWriteOffset;

	/* scan through list of handle/CCCD value pairs */
	while( iCount > 0 )
	{
		/* this loop is only executed more than once if the list referenced by pElement */
		/* comprises CCCDs located in services that have been registered with separate  */
		/* GATT_SERVICE_REGISTER_REQ messages (=> serviceHandle not unique !!)          */

		int            iLength;
		uint8_t *         pBuffer;
		PGATTService   pGATTService = NULL;

		iLength = iCount * 2*sizeof(uint16_t);
		if ( osBufferGet(UpstreamPoolID, iOffset+iLength, (PVOID)&pBuffer) )
		{
			GATT_TRACE_PRINTF_1(GATT_TRACE_MASK_ERROR,
			           "!!GATT: gattSendGATT_CCCD_IND / out of memory, size=%d",
			           iOffset+iLength);
			assert(FALSE);
			return;
		}
		else
		{
			uint16_t    wAttribIndex, wNbr;
			int     iPrevNbr;
			LPWORD  pWord;
			int     i, iCountSent;
			int     iFlags = 0;

			pWord      = (LPWORD)(pBuffer + iOffset);  /* start of result list */
			iCountSent = 0;                            /* number of pairs sent */
			iPrevNbr   = -1;

			for ( i=0; i<iCount; i++ )
			{
				/* translate handle into internal service number and attribute index */
				wAttribIndex  = GATT_ATTR_HANDLE_IDX_GET(pElement->attHandle);
				wNbr          = GATT_ATTR_HANDLE_NBR_GET(pElement->attHandle);

				//if ( wNbr < BT_GATT_SERVER_MAX_SERVICES_COUNT )
				if ( wNbr < otp_str_data.gEfuse_UpperStack_s.gatt_max_service_count )
				{
					/* associated service found */
					iFlags = gattCCCBitsGetFlags( pL2CChannel, pElement->attHandle );
					if ( iFlags < 0 )
					{
						iFlags = 0;
					}

					if ( !Client && (iFlags & GATT_CCCBITS_CLIENT_WRITE_ONLY) )
					{
					/* CCCD may be overwritten by client only (not by  DEVICE_DATA_RESP msg) */
					}
					else
					{
						pGATTService  = &pGATT->SrvCl.Server.pService[wNbr];

						/* if requested by application send CCCD value */
						if ( (pGATTService->pAttribFirst + wAttribIndex)->wFlags &
						                                                ATTRIB_FLAG_CCCD_APPL )
						{
							if ( iPrevNbr < 0 )
							{
								iPrevNbr = wNbr;
							}
							else if ( iPrevNbr != wNbr )
							{
								/* service number has changed, use new message !!!! */
								break;
							}
							*(pWord++) = wAttribIndex;
							*(pWord++) = pElement->cccBits;

							iCountSent++;
						}
					}
				}

				pElement++;
			}

			iCount -= i;

			if ( iCountSent > 0 )
			{
				GATT_TRACE_PRINTF_2(GATT_TRACE_MASK_TRACE,
				                  "GATT: CCCD_IND / serviceHandle=0x%x, count=%d",
				                  pGATTService, iCountSent);

                mdl_id = blueAPI_Check_LE_Link(pL2CChannel->cid);

				if(mdl_id != 0)
				{
                    if (!blueAPI_Send_GATTCCCDInfo(pBuffer, iOffset,pGATTService, iCountSent, mdl_id))
				    {
				        continue;
				    }
				}
                osBufferRelease(pBuffer);
			}
			else
			{
				osBufferRelease( pBuffer );
			}
		}
	}
}

/**
* @brief   gatt send le advertise confirm
*
* @param  applHandle
* @param  opCode
* @param  wCause
*
* @return  
*
*/
void gattSendGATT_LE_ADVERTISE_CONF(uint16_t opCode, uint16_t wCause)
{
	TBlueAPI_Cause cause;

    cause = blueAPI_GATTConvertCause(wCause);

	switch (opCode)
	{
	case GATT_LE_ADVERTISE_OPCODE_AD_PARAMETER: 
		blueAPI_Send_LEAdvertiseParameterSetRsp(cause);
		break;

	case GATT_LE_ADVERTISE_OPCODE_AD_DATA:
		blueAPI_Send_LEAdvertiseDataSetRsp(blueAPI_LEDataTypeAdvertisingData, cause);
		break;

	case GATT_LE_ADVERTISE_OPCODE_SC_RSP: 
		blueAPI_Send_LEAdvertiseDataSetRsp(blueAPI_LEDataTypeScanResponseData, cause);
		break;

	case GATT_LE_ADVERTISE_OPCODE_AD_ENABLE: 
		blueAPI_Send_LEAdvertiseRsp(blueAPI_LEAdvModeEnabled, cause);
		break;

	case GATT_LE_ADVERTISE_OPCODE_AD_DISABLE:
		blueAPI_Send_LEAdvertiseRsp(blueAPI_LEAdvModeDisabled, cause);
		break;

	case GATT_LE_ADVERTISE_OPCODE_AD_DIRECTED: 
		blueAPI_Send_LEAdvertiseRsp(blueAPI_LEAdvModeDirectedHighDuty, cause);
		break;

	default:
		break;
	}
}

/**
* @brief  send GATT_MTU_SIZE_IND(mtu size changed indicate)
*
* @param  mtuSize:
* @param  cid
*
* @return  
*
*/
void gattSendGATT_MTU_SIZE_IND( uint16_t mtuSize, uint16_t cid )
{
    uint16_t mdl_id = 0;

    mdl_id = blueAPI_Check_LE_Link(cid);
    if(mdl_id)
    {
        blueAPI_Send_GATTMtuSizeInfo(mdl_id,
								   mtuSize
								   );
        
    }
}

/**
* @brief		send GATT_DISCOVERY_CONF
*
* @param	status: 
* @param	cid
* @param	type
*
* @return
*
*/
void gattSendGATT_DISCOVERY_CONF( uint16_t status, uint16_t cid, uint16_t type )
{
    uint16_t mdl_id = 0;

    mdl_id = blueAPI_Check_LE_Link(cid);

	if (mdl_id)
	{
		TBlueAPI_GATTDiscoveryType   discoveryType;

		switch(type)
		{
		default:
		case GATT_TYPE_DISCOVERY_PSRV_ALL:
			discoveryType = blueAPI_GATTDiscoveryServices;
			break;
		case GATT_TYPE_DISCOVERY_PSRV_UUID:
			discoveryType = blueAPI_GATTDiscoveryServiceByUUID;
			break;
		case GATT_TYPE_DISCOVERY_RELATION:
			discoveryType = blueAPI_GATTDiscoveryRelationship;
			break;
		case GATT_TYPE_DISCOVERY_CHAR_ALL:
			discoveryType = blueAPI_GATTDiscoveryCharacteristics;
			break;
		case GATT_TYPE_DISCOVERY_CHAR_DESCR:
			discoveryType = blueAPI_GATTDiscoveryCharacDescriptors;
			break;
		}
		blueAPI_Send_GATTDiscoveryRsp(NULL,
							 mdl_id,
							 discoveryType,
							 status);
	}
}

/**
* @brief  send GATT_DISCOVERY_IND.
*
* @param  status:
* @param  cid
* @param  type
* @param  count: nbr. of ATT list elements
* @param  length: size of ATT list element
* @param  pList
* @parma  pCurrProc
*
* @return  
*
*/
uint16_t gattSendGATT_DISCOVERY_IND( uint16_t status, uint16_t cid,
                             uint16_t type,
                             uint16_t count, 
                             uint16_t length,   
                             uint8_t * pList, PGATTCurrProc  pCurrProc )
{
    uint8_t *            pBuffer;
    int               iOffset;
	int               i, iSize;
	uint8_t *            pElement;
	uint16_t              wLengthBf;     /* length of Blueface coded list element */
	uint16_t              wLastHandle    = 0;
	uint16_t              wLastHandleRel = 0;

	if ( count == 0 )
	{
		wLengthBf = 0;   /* empty result list */
		iSize     = 0;
	}
	else
	{
		/* determine length of Blueface coded list element */
		switch( type )
		{
		default:
		case GATT_TYPE_DISCOVERY_PSRV_ALL:
			if ( length == (2*sizeof(uint16_t) + UUID_16BIT_SIZE) )
			{
				iSize = sizeof(TGATT_GENERIC_ELEMENT_16);  /* 2 handles + 16 bit UUID */
			}
			else
			{
				iSize = sizeof(TGATT_GENERIC_ELEMENT_128); /* 2 handles + 128 bit UUID */
			}
			break;

		case GATT_TYPE_DISCOVERY_PSRV_UUID:
			iSize = sizeof(TGATT_PSRV_UUID_ELEMENT);
			break;

		case GATT_TYPE_DISCOVERY_RELATION:
			if ( length == (3*sizeof(uint16_t) + UUID_16BIT_SIZE) )
			{
				iSize = sizeof(TGATT_RELATION_ELEMENT_16);  /* ... + 16 bit UUID */
			}
			else
			{
		  		iSize = sizeof(TGATT_RELATION_ELEMENT_128); /* 128 bit UUID not in MTU !! */
			}
			break;

		case GATT_TYPE_DISCOVERY_CHAR_ALL:
			if ( length == (2*sizeof(uint16_t) + UUID_16BIT_SIZE + sizeof(uint8_t)) )
			{
		  		iSize = sizeof(TGATT_CHAR_ALL_ELEMENT_16);  /* ... + 16 bit UUID */
			}
			else
			{
				iSize = sizeof(TGATT_CHAR_ALL_ELEMENT_128); /* ... + 128 bit UUID */
			}
			break;

		  case GATT_TYPE_DISCOVERY_CHAR_DESCR:
		    if ( length == (sizeof(uint16_t) + UUID_16BIT_SIZE) )
			{
				iSize = sizeof(TGATT_CHAR_DESCR_ELEMENT_16);  /* handle + 16 bit UUID */
			}
		    else
			{
				iSize = sizeof(TGATT_CHAR_DESCR_ELEMENT_128); /* handle + 128 bit UUID */
			}
		    break;
		}
		wLengthBf = iSize;
	}

	/* assure that all msg headers of upper layers fit into buffer and that result */
	/* data starts at an even address (avoid conversion of Blueface coded result   */
	/* data in BlueAPI, osBufferGet() returns even buffer address ..):             */
	iOffset = ((gGattUsWriteOffset + 1) & ~0x0001);
	iSize = count*iSize + iOffset;

	if ( osBufferGet(UpstreamPoolID, iSize, (PVOID)&pBuffer) )
	{
		GATT_TRACE_PRINTF_2(GATT_TRACE_MASK_ERROR,
		             "!!GATT: gattSendGATT_DISCOVERY_IND / out of memory, size=%d, cid=0x%x",
		             iSize, cid);
		assert(FALSE);

		return( wLastHandle );

	}
	else
	{
		/* convert discovery result into Blueface format */
		pElement = pBuffer + iOffset;
		for ( i=0; i<count; i++ )
		{
			switch( type )
			{
			default:
			case GATT_TYPE_DISCOVERY_PSRV_ALL:
				/* 2 handles + 16/128 bit UUID:                            */
				if ( length == (2*sizeof(uint16_t) + UUID_16BIT_SIZE) )
				{
					/* 16 bit UUID */
					((PGATT_GENERIC_ELEMENT_16)pElement)[i].attHandle =
					                                             LE_EXTRN2WORD(pList);
					pList += sizeof(uint16_t);
					wLastHandle = LE_EXTRN2WORD(pList);
					((PGATT_GENERIC_ELEMENT_16)pElement)[i].endGroupHandle = wLastHandle;
					pList += sizeof(uint16_t);
					((PGATT_GENERIC_ELEMENT_16)pElement)[i].uuid16 =
					                                           LE_EXTRN2WORD(pList);
					pList += UUID_16BIT_SIZE;
				}
				else
				{
					/* 128 bit UUID */
					((PGATT_GENERIC_ELEMENT_128)pElement)[i].attHandle =
					                                             LE_EXTRN2WORD(pList);
					pList += sizeof(uint16_t);
					wLastHandle = LE_EXTRN2WORD(pList);
					((PGATT_GENERIC_ELEMENT_128)pElement)[i].endGroupHandle = wLastHandle;
					pList += sizeof(uint16_t);
					memcpy( ((PGATT_GENERIC_ELEMENT_128)pElement)[i].uuid128,
					                                       pList, UUID_128BIT_SIZE );
					pList += UUID_128BIT_SIZE;
				}
				break;

			case GATT_TYPE_DISCOVERY_PSRV_UUID:
				/* 2 handles + 16 bit UUID */
				((PGATT_PSRV_UUID_ELEMENT)pElement)[i].attHandle =
				                                         LE_EXTRN2WORD(pList);
				pList += sizeof(uint16_t);
				wLastHandle = LE_EXTRN2WORD(pList);
				((PGATT_PSRV_UUID_ELEMENT)pElement)[i].endGroupHandle = wLastHandle;
				pList += sizeof(uint16_t);
				break;

			case GATT_TYPE_DISCOVERY_RELATION:
				wLastHandleRel = LE_EXTRN2WORD(pList);
				if ( length == (3*sizeof(uint16_t) + UUID_16BIT_SIZE) )
				{
					/* 16 bit UUID */
					((PGATT_RELATION_ELEMENT_16)pElement)[i].declHandle = wLastHandleRel;
					pList += sizeof(uint16_t);
					((PGATT_RELATION_ELEMENT_16)pElement)[i].attHandle =
					                                             LE_EXTRN2WORD(pList);
					pList += sizeof(uint16_t);
					((PGATT_RELATION_ELEMENT_16)pElement)[i].endGroupHandle =
					                                             LE_EXTRN2WORD(pList);
					pList += sizeof(uint16_t);
					((PGATT_RELATION_ELEMENT_16)pElement)[i].uuid16 =
					                                             LE_EXTRN2WORD(pList);
					pList += UUID_16BIT_SIZE;
				}
				else
				{
					/* 128 bit UUID (NOT included in MTU !!) */
					((PGATT_RELATION_ELEMENT_128)pElement)[i].declHandle = wLastHandleRel;
					pList += sizeof(uint16_t);
					((PGATT_RELATION_ELEMENT_128)pElement)[i].attHandle =
					                                             LE_EXTRN2WORD(pList);
					pList += sizeof(uint16_t);
					((PGATT_RELATION_ELEMENT_128)pElement)[i].endGroupHandle =
					                                             LE_EXTRN2WORD(pList);
					/* XXXXMJMJ: not yet fully implemented, applic. can retrieve
					 * 128 bit UUID thru GATT_ATTRIBUTE_READ_REQ with the appropriate
					 * attribute handle (just as on ATT layer ...) and an updated
					 * (search start handle increased by 1 ..) GATT_DISCOVERY_REQ
					 */
					memset( ((PGATT_RELATION_ELEMENT_128)pElement)[i].uuid128,
					                                       0, UUID_128BIT_SIZE );
					pList += UUID_128BIT_SIZE;
				}
				break;

			case GATT_TYPE_DISCOVERY_CHAR_ALL:
				/* declaration handle, properties, value handle + 16/128 bit UUID */
				wLastHandle = LE_EXTRN2WORD(pList);

				if ( length == (2*sizeof(uint16_t) + UUID_16BIT_SIZE + sizeof(uint8_t)) )
				{
					/* 16 bit UUID */
					((PGATT_CHAR_ALL_ELEMENT_16)pElement)[i].declHandle  = wLastHandle;
					pList += sizeof(uint16_t);
					((PGATT_CHAR_ALL_ELEMENT_16)pElement)[i].properties  = *pList;
					pList += sizeof(uint8_t);
					((PGATT_CHAR_ALL_ELEMENT_16)pElement)[i].valueHandle =
					                                              LE_EXTRN2WORD(pList);
					pList += sizeof(uint16_t);
					((PGATT_CHAR_ALL_ELEMENT_16)pElement)[i].uuid16 =
					                                              LE_EXTRN2WORD(pList);
					pList += UUID_16BIT_SIZE;
				}
				else
				{
					/* 128 bit UUID */
					((PGATT_CHAR_ALL_ELEMENT_128)pElement)[i].declHandle  = wLastHandle;
					pList += sizeof(uint16_t);
					((PGATT_CHAR_ALL_ELEMENT_128)pElement)[i].properties  = *pList;
					pList += sizeof(uint8_t);
					((PGATT_CHAR_ALL_ELEMENT_128)pElement)[i].valueHandle =
					                                              LE_EXTRN2WORD(pList);
					pList += sizeof(uint16_t);
					memcpy( ((PGATT_CHAR_ALL_ELEMENT_128)pElement)[i].uuid128,
					                                       pList, UUID_128BIT_SIZE );
					pList += UUID_128BIT_SIZE;
				}
				break;

			case GATT_TYPE_DISCOVERY_CHAR_DESCR:
				wLastHandle = LE_EXTRN2WORD(pList);
				pList += sizeof(uint16_t);

				if ( length == (sizeof(uint16_t) + UUID_16BIT_SIZE) )
				{
					/* handle + 16 bit UUID */
					((PGATT_CHAR_DESCR_ELEMENT_16)pElement)[i].handle = wLastHandle;
					((PGATT_CHAR_DESCR_ELEMENT_16)pElement)[i].uuid16 =
					                                              LE_EXTRN2WORD(pList);
					pList += UUID_16BIT_SIZE;
				}
				else
				{
					/* handle + 128 bit UUID */
					((PGATT_CHAR_DESCR_ELEMENT_128)pElement)[i].handle = wLastHandle;
					memcpy( ((PGATT_CHAR_DESCR_ELEMENT_128)pElement)[i].uuid128,
					                                       pList, UUID_128BIT_SIZE );
					pList += UUID_128BIT_SIZE;
				}
				break;
			}
		}
	}

    
	do{
        uint16_t mdl_id = 0;

        mdl_id = blueAPI_Check_LE_Link(cid);
        if ( type == GATT_TYPE_DISCOVERY_RELATION )
		{
			wLastHandle = wLastHandleRel;
		}
		if(pCurrProc)
		{
			pCurrProc->p.discovery.wLastHandle = wLastHandle;
		}
        if(mdl_id != 0)
        {
          	if( !blueAPI_Send_GATTDiscoveryInd(pBuffer, iOffset,type, count, wLengthBf,
		                                           status, mdl_id) )
		    {
		        break;
		    }  
        }
        
		osBufferRelease(pBuffer);
	}while(0);
	
	return( wLastHandle );
}

/**
* @brief		send GATT_ATTRIB_READ_CONF
*
* @param	status
* @param	cid
* @param	type
* @param	iLength: length of PDU data part
* @param	iTruncated:  nbr. of bytes that have been truncated
* @param	pData: PDU data part
* @param	iNbrOfHandles: nbr. handles referenced by pHandle or pData
* @param	pHandles
* @param	iValueOffset: read offset in attribute 
*
* @return
*
*/
void gattSendGATT_ATTRIB_READ_CONF( uint16_t status, uint16_t cid, uint16_t type,
			               int    iLength,        
			               int    iTruncated,                                     
			               uint8_t * pData,          
			               int    iNbrOfHandles,                                        
			               LPWORD pHandles,                          
			               int    iValueOffset    
			              )
{

	int                 iSize, iValueLength;
	int                 iOffset;
	uint8_t *  pBuffer;

	/* assure that all msg headers of upper layers fit into buffer and that result */
	/* data starts at an even address (avoid conversion of Blueface coded result   */
	/* data in BlueAPI, osBufferGet() returns even buffer address ..):             */
	iOffset = ((gGattUsWriteOffset + 1) & ~0x0001);
	if ( iOffset < offsetof(TGATTAttribReadConfParam,handlesData) )
	{
		return;
	}

	iSize = iOffset + iLength;
	if ( (uint8_t *)pHandles == pData )
	{
	/* handles are stored in "Read By Type Response" PDU data part */
	/* => iLength counts handle sizes already */
	}
	else
	{
		iSize += iNbrOfHandles*sizeof(uint16_t);
	}

	if ( osBufferGet(UpstreamPoolID, iSize, (PVOID)&pBuffer) )
	{
		GATT_TRACE_PRINTF_2(GATT_TRACE_MASK_ERROR,
		             "!!GATT: gattSendGATT_ATTRIB_READ_CONF / out of memory, size=%d, cid=0x%x",
		             iSize, cid);
		assert(FALSE);
		return;
	}

	iValueLength = 0;

	if ( iLength != 0 )
	{
		uint8_t *  pDst;
		uint8_t *  pDstValue;
		int     i, iBytesToCopy;

		pDst = (uint8_t *)(pBuffer + iOffset);

		if ( (uint8_t *)pHandles == pData )
		{
			/* handles are stored in "Read By Type Response" PDU data part */
			/* insert handles and attribute value(s) in msg:               */
			iValueLength = (iLength + iTruncated)/iNbrOfHandles - sizeof(uint16_t);
			iBytesToCopy = iValueLength;
			pDstValue    = pDst + iNbrOfHandles*sizeof(uint16_t);

			for ( i=0; i<iNbrOfHandles; i++ )
			{
				*((LPWORD)pDst) = LE_EXTRN2WORD(pData);
				pDst  += sizeof(uint16_t);
				pData += sizeof(uint16_t);
				if ( (iTruncated != 0) && (i == (iNbrOfHandles-1)) )
				{
					iBytesToCopy -= iTruncated;
				}
				memcpy( pDstValue, pData, iBytesToCopy );
				pDstValue += iBytesToCopy;
				pData     += iBytesToCopy;
			}
		}
		else
		{
			iValueLength = iLength/iNbrOfHandles;

			/* handles are stored in local array */
			/* insert handles in msg:            */
			for ( i = 0; i < iNbrOfHandles; i++ )
			{
				*((LPWORD)pDst) = *(pHandles++);
				pDst  += sizeof(uint16_t);
			}

			/* copy attribute value(s) behind handles */
			memcpy( pDst, pData, iLength );
			iLength += iNbrOfHandles*sizeof(uint16_t);   /* total length incl. handles */
		}
	}
    
	{
        uint16_t mdl_id = 0;
        BOOL release = TRUE;
        mdl_id = blueAPI_Check_LE_Link(cid);
        

        if(mdl_id)
        {
            TBlueAPI_GATTReadType   readType;

		    switch(type)
		    {
	        default:
	        case GATT_READ_TYPE_BASIC:
	        case GATT_READ_TYPE_BLOB:
				readType = blueAPI_GATTReadTypeBasic;
				break;
	        case GATT_READ_TYPE_TYPE:
				readType = blueAPI_GATTReadTypeByUUID;
				break;
		    }

            release = blueAPI_Send_GATTAttributeReadRsp(NULL, pBuffer, iOffset,
                                       readType, iValueOffset, iLength,
	                                   iValueLength, iNbrOfHandles, status,mdl_id);

            if(! release)
                return;
        }
        osBufferRelease(pBuffer);
	}
}

/**
* @brief		send GATT_ATTRIB_WRITE_CONF   
*
* @param	status: 
* @param	cid
* @param	type
* @param	length
* @param	pPartAttValue
*
* @return
*
*/
BOOL gattSendGATT_ATTRIB_WRITE_CONF( uint16_t status, uint16_t cid,
                                     uint16_t type, int length, uint8_t * pPartAttValue )
{
    uint16_t mdl_id = 0;

    mdl_id = blueAPI_Check_LE_Link(cid);

    if(mdl_id)
    {
        TBlueAPI_GATTWriteType	 writeType;

		switch( type )
		{
		default:
		case GATT_WRITE_TYPE_REQ:
			writeType = blueAPI_GATTWriteTypeRequest;
			break;
		case GATT_WRITE_TYPE_CMD:
			writeType = blueAPI_GATTWriteTypeCommand;
			break;
		}
        blueAPI_Send_GATTAttributeWriteRsp(NULL,mdl_id,
											writeType,
											status,
											0/*length*/);
    }
    return TRUE;
}

/**
* @brief  send GATT_ATTRIB_INDICATION_IND / GATT_ATTRIB_NOTIFICATION_INFO
*
* @param  cid:
* @param  Notify
* @param  wHandle
* @param  iLength
* @param  pValue
*
* @return  
*
*/
void gattSendGATT_ATTRIB_NOTIF_IND( uint16_t cid, BOOL Notify,
                                  uint16_t wHandle, int iLength, uint8_t * pValue )
{
    uint16_t mdl_id = 0;

    mdl_id = blueAPI_Check_LE_Link(cid);

    if(mdl_id)
    {
        if(!blueAPI_Send_GATTAttributeNotifInd(pGATT->pBuffer, pValue - pGATT->pBuffer,
                                               Notify, wHandle, iLength, mdl_id) )
        {
            return;
        }
    }

    osBufferRelease(pGATT->pBuffer);
}

/**
* @brief  gatt handle le connect update confirm
*
* @param  pL2CChannel
* @param  status
*
* @return  
*
*/
void gattSendGATT_LE_CONNECTION_UPDATE_CONF(uint16_t update_cause, uint16_t cid)
{
    uint16_t mdl_id = 0;

    mdl_id = blueAPI_Check_LE_Link(cid);

    if(mdl_id)
    {
        blueAPI_Send_LEConnectionUpdateRsp(mdl_id, update_cause);
    }
}

/**
* @brief  send gatt le connection update indicate
*
* @param  pL2CChannel
* @param  type
* @param  pParam
*
* @return  
*
*/
void gattSendGATT_LE_CONNECTION_UPDATE_IND(PGATTL2CChannel pL2CChannel,
                                           uint16_t type, PGATTLEConnectionUpdateParam pParam)
{
    blueAPI_Handle_BT_GATT_LE_CONNECTION_UPDATE_IND(pL2CChannel->cid, type, pParam);
}

/**
* @brief  gatt modify whitelist confirm
*
* @param  appHandle
* @param  opCode
* @param  cause
*
* @return  
*
*/
void gattSendGATT_LE_MODIFY_WHITELIST_CONF(uint16_t opCode, uint16_t cause)
{
	TBlueAPI_Cause param_cause;

	param_cause = blueAPI_GATTConvertCause(cause);

	blueAPI_Send_LEModifyWhitelistRsp((TBlueAPI_LEWhitelistOp)opCode,
									  param_cause
									  );

}

void gattHandleGATT_SERVICE_REGISTER_REQ(uint16_t nbrOfAttrib, uint8_t * pService)
{
	int					iReturn;
	PGATTService		pGATTService = NULL;
#if F_BT_BREDR_SUPPORT    
	PGATTDeferredMsg	pDeferredMsg = NULL;
#endif
	uint16_t				wCause	   = GATT_ERR_OUT_OF_RESOURCE;
	

	if ( (wCause = gattServiceCheck( (PAttrib)pService,
				  nbrOfAttrib )) != GATT_SUCCESS
	 )
	{
		/* coding of service not OK or service already registered */
	}
	else
	{
		/* allocate profile/service descriptor */
		wCause = GATT_ERR_OUT_OF_RESOURCE;
		if ( (pGATTService = gattServiceAlloc( )) != (PGATTService)0 )
		{
#if F_BT_BREDR_SUPPORT
			BOOL	 SDP = FALSE;

			/* save REQ msg, CONF must be deferred until SDP ops are completed */
			pDeferredMsg = osQueueOut( &pGATT->DeferredMsgQueueFree );

			if ( pDeferredMsg != NULL )
			{
				pDeferredMsg->pGATTService = pGATTService;

				if (
#if (!GATT_NO_BUILTIN_SERVICE)
					 (pGATT->SrvCl.Server.pService[0].Sdp.State != SdpStateRegistered) ||
#endif
					 (pGATT->DeferredMsgQueue.Count != 0)
				   )
				{
					/* creation of services SDP records still in progress */
					SDP = TRUE;
				}

				/* create SDP record if needed, more service preparations .. */
                pGATTService->pAttribFirst  = (PAttrib)pService;
	            pGATTService->wAttribCnt    = nbrOfAttrib;
				iReturn = gattServicePrepare( pGATTService,
												SDP ? NULL : pDeferredMsg );
				if ( iReturn <= 0 )
				{
					/* error or no SDP op needed: immediate rsp, release descriptor */
					//osQueueIn( &pGATT->DeferredMsgQueueFree, pDeferredMsg );
					pDeferredMsg = NULL;
					if ( iReturn == 0 )
					{
						wCause = GATT_SUCCESS;
					}
				}
				if ( iReturn > 0 )
				{
					/* SDP op in progress or first service register req., save descriptor */
					wCause = GATT_SUCCESS;
					if(SDP)
					{
						osQueueIn( &pGATT->DeferredMsgQueue, pDeferredMsg );
					}
				}
			}
#else
            pGATTService->pAttribFirst  = (PAttrib)pService;
	        pGATTService->wAttribCnt    = nbrOfAttrib;
			iReturn = gattServicePrepare( pGATTService, NULL);
            if (iReturn == 1)
            {
                /* send immediate rsp */
                wCause = GATT_SUCCESS;
                gattSendGATT_SERVICE_REGISTER_CONF( wCause, pGATTService);
            }
#endif
		}
	}

	if ( (wCause != GATT_SUCCESS) && (pGATTService != NULL) )
	{
		/* free resources allocated so far .. */
		gattServiceFree( pGATTService );
		pGATTService = NULL;
	}

	if ( (wCause != GATT_SUCCESS)
#if F_BT_BREDR_SUPPORT
        || (pDeferredMsg == NULL) 
#endif
        )
	{
		/* send immediate rsp */
		gattSendGATT_SERVICE_REGISTER_CONF( wCause, pGATTService);
	}

    return;
}  

void gattHandleGATT_ATTRIB_UPDATE_REQ(uint8_t * pBuffer, PVOID reqHandle, PVOID serviceHandle,
                                      uint16_t attribIndex, uint16_t length, uint16_t offset)
{
	PGATTService  pGATTService;
	PAttrib 	  pAttrib;
	int 		  iIndex, iProperties;

	PGATTL2CChannel 		  pL2CChannel;
	int 					  iCCCBits = 0;
	uint16_t					  wHandle  = 0;
	uint16_t					  wCause = GATT_ERR_ILLEGAL_CODING;
	TGATTAttribUpdData		  AttribUpdData;
	PGATTAttribUpdData		  pAttribUpdData = &AttribUpdData;
	PGATTTxData 			  pTxData = NULL;
	BOOL					  ReleaseBuffer = TRUE;
	
	pGATTService = (PGATTService)serviceHandle;
	iIndex		 = attribIndex;
	pAttrib 	 = pGATTService->pAttribFirst + iIndex - 1;
	wHandle 	 = pGATTService->wHandleStart + iIndex;

	/* save parameters for (deferred) transmission of data and/or CONF */
	pAttribUpdData->reqHandle	  = reqHandle;
	pAttribUpdData->pGATTService  = pGATTService;
	pAttribUpdData->attribIndex   = attribIndex;
	/* pAttribUpdData->attribHandle  = wHandle; */
	pAttribUpdData->length		  = length;
	pAttribUpdData->pBuffer 	  = pBuffer;
	pAttribUpdData->offset		  = offset;

	  GATT_TRACE_PRINTF_4(GATT_TRACE_MASK_TRACE,
		   "GATT: ATTR_UPD_REQ /  reqHandle=0x%x, serviceHandle=0x%x, attribIndex=%d, attribHandle=0x%x",
		 /*  applHandle,*/ reqHandle, pGATTService, iIndex, wHandle);

	/* do some basic checks */
	if ( iIndex >= pGATTService->wAttribCnt )
	{
		wCause = GATT_ERR_ILLEGAL_PARAMETER;
	}
	else
	{
		/* check if indications/notifications are principally */
		/* allowed (char. def. precedes value:)				*/
		if ( (iProperties = gattCharPropertiesGet( pAttrib )) >= 0 )
		{
			if ( (iProperties > 0) &&
				 (iProperties & (GATT_CHAR_PROP_INDICATE | GATT_CHAR_PROP_NOTIFY))
			   )
			{
				/* check if authentication is needed */
				uint8_t	bPerm;

				wCause = GATT_SUCCESS;
				pAttrib++;					/* points to attribute value now */

				/* find established connection */
				iCCCBits = gattL2CChannelFindConnIndNotif( wHandle, &pL2CChannel );
				if ( pL2CChannel != NULL )
				{
					bPerm = GATT_PERM_NOTIF_IND_AUTHEN_GET(pAttrib->wPermissions);

					switch( bPerm )
					{
					default:
						/* no authentication required */
						break;
					case GATT_PERM_AUTHEN_REQ:
						if ( !(pL2CChannel->pRClient->bSecurityFlags & GATT_RCLIENT_SEC_AUTHEN) )
						{
					  		wCause = GATT_ERR_NOT_AUTHENTICATED;
						}
						break;
					case GATT_PERM_AUTHEN_MITM_REQ:
						if ( !(pL2CChannel->pRClient->bSecurityFlags & GATT_RCLIENT_SEC_AUTHEN_MITM) )
						{
							wCause = GATT_ERR_NOT_AUTHENTICATED;
						}
						break;
					}
				}
			}
			else
			{
				wCause = GATT_ERR_NOT_ALLOWED;
			}
		}
	}

	if ( wCause == GATT_SUCCESS )
	{
		/* check if attribute should be indicated/notified */
		switch( iCCCBits )
		{
        case GATT_CLIENT_CHAR_CONFIG_DEFAULT:
			/* connection exists, not configured for indications/notifications .. */
			wCause = GATT_ERR_NOTIF_IND_NOT_CFG;
			break;

		case GATT_CLIENT_CHAR_CONFIG_INDICATE:
			/* CONF is generated immediately, final confirmation */
			/* is received from client						   */
			wCause = GATT_ERR_NOTIF_IND_CONF_PD;
			break;

		default:
			if ( iCCCBits < 0 )
			{
				/* no established connection exists, get list of BDs that  */
				/* registered for notif./indications. use pBuffer as flag  */
				/* indicating that a request is already in progress (does  */
				/* NOT point to buffer !!). wsize=1 for this type of req.! */
				if ( pGATT->pServer->AttribUpdData.pBuffer != NULL )
				{
			 		wCause = GATT_ERR_ILLEGAL_STATE;
				}
				else
				{
			  		pAttribUpdData = &pGATT->pServer->AttribUpdData;
				}
			}
			else
			{
				/* established connection exists, send notification */
				pTxData = osQueueOut( &pL2CChannel->TxDataQueueFree );
				if ( pTxData != NULL )
				{
					pTxData->TxDataType = TxTypeNotification;
					pAttribUpdData	  = &pTxData->p.AttribUpdData;
				}
				else
				{
					wCause = GATT_ERR_OUT_OF_RESOURCE;
				}
			}
			break;
		}

		if ( (wCause == GATT_SUCCESS) ||			   /* notification or retrieval of BDs */
		   (wCause == GATT_ERR_NOTIF_IND_CONF_PD)  /* indication */
		 )
		{
			/* save data for deferred processing or CONF generation  */
			/* (triggered by tx buffer callback or DEVICE_DATA_RESP) */
			if ( wCause == GATT_SUCCESS )
			{
		  		*pAttribUpdData = AttribUpdData;
			}

			if ( iCCCBits > GATT_CLIENT_CHAR_CONFIG_DEFAULT )
			{
				/* established connection exists, send indication/notification */
				int  iReturn;

				if ( (iCCCBits == GATT_CLIENT_CHAR_CONFIG_INDICATE) &&
								pL2CChannel->pRClient->CurrInd.ConfPending )
				{
					/* transaction not yet completed */
					wCause = GATT_ERR_ILLEGAL_STATE;
				}
				else
				{
					iReturn = gattAttribNotifInd( pL2CChannel, iCCCBits, wHandle,
												pAttrib, pAttribUpdData, &ReleaseBuffer);
					if ( iReturn < 0 )
					{
						wCause = GATT_ERR_OUT_OF_RESOURCE;
                        ReleaseBuffer = TRUE;
					}
					else
					if ( iReturn > 0 )
					{
						/* link deactivated due to preceding timeout */
						wCause = GATT_ERR_LINK_DEACTIVATED;
                        ReleaseBuffer = TRUE;
					}
					else
					if ( iCCCBits == GATT_CLIENT_CHAR_CONFIG_INDICATE )
					{
						/* indication, save relevant data for GATT_ATTRIB_UPDATE_STATUS_IND */
						pL2CChannel->pRClient->CurrInd.ConfPending	= TRUE;

						pL2CChannel->pRClient->CurrInd.ServiceChanged = FALSE;


						pL2CChannel->pRClient->CurrInd.wAttribIndex	= iIndex;
						pL2CChannel->pRClient->CurrInd.pReqHandle 	=
														 pAttribUpdData->reqHandle;
						pL2CChannel->pRClient->CurrInd.pServiceHandle =
														 (PVOID)pAttribUpdData->pGATTService;
					}
					else
					{
						/* notification (no client acknowledgement) */
					}
				}
			}
			else
			{
				uint16_t	wHandleCCCD;

				/* no established connection exists */
				/* get list of BDs that registered for notif./indications */

				/* move forward to CCCD: */
				pAttrib = gattGoToAttribute( 1, wHandle,
						   GATT_UUID_CHAR_CLIENT_CONFIG, &wHandleCCCD );

				if ( pAttrib != NULL )
				{
					/* get list of BDs that configured this CCCD for notif. or inds. */
					if ( gattSendDEVICE_DATA_IND( DEVICE_DATA_GET_IND,
												  DEVICE_DATA_TYPE_GATT_CCC_BITS,
											 FALSE, NULL, wHandleCCCD, pAttribUpdData ) != 0 )
					{
					  wCause = GATT_ERR_OUT_OF_RESOURCE;
					}
				}
				else
				{
					/* no CCCD for this characteristic value */
					wCause = GATT_ERR_NOTIF_IND_NOT_CFG;
				}
			}
		}
	}

	if ( wCause == GATT_SUCCESS )
	{
		if ( pTxData != NULL )
		{
			osQueueIn( &pL2CChannel->TxDataQueue, pTxData );
		}
	}
	else
	{
		/* setup rsp, otherwise GATT_ATTRIB_UPDATE_CONF is generated in */
		/* tx buffer callback or DEVICE_DATA_RESP 					  */
		pAttribUpdData->cause = wCause;
		gattSendGATT_ATTRIB_UPDATE_CONF( pAttribUpdData, 0, NULL );

		if ( pTxData != NULL )
		{
			osQueueIn( &pL2CChannel->TxDataQueueFree, pTxData );
		}
	}

	if ( ReleaseBuffer )
	{
		osBufferRelease( pBuffer );
	}

    return;   /* msg buffer will not be released by callers */
}

void gattHandleGATT_ATTRIB_READ_RESP( uint16_t cid , uint16_t cause, uint16_t length, 
                                      PGATTAttribReadRespParam pParm)
{
	PGATTL2CChannel     pL2CChannel;
	pL2CChannel = gattL2CChannelFindHandle( FALSE, cid);
	if ( pL2CChannel == NULL )
	{
		assert(FALSE);
		return;
	}

	/* resume/complete read operation */
	gattReadResumeAppl( pL2CChannel, cause, length, (PGATTAttribReadRespParam)pParm);
	osBufferRelease( pParm );
}

void gattHandleGATT_ATTRIB_WRITE_RESP(uint16_t cid, uint16_t cause, uint16_t length,
                                              uint16_t        offset,       
                                              uint8_t *      pBuffer)
{
    PGATTL2CChannel      pL2CChannel;

	pL2CChannel = gattL2CChannelFindHandle( FALSE, cid);
	if ( pL2CChannel == NULL )
	{
		assert(FALSE);
		return;
	}

	/* resume/complete write operation */
	gattWriteResumeAppl( pL2CChannel, cause, length, offset, pBuffer);
    if(pBuffer != NULL)
    {
        osBufferRelease(pBuffer);
    }
}

void gattHandleGATT_DISCOVERY_REQ(PGATTDiscoveryReq  pDiscoveryReq)
{
	PGATTL2CChannel    pL2CChannel;
	uint16_t			   wStatus	= GATT_SUCCESS;

	/* trigger lower layer rsp */
	pL2CChannel = gattL2CChannelFindHandle( FALSE, pDiscoveryReq->cid);
	if ( pL2CChannel == NULL )
	{
		wStatus = GATT_ERR_INTERNAL;
	}
	else if ( (pL2CChannel->L2CState != gattL2CStateConnected) ||
	   (pL2CChannel->pClient->CurrProc.bOpCode != 0)
	 )
	{
		wStatus = GATT_ERR_ILLEGAL_STATE;
	}
	else
	{
		if ( (pDiscoveryReq->p.common.startingHandle == 0) ||
			 (pDiscoveryReq->p.common.startingHandle > pDiscoveryReq->p.common.endingHandle)
		   )
		{
			wStatus = GATT_ERR_ILLEGAL_PARAMETER;
		}
		else
		{
			/* start GATT procedure */
			switch( pDiscoveryReq->type )
			{
			default:
				wStatus = GATT_ERR_ILLEGAL_PARAMETER;
				break;
			case GATT_TYPE_DISCOVERY_PSRV_ALL:
			case GATT_TYPE_DISCOVERY_PSRV_UUID:
				wStatus = gattDiscoveryPrimaryService( pL2CChannel,
														pDiscoveryReq, NULL );
				break;
			case GATT_TYPE_DISCOVERY_RELATION:
			case GATT_TYPE_DISCOVERY_CHAR_ALL:
			case GATT_TYPE_DISCOVERY_CHAR_DESCR:
				wStatus = gattDiscoveryOthers( pL2CChannel,
														pDiscoveryReq, NULL  );
				break;
			}
		}
	}
	/* send local confirmation, results will be signaled in GATT_DISCOVERY_IND */
	gattSendGATT_DISCOVERY_CONF( wStatus, pDiscoveryReq->cid, pDiscoveryReq->type );

    return;
}

void gattHandleGATT_DISCOVERY_RESP(uint16_t cid, uint16_t type, uint16_t startingHandle, uint16_t endingHandle)
{
	PGATTL2CChannel 	pL2CChannel;
	uint16_t				wStartingHandle = 0;
	
	pL2CChannel = gattL2CChannelFindHandle( FALSE, cid);
	if ( pL2CChannel == NULL )
	{
	  /* discard msg */
	}
	else
	if ( (startingHandle != 0) ||
		 (endingHandle != 0) )
	{
		PGATTProcDiscovery  pProcDisc = &pL2CChannel->pClient->CurrProc.p.discovery;

		if ( pProcDisc->wLastHandle == 0xFFFF )
		{
			/* service signaled to applic. was last one, fake -rsp from peer */
			gattSendGATT_DISCOVERY_IND( ATT_ERR | ATT_ERR_ATTR_NOT_FOUND,
												pL2CChannel->cid,
												type,
												0,	/* nbr. of list elements  */
												0,	/* length of list element */
												NULL, NULL );
		}
		else
		{
			if ( startingHandle == 0 )
			{
				/* resume search with next handle */
				wStartingHandle = pProcDisc->wLastHandle + 1;
			}
			else
			{
				wStartingHandle = startingHandle;
			}

			if ( (wStartingHandle != 0) &&
				 (wStartingHandle <= endingHandle)
			   )
			{
				TGATTDiscoveryParam  Param;

				/* prepare parameters for search continuation */
				Param.wType			= type;
				Param.wStartingHandle = wStartingHandle;
				Param.wEndingHandle	= endingHandle;
				Param.wUUIDType		= pProcDisc->wUUIDType;
				Param.pUUID			= pProcDisc->UUID.b16;	/* OK for 128 bit too */

				switch( type )
				{
				default:
					break;
				case GATT_TYPE_DISCOVERY_PSRV_ALL:
				case GATT_TYPE_DISCOVERY_PSRV_UUID:
					gattDiscoveryPrimaryService( pL2CChannel, NULL, &Param );
					break;
				case GATT_TYPE_DISCOVERY_RELATION:
				case GATT_TYPE_DISCOVERY_CHAR_ALL:
				case GATT_TYPE_DISCOVERY_CHAR_DESCR:
					gattDiscoveryOthers( pL2CChannel, NULL, &Param );
					break;
				}
			}
		}
	} /* else search terminated */

    return;
}


void gattHandleGATT_ATTRIB_READ_REQ( PGATTAttribReadReq  pAttribReadReq)
{
	PGATTL2CChannel	  pL2CChannel;
	uint8_t *			  pUuid128 = NULL;
	uint16_t				  wStatus  = GATT_SUCCESS;

	if ( (pAttribReadReq->type == GATT_READ_TYPE_TYPE) &&
	   (pAttribReadReq->p.type.uuidType == UUID_TYPE_128)
	 )
	{
		pUuid128 = pAttribReadReq->p.type.uuid.pUuid128;
		if ( pUuid128 == NULL )
		{
			/* reject */
			wStatus = GATT_ERR_OUT_OF_RESOURCE;
		}
	}

	if ( wStatus == GATT_SUCCESS )
	{
		pL2CChannel = gattL2CChannelFindHandle( FALSE, pAttribReadReq->cid );
		if ( pL2CChannel != NULL )
		{
			if ( (pL2CChannel->L2CState != gattL2CStateConnected) ||
			   (pL2CChannel->pClient->CurrProc.bOpCode != 0)
			 )
			{
				wStatus = GATT_ERR_ILLEGAL_STATE;
			}
			else
			{
				switch( pAttribReadReq->type )
				{
				default:
					wStatus = GATT_ERR_ILLEGAL_PARAMETER;
					break;
				case GATT_READ_TYPE_BASIC:
					if ( pAttribReadReq->p.basic.handle == 0 )
					{
						wStatus = GATT_ERR_ILLEGAL_PARAMETER;
					}
					break;
				case GATT_READ_TYPE_BLOB:
					if ( pAttribReadReq->p.blob.handle == 0 )
					{
						wStatus = GATT_ERR_ILLEGAL_PARAMETER;
					}
					break;
				case GATT_READ_TYPE_TYPE:
					if ( (pAttribReadReq->p.type.startingHandle == 0) ||
						 ((pAttribReadReq->p.type.startingHandle == 0) >
						  (pAttribReadReq->p.type.endingHandle == 0))
					   )
					{
						wStatus = GATT_ERR_ILLEGAL_PARAMETER;
					}
					break;
				}
			}

			if ( wStatus == GATT_SUCCESS )
			{
				wStatus = gattAttribRead( pL2CChannel, pAttribReadReq );
			}
		}
		else
		{
			wStatus = GATT_ERR_INTERNAL;
		}
	}

	if ( wStatus != GATT_SUCCESS )
	{
		gattSendGATT_ATTRIB_READ_CONF( wStatus, pAttribReadReq->cid,
							   pAttribReadReq->type, 0, 0, NULL, 0, NULL, 0 );
	}
}

void gattHandleGATT_ATTRIB_WRITE_REQ(uint16_t cid, uint16_t type, uint16_t handle, uint16_t length,
                                     PGATTAttribWriteReqParam  pParam)
{
    int s;
    PGATTL2CChannel 	 pL2CChannel;
	PGATTTxData 		 pTxData = NULL;
	BOOL				 ReleaseBuffer = TRUE;
	uint16_t				 wStatus = GATT_SUCCESS;
    
    pL2CChannel = gattL2CChannelFindHandle( FALSE, cid );
	if ( pL2CChannel != NULL )
	{
		TGATTAttribWriteData		AttribWriteData;
		PGATTAttribWriteData		pAttribWriteData = &AttribWriteData;

		if ( (pL2CChannel->L2CState != gattL2CStateConnected) ||
		   ((type != GATT_WRITE_TYPE_CMD) &&
			(pL2CChannel->pClient->CurrProc.bOpCode != 0)
		   )
		 )
		{
			wStatus = GATT_ERR_ILLEGAL_STATE;
		}
		else
		{
			if (handle == 0)
			{
				wStatus = GATT_ERR_ILLEGAL_PARAMETER;
			}
			else
			{
				/* save parameters for (deferred) transmission of data and/or CONF */
				pAttribWriteData->length	= length;
				pAttribWriteData->pBuffer = (uint8_t *)pParam;
				pAttribWriteData->offset	= offsetof(TGATTAttribWriteReqParam, data)
																		  + pParam->offset;
				switch( type )
				{
				default:
					wStatus = GATT_ERR_ILLEGAL_PARAMETER;
					break;
				case GATT_WRITE_TYPE_CMD:
					/* no response from server needed, allow bursts ...			  */
					/* save parameters for (deferred) data transmission and/or CONF */
                    s = osInterruptDisable();
					pTxData = osQueueOut( &pL2CChannel->TxDataQueueFree );
					if ( pTxData != NULL )
					{
						pTxData->TxDataType = TxTypeWriteCommand;
						pAttribWriteData	= &pTxData->p.AttribWriteData;
						*pAttribWriteData	= AttribWriteData;
					}
					else
					{
						wStatus = GATT_ERR_OUT_OF_RESOURCE;
					}
                    osInterruptEnable(s);
					break;
				case GATT_WRITE_TYPE_REQ:

                case GATT_WRITE_TYPE_PREP:

					break;
				}
			}
		}

		if ( wStatus == GATT_SUCCESS )
		{
			wStatus = gattAttribWrite( pL2CChannel,
												  type, handle,pParam->writeOffset, pAttribWriteData );
			if ( wStatus == GATT_SUCCESS )
			{
				ReleaseBuffer = FALSE;
                s = osInterruptDisable();
				if ( pTxData != NULL )
				{
					osQueueIn( &pL2CChannel->TxDataQueue, pTxData );
				}
                osInterruptEnable(s);
			}
		}
	}
	else
	{
		wStatus = GATT_ERR_INTERNAL;
	}

	if ( wStatus != GATT_SUCCESS )
	{
        if (type == GATT_WRITE_TYPE_PREP)
        {
            gattSendGATT_ATTRIB_PREPARE_WRITE_CONF( wStatus, cid,
                                                    0, 0, NULL );
        }
        else
        {
		    gattSendGATT_ATTRIB_WRITE_CONF( wStatus, cid,
													  type, 0, NULL );
        }
        s = osInterruptDisable();
		if ( pTxData != NULL )
		{
			osQueueIn( &pL2CChannel->TxDataQueueFree, pTxData );
		}
        osInterruptEnable(s);
	}

	if ( ReleaseBuffer )
	{
		osBufferRelease( (PVOID)pParam );
	}
}

void gattHandleGATT_EXECUTE_WRITE_REQ( uint16_t cid, uint8_t flags)
{
    PGATTL2CChannel       pL2CChannel;
    uint16_t                  wStatus = GATT_SUCCESS;

    pL2CChannel = gattL2CChannelFindHandle( FALSE, cid );
    if ( pL2CChannel != NULL )
    {
        if ( (pL2CChannel->L2CState != gattL2CStateConnected) ||
                (pL2CChannel->pClient->CurrProc.bOpCode != 0)
           )
        {
            GATT_TRACE_PRINTF_2(GATT_TRACE_MASK_ERROR,
                                "!!gattHandleGATT_EXECUTE_WRITE_REQ: pL2CChannel->L2CState = %d, pL2CChannel->pClient->CurrProc.bOpCode = 0x%x",
                                pL2CChannel->L2CState,
                                pL2CChannel->pClient->CurrProc.bOpCode
                               );

            wStatus = GATT_ERR_ILLEGAL_STATE;
        }
        else
        {
            if ((flags == 0x00) ||
                    (flags == 0x01))
            {
                wStatus = gattExecuteWrite( pL2CChannel, flags);
            }
            else
            {
                wStatus = GATT_ERR_ILLEGAL_PARAMETER;
            }

        }

    }
    else
    {
        wStatus = GATT_ERR_INTERNAL;
    }

    if ( wStatus != GATT_SUCCESS )
    {
        gattSendGATT_EXECUTE_WRITE_CONF(wStatus, cid);
    }

}

void gattHandleGATT_EXECUTE_WRITE_RESP(uint16_t cid, uint16_t cause, uint16_t handle)
{
    PGATTL2CChannel      pL2CChannel;

    pL2CChannel = gattL2CChannelFindHandle( FALSE, cid );
    if ( pL2CChannel == NULL )
    {
        GATT_TRACE_PRINTF_0(GATT_TRACE_MASK_ERROR,
                    "!!GATT: gattHandleGATT_EXECUTE_WRITE_RESP / pL2CChannel is NULL");
        return;
    }

    /* resume/complete write operation */
    gattExecuteWriteResumeAppl( pL2CChannel, cause, handle);
}

void gattHandleGATT_ATTRIB_INDICATION_RESP(uint16_t cid)
{
    PGATTL2CChannel            pL2CChannel;

	pL2CChannel = gattL2CChannelFindHandle( FALSE, cid);
	if ( pL2CChannel != NULL )
	{
		/* send acknowledgement to server */
		attSendATT_GENERIC_PDU( pL2CChannel, ATT_HANDLE_VALUE_CONFIRMATION,  offsetof(TATTPDUGeneric, Param),0,0,0,0,NULL );
	}
	else
	{
		assert(FALSE);
	}

	return;
}

void gattHandleGATT_LE_ADVERTISE_REQ(/*uint16_t applHandle,*/ uint16_t opCode, uint8_t length, uint8_t * pData)
{
	/* state machine busy */
	if (pGATT->LE_adv.opCode != 0)
	{
		gattSendGATT_LE_ADVERTISE_CONF(opCode, GATT_ERR_ILLEGAL_STATE);
		return;
	}

	pGATT->LE_adv.opCode        = opCode;
	pGATT->LE_adv.opCodeCurrent = opCode;
	pGATT->LE_adv.status        = HCI_SUCCESS;

	switch( opCode )
	{
	case GATT_LE_ADVERTISE_OPCODE_AD_PARAMETER:
		{
			PGATTLEAdvertiseParameter pAdvParam = (PGATTLEAdvertiseParameter)pData;

			/* ADV_DIRECT_IND not allowed */
			if (pAdvParam->advType == LE_ADVERTISING_TYPE_ADV_DIRECT_HIGH_DUTY_IND)
			{
                memcpy(&pGATT->LE_adv.directed.bd, pAdvParam->bd, BD_ADDR_SIZE);
				pGATT->LE_adv.directed.bdType = pAdvParam->bdType;
				pGATT->LE_adv.directed.local_bdType = pAdvParam->local_bdType;
                pGATT->LE_adv.opCode        = 0;
                pGATT->LE_adv.opCodeCurrent = 0;
                gattSendGATT_LE_ADVERTISE_CONF(GATT_LE_ADVERTISE_OPCODE_AD_PARAMETER, GATT_SUCCESS);
                
			}
			else
			{
				memcpy(&pGATT->LE_adv.param, pAdvParam, sizeof(TGATTLEAdvertiseParameter));

				if (pGATT->LE_adv.enabled)
				{
					/* disable advertising first */
					pGATT->LE_adv.opCodeCurrent = GATT_LE_ADVERTISE_OPCODE_AD_DISABLE;
					gattSendLE_SET_ADVERTISE_ENABLE(FALSE);
				}
				else
				{
					/* forward parameters to HCI */
					gattSendLE_SET_ADVERTISING_PARAMETERS(pAdvParam, NULL, 0);
				}
			}
		}
		break;

	case GATT_LE_ADVERTISE_OPCODE_AD_DATA:
		hciCommandLESetAdvertisingData(HCI_LE_SET_ADVERTISING_DATA,
							pData,
							length
							);
        osBufferRelease(pData);
		hciLaterEntry();		//randy
		break;

	case GATT_LE_ADVERTISE_OPCODE_SC_RSP:
		hciCommandLESetAdvertisingData(HCI_LE_SET_SCAN_RESPONSE_DATA,
								pData,
								length
								);
        osBufferRelease(pData);
		hciLaterEntry();		//randy
		break;

	case GATT_LE_ADVERTISE_OPCODE_AD_ENABLE:
	    pGATT->LE_adv.enabled = TRUE;
	    gattSendLE_SET_ADVERTISE_ENABLE(TRUE);
		break;

	case GATT_LE_ADVERTISE_OPCODE_AD_DISABLE:
	    pGATT->LE_adv.enabled = FALSE;
	    gattSendLE_SET_ADVERTISE_ENABLE(FALSE);
		break;

	case GATT_LE_ADVERTISE_OPCODE_AD_DIRECTED:
		{
			if (pGATT->LE_adv.enabled)
			{
				/* disable advertising first */
				pGATT->LE_adv.opCodeCurrent = GATT_LE_ADVERTISE_OPCODE_AD_DISABLE;
				gattSendLE_SET_ADVERTISE_ENABLE(FALSE);
			}
			else
			{
                TGATTLEAdvertiseParameter advParam;

				/* only advType is used */
				advParam.advType        = LE_ADVERTISING_TYPE_ADV_DIRECT_HIGH_DUTY_IND;
				advParam.advChannelMap  = LE_ADVERTISING_CHANNEL_ALL;
				advParam.filterPolicy   = 0x00;
				advParam.minAdvInterval = 0x20;
				advParam.maxAdvInterval = 0x20;
                advParam.local_bdType = pGATT->LE_adv.directed.local_bdType;

				/* merge with bd */
				gattSendLE_SET_ADVERTISING_PARAMETERS(&advParam,
				                                    (LPBdAddr)&pGATT->LE_adv.directed.bd,
				                                    pGATT->LE_adv.directed.bdType
				                                    );
			}
		}
		break;

	default:	  
		pGATT->LE_adv.opCode        = 0;
		pGATT->LE_adv.opCodeCurrent = 0;

		gattSendGATT_LE_ADVERTISE_CONF(opCode, GATT_ERR_ILLEGAL_PARAMETER);

		break;
	}

    return;      /* 0 => calling routine will NOT release Blueface */
}


void gattHandleGATT_LE_SCAN_REQ(PGATTLEScanReq pScanReq)
{
  PGATTLEScanReq pScanParam = &pGATT->LE_scan.param;

  memcpy(pScanParam, pScanReq, sizeof(TGATTLEScanReq));

  if (pGATT->LE_scan.enabled)
  {
    /* scan currently enabled: disable first */
    pGATT->LE_scan.enabled = FALSE;
    gattSendLE_SET_SCAN_ENABLE(FALSE, FALSE);
  }
  else
  {
    /* scan is disabled, set parameters */
    gattSendLE_SET_SCAN_PARAMETERS(pScanReq);
  }
}

void gattHandleGATT_LE_MODIFY_WHITELIST_REQ(uint16_t opCode, uint8_t * bd, uint8_t bdType)
{
	/* statemachine busy */
	if (pGATT->LE_whitelist.opCode != 0)
	{
		gattSendGATT_LE_MODIFY_WHITELIST_CONF(opCode, GATT_ERR_ILLEGAL_STATE);
		return;
	}

	pGATT->LE_whitelist.opCode	  = opCode;

	switch (opCode)
	{
	case GATT_LE_MODIFY_WHITELIST_OPCODE_ADD:
		gattSendLE_ADD_DEVICE_TO_WHITELIST((LPBdAddr)bd, bdType);
		break;

	case GATT_LE_MODIFY_WHITELIST_OPCODE_REMOVE:
		gattSendLE_REMOVE_DEVICE_FROM_WHITELIST((LPBdAddr)bd, bdType);
		break;

	case GATT_LE_MODIFY_WHITELIST_OPCODE_CLEAR:
		gattSendLE_CLEAR_WHITELIST();
		break;

	default:
		pGATT->LE_whitelist.opCode = 0;
		gattSendGATT_LE_MODIFY_WHITELIST_CONF(opCode, GATT_ERR_ILLEGAL_PARAMETER);
		break;
	}
	
}

void gattHandleGATT_LE_CONNECTION_UPDATE_REQ(uint16_t cid, PGATTLEConnectionUpdateParam pParam)
{
	PGATTL2CChannel            pL2CChannel;

	pL2CChannel = gattL2CChannelFindHandle(FALSE,cid);
	if (pL2CChannel != NULL)
	{
        uint16_t status;
		/* send to L2CAP/HCI */
        status = l2cLEHandleConnectionUpdateReq(pL2CChannel->wLEConnectionHandle, pParam);
        if(status)
        {
            gattHandleLEConnectionUpdateConf(pL2CChannel->wLEConnectionHandle, status);
        }
	}
	else
	{
		gattSendGATT_LE_CONNECTION_UPDATE_CONF(GATT_ERR_ILLEGAL_HANDLE, cid);
	}

	return;
}

void gattHandleGATT_LE_CONNECTION_UPDATE_RESP(uint16_t cid, uint16_t cause)
{
    PGATTL2CChannel 			pL2CChannel;
	uint8_t						status;

	pL2CChannel = gattL2CChannelFindHandle(FALSE, cid);
	if (pL2CChannel != NULL)
	{
		status = (cause == GATT_SUCCESS) ? LE_CONNECTION_UPDATE_ACCEPT
													: LE_CONNECTION_UPDATE_REJECT;
		/* was a slave request */
		if (pL2CChannel->slaveConnectionUpdate)
		{
			/* send to L2CAP/HCI */
            l2cLEHandleConnectionUpdateResp( pL2CChannel->wLEConnectionHandle, &pL2CChannel->conParam, status);
		}
	}
}

void gattHandleCONFIG_GAP_DEVICE_NAME(uint8_t * pName,uint16_t len)
{
    if ( len > GATT_DEVICE_NAME_LENGTH )
        len = GATT_DEVICE_NAME_LENGTH;

    memcpy( gGattDeviceName, pName, len );
    gGattDeviceName[len] = 0;
}

void gattHandleCONFIG_GAP_APPEARANCE(uint16_t appearnce)
{
	SHORT2CHAR(gGattAppearance, appearnce);
}

void gattHandleCONFIG_GAP_PER_PREF_CONN_PARAM(uint16_t connIntervalMin,
                                              uint16_t connIntervalMax,
                                              uint16_t slaveLatency,
                                              uint16_t supervisionTimeout)
{
    uint16_t pos = 0;
    SHORT2CHAR(gGattPerPrefConnParam + pos, connIntervalMin);
    pos += 2;
    SHORT2CHAR(gGattPerPrefConnParam + pos, connIntervalMax);
    pos += 2;
    SHORT2CHAR(gGattPerPrefConnParam + pos, slaveLatency);
    pos += 2;
    SHORT2CHAR(gGattPerPrefConnParam + pos, supervisionTimeout);
    pos += 2;
}

void gattHandleBtsmSecurityStatus(uint8_t indId, uint8_t * bd, uint8_t bdType, uint16_t cause, uint8_t keyType, uint8_t keySize)
{
	GATT_TRACE_PRINTF_5(GATT_TRACE_MASK_TRACE,
	"GATT: SEC_STAT: BD=%s code=0x%x keysize=%d keytype=0x%x cause=0x%x",
							  TRACE_BDADDR1(GATT_TRACE_MASK_TRACE, bd),
							  indId,
							  keySize,
							  keyType,
							  cause
							  );

	if ( cause == HCI_SUCCESS )
	{
		PGATTL2CChannel  pL2CChannel;

		if ( (pL2CChannel = gattL2CChannelFindBD( (LPBdAddr)bd )) != NULL )
		{
			PGATTRClient	pRClient = pL2CChannel->pRClient;

			if (pRClient != NULL)
			{
				switch( indId )
				{
				default:
					break;
				case CONF_SECURITY_STATUS_AUTH_STARTED:
					pRClient->bSecurityFlags = 0;
					pRClient->bEncyptKeysize = 0;
					break;
				case CONF_SECURITY_STATUS_ENCRYPTED:
					pRClient->bSecurityFlags |= GATT_RCLIENT_SEC_ENCRYPTED;
					pRClient->bEncyptKeysize	= keySize;
				  /* no break */
				case CONF_SECURITY_STATUS_AUTH_SUCCESS:
					if ( (keyType == BLUEFACE_KEYTYPE_COMBINATION) ||
					   (keyType == BLUEFACE_KEYTYPE_UNAUTHENTICATED)
					 )
					{
						pRClient->bSecurityFlags |= GATT_RCLIENT_SEC_AUTHEN;
					}
					else if ( keyType == BLUEFACE_KEYTYPE_AUTHENTICATED )
					{
						pRClient->bSecurityFlags |= GATT_RCLIENT_SEC_AUTHEN;
						pRClient->bSecurityFlags |= GATT_RCLIENT_SEC_AUTHEN_MITM;
					}
					break;
				case CONF_SECURITY_STATUS_NOT_ENCRYPTED:
					pRClient->bSecurityFlags &= ~GATT_RCLIENT_SEC_ENCRYPTED;
					pRClient->bEncyptKeysize	= 0;
					/* no break */
				case CONF_SECURITY_STATUS_AUTH_FAILED:
					pRClient->bSecurityFlags &=
					   ~(GATT_RCLIENT_SEC_AUTHEN | GATT_RCLIENT_SEC_AUTHEN_MITM);
					break;
				}
			}
		}
	}    
}

void gattHandleBtsmResolvedAddress(uint8_t * bd, uint8_t bdType, uint8_t * resolvedBd, uint8_t resolvedBdType)
{
    
	PGATTL2CChannel			  pL2CChannel;

	GATT_TRACE_PRINTF_2(GATT_TRACE_MASK_TRACE,
						 "GATT: bd resolved bd=%s bdType=0x%x",
						 TRACE_BDADDR1(GATT_TRACE_MASK_TRACE, resolvedBd),
						 resolvedBdType);

	/* update BD in connection */
	pL2CChannel = gattL2CChannelFindBD((LPBdAddr)bd);
	if (pL2CChannel != NULL)
	{
		memcpy(pL2CChannel->RemoteBd, resolvedBd, BD_ADDR_SIZE);
		pL2CChannel->bdType = resolvedBdType;
	}

	{
		uint16_t			  i;

		/* update BD in CCC bit table */
		for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.gattServerCCCbitsCount; i++)
		{
			PGATTCCCBitsTable pCCCBitsTable = &pGATT->pServer->pCCCBits[i];

			if ((pCCCBitsTable->bUsed) &&
			  (memcmp(pCCCBitsTable->Bd, bd, BD_ADDR_SIZE) == 0)
			 )
			{
				memcpy(pCCCBitsTable->Bd,resolvedBd, BD_ADDR_SIZE);
			}
		}
	}

}

