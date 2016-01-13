/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       gattproc.c
* @brief     GATT (Generic Attribute Protocol) procedures/features
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

#include <upper_stack_global.h>
 /* parameters for generic routine gattHandleReadAndFindRequest() */
typedef struct _GATTSearchParam
{
	uint16_t              wType;             /* type of search  */
	BOOL              Primary;
	int               iOffset;           /* offset req. in result buffer */
	uint16_t              wStartingHandle;
	uint16_t              wEndingHandle;
	int               iCmpValueLength;   /* for search with value comparison */
	uint8_t *            pCmpValue;
	TATTRespFunction  ATTRespFunction;
} TGATTSearchParam, * PGATTSearchParam;

/**
* @brief  gatt server configuration
*
* @param  pL2CChannel:
*
* @return  
*
*/
void gattServerConfiguration( PGATTL2CChannel pL2CChannel )
{
	if ( pL2CChannel->wMTUSizeMax > ATT_MTU_SIZE_LE_DEFAULT )
	{
		attSendATT_EXCHANGE_MTU_REQUEST( pL2CChannel );
	}
}

/**
* @brief		primary service discovery
*
* @param	pL2CChannel: 
* @param	pDiscoveryReq:
* @param	pParam
*
* @return
*
*/
uint16_t gattDiscoveryPrimaryService( PGATTL2CChannel pL2CChannel,
                                  PGATTDiscoveryReq   pDiscoveryReq,
                                  PGATTDiscoveryParam pParam )
{
	uint16_t           wType, wStartingHandle, wEndingHandle;
	uint8_t           bUUID[2];
	PGATTCurrProc  pCurrProc;
	uint16_t           wStatus = GATT_SUCCESS;

	if ( pDiscoveryReq != NULL )
	{
		/* initial call triggered by GATT_DISCOVERY_REQ */
		wType = pDiscoveryReq->type;
	}
	else if ( pParam != NULL )
	{
		/* continuation call triggered by GATT_DISCOVERY_RESP */
		wType = pParam->wType;
	}
	else
	{
		return( GATT_ERR_ILLEGAL_PARAMETER );
	}

	pCurrProc = &pL2CChannel->pClient->CurrProc;

	if ( wType == GATT_TYPE_DISCOVERY_PSRV_ALL )
	{
		/* "Read By Group Type Request" */
		if ( pDiscoveryReq != NULL )
		{
			wStartingHandle = pDiscoveryReq->p.psrvAll.startingHandle;
			wEndingHandle   = pDiscoveryReq->p.psrvAll.endingHandle;
		}
		else
		{
			wStartingHandle = pParam->wStartingHandle;
			wEndingHandle   = pParam->wEndingHandle;
		}

		LE_WORD2EXTRN( bUUID, GATT_UUID_PRIMARY_SERVICE );
		wStatus = attSendATT_READ_BY_GROUP_TYPE_REQUEST( pL2CChannel,
		                                  wStartingHandle, wEndingHandle,
		                                  bUUID, UUID_16BIT_SIZE );
	}
	else
	{
		/* "Find By Type Value Request" */
		uint8_t *     pUUID;
		int        iUUIDLength;

		if ( pDiscoveryReq != NULL )
		{
			/* initial call */
			TGATTDiscoveryReqPSrvUUID * pPSrvUUID;

			pPSrvUUID       = &pDiscoveryReq->p.psrvUUID;
			wStartingHandle = pPSrvUUID->startingHandle;
			wEndingHandle   = pPSrvUUID->endingHandle;

			if ( pPSrvUUID->uuidType == UUID_TYPE_16 )
			{
				/* convert UUID to network representation */
				LE_WORD2EXTRN( bUUID, pPSrvUUID->uuid.uuid16 );
				pUUID       = bUUID;
				iUUIDLength = UUID_16BIT_SIZE;
			}
			else
			{
				pUUID       = pPSrvUUID->uuid.pUuid128;
				iUUIDLength = UUID_128BIT_SIZE;
			}

			/* save UUID for continuation call */
			pCurrProc->p.discovery.wUUIDType = pPSrvUUID->uuidType;
			memcpy( &pCurrProc->p.discovery.UUID, pUUID, iUUIDLength );
		}
		else
		{
			/* continuation call */
			wStartingHandle = pParam->wStartingHandle;
			wEndingHandle   = pParam->wEndingHandle;
			pUUID           = pParam->pUUID;
			iUUIDLength     = (pParam->wUUIDType == UUID_TYPE_16) ?
			                                 UUID_16BIT_SIZE : UUID_128BIT_SIZE;
		}

		wStatus = attSendATT_FIND_BY_TYPE_VALUE_REQUEST( pL2CChannel,
		                         wStartingHandle, wEndingHandle,
		                         iUUIDLength,
		                         pUUID );
	}

	if ( wStatus == GATT_SUCCESS )
	{
		pCurrProc->p.discovery.wLastHandle = 0; /* last ending handle signaled to appl. */
		pCurrProc->wProcType               = wType;
	}

	return( wStatus );
}

/**
* @brief		Discover relationship, characteristics, characteristic descriptors .. 
*
* @param	pL2CChannel: 
* @param	pDiscoveryReq
* @param	pParam: 
*
* @return
*
*/
uint16_t gattDiscoveryOthers( PGATTL2CChannel pL2CChannel,
                                             PGATTDiscoveryReq   pDiscoveryReq,
                                             PGATTDiscoveryParam pParam )
{
	uint16_t   wStartingHandle, wEndingHandle, wType;
	uint8_t   bUUID[2];
	uint16_t   wCause = GATT_SUCCESS;

	if ( pDiscoveryReq != NULL )
	{
		/* initial call triggered by GATT_DISCOVERY_REQ */
		wStartingHandle = pDiscoveryReq->p.relation.startingHandle;
		wEndingHandle   = pDiscoveryReq->p.relation.endingHandle;
		wType           = pDiscoveryReq->type;
	}
	else
	if ( pParam != NULL )
	{
		/* continuation call triggered by GATT_DISCOVERY_RESP */
		wStartingHandle = pParam->wStartingHandle;
		wEndingHandle   = pParam->wEndingHandle;
		wType           = pParam->wType;
	}
	else
	{
		return( GATT_ERR_ILLEGAL_PARAMETER );
	}

	switch( wType )
	{
	default:   /* GATT_TYPE_DISCOVERY_CHAR_DESCR */
		break;
	case GATT_TYPE_DISCOVERY_RELATION:
		LE_WORD2EXTRN( bUUID, GATT_UUID_INCLUDE );
		break;
	case GATT_TYPE_DISCOVERY_CHAR_ALL:
		LE_WORD2EXTRN( bUUID, GATT_UUID_CHARACTERISTIC );
		break;
	}

	if ( wType == GATT_TYPE_DISCOVERY_CHAR_DESCR )
	{
		wCause = attSendATT_FIND_INFO_REQUEST( pL2CChannel,
		                                 wStartingHandle, wEndingHandle );
	}
	else
	{
		wCause = attSendATT_READ_BY_TYPE_REQUEST( pL2CChannel,
		                                 wStartingHandle, wEndingHandle,
		                                 sizeof(uint16_t), bUUID );
	}
	if ( wCause == GATT_SUCCESS )
	{
		pL2CChannel->pClient->CurrProc.wProcType = wType;
	}

	return( wCause );
}

/**
* @brief		read attribute
*
* @param	pL2CChannel
* @param	pAttribReadReq
*
* @return
*
*/
uint16_t gattAttribRead( PGATTL2CChannel pL2CChannel,
                                           PGATTAttribReadReq  pAttribReadReq )
{
	uint8_t           bUUID[2];
	uint8_t *         pUUID;
	int            iUUIDSize;
	PGATTCurrProc  pCurrProc;
	uint16_t           wCause = GATT_SUCCESS;

	pCurrProc = &pL2CChannel->pClient->CurrProc;

	switch( pAttribReadReq->type )
	{
	default:
		wCause = GATT_ERR_UNSUPPORTED;
		break;

	case GATT_READ_TYPE_BASIC:
		wCause = attSendATT_READ_REQUEST( pL2CChannel, pAttribReadReq->p.basic.handle );
		if ( wCause == GATT_SUCCESS )
		{
			pCurrProc->p.read.wHandle      = pAttribReadReq->p.basic.handle;
			pCurrProc->p.read.wValueOffset = 0;
			pCurrProc->wProcType           = GATT_TYPE_CURR_PROC_READ;
		}
		break;

	case GATT_READ_TYPE_BLOB:
		wCause = attSendATT_READ_BLOB_REQUEST( pL2CChannel,
		                                  pAttribReadReq->p.blob.handle,
		                                  pAttribReadReq->p.blob.valueOffset );
		if ( wCause == GATT_SUCCESS )
		{
			pCurrProc->p.read.wHandle      = pAttribReadReq->p.blob.handle;
			pCurrProc->p.read.wValueOffset = pAttribReadReq->p.blob.valueOffset;
			pCurrProc->wProcType           = GATT_TYPE_CURR_PROC_READ;
		}
		break;

	case GATT_READ_TYPE_TYPE:
		if ( pAttribReadReq->p.type.uuidType == UUID_TYPE_16 )
		{
			LE_WORD2EXTRN( bUUID, pAttribReadReq->p.type.uuid.uuid16 );
			pUUID     = bUUID;
			iUUIDSize = UUID_16BIT_SIZE;
		}
		else
		{
			pUUID     = pAttribReadReq->p.type.uuid.pUuid128;
			iUUIDSize = UUID_128BIT_SIZE;
		}
		wCause = attSendATT_READ_BY_TYPE_REQUEST( pL2CChannel,
		                       pAttribReadReq->p.type.startingHandle,
		                       pAttribReadReq->p.type.endingHandle,
		                       iUUIDSize, pUUID );
		if ( wCause == GATT_SUCCESS )
		{
			pCurrProc->wProcType = GATT_TYPE_DISCOVERY_CMP_UUID;
		}
		break;
	}

	if ( wCause == GATT_SUCCESS )
	{
		pCurrProc->p.read.wReadType = pAttribReadReq->type;
	}

	return( wCause );
}

/**
* @brief		write attribute(s).
*
* @param	pL2CChannel: 
* @param	type
* @param	handle
* @param	pAttribWriteData
*
* @return
*
*/
uint16_t gattAttribWrite( PGATTL2CChannel      pL2CChannel,
                                   uint16_t type, uint16_t handle, uint16_t writeOffset,
                                   PGATTAttribWriteData pAttribWriteData )
{
	BOOL    Cmd    = FALSE;
	uint16_t    wCause = GATT_SUCCESS;

	switch( type )
	{
	default:
		wCause = GATT_ERR_UNSUPPORTED;
		break;

	case GATT_WRITE_TYPE_CMD:
		Cmd = TRUE;
	case GATT_WRITE_TYPE_REQ:
		wCause = attSendATT_WRITE_REQ_CMD( pL2CChannel, Cmd,
		                                 handle,
		                                 pAttribWriteData->pBuffer,
		                                 &pAttribWriteData->offset,
		                                 &pAttribWriteData->length
		                               );
		if ( !Cmd && (wCause == GATT_SUCCESS) )
		{
			pL2CChannel->pClient->CurrProc.wProcType = GATT_TYPE_CURR_PROC_WRITE;
		}
		break;
    case GATT_WRITE_TYPE_PREP:
        wCause = attSendATT_PREPARE_WRITE_REQ( pL2CChannel,
                                               writeOffset,
                                               handle,
                                               pAttribWriteData->pBuffer,
                                               &pAttribWriteData->offset,
                                               &pAttribWriteData->length
                                             );
        if ( wCause == GATT_SUCCESS )
            pL2CChannel->pClient->CurrProc.wProcType = GATT_TYPE_CURR_PROC_PREPARE_WRITE;
        break;

	}

	return( wCause );
}

uint16_t gattExecuteWrite( PGATTL2CChannel pL2CChannel, uint8_t flags)
{
    uint16_t    wCause = GATT_SUCCESS;
    wCause = attSendATT_EXECUTE_WRITE_REQ(pL2CChannel, flags);
    if ( wCause == GATT_SUCCESS )
        pL2CChannel->pClient->CurrProc.wProcType = GATT_TYPE_CURR_PROC_EXECUTE_WRITE;

    return ( wCause );
}

	
/**
* @brief		handle "Error Response" received from server.
* 			this routine is used for transaction timeout handling too !!!
*
* @param	pL2CChannel
* @param	Timeout
* @param	bReqOpcode
* @param	bErrorCode
* @param	wHandle
*
* @return
*
*/
void gattHandleATTErrorResponseOrTimeout( PGATTL2CChannel pL2CChannel,
                                 BOOL            Timeout,
                                 uint8_t            bReqOpcode,
                                 uint8_t            bErrorCode,
                                 uint16_t            wHandle )
{
	uint16_t  wStatus;
	uint16_t  wType = GATT_TYPE_UNDEFINED;

	if ( Timeout )
	{
		wStatus = GATT_ERR_TIMEOUT;
	}
	else
	{
		wStatus = ATT_ERR | bErrorCode;
	}

	switch( bReqOpcode )
	{
	default:
		break;

	case ATT_READ_BY_GROUP_TYPE_REQUEST:
	case ATT_FIND_BY_TYPE_VALUE_REQUEST:
	case ATT_READ_BY_TYPE_REQUEST:
	case ATT_FIND_INFO_REQUEST:
	case ATT_READ_REQUEST:
	case ATT_READ_BLOB_REQUEST:
	case ATT_WRITE_REQUEST:
    case ATT_PREPARE_WRITE_REQUEST:
    case ATT_EXECUTE_WRITE_REQUEST:

		wType = pL2CChannel->pClient->CurrProc.wProcType;
		break;
	}

	if ( wType != GATT_TYPE_UNDEFINED )
	{
		switch( wType )
		{
		default:
			/* e.g. no more attributes found in various discovery procedures */
			gattSendGATT_DISCOVERY_IND( wStatus, pL2CChannel->cid, wType,
			                                    0,  /* nbr. of list elements  */
			                                    0, /* length of list element */
			                                    NULL, NULL );
			break;

		case GATT_TYPE_DISCOVERY_CMP_UUID:
		case GATT_TYPE_CURR_PROC_READ:
			gattSendGATT_ATTRIB_READ_CONF( wStatus, pL2CChannel->cid,
			                        pL2CChannel->pClient->CurrProc.p.read.wReadType,
			                        0, 0, NULL, 0, NULL, 0 );
			break;

		case GATT_TYPE_CURR_PROC_WRITE:
			gattSendGATT_ATTRIB_WRITE_CONF( wStatus, pL2CChannel->cid,
			                                GATT_WRITE_TYPE_REQ, 0, NULL );
			break;
        case GATT_TYPE_CURR_PROC_PREPARE_WRITE:
            gattSendGATT_ATTRIB_PREPARE_WRITE_CONF( wStatus, pL2CChannel->cid,
                                                    0, 0, NULL );
            break;

        case GATT_TYPE_CURR_PROC_EXECUTE_WRITE:
            gattSendGATT_EXECUTE_WRITE_CONF( wStatus, pL2CChannel->cid);
            break;

		}
	}
	else
	{
		assert(FALSE);
	}

	pL2CChannel->pClient->CurrProc.wProcType = GATT_TYPE_UNDEFINED;
}

/**
* @brief  handle (some) ATT responses received from server
*
* @param  pL2CChannel:
* @param  iCount
* @param  iLength
* @param  pList
*
* @return  
*
*/
void gattHandleGenericResponse( PGATTL2CChannel  pL2CChannel,
                                int              iCount,
                                int              iLength,
                                uint8_t *           pList )
{
	PGATTCurrProc  pCurrProc = &pL2CChannel->pClient->CurrProc;
	uint16_t wProcType =  pCurrProc->wProcType;
	pCurrProc->wProcType = GATT_TYPE_UNDEFINED;

	gattSendGATT_DISCOVERY_IND( GATT_SUCCESS, pL2CChannel->cid,
	                         wProcType,
	                         iCount,  /* nbr. of list elements  */
	                         iLength, /* length of list element */
	                         pList, pCurrProc);
}

/**
* @brief  handle "Read By Type Response" received from server
*
* @param  pL2CChannel:
* @param  iCount
* @param  iLength
* @param  iTruncated
* @param  pAttDataList
*
* @return  
*
*/
void gattHandleReadByTypeResponse( PGATTL2CChannel  pL2CChannel,
                                   int              iCount,
                                   int              iLength,
                                   int              iTruncated,
                                   uint8_t *           pAttDataList )
{
	uint16_t           wStatus = GATT_SUCCESS;
	PGATTCurrProc  pCurrProc = &pL2CChannel->pClient->CurrProc;

	switch( pCurrProc->wProcType )
	{
	case GATT_TYPE_DISCOVERY_CMP_UUID:
		/* Characteristic Value Read Using UUID */
		gattSendGATT_ATTRIB_READ_CONF( GATT_SUCCESS, pL2CChannel->cid,
		                                 pCurrProc->p.read.wReadType,
		                                 iCount*iLength - iTruncated,
		                                 iTruncated, pAttDataList,
		                                 iCount,
		                                 (LPWORD)pAttDataList, /* handles in pAttDataList */
		                                 0 );
		break;

	case GATT_TYPE_DISCOVERY_RELATION:
		if ( iLength == 3*sizeof(uint16_t) )
		{
			/* 3 handles in list elements, 128 bit UUID has to be retrieved  */
			/* manually thru "Read Request" for each included service handle */
			/* in the result list :-( !!!        ..                          */

			/* XXXXMJMJ: not yet fully implemented, applic. can retrieve
			 * 128 bit UUID thru GATT_ATTRIBUTE_READ_REQ with the appropriate
			 * attribute handle (just as on ATT layer ...) and an updated
			 * (search start handle increased by 1 ..) GATT_DISCOVERY_REQ
			 */
			iCount  = 1;  /* signal only 1. included service, see notes above .. */
		}
		else
		{
			/* 3 handles + 16 bit UUID in list elements */
		}
	  /* fall thru !!!!!!!! */
	default:
		{
			uint16_t wProcType =	pCurrProc->wProcType;
			pCurrProc->wProcType = GATT_TYPE_UNDEFINED;
			gattSendGATT_DISCOVERY_IND( wStatus, pL2CChannel->cid,
			                           wProcType,
			                           iCount,  /* nbr. of list elements  */
			                           iLength, /* length of list element */
			                           pAttDataList, pCurrProc );
		} 
		break;
	}

	pCurrProc->wProcType = GATT_TYPE_UNDEFINED;
}

/**
* @brief  handle "Read Response", "Read Blob Response" .. received from server
*
* @param  pL2CChannel:
* @param  wType
* @param  iLength
* @param  pValue
*
* @return  
*
*/
void gattHandleReadResponse( PGATTL2CChannel pL2CChannel,
                             uint16_t            wType,
                             int             iLength,
                             uint8_t *          pValue )
{
	int            iValueOffset  = 0;
	int            iNbrOfHandles = 0;
	LPWORD         pHandles      = NULL;
	PGATTCurrProc  pCurrProc = &pL2CChannel->pClient->CurrProc;

	switch( wType )
	{
	default:
		iLength = 0;
		break;
	case GATT_READ_TYPE_BLOB:
		iValueOffset  = pCurrProc->p.read.wValueOffset;
	case GATT_READ_TYPE_BASIC:
		iNbrOfHandles = 1;
		pHandles      = &pCurrProc->p.read.wHandle;
		break;
	}

	gattSendGATT_ATTRIB_READ_CONF( GATT_SUCCESS, pL2CChannel->cid,
	                                    pCurrProc->p.read.wReadType,
	                                    iLength, 0, pValue,
	                                    iNbrOfHandles, pHandles, iValueOffset );

	pCurrProc->wProcType = GATT_TYPE_UNDEFINED;
}

/**
* @brief  handle "Write Response" and "Prepare Write Response" received from server
*
* @param  pL2CChannel:
* @param  wType
* @param  iLength
* @param  pValue
*
* @return  
*
*/
BOOL gattHandleWriteResponse( PGATTL2CChannel pL2CChannel,
                              uint16_t            wType,
                              int             iLength,
                              uint8_t *          pValue )
{
	BOOL ReleaseBuffer;

	ReleaseBuffer = gattSendGATT_ATTRIB_WRITE_CONF( GATT_SUCCESS,
	                                              pL2CChannel->cid, wType,
	                                              iLength, pValue );
	pL2CChannel->pClient->CurrProc.wProcType = GATT_TYPE_UNDEFINED;

	return( ReleaseBuffer );
}

/**
* @brief  handle "Handle Value Notification/Indication" received from server
*
* @param  pL2CChannel:
* @param  Notify
* @param  wHandle
* @param  iLength
* @param  pValue
*
* @return  
*
*/
void gattHandleNotifInd( PGATTL2CChannel pL2CChannel, BOOL Notify,
                                   uint16_t wHandle, int iLength, uint8_t * pValue )
{
	gattSendGATT_ATTRIB_NOTIF_IND( pL2CChannel->cid, Notify,
	                                               wHandle, iLength, pValue );
}

/**
* @brief		send handle/value indication/notification
*
* @param	pL2CChannel: 
* @param	wCCCBits
* @param	wHandle
* @param	pAttrib:
* @param	pAttribUpdData
*
* @return
*
*/
int  gattAttribNotifInd( PGATTL2CChannel     pL2CChannel,
                         uint16_t                wCCCBits,
                         uint16_t                wHandle,
                         PAttrib             pAttrib,
                         PGATTAttribUpdData  pAttribUpdData,
                         BOOL          * pReleaseBuffer
                       )
{
	int      iLength, iReturn;
	LPWORD   pwOffsetPayload;
	uint8_t *   pValue, *pBuffer;
    *pReleaseBuffer = TRUE;

	if ( pAttrib->wFlags & ATTRIB_FLAG_VALUE_APPL )
	{
		/* value was supplied by application */
		iLength         = pAttribUpdData->length;
		pwOffsetPayload = &pAttribUpdData->offset;
		pBuffer         = pAttribUpdData->pBuffer;
		pValue          = pBuffer + pAttribUpdData->offset;
        *pReleaseBuffer = FALSE;
	}
	else
	{
		pBuffer         = NULL;
		pwOffsetPayload = NULL;
		if ( pAttrib->wFlags & ATTRIB_FLAG_VALUE_INCL )
		{
	  		pValue = pAttrib->TypeValue.s.bValueIncl;
		}
		else
		{
	  		pValue = (uint8_t *)pAttrib->pValueContext;
		}
		iLength = pAttrib->wValueLen;
	}

	iReturn = attSendATT_HANDLE_VALUE_NOTIF_IND( pL2CChannel,
	                             (wCCCBits == GATT_CLIENT_CHAR_CONFIG_NOTIFY),
	                             pBuffer, pwOffsetPayload,
	                             wHandle, &iLength, pValue );

	if ( pBuffer != NULL )   /* ATT PDU header has been added .. */
	{
		pAttribUpdData->length = iLength;
	}

	return( iReturn );
}

/**
* @brief  handle handle/value confirmation
*
* @param  pL2CChannel:
* @param  wStatus
*
* @return  
*
*/
void gattHandleAttribConf( PGATTL2CChannel pL2CChannel, uint16_t wStatus )
{
	pL2CChannel->pRClient->CurrInd.ConfPending = FALSE;

	if ( pL2CChannel->pRClient->CurrInd.ServiceChanged )
	{
		/* triggered by ServiceChanged indication */
		pL2CChannel->pRClient->CurrInd.ServiceChanged = FALSE;
		if ( wStatus == GATT_SUCCESS )
		{
			/* instruct BTSEC to remove ServiceChanged characteristic */
			/* handle if bond was created/exists                      */
			gattSendDEVICE_DATA_IND( DEVICE_DATA_SET_IND,
			                           DEVICE_DATA_TYPE_GATT_SRV_CHG_HANDLE, TRUE,
			                           pL2CChannel, 0, NULL );
		}
	}
	else
	{
		gattSendGATT_ATTRIB_UPDATE_STATUS_IND( pL2CChannel->pRClient, wStatus );
	}
}

/**
* @brief		handle generic Read/Find request handler
*
* @param	pL2CChannel: 
* @param	pParam
*
* @return
*
*/
uint32_t gattHandleReadAndFindRequest(
                      PGATTL2CChannel   pL2CChannel,
                      PGATTSearchParam  pParam )
{
	int             iCnt, iSize;
	uint32_t           dwATTError  = ATT_OK;
	BOOL            DeferRsp    = FALSE;
	BOOL            ReadCCCBits = FALSE;
	PGATTCurrTrans  pCurrTrans  = &pL2CChannel->pRClient->CurrTrans;

	if ( (pParam->wType == GATT_TYPE_DISCOVERY_CMP_UUID) &&
	   (pParam->iCmpValueLength == UUID_16BIT_SIZE)    &&
	   (LE_EXTRN2WORD(pParam->pCmpValue) == GATT_UUID_CHAR_CLIENT_CONFIG)
	 )
	{
		ReadCCCBits = TRUE;
	}

	/* alloc. temp. work buffer to store search result */
	iSize = pL2CChannel->wMTUSize - pParam->iOffset;
	if ( (iSize = gattWorkBufferAlloc( iSize)) > 0 )
	{
		pCurrTrans->p.read.wRemainingSize = iSize;
		dwATTError = gattAttributesSearch( FALSE, pL2CChannel->pRClient,

		                      (pL2CChannel->bdType & BLUEFACE_BDTYPE_LE_MASK),

		                      pParam->wType, pParam->Primary,
		                      &pParam->wStartingHandle, pParam->wEndingHandle,
		                      pParam->iCmpValueLength, pParam->pCmpValue,
		                      &iCnt,
		                      pGATT->pWorkBuffer,
		                      &iSize /* size of list element on return */,
		                      -1 /* forced list element size, -1: not yet defined */);
		if ( iSize < 0 )
		{
			/* applic. has to supply value, save relevant transaction data. */
			/* => pParam->wType = GATT_TYPE_DISCOVERY_CMP_UUID / "Read      */
			/*    Characteristic Value Using UUID" and iCnt>=1 !!           */

			iSize = -iSize;
			pCurrTrans->wTransType = GATT_TYPE_DISCOVERY_CMP_UUID;
			pCurrTrans->p.read.wSize         = iSize;    /* size of list element */
			pCurrTrans->p.read.wLastValueLength = 0;     /* length of last attribute value */
			/* handle of current list element is already stored: */
			pCurrTrans->p.read.wTxOffset     = (iCnt-1)*iSize + sizeof(uint16_t);
			pCurrTrans->p.read.wRemainingSize -= pCurrTrans->p.read.wTxOffset;
			pCurrTrans->p.read.wReadOffset   = 0;
			pCurrTrans->p.read.pGATTService  =
			  &pGATT->pServer->pService[GATT_ATTR_HANDLE_NBR_GET(pParam->wStartingHandle)];
			pCurrTrans->p.read.wHandle       = pParam->wStartingHandle;
			pCurrTrans->p.read.wEndingHandle = pParam->wEndingHandle;
			pCurrTrans->p.read.wUUIDLength   = pParam->iCmpValueLength;
			memcpy( pCurrTrans->p.read.bUUID, pParam->pCmpValue,
			                                             pParam->iCmpValueLength );

			pCurrTrans->p.read.pWorkBuffer   = pGATT->pWorkBuffer;
			pGATT->pWorkBuffer = NULL;

			DeferRsp = TRUE;            /* defer response */

			gattSendGATT_ATTRIB_READ_IND( pL2CChannel );
		}
		else if ( iCnt > 0 )
		{
			if ( ReadCCCBits )
			{
				int    i;
				uint16_t   wHandle, wCCCBits;
				uint8_t * pHandle, *pValue;

				pHandle = pGATT->pWorkBuffer;
				for ( i=0; i<iCnt; i++ )
				{
					pValue  = pHandle + sizeof(uint16_t);
					wHandle = LE_EXTRN2WORD(pHandle);

					/* get instance value from BTSEC supplied table */
					wCCCBits = gattCCCBitsGet( TRUE, wHandle,
					                           (LPBdAddr)pL2CChannel->RemoteBd );

					/* one handle,value(=wCCCBits) pair: */
					LE_WORD2EXTRN( pValue, wCCCBits );

					pHandle += 2*sizeof(uint16_t);
				}
			}

			if ( !DeferRsp && (dwATTError == ATT_OK) )
			{
				/* send rsp */
				(*pParam->ATTRespFunction)(pL2CChannel, iCnt, iSize, pGATT->pWorkBuffer );
			}
		}
		if ( !DeferRsp )
		{
			gattWorkBufferFree( );
		}
	}
	else
	{
		dwATTError = ATT_ERR_INSUFFICIENT_RESOURCES;
	}

	if ( dwATTError != ATT_OK )
	{
		/* calling routine(s) will generate ATT Error Response */
		ATT_HANDLE2DWORD(dwATTError, pParam->wStartingHandle);
	}

	return( dwATTError );
}

/**
* @brief		handle "Read By Group Type Request" received from client
*
* @param	pL2CChannel: 
* @param	wStartingHandle
* @param	wEndingHandle
* @param	iAttGroupTypeLength
* @param	pAttGroupType
*
* @return
*
*/
uint32_t gattHandleReadByGroupTypeRequest( PGATTL2CChannel pL2CChannel,
                                        uint16_t   wStartingHandle,
                                        uint16_t   wEndingHandle,
                                        int    iAttGroupTypeLength,
                                        uint8_t * pAttGroupType )
{
	uint16_t    wUUID;
	uint32_t   dwATTError = ATT_ERR_INVALID_PDU;

	/* for GATT "Read By Group Type Request" is used for */
	/* service discovery only                            */
	if ( iAttGroupTypeLength == UUID_16BIT_SIZE )
	{
		wUUID = LE_EXTRN2WORD( pAttGroupType );
		if ( (wUUID == GATT_UUID_PRIMARY_SERVICE) ||
		     (wUUID == GATT_UUID_SECONDARY_SERVICE)
		   )
		{
			TGATTSearchParam  Param;

			Param.wType           = GATT_TYPE_DISCOVERY_PSRV_ALL;
			Param.Primary         = (wUUID == GATT_UUID_PRIMARY_SERVICE) ? TRUE : FALSE;
			Param.iOffset         = offsetof(TATTReadGroupTypeResp, AttDataList);
			Param.wStartingHandle = wStartingHandle;
			Param.wEndingHandle   = wEndingHandle;
			Param.iCmpValueLength = 0;
			Param.pCmpValue       = NULL;
			Param.ATTRespFunction = attSendATT_READ_BY_GROUP_TYPE_RESPONSE;

			dwATTError = gattHandleReadAndFindRequest( pL2CChannel, &Param );
		}
		else
		{
			dwATTError = ATT_ERR_UNSUPPORTED_GROUP_TYPE;
		}
	}

	if ( dwATTError != ATT_OK )
	{
		ATT_HANDLE2DWORD(dwATTError, wStartingHandle);
	}

	return( dwATTError );
}

/**
* @brief		handle "Find By Type Value Request" received from client
*
* @param	pL2CChannel: 
* @param	wStartingHandle
* @param	wEndingHandle
* @param	wUUID
* @param	iAttValueLength
* @param	pAttValue
*
* @return
*
*/
uint32_t gattHandleFindByTypeValueRequest( PGATTL2CChannel pL2CChannel,
                                        uint16_t   wStartingHandle,
                                        uint16_t   wEndingHandle,
                                        uint16_t   wUUID,      /* AttType */
                                        int    iAttValueLength,
                                        uint8_t * pAttValue )
{
	uint32_t   dwATTError = ATT_ERR_UNSUPPORTED_GROUP_TYPE;

	/* for GATT "Find By Type Value Request" is used for */
	/* service discovery only                            */
	if ( (wUUID == GATT_UUID_PRIMARY_SERVICE) ||
	   (wUUID == GATT_UUID_SECONDARY_SERVICE)
	 )
	{
		TGATTSearchParam  Param;

		Param.wType           = GATT_TYPE_DISCOVERY_PSRV_UUID;
		Param.Primary         = (wUUID == GATT_UUID_PRIMARY_SERVICE) ? TRUE : FALSE;
		Param.iOffset         = offsetof(TATTFindTypeValueResp, HandlesInfoList);
		Param.wStartingHandle = wStartingHandle;
		Param.wEndingHandle   = wEndingHandle;
		Param.iCmpValueLength = iAttValueLength;
		Param.pCmpValue       = pAttValue;
		Param.ATTRespFunction = attSendATT_FIND_BY_TYPE_VALUE_RESPONSE;

		dwATTError = gattHandleReadAndFindRequest( pL2CChannel, &Param );
	}
	else
	{
		dwATTError = ATT_ERR_UNSUPPORTED_GROUP_TYPE;
	}

	if ( dwATTError != ATT_OK )
	{
		ATT_HANDLE2DWORD(dwATTError, wStartingHandle);
	}

	return( dwATTError );
}

/**
* @brief		handle "Read By Type Request" received from client
*
* @param	pL2CChannel: 
* @param	wStartingHandle
* @param	wEndingHandle
* @param	iAttTypeLength
* @param	pAttType
*
* @return
*
*/
uint32_t gattHandleReadByTypeRequest( PGATTL2CChannel pL2CChannel,
                                        uint16_t            wStartingHandle,
                                        uint16_t            wEndingHandle,
                                        int             iAttTypeLength,
                                        uint8_t *          pAttType )
{
	uint16_t    wUUID;
	TGATTSearchParam  Param;
	uint32_t   dwATTError = ATT_ERR_INVALID_PDU;
	uint16_t    wType      = GATT_TYPE_DISCOVERY_CMP_UUID;

	/* attribute type is 16 or 128 bit UUID */
	if ( iAttTypeLength == UUID_16BIT_SIZE )
	{
		wUUID = LE_EXTRN2WORD( pAttType );
		switch( wUUID )
		{
		default:
			/* e.g. "Characteristic Value Read / Using UUID" */
			break;

		/* the following require special treatment since the relevant data */
		/* is only partially stored in the attribute value (due to server  */
		/* database layout, most stuff may be in ROM ...):                 */
		case GATT_UUID_INCLUDE:
			wType = GATT_TYPE_DISCOVERY_RELATION;
			break;
		case GATT_UUID_CHARACTERISTIC:
			wType = GATT_TYPE_DISCOVERY_CHAR_ALL;
			break;
		}
	}
	else
	{
		/* e.g. "Characteristic Value Read / Using UUID" */
	}

	if ( wType == GATT_TYPE_DISCOVERY_CMP_UUID )
	{
		/* type/UUID received from client is search criterion */
	}
	else
	{
		/* search criterion is implicitly defined by wType */
		iAttTypeLength = 0;
		pAttType       = NULL;
	}

	Param.wType           = wType;
	Param.Primary         = FALSE;
	Param.iOffset         = offsetof(TATTReadTypeResp, AttDataList);
	Param.wStartingHandle = wStartingHandle;
	Param.wEndingHandle   = wEndingHandle;
	Param.iCmpValueLength = iAttTypeLength;
	Param.pCmpValue       = pAttType;
	Param.ATTRespFunction = attSendATT_READ_BY_TYPE_RESPONSE;

	dwATTError = gattHandleReadAndFindRequest( pL2CChannel, &Param );

	if ( dwATTError != ATT_OK )
	{
		ATT_HANDLE2DWORD(dwATTError, wStartingHandle);
	}

	return( dwATTError );
}

/**
* @brief		handle "Read FindInformation Request" received from client
*
* @param	pL2CChannel
* @param	wStartingHandle
* @param	wEndingHandle
*
* @return
*
*/
uint32_t gattHandleFindInfoRequest( PGATTL2CChannel pL2CChannel,
                                 uint16_t            wStartingHandle,
                                 uint16_t            wEndingHandle )
{
  TGATTSearchParam  Param;
  uint32_t             dwATTError;

  Param.wType           = GATT_TYPE_DISCOVERY_GET_UUID;
  Param.Primary         = FALSE;
  Param.iOffset         = offsetof(TATTFindInfoResp, InfoData);
  Param.wStartingHandle = wStartingHandle;
  Param.wEndingHandle   = wEndingHandle;
  Param.iCmpValueLength = 0;
  Param.pCmpValue       = NULL;
  Param.ATTRespFunction = attSendATT_FIND_INFO_RESPONSE;

  dwATTError = gattHandleReadAndFindRequest( pL2CChannel, &Param );

  return( dwATTError );
}

/**
* @brief		handle "Read Request", "Read Blob Response" .. received from client.
*
* @param	pL2CChannel
* @param	wType
* @param	pPDU
*
* @return
*
*/
uint32_t gattHandleReadRequest( PGATTL2CChannel pL2CChannel,
                             uint16_t            wType,
                             uint8_t *          pPDU )
{
	int     i, iCnt, iSize;
	int     iOffset;            /* offset of payload in ATT response PDU */
	int     iAttOffset = 0;     /* read offset in attribute */
	uint16_t    wHandle    = 0;
	BOOL    DeferRsp   = FALSE;
	uint32_t   dwATTError = ATT_OK;
	PGATTCurrTrans  pCurrTrans = &pL2CChannel->pRClient->CurrTrans;

	switch( wType )
	{
	default:
	case GATT_READ_TYPE_BASIC:
		wHandle = LE_EXTRN2WORD( ((LPATTReadReq)pPDU)->AttHandle );
		iOffset = offsetof(TATTReadResp, AttValue);
		break;
	case GATT_READ_TYPE_BLOB:
		wHandle    = LE_EXTRN2WORD( ((LPATTReadBlobReq)pPDU)->AttHandle );
		iAttOffset = LE_EXTRN2WORD( ((LPATTReadBlobReq)pPDU)->ValueOffset );
		iOffset    = offsetof(TATTReadBlobResp, PartAttValue);
		break;
	case GATT_READ_TYPE_MULTIPLE:
		iOffset = offsetof(TATTReadMultiResp, SetOfValues);
		break;
	case GATT_READ_TYPE_TYPE:
		/* handled in gattHandleReadByTypeRequest() */
		return( dwATTError );
	}

	if ( pCurrTrans->wTransType != GATT_TYPE_UNDEFINED )
	{
		/* previous transaction not yet complete */
		dwATTError = ATT_ERR_UNLIKELY;
	}
	else
	{
		iCnt       = 1;

		/* alloc. temp. work buffer to store read result */
		iSize = pL2CChannel->wMTUSize - iOffset;
		if ( (iSize = gattWorkBufferAlloc( iSize)) > 0 )
		{
			pCurrTrans->p.read.wRemainingSize = iSize;

			//XXXXMJMJ prepared but not yet OK for GATT_READ_TYPE_MULTIPLE / iCnt>1 !!!!
			for ( i = 0; i < iCnt; i++ )
			{
				dwATTError = gattAttributeRead( pL2CChannel->pRClient, wHandle,
				                                iAttOffset, pGATT->pWorkBuffer,
				                    &iSize /* size of attribute value on return */ );
				if ( dwATTError == ATT_OK )
				{
					if ( iSize < 0 )
					{
						/* applic. or BTSEC has to supply value, save relevant transaction data */
						if ( iSize == GATT_RCODE_APPLIC_VALUE )
						{
							/* applic. has to supply value */
							DeferRsp = TRUE;            /* defer response */

							pCurrTrans->wTransType = GATT_TYPE_CURR_TRANS_READ;
							pCurrTrans->p.read.wSize        = 0;
							pCurrTrans->p.read.wReadOffset  = iAttOffset;
							pCurrTrans->p.read.wHandle      = wHandle;
							pCurrTrans->p.read.pGATTService =
							      &pGATT->pServer->pService[GATT_ATTR_HANDLE_NBR_GET(wHandle)];
							pCurrTrans->p.read.pWorkBuffer  = pGATT->pWorkBuffer;
							pGATT->pWorkBuffer              = NULL;

							gattSendGATT_ATTRIB_READ_IND( pL2CChannel );
						}
						else if ( iSize == GATT_RCODE_CLIENT_CHAR_CFG )
						{
							/* get instance value from BTSEC supplied local table */
							uint16_t  wCCCBits = gattCCCBitsGet( TRUE, wHandle,
							                           (LPBdAddr)pL2CChannel->RemoteBd );

							LE_WORD2EXTRN( pGATT->pWorkBuffer, wCCCBits );
							iSize = sizeof(uint16_t);
						}
					}
				}
				else
				{
					break;
				}
			}

			if ( !DeferRsp && (dwATTError == ATT_OK) )
			{
				switch( wType )
				{
				default:
				case GATT_READ_TYPE_BASIC:
					attSendATT_READx_RESPONSE( pL2CChannel, ATT_READ_RESPONSE,
					                                      iSize, pGATT->pWorkBuffer );
					break;
				case GATT_READ_TYPE_BLOB:
					attSendATT_READx_RESPONSE( pL2CChannel, ATT_READ_BLOB_RESPONSE,
					                                      iSize, pGATT->pWorkBuffer );
					break;
				case GATT_READ_TYPE_MULTIPLE:
					break;
				}
			}
			if ( pGATT->pWorkBuffer != NULL )
			{
				gattWorkBufferFree( );
			}
		}
	}

	if ( dwATTError != ATT_OK )
	{
		ATT_HANDLE2DWORD(dwATTError, wHandle);
	}

	return( dwATTError );
}

/**
* @brief		resume read operation after reception of server application supplied
*			attribute value
*
* @param	appHandle:
* @param	bLinkHandle: 
* @param	serviceHandle
* @param	cause
* @param	length
* @param	offset
* @param	pBuffer
*
* @return
*
*/
void gattReadResumeAppl( PGATTL2CChannel pL2CChannel,
									uint16_t              cause,             /* error code */
									uint16_t              length,            /* attribute value/data length    */
									PGATTAttribReadRespParam  pParam    /* additional parameters + data in buffer */
									/* due to sizeof(MESSAGET_T) limitations  */ )
{
	PGATTRClient        pRClient;
	PGATTTransRead      pTransRead;
	int                 iSize, iCnt;
	uint8_t                bReqOpcode;
	uint8_t                bATTError = ATT_OK;
	BOOL                DeferRsp  = FALSE;
	BOOL                Terminate = FALSE;

	pRClient   = pL2CChannel->pRClient;
	pTransRead = &pRClient->CurrTrans.p.read;

	if ( cause == GATT_SUCCESS )
	{
		uint8_t *  pData = pParam->data + pParam->offset;

		switch( pRClient->CurrTrans.wTransType )
		{
		default:
			bATTError = ATT_ERR_UNLIKELY;
			break;

		case GATT_TYPE_CURR_TRANS_READ:
			if ( length > pTransRead->wRemainingSize )
			{
				iSize = pTransRead->wRemainingSize;
			}
			else
			{
		  		iSize = length;
			}
			attSendATT_READx_RESPONSE( pRClient->pL2CChannel,
			    (pRClient->CurrTrans.p.read.wReadOffset != 0) ?
			                        ATT_READ_BLOB_RESPONSE : ATT_READ_RESPONSE,
			    iSize, pData );
			break;

		case GATT_TYPE_DISCOVERY_CMP_UUID:
			/* "Characteristic Value Read Using UUID" */
			if ( pTransRead->wLastValueLength == 0 )
			{
				pTransRead->wLastValueLength = length;
			}
			else
			if ( length != pTransRead->wLastValueLength )
			{
				/* terminate search, ATT_READ_BY_TYPE_RESPONSE values must all */
				/* have same size !!!                                          */
				Terminate = TRUE;
			}

		    if ( pTransRead->wSize == sizeof(uint16_t) )
		    {
				/* first call of this routine, bValueLen=0 in attribute */
				/* definition (variable length)                         */
				if ( pTransRead->wRemainingSize >= length )
				{
					pTransRead->wSize += length;
				}
				else
				{
					Terminate = TRUE;
				}
		    }

		    if ( Terminate )
		    {
				/* remove handle from result list: */
				pTransRead->wTxOffset -= sizeof(uint16_t);
				pTransRead->wHandle   = pTransRead->wEndingHandle;
		    }
		    else
		    {
				if ( length != (pTransRead->wSize - sizeof(uint16_t)) )
				{
					/* size of attribute value mismatch !!! */
					bATTError = ATT_ERR_UNLIKELY;
					break;
				}

				/* copy attribute value to result buffer */
				memcpy( pTransRead->pWorkBuffer+pTransRead->wTxOffset,
				  pData, length );
				pTransRead->wTxOffset      += length;
				pTransRead->wRemainingSize -= length;
				iSize = pTransRead->wRemainingSize;
		    }

		    /* resume read with next handle */
		    if ( pTransRead->wHandle < pTransRead->wEndingHandle )
		    {
				pTransRead->wHandle++;
				gattAttributesSearch( FALSE, pRClient,

				                    (pRClient->pL2CChannel->bdType & BLUEFACE_BDTYPE_LE_MASK),

				                    pRClient->CurrTrans.wTransType, FALSE,
				                    &pTransRead->wHandle, pTransRead->wEndingHandle,
				                    pTransRead->wUUIDLength, pTransRead->bUUID,
				                    &iCnt,
				                    pTransRead->pWorkBuffer+pTransRead->wTxOffset,
				                    &iSize  /* size of list element on return */,
				                    -1 /* element may have variable length !! */);
				if ( iSize < 0 )
				{
					/* applic. has to supply value, update relevant transaction data => iCnt=1*/
					/*iSize = -iSize;*/
					/* handle of current list element is already stored: */
					pTransRead->wRemainingSize -= sizeof(uint16_t);
					pTransRead->wTxOffset      += sizeof(uint16_t);
					pTransRead->pGATTService    =
					    &pGATT->pServer->pService[GATT_ATTR_HANDLE_NBR_GET(pTransRead->wHandle)];

					DeferRsp = TRUE;            /* defer response */
					gattSendGATT_ATTRIB_READ_IND( pRClient->pL2CChannel );
				}
				else
				{
					/* total nbr. of list elements in result buffer: */
					iCnt += pTransRead->wTxOffset / pTransRead->wSize;
				}
		    }
		    else
		    {
				/* search is complete */
				iCnt = pTransRead->wTxOffset / pTransRead->wSize;
		    }

		    if ( !DeferRsp )
		    {
				/* send rsp */
				attSendATT_READ_BY_TYPE_RESPONSE( pRClient->pL2CChannel,
				              iCnt,                 /* nbr. of list elements */
				              pTransRead->wSize,    /* size of list element  */
				              pTransRead->pWorkBuffer );
		    }
		    break;
		}
	}
	else
	{
		/* map all causes to ATT_ERR_UNLIKELY with a few exceptions */
		bATTError = gattCauseToATTError( cause );
	}

	if ( bATTError != ATT_OK )
	{
		/* send error rsp */
		switch( pRClient->CurrTrans.wTransType )
		{
		default:
			bReqOpcode = 0;
			break;
		case GATT_TYPE_CURR_TRANS_READ:
			bReqOpcode = (pRClient->CurrTrans.p.read.wReadOffset != 0) ?
			                            ATT_READ_BLOB_REQUEST : ATT_READ_REQUEST;
			break;
		case GATT_TYPE_DISCOVERY_CMP_UUID:
			bReqOpcode = ATT_READ_BY_TYPE_REQUEST;
			break;
		}
		attSendATT_ERROR_RESPONSE( pRClient->pL2CChannel,
		                              bReqOpcode,
		                              pTransRead->wHandle, /* handle causing the error */
		                              bATTError );
	}

	if ( !DeferRsp )
	{
		pRClient->CurrTrans.wTransType = GATT_TYPE_UNDEFINED;
		osBufferRelease( pTransRead->pWorkBuffer );
		pTransRead->pWorkBuffer = NULL;
	}
}

/**
* @brief		handle "Write Request", "Write Command", "Prepare Write Request" received
* 			from client.
*
* @param	pL2CChannel: 
* @param	wType
* @param	wHandle 
* @param	iLength
* @param	wWriteOffset
* @param	pAttValue
* @param	pReleaseBuffer
*
* @return
*
*/
uint32_t gattHandleWriteReqCmd( PGATTL2CChannel pL2CChannel,
                             uint16_t            wType,
                             uint16_t            wHandle,
                             int             iLength,
                             uint16_t            wWriteOffset,
                             uint8_t *          pAttValue,
                             BOOL          * pReleaseBuffer )
{
	BOOL            SaveTransData = FALSE;
	uint32_t           dwATTError = ATT_OK;
	BOOL            DeferRsp   = FALSE;
	PGATTCurrTrans  pCurrTrans = &pL2CChannel->pRClient->CurrTrans;

	*pReleaseBuffer = TRUE;

	if ( pCurrTrans->wTransType != GATT_TYPE_UNDEFINED )
	{
		/* previous transaction not yet complete */
		if ( wType == GATT_WRITE_TYPE_CMD )
		{
			/* ATT_WRITE_COMMAND is allowed anyhow ("interrupts" other request)! */
			SaveTransData = TRUE;
		}
		else
		{
			dwATTError = ATT_ERR_UNLIKELY;
		}
	}

	if ( dwATTError == ATT_OK )
	{
		int  iSize = iLength;

		//XXXXMJMJ: for GATT_WRITE_TYPE_PREP check for extended properties
		//XXXXMJMJ  and GATT_RCODE_APPLIC_VALUE !!!!!:
		dwATTError = gattAttributeWrite( pL2CChannel->pRClient, wHandle,
		                        pAttValue, &iSize, (wType == GATT_WRITE_TYPE_CMD) );
		if ( dwATTError == ATT_OK )
		{
			if ( iSize < 0 )
			{
				if ( iSize == GATT_RCODE_APPLIC_VALUE )
				{
					/* applic. has to write value */
					TGATTCurrTrans CurrTrans;

					/* save "interrupted" ATT_WRITE_REQUEST data */
					if ( SaveTransData )
					{
						CurrTrans = *pCurrTrans;
					}

					/* applic. has to write value, save relevant transaction data */
					switch( wType )
					{
					default:
					case GATT_WRITE_TYPE_CMD:
						pCurrTrans->wTransType = GATT_TYPE_UNDEFINED;
						break;
					case GATT_WRITE_TYPE_REQ:
						pCurrTrans->wTransType = GATT_TYPE_CURR_TRANS_WRITE;
						DeferRsp = TRUE;
						break;
					}

					pCurrTrans->p.write.pGATTService =
					      &pGATT->pServer->pService[GATT_ATTR_HANDLE_NBR_GET(wHandle)];
					pCurrTrans->p.write.wHandle      = wHandle;
					pCurrTrans->p.write.wCCCBits     = 0;

					gattSendGATT_ATTRIB_WRITE_IND( pL2CChannel->pRClient,
					                             wType, iLength, wWriteOffset, pAttValue );
					*pReleaseBuffer = FALSE;

					/* restore "interrupted" other request data */
					if ( SaveTransData )
					{
						*pCurrTrans = CurrTrans;
					}

				}
				else if ( iSize == GATT_RCODE_CLIENT_CHAR_CFG )
				{
					/* special: write to client characteristic configuration attribute. */
					/* each client has its own instance of this attribute value which   */
					/* is administrated (per BD, together with other items) by BlueAPI: */
					PAttrib   pAttrib;
					int       iProperties;
					uint16_t      wCCCBits = LE_EXTRN2WORD( pAttValue );

					if ( iLength != sizeof(uint16_t) )
					{
						if ( iLength == 1 )
						{
							wCCCBits &= 0x00FF;
						}
						else
						{
							iSize    = 0;   /* NOP */
							if ( iLength > sizeof(uint16_t) )
							{
								dwATTError = ATT_ERR_INVALID_VALUE_SIZE;
							}
						}
					}

					if ( iSize != 0 )
					{
						/* cross check with characteristic properties, "rewind" to */
						/* characteristic definition:                              */
						pAttrib = gattGoToAttribute(  -1, wHandle,
						                             GATT_UUID_CHARACTERISTIC, NULL );
						if ( pAttrib != NULL )
						{
							iProperties = gattCharPropertiesGet( pAttrib );
							if ( (wCCCBits & ~(GATT_CLIENT_CHAR_CONFIG_NOTIFY |
							                        GATT_CLIENT_CHAR_CONFIG_INDICATE)) ||
							   (!(iProperties & GATT_CHAR_PROP_NOTIFY) &&
							          (wCCCBits & GATT_CLIENT_CHAR_CONFIG_NOTIFY))     ||
							   (!(iProperties & GATT_CHAR_PROP_INDICATE) &&
							          (wCCCBits & GATT_CLIENT_CHAR_CONFIG_INDICATE))
							 )
							{
								dwATTError = ATT_ERR_INVALID_CCC_BITS;
							}
							else
							{
										/* OK, valid value */
								int  iValueChanged;

#if (GATT_CCC_BITS_TO_BTSEC_AT_CONN_END)
								/* write CCC bits to local table */
								//XXXXMJMJ assure that free entry exists !!!!!!!!
								iValueChanged = gattCCCBitsSet( pL2CChannel,
								                                       TRUE, wHandle, wCCCBits );
								if ( iValueChanged < 0 )
								{
									dwATTError = ATT_ERR_INSUFFICIENT_RESOURCES;
								}
								else
								{
									if ( iValueChanged > 0 )
									{
										pL2CChannel->pRClient->bCCCBitsModified = 1;

										{
											TDEVICE_DATA_ELEMENT_GATT_CCC_BITS HandleValuePair;

											/* if requested by application send CCCD value */
											HandleValuePair.attHandle = wHandle;
											HandleValuePair.cccBits   = wCCCBits;
											gattSendGATT_CCCD_IND( pL2CChannel,
											                               TRUE, 1, &HandleValuePair );
										}
									}
								}

								/* check if write was to GATT/ServiceChanged CCCD */
								pAttrib++;     /* characteristic value follows definition */
								if ( gattAttribIsType16( pAttrib, GATT_UUID_CHAR_SERVICE_CHANGED ) )
								{
									pL2CChannel->pRClient->iServiceChangedHandle =
									                    gattAttributeToHandle( pAttrib );
									if ( wCCCBits == 0 )
									{
										/* remove handle from keystore */
										pL2CChannel->pRClient->iServiceChangedHandle =
										             -pL2CChannel->pRClient->iServiceChangedHandle;
									}
								}

#else
								pCurrTrans->p.write.wCCCBits = wCCCBits;

								//XXXXMJMJ assure that free entry in local table exists !!!
								if ( gattSendDEVICE_DATA_IND( DEVICE_DATA_SET_IND,
								                          DEVICE_DATA_TYPE_GATT_CCC_BITS, FALSE,
								                          pL2CChannel, wHandle, NULL ) != 0 )
								{
									DeferRsp   = FALSE;
									dwATTError = ATT_ERR_INSUFFICIENT_RESOURCES;
								}
#endif  /* GATT_CCC_BITS_TO_BTSEC_AT_CONN_END */
							}
						}
						else
						{
							dwATTError = ATT_ERR_UNLIKELY;
						}
				  	}
				}
			}

			if ( dwATTError == ATT_OK )
			{
				if ( !DeferRsp && (wType == GATT_WRITE_TYPE_REQ) )
				{
					attSendATT_GENERIC_PDU( pL2CChannel, ATT_WRITE_RESPONSE, offsetof(TATTPDUGeneric, Param),0,0,0,0,NULL);
				}
			}
			else
			{
				pCurrTrans->wTransType = GATT_TYPE_UNDEFINED;
			}
		}
	}

	if ( dwATTError != ATT_OK )
	{
		ATT_HANDLE2DWORD(dwATTError, wHandle);
	}

	return( dwATTError );
}
uint32_t gattHandlePrepareWriteReq( PGATTL2CChannel pL2CChannel,
                                 uint16_t            wHandle,
                                 int             iLength,
                                 uint16_t            wWriteOffset,
                                 uint8_t *          pAttValue,
                                 BOOL          * pReleaseBuffer )
{
    uint32_t           dwATTError = ATT_OK;
    PGATTCurrTrans  pCurrTrans = &pL2CChannel->pRClient->CurrTrans;
	*pReleaseBuffer = TRUE;

    if ( pCurrTrans->wTransType != GATT_TYPE_UNDEFINED )
    {
        dwATTError = ATT_ERR_UNLIKELY;
    }

    if ( dwATTError == ATT_OK )
    {
        dwATTError = gattCheckPrepareWrite( pL2CChannel->pRClient, wHandle);
    }

    if ( dwATTError == ATT_OK )
    {
        pCurrTrans->wTransType = GATT_TYPE_CURR_PROC_PREPARE_WRITE;

        pCurrTrans->p.write.pGATTService =
            &pGATT->pServer->pService[GATT_ATTR_HANDLE_NBR_GET(wHandle)];
        pCurrTrans->p.write.wHandle      = wHandle;
        pCurrTrans->p.write.wCCCBits     = 0;
        pCurrTrans->p.write.wWriteOffset = wWriteOffset;

        gattSendGATT_ATTRIB_WRITE_IND( pL2CChannel->pRClient,
                                       GATT_WRITE_TYPE_PREP, iLength, wWriteOffset, pAttValue );
		*pReleaseBuffer = FALSE;
    }

    if ( dwATTError != ATT_OK )
    {
        ATT_HANDLE2DWORD(dwATTError, wHandle);
    }

    return ( dwATTError );
}

uint32_t gattHandleExecuteWriteReq( PGATTL2CChannel pL2CChannel,
                                 uint8_t            bFlags )
{
    uint32_t           dwATTError = ATT_OK;
    PGATTCurrTrans  pCurrTrans = &pL2CChannel->pRClient->CurrTrans;

    if ( pCurrTrans->wTransType != GATT_TYPE_UNDEFINED )
    {
        dwATTError = ATT_ERR_UNLIKELY;
    }

    if ( dwATTError == ATT_OK )
    {
        pCurrTrans->wTransType = GATT_TYPE_CURR_PROC_EXECUTE_WRITE;

        gattSendGATT_EXECUTE_WRITE_IND(pL2CChannel, bFlags);
    }

    return ( dwATTError );

}

BOOL gattHandlePrepareWriteResponse( PGATTL2CChannel pL2CChannel,
                                     uint16_t            wType,
                                     uint16_t            valueOffset,
                                     int             iLength,
                                     uint8_t *          pValue )
{
    //BOOL ReleaseBuffer;

    gattSendGATT_ATTRIB_PREPARE_WRITE_CONF(GATT_SUCCESS,
                    pL2CChannel->cid, valueOffset,
                    iLength, pValue );
    pL2CChannel->pClient->CurrProc.wProcType = GATT_TYPE_UNDEFINED;

    return ( TRUE );
}

void gattHandleExecuteWriteResponse( PGATTL2CChannel pL2CChannel )
{
    gattSendGATT_EXECUTE_WRITE_CONF( GATT_SUCCESS, pL2CChannel->cid);
    pL2CChannel->pClient->CurrProc.wProcType = GATT_TYPE_UNDEFINED;
}

void gattExecuteWriteResumeAppl( PGATTL2CChannel pL2CChannel,
                                 uint16_t cause, uint16_t handle)
{
    PGATTRClient        pRClient;
    uint8_t                bATTError = ATT_OK;

    pRClient    = pL2CChannel->pRClient;

    if ( cause != GATT_SUCCESS )
    {
        /* map all causes to ATT_ERR_UNLIKELY with a few exceptions */
        bATTError = gattCauseToATTError( cause );
    }

    if ( bATTError == ATT_OK )
    {
        attSendATT_EXECUTE_WRITE_RESPONSE( pRClient->pL2CChannel );
    }
    else
    {
        attSendATT_ERROR_RESPONSE( pRClient->pL2CChannel,
                                   ATT_EXECUTE_WRITE_REQUEST,
                                   handle, /* handle causing the error */
                                   bATTError );
    }

    pRClient->CurrTrans.wTransType = GATT_TYPE_UNDEFINED;

}

/**
* @brief		resume write operation after reception of server application response
* 			attribute value
*
* @param	pL2CChannel
* @param	cause
*
* @return
*
*/
void gattWriteResumeAppl( PGATTL2CChannel pL2CChannel,
									uint16_t cause, uint16_t length, uint16_t offset, uint8_t * pBuffer)
{
	PGATTRClient        pRClient;
	PGATTTransWrite     pTransWrite;
	uint8_t                bATTError = ATT_OK;
    uint8_t                bReqOpcode;

	pRClient    = pL2CChannel->pRClient;
	pTransWrite = &pRClient->CurrTrans.p.write;

	if ( cause != GATT_SUCCESS )
	{
		/* map all causes to ATT_ERR_UNLIKELY with a few exceptions */
		bATTError = gattCauseToATTError( cause );
	}

	if ( bATTError == ATT_OK )
	{

        uint8_t *  pData = pBuffer + offset;

		switch( pRClient->CurrTrans.wTransType )
		{
		default:
		case GATT_TYPE_CURR_TRANS_WRITE:
			attSendATT_GENERIC_PDU( pL2CChannel, ATT_WRITE_RESPONSE, offsetof(TATTPDUGeneric, Param),0,0,0,0,NULL );	
			break;

        case GATT_TYPE_CURR_TRANS_PREPARE_WRITE:
            attSendATT_PREPARE_WRITE_RESPONSE( pRClient->pL2CChannel,
                                               pTransWrite->wHandle,
                                               pTransWrite->wWriteOffset,
                                               length,
                                               pData);
            break;

		}
	}
	else
	{
        switch ( pRClient->CurrTrans.wTransType )
        {
        default:
        case GATT_TYPE_CURR_TRANS_WRITE:
            bReqOpcode = ATT_WRITE_REQUEST;
            break;

        case GATT_TYPE_CURR_TRANS_PREPARE_WRITE:
            bReqOpcode = ATT_PREPARE_WRITE_REQUEST;
            break;

        }
		/* send error rsp */
		attSendATT_ERROR_RESPONSE( pRClient->pL2CChannel,
		                              bReqOpcode,
		                              pTransWrite->wHandle, /* handle causing the error */
		                              bATTError );
	}

	pRClient->CurrTrans.wTransType = GATT_TYPE_UNDEFINED;
}

#if (!GATT_CCC_BITS_TO_BTSEC_AT_CONN_END)
/**
* @brief		 resume write operation after reception of BTSEC response
*
* @param	pRClient
* @param	wCause
*
* @return
*
*/
void gattWriteResumeBTSEC( PGATTRClient pRClient, uint16_t wCause )
{
	PGATTL2CChannel     pL2CChannel = pRClient->pL2CChannel;
	PGATTTransWrite     pTransWrite = &pRClient->CurrTrans.p.write;

	if ( wCause == BTSEC_SUCCESS )
	{
		/* write CCC bits to local table too */
		if ( gattCCCBitsSet( pTransWrite->wHandle, pL2CChannel,
		                                             pTransWrite->wCCCBits ) < 0 )
		{
			wCause = BTSEC_ERR_UNSPECIFIED;
		}
	}

	if ( pTransWrite->Cmd )
	{
		/* no response at all for ATT_WRITE_COMMAND */
	}
	else
	{
		if ( wCause != 0  )
		{
			/* error */
			attSendATT_ERROR_RESPONSE( pL2CChannel,
			                            ATT_WRITE_REQUEST,
			                            pTransWrite->wHandle, /* handle causing the error */
			                            ATT_ERR_INSUFFICIENT_RESOURCES );
		}
		else
		{
			attSendATT_GENERIC_PDU( pL2CChannel, ATT_WRITE_RESPONSE , offsetof(TATTPDUGeneric, Param),0,0,0,0,NULL);	  
		}
	}

	pRClient->CurrTrans.wTransType = GATT_TYPE_UNDEFINED;
}
#endif 

