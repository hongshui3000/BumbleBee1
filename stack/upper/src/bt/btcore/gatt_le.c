/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       gatt_le.c
* @brief     gatt LE specific routines
* @details   
*
* @author   	gordon
* @date      	2015-07-09
* @version	v0.1
*/

#include <flags.h>
#include <gattdef.h>
#include <gatt_api.h>
#include <blueapi_api.h>
#include <btsm_api.h>
#include <hci_code.h>
#include <hci_api.h>
#include <l2c_api.h>
#include <upper_stack_global.h>

#define TRACE_MODULE_ID     MID_BT_L2C
/**
* @brief  find LE channel based on handle or pGATT->pLELockedChannel
*
* @param  handle
* @param  status
*
* @return  
*
*/
PGATTL2CChannel gattLEChannelFind( uint16_t wHandle )
{
	PGATTL2CChannel  pL2CChannel;

	if ( (pL2CChannel = gattL2CChannelFindHandle( TRUE, wHandle )) == NULL )
	{
		/* early connection phase, connection handle is not yet available */
		pL2CChannel = pGATT->pLELockedChannel;
	}

	return( pL2CChannel );
}

/**
* @brief   send HCI_LE_SET_ADVERTISING_PARAMETERS command
*
* @param  pAdvParam
* @param  bd
* @param  bdType
*
* @return  
*
*/
int gattSendLE_SET_ADVERTISING_PARAMETERS( PGATTLEAdvertiseParameter pAdvParam,
                                           LPBdAddr bd, uint8_t bdType)
{
	if (bd != NULL)
	{
		uint8_t state = btsmMapRandomAddress((uint8_t*)bd,
										&bdType);

		if (state == LE_RESOLVE_ENTRY_STATE_NEGATIVE)
		{
			gattHandleCmdCompLE_SET_ADVERTISING_PARAMETERS(SECMAN_ERR | SECMAN_ERR_LE_BD_NOT_RESOLVED);
			return 0;
		}
	}
	/*l2cap nothing todo, just send to hci*/
	/*hci layer*/
	{
        if(bd != NULL)
        {
            hciCommandLESetAdvertisingParameters(pAdvParam->minAdvInterval, pAdvParam->maxAdvInterval,
    	                                      LE_ADVERTISING_TYPE_ADV_DIRECT_HIGH_DUTY_IND, pAdvParam->local_bdType, 
    	                                      bdType, (uint8_t*)bd, pAdvParam->advChannelMap,
    	                                      pAdvParam->filterPolicy);
        }
        else
        {
            hciCommandLESetAdvertisingParameters(pAdvParam->minAdvInterval, pAdvParam->maxAdvInterval,
    	                                      pAdvParam->advType, pAdvParam->local_bdType, pAdvParam->bdType,
    	                                      pAdvParam->bd, pAdvParam->advChannelMap,
    	                                      pAdvParam->filterPolicy);

        }
		hciLaterEntry();		
	}

	return 0;
}

/**
* @brief   enable/disable adv
*
* @param  Enable
*
* @return  
*
*/
int gattSendLE_SET_ADVERTISE_ENABLE( uint8_t Enable )
{
	hciCommandByteParameter(HCI_LE_SET_ADVERTISE_ENABLE, Enable);
	hciLaterEntry();		

	return 0;
}

/**
* @brief  send LE_SET_SCAN_ENABLE command
*
* @param  Enable
* @param  filterDups
*
* @return  
*
*/
int gattSendLE_SET_SCAN_ENABLE( uint8_t Enable, uint8_t filterDups )
{
    hciCommandLESetScanEnable(Enable, filterDups);
	hciLaterEntry();		
	return 0;
}

/**
* @brief  send HCI_LE_SET_SCAN_PARAMETERS command
*
* @param  pScanParam
*
* @return  
*
*/

int gattSendLE_SET_SCAN_PARAMETERS( PGATTLEScanReq pScanParam )
{
    hciCommandLESetScanParameters(pScanParam->scanType, pScanParam->scanInterval, pScanParam->scanWindow,
	                               pScanParam->localBdType, pScanParam->filterPolicy);
	hciLaterEntry();	

	return 0;
}

/**
* @brief  gatt send HCI_LE_CLEAR_WHITE_LIST
*
* @param  appHandle
* @param  opCode
* @param  bd
* @param  bdType
*
* @return  
*
*/
int gattSendLE_CLEAR_WHITELIST()
{
	hciCommandNoParameter(HCI_LE_CLEAR_WHITE_LIST);
//	hciLaterEntry();		//randy--611

	return 0;
}

/**
* @brief  gatt send HCI_LE_ADD_DEVICE_TO_WHITE_LIST command
*
* @param  bd
* @param  bdType
*
* @return  
*
*/
int gattSendLE_ADD_DEVICE_TO_WHITELIST( LPBdAddr bd, uint8_t bdType )
{	
	uint8_t state = btsmMapRandomAddress((uint8_t *)bd,
									&bdType);

	if (state == LE_RESOLVE_ENTRY_STATE_NEGATIVE)
	{	
		uint16_t opCode = pGATT->LE_whitelist.opCode;
		pGATT->LE_whitelist.opCode = 0;
		gattSendGATT_LE_MODIFY_WHITELIST_CONF(opCode, SECMAN_ERR | SECMAN_ERR_LE_BD_NOT_RESOLVED);
		return 0;
	}
	hciCommandLEChangeDeviceWhiteList(HCI_LE_ADD_DEVICE_TO_WHITE_LIST, (uint8_t *)bd, bdType);
	hciLaterEntry();		//randy--611
	return 0;
}

/**
* @brief  gatt send LE_REMOVE_DEVICE_FROM_WHITE_LIST command
*
* @param  bd
* @param  bdType
*
* @return  
*
*/
int gattSendLE_REMOVE_DEVICE_FROM_WHITELIST( LPBdAddr bd, uint8_t bdType )
{
	uint8_t state = btsmMapRandomAddress((uint8_t *)bd,
									&bdType);

	if (state == LE_RESOLVE_ENTRY_STATE_NEGATIVE)
	{
		uint16_t opCode = pGATT->LE_whitelist.opCode;
		pGATT->LE_whitelist.opCode = 0;
		gattSendGATT_LE_MODIFY_WHITELIST_CONF(opCode, SECMAN_ERR | SECMAN_ERR_LE_BD_NOT_RESOLVED);

		return 0;
	}
	hciCommandLEChangeDeviceWhiteList(HCI_LE_REMOVE_DEVICE_FROM_WHITE_LIST, (uint8_t *)bd, bdType);
	hciLaterEntry();		//randy--611

	return 0;
}

/**
* @brief  gatt send LE_CREATE_CONN command
*
* @param  pL2CChannel
* @param  pConPara
*
* @return  
*
*/
int gattSendLE_CREATE_CONNECTION( PGATTL2CChannel pL2CChannel, PCON_REQ_GATT pConPara )
{
	uint8_t		nullBD[6] = { 0, 0, 0, 0, 0, 0 };
	uint8_t	peerAddressType;
	uint8_t    initiatorFilterPolicy;
    TBdAddr peerAddress;

    if ( pGATT->pLELockedChannel != NULL )
    {
        /* early connection phases must be serialized :-( !!!! ..   */
        /* (BTMAN serialization is not sufficient: resets semaphore */
        /* already when GATT_CON_CONF is received):                 */
        return -2;
    }
    else
    {
    	/*take these before*/
    	gattL2CChangeState( pL2CChannel, gattL2CStateConConfPending );
    	
    	if (memcmp(pL2CChannel->RemoteBd, nullBD, BD_ADDR_SIZE) != 0)
    	{
    		initiatorFilterPolicy = LE_INITIATOR_FILTER_POLICY_WHITE_LIST_NOT_USED;
    		peerAddressType	   = pConPara->bdType;
    	}
    	else
    	{
    		initiatorFilterPolicy = LE_INITIATOR_FILTER_POLICY_WHITE_LIST_USED;
    		peerAddressType	   = BLUEFACE_BDTYPE_LE_PUBLIC;
    	}

    	/* must save reference to channel since ops must be serialized in */
    	/* early connection phase :-( .... :							  */
    	pGATT->pLELockedChannel = pL2CChannel;
    	
    	{

            memcpy(peerAddress, pL2CChannel->RemoteBd, BD_ADDR_SIZE);
    		uint8_t state = btsmMapRandomAddress(peerAddress,
    										&peerAddressType);

    		if (state == LE_RESOLVE_ENTRY_STATE_NEGATIVE)
    		{
    			gattHandleLECreateConnectionConf(0 , SECMAN_ERR | SECMAN_ERR_LE_BD_NOT_RESOLVED);
    			return 0;
    		}
    	}
        btsmUpdateLocalBdType(pConPara->localBdType);
    	{
            hciCommandLECreateConnection(pConPara->scanInterval, pConPara->scanWindow,
    	                              initiatorFilterPolicy, peerAddressType, peerAddress,
    	                              pConPara->localBdType, pConPara->connIntervalMin, pConPara->connIntervalMax,
    	                              pConPara->connLatency, pConPara->supervisionTimeout, 
    	                              pConPara->CE_Length, pConPara->CE_Length);
    		hciLaterEntry();		//randy--611
    	}
    }
	return 0;
}

/**
* @brief  gatt send LE_CREATE_CONNECTION_CANCEL command
*
* @param  pL2CChannel
*
* @return  
*
*/
int gattSendLE_CREATE_CONNECTION_CANCEL( PGATTL2CChannel pL2CChannel )
{
	hciCommandNoParameter(HCI_LE_CREATE_CONNECTION_CANCEL);
//	hciLaterEntry();		//randy--611

	return 0;
}

/**
* @brief  send LE DATA
*
* @param  pL2CChannel:
* @param  pBuffer
* @param  wLength
* @param  wOffset
*
* @return  
*
*/
int gattSendLE_DATA( PGATTL2CChannel pL2CChannel,
                               uint8_t * pBuffer, uint16_t wLength, uint16_t wOffset )
{
    MESSAGE_T Message;

	GATT_TRACE_PRINTF_2(GATT_TRACE_MASK_TRACE, "GATT: LE_DATA, len=0x%x, connHandle=0x%x",
	                 wLength, pL2CChannel->wLEConnectionHandle);

	Message.Command        = LE_MESSAGE_REQ;
	Message.MData.DataCB.BufferAddress = pBuffer;
	Message.MData.DataCB.Offset        = wOffset;
	Message.MData.DataCB.Length        = wLength;
	Message.MData.DataCB.Flag          = DATA_CB_RELEASE;

	l2cLEHandleDataReq(&Message, CID_ATTRIBUTE_PROTOCOL, pL2CChannel->wLEConnectionHandle);

	return( 0 );
}

/**
* @brief  send LE_DISCONNECT
*
* @param  pL2CChannel:
* @param  wStatus
*
* @return  
*
*/
int gattSendLE_DISCONNECT( PGATTL2CChannel pL2CChannel, uint16_t wStatus )
{
	gattL2CChangeState( pL2CChannel, gattL2CStateDiscPending );

	hciCommandDisconnect( pL2CChannel->wLEConnectionHandle, HCI_ERR_OTHER_END_TERMINATE_13);
	hciLaterEntry();		//randy--611
	return 0;
}

/**
* @brief   gatt handle le set adv param confirm
*
* @param  status
*
* @return  
*
*/
void gattHandleCmdCompLE_SET_ADVERTISING_PARAMETERS(uint16_t status)
{
	uint16_t opCode = pGATT->LE_adv.opCode;
	/* store first error */
	if ((status != HCI_SUCCESS) && (pGATT->LE_adv.status == HCI_SUCCESS))
	{
		pGATT->LE_adv.status = status;
	}

	switch (pGATT->LE_adv.opCode)
	{
	case GATT_LE_ADVERTISE_OPCODE_AD_PARAMETER:
		{
			if (pGATT->LE_adv.enabled)
			{
				/* parameters set, re-enable advertising again */
				pGATT->LE_adv.opCodeCurrent = GATT_LE_ADVERTISE_OPCODE_AD_ENABLE;
				gattSendLE_SET_ADVERTISE_ENABLE(TRUE);
			}
			else
			{
				pGATT->LE_adv.opCode        = 0;
				pGATT->LE_adv.opCodeCurrent = 0;
				/* parameters set and advertising not enabled */
				gattSendGATT_LE_ADVERTISE_CONF(opCode, pGATT->LE_adv.status);

			}
		}
		break;

	/* [ */
	case GATT_LE_ADVERTISE_OPCODE_AD_DIRECTED:
	  if (pGATT->LE_adv.opCodeCurrent == GATT_LE_ADVERTISE_OPCODE_AD_DIRECTED)
	  {
	    /* directed parameters set, re-enable advertising again */
	    pGATT->LE_adv.opCodeCurrent = GATT_LE_ADVERTISE_OPCODE_AD_ENABLE;
	    gattSendLE_SET_ADVERTISE_ENABLE(TRUE);
	  }
	  else if (pGATT->LE_adv.opCodeCurrent == GATT_LE_ADVERTISE_OPCODE_AD_PARAMETER)
	  {
	    if (pGATT->LE_adv.enabled)
	    {
	      /* normal parameters set, re-enable advertising again */
	      pGATT->LE_adv.opCodeCurrent = GATT_LE_ADVERTISE_OPCODE_AD_ENABLE;
	      gattSendLE_SET_ADVERTISE_ENABLE(TRUE);
	    }
	    pGATT->LE_adv.opCode        = 0;
	    pGATT->LE_adv.opCodeCurrent = 0;

	    /* normal parameters set, send CONF */
	    gattSendGATT_LE_ADVERTISE_CONF(opCode, pGATT->LE_adv.status);

	  }
	  break;

	default:
	  break;
	}

}

void gattHandleCmdCompLE_SET_ADV_DATA(uint16_t status)
{
	if (pGATT->LE_adv.opCode != 0)
	{
		uint16_t opCode = pGATT->LE_adv.opCode;
		pGATT->LE_adv.opCode		  = 0;
		pGATT->LE_adv.opCodeCurrent = 0;
		gattSendGATT_LE_ADVERTISE_CONF(opCode, status);
	}
}

void gattHandleCmdCompLE_SET_SCAN_PARAMETERS(uint16_t status)
{
    if ((status == HCI_SUCCESS) && pGATT->LE_scan.param.enable)
	{
		/* parameters set, enable scan */
		pGATT->LE_scan.enabled = TRUE;
		gattSendLE_SET_SCAN_ENABLE(TRUE, pGATT->LE_scan.param.filterDuplicates);
	}
	else
	{
		/* parameters set, send CONF */
        blueAPI_Send_LEScanRsp(status);
	}
}

void gattHandleCmdCompLE_SET_SCAN_ENABLE(uint16_t status)
{
    if (status != HCI_SUCCESS)
	{
		/* revert state */
		pGATT->LE_scan.enabled = !pGATT->LE_scan.enabled;
        blueAPI_Send_LEScanRsp(status);
	}
	else if (pGATT->LE_scan.enabled && pGATT->LE_scan.param.enable)
	{
		/* parameters set and scan enabled, send CONF */
        blueAPI_Send_LEScanRsp(status);
	}
	else if (!pGATT->LE_scan.enabled)
	{
		/* scan disabled, set parameters */
		gattSendLE_SET_SCAN_PARAMETERS(&pGATT->LE_scan.param);
	}
}

void gattHandleCmdCompLE_MODIFY_WHITE_LIST(uint16_t status)
{
    uint16_t opCode = pGATT->LE_whitelist.opCode;
	pGATT->LE_whitelist.opCode = 0;
	gattSendGATT_LE_MODIFY_WHITELIST_CONF(opCode, status);
}
/**
* @brief  gatt  handle le create conn confirm
*
* @param  handle
* @param  status
*
* @return  
*
*/
void gattHandleLECreateConnectionConf(uint16_t handle, uint16_t status)

{
	PGATTL2CChannel pL2CChannel;

	if ( (pL2CChannel = gattLEChannelFind( handle )) != NULL )
	{
		/* map channel nbr. to msg parameter cid, used by BTMAN for routing: */
		pL2CChannel->cid = GATT_IDX_TO_LE_CID(pL2CChannel->wNbr);

		if ( status == 0 )
		{
			gattL2CChangeState( pL2CChannel, gattL2CStateConnecting );
		}
		/* else application GATT_DISC_RESP will cleanup .. */
		gattLLConnectConf( pL2CChannel, status );
	}
}

/**
* @brief   gatt handle le set adv disable/enable confirm
*
* @param  status
*
* @return  
*
*/
void gattHandleCmdCompLE_SET_ADVERTISE_ENABLE( uint16_t status)
{
	uint16_t opCode = pGATT->LE_adv.opCode;
	/* store first error */
	if ((status != HCI_SUCCESS) && (pGATT->LE_adv.status == HCI_SUCCESS))
	{
		pGATT->LE_adv.status = status;
	}

	switch (pGATT->LE_adv.opCode)
	{
	case GATT_LE_ADVERTISE_OPCODE_AD_PARAMETER:
		if (pGATT->LE_adv.opCodeCurrent == GATT_LE_ADVERTISE_OPCODE_AD_ENABLE)
		{
			pGATT->LE_adv.opCode		= 0;
			pGATT->LE_adv.opCodeCurrent = 0;
			/* advertising enabled after changing parameters, send CONF */
			gattSendGATT_LE_ADVERTISE_CONF(opCode, pGATT->LE_adv.status);

		}
		else
		{
			/* advertising disabled, change parameters */
			pGATT->LE_adv.opCodeCurrent = GATT_LE_ADVERTISE_OPCODE_AD_PARAMETER;
			gattSendLE_SET_ADVERTISING_PARAMETERS( &pGATT->LE_adv.param, NULL, 0);
		}
		break;

	case GATT_LE_ADVERTISE_OPCODE_AD_ENABLE:
	case GATT_LE_ADVERTISE_OPCODE_AD_DISABLE:
		/* only send CONF if triggered by application (not by GATT server) */
		if (pGATT->LE_adv.opCode != 0)
		{
			pGATT->LE_adv.opCode		= 0;
			pGATT->LE_adv.opCodeCurrent = 0;
			gattSendGATT_LE_ADVERTISE_CONF(opCode, pGATT->LE_adv.status);

		}
		break;

	case GATT_LE_ADVERTISE_OPCODE_AD_DIRECTED:
		if (pGATT->LE_adv.opCodeCurrent == GATT_LE_ADVERTISE_OPCODE_AD_DISABLE)
		{
			TGATTLEAdvertiseParameter advParam;

			/* only advType is used */
			advParam.advType		= LE_ADVERTISING_TYPE_ADV_DIRECT_HIGH_DUTY_IND;
			advParam.advChannelMap	= LE_ADVERTISING_CHANNEL_ALL;
			advParam.filterPolicy	= 0x00;
			advParam.minAdvInterval = 0x20;
			advParam.maxAdvInterval = 0x20;
            advParam.local_bdType = pGATT->LE_adv.directed.local_bdType;

			pGATT->LE_adv.opCodeCurrent = GATT_LE_ADVERTISE_OPCODE_AD_DIRECTED;
			gattSendLE_SET_ADVERTISING_PARAMETERS(&advParam,
												  (LPBdAddr)&pGATT->LE_adv.directed.bd,
												  pGATT->LE_adv.directed.bdType
												  );
		}
		else if (pGATT->LE_adv.opCodeCurrent == GATT_LE_ADVERTISE_OPCODE_AD_ENABLE)
		{
			/* error when directed adv enabled */
			if (pGATT->LE_adv.status != HCI_SUCCESS)
			{
				/* revert parameters (not directed mode) */
				gattSendLE_SET_ADVERTISING_PARAMETERS(&pGATT->LE_adv.param, NULL, 0);
				pGATT->LE_adv.opCode		  = 0;
				pGATT->LE_adv.opCodeCurrent = 0;

				/* send error CONF */
				gattSendGATT_LE_ADVERTISE_CONF(opCode, pGATT->LE_adv.status);
			}
		}
		break;

	default:
		break;
	}
}

/**
* @brief  gatt handle le connect update confirm
*
* @param  handle
* @param  status
*
* @return  
*
*/
void gattHandleLEConnectionUpdateConf( uint16_t handle, uint16_t status)
{
	PGATTL2CChannel pL2CChannel;

	pL2CChannel = gattLEChannelFind(handle);
	if (pL2CChannel != NULL)
	{
		/* do not send CONF if update was triggered from slave */
		if (!pL2CChannel->slaveConnectionUpdate)
		{
			gattSendGATT_LE_CONNECTION_UPDATE_CONF(status, pL2CChannel->cid);
		}
		pL2CChannel->slaveConnectionUpdate = FALSE;
	}
}

void gattHandleHCI_LE_CONNECTION_COMPLETE_EVENT( uint16_t Handle, uint16_t Status, uint8_t peerAddressType,
			                                 uint8_t * peerAddress, uint16_t connInterval,
			                                 uint16_t connLatency, uint16_t supervisionTimeout)
{
	PGATTL2CChannel pL2CChannel;

	pL2CChannel = gattLEChannelFind( Handle );

	/* new connection */
	if ((pL2CChannel == NULL) && (Status == 0))
	{
		/* incoming call */
		if ( (pL2CChannel = gattL2CChannelAllocate( TRUE )) != NULL )
		{
			/* map channel nbr. to msg parameter cid, used by BTMAN for routing: */
			pL2CChannel->cid = GATT_IDX_TO_LE_CID(pL2CChannel->wNbr);
		}
		else
		{
			assert(FALSE);	 //XXXXMJMJ
		}
	}

	/* advertising resulted in new connection, disable advertising */
	if ((Status == 0) && (pGATT->LE_adv.enabled == TRUE))
	{
		pGATT->LE_adv.enabled = FALSE;
	}

	/* directed advertising was active */
	if (pGATT->LE_adv.opCode == GATT_LE_ADVERTISE_OPCODE_AD_DIRECTED)
	{
		/* store first error (timeout) */
		if ((Status != HCI_SUCCESS) && (pGATT->LE_adv.status == HCI_SUCCESS))
		{
			pGATT->LE_adv.status = Status;
		}

		/* revert parameters (not directed mode) */
		pGATT->LE_adv.opCodeCurrent = GATT_LE_ADVERTISE_OPCODE_AD_PARAMETER;
		gattSendLE_SET_ADVERTISING_PARAMETERS(&pGATT->LE_adv.param, NULL, 0);

		/* send CONF now (before CON_IND arrives at application) */
		if (Status == HCI_SUCCESS)		//randy--623
		{
			uint16_t opCode = pGATT->LE_adv.opCode;
			pGATT->LE_adv.opCode		= 0;
			pGATT->LE_adv.opCodeCurrent = 0;
			gattSendGATT_LE_ADVERTISE_CONF(opCode, pGATT->LE_adv.status);

		}
	}

	if (pL2CChannel != NULL)
	{
		if ( Status == 0 )
		{
			pL2CChannel->wLEChannel		  = CID_ATTRIBUTE_PROTOCOL; /*channle change in btsm layer*/
			pL2CChannel->wLEConnectionHandle = Handle;
			pL2CChannel->bdType			  = peerAddressType;
			memcpy( pL2CChannel->RemoteBd,
				 peerAddress,
				 sizeof(TBdAddr) );

			/* save current parameters */
			pL2CChannel->conParam.connIntervalMin	  = connInterval;
			pL2CChannel->conParam.connIntervalMax	  = connInterval;
			pL2CChannel->conParam.connLatency		  = connLatency;
			pL2CChannel->conParam.supervisionTimeout = supervisionTimeout;
			pL2CChannel->conParam.minimumCELength	  = 0;
			pL2CChannel->conParam.maximumCELength	  = 0;

			if ( pL2CChannel->OutgoingCall )
			{
				gattL2CChangeState( pL2CChannel, gattL2CStateConnected );
				gattLLConnected( pL2CChannel, 0 );
			}
			else
			{
				gattL2CChangeState( pL2CChannel, gattL2CStateConnecting );
				gattLLConnectInd( pL2CChannel );
			}
		}
		else
		{
			gattL2CChangeState( pL2CChannel, gattL2CStateDisconnecting );
			gattLLDisconnectInd( pL2CChannel, Status );		//randy--623
		}
		pGATT->pLELockedChannel = NULL;
	}  
}

void gattHandleLE_HCI_DISCONNECTION_COMPLETE( uint16_t Handle, uint16_t Reason)
{
   
	PGATTL2CChannel pL2CChannel;

	pL2CChannel = gattLEChannelFind( Handle);
	if(pL2CChannel != NULL)
	{
		if ( pL2CChannel->wConRespStatus == L2CAP_CONNECTION_ACCEPT )
		{
			gattLLDisconnectInd( pL2CChannel, Reason );
			gattL2CChangeState( pL2CChannel, gattL2CStateDisconnecting );
		}
		else
		{
			/* connection reject with GATT_CON_RESP/status != 0, special */
			/* needed: no DISC_IND to BTMAN !!!:                         */
			gattLLDisconnected( pL2CChannel, Reason, FALSE, TRUE );
		}
	}

}

void gattHandleHCI_LE_ADVERTISING_REPORT_EVENT( uint8_t eventType, uint8_t addressType,
                                            TBdAddr address, uint8_t rssi, uint8_t dataLength,
                                            uint8_t *    data)
{
    if ((pGATT->LE_scan.param.enable) &&(eventType != LE_EVENT_TYPE_ADV_DIRECT_IND))
	{
        blueAPI_Send_LEScanInfo(eventType, addressType, address, rssi, dataLength, data);
    }
}

void gattHandleHCI_LE_CONNECTION_UPDATE_COMPLETE_EVENT(uint16_t Handle, uint16_t status,
                                                   uint16_t connInterval, 
                                                   uint16_t connLatency, uint16_t supervisionTimeout)
{

	PGATTL2CChannel pL2CChannel;

	pL2CChannel = gattLEChannelFind( Handle);

	if (pL2CChannel != NULL)
	{ 
		  /* save current parameters */
		  pL2CChannel->conParam.connIntervalMin    = connInterval;
		  pL2CChannel->conParam.connIntervalMax    = pL2CChannel->conParam.connIntervalMin;
		  pL2CChannel->conParam.connLatency        = connLatency;
		  pL2CChannel->conParam.supervisionTimeout = supervisionTimeout;
		  pL2CChannel->conParam.minimumCELength    = 0;
		  pL2CChannel->conParam.maximumCELength    = 0;

		  /* forward to application */
		  gattSendGATT_LE_CONNECTION_UPDATE_IND(pL2CChannel,
		                                        GATT_LE_CONNECTION_UPDATE_TYPE_EVENT,
		                                        &pL2CChannel->conParam);
	}
		
}



void gattHandleL2CAP_CONNECTION_PARAMETER_UPDATE_REQUEST(uint16_t handle,PGATTLEConnectionUpdateParam pParam)
{

	PGATTL2CChannel pL2CChannel;
	pL2CChannel = gattLEChannelFind( handle);
    if (pL2CChannel != NULL)
    {
        memcpy(&(pL2CChannel->conParam), pParam, sizeof(TGATTLEConnectionUpdateParam));

		/* remember that slave requested the connection update */
		pL2CChannel->slaveConnectionUpdate = TRUE;

		/* forward to application */
		gattSendGATT_LE_CONNECTION_UPDATE_IND(pL2CChannel,
											  GATT_LE_CONNECTION_UPDATE_TYPE_REQUEST,
											  &pL2CChannel->conParam);
    }
		
}

