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
#include <hci_api.h>
#include <hci_code.h>
#include <gatt_api.h>
#include <btsm_api.h>
#include <l2c_api.h>
#include <blueapi_api.h>
#include <upper_stack_global.h>

#define TRACE_MODULE_ID     MID_BT_HCI

/**
* @brief  hci LE handle disconnect complete
*
* @param  Handle
* @param  Reason
*
* @return  
*
*/
LPLEMessage hciLEGetBufferUs(uint16_t MsgType, uint16_t Handle, uint16_t Status, uint16_t Length)
{
  LPLEMessage pLEMsg;

  if (osBufferGet(BTSystemPoolID, (uint16_t)(Length + offsetof(TLEMessage, p)), (void  *)&pLEMsg))
  {
    DebuggerBreak();
    return((LPLEMessage)0);
  }

  pLEMsg->msgType = MsgType;
  pLEMsg->status  = Status;
  pLEMsg->handle  = Handle;

  return(pLEMsg);
}

void hciCommandLEStartEncryption(uint16_t handle, uint8_t * randomNumber, 
                                          uint16_t encryptedDiversifier, uint8_t * longTermKey)
{
    uint8_t mbuf[HCI_MAX_SHORT_CMD_SIZE];
    uint16_t pos = 0;

    SHORT2CHAR(mbuf+pos, handle); pos += 2;
    memcpy(mbuf+pos, randomNumber, LE_RANDOM_NUMBER_SIZE); pos += LE_RANDOM_NUMBER_SIZE;
    SHORT2CHAR(mbuf+pos, encryptedDiversifier); pos += 2;
    memcpy(mbuf+pos, longTermKey, LE_LONG_TERM_KEY_SIZE); pos += LE_LONG_TERM_KEY_SIZE;

    hciSendDsCommand(pHCI->hciTransQueueID, HCI_LE_START_ENCRYPTION, mbuf, pos);
}


void hciCommandLELongTermKeyRequestReply(uint16_t handle, uint8_t * longTermKey)
{
    uint8_t mbuf[HCI_MAX_SHORT_CMD_SIZE];
    uint16_t pos = 0;

    SHORT2CHAR(mbuf+pos, handle); pos += 2;
    memcpy(mbuf+pos, longTermKey, LE_LONG_TERM_KEY_SIZE); pos += LE_LONG_TERM_KEY_SIZE;

    hciSendDsCommand(pHCI->hciQueueID, HCI_LE_LONG_TERM_KEY_REQUEST_REPLY, mbuf, pos);
}

void hciCommandLEEncrypt(uint8_t * key, uint8_t * plaintextData)
{
    uint8_t mbuf[HCI_MAX_SHORT_CMD_SIZE];
    uint16_t pos = 0;

    memcpy(mbuf+pos, key, LE_ENCRYPT_KEY_LENGTH); pos += LE_ENCRYPT_KEY_LENGTH;
    memcpy(mbuf+pos, plaintextData, LE_ENCRYPT_DATA_LENGTH); pos += LE_ENCRYPT_DATA_LENGTH;

    hciSendDsCommand(pHCI->hciQueueID, HCI_LE_ENCRYPT, mbuf, pos);
}

void hciCommandLESetAdvertisingParameters(uint16_t advertisingIntervalMin, uint16_t advertisingIntervalMax,
	                                      uint8_t advertisingType, uint8_t ownAddressType, uint8_t directAddressType,
	                                      uint8_t * directAddress, uint8_t advertisingChannelMap,
	                                      uint8_t advertisingFilterPolicy)
{
    uint8_t mbuf[HCI_MAX_SHORT_CMD_SIZE];
    uint16_t pos = 0;

    SHORT2CHAR(mbuf+pos, advertisingIntervalMin); pos += 2;
    SHORT2CHAR(mbuf+pos, advertisingIntervalMax); pos += 2;
    mbuf[pos++] = advertisingType;
    mbuf[pos++] = ownAddressType & LE_ADDRESS_TYPE_RANDOM;
    mbuf[pos++] = directAddressType & LE_ADDRESS_TYPE_RANDOM;
    memcpy(mbuf+pos, directAddress, BD_ADDR_SIZE); pos += BD_ADDR_SIZE;
    mbuf[pos++] = advertisingChannelMap;
    mbuf[pos++] = advertisingFilterPolicy;

    hciSendDsCommand(pHCI->hciQueueID, HCI_LE_SET_ADVERTISING_PARAMETERS, mbuf, pos);
}

void hciCommandLESetScanEnable(uint8_t scanEnable, uint8_t filterDuplicates)
{
    uint8_t mbuf[HCI_MAX_SHORT_CMD_SIZE];
    uint16_t pos = 0;

    mbuf[pos++] = scanEnable;
    mbuf[pos++] = filterDuplicates;

    hciSendDsCommand(pHCI->hciQueueID, HCI_LE_SET_SCAN_ENABLE, mbuf, pos);
}

void hciCommandLESetScanParameters(uint8_t scanType, uint16_t scanInterval, uint16_t scanWindow,
	                               uint8_t ownAddressType, uint8_t scanningFilterPolicy)
{
    uint8_t mbuf[HCI_MAX_SHORT_CMD_SIZE];
    uint16_t pos = 0;

    mbuf[pos++] = scanType;
    SHORT2CHAR(mbuf+pos, scanInterval); pos += 2;
    SHORT2CHAR(mbuf+pos, scanWindow); pos += 2;
    mbuf[pos++] = ownAddressType & LE_ADDRESS_TYPE_RANDOM;
    mbuf[pos++] = scanningFilterPolicy;

    hciSendDsCommand(pHCI->hciQueueID, HCI_LE_SET_SCAN_PARAMETERS, mbuf, pos);
}

void hciCommandLECreateConnection(uint16_t scanInterval, uint16_t scanWindow,
	                              uint8_t initiatorFilterPolicy, uint8_t peerAddressType, TBdAddr peerAddress,
	                              uint8_t ownAddressType, uint16_t connIntervalMin, uint16_t connIntervalMax,
	                              uint16_t connLatency, uint16_t supervisionTimeout, 
	                              uint16_t minimumCELength, uint16_t maximumCELength)
{
    uint8_t mbuf[HCI_MAX_SHORT_CMD_SIZE];
    uint16_t pos = 0;

    SHORT2CHAR(mbuf+pos, scanInterval); pos += 2;
    SHORT2CHAR(mbuf+pos, scanWindow); pos += 2;
    mbuf[pos++] = initiatorFilterPolicy;
    mbuf[pos++] = peerAddressType & LE_ADDRESS_TYPE_RANDOM;
    memcpy(mbuf+pos, peerAddress, BD_ADDR_SIZE); pos += BD_ADDR_SIZE;
    mbuf[pos++] = ownAddressType & LE_ADDRESS_TYPE_RANDOM;;
    SHORT2CHAR(mbuf+pos, connIntervalMin); pos += 2;
    SHORT2CHAR(mbuf+pos, connIntervalMax); pos += 2;
    SHORT2CHAR(mbuf+pos, connLatency); pos += 2;
    SHORT2CHAR(mbuf+pos, supervisionTimeout); pos += 2;
    SHORT2CHAR(mbuf+pos, minimumCELength); pos += 2;
    SHORT2CHAR(mbuf+pos, maximumCELength); pos += 2;

    hciSendDsCommand(pHCI->hciTransQueueID, HCI_LE_CREATE_CONNECTION, mbuf, pos);
}


void hciCommandLEConnectionUpdate(uint16_t handle, uint16_t connIntervalMin, uint16_t connIntervalMax,
	                              uint16_t connLatency, uint16_t supervisionTimeout,
	                              uint16_t minimumCELength, uint16_t maximumCELength)
{
    uint8_t mbuf[HCI_MAX_SHORT_CMD_SIZE];
    uint16_t pos = 0;

    SHORT2CHAR(mbuf+pos, handle); pos += 2;
    SHORT2CHAR(mbuf+pos, connIntervalMin); pos += 2;
    SHORT2CHAR(mbuf+pos, connIntervalMax); pos += 2;
    SHORT2CHAR(mbuf+pos, connLatency); pos += 2;
    SHORT2CHAR(mbuf+pos, supervisionTimeout); pos += 2;
    SHORT2CHAR(mbuf+pos, minimumCELength); pos += 2;
    SHORT2CHAR(mbuf+pos, maximumCELength); pos += 2;

    hciSendDsCommand(pHCI->hciTransQueueID, HCI_LE_CONNECTION_UPDATE, mbuf, pos);
}

void hciCommandLESetRandomAddressCommand(uint8_t * randomAddress)
{
    uint8_t mbuf[HCI_MAX_SHORT_CMD_SIZE];
    uint16_t pos = 0;
	memcpy(mbuf + pos, randomAddress, BD_ADDR_SIZE);
    pos += BD_ADDR_SIZE;
    hciSendDsCommand(pHCI->hciQueueID, HCI_LE_SET_RANDOM_ADDRESS, mbuf, pos);
}


void hciCommandLESetEventMask(PHCI pHCI)
{
    uint8_t mbuf[HCI_MAX_SHORT_CMD_SIZE];
    uint16_t pos = 0;

    mbuf[pos++] = 0x5F;      /* all events */
    mbuf[pos++] = 0;
    mbuf[pos++] = 0;
    mbuf[pos++] = 0;
    mbuf[pos++] = 0;
    mbuf[pos++] = 0;
    mbuf[pos++] = 0;
    mbuf[pos++] = 0;

    hciSendDsCommand(pHCI->hciQueueID, HCI_LE_SET_EVENT_MASK, mbuf, pos);
}


/**
* @brief  hci LE SET advertising data
*
* @param  cmd
* @param  pData
* @param  length
*
* @return  
*
*/
void hciCommandLESetAdvertisingData(uint16_t cmd, uint8_t * pData, uint16_t length)
{
	uint8_t mbuf[HCI_MAX_SHORT_CMD_SIZE];
	uint16_t pos = 0;

	if (length > LE_ADVERTISING_DATA_LENGTH)
	{
		length = LE_ADVERTISING_DATA_LENGTH;
	}

	mbuf[pos++] = (uint8_t)length;
	memset(mbuf+pos, 0, LE_ADVERTISING_DATA_LENGTH);
	memcpy(mbuf+pos, pData, length);
	pos += LE_ADVERTISING_DATA_LENGTH;

	hciSendDsCommand(pHCI->hciQueueID, cmd, mbuf, pos);
}

/**
* @brief  hci send change device white list related command
*
* @param  cmd
* @param  pBD
* @param  addressType
*
* @return  
*
*/
void hciCommandLEChangeDeviceWhiteList(uint16_t cmd, uint8_t * pBD, uint8_t addressType)
{
	uint8_t mbuf[HCI_MAX_SHORT_CMD_SIZE];
	uint16_t pos = 0;

	mbuf[pos++] = addressType & LE_ADDRESS_TYPE_RANDOM;
	memcpy(mbuf+pos, pBD, BD_ADDR_SIZE); pos += BD_ADDR_SIZE;

	hciSendDsCommand(pHCI->hciQueueID, cmd, mbuf, pos);
}

void hciCommandLERemoteConnectionParameterRequestReply()
{

}

void hciCommandLERemoteConnectionParameterRequestNegativeReply()
{

}

void hciCommandLESetDataLength()
{

}

void hciCommandLEReadSuggestedDefaultDataLength()
{

}

void hciCommandLEWriteSuggestedDefaultDataLength()
{

}

void hciCommandLEReadLocalP256PublicKey()
{

}
    
void hciCommandLEGenerateDHKey()
{

}
void hciCommandLEAddDeviceToResolvingList()
{

}

void hciCommandLERemoveDeviceFromResolvingList()
{

}

void hciCommandLEClearResolvingList()
{
}

void hciCommandLEReadResolvingListSize()
{

}
void hciCommandLEReadPeerResolvableAddress()
{

}

void hciCommandLEReadLocalResolvableAddress()
{

}

void hciCommandLESetAddressResolutionEnable()
{

}

void hciCommandLESetResolvablePrivateAddressTimeout()
{

}

void hciCommandLEReadMaximumDataLength()
{

}

 
/****************************************************************************/
/* Handle LE_SET_HOST_CHANNEL_CLASSIFICATION                                */
/****************************************************************************/

#if 0
void hciCommandLESetHostChannelClassification(LPLEMessage pLEMsg)

{
  hciSendDsCommand(pHCI->hciQueueID, HCI_LE_SET_HOST_CHANNEL_CLASSIFICATION,
                   pLEMsg->p.SetHostChannelClassification.channelMap,
                   sizeof(pLEMsg->p.SetHostChannelClassification.channelMap));
}
#endif
/****************************************************************************/
/* Handle LE_READ_REMOTE_USED_FEATURES                                      */
/****************************************************************************/

#if 0
void hciCommandLEReadRemoteUsedFeatures(LPLEMessage pLEMsg)
{
  uint8_t mbuf[HCI_MAX_SHORT_CMD_SIZE];
  uint16_t pos = 0;

  SHORT2CHAR(mbuf+pos, pLEMsg->handle);

  hciSendDsCommand(pHCI->hciTransQueueID, HCI_LE_READ_REMOTE_USED_FEATURES, mbuf, pos);
}
#endif

 
/****************************************************************************/
/* Handle LE_TRANSMITTER_TEST                                               */
/****************************************************************************/
#if 0
void hciCommandLETransmitterTest(LPLEMessage pLEMsg)
{
  uint8_t mbuf[HCI_MAX_SHORT_CMD_SIZE];
  uint16_t pos = 0;

  mbuf[pos++] = pLEMsg->p.TransmitterTest.txFrequency;
  mbuf[pos++] = pLEMsg->p.TransmitterTest.lengthOfTestData;
  mbuf[pos++] = pLEMsg->p.TransmitterTest.packetPayload;

  hciSendDsCommand(pHCI->hciQueueID, HCI_LE_TRANSMITTER_TEST, mbuf, pos);
}
#endif


void hciLESendMessage(PHCI pHCI, WORD QueueID, WORD Command, LPLEMessage pLEMsg, WORD Length)
{
    MESSAGE_T Message;

    Message.Command        = Command; 
    Message.MData.DataCB.BufferAddress = (LPBYTE)pLEMsg;
    Message.MData.DataCB.Offset        = 0;
    Message.MData.DataCB.Flag          = DATA_CB_RELEASE;
    Message.MData.DataCB.Length        = Length;
    osMessageSend(QueueID, &Message);
}



void hciLESendMessageConf(PHCI pHCI, WORD MsgType, WORD Status)
{
    LPLEMessage pLEMsg;

    pLEMsg = hciLEGetBufferUs(MsgType, 0, Status, 0);
    if (pLEMsg != (LPLEMessage)0)
    {
        hciLESendMessage(pHCI, l2cQueueID, LE_MESSAGE_CONF, pLEMsg, offsetof(TLEMessage, p));
    }
}

void hciLESendMessageConfTransactionHandle(PHCI pHCI, WORD Conf, WORD Status)
{
    LPLEMessage pLEMsg;

    pLEMsg = hciLEGetBufferUs(Conf, pHCI->transactionHandle, Status, 0);
    if (pLEMsg != (LPLEMessage)0)
    {
        hciLESendMessage(pHCI, l2cQueueID, LE_MESSAGE_CONF, pLEMsg, offsetof(TLEMessage, p));
    }
}

void hciLEHandleDisconnectionComplete(uint16_t Handle, uint16_t Reason)
{
	/* map "normal" outgoing/incoming disconnect causes to success */
	if ((Reason == (HCI_ERR | HCI_ERR_CONNECTION_TERMINATE_LOCALLY)) ||
	  (Reason == (HCI_ERR | HCI_ERR_OTHER_END_TERMINATE_13))
	 )
	{
		Reason = HCI_SUCCESS;
	}
   
	/*l2cap*/
	l2cLEHandleHCI_DISCONNECTION_COMPLETE(Handle);
	/*msg sec*/
	btsmHandleLE_HCI_DISCONNECTION_COMPLETE(Handle);
	/*gatt*/
    gattHandleLE_HCI_DISCONNECTION_COMPLETE(Handle, Reason);
	
	hciRemoveHandle(Handle);
}


 /**
* @brief  handle LE command complete event 
* 
* @param  fp: point to packet(without packet type, event type and length)
* @param  response: le opcode
* @param  status: command complete status
*
* @return  
*
*/
void hciLEProcessCommandComplete(uint8_t * fp, uint16_t response, uint16_t status)
{
    LPLEMessage pLEMsg;
    uint16_t    pos = 0;

	switch (response)
	{
	case HCI_LE_SET_EVENT_MASK:
		if (pHCI->status == HCI_STATUS_INITIALIZING && status == HCI_SUCCESS)
		{
			hciNextInitCommand();
		}
		break;

    case HCI_LE_READ_BUFFER_SIZE:
		if (pHCI->status == HCI_STATUS_INITIALIZING && status == HCI_SUCCESS)
		{
			uint16_t leDsAclTotal;			
			uint16_t leDsAclBlocksPerLink;	
			uint16_t i;
            uint16_t totolLink = otp_str_data.gEfuse_UpperStack_s.num_link_don+otp_str_data.gEfuse_UpperStack_s.num_link_doff;

			pHCI->leDsAclSize = CHAR2SHORT(fp+pos);   pos += 2;
			leDsAclTotal	  = fp[pos++];

			if (pHCI->leDsAclSize && leDsAclTotal)
			{
				leDsAclBlocksPerLink = leDsAclTotal / totolLink;
				leDsAclTotal %= totolLink;  /* here is the rest: increment tx rights for first links in the table */

				for (i=0; i < otp_str_data.gEfuse_UpperStack_s.num_link_don; i++)
				{
					pHCI->phciDescDon[i].leDsAclTotal = leDsAclBlocksPerLink;
					if (leDsAclTotal)
					{
						pHCI->phciDescDon[i].leDsAclTotal++;
						leDsAclTotal--;
					}
				}

				for (i=0; i < otp_str_data.gEfuse_UpperStack_s.num_link_doff; i++)
				{
					pHCI->phciDescDoff[i].leDsAclTotal = leDsAclBlocksPerLink;
					if (leDsAclTotal)
					{
						pHCI->phciDescDoff[i].leDsAclTotal++;
						leDsAclTotal--;
					}
				}
			}
//#if F_BT_BREDR_SUPPORT
			else
			{
				pHCI->leDsAclSize = pHCI->dsAclSize;
				
				for (i=0; i < otp_str_data.gEfuse_UpperStack_s.num_link_don; i++)
				{
					pHCI->phciDescDon[i].leDsAclTotal = pHCI->phciDescDon[i].dsAclTotal;
				}
				for (i=0; i < otp_str_data.gEfuse_UpperStack_s.num_link_doff; i++)
				{
					pHCI->phciDescDoff[i].leDsAclTotal = pHCI->phciDescDoff[i].dsAclTotal;
				}
			}
//#endif
			hciNextInitCommand();
		}
		break;

 
    case HCI_LE_READ_LOCAL_SUPPORTED_FEATURES:
		if (pHCI->status == HCI_STATUS_INITIALIZING && status == HCI_SUCCESS)
		{
			memcpy(pHCI->leLocalFeatures, fp+pos, LE_FEATURES_SIZE);
			hciNextInitCommand();
		}
		break;
 

    case HCI_LE_SET_ADVERTISING_PARAMETERS:
        gattHandleCmdCompLE_SET_ADVERTISING_PARAMETERS(status);
		break;

 
    case HCI_LE_READ_ADVERTISING_CHANNEL_TX_POWER:
      pLEMsg = hciLEGetBufferUs(LE_READ_ADVERTISING_CHANNEL_TX_POWER, 0, status,
                                sizeof(TLEReadAdvertisingChannelTxPowerLevel));
      if (pLEMsg != (LPLEMessage)0)
      {
        pLEMsg->p.ReadAdvertisingChannelTxPowerLevel.transmitPowerLevel = fp[pos];
        hciLESendMessage(pHCI, l2cQueueID, LE_MESSAGE_CONF, pLEMsg,
                         (offsetof(TLEMessage, p) + sizeof(TLEReadAdvertisingChannelTxPowerLevel)));
      }
      break;
 
	case HCI_LE_SET_ADVERTISING_DATA:
	case HCI_LE_SET_SCAN_RESPONSE_DATA:
        gattHandleCmdCompLE_SET_ADV_DATA(status);
		break;
    case HCI_LE_SET_ADVERTISE_ENABLE:
        gattHandleCmdCompLE_SET_ADVERTISE_ENABLE(status);
		break;

    case HCI_LE_SET_SCAN_PARAMETERS:
        gattHandleCmdCompLE_SET_SCAN_PARAMETERS(status);
		break;

    case HCI_LE_SET_SCAN_ENABLE:
        gattHandleCmdCompLE_SET_SCAN_ENABLE(status);
		break;

 	case HCI_LE_READ_WHITE_LIST_SIZE:
		pLEMsg = hciLEGetBufferUs(LE_READ_WHITE_LIST_SIZE, 0, status,
		                        sizeof(TLEReadWhiteListSize));
		if (pLEMsg != (LPLEMessage)0)
		{
			pLEMsg->p.ReadWhiteListSize.whiteListSize = fp[pos];
			hciLESendMessage(pHCI, l2cQueueID, LE_MESSAGE_CONF, pLEMsg,
			                 (offsetof(TLEMessage, p) + sizeof(TLEReadWhiteListSize)));
		}
		break;
 	case HCI_LE_CLEAR_WHITE_LIST:
    case HCI_LE_ADD_DEVICE_TO_WHITE_LIST:
    case HCI_LE_REMOVE_DEVICE_FROM_WHITE_LIST:
        gattHandleCmdCompLE_MODIFY_WHITE_LIST(status);
		break;

    case HCI_LE_CONNECTION_UPDATE:
		gattHandleLEConnectionUpdateConf(0, status);
		break;

 
    case HCI_LE_SET_HOST_CHANNEL_CLASSIFICATION:
		hciLESendMessageConf(pHCI, LE_SET_HOST_CHANNEL_CLASSIFICATION, status);
		break;
 
    case HCI_LE_READ_CHANNEL_MAP:
		pLEMsg = hciLEGetBufferUs(LE_READ_CHANNEL_MAP, 0, status,
		                        sizeof(TLEReadChannelMap));
		if (pLEMsg != (LPLEMessage)0)
		{
			memcpy(pLEMsg->p.ReadChannelMap.channelMap, fp+pos, sizeof(pLEMsg->p.ReadChannelMap.channelMap));
			hciLESendMessage(pHCI, l2cQueueID, LE_MESSAGE_CONF, pLEMsg,
			                 (offsetof(TLEMessage, p) + sizeof(TLEReadChannelMap)));
		}
		break;
 

 
    case HCI_LE_READ_REMOTE_USED_FEATURES:
		hciLESendMessageConf(pHCI, LE_READ_REMOTE_USED_FEATURES, status);
		break;
 

    case HCI_LE_ENCRYPT:
        btsmHandleHCI_LE_ENCRYPT(fp+pos);
		break;

    case HCI_LE_RAND:
        btsmHandleCmdCompLE_RAND(fp+pos);
		break;
    case HCI_LE_SET_RANDOM_ADDRESS:
		btsmHandleCmdCompHCI_LE_SET_RANDOM_ADDRESS(status);
        blueAPI_Send_SetRandomAddressRsp(status);
		break;

 
	case HCI_LE_READ_SUPPORTED_STATES:
		pLEMsg = hciLEGetBufferUs(LE_READ_SUPPORTED_STATES, 0, status,
		                        sizeof(TLEReadSupportedStates));
		if (pLEMsg != (LPLEMessage)0)
		{
			memcpy(pLEMsg->p.ReadSupportedStates.states, fp+pos, LE_STATES_SIZE);
			hciLESendMessage(pHCI, l2cQueueID, LE_MESSAGE_CONF, pLEMsg,
			                 (offsetof(TLEMessage, p) + sizeof(TLEReadSupportedStates)));
		}
		break;

	case HCI_LE_RECEIVER_TEST:
		hciLESendMessageConf(pHCI, LE_RECEIVER_TEST, status);
		break;

	case HCI_LE_TRANSMITTER_TEST:
		hciLESendMessageConf(pHCI, LE_TRANSMITTER_TEST, status);
		break;

	case HCI_LE_TEST_END:
		pLEMsg = hciLEGetBufferUs(LE_TEST_END, 0, status, sizeof(TLETestEnd));
		if (pLEMsg != (LPLEMessage)0)
		{
			pLEMsg->p.TestEnd.numberOfPackets = CHAR2SHORT(fp+pos);
			hciLESendMessage(pHCI, l2cQueueID, LE_MESSAGE_CONF, pLEMsg,
			                 (offsetof(TLEMessage, p) + sizeof(TLETestEnd)));
		}
		break;

 

    default:
		DebuggerBreak();
		break;
	}
}

/**
* @brief  handle LE command Status event 
* 
* @param  response: le opcode
* @param  status: command complete status
*
* @return  
*
*/
void hciLEProcessCommandStatus(uint16_t response, uint16_t status)
{
	switch (response)
	{
	case HCI_LE_CREATE_CONNECTION:
		gattHandleLECreateConnectionConf(0 , status);
		break;

	case HCI_LE_CONNECTION_UPDATE:
		gattHandleLEConnectionUpdateConf(pHCI->transactionHandle, status);
		break;

 
	case HCI_LE_READ_REMOTE_USED_FEATURES:
		hciLESendMessageConfTransactionHandle(pHCI, LE_READ_REMOTE_USED_FEATURES, status);
		break;
 
	  
	default:
		DebuggerBreak();
		break;
	}
}

/**
* @brief  handle le event
*
* @param  fp
* @param  pos
* @param  len
*
* @return  
*
*/
void hciLEProcessEventPacket(uint8_t * fp, uint16_t pos, uint16_t len)
{
	LPLEMessage pLEMsg;
	uint16_t           Status;
	uint16_t           Handle;
	uint8_t           Subevent;

	Subevent = fp[pos++];
	switch (Subevent)
	{
	case HCI_LE_CONNECTION_COMPLETE_EVENT:
		{
			ThciHandleDesc *hd = (ThciHandleDesc *)0;
			uint8_t            role;
			uint8_t            peerAddressType;
			uint8_t *          peerAddress;

			pHCI->transaction = 0;

			Status = hciStatus(fp[pos++]);
			Handle = CHAR2SHORT(fp + pos); pos += 2;
			role            = fp[pos++];
			peerAddressType = fp[pos++] | BLUEFACE_BDTYPE_LE_MASK;
			peerAddress     = fp + pos; pos += BD_ADDR_SIZE;

			if (Status == 0)
			{
				hd = hciNewHandle(peerAddress, Handle, BT_CONNECTION_TYPE_LE, peerAddressType);
				if (hd == NULL)
				{
					Status = HCI_ERR_NO_DESCRIPTOR;
					hciCommandDisconnect(Handle, HCI_ERR_OTHER_END_TERMINATE_13);
					break;
				}
				else
				{
					hd->currentRole = role;
				}
			}

            l2cLEHandleHCI_LE_CONNECTION_COMPLETE_EVENT(Handle, role, Status, peerAddress);

            pLEMsg = hciLEGetBufferUs(LE_CONNECTION_COMPLETE_EVENT, Handle, Status,
                                sizeof(TLEConnectionComplete));
            if (pLEMsg != (LPLEMessage)0)
            {
                pLEMsg->p.ConnectionComplete.role                = role;
                pLEMsg->p.ConnectionComplete.peerAddressType     = peerAddressType;
                memcpy(pLEMsg->p.ConnectionComplete.peerAddress, peerAddress, BD_ADDR_SIZE);
                pLEMsg->p.ConnectionComplete.connInterval        = CHAR2SHORT(fp+pos); pos += 2;
                pLEMsg->p.ConnectionComplete.connLatency         = CHAR2SHORT(fp+pos); pos += 2;
                pLEMsg->p.ConnectionComplete.supervisionTimeout  = CHAR2SHORT(fp+pos); pos += 2;
                pLEMsg->p.ConnectionComplete.masterClockAccuracy = fp[pos++];
                btsmSendLEMsg(LE_MESSAGE_IND, pLEMsg,
                                 (offsetof(TLEMessage, p) + sizeof(TLEConnectionComplete)));
            }
		}
		break;

    case HCI_LE_ADVERTISING_REPORT_EVENT:
        {
            uint8_t numReports = fp[pos++];
            uint8_t dataLength ;
            uint8_t loop;

            HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_TRACE,
                               "HCI_LE: numReport(%d)",
                               numReports
                              );

            for (loop = 0; loop < numReports; loop++)
            {
                pLEMsg = (LPLEMessage)0;
                dataLength = fp[pos + 8];
                pLEMsg = hciLEGetBufferUs(LE_ADVERTISING_REPORT_EVENT, 0, 0, (sizeof(TLEAdvertisingReport) + dataLength));
                if (pLEMsg != (LPLEMessage)0)
                {
                    pLEMsg->p.AdvertisingReport.eventType   = fp[pos++];
                    pLEMsg->p.AdvertisingReport.addressType = fp[pos++] | BLUEFACE_BDTYPE_LE_MASK;
                    memcpy(pLEMsg->p.AdvertisingReport.address, fp + pos, BD_ADDR_SIZE);
                    pos += BD_ADDR_SIZE;
                    pLEMsg->p.AdvertisingReport.dataLength  = fp[pos++];
                    memcpy(pLEMsg->p.AdvertisingReport.data, fp + pos, dataLength);
                    pos += dataLength;
                    pLEMsg->p.AdvertisingReport.rssi        = fp[pos++];

                    btsmSendLEMsg(LE_MESSAGE_IND, pLEMsg,
                               (offsetof(TLEMessage, p) +
                                sizeof(TLEAdvertisingReport) +
                                dataLength));
                }
                else
                {
                    break;
                }
            }
        }
     break;

	case HCI_LE_CONNECTION_UPDATE_COMPLETE_EVENT:
        {
            uint16_t    connInterval;
    	    uint16_t    connLatency;
    	    uint16_t    supervisionTimeout;
    		pHCI->transaction = 0;

    		Status = hciStatus(fp[pos++]);
    		Handle = CHAR2SHORT(fp + pos); pos += 2;
            connInterval = CHAR2SHORT(fp+pos); pos += 2;
    	    connLatency = CHAR2SHORT(fp+pos); pos += 2;
    	    supervisionTimeout = CHAR2SHORT(fp+pos); pos += 2;

            gattHandleHCI_LE_CONNECTION_UPDATE_COMPLETE_EVENT(Handle, Status, connInterval, connLatency, supervisionTimeout);
	    }
		break;

 
	case HCI_LE_READ_REMOTE_USED_FEATURES_COMPLETE_EVENT:
		pHCI->transaction = 0;

		Status = hciStatus(fp[pos++]);
		Handle = CHAR2SHORT(fp + pos); pos += 2;

		pLEMsg = hciLEGetBufferUs(LE_READ_REMOTE_USED_FEATURES_COMPLETE_EVENT, Handle, Status,
		                        sizeof(TLEReadRemoteUsedFeaturesComplete));
		if (pLEMsg != (LPLEMessage)0)
		{
			memcpy(pLEMsg->p.ReadRemoteUsedFeaturesComplete.features, (fp+pos), LE_FEATURES_SIZE);
			hciLESendMessage(pHCI, l2cQueueID, LE_MESSAGE_IND, pLEMsg,
			                 (offsetof(TLEMessage, p) + sizeof(TLEReadRemoteUsedFeaturesComplete)));
		}
		break;
 

	case HCI_LE_LONG_TERM_KEY_REQUEST_EVENT:
		{
			uint8_t	randomNumber[LE_RANDOM_NUMBER_SIZE];
			uint16_t	encryptedDiversifier;

			Handle = CHAR2SHORT(fp + pos); pos += 2;
			memcpy(randomNumber, (fp+pos), LE_RANDOM_NUMBER_SIZE); pos += LE_RANDOM_NUMBER_SIZE;
			encryptedDiversifier = CHAR2SHORT(fp + pos);

			btsmHandleHci_LE_LONG_TERM_KEY_REQUEST_EVENT(encryptedDiversifier, randomNumber, Handle);
		}
		break;
        
    case HCI_LE_REMOTE_CONNECTION_PARAMETER_REQUEST_EVENT:
    case HCI_LE_DATA_LENGTH_CHANGE_EVENT:
    case HCI_LE_READ_LOCAL_P256_PUBLIC_KEY_COMPLETE:
    case HCI_LE_GENERATE_DHKEY_COMPLETE:
    case HCI_LE_ENHANCED_CONNECTION_COMPLETE:
    case HCI_LE_DIRECT_ADVERTISING_REPORT:
        break;
    
	default:
		DebuggerBreak();
		break;
	}
}

