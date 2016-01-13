
#include "avdtp.h"
#include <bterrcod.h>
#include "a2dp.h"
#include "blueapi.h"
#include "mpa.h"
#include "btcommon.h"
#include "os_timer.h"
#include "os_mem.h"

extern PAppAvdtp pAppAvdtp;
extern int A2dpIndex;
extern PA2dpProfile a2dpProfile[MAX_INDEX];

static bool avdtp_SendSignal(PAppAvdtp pAvdtp, uint8_t Signal, uint8_t MessageType, unsigned char* buf, uint8_t Length, uint8_t cid);
int a2dp_getindex(TBdAddr *bd);
int a2dp_allocateindex(int num);
//Signal Response Payload Format Error Codes
bool avdtp_SendBadLength(TAvdtpLink* pLink, TAvdtpEventSignalID signalID, uint8_t transactId, uint16_t channel)
{
	TAvdtpUpStreamSignal signal;
	signal.data[0] = AVDTP_ERROR_BAD_LENGTH;

	return avdtp_SendUpstreamSignal(
			pLink,
			signalID,
			AVDTP_MESSAGE_TYPE_RESPONSE_REJECT,
			transactId,
			&signal,
			1, channel);
}

bool avdtp_SendBadAcpId(TAvdtpLink* pLink, TAvdtpEventSignalID signalID, uint8_t transactId, uint16_t channel)
{
	TAvdtpUpStreamSignal signal;
	signal.data[0] = AVDTP_ERROR_BAD_ACP_SEID;

	return avdtp_SendUpstreamSignal(
			pLink,
			signalID,
			AVDTP_MESSAGE_TYPE_RESPONSE_REJECT,
			transactId,
			&signal,
			1, channel);
}

bool avdtp_SendBadInUse(TAvdtpLink* pLink, TAvdtpEventSignalID signalID, uint8_t transactId, uint16_t channel)
{
	TAvdtpUpStreamSignal signal;
	signal.data[0] = AVDTP_MEDIA_TRANSPORT; 
	signal.data[1] = AVDTP_ERROR_SEP_IN_USE;

	return avdtp_SendUpstreamSignal(
			pLink,
			signalID,
			AVDTP_MESSAGE_TYPE_RESPONSE_REJECT,
			transactId,
			&signal,
			2, channel);
}

bool avdtp_SendBadNotInUse(TAvdtpLink* pLink, TAvdtpEventSignalID signalID, uint8_t transactId, uint16_t channel)
{
	TAvdtpUpStreamSignal signal;
	signal.data[0] = AVDTP_MEDIA_TRANSPORT; 
	signal.data[1] = AVDTP_ERROR_SEP_NOT_IN_USE;

	return avdtp_SendUpstreamSignal(
			pLink,
			signalID,
			AVDTP_MESSAGE_TYPE_RESPONSE_REJECT,
			transactId,
			&signal,
			2, channel);
}

bool avdtp_SendBadService(TAvdtpLink* pLink, TAvdtpEventSignalID signalID, uint8_t transactId, uint16_t channel)
{
	TAvdtpUpStreamSignal signal;
	signal.data[0] = AVDTP_MEDIA_TRANSPORT; 
	signal.data[1] = AVDTP_ERROR_BAD_SERV_CATEGORY;

	return avdtp_SendUpstreamSignal(
			pLink,
			signalID,
			AVDTP_MESSAGE_TYPE_RESPONSE_REJECT,
			transactId,
			&signal,
			2, channel);
}

bool avdtp_SendNotSupportedCommand(TAvdtpLink* pLink, TAvdtpEventSignalID signalID, uint8_t transactId, uint16_t channel)
{
	TAvdtpUpStreamSignal signal;
	signal.data[0] = AVDTP_ERROR_NOT_SUPPORTED_COMMAND;

	return avdtp_SendUpstreamSignal(
			pLink,
			signalID,
			AVDTP_MESSAGE_TYPE_RESPONSE_REJECT,
			transactId,
			&signal,
			1, channel);
}
//Transport Service Capabilities Error Codes
bool avdtp_SendBadMediaTransportFormat(TAvdtpLink* pLink, TAvdtpEventSignalID signalID, uint8_t transactId, uint16_t channel)
{
	TAvdtpUpStreamSignal signal;	
	signal.data[0] = AVDTP_MEDIA_TRANSPORT; 
	signal.data[1] = AVDTP_ERROR_BAD_MEDIA_TRANSPORT_FORMAT;

	return avdtp_SendUpstreamSignal(
			pLink,
			signalID,
			AVDTP_MESSAGE_TYPE_RESPONSE_REJECT,
			transactId,
			&signal,
			2, channel);
}


PAvdtpSepData avdtp_SEPFindSeidAndState(PAvdtpChannel pChannel,
                                       uint8_t Seid, TAvdtpSepState State)
{
	PAvdtpSepData pSepData;

	pSepData = (PAvdtpSepData)pChannel->SepQueue.First;

	while (pSepData != (PAvdtpSepData)0)
	{
		if (pSepData->LocalSeid == Seid &&
		    pSepData->State   == State)
		{
			break;
		}
		pSepData = (PAvdtpSepData)pSepData->Next;
	}

	return(pSepData);
}

PAvdtpChannel avdtp_ChannelFindSignalling(PAppAvdtp pAvdtp)
{
	int           i;
	PAvdtpChannel  pChannel;

	pAvdtp->pChannel = (PAvdtpChannel)0;

	for (i = 0; i < AVDTP_MAX_CHANNELS_COUNT; i++)
	{
		pChannel = &pAvdtp->ChannelArray[i];

		if ( pChannel->Used )
		{
			if (pChannel->Type == AVDTP_CHANNEL_TYPE_SIGNALLING)
			{
				pAvdtp->pChannel = pChannel;
				break;
			}
		}
	}

	return( pAvdtp->pChannel );
}

/**
* @brief    Find SEP  
*
* @param  pAvdtp
* @param  pRemoteBd: remote bdaddr
* @param  State: SEP state
*
* @return  
*
*/
PAvdtpSepData avdtp_SEPFind(PAvdtpChannel pChannel, TAvdtpSepState State)
{
	PAvdtpSepData pSepData;

	pSepData = (PAvdtpSepData)pChannel->SepQueue.First;

	while (pSepData != (PAvdtpSepData)0)
	{
		if (pSepData->State == State)
		{
			break;
		}
		pSepData = (PAvdtpSepData)pSepData->Next;
	}

	if (pSepData == (PAvdtpSepData)0)
	{
		//avdtp sep found fail...
	}

	return(pSepData);
}


/**
* @brief   Find signalling channel by SEP (State)  
*
* @param  pAvdtp
* @param  pRemoteBd: remote bdaddr
* @param  State: SEP state
*
* @return  
*
*/
PAvdtpChannel avdtp_ChannelFindBySEPState(PAppAvdtp pAvdtp, TBdAddr *pRemoteBd,
                                                        TAvdtpSepState State)
{
	PAvdtpChannel  pChannel;

	if ((pChannel = avdtp_ChannelFind(pAvdtp,(TBdAddr *)pRemoteBd,0)) != (PAvdtpChannel)0)
	{
		if (memcmp(pRemoteBd, pChannel->RemoteBd, BD_ADDR_SIZE) == 0)
		{
			if (avdtp_SEPFind(pAvdtp->pChannel, State) == (PAvdtpSepData)0)
			{
				/**no config sep*/
				pChannel = (PAvdtpChannel)0;
			}
		}
		else
		{
			/**other remote device*/
			pChannel = (PAvdtpChannel)0;
		}
	}
	else
	{
		/*no signal channel*/
		pChannel = (PAvdtpChannel)0;
	}

	return(pChannel);
}


/**
* @brief  allocate avdtp channel
* 
* @param pAvdtp
*
* @return 
*
*/
PAvdtpChannel avdtp_ChannelAllocate(PAppAvdtp pAvdtp )
{
	int           i;
	PAvdtpChannel  pChannel;

	pAvdtp->pChannel = (PAvdtpChannel)0;

	for (i = 0; i < AVDTP_MAX_CHANNELS_COUNT; i++)
	{
		pChannel = &pAvdtp->ChannelArray[i];

		if (pChannel->Used == FALSE)
		{
			memset( &pChannel->Used, 0,       /* preserve iNumber */
			      sizeof(TAvdtpChannel) - offsetof(TAvdtpChannel, Used) );
			pChannel->Used  = TRUE;
			pChannel->State = AVDTP_STATE_CLOSED;
			pAvdtp->pChannel = pChannel;

			break;
		}
	}

	if ( i >= AVDTP_MAX_CHANNELS_COUNT )
	{
		//allocate err
	}

	return( pAvdtp->pChannel );
}

PAvdtpChannel avdtp_ChannelFind(PAppAvdtp pAvdtp, TBdAddr *pRemoteBd, uint16_t cid)
{
	int           i;
	PAvdtpChannel  pChannel;

	pAvdtp->pChannel = (PAvdtpChannel)0;

	for (i = 0; i < AVDTP_MAX_CHANNELS_COUNT; i++)
	{
		pChannel = &pAvdtp->ChannelArray[i];

		if ( pChannel->Used )
		{
			if ( ((pRemoteBd != (TBdAddr *)0) &&
			    (memcmp(pChannel->RemoteBd, pRemoteBd, BD_ADDR_SIZE) == 0)) ||
			   ((pRemoteBd == (TBdAddr *)0) && (pChannel->cid == cid))
			 )
			{
				pAvdtp->pChannel = pChannel;
				break;
			}
		}
	}

	if ( i >= AVDTP_MAX_CHANNELS_COUNT )
	{
		//avdtp channel find fail....
	}

	return( pAvdtp->pChannel );
}

PAvdtpChannel avdtp_ChannelFindStream(PAppAvdtp pAvdtp, TBdAddr *pRemoteBd)
{
  int           i;
	PAvdtpChannel  pChannel;

	pAvdtp->pChannel = (PAvdtpChannel)0;

	for (i = 0; i < AVDTP_MAX_CHANNELS_COUNT; i++)
	{
		pChannel = &pAvdtp->ChannelArray[i];

		if ( pChannel->Used )
		{
			if ( ((pRemoteBd != (TBdAddr *)0) &&
			    (memcmp(pChannel->RemoteBd, pRemoteBd, BD_ADDR_SIZE) == 0)))
			{
				pAvdtp->pChannel = &pAvdtp->ChannelArray[i+1];
				break;
			}
		}
	}

	if ( i >= AVDTP_MAX_CHANNELS_COUNT )
	{
		//avdtp channel find fail....
	}

	return( pAvdtp->pChannel );	
}

static int GetStreamIndexFromRemoteId(TAvdtpLink* pLink, uint8_t remoteEndPointId)
{
	int i;
	for (i = 0; i < pLink->streamCount; i++)
	{
		if (pLink->streams[i].remoteEndPointId == remoteEndPointId)
		{
			return i;
		}
	}
	return INVALID_STREAM_INDEX;
}

bool avdtp_Discover(TAvdtpLink* pLink, uint16_t channel)
{
	TAvdtpUpStreamSignal signal;
	
	return avdtp_SendUpstreamSignal(
		pLink,
		AVDTP_DISCOVER,
		AVDTP_MESSAGE_TYPE_COMMAND,
		++pLink->transactId,
		&signal,
		0, channel);
}

bool avdtp_SendDiscoverResp(TAvdtpLink* pLink, uint8_t transactId, uint16_t channel)
{
	TAvdtpUpStreamSignal signal;
	int i;
	for (i = 0; i < MAX_INDEX; i++)
	{
		signal.data[(2*i)]      = (uint8_t)((a2dpProfile[i]->pA2DPData->link.streams[0].localEndPointId << 2) + (a2dpProfile[i]->pA2DPData->link.streams[0].used<<1));   //(0<<1 /* not in use */));
		signal.data[(2*i) + 1]  = (uint8_t)((a2dpProfile[i]->pA2DPData->link.streams[0].endPointType << 3) + (a2dpProfile[i]->pA2DPData->link.streams[0].mediaType << 4));
	}
	return avdtp_SendUpstreamSignal(
				pLink,
				AVDTP_DISCOVER,
				AVDTP_MESSAGE_TYPE_RESPONSE_ACCEPT,
				transactId,
				&signal,
				MAX_INDEX* 2, channel);

}

bool avdtp_GetCapabilities(TAvdtpLink* pLink, int streamIndex, uint16_t channel)
{
  TAvdtpUpStreamSignal signal;
  signal.data[0] = (pLink->streams[streamIndex].remoteEndPointId) << 2;
  return avdtp_SendUpstreamSignal(
    pLink,
    AVDTP_GET_CAPABILITIES,
    AVDTP_MESSAGE_TYPE_COMMAND,
    ++pLink->transactId,
    &signal,
    1, channel);
}

bool avdtp_SendGetCapsResp(TAvdtpLink* pLink, uint8_t transactId, uint16_t length, uint8_t * capsData, uint16_t channel)
{
	TAvdtpUpStreamSignal signal;
	if (length > UPSTREAM_SIGNAL_DATA_SIZE)
	{
//		debugOut(dbTrace, MPA_AVDTP_API_DEBUG_HEADER"avdtpSendGetCapsResp !!buffer too big");
		return false;
	}

	memcpy(&signal.data[0], capsData, length);
	return avdtp_SendUpstreamSignal(
				pLink,
				AVDTP_GET_CAPABILITIES,
				AVDTP_MESSAGE_TYPE_RESPONSE_ACCEPT,
				transactId,
				&signal,
				length, channel);
}

bool avdtp_SetConfiguration(TAvdtpLink* pLink, int streamIndex, uint16_t length, uint8_t * capsData, bool reconfigure, uint16_t channel)
{
	TAvdtpUpStreamSignal signal;
	int headerbytes = reconfigure?1:2; // reconfigure command doesn't have uint8_t SEID
	uint8_t command    = reconfigure?AVDTP_RECONFIGURE:AVDTP_SET_CONFIGURATION;

	length += headerbytes; /* add too bytes for header */
	if (length > UPSTREAM_SIGNAL_DATA_SIZE)
	{
//		debugOut(dbTrace, MPA_AVDTP_API_DEBUG_HEADER"avdtpSetConfiguration !!buffer too big");
		return false;
	}

	signal.data[0] = (pLink->streams[streamIndex].remoteEndPointId) << 2;
	if (!reconfigure)
	{
		signal.data[1] = pLink->streams[streamIndex].localEndPointId << 2;
	}
	if(reconfigure==TRUE)
	{
		length-=2;
		memcpy(&signal.data[headerbytes], capsData+2, length);
	}
	else
		memcpy(&signal.data[headerbytes], capsData, length);

	return avdtp_SendUpstreamSignal(
		pLink,
		command,
		AVDTP_MESSAGE_TYPE_COMMAND,
		++pLink->transactId,
		&signal,
		length, channel);
}

bool avdtp_SendSetConfigResp(TAvdtpLink* pLink, uint8_t transactId, uint16_t channel)
{
	TAvdtpUpStreamSignal signal;

	return avdtp_SendUpstreamSignal(
			pLink,
			AVDTP_SET_CONFIGURATION,
			AVDTP_MESSAGE_TYPE_RESPONSE_ACCEPT,
			transactId,
			&signal,
			0, channel);
}

bool avdtp_SendSetConfigReject(TAvdtpLink* pLink, uint8_t transactId, uint16_t channel)
{
	TAvdtpUpStreamSignal signal;
	signal.data[0] = AVDTP_MEDIA_TRANSPORT; /* Correct ?? */
	signal.data[1] = AVDTP_ERROR_BAD_STATE;

	return avdtp_SendUpstreamSignal(
			pLink,
			AVDTP_SET_CONFIGURATION,
			AVDTP_MESSAGE_TYPE_RESPONSE_REJECT,
			transactId,
			&signal,
			2, channel);
}

/************************** avdtpGetConfiguration ********************/
/*********************************************************************/
bool avdtp_GetConfiguration(TAvdtpLink* pLink, int streamIndex , uint16_t channel)
{
  TAvdtpUpStreamSignal signal;
  signal.signalID = AVDTP_GET_CONFIGURATION;
  signal.data[0] = (pLink->streams[streamIndex].remoteEndPointId) << 2;
  return avdtp_SendUpstreamSignal(
    pLink,
    AVDTP_GET_CONFIGURATION,
    AVDTP_MESSAGE_TYPE_COMMAND,
    ++pLink->transactId,
    &signal,
    1,channel);
}
/**
* @brief  avdtp send get config response
* 
* @param  pLink
* @param  transactId
* @param  channel
*
* @return  
*
*/
bool avdtp_SendGetConfigResp(TAvdtpLink* pLink, uint8_t transactId, uint16_t channel)
{
	TAvdtpUpStreamSignal signal;

	return avdtp_SendUpstreamSignal(
			pLink,
			AVDTP_GET_CONFIGURATION,
			AVDTP_MESSAGE_TYPE_RESPONSE_ACCEPT,
			transactId,
			&signal,
			0,channel);
}

/**
* @brief  avdtp send reconfigure response
* 
* @param  pLink
* @param  transactId
* @param  channel
*
* @return  
*
*/
bool avdtp_SendReconfigureResp(TAvdtpLink* pLink, uint8_t transactId, uint16_t channel)
{
	TAvdtpUpStreamSignal signal;


	return avdtp_SendUpstreamSignal(
		pLink,
		AVDTP_RECONFIGURE,
		AVDTP_MESSAGE_TYPE_RESPONSE_ACCEPT,
		transactId,
		&signal,
		0, channel);
}
/**
* @brief  avdtp send reconfigure reject
* 
* @param  pLink
* @param  transactId
* @param  channel
*
* @return  
*
*/
bool avdtp_SendReconfigureReject(TAvdtpLink* pLink, uint8_t transactId, uint16_t channel)
{
	TAvdtpUpStreamSignal signal;

	signal.data[0] = AVDTP_MEDIA_TRANSPORT; /* Correct ?? */
	signal.data[1] = AVDTP_ERROR_BAD_STATE;

	return avdtp_SendUpstreamSignal(
		pLink,
		AVDTP_RECONFIGURE,
		AVDTP_MESSAGE_TYPE_RESPONSE_REJECT,
		transactId,
		&signal,
		2, channel);
}
bool avdtp_SuspendStream(TAvdtpLink* pLink, int streamIndex , uint16_t channel)
{
  TAvdtpUpStreamSignal signal;
  signal.data[0] = (pLink->streams[streamIndex].remoteEndPointId) << 2;
  return avdtp_SendUpstreamSignal(
    pLink,
    AVDTP_SUSPEND,
    AVDTP_MESSAGE_TYPE_COMMAND,
    ++pLink->transactId,
    &signal,
    1,channel);
}
/**
* @brief  avdtp send suspend stream response
* 
* @param  pLink
* @param  transactId
* @param  channel
*
* @return  
*
*/
bool avdtp_SendSuspendStreamResp(TAvdtpLink* pLink, uint8_t transactId, uint16_t channel)
{
	TAvdtpUpStreamSignal signal;

	return avdtp_SendUpstreamSignal(
			pLink,
			AVDTP_SUSPEND,
			AVDTP_MESSAGE_TYPE_RESPONSE_ACCEPT,
			transactId,
			&signal,
			0, channel);
}

/**
* @brief  avdtp send suspend stream reject
* 
* @param  pLink
* @param  transactId
* @param  streamIndex
* @param  channel
*
* @return  
*
*/
bool avdtp_SendSuspendStreamReject(TAvdtpLink* pLink, uint8_t transactId, int streamIndex, uint16_t channel)
{
	TAvdtpUpStreamSignal signal;
	signal.data[0] = (pLink->streams[streamIndex].remoteEndPointId) << 2;
	signal.data[1] = AVDTP_ERROR_BAD_STATE;


	return avdtp_SendUpstreamSignal(
			pLink,
			AVDTP_SUSPEND,
			AVDTP_MESSAGE_TYPE_RESPONSE_REJECT,
			transactId,
			&signal,
			2, channel);
}

STATIC void avdtp_SaveSignallingHeader(PAppAvdtp pAvdtp, unsigned char* pData)
{
	pAvdtp->pChannel->SignalHeader[0] = pData[0];
	pAvdtp->pChannel->SignalHeader[1] = pData[1];
	pAvdtp->pChannel->SignalHeader[2] = pData[2];
}

uint16_t avdtp_SendSignalAVDTP_OPEN_CLOSE(PAppAvdtp pAvdtp, PAvdtpSepData pSepData,
                                     uint8_t Signal, uint8_t MessageType, uint16_t cid)
{
	uint8_t *		   mybuf;
	uint16_t op = pAvdtp->writeOffset;
//	MESSAGE_T msg;

	if (blueAPI_BufferGet(pAvdtp->dsPoolID, op + AVDTP_SINGLE_PACKET_HEADER_SIZE +
	                     sizeof(pSepData->LocalSeid), 24/*dsoffset count*/, (void FAR *)&mybuf))
	{
	//		BT_TRACE_VERB1(TRACE_PRuint8_tF_0(dbError,"!!lblueFacePutMessage : cannot allocate buffer for data_req"));
	//	bfStatus = blueFaceIllParameter;
		return false;
	}
			
	mybuf[op] = AVDTP_PACKET_TYPE_SINGLE | MessageType;
	if (MessageType != AVDTP_MESSAGE_TYPE_COMMAND)
	{
		mybuf[op] |= pAvdtp->TransactionLabel;
	}
	else
	{	
		mybuf[op] |=(++a2dpProfile[A2dpIndex]->pA2DPData->link.transactId) << 4;
	}
	op++;
	mybuf[op++] = Signal;;

	if (MessageType == AVDTP_MESSAGE_TYPE_COMMAND)
	{
		mybuf[op++] = pSepData->RemoteSeid << 2;
	}
//change it to mpa layer	blueAPI_L2cDataReq(mybuf, pAvdtp->writeOffset, cid, op - pAvdtp->writeOffset);
	mpa_Sendl2cDataReq(mybuf, pAvdtp->writeOffset, cid, op - pAvdtp->writeOffset);

#if 0
	BT_TRACE_VERB1(TRACE_PRuint8_tF_4(dbTrace,
		 "avdtpSendSignal: send single packet (signal=0x%x mt=0x%x length=0x%x)",
		 Signal, MessageType, mybuf, 0));
#endif
#if 0
	msg.Command 						= L2C_DATA_REQ;
	msg.MData.DataCBChan.BufferAddress	= mybuf;
	msg.MData.DataCBChan.Offset 		= pAvdtp->writeOffset;
	msg.MData.DataCBChan.Flag			= DATA_CB_RELEASE;
	msg.MData.DataCBChan.Channel		= cid;
	msg.MData.DataCBChan.Length 		= op - pAvdtp->writeOffset;
	
	gl2cHandleL2C_DATA_REQ(&msg, FALSE);
#endif

	if (MessageType == AVDTP_MESSAGE_TYPE_COMMAND)
	{
		avdtp_SaveSignallingHeader(pAvdtp, &mybuf[pAvdtp->writeOffset]);
//FIX TIMER		avdtp_TimerStart(pAvdtp, AVDTP_TIMERID_RTX_SIG_TIMER, RTX_SIG_TIMER);
	}

	return(AVDTP_SUCCESS);
}

uint16_t avdtp_SendSignalAVDTP_OPEN(PAppAvdtp pAvdtp, PAvdtpSepData pSepData,
                                              uint8_t MessageType, uint16_t channel)
{
	return(avdtp_SendSignalAVDTP_OPEN_CLOSE(pAvdtp, pSepData, AVDTP_OPEN,
                                                           MessageType, channel));
}

uint16_t avdtp_SendSignalAVDTP_CLOSE(PAppAvdtp pAvdtp, PAvdtpSepData pSepData,
                                              uint8_t MessageType, uint16_t channel)
{
	pSepData->State = AVDTP_SEP_STATE_CLOSESENT;
	return(avdtp_SendSignalAVDTP_OPEN_CLOSE(pAvdtp, pSepData, AVDTP_CLOSE,
	                                                       MessageType, channel));
}

/*********************************************************************/
bool avdtp_AbortStream(TAvdtpLink* pLink, int streamIndex, uint16_t channel)
{
  TAvdtpUpStreamSignal signal;
  signal.signalID = AVDTP_ABORT;
  signal.data[0] = (pLink->streams[streamIndex].remoteEndPointId) << 2;
  return avdtp_SendUpstreamSignal(
    pLink,
    AVDTP_ABORT,
    AVDTP_MESSAGE_TYPE_COMMAND,
    ++pLink->transactId,
    &signal,
    1,channel);
}

bool avdtp_SendAbortResponse(TAvdtpLink* pLink, int streamIndex,  uint8_t transactId, uint16_t channel)
{
	TAvdtpUpStreamSignal signal;

//	signal.data[0] = pLink->streams[streamIndex].remoteEndPointId << 2;

	return avdtp_SendUpstreamSignal(
		pLink,
		AVDTP_ABORT,
		AVDTP_MESSAGE_TYPE_RESPONSE_ACCEPT,
		transactId,
		&signal,
		0, channel);
}

bool avdtp_StartStream(TAvdtpLink* pLink, int streamIndex, uint16_t channel)
{
  TAvdtpUpStreamSignal signal;

  signal.data[0] = (pLink->streams[streamIndex].remoteEndPointId) << 2;
  return avdtp_SendUpstreamSignal(
    pLink,
    AVDTP_START,
    AVDTP_MESSAGE_TYPE_COMMAND,
    ++pLink->transactId,
    &signal,
    1,channel);
}

bool avdtp_SendStartStreamResp(TAvdtpLink* pLink, uint8_t transactId, uint16_t channel)
{
	TAvdtpUpStreamSignal signal;

	return avdtp_SendUpstreamSignal(
			pLink,
			AVDTP_START,
			AVDTP_MESSAGE_TYPE_RESPONSE_ACCEPT,
			transactId,
			&signal,
			0, channel);
}

bool avdtp_SendStartStreamReject(TAvdtpLink* pLink, uint8_t transactId, int streamIndex, uint16_t channel)
{
	TAvdtpUpStreamSignal signal;
	signal.data[0] = (pLink->streams[streamIndex].remoteEndPointId) << 2;
	signal.data[1] = AVDTP_ERROR_BAD_STATE;

	return avdtp_SendUpstreamSignal(
		pLink,
		AVDTP_START,
		AVDTP_MESSAGE_TYPE_RESPONSE_REJECT,
		transactId,
		&signal,
		2, channel);
}

bool avdtp_DisconnectStreamChannel(PAppAvdtp pAvdtp, uint16_t channel)
{
	PAvdtpSepData pSepData;

	pSepData = (PAvdtpSepData)pAvdtp->pChannel->SepQueue.First;
	while (pSepData != (PAvdtpSepData)0)
	{
		if (pSepData->State == AVDTP_SEP_STATE_OPENED ||
			pSepData->State == AVDTP_SEP_STATE_STREAMING)
		{
			avdtp_SendSignalAVDTP_CLOSE(pAvdtp, pSepData, AVDTP_MESSAGE_TYPE_COMMAND, channel);
			return(TRUE);
		}
		pSepData = pSepData->Next;
	}

	return(FALSE);
}
/**
* @brief  send L2C_DISC_REQ
* 
* @param  pAvdtp
* @param  status
* @param  holdLink
*
* @return  
*
*/
void avdtp_SendL2C_DISC_REQ( PAppAvdtp pAvdtp, uint16_t status, uint16_t cid )
{
		pAvdtp->pChannel->State=AVDTP_STATE_L2CDISCONNECTING;
		mpa_Sendl2cDiscReq(cid);
}

bool avdtp_CloseStream(TAvdtpLink* pLink, int streamIndex, uint16_t cid)
{
	PAvdtpChannel  pChannel;

	if ( (pChannel = avdtp_ChannelFind( pAppAvdtp, (TBdAddr *)0,
									cid )) != (PAvdtpChannel)0 )
	{
		pChannel->cause    = 0;
		pChannel->holdLink = FALSE;

		if (pChannel->Type == AVDTP_CHANNEL_TYPE_SIGNALLING)
		{
//			avdtp_ChangeState(pChannel, AVDTP_STATE_DISCONNECTING_STREAM);
			if (avdtp_DisconnectStreamChannel(pAppAvdtp, cid) == FALSE)
			{
				avdtp_SendL2C_DISC_REQ( pAppAvdtp, 0, cid );
			}
		}
		else
		{
			if (pChannel->pChannelSignalling->State != AVDTP_STATE_DISCONNECTING_STREAM)
			{
				pAppAvdtp->pChannel = pChannel->pChannelSignalling;	
				avdtp_SendSignalAVDTP_CLOSE(pAppAvdtp, pChannel->pSepData,
												   AVDTP_MESSAGE_TYPE_COMMAND, cid);
			}
		}
	}
	return true;
}

/**
* @brief  avdtp process data indication
* 
* @param  length
* @param  data
*
* @return  
*
*/
int avdtp_ProcessDataInd(uint16_t length, unsigned char* data, uint16_t channel, bool flag_signal, int release_pos)
{
	TAvdtpMessage msg;
	TAvdtpLink*	pLink = &a2dpProfile[A2dpIndex]->pA2DPData->link;

	msg.pLink = pLink;
	msg.streamIndex = 0;
	msg.dataLength = 0;
	msg.data = NULL;

	msg.type = data[0] & AVDTP_MESSAGE_TYPE_MASK;
	msg.signalID = data[1];
	msg.transactId = data[0] >> 4;
	msg.bufReleasePos = release_pos;
	
	if(flag_signal)
	{
		if (msg.type == AVDTP_MESSAGE_TYPE_COMMAND)
		{
			switch (msg.signalID)
			{
			case AVDTP_DISCOVER:
				if(length != 2)
				{						
					return avdtp_SendBadLength(msg.pLink, AVDTP_DISCOVER, msg.transactId, channel);									
				}
				break;
				
			case AVDTP_GET_CAPABILITIES:
				if(length != 3)
				{						
					return avdtp_SendBadLength(msg.pLink, AVDTP_GET_CAPABILITIES, msg.transactId, channel);									
				}
				msg.streamIndex = GetStreamIndexFromRemoteId(pLink, (uint8_t)(data[2] >> 2));
				break;

			case AVDTP_SET_CONFIGURATION:
				if(length != 14)
				{						
					return avdtp_SendBadLength(msg.pLink, AVDTP_SET_CONFIGURATION, msg.transactId, channel);									
				}
				if(((data[4] && 0x07) == 0) || ((data[6] && 0x07) == 0))
				{
					return avdtp_SendBadService(msg.pLink, AVDTP_SET_CONFIGURATION, msg.transactId, channel);
				}
				if((data[4] == 0x01) && (data[5] == 0x00))
				{
					msg.streamIndex = GetStreamIndexFromRemoteId(pLink, (uint8_t)(data[2] >> 2));
					msg.dataLength = length - 2;
					msg.data = &data[2];
				}else
				{
					return avdtp_SendBadMediaTransportFormat(msg.pLink, AVDTP_SET_CONFIGURATION, msg.transactId, channel);
				}
				break;

			case AVDTP_RECONFIGURE:
				if(length != 11)
				{						
					return avdtp_SendBadLength(msg.pLink, AVDTP_RECONFIGURE, msg.transactId, channel);									
				}
				msg.streamIndex = GetStreamIndexFromRemoteId(pLink, (uint8_t)(data[2] >> 2));
				msg.dataLength = length - 3;
				msg.data = &data[3];
				if((data[3] && 0x07) == 0)
				{
					return avdtp_SendBadService(msg.pLink, AVDTP_RECONFIGURE, msg.transactId, channel);
				}
				break;

		  	case AVDTP_GET_CONFIGURATION:
				if(length != 3)
				{						
					return avdtp_SendBadLength(msg.pLink, AVDTP_GET_CONFIGURATION, msg.transactId, channel);									
				}
				break;
				
			case AVDTP_START:
				if(length != 3)
				{						
					return avdtp_SendBadLength(msg.pLink, AVDTP_START, msg.transactId, channel);									
				}
				break;
				
			case AVDTP_ABORT:
				break;
				
			case AVDTP_OPEN:
				if(length != 3)
				{						
					return avdtp_SendBadLength(msg.pLink, AVDTP_OPEN, msg.transactId, channel);									
				}
				break;
				
			case AVDTP_SUSPEND:
				if(length != 3)
				{						
					return avdtp_SendBadLength(msg.pLink, AVDTP_SUSPEND, msg.transactId, channel);									
				}
		  		break;
				
			case AVDTP_SECURITY_CONTROL:				
			    break;
				
			default:
				avdtp_SendNotSupportedCommand(msg.pLink, msg.signalID, msg.transactId, channel);
				return AVDTP_INVALID_COMMAND;				
			} /* switch cmd */
		
			A2dpAvdtpCallback(&msg, a2dpProfile, channel);

//			SendDataResp(pLink);
			return msg.signalID;
		}
		else if (msg.type == AVDTP_MESSAGE_TYPE_RESPONSE_ACCEPT)
		{			
			switch (msg.signalID)
			{
			case AVDTP_DISCOVER:
			case AVDTP_GET_CAPABILITIES:
			case AVDTP_SET_CONFIGURATION:
			case AVDTP_GET_CONFIGURATION:
			case AVDTP_START:
			case AVDTP_RECONFIGURE:
			case AVDTP_SUSPEND:
			case AVDTP_ABORT:
			case AVDTP_SECURITY_CONTROL:
			break;

			default:
				//		    SendDataResp(pLink);
				return AVDTP_INVALID_COMMAND;
			} /* switch cmd */

			msg.dataLength = length - 2;
			msg.data = &data[2];

			A2dpAvdtpCallback(&msg, a2dpProfile, channel);

			return 0;		
		} /* if type */
		else if (msg.type == AVDTP_MESSAGE_TYPE_RESPONSE_REJECT)
		{
#if 0			
		  switch (msg.signalID)
		  {
		  case AVDTP_DISCOVER:
		  case AVDTP_GET_CAPABILITIES:
		  case AVDTP_SET_CONFIGURATION:
		  case AVDTP_GET_CONFIGURATION:
		  case AVDTP_START:
		  case AVDTP_RECONFIGURE:
		  case AVDTP_SUSPEND:
		  case AVDTP_ABORT:
		  case AVDTP_SECURITY_CONTROL:
		    break;
		  default:
		    SendDataResp(pLink);
		    return AVDTP_INVALID_COMMAND;
		  } /* switch cmd */

		  msg.dataLength = length - 2;
		  msg.data = &data[2];
		  SignalEvent(pLink, &msg);
		  SendDataResp(pLink);
		  return 0;
#endif			
		} /* if type */
	}
	else
	{	
		msg.type = AVDTP_MESSAGE_TYPE_COMMAND;
		msg.signalID = AVDTP_DATA;
		msg.dataLength = length;
		msg.data = data;
		msg.streamIndex = 0;
		A2dpAvdtpCallback(&msg, a2dpProfile, channel);	
		return AVDTP_STREAM_DATA;		
	}

	return 0;
}


bool avdtp_SendAVDTP_DATA_IND( PAppAvdtp pAvdtp, PBlueAPI_L2cDataInd pL2cDataInd, bool flag_signal )
{
#if 0
	int release_pos;
	pMessage->SourceTaskName = pAvdtp->TaskName;
	pMessage->MData.DataCBChan.Channel = pAvdtp->pChannel->cid;

	release_pos = blueFaceHandleDataInd(pBTA, BLUEFACE_PSM_AVDTP, pMessage);

	gavdtpProcessDataIndication(pMessage->MData.DataCBChan.Length, pMessage->MData.DataCBChan.BufferAddress +
	    pMessage->MData.DataCBChan.Offset, pMessage->MData.DataCBChan.Channel, flag_signal, release_pos);
#endif	
	//FIXMEE   need to fix release position problem...
	avdtp_ProcessDataInd(pL2cDataInd->length, pL2cDataInd->buf + pL2cDataInd->dataOffset, pL2cDataInd->cid, flag_signal, 0);
	return 0;
}
void avdtp_HandleAuthorizationInd(PBlueAPI_UserAuthorizationReqInd pind)
{	
	/*if(pind->server_channel == 0 && pind->outgoing == 0)	
		GAPBondlegacy_Authorize(pind->remote_BD, blueAPI_CauseAccept);*/
	void AppA2dpCallback(void *msg,TBdAddr bd,A2dpToAppMsgType AppA2dp_msg);
	AppA2dpCallback(NULL, pind->remote_BD, AUTHORIZATION_INDICATION);
}

void avdtp_HandleSecurityRegisterRsp(PBlueAPI_L2cSecurityRegisterRsp prsp)		
{
	void AppA2dpCallback(void *msg,TBdAddr bd,A2dpToAppMsgType AppA2dp_msg);
	AppA2dpCallback(NULL,0,REGISTER_COMPLETE);
}
void avdtp_HandleConnectReq(PBlueAPI_L2cConInd pL2cConInd)
{
	PAvdtpChannel pChannel;
	PAvdtpChannel pChannelSignalling;
	bool		  SignallingChannel = TRUE;
	uint8_t		  response = 0; 	/**< accept call */

	if ((pChannelSignalling = avdtp_ChannelFind(pAppAvdtp,(TBdAddr *)pL2cConInd->remote_BD,0)) !=
								 (PAvdtpChannel)0)
	{
		SignallingChannel = FALSE;
		pChannelSignalling = avdtp_ChannelFindBySEPState(pAppAvdtp, &pL2cConInd->remote_BD,
								AVDTP_SEP_STATE_OPENED);
		if (pChannelSignalling == (PAvdtpChannel)0)
		{
			response = L2CAP_ERR_REFUS_NO_RESOURCE; 	  /* reject */
		}
	}
	
	if (response == 0)
	{
		/** alloc. free channel */
		if ( (pChannel = avdtp_ChannelAllocate( pAppAvdtp )) != (PAvdtpChannel)0 )
		{
			memcpy( pChannel->RemoteBd, pL2cConInd->remote_BD, BD_ADDR_SIZE);
			pChannel->cid = pL2cConInd->cid;
			pChannel->L2C.wRxMtuSize = AVDTP_L2CAP_MTU_SIZE;  /** default 1691 */
			pChannel->L2C.wTxMtuSize = AVDTP_L2CAP_MTU_SIZE;  /** default 1691 */
			pChannel->BoundToAppl = TRUE;
			if (SignallingChannel)
			{
				pChannel->Type = AVDTP_CHANNEL_TYPE_SIGNALLING;
			}
			else
			{
				PAvdtpSepData pSepData;
	
				pSepData = avdtp_SEPFind(pChannelSignalling, AVDTP_SEP_STATE_OPENED);
				pSepData->pChannelMedia = pChannel;
				pChannel->LocalSeid 		 = pSepData->LocalSeid;
				pChannel->Type				 = AVDTP_CHANNEL_TYPE_STREAM;
				pChannel->pChannelSignalling = pChannelSignalling;
				pChannel->pSepData			 = pSepData;
			}
//chane it to mpa layer			blueAPI_L2cConnectConf(L2CAP_CONNECTION_ACCEPT, pChannel->cid);
			mpa_Sendl2cConConf(L2CAP_CONNECTION_ACCEPT, pChannel->cid);
		}
	}
	a2dp_L2cConInd(pL2cConInd, pChannel->Type);
}
void avdtp_HandleConnectRsp(PBlueAPI_L2cConRsp pL2cConRsp)
{	
	PAvdtpChannel pChannel;
	if ((pChannel = avdtp_ChannelFind(pAppAvdtp,(TBdAddr *)pL2cConRsp->remote_BD,0)) !=(PAvdtpChannel)0)
	{
		if(pChannel->cid==0)
			pChannel->cid = pL2cConRsp->cid;
		else 
		{
			pChannel = avdtp_ChannelFindStream(pAppAvdtp,(TBdAddr *)pL2cConRsp->remote_BD);
			pChannel->cid = pL2cConRsp->cid;
		}
	}
}

void avdtp_ChangeSepState( PAvdtpSepData pSepData, TAvdtpSepState State )
{
	pSepData->State = State;
}
void avdtp_HandleConnectComplete(PBlueAPI_L2cConActInd pL2cConActInd)
{
	PAvdtpChannel pChannel;
	if ( (pChannel = avdtp_ChannelFind( pAppAvdtp, 0,
											pL2cConActInd->cid)) != (PAvdtpChannel)0 )
	{
		pChannel->L2C.wRxMtuSize = pL2cConActInd->localUsMTU;
		if(pL2cConActInd->remoteMTU < AVDTP_L2CAP_MTU_SIZE)
			pChannel->L2C.wTxMtuSize = pL2cConActInd->remoteMTU;
		else
			pChannel->L2C.wTxMtuSize = AVDTP_L2CAP_MTU_SIZE;
		if (pChannel->Type == AVDTP_CHANNEL_TYPE_STREAM)
		{
			avdtp_ChangeSepState(pChannel->pSepData, AVDTP_SEP_STATE_STREAMING);
		}
		if(pChannel->cid == 0)
			pChannel->cid = pL2cConActInd->cid; //set channel cid
		pAppAvdtp->dsPoolID = pL2cConActInd->dsPoolID;
		
		a2dp_L2cConCompleteInd(pL2cConActInd, pChannel->Type);
	}
}

void avdtp_HandleL2cDataInd(PBlueAPI_L2cDataInd pL2cDataInd)
{
	bool ReleaseBuffer = TRUE;
	PAvdtpChannel  pChannel=avdtp_ChannelFind(pAppAvdtp, (TBdAddr *)0, pL2cDataInd->cid);
	if (pChannel != (PAvdtpChannel)0)
	{	
		if (pAppAvdtp->pChannel->Type == AVDTP_CHANNEL_TYPE_SIGNALLING)
		{
			a2dp_getindex((TBdAddr *)pChannel->RemoteBd);
			ReleaseBuffer = avdtp_HandleRxSignallingPacket(pAppAvdtp, pL2cDataInd);
		}
		else
		{                               /** media packet only */
			a2dpProfile[0]->AppA2dpCallback(pL2cDataInd,0,DATA_INDICATION);
			/*TAvdtpLink* pLink=&a2dpProfile[0]->pA2DPData->link;
			PAvdtpChannel  pChannel;		
			pChannel=avdtp_ChannelFindStream(pAppAvdtp, (TBdAddr *)pLink->remoteAddress);	
			mpa_Sendl2cDataReq(pL2cDataInd->buf,pL2cDataInd->dataOffset,pChannel->cid,pL2cDataInd->length); //wade
			//avdtp_SendAVDTP_DATA_IND(pAppAvdtp, pL2cDataInd, FALSE);*/
			ReleaseBuffer = FALSE;
		}
	}
	if (ReleaseBuffer)
	{
		if (pL2cDataInd->flag & DATA_CB_RELEASE)
		{
			blueAPI_BufferRelease(pL2cDataInd->buf);
		}
	}
}

void avdtp_ChannelFree( PAppAvdtp pAvdtp, PAvdtpChannel pChannel )
{
	PAvdtpSepData pSepData;

	while (pChannel->SepQueue.Count)
	{
		pSepData = (PAvdtpSepData)osQueueOut(&pChannel->SepQueue);
		if (pSepData != (PAvdtpSepData)0)
		{
	  		osQueueIn(&pAvdtp->FreeSepQueue , pSepData);
		}
	}

//	avdtp_ChangeState( pChannel, AVDTP_STATE_CLOSED );
	pChannel->Used = FALSE;
}


void avdtp_HandleL2cDisInd(PBlueAPI_L2cDiscInd pL2cDisInd)
{
	PAvdtpChannel		pChannel;
	
	if ( (pChannel = avdtp_ChannelFind( pAppAvdtp, (TBdAddr *)0,
									  pL2cDisInd->cid )) != (PAvdtpChannel)0 )
	{
		/* stop timer potentially running */
//	timer FIXMEEEE   	avdtp_TimerStop( pAvdtp, pAvdtp->pChannel->TimerID );
//	    pAppAvdtp->pChannel->TimeOut = FALSE;
	
		if (pChannel->Type == AVDTP_CHANNEL_TYPE_STREAM)
		{
			if (pChannel->pSepData != (PAvdtpSepData)0)
			{
				osQueueDelete(&pChannel->pChannelSignalling->SepQueue, pChannel->pSepData);
				osQueueIn(&pAppAvdtp->FreeSepQueue, pChannel->pSepData);
			}
		}

		avdtp_ChannelFree( pAppAvdtp, pChannel );
	}

//change it to mpa layer	blueAPI_L2cDisConf(L2CAP_NO_CAUSE, pL2cDisInd->cid);
	mpa_Sendl2cDiscConf(pL2cDisInd->cid);

	a2dp_L2cDisInd(pL2cDisInd, pChannel->Type,pChannel->RemoteBd);
}
void avdtp_HandleL2cDisRsp(PBlueAPI_L2cDiscRsp pL2cDisRsp)
{
	PAvdtpChannel		pChannel;
	if ( (pChannel = avdtp_ChannelFind( pAppAvdtp, (TBdAddr *)0,
										  pL2cDisRsp->cid )) != (PAvdtpChannel)0 )
	{
		avdtp_ChannelFree( pAppAvdtp, pChannel );
	}
}
	
void avdtp_CallBack(void* buf, ProtocolUsMsg l2c_msg)
{
	switch(l2c_msg)
	{
	case PROTOCOL_AUTHORIZATION_IND:
		avdtp_HandleAuthorizationInd((PBlueAPI_UserAuthorizationReqInd)buf);
		break;
	case PROTOCOL_SECURITY_REGISTER_RSP:
		avdtp_HandleSecurityRegisterRsp((PBlueAPI_L2cSecurityRegisterRsp)buf);		
		break;
	case L2CAP_CONNECT_IND:
		avdtp_HandleConnectReq((PBlueAPI_L2cConInd)buf);
		break;
	case L2CAP_CONNECT_RSP:
		avdtp_HandleConnectRsp((PBlueAPI_L2cConRsp)buf);
		break;		
	case L2CAP_CONNECT_COMPLETE:
		avdtp_HandleConnectComplete((PBlueAPI_L2cConActInd)buf);
		break;
	case L2CAP_DATA_IND:
		avdtp_HandleL2cDataInd((PBlueAPI_L2cDataInd)buf);
		break;
	case L2CAP_DISC_IND:
		avdtp_HandleL2cDisInd((PBlueAPI_L2cDiscInd)buf);
		break;
	case L2CAP_DISC_RSP:
		avdtp_HandleL2cDisRsp((PBlueAPI_L2cDiscRsp)buf);
		break;
	}
}

void avdtp_Connect(PAvdtpConReq	pConReq)
{
	PAvdtpChannel   pChannel = (PAvdtpChannel)0;
	uint16_t			wRxMtuSize;
	uint16_t			status = AVDTP_SUCCESS;

	/*if(pConReq->reconnectFlag == TRUE)
	{
		if(getProtocolBdByQueueID(pAppAvdtp->QueueID, pConReq->bd) == FALSE)
		{
			//get bdaddr fail....
			return;
		}
	}*/
	/* frame size check */
	if ( pConReq->frameSize == 0 )
	{
		wRxMtuSize = AVDTP_L2CAP_MTU_SIZE;		  /* default */
	} else
	{
		wRxMtuSize = pConReq->frameSize;

		if ( wRxMtuSize > AVDTP_L2CAP_MTU_SIZE )
		{
			status = AVDTP_ERR | AVDTP_ERR_INVALID_FRAME_SIZE;
		}
	}

	if (pConReq->channel == AVDTP_CHANNEL_TYPE_SIGNALLING)
	{
		if (avdtp_ChannelFind(pAppAvdtp,(TBdAddr *)pConReq->bd,0) != (PAvdtpChannel)0)
		{
			status = AVDTP_ERR | AVDTP_ERR_NORESOURCES;  /* support only one signalling channel */
		}
	}

	if ( status == AVDTP_SUCCESS )
	{
		/* alloc. free channel */
		if ( (pChannel = avdtp_ChannelAllocate(pAppAvdtp)) != (PAvdtpChannel)0 )
		{
			memcpy(pChannel->RemoteBd, pConReq->bd, BD_ADDR_SIZE);
			//OR	  pChannel->Initiator = TRUE;
			pChannel->reqId	  = pConReq->reqId; //0
			pChannel->L2C.wRxMtuSize = wRxMtuSize;

			pChannel->BoundToAppl = TRUE;
			pChannel->cid = 0;
			if (pConReq->channel == AVDTP_CHANNEL_TYPE_SIGNALLING)
			{
				pChannel->Type = AVDTP_CHANNEL_TYPE_SIGNALLING;

				mpa_Sendl2cConReq(PSM_AVDTP, UUID_AVDTP, pAppAvdtp->QueueID, wRxMtuSize, pChannel->RemoteBd);
			}
			else
			{
				PAvdtpChannel pChannelSignalling;

				pChannel->Type = AVDTP_CHANNEL_TYPE_STREAM;

				if ((pChannelSignalling = avdtp_ChannelFind(pAppAvdtp,(TBdAddr *)pConReq->bd,0)) !=
																  (PAvdtpChannel)0)
				{
					if (memcmp(pConReq->bd, pChannelSignalling->RemoteBd, BD_ADDR_SIZE) == 0)
					{
						PAvdtpSepData pSepData;
						uint8_t		  RemoteSeid = (uint8_t)(pConReq->RemoteSeid);

						pSepData = (PAvdtpSepData)pChannelSignalling->SepQueue.First;

						while (pSepData != (PAvdtpSepData)0)
						{
							if (pSepData->RemoteSeid == RemoteSeid &&
							  pSepData->State	== AVDTP_SEP_STATE_CONFIGURED)
							{
								break;
							}
							pSepData = (PAvdtpSepData)pSepData->Next;
						}

						if (pSepData != (PAvdtpSepData)0)
						{
							pSepData->pChannelMedia = pChannel;

							//pChannel->RemoteSeid		   = RemoteSeid;
							pChannel->Type			   = AVDTP_CHANNEL_TYPE_STREAM;
							pChannel->pChannelSignalling = pChannelSignalling;
							pChannel->pSepData		   = pSepData;

							status = avdtp_SendSignalAVDTP_OPEN(pAppAvdtp, pSepData,
															 AVDTP_MESSAGE_TYPE_COMMAND, pAppAvdtp->pChannel->cid);
						}
						else
						{
							status = AVDTP_ERR | AVDTP_ERR_BAD_ACP_SEID;
						}
					}
					else
					{
						status = AVDTP_ERR | AVDTP_ERR_NORESOURCES;
					}
				}
			else
			{
				status = AVDTP_ERR | AVDTP_ERR_NORESOURCES;
			}
		}
	}
	else
	{
		status = AVDTP_ERR | AVDTP_ERR_NORESOURCES;
	}
}
	if ( status != AVDTP_SUCCESS )
	{
		//connect fail.....
		if (pChannel != (PAvdtpChannel)0)
			avdtp_ChannelFree(pAppAvdtp, pChannel);
	}
}

bool avdtp_ProtocolInit(void)
{
	int i;
	int queueID;
	//malloc gloable param
	pAppAvdtp = osMemoryAllocate(RAM_TYPE_DATA_ON, sizeof(TAppAvdtp));

	if(pAppAvdtp == NULL)
	{
		//MALLOC FAIL.... RETURN FALSE;
		return FALSE;
	}
	memset(pAppAvdtp, 0, sizeof(TAppAvdtp));

	queueID = mpa_RegisterProtocol(PSM_AVDTP, avdtp_CallBack);
	if(queueID == -1)
	{
		/*get protocol queue id fail*/
		return FALSE;
	}
	pAppAvdtp->QueueID = queueID;
	pAppAvdtp->txOffset = DATACB_INIT_OFFSET;
	pAppAvdtp->writeOffset = (BT_L1_HCI_DS_OFFSET + ACL_HDR_LENGTH + L2CAP_HDR_LENGTH);

	for (i = 0; i < AVDTP_MAX_SEP_COUNT; i++)
	{
		osQueueIn(&pAppAvdtp->FreeSepQueue, &pAppAvdtp->SepData[i]);
	}

	memset( pAppAvdtp->ChannelArray, 0, sizeof(pAppAvdtp->ChannelArray) );

	for (i = 0; i < AVDTP_MAX_CHANNELS_COUNT; i++)
	{
		pAppAvdtp->ChannelArray[i].iNumber = i;
	}
	//change it to mpa layer	blueAPI_L2cProtocolRegister(PSM_AVDTP, pAppAvdtp->QueueID, 1);
	mpa_Sendl2cProtocolRegister(PSM_AVDTP, pAppAvdtp->QueueID, 1);
	return TRUE;
}

//wade
void avdtp_SetSepStateToConfigured(PAppAvdtp pAvdtp, uint8_t MessageType)
{
	PAvdtpSepData pSepData;

	pSepData = avdtp_SEPFind(pAvdtp->pChannel, AVDTP_SEP_STATE_IDLE);
	if (pSepData == (PAvdtpSepData)0)
	{
//		BT_TRACE_VERB1(
//		    TRACE_PRuint8_tF_0(dbError,"!!avdtp_SetSepStateToConfigured: no SEP in idle found"));
	}
	else
	{
		if (MessageType == AVDTP_MESSAGE_TYPE_RESPONSE_ACCEPT)
		{
			pSepData->State = AVDTP_SEP_STATE_CONFIGURED;
		}
		else
		{
			osQueueDelete(&pAvdtp->pChannel->SepQueue, pSepData);
			osQueueIn(&pAvdtp->FreeSepQueue, pSepData);
		}
	}
}

STATIC void avdtp_HandleSetConfigurationCommand(PAppAvdtp pAvdtp, uint8_t * pData)
{
	PAvdtpChannel pChannel;
	PAvdtpSepData pSepData;
	uint8_t          AcpSeid;

	pChannel = pAvdtp->pChannel;

	AcpSeid = pData[0] >> 2;
	pSepData = (PAvdtpSepData)pChannel->SepQueue.First;
	while (pSepData != (PAvdtpSepData)0)
	{
		if (pSepData->RemoteSeid == AcpSeid) /* remote device is aceptor */
		  break;
		pSepData = pSepData->Next;
	}

	if (pSepData != (PAvdtpSepData)0)
	{
		return;
	}

	pSepData = (PAvdtpSepData)osQueueOut(&pAvdtp->FreeSepQueue);
	if (pSepData != (PAvdtpSepData)0)
	{
		memset(pSepData, 0, sizeof(PAvdtpSepData));

		osQueueIn(&pChannel->SepQueue, pSepData);
		pSepData->State   = AVDTP_SEP_STATE_IDLE;
		pSepData->RemoteSeid = AcpSeid;
		pSepData->LocalSeid  = pData[1] >> 2; /* we are IntSeid */
	}
	else
	{
		//...
	}
}

bool avdtp_SendUpstreamSignal(TAvdtpLink* pLink,
                                              TAvdtpEventSignalID signalID,
                                              TAvdtpEventType type,
                                              uint8_t transactId,
                                              TAvdtpUpStreamSignal *pSignal,
                                              int size, uint16_t channel)
{
//	LPbtLink      pbtLink         =pLink->bLinkHandle;
//	blueFaceStatus bfStatus;
	unsigned char* mybuf;
	uint16_t op = pAppAvdtp->writeOffset;

	pSignal->signalID = signalID;
	pSignal->header = (transactId << 4) + AVDTP_PACKET_TYPE_SINGLE + type;
	size += 2; /* add header */

	if(signalID == AVDTP_SET_CONFIGURATION)
	{
		if (type == AVDTP_MESSAGE_TYPE_COMMAND)
		{
			avdtp_HandleSetConfigurationCommand(pAppAvdtp, (uint8_t *)pSignal->data);
		}
		else
		{
			avdtp_SetSepStateToConfigured(pAppAvdtp, type);
		}
	}

	if (blueAPI_BufferGet(pAppAvdtp->dsPoolID, op + size, 24/*dsoffset count*/, (void FAR *)&mybuf))
	{
//		bfStatus = blueFaceIllParameter;
		return false;
	}
	//if you use dsoffset, you need to fix here
	mybuf[op++] = pSignal->header;
	mybuf[op++] = pSignal->signalID;
	memcpy(mybuf + op, pSignal->data, (size - 2));

	avdtp_SendSignal(pAppAvdtp, signalID, type, mybuf, size, channel);

	return true;

}

/**
* @brief  Handle Rx OPEN (single packet) 
* 
* @param  pAvdtp
* @param  pMessage
* @param  MessageType
*
* @return  
*
*/
static void avdtp_HandleRxOpen(PAppAvdtp pAvdtp, PBlueAPI_L2cDataInd pL2cDataInd,
                                             int8_t MessageType)
{
	PAvdtpSepData pSepData;
	PAvdtpChannel pChannelSignalling;
	int8_t          AcpSeid;

	AcpSeid =                        /** or error code (response reject) */
	(pL2cDataInd->buf[pL2cDataInd->dataOffset +
	                                AVDTP_SINGLE_PACKET_HEADER_SIZE] >> 2);

	if (MessageType == AVDTP_MESSAGE_TYPE_COMMAND)
	{
		pSepData = avdtp_SEPFindSeidAndState(pAvdtp->pChannel,
		                                    AcpSeid, AVDTP_SEP_STATE_CONFIGURED);
	}
	else
	{
		pSepData = avdtp_SEPFind(pAvdtp->pChannel, AVDTP_SEP_STATE_CONFIGURED);
	}

	if (pSepData == (PAvdtpSepData)0)
	{
		/** send response ? */
	//	BT_TRACE_VERB1( TRACE_PRuint8_tF_1(dbError,"!!avdtp_HandleRxOpen: SEP=0x%x not configured",
	//	                                     AcpSeid));
		return;
	}

	switch (MessageType)
	{
	case AVDTP_MESSAGE_TYPE_COMMAND:
		avdtp_SendSignalAVDTP_OPEN(pAvdtp, pSepData, AVDTP_MESSAGE_TYPE_RESPONSE_ACCEPT, pL2cDataInd->cid);
		pSepData->State = AVDTP_SEP_STATE_OPENED;
		break;

	case AVDTP_MESSAGE_TYPE_RESPONSE_ACCEPT:
		pChannelSignalling = pAvdtp->pChannel;
		pAvdtp->pChannel = pSepData->pChannelMedia;
		mpa_Sendl2cConReq(PSM_AVDTP, UUID_AVDTP, pAvdtp->QueueID, pAvdtp->pChannel->L2C.wRxMtuSize, pAvdtp->pChannel->RemoteBd);
		pAvdtp->pChannel = pChannelSignalling;

		pSepData->State = AVDTP_SEP_STATE_OPENED;
		break;

	case AVDTP_MESSAGE_TYPE_RESPONSE_REJECT:
		pChannelSignalling = pAvdtp->pChannel;
		pAvdtp->pChannel = pSepData->pChannelMedia;
#if 0 //fix me		
		avdtp_SendAVDTP_CON_CONF(pAvdtp, 0 /*cid*/, pAvdtp->pChannel->RemoteBd,
		      (uint16_t)((pAvdtp->pChannel->LocalSeid << 8) | AVDTP_FLAG_STREAM_CHANNEL),
		      pAvdtp->pChannel->reqId, (uint16_t)(AVDTP_ERR | AcpSeid));
		avdtp_ChannelFree(pAvdtp, pAvdtp->pChannel);
#endif		
		pAvdtp->pChannel = pChannelSignalling;
		break;
	}
}

/**
* @brief  Handle Rx CLOSE (single packet) 
* 
* @param  pAvdtp
* @param  pMessage
* @param  MessageType
*
* @return  
*
*/
static void avdtp_HandleRxClose(PAppAvdtp pAvdtp, PBlueAPI_L2cDataInd pL2cDataInd,
                                              uint8_t MessageType)
{
	PAvdtpChannel pChannelSignalling;
	PAvdtpSepData pSepData;
	uint8_t          AcpSeid;         /**< = error code (response reject) */

	AcpSeid =
	(pL2cDataInd->buf[pL2cDataInd->dataOffset +
	                                   AVDTP_SINGLE_PACKET_HEADER_SIZE] >> 2);

	if (MessageType == AVDTP_MESSAGE_TYPE_COMMAND)
	{
		pSepData = (PAvdtpSepData)pAvdtp->pChannel->SepQueue.First;
		while (pSepData != (PAvdtpSepData)0)
		{
			if (pSepData->LocalSeid == AcpSeid)
			{
				break;
			}
			pSepData = (PAvdtpSepData)pSepData->Next;
		}

		if (pSepData != (PAvdtpSepData)0)
		{
			pChannelSignalling = pAvdtp->pChannel;
			avdtp_SendSignalAVDTP_CLOSE(pAvdtp, pSepData, AVDTP_MESSAGE_TYPE_RESPONSE_ACCEPT, pL2cDataInd->cid);
			osQueueDelete(&pChannelSignalling->SepQueue, pSepData);
			osQueueIn(&pAvdtp->FreeSepQueue, pSepData);
		}
		else
		{
			//
		}
	}
	else            /**< response */
	{
#if 1	
		pSepData = avdtp_SEPFind(pAvdtp->pChannel, AVDTP_SEP_STATE_CLOSESENT);

		if (pSepData != (PAvdtpSepData)0)
		{
			pChannelSignalling = pAvdtp->pChannel;
			if (pSepData->pChannelMedia != (PAvdtpChannel)0 &&
			  pSepData->pChannelMedia->State != AVDTP_STATE_CLOSED &&
			  pSepData->pChannelMedia->State != AVDTP_STATE_L2CDISCONNECTING)
			{
				pAvdtp->pChannel = pSepData->pChannelMedia;
				avdtp_SendL2C_DISC_REQ( pAvdtp, pAvdtp->pChannel->cause,pL2cDataInd->cid);
				pAvdtp->pChannel = pChannelSignalling;
			}
			else
			{
				if (avdtp_DisconnectStreamChannel(pAvdtp,pL2cDataInd->cid) == FALSE)
				{
					pAvdtp->pChannel = pChannelSignalling;
					pChannelSignalling->State  = AVDTP_STATE_L2CCONNECTED;
					avdtp_SendL2C_DISC_REQ( pAvdtp, pChannelSignalling->cause,pL2cDataInd->cid );
				}
			}
			osQueueDelete(&pChannelSignalling->SepQueue, pSepData);
			osQueueIn(&pAvdtp->FreeSepQueue, pSepData);
		}
		else
		{
			
		}
#endif		
	}   
}
static void avdtp_HandleRxSetConfigurationCommand(PAppAvdtp pAvdtp, uint8_t * pData)
{
	PAvdtpChannel pChannel;
	PAvdtpSepData pSepData;
	uint8_t          AcpSeid;

	pChannel = pAvdtp->pChannel;

	AcpSeid = pData[0] >> 2;
	pSepData = (PAvdtpSepData)pChannel->SepQueue.First;
	while (pSepData != (PAvdtpSepData)0)
	{
		if (pSepData->LocalSeid == AcpSeid) /**<  we are aceptor */
		{
			break;
		}
		pSepData = pSepData->Next;
	}

	if (pSepData != (PAvdtpSepData)0)
	{
//		BT_TRACE_VERB1(TRACE_PRuint8_tF_1(
//		       dbError,"!!avdtp_HandleSetConfigurationCommand: SEP=0x%x already configured",
//		       pData[0]));

		return;
	}

	pSepData = (PAvdtpSepData)osQueueOut(&pAvdtp->FreeSepQueue);
	if (pSepData != (PAvdtpSepData)0)
	{
		memset(pSepData, 0, sizeof(PAvdtpSepData));

		osQueueIn(&pChannel->SepQueue, pSepData);
		pSepData->State   = AVDTP_SEP_STATE_IDLE;
		pSepData->LocalSeid = AcpSeid;
		pSepData->RemoteSeid  = pData[1] >> 2; /**< we are IntSeid */
	}
	else
	{
//		BT_TRACE_VERB1(TRACE_PRuint8_tF_1(
//		       dbError,"!!avdtp_HandleSetConfigurationCommand: too many SEP (<= %d)",
//		       AVDTP_MAX_SEP_COUNT));
	}
}

static void avdtp_HandleRxSetConfiguration(PAppAvdtp pAvdtp, PBlueAPI_L2cDataInd pL2cDataInd,
                                                         uint16_t MessageType)
{
	if (MessageType == AVDTP_MESSAGE_TYPE_COMMAND)
	{
		avdtp_HandleRxSetConfigurationCommand(pAvdtp,
		                      pL2cDataInd->buf+
		                      pL2cDataInd->dataOffset+
		                      AVDTP_SINGLE_PACKET_HEADER_SIZE);
	}
	else
	{
		avdtp_SetSepStateToConfigured(pAvdtp, MessageType);
	}	
}
void AvdtpTimerCallBack(void *xTimer)
{
	uint8_t time_type = 1;
	if(pAppAvdtp->avdtp_time_handle == xTimer)
		a2dpProfile[A2dpIndex]->AppA2dpCallback(&time_type, a2dpProfile[A2dpIndex]->pA2DPData->link.remoteAddress, TIME_OUT);	
}
static bool avdtp_SendSignal(PAppAvdtp pAvdtp, uint8_t Signal, uint8_t MessageType, unsigned char* buf, uint8_t Length, uint8_t cid)
{
	PAvdtpChannel pChannel;
	uint8_t *		  pData;

	pData	= buf;

	pChannel = avdtp_ChannelFind(pAvdtp, (TBdAddr *)0, cid);

	if(pChannel == NULL)
	{
		return false;
	}

	if (pChannel->L2C.wTxMtuSize >= Length)
	{
//change it to mpa layer		blueAPI_L2cDataReq(buf, pAvdtp->writeOffset, cid, Length);
		mpa_Sendl2cDataReq(buf, pAvdtp->writeOffset, cid, Length);

		if (MessageType == AVDTP_MESSAGE_TYPE_COMMAND)
		{
			avdtp_SaveSignallingHeader(pAvdtp, pData);
//how to soulve thie problem			avdtp_TimerStart(pAvdtp, AVDTP_TIMERID_RTX_SIG_TIMER, RTX_SIG_TIMER);
			osStartTimer(&(pAppAvdtp->avdtp_time_handle), 0, 0, 0, AVDTP_WAIT_TIME, AvdtpTimerCallBack);
		}
		return true;
	}
	return(TRUE);
}

/**
* @brief  Handle rx signal (command or response)  
* 
* @param  pAvdtp
*
* @return  
*
*/
bool avdtp_HandleRxSignallingPacket(PAppAvdtp pAvdtp, PBlueAPI_L2cDataInd pL2cDataInd)
{
	PAvdtpChannel pChannel;
	unsigned char*        pData;
	uint16_t          Length = 0;
	uint8_t          Signal;
	uint8_t          PacketType;
	uint8_t          MessageType;
	uint8_t          TransactionLabel;
	uint8_t          ErrorCode = AVDTP_SUCCESS;
	bool            ReturnValue = TRUE;

	pData  = pL2cDataInd->buf +
	    pL2cDataInd->dataOffset;
	Length = pL2cDataInd->length;

	if (Length == 0)
	{
#if 0	
		BT_TRACE_VERB1(TRACE_PRuint8_tF_0(dbError,
		                     "!!avdtp_HandleRxSignallingPacket: signal received length=0"));
#endif
		return(ReturnValue);                   /**< ignore short signal */
	}

	PacketType       = *pData & AVDTP_PACKET_TYPE_MASK;
	MessageType      = *pData & AVDTP_MESSAGE_TYPE_MASK;
	TransactionLabel = *pData & 0xF0;
	pData++;

	Length--;

	if (MessageType != AVDTP_MESSAGE_TYPE_COMMAND &&
	  MessageType != AVDTP_MESSAGE_TYPE_RESPONSE_ACCEPT &&
	  MessageType != AVDTP_MESSAGE_TYPE_RESPONSE_REJECT)
	{
#if 0
		BT_TRACE_VERB1(TRACE_PRuint8_tF_1(dbError,
		                     "!!avdtp_HandleRxSignallingPacket: bad message type=0x%x",
		                     MessageType));
#endif
		ErrorCode = AVDTP_ERR_BAD_HEADER_FORMAT;
	}

	pChannel = pAvdtp->pChannel;

	if (ErrorCode == AVDTP_SUCCESS)
	{
		switch (PacketType)
		{
		case AVDTP_PACKET_TYPE_SINGLE:
			if (Length >= AVDTP_SINGLE_PACKET_HEADER_SIZE - 1)
			{
				Signal = *pData++;
				Length--;
#if 0
				BT_TRACE_VERB1(TRACE_PRuint8_tF_4(dbTrace,
				 "avdtp_HandleRxSignallingPacket: single packet received (header = 0x%x signal=0x%x length=0x%x)",
				 TransactionLabel|PacketType|MessageType, Signal, Length, 0));
#endif
			}
			else
			{
				ErrorCode = AVDTP_ERR_BAD_HEADER_FORMAT;
			}
			break;

		case AVDTP_PACKET_TYPE_START:
			if (Length >= AVDTP_START_PACKET_HEADER_SIZE - 1)
			{
				pChannel->nosp   = *pData++;
				pChannel->Signal = Signal = *pData++;
				Length -= (AVDTP_START_PACKET_HEADER_SIZE - 1);

#if 0
				BT_TRACE_VERB1(TRACE_PRuint8_tF_4(dbTrace,
				 "avdtp_HandleRxSignallingPacket: start packet received (header = 0x%x signal=0x%x nosp=0x%x length=0x%x)",			 
				 TransactionLabel|PacketType|MessageType, Signal, pChannel->nosp, Length));
#endif	
			}
			else
			{
				ErrorCode = AVDTP_ERR_BAD_HEADER_FORMAT;
			}
			break;

		case AVDTP_PACKET_TYPE_CONTINUE:
			Signal = pAvdtp->pChannel->Signal;

//			BT_TRACE_VERB1(TRACE_PRuint8_tF_4(dbTrace,
//			     "avdtp_HandleRxSignallingPacket: continue packet received (header = 0x%x signal=0x%x length=0x%x)",
//			     TransactionLabel|PacketType|MessageType, Signal, Length, 0));
			break;

		case AVDTP_PACKET_TYPE_END:
			Signal = pAvdtp->pChannel->Signal;

//			BT_TRACE_VERB1(TRACE_PRuint8_tF_4(dbTrace,
//			     "avdtp_HandleRxSignallingPacket: end packet received (header = 0x%x signal=0x%x length=0x%x)",
//			     TransactionLabel|PacketType|MessageType, Signal, Length, 0));
			break;
		}
	}

	switch (Signal)
	{
	case AVDTP_SET_CONFIGURATION:
	case AVDTP_DISCOVER:
	case AVDTP_GET_CAPABILITIES:
	case AVDTP_GET_CONFIGURATION:
	case AVDTP_RECONFIGURE:
	case AVDTP_OPEN:
	case AVDTP_START:
	case AVDTP_CLOSE:
	case AVDTP_SUSPEND:
	case AVDTP_ABORT:
	case AVDTP_SECURITY_CONTROL:
		break;

	default:
//		BT_TRACE_VERB1(TRACE_PRuint8_tF_1(dbError,
//		                 "!!avdtp_HandleRxSignallingPacket: unknown signal=0x%x",
//		                 Signal));
		ErrorCode = AVDTP_ERR_BAD_HEADER_FORMAT;
		break;
	}

	if (ErrorCode != AVDTP_SUCCESS &&
	  MessageType != AVDTP_MESSAGE_TYPE_COMMAND)
	{
		return(ReturnValue);           /**< ignore response with bad header format */
	}

	if (ErrorCode == AVDTP_SUCCESS &&
	  PacketType == AVDTP_PACKET_TYPE_SINGLE)
	{
		if (pChannel->pRxSignal != (uint8_t *)0)
		{
//			BT_TRACE_VERB1(TRACE_PRuint8_tF_1(dbTrace,
//			         "!!avdtp_HandleRxSignal: ignore last start packet (signal=0x%x)",
//			         pChannel->Signal));

			blueAPI_BufferRelease((PVOID)pChannel->pRxSignal);
			pChannel->pRxSignal      = (uint8_t *)0;
			pChannel->RxSignalLength = 0;
		}
	}

	if (ErrorCode == AVDTP_SUCCESS &&
	  PacketType != AVDTP_PACKET_TYPE_SINGLE)
	{
#if 0		//FIXMEE do not process start packet here	

		if (pChannel->pRxSignal == (uint8_t *)0)    
		{
			if (PacketType == AVDTP_PACKET_TYPE_START)
			{

				if (osBufferGet(pAvdtp->signalPoolID,
				                AVDTP_SIGNALLING_BUFFER_SIZE + DATACB_INIT_OFFSET,
				                (PVOID FAR *)&pChannel->pRxSignal) != 0)
				{
					BT_TRACE_VERB1(
					    TRACE_PRuint8_tF_0(dbError,"!!avdtp_HandleRxSignallingPacket: NO MEMORY"));
					return(ReturnValue);                            
				}
		                                                 /**< build header */
				pChannel->pRxSignal[0]   = TransactionLabel|AVDTP_PACKET_TYPE_SINGLE|MessageType;
				pChannel->pRxSignal[1]   = Signal;
				pChannel->RxSignalLength = 2;
			
			}
			else    /**< continue or end packet without start packet */
			{
				if (MessageType == AVDTP_MESSAGE_TYPE_COMMAND)
				{
//					BT_TRACE_VERB1(TRACE_PRuint8_tF_2(dbError,
//					     "!!avdtp_HandleRxSignallingPacket: not single or start command packet (header=0x%x signal=0x%x)",
//					     TransactionLabel|PacketType|MessageType, Signal));

					ErrorCode = AVDTP_ERR_BAD_HEADER_FORMAT;
				}
				else
				{
//					BT_TRACE_VERB1(TRACE_PRuint8_tF_2(dbError,
//					     "!!avdtp_HandleRxSignallingPacket: not single or start response packet (header=0x%x signal=0x%x)",
//					     TransactionLabel|PacketType|MessageType, Signal));

					return(ReturnValue);                        /**< ignore response */
				}
			}
			
		}
		else   /* last packet was start packet */
		{
			if (PacketType == AVDTP_PACKET_TYPE_START)
			{
				BT_TRACE_VERB1(TRACE_PRuint8_tF_1(dbError,
				         "!!avdtp_HandleRxSignallingPacket: ignore last start packet (signal=0x%x)",
				         pChannel->Signal));

				pChannel->RxSignalLength = 0;

				if (MessageType == AVDTP_MESSAGE_TYPE_COMMAND)
				{
					BT_TRACE_VERB1(TRACE_PRuint8_tF_2(dbError,
					     "!!avdtp_HandleRxSignallingPacket: not continue or end command packet (header=0x%x signal=0x%x)",
					     TransactionLabel|PacketType|MessageType, Signal));

					ErrorCode = AVDTP_ERR_BAD_HEADER_FORMAT;
				}
				else
				{
					BT_TRACE_VERB1(TRACE_PRuint8_tF_2(dbError,
					     "!!avdtp_HandleRxSignallingPacket: not continue or end response packet (header=0x%x signal=0x%x)",
					     TransactionLabel|PacketType|MessageType, Signal));

					return(ReturnValue);                         /**< ignore response */
				}
			}
		}
#endif	
	}

	if (ErrorCode == AVDTP_SUCCESS)
	{
		if (PacketType != AVDTP_PACKET_TYPE_SINGLE && Length > 0)
		{
			if ((Length + pChannel->RxSignalLength) > AVDTP_SIGNALLING_BUFFER_SIZE)
			{
				memcpy(pChannel->pRxSignal + pChannel->RxSignalLength + DATACB_INIT_OFFSET,
			       	pData, Length);
			}
			else
			{
//				BT_TRACE_VERB1(
//				      TRACE_PRuint8_tF_1(dbError,"!!avdtp_HandleRxSignallingPacket: signal too long (max=%d)",
//				      AVDTP_SIGNALLING_BUFFER_SIZE));
			}
			pChannel->RxSignalLength += Length;
		}

		if (PacketType == AVDTP_PACKET_TYPE_SINGLE ||
		    PacketType == AVDTP_PACKET_TYPE_END)
		{
//			MESSAGE_T Message;

			if (PacketType == AVDTP_PACKET_TYPE_SINGLE)
			{
#if 0			
				Message.MData.DataCBChan.BufferAddress =
				                          pL2cDataInd->buf;
				Message.MData.DataCBChan.Length        =
				                          pL2cDataInd->length;
				Message.MData.DataCBChan.Offset        =
				                          pL2cDataInd->dataOffset;
				Message.MData.DataCBChan.Flag          =
				                          pL2cDataInd->flag;
				Message.MData.DataCBChan.Channel=
				                          pL2cDataInd->cid;
#endif				
				ReturnValue = FALSE;
			}
			else
			{
#if 0			
				if (pChannel->RxSignalLength > AVDTP_SIGNALLING_BUFFER_SIZE)
				{
//					BT_TRACE_VERB1(
//					    TRACE_PRuint8_tF_2(dbError,"!!avdtp_HandleRxSignallingPacket: signal too long (max=%d - %d)",
//					    AVDTP_SIGNALLING_BUFFER_SIZE, pChannel->RxSignalLength));
				}

				Message.MData.DataCB.BufferAddress = pChannel->pRxSignal;
				Message.MData.DataCB.Length        = pChannel->RxSignalLength;
				Message.MData.DataCB.Offset        = DATACB_INIT_OFFSET;
				Message.MData.DataCB.Flag          = DATA_CB_RELEASE;

				pChannel->pRxSignal      = (uint8_t *)0;
				pChannel->RxSignalLength = 0;
#endif				
			}

			if (MessageType != AVDTP_MESSAGE_TYPE_COMMAND)         
			{
//FIXMEE how to use timer??				avdtp_TimerStop(pAvdtp, AVDTP_TIMERID_RTX_SIG_TIMER);
				osStopTimer(&(pAppAvdtp->avdtp_time_handle));
			}

			switch (Signal)
			{
			case AVDTP_OPEN:
				pAvdtp->TransactionLabel = TransactionLabel;      /**< for response */
				avdtp_HandleRxOpen(pAvdtp, pL2cDataInd, MessageType);
				ReturnValue = TRUE;
				break;

			case AVDTP_CLOSE:
				pAvdtp->TransactionLabel = TransactionLabel;      /* for response */
				avdtp_HandleRxClose(pAvdtp, pL2cDataInd, MessageType);
				ReturnValue = TRUE;
				break;

			case AVDTP_SET_CONFIGURATION:
				avdtp_HandleRxSetConfiguration(pAvdtp, pL2cDataInd, MessageType);
				/**no break, fall through*/
			default:
				avdtp_SendAVDTP_DATA_IND(pAvdtp, pL2cDataInd, TRUE);
				break;
			}
		}
	}
	else
	{
#if 0	
		ReturnValue = avdtp_SendSignal(pAvdtp, Signal,
		                                      AVDTP_MESSAGE_TYPE_RESPONSE_REJECT);
#endif
	}
	ReturnValue = TRUE;
	return(ReturnValue);
}

