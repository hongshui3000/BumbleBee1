#include "a2dp.h"
#include "a2dp_api.h"
#include "avdtp.h"

#include <sdp_code.h>
#include <bterrcod.h>
#include "legacy.h"
#include "blueapi.h"
#include "stdio.h"
#include "trace.h"
#include "btcommon.h"
#include "common_defs.h"
#include "os_timer.h"
#include "os_mem.h"
#define __FILE_NUM__ 0

int A2dpIndex=0;
PAppAvdtp pAppAvdtp;
PA2dpProfile a2dpProfile[MAX_INDEX];

static void a2dp_ClearAllBuffers( PA2dpProfile pProfile);
static int a2dp_CalcSBCBitRate(PA2dpProfile pProfile, TSBCParams* pParams, int frameSize);
static unsigned char* a2dp_AllocatePCMBuffer(PA2dpProfile pProfile);
void AppA2dpCallback(void *msg,TBdAddr bd,A2dpToAppMsgType AppA2dp_msg);

/*
 *  Initialize a list to an empty state.
 */

void listInit( PList pList )
{
  pList->pNext     = pList;
  pList->pPrevious = pList;
}
/*
 *  Remove an element from the list
 */

void listRemove( PList pElement )
{
  pElement->pPrevious->pNext = pElement->pNext;
  pElement->pNext->pPrevious = pElement->pPrevious;
  pElement->pNext     = pElement;
  pElement->pPrevious = pElement;
}

int a2dp_allocateindex(int num)
{
	int i;
	for(i=num;i<MAX_INDEX;i++)
	{
		if(a2dpProfile[i]->used==0)
		{
			a2dpProfile[i]->used=1;
			A2dpIndex=i;
			DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "allocate device A2dpIndex is %d\r\n",1,i);
			return A2DP_SUCCESS;
		}
	}
	DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "No enough endpointID can be used\r\n",0);
	return A2DP_NO_ENDPOINTID;
}
int a2dp_getindex(TBdAddr *bd)
{
	int i;
	for(i=0;i<MAX_INDEX;i++)
	{
		if((a2dpProfile[i]->used==1)&& memcmp(&(a2dpProfile[i]->pA2DPData->link.remoteAddress), bd, BD_ADDR_SIZE)==0)
		{
			A2dpIndex=i;
			return A2DP_SUCCESS;
		}
	}
	return A2DP_NO_ENDPOINTID;
}


static void a2dp_ChangePlayerState(PA2dpProfile pProfile, TA2dpPlayerState newstate, bool forceUpdate)
{
	if (
	  (pProfile->playerState != newstate) ||
	  (forceUpdate)
	 )
	{
		switch (newstate)
		{
		case A2DP_PLAYER_STATE_PLAYING:
			{
//				TMSG_A2DP_PLAY_IND msg;
			/*	int bitRate = 0;

				bitRate =
					a2dp_CalcSBCBitRate(pProfile,
					&(pProfile->pA2DPData->sinkStream.config),
					pProfile->pA2DPData->sinkStream.frameSize);*/
#if 0
				msg.channels          = pProfile->u.a2dpData.channels;
				msg.samplingRate      = pProfile->u.a2dpData.samplingFreq;
				msg.negotiatedQuality = MPA_A2DP_QUALITY_LOW;
				if (bitRate > 100000 )
				{
					msg.negotiatedQuality = MPA_A2DP_QUALITY_MEDIUM;
				}
				if (bitRate > 250000 )
				{
					msg.negotiatedQuality = A2DP_QUALITY_HIGH;
				}

				if (pProfile->u.a2dpData.negotiatedQuality != msg.negotiatedQuality)
				{
					pProfile->u.a2dpData.negotiatedQuality = msg.negotiatedQuality;
#if 0	 //FIXMEE	  
					mpaSendMessage(
					  pProfile,
					  MPA_A2DP_PLAY_IND,
					  MPA_STATUS_SUCCESS,
					  TRUE,
					  (uint8_t *)&msg, sizeof(msg)
					  );
#endif
				}
#endif				
			}
			break;
		case A2DP_PLAYER_STATE_PAUSED:
			pProfile->negotiatedQuality = 0xff; /* set to undefined value */
			//FIXMEEE     mpaSendStatusMessage(pProfile, MPA_A2DP_PAUSE_IND, MPA_STATUS_SUCCESS);
			break;
		case A2DP_PLAYER_STATE_STOPPED:
			pProfile->negotiatedQuality = 0xff; /* set to undefined value */
//			mpaSendStatusMessage(pProfile, MPA_A2DP_STOP_IND, MPA_STATUS_SUCCESS);
			break;
		}
		pProfile->playerState = newstate;
		int msg=newstate;
		pProfile->AppA2dpCallback(&msg,pProfile->pA2DPData->link.remoteAddress,PLAYER_STATE);
	}
}

void a2dp_ChangeStreamState( PA2dpProfile pProfile, TA2dpStreamState newstate)
{
	//char *s = "";
	if (pProfile->pA2DPData->streamState != newstate)
	{
		switch (newstate)
		{
		case AVDTP_STREAM_STATE_IDLE:
			//s = "idle";
			a2dp_ChangePlayerState(pProfile, A2DP_PLAYER_STATE_STOPPED, FALSE);
//			pProfile->pA2DPData->link.streams[0].bLinkHandle = 0;
			break;
		case AVDTP_STREAM_STATE_CONFIGURED:
			//s = "configured";
			break;
		case AVDTP_STREAM_STATE_OPEN:
			//s = "open";
			pProfile->outgoingCall = FALSE;
			//a2dp_ClearAllBuffers( pProfile);
			break;
		case AVDTP_STREAM_STATE_STARTED:
			//s = "started";
			pProfile->pA2DPData->sinkStream.lastSequenceNumberIn   = -1;
			//a2dp_ClearAllBuffers( pProfile);
			break;
		case AVDTP_STREAM_STATE_CLOSING:
			//s = "closing";
			break;
		case AVDTP_STREAM_STATE_ABORTING:
			//s = "aborting";
			break;
		}
//		debugOut(dbTrace, MPA_A2DP_DEBUG_HEADER"a2dp_ChangeStreamState: %s", s);
		pProfile->pA2DPData->streamState = newstate;
	}
}


static void a2dp_OnGetCapabilitiesCmd(PA2dpProfile pProfile, TAvdtpMessage* pMsg,  uint16_t channel)
{
	uint8_t data[4 + A2DP_LOSC];
	TA2dpCapabilities* pCaps;
	if(pProfile->pA2DPData->link.role== A2DP_ROLE_SINK)
		pCaps = &(pProfile->pA2DPData->sinkStream.localCaps);
	else if(pProfile->pA2DPData->link.role== A2DP_ROLE_SOURCE)
		pCaps = &(pProfile->pA2DPData->sourceStream.localCaps);

	data[0] = MEDIA_TRANSPORT;
	data[1] = 0;
	data[2] = MEDIA_CODEC;
	data[3] = A2DP_LOSC;
	data[4] = AVDTP_MEDIA_TYPE_AUDIO << 4;
	data[5] = A2DP_CODEC_SBC;
	data[6] = pCaps->samplingFrequency | pCaps->channelMode;
	data[7] = pCaps->blockNumber | pCaps->subbandNumber | pCaps->allocMethod;
	data[8] = pCaps->minBitpool;
	data[9] = pCaps->maxBitpool;
	avdtp_SendGetCapsResp(pMsg->pLink, pMsg->transactId, 4 + A2DP_LOSC, data,  channel);
}

static void a2dp_ConfigToParams(PA2dpProfile pProfile, TSBCParams* pParams, uint8_t * config)
{
	if (config[0] & 0x80)
	{
		pParams->samplingFrequency = SBC_FREQU16000;
	}
	else if (config[0] & 0x40)
	{
		pParams->samplingFrequency = SBC_FREQU32000;
	}
	else if (config[0] & 0x20)
	{
		pParams->samplingFrequency = SBC_FREQU44100;
	}
	else if (config[0] & 0x10)
	{
		pParams->samplingFrequency = SBC_FREQU48000;
	}

	if (config[0] & 0x8)
	{
		pParams->channelMode = SBC_MODE_MONO;
	}
	else if (config[0] & 0x4)
	{
		pParams->channelMode = SBC_MODE_DUAL;
	}
	else if (config[0] & 0x2)
	{
		pParams->channelMode = SBC_MODE_STEREO;
	}
	else if (config[0] & 0x1)
	{
		pParams->channelMode = SBC_MODE_JOINT;
	}

	if (config[1] & 0x80)
	{
		pParams->blockNumber = SBC_BLOCKS4;
	}
	else if (config[1] & 0x40)
	{
		pParams->blockNumber = SBC_BLOCKS8;
	}
	else if (config[1] & 0x20)
	{
		pParams->blockNumber = SBC_BLOCKS12;
	}
	else if (config[1] & 0x10)
	{
		pParams->blockNumber = SBC_BLOCKS16;
	}

	if (config[1] & 0x8)
	{
		pParams->subbandNumber = SBC_SUBBANDS4;
	}
	else if (config[1] & 0x4)
	{
		pParams->subbandNumber = SBC_SUBBANDS8;
	}

	if (config[1] & 0x2)
	{
		pParams->allocMethod = SBC_ALLOCSNR;
	}
	else if (config[1] & 0x1)
	{
		pParams->allocMethod = SBC_ALLOCLOUDNESS;
	}

	pParams->bitpool = config[3]; /**< set to min bitpool*/
}

static int a2dp_CalcSBCFrameSize(PA2dpProfile pProfile, TSBCParams* pParams)
{
	int fs = 4, sb, bb;
	sb = 4 * (pParams->subbandNumber + 1);
	bb = 4 * (pParams->blockNumber + 1) * pParams->bitpool;

	switch (pParams->channelMode)
	{
	case SBC_MODE_MONO:
		fs += (sb >>1 ) + (bb >> 3);
		break;

	case SBC_MODE_DUAL:
		fs += sb + (bb >> 2);
		break;

	case SBC_MODE_STEREO:
		fs += sb + (bb >> 3);
		break;

	case SBC_MODE_JOINT:
		fs += sb + ((sb + bb) >> 3);
		if (sb == 4)
		{
			fs += 1;
		}
		break;
	}
	return fs;
}
/**
* @brief  a2dp calculate SBC samples per Frame
* 
* @param  pProfile
* @param  pParams
*
* @return  
*
*/
STATIC int a2dp_CalcSBCSamplesPerFrame(PA2dpProfile pProfile, TSBCParams* pParams)
{
	int need = 4 * (pParams->subbandNumber + 1) * 4 * (pParams->blockNumber + 1);
	if (! pParams->channelMode == SBC_MODE_MONO)
	{
		need <<= 1;
	}
	return need;
}
STATIC void a2dp_CalculateFramesizes(PA2dpProfile pProfile)
{
  pProfile->pA2DPData->sourceStream.framesPerPacket =
    ( pProfile->pA2DPData->sourceStream.rtpBufferSize -
      A2DP_RTP_OFFSET -
      A2DP_RTP_HEADER_SIZE )
      / pProfile->pA2DPData->sourceStream.frameSize;
  pProfile->pA2DPData->sourceStream.timePerPacket =
    pProfile->pA2DPData->sourceStream.samplesPerFrame *
    pProfile->pA2DPData->sourceStream.framesPerPacket / 2;
//  pProfile->u.a2dpData.pA2DPData->sourceStream.unprocessedFrames = pProfile->u.a2dpData.pA2DPData->sourceStream.framesPerPacket;
   // sbcInitEncoder();
}

static int a2dp_CalcSBCBitRate(PA2dpProfile pProfile, TSBCParams* pParams, int frameSize)
{
	int rate;
	int frequ;
	int bands;
	int blocks;

	switch(pParams->samplingFrequency)
	{
	case SBC_FREQU16000:
		frequ = 16000;
		break;
	case SBC_FREQU32000:
		frequ = 32000;
		break;
	case SBC_FREQU44100:
		frequ = 44100;
		break;
	case SBC_FREQU48000:
		frequ = 48000;
		break;
	}

	bands = (pParams->subbandNumber + 1) * 4;
	blocks = (pParams->blockNumber + 1) * 4;

	rate = (int) (8 * frameSize * frequ / bands / blocks);
//	debugOut(dbTrace, MPA_A2DP_DEBUG_HEADER"a2dp_CalcSBCBitRate() bitrate %d, frequ %d, bands %d, blocks %d", rate, frequ, bands, blocks);

	return rate;
}

static void a2dp_SetConfiguration(PA2dpProfile pProfile, bool reconfigure, uint16_t channel)
{
	TSBCParams sbcParams = {0};
	TA2dpSBCCapabilities   *pSBCCapabilities	= &pProfile->remoteDevice.sbc_capabilities;
	TA2dpSupportedCapabilities *pRemoteCapabilities = &pProfile->remoteDevice.capabilities;
	TA2dpSupportedCapabilities *pMyCapabilities 	= &pProfile->capabilities;

	if (
		  pRemoteCapabilities->supported_sampling_frequencies &
		  pMyCapabilities->supported_sampling_frequencies &
		  A2DP_SUPPORTED_SAMPLING_FREQ_44100HZ)
	{
		sbcParams.samplingFrequency = SBC_FREQU44100;
	}
	else if (
		  pRemoteCapabilities->supported_sampling_frequencies &
		  pMyCapabilities->supported_sampling_frequencies &
		  A2DP_SUPPORTED_SAMPLING_FREQ_48000HZ)
	{
		sbcParams.samplingFrequency = SBC_FREQU48000;
	}
	else if (
		  pRemoteCapabilities->supported_sampling_frequencies &
		  pMyCapabilities->supported_sampling_frequencies &
		  A2DP_SUPPORTED_SAMPLING_FREQ_32000HZ)
	{
		sbcParams.samplingFrequency = SBC_FREQU32000;
	}
	else if (
		  pRemoteCapabilities->supported_sampling_frequencies &
		  pMyCapabilities->supported_sampling_frequencies &
		  A2DP_SUPPORTED_SAMPLING_FREQ_16000HZ)
	{
		sbcParams.samplingFrequency = SBC_FREQU16000;
	}

	if		((pSBCCapabilities->sbc_blocks & 0x10) == 0x10)
	{
		sbcParams.blockNumber		= SBC_BLOCKS16;
	}
	else if ((pSBCCapabilities->sbc_blocks & 0x20) == 0x20)
	{
		sbcParams.blockNumber		= SBC_BLOCKS12;
	}
	else if ((pSBCCapabilities->sbc_blocks & 0x40) == 0x40)
	{
		sbcParams.blockNumber		= SBC_BLOCKS8;
	}
	else
	{
		sbcParams.blockNumber		= SBC_BLOCKS4;
	}

	// set subbbands
	if		((pSBCCapabilities->sbc_blocks & 0x04) == 0x04)
	{
		sbcParams.subbandNumber 	= SBC_SUBBANDS8;
	}
	else
	{
		sbcParams.subbandNumber 	= SBC_SUBBANDS4;
	}


	if (
		  pRemoteCapabilities->supported_channels &
		  pMyCapabilities->supported_channels &
		  A2DP_CHANNELS_STEREO
	   )
	{
		if ((pSBCCapabilities->sbc_channelmodes & 0x02) == 0x02)
		{
			sbcParams.channelMode		= SBC_MODE_STEREO;
		}
		else if ((pSBCCapabilities->sbc_channelmodes & 0x01) == 0x01)
		{
			sbcParams.channelMode		= SBC_MODE_JOINT;
		}
		else if ((pSBCCapabilities->sbc_channelmodes & 0x04) == 0x04)
		{
			sbcParams.channelMode		= SBC_MODE_DUAL;
		}
	}
	else /** Must be mono in any case*/
	{
		sbcParams.channelMode		= SBC_MODE_MONO;
	}

	if ((pSBCCapabilities->sbc_blocks & 0x2) == 0x2)
	{
		sbcParams.allocMethod		= SBC_ALLOCSNR;
	}
	else
	{
		sbcParams.allocMethod		= SBC_ALLOCLOUDNESS;
	}


	if ( pProfile->outgoingCall )
	{
		sbcParams.bitpool = pSBCCapabilities->sbc_maxbitpool;
	}
	else
	{
		/* TODO: Calculate the bitpool value from 3 target bitrates (i.e. 75 kbps, 200 kbps, 325 kbps)*/
		switch (pProfile->requestedQuality)
		{
		case A2DP_QUALITY_HIGH:
			sbcParams.bitpool			  = pSBCCapabilities->sbc_maxbitpool;
			break;
		case A2DP_QUALITY_MEDIUM:
			sbcParams.bitpool			  = 35;
			break;
		default:
			sbcParams.bitpool			  = 8; /**< will recheck below*/
			break;
		}
	}

	if (sbcParams.bitpool  < pSBCCapabilities->sbc_minbitpool)
	{
		sbcParams.bitpool  = pSBCCapabilities->sbc_minbitpool;
	}
	if (sbcParams.bitpool  > 53) /**< limit bitrate */
	{
		sbcParams.bitpool  = 53;
	}
	if (sbcParams.bitpool  > pSBCCapabilities->sbc_maxbitpool)
	{
		sbcParams.bitpool  = pSBCCapabilities->sbc_maxbitpool;
	}

	{
		uint8_t data[4 + A2DP_LOSC];
		if(pProfile->pA2DPData->link.streams[0].role==(AVDTP_ROLE_INT | AVDTP_ROLE_SRC))
			pProfile->pA2DPData->link.role=A2DP_ROLE_SOURCE;
		else
			pProfile->pA2DPData->link.role=A2DP_ROLE_SINK;		
		if (pProfile->pA2DPData->link.role == A2DP_ROLE_SOURCE)
		{
			pProfile->pA2DPData->sourceStream.config = sbcParams;
			pProfile->pA2DPData->sourceStream.frameSize =
				a2dp_CalcSBCFrameSize(pProfile, &(pProfile->pA2DPData->sourceStream.config));
						pProfile->pA2DPData->sourceStream.bitRate =
				a2dp_CalcSBCBitRate(pProfile, &(pProfile->pA2DPData->sourceStream.config),
						pProfile->pA2DPData->sourceStream.frameSize);
			pProfile->pA2DPData->sourceStream.samplesPerFrame = 
				a2dp_CalcSBCSamplesPerFrame(
						pProfile,
						&(pProfile->pA2DPData->sourceStream.config));
			pProfile->pA2DPData->sourceStream.framesPerPacket =
				( pProfile->pA2DPData->sourceStream.rtpBufferSize -
				 A2DP_RTP_OFFSET -
				 A2DP_RTP_HEADER_SIZE
				)
				 / pProfile->pA2DPData->sourceStream.frameSize;
			pProfile->pA2DPData->sourceStream.timePerPacket =
				pProfile->pA2DPData->sourceStream.samplesPerFrame *
				pProfile->pA2DPData->sourceStream.framesPerPacket / 2;
			if (pProfile->pA2DPData->sourceStream.config.channelMode == SBC_MODE_MONO)
			{
				pProfile->pA2DPData->sourceStream.timePerPacket >>= 1;
			}
			pProfile->pA2DPData->sourceStream.packetPending = FALSE;
			pProfile->pA2DPData->sourceStream.demandPending = FALSE;
			pProfile->pA2DPData->sourceStream.sampleBufferReadOffset = 0;
			pProfile->pA2DPData->sourceStream.rtpBufferOffset = A2DP_RTP_OFFSET +
				 A2DP_RTP_HEADER_SIZE;
			data[6] = 1 << (7 - pProfile->pA2DPData->sourceStream.config.samplingFrequency);
			data[6] |= 1 << (3 - pProfile->pA2DPData->sourceStream.config.channelMode);
			data[7] = 1 << (7 - pProfile->pA2DPData->sourceStream.config.blockNumber);
			data[7] |= 1 << (3 - pProfile->pA2DPData->sourceStream.config.subbandNumber);
			data[7] |= 1 + pProfile->pA2DPData->sourceStream.config.allocMethod;
			data[8] = /*0x2;*/ pProfile->pA2DPData->sourceStream.config.bitpool;
			data[9] = /*0xfa;*/ pProfile->pA2DPData->sourceStream.config.bitpool;
		}
	  else
		{
			pProfile->pA2DPData->sinkStream.config = sbcParams;
			pProfile->pA2DPData->sinkStream.frameSize =
				a2dp_CalcSBCFrameSize(pProfile, &(pProfile->pA2DPData->sinkStream.config));
			pProfile->pA2DPData->sinkStream.bitRate =
				a2dp_CalcSBCBitRate(pProfile, &(pProfile->pA2DPData->sinkStream.config),
			pProfile->pA2DPData->sinkStream.frameSize);
			data[6] = 1 << (7 - pProfile->pA2DPData->sinkStream.config.samplingFrequency);
			data[6] |= 1 << (3 - pProfile->pA2DPData->sinkStream.config.channelMode);
			data[7] = 1 << (7 - pProfile->pA2DPData->sinkStream.config.blockNumber);
			data[7] |= 1 << (3 - pProfile->pA2DPData->sinkStream.config.subbandNumber);
			data[7] |= 1 + pProfile->pA2DPData->sinkStream.config.allocMethod;
			data[8] = /*0x2;*/ pProfile->pA2DPData->sinkStream.config.bitpool;			
			data[9] = /*0xfa;*/ pProfile->pA2DPData->sinkStream.config.bitpool;
		}
	  data[0] = MEDIA_TRANSPORT;
	  data[1] = 0;
	  data[2] = MEDIA_CODEC;
	  data[3] = A2DP_LOSC;
	  data[4] = AVDTP_MEDIA_TYPE_AUDIO << 4;
	  data[5] = A2DP_CODEC_SBC;

	  avdtp_SetConfiguration(&pProfile->pA2DPData->link, 0,
			4 + A2DP_LOSC, data, reconfigure, channel);
	}
}


static void a2dp_OnCapabilitiesInd(PA2dpProfile pProfile, uint8_t * pData, uint16_t channel)
{
	TA2dpSBCCapabilities   *pSBCCapabilities	= &pProfile->remoteDevice.sbc_capabilities;
	TA2dpSupportedCapabilities *pRemoteCapabilities = &pProfile->remoteDevice.capabilities;
//	TMPA_A2DP_Capabilities *pMyCapabilities 	= &pProfile->capabilities;

	pSBCCapabilities->sbc_channelmodes = pData[0];
	pSBCCapabilities->sbc_blocks	   = pData[1];
	pSBCCapabilities->sbc_minbitpool   = pData[2];
	pSBCCapabilities->sbc_maxbitpool   = pData[3];

	/** set supported channel number */
	pRemoteCapabilities->supported_channels = 0;
	if ((pSBCCapabilities->sbc_channelmodes & 0x08) != 0x00)
	{
		pRemoteCapabilities->supported_channels |= A2DP_CHANNELS_MONO;
	}
	if ((pSBCCapabilities->sbc_channelmodes & 0x07) != 0x00)
	{
		pRemoteCapabilities->supported_channels |= A2DP_CHANNELS_STEREO;
	}

	/** set supported qualities*/
	pRemoteCapabilities->supported_qualities = A2DP_QUALITY_HIGH; 

	/** set supported sampling frequencies*/
	pRemoteCapabilities->supported_sampling_frequencies = 0;
	if ((pSBCCapabilities->sbc_channelmodes & 0x80) == 0x80)
	{
		pRemoteCapabilities->supported_sampling_frequencies |=
					A2DP_SUPPORTED_SAMPLING_FREQ_16000HZ;
	}
	if ((pSBCCapabilities->sbc_channelmodes & 0x40) == 0x40)
	{
		pRemoteCapabilities->supported_sampling_frequencies |=
					A2DP_SUPPORTED_SAMPLING_FREQ_32000HZ;
	}
	if ((pSBCCapabilities->sbc_channelmodes & 0x20) == 0x20)
	{
		pRemoteCapabilities->supported_sampling_frequencies |=
					A2DP_SUPPORTED_SAMPLING_FREQ_44100HZ;
	}
	if ((pSBCCapabilities->sbc_channelmodes & 0x10) == 0x10)
	{
		pRemoteCapabilities->supported_sampling_frequencies |=
					A2DP_SUPPORTED_SAMPLING_FREQ_48000HZ;
	}

	if ( pProfile->outgoingCall ) /**connect from local*/
	{
		a2dp_SetConfiguration(pProfile, FALSE, channel);
	}
}

void a2dp_HandleSuspendStreamInd(PA2dpProfile pProfile)
{
	a2dp_ChangePlayerState(pProfile, A2DP_PLAYER_STATE_PAUSED, FALSE);
}

static bool a2dp_OnSetConfigurationCmd(PA2dpProfile pProfile, TAvdtpMessage* pMsg, uint16_t channel)
{
	int rest = pMsg->dataLength;
	uint8_t * pData = pMsg->data;
	//int StreamIndex=a2dp_FindAcpID( pProfile);
	pProfile->pA2DPData->link.streams[0].used=1;
	pMsg->pLink->streams[0].remoteEndPointId = pMsg->data[1] >> 2; /**< uint8_t SEID - remote SEID */
	pData += 2; /**< skip int / acp seid */

	/** iterate through sevice capabilities information elements */
	while (rest > 0)
	{
		uint8_t losc = pData[1];
		if (pData[0] == MEDIA_CODEC && losc == A2DP_LOSC)
		{
			if (pData[2] >> 4 == AVDTP_MEDIA_TYPE_AUDIO && pData[3] == A2DP_CODEC_SBC)
			{
			/** a2dp sbc configuration detected */
//#if (F_BT_MPA_A2DP_SINK)
				if (pProfile->pA2DPData->link.role == A2DP_ROLE_SINK)
				{
					a2dp_ConfigToParams(pProfile, &(pProfile->pA2DPData->sinkStream.config), &pData[4]);
					pProfile->pA2DPData->sinkStream.frameSize =
					a2dp_CalcSBCFrameSize(pProfile, &(pProfile->pA2DPData->sinkStream.config));
					pProfile->pA2DPData->sinkStream.bitRate =
					a2dp_CalcSBCBitRate(pProfile, &(pProfile->pA2DPData->sinkStream.config), pProfile->pA2DPData->sinkStream.frameSize);
					a2dp_OnCapabilitiesInd( pProfile, &pData[4], channel);
				}
//#endif
				return 0;
			}
		}
		pData += losc + 2;
		rest -= losc + 2;
	}
	return 0;
}


STATIC void mpaA2dpHandleReconfigureResp(PA2dpProfile pProfile, uint16_t channel)
{
  if (pProfile->streamLink.state == linkStateConnecting)
  {
    //debugOut(dbTrace, MPA_A2DP_DEBUG_HEADER"a2dp_StartStream()");
    avdtp_StartStream(&pProfile->pA2DPData->link, 0, channel);
  }
}
/**
* @brief  a2dp on reconfig command
* 
* @param  pProfile
* @param  pMsg
*
* @return  
*
*/
STATIC void a2dp_OnReconfigurationCmd(PA2dpProfile pProfile, TAvdtpMessage* pMsg, uint16_t channel)
{
	int rest = pMsg->dataLength;
	uint8_t * pData = pMsg->data;


	/* iterate through sevice capabilities information elements */
	while (rest > 0)
	{
		uint8_t losc = pData[1];
		if (pData[0] == MEDIA_CODEC && losc == A2DP_LOSC)
		{
			if (pData[2] >> 4 == AVDTP_MEDIA_TYPE_AUDIO && pData[3] == A2DP_CODEC_SBC)
			{
				/* a2dp sbc configuration detected */
				if (pProfile->pA2DPData->link.role == A2DP_ROLE_SINK)
				{
					a2dp_ConfigToParams(pProfile, &(pProfile->pA2DPData->sinkStream.config), &pData[4]);
					pProfile->pA2DPData->sinkStream.frameSize =
					a2dp_CalcSBCFrameSize(pProfile, &(pProfile->pA2DPData->sinkStream.config));
					pProfile->pA2DPData->sinkStream.bitRate =
					a2dp_CalcSBCBitRate(pProfile, &(pProfile->pA2DPData->sinkStream.config), pProfile->pA2DPData->sinkStream.frameSize);
					a2dp_OnCapabilitiesInd( pProfile, &pData[4], channel);
				}
				return;
			}
		}
		pData += losc + 2;
		rest -= losc + 2;
	}
}


bool avdtp_SendStreamDataResp(TAvdtpLink* pLink, int streamIndex)
{
#if 0  //FIXMEEE HOW to release buffer
//	blueFaceStatus result = blueFaceNoError;
	LPbtLink		pbtLink = pLink->streams[streamIndex].bLinkHandle;

	osBufferRelease(pbtLink->usBufferAddress[pbtLink->dataRespPos]);
	if (++pbtLink->dataRespPos >= BT_MAXOPENINDICATIONS)
	{
		pbtLink->dataRespPos = 0;
	}

	pbtLink->openDataResponses--;
#endif
	return true;
}

/* Get a SBC buffer from the queue */
static TAvdtpMessage *a2dp_DequeueSBCBuffer(PA2dpProfile pProfile)
{
	TAvdtpMessage *pAVDTPMessage = NULL;

	if (pProfile->pA2DPData->sinkStream.AVDTPMessageQueueCount > 0)
	{
		/* get a copy of the message from the queue */
		pAVDTPMessage =
		  &pProfile->pA2DPData->sinkStream.AVDTPMessageQueue[pProfile->pA2DPData->sinkStream.AVDTPMessageQueueReadIndex];

		pProfile->pA2DPData->sinkStream.AVDTPMessageQueueCount--;
		pProfile->pA2DPData->sinkStream.AVDTPMessageQueueReadIndex++;

		/* Check index overflow */
		pProfile->pA2DPData->sinkStream.AVDTPMessageQueueReadIndex %= A2DP_SBC_BUFFER_QUEUE_SIZE;
	}

	return pAVDTPMessage;
}

static void a2dp_ClearSBCBufferQueue(PA2dpProfile pProfile)
{
	TAvdtpMessage *pMsg;

	while ((pMsg = a2dp_DequeueSBCBuffer(pProfile)) != NULL)
	{
		/* Tell lower layer that the buffer can be reused */
		avdtp_SendStreamDataResp(pMsg->pLink, pMsg->streamIndex);
		pProfile->pA2DPData->sinkStream.pendingDataResponses--;
	};
	pProfile->pA2DPData->sinkStream.AVDTPMessageQueueReadIndex  = 0;
	pProfile->pA2DPData->sinkStream.AVDTPMessageQueueWriteIndex = 0;
	pProfile->pA2DPData->sinkStream.AVDTPMessageQueueCount      = 0;
	pProfile->pA2DPData->sinkStream.CurrentSBCBufferOffset      = 0;
	pProfile->pA2DPData->sinkStream.pCurrentAVDTPMessage        = NULL;

#if  0
	/* also send deferred data responses */
	if (pProfile->u.a2dpData.pA2DPData->sinkStream.pendingDataResponses > 0)
	{
	while(pProfile->u.a2dpData.pA2DPData->sinkStream.pendingDataResponses--)
	{
	  if (pProfile->u.a2dpData.pA2DPData->sinkStream.pLink)
	  {
	    avdtp_SendStreamDataResp(
	      pProfile->u.a2dpData.pA2DPData->sinkStream.pLink,
	      pProfile->u.a2dpData.pA2DPData->sinkStream.streamIndex);
	  }
	}
	}
#endif
	pProfile->pA2DPData->sinkStream.pendingDataResponses = 0;

}

static uint8_t * a2dp_DequeuePCMBuffer(PA2dpProfile pProfile)
{
	uint8_t * pBuffer = NULL;
	if (pProfile->pA2DPData->sinkStream.DeliveredPCMBufferQueue.pNext != NULL)
	{
		if (!LIST_IS_EMPTY(&pProfile->pA2DPData->sinkStream.DeliveredPCMBufferQueue))
		{
			pBuffer = (uint8_t *)LIST_QUEUE_OUT(&pProfile->pA2DPData->sinkStream.DeliveredPCMBufferQueue);
		}
	}

	return pBuffer;
}

static void a2dp_ReleasePCMBuffer(PA2dpProfile pProfile, uint8_t * pBuffer)
{
	if (pBuffer)
	{
		osMemoryFree(pBuffer);
	}
}

/* Clear PCM buffer queue */
static void a2dp_ClearDeliveredPCMBufferQueue(PA2dpProfile pProfile)
{
	uint8_t * pBuffer;

	while (!LIST_IS_EMPTY(&pProfile->pA2DPData->sinkStream.DeliveredPCMBufferQueue))
	{
		pBuffer = a2dp_DequeuePCMBuffer(pProfile);
		a2dp_ReleasePCMBuffer( pProfile, pBuffer);
	};

	listInit(&pProfile->pA2DPData->sinkStream.DeliveredPCMBufferQueue);
}

static void a2dp_ClearAllBuffers( PA2dpProfile pProfile)
{
	//a2dp sink
	a2dp_ClearSBCBufferQueue( pProfile);
	a2dp_ClearDeliveredPCMBufferQueue( pProfile);
}

void a2dp_ChangeSignalLinkState(PA2dpProfile pProfile, TLinkState newState, T_Status status)
{
#if 1
	if (pProfile->signalLink.state != newState)
	{
		switch (newState)
		{
		case linkStateIdle:
			/** we are not playing anymore*/
			a2dp_ChangePlayerState(pProfile, A2DP_PLAYER_STATE_STOPPED, FALSE);
			a2dp_ClearAllBuffers( pProfile);
			//mpaSendDisconnectIndication(pProfile, status);
			pProfile->ActiveDisconnect = FALSE;
			break;
		case linkStateSearchingFirstBond:
			break;
		case linkStateSearchingChannel:
			break;
		case linkStateWaitforConnectConfirmation:
			break;
		case linkStateIncomingCall:
			break;
		case linkStateConnecting:
			break;
		case linkStateConnected:
			//mpaSendStatusMessage(pProfile, MPA_A2DP_CONNECT_ACT_IND, status);
			a2dp_ChangePlayerState(pProfile, A2DP_PLAYER_STATE_STOPPED, FALSE);
			break;
		case linkStateActiveDisconnect:
			break;
		case linkStateDisconnecting:
			break;
		default:
			break;
		}
		pProfile->signalLink.state = newState;
	}
#endif	
}

void a2dp_ChangeStreamLinkState(PA2dpProfile pProfile, TLinkState newState)
{
	if (pProfile->streamLink.state != newState)
	{
		switch (newState)
		{
		case linkStateIdle:
#if (0)
			/* Release RTP Buffer */
			if (pProfile->u.a2dpData.role == MPA_A2DP_ROLE_SOURCE)
			{
				if (pProfile->u.a2dpData.pA2DPData->sourceStream.pRtpBuffer)
				{
					osBufferRelease( pProfile->u.a2dpData.pA2DPData->sourceStream.pRtpBuffer);
				}
				pProfile->u.a2dpData.pA2DPData->sourceStream.rtpBufferSize = 0;
				pProfile->u.a2dpData.pA2DPData->sourceStream.pRtpBuffer    = NULL;
			}
#endif
			//a2dp_ClearAllBuffers( pProfile);

//timer how to solve it			mpaTimerStop(pProfile, MPA_A2DP_TIMER_CONNECT_STREAM);
//FIXMEE  TIMER TIMER TIMER			mpaTimerStop(pProfile, MPA_A2DP_TIMER_DISCONNECT_STREAM);

			/*if (pProfile->ActiveDisconnect)
			{
				pProfile->ActiveDisconnect = FALSE;
				 Disconnect signal link, too 
				if (pProfile->signalLink.state != linkStateIdle)
				{
					a2dp_ChangeSignalLinkState( pProfile, linkStateDisconnecting, MPA_STATUS_SUCCESS);
					a2dp_Disconnect(pProfile);
				}
			}*/
			break;
		case linkStateSearchingFirstBond:
			break;
		case linkStateSearchingChannel:
			break;
		case linkStateWaitforConnectConfirmation:
			break;
		case linkStateIncomingCall:
			break;
		case linkStateConnecting:
			/* start supervision timer */
			break;
		case linkStateConnected:
//TIMER			mpaTimerStop(pProfile, MPA_A2DP_TIMER_CONNECT_STREAM);
			break;
		case linkStateActiveDisconnect:
			break;
		case linkStateDisconnecting:
			/* start supervision timer */
//			mpaTimerSetMS(pProfile, MPA_A2DP_TIMER_DISCONNECT_STREAM,
//TIMER			  MPA_A2DP_DISCONNECT_STREAM_TIMEOUT);
			break;
		default:
			break;
		}
		pProfile->streamLink.state = newState;
	}
}

static void a2dp_HandleStartStreamInd(PA2dpProfile pProfile)
{
	// stream is connected -> tell application to start playing
	a2dp_ChangePlayerState(pProfile, A2DP_PLAYER_STATE_PLAYING, FALSE);
}

static bool a2dp_QueueSBCBuffer(PA2dpProfile pProfile, TAvdtpMessage *pAVDTPMessage)
{
	bool result = FALSE;
	//bool sequencenumberOK = TRUE;

	if (pProfile->pA2DPData->sinkStream.AVDTPMessageQueueCount < A2DP_SBC_BUFFER_QUEUE_SIZE)
	{
		int sequencenumber;
		TA2dpRTPHeader *pRtpHeader =
			(TA2dpRTPHeader *)pAVDTPMessage->data;

		sequencenumber = (pRtpHeader->SequenceNumber1 << 8) + pRtpHeader->SequenceNumber0;

		if (pProfile->pA2DPData->sinkStream.lastSequenceNumberIn >= 0)
		{
#if 0
			debugOut(dbTrace, MPA_A2DP_DEBUG_HEADER"a2dp_QueueSBCBuffer(): sequence number 0x%04x",
				sequencenumber);
#endif
			if (pProfile->pA2DPData->sinkStream.lastSequenceNumberIn+1 != sequencenumber)
			{
			#if 0
				/* wrong sequence number -> ignore complete frame */
				debugOut(dbTrace,
					MPA_A2DP_DEBUG_HEADER
					"!! a2dp_QueueSBCBuffer(): wrong RTP sequence number %d, should be %d",
					sequencenumber,
					pProfile->u.a2dpData.pA2DPData->sinkStream.lastSequenceNumberIn+1);
			#endif
				sequencenumber = -1;
				//sequencenumberOK = FALSE;
			}
		}
		pProfile->pA2DPData->sinkStream.lastSequenceNumberIn = sequencenumber;

		/* if (sequencenumberOK) */ /* process anyway -> */
		{
			/* put a copy of the message in the queue */
			pProfile->pA2DPData->sinkStream.AVDTPMessageQueue[pProfile->pA2DPData->sinkStream.AVDTPMessageQueueWriteIndex] =
			*pAVDTPMessage;
			pProfile->pA2DPData->sinkStream.AVDTPMessageQueueCount++;

#if 0
			debugOut(dbTrace,
				MPA_A2DP_DEBUG_HEADER"a2dp_QueueSBCBuffer: addr 0x%08x, size 0x%x entries %d write %d read %d",
				  pAVDTPMessage->data,
				  pAVDTPMessage->dataLength,
				  pProfile->u.a2dpData.pA2DPData->sinkStream.AVDTPMessageQueueCount,
				  pProfile->u.a2dpData.pA2DPData->sinkStream.AVDTPMessageQueueWriteIndex,
				  pProfile->u.a2dpData.pA2DPData->sinkStream.AVDTPMessageQueueReadIndex
				  );
#endif

			/* Check index overflow */
			pProfile->pA2DPData->sinkStream.AVDTPMessageQueueWriteIndex++;
			pProfile->pA2DPData->sinkStream.AVDTPMessageQueueWriteIndex %= A2DP_SBC_BUFFER_QUEUE_SIZE;

			result = TRUE;
		}
	}
	else
	{
		/* queue full -> discard and ignore data packet */
//		debugOut(dbTrace, MPA_A2DP_DEBUG_HEADER"!! a2dp_QueueSBCBuffer: queue full");
	}

	return result;
}

uint8_t * a2dp_GetNextSBCBuffer(PA2dpProfile pProfile)
{
	uint8_t * pSBCBuffer = NULL;
	/* check if we need to pull an SBC frame from the queue */
	if (pProfile->pA2DPData->sinkStream.pCurrentAVDTPMessage == NULL)
	{
		pProfile->pA2DPData->sinkStream.pCurrentAVDTPMessage = a2dp_DequeueSBCBuffer(pProfile);
		if (pProfile->pA2DPData->sinkStream.pCurrentAVDTPMessage != NULL)
		{
			/* Forward offset counter by RTP header size */
			pProfile->pA2DPData->sinkStream.CurrentSBCBufferOffset =
				A2DP_RTP_HEADER_SIZE +
				((pProfile->pA2DPData->sinkStream.pCurrentAVDTPMessage->data[0]
				  & A2DP_RTP_HEADER_CSRC_COUNT_MASK) * 4);
			pSBCBuffer =
				&pProfile->pA2DPData->sinkStream.pCurrentAVDTPMessage->data
				    [pProfile->pA2DPData->sinkStream.CurrentSBCBufferOffset];
		}
	}
	else
	{
		pSBCBuffer =
			&pProfile->pA2DPData->sinkStream.pCurrentAVDTPMessage->data
			  [pProfile->pA2DPData->sinkStream.CurrentSBCBufferOffset];
	}

#if 0
	debugOut(dbTrace, "a2dp_GetNextSBCBuffer: addr 0x%08x, offset 0x%x size 0x%x",
	pSBCBuffer,
	pProfile->u.a2dpData.pA2DPData->sinkStream.CurrentSBCBufferOffset,
	a2dp_GetCurrentSBCBufferSize(pProfile)
	);
#endif

	return (pSBCBuffer);
}

static int a2dp_GetCurrentSBCBufferSize(PA2dpProfile pProfile)
{
	int size = 0;
	if (pProfile->pA2DPData->sinkStream.pCurrentAVDTPMessage != NULL)
	{
		size = pProfile->pA2DPData->sinkStream.pCurrentAVDTPMessage->dataLength -
		       pProfile->pA2DPData->sinkStream.CurrentSBCBufferOffset;
	}
	return size;
}

static void a2dp_ForwardSBCBuffer(PA2dpProfile pProfile, int decodedBytes, uint16_t channel)
{
//    LPbtLink pLink;

#if 0
  debugOut(dbTrace, "a2dp_ForwardSBCBuffer: skip 0x%x",decodedBytes);
#endif

  pProfile->pA2DPData->sinkStream.CurrentSBCBufferOffset += decodedBytes;

  /* Check if there is another SBC frame in our buffer */
//FIXMEE  if (a2dp_GetCurrentSBCBufferSize(pProfile) < 2)
  { /* remaining part is too small */
    //TAvdtpMessage *pAVDTPMessage;
    /* Tell the lower layer that the SBC buffer can be reused */
    //pAVDTPMessage =
     // pProfile->pA2DPData->sinkStream.pCurrentAVDTPMessage;

#if 0
    debugOut(dbTrace, "avdtp_SendStreamDataResp: addr 0x%08x, pending %d",
        pAVDTPMessage->data,
        pProfile->u.a2dpData.pA2DPData->sinkStream.pendingDataResponses
        );
#endif

#if  0
    if (pProfile->u.a2dpData.pA2DPData->sinkStream.holdBackFirstDataResponse)
    {
     /* I have to remember the current link context because I have to send
        the deferred data response on clean up. I assume that the link context is the same
        for the complete session */
      pProfile->u.a2dpData.pA2DPData->sinkStream.pLink       = pProfile->u.a2dpData.pA2DPData->sinkStream.pCurrentAVDTPMessage->pLink;
      pProfile->u.a2dpData.pA2DPData->sinkStream.streamIndex = pProfile->u.a2dpData.pA2DPData->sinkStream.pCurrentAVDTPMessage->streamIndex;
    }
    else
    {
#endif
#if 1
      avdtp_SendStreamDataResp(
       pProfile->pA2DPData->sinkStream.pCurrentAVDTPMessage->pLink,
       pProfile->pA2DPData->sinkStream.pCurrentAVDTPMessage->streamIndex);
#else
		pLink = blueFaceFindLinkByHandle(pBTA, channel, PSM_AVDTP);

		osBufferRelease(pLink->usBufferAddress[pAVDTPMessage->bufReleasePos]);

#endif
      pProfile->pA2DPData->sinkStream.pendingDataResponses--;
#if  0
    }
    pProfile->u.a2dpData.pA2DPData->sinkStream.holdBackFirstDataResponse = FALSE;
#endif
#if 0
	pLink = blueFaceFindLinkByHandle(pBTA, channel, PSM_AVDTP);
	
	osBufferRelease(pLink->usBufferAddress[pAVDTPMessage->bufReleasePos]);
#endif
    /* reset my own variables */
    pProfile->pA2DPData->sinkStream.pCurrentAVDTPMessage = NULL;
    pProfile->pA2DPData->sinkStream.CurrentSBCBufferOffset = 0;
  }
}


static bool a2dp_ProcessNextSBCBuffer(PA2dpProfile pProfile, uint16_t channel)
{
	bool result = FALSE;
	if (
	/*(pProfile->commonData.state == ProfileStateIdle) &&*/
	(pProfile->streamLink.state == linkStateConnected)
	)
	{
		int decodedPCMLenght = 512; /* size of buffer needed for SBC decoding  */
		                        /* XYXY check or calculate the "512" */

		/* Get next PCM buffer if needed */
		if (pProfile->pA2DPData->sinkStream.pPCMBuffer == NULL)
		{
			pProfile->pA2DPData->sinkStream.pPCMBuffer = a2dp_AllocatePCMBuffer( pProfile);
			pProfile->pA2DPData->sinkStream.PCMBufferWriteIndex = A2DP_PCM_BUFFER_GAP;
		}

		/* Check if we have a PCM buffer to write the decoded SBC data to */
		if (pProfile->pA2DPData->sinkStream.pPCMBuffer != NULL)
		/* Check if decoded SBC data will fit in the PCM buffer */
		{
			while ((pProfile->pA2DPData->sinkStream.PCMBufferWriteIndex + decodedPCMLenght) <= A2DP_PCM_BUFFER_SIZE)
			{
				/* Get a SBC buffer from the queue */
				uint8_t * pSBCBuffer = a2dp_GetNextSBCBuffer(pProfile);
				int SBCBufferSize = a2dp_GetCurrentSBCBufferSize(pProfile);

				if (pSBCBuffer  == NULL)
				{
					break; /* exit while() */
				}
				else
				{
					/* Write the decoded SBC buffer to the PCM buffer */
					int decodedBytes;

					/* check if first byte is a SBC sync byte */
					if (pSBCBuffer[0] != 0x9c)
					{
						/* the frame doesn't start with a SBC sync byte ->
						 * skip the complete data buffer */
	//					debugOut(dbTrace, MPA_A2DP_DEBUG_HEADER"!!a2dp: no SBC sync byte found. Skipping rest of the buffer");

						decodedBytes = SBCBufferSize;
					}
					else
					{ /* sync byte is OK */

						/* evaluate bitpool value within sbc packet */
						if (pProfile->pA2DPData->sinkStream.config.bitpool != pSBCBuffer[2] )
						{

							pProfile->pA2DPData->sinkStream.config.bitpool = pSBCBuffer[2];
							pProfile->pA2DPData->sinkStream.frameSize =
								a2dp_CalcSBCFrameSize(pProfile,
								&(pProfile->pA2DPData->sinkStream.config));
							pProfile->pA2DPData->sinkStream.bitRate =
								a2dp_CalcSBCBitRate(pProfile,
								&(pProfile->pA2DPData->sinkStream.config),
								pProfile->pA2DPData->sinkStream.frameSize);

							/* Quality has changed -> Send new play indication */
							a2dp_ChangePlayerState(pProfile, A2DP_PLAYER_STATE_PLAYING, TRUE);
						}
#if 1//FIXMEE
						decodedBytes = sbcDecode(pSBCBuffer, SBCBufferSize,
						  &pProfile->pA2DPData->sinkStream.pPCMBuffer[pProfile->pA2DPData->sinkStream.PCMBufferWriteIndex],
						  &decodedPCMLenght);
#endif
						if (decodedBytes < 0)
						{
#if 0						
							debugOut(dbTrace,
								MPA_A2DP_DEBUG_HEADER
								"!!a2dp: SBC decoding error %d"
								" addr 0x%08x size 0x%x offset 0x%x len 0x%x 0x%02x%02x%02x%02x",
								decodedBytes,
								pProfile->u.a2dpData.pA2DPData->sinkStream.pCurrentAVDTPMessage->data,
								a2dpGetCurrentSBCBufferSize(pProfile),
								pProfile->u.a2dpData.pA2DPData->sinkStream.CurrentSBCBufferOffset,
								pProfile->u.a2dpData.pA2DPData->sinkStream.pCurrentAVDTPMessage->dataLength,
								pSBCBuffer[0],
								pSBCBuffer[1],
								pSBCBuffer[2],
								pSBCBuffer[3]);
#endif
						}
					}

					/* Remember that we decoded some bytes */
					a2dp_ForwardSBCBuffer(pProfile, decodedBytes, channel);

					pProfile->pA2DPData->sinkStream.PCMBufferWriteIndex += decodedPCMLenght;

					result = TRUE;
				}
			}

			if ((pProfile->pA2DPData->sinkStream.PCMBufferWriteIndex + decodedPCMLenght) > A2DP_PCM_BUFFER_SIZE)
			/* not enough space in PCM buffer for another SBC block */
			{
				/* send PCM buffer to higher layer */
				{
#if (F_BT_MPA_A2DP_SINK)
#if 0 //FIXMEEE
					mpaA2dpHandleDataInd( pProfile,
					(uint16_t)(pProfile->u.a2dpData.pA2DPData->sinkStream.PCMBufferWriteIndex - (uint16_t)A2DP_PCM_BUFFER_GAP),
					(uint16_t)(A2DP_PCM_BUFFER_GAP - A2DP_PCM_BUFFER_OFFSET),
					&pProfile->u.a2dpData.pA2DPData->sinkStream.pPCMBuffer[A2DP_PCM_BUFFER_OFFSET]);
#endif  
#endif

					/* remember that we delivered this buffer */
					//FIXMEE          a2dp_QueuePCMBuffer(pProfile, pProfile->u.a2dpData.pA2DPData->sinkStream.pPCMBuffer);
					a2dp_ReleasePCMBuffer(pProfile, pProfile->pA2DPData->sinkStream.pPCMBuffer);
					/* This buffer is not valid anymore */
					pProfile->pA2DPData->sinkStream.pPCMBuffer = NULL;
					pProfile->pA2DPData->sinkStream.PCMBufferWriteIndex = A2DP_PCM_BUFFER_GAP;
				}
			}
		} /*if (pProfile->u.a2dpData.pA2DPData->sinkStream.pPCMBuffer != NULL)*/
	}

	return result;
}

STATIC void a2dp_OnDiscoverResp(PA2dpProfile pProfile, TAvdtpMessage* pMsg, uint16_t channel)
{
  int i; 
  pProfile->pA2DPData->link.streams[0].used=1;
  for (i = 0; i < pMsg->dataLength; i += 2)
  {
    //XYXY TODO: Check my own role. Don't connect SINK-SINK or SOURCE-SOURCE
    if (!(pMsg->data[i + 1] >> 3) & AVDTP_TSEP_SNK) /* we are only inquiring sinks */
    {
      if ((pMsg->data[i + 1] >> 4) == pMsg->pLink->streams[0].mediaType)  /* matching media type */
      {
        if (! (pMsg->data[i] & 2) /* not in use */)
        {
          /* a suitable sink stream end point has been detected; save it and get it's capabilities */
          pMsg->pLink->streams[0].remoteEndPointId = pMsg->data[i] >> 2;
			avdtp_GetCapabilities(&pProfile->pA2DPData->link, 0, channel);
//          a2dp_GetCapabilities(pProfile, 0);
          break;
        }
      }
    } else
    {
      if ((pMsg->data[i + 1] >> 4) == pMsg->pLink->streams[0].mediaType)  /* matching media type */
      {
        if (! (pMsg->data[i] & 2) /* not in use */)
        {
          /* a suitable source stream end point has been detected; save it and get it's capabilities */
          pMsg->pLink->streams[0].remoteEndPointId = pMsg->data[i] >> 2;
		  avdtp_GetCapabilities(&pProfile->pA2DPData->link, 0, channel);
 //         a2dp_GetCapabilities(pProfile, 0);
          break;
        }
      }
    }
  }
}

STATIC void a2dp_OnGetCapabilitiesResp(PA2dpProfile pProfile, TAvdtpMessage* pMsg, uint16_t channel)
{
  int rest = pMsg->dataLength;
  uint8_t * pData = pMsg->data;
  /* iterate through sevice capabilities information elements */
  while (rest > 0)
  {
    uint8_t losc = pData[1];
    if (pData[0] == MEDIA_CODEC && losc == A2DP_LOSC)
    {
      if (pData[2] >> 4 == AVDTP_MEDIA_TYPE_AUDIO && pData[3] == A2DP_CODEC_SBC)
      {
        if (pProfile->pA2DPData->link.role == A2DP_ROLE_SINK)
        {
          pProfile->pA2DPData->sinkStream.remoteCaps.samplingFrequency = pData[4] & 0xf0;
          pProfile->pA2DPData->sinkStream.remoteCaps.channelMode = pData[4] & 0xf;
          pProfile->pA2DPData->sinkStream.remoteCaps.blockNumber = pData[5] & 0xf0;
          pProfile->pA2DPData->sinkStream.remoteCaps.subbandNumber = pData[5] & 0xc;
          pProfile->pA2DPData->sinkStream.remoteCaps.allocMethod = pData[5] & 0x3;
          pProfile->pA2DPData->sinkStream.remoteCaps.minBitpool = pData[6];
          pProfile->pA2DPData->sinkStream.remoteCaps.maxBitpool = pData[7];
        }
				else
				{
					pProfile->pA2DPData->sourceStream.remoteCaps.samplingFrequency = pData[4] & 0xf0;
          pProfile->pA2DPData->sourceStream.remoteCaps.channelMode = pData[4] & 0xf;
          pProfile->pA2DPData->sourceStream.remoteCaps.blockNumber = pData[5] & 0xf0;
          pProfile->pA2DPData->sourceStream.remoteCaps.subbandNumber = pData[5] & 0xc;
          pProfile->pA2DPData->sourceStream.remoteCaps.allocMethod = pData[5] & 0x3;
          pProfile->pA2DPData->sourceStream.remoteCaps.minBitpool = pData[6];
          pProfile->pA2DPData->sourceStream.remoteCaps.maxBitpool = pData[7];
				}
      }
      a2dp_OnCapabilitiesInd( pProfile, &pData[4], channel);
      return;
    }
    pData += losc + 2;
    rest -= losc + 2;
  }
}

STATIC void a2dp_HandleSetConfigurationResp(PA2dpProfile pProfile)
{
	if (pProfile->streamLink.state == linkStateConnecting)
	{
		TBtConReq tavdtpConReq;

		memset(&tavdtpConReq, 0, sizeof(TBtConReq));
		
		tavdtpConReq.frameSize = 0;
		tavdtpConReq.channel = AVDTP_CHANNEL_TYPE_STREAM;
		tavdtpConReq.RemoteSeid = pProfile->pA2DPData->link.streams[0].remoteEndPointId;
		memcpy(tavdtpConReq.bd, &pProfile->pA2DPData->link.remoteAddress, BLUE_API_BD_SIZE);
		a2dpProfile[A2dpIndex]->outgoingCall= TRUE; 
		avdtp_Connect(&tavdtpConReq);
	}
}


/**
* @brief  a2dp avdtp callback
* 
* @param  pMsg
* @param  eventcontext
*
* @return  
*
*/
void A2dpAvdtpCallback(TAvdtpMessage* pMsg, void* eventcontext, uint16_t channel)
{
//	LPMPAProfile pProfile  = (LPMPAProfile)eventcontext;
	switch(pMsg->type)
	{
	/** Handle Commands */
	case AVDTP_MESSAGE_TYPE_COMMAND:
	{ 
		switch(pMsg->signalID)
		{
		case AVDTP_DISCOVER:
			avdtp_SendDiscoverResp(pMsg->pLink, pMsg->transactId, channel);
			break;
		case AVDTP_GET_CAPABILITIES:
			a2dp_OnGetCapabilitiesCmd(a2dpProfile[A2dpIndex], pMsg, channel);
			break;
		case AVDTP_SET_CONFIGURATION:
			if (a2dpProfile[A2dpIndex]->pA2DPData->streamState == AVDTP_STREAM_STATE_IDLE)
			{
				a2dp_OnSetConfigurationCmd(a2dpProfile[A2dpIndex], pMsg, channel);	
				if((pMsg->data[0] & 0x02) == 1)
					avdtp_SendBadInUse(pMsg->pLink,AVDTP_SET_CONFIGURATION,pMsg->transactId,channel);
				else
				{
					avdtp_SendSetConfigResp(pMsg->pLink, pMsg->transactId, channel);
					a2dp_ChangeStreamState( a2dpProfile[A2dpIndex], AVDTP_STREAM_STATE_CONFIGURED);
				}	
			}
			else
			{
				avdtp_SendSetConfigReject(pMsg->pLink, pMsg->transactId, channel);
			}

			break;

		case AVDTP_GET_CONFIGURATION:
			avdtp_SendGetConfigResp(pMsg->pLink, pMsg->transactId, channel);
			break;
		
		case AVDTP_RECONFIGURE:
			if (a2dpProfile[A2dpIndex]->pA2DPData->streamState == AVDTP_STREAM_STATE_OPEN)
			{
				if((pMsg->data[0] & 0x02) == 0)
					avdtp_SendBadNotInUse(pMsg->pLink,AVDTP_RECONFIGURE,pMsg->transactId,channel);
				else	
				{
					a2dp_OnReconfigurationCmd(a2dpProfile[A2dpIndex], pMsg, channel);
					//a2dp_SendPlayInd(a2dpProfile);
					avdtp_SendReconfigureResp(pMsg->pLink, pMsg->transactId, channel);
				}
			}
			else
			{
				avdtp_SendReconfigureReject(pMsg->pLink, pMsg->transactId, channel);
			}
			break;
		
		case AVDTP_START:
			if  (a2dpProfile[A2dpIndex]->pA2DPData->streamState == AVDTP_STREAM_STATE_OPEN)
			{				
				avdtp_SendStartStreamResp(pMsg->pLink, pMsg->transactId, channel);
				a2dp_ChangeStreamState( a2dpProfile[A2dpIndex], AVDTP_STREAM_STATE_STARTED);
				a2dp_HandleStartStreamInd( a2dpProfile[A2dpIndex]);
			}
			else
			{
				avdtp_SendStartStreamReject(pMsg->pLink, pMsg->transactId, 0 , channel); /**0--need to fix, now is ok, default 0*/
			}

			break;
#if 1
		case AVDTP_CLOSE:
			break;

		case AVDTP_SUSPEND:
			if (a2dpProfile[A2dpIndex]->pA2DPData->streamState == AVDTP_STREAM_STATE_STARTED)
			{
				a2dp_ChangeStreamState( a2dpProfile[A2dpIndex], AVDTP_STREAM_STATE_OPEN);
				a2dp_HandleSuspendStreamInd( a2dpProfile[A2dpIndex]);
				avdtp_SendSuspendStreamResp(pMsg->pLink, pMsg->transactId, channel);
			}
			else
			{
				avdtp_SendSuspendStreamReject(pMsg->pLink, pMsg->transactId, 0 /*XYXY manage stream indexes */, channel);
			}
			break;
#endif
		case AVDTP_ABORT:
			if (a2dpProfile[A2dpIndex]->pA2DPData->streamState != AVDTP_STREAM_STATE_IDLE)
			{
				// XYXY This is not clean. What if we are i.e. reconfiguring?
				a2dp_ChangeStreamState( a2dpProfile[A2dpIndex], AVDTP_STREAM_STATE_CLOSING);
			  //avdtp_CloseStream(&a2dpProfile[A2dpIndex]->pA2DPData->link, 0, channel);
			}
			avdtp_SendAbortResponse(pMsg->pLink, 0, pMsg->transactId, channel);
			break;
#if 0			
		case AVDTP_SECURITY_CONTROL:
			avdtp_SendSecurityControlReject(pMsg->pLink, pMsg->transactId, channel);
			break;
#endif
		case AVDTP_DATA:
			{
				bool processData = TRUE;
				if (
				  /*(a2dpProfile->commonData.state != ProfileStateIdle) ||*/
				  (a2dpProfile[A2dpIndex]->pA2DPData->streamState != AVDTP_STREAM_STATE_STARTED)
				 )
				{
					//debugOut(dbTrace, MPA_A2DP_DEBUG_HEADER"!Received A2DP data in wrong state");
					avdtp_SendStreamDataResp(pMsg->pLink, pMsg->streamIndex);
					processData = FALSE;
				}
				else if (!a2dp_QueueSBCBuffer(a2dpProfile[A2dpIndex], pMsg))
				{
					avdtp_SendStreamDataResp(pMsg->pLink, pMsg->streamIndex);
					processData = FALSE;
				}

				if (processData)
				/* put message in my own queue */
				{
					a2dpProfile[A2dpIndex]->pA2DPData->sinkStream.pendingDataResponses++;
					a2dp_ProcessNextSBCBuffer(a2dpProfile[A2dpIndex], channel);
				}

			} /*if (pProfile->u.a2dpData.role == MPA_A2DP_ROLE_SINK) */
		  break;
		}
	break;
	}
	
	/* Handle positive responses */
	case AVDTP_MESSAGE_TYPE_RESPONSE_ACCEPT:
	{

	switch(pMsg->signalID)
	{
	case AVDTP_DISCOVER:
		a2dp_OnDiscoverResp(a2dpProfile[A2dpIndex], pMsg, channel);
		break;

	case AVDTP_GET_CAPABILITIES:
		a2dp_OnGetCapabilitiesResp(a2dpProfile[A2dpIndex], pMsg, channel);
		break;

	case AVDTP_START:
	  if  (a2dpProfile[A2dpIndex]->pA2DPData->streamState == AVDTP_STREAM_STATE_OPEN)
	  {
	    a2dp_ChangeStreamState( a2dpProfile[A2dpIndex], AVDTP_STREAM_STATE_STARTED);
	    a2dp_HandleStartStreamInd( a2dpProfile[A2dpIndex]);
	  }
	  break;
	case AVDTP_SUSPEND:
	  if (a2dpProfile[A2dpIndex]->pA2DPData->streamState == AVDTP_STREAM_STATE_STARTED)
	  {
	    a2dp_ChangeStreamState( a2dpProfile[A2dpIndex], AVDTP_STREAM_STATE_OPEN);
	    a2dp_HandleSuspendStreamInd( a2dpProfile[A2dpIndex]);
	  }
	break;

	case AVDTP_SET_CONFIGURATION:
		if (a2dpProfile[A2dpIndex]->pA2DPData->streamState == AVDTP_STREAM_STATE_IDLE)
		{
			a2dp_ChangeStreamState( a2dpProfile[A2dpIndex], AVDTP_STREAM_STATE_CONFIGURED);
			a2dp_HandleSetConfigurationResp( a2dpProfile[A2dpIndex]);
		}
		break;
#if 1	  
	case AVDTP_RECONFIGURE:
		if (a2dpProfile[A2dpIndex]->pA2DPData->streamState == AVDTP_STREAM_STATE_OPEN)
		{
			// Stay in suspended state. Reconfiguration is just an option
			// a2dp_ChangeStreamState( pA2DPData, avdtp_StreamStateConfigured);
			mpaA2dpHandleReconfigureResp( a2dpProfile[A2dpIndex], channel);
		}
	  break;

	case AVDTP_GET_CONFIGURATION:
		{	
			TSBCParams pParams;
			TA2dpCapabilities Capabilities;
			a2dp_ConfigToParams(0,&pParams,&pMsg->data[6]);
			Capabilities.samplingFrequency = pParams.samplingFrequency;
			Capabilities.blockNumber= pParams.blockNumber;
			Capabilities.channelMode= pParams.channelMode;
			Capabilities.allocMethod= pParams.allocMethod;
			Capabilities.subbandNumber= pParams.subbandNumber;
			Capabilities.minBitpool= pParams.bitpool;
			Capabilities.maxBitpool= Capabilities.minBitpool;
			a2dpProfile[A2dpIndex]->AppA2dpCallback(&Capabilities,a2dpProfile[A2dpIndex]->pA2DPData->link.remoteAddress,GET_CONFIGURATION);
		}
	case AVDTP_OPEN:
	case AVDTP_CLOSE:
	case AVDTP_ABORT:
	case AVDTP_SECURITY_CONTROL:
	  break;
#endif	  
	}
	break;
  }
#if 0
	/* Handle negative responses */
	case AVDTP_MESSAGE_TYPE_RESPONSE_REJECT:
	debugOut(dbTrace, MPA_A2DP_DEBUG_HEADER"! reject cause 0x%x", pMsg->data[0]);
	switch(pMsg->signalID)
	{
	case AVDTP_RECONFIGURE:
	  //XYXY Quick hack. This should be in next higher layer.
	  //Reconfiguration is not mandatory. Start stream anyway.
	  debugOut(dbTrace, MPA_A2DP_DEBUG_HEADER"a2dp_StartStream()");
	  avdtp_StartStream(&pProfile->u.a2dpData.pA2DPData->link, 0);
	  break;
	case AVDTP_SUSPEND:
	  /* If remote side doesn_t support suspend mode, continue streaming. */
	  break;
	case AVDTP_DISCOVER:
	case AVDTP_GET_CAPABILITIES:
	case AVDTP_SET_CONFIGURATION:
	case AVDTP_GET_CONFIGURATION:
	case AVDTP_OPEN:
	case AVDTP_START:
	case AVDTP_CLOSE:
	case AVDTP_ABORT:
	case AVDTP_SECURITY_CONTROL:
	  //XYXY TODO
	  debugOut(dbTrace, MPA_A2DP_DEBUG_HEADER"a2dp_CloseStream()");
	  a2dp_ChangeStreamState( pProfile, avdtp_StreamStateClosing);
	  avdtp_CloseStream(&pProfile->u.a2dpData.pA2DPData->link, STREAM_INDEX);
	  break;
	default:
	  break;
	}
	break;
#endif		
	default:
	break;
	}

}

void A2dpTimerCallBack(void *xTimer)
{
	uint8_t time_type = 0;
	if(a2dpProfile[A2dpIndex]->a2dp_time_handle == xTimer)
	{
		a2dpProfile[A2dpIndex]->AppA2dpCallback(&time_type, a2dpProfile[A2dpIndex]->pA2DPData->link.remoteAddress, TIME_OUT);	
		a2dpProfile[A2dpIndex]->used = 0;
	}
}

TA2dpResult a2dp_Connect(TBdAddr *bd)
{
	TBtConReq tavdtpConReq;
	if(a2dp_allocateindex(0)==A2DP_NO_ENDPOINTID)
		return A2DP_NO_ENDPOINTID;
	else 
	{
	a2dpProfile[A2dpIndex]->pA2DPData->link.role=A2DP_ROLE_SOURCE;
	
	memset(&tavdtpConReq, 0, sizeof(TBtConReq));
	memcpy(tavdtpConReq.bd,bd,BLUE_API_BD_SIZE);
	memcpy(&a2dpProfile[A2dpIndex]->pA2DPData->link.remoteAddress, bd, BLUE_API_BD_SIZE);
	tavdtpConReq.frameSize = 0;
	tavdtpConReq.channel = AVDTP_CHANNEL_TYPE_SIGNALLING;
	tavdtpConReq.reconnectFlag = FALSE;

	a2dpProfile[A2dpIndex]->outgoingCall = TRUE; 
	avdtp_Connect(&tavdtpConReq);	
	osStartTimer(&(a2dpProfile[A2dpIndex]->a2dp_time_handle), 0, 0, 0, A2DP_WAIT_TIME, A2dpTimerCallBack);
	return A2DP_SUCCESS;
	}
}

TA2dpResult a2dp_InitConfiguration(TA2dpSetCapabilities localCaps,PAppA2dpCallback AppA2dpCallback)
{
	void initA2dpProfile(PA2dpProfile profile,TA2dpSetCapabilities localCaps,PAppA2dpCallback AppA2dpCallback);
	if(A2dpIndex == MAX_INDEX)				
		return A2DP_NO_ENDPOINTID;

	a2dpProfile[A2dpIndex] = osMemoryAllocate(RAM_TYPE_DATA_ON, sizeof(TA2dpProfile));
	if(a2dpProfile[A2dpIndex] == NULL)
	{
		//....
		return A2DP_NO_RESOURCE;
	}
	memset(a2dpProfile[A2dpIndex], 0, sizeof(TA2dpProfile)); //pvPortMalloc do not memset memory
	initA2dpProfile(a2dpProfile[A2dpIndex++],localCaps,AppA2dpCallback);
	return A2DP_SUCCESS;
}

TA2dpResult a2dp_Reconnect(TBdAddr *bd)
{
	TBtConReq tavdtpConReq;
	uint8_t BumbleBeeBd[6] = {0x59,0x83,0x00,0xa0,0xfd,0x74};
	memset(&tavdtpConReq, 0, sizeof(TBtConReq));
	tavdtpConReq.frameSize = 0;
	tavdtpConReq.channel = AVDTP_CHANNEL_TYPE_SIGNALLING;
	if(a2dp_allocateindex((memcmp(bd,BumbleBeeBd,6) == 0)?0:1) == A2DP_NO_ENDPOINTID)
		return A2DP_NO_ENDPOINTID;
	
	if(a2dpProfile[A2dpIndex]->pA2DPData->sinkStream.pStream->used == 0)
	{	
		a2dpProfile[A2dpIndex]->pA2DPData->link.role=A2DP_ROLE_SINK;	   	
	}
	else if(a2dpProfile[A2dpIndex]->pA2DPData->sourceStream.pStream->used == 0)
	{
		a2dpProfile[A2dpIndex]->pA2DPData->link.role=A2DP_ROLE_SOURCE;
	}
	a2dpProfile[A2dpIndex]->outgoingCall = TRUE;
	memcpy(a2dpProfile[A2dpIndex]->pA2DPData->link.remoteAddress,bd,BLUE_API_BD_SIZE);	
	memcpy(tavdtpConReq.bd,a2dpProfile[A2dpIndex]->pA2DPData->link.remoteAddress,BLUE_API_BD_SIZE);
	avdtp_Connect(&tavdtpConReq);
	return A2DP_SUCCESS;
}

TA2dpResult a2dp_GetConfiguration(TBdAddr *pRemoteBd)
{
	PAvdtpChannel  pChannel = avdtp_ChannelFind(pAppAvdtp,pRemoteBd,0);
	if(a2dp_getindex(pRemoteBd) == A2DP_NO_ENDPOINTID)
		return A2DP_NO_ENDPOINTID;

	if (a2dpProfile[A2dpIndex]->streamLink.state == linkStateConnected)
	{
		if (a2dpProfile[A2dpIndex]->pA2DPData->streamState == AVDTP_STREAM_STATE_CONFIGURED||
			 a2dpProfile[A2dpIndex]->pA2DPData->streamState == AVDTP_STREAM_STATE_OPEN||
		   a2dpProfile[A2dpIndex]->pA2DPData->streamState == AVDTP_STREAM_STATE_STARTED)
		{
		 	return avdtp_GetConfiguration(&a2dpProfile[A2dpIndex]->pA2DPData->link, 0, pChannel->cid)?A2DP_SUCCESS:A2DP_NO_RESOURCE;
		}
		return A2DP_STREAM_STATE_ERR;
	}
	return A2DP_STREAM_LINK_ERROR;
}

TA2dpResult a2dp_Disconnect(TBdAddr *pRemoteBd)
{
	PAvdtpChannel  pChannel = avdtp_ChannelFind(pAppAvdtp,pRemoteBd,0);
	if(a2dp_getindex(pRemoteBd) == A2DP_NO_ENDPOINTID)
		return A2DP_NO_ENDPOINTID;
	
	if (a2dpProfile[A2dpIndex]->playerState != A2DP_PLAYER_STATE_STOPPED)
	{
		if (a2dpProfile[A2dpIndex]->streamLink.state != linkStateIdle)
		{
		  a2dp_ChangeStreamLinkState( a2dpProfile[A2dpIndex], linkStateDisconnecting);
		  a2dp_ChangeStreamState( a2dpProfile[A2dpIndex], AVDTP_STREAM_STATE_CLOSING);
		  return avdtp_DisconnectStreamChannel(pAppAvdtp, pChannel->cid)?A2DP_SUCCESS:A2DP_NO_RESOURCE;
		}
		return A2DP_STREAM_LINK_ERROR;
	}
	return A2DP_PLAY_STATE_ERROR;
}

TA2dpResult a2dp_Suspend(TBdAddr *pRemoteBd)
{
	PAvdtpChannel  pChannel = avdtp_ChannelFind(pAppAvdtp,pRemoteBd,0);
	if(a2dp_getindex(pRemoteBd) == A2DP_NO_ENDPOINTID)
		return A2DP_NO_ENDPOINTID;
	
	if (a2dpProfile[A2dpIndex]->streamLink.state == linkStateConnected)
	{
		if (a2dpProfile[A2dpIndex]->playerState == A2DP_PLAYER_STATE_PLAYING)
		{
		 	return avdtp_SuspendStream(&a2dpProfile[A2dpIndex]->pA2DPData->link, 0,pChannel->cid)?A2DP_SUCCESS:A2DP_NO_RESOURCE;	 
		}
		return A2DP_PLAY_STATE_ERROR;
	} 
	return A2DP_STREAM_LINK_ERROR;
}

TA2dpResult a2dp_Start(TBdAddr *pRemoteBd)
{
	PAvdtpChannel  pChannel = avdtp_ChannelFind(pAppAvdtp,pRemoteBd,0);
	if(a2dp_getindex(pRemoteBd) == A2DP_NO_ENDPOINTID)
		return A2DP_NO_ENDPOINTID;

	if (a2dpProfile[A2dpIndex]->streamLink.state == linkStateConnected)
	{
		if (a2dpProfile[A2dpIndex]->pA2DPData->streamState != AVDTP_STREAM_STATE_STARTED)
		{
			return avdtp_StartStream(&a2dpProfile[A2dpIndex]->pA2DPData->link, 0 ,pChannel->cid)?A2DP_SUCCESS:A2DP_NO_RESOURCE;   
		}	
		return A2DP_STREAM_STATE_ERR;
	}  
	return A2DP_STREAM_LINK_ERROR;
}

TA2dpResult a2dp_SendAudioData(void* msg)
{
	/** media packet only */
	TAvdtpLink* pLink=&a2dpProfile[0]->pA2DPData->link;
	PAvdtpChannel  pChannel;		
	pChannel=avdtp_ChannelFindStream(pAppAvdtp, (TBdAddr *)pLink->remoteAddress);		
	mpa_Sendl2cDataReq(((PBlueAPI_L2cDataInd)msg)->buf,((PBlueAPI_L2cDataInd)msg)->dataOffset,pChannel->cid,((PBlueAPI_L2cDataInd)msg)->length); //send data to Bumblebee
	return A2DP_SUCCESS;
}

void a2dp_L2cConInd(PBlueAPI_L2cConInd pL2cConInd, TAvdtpChannelType avdtpChanType)
{
	/*a2dp related*/
	if(avdtpChanType == AVDTP_CHANNEL_TYPE_SIGNALLING)
	{	
		extern uint8_t BumbleBeeBd[6];
		if(memcmp(BumbleBeeBd,pL2cConInd->remote_BD,BD_ADDR_SIZE) == 0)
			A2dpIndex=0;
		else						
			a2dp_allocateindex(1);
		
		if(a2dpProfile[A2dpIndex]->pA2DPData->link.streams[0].role == (AVDTP_ROLE_ACP | AVDTP_ROLE_SNK))
			a2dpProfile[A2dpIndex]->pA2DPData->link.role = A2DP_ROLE_SINK;
		else if(a2dpProfile[A2dpIndex]->pA2DPData->link.streams[0].role == (AVDTP_ROLE_INT | AVDTP_ROLE_SRC))
			a2dpProfile[A2dpIndex]->pA2DPData->link.role = A2DP_ROLE_SOURCE;
		
		memcpy(&(a2dpProfile[A2dpIndex]->pA2DPData->link.remoteAddress),
		pL2cConInd->remote_BD, BD_ADDR_SIZE);
	
		a2dpProfile[A2dpIndex]->outgoingCall = false;
	}else
	{
		//.....
	}
}

void a2dp_L2cDisInd(PBlueAPI_L2cDiscInd pL2cDisInd, TAvdtpChannelType avdtpChanType,uint8_t* pRemoteBd)
{
	a2dp_getindex((TBdAddr *)pRemoteBd);
	/*a2dp related*/
	if(avdtpChanType == AVDTP_CHANNEL_TYPE_SIGNALLING)
	{
		a2dp_ChangeSignalLinkState(a2dpProfile[A2dpIndex], linkStateIdle,
		  pL2cDisInd->cause);
		if(A2dpIndex != 0)
		{
			a2dpProfile[A2dpIndex]->AppA2dpCallback(NULL, pRemoteBd, DISCONNECT_COMPLETE);
		}

	}else
	{
		if(A2dpIndex == 0)
			a2dpProfile[A2dpIndex]->pA2DPData->sourceStream.pStream->used=0;
		else
			a2dpProfile[A2dpIndex]->pA2DPData->sinkStream.pStream->used=0;
		
		a2dpProfile[A2dpIndex]->used = 0;
	  	a2dpProfile[A2dpIndex]->pA2DPData->link.streams[0].used = 0;
		a2dpProfile[A2dpIndex]->pA2DPData->link.transactId = 0;
		a2dp_ChangeStreamState( a2dpProfile[A2dpIndex], AVDTP_STREAM_STATE_IDLE); //XYXY TODO: use only one state machine
		a2dp_ChangeStreamLinkState( a2dpProfile[A2dpIndex], linkStateIdle);
		if(A2dpIndex == 0)
		{
			a2dpProfile[A2dpIndex]->AppA2dpCallback(NULL, pRemoteBd, DISCONNECT_COMPLETE);
		}
	}
}


static void a2dp_CreateSBCBufferQueue(PA2dpProfile pProfile)
{
	pProfile->pA2DPData->sinkStream.AVDTPMessageQueueReadIndex  = 0;
	pProfile->pA2DPData->sinkStream.AVDTPMessageQueueWriteIndex = 0;
	pProfile->pA2DPData->sinkStream.AVDTPMessageQueueCount      = 0;
	pProfile->pA2DPData->sinkStream.CurrentSBCBufferOffset      = 0;
	pProfile->pA2DPData->sinkStream.pCurrentAVDTPMessage        = NULL;
}

static void a2dp_CreateDeliveredPCMBufferQueue(PA2dpProfile pProfile)
{
	listInit(&pProfile->pA2DPData->sinkStream.DeliveredPCMBufferQueue);
}

static unsigned char* a2dp_AllocatePCMBuffer(PA2dpProfile pProfile)
{
	unsigned char* pBuffer = NULL;
	pBuffer = osMemoryAllocate(RAM_TYPE_DATA_OFF, A2DP_PCM_BUFFER_SIZE);
	
	return pBuffer;
}

void a2dp_L2cConCompleteInd(PBlueAPI_L2cConActInd pL2cConActInd, TAvdtpChannelType type)
{
	if(type == AVDTP_CHANNEL_TYPE_STREAM)
	{
		int role=a2dpProfile[A2dpIndex]->pA2DPData->link.role;
		a2dp_ChangeStreamLinkState( a2dpProfile[A2dpIndex], linkStateConnected);

		if(role == A2DP_ROLE_SOURCE)
		{
			a2dpProfile[A2dpIndex]->pA2DPData->sourceStream.rtpBufferSize = pL2cConActInd->localUsMTU; //NEED TO DOUBLE CHECK!!!
			a2dpProfile[A2dpIndex]->pA2DPData->link.streams[0].frameSize  = pL2cConActInd->localUsMTU;//NEED TO DOUBLE CHECK!!!

			/*a2dpProfile[A2dpIndex]->pA2DPData->sourceStream.pRtpBuffer=pvPortMalloc(pL2cConActInd->localUsMTU);
			if (a2dpProfile[A2dpIndex]->pA2DPData->sourceStream.pRtpBuffer == NULL)
			{
			a2dpProfile[A2dpIndex]->pA2DPData->sourceStream.rtpBufferSize = 0;
			}*/
			a2dp_CalculateFramesizes(a2dpProfile[A2dpIndex]);
		}
		else
		{
			a2dpProfile[A2dpIndex]->pA2DPData->link.streams[0].frameSize  = pL2cConActInd->localUsMTU;
			a2dp_CreateSBCBufferQueue( a2dpProfile[A2dpIndex]);
			a2dp_CreateDeliveredPCMBufferQueue( a2dpProfile[A2dpIndex]);

			/* get first PCM buffer */
			/*if (a2dpProfile[A2dpIndex]->pA2DPData->sinkStream.pPCMBuffer == NULL)
			{
			a2dpProfile[A2dpIndex]->pA2DPData->sinkStream.pPCMBuffer = a2dpAllocatePCMBuffer( a2dpProfile[A2dpIndex]);
			a2dpProfile[A2dpIndex]->pA2DPData->sinkStream.PCMBufferWriteIndex = A2DP_PCM_BUFFER_GAP;
			}
			a2dpProfile[A2dpIndex]->pA2DPData->sinkStream.pendingDataResponses = 0;*/
			
			//FIXMEE		a2dpProfile->pA2DPData->link.streams[0].poolId = pBTA->dsPoolID;
			//FIXMEE		a2dpProfile->pA2DPData->link.streams[0].openRequests = min(A2DP_MAX_OPEN_REQUESTS, maxOpenReq);
		}
		a2dp_ChangeStreamState( a2dpProfile[A2dpIndex], AVDTP_STREAM_STATE_OPEN);
		{
			TSBCParams *pSBCParams = NULL;
			if(role == A2DP_ROLE_SOURCE)
				pSBCParams = &a2dpProfile[A2dpIndex]->pA2DPData->sourceStream.config;
			else	
				pSBCParams = &a2dpProfile[A2dpIndex]->pA2DPData->sinkStream.config;
			
			switch (pSBCParams->channelMode)
			{
			case SBC_MODE_MONO:
				a2dpProfile[A2dpIndex]->channels = 1;
				break;
			case SBC_MODE_DUAL:
			case SBC_MODE_STEREO:
			case SBC_MODE_JOINT:
				a2dpProfile[A2dpIndex]->channels = 2;
				break;
			}
	
			switch (pSBCParams->samplingFrequency)
			{
			case SBC_FREQU16000:
				a2dpProfile[A2dpIndex]->samplingFreq = 16000;
				break;
			case SBC_FREQU32000:
				a2dpProfile[A2dpIndex]->samplingFreq = 32000;
				break;
			case SBC_FREQU44100:
				a2dpProfile[A2dpIndex]->samplingFreq = 44100;
				break;
			case SBC_FREQU48000:
				a2dpProfile[A2dpIndex]->samplingFreq = 48000;
				break;
			}
			A2dpToAppMsg msg;
			msg.player_state=a2dpProfile[A2dpIndex]->playerState;
			msg.sound_channel=a2dpProfile[A2dpIndex]->channels;
			msg.samplefrequence=a2dpProfile[A2dpIndex]->samplingFreq;
			msg.used=a2dpProfile[A2dpIndex]->used;			
			a2dpProfile[A2dpIndex]->AppA2dpCallback(&msg,a2dpProfile[A2dpIndex]->pA2DPData->link.remoteAddress,CONNECT_COMPLETE);
			osStopTimer(&(a2dpProfile[A2dpIndex]->a2dp_time_handle));
		}
	}else
	{

		if ( a2dpProfile[A2dpIndex]->outgoingCall )
		{
			a2dp_ChangeStreamLinkState(a2dpProfile[A2dpIndex], linkStateConnecting);
			avdtp_Discover(&a2dpProfile[A2dpIndex]->pA2DPData->link, pL2cConActInd->cid);
		}
	}
}

bool a2dp_SdpRegister(void)
{
	uint16_t length = 0;
	void *pbuffer = NULL;
	
	//source
	length = legacy_SDPRecordLength("< I<U> I<<UI><UI>> I<U> I<III> I<<UI>> IS>",
                              SDP_ATTR_SERVICECLASSIDLIST, UUID_AUDIOSOURCE,
                              SDP_ATTR_PROTOCOLDESCRIPTORLIST,
                              UUID_L2CAP, PSM_AVDTP, UUID_AVDTP, 0x0100 /*version 1.0*/,
                              SDP_ATTR_BROWSEGROUPLIST, UUID_PUBLIC_BROWSE_GROUP,
                              SDP_ATTR_LANGUAGEBASEATTRIBUTEIDLIST,
                              SDP_LANGUAGE_ENGLISH, SDP_CHARCODE_UTF8, SDP_BASE_LANG_OFFSET,
                              SDP_ATTR_BLUETOOTHPROFILEDESCRIPTORLIST, UUID_ADVANCEDAUDIODISTRIBUTION, 0x0100 /* version 1.0 */,
                              SDP_ATTR_SERVICENAME + SDP_BASE_LANG_OFFSET , "a2dp_source" /* English Servicename */
                              );

	//DBG_BUFFER(MODULE_APP, LEVEL_INFO, "TestProfileInit: SDP buffer length cal is %d", 1, length);
	
	if (length)
	{
		pbuffer = osMemoryAllocate(RAM_TYPE_DATA_ON, length);
		length = legacy_SDPCreateDes(pbuffer,
								  "< I<U> I<<UI><UI>> I<U> I<III> I<<UI>> IS>",
                                  SDP_ATTR_SERVICECLASSIDLIST, UUID_AUDIOSOURCE,
                                  SDP_ATTR_PROTOCOLDESCRIPTORLIST,
                                  UUID_L2CAP, PSM_AVDTP, UUID_AVDTP, 0x0100 /*version 1.0*/,
                                  SDP_ATTR_BROWSEGROUPLIST, UUID_PUBLIC_BROWSE_GROUP,
                                  SDP_ATTR_LANGUAGEBASEATTRIBUTEIDLIST,
                                  SDP_LANGUAGE_ENGLISH, SDP_CHARCODE_UTF8, SDP_BASE_LANG_OFFSET,
                                  SDP_ATTR_BLUETOOTHPROFILEDESCRIPTORLIST, UUID_ADVANCEDAUDIODISTRIBUTION, 0x0100 /* version 1.0 */,
                                  SDP_ATTR_SERVICENAME + SDP_BASE_LANG_OFFSET , "a2dp_source" /* English Servicename */
                                  );

		//DBG_BUFFER(MODULE_APP, LEVEL_INFO, "TestProfileInit: SDP buffer length is %d", 1, length);

		if (length)
		{
			legacy_AddSDPRecord(pbuffer, length);
		}
		else
		{
			osMemoryFree(pbuffer);
			return FALSE;
		}
	}

	//sink 
	 length = legacy_SDPRecordLength("< I<U> I<<UI><UI>> I<U> I<III> I<<UI>> IS>",
                                      SDP_ATTR_SERVICECLASSIDLIST, UUID_AUDIOSINK,
                                      SDP_ATTR_PROTOCOLDESCRIPTORLIST,
                                      UUID_L2CAP, PSM_AVDTP, UUID_AVDTP, 0x0100 /*version 1.0*/,
                                      SDP_ATTR_BROWSEGROUPLIST, UUID_PUBLIC_BROWSE_GROUP,
                                      SDP_ATTR_LANGUAGEBASEATTRIBUTEIDLIST,
                                      SDP_LANGUAGE_ENGLISH, SDP_CHARCODE_UTF8, SDP_BASE_LANG_OFFSET,
                                      SDP_ATTR_BLUETOOTHPROFILEDESCRIPTORLIST, UUID_ADVANCEDAUDIODISTRIBUTION, 0x0100 /* version 1.0 */,
                                      SDP_ATTR_SERVICENAME + SDP_BASE_LANG_OFFSET , "a2dp_sink" /* English Servicename */
                                      );

	//DBG_BUFFER(MODULE_APP, LEVEL_INFO, "TestProfileInit: SDP buffer length cal is %d", 1, length);

	if (length)
	{
		pbuffer = osMemoryAllocate(RAM_TYPE_DATA_ON, length);
		length = legacy_SDPCreateDes(pbuffer,
									  "< I<U> I<<UI><UI>> I<U> I<III> I<<UI>> IS>",
                                      SDP_ATTR_SERVICECLASSIDLIST, UUID_AUDIOSINK,
                                      SDP_ATTR_PROTOCOLDESCRIPTORLIST,
                                      UUID_L2CAP, PSM_AVDTP, UUID_AVDTP, 0x0100 /*version 1.0*/,
                                      SDP_ATTR_BROWSEGROUPLIST, UUID_PUBLIC_BROWSE_GROUP,
                                      SDP_ATTR_LANGUAGEBASEATTRIBUTEIDLIST,
                                      SDP_LANGUAGE_ENGLISH, SDP_CHARCODE_UTF8, SDP_BASE_LANG_OFFSET,
                                      SDP_ATTR_BLUETOOTHPROFILEDESCRIPTORLIST, UUID_ADVANCEDAUDIODISTRIBUTION, 0x0100 /* version 1.0 */,
                                      SDP_ATTR_SERVICENAME + SDP_BASE_LANG_OFFSET , "a2dp_sink" /* English Servicename */
                                      );

		//DBG_BUFFER(MODULE_APP, LEVEL_INFO, "TestProfileInit: SDP buffer length is %d", 1, length);

		if (length)
		{
			legacy_AddSDPRecord(pbuffer, length);
		}
		else
		{
			osMemoryFree(pbuffer);
			return FALSE;
		}
	}
	return TRUE;
}

void initA2dpData(PA2dpProfile profile,PAppA2dpCallback AppA2dpCallback,TA2dpSetCapabilities localCaps)
{
	if(profile->pA2DPData == NULL)
	{
		profile->pA2DPData = osMemoryAllocate(RAM_TYPE_DATA_ON, sizeof(TA2DPData));
		if(profile->pA2DPData == NULL)
		{
			//allocate fail....
			return;
		}else
		{
			memset(profile->pA2DPData, 0, sizeof(TA2DPData));
		}
	}
	if(profile->pA2DPData)
	{
		if(!profile->pA2DPData->initialized)
		{	
			memset(&profile->pA2DPData->link, 0, sizeof(TAvdtpLink));
			profile->pA2DPData->link.streamCount=1;
			profile->AppA2dpCallback=AppA2dpCallback;
			if ((localCaps.Capabilities.channelMode & 0x08) != 0x00)
			{
				profile->capabilities.supported_channels |= A2DP_CHANNELS_MONO;
			}
			if ((localCaps.Capabilities.channelMode & 0x07) != 0x00)
			{
				profile->capabilities.supported_channels |= A2DP_CHANNELS_STEREO;
			}

			if ((localCaps.Capabilities.samplingFrequency & 0x80) == 0x80)
			{
				profile->capabilities.supported_sampling_frequencies |=
							A2DP_SUPPORTED_SAMPLING_FREQ_16000HZ;
			}
			if ((localCaps.Capabilities.samplingFrequency & 0x40) == 0x40)
			{
				profile->capabilities.supported_sampling_frequencies |=
							A2DP_SUPPORTED_SAMPLING_FREQ_32000HZ;
			}
			if ((localCaps.Capabilities.samplingFrequency & 0x20) == 0x20)
			{
				profile->capabilities.supported_sampling_frequencies |=
							A2DP_SUPPORTED_SAMPLING_FREQ_44100HZ;
			}
			if ((localCaps.Capabilities.samplingFrequency & 0x10) == 0x10)
			{
				profile->capabilities.supported_sampling_frequencies |=
							A2DP_SUPPORTED_SAMPLING_FREQ_48000HZ;
			}

			if(localCaps.role==(AVDTP_ROLE_INT|AVDTP_ROLE_SRC))
			{
			profile->pA2DPData->link.streams[0].role= localCaps.role;
			profile->pA2DPData->link.streams[0].mediaType  = localCaps.mediaType;
			profile->pA2DPData->link.streams[0].endPointType	 = localCaps.endPointType;
			profile->pA2DPData->link.streams[0].localEndPointId  = localCaps.localEndPointId;
			profile->pA2DPData->sourceStream.pStream			 = &(profile->pA2DPData->link.streams[0]);

			profile->pA2DPData->sourceStream.localCaps.samplingFrequency = localCaps.Capabilities.samplingFrequency;
			profile->pA2DPData->sourceStream.localCaps.channelMode = localCaps.Capabilities.channelMode;
			profile->pA2DPData->sourceStream.localCaps.blockNumber = localCaps.Capabilities.blockNumber;
			profile->pA2DPData->sourceStream.localCaps.subbandNumber = localCaps.Capabilities.subbandNumber;
			profile->pA2DPData->sourceStream.localCaps.allocMethod = localCaps.Capabilities.allocMethod;
			profile->pA2DPData->sourceStream.localCaps.minBitpool = localCaps.Capabilities.minBitpool;
			profile->pA2DPData->sourceStream.localCaps.maxBitpool = localCaps.Capabilities.maxBitpool;
		
			profile->pA2DPData->sourceStream.rtpSequNumber = 0;
			profile->pA2DPData->sourceStream.rtpTimeStamp = 0;	/* todo: initialize with random value */

			profile->pA2DPData->sourceStream.pRtpBuffer    = NULL;
			profile->pA2DPData->sourceStream.rtpBufferSize = 0;
			
			//sbcInitEncoder(); /*XYXYJZ What if we have more than 1 source instance ?? */
			}
			else if(localCaps.role==(AVDTP_ROLE_ACP|AVDTP_ROLE_SNK))
		/*a2dp sink init*/
			{
			profile->pA2DPData->link.streams[0].localEndPointId  = localCaps.localEndPointId;
			profile->pA2DPData->link.streams[0].role = localCaps.role;
			profile->pA2DPData->link.streams[0].mediaType  = localCaps.mediaType;
			profile->pA2DPData->link.streams[0].endPointType	 = localCaps.endPointType;
			profile->pA2DPData->sinkStream.pStream				 = &(profile->pA2DPData->link.streams[0]);
			
			profile->pA2DPData->sinkStream.localCaps.samplingFrequency = localCaps.Capabilities.samplingFrequency;
			profile->pA2DPData->sinkStream.localCaps.channelMode = localCaps.Capabilities.channelMode;
			profile->pA2DPData->sinkStream.localCaps.blockNumber = localCaps.Capabilities.blockNumber;
			profile->pA2DPData->sinkStream.localCaps.subbandNumber = localCaps.Capabilities.subbandNumber;
			profile->pA2DPData->sinkStream.localCaps.allocMethod = localCaps.Capabilities.allocMethod;
			profile->pA2DPData->sinkStream.localCaps.minBitpool = localCaps.Capabilities.minBitpool;
			profile->pA2DPData->sinkStream.localCaps.maxBitpool = localCaps.Capabilities.maxBitpool;
			profile->pA2DPData->sinkStream.pPCMBuffer			= NULL;
			profile->pA2DPData->sinkStream.PCMBufferWriteIndex	= A2DP_PCM_BUFFER_GAP;
			profile->pA2DPData->sinkStream.lastSequenceNumberIn   = -1;
			profile->pA2DPData->sinkStream.pendingDataResponses = 0;
			
			//We can add sbc init here ...sbcInitDecoder();
			
			sbcInitDecoder();
			listInit(&profile->pA2DPData->sinkStream.DeliveredPCMBufferQueue);
   			
		  }

			profile->pA2DPData->streamState = AVDTP_STREAM_STATE_IDLE;
			profile->pA2DPData->initialized = TRUE;
		}
	}
}
void initA2dpProfile(PA2dpProfile profile,TA2dpSetCapabilities localCaps,PAppA2dpCallback AppA2dpCallback)
{
	if(profile->enabled)
	{
		//already init.
		return;
	}else
	{
		profile->enabled = true;
		initA2dpData(profile,AppA2dpCallback,localCaps);
	}
}
TA2dpResult a2dp_Init(void)
{
	/**Sdp register*/
	if(a2dp_SdpRegister() == FALSE)
		return A2DP_NO_RESOURCE;
	/**Protocol init*/
	if(avdtp_ProtocolInit() == FALSE)
		return A2DP_NO_RESOURCE;
	
	return A2DP_SUCCESS;
}
