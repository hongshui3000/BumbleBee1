/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file        a2dp.h
* @brief      Advanced audio distribution profile (A2DP) definitions
* @details   
*
* @author   	gordon
* @date      	
* @version	v0.1
*/

#ifndef __A2DP_H
#define __A2DP_H
#include "avdtp.h"
#include "a2dp_api.h"
#include "sbc.h"
#include "mpa.h"

#define BIT(_n)   (UINT32)(1U<<(_n))
#define A2DP_WAIT_TIME  5000

#define A2DP_SBC_BUFFER_QUEUE_SIZE 24
#define BT_A2DP_SINK_PCM_BUFFER_SIZE_COUNT 1024        /*Size of PCM buffers for SBC decoding*/

#define A2DP_PCM_BUFFER_OFFSET sizeof( TList)
#define A2DP_PCM_BUFFER_GAP (A2DP_PCM_BUFFER_OFFSET+ 0x20) /* leave some space so the upper layers can add some headers */
//#define BT_MPA_A2DP_SINK_PCM_BUFFER_SIZE_COUNT 1024
#define A2DP_PCM_BUFFER_SIZE (BT_A2DP_SINK_PCM_BUFFER_SIZE_COUNT + A2DP_PCM_BUFFER_GAP)

#define A2DP_RTP_OFFSET    32
#define A2DP_RTP_HEADER_SIZE sizeof(TA2dpRTPHeader)

// #defines for RTP Header fields
// Octet 1
#define A2DP_RTP_HEADER_VERSION_SHIFT 6
#define A2DP_RTP_HEADER_VERSION_MASK  0xc0
#define A2DP_RTP_HEADER_PADDING_SHIFT 5
#define A2DP_RTP_HEADER_PADDING_MASK  0x20
#define A2DP_RTP_HEADER_EXTENSION_SHIFT 4
#define A2DP_RTP_HEADER_EXTENSION_MASK  0x10
#define A2DP_RTP_HEADER_CSRC_COUNT_SHIFT 0
#define A2DP_RTP_HEADER_CSRC_COUNT_MASK  0x0f

#define A2DP_CONTEXT_SIG     1
#define A2DP_CONTEXT_STREAM1 2
#define A2DP_CONTEXT_STREAM2 3

#define A2DP_MAX_OPEN_REQUESTS    4

#define A2DP_LOSC    6
#define A2DP_CODEC_SBC    0

/* Bitfield containing the supported sample rates on the PCM audio interface */
typedef uint8_t TA2DP_SUPPORTED_SAMLING_FREQUENCIES;
#define A2DP_SUPPORTED_SAMPLING_FREQ_16000HZ  0x01
#define A2DP_SUPPORTED_SAMPLING_FREQ_22050HZ  0x02
#define A2DP_SUPPORTED_SAMPLING_FREQ_24000HZ  0x04
#define A2DP_SUPPORTED_SAMPLING_FREQ_32000HZ  0x08
#define A2DP_SUPPORTED_SAMPLING_FREQ_44100HZ  0x10
#define A2DP_SUPPORTED_SAMPLING_FREQ_48000HZ  0x20

/** Bit field containing the supported quality capabilities of the A2DP devices */
typedef uint8_t TA2DP_QUALITY;
#define A2DP_QUALITY_LOW    0x01
#define A2DP_QUALITY_MEDIUM 0x02
#define A2DP_QUALITY_HIGH   0x04

typedef enum
{
	A2DP_ROLE_SOURCE = 0,
	A2DP_ROLE_SINK   = 1
} A2dpRole;

typedef enum tagAvdtpServiceCategory
{
  MEDIA_TRANSPORT = 0x1,
  REPORTING = 0x2,
  RECOVERY = 0x3,
  CONTENT_PROTECTION = 0x4,
  HEADER_COMPRESSION = 0x5,
  MULTIPLEXING = 0x6,
  MEDIA_CODEC = 0x7,
} TAvdtpServiceCategory;

typedef PACKED(struct)
{
  uint8_t sbc_channelmodes;
  uint8_t sbc_blocks;
  uint8_t sbc_minbitpool;
  uint8_t sbc_maxbitpool;
} TA2dpSBCCapabilities;

/** Structure containing the capabilities of the a2dp units.
 */
typedef PACKED(struct)
{
  TA2DP_SUPPORTED_SAMLING_FREQUENCIES  supported_sampling_frequencies; /**< supported sampling frequencies */
  TA2DP_QUALITY                       supported_qualities;           /**< supported audio qualities */
  TA2DP_CHANNELS                      supported_channels;            /**< supported audio channel numbers */
} TA2dpSupportedCapabilities;

typedef struct _TA2dpRemoteDevice
{
  TBdAddr                     bdAddr;
  TA2dpSBCCapabilities        sbc_capabilities; // Capabilities as reported by "get capabilities"
  TA2dpSupportedCapabilities      capabilities;    // Some capabilities translated into simple MPA format
  uint8_t           qualities;
} TA2dpRemoteDevice;

typedef enum tagAvdtpStreamState
{
	AVDTP_STREAM_STATE_IDLE,
	AVDTP_STREAM_STATE_CONFIGURED,
	AVDTP_STREAM_STATE_OPEN,
	AVDTP_STREAM_STATE_STARTED,
	AVDTP_STREAM_STATE_CLOSING,
	AVDTP_STREAM_STATE_ABORTING
} TA2dpStreamState;

typedef struct tagA2dpSourceStream
{
  TAvdtpStream* pStream;
  TA2dpCapabilities localCaps;
  TA2dpCapabilities remoteCaps;
  TSBCParams config;
  int frameSize;
  int bitRate;
  int samplesPerFrame;
  int framesPerPacket;
  int timePerPacket;
  int unprocessedSamples;
  int sampleBufferReadOffset;
  uint8_t * pRtpBuffer;
  int rtpBufferOffset;
  uint16_t rtpBufferSize;
  BOOL packetPending;
  BOOL demandPending;
  uint8_t * pSampleBuffer;
  uint16_t rtpSequNumber;
  uint32_t rtpTimeStamp;
} TA2dpSourceStream;

typedef struct tagA2dpSinkStream
{
  TAvdtpStream* pStream;
  TA2dpCapabilities localCaps;
  TA2dpCapabilities remoteCaps;
  TSBCParams config;
  int frameSize;
  int bitRate;

  /* Queue for SBC buffers from Blueface */
  TAvdtpMessage AVDTPMessageQueue[A2DP_SBC_BUFFER_QUEUE_SIZE];
  uint16_t AVDTPMessageQueueCount;
  uint16_t AVDTPMessageQueueReadIndex;
  uint16_t AVDTPMessageQueueWriteIndex;

  /* Management of currently used SBC buffers */
  TAvdtpMessage *pCurrentAVDTPMessage;
  int           CurrentSBCBufferOffset;

  /* I use this queue to remember PCM buffers that I sent to the higher layer with "a2dpStreamDataInd".
     When I get a2dpStreamDataResp() I pull the first entry from the queue and release the used memory */
  TList       DeliveredPCMBufferQueue;

  /* Data for currently used PCM buffer */
  unsigned char*      pPCMBuffer;
  uint16_t        PCMBufferWriteIndex;
  int         lastSequenceNumberIn;
  int         pendingDataResponses;
#if  0
  BOOL        holdBackFirstDataResponse; /*XYXY workaround for buffer overwrite effect */
  TAvdtpLink* pLink;    /* I have to remember the current link context because I have to send
                           the deferred data response on clean up */
  int         streamIndex;
#endif

} TA2dpSinkStream;

typedef struct tagA2DPData
{
	bool  initialized;
	TAvdtpLink link;
	TA2dpStreamState streamState;
    TA2dpSourceStream sourceStream;
	TA2dpSinkStream sinkStream;
} TA2DPData;
typedef TA2DPData *PA2DPData;

typedef enum _TLinkState
{
  linkStateIdle,
  linkStateSearchingFirstBond,
  linkStateSearchingChannel,
  linkStateWaitforConnectConfirmation,
  linkStateIncomingCall,
  linkStateConnecting,
  linkStateConnected,
  linkStateActiveDisconnect,
  linkStateDisconnecting
} TLinkState;

typedef struct _TLink
{
//  BLINKHANDLE             bLinkHandle;        /* link handle                          */
	TLinkState              state;
} TLink;

/*******/
typedef uint16_t T_Status;
typedef T_Status FAR *LP_Status;

typedef struct tagA2dpProfile
{
	bool enabled; //if profile init
	bool used;
	PA2DPData pA2DPData;
	bool					   ActiveDisconnect; /* We are disconneting both links */
	TLink                      signalLink;
	TA2dpRemoteDevice          remoteDevice;
	TA2dpSupportedCapabilities     capabilities;
	TA2DP_QUALITY		   requestedQuality; /* desired quality for PLAY_REQ */
	TLink                      streamLink;
	bool outgoingCall;
	TA2DP_CHANNELS          channels    ;  /* result of "number of channels" negotiation (Mono/Stereo) */
	uint32_t                      samplingFreq;   /* result of sampling frequency negotiation */
	TA2dpPlayerState           playerState;
	TA2DP_QUALITY          negotiatedQuality;    /**< result of quality negotiation */
	PAppA2dpCallback AppA2dpCallback;
	void *a2dp_time_handle;
}TA2dpProfile;
typedef TA2dpProfile *PA2dpProfile;

typedef struct tagA2dpRTPHeader
{
	uint8_t Version_Padding_Extension_CSRCCount;
	uint8_t Marker_PayloadType; // A2DP: always 0
	uint8_t SequenceNumber1;
	uint8_t SequenceNumber0;
	uint8_t TimeStamp3;
	uint8_t TimeStamp2;
	uint8_t TimeStamp1;
	uint8_t TimeStamp0;
	uint8_t SSRC3;
	uint8_t SSRC2;
	uint8_t SSRC1;
	uint8_t SSRC0;
	uint8_t SBCMediaHeader;
} TA2dpRTPHeader;

void a2dp_L2cConInd(PBlueAPI_L2cConInd pL2cConInd, TAvdtpChannelType avdtpChanType);
void a2dp_L2cConCompleteInd(PBlueAPI_L2cConActInd pL2cConActInd, TAvdtpChannelType type);
void A2dpAvdtpCallback(TAvdtpMessage* pMsg, void* eventcontext, uint16_t channel);
void a2dp_L2cDisInd(PBlueAPI_L2cDiscInd pL2cDisInd, TAvdtpChannelType avdtpChanType,uint8_t* pRemoteBd);
void a2dp_ChangeStreamState( PA2dpProfile pProfile, TA2dpStreamState newstate);
void a2dp_ChangeStreamLinkState(PA2dpProfile pProfile, TLinkState newState);


#endif
