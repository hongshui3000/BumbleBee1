#ifndef __A2DP_API_H
#define __A2DP_API_H 
#include "rtl_types.h"
/*MAX ENDPOINT NUMBER */
#define MAX_INDEX 2

#define AVDTP_ROLE_INT   0x1
#define AVDTP_ROLE_ACP   0x2
#define AVDTP_ROLE_SRC   0x4
#define AVDTP_ROLE_SNK   0x8

/* AVDTP Media Type                                                         */
#define AVDTP_MEDIA_TYPE_AUDIO              0
#define AVDTP_MEDIA_TYPE_VIDEO              1
#define AVDTP_MEDIA_TYPE_MULTIMEDIA         2

/** Bit field containing the supported number of channels. */
typedef uint8_t TA2DP_CHANNELS;
#define A2DP_CHANNELS_MONO    0x01
#define A2DP_CHANNELS_STEREO  0x02

typedef uint16_t TA2dpSamplingFrequency;
typedef uint16_t  TA2dpChannelMode;
typedef uint16_t  TA2dpBlockNumber;
typedef uint16_t  TA2dpSubbandNumber;
typedef uint16_t  TA2dpAllocMethod;

#define BD_ADDR_SIZE      6
typedef uint8_t TBdAddr[BD_ADDR_SIZE];
typedef uint8_t TAvdtpRole;   //wade

typedef struct
{
	int player_state;
	int sound_channel;
	int samplefrequence;
	int used;
}A2dpToAppMsg;

typedef enum
{
  A2DP_PLAYER_STATE_STOPPED,
  A2DP_PLAYER_STATE_PAUSED,
  A2DP_PLAYER_STATE_PLAYING
} TA2dpPlayerState;

typedef struct tagA2dpCapabilities
{
	TA2dpSamplingFrequency samplingFrequency;
	TA2dpBlockNumber blockNumber;
	TA2dpChannelMode channelMode;
	TA2dpAllocMethod allocMethod;
	TA2dpSubbandNumber subbandNumber;
	uint8_t minBitpool;
	uint8_t maxBitpool;
} TA2dpCapabilities;

typedef struct tagA2dpSetCapabilities
{
	TA2dpCapabilities Capabilities;
	TAvdtpRole role;
	uint8_t mediaType;
	uint8_t endPointType;
	uint8_t localEndPointId;
}TA2dpSetCapabilities;

typedef enum
{
	A2DP_SUCCESS,
	A2DP_NO_ENDPOINTID,
	A2DP_NO_RESOURCE,				//has not memory
	A2DP_STREAM_STATE_ERR,
	A2DP_PLAY_STATE_ERROR,	   //should be Playing
	A2DP_STREAM_LINK_ERROR, 		//streamlink has not connect
} TA2dpResult;
typedef enum
{
	PLAYER_STATE,
	REGISTER_COMPLETE,
	AUTHORIZATION_INDICATION,
	CONNECT_COMPLETE,
	GET_CONFIGURATION,
	DATA_INDICATION,
	DISCONNECT_COMPLETE,
	RESULT,
	TIME_OUT
}A2dpToAppMsgType;

typedef void(*PAppA2dpCallback)(void *msg,TBdAddr bd,A2dpToAppMsgType AppA2dp_msg);

TA2dpResult a2dp_Init(void);
TA2dpResult a2dp_Connect(TBdAddr *pRemoteBd);
TA2dpResult a2dp_Reconnect(TBdAddr *pRemoteBd);
TA2dpResult a2dp_Suspend(TBdAddr *pRemoteBd);
TA2dpResult a2dp_Start(TBdAddr *pRemoteBd);
TA2dpResult a2dp_GetConfiguration(TBdAddr *pRemoteBd);
TA2dpResult a2dp_Disconnect(TBdAddr *pRemoteBd);
TA2dpResult a2dp_InitConfiguration(TA2dpSetCapabilities localCaps,PAppA2dpCallback AppA2dpCallback);
TA2dpResult a2dp_SendAudioData(void* msg);

#endif
