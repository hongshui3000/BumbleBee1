
#ifndef _AVDTP_H
#define _AVDTP_H

#include <FreeRTOS.h>
#include <mpa.h>
#include "sdp_code.h"
#include "rtl_types.h"
#include "os_queue.h"

#define FAR
#define AVDTP_WAIT_TIME  1000
#define AVDTP_MAX_CHANNELS_COUNT	6
#define AVDTP_L2CAP_MTU_SIZE  		1691
#define BT_L1_HCI_DS_OFFSET			4
#define AVDTP_MAX_SEP_COUNT			4
#define AVDTP_SIGNALLING_BUFFER_SIZE AVDTP_L2CAP_MTU_SIZE

#define AVDTP_SINGLE_PACKET_HEADER_SIZE       2    /* single packet         */
#define AVDTP_START_PACKET_HEADER_SIZE        3    /* start packet          */
#define AVDTP_OTHER_PACKET_HEADER_SIZE        1    /* continue + end packet */

#define AVDTP_FLAG_SIGNAL    0
#define AVDTP_FLAG_STREAM    1

#define UPSTREAM_SIGNAL_OFFSET             64
#define UPSTREAM_SIGNAL_DATA_SIZE          25

#define AVDTP_INVALID_COMMAND    (int) -1
#define AVDTP_INVALID_CONTEXT    (int) -2
#define AVDTP_STREAM_DATA    (int) -3
#define INVALID_STREAM_INDEX    -1

#define AVDTP_ROLE_INT   0x1
#define AVDTP_ROLE_ACP   0x2
#define AVDTP_ROLE_SRC   0x4
#define AVDTP_ROLE_SNK   0x8

#define L2CAP_HDR_LENGTH 4
#define ACL_HDR_LENGTH   5

typedef uint8_t TAvdtpRole;   //wade

#define BD_ADDR_SIZE      6
typedef uint8_t TBdAddr[BD_ADDR_SIZE];

typedef enum _TAvdtpState
{
	AVDTP_STATE_CLOSED = 0,               /* no L2CAP connection  */
	AVDTP_STATE_L2CCON_CONFPENDING,
	AVDTP_STATE_L2CCONNECTING,
	AVDTP_STATE_L2CDISCONNECTING,
	AVDTP_STATE_L2CCONNECTED,             /* L2CAP connected               */
	AVDTP_STATE_L2CCONFIGURING,
	AVDTP_STATE_L2COPEN,                  /* ready for L2CAP data exchange */
	AVDTP_STATE_DISCONNECTING_STREAM
} TAvdtpState;

typedef enum _TAvdtpChannelType
{
	AVDTP_CHANNEL_TYPE_SIGNALLING = 1,
	AVDTP_CHANNEL_TYPE_STREAM
} TAvdtpChannelType;

/**
 * SEP data
 */
typedef enum _TAvdtpSepState
{
	AVDTP_SEP_STATE_IDLE = 0,
	AVDTP_SEP_STATE_CONFIGURED,
	AVDTP_SEP_STATE_OPENED,
	AVDTP_SEP_STATE_STREAMING,
	AVDTP_SEP_STATE_CLOSESENT
} TAvdtpSepState;

typedef enum _AvdtpTimerID
{
  AVDTP_TIMERID_RTX_SIG_TIMER = 1  /* TGAVDP100 0,5 - 3 second               */
} TAvdtpTimerID;
/**
 * AVDTP channel data
 */
typedef struct _TAvdtpL2CInfo         /**< L2CAP data */
{
	int16_t              wRxMtuSize;
	int16_t              wTxMtuSize;

	unsigned char              bConfBits;
} TAvdtpL2CInfo;
typedef TAvdtpL2CInfo * PAvdtpL2CInfo;

typedef struct _TAvdtpChannel
{
	int                 iNumber;         /**< MUST be first entry !!!!  */
	bool                Used;            /**< MUST be second entry !!!! */
	bool                BoundToAppl;     /**< TRUE: at least 1 appl. msg sent/rcvd*/
	//OR  BOOL                Initiator;       	 /**< TRUE: outgoing connection        */

	TAvdtpState         State;
	TBdAddr             RemoteBd;        /**< Bluetooth address of remote side */
	TAvdtpChannelType   Type;
	int16_t                cid;             /**< channel ID                       */
	int16_t                reqId;           /**< Id in xxx_REQ, returned xxx_CONF */
	int16_t                cause;           /**< disconnect cause                         */
	int8_t                holdLink;        /**< ==0: ACL link can be released immed. (no timeout) */

	/** Media channel */
	struct _TAvdtpChannel *pChannelSignalling;
	struct _TAvdtpSepData *pSepData;
	unsigned char                LocalSeid;

	/** Signalling channel */
	unsigned char*              pRxSignal;
	int16_t                RxSignalLength;
	unsigned char                nosp;            /**< number of signal packets */
	unsigned char                TransactionLabel;
	unsigned char                Signal;
	bool                TimerStarted;
	bool                TimeOut;
	TAvdtpTimerID       TimerID;         /**< timer currently started */
    /** last sent signalling command */
	unsigned char                SignalHeader[AVDTP_SINGLE_PACKET_HEADER_SIZE + 1];

	QUEUE_T             SepQueue;
	/** L2CAP data */
	TAvdtpL2CInfo       L2C;
} TAvdtpChannel;
typedef TAvdtpChannel * PAvdtpChannel;

typedef struct _TAvdtpSepData
{
	struct _TAvdtpSepData         *Next;
	TAvdtpSepState                State;
	PAvdtpChannel                 pChannelMedia;
	unsigned char                 LocalSeid;
	unsigned char                 RemoteSeid;
} TAvdtpSepData;
typedef TAvdtpSepData *PAvdtpSepData;

typedef struct _TAppAvdtp
{
	unsigned short  QueueID;              /**< own (input) queue     */
		/* AVDTP channels/connections */
	TAvdtpSepData  SepData[AVDTP_MAX_SEP_COUNT];
	QUEUE_T        FreeSepQueue;
	PAvdtpChannel  pChannel; 
	uint8_t        txOffset;
    uint8_t 	   writeOffset;
	uint8_t        TransactionLabel;
	TAvdtpChannel  ChannelArray[AVDTP_MAX_CHANNELS_COUNT];
	uint16_t		dsPoolID;
	void * avdtp_time_handle;
} TAppAvdtp;
typedef TAppAvdtp * PAppAvdtp;


typedef struct tagAvdtpStream
{
	uint8_t used;    //wade
	TAvdtpRole role;
	uint8_t mediaType;
	uint8_t endPointType;
	uint8_t localEndPointId;
	uint8_t remoteEndPointId;
	uint16_t openRequests;
	uint16_t frameSize;
} TAvdtpStream;

typedef struct tagAvdtpLink
{
	TAvdtpRole role;
	TBdAddr remoteAddress;
	uint8_t transactId;
	uint8_t streamCount;
	TAvdtpStream streams[1];
} TAvdtpLink;

typedef struct tagAvdtpMessage
{
  uint8_t     type;
  uint8_t   signalID;
  TAvdtpLink* pLink;
  int streamIndex;
  uint8_t transactId;
  uint16_t dataLength;
  unsigned char* data;
  int bufReleasePos;
} TAvdtpMessage;

typedef struct                  
{
    uint8_t bd[6];                
    uint16_t    channel;          
    uint16_t    reqId;              
    uint16_t    frameSize;    
	uint8_t		reconnectFlag;
	uint8_t		RemoteSeid;
} TBtConReq, *PBtConReq;
typedef TBtConReq * PAvdtpConReq;

typedef uint8_t TAvdtpEventType;
typedef uint8_t TAvdtpEventSignalID;

typedef struct tagAvdtpUpStreamSignal
{
	uint8_t gap[UPSTREAM_SIGNAL_OFFSET];
	uint8_t header;
	TAvdtpEventSignalID signalID;
	uint8_t data[UPSTREAM_SIGNAL_DATA_SIZE]; //need check UPSTREAM_SIGNAL_DATA_SIZE
} TAvdtpUpStreamSignal;

/****************************************************************************/
/* AVDTP signal                                                             */
/****************************************************************************/

#define AVDTP_DISCOVER                      0x01
#define AVDTP_GET_CAPABILITIES              0x02
#define AVDTP_SET_CONFIGURATION             0x03
#define AVDTP_GET_CONFIGURATION             0x04
#define AVDTP_RECONFIGURE                   0x05
#define AVDTP_OPEN                          0x06
#define AVDTP_START                         0x07
#define AVDTP_CLOSE                         0x08
#define AVDTP_SUSPEND                       0x09
#define AVDTP_ABORT                         0x0A
#define AVDTP_SECURITY_CONTROL              0x0B
/* This is a non-standard declaration that I use to tranport A2DP data
 * indications using the same message as for the above commands */
#define AVDTP_DATA                          0x80


/****************************************************************************/
/* AVDTP Media Type                                                         */
/****************************************************************************/
#define AVDTP_MEDIA_TYPE_AUDIO              0
#define AVDTP_MEDIA_TYPE_VIDEO              1
#define AVDTP_MEDIA_TYPE_MULTIMEDIA         2

/****************************************************************************/
/* AVDTP TSEP                                                               */
/****************************************************************************/
#define AVDTP_TSEP_SRC                      0
#define AVDTP_TSEP_SNK                      1

/****************************************************************************/
/* AVDTP header                                                             */
/****************************************************************************/

/******** Message type Bit 1-0 **********************************************/
#define AVDTP_MESSAGE_TYPE_COMMAND          0x00 /* command            */
#define AVDTP_MESSAGE_TYPE_RFD              0x01 /* not used, yet    */
#define AVDTP_MESSAGE_TYPE_RESPONSE_ACCEPT  0x02 /* response accept    */
#define AVDTP_MESSAGE_TYPE_RESPONSE_REJECT  0x03 /* response reject    */

#define AVDTP_MESSAGE_TYPE_MASK             0x03

/******** Packet type Bit 3-2 ***********************************************/
#define AVDTP_PACKET_TYPE_SINGLE            0x00 /* 00 - single   packet    */
#define AVDTP_PACKET_TYPE_START             0x04 /* 01 - start    packet    */
#define AVDTP_PACKET_TYPE_CONTINUE          0x08 /* 10 - continue packet    */
#define AVDTP_PACKET_TYPE_END               0x0C /* 11 - end      packet    */

#define AVDTP_PACKET_TYPE_MASK              0x0C

/****************************************************************************/
/* AVDTP error codes */
/****************************************************************************/

#define AVDTP_ERROR_BAD_HEADER_FORMAT              0x01
#define AVDTP_ERROR_BAD_LENGTH                     0x11
#define AVDTP_ERROR_BAD_ACP_SEID                   0x12
#define AVDTP_ERROR_SEP_IN_USE                     0x13
#define AVDTP_ERROR_SEP_NOT_IN_USE                 0x14
#define AVDTP_ERROR_BAD_SERV_CATEGORY              0x17
#define AVDTP_ERROR_BAD_PAYLOAD_FORMAT             0x18
#define AVDTP_ERROR_NOT_SUPPORTED_COMMAND          0x19
#define AVDTP_ERROR_INVALID_CAPABILITIES           0x1A
#define AVDTP_ERROR_BAD_RECOVERY_TYPE              0x22
#define AVDTP_ERROR_BAD_MEDIA_TRANSPORT_FORMAT     0x23
#define AVDTP_ERROR_BAD_RECOVERY_FORMAT            0x25
#define AVDTP_ERROR_BAD_ROHC_FORMAT                0x26
#define AVDTP_ERROR_BAD_CP_FORMAT                  0x27
#define AVDTP_ERROR_BAD_MULTIPLEXING_FORMAT        0x28
#define AVDTP_ERROR_UNSUPPORTED_CONFIGURAION       0x29
#define AVDTP_ERROR_BAD_STATE                      0x31

/****************************************************************************/
/* AVDTP service category                                                   */
/****************************************************************************/

#define AVDTP_MEDIA_TRANSPORT                 0x01
#define AVDTP_REPORTING                       0x02
#define AVDTP_RECOVERY                        0x03
#define AVDTP_CONTENT_PROTECTION              0x04
#define AVDTP_HEADER_COMPRESSION              0x05
#define AVDTP_MULTIPLEXING                    0x06
#define AVDTP_MEDIA_CODEC                     0x07

/****** Recovery type *******************************************************/
#define AVDTP_RECOVERY_TYPE_RFC2733           0x01

/****** Recovery window size (MRWS) *****************************************/
#define AVDTP_RECOVERY_MRWS_MIN               0x01
#define AVDTP_RECOVERY_MRWS_MAX               0x18

/****** Recovery number of media packets in parity code (MNMP) **************/
#define AVDTP_RECOVERY_MNMP_MIN               0x01
#define AVDTP_RECOVERY_MNMP_MAX               0x18

/****** Header compression **************************************************/
#define AVDTP_HEADER_COMPRESSION_BACK_CHANNEL 0x80
#define AVDTP_HEADER_COMPRESSION_MEDIA        0x40
#define AVDTP_HEADER_COMPRESSION_RECOVERY     0x20

/** AVDTP Layer To Layer Error Code (the values 0x01 ... 0x31 are defined by AVDTP)  */
#define AVDTP_SUCCESS                         0x00

#define AVDTP_ERR_BAD_HEADER_FORMAT           0x01

#define AVDTP_ERR_BAD_LENGTH                  0x11
#define AVDTP_ERR_BAD_ACP_SEID                0x12
#define AVDTP_ERR_SEP_IN_USE                  0x13
#define AVDTP_ERR_SEP_NOT_IN_USE              0x14
#define AVDTP_ERR_BAD_SERV_CATEGORY           0x17
#define AVDTP_ERR_BAD_PAYLOAD_FORMAT          0x18
#define AVDTP_ERR_NOT_SUPPORTED_COMMAND       0x19
#define AVDTP_ERR_INVALID_CAPABILITIES        0x1A

#define AVDTP_ERR_BAD_RECOVERY_TYPE           0x22
#define AVDTP_ERR_BAD_MEDIA_TRANSPORT_FORMAT  0x23
#define AVDTP_ERR_BAD_RECOVERY_FORMAT         0x25
#define AVDTP_ERR_BAD_ROHC_FORMAT             0x26
#define AVDTP_ERR_BAD_CP_FORMAT               0x27
#define AVDTP_ERR_BAD_MULTIPLEXING_FORMAT     0x28
#define AVDTP_ERR_UNSUPPORTED_CONFIGURATION   0x29

#define AVDTP_ERR_BAD_STATE                   0x31


#define AVDTP_ERR_NORESOURCES                 0xF1
#define AVDTP_ERR_INVALID_FRAME_SIZE          0xF2
#define AVDTP_ERR_TIMEOUT                     0xF3
#define AVDTP_ERR_UNKNOWN_SIGNAL              0xF4
#define AVDTP_ERR_ILLEGAL_COMMAND             0xF5
#define AVDTP_ERR_ILLEGAL_RESPONSE            0xF6


bool avdtp_ProtocolInit(void);
bool avdtp_HandleRxSignallingPacket(PAppAvdtp pAvdtp, PBlueAPI_L2cDataInd pL2cDataInd);
PAvdtpSepData avdtp_SEPFindSeidAndState(PAvdtpChannel pChannel,
                                       uint8_t Seid, TAvdtpSepState State);
bool avdtp_SendAVDTP_DATA_IND( PAppAvdtp pAvdtp, PBlueAPI_L2cDataInd pMessage, bool flag_signal );
bool avdtp_SendDiscoverResp(TAvdtpLink* pLink, uint8_t  transactId, uint16_t channel);
PAvdtpChannel avdtp_ChannelFind(PAppAvdtp pAvdtp, TBdAddr *pRemoteBd, uint16_t cid);
bool avdtp_SendUpstreamSignal(TAvdtpLink* pLink,
                                              TAvdtpEventSignalID signalID,
                                              TAvdtpEventType type,
                                              uint8_t transactId,
                                              TAvdtpUpStreamSignal *pSignal,
                                              int size, uint16_t channel);

bool avdtp_SendGetCapsResp(TAvdtpLink* pLink, uint8_t  transactId, uint16_t length, uint8_t  * capsData, uint16_t channel);
bool avdtp_SetConfiguration(TAvdtpLink* pLink, int streamIndex, uint16_t length, uint8_t  * capsData, bool reconfigure, uint16_t channel);
bool avdtp_SendSetConfigResp(TAvdtpLink* pLink, uint8_t  transactId, uint16_t channel);
PAvdtpSepData avdtp_SEPFind(PAvdtpChannel pChannel, TAvdtpSepState State);
bool avdtp_SendStartStreamResp(TAvdtpLink* pLink, uint8_t  transactId, uint16_t channel);
bool avdtp_SendStartStreamReject(TAvdtpLink* pLink, uint8_t  transactId, int streamIndex, uint16_t channel);
bool avdtp_SendSetConfigReject(TAvdtpLink* pLink, uint8_t  transactId, uint16_t channel);
bool avdtp_SendAbortResponse(TAvdtpLink* pLink, int streamIndex,  uint8_t  transactId, uint16_t channel);
bool avdtp_CloseStream(TAvdtpLink* pLink, int streamIndex, uint16_t cid);
uint16_t avdtp_SendSignalAVDTP_CLOSE(PAppAvdtp pAvdtp, PAvdtpSepData pSepData,
                                              uint8_t  MessageType, uint16_t channel);
bool avdtp_DisconnectStreamChannel(PAppAvdtp pAvdtp, uint16_t channel);
bool avdtp_Discover(TAvdtpLink* pLink, uint16_t channel);
void avdtp_Connect(PAvdtpConReq	pConReq);
uint16_t avdtp_SendSignalAVDTP_OPEN(PAppAvdtp pAvdtp, PAvdtpSepData pSepData,
                                              uint8_t  MessageType, uint16_t channel);
																							
bool avdtp_GetCapabilities(TAvdtpLink* pLink, int streamIndex, uint16_t channel);
bool avdtp_AbortStream(TAvdtpLink* pLink, int streamIndex, uint16_t channel);    
bool avdtp_StartStream(TAvdtpLink* pLink, int streamIndex, uint16_t channel);   
bool avdtp_GetConfiguration(TAvdtpLink* pLink, int streamIndex , uint16_t channel);  
bool avdtp_SendGetConfigResp(TAvdtpLink* pLink, uint8_t  transactId, uint16_t channel);    

bool avdtp_SuspendStream(TAvdtpLink* pLink, int streamIndex , uint16_t channel);
bool avdtp_SendSuspendStreamReject(TAvdtpLink* pLink, uint8_t  transactId, int streamIndex, uint16_t channel);
bool avdtp_SendSuspendStreamResp(TAvdtpLink* pLink, uint8_t  transactId, uint16_t channel);

bool avdtp_SendReconfigureResp(TAvdtpLink* pLink, uint8_t  transactId, uint16_t channel);
bool avdtp_SendReconfigureReject(TAvdtpLink* pLink, uint8_t  transactId, uint16_t channel);

PAvdtpChannel avdtp_ChannelFindStream(PAppAvdtp pAvdtp, TBdAddr *pRemoteBd);

bool avdtp_SendBadLength(TAvdtpLink* pLink, TAvdtpEventSignalID signalID, uint8_t  transactId, uint16_t channel);
bool avdtp_SendBadAcpId(TAvdtpLink* pLink, TAvdtpEventSignalID signalID, uint8_t  transactId, uint16_t channel);
bool avdtp_SendBadInUse(TAvdtpLink* pLink, TAvdtpEventSignalID signalID, uint8_t  transactId, uint16_t channel);
bool avdtp_SendBadNotInUse(TAvdtpLink* pLink, TAvdtpEventSignalID signalID, uint8_t  transactId, uint16_t channel);
bool avdtp_SendBadService(TAvdtpLink* pLink, TAvdtpEventSignalID signalID, uint8_t  transactId, uint16_t channel);
bool avdtp_SendNotSupportedCommand(TAvdtpLink* pLink, TAvdtpEventSignalID signalID, uint8_t transactId, uint16_t channel);
bool avdtp_SendBadMediaTransportFormat(TAvdtpLink* pLink, TAvdtpEventSignalID signalID, uint8_t transactId, uint16_t channel);

#endif
