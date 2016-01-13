/**
*****************************************************************
*	Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************
* @file       l2c.c
* @brief     Bluetooth L2CAP Layer
* @details   
*
* @author   	gordon
* @date      	2015-07-13
* @version	v0.1
*/

#ifndef __L2C_H
#define __L2C_H

#include <flags.h>
#include <os_message.h>
#include <os_queue.h>
#include <message.h>
#include <bt_msg.h>

#if (F_BT_L2C_EXT_FEATURE_SUPPORT)
#include <l2c_extf.h>
#endif
#include <l2c_code.h>

#ifdef __cplusplus
extern "C" {
#endif



#if !defined(BT_L2C_MAX_CMD_RETRIES_COUNT)
#define BT_L2C_MAX_CMD_RETRIES_COUNT   0      /**< maximum number of retries for a request */
#endif /**< !defined(BT_L2C_MAX_CMD_RETRIES_COUNT) */



/** L2CAP protocol opcode Definitions */
#define L2CAP_COMMAND_REJECT                         0x01
#define L2CAP_CONNECTION_REQUEST                     0x02
#define L2CAP_CONNECTION_RESPONSE                    0x03
#define L2CAP_CONFIGURE_REQUEST                      0x04
#define L2CAP_CONFIGURE_RESPONSE                     0x05
#define L2CAP_DISCONNECTION_REQUEST                  0x06
#define L2CAP_DISCONNECTION_RESPONSE                 0x07
#define L2CAP_ECHO_REQUEST                           0x08
#define L2CAP_ECHO_RESPONSE                          0x09
#define L2CAP_INFORMATION_REQUEST                    0x0a
#define L2CAP_INFORMATION_RESPONSE                   0x0b
#define L2CAP_CONNECTION_PARAMETER_UPDATE_REQUEST    0x12
#define L2CAP_CONNECTION_PARAMETER_UPDATE_RESPONSE   0x13
#define L2CAP_LE_CREDIT_BASED_CONNECTION_REQUEST     0x14
#define L2CAP_LE_CREDIT_BASED_CONNECTION_RESPONSE    0x15
#define L2CAP_LE_FLOW_CONTROL_CREDIT                 0x16

#define L2CAP_INFOTYPE_CONNECTIONLESS_MTU         0x0001
#define L2CAP_INFOTYPE_EXTENDED_FEATURES_SUPPORT  0x0002

/** Bitmasks for the BT 1.2 info type extended feature mask */
#define L2CAP_EX_FEATURE_FLOWCONTROL            0x0001
#define L2CAP_EX_FEATURE_RETRANS                0x0002
#define L2CAP_EX_FEATURE_BIDIR_QOS              0x0004
#define L2CAP_EX_FEATURE_ENHANCED_RETRANS       0x0008
#define L2CAP_EX_FEATURE_STREAMING              0x0010
#define L2CAP_EX_FEATURE_OPTIONAL_FCS           0x0020

#define L2CAP_MODE_BASIC                        0x00    /**< Basic L2CAP mode */
#if F_BT_L2C_ENHANCED_FEATURE_SUPPORT   
#define L2CAP_MODE_RETRANSMISSION               0x01    /**< Retransmission mode */
#define L2CAP_MODE_FLOWCONTROL                  0x02    /**< Flow control mode */
#endif
#define L2CAP_MODE_ENHANCED_RETRANSMISSION      0x03    /**< Enhanced Retransmission mode */
#define L2CAP_MODE_STREAMING                    0x04    /**< Streaming mode */
#define L2CAP_RTX_TIME  40              /**< Response Timeout timer value spec says 1 to 60 sec */
#define L2CAP_ERTX_TIME 60              /**< extended Response Timeout timer value spec says 60 to 300 sec */
#define L2CAP_INFO_RESP_RTX_TIME  1     /**< Response Timeout for pending Info Request */
#define L2CAP_RTX_TID           0x80    /**< timer ID for RTX timer */
#define L2CAP_CONFIG_TID        0x81    /**< timer ID for RTX timer */
#define CLOSEDELAY_TID          0x82    /**< timer ID for closing HCI channel */
#define L2CAP_MONITOR_TID        0x83    /**< timer ID for monitor timer */
#define L2CAP_RETRANSMISSION_TID 0x84    /**< timer ID for retransmission timer */
#define L2CAP_ACK_TID            0x85    /**< timer ID for ACK timer */
#define L2CAP_TC_TID             0x86    /**< timer ID conformance test timer */



/** dynamically allocated */
#define CID_MIN 0x40        /**< see L2CAP spec start of allocated space  */
#define CID_MAX 0xFF        /**< restricted due to our UPL I/F and internal use as
                               timer channel ID */
#define CID_LE_MAX 0x7F
#define L2CAP_MAX_ENHANCED_FEATURE_CHANNELS   5     /**<Maximum nbr of concurrent channel for enhanced feature*/

#define L2CAP_SIGNAL_MTU    48
#define L2CAP_SIGNAL_MTU_LE 23

#define MTU_DEFAULT 672
#define FLUSHTIMEOUT_DEFAULT 0xffff /**< Always retransmit */

#define L2C_WRITE_OFFSET (BT_L1_HCI_DS_OFFSET + ACL_HDR_LENGTH + L2CAP_HDR_LENGTH)

#define L2CAP_CONFIG_C_FLAG_MASK 0x01

/** size of the Header in byte as send over the air (16 bit length, 16 bit channel ID) */
typedef struct
{
	uint16_t length;
	uint16_t cid;
} T_L2C_HEADER;

#if (F_BT_BREDR_SUPPORT)||(F_BT_LE_BT41_SUPPORT)
typedef struct                  /**< parameter set of L2CAP Disconnect Request / Response*/
{
	uint16_t dcid;                  /**< Destination CID */
	uint16_t scid;                  /**< Source CID */
} T_L2CAP_DiscR_para, *P_L2CAP_DiscR_para;
#endif

typedef struct 
{
    uint8_t code;
    uint8_t id;                            /**< unique L2CAP id to match command and response */
    uint16_t length;
} T_L2CAP_Command, *P_L2CAP_Command;

typedef enum 
{
    undef,
    originate,
    terminate,
    pingreq,//for bredr
    inforeq //for bredr
} TAG_ROLE;

typedef enum _TL2cState
{
    l2cStateClosed,
    l2cStateConfig,//for bredr
    l2cStateOpen,
    l2cStateWaitForL2C_CON_RESP,//for bredr
    l2cStateWaitForL2C_DISC_RESP,
    l2cStateWaitForL2CAP_CONNECTION_RESPONSE,//for bredr
    l2cStateWaitForL2CAP_DISCONNECTION_RESPONSE,
#if F_BT_LE_BT41_SUPPORT
    l2cStateWaitForL2C_CON_LE_DATA_CHANNEL_RESP,
    l2cStateWaitForL2CAP_LE_CREDIT_BASED_CONNECTION_RESPONSE,
    l2cStateWaitForSECMAN_AUTHENTICATION_RESP
#endif
} TL2cState;

#if F_BT_LE_BT41_SUPPORT
typedef struct _L2CTxData
{
    struct _L2CTxData *pNext;
    uint8_t *            pBuffer;
    uint16_t              length;
    uint16_t              offset;   
} TL2CTxData, * PL2CTxData;
#endif

typedef struct _TL2C *PL2C;
typedef const struct _TL2C *PCL2C;

typedef struct _L2CAP_CHANNEL * P_L2CAP_CHANNEL;    /**< forward declaration */
typedef const struct _L2CAP_CHANNEL * PC_L2CAP_CHANNEL;    /**< forward declaration */


typedef struct aclPoolDesc 
{
    enum { aclStateFree = 0, aclStateAllocated, aclStateOpening, aclStateInuse, aclStateClosePending} state ;
    uint16_t        uses;   
    uint16_t        handle;
    TBdAddr     remote_bd;                  /**< remote Bluetooth adress */
#if F_BT_BREDR_SUPPORT
     uint32_t       remoteExtendedFeatures;     /**< feature vector from remote side, got via info request*/
     uint16_t        mtu;                        /**< Maximum transmission unit size of all L2cap Channels */
    uint16_t        FlushTO;                    /**< Flush TimeOut */
    uint16_t        LinkTO;                     /**< Link  TimeOut */
    uint8_t        closeId;
#endif

    uint16_t        signalingCID;
    uint8_t        conType;
#if (F_BT_LOW_ENERGY)
    uint8_t        role;
                                            /**< Used only for Connection Parameters Update */
    uint8_t        SigIdSent;                  /**< last sent command id */
#endif
    uint8_t        ReqIdRcv;                /**< last received Request id, used incase we do not have a channel descriptor yet */
    
    struct 
   	{
        uint16_t    len;                        /**< length of actual data in pd */
        uint16_t    expected;                   /**< number of bytes we expect for this PDU */
        uint16_t    offset;
        uint8_t    isLeChannel;
        uint8_t *  pd;                         /**< pointer to contents buffer */
#if (F_BT_BREDR_SUPPORT)||(F_BT_LE_BT41_SUPPORT)
        P_L2CAP_CHANNEL pChan;              /**< pointer to the channel descriptor */
#endif
    } usPakOpen;
    uint8_t        dsBufQueue;                 /**< queue to buffer downstream data pakets */
    uint8_t        dsPrioBufQueue;             /**< priority queue to buffer downstream data pakets */
    MESSAGE_T   dsMessage;
    void * RTXTimerHandle;
#if F_BT_BREDR_SUPPORT
    void * CloseDelayTimerHandle;//for bredr
#endif
} T_ACLPOOLDESC;

typedef T_ACLPOOLDESC * P_ACLPOOLDESC;
typedef CONST T_ACLPOOLDESC * PC_ACLPOOLDESC;

#define HCI_HANDLE_INVALID 0xFFFF             /**< an unused HCI handle */

#define L2CAP_CHANNEL_MAGIC 0x1561
#define PARANOIA_CHECK_CHANNEL(channel) (channel->magic != L2CAP_CHANNEL_MAGIC)

typedef struct 
{                        /**< space to store the last send command for repeation */
	uint16_t len;
	uint8_t buf[L2CAP_SIGNAL_MTU];
} TCmdBuf;
typedef TCmdBuf * LPCmdBuf;
typedef CONST TCmdBuf * LPCCmdBuf;

typedef struct _L2CAP_CHANNEL
{
	struct _L2CAP_CHANNEL * pNext;        /**< link pointers */
	uint16_t magic;
	P_ACLPOOLDESC pHciDesc;               /**< pointer to the HCI ACL connection descriptor */
	uint16_t LocalCid;
	uint16_t RemoteCid; 
	uint16_t LocalMtu;
    uint16_t RemoteMtu;
    
    TL2cState    State;
    TAG_ROLE role;                        /**< our role: org or terminate */

	uint8_t SigIdSent;                       /**< last sent command id */
	uint8_t ReqIdRcv;                        /**< last received Request id*/
	uint8_t RespIdRcv;                       /**< last received Response id*/

	uint8_t authenticationActive;
    void * RTXTimerHandle;
    TCmdBuf lastRspSend; //fixme                 /**< space to store the last send response for repeation */
#if F_BT_BREDR_SUPPORT    
	/* in case of incomming connections next 2 elements are copies of values in upperLayer Desciptor uplDesc */
	uint16_t psm;                             /**< PSM associated to this channel */
	uint16_t uuid;                            /**< UUID associated to this channel */
	uint8_t usQueueID;               /**< upstream queue id associated with this channel */
	                     /**< max MTU upstream as requested by upper layer*/
	uint16_t LocalUsMtu;                      /**< MTU used upstream (maybe < LocalMtu if reported by remote site)  */
	uint16_t LocalDsMtu;                      /**< MTU used downstream  */
	                      /**< max MTU downstream */
	uint16_t RemoteFlushTimeout;              /**< POSOPT belongs into the HCI desc */
	uint8_t Mode;
	uint8_t MaxDsBuffer;                     /**< max. downstream buffer */
	T_L2C_Conf_para ConfParaLocal;        /**< from CON/LISTEN_REQ */
	T_L2C_Conf_para ConfParaRemote;       /**< from remote */
	uint16_t CurrentMode;
	uint16_t RespMode;                        /**<  L2CAP_CONFIGURE_RESP */
	uint16_t UnacceptableMode;                /**< Last unacceptable mode from remote side */


	/** Flag Section - some flags that control flow and behaviour */
	/** POSOPT BOOL values may be changed to bit field to reduce RAM usage */
	BOOL ConfReqReady;    /**< used to indicate that we received positive rsp on our
	                     req */
	BOOL ConfRspReady;    /**< used to indicate that we replied positive on a config
	                     req */
	void * CONFIGTimerHandle;

 	BOOL L2capInformationRequestSent;  /**< waiting for L2CAL_INFORMATION_RESPONSE */
	BOOL SendL2capConfigureRequest;    /**< send L2CAL_CONFIGURE_REQUEST */
	BOOL SendL2capConfigureResponse;   /**< send L2CAL_CONFIGURE_RESPONSE */
 	BOOL ACLDiscImmediate;/**< TRUE = disconnect the ACL link Immediate to speed up flow */
	BOOL OpenL2cDiscConf; /**< we have to send the L2C_DISC_CONF                         */
    
	
	uint16_t status;                          /**< a paramter to transfer info between states                         */
	uint16_t ExtFLength;                      /**< != 0 for FC or R */
#if (F_BT_L2C_ENHANCED_FEATURE_SUPPORT)
	uint16_t                 RespTxWindowSize; /**<  L2CAP_CONFIGURE_RESP */
	uint16_t                 RespMaxPDUSize;   /**<  L2CAP_CONFIGURE_RESP */
	uint16_t                 Fcs;
	uint16_t                 FcsLength;
	P_L2CAP_CHANNEL_EXT  pExtF;
#endif
#endif

#if (BT_L2C_MAX_CMD_RETRIES_COUNT != 0)
	uint8_t ReqCmdRetryCount;                /**< retry counter, counts down to 0              */
	TCmdBuf lastReqSend;                  /**< space to store the last send command for repeation */
#else /**< (BT_L2C_MAX_CMD_RETRIES_COUNT != 0) */
	uint8_t  lastReqSend;                    /**< command code of the last Request we send out */
	uint16_t  lastConfigFlagSend;             /**< last flag field send in L2CAP_CONFIGURE_REQUEST */
#endif /**< (BT_L2C_MAX_CMD_RETRIES_COUNT != 0) */


    

#if F_BT_LE_BT41_SUPPORT
    BOOL WaitDisRsp;
    uint16_t le_psm;                             /* PSM associated to this channel */

    uint16_t LocalMps;
    uint16_t LocalInitialCredits;
	uint16_t InitialCredits;
    uint16_t creditsIncrease;

    uint16_t RemoteMps;
    uint16_t RemoteInitialCredits;

    struct
    {
        uint16_t    len;                        /* length of actual data in pd */
        uint16_t    expected;                   /* number of bytes we expect for this PDU */
        uint16_t    offset;
        uint8_t *  pd;                         /* pointer to contents buffer */
    } usLEFrames;
    MESSAGE_T   dsMessage;

	TL2CTxData *pTxData;
	QUEUE_T         TxDataQueueFree;
    QUEUE_T         TxDataQueue;
#endif
} T_L2CAP_CHANNEL;

#if F_BT_BREDR_SUPPORT
typedef struct _upl 
{           					/**< descriptor for registered Upper Protocol Layers */
	uint16_t          psm;            /**< Protocol Service Multiplexer, 0 = unused entry */
	uint8_t  queueID;        /**< queue ID for input queue of the protocol layer */
} TupperlayerDesc;
#endif

typedef struct _TL2C 
{
    MESSAGE_T       Message;

#if (F_BT_L2C_ENHANCED_FEATURE_SUPPORT)
    uint8_t            SFramePool;
#endif

#if F_BT_BREDR_SUPPORT    
    TupperlayerDesc *uplDesc;     /**< array of decriptors for registered upper protocols */
#endif
    T_ACLPOOLDESC   *pAclDescDon;
    T_ACLPOOLDESC   *pAclDescDoff;

#if (F_BT_BREDR_SUPPORT)||(F_BT_LE_BT41_SUPPORT)
    T_L2CAP_CHANNEL *pLLDataChanDon;
    T_L2CAP_CHANNEL *pLLDataChanDoff;

    QUEUE_T         ChannelListHdr;                 /**< header pointers for channel list */
    P_L2CAP_CHANNEL pChannelListCur;                /**< work pointer to Channel list */


#endif
    uint8_t            L2cap_next_cmdID;               /**< Command ID to be used for next cmd
                                                       !! it is used global and not channel
                                                       orientated cause timers are identified
                                                       using the command ID
                                                                                            */
    uint8_t            dsAclPoolID;
    uint16_t            leDsMtu;                        /**< Mtu of the lower protocol    */
#if F_BT_BREDR_SUPPORT
    uint16_t            dsMtu;                          /**< Mtu of the lower protocol    */

#if (F_BT_L2C_ENHANCED_FEATURE_SUPPORT)
    T_L2CAP_CHANNEL_EXT  ExtF[L2CAP_MAX_ENHANCED_FEATURE_CHANNELS];
#endif
#endif
    uint8_t            mbuf[L2CAP_SIGNAL_MTU];

} TL2C;


void l2cStopRTXTimeout(P_L2CAP_CHANNEL pChan);
void l2cSendL2CAP_PDU(uint16_t HciHandle, MESSAGE_P pMsg, uint16_t L2CapSize, uint16_t cid, BOOL startPak);
void l2cFragmentDATA_REQ(P_ACLPOOLDESC pHciDesc);
void l2cRemoveHCI(P_ACLPOOLDESC pHciDesc, uint8_t CloseId);
void l2cSendSECMAN_AUTHENTICATION_IND(P_L2CAP_CHANNEL pChan, uint8_t outgoing, uint8_t active);
void l2cSendL2CAPMessage(P_L2CAP_CHANNEL pChan, P_ACLPOOLDESC pHciDesc, uint8_t cmd_code, LPCBYTE para_buf, uint16_t length, uint16_t timeout);
void l2cSendL2C_DISC_IND(PC_L2CAP_CHANNEL pChan, uint16_t status);
void l2cChangeState(P_L2CAP_CHANNEL pChan, TL2cState NewState);


void l2cSendL2C_DISC_CONF(uint16_t usQueueID, uint16_t LocalCid, uint16_t result);
void l2cRemoveChan(P_L2CAP_CHANNEL pChan, BOOL HCIDisconnected);


void l2cReSendL2CAPMessage(PC_L2CAP_CHANNEL pChan, LPCCmdBuf pLastCmd);
void l2cSendL2CAP_DISCONNECTION_REQUEST(P_L2CAP_CHANNEL pChan);
void l2cSendL2CAP_DISCONNECTION_RESPONSE(P_L2CAP_CHANNEL pChan);
void l2cSendL2CAP_COMMAND_REJECT(P_L2CAP_CHANNEL pChan, P_ACLPOOLDESC pHciDesc, uint16_t reason, uint16_t para1, uint16_t para2);
void l2cStartRTXTimeout(P_L2CAP_CHANNEL pChan, uint16_t secs);


#include <l2cchman.h>
#if (F_BT_LOW_ENERGY)
#include <l2c_le.h>
#endif

#ifdef __cplusplus
}
#endif

#endif
