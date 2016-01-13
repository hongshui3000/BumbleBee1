/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       gattdef.h
* @brief     
* @details   
*
* @author   	gordon
* @date      	2015-07-10
* @version	v0.1
*/

#ifndef __GATTDEF_H
#define __GATTDEF_H

#include <flags.h>
#include <os_message.h>
#include <os_pool.h>
#include <message.h>
#include <gatt.h>
#include <trace_binary.h>
#include <blueface.h>

#ifdef __cplusplus
extern "C" {
#endif


#define GATT_LAST_END_GROUP_HANDLE_FFFF  1  /**< 0: last end group handle is 0xFFFF */
#define GATT_NO_BUILTIN_SERVICE          0  /**< 0: GAP/GATT are NOT built-in services */

#define GATT_CONSECUTIVE_ATTRIB_HANDLES  1  /**< 0: generate handles from 1,..,max */
                                            /**<      without gaps                   */
#define GATT_CCC_BITS_TO_BTSEC_AT_CONN_END   1
#define GATT_ATT_TIMER_DISABLED          0  /**< 0: transaction timer disabled, */
                                            /**<      test only !!!               */
#define GATT_USE_LL_WSIZE_EQ_1           1  /**< 1: on link layer (L2CAP) interface */
                                            /**<       window size = 1 is used         */

#define ATT_MTU_SIZE_LE_DEFAULT   23

/** result codes used in some routines (must be < 0 !!!): */
#define  GATT_RCODE_APPLIC_VALUE      -1
#define  GATT_RCODE_CLIENT_CHAR_CFG   -2

/** offset used in some upstream msgs that allows upper layers ---*/
/** (e.g. BlueAPI) to re-use buffer (no copy ..). MUST be an   ---*/
/** even value (alignment, API packed structs conversion ..):  ---*/




/**GATT server internal attribute descriptor (must match with applic. view of
 * attribute (gatt.h)!
 */

typedef struct _ShortUUIDValue
{
	uint8_t   bType[UUID_16BIT_SIZE];
	uint8_t   bValueIncl[14];
} TShortUUIDValue;

typedef struct _LongUUID
{
	uint8_t   bType[UUID_128BIT_SIZE];
} TLongUUID;

typedef struct _Attrib
{
	uint16_t    wFlags;
	union
	{
		TShortUUIDValue   s;    /**< 16 bit UUID + included value */
		TLongUUID         l;    /**< 128 bit UUID                 */
	} TypeValue;
	uint16_t    wValueLen;        /**< length of value              */
	PVOID  pValueContext;    /**< ptr to value if not included or applic. supplied */
	                          /**< context if flag ATTRIB_FLAG_VALUE_APPL is set    */
	uint16_t    wPermissions;     /**< read/write permissions, encryption key size ..   */
} TAttrib, * PAttrib;


#if F_BT_BREDR_SUPPORT
/** SDP related definitions buffer for SDP record setup */
#define  GATT_SDP_BUFFER_SIZE_16    43    /**< 16 bit UUID record  */
#define  GATT_SDP_BUFFER_SIZE_128   57    /**< 128 bit UUID record */

typedef enum _GATTSdpState
{
	SdpStateReleased     = 0,
	SdpStateRegistering,
	SdpStateRegistered,
	SdpStateReleasing
} TGATTSdpState, * PGATTSdpState;

typedef struct _GATTSdp
{
	TGATTSdpState State;
	uint16_t          wHandleNext;      /**< next handle for database search */
	BOOL          Last;
} TGATTSdp, * PGATTSdp;
#endif


/**
 * GATT internal profile/service descriptor.
 * one descriptor may represent one or more services (entire profile ..)!
*/
typedef struct _GATTService    /**< descriptor for one or more services */
{
	uint16_t          wNbr;
	uint16_t          wUsed;
	PAttrib       pAttribFirst;          /**< 1. attribute (primary service) */
	uint16_t          wAttribCnt;            /**< # of attributes                */
	uint16_t          wHandleStart;          /**< 1. attribute handle            */
#if F_BT_BREDR_SUPPORT
	TGATTSdp      Sdp;                   /**< BR/EDR SDP related data        */
#endif
} TGATTService, * PGATTService;

#if (GATT_CONSECUTIVE_ATTRIB_HANDLES)
#define GATT_ATTR_HANDLE_NBR_GET(h)       gattHandleToServiceNbrIdx(TRUE, h)
#define GATT_ATTR_HANDLE_IDX_GET(h)       gattHandleToServiceNbrIdx(FALSE, h)
#else
/**
*	internally an attribute handle is composed of a base value and the
*	index of its service(s) descriptor in the TGATTService array: stored 
*	in the high nibble of the attrib. handle:
*/
#define GATT_ATTR_HANDLE_NBR_MASK         0xF000
#define GATT_ATTR_HANDLE_NBR_PUT(h,n)     h &= ~GATT_ATTR_HANDLE_NBR_MASK; \
                                          h = ((uint16_t)(h | (n << 12)))
#define GATT_ATTR_HANDLE_NBR_GET(h)       \
                              ((uint16_t)((GATT_ATTR_HANDLE_NBR_MASK & h) >> 12))
#define GATT_ATTR_HANDLE_IDX_GET(h)       ((h & ~GATT_ATTR_HANDLE_NBR_MASK)-1)
#endif


/**GATT remote client related definitions */
/** data related to current transaction */
typedef struct _GATTTransRead       /**< remote client read */
{
	PGATTService   pGATTService;
	uint16_t           wHandle;           /**< current attribute handle   */
	uint16_t           wEndingHandle;
	uint16_t           wReadOffset;       /**< read offset in attribute   */
	uint16_t           wUUIDLength;       /**< 2 or 16                    */
	uint8_t           bUUID[UUID_128BIT_SIZE];
	/* result/work buffer data */
	uint16_t           wRemainingSize;
	uint16_t           wSize;             /**< <>0: size of list element  */
	uint16_t           wLastValueLength;  /**< length of last attribute value */
	uint16_t           wTxOffset;         /**< <>0: next write location   */
	uint8_t *         pWorkBuffer;
} TGATTTransRead, * PGATTTransRead;

typedef struct _GATTTransWrite      /**< remote client write */
{
	PGATTService   pGATTService;
	uint16_t           wHandle;          	/**< current attribute handle  */
	uint16_t           wWriteOffset;      	/**< write offset in attribute */
	                                	/**< (PrepareWrite only ...)   */
	uint16_t           wCCCBits;
} TGATTTransWrite, * PGATTTransWrite;

typedef struct _GATTCurrTrans       /**< GATT/ATT related data of current procedure */
{
	uint16_t            wTransType;      /**< type of transaction in progress   */
	union                            /**< parameters depending on wProcType */
	{
		TGATTTransRead    read;
		TGATTTransWrite   write;
	} p;
} TGATTCurrTrans, * PGATTCurrTrans;

typedef struct _GATTCurrInd        	/**< GATT related data of current handle/value */
{                                  		/**< indication                                */
	BOOL            ConfPending;     	/**< TRUE: busy, waiting for confirmation      */
#if (F_BT_GATT_SERVICE_CHANGED)
	BOOL            ServiceChanged;  	/**< TRUE: indications is triggered by "service changed" */
#endif
	uint16_t            wApplHandle;
	uint16_t            wAttribIndex;    	/**< index of attrib. in service structure   */
	PVOID          pServiceHandle;
	PVOID          pReqHandle;      	/**< req. handle from GATT_ATTRIB_UPDATE_REQ */
	                               		/**< (NOT attribute handle !!)               */
} TGATTCurrInd, * PGATTCurrInd;


typedef struct _GATTRClient
{
	/** each remote client maps uniquely to a bearer channel: */
	struct _GATTL2CChannel * pL2CChannel;
	uint8_t                     bReserved;
	/** order of elements before this line MUST be preserved !!!*/

	/** security related data */
	uint8_t             bSecurityFlags;
	uint8_t             bEncyptKeysize;

	/** CCCD data */
	uint8_t             bCCCBitsModified;

#if (F_BT_GATT_SERVICE_CHANGED)
	/** handle ServiceChanged characteristic: <> 0 when CCCD was written, */
	/**  < 0 : delete handle from keystore, > 0: add handle                */
	int              iServiceChangedHandle;
#endif

	/** data related to current client transaction (read, write ..) */
	TGATTCurrTrans   CurrTrans;

	/** data related to current handle/value indication */
	BOOL             TimerStarted;
    void *           timerHandle;
	TGATTCurrInd     CurrInd;
} TGATTRClient, * PGATTRClient;

/** bSecurityFlags bits */
#define GATT_RCLIENT_SEC_AUTHEN           0x01
#define GATT_RCLIENT_SEC_AUTHEN_MITM      0x02
#define GATT_RCLIENT_SEC_ENCRYPTED        0x04


/** instantiations of Client Characteristic Confirmation Bits */
typedef struct _GATTCCCBitsTable
{
	uint8_t       bUsed;
	uint8_t       bFlags;
	TBdAddr    Bd;
	uint16_t       wHandle;
	uint16_t       wCCCBits;
} TGATTCCCBitsTable, * PGATTCCCBitsTable;

/** bFlags bits */
#define GATT_CCCBITS_CLIENT_WRITE_ONLY    0x01  /**< only client may overwrite value, */
                                                /**< not DEVICE_DATA_RESP msg         */


typedef enum _GATTCCCBitsOp   /**< operations performed with CCC bits */
{
	CCCBitsClear,
	CCCBitsCount,
	CCCBitsAppend
} TGATTCCCBitsOp;

/** GATT server specific control data */
#define GATT_DEVICE_NAME_LENGTH     40


/** GATT_ATTRIB_UPDATE_REQ/CONF specific data  used for tx bursts */
typedef struct _GATTAttribUpdData
{
	uint16_t              applHandle;
	uint16_t              cause;
	PVOID            reqHandle;      /**< request handle, not attribute handle!! */
	PGATTService      pGATTService;
	uint16_t              attribIndex;    /**< index of attrib. in service structure  */
#if 0
	uint16_t              attribHandle;   /* attribute handle */
#endif
	uint8_t *            pBuffer;
	uint16_t              length;
	uint16_t              offset;         /**< offset of payload in (OSIF) buffer */
} TGATTAttribUpdData, * PGATTAttribUpdData;


typedef struct _GATTServer
{
    TGATTRClient      * pRClientDon;
    TGATTRClient      * pRClientDoff;

    TGATTService      * pService;
    TGATTCCCBitsTable  *pCCCBits;



	TGATTAttribUpdData AttribUpdData;
} TGATTServer, * PGATTServer;





/**GATT client specific definitions */

/** discovery (services, relationship ..) related parameters */
typedef struct _GATTDiscoveryParam
{
	uint16_t    wType;           /**< type of discovery */
	uint16_t    wStartingHandle;
	uint16_t    wEndingHandle;
	uint16_t    wUUIDType;       /**< 16/128 bit */
	uint8_t *  pUUID;
} TGATTDiscoveryParam, * PGATTDiscoveryParam;


/** GATT/ATT related data of current procedure */
typedef struct _GATTProcDiscovery  /**< discovery */
{
	uint16_t            wLastHandle;     /**< used for discovery continuation */
	uint16_t            wUUIDType;       /**< 16/128 bit */
	union
	{
		uint8_t     b16[UUID_16BIT_SIZE]; /**< network representation !!!!! */
		uint8_t     b128[UUID_128BIT_SIZE];
	}               UUID;
} TGATTProcDiscovery, * PGATTProcDiscovery;

typedef struct _GATTProcRead       /**< read .. */
{
	uint16_t            wReadType;       /**< type of read (basic, blob ..) */
	uint16_t            wHandle;         
	                               /**<"Read Multiple Request" !!  */
	uint16_t            wValueOffset;
} TGATTProcRead, * PGATTProcRead;

typedef struct _GATTCurrProc       /**< GATT/ATT related data of current procedure */
{
	uint8_t            bOpCode;         /**< <>0: ATT operation in progress    */
	uint8_t            bMTUReqCnt;      /**< <>0: ATT_EXCHANGE_MTU_REQUEST op. in progress */
	uint16_t            wProcType;       /**< type of procedure in progress     */
	union                            /**< parameters depending on wProcType */
	{
		TGATTProcDiscovery  discovery;
		TGATTProcRead       read;
	} p;
} TGATTCurrProc, * PGATTCurrProc;


/** GATT client specific control data */
typedef struct _GATTClient
{
	/** each local client maps uniquely to a bearer channel: */
	struct _GATTL2CChannel * pL2CChannel;
	uint8_t                     bReserved;
	/** order of elements before this line MUST be preserved !!! */

	/** GATT/ATT related data of current procedure (discovery, read ..) */
	BOOL             TimerStarted;
	TGATTCurrProc    CurrProc;
    void *           timerHandle;
} TGATTClient, * PGATTClient;


/** GATT_ATTRIB_REQ/type=command specific data used for tx bursts */
typedef struct _GATTAttribWriteData
{
	uint8_t *            pBuffer;
	uint16_t              length;
	uint16_t              offset;         /**< offset of payload in (OSIF) buffer */
} TGATTAttribWriteData, * PGATTAttribWriteData;



/** q of deferred applic. msgs (waiting for other modules to respond etc.) */
typedef struct _GATTDeferredMsg
{
	struct _GATTDeferredMsg *pNext;
	PGATTService             pGATTService;
} TGATTDeferredMsg, * PGATTDeferredMsg;

#define  GATT_MAX_DEFERRED_MSGS   2     


/** q of tx data (notifications/indications or write commands) */
typedef enum _GATTTxDataType
{
	TxTypeWriteCommand,
	TxTypeNotification
} GATTTxDataType;

typedef struct _GATTTxData
{
	struct _GATTTxData *pNext;
	GATTTxDataType      TxDataType;
	union
	{

		TGATTAttribUpdData    AttribUpdData;


		TGATTAttribWriteData  AttribWriteData;

		int iDummy;
	} p;
} TGATTTxData, * PGATTTxData;

/** 
* windows size that may be used for ATT_HANDLE_VALUE_NOTIFICATIONs 
* and ATT_WRITE_COMMANDs:                                          
*/


/** deferred link layer (LL) msg (waiting for other modules to respond etc.) */
typedef struct _GATTDeferredMsgLL
{
	uint8_t *  pBuffer;
	uint16_t    wFlags;
	uint16_t    wLength;
	uint16_t    wOffset;
} TGATTDeferredMsgLL, * PGATTDeferredMsgLL;


/** L2CAP channel (ATT bearer) data */
typedef enum _TGATTL2CState
{
	gattL2CStateIdle = 0,
	/**gattL2CStateListen,*/
	gattL2CStateConConfPending,
	gattL2CStateConnecting,
	gattL2CStateDiscPending,
	gattL2CStateDisconnecting,
	gattL2CStateConnected,           /**< L2CAP connected  */
} TGATTL2CState;

typedef struct _GATTL2CChannel
{
	uint16_t            wNbr;
	uint16_t            wUsed;

	PGATTClient     pClient;         /**< associated local client */
	PGATTRClient    pRClient;        /**< associated remote client */
    PGATTTxData pTxData;        //add--4.1 feature

	uint16_t            wReserved;

	/** order of elements before this line MUST be preserved !!! */

	TGATTL2CState   L2CState;
	uint16_t            wConRespStatus;  /**< status rcvd in GATT_CON_RESP msg */
	TBdAddr         RemoteBd;        /**< Bluetooth address of remote side */
	                               	/**< (network byte order)             */
#if (F_BT_LOW_ENERGY)
	uint8_t            bdType;          /**< BLUEFACE_BDTYPE_* */
#endif
	BOOL                Ready;           /**< TRUE: msgs from LL can be processed */
	TGATTDeferredMsgLL  DeferredMsgLL;
#if F_BT_BREDR_SUPPORT
	uint16_t            psm;
#endif
	uint16_t            cid;             /**< channel ID used by L2CAP/BR and BTMAN */
	uint16_t            wMTUSizeMax;     /**< local maximum MTU size   */
	uint16_t            wMTUSize;        /**< MTU size used/negotiated */

	BOOL            ATTTimeout;      /**< TRUE: ATT transaction timeout occurred */

	BOOL            DevDataGetRspPending; /**< TRUE: DEVICE_DATA_GET_RESP msg pending */


#if (F_BT_LOW_ENERGY)
	BOOL            OutgoingCall;
	uint16_t            wLEChannel;
	uint16_t            wLEConnectionHandle;
	BOOL            slaveConnectionUpdate;
	TGATTLEConnectionUpdateParam conParam;
#endif

	/* q of tx data (notifications/indications or write commands) */
	QUEUE_T         TxDataQueueFree;
	QUEUE_T         TxDataQueue;
//	TGATTTxData     TxData[GATT_MAX_TX_WSIZE];

} TGATTL2CChannel, * PGATTL2CChannel;

/** for LE map channel nbr. to msg parameter cid, used by BTMAN for routing: */
#define GATT_IDX_TO_LE_CID(x)       (x+1)

#if (F_BT_LOW_ENERGY)
/** Advertising state machine */
typedef struct _TGATTLEAdvSM
{
	uint8_t                        enabled;
	uint16_t                        opCode;
	uint16_t                        opCodeCurrent;
	uint16_t                        status;
	TGATTLEAdvertiseParameter   param;
	TGATTLEAdvertiseDirected    directed;
} TGATTLEAdvSM;

/** Scan state machine */
typedef struct _TGATTLEScanSM
{
	uint8_t                        enabled;
	TGATTLEScanReq              param;
} TGATTLEScanSM;

/** whitelist state machine */
typedef struct _TGATTLEWhitelistSM
{
	uint16_t                        opCode;
} TGATTLEWhitelistSM;
#endif /**< (F_BT_LOW_ENERGY) */


/**
* main GATT control structure
*/

/** server/client control data structures */
typedef struct _GATTSrvClData
{
	TGATTServer   Server;


	/* each local client maps uniquely to a bearer channel: */
    TGATTClient *pClientDon;
    TGATTClient *pClientDoff;


} TGATTSrvClData, * PGATTSrvClData;

/** GATT instance data */
typedef struct _TGATT
{
	MESSAGE_T     Message;         /**< current msg           */
	uint8_t *        pBuffer;         /**< setup for some (rx) messages that carry payload */

#if F_BT_BREDR_SUPPORT
	/** q of deferred msgs (currently used for context of SDP record creation only) */
	QUEUE_T           DeferredMsgQueueFree;
	QUEUE_T           DeferredMsgQueue;
	TGATTDeferredMsg  DeferredMsg[GATT_MAX_DEFERRED_MSGS];
#endif

	/** L2CAP channels (ATT bearers) ---*/
	uint16_t              wRxOffset;
    TGATTL2CChannel   *pL2CChannelDon;
    TGATTL2CChannel   *pL2CChannelDoff;

#if (F_BT_LOW_ENERGY)
	/**
	*L2CAP/HCI don't provide connection handle before LE_CONNECTION_COMPLETE_EVENT 
	* event => prior to this CONFs and LE_DISCONNECT_REQ can't be assigned to       
	* initiating LE_CREATE_CONNECTION => channel must be locked for serialization 
	* in this phase :-( ...   
	*/
	PGATTL2CChannel   pLELockedChannel;

	TGATTLEAdvSM      LE_adv;
	TGATTLEScanSM     LE_scan;

	TGATTLEWhitelistSM  LE_whitelist;
#endif
	/** server/client specific control data */
	PGATTServer     pServer;   /**< shortcut into SrvCl.Server */

	TGATTSrvClData  SrvCl;

	/** miscellaneous */
	uint8_t *          pWorkBuffer;
} TGATT;
typedef TGATT * PGATT;



int  gattSendDEVICE_DATA_IND( uint16_t wCommand, uint8_t bDataType,
                                   BOOL Delete, PGATTL2CChannel pL2CChannel,
                                   uint16_t wHandle, PGATTAttribUpdData pAttribUpdData );

void gattSendGATT_ATTRIB_UPDATE_CONF( PGATTAttribUpdData pAttribUpdData,
                          int  iCount,
                          PDEVICE_DATA_ELEMENT_GATT_CCC_BITS_REV pElement );
void gattSendGATT_ATTRIB_UPDATE_STATUS_IND( PGATTRClient pRClient,
                                                         uint16_t wStatus );
void gattSendGATT_ATTRIB_READ_IND( PGATTL2CChannel pL2CChannel );
void gattSendGATT_ATTRIB_WRITE_IND( PGATTRClient pRClient,
                                       uint16_t wType, int iLength,
                                       uint16_t writeOffset, uint8_t * pAttValue );
void gattSendGATT_CCCD_IND( PGATTL2CChannel pL2CChannel, BOOL Client,
                            int iCount, PDEVICE_DATA_ELEMENT_GATT_CCC_BITS pElement );

int          gattGATTServiceBuiltinInit(void);
#if (F_BT_LOW_ENERGY)
void gattSendGATT_LE_ADVERTISE_CONF(uint16_t opCode, uint16_t wCause);
#endif /**< (F_BT_LOW_ENERGY) */

void gattSendGATT_CON_IND( uint16_t cid, TBdAddr *pBd, uint8_t bdType );
void gattSendGATT_CON_ACT_IND( PGATTL2CChannel pL2CChannel );


#if (F_BT_LOW_ENERGY)
void gattSendGATT_MTU_SIZE_IND( uint16_t mtuSize, uint16_t cid );
#endif /**< (F_BT_LOW_ENERGY) */


void gattSendGATT_DISCOVERY_CONF( uint16_t status, uint16_t cid, uint16_t type );
uint16_t gattSendGATT_DISCOVERY_IND( uint16_t status, uint16_t cid,
                             uint16_t type, uint16_t count, uint16_t length, uint8_t * pList, PGATTCurrProc  pCurrProc );
void gattSendGATT_ATTRIB_READ_CONF( uint16_t status, uint16_t cid, uint16_t type,
                                       int iLength, int iTruncated, uint8_t * pData,
                                       int iNbrOfHandles, LPWORD pHandles,
                                       int iValueOffset );
BOOL gattSendGATT_ATTRIB_WRITE_CONF( uint16_t status, uint16_t cid,
                                     uint16_t type, int length, uint8_t * pPartAttValue );



#if (F_BT_LOW_ENERGY)
void gattSendGATT_LE_CONNECTION_UPDATE_CONF(uint16_t update_cause, uint16_t cid);
void gattSendGATT_LE_CONNECTION_UPDATE_IND(PGATTL2CChannel pL2CChannel,
                                           uint16_t type, PGATTLEConnectionUpdateParam pParam);
void gattSendGATT_LE_MODIFY_WHITELIST_CONF(uint16_t opCode, uint16_t cause);
#endif /**< (F_BT_LOW_ENERGY) */

/** GATT_LL.C */
int gattLLConnectReq( PGATTL2CChannel pL2CChannel, PCON_REQ_GATT pConPara);
void gattLLConnectConf( PGATTL2CChannel pL2CChannel, uint16_t wStatus );
void gattLLConnectInd( PGATTL2CChannel pL2CChannel );
void gattLLConnectResp( PGATTL2CChannel pL2CChannel, uint16_t wStatus );
void gattLLConnected( PGATTL2CChannel pL2CChannel, int iLevel );
void gattLLDisconnectReq( PGATTL2CChannel pL2CChannel, uint16_t wStatus );
void gattLLDisconnectInd( PGATTL2CChannel pL2CChannel, uint16_t wStatus );
void gattLLDisconnected( PGATTL2CChannel pL2CChannel, uint16_t wStatus,
                                         BOOL DiscConf, BOOL ForceCleanup );
int  gattLLBufferCallbackSet( TBufferCallBack CallbackFunction,
                                     PGATTL2CChannel pL2CChannel, uint8_t * pBuffer );
uint8_t * gattLLBufferGet( int iLength );
void gattLLDataSend( PGATTL2CChannel pL2CChannel,
                                     uint8_t * pBuffer, int iLength, uint16_t wOffset );
BOOL gattLLDataReceived( PGATTL2CChannel pL2CChannel,
                         uint8_t * pBuffer, uint16_t wLength, uint16_t wOffset );

#if (F_BT_LOW_ENERGY)
/** GATT_LE.C */
int  gattSendLE_SET_ADVERTISING_PARAMETERS( PGATTLEAdvertiseParameter pAdvParam,
                                            LPBdAddr bd, uint8_t bdType);
int  gattSendLE_SET_ADVERTISE_ENABLE( uint8_t Enable );
int  gattSendLE_SET_SCAN_ENABLE( uint8_t Enable, uint8_t filterDups );
int  gattSendLE_SET_SCAN_PARAMETERS( PGATTLEScanReq pScanParam );

int  gattSendLE_ADD_DEVICE_TO_WHITELIST( LPBdAddr bd, uint8_t bdType );
int  gattSendLE_REMOVE_DEVICE_FROM_WHITELIST( LPBdAddr bd, uint8_t bdType );
int  gattSendLE_CLEAR_WHITELIST(void);

int  gattSendLE_CREATE_CONNECTION(PGATTL2CChannel pL2CChannel, PCON_REQ_GATT pConPara );
int  gattSendLE_CREATE_CONNECTION_CANCEL(PGATTL2CChannel pL2CChannel );

int  gattSendLE_DATA(PGATTL2CChannel pL2CChannel,
                               uint8_t * pBuffer, uint16_t wLength, uint16_t wOffset );
int  gattSendLE_DISCONNECT(PGATTL2CChannel pL2CChannel, uint16_t wStatus );

#endif


/** GATTPROC.C */

void gattServerConfiguration(PGATTL2CChannel pL2CChannel );
uint16_t gattDiscoveryPrimaryService(PGATTL2CChannel pL2CChannel,
                                  PGATTDiscoveryReq   pDiscoveryReq,
                                  PGATTDiscoveryParam pParam );
uint16_t gattDiscoveryOthers(PGATTL2CChannel pL2CChannel,
                                PGATTDiscoveryReq   pDiscoveryReq,
                                PGATTDiscoveryParam pParam );
uint16_t gattAttribRead(PGATTL2CChannel pL2CChannel,
                                          PGATTAttribReadReq  pAttribReadReq );
uint16_t gattAttribWrite(PGATTL2CChannel      pL2CChannel,
                                   uint16_t type, uint16_t handle, uint16_t writeOffset,
                                   PGATTAttribWriteData pAttribWriteData );
void gattHandleATTErrorResponseOrTimeout(PGATTL2CChannel pL2CChannel,
                                 BOOL            Timeout,
                                 uint8_t            bReqOpcode,
                                 uint8_t            bErrorCode,
                                 uint16_t            wHandle );
void gattHandleGenericResponse( PGATTL2CChannel  pL2CChannel,
                                int              iCount,
                                int              iLength,
                                uint8_t *           pList );
void gattHandleReadByTypeResponse( PGATTL2CChannel  pL2CChannel,
                                   int              iCount,
                                   int              iLength,
                                   int              iTruncated,
                                   uint8_t *           pAttDataList );
void gattHandleReadResponse( PGATTL2CChannel pL2CChannel,
                             uint16_t            wType,
                             int             iLength,
                             uint8_t *          pValue );
BOOL gattHandleWriteResponse( PGATTL2CChannel pL2CChannel,
                              uint16_t            wType,
                              int             iLength,
                              uint8_t *          pValue );
void gattHandleNotifInd( PGATTL2CChannel pL2CChannel, BOOL Notify,
                                   uint16_t wHandle, int iLength, uint8_t * pValue );


int  gattAttribNotifInd( PGATTL2CChannel     pL2CChannel,
                         uint16_t                wCCCBits,
                         uint16_t                wHandle,
                         PAttrib             pAttrib,
                         PGATTAttribUpdData  pAttribUpdData,
                         BOOL          * pReleaseBuffer
                       );
void  gattHandleAttribConf(PGATTL2CChannel pL2CChannel, uint16_t wStatus );
uint32_t gattHandleReadByGroupTypeRequest( PGATTL2CChannel pL2CChannel,
                                        uint16_t   wStartingHandle,
                                        uint16_t   wEndingHandle,
                                        int    iAttGroupTypeLength,
                                        uint8_t * pAttGroupType );
uint32_t gattHandleFindByTypeValueRequest( PGATTL2CChannel pL2CChannel,
                                        uint16_t   wStartingHandle,
                                        uint16_t   wEndingHandle,
                                        uint16_t   wUUID,
                                        int    iAttValueLength,
                                        uint8_t * pAttValue );
uint32_t gattHandleReadByTypeRequest( PGATTL2CChannel pL2CChannel,
                                   uint16_t            wStartingHandle,
                                   uint16_t            wEndingHandle,
                                   int             iAttTypeLength,
                                   uint8_t *          pAttType );
uint32_t gattHandleFindInfoRequest( PGATTL2CChannel pL2CChannel,
                                 uint16_t            wStartingHandle,
                                 uint16_t            wEndingHandle );
uint32_t gattHandleReadRequest( PGATTL2CChannel pL2CChannel,
                             uint16_t            wType,
                             uint8_t *          pPDU );
uint32_t gattHandleWriteReqCmd( PGATTL2CChannel pL2CChannel,
                             uint16_t            wType,
                             uint16_t            wHandle,
                             int             iLength,
                             uint16_t            wWriteOffset,
                             uint8_t *          pAttValue,
                             BOOL          * pReleaseBuffer );

uint32_t gattHandlePrepareWriteReq( PGATTL2CChannel pL2CChannel,
                                 uint16_t            wHandle,
                                 int             iLength,
                                 uint16_t            wWriteOffset,
                                 uint8_t *          pAttValue,
                                 BOOL          * pReleaseBuffer );


uint32_t gattHandleExecuteWriteReq( PGATTL2CChannel pL2CChannel,
                                 uint8_t            bFlags );
BOOL gattHandlePrepareWriteResponse( PGATTL2CChannel pL2CChannel,
                                     uint16_t            wType,
                                     uint16_t            valueOffset,
                                     int             iLength,
                                     uint8_t *          pValue );
void gattHandleExecuteWriteResponse( PGATTL2CChannel pL2CChannel );
uint16_t gattExecuteWrite( PGATTL2CChannel pL2CChannel, uint8_t flags);
void gattExecuteWriteResumeAppl( PGATTL2CChannel pL2CChannel,
                                 uint16_t cause, uint16_t handle);
uint32_t  gattCheckPrepareWrite( PGATTRClient pRClient, uint16_t wHandle);
void gattSendGATT_ATTRIB_PREPARE_WRITE_CONF( uint16_t status, uint16_t cid,
        uint16_t valueOffset, int length, uint8_t * pPartAttValue );
void gattSendGATT_EXECUTE_WRITE_IND( PGATTL2CChannel pL2CChannel,
                                     uint8_t            bFlags );
void gattSendGATT_EXECUTE_WRITE_CONF( uint16_t status, uint16_t cid );


//uint32_t gattHandleExecuteWriteReq( PGATTL2CChannel pL2CChannel,
//                                 uint8_t            bFlags );
//void gattExecuteWriteResumeAppl( PGATTL2CChannel pL2CChannel,
//                                      PGATTExecuteWriteResp pExecuteWriteResp );
void gattReadResumeAppl( PGATTL2CChannel pL2CChannel,
									uint16_t              cause,             
									uint16_t              length,            
									PGATTAttribReadRespParam  pParam   
									);
void gattWriteResumeAppl( PGATTL2CChannel pL2CChannel,
									uint16_t  cause, uint16_t length, uint16_t offset, uint8_t * pBuffer);

#if (!GATT_CCC_BITS_TO_BTSEC_AT_CONN_END)
void gattWriteResumeBTSEC( PGATTRClient pRClient, uint16_t wCause );
#endif

void            gattL2CChangeState( PGATTL2CChannel pL2CChannel,
                                                      TGATTL2CState L2CState );
PGATTL2CChannel gattL2CChannelAllocate( BOOL LE );
void            gattL2CChannelFree( PGATTL2CChannel pL2CChannel );
PGATTL2CChannel gattL2CChannelFindHandle( BOOL LE, uint16_t wHandle );
int          gattL2CChannelFindConnIndNotif( uint16_t wHandle,
                                             PGATTL2CChannel * ppL2CChannel );
PGATTL2CChannel  gattL2CChannelFindBD( LPBdAddr pBdAddr );
int          gattWorkBufferAlloc( int iSize );
void         gattWorkBufferFree(void);
uint16_t         gattServiceCheck( PAttrib pAttrib, int iCnt );
PGATTService gattServiceAlloc( void );
void         gattServiceFree( PGATTService pGATTService );
void         gattServiceInit( PGATTService pGATTService );

int          gattServicePrepare( PGATTService     pGATTService,
                                              PGATTDeferredMsg pDeferredMsg );

uint32_t        gattAttributesSearch( BOOL SdpSearch, PGATTRClient pRClient,
                                     BOOL LE, uint16_t wType, BOOL Primary,
                                     LPWORD pwStartingHandle, uint16_t wEndingHandle,
                                     int    iCmpValueLength, uint8_t * pCmpValue,
                                     int   *piCnt,
                                     uint8_t * pList, int *pSize, int iListEntrySize );
uint32_t        gattAttributeRead( PGATTRClient pRClient,
                                              uint16_t wHandle, int iOffset,
                                              uint8_t * pBuffer, int *piSize );
uint32_t        gattAttributeWrite( PGATTRClient pRClient,
                                  uint16_t wHandle, uint8_t * pAttValue, int *piSize, BOOL Cmd );

uint16_t         gattCCCBitsGet( BOOL DirectAccess, uint16_t wHandle,
                                                            LPBdAddr pBdAddr );
int          gattCCCBitsGetFlags( PGATTL2CChannel pL2CChannel, uint16_t wHandle );
int          gattCCCBitsSet( PGATTL2CChannel pL2CChannel,
                                           BOOL Client, uint16_t wHandle, uint16_t wCCCBits );
int          gattCCCBitsOp( TGATTCCCBitsOp Op,
                                    PGATTL2CChannel pL2CChannel, uint8_t * pList );
BOOL         gattAttribIsType16( PAttrib pAttrib, uint16_t wUUID );
PAttrib      gattGoToAttribute( int iDirection, uint16_t wHandle,
                                             uint16_t wUUID, uint16_t * pwDstHandle );
PAttrib      gattHandleToAttribute( uint16_t wHandle );
#if (GATT_CONSECUTIVE_ATTRIB_HANDLES)
int          gattHandleToServiceNbrIdx( BOOL ToNbr, uint16_t wHandle );
#endif
uint16_t         gattAttributeToHandle( PAttrib pAttrib );
int          gattCharPropertiesGet( PAttrib pAttrib );
BOOL         gattDeferMsgIfChannelNotReady( uint8_t * pData );
void         gattDeferredMsgLLSave( PGATTL2CChannel pL2CChannel, MESSAGE_P pMsg);
void         gattDeferredMsgLLFree( PGATTL2CChannel pL2CChannel,
                                                             BOOL ReleaseBuffer );
void         gattDeferredMsgLLProcess( PGATTL2CChannel pL2CChannel );

uint8_t         gattCauseToATTError( uint16_t wCause );
void         gattInitData( void);


PGATTL2CChannel gattLEChannelFind( uint16_t wHandle );
void gattSendGATT_LE_MODIFY_WHITELIST_CONF(uint16_t opCode, uint16_t cause);
void gattSendGATT_SERVICE_REGISTER_CONF( uint16_t         wCause,
                                                PGATTService pGATTService);
void gattSendGATT_ATTRIB_NOTIF_IND( uint16_t cid, BOOL Notify,
                                  uint16_t wHandle, int iLength, uint8_t * pValue );

#if (F_BT_BREDR_SUPPORT)
/** GATT_BR.C */
int  gattSendL2C_CON_REQ( PGATTL2CChannel pL2CChannel );
int  gattSendL2C_CON_RESP( PGATTL2CChannel pL2CChannel,
                                                      uint16_t wStatus, uint16_t cid );
void gattSendL2C_DISC_REQ( PGATTL2CChannel pL2CChannel, uint16_t wStatus );
void gattSendL2C_DATA_REQ( PGATTL2CChannel pL2CChannel,
                           uint8_t * pBuffer, uint16_t wLength, uint16_t wRxOffset );
int          gattSDPCreateDes( PGATTService pGATTService,
                               PVOID pHandle );
#endif

#ifdef __cplusplus
}
#endif

#endif

