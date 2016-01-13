/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       bt_msg.h
* @brief     Bluetooth Upper Layer Protocol Stack Specific Message definitions
* @details   
*
* @author   	gordon
* @date      	2015-07-09
* @version	v0.1
*/

#ifndef __BT_MSG_H
#define __BT_MSG_H

#include <bttypes.h>                /**< definition of Bluetooth specific types */
#include <blueface.h>               /**< definition of types transparent to blueface layer */ /* POSOP move types to bttypes.h */

/** BLUETOOTH SECURITY MANAGER */
/** Register MID and RCV QUEUEID to BT_SECMAN */
#define SECMAN_AUTHENTICATION_REQ    (SECMAN_TYPE|BM_AUTHENTICATION|REQUEST)
#define SECMAN_AUTHENTICATION_IND    (SECMAN_TYPE|BM_AUTHENTICATION|INDICATION)

/** RFCOMM protocol layer */
//#define RFC_DATA_REQ                    (RFC_TYPE|BM_DATA|REQUEST)

/** GATT protocol layer */
#define GATT_SERVICE_REGISTER_REQ       (GATT_TYPE|BM_REGISTER|REQUEST)

/** HCI protocol layer */
/** Requests */
#define HCI_DATA_REQ                     (HCI_TYPE|BM_DATA|REQUEST)

/** Indications */
#define HCI_DATA_IND                     (HCI_TYPE|BM_DATA|INDICATION)
#define HCI_CONFIGURATION_IND            (HCI_TYPE|BM_CONFIGURATION|INDICATION)

/** Security Interface */
#define HCI_AUTH_REQ                     (HCI_TYPE|BM_AUTHENTICATION|REQUEST)

#if (F_BT_LOW_ENERGY)
/** Bluetooth Low Energy */
#define LE_MESSAGE_REQ                   (BLE_TYPE|BM_LE_MESSAGE|REQUEST)
#define LE_MESSAGE_CONF                  (BLE_TYPE|BM_LE_MESSAGE|CONFIRMATION)
#define LE_MESSAGE_IND                   (BLE_TYPE|BM_LE_MESSAGE|INDICATION)
#endif

/** BTSEC device data */
#define DEVICE_DATA_GET_IND             (SEC_TYPE|BM_DEVICE|INDICATION)
#define DEVICE_DATA_GET_RESP            (SEC_TYPE|BM_DEVICE|RESPONSE)
#define DEVICE_DATA_SET_IND             (SEC_TYPE|BM_CONFIGURATION|INDICATION)
#define DEVICE_DATA_SET_RESP            (SEC_TYPE|BM_CONFIGURATION|RESPONSE)


/**  BLUETOOTH PROTOCOL STACK MESSAGE DEFINITION */
/** BLUETOOTH Stack Generic Message Argument Definitions */
/**
*	NAME CONVENTIONS:
*
*	cid     = Channel Identifier - from lower layer assigend handle to identify the connection
*	channel =                      protocol specific info to adress a special link,
*                                  PSM on L2CAP, server channel on RFCOMM etc
*	status  =                      the result of a request or indication, any state, condition and result codes
*/

/** this is config stuff L2CAP */
typedef struct                  /**< Quality of Service Options */  /* !! do not use in in overlay union */
{
    uint8_t    flags;
    uint8_t    serviceType;
    uint32_t   tokenRate;
    uint32_t   tokenBucket;
    uint32_t   peakBandwidth;
    uint32_t   latency;
    uint32_t   delayVariation;
} T_L2C_flowSpec, *P_L2C_flowSpec;

/** L2CAP mode bits definition */
                                       /**< (1 << mode) */
#define L2C_MODE_BASIC                    (1 << 0x00)    /**< Basic L2CAP mode */
#if 0    /**< not supported */
#define L2C_MODE_RETRANSMISSION           (1 << 0x01)    /* Retransmission mode */
#define L2C_MODE_FLOWCONTROL              (1 << 0x02)    /* Flow control mode */
#endif
#define L2C_MODE_ENHANCED_RETRANSMISSION  (1 << 0x03)    /**< Enhanced Retransmission mode */
#define L2C_MODE_STREAMING                (1 << 0x04)    /**< Streaming mode */

typedef struct
{
    uint16_t    mode;                    /**< Bit mask */
    uint8_t    txWindowSize;            /**< 0: default */
    uint8_t    maxTransmit;             /**< 0; default 2 */
    uint16_t    retransmissionTimeout;   /**< 0: default */
    uint16_t    monitorTimeout;          /**< 0: default */
    uint16_t    maxPDUSize;              /**< 0: default */
} T_L2C_flowControl, *P_L2C_flowControl;


typedef struct                  /**< !! do not use in overlay union */
{
    uint16_t             reqId;
    uint16_t             mtuSize;
    struct             /**< Bitmask defining valid fields                    */
    {
        unsigned FlushTO         :1;
        unsigned Qos             :1;
        unsigned FlowControl     :1;
        unsigned Fcs             :1;
        unsigned reserved        :12;
    } validMask;
    uint16_t FlushTO;               /**< Flush timeout in ms
                                         0 = use existing otherwise default 0xFFFF
                                         1 = perform no retransmission
                                    	   0xffff = retransmit until link timeout          */

    T_L2C_flowSpec    flowSpec;
    T_L2C_flowControl flowControl;
    uint8_t              fcs;      /**< frame check sequence: 0 - no FCS */
                                /**<                       1 - 16-bit FCS */
    uint8_t maxDsBuffer;           /**< max. downstream buffer */
} T_L2C_Conf_para, *P_L2C_Conf_para;
typedef T_L2C_Conf_para /**<XXXX */ * LP_L2C_Conf_para;

#define LINKTIMEOUT_DEFAULT  0x7D00     /**< ~ 20 sec timeout */

/** Connect INDICATION */

typedef struct                  /**< RFCOMM specific part of ConInd  */
{
    uint16_t frameSize;             /**< frameSize negotiated for this connection */
    uint8_t creditBased;
    uint8_t outgoing;
} TBtConIndRfc;

typedef struct                  /**< L2CAP specific part of ConInd  */
{
    uint8_t id;                    /**< connection ID - only used for TCI other may ignore */
} TBtConIndL2cap;

typedef struct                  /**< HCI specific part of ConInd  */
{
    TDevClass devClass;
} TBtConIndHCI;

typedef struct                  /**< GATT specific part of ConInd  */
{
    uint8_t bdType;                /**< BLUEFACE_BDTYPE_* */
    uint16_t mtuSize;               /**<XXXXMJMJ */
} TBtConIndGATT;

typedef struct                  /**< parameter set of CONNECT INDICATION */
{
    uint16_t    cid;                /**< Identifier for this connection */
    TBdAddr bd;                 /**< Source BlueTooth address       */
    uint16_t    channel;            /**< protocol specific channel info
                                    RFCOMM: serverchannel
                                    BNEP:   Destination UUID, Service on the local side to connect to
                                    AVDTP:  lower byte: flags bit 0: 0 = Signaling Channel, 1 = Stream Channel
                                            upper byte: acpSeid
                                    HCI:    linktype
                                    L2CAP:  psm
                                    TCSBIN: psm
                                 */
    union
    {   
    	/* protocol specific part */
    	TBtConIndRfc    rfc;
    	TBtConIndL2cap  l2cap;
    	TBtConIndHCI    hci;
    	TBtConIndGATT   gatt;
    }p;
} TBtConInd, *PBtConInd;

/** Connect RESPONSE */
typedef struct                  /**< RFCOMM specific part of ConResp  */
{
    uint16_t frameSize;
    uint8_t mscStatus;
    uint8_t maxCredits;
} TBtConRespRfc;

typedef struct                  /**< L2CAP specific part of ConResp  */
{
    uint16_t             statusDetails;   /**< details of status - used if connection pending for values see L2CAP spec */
    LP_L2C_Conf_para pConfPara;       /**< Configuration parameter */
} TBtConRespL2cap;

typedef union
{   
	/**< protocol specific part */
    TBtConRespRfc   rfc;
    TBtConRespL2cap l2cap;
} TBtConRespPSpecifc, * PBtConRespPSpecifc;

typedef struct                  /**< L2CAP specific part of ConActInd  */
{
    LP_L2C_Conf_para    pConfPara;     /**< parameter */
    uint16_t                maxPDUSize;    /**< incoming */
    uint8_t                txWindowSize;  /**< incoming */
} TBtConActIndL2CAP;


typedef struct _GATTConActIndParam
{
	TBdAddr bd;
	uint8_t    bdType;
	uint16_t    credits;        /**< applicable for WriteCommand and Notification only */
	uint16_t    connInterval;
	uint16_t    connLatency;
	uint16_t    supervisionTimeout;
} TGATTConActIndParam, * PGATTConActIndParam;

typedef struct
{
	/**< due to sizeof(MESSAGET_T) limitations more parameters in pBuffer :-( .. : */
	uint8_t *  pParam;
} TBtConActIndGATT;

typedef struct
{
    uint16_t    cid;                /**< Identifier for this connection           */
    uint16_t    status;             /**< 0 = connection ready                     */
    uint16_t    frameSize;          /**< negotiated frameSize                     */
                                /**< L2cap = MTU size incoming                */
    uint16_t    channel;            /**< protocol specific channel info */
    union
    {
    	TBtConActIndL2CAP l2cap;
    	TBtConActIndGATT gatt;
    } p;
} TBtConActInd, *PBtConActInd;
typedef TBtConActInd  * LPBtConActInd;

/** SDP IF Message Argument Definitions */
typedef struct        /**< !! not use in in overlay union */
{
    uint16_t linkid;
    uint16_t maxHandles;
    uint16_t uuidLen;
    uint8_t uuidList[1]; /**< artificial size */
} TsdpSearchReq;
typedef TsdpSearchReq  * LPsdpSearchReq;

typedef struct        /**< !! not use in in overlay union */
{
    uint16_t linkid;
    uint16_t totalHandles;
    uint32_t handles[1];
} TsdpSearchConf;
typedef TsdpSearchConf  * LPsdpSearchConf;

typedef struct        /**< !! not use in in overlay union */
{
    uint16_t linkid;
    uint32_t handle;
    uint16_t maxCount;      /**< Max. no of bytes to return */
    uint16_t aLen;
    uint8_t aList[1];
} TsdpAttributeReq;
typedef TsdpAttributeReq  * LPsdpAttributeReq;

typedef struct        /**< !! not use in in overlay union */
{
    uint16_t linkid;
    uint16_t byteCount;
    uint8_t aList[1];    /**< !! component must be on uint32_t boundary ! */
} TsdpAttributeConf;
typedef TsdpAttributeConf  * LPsdpAttributeConf;

typedef struct          /**< !! not use in in overlay union */
{
    uint16_t linkid;
    uint16_t maxCount;      /**< Max. no of bytes to return */
    uint16_t sLen;          /**< Length of Service List */
    uint16_t aLen;          /**< Length of Attribute List */
                      /**< !! component must be on uint32_t boundary ! */
    uint8_t lists[1];      /* service pattern (first) and attribute pattern (second) */
} TsdpSearchAttributeReq;
typedef TsdpSearchAttributeReq  * LPsdpSearchAttributeReq;

typedef struct          /**< !! not use in in overlay union */
{
    uint16_t linkid;
    uint16_t byteCount;
    uint8_t aList[1];    /**< !! component must be on uint32_t boundary ! */
} TsdpSearchAttributeConf;
typedef TsdpSearchAttributeConf  * LPsdpSearchAttributeConf;

/** RFCOMM IF Message Argument Definitions */
#if defined(PRAGMA_PACK)
#pragma pack(1)
#endif

/** RPN command */
typedef PACKED(struct)
{
    uint8_t dlci;
    uint8_t baudrate;
    uint8_t oc3;
    uint8_t flctl;
    uint8_t xon;
    uint8_t xoff;
    uint8_t pm1;
    uint8_t pm2;
} Trpn;
typedef Trpn  * LPrpn;
typedef CONST Trpn  * LPCrpn;

#if defined(PRAGMA_PACK)
#pragma pack()
#endif


/** GATT IF Message Argument Definitions */



typedef struct _GATTAttribUpdateConfParam
{
	PVOID            serviceHandle;
	uint16_t              attribIndex;   /**< index of attrib. in service structure   */
	uint16_t              count;         /**< nbr. of elements in list (<> 0 only for */
                                   /**< some cause values ..)                   */
	uint16_t              offset;        /**< offset of first list element in list[]  */
	uint8_t              list[1];       /**< <BD,type> depending on cause value      */
} TGATTAttribUpdateConfParam, * PGATTAttribUpdateConfParam;

typedef struct _GATTAttribReadRespParam
{
	PVOID            serviceHandle;
	uint16_t              offset;         /**< offset of attrib. value in data[]      */
	uint8_t              data[1];
} TGATTAttribReadRespParam, * PGATTAttribReadRespParam;

typedef struct _GATTAttribWriteIndParam  /**< (some) GATTAttribWriteInd parameters */
{                                        /**< (due to sizeof(MESSAGE_T) limitations .. ) */
	uint16_t              attribIndex;    /**< index of attrib. in service structure */
	uint16_t              type;           /**< type of write (REQ,CMD,PREP..)        */
	uint16_t              length;         /**< attribute value/data length           */
	uint16_t              writeOffset;    /**< write offset in attribute             */
	uint16_t              offset;         /**< offset of attrib. value in data[]     */
	uint8_t              data[1];
} TGATTAttribWriteIndParam, * PGATTAttribWriteIndParam;





/** client messages */
/** generic parameters <handle range, UUID> */
typedef struct _GATTGenericParamRangeUUID
{
	uint16_t  startingHandle; /**< handles MUST be first in struct !! */
	uint16_t  endingHandle;
	uint16_t  uuidType;       /**< 16/128 bit */
	union
	{
    	uint16_t     uuid16;
    	uint8_t *   pUuid128;
	} uuid;
} TGATTGenericParamRangeUUID, * PGATTGenericParamRangeUUID;


/** discovery (services, relationships, characteristics ..) */
/** discovery requests, confirmations */
typedef PACKED(struct) _GATTDiscoveryReqCommon
{
	uint16_t  startingHandle; /**< all discovery req. have these common parameters, */
	uint16_t  endingHandle;   /**< used to save some code ....                      */
} TGATTDiscoveryReqCommon;

/** all primary services */
typedef TGATTDiscoveryReqCommon     TGATTDiscoveryReqPSrvAll,
                                                * PGATTDiscoveryReqPSrvAll;
typedef TGATTGenericParamRangeUUID  TGATTDiscoveryReqPSrvUUID,
                                                * PGATTDiscoveryReqPSrvUUID;

/** relationship */
typedef TGATTDiscoveryReqCommon TGATTDiscoveryReqRelation,
                                              * PGATTDiscoveryReqRelation;

/** all characteristics */
typedef TGATTDiscoveryReqCommon TGATTDiscoveryReqCharAll,
                                              * PGATTDiscoveryReqCharAll;


typedef struct _GATTDiscoveryReq
{
	uint16_t  cid;
	uint16_t  type;          /**< type of discovery */
	union
	{
    	TGATTDiscoveryReqPSrvAll    psrvAll;
    	TGATTDiscoveryReqPSrvUUID   psrvUUID;
    	TGATTDiscoveryReqRelation   relation;
    	TGATTDiscoveryReqCharAll    charAll;
    	TGATTDiscoveryReqCommon     common;
	} p;
} TGATTDiscoveryReq, * PGATTDiscoveryReq;

/** read attribute value(s) */
typedef struct _GATTAttribReadReqBasic
{
	uint16_t  handle;
} TGATTAttribReadReqBasic, * PGATTAttribReadReqBasic;

typedef struct _GATTAttribReadReqBlob
{
	uint16_t  handle;
	uint16_t  valueOffset;
} TGATTAttribReadReqBlob, * PGATTAttribReadReqBlob;

typedef struct _GATTAttribReadReqMultiple
{
	uint16_t  nbrOfHandles;
	uint16_t  handle[1];
} TGATTAttribReadReqMultiple, * PGATTAttribReadReqMultiple;

typedef TGATTGenericParamRangeUUID  TGATTAttribReadReqType,
                                                * PGATTAttribReadReqType;

typedef struct _GATTAttribReadReq
{
	uint16_t  cid;
	uint16_t  type;          /**< type of read */
	union
	{
    	TGATTAttribReadReqBasic      basic;
    	TGATTAttribReadReqBlob       blob;
    	TGATTAttribReadReqMultiple   multiple;
    	TGATTAttribReadReqType       type;
	} p;
} TGATTAttribReadReq, * PGATTAttribReadReq;

typedef struct _GATTAttribReadConfParam
{
	uint16_t    valueOffset;   /**< read offset in attribute           */
	uint16_t    valueLength;   /**< attribute value length             */
	uint16_t    nbrOfHandles;  /**< nbr. of handles (and values)       */
	uint16_t    length;        /**< total length of handles and values */
	uint16_t    offset;        /**< offset of first handle in handlesData[] */
	uint8_t    handlesData[1];
} TGATTAttribReadConfParam, * PGATTAttribReadConfParam;

/** write attribute value(s) */
typedef struct _GATTAttribWriteReqParam
{
	uint16_t    writeOffset;   /**< write offset in attribute            */
	uint16_t    offset;        /**< offset of attrib. value in data[]    */
	uint8_t    data[1];
} TGATTAttribWriteReqParam, * PGATTAttribWriteReqParam;

typedef struct _GATTAttribWriteConfParam /**< (some) GATTAttribWriteConf parameters */
{                                        /**< (due to sizeof(MESSAGE_T) limitations .. ) */
	uint16_t    length;         /**< attribute value/data length           */
	uint16_t    offset;         /**< offset of attrib. value in data[]     */
	uint8_t    data[1];
} TGATTAttribWriteConfParam, * PGATTAttribWriteConfParam;

#if (F_BT_LOW_ENERGY) /**< [ */
typedef struct _GATTLEScanReq
{
//	uint16_t              applHandle;
	uint8_t              enable;
	uint8_t              scanType;
	uint16_t              scanInterval;
	uint16_t              scanWindow;
	uint8_t              filterPolicy;
    uint8_t              localBdType;
	uint8_t              filterDuplicates;
} TGATTLEScanReq, * PGATTLEScanReq;

typedef struct _GATTLEConnectionUpdateParam
{
	uint16_t    connIntervalMin;
	uint16_t    connIntervalMax;
	uint16_t    connLatency;
	uint16_t    supervisionTimeout;
	uint16_t    minimumCELength;
	uint16_t    maximumCELength;
} TGATTLEConnectionUpdateParam, * PGATTLEConnectionUpdateParam;
#endif /**< ] (F_BT_LOW_ENERGY) */

typedef struct _GATTExecuteWriteResp
{
	uint16_t    cid;
	uint16_t    applHandle;
	uint16_t    cause;
} TGATTExecuteWriteResp, * PGATTExecuteWriteResp;


/************************************************************************************************/
typedef struct
{
    uint16_t status;
    uint16_t handle;
    TBdAddr bd;
} ThciConActInd;
/** reqID and IndID values for HCI Configuration */
#define HCI_CONF_NO_CONNECTIONS          BLUEFACE_CONF_NO_CONNECTIONS         /**< US:   No of ACL connections      */

/** Configuration Indications */
typedef struct
{
    uint16_t indID;     /**< Type of Indication */
    uint16_t len;       /**< argument length */
    uint8_t * ptr;     /**< argument pointer */
} ThciConfigureInd;

/** Authentication Parameter Structures */
typedef struct
{
    TBdAddr bd;
    uint8_t id;
} ThciAuthReq;

/** Key Parameter Structures */
/** values for ThciKeyInd.reqType */
#define LINK_KEY_REQUEST                 1
#define PIN_CODE_REQUEST                 2
#define LINK_KEY_REQUEST_MITM            3

/** keyType used in THciNewKeyInd */
#define HCI_KEYTYPE_COMBINATION     BLUEFACE_KEYTYPE_COMBINATION      /**< bt2.0 key */
#define HCI_KEYTYPE_LOCAL_UNIT      BLUEFACE_KEYTYPE_LOCAL_UNIT
#define HCI_KEYTYPE_REMOTE_UNIT     BLUEFACE_KEYTYPE_REMOTE_UNIT
#define HCI_KEYTYPE_DEBUG           0x03                              /**< bt2.1 debug key */
#define HCI_KEYTYPE_UNAUTHENTICATED BLUEFACE_KEYTYPE_UNAUTHENTICATED  /**< bt2.1 ssp key */
#define HCI_KEYTYPE_AUTHENTICATED   BLUEFACE_KEYTYPE_AUTHENTICATED    /**< bt2.1 ssp mitm key */
#define HCI_KEYTYPE_CHANGED         BLUEFACE_KEYTYPE_CHANGED          /**< bt2.1 new key but same keytype */

#if (F_BT_LOW_ENERGY)
/** Address type */
#define LE_ADDRESS_TYPE_PUBLIC                0x00 /**< Public Device Address */
#define LE_ADDRESS_TYPE_RANDOM                0x01 /**< Random Device Address */

/** Advertising type */
#define LE_ADVERTISING_TYPE_ADV_IND                     0x00   /**< Connectable undirected */
#define LE_ADVERTISING_TYPE_ADV_DIRECT_HIGH_DUTY_IND    0x01   /**< Connectable directed */
#define LE_ADVERTISING_TYPE_ADV_SCAN_IND                0x02   /**< Scanable undirected */
#define LE_ADVERTISING_TYPE_ADV_NONCONN_IND             0x03   /**< None connectable undirected */
#define LE_ADVERTISING_TYPE_ADV_DIRECT_LOW_DUTY_IND     0x04   /* Connectable directed */

/** Advertising channel */
#define LE_ADVERTISING_CHANNEL_37             0x01   /**< Enable channel 37 */
#define LE_ADVERTISING_CHANNEL_38             0x02   /**< Enable channel 38 */
#define LE_ADVERTISING_CHANNEL_39             0x04   /**< Enable channel 39 */
#define LE_ADVERTISING_CHANNEL_ALL            0x07   /**< Enable all channels */

/** Advertising filter policy                                   Scan              Connect */
#define LE_ADVERTISING_FILTER_SCAN_ANY_CONNECT_ANY   0x00   /**< from Any        + from Any */
#define LE_ADVERTISING_FILTER_SCAN_WL_CONNECT_ANY    0x01   /**< from White List + from Any */
#define LE_ADVERTISING_FILTER_SCAN_ANY_CONNECT_WL    0x02   /**< from Any        + from White List */
#define LE_ADVERTISING_FILTER_SCAN_WL_CONNECT_WL     0x03   /**< from White List + from White List */

typedef struct _TLEReadAdvertisingChannelTxPowerLevel
{
	uint8_t    transmitPowerLevel;
} TLEReadAdvertisingChannelTxPowerLevel;

/** Advertising/Scan Response data */
#define LE_ADVERTISING_DATA_LENGTH   0x1F
#define LE_SCAN_RESPONSE_LENGTH      0x1F

/** Initiator filter policy */
#define LE_INITIATOR_FILTER_POLICY_WHITE_LIST_NOT_USED     0x00
#define LE_INITIATOR_FILTER_POLICY_WHITE_LIST_USED         0x01

typedef struct _TLEReadWhiteListSize
{
	uint8_t    whiteListSize;
} TLEReadWhiteListSize;

/** status for connection updata response */
#define LE_CONNECTION_UPDATE_ACCEPT      0x00
#define LE_CONNECTION_UPDATE_REJECT      0x01

typedef struct _TLESetHostChannelClassification
{
	uint8_t    channelMap[5];
} TLESetHostChannelClassification;

typedef struct _TLEReadChannelMap                      /**< Channel + Handle */
{
	uint8_t    channelMap[5];
} TLEReadChannelMap;

#define LE_ENCRYPT_KEY_LENGTH        0x10
#define LE_ENCRYPT_DATA_LENGTH       0x10

typedef struct _TLEEncrypt
{
	uint8_t    key[LE_ENCRYPT_KEY_LENGTH];
	uint8_t    plaintextData[LE_ENCRYPT_DATA_LENGTH];
} TLEEncrypt;

/** Random number */
#define LE_RANDOM_NUMBER_SIZE        8

/** Long Term Key */
#define LE_LONG_TERM_KEY_SIZE        16

/** States */
#define LE_STATES_SIZE        8

typedef struct _TLEReadSupportedStates
{
	uint8_t    states[LE_STATES_SIZE];
} TLEReadSupportedStates;

typedef struct _TLEReceiverTest
{
	uint8_t    rxFrequency;
} TLEReceiverTest;

typedef struct _TLETransmitterTest
{
	uint8_t    txFrequency;
	uint8_t    lengthOfTestData;
	uint8_t    packetPayload;
} TLETransmitterTest;

typedef struct _TLETestEnd
{
	uint8_t    numberOfPackets;     /**< receiver test only */
} TLETestEnd;

/** Role */
#define LE_ROLE_MASTER         0x00
#define LE_ROLE_SLAVE          0x01

typedef struct _TLEConnectionComplete                  /**< Channel + Handle */
{
	uint8_t    role;
	uint8_t    peerAddressType;
	TBdAddr peerAddress;
	uint16_t    connInterval;
	uint16_t    connLatency;
	uint16_t    supervisionTimeout;
	uint8_t    masterClockAccuracy;
} TLEConnectionComplete;

/** Event type */
#define LE_EVENT_TYPE_ADV_IND           0x00  /**< Connectable undirected */
#define LE_EVENT_TYPE_ADV_DIRECT_IND    0x01  /**< Connectable directed */
#define LE_EVENT_TYPE_ADV_SCAN_IND      0x02  /**< Scannable undirected */
#define LE_EVENT_TYPE_ADV_NONCONN_IND   0x03  /**< None connectable undirected */
#define LE_EVENT_TYPE_ADV_SCAN_RSP      0x04  /**< Scan response */

typedef struct _TLEAdvertisingReport
{
	uint8_t    eventType;
	uint8_t    addressType;
	TBdAddr address;
	uint8_t    rssi;
	uint8_t    dataLength;
	uint8_t    data[1];
} TLEAdvertisingReport;

/** Features */
#define LE_FEATURES_SIZE        8

typedef struct _TLEReadRemoteUsedFeaturesComplete      /**< Channel + Handle */
{
	uint8_t    features[LE_FEATURES_SIZE];
} TLEReadRemoteUsedFeaturesComplete;

typedef union _TLEData
{
	TLEReadAdvertisingChannelTxPowerLevel   ReadAdvertisingChannelTxPowerLevel;  /**< Conf */
	TLEReadWhiteListSize                    ReadWhiteListSize;  /**< Conf */
	TLESetHostChannelClassification         SetHostChannelClassification;
	TLEReadChannelMap                       ReadChannelMap;
	TLEEncrypt                              Encrypt;
    TLEReadSupportedStates                  ReadSupportedStates;  /**< Conf */
    TLEReceiverTest                         ReceiverTest;
    TLETransmitterTest                      TransmitterTest;
    TLETestEnd                              TestEnd;

    TLEConnectionComplete                   ConnectionComplete;
    TLEAdvertisingReport                    AdvertisingReport;
    TLEReadRemoteUsedFeaturesComplete       ReadRemoteUsedFeaturesComplete;

    uint8_t                                    Data[1];
} TLEData;
typedef TLEData *LPLEData;


/* LE internal Commands (msgtype) */
#define LE_SET_RANDOM_ADDRESS                        0x0001
#define LE_SET_ADVERTISING_PARAMETERS                0x0002
#define LE_READ_ADVERTISING_CHANNEL_TX_POWER         0x0003
#define LE_SET_ADVERTISING_DATA                      0x0004
#define LE_SET_SCAN_RESPONSE_DATA                    0x0005
#define LE_SET_ADVERTISE_ENABLE                      0x0006
#define LE_SET_SCAN_PARAMETERS                       0x0007
#define LE_SET_SCAN_ENABLE                           0x0008
#define LE_CREATE_CONNECTION                         0x0009
#define LE_CREATE_CONNECTION_CANCEL                  0x000A
#define LE_READ_WHITE_LIST_SIZE                      0x000B
#define LE_CLEAR_WHITE_LIST                          0x000C
#define LE_ADD_DEVICE_TO_WHITE_LIST                  0x000D
#define LE_REMOVE_DEVICE_FROM_WHITE_LIST             0x000E
#define LE_CONNECTION_UPDATE                         0x000F  /* Channel + Handle */
#define LE_SET_HOST_CHANNEL_CLASSIFICATION           0x0010
#define LE_READ_CHANNEL_MAP                          0x0011  /* Channel + Handle */
#define LE_READ_REMOTE_USED_FEATURES                 0x0012  /* Channel + Handle */
#define LE_ENCRYPT                                   0x0013
#define LE_RAND                                      0x0014
#define LE_START_ENCRYPTION                          0x0015  /* Channel + Handle */
#define LE_LONG_TERM_KEY_REQUEST_REPLY               0x0016  /* Channel + Handle */
#define LE_LONG_TERM_KEY_REQUEST_NEGATIVE_REPLY      0x0017  /* Channel + Handle */
#define LE_READ_SUPPORTED_STATES                     0x0018
#define LE_RECEIVER_TEST                             0x0019
#define LE_TRANSMITTER_TEST                          0x001A
#define LE_TEST_END                                  0x001B

#define LE_CONNECTION_COMPLETE_EVENT                 0x0020  /* Channel + Handle */
#define LE_ADVERTISING_REPORT_EVENT                  0x0021
#define LE_CONNECTION_UPDATE_COMPLETE_EVENT          0x0022  /* Channel + Handle */
#define LE_READ_REMOTE_USED_FEATURES_COMPLETE_EVENT  0x0023  /* Channel + Handle */
#define LE_LONG_TERM_KEY_REQUEST_EVENT               0x0024  /* Channel + Handle */
#define LE_DATA_LENGTH_CHANGE_EVENT                  0x0025

#define LE_DATA                                      0x0030  /* Channel + Handle */
#define LE_DISCONNECT                                0x0031  /* Channel + Handle */
#define LE_ENCRYPTION_CHANGE                         0x0032  /* Channel + Handle */
#define LE_ENCRYPTION_KEY_REFRESH_COMPLETE           0x0033  /* Channel + Handle */


#define LE_VENDOR_SEND_HCI_CMD       0x0040
#define LE_VENDOR_SET_BLE_TX_POWER       0x0041

//
#define LE_SET_DATA_LENGTH                                                          0x0050
#define LE_READ_SUGGESTED_DEFAULT_DATA_LENGTH                  0x0051
#define LE_WRITE_SUGGESTED_DEFAULT_DATA_LENGTH                0x0052

typedef struct _TLEMessage
{
    uint16_t          msgType;
    uint16_t          handle;         /**< connection handle */
    uint16_t          channel;        /**< L2CAP cid */
	uint16_t          status;         /**< confirmation and connection update response only */
	TLEData    p;
} TLEMessage;
typedef TLEMessage *LPLEMessage;

/** min. offset for LE data request (DataCB.Offset)                          */
/** offsetof (TLEMessage, p) + BT_LE_MESSAGE_OFFSET >                        */
/** (HCI packet + HCI header + L2CAP Header = 1 + 4 + 4) */
#define BT_LE_MESSAGE_OFFSET        (BT_L1_HCI_DS_OFFSET  +2)

#endif /**< (F_BT_LOW_ENERGY) */


/** internal definitions */
/** Connection Type */
#define BT_CONNECTION_TYPE_BR_ACL   0
#define BT_CONNECTION_TYPE_BR_SCO   1
#if (F_BT_LOW_ENERGY)
#define BT_CONNECTION_TYPE_LE       2
#endif /**< (F_BT_LOW_ENERGY) */

/** Misc Structures */
typedef struct
{
	uint16_t status;
	uint16_t count;
	uint16_t maxHdl;
	uint32_t dummy; /**< align the following member to uint32_t boundary! */
	uint8_t bd[1];
} TinqBuf;
typedef TinqBuf  * LPinqBuf;

/** PH_ERROR_IND Codes */
#define BT_ERROR_IND_NOMEM               1
#define BT_ERROR_IND_NOSYNC              2

#define SECMAN_REQ_MITM                   0x01  /**< requires authenticated (mitm secure) linkkey */
#define SECMAN_REQ_FORCE_PAIRING          0x10  /**< do not use stored key */

typedef struct
{
 	TBdAddr bd;
	uint16_t    requirements;     /**< SECMAN_REQ_* link requirements */
    uint8_t    minKeySize;       /**< minimum link key size     */
	uint8_t	Source;
} TSecManAuthenticationReq;
typedef TSecManAuthenticationReq * PTSecManAuthenticationReq;

#define SECMAN_SOURCE_L2CAP                 0x01
#define SECMAN_SOURCE_RFCOMM                0x02
#define SECMAN_SOURCE_GATT                  0x03

typedef struct
{
    TBdAddr bd;
    uint16_t    ref;
    uint16_t    channelId;  /**< L2CAP: PSM, RFCOMM: DLCI */
    uint16_t    uuid;
    uint8_t    outgoing;   /**< ignore if deactivate */
    uint8_t    active;     /**< = 1 activate the authentication, = 0 deactivate */
    uint8_t    Source;     /**< 1:indication from l2cap, 2:indication from rfcomm*/
    uint8_t    conType;
} TSecManAuthenticationInd;
typedef TSecManAuthenticationInd * PTSecManAuthenticationInd;

/** BTSEC device data */
/** from blueface.h: */
typedef TDEVICE_DATA_IND  TDeviceDataInd,  * PDeviceDataInd;
typedef TDEVICE_DATA_RESP TDeviceDataResp, * PDeviceDataResp;

/** Macro with defines the Bluetooth stack specific members of the message parameter overlay M_DATA_T */
#define BT_MDATA_MEMBERS									\
	/** HCI IF */											\
	ThciConfigureInd            hciConfigureInd;			\
	ThciAuthReq                 hciAuthReq;					\
	/** SecMan */											\
	TSecManAuthenticationReq    secManAuthenticationReq;	\
	TSecManAuthenticationInd    secManAuthenticationInd;
#endif /**< __BT_MSG_H */
