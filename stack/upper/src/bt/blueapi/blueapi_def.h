/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file        blueapi_def.h
* @brief      
* @details   
*
* @author   gordon
* @date      2015-07-10
* @version  v0.1
*/

#ifndef __BLUEAPI_DEF_H
#define __BLUEAPI_DEF_H

#include <trace_binary.h>
#include <blueapi_types.h>

#if defined (__cplusplus)
extern "C" {
#endif

#define BLUE_API_MCL_ELEMENT_COUNT  2

/** generate info-uint32_t for COM_InternalEventInfo messages */
#define BLUE_API_GENERATE_EVENT_ID  ((0x00)<<24 | (((BLUE_API_SOURCE_FILE_ID) & 0xFF)<<16) | ((__LINE__) & 0xFFFF))

/** default settings definitions */
#define BLUE_API_INVALID_MDL_ID     0x0000
#define BLUE_API_FIRST_MDL_ID       0x0001
#define BLUE_API_LAST_LOCAL_MDL_ID  0x00FE

#define BLUE_API_MAGIG              0x6789

#define BLUE_API_STATE_BF_ACT_IND   0x0001
#define BLUE_API_STATE_READY        BLUE_API_STATE_BF_ACT_IND

#define BLUE_API_SDP_SERVICE_HANDLE_COUNT  8

/** event definitions */
typedef enum
{  
    blueAPI_Event_BT__LinkCloseInd  = 0x81,
    blueAPI_Event_BT__LinkOpenInd
}TBlueAPI__BT_Event;
typedef TBlueAPI__BT_Event *PBlueAPI__BT_Event;

/** message definitions */
typedef struct
{ 
    struct _TBlueAPI_LinkDescriptor *pLinkContext;
    TBlueAPI_Cause                   cause;
}TBlueAPI__BT__LinkCloseInd;
typedef TBlueAPI__BT__LinkCloseInd *PBlueAPI__BT__LinkCloseInd;

typedef struct
{  
    struct _TBlueAPI_LinkDescriptor *pLinkContext;
    TBlueAPI_Cause                   cause;
}TBlueAPI__BT__LinkOpenInd;
typedef TBlueAPI__BT__LinkOpenInd *PBlueAPI__BT__LinkOpenInd;

typedef union
{
    TBlueAPI__BT__LinkOpenInd        BT__LinkOpenInd;
    TBlueAPI__BT__LinkCloseInd       BT__LinkCloseInd;
}TBlueAPI__BT_EventData;
typedef TBlueAPI__BT_EventData *PBlueAPI__BT_EventData;

typedef uint8_t TBlueAPI_Event;
typedef TBlueAPI_Event *PBlueAPI_Event;

/** link definitions */
typedef enum
{
    blueAPI_LinkStateReserved,
    blueAPI_Idle,
    blueAPI_Listening,
    blueAPI_Connecting,
    blueAPI_Connected,
    blueAPI_Disconnecting,
    blueAPI_Disconnected
}TBlueAPI_LinkState;
typedef TBlueAPI_LinkState *PBTBedLinkState;

typedef enum
{
    blueAPI_SubRoleUndefined,
    blueAPI_SubRoleInitiator,
    blueAPI_SubRoleAcceptor
}TBlueAPI_LinkSubRole;
typedef TBlueAPI_LinkSubRole *PBlueAPI_LinkSubRole;

/*GATT server state definitions ******************************************************/
#define BLUE_API_GATT_SRV_NOT_USED              0x0000
#define BLUE_API_GATT_SRV_REGISTERING           0x0001   /* ReqMDEPReq received */
#define BLUE_API_GATT_SRV_RELEASING             0x0002   /* RelMDEPReq received */
#define BLUE_API_GATT_SRV_REGISTERED            0x0008   /* MDEP in use */

/** MDL definitions */
#define BLUE_API_MDL_FLAG_ANNY_FLAG             0x0000
#define BLUE_API_MDL_FLAG_DISCONNECT_IND        0x0001
#define BLUE_API_MDL_FLAG_DELETE_INFO           0x0002
#define BLUE_API_MDL_FLAG_DELETE                0x0004
#define BLUE_API_MDL_FLAG_NOT_ACTIVATED         0x0008

typedef struct _TBlueAPI_MDL
{
    /** MCAP configuration */
    uint16_t                pendingFlags;
    TBlueAPI_Cause          pendingCause;

    /** MCAP configuration */
    uint16_t                local_MDL_ID;
    uint8_t                 remoteBd[6];
    TBlueAPI_RemoteBDType   remoteBdType;
    TBlueAPI_LinkConfigType linkConfigType;

    /** L2CAP/RFCOMM channel configuration */
    uint16_t                maxTPDUSize;
    uint8_t                 maxUsCredits;   /**< from CreateMDLConf */
    uint8_t                 dsCredits;
}TBlueAPI_MDL;
typedef TBlueAPI_MDL *PBlueAPI_MDL;

typedef enum
{
    blueAPI_DS_Reserved,
    blueAPI_DS_Idle,
    blueAPI_DS_ControlConnected,
    blueAPI_DS_DataConnecting,
    blueAPI_DS_DataConnected,
    blueAPI_DS_DataDisconnecting,
    blueAPI_DS_DataListening,
}TBlueAPI_MCLState;

typedef enum
{
    blueAPI_MCLType_Invalid,
    blueAPI_MCLType_GATT,
    blueAPI_MCLType_SDP,
}TBlueAPI_MCLType;

typedef enum _TBlueAPI_MCLTimerID
{
    blueAPI_TimerID_LEScanTO = 0x01,
    blueAPI_TimerID_Illegal
}TBlueAPI_MCLTimerID;

typedef struct
{
    /** This MUST be the FIRST structure element ! */
    ELEMENT_T           QueueElement;
    /** the 'real' data... */
    BOOL                used;
    PBlueAPI_DsMessage  lpCOMMsg;
}TBlueAPI_COMCommandElement;
typedef TBlueAPI_COMCommandElement *PBlueAPI_COMCommandElement;

typedef struct _TBlueAPI_MCL
{
    TBlueAPI_MCLState               state;
    uint8_t                         number;
    TBlueAPI_MCLType                mclType;
    TBlueAPI_RemoteBDType           bdType;
    uint8_t                         bd[6];

    /** deferred COM Command handling */
#if BLUE_API_MCL_ELEMENT_COUNT
    QUEUE_T                         COMCommandQueue;
    TBlueAPI_COMCommandElement      COMCommandElement[BLUE_API_MCL_ELEMENT_COUNT];
#endif

    /** state machine relevant parameter  */
    TBlueAPI_Command                DS_CommandInProgress;
    PBlueAPI_DsMessage              DS_CommandMsg;
    TBlueAPI_Command                US_CommandInProgress;
    TBlueAPI_UsCommandData          US_CommandData;

    void                            *TimeHandleLEScanTO;
    struct _TBlueAPI_LinkDescriptor *pDataChannel;
}TBlueAPI_MCL;
typedef TBlueAPI_MCL *PBlueAPI_MCL;

/** link descriptor definitions */
typedef struct
{
    /** This MUST be the FIRST structure element ! */
    ELEMENT_T       QueueElement;
    /** the 'real' data... */
    BOOL            used;
    PVOID           lpMsg;
}TBlueAPI_BufferElement;
typedef TBlueAPI_BufferElement *PBlueAPI_BufferElement;

typedef struct _TBlueAPI_LinkDescriptor
{
    /** link relevant parameter */
    BLINKHANDLE             handle;
    TBlueAPI_LinkState      linkState;
    uint8_t                 MDLConnected;

    /** L2CAP */
    uint16_t                dsPoolID;

#if (F_BT_LOW_ENERGY)
    uint16_t                LEConnectionInterval;
    uint16_t                LEConnectionLatency;
    uint16_t                LESupervisionTimeout;
#endif

    /** profile relevant parameter */
    PBlueAPI_MDL            pMDL;
    PBlueAPI_MCL            pMCL;
    TBlueAPI_LinkSubRole    linkSubRole;
    TBlueAPI_LinkConfigType linkConfigType;
}TBlueAPI_LinkDescriptor;
typedef TBlueAPI_LinkDescriptor *PBlueAPI_LinkDescriptor;

typedef struct
{
    uint16_t                used;
    uint16_t                actInfoSent;
    TBlueAPIAppHandle       appHandle; /*SPPInstance*/
    PVOID                   MDHmsgHandlerCallback; /*sppdBlueAPICallback*/
    struct _TBlueAPI_Data   *pBlueAPIdata;
}TBlueAPI_App;
typedef TBlueAPI_App *PBlueAPI_App;

#define BLUE_API_DATASTORE_SMALL_ENTRY_SIZE    4
#define BLUE_API_DATASTORE_LARGE_ENTRY_SIZE    28

typedef struct _TBlueAPI_DSPeer
{
    uint8_t        used;
    uint8_t        nextPeerIdx;
    uint8_t        bdType;
    TBdAddr        bd;
    uint32_t       appData;
    uint8_t        deviceName[BLUE_API_DEVICE_NAME_LENGTH];
}TBlueAPI_DSPeer, *PBlueAPI_DSPeer;

typedef struct _TDataStoreEntry
{
    uint8_t        peerIdx;
    uint8_t        dataType;
    uint8_t        dataLen;
    uint8_t        _dummy;
    uint8_t        data[1];
}TBlueAPI_DSEntry, *PBlueAPI_DSEntry;

typedef struct _TBlueAPI_DSEntryS
{
    uint8_t        peerIdx;
    uint8_t        dataType;
    uint8_t        dataLen;
    uint8_t        _dummy;
    uint8_t        data[BLUE_API_DATASTORE_SMALL_ENTRY_SIZE];
}TBlueAPI_DSEntryS, *PBlueAPI_DSEntryS;

typedef struct _TBlueAPI_DSEntryL
{
    uint8_t        peerIdx;
    uint8_t        dataType;
    uint8_t        dataLen;
    uint8_t        _dummy;
    uint8_t        data[BLUE_API_DATASTORE_LARGE_ENTRY_SIZE];
}TBlueAPI_DSEntryL, *PBlueAPI_DSEntryL;

typedef struct _TBlueAPI_DSNVData
{
    uint16_t        checksum;
    uint16_t        size;
    uint8_t         maxPeerCount;
    uint8_t         firstPeerIdx;
    uint8_t         lastPeerIdx;
    PVOID           dataDyn;        /**< PVOID to get correct alignment */
}TBlueAPI_DSNVData, *PBlueAPI_DSNVData;

typedef struct _TBlueAPI_DS
{
    PBlueAPI_DSNVData   pNVData;
    PBlueAPI_DSPeer     pPeers;
    PBlueAPI_DSEntryS   pEntriesS;
    PBlueAPI_DSEntryL   pEntriesL;
    uint8_t             peerCount;      /**< BLUE_API_DATASTORE_PEER_COUNT */
    uint8_t             entriesSCount;  /**< BLUE_API_DATASTORE_SMALL_ENTRY_COUNT */
    uint8_t             entriesLCount;  /**< BLUE_API_DATASTORE_LARGE_ENTRY_COUNT */

    uint8_t             changed;
    uint8_t             maxPeerCount;   /**< copy of pNVData->maxPeerCount */
    uint8_t             firstPeerIdx;   /**< copy of pNVData->firstPeerIdx */
    uint8_t             lastPeerIdx;    /**< copy of pNVData->lastPeerIdx */
}TBlueAPI_DS, *PBlueAPI_DS;

typedef struct
{
    uint16_t            StoreBondMode;      /**< not stored in DB */
    uint16_t            StoreBondSize;      /**< not stored in DB, only initial parameter */
    uint32_t            ClassOfDevice;      /**< not stored in DB */
    uint16_t            limitedDiscovery;   /**< not stored in DB */
}TBlueAPI_Parameter;
typedef TBlueAPI_Parameter *PBlueAPI_Parameter;

typedef struct 
{
    uint8_t         DID_PrimaryFound;
    uint32_t        DID_ServiceHandle;    /**< DID has different service handle! */
    uint16_t        DID_VendorID;
    uint16_t        DID_VendorID_Source;
    uint16_t        DID_ProductID;
    uint16_t        DID_Version;
    uint8_t         DID_Device_Name[BLUE_API_DEVICE_NAME_LENGTH];
}TBlueAPI_SDPParserData;
typedef TBlueAPI_SDPParserData *PBlueAPI_SDPParserData;

/** instance definitions*/
typedef struct _TBlueAPI_Data
{
    uint8_t                 usBlueAPIPoolID;
    uint8_t                 dsBlueAPIPoolID;

    uint16_t                state;      /**< BLUE_API_STATE_ bitmask */
    TBlueAPI_Cause          stateCause;
    BAPPHANDLE              AppHandle;  /**< for blueface handle */

    uint8_t                 systemPoolID;
    uint16_t                SM_Active;
    TBlueAPI_App            appDescriptorTable;

    TBlueAPI_MCL            *pMCLDescriptorTableDon;
    TBlueAPI_MCL            *pMCLDescriptorTableDoff;

    TBlueAPI_MDL            *pMDLDescriptorTableDon;
    TBlueAPI_MDL            *pMDLDescriptorTableDoff;

    TBlueAPI_LinkDescriptor *plinkDescriptorTableDon;
    TBlueAPI_LinkDescriptor *plinkDescriptorTableDoff;

    uint16_t                nextLocal_MDL_ID;

    PBlueAPI_App            pActiveApp;
    PBlueAPI_MCL            pActiveMCL;
    TBlueAPI_Parameter      ConfigParameter;
    TBlueAPI_DS             DataStore;

    QUEUE_T                 DataStoreQueue;
    TBlueAPI_BufferElement  DSBufferElement[4];
    LPblueFaceMsg           pDSCurrentMsg;
    uint16_t                dsCCCSearchHandle;

    /** security State Machine (RadioMode, PairableMode, AuthReq, AuthDelete) */
    PBlueAPI_DsMessage      SM_SEC_DS_CommandMsg;

    /** Inquiry/ServiceDiscovery State Machine */
    TBlueAPI_Command        SM_SDP_DS_Command;         /**< current cmd */
    PBlueAPI_DsMessage      SM_SDP_DS_CommandMsg;      /**< request msg of current cmd */
    TBlueAPI_Cause          SM_SDP_DS_Cause;
    TBlueAPI_Command        SM_SDP_US_Command;
    uint32_t                SM_SDP_US_Handle;
    uint16_t                SM_SDP_ServiceUUID;        /**< searched UUID */
    TBlueAPI_SDPParserData  SM_SDP_Data;
    PBlueAPI_LinkDescriptor pSDPLinkContext;
    TSDP_SEARCH_CONF        *pSearchConf;
    uint16_t                serviceHandleIndex;
    uint32_t                attributeRange;

    uint8_t                 local_BD[6];                /**< blueface adapter BD */
}TBlueAPI_Data;
typedef TBlueAPI_Data * PBlueAPI_Data;


void blueAPI_SetDeviceName(PBlueAPI_Data pBlueAPIdata, uint8_t * pName);
void blueAPI_GetEIRDeviceName(uint8_t * pEIR, uint8_t * pName, uint16_t nameSize);
TBlueAPI_Cause blueAPI_SDP_Connect(PBlueAPI_Data pBlueAPIdata, PBlueAPI_DsMessage pReqMsg, uint8_t * remote_BD);
void blueAPI_SDP_Disconnect(PBlueAPI_Data pBlueAPIdata, PBlueAPI_LinkDescriptor pLinkContext, TBlueAPI_Cause cause);

void blueAPI_DumpLink (PBlueAPI_LinkDescriptor pLinkContext,const char* pPrefix);
void blueAPI_DumpMCL(PBlueAPI_MCL pMCL, const char* pPrefix);
void blueAPI_DumpMDL(PBlueAPI_MDL pMDL, const char* pPrefix);
void blueAPI_DumpSystem(PBlueAPI_Data pBlueAPIdata);

void blueAPI_Send_BLUEFACE_CONF_INQUIRY_SCAN_IACS(PBlueAPI_Data pBlueAPIdata, BOOL limitedDiscoverable);

#if BLUEAPI_TRACE_VERBOSITY_COUNT
#define TRACE_DUMP_SYSTEM(p1)    blueAPI_DumpSystem(p1)
#define TRACE_DUMP_MCL(p1,p2)    blueAPI_DumpMCL(p1,p2)
#define TRACE_DUMP_LINK(p1,p2)   blueAPI_DumpLink(p1,p2)
#define TRACE_DUMP_MDL(p1,p2)    blueAPI_DumpMDL(p1,p2)
#define TRACE_DUMP_MDL_FLAGS(p1) blueAPI_MDLFlagsString(p1)
#else  
#define TRACE_DUMP_SYSTEM(p1)
#define TRACE_DUMP_MCL(p1,p2)
#define TRACE_DUMP_LINK(p1,p2)
#define TRACE_DUMP_MDL(p1,p2)
#define TRACE_DUMP_MDL_FLAGS(p1)
#endif 
PBlueAPI_COMCommandElement blueAPI_GetNextCOMCommand (PBlueAPI_MCL pMCL);
BOOL                    blueAPI_QueueCOMCommand(PBlueAPI_MCL pMCL, PBlueAPI_DsMessage pCOMCommand);
BOOL                    blueAPI_CheckLinkConfigType(uint16_t psm, TBlueAPI_LinkConfigType linkConfigType);

void                    blueAPI_CheckMCLDisconnect(PBlueAPI_Data pBlueAPIdata, PBlueAPI_MCL pMCL);
PBlueAPI_App            blueAPI_AllocCOMApp(PBlueAPI_Data pBlueAPIdata);
PBlueAPI_MCL blueAPI_MCLAllocate(uint8_t * pBD, TBlueAPI_RemoteBDType bdType, TBlueAPI_MCLType mclType);
void                    blueAPI_MCLRelease(PBlueAPI_Data pBlueAPIdata, PBlueAPI_MCL pMCL);
TBlueAPI_MCLType        blueAPI_LinkConfigType2MCLType(TBlueAPI_LinkConfigType linkConfigType);
PBlueAPI_MCL blueAPI_MCLFindByBD(uint8_t *pBD, TBlueAPI_MCLType mclType);
PBlueAPI_MCL            blueAPI_MCLFindByTimerChannel(PBlueAPI_Data pBlueAPIdata, uint16_t timerChannel);
PBlueAPI_MCL  blueAPI_MCLFindByLocal_MDL_ID(uint16_t local_MDL_ID);
PBlueAPI_LinkDescriptor blueAPI_MCLFindLinkByState(PBlueAPI_MCL pMCL,TBlueAPI_LinkState linkState);
TBlueAPI_MCLStatus      blueAPI_MCLState2Status(TBlueAPI_MCLState state);
void                    blueAPI_MCLSetState(PBlueAPI_MCL pMCL,TBlueAPI_MCLState newState);
BOOL                    blueAPI_MCLInTransition(PBlueAPI_MCL pMCL);
void                    blueAPI_MCL_US_ContextClear(PBlueAPI_MCL pMCL);
void                    blueAPI_ChannelConnect(PBlueAPI_Data pBlueAPIdata, PBlueAPI_MCL pMCL, PBlueAPI_LinkDescriptor pLinkContext);
void                    blueAPI_ChannelAccept(PBlueAPI_Data pBlueAPIdata, PBlueAPI_LinkDescriptor pLinkContext, BOOL accept);
BOOL                    blueAPI_MCLAddDataChannel(PBlueAPI_MCL pMCL, PBlueAPI_LinkDescriptor pLinkContext);
void                    blueAPI_ChannelRelease(PBlueAPI_Data pBlueAPIdata, PBlueAPI_LinkDescriptor pLinkContext);
void                    blueAPI_ChannelDisconnect(PBlueAPI_Data pBlueAPIdata, PBlueAPI_LinkDescriptor pLinkContext, BOOL ignoreMCLState);
PBlueAPI_LinkDescriptor blueAPI_ChannelAllocate(PBlueAPI_MCL pMCL, TBlueAPI_LinkSubRole linkSubRole, TBlueAPI_LinkConfigType linkConfigType);
void                    blueAPI_ChannelSetConfiguration(PBlueAPI_Data pBlueAPIdata, PBlueAPI_LinkDescriptor pLinkContext,TBlueAPI_LinkConfigType linkConfigType);
PBlueAPI_LinkDescriptor blueAPI_ChannelFindByLocal_MDL_ID(uint16_t local_MDL_ID);

PBlueAPI_MDL blueAPI_MDLAllocate(PBlueAPI_MCL pMCL, TBlueAPI_LinkConfigType linkConfigType);
void         blueAPI_MDLRelease(PBlueAPI_MDL pMDL);
PBlueAPI_MDL blueAPI_MDLFindByLocal_MDL_ID (PBlueAPI_Data pBlueAPIdata,uint16_t local_MDL_ID);
void         blueAPI_MDLAddFlags(PBlueAPI_MDL pMDL, uint16_t addFlags);
void         blueAPI_MDLSubFlags(PBlueAPI_MDL pMDL, uint16_t subFlags);
PBlueAPI_MDL blueAPI_MDLFindByFlags(PBlueAPI_Data pBlueAPIdata, PBlueAPI_MCL pMCL, uint16_t flags);

void  blueAPI_TimerTimeout(PBlueAPI_Data pBlueAPIdata, TBlueAPI_MCLTimerID timerID, uint16_t timerChannel);
void  blueAPI_TimerStart(PBlueAPI_MCL pMCL, TBlueAPI_MCLTimerID timerID, uint32_t timeout_ms);
void  blueAPI_TimerRetrigger(PBlueAPI_MCL pMCL, TBlueAPI_MCLTimerID timerID, uint32_t timeout_ms);
void  blueAPI_TimerStop(PBlueAPI_MCL pMCL, TBlueAPI_MCLTimerID timerID);

void blueAPI_SDP_DS_ContextSet(PBlueAPI_Data pBlueAPIdata, PBlueAPI_DsMessage pMsg);
void blueAPI_SDP_DS_ContextClear(PBlueAPI_Data pBlueAPIdata, PBlueAPI_DsMessage pMsg);

void blueAPI_Send_BLUEFACE_CONF_INQUIRY_SCAN_ACTIVITY_EX(PBlueAPI_Data pBlueAPIdata, TBlueAPI_BRInquiryScanType scanType, uint16_t scanInterval, uint16_t scanWindow);
void blueAPI_Send_BLUEFACE_CONF_READ_LOCAL_OOB_DATA(void);
void blueAPI_Send_BLUEFACE_CONF_LE_SSP_PARAMETER(PBlueAPI_Data pBlueAPIdata, uint32_t fixedDisplayValue);
void blueAPI_Send_BLUEFACE_CONF_SNIFF_SUBRATING(PBlueAPI_Data pBlueAPIdata, uint8_t * remote_BD, uint16_t maxLatency, uint16_t minRemoteTimeout, uint16_t minLocalTimeout);

void blueAPI_Send_BT_SDP_SEARCH_REQ(PBlueAPI_Data pBlueAPIdata, uint16_t uuid1, uint16_t uuid2);
void blueAPI_Send_BT_SDP_ATTRIBUTE_REQ(PBlueAPI_Data pBlueAPIdata, uint32_t serviceHandle, uint32_t attributeRange, uint16_t maxLen);

void blueAPI_Send_ConnectMDLInfo(PBlueAPI_LinkDescriptor pLinkContext);
void blueAPI_Send_DeleteMDLInfo(PBlueAPI_MDL pMDL);
void blueAPI_Send_CreateMDLInd(PBlueAPI_MCL pMCL, PBlueAPI_MDL pMDL);
void blueAPI_Send_ConnectMDLRsp(PBlueAPI_DsMessage pReqMsg, PBlueAPI_MCL pMCL, uint8_t * pBD, TBlueAPI_RemoteBDType bdType, uint16_t local_MDL_ID, TBlueAPI_Cause cause);
void blueAPI_Send_DisconnectMDLInd(PBlueAPI_MCL pMCL, PBlueAPI_MDL pMDL, TBlueAPI_Cause cause);
void blueAPI_Send_DisconnectMDLRsp(PBlueAPI_DsMessage pReqMsg, PBlueAPI_MCL pMCL, uint16_t local_MDL_ID, TBlueAPI_Cause cause);
void blueAPI_Send_InquiryRsp(PBlueAPI_DsMessage pReqMsg, BOOL cancelInquiry, TBlueAPI_Cause cause);

void blueAPI_Send_GATTServiceRegisterRsp( PVOID serviceHandle, uint16_t subCause);
void blueAPI_Send_GATTSDPDiscoveryRsp(PBlueAPI_DsMessage pReqMsg, TBlueAPI_Cause cause);
void blueAPI_Send_GATTSDPDiscoveryInd(PBlueAPI_Data pBlueAPIdata, uint32_t serviceHandle, uint8_t * remote_BD, uint16_t remote_GATT_UUID, uint16_t remote_GATT_StartHandle, uint16_t remote_GATT_EndHandle);
void blueAPI_Send_GATTSecurityRsp(uint16_t local_MDL_ID, TBlueAPI_KeyType keyType, uint8_t keySize, TBlueAPI_Cause cause);
void blueAPI_Send_GATTServerStoreInd(TBlueAPI_GATTStoreOpCode opCode, uint8_t * remote_BD, TBlueAPI_RemoteBDType remote_BD_Type, uint16_t restartHandle, uint8_t dataLength, uint8_t * data);

void blueAPI_Send_RadioModeSetRsp(TBlueAPI_Cause cause);
void blueAPI_Send_DeviceConfigSetRsp(PBlueAPI_DsMessage pReqMsg, TBlueAPI_DeviceConfigOpcode opCode, TBlueAPI_Cause cause);
void blueAPI_Send_InquiryDeviceInfo(uint8_t * remote_BD, uint32_t remote_Device_Class, uint8_t remote_RSSI, uint8_t * remote_Device_Name);
void blueAPI_Send_DeviceNameRsp(PBlueAPI_DsMessage pReqMsg, uint8_t * remote_BD, uint8_t * remote_Device_Name, TBlueAPI_Cause cause);
void blueAPI_Send_DIDDeviceInd(PBlueAPI_Data pBlueAPIdata, uint32_t serviceHandle, uint8_t * remote_BD, uint16_t remote_VendorID, uint16_t remote_VendorID_Source, uint16_t remote_ProductID, uint16_t remote_Version, uint8_t * remote_Device_Name);
void blueAPI_Send_SDPDiscoveryRsp(PBlueAPI_Data pBlueAPIdata, PBlueAPI_DsMessage pReqMsg, TBlueAPI_Cause cause);
void blueAPI_Send_SDPEndpointInd(PBlueAPI_Data pBlueAPIdata, uint32_t serviceHandle, uint8_t * remote_BD, uint8_t remote_MDEP_ID, TBlueAPI_MDEPDataType remote_MDEP_DataType, uint8_t * remote_MDEP_Description, uint16_t remoteversion, uint16_t supportedfeatures);
void blueAPI_Send_MCLStatusInfo(uint8_t * pBD, uint16_t localMCL_ID, TBlueAPI_MCLStatus status);
void blueAPI_SendToAll_ActInfo(PBlueAPI_Data pBlueAPIdata, TBlueAPI_Cause cause, BOOL force);
void blueAPI_Send_InternalEventInfo(TBlueAPI_InternalEventType eventType, uint32_t eventInfo, TBlueAPI_Cause cause);
void blueAPI_Send_PairableModeSetRsp( PBlueAPI_DsMessage pReqMsg, TBlueAPI_Cause cause);
void blueAPI_Send_AuthRsp(PBlueAPI_DsMessage pReqMsg, uint8_t * remote_BD, TBlueAPI_Cause cause);
void blueAPI_Send_AuthResultRequestInd(uint8_t * remote_BD, TBlueAPI_RemoteBDType remote_BD_Type, TBlueAPI_LinkKeyType keyType, uint16_t restartHandle);
void blueAPI_Send_AuthResultInd(uint8_t * remote_BD, TBlueAPI_RemoteBDType remote_BD_Type, uint8_t linkKeyLength, uint8_t * linkKey, TBlueAPI_LinkKeyType keyType, TBlueAPI_Cause cause);
void blueAPI_Send_AuthListInfo(uint8_t * remote_BD, TBlueAPI_RemoteBDType remote_BD_Type, TBlueAPI_LinkKeyType keyType, uint32_t AppData, uint8_t * Remote_DeviceName);

#if (F_BT_LOW_ENERGY)
//void blueAPI_Send_LEConnectionUpdateRsp     (PBlueAPI_Data pBlueAPIdata, uint16_t local_MDL_ID, TBlueAPI_Cause cause);
void blueAPI_Send_LEConnectionUpdateInd(uint16_t local_MDL_ID, TGATT_LE_CONNECTION_UPDATE_PARAM *pParam);
void blueAPI_Send_LEConnectionParameterInfo(PBlueAPI_LinkDescriptor pLinkContext);
#if (F_BT_LE_PRIVACY_MODE)
void blueAPI_Send_LEPrivacyModeRsp(PBlueAPI_Data pBlueAPIdata, uint8_t * local_BD, TBlueAPI_RemoteBDType local_BD_Type, TBlueAPI_Cause cause);
#endif
#endif

void blueAPI_Handle_BT_Event(PBlueAPI_Data pBlueAPIdata, TBlueAPI__BT_Event BT_event, PBlueAPI__BT_Event pBTEvent);
PVOID blueAPI_Main(PBlueAPI_Data pBlueAPIdata);

void   blueAPI_SDPDecodeDIPAttr(PBlueAPI_Data pBlueAPIdata, uint32_t serviceHandle, uint8_t * pAttrBuf, uint16_t attrLen);
void   blueAPI_SDPDecodeRFCOMMAttr(PBlueAPI_Data pBlueAPIdata, uint32_t serviceHandle, uint8_t * pAttrBuf, uint16_t attrLen);
void   blueAPI_SDPDecodeGATTAttr(PBlueAPI_Data pBlueAPIdata, uint32_t serviceHandle, uint8_t * pAttrBuf, uint16_t attrLen);
void   blueAPI_SDPGetNextService(PBlueAPI_Data pBlueAPIdata);
BOOL blueAPI_TgtSendEvent(PBlueAPI_App  pBlueAPIApp, PBlueAPI_DsMessage pOldMsg, PBlueAPI_UsMessage pMsg);

BOOL blueAPIStore_QueueIn(PBlueAPI_Data pBlueAPIdata, LPblueFaceMsg pMsg);
void blueAPIStore_QueueTrigger(PBlueAPI_Data pBlueAPIdata, BOOL releaseMsg);

BOOL blueAPIStore_SetBRLinkKey(PBlueAPI_Data pBlueAPIdata, uint8_t * pBD, PDEVICE_DATA_ELEMENT_BREDR_LINKKEY pElement);
BOOL blueAPIStore_GetBRLinkKey(PBlueAPI_Data pBlueAPIdata, uint8_t * pBD, PDEVICE_DATA_ELEMENT_BREDR_LINKKEY pElement);
BOOL blueAPIStore_SetPeerInfo(PBlueAPI_Data pBlueAPIdata, uint8_t * pBD, uint8_t bdType, uint8_t * pDeviceName, uint16_t nameLength, uint32_t* pAppData);
BOOL blueAPIStore_GetPeerInfo(PBlueAPI_Data pBlueAPIdata, uint8_t * pBD, uint8_t bdType, uint8_t * pDeviceName, uint16_t nameLength, uint32_t* pAppData);
BOOL blueAPIStore_DeletePeer(PBlueAPI_Data pBlueAPIdata, uint8_t * pBD, uint8_t bdType);
BOOL blueAPIStore_SendAuthList(PBlueAPI_Data pBlueAPIdata, uint8_t * pBD, uint8_t bdType);
void blueAPIStore_ChangeMode(PBlueAPI_Data pBlueAPIdata, TBlueAPI_StoreBondModes storeBondMode, uint16_t storeBondSize, BOOL init);
uint16_t blueAPIStore_GetNVDataSize(PBlueAPI_Data pBlueAPIdata);
void blueAPIStore_Init(PBlueAPI_Data pBlueAPIdata);

BOOL blueAPI_Handle_Command(PBlueAPI_DsMessage pMsg);

#if F_BT_LE_BT41_SUPPORT
TBlueAPI_Cause blueAPI_L2CAPConvertCause(uint16_t l2cCause);
#endif

#if defined (__cplusplus)
}
#endif

#endif /**< __BLUEAPI_DEF_H */

