/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       hci.h
* @brief     HCI Protocol Layer
* @details   
*
* @author  	gordon
* @date      	2015-06-26
* @version	v0.1
*/

#ifndef __HCI_H
#define __HCI_H

#include <os_message.h>
#include <os_queue.h>
#include <trace_binary.h>
#include <btcommon.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HCI_INIT_STATE_REREAD_LOCAL_VERSION   10
#define HCI_OFFSET                      4 /**< Nbr of free Bytes for HCI Packet Type Header */

#define HCI_ROLE_MASTER                       0
#define HCI_ROLE_SLAVE                        1
#define HCI_ROLE_UNKNOWN                      0xFF



/**authentication-encryption management */
typedef enum 
{
    HCI_ENCRYPTION_STATE_OFF = 0,
    HCI_ENCRYPTION_STATE_ON,
    HCI_ENCRYPTION_STATE_OFF_WHILE_AUTHENTICATION_PENDING,
    HCI_ENCRYPTION_STATE_ON_AFTER_AUTHENTICATION
}THciEncryptionStatus;

typedef enum 
{
    HCI_EA_ENCRYPT_ON_IND = 0,
    HCI_EA_ENCRYPT_OFF_IND,
    HCI_EA_ENCRYPT_REQ,
    HCI_EA_AUTH_REQ,
    HCI_EA_AUTH_CONF_IND
}THciEncryptionAuthenticationEvent;

#define HCI_STATUS_IDLE                  0
#define HCI_STATUS_RESETTING             1
#define HCI_STATUS_INITIALIZING          2
#define HCI_STATUS_READY                 3


#define HCI_LINK_LEVEL_ENCRYPTION_OFF    0x00    /**< enable encryption     */
#define HCI_LINK_LEVEL_ENCRYPTION_ON     0x01    /**< disable encryption    */

#define HCI_ACL_SIZE                     BT_US_BUFFER_SIZE - BT_US_WRITE_OFFSET_COUNT
#define HCI_SCO_SIZE                     512
#define HCI_ACL_CNT                      7
#if HCI_ACL_CNT >= BT_US_BUFFER_COUNT
#pragma message("HCI_ACL_CNT must be less then BT_US_BUFFER_COUNT")
#endif /**< HCI_ACL_CNT >= BT_US_BUFFER_COUNT */


#define HCI_MAX_LONG_CMD_SIZE            (256+4) /**< Absolute max length of HCI command   */
#define HCI_MAX_SHORT_CMD_SIZE           (32+4) /**< Max length of HCI short command   */


#define HCI_MAX_REMDEV                   8 		/**< Cache size for remote device parameters */

#define PAGE_SCAN_ACTIVITY_INTERVAL             0x0800
#define PAGE_SCAN_ACTIVITY_WINDOW               0x0012
#define BT1_1_INQUIRY_SCAN_ACTIVITY_INTERVAL    0x0800
#define BT1_2_INQUIRY_SCAN_ACTIVITY_INTERVAL    0x1000
#define INQUIRY_SCAN_ACTIVITY_WINDOW            0x0012

enum {
	NO_ACTION_PARAM,
	SAVE_PARAM,
	RESTORE_PARAM
};

#define HCI_SCAN_NON_DISCOVERABLE     0
#define HCI_SCAN_INQUIRY              1
#define HCI_SCAN_PAGE                 2


#define HCI_MODE_STATE_ACTIVE                 0 /**< active mode            */
#define HCI_MODE_STATE_SNIFF_PENDING          1 /**< sniff req sent down    */
#define HCI_MODE_STATE_NEW_SNIFF_PENDING      2 /**< new sniff req received in HCI_MODE_STATE_SNIFF_PENDING */
#define HCI_MODE_STATE_SNIFF                  3 /**< sniff active           */
#define HCI_MODE_STATE_WAKE_UP_PENDING        4 /**< exit sniff mode sent down */
#define HCI_MODE_STATE_WAKE_UP_SNIFF_PENDING  5 /**< exit sniff mode sent down, sniff req is pending */


typedef struct
{
    uint8_t used;              /**< Descriptor in use flag  */
    uint8_t bdType;            /**< BLUEFACE_BDTYPE_*   */
    uint16_t handle;            /**< Connection handle     */
    TBdAddr bd;             /**< Address of connected BD */   
    uint8_t dsAclQueueID;
    uint8_t conType;
    uint8_t i;                 /**< self index */
    uint8_t currentRole;       /**< current role the device is performing 0xFF - unknown, 0-master, 1-slave */
#if (F_BT_LOW_ENERGY)
    uint16_t leDsAclCount;      /**< LE: No of ACL Blocks sent to baseband */
    uint16_t leDsAclTotal;      /**< LE: No of ACL Blocks/one link  */
#endif
//#if F_BT_BREDR_SUPPORT
    uint16_t dsAclCount;        /**< No of ACL Blocks sent to baseband  */
    uint16_t dsAclTotal;        /**< No of ACL Blocks/one link   */
    uint16_t usAclCount;
//#endif
#if F_BT_BREDR_SUPPORT   
    uint8_t switchRoleTries;

    uint8_t piconetType;       /**< link specific HCI Piconet operation mode        */
    uint16_t link_policy;       /**< as define in HCI Link_Policy_Settings           */

    uint8_t nModeState;        /**< see active/sniff mode states                   */
    uint16_t sniffMax; 
    uint16_t sniffMin; 
    uint16_t sniffAttempt; 
    uint16_t sniffTimeout; /**< sniff params */
    uint16_t supervisionTimeout;

    THciEncryptionStatus encryptionStatus;
    uint8_t tid;                /**< transaction identifier */
#endif
} ThciHandleDesc;
typedef ThciHandleDesc * LPThciHandleDesc;

typedef struct
{
    BYTE used;              /* Descriptor in use flag                     */
    BYTE bdType;            /* BLUEFACE_BDTYPE_*                          */
    TBdAddr bd;             /* Address of connected BD                    */
    struct _THCI * pHCI;    /* Back Pointer to Instance Data of HCI       */
    WORD handle;            /* Connection handle                        */
    WORD dsAclQueueID;


    BYTE i;                 /* self index */
    BYTE currentRole;       /* current role the device is performing 0xFF - unknown, 0-master, 1-slave */
} ThciLEHandleDesc;
typedef ThciLEHandleDesc * PThciLEHandleDesc;

typedef struct
{
    TBdAddr bd;
    uint8_t    pageScanRepMode;
    uint8_t    pageScanMode;
    uint16_t    clockOffset;
} ThciRemDevice;

typedef struct
{
    uint16_t interval;
    uint16_t window;
} ThciPageScan, * LPThciPageScan;


#define HCI_LLAPI_QUEUE_SIZE         (BT_DS_BUFFER_COUNT * 2)

typedef struct _tHciLLQqueueData
{
	struct _tHciLLQqueueData *Next;
	DATA_CB_T                DataCB;
} THciLLQueueData;
typedef THciLLQueueData *PHciLLQueueData;


typedef struct _THCI 
{
    MESSAGE_T   Message;

    uint16_t    ReadOffset;
    uint16_t    WriteOffset;      /**< Offset used for local packets in DS dir*/
    uint8_t     dsQueueID;        /**< Downstream Queue ID: to physical line  */
    uint8_t     hciQueueID;       /**< Queue for open HCI commands            */
    uint8_t    hciTransQueueID;  /**< Queue for open HCI commands (transactions) */
    uint8_t    hciTransQueueID2; /**< Queue for open disc HCI commands (transactions) */
    
    TBdAddr localBdAddr;
    uint16_t    numHCI;           /**< No of free Command Slots in Controller */
    uint8_t    status;
    uint8_t    initPhase;
    uint16_t    transaction;      /**< a transaction is in progress (connect, disconnect, etc.) */
    uint16_t    transactionHandle;
    uint16_t    startOffset;      /**< start offset for downstream queueing   */
    uint16_t    startOffset_off; 
    uint16_t    dsGlobalFreeBufferCount; /* ds count without handle      */
    
#if (F_BT_LOW_ENERGY)
    uint16_t    leAclHandleCnt;   /**< No of allocated LE ACL handles         */
    uint16_t    leDsAclSize;      /**< LE: Max size of ACL Packet downstream      */
 
    uint8_t    leLocalFeatures[LE_FEATURES_SIZE]; /**< set of local supported feature */
 
#endif
    uint16_t    dsAclSize;        /**< Max size of ACL Packet downstream      */
#if F_BT_BREDR_SUPPORT
    uint16_t    usAclHandshake;   /**< no of unacked upstream acl buffers     */
    uint16_t    aclHandleCnt;     /**< No of allocated ACL handles            */
    
    uint8_t    tid;              /**< Temp. storage for requests             */
    uint8_t    linkType;         /**< Last type of link requested (ACL/SCO)  */
    uint8_t    piconetType;      /**< HCI Piconet operation mode             */
    BOOL    internScanEnable;
    uint8_t    saveScanEnable;
    uint8_t    crrScanEnable;
    uint16_t    link_policy;              /**< as define in HCI Link_Policy_Settings  */
    uint16_t    link_supervision_timeout; /**< default 0x7D00 = 20 secs     */
    uint8_t    localFeatures[8]; /* set of local supported feature         */
    TBdAddr remoteBdAddr;    
    TBdAddr transactionBdAddr;   
    TBdAddr keyBdAddr;        /**< BD addr of linkkey commands (in case of errors) */
    uint32_t   capabilities;     /**< 32 bit capability vector */
	uint8_t *  wEIRp;
    LPinqBuf inqBuf;          /**< temp buffer for inquiry operations     */
    ThciPageScan pageScanActivity;
    uint8_t    scanRepetitionModeReq;     /**< received from app(HCI_CONFIGURATION_REQ.pageCharacteristics), 0 - 2: Pageing: Scan repetition mode as defined in Baseband Spec      */
    uint16_t    pageTimeout;               /**< timeout used for paging, in 625us range 1 - 0xFFFF, 0 use stack default */
    	/**
	*this is a local cache for remote device parameters (e.g. clockoffset). cache replacement
	*   strategy is to replace at steadily increasing position (random replace)
	*/
    uint16_t           remPos;
    ThciRemDevice  remDev[HCI_MAX_REMDEV];
#endif
    
    
    uint8_t     hciVersion;
    uint16_t    hciRevision;
    uint16_t    manuName;
    uint8_t     lmpVersion;
    uint16_t    lmpSubVersion;

    void             *llHandle;   /**< lower layer handle */
    QUEUE_T          llDeferredQueue;
    QUEUE_T          llDeferredQueueFree;
    THciLLQueueData  llDeferredQueueData[HCI_LLAPI_QUEUE_SIZE];
    
    ThciHandleDesc* phciDescDon;
    ThciHandleDesc* phciDescDoff;

#if UPPER_STACK_USE_VIRTUAL_HCI
    uint8_t bWritePending;
#endif

} THCI;
typedef THCI * PHCI;



/** capability bit definitions */
#define HCI_CAP_HOST_FLOW                                   (1<<0)   /**< Controller does support upstream flow control */
#define HCI_CAP_NO_CONCURRENT_SCAN_ENABLE                   (1<<1)   /**< Controller supports concurrent connection and scan enable */
#define HCI_SET_EVENT_FILTER_NOT_SUPPORTED                  (1<<4)   /**< List of not supported commands */
#define HCI_SHORT_SET_LOCAL_NAME_SUPPORTED                  (1<<5)   /**< Short form of set name supported */
#define HCI_CHANGE_CON_PACKET_PROBLEM                       (1<<6)   /**< Change connection packet type problem (no answer) */
#define HCI_ENABLE_ROLE_SWITCH_NOT_SUPPORTED                (1<<7)   /**< role switch allowable not supported */
#define HCI_DISCONNECTION_COMPLETE_PATCH                    (1<<8)   /**< create con sometimes replied with disc complete */
#define HCI_WRITE_VOICE_SETTING_NOT_SUPPORTED               (1<<10) /* do not send WRITE_VOICE_SETTING command */
#define HCI_CAP_FAST_TRANSACTION                            (1<<11) /* fast transaction are terminated after the STATUS phase */
#define HCI_WRITE_PAGE_SCAN_ACTIVITY_NOT_SUPPORTED          (1<<12) /* Controller does not support WRITE_PAGE_SCAN_ACTIVITY */
#define HCI_WRITE_INQUIRY_SCAN_ACTIVITY_NOT_SUPPORTED       (1<<13) /* Controller does not support WRITE_INQUIRY_SCAN_ACTIVITY */
#define HCI_SYNCHRONOUS_CONNECTION_COMMAND_NOT_SUPPORTED    (1<<14) /* BT1.2 Controller does not support setup/accept/reject-synchronous-connection */


/** Function declarations - functions used by vendor specific hci parts */
BOOL  hciInit(void);
void  hciEntry(THCI  *lpHCI);
void hciHandleMessage(void);
void hciHandleUpDataReq(void);
void hciProcessEventPacket(uint8_t * fp);
void hciProcessAclPacket(PHCI pHCI, uint8_t * fp, int foffset, int flength);

void hciSendDsCommand(uint16_t queueID, uint16_t command, uint8_t * buf, uint16_t length);
void hciSendDsMessageFromBuffer(uint16_t queueID, uint16_t offset, uint8_t * buf, uint16_t length);
uint16_t hciStatus(uint8_t code);
ThciHandleDesc * hciFindHandle(PHCI pHCI, uint16_t handle);
ThciHandleDesc * hciNewHandle(uint8_t * bd, uint16_t handle, uint8_t conType, uint8_t bdType);
void hciRemoveHandle(uint16_t handle);
ThciHandleDesc * hciFindBd(uint8_t * bd, uint8_t conType);
void hciConfigureNoConnectionsInd(uint8_t indId, uint8_t * bd, uint8_t bdType);
void hciCommandSetEventMask(uint8_t * mask, BOOL le);

void hciNextInitCommand(void);
void hciInitCompleted(void);
void hciShutDown(void);
void hciNotInSync(void);
void hciHandlePhDataIndExt(PHCI pHCI);

void hciDiscAllScoByAcl(ThciHandleDesc *hd, uint16_t handle, uint8_t status);





#ifdef __cplusplus
}
#endif

#endif
