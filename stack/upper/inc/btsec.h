/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       btsec.h
* @brief     
* @details   
*
* @author  	gordon
* @date      	2015-06-29
* @version	v0.1
*/

#if !defined(__BTSEC_H)
#define __BTSEC_H




/** defines and macros */
#define BD_ADDR_SIZE            6
#define C_R_SIZE                16

/** size of header */
#define BtsecMsgHeaderSize (offsetof(TBtsecMsg,P))

/** length of message until start of specified element */
#define BtsecMsgLength(Elem) offsetof(TBtsecMsg, Elem)

#define BTSEC_OWN_DEVICE_NAME_LENGTH  50   /**< Max length of OWN device name */
#define BTSEC_OWN_DEVICE_PIN_LENGTH   16   /**< Max length of OWN device Pin  */
#define BTSEC_OWN_DEVICE_PIN_NUMERIC   0   /**< numeric pin only?             */
#define BTSEC_OWN_DEVICE_MAX_NUM_OF_IAC   2   /**< max number of Inquiry Acccess Code */

#if !defined(BTSEC_DEVICE_NAME_LENGTH)
/** Max length of device name */
#define BTSEC_DEVICE_NAME_LENGTH      32
#endif

/** Max length of service name */
#define BTSEC_SERVICE_NAME_LENGTH     32

/** Flags for RegisterRequest  */
#define BTSEC_FLAG_NO_INDICATION                 ((uint32_t)0x00000000ul)
        /**< => app is not interested on anny system information              */

#define BTSEC_FLAG_OWN_DEVICE_INDICATION         ((uint32_t)0x00000001ul)
        /**< => app is interested in changes of 'own device' settings         */
        /**<    like 'device name'                                            */
        /**<    for more information see 'BtsecCommandOwnDevice' messages     */

#define BTSEC_FLAG_PROGRESS_INDICATION           ((uint32_t)0x00000002ul)
        /**< => app is interested in 'progress' indications                   */
        /**<    (like number of active ACL connections)                       */
        /**<    for more information see 'BtsecCommandProgressInd' message    */

#define BTSEC_FLAG_INQUIRY_INDICATION            ((uint32_t)0x00000004ul)
        /**< => app is interested in indications of active inquiry state      */
        /**<    like 'inquiry started' and 'inquiry stopped'                  */
        /**<    for more information see 'BtsecCommandInquiry' messages       */

#define BTSEC_FLAG_DEVICE_INDICATION             ((uint32_t)0x00000008ul)
        /**< => app is interested in indications of inquired devices and      */
        /**<    discovered services                                           */
        /**<    for more information see:                                     */
        /**<    - 'BtsecCommandNameDiscovery'                                 */
        /**<    - 'BtsecCommandNameDiscovery'                                 */
        /**<    - 'BtsecCommandDevice'                                        */
        /**<    - 'BtsecCommandService'                                       */

#define BTSEC_FLAG_SECURITY_INDICATION           ((uint32_t)0x00000010ul)
        /**< => app is interested in indications regarding security           */
        /**<    like setting changes or bonded devices                        */
        /**<    for more information see 'BtsecCommandSecurity' messages      */

#define BTSEC_FLAG_SECURITY_USER_PIN_INDICATION  ((uint32_t)0x00000020ul)
        /**< => app can handle PIN requsts from security handler              */
        /**<    (app can handle 'BtsecCommandSecurityUserPin' type messages)  */
        /**<    there can only be one app with this flag in the system!!      */

#define BTSEC_FLAG_LINK_MODE_INDICATION          ((uint32_t)0x00000040ul)
        /**< => app is interested in indications regarding ACL link modes     */
        /**<    like link is in 'sniffmode'                                   */
        /**<    for more information see 'BtsecCommandLinkMode' messages      */

#define BTSEC_FLAG_SECURITY_STATUS_INDICATION    ((uint32_t)0x00000080ul)
        /**< => app is interested in indications regarding ACL link security  */
        /**<    status like 'authentication startet' or 'link encrypted'      */
        /**<    for more information see 'BtsecCommandSecurityStatusInd' msg. */

#define BTSEC_FLAG_ALL_EXEPT_USER_PIN_INDICATION ((uint32_t)0xFFFFFFDFul)
#define BTSEC_FLAG_ALL                           ((uint32_t)0xFFFFFFFFul)

/** Causes */
typedef enum _TBtsecCause
{
	BtsecCauseSuccess                =  0,   /**< operation was successfull     */
	BtsecCauseError                  =  1,   /**< error > RawError for more Info*/
    BtsecCauseInquiryAbort           =  2,   /**< inquiry was aborted           */
    BtsecCauseDeviceAdd              =  3,   /**< ind -> new device found       */
    BtsecCauseDeviceChange           =  4,   /**< ind -> device data changed    */
    BtsecCauseDeviceDelete           =  5,   /**< ind -> device deleted         */
    BtsecCauseDeviceDeleteAll        =  6,   /**< ind -> all devices deleted    */
    BtsecCauseServiceAdd             =  7,   /**< ind -> new service founf      */
    BtsecCauseServiceDelete          =  8,   /**< ind -> service deleted        */
    BtsecCauseServiceDeleteAll       =  9,   /**< ind -> all services deleted   */
    BtsecCauseServiceDeleteForThisBd = 10,   /**< ind -> all services deleted   */
                                             /**<        for this device        */
    BtsecCauseEndOfList              = 11,   /**< requested item is not in list */
    BtsecCauseDiscoveryCancelled     = 12,   /**< ind -> discovery was canceled */
    BtsecCauseResourceDisplaced      = 13,   /**< ind -> QoS canceled (priority)*/
    BtsecCauseResourceRemoved        = 14,   /**< ind -> QoS canceled(link lost)*/

    BtsecCauseLast
} TBtsecCause;

/** Commands   */
/** BBBB MMMS UUUU UBBB */
/** message blocking flags  */
#define BS_BLOCK_IGN         0x0000 /**< message has no efect on block level  */
#define BS_IDLE__CLI         0x1000 /**< client goes to idle                  */
#define BS_BLOCK_CLI         0x2000 /**< message handling blocks client       */
#define BS_IDLE__SYS         0x4000 /**< system goes to idle                  */
#define BS_BLOCK_SYS         0x8000 /**< message handling blocks whole system */

#define BS_BLOCK_MASK        0xF000

/** message main groops  */
#define BS_COMMON            0x0200 /**< no specific section                  */
#define BS_OWN_DEVICE        0x0400 /**< own device section                   */
#define BS_GAP               0x0600 /**< inquiry, device/service handling     */
#define BS_SECURITY          0x0800 /**< security section                     */

#define BS_MAIN_GROOP_MASK   0x0E00

/** message special action flags */
#define BS_CANCEL            0x0100 /**< cancel pening request                */

#define BS_ACTION_MASK       0x0100

/** message sub groops */
#define BS_REGISTER          0x0008
#define BS_RELEASE           0x0010
#define BS_SET               0x0018
#define BS_GET               0x0020
#define BS_INQ_CLASS         0x0028
#define BS_INQUIRY           0x0030
#define BS_INQ_START         0x0038
#define BS_INQ_STOP          0x0040
#define BS_NAME_DIS          0x0048
#define BS_SERV_DIS          0x0050
#define BS_SERV_ATTR         0x0058
#define BS_DEVICE            0x0060
#define BS_DEVICE_GET        0x0068
#define BS_DEVICE_SET        0x0070
#define BS_SERVICE           0x0078
#define BS_SERVICE_GET       0x0080
#define BS_SERVICE_SET       0x0088
#define BS_USER_PIN          0x0090
#define BS_BOND              0x0098
#define BS_STATUS            0x00A0
#define BS_LINK_MODE         0x00A8
#define BS_QOS               0x00B8
#define BS_KEYPRESS_NOTIFICATION     0x00C0
#define BS_USER_PASSKEY_N    0x00C8
#define BS_USER_CONFREQ      0x00D0
#define BS_REMOTE_OOB        0x00D8
#define BS_SSP_COMPLETE      0x00E0
#define BS_USER_PASSKEY_R    0x00E8
#define BS_LOCAL_OOB         0x00F0
#define BS_SUB_GROOP_MASK    0x00F8

/** basic message types  */
#define BS_REQ               0x0001 /**< request   */
#define BS_CONF              0x0002 /**< confirmation  */
#define BS_IND               0x0003 /**< indication   */
#define BS_EXTIND            0x0004 /**< extended indication */
#define BS_EXTRESP           0x0005 /**< extended response   */

#define BS_BASIC_TYPE_MASK   0x0007

typedef enum _TBtsecCommand
{
    /** common messages */
	BtsecCommandRegisterReq                = BS_BLOCK_SYS|BS_COMMON    |BS_REGISTER   |BS_REQ,
    BtsecCommandRegisterConf               = BS_IDLE__SYS|BS_COMMON    |BS_REGISTER   |BS_CONF,
    BtsecCommandReleaseReq                 = BS_BLOCK_SYS|BS_COMMON    |BS_RELEASE    |BS_REQ,
    BtsecCommandReleaseConf                = BS_IDLE__SYS|BS_COMMON    |BS_RELEASE    |BS_CONF,
    BtsecCommandProgressInd                = BS_BLOCK_IGN|BS_COMMON    |BS_STATUS     |BS_IND,
    BtsecCommandGenericConf                = BS_IDLE__CLI|BS_COMMON                   |BS_CONF,
    /** messages for own device properties */
    BtsecCommandOwnDeviceSetReq            = BS_BLOCK_SYS|BS_OWN_DEVICE|BS_SET        |BS_REQ,
    BtsecCommandOwnDeviceSetConf           = BS_IDLE__SYS|BS_OWN_DEVICE|BS_SET        |BS_CONF,
    BtsecCommandOwnDeviceGetReq            = BS_BLOCK_SYS|BS_OWN_DEVICE|BS_GET        |BS_REQ,
    BtsecCommandOwnDeviceGetConf           = BS_IDLE__SYS|BS_OWN_DEVICE|BS_GET        |BS_CONF,
    BtsecCommandOwnDeviceChangeInd         = BS_BLOCK_IGN|BS_OWN_DEVICE               |BS_IND,
    BtsecCommandLinkModeSetReq             = BS_BLOCK_SYS|BS_OWN_DEVICE|BS_LINK_MODE  |BS_REQ,
    BtsecCommandLinkModeSetReqExtInd       = BS_BLOCK_SYS|BS_OWN_DEVICE|BS_LINK_MODE  |BS_EXTIND,
    BtsecCommandLinkModeSetReqExtResp      = BS_BLOCK_IGN|BS_OWN_DEVICE|BS_LINK_MODE  |BS_EXTRESP,
    BtsecCommandLinkModeSetConf            = BS_IDLE__SYS|BS_OWN_DEVICE|BS_LINK_MODE  |BS_CONF,
    BtsecCommandLinkModeInd                = BS_BLOCK_IGN|BS_OWN_DEVICE|BS_LINK_MODE  |BS_IND,
    BtsecCommandQoSSetReq                  = BS_BLOCK_IGN|BS_OWN_DEVICE|BS_QOS        |BS_REQ,
    BtsecCommandQoSSetConf                 = BS_BLOCK_IGN|BS_OWN_DEVICE|BS_QOS        |BS_CONF,
    BtsecCommandQoSInd                     = BS_BLOCK_IGN|BS_OWN_DEVICE|BS_QOS        |BS_IND,
    /** messages for device inquiry and service discovery */
    BtsecCommandInquiryClassMaskSetReq     = BS_BLOCK_SYS|BS_GAP       |BS_INQ_CLASS  |BS_REQ,
    BtsecCommandInquiryClassMaskSetConf    = BS_IDLE__SYS|BS_GAP       |BS_INQ_CLASS  |BS_CONF,
    BtsecCommandInquiryReq                 = BS_BLOCK_SYS|BS_GAP       |BS_INQUIRY    |BS_REQ,
    BtsecCommandInquiryConf                = BS_IDLE__SYS|BS_GAP       |BS_INQUIRY    |BS_CONF,
    BtsecCommandInquiryCancelReq           = BS_BLOCK_IGN|BS_GAP|BS_INQUIRY|BS_CANCEL |BS_REQ,
    BtsecCommandInquiryCancelConf          = BS_BLOCK_IGN|BS_GAP|BS_INQUIRY|BS_CANCEL |BS_CONF,
    BtsecCommandNameDiscoveryReq           = BS_BLOCK_SYS|BS_GAP       |BS_NAME_DIS   |BS_REQ,
    BtsecCommandNameDiscoveryConf          = BS_IDLE__SYS|BS_GAP       |BS_NAME_DIS   |BS_CONF,
    BtsecCommandNameDiscoveryCancelReq     = BS_BLOCK_IGN|BS_GAP|BS_NAME_DIS|BS_CANCEL|BS_REQ,
    BtsecCommandNameDiscoveryCancelConf    = BS_BLOCK_IGN|BS_GAP|BS_NAME_DIS|BS_CANCEL|BS_CONF,
    BtsecCommandServiceDiscoveryReq        = BS_BLOCK_CLI|BS_GAP       |BS_SERV_DIS   |BS_REQ,
    BtsecCommandServiceDiscoveryConf       = BS_IDLE__CLI|BS_GAP       |BS_SERV_DIS   |BS_CONF,
    BtsecCommandServiceDiscoveryCancelReq  = BS_BLOCK_IGN|BS_GAP|BS_SERV_DIS|BS_CANCEL|BS_REQ,
    BtsecCommandServiceDiscoveryCancelConf = BS_BLOCK_IGN|BS_GAP|BS_SERV_DIS|BS_CANCEL|BS_CONF,
    BtsecCommandServiceAttributeReq        = BS_BLOCK_CLI|BS_GAP       |BS_SERV_ATTR  |BS_REQ,
    BtsecCommandServiceAttributeExtInd     = BS_BLOCK_IGN|BS_GAP       |BS_SERV_ATTR  |BS_EXTIND,
    BtsecCommandServiceAttributeExtResp    = BS_BLOCK_IGN|BS_GAP       |BS_SERV_ATTR  |BS_EXTRESP,
    BtsecCommandServiceAttributeConf       = BS_IDLE__CLI|BS_GAP       |BS_SERV_ATTR  |BS_CONF,
    BtsecCommandServiceAttributeCancelReq  = BS_BLOCK_IGN|BS_GAP|BS_SERV_ATTR|BS_CANCEL|BS_REQ,
    BtsecCommandServiceAttributeCancelConf = BS_BLOCK_IGN|BS_GAP|BS_SERV_ATTR|BS_CANCEL|BS_CONF,
    BtsecCommandInquiryStartInd            = BS_BLOCK_IGN|BS_GAP       |BS_INQ_START  |BS_IND,
    BtsecCommandInquiryStopInd             = BS_BLOCK_IGN|BS_GAP       |BS_INQ_STOP   |BS_IND,
    BtsecCommandDeviceGetReq               = BS_BLOCK_IGN|BS_GAP       |BS_DEVICE_GET |BS_REQ,
    BtsecCommandDeviceGetConf              = BS_BLOCK_IGN|BS_GAP       |BS_DEVICE_GET |BS_CONF,
    BtsecCommandDeviceSetReq               = BS_BLOCK_IGN|BS_GAP       |BS_DEVICE_SET |BS_REQ,
    BtsecCommandDeviceSetConf              = BS_BLOCK_IGN|BS_GAP       |BS_DEVICE_SET |BS_CONF,
    BtsecCommandServiceGetReq              = BS_BLOCK_IGN|BS_GAP       |BS_SERVICE_GET|BS_REQ,
    BtsecCommandServiceGetConf             = BS_BLOCK_IGN|BS_GAP       |BS_SERVICE_GET|BS_CONF,
    BtsecCommandServiceSetReq              = BS_BLOCK_IGN|BS_GAP       |BS_SERVICE_SET|BS_REQ,
    BtsecCommandServiceSetConf             = BS_BLOCK_IGN|BS_GAP       |BS_SERVICE_SET|BS_CONF,
    BtsecCommandDeviceChangeInd            = BS_BLOCK_IGN|BS_GAP       |BS_DEVICE     |BS_IND,
    BtsecCommandServiceChangeInd           = BS_BLOCK_IGN|BS_GAP       |BS_SERVICE    |BS_IND,
    /** messages for security handling                                                   */
    BtsecCommandSecuritySetReq             = BS_BLOCK_IGN|BS_SECURITY  |BS_SET        |BS_REQ,
    BtsecCommandSecuritySetConf            = BS_BLOCK_IGN|BS_SECURITY  |BS_SET        |BS_CONF,
    BtsecCommandSecurityGetReq             = BS_BLOCK_IGN|BS_SECURITY  |BS_GET        |BS_REQ,
    BtsecCommandSecurityGetConf            = BS_BLOCK_IGN|BS_SECURITY  |BS_GET        |BS_CONF,
    BtsecCommandSecurityChangeInd          = BS_BLOCK_IGN|BS_SECURITY                 |BS_IND,
    BtsecCommandSecurityUserPinReqExtInd   = BS_BLOCK_IGN|BS_SECURITY  |BS_USER_PIN   |BS_EXTIND,
    BtsecCommandSecurityUserPinReqExtResp  = BS_BLOCK_IGN|BS_SECURITY  |BS_USER_PIN   |BS_EXTRESP,
    BtsecCommandSecurityUserPinCancelInd   = BS_BLOCK_IGN|BS_SECURITY  |BS_USER_PIN|BS_CANCEL|BS_IND,
    BtsecCommandSecurityBondReq            = BS_BLOCK_SYS|BS_SECURITY  |BS_BOND       |BS_REQ,
    BtsecCommandSecurityBondConf           = BS_IDLE__SYS|BS_SECURITY  |BS_BOND       |BS_CONF,
    BtsecCommandSecurityStatusInd          = BS_BLOCK_IGN|BS_SECURITY  |BS_STATUS     |BS_IND,
    /**  Start of Security Management part */
    BtsecCommandSecurityUserConfirmationReqInd        = BS_BLOCK_IGN|BS_SECURITY |BS_USER_CONFREQ |BS_EXTIND,
    BtsecCommandSecurityUserConfirmationReqConf       = BS_BLOCK_IGN|BS_SECURITY |BS_USER_CONFREQ |BS_CONF,
    BtsecCommandSecurityUserAuthorizationReqInd       = BS_BLOCK_IGN|BS_SECURITY |BS_BOND |BS_EXTIND,
    BtsecCommandSecurityUserAuthorizationReqConf      = BS_BLOCK_IGN|BS_SECURITY |BS_BOND |BS_EXTRESP,
    BtsecCommandSecurityUserPasskeyReqInd             = BS_BLOCK_IGN|BS_SECURITY |BS_USER_PASSKEY_R |BS_EXTIND,
    BtsecCommandSecurityUserPasskeyReqConf            = BS_BLOCK_IGN|BS_SECURITY |BS_USER_PASSKEY_R |BS_CONF,
    BtsecCommandSecurityUserPasskeyReqReplyReq        = BS_BLOCK_IGN|BS_SECURITY |BS_USER_PASSKEY_R |BS_REQ,
    BtsecCommandSecurityUserPasskeyReqReplyRsp        = BS_BLOCK_IGN|BS_SECURITY |BS_USER_PASSKEY_R |BS_EXTRESP,
    BtsecCommandSecurityUserPasskeyNotificationInfo   = BS_BLOCK_IGN|BS_SECURITY |BS_USER_PASSKEY_N |BS_EXTIND,
    BtsecCommandSecurityUserKeypressNotificationReq   = BS_BLOCK_IGN|BS_SECURITY |BS_KEYPRESS_NOTIFICATION |BS_REQ,
    BtsecCommandSecurityKeypressNotificationRsp       = BS_BLOCK_IGN|BS_SECURITY |BS_KEYPRESS_NOTIFICATION |BS_EXTRESP,
    BtsecCommandSecurityKeypressNotificationInfo      = BS_BLOCK_IGN|BS_SECURITY |BS_KEYPRESS_NOTIFICATION |BS_IND,
    BtsecCommandSecurityRemoteOOBDataReqInd           = BS_BLOCK_IGN|BS_SECURITY |BS_REMOTE_OOB |BS_EXTIND,
    BtsecCommandSecurityRemoteOOBDataReqConf          = BS_BLOCK_IGN|BS_SECURITY |BS_REMOTE_OOB |BS_EXTRESP,
	BtsecSecurityLocalOOBDataReq                      = BS_BLOCK_IGN|BS_SECURITY |BS_LOCAL_OOB |BS_REQ,
    BtsecSecurityLocalOOBDataRsp                      = BS_BLOCK_IGN|BS_SECURITY |BS_LOCAL_OOB |BS_EXTRESP,

    /** gerneric confirmation  */
    BtsecCommandLast
} TBtsecCommand;

/**
* Register request  
* Description:
* has to be used to register an app to blue secure  
* the flags indicate the type of events to app whants to be informed of    */
typedef struct _TBtsecRegisterReq
{
    uint32_t               Flag;             /**< valid flags for this application */
                                        /**< grep 'BTSEC_FLAG_' for more info */
} TBtsecRegisterReq; 
typedef TBtsecRegisterReq  * PBtsecRegisterReq;

/** Register confirmation */
/** Description:----*/
/** the confirmation of a register request. If the Cause indicates success   */
/** the handle has to be stored and used for all msg to come                 */
typedef struct _TBtsecRegisterConf
{
	TBtsecCause         Cause;           /**< indicates result of reg. attempt  */
	THandle             Handle;          /**< has to be stored and used for all */
                                       /**< communication with blue secure    */
} TBtsecRegisterConf; 
typedef TBtsecRegisterConf  * PBtsecRegisterConf;

/** Release request */
/** Description:*/
/** has to be user to release from blue secure. no more messages will be     */
/** accepted after this                                                      */
typedef struct _TBtsecReleaseReq
{
	uint8_t                Dummy;           /**< nothing                           */
} TBtsecReleaseReq;
typedef TBtsecReleaseReq  * PBtsecReleaseReq;

/** Release confirmation */
/** Description:---*/
/** confirmation for release attempt. If cause indicates success, this will  */
/** be the last msg to be received from blue secure for this app             */
typedef struct _TBtsecReleaseConf
{
	TBtsecCause         Cause;
} TBtsecReleaseConf; 
typedef TBtsecReleaseConf  * PBtsecReleaseConf;

/** generic confirmation */
/** Description:---*/
/** this msg is a confirmation for an unknown request */
typedef struct _TBtsecGenericConf
{
    TBtsecCause         Cause;    /**< cause for this message */
    uint16_t                RawError; /**< error code  */
} TBtsecGenericConf; 
typedef TBtsecGenericConf  * PBtsecGenericConf;
                                                               
/** own device parameter handling  */
/** TBtsecOwnDeviceParameterId  */
typedef enum _TBtsecOwnDeviceParameterId
{
	BtsecOwnDeviceParameterIdName     = 1,      /**< device name                */
    BtsecOwnDeviceParameterIdClassCode,         /**< device class code          */
    BtsecOwnDeviceParameterIdBd,                /**< bluetooth address          */
    BtsecOwnDeviceParameterIdInqScanRepMode,    /**< inq. scan repetition mode  */
    BtsecOwnDeviceParameterIdPageScanRepMode,   /**< page scan repetition mode  */
    BtsecOwnDeviceParameterIdPageMode,          /**< inq./page scan mode        */
    BtsecOwnDeviceParameterIdDeviceRole,        /**< device role (master/slave) */
    BtsecOwnDeviceParameterIdLinkPolicy,        /**< link policy (role/mode)    */
    BtsecOwnDeviceParameterIdSupervisionTimeout,/**< supervision timeout        */
    BtsecOwnDeviceParameterIdIAC,               /**< Inquiry Access Codes       */
    BtsecOwnDeviceParameterIdLast               /**< do not use!!               */
} TBtsecOwnDeviceParameterId; /**< idetifies the parameter to be handled      */

/** TBtsecOwnDeviceScanRepModeType */
typedef enum _TBtsecOwnDeviceScanRepModeType
{
    BtsecOwnDeviceScanTypeStandard   = 0,       /**< mandatrory:norm. scan mode */
    BtsecOwnDeviceScanTypeInterlaced = 1        /**< optional  :interlaced mode */
} TBtsecOwnDeviceScanRepModeType;
typedef TBtsecOwnDeviceScanRepModeType * PBtsecOwnDeviceScanRepModeType;

/** TBtsecOwnDeviceInqScanRepMode */
typedef struct _TBtsecOwnDeviceInqScanRepMode
{
  	uint16_t interval;                     /**< interval/window in 625us slots, see */
	uint16_t window;                       /**< BLUEFACE_CONF_INQUIRY_SCAN_ACTIVITY */
   	TBtsecOwnDeviceScanRepModeType scanType;
 } TBtsecOwnDeviceInqScanRepMode;     /**< parameter for Inq. scan rep. mode   */
typedef TBtsecOwnDeviceInqScanRepMode * PBtsecOwnDeviceInqScanRepMode;

/** TBtsecOwnDevicePageScanRepMode **/
typedef enum _TBtsecOwnDevicePageScanRepModeType
{
	BtsecOwnDevicePageScanRepModeTypeContinousScan   = 0,
	BtsecOwnDevicePageScanRepModeType_1_28s_Interval = 1,
 	BtsecOwnDevicePageScanRepModeType_2_25s_Interval = 2,
   	BtsecOwnDevicePageScanRepModeTypeManualSetting   = 0xFF /**< apply interval*/
                                                           /**< and window    */
} TBtsecOwnDevicePageScanRepModeType;
typedef TBtsecOwnDevicePageScanRepModeType * PBtsecOwnDevicePageScanRepModeType;

/** TBtsecOwnDevicePageScanRepMode */
typedef struct _TBtsecOwnDevicePageScanRepMode
{
  	TBtsecOwnDevicePageScanRepModeType repetitionMode;
  	uint16_t intervalMs;                   	/**< interval/window in 625us slots, see */
  	uint16_t windowMs;                     	/**< BLUEFACE_CONF_PAGE_SCAN_ACTIVITY    */
   	TBtsecOwnDeviceScanRepModeType scanType;
 } TBtsecOwnDevicePageScanRepMode;    /**< parameter for page scan rep. mode   */
typedef TBtsecOwnDevicePageScanRepMode * PTBtsecOwnDevicePageScanRepMode;

/** TBtsecWriteScanEnable  */
typedef enum _TBtsecWriteScanEnable
{
  	BtsecOwnDeviceNoScanEnabled             = 0,/**< not visable, not connectabl*/
  	BtsecOwnDeviceInquiryScanEnabled        = 1,/**< visable, not connectable   */
  	BtsecOwnDevicePageScanEnable            = 2,/**< connectable, not visable   */
  	BtsecOwnDeviceInquiryAndPageScanEnabled = 3 /**< visable and connectable    */
} TBtsecWriteScanEnable;             /**< settings for page mode              */
typedef TBtsecWriteScanEnable * PBtsecWriteScanEnable;

/** TBtsecOwnDevicePageMode */
typedef struct _TBtsecOwnDevicePageMode
{
  	TBtsecWriteScanEnable PageMode;    		/**< page mode                           */
} TBtsecOwnDevicePageMode;           /**< parameter for page mode             */
typedef TBtsecOwnDevicePageMode * PTBtsecOwnDevicePageMode;

/** TBtsecOwnDeviceRole */
typedef enum _TBtsecOwnDeviceRole
{
  	BtsecOwnDeviceRoleSlaveRequired,   /**< only connect as slave               */
  	BtsecOwnDeviceRoleSlavePreferred,  /**< try to connect as slave             */
  	BtsecOwnDeviceRoleDontCare,        /**< do not care for role                */
  	BtsecOwnDeviceRoleMasterPreferred, /**< try to connect as master            */
  	BtsecOwnDeviceRoleMasterRequired,  /**< only connect as master              */
  	BtsecOwnDeviceRoleLast             /**< do not use!!                        */
} TBtsecOwnDeviceDeviceRole;         /**< defines device role of device       */
typedef TBtsecOwnDeviceDeviceRole * PBtsecOwnDeviceDeviceRole;

/** BTSEC_LINK_POLICY settings */
#define BTSEC_LINK_POLICY_DISABLE_ALL         0x00
#define BTSEC_LINK_POLICY_ENABLE_ROLE_SWITCH  0x01
#define BTSEC_LINK_POLICY_ENABLE_HOLD_MODE    0x02
#define BTSEC_LINK_POLICY_ENABLE_SNIFF_MODE   0x04
#define BTSEC_LINK_POLICY_ENABLE_PARK_MODE    0x08
#define BTSEC_LINK_POLICY_VALID_MASK          0x0F

/** TBtsecOwnDeviceLinkPolicy */
typedef uint16_t TBtsecOwnDeviceLinkPolicy;	/**< a combination of BTSEC_LINK_POLICY*/
                                       			/**< settings  */
typedef TBtsecOwnDeviceLinkPolicy * PBtsecOwnDeviceLinkPolicy;

/** TBtsecOwnDeviceSupervisionTimeout  */
typedef uint16_t TBtsecOwnDeviceSupervisionTimeout;
typedef TBtsecOwnDeviceSupervisionTimeout * PBtsecOwnDeviceSupervisionTimeout;

/** TBtsecOwnDeviceNumOfIAC */
typedef struct _TBtsecOwnDeviceNumOfIAC
{
  	uint8_t  cnt;                                                  /**< count of IACs to use  */
  	uint8_t  idxIACs[BTSEC_OWN_DEVICE_MAX_NUM_OF_IAC];             /**< table of IAC indexes   */
} TBtsecOwnDeviceNumOfIAC, * PBtsecOwnDeviceNumOfIAC;      /**< parameter for WRITE_CURRENT_IAC_LAP */

/** TBtsecOwnDeviceParameterValue  */
typedef  union _TBtsecOwnDeviceParameterValue
{
  	char                              Name[BTSEC_OWN_DEVICE_NAME_LENGTH+1/**< null terminated string */];
  	uint32_t                             ClassCode;
  	TBdAddr                           Bd;
  	TBtsecOwnDevicePageScanRepMode    PageScanRepMode;
  	TBtsecOwnDeviceInqScanRepMode     InqScanRepMode;
  	TBtsecOwnDevicePageMode           PageMode;
  	TBtsecOwnDeviceDeviceRole         DeviceRole;
  	TBtsecOwnDeviceLinkPolicy         LinkPolicy;
  	TBtsecOwnDeviceSupervisionTimeout SupervisionTimeout;
  	TBtsecOwnDeviceNumOfIAC           numOfIAC;
} TBtsecOwnDeviceParameterValue; /**< includes parameter values to be handled */
typedef TBtsecOwnDeviceParameterValue * PBtsecOwnDeviceParameterValue;

typedef struct _TBtsecOwnDeviceSetReq
{
  	TBtsecOwnDeviceParameterId    Id;    /**< defines parameter to be changed   */
  	TBtsecOwnDeviceParameterValue Value; /**< new parameter setting             */
} TBtsecOwnDeviceSetReq; 
typedef TBtsecOwnDeviceSetReq  * PBtsecOwnDeviceSetReq;

/** own device set confirmation */
/** Description:--*/
/** confirmation for parameter change attempt. Is cause indicates success    */
/** parameter is changed    */
typedef struct _TBtsecOwnDeviceSetConf
{
  	TBtsecCause                   Cause;    /**< result of change attempt       */
  	uint16_t                          RawError; /**< error code                     */
  	TBtsecOwnDeviceParameterId    Id;       /**< defines affected parameter     */
} TBtsecOwnDeviceSetConf; 
typedef TBtsecOwnDeviceSetConf  * PBtsecOwnDeviceSetConf;

/** own device set request */
/** Description:-*/
/** can be used to get on of the parameter tor the own device                */
typedef struct _TBtsecOwnDeviceGetReq
{
  	TBtsecOwnDeviceParameterId    Id;    /**< defines requested parameter       */
} TBtsecOwnDeviceGetReq;
typedef TBtsecOwnDeviceGetReq  * PBtsecOwnDeviceGetReq;

/** own device get confirmation  */
/** Description:*/
/** confirmation for parameter get attempt. If cause indicates success       */
/** parameter setting is included in this msg      */
typedef struct _TBtsecOwnDeviceGetConf
{
  	TBtsecCause                   Cause; /**< result of get attempt             */
  	TBtsecOwnDeviceParameterId    Id;    /**< defines requested parameter       */
  	TBtsecOwnDeviceParameterValue Value; /**< requested parameter setting       */
} TBtsecOwnDeviceGetConf; 
typedef TBtsecOwnDeviceGetConf  * PBtsecOwnDeviceGetConf;

/** own device change indication  */
/** Description:*/
/** indicate that for some reason one of the settings of a 'own device'      */
/** parameter is changed                                                     */
/** Enable register-flag BTSEC_FLAG_OWN_DEVICE_INDICATION to get this message*/
typedef struct _TBtsecOwnDeviceChangeInd
{
  	TBtsecOwnDeviceParameterId    Id;    /**< defines changed parameter         */
  	TBtsecOwnDeviceParameterValue Value; /**< changed parameter setting         */
} TBtsecOwnDeviceChangeInd; 
typedef TBtsecOwnDeviceChangeInd  * PBtsecOwnDeviceChangeInd;
                                                                        
/** link mode handling  */
/** TBtsecLinkModeId  */
typedef enum _TBtsecLinkModeId
{
  	BtsecLinkModeIdEnterActiveMode     = 0, /**< link goes to fully active mode */
  	BtsecLinkModeIdEnterHoldMode       = 1, /**< link goes to hold mode         */
  	BtsecLinkModeIdEnterSniffMode      = 2, /**< link goes to sniff mode        */
  	BtsecLinkModeIdEnterParkMode       = 3, /**< link goes to park mode         */
  	BtsecLinkModeIdIsInActiveMode      = 4, /**< link is in fully active mode   */
  	BtsecLinkModeIdIsInHoldMode        = 5, /**< link is in hold mode           */
  	BtsecLinkModeIdIsInSniffMode       = 6, /**< link is in sniff mode          */
  	BtsecLinkModeIdIsInParkMode        = 7, /**< link is in park mode           */
  	BtsecLinkModeIdEnterMasterRole     = 8, /**< we try to become master/slave  */
  	BtsecLinkModeIdIsMasterRole        = 9, /**< we are link master/slave       */
  	BtsecLinkModeIdEnterSlaveRole      =10, /**< we try to become master/slave  */
  	BtsecLinkModeIdIsSlaveRole         =11, /**< we are link master/slave       */
  	BtsecLinkModeIdConACLConnection    =12, /**< there is a new ACL connection  */
  	BtsecLinkModeIdDisACLConnection    =13, /**< there is a ACL connection      */
                                          /**< disconnected                   */
  	BtsecLinkModeIdLast
} TBtsecLinkModeId;                       /**< defines link mode              */
typedef TBtsecLinkModeId * PBtsecLinkModeId;

/** TBtsecSniffMode */
typedef struct _TBtsecSniffMode
{
  	uint16_t        MinSniffInterval;           /**< min sniff intervall in slots   */
  	uint16_t        MaxSniffInterval;           /**< min sniff intervall in slots   */
  	uint16_t        Attempt;                    /**< rec. slots to wait for master  */
  	uint16_t        TimeOut;                    /**< rec. slots to stay awake after */
                                          /**< last transaction               */
} TBtsecSniffMode;                        /**< defines param. for sniff mode  */
typedef TBtsecSniffMode * PBtsecSniffMode;

/** TBtsecLinkRoleParameter */
typedef struct _TBtsecLinkRoleParameter
{
   BOOL required;
} TBtsecLinkRoleParameter;
typedef TBtsecLinkRoleParameter * PBtsecLinkRoleParameter;

/** TBtsecLinkModeParameter */
typedef union _TBtsecLinkModeParameter
{
  	TBtsecSniffMode         sniff;          /**< the only supported link mode by*/
                                          /**< btcore                         */
  	TBtsecLinkRoleParameter linkRole;       /**< aktiv link roule,(master/slave)*/
} TBtsecLinkModeParameter;                /**< defines param. for link modes  */
typedef TBtsecLinkModeParameter * PBtsecLinkModeParameter;

/** link mode request extended indication */
/** Description:-*/
/** this msg does indicate that an app wants to put a specific ACL link into */
/** a new link mode. The 'LinkModeSetReqResp' response msg can be used to    */
/** refuse the mode change.                                                  */
/** This message MUST answered with a LinkModeSetReqExtResp message          */
/** Enable register-flag BTSEC_FLAG_LINK_MODE_INDICATION to get this message */
typedef struct _TBtsecLinkModeSetReqExtInd
{
  	TBtsecLinkModeId        Id;             /**< type of link mode              */
  	TBdAddr                 Bd;             /**< reference affected ACL link    */
  	TBtsecLinkModeParameter p;              /**< specific link mode parameter   */
} TBtsecLinkModeSetReqExtInd; 
typedef TBtsecLinkModeSetReqExtInd * PBtsecLinkModeSetReqExtInd;

/** link mode request extended response */
/** Description:*/
/** with this msg an app might reject a requestet link mode change on a      */
/** specific ACL link                                                        */
typedef struct _TBtsecLinkModeSetReqExtResp
{
  	TBtsecLinkModeId Id;                    /**< type of link mode              */
  	TBdAddr          Bd;                    /**< reference affected ACL link    */
  	TBtsecCause      Cause;                 /**< Cause!=BtsecCauseSuccess ==>   */
                                          /**< reject link mode change        */
} TBtsecLinkModeSetReqExtResp; 
typedef TBtsecLinkModeSetReqExtResp * PBtsecLinkModeSetReqExtResp;

/** link mode set request */
/** Description:---*/
/** with this msg an app might request a link mode change on a speciffic     */
/** ACL link. This request might be rejected by other apps in the system     */
typedef struct _TBtsecLinkModeSetReq
{
  	TBtsecLinkModeId Id;            /**< type of link mode                      */
  	TBdAddr          Bd;            /**< reference affected ACL link            */
  	THandle          appLinkHandle; /**<!!!!! temporary !!!!!    */
                                  /**< !!!!! remove when possible!!!!         */
                                  /**< (btcore needs this right now..)        */
  	TBtsecLinkModeParameter  p;     /**< link mode parameter                    */
} TBtsecLinkModeSetReq; 
typedef TBtsecLinkModeSetReq * PBtsecLinkModeSetReq;

/** link mode set confirmation  */
/** Description:--*/
/** this msg is the confirmation for a LinkModeSetReq msg. If the cause      */
/** indicates success the link mode is changed to the requested mode         */
typedef struct _TBtsecLinkModeSetConf
{
  	TBtsecLinkModeId Id;                    /**< type of link mode              */
  	TBdAddr          Bd;                    /**< reference affected ACL link    */
  	TBtsecCause      Cause;                 /**< result for request             */
  	uint16_t             RawError;              /**< error code                     */
} TBtsecLinkModeSetConf; 
typedef TBtsecLinkModeSetConf * PBtsecLinkModeSetConf;

/** link mode indication  */
/** Description:-----*/
/** this msg indicates that the link mode of a specific ACL link is changed  */
/** this can be done from the local-, or remote side                         */
/**                                                                          */
/** Enable register-flag BTSEC_FLAG_LINK_MODE_INDICATION to get this message */
typedef struct _TBtsecLinkModeInd
{
  	TBtsecLinkModeId  Id;                   /**< type of link mode              */
  	TBdAddr           Bd;                   /**< reference affected ACL link    */
  	uint16_t              Interval;             /**< mode interval                  */
                                          /**< no blueface/HCI doku.....      */
  	uint16_t              RawStatus;            /**< raw HCI_MODE_CHANGE status     */
} TBtsecLinkModeInd; 
typedef TBtsecLinkModeInd * PBtsecLinkModeInd;

/** QoS handling  */
/** TBtsecQoSId */
typedef enum _TBtsecQoSId
{
  	BtsecQoSIdApply = 0,                    /**< add QoS requirement to link    */
  	BtsecQoSIdDelete,                       /**< delete QoS requirement of link */
  	BtsecQoSIdLast
} TBtsecQoSId;
typedef TBtsecQoSId * PBtsecQoSId;

/** BTSEC_QOS_VALID_ */
#define BTSEC_QOS_VALID_PRIORITY       0x0001
#define BTSEC_QOS_VALID_MIN_THROUGHPUT 0x0002
#define BTSEC_QOS_VALID_MAX_LATENCY    0x0004
#define BTSEC_QOS_VALID_ALL_VALID      0xFFFF

/** TBtsecQoSParameter */
typedef struct _TBtsecQoSParameter
{
  	uint16_t        validMask;                  /**< defines witch of the parameter */
  	uint8_t        Priority;                   /**< priority of request. Allowed   */
                                          /**< range is 1 (low) to 127 (high) */
                                          /**< of the QoS settings are valid. */
  	/** All 'BTSEC_QOS_VALID_' combinations are allowed                        */
  	uint32_t       ThroughputMinBitsPerSecond; /**< defined minimum Throughput     */
                                          /**< on baseband level              */
  	uint16_t        LatencyMax625usSlots;       /**< defined maximum Latency        */
                                          /**< in 625us slots                 */
} TBtsecQoSParameter; /**< defines QoS requirements                           */
typedef TBtsecQoSParameter * PBtsecQoSParameter;

/** TBtsecQoSLink  */
typedef struct _TBtsecQoSLinkParameter
{
	BLINKHANDLE        LinkHandle;          /**< Blueface handle f. logical Link*/
} TBtsecQoSLink;
typedef TBtsecQoSLink * PBtsecQoSLink;

/** Quality of Service set request */
/** Description:*/
/** this msg can be used to request certain QoS requirement for a link       */
typedef struct _TBtsecQoSSetReq
{
	TBtsecQoSId        Id;                  /**< type of QoS action             */
	TBdAddr            Bd;                  /**< reference affected ACL link    */
  	TBtsecQoSLink      link;                /**< Blueface/Bluetooth identifier  */
  	TBtsecQoSParameter para;                /**< specific QoS parameter         */
} TBtsecQoSSetReq; 
typedef TBtsecQoSSetReq * PBtsecQoSSetReq;

/** Quality of Service set confirmation */
/** Description:*/
/** this msg is a confirmation to a QoSSetReq message                        */
typedef struct _TBtsecQoSSetConf
{
	TBtsecQoSId   Id;                       /**< type of QoS action             */
	TBdAddr       Bd;                       /**< reference affected ACL link    */
	TBtsecQoSLink link;                     /**< Blueface handle f. logical Link*/
	TBtsecCause   Cause;                    /**< result for request             */
	uint16_t          RawError;                 /**< error code                     */
} TBtsecQoSSetConf; 
typedef TBtsecQoSSetConf * PBtsecQoSSetConf;

/** Quality of Service indication */
/** Description:-*/
/** this msg indicates a status change of given QoS requirements to a link   */
typedef struct _TBtsecQoSInd
{
	TBtsecQoSId        Id;                  /**< type of QoS action             */
	TBdAddr            Bd;                  /**< reference affected ACL link    */
	TBtsecQoSLink      link;                /**< Blueface handle f. logical Link*/
	TBtsecQoSParameter para;                /**< specific QoS parameter         */
	TBtsecCause        Cause;               /**< cause for indication           */
} TBtsecQoSInd;
typedef TBtsecQoSInd * PBtsecQoSInd;

/** progress indication handling  */
/** TBtsecProgressId */
typedef enum _TBtsecProgressId
{
	BtsecProgressIdNbrOfConnections = 1, /**< number of ACL/SCO connections has */
                                       /**< changed                           */
	BtsecProgressIdLast                  /**< do not use!!                      */
} TBtsecProgressId;                    /**< generic progress indication       */

/** TBtsecProgressValue */
typedef struct _TBtsecProgressValue
{
	uint16_t                NbrOfConnectionsAcl; /**< number of ACL connections     */
	uint16_t                NbrOfConnectionsSco; /**< number of SCO connections     */
} TBtsecProgressValue;                     /**< data for 'IdNbrOfConnections' */
typedef TBtsecProgressValue  * PBtsecProgressValue;

/** system progress indocation */
/** Description:*/
/** this msg indicates generic information of the bluetooth stack            */
/**                                                                          */
/** Enable register-flag BTSEC_FLAG_PROGRESS_INDICATION to get this message  */
typedef struct _TBtsecProgressInd
{
	TBtsecProgressId    Id;    /**< identifier for event                        */
	TBtsecProgressValue Value; /**< event specific data                         */
} TBtsecProgressInd; 
typedef TBtsecProgressInd  * PBtsecProgressInd;


/**  Device database handling */
/**  TBtsecGetWhat */
typedef enum _TBtsecGetWhat
{
	BtsecGetFirst,        /**< get first element from database                  */
                        /**< works for services and devices                   */
	BtsecGetNext,         /**< get next element from database                   */
                        /**< works for devices, bd identifies 'last' device   */
	BtsecGetThisBd,       /**< get device for this device address               */
	BtsecGetThisBdFirst,  /**< get first service for this device address        */
	BtsecGetThisBdNext,   /**< get next service for this device address         */
                        /**< service handle identifies 'last' service         */
	BtsecGetThisListPos   /**< get specific element from list                   */
} TBtsecGetWhat;        /**< defines get operation in device/service database */

/** TBtsecSetWhat */
typedef enum _TBtsecSetWhat
{
	BtsecSetAdd,        /**< add a new element to database                      */
	BtSecSetChange,     /**< change a known element in database                 */
	BtSecSetDelete,     /**< delete a element in database                       */
	BtSecSetDeleteAll   /**< clear this database                                */
} TBtsecSetWhat;      /**< defines get operation in device/service database   */

/** TBtsecDeviceEntry */
typedef struct _TBtsecDeviceEntry
{
	TBdAddr Bd;                               /**< device bluetooth address     */
	uint32_t   ClassCode;                        /**< device class code            */
	uint8_t    Name[BTSEC_DEVICE_NAME_LENGTH+1]; /**< device name                  */
   uint8_t    rssi;                             /**< signal quality               */
 } TBtsecDeviceEntry;                        /**< database device entry        */
typedef TBtsecDeviceEntry  * PBtsecDeviceEntry;

/** device get request                                                       */
/** Description:*/
/** this msg get a device out of device database                             */
typedef struct _TBtsecDeviceGetReq
{
	TBtsecGetWhat What;      /**< specifies operation                           */
	TBdAddr       Bd;        /**< device handle                                 */
	uint16_t          ListPos;   /**< enumeration position in device list           */
} TBtsecDeviceGetReq; 
typedef TBtsecDeviceGetReq  * PBtsecDeviceGetReq;

/** device get confirmation  */
/** Description:*/
/** confirmation for device get attempt. If cause indicates success requested*/
/** device is included in msg                                                */
typedef struct _TBtsecDeviceGetConf
{
	TBtsecCause       Cause;    /**< result of get attempt                      */
	uint16_t              RawError; /**< error code                                 */
	TBtsecDeviceEntry Device;   /**< requested device                           */
} TBtsecDeviceGetConf; 
typedef TBtsecDeviceGetConf  * PBtsecDeviceGetConf;

/** device set request */
/** Description:*/
/** this messages can be used to change the content of the database          */
typedef struct _TBtsecDeviceSetReq
{
	TBtsecSetWhat     What;   /**< specifies operation                          */
	TBtsecDeviceEntry Device; /**< new device data                              */
} TBtsecDeviceSetReq; 
typedef TBtsecDeviceSetReq  * PBtsecDeviceSetReq;

/** device set confirmation  */
/** Description:*/
/** confirmation of device set attempt. If cause indicated success device    */
/** data is added/changed in database                                        */
typedef struct _TBtsecDeviceSetConf
{
	TBtsecCause       Cause;    /**< result of set attempt                      */
	uint16_t              RawError; /**< error code                                 */
	TBtsecDeviceEntry Device;   /**< identified changed device                  */
} TBtsecDeviceSetConf; 
typedef TBtsecDeviceSetConf  * PBtsecDeviceSetConf;

/** device change indication */
/** Description:-*/
/** indicates that the device database contend has changed for some reason.  */
/**                                                                          */
/** Enable register-flag BTSEC_FLAG_DEVICE_INDICATION to get this message    */
typedef struct _TBtsecDeviceChangeInd
{
	TBtsecCause       Cause;    /**< reason for change                          */
	uint16_t              RawError; /**< error code                                 */
	TBtsecDeviceEntry Device;   /**< new device data                            */
} TBtsecDeviceChangeInd; 
typedef TBtsecDeviceChangeInd  * PBtsecDeviceChangeInd;


/**  Service database handling */
/** TBtsecUUID */

typedef uint32_t TBtsecUUID;
typedef TBtsecUUID  * PBtsecUUID;

/** TBtsecServiceEntry */
typedef struct _TBtsecServiceEntry
{
	TBdAddr    Bd;                             /**< bluetooth address of device */
                                             /**< that offeres this service   */
	char       Name[BTSEC_SERVICE_NAME_LENGTH+1]; /**< service name             */
	TBtsecUUID Uuid;                           /**< UUID identifier for service */
	uint32_t      ServerChannel;                  /**< service ch./psm of service  */
	uint32_t      ServiceHandle;                  /**< service handle from remote  */
                                             /**< device for this service     */
	uint16_t       RawDataLength;                  /**< length of raw data          */
} TBtsecServiceEntry;                        /**< database service entry      */
typedef TBtsecServiceEntry  * PBtsecServiceEntry;

/** TBtsecServiceId */
typedef THandle TBtsecServiceId;             /**< service handle              */

/** service get request */
/** Description:*/
/** this msg can be used to get a service out of the service database        */
typedef struct _TBtsecServiceGetReq
{
	TBtsecGetWhat      What;    /**< identifier for requested operation         */
	TBdAddr            Bd;      /**< device address of owner device             */
	TBtsecServiceId    Id;      /**< service handle for requested service       */
	uint16_t               ListPos; /**< service enumeration position for device    */
} TBtsecServiceGetReq; 
typedef TBtsecServiceGetReq  * PBtsecServiceGetReq;

/** service get confirmation                                                 */
/** Description:-*/
/** confirmation for service get attempt. If cause indicates success the     */
/** requested service is includes in this msg                                */
typedef struct _TBtsecServiceGetConf
{
	TBtsecCause        Cause;    /**< result for get attempt                    */
	uint16_t               RawError; /**< error code                                */
	TBtsecServiceId    Id;       /**< service handle for requested service      */
	TBtsecServiceEntry Service;  /**< requested service data                    */
} TBtsecServiceGetConf; 
typedef TBtsecServiceGetConf  * PBtsecServiceGetConf;

/** service set request */
/** Description:-*/
/** with this msg a service can be added/changed in service database         */
typedef struct _TBtsecServiceSetReq
{
	TBtsecSetWhat      What;    /**< identifier for requested operation         */
	TBtsecServiceEntry Service; /**< new service data                           */
} TBtsecServiceSetReq; 
typedef TBtsecServiceSetReq  * PBtsecServiceSetReq;

/** service set confirmation */
/** Description:-*/
/** this is the confirmation for a service set attempt.If the cause indicates*/
/** success the service data is added/changed in the database                */
typedef struct _TBtsecServiceSetConf
{
	TBtsecCause        Cause;    /**< result for set attempt                    */
	uint16_t               RawError; /**< error code                                */
	TBtsecServiceEntry Service;  /**< added/changed service data                */
} TBtsecServiceSetConf; 
typedef TBtsecServiceSetConf  * PBtsecServiceSetConf;

/** service change in dication */
/** Description:-*/
/** indicates that the service database contend has changed for some reason. */
/**                                                                          */
/** Enable register-flag BTSEC_FLAG_DEVICE_INDICATION to get this message    */
typedef struct _TBtsecServiceChangeInd
{
	TBtsecCause        Cause;    /**< reason for change                         */
	uint16_t               RawError; /**< error code                                */
	TBtsecServiceEntry Service;  /**< added/changed service data                */
} TBtsecServiceChangeInd;
typedef TBtsecServiceChangeInd  * PBtsecServiceChangeInd;


/** inquiry handling  */
/** class mask set request */
/** Description:*/
/** with this msg you can set specific device classes that should be found   */
/** with inquiries to come. All other devices fill be filtered out           */
/** you can set codes in BTSEC_INQUIRY_CLASS_MASK_COUNT                      */
/** nothing set (default)  : NO mask set => store all for me                 */
/** special code 0x00000000: mask set to nothing: store nothing for me       */
typedef struct _TBtsecInquiryClassMaskSetReq
{
	TBtsecSetWhat What;           /**< add/remove classcode from filter list    */
	uint32_t         ClassCode;      /**< request data                             */
} TBtsecInquiryClassMaskSetReq; 
typedef TBtsecInquiryClassMaskSetReq  * PBtsecInquiryClassMaskSetReq;

/** class mask set confirmation  */
/** Description:*/
/** this msg is the confirmation for a ClassMaskSetReq message. If the cause */
/** indicates success, the requested operation is done                       */
typedef struct _TBtsecInquiryClassMaskSetConf
{
	TBtsecCause Cause;             /**< result of request attempt               */
	uint16_t        RawError;          /**< error code                              */
	uint32_t       ClassCode;         /**< requested data                          */
} TBtsecInquiryClassMaskSetConf;
typedef TBtsecInquiryClassMaskSetConf  * PBtsecInquiryClassMaskSetConf;

/** inquiry request  */
/** Description:---*/
/** this msg can be used to start an inquiry                                 */
/** The results can be filtered using ClassMask and ClassValue.              */
/** ClassMask defines a mask for the remote device’s classcode               */
/** (classcode & classmask). The masked classcode is compared to ClassValue. */
/** Only classcodes matching the filter are reported                         */
typedef struct _TBtsecInquiryReq
{
	uint16_t   Timeout;                 /**< timeout for inquiry                    */
	uint32_t  ClassMask;               /**< mask for searched classcode            */
	uint32_t  ClassValue;              /**< reference value for classcode          */
	uint8_t   idxIAC;                  /**< inquiry access code index              */
} TBtsecInquiryReq; 
typedef TBtsecInquiryReq  * PBtsecInquiryReq;

/** inquiry cancel request  */
/** Description:--*/
/** this message can be used to stop a running inquiry                       */
typedef struct _TBtsecInquiryCancelReq
{
	BOOL   dummy;                   /**< no parameter used                      */
} TBtsecInquiryCancelReq;
typedef TBtsecInquiryCancelReq  * PBtsecInquiryCancelReq;

/** inquiry cancel confirmation */
/** Description:--*/
/** this message is a confirmation for a InquiryCancelReq message            */
typedef struct _TBtsecInquiryCancelConf
{
	uint16_t        RawError;
	TBtsecCause Cause;             /**< result of request attempt               */
} TBtsecInquiryCancelConf; 
typedef TBtsecInquiryCancelConf  * PBtsecInquiryCancelConf;

/** inquiry request confirmation                                             */
/** Description:-*/
/** this msg is a confirmation of a InquiryReq msg. If the cause indicates   */
/** success the inquiry is finalized successfully                            */
typedef struct _TBtsecInquiryConf
{
	TBtsecCause Cause;              /**< result of inquiry attempt              */
	uint16_t        RawError;           /**< error code                             */
} TBtsecInquiryConf; 
typedef TBtsecInquiryConf  * PBtsecInquiryConf;

/** inquiry started indication                                               */
/** Description:--*/
/** indicates that a inquiry is started                                      */
/**                                                                          */
/** Enable register-flag BTSEC_FLAG_INQUIRY_INDICATION to get this message   */
typedef struct _TBtsecInquiryStartInd
{
	uint16_t  Timeout;                  /**< duration of inquiry                    */
	uint32_t ClassMask;                /**< mask for classcode                     */
	uint32_t ClassValue;               /**< reference value for classcode          */
	uint8_t  idxIAC;                   /**< Inquiry Access Code index              */
} TBtsecInquiryStartInd; 
typedef TBtsecInquiryStartInd  * PBtsecInquiryStartInd;

/** inquiry stoped indication  */
/** Description:-*/
/** indicates that a inquiry ended  */
/**                                                                          */
/** Enable register-flag BTSEC_FLAG_INQUIRY_INDICATION to get this message   */
typedef struct _TBtsecInquiryStopInd
{
	TBtsecCause Cause;              /**< cause for end of inquiry               */
} TBtsecInquiryStopInd; 
typedef TBtsecInquiryStopInd  * PBtsecInquiryStopInd;


/** device and service discovery handling */
/** device name discovery request  */
/** Description:*/
/** this msg can be used do start a device name discovery for a remote device*/
typedef struct _TBtsecNameDiscoveryReq
{
	TBdAddr Bd;       /**< bluetooth device address of remode device            */
	BOOL    holdLink; /**< if set to true:                                      */
          /**< blue secure will establish a (SDP) connection to the device.   */
          /**< After connection is established, it will start a name request  */
          /**< and disconnects the SDP link wihout disconnection on ACL level */
          /**< (this will be done with 2 sec. delay See BlueFace+ docu for    */
          /**< more information) this feature can be used for a fast          */
          /**< connection setup after name discovery                          */

                    /**< if set to false:                                     */
                    /**< the name request will be done on HCI without a 'real'*/
                    /**< ACL connection                                       */
} TBtsecNameDiscoveryReq; 
typedef TBtsecNameDiscoveryReq  * PBtsecNameDiscoveryReq;

/** device name discovery confirmation */
/** Description:---*/
/** this msg is the confirmation for a NameDiscoveryReq. If cause indicates  */
/** success, the requested device name will be included in this msg          */
typedef struct _TBtsecNameDiscoveryConf
{
	TBtsecCause Cause;                           /**< result of nameReq attempt */
	uint16_t        RawError;                        /**< error code                */
	TBdAddr     Bd;                              /**< address of remote device  */
	char        Name[BTSEC_DEVICE_NAME_LENGTH+1];/**< device name               */
} TBtsecNameDiscoveryConf; 
typedef TBtsecNameDiscoveryConf  * PBtsecNameDiscoveryConf;

/** device name discovery cancel request */
/** Description:--*/
/** this msg can be used to cancel a pending name discovery request          */
typedef struct _TBtsecNameDiscoveryCancelReq
{
	TBdAddr     Bd;                              /**< address of remote device  */
} TBtsecNameDiscoveryCancelReq; 
typedef TBtsecNameDiscoveryCancelReq  * PBtsecNameDiscoveryCancelReq;

/** device name discovery cancel confirmation */
/** Description:--*/
/** this msg is a confirmation to a NameDiscoveryCancelReq message           */
typedef struct _TBtsecNameDiscoveryCancelConf
{
	TBtsecCause Cause;                           /**< result of cancel attempt  */
	uint16_t        RawError;                        /**< error code                */
	TBdAddr     Bd;                              /**< address of remote device  */
} TBtsecNameDiscoveryCancelConf; 
typedef TBtsecNameDiscoveryCancelConf  * PBtsecNameDiscoveryCancelConf;

/** TBtsecServiceDiscoveryReqType */
typedef enum _TBtsecServiceDiscoveryReqType
{
   automatic        = 0,         /**< now same as 'l2cLevelServices'          */
                                 /**< History:                                */
                                 /**< was 'public browse group' request, if   */
                                 /**< nothing found request all services in   */
                                 /**< blue secure UUID list (grep UuidList[]) */

   serviceListOnly  = 1,         /**< request all services in btsec UUID list */

   thisServiceOnly  = 2,         /**< request one specific service using SDP  */
                                 /**< code SDP_ServiceSearchAttributeRequest  */
   thisServiceOnlyNoAttr = 3,    /**< request one specific service using SDP  */
                                 /**< code SDP_ServiceSearchRequest           */
   thisATTServiceOnlyNoAttr = 4, /**< same as thisServiceOnlyNoAttr + ATT UUID */
   l2cLevelServices = 5          /**< request all services that have the L2C  */
                                 /**< UUID included... that means all :-)     */
} TBtsecServiceDiscoveryReqType; /**< defines types of service discoveries    */
typedef TBtsecServiceDiscoveryReqType  * PBtsecServiceDiscoveryReqType;

/** service discovery request  */
/** Description:*/
/** this msg can be used to start a service discovery on a remote device     */
typedef struct _TBtsecServiceDiscoveryReq
{
	TBdAddr                       Bd;            /**< address of remote device  */
	TBtsecServiceDiscoveryReqType reqType;       /**< request type              */
	TBtsecUUID                    thisUUID;      /**< UUID for 'thisServiceOnly'*/

	uint32_t                         attributRange; /**< attribut range to be used */
                                /**< to collect service information. if set to*/
                                /**< zero ==> request all attributes          */

	BOOL                          holdLink;      /**< ACL link stays active for */
                                /**< about 3 seconds after SDP disconnect     */
                                /**< this feature can be used for a fast      */
                                /**< connection setup after service discovery */
} TBtsecServiceDiscoveryReq; 
typedef TBtsecServiceDiscoveryReq  * PBtsecServiceDiscoveryReq;

/** service discovery confirmation */
/** Description:*/
/** this msg is the confirmation of a ServiceDiscoveryReq msg. If cause      */
/** indicates success the serched services are indicated by ServiceChangeInd */
/** messages                                                                 */
typedef struct _TBtsecServiceDiscoveryConf
{
	TBtsecCause Cause;            /**< result of service search attempt         */
	uint16_t        RawError;         /**< BlueFace+ raw error                      */
	TBdAddr     Bd;               /**< address of remote device                 */
} TBtsecServiceDiscoveryConf; 
typedef TBtsecServiceDiscoveryConf  * PBtsecServiceDiscoveryConf;


/** service discovery cancel request */
/** Description:--*/
/** this msg can be used to cancel a ongoing service discovery               */
typedef struct _TBtsecServiceDiscoveryCancelReq
{
	TBdAddr Bd;                      /**< device address of remote device       */
} TBtsecServiceDiscoveryCancelReq; 
typedef TBtsecServiceDiscoveryCancelReq  * PBtsecServiceDiscoveryCancelReq;

/** service discovery cancel confirmation */
/** Description:*/
/** this msg is the confirmation for a ServiceDiscoveryCancelReq. If Cause   */
/** indicates success the service discovery is canceled                      */
typedef struct _TBtsecServiceDiscoveryCancelConf
{
	TBtsecCause Cause;    /**< result of cancel attempt                         */
	uint16_t        RawError; /**< error code                                       */
	TBdAddr     Bd;       /**< device address of remote device                  */
} TBtsecServiceDiscoveryCancelConf; 
typedef TBtsecServiceDiscoveryCancelConf  * PBtsecServiceDiscoveryCancelConf;


/** service attribute handling  */
/** service attribute request */
/** Description:*/
/** this message can be used to request specific attributes from services of */
/** a remote device                                                          */
typedef struct _TBtsecServiceAttributeReq
{
	TBdAddr Bd;                 /**< address of remote device                   */
	uint32_t   ServiceHandle;      /**< service handle for service of remote device*/
                              /**< (comes with 'ServiceChangeInd' msg)        */

	uint16_t    AttributeRangeFrom; /**< requested attribute range starts at?       */

	uint16_t    AttributeRangeTo;   /**< requested attribute range ends at?         */
                              /**< for more information:                      */
                              /**< grep in SDP Documentation                  */
                              /**< 'ServiceSearchAttributeTransaction'        */

	BOOL    holdLink;           /**< ACL link stays active for about 3 seconds  */
          /**< after SDP disconnect this feature can be used for a fast       */
          /**< connection setup after service discovery                       */
} TBtsecServiceAttributeReq; 
typedef TBtsecServiceAttributeReq  * PBtsecServiceAttributeReq;

/** Service attribute extended indication */
/** Description:--*/
/** this msg indicates that attributes are found for a specific service of a */
/** remote device                                                            */
/**                                                                          */
/** This msg MUST be answered with a 'ServiceAttributeExtResp' msg           */
typedef struct _TBtsecServiceAttributeExtInd
{
	TBdAddr Bd;                 /**< device address of remote device            */
	uint32_t   ServiceHandle;      /**< handle of remote service                   */
	uint16_t    AttributeRangeFrom; /**< requested attribute range starts here      */
	uint16_t    AttributeRangeTo;   /**< requested attribute range stops here       */
	uint16_t    DataLength;         /**< length of found service attrubute field    */
	uint8_t    buf[1];             /**< variable data size                         */
} TBtsecServiceAttributeExtInd; 
typedef TBtsecServiceAttributeExtInd * PBtsecServiceAttributeExtInd;

/** service attribute extended response */
/** Description:--*/
/** this msg is the necessary response to a ServiceAttributeExtInd msg       */
typedef struct _TBtsecServiceAttributeExtResp
{
	TBdAddr Bd;                 /**< device address of remote device            */
	uint32_t   ServiceHandle;      /**< handle of remode service                   */
} TBtsecServiceAttributeExtResp; 
typedef TBtsecServiceAttributeExtResp * PBtsecServiceAttributeExtResp;

/** service attribute confirmation  */
/** Description:--*/
/** This msg is the confirmation of a ServiceAttributeReq msg. If Cause      */
/** indicates success the serched attribures are indicated by                */
/** ServiceAttributeExtInd messages                                          */
typedef struct _TBtsecServiceAttributeConf
{
	TBdAddr     Bd;                 /**< device address of remote device        */
	TBtsecCause Cause;              /**< result of seach attribute attempt      */
	uint16_t        RawError;           /**< error code                             */
} TBtsecServiceAttributeConf; 
typedef TBtsecServiceAttributeConf  * PBtsecServiceAttributeConf;

/** service attribute cancel request */
/** Description:--*/
/** this message can be used to cancel a pending ServiceAttributeReq msg     */
typedef struct _TBtsecServiceAttributeCancelReq
{
	TBdAddr Bd;                 /**< address of remote device                   */
} TBtsecServiceAttributeCancelReq; 
typedef TBtsecServiceAttributeCancelReq  * PBtsecServiceAttributeCancelReq;

/**  service attribute cancel confirmation */
/** Description:--*/
/** this message is the confirmation for a ServiceAttributeCancelReq message */
typedef struct _TBtsecServiceAttributeCancelConf
{
	TBtsecCause Cause;          /**< result of cancel attribute request attempt */
	uint16_t        RawError;       /**< error code                                 */
	TBdAddr     Bd;             /**< address of remote device                   */
} TBtsecServiceAttributeCancelConf; 
typedef TBtsecServiceAttributeCancelConf  * PBtsecServiceAttributeCancelConf;

/** security parameter handling */
/** TBtsecSecurityPin */
typedef struct _TBtsecSecurityPin
{
	uint8_t PinString[LINK_KEY_SIZE+1]; /**< contains binary PIN or linkkey        */
	uint16_t PinLength;                  /**< length of PIN                         */
} TBtsecSecurityPin;               /**< PIN definition                        */
typedef TBtsecSecurityPin * PBtsecSecurityPin;
typedef TBtsecSecurityPin * LPBtsecSecurityPin;

/** TBtsecSecurityMode */
typedef enum _TBtsecSecurityMode
{
	BtsecNoSecurity = 1, /**< no security on blue secure level, app will handle */
                       /**< all security procedures                           */

	BtSecSecMode2,       /**< blue secure handles all security procedures       */

	BtSecSecMode3,       /**< link manager will initiate security (sec mode 3)  */
                       /**< blue secure will handle all security procedures   */
	BtSecSecMode4        /**< Secure Simple Pairing                             */
} TBtsecSecurityMode;  /**< sets layer of security handler                    */
typedef TBtsecSecurityMode* PBtsecSecurityMode;

/** TBtsecSecuritySubMode */
typedef enum _TBtsecSecuritySubMode
{
	BtsecSecSubBondable,      /**< we accept authentication/pairing but do not  */
                            /**< initiate any of this proccesses, stay passive*/

	BtsecSecSubActivSecurity, /**< we initiate authentication, if link key is   */
                            /**< missing we start the pairing proccess        */

	BtsecSecSubBondAllways,   /**< we allways start pairing proccess, no matter */
                            /**< if we have a link key or not                 */

	BtsecSecSubNonBondable,   /**< we only accept authentication with link key  */
                            /**< but reject pairing with PIN                  */

	BtsecSecSubRebondAllways, /**< we only accept connections bonded devices    */
                            /**< (we have a link key) but allways re-initiate */
                            /**< a new paring proccess                        */

	BtsecSecSubNonConnectable,/**< we do not accept any connections             */

	BtsecSecSubRefusePairing, /**< we do not accept pairing attempts from the   */
							/**< remote device but accept authentication      */

	BtsecSecSubRefuseSecurity /**< we do not accept authentication/pairing      */
                            /**< attempts from the remote device              */

} TBtsecSecuritySubMode;    /**< security settings for connect attempts       */
typedef TBtsecSecuritySubMode* PBtsecSecuritySubMode;

typedef enum _TBtsecSecurityStoreMode
{
	BtsecSecStoreNoStore  = 0, /**< do not store bonding results                */
	BtsecSecStoreNVStore  = 1, /**< store bonding results permanently in NV RAM */
	BtsecSecStoreRAMStore = 2, /**< store bonding results in RAM only           */
	BtsecSecStoreExternal = 3  /**< store bonding results externally            */
} TBtsecSecurityStoreMode;
typedef TBtsecSecurityStoreMode * PBtsecSecurityStoreMode;

/** TBtsecSecurityBTMode */
typedef enum _TBtsecSecurityBTMode
{   
	/** definitions are HCI compliant, do not change */
	BtsecSecBTMode21SupportDisabled = 0x00, /**< no support for SSP features    */
	BtsecSecBTMode21SupportEnabled  = 0x01, /**< support for SSP features       */
	BtsecSecBTMode21DebugSupport    = 0x02  /**< support for SSP debug mode     */
} TBtsecSecurityBTMode;
typedef TBtsecSecurityBTMode * PBtsecSecurityBTMode;

/** TBtsecSecurityAuthRequirements */
typedef enum _TBtsecSecurityAuthRequirements
{ 
	/** definitions are HCI compliant, do not change */
	BtsecSecAuthReqNoMITMNoStore   = 0x00,
	BtsecSecAuthReqMITMNoStore     = 0x01,
	BtsecSecAuthReqnoMITMDedicBond = 0x02,
	BtsecSecAuthReqMITMDedicBond   = 0x03,
	BtsecSecAuthReqNoMITMGenBond   = 0x04,
	BtsecSecAuthReqMITMGenBond     = 0x05
} TBtsecSecurityAuthRequirements;
typedef TBtsecSecurityAuthRequirements* PBtsecSecurityAuthRequirements;

/** TBtsecSecurityIOCapabilities */
typedef enum _TBtsecSecurityIOCapabilities
{ 
	/** definitions are HCI compliant, do not change */
	BtsecSecIODisplayOnly    = 0x00,   /**< only a display present, no keyboard */
	BtsecSecIODisplayYesNo   = 0x01,   /**< Display and yes/no buttons present  */
	BtsecSecIOKeyboardOnly   = 0x02,   /**< only a Keyboard present, no Display */
	BtsecSecIONoIOCapability = 0x03,   /**< no IO capabilities                  */
	BtsecSecIOKeyboardDisplay= 0x04    /**< Keyboard AND Display (BLE only)     */
} TBtsecSecurityIOCapabilities;
typedef TBtsecSecurityIOCapabilities* PBtsecSecurityIOCapabilities;

/** bonded device database handling  */
#define BTSEC_BOND_TABLE_CHANGE_NAME      0x0001
#define BTSEC_BOND_TABLE_CHANGE_LINKKEY   0x0002
#define BTSEC_BOND_TABLE_CHANGE_CLASSCODE 0x0004
#define BTSEC_BOND_TABLE_CHANGE_USERDATA  0x0008

/** TBtsecSecurityUserData */
typedef struct _TBtsecSecurityUserData
{
  TBdAddr Bd;                                 /* identifier for entry       */
  uint32_t   UserData;                           /* new user data to be set    */
} TBtsecSecurityUserData;
typedef TBtsecSecurityUserData * LPBtsecSecurityUserData;

/** TBtsecSecurityBondTableEntry */
typedef struct _TBtsecSecurityBondTableEntry
{
   char     Name[BTSEC_DEVICE_NAME_LENGTH+1]; /**< device name                */
   TLinkKey Key;                              /**< link key                   */
   uint32_t    ClassCode;                        /**< class of device            */
   uint32_t    UserData;                         /**< app data, ignored by       */
   uint8_t     keyType;                          /**< BLUEFACE_KEYTYPE_*         */
                                              /**< blue secure                */
} TBtsecSecurityBondTableEntry;               /* bonded device data         */
typedef TBtsecSecurityBondTableEntry *    PBtsecSecurityBondTableEntry;
typedef TBtsecSecurityBondTableEntry * LPBtsecSecurityBondTableEntry;

/** TBtsecCommandSecurityBondTableData */
typedef struct _TBtsecSecurityBondTableData
{
  TBdAddr                      Bd;            /* identifier for entry       */
  TBtsecSecurityBondTableEntry entryData;     /* device data for entry      */
} TBtsecSecurityBondTableData;                /* entry of security database */
typedef TBtsecSecurityBondTableData  * LPBtsecSecurityBondTableData;

/** trusted device database handling                                         */
/** TBtsecSecurityTrustedDeviceTableEntry */
typedef struct _TBtsecSecurityTrustedDeviceTableEntry
{
	TBtsecSecurityMode             SecModeIncomming;
	TBtsecSecuritySubMode          SecSubModeIncomming;
	TBtsecSecurityMode             SecModeOutgoing;
	TBtsecSecuritySubMode          SecSubModeOutgoing;
	BOOL                           UseEncryption;
	TBtsecSecurityPin              SysPin;
} TBtsecSecurityTrustedDeviceTableEntry; /**< def. security settings of device*/
typedef TBtsecSecurityTrustedDeviceTableEntry * LPBtsecSecurityTrustedDeviceTableEntry;

/** BTSEC_TRUSTED_DEVICE_VALID_ */
#define BTSEC_TRUSTED_DEVICE_VALID_USE_SYS_PIN_FOR_INCOMMING 0x0001
#define BTSEC_TRUSTED_DEVICE_VALID_USE_SYS_PIN_FOR_OUTGOING  0x0002
#define BTSEC_TRUSTED_DEVICE_VALID_SEC_MODE_INCOMMING        0x0004
#define BTSEC_TRUSTED_DEVICE_VALID_SEC_MODE_OUTGOING         0x0008
#define BTSEC_TRUSTED_DEVICE_VALID_SUB_SEC_MODE_INCOMMING    0x0010
#define BTSEC_TRUSTED_DEVICE_VALID_SUB_SEC_MODE_OUTGOING     0x0020
#define BTSEC_TRUSTED_DEVICE_VALID_USE_ENCRYPTION            0x0040
#define BTSEC_TRUSTED_DEVICE_VALID_SYS_PIN                   0x0080

/** TBtsecSecurityTrustedDeviceTableData */
typedef struct _TBtsecSecurityTrustedDeviceTableData
{
	TBdAddr                               Bd;        /**< device address of     */
                                                   /**< trusted device        */

	uint16_t                                  validMask; /**< defines witch of the  */
    /**< parameter of the trusted device settings are valid. for all other      */
    /**< settings the normal blue secure settings will be used.                 */
    /**< All 'BTSEC_TRUSTED_DEVICE_VALID_' combinations are allowed             */

	TBtsecSecurityTrustedDeviceTableEntry entryData; /**< device settings       */
} TBtsecSecurityTrustedDeviceTableData;            /**<entry in trusted device*/
                                                   /**<database               */
typedef TBtsecSecurityTrustedDeviceTableData * LPBtsecSecurityTrustedDeviceTableData;


typedef struct _TBtsecSecurityPairableModeSetReq
{
	BOOL                           EnablePairableMode;
	TBtsecSecurityBTMode           BluetoothMode;
	TBtsecSecurityAuthRequirements AuthRequirements;
	TBtsecSecurityIOCapabilities   IOCapabilities;
	BOOL                           remoteOOBDataPresent;
} TBtsecSecurityPairableModeSetReq, * LPBtsecSecurityPairableModeSetReq;

typedef struct _TBtsecSecurityServiceConfigurationSetReq
{
	uint16_t psm;           /**< L2CAP PSM. =0: not used                            */
	uint16_t serverChannel; /**< RFCOMM channel. =0: not used                       */
	BOOL outgoing;      /**< TRUE: define requirement for outgoing connection   */
                      /**< or for incoming connection (FALSE)                 */
	BOOL active;        /**< TRUE: entry is active. FALSE: delete entry         */
	uint16_t uuid;          /**< UUID used in corresponding AUTHORIZATION_IND       */
	BOOL authenticate;  /**< TRUE: authentication required                      */
	BOOL authorize;     /**< TRUE: link establishment requires explicit user    */
                      /**< level authorization (done before authentication)   */
	BOOL encryption;    /**< TRUE: activate encryption on this link,            */
                      /**<      only possible if authen_setting != none      */
	BOOL mitm;          /**< TRUE: mitm requiered, FALSE: no MITM required      */
} TBtsecSecurityServiceConfigurationSetReq, * LPBtsecSecurityServiceConfigurationSetReq;

typedef struct _TBtsecSecurityUserAuthorizationReqInd
{
	uint8_t                   remote_BD[BD_ADDR_SIZE];
} TBtsecSecurityUserAuthorizationReqInd,  * LPBtsecSecurityUserAuthorizationReqInd;

typedef struct _TBtsecSecurityUserAuthorizationReqConf
{
	uint8_t                   remote_BD[BD_ADDR_SIZE];
	uint8_t                   Status;
} TBtsecSecurityUserAuthorizationReqConf,  * LPBtsecSecurityUserAuthorizationReqConf;

typedef struct _TBtsecSecurityUserKeypressNotificationReq
{
	uint8_t                    remote_BD[BD_ADDR_SIZE];
	uint8_t                    eventType;
} TBtsecSecurityUserKeypressNotificationReq, * LPBtsecSecurityUserKeypressNotificationReq;

typedef struct _TBtsecSecurityRemoteOOBDataReqConf
{
	uint8_t                   remote_BD[BD_ADDR_SIZE];
	uint8_t                   C[C_R_SIZE];
	uint8_t                   R[C_R_SIZE];
	uint8_t                   Status;
} TBtsecSecurityRemoteOOBDataReqConf,  * LPBtsecSecurityRemoteOOBDataReqConf;

typedef struct _TBtsecSecurityLocalOOBDataRsp
{
	uint8_t                   C[C_R_SIZE];
	uint8_t                   R[C_R_SIZE];
	uint8_t                   Status;
} TBtsecSecurityLocalOOBDataRsp,  * LPBtsecSecurityLocalOOBDataRsp;

typedef struct _TBtsecSecurityUserConfirmationReqInd
{
	uint8_t                   remote_BD[BD_ADDR_SIZE];
	uint32_t                  displayValue;
} TBtsecSecurityUserConfirmationReqInd,  * LPBtsecSecurityUserConfirmationReqInd;

typedef struct _TBtsecSecurityUserConfirmationReqConf
{
	uint8_t                   remote_BD[BD_ADDR_SIZE];
	uint8_t                   Status;
} TBtsecSecurityUserConfirmationReqConf,  * LPBtsecSecurityUserConfirmationReqConf;

typedef struct _TBtsecSecurityUserPasskeyReqInd
{
	uint8_t                    remote_BD[BD_ADDR_SIZE];
} TBtsecSecurityUserPasskeyReqInd,  * LPBtsecSecurityUserPasskeyReqInd;

typedef struct _TBtsecSecurityUserPasskeyReqConf
{
	uint8_t                    remote_BD[BD_ADDR_SIZE];
	uint8_t                    Status;
} TBtsecSecurityUserPasskeyReqConf,  * LPBtsecSecurityUserPasskeyReqConf;

typedef struct _TBtsecSecurityUserPasskeyReqReplyReq
{
	uint8_t                    remote_BD[BD_ADDR_SIZE];
	uint32_t                   passKey;
	uint8_t                    Status;
}TBtsecSecurityUserPasskeyReqReplyReq,  *LPBtsecSecurityUserPasskeyReqReplyReq;

typedef struct _TBtsecSecurityUserPasskeyReqReplyRsp
{
	uint16_t                   Status;
} TBtsecSecurityUserPasskeyReqReplyRsp, * LPBtsecSecurityUserPasskeyReqReplyRsp;

typedef struct _TBtsecSecurityUserPasskeyNotificationInfo
{
	uint8_t                    remote_BD[BD_ADDR_SIZE];
	uint32_t                   displayValue;
} TBtsecSecurityUserPasskeyNotificationInfo, * LPBtsecSecurityUserPasskeyNotificationInfo;

typedef struct _TBtsecSecurityKeypressNotificationRsp
{
	uint16_t                   Status;
} TBtsecSecurityKeypressNotificationRsp, * LPBtsecSecurityKeypressNotificationRsp;

typedef struct _TBtsecSecurityKeypressNotificationInfo
{
	uint8_t                    remote_BD[BD_ADDR_SIZE];
	uint8_t                    eventType;
} TBtsecSecurityKeypressNotificationInfo, * LPBtsecSecurityKeypressNotificationInfo;

typedef struct _TBtsecSecurityRemoteOOBDataReqInd
{
	uint8_t                    remote_BD[BD_ADDR_SIZE];
} TBtsecSecurityRemoteOOBDataReqInd, * LPBtsecSecurityRemoteOOBDataReqInd;

/** security control commands */
/** TBtsecSecurityChangePin */
typedef struct _TBtsecSecurityChangePin
{
	BOOL  CheckPin;            /**< if true, btsec checks 'old PIN' with sys PIN*/
                             /**< can be used to verify user knows 'old PIN'  */

	TBtsecSecurityPin OldPin;  /**< for check, this PIN must be identical with  */
                             /**< system PEN                                  */

	TBtsecSecurityPin NewPin;  /**< PIN to be used as system PIN                */

} TBtsecSecurityChangePin;   /**< tata to chage system PIN                    */
typedef TBtsecSecurityChangePin* PBtsecSecurityChangePin;

/** TBtsecSecurityParameterId */
typedef enum _TBtsecSecurityParameterId
{
	BtsecSecurityParameterUndefined                    = 0x00,
	BtsecSecurityParameterIdSysPin                     = 0x01,
	BtsecSecurityParameterIdUseSysPinIn                = 0x02,
	BtsecSecurityParameterIdUseSysPinOut               = 0x05,
	BtsecSecurityParameterIdstoreNewBondings           = 0x09,
	BtsecSecurityParameterIdBondTableGetFirst          = 0x0A,
	BtsecSecurityParameterIdBondTableGetNext           = 0x0B,
	BtsecSecurityParameterIdBondTableGetThis           = 0x0C,
	BtsecSecurityParameterIdBondTableAdd               = 0x0D,
	BtsecSecurityParameterIdBondTableDelete            = 0X0E,
	BtsecSecurityParameterIdBondTableChange            = 0x0F,
	BtsecSecurityParameterIdBondTableUserDataSet       = 0x10,
	BtsecSecurityParameterIdTrustedDeviceTableGetFirst = 0x11,
	BtsecSecurityParameterIdTrustedDeviceTableGetNext  = 0x12,
	BtsecSecurityParameterIdTrustedDeviceTableGetThis  = 0x13,
	BtsecSecurityParameterIdTrustedDeviceTableAdd      = 0x14,
	BtsecSecurityParameterIdTrustedDeviceTableDelete   = 0X15,
	BtsecSecurityParameterIdTrustedDeviceTableChange   = 0x16,
	BtsecSecurityParameterIdPairableModeSet            = 0x17,
	BtsecSecurityParameterIdForServiceSet              = 0x18,
	BtsecSecurityParameterIdPairableMode               = 0x19,
	BtsecSecurityParameterIdBTMode                     = 0x1A,
	BtsecSecurityParameterIdAuthRequirements           = 0x1B,
	BtsecSecurityParameterIdIOCapabilities             = 0x1C,
	BtsecSecurityParameterIdInitBondTableSize          = 0x1D,
	BtsecSecurityParameterIdBondTableSize              = 0x1E,
	BtsecSecurityParameterIdBondMode                   = 0x1F,
 	BtsecSecurityParameterIdLast                       = 0xFF
} TBtsecSecurityParameterId; /**< identifier for parameter/action to be taken  */
typedef TBtsecSecurityParameterId* PBtsecSecurityParameterId;

/** TBtsecSecurityParameterValue */
typedef  union _TBtsecSecurityParameterValue
{
	TBtsecSecurityPin           SysPin;                /**< for indication only! */
	TBtsecSecurityChangePin     ChangePin;             /**< for change req only! */
	TBtsecSecurityMode          SecModeIncomming;
	TBtsecSecuritySubMode       SecSubModeIncomming;
	TBtsecSecurityMode          SecModeOutgoing;
	TBtsecSecuritySubMode       SecSubModeOutgoing;
	BOOL                        UseEncryption;
	TBtsecSecurityStoreMode     StoreNewBondings;
	TBtsecSecurityStoreMode     BondMode;
	uint8_t                        BondTableSize;
	TBtsecSecurityBondTableData BondTableEntry;
	TBtsecSecurityUserData      SetUserData;
	TBtsecSecurityPairableModeSetReq         PairableMode;
	TBtsecSecurityServiceConfigurationSetReq ServiceConfiguration;
	BOOL                                     EnablePairableMode;
	TBtsecSecurityBTMode                     BluetoothMode;
	TBtsecSecurityAuthRequirements           AuthRequirements;
	TBtsecSecurityIOCapabilities             IOCapabilities;
} TBtsecSecurityParameterValue; /**< holds the information needed for action  */
typedef TBtsecSecurityParameterValue * PBtsecSecurityParameterValue;

/** security parameter/database set request  */
/** Description:*/
/** with this msg a parameter or database content change can be requested    */
typedef struct _TBtsecSecuritySetReq
{
	TBtsecSecurityParameterId    Id;    /**< identifier for requested operation */
	TBtsecSecurityParameterValue Value; /**< data for requested operation       */
} TBtsecSecuritySetReq; 
typedef TBtsecSecuritySetReq  * PBtsecSecuritySetReq;

/** security parameter/database set confirmation */
/** Description:*/
/** this msg is a confirmation for a 'SecuritySetReq' msg. if the cause      */
/** indicates success, the requested operation is done                       */
typedef struct _TBtsecSecuritySetConf
{
	TBtsecCause               Cause;    /**< result of operation attempt        */
	uint16_t                      RawError; /**< error code                         */
	TBtsecSecurityParameterId Id;       /**< identifier for requested operation */
	TBtsecSecurityParameterValue Value; /**< data for requested operation       */
} TBtsecSecuritySetConf; 
typedef TBtsecSecuritySetConf  * PBtsecSecuritySetConf;

/** security parameter/database get request  */
/** Description:*/
/** with this msg a parameter or database content can be requested           */
typedef struct _TBtsecSecurityGetReq
{
	TBtsecSecurityParameterId    Id;    /**< identifier for requested content   */
	TBdAddr                      Bd;    /**< identifier for database element    */
                                      /**<(not needed for parameter operation)*/
} TBtsecSecurityGetReq; 
typedef TBtsecSecurityGetReq  * PBtsecSecurityGetReq;

/** security parameter/database get confirmation  */
/** Description:-*/
/** this msg is a confirmation for a 'SecurityGetReq' msg. If the cause      */
/** indicates success the requested content is included in this msg          */
typedef struct _TBtsecSecurityGetConf
{
	TBtsecCause                  Cause;    /**< result of request attempt       */
	uint16_t                         RawError; /**< error code                      */
	TBtsecSecurityParameterId    Id;       /**< identifier for requested content*/
	TBtsecSecurityParameterValue Value;    /**< requested content               */
} TBtsecSecurityGetConf;
typedef TBtsecSecurityGetConf  * PBtsecSecurityGetConf;

/** security parameter/database change indication */
/** Description:-*/
/** this msg indicates that a parameter or the database contend has changed  */
/** for some reason.                                                         */
/**                                                                          */
/** Enable register-flag BTSEC_FLAG_SECURITY_INDICATION to get this message  */
typedef struct _TBtsecSecurityChangeInd
{
	TBtsecCause                  Cause;      /**< result of request attempt     */
	uint16_t                         RawError;   /**< error code                    */
	uint16_t                         changeMask; /**< bitmask to ident. changes     */
	TBtsecSecurityParameterId    Id;         /**< identifier for changed content*/
	TBtsecSecurityParameterValue Value;      /**< changed content               */
} TBtsecSecurityChangeInd; 
typedef TBtsecSecurityChangeInd  * PBtsecSecurityChangeInd;

/** active bonding commands */
/** TBtsecSecurityPinSelect */
typedef enum _TBtsecSecurityPinSelect
{
	useThisPin          = 0, /**< use PIN included in this msg                  */
	useSysPin           = 1, /**< use system PIN                                */
	useSysPinInSetting  = 2, /**< use PIN selected in 'sys pin in' setting      */
	useSysPinOutSetting = 3, /**< use PIN selected in 'sys pin out' setting     */
	askUserIfRequired   = 4  /**< BOND Req for SSP */
} TBtsecSecurityPinSelect; /**< defines with PIN shold be used for bonding    */
typedef TBtsecSecurityPinSelect * PBtsecSecurityPinSelect;

/** security bond / authentication request */
/** Description:*/
/** requests bonding/authentication for a remote device or an existing link  */
typedef struct _TBtsecSecurityBondReq
{
	TBdAddr                 Bd;            /**< bluetooth address of remote     */
                                         /**< device (needed in anny case)    */

	TBtsecSecuritySubMode   sysSecSubMode; /**< bonding / authentication level  */
                                         /**< (PIN or link key)               */

	TBtsecSecurityPinSelect pinSelect;     /**< if PIN is needed, witch one?    */

	TBtsecSecurityPin       bondPin;       /**< temporary PIN, used in the case */
                                         /**< that 'pinSelect' is set to      */
                                         /**< 'useThisPin'                    */

	BLINKHANDLE             bLinkHandle;   /**< link handle of exsisting link   */
                                         /**< if no link exsist, set to NULL  */
} TBtsecSecurityBondReq; 
typedef TBtsecSecurityBondReq  * PBtsecSecurityBondReq;

/** security bond / authentication confirmation */
/** Description:----*/
/** confirmation for 'SecurityBondReq' msg. If cause indicates success       */
/** bonding / authentication was successfull.                                */
typedef struct _TBtsecSecurityBondConf
{
	TBtsecCause Cause;       /**< result of bond / authentication attempt       */
	uint16_t        RawError;    /**< raw blueface error                            */
	TBdAddr     Bd;          /**< bluetooth address of remote device            */
	BLINKHANDLE bLinkHandle; /**< link handle of exsisting link                 */
} TBtsecSecurityBondConf; 
typedef TBtsecSecurityBondConf  * PBtsecSecurityBondConf;


/** user PIN handling  */
/** user PIN request extended indication  */
/** Description:-*/
/** indication that a user PIN is needed. Will be send to app if a PIN is    */
/** needed vor authentication, but the parameter 'UseSysPinForIncomming' or  */
/** 'UseSysPinForOutgoing' are set to false (==> request user PIN)           */
/**                                                                          */
/** Enable register-flag BTSEC_FLAG_SECURITY_USER_PIN_INDICATION to get this */
/** message                                                                  */
typedef struct _TBtsecSecurityUserPinReqExtInd
{
	TBdAddr Bd;                     /**< bluetooth device addr. from remote dev.*/
	uint16_t    RequestType;            /**< blueface request type. Might be        */
                                  /**< BLUEFACE_REMOTEPIN or BLUEFACE_LOCALPIN*/
	THandle hLink;                  /**<link handle for link to be authenticated*/

} TBtsecSecurityUserPinReqExtInd; 
typedef TBtsecSecurityUserPinReqExtInd  * LPBtsecSecurityUserPinReqExtInd;

/** user PIN request extended response */
/** Description:--*/
/** this is the response of an app that got an 'UserPinReqExtInd' msg.       */
/** it might contain a PIN for authentication or an error code               */
typedef struct _TBtsecSecurityUserPinReqExtResp
{
	TBtsecCause       Cause;        /**< if no success, authentication will be  */
                                  /**< aborted                                */
	TBdAddr           Bd;           /**< bluetooth device addr. of remote device*/
	uint16_t              RequestType;  /**< same as in 'UserPinReqExtInd'          */
	uint8_t              keyType;      /**< BLUEFACE_KEYTYPE_*                     */
	TBtsecSecurityPin UserPin;      /**< user PIN or linkKey and keyType        */
} TBtsecSecurityUserPinReqExtResp; 
typedef TBtsecSecurityUserPinReqExtResp  * LPBtsecSecurityUserPinReqExtResp;

/** user PIN cancel indication */
/** Description:--*/
/** indicates that a pending user PIN is no longer needed                    */
/**                                                                          */
/** Enable register-flag BTSEC_FLAG_SECURITY_USER_PIN_INDICATION to get this */
/** message                                                                  */
typedef struct _TBtsecSecurityUserPinCancelInd
{
	TBdAddr Bd;                     /**< bluetooth device addr. of remote dev.  */
	THandle hLink;                  /**< link handle for link with              */
                                  /**< authentication to be canceled          */
} TBtsecSecurityUserPinCancelInd;
typedef TBtsecSecurityUserPinCancelInd  * LPBtsecSecurityUserPinCancelInd;


/** security status handling  */
/** TBtsecSecurityStatusIndId */
typedef enum _TBtsecSecurityStatusIndId
{
	BtsecSecurityStatusIdAuthRequested, /**< we requested authentication start  */

	BtsecSecurityStatusIdAuthStarted,   /**< authentication is started          */
                                      /**< (we got a linkkey / PIN request)   */

	BtsecSecurityStatusIdAuthDone,      /**< authentication is done             */

	BtsecSecurityStatusIdEncryption     /**< encryp. status of ACL link changed */

} TBtsecSecurityStatusIndId;          /**< defines types of security events   */
typedef TBtsecSecurityStatusIndId  * PBtsecSecurityStatusIndId;

/** TBtsecSecurityStatusIndAuthLevel */
typedef enum _TBtsecSecurityStatusIndAuthLevel
{
	AuthLevelLinkKey = 0,             /**< auth. on link key level              */
	AuthLevelSysPin,                  /**< auth. system PIN level               */
	AuthLevelUserPin,                 /**< auth. on user PIN level              */
	AuthLevelTrustedDevicePin,        /**< auth. on trusted device PIN level    */
	AuthLevelSecurityManager          /**< auth. handled by security manager    */
} TBtsecSecurityStatusIndAuthLevel; /* defines level for ongoing authenticat*/
typedef TBtsecSecurityStatusIndAuthLevel  * PBtsecSecurityStatusIndAuthLevel;

/** TBtsecSecurityStatusAuthStarted */
typedef struct _TBtsecSecurityStatusAuthStarted
{
	TBtsecSecurityStatusIndAuthLevel AuthLevel;
} TBtsecSecurityStatusAuthStarted; /**< contains data for 'AuthStarted' event */
typedef TBtsecSecurityStatusAuthStarted  * PBtsecSecurityStatusAuthStarted;

/** TBtsecSecurityStatusAuthDone */
typedef struct _TBtsecSecurityStatusAuthDone
{
	TBtsecCause Cause;              /**< result of authentication attempt       */
	uint16_t        RawError;           /**< error code                             */
} TBtsecSecurityStatusAuthDone;   /**< contains data for 'AuthDone' event     */
typedef TBtsecSecurityStatusAuthDone  * PBtsecSecurityStatusAuthDone;

/** TBtsecSecurityStatusEnrcyption */
typedef struct _TBtsecSecurityStatusEnrcyption
{
	TBtsecCause Cause;              /**< result of encrypt attempt              */
	uint16_t        RawError;           /**< error code                             */
	BOOL        State;              /**< TRUE => link is encrypted now          */
} TBtsecSecurityStatusEnrcyption; /**< contains data for 'Encryption' event   */
typedef TBtsecSecurityStatusEnrcyption  * PBtsecSecurityStatusEnrcyption;

/** TBtsecSecurityStatusValue */
typedef union _TBtsecSecurityStatusValue
{
	TBtsecSecurityStatusAuthStarted AuthStarted;
	TBtsecSecurityStatusAuthDone    AuthDone;
	TBtsecSecurityStatusEnrcyption  Encryption;
} TBtsecSecurityStatusValue; /**< contains data for 'SecurityStatusInd' msg   */
typedef TBtsecSecurityStatusValue  * PBtsecSecurityStatusValue;

/** security link status indication */
/** Description:-*/
/** indicates security specific events on an ACL link                        */
/**                                                                          */
/** Enable register-flag BTSEC_FLAG_SECURITY_STATUS_INDICATION to get this   */
/** message                                                                  */
typedef struct _TBtsecSecurityStatusInd
{
	TBtsecSecurityStatusIndId Id;    /**< identifier for event type             */
	TBdAddr                   Bd;    /**< identifier for ACL link               */
	TBtsecSecurityStatusValue Value; /**< event specific information            */
} TBtsecSecurityStatusInd; 
typedef TBtsecSecurityStatusInd  * PBtsecSecurityStatusInd;


/** Btsec Message */
typedef struct _TBtsecMsg
{
	TBtsecCommand       Command;
	uint16_t                Length;
	THandle             Handle;
	union _TBtsecMsgP
	{
    	TBtsecRegisterReq                  RegisterReq;
    	TBtsecRegisterConf                 RegisterConf;
    	TBtsecReleaseReq                   ReleaseReq;
    	TBtsecReleaseConf                  ReleaseConf;
   		TBtsecOwnDeviceSetReq              OwnDeviceSetReq;
    	TBtsecOwnDeviceSetConf             OwnDeviceSetConf;
    	TBtsecOwnDeviceGetReq              OwnDeviceGetReq;
    	TBtsecOwnDeviceGetConf             OwnDeviceGetConf;
    	TBtsecOwnDeviceChangeInd           OwnDeviceChangeInd;
    	TBtsecSecuritySetReq               SecuritySetReq;
    	TBtsecSecuritySetConf              SecuritySetConf;
    	TBtsecSecurityGetReq               SecurityGetReq;
    	TBtsecSecurityGetConf              SecurityGetConf;
    	TBtsecSecurityChangeInd            SecurityChangeInd;
    	TBtsecSecurityBondReq              SecurityBondReq;
    	TBtsecSecurityBondConf             SecurityBondConf;
    	TBtsecSecurityUserPinReqExtInd     SecurityUserPinReqExtInd;
    	TBtsecSecurityUserPinReqExtResp    SecurityUserPinReqExtResp;
    	TBtsecSecurityUserPinCancelInd     SecurityUserPinCancelInd;
    	TBtsecSecurityStatusInd            SecurityStatusInd;
    	TBtsecSecurityUserKeypressNotificationReq   SecurityUserKeypressNotificationReq;
    	TBtsecSecurityUserAuthorizationReqInd       SecurityUserAuthorizationReqInd;
    	TBtsecSecurityUserAuthorizationReqConf      SecurityUserAuthorizationReqConf;
    	TBtsecSecurityRemoteOOBDataReqConf          SecurityRemoteOOBDataReqConf;
    	TBtsecSecurityLocalOOBDataRsp               SecurityLocalOOBDataRsp;
    	TBtsecSecurityUserConfirmationReqInd        SecurityUserConfirmationReqInd;
    	TBtsecSecurityUserConfirmationReqConf       SecurityUserConfirmationReqConf;
    	TBtsecSecurityUserPasskeyReqInd             SecurityUserPasskeyReqInd;
    	TBtsecSecurityUserPasskeyReqConf            SecurityUserPasskeyReqConf;
    	TBtsecSecurityUserPasskeyReqReplyRsp        SecurityUserPasskeyReqReplyRsp;
    	TBtsecSecurityUserPasskeyReqReplyReq        SecurityUserPasskeyReqReplyReq;
    	TBtsecSecurityUserPasskeyNotificationInfo   SecurityUserPasskeyNotificationInfo;
    	TBtsecSecurityKeypressNotificationRsp       SecurityKeypressNotificationRsp;
    	TBtsecSecurityKeypressNotificationInfo      SecurityKeypressNotificationInfo;
    	TBtsecSecurityRemoteOOBDataReqInd           SecurityRemoteOOBDataReqInd;
    	TBtsecInquiryClassMaskSetReq       InquiryClassMaskSetReq;
    	TBtsecInquiryClassMaskSetConf      InquiryClassMaskSetConf;
    	TBtsecInquiryReq                   InquiryReq;
    	TBtsecInquiryConf                  InquiryConf;
    	TBtsecInquiryStartInd              InquiryStartInd;
    	TBtsecInquiryStopInd               InquiryStopInd;
    	TBtsecInquiryCancelReq             InquiryCancelReq;
    	TBtsecInquiryCancelConf            InquiryCancelConf;
    	TBtsecNameDiscoveryReq             NameDiscoveryReq;
    	TBtsecNameDiscoveryConf            NameDiscoveryConf;
    	TBtsecNameDiscoveryCancelReq       NameDiscoveryCancelReq;
    	TBtsecNameDiscoveryCancelConf      NameDiscoveryCancelConf;
    	TBtsecServiceDiscoveryReq          ServiceDiscoveryReq;
    	TBtsecServiceDiscoveryConf         ServiceDiscoveryConf;
    	TBtsecServiceDiscoveryCancelReq    ServiceDiscoveryCancelReq;
    	TBtsecServiceDiscoveryCancelConf   ServiceDiscoveryCancelConf;
    	TBtsecServiceAttributeReq          ServiceAttributeReq;
    	TBtsecServiceAttributeExtInd       ServiceAttributeExtInd;
    	TBtsecServiceAttributeExtResp      ServiceAttributeExtResp;
    	TBtsecServiceAttributeConf         ServiceAttributeConf;
    	TBtsecServiceAttributeCancelReq    ServiceAttributeCancelReq;
    	TBtsecServiceAttributeCancelConf   ServiceAttributeCancelConf;
    	TBtsecDeviceGetReq                 DeviceGetReq;
    	TBtsecDeviceGetConf                DeviceGetConf;
    	TBtsecDeviceSetReq                 DeviceSetReq;
    	TBtsecDeviceSetConf                DeviceSetConf;
    	TBtsecServiceGetReq                ServiceGetReq;
    	TBtsecServiceGetConf               ServiceGetConf;
    	TBtsecServiceSetReq                ServiceSetReq;
    	TBtsecServiceSetConf               ServiceSetConf;
    	TBtsecDeviceChangeInd              DeviceChangeInd;
    	TBtsecServiceChangeInd             ServiceChangeInd;
    	TBtsecProgressInd                  ProgressInd;
    	TBtsecLinkModeSetReqExtInd         LinkModeSetReqExtInd;
    	TBtsecLinkModeSetReqExtResp        LinkModeSetReqExtResp;
    	TBtsecLinkModeSetReq               LinkModeSetReq;
    	TBtsecLinkModeSetConf              LinkModeSetConf;
    	TBtsecLinkModeInd                  LinkModeInd;
    	TBtsecQoSSetReq                    QoSSetReq;
    	TBtsecQoSSetConf                   QoSSetConf;
    	TBtsecQoSInd                       QoSInd;
    	TBtsecGenericConf                  GenericConf;
  	} P;
} TBtsecMsg;
typedef        TBtsecMsg  * PBtsecMsg;


#ifdef __cplusplus
extern "C" 
{
#endif

/** methods */
/** methods provided by BlueSecure */
/** methods provided by target interface */
#ifdef __cplusplus
}
#endif

#endif /**< !defined(__BTSEC_H) */

/** End of file */
