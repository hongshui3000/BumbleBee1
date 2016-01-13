/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file        blueapi_types.h
* @brief      
* @details   
*
* @author   	gordon
* @date      	2015-07-10
* @version	v0.1
*/

#if !defined(__BLUEAPI_TYPES_H)
#define      __BLUEAPI_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include <flags.h>

#if defined (__cplusplus)
extern "C" {
#endif

#if !defined(BLUE_API_DEVICE_NAME_LENGTH)
#define BLUE_API_DEVICE_NAME_LENGTH            40
#endif

#if !defined(BLUE_API_HDP_SERVICE_NAME_LENGTH)
#define BLUE_API_HDP_SERVICE_NAME_LENGTH       24
#endif

#if !defined(BLUE_API_MDEP_NAME_LENGTH)
#define BLUE_API_MDEP_NAME_LENGTH              24    /**< max. length in message   */
#define BLUE_API_MDEP_NAME_LENGTH_HDP_SDP      10    /**< max. length used for HDP */
                                                    /**< SDP records.             */
                                                    /**< this is due to limited   */
                                                    /**< SDP record size          */
#else
#define BLUE_API_MDEP_NAME_LENGTH_HDP_SDP      BLUE_API_MDEP_NAME_LENGTH
#endif

#if !defined(BLUE_API_VERSION_LENGTH)
#define BLUE_API_VERSION_LENGTH                18
#endif



#define BLUE_API_BD_SIZE                       6

/** API version */
#define BLUEAPI_API_VERSION                    0x20131212

/**constant definitions*/
#define BLUE_API_ENABLE_ACL_INFO_CONNECTED_ACTIVE         0x0001
#define BLUE_API_ENABLE_ACL_INFO_CONNECTED_SNIFF          0x0002
#define BLUE_API_ENABLE_ACL_INFO_AUTH_STARTED             0x0004
#define BLUE_API_ENABLE_ACL_INFO_AUTH_SUCCESS             0x0008
#define BLUE_API_ENABLE_ACL_INFO_AUTH_FAILED              0x0010
#define BLUE_API_ENABLE_ACL_INFO_CONNECTION_ENCRYPTED     0x0020
#define BLUE_API_ENABLE_ACL_INFO_CONNECTION_DISCONNECTED  0x0040
#define BLUE_API_ENABLE_ACL_INFO_ADDRESS_RESOLVED         0x0080
#define BLUE_API_ENABLE_ACL_INFO_DEVICE_ROLE              0x0100

#define BLUE_API_ENABLE_ACL_INFO_ALL                      0x01FF

#define BLUE_API_ENABLE_MCL_INFO_IDLE                     0x0001
#define BLUE_API_ENABLE_MCL_INFO_CTRL_CONNECTING          0x0002
#define BLUE_API_ENABLE_MCL_INFO_CTRL_CONNECTED           0x0004
#define BLUE_API_ENABLE_MCL_INFO_CTRL_DISCONNECTING       0x0008
#define BLUE_API_ENABLE_MCL_INFO_CTRL_LISTEN              0x0010
#define BLUE_API_ENABLE_MCL_INFO_DATA_CONNECTING          0x0020
#define BLUE_API_ENABLE_MCL_INFO_DATA_CONNECTED           0x0040
#define BLUE_API_ENABLE_MCL_INFO_DATA_DISCONNECTING       0x0080
#define BLUE_API_ENABLE_MCL_INFO_DATA_LISTEN              0x0100
#define BLUE_API_ENABLE_MCL_INFO_CTRL_WAIT_FOR_RSP        0x0200
#define BLUE_API_ENABLE_MCL_INFO_WAIT_FOR_RSP             0x0400

#define BLUE_API_ENABLE_MCL_INFO_ALL                      0x07FF

#define GATT_APPEARANCE_UNKNOWN                                0
#define GATT_APPEARANCE_GENERIC_PHONE                          64
#define GATT_APPEARANCE_GENERIC_COMPUTER                       128

#define GATT_APPEARANCE_GENERIC_WATCH                          192
#define GATT_APPEARANCE_WATCH_SPORTS_WATCH                     193

#define GATT_APPEARANCE_GENERIC_CLOCK                          256
#define GATT_APPEARANCE_GENERIC_DISPLAY                        320
#define GATT_APPEARANCE_GENERIC_REMOTE_CONTROL                 384
#define GATT_APPEARANCE_GENERIC_EYE_GLASSES                    448
#define GATT_APPEARANCE_GENERIC_TAG                            512
#define GATT_APPEARANCE_GENERIC_KEYRING                        576
#define GATT_APPEARANCE_GENERIC_MEDIA_PLAYER                   640
#define GATT_APPEARANCE_GENERIC_BARCODE_SCANNER                704

#define GATT_APPEARANCE_GENERIC_THERMOMETER                    768
#define GATT_APPEARANCE_THERMOMETER_EAR                        769

#define GATT_APPEARANCE_GENERIC_HEART_RATE_SENSOR              832
#define GATT_APPEARANCE_HEART_RATE_SENSOR_HEART_RATE_BELT      833

#define GATT_APPEARANCE_GENERIC_BLOOD_PRESSURE                 896
#define GATT_APPEARANCE_BLOOD_PRESSURE_ARM                     897
#define GATT_APPEARANCE_BLOOD_PRESSURE_WRIST                   898

#define GATT_APPEARANCE_HUMAN_INTERFACE_DEVICE                 960
#define GATT_APPEARANCE_KEYBOARD                               961
#define GATT_APPEARANCE_MOUSE                                  962
#define GATT_APPEARANCE_JOYSTICK                               963
#define GATT_APPEARANCE_GAMEPAD                                964
#define GATT_APPEARANCE_DIGITIZER_TABLET                       965
#define GATT_APPEARANCE_CARD_READER                            966
#define GATT_APPEARANCE_DIGITAL_PEN                            967
#define GATT_APPEARANCE_BARCODE_SCANNER                        968

#define GATT_APPEARANCE_GENERIC_GLUCOSE_METER                  1024

#define GATT_APPEARANCE_GENERIC_RUNNING_WALKING_SENSOR         1088
#define GATT_APPEARANCE_RUNNING_WALKING_SENSOR_IN_SHOE         1089
#define GATT_APPEARANCE_RUNNING_WALKING_SENSOR_ON_SHOE         1090
#define GATT_APPEARANCE_RUNNING_WALKING_SENSOR_ON_HIP          1091

#define GATT_APPEARANCE_GENERIC_CYCLING                        1152
#define GATT_APPEARANCE_CYCLING_CYCLING_COMPUTER               1153
#define GATT_APPEARANCE_CYCLING_SPEED_SENSOR                   1154
#define GATT_APPEARANCE_CYCLING_CADENCE_SENSOR                 1155
#define GATT_APPEARANCE_CYCLING_POWER_SENSOR                   1156
#define GATT_APPEARANCE_CYCLING_SPEED_AND_CADENCE_SENSOR       1157

#define GATT_APPEARANCE_GENERIC_PULSE_OXIMETER                 3136
#define GATT_APPEARANCE_FINGERTIP                              3137
#define GATT_APPEARANCE_WRIST_WORN                             3138
#define GATT_APPEARANCE_GENERIC_WEIGHT_SCALE                   3200

#define GATT_APPEARANCE_GENERIC_OUTDOOR_SPORTS_ACTIVITY        5184
#define GATT_APPEARANCE_LOCATION_DISPLAY_DEVICE                5185
#define GATT_APPEARANCE_LOCATION_AND_NAVIGATION_DISPLAY_DEVICE 5186
#define GATT_APPEARANCE_LOCATION_POD                           5187
#define GATT_APPEARANCE_LOCATION_AND_NAVIGATION_POD            5188

/**type definitions */
typedef enum
{
	blueAPI_EventConnectMDLReq = 0x01,
	blueAPI_EventConnectMDLRsp,
	blueAPI_EventDisconnectMDLReq,
	blueAPI_EventDisconnectMDLRsp,
	blueAPI_EventDisconnectMDLInd,
	blueAPI_EventDisconnectMDLConf,
	blueAPI_EventCreateMDLInd,
	blueAPI_EventCreateMDLConf,
	blueAPI_EventConnectMDLInfo,
	blueAPI_EventDeleteMDLInfo,
	blueAPI_EventMCLStatusInfo,
	blueAPI_EventACLStatusInfo,

	blueAPI_EventRegisterReq,
	blueAPI_EventRegisterRsp,
	blueAPI_EventReleaseReq,
	blueAPI_EventReleaseRsp,
	blueAPI_EventActInfo,
	blueAPI_EventInternalEventInfo,

	blueAPI_EventDeviceConfigSetReq,
	blueAPI_EventDeviceConfigSetRsp,

    blueAPI_EventDeviceConfigAppearanceGetReq,
    blueAPI_EventDeviceConfigAppearanceGetRsp,
	blueAPI_EventDeviceConfigPerPrefConnParamGetReq,
	blueAPI_EventDeviceConfigPerPrefConnParamGetRsp,
	blueAPI_EventDeviceConfigDeviceNameGetReq,
    blueAPI_EventDeviceConfigDeviceNameGetRsp,
	
	/** GATT ---*/
	/** server */
	blueAPI_EventGATTServiceRegisterReq,
	blueAPI_EventGATTServiceRegisterRsp,
	//blueAPI_EventGATTServiceReleaseReq,
	//blueAPI_EventGATTServiceReleaseRsp,
	blueAPI_EventGATTAttributeUpdateReq,
	blueAPI_EventGATTAttributeUpdateRsp,
	blueAPI_EventGATTAttributeUpdateStatusInd,
	blueAPI_EventGATTAttributeUpdateStatusConf,
	blueAPI_EventGATTAttributeReadInd,
	blueAPI_EventGATTAttributeReadConf,
	blueAPI_EventGATTAttributeWriteInd,
	blueAPI_EventGATTAttributeWriteConf,
	blueAPI_EventGATTAttributePrepareWriteInd,
    blueAPI_EventGATTAttributePrepareWriteConf,
    blueAPI_EventGATTAttributeExecuteWriteInd,
    blueAPI_EventGATTAttributeExecuteWriteConf,
	blueAPI_EventGATTAttributeWriteCommandInfo,

	blueAPI_EventGATTCCCDInfo,
	/** client */
	blueAPI_EventGATTDiscoveryReq,
	blueAPI_EventGATTDiscoveryRsp,
	blueAPI_EventGATTDiscoveryInd,
	blueAPI_EventGATTDiscoveryConf,
	blueAPI_EventGATTAttributeReadReq,
	blueAPI_EventGATTAttributeReadRsp,
	blueAPI_EventGATTAttributeReadMultipleReq,
    blueAPI_EventGATTAttributeReadMultipleRsp,
	blueAPI_EventGATTAttributeWriteReq,
	blueAPI_EventGATTAttributeWriteRsp,
	blueAPI_EventGATTAttributePrepareWriteReq,
    blueAPI_EventGATTAttributePrepareWriteRsp,
    blueAPI_EventGATTAttributeExecuteWriteReq,
    blueAPI_EventGATTAttributeExecuteWriteRsp,

	blueAPI_EventGATTAttributeInd,
	blueAPI_EventGATTAttributeConf,
	blueAPI_EventGATTAttributeNotificationInfo,
	
	/** common */
	blueAPI_EventGATTSecurityReq,
	blueAPI_EventGATTSecurityRsp,
	blueAPI_EventGATTServerStoreInd,
	blueAPI_EventGATTServerStoreConf,
	blueAPI_EventGATTMtuSizeInfo,
	/** end of GATT ---*/

	/** Start of definition part: Secutity Management */
	blueAPI_EventPairableModeSetReq,
	blueAPI_EventPairableModeSetRsp,
	blueAPI_EventUserPasskeyReqInd,
	blueAPI_EventUserPasskeyReqConf,
	blueAPI_EventUserPasskeyReqReplyReq,
	blueAPI_EventUserPasskeyReqReplyRsp,
	blueAPI_EventUserPasskeyNotificationInfo,
	blueAPI_EventRemoteOOBDataReqInd,
	blueAPI_EventRemoteOOBDataReqConf,
	blueAPI_EventAuthResultInd,
	blueAPI_EventAuthResultConf,
	blueAPI_EventAuthResultRequestInd,
	blueAPI_EventAuthResultRequestConf,
	
	blueAPI_EventLEAdvertiseReq,
	blueAPI_EventLEAdvertiseRsp,
	blueAPI_EventLEAdvertiseParameterSetReq,
	blueAPI_EventLEAdvertiseParameterSetRsp,
	blueAPI_EventLEAdvertiseDataSetReq,
	blueAPI_EventLEAdvertiseDataSetRsp,
	blueAPI_EventLEScanReq,
	blueAPI_EventLEScanRsp,
	blueAPI_EventLEScanInfo,
	blueAPI_EventLEModifyWhitelistReq,
	blueAPI_EventLEModifyWhitelistRsp,
	blueAPI_EventLEConnectionUpdateReq,
	blueAPI_EventLEConnectionUpdateRsp,
	blueAPI_EventLEConnectionUpdateInd,
	blueAPI_EventLEConnectionUpdateConf,
	blueAPI_EventLEConnectionParameterInfo,
	blueAPI_EventLEPrivacyModeReq,
	blueAPI_EventLEPrivacyModeRsp,

    blueAPI_EventCreateLEDataChannelReq,
    blueAPI_EventCreateLEDataChannelRsp,
    blueAPI_EventCreateLEDataChannelInd,
    blueAPI_EventCreateLEDataChannelConf,
    blueAPI_EventDisconnectLEDataChannelReq,
    blueAPI_EventDisconnectLEDataChannelRsp,
    blueAPI_EventDisconnectLEDataChannelInd,
    blueAPI_EventDisconnectLEDataChannelConf,
    blueAPI_EventSendLEFlowControlCreditReq,
    blueAPI_EventSendLEFlowControlCreditRsp,
    blueAPI_EventLEDataReq,
    blueAPI_EventLEDataRsp,
    blueAPI_EventLEDataInd,
    blueAPI_EventLEDataConf,
    blueAPI_EventLEDataChannelParameterInfo,
    blueAPI_EventLEDataChannelCreditsAlertInfo,
    blueAPI_EventLEDataChannelDeleteInfo,
    blueAPI_EventLEPsmSecuritySetReq,
    blueAPI_EventLEPsmSecuritySetRsp,
    blueAPI_EventVendorSetVoiceParaReq,
    blueAPI_EventVendorSetVoiceParaRsp,
    blueAPI_EventSetBleTxPowerReq,
    blueAPI_EventSetBleTxPowerRsp,
    blueAPI_EventSetRandomAddressReq,
    blueAPI_EventSetRandomAddressRsp,
    blueAPI_EventSetDataLengthReq,
    blueAPI_EventSetDataLengthRsp,
    blueAPI_EventDataLengthChangeInfo,

    blueAPI_EventReconnectMDLReq = 0xC8,
	blueAPI_EventReconnectMDLRsp,
	blueAPI_EventReconnectMDLInd,
	blueAPI_EventReconnectMDLConf,

	blueAPI_EventRadioModeSetReq,
	blueAPI_EventRadioModeSetRsp,

    blueAPI_EventInquiryReq,
	blueAPI_EventInquiryRsp,
	blueAPI_EventInquiryDeviceInfo,
	blueAPI_EventDeviceNameReq,
	blueAPI_EventDeviceNameRsp,
	blueAPI_EventDIDDeviceInd,
	blueAPI_EventDIDDeviceConf,

	blueAPI_EventSDPDiscoveryReq,
	blueAPI_EventSDPDiscoveryRsp,
	blueAPI_EventSDPEndpointInd,
	blueAPI_EventSDPEndpointConf,

    blueAPI_EventGATTSDPDiscoveryReq,
	blueAPI_EventGATTSDPDiscoveryRsp,
	blueAPI_EventGATTSDPDiscoveryInd,
	blueAPI_EventGATTSDPDiscoveryConf,

    blueAPI_EventAuthReq,
	blueAPI_EventAuthRsp,
	blueAPI_EventUserAuthorizationReqInd,
	blueAPI_EventUserAuthorizationReqConf,
	blueAPI_EventUserAuthRequestInd,
	blueAPI_EventUserAuthRequestConf,
	blueAPI_EventUserConfirmationReqInd,
	blueAPI_EventUserConfirmationReqConf,
	blueAPI_EventKeypressNotificationReq,
	blueAPI_EventKeypressNotificationRsp,
	blueAPI_EventKeypressNotificationInfo,
	blueAPI_EventLegacyRemoteOOBDataReqInd,
	blueAPI_EventLegacyRemoteOOBDataReqConf,
	blueAPI_EventLocalOOBDataReq,
	blueAPI_EventLocalOOBDataRsp,
	blueAPI_EventAuthDeleteReq,
	blueAPI_EventAuthDeleteRsp,
	blueAPI_EventAuthListReq,
	blueAPI_EventAuthListInfo,
	blueAPI_EventAuthListRsp,
	/**   End of definition part: Secutity Management */

	blueAPI_EventACLConfigReq,
	blueAPI_EventACLConfigRsp,

    /*adapter layer*/
    blueAPI_EventSDPRegisterReq,
    blueAPI_EventSDPRegisterRsp,
    blueAPI_EventL2cProtocolRegisterReq,
    blueAPI_EventL2cProtocolRegisterRsp,
    blueAPI_EventL2cConReq,
    blueAPI_EventL2cConRsp,
    blueAPI_EventL2cConInd,
    blueAPI_EventL2cConConf,
    blueAPI_EventL2cConActInd,
    blueAPI_EventL2cDataReq,
    blueAPI_EventL2cDataRsp,
    blueAPI_EventL2cDataInd,
    blueAPI_EventL2cDataConf,
    blueAPI_EventL2cDiscReq,
    blueAPI_EventL2cDiscRsp,
    blueAPI_EventL2cDiscInd,
    blueAPI_EventL2cDiscConf,

    blueAPI_EventL2cSecurityRegisterReq,
    blueAPI_EventL2cSecurityRegisterRsp,
    blueAPI_EventRFCAuthenticationReq,
    blueAPI_EventRFCAuthenticationRsp,

    blueAPI_EventSCOConReq,
    blueAPI_EventSCOConRsp,
    blueAPI_EventSCOConInd,
    blueAPI_EventSCOConConf,
    blueAPI_EventSCOConActInd,
    blueAPI_EventSCODiscReq,
    blueAPI_EventSCODiscRsp,
    blueAPI_EventSCODiscInd,
    blueAPI_EventSCODiscConf,

	blueAPI_EventIdle,
} TBlueAPI_Command;
typedef TBlueAPI_Command * PBlueAPI_Command;

typedef enum
{
	blueAPI_CauseSuccess             = 0x00,
	blueAPI_CauseAccept,
	blueAPI_CauseReject,
	blueAPI_CauseResourceError,
	blueAPI_CauseInvalidParameter,
	blueAPI_CauseInvalidState,
	blueAPI_CauseConnectionDisconnect,
	blueAPI_CauseConnectionLost,
	blueAPI_CauseAuthenticationFailed,
	blueAPI_CauseInitTimeout,
	blueAPI_CauseInitOutofSync,
	blueAPI_CauseInitHardwareFailure,
	blueAPI_CauseLowerLayerError,
	blueAPI_CauseAddressNotResolved,
	blueAPI_CauseConnectionPaused = 0x30,
	blueAPI_CauseFlowcontrolViolation,
	
	blueAPI_CauseUnspecified         = 0xFD,
	blueAPI_CauseNotSupported        = 0xFE
} TBlueAPI_Cause;
typedef TBlueAPI_Cause * PBlueAPI_Cause;

typedef uint16_t TBlueAPI_SubCause;
typedef TBlueAPI_SubCause * PBlueAPI_SubCause;

typedef uint8_t  TBlueAPI_TimeStamp[8];
typedef TBlueAPI_TimeStamp * PBlueAPI_TimeStamp;

typedef enum
{
	blueAPI_RemoteBDTypeClassic  = 0x00,
	blueAPI_RemoteBDTypeLEPublic = 0x02,
	blueAPI_RemoteBDTypeLERandom = 0x03,
	blueAPI_RemoteBDTypeAny = 0x04,
	blueAPI_RemoteBDTypeLEResolved = 0x0A
} TBlueAPI_RemoteBDType;
typedef TBlueAPI_RemoteBDType * PBlueAPI_RemoteBDType;

/** @brief define local adress type */
typedef enum
{
    blueAPI_LocalBDTypeLEPublic = 0x00,         /**<  Bluetooth low energy public address. */
    blueAPI_LocalBDTypeLERandom = 0x01,         /**<  Bluetooth low energy random address. */
} TBlueAPI_LocalBDType;
typedef TBlueAPI_LocalBDType * PBlueAPI_LocalBDType;

typedef enum
{
	blueAPI_FrameTypeUnsegmented     = 0x00,
	blueAPI_FrameTypeFirstSegment    = 0x01,
	blueAPI_FrameTypeLastSegment     = 0x02,
	blueAPI_FrameTypeContinueSegment = 0x03
} TBlueAPI_FrameType;
typedef TBlueAPI_FrameType * PBlueAPI_FrameType;

typedef enum
{
	blueAPI_MDEPRoleSource    = 0, /**< HDP source role */
	blueAPI_MDEPRoleSink      = 1, /**< HDP sink role  */
	blueAPI_MDEPRoleMCAP      = 2, /**< not defined in HDP accepts all configs */
	blueAPI_MDEPRoleRFCOMM    = 3, /**< SPP/RFCOMM Mdep (internal use only) */
	blueAPI_MDEPRoleGATT      = 4, /**< GATT Mdep (internal use only)       */
	blueAPI_MDEPRoleL2CAP     = 5, /**< L2CAP Mdep (internal use only)      */
	blueAPI_MDEPRoleHID       = 6, /**< HID Mdep (internal use only)        */
	blueAPI_MDEPRoleDontCare  = 99 /**< internal use only                   */
} TBlueAPI_MDEPRole;
typedef TBlueAPI_MDEPRole * PBlueAPI_MDEPRole;

typedef enum _TBlueAPI_LinkConfigType
{  
	blueAPI_LinkConfigHDPDontCare  = 0,  /**< HDP: no QoS preference */
	blueAPI_LinkConfigHDPReliable  = 1,  /**< HDP: request reliable (lossless) channel */
	blueAPI_LinkConfigHDPStreaming = 2,  /**< HDP: request streaming (lossy) channel */
	blueAPI_LinkConfigSPPBasic     = 3,  /**< SPP/RFCOMM channel */
	blueAPI_LinkConfigGATT         = 4,  /**< GATT channel */
	blueAPI_LinkConfigSDP          = 5,  /**< SDP channel (internal/do not use) */
	blueAPI_LinkConfigAppleIAP     = 6,  /**< Apple iAP/RFCOMM channel, General Lingo subset */
	blueAPI_LinkConfigL2CapBasic     = 7,  /**< L2CAP: basic mode */
	blueAPI_LinkConfigL2CapReliable  = 8,  /**< L2CAP: reliable mode  */
	blueAPI_LinkConfigL2CapStreaming = 9,  /**< L2CAP: streaming mode */
	blueAPI_LinkConfigAppleIAPSimpleRemote = 10, /**< Apple iAP/RFCOMM channel, Simple Remote Lingo */
	blueAPI_LinkConfigHID            =       11, /**< HID */
} TBlueAPI_LinkConfigType;
typedef TBlueAPI_LinkConfigType * PBlueAPI_LinkConfigType;

typedef enum
{
	/** HID */
	blueAPI_MDEPUUIDHID                 = 0x0011,

    blueAPI_MDEPUUIDL2CAP               = 0x0100,

	/** HDP */
	blueAPI_MDEPUUIDPulseOximeter       = 0x1004,
	blueAPI_MDEPUUIDBasicECGHeartRate   = 0x1006,
	blueAPI_MDEPUUIDBloodPressure       = 0x1007,
	blueAPI_MDEPUUIDBodyTemperature     = 0x1008,
	blueAPI_MDEPUUIDWeight              = 0x100F,
	blueAPI_MDEPUUIDGlucose             = 0x1011,
	blueAPI_MDEPUUIDINRMonitor          = 0x1012,
	blueAPI_MDEPUUIDBodyComposition     = 0x1014,
	blueAPI_MDEPUUIDPeakFlowMonitor     = 0x1015,
	blueAPI_MDEPUUIDCardiocasFandAMon   = 0x1029,
	blueAPI_MDEPUUIDStrengthFitness     = 0x102A,
	blueAPI_MDEPUUIDLivingActivityHub   = 0x1047,
	blueAPI_MDEPUUIDStepCounter         = 0x1068,
	blueAPI_MDEPUUIDFallSensor          = 0x1075,
	blueAPI_MDEPUUIDPersEmergancyRsp    = 0x1076,
	blueAPI_MDEPUUIDSmokeSensor         = 0x1077,
	blueAPI_MDEPUUIDCOSensor            = 0x1078,
	blueAPI_MDEPUUIDWaterSensor         = 0x1079,
	blueAPI_MDEPUUIDGasSensor           = 0x107A,
	blueAPI_MDEPUUIDMotionSensor        = 0x107B,
	blueAPI_MDEPUUIDPropertyExit        = 0x107C,
	blueAPI_MDEPUUIDEnuresisSensor      = 0x107D,
	blueAPI_MDEPUUIDContactClosure      = 0x107E,
	blueAPI_MDEPUUIDUsageSensor         = 0x107F,
	blueAPI_MDEPUUIDSwitchSensor        = 0x1080,
	blueAPI_MDEPUUIDMedicationDosing    = 0x1081,
	blueAPI_MDEPUUIDTemperature         = 0x1082,
	blueAPI_MDEPUUIDMedicationMonitor   = 0x1082,

	/** SPP */
	blueAPI_MDEPUUIDSPPSerialPort       = 0x1101,
	blueAPI_MDEPUUIDSPPDialupNetworking = 0x1103,

	/** GOEP */
	blueAPI_MDEPUUIDOBEXSynch          = 0x1104,
	blueAPI_MDEPUUIDOBEXObjectPush     = 0x1105,
	blueAPI_MDEPUUIDOBEXFileTransfer   = 0x1106,
	blueAPI_MDEPUUIDOBEXSynchCommand   = 0x1107,

	/** Apple iPod Accessory Protocol (iAP) */
	blueAPI_MDEPUUIDAppleIAP            = 0xFA01,

	/** Special */
	blueAPI_MDEPUUIDTypeSpecial         = 0xFFFE,
	blueAPI_MDEPUUIDTypeEcho            = 0xFFFF
} TBlueAPI_MDEPDataType;
typedef TBlueAPI_MDEPDataType * PBlueAPI_MDEPDataType;

typedef enum
{
	blueAPI_MCLIdle                   = 0x01,
	blueAPI_MCLControlConnecting      = 0x02,
	blueAPI_MCLControlConnected       = 0x03,
	blueAPI_MCLControlDisconnecting   = 0x04,
	blueAPI_MCLControlListening       = 0x05,
	blueAPI_MCLDataConnecting         = 0x06,
	blueAPI_MCLDataConnected          = 0x07,
	blueAPI_MCLDataDisconnecting      = 0x08,
	blueAPI_MCLDataListening          = 0x09,
	blueAPI_MCLControlWaitForResponse = 0x0A,
	blueAPI_MCLComWaitForResponse     = 0x0B,
	blueAPI_MCLReleased               = 0x0C,
	blueAPI_MCLControlConnectingDataListening  = 0x0D,
	blueAPI_MCLControlConnectingDataConnecting = 0x0E,
	blueAPI_MCLControlConnectedDataConnecting  = 0x0F
} TBlueAPI_MCLStatus;
typedef TBlueAPI_MCLStatus * PBlueAPI_MCLStatus;

typedef enum
{
	blueAPI_ACLConnectedActive        = 0x01,
	blueAPI_ACLConnectedSniff         = 0x02,
	blueAPI_ACLAuthenticationStarted  = 0x03,
	blueAPI_ACLAuthenticationSuccess  = 0x04,
	blueAPI_ACLAuthenticationFailure  = 0x05,
	blueAPI_ACLConnectionEncrypted    = 0x06,
	blueAPI_ACLConnectionDisconnected = 0x07,
	blueAPI_ACLConnectionNotEncrypted = 0x08,
	blueAPI_ACLAddressResolved        = 0x09,
	blueAPI_ACLDeviceRoleMaster       = 0x0A,
	blueAPI_ACLDeviceRoleSlave        = 0x0B,
	blueAPI_ACLConnectedSniffSubrate  = 0x0C,
	blueAPI_ACLConnectedLinkStatus    = 0x0D,

} TBlueAPI_ACLStatus;
typedef TBlueAPI_ACLStatus * PBlueAPI_ACLStatus;

typedef enum
{
	blueAPI_LinkKeyTypeCombination      = 0x00,     /**< BT2.0 link key              */
	blueAPI_LinkKeyTypeUnauthenticated  = 0x04,     /**< SSP generated link key without MITM protection */
	blueAPI_LinkKeyTypeAuthenticated    = 0x05,     /**< SSP generated link key with MITM protection    */
	blueAPI_LinkKeyTypeLELocalLTK       = 0x11,     /**< BLE Long Term Key Blob                         */
	blueAPI_LinkKeyTypeLERemoteLTK      = 0x12,     /**< BLE Long Term Key Blob                         */
	blueAPI_LinkKeyTypeLELocalIRK       = 0x13,     /**< BLE Identity Resolving Key                     */
	blueAPI_LinkKeyTypeLERemoteIRK      = 0x14,     /**< BLE Identity Resolving Key                     */
	blueAPI_LinkKeyTypeRequestBR        = 0x80,     /**< only used to request a BR linkkey              */
	blueAPI_LinkKeyTypeDeleted          = 0xFF      /**< Link key is no longer valid and deleted        */
} TBlueAPI_LinkKeyType;

/** message definitions */
/** handles exchanged between application and BlueAPI */
typedef void * TBlueAPIAppHandle;    /**< app -> BlueAPI */
typedef void * TBlueAPIHandle;       /**< BlueAPI -> app */

typedef struct _TBlueAPI_ConnectMDLReqGATT
{
	uint16_t scanInterval;
	uint16_t scanWindow;
	uint16_t scanTimeout;
	uint16_t connIntervalMin;
	uint16_t connIntervalMax;
	uint16_t connLatency;
	uint16_t supervisionTimeout;
  	uint16_t CE_Length;
} TBlueAPI_ConnectMDLReqGATT;
typedef TBlueAPI_ConnectMDLReqGATT * PBlueAPI_ConnectMDLReqGATT;

typedef union _TBlueAPI_ConnectMDLReqParam
{
	TBlueAPI_ConnectMDLReqGATT  gatt;
} TBlueAPI_ConnectMDLReqParam;
typedef TBlueAPI_ConnectMDLReqParam * PBlueAPI_ConnectMDLReqParam;

typedef struct _TBlueAPI_ConnectMDLReq
{
	/** cmd specific section */
	uint8_t                        remote_BD[BLUE_API_BD_SIZE];
	TBlueAPI_RemoteBDType          remote_BD_Type;
    TBlueAPI_LocalBDType           local_BD_Type;
	uint8_t                        local_MDEP_ID;
	TBlueAPI_LinkConfigType        linkConfigType;
	TBlueAPI_ConnectMDLReqParam    p;
} TBlueAPI_ConnectMDLReq;
typedef TBlueAPI_ConnectMDLReq * PBlueAPI_ConnectMDLReq;

typedef struct
{
    /** cmd specific section */
	uint8_t           remote_BD[BLUE_API_BD_SIZE];
	TBlueAPI_RemoteBDType  remote_BD_type;
	uint16_t          local_MDL_ID;
	TBlueAPI_Cause    cause;
} TBlueAPI_ConnectMDLRsp;
typedef TBlueAPI_ConnectMDLRsp * PBlueAPI_ConnectMDLRsp;

typedef struct
{
	/** cmd specific section */
	uint8_t              remote_BD[BLUE_API_BD_SIZE];
	TBlueAPI_RemoteBDType remote_BD_type;
	uint16_t             local_MDL_ID;
}  TBlueAPI_CreateMDLInd;
typedef TBlueAPI_CreateMDLInd * PBlueAPI_CreateMDLInd;

typedef struct
{
	/** cmd specific section */
	uint16_t             local_MDL_ID;
	uint8_t              maxTPDUusCredits;
	TBlueAPI_Cause       cause;
}  TBlueAPI_CreateMDLConf;
typedef TBlueAPI_CreateMDLConf * PBlueAPI_CreateMDLConf;

typedef struct
{
    /** cmd specific section */
	uint16_t            local_MDL_ID;
	TBlueAPI_Cause      cause;
}  TBlueAPI_DisconnectMDLReq;
typedef TBlueAPI_DisconnectMDLReq * PBlueAPI_DisconnectMDLReq;

typedef struct
{
	/** cmd specific section */
	uint16_t          local_MDL_ID;
	TBlueAPI_Cause    cause;
}  TBlueAPI_DisconnectMDLInd;
typedef TBlueAPI_DisconnectMDLInd * PBlueAPI_DisconnectMDLInd;

typedef struct
{
	/** cmd specific section */
	uint16_t          local_MDL_ID;
}  TBlueAPI_DisconnectMDLConf;
typedef TBlueAPI_DisconnectMDLConf * PBlueAPI_DisconnectMDLConf;

typedef struct
{
	/** cmd specific section */
	uint16_t          local_MDL_ID;
	TBlueAPI_Cause    cause;
}  TBlueAPI_DisconnectMDLRsp;
typedef TBlueAPI_DisconnectMDLRsp * PBlueAPI_DisconnectMDLRsp;

typedef struct
{
	/** cmd specific section */
	uint16_t    local_MDL_ID;
	uint16_t    remote_Control_PSM;
	uint16_t    remote_Data_PSM;
}  TBlueAPI_ReconnectMDLReq;
typedef TBlueAPI_ReconnectMDLReq * PBlueAPI_ReconnectMDLReq;

typedef struct
{
	/** cmd specific section */
	uint16_t       local_MDL_ID;
	TBlueAPI_Cause cause;
}  TBlueAPI_ReconnectMDLRsp;
typedef TBlueAPI_ReconnectMDLRsp * PBlueAPI_ReconnectMDLRsp;

typedef struct
{
	/** cmd specific section */
	uint16_t    local_MDL_ID;
}  TBlueAPI_ReconnectMDLInd;
typedef TBlueAPI_ReconnectMDLInd * PBlueAPI_ReconnectMDLInd;

typedef struct
{
	/** cmd specific section */
	uint16_t       local_MDL_ID;
	TBlueAPI_Cause cause;
}  TBlueAPI_ReconnectMDLConf;
typedef TBlueAPI_ReconnectMDLConf * PBlueAPI_ReconnectMDLConf;

typedef struct
{
	/** cmd specific section */
	uint16_t    local_MDL_ID;
	uint16_t    dsPoolID;
	uint16_t    dsDataOffset;
	uint16_t    maxTPDUSize; /**< Max segment size   */
	TBlueAPI_LinkConfigType linkConfigType;
	uint8_t     maxTPDUdsCredits;
} TBlueAPI_ConnectMDLInfo;
typedef TBlueAPI_ConnectMDLInfo * PBlueAPI_ConnectMDLInfo;

typedef struct
{
	/** cmd specific section */
	uint16_t    local_MDL_ID;
} TBlueAPI_DeleteMDLInfo;
typedef TBlueAPI_DeleteMDLInfo * PBlueAPI_DeleteMDLInfo;

typedef struct
{
	/** cmd specific section */
	uint8_t           remote_BD[BLUE_API_BD_SIZE];
	uint16_t          local_MCL_ID;
	TBlueAPI_MCLStatus status;
} TBlueAPI_MCLStatusInfo;
typedef TBlueAPI_MCLStatusInfo * PBlueAPI_MCLStatusInfo;

typedef enum
{
    blueAPI_KeyType_Unauthenticated       = 0x04,  /**<  no mitm key. */
    blueAPI_KeyType_Authenticated         = 0x05,  /**<  mitm key. */
    blueAPI_KeyType_Delete                = 0xff,  /**<  the key deleted. */
} TBlueAPI_KeyType;

typedef struct
{
	TBlueAPI_KeyType  keyType;
	uint8_t           keySize;
} TBlueAPI_ACLStatusParamAuth;
typedef TBlueAPI_ACLStatusParamAuth * PBlueAPI_ACLStatusParamAuth;

typedef struct
{
	uint8_t               remote_BD[BLUE_API_BD_SIZE];
	TBlueAPI_RemoteBDType remote_BD_type;
} TBlueAPI_ACLStatusParamResolve;
typedef TBlueAPI_ACLStatusParamResolve * PBlueAPI_ACLStatusParamResolve;

typedef struct
{
	uint16_t  interval;
} TBlueAPI_ACLStatusParamSniff;
typedef TBlueAPI_ACLStatusParamSniff * PBlueAPI_ACLStatusParamSniff;

typedef struct
{
	uint16_t  maxTxLatency;
	uint16_t  maxRxLatency;
	uint16_t  minRemoteTimeout;
	uint16_t  minLocalTimeout;
} TBlueAPI_ACLStatusParamSniffSubrate;
typedef TBlueAPI_ACLStatusParamSniffSubrate * PBlueAPI_ACLStatusParamSniffSubrate;

typedef struct
{
	uint8_t   linkQuality;
	int8_t    rssi;
	uint16_t  failedContacts;
	int8_t    txPower;
	int8_t    absoluteRssi;
} TBlueAPI_ACLStatusParamLinkStatus;
typedef TBlueAPI_ACLStatusParamLinkStatus * PBlueAPI_ACLStatusParamLinkStatus;

typedef union
{
	TBlueAPI_ACLStatusParamAuth    auth;
	TBlueAPI_ACLStatusParamResolve resolve;
	TBlueAPI_ACLStatusParamSniff        sniff;
	TBlueAPI_ACLStatusParamSniffSubrate sniffSubrate;
	TBlueAPI_ACLStatusParamLinkStatus   linkStatus;
} TBlueAPI_ACLStatusParam;
typedef TBlueAPI_ACLStatusParam * PBlueAPI_ACLStatusParam;

typedef struct
{
	/** cmd specific section */
	uint8_t                  remote_BD[BLUE_API_BD_SIZE];
	TBlueAPI_RemoteBDType    remote_BD_type;
	TBlueAPI_ACLStatus       status;
	TBlueAPI_ACLStatusParam  p;
} TBlueAPI_ACLStatusInfo;
typedef TBlueAPI_ACLStatusInfo * PBlueAPI_ACLStatusInfo;

typedef struct
{
	/** generic cmd section  */
	union
	{
		TBlueAPIAppHandle  appHandle;
		TBlueAPIHandle     blueAPIHandle;
	} u;
	/** cmd specific section */
} TBlueAPI_Generic;
typedef TBlueAPI_Generic * PBlueAPI_Generic;

typedef enum
{
	blueAPI_ServiceApplicationDefined = 0,   /**< application provides services */
} TBlueAPI_GATTServiceID;

typedef struct
{
    void *             MDHmsgHandlerCallback;
    uint32_t           apiVersion;
} TBlueAPI_RegisterReq;
typedef TBlueAPI_RegisterReq * PBlueAPI_RegisterReq;

typedef struct
{
	TBlueAPI_Cause     cause;
} TBlueAPI_RegisterRsp;
typedef TBlueAPI_RegisterRsp * PBlueAPI_RegisterRsp;

typedef struct
{
	/** generic cmd section  */
	uint8_t Reserved;
} TBlueAPI_ReleaseReq;
typedef TBlueAPI_ReleaseReq * PBlueAPI_ReleaseReq;

typedef struct
{
	/** cmd specific section */
	TBlueAPI_Cause     cause;
} TBlueAPI_ReleaseRsp;
typedef TBlueAPI_ReleaseRsp * PBlueAPI_ReleaseRsp;

typedef struct
{
	/** cmd specific section */
	uint8_t     local_BD[BLUE_API_BD_SIZE];
	uint8_t     version[BLUE_API_VERSION_LENGTH];
	uint16_t    systemPoolID;
	TBlueAPI_Cause cause;
} TBlueAPI_ActInfo;
typedef TBlueAPI_ActInfo * PBlueAPI_ActInfo;

typedef enum
{
	blueAPI_InternalEventInvalidCreateMDLConf = 0x01,
	blueAPI_InternalEventInvalidDisconnectMDLConf,
	blueAPI_InternalEventInvalidSecurityConf,
	blueAPI_InternalEventInvalidRemoteEvent,
	blueAPI_InternalEventCommunicationTimeout,
	blueAPI_InternalEventInvalidGATTAttributeReadConf,
	blueAPI_InternalEventInvalidGATTAttributeWriteConf,
	blueAPI_InternalEventInvalidGATTAttributePrepareWriteConf,
	blueAPI_InternalEventInvalidGATTExecuteWriteConf,
	blueAPI_InternalEventInvalidGATTDiscoveryConf,
	blueAPI_InternalEventInvalidGATTAttributeConf,
	blueAPI_InternalEventInvalidLEConnectionUpdateConf,
    blueAPI_InternalEventInvalidCreateLEDataChannelConf,
    blueAPI_InternalEventInvalidDisconnectLEDataChannelConf,
    blueAPI_InternalEventInvalidLEDataConf,
    
	blueAPI_InternalEventInvalidGATTSDPDiscoveryConf = 0x30,
	blueAPI_InternalEventInvalidHIDDataChannelCount,
	blueAPI_InternalEventInvalidReconnectMDLConf,
	blueAPI_InternalEventInvalidDataConf,
	blueAPI_InternalEventInvalidDIDDeviceConf,
	blueAPI_InternalEventInvalidHDPServiceConf,
	blueAPI_InternalEventInvalidHDPEndpointConf,
	blueAPI_InternalEventInvalidSPPEndpointConf,
	blueAPI_InternalEventInvalidOBEXEndpointConf,
} TBlueAPI_InternalEventType;

typedef struct
{
	/** cmd specific section */
	TBlueAPI_InternalEventType    eventType;
	uint32_t                      eventInfo;
	TBlueAPI_Cause                cause;
} TBlueAPI_InternalEventInfo;
typedef TBlueAPI_InternalEventInfo * PBlueAPI_InternalEventInfo;

/**Secutity Management specific definitions*/
typedef enum
{
	blueAPI_SSPEntryStarted,              /**< the passkey key entry protocol procedure is started    */
	blueAPI_SSPDigitEntered,              /**< a digit is entered by the remote user */
	blueAPI_SSPDigitErased,               /**< a digit is erased by the remote user */
	blueAPI_SSPCleared,                   /**< the display is cleared by the remote user */
	blueAPI_SSPEntryComplete,             /**< the passkey key entry protocol procedure is completed  */
} TBlueAPI_SSPKeyEvent;

typedef enum
{
	blueAPI_BTMode21Disabled,        /**< disable all Bluetooth 2.1 functionality */
	blueAPI_BTMode21Enabled,         /**< enable Bluetooth 2.1 functionality  */
	blueAPI_BTMode21DebugEnabled     /**< enable Bluetooth 2.1 functionality with SSP debug support
	                                     (Note: this mode shall be used only for debugging purpose
	                                     since it disables over-the-air data encryption)        */
} TBlueAPI_BluetoothMode;

typedef enum
{
	blueAPI_IOCapDisplayOnly,                        /**< only a Display present, no Keyboard or Yes/No Keys     */
	blueAPI_IOCapDisplayYesNo,                       /**< Display and Yes/No Keys present  */
	blueAPI_IOCapKeyboardOnly,                       /**< only a Keyboard present, no Display    */
	blueAPI_IOCapNoIO,                               /**< no input/output capabilities   */
	blueAPI_IOCapKeyboardDisplay                     /**< BLE: Keyboard and Display present    */
} TBlueAPI_IOCapabilities;

typedef enum
{
	blueAPI_AuthNoMITMRequiredNoStore,              /**< MITM protection not required, no bonding               */
	blueAPI_AuthMITMRequiredNoStore,                /**< MITM protection required, use IO capabilities, no bonding  */
	blueAPI_AuthNoMITMRequiredBonding,     /**< MITM protection not required, perform dedicated Bonding  */
	blueAPI_AuthMITMRequiredBonding,       /**< MITM protection required, use IO capabilities, perform dedicated Bonding */
	blueAPI_AuthNoMITMRequiredGeneralBonding,       /**< MITM protection not required, perform general Bonding  */
	blueAPI_AuthMITMRequiredGeneralBonding          /**< MITM protection not required, use IO capabilities, perform general Bonding */
} TBlueAPI_AuthRequirements;

typedef enum
{
	blueAPI_StoreBondModeNoStore,                   /**< Bonds persists for the duration of the authenticated connection */
	blueAPI_StoreBondModeNVStore,                   /**< Bonds are permanently stored in the NVRAM of the BlueRS+        */
	blueAPI_StoreBondModeRAMStore,                  /**< Bonds persist until the next power cycle                        */
	blueAPI_StoreBondModeExtStore                   /**< Bonds are saved by the Application                              */
} TBlueAPI_StoreBondModes;

/** used by the MDH to set the pairable mode of the local device */
typedef struct
{
	/** cmd specific section */
	bool                       enablePairableMode;     /**< If this parameter is set to TRUE, pairable mode is enabled,
	                                                 otherwise pairable mode is disabled                                        */
    TBlueAPI_BluetoothMode     BluetoothMode;   
	TBlueAPI_AuthRequirements  AuthRequirements;       /**< This parameter defines the authentication requirements for authentication
	                                                 while the local device is in pairable mode */
	TBlueAPI_IOCapabilities    IOCapabilities;         /**< defines the input/output capabilities that can be used for authentication  */
	bool                       remoteOOBDataPresent;   /**< If this parameter is set to TRUE, remote OOB data is present and can be used
	                                                 for authentication, otherwise OOB functionality is disabled                */
    bool                       isSetBluetoothMode;
                                                     
} TBlueAPI_PairableModeSetReq;
typedef TBlueAPI_PairableModeSetReq * PBlueAPI_PairableModeSetReq;

typedef enum
{
	blueAPI_RadioVisibleConnectable=0x01,  /**< The device is visible for inquiring devices and scans for
	                                      incoming connections (inquiry scan and page scan
	                                      enabled on local device) */
	blueAPI_RadioVisible=0x02,             /**< The device is visible for inquiring devices (inquiry scan
	                                      enabled, no page scan) */
	blueAPI_RadioConnectable=0x03,         /**< The device scans for incoming connections (page scan
	                                      enabled, no inquiry scan) */
	blueAPI_RadioNonDiscoverable=0x04,     /**< The device is not visible for inquiring devices and does
	                                      not scan for incoming connections (no inquiry scan and
	                                      page scan disabled) but is able to initiate connections */
	blueAPI_RadioDeepSleep=0x05,           /**< The Bluetooth Radio is in a non-operational mode but
	                                      retains its local configuration and can be restored to
	                                      normal operation without reconfiguration */
	blueAPI_RadioOff=0x06                  /**< The Bluetooth Radio is switched off and has to be
	                                      reconfigured for normal operation */
} TBlueAPI_RadioMode;

typedef struct
{
	/** cmd specific section */
	TBlueAPI_RadioMode localRadioMode;      /**< Bluetooth radio mode to be requested for local radio */
	bool               limitedDiscoverable;
} TBlueAPI_RadioModeSetReq;
typedef TBlueAPI_RadioModeSetReq * PBlueAPI_RadioModeSetReq;

typedef struct
{
	/** cmd specific section */
	TBlueAPI_Cause  cause;                  /**< indicates the result of the requested operation.
	                       blueAPI_CauseSuccess       The requested operation is completed successfully
	                       blueAPI_CauseRejected      The requested operation could not be completed due to
	                                                  a conflict with the MDC status (e.g. deep sleep request
	                                                  during an active connection)
	                       blueAPI_CauseNotSupported  The requested mode is not supported. */
} TBlueAPI_RadioModeSetRsp;
typedef TBlueAPI_RadioModeSetRsp * PBlueAPI_RadioModeSetRsp;

typedef enum
{
	blueAPI_BRScanModeNoScan = 0,
	blueAPI_BRScanModeInquiryScan,
	blueAPI_BRScanModePageScan,
	blueAPI_BRScanModeInquiryAndPageScan,
} TBlueAPI_BRPageScanMode;

typedef enum
{
	blueAPI_BRPageScanTypeStandard = 0,
	blueAPI_BRPageScanTypeInterlaced,
} TBlueAPI_BRPageScanType;

typedef enum
{
	blueAPI_BRPageScanRepMode_Continous = 0,
	blueAPI_BRPageScanRepMode_1_28s_Interval,
	blueAPI_BRPageScanRepMode_2_56s_Interval,
	blueAPI_BRPageScanRepMode_Manual = 0xFF,
} TBlueAPI_BRPageScanRepMode;

typedef enum
{
	blueAPI_BRInquiryScanTypeStandard = 0,
	blueAPI_BRInquiryScanTypeInterlaced,
} TBlueAPI_BRInquiryScanType;

typedef enum
{
	blueAPI_BRStandardInquiryResult = 0,
	blueAPI_BRInquiryResultWithRSSI,
	blueAPI_BRExtendedInquiryResult,
} TBlueAPI_BRInquiryMode;

typedef uint8_t TBlueAPI_BRLinkPolicy;

#define blueAPI_BRLinkPolicyDisableAll              0x00
#define blueAPI_BRLinkPolicyEnableRoleSwitch        0x01
#define blueAPI_BRLinkPolicyEnableSniffMode         0x04

typedef enum
{
	blueAPI_BRDeviceRoleDontCare = 0,
	blueAPI_BRDeviceRoleMasterPreferred,
	blueAPI_BRDeviceRoleMasterRequired,
	blueAPI_BRDeviceRoleSlavePreferred,
	blueAPI_BRDeviceRoleSlaveRequired,
} TBlueAPI_BRDeviceRole;

typedef enum
{
    blueAPI_DeviceConfigDeviceName,       /**<  Response to blueAPI_DeviceConfigDeviceNameSetReq.  */
    blueAPI_DeviceConfigAppearance,       /**<  Response to blueAPI_DeviceConfigAppearanceSetReq.  */
    blueAPI_DeviceConfigPerPrefConnParam, /**<  Response to blueAPI_DeviceConfigPerPrefConnParamSetReq.  */
    blueAPI_DeviceConfigSecurity,         /**<  Response to blueAPI_DeviceConfigSecuritySetReq.  */
    blueAPI_DeviceConfigStore,             /**<  Response to blueAPI_DeviceConfigStoreSetReq.  */
    
	blueAPI_DeviceConfigDevice = 0x10,
	blueAPI_DeviceConfigDID,
	blueAPI_DeviceConfigExtraEIR,
	blueAPI_DeviceConfigL2CAP,
	blueAPI_DeviceConfigPagescan,
	blueAPI_DeviceConfigInquiryscan,
	blueAPI_DeviceConfigInquirymode,
	blueAPI_DeviceConfigLinkpolicy,
	blueAPI_DeviceConfigMaxTxPower,
} TBlueAPI_DeviceConfigOpcode;

typedef struct
{
	uint32_t  classOfDevice;
	uint8_t   deviceName[BLUE_API_DEVICE_NAME_LENGTH];
} TBlueAPI_DeviceConfigDevice;

typedef struct
{
	uint16_t  vendorID;
	uint16_t  vendorIDSource;
	uint16_t  productID;
	uint16_t  productVersion;
} TBlueAPI_DeviceConfigDID;

typedef struct
{
	uint8_t   *pdata;
} TBlueAPI_DeviceConfigExtraEIR;

typedef struct
{
	TBlueAPI_BRPageScanType     scanType;
	TBlueAPI_BRPageScanRepMode  repMode;
	uint16_t                    repInterval;
	uint16_t                    repWindow;
	uint16_t                    pageTimeout;
} TBlueAPI_DeviceConfigPagescan;

typedef struct
{
	TBlueAPI_BRInquiryScanType  scanType;
	uint16_t                    interval;
	uint16_t                    window;
} TBlueAPI_DeviceConfigInquiryscan;

typedef struct
{
	TBlueAPI_BRInquiryMode  mode;
} TBlueAPI_DeviceConfigInquirymode;

typedef struct
{
	TBlueAPI_BRLinkPolicy       linkPolicy;
	TBlueAPI_BRDeviceRole       deviceRole;
	uint16_t                    supervisionTimeout;
} TBlueAPI_DeviceConfigLinkpolicy;

typedef struct
{
	int16_t txPower;
} TBlueAPI_DeviceConfigMaxTxPower;

typedef struct
{
    uint8_t   deviceName[BLUE_API_DEVICE_NAME_LENGTH];
} TBlueAPI_DeviceConfigDeviceName;

typedef struct
{
    uint16_t   appearance;
} TBlueAPI_DeviceConfigAppearance;

typedef struct
{
    uint16_t   connIntervalMin;
    uint16_t   connIntervalMax;
    uint16_t   slaveLatency;
    uint16_t   supervisionTimeout;
} TBlueAPI_DeviceConfigPerPrefConnParam;

#define BLUE_API_USE_LE_FIXED_DISPLAY_VALUE 0x80000000

typedef struct
{
	uint32_t                leFixedDisplayValue;
} TBlueAPI_DeviceConfigSecurity;

typedef struct
{
    TBlueAPI_StoreBondModes storeBondMode;
    uint8_t                 storeBondSize;
} TBlueAPI_DeviceConfigStore;

typedef struct
{
    /** cmd specific section */
    TBlueAPI_DeviceConfigOpcode         opCode;
    union 
    {
        TBlueAPI_DeviceConfigDevice       dev;
        TBlueAPI_DeviceConfigDID          did;
        TBlueAPI_DeviceConfigExtraEIR     extraEIR;
        TBlueAPI_DeviceConfigPagescan     pagescan;
        TBlueAPI_DeviceConfigInquiryscan  inquiryscan;
        TBlueAPI_DeviceConfigInquirymode  inquirymode;
        TBlueAPI_DeviceConfigLinkpolicy   linkpolicy;
        TBlueAPI_DeviceConfigMaxTxPower   txpower;

        TBlueAPI_DeviceConfigDeviceName   device;
        TBlueAPI_DeviceConfigAppearance appearance;
        TBlueAPI_DeviceConfigPerPrefConnParam conn;
        TBlueAPI_DeviceConfigSecurity     security;
        TBlueAPI_DeviceConfigStore        store;
    } p;
} TBlueAPI_DeviceConfigSetReq;
typedef TBlueAPI_DeviceConfigSetReq * PBlueAPI_DeviceConfigSetReq;

typedef struct
{
	/** cmd specific section */
	TBlueAPI_DeviceConfigOpcode opCode;
	TBlueAPI_Cause              cause;
} TBlueAPI_DeviceConfigSetRsp;
typedef TBlueAPI_DeviceConfigSetRsp * PBlueAPI_DeviceConfigSetRsp;

typedef enum
{
	blueAPI_ACLConfigLinkpolicy,
	blueAPI_ACLConfigSniffmode,
	blueAPI_ACLConfigLinkstatus,
} TBlueAPI_ACLConfigOpcode;

typedef struct
{
	TBlueAPI_BRLinkPolicy       linkPolicy;
	TBlueAPI_BRDeviceRole       deviceRole;
	uint16_t                    supervisionTimeout;
} TBlueAPI_ACLConfigParamLinkpolicy;
typedef TBlueAPI_ACLConfigParamLinkpolicy * PBlueAPI_ACLConfigParamLinkpolicy;

typedef struct
{
	uint16_t  minInterval;
	uint16_t  maxInterval;
	uint16_t  sniffAttempt;
	uint16_t  sniffTimeout;
	uint16_t  maxLatency;
	uint16_t  minRemoteTimeout;
	uint16_t  minLocalTimeout;
} TBlueAPI_ACLConfigParamSniffmode;
typedef TBlueAPI_ACLConfigParamSniffmode * PBlueAPI_ACLConfigParamSniffmode;

typedef struct
{
	uint16_t  pollInterval;
} TBlueAPI_ACLConfigParamLinkstatus;
typedef TBlueAPI_ACLConfigParamLinkstatus * PBlueAPI_ACLConfigParamLinkstatus;

typedef union
{
	TBlueAPI_ACLConfigParamLinkpolicy linkpolicy;
	TBlueAPI_ACLConfigParamSniffmode  sniffmode;
	TBlueAPI_ACLConfigParamLinkstatus linkstatus;
} TBlueAPI_ACLConfigParam;
typedef TBlueAPI_ACLConfigParam * PBlueAPI_ACLConfigParam;

typedef struct
{
	/** cmd specific section */
	uint8_t                  remote_BD[BLUE_API_BD_SIZE];
	TBlueAPI_RemoteBDType    remote_BD_Type;

	TBlueAPI_ACLConfigOpcode opCode;
	TBlueAPI_ACLConfigParam  p;
} TBlueAPI_ACLConfigReq;
typedef TBlueAPI_ACLConfigReq * PBlueAPI_ACLConfigReq;

typedef struct
{
	/** cmd specific section */
	uint8_t                  remote_BD[BLUE_API_BD_SIZE];
	TBlueAPI_RemoteBDType    remote_BD_Type;

	TBlueAPI_ACLConfigOpcode opCode;
	TBlueAPI_Cause           cause;
} TBlueAPI_ACLConfigRsp;
typedef TBlueAPI_ACLConfigRsp * PBlueAPI_ACLConfigRsp;

/** used by the MDC to respond to a PairableModeSetReq of the MDH */
typedef struct
{
	/** cmd specific section */
	TBlueAPI_Cause     cause;                           /**< indicates the result of the requested operation.
	                                  blueAPI_CauseSuccess           - Operation completed successfully
	                                  blueAPI_CauseInvalidState      - Operation can not be performed due to invalid state
	                                  blueAPI_CauseInvalidParameter  - Indicates that the authentication code was detected as invalid by the remote device */
} TBlueAPI_PairableModeSetRsp;
typedef TBlueAPI_PairableModeSetRsp * PBlueAPI_PairableModeSetRsp;

/** used by the MDH to initiate an authentication with a given remote device. */
typedef struct
{
	/** cmd specific section */
	uint8_t              remote_BD[BLUE_API_BD_SIZE];           /**< Bluetooth device address of remote device */
} TBlueAPI_AuthReq;
typedef TBlueAPI_AuthReq * PBlueAPI_AuthReq;

/** used by the MDC to respond to an AuthReq of the MDH */
typedef struct
{
	/** cmd specific section */
	uint8_t                 remote_BD[BLUE_API_BD_SIZE];         /**< Bluetooth device address of corresponding request */
	TBlueAPI_Cause          cause;                /**< indicates the result of the requested operation.
	                                  blueAPI_CauseSuccess           - Operation completed successfully
	                                  blueAPI_CauseReject            - Indicates that the remote device rejects the authentication attempt
	                                  blueAPI_CauseInvalidParameter  - Indicates that the authentication code was detected as invalid by the remote device
                                      blueAPI_CauseConnectionLost    - Indicates that the connection attempt failed (e.g. device was out of range) */
} TBlueAPI_AuthRsp;
typedef TBlueAPI_AuthRsp * PBlueAPI_AuthRsp;

/** used by MDC to initiate an authentication with a given remote device. */
typedef struct
{
    /** cmd specific section */
    uint8_t         outgoing;                       /**< 1:outgoing, 0:incoming */
    uint16_t        psm;                            /**< authorize protocol psm */
    uint16_t        server_channel;                 /**< server channel for rfcomm, 0 for l2cap */
    uint16_t        uuid;                           /**< uuid to connect, used for outgoing */
    uint8_t         remote_BD[BLUE_API_BD_SIZE];    /**< Bluetooth device address of corresponding request */
} TBlueAPI_UserAuthorizationReqInd;
typedef TBlueAPI_UserAuthorizationReqInd * PBlueAPI_UserAuthorizationReqInd;

/** used by MDC to indicate the requirement for a non SSP user level authentication via PIN entry (legacy pairing).
This authentication might be either triggered by the local MDC or the remote device */
typedef struct
{
	/** cmd specific section */
	uint8_t            remote_BD[BLUE_API_BD_SIZE];           /**< Bluetooth device address of remote device */
} TBlueAPI_UserAuthRequestInd;
typedef TBlueAPI_UserAuthRequestInd * PBlueAPI_UserAuthRequestInd;

/** used by MDC to indicate the requirement for a external stored link key.
This authentication might be either triggered by the local MDC or the remote device */
typedef struct
{
	/** cmd specific section */
	uint8_t               remote_BD[BLUE_API_BD_SIZE];         /**< Bluetooth device address of remote device */
	TBlueAPI_RemoteBDType remote_BD_Type;
	TBlueAPI_LinkKeyType  keyType;
	uint16_t              restartHandle;
} TBlueAPI_AuthResultRequestInd;
typedef TBlueAPI_AuthResultRequestInd * PBlueAPI_AuthResultRequestInd;

/** used by the MDH to respond to an AuthResultRequestInd of the MDC. */
typedef struct
{
	/** cmd specific section */
	uint8_t               remote_BD[BLUE_API_BD_SIZE];         /**< Bluetooth device address of remote device        */
	TBlueAPI_RemoteBDType remote_BD_Type;
	uint8_t               linkKeyLength;
	uint8_t               linkKey[28];          /**< Link Key */
	TBlueAPI_LinkKeyType  keyType;
	uint16_t              restartHandle;
	TBlueAPI_Cause        cause;                /**< indicates the result of the authentication.
	                                  blueAPI_CauseAccept    Indicates that the MDH accepts the authentication attempt and the AuthCode in this message can be used for authentication purposes
                                      blueAPI_CauseReject    Indicates that the MDH rejects the authentication attempt */
} TBlueAPI_AuthResultRequestConf;
typedef TBlueAPI_AuthResultRequestConf * PBlueAPI_AuthResultRequestConf;

/** used by the MDH to respond to an UserAuthRequestInd of the MDC. */
typedef struct
{
	/** cmd specific section */
	uint8_t               remote_BD[BLUE_API_BD_SIZE];         /**< Bluetooth device address of remote device        */
	uint8_t               AuthCodeLength;       /**< valid length of Binary authentication code */
	uint8_t               AuthCode[16];         /**< Binary authentication code provided by user interaction */
	TBlueAPI_Cause        cause;                /**< indicates the result of the authentication.
	                                  blueAPI_CauseAccept    Indicates that the MDH accepts the authentication attempt and the AuthCode in this message can be used for authentication purposes
                                      blueAPI_CauseReject    Indicates that the MDH rejects the authentication attempt */
} TBlueAPI_UserAuthRequestConf;
typedef TBlueAPI_UserAuthRequestConf * PBlueAPI_UserAuthRequestConf;

/** If a SSP procedure is started that requires "Display Yes/No" functionality the MDC will send this message to the
MDH to request a value to be displayed as a 6 digit decimal value to the User and wait for a User interaction */
typedef struct
{
	/** cmd specific section */
	uint8_t            remote_BD[BLUE_API_BD_SIZE];         /**< Bluetooth device address of remote device      */
	uint32_t           displayValue;         /**< Value to be displayed as a 6 digit decimal number */
} TBlueAPI_UserConfirmationReqInd;
typedef TBlueAPI_UserConfirmationReqInd * PBlueAPI_UserConfirmationReqInd;

/* used by MDH to respond to authorization request indication */
typedef struct
{
  /* cmd specific section */
  uint8_t                 remote_BD[BLUE_API_BD_SIZE];         /* Bluetooth device address of corresponding request */
  TBlueAPI_Cause          cause;                /* indicates the result of the requested operation.
                                      blueAPI_CauseAccept    Indicates that the MDH accepts the authentication attempt
                                      blueAPI_CauseReject    Indicates that the MDH rejects the authentication attempt */
} TBlueAPI_UserAuthorizationReqConf;
typedef TBlueAPI_UserAuthorizationReqConf * PBlueAPI_UserAuthorizationReqConf;

/** With this message the MDH shall respond to a SSP driven UserConfirmationReqInd as soon as User interaction is performed */
typedef struct
{
	/** cmd specific section */
	uint8_t               remote_BD[BLUE_API_BD_SIZE];         /**< Bluetooth device address of remote device      */
	TBlueAPI_Cause        cause;                /**< indicates the result of the operation.
	                                  blueAPI_CauseAccept  - Indicates that the User accepts the authentication
	                                  blueAPI_CauseReject  - Indicates that the User rejects the authentication */
} TBlueAPI_UserConfirmationReqConf;
typedef TBlueAPI_UserConfirmationReqConf * PBlueAPI_UserConfirmationReqConf;

/** If a SSP procedure is started that requires Keyboard functionality from the remote peer,
and display functionality from the local device the MDC will send this message to the MDH to request User interaction */
typedef struct
{
	/** cmd specific section */
	uint8_t            remote_BD[BLUE_API_BD_SIZE];         /**< Bluetooth device address of remote device      */
} TBlueAPI_UserPasskeyReqInd;
typedef TBlueAPI_UserPasskeyReqInd * PBlueAPI_UserPasskeyReqInd;

/** With this message the MDH shall respond to a SSP driven UserPasskeyReqInd as soon as User interaction is performed */
typedef struct
{
	/** cmd specific section */
	uint8_t               remote_BD[BLUE_API_BD_SIZE];         /**< Bluetooth device address of remote device      */
	TBlueAPI_Cause        cause;                /* indicates the result of the operation.
	                                blueAPI_CauseAccep   - Indicates that the User accepts the authentication
	                                blueAPI_CauseReject  - Indicates that the User rejects the authentication */
} TBlueAPI_UserPasskeyReqConf;
typedef TBlueAPI_UserPasskeyReqConf * PBlueAPI_UserPasskeyReqConf;

/** If a SSP procedure is completed that requires Keyboard functionality from the local,
and display functionality from the remote device the MDH shall send this message to the MDC
to transfer the result of the Keyboard input and request completion of the SSP procedures */
typedef struct
{
	/** cmd specific section */
	uint8_t               remote_BD[BLUE_API_BD_SIZE];         /**< Bluetooth device address of remote device      */
	uint32_t              passKey;              /**< result of Keyboard input                       */
	TBlueAPI_Cause        cause;                /**< indicates the result of the operation.
	                                blueAPI_CauseAccept  - Indicates that the User accepts the authentication
	                                blueAPI_CauseReject  - Indicates that the User rejects the authentication */
} TBlueAPI_UserPasskeyReqReplyReq;
typedef TBlueAPI_UserPasskeyReqReplyReq * PBlueAPI_UserPasskeyReqReplyReq;

/** With this message the MDC will respond to a SSP driven UserPasskeyReqReplyReq message of the MDH. */
typedef struct
{
	/** cmd specific section */
	TBlueAPI_Cause     cause;                /**< indicates the result of the operation.
	                                blueAPI_CauseSuccess       - Indicates that the operation was performed successfully
	                                blueAPI_CauseInvalidState  - Operation can not be performed due to invalid state   */
} TBlueAPI_UserPasskeyReqReplyRsp;
typedef TBlueAPI_UserPasskeyReqReplyRsp * PBlueAPI_UserPasskeyReqReplyRsp;

/** If a SSP procedure is completed that requires Keyboard functionality from the remote
and display functionality from the local device the MDC will send this message to notify
the MDH about the initial state of the display content */
typedef struct
{
	/** cmd specific section */
	uint8_t            remote_BD[BLUE_API_BD_SIZE];         /**< Bluetooth device address of remote device      */
	uint32_t           displayValue;         /* Value to be displayed as a 6 digit decimal number */
} TBlueAPI_UserPasskeyNotificationInfo;
typedef TBlueAPI_UserPasskeyNotificationInfo * PBlueAPI_UserPasskeyNotificationInfo;

/** If a SSP procedure is started that requires Keyboard functionality from the local
and display functionality from the remote device the MDH shall send this message to the MDC to indicate User interaction. */
typedef struct
{
	/** cmd specific section */
	uint8_t               remote_BD[BLUE_API_BD_SIZE];         /**< Bluetooth device address of remote device */
	TBlueAPI_SSPKeyEvent  eventType;            /**< event to be communicated                  */
} TBlueAPI_KeypressNotificationReq;
typedef TBlueAPI_KeypressNotificationReq * PBlueAPI_KeypressNotificationReq;

/** With this message the MDC will respond to a SSP driven KeypressNotificationReq message of the MDH */
typedef struct
{
	/** cmd specific section */
	TBlueAPI_Cause     cause;                /**< indicates the result of the operation.
	                                blueAPI_CauseSuccess       - Indicates that the User accepts the authentication
	                                blueAPI_CauseInvalidState  - Operation can not be performed due to invalid state */
} TBlueAPI_KeypressNotificationRsp;
typedef TBlueAPI_KeypressNotificationRsp * PBlueAPI_KeypressNotificationRsp;

/** If a SSP procedure is in progress that requires Keyboard functionality from the remote
and display functionality from the local device the MDC will send this message to notify the MDH about User interaction. */
typedef struct
{
	/** cmd specific section */
	uint8_t               remote_BD[BLUE_API_BD_SIZE];         /**< Bluetooth device address of remote device      */
	TBlueAPI_SSPKeyEvent  eventType;            /**< event to be communicated                       */
} TBlueAPI_KeypressNotificationInfo;
typedef TBlueAPI_KeypressNotificationInfo * PBlueAPI_KeypressNotificationInfo;

/** If a SSP procedure is started that requires OOB functionality from the local and remote device
the MDC will send this message to the MDH to request OOB data from the remote device via OOB communication. */
typedef struct
{
	/** cmd specific section */
	uint8_t     remote_BD[BLUE_API_BD_SIZE];                   /**< Bluetooth device address of remote device      */
} TBlueAPI_RemoteOOBDataReqInd;
typedef TBlueAPI_RemoteOOBDataReqInd * PBlueAPI_RemoteOOBDataReqInd;

/** With this message the MDH shall respond to a SSP driven RemoteOOBDataReqInd message of the MDC. */
typedef struct
{
	/** cmd specific section */
	uint8_t               remote_BD[BLUE_API_BD_SIZE];         /**< Bluetooth device address of remote device      */
	uint8_t               C[16];                /**< Hash C value                                   */
	TBlueAPI_Cause        cause;                /**< indicates the result of the operation.
	                                blueAPI_CauseAccept  - Indicates that the User accepts the authentication
	                                blueAPI_CauseReject  - Indicates that the User rejects the authentication */
} TBlueAPI_RemoteOOBDataReqConf;
typedef TBlueAPI_RemoteOOBDataReqConf * PBlueAPI_RemoteOOBDataReqConf;

typedef struct
{
	/** cmd specific section */
	uint8_t     remote_BD[BLUE_API_BD_SIZE];                   /**< Bluetooth device address of remote device      */
} TBlueAPI_LegacyRemoteOOBDataReqInd;
typedef TBlueAPI_LegacyRemoteOOBDataReqInd * PBlueAPI_LegacyRemoteOOBDataReqInd;

/** With this message the MDH shall respond to a SSP driven RemoteOOBDataReqInd message of the MDC. */
typedef struct
{
	/** cmd specific section */
	uint8_t               remote_BD[BLUE_API_BD_SIZE];         /**< Bluetooth device address of remote device      */
	uint8_t               C[16];                /**< Hash C value                                   */
	uint8_t               R[16];                /**< Randomizer R value                             */
	TBlueAPI_Cause        cause;                /**< indicates the result of the operation.
	                                blueAPI_CauseAccept  - Indicates that the User accepts the authentication
	                                blueAPI_CauseReject  - Indicates that the User rejects the authentication */
} TBlueAPI_LegacyRemoteOOBDataReqConf;
typedef TBlueAPI_LegacyRemoteOOBDataReqConf * PBlueAPI_LegacyRemoteOOBDataReqConf;

/** If a SSP procedure is started that requires OOB functionality from the local and remote device
the MDH can send this message to the MDC to request OOB data from the local device to be transferred via OOB
communication to the remote device for authentication. */
typedef struct
{
	/** generic cmd section  */
	TBlueAPIHandle     blueAPIHandle;
} TBlueAPI_LocalOOBDataReq;
typedef TBlueAPI_LocalOOBDataReq * PBlueAPI_LocalOOBDataReq;

/** With this message the MDC will respond to a SSP driven LocalOOBDataReq message of the MDH. */
typedef struct
{
	/** cmd specific section */
	uint8_t               C[16];                /**< Hash C value                                   */
	uint8_t               R[16];                /**< Randomizer R value                             */
	TBlueAPI_Cause        cause;                /**< indicates the result of the operation.
	                                blueAPI_CauseSuccess               - Indicates that the operation was performed successfully
	                                blueAPI_CauseNotSuppoerted         - Indicates that the OOB data can not be supplied */
} TBlueAPI_LocalOOBDataRsp;
typedef TBlueAPI_LocalOOBDataRsp * PBlueAPI_LocalOOBDataRsp;

/** This command is used by the MDC to indicate the resulting information of a performed authentication. */
typedef struct
{
	/** cmd specific section */
	uint8_t               remote_BD[BLUE_API_BD_SIZE];         /**< Bluetooth device address of remote device      */
	TBlueAPI_RemoteBDType remote_BD_Type;
	uint8_t               linkKeyLength;
    TBlueAPI_LinkKeyType  keyType;
	uint8_t               linkKey[28];          /**< Link Key */
	TBlueAPI_Cause        cause;                /**< indicates the result of the authentication.
	                                blueAPI_CauseSuccess    Authentication is completed successfully
	                                blueAPI_CauseReject    Indicates that the remote device rejects the authentication attempt
	                                blueAPI_CauseInvalidParameter    Indicates that the authentication code was detected as invalid by the remote device
	                                blueAPI_CauseConnectionLost    Indicates that the connection attempt failed (e.g. device was out of range) */
} TBlueAPI_AuthResultInd;
typedef TBlueAPI_AuthResultInd * PBlueAPI_AuthResultInd;

/** This command is used by the MDH to respond to an AuthResultInd of the MDC.
In case the authentication was indicated to be successfully, the response can be used to assign MDH specific information to a given bond table entry. */
typedef struct
{
	/** cmd specific section */
	uint8_t               remote_BD[BLUE_API_BD_SIZE];         /**< Bluetooth device address of remote device      */
	TBlueAPI_RemoteBDType remote_BD_Type;
	TBlueAPI_Cause        cause;                /**< indicates the result of the trasaction
	                                blueAPI_CauseSuccess    Operation is completed successfully */
} TBlueAPI_AuthResultConf;
typedef TBlueAPI_AuthResultConf * PBlueAPI_AuthResultConf;

/** This command is used by the MDH to delete a formally established trusted relation to a remote device (bond table entry). */
typedef struct
{
	/** cmd specific section */
	uint8_t               remote_BD[BLUE_API_BD_SIZE];         /**< Bluetooth device address of remote device. If this parameter is set to ?ll zero?
	                                all trusted relations will be deleted. */
	TBlueAPI_RemoteBDType remote_BD_Type;
} TBlueAPI_AuthDeleteReq;
typedef TBlueAPI_AuthDeleteReq * PBlueAPI_AuthDeleteReq;

/** With this message the MDC responds to an AuthDeleteReq message. */
typedef struct
{
	/** cmd specific section */
	uint8_t               remote_BD[BLUE_API_BD_SIZE];         /**< Bluetooth device address from corresponding request  */
	TBlueAPI_RemoteBDType remote_BD_Type;
	TBlueAPI_Cause        cause;                /**< indicates the result of the authentication
	                                blueAPI_CauseSuccess           - Operation completed successfully
	                                blueAPI_CauseInvalidParameter  - Indicates that no trusted relation can be identified for the specified device.  */
} TBlueAPI_AuthDeleteRsp;
typedef TBlueAPI_AuthDeleteRsp * PBlueAPI_AuthDeleteRsp;

/** This command is used by the MDH to list all formally established trusted relations to remote devices (bond table entries). */
typedef struct
{
	/** cmd specific section */
	uint8_t               remote_BD[BLUE_API_BD_SIZE];         /**< Bluetooth device address of remote device. If this parameter is set to ?ll zero?
	                                all trusted relations will be listed. */
	TBlueAPI_RemoteBDType remote_BD_Type;
} TBlueAPI_AuthListReq;
typedef TBlueAPI_AuthListReq * PBlueAPI_AuthListReq;

/** This command is used by the MDC to indicate a single entry of the MDC internal list of authenticated devices.
Please be aware that multiple of this AuthListInfo messages might be generated as a result of a AuthListReq. */
typedef struct
{
	/** cmd specific section */
	uint8_t               remote_BD[BLUE_API_BD_SIZE];         /**< Bluetooth device address of remote device      */
	TBlueAPI_RemoteBDType remote_BD_Type;
	TBlueAPI_LinkKeyType  keyType;
	uint32_t              AppData;              /**< This parameter indicates MDH assigned data of the given bond table entry. */
	uint8_t               Remote_DeviceName[BLUE_API_DEVICE_NAME_LENGTH]; /**< UTF-8 coded, zero terminated string of octets including the user friendly name of the remote device.
	                                Note that this field may be empty. The maximum size of this field is defined with the BLUE_API_DEVICE_NAME_LENGTH constant definition. */
} TBlueAPI_AuthListInfo;
typedef TBlueAPI_AuthListInfo * PBlueAPI_AuthListInfo;

/** With this message the MDC responds to an AuthListReq message. */
typedef struct
{
	/** cmd specific section */
	uint8_t               remote_BD[BLUE_API_BD_SIZE];         /**< Bluetooth device address of corresponding request */
	TBlueAPI_RemoteBDType remote_BD_Type;
	TBlueAPI_Cause        cause;                /**< indicates the result of the operation
	                                blueAPI_CauseSuccess           - Operation completed successfully
                                    	blueAPI_CauseInvalidParameter  - Indicates that no trusted relation can be identified for the specified device. */

} TBlueAPI_AuthListRsp;
typedef TBlueAPI_AuthListRsp * PBlueAPI_AuthListRsp;

/** service register ---*/
typedef struct
{
	uint16_t    nbrOfAttrib;
	void *      pService;
} TBlueAPI_GATTServiceRegisterReq;
typedef TBlueAPI_GATTServiceRegisterReq * PBlueAPI_GATTServiceRegisterReq;

typedef struct
{
	/** cmd specific section */
	void *         serviceHandle;
	TBlueAPI_Cause cause;
	uint16_t       subCause;           /**< cause from protocol layers */
} TBlueAPI_GATTServiceRegisterRsp;
typedef TBlueAPI_GATTServiceRegisterRsp * PBlueAPI_GATTServiceRegisterRsp;

/** attribute update (trigger indication/notification) ---*/
typedef struct
{
	/** cmd specific section */
	void *         serviceHandle;
	void *         requestHandle;
	uint16_t       attribIndex;
	uint16_t       attribLength;
	uint16_t       gap;                /**< offset of attrib. value in data[] */
	uint8_t        data[1];
} TBlueAPI_GATTAttributeUpdateReq;
typedef TBlueAPI_GATTAttributeUpdateReq * PBlueAPI_GATTAttributeUpdateReq;

typedef struct
{
	/** cmd specific section */
	void *         serviceHandle;
	void *         requestHandle;
	uint16_t       attribIndex;
	TBlueAPI_Cause cause;
	uint16_t       subCause;       /**< cause from protocol layers */
	uint16_t       count;          /**< nbr. of elements in list (<> 0 only for */
	                              /**< some cause values ..)                   */
	uint16_t       gap;            /**< offset of list in list[]                */
	uint8_t        list[1];        /**< <BD,type>, depending on cause value     */
} TBlueAPI_GATTAttributeUpdateRsp;
typedef TBlueAPI_GATTAttributeUpdateRsp * PBlueAPI_GATTAttributeUpdateRsp;

typedef struct     /**< list element in TBlueAPI_GATTAttributeUpdateRsp */
{
	uint8_t remote_BD[BLUE_API_BD_SIZE];
	uint8_t remote_BD_Type;
} TBlueAPI_GATTAttributeUpdateListElement, * PBlueAPI_GATTAttributeUpdateListElement;

typedef struct
{
	/** cmd specific section */
	void *         serviceHandle;
	void *         requestHandle;
	uint16_t       attribIndex;
	TBlueAPI_Cause cause;
	uint16_t       subCause;
	uint8_t                 remote_BD[BLUE_API_BD_SIZE];
	TBlueAPI_RemoteBDType   remote_BD_Type;
} TBlueAPI_GATTAttributeUpdateStatusInd;
typedef TBlueAPI_GATTAttributeUpdateStatusInd * PBlueAPI_GATTAttributeUpdateStatusInd;

typedef struct
{
	/** cmd specific section */
	void *         serviceHandle;
	void *         requestHandle;
	uint16_t       attribIndex;
} TBlueAPI_GATTAttributeUpdateStatusConf;
typedef TBlueAPI_GATTAttributeUpdateStatusConf * PBlueAPI_GATTAttributeUpdateStatusConf;

/** server application supplied attribute read ---*/
typedef struct
{
	/** cmd specific section */
	uint16_t       local_MDL_ID;
	void *         serviceHandle;
	uint16_t       attribIndex;
	uint16_t       readOffset;         /**< read offset in attribute */
} TBlueAPI_GATTAttributeReadInd;
typedef TBlueAPI_GATTAttributeReadInd * PBlueAPI_GATTAttributeReadInd;

typedef struct
{
	/** cmd specific section */
	void *         serviceHandle;
	uint16_t       local_MDL_ID;
	TBlueAPI_Cause cause;
	uint16_t       subCause;
	uint16_t       attribIndex;
	uint16_t       attribLength;
	uint16_t       gap;                /**< offset of attrib. value in data[] */
	uint8_t        data[1];
} TBlueAPI_GATTAttributeReadConf;
typedef TBlueAPI_GATTAttributeReadConf * PBlueAPI_GATTAttributeReadConf;

/**- server application supplied attribute write with response ---*/
typedef struct
{
	/** cmd specific section */
	uint16_t       local_MDL_ID;
	void *         serviceHandle;
	uint16_t       attribIndex;
	uint16_t       attribLength;
	uint16_t       handle;  
	uint16_t       writeOffset;        /**< write offset in attribute */
	uint16_t       gap;                /**< offset of attrib. value in data[] */
	uint8_t        data[1];
} TBlueAPI_GATTAttributeWriteInd;
typedef TBlueAPI_GATTAttributeWriteInd * PBlueAPI_GATTAttributeWriteInd;

typedef struct
{
	/** cmd specific section */
	uint16_t       local_MDL_ID;
	void *         serviceHandle;
	TBlueAPI_Cause cause;
	uint16_t       subCause;
	uint16_t       attribIndex;
} TBlueAPI_GATTAttributeWriteConf;
typedef TBlueAPI_GATTAttributeWriteConf * PBlueAPI_GATTAttributeWriteConf;

/** server application supplied attribute write without response ---*/
typedef TBlueAPI_GATTAttributeWriteInd  TBlueAPI_GATTAttributeWriteCommandInfo;
typedef TBlueAPI_GATTAttributeWriteCommandInfo * PBlueAPI_GATTAttributeWriteCommandInfo;


typedef struct
{
    void *         serviceHandle;
    uint16_t       local_MDL_ID;
    TBlueAPI_Cause cause;
    uint16_t       subCause;
    uint16_t       attribIndex;
    uint16_t       attribLength;
    uint16_t       gap;
    uint8_t        data[1];
} TBlueAPI_GATTAttributePrepareWriteConf;
typedef TBlueAPI_GATTAttributePrepareWriteConf * PBlueAPI_GATTAttributePrepareWriteConf;

/**
 * @brief  blueAPI_EventGATTAttributeExecuteWriteInd command data.
 *
 *  server application supplied attribute execute write with response
 *  triggered by ATT_EXECUTE_WRITE_REQUEST)
 */
typedef struct
{
    uint16_t       local_MDL_ID;       /**<  local link ID for this link. */
    uint8_t        flags;              /**<  flags:0x00-Cancel all prepared writes
                                                                                      0x01-Immediately write all pending prepared values. */
} TBlueAPI_GATTAttributeExecuteWriteInd;
typedef TBlueAPI_GATTAttributeExecuteWriteInd * PBlueAPI_GATTAttributeExecuteWriteInd;

typedef struct
{
    uint16_t       local_MDL_ID;
    TBlueAPI_Cause cause;
    uint16_t       subCause;
    uint16_t       handle;
} TBlueAPI_GATTAttributeExecuteWriteConf;
typedef TBlueAPI_GATTAttributeExecuteWriteConf * PBlueAPI_GATTAttributeExecuteWriteConf;

/**
 * @brief  blueAPI_EventGATTAttributePrepareWriteRsp command data.
 *
 * used to signal the result of a GATTAttributePrepareWriteReq to the MDH.
 */
typedef struct
{
    uint16_t       local_MDL_ID;         /**<  local link ID.  */ 
    TBlueAPI_Cause cause;                /**<  Indicates the result of the transaction.  */ 
    uint16_t       subCause;             /**<  More detailed result information from lower protocol layers. */ 
    uint16_t       writeOffset;          /**<  The offset of the first octet to be written.  */
    uint16_t       attribLength;         /**<  attrib. value length.  */ 
    uint16_t       gap;                  /**<  offset of attrib. value in data[].  */
    uint8_t        data[1];              /**<  attrib. value in data[].  */
} TBlueAPI_GATTAttributePrepareWriteRsp;
typedef TBlueAPI_GATTAttributePrepareWriteRsp * PBlueAPI_GATTAttributePrepareWriteRsp;

typedef struct
{
    uint16_t       local_MDL_ID;
    uint8_t        flags;
} TBlueAPI_GATTAttributeExecuteWriteReq;
typedef TBlueAPI_GATTAttributeExecuteWriteReq * PBlueAPI_GATTAttributeExecuteWriteReq;

/**
 * @brief  blueAPI_EventGATTAttributeExecuteWriteRsp command data.
 *
 * used to signal the result of a GATTAttributeExecuteWriteReq to the MDH. 
 */
typedef struct
{
    uint16_t       local_MDL_ID;      /**<  local link ID.  */   
    TBlueAPI_Cause        cause;      /**<  Indicates the result of the transaction.  */           
    uint16_t       subCause;          /**<  More detailed result information from lower protocol layers. */   
} TBlueAPI_GATTAttributeExecuteWriteRsp;
typedef TBlueAPI_GATTAttributeExecuteWriteRsp * PBlueAPI_GATTAttributeExecuteWriteRsp;

//#endif

/** client characteristic configuration descriptor (CCCD) values ---*/
typedef struct
{
	/** cmd specific section */
	uint16_t       local_MDL_ID;
	void *         serviceHandle;
	uint16_t       count;              /**< nbr. of attribIndex/CCCD value pairs */
	uint16_t       gap;                /**< offset of first pair in data[]       */
	uint8_t        data[1];
} TBlueAPI_GATTCCCDInfo;
typedef TBlueAPI_GATTCCCDInfo * PBlueAPI_GATTCCCDInfo;

/** GATT client application messages -------------*/
/** service, characteristics, descriptors and relationship discovery ---*/
typedef enum              /**< discovery types */
{
	blueAPI_GATTDiscoveryServices=0x01,       /**< all primary services */
	blueAPI_GATTDiscoveryServiceByUUID,       /**< service by UUID      */
	blueAPI_GATTDiscoveryCharacteristics,     /**< all characteristics  */
	blueAPI_GATTDiscoveryCharacDescriptors,   /**< all characteristic descriptors   */
	blueAPI_GATTDiscoveryRelationship         /**< relationship (included services) */
} TBlueAPI_GATTDiscoveryType;

typedef struct
{
	/** cmd specific section */
	uint16_t       local_MDL_ID;
	TBlueAPI_GATTDiscoveryType   discoveryType;
	uint16_t       startHandle;
	uint16_t       endHandle;
	uint16_t       UUID16;
	uint8_t        UUID128[16];
} TBlueAPI_GATTDiscoveryReq;
typedef TBlueAPI_GATTDiscoveryReq * PBlueAPI_GATTDiscoveryReq;

typedef struct
{
	/** cmd specific section */
	uint16_t       local_MDL_ID;
	TBlueAPI_GATTDiscoveryType   discoveryType;
	TBlueAPI_Cause cause;
	uint16_t       subCause;
} TBlueAPI_GATTDiscoveryRsp;
typedef TBlueAPI_GATTDiscoveryRsp * PBlueAPI_GATTDiscoveryRsp;

/** TBlueAPI_GATTDiscoveryInd list elements, some discovery type results */
/** share the same generic list element format:                          */
typedef struct
{
	uint16_t   attHandle;
	uint16_t   endGroupHandle;
	uint16_t   UUID16;                /**< 16 bit UUID */
} TBlueAPI_GATTGenericElement16, * PBlueAPI_GATTGenericElement16;

typedef struct
{
	uint16_t   attHandle;
	uint16_t   endGroupHandle;
	uint8_t    UUID128[16];           /**< 128 bit UUID */
} TBlueAPI_GATTGenericElement128, * PTBlueAPI_GATTGenericElement128;

/** discoveryType = blueAPI_GATTDiscoveryServices */
typedef TBlueAPI_GATTGenericElement16        TBlueAPI_GATTServicesElement16;
typedef TBlueAPI_GATTServicesElement16  * PBlueAPI_GATTServicesElement16;

typedef TBlueAPI_GATTGenericElement128       TBlueAPI_GATTServicesElement128;
typedef TBlueAPI_GATTServicesElement128 * PBlueAPI_GATTServicesElement128;

/** discoveryType = blueAPI_GATTDiscoveryServiceByUUID */
typedef struct
{
	uint16_t   attHandle;
	uint16_t   endGroupHandle;
} TBlueAPI_GATTServiceByUUIDElement, * PBlueAPI_GATTServiceByUUIDElement;

/** discoveryType = blueAPI_GATTDiscoveryCharacteristics */
typedef struct
{
	uint16_t   declHandle;
	uint16_t   properties;            /**< high nibble is reserved */
	uint16_t   valueHandle;
	uint16_t   UUID16;                /**< 16 bit UUID */
} TBlueAPI_GATTCharacteristicsElement16, * PBlueAPI_GATTCharacteristicsElement16;

typedef struct
{
	uint16_t   declHandle;
	uint16_t   properties;            /**< high nibble is reserved */
	uint16_t   valueHandle;
	uint8_t    UUID128[16];           /**< 128 bit UUID */
} TBlueAPI_GATTCharacteristicsElement128, * PBlueAPI_GATTCharacteristicsElement128;

/** discoveryType = blueAPI_GATTDiscoveryCharacDescriptors */
typedef struct
{
	uint16_t   handle;
	uint16_t   UUID16;                /**< 16 bit UUID */
} TBlueAPI_GATTCharacDescriptorsElement16, * PBlueAPI_GATTCharacDescriptorsElement16;

typedef struct
{
	uint16_t   handle;
	uint8_t    UUID128[16];           /**< 128 bit UUID */
} TBlueAPI_GATTCharacDescriptorsElement128, * PBlueAPI_GATTCharacDescriptorsElement128;

/** discoveryType = blueAPI_GATTDiscoveryRelationship */
typedef struct
{
	uint16_t   declHandle;
	uint16_t   attHandle;
	uint16_t   endGroupHandle;
	uint16_t   UUID16;                /**< 16 bit UUID */
} TBlueAPI_GATTRelationshipElement16, * PBlueAPI_GATTRelationshipElement16;

typedef struct
{
	uint16_t   declHandle;
	uint16_t   attHandle;
	uint16_t   endGroupHandle;
	uint8_t    UUID128[16];           /**< 128 bit UUID */
} TBlueAPI_GATTRelationshipElement128, * PBlueAPI_GATTRelationshipElement128;

typedef union   /**< list element overlay */
{
	TBlueAPI_GATTGenericElement16             generic16[1];
	TBlueAPI_GATTGenericElement128            generic128[1];
	TBlueAPI_GATTServicesElement16            services16[1];
	TBlueAPI_GATTServicesElement128           services128[1];
	TBlueAPI_GATTServiceByUUIDElement         serviceByUUID[1];
	TBlueAPI_GATTCharacteristicsElement16     characteristics16[1];
	TBlueAPI_GATTCharacteristicsElement128    characteristics128[1];
	TBlueAPI_GATTCharacDescriptorsElement16   characDescriptors16[1];
	TBlueAPI_GATTCharacDescriptorsElement128  characDescriptors128[1];
	TBlueAPI_GATTRelationshipElement16        relationship16[1];
	TBlueAPI_GATTRelationshipElement128       relationship128[1];
} TBlueAPI_GATTDiscoveryListElement, * PBlueAPI_GATTDiscoveryListElement;

typedef struct
{
	/** cmd specific section */
	uint16_t       local_MDL_ID;
	TBlueAPI_GATTDiscoveryType   discoveryType;
	TBlueAPI_Cause cause;
	uint16_t       subCause;
	uint16_t       elementCount;
	uint16_t       elementLength;
	uint16_t       gap;           /**< offset of first element in elementList[] */
	uint8_t        list[1];
} TBlueAPI_GATTDiscoveryInd;
typedef TBlueAPI_GATTDiscoveryInd * PBlueAPI_GATTDiscoveryInd;

typedef struct
{
	/** cmd specific section */
	uint16_t       local_MDL_ID;
	TBlueAPI_GATTDiscoveryType   discoveryType;
	uint16_t       startHandle;
	uint16_t       endHandle;
} TBlueAPI_GATTDiscoveryConf;
typedef TBlueAPI_GATTDiscoveryConf * PBlueAPI_GATTDiscoveryConf;

/** read attribute value(s) ---*/
typedef enum              /**< read types */
{
	blueAPI_GATTReadTypeBasic=0x01,    /**< ATT "Read (Blob) Request"  */
	blueAPI_GATTReadTypeByUUID         /**< ATT "Read By Type Request" */
} TBlueAPI_GATTReadType;

typedef struct
{
	/** cmd specific section */
	uint16_t       local_MDL_ID;
	TBlueAPI_GATTReadType   readType;
	uint16_t       readOffset;
	uint16_t       startHandle;
	uint16_t       endHandle;
	uint16_t       UUID16;             /**< 16 bit UUID */
	uint8_t        UUID128[16];        /**< 128 bit UUID */
} TBlueAPI_GATTAttributeReadReq;
typedef TBlueAPI_GATTAttributeReadReq * PBlueAPI_GATTAttributeReadReq;

typedef struct
{
	/** cmd specific section */
	uint16_t       local_MDL_ID;
	TBlueAPI_GATTReadType   readType;
	TBlueAPI_Cause cause;
	uint16_t       subCause;
	uint16_t       readOffset;
	uint16_t       totalLength;
	uint16_t       attribLength;
	uint16_t       nbrOfHandles;
	uint16_t       gap;             /**< offset of first handle in handlesData[] */
	uint8_t        handlesData[1];
} TBlueAPI_GATTAttributeReadRsp;
typedef TBlueAPI_GATTAttributeReadRsp * PBlueAPI_GATTAttributeReadRsp;

/** write attribute value ---*/
typedef enum              /**< write types */
{
    blueAPI_GATTWriteTypeRequest = 0x01,   /**<  ATT "Write Request"  */
    blueAPI_GATTWriteTypeCommand           /**<  ATT "Write Command"  */
} TBlueAPI_GATTWriteType;

typedef struct
{
	/** cmd specific section */
	uint16_t       local_MDL_ID;
	uint8_t           writeType;//TBlueAPI_GATTWriteType
	uint16_t       attribHandle;
	uint16_t       attribLength;         /**< attrib. value length      */
	uint16_t       writeOffset;          /**< write offset in attribute */
	uint16_t       gap;                  /**< offset of attrib. value in data[] */
	uint8_t        data[1];
} TBlueAPI_GATTAttributeWriteReq;
typedef TBlueAPI_GATTAttributeWriteReq * PBlueAPI_GATTAttributeWriteReq;

typedef struct
{
	/** cmd specific section */
	uint16_t       local_MDL_ID;
	TBlueAPI_GATTWriteType   writeType;
	TBlueAPI_Cause cause;
	uint16_t       subCause;
	uint16_t       attribLength;         /**< attrib. value length              */
	uint16_t       gap;                  /**< offset of attrib. value in data[] */
	uint8_t        data[1];
} TBlueAPI_GATTAttributeWriteRsp;
typedef TBlueAPI_GATTAttributeWriteRsp * PBlueAPI_GATTAttributeWriteRsp;

/** attribute handle/value notification/indication ---*/
typedef struct          /**< indication */
{
	/** cmd specific section */
	uint16_t       local_MDL_ID;
	uint16_t       attribHandle;
	uint16_t       attribLength;         /**< attrib. value length      */
	uint16_t       gap;                  /**< offset of attrib. value in data[] */
	uint8_t        data[1];
} TBlueAPI_GATTAttributeInd;
typedef TBlueAPI_GATTAttributeInd * PBlueAPI_GATTAttributeInd;

typedef struct
{
	/** cmd specific section */
	uint16_t       local_MDL_ID;
} TBlueAPI_GATTAttributeConf;
typedef TBlueAPI_GATTAttributeConf * PBlueAPI_GATTAttributeConf;

typedef TBlueAPI_GATTAttributeInd  TBlueAPI_GATTAttributeNotificationInfo;
typedef TBlueAPI_GATTAttributeNotificationInfo * PBlueAPI_GATTAttributeNotificationInfo;

typedef struct
{
	uint8_t        remote_BD[BLUE_API_BD_SIZE];
	uint16_t       remote_GATT_UUID;
	bool           remote_DID_Discovery;
} TBlueAPI_GATTSDPDiscoveryReq;
typedef TBlueAPI_GATTSDPDiscoveryReq  * PBlueAPI_GATTSDPDiscoveryReq;

typedef struct
{
	TBlueAPI_Cause cause;
} TBlueAPI_GATTSDPDiscoveryRsp;
typedef TBlueAPI_GATTSDPDiscoveryRsp  * PBlueAPI_GATTSDPDiscoveryRsp;

typedef struct
{
	uint32_t       serviceHandle;
	uint8_t        remote_BD[BLUE_API_BD_SIZE];
	uint16_t       remote_GATT_UUID;
	uint16_t       remote_GATT_StartHandle;
	uint16_t       remote_GATT_EndHandle;
} TBlueAPI_GATTSDPDiscoveryInd;
typedef TBlueAPI_GATTSDPDiscoveryInd  * PBlueAPI_GATTSDPDiscoveryInd;

typedef struct
{
	uint32_t       serviceHandle;
	TBlueAPI_Cause cause;
} TBlueAPI_GATTSDPDiscoveryConf;
typedef TBlueAPI_GATTSDPDiscoveryConf  * PBlueAPI_GATTSDPDiscoveryConf;

typedef enum
{
	blueAPI_GATTSecurityRequireMITM = 0x0001,
	blueAPI_GATTSecurityForcePairing = 0x0010,
} TBlueAPI_GATTSecurityRequirement;

typedef struct
{
	uint16_t       local_MDL_ID;
	uint16_t       requirements;
	uint8_t        minKeySize;
} TBlueAPI_GATTSecurityReq;
typedef TBlueAPI_GATTSecurityReq * PBlueAPI_GATTSecurityReq;



typedef struct
{
	uint16_t              local_MDL_ID;
	TBlueAPI_KeyType  keyType;
	uint8_t               keySize;
	TBlueAPI_Cause        cause;
} TBlueAPI_GATTSecurityRsp;
typedef TBlueAPI_GATTSecurityRsp * PBlueAPI_GATTSecurityRsp;

typedef enum
{
	blueAPI_GATTStoreOpGetCCCBits,             /**< get CCC Bits for one <bd,bdtype>                 */
	blueAPI_GATTStoreOpSetCCCBits,             /**< set CCC Bits for one <bd,bdtype>                 */
	blueAPI_GATTStoreOpGetAllCCCBits,          /**< get CCC Bits for all peers                       */
	blueAPI_GATTStoreOpDeleteAllCCCBits,       /**< delete all CCC Bits for all peers                */
	blueAPI_GATTStoreOpGetSrvChg,              /**< get Service Changed Handle/Flag for <bd,bdtype>  */
	blueAPI_GATTStoreOpSetSrvChg,              /**< set Service Changed Handle/Flag for <bd,bdtype>  */
	blueAPI_GATTStoreOpSetSrvChgFlag           /**< set Service Changed Flag for all peers           */
} TBlueAPI_GATTStoreOpCode;

typedef struct
{
	TBlueAPI_GATTStoreOpCode      opCode;
	TBlueAPI_RemoteBDType         remote_BD_Type;
	uint8_t                       remote_BD[BLUE_API_BD_SIZE];
	
	uint16_t                      restartHandle;
	uint8_t                       dataLength;
	uint8_t                       data[32];
} TBlueAPI_GATTServerStoreInd;
typedef TBlueAPI_GATTServerStoreInd * PBlueAPI_GATTServerStoreInd;

typedef struct
{
	TBlueAPI_GATTStoreOpCode      opCode;
	uint8_t                       remote_BD[BLUE_API_BD_SIZE];
	TBlueAPI_RemoteBDType         remote_BD_Type;
	uint16_t                      restartHandle;
	uint8_t                       dataLength;
	uint8_t                       data[32];
	TBlueAPI_Cause                cause;
} TBlueAPI_GATTServerStoreConf;
typedef TBlueAPI_GATTServerStoreConf * PBlueAPI_GATTServerStoreConf;

typedef struct
{
	uint16_t       local_MDL_ID;
	uint16_t       mtuSize;        /**< ATT layer MTU size */
} TBlueAPI_GATTMtuSizeInfo;
typedef TBlueAPI_GATTMtuSizeInfo * PBlueAPI_GATTMtuSizeInfo;

typedef struct
{
    bool               limitedInquiry;
    bool               cancelInquiry;
    uint8_t            InquiryTimeout;
} TBlueAPI_InquiryReq;
typedef TBlueAPI_InquiryReq * PBlueAPI_InquiryReq;

typedef struct
{
	bool           cancelInquiry;
	TBlueAPI_Cause cause;
} TBlueAPI_InquiryRsp;
typedef TBlueAPI_InquiryRsp * PBlueAPI_InquiryRsp;

typedef struct
{
	uint8_t     remote_BD[BLUE_API_BD_SIZE];
	uint32_t    remote_Device_Class;
	int8_t      remote_RSSI;
	uint8_t     remote_Device_Name[BLUE_API_DEVICE_NAME_LENGTH];
} TBlueAPI_InquiryDeviceInfo;
typedef TBlueAPI_InquiryDeviceInfo * PBlueAPI_InquiryDeviceInfo;

typedef struct
{
	uint8_t     remote_BD[BLUE_API_BD_SIZE];
} TBlueAPI_DeviceNameReq;
typedef TBlueAPI_DeviceNameReq * PBlueAPI_DeviceNameReq;

typedef struct
{
	uint8_t     remote_BD[BLUE_API_BD_SIZE];
	uint8_t     remote_Device_Name[BLUE_API_DEVICE_NAME_LENGTH];
	TBlueAPI_Cause cause;
} TBlueAPI_DeviceNameRsp;
typedef TBlueAPI_DeviceNameRsp * PBlueAPI_DeviceNameRsp;

typedef struct
{
	uint32_t    serviceHandle;
	uint8_t     remote_BD[BLUE_API_BD_SIZE];
	uint16_t    remote_VendorID;
	uint16_t    remote_VendorID_Source;
	uint16_t    remote_ProductID;
	uint16_t    remote_Version;
	uint8_t     remote_Device_Name[BLUE_API_DEVICE_NAME_LENGTH];
} TBlueAPI_DIDDeviceInd;
typedef TBlueAPI_DIDDeviceInd * PBlueAPI_DIDDeviceInd;

typedef struct
{
	uint32_t    serviceHandle;
	TBlueAPI_Cause cause;
} TBlueAPI_DIDDeviceConf;
typedef TBlueAPI_DIDDeviceConf * PBlueAPI_DIDDeviceConf;

typedef struct
{
	uint8_t               remote_BD[BLUE_API_BD_SIZE];
	TBlueAPI_MDEPDataType remote_MDEP_DataType;
	bool                  remote_DID_Discovery;
} TBlueAPI_SDPDiscoveryReq;
typedef TBlueAPI_SDPDiscoveryReq * PBlueAPI_SDPDiscoveryReq;

typedef struct
{
	TBlueAPI_Cause        cause;
} TBlueAPI_SDPDiscoveryRsp;
typedef TBlueAPI_SDPDiscoveryRsp * PBlueAPI_SDPDiscoveryRsp;

typedef struct
{
	uint32_t                serviceHandle;
	uint8_t                 remote_BD[BLUE_API_BD_SIZE];
	uint8_t                 remote_MDEP_ID;
	TBlueAPI_MDEPDataType   remote_MDEP_DataType;
	uint8_t                 remote_MDEP_Description[BLUE_API_MDEP_NAME_LENGTH];
    uint16_t                remoteversion;
    uint16_t                supportedfeatures;
} TBlueAPI_SDPEndpointInd;
typedef TBlueAPI_SDPEndpointInd * PBlueAPI_SDPEndpointInd;

typedef struct
{
	uint32_t       serviceHandle;
	TBlueAPI_Cause cause;
} TBlueAPI_SDPEndpointConf;
typedef TBlueAPI_SDPEndpointConf * PBlueAPI_SDPEndpointConf;

typedef enum
{
	blueAPI_LEAdvModeDisabled = 0,
	blueAPI_LEAdvModeEnabled,
	blueAPI_LEAdvModeDirectedHighDuty
} TBlueAPI_LEAdvMode;

typedef enum
{
	blueAPI_LEAdvTypeUndirected = 0,
	blueAPI_LEAdvTypeDirectedHighDuty,
	blueAPI_LEAdvTypeScannable,
	blueAPI_LEAdvTypeNonConnectable,
	blueAPI_LEAdvTypeDirectedLowDuty
} TBlueAPI_LEAdvType;

/** This enum type describes adv type for blueAPI_EventLEScanInfo. */
typedef enum
{
    blueAPI_LEAdvEventTypeUndirected = 0,    /**<  Connectable  undirected advertising. */
	blueAPI_LEAdvEventTypeDirected = 1, /**<  Connectable directed advertising. */ 
    blueAPI_LEAdvEventTypeScannable = 2,     /**<  Scanable undirected advertising. */
    blueAPI_LEAdvEventTypeNonConnectable,    /**<  Nonconnectable undirected advertising. */
    blueAPI_LEAdvEventTypeScanResponse    /**<  scan response. */
} TBlueAPI_LEAdvEventType;

typedef enum
{
	blueAPI_LEFilterAny = 0,
	blueAPI_LEFilterWhitelist
} TBlueAPI_LEFilterPolicy;

typedef enum
{
	blueAPI_LEDataTypeAdvertisingData = 0,
	blueAPI_LEDataTypeScanResponseData
} TBlueAPI_LEDataType;

typedef enum
{
	blueAPI_LEScanDisabled = 0,
	blueAPI_LEScanPassive,
	blueAPI_LEScanActive
} TBlueAPI_LEScanMode;

typedef enum
{
	blueAPI_LEWhitelistOpClear = 0,
	blueAPI_LEWhitelistOpAdd,
	blueAPI_LEWhitelistOpRemove
} TBlueAPI_LEWhitelistOp;

typedef struct
{
	TBlueAPI_LEAdvMode      advMode;
} TBlueAPI_LEAdvertiseReq;
typedef TBlueAPI_LEAdvertiseReq * PBlueAPI_LEAdvertiseReq;

typedef struct
{
	TBlueAPI_LEAdvMode      advMode;
	TBlueAPI_Cause          cause;
} TBlueAPI_LEAdvertiseRsp;
typedef TBlueAPI_LEAdvertiseRsp * PBlueAPI_LEAdvertiseRsp;

typedef struct
{
    uint8_t      advType;//TBlueAPI_LEAdvType
    TBlueAPI_LEFilterPolicy filterScanReq;
    TBlueAPI_LEFilterPolicy filterConnectReq;
    uint16_t                minAdvInterval;
    uint16_t                maxAdvInterval;
    uint8_t                 remote_BD[BLUE_API_BD_SIZE];
    uint8_t                 remote_BD_type;//TBlueAPI_RemoteBDType
    uint8_t                 local_BD_type;
} TBlueAPI_LEAdvertiseParameterSetReq;
typedef TBlueAPI_LEAdvertiseParameterSetReq * PBlueAPI_LEAdvertiseParameterSetReq;

typedef struct
{
	TBlueAPI_Cause          cause;
} TBlueAPI_LEAdvertiseParameterSetRsp;
typedef TBlueAPI_LEAdvertiseParameterSetRsp * PBlueAPI_LEAdvertiseParameterSetRsp;

typedef struct
{
	TBlueAPI_LEDataType     dataType;
	uint8_t                 dataLength;
	uint8_t     *pDataBuffer;
} TBlueAPI_LEAdvertiseDataSetReq;
typedef TBlueAPI_LEAdvertiseDataSetReq * PBlueAPI_LEAdvertiseDataSetReq;

typedef struct
{
	TBlueAPI_LEDataType     dataType;
	TBlueAPI_Cause          cause;
} TBlueAPI_LEAdvertiseDataSetRsp;
typedef TBlueAPI_LEAdvertiseDataSetRsp * PBlueAPI_LEAdvertiseDataSetRsp;

typedef struct
{
	TBlueAPI_LEScanMode     scanMode;
	uint16_t                scanInterval;
	uint16_t                scanWindow;
	TBlueAPI_LEFilterPolicy filterPolicy;
    TBlueAPI_LocalBDType     local_BD_Type;
	uint8_t                 filterDuplicates;
} TBlueAPI_LEScanReq;
typedef TBlueAPI_LEScanReq * PBlueAPI_LEScanReq;

typedef struct
{
	TBlueAPI_Cause          cause;
} TBlueAPI_LEScanRsp;
typedef TBlueAPI_LEScanRsp * PBlueAPI_LEScanRsp;

typedef struct
{
	uint8_t                 remote_BD[BLUE_API_BD_SIZE];
	TBlueAPI_RemoteBDType   remote_BD_type;
	TBlueAPI_LEAdvEventType advType;
	int8_t                  rssi;
	uint8_t                 dataLength;
	uint8_t                 data[31];
} TBlueAPI_LEScanInfo;
typedef TBlueAPI_LEScanInfo * PBlueAPI_LEScanInfo;

typedef struct
{
	TBlueAPI_LEWhitelistOp  operation;
	uint8_t                 remote_BD[BLUE_API_BD_SIZE];
	TBlueAPI_RemoteBDType   remote_BD_type;
} TBlueAPI_LEModifyWhitelistReq;
typedef TBlueAPI_LEModifyWhitelistReq * PBlueAPI_LEModifyWhitelistReq;

typedef struct
{
	TBlueAPI_LEWhitelistOp  operation;
	TBlueAPI_Cause          cause;
} TBlueAPI_LEModifyWhitelistRsp;
typedef TBlueAPI_LEModifyWhitelistRsp * PBlueAPI_LEModifyWhitelistRsp;

typedef struct
{
	uint16_t                local_MDL_ID;
	uint16_t                connIntervalMin;
	uint16_t                connIntervalMax;
	uint16_t                connLatency;
	uint16_t                supervisionTimeout;
} TBlueAPI_LEConnectionUpdateReq;
typedef TBlueAPI_LEConnectionUpdateReq * PBlueAPI_LEConnectionUpdateReq;

typedef struct
{
	uint16_t                local_MDL_ID;
	TBlueAPI_Cause          cause;
} TBlueAPI_LEConnectionUpdateRsp;
typedef TBlueAPI_LEConnectionUpdateRsp * PBlueAPI_LEConnectionUpdateRsp;

typedef struct
{
	uint16_t                local_MDL_ID;
	uint16_t                connIntervalMin;
	uint16_t                connIntervalMax;
	uint16_t                connLatency;
	uint16_t                supervisionTimeout;
} TBlueAPI_LEConnectionUpdateInd;
typedef TBlueAPI_LEConnectionUpdateInd * PBlueAPI_LEConnectionUpdateInd;

typedef struct
{
	uint16_t                local_MDL_ID;
	TBlueAPI_Cause          cause;
} TBlueAPI_LEConnectionUpdateConf;
typedef TBlueAPI_LEConnectionUpdateConf * PBlueAPI_LEConnectionUpdateConf;

typedef struct
{
	uint16_t                local_MDL_ID;
	uint16_t                connInterval;
	uint16_t                connLatency;
	uint16_t                supervisionTimeout;
} TBlueAPI_LEConnectionParameterInfo;
typedef TBlueAPI_LEConnectionParameterInfo * PBlueAPI_LEConnectionParameterInfo;

typedef struct
{
    uint8_t          random_BD[BLUE_API_BD_SIZE];
} TBlueAPI_SetRandomAddressReq;
typedef TBlueAPI_SetRandomAddressReq * PBlueAPI_SetRandomAddressReq;

typedef struct
{
    TBlueAPI_Cause   cause;          /**<  indicates the result of the transaction. */
    uint16_t         subCause;       /**<  More detailed result information from lower protocol layers. */ 
} TBlueAPI_SetRandomAddressRsp;
typedef TBlueAPI_SetRandomAddressRsp * PBlueAPI_SetRandomAddressRsp;

//#if F_BT_LE_BT41_SUPPORT
typedef struct
{
    uint16_t                local_MDL_ID;
    uint16_t                le_psm;
    uint16_t                mtu;
    uint16_t                mps;
    uint16_t                initialCredits;
    uint16_t                creditsIncrease;
} TBlueAPI_CreateLEDataChannelReq;
typedef TBlueAPI_CreateLEDataChannelReq * PBlueAPI_CreateLEDataChannelReq;

/**
 * @brief  blueAPI_EventCreateLEDataChannelRsp command data.
 *
 * used by the MDC to respond to an CreateLEDataChannelReq of the MDH. 
 */
typedef struct
{
    uint16_t                local_MDL_ID;   /**<  local link ID.  */
    uint16_t                channel;        /**<  channel ID.  */
    TBlueAPI_Cause          cause;          /**<  indicates the result of the transaction. */
    uint16_t                subCause;       /**<  More detailed result information from lower protocol layers. */ 
} TBlueAPI_CreateLEDataChannelRsp;
typedef TBlueAPI_CreateLEDataChannelRsp * PBlueAPI_CreateLEDataChannelRsp;

/**
 * @brief  blueAPI_EventCreateLEDataChannelInd command data.
 *
 * Used by the MDC to indicate a creation of LE credit based connection to the MDH.
 */
typedef struct
{
    uint16_t                local_MDL_ID; /**<  local link ID.  */
    uint16_t                channel;      /**<  channel ID.  */
} TBlueAPI_CreateLEDataChannelInd;
typedef TBlueAPI_CreateLEDataChannelInd * PBlueAPI_CreateLEDataChannelInd;

typedef struct
{
    uint16_t                local_MDL_ID;
    uint16_t                channel;
    uint16_t                mtu;
    uint16_t                mps;
    uint16_t                initialCredits;
    uint16_t                creditsIncrease;
    TBlueAPI_Cause          cause;
} TBlueAPI_CreateLEDataChannelConf;
typedef TBlueAPI_CreateLEDataChannelConf * PBlueAPI_CreateLEDataChannelConf;


typedef struct
{
    uint16_t                local_MDL_ID;
    uint16_t                channel;
} TBlueAPI_DisconnectLEDataChannelReq;
typedef TBlueAPI_DisconnectLEDataChannelReq * PBlueAPI_DisconnectLEDataChannelReq;

/**
 * @brief  blueAPI_EventDisconnectLEDataChannelRsp command data.
 *
 * used by the MDC to respond to an DisconnectLEDataChannelReq of the MDH. 
 */
typedef struct
{
    uint16_t                local_MDL_ID;  /**<  local link ID.  */
    uint16_t                channel;       /**<  channel ID.  */
    TBlueAPI_Cause          cause;         /**<  indicates the result of the transaction. */
    uint16_t                subCause;      /**<  More detailed result information from lower protocol layers. */ 
} TBlueAPI_DisconnectLEDataChannelRsp;
typedef TBlueAPI_DisconnectLEDataChannelRsp * PBlueAPI_DisconnectLEDataChannelRsp;

/**
 * @brief  blueAPI_EventDisconnectLEDataChannelInd command data.
 *
 * used by the MDC to indicate a disconnection of an established LE credit based connection to the MDH.
 */
typedef struct
{
    uint16_t                local_MDL_ID;  /**<  local link ID.  */
    uint16_t                channel;       /**<  channel ID to be disconnected.  */
    TBlueAPI_Cause          cause;         /**<  indicates the result of the transaction. */
    uint16_t                subCause;      /**<  More detailed result information from lower protocol layers. */ 
} TBlueAPI_DisconnectLEDataChannelInd;
typedef TBlueAPI_DisconnectLEDataChannelInd * PBlueAPI_DisconnectLEDataChannelInd;

typedef struct
{
    uint16_t                local_MDL_ID;
    uint16_t                channel;
    TBlueAPI_Cause          cause;
} TBlueAPI_DisconnectLEDataChannelConf;
typedef TBlueAPI_DisconnectLEDataChannelConf * PBlueAPI_DisconnectLEDataChannelConf;


typedef struct
{
    uint16_t                local_MDL_ID;
    uint16_t                channel;
    uint16_t                credits;
} TBlueAPI_SendLEFlowControlCreditReq;
typedef TBlueAPI_SendLEFlowControlCreditReq * PBlueAPI_SendLEFlowControlCreditReq;

/**
 * @brief  blueAPI_EventSendLEFlowControlCreditRsp command data.
 *
 * used by the MDC to respond to an SendLEFlowControlCreditReq of the MDH. 
 */
typedef struct
{
    uint16_t                local_MDL_ID;   /**<  local link ID.  */
	uint16_t                channel;        /**<  channel ID  */
    TBlueAPI_Cause          cause;          /**<  indicates the result of the transaction. */
    uint16_t                subCause;       /**<  More detailed result information from lower protocol layers. */ 
} TBlueAPI_SendLEFlowControlCreditRsp;
typedef TBlueAPI_SendLEFlowControlCreditRsp * PBlueAPI_SendLEFlowControlCreditRsp;

typedef struct
{
    uint16_t                local_MDL_ID;
    uint16_t                channel;
    uint16_t                valueLength;
    uint16_t                gap;
    uint8_t                 data[1];
} TBlueAPI_LEDataReq;
typedef TBlueAPI_LEDataReq * PBlueAPI_LEDataReq;

/**
 * @brief  blueAPI_EventLEDataRsp command data.
 *
 * used by the MDC to respond to an LEDataReq of the MDH. 
 */
typedef struct
{
    uint16_t                local_MDL_ID;   /**<  local link ID.  */
    uint16_t                channel;        /**<  channel ID  */
    TBlueAPI_Cause          cause;          /**<  indicates the result of the transaction. */
    uint16_t                subCause;       /**<  More detailed result information from lower protocol layers. */ 
} TBlueAPI_LEDataRsp;
typedef TBlueAPI_LEDataRsp * PBlueAPI_LEDataRsp;


/**
 * @brief  blueAPI_EventLEDataInd command data.
 *
 * used by the MDC to indicate LE Data coming from the remote device.
 */
typedef struct
{
    uint16_t                local_MDL_ID;   /**<  local link ID.  */
    uint16_t                channel;        /**<  channel ID  */
    uint16_t                valueLength;    /**<  value length  */
    uint16_t                gap;            /**<  offset of value in data[].  */   
    uint8_t                 data[1];        /**<  value.  */  
} TBlueAPI_LEDataInd;
typedef TBlueAPI_LEDataInd * PBlueAPI_LEDataInd;

typedef struct
{
    uint16_t                local_MDL_ID;
    uint16_t                channel;
    TBlueAPI_Cause          cause;
} TBlueAPI_LEDataConf;
typedef TBlueAPI_LEDataConf * PBlueAPI_LEDataConf;


/**
 * @brief  blueAPI_EventLEDataChannelParameterInfo command data.
 *
 * used by the MDC to inform the MDH about updated LE credit based Connection parameters. 
 */
typedef struct
{
    uint16_t                local_MDL_ID;  /**<  local link ID.  */
    uint16_t                channel;       /**<  channel ID  */
    uint16_t                remote_mtu;    /**<  remote Maximum Transmission Unit */
    uint16_t                remote_mps;     /**<  remote Maximum PDU Size */
    uint16_t                remote_initial_credits;   /**<  remote initial credits*/
	uint16_t                maxDsCredits;    /**<  max downstream credits */
} TBlueAPI_LEDataChannelParameterInfo;
typedef TBlueAPI_LEDataChannelParameterInfo * PBlueAPI_LEDataChannelParameterInfo;

/**
 * @brief  blueAPI_EventLEDataChannelCreditsAlertInfo command data.
 *
 * used by the MDC to inform the MDH about local credits is zero. Remote device can not send LE-frame.
 * please use blueAPI_SendLEFlowControlCreditReq to increase credits.
 */
typedef struct
{
    uint16_t                local_MDL_ID;   /**<  local link ID.  */
    uint16_t                channel;        /**<  channel ID  */
} TBlueAPI_LEDataChannelCreditsAlertInfo;
typedef TBlueAPI_LEDataChannelCreditsAlertInfo * PBlueAPI_LEDataChannelCreditsAlertInfo;

typedef struct
{
    uint16_t                local_MDL_ID;
    uint16_t                channel;
} TBlueAPI_LEDataChannelDeleteInfo;
typedef TBlueAPI_LEDataChannelDeleteInfo * PBlueAPI_LEDataChannelDeleteInfo;

typedef enum
{
    blueAPI_SecNoSecurity,
    blueAPI_SecUnauthenticatedEncryption,
    blueAPI_SecAuthenticatedEncryption,
    blueAPI_SecUnauthenticatedDataSigning,
    blueAPI_SecAuthenticatedDataSigning,
    blueAPI_SecAuthorization
} TBlueAPI_LESecurityMode;

typedef struct
{
    uint16_t  le_psm;
    bool      active;
    TBlueAPI_LESecurityMode      secMode;
    uint8_t   keySize;
} TBlueAPI_LEPsmSecuritySetReq;
typedef TBlueAPI_LEPsmSecuritySetReq * PBlueAPI_LEPsmSecuritySetReq;

/**
 * @brief  blueAPI_EventLEPsmSecuritySetRsp command data.
 *
 * used by the MDC to respond to an LEPsmSecuritySetReq of the MDH. 
 */
typedef struct
{
    TBlueAPI_Cause          cause;   /**<  indicates the result of the transaction. */
} TBlueAPI_LEPsmSecuritySetRsp;
typedef TBlueAPI_LEPsmSecuritySetRsp * PBlueAPI_LEPsmSecuritySetRsp;

//#endif

typedef struct
{
    void *      buf;
    uint8_t     offset;
} TBlueAPI_SDPRegisterReq;
typedef TBlueAPI_SDPRegisterReq * PBlueAPI_SDPRegisterReq;

typedef struct
{
    uint16_t    psm;
    uint16_t    listenQueue;
    uint8_t     action;
} TBlueAPI_L2cProtocolRegisterReq;
typedef TBlueAPI_L2cProtocolRegisterReq * PBlueAPI_L2cProtocolRegisterReq;

typedef struct
{
    TBlueAPI_Cause      cause;
} TBlueAPI_SDPRegisterRsp;
typedef TBlueAPI_SDPRegisterRsp * PBlueAPI_SDPRegisterRsp;

typedef struct
{
    uint8_t     psm;                /**< L2CAP PSM. =0: not used */
    uint16_t    server_channel;     /**< RFCOMM channel. =0: not used */
    uint8_t     outgoing;           /**< define requirement for outgoing connection (true) or for incoming connection (false) */
    uint8_t     active;             /**< TRUE: entry is active. FALSE: delete corresponding entry */
    uint16_t    uuid;               /**< UUID used in corresponding AUTHORIZATION_IND */
    bool        authentication;     /**< one of AUTHEN_SETTING_XXX coding*/
                                    /**< if set and authentication not already done, then on*/
                                    /**< - outgoing:       start authentication before sending conReq to remote*/
                                    /**< - incoming bt2.0: start authentication before sending conResp to remote*/
                                    /**< - incoming bt2.1: reject connection (bt2.1 specification requirement)*/
    bool        authorize;          /**< link establishment requires explicit user level authorization (done before authentication) */
    bool        encryption;         /**< TRUE: activate encryption on this link, only possible if authen_setting != none*/
                                    /**< if set and encryption not already present, then on*/
                                    /**< - outgoing:       enable encryption before sending conReq to remote*/
                                    /**< - incoming bt2.0: enable encryption before sending conResp to remote*/
                                    /**< - incoming bt2.1: reject connection (bt2.1 specification requirement) */
    bool        mitm;               /**< TRUE: mitm required, only possible if authen_setting != none, FALSE: no MITM required */
}TBlueAPI_L2cSecurityRegisterReq;
typedef TBlueAPI_L2cSecurityRegisterReq *PBlueAPI_L2cSecurityRegisterReq;

typedef struct
{
    uint8_t     psm;
    uint16_t    server_channel;
    uint8_t     outgoing;
    uint8_t     active;
    uint16_t    uuid;
    uint16_t    cause;
}TBlueAPI_L2cSecurityRegisterRsp;
typedef TBlueAPI_L2cSecurityRegisterRsp *PBlueAPI_L2cSecurityRegisterRsp;

typedef struct
{
    uint16_t    channel;
    uint16_t    dlci;
    uint16_t    uuid;
    uint8_t     outgoing;
    uint8_t     active;
    uint8_t     bd[BLUE_API_BD_SIZE];
}TBlueAPI_RFCAuthenticationReq;
typedef TBlueAPI_RFCAuthenticationReq *PBlueAPI_RFCAuthenticationReq;

typedef struct
{
    uint16_t    channel;
    uint16_t    dlci;
    uint8_t     outgoing;
    uint16_t    cause;
    uint8_t     bd[BLUE_API_BD_SIZE];
}TBlueAPI_RFCAuthenticationRsp;
typedef TBlueAPI_RFCAuthenticationRsp *PBlueAPI_RFCAuthenticationRsp;

typedef struct
{
    uint16_t    cid;
    uint16_t    usQueueID;
    uint8_t     remote_BD[BLUE_API_BD_SIZE];
} TBlueAPI_L2cConInd;
typedef TBlueAPI_L2cConInd * PBlueAPI_L2cConInd;

typedef struct
{
    uint16_t    status;
    uint16_t    cid;
} TBlueAPI_L2cConConf;
typedef TBlueAPI_L2cConConf * PBlueAPI_L2cConConf;

typedef struct
{
    uint16_t    psm;
    uint16_t    uuid;
    uint16_t    usQueueID;
    uint16_t    mtuSize;
    uint8_t     remoteBd[BLUE_API_BD_SIZE];
}TBlueAPI_L2cConReq;
typedef TBlueAPI_L2cConReq * PBlueAPI_L2cConReq;

typedef struct
{
    uint16_t    cid;
    uint16_t    usQueueID;
    uint16_t    status;
    uint8_t     remote_BD[BLUE_API_BD_SIZE];
} TBlueAPI_L2cConRsp;
typedef TBlueAPI_L2cConRsp * PBlueAPI_L2cConRsp;

typedef struct
{
    uint16_t    cid;
    uint16_t    localUsMTU;
    uint16_t    remoteMTU;
    uint16_t    dsPoolID;
    uint16_t    usQueueID;
    uint8_t     remoteBd[BLUE_API_BD_SIZE];
} TBlueAPI_L2cConActInd;
typedef TBlueAPI_L2cConActInd * PBlueAPI_L2cConActInd;

typedef struct
{
    uint16_t    cid;
    uint16_t    usQueueID;
    uint16_t    cause;
} TBlueAPI_L2cDiscInd;
typedef TBlueAPI_L2cDiscInd * PBlueAPI_L2cDiscInd;

typedef struct
{
    uint16_t    cid;
} TBlueAPI_L2cDiscConf;
typedef TBlueAPI_L2cDiscConf * PBlueAPI_L2cDiscConf;

typedef struct
{
    uint16_t    cid;
} TBlueAPI_L2cDiscReq;
typedef TBlueAPI_L2cDiscReq * PBlueAPI_L2cDiscReq;

typedef struct
{
    uint16_t    cid;
    uint16_t    usQueueID;
    uint16_t    status;
} TBlueAPI_L2cDiscRsp;
typedef TBlueAPI_L2cDiscRsp * PBlueAPI_L2cDiscRsp;

typedef struct
{
    void *      buf;
    uint8_t     writeOffset;
    uint16_t    cid;
    uint16_t    length;
}TBlueAPI_L2cDataReq;
typedef TBlueAPI_L2cDataReq * PBlueAPI_L2cDataReq;

typedef struct
{
    uint16_t    cid;
    uint16_t    dataOffset;
    uint16_t    length;
    uint8_t*    buf;
    uint16_t    flag;
    uint16_t    usQueueID;
} TBlueAPI_L2cDataInd;
typedef TBlueAPI_L2cDataInd * PBlueAPI_L2cDataInd;

typedef struct
{
    uint8_t     remote_BD[BLUE_API_BD_SIZE];
    uint32_t    txBandwidth;
    uint32_t    rxBandwidth;
    uint16_t    maxLatency;
    uint16_t    voiceSetting;
    uint16_t    packetType;
    uint8_t     retransEffort;
}TBlueAPI_SCOConReq;
typedef TBlueAPI_SCOConReq * PBlueAPI_SCOConReq;

typedef struct
{
    uint16_t    status;
    uint8_t     remote_BD[BLUE_API_BD_SIZE];
} TBlueAPI_SCOConRsp;
typedef TBlueAPI_SCOConRsp * PBlueAPI_SCOConRsp;

typedef struct
{
    uint8_t     remote_BD[BLUE_API_BD_SIZE];
} TBlueAPI_SCOConInd;
typedef TBlueAPI_SCOConInd * PBlueAPI_SCOConInd;

typedef struct
{
    uint8_t         remote_BD[BLUE_API_BD_SIZE];
    uint32_t        txBandwidth;
    uint32_t        rxBandwidth;
    uint16_t        maxLatency;
    uint16_t        voiceSetting;
    uint16_t        packetType;
    uint8_t         retransEffort;
    TBlueAPI_Cause  cause;
}TBlueAPI_SCOConConf;
typedef TBlueAPI_SCOConConf * PBlueAPI_SCOConConf;

typedef struct
{
    uint8_t     airMode;
    uint8_t     remote_BD[BLUE_API_BD_SIZE];
} TBlueAPI_SCOConActInd;
typedef TBlueAPI_SCOConActInd * PBlueAPI_SCOConActInd;

typedef struct
{
    uint8_t     remote_BD[BLUE_API_BD_SIZE];
} TBlueAPI_SCODiscReq;
typedef TBlueAPI_SCODiscReq * PBlueAPI_SCODiscReq;

typedef struct
{
    uint8_t     remote_BD[BLUE_API_BD_SIZE];
} TBlueAPI_SCODiscInd;
typedef TBlueAPI_SCODiscInd * PBlueAPI_SCODiscInd;

typedef union
{
	TBlueAPI_ConnectMDLReq       ConnectMDLReq;
	TBlueAPI_CreateMDLConf       CreateMDLConf; 
	TBlueAPI_DisconnectMDLReq    DisconnectMDLReq;
	TBlueAPI_DisconnectMDLConf   DisconnectMDLConf;
	
	TBlueAPI_RegisterReq         RegisterReq;
	TBlueAPI_ReleaseReq          ReleaseReq;

	TBlueAPI_DeviceConfigSetReq  DeviceConfigSetReq;
	TBlueAPI_GATTServiceRegisterReq         GATTServiceRegisterReq;
	TBlueAPI_GATTAttributeUpdateReq         GATTAttributeUpdateReq;
	TBlueAPI_GATTAttributeUpdateStatusConf  GATTAttributeUpdateStatusConf;
	TBlueAPI_GATTAttributeReadConf          GATTAttributeReadConf;
	TBlueAPI_GATTAttributeWriteConf         GATTAttributeWriteConf;
    TBlueAPI_GATTAttributePrepareWriteConf  GATTAttributePrepareWriteConf;
    TBlueAPI_GATTAttributeExecuteWriteConf  GATTAttributeExecuteWriteConf;
    TBlueAPI_GATTAttributeExecuteWriteReq   GATTAttributeExecuteWriteReq;
	TBlueAPI_GATTDiscoveryReq               GATTDiscoveryReq;
	TBlueAPI_GATTDiscoveryConf              GATTDiscoveryConf;
	TBlueAPI_GATTAttributeReadReq           GATTAttributeReadReq;
	TBlueAPI_GATTAttributeWriteReq          GATTAttributeWriteReq;
    TBlueAPI_GATTAttributeConf              GATTAttributeConf;
	
	TBlueAPI_GATTSecurityReq                GATTSecurityReq;
	TBlueAPI_GATTServerStoreConf            GATTServerStoreConf;

	/** Start of Secutity Management part */
	TBlueAPI_PairableModeSetReq           PairableModeSetReq;
	
	
	TBlueAPI_UserPasskeyReqConf           UserPasskeyReqConf;
	TBlueAPI_UserPasskeyReqReplyReq       UserPasskeyReqReplyReq;
	TBlueAPI_RemoteOOBDataReqConf         RemoteOOBDataReqConf;
	TBlueAPI_AuthResultConf               AuthResultConf;
	TBlueAPI_AuthResultRequestConf        AuthResultRequestConf;
	/**  End of Secutity Management part */

	TBlueAPI_LEAdvertiseReq               LEAdvertiseReq;
	TBlueAPI_LEAdvertiseParameterSetReq   LEAdvertiseParameterSetReq;
	TBlueAPI_LEAdvertiseDataSetReq        LEAdvertiseDataSetReq;
	TBlueAPI_LEScanReq                    LEScanReq;
	TBlueAPI_LEModifyWhitelistReq         LEModifyWhitelistReq;
	TBlueAPI_LEConnectionUpdateReq        LEConnectionUpdateReq;
    TBlueAPI_LEConnectionUpdateConf       LEConnectionUpdateConf;
    TBlueAPI_SetRandomAddressReq          SetRandomAddressReq;

//#if F_BT_LE_BT41_SUPPORT
    TBlueAPI_CreateLEDataChannelReq       CreateLEDataChannelReq;
    TBlueAPI_CreateLEDataChannelConf      CreateLEDataChannelConf;
    TBlueAPI_DisconnectLEDataChannelReq   DisconnectLEDataChannelReq;
    TBlueAPI_DisconnectLEDataChannelConf  DisconnectLEDataChannelConf;
    TBlueAPI_SendLEFlowControlCreditReq   SendLEFlowControlCreditReq;
    TBlueAPI_LEDataReq                    LEDataReq;
    TBlueAPI_LEDataConf                   LEDataConf;
    TBlueAPI_LEPsmSecuritySetReq          LEPsmSecuritySetReq;
//#endif

	TBlueAPI_ReconnectMDLReq     ReconnectMDLReq;
	TBlueAPI_ReconnectMDLConf    ReconnectMDLConf;
    TBlueAPI_ACLConfigReq        ACLConfigReq;

	/** stollmann specific Inquiry/discovery msgs  */
	TBlueAPI_InquiryReq          InquiryReq;
	TBlueAPI_DeviceNameReq       DeviceNameReq;
	TBlueAPI_DIDDeviceConf       DIDDeviceConf;
	TBlueAPI_SDPDiscoveryReq     SDPDiscoveryReq;
	TBlueAPI_SDPEndpointConf     SDPEndpointConf;
	TBlueAPI_RadioModeSetReq     RadioModeSetReq;

    TBlueAPI_GATTSDPDiscoveryReq            GATTSDPDiscoveryReq;
	TBlueAPI_GATTSDPDiscoveryConf           GATTSDPDiscoveryConf;

    TBlueAPI_AuthReq                      AuthReq;
	TBlueAPI_UserAuthRequestConf          UserAuthRequestConf;

    TBlueAPI_UserAuthorizationReqConf     UserAuthorizationReqConf;
    TBlueAPI_UserConfirmationReqConf      UserConfirmationReqConf;
    TBlueAPI_KeypressNotificationReq      KeypressNotificationReq;
    TBlueAPI_LegacyRemoteOOBDataReqConf   LegacyRemoteOOBDataReqConf;
	TBlueAPI_LocalOOBDataReq              LocalOOBDataReq;
    TBlueAPI_AuthDeleteReq                AuthDeleteReq;
	TBlueAPI_AuthListReq                  AuthListReq;

    TBlueAPI_SDPRegisterReq         SDPRegisterReq;
    TBlueAPI_L2cProtocolRegisterReq L2cProtocolRegisterReq;
    TBlueAPI_L2cConReq              L2cConReq;
    TBlueAPI_L2cConConf             L2cConConf;
    TBlueAPI_L2cDataReq             L2cDataReq;
    TBlueAPI_L2cDiscConf            L2cDiscConf;
    TBlueAPI_L2cDiscReq             L2cDiscReq;
    TBlueAPI_L2cSecurityRegisterReq L2cSecurityRegisterReq;
    TBlueAPI_RFCAuthenticationReq   RFCAuthenticationReq;

    TBlueAPI_SCOConReq              SCOConReq;
    TBlueAPI_SCOConConf             SCOConConf;
    TBlueAPI_SCODiscReq             SCODiscReq;
} TBlueAPI_DsCommandData;
typedef TBlueAPI_DsCommandData * PBlueAPI_DsCommandData;
    
typedef union
{
	TBlueAPI_ConnectMDLRsp       ConnectMDLRsp;
	TBlueAPI_CreateMDLInd        CreateMDLInd;
	TBlueAPI_DeleteMDLInfo       DeleteMDLInfo;
	TBlueAPI_ConnectMDLInfo      ConnectMDLInfo;
	TBlueAPI_DisconnectMDLRsp    DisconnectMDLRsp;
	TBlueAPI_DisconnectMDLInd    DisconnectMDLInd;

	/** stollmann specific msgs  */
	TBlueAPI_MCLStatusInfo       MCLStatusInfo;
	TBlueAPI_ACLStatusInfo       ACLStatusInfo;
	
	TBlueAPI_RegisterRsp         RegisterRsp;
	TBlueAPI_ReleaseRsp          ReleaseRsp;
	TBlueAPI_ActInfo             ActInfo;
	TBlueAPI_InternalEventInfo   InternalEventInfo;

	
	TBlueAPI_DeviceConfigSetRsp  DeviceConfigSetRsp;

	TBlueAPI_GATTServiceRegisterRsp         GATTServiceRegisterRsp;
	TBlueAPI_GATTAttributeUpdateRsp         GATTAttributeUpdateRsp;
	TBlueAPI_GATTAttributeUpdateStatusInd   GATTAttributeUpdateStatusInd;
	TBlueAPI_GATTAttributeReadInd           GATTAttributeReadInd;
	TBlueAPI_GATTAttributeWriteInd          GATTAttributeWriteInd;
	TBlueAPI_GATTAttributeWriteCommandInfo  GATTAttributeWriteCommandInfo;

	TBlueAPI_GATTCCCDInfo                   GATTCCCDInfo;

	TBlueAPI_GATTDiscoveryRsp               GATTDiscoveryRsp;
	TBlueAPI_GATTDiscoveryInd               GATTDiscoveryInd;
	TBlueAPI_GATTAttributeReadRsp           GATTAttributeReadRsp;
	TBlueAPI_GATTAttributeWriteRsp          GATTAttributeWriteRsp;
    TBlueAPI_GATTAttributeExecuteWriteInd   GATTAttributeExecuteWriteInd;
    TBlueAPI_GATTAttributePrepareWriteRsp   GATTAttributePrepareWriteRsp;
    TBlueAPI_GATTAttributeExecuteWriteRsp   GATTAttributeExecuteWriteRsp;
	TBlueAPI_GATTAttributeInd               GATTAttributeInd;
	TBlueAPI_GATTAttributeNotificationInfo  GATTAttributeNotificationInfo;
	

	TBlueAPI_GATTSecurityRsp                GATTSecurityRsp;
	TBlueAPI_GATTServerStoreInd             GATTServerStoreInd;
	TBlueAPI_GATTMtuSizeInfo                GATTMtuSizeInfo;

	/** Start of Secutity Management part */
	TBlueAPI_PairableModeSetRsp           PairableModeSetRsp;
	TBlueAPI_AuthResultRequestInd         AuthResultRequestInd;
	TBlueAPI_UserPasskeyReqInd            UserPasskeyReqInd;
	TBlueAPI_UserPasskeyReqReplyRsp       UserPasskeyReqReplyRsp;
	TBlueAPI_UserPasskeyNotificationInfo  UserPasskeyNotificationInfo;
	TBlueAPI_RemoteOOBDataReqInd          RemoteOOBDataReqInd;
	TBlueAPI_AuthResultInd                AuthResultInd;
	
	/**  End of Secutity Management part */

	TBlueAPI_LEAdvertiseRsp               LEAdvertiseRsp;
	TBlueAPI_LEAdvertiseParameterSetRsp   LEAdvertiseParameterSetRsp;
	TBlueAPI_LEAdvertiseDataSetRsp        LEAdvertiseDataSetRsp;
	TBlueAPI_LEScanRsp                    LEScanRsp;
	TBlueAPI_LEScanInfo                   LEScanInfo;
	TBlueAPI_LEModifyWhitelistRsp         LEModifyWhitelistRsp;
	TBlueAPI_LEConnectionUpdateRsp        LEConnectionUpdateRsp;
	TBlueAPI_LEConnectionUpdateInd        LEConnectionUpdateInd;
	TBlueAPI_LEConnectionParameterInfo    LEConnectionParameterInfo;
    TBlueAPI_SetRandomAddressRsp		  SetRandomAddressRsp;

//#if F_BT_LE_BT41_SUPPORT
    TBlueAPI_CreateLEDataChannelRsp       CreateLEDataChannelRsp;
    TBlueAPI_CreateLEDataChannelInd       CreateLEDataChannelInd;
    TBlueAPI_DisconnectLEDataChannelRsp   DisconnectLEDataChannelRsp;
    TBlueAPI_DisconnectLEDataChannelInd   DisconnectLEDataChannelInd;
    TBlueAPI_SendLEFlowControlCreditRsp   SendLEFlowControlCreditRsp;
    TBlueAPI_LEDataRsp                    LEDataRsp;
    TBlueAPI_LEDataInd                    LEDataInd;
    TBlueAPI_LEDataChannelParameterInfo    LEDataChannelParameterInfo;
    TBlueAPI_LEDataChannelCreditsAlertInfo LEDataChannelCreditsAlertInfo;
    TBlueAPI_LEDataChannelDeleteInfo       LEDataChannelDeleteInfo;
    TBlueAPI_LEPsmSecuritySetRsp           LEPsmSecuritySetRsp;
//#endif

    TBlueAPI_ReconnectMDLRsp     ReconnectMDLRsp;
	TBlueAPI_ReconnectMDLInd     ReconnectMDLInd;
    TBlueAPI_ACLConfigRsp        ACLConfigRsp;

	/** stollmann specific Inquiry/discovery msgs  */
	TBlueAPI_InquiryRsp          InquiryRsp;
	TBlueAPI_InquiryDeviceInfo   InquiryDeviceInfo;
	TBlueAPI_DeviceNameRsp       DeviceNameRsp;
	TBlueAPI_DIDDeviceInd        DIDDeviceInd;

    TBlueAPI_SDPDiscoveryRsp     SDPDiscoveryRsp;
    TBlueAPI_SDPEndpointInd      SDPEndpointInd;

	TBlueAPI_RadioModeSetRsp     RadioModeSetRsp;

    TBlueAPI_GATTSDPDiscoveryRsp            GATTSDPDiscoveryRsp;
	TBlueAPI_GATTSDPDiscoveryInd            GATTSDPDiscoveryInd;

    TBlueAPI_UserAuthorizationReqInd      UserAuthorizationReqInd;
	TBlueAPI_AuthRsp                      AuthRsp;
	TBlueAPI_UserAuthRequestInd           UserAuthRequestInd;
    TBlueAPI_UserConfirmationReqInd       UserConfirmationReqInd;
    TBlueAPI_KeypressNotificationRsp      KeypressNotificationRsp;
	TBlueAPI_KeypressNotificationInfo     KeypressNotificationInfo;
    TBlueAPI_LegacyRemoteOOBDataReqInd    LegacyRemoteOOBDataReqInd;
    TBlueAPI_LocalOOBDataRsp              LocalOOBDataRsp;
    TBlueAPI_AuthDeleteRsp                AuthDeleteRsp;
	TBlueAPI_AuthListInfo                 AuthListInfo;
	TBlueAPI_AuthListRsp                  AuthListRsp;

    TBlueAPI_SDPRegisterRsp             SDPRegisterRsp;
    TBlueAPI_L2cConInd                  L2cConInd;
    TBlueAPI_L2cConActInd               L2cConActInd;
    TBlueAPI_L2cDataInd                 L2cDataInd;
    TBlueAPI_L2cDiscInd                 L2cDiscInd;
    TBlueAPI_L2cConRsp                  L2cConRsp;
    TBlueAPI_L2cDiscRsp                 L2cDiscRsp;
    TBlueAPI_L2cSecurityRegisterRsp     L2cSecurityRegisterRsp;
    TBlueAPI_RFCAuthenticationRsp       RFCAuthenticaionRsp;

    TBlueAPI_SCOConInd          SCOConInd;
    TBlueAPI_SCOConRsp          SCOConRsp;
    TBlueAPI_SCOConActInd       SCOConActInd;
    TBlueAPI_SCODiscInd         SCODiscInd;
} TBlueAPI_UsCommandData;
typedef TBlueAPI_UsCommandData * PBlueAPI_UsCommandData;

///@cond
typedef struct _TBlueAPI_DsMessage
{
    uint16_t     Command;
    uint16_t     Length;
    TBlueAPI_DsCommandData p;
} TBlueAPI_DsMessage;
typedef TBlueAPI_DsMessage * PBlueAPI_DsMessage;
///@endcond


typedef struct _TBlueAPI_UsMessage
{
    uint16_t     Command;
    uint16_t     Length;
    TBlueAPI_UsCommandData p;
} TBlueAPI_UsMessage;
typedef TBlueAPI_UsMessage * PBlueAPI_UsMessage;

typedef void (* PBlueAPI_CallBack)(PBlueAPI_UsMessage pMsg);

#if defined (__cplusplus)
 }
#endif

#endif  /**< !defined(__BLUEAPI_TYPES_H) */
