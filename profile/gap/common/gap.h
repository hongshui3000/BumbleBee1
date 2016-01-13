/**
*****************************************************************************************
*     Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
  * @file    gap.h
  * @brief   This file contains all the constants and functions prototypes for GAP protocol. 
  * @details
  * @author  Ranhui
  * @date    2015-5-22
  * @version v0.1
  * *************************************************************************************
  */

/* Define to prevent recursive inclusion **/
#ifndef GAP_H
#define GAP_H

#ifdef __cplusplus
extern "C"
{
#endif

/** Add Includes here **/
#include <rtl_types.h>
#include <blueapi_types.h>
#include <bterrcod.h>
#include <bee_message.h>

//#define BT_GAP_PARAM_TX_POWER_SET

/** this MACRO used for internal debug. Normally closed. **/
//#define GAP_DEBUG_FLAG

/** @addtogroup RTK_GAP_MODULE
  * @{
  */

/** @defgroup GAP_COMMON_DEFINITIONS GAP Common Definitions
  * @{
  */ 

/** @defgroup GAP_Exported_Constants GAP Exported Constants
  * @{
  */

/** @defgroup GAP_Common_Constants GAP Common Constants
  * @{
  */
#define B_ADDR_LEN					6   //!< Default BT Public and Random Address Length
#define KEYLEN                      16  //!< Default key length
#define B_MAX_ADV_LEN               31  //!< BLE Maximum Advertising or Scan Response Packet Length
#define GAP_DEVICE_NAME_LEN         (39+1)  //!< Max length of device name, if device name length exceeds it, it will be truncated.
#define ADV_CHANMAP_SIZE            5	//!<  Advertiser Channel Map.
#define GAP_PASSCODE_MAX            999999  //!< Maximum Pairing Passcode/Passkey value.  Range of a passkey can be 0 - 999,999.
/**
  * @}
  */

/** @defgroup GAP_BT_MSG_TYPE GAP BT Message Type Definitions
  * @{
  */
#define BT_MSG_TYPE_CONN_STATE_CHANGE       0x01
#define BT_MSG_TYPE_BOND_STATE_CHANGE       0x02
#define BT_MSG_TYPE_BOND_PASSKEY_DISPLAY    0x03
#define BT_MSG_TYPE_BOND_PASSKEY_INPUT      0x04
#define BT_MSG_TYPE_BOND_OOB_INPUT          0x05
#define BT_MSG_TYPE_ENCRYPT_STATE_CHANGE    0x06
#define BT_MSG_TYPE_CONN_PARA_UPDATE_CHANGE 0x07    
#ifdef BT_GAP_PARAM_TX_POWER_SET
#define BT_MSG_TYPE_PARAM_SET_RESULT        0x08
#endif

#define BT_MSG_TYPE_BOND_LEGACY_OOB_INPUT   0x10
/**
  * @}
  */

/** @defgroup GAP_PARAM GAP Parameters
  * @{
  */

/** @defgroup GAP_PARAM_ALL_ROLES All Roles
  * @{
  */
#define GAPPARA_PROFILE_ROLE            0x200  //!< Device's Profile Role. Read Only. Size is uint8.
#define GAPPARA_BD_ADDR                 0x201  //!< Device's Address. Read Only. Size is uint8[B_ADDR_LEN]. This item is read from the controller.
#define GAPPARA_BD_ADDR_TYPE            0x202  //!< Device's Address's Type. Read/Write. Address's Type is public or random.
#define GAPPRRA_DEVICE_NAME             0x203  //!< Device's Name. Write Only. Name string length is GAP_DEVICE_NAME_LEN.
#define GAPPRRA_APPEARANCE              0x204  //!< Device's Appearance. Read/Write. Appearance value please refer to GAP Appearance Values.
#define GAPPRRA_ROLE_STATE              0x205  //!< Device's current GAP role's state. Read/Write. Each role may have different states.
//#ifdef BT_GAP_PARAM_TX_POWER_SET
#define GAPPARA_BLE_TX_POWER            0x206
//#endif
/**
  * @}
  */
  
/** @defgroup GAP_PARAM_PERIPH_BROAD Peripheral and Broadcaster
  * @{
  */
#define GAPPRRA_ADVERT_DATA             0x210  //!< Advertisement Data. Read/Write. Max size is uint8[B_MAX_ADV_LEN]. Default is "02:01:01", which means that it is a Limited Discoverable Advertisement.
#define GAPPRRA_SCAN_RSP_DATA           0x211  //!< Scan Response Data. Read/Write. Max size is uint8[B_MAX_ADV_LEN]. Defaults to all 0.
#define GAPPRRA_ADV_EVENT_TYPE      	0x212  //!< Advertisement Type. Read/Write. Size is uint8.  Default is GAP_ADTYPE_ADV_IND (defined in GAP.h).
#define GAPPRRA_ADV_DIRECT_ADDR_TYPE    0x213  //!< Direct Advertisement Address Type. Read/Write. Size is uint8. 
#define GAPPRRA_ADV_DIRECT_ADDR     	0x214  //!< Direct Advertisement Address. Read/Write. Size is uint8[B_ADDR_LEN]. Default is NULL.
#define GAPPRRA_ADV_CHANNEL_MAP     	0x215  //!< Which channels to advertise on. Read/Write Size is uint8. Default is GAP_ADVCHAN_ALL (defined in GAP.h)
#define GAPPRRA_ADV_FILTER_POLICY   	0x216  //!< Filter Policy. Ignored when directed advertising is used. Read/Write. Size is uint8. Default is GAP_FILTER_POLICY_ALL (defined in GAP.h).
#define GAPPRRA_ADV_INTERVAL_MIN     	0x217  //!< Minimum advertising interval for undirected and low duty cycle directed advertising. Value range: 0x0020 - 0x4000 (20ms - 10240ms 0.625ms/step),Read/Write Size is uint16_t.                                                                               
#define GAPPRRA_ADV_INTERVAL_MAX   		0x218  //!< Maximum advertising interval for undirected and low duty cycle directed  advertising. Value range: 0x0020 - 0x4000 (20ms - 10240ms 0.625ms/step)),Read/Write Size is uint16_t.         
#define GAPPRRA_ADV_ENABLE_DEFAULT   	0x219  //!< Advertising enable default. Read/Write. Size is uint8_t.
/**
  * @}
  */

/** @defgroup GAP_PARAM_CENTRAL_OBSERV Central and Observer
  * @{
  */
#define GAPPARA_SCANMODE                0x220 //!< Scan mode. Size is uint8. 
#define GAPPARA_SCANINTERVAL            0x221 //!< Scan Interval. Size is uint16_t.
#define GAPPARA_SCANWINDOW              0x222 //!< Scan Window. Size is uint16_t.
#define GAPPARA_FILTERPOLICY            0x223 //!< Scan Filter Policy.Size is uint8_t.
#define GAPPARA_FILTERDUPLICATES        0x224 //!< Scan Filter Duplicates.Size is uint8_t.
#define GAPPRRA_SCAN_ENABLE_DEFAULT     0x225 //!< Scan Filter Duplicates.Size is uint8_t.
/**
  * @}
  */

/** @defgroup GAP_PARAM_PERIPH_CENTRAL Peripheral and Central
  * @{
  */
#define GAPPRRA_MIN_CONN_INTERVAL   0x230  //!< Minimum Connection Interval to allow (n * 1.25ms).  Range: 7.5 msec to 4 seconds (0x0006 to 0x0C80). Read/Write. Size is uint16. Default is 7.5 milliseconds (0x0006).
#define GAPPRRA_MAX_CONN_INTERVAL   0x231  //!< Maximum Connection Interval to allow (n * 1.25ms).  Range: 7.5 msec to 4 seconds (0x0006 to 0x0C80). Read/Write. Size is uint16. Default is 4 seconds (0x0C80).
#define GAPPRRA_SLAVE_LATENCY       0x232  //!< Update Parameter Slave Latency. Range: 0 - 499. Read/Write. Size is uint16. Default is 0.
#define GAPPRRA_TIMEOUT_MULTIPLIER  0x233  //!< Update Parameter Timeout Multiplier (n * 10ms). Range: 100ms to 32 seconds (0x000a - 0x0c80). Read/Write. Size is uint16. Default is 1000.
#define GAPPRRA_CONN_BD_ADDR        0x234  //!< Address of connected device. Read only. Size is uint8[B_MAX_ADV_LEN]. Set to all zeros when not connected.
#define GAPPRRA_CONN_BD_ADDR_TYPE 	0x235  //!< Address type of connected device. Read only. Size is uint8[B_MAX_ADV_LEN]. Set to all zeros when not connected.
#define GAPPRRA_CONN_INTERVAL       0x236  //!< Current connection interval.  Read only. Size is uint16.  Range is 7.5ms to 4 seconds (0x0006 to 0x0C80).  Default is 0 (no connection).
#define GAPPRRA_CONN_LATENCY        0x237  //!< Current slave latency.  Read only.  Size is uint16.  Range is 0 to 499. Default is 0 (no slave latency or no connection).
#define GAPPRRA_CONN_TIMEOUT        0x238  //!< Current timeout value.  Read only.  size is uint16.  Range is 100ms to 32 seconds.  Default is 0 (no connection).
#define GAPPRRA_CONNHANDLE          0x240  //!< Connection Handle.  Read only.  size is uint16. 
#define GAPPRRA_DSPOOLID            0x241  //!< Downstream PoolID.  Read only.  size is uint16. 
#define GAPPRRA_DSDATAOFFSET        0x242  //!< Downstream Data Offset.  Read only.  size is uint16. 
#define GAPPRRA_MAXTPDUSIZE         0x243  //!< Max Tx pdu size.  Read only.  size is uint16. 
#define GAPPRRA_MAXTPDUDSCREDITS 	0x244  //!< Max Tx pdu ds credits.  Read only.  size is uint16. 
#define GAPPRRA_MTUSIZE             0x245  //!< MTU size.  Read only.  size is uint16. 
/**
  * @}
  */

/** @defgroup GAP_PARAM_PERIPH_ONLY Peripheral Only
  * @{
  */
#define GAPPRRA_DISCONNECTED_REASON	0x250 //!< Disconnected Reason. Read only. Size is uint16. 
#define GAPPRRA_ADV_WL_BD_ADDR      0x251 //!< Address of device in Whiter List. Read Only. Size is uint8[B_ADDR_LEN]. 
#define GAPPRRA_ADV_WL_BD_ADDR_TYPE 0x252 //!< Address of device in Whiter List. Read Only. Size is uint8_t. 
/**
  * @}
  */

/** @defgroup GAP_PARAM_CENTRAL_ONLY Central Only
  * @{
  */
#define GAPPRRA_CONN_DEST_ADDR		0x253  //!< Destination address. Read/Write. Size is uint8[B_MAX_ADV_LEN].
#define GAPPRRA_CONN_DEST_ADDR_TYPE 0x254  //!< Destination address type. Read/Write.
#define GAPPRRA_CONN_SCAN_TIMEOUT   0x255  //!< Scanning timeout. Read/Write.
#define GAPPRRA_CE_LENGTH           0x256  //!< Connection Event length. Read/Write.
/**
  * @}
  */

/** @defgroup GAP_PARAM_LEGACY_ONLY Legacy Only
  * @{
  */
#define GAPPARA_CLASS_OF_DEVICE         0x260
#define GAPPARA_DEVICE_ROLE             0x261
#define GAPPARA_SUPERVISIONTIMEOUT      0x262
#define GAPPARA_LINK_POLICY             0x263
#define GAPPARA_RADIOMODE               0x264
#define GAPPARA_LIMITEDDISCOVERABLE     0x265
#define GAPPARA_PAGESCAN_TYPE           0x266
#define GAPPARA_PAGESCAN_INTERVAL       0x267
#define GAPPARA_PAGESCAN_WINDOW         0x268
#define GAPPARA_PAGETIMEOUT             0x269
#define GAPPARA_INQUIRYSCAN_TYPE        0x26a
#define GAPPARA_INQUIRYSCAN_INTERVAL    0x26b
#define GAPPARA_INQUIRYSCAN_WINDOW      0x26c
#define GAPPARA_INQUIRYMODE             0x26d

/**
  * @}
  */

/**
  * @} End GAP_PARAM
  */

///@cond
/* GAP Roles definition. */
#define GAP_PROFILE_BROADCASTER   0x01 //!< A device that sends advertising events only.
#define GAP_PROFILE_OBSERVER      0x02 //!< A device that receives advertising events only.
#define GAP_PROFILE_PERIPHERAL    0x04 //!< A device that accepts the establishment of an LE physical link using the connection establishment procedure
#define GAP_PROFILE_CENTRAL       0x08 //!< A device that supports the Central role initiates the establishment of a physical connection
///@endcond

/** @defgroup GAP_LOCAL_ADDR_TYPE_DEFINES GAP Address Types
 * @{
 */
#define LOCAL_ADDRTYPE_PUBLIC             blueAPI_LocalBDTypeLEPublic
#define LOCAL_ADDRTYPE_RANDOM             blueAPI_LocalBDTypeLERandom
/** @} End GAP_LOCAL_ADDR_TYPE_DEFINES */

/** @defgroup GAP_PEER_ADDR_TYPE_DEFINES GAP Peer Address Types
 * @{
 */
#define PEER_ADDRTYPE_PUBLIC               blueAPI_RemoteBDTypeLEPublic  //!< Peer Device use public address.
#define PEER_ADDRTYPE_RANDOM               blueAPI_RemoteBDTypeLERandom  //!< Peer Device use random address.
#define PEER_ADDRTYPE_PRIVATE_NONRESOLVE   0x02  //!< Peer Device use private nonresolve address.
#define PEER_ADDRTYPE_PRIVATE_RESOLVE      0x03  //!< Peer Device use private resolvable address.
/** @} End GAP_PEER_ADDR_TYPE_DEFINES */

/** @defgroup GAP_ADVERTISEMENT_TYPE_DEFINES GAP Advertising Event Types
 * for eventType field in gapAdvertisingParams_t
 * @{
 */
#define GAP_ADTYPE_ADV_IND                0x00  //!< Connectable undirected advertisement
#define GAP_ADTYPE_ADV_HDC_DIRECT_IND     0x01  //!< Connectable high duty cycle directed advertisement
#define GAP_ADTYPE_ADV_SCAN_IND           0x02  //!< Scannable undirected advertisement
#define GAP_ADTYPE_ADV_NONCONN_IND        0x03  //!< Non-Connectable undirected advertisement
#define GAP_ADTYPE_ADV_LDC_DIRECT_IND     0x04  //!< Connectable low duty cycle directed advertisement
/** @} End GAP_ADVERTISEMENT_TYPE_DEFINES */

/** @defgroup GAP_ADVERTISEMENT_REPORT_TYPE_DEFINES GAP Advertising Report Event Types
 * for eventType field in gapDevRec_t and gapDeviceInfoEvent_t
 * @{
 */
#define GAP_ADRPT_ADV_IND                 0x00  //!< Connectable undirected advertisement
#define GAP_ADRPT_ADV_DIRECT_IND          0x01  //!< Connectable directed advertisement
#define GAP_ADRPT_ADV_SCAN_IND            0x02  //!< Scannable undirected advertisement
#define GAP_ADRPT_ADV_NONCONN_IND         0x03  //!< Non-Connectable undirected advertisement
#define GAP_ADRPT_SCAN_RSP                0x04  //!< Scan Response
/** @} End GAP_ADVERTISEMENT_REPORT_TYPE_DEFINES */

/** @defgroup GAP_FILTER_POLICY_DEFINES GAP Advertiser Filter Scan Parameters
 * @{
 */
#define GAP_FILTER_POLICY_ALL         0x00 //!< Allow Scan Request from Any, Allow Connect Request from Any (default).
#define GAP_FILTER_POLICY_WHITE_SCAN  0x01 //!< Allow Scan Request from White List Only, Allow Connect from Any
#define GAP_FILTER_POLICY_WHITE_CON   0x02 //!< Allow Scan Request from Any, Connect from White List Only
#define GAP_FILTER_POLICY_WHITE       0x03 //!< Allow Scan Request and Connect from White List Only
/** @} End GAP_FILTER_POLICY_DEFINES */

/** @defgroup GAP_ADVCHAN_DEFINES GAP Advertisement Channel Map
 * @{
 */
#define GAP_ADVCHAN_37  0x01  //!< Advertisement Channel 37
#define GAP_ADVCHAN_38  0x02  //!< Advertisement Channel 38
#define GAP_ADVCHAN_39  0x04  //!< Advertisement Channel 39
#define GAP_ADVCHAN_ALL (GAP_ADVCHAN_37 | GAP_ADVCHAN_38 | GAP_ADVCHAN_39) //!< All Advertisement Channels Enabled
/** @} End GAP_ADVCHAN_DEFINES */



/** @defgroup GAP_ADTYPE_DEFINES GAP Advertisment Data Types
 * These are the data type identifiers for the data tokens in the advertisement data field.
 * @{
 */
#define GAP_ADTYPE_FLAGS                        0x01 //!< Discovery Mode: @ref GAP_ADTYPE_FLAGS_MODES
#define GAP_ADTYPE_16BIT_MORE                   0x02 //!< Service: More 16-bit UUIDs available
#define GAP_ADTYPE_16BIT_COMPLETE               0x03 //!< Service: Complete list of 16-bit UUIDs
#define GAP_ADTYPE_32BIT_MORE                   0x04 //!< Service: More 32-bit UUIDs available
#define GAP_ADTYPE_32BIT_COMPLETE               0x05 //!< Service: Complete list of 32-bit UUIDs
#define GAP_ADTYPE_128BIT_MORE                  0x06 //!< Service: More 128-bit UUIDs available
#define GAP_ADTYPE_128BIT_COMPLETE              0x07 //!< Service: Complete list of 128-bit UUIDs
#define GAP_ADTYPE_LOCAL_NAME_SHORT             0x08 //!< Shortened local name
#define GAP_ADTYPE_LOCAL_NAME_COMPLETE          0x09 //!< Complete local name
#define GAP_ADTYPE_POWER_LEVEL                  0x0A //!< TX Power Level: 0xXX: -127 to +127 dBm
#define GAP_ADTYPE_OOB_CLASS_OF_DEVICE          0x0D //!< Simple Pairing OOB Tag: Class of device (3 octets)
#define GAP_ADTYPE_OOB_SIMPLE_PAIRING_HASHC     0x0E //!< Simple Pairing OOB Tag: Simple Pairing Hash C (16 octets)
#define GAP_ADTYPE_OOB_SIMPLE_PAIRING_RANDR     0x0F //!< Simple Pairing OOB Tag: Simple Pairing Randomizer R (16 octets)
#define GAP_ADTYPE_SM_TK                        0x10 //!< Security Manager TK Value
#define GAP_ADTYPE_SM_OOB_FLAG                  0x11 //!< Secutiry Manager OOB Flags
#define GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE    0x12 //!< Min and Max values of the connection interval (2 octets Min, 2 octets Max) (0xFFFF indicates no conn interval min or max)
#define GAP_ADTYPE_SIGNED_DATA                  0x13 //!< Signed Data field
#define GAP_ADTYPE_SERVICES_LIST_16BIT          0x14 //!< Service Solicitation: list of 16-bit Service UUIDs
#define GAP_ADTYPE_SERVICES_LIST_128BIT         0x15 //!< Service Solicitation: list of 128-bit Service UUIDs
#define GAP_ADTYPE_SERVICE_DATA                 0x16 //!< Service Data
#define GAP_ADTYPE_APPEARANCE                   0x19 //!< Appearance
#define GAP_ADTYPE_MESH_NETWORK                         0xF0 //!< Mesh
#define GAP_ADTYPE_MANUFACTURER_SPECIFIC        0xFF //!< Manufacturer Specific Data: first 2 octets contain the Company Identifier Code followed by the additional manufacturer specific data
/** @} End GAP_ADTYPE_DEFINES */

/** @defgroup GAP_ADTYPE_FLAGS_MODES GAP ADTYPE Flags Discovery Modes
 * @{
 */
#define GAP_ADTYPE_FLAGS_LIMITED                0x01 //!< Discovery Mode: LE Limited Discoverable Mode
#define GAP_ADTYPE_FLAGS_GENERAL                0x02 //!< Discovery Mode: LE General Discoverable Mode
#define GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED    0x04 //!< Discovery Mode: BR/EDR Not Supported
/** @} End GAP_ADTYPE_FLAGS_MODES */

/** @defgroup GAP_APPEARANCE_VALUES GAP Appearance Values
 * @{
 */

/*GATT appearance definitions */
#define GAP_GATT_APPEARANCE_UNKNOWN                                GATT_APPEARANCE_UNKNOWN
#define GAP_GATT_APPEARANCE_GENERIC_PHONE                          GATT_APPEARANCE_GENERIC_PHONE
#define GAP_GATT_APPEARANCE_GENERIC_COMPUTER                       GATT_APPEARANCE_GENERIC_COMPUTER

#define GAP_GATT_APPEARANCE_GENERIC_WATCH                          GATT_APPEARANCE_GENERIC_WATCH
#define GAP_GATT_APPEARANCE_WATCH_SPORTS_WATCH                     GATT_APPEARANCE_WATCH_SPORTS_WATCH

#define GAP_GATT_APPEARANCE_GENERIC_CLOCK                          GATT_APPEARANCE_GENERIC_CLOCK
#define GAP_GATT_APPEARANCE_GENERIC_DISPLAY                        GATT_APPEARANCE_GENERIC_DISPLAY
#define GAP_GATT_APPEARANCE_GENERIC_REMOTE_CONTROL                 GATT_APPEARANCE_GENERIC_REMOTE_CONTROL
#define GAP_GATT_APPEARANCE_GENERIC_EYE_GLASSES                    GATT_APPEARANCE_GENERIC_EYE_GLASSES
#define GAP_GATT_APPEARANCE_GENERIC_TAG                            GATT_APPEARANCE_GENERIC_TAG
#define GAP_GATT_APPEARANCE_GENERIC_KEYRING                        GATT_APPEARANCE_GENERIC_KEYRING
#define GAP_GATT_APPEARANCE_GENERIC_MEDIA_PLAYER                   GATT_APPEARANCE_GENERIC_MEDIA_PLAYER
#define GAP_GATT_APPEARANCE_GENERIC_BARCODE_SCANNER                GATT_APPEARANCE_GENERIC_BARCODE_SCANNER

#define GAP_GATT_APPEARANCE_GENERIC_THERMOMETER                    GATT_APPEARANCE_GENERIC_THERMOMETER
#define GAP_GATT_APPEARANCE_THERMOMETER_EAR                        GATT_APPEARANCE_THERMOMETER_EAR

#define GAP_GATT_APPEARANCE_GENERIC_HEART_RATE_SENSOR              GATT_APPEARANCE_GENERIC_HEART_RATE_SENSOR
#define GAP_GATT_APPEARANCE_HEART_RATE_SENSOR_HEART_RATE_BELT      GATT_APPEARANCE_HEART_RATE_SENSOR_HEART_RATE_BELT

#define GAP_GATT_APPEARANCE_GENERIC_BLOOD_PRESSURE                 GATT_APPEARANCE_GENERIC_BLOOD_PRESSURE
#define GAP_GATT_APPEARANCE_BLOOD_PRESSURE_ARM                     GATT_APPEARANCE_BLOOD_PRESSURE_ARM
#define GAP_GATT_APPEARANCE_BLOOD_PRESSURE_WRIST                   GATT_APPEARANCE_BLOOD_PRESSURE_WRIST

#define GAP_GATT_APPEARANCE_HUMAN_INTERFACE_DEVICE                 GATT_APPEARANCE_HUMAN_INTERFACE_DEVICE
#define GAP_GATT_APPEARANCE_KEYBOARD                               GATT_APPEARANCE_KEYBOARD
#define GAP_GATT_APPEARANCE_MOUSE                                  GATT_APPEARANCE_MOUSE
#define GAP_GATT_APPEARANCE_JOYSTICK                               GATT_APPEARANCE_JOYSTICK
#define GAP_GATT_APPEARANCE_GAMEPAD                                GATT_APPEARANCE_GAMEPAD
#define GAP_GATT_APPEARANCE_DIGITIZER_TABLET                       GATT_APPEARANCE_DIGITIZER_TABLET
#define GAP_GATT_APPEARANCE_CARD_READER                            GATT_APPEARANCE_CARD_READER
#define GAP_GATT_APPEARANCE_DIGITAL_PEN                            GATT_APPEARANCE_DIGITAL_PEN
#define GAP_GATT_APPEARANCE_BARCODE_SCANNER                        GATT_APPEARANCE_BARCODE_SCANNER

#define GAP_GATT_APPEARANCE_GENERIC_GLUCOSE_METER                  GATT_APPEARANCE_GENERIC_GLUCOSE_METER

#define GAP_GATT_APPEARANCE_GENERIC_RUNNING_WALKING_SENSOR         GATT_APPEARANCE_GENERIC_RUNNING_WALKING_SENSOR
#define GAP_GATT_APPEARANCE_RUNNING_WALKING_SENSOR_IN_SHOE         GATT_APPEARANCE_RUNNING_WALKING_SENSOR_IN_SHOE
#define GAP_GATT_APPEARANCE_RUNNING_WALKING_SENSOR_ON_SHOE         GATT_APPEARANCE_RUNNING_WALKING_SENSOR_ON_SHOE
#define GAP_GATT_APPEARANCE_RUNNING_WALKING_SENSOR_ON_HIP          GATT_APPEARANCE_RUNNING_WALKING_SENSOR_ON_HIP

#define GAP_GATT_APPEARANCE_GENERIC_CYCLING                        GATT_APPEARANCE_GENERIC_CYCLING
#define GAP_GATT_APPEARANCE_CYCLING_CYCLING_COMPUTER               GATT_APPEARANCE_CYCLING_CYCLING_COMPUTER
#define GAP_GATT_APPEARANCE_CYCLING_SPEED_SENSOR                   GATT_APPEARANCE_CYCLING_SPEED_SENSOR
#define GAP_GATT_APPEARANCE_CYCLING_CADENCE_SENSOR                 GATT_APPEARANCE_CYCLING_CADENCE_SENSOR
#define GAP_GATT_APPEARANCE_CYCLING_POWER_SENSOR                   GATT_APPEARANCE_CYCLING_POWER_SENSOR
#define GAP_GATT_APPEARANCE_CYCLING_SPEED_AND_CADENCE_SENSOR       GATT_APPEARANCE_CYCLING_SPEED_AND_CADENCE_SENSOR

#define GAP_GATT_APPEARANCE_GENERIC_PULSE_OXIMETER                 GATT_APPEARANCE_GENERIC_PULSE_OXIMETER
#define GAP_GATT_APPEARANCE_FINGERTIP                              GATT_APPEARANCE_FINGERTIP
#define GAP_GATT_APPEARANCE_WRIST_WORN                             GATT_APPEARANCE_WRIST_WORN
#define GAP_GATT_APPEARANCE_GENERIC_WEIGHT_SCALE                   GATT_APPEARANCE_GENERIC_WEIGHT_SCALE

#define GAP_GATT_APPEARANCE_GENERIC_OUTDOOR_SPORTS_ACTIVITY        GATT_APPEARANCE_GENERIC_OUTDOOR_SPORTS_ACTIVITY
#define GAP_GATT_APPEARANCE_LOCATION_DISPLAY_DEVICE                GATT_APPEARANCE_LOCATION_DISPLAY_DEVICE
#define GAP_GATT_APPEARANCE_LOCATION_AND_NAVIGATION_DISPLAY_DEVICE GATT_APPEARANCE_LOCATION_AND_NAVIGATION_DISPLAY_DEVICE
#define GAP_GATT_APPEARANCE_LOCATION_POD                           GATT_APPEARANCE_LOCATION_POD
#define GAP_GATT_APPEARANCE_LOCATION_AND_NAVIGATION_POD            GATT_APPEARANCE_LOCATION_AND_NAVIGATION_POD
/** @} End GAP_APPEARANCE_VALUES */


/** @defgroup GAP_SCAN_MODE_VALUES GAP Scan Mode Values
 * @{
 */
#define GAP_SCAN_MODE_ACTIVE     blueAPI_LEScanActive
#define GAP_SCAN_MODE_PASSIVE   blueAPI_LEScanPassive
/** @} End GAP_SCAN_MODE_VALUES */

/** @defgroup GAP_SCAN_FILTER_POLICY_VALUES GAP Scan Filter Policy Values
 * @{
 */
#define GAP_SCAN_FILTER_ANY   blueAPI_LEFilterAny
#define GAP_SCAN_FILTER_WHITE_LIST   blueAPI_LEFilterWhitelist
/** @} End GAP_SCAN_FILTER_POLICY_VALUES */

/** @defgroup GAP_SCAN_FILTER_DUPLICATE_VALUES GAP Scan Filter Duplicate Values
 * @{
 */
#define GAP_SCAN_FILTER_DUPLICATE_ENABLE        1
#define GAP_SCAN_FILTER_DUPLICATE_DISABLE       0
/** @} End GAP_SCAN_FILTER_DUPLICATE_VALUES */

/** @defgroup TAppCB_MsgType
 * @{
 */
#define ADV_SCAN_RSP_DATA_MSGTYPE               0x00
#define CONN_PARAM_CONFIRM_MSGTYPE              0x01
/** @} End TAppCB_MsgType */

///@cond
/* All GAP States used by all gap roles. */
#define    GAPSTATE_INIT                    0   //!< Waiting to be started
#define    GAPSTATE_STACK_READY             1   //!< Started but not advertising
#define    GAPSTATE_ADVERTISING             2   //!< Advertising
#define    GAPSTATE_CONNECTED               3   //!< Connected
#define    GAPSTATE_CONNECTED_ADV           4   //!< Connected and advertising
#define    GAPSTATE_IDLE_NO_ADV_NO_CONN     5   //!< Idle, no adverting and no connection
#define    GAPSTATE_IDLE_NO_SCAN_NO_CONN    6	//!< Idle, no scanning and no connection
#define    GAPSTATE_SCANNING                7	//!< Scanning
#define    GAPSTATE_CONNECTING              8	//!< Scanning
#define    GAPSTATE_IDLE                    9	//!< Idle
#define    GAPSTATE_INQUIRYING              10
#define    GAPSTATE_BRCONNECTED             11  //!<  BR/EDR connected while  GAPSTATE_CONNECTED is LE connected
///@endcond

/** @defgroup GAP_STATE GAP Role States
 * @{
 */
 
/** @defgroup GAP_STATE_PERIPHERAL Peripheral Role States
 * @{
 */
#define    GAP_PERIPHERAL_STATE_INIT                    GAPSTATE_INIT
#define    GAP_PERIPHERAL_STATE_STACK_READY             GAPSTATE_STACK_READY
#define    GAP_PERIPHERAL_STATE_ADVERTISING             GAPSTATE_ADVERTISING
#define    GAP_PERIPHERAL_STATE_CONNECTED               GAPSTATE_CONNECTED
#define    GAP_PERIPHERAL_STATE_CONNECTED_ADV           GAPSTATE_CONNECTED_ADV
#define    GAP_PERIPHERAL_STATE_IDLE_NO_ADV_NO_CONN     GAPSTATE_IDLE_NO_ADV_NO_CONN
/** End of GAP_STATE_PERIPHERAL
  * @}
  */

/** @defgroup GAP_STATE_CENTRAL Central Role States
 * @{
 */
#define    GAP_CENTRAL_STATE_INIT                       GAPSTATE_INIT
#define    GAP_CENTRAL_STATE_STACK_READY                GAPSTATE_STACK_READY
#define    GAP_CENTRAL_STATE_IDLE_NO_SCAN_NO_CONN       GAPSTATE_IDLE_NO_SCAN_NO_CONN
#define    GAP_CENTRAL_STATE_SCANNING                   GAPSTATE_SCANNING
#define    GAP_CENTRAL_STATE_CONNECTING                 GAPSTATE_CONNECTING
#define    GAP_CENTRAL_STATE_CONNECTED                  GAPSTATE_CONNECTED
/** End of GAP_STATE_CENTRAL
  * @}
  */

/** @defgroup GAP_STATE_BROADCASTER Broadcaster Role States
 * @{
 */
#define    GAP_BROADCASTER_STATE_INIT                   GAPSTATE_INIT
#define    GAP_BROADCASTER_STATE_STACK_READY            GAPSTATE_STACK_READY
#define    GAP_BROADCASTER_STATE_ADVERTISING            GAPSTATE_ADVERTISING
#define    GAP_BROADCASTER_STATE_IDLE                   GAPSTATE_IDLE
/** End of GAP_STATE_BROADCASTER
  * @}
  */

/** @defgroup GAP_STATE_OBSERVER Observer Role States
 * @{
 */
#define    GAP_OBSERVER_STATE_INIT                      GAPSTATE_INIT
#define    GAP_OBSERVER_STATE_STACK_READY               GAPSTATE_STACK_READY
#define    GAP_OBSERVER_STATE_IDLE_NO_SCAN_NO_CONN      GAPSTATE_IDLE_NO_SCAN_NO_CONN
#define    GAP_OBSERVER_STATE_SCANNING                  GAPSTATE_SCANNING
/** End of GAP_STATE_OBSERVER
  * @}
  */

/** @defgroup GAP_STATE_OB Observer Broadcaster Role States
 * @{
 */
#define    GAP_OB_STATE_INIT                      GAPSTATE_INIT
#define    GAP_OB_STATE_STACK_READY               GAPSTATE_STACK_READY
/** End of GAP_STATE_OB
  * @}
  */

/** @defgroup GAP_STATE_LEGACY Legacy Role States
 * @{
 */
#define    GAP_LEGACY_STATE_INIT                    GAPSTATE_INIT
#define    GAP_LEGACY_STATE_STACK_READY             GAPSTATE_STACK_READY
#define    GAP_LEGACY_STATE_IDLE                    GAPSTATE_IDLE
#define    GAP_LEGACY_STATE_CONNECTED               GAPSTATE_BRCONNECTED
#define    GAP_LEGACY_STATE_INQUIRYING              GAPSTATE_INQUIRYING
/** End of GAP_STATE_LEGACY
  * @}
  */

/** End of GAP_STATE
  * @}
  */

/** End of GAP_Exported_Constants
  * @}
  */


/** @defgroup GAP_Exported_Types GAP Exported Types
  * @{
  */

/** @defgroup GAP_STATUS GAP Status
  * @{
  */
typedef enum _TGAP_STATUS
{
    gapAPI_CauseSuccess = 0x00,
    gapAPI_AlreadyInRequestedMode = 0x01,
    gapAPI_IncorrectMode = 0x02,
    gapAPI_InvalidRange = 0x03,
    gapAPI_NotConnected = 0x04,
    gapAPI_ErrorUnknown = 0x05,
    gapAPI_InvalidPara  = 0x06
} TGAP_STATUS;    
/**
  * @}
  */

/** @defgroup TGAPCONN_STATE_CHANGE TGAPCONN_STATE_CHANGE
  * @{
  */
typedef struct _TGAPCONN_STATE_CHANGE
{
    uint8_t newState;
} TGAPCONN_STATE_CHANGE;
/** End of TGAPCONN_STATE_CHANGE
  * @}
  */

/** @defgroup TGAPCONN_PARA_UPDATE_CHANGE TGAPCONN_PARA_UPDATE_CHANGE
  * @{
  */
typedef struct _TGAPCONN_PARA_UPDATE_CHANGE
{
    uint16_t connHandle;
    uint8_t status;
} TGAPCONN_PARA_UPDATE_CHANGE;
/** End of TGAPCONN_PARA_UPDATE_CHANGE
  * @}
  */

/** @defgroup TGAPBONDTATE_CHANGE TGAPBONDTATE_CHANGE
  * @{
  */
typedef struct _TGAPBOND_STATE_CHANGE
{
    uint16_t connHandle;
    uint8_t newState;
    uint8_t status;
} TGAPBONDTATE_CHANGE;
/** End of TGAPBONDTATE_CHANGE
  * @}
  */

/** @defgroup TGAPBOND_PASSKEY_DISPLAY TGAPBOND_PASSKEY_DISPLAY
  * @{
  */
typedef struct _TGAPBOND_PASSKEY_DISPLAY
{
    uint16_t connHandle;
} TGAPBOND_PASSKEY_DISPLAY;
/** End of TGAPBOND_PASSKEY_DISPLAY
  * @}
  */

/** @defgroup TGAPBOND_PASSKEY_INPUT TGAPBOND_PASSKEY_INPUT
  * @{
  */
typedef struct _TGAPBOND_PASSKEY_INPUT
{
    uint16_t connHandle;
} TGAPBOND_PASSKEY_INPUT;
/** End of TGAPBOND_PASSKEY_INPUT
  * @}
  */

/** @defgroup TGAPBOND_OOB_INPUT TGAPBOND_OOB_INPUT
  * @{
  */
typedef struct _TGAPBOND_OOB_INPUT
{
    uint16_t connHandle;
} TGAPBOND_OOB_INPUT;
/** End of TGAPBOND_OOB_INPUT
  * @}
  */

/** @defgroup TGAPENCRYPT_STATE_CHANGE TGAPENCRYPT_STATE_CHANGE
  * @{
  */
typedef struct _TGAPENCRYPT_STATE_CHANGE
{
    uint16_t connHandle;
    uint8_t newState;
} TGAPENCRYPT_STATE_CHANGE;
/** End of TGAPENCRYPT_STATE_CHANGE
  * @}
  */

#ifdef BT_GAP_PARAM_TX_POWER_SET
/** @defgroup BLE_TX_POWER BLE TX POWER
  * @{
  */
typedef enum _TBLE_TX_POWER_INDEX
{
    TX_POWER_MINUS_16_DBM = 0x00,
    TX_POWER_0_DBM = 0x06,
    TX_POWER_3_DBM = 0x07,
    TX_POWER_UNKNOWN = 0xFF
} TBLE_TX_POWER_INDEX;    
/**
  * @}
  */
#endif

#ifdef BT_GAP_PARAM_TX_POWER_SET
typedef enum _TBLE_PARAM_TYPE
{
    BLE_PARAM_TYPE_TX_POWER,
}TBLE_PARAM_TYPE;
#endif

#ifdef BT_GAP_PARAM_TX_POWER_SET
typedef struct 
{
    TBLE_PARAM_TYPE bleParamType;
    BOOL    result;
}TGAPBT_PARAM_SET_RESULT;
#endif

/** @defgroup TBT_STACK_MSG_DATA TBT_STACK_MSG_DATA
  * @{
  */
typedef union
{
    TGAPCONN_STATE_CHANGE           gapConnStateChange;
    TGAPBONDTATE_CHANGE             gapBondStateChange;
    TGAPBOND_PASSKEY_DISPLAY        gapBondPasskeyDisplay;
    TGAPBOND_PASSKEY_INPUT          gapBondPasskeyInput;
    TGAPBOND_OOB_INPUT              gapBondOobInput;
    TGAPENCRYPT_STATE_CHANGE        gapEncryptStateChange;
    TGAPCONN_PARA_UPDATE_CHANGE     gapConnParaUpdateChange;
#ifdef BT_GAP_PARAM_TX_POWER_SET
    TGAPBT_PARAM_SET_RESULT         gapBTParamSetResult;
#endif
} TBT_STACK_MSG_DATA;
/** End of TBT_STACK_MSG_DATA
  * @}
  */

/** @defgroup BT_STACK_MSG BT_STACK_MSG
  * @{
  */
typedef struct _BT_STACK_MSG
{
    TBT_STACK_MSG_DATA msgData;
} BT_STACK_MSG;
/** End of BT_STACK_MSG
  * @}
  */

/** @defgroup gaprole_States_t gaprole_States_t
  * @{
  */
typedef uint8_t gaprole_States_t;
/** End of gaprole_States_t
  * @}
  */

enum {
    API_TYPE_VENDOR_CMD = 1,
    API_TYPE_READ_RSSI_CMD = 2,
    API_TYPE_READ_PATCH_VERSION = 3,
    API_TYPE_RESERVED = 0xFFFF    
};

typedef struct _TApiBufVendorCmdOnOffLatency{
    uint16_t opCode;
    uint8_t   len;
    uint8_t   latency_enable;    
}TApiBufVendorCmdOnOffLatency;

typedef struct _TApiBufVendorCmd{
    uint16_t opCode;
    uint8_t   len;
    uint8_t   para[255];    
}TApiBufVendorCmd;

typedef struct _TApiBufUserDefined{
    uint16_t Type;
    union{
        TApiBufVendorCmd ApiBufVendorCmd;
        uint16_t *pPatchVersion;
    }p;    
}TApiBufUserDefined;

bool blueAPI_ReadRSSI(uint16_t local_MDL_ID);
bool blueAPI_ReadPatchVersion(uint16_t * PatchVersion);

/** End of GAP_Exported_Types
  * @}
  */

/** End of GAP_COMMON_DEFINITIONS
  * @}
  */

/** End of RTK_GAP_MODULE
  * @}
  */

///@cond
/* GAP common APIs declared here for all roles to use. */
extern bool GAP_StartBtStack(void);
extern void GAP_SendBtMsgToApp(BEE_IO_MSG *pmsg);
extern void GAP_StateNotificationCB(gaprole_States_t new_state);
///@endcond

/*-------------------------------------------------------------------
-------------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* GAP_H */
