/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       bttypes.h
* @brief     BLUETOOTH global type definitions  
* @details   
*
* @author   	gordon
* @date      	2015-07-13
* @version	v0.1
*/

#ifndef __BTTYPES_H
#define __BTTYPES_H

#ifndef __BASETYPE_H
#include <rtl_types.h>
#endif

/** Bluetooth NAME LENGTH */
#define NAME_LENGTH                 248   /**< Max. Length of Bluetooth Name */

/** Bluetooth device address */
#define BD_ADDR_SIZE      6

typedef uint8_t TBdAddr[BD_ADDR_SIZE];
typedef CONST uint8_t TCBdAddr[BD_ADDR_SIZE];
typedef TBdAddr  * LPBdAddr;
typedef TCBdAddr  * LPCBdAddr;

/** Bluetooth Class of Device */
#define BT_CLASS_SIZE     3
typedef uint8_t TDevClass[BT_CLASS_SIZE];

/** Bluetooth LinkKey */
#define LINK_KEY_SIZE               16
typedef uint8_t TLinkKey[LINK_KEY_SIZE];

/** Bluetooth App Profile  */
#define BT_PROFILE_NAME_LENGTH        32
#define BT_PROFILE_DESCRIPTION_LENGTH 32

typedef enum
{
	stringsValid       = 0x01,
	extValid           = 0x02,
	useRedirect        = 0x04,
	shared             = 0x08,
	predefined         = 0x10
} TProfileFlags;
typedef TProfileFlags* PProfileFlags;
typedef TProfileFlags  * LPProfileFlags;

typedef enum
{
	noError = 0,
	profileUnknown,        /**< releaseConf, releaseResp, changeResp        */
	profileUsed,           /**< registerConf                                */
	profileNotSupported,   /**< registerConf                                */
	invalidParameter,      /**< registerConf, profChangeResp                */
	invalidFlags
} TProfileError;
typedef TProfileError* PProfileError;
typedef TProfileError * LPProfileError;

typedef enum
{
	visable            = 0x01,
	audioFeedback      = 0x02
} TProfileExtValidFlag;
typedef TProfileExtValidFlag* PProfileExtValidFlag;
typedef TProfileExtValidFlag * LPProfileExtValidFlag;

typedef struct
{
	TProfileExtValidFlag extValidFlags; /**< validation of existence of    */
                                        /**< ext. parameter                */
	BOOL  visable;                      /**< discoverable via SDP          */
	BOOL  audioFeedback;                /**< DUN                           */
} TProfileExt;
typedef TProfileExt* PProfileExt;
typedef TProfileExt * LPProfileExt;

typedef enum
{
	serviceName        = 0x01,
	serviceDescription = 0x02
} TProfileStringsValidFlag;
typedef TProfileStringsValidFlag* PProfileStringsValidFlag;
typedef TProfileStringsValidFlag * LPProfileStringsValidFlag;

typedef struct
{
	TProfileStringsValidFlag stringsValidFlag;

	uint8_t serviceName[BT_PROFILE_NAME_LENGTH+1];
	uint8_t serviceDescription[BT_PROFILE_DESCRIPTION_LENGTH+1];
} TBTProfileStrings;
typedef TBTProfileStrings* PBTProfileStrings;
typedef TBTProfileStrings * LPBTProfileStrings;

typedef struct
{
	uint16_t handle;                   
	uint16_t uuid;                     /**< if 0 -> for all uuid     */
	uint8_t serverChannel;            /**< if 0 -> for all channels */
	uint16_t psm;                      /**< if 0 -> for all psm's    */
} TBTProfile;
typedef TBTProfile* PBTProfile;
typedef TBTProfile * LPBTProfile;

typedef struct
{
	TProfileFlags     profileFlags;
	TBTProfile        profile;
	TBTProfileStrings strings;
	TProfileExt       ext;
} TBTAppProfile;
typedef TBTAppProfile * PBTAppProfile;
typedef TBTAppProfile * LPBTAppProfile;
#endif  /**<__BTTYPES_H */
