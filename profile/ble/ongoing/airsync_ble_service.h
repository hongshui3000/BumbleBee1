/**
*****************************************************************************************
*     Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
  * @file     airsync_ble_service.h
  * @brief    Demonstration of how to implement a self-definition service.
  * @details  Demonstration of different kinds of service interfaces.
  * @author   ethan
  * @date     2015-5-10
  * @version  v0.1
  * *************************************************************************************
  */

/* Define to prevent recursive inclusion */
#ifndef _AIRSYNC_BLE_SERVICE_H_
#define _AIRSYNC_BLE_SERVICE_H_

#ifdef __cplusplus
extern "C"  {
#endif      /* __cplusplus */

/* Add Includes here */
#include "profile_api.h"

/** @addtogroup RTK_Profile_Module RTK Profile Module
  * @{
  */

/** @defgroup SIMP SIMP
  * @brief Simple BLE Service
  * @{
  */ 

/** @defgroup SIMP_Exported_Constants SIMP Exported Constants
  * @brief macros that other .c files may use all defined here
  * @{
  */      
/** @defgroup SIMP_Application_Parameters SIMP Application Parameters
  * @brief  Type of parameters set/got from application.
  * @{
  */
#define SIMPLE_BLE_SERVICE_PARAM_V1_READ_CHAR_VAL           0x01
#define SIMPLE_BLE_SERVICE_PARAM_CTL_PNT_PROG_CLR           0x02
/** @} */

///@cond
/** @brief  Demo Profile service related UUIDs. */
#define GATT_UUID_AIRSYNC_SERVICE                           0xFEE7
#define GATT_UUID_CHAR_SIMPLE_V2_WRITE                      0xFEC7
#define GATT_UUID_CHAR_SIMPLE_V4_INDICATE                   0xFEC8
#define GATT_UUID_CHAR_SIMPLE_V1_READ                       0xFEC9


/** @brief  Index of each characteristic in Demo Profile service database. */
#define AIRSYNC_BLE_SERVICE_CHAR_WRITE_INDEX              0x02
#define AIRSYNC_BLE_SERVICE_CHAR_INDICATE_INDEX           0x04
#define AIRSYNC_BLE_SERVICE_CHAR_READ_INDEX               0x06

#define AIRSYNC_BLE_SERVICE_CHAR_INDICATE_CCCD_INDEX (AIRSYNC_BLE_SERVICE_CHAR_INDICATE_INDEX + 1)

///@endcond

/** @defgroup SIMP_Upstream_Message SIMP Upstream Message
  * @brief  Upstream message used to inform application.
  * @{
  */

/** @defgroup SIMP_Read_Info SIMP Read Info
  * @brief  Parameter for reading characteristic value.
  * @{
  */
#define AIRSYNC_READ_V1                                        1
/** @} */

/** @defgroup SIMP_Write_Info SIMP Write Info
  * @brief  Parameter for writing characteristic value.
  * @{ 
  */
#define AIRSYNC_WRITE_V2                                       1

/** @} */

/** @defgroup SIMP_Notify_Indicate_Info SIMP Notify Indicate Info
  * @brief  Parameter for enable or disable notification or indication.
  * @{
  */

#define AIRSYNC_NOTIFY_INDICATE_V4_ENABLE     3
#define AIRSYNC_NOTIFY_INDICATE_V4_DISABLE    4

/** @} */

/** @} End of SIMP_Upstream_Message */



/** @} End of SIMP_Exported_Constants */

/** @defgroup SIMP_Exported_Types SIMP Exported Types
  * @brief  types that other.c files may use all defined here
  * @{
  */

/* Add all public types here */
/** @defgroup SIMP_Callback_Data SIMP Callback Data
  * @brief SIMP data struct for notification data to application.
  * @{
  */
/** Write content */
typedef struct _TSIMP_WRITE_MSG{
    uint8_t opcode; //!< ref: @ref SIMP_Control_Point_OpCodes, @ref SIMP_Write_Info
    uint8_t value;
}TSIMP_WRITE_MSG;

/** Pre Write content */
typedef struct _TSIMP_PRE_WRITE_MSG{
    uint8_t opcode; //!< ref: @ref SIMP_Control_Point_OpCodes, @ref SIMP_Write_Info
	uint16_t Len;
	uint8_t* pValue;
}TSIMP_PRE_WRITE_MSG;

/** Message content: @ref SIMP_Upstream_Message */
typedef union _TSIMP_UPSTREAM_MSG_DATA
{
    uint8_t notification_indification_index; //!< ref: @ref SIMP_Notify_Indicate_Info
    uint8_t read_value_index; //!< ref: @ref SIMP_Read_Info
    TSIMP_PRE_WRITE_MSG pre_write;
	TSIMP_WRITE_MSG write;
}TSIMP_UPSTREAM_MSG_DATA;

/** Simple profile service data to inform application */
typedef struct _TSIMP_CALLBACK_DATA
{
    TSERVICE_CALLBACK_TYPE     msg_type;
    TSIMP_UPSTREAM_MSG_DATA    msg_data;
} TSIMP_CALLBACK_DATA;
/** @} */

/** @} End of SIMP_Exported_Types */

/** @defgroup SIMP_Exported_Functions SIMP Exported Functions
  * @brief functions that other .c files may use all defined here
  * @{
  */
/* set Simple Profile related data from application */
extern bool AirSyncBleService_SetParameter( uint8_t param_type, uint8_t len, void *value_ptr);
/* add Simple Profile to application */
extern uint8_t AirSyncBleService_AddService(void* pFunc);

/* indicate current v4 data from application */
extern bool AirSyncBleService_SimpleV4Indicate( uint8_t ServiceId, uint8_t *pData, uint32_t data_len);

/** @} End of SIMP_Exported_Functions */

/** @} End of SIMP */

/** @} End of RTK_Profile_Module */

#ifdef __cplusplus
}
#endif

#endif /* _SIMPLE_BLE_SERVICE_H_ */
