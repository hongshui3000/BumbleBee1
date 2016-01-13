/**
*****************************************************************************************
*     Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
  * @file     rtu_service.h
  * @brief    Variables and interfaces for using Reference Time Update Service.
  * @details  RTU data structs and functions.
  * @author   ethan_su
  * @date     2014-10-10
  * @version  v0.1
  * *************************************************************************************
  */

/* Define to prevent recursive inclusion */
#ifndef _RTU_DEF_H_
#define _RTU_DEF_H_

#ifdef  __cplusplus
extern "C" {
#endif      /* __cplusplus */

/* Add Includes here */
#include "profile_api.h"

/** @addtogroup RTK_Profile_Module RTK Profile Module
  * @{
  */

/** @defgroup RTU RTU
  * @brief Reference Time Update Service
  * @{
  */ 

/** @defgroup RTU_Exported_Constants RTU Exported Constants
  * @brief macros that other .c files may use all defined here
  * @{
  */      
/** @defgroup RTU_Application_Parameters RTU Application Parameters
  * @brief  Type of parameters set/got from application.
  * @{
  */
#define RTU_PARAM_UPDATE_CMD                0x01
#define RTU_PARAM_UPDATE_STATE              0x02
/** @} */

///@cond
/** @brief  RTU service related UUIDs. */
#define GATT_UUID_REFERENCE_TIME_UPDATE     0x1806
#define GATT_UUID_CHAR_RTUS_CONTROL_POINT   0x2A16
#define GATT_UUID_CHAR_RTUS_STATE           0x2A17

/** @brief  Index of each characteristic in RTU service database. */
#define GATT_SVC_RTU_CTL_PNT_INDEX          2
#define GATT_SVC_RTU_STATE_INDEX            4
///@endcond

/** @defgroup RTU_Upstream_Message RTU Upstream Message
  * @brief  Upstream message used to inform application.
  * @{
  */

/** @defgroup RTU_Read_Info RTU Read Info
  * @brief  Parameter for reading characteristic value.
  * @{
  */
#define RTU_READ_TIME_UPDATE_STATE                      1
/** @} */

/** @} End of RTU_Upstream_Message */
  
/** @defgroup RTU_Control_Point_Command RTU Control Point Command
  * @brief  Control Point Commands
  * @{
  */
#define RTU_CP_CMD_RESERVED                 0
#define RTU_CP_CMD_GET_REF_UPDATE           1
#define RTU_CP_CMD_CANCEL_REF_UPDATE        2
/** @} */

/** @defgroup RTU_Current_State RTU Current State
  * @brief  current state values
  * @{
  */
#define RTU_CURRENT_STATE_IDLE              0
#define RTU_CURRENT_STATE_UPDATE_PENDING    1
/** @} */

/** @defgroup RTU_Time_Update_Result RTU Time Update Result
  * @brief  reference time update results
  * @{
  */
#define RTU_RESULT_SUCCESS                  0
#define RTU_RESULT_CANCELED                 1
#define RTU_RESULT_NO_CONNECTION_TO_REF     2
#define RTU_RESULT_REF_RSP_WITH_ERROR       3
#define RTU_RESULT_TIMEOUT                  4
#define RTU_RESULT_UPDATE_NOT_ATTEMPTED     5
/** @} */

/** @} End of RTU_Exported_Constants */

/** @defgroup RTU_Exported_Types RTU Exported Types
  * @brief  types that other.c files may use all defined here
  * @{
  */
/* Add all public types here */
/** @brief Time Update Control Point definition. */
typedef struct _TimeUpdateCtlPnt
{
    uint8_t             cmd; //!< ref: @ref RTU_Control_Point_Command
} TTimeUpdateCtlPnt, * PTimeUpdateCtlPnt;

/** @brief Time Update State definition. */
typedef struct _TimeUpdateState
{
    uint8_t             current_state; //!< ref: @ref RTU_Current_State
    uint8_t             result; //!< ref: @ref RTU_Time_Update_Result
} TTimeUpdateState, * PTimeUpdateState;

/** @defgroup RTU_Callback_Data RTU Callback Data
  * @brief RTU data struct for notification data to application.
  * @{
  */
typedef struct _TRTU_WRITE_MSG{
    uint8_t opcode; //!< ref: @ref RTU_Control_Point_Command
}TRTU_WRITE_MSG;

typedef union _TRTU_UPSTREAM_MSG_DATA
{
    uint8_t read_value_index; //!< ref: @ref RTU_Read_Info
    TRTU_WRITE_MSG write;
}TRTU_UPSTREAM_MSG_DATA;

/** RTU service data to inform application */
typedef struct _TRTU_CALLBACK_DATA
{
    TSERVICE_CALLBACK_TYPE     msg_type;
    TRTU_UPSTREAM_MSG_DATA    msg_data;
} TRTU_CALLBACK_DATA;
/** @} */

/** @} End of RTU_Exported_Types */

/** @defgroup RTU_Exported_Functions RTU Exported Functions
  * @brief functions that other .c files may use all defined here
  * @{
  */
/* set RTU Service related data from application */
extern bool RTU_SetParameter( uint8_t param_type, uint8_t len, void *value_ptr);
/* get RTU Service related data from application */
extern bool RTU_GetParameter( uint8_t param_type, void *value_ptr);
/* add RTU Service to application */
extern uint8_t RTU_AddService(void* pFunc);
/** @} End of RTU_Exported_Functions */

/** @} End of RTU */

/** @} End of RTK_Profile_Module */
#ifdef  __cplusplus
}
#endif      /*  __cplusplus */

#endif  /* _RTU_DEF_H_ */

