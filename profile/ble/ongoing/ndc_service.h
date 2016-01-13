/**
*****************************************************************************************
*     Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
  * @file     ndc_service.h
  * @brief    Variables and interfaces for using Next DST Change Service.
  * @details  NDC data structs and functions.
  * @author   ethan_su
  * @date     2014-10-10
  * @version  v0.1
  * *************************************************************************************
  */

/* Define to prevent recursive inclusion */
#ifndef _NDC_SERVICE_DEF_H
#define _NDC_SERVICE_DEF_H

#ifdef  __cplusplus
extern "C" {
#endif      /* __cplusplus */

/* Add Includes here */
#include "profile_api.h"

/** @addtogroup RTK_Profile_Module RTK Profile Module
  * @{
  */

/** @defgroup NDC NDC
  * @brief Next DST Change Service
  * @{
  */ 

/** @defgroup NDC_Exported_Constants NDC Exported Constants
  * @brief macros that other .c files may use all defined here
  * @{
  */      
/** @defgroup NDC_Application_Parameters NDC Application Parameters
  * @brief  Type of parameters set/got from application.
  * @{
  */
#define NDC_PARAM_TIME_WITH_DST             0x01
/** @} */

///@cond
/** @brief  DST Offset bits. */
#define NDC_DST_OFFSET_STANDARD             0
#define NDC_DST_OFFSET_HALF_AN_HOUR         2
#define NDC_DST_OFFSET_ONE_HOUR             4
#define NDC_DST_OFFSET_TWO_HOURS            8

/** @brief  NDC service related UUIDs. */
#define GATT_UUID_NEXT_DST_CHANGE           0x1807
#define GATT_UUID_CHAR_NDCS_TIME_WITH_DST   0x2A11

/** @brief  Index of each characteristic in NDC service database. */
#define GATT_SVC_NDC_TIME_WITH_DST_INDEX    2
///@endcond

/** @defgroup NDC_Upstream_Message NDC Upstream Message
  * @brief  Upstream message used to inform application.
  * @{
  */

/** @defgroup NDC_Read_Info NDC Read Info
  * @brief  Parameter for reading characteristic value.
  * @{
  */
#define NDC_READ_TIME_WITH_DST  1
/** @} */

/** @} End of NDC_Upstream_Message */

/** @} End of NDC_Exported_Constants */

/** @defgroup NDC_Exported_Types NDC Exported Types
  * @brief  types that other.c files may use all defined here
  * @{
  */

/* Add all public types here */
/** @brief Time Stamp data: Time Stamp include year, month, day, hour, minute, second infomation. */
typedef uint8_t     TIMESTAMP[7];

/** @brief Next DST Change (mandatory) definitions */
typedef struct _TimeWithDST
{
    TIMESTAMP         data_time;
    uint8_t           DST_offset;
} TTimeWithDST, * PTimeWithDST;

/** @defgroup NDC_Callback_Data NDC Callback Data
  * @brief NDC data struct for notification data to application.
  * @{
  */
typedef union _TNDC_UPSTREAM_MSG_DATA
{
    uint8_t read_value_index; //!< ref: @ref NDC_Read_Info
}TNDC_UPSTREAM_MSG_DATA;

typedef struct _TNDC_CALLBACK_DATA
{
    TSERVICE_CALLBACK_TYPE     msg_type;
    TNDC_UPSTREAM_MSG_DATA    msg_data;
} TNDC_CALLBACK_DATA;
/** @} */

/** @} End of NDC_Exported_Types */

/** @defgroup NDC_Exported_Functions NDC Exported Functions
  * @brief functions that other .c files may use all defined here
  * @{
  */
/* set NDC Service related data from application */
extern bool NDC_SetParameter( uint8_t param_type, uint8_t len, void *value_ptr);
/* get NDC Service related data from application */
extern bool NDC_GetParameter( uint8_t param_type, void *value_ptr);
/* add NDC Service to application */
extern uint8_t NDC_AddService(void* pFunc);
/** @} End of NDC_Exported_Functions */

/** @} End of NDC */

/** @} End of RTK_Profile_Module */
#ifdef  __cplusplus
}
#endif      /*  __cplusplus */

#endif  /* _NDC_DEF_H */

