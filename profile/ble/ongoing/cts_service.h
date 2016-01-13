/**
*****************************************************************************************
*     Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
  * @file     cts_service.h
  * @brief    Variables and interfaces for using current time service.
  * @details  current time service data structs and functions.
  * @author   ethan_su
  * @date     2015-04-09
  * @version  v0.1
  * *************************************************************************************
  */

/* Define to prevent recursive inclusion */
#ifndef _CTS_SERVICE_DEF_H
#define _CTS_SERVICE_DEF_H

#ifdef  __cplusplus
extern "C" {
#endif      /* __cplusplus */
    
/* Add Includes here */
#include "profile_api.h"

/** @addtogroup RTK_Profile_Module RTK Profile Module
  * @{
  */

/** @defgroup CTS CTS
  * @brief Current Time Service
  * @{
  */ 

/** @defgroup CTS_Exported_Constants CTS Exported Constants
  * @brief macros that other .c files may use all defined here
  * @{
  */      
/** @defgroup CTS_Application_Parameters CTS Application Parameters
  * @brief  Type of parameters set/got from application.
  * @{
  */
#define CTS_PARAM_CUR_TIME                                  0x01
#define CTS_PARAM_LOCAL_TIME                                0x02
#define CTS_PARAM_REF_TIME                                  0x03
/** @} */

///@cond
/** @brief  CTS service related UUIDs. */
#define GATT_UUID_CURRENT_TIME                              0x1805
#define GATT_UUID_CHAR_CTS_CURRENT_TIME                     0x2A2B
#define GATT_UUID_CHAR_CTS_LOCAL_TIME_INFO                  0x2A0F
#define GATT_UUID_CHAR_CTS_REF_TIME_INFO                    0x2A14

/** @brief  Index of each characteristic in CTS service database. */
#define GATT_SVC_CTS_CUR_TIME_INDEX                         2
#define GATT_SVC_CTS_LOCAL_TIME_INDEX                       5
#define GATT_SVC_CTS_REF_TIME_INDEX                         7
#define GATT_SVC_CTS_CUR_TIME_CCCD_INDEX   (GATT_SVC_CTS_CUR_TIME_INDEX + 1)
///@endcond

/** @defgroup CTS_Upstream_Message CTS Upstream Message
  * @brief  Upstream message used to inform application.
  * @{
  */

/** @defgroup CTS_Read_Info CTS Read Info
  * @brief  Parameter for reading characteristic value.
  * @{
  */
#define CTS_READ_CUR_TIME                                   1
#define CTS_READ_LOCAL_TIME                                 2
#define CTS_READ_REGERENCE_TIME                             3
/** @} */

/** @defgroup CTS_Write_Info CTS Write Info
  * @brief  Parameter for writing characteristic value.
  * @{ 
  */
#define CTS_WRITE_CUR_TIME                                  1
#define CTS_WRITE_LOCAL_TIME                                1
/** @} */

/** @defgroup CTS_Notify_Indicate_Info CTS Notify Indicate Info
  * @brief  Parameter for enable or disable notification or indication.
  * @{
  */
#define CTS_NOTIFY_INDICATE_CUR_TIME_ENABLE                 1
#define CTS_NOTIFY_INDICATE_CUR_TIME_DISABLE                2
/** @} */

/** @} End of CTS_Upstream_Message */

/** @defgroup CTS_Week CTS Week
  * @brief  Day of the Week values
  * @{
  */
#define CTS_WEEKDAY_UNKNOWN                                 0
#define CTS_WEEKDAY_MONDAY                                  1
#define CTS_WEEKDAY_TUESDAY                                 2
#define CTS_WEEKDAY_WEDNESAY                                3
#define CTS_WEEKDAY_THURSDAY                                4
#define CTS_WEEKDAY_FRIDAY                                  5
#define CTS_WEEKDAY_SATURDAY                                6
#define CTS_WEEKDAY_SUNDAY                                  7
/** @} */

/** @defgroup CTS_Adjust_Reason CTS Adjust Reason
  * @brief  Adjust Reason bits
  * @{
  */
#define CTS_ADJUST_NOT_PRESENT                              0
#define CTS_ADJUST_MANUAL_TIME_UPDATE                       1
#define CTS_ADJUST_EXT_REF_TIME_UPDATE                      2
#define CTS_ADJUST_CHANGE_OF_TIME_ZONE                      4
#define CTS_ADJUST_CHANGE_OF_DST                            8
/** @} */

/** @defgroup CTS_DST_Offset CTS DST Offset
  * @brief  DST Offset bits
  * @{
  */
#define CTS_DST_OFFSET_STANDARD                             0
#define CTS_DST_OFFSET_HALF_AN_HOUR                         2
#define CTS_DST_OFFSET_ONE_HOUR                             4
#define CTS_DST_OFFSET_TWO_HOURS                            8
/** @} */

/** @defgroup CTS_Time_Source CTS Time Source
  * @brief  Time Source bits
  * @{
  */
#define CTS_TIME_SOURCE_UNKNOWN                             0
#define CTS_TIME_SOURCE_NTP                                 1   /* Network Time Protocol */
#define CTS_TIME_SOURCE_GPS                                 2
#define CTS_TIME_SOURCE_RADIO                               3
#define CTS_TIME_SOURCE_MANUAL                              4
#define CTS_TIME_SOURCE_ATOMIC_CLOCK                        5
#define CTS_TIME_SOURCE_CELLULAR_NET                        6
/** @} */

///@cond
/** @brief   Time Accuracy bits. */
#define CTS_TIME_ACCURACY_OUT_OF_RANGE                      0xFE
#define CTS_TIME_ACCURACY_UNKNOWN                           0xFF

/** @brief  Error codes defined in CTS service. */
#define CTS_ERR_DATA_FIELD_IGNORED                          0x80
///@endcond

/** @} End of CTS_Exported_Constants */

/** @defgroup CTS_Exported_Types CTS Exported Types
  * @brief  types that other.c files may use all defined here
  * @{
  */

/* Add all public types here */
/** @brief Time Stamp data: Time Stamp include year, month, day, hour, minute, second infomation. */
typedef struct _DATE_TIME
{
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
} _PACKED_ DATE_TIME;

/**
 * @brief Date Time data definition.
 *
*/
typedef struct _DayDateTime
{
    DATE_TIME         date_time;
    uint8_t           day_of_the_week;
} _PACKED_ TDayDateTime, * PDayDateTime;

/**
 * @brief Exact time 256 definition.
 *
*/
typedef struct _ExactTime256
{
    TDayDateTime      day_date_time;
    uint8_t           fractions256;
}_PACKED_ TExactTime256, * PExactTime256;

/**
 * @brief CTS Current Time value.
 *
*/
typedef struct _CurrentTime
{
    TExactTime256     exact_time_256;
    uint8_t           adjust_reason;
}_PACKED_ TCurrentTime, * PCurrentTime;

/**
 * @brief CTS Local Time Information (optional) definitions.
 *
*/
typedef struct _LocalTimeInfo
{
    INT8              time_zone;        /**<  UTC+f(value>), value=-48,..,56. */
    uint8_t           DST_offset;
}_PACKED_ TLocalTimeInfo, * PLocalTimeInfo;

/**
 * @brief CTS Reference Time Information (optional) definitions.
 *
*/
typedef struct _ReferenceTimeInfo
{
    uint8_t       source;
    uint8_t       accuracy;
    uint8_t       days_since_update;
    uint8_t       hours_since_update;
}_PACKED_ TReferenceTimeInfo, * PReferenceTimeInfo;

/** @defgroup CTS_Callback_Data CTS Callback Data
  * @brief CTS data struct for notification data to application.
  * @{
  */
typedef union _TCTS_WRITE_MSG{
    TCurrentTime cur_time;
    TLocalTimeInfo local_time;
}TCTS_WRITE_MSG;

typedef union _TCTS_UPSTREAM_MSG_DATA
{
    uint8_t notification_indification_index; //!< ref: @ref CTS_Notify_Indicate_Info
    uint8_t read_value_index; //!< ref: @ref CTS_Read_Info
    TCTS_WRITE_MSG write;
}TCTS_UPSTREAM_MSG_DATA;

/**
 * @brief CTS data struct for notification data to application.
 *
 * CTS service data to inform application.
*/
typedef struct _TCTS_CALLBACK_DATA
{
    TSERVICE_CALLBACK_TYPE     msg_type;
    TCTS_UPSTREAM_MSG_DATA    msg_data;
} TCTS_CALLBACK_DATA;
/** @} */

/** @} End of CTS_Exported_Types */

/** @defgroup CTS_Exported_Functions CTS Exported Functions
  * @brief functions that other .c files may use all defined here
  * @{
  */
/* set CTS Service related data from application */
extern bool CTS_SetParameter( uint8_t param_type, uint8_t length, void *value_ptr );
/* get CTS Service related data from application */
extern bool CTS_GetParameter( uint8_t param_type, void *value_ptr );
/* notify Current Time data from application */
extern bool CTS_CurTimeNotify( uint8_t ServiceId );
/* add CTS Service to application */
extern uint8_t CTS_AddService(void* pFunc);
/** @} End of CTS_Exported_Functions */

/** @} End of CTS */

/** @} End of RTK_Profile_Module */

#ifdef  __cplusplus
}
#endif      /*  __cplusplus */

#endif  /* _CTS_DEF_H */

