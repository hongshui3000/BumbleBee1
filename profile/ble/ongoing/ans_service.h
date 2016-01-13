/**
************************************************************************************************************
*               Copyright(c) 2014, Realtek Semiconductor Corporation. All rights reserved.
************************************************************************************************************
* @file     ans_service.h
* @brief    Variables and interfaces for using Alert Notifications Service.
* @details  Alert Notifications Service data structs and functions.
* @author   ethan_su
* @date     2015-04-13
* @version  v0.1
*************************************************************************************************************
*/
#ifndef _ANS_SERVICE_DEF_H
#define _ANS_SERVICE_DEF_H

#include "profile.h"
#define GATT_ANS_SUPPORT_NEW_ALERT_CTGRY_CHAR_VALUE_INDEX   2
#define GATT_ANS_SUPPORT_NEW_ALERT_CHAR_VALUE_INDEX   4
#define GATT_ANS_SUPPORT_UNREAD_ALERT_CHAR_VALUE_INDEX   7
#define GATT_ANS_SUPPORT_UNREAD_ALERT_STATUS_CHAR_VALUE_INDEX   9
#define GATT_ANS_SUPPORT_ALERT_NOTIFICATION_CONTROL_POINT_CHAR_VALUE_INDEX   12


typedef struct _ANSControlPoint
{
    uint8_t CommandId;
    uint8_t Category;
} TANSControlPoint, * PANSControlPoint;

uint8_t *  gattAttribGetValue_ANS( int32_t iAttribIndex, int32_t iOffset, int32_t * piLength );
uint16_t gattAttribSetValue_ANS(int32_t iAttribIndex, uint8_t * pValue, uint16_t wLength );



#endif // _ANS_SERVICE_DEF_H

