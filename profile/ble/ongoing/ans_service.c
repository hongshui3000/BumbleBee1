enum { __FILE_NUM__ = 0 };

/**
*********************************************************************************************************
*               Copyright(c) 2014, Realtek Semiconductor Corporation. All rights reserved.
**********************************************************************************************************
* @file     ans_service.c
* @brief    Alert Notifications Service interfaces.
* @details  Alert Notifications Service interfaces.
* @author   ethan_su
* @date     2015-04-13
* @version  v0.1
*********************************************************************************************************
*/

#include "profile.h"
#include "ans_service.h"
#include "stdint.h"
#include "stddef.h"
#include "gatt.h"


#include"trace.h"
#include"profile_api.h"


extern uint32_t gSelectModeFlag;

extern APP_CALLBACK gAppCallback;




const TAttribAppl gattdANSService[] =
{
    /*-----------------  Alert Notification Service -------------------*/
    {
        (ATTRIB_FLAG_VALUE_INCL | ATTRIB_FLAG_LE),  /* wFlags     */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_PRIMARY_SERVICE),
            HI_WORD(GATT_UUID_PRIMARY_SERVICE),
            LO_WORD(GATT_UUID_ALERT_NOTIFICATION_SERVICE),    /* service UUID */
            HI_WORD(GATT_UUID_ALERT_NOTIFICATION_SERVICE)
        },
        UUID_16BIT_SIZE,                            /* bValueLen     */
        NULL,                                       /* pValueContext */
        GATT_PERM_READ                              /* wPermissions  */
    },

    /* Supported New Alert Category Characteristic*/
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_READ,              /* characteristic properties */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },

    /* Supported New Alert Category Characteristic value  */
    {
        ATTRIB_FLAG_VALUE_APPL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHAR_ANS_NEW_ALERT_CATEGARY),
            HI_WORD(GATT_UUID_CHAR_ANS_NEW_ALERT_CATEGARY)
        },
        0,                                          /* variable size */
        NULL,
        GATT_PERM_READ                             /* wPermissions */
    },

    /* New Alert Characteristic*/
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_NOTIFY,                /* characteristic properties */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },

    /* SNew Alert Characteristic value    */
    {
        ATTRIB_FLAG_VALUE_APPL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHAR_ANS_NEW_ALERT),
            HI_WORD(GATT_UUID_CHAR_ANS_NEW_ALERT)
        },
        0,                                          /* variable size */
        NULL,
        GATT_PERM_READ                             /* wPermissions */
    },
    /* client characteristic configuration */
    {
        ATTRIB_FLAG_VALUE_INCL | ATTRIB_FLAG_CCCD_APPL,                   /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHAR_CLIENT_CONFIG),
            HI_WORD(GATT_UUID_CHAR_CLIENT_CONFIG),
            /* NOTE: this value has an instantiation for each client, a write to */
            /* this attribute does not modify this default value:                */
            LO_WORD(GATT_CLIENT_CHAR_CONFIG_DEFAULT), /* client char. config. bit field */
            HI_WORD(GATT_CLIENT_CHAR_CONFIG_DEFAULT)
        },
        2,                                          /* bValueLen */
        NULL,
        (GATT_PERM_READ | GATT_PERM_WRITE)          /* wPermissions */
    },
    /* Supported Unread Alert Category Characteristic*/
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_READ,              /* characteristic properties */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },

    /* Supported Unread Alert Category Characteristic value  */
    {
        ATTRIB_FLAG_VALUE_APPL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHAR_ANS_UNREAD_NEW_ALERT_CATEGARY),
            HI_WORD(GATT_UUID_CHAR_ANS_UNREAD_NEW_ALERT_CATEGARY)
        },
        0,                                          /* variable size */
        NULL,
        GATT_PERM_READ                             /* wPermissions */
    },
    /* Unread Alert Status Characteristic*/
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                            /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_NOTIFY,            /* characteristic properties */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },

    /* Unread Alert Status Characteristic value  */
    {
        ATTRIB_FLAG_VALUE_APPL,                     /* wFlags */
        {                                            /* bTypeValue */
            LO_WORD(GATT_UUID_CHAR_ANS_UNREAD_ALERT_STATUS),
            HI_WORD(GATT_UUID_CHAR_ANS_UNREAD_ALERT_STATUS)
        },
        0,                                          /* variable size */
        NULL,
        GATT_PERM_READ                             /* wPermissions */
    },
    /* client characteristic configuration */
    {
        ATTRIB_FLAG_VALUE_INCL | ATTRIB_FLAG_CCCD_APPL,                   /* wFlags */
        {                                            /* bTypeValue */
            LO_WORD(GATT_UUID_CHAR_CLIENT_CONFIG),
            HI_WORD(GATT_UUID_CHAR_CLIENT_CONFIG),
            /* NOTE: this value has an instantiation for each client, a write to */
            /* this attribute does not modify this default value:                */
            LO_WORD(GATT_CLIENT_CHAR_CONFIG_DEFAULT), /* client char. config. bit field */
            HI_WORD(GATT_CLIENT_CHAR_CONFIG_DEFAULT)
        },
        2,                                          /* bValueLen */
        NULL,
        (GATT_PERM_READ | GATT_PERM_WRITE)          /* wPermissions */
    },
    /* Alert Notification Control Point Characteristic*/
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                         /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_WRITE,             /* characteristic properties */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },

    /* Alert Notification Control Point value  */
    {
        ATTRIB_FLAG_VALUE_APPL,                     /* wFlags */
        {                                         /* bTypeValue */
            LO_WORD(GATT_UUID_CHAR_ANS_CONTROL_POINT),
            HI_WORD(GATT_UUID_CHAR_ANS_CONTROL_POINT)
        },
        0,                                          /* variable size */
        NULL,
        GATT_PERM_READ                             /* wPermissions */
    },

};

const int32_t iGattANSServiceSize = sizeof(gattdANSService);

uint8_t* g_PAlertCateValue = NULL;
uint32_t g_AlertCateValueLen = 0;

uint8_t* g_PUnreadAlertCateValue = NULL;
uint32_t g_UnreadAlertCateValueLen = 0;


uint8_t* g_PNewAlertValue = NULL;
uint32_t g_NewAlertValueLen = 0;

uint8_t gUnreadAlertStatus[2] = {0};


uint8_t *  gattAttribGetValue_ANS( int32_t iAttribIndex, int32_t iOffset, int32_t * piLength )
{
    uint8_t * pValue = NULL;

    THogpDemoAppData data = {0};
    *piLength = 0;

    switch (iAttribIndex)
    {
    case GATT_ANS_SUPPORT_NEW_ALERT_CTGRY_CHAR_VALUE_INDEX://read
        data.iParameterCount = 0;
        data.eventId = ANS_READ_SUPPORT_NEW_ALERT_CTGRY_EVT;
        if (gAppCallback)
        {
            gAppCallback((void*)&data);
        }
        pValue = g_PAlertCateValue;
        *piLength = g_AlertCateValueLen;
        break;
    case GATT_ANS_SUPPORT_UNREAD_ALERT_CHAR_VALUE_INDEX://read
        data.iParameterCount = 0;
        data.eventId = ANS_READ_SUPPORT_UNREAD_ALERT_EVT;
        if (gAppCallback)
        {
            gAppCallback((void*)&data);
        }
        pValue = g_PUnreadAlertCateValue;
        *piLength = g_UnreadAlertCateValueLen;
        break;
    case GATT_ANS_SUPPORT_NEW_ALERT_CHAR_VALUE_INDEX://Notification
        pValue = g_PNewAlertValue;
        *piLength = g_NewAlertValueLen;
        break;
    case GATT_ANS_SUPPORT_UNREAD_ALERT_STATUS_CHAR_VALUE_INDEX://Notification
        pValue = &gUnreadAlertStatus;
        *piLength = 2;
        break;
    }
    if(pValue)
    {
        DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "<-- gattAttribGetValue_ANS  iAttribIndex=%d value=%d  ",
                   2,
                   iAttribIndex,
                   *pValue);
    }
    return ( pValue );
}

uint16_t gattAttribSetValue_ANS(int32_t iAttribIndex, uint8_t * pValue, uint16_t wLength )
{
    uint16_t   wCause  = GATT_SUCCESS;
    THogpDemoAppData data = {0};

    if (GATT_ANS_SUPPORT_ALERT_NOTIFICATION_CONTROL_POINT_CHAR_VALUE_INDEX == iAttribIndex)
    {
        if ( wLength > sizeof(TANSControlPoint) )
        {
            wCause  = ATT_ERR | ATT_ERR_INVALID_VALUE_SIZE;
        }
        else
        {
            data.sParas[0] = *pValue;
            data.sParas[1] = *(pValue + 1);
            data.iParameterCount = 1;
            data.eventId = ANS_CONTROL_POINT_EVT;

            if (gAppCallback)
            {
                gAppCallback((void*)&data);
            }
        }
    }

    return wCause;
}


