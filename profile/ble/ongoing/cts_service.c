enum { __FILE_NUM__ = 0 };

/**
*********************************************************************************************************
*               Copyright(c) 2014, Realtek Semiconductor Corporation. All rights reserved.
**********************************************************************************************************
* @file     cts_service.c
* @brief    Current Time interfaces.
* @details  Current Time interfaces.
* @author   ethan_su
* @date     2015-04-09
* @version  v0.1
*********************************************************************************************************
*/
#include "trace.h"
#include <string.h>
#include "FreeRTOS.h"
#include "gatt.h"
#include "profile_api.h"
#include "cts_service.h"

/********************************************************************************************************
* local static variables defined here, only used in this source file.
********************************************************************************************************/
/**<  CTS current time data. */
static TCurrentTime CTS_CurrentTime =   
                    {
                        { /* ExactTime256  */
                            { /* DayDateTime */
                                { /* DateTime  */
                                    2015,  /* YY:YY */
                                    06,        /* MM */
                                    01,        /* DD */
                                    18,        /* HH */
                                    42,        /* MM */
                                    15         /* SS */
                                },
                                CTS_WEEKDAY_MONDAY
                            },
                            0  /* Fractions256 */
                        },
                        CTS_ADJUST_MANUAL_TIME_UPDATE
                    };
/**<  CTS local time infomation data. */
static TLocalTimeInfo CTS_LocalTimeInfo = 
                        {
                            -4,                   /* 0: UTC+0:00 / -4: UTC-1:00 / -8: UTC-2:00 */
                            CTS_DST_OFFSET_ONE_HOUR
                        };
/**<  CTS reference time infomation data. */
static TReferenceTimeInfo CTS_RefTimeInfo = 
                            {
                                CTS_TIME_SOURCE_MANUAL,
                                CTS_TIME_ACCURACY_UNKNOWN,
                                5,  /* DaysSinceUpdate  */
                                7   /* HoursSinceUpdate */
                            };
/**<  Function pointer used to send event to application from CTS profile. Initiated in CTS_AddService. */
static pfnAPPHandleInfoCB_t pfnCtsCB = NULL;

/**< @brief  profile/service definition.  */
const TAttribAppl CtsAttrTbl[] =
{
    /*-------------------------- Current Time Service ---------------------------*/
    /* <<Primary Service>>, ..  Index 0 */
    {
        (ATTRIB_FLAG_VALUE_INCL | ATTRIB_FLAG_LE),  /* wFlags     */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_PRIMARY_SERVICE),
            HI_WORD(GATT_UUID_PRIMARY_SERVICE),
            LO_WORD(GATT_UUID_CURRENT_TIME),          /* service UUID */
            HI_WORD(GATT_UUID_CURRENT_TIME)
        },
        UUID_16BIT_SIZE,                            /* bValueLen     */
        NULL,                                       /* pValueContext */
        GATT_PERM_READ                              /* wPermissions  */
    },

    /* <<Characteristic>>, ..  Index 1 */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            (GATT_CHAR_PROP_READ |                  /* characteristic properties */
            GATT_CHAR_PROP_WRITE |                  /* write property is optional */
            GATT_CHAR_PROP_NOTIFY)
            /* characteristic UUID not needed here, is UUID of next attrib. */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },
    /* Current Time value, ..  Index 2 */
    {
        ATTRIB_FLAG_VALUE_APPL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHAR_CTS_CURRENT_TIME),
            HI_WORD(GATT_UUID_CHAR_CTS_CURRENT_TIME)
        },
        0,                                          /* bValueLen */
        NULL,
        (GATT_PERM_READ | GATT_PERM_WRITE)          /* wPermissions */
    },
    /* client characteristic configuration, ..  Index 3 */
    {
        ATTRIB_FLAG_VALUE_INCL | ATTRIB_FLAG_VALUE_APPL,  /* wFlags */
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

    /* <<Characteristic>>,  ..  Index 4 */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            (GATT_CHAR_PROP_READ |                  /* characteristic properties */
            GATT_CHAR_PROP_WRITE)                   /* write property is optional */
            /* characteristic UUID not needed here, is UUID of next attrib. */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },
    /* Local Time Information value, ..  Index 5 */
    {
        ATTRIB_FLAG_VALUE_APPL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHAR_CTS_LOCAL_TIME_INFO),
            HI_WORD(GATT_UUID_CHAR_CTS_LOCAL_TIME_INFO)
        },
        0,                                          /* bValueLen */
        NULL,
        (GATT_PERM_READ | GATT_PERM_WRITE)          /* wPermissions */
    },

    /* <<Characteristic>>,  ..  Index 6 */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_READ                       /* characteristic properties */
            /* characteristic UUID not needed here, is UUID of next attrib. */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },
    /* Reference Time Information value, ..  Index 7 */
    {
        ATTRIB_FLAG_VALUE_APPL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHAR_CTS_REF_TIME_INFO),
            HI_WORD(GATT_UUID_CHAR_CTS_REF_TIME_INFO)
        },
        0,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    }
};
/**< @brief  CTS service size definition.  */
const int32_t iCtsAttrTblSize = sizeof(CtsAttrTbl);

/**
  * @brief  Set service related data from application.
  *
  * @param[in] param_type            parameter type to set: @ref CTS_Application_Parameters
  * @param[in] length                value length to be set.
  * @param[in] value_ptr             value to set.
  * @return parameter set result.
  * @retval 0 FALSE
  * @retval 1 TRUE
  */
bool CTS_SetParameter( uint8_t param_type, uint8_t length, void *value_ptr )
{
    bool ret = TRUE;

    switch (param_type)
    {
    default:
        ret = FALSE;
        break;
    case CTS_PARAM_CUR_TIME:
        CTS_CurrentTime = *(TCurrentTime *)value_ptr;
        break;
    case CTS_PARAM_LOCAL_TIME:
        CTS_LocalTimeInfo = *(TLocalTimeInfo *)value_ptr;
        break;
    case CTS_PARAM_REF_TIME:
        CTS_RefTimeInfo = *(TReferenceTimeInfo *)value_ptr;
        break;
    }

    if ( !ret )
        DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "CTS parameter set failed\n", 0 );

    return ret;
}

/**
  * @brief  Get service related data by application.
  *
  * @param[in] param_type            parameter type to set: @ref CTS_Application_Parameters
  * @param[out] value_ptr            value to set.
  * @return parameter get result.
  * @retval 0 FALSE
  * @retval 1 TRUE
  */
bool CTS_GetParameter( uint8_t param_type, void *value_ptr)
{
    bool ret = TRUE;

    switch (param_type)
    {
    default:
        ret = FALSE;
        break;
    case CTS_PARAM_CUR_TIME:
        *(TCurrentTime *)value_ptr = CTS_CurrentTime;
        break;
    case CTS_PARAM_LOCAL_TIME:
        *(TLocalTimeInfo *)value_ptr = CTS_LocalTimeInfo;
        break;
    case CTS_PARAM_REF_TIME:
        *(TReferenceTimeInfo *)value_ptr = CTS_RefTimeInfo;
        break;
    }

    if ( !ret )
        DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "CTS parameter get failed\n", 0 );

    return ret;
}

/**
 * @brief read characteristic data from service.
 *
 * @param ServiceId          ServiceID of characteristic data.
 * @param iAttribIndex          Attribute index of getting characteristic data.
 * @param iOffset                Used for Blob Read.
 * @param piLength            length of getting characteristic data.
 * @param ppValue            data got from service.
 * @return Profile procedure result
*/
TProfileResult  CTS_AttrReadCb( uint8_t ServiceId, uint16_t iAttribIndex, uint16_t iOffset, uint16_t * piLength, uint8_t **ppValue )
{
    TProfileResult  wCause  = ProfileResult_Success;

    /* TODO: we should check offset value here, if invalid, return ATT_INVALID_PARAMETER or ATT_NOT_LONG. */
    TCTS_CALLBACK_DATA callback_data;
    callback_data.msg_type = SERVICE_CALLBACK_TYPE_READ_CHAR_VALUE;
    
    switch ( iAttribIndex )
    {
    default:
        DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "<-- CTS_AttrRead, Attr not found, index=%d", 1, iAttribIndex);
        wCause  = ProfileResult_AttrNotFound;
        break;
    case GATT_SVC_CTS_CUR_TIME_INDEX:
        {
            callback_data.msg_data.read_value_index = CTS_READ_CUR_TIME;            
            pfnCtsCB(ServiceId, (void*)&callback_data);
            *ppValue = (uint8_t *)&CTS_CurrentTime;
            *piLength = sizeof(CTS_CurrentTime);
        }
        break;
    case GATT_SVC_CTS_LOCAL_TIME_INDEX:
        {
            callback_data.msg_data.read_value_index = CTS_READ_LOCAL_TIME;            
            pfnCtsCB(ServiceId, (void*)&callback_data);            
            *ppValue = (uint8_t *)&CTS_LocalTimeInfo;
            *piLength = sizeof(CTS_LocalTimeInfo);
        }
        break;
    case GATT_SVC_CTS_REF_TIME_INDEX:
        {
            callback_data.msg_data.read_value_index = CTS_READ_REGERENCE_TIME;            
            pfnCtsCB(ServiceId, (void*)&callback_data);                    
            *ppValue = (uint8_t *)&CTS_RefTimeInfo;
            *piLength = sizeof(CTS_RefTimeInfo);
        }
        break;
    }

    return ( wCause );
}

/**
 * @brief write characteristic data from service.
 *
 * @param ServiceID          ServiceID to be written.
 * @param iAttribIndex          Attribute index of characteristic.
 * @param wLength            length of value to be written.
 * @param pValue            value to be written.
 * @param pWriteIndPostProc pointer of a function to handle control point write.
 * @return Profile procedure result
*/
TProfileResult CTS_AttrWriteCb(uint8_t ServiceId, uint16_t iAttribIndex, uint16_t wLength, uint8_t * pValue, TGATTDWriteIndPostProc * pWriteIndPostProc)
{
    TCTS_CALLBACK_DATA callback_data;
    callback_data.msg_type = SERVICE_CALLBACK_TYPE_WRITE_CHAR_VALUE;

    TProfileResult  wCause  = ProfileResult_Success;
    if (GATT_SVC_CTS_CUR_TIME_INDEX == iAttribIndex)
    {
        /* make sure written value size is valid. */
        if ( (wLength > sizeof(CTS_CurrentTime)) || (pValue == NULL))
        {
            wCause  = ProfileResult_InvalidValueSize;
        }
        else
        {
            if ( 1 )
            {
                /* TODO: add flag of ignored fields. Define some related macros. */
                /* when no field in written values ignored, we will write whole values. */
                memcpy(&CTS_CurrentTime, pValue, wLength);
            }
            else
            {
                /* if some fields ignored, partially written should be operated, and return FIELD_IGNORED error. */
                wCause = ProfileResult_AppErr;
            }
            /* Notify Application. */
            memcpy(&callback_data.msg_data.write.cur_time, pValue, wLength);
            if (pfnCtsCB)
            {
                pfnCtsCB(ServiceId, (void*)&callback_data);
            }
        }
    }
    else if (GATT_SVC_CTS_LOCAL_TIME_INDEX == iAttribIndex)
    {
        /* make sure written value size is valid. */
        if ( (wLength > sizeof(CTS_LocalTimeInfo)) || (pValue == NULL))
        {
            wCause  = ProfileResult_InvalidValueSize;
        }
        else
        {
            if ( 1 )
            {
                /* when no field in written values ignored, we will write whole values. */
                memcpy(&CTS_LocalTimeInfo, pValue, wLength);
            }
            else
            {
                /* ignore some fields, partially written should be operated. */
                wCause = ProfileResult_AppErr;
            }
            /* Notify Application. */            
            memcpy(&callback_data.msg_data.write.local_time, pValue, wLength);            
            if (pfnCtsCB)
            {
                pfnCtsCB(ServiceId, (void*)&callback_data);
            }
        }
    }
    else
    {
        DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "--> CTS_AttrWrite Attr not found! iAttribIndex = 0x%x wLength=%d",
                   2, iAttribIndex, wLength);
        wCause = ProfileResult_AttrNotFound;
    }

    return wCause;
}

/**
  * @brief send notification of Current Time.
  *
  * @param[in] ServiceId         service ID of service.
  * @return notification action result
  * @retval 1 TRUE
  * @retval 0 FALSE
  */
bool CTS_CurTimeNotify( uint8_t ServiceId )
{
    uint16_t attrib_index = GATT_SVC_CTS_CUR_TIME_INDEX;
    uint8_t *pData = (uint8_t *)&CTS_CurrentTime;
    uint16_t dataLen = sizeof(CTS_CurrentTime);

    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "<-- CTS Current Time Notification", 0 );
    return ProfileAPI_SendData(ServiceId, attrib_index, pData, dataLen);
}

/**
 * @brief update CCCD bits from stack.
 *
 * @param ServiceId          Service ID.
 * @param Index          Attribute index of characteristic data.
 * @param wCCCBits         CCCD bits from stack.
 * @return None
*/
void CTS_CccdUpdateCb(uint8_t serviceId, uint16_t Index, uint16_t wCCCBits)
{
    TCTS_CALLBACK_DATA callback_data;
    bool bHandle = FALSE;
    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "CTS_CccdUpdate Index = %d wCCCDBits %x", 2, Index, wCCCBits);
    callback_data.msg_type = SERVICE_CALLBACK_TYPE_INDIFICATION_NOTIFICATION;
    switch (Index)
    {
    case GATT_SVC_CTS_CUR_TIME_CCCD_INDEX:
        if (wCCCBits & GATT_CCCD_NOTIFICATION_BIT)
        {
            // Enable Notification.
            callback_data.msg_data.notification_indification_index = CTS_NOTIFY_INDICATE_CUR_TIME_ENABLE;
        }
        else
        {
            // Disable Notification.
            callback_data.msg_data.notification_indification_index = CTS_NOTIFY_INDICATE_CUR_TIME_DISABLE;
        }
        bHandle =  TRUE;
        break;
    default:
        break;
    }
    /* Notify Application. */
    if (pfnCtsCB && (bHandle == TRUE))
    {
        pfnCtsCB(serviceId, (void*)&callback_data);
    }
}

/**
 * @brief CTS Service Callbacks.
*/
CONST gattServiceCBs_t ctsCBs =
{
    CTS_AttrReadCb,  // Read callback function pointer
    CTS_AttrWriteCb, // Write callback function pointer
    CTS_CccdUpdateCb  // CCCD update callback function pointer
};

/**
  * @brief add CTS service to application.
  *
  * @param[in] pFunc          pointer of app callback function called by profile.
  * @return service ID auto generated by profile layer.
  * @retval ServiceId
  */
uint8_t CTS_AddService(void* pFunc)
{
    uint8_t ServiceId;

    /* register CTS service to profile layer. */
    if (FALSE == ProfileAPI_AddService(&ServiceId,
                                       (uint8_t*)CtsAttrTbl,
                                       iCtsAttrTblSize,
                                       ctsCBs))
    {
        DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "CTS_AddService: ServiceId %d", 1, ServiceId);
        ServiceId = 0xff;
        return ServiceId;
    }

    /* register callback for profile to inform application that some events happened. */
    pfnCtsCB = (pfnAPPHandleInfoCB_t)pFunc;
    return ServiceId;
}

