enum { __FILE_NUM__ = 0 };

/**
*********************************************************************************************************
*               Copyright(c) 2014, Realtek Semiconductor Corporation. All rights reserved.
**********************************************************************************************************
* @file     rtu_service.c
* @brief    RTU profile interfaces called in profile.c.
* @details  Interfaces to get and put rtu characteristics value and rtu control point procedure.
* @author   ethan_su
* @date     2014-10-10
* @version  v0.1
*********************************************************************************************************
*/
#include "trace.h"
#include "FreeRTOS.h"
#include "gatt.h"
#include "profile_api.h"
#include "rtu_service.h"

/********************************************************************************************************
* local static variables defined here, only used in this source file.
********************************************************************************************************/
/**<  RTU time update control point data. */
static TTimeUpdateCtlPnt  RTU_TimeUpdateCtlPnt;
/**<  RTU time update state. */
static TTimeUpdateState RTU_TimeUpdateState;
/**<  Function pointer used to send event to application from RTU profile. Initiated in RTU_AddService. */
static pfnAPPHandleInfoCB_t pfnRtuCB = NULL;

/**< @brief  profile/service definition.  */
const TAttribAppl RtuAttrTbl[] =
{
    /*-------------- Reference Time Update Service ----------------*/
    /* <<Primary Service>>, .. index 0 */
    {
        (ATTRIB_FLAG_VALUE_INCL | ATTRIB_FLAG_LE),  /* wFlags     */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_PRIMARY_SERVICE),
            HI_WORD(GATT_UUID_PRIMARY_SERVICE),
            LO_WORD(GATT_UUID_REFERENCE_TIME_UPDATE), /* service UUID */
            HI_WORD(GATT_UUID_REFERENCE_TIME_UPDATE)
        },
        UUID_16BIT_SIZE,                            /* bValueLen     */
        NULL,                                       /* pValueContext */
        GATT_PERM_READ                              /* wPermissions  */
    },

    /* <<Characteristic>>, .. index 1 */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_WRITE_NO_RSP               /* characteristic properties */
            /* characteristic UUID not needed here, is UUID of next attrib. */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },
    /* Time Update Control Point value .. index 2 */
    {
        ATTRIB_FLAG_VALUE_APPL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHAR_RTUS_CONTROL_POINT),
            HI_WORD(GATT_UUID_CHAR_RTUS_CONTROL_POINT)
        },
        0,                                          /* bValueLen */
        NULL,
        GATT_PERM_WRITE                             /* wPermissions */
    },

    /* <<Characteristic>>, .. index 3 */
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
    /* Time Update State value ..index 4 */
    {
        ATTRIB_FLAG_VALUE_APPL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHAR_RTUS_STATE),
            HI_WORD(GATT_UUID_CHAR_RTUS_STATE)
        },
        2,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    }
};
/**< @brief  RTU service size definition.  */
const int32_t iRtuAttrTblSize = sizeof(RtuAttrTbl);

/**
  * @brief  Set service related data from application.
  *
  * @param[in] param_type            parameter type to set: @ref RTU_Application_Parameters
  * @param[in] len                   value length to be set.
  * @param[in] value_ptr             value to set.
  * @return parameter set result.
  * @retval 0 FALSE
  * @retval 1 TRUE
  */
bool RTU_SetParameter( uint8_t param_type, uint8_t len, void *value_ptr)
{
    bool ret = TRUE;

    switch (param_type)
    {
    default:
        ret = FALSE;
        break;
    case RTU_PARAM_UPDATE_CMD:
        RTU_TimeUpdateCtlPnt = *(TTimeUpdateCtlPnt *)value_ptr;
        break;
    case RTU_PARAM_UPDATE_STATE:
        RTU_TimeUpdateState = *(TTimeUpdateState *)value_ptr;
        break;
    }

    if ( !ret )
        DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "RTU parameter set failed\n", 0 );

    return ret;
}

/**
  * @brief  Get service related data from application.
  *
  * @param[in] param_type            parameter type to get: @ref RTU_Application_Parameters
  * @param[out] value_ptr            value to get .
  * @return parameter set result.
  * @retval 0 FALSE
  * @retval 1 TRUE
  */
bool RTU_GetParameter( uint8_t param_type, void *value_ptr)
{
    bool ret = TRUE;

    switch (param_type)
    {
    default:
        ret = FALSE;
        break;
    case RTU_PARAM_UPDATE_CMD:
        *(TTimeUpdateCtlPnt *)value_ptr = RTU_TimeUpdateCtlPnt;
        break;
    case RTU_PARAM_UPDATE_STATE:
        *(TTimeUpdateState *)value_ptr = RTU_TimeUpdateState;
        break;
    }

    if ( !ret )
        DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "RTU parameter get failed\n", 0 );

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
TProfileResult  RtuAttrReadCb( uint8_t ServiceId, uint16_t iAttribIndex, uint16_t iOffset, uint16_t * piLength, uint8_t **ppValue )
{
    TProfileResult  wCause  = ProfileResult_Success;

    /* TODO: we should check offset value here, if invalid, return ATT_INVALID_PARAMETER or ATT_NOT_LONG. */

    switch ( iAttribIndex )
    {
    default:
        DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "<-- RTU_AttrRead, Attr not found, index=%d", 1, iAttribIndex);
        wCause  = ProfileResult_AttrNotFound;
        break;
    case GATT_SVC_RTU_STATE_INDEX:
        {
            TRTU_CALLBACK_DATA callback_data;
            callback_data.msg_type = SERVICE_CALLBACK_TYPE_READ_CHAR_VALUE;
            callback_data.msg_data.read_value_index = RTU_READ_TIME_UPDATE_STATE;
            pfnRtuCB(ServiceId, (void*)&callback_data);
            *ppValue = (uint8_t *)&RTU_TimeUpdateState;
            *piLength = sizeof(RTU_TimeUpdateState);
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
TProfileResult RtuAttrWriteCb(uint8_t ServiceId, uint16_t iAttribIndex, uint16_t wLength, uint8_t * pValue, TGATTDWriteIndPostProc * pWriteIndPostProc)
{
    TProfileResult  wCause  = ProfileResult_Success;

    TRTU_CALLBACK_DATA callback_data;    
    callback_data.msg_type = SERVICE_CALLBACK_TYPE_WRITE_CHAR_VALUE;

    if (GATT_SVC_RTU_CTL_PNT_INDEX == iAttribIndex)
    {
        /* make sure written value size is valid. */
        if ( (wLength > sizeof(RTU_TimeUpdateCtlPnt)) || (pValue == NULL))
        {
            wCause  = ProfileResult_InvalidValueSize;
        }
        else
        {
            RTU_TimeUpdateCtlPnt = *(TTimeUpdateCtlPnt *)pValue;
            DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "    RTU Control Point request: 0x%x", 1,
                       RTU_TimeUpdateCtlPnt.cmd);

            switch ( RTU_TimeUpdateCtlPnt.cmd )
            {
            default:
                break;
            case RTU_CP_CMD_GET_REF_UPDATE:
                /* Notify Application. */
                callback_data.msg_data.write.opcode = RTU_CP_CMD_GET_REF_UPDATE;

                if (pfnRtuCB)
                {
                    pfnRtuCB(ServiceId, (void*)&callback_data);
                }
                break;
            case RTU_CP_CMD_CANCEL_REF_UPDATE:
                /* Notify Application. */
                callback_data.msg_data.write.opcode = RTU_CP_CMD_CANCEL_REF_UPDATE;
                if (pfnRtuCB)
                {
                    pfnRtuCB(ServiceId, (void*)&callback_data);
                }
                break;
            }
        }
    }
    else
    {
        DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "--> RTU_AttrWrite Attr not found! iAttribIndex = 0x%x wLength=%d",
                   2, iAttribIndex, wLength);
        wCause = ProfileResult_AttrNotFound;
    }

    return wCause;
}

/**
 * @brief RTU Service Callbacks.
*/
CONST gattServiceCBs_t rtuCBs =
{
    RtuAttrReadCb,  // Read callback function pointer
    RtuAttrWriteCb, // Write callback function pointer
    NULL    // CCCD update callback function pointer
};

/**
  * @brief add RTU service to application.
  *
  * @param[in] pFunc          pointer of app callback function called by profile.
  * @return service ID auto generated by profile layer.
  * @retval ServiceId
  */
uint8_t RTU_AddService(void* pFunc)
{
    uint8_t ServiceId;

    /* Initiate RTU service related data, modify according to user's demand. */
    RTU_TimeUpdateCtlPnt.cmd = RTU_CP_CMD_RESERVED;
    RTU_TimeUpdateState.current_state = RTU_CURRENT_STATE_IDLE;
    RTU_TimeUpdateState.result = RTU_RESULT_SUCCESS;

    /* register RTU service to profile layer. */
    if (FALSE == ProfileAPI_AddService(&ServiceId,
                                       (uint8_t*)RtuAttrTbl,
                                       iRtuAttrTblSize,
                                       rtuCBs))
    {
        DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "RTU_AddService: ServiceId %d", 1, ServiceId);
        ServiceId = 0xff;
        return ServiceId;
    }

    /* register callback for profile to inform application that some events happened. */
    pfnRtuCB = (pfnAPPHandleInfoCB_t)pFunc;
    return ServiceId;
}

