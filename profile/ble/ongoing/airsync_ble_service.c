enum { __FILE_NUM__ = 0 };

/**
*********************************************************************************************************
*               Copyright(c) 2014, Realtek Semiconductor Corporation. All rights reserved.
**********************************************************************************************************
* @file     airsync_ble_service.c
* @brief    simple BLE profile source file.
* @details  Demonstration of how to implement a self-definition profile.
* @author
* @date     2015-5-10
* @version  v0.1
*********************************************************************************************************
*/
#include "trace.h"
#include <string.h>
#include "FreeRTOS.h"
#include "gatt.h"
#include "profile_api.h"
#include "airsync_ble_service.h"

/********************************************************************************************************
* local static variables defined here, only used in this source file.
********************************************************************************************************/
/**<  Value of simple read characteristic. */
static uint8_t airsyncCharReadValue = 0xf0;



/**<  Function pointer used to send event to application from simple profile. Initiated in AirSyncBleService_AddService. */
static pfnAPPHandleInfoCB_t pfnAirSyncBleServiceCB = NULL;

bool AirSyncBleService_V5Indicate( uint8_t ServiceId, uint8_t OpCode, uint8_t RspCode );

/**< @brief  profile/service definition.  */
const TAttribAppl airsync_ble_service_tbl[] =
{
    /* <<Primary Service>>, .. */
    {
        (ATTRIB_FLAG_VALUE_INCL | ATTRIB_FLAG_LE),  /* wFlags     */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_PRIMARY_SERVICE),
            HI_WORD(GATT_UUID_PRIMARY_SERVICE),
            LO_WORD(GATT_UUID_AIRSYNC_SERVICE),      /* service UUID */
            HI_WORD(GATT_UUID_AIRSYNC_SERVICE)
        },
        UUID_16BIT_SIZE,                            /* bValueLen     */
        NULL,                                       /* pValueContext */
        GATT_PERM_READ                              /* wPermissions  */
    },
	
	/* <<Characteristic>> demo for write */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            (GATT_CHAR_PROP_WRITE)  /* characteristic properties */
            /* characteristic UUID not needed here, is UUID of next attrib. */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ								/* wPermissions */
    },

    {
        ATTRIB_FLAG_VALUE_APPL,                     /* wFlags */
        {                                         	/* bTypeValue */
            LO_WORD(GATT_UUID_CHAR_SIMPLE_V2_WRITE),
            HI_WORD(GATT_UUID_CHAR_SIMPLE_V2_WRITE)
        },
        0,                                          /* bValueLen */
        NULL,
        GATT_PERM_WRITE                             /* wPermissions */
    },
	
	/* <<Characteristic>> demo for indicate */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                          /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            (GATT_CHAR_PROP_INDICATE)               /* characteristic properties */
            /* characteristic UUID not needed here, is UUID of next attrib. */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },

    {
        ATTRIB_FLAG_VALUE_APPL,                     /* wFlags */
        {                                         /* bTypeValue */
            LO_WORD(GATT_UUID_CHAR_SIMPLE_V4_INDICATE),
            HI_WORD(GATT_UUID_CHAR_SIMPLE_V4_INDICATE)
        },
        0,                                          /* bValueLen */
        NULL,
        GATT_PERM_NOTIF_IND                              /* wPermissions */
    },
    /* client characteristic configuration */
    {
        ATTRIB_FLAG_VALUE_INCL | ATTRIB_FLAG_CCCD_APPL,                 /* wFlags */
        {                                         /* bTypeValue */
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
	
    /* <<Characteristic>> demo for read */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_READ  /* characteristic properties */
            /* characteristic UUID not needed here, is UUID of next attrib. */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },

    {
        ATTRIB_FLAG_VALUE_APPL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHAR_SIMPLE_V1_READ),
            HI_WORD(GATT_UUID_CHAR_SIMPLE_V1_READ)
        },
        0,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ             					/* wPermissions */
    },
};

/**
  * @brief  Set service related data from application.
  *
  * @param[in] param_type            parameter type to set: @ref SIMP_Application_Parameters
  * @param[in] len                   value length to be set.
  * @param[in] value_ptr             value to set.
  * @return parameter set result.
  * @retval 0 FALSE
  * @retval 1 TRUE
  */
bool AirSyncBleService_SetParameter( uint8_t param_type, uint8_t len, void *value_ptr)
{
    bool ret = TRUE;

    switch (param_type)
    {
    default:
        ret = FALSE;
        break;
    case SIMPLE_BLE_SERVICE_PARAM_V1_READ_CHAR_VAL:
        airsyncCharReadValue = *(uint8_t *)value_ptr;
        break;  
    }

    if ( !ret )
        DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "AirSyncBleService_SetParameter failed\n", 0 );

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
TProfileResult  AirSyncBleServiceAttrReadCb( uint8_t ServiceId, uint16_t iAttribIndex, uint16_t iOffset, uint16_t * piLength, uint8_t **ppValue )
{
    TProfileResult  wCause  = ProfileResult_Success;

    switch ( iAttribIndex )
    {
    default:
        DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "<-- AirSyncBleServiceAttrReadCb, Attr not found, index=%d",
                   1, iAttribIndex);
        wCause  = ProfileResult_AttrNotFound;
        break;
    case AIRSYNC_BLE_SERVICE_CHAR_READ_INDEX:
        {
            TSIMP_CALLBACK_DATA callback_data;
            callback_data.msg_type = SERVICE_CALLBACK_TYPE_READ_CHAR_VALUE;
            callback_data.msg_data.read_value_index = AIRSYNC_READ_V1;
            pfnAirSyncBleServiceCB(ServiceId, (void*)&callback_data);            
            *ppValue    = &airsyncCharReadValue;
            *piLength = sizeof(airsyncCharReadValue);
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
 * @return Profile procedure result
*/
TProfileResult AirSyncBleServiceAttrWriteCb(uint8_t ServiceId, uint16_t iAttribIndex, uint16_t wLength, uint8_t * pValue, TGATTDWriteIndPostProc * pWriteIndPostProc)
{
    TSIMP_CALLBACK_DATA callback_data;
    TProfileResult  wCause = ProfileResult_Success;
    if (AIRSYNC_BLE_SERVICE_CHAR_WRITE_INDEX == iAttribIndex)
    {
        /* Make sure written value size is valid. */
        if(!pValue)
        {
            wCause  = ProfileResult_InvalidParameter;
        }
        else
        {
            /* Notify Application. */
            callback_data.msg_type = SERVICE_CALLBACK_TYPE_WRITE_CHAR_VALUE;
            callback_data.msg_data.pre_write.opcode = AIRSYNC_WRITE_V2;
            callback_data.msg_data.pre_write.pValue = pValue;   
			callback_data.msg_data.pre_write.Len = wLength;
            
            if (pfnAirSyncBleServiceCB)
            {
                pfnAirSyncBleServiceCB(ServiceId, (void*)&callback_data);
            }
        }
    }  
    else
    {
        DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "--> AirSyncBleServiceAttrWriteCb Error  iAttribIndex = 0x%x wLength=%d",
                   2,
                   iAttribIndex,
                   wLength);
        wCause = ProfileResult_AttrNotFound;
    }
    return wCause;
}

/**
  * @brief send indication of simple indicate characteristic value.
  *
  * @param[in] ServiceId         service ID of service.
  * @param[in] value             characteristic value to indicate
  * @return notification action result
  * @retval 1 TRUE
  * @retval 0 FALSE
  */
bool AirSyncBleService_SimpleV4Indicate( uint8_t ServiceId, uint8_t *pData, uint32_t data_len)
{
    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "<-- AirSyncBleService_SimpleV4Indicate", 0 );
    // send indication to client
    return ProfileAPI_SendData(ServiceId, AIRSYNC_BLE_SERVICE_CHAR_INDICATE_INDEX, pData, data_len);
}

/**
 * @brief update CCCD bits from stack.
 *
 * @param ServiceId          Service ID.
 * @param Index          Attribute index of characteristic data.
 * @param wCCCBits         CCCD bits from stack.
 * @return None
*/
void AirSyncBleServiceCccdUpdateCb(uint8_t serviceId, uint16_t Index, uint16_t wCCCBits)
{
    TSIMP_CALLBACK_DATA callback_data;
    bool bHandle = FALSE;
    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "AirSyncBleServiceCccdUpdateCb  Index = %d wCCCDBits %x", 2, Index, wCCCBits);
    switch (Index)
    {
  
    case AIRSYNC_BLE_SERVICE_CHAR_INDICATE_CCCD_INDEX:
        {
            if (wCCCBits & GATT_CCCD_INDICATION_BIT)
            {
                // Enable Indication
                callback_data.msg_type = SERVICE_CALLBACK_TYPE_INDIFICATION_NOTIFICATION;
                callback_data.msg_data.notification_indification_index = AIRSYNC_NOTIFY_INDICATE_V4_ENABLE;
            }
            else
            {
                // Disable Indication
                callback_data.msg_type = SERVICE_CALLBACK_TYPE_INDIFICATION_NOTIFICATION;
                callback_data.msg_data.notification_indification_index = AIRSYNC_NOTIFY_INDICATE_V4_DISABLE;
             }
            bHandle =  TRUE;
        }
        break;

    default:
        break;
    }
    /* Notify Application. */
    if (pfnAirSyncBleServiceCB && (bHandle == TRUE))
    {
        pfnAirSyncBleServiceCB(serviceId, (void*)&callback_data);
    }
}

/**
 * @brief Simple ble Service Callbacks.
*/
CONST gattServiceCBs_t airsyncBleServiceCBs =
{
    AirSyncBleServiceAttrReadCb,  // Read callback function pointer
    AirSyncBleServiceAttrWriteCb, // Write callback function pointer
    AirSyncBleServiceCccdUpdateCb,  // CCCD update callback function pointer
    NULL
};

/**
  * @brief add Simple BLE service to application.
  *
  * @param[in] pFunc          pointer of app callback function called by profile.
  * @return service ID auto generated by profile layer.
  * @retval ServiceId
  */
uint8_t AirSyncBleService_AddService(void* pFunc)
{
    uint8_t ServiceId;
    if (FALSE == ProfileAPI_AddService(&ServiceId,
                                       (uint8_t*)airsync_ble_service_tbl,
                                       sizeof(airsync_ble_service_tbl),
                                       airsyncBleServiceCBs))
    {
        DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "AirSyncBleService_AddService: ServiceId %d", 1, ServiceId);
        ServiceId = 0xff;
        return ServiceId;
    }
    pfnAirSyncBleServiceCB = (pfnAPPHandleInfoCB_t)pFunc;
    return ServiceId;
}

