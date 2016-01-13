enum { __FILE_NUM__ = 0 };

/**
*********************************************************************************************************
*               Copyright(c) 2014, Realtek Semiconductor Corporation. All rights reserved.
**********************************************************************************************************
* @file     ndc_service.c
* @brief    NDC profile interfaces called in profile.c.
* @details  Interfaces to get and put ndc characteristics value and ndc control point procedure.
* @author   ethan_su
* @date     2014-10-10
* @version  v0.1
*********************************************************************************************************
*/
#include "trace.h"
#include "FreeRTOS.h"
#include "gatt.h"
#include "profile_api.h"
#include "ndc_service.h"

/********************************************************************************************************
* local static variables defined here, only used in this source file.
********************************************************************************************************/
/**<  NDC time with DST data. */
static TTimeWithDST NDC_TimeWithDST = 
                    { 
                        { /* DateTime  */
                            LO_WORD(2015), HI_WORD(2015),  /* YY:YY */
							06,        /* MM */
							01,        /* DD */
							19,        /* HH */
							13,        /* MM */
							25         /* SS */
						},
						NDC_DST_OFFSET_STANDARD
					};
/**<  Function pointer used to send event to application from NDC profile. Initiated in NDC_AddService. */
static pfnAPPHandleInfoCB_t pfnNdcCB = NULL;

/**< @brief  profile/service definition.  */
const TAttribAppl NdcAttrTbl[] =
{
    /*----------------- Next DST Change Service -------------------*/
    /* <<Primary Service>>, .. index 0 */
    {
        (ATTRIB_FLAG_VALUE_INCL | ATTRIB_FLAG_LE),  /* wFlags     */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_PRIMARY_SERVICE),
            HI_WORD(GATT_UUID_PRIMARY_SERVICE),
            LO_WORD(GATT_UUID_NEXT_DST_CHANGE),       /* service UUID */
            HI_WORD(GATT_UUID_NEXT_DST_CHANGE)
        },
        UUID_16BIT_SIZE,                            /* bValueLen     */
        NULL,                                       /* pValueContext */
        GATT_PERM_READ                              /* wPermissions  */
    },

    /* <<Characteristic>>, ..  index 1 */
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
    /* Time with DST value  index 2 */
    {
        ATTRIB_FLAG_VALUE_APPL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHAR_NDCS_TIME_WITH_DST),
            HI_WORD(GATT_UUID_CHAR_NDCS_TIME_WITH_DST)
        },
        0,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    }
};
/**< @brief  NDC service size definition.  */
const int iNdcAttrTblSize = sizeof(NdcAttrTbl);

/**
  * @brief  Set service related data from application.
  *
  * @param[in] param_type            parameter type to set: @ref NDC_Application_Parameters
  * @param[in] len                   value length to be set.
  * @param[in] value_ptr             value to set.
  * @return parameter set result.
  * @retval 0 FALSE
  * @retval 1 TRUE
  */
bool NDC_SetParameter( uint8_t param_type, uint8_t len, void *value_ptr)
{
    bool ret = TRUE;

    switch (param_type)
    {
    default:
        ret = FALSE;
        break;
    case NDC_PARAM_TIME_WITH_DST:
        NDC_TimeWithDST = *(TTimeWithDST *)value_ptr;
        break;
    }

    if ( !ret )
        DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "NDC parameter set failed\n", 0 );

    return ret;
}

/**
  * @brief  Get service related data by application.
  *
  * @param[in] param_type            parameter type to set: @ref NDC_Application_Parameters
  * @param[in] value_ptr             value to set.
  * @return parameter get result.
  * @retval 0 FALSE
  * @retval 1 TRUE
  */
bool NDC_GetParameter( uint8_t param_type, void *value_ptr)
{
    bool ret = TRUE;

    switch (param_type)
    {
    default:
        ret = FALSE;
        break;
    case NDC_PARAM_TIME_WITH_DST:
        *(TTimeWithDST *)value_ptr = NDC_TimeWithDST;
        break;
    }

    if ( !ret )
        DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "NDC parameter get failed\n", 0 );

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
TProfileResult  NdcAttrReadCb( uint8_t ServiceId, uint16_t iAttribIndex, uint16_t iOffset, uint16_t * piLength, uint8_t **ppValue )
{
    TProfileResult  wCause  = ProfileResult_Success;

    /* TODO: we should check offset value here, if invalid, return ATT_INVALID_PARAMETER or ATT_NOT_LONG. */

    switch ( iAttribIndex )
    {
    default:
        DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "<-- NDC_AttrRead, Attr not found, index=%d", 1, iAttribIndex);
        wCause  = ProfileResult_AttrNotFound;
        break;
    case GATT_SVC_NDC_TIME_WITH_DST_INDEX:
        {
            TNDC_CALLBACK_DATA callback_data;
            callback_data.msg_type = SERVICE_CALLBACK_TYPE_READ_CHAR_VALUE;
            callback_data.msg_data.read_value_index = NDC_READ_TIME_WITH_DST;
            pfnNdcCB(ServiceId, (void*)&callback_data);
            *ppValue = (uint8_t *)&NDC_TimeWithDST;
            *piLength = sizeof(NDC_TimeWithDST);
        }
        break;
    }

    return ( wCause );
}

/**
 * @brief NDC Service Callbacks.
*/
CONST gattServiceCBs_t ndcCBs =
{
    NdcAttrReadCb,  // Read callback function pointer
    NULL, // Write callback function pointer
    NULL    // CCCD update callback function pointer
};

/**
  * @brief add NDC service to application.
  *
  * @param[in] pFunc          pointer of app callback function called by profile.
  * @return service ID auto generated by profile layer.
  * @retval ServiceId
  */
uint8_t NDC_AddService(void* pFunc)
{
    uint8_t ServiceId;

    /* register NDC service to profile layer. */
    if (FALSE == ProfileAPI_AddService(&ServiceId,
                                       (uint8_t*)NdcAttrTbl,
                                       iNdcAttrTblSize,
                                       ndcCBs))
    {
        DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "NDC_AddService: ServiceId %d", 1, ServiceId);
        ServiceId = 0xff;
        return ServiceId;
    }

    /* register callback for profile to inform application that some events happened. */
    pfnNdcCB = (pfnAPPHandleInfoCB_t)pFunc;
    return ServiceId;
}

