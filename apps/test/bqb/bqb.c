enum { __FILE_NUM__= 0 };
/**
*********************************************************************************************************
*               Copyright(c) 2014, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file      bqb.c
* @brief     bqb releated functions implementation.
* @details   none.
* @author    Tifnan
* @date      2014-08-29
* @version   v0.1
* *********************************************************************************************************
*/
#include "stdint.h"
#include "stddef.h"
#include "gatt.h"
#include "bqb.h"
#include "bqb_database.h"
#include "bqb_demo.h"

/**
 * @brief get service index according to service id.
 *
 * @param  pGATTDemo -- pointer to global struct GATTDemo.
 * @param  ServiceID -- the id of the service.
 * @return the service index.
 * @retval 0.1.2... the service index
 *        -1 not found
*/
int BQB_GetServiceIndexByServiceID(PGATTDemo pGATTDemo, BQB_GATT_ServiceId ServiceID)
{
    uint8_t i;
    for (i = 0; i < BQB_ServiceNum; i++)
    {
        if (ServiceID == pGATTDemo->Service[i].ServiceID)
        {
            return i;
        }
    }
    return -1;
}

/**
 * @brief get service index according to service id.
 *
 * @param  pGATTDemo -- pointer to global struct GATTDemo.
 * @param  idx -- link index.
 * @param  wHandle -- the handle of the attribute.
 * @return the 16-bit uuid of the attribute.
 * @retval 0-- not found
 *         !=0 -- the uuid of the attribute.
*/
uint16_t BQB_GetUuidByHandle(PGATTDemo pGATTDemo, uint16_t idx, uint16_t wHandle)
{
    uint8_t  i;
    uint16_t uuid16 = 0;

    for (i = 0; i < pGATTDemo->linkTable[idx].iHandleCnt; i++)
    {
        if (pGATTDemo->linkTable[idx].HandleUUID[i].wHandle == wHandle)
        {
            uuid16 = pGATTDemo->linkTable[idx].HandleUUID[i].wUUID;
        }
    }
    if (uuid16 == 0)
    {
        BQB_CmdPrint(pGATTDemo, " : handle not found in local table !\r\n%s",
                     pGATTDemo->CmdIF.cPrompt );
    }
    return (uuid16);
}

/* globals for attribute value */
uint8_t ChValInValLen[] = {0x01, 0x02};
uint8_t ChValDefault[] = {0x01};

/**
 * @brief service/attribute specific write data setup .
 *
 * @param  pGATTDemo -- pointer to global struct GATTDemo.
 * @param  pParseResult -- the parse result of com command.
 * @param  pwLength -- out paremeter, the length of the write data.
 * @param  ppValue -- out paremeter, pointer to the address of the address of write data.
 * @return the service index.
 * @retval 0.1.2... the service index
 *        -1 not found
*/
uint16_t  BQB_AttribGetWriteData(PGATTDemo pGATTDemo,
                                 TGATTDemoParseResult *pParseResult,
                                 uint16_t * pwLength, uint8_t **ppValue )
{
    //uint16_t     uuid16;
    //uint16_t     link_idx = 0;
    //uint16_t     idx = pParseResult->dwParameter[0];        /* link index */
    uint16_t     wHandle = pParseResult->dwParameter[1];
    uint16_t     wParam  = pParseResult->dwParameter[2];
    //uint16_t    uuid = 0;

    BQB_CmdPrint(pGATTDemo, "write to handle=0x%x", wHandle);

    *pwLength = 0;
    *ppValue  = NULL;

    //uuid = BQB_GetUuidByHandle(pGATTDemo, idx, wHandle);    /* to be check */

    if (SMALL_DATABASE == BQB_GATT_Database_Index)
    {
        return GATT_ERR_NOT_ALLOWED;  /* can not write attribute of small database */
    }
    else if (LARGE_DATABASE1 == BQB_GATT_Database_Index)
    {
        /* save write data, bqb data base is much large!! we must choose some attribute to write, others ignored, to be continue */
        switch (wHandle)
        {
        /* Write Characteristic Value - server timeout - Client TP/GAW/CL/BV-02-C, to be added */


        case 0x00ED: /* Write Characteristic Value - Invalid handle - Client TP/GAW/CL/BI-02-C */
            *ppValue = &ChValDefault[0];
            *pwLength = 1;
            break;

        case 0x0023:    /*  Characteristic Value - Write not permitted - Client TP/GAW/CL/BI-03-C */
            *ppValue = &ChValDefault[0];
            *pwLength = 1;
            break;

        case 0x0029:    /* Write Characteristic Value - Invalid Attribute Value Length - Client TP/GAW/CL/BI-33-C */
            *ppValue = &ChValInValLen[0];
            *pwLength = 2;
            break;

        case 0x0032:   /*Write Characteristic Value - by client TP/GAW/CL/BV-03-C */
            *ppValue = &ChValInValLen[0];
            *pwLength = 2;
            break;

        case 0x0052: /* Write without response - by client TP/GAW/CL/BV-01-C */
            *ppValue = &ChValDefault[0];
            *pwLength = 1;
            break;

        case 0x0082: /* Write Characteristic Value - Insufficient Authentication TP/GAW/CL/BI-05-C */
            *ppValue = &ChValDefault[0];
            *pwLength = 1;
            break;

        case 0x0062: /* Write Characteristic Value - Insufficient Authorization TP/GAW/CL/BI-04-C */
            *ppValue = &ChValDefault[0];
            *pwLength = 1;
            break;

        /* Write Characteristic Value - Insufficient Encryption Key Size TP/GAW/CL/BI-06-C, to be added */

        case 0x00EE: /*Write Characteristic Descriptor - Invalid handle  TP/GAW/CL/BI-20-C */
            *ppValue = &ChValDefault[0];
            *pwLength = 1;
            break;

        case 0x0073: /* Write Characteristic Descriptor - Write not permitted TP/GAW/CL/BI-21-C */
            *ppValue = &ChValInValLen[0];
            *pwLength = 2;
            break;

        case 0x0065: /* Write Characteristic Descriptors - by client TP/GAW/CL/BV-08-C */
            *ppValue = &ChValInValLen[0];
            *pwLength = 2;
            break;

        case 0x0074: /* Write Characteristic Descriptors - Invalid Attribute Value Length TP/GAW/CL/BI-35-C */
            *ppValue = &ChValInValLen[0];
            *pwLength = 2;
            break;

        case 0x0085: /* Write Characteristic Descriptor - Insufficient Authentication TP/GAW/CL/BI-23-C */
            *ppValue = &ChValDefault[0];
            *pwLength = 1;
            break;

        case 0x0083: /* Write Characteristic Descriptor - Insufficient Authorization TP/GAW/CL/BI-22-C */
            *ppValue = &ChValDefault[0];
            *pwLength = 1;
            break;

        /* Write Characteristic Descriptor - Insufficient Encryption Key Size TP/GAW/CL/BI-24-C, to be added */

        /* write long decriptor & long characteristic, reliable write /wait for stack */

        case 0x0053:    /* in large database1 */
            LE_WORD2EXTRN( pGATTDemo->bCCCBits, wParam );
            /* PUT external 2 uint8_t from local uint16_t, Little-Endian Format */
            *ppValue  = pGATTDemo->bCCCBits;
            *pwLength = sizeof(pGATTDemo->bCCCBits);
            break;
        }
    }
    //BQB_CmdPrint( pGATTDemo, "\r\n%s", pGATTDemo->CmdIF.cPrompt );
    return (GATT_SUCCESS);
}


/**
 * @brief save characteristic/descriptor handle- uuid pair.
 *
 * @param  pGATTDemo -- pointer to global struct GATTDemo.
 * @param  idx -- link index.
 * @param  wHandle -- handle of a characteristic or descriptor.
 * @param  wUUID16 -- the uuid of this characteristic or descriptor.
 * @return the service index.
 * @retval 0.1.2... the service index
 *        -1 not found
*/
void BQB_HandleUUIDSave(PGATTDemo pGATTDemo, uint16_t idx, uint16_t wHandle, uint16_t wUUID16)
{
    uint8_t i;

    for ( i = 0; i < pGATTDemo->linkTable[idx].iHandleCnt; i++ )
    {
        if ( pGATTDemo->linkTable[idx].HandleUUID[i].wHandle == wHandle )
        {
            return;
        }
    }

    if ( pGATTDemo->linkTable[idx].iHandleCnt < BQB_MAX_HANDLE_UUID )
    {
        pGATTDemo->linkTable[idx].HandleUUID[pGATTDemo->linkTable[idx].iHandleCnt].wHandle = wHandle;
        pGATTDemo->linkTable[idx].HandleUUID[pGATTDemo->linkTable[idx].iHandleCnt].wUUID   = wUUID16;
        pGATTDemo->linkTable[idx].iHandleCnt++;
    }
    else
    {
        BQB_CmdPrint( pGATTDemo,
                      "!! GATTDEMO_MAX_HANDLE_UUID=%d is too small !!\r\n%s",
                      BQB_MAX_HANDLE_UUID, pGATTDemo->CmdIF.cPrompt );
    }
}

/**
 * @brief init gatt services, call it in gattdHandle_RegisterRsp() function.
 *
 * @param  pGATTDemo -- pointer to global struct GATTDemo.
 * @return none
 * @retval void
*/
void  BQB_ServiceInitData(PGATTDemo pGATTDemo)
{
    /* do nothing */
}

/**
 * @brief handle services update, call it in gattdHandle_GATTAttributeUpdateRsp function.
 * @param  pGATTDemo -- pointer to global struct GATTDemo.
 * @param  wCause -- update result.
 * @param  wAttribIndex -- the index of service which is updated.
 * @return none
 * @retval void
*/
void  BQB_ServiceUpdateCallback(PGATTDemo pGATTDemo,
                                uint16_t wCause, uint16_t wAttribIndex)
{
    /* do nothing */
}

/**
 * @brief put/write local attribute value.
 *
 * @param  pGATTDemo -- pointer to global struct GATTDemo.
 * @param  pService -- pointer to the service which the attribute belongs.
 * @param  iAttribIndex -- the index of the attribute in this service.
 * @param  wLength -- the length of the attribute value to be written.
 * @param  pValue -- pointer to the address of the pointer to the write data.
 * @param  pWriteIndPostProc  -- not used
 * @return write result
*/
uint16_t BQB_AttribPut(PGATTDemo pGATTDemo, PGATTDService pService, uint16_t iAttribIndex,
                       uint16_t wLength, uint8_t * pValue, TBQBGATTDWriteIndPostProc * pWriteIndPostProc)
{
    return GATT_SUCCESS;
}

/**
 * @brief put/write local attribute value.
 *
 * @param  pGATTDemo -- pointer to global struct GATTDemo.
 * @param  pService -- pointer to the service which the attribute belongs.
 * @param  iAttribIndex -- the index of the attribute in this service.
 * @param  wLength -- the length of the attribute value to be written.
 * @param  pValue -- pointer to the address of the pointer to the write data.
 * @return write result
*/
uint16_t BQB_AttribGet(PGATTDemo pGATTDemo, PGATTDService pService, uint16_t iAttribIndex,
                       int iOffset, uint16_t* pwLength, uint8_t** ppValue)
{
    uint8_t * pValue = NULL;
    *pwLength = 0;
    uint8_t i = 0;
	uint8_t service_idx = 0;
	uint16_t service_off = 0;
	PAttribAppl p_attr = NULL;

#if 0
    if (BQB_NOTIFICATION_VALUE_INDEX == iAttribIndex)
    {
        pValue = (uint8_t*)&ChaValV6;
        *pwLength = sizeof(ChaValV6);
    }
    else if (28 == iAttribIndex) /* handle = 0x00DC */
    {
        pValue = (uint8_t*)&ChaValV2_9;
        *pwLength = sizeof(ChaValV2_9) - 1;
    }
#endif

	for(service_idx = 0; service_idx < pGATTDemo->iServiceCount; service_idx++)
	{
		if(pGATTDemo->Service[service_idx].ServiceID == pService->ServiceID)
		{
			break;
		}
	}

	/* find valid sercice id */
	for(i = 0; i < service_idx; i++ )
	{
		service_off+= Service_Info[i].num_attr;
	}

	p_attr = (TAttribAppl*)(&LargeDatabase1[0] + service_off + iAttribIndex);
	
	pValue = (uint8_t*)p_attr->bTypeValue[2];
	*pwLength = p_attr->wValueLen;
	
    if(NULL != pValue)
    {
    	BQB_CmdPrint( pGATTDemo,  "BQB: gattAttribGet: pValue:0x%x,length:0x%x\n", pValue, *pwLength);
    }
    else
    {
    	BQB_CmdPrint( pGATTDemo,  "BQB: gattAttribGet:pvalue=NULL, len = 0x%x\n", *pwLength);
    }
    
    return GATT_SUCCESS;
}
