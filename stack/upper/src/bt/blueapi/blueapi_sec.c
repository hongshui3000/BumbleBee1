/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       blueapi_sec.c
* @brief     LTP Application BlueSecure Interfave
* @details   
*
* @author   	gordon
* @date      	2015-07-08
* @version	v0.1
*/

#include <flags.h>
#include <rtl_types.h>
#include <os_message.h>
#include <os_pool.h>
#include <blueface.h>
#include <btglbdef.h>
#include <blueapi_types.h>
#include <sdp_code.h>
#include <sdplib.h>
#include <blueapi_def.h>
#include <bt_api.h>

#define BLUEAPI_SDP_ATTR_RANGE_DID  0x02000205
#define BLUEAPI_SDP_ATTR_RANGE_HDP1 0x03010302
#define BLUEAPI_SDP_ATTR_RANGE_HDP2 0x00000201
#define BLUEAPI_SDP_ATTR_RANGE_SPP  0x0000FFFF
#define BLUEAPI_SDP_ATTR_RANGE_GATT 0x0000FFFF

/* File Id for COM_InternalEventInfo */
#define BLUE_API_SOURCE_FILE_ID     0x07

/**
* @brief  blueapi sdp decode attribute
* 
* @param  pBlueAPIdata
* @param  serviceHandle
* @param  pAttrBuf
* @param  attrLen
*
* @return  
*
*/
void blueAPI_SDPDecodeDIPAttr(PBlueAPI_Data pBlueAPIdata, uint32_t serviceHandle, uint8_t * pAttrBuf, uint16_t attrLen)
{
	PBlueAPI_SDPParserData  pSDPData      = &pBlueAPIdata->SM_SDP_Data;
	uint8_t *                  pAttrBufEnd   = pAttrBuf + attrLen;
	uint8_t *                  pAttribute;
	BOOL                    primaryRecord;

	/* PrimaryRecord */
	pAttribute    = sdpAccessAttributeElement(pAttrBuf, attrLen,
	                                         (uint32_t)SDP_ATTR_DIP_PRIMARY_RECORD);
	primaryRecord = (BOOL)sdpGetDValue(pAttribute, pAttrBufEnd);

	if (primaryRecord)
	{
		pSDPData->DID_PrimaryFound         = TRUE;
		/* save handle for APP conf */
		pSDPData->DID_ServiceHandle        = serviceHandle;
		/* VendorID */
		pAttribute                         = sdpAccessAttributeElement(pAttrBuf, attrLen,
		                                                            (uint32_t)SDP_ATTR_DIP_VENDOR_ID);
		pSDPData->DID_VendorID             = (uint16_t)sdpGetDValue(pAttribute, pAttrBufEnd);
		/* ProductID */
		pAttribute                         = sdpAccessAttributeElement(pAttrBuf, attrLen,
		                                                            (uint32_t)SDP_ATTR_DIP_PRODUCT_ID);
		pSDPData->DID_ProductID            = (uint16_t)sdpGetDValue(pAttribute, pAttrBufEnd);
		/* Version */
		pAttribute                         = sdpAccessAttributeElement(pAttrBuf, attrLen,
		                                                            (uint32_t)SDP_ATTR_DIP_PRODUCT_VERSION);
		pSDPData->DID_Version              = (uint16_t)sdpGetDValue(pAttribute, pAttrBufEnd);
		/* VendorIDSource */
		pAttribute                         = sdpAccessAttributeElement(pAttrBuf, attrLen,
		                                                            (uint32_t)SDP_ATTR_DIP_VENDOR_ID_SOURCE);
		pSDPData->DID_VendorID_Source      = (uint16_t)sdpGetDValue(pAttribute, pAttrBufEnd);
	}
}

/**
* @brief  decode rfcomm attribute
* 
* @param  pBlueAPIdata
* @param  serviceHandle
* @param  pAttrBuf
* @param  attrLen
*
* @return  
*
*/
void blueAPI_SDPDecodeRFCOMMAttr(PBlueAPI_Data pBlueAPIdata, uint32_t serviceHandle, uint8_t * pAttrBuf, uint16_t attrLen)
{
	uint8_t *  pAttrBufEnd = pAttrBuf + attrLen;
	uint8_t *  pAttribute;
	uint8_t *  pAttributeParameter;
	uint8_t *  pElement;
	uint16_t    loop;
    uint16_t    serviceUUID = 0;
    uint8_t    serverChannel = 0;
    uint8_t    serviceName[BLUE_API_MDEP_NAME_LENGTH];
    uint16_t    supportedfeatures = 0;
    uint16_t    remoteversion = 0;


	pAttribute  = sdpAccessAttributeElement(pAttrBuf, attrLen,
	                                       (uint32_t)SDP_ATTR_SERVICECLASSIDLIST
	                                       );
	loop = 1;
	while ((pElement = sdpAccessElement(pAttribute, pAttrBufEnd, loop)) != NULL)
	{ 
		serviceUUID = (uint16_t)sdpGetDValue(pElement, pAttrBufEnd);
		if (serviceUUID != 0xFFFF)
		{ 
			break;
		} else
		{  
			loop++;
		}
	}

	/* get PSM and serverChannel*/
	pAttribute = sdpAccessAttributeElement(pAttrBuf, attrLen,
	                                      (uint32_t)SDP_ATTR_PROTOCOLDESCRIPTORLIST
	                                      );
	loop       = 1;
	while ((pElement = sdpAccessElement(pAttribute, pAttrBufEnd, loop)) != NULL)
	{  /* try to get protocoll specific parameter0                           */
		pAttributeParameter = sdpAccessElement(pElement, pAttrBufEnd, 2);

		/* if we found a parameter => get pointer to prot. UUID and decode it */
		if (pAttributeParameter)
		{  
			switch ((uint16_t)sdpGetDValue(sdpAccessElement(pElement, pAttrBufEnd, 1), pAttrBufEnd))
			{  
			case UUID_RFCOMM: 
			case UUID_L2CAP: 
			   serverChannel = (uint8_t)sdpGetDValue(pAttributeParameter, pAttrBufEnd);
			   break;
			default: 
			   break;
			}
		}
		loop++;
	}

	/* get service name */
	pAttribute = sdpAccessAttributeElement(pAttrBuf, attrLen,
	                                      (uint32_t)(SDP_ATTR_SERVICENAME + SDP_BASE_LANG_OFFSET)
	                                      );
	sdpGetString(pAttribute, pAttrBufEnd, (uint8_t *)serviceName, BLUE_API_MDEP_NAME_LENGTH);

    /* get remote version */
    pAttribute = sdpAccessAttributeElement(pAttrBuf, attrLen,
                                           (uint32_t)SDP_ATTR_BLUETOOTHPROFILEDESCRIPTORLIST
                                           );
    if (pAttribute)
    {
        loop = 1;
        while ((pElement = sdpAccessElement(pAttribute, pAttrBufEnd, loop)) != NULL)
        {
            pAttributeParameter = sdpAccessElement(pElement, pAttrBufEnd, 2);

            if (pAttributeParameter)
            {
                remoteversion = (uint16_t)sdpGetDValue(pAttributeParameter, pAttrBufEnd);
                break;
            }
            loop++;
        }
    }

    /* get supported feeatrues */
    pAttribute = sdpAccessAttributeElement(pAttrBuf, attrLen,
                                           (uint32_t)SDP_ATTR_SUPPORTEDFEATURES
                                           );
    if (pAttribute)
    {
        supportedfeatures = (uint16_t)sdpGetDValue(pAttribute, pAttrBufEnd);
    }

	if(pBlueAPIdata->SM_SDP_DS_Command == blueAPI_EventSDPDiscoveryReq)
	{
		PBlueAPI_SDPDiscoveryReq pProgress   = &pBlueAPIdata->SM_SDP_DS_CommandMsg->p.SDPDiscoveryReq;

		blueAPI_Send_SDPEndpointInd(pBlueAPIdata,
		                          serviceHandle,
		                          pProgress->remote_BD,
		                          (uint8_t)serverChannel,
		                          (TBlueAPI_MDEPDataType)serviceUUID,
		                          serviceName,
		                          remoteversion,
		                          supportedfeatures
		                         );
	}
}

/**
* @brief  sdp decode gatt attribute
* 
* @param  pBlueAPIdata
* @param  serviceHandle
* @param  pAttrBuf
* @param  attrLen
*
* @return  
*
*/
void blueAPI_SDPDecodeGATTAttr(PBlueAPI_Data pBlueAPIdata, uint32_t serviceHandle, uint8_t * pAttrBuf, uint16_t attrLen)
{
	PBlueAPI_GATTSDPDiscoveryReq pProgress   = &pBlueAPIdata->SM_SDP_DS_CommandMsg->p.GATTSDPDiscoveryReq;
	uint8_t *                       pAttrBufEnd = pAttrBuf + attrLen;

	uint8_t * pAttribute;
	uint8_t * pAttributeParameter;
	uint8_t * pElement;
	uint16_t   serviceUUID;
	uint16_t   serviceStartHandle  = 0;
	uint16_t   serviceEndHandle    = 0;
	uint16_t   loop;

	pAttribute  = sdpAccessAttributeElement(pAttrBuf, attrLen,
	                                      (uint32_t)SDP_ATTR_SERVICECLASSIDLIST
	                                      );

	pElement    = sdpAccessElement(pAttribute, pAttrBufEnd, 1);
	serviceUUID = (uint16_t)sdpGetDValue(pElement, pAttrBufEnd);

	pAttribute  = sdpAccessAttributeElement(pAttrBuf, attrLen,
	                                      (uint32_t)SDP_ATTR_PROTOCOLDESCRIPTORLIST
	                                      );

	loop = 1;
	while ((pElement = sdpAccessElement(pAttribute, pAttrBufEnd, loop)) != NULL)
	{
		pAttributeParameter = sdpAccessElement(pElement, pAttrBufEnd, 1);
		switch ((uint16_t)sdpGetDValue(pAttributeParameter, pAttrBufEnd))
		{
		case UUID_ATT:
			serviceStartHandle = (uint16_t)sdpGetDValue(sdpAccessElement(pElement, pAttrBufEnd, 2), pAttrBufEnd);
			serviceEndHandle   = (uint16_t)sdpGetDValue(sdpAccessElement(pElement, pAttrBufEnd, 3), pAttrBufEnd);
			break;

		default:
			break;
		}
		loop++;
	}

	blueAPI_Send_GATTSDPDiscoveryInd(pBlueAPIdata,
	                               serviceHandle,
	                               pProgress->remote_BD,
	                               serviceUUID,
	                               serviceStartHandle,
	                               serviceEndHandle
	                               );
}

/**
* @brief  sdp get next service
* 
* @param  app
* @param  bLinkContext
* @param  handles
* @param  count
* @param  offset
*
* @return  
*
*/
void blueAPI_SDPGetNextService(PBlueAPI_Data pBlueAPIdata)
{
	uint32_t* pHandles;

	if ((pBlueAPIdata->pSearchConf != NULL) &&
	  (pBlueAPIdata->pSearchConf->totalHandles > pBlueAPIdata->serviceHandleIndex)
	 )
	{
		pHandles = (uint32_t*)((uint8_t *)pBlueAPIdata->pSearchConf->handles +
		                             pBlueAPIdata->pSearchConf->gap);

		switch (pBlueAPIdata->SM_SDP_ServiceUUID)
		{  
		case UUID_PNPINFORMATION:
			pBlueAPIdata->attributeRange = BLUEAPI_SDP_ATTR_RANGE_DID;
			break;
		case UUID_ATT:
			pBlueAPIdata->attributeRange = BLUEAPI_SDP_ATTR_RANGE_GATT;
			break;
		default:
			pBlueAPIdata->attributeRange = BLUEAPI_SDP_ATTR_RANGE_SPP;
			break;
		}

		/* request attributes */
		blueAPI_Send_BT_SDP_ATTRIBUTE_REQ(pBlueAPIdata,
		                                  pHandles[pBlueAPIdata->serviceHandleIndex],
		                                  pBlueAPIdata->attributeRange,
		                                  0 /* reassembly in SDP */
		                                  );
	}
	else
	{  
		if (pBlueAPIdata->SM_SDP_ServiceUUID == UUID_PNPINFORMATION)
		{
			/* no DID record found -> start NameDiscovery */
			btApiBT_HCI_NAME_REQ(pBlueAPIdata->AppHandle, pBlueAPIdata->pSDPLinkContext->pMCL->bd);
		}
		else		
		{
			/* no SDP services found / all requested -> disconnect */
			blueAPI_ChannelDisconnect(pBlueAPIdata, pBlueAPIdata->pSDPLinkContext, FALSE);
		}

		if (pBlueAPIdata->pSearchConf != NULL)
		{
			osBufferRelease(((uint8_t *)pBlueAPIdata->pSearchConf - offsetof(TblueFaceMsg, p.sdpSearchConfirmation)));
			pBlueAPIdata->pSearchConf = NULL;
		}
	}
} /* blueAPI_SDPGetNextService */
