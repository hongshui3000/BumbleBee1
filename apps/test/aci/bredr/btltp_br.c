enum { __FILE_NUM__ = 0 };

#include "btltp_br.h"
#include "gattsvc_dis.h"
#include "gattsvc_cts.h"
#include "gattsvc_bas.h"
#include "gattsvc_gls.h"
#include "gattsvc_ndcs.h"
#include "gattsvc_rtus.h"

#include <blueapi_types.h>
#include <blueapi.h>
//#include "btltp.h"
//#include "gatt.h"
#include "trace.h"

#define LTP_SOURCE_FILE_ID 0x84


PVOID blueAPI_GATTServiceGet(GATTServiceID serviceID,
                              LPWORD                 pNbrOfAttrib)
{
	void * pService = NULL;

	switch( serviceID )
	{
	default:
		*pNbrOfAttrib = 0;
		break;
    case blueAPI_ApplicationDefined:
		pService      = (void *)GattdFindMeProfile;
		*pNbrOfAttrib = gattSvcFindMeNbrOfAttrib;
		break;    
        
	case blueAPI_ServiceBAS:
		pService      = (void *)gattSvcBAS;
		*pNbrOfAttrib = gattSvcBASNbrOfAttrib;
		break;

	case blueAPI_ServiceDIS:
		pService      = (void *)gattSvcDIS;
		*pNbrOfAttrib = gattSvcDISNbrOfAttrib;
		break;

	case blueAPI_ServiceRTUS:
		pService      = (void *)gattSvcRTUS;
		*pNbrOfAttrib = gattSvcRTUSNbrOfAttrib;
		break;

	case blueAPI_ServiceGLS:
		pService      = (void *)gattSvcGLS;
		*pNbrOfAttrib = gattSvcGLSNbrOfAttrib;
		break;    
		
	case blueAPI_ServiceNDCS:
		pService      = (void *)gattSvcNDCS;
		*pNbrOfAttrib = gattSvcNDCSNbrOfAttrib;
		break;     

	case blueAPI_ServiceCTS:
		pService      = (void *)gattSvcCTS;
		*pNbrOfAttrib = gattSvcCTSNbrOfAttrib;
		break;     

	}

	return pService;
}

BOOL LTPLibSendAuthResultRequestInd(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t * rem_BD)
{
  return LTPLibSendMessage_BD(pLTPLib, copmsk, pOpt, LTP_AUTH_RESULT_REQUEST_IND, rem_BD);
}

BOOL LTPLibSendAuthDeleteRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause, uint8_t * rem_BD)
{

  return LTPLibSendMessage_BYTE_BD(pLTPLib, copmsk, pOpt, LTP_AUTH_DELETE_RSP, cause, rem_BD);
}

BOOL LTPLibSendAuthListRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause, uint8_t * rem_BD)
{
  return LTPLibSendMessage_BYTE_BD(pLTPLib, copmsk, pOpt, LTP_AUTH_LIST_RSP, cause, rem_BD);
}


BOOL LTPLibSendConnectMDLRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause, uint8_t * rem_BD)
{
  uint16_t   pos     = pLTPLib->SendOffset;
  uint8_t * pBuffer = LTPLibWriteHeader(pLTPLib, &pos, LTP_CONNECT_MDL_RSP, copmsk, pOpt, 0);

  if (!pBuffer)
  {
    return FALSE;
  }

  pBuffer[pos++]=(uint8_t)cause;
  memcpy(&pBuffer[pos],rem_BD,6);
  pos+=6;


  return (BTLTPTgtSendLTPMessage(pLTPLib->AppHandle, pBuffer, pLTPLib->SendOffset, pos - pLTPLib->SendOffset));
} /* end of LTPLibSendConnectMDLRsp */

BOOL LTPLibSendConfigTunnelRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause)
{
  return LTPLibSendMessage_BYTE(pLTPLib, copmsk, pOpt, LTP_CONFIG_TUNNEL_RSP, cause);
}


BOOL LTPLibSendRegisterSPPMDEPRsp(PLTPLib pLTPLib,uint8_t copmsk, uint8_t * pOpt, uint8_t cause, uint8_t MDEP_handle)
{
  return LTPLibSendMessage_BYTE_BYTE(pLTPLib, copmsk, pOpt, LTP_REGISTER_SPP_MDEP_RSP, cause, MDEP_handle);
} /* end of LTPLibSendRegisterSPPMDEPRsp */

BOOL LTPLibSendDeviceConfigDeviceSetRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause)
{
    return LTPLibSendMessage_BYTE(pLTPLib, copmsk, pOpt, LTP_DEVICE_CONFIG_DEVICE_SET_RSP, cause);
}

BOOL LTPLibSendDeviceConfigDIDSetRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause)
{
  return LTPLibSendMessage_BYTE(pLTPLib, copmsk, pOpt, LTP_DEVICE_CONFIG_DID_SET_RSP, cause);
}

BOOL LTPLibSendDeviceConfigSPPSetRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause)
{
  return LTPLibSendMessage_BYTE(pLTPLib, copmsk, pOpt, LTP_DEVICE_CONFIG_SPP_SET_RSP, cause);
}

BOOL LTPLibSendDeviceConfigPagescanSetRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause)
{
  return LTPLibSendMessage_BYTE(pLTPLib, copmsk, pOpt, LTP_DEVICE_CONFIG_PAGESCAN_SET_RSP, cause);
}

BOOL LTPLibSendDeviceConfigLinkpolicySetRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause)
{
  return LTPLibSendMessage_BYTE(pLTPLib, copmsk, pOpt, LTP_DEVICE_CONFIG_LINKPOLICY_SET_RSP, cause);
}

BOOL LTPLibSendACLSniffSubrateInfo(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt,
                                 uint8_t * rem_BD, uint16_t maxTxLatency, uint16_t maxRxLatency,
                                 uint16_t minRemoteTimeout, uint16_t minLocalTimeout
                                 )
{
  uint16_t   pos     = pLTPLib->SendOffset;
  uint8_t * pBuffer = LTPLibWriteHeader(pLTPLib, &pos, LTP_ACL_SNIFF_SUBRATE_INFO, copmsk, pOpt, 0);

  if (!pBuffer)
  {
    return FALSE;
  }

  memcpy(&pBuffer[pos], rem_BD, 6); pos += 6;
  NETSHORT2CHAR(&pBuffer[pos], maxTxLatency); pos += 2;
  NETSHORT2CHAR(&pBuffer[pos], maxRxLatency); pos += 2;
  NETSHORT2CHAR(&pBuffer[pos], minRemoteTimeout); pos += 2;
  NETSHORT2CHAR(&pBuffer[pos], minLocalTimeout); pos += 2;

  return (BTLTPTgtSendLTPMessage(pLTPLib->AppHandle, pBuffer, pLTPLib->SendOffset, pos - pLTPLib->SendOffset));
} /* LTPLibSendACLSniffSubrateInfo */

 
BOOL LTPLibSendAuthResultInd(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause, uint8_t * rem_BD, uint8_t * linkKey, uint8_t keyType, uint32_t appData)
{
  uint16_t   pos     = pLTPLib->SendOffset;
  uint8_t * pBuffer = LTPLibWriteHeader(pLTPLib, &pos, LTP_AUTH_RESULT_IND, copmsk, pOpt, 0);

  if(!pBuffer)
  {
     return TRUE;
  }

  pBuffer[pos++] = cause;
  memcpy(&pBuffer[pos],rem_BD,6); pos+=6;
  memcpy(&pBuffer[pos],linkKey,16); pos+=16;
  pBuffer[pos++] = keyType;
  NETLONG2CHAR(&pBuffer[pos],appData); pos+=4;

  return BTLTPTgtSendLTPMessage(pLTPLib->AppHandle, pBuffer, pLTPLib->SendOffset, pos - pLTPLib->SendOffset);
} /* end of LTPLibSendAuthResultInd */


#if 0
//  DownStream need LTPLibSend  end

//  UpStream need LTPLibSend  start
#endif


BOOL LTPLibSendReleaseMDEPRsp(PLTPLib pLTPLib,uint8_t copmsk, uint8_t * pOpt, uint8_t cause)
{
  return LTPLibSendMessage_BYTE(pLTPLib, copmsk, pOpt, LTP_RELEASE_MDEP_RSP, cause);
}


BOOL LTPLibSendAuthRequestInd(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t * rem_BD)
{
  return LTPLibSendMessage_BD(pLTPLib, copmsk, pOpt, LTP_AUTH_REQUEST_IND, rem_BD);
}


BOOL LTPLibSendRadioModeSetRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause)
{
  return LTPLibSendMessage_BYTE(pLTPLib, copmsk, pOpt, LTP_RADIO_MODE_SET_RSP, cause);
}


BOOL LTPLibSendSPPDiscoveryRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause)
{
  return LTPLibSendMessage_BYTE(pLTPLib, copmsk, pOpt, LTP_SPP_DISCOVERY_RSP, cause);
}


BOOL LTPLibSendSPPEndpointInfo(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t * rem_BD, uint16_t rem_MDEP_Type, uint8_t rem_MDEP_ID, uint8_t * rem_MDEP_Name)
{
  uint16_t   strLen  = (uint16_t) strlen((char*)rem_MDEP_Name)+1;
  uint16_t   pos     = pLTPLib->SendOffset;
  uint8_t * pBuffer = LTPLibWriteHeader(pLTPLib, &pos, LTP_SPP_ENDPOINT_INFO, copmsk, pOpt, strLen);

  if (!pBuffer)
  {
    return FALSE;
  }

  memcpy(&pBuffer[pos],rem_BD,6);
  pos+=6;

  NETSHORT2CHAR(&pBuffer[pos],rem_MDEP_Type);
  pos+=2;

  pBuffer[pos++]=rem_MDEP_ID;

  memcpy(&pBuffer[pos],rem_MDEP_Name,strLen);
  pos+=strLen;

  return (BTLTPTgtSendLTPMessage(pLTPLib->AppHandle, pBuffer, pLTPLib->SendOffset, pos - pLTPLib->SendOffset));
} /* end of LTPLibSendSPPEndpointInfo */


BOOL LTPLibSendInquiryRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause)
{
  return LTPLibSendMessage_BYTE(pLTPLib, copmsk, pOpt, LTP_INQUIRY_RSP, cause);
}


BOOL LTPLibSendInquiryDeviceInfo(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t * rem_BD, uint8_t * rem_Device_Name)
{
  uint16_t   strLen  = (uint16_t) strlen((char*)rem_Device_Name)+1;
  uint16_t   pos     = pLTPLib->SendOffset;
  uint8_t * pBuffer = LTPLibWriteHeader(pLTPLib, &pos, LTP_INQUIRY_DEVICE_INFO, copmsk, pOpt, strLen);

  if (!pBuffer)
  {
    return FALSE;
  }

  memcpy(&pBuffer[pos],rem_BD,6);               pos+=6;
  memcpy(&pBuffer[pos],rem_Device_Name,strLen); pos+=strLen;

  return (BTLTPTgtSendLTPMessage(pLTPLib->AppHandle, pBuffer, pLTPLib->SendOffset, pos - pLTPLib->SendOffset));
} /* end of LTPLibSendInquiryDeviceInfo */


BOOL LTPLibSendDIDDeviceInfo(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t * rem_BD, uint16_t rem_VendorID, uint16_t rem_Product_ID, uint16_t rem_Version, uint8_t * rem_Device_Name)
{
  uint16_t   strLen  = (uint16_t) strlen((char*)rem_Device_Name)+1;
  uint16_t   pos     = pLTPLib->SendOffset;
  uint8_t * pBuffer = LTPLibWriteHeader(pLTPLib, &pos, LTP_DID_DEVICE_INFO, copmsk, pOpt, strLen);

  if (!pBuffer)
  {
    return FALSE;
  }

  memcpy(&pBuffer[pos],rem_BD,6);
  pos+=6;

  NETSHORT2CHAR(&pBuffer[pos],rem_VendorID);
  pos+=2;
  NETSHORT2CHAR(&pBuffer[pos],rem_Product_ID);
  pos+=2;
  NETSHORT2CHAR(&pBuffer[pos],rem_Version);
  pos+=2;

  memcpy(&pBuffer[pos],rem_Device_Name,strLen); pos+=strLen;

  return (BTLTPTgtSendLTPMessage(pLTPLib->AppHandle, pBuffer, pLTPLib->SendOffset, pos - pLTPLib->SendOffset));
} /* end of LTPLibSendDIDDeviceInfo */


 
BOOL LTPLibSendAuthListInfo(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t * rem_BD, uint8_t keyType, uint32_t appData, uint8_t * rem_Device_Name)
{
  uint16_t   pos     = pLTPLib->SendOffset;
  uint16_t   strLen  = (uint16_t) strlen((char*)rem_Device_Name)+1;
  uint8_t * pBuffer = LTPLibWriteHeader(pLTPLib, &pos, LTP_AUTH_LIST_INFO, copmsk, pOpt, strLen);

  if(!pBuffer)
  {
     return TRUE;
  }

  memcpy(&pBuffer[pos],rem_BD,6); pos+=6;
  pBuffer[pos++] = keyType;
  NETLONG2CHAR(&pBuffer[pos],appData); pos+=4;
  memcpy(&pBuffer[pos],rem_Device_Name,strLen); pos+=strLen;

  return BTLTPTgtSendLTPMessage(pLTPLib->AppHandle, pBuffer, pLTPLib->SendOffset, pos - pLTPLib->SendOffset);
} /* end of LTPLibSendAuthListInfo */


BOOL LTPLibSendAuthRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause, uint8_t * rem_BD)
{
  return LTPLibSendMessage_BYTE_BD(pLTPLib, copmsk, pOpt, LTP_AUTH_RSP, cause, rem_BD);
}


BOOL LTPLibSendAuthorizationReqInd(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t * rem_BD)
{
  return LTPLibSendMessage_BD(pLTPLib, copmsk, pOpt, LTP_AUTHORIZATION_REQ_IND, rem_BD);
}


BOOL LTPLibSendUserConfRequestInd(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t * rem_BD, uint32_t displayValue)
{
  return LTPLibSendMessage_BD_DWORD(pLTPLib, copmsk, pOpt, LTP_USER_CONF_REQUEST_IND, rem_BD, displayValue);
}


BOOL LTPLibSendKeypressNotificationRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause)
{
  return LTPLibSendMessage_BYTE(pLTPLib, copmsk, pOpt, LTP_KEYPRESS_NOTIFICATION_RSP, cause);
}


BOOL LTPLibSendKeypressNotificationInfo(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t * rem_BD, uint8_t eventType)
{
  return LTPLibSendMessage_BD_BYTE(pLTPLib, copmsk, pOpt, LTP_KEYPRESS_NOTIFICATION_INFO, rem_BD, eventType);
} /* end of LTPLibSendKeypressNotificationInfo */


BOOL LTPLibSendLegacyRemoteOOBRequestInd(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t * rem_BD)
{
    return LTPLibSendMessage_BD(pLTPLib, copmsk, pOpt, LTP_LEGACY_REMOTE_OOB_REQUEST_IND, rem_BD);
}


BOOL LTPLibSendLocalOOBRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause, uint8_t * pC, uint8_t * pR)
{
  uint16_t   pos     = pLTPLib->SendOffset;
  uint8_t * pBuffer = LTPLibWriteHeader(pLTPLib, &pos, LTP_LOCAL_OOB_RSP, copmsk, pOpt, 0);

  if (!pBuffer)
  {
    return FALSE;
  }

  pBuffer[pos++]=(uint8_t)cause;
  memcpy(&pBuffer[pos],pC,16); pos+=16;
  memcpy(&pBuffer[pos],pR,16); pos+=16;

  return (BTLTPTgtSendLTPMessage(pLTPLib->AppHandle, pBuffer, pLTPLib->SendOffset, pos - pLTPLib->SendOffset));
} /* end of LTPLibSendLocalOOBRsp */


BOOL LTPLibSendDeviceNameRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause, uint8_t * rem_BD, uint8_t * rem_Device_Name)
{
  uint16_t   pos     = pLTPLib->SendOffset;
  uint16_t   strLen  = (uint16_t) strlen((char*)rem_Device_Name)+1;
  uint8_t * pBuffer = LTPLibWriteHeader(pLTPLib, &pos, LTP_DEVICE_NAME_RSP, copmsk, pOpt, strLen);

  if (!pBuffer)
  {
    return FALSE;
  }

  pBuffer[pos++]=(uint8_t)cause;
  memcpy(&pBuffer[pos],rem_BD,6); pos+=6;
  memcpy(&pBuffer[pos],rem_Device_Name,strLen); pos+=strLen;

  return (BTLTPTgtSendLTPMessage(pLTPLib->AppHandle, pBuffer, pLTPLib->SendOffset, pos - pLTPLib->SendOffset));
} /* end of LTPLibSendDeviceNameRsp */

BOOL LTPLibSendGATTSDPDiscoveryRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause)
{
  return LTPLibSendMessage_BYTE(pLTPLib, copmsk, pOpt, LTP_GATT_SDP_DISCOVERY_RSP, cause);
}

BOOL LTPLibSendGATTSDPDiscoveryInfo(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t * rem_BD, uint16_t serviceUuid, uint16_t startHandle, uint16_t endHandle)
{
  uint16_t   pos     = pLTPLib->SendOffset;
  uint8_t * pBuffer = LTPLibWriteHeader(pLTPLib, &pos, LTP_GATT_SDP_DISCOVERY_INFO, copmsk, pOpt, 0);

  if (!pBuffer)
  {
    return FALSE;
  }

  memcpy(&pBuffer[pos],rem_BD,6);  pos+=6;
  NETSHORT2CHAR(&pBuffer[pos], serviceUuid);  pos+=2;
  NETSHORT2CHAR(&pBuffer[pos], startHandle);  pos+=2;
  NETSHORT2CHAR(&pBuffer[pos], endHandle);  pos+=2;

  return (BTLTPTgtSendLTPMessage(pLTPLib->AppHandle, pBuffer, pLTPLib->SendOffset, pos - pLTPLib->SendOffset));
}


BOOL LTPLibSendACLConfigLinkpolicyRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt,
                                      uint8_t cause, uint8_t * rem_BD
                                      )
{
  return LTPLibSendMessage_BYTE_BD(pLTPLib, copmsk, pOpt, LTP_ACL_CONFIG_LINKPOLICY_RSP, cause, rem_BD);
} /* LTPLibSendACLConfigLinkpolicyRsp */


BOOL LTPLibSendACLConfigSniffmodeRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt,
                                     uint8_t cause, uint8_t * rem_BD
                                     )
{
  return LTPLibSendMessage_BYTE_BD(pLTPLib, copmsk, pOpt, LTP_ACL_CONFIG_SNIFFMODE_RSP, cause, rem_BD);
} /* LTPLibSendACLConfigSniffmodeRsp */


BOOL LTPLibSendACLConfigLinkstatusRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt,
                                     uint8_t cause, uint8_t * rem_BD, uint8_t rem_BD_Type
                                     )
{
  uint16_t   pos     = pLTPLib->SendOffset;
  uint8_t * pBuffer = LTPLibWriteHeader(pLTPLib, &pos, LTP_ACL_CONFIG_LINKSTATUS_RSP, copmsk, pOpt, 0);

  if(!pBuffer)
  {
    return TRUE;
  }

  pBuffer[pos++] = cause;
  memcpy(&pBuffer[pos], rem_BD, 6); pos += 6;
  pBuffer[pos++] = rem_BD_Type;

  return BTLTPTgtSendLTPMessage(pLTPLib->AppHandle, pBuffer, pLTPLib->SendOffset, pos - pLTPLib->SendOffset);
} /* LTPLibSendACLConfigLinkstatusRsp */


BOOL LTPLibSendACLLinkstatusInfo(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt,
                                 uint8_t * rem_BD, uint8_t rem_BD_Type, uint8_t linkQuality, uint8_t rssi,
                                 uint16_t failedContacts, uint8_t txPower, uint8_t absoluteRssi
                                 )
{
  uint16_t   pos     = pLTPLib->SendOffset;
  uint8_t * pBuffer = LTPLibWriteHeader(pLTPLib, &pos, LTP_ACL_LINKSTATUS_INFO, copmsk, pOpt, 0);

  if (!pBuffer)
  {
    return FALSE;
  }

  memcpy(&pBuffer[pos], rem_BD, 6); pos += 6;
  pBuffer[pos++] = rem_BD_Type;
  pBuffer[pos++] = linkQuality;
  pBuffer[pos++] = rssi;
  NETSHORT2CHAR(&pBuffer[pos], failedContacts); pos += 2;
  pBuffer[pos++] = txPower;
  pBuffer[pos++] = absoluteRssi;

  return (BTLTPTgtSendLTPMessage(pLTPLib->AppHandle, pBuffer, pLTPLib->SendOffset, pos - pLTPLib->SendOffset));
} /* LTPLibSendACLLinkstatusInfo */



void BTLTPHandleAuthDeleteReq(PBTLtp pBTLtp,uint8_t copmsk, uint8_t * pOpt,uint16_t lenPara,uint8_t * pPara)
{
   uint16_t   pos = 0;
   uint8_t * rem_BD;                                              /* mandatory */
   TBlueAPI_RemoteBDType rem_BD_Type = blueAPI_RemoteBDTypeClassic;

   /* read mandatory parameters                                             */
   rem_BD = &pPara[pos]; pos+=6;

   /* read optional parameters                                              */
   pos = 0;
   if (copmsk & LTP_AUTH_DELETE_REQ_OPT_MASK_BD_TYPE)
   {
     rem_BD_Type = (TBlueAPI_RemoteBDType)pOpt[pos++];
   }
   
    blueAPI_AuthDeleteReq(rem_BD, rem_BD_Type);
}

void BTLTPHandleAuthListReq(PBTLtp pBTLtp,uint8_t copmsk, uint8_t * pOpt,uint16_t lenPara,uint8_t * pPara)
{
   uint16_t   pos = 0;
   uint8_t * rem_BD;
   TBlueAPI_RemoteBDType rem_BD_Type = blueAPI_RemoteBDTypeClassic;

   /* read mandatory parameters                                             */
   rem_BD = &pPara[pos]; pos+=6;

   /* read optional parameters                                              */
   pos = 0;
   if (copmsk & LTP_AUTH_LIST_REQ_OPT_MASK_BD_TYPE)
   {
     rem_BD_Type = (TBlueAPI_RemoteBDType)pOpt[pos++];
   }
   /* non */

   blueAPI_AuthListReq(//pBTLtp->pMsgBuffer,
                       //pBTLtp->blueAPIHandle,
                       rem_BD,
                       rem_BD_Type
                       );

}

void BTLTPHandleConnectMDLReq(PBTLtp pBTLtp,uint8_t copmsk, uint8_t * pOpt,uint16_t lenPara,uint8_t * pPara)
{
   uint16_t                 pos = 0;
   uint8_t *               rem_BD;                                /* mandatory */
   //uint8_t                 rem_MDEP_ID;                           /* mandatory */
   uint8_t                 loc_MDEP_ID    = 0x01;                 /* default   */
   TBlueAPI_LinkConfigType linkConfigType = blueAPI_LinkConfigSPPBasic;/* default */

   /* read mandatory parameters                                             */
   rem_BD = pPara; pos+=6;

   //rem_MDEP_ID = pPara[pos++];

   /* read optional parameters                                              */
   pos = 0;
   //if(copmsk&LTP_CONNECT_MDL_REQ_OPT_MASK_LINK_TYPE)
   //{  //linkConfigType = (TBlueAPI_LinkConfigType)pOpt[pos++];		//In SerialTester, linkConfigType is not used
   //}

   if(copmsk&LTP_CONNECT_MDL_REQ_OPT_MASK_LOC_MDEP_ID)
   {  loc_MDEP_ID = pOpt[pos++];
   }

   switch (linkConfigType)
   {
#if (0)
     case blueAPI_LinkConfigSPPBasic:
       /* SPP connection */
       blueAPI_ConnectSPPMDLReq(//pBTLtp->pMsgBuffer,
                                //pBTLtp->blueAPIHandle,
                                rem_BD,
                                rem_MDEP_ID,
                                loc_MDEP_ID
                                );
       break;
#endif

     default:
       {
         uint8_t pOpt[2];

         pOpt[0] = 0x00; /* loc_MDL_ID */
         pOpt[1] = loc_MDEP_ID;

         LTPLibSendConnectMDLRsp(&pBTLtp->LTPLib,
                                 (BTLTP_DEFAULT_COPMSK |
                                  LTP_CONNECT_MDL_RSP_OPT_MASK_LOC_MDL_ID |
                                  LTP_CONNECT_MDL_RSP_OPT_MASK_LOC_MDEP_ID),
                                 pOpt,
                                 LTP_CAUSE_INVALID_PARAMETER,
                                 rem_BD
                                 );
         break;
       }
   }
}

void BTLTPHandleAuthRequestCnf(PBTLtp pBTLtp,uint8_t copmsk, uint8_t * pOpt,uint16_t lenPara,uint8_t * pPara)
{
   uint16_t          pos            = 0;
   uint8_t *        rem_BD         = NULL;                        /* mandatory */
   uint8_t          authCodeLength = 0;                           /* mandatory */
   uint8_t *        authCode       = NULL;                        /* mandatory */
   uint8_t          cause;                                        /* mandatory */


   /* read mandatory parameters                                             */
   cause = pPara[pos++];

   if(cause!=LTP_CAUSE_NOT_SUPPORTED)
   {  rem_BD          = &pPara[pos]; pos+=6;
      authCodeLength  = (uint8_t)(lenPara - pos);
      authCode        = &pPara[pos];
   }

   /* read optional parameters                                              */
   /* non */

   blueAPI_UserAuthRequestConf(rem_BD,
                               authCodeLength,
                               authCode,
                               BTLTPConvertLTPtoCOMcause(cause)
                               );
}

void BTLTPHandleReleaseMDEPReq(PBTLtp pBTLtp,uint8_t copmsk, uint8_t * pOpt,uint16_t lenPara,uint8_t * pPara)
{

}

void BTLTPHandleInquiryReq(PBTLtp pBTLtp,uint8_t copmsk, uint8_t * pOpt,uint16_t lenPara,uint8_t * pPara)
{
   uint8_t limitedInquiry = 0;
   uint8_t cancelInquiry  = 0;
   uint16_t pos            = 0;

   /* read mandatory parameters                                             */
   // non...

   /* read optional parameters                                              */
   if (copmsk & LTP_INQUIRY_REQ_OPT_MASK_LIMITED_INQUIRY)
   {
     limitedInquiry = pOpt[pos++];
   }

   if (copmsk & LTP_INQUIRY_REQ_OPT_MASK_CANCEL_INQUIRY)
   {
     cancelInquiry = pOpt[pos++];
   }

   blueAPI_InquiryReq(limitedInquiry,
                      cancelInquiry,
                      10
                      );
}

void BTLTPHandleAuthReq(PBTLtp pBTLtp,uint8_t copmsk, uint8_t * pOpt,uint16_t lenPara,uint8_t * pPara)
{
   uint16_t   pos = 0;
   uint8_t * rem_BD;                                              /* mandatory */


   /* read mandatory parameters                                             */
   rem_BD = &pPara[pos]; pos+=6;

   /* read optional parameters                                              */
   /* non */

   blueAPI_AuthReq( rem_BD);
}

void BTLTPHandleAuthResultRequestCnf(PBTLtp pBTLtp,uint8_t copmsk, uint8_t * pOpt,uint16_t lenPara,uint8_t * pPara)
{
  uint16_t          pos            = 0;
  uint8_t *        rem_BD         = NULL;                        /* mandatory */
  uint8_t *        linkKey        = NULL;                        /* mandatory */
  uint8_t          keyType        = 0;                           /* mandatory */
  uint8_t          cause;                                        /* mandatory */


  /* read mandatory parameters                                             */
  cause = pPara[pos++];

  if(cause!=LTP_CAUSE_NOT_SUPPORTED)
  {
    rem_BD  = &pPara[pos]; pos+=6;
    linkKey = &pPara[pos]; pos+=16; /*LINK_KEY_SIZE*/
    keyType = pPara[pos++];
  }

  /* read optional parameters                                              */
  /* non */

  blueAPI_AuthResultRequestConf(rem_BD,
                                blueAPI_RemoteBDTypeClassic,
                                16/*LINK_KEY_SIZE*/,
                                linkKey,
                                (TBlueAPI_LinkKeyType)keyType,
                                0x0000,
                                BTLTPConvertLTPtoCOMcause(cause)
                                );
} /* BTLTPHandleAuthResultRequestCnf */

void BTLTPHandleUserConfRequestCnf(PBTLtp pBTLtp,uint8_t copmsk, uint8_t * pOpt,uint16_t lenPara,uint8_t * pPara)
{
   uint16_t   pos = 0;
   uint8_t   cause;                                               /* mandatory */
   uint8_t * rem_BD = NULL;                                       /* mandatory */

   /* read mandatory parameters                                             */
   cause = pPara[pos++];

   if(cause!=LTP_CAUSE_NOT_SUPPORTED)
   {  rem_BD = &pPara[pos]; pos+=6;
   }

   /* read optional parameters                                              */
   /* non */

    blueAPI_UserConfirmationReqConf(
								rem_BD,
								BTLTPConvertLTPtoCOMcause(cause)
								);


}

void BTLTPHandleAuthResultCnf(PBTLtp pBTLtp,uint8_t copmsk, uint8_t * pOpt,uint16_t lenPara,uint8_t * pPara)
{
   uint16_t   pos = 0;
   uint8_t * rem_BD = NULL;                                       /* mandatory */
   //uint32_t  AppData =0x00000000;                                 /* mandatory */
   uint8_t   cause;                                               /* mandatory */


   /* read mandatory parameters                                             */
   cause   = pPara[pos++];

   if(cause!=LTP_CAUSE_NOT_SUPPORTED)
   {  rem_BD  = &pPara[pos]; pos+=6;
      //AppData = NETCHAR2LONG(&pPara[pos]); pos+=4;
   }
   /* read optional parameters                                              */
   /* non */

   blueAPI_AuthResultConf(rem_BD,
                          blueAPI_RemoteBDTypeClassic,
                          //AppData,
                          BTLTPConvertLTPtoCOMcause(cause)
                          );
}

BOOL BTLTPHandleConfigTunnelReq(PBTLtp pBTLtp,uint8_t copmsk, uint8_t * pOpt,uint16_t lenPara,uint8_t * pPara)
{
   LTPLibSendConfigTunnelRsp(&pBTLtp->LTPLib, BTLTP_DEFAULT_COPMSK, NULL, LTP_CAUSE_NOT_SUPPORTED);
   return TRUE;
}

void BTLTPHandleRadioModeSetReq(PBTLtp pBTLtp,uint8_t copmsk, uint8_t * pOpt,uint16_t lenPara,uint8_t * pPara)
{
   uint16_t pos = 0;
   TBlueAPI_RadioMode localRadioMode;
   BOOL limitedDiscoverable = FALSE;

   /* read mandatory parameters                                             */
   localRadioMode = (TBlueAPI_RadioMode)pPara[pos++];

   /* read optional parameters                                              */
   pos = 0;
   if (copmsk & LTP_RADIO_MODE_SET_REQ_OPT_MASK_LIMITED_DISCOVERABLE)
   {
     limitedDiscoverable = pOpt[pos++];
   }

   blueAPI_RadioModeSetReq(localRadioMode,
                           limitedDiscoverable
                           );
}

void BTLTPHandleKeypressNotificationReq(PBTLtp pBTLtp,uint8_t copmsk, uint8_t * pOpt,uint16_t lenPara,uint8_t * pPara)
{
   uint16_t pos = 0;
   uint8_t *               rem_BD;                                /* mandatory */
   TBlueAPI_SSPKeyEvent eventType;                             /* mandatory */


   /* read mandatory parameters                                             */
   rem_BD = &pPara[pos]; pos+=6;
   eventType = (TBlueAPI_SSPKeyEvent)pPara[pos++];

   /* read optional parameters                                              */
   /* non */

   blueAPI_KeypressNotificationReq(rem_BD,
                                   eventType
                                   );
}

void BTLTPHandleLocalOOBReq(PBTLtp pBTLtp,uint8_t copmsk, uint8_t * pOpt,uint16_t lenPara,uint8_t * pPara)
{
   /* read mandatory parameters                                             */
   /* non */

   /* read optional parameters                                              */
   /* non */

   blueAPI_LocalOOBDataReq();
}

void BTLTPHandleDeviceNameReq(PBTLtp pBTLtp,uint8_t copmsk, uint8_t * pOpt,uint16_t lenPara,uint8_t * pPara)
{
   uint16_t   pos = 0;
   uint8_t * rem_BD;


   /* read mandatory parameters                                             */
   rem_BD = &pPara[pos]; pos+=6;

   /* read optional parameters                                              */
   /* non */

   blueAPI_DeviceNameReq(rem_BD);
}

#if 0
void BTLTPHandleRegisterSPPMDEPReq(PBTLtp pBTLtp,uint8_t copmsk, uint8_t * pOpt,uint16_t lenPara,uint8_t * pPara)
{
   uint16_t pos = 0;
   uint8_t *             pName;
   uint16_t               lenName     = lenPara-7;
   uint8_t               tempBuffer[BLUE_API_MDEP_NAME_LENGTH];
   uint8_t               MDEP_ID;                                 /* mandatory */
   TBlueAPI_MDEPDataType MDEP_DataType;                        /* mandatory */
   BOOL               RequireAuthenticationIn;                 /* mandatory */
   BOOL               RequireAuthorizationIn;                  /* mandatory */
   BOOL               RequireMITMIn;                           /* mandatory */
   BOOL               RequireEncryptionIn;                     /* mandatory */


   /* read mandatory parameters                                             */
   MDEP_ID = pPara[pos++];
   MDEP_DataType = (TBlueAPI_MDEPDataType)(NETCHAR2SHORT(&pPara[pos])); pos+=2;
   RequireAuthenticationIn = (BOOL)pPara[pos++];
   RequireAuthorizationIn  = (BOOL)pPara[pos++];
   RequireMITMIn           = (BOOL)pPara[pos++];
   RequireEncryptionIn     = (BOOL)pPara[pos++];

   pName                   = &pPara[pos++];

   if (lenName >= BLUE_API_MDEP_NAME_LENGTH)
   {  lenName = BLUE_API_MDEP_NAME_LENGTH -1;
#if 0
      BLUEAPI_TRACE_VERBOSITY_1(BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE,
                                "!!LTP: temp buffer to small!"
                                ));
#endif
	DBG_BUFFER(MODULE_LTP, LEVEL_ERROR, "!!LTP: temp buffer to small!", 0);
   }

   memcpy(tempBuffer,pName,lenName);
   tempBuffer[lenName]=0;

   /* read optional parameters                                              */
   /* non */

   blueAPI_RegSPPMDEPReq(MDEP_ID,
                         MDEP_DataType,
                         tempBuffer,
                         RequireAuthenticationIn,
                         RequireAuthorizationIn,
                         RequireMITMIn,
                         RequireEncryptionIn
                         );
}
#endif

void BTLTPHandleSPPDiscoveryReq(PBTLtp pBTLtp,uint8_t copmsk, uint8_t * pOpt,uint16_t lenPara,uint8_t * pPara)
{
   uint16_t               pos       = 0;
   uint8_t *             rem_BD;
   TBlueAPI_MDEPDataType rem_MDEP_Type;
   BOOL               rem_DID_Discovery = FALSE; /* not specified for LTP   */


   /* read mandatory parameters                                             */
   rem_BD = pPara; pos+=6;
   rem_MDEP_Type = (TBlueAPI_MDEPDataType)(NETCHAR2SHORT(&pPara[pos])); pos+=2;

   /* read optional parameters                                              */
   pos=0;
   if (copmsk & LTP_SPP_DISCOVERY_REQ_OPT_MASK_DID_DISCOVERY)
   {  rem_DID_Discovery = pOpt[pos] ? TRUE : FALSE;
   }

   blueAPI_SDPDiscoveryReq(rem_BD,
                           rem_MDEP_Type,
                           rem_DID_Discovery
                           );
}

void BTLTPHandleAuthorizationReqCnf(PBTLtp pBTLtp,uint8_t copmsk, uint8_t * pOpt,uint16_t lenPara,uint8_t * pPara)
{
   uint16_t   pos = 0;
   uint8_t   cause;                                               /* mandatory */
   uint8_t * rem_BD = NULL;                                       /* mandatory */


   /* read mandatory parameters                                             */
   cause = pPara[pos++];

   if(cause!=LTP_CAUSE_NOT_SUPPORTED)
   {  rem_BD = &pPara[pos]; pos+=6;
   }
   /* read optional parameters                                              */
   /* non */
   blueAPI_UserAuthorizationReqConf(rem_BD,
                                    BTLTPConvertLTPtoCOMcause(cause)
                                    );

}

void BTLTPHandleDeviceConfigDeviceSetReq(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara)
{
    uint16_t  pos = 0;
    uint32_t classOfDevice;
    uint16_t  lenDeviceName = lenPara -3;
    uint8_t  deviceName[BLUE_API_DEVICE_NAME_LENGTH];

    /* read mandatory parameters                                             */
    classOfDevice = ((NETCHAR2LONG(&pPara[pos])) >> 8) & 0x00FFFFFF; pos += 3;

    if (lenDeviceName > BLUE_API_DEVICE_NAME_LENGTH-1)
    {
        lenDeviceName = BLUE_API_DEVICE_NAME_LENGTH-1;
    }

    memcpy(deviceName, &pPara[pos], lenDeviceName); pos += lenDeviceName;
    deviceName[lenDeviceName] = 0x00;

    blueAPI_DeviceConfigDeviceSetReq(classOfDevice,
                                   deviceName
                                   );
}

/****************************************************************************/
/* void BTLTPHandleDeviceConfigDIDSetReq                                    */
/****************************************************************************/
void BTLTPHandleDeviceConfigDIDSetReq(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara)
{
  uint16_t pos = 0;
  uint16_t vendorID;
  uint8_t vendorIDSource;
  uint16_t productID;
  uint16_t productVersion;

  /* read mandatory parameters                                             */
  vendorID        = NETCHAR2SHORT(&pPara[pos]); pos += 2;
  vendorIDSource  = pPara[pos++];
  productID       = NETCHAR2SHORT(&pPara[pos]); pos += 2;
  productVersion  = NETCHAR2SHORT(&pPara[pos]); pos += 2;

  blueAPI_DeviceConfigDIDSetReq(vendorID,
                                vendorIDSource,
                                productID,
                                productVersion
                                );
}

#if (0)
/****************************************************************************/
/* void BTLTPHandleDeviceConfigSPPSetReq                                    */
/****************************************************************************/
void BTLTPHandleDeviceConfigSPPSetReq(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara)
{
  uint16_t pos = 0;
  BOOL secAUTHENout;
  BOOL secENCRout;
  BOOL secMITMout;
  BOOL secAUTHORout;

  /* read mandatory parameters                                             */
  secAUTHENout  = pPara[pos++];
  secENCRout    = pPara[pos++];
  secMITMout    = pPara[pos++];
  secAUTHORout  = pPara[pos++];

  blueAPI_DeviceConfigSPPSetReq(secAUTHENout,
                                secENCRout,
                                secMITMout,
                                secAUTHORout
                                );
}
#endif

/****************************************************************************/
/* void BTLTPHandleDeviceConfigPagescanSetReq                               */
/****************************************************************************/
void BTLTPHandleDeviceConfigPagescanSetReq(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara)
{
  uint16_t pos = 0;
  TBlueAPI_BRPageScanType     scanType;
  TBlueAPI_BRPageScanRepMode  repMode;
  uint16_t                        repInterval;
  uint16_t                        repWindow;
  uint16_t                        pageTimeout;

  /* read mandatory parameters                                             */
  scanType    = (TBlueAPI_BRPageScanType)pPara[pos++];
  repMode     = (TBlueAPI_BRPageScanRepMode)pPara[pos++];
  repInterval = NETCHAR2SHORT(&pPara[pos]); pos += 2;
  repWindow   = NETCHAR2SHORT(&pPara[pos]); pos += 2;
  pageTimeout = NETCHAR2SHORT(&pPara[pos]); pos += 2;

  blueAPI_DeviceConfigPagescanSetReq(scanType,
                                     repMode,
                                     repInterval,
                                     repWindow,
                                     pageTimeout
                                     );
} /* BTLTPHandleDeviceConfigPagescanSetReq */

/****************************************************************************/
/* void BTLTPHandleDeviceConfigLinkpolicySetReq                             */
/****************************************************************************/
void BTLTPHandleDeviceConfigLinkpolicySetReq(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara)
{
  uint16_t pos = 0;
  TBlueAPI_BRLinkPolicy linkPolicy;
  TBlueAPI_BRDeviceRole deviceRole;
  uint16_t                  supervisionTimeout;

  /* read mandatory parameters                                             */
  linkPolicy          = (TBlueAPI_BRLinkPolicy)pPara[pos++];
  deviceRole          = (TBlueAPI_BRDeviceRole)pPara[pos++];
  supervisionTimeout  = NETCHAR2SHORT(&pPara[pos]); pos += 2;

  blueAPI_DeviceConfigLinkpolicySetReq(linkPolicy,
                                       deviceRole,
                                       supervisionTimeout
                                       );
} /* BTLTPHandleDeviceConfigLinkpolicySetReq */

#if (F_BTEXT_RF_MAX_TX_PWR)
/****************************************************************************/
/* void BTLTPHandleDeviceConfigLinkpolicySetReq                             */
/****************************************************************************/
void BTLTPHandleDeviceConfigMaxTxPowerSetReq(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara)
{
  uint16_t pos = 0;
  uint8_t txPower;

  /* read mandatory parameters                                             */
  txPower = pPara[pos++];
  
#if 0
  blueAPI_DeviceConfigMaxTxPowerSetReq(pBTLtp->pMsgBuffer,
                                       pBTLtp->blueAPIHandle,
                                       txPower
                                       );
#endif
  
} /* BTLTPHandleDeviceConfigMaxTxPowerSetReq */
#endif /* (F_BTEXT_RF_MAX_TX_PWR) */

void BTLTPHandleGATTSDPDiscoveryReq(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt,
                                    uint16_t lenPara, uint8_t * pPara)
{
  uint16_t pos = 0;
  uint8_t * rem_BD;
  uint16_t rem_GATT_UUID;
  uint8_t rem_DID_Discovery;

  /* read mandatory parameters                                             */
  rem_BD            = &pPara[pos]; pos += 6;
  rem_GATT_UUID     = NETCHAR2SHORT(&pPara[pos]); pos += 2;
  rem_DID_Discovery = pPara[pos++];

  /* read optional parameters                                              */
  /* none */

  blueAPI_GATTSDPDiscoveryReq(rem_BD,
                              rem_GATT_UUID,
                              rem_DID_Discovery
                              );
} /* BTLTPHandleGATTSDPDiscoveryReq */

void BTLTPHandleACLConfigLinkpolicyReq(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt,
                                       uint16_t lenPara, uint8_t * pPara)
{
  uint16_t pos = 0;
  uint8_t *                remote_BD;
  TBlueAPI_BRLinkPolicy linkPolicy;
  TBlueAPI_BRDeviceRole deviceRole;
  uint16_t                  supervisionTimeout;

  /* read mandatory parameters                                             */
  remote_BD           = &pPara[pos]; pos += 6;
  linkPolicy          = (TBlueAPI_BRLinkPolicy)pPara[pos++];
  deviceRole          = (TBlueAPI_BRDeviceRole)pPara[pos++];
  supervisionTimeout  = NETCHAR2SHORT(&pPara[pos]); pos += 2;

  blueAPI_ACLConfigLinkpolicyReq(remote_BD,
                                 linkPolicy,
                                 deviceRole,
                                 supervisionTimeout
                                 );
} /* BTLTPHandleACLConfigLinkpolicyReq */

/****************************************************************************
 * BTLTPHandleACLConfigSniffmodeReq
 ****************************************************************************/
void BTLTPHandleACLConfigSniffmodeReq(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt,
                                      uint16_t lenPara, uint8_t * pPara)
{
  uint16_t pos = 0;
  uint8_t *                remote_BD;
  uint16_t                  minInterval;
  uint16_t                  maxInterval;
  uint16_t                  sniffAttempt;
  uint16_t                  sniffTimeout;
  uint16_t                  maxLatency;
  uint16_t                  minRemoteTimeout;
  uint16_t                  minLocalTimeout;

  /* read mandatory parameters                                             */
  remote_BD           = &pPara[pos]; pos += 6;
  minInterval         = NETCHAR2SHORT(&pPara[pos]); pos += 2;
  maxInterval         = NETCHAR2SHORT(&pPara[pos]); pos += 2;
  sniffAttempt        = NETCHAR2SHORT(&pPara[pos]); pos += 2;
  sniffTimeout        = NETCHAR2SHORT(&pPara[pos]); pos += 2;
  maxLatency          = NETCHAR2SHORT(&pPara[pos]); pos += 2;
  minRemoteTimeout    = NETCHAR2SHORT(&pPara[pos]); pos += 2;
  minLocalTimeout     = NETCHAR2SHORT(&pPara[pos]); pos += 2;

  blueAPI_ACLConfigSniffmodeReq(remote_BD,
                                minInterval,
                                maxInterval,
                                sniffAttempt,
                                sniffTimeout,
                                maxLatency,
                                minRemoteTimeout,
                                minLocalTimeout
                                );
} /* BTLTPHandleACLConfigSniffmodeReq */

#if (F_BT_HCI_RADIO_STATUS_CONF)
/****************************************************************************
 * BTLTPHandleACLConfigLinkstatusReq
 ****************************************************************************/
void BTLTPHandleACLConfigLinkstatusReq(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt,
                                       uint16_t lenPara, uint8_t * pPara)
{
  uint16_t    pos = 0;
  uint8_t *  remote_BD;
  TBlueAPI_RemoteBDType remote_BD_Type;
  uint16_t    pollInterval;

  /* read mandatory parameters                                             */
  remote_BD      = &pPara[pos]; pos += 6;
  remote_BD_Type = (TBlueAPI_RemoteBDType)(pPara[pos++]);
  pollInterval   = NETCHAR2SHORT(&pPara[pos]); pos += 2;

  blueAPI_ACLConfigLinkstatusReq(pBTLtp->pMsgBuffer,
                                pBTLtp->blueAPIHandle,
                                remote_BD,
                                remote_BD_Type,
                                pollInterval
                                );
} /* BTLTPHandleACLConfigLinkstatusReq */
#endif /* (F_BT_HCI_RADIO_STATUS_CONF) */


void BTLTPHandleLegacyOOBRequestCnf(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara)
{
    uint16_t   pos = 0;
    uint8_t * rem_BD = NULL;                                       /* mandatory */
    uint8_t * C      = NULL;                                       /* mandatory */
    uint8_t * R      = NULL;                                       /* mandatory */
    uint8_t   cause;                                               /* mandatory */


    /* read mandatory parameters                                             */
    cause = pPara[pos++];

    if(cause!=LTP_CAUSE_NOT_SUPPORTED)
    {
        rem_BD = &pPara[pos]; pos+=6;
        C      = &pPara[pos]; pos+=16;
        R      = &pPara[pos]; pos+=16;
    }
    /* read optional parameters                                              */
    /* non */

    blueAPI_LegacyRemoteOOBDataReqConf(rem_BD,
                            C,
                            R,
                            BTLTPConvertLTPtoCOMcause(cause)
                            );

}


void BTLTPHandleBREDRLtpMsg(PBTLtp pBTLtp, uint8_t command, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara)
{

    switch (command)
    {
    case LTP_DATA_UNSEGMENTED: /*-----------------------------------------*/
    case LTP_DATA_START_SEGMENT: /*---------------------------------------*/
    case LTP_DATA_END_SEGMENT: /*-----------------------------------------*/
    case LTP_DATA_CONTINUE_SEGMENT: /*------------------------------------*/
      //BTLTPHandleDataSegment(pBTLtp, command, copmsk, pOpt, lenPara, pPara);
      break;
	
    case LTP_AUTH_DELETE_REQ: /*------------------------------------------*/
      BTLTPHandleAuthDeleteReq(pBTLtp,copmsk,pOpt,lenPara,pPara);
      break;
    case LTP_AUTH_LIST_REQ: /*--------------------------------------------*/
      BTLTPHandleAuthListReq(pBTLtp,copmsk,pOpt,lenPara,pPara);
      break;
	  
    case LTP_CONNECT_MDL_REQ: /*-------------------------------------------*/
        BTLTPHandleConnectMDLReq(pBTLtp,copmsk,pOpt,lenPara,pPara);
        break;
    case LTP_AUTH_REQUEST_CNF: /*-----------------------------------------*/
      BTLTPHandleAuthRequestCnf(pBTLtp,copmsk,pOpt,lenPara,pPara);
      break;
    case LTP_RELEASE_MDEP_REQ: /*-----------------------------------------*/
      BTLTPHandleReleaseMDEPReq(pBTLtp,copmsk,pOpt,lenPara,pPara);
      break;
    case LTP_INQUIRY_REQ: /*----------------------------------------------*/
      BTLTPHandleInquiryReq(pBTLtp,copmsk,pOpt,lenPara,pPara);
      break;
    case LTP_AUTH_REQ: /*-------------------------------------------------*/
      BTLTPHandleAuthReq(pBTLtp,copmsk,pOpt,lenPara,pPara);
      break;
    case LTP_AUTH_RESULT_REQUEST_CNF: /*----------------------------------*/
      BTLTPHandleAuthResultRequestCnf(pBTLtp,copmsk,pOpt,lenPara,pPara);
      break;
    case LTP_USER_CONF_REQUEST_CNF: /*------------------------------------*/
      BTLTPHandleUserConfRequestCnf(pBTLtp,copmsk,pOpt,lenPara,pPara);
      break;	  
    case LTP_AUTH_RESULT_CNF: /*------------------------------------------*/
      BTLTPHandleAuthResultCnf(pBTLtp,copmsk,pOpt,lenPara,pPara);
      break;
    case LTP_CONFIG_TUNNEL_REQ: /*----------------------------------------*/
      BTLTPHandleConfigTunnelReq(pBTLtp,copmsk,pOpt,lenPara,pPara);
      break;	  
    case LTP_RADIO_MODE_SET_REQ: /*---------------------------------------*/
      BTLTPHandleRadioModeSetReq(pBTLtp,copmsk,pOpt,lenPara,pPara);
      break;
    case LTP_KEYPRESS_NOTIFICATION_REQ: /*--------------------------------*/
      BTLTPHandleKeypressNotificationReq(pBTLtp,copmsk,pOpt,lenPara,pPara);
      break;
    case LTP_LOCAL_OOB_REQ: /*--------------------------------------------*/
      BTLTPHandleLocalOOBReq(pBTLtp,copmsk,pOpt,lenPara,pPara);
      break;
    case LTP_DEVICE_NAME_REQ: /*------------------------------------------*/
      BTLTPHandleDeviceNameReq(pBTLtp,copmsk,pOpt,lenPara,pPara);
      break;
    case LTP_SPP_DISCOVERY_REQ: /*----------------------------------------*/
      BTLTPHandleSPPDiscoveryReq(pBTLtp,copmsk,pOpt,lenPara,pPara);
      break;
#if 0
    case LTP_REGISTER_SPP_MDEP_REQ: /*------------------------------------*/
      BTLTPHandleRegisterSPPMDEPReq(pBTLtp,copmsk,pOpt,lenPara,pPara);
      break;
#endif

    case LTP_AUTHORIZATION_REQ_CNF: /*------------------------------------*/
      BTLTPHandleAuthorizationReqCnf(pBTLtp,copmsk,pOpt,lenPara,pPara);
      break;
    case LTP_DEVICE_CONFIG_DEVICE_SET_REQ:
        BTLTPHandleDeviceConfigDeviceSetReq(pBTLtp,copmsk,pOpt,lenPara,pPara);
        break;
    case LTP_DEVICE_CONFIG_DID_SET_REQ:
      BTLTPHandleDeviceConfigDIDSetReq(pBTLtp,copmsk,pOpt,lenPara,pPara);
      break;
#if (0)
    case LTP_DEVICE_CONFIG_SPP_SET_REQ:
      BTLTPHandleDeviceConfigSPPSetReq(pBTLtp,copmsk,pOpt,lenPara,pPara);
      break;
#endif
    case LTP_DEVICE_CONFIG_PAGESCAN_SET_REQ: /*---------------------------*/
      BTLTPHandleDeviceConfigPagescanSetReq(pBTLtp, copmsk, pOpt, lenPara, pPara);
      break;
    case LTP_DEVICE_CONFIG_LINKPOLICY_SET_REQ: /*-------------------------*/
      BTLTPHandleDeviceConfigLinkpolicySetReq(pBTLtp, copmsk, pOpt, lenPara, pPara);
      break;
#if (F_BTEXT_RF_MAX_TX_PWR)
    case LTP_DEVICE_CONFIG_MAXTXPOWER_SET_REQ: /*-------------------------*/
      BTLTPHandleDeviceConfigMaxTxPowerSetReq(pBTLtp, copmsk, pOpt, lenPara, pPara);
      break;
#endif /* (F_BTEXT_RF_MAX_TX_PWR) */
    case LTP_GATT_SDP_DISCOVERY_REQ: /*-----------------------------------*/
      BTLTPHandleGATTSDPDiscoveryReq(pBTLtp, copmsk, pOpt, lenPara, pPara);
      break;
    case LTP_ACL_CONFIG_LINKPOLICY_REQ: /*--------------------------------*/
       BTLTPHandleACLConfigLinkpolicyReq(pBTLtp, copmsk, pOpt, lenPara, pPara);
       break;
     case LTP_ACL_CONFIG_SNIFFMODE_REQ: /*---------------------------------*/
       BTLTPHandleACLConfigSniffmodeReq(pBTLtp, copmsk, pOpt, lenPara, pPara);
       break;
#if (F_BT_HCI_RADIO_STATUS_CONF)
     case LTP_ACL_CONFIG_LINKSTATUS_REQ: /*--------------------------------*/
       BTLTPHandleACLConfigLinkstatusReq(pBTLtp, copmsk, pOpt, lenPara, pPara);
       break;
#endif /* (F_BT_HCI_RADIO_STATUS_CONF) */

    case LTP_LEGACY_REMOTE_OOB_REQUEST_CNF: /*-----------------------------------*/
        BTLTPHandleLegacyOOBRequestCnf(pBTLtp, copmsk, pOpt, lenPara, pPara);
        break;
		
    default: /*-----------------------------------------------------------*/
        DBG_BUFFER(MODULE_LTP, LEVEL_ERROR, "BTLTPHandleBREDRLtpMsg, unknown command = 0x%x", 1, command);
        break;
    }
	
}


#if 0
STATIC BOOL LTPHandle_DataInd(PBTLtp pBTLtp,uint8_t * pBuffer, PBlueAPI_DataInd pCOM_DataInd)
{
   uint8_t    ltpOpcode;
   uint16_t    msgLen;
   uint16_t    offset;
   uint8_t    pOpt[3];
   uint16_t    optPos = 0;
   uint8_t    copmsk = BTLTP_DEFAULT_COPMSK;
   uint16_t    pos;
   PBTLtpMDLContext     pMDLContext;
   TBlueAPI_FrameType   returnCreditPacketType;

   switch (pCOM_DataInd->frameType)
   {  case blueAPI_FrameTypeUnsegmented: /*------------------------------------*/
         ltpOpcode  = LTP_DATA_UNSEGMENTED;
         returnCreditPacketType = blueAPI_FrameTypeUnsegmented;
         break;
      case blueAPI_FrameTypeFirstSegment: /*-----------------------------------*/
         ltpOpcode  = LTP_DATA_START_SEGMENT;
         returnCreditPacketType = blueAPI_FrameTypeContinueSegment;
         break;
      case blueAPI_FrameTypeLastSegment: /*------------------------------------*/
         ltpOpcode  = LTP_DATA_END_SEGMENT;
         returnCreditPacketType = blueAPI_FrameTypeUnsegmented;
         break;
      case blueAPI_FrameTypeContinueSegment: /*--------------------------------*/
         ltpOpcode  = LTP_DATA_CONTINUE_SEGMENT;
         returnCreditPacketType = blueAPI_FrameTypeContinueSegment;
         break;
      default: /*-----------------------------------------------------------*/
#if 0
#if (F_OSIF)
         DebuggerBreak();
#endif

BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE,
                                   "!!LTP: unknown data packet type => ignore"
                                   );
#endif
	DBG_BUFFER(MODULE_LTP, LEVEL_ERROR, "!!LTP: unknown data packet type => ignore", 0);
	
         return TRUE; /* dump data */
   }

   pOpt[optPos++] = (uint8_t)pCOM_DataInd->local_MDL_ID;
   copmsk |= LTP_DATA_OPT_MASK_LOC_MDL_ID;

   pMDLContext = BTLTPFindMDLContext(pBTLtp, (uint8_t)pCOM_DataInd->local_MDL_ID);
   if (pMDLContext != NULL && pMDLContext->maxUsCredits != 0)
   {
      pMDLContext->returnCreditPacketType = returnCreditPacketType;
      /* creditbased flowcontrol: send collected credits */
      if (pMDLContext->collectedUsCredits > 0)
      {
         copmsk |= LTP_DATA_OPT_MASK_RETURN_CREDITS;
         pOpt[optPos++] = pMDLContext->collectedUsCredits;

         pMDLContext->collectedUsCredits = 0;
      }
   }
   else
   {
#if 0
	  BLUEAPI_TRACE_PRINTF_4(BLUEAPI_TRACE_MASK_TRACE,
                                "LTP: -> DataSegment type[%d] local_mdl[%d] lenFrame[%d] lenData[%d] credits[-]",
                                (uint8_t)pCOM_DataInd->frameType,
                                pCOM_DataInd->local_MDL_ID,
                                pCOM_DataInd->frameLength,
                                pCOM_DataInd->payloadLength
                                );
#endif
	  DBG_BUFFER(MODULE_LTP, LEVEL_ERROR, "!!LTP: -> DataSegment type[%d] local_mdl[%d] lenFrame[%d] lenData[%d] credits[-]", 4, 
                                (uint8_t)pCOM_DataInd->frameType,
                                pCOM_DataInd->local_MDL_ID,
                                pCOM_DataInd->frameLength,
                                pCOM_DataInd->payloadLength
                                );

      /* no creditbased flowcontrol: send DataConf in bufferCallback */
      pBTLtp->pBufferAction = BTLTPAllocateAction(pBTLtp);
      if (pBTLtp->pBufferAction)
      {
        pBTLtp->pBufferAction->Action   = btltpActionSendDataConf;
        pBTLtp->pBufferAction->p.MDL_ID = pCOM_DataInd->local_MDL_ID;
      } 
#if 0
	  else
      {

#if (F_OSIF)
        DebuggerBreak(); //ka todo: do something usefull here...
#endif

      }
#endif
   }

   offset = offsetof(TBlueAPI_UsMessage, p.DataInd.data) + pCOM_DataInd->gap;

   msgLen = LTPLibInsertHeader(&pBTLtp->LTPLib,
                               pBuffer, &offset,
                               pCOM_DataInd->payloadLength,
                               ltpOpcode,
                               copmsk, pOpt,
                               &pos
                               );
   if (msgLen > 0)
   {
     if (ltpOpcode == LTP_DATA_START_SEGMENT)
     {
       NETSHORT2CHAR(&pBuffer[pos], pCOM_DataInd->frameLength); pos += 2;
     }

     BTLTPTgtSendLTPMessage(pBTLtp, pBuffer, offset, msgLen);
   }

   return FALSE;
}

STATIC BOOL LTPHandle_DataRsp(PBTLtp pBTLtp, uint8_t * pBuffer, PBlueAPI_DataRsp pCOM_DataRsp)
{
   PBTLtpMDLContext pMDLContext = BTLTPFindMDLContext(pBTLtp, (uint8_t)pCOM_DataRsp->local_MDL_ID);
   if (pMDLContext != NULL)
   {
      if (pCOM_DataRsp->cause == blueAPI_CauseSuccess)
      {
         /* send InternalEventInfo on next DataRsp with cause != success */
         pMDLContext->flags &= ~(LTP_MDL_DATARSP_ERROR);

         if (pMDLContext->maxUsCredits != 0)
         {  /* increment used usCredits */
            pMDLContext->collectedUsCredits++;

            {
               TBlueAPI_DataInd dataInd;
               dataInd.frameType     = pMDLContext->returnCreditPacketType;
               dataInd.local_MDL_ID  = pCOM_DataRsp->local_MDL_ID;
               dataInd.gap           = sizeof(TBlueAPI_DataRsp);
               dataInd.frameLength   = 0;
               dataInd.payloadLength = 0;
               return LTPHandle_DataInd(pBTLtp, pBuffer, &dataInd);
            }
         }
      }
      else if (pCOM_DataRsp->cause != blueAPI_CauseConnectionDisconnect &&
               pCOM_DataRsp->cause != blueAPI_CauseConnectionLost &&
               !(pMDLContext->flags & LTP_MDL_DATARSP_ERROR)
              )
      {
         LTPLibSendInternalEventInfo(&pBTLtp->LTPLib,
                                     BTLTP_DEFAULT_COPMSK,
                                     NULL,
                                     BTLTPConvertCOMtoLTPcause(pCOM_DataRsp->cause),
                                     LTP_INTERNAL_EVENT_INVALID_DATA_RECEIVED,
                                     LTP_GENERATE_EVENT_ID
                                     );

         /* send this InternalEventInfo only once (limited for HW FLC based connection loss) */
         pMDLContext->flags |= LTP_MDL_DATARSP_ERROR;
      }
   }

   return TRUE;
}
#endif

#if 0
STATIC void LTPHandle_RelMDEPRsp(PBTLtp pBTLtp,PBlueAPI_RelMDEPRsp pCOM_RelMDEPRsp)
{
  uint8_t pOpt[1];

  pOpt[0] = (uint8_t)pCOM_RelMDEPRsp->MDEP_EntryHandle;

  LTPLibSendReleaseMDEPRsp(&pBTLtp->LTPLib,
                           (BTLTP_DEFAULT_COPMSK |
                            LTP_RELEASE_MDEP_RSP_OPT_MASK_MDEP_HANDLE
                           ),
                           pOpt,
                           BTLTPConvertCOMtoLTPcause(pCOM_RelMDEPRsp->cause)
                           );
}
#endif

STATIC void LTPHandle_UserAuthRequestInd(PBTLtp pBTLtp,PBlueAPI_UserAuthRequestInd pCOM_UserAuthRequestInd)
{
   LTPLibSendAuthRequestInd(&pBTLtp->LTPLib,
                            BTLTP_DEFAULT_COPMSK,
                            NULL,
                            pCOM_UserAuthRequestInd->remote_BD
                           );
}

STATIC void LTPHandle_RadioModeSetRsp(PBTLtp pBTLtp,PBlueAPI_RadioModeSetRsp pCOM_RadioModeSetRsp)
{
   LTPLibSendRadioModeSetRsp(&pBTLtp->LTPLib,
                             BTLTP_DEFAULT_COPMSK,
                             NULL,
                             BTLTPConvertCOMtoLTPcause(pCOM_RadioModeSetRsp->cause)
                            );
}


#if 0
STATIC void LTPHandle_RegSPPMDEPRsp(PBTLtp pBTLtp, PBlueAPI_RegSPPMDEPRsp pCOM_RegSPPMDEPRsp)
{
  uint8_t pOpt[3];
  uint8_t pos = 0;

  pOpt[pos++] = pCOM_RegSPPMDEPRsp->MDEP_ID;
  NETSHORT2CHAR(&pOpt[pos], pCOM_RegSPPMDEPRsp->MDEP_DataType); pos += 2;

  LTPLibSendRegisterSPPMDEPRsp(&pBTLtp->LTPLib,
                               (BTLTP_DEFAULT_COPMSK |
                                LTP_REGISTER_SPP_MDEP_RSP_OPT_MASK_MDEP_ID |
                                LTP_REGISTER_SPP_MDEP_RSP_OPT_MASK_DATA_TYPE
                               ),
                               pOpt,
                               BTLTPConvertCOMtoLTPcause(pCOM_RegSPPMDEPRsp->cause),
                               (uint8_t)pCOM_RegSPPMDEPRsp->MDEP_EntryHandle
                               );
}

STATIC void LTPHandle_CreateLegacyMDLInd(PBTLtp pBTLtp,PBlueAPI_CreateLegacyMDLInd pCOM_LegacyCreateMDLInd)
{
   uint8_t pOpt[4];

   pOpt[0] = pCOM_LegacyCreateMDLInd->remote_BD_type;
   pOpt[1] = (uint8_t)pCOM_LegacyCreateMDLInd->linkConfigType;
   pOpt[2] = pCOM_LegacyCreateMDLInd->local_MDEP_ID;
   pOpt[3] = pCOM_LegacyCreateMDLInd->remote_MDEP_ID;
   

   /* Allocate Context for this MDL */
   BTLTPAllocateMDLContext(pBTLtp,
                           (uint8_t)pCOM_LegacyCreateMDLInd->local_MDL_ID,
                           pCOM_LegacyCreateMDLInd->local_MDEP_ID,
                           pCOM_LegacyCreateMDLInd->linkConfigType
                           );

   LTPLibSendCreateMDLInd(&pBTLtp->LTPLib,
                          (BTLTP_DEFAULT_COPMSK |
                           LTP_CREATE_MDL_IND_OPT_MASK_LINK_TYPE |
                           LTP_CREATE_MDL_IND_OPT_MASK_LOC_MDEP_ID |
                           LTP_CREATE_MDL_IND_OPT_MASK_REM_MDEP_ID |
                           LTP_CREATE_MDL_IND_OPT_MASK_BD_TYPE
                          ),
                          pOpt,
                          pCOM_LegacyCreateMDLInd->remote_BD,
                          (uint8_t)pCOM_LegacyCreateMDLInd->local_MDL_ID
                         );
}
#endif


 STATIC void LTPHandle_SPPDiscoveryRsp(PBTLtp pBTLtp,PBlueAPI_SDPDiscoveryRsp pCOM_SDPDiscoveryRsp)
{
   LTPLibSendSPPDiscoveryRsp(&pBTLtp->LTPLib,
                             BTLTP_DEFAULT_COPMSK,
                             NULL,
                             BTLTPConvertCOMtoLTPcause(pCOM_SDPDiscoveryRsp->cause)
                            );
}

 STATIC BOOL LTPHandle_SPPEndpointInd(PBTLtp pBTLtp, uint8_t * pBuffer, PBlueAPI_SDPEndpointInd pCOM_SDPEndpointInd)
{
   BOOL ret = FALSE;

//#if (WIN32) || (!F_OSIF)
   /* do not use buffer for confirmation */
   pBuffer = NULL;
//#endif /* (WIN32) */

   pBTLtp->pBufferAction=BTLTPAllocateAction(pBTLtp);

   if(pBTLtp->pBufferAction)
   {  pBTLtp->pBufferAction->Action                        = btltpActionSendSPPEndpointConf;
      pBTLtp->pBufferAction->p.serviceAction.pBuffer       = pBuffer;
      pBTLtp->pBufferAction->p.serviceAction.serviceHandle = pCOM_SDPEndpointInd->serviceHandle;

      ret = LTPLibSendSPPEndpointInfo(&pBTLtp->LTPLib,
                                      BTLTP_DEFAULT_COPMSK,
                                      NULL,
                                      pCOM_SDPEndpointInd->remote_BD,
                                      pCOM_SDPEndpointInd->remote_MDEP_DataType,
                                      pCOM_SDPEndpointInd->remote_MDEP_ID,
                                      pCOM_SDPEndpointInd->remote_MDEP_Description
                                      );
   }


   if (ret == FALSE)
   {  if (pBTLtp->pBufferAction)
      {  pBTLtp->pBufferAction->Action = btltpActionNotUsed;
         pBTLtp->pBufferAction         = NULL;
      }

      blueAPI_SDPEndpointConf(//pBuffer,
                              //pBTLtp->blueAPIHandle,
                              pCOM_SDPEndpointInd->serviceHandle,
                              blueAPI_CauseResourceError
                              );
   }
   return FALSE;
}


STATIC void LTPHandle_InquiryRsp(PBTLtp pBTLtp,PBlueAPI_InquiryRsp pCOM_InquiryRsp)
{
  uint8_t pOpt[1];

  pOpt[0] = (uint8_t)pCOM_InquiryRsp->cancelInquiry;

  LTPLibSendInquiryRsp(&pBTLtp->LTPLib,
                       (BTLTP_DEFAULT_COPMSK |
                        LTP_INQUIRY_RSP_OPT_MASK_CANCEL_INQUIRY
                       ),
                       pOpt,
                       BTLTPConvertCOMtoLTPcause(pCOM_InquiryRsp->cause)
                       );
}

STATIC void LTPHandle_InquiryDeviceInfo(PBTLtp pBTLtp,PBlueAPI_InquiryDeviceInfo pCOM_InquiryDeviceInfo)
{
   uint8_t pOpt[4];
   uint16_t pos = 0;

   NET24BIT2CHAR(&pOpt[pos], pCOM_InquiryDeviceInfo->remote_Device_Class); pos += 3;
   pOpt[pos++] = pCOM_InquiryDeviceInfo->remote_RSSI;

   LTPLibSendInquiryDeviceInfo(&pBTLtp->LTPLib,
                               BTLTP_DEFAULT_COPMSK |
                               LTP_INQUIRY_DEVICE_INFO_OPT_MASK_DEVICE_CLASS |
                               LTP_INQUIRY_DEVICE_INFO_OPT_MASK_RSSI,
                               pOpt,
                               pCOM_InquiryDeviceInfo->remote_BD,
                               pCOM_InquiryDeviceInfo->remote_Device_Name
                              );
}

 
STATIC BOOL LTPHandle_DIDDeviceInd(PBTLtp pBTLtp, uint8_t * pBuffer, PBlueAPI_DIDDeviceInd pCOM_DIDDeviceInd)
{
   uint8_t pOpt[2];
   uint16_t pos = 0;
   BOOL ret = FALSE;

   NETSHORT2CHAR(&pOpt[pos], pCOM_DIDDeviceInd->remote_VendorID_Source); pos+=2;

//#if (WIN32) || (!F_OSIF)
   /* do not use buffer for confirmation */
   pBuffer = NULL;
//#endif /* (WIN32) */

   pBTLtp->pBufferAction=BTLTPAllocateAction(pBTLtp);

   if(pBTLtp->pBufferAction)
   {  pBTLtp->pBufferAction->Action                         = btltpActionSendDIDDeviceConf;
      pBTLtp->pBufferAction->p.serviceAction.pBuffer        = pBuffer;
      pBTLtp->pBufferAction->p.serviceAction.serviceHandle  = pCOM_DIDDeviceInd->serviceHandle;

      ret = LTPLibSendDIDDeviceInfo(&pBTLtp->LTPLib,
                                    BTLTP_DEFAULT_COPMSK |
                                    LTP_DID_DEVICE_INFO_OPT_MASK_VENDOR_ID_SOURCE,
                                    pOpt,
                                    pCOM_DIDDeviceInd->remote_BD,
                                    pCOM_DIDDeviceInd->remote_VendorID,
                                    pCOM_DIDDeviceInd->remote_ProductID,
                                    pCOM_DIDDeviceInd->remote_Version,
                                    pCOM_DIDDeviceInd->remote_Device_Name
                                    );
   }

   if (ret == FALSE)
   {  if (pBTLtp->pBufferAction)
      {  pBTLtp->pBufferAction->Action = btltpActionNotUsed;
         pBTLtp->pBufferAction         = NULL;
      }

      blueAPI_DIDDeviceConf(//pBuffer,
                            //pBTLtp->blueAPIHandle,
                            pCOM_DIDDeviceInd->serviceHandle,
                            blueAPI_CauseResourceError
                            );
   }
   return FALSE;
}

STATIC void LTPHandle_AuthListInfo(PBTLtp pBTLtp,PBlueAPI_AuthListInfo pCOM_AuthListInfo)
{
  uint8_t pOpt[1];

  pOpt[0] = pCOM_AuthListInfo->remote_BD_Type;

  LTPLibSendAuthListInfo(&pBTLtp->LTPLib,
                         (BTLTP_DEFAULT_COPMSK |
                          LTP_AUTH_LIST_INFO_OPT_MASK_BD_TYPE
                         ),
                         pOpt,
                         pCOM_AuthListInfo->remote_BD,
                         pCOM_AuthListInfo->keyType,
                         pCOM_AuthListInfo->AppData,
                         pCOM_AuthListInfo->Remote_DeviceName
                         );
}

STATIC void LTPHandle_AuthListRsp(PBTLtp pBTLtp,PBlueAPI_AuthListRsp pCOM_AuthListRsp)
{
  uint8_t pOpt[1];

  pOpt[0] = pCOM_AuthListRsp->remote_BD_Type;

  LTPLibSendAuthListRsp(&pBTLtp->LTPLib,
                        (BTLTP_DEFAULT_COPMSK |
                         LTP_AUTH_LIST_RSP_OPT_MASK_BD_TYPE
                        ),
                        pOpt,
                        BTLTPConvertCOMtoLTPcause(pCOM_AuthListRsp->cause),
                        pCOM_AuthListRsp->remote_BD
                        );
}

STATIC void LTPHandle_AuthDeleteRsp(PBTLtp pBTLtp,PBlueAPI_AuthDeleteRsp pCOM_AuthDeleteRsp)
{
  uint8_t pOpt[1];

  pOpt[0] = pCOM_AuthDeleteRsp->remote_BD_Type;

  LTPLibSendAuthDeleteRsp(&pBTLtp->LTPLib,
                          (BTLTP_DEFAULT_COPMSK |
                           LTP_AUTH_DELETE_RSP_OPT_MASK_BD_TYPE
                          ),
                          pOpt,
                          BTLTPConvertCOMtoLTPcause(pCOM_AuthDeleteRsp->cause),
                          pCOM_AuthDeleteRsp->remote_BD
                          );
}

STATIC void LTPHandle_AuthRsp(PBTLtp pBTLtp,PBlueAPI_AuthRsp pCOM_AuthRsp)
{
   LTPLibSendAuthRsp(&pBTLtp->LTPLib,
                     BTLTP_DEFAULT_COPMSK,
                     NULL,
                     BTLTPConvertCOMtoLTPcause(pCOM_AuthRsp->cause),
                     pCOM_AuthRsp->remote_BD
                    );
}

STATIC void LTPHandle_UserAuthorizationReqInd(PBTLtp pBTLtp,PBlueAPI_UserAuthorizationReqInd pCOM_UserAuthorizationReqInd)
{
   LTPLibSendAuthorizationReqInd(&pBTLtp->LTPLib,
                                 BTLTP_DEFAULT_COPMSK,
                                 NULL,
                                 pCOM_UserAuthorizationReqInd->remote_BD
                                );
}

STATIC void LTPHandle_UserConfirmationReqInd(PBTLtp pBTLtp,PBlueAPI_UserConfirmationReqInd pCOM_UserConfirmationReqInd)
{
   LTPLibSendUserConfRequestInd(&pBTLtp->LTPLib,
                                BTLTP_DEFAULT_COPMSK,
                                NULL,
                                pCOM_UserConfirmationReqInd->remote_BD,
                                pCOM_UserConfirmationReqInd->displayValue
                               );
}

STATIC void LTPHandle_KeypressNotificationRsp(PBTLtp pBTLtp,PBlueAPI_KeypressNotificationRsp pCOM_KeypressNotificationRsp)
{
   LTPLibSendKeypressNotificationRsp(&pBTLtp->LTPLib,
                                     BTLTP_DEFAULT_COPMSK,
                                     NULL,
                                     BTLTPConvertCOMtoLTPcause(pCOM_KeypressNotificationRsp->cause)
                                    );
}

STATIC void LTPHandle_KeypressNotificationInfo(PBTLtp pBTLtp,PBlueAPI_KeypressNotificationInfo pCOM_KeypressNotificationInfo)
{
   LTPLibSendKeypressNotificationInfo(&pBTLtp->LTPLib,
                                      BTLTP_DEFAULT_COPMSK,
                                      NULL,
                                      pCOM_KeypressNotificationInfo->remote_BD,
                                      pCOM_KeypressNotificationInfo->eventType
                                     );
}


STATIC void LTPHandle_LegacyRemoteOOBDataReqInd(PBTLtp pBTLtp,PBlueAPI_LegacyRemoteOOBDataReqInd pCOM_RemoteOOBDataReqInd)
{
   LTPLibSendLegacyRemoteOOBRequestInd(&pBTLtp->LTPLib,
                                 BTLTP_DEFAULT_COPMSK,
                                 NULL,
                                 pCOM_RemoteOOBDataReqInd->remote_BD
                                );
}

STATIC void LTPHandle_LocalOOBDataRsp(PBTLtp pBTLtp,PBlueAPI_LocalOOBDataRsp pCOM_LocalOOBDataRsp)
{
   LTPLibSendLocalOOBRsp(&pBTLtp->LTPLib,
                         BTLTP_DEFAULT_COPMSK,
                         NULL,
                         BTLTPConvertCOMtoLTPcause(pCOM_LocalOOBDataRsp->cause),
                         pCOM_LocalOOBDataRsp->C,
                         pCOM_LocalOOBDataRsp->R
                        );
}

STATIC void LTPHandle_DeviceNameRsp(PBTLtp pBTLtp,PBlueAPI_DeviceNameRsp pCOM_DeviceNameRsp)
{
   LTPLibSendDeviceNameRsp(&pBTLtp->LTPLib,
                           BTLTP_DEFAULT_COPMSK,
                           NULL,
                           BTLTPConvertCOMtoLTPcause(pCOM_DeviceNameRsp->cause),
                           pCOM_DeviceNameRsp->remote_BD,
                           pCOM_DeviceNameRsp->remote_Device_Name
                          );
}

static void LTPHandle_GATTSDPDiscoveryInd(PBTLtp pBTLtp,
                                            PBlueAPI_GATTSDPDiscoveryInd pSDPDiscoveryInd)
{
  	LTPLibSendGATTSDPDiscoveryInfo(&pBTLtp->LTPLib,
                           BTLTP_DEFAULT_COPMSK,
                           NULL,
                           pSDPDiscoveryInd->remote_BD,
                           pSDPDiscoveryInd->remote_GATT_UUID,
                           pSDPDiscoveryInd->remote_GATT_StartHandle,
                           pSDPDiscoveryInd->remote_GATT_EndHandle);
  	blueAPI_GATTSDPDiscoveryConf( pSDPDiscoveryInd->serviceHandle,
                               blueAPI_CauseSuccess
                               );
}

static void LTPHandle_GATTSDPDiscoveryRsp(PBTLtp pBTLtp,
                                            PBlueAPI_GATTSDPDiscoveryRsp pSDPDiscoveryRsp)
{          
      LTPLibSendGATTSDPDiscoveryRsp(&pBTLtp->LTPLib,
                             BTLTP_DEFAULT_COPMSK,
                             NULL,
                             BTLTPConvertCOMtoLTPcause(pSDPDiscoveryRsp->cause)
                            );
}

STATIC void LTPHandle_ACLConfigRsp(PBTLtp pBTLtp, PBlueAPI_ACLConfigRsp pACLConfigRsp)
{
  switch (pACLConfigRsp->opCode)
  {
    case blueAPI_ACLConfigLinkpolicy: /*------------------------------------*/
      LTPLibSendACLConfigLinkpolicyRsp(&pBTLtp->LTPLib,
                                       BTLTP_DEFAULT_COPMSK,
                                       NULL,
                                       pACLConfigRsp->cause,
                                       pACLConfigRsp->remote_BD
                                       );
      break;
    case blueAPI_ACLConfigSniffmode: /*-------------------------------------*/
      LTPLibSendACLConfigSniffmodeRsp(&pBTLtp->LTPLib,
                                      BTLTP_DEFAULT_COPMSK,
                                      NULL,
                                      pACLConfigRsp->cause,
                                      pACLConfigRsp->remote_BD
                                      );
      break;
#if (F_BT_HCI_RADIO_STATUS_CONF)
    case blueAPI_ACLConfigLinkstatus: /*------------------------------------*/
      LTPLibSendACLConfigLinkstatusRsp(&pBTLtp->LTPLib,
                                       BTLTP_DEFAULT_COPMSK,
                                       NULL,
                                       pACLConfigRsp->cause,
                                       pACLConfigRsp->remote_BD,
                                       pACLConfigRsp->remote_BD_Type
                                       );
      break;
#endif /* (F_BT_HCI_RADIO_STATUS_CONF) */
    default: /*-------------------------------------------------------------*/
      break;
  }
}

void LTPHandle_BREDR_DeviceConfigSetRsp(PBTLtp pBTLtp, PBlueAPI_DeviceConfigSetRsp pCOM_DeviceConfigSetRsp)
{
    switch (pCOM_DeviceConfigSetRsp->opCode)
    {
    case blueAPI_DeviceConfigDevice:
        LTPLibSendDeviceConfigDeviceSetRsp(&pBTLtp->LTPLib,
                                         BTLTP_DEFAULT_COPMSK,
                                         NULL,
                                         pCOM_DeviceConfigSetRsp->cause
                                         );
        break;

    case blueAPI_DeviceConfigDID: /*----------------------------------------*/
      LTPLibSendDeviceConfigDIDSetRsp(&pBTLtp->LTPLib,
                                      BTLTP_DEFAULT_COPMSK,
                                      NULL,
                                      pCOM_DeviceConfigSetRsp->cause
                                      );
      break;

#if (0)
    case blueAPI_DeviceConfigSPP: /*----------------------------------------*/
      LTPLibSendDeviceConfigSPPSetRsp(&pBTLtp->LTPLib,
                                      BTLTP_DEFAULT_COPMSK,
                                      NULL,
                                      pCOM_DeviceConfigSetRsp->cause
                                      );
      break;
#endif

    case blueAPI_DeviceConfigPagescan: /*-----------------------------------*/
      LTPLibSendDeviceConfigPagescanSetRsp(&pBTLtp->LTPLib,
                                           BTLTP_DEFAULT_COPMSK,
                                           NULL,
                                           pCOM_DeviceConfigSetRsp->cause
                                           );
      break;

    case blueAPI_DeviceConfigLinkpolicy: /*---------------------------------*/
      LTPLibSendDeviceConfigLinkpolicySetRsp(&pBTLtp->LTPLib,
                                             BTLTP_DEFAULT_COPMSK,
                                             NULL,
                                             pCOM_DeviceConfigSetRsp->cause
                                             );
      break;

     default: /*-------------------------------------------------------------*/
        break;
    }
}

void BTLTPHandleBREDRBLUE_API_MSG(PBTLtp pBTLtp, uint8_t * pBuffer, uint16_t offset)
{
    PBlueAPI_UsMessage pCOMMsg       = (PBlueAPI_UsMessage)(pBuffer + offset);

    switch (pCOMMsg->Command)
    {
#if 0
      case blueAPI_EventDataRsp: /*---------------------------------------*/
         LTPHandle_DataRsp(pBTLtp, pBuffer, &pCOMMsg->p.DataRsp);
         break;
      case blueAPI_EventDataInd: /*---------------------------------------*/
         LTPHandle_DataInd(pBTLtp, pBuffer, &pCOMMsg->p.DataInd);
         break;
	case blueAPI_EventCreateLegacyMDLInd: /*----------------------------------*/
         LTPHandle_CreateLegacyMDLInd(pBTLtp,&pCOMMsg->p.CreateLegacyMDLInd);
         break;
      case blueAPI_EventRegSPPMDEPRsp: /*---------------------------------*/
         LTPHandle_RegSPPMDEPRsp(pBTLtp,&pCOMMsg->p.RegSPPMDEPRsp);
         break;
      case blueAPI_EventRelMDEPRsp: /*------------------------------------*/
         LTPHandle_RelMDEPRsp(pBTLtp,&pCOMMsg->p.RelMDEPRsp);
         break;
#endif
      case blueAPI_EventUserAuthRequestInd: /*----------------------------*/
         LTPHandle_UserAuthRequestInd(pBTLtp,&pCOMMsg->p.UserAuthRequestInd);
         break;
      case blueAPI_EventRadioModeSetRsp: /*-------------------------------*/
         LTPHandle_RadioModeSetRsp(pBTLtp,&pCOMMsg->p.RadioModeSetRsp);
         break;
      case blueAPI_EventSDPDiscoveryRsp: /*-------------------------------*/
         LTPHandle_SPPDiscoveryRsp(pBTLtp,&pCOMMsg->p.SDPDiscoveryRsp);
         break;
      case blueAPI_EventSDPEndpointInd: /*--------------------------------*/
         LTPHandle_SPPEndpointInd(pBTLtp, pBuffer, &pCOMMsg->p.SDPEndpointInd);
         break;
      case blueAPI_EventInquiryRsp: /*------------------------------------*/
         LTPHandle_InquiryRsp(pBTLtp,&pCOMMsg->p.InquiryRsp);
         break;
      case blueAPI_EventInquiryDeviceInfo: /*-----------------------------*/
         LTPHandle_InquiryDeviceInfo(pBTLtp,&pCOMMsg->p.InquiryDeviceInfo);
         break;	 
      case blueAPI_EventDIDDeviceInd: /*----------------------------------*/
         LTPHandle_DIDDeviceInd(pBTLtp, pBuffer, &pCOMMsg->p.DIDDeviceInd);
         break;
      case blueAPI_EventAuthListInfo: /*----------------------------------*/
         LTPHandle_AuthListInfo(pBTLtp,&pCOMMsg->p.AuthListInfo);
         break;
      case blueAPI_EventAuthListRsp: /*-----------------------------------*/
         LTPHandle_AuthListRsp(pBTLtp,&pCOMMsg->p.AuthListRsp);
         break;
      case blueAPI_EventAuthDeleteRsp: /*---------------------------------*/
         LTPHandle_AuthDeleteRsp(pBTLtp,&pCOMMsg->p.AuthDeleteRsp);
         break;
      case blueAPI_EventAuthRsp: /*---------------------------------------*/
         LTPHandle_AuthRsp(pBTLtp,&pCOMMsg->p.AuthRsp);
         break;
      case blueAPI_EventUserAuthorizationReqInd: /*-----------------------*/
         LTPHandle_UserAuthorizationReqInd(pBTLtp,&pCOMMsg->p.UserAuthorizationReqInd);
         break;
      case blueAPI_EventUserConfirmationReqInd: /*------------------------*/
         LTPHandle_UserConfirmationReqInd(pBTLtp,&pCOMMsg->p.UserConfirmationReqInd);
         break;
      case blueAPI_EventKeypressNotificationRsp: /*-----------------------*/
         LTPHandle_KeypressNotificationRsp(pBTLtp,&pCOMMsg->p.KeypressNotificationRsp);
         break;
      case blueAPI_EventKeypressNotificationInfo: /*----------------------*/
         LTPHandle_KeypressNotificationInfo(pBTLtp,&pCOMMsg->p.KeypressNotificationInfo);
         break;
      case blueAPI_EventLegacyRemoteOOBDataReqInd: /*---------------------------*/
         LTPHandle_LegacyRemoteOOBDataReqInd(pBTLtp,&pCOMMsg->p.LegacyRemoteOOBDataReqInd);
         break;
      case blueAPI_EventLocalOOBDataRsp: /*-------------------------------*/
         LTPHandle_LocalOOBDataRsp(pBTLtp,&pCOMMsg->p.LocalOOBDataRsp);
         break;
      case blueAPI_EventDeviceNameRsp: /*---------------------------------*/
         LTPHandle_DeviceNameRsp(pBTLtp,&pCOMMsg->p.DeviceNameRsp);
         break;
      case blueAPI_EventACLConfigRsp: /*----------------------------------*/
         LTPHandle_ACLConfigRsp(pBTLtp, &pCOMMsg->p.ACLConfigRsp);
         break;
     case blueAPI_EventGATTSDPDiscoveryInd:
         LTPHandle_GATTSDPDiscoveryInd(pBTLtp, &pCOMMsg->p.GATTSDPDiscoveryInd);
         break;		
     case blueAPI_EventGATTSDPDiscoveryRsp:
         LTPHandle_GATTSDPDiscoveryRsp(pBTLtp, &pCOMMsg->p.GATTSDPDiscoveryRsp);
         break;	        
    default: /*---------------------------------------------------------*/
        DBG_BUFFER(MODULE_LTP, LEVEL_ERROR, "BTLTPHandleBREDRBLUE_API_MSG, unknown pCOMMsg->Command = 0x%x", 1, pCOMMsg->Command);
        break;
    }

    return;
}

