/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       blueapi_bface.c
* @brief     blueface config api
* @details   
*
* @author   	gordon
* @date      	2015-06-26
* @version	v0.1
*/

#include <flags.h>
#include <rtl_types.h>
#include <blueface.h>
#include <btglbdef.h>
#include <bt_api.h>
#include <sdp_code.h>
#include <sdplib.h>
#include <blueapi_types.h>
#include <blueapi_osif.h>
#include <blueapi_def.h>
#include <blueapi_api.h>
#include <os_message.h>
#include <os_pool.h>
#include <btman.h>
#include <btsend.h>
#include <btsm_api.h>
#include <hci_api.h>
#include <l2c_api.h>
#include <sdp_api.h>
#include <gatt_api.h>
#include <btglib.h>
#include <hci_code.h>
#include <blueface.h>
#include <upper_stack_global.h>

/* File Id for COM_InternalEventInfo */
#define BLUE_API_SOURCE_FILE_ID     0x02
#define TRACE_MODULE_ID     MID_BT_GATT

/**
* @brief  send set inquiry scan LAP
*
* @param  pBlueAPIdata:
* @param  limitedDiscoverable: limited discoverable
*
* @return  
*
*/
void blueAPI_Send_BLUEFACE_CONF_INQUIRY_SCAN_IACS(PBlueAPI_Data pBlueAPIdata,
                                                         BOOL limitedDiscoverable
                                                         )
{
	uint8_t          IACS[3];
    uint32_t classOfDevice = pBlueAPIdata->ConfigParameter.ClassOfDevice;

	IACS[0] = 1;
	IACS[1] = IAC_GIAC_IDX;

	if (limitedDiscoverable)
	{
		IACS[0] = 2;
		IACS[2] = IAC_LIAC_IDX;
	}

	hciCommandWriteCurrentIACs(IACS[0]);

    if (pBlueAPIdata->ConfigParameter.limitedDiscovery)
    {
        classOfDevice |= HCI_SERVICE_CLASS_LIMITED_DISCOVERABLE_MODE;
    }

	hciHandleUpWriteClassOfDevice((uint8_t *)&classOfDevice);

	blueAPI_Send_RadioModeSetRsp(blueAPI_CauseSuccess);
} /* blueAPI_Send_BLUEFACE_CONF_INQUIRY_SCAN_IACS */

/**
* @brief  blueapi send read local oob data command
*
* @param  pBlueAPIdata: 
*
* @return  
*
*/
void blueAPI_Send_BLUEFACE_CONF_READ_LOCAL_OOB_DATA(void)
{  
	hciCommandNoParameter(HCI_READ_LOCAL_OOB_DATA); 
//	hciLaterEntry();
} /* blueAPI_Send_BLUEFACE_CONF_READ_LOCAL_OOB_DATA */

/**
* @brief  blueapid send blueface config LE ssp param
*		set LE_fixedDisplayValue value		
*
* @param  pBlueAPIdata: 
* @param  fixedDisplayValue
*
* @return  
*
*/
void blueAPI_Send_BLUEFACE_CONF_LE_SSP_PARAMETER(PBlueAPI_Data  pBlueAPIdata,
                                                 uint32_t          fixedDisplayValue
                                                 )
{
    TSEC_CONF_LE_SSP_PARAMETER  leSSPParameter;
	uint16_t cause =HCI_SUCCESS;

    leSSPParameter.fixedDisplayValue = fixedDisplayValue;
    
	cause = btsmHandleConfigurationReq(BLUEFACE_CONF_LE_SSP_PARAMETER,
                   (uint8_t *)&leSSPParameter, FALSE);

	if ((pBlueAPIdata->SM_SEC_DS_CommandMsg != NULL) &&
	  (pBlueAPIdata->SM_SEC_DS_CommandMsg->Command == blueAPI_EventDeviceConfigSetReq)
	 )
	{
		TBlueAPI_Cause param_cause = (cause == HCI_SUCCESS) ? blueAPI_CauseSuccess : blueAPI_CauseInvalidParameter; 

		osBufferRelease(pBlueAPIdata->SM_SEC_DS_CommandMsg);
		pBlueAPIdata->SM_SEC_DS_CommandMsg = NULL;
		blueAPI_Send_DeviceConfigSetRsp(
		                                pBlueAPIdata->SM_SEC_DS_CommandMsg,
		                                blueAPI_DeviceConfigSecurity,
		                                param_cause
		                                );
	}
} /* blueAPI_Send_BLUEFACE_CONF_LE_SSP_PARAMETER */

/**
* @brief  blueapid send sniff subrating config
*
* @param  pBlueAPIdata: 
* @param  remote_BD
* @param  maxLatency
* @param  minRemoteTimeout
* @param  minLocalTimeout
*
* @return  
*
*/
void blueAPI_Send_BLUEFACE_CONF_SNIFF_SUBRATING(PBlueAPI_Data pBlueAPIdata,
                                                       uint8_t *        remote_BD,
                                                       uint16_t          maxLatency,
                                                       uint16_t          minRemoteTimeout,
                                                       uint16_t          minLocalTimeout)
{
	hciHandleConfSniffSubrating(remote_BD, maxLatency, minRemoteTimeout, minLocalTimeout);
} /* blueAPI_Send_BLUEFACE_CONF_SNIFF_SUBRATING */
/* (F_BT_SNIFF) */

/**
* @brief  blueapid send sdp search request
*
* @param  pBlueAPIdata: 
* @param  uuid1
* @param  uuid2
*
* @return  
*
*/
void blueAPI_Send_BT_SDP_SEARCH_REQ(PBlueAPI_Data pBlueAPIdata, uint16_t uuid1, uint16_t uuid2)
{
    uint8_t buffer1[8];
    uint16_t uuidLen;

    if (uuid2 == 0)
    {
        uuidLen = (uint16_t)sdpCreateDes(buffer1, "<U>", uuid1);
    }
    else
    {
        uuidLen = (uint16_t)sdpCreateDes(buffer1, "<U U>", uuid1, uuid2);
    }

    btApiBT_SDP_SEARCH_REQ(pBlueAPIdata->pSDPLinkContext->handle, BLUE_API_SDP_SERVICE_HANDLE_COUNT,
        uuidLen, (uint8_t *)&buffer1);
}

/**
 * @brief  send sdp attribute request
 * 
 * @param  pBlueAPIdata
 * @param  serviceHandle
 * @param  attributeRange
 * @param  maxLen
 *
 * @return  
 *
 */
void blueAPI_Send_BT_SDP_ATTRIBUTE_REQ(PBlueAPI_Data pBlueAPIdata, uint32_t serviceHandle,
    uint32_t attributeRange, uint16_t maxLen)
{
    uint8_t buffer1[8];
    uint16_t aLen;

    aLen = (uint16_t)sdpCreateDes(buffer1, "<L>", attributeRange);

    btApiBT_SDP_ATTRIBUTE_REQ(pBlueAPIdata->pSDPLinkContext->handle, maxLen, serviceHandle, aLen, buffer1);
}

void blueAPI_Handle_GATT_SERVICE_REGISTER_CONF(uint16_t wCause, 
    PVOID serviceHandle)
{
    blueAPI_Send_GATTServiceRegisterRsp(serviceHandle, wCause);
}

void blueAPI_Handle_BT_GATT_LE_CONNECTION_UPDATE_IND(uint16_t cid,
    uint16_t type, PGATTLEConnectionUpdateParam pParam)
{
    PBlueAPI_LinkDescriptor pLinkContext;
    LPbtLink pLink;

    pLink = blueFaceFindAndCheckLink(pBTA, cid, NULL);
    if (pLink == NULL)
    {
        assert(FALSE);
        return;
    }
    pLinkContext  = (PBlueAPI_LinkDescriptor)(pLink->context);


    if (type == GATT_LE_CONNECTION_UPDATE_TYPE_REQUEST)
    {
        blueAPI_Send_LEConnectionUpdateInd(pLinkContext->pMDL->local_MDL_ID, (TGATT_LE_CONNECTION_UPDATE_PARAM *)pParam);
    }
    else
    {
        pLinkContext->LEConnectionInterval  = pParam->connIntervalMin;
        pLinkContext->LEConnectionLatency   = pParam->connLatency;
        pLinkContext->LESupervisionTimeout  = pParam->supervisionTimeout;

        blueAPI_Send_LEConnectionParameterInfo( pLinkContext);
    }
}

uint16_t blueAPI_Check_LE_Link(uint16_t cid)
{
    PBlueAPI_LinkDescriptor  pLinkContext;
    LPbtLink     pLink;
    
    pLink = blueFaceFindAndCheckLink(pBTA, cid, NULL);
#if CHECK_API_PARAM
    if (pLink==NULL)
    {
        return 0;
    }
#endif
    pLinkContext = (PBlueAPI_LinkDescriptor)(pLink->context);	//randy--610

    if ((pLinkContext->linkState == blueAPI_Connected) && pLinkContext->MDLConnected)
    {
        return pLinkContext->pMDL->local_MDL_ID;
    } /* else ignore, GATT layer cleans up ... */
    return 0;
}

#if F_BT_LE_BT41_SUPPORT
uint16_t blueAPI_GetMdlID(uint16_t handle)
{
    PBlueAPI_LinkDescriptor  pLinkContext;
    LPbtLink     pLink;
    uint16_t cid = gattGetLEConnectionCid(handle);
    pLink = blueFaceFindAndCheckLink(pBTA, cid, NULL);
#if CHECK_API_PARAM
    if (pLink==NULL)
    {
        return 0;
    }
#endif
    pLinkContext = (PBlueAPI_LinkDescriptor)(pLink->context);	//randy--610

    if ((pLinkContext->linkState == blueAPI_Connected) && pLinkContext->MDLConnected)
    {
        return pLinkContext->pMDL->local_MDL_ID;
    } /* else ignore, GATT layer cleans up ... */
    return 0;
}
#endif

void blueAPI_Handle_GATT_CON_CONF( uint16_t status, uint16_t cid)
{
	LPbtLink pLink		= pBTA->activeLink;

	BLUEFACE_TRACE_PRINTF_2(BLUEFACE_TRACE_MASK_TRACE,"BTMAN: CON_CONF cid %X status %X ",
				   cid, status);

	pLink->handle		 = cid;

	blueFaceReleaseActiveLinkSemaphore(); //	  pBTA->activeLink = NULL;

	if (status)
	{
		/* conConf with error : show disc ind to app and terminate */
		if (pLink->app==NULL)
		{
			/* there is no application to control this link: kill everything immediately */
			blueFaceDeallocateLink(pLink);
		} /* no app */
		else
		{
			blueFaceSendBT_DISC_IND(pLink->context, status);
		}
	}
}

void blueAPI_Handle_SECURITY_STATUS(uint8_t indId, uint8_t * bd, uint8_t bdType, uint8_t keyType, uint8_t keySize)
{
    if (pBTA->secApp)
	{
		TBlueAPI_ACLStatusParam    aclStatusParam;

		aclStatusParam.auth.keySize = keySize;
		aclStatusParam.auth.keyType = (TBlueAPI_KeyType)keyType;

		switch (indId)
		{
#if (BLUEAPI_ENABLE_ACL_INFO & BLUE_API_ENABLE_ACL_INFO_AUTH_STARTED)
		case CONF_SECURITY_STATUS_AUTH_STARTED:
			blueAPI_Send_ACLStatusInfo(bd,
									   (TBlueAPI_RemoteBDType)bdType,
									   blueAPI_ACLAuthenticationStarted,
									   &aclStatusParam);
			break;
#endif
#if (BLUEAPI_ENABLE_ACL_INFO & BLUE_API_ENABLE_ACL_INFO_AUTH_SUCCESS)
		case CONF_SECURITY_STATUS_AUTH_SUCCESS:
			blueAPI_Send_ACLStatusInfo(bd,
									   (TBlueAPI_RemoteBDType)bdType,
									   blueAPI_ACLAuthenticationSuccess,
									   &aclStatusParam);
			break;
#endif
#if (BLUEAPI_ENABLE_ACL_INFO & BLUE_API_ENABLE_ACL_INFO_AUTH_FAILED)
		case CONF_SECURITY_STATUS_AUTH_FAILED:
			blueAPI_Send_ACLStatusInfo(bd,
									   (TBlueAPI_RemoteBDType)bdType,
									   blueAPI_ACLAuthenticationFailure,
									   &aclStatusParam);
			break;
#endif
#if (BLUEAPI_ENABLE_ACL_INFO & BLUE_API_ENABLE_ACL_INFO_CONNECTION_ENCRYPTED)
		case CONF_SECURITY_STATUS_ENCRYPTED:
			blueAPI_Send_ACLStatusInfo(bd,
									   (TBlueAPI_RemoteBDType)bdType,
									   blueAPI_ACLConnectionEncrypted,
									   &aclStatusParam);
			break;
		case CONF_SECURITY_STATUS_NOT_ENCRYPTED:
			blueAPI_Send_ACLStatusInfo(bd,
									   (TBlueAPI_RemoteBDType)bdType,
									   blueAPI_ACLConnectionNotEncrypted,
									   &aclStatusParam);
			break;
#endif
		default:
			break;
		}
	}
}

void blueAPI_Handle_BTSM_RESOLVED_ADDRESS(uint8_t * bd, uint8_t bdType,
    uint8_t * resolvedBd, uint8_t resolvedBdType)
{
    if (pBTA->secApp)
    {
        TBlueAPI_ACLStatusParam aclStatusParam;
        PBlueAPI_MCL pMCL = blueAPI_MCLFindByBD(bd, blueAPI_MCLType_GATT);

        if (pMCL != NULL)
        {
            PBlueAPI_LinkDescriptor pLinkContext = blueAPI_MCLFindLinkByState(pMCL, blueAPI_Connected);
            if ((pLinkContext != NULL) && (pLinkContext->pMDL != NULL))
            {
                PBlueAPI_MDL pMDL = pLinkContext->pMDL;

                memcpy(pMCL->bd, resolvedBd, BD_ADDR_SIZE);
                pMCL->bdType = (TBlueAPI_RemoteBDType)resolvedBdType;
                memcpy(pMDL->remoteBd, resolvedBd, BD_ADDR_SIZE);
                pMDL->remoteBdType = (TBlueAPI_RemoteBDType)resolvedBdType;
            }
        }

        memcpy(aclStatusParam.resolve.remote_BD, resolvedBd, BD_ADDR_SIZE);
        aclStatusParam.resolve.remote_BD_type = (TBlueAPI_RemoteBDType)resolvedBdType;

        blueAPI_Send_ACLStatusInfo(bd, (TBlueAPI_RemoteBDType)bdType, blueAPI_ACLAddressResolved,
            &aclStatusParam);
    }
}

void blueAPI_Handle_BT_SECMAN_AUTH_CONF(uint8_t * bd, uint8_t keyType, uint8_t keySize, uint16_t cause)
{
    PBlueAPI_MCL pMCL;
    PBlueAPI_LinkDescriptor pLinkContext;
    TBlueAPI_Cause param_cause;

    pMCL = blueAPI_MCLFindByBD((uint8_t *)bd, blueAPI_MCLType_GATT);

    switch (cause)
    {
    case HCI_SUCCESS:
        param_cause = blueAPI_CauseSuccess;
        break;

    case (HCI_ERR | HCI_ERR_PARING_NOT_ALLOWED):
        param_cause = blueAPI_CauseReject;
        break;

    case (HCI_ERR | HCI_ERR_AUTHENTICATION_FAILED):
    case (HCI_ERR | HCI_ERR_KEY_MISSING):
    case (HCI_ERR | HCI_ERR_INSUFFICIENT_SECURITY):
        param_cause = blueAPI_CauseAuthenticationFailed;
        break;

    default:
        param_cause = blueAPI_CauseUnspecified;
        break;
    }

    if (pMCL != NULL)
    {
        pLinkContext = blueAPI_MCLFindLinkByState(pMCL, blueAPI_Connected);
        if (pLinkContext != NULL)
        {
            blueAPI_Send_GATTSecurityRsp(pLinkContext->pMDL->local_MDL_ID, (TBlueAPI_KeyType)keyType,
                keySize, param_cause);
        }
    }
}

void blueAPI_Handle_NewKeyInd(uint8_t * bd, uint8_t * key, uint8_t keyType)
{
    LPbtLink pLink;

    BLUEFACE_TRACE_PRINTF_1(BLUEFACE_TRACE_MASK_TRACE,"BTMAN: NEW_KEY_IND bd %s", TRACE_BDADDR1(BLUEFACE_TRACE_MASK_TRACE, bd));

    pLink = blueFaceFindLinkByBD(bd);
    if (pLink == NULL || (pBTA->secApp && pBTA->secApp->security))
    {
        if (pBTA->secApp == NULL)
        {
            BLUEFACE_TRACE_PRINTF_1(BLUEFACE_TRACE_MASK_ERROR,"!!BTMAN: NEW_KEY_IND bd %s -- no link found", TRACE_BDADDR1(BLUEFACE_TRACE_MASK_ERROR, bd));
        }
        else
        {
            /* no link, but we got a control app, let it handly by that one */
            blueFaceSendBT_HCI_NEWKEY_IND(NULL, bd, key, keyType);
        }
    }
    else
    {
        /* we have found a corresponding link, send the message upstream */
        blueFaceSendBT_HCI_NEWKEY_IND(pLink->context, bd, key, keyType);
    }
} /* btsmSendNewKeyInd */

void blueAPI_Handle_BLUEFACE_CONF_AUTHORIZE(LPTSECLINK link)
{
    if (pBTA->secApp)
    {
        if (link->auth_source == SECMAN_SOURCE_RFCOMM)
        {
            blueAPI_Send_UserAuthorizationReqInd(link->bd, link->auth_outg, PSM_RFCOMM, (link->auth_cid >> 1), link->auth_uuid);
        }
        else
        {
            blueAPI_Send_UserAuthorizationReqInd(link->bd, link->auth_outg, link->auth_cid, 0, link->auth_uuid);
        }
    }
} /* btsmSendBLUEFACE_CONF_AUTHORIZE */

void blueAPI_Handle_AuthConf(uint8_t id, uint16_t status)
{
	LPbtLink	   pLink;

    if(id < 0x10)
    {
	    pLink	 = &pBTA->pLinksDon[id];
    }
    else
    {
        pLink	 = &pBTA->pLinksDoff[id - 0x10];
    }

	BLUEFACE_TRACE_PRINTF_2(BLUEFACE_TRACE_MASK_TRACE, "BTA: AUTH_CONF id %x status %x",
		id, status);

	pLink->activeAuth = FALSE;	   /* authentication is not active any more */

	blueFaceSendBT_HCI_AUTH_CONF(status);
}

void blueAPI_Handle_HciKeyInd(uint8_t * bd, uint8_t reqType)
{
    LPbtLink lpLink;
    BLUEFACE_TRACE_PRINTF_2(BLUEFACE_TRACE_MASK_TRACE,"BTMAN: KEY_IND req %X bd %s", reqType, TRACE_BDADDR1(BLUEFACE_TRACE_MASK_TRACE, bd));

    lpLink = blueFaceFindLinkByBD(bd);
    if (lpLink == NULL || (pBTA->secApp && pBTA->secApp->security))
    {
        /* wither no link for this key ind or an expicit security manager exists: probably security mode 3
                (device level security) send the message to the security manager ! (there should be one!)
                */
        if (pBTA->secApp == NULL)
        {
            BLUEFACE_TRACE_PRINTF_2(BLUEFACE_TRACE_MASK_ERROR,"!!BTMAN: KEY_IND req %X bd %s -- no link / no app found", reqType, TRACE_BDADDR1(BLUEFACE_TRACE_MASK_ERROR, bd));
            btsmHandleBT_HCI_KEY_RESP(bd, reqType, 1 /* negative */, NULL, 0);
            return;
        } /* secApp */

        /* we have no link, but a control app -> send it the request with
                NULL context to indicate that we have NO LINK
                */
        if (reqType == LINK_KEY_REQUEST)
        {
            reqType = BLUEFACE_LINKKEY;
        }
        else
        {
            if (lpLink != NULL && lpLink->activeAuth)
            {
                reqType = BLUEFACE_LOCALPIN;
            }
            else
            {
                reqType = BLUEFACE_REMOTEPIN;
            }
        }

        if (lpLink && (lpLink->app == pBTA->secApp))
        {
            blueFaceSendBT_HCI_KEY_IND(reqType, bd);
        }
        else
        {
            blueFaceSendBT_HCI_KEY_IND(reqType, bd);
        }

        return;
    }

    /* we have found a corresponding link, send the message upstream */
    if (reqType == LINK_KEY_REQUEST)
    {
        reqType = BLUEFACE_LINKKEY;
    }
    else if (lpLink->activeAuth)
    {
        reqType = BLUEFACE_LOCALPIN;
    }
    else
    {
        reqType = BLUEFACE_REMOTEPIN;
    }
    blueFaceSendBT_HCI_KEY_IND(reqType, bd);
}

void blueAPI_Handle_HCI_LINK_KEY_NOTIFICATION(uint8_t * bd, uint8_t * key, uint8_t keyType)
{
    LPbtLink pLink;
    BLUEFACE_TRACE_PRINTF_1(BLUEFACE_TRACE_MASK_TRACE, "BTMAN: NEW_KEY_IND bd %s", TRACE_BDADDR1(BLUEFACE_TRACE_MASK_TRACE, bd));

    /* XXX ore: mapping HCI_KEYTYPE_* => BLUEFACE_KEYTYPE_* currently a NOP */
    pLink = blueFaceFindLinkByBD(bd);
    if (pLink == NULL || (pBTA->secApp && pBTA->secApp->security))
    {
        if (pBTA->secApp==NULL)
        {
            BLUEFACE_TRACE_PRINTF_1(BLUEFACE_TRACE_MASK_ERROR, "!!BTMAN: NEW_KEY_IND bd %s -- no link found", TRACE_BDADDR1(BLUEFACE_TRACE_MASK_ERROR, bd));
        }
        else
        {
            /* no link, but we got a control app, let it handly by that one */
            blueFaceSendBT_HCI_NEWKEY_IND(NULL, bd, key, keyType);
        }
    }
    else
    {
        /* we have found a corresponding link, send the message upstream */
        blueFaceSendBT_HCI_NEWKEY_IND(pLink->context, bd, key, keyType);
    }
}

void blueAPI_Handle_GATT_CON_ACT_IND( uint16_t cid, uint16_t wMTUSize, uint16_t * param)
{   
    LPbtLink pLink;

    pLink = blueFaceFindLinkByHandle(cid, BLUEFACE_PSM_GATT);

    if (!pLink)
    {
        BLUEFACE_TRACE_PRINTF_2(BLUEFACE_TRACE_MASK_ERROR,"!!BTMAN: CON_ACT_IND(psm %X) cannnot find link %x", BLUEFACE_PSM_GATT, cid);
        /* if pLink is not set, then the controlling application died in the meantime */
        /* Connection setup with no error, we better close the link... */
        gattHandleUpDisconnectReq(cid, HCI_ERR_OTHER_END_TERMINATE_13);
        return;
    }

    BLUEFACE_TRACE_PRINTF_4(BLUEFACE_TRACE_MASK_TRACE,"BTMAN: CON_ACT_IND(psm 0x%X) cid %X frameSize %d, status 0x%X",
    BLUEFACE_PSM_GATT, cid, wMTUSize, GATT_SUCCESS);

    pLink->state       = link_Connected;

    blueFaceSendBT_CON_ACT_IND(pLink->context, wMTUSize, DownstreamPoolID, param);

    if(pLink->encryptState == ENCRYPT_REQUIRED)
    {
        pLink->encryptState = ENCRYPT_REQUESTED;
        hciHandleUpEncryptReq((uint8_t*)pLink->bd, TRUE);
    }
}

void blueAPI_Handle_GATT_DISC_CONF(uint16_t status, uint16_t cid )
{
	LPbtLink pLink;

	BLUEFACE_TRACE_PRINTF_2(BLUEFACE_TRACE_MASK_TRACE,"BTMAN: DISC_CONF cid %x status %x",
		cid, status);

	pLink = blueFaceFindLinkByHandle(cid, BLUEFACE_PSM_GATT);
	if (pLink==NULL)
	{
		BLUEFACE_TRACE_PRINTF_2(BLUEFACE_TRACE_MASK_ERROR,"!!BTMAN: DISC_CONF cid %x cause %x -- no link found",
			cid, status);
		return;
	}

	/* disc_conf is confirmation for active disconnect: send DISC_IND upstream */
	if (pLink->app==NULL)
	{
		/* there is no application to control the link: kill everything immediately */
		BLUEFACE_TRACE_PRINTF_1(BLUEFACE_TRACE_MASK_TRACE, "BTMAN: DISC_CONF, no app for cid %X", cid);
		blueFaceDeallocateLink(pLink);
		return;
	} /* no app */

	blueFaceSendBT_DISC_IND(pLink->context, status);
}

void blueAPI_Handle_GATT_DISC_IND(uint16_t status, uint16_t cid )
{
	LPbtLink pLink;

	BLUEFACE_TRACE_PRINTF_4(BLUEFACE_TRACE_MASK_TRACE,"BTMAN: DISC_IND cid %X cause %X",
	 cid, status, 0, 0);
	pLink = blueFaceFindLinkByHandle(cid, BLUEFACE_PSM_GATT);
	if (pLink==NULL)
	{
		BLUEFACE_TRACE_PRINTF_2(BLUEFACE_TRACE_MASK_ERROR,"!!BTMAN: DISC_IND: cannot find cid %X Chan %X",
		 cid, 0);
		return;
	}

	if (pLink->app==NULL)
	{
		/* there is no application to control the link: kill everything immediately */
		BLUEFACE_TRACE_PRINTF_1(BLUEFACE_TRACE_MASK_TRACE, "BTMAN: DISC_IND, no app for cid %X", cid);
		blueFaceDeallocateLink(pLink);
		return;
	} /* no app */

	if (pLink->state == link_Incoming && !pLink->alerting)
	{
		pLink->state = link_Incoming_Disc;
	}
	else
	{
		blueFaceSendBT_DISC_IND(pLink->context, status);
	}
}

/**
* @brief sdp send upstream connect confirm
*
* @param  status: 
* @param  linkid
*
* @return  
*
*/
void blueAPI_Handle_SDP_CON_CONF(uint16_t status, uint16_t linkid)
{
    LPbtLink pLink = pBTA->activeLink;

    BLUEAPI_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_TRACE, "BTMAN: SDP_CON_CONF, linkid %x status %x", linkid, status);

    if (pLink == NULL)
    {
        /* if pLink is not set, then the controlling application died in the meantime */
        BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_ERROR, "!!BTMAN: SDP_CON_CONF, active Link not set, chan 0x%x", linkid);
        if (status == 0)
        {
            /* Connection setup with no error, we better close the link... */
            sdpHandleUpDiscReq(linkid, FALSE);
        }
        return;
    }

    blueFaceReleaseActiveLinkSemaphore(); //      pBTA->activeLink = NULL;
    if (status)
    {
        /* error in connection setup */
        blueFaceSendBT_DISC_IND(pLink->context, status);
        /* link descriptor is deallocated in disc_resp */
        return;
    }

    /* connection setup was ok */
    pLink->handle      = linkid;
    /*if disconnecting is requested during connecting, we do disconnet now*/
    if (pLink->delayedDisc)
    {
        /* Connection setup with no error, but we wanted to close it.... */
        BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE, "BTMAN: SDP_CON_CONF, executing delayed disc");
        btgSendDiscReq(blueFacePSM2hQueue(pLink->psm), pLink->handle, HCI_ERR_OTHER_END_TERMINATE_13, FALSE);
        pLink->delayedDisc = FALSE;
        pLink->state       = link_Disconnecting;
        return;
    } /* delayedDisc */
}

/**
* @brief sdp send connection active indicate
*
* @param  frameSize: 
* @param  linkid:
*
* @return  
*
*/
void blueAPI_Handle_SDP_CON_ACT_IND(uint16_t frameSize, uint16_t linkid)
{
    LPbtLink pLink = blueFaceFindLinkByHandle(linkid, BLUEFACE_PSM_SDP);

    if (!pLink)
    {
        BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_ERROR, "!!BTMAN: SDP_CON_ACT_IND, cannnot find link %x", linkid);
        return;
    }

    BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE, "BTMAN: SDP_CON_ACT_IND, cid %x", linkid);

    pLink->state       = link_Connected;

    blueFaceSendBT_CON_ACT_IND(pLink->context, frameSize, DownstreamPoolID, NULL);
}

/**
* @brief sdp send disconnect indicated
*
* @param	status: 
* @param	linkid: 
*
* @return
*
*/
void blueAPI_Handle_SDP_DISC_IND(uint16_t status, uint16_t linkid)
{
    LPbtLink pLink;

    BLUEAPI_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_TRACE, "BTMAN: SDP_DISC_IND, linkid %X status %X", linkid, status);
    pLink = blueFaceFindLinkByHandle(linkid, BLUEFACE_PSM_SDP);
    if (pLink==NULL)
    {
        BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_ERROR, "!!BTMAN: SDP_DISC_IND, cannot find linkid %X", linkid);
        return;
    }
    if (pLink->state == link_Incoming && !pLink->alerting)
	{
    	pLink->state = link_Incoming_Disc;
	}
    else
	{
    	blueFaceSendBT_DISC_IND(pLink->context, status);
	}
}

/**
* @brief  bt controller active confirm
*
* @param  bd: local bdaddr
* @param  status: status
*
* @return  
*
*/
void blueAPI_Handle_HCI_ACT_CONF(uint8_t * bd, uint16_t status)
{
    /* actConf with status 0xffff indicates local message after application registration */
    if (status != 0xffff)
    {
        BLUEFACE_TRACE_PRINTF_2(BLUEFACE_TRACE_MASK_TRACE,
                                "BTMAN: ACTIVATION complete, bd %s status %x",
                                TRACE_BDADDR1(BLUEFACE_TRACE_MASK_TRACE, bd), status
                                );

        /* copy message arguments to instance data, for later use in act_ind */
        memcpy(pBTA->localBd, bd, BD_ADDR_SIZE);
        pBTA->localCause = status;

        if (pBTA->localCause == 0)        /* Baseband initialisation ok */
        {
            /*nothing todo!*/
        }

        /* this is a valid activation of the adapter, this might be a reactivation of the adapter after a sync loss on the HCI link */
        if (pBTA->activeApp)
        {
            BLUEFACE_TRACE_PRINTF_1(BLUEFACE_TRACE_MASK_TRACE, "!!BTMAN: activeApp is set on ACT_IND %lx", pBTA->activeApp);
            pBTA->activeApp = NULL;
        }
        if (pBTA->activeLink)
        {
            BLUEFACE_TRACE_PRINTF_1(BLUEFACE_TRACE_MASK_TRACE, "!!BTMAN: activeLink is set on ACT_IND %lx", pBTA->activeLink);
            blueFaceReleaseActiveLinkSemaphore(); //      pBTA->activeLink = NULL;
        }
    } /* if actConf->cause */

    if (pBTA->localCause != 0xffff)
    {
        if (pBTA->app.used && (!pBTA->app.indicated || pBTA->localCause))
        {
            blueFaceSendBT_ACT_IND(pBTA->localBd, pBTA->localCause);
            pBTA->app.indicated = TRUE;
        }
    }
}

void blueAPI_Handle_HCI_READ_LOCAL_OOB(uint8_t * fp, uint16_t pos, uint8_t status)
{
	THCI_CONF_OOB_DATA tConfOOBData;
			
	pBTA->activeApp = NULL;
	memcpy (tConfOOBData.C , fp+pos, 16);  pos += 16;
	memcpy (tConfOOBData.R , fp+pos, 16);  pos += 16;
	
	blueAPI_Send_LocalOOBDataRsp((status == HCI_SUCCESS) ? blueAPI_CauseSuccess : blueAPI_CauseNotSupported,
								 tConfOOBData.C,
								 tConfOOBData.R
								 );
}

void blueAPI_Handle_HCI_LISTEN_CONF(uint16_t status)
{
    LPbtApplication app = pBTA->activeApp;

    BLUEFACE_TRACE_PRINTF_1(BLUEFACE_TRACE_MASK_TRACE, "BTMAN: LISTEN_CONF status %x", status);

    if (app == NULL)
    {
        BLUEFACE_TRACE_PRINTF_1(BLUEFACE_TRACE_MASK_ERROR, "!!BTMAN: LISTEN_CONF and NO APP", NULL);
    }
    else
    {
        pBTA->activeApp = NULL;
        blueAPI_Send_InquiryRsp(pBlueAPIData->SM_SDP_DS_CommandMsg, FALSE,
            (status == HCI_SUCCESS) ? blueAPI_CauseSuccess : blueAPI_CauseUnspecified);
    }
}

/**
* @brief		hci name confirm
*
* @param	bd: 
* @param	status
* @param	name
*
* @return
*
*/
void blueAPI_Handle_HCI_NAME_CONF(uint8_t * bd, uint16_t status, uint8_t * name)
{
    LPbtApplication app = pBTA->activeApp;
    BLUEFACE_TRACE_PRINTF_3(BLUEFACE_TRACE_MASK_TRACE,
                            "BTA: HCI_NAME_CONF status %x bd %s name %s",
                            status, TRACE_BDADDR1(BLUEFACE_TRACE_MASK_TRACE, bd),
                            BTRACE_RAMDATA1(BLUEFACE_TRACE_MASK_TRACE, name)
                            );

    if (app == NULL)
    {
        BLUEFACE_TRACE_PRINTF_0(BLUEFACE_TRACE_MASK_ERROR, "!!blueAPI_Handle_HCI_NAME_CONF: NO APP");
    }
    else
    {
        pBTA->activeApp = NULL;
        blueFaceSendBT_HCI_NAME_CONF(bd, name, status);
    }
}

/**
* @brief  HCI config indicate mode change
* 
* @param  pmc
*
* @return  
*
*/
void blueAPI_Handle_HCI_MODE_CHANGE_IND(LPHCI_MODECHANGE_IND pmc)
{
    /* destination is selected by link reference thru BD */
    LPbtLink link = blueFaceFindLinkByBD((uint8_t *)pmc->bd);

    if(!link || (pBTA->secApp && link->app != pBTA->secApp))
    {
        /* keep secApp informed in anny case */
        pmc->bLinkContext=NULL;
        blueFaceSendBT_CONF_IND_Mode_Change(pmc);
    }

    if (link)
    {
        pmc->bLinkContext = link->context;
    }
    blueFaceSendBT_CONF_IND_Mode_Change(pmc);
}

/**
* @brief  HCI indicate inquiry result 
* 
* @param  pConfInd_param
*
* @return  
*
*/
void blueAPI_Handle_HCI_INQUIRY_RESULT(THCI_CONF_INQUIRY_RESULT *pConfInd_param)
{
    uint8_t *pCOD = (uint8_t *)&pConfInd_param->classOfDevice;
    uint8_t name[BLUE_API_DEVICE_NAME_LENGTH];

    if ((pBlueAPIData->SM_SDP_DS_Command == blueAPI_EventInquiryReq) &&
        !pBlueAPIData->SM_SDP_DS_CommandMsg->p.InquiryReq.cancelInquiry)
    {
        blueAPI_GetEIRDeviceName(pConfInd_param->extendedResponse, name, BLUE_API_DEVICE_NAME_LENGTH);

        blueAPI_Send_InquiryDeviceInfo(pConfInd_param->bd,((pCOD[2] << 16)|(pCOD[1] << 8)|pCOD[0]),
            pConfInd_param->rssi, name);
        }
        osBufferRelease(pConfInd_param);
}

void blueAPI_Handle_HCI_ROLE_CHANGE(uint16_t indId, TBdAddr bd, uint8_t bdType)
{
	TBlueAPI_ACLStatus status;
	status = (indId == ROLE_CHANGE_TO_MASTER) ? blueAPI_ACLDeviceRoleMaster
												 : blueAPI_ACLDeviceRoleSlave;

#if (BLUEAPI_ENABLE_ACL_INFO & BLUE_API_ENABLE_ACL_INFO_DEVICE_ROLE)
	blueAPI_Send_ACLStatusInfo(bd,
						 (TBlueAPI_RemoteBDType)bdType,
						 status,
						 NULL);
#endif
}

/**
* @brief  blueface send page mode confirm to app
*
* @param  cause
*
* @return  
*
*/
void blueAPI_Handle_HCI_WRITE_SCAN_ENABLE(uint16_t cause)
{
    if (cause != HCI_SUCCESS)
    {
        blueAPI_Send_RadioModeSetRsp(blueAPI_CauseReject);
    }
    else    /*radio mode set, will send inquiry scan iacs auto*/
    {
        blueAPI_Send_BLUEFACE_CONF_INQUIRY_SCAN_IACS(pBlueAPIData,
            pBlueAPIData->ConfigParameter.limitedDiscovery);
    }
}

#if (F_BT_SCO)
/**
 * @brief  hci SCO connection indicate
 *
 * @param  bd
 * @param  devClass
 * @param  linktype
 *
 * @return
 *
 */
void blueAPI_Handle_SCO_CON_IND(uint8_t * bd, uint8_t * devClass, uint8_t linktype)
{
    LPbtLink  link, actLink;
    TCON_RESP_SCO con_resp;

    HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_TRACE, "blueAPI_Handle_SCO_CON_IND: bd %s", TRACE_BDADDR1(HCI_TRACE_MASK_TRACE, bd));

    actLink = blueFaceFindLinkByBD(bd);
    if (actLink == NULL)
    {
        HCI_TRACE_PRINTF_0(HCI_TRACE_MASK_ERROR,"!!blueAPI_Handle_SCO_CON_IND: could not find acl link for this bd");
        hciHandleUpSCOConConf(bd, BLUEFACE_CON_REJECT, &con_resp);
        return;
    }

    link = blueFaceAllocateLink();
    if (link == NULL)
    {
        HCI_TRACE_PRINTF_0(HCI_TRACE_MASK_ERROR,"!!blueAPI_Handle_SCO_CON_IND: could not allocate descriptor");
        hciHandleUpSCOConConf(bd, BLUEFACE_CON_REJECT, &con_resp);
        return;
    }

    link->app           = actLink->app;
    link->context       = actLink->context;
    link->psm           = BLUEFACE_PSM_SCO;
    memcpy(link->bd, bd, BD_ADDR_SIZE);
    link->state         = link_Incoming;
    link->encryptState  = ENCRYPT_IDLE;
    pBTA->activeLink    = link;

    blueAPI_Send_SCOConInd(bd);
}

/**
 * @brief	 hci SCO connect confirm
 *
 * @param  bd
 * @param  status
 *
 * @return
 *
 */
void blueAPI_Handle_SCO_CON_CONF(uint8_t * bd, uint16_t status)
{
    LPbtLink pLink = pBTA->activeLink;

    HCI_TRACE_PRINTF_2(HCI_TRACE_MASK_TRACE, "blueAPI_Handle_SCO_CON_CONF: bd %s status 0x%X",
                        TRACE_BDADDR1(HCI_TRACE_MASK_TRACE, bd), status);

    if (pLink == NULL)
    {
        /* there is no link found related to this BD, this is a severe internal error */
        HCI_TRACE_PRINTF_0(HCI_TRACE_MASK_ERROR,"!!blueAPI_Handle_SCO_CON_CONF: no active link found");
        return;
    }

    if (status)
    {
        blueFaceReleaseActiveLinkSemaphore();
        blueFaceDeallocateLink(pLink);
    }

    blueAPI_Send_SCOConRsp(bd, status);
}

/**
 * @brief	 hci SCO active indicate
 *
 * @param  bd
 * @param  SCOhandle
 * @param  status
 * @param  airMode
 *
 * @return
 *
 */
void blueAPI_Handle_SCO_CON_ACT_IND(uint8_t * bd, uint16_t SCOhandle, uint16_t status, uint8_t airMode)
{
    LPbtLink pLink = pBTA->activeLink;

    HCI_TRACE_PRINTF_4(HCI_TRACE_MASK_TRACE, "blueAPI_Handle_SCO_CON_ACT_IND: bd %s handle SCO %X status %X, air mode %i",
                        TRACE_BDADDR1(HCI_TRACE_MASK_TRACE, bd), SCOhandle, status, airMode);

    if (pLink == NULL)
    {
        /* there is no link found related to this BD, this is a severe internal error */
        HCI_TRACE_PRINTF_0(HCI_TRACE_MASK_ERROR,"!!blueAPI_Handle_SCO_CON_ACT_IND: no active link found");
        return;
    }

    blueFaceReleaseActiveLinkSemaphore();

    if (status)
    {
        /* conConf with error : show disc ind to app and terminate */
        blueFaceDeallocateLink(pLink);
        blueAPI_Send_SCODiscInd(bd);
        return;
    }

    pLink->handle = SCOhandle;

    if (pLink->delayedDisc)
    {
        /* Connection setup with no error, but we wanted to close it.... */
        BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE, "blueAPI_Handle_SCO_CON_ACT_IND: executing delayed disc");
        btgSendDiscReq(blueFacePSM2hQueue(pLink->psm), pLink->handle, HCI_ERR_OTHER_END_TERMINATE_13, FALSE);
        pLink->delayedDisc = FALSE;
        pLink->state       = link_Disconnecting;
        return;
    }

    memcpy(pLink->bd, bd, BD_ADDR_SIZE);
    pLink->state = link_Connected;

    blueAPI_Send_SCOConActInd(bd, airMode);
}

/**
 * @brief	 handle SCO disconnect
 *
 * @param  handle
 * @param  status
 *
 * @return
 *
 */
void blueAPI_Handle_SCO_DISC(uint16_t handle, uint8_t status)
{
    LPbtLink pLink = blueFaceFindLinkByHandle(handle, BLUEFACE_PSM_SCO);

    if (pLink == NULL)
    {
        HCI_TRACE_PRINTF_0(HCI_TRACE_MASK_ERROR,"!!blueAPI_Handle_SCO_DISC: could not find link");
        return;
    }

    if (pLink->state == link_Incoming && !pLink->alerting)
    {
        pLink->state = link_Incoming_Disc;
    }
    else
    {
        if (pLink->state != link_Disconnecting)
        {
            blueAPI_Send_SCODiscInd(pLink->bd);
        }
        blueFaceDeallocateLink(pLink);
    }
}

#endif
