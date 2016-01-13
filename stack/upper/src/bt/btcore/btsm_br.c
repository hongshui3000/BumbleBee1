/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       btsm_br.c
* @brief     Bluetooth Security Manager 2.1
* @details   
*
* @author   	jane
* @date      	2015-10-23
* @version	v0.1
*/

#include <flags.h>
#include <rtl_types.h>
#include <message.h>
#include <btsm.h>
#include <btsmprot.h>
#include <btcommon.h>
#include <hci_api.h>
#include <btsm_api.h>
#include <l2c_api.h>
#include <hci_code.h>
#include <blueapi_api.h>
#include <upper_stack_global.h>

#define TRACE_MODULE_ID     MID_BTSECMAN

/**
* @brief  allocate security entry
*
* @param  newEntry:  cecurity entry
*
* @return  
*
*/
LPTSEC_CONF_SECURITY btsmAllocateSecEntry(LPTSEC_CONF_SECURITY newEntry)
{
	int i;
	for (i = 0; i < BT_SECMAN_POLICY_COUNT; i++)
	{
		LPTSEC_CONF_SECURITY entry = &pBtSM->secEntries[i];
		if (!entry->active)
		{
			memcpy(entry, newEntry, sizeof(TSEC_CONF_SECURITY));
			BTSECMAN_TRACE_PRINTF_6(BTSECMAN_TRACE_MASK_TRACE,
			                     "btsmAllocateSecEntry: id:%d psm:0x%04x sc:%02d out:%d uuid:0x%04x author|authen|mitm|enc:%04x",
			                     i, entry->psm, entry->serverChannel, entry->outgoing, entry->uuid,
			                     (entry->authorize << 12) | (entry->authenSetting << 8) | (entry->mitm << 4) | entry->encryption
			                     );
			return entry;
		} /* if */
	} /* for */

	BTSECMAN_TRACE_PRINTF_0(BTSECMAN_TRACE_MASK_ERROR,
	                          "!!btsmAllocateSecEntry: could not allocate entry"
	                          );
	return NULL;
} /* btsmAllocateSecEntry */

/**
* @brief  deallocate security entry
*
* @param  delEntry:  deallocate entry
*
* @return  
*
*/
uint16_t btsmDeAllocateSecEntry(LPTSEC_CONF_SECURITY delEntry)
{
	/* find corresponding entry in security database and delete it */
	LPTSEC_CONF_SECURITY entry = btsmFindSecEntry(delEntry->outgoing,
	                                             delEntry->psm,
	                                             delEntry->serverChannel,
	                                             delEntry->uuid
	                                             );
	if (entry)
	{
		entry->active = FALSE;
		return 0; /* success */
	}
	/* item not found, return error to the caller */
	return (SECMAN_ERR | SECMAN_ERR_NOENTRY);
} /* btsmDeAllocateSecEntry */

/**
* @brief  find security entry
*
* @param  outgoing
* @param  psm
* @param  serverChannel
* @param  uuid
*
* @return  
*
*/
LPTSEC_CONF_SECURITY btsmFindSecEntry(uint8_t outgoing, uint16_t psm, uint16_t serverChannel, uint16_t uuid)
{
    /* find security requirements according to values */
    int i;

    for (i = 0; i < BT_SECMAN_POLICY_COUNT; i++)
    {
        LPTSEC_CONF_SECURITY entry = &pBtSM->secEntries[i];
        if (entry->active && entry->outgoing == outgoing)
        {
            if (entry->outgoing)
            {
                /* outgoing: match psm and UUID */
                if (psm == entry->psm && entry->uuid == uuid)
                {
                    return entry;
                }
            }
            else if (!entry->outgoing)
            {
                /*  incoming: match psm and server channel, l2cap server channel = 0 */
                if (entry->serverChannel == serverChannel && entry->psm == psm)
                {
                    return entry;
                }
            }
        }
    } /* for */

	return NULL; /* nothing relevant found */
} /* btsmFindSecEntry */

/**
* @brief  check if security entry valid
*
* @param  sec: security entry configure
*
* @return  
*
*/
BOOL btsmCheckNewSecEntry(LPTSEC_CONF_SECURITY sec)
{
    BOOL valid = TRUE;
    /* check for some illegal combinations */

    if (sec->active)
    {
        /* encrypt without authenticate */
        if (sec->encryption && sec->authenSetting != AUTHEN_SETTING_AUTHENTICATE)
        {
            valid = FALSE;
        }
        /* MITM without authenticate */
        if (valid && sec->mitm && sec->authenSetting != AUTHEN_SETTING_AUTHENTICATE)
        {
            valid = FALSE;
        }
        /* no authenticate no authorize */
        if (valid && sec->authorize == FALSE && sec->authenSetting != AUTHEN_SETTING_AUTHENTICATE)
        {
            valid = FALSE;
        }
        /* no psm */
        if (valid && sec->psm == 0)
        {
            valid = FALSE;
        }

        if (sec->outgoing)
        {
            /* no uuid */
            if (valid && sec->uuid == 0)
            {
                valid = FALSE;
            }
        }
        else
        {
            /* rfcomm no server channel */
            if (valid && sec->psm == PSM_RFCOMM && sec->serverChannel == 0)
            {
                valid = FALSE;
            }
        }

        /* check for double entries */
        if(valid && btsmFindSecEntry(sec->outgoing, sec->psm, sec->serverChannel, sec->uuid))
        {
            valid = FALSE;
        }
    }
    else
    {
        valid = FALSE;
    }

    return valid;
} /* btsmCheckNewSecEntry */

/**
* @brief  security check if is just work		
*
* @param  link
*
* @return  
*
*/
BOOL btsmCheckForJustWorks(LPTSECLINK link)
{
	BOOL justWorks      = FALSE;
	uint8_t authenSettings = pBtSM->devSecurity.authenSettings;

	/* service policy requires MITM, temporarily force MITM device settings */
	if (link->state == SEC_LINK_AUTHEN_MITM_REQUESTED)
	{
		authenSettings |= DEVAUTH_SETTING_MITM_MASK;
	}

	if((!(link->remote_authRequirements & DEVAUTH_SETTING_MITM_MASK)) &&
	 (!(authenSettings & DEVAUTH_SETTING_MITM_MASK)))
	{
		justWorks = TRUE;
	}
	/* handle out-of-band pairing as mitm secure */
	else if (!pBtSM->devSecurity.oob_present)
	{
		switch (pBtSM->devSecurity.io_capabilities)
		{
		case BLUEFACE_IOCAPA_DISPLAYONLY :
			if((link->remote_ioCapability == BLUEFACE_IOCAPA_DISPLAYONLY) ||
			(link->remote_ioCapability == BLUEFACE_IOCAPA_DISPLAYYESNO) ||
			(link->remote_ioCapability == BLUEFACE_IOCAPA_NOIO))
			{
				justWorks = TRUE;
			}
			break;
		case BLUEFACE_IOCAPA_KEYBOARDONLY :
		case BLUEFACE_IOCAPA_KEYBOARDDISPLAY:
			if((link->remote_ioCapability == BLUEFACE_IOCAPA_NOIO))
			{
				justWorks = TRUE;
			}
			break;
		case BLUEFACE_IOCAPA_NOIO:
			justWorks = TRUE;
			break;
		default :
		    break;
		}
	}
	return justWorks;
}

/**
* @brief	 secmanager check for mitm
*
* @param	link: 
*
* @return
*
*/
BOOL btsmCheckForMitm(LPTSECLINK link)
{
	BOOL mitm           = TRUE;
	uint8_t authenSettings = pBtSM->devSecurity.authenSettings;

	/* service policy requires MITM, temporarily force MITM device settings */
	if (link->state == SEC_LINK_AUTHEN_MITM_REQUESTED)
	{
		authenSettings |= DEVAUTH_SETTING_MITM_MASK;
	}

	if((!(link->remote_authRequirements & DEVAUTH_SETTING_MITM_MASK)) &&
	 (!(authenSettings & DEVAUTH_SETTING_MITM_MASK)))
	{
		mitm = FALSE;
	}
	/* handle out-of-band pairing as mitm secure */
	else if (!pBtSM->devSecurity.oob_present)
	{
		switch (pBtSM->devSecurity.io_capabilities)
		{
		case BLUEFACE_IOCAPA_DISPLAYONLY :
			if((link->remote_ioCapability == BLUEFACE_IOCAPA_DISPLAYONLY) ||
			(link->remote_ioCapability == BLUEFACE_IOCAPA_DISPLAYYESNO) ||
			(link->remote_ioCapability == BLUEFACE_IOCAPA_NOIO))
			{
				mitm = FALSE;
			}
			break;
		case BLUEFACE_IOCAPA_DISPLAYYESNO:
			if((link->remote_ioCapability == BLUEFACE_IOCAPA_DISPLAYONLY) ||
			(link->remote_ioCapability == BLUEFACE_IOCAPA_NOIO))
			{
				mitm = FALSE;
			}
			break;
		case BLUEFACE_IOCAPA_KEYBOARDONLY :
		case BLUEFACE_IOCAPA_KEYBOARDDISPLAY:
			if((link->remote_ioCapability == BLUEFACE_IOCAPA_NOIO))
			{
				mitm = FALSE;
			}
			break;
		case BLUEFACE_IOCAPA_NOIO:
			mitm = FALSE;
			break;
		default :
		    break;
		}
	}
	return mitm;
}

/**
* @brief	 secmanager check security
*
* @param	link: 
*
* @return
*
*/
void btsmCheckSecurity(LPTSECLINK link)
{
	BOOL reply = FALSE;
	uint16_t cause = HCI_SUCCESS;

	BTSECMAN_TRACE_PRINTF_3(BTSECMAN_TRACE_MASK_TRACE,
	                         "btsmCheckSecurity: bd:%s state:0x%x mode:0x%x",
	                         TRACE_BDADDR1(BTSECMAN_TRACE_MASK_TRACE, link->bd),
	                         link->state, link->mode
	                         );

	/* not triggered from protocol layer */
	if (link->auth_source == 0x00)
	{
		return;
	}

	/* SSP status of remote is not yet known:
	* stop processing now, wait for SSP information from HCI layer, proceed then
	*/
	if ((link->mode & LINKMODE_SSP_KNOWN) == 0)
	{
		return;
	}

	do /* the do loop just marks a common exit label */
	{
		/* link requires authorization */
		if ((link->mode & (LINKMODE_AUTHOR_REQUIRED | LINKMODE_AUTHOR)) == LINKMODE_AUTHOR_REQUIRED)
		{
			if (link->state != SEC_LINK_AUTHOR_REQUESTED)
			{
				BTSECMAN_TRACE_PRINTF_0(BTSECMAN_TRACE_MASK_TRACE,
				                        "btsmCheckSecurity: trigger AUTHORIZATION"
				                        );

				link->state = SEC_LINK_AUTHOR_REQUESTED;
				blueAPI_Handle_BLUEFACE_CONF_AUTHORIZE(link);
			}
			break;
		}

		/* when both peers are bt2.1 and this is an incomming connection,
		 * and this is not an SDP (L2CAP PSM 0x0001) connect,
		 * => REJECT unauthenticated/unencrypted connections
		 */
		if ((link->mode & LINKMODE_SSP_CAPA) && (link->auth_outg == 0) &&
		    (link->auth_source != SECMAN_SOURCE_L2CAP|| link->auth_cid != 0x0001)
		   )
		{
			/* authentication required */
			if ((link->mode & LINKMODE_AUTHEN) == 0)
			{
				BTSECMAN_TRACE_PRINTF_0(BTSECMAN_TRACE_MASK_ERROR,
				                           "!!btsmCheckSecurity: Incoming bt2.1 connection not authenticated"
				                           );

				cause = HCI_ERR | HCI_ERR_INSUFFICIENT_SECURITY;
				reply = TRUE;
				break;
			}

		  /* encryption required
		   * during authentication (MITM upgrade) encryption is temporarily disabled
		   */
		  else if (((link->mode & LINKMODE_ENCRYPTED) == 0) &&
		           (link->state != SEC_LINK_AUTHEN_REQUESTED) &&
		           (link->state != SEC_LINK_AUTHEN_MITM_REQUESTED)
		          )
			{
				BTSECMAN_TRACE_PRINTF_0(BTSECMAN_TRACE_MASK_ERROR,
				                           "!!btsmCheckSecurity: Incoming bt2.1 connection not encrypted"
				                           );

				cause = HCI_ERR | HCI_ERR_INSUFFICIENT_SECURITY;
				reply = TRUE;
				break;
			}
		} /* bt2.1 incoming */

		/* authenticated (MITM) linkkey required */
		if ((link->mode & (LINKMODE_MITM_REQUIRED | LINKMODE_MITM)) == LINKMODE_MITM_REQUIRED)
		{
			/* check if MITM is possible with local io-capabilities */
			if ((link->mode & LINKMODE_SSP_CAPA) &&
			  (pBtSM->devSecurity.io_capabilities != BLUEFACE_IOCAPA_NOIO)
			 )
			{
				if ((link->state != SEC_LINK_AUTHEN_REQUESTED) &&
				    (link->state != SEC_LINK_AUTHEN_MITM_REQUESTED)
				   )
				{
					BTSECMAN_TRACE_PRINTF_0(BTSECMAN_TRACE_MASK_TRACE,
					                         "btsmCheckSecurity: trigger MITM AUTHENTICATION"
					                         );

					link->state = SEC_LINK_AUTHEN_MITM_REQUESTED;
			

					btsmSendLHciAuthReq(link->bd, link->index/* id is index into link table */);				
					btsmUpdateSecurityStatus(link, SECSTAT_AUTH_REQ, HCI_SUCCESS);
				}
			}
			else
			{
				BTSECMAN_TRACE_PRINTF_0(BTSECMAN_TRACE_MASK_ERROR,
				                           "!!btsmCheckSecurity: MITM is not possible"
				                           );

				cause = HCI_ERR | HCI_ERR_INSUFFICIENT_SECURITY;
				reply = TRUE;
			}
			break;
		}

		/* unauthenticated linkkey is required by service policy or
		 * an outgoing BT2.1 connect (but not to SDP (L2CAP PSM 0x0001))
		 */
		if ((link->mode & LINKMODE_AUTHEN_REQUIRED) ||
		    ((link->mode & LINKMODE_SSP_CAPA) && (link->auth_outg == 1) &&
		     (link->auth_source != SECMAN_SOURCE_L2CAP|| link->auth_cid != 0x0001)
		   ))
		{
			if ((link->mode & LINKMODE_AUTHEN) == 0)
			{
				if ((link->state != SEC_LINK_AUTHEN_REQUESTED) &&
				    (link->state != SEC_LINK_AUTHEN_MITM_REQUESTED)
				   )
				{
					BTSECMAN_TRACE_PRINTF_0(BTSECMAN_TRACE_MASK_TRACE,
					                         "btsmCheckSecurity: trigger AUTHENTICATION"
					                         );

					link->state = SEC_LINK_AUTHEN_REQUESTED;

					btsmSendLHciAuthReq(link->bd, link->index /* id is index into link table */);
				
					btsmUpdateSecurityStatus(link, SECSTAT_AUTH_REQ, HCI_SUCCESS);
				}
				break;
			}
		}

		/* encryption is required by service policy or
		 * an outgoing BT2.1 connect (but not to SDP (L2CAP PSM 0x0001))
		 */
		if ((link->mode & LINKMODE_ENCR_REQUIRED) ||
		    ((link->mode & LINKMODE_SSP_CAPA) && (link->auth_outg == 1) &&
		     (link->auth_source != SECMAN_SOURCE_L2CAP|| link->auth_cid != 0x0001)
		   ))
		{
			if ((link->mode & LINKMODE_ENCRYPTED) == 0)
			{
				if (link->state != SEC_LINK_ENCRYPT_REQUESTED)
				{
					BTSECMAN_TRACE_PRINTF_0(BTSECMAN_TRACE_MASK_TRACE,
					                         "btsmCheckSecurity: trigger ENCRYPTION"
					                         );

					link->state = SEC_LINK_ENCRYPT_REQUESTED;
	                hciHandleUpEncryptReq((uint8_t*)link->bd, TRUE);
				}
				break;
			}
		}

		/* all requirements met, accept */
		cause = SECMAN_SUCCESS;
		reply = TRUE;
	} while (0);

	if (reply)
	{
		btsmAuthenticationComplete(link, cause);
	}
}


/**
* @brief  btsm send hci key response
* 
* @param  bd
* @param  reqType
* @param  cause
* @param  key
* @param  keyLen
*
* @return  
*
*/
void btsmSendLHciKeyResp(LPCBYTE bd, uint8_t reqType, uint16_t cause, LPCBYTE key, uint16_t keyLen)
{
	HCI_TRACE_PRINTF_3(HCI_TRACE_MASK_TRACE,"hciHandleUpKeyResp: req %x bd %s status %x", reqType, TRACE_BDADDR1(HCI_TRACE_MASK_TRACE, bd), cause);

	if (reqType == LINK_KEY_REQUEST)
	{
		if (cause)
		{
			hciCommandBDAddrParameter(HCI_LINK_KEY_REQUEST_NEG_REPLY, (uint8_t *)bd);
		}
		else
		{
			hciCommandLinkKeyRequestReply((uint8_t *)bd, (uint8_t *)key);
		}
	}
	else if (reqType == PIN_CODE_REQUEST)
	{
		if (cause)
		{
			hciCommandBDAddrParameter(HCI_PIN_CODE_REQUEST_NEG_REPLY, (uint8_t *)bd);
		}
		else
		{
			hciCommandPinCodeRequestReply((uint8_t *)bd, (uint8_t *)key, keyLen);
		}
	}

	hciLaterEntry();
} /* btsmSendLHciKeyResp */

/**
* @brief  send io capability reply to remote
*
* @param  bd
* @param  capability
* @param  oob_data_present
* @param  auth_requirements
* @param  status
*
* @return  
*
*/
void btsmSendLHciSspIoCapabilityReplyReq(uint8_t * bd, uint8_t capability, uint8_t oob_data_present, uint8_t auth_requirements, uint8_t status)
{
	if (status == 0)
	{
        hciCommandIoCapabilityRequestReply(bd, capability, oob_data_present, auth_requirements);
	} else
	{
		hciCommandBDAddrByteParameter(HCI_IO_CAPABILITY_REQUEST_NEGATIVE_REPLY,
									bd,
									status);
	}
	hciLaterEntry();
} /* btsmSendLHciSspIoCapabilityReplyReq */

/**
* @brief  send hci user confirm request response	
*
* @param  bd
* @param  result
*
* @return  
*
*/
void btsmSendLHciUserConfReqResp(uint8_t * bd, uint8_t result)
{
	if (result == 0)
	{
		hciCommandBDAddrParameter(HCI_USER_CONFIRMATION_REQUEST_REPLY,
								bd);
	}
	else
	{
		hciCommandBDAddrParameter(HCI_USER_CONFIRMATION_REQUEST_NEGATIVE_REPLY,
								bd);
	}
	hciLaterEntry();
} /* btsmSendLHciUserConfReqResp */

/**
* @brief  btsm send auth request
* 
* @param  bd
* @param  id
*
* @return  
*
*/
void btsmSendLHciAuthReq(LPCBYTE bd, uint8_t id)
{
	hciHandleUpAuthReq((uint8_t *)bd, id);
	hciLaterEntry();
} /* btsmSendLHciAuthReq */

/**
* @brief	 secmanager authentication response
*
* @param	module: 
* @param	bd
* @param	ref
* @param	channelId
* @param	outgoing
* @param	accept
*
* @return
*
*/
void btsmSendSECMAN_AUTHENTICATION_RESP(uint8_t Source, uint8_t * bd, uint16_t ref, uint16_t channelId, uint8_t outgoing, BOOL accept)
{
    uint16_t      cause;

	if (Source == SECMAN_SOURCE_L2CAP)
	{
		cause = L2CAP_ERR | L2CAP_ERR_REFUS_SEC_BLOCK;
		cause = accept ? SECMAN_SUCCESS : cause;
		l2cHandleSECMAN_AUTHENTICATION_RESP(bd, ref, channelId, 0, outgoing, cause);
	}
	else if (Source == SECMAN_SOURCE_RFCOMM)
	{
		cause = RFCOMM_ERR | RFCOMM_ERR_REJECTED;
		cause = accept ? SECMAN_SUCCESS : cause;
        blueAPI_Send_RFCAuthenticationRsp(bd, ref, channelId, outgoing, cause);
	}
} /* btsmSendBT_SECMAN_CONFIGURATION_RESP */

uint16_t btsmHandleBT_HCI_KEY_RESP(LPCBYTE bd, uint8_t reqType, uint16_t cause, LPCBYTE key, uint16_t keyLen)
{
	LPTSECLINK	 link	 = NULL;

	BTSECMAN_TRACE_PRINTF_3(BTSECMAN_TRACE_MASK_TRACE,
						   "btsmHandleHciKeyResp: reqType:0x%x status:0x%x key:%s",
						   reqType, cause,
						   BTRACE_RAMDATA1(BTSECMAN_TRACE_MASK_TRACE,btsmHexString((uint8_t *)key, keyLen)));

	/*	if this link key is an authenticated link key,
	then remember this particular fact in the security properties of this link
	*/
	if (reqType == LINK_KEY_REQUEST ||reqType == LINK_KEY_REQUEST_MITM)
	{
		link = btsmFindLink((uint8_t *)bd);
		if (link)
		{
			if (cause == HCI_SUCCESS)
			{
				link->mode   |= (LINKMODE_AUTHEN | LINKMODE_STORED_KEY);
				/* actual keytype & keysize not known on BR/EDR */
				link->keyType = BLUEFACE_KEYTYPE_UNAUTHENTICATED;
				link->keySize = 16;

				if (reqType == LINK_KEY_REQUEST_MITM)
				{
					link->mode	 |= LINKMODE_MITM;
					link->keyType = BLUEFACE_KEYTYPE_AUTHENTICATED;
				}
				else if (link->state == SEC_LINK_AUTHEN_MITM_REQUESTED)
				{
					/* not a MITM key, but MITM was requested */
					cause = SECMAN_ERR | SECMAN_ERR_LINKKEY_NOT_SUFFICIENT;
					link->mode &= ~(LINKMODE_STORED_KEY);
				}
			}
			else
			{
				link->mode   &= ~(LINKMODE_STORED_KEY);
				link->keyType = BLUEFACE_KEYTYPE_DELETE;
				link->keySize = 0;
			}
		}

		/* reset keytype to linkkey for HCI layer! */
		reqType = LINK_KEY_REQUEST;
	}
	
	btsmSendLHciKeyResp(bd, reqType, cause, key, keyLen);

	return BLUEFACE_ERR | BF_SUCCESS;
} /* btsmHandleBT_HCI_KEY_RESP */


void btsmSendMsgHciAuthReq(LPCBYTE bd, uint8_t id)
{
    MESSAGE_T msg;
    ThciAuthReq * authReq = (ThciAuthReq *) msg.MData.MessageData;

    memcpy(authReq->bd, bd, BD_ADDR_SIZE);
    authReq->id        = id;
    msg.Command        = HCI_AUTH_REQ;

    osMessageSend(btsmQueueID, &msg);
}

void btsmHandleHciAuthReq()
{
    ThciAuthReq * authReq = &pBtSM->message.MData.hciAuthReq;
	LPTSECLINK    link    = btsmFindLink(authReq->bd);

	if (link == NULL)
	{
		blueAPI_Handle_AuthConf(authReq->id, HCI_ERR | HCI_ERR_REJECT_LIMITED_RESOURCES);
		return;
	}

	if (link->state != SEC_LINK_IDLE)
	{
		btsmDeferredQueueIn(link);
		return;
	}

	link->state    = SEC_LINK_EXTERNAL_AUTH_REQ;
	link->mode    &= ~(LINKMODE_AUTHEN | LINKMODE_MITM | LINKMODE_STORED_KEY);
	link->keyType  = BLUEFACE_KEYTYPE_DELETE;
	link->keySize  = 0;

	btsmUpdateSecurityStatus(link, SECSTAT_AUTH_REQ, HCI_SUCCESS);

	link->extAuthReqId = authReq->id;

    authReq->id =link->index;

	hciHandleUpAuthReq(authReq->bd, authReq->id);
	hciLaterEntry();
}

/**
* @brief  security manager handle authentication indicate
* 
* @param  bd
* @param  ref
* @param  channelId
* @param  uuid
* @param  outgoing
* @param  active
* @param  module
*
* @return  
*
*/
void btsmHandleAuthenticationInd()
{
    PTSecManAuthenticationInd auth = &pBtSM->message.MData.secManAuthenticationInd;
    uint8_t Source                    = auth->Source;
	LPTSECLINK link                = btsmFindAllocateLink(BLUEFACE_BDTYPE_BR_EDR, auth->bd);
	BOOL reply                     = TRUE; /* do reply with AUTH_RESP */
	LPTSEC_CONF_SECURITY entry     = NULL;

	BTSECMAN_TRACE_PRINTF_5(BTSECMAN_TRACE_MASK_TRACE,
	                         "btsmHandleAuthenticationInd: Source:0x%x bd:%s ref:0x%x cid:0x%x out:%d",
	                         Source, TRACE_BDADDR1(BTSECMAN_TRACE_MASK_TRACE, auth->bd),
	                         auth->ref, auth->channelId, auth->outgoing
	                         );

	do /* the do loop just marks a common exit label */
	{
		if (!link)
		{
			/* no link descriptor found (this signal an internal error (too many links), reject the connection attempt */
			break;
		} 

		/* authentication to/from this BD already running */
		if (link->state != SEC_LINK_IDLE)
		{
			btsmDeferredQueueIn(link);
			reply = FALSE;
			break;
		}

		if (!auth->active)
		{
			/* this is an information, that the request for permission is cancelled, stop any ongoing operation */
			reply = FALSE;
			if (link->state == SEC_LINK_AUTHOR_REQUESTED)
			{
				blueAPI_Handle_BLUEFACE_CONF_AUTHORIZE(link);
				link->state       = SEC_LINK_IDLE;
				link->auth_source = 0x00;
			}
			break;
		} 

		if (Source == SECMAN_SOURCE_L2CAP)
		{
			entry = btsmFindSecEntry(auth->outgoing, auth->channelId, 0 /* serverChannel */, auth->uuid);
		}
		else if (Source == SECMAN_SOURCE_RFCOMM)
		{
			entry = btsmFindSecEntry(auth->outgoing, PSM_RFCOMM /* PSM */, (auth->channelId >> 1), auth->uuid);
		}
		else
		{
			BTSECMAN_TRACE_PRINTF_1(BTSECMAN_TRACE_MASK_ERROR,
			                           "!!btsmHandleAuthenticationInd: unknown Source 0x%x",
			                           Source
			                           );
			return;
		}

		link->auth_ref    = auth->ref;        /* remember relevant parameters for _RESP */
		link->auth_cid    = auth->channelId;  /* remember relevant parameters for _RESP */
		link->auth_source = Source;
		link->auth_outg   = auth->outgoing;

		/* policy based request, allow stored keys (btsec will handle keys), reset all requirements */
		link->mode       |= LINKMODE_POLICY_BASED;
		link->mode       &= ~(LINKMODE_FORCE_PAIRING |
		                      LINKMODE_AUTHEN_REQUIRED | LINKMODE_MITM_REQUIRED |
		                      LINKMODE_ENCR_REQUIRED | LINKMODE_AUTHOR_REQUIRED);

		if (entry)
		{
			link->auth_uuid = auth->outgoing ? auth->uuid : entry->uuid;

			if (entry->authenSetting)
			{
				link->mode |= LINKMODE_AUTHEN_REQUIRED;
			}

			/* authenticated (MITM) linkkey required by policy or device settings */
			if (entry->mitm ||
			  (pBtSM->devSecurity.authenSettings & DEVAUTH_SETTING_MITM_MASK)
			 )
			{
				link->mode |= LINKMODE_MITM_REQUIRED;
			}

			if (entry->encryption)
			{
				link->mode |= LINKMODE_ENCR_REQUIRED;
			}

			if (entry->authorize)
			{
				link->mode |= LINKMODE_AUTHOR_REQUIRED;
			}
		}
		else
		{
			link->auth_uuid = auth->outgoing ? auth->uuid : 0;
		}

		reply = FALSE;
		btsmCheckSecurity(link);

	} while (0);

	if (reply)
	{
		btsmSendSECMAN_AUTHENTICATION_RESP(Source, auth->bd, auth->ref, auth->channelId, auth->outgoing, FALSE);
	}
} /* btsmHandleAuthenticationInd */

void btsmHandleHciAuthConf(uint8_t * bd, uint8_t id, uint16_t status)
{
	LPTSECLINK     link         = NULL;

	BTSECMAN_TRACE_PRINTF_2(BTSECMAN_TRACE_MASK_TRACE,
	                          "btsmHandleHciAuthConf: id:0x%x status:0x%x",
	                          id, status
	                          );

    if(id < 0x10)
    {
        if(id < otp_str_data.gEfuse_UpperStack_s.num_link_doff)
        {
	        link       = &pBtSM->plinksDoff[id];
        }
    }
	else
	{
        id -= 0x10;
        if(id < otp_str_data.gEfuse_UpperStack_s.num_link_don)
        {
	        link       = &pBtSM->plinksDon[id];
        }
	}

	if (link)
	{
		if (status == HCI_SUCCESS)
		{
			link->mode |= LINKMODE_AUTHEN; /* link is now authenticated */
		}
		else if ((status == (HCI_ERR | HCI_ERR_KEY_MISSING)) &&
		       (link->mode & LINKMODE_STORED_KEY)
		      )
		{
			link->keyType = BLUEFACE_KEYTYPE_DELETE;
			link->keySize = 0;

			if (pBtSM->devSecurity.pairable_mode)
			{
			  /* pairing failed due to key missing, remove the key from the applications database */
			  blueAPI_Handle_NewKeyInd(bd, NULL, BLUEFACE_KEYTYPE_DELETE);
			}
		}

		btsmUpdateSecurityStatus(link, SECSTAT_AUTH_COMPLETE, status);

		/* forward ext. AUTH_REQ to BTMAN */
		if (link->state == SEC_LINK_EXTERNAL_AUTH_REQ)
		{
		    if ((!pBtSM->devSecurity.pairable_mode) &&
		        (status == (HCI_ERR | HCI_ERR_AUTHENTICATION_FAILED))
		       )
		    {
				status = HCI_ERR | HCI_ERR_PARING_NOT_ALLOWED;

				BTSECMAN_TRACE_PRINTF_1(BTSECMAN_TRACE_MASK_ERROR,
				                         "!!btsmHandleHciAuthConf for bd:%s own device is non pairable",
				                         TRACE_BDADDR1(BTSECMAN_TRACE_MASK_ERROR, bd)
				                         );
		    }

		    id = link->extAuthReqId;
            blueAPI_Handle_AuthConf(id, status);
	    	link->state = SEC_LINK_IDLE;
	  	}
	  	else
	  	{
		    if (status == HCI_SUCCESS)
		    {
				/* MITM requested, but link not MITM secured */
				if (link->state == SEC_LINK_AUTHEN_MITM_REQUESTED &&
				  ((link->mode & LINKMODE_MITM) == 0)
				 )
				{
					BTSECMAN_TRACE_PRINTF_0(BTSECMAN_TRACE_MASK_ERROR,
					                           "!!btsmHandleHciAuthConf: key is not MITM"
					                           );

					status = SECMAN_ERR | SECMAN_ERR_LINKKEY_NOT_SUFFICIENT;
				}
				else
				{
					btsmCheckSecurity(link);
				}
		    }
	    	else if ((status == (HCI_ERR | HCI_ERR_KEY_MISSING)) &&
	             (link->mode & LINKMODE_STORED_KEY) &&
	             (pBtSM->devSecurity.pairable_mode)
	            )
	    	{
				/* authfail using stored key, retry authentication without key */
				BTSECMAN_TRACE_PRINTF_0(BTSECMAN_TRACE_MASK_ERROR,
				                         "btsmHandleHciAuthConf: RETRY"
				                         );

				link->state   = SEC_LINK_AUTHEN_RETRY;
				link->mode   &= ~(LINKMODE_AUTHEN | LINKMODE_MITM | LINKMODE_STORED_KEY);

				/* wait 1s to prevent HCI_REPEATED_ATTEMPT error */
				btsmStartTimer(link, SM_TIMER_AUTH_RETRY, 1);

				/* do not sent AUTH_RESP yet */
				status = HCI_SUCCESS;
	    	}

			if (status)
			{
                btsmAuthenticationComplete(link, HCI_ERR | HCI_ERR_AUTHENTICATION_FAILED);
			}
	  	}
	} /* id is OK */
	else
	{
		BTSECMAN_TRACE_PRINTF_1(BTSECMAN_TRACE_MASK_ERROR,
		                         "!!btsmHandleHciAuthConf: invalid id:0x%x",
		                         id
		                         );
	}
}

void btsmHandleHciKeyInd(uint8_t * bd, uint8_t reqType)
{
    /* answer the message locally if the reaction is clear ! */
	/* otherwise forward the message to BTMAN for further processing */
	BOOL         forward = TRUE;
	LPTSECLINK   pLink   = btsmFindLink(bd);

	if (reqType == PIN_CODE_REQUEST)
	{
		btsmUpdateSecurityStatus(pLink, SECSTAT_PIN_CODE_REQ, HCI_SUCCESS);

		if (!pBtSM->devSecurity.pairable_mode)
		{
			/* received PIN CODE REQUEST, but the device is not bondable, reject the PIN CODE REQUEST */
			BTSECMAN_TRACE_PRINTF_1(BTSECMAN_TRACE_MASK_ERROR,
			                          "!!btsmHandleHciKeyInd for bd:%s own device is non pairable",
			                          TRACE_BDADDR1(BTSECMAN_TRACE_MASK_ERROR, bd)
			                          );
			btsmSendLHciKeyResp(bd, reqType, 1 /* REJECT */, NULL /* key */, 0 /* keylen */);
			forward = FALSE;
		}
	}
	else
	{
		btsmUpdateSecurityStatus(pLink, SECSTAT_LINK_KEY_REQ, HCI_SUCCESS);

		/* do not use a stored linkkey */
		if (pLink->mode & LINKMODE_FORCE_PAIRING)
		{
			btsmSendLHciKeyResp(bd, reqType, 1 /* REJECT */, NULL /* key */, 0 /* keylen */);
			forward = FALSE;
		}
	}

	if (forward)
	{
		blueAPI_Handle_HciKeyInd(bd, reqType);
	}

}

void btsmHandleHciNewHandle(uint8_t bdType, TBdAddr bd)
{
    btsmFindAllocateLink(bdType, bd);
#if (BLUEAPI_ENABLE_ACL_INFO & BLUE_API_ENABLE_ACL_INFO_CONNECTED_ACTIVE)			
	blueAPI_Send_ACLStatusInfo(bd,
							   (TBlueAPI_RemoteBDType)bdType,
							   blueAPI_ACLConnectedActive,
							   NULL);
#endif
}

void btsmHandleHciEncryptInd(uint8_t * bd, uint16_t status, uint8_t enable)
{
    LPTSECLINK       link;
    link = btsmFindLink(bd);

	BTSECMAN_TRACE_PRINTF_3(BTSECMAN_TRACE_MASK_TRACE,
	                          "btsmHandleHciEncryptInd: bd:%s encrypted:%d status:0x%x",
	                          TRACE_BDADDR1(BTSECMAN_TRACE_MASK_TRACE, bd),
	                          enable, status
	                          );

	if (link)
	{
		btsmSendSECURITY_STATUS(
		     enable ? CONF_SECURITY_STATUS_ENCRYPTED : CONF_SECURITY_STATUS_NOT_ENCRYPTED,
		     link->bd, link->bdType, status, link->keyType, link->keySize);

		if (status == HCI_SUCCESS)
		{
			if (enable)
			{
				link->mode |= LINKMODE_ENCRYPTED;
			}
			else
			{
				link->mode &= ~(LINKMODE_ENCRYPTED);
			}
		}

		if (link->state == SEC_LINK_ENCRYPT_REQUESTED)
		{
			if (link->mode & LINKMODE_ENCRYPTED)
			{
				btsmCheckSecurity(link);
			}
			else
			{
				btsmAuthenticationComplete(link, HCI_ERR | HCI_ERR_AUTHENTICATION_FAILED);
			}
		}
	}
}

void btsmHandleHciAclConnection(TBdAddr bd, uint16_t indId)
{
	LPTSECLINK link = btsmFindLink(bd);
	if (link)
	{
		link->mode |= LINKMODE_SSP_KNOWN;    
		if ((pBtSM->devSecurity.bt_mode != BTMODE_SETTING_NO_BT21) &&
		    (indId == ACL_CONNECTION_SSP)
		   )
		{
			/* remember SSP capability of the link */
			link->mode |= LINKMODE_SSP_CAPA;
		}

		btsmCheckSecurity(link);
	} 
}

void btsmHandleHCI_LINK_KEY_NOTIFICATION(uint8_t * bd, uint8_t * key, uint8_t keyType)
{
	LPTSECLINK     link      = btsmFindLink(bd);

	BTSECMAN_TRACE_PRINTF_3(BTSECMAN_TRACE_MASK_TRACE,
	                          "btsmHandleHciNewkeyInd: bd:%s keyType:0x%x key:%s",
	                          TRACE_BDADDR1(BTSECMAN_TRACE_MASK_TRACE, bd),
	                          keyType,
	                          BTRACE_RAMDATA1(BTSECMAN_TRACE_MASK_TRACE, btsmHexString(key, 16))
	                          );

	if (link)
	{
		link->mode |= LINKMODE_AUTHEN; /* remember that this is authenticated (by a new link key) */
		link->mode &= ~(LINKMODE_STORED_KEY);

		/* map DEBUG keytype (upper layer should handle this as a normal key) */
		if (keyType == HCI_KEYTYPE_DEBUG)
		{
			keyType = btsmCheckForMitm(link) ? BLUEFACE_KEYTYPE_AUTHENTICATED : BLUEFACE_KEYTYPE_UNAUTHENTICATED;
			BTSECMAN_TRACE_PRINTF_1(BTSECMAN_TRACE_MASK_TRACE,
			                           "btsm: mapping DEBUG keytype to 0x%x",
			                           keyType
			                           );
		}

		if (keyType == HCI_KEYTYPE_AUTHENTICATED)
		{
			link->mode |= LINKMODE_MITM;
		}

		link->keyType = keyType;
		link->keySize = 16; /* actual keysize not known on BR/EDR */

		btsmUpdateSecurityStatus(link, SECSTAT_NEW_LINK_KEY, HCI_SUCCESS);
		/* (F_BT_SECMAN_STATUS_UPDATES) */

		if (link->state == SEC_LINK_REMOTE_SSP)
		{
			/* remote SSP auth/encrypt completed */
			link->state = SEC_LINK_IDLE;
			btsmDeferredQueueOut();
		}
	}
	else if (keyType == HCI_KEYTYPE_DEBUG)
	{
		/* no link found, map DEBUG keytype */
		keyType = BLUEFACE_KEYTYPE_UNAUTHENTICATED;
	}

	if ((pBtSM->devSecurity.authenSettings & DEVAUTH_SETTING_BOND_MASK) != DEVAUTH_SETTING_NO_BOND)
	{
        blueAPI_Handle_HCI_LINK_KEY_NOTIFICATION(bd, key, keyType);
	}
}

void btsmHandleHCI_IO_CAPABILITY_REQUEST(uint8_t * bd)
{
	LPTSECLINK		 link	 = NULL;
	uint8_t status         = pBtSM->devSecurity.pairable_mode ? HCI_SUCCESS : HCI_ERR_PARING_NOT_ALLOWED;
	uint8_t authenSettings = pBtSM->devSecurity.authenSettings;
	uint8_t ioCaps         = pBtSM->devSecurity.io_capabilities;

	/* service policy requires MITM, temporarily force MITM device settings */
	link = btsmFindLink(bd);
	if (link)
	{
		if (link->state == SEC_LINK_IDLE)
		{
			/* remote started SSP Pairing */
			link->state = SEC_LINK_REMOTE_SSP;
		}

		btsmUpdateSecurityStatus(link, SECSTAT_SSP_START, HCI_SUCCESS);
		/* F_BT_SECMAN_STATUS_UPDATES */

		if (link->state == SEC_LINK_AUTHEN_MITM_REQUESTED)
		{
			authenSettings |= DEVAUTH_SETTING_MITM_MASK;
		}
	}

	/* map BLE only ioCaps KEYBOARDDISPLAY to KEYBOARDONLY or DISPLAYONLY */
	if (pBtSM->devSecurity.io_capabilities == BLUEFACE_IOCAPA_KEYBOARDDISPLAY)
	{
		if (link && (link->mode & LINKMODE_IOCAPS_KNOWN) &&
		    (link->remote_ioCapability == BLUEFACE_IOCAPA_KEYBOARDONLY)
		   )
		{
			ioCaps = BLUEFACE_IOCAPA_DISPLAYONLY;
		}
		else
		{
			ioCaps = BLUEFACE_IOCAPA_KEYBOARDONLY;
		}
	}

	BTSECMAN_TRACE_PRINTF_4(BTSECMAN_TRACE_MASK_TRACE,
	                         "btsm: send IO_CAPABILITY_REPLY io:%d oob:%d auth:%d status:0x%x",
	                         ioCaps, pBtSM->devSecurity.oob_present,
	                         authenSettings, status
	                         );

	btsmSendLHciSspIoCapabilityReplyReq(bd,
	                                  ioCaps,
	                                  pBtSM->devSecurity.oob_present,
	                                  authenSettings,
	                                  status
	                                 );
} /* hciHandleIoCapabilityReq */

void btsmHandleHCI_IO_CAPABILITY_RESPONSE(uint8_t * bd, uint8_t ioCapability, uint8_t authRequirements )
{
	LPTSECLINK		 link	 = NULL;

	link = btsmFindLink(bd);

	/* we are informed regarding the io capabilities, MITM requirements and bonding mode of the remote side */
	BTSECMAN_TRACE_PRINTF_3(BTSECMAN_TRACE_MASK_TRACE,
							   "btsm: recvd IO_CAPABILITY_REPLY io:%d auth:%d status:0x%x",
							   ioCapability,
							   authRequirements,
							   0
							   );

	if (link)
	{
		link->mode				   |= LINKMODE_IOCAPS_KNOWN;
		link->remote_authRequirements = authRequirements;
		link->remote_ioCapability 	= ioCapability;
	}
} /* hciHandleIoCapabilityResp */

void btsmHandleHCI_USER_CONFIRMATION_REQUEST(uint8_t * bd, uint32_t value)
{
	LPTSECLINK		 link	 = NULL;

	link = btsmFindLink(bd);
	if (link)
	{
		if (btsmCheckForJustWorks(link))
		{
			/* we can reply this command locally, without interference / action from upper layer */
			btsmSendLHciUserConfReqResp( bd, 0 /* accept */);
		}
		else
		{
            blueAPI_Send_UserConfirmationReqInd(bd, value);
		}
	}
} /* hciHandleUserConfirmationReq */

void btsmHandleHCI_SIMPLE_PAIRING_COMPLETE(uint8_t * bd, uint16_t status)
{
	LPTSECLINK		 link	 = NULL;
	link = btsmFindLink(bd);

	btsmUpdateSecurityStatus(link, SECSTAT_SSP_COMPLETE, status);
	/* F_BT_SECMAN_STATUS_UPDATES */

	if (link)
	{
		if (status == 0)
		{
			link->mode |= LINKMODE_AUTHEN; /* link is authenticated now */
		}
		else
		{
			if (link->state == SEC_LINK_REMOTE_SSP)
			{
				/* remote SSP failed */
				link->state = SEC_LINK_IDLE;
				btsmDeferredQueueOut();
			}
		}
	}
} /* hciHandleSimplePairingComplete */

void btsmHandleCmdCompHCI_LINK_KEY_REQUEST_REPLY(uint16_t command, uint8_t * bd, uint16_t status)
{
	LPTSECLINK link = btsmFindLink(bd);
	if (command != HCI_LINK_KEY_REQUEST_REPLY)
	{
		status = HCI_ERR | HCI_ERR_KEY_MISSING;
	}
	btsmUpdateSecurityStatus(link, SECSTAT_LINK_KEY_REQ_RESP, status);
}

