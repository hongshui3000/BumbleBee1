/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       btsmmsg.c
* @brief     bt security manager msg handle
* @details   
*
* @author   	gordon
* @date      	2015-06-29
* @version	v0.1
*/

#include <flags.h>
#include <rtl_types.h>
#include <os_pool.h>
#include <message.h>
#include <btsm.h>
#include <btsmprot.h>
#include <btsm_api.h>
#include <hci_code.h>
#include <hci_api.h>
#include <gatt_api.h>
#include <blueapi_api.h>
#include <upper_stack_global.h>

#define TRACE_MODULE_ID     MID_BTSECMAN
/**
* @brief  update security status
*
* @param  link
* @param  secStat
* @param  cause
*
* @return  
*
*/
void btsmUpdateSecurityStatus(LPTSECLINK link, uint8_t secStat, uint16_t cause)
{
	BOOL sendResp = FALSE;

	if (link == NULL)
	{
		BTSECMAN_TRACE_PRINTF_0(BTSECMAN_TRACE_MASK_ERROR,
		                       "!!btsmUpdateSecurityStatus: link is NULL"
		                       );
		return;
	}

	switch (secStat)
	{
#if F_BT_BREDR_SUPPORT
	case SECSTAT_AUTH_REQ:          /* BR: outgoing auth */
	case SECSTAT_SSP_START:         /* BR: incoming SSP auth */
	case SECSTAT_LINK_KEY_REQ:      /* BR: incoming linkkey request */
	case SECSTAT_PIN_CODE_REQ:      /* BR: incoming legacy pairing  */
#endif
	case SECSTAT_LE_AUTH_REQ:       /* LE: local request */
	case SECSTAT_SMP_SECURITY_REQ:  /* LE: master incoming SMP pairing */
	case SECSTAT_SMP_PAIRING_REQ:   /* LE: slave incoming SMP pairing */
	case SECSTAT_LTK_REQ:           /* LE: slave incoming linkkey request */
		if (link->secStat == SECSTAT_IDLE)
		{
			link->secStat = secStat;
			btsmSendSECURITY_STATUS(CONF_SECURITY_STATUS_AUTH_STARTED,
			                                      link->bd, link->bdType, HCI_SUCCESS, 0, 0);
		}
		break;
#if F_BT_BREDR_SUPPORT
	case SECSTAT_AUTH_COMPLETE:
		if (link->secStat == SECSTAT_AUTH_REQ)
		{
			sendResp = TRUE; /* BR: outgoing auth completed */
		}
		break;

	case SECSTAT_SSP_COMPLETE:
		if (link->secStat == SECSTAT_SSP_START)
		{
			sendResp = TRUE; /* BR: incoming SSP auth completed */
		}
		break;

	case SECSTAT_LINK_KEY_REQ_RESP:
		if (link->secStat == SECSTAT_LINK_KEY_REQ)
		{
			sendResp = TRUE; /* BR: incoming linkkey request completed */
		}
		break;

	case SECSTAT_NEW_LINK_KEY:
		if (link->secStat == SECSTAT_PIN_CODE_REQ)
		{
			sendResp = TRUE; /* BR: incoming legacy pairing completed */
		}
		break;
#endif
	case SECSTAT_LE_KEYDIST_COMPLETE:
		if ((link->secStat == SECSTAT_SMP_PAIRING_REQ) ||
		  (link->secStat == SECSTAT_SMP_SECURITY_REQ) ||
		  (link->secStat == SECSTAT_LE_AUTH_REQ)
		 )
		{
			sendResp = TRUE; /* LE: pairing completed */
		}
		break;

	case SECSTAT_LE_ENCRYPTION_CHANGE:
		if ((link->secStat == SECSTAT_SMP_SECURITY_REQ) ||
		  (link->secStat == SECSTAT_LTK_REQ) ||
		  (link->secStat == SECSTAT_LE_AUTH_REQ)
		 )
		{
			sendResp = TRUE; /* LE: link key in use */
		}
		break;

	case SECSTAT_LINK_DISCONNECTED:
	case SECSTAT_ABORT:
		if (link->secStat != SECSTAT_IDLE)
		{
			sendResp = TRUE;
			cause = HCI_ERR | HCI_ERR_AUTHENTICATION_FAILED;
		}
		break;
	}

	if (sendResp)
	{
		link->secStat = SECSTAT_IDLE;		//randy 616
		if (cause == 0x00)
		{
			btsmSendSECURITY_STATUS(CONF_SECURITY_STATUS_AUTH_SUCCESS,
		                                    link->bd, link->bdType, HCI_SUCCESS,
		                                    link->keyType, link->keySize);
		}
		else
		{
			btsmSendSECURITY_STATUS(CONF_SECURITY_STATUS_AUTH_FAILED,
		                                    link->bd, link->bdType, cause, 0, 0);
		}
	}
}

/**
* @brief	 secmanager authentication complete
*
* @param	pLink: 
* @param	cause
*
* @return
*
*/
void btsmAuthenticationComplete(LPTSECLINK pLink, uint16_t cause)
{
	if (cause != SECMAN_SUCCESS)
	{
	    btsmUpdateSecurityStatus(pLink, SECSTAT_ABORT, cause);
	}
#if F_BT_BREDR_SUPPORT
	if (pLink->bdType == BLUEFACE_BDTYPE_BR_EDR)
	{
		if (pLink->mode & LINKMODE_POLICY_BASED)
		{
			btsmSendSECMAN_AUTHENTICATION_RESP(pLink->auth_source, pLink->bd, pLink->auth_ref,
		                                 pLink->auth_cid, pLink->auth_outg, (cause == SECMAN_SUCCESS));
		}
		else
		{
			blueAPI_Handle_BT_SECMAN_AUTH_CONF(pLink->bd,/* pLink->auth_ref,*/
		                                 pLink->keyType, pLink->keySize, cause);
		}
	}
#endif
	if (pLink->bdType & BLUEFACE_BDTYPE_LE_MASK)
	{
		if ((pLink->state == SEC_LINK_AUTHEN_REQUESTED) ||
		    (pLink->state == SEC_LINK_AUTHEN_MITM_REQUESTED)
		   )
		{
			blueAPI_Handle_BT_SECMAN_AUTH_CONF(pLink->bd,/* pLink->auth_ref,*/
			                                 pLink->keyType, pLink->keySize, cause);
		}

		btsmCancelSMPTimer(pLink);
		btsmDeAllocateSMPData(pLink);
	}

	/* Pairing complete, reset enforcement */
	/* TODO: only if status == success? */
	pLink->mode &= ~(LINKMODE_FORCE_PAIRING);
#if F_BT_BREDR_SUPPORT
	pLink->auth_source = 0x00;
#endif
	pLink->state       = SEC_LINK_IDLE;

	btsmDeferredQueueOut();
} /* btsmAuthenticationComplete */

/**
* @brief  blueface config securtiy status
*
* @param  indId: 
* @param  bd: 
* @param  bdType
* @param  cause
* @param  keyType
* @param  keySize
*
* @return  
*
*/
void btsmSendSECURITY_STATUS(uint8_t indId, uint8_t * bd, uint8_t bdType, uint16_t cause, uint8_t keyType, uint8_t keySize)
{
	blueAPI_Handle_SECURITY_STATUS(indId,bd, bdType, keyType, keySize);
	gattHandleBtsmSecurityStatus(indId,bd, bdType, cause, keyType, keySize);
}

/**
* @brief  security manager timer expired func
* 
* @param  pSetTimer
*
* @return  
*
*/
void btsmHandleTimerExpired(uint8_t TimerID, uint16_t TimerChannel)
{
	LPTSECLINK  pLink ;

	if(TimerChannel& 0x10)
		pLink     = &pBtSM->plinksDon[TimerChannel-0x10];
	else
	    pLink     = &pBtSM->plinksDoff[TimerChannel];


    BTSECMAN_TRACE_PRINTF_1(BTSECMAN_TRACE_MASK_ERROR, "btsmHandleTimerExpired timerID %d", TimerID);
	switch (TimerID)
	{
#if F_BT_BREDR_SUPPORT
	case SM_TIMER_AUTH_RETRY:
		{
			if (pLink->state == SEC_LINK_AUTHEN_RETRY)
			{
				btsmCheckSecurity(pLink);
			}
			break;
		}
#endif
	case SM_TIMER_SMP_TIMEOUT:
		{
			if ((pLink->used != SM_LINK_FREE) &&
			  (pLink->bdType & BLUEFACE_BDTYPE_LE_MASK) &&
			  (pLink->pSMPData != NULL)
			 )
			{
				pLink->pSMPData->smpState &= ~(LE_SMPSTATE_TIMEOUT_RUNNING);

				btsmAuthenticationComplete(pLink, HCI_ERR | HCI_ERR_TIMEOUT);

				/* Core4.0, Vol3, PartH, 3.4
				 * No further SMP commands shall be sent over the L2CAP Security Manager Channel.
				 * A new SM procedure shall only be performed when a new physical link has been
				 * established.
				 */
				btsmSendLHci_LE_DISCONNECT(pLink->handle, HCI_ERR_AUTHENTICATION_FAILED);
			}
			break;
		}

	default:
		break;
	}
}

void btsmHandleAuthenticationReq()
{
    PTSecManAuthenticationReq pAuthReq = &pBtSM->message.MData.secManAuthenticationReq;
    LPTSECLINK                pLink    = btsmFindLink(pAuthReq->bd);

    if (pLink == NULL)
    {
        blueAPI_Handle_BT_SECMAN_AUTH_CONF(pAuthReq->bd, 0, 0, HCI_ERR | HCI_ERR_REJECT_LIMITED_RESOURCES);
        return;
    }

    if (pLink->state != SEC_LINK_IDLE)
    {
        btsmDeferredQueueIn(pLink);
        return;
    }

    /* store values for CONF message */
#if F_BT_BREDR_SUPPORT
    pLink->auth_cid	  = 0;
    pLink->auth_uuid   = 0;
    pLink->auth_source = pAuthReq->Source;
    pLink->auth_outg   = 0;
#endif
    /* reset all requirements */
    pLink->mode &= ~(LINKMODE_POLICY_BASED	 |
                    LINKMODE_AUTHEN_REQUIRED | LINKMODE_MITM_REQUIRED |
                    LINKMODE_ENCR_REQUIRED	 | LINKMODE_AUTHOR_REQUIRED);

    /* MITM requirement */
    if ((pBtSM->devSecurity.authenSettings & DEVAUTH_SETTING_MITM_MASK) ||
        (pAuthReq->requirements & SECMAN_REQ_MITM)
        )
    {
        pLink->state = SEC_LINK_AUTHEN_MITM_REQUESTED;
        pLink->mode |= (LINKMODE_AUTHEN_REQUIRED | LINKMODE_MITM_REQUIRED | LINKMODE_ENCR_REQUIRED);
    }
    else
    {
        pLink->state = SEC_LINK_AUTHEN_REQUESTED;
        pLink->mode |= (LINKMODE_AUTHEN_REQUIRED | LINKMODE_ENCR_REQUIRED);
    }

    /* check if stored keys are allowed */
    if (pAuthReq->requirements & SECMAN_REQ_FORCE_PAIRING)
    {
        pLink->mode |= LINKMODE_FORCE_PAIRING;
    }
    else
    {
        pLink->mode &= ~(LINKMODE_FORCE_PAIRING);
    }

    /* minimum keysize */
    pLink->minKeySize = (pAuthReq->minKeySize == 0x00) ? pBtSM->LE_maxKeySize : pAuthReq->minKeySize;

    if (pLink->bdType & BLUEFACE_BDTYPE_LE_MASK)
    {
        btsmUpdateSecurityStatus(pLink, SECSTAT_LE_AUTH_REQ, HCI_SUCCESS);

        if (pLink->role == LE_ROLE_MASTER)
        {
            /* request link key from storage */
            btsmSendDEVICE_DATA_GET_IND(pLink, DEVICE_DATA_TYPE_SECMAN_LTK_REMOTE);
        }
        else
        {
        uint8_t authReq = btsmGetSMPAuthRequirement(pLink);

        if (pLink->pSMPData == NULL)
        {
            if (btsmAllocateSMPData(pLink) == NULL)
            {
                blueAPI_Handle_BT_SECMAN_AUTH_CONF(pAuthReq->bd,/* pAuthReq->ref, */0, 0, HCI_ERR | HCI_ERR_REJECT_LIMITED_RESOURCES);

                BTSECMAN_TRACE_PRINTF_1(BTSECMAN_TRACE_MASK_ERROR,
                                        "!!btsmHandleAuthenticationReq: failed to allocate SMPData for handle:0x%x",
                                        pLink->handle
                                        );
                return;
            }
        }

        btsmSendSMPSecurityRequest(pLink, authReq);
        }
    }
#if F_BT_BREDR_SUPPORT
    else
    {
        /* BR statemachine needs IDLE state */
        pLink->state = SEC_LINK_IDLE;
        btsmCheckSecurity(pLink);
    }
#endif
    return;
}


uint16_t btsmHandleConfigurationReq(uint16_t reqID, uint8_t * ptr, BOOL isSetBluetoothMode)
{
    uint16_t res = 0;  /* success return code by reasonable default*/
 
    switch (reqID)
    {
#if F_BT_BREDR_SUPPORT
    case BLUEFACE_CONF_SECURITY:
        {
            LPTSEC_CONF_SECURITY sec = (LPTSEC_CONF_SECURITY)ptr;
            if (sec->active)
            {
                if (btsmCheckNewSecEntry(sec))
                {
                    if (btsmAllocateSecEntry(sec) == NULL)
                    {
                        res = SECMAN_ERR | SECMAN_ERR_DBFULL;
                    }
                }
                else
                {
                    res = SECMAN_ERR | SECMAN_ERR_INVALID_PARAMETER;
                }
            }
            else
            {
                res = btsmDeAllocateSecEntry(sec);
            }
        }
        break;

    case BLUEFACE_CONF_AUTHORIZE:
        {
            LPTSEC_CONF_AUTHORIZE pAuth = (LPTSEC_CONF_AUTHORIZE)ptr;
            LPTSECLINK            link  = btsmFindLink(pAuth->bd);
            if (link)
            {
                if (pAuth->active)
                {
                    /* application does NOT authorize the connection */
                    btsmAuthenticationComplete(link, HCI_ERR | HCI_ERR_AUTHENTICATION_FAILED);
                }
                else
                {
                    /* application does authorize the connection, check for authentication */
                    link->mode |= LINKMODE_AUTHOR;
                    btsmCheckSecurity(link);
                }
            }
        }
        break;
#endif

    case BLUEFACE_CONF_DEVICE_SECURITY:
        memcpy(&pBtSM->devSecurity, ptr, sizeof(pBtSM->devSecurity));
#if F_BT_BREDR_SUPPORT
        if(isSetBluetoothMode)
        {
        	if (pBtSM->devSecurity.bt_mode >= 1)        
        	{
        		hciCommandByteParameter(HCI_WRITE_SIMPLE_PAIRING_MODE, 1);
        	}
            else
            {
                hciCommandByteParameter(HCI_WRITE_SIMPLE_PAIRING_MODE, 0);
            }
			hciLaterEntry();
        }
 #endif
        break;
#if F_BT_LE_BT41_SUPPORT
    case BLUEFACE_CONF_LE_SECURITY:
        {
            LPTSEC_CONF_LE_SECURITY sec = (LPTSEC_CONF_LE_SECURITY)ptr;
            if (sec->active)
            {
                if (btsmCheckNewLESecEntry(sec))
                {
                    if (btsmAllocateLESecEntry(sec) == NULL)
                    {
                        res = SECMAN_ERR | SECMAN_ERR_DBFULL;
                    }
                }
                else
                {
                    res = SECMAN_ERR | SECMAN_ERR_INVALID_PARAMETER;
                }
            }
            else
            {
                res = btsmDeAllocateLESecEntry(sec);
            }
        }
        break;
#endif
    case BLUEFACE_CONF_LE_SSP_PARAMETER:
        {
            LPSEC_CONF_LE_SSP_PARAMETER pSspParam = (LPSEC_CONF_LE_SSP_PARAMETER)ptr;

            if ((pSspParam->fixedDisplayValue == 0xFFFFFFFF) ||
                (pSspParam->fixedDisplayValue < 1000000)
                )
            {
                pBtSM->LE_fixedDisplayValue = pSspParam->fixedDisplayValue;
            }
            else
            {
                res = SECMAN_ERR | SECMAN_ERR_INVALID_PARAMETER;
            }
        }
        break;
    
    default:
        BTSECMAN_TRACE_PRINTF_1(BTSECMAN_TRACE_MASK_ERROR,
                               "!!btsmHandleConfigurationReq: not handled 0x%x",
                               reqID
                               );
        break;
    }

    return res;
}


void btsmHandleRemoteOOBDataReqConf(uint8_t * bd, uint8_t * C, uint8_t * R, uint8_t status)
{
	LPTSECLINK pLink = btsmFindLink(bd);
	
	if ((pLink != NULL) && (pLink->bdType & BLUEFACE_BDTYPE_LE_MASK))
	{
		if (pLink->pSMPData && (pLink->pSMPData->sspMech == LE_SMP_MECH_OUT_OF_BAND))
		{
			memcpy(pLink->pSMPData->p.ph2.local_TK, C, 16);

			if (status == 0x00)
			{
				/* start CONF value generation */
				pLink->cryptState = LE_CRYPT_GEN_RAND_LO;
				btsmSendLHci_LE_RAND(LE_LINK2HANDLE(pBtSM, pLink));
			}
			else
			{
				btsmSendSMPPairingFailed(pLink, LE_SMP_ERROR_OOB_NOT_AVAIABLE);

				btsmAuthenticationComplete(pLink, HCI_ERR | HCI_ERR_AUTHENTICATION_FAILED);
			}
		}
	}
#if F_BT_BREDR_SUPPORT
    else
	{
		if (status == 0)
		{
			hciCommandRemoteOOBDataRequestReply(bd,
												C,
												R);
		}
		else
		{
			hciCommandBDAddrParameter(HCI_REMOTE_OOB_DATA_REQUEST_NEGATIVE_REPLY,
									  bd);
		}
        hciLaterEntry();
	}
#endif	
}

void btsmHandleUserPasskeyReqConf(uint8_t * bd, uint8_t result, uint32_t value)
{
	LPTSECLINK pLink = btsmFindLink(bd);
	if ((pLink != NULL) && (pLink->bdType & BLUEFACE_BDTYPE_LE_MASK))
	{

	}
#if F_BT_BREDR_SUPPORT
    else
	{
		if (result != 0)
		{
			hciCommandBDAddrParameter(HCI_USER_PASSKEY_REQUEST_NEGATIVE_REPLY,
									  bd);
		}
	}
	hciLaterEntry();
#endif
}

void btsmHandleUserPasskeyReqReplyReq(uint8_t * bd, uint8_t result, uint32_t value)
{
	LPTSECLINK pLink = btsmFindLink(bd);
	
	if ((pLink != NULL) && (pLink->bdType & BLUEFACE_BDTYPE_LE_MASK))
	{
		uint16_t status = HCI_SUCCESS;		
		TBlueAPI_Cause param_cause = blueAPI_CauseSuccess;
		

		if ((pLink->pSMPData == NULL) || (pLink->pSMPData->sspMech != LE_SMP_MECH_KEYBOARD))
		{
			status = HCI_ERR | HCI_ERR_ILLEGAL_HANDLE;
		}
		else
		{
			LPLE_SMP_DATA_PHASE2 pSMPDataPh2 = &pLink->pSMPData->p.ph2;

			pSMPDataPh2->local_TK[0] = (uint8_t)(value & 0xFF);
			pSMPDataPh2->local_TK[1] = (uint8_t)((value >> 8) & 0xFF);
			pSMPDataPh2->local_TK[2] = (uint8_t)((value >> 16) & 0x0F);

			if (result == 0x00)
			{
				/* start CONF value generation */
				pLink->cryptState = LE_CRYPT_GEN_RAND_LO;
				btsmSendLHci_LE_RAND(LE_LINK2HANDLE(pBtSM, pLink));
			}
			else
			{
				btsmSendSMPPairingFailed(pLink, LE_SMP_ERROR_PASSKEY_ENTRY_FAILED);

				btsmAuthenticationComplete(pLink, HCI_ERR | HCI_ERR_AUTHENTICATION_FAILED);
			}
	  	}

		if (status != HCI_SUCCESS)
		{
			param_cause = blueAPI_CauseInvalidState;
		}

		blueAPI_Send_UserPasskeyReqReplyRsp(param_cause);
	}
#if F_BT_BREDR_SUPPORT
    else
	{
		if (result == 0)
		{
			hciCommandUserPasskeyRequestReply(bd, value);
		}
		else
		{
			hciCommandBDAddrParameter(HCI_USER_PASSKEY_REQUEST_NEGATIVE_REPLY, bd);
		}
	}
	hciLaterEntry();
#endif
}

void btsmUpdateLocalAddress(uint8_t * bd)
{
    memcpy(pBtSM->localBD, bd, BD_ADDR_SIZE);
}

void btsmHandleHciConfigurationInd()
{
  ThciConfigureInd * pHciConfigureInd = &pBtSM->message.MData.hciConfigureInd;
  THCI_CONF_NO_CONNECTIONS * pnoConnectionsInd = (THCI_CONF_NO_CONNECTIONS *) pHciConfigureInd->ptr;
  BOOL               release          = TRUE;

  switch (pHciConfigureInd->indID)
  {
  /* hci layer sends update regarding number of active ACL/SCO connections */
  case HCI_CONF_NO_CONNECTIONS:
    {
      /* link establishment / removal signalled from HCI layer */
      BTSECMAN_TRACE_PRINTF_1(BTSECMAN_TRACE_MASK_TRACE,
                                 "btsm: HCI_CONF_NO_CONNECTIONS indID:0x%x",
                                 pnoConnectionsInd->indId
                                 );

      switch (pnoConnectionsInd->indId)
      {
      case NEW_LE_CONNECTION:
      case CLEAR_LE_CONNECTION:
        {
          uint8_t state = btsmResolveRandomAddress(pnoConnectionsInd->bd, &pnoConnectionsInd->bdType);

          /* queue all connect complete events */
          if (state & LE_RESOLVE_ENTRY_STATE_RESOLVING_MASK)
          {
            release = FALSE;
            btsmDeferredQueueIn(NULL);
          }
          else
	      {
#if (BLUEAPI_ENABLE_ACL_INFO & BLUE_API_ENABLE_ACL_INFO_CONNECTED_ACTIVE)	
            blueAPI_Send_ACLStatusInfo(pnoConnectionsInd->bd,
						   (TBlueAPI_RemoteBDType)pnoConnectionsInd->bdType,
						   (pnoConnectionsInd->indId == NEW_LE_CONNECTION ?blueAPI_ACLConnectedActive : blueAPI_ACLConnectionDisconnected),
						   NULL);
#endif
	      }
        }
        break;
      case CLEAR_ALL_CONNECTIONS:
        /* free all link descriptors */
        btsmDeAllocateLink(NULL);
        break;
#if F_BT_BREDR_SUPPORT
      case CLEAR_ACL_CONNECTION:
        {
          LPTSECLINK link;
          BOOL       forward = TRUE;
          uint8_t       state = btsmResolveRandomAddress(pnoConnectionsInd->bd, &pnoConnectionsInd->bdType);

          /* queue all connect complete events */
          if (state & LE_RESOLVE_ENTRY_STATE_RESOLVING_MASK)
          {
            release = FALSE;
            btsmDeferredQueueIn(NULL);
            break;
          }
/* (F_BT_LE_PRIVACY_RESOLVING) */

          link = btsmFindLink(pnoConnectionsInd->bd);
          if (link)
          {
            if (link->state == SEC_LINK_AUTHOR_REQUESTED)
            {
              blueAPI_Handle_BLUEFACE_CONF_AUTHORIZE(link);
              link->state = SEC_LINK_IDLE;
            }

            /* only forward if a NEW_ACL_CONNECTION was sent */
            if (!(link->mode & LINKMODE_SSP_KNOWN))
            {
              forward = FALSE;
            }

            btsmUpdateSecurityStatus(link, SECSTAT_LINK_DISCONNECTED, HCI_SUCCESS);
/* (F_BT_SECMAN_STATUS_UPDATES) */
            btsmDeAllocateLink(link);
          }
          else
          {
            forward = FALSE;
          }
          if(forward)
	      {
#if (BLUEAPI_ENABLE_ACL_INFO & BLUE_API_ENABLE_ACL_INFO_CONNECTION_DISCONNECTED)	
		    blueAPI_Send_ACLStatusInfo(pnoConnectionsInd->bd,
								   (TBlueAPI_RemoteBDType)pnoConnectionsInd->bdType,
								   blueAPI_ACLConnectionDisconnected,
								   NULL);
#endif		
	        }
          
        } /* CLEAR_xxx_CONNECTION */
        break;
#endif
      default:
        break;
        } /* switch */
      } /* HCI_CONF_NO_CONNECTIONS */
      break;


  default:
/* it is perfectly ok that this particular message is not handled here, it might be handled elsewhere */
    break;
  } /* switch */
  
  if (release)
  {
    osBufferRelease(pHciConfigureInd->ptr);
  }
} /* btsmHandleConfigurationInd */

void btsmSendMsgAuthenticationReq(uint8_t * bd, uint16_t requirements, uint8_t minKeySize)
{
    MESSAGE_T msg;
    TSecManAuthenticationReq * pSecManAuthReq =
                             (TSecManAuthenticationReq *) msg.MData.MessageData;

    pSecManAuthReq->requirements = requirements;
    pSecManAuthReq->minKeySize   = minKeySize;
    pSecManAuthReq->Source       = SECMAN_SOURCE_GATT;
    memcpy( &pSecManAuthReq->bd, bd, sizeof(TBdAddr));

    msg.Command = SECMAN_AUTHENTICATION_REQ;

    osMessageSend(btsmQueueID, &msg);    
}

void btsmSendMsgHciConfigureInd(uint16_t code, uint16_t len, uint8_t * ptr)
{
    MESSAGE_T lmsg;
    ThciConfigureInd * confInd      = (ThciConfigureInd *) lmsg.MData.MessageData;

    confInd->indID                  = code;
    confInd->len                    = len;
    confInd->ptr                    = ptr;
    lmsg.Command                    = HCI_CONFIGURATION_IND;

    osMessageSend(btsmQueueID, &lmsg);
/* F_BT_SECMAN */
}

void btsmSendMsgAuthenticationInd( TBdAddr bd,
                                                uint16_t    ref,
                                                uint16_t    channelId, /* L2CAP: PSM, RFCOMM: DLCI */
                                                uint16_t    uuid,
                                                uint8_t    outgoing,  /* ignore if deactivate */
                                                uint8_t    active,
                                                uint8_t    Source,
                                                uint8_t    conType)
{
    MESSAGE_T msg;
    TSecManAuthenticationInd *pSecManAuthenticationInd = (TSecManAuthenticationInd *)&msg.MData.secManAuthenticationInd;

    msg.Command = SECMAN_AUTHENTICATION_IND;

    memcpy(pSecManAuthenticationInd->bd, bd, BD_ADDR_SIZE);        /* get remote adress */
    pSecManAuthenticationInd->active    = active;
    pSecManAuthenticationInd->outgoing  = outgoing;
    pSecManAuthenticationInd->channelId = channelId;
    pSecManAuthenticationInd->uuid      = uuid;
    pSecManAuthenticationInd->ref       = ref;
    pSecManAuthenticationInd->Source    = Source;
    pSecManAuthenticationInd->conType   = conType;

    osMessageSend(btsmQueueID, &msg);

    BTSECMAN_TRACE_PRINTF_6(BTSECMAN_TRACE_MASK_TRACE,
                                  "L2C<- SECMAN_AUTHENTICATION_IND bd %s CID %x PSM %x UUID %x outgoing %d active %d",
                                  TRACE_BDADDR1(BTSECMAN_TRACE_MASK_TRACE, pSecManAuthenticationInd->bd),
                                  pSecManAuthenticationInd->ref,
                                  pSecManAuthenticationInd->channelId,
                                  pSecManAuthenticationInd->uuid,
                                  pSecManAuthenticationInd->outgoing,
                                  pSecManAuthenticationInd->active);

}
//fixme
void btsmHandleKeypressNotificationReq(uint8_t * bd, uint8_t type)
{
	LPTSECLINK pLink = btsmFindLink(bd);
	TBlueAPI_Cause cause = blueAPI_CauseSuccess;
	
	if ((pLink != NULL) && (pLink->bdType & BLUEFACE_BDTYPE_LE_MASK))
	{
		uint16_t status = HCI_SUCCESS;

		if ((pLink->pSMPData == NULL) || (pLink->pSMPData->sspMech != LE_SMP_MECH_KEYBOARD))
		{
			status = HCI_ERR | HCI_ERR_ILLEGAL_HANDLE;
		}
		if (status != HCI_SUCCESS)
		{
			cause = blueAPI_CauseInvalidState;
		}

		blueAPI_Send_KeypressNotificationRsp(cause);

	}
#if F_BT_BREDR_SUPPORT
    else
	{
		hciCommandBDAddrByteParameter(HCI_SEND_KEYPRESS_NOTIFICATION,
									  bd,
									  type);
	}

	hciLaterEntry();
#endif
}

void btsmHandleMessage()
{
    BOOL releaseBuffer = FALSE;

    switch (pBtSM->message.Command)
    {
    case LE_MESSAGE_IND:
      releaseBuffer = btsmHandleLE_MESSAGE_IND();
      break;
        /* from HCI layer */
    case HCI_CONFIGURATION_IND:
        btsmHandleHciConfigurationInd();
        break;
#if F_BT_BREDR_SUPPORT
    case HCI_AUTH_REQ:
        btsmHandleHciAuthReq();
        break;
#endif        
    case SECMAN_AUTHENTICATION_IND:
        /* protocol layer requests authentication */
        {
            PTSecManAuthenticationInd auth = &pBtSM->message.MData.secManAuthenticationInd;
#if F_BT_BREDR_SUPPORT
            if(auth->conType == BLUEFACE_CON_TYPE_BR_EDR)
            {
                btsmHandleAuthenticationInd();
            }
#endif
#if F_BT_LE_BT41_SUPPORT
            if(auth->conType == BLUEFACE_CON_TYPE_LE)
            {
                btsmLEHandleAuthenticationInd();
            }
#endif
        }
        break;
    case SECMAN_AUTHENTICATION_REQ:
        btsmHandleAuthenticationReq();
        break;
    default:
      BTSECMAN_TRACE_PRINTF_1(BTSECMAN_TRACE_MASK_ERROR,
                                 "!!btsmHandleMessage: not handled 0x%x",
                                 pBtSM->message.Command
                                 );
      break;
    }

    if (releaseBuffer && (pBtSM->message.MData.DataCB.Flag & DATA_CB_RELEASE))
    {
        osBufferRelease((PVOID)pBtSM->message.MData.DataCB.BufferAddress);
    }
}


#if !F_BT_BREDR_SUPPORT
void btsmSendLHciUserConfReqResp              (uint8_t * bd, uint8_t result){}
uint16_t btsmHandleBT_HCI_KEY_RESP(LPCBYTE bd, uint8_t reqType, uint16_t cause, LPCBYTE key, uint16_t keyLen)
{
    return 0;
}
void btsmSendMsgHciAuthReq(LPCBYTE bd, uint8_t id){}
#endif

