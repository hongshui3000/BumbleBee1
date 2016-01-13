/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       btsm_smp.c
* @brief     Security Manager Protocol
* @details   
*
* @author   	gordon
* @date      	2015-07-09
* @version	v0.1
*/

#include <flags.h>
#include <rtl_types.h>
#include <os_message.h>
#include <os_pool.h>
#include <os_sched.h>
#include <message.h>
#include <btsm.h>
#include <btsmprot.h>
#include <btcommon.h>
#include <l2c_api.h>
#include <blueapi_api.h>
#include <upper_stack_global.h>

#define TRACE_MODULE_ID     MID_BTSECMAN
/**
* @brief  btsm get ssp mechanism
*
* @param  init_IOCap
* @param  resp_IOCap
* @param  isInitiator
*
* @return  
*
*/
uint8_t btsmLEGetSSPMechanism(uint8_t init_IOCap, uint8_t resp_IOCap, BOOL isInitiator)
{
	/* see Core4.0, Vol3, PartH, 2.3.5.1 */
	switch (init_IOCap)
	{
	case LE_SMP_IOCAP_DISPLAY_ONLY: 
	case LE_SMP_IOCAP_DISPLAY_YES_NO:
		if (resp_IOCap == LE_SMP_IOCAP_KEYBOARD_ONLY ||
		  resp_IOCap == LE_SMP_IOCAP_KEYBOARD_DISPLAY)
		{
			return (isInitiator) ? LE_SMP_MECH_DISPLAY : LE_SMP_MECH_KEYBOARD;
		}
		break;

	case LE_SMP_IOCAP_KEYBOARD_ONLY:
		if (resp_IOCap == LE_SMP_IOCAP_DISPLAY_ONLY ||
		  resp_IOCap == LE_SMP_IOCAP_DISPLAY_YES_NO ||
		  resp_IOCap == LE_SMP_IOCAP_KEYBOARD_DISPLAY)
		{
			return (isInitiator) ? LE_SMP_MECH_KEYBOARD : LE_SMP_MECH_DISPLAY;
		}
		else if (resp_IOCap == LE_SMP_IOCAP_KEYBOARD_ONLY)
		{
			return LE_SMP_MECH_KEYBOARD; /* both keyboard */
		}
		break;

	case LE_SMP_IOCAP_KEYBOARD_DISPLAY:
		if (resp_IOCap == LE_SMP_IOCAP_DISPLAY_ONLY ||
		  resp_IOCap == LE_SMP_IOCAP_DISPLAY_YES_NO)
		{
			return (isInitiator) ? LE_SMP_MECH_KEYBOARD : LE_SMP_MECH_DISPLAY;
		}
		else if (resp_IOCap == LE_SMP_IOCAP_KEYBOARD_ONLY ||
		       resp_IOCap == LE_SMP_IOCAP_KEYBOARD_DISPLAY)
		{
			return (isInitiator) ? LE_SMP_MECH_DISPLAY : LE_SMP_MECH_KEYBOARD;
		}
		break;

	default:
		break;
	}

	return LE_SMP_MECH_JUST_WORKS; /* Just Works */
}

/**
* @brief  btsm check all keys received
*
* @param  pLink
* @param  keyDist
*
* @return  
*
*/
BOOL btsmLECheckAllKeysReceived(LPTSECLINK pLink, uint8_t keyDist)
{
	LPLE_SMP_DATA pSMPData = pLink->pSMPData;

	/* check that peer transmitted all keys needed for encryption */
	if (keyDist & LE_SMP_KEYDIST_ENCKEY)
	{
		if (!(pSMPData->smpState & LE_SMPSTATE_ENCRYPT_INFO_RECVD) ||
		    !(pSMPData->smpState & LE_SMPSTATE_MASTER_ID_RECVD)
		   )
		{
			return FALSE;
		}
	}

	/* check that peer transmitted all keys needed for privacy mode */
	if (keyDist & LE_SMP_KEYDIST_IDKEY)
	{
		if (!(pSMPData->smpState & LE_SMPSTATE_IDENT_INFO_RECVD) ||
		    !(pSMPData->smpState & LE_SMPSTATE_IDENT_ADDR_RECVD)
		   )
		{
			return FALSE;
		}
	}

	return TRUE;
}

/**
* @brief  btsm LE key exchange
*
* @param  pLink
*
* @return  
*
*/
void btsmLEKeyExchange(LPTSECLINK pLink)
{
	LPLE_SMP_DATA pSMPData    = pLink->pSMPData;
	uint8_t          sendKeyDist;
	uint8_t          recvKeyDist;
	BOOL          retry;

	// TODO: check that link is encrypted / check pLink->state SEC_LINK_* if keyexchange is needed now

	if (pLink->role == LE_ROLE_SLAVE)
	{
		sendKeyDist = pSMPData->remote_respKeyDist;
		recvKeyDist = pSMPData->remote_initKeyDist;
	}
	else
	{
		sendKeyDist = pSMPData->remote_initKeyDist;
		recvKeyDist = pSMPData->remote_respKeyDist;
	}

	BTSECMAN_TRACE_PRINTF_5(BTSECMAN_TRACE_MASK_TRACE,
	                         "btsmLEKeyExchange: smpState:0x%x dist:0x%x/0x%x store:0x%x/0x%x",
	                         pSMPData->smpState,
	                         sendKeyDist, recvKeyDist,
	                         pSMPData->p.ph3.localKeysStored,
	                         pSMPData->p.ph3.remoteKeysStored
	                         );
	do
	{
		retry = FALSE;

		/* wait for datastore */
		if (pSMPData->smpState & LE_SMPSTATE_DEVICE_DATA_STORE)
		{
			return;
		}

		/* Wait for IRK from peer (use real BD for DATA_SET_INDs) */
		if (!(recvKeyDist & LE_SMP_KEYDIST_IDKEY) ||
		((pSMPData->smpState & LE_SMPSTATE_IDENT_INFO_RECVD) &&
		 (pSMPData->smpState & LE_SMPSTATE_IDENT_ADDR_RECVD)
		))
		/* (F_BT_LE_PRIVACY_RESOLVING) */
		{
			/* store local LTK */
			if ((pSMPData->smpState & LE_SMPSTATE_ENCRYPT_INFO_SENT) &&
			  (pSMPData->smpState & LE_SMPSTATE_MASTER_ID_SENT) &&
			  !(pSMPData->p.ph3.localKeysStored & LE_SMP_KEYDIST_ENCKEY)
			 )
			{
				btsmSendDEVICE_DATA_SET_IND(pLink, DEVICE_DATA_TYPE_SECMAN_LTK_LOCAL,
				                            (uint8_t *)&pSMPData->p.ph3.lLTK,
				                            sizeof(TDEVICE_DATA_ELEMENT_SECMAN_LTK)
				                            );

				pSMPData->smpState |= LE_SMPSTATE_DEVICE_DATA_STORE;
			}
			/* store remote LTK */
			else if ((pSMPData->smpState & LE_SMPSTATE_ENCRYPT_INFO_RECVD) &&
			       (pSMPData->smpState & LE_SMPSTATE_MASTER_ID_RECVD) &&
			       !(pSMPData->p.ph3.remoteKeysStored & LE_SMP_KEYDIST_ENCKEY)
			      )
			{
				btsmSendDEVICE_DATA_SET_IND(pLink, DEVICE_DATA_TYPE_SECMAN_LTK_REMOTE,
				                            (uint8_t *)&pSMPData->p.ph3.rLTK,
				                            sizeof(TDEVICE_DATA_ELEMENT_SECMAN_LTK)
				                            );

				pSMPData->smpState |= LE_SMPSTATE_DEVICE_DATA_STORE;
			}
			/* store remote IRK */
			else if ((pSMPData->smpState & LE_SMPSTATE_IDENT_INFO_RECVD) &&
			       (pSMPData->smpState & LE_SMPSTATE_IDENT_ADDR_RECVD) &&
			       !(pSMPData->p.ph3.remoteKeysStored & LE_SMP_KEYDIST_IDKEY)
			      )
			{
				btsmSendDEVICE_DATA_SET_IND(pLink, DEVICE_DATA_TYPE_SECMAN_IRK_REMOTE,
				                            (uint8_t *)&pSMPData->p.ph3.rIRK,
				                            sizeof(TDEVICE_DATA_ELEMENT_SECMAN_IRK)
				                            );

				pSMPData->smpState |= LE_SMPSTATE_DEVICE_DATA_STORE;
			}
		}

		/* wait for datastore */
		if (pSMPData->smpState & LE_SMPSTATE_DEVICE_DATA_STORE)
		{
			return;
		}

		/* wait until slave sent all requested keys */
		if (pLink->role == LE_ROLE_MASTER &&
		    !btsmLECheckAllKeysReceived(pLink, recvKeyDist)
		   )
		{
			return;
		}
		/* Encryption Key Requested */
		else if ((sendKeyDist & LE_SMP_KEYDIST_ENCKEY) &&
		    !(pSMPData->smpState & LE_SMPSTATE_ENCRYPT_INFO_SENT)
		   )
		{
			/* start LTK generation */
			pLink->cryptState = LE_CRYPT_GEN_LTK_LO;
			btsmSendLHci_LE_RAND(LE_LINK2HANDLE(pBtSM, pLink));
		}
		else if ((sendKeyDist & LE_SMP_KEYDIST_ENCKEY) &&
		     !(pSMPData->smpState & LE_SMPSTATE_MASTER_ID_SENT)
		    )
		{
			/* start RAND/EDIV generation */
			pLink->cryptState = LE_CRYPT_GEN_MA_ID_LO;
			btsmSendLHci_LE_RAND(LE_LINK2HANDLE(pBtSM, pLink));
		}
		else
		{
			/* wait until master sent all requested keys */
			if (pLink->role == LE_ROLE_SLAVE &&
			  !btsmLECheckAllKeysReceived(pLink, recvKeyDist)
			 )
			{
				return;
			}

			btsmUpdateSecurityStatus(pLink, SECSTAT_LE_KEYDIST_COMPLETE, SECMAN_SUCCESS);
			btsmAuthenticationComplete(pLink, SECMAN_SUCCESS);
		}
	} while (retry);
}

/**
* @brief  btsm handle SMP pairing exchange
*
* @param  pLink
* @param  pLEMsg
*
* @return  
*
*/
uint8_t btsmHandleSMPPairingExchange(LPTSECLINK pLink, LPLEMessage pLEMsg)
{
	LPLE_SMP_DATA pSMPData    = pLink->pSMPData;
	uint16_t          pos         = 0;
	uint8_t          command     = pLEMsg->p.Data[pos++];
	uint8_t          ioCap       = pLEMsg->p.Data[pos++];
	uint8_t          oobFlag     = pLEMsg->p.Data[pos++];
	uint8_t          authReq     = pLEMsg->p.Data[pos++];
	uint8_t          maxKeySize  = pLEMsg->p.Data[pos++];
	uint8_t          initKeyDist = pLEMsg->p.Data[pos++];
	uint8_t          respKeyDist = pLEMsg->p.Data[pos++];
	uint16_t          keyDist     = btsmGetSMPKeyDistribution(pLink);

	BTSECMAN_TRACE_PRINTF_7(BTSECMAN_TRACE_MASK_TRACE,
	                         "btsmHandleSMPPairing: bd:%s io:%x oob:%x auth:%x maxkey:%x iKeyDist:%x rKeyDist:%x",
	                         //(command == LE_SMP_PAIRING_REQUEST ? pRequest : pResponse),
	                         TRACE_BDADDR1(BTSECMAN_TRACE_MASK_TRACE, pLink->bd),
	                         ioCap, oobFlag, authReq,
	                         maxKeySize, initKeyDist, respKeyDist
	                         );

	if (pLink->role == LE_ROLE_MASTER)
	{
		/* requests or invalid ordered responses are not allowed on master side */
		if ((command != LE_SMP_PAIRING_RESPONSE) ||
		    !(pSMPData->smpState & LE_SMPSTATE_PAIRING_REQ_SENT) ||
		    (pSMPData->smpState & LE_SMPSTATE_PAIRING_RSP_RECVD)
		   )
		{
			return LE_SMP_ERROR_UNSPECIFIED_REASON;
		}

		/* store PairingResponse for CONF value generation */
		memcpy(pSMPData->pairingRsp, pLEMsg->p.Data, LE_SMP_PAIRING_RESPONSE_LENGTH);
		pSMPData->smpState |= LE_SMPSTATE_PAIRING_RSP_RECVD;
	}
	else
	{
		/* responses or multiple requests are not allowed on slave side */
		if ((command != LE_SMP_PAIRING_REQUEST) ||
		    (pSMPData->smpState & LE_SMPSTATE_PAIRING_REQ_RECVD)
		   )
		{
			return LE_SMP_ERROR_UNSPECIFIED_REASON;
		}

		btsmUpdateSecurityStatus(pLink, SECSTAT_SMP_PAIRING_REQ, HCI_SUCCESS);

		/* store PairingRequest for CONF value generation */
		memcpy(pSMPData->pairingReq, pLEMsg->p.Data, LE_SMP_PAIRING_REQUEST_LENGTH);
		pSMPData->smpState |= LE_SMPSTATE_PAIRING_REQ_RECVD;

		if (pLink->state == SEC_LINK_IDLE)
		{
			pLink->state = SEC_LINK_PAIRING_REQ;
			pLink->mode |= LINKMODE_FORCE_PAIRING | LINKMODE_AUTHEN_REQUIRED | LINKMODE_ENCR_REQUIRED;
		}
	}

	pSMPData->remote_initKeyDist = initKeyDist;
	pSMPData->remote_respKeyDist = respKeyDist;

	/* own device must be pairable */
	if (!pBtSM->devSecurity.pairable_mode)
	{
		return LE_SMP_ERROR_PAIRING_NOT_SUPPORTED; /* Core4.0, Vol3, PartH, 2.3 */
	}

	/* default to just works */
	pSMPData->sspMech = LE_SMP_MECH_JUST_WORKS;

	/* if both devices have oob data, then ignore authrequirements and use oob */
	if ((oobFlag == LE_SMP_OOB_DATA_PRESENT) && (pBtSM->devSecurity.oob_present))
	{
		pSMPData->sspMech = LE_SMP_MECH_OUT_OF_BAND;
	}
	/* check if MITM is required and possible with ioCap settings */
	else if ((pLink->mode & LINKMODE_MITM_REQUIRED) ||
	       (pBtSM->devSecurity.authenSettings & DEVAUTH_SETTING_MITM) ||
	       (authReq & LE_SMP_AUTHREQ_MITM)
	      )
	{
#if (X_BT_LE_STRICT_OOB_HANDLING)
		/* if master has oob data, but slave has not, the slave MAY respond with PairingFailed */
		if ((pLink->role == LE_ROLE_SLAVE) &&
		    (oobFlag == LE_SMP_OOB_DATA_PRESENT) &&
		    !(pBtSM->devSecurity.oob_present)
		   )
		{
			return LE_SMP_ERROR_OOB_NOT_AVAIABLE; /* Core 4.0, Vol3, PartH, 2.3.5.1 */
		}
#endif  /* (X_BT_LE_STRICT_OOB_HANDLING) */
		if (pLink->role == LE_ROLE_MASTER)
		{
			pSMPData->sspMech = btsmLEGetSSPMechanism(pBtSM->devSecurity.io_capabilities, ioCap, TRUE);
		}
		else
		{
			pSMPData->sspMech = btsmLEGetSSPMechanism(ioCap, pBtSM->devSecurity.io_capabilities, FALSE);
		}
	}

	/* check against minimum keysize */
	if (maxKeySize < pLink->minKeySize)
	{
		return LE_SMP_ERROR_ENCRYPTION_KEY_SIZE; /* Core 4.0, Vol3, PartH, 2.3.4 */
	}

	/* store max possible keysize */
	pLink->keySize = pBtSM->LE_maxKeySize;
	if (maxKeySize < pLink->keySize)
	{
		pLink->keySize = maxKeySize;
	}

	/* check that no keys are requested if remote side is non-bondable */
	if (((authReq & LE_SMP_AUTHREQ_BONDING_MASK) == 0x00) &&
	  ((pSMPData->remote_initKeyDist != 0) || (pSMPData->remote_respKeyDist != 0))
	 )
	{
		BTSECMAN_TRACE_PRINTF_3(BTSECMAN_TRACE_MASK_ERROR,
		                           "!!btsmHandleSMPPairingExchange: invalid keydist authReq:0x%x init:0x%x resp:0x%x",
		                           authReq,
		                           pSMPData->remote_initKeyDist,
		                           pSMPData->remote_respKeyDist
		                           );
		return LE_SMP_ERROR_INVALID_PARAMETERS; /* Core4.0, Vol3, PartC, 9.4.2.2 */
	}

	if (pLink->role == LE_ROLE_MASTER)
	{
		/* check only offered keys are requested */
		if ((~(keyDist >> 8) & pSMPData->remote_initKeyDist) ||
		    (~(keyDist & 0xFF) & pSMPData->remote_respKeyDist)
		   )
		{
			BTSECMAN_TRACE_PRINTF_4(BTSECMAN_TRACE_MASK_ERROR,
			                         "!!btsmHandleSMPPairingExchange: invalid keydist local:0x%x/0x%x remote:0x%x/0x%x",
			                         (keyDist >> 8), (keyDist & 0xFF),
			                         pSMPData->remote_initKeyDist,
			                         pSMPData->remote_respKeyDist
			                         );
			return LE_SMP_ERROR_INVALID_PARAMETERS; /* Core4.0, Vol3, PartH, 2.3 */
		}
	}
	else
	{
		/* key distribution: only set bits that the master offers */
		pSMPData->remote_initKeyDist &= (keyDist >> 8);
		pSMPData->remote_respKeyDist &= (keyDist & 0xFF);

		btsmSendSMPPairingExchange(pLink, LE_SMP_PAIRING_RESPONSE,
		                           pBtSM->devSecurity.io_capabilities,
		                           pBtSM->devSecurity.oob_present,
		                           btsmGetSMPAuthRequirement(pLink),
		                           pBtSM->LE_maxKeySize,
		                           pSMPData->remote_initKeyDist,
		                           pSMPData->remote_respKeyDist
		                           );
		btsmStartSMPTimer(pLink);
	}

	switch (pSMPData->sspMech)
	{
	case LE_SMP_MECH_DISPLAY:
		/* generate 6digit TK and display as passkey */
		pLink->cryptState = LE_CRYPT_GEN_PASSKEY;
		btsmSendLHci_LE_RAND(LE_LINK2HANDLE(pBtSM, pLink));
		break;

	case LE_SMP_MECH_KEYBOARD: 
		/* start keyboard entry (get TK from keyboard) */
		blueAPI_Send_UserPasskeyReqInd(pLink->bd);
		break;

	case LE_SMP_MECH_OUT_OF_BAND: 
		/* if both device have oob data, use oob as TK value */
        blueAPI_Send_RemoteOOBDataReqInd(pLink->bd);
		break;

	default: 
	case LE_SMP_MECH_JUST_WORKS: 
		/* use 0x00 as TK */
		memset(pSMPData->p.ph2.local_TK, 0x00, LE_ENCRYPT_KEY_LENGTH);

		/* start CONF value generation */
		pLink->cryptState = LE_CRYPT_GEN_RAND_LO;
		btsmSendLHci_LE_RAND(LE_LINK2HANDLE(pBtSM, pLink));
		break;
	}

	return LE_SMP_ERROR_SUCCESS;
} /* btsmHandleSMPPairingExchange */

/**
* @brief  btsm handle smp pairing confirm
* 
* @param  pLink
* @param  pLEMsg
*
* @return 
*
*/
uint8_t btsmHandleSMPPairingConfirm(LPTSECLINK pLink, LPLEMessage pLEMsg)
{
	LPLE_SMP_DATA pSMPData = pLink->pSMPData;
	uint8_t *        pConf    = &pLEMsg->p.Data[1];

	BTSECMAN_TRACE_PRINTF_2(BTSECMAN_TRACE_MASK_TRACE,
	                         "btsmHandleSMPPairingConfirm: bd:%s conf:%s",
	                         TRACE_BDADDR1(BTSECMAN_TRACE_MASK_TRACE, pLink->bd),
	                         BTRACE_RAMDATA1(BTSECMAN_TRACE_MASK_TRACE,btsmHexString(pConf, 16))
	                         );

	/* invalid if not the first PairingConf, or out of order */
	if ((pSMPData->smpState & LE_SMPSTATE_PAIRING_CONF_RECVD) ||
	  ((pLink->role == LE_ROLE_MASTER) && !(pSMPData->smpState & LE_SMPSTATE_PAIRING_CONF_SENT)) ||
	  ((pLink->role == LE_ROLE_SLAVE) && !(pSMPData->smpState & LE_SMPSTATE_PAIRING_RSP_SENT))
	 )
	{
		return LE_SMP_ERROR_UNSPECIFIED_REASON;
	}

	memcpy(pSMPData->p.ph2.remote_CONF, pConf, 16);
	pSMPData->smpState |= LE_SMPSTATE_PAIRING_CONF_RECVD;

	/* Master: send RAND value right away */
	if (pLink->role == LE_ROLE_MASTER)
	{
		btsmSendSMP128BitValue(pLink, LE_SMP_PAIRING_RANDOM, pSMPData->p.ph2.local_RAND);
	}
	/* Slave: send CONF value when ready */
	else if (pSMPData->smpState & LE_SMPSTATE_LOCAL_CONF_VALID)
	{
		btsmSendSMP128BitValue(pLink, LE_SMP_PAIRING_CONFIRM, pSMPData->p.ph2.local_CONF);
	}

	btsmStartSMPTimer(pLink);
	return LE_SMP_ERROR_SUCCESS;
} /* btsmHandleSMPPairingConfirm */

/**
* @brief  btsm handle smp pairing random
* 
* @param  pLink
* @param  pLEMsg
*
* @return 
*
*/
uint8_t btsmHandleSMPPairingRandom(LPTSECLINK pLink, LPLEMessage pLEMsg)
{
	LPLE_SMP_DATA pSMPData = pLink->pSMPData;
	uint8_t *        pRand    = &pLEMsg->p.Data[1];
	uint8_t          temp[LE_ENCRYPT_DATA_LENGTH];

	BTSECMAN_TRACE_PRINTF_2(BTSECMAN_TRACE_MASK_TRACE,
	                         "btsmHandleSMPPairingRandom: bd:%s rand:%s",
	                         TRACE_BDADDR1(BTSECMAN_TRACE_MASK_TRACE, pLink->bd),
	                         BTRACE_RAMDATA1(BTSECMAN_TRACE_MASK_TRACE, btsmHexString(pRand, 16))
	                         );

	/* invalid if not the first PairingConf, or out of order */
	if ((pSMPData->smpState & LE_SMPSTATE_PAIRING_RAND_RECVD) ||
	  ((pLink->role == LE_ROLE_MASTER) && !(pSMPData->smpState & LE_SMPSTATE_PAIRING_RAND_SENT)) ||
	  ((pLink->role == LE_ROLE_SLAVE) && !(pSMPData->smpState & LE_SMPSTATE_PAIRING_CONF_SENT))
	 )
	{
		return LE_SMP_ERROR_UNSPECIFIED_REASON;
	}

	memcpy(pSMPData->p.ph2.remote_RAND, pRand, 16);
	pSMPData->smpState |= LE_SMPSTATE_PAIRING_RAND_RECVD;

	btsmSetupLECryptoConfirmValue1(pLink, temp, pSMPData->p.ph2.remote_RAND);

	pLink->cryptState = LE_CRYPT_CHK_CONF_1;
	btsmSendLHci_LE_ENCRYPT(LE_LINK2HANDLE(pBtSM, pLink), pSMPData->p.ph2.local_TK, temp);
	return LE_SMP_ERROR_SUCCESS;
} /* btsmHandleSMPPairingRandom */

/**
* @brief  btsm handle smp pairing failed
* 
* @param  pLink
* @param  pLEMsg
*
* @return 
*
*/
uint8_t btsmHandleSMPPairingFailed(LPTSECLINK pLink, LPLEMessage pLEMsg)
{
	LPLE_SMP_DATA pSMPData = pLink->pSMPData;
	uint8_t          cause    = pLEMsg->p.Data[1];

	BTSECMAN_TRACE_PRINTF_2(BTSECMAN_TRACE_MASK_TRACE,
	                         "btsmHandleSMPPairingFailed: bd:%s cause:%d",
	                         TRACE_BDADDR1(BTSECMAN_TRACE_MASK_TRACE, pLink->bd),
	                         cause
	                         );

	pSMPData->smpState |= LE_SMPSTATE_PAIRING_FAILED_RECVD;

	return cause;
} /* btsmHandleSMPPairingFailed */

/**
* @brief  btsm handle smp encryption information
* 
* @param  pLink
* @param  pLEMsg
*
* @return 
*
*/
uint8_t btsmHandleSMPEncryptionInformation(LPTSECLINK pLink, LPLEMessage pLEMsg)
{
	LPLE_SMP_DATA pSMPData = pLink->pSMPData;
	uint8_t *        ltk      = pLEMsg->p.Data + 1;

	BTSECMAN_TRACE_PRINTF_2(BTSECMAN_TRACE_MASK_TRACE,
	                         "btsmHandleSMPEncryptionInformation: bd:%s ltk:%s",
	                         TRACE_BDADDR1(BTSECMAN_TRACE_MASK_TRACE, pLink->bd),
	                         BTRACE_RAMDATA1(BTSECMAN_TRACE_MASK_TRACE, btsmHexString(ltk, 16))
	                         );

	/* link not encrypted, not the first EncryptInfo or not requested */
	if (!(pLink->mode & LINKMODE_ENCRYPTED) ||
	  (pSMPData->smpState & LE_SMPSTATE_ENCRYPT_INFO_RECVD) ||
	  ((pLink->role == LE_ROLE_MASTER) && !(pSMPData->remote_respKeyDist & LE_SMP_KEYDIST_ENCKEY)) ||
	  ((pLink->role == LE_ROLE_SLAVE) && !(pSMPData->remote_initKeyDist & LE_SMP_KEYDIST_ENCKEY))
	 )
	{
		return LE_SMP_ERROR_UNSPECIFIED_REASON;
	}

	memcpy(pSMPData->p.ph3.rLTK.key, ltk, 16);
	pSMPData->smpState |= LE_SMPSTATE_ENCRYPT_INFO_RECVD;

	/* continue key exchange phase */
	btsmLEKeyExchange(pLink);

	return LE_SMP_ERROR_SUCCESS;
} /* btsmHandleSMPEncryptionInformation */

/**
* @brief  btsm handle smp master identification
* 
* @param  pLink
* @param  pLEMsg
*
* @return 
*
*/
uint8_t btsmHandleSMPMasterIdentification(LPTSECLINK pLink, LPLEMessage pLEMsg)
{
	LPLE_SMP_DATA pSMPData = pLink->pSMPData;
	uint16_t          ediv     = CHAR2SHORT(pLEMsg->p.Data + 1);
	uint8_t *        rand     = pLEMsg->p.Data + 3;

	BTSECMAN_TRACE_PRINTF_3(BTSECMAN_TRACE_MASK_TRACE,
	                         "btsmHandleSMPMasterIdentification: bd:%s ediv:0x%x rand:%s",
	                         TRACE_BDADDR1(BTSECMAN_TRACE_MASK_TRACE, pLink->bd), ediv,
	                         BTRACE_RAMDATA1(BTSECMAN_TRACE_MASK_TRACE, btsmHexString(rand, 8))
	                         );

	/* link not encrypted, not the first MasterIdent, no EncryptInfo or not requested */
	if (!(pLink->mode & LINKMODE_ENCRYPTED) ||
	  (pSMPData->smpState & LE_SMPSTATE_MASTER_ID_RECVD) ||
	  !(pSMPData->smpState & LE_SMPSTATE_ENCRYPT_INFO_RECVD) ||
	  ((pLink->role == LE_ROLE_MASTER) && !(pSMPData->remote_respKeyDist & LE_SMP_KEYDIST_ENCKEY)) ||
	  ((pLink->role == LE_ROLE_SLAVE) && !(pSMPData->remote_initKeyDist & LE_SMP_KEYDIST_ENCKEY))
	 )
	{
		return LE_SMP_ERROR_UNSPECIFIED_REASON;
	}

	pSMPData->smpState |= LE_SMPSTATE_MASTER_ID_RECVD;

	memcpy(pSMPData->p.ph3.rLTK.rand, rand, 8);
	pSMPData->p.ph3.rLTK.ediv     = ediv;
	pSMPData->p.ph3.rLTK.keySize  = pLink->keySize;
	pSMPData->p.ph3.rLTK.mitmFlag = (pLink->keyType == BLUEFACE_KEYTYPE_AUTHENTICATED) ? 1 : 0;

	/* continue key exchange phase */
	btsmLEKeyExchange(pLink);

	return LE_SMP_ERROR_SUCCESS;
} /* btsmHandleSMPMasterIdentification */

/**
* @brief  btsm handle smp  identity information
* 
* @param  pLink
* @param  pLEMsg
*
* @return 
*
*/
uint8_t btsmHandleSMPIdentityInformation(LPTSECLINK pLink, LPLEMessage pLEMsg)
{
	LPLE_SMP_DATA pSMPData = pLink->pSMPData;
	uint8_t *        irk      = pLEMsg->p.Data + 1;

	BTSECMAN_TRACE_PRINTF_2(BTSECMAN_TRACE_MASK_TRACE,
	                         "btsmHandleSMPIdentityInformation: bd:%s irk:%s",
	                         TRACE_BDADDR1(BTSECMAN_TRACE_MASK_TRACE, pLink->bd),
	                         BTRACE_RAMDATA1(BTSECMAN_TRACE_MASK_TRACE, btsmHexString(irk, 16))
	                         );

	/* link not encrypted, not the first IdentInfo or not requested */
	if (!(pLink->mode & LINKMODE_ENCRYPTED) ||
	  (pSMPData->smpState & LE_SMPSTATE_IDENT_INFO_RECVD) ||
	  ((pLink->role == LE_ROLE_MASTER) && !(pSMPData->remote_respKeyDist & LE_SMP_KEYDIST_IDKEY)) ||
	  ((pLink->role == LE_ROLE_SLAVE) && !(pSMPData->remote_initKeyDist & LE_SMP_KEYDIST_IDKEY))
	 )
	{
		return LE_SMP_ERROR_UNSPECIFIED_REASON;
	}

	pSMPData->smpState |= LE_SMPSTATE_IDENT_INFO_RECVD;

	memcpy(pSMPData->p.ph3.rIRK.key, irk, 16);

	/* continue key exchange phase */
	btsmLEKeyExchange(pLink);

	return LE_SMP_ERROR_SUCCESS;
} /* btsmHandleSMPIdentityInformation */

/**
* @brief  btsm handle smp  identity address information
* 
* @param  pLink
* @param  pLEMsg
*
* @return 
*
*/
uint8_t btsmHandleSMPIdentityAddressInformation(LPTSECLINK pLink, LPLEMessage pLEMsg)
{
	LPLE_SMP_DATA pSMPData    = pLink->pSMPData;
	uint8_t          addressType = pLEMsg->p.Data[1];
	uint8_t *        bd          = pLEMsg->p.Data + 2;

	BTSECMAN_TRACE_PRINTF_2(BTSECMAN_TRACE_MASK_TRACE,
	                         "btsmHandleSMPIdentityAddressInformation: addressType:%d realbd:%s",
	                         addressType, TRACE_BDADDR1(BTSECMAN_TRACE_MASK_TRACE, bd)
	                         );

	/* link not encrypted, not the first IdentAddr, no IdentInfo or not requested */
	if (!(pLink->mode & LINKMODE_ENCRYPTED) ||
	  (pSMPData->smpState & LE_SMPSTATE_IDENT_ADDR_RECVD) ||
	  !(pSMPData->smpState & LE_SMPSTATE_IDENT_INFO_RECVD) ||
	  ((pLink->role == LE_ROLE_MASTER) && !(pSMPData->remote_respKeyDist & LE_SMP_KEYDIST_IDKEY)) ||
	  ((pLink->role == LE_ROLE_SLAVE) && !(pSMPData->remote_initKeyDist & LE_SMP_KEYDIST_IDKEY))
	 )
	{
		return LE_SMP_ERROR_UNSPECIFIED_REASON;
	}

	pSMPData->smpState |= LE_SMPSTATE_IDENT_ADDR_RECVD;

	{
		uint16_t i;

		addressType |= BLUEFACE_BDTYPE_LE_MASK | BLUEFACE_BDTYPE_LE_RESOLVED_MASK;

		/* update cache entry */
		for (i = 0; i < LE_RESOLVE_CACHE_SIZE; i++)
		{
			PRESOLVEENTRY pSearch = &pBtSM->LE_resolveCache[i];

			if ((pSearch->state != LE_RESOLVE_ENTRY_STATE_FREE) &&
			  (memcmp(pSearch->random_bd, pLink->unresolved_bd, BD_ADDR_SIZE) == 0)
			 )
			{
				BTSECMAN_TRACE_PRINTF_1(BTSECMAN_TRACE_MASK_ERROR,
				                           "btsm: insert bd:%s into resolving cache",
				                           TRACE_BDADDR1(BTSECMAN_TRACE_MASK_ERROR, pSearch->random_bd)
				                           );

				memcpy(pSearch->resolved_bd, bd, BD_ADDR_SIZE);
				pSearch->resolved_bdType  = addressType;
				pSearch->state            = LE_RESOLVE_ENTRY_STATE_POSITIVE;
				pSearch->timestamp        = osGetSystemTime();
				break;
			}
		}
	}

	/* notify upper layers about BD resolving */
	if (!(pLink->bdType & BLUEFACE_BDTYPE_LE_RESOLVED_MASK))
	{
		btsmSendBLUEFACE_CONF_RESOLVED_ADDRESS(pLink->unresolved_bd, pLink->unresolved_bdType, bd, addressType);
	}

	/* bd/bdtype only used for local IRK */
	memset(pSMPData->p.ph3.rIRK.bd, 0x00, BD_ADDR_SIZE);
	pSMPData->p.ph3.rIRK.mitmFlag = (pLink->keyType == BLUEFACE_KEYTYPE_AUTHENTICATED) ? 1 : 0;
	/* use "real" BD as new pLink BD */
	memcpy(pLink->bd, bd, BD_ADDR_SIZE);
	pLink->bdType = addressType;

	/* continue key exchange phase */
	btsmLEKeyExchange(pLink);

	return LE_SMP_ERROR_SUCCESS;
} /* btsmHandleSMPIdentityAddressInformation */

/**
* @brief   btsm handle smp signing information
*
* @param  pLink
* @param  pLEMsg
*
* @return  
*
*/
uint8_t btsmHandleSMPSigningInformation(LPTSECLINK pLink, LPLEMessage pLEMsg)
{
	LPLE_SMP_DATA pSMPData = pLink->pSMPData;
	uint8_t *        csrk     = pLEMsg->p.Data + 1;

	BTSECMAN_TRACE_PRINTF_2(BTSECMAN_TRACE_MASK_TRACE,
	                         "btsmHandleSMPSigningInformation: bd:%s csrk:%s",
	                         TRACE_BDADDR1(BTSECMAN_TRACE_MASK_TRACE, pLink->bd),
	                         BTRACE_RAMDATA1(BTSECMAN_TRACE_MASK_TRACE, btsmHexString(csrk, 16))
	                         );

	/* link not encrypted, not the first IdentInfo or not requested */
	if (!(pLink->mode & LINKMODE_ENCRYPTED) ||
	  (pSMPData->smpState & LE_SMPSTATE_SIGN_INFO_RECVD) ||
	  ((pLink->role == LE_ROLE_MASTER) && !(pSMPData->remote_respKeyDist & LE_SMP_KEYDIST_SIGNKEY)) ||
	  ((pLink->role == LE_ROLE_SLAVE) && !(pSMPData->remote_initKeyDist & LE_SMP_KEYDIST_SIGNKEY))
	 )
	{
		return LE_SMP_ERROR_UNSPECIFIED_REASON;
	}

	pSMPData->smpState |= LE_SMPSTATE_SIGN_INFO_RECVD;

	UNUSED_PARAMETER(csrk);

	/* continue key exchange phase */
	btsmLEKeyExchange(pLink);

	return LE_SMP_ERROR_SUCCESS;
} /* btsmHandleSMPSigningInformation */

/**
* @brief   btsm handle security request
*
* @param  pLink
* @param  pLEMsg
*
* @return  
*
*/
uint8_t btsmHandleSMPSecurityRequest(LPTSECLINK pLink, LPLEMessage pLEMsg)
{
	LPLE_SMP_DATA pSMPData = pLink->pSMPData;
	uint8_t          authReq  = pLEMsg->p.Data[1];

	BTSECMAN_TRACE_PRINTF_2(BTSECMAN_TRACE_MASK_TRACE,
	                         "btsmHandleSMPSecurityRequest: bd:%s authReq:0x%x",
	                         TRACE_BDADDR1(BTSECMAN_TRACE_MASK_TRACE, pLink->bd), authReq
	                         );

	/* master and first SecurityReq */
	if ((pLink->role != LE_ROLE_MASTER) &&
	  (pSMPData->smpState & LE_SMPSTATE_SECURITY_REQ_RECVD)
	 )
	{
		return LE_SMP_ERROR_UNSPECIFIED_REASON;
	}

	/* ignore SecurityReq if a PairingReq was already sent */
	if (!(pSMPData->smpState & LE_SMPSTATE_PAIRING_REQ_SENT))
	{
		pSMPData->smpState |= LE_SMPSTATE_SECURITY_REQ_RECVD;

		/* only start new auth if link is idle */
		if (pLink->state == SEC_LINK_IDLE)
		{
			btsmUpdateSecurityStatus(pLink, SECSTAT_SMP_SECURITY_REQ, HCI_SUCCESS);
			/* store if MITM secure key is requested */
			pLink->state = (authReq & LE_SMP_AUTHREQ_MITM) ? SEC_LINK_SECURITY_MITM_REQ
			                                             : SEC_LINK_SECURITY_REQ;
			
			// TODO: LINKMODE_FORCE_PAIRING <-> LE_SMP_AUTHREQ_BONDING

			/* allow stored keys but do not store the MITM requirement in ->mode */
			pLink->mode = (pLink->mode & ~(LINKMODE_FORCE_PAIRING)) | LINKMODE_AUTHEN_REQUIRED | LINKMODE_ENCR_REQUIRED;

			/* request link key from storage */
			btsmSendDEVICE_DATA_GET_IND(pLink, DEVICE_DATA_TYPE_SECMAN_LTK_REMOTE);
		}
	}

	return LE_SMP_ERROR_SUCCESS;
} /* btsmHandleSMPSecurityRequest */

/**
* @brief  bt sm le get msg buf
*
* @param  pMessage
* @param  pLEMsg
* @param  length
* @param  msgType
* @param  handle
*
* @return  
*
*/
int btsmLEMsgBufferGet(MESSAGE_P pMessage, uint16_t length, void  *  *BufferPtr)
{
	uint8_t * pBuffer;
	int    result;

	result = osBufferGet(DownstreamPoolID, length + BT_LE_MESSAGE_OFFSET + 8, (PVOID  *)&pBuffer);

	if (!result)
	{
		pMessage->Command                    = LE_MESSAGE_REQ;
		pMessage->MData.DataCB.BufferAddress = pBuffer;
		pMessage->MData.DataCB.Offset        = BT_LE_MESSAGE_OFFSET + 8;
		pMessage->MData.DataCB.Length        = length;
		pMessage->MData.DataCB.Flag          = DATA_CB_RELEASE;

        /* return pointer to data */
        *BufferPtr = pBuffer + pMessage->MData.DataCB.Offset;
	}
	else
	{
		BTSECMAN_TRACE_PRINTF_1(BTSECMAN_TRACE_MASK_ERROR,
		                           "!!btsmLEMsgBufferGet: No Memory for len:0x%x",
		                           (length + BT_LE_MESSAGE_OFFSET)
		                           );
	}

	return result;
} /* btsmLEMsgBufferGet */

/**
* @brief   btsm handle smp message
*
* @param  pLEMsg
* @param  length
*
* @return  
*
*/
void btsmHandleSMPMessage(LPLEMessage pLEMsg, uint16_t length)
{
	LPTSECLINK pLink   = btsmFindLELinkByHandle(pLEMsg->handle);
	uint8_t       command = pLEMsg->p.Data[0];
	uint8_t       cause   = LE_SMP_ERROR_SUCCESS;

	if (pLink == NULL)
	{
		return;
	}

	if (pLink->pSMPData == NULL)
	{
		switch (command)
		{
		case LE_SMP_PAIRING_REQUEST: 
		case LE_SMP_SECURITY_REQUEST:
			if (btsmAllocateSMPData(pLink) == NULL)
			{
				BTSECMAN_TRACE_PRINTF_2(BTSECMAN_TRACE_MASK_ERROR,
				                         "!!btsmHandleSMPMessage: failed to allocate SMPData for handle:0x%x cmd:%d",
				                         pLink->handle, command
				                         );

				cause = LE_SMP_ERROR_UNSPECIFIED_REASON;
				break;
			}
			break;

		  default:
		    BTSECMAN_TRACE_PRINTF_2(BTSECMAN_TRACE_MASK_ERROR,
		                               "!!btsmHandleSMPMessage: no SMPData for handle:0x%x cmd:%d",
		                               pLink->handle,command
		                               );

		    cause = LE_SMP_ERROR_UNSPECIFIED_REASON;
		    break;
		}
	}

	length -= offsetof(TLEMessage, p.Data[0]);

	if (cause == LE_SMP_ERROR_SUCCESS)
	{
		/* default to invalid length of parameters */
		cause = LE_SMP_ERROR_INVALID_PARAMETERS;

		switch (command)
		{
		case LE_SMP_PAIRING_REQUEST: 
		case LE_SMP_PAIRING_RESPONSE:
			if (length == LE_SMP_PAIRING_REQUEST_LENGTH)
			{
				cause = btsmHandleSMPPairingExchange(pLink, pLEMsg);
			}
			break;

		case LE_SMP_PAIRING_CONFIRM:
			if (length == LE_SMP_PAIRING_CONFIRM_LENGTH)
			{
				cause = btsmHandleSMPPairingConfirm(pLink, pLEMsg);
			}
			break;

		case LE_SMP_PAIRING_RANDOM: 
			if (length == LE_SMP_PAIRING_RANDOM_LENGTH)
			{
				cause = btsmHandleSMPPairingRandom(pLink, pLEMsg);
			}
			break;

		case LE_SMP_PAIRING_FAILED:
			if (length == LE_SMP_PAIRING_FAILED_LENGTH)
			{
				cause = btsmHandleSMPPairingFailed(pLink, pLEMsg);
			}
			break;

		case LE_SMP_ENCRYPTION_INFORMATION:
			if (length == LE_SMP_ENCRYPTION_INFORMATION_LENGTH)
			{
				cause = btsmHandleSMPEncryptionInformation(pLink, pLEMsg);
			}
			break;

		case LE_SMP_MASTER_IDENTIFICATION: 
			if (length == LE_SMP_MASTER_IDENTIFICATION_LENGTH)
			{
				cause = btsmHandleSMPMasterIdentification(pLink, pLEMsg);
			}
			break;

		case LE_SMP_IDENTITY_INFORMATION:
			if (length == LE_SMP_IDENTITY_INFORMATION_LENGTH)
			{
				cause = btsmHandleSMPIdentityInformation(pLink, pLEMsg);
			}
			break;

		case LE_SMP_IDENTITY_ADDRESS_INFORMATION:
			if (length == LE_SMP_IDENTITY_ADDRESS_INFORMATION_LENGTH)
			{
				cause = btsmHandleSMPIdentityAddressInformation(pLink, pLEMsg);
			}
			break;

		case LE_SMP_SIGNING_INFORMATION:
			if (length == LE_SMP_SIGNING_INFORMATION_LENGTH)
			{
				cause = btsmHandleSMPSigningInformation(pLink, pLEMsg);
			}
			break;

		case LE_SMP_SECURITY_REQUEST:
			if (length == LE_SMP_SECURITY_REQUEST_LENGTH)
			{
				cause = btsmHandleSMPSecurityRequest(pLink, pLEMsg);
			}
			break;

		default:
			cause = LE_SMP_ERROR_COMMAND_NOT_SUPPORTED;
			break;
		}

		/* invalid state or message order */
		if ((cause == LE_SMP_ERROR_UNSPECIFIED_REASON) &&
		    (command != LE_SMP_PAIRING_FAILED)
		   )
		{
			BTSECMAN_TRACE_PRINTF_3(BTSECMAN_TRACE_MASK_ERROR,
			                         "!!btsmHandleSMPMessage: invalid cmd:%d role:%d smpState:0x%x",
			                         command,
			                         pLink->role, pLink->pSMPData->smpState
			                         );
		}
	}

	if (cause != LE_SMP_ERROR_SUCCESS)
	{
		/* only respond with Pairing Failed if pairing is still active */
		if ((command != LE_SMP_PAIRING_FAILED) && (pLink->pSMPData != NULL))
		{
			btsmSendSMPPairingFailed(pLink, cause);
		}

		if (cause == LE_SMP_ERROR_PAIRING_NOT_SUPPORTED)
		{
			cause = HCI_ERR_PARING_NOT_ALLOWED;
		}
		else
		{
			cause = HCI_ERR_AUTHENTICATION_FAILED;
		}

		btsmAuthenticationComplete(pLink, HCI_ERR | cause);
	}
} /* btsmHandleSMPMessage */

/**
* @brief  btsm send smp pairing exchange
*
* @param  pLink
* @param  command
* @param  ioCap
* @param  oobFlag
* @param  authReq
* @param  maxKeySize
* @param  initKeyDist
* @param  respKeyDist
*
* @return  
*
*/
void btsmSendSMPPairingExchange(LPTSECLINK pLink, uint8_t command,
                                uint8_t ioCap, uint8_t oobFlag, uint8_t authReq,
                                uint8_t maxKeySize, uint8_t initKeyDist, uint8_t respKeyDist)
{
	LPLE_SMP_DATA pSMPData = pLink->pSMPData;
    uint8_t * pData = NULL;
	MESSAGE_T     msg;
	uint16_t          pos      = 0;

	BTSECMAN_TRACE_PRINTF_7(BTSECMAN_TRACE_MASK_TRACE,
	                         "btsmSendSMPPairing: bd:%s io:%x oob:%x auth:%x maxkey:%x iKeyDist:%x rKeyDist:%x",
	                         //(command == LE_SMP_PAIRING_REQUEST ? pRequest : pResponse),
	                         TRACE_BDADDR1(BTSECMAN_TRACE_MASK_TRACE, pLink->bd), ioCap, oobFlag, authReq,
	                         maxKeySize, initKeyDist, respKeyDist
	                         );

	if (btsmLEMsgBufferGet(&msg, LE_SMP_PAIRING_REQUEST_LENGTH, (PVOID)&pData
	                    ))
	{
		return;
	}

	pData[pos++] = command;
	pData[pos++] = ioCap;
	pData[pos++] = oobFlag;
	pData[pos++] = authReq;
	pData[pos++] = maxKeySize;
	pData[pos++] = initKeyDist;
	pData[pos++] = respKeyDist;

	/* store PairingExchange for CONF value generation */
	if (command == LE_SMP_PAIRING_REQUEST)
	{
		memcpy(pSMPData->pairingReq, pData, LE_SMP_PAIRING_REQUEST_LENGTH);
		pSMPData->smpState |= LE_SMPSTATE_PAIRING_REQ_SENT;
	}
	else if (command == LE_SMP_PAIRING_RESPONSE)
	{
		memcpy(pSMPData->pairingRsp, pData, LE_SMP_PAIRING_RESPONSE_LENGTH);
		pSMPData->smpState |= LE_SMPSTATE_PAIRING_RSP_SENT;
	}

	l2cLEHandleDataReq(&msg, CID_SECURITY_MANAGER, pLink->handle);
} /* btsmSendSMPPairingRequest */

/**
* @brief  btsm send smp 128 bit value
*
* @param  pLink
* @param  command
* @param  value
*
* @return  
*
*/
void btsmSendSMP128BitValue(LPTSECLINK pLink, uint8_t command, uint8_t * value)
{
	LPLE_SMP_DATA pSMPData = pLink->pSMPData;
	uint8_t * pData = NULL;
	MESSAGE_T     msg;
	uint16_t          pos      = 0;

	BTSECMAN_TRACE_PRINTF_2(BTSECMAN_TRACE_MASK_TRACE,
	                         "btsmSendSMP128BitValue: %d value:%s",
	                         command,
	                         BTRACE_RAMDATA1(BTSECMAN_TRACE_MASK_TRACE, btsmHexString(value, 16))
	                         );

	if (btsmLEMsgBufferGet(&msg, LE_SMP_128BIT_VALUE_LENGTH, (PVOID)&pData
	                    ))
	{
		return;
	}

	switch (command)
	{
	case LE_SMP_PAIRING_RANDOM:
		pSMPData->smpState |= LE_SMPSTATE_PAIRING_RAND_SENT;
		break;
	case LE_SMP_PAIRING_CONFIRM: 
		pSMPData->smpState |= LE_SMPSTATE_PAIRING_CONF_SENT;
		break;
	case LE_SMP_ENCRYPTION_INFORMATION: 
		pSMPData->smpState |= LE_SMPSTATE_ENCRYPT_INFO_SENT;
		break;
	default: 
		break;
	}

	pData[pos++] = command;
	memcpy(pData + pos, value, 16); pos += 16;

	l2cLEHandleDataReq(&msg, CID_SECURITY_MANAGER, pLink->handle);
} /* btsmSendSMPPairingConfirm */

/**
* @brief  btsm send SMP pairing failed
*
* @param  pLink
* @param  cause
*
* @return  
*
*/
void btsmSendSMPPairingFailed(LPTSECLINK pLink, uint8_t cause)
{
	MESSAGE_T   msg;
	uint8_t * pData = NULL;
	uint16_t        pos    = 0;

	BTSECMAN_TRACE_PRINTF_2(BTSECMAN_TRACE_MASK_TRACE,
	                         "btsmSendSMPPairingFailed: bd:%s cause:%d",
	                         TRACE_BDADDR1(BTSECMAN_TRACE_MASK_TRACE, pLink->bd),
	                         cause
	                         );

	if (btsmLEMsgBufferGet(&msg, LE_SMP_PAIRING_FAILED_LENGTH, (PVOID)&pData
	                    ))
	{
		return;
	}

	/* might be NULL if responding to an invalid message */
	if (pLink->pSMPData != NULL)
	{
		pLink->pSMPData->smpState |= LE_SMPSTATE_PAIRING_FAILED_SENT;
	}

	pData[pos++] = LE_SMP_PAIRING_FAILED;
	pData[pos++] = cause;

	l2cLEHandleDataReq(&msg, CID_SECURITY_MANAGER, pLink->handle);
} /* btsmSendSMPPairingFailed */

/**
* @brief  bt sm send SMP master indentification
*
* @param  pLink
* @param  ediv
* @param  rand
*
* @return  
*
*/
void btsmSendSMPMasterIdentification(LPTSECLINK pLink, uint16_t ediv, uint8_t * rand)
{
	MESSAGE_T   msg;
	uint8_t * pData = NULL;
	uint16_t        pos    = 0;

	BTSECMAN_TRACE_PRINTF_3(BTSECMAN_TRACE_MASK_TRACE,
	                         "btsmSendSMPMasterIdentification: bd:%s ediv:0x%x rand:%s",
	                         TRACE_BDADDR1(BTSECMAN_TRACE_MASK_TRACE, pLink->bd),
	                         ediv, BTRACE_RAMDATA1(BTSECMAN_TRACE_MASK_TRACE, btsmHexString(rand, 8))
	                         );

	if (btsmLEMsgBufferGet(&msg, LE_SMP_MASTER_IDENTIFICATION_LENGTH, (PVOID)&pData
	                    ))
	{
		return;
	}

	pLink->pSMPData->smpState |= LE_SMPSTATE_MASTER_ID_SENT;

	pData[pos++] = LE_SMP_MASTER_IDENTIFICATION;
	SHORT2CHAR(pData + pos, ediv); pos += 2;
	memcpy(pData + pos, rand, 8);  pos += 8;

	l2cLEHandleDataReq(&msg, CID_SECURITY_MANAGER, pLink->handle);
} /* btsmSendSMPMasterIdentification */

/**
* @brief  smp security request
*
* @param  pLink: 
* @param  authReq
*
* @return  
*
*/
void btsmSendSMPSecurityRequest(LPTSECLINK pLink, uint8_t authReq)
{
	MESSAGE_T   msg;
	uint8_t * pData = NULL;
	uint16_t        pos    = 0;

	BTSECMAN_TRACE_PRINTF_2(BTSECMAN_TRACE_MASK_TRACE,
	                         "btsmSendSMPSecurityRequest: bd:%s authReq:0x%x",
	                         TRACE_BDADDR1(BTSECMAN_TRACE_MASK_TRACE, pLink->bd), authReq
	                         );

	if (btsmLEMsgBufferGet(&msg, LE_SMP_SECURITY_REQUEST_LENGTH, (PVOID)&pData
	                    ))
	{
		return;
	}

	pLink->pSMPData->smpState |= LE_SMPSTATE_SECURITY_REQ_SENT;

	pData[pos++] = LE_SMP_SECURITY_REQUEST;
	pData[pos++] = authReq;

	l2cLEHandleDataReq(&msg, CID_SECURITY_MANAGER, pLink->handle);
} /* btsmSendSMPSecurityRequest */

