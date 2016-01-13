/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       btsm_le.c
* @brief     Bluetooth Security Manager 2.1
* @details   
*
* @author   	gordon
* @date      	2015-07-09
* @version	v0.1
*/

#include <flags.h>
#include <rtl_types.h>
#include <message.h>
#include <os_pool.h>
#include <os_sched.h>
#include <btsm.h>
#include <btsmprot.h>
#include <hci_api.h>
#include <btsm_api.h>
#include <l2c_api.h>
#include <hci_code.h>
#include <gatt_api.h>
#include <blueapi_api.h>
#include <upper_stack_global.h>

#define TRACE_MODULE_ID     MID_BTSECMAN

/**
* @brief  btsm setup le cryp to confirm value
* 
* @param  pLink
* @param  pBuffer
* @param  pRand
*
* @return 
*
*/
void btsmSetupLECryptoConfirmValue1(LPTSECLINK pLink, uint8_t * pBuffer, uint8_t * pRand)
{
	LPLE_SMP_DATA pSMPData        = pLink->pSMPData;
	uint8_t          peerAddressType = (pLink->unresolved_bdType & LE_ADDRESS_TYPE_RANDOM);
	uint16_t          i;
    
    BTSECMAN_TRACE_PRINTF_1(BTSECMAN_TRACE_MASK_TRACE,
							"btsmSetupLECryptoConfirmValue1: ownAddressType 0x%x",
							pLink->ownAddressType
						   );
	/* p1 is PairingRequest[7] | PairingResponse[7] | initiatorAddressType | responderAddressType */
	pBuffer[0] = ((pLink->role == LE_ROLE_MASTER) ? pLink->ownAddressType : peerAddressType);
	pBuffer[1] = ((pLink->role == LE_ROLE_MASTER) ? peerAddressType : pLink->ownAddressType);

	memcpy(&pBuffer[2], pSMPData->pairingReq, LE_SMP_PAIRING_REQUEST_LENGTH);
	memcpy(&pBuffer[9], pSMPData->pairingRsp, LE_SMP_PAIRING_RESPONSE_LENGTH);

	/* XOR with RAND value */
	for (i = 0; i < LE_ENCRYPT_DATA_LENGTH; i++)
	{
		pBuffer[i] ^= pRand[i];
	}
} /* btsmSetupLECryptoConfirmValue1 */

/**
* @brief  btsm setup le cryp to confirm value2
* 
* @param  pLink
* @param  pBuffer
* @param  pFirstRound
*
* @return 
*
*/
void btsmSetupLECryptoConfirmValue2(LPTSECLINK pLink, uint8_t * pBuffer, uint8_t * pFirstRound)
{
	uint16_t i;
	uint8_t * localBD = pBtSM->localBD;
	uint8_t * peerBD  = pLink->unresolved_bd;

    if(pLink->ownAddressType == LE_ADDRESS_TYPE_PUBLIC)
	{
		localBD = pBtSM->localBD;
	}
	else if(pLink->ownAddressType == LE_ADDRESS_TYPE_RANDOM)
	{
		localBD = pBtSM->randomBD;
	}
    else
	{
		BTSECMAN_TRACE_PRINTF_1(BTSECMAN_TRACE_MASK_ERROR,
							"!!btsmSetupLECryptoConfirmValue2: ownAddressType 0x%x",
							pLink->ownAddressType
						   );
	}

	/* p2 is padding | initiating BD | responding BD */
	memcpy(&pBuffer[0], ((pLink->role == LE_ROLE_MASTER) ? peerBD : localBD), BD_ADDR_SIZE);
	memcpy(&pBuffer[6], ((pLink->role == LE_ROLE_MASTER) ? localBD : peerBD), BD_ADDR_SIZE);
	memset(&pBuffer[12], 0x00, 4);

	/* XOR with FirstRound value */
	for (i = 0; i < LE_ENCRYPT_DATA_LENGTH; i++)
	{
		pBuffer[i] ^= pFirstRound[i];
	}
} /* btsmSetupLECryptoConfirmValue2 */

/**
* @brief  btsm setup le crypt to STK
*
* @param  pLink
* @param  pBuffer
*
* @return  
*
*/
void btsmSetupLECryptoSTK(LPTSECLINK pLink, uint8_t * pBuffer)
{
	LPLE_SMP_DATA_PHASE2 pSMPDataPh2 = &pLink->pSMPData->p.ph2;

	memcpy(&pBuffer[0], ((pLink->role == LE_ROLE_MASTER) ? pSMPDataPh2->local_RAND : pSMPDataPh2->remote_RAND), LE_RANDOM_NUMBER_SIZE);
	memcpy(&pBuffer[8], ((pLink->role == LE_ROLE_MASTER) ? pSMPDataPh2->remote_RAND : pSMPDataPh2->local_RAND), LE_RANDOM_NUMBER_SIZE);
}

/**
* @brief  btsm generate le passkey
*
* @param  rand
*
* @return  
*
*/
uint32_t btsmGenerateLEPasskey(uint8_t * rand)
{
	uint32_t value;
	uint16_t  i;

	/* use a moving 20bit mask to get a value of 0-999999 from random data */
	for (i = 0; i < (LE_RANDOM_NUMBER_SIZE -2); i++)
	{
		value = rand[i] + (rand[i+1] << 8) + (rand[i+2] << 16);
		if ((value & 0x0FFFFF) < 1000000)
		{
			return (value & 0x0FFFFF);
		}

		value >>= 4;
		if ((value & 0x0FFFFF) < 1000000)
		{
			return (value & 0x0FFFFF);
		}
	}

	/* invalid value */
	return 0xFFFFFFFF;
} /* btsmGenerateLEPasskey */

/**
* @brief  bt sm send hci le encrypt
*
* @param  handle
* @param  key
* @param  plaintextData
*
* @return  
*
*/
void btsmSendLHci_LE_ENCRYPT(uint16_t handle, uint8_t * key, uint8_t * plaintextData)
{
	MESSAGE_T   msg;
	LPLEMessage pLEMsg;

	if (btsmLEMsgBufferGet2(&msg, &pLEMsg, sizeof(TLEEncrypt),
					   LE_ENCRYPT, handle
					  ))
	{
		return;
	}

	memcpy(pLEMsg->p.Encrypt.key, key, LE_ENCRYPT_KEY_LENGTH);
	memcpy(pLEMsg->p.Encrypt.plaintextData, plaintextData, LE_ENCRYPT_DATA_LENGTH);

	osMessageSend(pBtSM->cryptQueueID, &msg);
	/* trigger transaction queue */
	btsmTriggerCryptQueue();
} /* btsmSendLHci_LE_ENCRYPT */


/**
* @brief  bt sm send hci le rand
*
* @param  handle
*
* @return  
*
*/
void btsmSendLHci_LE_RAND(uint16_t handle)
{
	MESSAGE_T   msg;
	LPLEMessage pLEMsg;

	if (btsmLEMsgBufferGet2(&msg, &pLEMsg, 0,
					   LE_RAND, handle
					  ))

	{
		return;
	}
	osMessageSend(pBtSM->cryptQueueID, &msg);

	/* trigger transaction queue */
	btsmTriggerCryptQueue();
} /* btsmSendLHci_LE_RAND */

/**
* @brief  btsm send hci le start encryption
*
* @param  handle
* @param  randomNumber
* @param  encryptedDiversifier
* @param  longTermKey
*
* @return  
*
*/
void btsmSendLHci_LE_START_ENCRYPTION(uint16_t handle, uint8_t * randomNumber, uint16_t encryptedDiversifier, uint8_t * longTermKey)
{
    hciCommandLEStartEncryption(handle, randomNumber, encryptedDiversifier, longTermKey);
	hciLaterEntry();		
} /* btsmSendLHci_LE_START_ENCRYPTION */

/**
* @brief  btsm send le longterm key request reply
*
* @param  handle
* @param  longTermKey
*
* @return  
*
*/
void btsmSendLHci_LE_LONGTERM_KEY_REQUEST_REPLY(uint16_t handle, uint8_t * longTermKey)
{
    hciCommandLELongTermKeyRequestReply(handle, longTermKey);
	hciLaterEntry();		
}

/**
* @brief  bt sm send HCI_LE_LONG_TERM_KEY_REQUESTED_NEGATIVE_REPLY
*
* @param  handle
*
* @return  
*
*/
void btsmSendLHci_LE_LONGTERM_KEY_REQUEST_NEGATIVE_REPLY(uint16_t handle)
{
	hciCommandWordParameter(HCI_LE_LONG_TERM_KEY_REQUESTED_NEGATIVE_REPLY, handle);
	hciLaterEntry();		
} /* btsmSendLHci_LE_LONGTERM_KEY_REQUEST_NEGATIVE_REPLY */

/**
* @brief  bt sm send HCI_DISCONNECT command
*
* @param  handle
* @param  cause
*
* @return  
*
*/
void btsmSendLHci_LE_DISCONNECT(uint16_t handle, uint16_t cause)
{
	hciCommandDisconnect(handle,cause);
	hciLaterEntry();	
} /* btsmSendLHci_LE_DISCONNECT */

void btsmHandleLE_SET_RANDOM_ADDRESS(uint8_t * randomAddress)
{
    memcpy(pBtSM->randomBD, randomAddress, BD_ADDR_SIZE);
}

void btsmHandleCmdCompHCI_LE_SET_RANDOM_ADDRESS(uint16_t status)
{
}

void btsmUpdateLocalBdType(uint8_t localBdType)
{
    pBtSM->localBdType = localBdType;
}
/**
* @brief  gatt handle le encrypt confirm
*
* @param  handle
* @param  result
*
* @return  
*
*/
void btsmHandleHci_LE_ENCRYPT_CONF(uint16_t handle, uint8_t * result)
{
	uint8_t          temp[LE_ENCRYPT_DATA_LENGTH];
	LPTSECLINK    pLink       = NULL;

	PRESOLVEENTRY pEntry      = NULL;
	LPLE_SMP_DATA pSMPData    = NULL;
	uint16_t          cryptState  = LE_CRYPT_IDLE;

	if (handle & LE_CRYPT_HANDLE_LINK)
	{
	
        uint16_t index = handle & LE_CRYPT_HANDLE_MASK;
		if(index & 0x10)
			 pLink       = &pBtSM->plinksDon[index -0x10];
		else
			 pLink       = &pBtSM->plinksDoff[index];
	
		pSMPData    = pLink->pSMPData;
		cryptState  = pLink->cryptState;
	}
	else if ((handle & LE_CRYPT_HANDLE_CACHE) &&
	   ((handle & LE_CRYPT_HANDLE_MASK) < LE_RESOLVE_CACHE_SIZE)
	  )
	{
		pEntry = &pBtSM->LE_resolveCache[handle & LE_CRYPT_HANDLE_MASK];
		if (pEntry->state == LE_RESOLVE_ENTRY_STATE_RESOLVING)
		{
			cryptState = LE_CRYPT_RESOLVE_IRK;
		}
	}
	else
	{
		BTSECMAN_TRACE_PRINTF_1(BTSECMAN_TRACE_MASK_ERROR,
		                       "!!btsmHandleHci_LE_ENCRYPT_Conf: no link/SMPData found for handle 0x%x",
		                       handle
		                       );
		return;
	}

	switch (cryptState)
	{
	case LE_CRYPT_GEN_CONF_1:
		/* first round of local_CONF generated, trigger second round  */
		pLink->cryptState = LE_CRYPT_GEN_CONF_2;
		btsmSetupLECryptoConfirmValue2(pLink, temp, result);
		btsmSendLHci_LE_ENCRYPT(LE_LINK2HANDLE(pBtSM, pLink), pSMPData->p.ph2.local_TK, temp);
		break;

	case LE_CRYPT_GEN_CONF_2:
		/* second round of local_CONF generated */
		memcpy(pSMPData->p.ph2.local_CONF, result, LE_ENCRYPT_DATA_LENGTH);
		pLink->cryptState = LE_CRYPT_IDLE;

		BTSECMAN_TRACE_PRINTF_1(BTSECMAN_TRACE_MASK_TRACE,
		                         "btsm: CONF:%s",
		                         BTRACE_RAMDATA1(BTSECMAN_TRACE_MASK_TRACE, btsmHexString(pSMPData->p.ph2.local_CONF, 16))
		                         );

		pSMPData->smpState |= LE_SMPSTATE_LOCAL_CONF_VALID;

		/* send local_CONF value */
		if ((pLink->role == LE_ROLE_MASTER) ||
		  pSMPData->smpState & LE_SMPSTATE_PAIRING_CONF_RECVD
		 )
		{
			btsmSendSMP128BitValue(pLink, LE_SMP_PAIRING_CONFIRM, pSMPData->p.ph2.local_CONF);
			btsmStartSMPTimer(pLink);
		}
		break;

	case LE_CRYPT_CHK_CONF_1:
		/* first round of remote_CONF check completed, trigger second round */
		pLink->cryptState = LE_CRYPT_CHK_CONF_2;
		btsmSetupLECryptoConfirmValue2(pLink, temp, result);
		btsmSendLHci_LE_ENCRYPT(LE_LINK2HANDLE(pBtSM, pLink), pSMPData->p.ph2.local_TK, temp);
		break;

	case LE_CRYPT_CHK_CONF_2:
		/* second round of remote_CONF check completed */
		pLink->cryptState = LE_CRYPT_IDLE;

		BTSECMAN_TRACE_PRINTF_1(BTSECMAN_TRACE_MASK_TRACE,
		                         "btsm: CHECK:%s",
		                         BTRACE_RAMDATA1(BTSECMAN_TRACE_MASK_TRACE, btsmHexString(result, 16))
		                         );

		if (memcmp(pSMPData->p.ph2.remote_CONF, result, LE_ENCRYPT_DATA_LENGTH) == 0)
		{
			if (pLink->role == LE_ROLE_SLAVE)
			{
				btsmSendSMP128BitValue(pLink, LE_SMP_PAIRING_RANDOM, pSMPData->p.ph2.local_RAND);
				btsmStartSMPTimer(pLink);
			}

			/* trigger STK generation */
			pLink->cryptState = LE_CRYPT_GEN_STK;
			btsmSetupLECryptoSTK(pLink, temp);
			btsmSendLHci_LE_ENCRYPT(LE_LINK2HANDLE(pBtSM, pLink), pSMPData->p.ph2.local_TK, temp);
		}
		else
		{
			btsmSendSMPPairingFailed(pLink, LE_SMP_ERROR_CONFIRM_VALUE_FAILED);
			btsmAuthenticationComplete(pLink, HCI_ERR | HCI_ERR_AUTHENTICATION_FAILED);
		}
		break;

	case LE_CRYPT_GEN_STK:
		/* STK generation completed */
		pLink->cryptState = LE_CRYPT_IDLE;

		/* shorten STK if needed */
		if (pLink->keySize < LE_LONG_TERM_KEY_SIZE)
		{
			memset(result + pLink->keySize, 0x00, (LE_LONG_TERM_KEY_SIZE - pLink->keySize));
		}

		if ((pSMPData->sspMech == LE_SMP_MECH_DISPLAY) ||
		  (pSMPData->sspMech == LE_SMP_MECH_KEYBOARD) ||
		  (pSMPData->sspMech == LE_SMP_MECH_OUT_OF_BAND)
		 )
		{
			pLink->mode    = (pLink->mode & ~(LINKMODE_STORED_KEY)) | LINKMODE_AUTHEN | LINKMODE_MITM;
			pLink->keyType = BLUEFACE_KEYTYPE_AUTHENTICATED;
		}
		else
		{
			pLink->mode    = (pLink->mode & ~(LINKMODE_STORED_KEY)) | LINKMODE_AUTHEN;
			pLink->keyType = BLUEFACE_KEYTYPE_UNAUTHENTICATED;
		}

		BTSECMAN_TRACE_PRINTF_3(BTSECMAN_TRACE_MASK_TRACE,
		                         "btsm: STK:%s keyType:%d keySize:%d",
		                         BTRACE_RAMDATA1(BTSECMAN_TRACE_MASK_TRACE, btsmHexString(result, 16)),
		                         pLink->keyType, pLink->keySize
		                         );

		pSMPData->smpState |= LE_SMPSTATE_STK_VALID;

		if (pLink->role == LE_ROLE_MASTER)
		{
			/* request STK from peer (ediv/rand == 0x00) */
			memset(temp, 0x00, LE_RANDOM_NUMBER_SIZE);
			btsmSendLHci_LE_START_ENCRYPTION(pLink->handle, temp, 0x0000, result);
		}
		else
		{
			/* store STK (until it is requested by HCI) */
			memcpy(pSMPData->p.stk, result, LE_ENCRYPT_DATA_LENGTH);

			if (pSMPData->smpState & LE_SMPSTATE_STK_REQUESTED)
			{
				pLink->pSMPData->smpState &= ~(LE_SMPSTATE_STK_REQUESTED);
				btsmSendLHci_LE_LONGTERM_KEY_REQUEST_REPLY(pLink->handle, pSMPData->p.stk);
			}
		}
		break;

	case LE_CRYPT_RESOLVE_IRK:
		/* matching IRK? */
		if (memcmp(&result[0], &pEntry->random_bd[0], 3) == 0)
		{
			uint16_t i;

			/* check cache for duplicated (resolved_bd) entry */
			for (i = 0; i < LE_RESOLVE_CACHE_SIZE; i++)
			{
				PRESOLVEENTRY pSearch = &pBtSM->LE_resolveCache[i];

				if ((pSearch->state & LE_RESOLVE_ENTRY_STATE_RESOLVED_MASK) &&
				  (pSearch->resolved_bdType == pEntry->resolved_bdType) &&
				  (memcmp(pSearch->resolved_bd, pEntry->resolved_bd, BD_ADDR_SIZE) == 0)
				 )
				{
					memcpy(pSearch->random_bd, pEntry->random_bd, BD_ADDR_SIZE);
					pEntry->state = LE_RESOLVE_ENTRY_STATE_FREE;
					pEntry = pSearch;
					break;
				}
			}

			BTSECMAN_TRACE_PRINTF_1(BTSECMAN_TRACE_MASK_ERROR,
			                           "btsm: found matching IRK for bd:%s",
			                           TRACE_BDADDR1(BTSECMAN_TRACE_MASK_ERROR, pEntry->random_bd)
			                           );

			pEntry->state     = LE_RESOLVE_ENTRY_STATE_POSITIVE;
			pEntry->timestamp = osGetSystemTime();
			btsmDeferredQueueOut();
		}
		else if (pEntry->restartHandle != 0x0000)
		{
			pEntry->state = LE_RESOLVE_ENTRY_STATE_STORE_GET;

			/* get next IRK */
			btsmSendDEVICE_DATA_GET_IND_EXT(pEntry, NULL,
			                                BLUEFACE_BDTYPE_ANY,
			                                DEVICE_DATA_TYPE_SECMAN_IRK_REMOTE,
			                                pEntry->restartHandle
			                                );
		}
		else
		{
			BTSECMAN_TRACE_PRINTF_1(BTSECMAN_TRACE_MASK_ERROR,
			                           "btsm: no matching IRK found for bd:%s",
			                           TRACE_BDADDR1(BTSECMAN_TRACE_MASK_ERROR, pEntry->random_bd)
			                           );

			pEntry->state     = LE_RESOLVE_ENTRY_STATE_NEGATIVE;
			pEntry->timestamp = osGetSystemTime();
			btsmDeferredQueueOut();
		}
		break;

	default:
		break;
	}
} /* btsmHandleHci_LE_ENCRYPT_Conf */

/**
* @brief  bt sm handle le rand confirm
*
* @param  handle
* @param  result
*
* @return  
*
*/
void btsmHandleHci_LE_RAND_CONF(uint16_t handle, uint8_t * result)
{
	uint8_t          temp[LE_ENCRYPT_DATA_LENGTH];
	LPTSECLINK    pLink       = NULL;
	LPLE_SMP_DATA pSMPData    = NULL;
	uint16_t          cryptState  = LE_CRYPT_IDLE;

     if ((handle & LE_CRYPT_HANDLE_LINK) 
	 )
	{		
        uint16_t index = handle & LE_CRYPT_HANDLE_MASK;
		if(index & 0x10)
		    pLink       = &pBtSM->plinksDon[index -0x10];
		else
		    pLink       = &pBtSM->plinksDoff[index];
		
		pSMPData    = pLink->pSMPData;
		cryptState  = pLink->cryptState;
	}
	else
	{
		BTSECMAN_TRACE_PRINTF_1(BTSECMAN_TRACE_MASK_ERROR,
		                           "!!btsmHandleHci_LE_RAND_Conf: no link/SMPData found for handle 0x%x",
		                           handle
		                           );
		return;
	}

	switch (cryptState)
	{
	case LE_CRYPT_GEN_RAND_LO: 
		/* first part of local_RAND generated */
		memcpy(&pSMPData->p.ph2.local_RAND[0], result, LE_RANDOM_NUMBER_SIZE);

		/* trigger second part of local_RAND */
		pLink->cryptState = LE_CRYPT_GEN_RAND_HI;
		btsmSendLHci_LE_RAND(LE_LINK2HANDLE(pBtSM, pLink));
		break;

	case LE_CRYPT_GEN_RAND_HI:
		/* second part of local_RAND generated */
		memcpy(&pSMPData->p.ph2.local_RAND[8], result, LE_RANDOM_NUMBER_SIZE);

		BTSECMAN_TRACE_PRINTF_1(BTSECMAN_TRACE_MASK_TRACE,
		                         "btsm: RAND:%s",
		                         BTRACE_RAMDATA1(BTSECMAN_TRACE_MASK_TRACE, btsmHexString(pSMPData->p.ph2.local_RAND, 16))
		                         );

		/* trigger first round of local_CONF */
		pLink->cryptState = LE_CRYPT_GEN_CONF_1;
		btsmSetupLECryptoConfirmValue1(pLink, temp, pSMPData->p.ph2.local_RAND);
		btsmSendLHci_LE_ENCRYPT(LE_LINK2HANDLE(pBtSM, pLink), pSMPData->p.ph2.local_TK, temp);
		break;

	case LE_CRYPT_GEN_PASSKEY:
		/* create 6digit passkey */
		{
			uint32_t value = btsmGenerateLEPasskey(result);

			/* use application supplied DisplayValue */
			if (pBtSM->LE_fixedDisplayValue != 0xFFFFFFFF)
			{
				value = pBtSM->LE_fixedDisplayValue;

				BTSECMAN_TRACE_PRINTF_1(BTSECMAN_TRACE_MASK_TRACE,
				                         "btsm: using fixed display value %d", value
				                         );
			}

			if (value != 0xFFFFFFFF)
			{
				/* store value as local TK */
				pSMPData->p.ph2.local_TK[0] = (uint8_t)(value & 0xFF);
				pSMPData->p.ph2.local_TK[1] = (uint8_t)((value >> 8) & 0xFF);
				pSMPData->p.ph2.local_TK[2] = (uint8_t)((value >> 16) & 0xFF);

				/* send value to upper layer */
				blueAPI_Send_UserPasskeyNotificationInfo(pLink->bd, value);
				/* start CONF value generation */
				pLink->cryptState = LE_CRYPT_GEN_RAND_LO;
				btsmSendLHci_LE_RAND(LE_LINK2HANDLE(pBtSM, pLink));
			}
			else
			{
				/* no matching value found, get new random data */
				pLink->cryptState = LE_CRYPT_GEN_PASSKEY;
				btsmSendLHci_LE_RAND(LE_LINK2HANDLE(pBtSM, pLink));
			}
		}
		break;

	case LE_CRYPT_GEN_LTK_LO:
		/* first part of local_LTK generated */
		memcpy(&pSMPData->p.ph3.lLTK.key[0], result, LE_RANDOM_NUMBER_SIZE);

		/* trigger second part of local_LTK */
		pLink->cryptState = LE_CRYPT_GEN_LTK_HI;
		btsmSendLHci_LE_RAND(LE_LINK2HANDLE(pBtSM, pLink));
		break;

	case LE_CRYPT_GEN_LTK_HI: 
		/* second part of local_LTK generated */
		memcpy(&pSMPData->p.ph3.lLTK.key[8], result, LE_RANDOM_NUMBER_SIZE);
		pLink->cryptState = LE_CRYPT_IDLE;

		/* shorten LTK if needed */
		if (pLink->keySize < LE_LONG_TERM_KEY_SIZE)
		{
			memset(pSMPData->p.ph3.lLTK.key + pLink->keySize, 0x00, (LE_LONG_TERM_KEY_SIZE - pLink->keySize));
		}

		BTSECMAN_TRACE_PRINTF_2(BTSECMAN_TRACE_MASK_TRACE,
		                         "btsm: LTK:%s keySize:%d",
		                         BTRACE_RAMDATA1(BTSECMAN_TRACE_MASK_TRACE, btsmHexString(pSMPData->p.ph3.lLTK.key, 16)),
		                         pLink->keySize
		                         );

		btsmSendSMP128BitValue(pLink, LE_SMP_ENCRYPTION_INFORMATION, pSMPData->p.ph3.lLTK.key);
		btsmStartSMPTimer(pLink);

		/* continue with key exchange */
		btsmLEKeyExchange(pLink);
		break;

	case LE_CRYPT_GEN_MA_ID_LO:
		/* first part of local_RAND/EDIV generated */
		memcpy(&pSMPData->p.ph3.lLTK.rand, result, LE_RANDOM_NUMBER_SIZE);

		/* trigger second part of local_LTK */
		pLink->cryptState = LE_CRYPT_GEN_MA_ID_HI;
		btsmSendLHci_LE_RAND(LE_LINK2HANDLE(pBtSM, pLink));
		break;

	case LE_CRYPT_GEN_MA_ID_HI:
		/* second part of local_RAND/EDIV generated */
		{
			pSMPData->p.ph3.lLTK.ediv = CHAR2SHORT(result);

			pLink->cryptState = LE_CRYPT_IDLE;
			BTSECMAN_TRACE_PRINTF_2(BTSECMAN_TRACE_MASK_TRACE,
			                           "btsm: EDIV:0x%x RAND:%s",
			                           pSMPData->p.ph3.lLTK.ediv,
			                           BTRACE_RAMDATA1(BTSECMAN_TRACE_MASK_TRACE, btsmHexString(pSMPData->p.ph3.lLTK.rand, 8))
			                           );

			btsmSendSMPMasterIdentification(pLink, pSMPData->p.ph3.lLTK.ediv, pSMPData->p.ph3.lLTK.rand);
			btsmStartSMPTimer(pLink);

			pSMPData->p.ph3.lLTK.keySize  = pLink->keySize;
			pSMPData->p.ph3.lLTK.mitmFlag = (pLink->keyType == BLUEFACE_KEYTYPE_AUTHENTICATED) ? 1 : 0;

			/* continue with key exchange */
			btsmLEKeyExchange(pLink);
		}
		break;
	default:
		break;
	}
} /* btsmHandleHci_LE_RAND_Conf */

/**
* @brief  bt sm handle hci le long term key request event
*
* @param  ediv
* @param  rand
* @param  handle
*
* @return  
*
*/
void btsmHandleHci_LE_LONG_TERM_KEY_REQUEST_EVENT(uint16_t ediv, uint8_t * rand, uint16_t handle)
{
	LPTSECLINK pLink = btsmFindLELinkByHandle(handle);

	BTSECMAN_TRACE_PRINTF_3(BTSECMAN_TRACE_MASK_TRACE,
	                         "btsmHandleHci_LE_LONG_TERM_KEY_REQUEST_EVENT: handle:0x%x ediv:0x%x rand:%s",
	                         handle, ediv,
	                         BTRACE_RAMDATA1(BTSECMAN_TRACE_MASK_TRACE, btsmHexString(rand, 8))
	                         );

	if (pLink == NULL)
	{
		return;
	}

	if (pLink->role == LE_ROLE_SLAVE)
	{
		uint8_t nullRand[LE_RANDOM_NUMBER_SIZE];

		memset(nullRand, 0x00, LE_RANDOM_NUMBER_SIZE);

		/* STK is identified with ediv and rand == 0x00 */
		if ((ediv == 0x0000) && (memcmp(rand, nullRand, LE_RANDOM_NUMBER_SIZE) == 0))
		{
			if (pLink->pSMPData != NULL)
			{
				if (pLink->pSMPData->smpState & LE_SMPSTATE_STK_VALID)
				{
					/* use generated STK */
					btsmSendLHci_LE_LONGTERM_KEY_REQUEST_REPLY(pLink->handle, pLink->pSMPData->p.stk);
				}
				else
				{
					pLink->pSMPData->smpState |= LE_SMPSTATE_STK_REQUESTED;
				}
			}
			else
			{
				btsmSendLHci_LE_LONGTERM_KEY_REQUEST_NEGATIVE_REPLY(pLink->handle);
			}
		}
		else
		{
			btsmUpdateSecurityStatus(pLink, SECSTAT_LTK_REQ, HCI_SUCCESS);
			/* ignore value of ediv and rand, only one bond per bd/bdType */
			btsmSendDEVICE_DATA_GET_IND(pLink, DEVICE_DATA_TYPE_SECMAN_LTK_LOCAL);
		}
	}
} /* btsmHandleHci_LE_LONG_TERM_KEY_REQUEST_EVENT */

/**
* @brief  btsm handle hci le disconnect event
*
* @param  handle
*
* @return  
*
*/
void btsmHandleLE_HCI_DISCONNECTION_COMPLETE(uint16_t handle)
{
	LPTSECLINK pLink = btsmFindLELinkByHandle(handle);

	if (pLink == NULL)
	{
		return;
	}

	if (pLink->state != SEC_LINK_IDLE)
	{
		btsmAuthenticationComplete(pLink, HCI_ERR | HCI_ERR_AUTHENTICATION_FAILED);
	}

	btsmCancelSMPTimer(pLink);
	btsmDeAllocateSMPData(pLink);

	{
		uint16_t i;

		/* flush cache entry */
		for (i = 0; i < LE_RESOLVE_CACHE_SIZE; i++)
		{
			PRESOLVEENTRY pSearch = &pBtSM->LE_resolveCache[i];

			if ((pSearch->state != LE_RESOLVE_ENTRY_STATE_FREE) &&
			  (memcmp(pSearch->random_bd, pLink->unresolved_bd, BD_ADDR_SIZE) == 0)
			 )
			{
				BTSECMAN_TRACE_PRINTF_1(BTSECMAN_TRACE_MASK_ERROR,
				                           "btsm: removing bd:%s from resolving cache",
				                           TRACE_BDADDR1(BTSECMAN_TRACE_MASK_ERROR, pSearch->random_bd)
				                           );

				pSearch->state = LE_RESOLVE_ENTRY_STATE_FREE;
				break;
			}
		}
	}

	btsmDeAllocateLink(pLink);
} /* btsmHandleLE_HCI_DISCONNECTION_COMPLETE */

/**
* @brief  btsm Handle HCI_ENCRYPTION_CHANGE   
*
* @param  handle
* @param  encrypted
* @param  status
*
* @return  
*
*/
void btsmHandleHci_LE_ENCRYPTION_CHANGE(uint16_t handle, uint8_t encrypted, uint16_t status)
{
	LPTSECLINK pLink     = btsmFindLELinkByHandle(handle);

	BTSECMAN_TRACE_PRINTF_3(BTSECMAN_TRACE_MASK_TRACE,
	                         "btsmHandleHci_LE_ENCRYPTION_CHANGE: handle:0x%x status:0x%x encrypted:%d",
	                         handle, status, encrypted
	                         );

	if (pLink == NULL)
	{
		return;
	}

	if (status == 0x00)
	{
		btsmSendSECURITY_STATUS(encrypted ? CONF_SECURITY_STATUS_ENCRYPTED : CONF_SECURITY_STATUS_NOT_ENCRYPTED,
	                                      pLink->bd, pLink->bdType, status, pLink->keyType, pLink->keySize);
	}
	
	if (encrypted)
	{
		pLink->mode |= LINKMODE_ENCRYPTED;

		if (pLink->pSMPData == NULL)
		{
			btsmUpdateSecurityStatus(pLink, SECSTAT_LE_ENCRYPTION_CHANGE, SECMAN_SUCCESS);
			btsmAuthenticationComplete(pLink, SECMAN_SUCCESS);
		}
		else
		{
			/* clear union */
			memset(&pLink->pSMPData->p.ph3, 0x00, sizeof(TLE_SMP_DATA_PHASE3));
			pLink->pSMPData->smpState &= ~(LE_SMPSTATE_STK_VALID | LE_SMPSTATE_LOCAL_CONF_VALID);
			btsmLEKeyExchange(pLink);
		}
	}
	else
	{
		pLink->mode &= ~(LINKMODE_ENCRYPTED);
		pLink->keyType = BLUEFACE_KEYTYPE_DELETE;
		pLink->keySize = 0;

		/* delete key if still pairable */
		if ((status == (HCI_ERR | HCI_ERR_KEY_MISSING)) &&
		    (pBtSM->devSecurity.pairable_mode)
		   )
		{
			btsmSendDEVICE_DATA_SET_IND(pLink, DEVICE_DATA_TYPE_SECMAN_LTK_REMOTE, NULL, 0);
		}

		btsmAuthenticationComplete(pLink, status);
	}
} /* btsmHandleHci_LE_ENCRYPTION_CHANGE */

/**
* @brief  btsm handle device data get respons
*
* @param  BufferAddress:
* @param  offset
*
* @return  
*
*/
void btsmHandleDEVICE_DATA_GET_RESP(PDeviceDataResp pDevDataResp)
{
	LPTSECLINK      pLink        = (LPTSECLINK)pDevDataResp->handle;

	BTSECMAN_TRACE_PRINTF_5(BTSECMAN_TRACE_MASK_TRACE,
	                         "btsmHandleDEVICE_DATA_GET_RESP: bd:%s keyType:0x%x status:0x%x count:%d restart:0x%x",
	                         TRACE_BDADDR1(BTSECMAN_TRACE_MASK_TRACE, pDevDataResp->deviceData.bd),
	                         pDevDataResp->deviceData.dataType,
	                         pDevDataResp->status,
	                         pDevDataResp->deviceData.elementCount,
	                         pDevDataResp->deviceData.restartHandle
	                         );

	switch (pDevDataResp->deviceData.dataType)
	{
	case DEVICE_DATA_TYPE_SECMAN_LTK_LOCAL:
		/* triggered by LE_LONGTERM_KEY_REQUEST */
		if (pLink->role == LE_ROLE_SLAVE)
		{
			if ((pDevDataResp->status == 0x00) &&
			    !(pLink->mode & LINKMODE_FORCE_PAIRING)
			   )
			{
				PDEVICE_DATA_ELEMENT_SECMAN_LTK pLTK = &pDevDataResp->deviceData.p.leLTK;

				pLink->keySize = pLTK->keySize;

				if (pLTK->mitmFlag)
				{
					pLink->keyType = BLUEFACE_KEYTYPE_AUTHENTICATED;
					pLink->mode   |= (LINKMODE_AUTHEN | LINKMODE_MITM | LINKMODE_STORED_KEY);
				}
				else
				{
					pLink->keyType = BLUEFACE_KEYTYPE_UNAUTHENTICATED;
					pLink->mode   |= (LINKMODE_AUTHEN | LINKMODE_STORED_KEY);
				}

				btsmSendLHci_LE_LONGTERM_KEY_REQUEST_REPLY(pLink->handle, pLTK->key);
			}
			else
			{
				btsmSendLHci_LE_LONGTERM_KEY_REQUEST_NEGATIVE_REPLY(pLink->handle);
				/* no further events on slave side */
				btsmAuthenticationComplete(pLink, HCI_ERR | HCI_ERR_KEY_MISSING);
			}
		}
		break;

	case DEVICE_DATA_TYPE_SECMAN_LTK_REMOTE: 
		/* triggered by SMP_SECURITY_REQUEST or SECMAN_AUTH_REQ */
		if (pLink->role == LE_ROLE_MASTER)
		{
			PDEVICE_DATA_ELEMENT_SECMAN_LTK pLTK = &pDevDataResp->deviceData.p.leLTK;

			/* no key found, or key not sufficient (mitm requirement/keysize) */
			if ((pDevDataResp->status != 0x00) ||
			    (pLink->mode & LINKMODE_FORCE_PAIRING) ||
			    ((pLink->mode & LINKMODE_MITM_REQUIRED) && !(pLTK->mitmFlag)) ||
			    ((pLink->state == SEC_LINK_SECURITY_MITM_REQ) && !(pLTK->mitmFlag)) ||
			    (pLTK->keySize < pLink->minKeySize)
			   )
			{
				if (!pBtSM->devSecurity.pairable_mode)
				{
					/* triggered by SecurityReq from slave */
					if ((pLink->state == SEC_LINK_SECURITY_REQ) ||
					    (pLink->state == SEC_LINK_SECURITY_MITM_REQ)
					   )
					{
						btsmSendSMPPairingFailed(pLink, LE_SMP_ERROR_PAIRING_NOT_SUPPORTED);
					}

					btsmAuthenticationComplete(pLink, HCI_ERR | HCI_ERR_PARING_NOT_ALLOWED);
				}
				else
				{
					uint16_t keyDist = btsmGetSMPKeyDistribution(pLink);

					/* SMP data not allocated yet */
					if (pLink->pSMPData == NULL)
					{
						if (btsmAllocateSMPData(pLink) == NULL)
						{
							BTSECMAN_TRACE_PRINTF_1(BTSECMAN_TRACE_MASK_ERROR,
							                           "!!btsmHandleDEVICE_DATA_GET_RESP: failed to allocate SMPData for handle 0x%x",
							                           pLink->handle
							                           );
							return;
						}
					}

					btsmSendSMPPairingExchange(pLink, LE_SMP_PAIRING_REQUEST,
					                          pBtSM->devSecurity.io_capabilities,
					                          pBtSM->devSecurity.oob_present,
					                          btsmGetSMPAuthRequirement(pLink),
					                          pBtSM->LE_maxKeySize,
					                          (keyDist >> 8), (keyDist & 0xFF)
					                          );

					btsmStartSMPTimer(pLink);
				}
			}
			else
			{
				pLink->keySize = pLTK->keySize;

				if (pLTK->mitmFlag)
				{
					pLink->keyType = BLUEFACE_KEYTYPE_AUTHENTICATED;
					pLink->mode   |= (LINKMODE_AUTHEN | LINKMODE_MITM | LINKMODE_STORED_KEY);
				}
				else
				{
					pLink->keyType = BLUEFACE_KEYTYPE_UNAUTHENTICATED;
					pLink->mode   |= (LINKMODE_AUTHEN | LINKMODE_STORED_KEY);
				}

				/* key found and sufficient */
				btsmSendLHci_LE_START_ENCRYPTION(pLink->handle, pLTK->rand, pLTK->ediv, pLTK->key);
			}
		}
		break;
	case DEVICE_DATA_TYPE_SECMAN_IRK_REMOTE: 
		{
			PRESOLVEENTRY pEntry = (PRESOLVEENTRY)pDevDataResp->handle;

			if (pDevDataResp->status == 0x00)
			{
				PDEVICE_DATA_ELEMENT_SECMAN_IRK pIRK = &pDevDataResp->deviceData.p.leIRK;
				uint8_t                            buffer[LE_ENCRYPT_DATA_LENGTH];

				/* store (possible) public BD in entry */
				pEntry->resolved_bdType = pDevDataResp->deviceData.bdType;
				memcpy(pEntry->resolved_bd, pDevDataResp->deviceData.bd, BD_ADDR_SIZE);

				/* store restartHandle for more IRKs */
				pEntry->restartHandle = pDevDataResp->deviceData.restartHandle;

				pEntry->state = LE_RESOLVE_ENTRY_STATE_RESOLVING;
				memset(buffer, 0x00, LE_ENCRYPT_DATA_LENGTH);
				memcpy(&buffer[0], &pEntry->random_bd[3], 3);
				btsmSendLHci_LE_ENCRYPT(LE_CACHE2HANDLE(pBtSM, pEntry), pIRK->key, buffer);
			}
			else
			{
				BTSECMAN_TRACE_PRINTF_1(BTSECMAN_TRACE_MASK_ERROR,
				                         "btsm: no matching IRK found for bd:%s",
				                         TRACE_BDADDR1(BTSECMAN_TRACE_MASK_ERROR, pEntry->random_bd)
				                         );

				/* entry could not be resolved, keep in negative cache */
				pEntry->state     = LE_RESOLVE_ENTRY_STATE_NEGATIVE;
				pEntry->timestamp = osGetSystemTime();
				btsmDeferredQueueOut();
			}
		}
		break;
	default:
		break;
	}
} /* btsmHandleDEVICE_DATA_GET_RESP */

/**
* @brief  btsm handle device data set respons
*
* @param  BufferAddress:
* @param  offset
*
* @return  
*
*/
void btsmHandleDEVICE_DATA_SET_RESP(PDeviceDataResp pDevDataResp)
{
	LPTSECLINK      pLink        = (LPTSECLINK)pDevDataResp->handle;
	LPLE_SMP_DATA   pSMPData     = NULL;

	BTSECMAN_TRACE_PRINTF_5(BTSECMAN_TRACE_MASK_TRACE,
	                         "btsmHandleDEVICE_DATA_SET_RESP: bd:%s keyType:0x%x status:0x%x count:%d restart:0x%x",
	                         TRACE_BDADDR1(BTSECMAN_TRACE_MASK_TRACE, pDevDataResp->deviceData.bd),
	                         pDevDataResp->deviceData.dataType,
	                         pDevDataResp->status,
	                         pDevDataResp->deviceData.elementCount,
	                         pDevDataResp->deviceData.restartHandle
	                         );

/* (F_BT_LE_PRIVACY_MODE) */

	if ((pLink == NULL) || (pLink->pSMPData == NULL))
	{
		return;
	}
	else if (pDevDataResp->status != 0x00)
	{
		btsmSendSMPPairingFailed(pLink, LE_SMP_ERROR_UNSPECIFIED_REASON);
		btsmAuthenticationComplete(pLink, HCI_ERR | HCI_ERR_AUTHENTICATION_FAILED);
		return;
	}

	pSMPData = pLink->pSMPData;

	switch (pDevDataResp->deviceData.dataType)
	{
    case DEVICE_DATA_TYPE_SECMAN_LTK_LOCAL:
		pSMPData->p.ph3.localKeysStored |= LE_SMP_KEYDIST_ENCKEY;
		break;
    case DEVICE_DATA_TYPE_SECMAN_LTK_REMOTE:
		pSMPData->p.ph3.remoteKeysStored |= LE_SMP_KEYDIST_ENCKEY;
		break;

    case DEVICE_DATA_TYPE_SECMAN_IRK_REMOTE:
		pSMPData->p.ph3.remoteKeysStored |= LE_SMP_KEYDIST_IDKEY;
		break;

    default:
		return;
	}

	pSMPData->smpState &= ~(LE_SMPSTATE_DEVICE_DATA_STORE);
	btsmLEKeyExchange(pLink);
} /* btsmHandleDEVICE_DATA_SET_RESP */

/**
* @brief  bt sm trigger crypt queue
*
*
* @return  
*
*/
void btsmTriggerCryptQueue()
{
	MESSAGE_T   Message;
	LPLEMessage pLEMsg;

	/* Serialize LE_RAND and LE_ENCRYPT and use ->cryptHandle to store the handle */
	if (pBtSM->cryptHandle == LE_CRYPT_HANDLE_IDLE &&
	!osMessageReceive(pBtSM->cryptQueueID, &Message)
	)
	{
		/* store transaction handle */
		pLEMsg = (LPLEMessage)(Message.MData.DataCB.BufferAddress + Message.MData.DataCB.Offset);
		pBtSM->cryptHandle = pLEMsg->handle;

		if(pLEMsg->msgType == LE_RAND)
		{
			hciCommandNoParameter(HCI_LE_RAND);	
		}else if(pLEMsg->msgType == LE_ENCRYPT)
		{
            hciCommandLEEncrypt(pLEMsg->p.Encrypt.key, pLEMsg->p.Encrypt.plaintextData);
			hciLaterEntry();		//randy---619
		}
//		hciLaterEntry();		//randy---619
		osBufferRelease(Message.MData.DataCB.BufferAddress);

	}
} /* btsmTriggerCryptQueue */

void btsmHandleCmdCompLE_RAND(uint8_t * result)
{
    uint8_t	randomNumber[LE_RANDOM_NUMBER_SIZE];
			
	memcpy(randomNumber, result, LE_RANDOM_NUMBER_SIZE);

	btsmHandleHci_LE_RAND_CONF(pBtSM->cryptHandle, randomNumber);
	pBtSM->cryptHandle = LE_CRYPT_HANDLE_IDLE;

	btsmTriggerCryptQueue();			 /* trigger transaction queue  */
}

void btsmHandleHCI_ENCRYPTION_KEY_REFRESH_COMPLETE(uint16_t handle, uint8_t status)
{
    LPTSECLINK pLink = btsmFindLELinkByHandle(handle);

    BTSECMAN_TRACE_PRINTF_2(BTSECMAN_TRACE_MASK_TRACE,
                             "btsmHandleHci_LE_ENCRYPTION_KEY_REFRESH_COMPLETE: handle:0x%x status:0x%x",
                             handle, status
                             );

    if (pLink == NULL)
    {
    	return;
    }

    if (status == HCI_SUCCESS)
    {
    	pLink->mode |= LINKMODE_ENCRYPTED;

    	if (pLink->pSMPData == NULL)
    	{

    		btsmUpdateSecurityStatus(pLink, SECSTAT_LE_ENCRYPTION_CHANGE, SECMAN_SUCCESS);
    		/* F_BT_SECMAN_STATUS_UPDATES */
    		btsmAuthenticationComplete(pLink, SECMAN_SUCCESS);
    	}
    	else
    	{
    		/* clear union */
    		memset(&pLink->pSMPData->p.ph3, 0x00, sizeof(TLE_SMP_DATA_PHASE3));
    		pLink->pSMPData->smpState &= ~(LE_SMPSTATE_STK_VALID | LE_SMPSTATE_LOCAL_CONF_VALID);
    		btsmLEKeyExchange(pLink);
    	}
    }
}

void btsmHandleHCI_LE_ENCRYPT(uint8_t * data)
{
    uint8_t    encryptedData[LE_ENCRYPT_DATA_LENGTH];
	memcpy(encryptedData, data, LE_ENCRYPT_DATA_LENGTH);

	btsmHandleHci_LE_ENCRYPT_CONF(pBtSM->cryptHandle, encryptedData);
	pBtSM->cryptHandle = LE_CRYPT_HANDLE_IDLE;

	btsmTriggerCryptQueue();			 /* trigger transaction queue  */
}

void btsmSendLEMsg(uint16_t Command, LPLEMessage pLEMsg, uint16_t Length)
{
  MESSAGE_T Message;

  Message.Command        = Command;
  Message.MData.DataCB.BufferAddress = (uint8_t *)pLEMsg;
  Message.MData.DataCB.Offset        = 0;
  Message.MData.DataCB.Flag          = DATA_CB_RELEASE;
  Message.MData.DataCB.Length        = Length;
  osMessageSend(btsmQueueID, &Message);
}

BOOL btsmHandleLE_MESSAGE_IND()
{
    LPLEMessage pLEMsg  = (LPLEMessage)(pBtSM->message.MData.DataCB.BufferAddress +
                                  pBtSM->message.MData.DataCB.Offset);
    BOOL        forward = TRUE;
    BOOL        release = TRUE;

    switch (pLEMsg->msgType)
    {
    case LE_CONNECTION_COMPLETE_EVENT: /* ********************************** */
    {

        TBdAddr peerAddress;
        uint8_t    peerAddressType;
        uint8_t    state;

        /* save original BD */
        memcpy(peerAddress, pLEMsg->p.ConnectionComplete.peerAddress, BD_ADDR_SIZE);
        peerAddressType = pLEMsg->p.ConnectionComplete.peerAddressType;

        state = btsmResolveRandomAddress(pLEMsg->p.ConnectionComplete.peerAddress,
                                         &pLEMsg->p.ConnectionComplete.peerAddressType);

        /* queue all connect complete events */
        if (state & LE_RESOLVE_ENTRY_STATE_RESOLVING_MASK)
        {
            forward = FALSE;
            release = FALSE;
            btsmDeferredQueueIn(NULL);
        }
        else
        /* (F_BT_LE_PRIVACY_RESOLVING) */
        {
          LPTSECLINK pLink;

          if (pLEMsg->status == 0)
          {
                pLink = btsmFindAllocateLink(pLEMsg->p.ConnectionComplete.peerAddressType,
                                         pLEMsg->p.ConnectionComplete.peerAddress
                                         );
                if (pLink != NULL)
                {
                pLink->handle          = pLEMsg->handle;
                pLink->role            = pLEMsg->p.ConnectionComplete.role;
                pLink->minKeySize      = 7; /* FIXME: minimum allowed keysize */
                pLink->ownAddressType     = pBtSM->localBdType;

                /* store original BD for SMP CONF value generation */
                memcpy(pLink->unresolved_bd, peerAddress, BD_ADDR_SIZE);
                pLink->unresolved_bdType = peerAddressType;
                /* (F_BT_LE_PRIVACY_RESOLVING) */
            }
          }
        }
        if(forward)
        {
            gattHandleHCI_LE_CONNECTION_COMPLETE_EVENT(pLEMsg->handle, pLEMsg->status, 
                                             pLEMsg->p.ConnectionComplete.peerAddressType,
			                                 pLEMsg->p.ConnectionComplete.peerAddress, 
			                                 pLEMsg->p.ConnectionComplete.connInterval,
			                                 pLEMsg->p.ConnectionComplete.connLatency,
			                                 pLEMsg->p.ConnectionComplete.supervisionTimeout);
        }
    }
    break;

    case LE_ADVERTISING_REPORT_EVENT: /* *********************************** */
        if (pLEMsg->p.AdvertisingReport.eventType != LE_EVENT_TYPE_ADV_DIRECT_IND)
        {
            uint8_t state = btsmResolveRandomAddress(pLEMsg->p.AdvertisingReport.address,
                                                  &pLEMsg->p.AdvertisingReport.addressType);

            /* do not forward message */
            if (state & LE_RESOLVE_ENTRY_STATE_RESOLVING_MASK)
            {
                forward = FALSE;

                /* address resolving started, queue only first advertising event */
                if (state == LE_RESOLVE_ENTRY_STATE_QUEUE)
                {
                    btsmDeferredQueueIn(NULL);
                    release = FALSE;
                }
            }
        }
        if(forward)
        {
            gattHandleHCI_LE_ADVERTISING_REPORT_EVENT(pLEMsg->p.AdvertisingReport.eventType,
                                                  pLEMsg->p.AdvertisingReport.addressType, 
                                                  pLEMsg->p.AdvertisingReport.address, 
                                                  pLEMsg->p.AdvertisingReport.rssi, 
                                                  pLEMsg->p.AdvertisingReport.dataLength, 
                                                  pLEMsg->p.AdvertisingReport.data);
        }
    break;
    default: /* ************************************************************ */
    break;
    }

    return release; /* release buffer */
}
#if F_BT_LE_BT41_SUPPORT
void btsmLEHandleAuthenticationInd()
{
    PTSecManAuthenticationInd auth = &pBtSM->message.MData.secManAuthenticationInd;
    LPTSECLINK link                = btsmFindLELinkByHandle(auth->uuid);
    BOOL reply                     = TRUE; /* do reply with AUTH_RESP */
    LPTSEC_CONF_LE_SECURITY entry     = NULL;
    uint16_t cause = LE_CREDIT_CONNECTION_SUCCESS;

    BTSECMAN_TRACE_PRINTF_5(BTSECMAN_TRACE_MASK_TRACE,
                            "btsmHandleAuthenticationInd: cid:0x%x psm:0x%x keytype:0x%x keySize %d mode 0x%X",
                            auth->ref, auth->channelId, link->keyType, link->keySize, link->mode
                           );

    do
    {
        if (!link)
        {
            cause = LE_CREDIT_ERR_LINK_NOT_EXIST;
            break;
        }

        if (link->state != SEC_LINK_IDLE)
        {
            btsmDeferredQueueIn(link);
            reply = FALSE;
            break;
        }

        entry = btsmFindLESecEntry(auth->channelId);

        if (entry)
        {
            switch (entry->secMode)
            {
            case AUTHEN_SETTING_AUTHORIZATION:
                cause = LE_CREDIT_ERR_INSUFFICIENT_AUTHORIZATION;
                break;
            case AUTHEN_SETTING_UNAUTHENTICATED_ENCRYTION:
                if ((link->keyType == BLUEFACE_KEYTYPE_AUTHENTICATED)
                        || (link->keyType == BLUEFACE_KEYTYPE_UNAUTHENTICATED))
                {
                    cause = LE_CREDIT_CONNECTION_SUCCESS;
                }
                else
                {
                    cause = LE_CREDIT_ERR_INSUFFICIENT_ENCRYPTION;
                }
                break;
            case AUTHEN_SETTING_AUTHENTICATED_ENCRYTION:
                if (link->keyType == BLUEFACE_KEYTYPE_AUTHENTICATED)
                {
                    cause = LE_CREDIT_CONNECTION_SUCCESS;
                }
                else
                {
                    cause = LE_CREDIT_ERR_INSUFFICIENT_AUTHENTICATION;
                }
                break;
            default:
                break;
            }
            if ((cause == LE_CREDIT_CONNECTION_SUCCESS) && (link->keySize != 0))
            {
                if (entry->keySize > link->keySize)
                {
                    cause = LE_CREDIT_ERR_INSUFFICIENT_ENCRYPTION_KEY_SIZE;
                }
            }

        }
        else
        {
            break;
        }
    }
    while (0);

    if (reply)
    {
        l2cHandleSECMAN_AUTHENTICATION_RESP(auth->bd, auth->ref, auth->channelId, 0, auth->outgoing, cause);
    }
}
#endif

/**
* @brief  btsm send config resolved address
* 
* @param  bd
* @param  bdType
* @param  resolvedBd
* @param  resolvedBdType
*
* @return 
*
*/
void btsmSendBLUEFACE_CONF_RESOLVED_ADDRESS(uint8_t * bd, uint8_t bdType, uint8_t * resolvedBd, uint8_t resolvedBdType)
{
	gattHandleBtsmResolvedAddress(bd, bdType, resolvedBd, resolvedBdType);
    blueAPI_Handle_BTSM_RESOLVED_ADDRESS(bd, bdType, resolvedBd, resolvedBdType);
} /* btsmSendBLUEFACE_CONF_RESOLVED_ADDRESS */

/**
* @brief  get data indicator ext
*
* @param  pLink: 
* @param  keyType
*
* @return  
*
*/
void btsmSendDEVICE_DATA_GET_IND_EXT(PVOID handle, TBdAddr bd, uint8_t bdType, uint8_t keyType, uint16_t restartHandle)
{
	MESSAGE_T      msg;
	PDeviceDataInd pDevDataInd;

	BTSECMAN_TRACE_PRINTF_2(BTSECMAN_TRACE_MASK_TRACE,
	                         "btsmSendDEVICE_DATA_GET_IND_EXT: bd:%s keyType:0x%x",
	                         TRACE_BDADDR1(BTSECMAN_TRACE_MASK_TRACE, bd), keyType
	                         );

	if (btsmDataCBBufferGet(&msg, DEVICE_DATA_GET_IND,
	                      (PVOID)&pDevDataInd, sizeof(TDEVICE_DATA_IND),
	                      offsetof(TblueFaceMsg, p.deviceDataIndication)
	                     ))
	{
		return;
	}

	pDevDataInd->handle                   = handle;
	pDevDataInd->deviceData.bdType        = bdType;
	pDevDataInd->deviceData.dataType      = keyType;
	pDevDataInd->deviceData.restartHandle = restartHandle;
	pDevDataInd->deviceData.elementCount  = 0;

	if (bd != NULL)
	{
		memcpy(pDevDataInd->deviceData.bd, bd, BD_ADDR_SIZE);
	}
	else
	{
		memset(pDevDataInd->deviceData.bd, 0x00, BD_ADDR_SIZE);
	}

	blueFaceSendBT_DEVICE_DATA_IND(&msg);

} /* btsmSendDEVICE_DATA_GET_IND_EXT */

/**
* @brief  get data indicator
*
* @param  pLink: 
* @param  keyType
*
* @return  
*
*/
void btsmSendDEVICE_DATA_GET_IND(LPTSECLINK pLink, uint8_t keyType)
{
	btsmSendDEVICE_DATA_GET_IND_EXT((PVOID)pLink, pLink->bd, pLink->bdType, keyType, 0x0000);
} /* btsmSendDEVICE_DATA_GET_IND */

/**
* @brief  btsm send device data set indicate ext
*
* @param  handle
* @param  bd
* @param  bdType
* @param  keyType
* @param  keyData
* @param  keyLength
* @param  restartHandle
*
* @return  
*
*/
void btsmSendDEVICE_DATA_SET_IND_EXT(PVOID handle, TBdAddr bd, uint8_t bdType, uint8_t keyType, PVOID keyData, uint16_t keyLength, uint16_t restartHandle)
{
	MESSAGE_T      msg;
	PDeviceDataInd pDevDataInd;

	BTSECMAN_TRACE_PRINTF_3(BTSECMAN_TRACE_MASK_TRACE,
	                     "btsmSendDEVICE_DATA_SET_IND_EXT: bd:%s keyType:0x%x keyLength:%d",
	                     TRACE_BDADDR1(BTSECMAN_TRACE_MASK_TRACE, bd), keyType, keyLength
	                     );

	if (btsmDataCBBufferGet(&msg, DEVICE_DATA_SET_IND,
	                  (PVOID)&pDevDataInd, sizeof(TDEVICE_DATA_IND),
	                  offsetof(TblueFaceMsg, p.deviceDataIndication)
	                 ))
	{
		return;
	}

	pDevDataInd->handle                   = handle;
	pDevDataInd->deviceData.bdType        = bdType;
	pDevDataInd->deviceData.dataType      = keyType;
	pDevDataInd->deviceData.restartHandle = restartHandle;
	pDevDataInd->deviceData.elementCount  = (keyLength > 0) ? 1 : 0;
	if (bd != NULL)
	{
		memcpy(pDevDataInd->deviceData.bd, bd, BD_ADDR_SIZE);
	}
	else
	{
		memset(pDevDataInd->deviceData.bd, 0x00, BD_ADDR_SIZE);
	}

	if (keyLength)
	{
		memcpy(pDevDataInd->deviceData.p.data, keyData, keyLength);
	}

	blueFaceSendBT_DEVICE_DATA_IND(&msg);

} /* btsmSendDEVICE_DATA_SET_IND_EXT */

/**
* @brief  btsm send device data set indicate
*
* @param  pLink
* @param  keyType
* @param  keyData
* @param  keyLength
*
* @return  
*
*/
void btsmSendDEVICE_DATA_SET_IND(LPTSECLINK pLink, uint8_t keyType, PVOID keyData, uint16_t keyLength)
{
	btsmSendDEVICE_DATA_SET_IND_EXT((PVOID)pLink, pLink->bd, pLink->bdType, keyType, keyData, keyLength, 0x0000);
} /* btsmSendDEVICE_DATA_SET_IND */

