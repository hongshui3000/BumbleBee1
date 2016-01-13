/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       btsmprot.h
* @brief     Bluetooth Security Manager 2.1
* @details   
*
* @author   	gordon
* @date      	2015-07-10
* @version	v0.1
*/

#if !defined(__BTSMPROT_H)
#define      __BTSMPROT_H

#include <btsm.h>

//btsmmsg.c
void btsmHandleMessage                        (void);
void btsmSendSECURITY_STATUS    (uint8_t indId, uint8_t * bd, uint8_t bdType, uint16_t cause, uint8_t keyType, uint8_t keySize);
void btsmUpdateSecurityStatus             (LPTSECLINK link, uint8_t secStat, uint16_t cause);
void btsmAuthenticationComplete           (LPTSECLINK pLink, uint16_t cause);


//btsm_le.c
BOOL btsmHandleLE_MESSAGE_IND      (void);
void btsmSetupLECryptoConfirmValue1  (LPTSECLINK pLink, uint8_t * pBuffer, uint8_t * pRand);
void btsmTriggerCryptQueue         (void);
void btsmSendBLUEFACE_CONF_RESOLVED_ADDRESS   (uint8_t * bd, uint8_t bdType, uint8_t * resolvedBd, uint8_t resolvedBdType);
/** BLE towards HCI */
void btsmSendLHci_LE_ENCRYPT                              (uint16_t handle, uint8_t * key, uint8_t * plaintextData);
void btsmSendLHci_LE_RAND                                 (uint16_t handle);
void btsmSendLHci_LE_START_ENCRYPTION                     (uint16_t handle, uint8_t * randomNumber, uint16_t encryptedDiversifier, uint8_t * longTermKey);
void btsmSendLHci_LE_LONGTERM_KEY_REQUEST_REPLY           (uint16_t handle, uint8_t * longTermKey);
void btsmSendLHci_LE_LONGTERM_KEY_REQUEST_NEGATIVE_REPLY  (uint16_t handle);
void btsmSendLHci_LE_DISCONNECT                           (uint16_t handle, uint16_t cause);
/** BLE key storage */
void btsmSendDEVICE_DATA_GET_IND_EXT      (PVOID handle, TBdAddr bd, uint8_t bdType, uint8_t keyType, uint16_t restartHandle);
void btsmSendDEVICE_DATA_GET_IND          (LPTSECLINK pLink, uint8_t keyType);
void btsmSendDEVICE_DATA_SET_IND_EXT      (PVOID handle, TBdAddr bd, uint8_t bdType, uint8_t keyType, PVOID keyData, uint16_t keyLength, uint16_t restartHandle);
void btsmSendDEVICE_DATA_SET_IND          (LPTSECLINK pLink, uint8_t keyType, PVOID keyData, uint16_t keyLength);
void btsmHandleHci_LE_RAND_CONF(uint16_t handle, uint8_t * result);
void btsmHandleHci_LE_ENCRYPT_CONF(uint16_t handle, uint8_t * result);
#if F_BT_LE_BT41_SUPPORT
void btsmLEHandleAuthenticationInd(void);
#endif

//btsmutil_le.c
LPLE_SMP_DATA btsmAllocateSMPData           (LPTSECLINK pLink);
void          btsmDeAllocateSMPData         (LPTSECLINK pLink);
#if F_BT_LE_BT41_SUPPORT
LPTSEC_CONF_LE_SECURITY btsmAllocateLESecEntry(LPTSEC_CONF_LE_SECURITY newEntry);
uint16_t btsmDeAllocateLESecEntry(LPTSEC_CONF_LE_SECURITY delEntry);
LPTSEC_CONF_LE_SECURITY btsmFindLESecEntry(uint16_t psm);
BOOL btsmCheckNewLESecEntry(LPTSEC_CONF_LE_SECURITY sec);
#endif
uint8_t  btsmGetSMPAuthRequirement               (LPTSECLINK pLink);
uint16_t  btsmGetSMPKeyDistribution               (LPTSECLINK pLink);
void   btsmStartSMPTimer      (LPTSECLINK pLink);
void   btsmCancelSMPTimer     (LPTSECLINK pLink);
int btsmDataCBBufferGet(MESSAGE_P pMessage, uint16_t command,
                               void  *  *BufferPtr, uint16_t length, uint16_t offset);
int btsmLEMsgBufferGet2(MESSAGE_P pMessage, LPLEMessage * pLEMsg, uint16_t length, uint16_t msgType, uint16_t handle);
LPTSECLINK            btsmFindLELinkByHandle  (uint16_t handle);


/** btsm_smp.c */
void btsmLEKeyExchange                    (LPTSECLINK pLink);
/** BLE security manager protocol */
void btsmSendSMPPairingExchange           (LPTSECLINK pLink, uint8_t command, uint8_t ioCap, uint8_t oobFlag, uint8_t authReq, uint8_t maxKeySize, uint8_t initKeyDist, uint8_t respKeyDist);
void btsmSendSMP128BitValue               (LPTSECLINK pLink, uint8_t command, uint8_t * value);
void btsmSendSMPPairingFailed             (LPTSECLINK pLink, uint8_t cause);
void btsmSendSMPMasterIdentification      (LPTSECLINK pLink, uint16_t ediv, uint8_t * rand);
void btsmSendSMPSecurityRequest(LPTSECLINK pLink, uint8_t authReq);


/*btsm_br.c*/
/** towards HCI */
#if F_BT_BREDR_SUPPORT
void btsmSendLHciAuthReq                      (LPCBYTE bd, uint8_t id);
void btsmSendLHciKeyResp                      (LPCBYTE bd, uint8_t reqType, uint16_t cause, LPCBYTE key, uint16_t keyLen);
void btsmSendLHciSspIoCapabilityReplyReq      (uint8_t * bd, uint8_t capability, uint8_t oob_data_present, uint8_t auth_requirements, uint8_t status);

void btsmCheckSecurity(LPTSECLINK link);
BOOL btsmCheckForMitm(LPTSECLINK link);
BOOL btsmCheckForJustWorks(LPTSECLINK link);
void btsmHandleHciAuthReq(void);
void btsmHandleAuthenticationInd(void);
/** towards protocols */
void btsmSendSECMAN_AUTHENTICATION_RESP(uint8_t Source, uint8_t * bd, uint16_t ref, uint16_t channelId, uint8_t outgoing, BOOL accept);
/** db entries */
BOOL                  btsmCheckNewSecEntry    (LPTSEC_CONF_SECURITY sec);
LPTSEC_CONF_SECURITY  btsmAllocateSecEntry    (LPTSEC_CONF_SECURITY newEntry);
uint16_t                  btsmDeAllocateSecEntry  (LPTSEC_CONF_SECURITY delEntry);
LPTSEC_CONF_SECURITY  btsmFindSecEntry        (uint8_t outgoing, uint16_t psm, uint16_t serverChannel, uint16_t uuid);
#endif


/** btsmutil.c */
void btsmDeferredQueueIn                  (LPTSECLINK pLink);
void btsmDeferredQueueOut                 (void);

/** link descriptor management */
LPTSECLINK            btsmFindLink            (TBdAddr bd);
LPTSECLINK btsmFindAllocateLink(uint8_t bdType, TBdAddr bd);
void  btsmHexString_r         (uint8_t * value, LPSTR output, uint16_t length);
LPSTR btsmHexString           (uint8_t * value, uint16_t length);
/** timer handling */
void   btsmStartTimer         (LPTSECLINK pLink, uint8_t timerID, uint16_t seconds);
void   btsmDeAllocateLink      (LPTSECLINK link);


#endif  /**< __BTSMPROT_H */

