/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       btsm_api.h
* @brief     
* @details   
*
* @author  	gordon
* @date      	2015-07-13
* @version	v0.1
*/

#ifndef __BTSM_API_H
#define __BTSM_API_H

#include <flags.h>
#include <rtl_types.h>
#include <btsm.h>


#ifdef __cplusplus
extern "C" {
#endif
//btsm.c
int  btsmInit(void);

//btsm_smp.c
void btsmHandleSMPMessage                 (LPLEMessage pLEMsg, uint16_t length);

//btsmmsg.c
//for blueapi
uint16_t btsmHandleConfigurationReq(uint16_t reqID,uint8_t * ptr, BOOL isSetBluetoothMode);
void btsmHandleRemoteOOBDataReqConf(uint8_t * bd, uint8_t * C, uint8_t * R, uint8_t status);
void btsmHandleUserPasskeyReqReplyReq(uint8_t * bd, uint8_t result, uint32_t value);
void btsmHandleUserPasskeyReqConf(uint8_t * bd, uint8_t result, uint32_t value);
void btsmSendMsgAuthenticationReq(uint8_t * bd, uint16_t requirements, uint8_t minKeySize);
//for hci
void btsmSendMsgHciConfigureInd(uint16_t code, uint16_t len, uint8_t * ptr);
void btsmUpdateLocalAddress(uint8_t * bd);
//for hw timer
void btsmHandleTimerExpired( uint8_t TimerID, uint16_t TimerChannel);
//for l2cap rfcomm
void btsmSendMsgAuthenticationInd( TBdAddr bd,
								   uint16_t    ref,
								   uint16_t    channelId, /* L2CAP: PSM, RFCOMM: DLCI */
								   uint16_t    uuid,
								   uint8_t    outgoing,  /* ignore if deactivate */
								   uint8_t    active,
								   uint8_t    Source,
								   uint8_t    conType);

//btsm_br.c
void btsmSendLHciUserConfReqResp(uint8_t * bd, uint8_t result);
//for blueapi
void btsmHandleKeypressNotificationReq(uint8_t * bd, uint8_t type);
uint16_t btsmHandleBT_HCI_KEY_RESP(LPCBYTE bd, uint8_t reqType, uint16_t cause, LPCBYTE key, uint16_t keyLen);
void btsmSendMsgHciAuthReq(LPCBYTE bd, uint8_t id);
//for hci
void btsmHandleHCI_IO_CAPABILITY_REQUEST(uint8_t * bd);
void btsmHandleHCI_IO_CAPABILITY_RESPONSE(uint8_t * bd, uint8_t ioCapability, uint8_t authRequirements );
void btsmHandleHCI_USER_CONFIRMATION_REQUEST(uint8_t * bd, uint32_t value);
void btsmHandleHCI_SIMPLE_PAIRING_COMPLETE(uint8_t * bd, uint16_t status);
void btsmHandleHCI_LINK_KEY_NOTIFICATION(uint8_t * bd, uint8_t * key, uint8_t keyType);
void btsmHandleCmdCompHCI_LINK_KEY_REQUEST_REPLY(uint16_t command, uint8_t * bd, uint16_t status);

void btsmHandleHciAuthConf(uint8_t * bd, uint8_t id, uint16_t status);
void btsmHandleHciEncryptInd(uint8_t * bd, uint16_t status, uint8_t enable);
void btsmHandleHciKeyInd(uint8_t * bd, uint8_t reqType);
void btsmHandleHciNewHandle(uint8_t bdType, TBdAddr bd);
void btsmHandleHciAclConnection(TBdAddr bd, uint16_t indId);


//btsm_le.c
//for hci
void btsmHandleHCI_LE_ENCRYPT(uint8_t * data);
void btsmHandleHCI_ENCRYPTION_KEY_REFRESH_COMPLETE(uint16_t handle, uint8_t status);
void btsmHandleLE_HCI_DISCONNECTION_COMPLETE(uint16_t handle);
void btsmHandleHci_LE_LONG_TERM_KEY_REQUEST_EVENT(uint16_t ediv, uint8_t * rand, uint16_t handle);
void btsmHandleHci_LE_ENCRYPTION_CHANGE(uint16_t handle, uint8_t encrypted, uint16_t status);
void btsmHandleCmdCompLE_RAND(uint8_t * result);
void btsmHandleCmdCompHCI_LE_SET_RANDOM_ADDRESS(uint16_t status);
void btsmSendLEMsg(uint16_t Command, LPLEMessage pLEMsg, uint16_t Length);
//for blueapi
void btsmHandleDEVICE_DATA_GET_RESP(PDeviceDataResp pDevDataResp);
void btsmHandleDEVICE_DATA_SET_RESP(PDeviceDataResp pDevDataResp);
void btsmHandleLE_SET_RANDOM_ADDRESS(uint8_t * randomAddress);
void btsmUpdateLocalBdType(uint8_t localBdType);


//btsmutil.c
uint8_t  btsmResolveRandomAddress                (TBdAddr bd, uint8_t * bdType);
uint8_t  btsmMapRandomAddress                    (TBdAddr bd, uint8_t * bdType);

#ifdef __cplusplus
}
#endif

#endif
