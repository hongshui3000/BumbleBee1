/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       l2c_api.h
* @brief     
* @details   
*
* @author  	gordon
* @date      	2015-07-13
* @version	v0.1
*/

#ifndef __L2C_API_H
#define __L2C_API_H

#include <flags.h>
#include <rtl_types.h>
#include <message.h>
#include <bttypes.h>
#include <os_message.h>

#ifdef __cplusplus
extern "C" {
#endif

#if F_BT_LE_BT41_SUPPORT
#define LE_CREDIT_CONNECTION_SUCCESS                      0x00
#define LE_CREDIT_ERR_LE_PSM_NOT_SUPPORTED                0x02
#define LE_CREDIT_ERR_NO_RESOURCES_AVAILABLE              0x04
#define LE_CREDIT_ERR_INSUFFICIENT_AUTHENTICATION         0x05
#define LE_CREDIT_ERR_INSUFFICIENT_AUTHORIZATION          0x06
#define LE_CREDIT_ERR_INSUFFICIENT_ENCRYPTION_KEY_SIZE    0x07
#define LE_CREDIT_ERR_INSUFFICIENT_ENCRYPTION             0x08
#define LE_CREDIT_ERR_INVAlID_SOURCE_CID                  0x09
#define LE_CREDIT_ERR_SOURCE_CID_ALREADY_ALLOCATED        0x0a
#define LE_CREDIT_ERR_LINK_NOT_EXIST                      0xef
#endif

/** fixed channel */
#define CID_SIGNALING_BR                 0x01
#define CID_CONNECTIONLESS               0x02
#define CID_ATTRIBUTE_PROTOCOL           0x04
#define CID_SIGNALING_LE                 0x05
#define CID_SECURITY_MANAGER             0x06

BOOL l2cInit(void);
void l2cHandleHciInitCompleted(uint16_t leDsAclSize, uint16_t status, uint16_t aclSize);
void l2cHandleTIMER_EXPIRED(uint8_t TimerID, uint16_t TimerChannel);

void l2cHandleSECMAN_AUTHENTICATION_RESP(TBdAddr bd, uint16_t ref, uint16_t channelId,
										 uint16_t uuid, uint8_t outgoing, uint16_t cause);
void l2cHandleBTG_DISC_REQ(uint16_t cid, BOOL holdLink);
void l2cHandleL2C_CON_RESP(uint16_t cid, uint16_t status, PBtConRespPSpecifc p);
void l2cHandleL2C_DISC_RESP(uint16_t lcid);
void l2cHandleL2C_DATA_REQ(MESSAGE_P  pmsg, BOOL prio);
void l2cHandleHciDiscInd(uint16_t handle, uint16_t cause, uint8_t conType);
void l2cHandleHciConCnf(uint8_t * bd, uint16_t status);
void l2cHandleHciConInd(uint8_t * bd, uint8_t * devClass, uint8_t linktype);
void l2cHandleHciConActInd(uint16_t status, uint16_t handle, uint8_t * bd);

uint16_t l2cUSendListenReq(uint16_t psm, uint8_t listenerQueue, uint8_t action);
void l2cUSendConReq(uint8_t RequesterQueue, uint8_t * remoteBd, uint16_t psm, uint16_t uuid, uint16_t frameSize,LP_L2C_Conf_para pConfPara);

uint16_t l2cLEHandleDataReq(MESSAGE_P pmsg, uint16_t channel, uint16_t handle);
uint16_t l2cLEHandleConnectionUpdateReq(uint16_t handle, PGATTLEConnectionUpdateParam pParam);
void l2cLEHandleConnectionUpdateResp( uint16_t handle, PGATTLEConnectionUpdateParam pParam, uint8_t status );
void l2cLEHandleHCI_DISCONNECTION_COMPLETE(uint16_t handle);
void l2cLEHandleHCI_LE_CONNECTION_COMPLETE_EVENT(uint16_t Handle, uint8_t role, uint16_t Status, uint8_t * peerAddress);

#if F_BT_LE_BT41_SUPPORT
uint16_t l2cLEHandle_CreateLEDataChannelReq(uint16_t handle, uint16_t le_psm, uint16_t mtu, uint16_t mps,
                                        uint16_t initialCredits, uint16_t creditsIncrease);
void l2cLEHandle_CreateLEDataChannelConf(uint16_t channel, uint16_t mtu, uint16_t mps, uint16_t initialCredits,
                                         uint16_t creditsIncrease, uint16_t cause);
uint16_t l2cLEHandle_DisconnectLEDataChannelReq(uint16_t channel);
uint16_t l2cLEHandle_SendLEFlowControlCreditReq(uint16_t channel, uint16_t credits);
uint16_t l2cLEHandle_LEDataReq(uint16_t channel, uint8_t * pBuffer, uint16_t offset, uint16_t valueLength);
void l2cLEHandle_LEDataConf(uint16_t channel, uint16_t cause);
#endif

#ifdef __cplusplus
}
#endif

#endif
