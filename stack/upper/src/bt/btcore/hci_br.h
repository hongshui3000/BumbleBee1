/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       hci_le.h
* @brief     HCI Protocol Layer (Low Energy)
* @details   
*
* @author  	gordon
* @date      	2015-07-13
* @version	v0.1
*/

#ifndef __HCI_BR_H
#define __HCI_BR_H

#include <flags.h>
#include <rtl_types.h>


#ifdef __cplusplus
extern "C" {
#endif

/** Function declarations */
void hciChangeLinkPolicyAndRole(ThciHandleDesc *pHd);
void hciChangeConnPacketType(uint16_t handle);
void hciEncryptInd(uint8_t * bd, uint16_t status, uint8_t enable);
BOOL hciAuthenticationOnLinkEncrypted(ThciHandleDesc *hd, THciEncryptionAuthenticationEvent eAEvent);
void hciListenCnf(uint16_t status);
void hciCommandHostBufferSize(uint16_t aclLen, uint8_t scoLen, uint16_t aclCnt, uint16_t scoCnt);
void hciCommandHostNumberOfCompletedPackets(uint16_t handle, uint16_t count);



#ifdef __cplusplus
}
#endif

#endif
