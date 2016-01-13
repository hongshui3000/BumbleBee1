/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       btsend.h
* @brief     BT control application
* @details   
*
* @author   	gordon
* @date      	2015-07-02
* @version	v0.1
*/

#ifndef __BTSEND_H
#define __BTSEND_H

#ifdef __cplusplus
extern "C" {
#endif

#include <btman.h>
#include <sdp.h>
/** Message Generators Upstream (CAPI)*/
void blueFaceSendBT_CON_CONF(BLINKCONTEXT bLinkContext, BLINKHANDLE bLinkHandle, uint16_t cause);
void blueFaceSendBT_CON_IND(LPbtLink link);
void blueFaceSendBT_DISC_IND(BLINKCONTEXT bLinkContext, uint16_t cause);

void blueFaceSendBT_SDP_SEARCH_CONF(BLINKCONTEXT bLinkContext, uint32_t* handles, uint16_t count, uint16_t offset);
void blueFaceSendBT_SDP_ATTRIBUTE_CONF(BLINKCONTEXT bLinkContext, uint8_t * avList, uint16_t length, uint16_t offset, TsdpChanDesc *tc);

void blueFaceSendBT_HCI_NAME_CONF(LPCBYTE bd, uint8_t *name, uint16_t cause);
void blueFaceSendBT_HCI_AUTH_CONF(uint16_t cause);
void blueFaceSendBT_HCI_KEY_IND(uint16_t keyRequestType, LPCBYTE bd);
void blueFaceSendBT_HCI_NEWKEY_IND(BLINKCONTEXT bLinkContext, LPCBYTE bd, LPCBYTE linkKey, uint8_t keyType);

void blueFaceSendBT_ACT_IND(TCBdAddr bd, uint16_t status);

void blueFaceSendBT_CON_ACT_IND(BLINKCONTEXT bLinkContext, uint16_t frameSize, uint16_t poolId, uint16_t * param);

LPbtLink blueFaceFindAndCheckLink(PBTA pBTA, uint16_t cid, const char *pTxt);
LP_L2C_Conf_para commonPrepareL2cConfPara(uint16_t mtuSize, uint16_t maxPDUSize, uint16_t reqId);
void blueFaceSendBT_CONF_IND_Mode_Change(LPHCI_MODECHANGE_IND	pModeChangeInd);
void blueFaceHandleConInd(LPbtLink link, LPCbtApplication app);


#ifdef __cplusplus
}
#endif

#endif
