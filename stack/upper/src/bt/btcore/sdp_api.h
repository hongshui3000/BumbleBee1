/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       sdp_api.h
* @brief     
* @details   
*
* @author  	gordon
* @date      	2015-07-13
* @version	v0.1
*/

#ifndef __SDP_API_H
#define __SDP_API_H

#include <flags.h>
#include <rtl_types.h>


#ifdef __cplusplus
extern "C" {
#endif

BOOL sdpInit(void);

void sdpHandleCONFIG_GAP_DEVICE_NAME(uint8_t * pName,uint16_t len);
void sdpHandleConfigDID(uint16_t VendorID, uint16_t ProductID, uint16_t ProductVersion, uint16_t IDSource);
void sdpHandleExtraEIRdata(char *pdata);
void sdpHandleUpConReq(LPCBYTE bd, uint16_t mtuSize);
void sdpHandleUpDiscReq(uint16_t cid, uint16_t holdLink);
uint16_t sdpHandleUpSearchReq(uint16_t linkid, uint16_t maxHandles, uint16_t uuidLen, LPCBYTE uuidList);
uint16_t sdpHandleUpAttributeReq(uint16_t linkid, uint32_t handle, uint16_t byteCnt, uint16_t aLen, LPCBYTE aList);

void sdpHandleL2cConConf(TBdAddr remote_bd, uint16_t cid, uint16_t status);
void sdpHandleL2cConActInd(uint16_t cid, uint16_t status, uint16_t mtuSize);
void sdpHandleL2cDiscInd(uint16_t localCID, uint16_t status);
void sdpHandleL2cDiscConf(uint16_t LocalCid, uint16_t result);
void sdpHandleL2cConInd(uint16_t localCID);

void sdpHandleTimer(uint8_t chan);

BOOL sdpHandleUpRegReq(void *buf, uint16_t offset);
void sdpHandleLDataInd(uint8_t * p, uint16_t chan, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif
