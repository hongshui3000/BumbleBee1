/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       gatt_api.h
* @brief     
* @details   
*
* @author  	gordon
* @date      	2015-07-13
* @version	v0.1
*/

#ifndef __GATT_API_H
#define __GATT_API_H

#include <flags.h>
#include <rtl_types.h>
#include <bt_msg.h>
#include <gattdef.h>


#ifdef __cplusplus
extern "C" {
#endif
//gatt.c
BOOL gattInit(void);

//gattmsg.c
//for blueapi
void gattHandleGATT_SERVICE_REGISTER_REQ(uint16_t nbrOfAttrib, uint8_t * pService);
void gattHandleGATT_ATTRIB_UPDATE_REQ(uint8_t * pBuffer,PVOID reqHandle, PVOID serviceHandle,
                                      uint16_t attribIndex, uint16_t length, uint16_t offset);
void gattHandleGATT_ATTRIB_READ_RESP( uint16_t cid , uint16_t cause, uint16_t length, 
                                      PGATTAttribReadRespParam pParm);
void gattHandleGATT_ATTRIB_WRITE_RESP(uint16_t cid, uint16_t cause, uint16_t length,
                                              uint16_t        offset,       
                                              uint8_t *      pBuffer);
void gattHandleGATT_DISCOVERY_REQ(PGATTDiscoveryReq  pDiscoveryReq);
void gattHandleGATT_DISCOVERY_RESP(uint16_t cid, uint16_t type, uint16_t startingHandle, uint16_t endingHandle);
void gattHandleGATT_ATTRIB_READ_REQ( PGATTAttribReadReq  pAttribReadReq);
void gattHandleGATT_ATTRIB_WRITE_REQ(uint16_t cid, uint16_t type, uint16_t handle, uint16_t length,
                                     PGATTAttribWriteReqParam  pParam);
void gattHandleGATT_ATTRIB_INDICATION_RESP(uint16_t cid);
void gattHandleGATT_LE_ADVERTISE_REQ(uint16_t opCode, uint8_t length, uint8_t * pData);
void gattHandleGATT_LE_SCAN_REQ(PGATTLEScanReq pScanReq);
void gattHandleGATT_LE_MODIFY_WHITELIST_REQ(uint16_t opCode, uint8_t * bd, uint8_t bdType);
void gattHandleGATT_LE_CONNECTION_UPDATE_REQ(uint16_t cid, PGATTLEConnectionUpdateParam pParam);
void gattHandleGATT_LE_CONNECTION_UPDATE_RESP(uint16_t cid, uint16_t cause);
void gattHandleCONFIG_GAP_DEVICE_NAME(uint8_t * pName,uint16_t len);
void gattHandleCONFIG_GAP_APPEARANCE(uint16_t appearnce);
void gattHandleCONFIG_GAP_PER_PREF_CONN_PARAM(uint16_t connIntervalMin,
                                              uint16_t connIntervalMax,
                                              uint16_t slaveLatency,
                                              uint16_t supervisionTimeout);
void gattHandleDEVICE_DATA_RESP(uint16_t cmd,  PDeviceDataResp	pDeviceDataResp);

void gattHandleGATT_EXECUTE_WRITE_RESP(uint16_t cid, uint16_t cause, uint16_t handle);
void gattHandleGATT_EXECUTE_WRITE_REQ( uint16_t cid, uint8_t flags);

//for btsm
void gattHandleBtsmSecurityStatus(uint8_t indId, uint8_t * bd, uint8_t bdType, uint16_t cause, uint8_t keyType, uint8_t keySize);
void gattHandleBtsmResolvedAddress(uint8_t * bd, uint8_t bdType, uint8_t * resolvedBd, uint8_t resolvedBdType);


//gatt_le.c
//handle hci event
void gattHandleHCI_LE_CONNECTION_COMPLETE_EVENT( uint16_t Handle, uint16_t Status, uint8_t peerAddressType,
			                                 uint8_t * peerAddress, uint16_t connInterval,
			                                 uint16_t connLatency, uint16_t supervisionTimeout);
void gattHandleLE_HCI_DISCONNECTION_COMPLETE( uint16_t Handle, uint16_t Reason);
void gattHandleHCI_LE_ADVERTISING_REPORT_EVENT( uint8_t eventType, uint8_t addressType,
                                            TBdAddr address, uint8_t rssi, uint8_t dataLength,
                                            uint8_t *    data);
void gattHandleHCI_LE_CONNECTION_UPDATE_COMPLETE_EVENT(uint16_t Handle, uint16_t status,
                                                   uint16_t connInterval, 
                                                   uint16_t connLatency, uint16_t supervisionTimeout);
void gattHandleCmdCompLE_SET_ADVERTISING_PARAMETERS(uint16_t status);
void gattHandleCmdCompLE_SET_ADV_DATA(uint16_t status);
void gattHandleCmdCompLE_SET_ADVERTISE_ENABLE( uint16_t status);
void gattHandleCmdCompLE_SET_SCAN_PARAMETERS(uint16_t status);
void gattHandleCmdCompLE_SET_SCAN_ENABLE(uint16_t status);
void gattHandleCmdCompLE_MODIFY_WHITE_LIST(uint16_t status);
//
void gattHandleLECreateConnectionConf(uint16_t handle, uint16_t status);
void gattHandleLEConnectionUpdateConf( uint16_t handle, uint16_t status);
//for l2cap
void gattHandleL2CAP_CONNECTION_PARAMETER_UPDATE_REQUEST(uint16_t handle,PGATTLEConnectionUpdateParam pParam);

//gatt_ll.c
void gattHandleUpConnectReq(uint8_t bdType, uint8_t * bd, PCON_REQ_GATT pConPara);
void gattHandleUpConnectResp(uint16_t cid, uint16_t status);
void gattHandleUpDisconnectReq(uint16_t cid, uint16_t cause);
void gattHandleDisconnected( uint16_t cid, uint16_t result, BOOL DiscConf);
//for l2cap
void gattHandleAttData(uint16_t handle, MESSAGE_P pMsg, BOOL isL2c);

//gatt_br.c
#if F_BT_BREDR_SUPPORT
void gattHandleL2cConnectConf(uint16_t reqId, uint16_t cid, uint16_t status);
void gattHandleL2cConnectInd(uint16_t LocalCid, uint16_t psm, uint8_t * remote_bd);
void gattHandleL2cConnectActInd(uint16_t cid, uint16_t status, uint16_t mtuSize);
void gattHandleL2cDisconnectInd( uint16_t LocalCid, uint16_t status);
//for sdp
void gattHandleSDP_CONFIGURE_CONF(uint16_t status, uint32_t reference, uint32_t context);
#endif
//att.c
void attTimerExpired(uint8_t TimerID, BOOL ServerTimer );
//gattutil.c
#if F_BT_LE_BT41_SUPPORT
uint16_t gattGetLEConnectionHandle(uint16_t cid);
uint16_t gattGetLEConnectionCid(uint16_t handle);
#endif

#ifdef __cplusplus
}
#endif

#endif
