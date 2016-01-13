/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       hci_api.h
* @brief     
* @details   
*
* @author  	gordon
* @date      	2015-07-13
* @version	v0.1
*/

#ifndef __HCI_API_H
#define __HCI_API_H

#include <flags.h>
#include <rtl_types.h>
#include <bttypes.h>
#include <hci.h>
#include <blueface.h>

#ifdef __cplusplus
extern "C" {
#endif
/*hci.c*/
BOOL hciInit(void);

void hciActivateStack(void);
void hciDeactReq(void);
void hciLaterEntry(void);

void hciCommandDisconnect( uint16_t handle, uint8_t cause);
void hciCommandNoParameter(uint16_t command);
void hciCommandByteParameter(uint16_t command, uint8_t parameter);
void hciCommandWordParameter(uint16_t command, uint16_t parameter);
void hciCommandWordByteParameter(uint16_t command, uint16_t param1, uint8_t param2);
void hciCommandWordWordParameter(uint16_t command, uint16_t param1, uint16_t param2);
void hciCommandBDAddrParameter(uint16_t command, uint8_t * bd);
void hciCommandBDAddrByteParameter(uint16_t command, uint8_t * bd, uint8_t parameter);

/*hci_br.c*/
void hciCommandWriteExtendedInquiryResponse(uint8_t fec, uint8_t * extendedResult);
void hciCommandUserPasskeyRequestReply(uint8_t * bd, uint32_t value);
void hciCommandRemoteOOBDataRequestReply(uint8_t * bd, uint8_t * C, uint8_t * R);
void hciCommandLinkKeyRequestReply(uint8_t * bd, uint8_t * key);
void hciCommandPinCodeRequestReply(uint8_t * bd, uint8_t * key, uint8_t length);
void hciCommandSniffSubrating(uint16_t handle, uint16_t maxLatency, uint16_t minRemoteTimeout, uint16_t minLocalTimeout);
void hciCommandSniffMode(uint16_t handle, uint16_t max, uint16_t min, uint16_t attempt, uint16_t timeout);
void hciCommandIoCapabilityRequestReply(uint8_t * bd, uint8_t capability, uint8_t oob_data_present, uint8_t auth_requirements);
void hciCommandInquiry(uint8_t idxIAC, uint8_t timeout, uint8_t maxresult);
void hciCommandChangeLocalName(uint8_t * localName, uint16_t len);
void hciCommandRemoteNameRequest(uint8_t * bd, uint8_t pageScanRepMode, uint8_t pageScanMode, uint16_t clockOffset);
void hciCommandWriteCurrentIACs(uint8_t numCurrentIAC);

void hciHandleUpAuthReq(TBdAddr bd, uint8_t id);
void hciHandleUpEncryptReq(TBdAddr bd, uint8_t enable);
void hciHandleUpL2cConnectReq(TBdAddr remote_bd);
void hciHandleUpWriteExtendedInquiryResponse(LPHCI_CONF_EXTENDED_INQUIRY_RESPONSE eirResponse);
void hciHandleUpNameReq(uint8_t * Bd);
bool hciHandleUpSniffReq(LPCBYTE bd, uint8_t bd_type, uint16_t maxSniff, uint16_t minSniff, uint16_t attempt, uint16_t timeout);
void hciHandleUpInquiryReq(uint8_t timeOut, uint16_t maxHdl, uint8_t idxIAC);
void hciHandleUpWriteScanEnable(uint8_t enable, uint8_t parAction);
void hciHandleUpWriteClassOfDevice(uint8_t * ptr);

void hciHandleConfLinkPolicy(uint8_t * remote_BD,
									TBlueAPI_BRDeviceRole role,
									TBlueAPI_BRLinkPolicy link_policy,
									uint16_t link_supervision_timeout);
void hciHandleConfLinkPolicyDefault(TBlueAPI_BRDeviceRole role,
											TBlueAPI_BRLinkPolicy link_policy,
											uint16_t link_supervision_timeout);
void hciHandleConfPageCharacteristic(TBlueAPI_BRPageScanRepMode  scanRepetitionMode,
												uint16_t pageTimeout);
void hciHandleConfPageScanActivityEx(TBlueAPI_BRPageScanRepMode  scanRepetitionMode,
												TBlueAPI_BRPageScanType scanType,
												uint16_t scanInterval, uint16_t scanWindow);
void hciHandleConfSniffSubrating(uint8_t * remote_BD, uint16_t maxLatency,
                                          uint16_t minRemoteTimeout, uint16_t minLocalTimeout);
void hciHandleConfInquiryScanActivityEx(TBlueAPI_BRInquiryScanType scanType,
												uint16_t scanInterval, uint16_t scanWindow);
void hciHandleConfInquiryMode(uint8_t mode);

/*hci_le.c*/
void hciCommandLEStartEncryption(uint16_t handle, uint8_t * randomNumber, uint16_t encryptedDiversifier, uint8_t * longTermKey);
void hciCommandLELongTermKeyRequestReply(uint16_t handle, uint8_t * longTermKey);
void hciCommandLEEncrypt(uint8_t * key, uint8_t * plaintextData);
void hciCommandLESetAdvertisingParameters(uint16_t advertisingIntervalMin, uint16_t advertisingIntervalMax,
	                                      uint8_t advertisingType, uint8_t ownAddressType, uint8_t directAddressType,
	                                      uint8_t * directAddress, uint8_t advertisingChannelMap,
	                                      uint8_t advertisingFilterPolicy);
void hciCommandLESetScanEnable(uint8_t scanEnable, uint8_t filterDuplicates);
void hciCommandLESetScanParameters(uint8_t scanType, uint16_t scanInterval, uint16_t scanWindow, uint8_t ownAddressType, uint8_t scanningFilterPolicy);
void hciCommandLECreateConnection(uint16_t scanInterval, uint16_t scanWindow,
	                              uint8_t initiatorFilterPolicy, uint8_t peerAddressType, TBdAddr peerAddress,
	                              uint8_t ownAddressType, uint16_t connIntervalMin, uint16_t connIntervalMax,
	                              uint16_t connLatency, uint16_t supervisionTimeout, 
	                              uint16_t minimumCELength, uint16_t maximumCELength);
void hciCommandLEConnectionUpdate(uint16_t handle, uint16_t connIntervalMin, uint16_t connIntervalMax,
	                              uint16_t connLatency, uint16_t supervisionTimeout,
	                              uint16_t minimumCELength, uint16_t maximumCELength);
void hciCommandLESetRandomAddressCommand(uint8_t * randomAddress);

void hciCommandLEChangeDeviceWhiteList(uint16_t cmd, uint8_t * pBD, uint8_t addressType);
void hciCommandLESetAdvertisingData(uint16_t cmd, uint8_t * pData, uint16_t length);

void hciHandleUpSCOConConf(TBdAddr bd, uint16_t accept, PCON_RESP_SCO pSCO_RESP);
void hciHandleUpSCOConReq(uint8_t * bd, PCON_REQ_SCO pSCOConReq);

#ifdef __cplusplus
}
#endif

#endif
