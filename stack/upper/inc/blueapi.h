/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file        blueapi.h
* @brief      Bluetooth Health Profile API Functions
* @details   
*
* @author   	gordon
* @date      	2015-07-10
* @version	v0.1
*/

#ifndef __BLUEAPI_H
#define __BLUEAPI_H

#ifndef __BLUEAPI_TYPES_H
#include <blueapi_types.h>
#include <sdp_code.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif


TBlueAPI_Cause blueAPI_BufferGet(uint16_t dsPoolID, uint16_t dsTPDUSize,
    uint16_t dsDataOffset, void **ppBuffer);

TBlueAPI_Cause blueAPI_BufferRelease(void *pBuffer);

bool blueAPI_ConnectGATTMDLReq(uint8_t * remote_BD, TBlueAPI_RemoteBDType remote_BD_Type,
    TBlueAPI_LocalBDType local_BD_Type, uint16_t scanInterval, uint16_t scanWindow,
    uint16_t scanTimeout, uint16_t connIntervalMin, uint16_t connIntervalMax, uint16_t connLatency,
    uint16_t supervisionTimeout, uint16_t CE_Length);

bool blueAPI_CreateMDLConf(uint16_t local_MDL_ID, uint8_t maxTPDUusCredits,
    TBlueAPI_Cause cause);

bool blueAPI_DisconnectMDLReq(uint16_t local_MDL_ID, TBlueAPI_Cause cause);

bool blueAPI_DisconnectMDLConf(uint16_t local_MDL_ID);

bool blueAPI_GATTServiceRegisterReq(uint16_t nbrOfAttrib,void * pService);

bool blueAPI_GATTServiceReleaseReq(void * serviceHandle);

bool blueAPI_GATTAttributeUpdateReq(void * pBuffer, void * serviceHandle,
    void * requestHandle, uint16_t attribIndex, uint16_t attribLength, uint16_t offset);

bool blueAPI_GATTAttributeUpdateStatusConf(void * serviceHandle,
    void * requestHandle, uint16_t attribIndex);

bool blueAPI_GATTAttributeReadConf(void * pBuffer, uint16_t local_MDL_ID,
    void * serviceHandle, TBlueAPI_Cause cause, uint16_t subCause,
    uint16_t attribIndex, uint16_t attribLength, uint16_t offset);

bool blueAPI_GATTAttributeWriteConf(uint16_t local_MDL_ID, void * serviceHandle,
    TBlueAPI_Cause cause, uint16_t subCause, uint16_t attribIndex);

bool blueAPI_GATTAttributePrepareWriteConf(void * pBuffer, uint16_t local_MDL_ID,
    void * serviceHandle, TBlueAPI_Cause cause, uint16_t subCause,
    uint16_t attribIndex, uint16_t attribLength, uint16_t offset);

bool blueAPI_GATTAttributeExecuteWriteConf(uint16_t local_MDL_ID, TBlueAPI_Cause cause,
    uint16_t subCause, uint16_t handle);

bool blueAPI_GATTAttributePrepareWriteReq(void * pBuffer, uint16_t local_MDL_ID,
    uint16_t attribHandle, uint16_t attribLength, uint16_t writeOffset, uint16_t offset);

bool blueAPI_GATTAttributeExecuteWriteReq(uint16_t local_MDL_ID, uint8_t flags);

bool blueAPI_GATTDiscoveryReq(uint16_t local_MDL_ID, TBlueAPI_GATTDiscoveryType discoveryType,
    uint16_t startHandle, uint16_t endHandle, uint16_t UUID16, uint8_t * pUUID128);

bool blueAPI_GATTDiscoveryConf(uint16_t local_MDL_ID, TBlueAPI_GATTDiscoveryType discoveryType,
    uint16_t startHandle, uint16_t endHandle);

bool blueAPI_GATTAttributeReadReq(uint16_t local_MDL_ID, TBlueAPI_GATTReadType readType,
    uint16_t readOffset, uint16_t startHandle, uint16_t endHandle,
    uint16_t UUID16, uint8_t * pUUID128);

bool blueAPI_GATTAttributeWriteReq(void * pBuffer, uint16_t local_MDL_ID,
    TBlueAPI_GATTWriteType writeType, uint16_t attribHandle, uint16_t attribLength, uint16_t offset);

bool blueAPI_GATTAttributeConf(uint16_t local_MDL_ID);

bool blueAPI_DeviceConfigDeviceNameSetReq(uint8_t * deviceName);

bool blueAPI_DeviceConfigAppearanceSetReq(uint16_t appearance);

bool blueAPI_DeviceConfigPerPrefConnParamSetReq(uint16_t connIntervalMin,
    uint16_t connIntervalMax, uint16_t slaveLatency, uint16_t supervisionTimeout);

bool blueAPI_DeviceConfigSecuritySetReq(uint32_t leFixedDisplayValue);

bool blueAPI_DeviceConfigStoreSetReq(TBlueAPI_StoreBondModes storeBondMode, uint8_t storeBondSize);

bool blueAPI_GATTSDPDiscoveryReq(uint8_t * remote_BD,
    uint16_t remote_GATT_UUID, bool remote_DID_Discovery);

bool blueAPI_GATTSDPDiscoveryConf(uint32_t serviceHandle, TBlueAPI_Cause cause);

bool blueAPI_GATTSecurityReq(uint16_t local_MDL_ID, uint16_t requirements, uint8_t minKeySize);

bool blueAPI_GATTServerStoreConf(TBlueAPI_GATTStoreOpCode opCode, uint8_t * remote_BD,
    TBlueAPI_RemoteBDType remote_BD_Type, uint16_t restartHandle, uint8_t dataLength,
    uint8_t * data, TBlueAPI_Cause cause);

bool blueAPI_InquiryReq(bool limitedInquiry, bool cancelInquiry, uint8_t timeout);

bool blueAPI_DeviceConfigDeviceSetReq(uint32_t classOfDevice, uint8_t * deviceName);

bool blueAPI_DeviceConfigDIDSetReq(uint16_t vendorID, uint16_t vendorIDSource,
    uint16_t productID, uint16_t productVersion);

bool blueAPI_DeviceConfigExtraEIRSetReq(uint8_t * pdata);

bool blueAPI_DeviceConfigPagescanSetReq(TBlueAPI_BRPageScanType scanType,
    TBlueAPI_BRPageScanRepMode repMode, uint16_t repInterval,
    uint16_t repWindow, uint16_t pageTimeout);

bool blueAPI_DeviceConfigInquiryscanSetReq(TBlueAPI_BRInquiryScanType scanType,
    uint16_t interval, uint16_t window);

bool blueAPI_DeviceConfigInquiryModeReq(TBlueAPI_BRInquiryMode mode);

bool blueAPI_DeviceConfigLinkpolicySetReq(TBlueAPI_BRLinkPolicy linkPolicy,
    TBlueAPI_BRDeviceRole deviceRole, uint16_t supervisionTimeout);

bool blueAPI_ACLConfigLinkpolicyReq(uint8_t * remote_BD, TBlueAPI_BRLinkPolicy linkPolicy,
    TBlueAPI_BRDeviceRole deviceRole, uint16_t supervisionTimeout);

bool blueAPI_ACLConfigSniffmodeReq(uint8_t * remote_BD, uint16_t minInterval,
    uint16_t maxInterval, uint16_t sniffAttempt, uint16_t sniffTimeout, uint16_t maxLatency,
    uint16_t minRemoteTimeout, uint16_t minLocalTimeout);

bool blueAPI_DeviceNameReq(uint8_t * remote_BD);

bool blueAPI_DIDDeviceConf(uint32_t serviceHandle, TBlueAPI_Cause cause);

bool blueAPI_RadioModeSetReq(TBlueAPI_RadioMode localRadioMode, bool limitedDiscoverable);

bool blueAPI_SDPDiscoveryReq(uint8_t * remote_BD,
    TBlueAPI_MDEPDataType remote_MDEP_DataType, bool remote_DID_Discovery);

bool blueAPI_SDPEndpointConf(uint32_t serviceHandle, TBlueAPI_Cause cause);

bool blueAPI_RegisterReq(TBlueAPIAppHandle appHandle, void * MDHmsgHandlerCallback);

bool blueAPI_ReleaseReq(void);

bool blueAPI_UserAuthRequestConf(uint8_t * remote_BD, uint8_t AuthCodeLength,
    uint8_t * AuthCode, TBlueAPI_Cause cause);

bool blueAPI_AuthResultRequestConf(uint8_t * remote_BD, TBlueAPI_RemoteBDType remote_BD_Type,
    uint8_t linkKeyLength, uint8_t * linkKey, TBlueAPI_LinkKeyType keyType,
    uint16_t restartHandle, TBlueAPI_Cause cause);

bool blueAPI_PairableModeSetReq(bool enablePairableMode, TBlueAPI_AuthRequirements AuthRequirements,
    TBlueAPI_IOCapabilities IOCapabilities, bool remoteOOBDataPresent);

bool blueAPI_ExtendPairableModeSetReq(bool enablePairableMode,
    TBlueAPI_BluetoothMode BluetoothMode, TBlueAPI_AuthRequirements AuthRequirements,
    TBlueAPI_IOCapabilities IOCapabilities, bool remoteOOBDataPresent);

bool blueAPI_AuthReq(uint8_t * remote_BD);

bool blueAPI_UserConfirmationReqConf(uint8_t * remote_BD, TBlueAPI_Cause cause);

bool blueAPI_UserAuthorizationReqConf(uint8_t *remote_BD, TBlueAPI_Cause cause);

bool blueAPI_UserPasskeyReqConf(uint8_t * remote_BD, TBlueAPI_Cause cause);

bool blueAPI_UserPasskeyReqReplyReq(uint8_t * remote_BD, uint32_t passKey, TBlueAPI_Cause cause);

bool blueAPI_KeypressNotificationReq(uint8_t * remote_BD, TBlueAPI_SSPKeyEvent eventType);

bool blueAPI_LegacyRemoteOOBDataReqConf(uint8_t * remote_BD,
    uint8_t * pC, uint8_t * pR, TBlueAPI_Cause cause);

bool blueAPI_RemoteOOBDataReqConf(uint8_t * remote_BD, uint8_t * pC, TBlueAPI_Cause cause);

bool blueAPI_LocalOOBDataReq(void);

bool blueAPI_AuthResultConf(uint8_t * remote_BD, TBlueAPI_RemoteBDType remote_BD_Type, TBlueAPI_Cause cause);

bool blueAPI_AuthDeleteReq(uint8_t * remote_BD, TBlueAPI_RemoteBDType remote_BD_Type);

bool blueAPI_AuthListReq(uint8_t * remote_BD, TBlueAPI_RemoteBDType  remote_BD_Type);

bool blueAPI_LEAdvertiseReq(TBlueAPI_LEAdvMode advMode);

bool blueAPI_LEAdvertiseParameterSetReq(TBlueAPI_LEAdvType advType,
    TBlueAPI_LEFilterPolicy filterScanReq, TBlueAPI_LEFilterPolicy filterConnectReq,
    uint16_t minAdvInterval, uint16_t maxAdvInterval, TBlueAPI_LocalBDType local_BD_type,
    uint8_t * remote_BD, TBlueAPI_RemoteBDType remote_BD_type);

bool blueAPI_LEAdvertiseDataSetReq(TBlueAPI_LEDataType dataType, uint8_t dataLength, uint8_t * data);

bool blueAPI_LEScanReq(TBlueAPI_LEScanMode scanMode, uint16_t scanInterval, uint16_t scanWindow,
    TBlueAPI_LEFilterPolicy filterPolicy, TBlueAPI_LocalBDType local_BD_Type, uint8_t filterDuplicates);

bool blueAPI_LEModifyWhitelistReq(TBlueAPI_LEWhitelistOp operation, uint8_t * remote_BD,
    TBlueAPI_RemoteBDType remote_BD_type);

bool blueAPI_LEConnectionUpdateReq(uint16_t local_MDL_ID, uint16_t connIntervalMin,
    uint16_t connIntervalMax, uint16_t connLatency, uint16_t supervisionTimeout);

bool blueAPI_LEConnectionUpdateConf(uint16_t local_MDL_ID, TBlueAPI_Cause cause);

bool blueAPI_SetRandomAddressReq(uint8_t * random_BD);

bool blueAPI_DataRamPoolInit(uint8_t * pBufferDataOff, uint16_t size);

uint16_t blueAPI_DataRamPoolCreate(uint16_t poolElementSize, uint16_t poolElementCount);

bool blueAPI_DataRamPoolExtend(uint16_t poolID, uint16_t poolElementSize, uint16_t poolElementCount);

void * blueAPI_DataRamPoolBufferGet(uint16_t AppPoolID, uint16_t len);

void blueAPI_DataRamPoolBufferRelease(uint16_t AppPoolID, void * pBuffer);

bool blueAPI_SDPRegister(void* buf, uint8_t offset);

bool blueAPI_L2cProtocolRegister(uint16_t psm, uint16_t listenerQueue, uint8_t action);

bool blueAPI_L2cConConf(uint16_t status, uint16_t cid);

bool blueAPI_L2cDataReq(void* buf, uint8_t writeOffset, uint16_t cid, uint16_t length);

bool blueAPI_L2cDiscConf(uint16_t cid);

bool blueAPI_L2cConReq(uint16_t psm, uint16_t uuid, uint8_t* remoteBd, uint16_t usQueueID, uint16_t mtuSize);

bool blueAPI_L2cDiscReq(uint16_t cid);

bool blueAPI_L2cSecurityRegister(uint8_t active, uint8_t outgoing, uint8_t psm, uint16_t server_channel,
    uint16_t uuid, bool authentication, bool authorize, bool encryption, bool mitm);

bool blueAPI_RFCAuthenticationReq(uint8_t * bd, uint16_t channel, uint16_t dlci,
    uint16_t uuid, uint8_t outgoing, uint8_t active);

uint32_t blueAPI_SDPRecordLength(uint8_t *format, ...);

#define blueAPI_SDPCreateDes sdpCreateDes

bool blueAPI_SCOConReq(uint8_t *remote_BD, uint32_t txBandwidth, uint32_t rxBandwidth,
    uint16_t maxLatency, uint16_t voiceSetting, uint8_t  retransEffort, uint16_t packetType);

bool blueAPI_SCOConConf(uint8_t *remote_BD, uint32_t txBandwidth, uint32_t rxBandwidth,
    uint16_t maxLatency, uint16_t voiceSetting, uint8_t  retransEffort,
    uint16_t packetType, TBlueAPI_Cause cause);

bool blueAPI_SCODiscReq(uint8_t * remote_BD);

bool blueAPI_CreateLEDataChannelReq(uint16_t local_MDL_ID, uint16_t le_psm, uint16_t mtu,
    uint16_t mps, uint16_t initialCredits, uint16_t creditsIncrease);

bool blueAPI_CreateLEDataChannelConf(uint16_t local_MDL_ID, uint16_t channel, uint16_t mtu,
    uint16_t mps, uint16_t initialCredits,  uint16_t creditsIncrease, TBlueAPI_Cause cause);

bool blueAPI_DisconnectLEDataChannelReq(uint16_t local_MDL_ID, uint16_t channel);

bool blueAPI_DisconnectLEDataChannelConf(uint16_t local_MDL_ID, uint16_t channel, TBlueAPI_Cause cause);

bool blueAPI_SendLEFlowControlCreditReq(uint16_t local_MDL_ID, uint16_t channel, uint16_t credits);

bool blueAPI_LEDataReq(void * pBuffer, uint16_t local_MDL_ID, uint16_t channel,
    uint16_t valueLength, uint16_t offset);

bool blueAPI_LEDataConf(uint16_t local_MDL_ID, uint16_t channel, TBlueAPI_Cause cause);

bool blueAPI_LEPsmSecuritySetReq(uint16_t le_psm, bool active,
    TBlueAPI_LESecurityMode secMode, uint8_t keySize);

#ifdef __cplusplus
}
#endif
#endif  /**< __BLUEAPI_H */
