/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       blueapi_api.h
* @brief     
* @details   
*
* @author  	gordon
* @date      	2015-07-13
* @version	v0.1
*/

#ifndef __BLUEAPI_API_H
#define __BLUEAPI_API_H

#include <flags.h>
#include <rtl_types.h>
#include <btsm.h>


#ifdef __cplusplus
extern "C" {
#endif
uint16_t blueAPI_Check_LE_Link(uint16_t cid);
BOOL blueAPI_Send_GATTAttributeUpdateRsp(PVOID   pBuffer,
                                         uint16_t     offset,
                                         PVOID   serviceHandle,
	                                     PVOID   requestHandle,
	                                     uint16_t     attribIndex,
	                                     uint16_t     subCause,
	                                     uint16_t count
                                        );
void blueAPI_Send_GATTAttributeUpdateStatusInd(uint16_t            wAttribIndex,
	                                           PVOID          pServiceHandle,
	                                           PVOID          pReqHandle, 
	                                           uint8_t            bdType,
	                                           uint8_t *          RemoteBd,
                                               uint16_t subCause
                                               );
void blueAPI_Send_GATTAttributeReadInd(PVOID          serviceHandle,
	                                   uint16_t            attribIndex,
	                                   uint16_t            readOffset,
                                       uint16_t            local_MDL_ID
                                      );
BOOL blueAPI_Send_GATTAttributeWriteInd(PVOID   pBuffer,
                                        uint16_t     offset,
                                        uint16_t     type, 
                                        PVOID   serviceHandle,
	                                    uint16_t     attribIndex,
	                                    uint16_t     attribLength,
	                                    uint16_t     handle,  
	                                    uint16_t     writeOffset, 
                                        uint16_t     local_MDL_ID
                                       );
BOOL blueAPI_Send_GATTCCCDInfo(PVOID   pBuffer,
                               uint16_t     offset,
                               PVOID   serviceHandle,
	                           uint16_t     count,
                               uint16_t     local_MDL_ID
                              );
void blueAPI_Send_GATTMtuSizeInfo(uint16_t local_MDL_ID, uint16_t mtuSize);
BOOL blueAPI_Send_GATTDiscoveryInd(PVOID          pBuffer,
                                   uint16_t            offset,
                                   uint16_t            type,
	                               uint16_t            elementCount,
	                               uint16_t            elementLength,
	                               uint16_t            subCause,
                                   uint16_t            local_MDL_ID
                                  );
BOOL blueAPI_Send_GATTAttributeReadRsp(PBlueAPI_DsMessage pReqMsg,
                                       PVOID          pBuffer,
                                       uint16_t            offset,
                                       TBlueAPI_GATTReadType   readType,
	                                   uint16_t            readOffset,
	                                   uint16_t            totalLength,
	                                   uint16_t            attribLength,
	                                   uint16_t            nbrOfHandles,
                                       uint16_t            subCause,
                                       uint16_t            local_MDL_ID
                                      );
BOOL blueAPI_Send_GATTAttributeWriteRsp(PBlueAPI_DsMessage pReqMsg,
                                        uint16_t             local_MDL_ID,
                                        TBlueAPI_GATTWriteType   writeType,
                                        uint16_t             subCause,
                                        uint16_t             length
                                       );

BOOL blueAPI_Send_GATTAttributeNotifInd(PVOID          pBuffer,
                                        uint16_t            offset,
                                        BOOL            Notify,
                                        uint16_t            attribHandle,
	                                    uint16_t            attribLength,
                                        uint16_t            local_MDL_ID
                                       );
void blueAPI_Send_LEScanRsp(uint16_t subCause);
void blueAPI_Send_LEConnectionUpdateRsp(uint16_t local_MDL_ID, uint16_t subCause);
void blueAPI_Send_LEScanInfo(uint8_t eventType, uint8_t addressType,
                             uint8_t * address, uint8_t rssi, uint8_t dataLength,
                             uint8_t * data);
void blueAPI_Send_GATTDiscoveryRsp      (PBlueAPI_DsMessage pReqMsg, uint16_t local_MDL_ID, TBlueAPI_GATTDiscoveryType discoveryType, uint16_t subCause);
void blueAPI_Send_SetRandomAddressRsp(uint16_t subCause);
void blueAPI_Handle_GATT_SERVICE_REGISTER_CONF(/*uint16_t wApplHandle,*/ uint16_t wCause, 
                                               PVOID           serviceHandle);
void blueAPI_Handle_BT_GATT_LE_CONNECTION_UPDATE_IND(uint16_t cid, uint16_t type, PGATTLEConnectionUpdateParam pParam);

void blueAPI_Handle_GATT_CON_CONF( uint16_t status, uint16_t cid);
void blueAPI_Handle_SECURITY_STATUS(uint8_t indId, uint8_t * bd, uint8_t bdType, uint8_t keyType, uint8_t keySize);
void blueAPI_Handle_BTSM_RESOLVED_ADDRESS(uint8_t * bd, uint8_t bdType, uint8_t * resolvedBd, uint8_t resolvedBdType);
void blueAPI_Handle_BT_SECMAN_AUTH_CONF(uint8_t * bd,/* uint16_t ref,*/ uint8_t keyType, uint8_t keySize, uint16_t cause);
void blueAPI_Handle_NewKeyInd(uint8_t * bd, uint8_t * key, uint8_t keyType);
void blueAPI_Handle_BLUEFACE_CONF_AUTHORIZE(LPTSECLINK link);
void blueAPI_Handle_AuthConf(uint8_t id, uint16_t status);
void blueAPI_Handle_HciKeyInd(uint8_t * bd, uint8_t reqType);
void blueAPI_Handle_HCI_LINK_KEY_NOTIFICATION(uint8_t * bd, uint8_t * key, uint8_t keyType);

void blueAPI_Send_UserConfirmationReqInd(uint8_t * remote_BD, uint32_t displayValue);
void blueAPI_Send_UserPasskeyReqInd(uint8_t * remote_BD);
void blueAPI_Send_LegacyRemoteOOBDataReqInd(uint8_t * remote_BD);
void blueAPI_Send_UserPasskeyNotificationInfo(uint8_t * remote_BD, uint32_t displayValue);
void blueAPI_Send_KeypressNotificationInfo(uint8_t * remote_BD, uint8_t eventType);
void blueAPI_Send_UserAuthorizationReqInd(uint8_t * remote_BD, uint8_t outgoing, uint16_t psm, uint16_t server_channel, uint16_t uuid);
void blueAPI_Send_RemoteOOBDataReqInd(uint8_t * remote_BD);

void blueAPI_Send_LEAdvertiseParameterSetRsp(TBlueAPI_Cause cause);
void blueAPI_Send_LEAdvertiseDataSetRsp     (TBlueAPI_LEDataType dataType, TBlueAPI_Cause cause);
void blueAPI_Send_LEAdvertiseRsp            (TBlueAPI_LEAdvMode advMode, TBlueAPI_Cause cause);
void blueAPI_Send_LEModifyWhitelistRsp      (TBlueAPI_LEWhitelistOp operation, TBlueAPI_Cause cause);


void blueAPI_Send_GATTExecuteWriteInd   (uint16_t local_MDL_ID, uint8_t flags);
BOOL blueAPI_Send_GATTAttributePrepareWriteRsp(PBlueAPI_DsMessage pReqMsg,
                                               PVOID   pBuffer,
                                               uint16_t     offset,
	                                           uint16_t     attribLength,
	                                           uint16_t     writeOffset,
	                                           uint16_t     subCause,  
                                               uint16_t     local_MDL_ID
                                              );
void blueAPI_Send_GATTExecuteWriteRsp   (PBlueAPI_DsMessage pReqMsg,
        uint16_t local_MDL_ID,
        uint16_t subCause);

void blueAPI_Send_UserPasskeyReqReplyRsp(TBlueAPI_Cause cause);
void blueAPI_Send_LocalOOBDataRsp       (TBlueAPI_Cause cause, uint8_t * pC, uint8_t * pR);
void blueAPI_Send_KeypressNotificationRsp(TBlueAPI_Cause cause);

void blueAPI_Send_ACLStatusInfo         (uint8_t * pBD, TBlueAPI_RemoteBDType remote_BD_type, TBlueAPI_ACLStatus status, PBlueAPI_ACLStatusParam pParam);
void blueAPI_Send_UserAuthRequestInd(uint8_t * remote_BD);
void blueAPI_Send_ACLConfigRsp          (PBlueAPI_DsMessage pReqMsg, uint8_t * remote_BD, TBlueAPI_RemoteBDType remote_BD_Type, TBlueAPI_ACLConfigOpcode opCode, TBlueAPI_Cause cause);

TBlueAPI_Cause blueAPI_GATTConvertCause(uint16_t wCause);

#if F_BT_LE_BT41_SUPPORT
void blueAPI_Send_CreateLEDataChannelRsp (uint16_t local_MDL_ID, uint16_t channel, uint16_t subCause);
void blueAPI_Send_CreateLEDataChannelInd (uint16_t local_MDL_ID, uint16_t channel);
void blueAPI_Send_DisconnectLEDataChannelRsp (uint16_t local_MDL_ID, uint16_t channel, uint16_t subCause);
void blueAPI_Send_DisconnectLEDataChannelInd ( uint16_t local_MDL_ID, uint16_t channel, uint16_t subCause);
void blueAPI_Send_SendLEFlowControlCreditRsp (uint16_t local_MDL_ID, uint16_t channel, uint16_t subCause);
void blueAPI_Send_LEDataRsp (uint16_t local_MDL_ID, uint16_t channel, uint16_t subCause);
BOOL blueAPI_Send_LEDataInd(PVOID          pBuffer,
                            uint16_t            offset,
                            uint16_t            channel,
                            uint16_t            length,
                            uint16_t            local_MDL_ID);
void blueAPI_Send_LEPsmSecuritySetRsp(TBlueAPI_Cause cause);
void blueAPI_Send_LEDataChannelParameterInfo (uint16_t local_MDL_ID, uint16_t channel, 
                                              uint16_t remote_mtu, uint16_t remote_mps,
                                              uint16_t remote_initial_credits,
                                              uint16_t tx_size);
void blueAPI_Send_LEDataChannelCreditsAlertInfo (uint16_t local_MDL_ID, uint16_t channel);
uint16_t blueAPI_GetMdlID(uint16_t handle);

#endif

void blueAPI_Handle_GATT_CON_ACT_IND( uint16_t cid, uint16_t wMTUSize, uint16_t * param);
void blueAPI_Handle_GATT_DISC_CONF(uint16_t status, uint16_t cid );
void blueAPI_Handle_GATT_DISC_IND(uint16_t status, uint16_t cid );

void blueAPI_Handle_SDP_CON_CONF(uint16_t status, uint16_t linkid);
void blueAPI_Handle_SDP_CON_ACT_IND(uint16_t frameSize, uint16_t linkid);
void blueAPI_Handle_SDP_DISC_IND(uint16_t status, uint16_t linkid);

void blueAPI_Handle_HCI_ACT_CONF(uint8_t * bd, uint16_t status);
void blueAPI_Handle_HCI_READ_LOCAL_OOB(uint8_t * fp, uint16_t pos, uint8_t status);
void blueAPI_Handle_HCI_LISTEN_CONF(uint16_t status);
void blueAPI_Handle_HCI_NAME_CONF(uint8_t * bd, uint16_t status, uint8_t * name);
void blueAPI_Handle_HCI_MODE_CHANGE_IND(LPHCI_MODECHANGE_IND pmc);
void blueAPI_Handle_HCI_INQUIRY_RESULT(THCI_CONF_INQUIRY_RESULT *pConfInd_param);

void blueAPI_Handle_HCI_ROLE_CHANGE(uint16_t indId, TBdAddr bd, uint8_t bdType);
void blueAPI_Handle_HCI_WRITE_SCAN_ENABLE(uint16_t cause);

void blueAPI_Send_SDPRegisterRsp(TBlueAPI_Cause cause);
void blueAPI_Send_L2cConInd(uint16_t cid, uint8_t usQueueID, uint8_t* remoteBd);
void blueAPI_Send_L2cConActInd(uint16_t cid, uint16_t localUsMTU, uint16_t remoteMTU, uint16_t dsPoolID, uint16_t usQueueID, uint8_t* remoteBd);
void blueAPI_Send_L2cDataInd(MESSAGE_T * pmsg, uint16_t usQueueID);
void blueAPI_Send_L2cDiscInd(uint16_t cid, uint16_t usQueueID, uint16_t cause);
void blueAPI_Send_L2cConRsp(uint16_t cid, uint16_t usQueueID, uint8_t* remoteBd, uint16_t status);
void blueAPI_Send_L2cDiscRsp(uint16_t usQueueID, uint16_t cid, uint16_t reslut);
void blueAPI_Send_RFCAuthenticationRsp(TBdAddr bd, uint16_t ref, uint16_t channelId, uint8_t outgoing, uint16_t cause);

void blueAPI_Handle_SCO_CON_IND(uint8_t * bd, uint8_t * devClass, uint8_t linktype);
void blueAPI_Handle_SCO_CON_CONF(uint8_t * bd, uint16_t status);
void blueAPI_Handle_SCO_CON_ACT_IND(uint8_t * bd, uint16_t SCOhandle, uint16_t status, uint8_t airMode);
void blueAPI_Handle_SCO_DISC(uint16_t handle, uint8_t status);

void blueAPI_Send_SCOConInd(LPCBYTE remote_BD);
void blueAPI_Send_SCOConRsp(LPCBYTE remote_BD, uint8_t status);
void blueAPI_Send_SCOConActInd(LPCBYTE remote_BD, uint8_t airMode);
void blueAPI_Send_SCODiscInd(LPCBYTE remote_BD);

void blueFaceSendBT_DEVICE_DATA_IND(MESSAGE_T * pmsg);
#ifdef __cplusplus
}
#endif

#endif
