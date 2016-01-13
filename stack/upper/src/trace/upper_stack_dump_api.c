
#include <upper_stack_dump_api.h>
#include <rtl_types.h>
#include <blueface.h>
#include <blueapi_types.h>
#include <l2c.h>
#include <sdp.h>
#include <hci.h>
#include <gattdef.h>
#include <btsm.h>
#include <btman.h>
#include <blueapi_def.h>
#include <blueapi_osif.h>
#include <os_pool.h>
#include <upper_stack_global.h>
#include <efuse_config.h>
#include <swtimer.h>
#include <trace_binary.h>

#define TRACE_MODULE_ID     MID_BLUEFACE

extern uint8_t g_time_queue_element_count;
#if 0
void blueAPI_dump_BlueAPIDsMessageLength(void)
{
    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "sizeof(TBlueAPI_DsMessage) =%d",
        sizeof(TBlueAPI_DsMessage)
    );

    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_ConnectMDLReq =%d",
        sizeof(TBlueAPI_ConnectMDLReq)
    );

    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_CreateMDLConf = %d",
        sizeof(TBlueAPI_CreateMDLConf)
    );


    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_DisconnectMDLReq = %d",
        sizeof(TBlueAPI_DisconnectMDLReq)

    );
    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_RegisterReq =%d",
        sizeof(TBlueAPI_RegisterReq)
    );
    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_ReleaseReq = %d",
        sizeof(TBlueAPI_ReleaseReq)
    );

    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_DeviceConfigSetReq =%d",
        sizeof(TBlueAPI_DeviceConfigSetReq)
    );

    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_GATTServiceRegisterReq =%d",
        sizeof(TBlueAPI_GATTServiceRegisterReq)
    );

    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_GATTAttributeUpdateReq = %d",
        sizeof(TBlueAPI_GATTAttributeUpdateReq)
    );

    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_GATTAttributeUpdateStatusConf = %d",
        sizeof(TBlueAPI_GATTAttributeUpdateStatusConf)
    );

    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_GATTAttributeReadConf = %d",
        sizeof(TBlueAPI_GATTAttributeReadConf)
    );


    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_GATTAttributeWriteConf = %d",
        sizeof(TBlueAPI_GATTAttributeWriteConf)
    );
	BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_GATTAttributePrepareWriteConf = %d",
        sizeof(TBlueAPI_GATTAttributePrepareWriteConf)
    );

	BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_GATTAttributeExecuteWriteConf = %d",
        sizeof(TBlueAPI_GATTAttributeExecuteWriteConf)
    );

    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_GATTDiscoveryReq =%d",
        sizeof(TBlueAPI_GATTDiscoveryReq)
    );
    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_GATTDiscoveryConf = %d",
        sizeof(TBlueAPI_GATTDiscoveryConf)
    );

    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_GATTAttributeReadReq =%d",
        sizeof(TBlueAPI_GATTAttributeReadReq)
    );

	BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_GATTAttributeReadMultipleReq =%d",
        sizeof(TBlueAPI_GATTAttributeReadMultipleReq)
    );

    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_GATTAttributeWriteReq = %d",
        sizeof(TBlueAPI_GATTAttributeWriteReq)

    );

	BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_GATTAttributeExecuteWriteReq = %d",
        sizeof(TBlueAPI_GATTAttributeExecuteWriteReq)

    );

    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_GATTAttributeConf = %d",
        sizeof(TBlueAPI_GATTAttributeConf)
    );
    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_GATTSecurityReq = %d",
        sizeof(TBlueAPI_GATTSecurityReq)
    );

    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_GATTServerStoreConf = %d",
        sizeof(TBlueAPI_GATTServerStoreConf)

    );

    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_PairableModeSetReq =%d",
        sizeof(TBlueAPI_PairableModeSetReq)

    );

    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_UserPasskeyReqConf = %d",
        sizeof(TBlueAPI_UserPasskeyReqConf)

    );
    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_UserPasskeyReqReplyReq = %d",
        sizeof(TBlueAPI_UserPasskeyReqReplyReq)

    );


    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_RemoteOOBDataReqConf = %d",
        sizeof(TBlueAPI_RemoteOOBDataReqConf)
    );

    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_AuthResultConf = %d",
        sizeof(TBlueAPI_AuthResultConf)
    );

    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_AuthResultRequestConf = %d",
        sizeof(TBlueAPI_AuthResultRequestConf)
    );

    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_LEAdvertiseReq =%d",
        sizeof(TBlueAPI_LEAdvertiseReq)
    );

    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_LEAdvertiseParameterSetReq = %d",
        sizeof(TBlueAPI_LEAdvertiseParameterSetReq)
    );

    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_LEAdvertiseDataSetReq =%d",
        sizeof(TBlueAPI_LEAdvertiseDataSetReq)
    );

    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_LEScanReq = %d",
        sizeof(TBlueAPI_LEScanReq)

    );

    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_LEModifyWhitelistReq = %d",
        sizeof(TBlueAPI_LEModifyWhitelistReq)
    );

    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_LEConnectionUpdateReq = %d",
        sizeof(TBlueAPI_LEConnectionUpdateReq)
    );

    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_LEConnectionUpdateConf = %d",
        sizeof(TBlueAPI_LEConnectionUpdateConf)
    );

	BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_VendorSetVoiceParaReq = %d",
        sizeof(TBlueAPI_VendorSetVoiceParaReq)
    );
	BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_SetBleTxPowerReq = %d",
        sizeof(TBlueAPI_SetBleTxPowerReq)
    );
	BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_SetRandomAddressReq = %d",
        sizeof(TBlueAPI_SetRandomAddressReq)
    );

	BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_CreateLEDataChannelReq = %d",
        sizeof(TBlueAPI_CreateLEDataChannelReq)
    );
	
	BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_CreateLEDataChannelConf = %d",
        sizeof(TBlueAPI_CreateLEDataChannelConf)
    );
	BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_DisconnectLEDataChannelReq = %d",
        sizeof(TBlueAPI_DisconnectLEDataChannelReq)
    );
	BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_DisconnectLEDataChannelConf = %d",
        sizeof(TBlueAPI_DisconnectLEDataChannelConf)
    );
	BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_SendLEFlowControlCreditReq = %d",
        sizeof(TBlueAPI_SendLEFlowControlCreditReq)
    );
	BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_LEDataReq = %d",
        sizeof(TBlueAPI_LEDataReq)
    );
	BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_LEDataConf = %d",
        sizeof(TBlueAPI_LEDataConf)
    );
	BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_LEPsmSecuritySetReq = %d",
        sizeof(TBlueAPI_LEPsmSecuritySetReq)
    );


	
}

void blueAPI_dump_BlueAPIUsMessageLength(void)
{
    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "sizeof(TBlueAPI_UsMessage) =%d",
        sizeof(TBlueAPI_UsMessage)
    );

    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_ConnectMDLRsp = %d",
        sizeof(TBlueAPI_ConnectMDLRsp)
    );
    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_CreateMDLInd = %d",
        sizeof(TBlueAPI_CreateMDLInd)
    );

    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_DeleteMDLInfo =%d",
        sizeof(TBlueAPI_DeleteMDLInfo)
    );
    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_ConnectMDLInfo = %d",
        sizeof(TBlueAPI_ConnectMDLInfo)
    );
    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_DisconnectMDLRsp = %d",
        sizeof(TBlueAPI_DisconnectMDLRsp)
    );
	BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_DisconnectMDLInd = %d",
        sizeof(TBlueAPI_DisconnectMDLInd)
    );
    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_MCLStatusInfo = %d",
        sizeof(TBlueAPI_MCLStatusInfo)
    );
    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_RegisterRsp = %d",
        sizeof(TBlueAPI_RegisterRsp)
    );
    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_ReleaseRsp = %d",
        sizeof(TBlueAPI_ReleaseRsp)
    );

    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_ActInfo =%d",
        sizeof(TBlueAPI_ActInfo)
    );

    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_InternalEventInfo = %d",
        sizeof(TBlueAPI_InternalEventInfo)
    );

    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_DeviceConfigSetRsp = %d",
        sizeof(TBlueAPI_DeviceConfigSetRsp)
    );

    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_GATTServiceRegisterRsp = %d",
        sizeof(TBlueAPI_GATTServiceRegisterRsp)
    );
    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_GATTAttributeUpdateRsp = %d",
        sizeof(TBlueAPI_GATTAttributeUpdateRsp)
    );

    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_GATTAttributeUpdateStatusInd =%d",
        sizeof(TBlueAPI_GATTAttributeUpdateStatusInd)
    );
    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_GATTAttributeReadInd = %d",
        sizeof(TBlueAPI_GATTAttributeReadInd)
    );

    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_GATTAttributeWriteInd =%d",
        sizeof(TBlueAPI_GATTAttributeWriteInd)
    );
	BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_GATTAttributeExecuteWriteInd =%d",
        sizeof(TBlueAPI_GATTAttributeExecuteWriteInd)
    );
    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_GATTAttributeWriteCommandInfo = %d",
        sizeof(TBlueAPI_GATTAttributeWriteCommandInfo)
    );

    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_GATTCCCDInfo = %d",
        sizeof(TBlueAPI_GATTCCCDInfo)
    );

    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_GATTDiscoveryRsp = %d",
        sizeof(TBlueAPI_GATTDiscoveryRsp)
    );
    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_GATTDiscoveryInd = %d",
        sizeof(TBlueAPI_GATTDiscoveryInd)
    );

    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_GATTAttributeReadRsp = %d",
        sizeof(TBlueAPI_GATTAttributeReadRsp)
    );
	BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_GATTAttributeReadMultipleRsp = %d",
        sizeof(TBlueAPI_GATTAttributeReadMultipleRsp)
    );
    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_GATTAttributeWriteRsp = %d",
        sizeof(TBlueAPI_GATTAttributeWriteRsp)
    );
	BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_GATTAttributePrepareWriteRsp = %d",
        sizeof(TBlueAPI_GATTAttributePrepareWriteRsp)
    );
	BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_GATTAttributeExecuteWriteRsp = %d",
        sizeof(TBlueAPI_GATTAttributeExecuteWriteRsp)
    );

    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_GATTAttributeInd =%d",
        sizeof(TBlueAPI_GATTAttributeInd)
    );
    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_GATTAttributeNotificationInfo = %d",
        sizeof(TBlueAPI_GATTAttributeNotificationInfo)
    );

    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_GATTSecurityRsp =%d",
        sizeof(TBlueAPI_GATTSecurityRsp)
    );

    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_GATTServerStoreInd = %d",
        sizeof(TBlueAPI_GATTServerStoreInd)
    );
    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_GATTMtuSizeInfo = %d",
        sizeof(TBlueAPI_GATTMtuSizeInfo)
    );

    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_PairableModeSetRsp = %d",
        sizeof(TBlueAPI_PairableModeSetRsp)
    );

    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_UserPasskeyReqInd =%d",
        sizeof(TBlueAPI_UserPasskeyReqInd)
    );

    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_UserPasskeyReqReplyRsp = %d",
        sizeof(TBlueAPI_UserPasskeyReqReplyRsp)
    );

    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_UserPasskeyNotificationInfo =%d",
        sizeof(TBlueAPI_UserPasskeyNotificationInfo)
    );
    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_RemoteOOBDataReqInd = %d ",
        sizeof(TBlueAPI_RemoteOOBDataReqInd)
    );

    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_AuthResultInd = %d",
        sizeof(TBlueAPI_AuthResultInd)
    );
    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_AuthResultRequestInd = %d",
        sizeof(TBlueAPI_AuthResultRequestInd)
    );

    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_LEAdvertiseRsp = %d",
        sizeof(TBlueAPI_LEAdvertiseRsp)
    );
    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_LEAdvertiseParameterSetRsp = %d",
        sizeof(TBlueAPI_LEAdvertiseParameterSetRsp)
    );

    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_LEAdvertiseDataSetRsp = %d",
        sizeof(TBlueAPI_LEAdvertiseDataSetRsp)
    );
    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_LEScanRsp = %d",
        sizeof(TBlueAPI_LEScanRsp)
    );

    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_LEScanInfo =%d",
        sizeof(TBlueAPI_LEScanInfo)
    );
    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_LEModifyWhitelistRsp = %d",
        sizeof(TBlueAPI_LEModifyWhitelistRsp)
    );

    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_LEConnectionUpdateRsp =%d",
        sizeof(TBlueAPI_LEConnectionUpdateRsp)
    );
    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_LEConnectionUpdateInd = %d",
        sizeof(TBlueAPI_LEConnectionUpdateInd)
    );

    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_LEConnectionParameterInfo = %d",
        sizeof(TBlueAPI_LEConnectionParameterInfo)
    );
	
	BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_VendorSetVoiceParaRsp =%d",
        sizeof(TBlueAPI_VendorSetVoiceParaRsp)
    );
    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_SetBleTxPowerRsp = %d",
        sizeof(TBlueAPI_SetBleTxPowerRsp)
    );

    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_SetRandomAddressRsp = %d",
        sizeof(TBlueAPI_SetRandomAddressRsp)
    );

	BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_CreateLEDataChannelRsp =%d",
        sizeof(TBlueAPI_CreateLEDataChannelRsp)
    );
    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_CreateLEDataChannelInd = %d",
        sizeof(TBlueAPI_CreateLEDataChannelInd)
    );

    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_DisconnectLEDataChannelRsp = %d",
        sizeof(TBlueAPI_DisconnectLEDataChannelRsp)
    );
	BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_DisconnectLEDataChannelInd = %d",
        sizeof(TBlueAPI_DisconnectLEDataChannelInd)
    );

	BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_SendLEFlowControlCreditRsp =%d",
        sizeof(TBlueAPI_SendLEFlowControlCreditRsp)
    );
    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_LEDataRsp = %d",
        sizeof(TBlueAPI_LEDataRsp)
    );

    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_LEDataInd = %d",
        sizeof(TBlueAPI_LEDataInd)
    );
	BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_LEDataChannelParameterInfo = %d",
        sizeof(TBlueAPI_LEDataChannelParameterInfo)
    );

	BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_LEDataChannelCreditsAlertInfo =%d",
        sizeof(TBlueAPI_LEDataChannelCreditsAlertInfo)
    );
    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_LEDataChannelDeleteInfo = %d",
        sizeof(TBlueAPI_LEDataChannelDeleteInfo)
    );

    BLUEAPI_TRACE_PRINTF_1(
        BLUEAPI_TRACE_MASK_ERROR,
        "TBlueAPI_LEPsmSecuritySetRsp = %d",
        sizeof(TBlueAPI_LEPsmSecuritySetRsp)
    );

}


void blueAPI_dump_messageOffset(void)
{

    BLUEFACE_TRACE_PRINTF_4(BLUEFACE_TRACE_MASK_ERROR,
                            "sizeof(TBlueAPI_DsMessage) = %d, sizeof(TBlueAPI_UsMessage) = %d, sizeof(MESSAGE_T) = %d, sizeof(TblueFaceMsg) = %d",
                            sizeof(TBlueAPI_DsMessage),
                            sizeof(TBlueAPI_UsMessage),
                            sizeof(MESSAGE_T),
                            sizeof(TblueFaceMsg)
                           );
    BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_ERROR,
                           "offsetof(TBlueAPI_DsMessage, p.GATTAttributeUpdateReq.data) = %d",
                           offsetof(TBlueAPI_DsMessage, p.GATTAttributeUpdateReq.data)
                          );

    BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_ERROR,
                           "offsetof(TBlueAPI_DsMessage, p.GATTAttributeReadConf.data) = %d",
                           offsetof(TBlueAPI_DsMessage, p.GATTAttributeReadConf.data)
                          );
    BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_ERROR,
                           "offsetof(TBlueAPI_DsMessage, p.GATTAttributeWriteReq.data) = %d",
                           offsetof(TBlueAPI_DsMessage, p.GATTAttributeWriteReq.data)
                          );
    BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_ERROR,
                           "offsetof(TBlueAPI_DsMessage, p.GATTAttributePrepareWriteConf.data) = %d",
                           offsetof(TBlueAPI_DsMessage, p.GATTAttributePrepareWriteConf.data)
                          );
	
    BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_ERROR,
                           "offsetof(TBlueAPI_DsMessage, p.LEDataReq.data) = %d",
                           offsetof(TBlueAPI_DsMessage, p.LEDataReq.data)
                          );

    BLUEFACE_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_ERROR,
                            "blueFaceMsgSize + offsetof(TGATT_ATTRIB_UPDATE_REQ, data) = %d",
                            blueFaceMsgSize + offsetof(TGATT_ATTRIB_UPDATE_REQ, data)
                           );


    BLUEFACE_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_ERROR,
                            "blueFaceMsgSize + offsetof(TGATT_ATTRIB_READ_RESP, data) = %d",
                            blueFaceMsgSize + offsetof(TGATT_ATTRIB_READ_RESP, data)
                           );

    BLUEFACE_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_ERROR,
                            "blueFaceMsgSize + offsetof(TGATT_ATTRIB_WRITE_REQ, data) = %d",
                            blueFaceMsgSize + offsetof(TGATT_ATTRIB_WRITE_REQ, data)
                           );


    BLUEFACE_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_ERROR,
                            "blueFaceMsgSize + offsetof(TGATT_ATTRIB_WRITE_RESP, data) = %d",
                            blueFaceMsgSize + offsetof(TGATT_ATTRIB_WRITE_RESP, data)
                           );
	BLUEFACE_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_ERROR,
                            "blueFaceMsgSize + offsetof(TLE_DATA_REQ, data) = %d",
                            blueFaceMsgSize + offsetof(TLE_DATA_REQ, data)
                           );

    //RX indification/notification
    BLUEFACE_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_ERROR,
                            "offsetof(TblueFaceMsg, p.gattAttribIndicationIndication.data) = %d",
                            offsetof(TblueFaceMsg, p.gattAttribIndicationIndication.data)
                           );

    BLUEFACE_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_ERROR,
                            "offsetof(TBlueAPI_UsMessage, p.GATTAttributeInd.data) = %d",
                            offsetof(TBlueAPI_UsMessage, p.GATTAttributeInd.data)
                           );


    //Write command indication
    BLUEFACE_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_ERROR,
                            "offsetof(TblueFaceMsg, p.gattAttribWriteIndication.data) = %d",
                            offsetof(TblueFaceMsg, p.gattAttribWriteIndication.data)
                           );

    BLUEFACE_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_ERROR,
                            "offsetof(TBlueAPI_UsMessage, p.GATTAttributeWriteInd.data) = %d",
                            offsetof(TBlueAPI_UsMessage, p.GATTAttributeWriteInd.data)
                           );

    BLUEFACE_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_ERROR,
                            "offsetof(TGATTAttribWriteIndParam, data) = %d",
                            offsetof(TGATTAttribWriteIndParam, data)
                           );
    //
    BLUEFACE_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_ERROR,
                            "offsetof(TblueFaceMsg, p.gattAttribUpdateConfirmation.list)= %d",
                            offsetof(TblueFaceMsg, p.gattAttribUpdateConfirmation.list)
                           );

    BLUEFACE_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_ERROR,
                            "offsetof(TGATTAttribUpdateConfParam, list) = %d",
                            offsetof(TGATTAttribUpdateConfParam, list)
                           );

    BLUEFACE_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_ERROR,
                            "offsetof(TblueFaceMsg, p.gattDiscoveryIndication.list) = %d",
                            offsetof(TblueFaceMsg, p.gattDiscoveryIndication.list)
                           );

    BLUEFACE_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_ERROR,
                            "offsetof(TblueFaceMsg, p.gattAttribReadConfirmation.handlesData) = %d",
                            offsetof(TblueFaceMsg, p.gattAttribReadConfirmation.handlesData)
                           );

    BLUEFACE_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_ERROR,
                            "offsetof(TGATTAttribReadConfParam, handlesData) = %d",
                            offsetof(TGATTAttribReadConfParam, handlesData)
                           );

    BLUEFACE_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_ERROR,
                            "offsetof(TblueFaceMsg, p.gattAttribWriteConfirmation.data) = %d",
                            offsetof(TblueFaceMsg, p.gattAttribWriteConfirmation.data)
                           );

    BLUEFACE_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_ERROR,
                            "offsetof(TGATTAttribWriteConfParam, data) = %d",
                            offsetof(TGATTAttribWriteConfParam, data)
                           );
	BLUEFACE_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_ERROR,
                            "offsetof(TblueFaceMsg, p.LEDataIndication.data) = %d",
                            offsetof(TblueFaceMsg, p.LEDataIndication.data)
                           );
	BLUEFACE_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_ERROR,
                            "offsetof(TblueFaceMsg, p.gattGetLNameConfirmation.data) = %d",
                             offsetof(TblueFaceMsg, p.gattGetLNameConfirmation.data)
                           );
	BLUEFACE_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_ERROR,
                            "offsetof(TGATTGetDeviceNameParam, data) = %d",
                             offsetof(TGATTGetDeviceNameParam, data)
                           );
	BLUEFACE_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_ERROR,
                            "offsetof(TBlueAPI_UsMessage, p.DeviceConfigDeviceNameGetRsp.deviceName) = %d",
                             offsetof(TBlueAPI_UsMessage, p.DeviceConfigDeviceNameGetRsp.deviceName)
                           );
	BLUEFACE_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_ERROR,
                            "offsetof(TBlueAPI_UsMessage, p.LEDataInd.data) = %d",
                             offsetof(TBlueAPI_UsMessage, p.LEDataInd.data)
                           );

}
#endif

void blueAPI_dump_ProtocolData(void)
{
    BLUEFACE_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_ERROR,
                            "Total = %d",
                            sizeof(THCI) + sizeof(TL2C) + sizeof(TGATT) + sizeof(TBTA) + sizeof(TBlueAPI_Data) + sizeof(TBtSM)
                           );

    BLUEFACE_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_ERROR,
                            "sizeof(THCI) = %d, QueueID = 0x%x",
                            sizeof(THCI),
                            hciQueueID
                           );
  #if 0
    BLUEFACE_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_ERROR,
                            "sizeof(TL2C) = %d, QueueID = 0x%x",
                            sizeof(TL2C),
                            pL2C->QueueID
                           );
  #endif
    BLUEFACE_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_ERROR,
                            "sizeof(TGATT) = %d, QueueID = 0x%x",
                            sizeof(TGATT),
                            gattQueueID
                           );
    BLUEFACE_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_ERROR,
                            "sizeof(TBTA) = %d, QueueID = 0x%x",
                            sizeof(TBTA),
                            btaQueueID
                           );
#if 0
    BLUEFACE_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_ERROR,
                            "usAclPoolID = 0x%x, dsAclPoolID = 0x%x",
                            pBTA->usAclPoolID,
                            pBTA->dsAclPoolID
                           );
#endif
    BLUEFACE_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_ERROR,
                            "sizeof(TBlueAPI_Data) = %d, QueueID = 0x%x",
                            sizeof(TBlueAPI_Data),
                            blueAPIQueueID
                           );

    BLUEFACE_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_ERROR,
                            "sizeof(TBtSM) = %d, QueueID = 0x%x",
                            sizeof(TBtSM),
                            btsmQueueID
                           );
}

void blueAPI_dump_all_pools(void)
{
    osPoolDump();
}

#if 0
void blueAPI_dump_memory(void)
{
    blueAPI_dump_gatt_queue();
    BLUEFACE_TRACE_PRINTF_4(BLUEFACE_TRACE_MASK_ERROR,
                            "DataOffBottom:%p, DataOffLastMem:%p, DataOff used = %d, DataOff avaible= %d",
                            DataRamPartialOffHeapBottom,
                            DataRamPartialOffHeapLastMem,
                            DataRamPartialOffHeapLastMem + gDataOffSize - DataRamPartialOffHeapBottom,
                            DataRamPartialOffHeapBottom - DataRamPartialOffHeapLastMem
                           );
    BLUEFACE_TRACE_PRINTF_4(BLUEFACE_TRACE_MASK_ERROR,
                            "DataOnBottom%p, DataOnLastMem%p, DataOn used = %d, DataOn avaible = %d",
                            DataRamPartialOnHeapBottom,
                            DataRamPartialOnHeapLastMem,
                            DataRamPartialOnHeapLastMem + gDataOnSize - DataRamPartialOnHeapBottom,
                            DataRamPartialOnHeapBottom - DataRamPartialOnHeapLastMem
                           );
    BLUEFACE_TRACE_PRINTF_4(BLUEFACE_TRACE_MASK_ERROR,
                            "BufOffBottom:%p,BufOffLastMem:%p, BufOff used = %d, BufOff avaible = %d",
                            BufRamPartialOffHeapBottom,
                            BufRamPartialOffHeapLastMem,
                            BufRamPartialOffHeapLastMem + gBufOffSize - BufRamPartialOffHeapBottom,
                            BufRamPartialOffHeapBottom - BufRamPartialOffHeapLastMem
                           );
}
#endif

#if 0
uint16_t blueAPI_dump_DataRamOff_Module(void)
{
    uint16_t totalDataOffRamSize = 
    sizeof(TBlueAPI_LinkDescriptor)*otp_str_data.gEfuse_UpperStack_s.num_link_doff+
    sizeof(TbtLink)*otp_str_data.gEfuse_UpperStack_s.num_link_doff+
    sizeof(TBlueAPI_MCL)*otp_str_data.gEfuse_UpperStack_s.num_link_doff+
    sizeof(TBlueAPI_MDL)*otp_str_data.gEfuse_UpperStack_s.num_link_doff+
    sizeof(TSECLINK)*otp_str_data.gEfuse_UpperStack_s.num_link_doff+
    sizeof(TGATTTxData)*otp_str_data.gEfuse_UpperStack_s.gatt_max_tx_wsize *otp_str_data.gEfuse_UpperStack_s.num_link_doff+
    //sizeof(TL2CTxData)*otp_str_data.gEfuse_UpperStack_s.le_data_max_tx_wsize*otp_str_data.gEfuse_UpperStack_s.num_le_data_chan_doff+
    sizeof(TGATTL2CChannel)*otp_str_data.gEfuse_UpperStack_s.num_link_doff+
    sizeof(TGATTClient)*otp_str_data.gEfuse_UpperStack_s.num_link_doff+
    sizeof(TGATTRClient)*otp_str_data.gEfuse_UpperStack_s.num_link_doff+
    sizeof(ThciHandleDesc)*otp_str_data.gEfuse_UpperStack_s.num_link_doff+
    sizeof(T_ACLPOOLDESC)*otp_str_data.gEfuse_UpperStack_s.num_link_doff+
    //sizeof(T_L2CAP_LE_DATA_CHANNEL)*otp_str_data.gEfuse_UpperStack_s.num_le_data_chan_doff+
    sizeof(QUEUE_T)*os.os_buffer_callback_buffer_count+
    sizeof(TQueueElement)*os.QueueElementCount+
    sizeof(TQueueElement)*os.QueueElementCount+
    (BT_SYS_SHORT_BUFFER_SIZE+4)*BT_SYS_SHORT_BUFFER_COUNT + (BT_SYS_MIDDLE_BUFFER_SIZE+4)*BT_SYS_MIDDLE_BUFFER_COUNT+
    (otp_str_data.gEfuse_UpperStack_s.UpsteamShortBufferLength+4)*otp_str_data.gEfuse_UpperStack_s.UpsteamShortBufferCount+
    (otp_str_data.gEfuse_UpperStack_s.DownstreamLongBufferLength + 4)*otp_str_data.gEfuse_UpperStack_s.DownstreamLongBufferCount+
    (otp_str_data.gEfuse_UpperStack_s.UpstreamLongBufferLength+4)*otp_str_data.gEfuse_UpperStack_s.UpstreamLongBufferCount+
    (sizeof(TBlueAPI_UsMessage) + 4)*((otp_str_data.gEfuse_UpperStack_s.num_link_don + otp_str_data.gEfuse_UpperStack_s.num_link_doff) * 3 + 3)+
    (sizeof(TBlueAPI_DsMessage) + 4)*((otp_str_data.gEfuse_UpperStack_s.num_link_don + otp_str_data.gEfuse_UpperStack_s.num_link_doff) * 3 + 2);

    totalDataOffRamSize = (totalDataOffRamSize + 3)&~3;





    return totalDataOffRamSize;

}
#endif

#if 0
uint16_t blueAPI_dump_DataRamOn_Module(void)
{
    uint16_t nvSize = 0;  

    if(otp_str_data.gEfuse_UpperStack_s.support_key_store)
    {
        nvSize = 588;
    }
    
    uint16_t totalDataOnRamSize =
    sizeof(TBTMOSData) +
    sizeof(TBlueAPI_LinkDescriptor)*otp_str_data.gEfuse_UpperStack_s.num_link_don+
    sizeof(TBlueAPI_MCL)*otp_str_data.gEfuse_UpperStack_s.num_link_don+
    sizeof(TBlueAPI_MDL)*otp_str_data.gEfuse_UpperStack_s.num_link_don+
    sizeof(TBTA)+
    sizeof(TbtLink)*otp_str_data.gEfuse_UpperStack_s.num_link_don+
    sizeof(TBtSM)+    
    sizeof(TSECLINK)*otp_str_data.gEfuse_UpperStack_s.num_link_don+
    sizeof(TGATT)+
    sizeof(TGATTTxData)*otp_str_data.gEfuse_UpperStack_s.gatt_max_tx_wsize+
    //sizeof(TL2CTxData)*otp_str_data.gEfuse_UpperStack_s.le_data_max_tx_wsize*otp_str_data.gEfuse_UpperStack_s.num_le_data_chan_don+
    sizeof(TGATTL2CChannel)*otp_str_data.gEfuse_UpperStack_s.num_link_don+
    sizeof(TGATTClient)*otp_str_data.gEfuse_UpperStack_s.num_link_don+
    sizeof(TGATTRClient)*otp_str_data.gEfuse_UpperStack_s.num_link_don+
    sizeof(TGATTService)*otp_str_data.gEfuse_UpperStack_s.gatt_max_service_count+
    sizeof(TGATTCCCBitsTable)*otp_str_data.gEfuse_UpperStack_s.gattServerCCCbitsCount+
    sizeof(THCI)+
    sizeof(ThciHandleDesc)*otp_str_data.gEfuse_UpperStack_s.num_link_don+
    sizeof(TL2C)+
    sizeof(T_ACLPOOLDESC)*otp_str_data.gEfuse_UpperStack_s.num_link_don+
    //sizeof(T_L2CAP_LE_DATA_CHANNEL)*otp_str_data.gEfuse_UpperStack_s.num_le_data_chan_don+
    sizeof(TIMER_QUEUE_ELEMENT_T)*g_time_queue_element_count+
    sizeof(TQueueNames)*os.QueueNameTableSize*OS_MAX_QUEUE_NAME+
    sizeof(TQueueNameTable)*os.QueueNameTableSize+
    sizeof(TPartition)*(6+2)+
    sizeof(TBuffer)*
    (BT_SYS_SHORT_BUFFER_COUNT + BT_SYS_MIDDLE_BUFFER_COUNT+
    otp_str_data.gEfuse_UpperStack_s.UpsteamShortBufferCount + 
    otp_str_data.gEfuse_UpperStack_s.UpstreamLongBufferCount + 
    (otp_str_data.gEfuse_UpperStack_s.DownsteamShortBufferCount + otp_str_data.gEfuse_UpperStack_s.DownstreamLongBufferCount)+
    ((otp_str_data.gEfuse_UpperStack_s.num_link_don + otp_str_data.gEfuse_UpperStack_s.num_link_doff) * 3 + 3)+
    ((otp_str_data.gEfuse_UpperStack_s.num_link_don + otp_str_data.gEfuse_UpperStack_s.num_link_doff) * 3 + 2)
    ) +
    nvSize;
    
    totalDataOnRamSize = (totalDataOnRamSize +3 )&~3;    

    return totalDataOnRamSize;

}
#endif

uint16_t blueAPI_dump_BufRamOff_Module(void)
{
    uint16_t totalBufOffRamSize =    
    (otp_str_data.gEfuse_UpperStack_s.DownsteamShortBufferLength + 4)*otp_str_data.gEfuse_UpperStack_s.DownsteamShortBufferCount;

    return totalBufOffRamSize;
}

#if 0
void blueAPI_dump_gatt_queue(void)
{
    BLUEFACE_TRACE_PRINTF_2(BLUEFACE_TRACE_MASK_ERROR,
                                                    "TxDataQueueFree.ElementCount = %d, TxDataQueue.ElementCount = %d",
                                                    g_pGATT->pL2CChannelDon[0].TxDataQueueFree.ElementCount, 
                                                    g_pGATT->pL2CChannelDon[0].TxDataQueue.ElementCount
                                                    );

}
#endif




