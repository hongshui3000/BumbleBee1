enum { __FILE_NUM__ = 0 };

/**
 ***************************************************************************************
 *               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
 ***************************************************************************************
 * @file      dualmode.c
 * @brief    This file provides gap dualmode role functions
 * @details
 * @author  kyle_xu
 * @date     2015-11-28
 * @version  v0.1
 ***************************************************************************************
 */
#include <dualmode.h>
#include <legacy.h>
#include <peripheral.h>
#include <central.h>
#include <broadcaster.h>
#include <observer.h>
#include <gap.h>
#include <blueapi.h>

extern uint8_t  gapPara_profileRole;

/**
 * @brief      dulemode start bt stack 
 * @param   void
 * @return     bool
 */
bool dualmode_StartBtStack(void)
{
    return GAP_StartBtStack();
}

/**
 * @brief      LE process TBlueAPI_UsMessage message from bt stack,
 *              call different function to process message based on role
 * @param    pmsg  - pointer to TBlueAPI_UsMessage message
 * @return     bool
 */
bool LE_HandleBlueAPIMessage(PBlueAPI_UsMessage pmsg)
{
    switch(gapPara_profileRole)
    {
    case GAP_PROFILE_PERIPHERAL:
        peripheral_HandleBlueAPIMessage(pmsg);
        break;

    case GAP_PROFILE_CENTRAL:
        central_HandleBlueAPIMessage(pmsg);
        break;

    case GAP_PROFILE_BROADCASTER:
        broadcaster_HandleBlueAPIMessage(pmsg);
        break;

    case GAP_PROFILE_OBSERVER:
        observer_HandleBlueAPIMessage(pmsg);
        break;

    default:
        break;
    }

    return true;
}

/**
  * @brief  process blueAPI_EventRegisterRsp message from bt stack
  * @param  prsp - message sent from upper stack.
  * @retval void
  */
void dualmode_HandleRegisterRsp(PBlueAPI_RegisterRsp prsp)
{
    if (prsp->cause == blueAPI_CauseSuccess)
    {
        /* Upper Stack register success */
    }
    else
    {
        /* Upper Stack register fail */
    }
}

/**
 * @brief      process BlueAPI_InternalEventInfo message from stack
 * @param    pmsg  - pointer to TBlueAPI_UsMessage message
 * @return     void
 */
void dualmode_HandleInternalEventInfo(PBlueAPI_InternalEventInfo pinfo)
{
#ifdef GAP_DEBUG_FLAG
    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "dualmode_HandleInternalEventInfo: eventType=0x%x.", 1, pinfo->eventType);
#endif
}

/**
 * @brief      dualmode process TBlueAPI_UsMessage message from bt stack, only gap
 *                related messages are handled here
 * @param    pmsg  - pointer to TBlueAPI_UsMessage message
 * @return     void
 */
void dualmode_HandleDeviceConfigSetRsp(PBlueAPI_UsMessage pmsg)
{
    switch (pmsg->p.DeviceConfigSetRsp.opCode)
    {
    case blueAPI_DeviceConfigDevice:
    case blueAPI_DeviceConfigPagescan:
    case blueAPI_DeviceConfigInquiryscan:
    case blueAPI_DeviceConfigInquirymode:
    case blueAPI_DeviceConfigLinkpolicy:
        legacy_HandleBlueAPIMessage(pmsg);
        break;

    case blueAPI_DeviceConfigDeviceName:
    case blueAPI_DeviceConfigAppearance:
    case blueAPI_DeviceConfigPerPrefConnParam:
    case blueAPI_DeviceConfigSecurity:
        LE_HandleBlueAPIMessage(pmsg);
        break;

    default:
        break;
    }
}

/**
 * @brief      dualmode process TBlueAPI_UsMessage message from bt stack, only gap 
 *                related messages are handled here
 * @param    pmsg  - pointer to TBlueAPI_UsMessage message
 * @return     bool
 */
bool dualmode_HandleBlueAPIMessage(PBlueAPI_UsMessage pmsg)
{
    switch (pmsg->Command)
    {
    case blueAPI_EventConnectMDLRsp:
        if (pmsg->p.ConnectMDLRsp.remote_BD_type == blueAPI_RemoteBDTypeClassic)
        {
            legacy_HandleBlueAPIMessage(pmsg);
        }
        else
        {
            LE_HandleBlueAPIMessage(pmsg);
        }
        break;

    ///TODO: this group is send to le gap since mcl/mdl will not used in legacy, otherwise should fix it
    case blueAPI_EventDisconnectMDLRsp:
    case blueAPI_EventDisconnectMDLInd:
    case blueAPI_EventCreateMDLInd:
    case blueAPI_EventConnectMDLInfo:
    case blueAPI_EventDeleteMDLInfo:
    case blueAPI_EventMCLStatusInfo:
        LE_HandleBlueAPIMessage(pmsg);
        break;

    case blueAPI_EventACLStatusInfo:
        if (pmsg->p.ACLStatusInfo.remote_BD_type == blueAPI_RemoteBDTypeClassic)
        {
            legacy_HandleBlueAPIMessage(pmsg);
        }
        else
        {
            LE_HandleBlueAPIMessage(pmsg);
        }
        break;

    case blueAPI_EventRegisterRsp:
        dualmode_HandleRegisterRsp(&pmsg->p.RegisterRsp);
        break;

    case blueAPI_EventActInfo:
        legacy_HandleBlueAPIMessage(pmsg);
        LE_HandleBlueAPIMessage(pmsg);
        break;

    case blueAPI_EventInternalEventInfo:
        dualmode_HandleInternalEventInfo(&pmsg->p.InternalEventInfo);
        break;

    case blueAPI_EventDeviceConfigSetRsp:
        dualmode_HandleDeviceConfigSetRsp(pmsg);
        break;

    case blueAPI_EventAuthResultInd:
        if (pmsg->p.AuthResultInd.remote_BD_Type == blueAPI_RemoteBDTypeClassic)
        {
            legacy_HandleBlueAPIMessage(pmsg);
        }
        else
        {
            LE_HandleBlueAPIMessage(pmsg);
        }
        break;

    case blueAPI_EventAuthResultRequestInd:
        if (pmsg->p.AuthResultRequestInd.remote_BD_Type == blueAPI_RemoteBDTypeClassic)
        {
            legacy_HandleBlueAPIMessage(pmsg);
        }
        else
        {
            LE_HandleBlueAPIMessage(pmsg);
        }
        break;

    case blueAPI_EventAuthDeleteRsp:
        if (pmsg->p.AuthDeleteRsp.remote_BD_Type == blueAPI_RemoteBDTypeClassic)
        {
            legacy_HandleBlueAPIMessage(pmsg);
        }
        else
        {
            LE_HandleBlueAPIMessage(pmsg);
        }
        break;

    case blueAPI_EventAuthListInfo:
        if (pmsg->p.AuthListInfo.remote_BD_Type == blueAPI_RemoteBDTypeClassic)
        {
            legacy_HandleBlueAPIMessage(pmsg);
        }
        else
        {
            LE_HandleBlueAPIMessage(pmsg);
        }
        break;

    case blueAPI_EventAuthListRsp:
        if (pmsg->p.AuthListRsp.remote_BD_Type == blueAPI_RemoteBDTypeClassic)
        {
            legacy_HandleBlueAPIMessage(pmsg);
        }
        else
        {
            LE_HandleBlueAPIMessage(pmsg);
        }
        break;

    case blueAPI_EventRadioModeSetRsp:
    case blueAPI_EventLegacyRemoteOOBDataReqInd:
    case blueAPI_EventACLConfigRsp:
    case blueAPI_EventInquiryRsp:
    case blueAPI_EventInquiryDeviceInfo:
    case blueAPI_EventDeviceNameRsp:
    case blueAPI_EventDIDDeviceInd:
    case blueAPI_EventSDPRegisterRsp:
    case blueAPI_EventSDPDiscoveryRsp:
    case blueAPI_EventSDPEndpointInd:
    case blueAPI_EventGATTSDPDiscoveryRsp:
    case blueAPI_EventGATTSDPDiscoveryInd:
    case blueAPI_EventL2cProtocolRegisterRsp:
    case blueAPI_EventL2cConRsp:
    case blueAPI_EventL2cConInd:
    case blueAPI_EventL2cConActInd:
    case blueAPI_EventL2cDataInd:
    case blueAPI_EventL2cDiscRsp:
    case blueAPI_EventL2cDiscInd:
    case blueAPI_EventAuthRsp:
    case blueAPI_EventUserAuthorizationReqInd:
    case blueAPI_EventUserAuthRequestInd:
    case blueAPI_EventUserConfirmationReqInd:
    case blueAPI_EventKeypressNotificationRsp:
    case blueAPI_EventKeypressNotificationInfo:
    case blueAPI_EventLocalOOBDataRsp:
    case blueAPI_EventSCOConInd:
    case blueAPI_EventSCOConRsp:
    case blueAPI_EventSCOConActInd:
    case blueAPI_EventSCODiscInd:
        legacy_HandleBlueAPIMessage(pmsg);
        break;

    case blueAPI_EventUserPasskeyReqInd:            /* LE and legacy handle the same way */
    case blueAPI_EventUserPasskeyNotificationInfo:  /* LE and legacy handle the same way */
    case blueAPI_EventRemoteOOBDataReqInd:
    case blueAPI_EventDeviceConfigAppearanceGetRsp:
    case blueAPI_EventDeviceConfigPerPrefConnParamGetRsp:
    case blueAPI_EventDeviceConfigDeviceNameGetRsp:
    case blueAPI_EventLEAdvertiseRsp:
    case blueAPI_EventLEAdvertiseParameterSetRsp:
    case blueAPI_EventLEAdvertiseDataSetRsp:
    case blueAPI_EventLEScanRsp:
    case blueAPI_EventLEScanInfo:
    case blueAPI_EventLEModifyWhitelistRsp:
    case blueAPI_EventLEConnectionUpdateRsp:
    case blueAPI_EventLEConnectionUpdateInd:
    case blueAPI_EventLEConnectionParameterInfo:
    case blueAPI_EventLEPrivacyModeRsp:
    case blueAPI_EventCreateLEDataChannelRsp:
    case blueAPI_EventCreateLEDataChannelInd:
    case blueAPI_EventDisconnectLEDataChannelRsp:
    case blueAPI_EventDisconnectLEDataChannelInd:
    case blueAPI_EventSendLEFlowControlCreditRsp:
    case blueAPI_EventLEDataRsp:
    case blueAPI_EventLEDataInd:
    case blueAPI_EventLEDataChannelParameterInfo:
    case blueAPI_EventLEDataChannelCreditsAlertInfo:
    case blueAPI_EventLEDataChannelDeleteInfo:
    case blueAPI_EventLEPsmSecuritySetRsp:
    case blueAPI_EventVendorSetVoiceParaRsp:
    case blueAPI_EventSetBleTxPowerRsp:
    case blueAPI_EventSetRandomAddressRsp:
    case blueAPI_EventSetDataLengthRsp:
    case blueAPI_EventDataLengthChangeInfo:
    case blueAPI_EventGATTServiceRegisterRsp:
    case blueAPI_EventGATTAttributeUpdateRsp:
    case blueAPI_EventGATTAttributeUpdateStatusInd:
    case blueAPI_EventGATTAttributeReadInd:
    case blueAPI_EventGATTAttributeWriteInd:
    case blueAPI_EventGATTAttributePrepareWriteInd:
    case blueAPI_EventGATTAttributeExecuteWriteInd:
    case blueAPI_EventGATTAttributeWriteCommandInfo:
    case blueAPI_EventGATTCCCDInfo:
    case blueAPI_EventGATTDiscoveryRsp:
    case blueAPI_EventGATTDiscoveryInd:
    case blueAPI_EventGATTAttributeReadRsp:
    case blueAPI_EventGATTAttributeReadMultipleRsp:
    case blueAPI_EventGATTAttributeWriteRsp:
    case blueAPI_EventGATTAttributePrepareWriteRsp:
    case blueAPI_EventGATTAttributeExecuteWriteRsp:
    case blueAPI_EventGATTAttributeInd:
    case blueAPI_EventGATTAttributeNotificationInfo:
    case blueAPI_EventGATTSecurityRsp:
    case blueAPI_EventGATTServerStoreInd:
    case blueAPI_EventGATTMtuSizeInfo:
        LE_HandleBlueAPIMessage(pmsg);
        break;

    default:
        break;
    }

    if (pmsg)
    {
        blueAPI_BufferRelease(pmsg);
        pmsg = NULL;
    }

    return true;
}

