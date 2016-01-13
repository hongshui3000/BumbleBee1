enum { __FILE_NUM__ = 0 };
/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file      test_legacy_application.c
* @brief     test legacy application implementation
* @details   test legacy application implementation
* @author    kyle_xu
* @date      2015-11-25
* @version   v0.1
* *********************************************************************************************************
*/
#include "os_mem.h"
#include "application.h"
#include "gap.h"
#include "gapbond_legacy.h"
#include "profile_common.h"
#include "trace.h"
#include "string.h"
#include "legacy.h"
#include "sdp_code.h"
#include "common_defs.h"

#include "my_stdio.h"

/** @brief  global variable, indicate current GAP role state. */
gaprole_States_t gapProfileState = GAPSTATE_INIT;
static void AppLegacyCallback(void *buf, TLegacyUsMsg legacy_msg);

static void legacy_HandleBtGapStateChangeEvt(uint8_t new_state)
{
    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "legacy_HandleBtGapStateChangeEvt: new state = %d", 1, new_state);

    gapProfileState = (gaprole_States_t)new_state;
}

static void legacy_HandleBtGapBondStateChangeEvt(uint8_t new_state)
{
    switch (new_state)
    {
    case GAPBOND_PAIRING_STATE_STARTED:
        DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BT_MSG_TYPE_BOND_STATE_CHANGE:(GAPBOND_PAIRING_STATE_STARTED)", 0);
        break;

    case GAPBOND_PAIRING_STATE_COMPLETE:
        DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BT_MSG_TYPE_BOND_STATE_CHANGE:(GAPBOND_PAIRING_STATE_COMPLETE)", 0);
        break;

    case GAPBOND_PAIRING_STATE_BONDED:
        DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BT_MSG_TYPE_BOND_STATE_CHANGE:(GAPBOND_PAIRING_STATE_BONDED)", 0);
        break;

    default:
        DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BT_MSG_TYPE_BOND_STATE_CHANGE:(unknown newstate: %d)", 1, new_state);
        break;
    }
}

static void legacy_HandleBtGapEncryptStateChangeEvt(uint8_t new_state)
{
    switch (new_state)
    {
    case GAPBOND_ENCRYPT_STATE_ENABLED:
        DBG_BUFFER(MODULE_APP, LEVEL_INFO, "GAPBOND_ENCRYPT_STATE_ENABLED", 0);
        break;

    case GAPBOND_ENCRYPT_STATE_DISABLED:
        DBG_BUFFER(MODULE_APP, LEVEL_INFO, "GAPBOND_ENCRYPT_STATE_DISABLED", 0);
        break;

    default:
        break;
    }
}

/**
* @brief
*
*
* @param   pBeeIoMsg
* @return  void
*/
static void legacy_HandleBtGapMessage(BEE_IO_MSG *p_msg)
{
    BT_STACK_MSG stack_msg;
    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "legacy_HandleBtGapMessage subType = %d", 1, p_msg->subType);    
    memcpy(&stack_msg, &p_msg->parm, sizeof(p_msg->parm));

    switch (p_msg->subType)
    {
    case BT_MSG_TYPE_CONN_STATE_CHANGE:
        {
            DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BT_MSG_TYPE_CONN_STATE_CHANGE:(%d->%d)",
                       2, gapProfileState, stack_msg.msgData.gapConnStateChange.newState);

            legacy_HandleBtGapStateChangeEvt(stack_msg.msgData.gapConnStateChange.newState);
        }
        break;

    case BT_MSG_TYPE_BOND_STATE_CHANGE:
        {
            legacy_HandleBtGapBondStateChangeEvt(stack_msg.msgData.gapBondStateChange.newState);
        }
        break;

    case BT_MSG_TYPE_BOND_PASSKEY_DISPLAY:
        {
            uint32_t display_value = 0;
            GAPBondlegacy_GetParameter(GAPBOND_PASSKEY, &display_value);
            DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BT_MSG_TYPE_BOND_PASSKEY_DISPLAY: %d", 1, display_value);
        }
        break;

    case BT_MSG_TYPE_BOND_PASSKEY_INPUT:
        {
            uint32_t passkey = 888888;
            DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BT_MSG_TYPE_BOND_PASSKEY_INPUT", 0);

            GAPBondlegacy_SetParameter(GAPBOND_PASSKEY, sizeof(passkey), &passkey);
            GAPBondlegacy_InputPassKey();
        }
        break;

    case BT_MSG_TYPE_BOND_LEGACY_OOB_INPUT:
        {
            uint8_t oobdata_r[KEYLEN] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
            uint8_t oobdata_c[KEYLEN] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
            DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BT_MSG_TYPE_BOND_OOB_INPUT", 0);

            GAPBondlegacy_SetParameter(GAPBOND_OOB_DATA_R, KEYLEN, oobdata_r);
            GAPBondlegacy_SetParameter(GAPBOND_OOB_DATA_C, KEYLEN, oobdata_c);
            GAPBondlegacy_InputOOBData();
        }
        break;

    case BT_MSG_TYPE_ENCRYPT_STATE_CHANGE:
        legacy_HandleBtGapEncryptStateChangeEvt(stack_msg.msgData.gapEncryptStateChange.newState);
        break;

    default:
        DBG_BUFFER(MODULE_APP, LEVEL_ERROR, "legacy_HandleBtGapMessage unknown subtype", 1, p_msg->subType);
        break;
    }
}

/**
* @brief  All the application events are pre-handled in this function.
*
* All the IO MSGs are sent to this function.
* Then the event handling function shall be called according to the MSG type.
*
* @param   io_driver_msg_recv  The BEE_IO_MSG from peripherals or BT stack state machine.
* @return  void
*/
void AppHandleIODriverMessage(BEE_IO_MSG io_driver_msg_recv)
{
    uint16_t msgtype = io_driver_msg_recv.IoType;

    switch (msgtype)
    {
    case BT_STATUS_UPDATE:
        legacy_HandleBtGapMessage(&io_driver_msg_recv);
        break;

    case IO_UART_MSG_TYPE:
        break;

    default:
        break;
    }
}

static void AppHandleInquiryInfo(PBlueAPI_InquiryDeviceInfo pinfo)
{
    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "APP inquiry info: remote bd[%x:%x:%x:%x:%x:%x], COD[0x%x]\n", 7,
                pinfo->remote_BD[0], pinfo->remote_BD[1], pinfo->remote_BD[2], pinfo->remote_BD[3],
                pinfo->remote_BD[4], pinfo->remote_BD[5], pinfo->remote_Device_Class);
}

static void AppHandleInquiryRsp(PBlueAPI_InquiryRsp prsp)
{
    uint8_t bd[6];
    bd[5] = 0x0C;
    bd[4] = 0x12;
    bd[3] = 0x62;
    bd[2] = 0x27;
    bd[1] = 0x8B;
    bd[0] = 0x29;

    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "APP inquiry response, cause: %d\n", 1, prsp->cause);
    legacy_StartSDPDiscovery(bd, blueAPI_MDEPUUIDL2CAP, true, AppLegacyCallback);
}

static void AppHandleSDPDiscoveryInfo(PBlueAPI_SDPEndpointInd pinfo)
{
    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "APP SDP discovery info: MDEP_ID=%2.2x, MDEP_DataType=%4.4x, remoteVersion=%x, supportedfeatures=%d \r\n", 4,
					pinfo->remote_MDEP_ID, pinfo->remote_MDEP_DataType, pinfo->remoteversion, pinfo->supportedfeatures);
}

static void AppHandleSDPDiscoveryRsp(PBlueAPI_SDPDiscoveryRsp prsp)
{
    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "APP SDP discovery response, cause: %d\n", 1, prsp->cause);
    uint8_t bd[6];
    bd[5] = 0x0C;
    bd[4] = 0x12;
    bd[3] = 0x62;
    bd[2] = 0x27;
    bd[1] = 0x8B;
    bd[0] = 0x29;
    legacy_StartGATTSDPDiscovery(bd, UUID_ATT, true, AppLegacyCallback);
}

static void AppHandleGATTSDPDiscoveryInfo(PBlueAPI_GATTSDPDiscoveryInd pinfo)
{
    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "APP GATT SDP discovery info: service handle=%x, uuid=%x, start handle=%d, end handle=%d \r\n", 4,
					pinfo->serviceHandle, pinfo->remote_GATT_UUID, pinfo->remote_GATT_StartHandle, pinfo->remote_GATT_EndHandle);
}

static void AppHandleGATTSDPDiscoveryRsp(PBlueAPI_GATTSDPDiscoveryRsp prsp)
{
    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "APP GATT SDP discovery response, cause: %d\n", 1, prsp->cause);
    uint8_t bd[6];
    bd[5] = 0x0C;
    bd[4] = 0x12;
    bd[3] = 0x62;
    bd[2] = 0x27;
    bd[1] = 0x8B;
    bd[0] = 0x29;
    GAPBondlegacy_Pair(bd);
}

static void AppHandleDeviceInfo(PBlueAPI_DIDDeviceInd pinfo)
{
    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "APP device info: vendorID=%x, vendorID source=%x, productID=%x, remote version=%x \r\n", 4,
                pinfo->remote_VendorID, pinfo->remote_VendorID_Source, pinfo->remote_ProductID, pinfo->remote_Version);
}

static void AppLegacyCallback(void *buf, TLegacyUsMsg legacy_msg)
{
    switch (legacy_msg)
    {
    case INQUIRY_DEVICE_INFO:
        AppHandleInquiryInfo((PBlueAPI_InquiryDeviceInfo)buf);
        break;

    case INQUIRY_RSP:
        AppHandleInquiryRsp((PBlueAPI_InquiryRsp)buf);
        break;

    case SDP_DISCOVERY_INFO:
        AppHandleSDPDiscoveryInfo((PBlueAPI_SDPEndpointInd)buf);
        break;

    case SDP_DISCOVERY_RSP:
        AppHandleSDPDiscoveryRsp((PBlueAPI_SDPDiscoveryRsp)buf);
        break;

    case GATT_SDP_DISCOVERY_INFO:
        AppHandleGATTSDPDiscoveryInfo((PBlueAPI_GATTSDPDiscoveryInd)buf);
        break;

    case GATT_SDP_DISCOVERY_RSP:
        AppHandleGATTSDPDiscoveryRsp((PBlueAPI_GATTSDPDiscoveryRsp)buf);
        break;

    case DEVICE_INFO:
        AppHandleDeviceInfo((PBlueAPI_DIDDeviceInd)buf);
        break;

    default:
        break;
    }
}

TAppResult AppProfileCallback(uint8_t service_id, void *p_data)
{
    if (service_id == ProfileAPI_ServiceUndefined)
    {
        TEventInfoCBs_t *p_info = (TEventInfoCBs_t *)p_data;

        switch (p_info->eventId)
        {
        case PROFILE_EVT_SDP_REG_COMPLETE:
            DBG_BUFFER(MODULE_APP, LEVEL_INFO, "--------profile callback PROFILE_EVT_SDP_REG_COMPLETE\n", 0);
            printf("SDP reg complete\r\n");
            break;

        default:
            break;
        }
    }
    return AppResult_Success;
}

void TestProfileInit(void)
{
    uint16_t length = 0;
    void *pbuffer = NULL;

    length = legacy_SDPRecordLength("< I<UU> I<<U><UB>> I<U> I<III> I<<UI>> IS II >",
                            SDP_ATTR_SERVICECLASSIDLIST, UUID_HANDSFREE, UUID_GENERICAUDIO,
                            SDP_ATTR_PROTOCOLDESCRIPTORLIST,UUID_L2CAP, UUID_RFCOMM, 0x03,
                            SDP_ATTR_BROWSEGROUPLIST, UUID_PUBLIC_BROWSE_GROUP,
                            SDP_ATTR_LANGUAGEBASEATTRIBUTEIDLIST,SDP_LANGUAGE_ENGLISH, SDP_CHARCODE_UTF8, SDP_BASE_LANG_OFFSET,
                            SDP_ATTR_BLUETOOTHPROFILEDESCRIPTORLIST, UUID_HANDSFREE, 0x0107,
                            SDP_ATTR_SERVICENAME + SDP_BASE_LANG_OFFSET, "hfp",
                            SDP_ATTR_SUPPORTEDFEATURES, 0x03ff
                            );

    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "TestProfileInit: SDP buffer length cal is %d", 1, length);

    if (length)
    {
        pbuffer = osMemoryAllocate(RAM_TYPE_DATA_ON, length);
        length = legacy_SDPCreateDes(pbuffer,
                                "< I<UU> I<<U><UB>> I<U> I<III> I<<UI>> IS II >",
                                SDP_ATTR_SERVICECLASSIDLIST, UUID_HANDSFREE, UUID_GENERICAUDIO,
                                SDP_ATTR_PROTOCOLDESCRIPTORLIST,UUID_L2CAP, UUID_RFCOMM, 0x03,
                                SDP_ATTR_BROWSEGROUPLIST, UUID_PUBLIC_BROWSE_GROUP,
                                SDP_ATTR_LANGUAGEBASEATTRIBUTEIDLIST,SDP_LANGUAGE_ENGLISH, SDP_CHARCODE_UTF8, SDP_BASE_LANG_OFFSET,
                                SDP_ATTR_BLUETOOTHPROFILEDESCRIPTORLIST, UUID_HANDSFREE, 0x0107,
                                SDP_ATTR_SERVICENAME + SDP_BASE_LANG_OFFSET, "hfp",
                                SDP_ATTR_SUPPORTEDFEATURES, 0x03ff
                                );

        DBG_BUFFER(MODULE_APP, LEVEL_INFO, "TestProfileInit: SDP buffer length is %d", 1, length);

        if (length)
        {
            legacy_AddSDPRecord(pbuffer, length);
        }
        else
        {
            osMemoryFree(pbuffer);
        }
    }
}
