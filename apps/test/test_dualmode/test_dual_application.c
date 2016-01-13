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
#include "gapbond_dual.h"
#include "profile_common.h"
#include "trace.h"
#include "string.h"
#include "legacy.h"
#include "sdp_code.h"

#include "peripheral.h"
#include <common_defs.h>

#if SIMPLE_SERVICE_EN
extern uint8_t gSimpleProfileServiceId;
#endif
#if DIS_SERVICE_EN
extern uint8_t gDisServiceId;
#endif
#if BAS_SERVICE_EN
extern uint8_t gBasServiceId;
#endif
#if RSC_SERVICE_EN
extern uint8_t gRscServiceId;
uint8_t gRsc_SensorLocation = 0;
uint32_t gRsc_CulValue = 0;
#endif
#if CSC_SERVICE_EN
extern uint8_t gCscServiceId;
uint8_t gCsc_SensorLocation = 0;
uint32_t gCsc_CulValue = 0;
#endif

/** @brief  global variable, indicate current GAP role state. */
gaprole_States_t gapProfileState = GAPSTATE_INIT;
static void AppLegacyCallback(void *buf, TLegacyUsMsg legacy_msg);

static void dualmode_HandleBtGapStateChangeEvt(uint8_t new_state)
{
    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "dualmode_HandleBtGapStateChangeEvt: new state = %d", 1, new_state);

    switch (new_state)
    {
    case GAPSTATE_IDLE_NO_ADV_NO_CONN:
    {
        if (gapProfileState == GAPSTATE_CONNECTED)
        {
            uint8_t disc_reason;
            peripheralGetGapParameter(GAPPRRA_DISCONNECTED_REASON, &disc_reason);
            DBG_BUFFER(MODULE_APP, LEVEL_INFO, "dualmode_HandleBtGapStateChangeEvt: disc_reason = %d", 1, disc_reason);
#if RSC_SERVICE_EN
            RSC_SetParameter(RSC_PARAM_CTL_PNT_PROG_CLR, 0, NULL);
#endif
#if CSC_SERVICE_EN
            CSC_SetParameter(CSC_PARAM_CTL_PNT_PROG_CLR, 0, NULL);
#endif
            peripheral_StartAdvertising();
        }
        else
        {
            /* Link disconnect cause switch to this state. Set Bee to connectable mode */
            peripheral_StartAdvertising();
        }
    }
        break;

    case GAPSTATE_ADVERTISING:
        break;

    case GAPSTATE_CONNECTED:
        break;

    case GAPSTATE_CONNECTED_ADV:
        break;

    default:
        break;
    }

    gapProfileState = (gaprole_States_t)new_state;
}

static void dualmode_HandleBtGapBondStateChangeEvt(uint8_t new_state)
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

static void dualmode_HandleBtGapEncryptStateChangeEvt(uint8_t new_state)
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

static void dualmode_HandleBtGapConnParaChangeEvt(uint8_t status)
{
    if (status == 0)
    {
        uint16_t con_interval;
        uint16_t conn_slave_latency;
        uint16_t conn_supervision_timeout;

        peripheralGetGapParameter(GAPPRRA_CONN_INTERVAL, &con_interval);
        peripheralGetGapParameter(GAPPRRA_CONN_LATENCY, &conn_slave_latency);
        peripheralGetGapParameter(GAPPRRA_CONN_TIMEOUT, &conn_supervision_timeout);

        DBG_BUFFER(MODULE_APP, LEVEL_INFO,
                   "BT_MSG_TYPE_CONN_PARA_UPDATE_CHANGE success, con_interval = 0x%x, conn_slave_latency = 0x%x, conn_supervision_timeout = 0x%x",
                   3, con_interval, conn_slave_latency, conn_supervision_timeout);
    }
    else
    {
        DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BT_MSG_TYPE_CONN_PARA_UPDATE_CHANGE failed, status = %d",
                   1, status);
    }
}

/**
* @brief
*
*
* @param   pBeeIoMsg
* @return  void
*/
static void dualmode_HandleBtGapMessage(BEE_IO_MSG *p_msg)
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

            dualmode_HandleBtGapStateChangeEvt(stack_msg.msgData.gapConnStateChange.newState);
        }
        break;

    case BT_MSG_TYPE_BOND_STATE_CHANGE:
        {
            dualmode_HandleBtGapBondStateChangeEvt(stack_msg.msgData.gapBondStateChange.newState);
        }
        break;

    case BT_MSG_TYPE_BOND_PASSKEY_DISPLAY:
        {
            uint32_t display_value = 0;
            GAPBonddual_GetParameter(GAPBOND_PASSKEY, &display_value);
            DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BT_MSG_TYPE_BOND_PASSKEY_DISPLAY: %d", 1, display_value);
        }
        break;

    case BT_MSG_TYPE_BOND_PASSKEY_INPUT:
        {
            uint32_t passkey = 888888;
            DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BT_MSG_TYPE_BOND_PASSKEY_INPUT", 0);

            GAPBonddual_SetParameter(GAPBOND_PASSKEY, sizeof(passkey), &passkey);
            GAPBonddual_InputPassKey();
        }
        break;

    case BT_MSG_TYPE_BOND_LEGACY_OOB_INPUT:
        {
            uint8_t oobdata_r[KEYLEN] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
            uint8_t oobdata_c[KEYLEN] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
            DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BT_MSG_TYPE_BOND_LEGACY_OOB_INPUT", 0);

            GAPBonddual_SetParameter(GAPBOND_OOB_DATA_R, KEYLEN, oobdata_r);
            GAPBonddual_SetParameter(GAPBOND_OOB_DATA_C, KEYLEN, oobdata_c);
            GAPBonddual_InputlegacyOOBData();
        }
        break;

    case BT_MSG_TYPE_BOND_OOB_INPUT:
        {
            uint8_t oob_data[KEYLEN] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
            DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BT_MSG_TYPE_BOND_OOB_INPUT", 0);
    
            GAPBonddual_SetParameter(GAPBOND_OOB_DATA, KEYLEN, oob_data);
            GAPBonddual_InputLEOOBData();
        }
        break;

    case BT_MSG_TYPE_ENCRYPT_STATE_CHANGE:
        dualmode_HandleBtGapEncryptStateChangeEvt(stack_msg.msgData.gapEncryptStateChange.newState);
        break;

    case BT_MSG_TYPE_CONN_PARA_UPDATE_CHANGE:
        dualmode_HandleBtGapConnParaChangeEvt(stack_msg.msgData.gapConnParaUpdateChange.status);
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
        dualmode_HandleBtGapMessage(&io_driver_msg_recv);
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
            DBG_BUFFER(MODULE_APP, LEVEL_INFO, "profile callback PROFILE_EVT_SDP_REG_COMPLETE\n", 0);
            legacy_StartInquiry(0, 8, AppLegacyCallback);
            break;

        case PROFILE_EVT_SRV_REG_COMPLETE:// srv register result event.
            DBG_BUFFER(MODULE_APP, LEVEL_INFO, "profile callback PROFILE_EVT_SRV_REG_COMPLETE\n", 0);
            peripheral_StartAdvertising();
            break;

        case PROFILE_EVT_SEND_DATA_COMPLETE:
#if RSC_SERVICE_EN
            RSC_SetParameter(RSC_PARAM_CTL_PNT_PROG_CLR, 0, NULL);
#endif
#if CSC_SERVICE_EN
            CSC_SetParameter(CSC_PARAM_CTL_PNT_PROG_CLR, 0, NULL);
#endif
            DBG_BUFFER(MODULE_APP, LEVEL_INFO, "!!!************Ind/Not complete!!!**************\n", 0);
            break;

        default:
            break;
        }
    }

#if DIS_SERVICE_EN
    else if (service_id == gDisServiceId)
    {
        TDIS_CALLBACK_DATA *pDisCallbackData = (TDIS_CALLBACK_DATA *)p_data;
        switch (pDisCallbackData->msg_type)
        {
        case SERVICE_CALLBACK_TYPE_READ_CHAR_VALUE:
        {
            if(pDisCallbackData->msg_data.read_value_index == DIS_READ_MANU_NAME_INDEX)
            {
                /* Client is reading Manufacturer Name String. */
                /* May be do some thing here. */
            }
            else
            if(pDisCallbackData->msg_data.read_value_index == DIS_READ_MODEL_NUM_INDEX)
            {
                /* Client is reading Model Number String. */
                /* May be do some thing here. */

            }
        }
            break;
        }
    }
#endif

#if BAS_SERVICE_EN
    else if (service_id == gBasServiceId)
    {
        TBAS_CALLBACK_DATA *pBasCallbackData = (TBAS_CALLBACK_DATA *)p_data;
        switch (pBasCallbackData->msg_type)
        {
        case SERVICE_CALLBACK_TYPE_INDIFICATION_NOTIFICATION:
        {
            if(pBasCallbackData->msg_data.notification_indification_index == BAS_NOTIFY_BATTERY_LEVEL_ENABLE)
            {
                /* Battery Level notify has been enabled by Client. */
                /* Do some thing here. */
            }
            else
            if(pBasCallbackData->msg_data.notification_indification_index == BAS_NOTIFY_BATTERY_LEVEL_DISABLE)
            {
                /* Battery Level notify has been disabled by Client. */
                /* Do some thing here. */
            }
                     
        }
            break;
        case SERVICE_CALLBACK_TYPE_READ_CHAR_VALUE:
        {
            if(pBasCallbackData->msg_data.read_value_index == BAS_READ_BATTERY_LEVEL)
            {
                /* Battery Level has been read by Client. */
                /* May be so some thing here. */
            }
        }
            break;
        }

    }
#endif

#if RSC_SERVICE_EN
    else if (service_id == gRscServiceId)
    {
        TRSC_CALLBACK_DATA *pRscCallbackData = (TRSC_CALLBACK_DATA *)p_data;
        switch (pRscCallbackData->msg_type)
        {
        case SERVICE_CALLBACK_TYPE_INDIFICATION_NOTIFICATION:
        {
            if(pRscCallbackData->msg_data.notification_indification_index == RSC_NOTIFY_INDICATE_MEASUREMENT_ENABLE)
            {
                
            }
            else
            if(pRscCallbackData->msg_data.notification_indification_index == RSC_NOTIFY_INDICATE_MEASUREMENT_DISABLE)
            {
                
            }
            else
            if(pRscCallbackData->msg_data.notification_indification_index == RSC_NOTIFY_INDICATE_SC_CP_ENABLE)
            {
                
            }
            else
            if(pRscCallbackData->msg_data.notification_indification_index == RSC_NOTIFY_INDICATE_SC_CP_DISABLE)
            {
                
            }
                     
        }
            break;
        case SERVICE_CALLBACK_TYPE_READ_CHAR_VALUE:
        {
            if(pRscCallbackData->msg_data.read_value_index == RSC_READ_RSC_FEATURE)
            {
                uint16_t rsc_feature = RSC_ALL_FEATURE_SUPPORTED;
                RSC_SetParameter(RSC_PARAM_CSC_FEATURE, 2, &rsc_feature);
            }
            else
            if(pRscCallbackData->msg_data.read_value_index == RSC_READ_SENSOR_LOCATION)
            {
                RSC_SetParameter(RSC_PARAM_SENSOR_LOC, 1, &gRsc_SensorLocation);

            }
        }
            break;
        case SERVICE_CALLBACK_TYPE_WRITE_CHAR_VALUE:
        {

            if(pRscCallbackData->msg_data.write.opcode == RSC_CP_OPCODE_SET_CUMULATIVE)
            {
                gRsc_CulValue = pRscCallbackData->msg_data.write.cp_parameter.cumulative_value;
            }
            else
            if(pRscCallbackData->msg_data.write.opcode == RSC_CP_OPCODE_START_CALIBRATION)
            {
                
            }                
            else
            if(pRscCallbackData->msg_data.write.opcode == RSC_CP_OPCODE_UPDATE_SENS_LOC)
            {
                gRsc_SensorLocation = pRscCallbackData->msg_data.write.cp_parameter.sensor_location_value;
            }
            else
            if(pRscCallbackData->msg_data.write.opcode == RSC_CP_OPCODE_REQ_SENS_LOC_LIST)
            {

            }
            
        }
        break;                
        }
    }
#endif

#if CSC_SERVICE_EN
    else if (service_id == gCscServiceId)
    {
        TCSC_CALLBACK_DATA *pCscCallbackData = (TCSC_CALLBACK_DATA *)p_data;
        switch (pCscCallbackData->msg_type)
        {
        case SERVICE_CALLBACK_TYPE_INDIFICATION_NOTIFICATION:
        {
            if(pCscCallbackData->msg_data.notification_indification_index == CSC_NOTIFY_INDICATE_MEASUREMENT_ENABLE)
            {
            }
            else
            if(pCscCallbackData->msg_data.notification_indification_index == CSC_NOTIFY_INDICATE_MEASUREMENT_DISABLE)
            {
            }
            else
            if(pCscCallbackData->msg_data.notification_indification_index == CSC_NOTIFY_INDICATE_SC_CP_ENABLE)
            {
            }
            else
            if(pCscCallbackData->msg_data.notification_indification_index == CSC_NOTIFY_INDICATE_SC_CP_DISABLE)
            {
            }    
        }
            break;
        case SERVICE_CALLBACK_TYPE_READ_CHAR_VALUE:
        {
            if(pCscCallbackData->msg_data.read_value_index == CSC_READ_CSC_FEATURE)
            {
                uint16_t csc_feature = CSC_ALL_FEATURE_SUPPORTED;
                CSC_SetParameter(CSC_PARAM_CSC_FEATURE, 2, &csc_feature);
            }
            else
            if(pCscCallbackData->msg_data.read_value_index == CSC_READ_SENSOR_LOCATION)
            {
                CSC_SetParameter(CSC_PARAM_SENSOR_LOC, 1, &gCsc_SensorLocation);
            }
        }
            break;
        case SERVICE_CALLBACK_TYPE_WRITE_CHAR_VALUE:
        {
            if(pCscCallbackData->msg_data.write.opcode == CSC_CP_OPCODE_SET_CUMULATIVE)
            {
                gCsc_CulValue = pCscCallbackData->msg_data.write.cp_parameter.cumulative_value;
            }
            else
            if(pCscCallbackData->msg_data.write.opcode == CSC_CP_OPCODE_UPDATE_SENS_LOC)
            {
                gCsc_SensorLocation = pCscCallbackData->msg_data.write.cp_parameter.sensor_location_value;
            }
            else
            if(pCscCallbackData->msg_data.write.opcode == CSC_CP_OPCODE_REQ_SENS_LOC_LIST)
            {

            }
        }
        break;
        }
    }
#endif

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

