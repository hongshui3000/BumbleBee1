enum { __FILE_NUM__ = 0 };
/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file      test_profile_application.c
* @brief     test_profile application implementation
* @details   test_profile application implementation
* @author    ranhui
* @date      2015-03-27
* @version   v0.3
* *********************************************************************************************************
*/
#include "application.h"
//#include "test_peripheral_application.h"

#include "gapbondmgr.h"
#include "profile_api.h"
#include "peripheral_stack_api.h"
#include "trace.h"

#include "peripheral.h"
#include "central.h"
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

extern uint8_t  gapPara_profileRole;   /* GAP role. */
/** @brief  global variable, indicate current GAP role state. */
gaprole_States_t gapProfileState = GAPSTATE_INIT;

/********************************************************************************************************
* local static variables defined here, only used in this source file.
********************************************************************************************************/


void peripheral_HandleBtGapMessage(BEE_IO_MSG  *pBeeIoMsg);
void central_HandleBtGapMessage(BEE_IO_MSG *pBeeIoMsg);
void broadcaster_HandleBtGapMessage(BEE_IO_MSG *io_driver_msg_recv);
void observer_HandleBtGapMessage(BEE_IO_MSG *pBeeIoMsg);


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
        {
            if(gapPara_profileRole == GAP_PROFILE_PERIPHERAL)
            {
                peripheral_HandleBtGapMessage(&io_driver_msg_recv);
            }
            else if(gapPara_profileRole == GAP_PROFILE_CENTRAL)
            {
                central_HandleBtGapMessage(&io_driver_msg_recv);
            }
            else if(gapPara_profileRole == GAP_PROFILE_BROADCASTER)
            {
                broadcaster_HandleBtGapMessage(&io_driver_msg_recv);
            }
            else if(gapPara_profileRole == GAP_PROFILE_OBSERVER)
            {
                observer_HandleBtGapMessage(&io_driver_msg_recv);
            }
        }
        break;
    case IO_UART_MSG_TYPE:
        {
            //char RxChar = (char)io_driver_msg_recv.subType;
            //user_CmdCollect(&gUserCmdIF, &RxChar, sizeof(RxChar), userCmdTable);
        }
        break;
    default:
        break;
    }
}


void peripheral_HandleBtGapStateChangeEvt(uint8_t newState)
{

    switch ( newState )
    {
    case GAPSTATE_IDLE_NO_ADV_NO_CONN:
        {
            if (gapProfileState == GAPSTATE_CONNECTED)
            {
                uint8_t disc_reason;
                peripheralGetGapParameter(GAPPRRA_DISCONNECTED_REASON, &disc_reason);
                DBG_BUFFER(MODULE_APP, LEVEL_INFO, "peripheral_HandleBtGapStateChangeEvt: disc_reason = %d", 1, disc_reason);
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
        {
        }
        break;
    case GAPSTATE_CONNECTED:
        {
        }
        break;
    case GAPSTATE_CONNECTED_ADV:
        {
        }
        break;
    default:
        {
        }
        break;
    }

    gapProfileState = (gaprole_States_t)newState;
}

void peripheral_HandleBtGapBondStateChangeEvt(uint8_t newState)
{

    switch (newState)
    {
    case GAPBOND_PAIRING_STATE_STARTED:
        {
            DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BT_MSG_TYPE_BOND_STATE_CHANGE:(GAPBOND_PAIRING_STATE_STARTED)", 0);
        }
        break;

    case GAPBOND_PAIRING_STATE_COMPLETE:
        {
            DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BT_MSG_TYPE_BOND_STATE_CHANGE:(GAPBOND_PAIRING_STATE_COMPLETE)", 0);
        }
        break;

    case GAPBOND_PAIRING_STATE_BONDED:
        {
            DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BT_MSG_TYPE_BOND_STATE_CHANGE:(GAPBOND_PAIRING_STATE_BONDED)", 0);
        }
        break;

    default:
        {
            DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BT_MSG_TYPE_BOND_STATE_CHANGE:(unknown newstate: %d)", 1, newState);
        }
        break;
    }

}


void peripheral_HandleBtGapEncryptStateChangeEvt(uint8_t newState)
{
    switch (newState)
    {
    case GAPBOND_ENCRYPT_STATE_ENABLED:
        {
            DBG_BUFFER(MODULE_APP, LEVEL_INFO, "GAPBOND_ENCRYPT_STATE_ENABLED", 0);
        }
        break;

    case GAPBOND_ENCRYPT_STATE_DISABLED:
        {
            DBG_BUFFER(MODULE_APP, LEVEL_INFO, "GAPBOND_ENCRYPT_STATE_DISABLED", 0);
        }
        break;

    default:
        break;
    }
}


void peripheral_HandleBtGapConnParaChangeEvt(uint8_t status)
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
void peripheral_HandleBtGapMessage(BEE_IO_MSG  *pBeeIoMsg)
{
    BT_STACK_MSG BtStackMsg;
    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "peripheral_HandleBtGapMessage subType = %d", 1, pBeeIoMsg->subType);    
    memcpy(&BtStackMsg, &pBeeIoMsg->parm, sizeof(pBeeIoMsg->parm));

    switch (pBeeIoMsg->subType)
    {
    case BT_MSG_TYPE_CONN_STATE_CHANGE:
        {
            DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BT_MSG_TYPE_CONN_STATE_CHANGE:(%d->%d)",
                       2, gapProfileState, BtStackMsg.msgData.gapConnStateChange.newState);

            peripheral_HandleBtGapStateChangeEvt(BtStackMsg.msgData.gapConnStateChange.newState);
        }
        break;

    case BT_MSG_TYPE_BOND_STATE_CHANGE:
        {
            peripheral_HandleBtGapBondStateChangeEvt(BtStackMsg.msgData.gapBondStateChange.newState);
        }
        break;

    case BT_MSG_TYPE_BOND_PASSKEY_DISPLAY:
        {
            uint32_t displayValue = 0;
            peripheralGetGapParameter(GAPBOND_PASSKEY, &displayValue);
            DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BT_MSG_TYPE_BOND_PASSKEY_DISPLAY: %d", 1, displayValue);
        }
        break;

    case BT_MSG_TYPE_BOND_PASSKEY_INPUT:
        {
            uint32_t passKey = 888888;
            DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BT_MSG_TYPE_BOND_PASSKEY_INPUT", 0);

            peripheralSetGapParameter(GAPBOND_PASSKEY, sizeof(passKey), &passKey);
            GAPBondMgr_InputPassKey();
        }
        break;

    case BT_MSG_TYPE_BOND_OOB_INPUT:
        {
            uint8_t ooBData[KEYLEN] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
            DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BT_MSG_TYPE_BOND_OOB_INPUT", 0);

            peripheralSetGapParameter(GAPBOND_OOB_DATA, KEYLEN, ooBData);
            GAPBondMgr_InputOobData();
        }
        break;

    case BT_MSG_TYPE_ENCRYPT_STATE_CHANGE:
        {
            peripheral_HandleBtGapEncryptStateChangeEvt(BtStackMsg.msgData.gapEncryptStateChange.newState);
        }
        break;

    case BT_MSG_TYPE_CONN_PARA_UPDATE_CHANGE:
        {
            peripheral_HandleBtGapConnParaChangeEvt(BtStackMsg.msgData.gapConnParaUpdateChange.status);
        }
        break;

    default:
        DBG_BUFFER(MODULE_APP, LEVEL_ERROR, "peripheral_HandleBtGapMessage unknown subtype", 1, pBeeIoMsg->subType);
        break;

    }

}

void central_HandleBtGapStateChangeEvt(uint8_t newState)
{
    switch ( newState )
    {
      case GAPSTATE_IDLE_NO_SCAN_NO_CONN:
            {

            }

            break;
            
        case GAPSTATE_SCANNING:
            {
                
            }
            break;

        case GAPSTATE_CONNECTING:
            {

            }
            break;

        case GAPSTATE_CONNECTED:
            {
                
            }
            break;

        default:
            {

            }
            break;

    }

    gapProfileState = (gaprole_States_t)newState;
}

void central_HandleBtGapBondStateChangeEvt(uint8_t newState)
{

    switch(newState)
    {
        case GAPBOND_PAIRING_STATE_STARTED:
        {
            DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BT_MSG_TYPE_BOND_STATE_CHANGE:(GAPBOND_PAIRING_STATE_STARTED)", 0);
        }
        break;

        case GAPBOND_PAIRING_STATE_COMPLETE:
        {
            DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BT_MSG_TYPE_BOND_STATE_CHANGE:(GAPBOND_PAIRING_STATE_COMPLETE)", 0);
        }
        break;

        case GAPBOND_PAIRING_STATE_BONDED:
        {
            DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BT_MSG_TYPE_BOND_STATE_CHANGE:(GAPBOND_PAIRING_STATE_BONDED)", 0);
        }
        break;

        default:
        {
            DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BT_MSG_TYPE_BOND_STATE_CHANGE:(unknown newstate: %d)", 1, newState);
        }
        break;
    }

}


void central_HandleBtGapEncryptStateChangeEvt(uint8_t newState)
{
    switch(newState)
    {
        case GAPBOND_ENCRYPT_STATE_ENABLED:
        {
            DBG_BUFFER(MODULE_APP, LEVEL_INFO, "GAPBOND_ENCRYPT_STATE_ENABLED", 0);
        }
        break;

        case GAPBOND_ENCRYPT_STATE_DISABLED:
        {
            DBG_BUFFER(MODULE_APP, LEVEL_INFO, "GAPBOND_ENCRYPT_STATE_DISABLED", 0);
        }
        break;

        default:
        break;
    }
}


void central_HandleBtGapConnParaChangeEvt(uint8_t status)
{
    if(status == 0)
    {
        uint16_t con_interval;
        uint16_t conn_slave_latency;
        uint16_t conn_supervision_timeout;

        centralGetGapParameter(GAPPRRA_CONN_INTERVAL, &con_interval);
        centralGetGapParameter(GAPPRRA_CONN_LATENCY, &conn_slave_latency);
        centralGetGapParameter(GAPPRRA_CONN_TIMEOUT, &conn_supervision_timeout);

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
void central_HandleBtGapMessage(BEE_IO_MSG *pBeeIoMsg)
{
    BT_STACK_MSG BtStackMsg;
    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "peripheral_HandleBtGapMessage subType = %d", 1, pBeeIoMsg->subType);
    
    memcpy(&BtStackMsg, &pBeeIoMsg->parm, sizeof(pBeeIoMsg->parm));

    switch(pBeeIoMsg->subType)
    {
    case BT_MSG_TYPE_CONN_STATE_CHANGE:
    {
        DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BT_MSG_TYPE_CONN_STATE_CHANGE:(%d->%d)", 
                 2, gapProfileState, BtStackMsg.msgData.gapConnStateChange.newState);

        central_HandleBtGapStateChangeEvt(BtStackMsg.msgData.gapConnStateChange.newState);
    }
    break;

    case BT_MSG_TYPE_BOND_STATE_CHANGE:
    {
        central_HandleBtGapBondStateChangeEvt(BtStackMsg.msgData.gapBondStateChange.newState);
    }
    break;

    case BT_MSG_TYPE_BOND_PASSKEY_DISPLAY:
    {
        uint32_t displayValue = 0;
        GAPBondMgr_GetParameter(GAPBOND_PASSKEY, &displayValue);       
        DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BT_MSG_TYPE_BOND_PASSKEY_DISPLAY: %d", 1, displayValue);
    }
    break;

    case BT_MSG_TYPE_BOND_PASSKEY_INPUT:
    {
        uint32_t passKey = 888888;
        DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BT_MSG_TYPE_BOND_PASSKEY_INPUT", 0);

        GAPBondMgr_SetParameter(GAPBOND_PASSKEY, sizeof(passKey), &passKey);
        GAPBondMgr_InputPassKey();
    }
    break;

    case BT_MSG_TYPE_BOND_OOB_INPUT:
    {
        uint8_t ooBData[KEYLEN] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BT_MSG_TYPE_BOND_OOB_INPUT", 0);
        
        GAPBondMgr_SetParameter(GAPBOND_OOB_DATA, KEYLEN, ooBData);
        GAPBondMgr_InputOobData();
    }
    break;
    
    case BT_MSG_TYPE_ENCRYPT_STATE_CHANGE:
    {
       central_HandleBtGapEncryptStateChangeEvt(BtStackMsg.msgData.gapEncryptStateChange.newState);
    }
    break;

    case BT_MSG_TYPE_CONN_PARA_UPDATE_CHANGE:
    {
        central_HandleBtGapConnParaChangeEvt(BtStackMsg.msgData.gapConnParaUpdateChange.status);
    }
    break;
    
    default:
        DBG_BUFFER(MODULE_APP, LEVEL_ERROR, "peripheral_HandleBtGapMessage unknown subtype", 1, pBeeIoMsg->subType);
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
void broadcaster_HandleBtGapMessage(BEE_IO_MSG *pBeeIoMsg)
{
    BT_STACK_MSG BtStackMsg;
    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "broadcaster_HandleBtGapMessage subType = %d", 1, pBeeIoMsg->subType);
    
    memcpy(&BtStackMsg, &pBeeIoMsg->parm, sizeof(pBeeIoMsg->parm));

    switch(pBeeIoMsg->subType)
    {
    case BT_MSG_TYPE_CONN_STATE_CHANGE:
    {
        DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BT_MSG_TYPE_CONN_STATE_CHANGE:(%d->%d)", 
                  2, gapProfileState, BtStackMsg.msgData.gapConnStateChange.newState);

        switch ( BtStackMsg.msgData.gapConnStateChange.newState )
        {
        case GAPSTATE_STACK_READY:
            {
              
            }
            break;

        case GAPSTATE_ADVERTISING:
            {

            }
            break;
            
        case GAPSTATE_IDLE:
            {

            }
            break;    

        default:
            {

            }
            break;

        }

        gapProfileState = (gaprole_States_t)BtStackMsg.msgData.gapConnStateChange.newState;

    }
    break;


    
    default:
        DBG_BUFFER(MODULE_APP, LEVEL_ERROR, "broadcaster_HandleBtGapMessage unknown subtype", 1, pBeeIoMsg->subType);
    break;

    }

}


void observer_HandleBtGapStateChangeEvt(uint8_t newState)
{
    switch ( newState )
    {
      case GAPSTATE_IDLE_NO_SCAN_NO_CONN:
            {

            }

            break;
            
        case GAPSTATE_SCANNING:
            {
                gapProfileState = GAPSTATE_SCANNING;
            }
            break;

        default:
            {

            }
            break;

    }

    gapProfileState = (gaprole_States_t)newState;
}

void observer_HandleBtGapBondStateChangeEvt(uint8_t newState)
{

    switch(newState)
    {
        case GAPBOND_PAIRING_STATE_STARTED:
        {
            DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BT_MSG_TYPE_BOND_STATE_CHANGE:(GAPBOND_PAIRING_STATE_STARTED)", 0);
        }
        break;

        case GAPBOND_PAIRING_STATE_COMPLETE:
        {
            DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BT_MSG_TYPE_BOND_STATE_CHANGE:(GAPBOND_PAIRING_STATE_COMPLETE)", 0);
        }
        break;

        case GAPBOND_PAIRING_STATE_BONDED:
        {
            DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BT_MSG_TYPE_BOND_STATE_CHANGE:(GAPBOND_PAIRING_STATE_BONDED)", 0);
        }
        break;

        default:
        {
            DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BT_MSG_TYPE_BOND_STATE_CHANGE:(unknown newstate: %d)", 1, newState);
        }
        break;
    }

}


void observer_HandleBtGapEncryptStateChangeEvt(uint8_t newState)
{
    switch(newState)
    {
        case GAPBOND_ENCRYPT_STATE_ENABLED:
        {
            DBG_BUFFER(MODULE_APP, LEVEL_INFO, "GAPBOND_ENCRYPT_STATE_ENABLED", 0);
        }
        break;

        case GAPBOND_ENCRYPT_STATE_DISABLED:
        {
            DBG_BUFFER(MODULE_APP, LEVEL_INFO, "GAPBOND_ENCRYPT_STATE_DISABLED", 0);
        }
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
void observer_HandleBtGapMessage(BEE_IO_MSG *pBeeIoMsg)
{
    BT_STACK_MSG BtStackMsg;
    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "peripheral_HandleBtGapMessage subType = %d", 1, pBeeIoMsg->subType);
    
    memcpy(&BtStackMsg, &pBeeIoMsg->parm, sizeof(pBeeIoMsg->parm));

    switch(pBeeIoMsg->subType)
    {
    case BT_MSG_TYPE_CONN_STATE_CHANGE:
    {
        DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BT_MSG_TYPE_CONN_STATE_CHANGE:(%d->%d)", 
               2, gapProfileState, BtStackMsg.msgData.gapConnStateChange.newState);

        observer_HandleBtGapStateChangeEvt(BtStackMsg.msgData.gapConnStateChange.newState);
    }
    break;
    
    default:
        DBG_BUFFER(MODULE_APP, LEVEL_ERROR, "peripheral_HandleBtGapMessage unknown subtype", 1, pBeeIoMsg->subType);        
    break;

    }

}

TAppResult AppProfileCallback(uint8_t serviceID, void* pData)
{
    if (serviceID == ProfileAPI_ServiceUndefined)
    {
        TEventInfoCBs_t *pPara = (TEventInfoCBs_t *)pData;
        switch (pPara->eventId)
        {
        case PROFILE_EVT_SRV_REG_COMPLETE:// srv register result event.
            DBG_BUFFER(MODULE_APP, LEVEL_INFO, "profile callback PROFILE_EVT_SRV_REG_COMPLETE\n", 0);
            {
                testApp_StartAdv();
            }
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
    else if (serviceID == gDisServiceId)
    {
        TDIS_CALLBACK_DATA *pDisCallbackData = (TDIS_CALLBACK_DATA *)pData;
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
    else if (serviceID == gBasServiceId)
    {
        TBAS_CALLBACK_DATA *pBasCallbackData = (TBAS_CALLBACK_DATA *)pData;
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
    else if (serviceID == gRscServiceId)
    {
        TRSC_CALLBACK_DATA *pRscCallbackData = (TRSC_CALLBACK_DATA *)pData;
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
    else if (serviceID == gCscServiceId)
    {
        TCSC_CALLBACK_DATA *pCscCallbackData = (TCSC_CALLBACK_DATA *)pData;
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

