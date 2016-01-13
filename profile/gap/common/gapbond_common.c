enum { __FILE_NUM__ = 0 };
/**
 ****************************************************************************** 
 *     Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
 ****************************************************************************** 
 * @file    gapbond_common.c
 * @brief   This file provides common GAP bond and pairing related functions.
 * @details
 * @author  kyle_xu
 * @date    2015-11-5
 * @version v0.1
 ******************************************************************************
 * @attention
 * <h2><center>&copy; COPYRIGHT 2015 Realtek Semiconductor Corporation</center></h2>
 ******************************************************************************
 */

#include <rtl_types.h>
#include <blueapi.h>
#include <gap.h>
#include <string.h>
#include <gapbond_common.h>
#include <trace.h>

static uint8_t gapBond_PairingMode = GAPBOND_PAIRING_MODE_PAIRABLE;
static uint8_t gapBond_MITM = GAPBOND_AUTH_NO_MITM_YES_BOND;
static uint8_t gapBond_IOCap = GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT;
static uint8_t gapBond_OOBDataFlag = FALSE;
static uint32_t gapBond_Passcode = 0;
uint16_t gapBond_ConnHandle = 0;
uint8_t  gapBond_ConnectedDevAddr[B_ADDR_LEN] = {0};

/**
 * @brief  initialize common parameters of gap bond manager
 * @param  none
 * @return  none
 * @retval   void
 */
void GAPBondCom_ParaInit(void)
{
    gapBond_PairingMode = GAPBOND_PAIRING_MODE_PAIRABLE;
    gapBond_MITM = GAPBOND_AUTH_NO_MITM_YES_BOND;
    gapBond_IOCap = GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT;
    gapBond_OOBDataFlag = FALSE;
    gapBond_Passcode = 0;
    gapBond_ConnHandle = 0;
}

/**
 * @brief           set a GAP Bond Manager parameter.
 *
 *                      NOTE: You can call this function with a GAP Bond Parameter ID and it will set the
 *                      GAP Bond Parameter.  GAP Bond Parameters are defined in (gapbond_common.h).  Also,
 *                      the "len" field must be set to the size of a "uint16_t" and the
 *                      "pValue" field must point to a "uint16".
 *
 * @param         param - Profile parameter ID: @ref GAPBOND_MANAGER_PARAMETERS
 * @param         len - length of data to write
 * @param         pvalue - pointer to data to write.  This is dependent on
 *                      the parameter ID and will be cast to the appropriate
 *                      data type (example: data type of uint16 will be cast to
 *                      uint16 pointer).
 *
 * @return          gapAPI_CauseSuccess or gapAPI_InvalidPara (invalid paramID) or gapAPI_InvalidRange
 */
TGAP_STATUS GAPBondCom_SetParameter(uint16_t param, uint8_t len, void *pvalue)
{
    TGAP_STATUS ret = gapAPI_CauseSuccess;

    switch (param)
    {
    case GAPBOND_PAIRING_MODE:
        if ((len == sizeof(uint8_t)) && (*((uint8_t*)pvalue) <= GAPBOND_PAIRING_MODE_PAIRABLE))
        {
            gapBond_PairingMode = *((uint8_t*)pvalue);
        }
        else
        {
            ret = gapAPI_InvalidRange;
        }
        break;

    case GAPBOND_MITM_PROTECTION:
        if ((len == sizeof(uint8_t)) && (*((uint8_t*)pvalue) <= GAPBOND_AUTH_YES_MITM_YES_BOND))
        {
            gapBond_MITM = *((uint8_t*)pvalue);
        }
        else
        {
            ret = gapAPI_InvalidRange;
        }
        break;

    case GAPBOND_IO_CAPABILITIES:
        if ((len == sizeof(uint8_t)) && (*((uint8_t*)pvalue) <= GAPBOND_IO_CAP_KEYBOARD_DISPLAY) )
        {
            gapBond_IOCap = *((uint8_t*)pvalue);
        }
        else
        {
            ret = gapAPI_InvalidRange;
        }
        break;

    case GAPBOND_OOB_ENABLED:
        if ((len == sizeof(uint8_t)) && (*((uint8_t*)pvalue) <= TRUE))
        {
            gapBond_OOBDataFlag = *((uint8_t*)pvalue);
        }
        else
        {
            ret = gapAPI_InvalidRange;
        }
        break;

    case GAPBOND_PASSKEY:
        if ((len == sizeof(uint32_t)) && (*((uint32_t*)pvalue) <= GAP_PASSCODE_MAX))
        {
            gapBond_Passcode = *((uint32_t*)pvalue);
        }
        else
        {
            ret = gapAPI_InvalidRange;
        }
        break;

    default:
        ret = gapAPI_InvalidPara;
        break;
    }

    return ( ret );
}

/**
 * @brief           get a GAP Bond Manager Parameter.
 *
 *                      NOTE: You can call this function with a GAP Bond Manager Parameter ID and it will get a
 *                      GAP Bond Manager Parameter.  GAP Bond Manager  Parameters are defined in (gapbond_common.h). 
 *                       Also, the "pValue" field must point to a "uint16".
 *
 * @param         param - Profile parameter ID: @ref GAPBOND_MANAGER_PARAMETERS
 * @param         pValue - pointer to location to get the value.  This is dependent on
 *                      the parameter ID and WILL be cast to the appropriate
 *                      data type (example: data type of uint16 will be cast to
 *                      uint16 pointer).
 *
 * @return          gapAPI_CauseSuccess or gapAPI_InvalidPara (invalid paramID)
 */

TGAP_STATUS GAPBondCom_GetParameter(uint16_t param, void *pvalue)
{
    TGAP_STATUS ret = gapAPI_CauseSuccess;

    switch (param)
    {
    case GAPBOND_PAIRING_MODE:
        *((uint8_t*)pvalue) = gapBond_PairingMode;
        break;

    case GAPBOND_MITM_PROTECTION:
        *((uint8_t*)pvalue) = gapBond_MITM;
        break;

    case GAPBOND_IO_CAPABILITIES:
        *((uint8_t*)pvalue) = gapBond_IOCap;
        break;

    case GAPBOND_OOB_ENABLED:
        *((uint8_t*)pvalue) = gapBond_OOBDataFlag;
        break;

    case GAPBOND_PASSKEY:
        *((uint32_t*)pvalue) = gapBond_Passcode;
        break;

    default:
        ret = gapAPI_InvalidPara;        
        break;
    }

    return ( ret );
}

/**
 * @brief      send passkey to gap bond manager
 * @param   void
 * @return   void
 */
void GAPBondCom_InputPassKey(void)
{
    blueAPI_UserPasskeyReqReplyReq(gapBond_ConnectedDevAddr, gapBond_Passcode, blueAPI_CauseAccept);
}

/**
 * @brief      send passkey display value message to app task
 * @param    handle - handle of connection
 * @param    display_value - value to be dispalyed by app
 * @return     void
 */
void GAPBondCom_PasskeyEntryDisplayCB(uint16_t handle, uint32_t display_value)
{
    BEE_IO_MSG bee_io_msg;
    BT_STACK_MSG bt_gap_msg;

#ifdef GAP_DEBUG_FLAG
    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "GAPBondCom_PasskeyEntryDisplayCB: %d", 1, display_value);
#endif

    bee_io_msg.IoType = BT_STATUS_UPDATE;
    bee_io_msg.subType = BT_MSG_TYPE_BOND_PASSKEY_DISPLAY;
    bt_gap_msg.msgData.gapBondPasskeyDisplay.connHandle = handle;

    memcpy(&bee_io_msg.parm, &bt_gap_msg, sizeof(bee_io_msg.parm));

    GAP_SendBtMsgToApp(&bee_io_msg);
}

/**
 * @brief      send passkey entry input  message to app task
 * @param    handle - handle of connection
 * @return    void
 */
void GAPBondCom_PasskeyEntryKeyboardInputCB(uint16_t handle)
{
    BEE_IO_MSG bee_io_msg;
    BT_STACK_MSG bt_gap_msg;

#ifdef GAP_DEBUG_FLAG
    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "GAPBondCom_PasskeyEntryKeyboardInputCB", 0);
#endif

    bee_io_msg.IoType = BT_STATUS_UPDATE;
    bee_io_msg.subType = BT_MSG_TYPE_BOND_PASSKEY_INPUT;
    bt_gap_msg.msgData.gapBondPasskeyInput.connHandle = handle;

    memcpy(&bee_io_msg.parm, &bt_gap_msg, sizeof(bee_io_msg.parm));

    GAP_SendBtMsgToApp(&bee_io_msg);
}

/**
 * @brief      send pair state change message to app task
 * @param    handle - handle of connection
 * @param    state - pairing state
 * @param    status - pair result status
 * @return     void
 */
void GAPBondCom_PairStateCB(uint16_t handle, uint8_t state, uint8_t status)
{
    BEE_IO_MSG bee_io_msg;
    BT_STACK_MSG bt_gap_msg;

#ifdef GAP_DEBUG_FLAG
    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "GAPBondCom_PairStateCB handle=0x%x, state = %d, status = %d", 3, handle, state, status);
#endif

    bee_io_msg.IoType = BT_STATUS_UPDATE;
    bee_io_msg.subType = BT_MSG_TYPE_BOND_STATE_CHANGE;
    bt_gap_msg.msgData.gapBondStateChange.connHandle = handle;
    bt_gap_msg.msgData.gapBondStateChange.newState = state;
    bt_gap_msg.msgData.gapBondStateChange.status = status;

    memcpy(&bee_io_msg.parm, &bt_gap_msg, sizeof(bee_io_msg.parm));

    GAP_SendBtMsgToApp(&bee_io_msg);
}

/**
 * @brief      send encryption state change message to app task
 * @param    handle - handle of connection
 * @param    state - encryption state
 * @return    void
 */
void GAPBondCom_EncryptStateCB(uint16_t handle, uint8_t state)
{
    BEE_IO_MSG bee_io_msg;
    BT_STACK_MSG bt_gap_msg;
#ifdef GAP_DEBUG_FLAG
    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "GAPBondCom_EncryptStateCB handle=0x%x, state = %d", 2, handle, state);
#endif

    bee_io_msg.IoType = BT_STATUS_UPDATE;
    bee_io_msg.subType = BT_MSG_TYPE_ENCRYPT_STATE_CHANGE;
    bt_gap_msg.msgData.gapEncryptStateChange.connHandle = handle;
    bt_gap_msg.msgData.gapEncryptStateChange.newState = state;

    memcpy(&bee_io_msg.parm, &bt_gap_msg, sizeof(bee_io_msg.parm));

    GAP_SendBtMsgToApp(&bee_io_msg);
}

/**
 * @brief      process blueAPI_EventACLStatusInfo message from bt stack
 * @param    pinfo  - pointer to TBlueAPI_ACLStatusInfo message
 * @return    bool
 */
bool GAPBondCom_HandleACLStatusInfo(PBlueAPI_ACLStatusInfo pinfo)
{
#ifdef GAP_DEBUG_FLAG
    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO,"GAPBondCom_HandleACLStatusInfo: status=%d", 1, pinfo->status);
#endif
    switch (pinfo->status)
    {
    case blueAPI_ACLConnectedActive:
        memcpy(gapBond_ConnectedDevAddr, pinfo->remote_BD, B_ADDR_LEN);
        break;

    case blueAPI_ACLAuthenticationStarted:
        GAPBondCom_PairStateCB(gapBond_ConnHandle, GAPBOND_PAIRING_STATE_STARTED, 0);
        break;

    case blueAPI_ACLAuthenticationFailure:
        GAPBondCom_PairStateCB(gapBond_ConnHandle, GAPBOND_PAIRING_STATE_COMPLETE, 1);
        break;

    case blueAPI_ACLAuthenticationSuccess:
        GAPBondCom_PairStateCB(gapBond_ConnHandle, GAPBOND_PAIRING_STATE_COMPLETE, 0);
        break;

    case blueAPI_ACLConnectionEncrypted:
        GAPBondCom_EncryptStateCB(gapBond_ConnHandle, GAPBOND_ENCRYPT_STATE_ENABLED);
        break;

    case blueAPI_ACLConnectionNotEncrypted:
        GAPBondCom_EncryptStateCB(gapBond_ConnHandle, GAPBOND_ENCRYPT_STATE_DISABLED);
        break;

    default:
        break;
    }

    return true;
}

/**
 * @brief      process blueAPI_EventUserPasskeyReqInd message from bt stack
 * @param    pind  - pointer to TBlueAPI_UserPasskeyReqInd message
 * @return     bool
 */
bool GAPBondCom_HandleUserPasskeyReqInd(PBlueAPI_UserPasskeyReqInd pind)
{
#ifdef GAP_DEBUG_FLAG
    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "GAPBondCom_HandleUserPasskeyReqInd.",0);
#endif
    blueAPI_UserPasskeyReqConf(pind->remote_BD, blueAPI_CauseAccept);

    //notify app to input passkey
    GAPBondCom_PasskeyEntryKeyboardInputCB(gapBond_ConnHandle);

    return true;
}

/**
 * @brief      process blueAPI_EventUserPasskeyNotificationInfo message from bt stack
 * @param    pinfo  - pointer to TBlueAPI_UserPasskeyNotificationInfo message
 * @return     bool
 */
bool GAPBondCom_HandleUserPasskeyNotificationInfo(PBlueAPI_UserPasskeyNotificationInfo pinfo)
{
#ifdef GAP_DEBUG_FLAG
    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "GAPBondCom_HandleUserPasskeyNotificationInfo diapVal=%d.",
                    1, pinfo->displayValue);
#endif
    //notify app to display passkey
    gapBond_Passcode = pinfo->displayValue;
    GAPBondCom_PasskeyEntryDisplayCB(gapBond_ConnHandle, pinfo->displayValue);

    return true;
}

