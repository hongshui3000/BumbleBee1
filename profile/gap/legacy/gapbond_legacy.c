enum { __FILE_NUM__ = 0 };

/**
*****************************************************************************************
*     Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
  * @file    gapbond_legacy.c
  * @brief   This file provides legacy GAP bond and pairing related functions.
  * @details
  * @author  kyle_xu
  * @date    2015-11-20
  * @version v0.1
  ******************************************************************************
  * @attention
  * <h2><center>&copy; COPYRIGHT 2015 Realtek Semiconductor Corporation</center></h2>
  ******************************************************************************
  */

/** Add Includes here **/
#include <rtl_types.h>
#include <blueapi.h>
#include <gap.h>
#include <string.h>
#include <trace.h>
#include <gapbond_common.h>
#include <gapbond_legacy.h>

static uint8_t GapBondBTMode = GAPBOND_BTMODE_21ENABLED;
static uint8_t GapBondOOBR[KEYLEN] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t GapBondOOBC[KEYLEN] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

extern uint16_t gapBond_ConnHandle;
extern uint8_t  gapBond_ConnectedDevAddr[B_ADDR_LEN];

/**
 * @brief      initialize parameters of legacy gap bond manager
 * @param   void
 * @return   void
 */
void GAPBondlegacy_ParaInit(void)
{
    GAPBondCom_ParaInit();
    GapBondBTMode = GAPBOND_BTMODE_21ENABLED;
}

/**
 * @brief           set a legacy GAP Bond Manager parameter
 *
 *                      NOTE: You can call this function with a GAP Bond Parameter ID and it will set the
 *                      GAP Bond Parameter.  GAP Bond Parameters are defined in (gapbond_common.h).  Also,
 *                      the "len" field must be set to the size of a "uint16_t" and the
 *                      "pvalue" field must point to a "uint16".
 *
 * @param         param - Profile parameter ID: @ref GAPBOND_MANAGER_PARAMETERS
 * @param         len - length of data to write
 * @param         pvalue - pointer to data to write.  This is dependent on
 *                      the parameter ID and WILL be cast to the appropriate
 *                      data type (example: data type of uint16 will be cast to
 *                      uint16 pointer).
 *
 * @return          gapAPI_CauseSuccess or gapAPI_InvalidPara (invalid paramID)
 */

TGAP_STATUS GAPBondlegacy_SetParameter(uint16_t param, uint8_t len, void *pvalue)
{
    TGAP_STATUS ret = gapAPI_CauseSuccess;

    switch (param)
    {
    case GAPBOND_BTMODE:
        if ((len == sizeof(uint8_t)) && (*((uint8_t*)pvalue) <= GAPBOND_BTMODE_21ENABLED))
        {
            GapBondBTMode = *((uint8_t*)pvalue);
        }
        else
        {
            ret = gapAPI_InvalidRange;
        }
        break;

    case GAPBOND_OOB_DATA_R:
        if (len == KEYLEN)
        {
            memcpy(GapBondOOBR, pvalue, KEYLEN);
        }
        else
        {
            ret = gapAPI_InvalidRange;
        }
        break;

    case GAPBOND_OOB_DATA_C:
        if (len == KEYLEN)
        {
            memcpy(GapBondOOBC, pvalue, KEYLEN);
        }
        else
        {
            ret = gapAPI_InvalidRange;
        }
        break;

    case GAPBOND_PAIRING_MODE:
    case GAPBOND_MITM_PROTECTION:
    case GAPBOND_IO_CAPABILITIES:
    case GAPBOND_OOB_ENABLED:
    case GAPBOND_PASSKEY:
         ret = GAPBondCom_SetParameter(param, len, pvalue);
         break;

    default:
        ret = gapAPI_InvalidPara;
        break;
    }

    return ret;
}


/**
 * @brief           get a legacy GAP Bond Manager Parameter.
 *
 *                      NOTE: You can call this function with a GAP Bond Manager Parameter ID and it will get a
 *                      GAP Bond Manager Parameter.  GAP Bond Manager  Parameters are defined in (gapbond_common.h).  Also, the
 *                      "pvalue" field must point to a "uint16".
 *
 * @param         param - Profile parameter ID: @ref GAPBOND_MANAGER_PARAMETERS
 * @param         pvalue - pointer to location to get the value.  This is dependent on
 *                      the parameter ID and WILL be cast to the appropriate
 *                      data type (example: data type of uint16 will be cast to
 *                      uint16 pointer).
 *
 * @return          gapAPI_CauseSuccess or gapAPI_InvalidPara (invalid paramID)
 */
TGAP_STATUS GAPBondlegacy_GetParameter(uint16_t param, void *pvalue)
{
    TGAP_STATUS ret = gapAPI_CauseSuccess;

    switch (param)
    {
    case GAPBOND_BTMODE:
        *((uint8_t*)pvalue) = GapBondBTMode;
        break;

    case GAPBOND_OOB_DATA_R:
        memcpy(pvalue, GapBondOOBR, KEYLEN);
        break;

    case GAPBOND_OOB_DATA_C:
        memcpy(pvalue, GapBondOOBC, KEYLEN);
        break;

    case GAPBOND_PAIRING_MODE:
    case GAPBOND_MITM_PROTECTION:
    case GAPBOND_IO_CAPABILITIES:
    case GAPBOND_OOB_ENABLED:
    case GAPBOND_PASSKEY:
        ret = GAPBondCom_GetParameter(param, pvalue);
        break;

    default:
        ret = gapAPI_InvalidPara;
        break;
    }

    return ret;
}

/**
 * @brief      set Pairable mode of device.
 * @param   void
 * @return    void
 */
void GAPBondlegacy_SetPairable(void)
{
    uint8_t pairing_mode = GAPBOND_PAIRING_MODE_PAIRABLE;
    uint8_t mitm = GAPBOND_AUTH_NO_MITM_YES_BOND;
    uint8_t iocapabilites = GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT;
    uint8_t oob_flag = FALSE;

    GAPBondCom_GetParameter(GAPBOND_PAIRING_MODE, &pairing_mode);
    GAPBondCom_GetParameter(GAPBOND_MITM_PROTECTION, &mitm);
    GAPBondCom_GetParameter(GAPBOND_IO_CAPABILITIES, &iocapabilites);
    GAPBondCom_GetParameter(GAPBOND_OOB_ENABLED, &oob_flag);
    
    blueAPI_ExtendPairableModeSetReq(pairing_mode,
                                    (TBlueAPI_BluetoothMode)GapBondBTMode,
                                    (TBlueAPI_AuthRequirements)mitm,
                                    (TBlueAPI_IOCapabilities)iocapabilites,
                                    oob_flag
                                    );
}

/**
 * @brief      start pairing
 * @param
 * @return     void
 */
void GAPBondlegacy_Pair(uint8_t *remote_bd)
{
    blueAPI_AuthReq(remote_bd);
}

/**
 * @brief      send passkey to gap bond manager when pairing with passkey entry,
 *                and local should input passkey.       
 * @param   void
 * @return   void
 */
void GAPBondlegacy_InputPassKey(void)
{
    GAPBondCom_InputPassKey();
}

/**
 * @brief      send legacy oob data to gap bond manager when pairing with out of bond,
 *                and local should input oob data.       
 * @param   void
 * @return   void
 */
void GAPBondlegacy_InputOOBData(void)
{
    blueAPI_LegacyRemoteOOBDataReqConf(gapBond_ConnectedDevAddr, GapBondOOBC, GapBondOOBR, blueAPI_CauseAccept);
}

/**
 * @brief      send authorization result to upper stack
 * @param    remote_bd   remote bd adress
 * @param    result   authorization result
 * @return    void
 */
void GAPBondlegacy_Authorize(uint8_t *remote_bd, TBlueAPI_Cause result)
{
    blueAPI_UserAuthorizationReqConf(remote_bd, result);
}

/**
 * @brief      get bonded device from flash storage, if bonded device exists, 
 *                return true, and device information will be returned by pBondedDevice  
 *                and local should input oob data.       
 * @param    pBondedDevice  - pointer to bonded device information 
 *
 * @return  bool
 */
bool GAPBondlegacy_GetBondedDevice(remote_BD_struct *pbonded_device)
{
    ///TODO: add get bond device after decide save struct
	return true;
}

/**
 * @brief      erase bonding device inforamtion from falsh storage
 * @param   void
 * @return   void
 */
void GAPBondlegacy_EraseAllBondings(void)
{
    ///TODO: add erase bond device after decide save struct
}

/**
 * @brief      send oob input message to app task.   
 * @param    handle   handle of connection
 * @return    void
 */
void GAPBondlegacy_OOBInputCB(uint16_t handle)
{
    BEE_IO_MSG bee_io_msg;
    BT_STACK_MSG bt_gap_msg;

#ifdef GAP_DEBUG_FLAG
    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "GAPBondlegacy_OOBInputCB", 0);
#endif

    bee_io_msg.IoType = BT_STATUS_UPDATE;
    bee_io_msg.subType = BT_MSG_TYPE_BOND_LEGACY_OOB_INPUT;
    bt_gap_msg.msgData.gapBondOobInput.connHandle = handle;

    memcpy(&bee_io_msg.parm, &bt_gap_msg, sizeof(bee_io_msg.parm));

    GAP_SendBtMsgToApp(&bee_io_msg);
}

/**
 * @brief      process blueAPI_EventConnectMDLInfo message from bt stack
 * @param    pinfo  - pointer to TBlueAPI_ConnectMDLInfo message
 * @return    void
 */
void gapbondlegacy_HandleConnectMDLInfo(PBlueAPI_ConnectMDLInfo pinfo)
{
    gapBond_ConnHandle = pinfo->local_MDL_ID;
}

/**
 * @brief      process blueAPI_EventAuthResultInd message from bt stack
 * @param    pind    pointer to blueAPI_AuthResultInd message
 * @return     void
 */
bool gapbondlegacy_HandleAuthResultInd(PBlueAPI_AuthResultInd pind)
{
    TBlueAPI_Cause cause = blueAPI_CauseSuccess;

#ifdef GAP_DEBUG_FLAG
    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO,"gapbondmgr_Handle_AuthResultInd: cause=0x%x",
               1, pind->cause);
#endif

    switch (pind->keyType)
    {
    case blueAPI_LinkKeyTypeCombination:
        break;

    case blueAPI_LinkKeyTypeUnauthenticated:
        break;

    case blueAPI_LinkKeyTypeAuthenticated:
        break;

    case blueAPI_LinkKeyTypeRequestBR:
        break;

    case blueAPI_LinkKeyTypeDeleted:
        break;

    default:
        break;
    }
#ifdef GAP_DEBUG_FLAG
    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "store data into flash", 0);
#endif

    blueAPI_AuthResultConf(pind->remote_BD,
                           (TBlueAPI_RemoteBDType)pind->remote_BD_Type,
                           cause);
    return true;
}

/**
 * @brief      process blueAPI_EventAuthResultInd message from bt stack
 * @param    pind   pointer to blueAPI_AuthResultRequestInd message
 * @return     bool
 */
bool gapbondlegacy_HandleAuthResultRequestInd(PBlueAPI_AuthResultRequestInd pind)
{
    //uint32_t errorno = 0;
    bool bIndError = false;

#ifdef GAP_DEBUG_FLAG
    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO,
               "gapbondlegacy_HandleAuthResultRequestInd: bd=0x%x 0x%x 0x%x 0x%x 0x%x 0x%x bdtype = 0x%x, keytype=0x%x \n", 8,
               pind->remote_BD[0], pind->remote_BD[1], pind->remote_BD[2], pind->remote_BD[3],
               pind->remote_BD[4], pind->remote_BD[5], pind->remote_BD_Type, pind->keyType);
#endif
    // begin - load from flash
#if 0
    remote_BD_struct s_remote_BD;
    errorno = fs_load_remote_BD_struct(&s_remote_BD);

    uint8_t remote_BD_equal = memcmp(pAuthResultRequestInd->remote_BD, s_remote_BD.addr, 6);

    //Fix IOT issue with IOS
    if (errorno != 0 || pAuthResultRequestInd->remote_BD_Type != s_remote_BD.remote_bd_type )
    {
        blueAPI_AuthResultRequestConf(pAuthResultRequestInd->remote_BD,
                                      pAuthResultRequestInd->remote_BD_Type,
                                      0,
                                      NULL,
                                      pAuthResultRequestInd->keyType,
                                      pAuthResultRequestInd->restartHandle,
                                      blueAPI_CauseReject);
        bIndError = true;
        return true;
    }
#endif

    static LTK_struct s_tmp; // for return s_tmp.linkKey
    switch (pind->keyType)
    {
    case blueAPI_LinkKeyTypeCombination:
        break;

    case blueAPI_LinkKeyTypeUnauthenticated:
        break;

    case blueAPI_LinkKeyTypeAuthenticated:
        break;

    case blueAPI_LinkKeyTypeRequestBR:
        break;

    case blueAPI_LinkKeyTypeDeleted:
        break;

    default:
        break;
    }

    if (bIndError == true)
    {
        return true;
    }

#ifdef GAP_DEBUG_FLAG
    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "linkKeyLength=0x%x \n", 1, s_tmp.linkKeyLength);
#endif

    UINT8 i = 0;
    for (i = 0; i < 28; i++)
    {
#ifdef GAP_DEBUG_FLAG
        DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "key=0x%x \n", 1, s_tmp.linkKey[i]);
#endif
    }
#ifdef GAP_DEBUG_FLAG
    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "restartHandle=0x%x \n", 1, pind->restartHandle);
#endif
    blueAPI_AuthResultRequestConf(pind->remote_BD,
                                  pind->remote_BD_Type,
                                  s_tmp.linkKeyLength,
                                  s_tmp.linkKey,
                                  pind->keyType,
                                  pind->restartHandle,
                                  blueAPI_CauseAccept
                                 );

    return true;
}

/**
 * @brief      process blueAPI_EventPairableModeSetRsp message from bt stack
 * @param    pPairableModeSetRsp   pointer to TBlueAPI_PairableModeSetRsp message
 * @return     bool
 */
bool gapbondlegacy_HandlePairableModeSetRsp(PBlueAPI_PairableModeSetRsp prsp)
{
#ifdef GAP_DEBUG_FLAG
    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "gapbondlegacy_HandlePairableModeSetRsp: cause=0x%x.", 1, prsp->cause);
#endif
    return true;
}

/**
 * @brief      process blueAPI_EventLegacyRemoteOOBDataReqInd message from bt stack
 * @param    pind  pointer to TBlueAPI_RemoteOOBDataReqInd message
 * @return     bool
 */
bool gapbondlegacy_HandleLegacyRemoteOOBDataReqInd(PBlueAPI_LegacyRemoteOOBDataReqInd pind)
{
#ifdef GAP_DEBUG_FLAG
    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "gapbondmgr_Handle_RemoteOOBDataReqInd ",0);
#endif
    //notify app to input oob data
    GAPBondlegacy_OOBInputCB(gapBond_ConnHandle);

    return true;
}

/**
 * @brief      process blueAPI_EventUserPasskeyReqReplyRsp message from bt stack
 * @param    prsp   pointer to TBlueAPI_UserPasskeyReqReplyRsp message
 * @return     bool
 */
bool gapbondlegacy_HandleUserPasskeyReqReplyRsp(PBlueAPI_UserPasskeyReqReplyRsp prsp)
{
#ifdef GAP_DEBUG_FLAG
    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "gapbondlegacy_HandleUserPasskeyReqReplyRsp cause=0x%x.",1, prsp->cause);
#endif
    return true;
}

/**
 * @brief      process blueAPI_AuthRsp message from bt stack
 * @param    prsp   pointer to blueAPI_AuthRsp message
 * @return    bool
 */
bool gapbondlegacy_HandleAuthRsp(PBlueAPI_AuthRsp prsp)
{
#ifdef GAP_DEBUG_FLAG
    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "gapbondlegacy_HandleUserPasskeyReqReplyRsp cause=0x%x.", 1, prsp->cause);
#endif
    return true;
}

/**
 * @brief      process blueAPI_UserAuthRequestInd message from bt stack
 * @param    pind   pointer to blueAPI_UserAuthRequestInd message
 * @return    bool
 */
bool gapbondlegacy_HandleUserAuthRequestInd(PBlueAPI_UserAuthRequestInd pind)
{
    uint8_t length = 16;
    uint8_t link_key[16] = {0};
    ///TODO: get link key stored
    blueAPI_UserAuthRequestConf(pind->remote_BD, length, link_key, blueAPI_CauseAccept);
    return true;
}

/**
 * @brief      process blueAPI_UserConfirmationReqInd message from bt stack
 * @param    pind   pointer to blueAPI_UserConfirmationReqInd message
 * @return    bool
 */
bool gapbondlegacy_HandleUserConfirmationReqInd(PBlueAPI_UserConfirmationReqInd pind)
{
    GAPBondCom_SetParameter(GAPBOND_PASSKEY, sizeof(uint32_t), &(pind->displayValue));

    GAPBondCom_PasskeyEntryDisplayCB(gapBond_ConnHandle, pind->displayValue);

    blueAPI_UserConfirmationReqConf(pind->remote_BD, blueAPI_CauseAccept);
    return true;
}

/**
 * @brief      process blueAPI_KeypressNotificationRsp message from bt stack
 * @param    prsp   pointer to blueAPI_KeypressNotificationRsp message
 * @return    bool
 */
bool gapbondlegacy_HandleKeypressNotificationRsp(PBlueAPI_KeypressNotificationRsp prsp)
{
#ifdef GAP_DEBUG_FLAG
    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "gapbondlegacy_HandleKeypressNotificationRsp cause=0x%x.", 1, prsp->cause);
#endif
    return true;
}

/**
 * @brief      process blueAPI_KeypressNotificationInfo message from bt stack
 * @param    pinfo   pointer to blueAPI_KeypressNotificationInfo message
 * @return    bool
 */
bool gapbondlegacy_HandleKeypressNotificationInfo(PBlueAPI_KeypressNotificationInfo pinfo)
{
#ifdef GAP_DEBUG_FLAG
    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "gapbondlegacy_HandleKeypressNotificationInfo eventType=0x%x.", 1, pinfo->eventType);
#endif
    return true;
}

/**
 * @brief      process blueAPI_LocalOOBDataRsp message from bt stack
 * @param    prsp   pointer to blueAPI_LocalOOBDataRsp message
 * @return    bool
 */
bool gapbondlegacy_HandleLocalOOBDataRsp(PBlueAPI_LocalOOBDataRsp prsp)
{
    if (prsp->cause == blueAPI_CauseSuccess)
    {
        memcpy(GapBondOOBC, prsp->C, KEYLEN);
        memcpy(GapBondOOBR, prsp->R, KEYLEN);
    }
    return true;
}

/**
 * @brief      process blueAPI_AuthDeleteRsp message from bt stack
 * @param    prsp   pointer to blueAPI_AuthDeleteRsp message
 * @return    bool
 */
bool gapbondlegacy_HandleAuthDeleteRsp(PBlueAPI_AuthDeleteRsp prsp)
{
#ifdef GAP_DEBUG_FLAG
    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "gapbondlegacy_HandleAuthDeleteRsp cause=0x%x.", 1, prsp->cause);
#endif
    return true;
}

/**
 * @brief      process blueAPI_AuthListInfo message from bt stack
 * @param    pinfo   pointer to blueAPI_AuthListInfo message
 * @return    bool
 */
bool gapbondlegacy_HandleAuthListInfo(PBlueAPI_AuthListInfo pinfo)
{
    return true;
}

/**
 * @brief      process blueAPI_AuthListRsp message from bt stack
 * @param    prsp   pointer to blueAPI_AuthListRsp message
 * @return    bool
 */
bool gapbondlegacy_HandleAuthListRsp(PBlueAPI_AuthListRsp prsp)
{
#ifdef GAP_DEBUG_FLAG
    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "gapbondlegacy_HandleAuthListRsp cause=0x%x.", 1, prsp->cause);
#endif
    return true;
}

/**
 * @brief      process blueapi upstream messages here
 * @param    pmsg   pointer to type TBlueAPI_UsMessage message
 * @return     bool
 */
bool GAPBondlegacy_ProcessEvent(PBlueAPI_UsMessage pmsg)
{
    switch (pmsg->Command)
    {
    default:
#ifdef GAP_DEBUG_FLAG
        DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "GAPBondMgr_Not_ProcessEvent: Command(0x%02x)", 1, pmsg->Command);
#endif
        break;

    case blueAPI_EventConnectMDLInfo:
        gapbondlegacy_HandleConnectMDLInfo(&pmsg->p.ConnectMDLInfo);
        break;

    case blueAPI_EventACLStatusInfo:
        GAPBondCom_HandleACLStatusInfo(&pmsg->p.ACLStatusInfo);
        break;

    case blueAPI_EventAuthResultInd:
        gapbondlegacy_HandleAuthResultInd(&pmsg->p.AuthResultInd);
        break;

    case blueAPI_EventAuthResultRequestInd:
        gapbondlegacy_HandleAuthResultRequestInd(&pmsg->p.AuthResultRequestInd);
        break;

    case blueAPI_EventPairableModeSetRsp:
        gapbondlegacy_HandlePairableModeSetRsp(&pmsg->p.PairableModeSetRsp);
        break;

    case blueAPI_EventUserPasskeyReqInd:
        GAPBondCom_HandleUserPasskeyReqInd(&pmsg->p.UserPasskeyReqInd);
        break;

    case blueAPI_EventLegacyRemoteOOBDataReqInd:
        gapbondlegacy_HandleLegacyRemoteOOBDataReqInd(&pmsg->p.LegacyRemoteOOBDataReqInd);
        break;

    case blueAPI_EventUserPasskeyNotificationInfo:
        GAPBondCom_HandleUserPasskeyNotificationInfo(&pmsg->p.UserPasskeyNotificationInfo);
        break;

    case blueAPI_EventUserPasskeyReqReplyRsp:
        gapbondlegacy_HandleUserPasskeyReqReplyRsp(&pmsg->p.UserPasskeyReqReplyRsp);
        break;

    case blueAPI_EventAuthRsp:
        gapbondlegacy_HandleAuthRsp(&pmsg->p.AuthRsp);
        break;

    case blueAPI_EventUserAuthRequestInd:
        gapbondlegacy_HandleUserAuthRequestInd(&pmsg->p.UserAuthRequestInd);
        break;

    case blueAPI_EventUserConfirmationReqInd:
        gapbondlegacy_HandleUserConfirmationReqInd(&pmsg->p.UserConfirmationReqInd);
        break;

    case blueAPI_EventKeypressNotificationRsp:
        gapbondlegacy_HandleKeypressNotificationRsp(&pmsg->p.KeypressNotificationRsp);
        break;

    case blueAPI_EventKeypressNotificationInfo:
        gapbondlegacy_HandleKeypressNotificationInfo(&pmsg->p.KeypressNotificationInfo);
        break;

    case blueAPI_EventLocalOOBDataRsp:
        gapbondlegacy_HandleLocalOOBDataRsp(&pmsg->p.LocalOOBDataRsp);
        break;

    case blueAPI_EventAuthDeleteRsp:
        gapbondlegacy_HandleAuthDeleteRsp(&pmsg->p.AuthDeleteRsp);
        break;

    case blueAPI_EventAuthListInfo:
        gapbondlegacy_HandleAuthListInfo(&pmsg->p.AuthListInfo);
        break;

    case blueAPI_EventAuthListRsp:
        gapbondlegacy_HandleAuthListRsp(&pmsg->p.AuthListRsp);
        break;
    }
    return true;
}

