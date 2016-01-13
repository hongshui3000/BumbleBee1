enum { __FILE_NUM__ = 0 };

/**
*****************************************************************************************
*     Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
  * @file    gapbondmgr.c
  * @brief   This file provides all the GAP bond and pairing related functions.
  * @details
  * @author  ranhui
  * @date    11-5-2015
  * @version v1.0
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
#include <gapbondmgr.h>
#include <trace.h>
#include <flash_storage.h>

// GAPBonding Parameters

static uint8_t gapBond_SecReqEnable = FALSE;
static uint8_t gapBond_SecReqRequirements = GAPBOND_SEC_REQ_NO_MITM;
static uint8_t gapBond_OOBData[KEYLEN] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t gapBond_FixedPasskeyEnable = FALSE;

extern uint16_t gapBond_ConnHandle;
extern uint8_t  gapBond_ConnectedDevAddr[B_ADDR_LEN];

/**
 * @fn          GAPBondMgr_ParaInit
 * @brief      Initialize parameters of gap bond manager.            
 *
 * @return     void
 */
void GAPBondMgr_ParaInit(void)
{
    GAPBondCom_ParaInit();
    gapBond_FixedPasskeyEnable = FALSE;
}

/**
 * @brief           Set a GAP Bond Manager parameter.
 *
 *                      NOTE: You can call this function with a GAP Bond Parameter ID and it will set the
 *                      GAP Bond Parameter.  GAP Bond Parameters are defined in (gapbondmanager.h).  Also,
 *                      the "len" field must be set to the size of a "uint16_t" and the
 *                      "pValue" field must point to a "uint16".
 *
 * @param[in]         param - Profile parameter ID: @ref GAPBOND_MANAGER_PARAMETERS
 * @param[in]         len - length of data to write
 * @param[in]         pValue - pointer to data to write.  This is dependent on
 *                      the parameter ID and WILL be cast to the appropriate
 *                      data type (example: data type of uint16 will be cast to
 *                      uint16 pointer).
 *
 * @return          gapAPI_CauseSuccess or INVALIDPARAMETER (invalid paramID)
 */
TGAP_STATUS GAPBondMgr_SetParameter( uint16_t param, uint8_t len, void *pValue )
{
    TGAP_STATUS ret = gapAPI_CauseSuccess;  // return value

    switch ( param )
    {
    case GAPBOND_PAIRING_MODE:
    case GAPBOND_MITM_PROTECTION:
    case GAPBOND_IO_CAPABILITIES:
    case GAPBOND_OOB_ENABLED:        
    case GAPBOND_PASSKEY:
        GAPBondCom_SetParameter(param, len, pValue);
        break;

    case GAPBOND_OOB_DATA:
        if ( len == KEYLEN )
        {
            memcpy( gapBond_OOBData, pValue, KEYLEN ) ;
        }
        else
        {
            ret = gapAPI_InvalidRange;
        }
        break;

    case GAPBOND_FIXED_PASSKEY_ENABLE:
        if ( (len == sizeof ( uint8_t )) && (*((uint8_t*)pValue) <= TRUE) )
        {
            gapBond_FixedPasskeyEnable = *((uint8_t*)pValue);
        }
        else
        {
            ret = gapAPI_InvalidRange;
        }
        break;

    case GAPBOND_SEC_REQ_ENABLE:
        if ( (len == sizeof ( uint8_t )) && (*((uint8_t*)pValue) <= TRUE) )
        {
            gapBond_SecReqEnable = *((uint8_t*)pValue);
        }
        else
        {
            ret = gapAPI_InvalidRange;
        }
        break;
            
    case GAPBOND_SEC_REQ_REQUIREMENT:
        if ( (len == sizeof ( uint8_t )) && (*((uint8_t*)pValue) <= GAPBOND_SEC_REQ_YES_MITM) )
        {
            gapBond_SecReqRequirements = *((uint8_t*)pValue);
        }
        else
        {
            ret = gapAPI_InvalidRange;
        }
        break;

    default:
        ret = gapAPI_InvalidRange;
        break;
    }

    return ( ret );
}

/**
 * @brief           Get a GAP Bond Manager Parameter.
 *
 *                      NOTE: You can call this function with a GAP Bond Manager Parameter ID and it will get a
 *                      GAP Bond Manager Parameter.  GAP Bond Manager  Parameters are defined in (gapbondmanager.h).  Also, the
 *                      "pValue" field must point to a "uint16".
 *
 * @param[in]         param - Profile parameter ID: @ref GAPBOND_MANAGER_PARAMETERS
 * @param[out]         pValue - pointer to location to get the value.  This is dependent on
 *                      the parameter ID and WILL be cast to the appropriate
 *                      data type (example: data type of uint16 will be cast to
 *                      uint16 pointer).
 *
 * @return          gapAPI_CauseSuccess or INVALIDPARAMETER (invalid paramID)
 */

TGAP_STATUS GAPBondMgr_GetParameter( uint16_t param, void *pValue )
{
    TGAP_STATUS ret = gapAPI_CauseSuccess;  // return value

    switch (param)
    {
    case GAPBOND_PAIRING_MODE:
    case GAPBOND_MITM_PROTECTION:
    case GAPBOND_IO_CAPABILITIES:
    case GAPBOND_OOB_ENABLED:
    case GAPBOND_PASSKEY:
        GAPBondCom_GetParameter(param, pValue);
        break;

    case GAPBOND_OOB_DATA:
        memcpy(pValue, gapBond_OOBData, KEYLEN) ;
        break;

    case GAPBOND_FIXED_PASSKEY_ENABLE:
        *((uint32_t*)pValue) = gapBond_FixedPasskeyEnable;
        break;

    default:
        ret = gapAPI_InvalidRange;        
        break;
    }

    return ( ret );
}

/**
 * @fn          GAPBondMgr_SetPairable
 * @brief      Set Pairable mode of device.       
 *
 * @return     void
 */
void GAPBondMgr_SetPairable(void)
{
    uint8_t pairing_mode = GAPBOND_PAIRING_MODE_PAIRABLE;
    uint8_t mitm = GAPBOND_AUTH_NO_MITM_YES_BOND;
    uint8_t iocapabilites = GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT;
    uint8_t oob_flag = FALSE;

    GAPBondCom_GetParameter(GAPBOND_PAIRING_MODE, &pairing_mode);
    GAPBondCom_GetParameter(GAPBOND_MITM_PROTECTION, &mitm);
    GAPBondCom_GetParameter(GAPBOND_IO_CAPABILITIES, &iocapabilites);
    GAPBondCom_GetParameter(GAPBOND_OOB_ENABLED, &oob_flag);

    blueAPI_PairableModeSetReq(pairing_mode, (TBlueAPI_AuthRequirements)mitm, (TBlueAPI_IOCapabilities)iocapabilites, oob_flag);
}

/**
 * @fn          GAPBondMgr_Pair
 * @brief      start pairing.       
 *
 * @return     void
 */
void GAPBondMgr_Pair(void)
{
    uint8_t mitm = 0;
    GAPBondCom_GetParameter(GAPBOND_MITM_PROTECTION, &mitm);

    blueAPI_GATTSecurityReq(gapBond_ConnHandle, mitm, 7);
}

/**
 * @fn          GAPBondMgr_InputPassKey
 * @brief      Send passkey to gap bond manager when pairing with passkey entry,
 *                and local should input passkey.       
 *
 * @return     void
 */
void GAPBondMgr_InputPassKey(void)
{
    GAPBondCom_InputPassKey();
}

/**
 * @fn          GAPBondMgr_InputOobData
 * @brief      Send oob data to gap bond manager when pairing with out of bond,
 *                and local should input oob data.       
 *
 * @return     void
 */
void GAPBondMgr_InputOobData(void)
{
    blueAPI_RemoteOOBDataReqConf(gapBond_ConnectedDevAddr, gapBond_OOBData, blueAPI_CauseAccept);
}

/**
 * @brief      Get bonded device from flash storage, if bonded device exists, 
 *                return true, and device information will be returned by pBondedDevice  
 *                and local should input oob data.       
 * @param[out]    pBondedDevice  - pointer to bonded device information 
 *
 * @retval     true or false
 */
bool GAPBondMgr_GetBondedDevice(remote_BD_struct *pBondedDevice)
{
    bool bRet= false;
    int remote_BD_equal;
    remote_BD_struct remotebd;
    remote_BD_struct invalid_remotebd;
    memset(&invalid_remotebd, 0xFF, sizeof(invalid_remotebd));

    fs_load_remote_BD_struct(&remotebd);
    
    remote_BD_equal = memcmp(&remotebd, &invalid_remotebd, sizeof(invalid_remotebd));

    if (remote_BD_equal != 0)
    {
        memcpy(pBondedDevice, &remotebd, sizeof(remotebd));
        bRet=  true;
    }
    else
    {
        bRet= false;
    }
    
    return bRet;
}

/**
 * @fn          GAPBondMgr_EraseAllBondings
 * @brief      Erase bonding device inforamtion from falsh storage
 *
 * @return     void
 */
void GAPBondMgr_EraseAllBondings(void)
{
    remote_BD_struct remotebd;
    LTK_struct ltk;
    remLTK_struct remltk;
    IRK_struct       irk;
    cccData_struct cccData;

    memset(&remotebd, 0xFF, sizeof(remotebd));
    ltk.linkKeyLength = 0;
    remltk.linkKeyLength = 0;
    irk.linkKeyLength = 0;
    cccData.data_length = 0;

    fs_save_remote_BD_struct(&remotebd);
    fs_save_LTK_struct(&ltk);
    fs_save_remLTK_struct(&remltk);
    fs_save_IRK_struct(&irk);
    fs_save_cccData_struct(&cccData);

}

/******************************************************************
 * @fn          GAP_OOBInputCB
 * @brief      callback to send oob unput message to app task.   
 * @param    connHandle - handle of connection 
 *
 * @return     void
 */
void GAP_OOBInputCB(uint16_t handle)
{
    BEE_IO_MSG bee_io_msg;
    BT_STACK_MSG bt_gap_msg;

#ifdef GAP_DEBUG_FLAG
    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "GAP_OOBInputCB", 0);
#endif

    bee_io_msg.IoType = BT_STATUS_UPDATE;
    bee_io_msg.subType = BT_MSG_TYPE_BOND_OOB_INPUT;
    bt_gap_msg.msgData.gapBondOobInput.connHandle = handle;

    memcpy(&bee_io_msg.parm, &bt_gap_msg, sizeof(bee_io_msg.parm));

    GAP_SendBtMsgToApp(&bee_io_msg);
}

/**
 * @fn          gapbondmgr_Handle_ConnectMDLInfo
 * @brief      process blueAPI_EventConnectMDLInfo message from bt stack
 * @param    pConnectMDLInfo  - pointer to TBlueAPI_ConnectMDLInfo message
 *
 * @return     void
 */
void gapbondmgr_Handle_ConnectMDLInfo(PBlueAPI_ConnectMDLInfo pConnectMDLInfo )
{
#ifdef GAP_DEBUG_FLAG
    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "gapbondmgr_Handle_ConnectMDLInfo: MDL_ID=0x%x,dsPoolID=0x%x,dsDataOffset=0x%x,MTU=0x%x,Credits=0x%x.",
                5, pConnectMDLInfo->local_MDL_ID,
                   pConnectMDLInfo->dsPoolID,
                   pConnectMDLInfo->dsDataOffset,
                   pConnectMDLInfo->maxTPDUSize,
                   pConnectMDLInfo->maxTPDUdsCredits);
#endif
    gapBond_ConnHandle = pConnectMDLInfo->local_MDL_ID;
    /* Send Security Request. Mostly sent by slave. */
    if(TRUE == gapBond_SecReqEnable)
    {
        blueAPI_GATTSecurityReq(gapBond_ConnHandle, gapBond_SecReqRequirements, 7);
    }
}

/**
 * @fn          gapbondmgr_Handle_GATTServerStoreInd
 * @brief      process blueAPI_EventGATTServerStoreInd message from bt stack
 * @param    pStoreInd  - pointer to TBlueAPI_GATTServerStoreInd message
 *
 * @return     void
 */
void gapbondmgr_Handle_GATTServerStoreInd(PBlueAPI_GATTServerStoreInd pStoreInd)
{
    uint8_t         dataLen   = 0;
    uint8_t *       pData     = NULL;
    uint8_t remote_BD_equal;
    remote_BD_struct s_remote_BD;
    TBlueAPI_Cause  cause     = blueAPI_CauseReject;
    static cccData_struct s_cccData;
#ifdef GAP_DEBUG_FLAG
    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO,"gapbondmgr_Handle_GATTServerStoreInd: opCode=0x%x",
               1, pStoreInd->opCode);
#endif
    fs_load_remote_BD_struct(&s_remote_BD);
    remote_BD_equal = memcmp(pStoreInd->remote_BD, s_remote_BD.addr, 6);

    switch (pStoreInd->opCode)
    {
    case blueAPI_GATTStoreOpGetCCCBits:
        {
            if ( s_remote_BD.remote_bd_type == pStoreInd->remote_BD_Type && remote_BD_equal == 0 )
            {
                fs_load_cccData_struct(&s_cccData);

                if (s_cccData.data_length == 0)
                {
                    dataLen = 0;
                    pData   = NULL;
                    cause   = blueAPI_CauseReject;
                }
                else
                {
                    dataLen = s_cccData.data_length;
                    pData = s_cccData.val;
                    cause   = blueAPI_CauseSuccess;
                }
            }
            else
            {

            }
        }
        break;

    case blueAPI_GATTStoreOpGetAllCCCBits:
        {

            if (s_cccData.data_length == 0)
            {
                dataLen = 0;
                pData   = NULL;
                cause   = blueAPI_CauseReject;
            }
            else
            {
                dataLen = s_cccData.data_length;
                pData = s_cccData.val;
                cause   = blueAPI_CauseSuccess;
            }
        }
        break;

    case blueAPI_GATTStoreOpSetCCCBits:
        {
            if ( s_remote_BD.remote_bd_type == pStoreInd->remote_BD_Type && remote_BD_equal == 0)
            {
                s_cccData.data_length = pStoreInd->dataLength;
                memcpy( s_cccData.val, pStoreInd->data, pStoreInd->dataLength);
                fs_save_cccData_struct(&s_cccData);

                dataLen = pStoreInd->dataLength;
                pData   = pStoreInd->data;
                cause   = blueAPI_CauseSuccess;
            }
            else
            {
                dataLen = 0;
                pData   = NULL;
                cause   = blueAPI_CauseReject;
            }
        }
        break;

    case blueAPI_GATTStoreOpDeleteAllCCCBits:
        {
            cccData_struct s_cccData;
            s_cccData.data_length = 0;
            fs_save_cccData_struct(&s_cccData);

            cause   = blueAPI_CauseSuccess;
        }
        break;

    default:
        break;
    }

    blueAPI_GATTServerStoreConf(pStoreInd->opCode,
                                pStoreInd->remote_BD,
                                pStoreInd->remote_BD_Type,
                                pStoreInd->restartHandle,
                                dataLen,
                                pData,
                                cause
                               );
#ifdef GAP_DEBUG_FLAG
    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "<-- gapbondmgr_Handle_GATTServerStoreInd\n", 0 );
#endif
}

/**
 * @fn          gapbondmgr_Handle_AuthResultInd
 * @brief      process blueAPI_EventAuthResultInd message from bt stack
 * @param    pAuthResultInd  - pointer to TBlueAPI_AuthResultInd message
 *
 * @return     void
 */
bool gapbondmgr_Handle_AuthResultInd(PBlueAPI_AuthResultInd pAuthResultInd)
{
    TBlueAPI_Cause  cause = blueAPI_CauseSuccess;
#ifdef GAP_DEBUG_FLAG
    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO,"gapbondmgr_Handle_AuthResultInd: cause=0x%x",
               1, pAuthResultInd->cause);
#endif
// begin - save to flash
    remote_BD_struct s_remote_BD;
    s_remote_BD.addr[0] = pAuthResultInd->remote_BD[0];
    s_remote_BD.addr[1] = pAuthResultInd->remote_BD[1];
    s_remote_BD.addr[2] = pAuthResultInd->remote_BD[2];
    s_remote_BD.addr[3] = pAuthResultInd->remote_BD[3];
    s_remote_BD.addr[4] = pAuthResultInd->remote_BD[4];
    s_remote_BD.addr[5] = pAuthResultInd->remote_BD[5];
    s_remote_BD.remote_bd_type = pAuthResultInd->remote_BD_Type;
    fs_save_remote_BD_struct(&s_remote_BD);

    switch (pAuthResultInd->keyType)
    {
    case blueAPI_LinkKeyTypeLELocalLTK:
        {
            LTK_struct s_tmp;
            s_tmp.linkKeyLength = pAuthResultInd->linkKeyLength;
            memcpy( &s_tmp.linkKey, &pAuthResultInd->linkKey, pAuthResultInd->linkKeyLength);
            fs_save_LTK_struct(&s_tmp);
            //for slave
            GAPBondCom_PairStateCB(gapBond_ConnHandle, GAPBOND_PAIRING_STATE_BONDED, 0);
        }
        break;

    case blueAPI_LinkKeyTypeLERemoteLTK:
        {
            remLTK_struct s_tmp;
            s_tmp.linkKeyLength = pAuthResultInd->linkKeyLength;
            memcpy( &s_tmp.linkKey, &pAuthResultInd->linkKey, pAuthResultInd->linkKeyLength);
            fs_save_remLTK_struct(&s_tmp);
            //for master
            GAPBondCom_PairStateCB(gapBond_ConnHandle, GAPBOND_PAIRING_STATE_BONDED, 0);

        }
        break;

    case blueAPI_LinkKeyTypeLERemoteIRK:
        {
            IRK_struct s_tmp;
            s_tmp.linkKeyLength = pAuthResultInd->linkKeyLength;
            memcpy( &s_tmp.linkKey, &pAuthResultInd->linkKey, pAuthResultInd->linkKeyLength);
            fs_save_IRK_struct(&s_tmp);
        }
        break;

    case blueAPI_LinkKeyTypeDeleted:
        {
            {
                LTK_struct s_tmp;
                s_tmp.linkKeyLength = 0;
                fs_save_LTK_struct(&s_tmp);
            }

            {
                remLTK_struct s_tmp;
                s_tmp.linkKeyLength = 0;
                fs_save_remLTK_struct(&s_tmp);
            }

            {
                IRK_struct s_tmp;
                s_tmp.linkKeyLength = 0;
                fs_save_IRK_struct(&s_tmp);
            }
        }
    }
#ifdef GAP_DEBUG_FLAG
    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "store data into flash", 0);
#endif
    // end - save to flash

    blueAPI_AuthResultConf(pAuthResultInd->remote_BD,
                           (TBlueAPI_RemoteBDType)pAuthResultInd->remote_BD_Type,
                           cause);
    return true;
}

/**
 * @fn          gapbondmgr_Handle_AuthResultRequestInd
 * @brief      process blueAPI_EventAuthResultInd message from bt stack
 * @param    pAuthResultRequestInd  - pointer to TBlueAPI_AuthResultRequestInd message
 *
 * @return     bool
 */
bool gapbondmgr_Handle_AuthResultRequestInd(PBlueAPI_AuthResultRequestInd pAuthResultRequestInd)
{
    uint32_t errorno = 0;
    bool bIndError = false;
#ifdef GAP_DEBUG_FLAG
    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO,
               "gapbondmgr_Handle_AuthResultRequestInd: bd=0x%x 0x%x 0x%x 0x%x 0x%x 0x%x bdtype = 0x%x, keytype=0x%x \n", 8,
               pAuthResultRequestInd->remote_BD[0],
               pAuthResultRequestInd->remote_BD[1],
               pAuthResultRequestInd->remote_BD[2],
               pAuthResultRequestInd->remote_BD[3],
               pAuthResultRequestInd->remote_BD[4],
               pAuthResultRequestInd->remote_BD[5],
               pAuthResultRequestInd->remote_BD_Type,
               pAuthResultRequestInd->keyType);
#endif
    // begin - load from flash
    remote_BD_struct s_remote_BD;
    errorno = fs_load_remote_BD_struct(&s_remote_BD);

    uint8_t remote_BD_equal = memcmp(pAuthResultRequestInd->remote_BD, s_remote_BD.addr, 6);
//    if (errorno != 0 || remote_BD_equal != 0 || pAuthResultRequestInd->remote_BD_Type != s_remote_BD.remote_bd_type )
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

    static LTK_struct s_tmp; // for return s_tmp.linkKey
    switch (pAuthResultRequestInd->keyType)
    {
    case blueAPI_LinkKeyTypeLELocalLTK:
        {
            errorno = fs_load_LTK_struct(&s_tmp);
            if (errorno != 0 || s_tmp.linkKeyLength == 0)
            {
                blueAPI_AuthResultRequestConf(pAuthResultRequestInd->remote_BD,
                                              pAuthResultRequestInd->remote_BD_Type,
                                              0,
                                              NULL,
                                              pAuthResultRequestInd->keyType,
                                              pAuthResultRequestInd->restartHandle,
                                              blueAPI_CauseReject);
                bIndError = true;
            }
        }
        break;

    case blueAPI_LinkKeyTypeLERemoteLTK:
        {
            errorno = fs_load_remLTK_struct((remLTK_struct*)&s_tmp);
            if (errorno != 0 || s_tmp.linkKeyLength == 0)
            {
                blueAPI_AuthResultRequestConf(pAuthResultRequestInd->remote_BD,
                                              pAuthResultRequestInd->remote_BD_Type,
                                              0,
                                              NULL,
                                              pAuthResultRequestInd->keyType,
                                              pAuthResultRequestInd->restartHandle,
                                              blueAPI_CauseReject);
                bIndError = true;
            }
        }
        break;

    case blueAPI_LinkKeyTypeLERemoteIRK:
        {
            errorno = fs_load_IRK_struct((IRK_struct*)&s_tmp);
            if (errorno != 0 || s_tmp.linkKeyLength == 0)
            {
                blueAPI_AuthResultRequestConf(pAuthResultRequestInd->remote_BD,
                                              pAuthResultRequestInd->remote_BD_Type,
                                              0,
                                              NULL,
                                              pAuthResultRequestInd->keyType,
                                              pAuthResultRequestInd->restartHandle,
                                              blueAPI_CauseReject);
                bIndError = true;
            }
        }
        break;

    case blueAPI_LinkKeyTypeDeleted:
        {
            // oxff to erase
            blueAPI_AuthResultRequestConf(pAuthResultRequestInd->remote_BD,
                                          pAuthResultRequestInd->remote_BD_Type,
                                          0,
                                          NULL,
                                          pAuthResultRequestInd->keyType,
                                          pAuthResultRequestInd->restartHandle,
                                          blueAPI_CauseReject);
            bIndError = true;
        }
        break;
    }

    if (bIndError == true)
    {
        return true;
    }

#ifdef GAP_DEBUG_FLAG
    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "keytype=0x%x \n", 1, s_remote_BD.remote_bd_type);
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
    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "restartHandle=0x%x \n", 1, pAuthResultRequestInd->restartHandle);
#endif
    blueAPI_AuthResultRequestConf(pAuthResultRequestInd->remote_BD,
                                  pAuthResultRequestInd->remote_BD_Type,
                                  s_tmp.linkKeyLength,
                                  s_tmp.linkKey,
                                  pAuthResultRequestInd->keyType,
                                  pAuthResultRequestInd->restartHandle,
                                  blueAPI_CauseAccept
                                 );

    return true;
}

/**
 * @fn          gapbondmgr_Handle_PairableModeSetRsp
 * @brief      process blueAPI_EventPairableModeSetRsp message from bt stack
 * @param    pPairableModeSetRsp  - pointer to TBlueAPI_PairableModeSetRsp message
 *
 * @return     bool
 */
bool gapbondmgr_Handle_PairableModeSetRsp(PBlueAPI_PairableModeSetRsp pPairableModeSetRsp)
{
#ifdef GAP_DEBUG_FLAG
    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "gapbondmgr_Handle_PairableModeSetRsp: cause=0x%x.",
                1, pPairableModeSetRsp->cause);
#endif
    return true;
}

/**
 * @fn          gapbondmgr_Handle_UserPasskeyReqReplyRsp
 * @brief      process blueAPI_EventUserPasskeyReqReplyRsp message from bt stack
 * @param    pUserPasskeyReqReplyRsp  - pointer to TBlueAPI_UserPasskeyReqReplyRsp
 *                 message
 *
 * @return     bool
 */
bool gapbondmgr_Handle_UserPasskeyReqReplyRsp(PBlueAPI_UserPasskeyReqReplyRsp pUserPasskeyReqReplyRsp)
{
#ifdef GAP_DEBUG_FLAG
    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "gapbondmgr_Handle_UserPasskeyReqReplyRsp cause=0x%x.",
                    1, pUserPasskeyReqReplyRsp->cause);
#endif
    return true;
}

/**
 * @fn          gapbondmgr_Handle_RemoteOOBDataReqInd
 * @brief      process blueAPI_EventRemoteOOBDataReqInd message from bt stack
 * @param    pRemoteOOBDataReqInd  - pointer to TBlueAPI_RemoteOOBDataReqInd message
 *
 * @return     bool
 */
bool gapbondmgr_Handle_RemoteOOBDataReqInd(PBlueAPI_RemoteOOBDataReqInd pRemoteOOBDataReqInd)
{
#ifdef GAP_DEBUG_FLAG
    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "gapbondmgr_Handle_RemoteOOBDataReqInd .",0);
#endif
    //notify app to input oob data
    GAP_OOBInputCB(gapBond_ConnHandle);

    return true;
}

/**
 * @fn          gapbondmgr_Handle_GATTSecurityRsp
 * @brief      process blueAPI_EventGATTSecurityRsp message from bt stack
 * @param    pGATTSecurityRsp  - pointer to TBlueAPI_GATTSecurityRsp message
 *
 * @return     bool
 */
bool gapbondmgr_Handle_GATTSecurityRsp(PBlueAPI_GATTSecurityRsp pGATTSecurityRsp)
{
#ifdef GAP_DEBUG_FLAG
    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "<-- gapbondmgr_Handle_GATTSecurityRsp: keyType=%d keySize=%d cause=%d\r\n", 3,
               pGATTSecurityRsp->keyType,
               pGATTSecurityRsp->keySize,
               pGATTSecurityRsp->cause
              );
#endif
    return true;
}

/**
 * @fn          GAPBondMgr_ProcessEvent
 * @brief      process blueapi upstream messages here
 * @param    pMsg  - pointer to type TBlueAPI_UsMessage message
 *
 * @return     bool
 */
bool GAPBondMgr_ProcessEvent( PBlueAPI_UsMessage pMsg )
{
    switch ( pMsg->Command )
    {
    default:
#ifdef GAP_DEBUG_FLAG
        DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "GAPBondMgr_Not_ProcessEvent: Command(0x%02x)", 1, pMsg->Command);
#endif
        break;
    case blueAPI_EventConnectMDLInfo:
        gapbondmgr_Handle_ConnectMDLInfo(&pMsg->p.ConnectMDLInfo);
        break;

    case blueAPI_EventACLStatusInfo:
        GAPBondCom_HandleACLStatusInfo(&pMsg->p.ACLStatusInfo);
        break;

    case blueAPI_EventGATTServerStoreInd:
        gapbondmgr_Handle_GATTServerStoreInd(&pMsg->p.GATTServerStoreInd);
        break;

    case blueAPI_EventAuthResultInd:
        gapbondmgr_Handle_AuthResultInd(&pMsg->p.AuthResultInd);
        break;

    case blueAPI_EventAuthResultRequestInd:
        gapbondmgr_Handle_AuthResultRequestInd(&pMsg->p.AuthResultRequestInd);
        break;

    case blueAPI_EventPairableModeSetRsp:
        gapbondmgr_Handle_PairableModeSetRsp(&pMsg->p.PairableModeSetRsp);
        break;

    case blueAPI_EventUserPasskeyReqInd:
        GAPBondCom_HandleUserPasskeyReqInd(&pMsg->p.UserPasskeyReqInd);
        break;

    case blueAPI_EventUserPasskeyNotificationInfo:
        GAPBondCom_HandleUserPasskeyNotificationInfo(&pMsg->p.UserPasskeyNotificationInfo);
        break;


    case blueAPI_EventUserPasskeyReqReplyRsp:
        gapbondmgr_Handle_UserPasskeyReqReplyRsp(&pMsg->p.UserPasskeyReqReplyRsp);
        break;

    case blueAPI_EventRemoteOOBDataReqInd:
        gapbondmgr_Handle_RemoteOOBDataReqInd(&pMsg->p.RemoteOOBDataReqInd);
        break;

    //for master only
    case blueAPI_EventGATTSecurityRsp:
        gapbondmgr_Handle_GATTSecurityRsp(&pMsg->p.GATTSecurityRsp);
        break;
    }
    return true;
}

