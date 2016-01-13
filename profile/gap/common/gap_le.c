enum { __FILE_NUM__ = 0 };
/**
*****************************************************************************************
*     Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
  * @file    gap_le.c
  * @brief   This file provides the GAP LE role common functions, used by all GAP LE roles.
  * @details
  * @author  Ethan
  * @date    11-Aug-2015
  * @version v0.1
  ******************************************************************************
  * @attention
  * <h2><center>&copy; COPYRIGHT 2015 Realtek Semiconductor Corporation</center></h2>
  ******************************************************************************
  */

/** Add Includes here **/
#include <string.h>
#include <FreeRTOS.h>
#include <blueapi.h>
#include <trace.h>
#include <gap.h>
#include <gap_le.h>
#include <bee_message.h>
#include <common_defs.h>

/** @cond private**/
/** @addtogroup Gap_Common
  * @{
  */

/** @defgroup Gap_Common_Private_Variables Gap Commnon Private Variables
  * @{
  */
/** Add all private variables here **/
uint8_t  gapPara_profileRole = GAP_PROFILE_PERIPHERAL;   /* GAP role. */
static uint8_t  gapPara_bdAddr[B_ADDR_LEN]; /* remote device BT address. */
static uint8_t  gapPara_bdAddrType = blueAPI_LocalBDTypeLEPublic;   /* ? remote or local device BT address? */
static uint8_t  gapPara_DeviceName[GAP_DEVICE_NAME_LEN] = "GAP";    /* local device name. */
static uint16_t gapPara_Appearance = GAP_GATT_APPEARANCE_UNKNOWN;   /* local device appearence. */
#ifdef BT_GAP_PARAM_TX_POWER_SET
static int8_t   gapPara_btTxPower = 0;
#endif
/** End of Gap_Common_Private_Variables
  * @}
  */

/** End of Gap_Common
  * @}
  */
/** @endcond **/

/**
  * @brief  Initialize parameters of gap GAP role. 
  * @param  gapRole: GAP role.
  * @retval None
  */
void GAP_ParaInit(uint8_t gapRole)
{
    gapPara_profileRole = gapRole;
    gapPara_bdAddrType = blueAPI_LocalBDTypeLEPublic;
    gapPara_Appearance = GAP_GATT_APPEARANCE_UNKNOWN;

#if defined (RTL8762AX_VA)
    memcpy(gapPara_bdAddr, (uint8_t *)0x20000266, 6);    
#elif defined (RTL8762AX_VB)
    memcpy(gapPara_bdAddr, (uint8_t *)0x20000076, 6);    
#endif
}

/**
  * @brief  Set a GAP Role parameter.
  *         NOTE: You can call this function with a GAP Peripheral Parameter ID and it will set the
  *         GAP Parameter.  GAP Peripheral Parameters are defined in (GAP.h).  Also,
  *         the "len" field must be set to the size of a "uint16_t" and the
  *         "pValue" field must point to a "uint16".
  * @param  param - Profile parameter ID: @ref GAP_PERIPHERAL_PARAMETERS
  * @param  len - length of data to write
  * @param  pValue - pointer to data to write.  This is dependent on
  *                  the parameter ID and WILL be cast to the appropriate
  *                  data type (example: data type of uint16 will be cast to
  *                  uint16 pointer).
  * @retval gapAPI_CauseSuccess or INVALIDPARAMETER (invalid paramID)
  */
TGAP_STATUS GAP_SetParameter( uint16_t param, uint8_t len, void *pValue )
{
    TGAP_STATUS ret = gapAPI_CauseSuccess;

    /* Common param set should be added here. */
    switch ( param )
    {
        case GAPPARA_PROFILE_ROLE:
            if (len == sizeof ( uint8_t ))
            {
                gapPara_profileRole = *((uint8_t*)pValue);
            }
            else
            {
                ret = gapAPI_InvalidRange;
            }
            break;

        case GAPPARA_BD_ADDR_TYPE:
            if (len == sizeof( uint8_t ))
            {
                gapPara_bdAddrType = *((uint8_t*)pValue);
            }
            else
            {
                ret = gapAPI_InvalidRange;
            }
            break;

        case GAPPRRA_DEVICE_NAME:
            memset( gapPara_DeviceName, 0, GAP_DEVICE_NAME_LEN);
            memcpy( gapPara_DeviceName, pValue, len);
            break;

        case GAPPRRA_APPEARANCE:
            if ( len == sizeof ( uint16_t ) )
            {
                gapPara_Appearance = *((uint16_t*)pValue);
            }
            else
            {
                ret = gapAPI_InvalidRange;
            }
            break;

        default:
            /* Param will be set in specific roles. */
            break;
    }

    return ( ret );
}

/**
  * @brief  Get a GAP Role parameter.
  *
  *         NOTE: You can call this function with a GAP Peripheral Parameter ID and it will get a
  *         GAP Peripheral Parameter.  GAP Peripheral Parameters are defined in (GAP.h).  Also, the
  *         "pValue" field must point to a "uint16".
  *
  * @param  param - Profile parameter ID: @ref GAP_PERIPHERAL_PARAMETERS
  * @param  pValue - pointer to location to get the value.  This is dependent on
  *                  the parameter ID and WILL be cast to the appropriate
  *                  data type (example: data type of uint16 will be cast to
  *                  uint16 pointer).
  *
  * @return gapAPI_CauseSuccess or INVALIDPARAMETER (invalid paramID)
  */
TGAP_STATUS GAP_GetParameter( uint16_t param, void *pValue )
{
    TGAP_STATUS ret = gapAPI_CauseSuccess;

    /* Common param set should be added here. */
    switch ( param )
    {
        case GAPPARA_PROFILE_ROLE:
            *((uint8_t*)pValue) = gapPara_profileRole;
            break;
        case GAPPARA_BD_ADDR:
            memcpy( pValue, gapPara_bdAddr, B_ADDR_LEN ) ;
            break;
        case GAPPARA_BD_ADDR_TYPE:
            *((uint8_t*)pValue) = gapPara_bdAddrType;
            break;
        case GAPPRRA_APPEARANCE:
            *((uint16_t*)pValue) = gapPara_Appearance;
            break;
#ifdef BT_GAP_PARAM_TX_POWER_SET
        case GAPPARA_BLE_TX_POWER:
            *((int8_t*)pValue) = gapPara_btTxPower;
            break;
#endif
        default:
            /* Param will be got in specific roles. */
            break;
    }

    return ( ret );
}

/******************************************************************
 * @fn          peripheralConnParaUpdateCB
 * @brief      callback to send connection parameter update change message to app task.   
 * @param    connHandle - handle of connection 
 * @param    status - status of update result, 0 -success, otherwise fail.  
 *
 * @return     void
 */
void GAP_ConnParaUpdateCB(uint16_t handle, uint8_t status)
{
    BEE_IO_MSG bee_io_msg;
    BT_STACK_MSG bt_gap_msg;

#ifdef GAP_DEBUG_FLAG
    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "GAP_ConnParaUpdateCB handle=0x%x, status=%d", 2, handle, status);
#endif

    bee_io_msg.IoType = BT_STATUS_UPDATE;
    bee_io_msg.subType = BT_MSG_TYPE_CONN_PARA_UPDATE_CHANGE;
    bt_gap_msg.msgData.gapConnParaUpdateChange.connHandle = handle;
    bt_gap_msg.msgData.gapConnParaUpdateChange.status = status;

    memcpy(&bee_io_msg.parm, &bt_gap_msg, sizeof(bee_io_msg.parm));

    GAP_SendBtMsgToApp(&bee_io_msg);
}

/**
  * @brief  process blueAPI_EventRegisterRsp message from bt stack.
  * @param  pRegisterRsp - message sent from upper stack.
  * @retval TRUE - need to release buffer by application.
  *         FALSE - no need to release buffer by application.
  */
bool GAP_Handle_RegisterRsp(PBlueAPI_RegisterRsp pRegisterRsp )
{
#ifdef GAP_DEBUG_FLAG
    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "GAP_Handle_RegisterRsp: cause=%d", 1, pRegisterRsp->cause);
#endif
    if ( pRegisterRsp->cause == blueAPI_CauseSuccess )
    {
        /* Upper Stack register success */
    }
    else
    {
        /* Upper Stack register fail */
    }

    return( true );
}

/**
  * @brief  process blueAPI_EventActInfo message from bt stack.
  * @param  pActInfo - message sent from upper stack.
  * @retval TRUE - need to release buffer by application.
  *         FALSE - no need to release buffer by application.
  */
bool GAP_Handle_ActInfo(PBlueAPI_ActInfo pActInfo )
{
#ifdef GAP_DEBUG_FLAG
    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "GAP_Handle_ActInfo: cause=%d", 1, pActInfo->cause);
#endif
    if (pActInfo->cause == blueAPI_CauseSuccess)
    {
        //gapRole_state = GAPSTATE_STACK_READY;
        memcpy(gapPara_bdAddr, pActInfo->local_BD, B_ADDR_LEN );
        DBG_BUFFER(MODULE_PROFILE,LEVEL_INFO,"    My Local BD-- 0x%2x,0x%2x,0x%2x,0x%2x,0x%2x,0x%2x",6,
                    pActInfo->local_BD[5],
                    pActInfo->local_BD[4],
                    pActInfo->local_BD[3],
                    pActInfo->local_BD[2],
                    pActInfo->local_BD[1],
                    pActInfo->local_BD[0]);
        /* first config device name. */
        blueAPI_DeviceConfigDeviceNameSetReq(gapPara_DeviceName);
    }
    return ( true );
}

#ifdef BT_GAP_PARAM_TX_POWER_SET
/**
  * @brief  process blueAPI_EventSetBleTxPowerRsp message from bt stack.
  * @param  pActInfo - message sent from upper stack.
  * @retval TRUE - need to release buffer by application.
  *         FALSE - no need to release buffer by application.
  */
void GAP_Handle_SetBleTxPowerRsp(PBlueAPI_SetBleTxPowerRsp pSetBleTxPowerRsp)
{
    BOOL respResult = TRUE;
    BEE_IO_MSG bee_io_msg;
    BT_STACK_MSG btGapMsg;
    
#ifdef GAP_DEBUG_FLAG
    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "GAP_Handle_SetBleTxPowerRsp: cause=%d tx_power_index = %d subCause=0x%x\r\n",3,
                              pSetBleTxPowerRsp->cause,
                              pSetBleTxPowerRsp->tx_power_index,
                              pSetBleTxPowerRsp->subCause
                              );
#endif

    if(pSetBleTxPowerRsp->cause == blueAPI_CauseSuccess)
    {
        switch(pSetBleTxPowerRsp->tx_power_index)
        {
            case TX_POWER_MINUS_16_DBM:
                gapPara_btTxPower = -16;
                break;
            case TX_POWER_0_DBM:
                gapPara_btTxPower = 0;
                break;
            case TX_POWER_3_DBM:
                gapPara_btTxPower = 3;
                break;
            default:
                respResult = FALSE;
                DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "GAP_Handle_SetBleTxPowerRsp: tx_power_index error!\r\n",0);
                break;
        }
    }
    else
    {
        respResult = FALSE;
    }

    bee_io_msg.IoType = BT_STATUS_UPDATE;
    bee_io_msg.subType = BT_MSG_TYPE_PARAM_SET_RESULT;
    btGapMsg.msgData.gapBTParamSetResult.bleParamType = BLE_PARAM_TYPE_TX_POWER;
    btGapMsg.msgData.gapBTParamSetResult.result = respResult;

    memcpy(&bee_io_msg.parm, &btGapMsg, sizeof(bee_io_msg.parm));

    GAP_SendBtMsgToApp(&bee_io_msg);
}
#endif

/**
  * @brief  process TBlueAPI_UsMessage message from bt stack, only gap related messages are handled here.
  * @param  pMsg - message sent from upper stack.
  * @retval TRUE - need to release buffer by application.
  *         FALSE - no need to release buffer by application.
  */
bool GAP_ProcessGapEvent( PBlueAPI_UsMessage pMsg )
{
#ifdef GAP_DEBUG_FLAG
    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "GAPRole_ProcessEvent: Command(0x%02x)", 1, pMsg->Command);
#endif

    switch ( pMsg->Command )
    {
    default:
#ifdef GAP_DEBUG_FLAG
    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "GAPRole_Not_ProcessEvent: Command(0x%02x)", 1, pMsg->Command);
#endif
        break;

    case blueAPI_EventRegisterRsp:
        GAP_Handle_RegisterRsp( &pMsg->p.RegisterRsp );
        break;

    case blueAPI_EventActInfo:
        GAP_Handle_ActInfo( &pMsg->p.ActInfo );
        break;

#ifdef BT_GAP_PARAM_TX_POWER_SET
    case blueAPI_EventSetBleTxPowerRsp:
        GAP_Handle_SetBleTxPowerRsp( &pMsg->p.SetBleTxPowerRsp );
        break;
#endif

    }
    return true;
}

