/**
*****************************************************************************************
*     Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
  * @file    gapbondmgr.h
  * @brief   This file contains all the common functions prototypes for the GAP bond and pairing
  * @details
  * @author  kyle_xu
  * @date    2015-11-25
  * @version v0.1
  * *************************************************************************************
  */

/* Define to prevent recursive inclusion **/
#ifndef __GAPBOND_COMMON_H__
#define __GAPBOND_COMMON_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** Add Includes here **/
#include <gap.h>
#include <rtl_types.h>
#include <blueapi_types.h>
#include <flash_storage.h>

/** @addtogroup RTK_GAP_MODULE RTK GAP Module
  * @{
  */

/** @defgroup GAP_Bond_Manager GAP Bond Manager
  * @{
  */ 


/** @defgroup GAP_BOND_MANAGER_Exported_Constants GAP Bond Manager Exported Constants
  * @{
  */

/** @defgroup GAPBOND_MANAGER_PARAMETERS GAP Bond Manager Parameters
 * @{
 */
#define GAPBOND_PAIRING_MODE                    0x100  //!< Pairing Mode: @ref  GAPBOND_PAIRING_MODE_DEFINES. Read/Write. Size is uint8_t. Default is GAPBOND_PAIRING_MODE_WAIT_FOR_REQ.
#define GAPBOND_MITM_PROTECTION                 0x102  //!< Man-In-The-Middle (MITM) basically turns on Passkey protection in the pairing algorithm. Read/Write. Size is uint8_t. Default is 0(disabled).
#define GAPBOND_IO_CAPABILITIES                 0x103  //!< I/O capabilities.  Read/Write. Size is uint8_t. Default is GAPBOND_IO_CAP_DISPLAY_ONLY @ref GAPBOND_IO_CAP_DEFINES.
#define GAPBOND_OOB_ENABLED                     0x104  //!< OOB data available for pairing algorithm. Read/Write. Size is uint8_t. Default is 0(disabled).
#define GAPBOND_OOB_DATA                        0x105  //!< OOB Data. Read/Write. size uint8_t[16]. Default is all 0's.
#define GAPBOND_PASSKEY                         0x106  //!< The default passcode for MITM protection. size is uint32_t. Range is 0 - 999,999. Default is 0.
#define GAPBOND_FIXED_PASSKEY_ENABLE            0x107  //!< The default passcode for MITM protection. size is uint32_t. Range is 0 - 999,999. Default is 0.
#define GAPBOND_SEC_REQ_ENABLE                  0x108  //!< send smp security request when connected
#define GAPBOND_SEC_REQ_REQUIREMENT             0x109  //!< security request requirements

#define GAPBOND_BTMODE                          0x200
#define GAPBOND_OOB_DATA_R                      0x201
#define GAPBOND_OOB_DATA_C                      0x202

/** @} End GAPBOND_MANAGER_PARAMETERS */

/** @defgroup GAPBOND_PAIRING_MODE_DEFINES GAP Bond Manager Pairing Modes
 * @{
 */
#define GAPBOND_PAIRING_MODE_NO_PAIRING         0x00  //!< Pairing is not allowed
#define GAPBOND_PAIRING_MODE_PAIRABLE           0x01  //!< Wait for a pairing request or slave security request
/** @} End GAPBOND_PAIRING_MODE_DEFINES */

/** @defgroup GAPBOND_SEC_REQ_DEFINES GAP Bond Manager Pairing Modes
 * @{
 */
#define GAPBOND_SEC_REQ_NO_MITM                 0x00  //!< Pairing is not allowed
#define GAPBOND_SEC_REQ_YES_MITM                0x01  //!< Wait for a pairing request or slave security request
/** @} End GAPBOND_SEC_REQ_DEFINES */

/** @defgroup GAPBOND_IO_CAP_DEFINES GAP Bond Manager I/O Capabilities
 * @{
 */
#define GAPBOND_IO_CAP_DISPLAY_ONLY             blueAPI_IOCapDisplayOnly            /**<  only a Display present, no Keyboard or Yes/No Keys.  */
#define GAPBOND_IO_CAP_DISPLAY_YES_NO           blueAPI_IOCapDisplayYesNo           /**<  Display and Yes/No Keys present.  */
#define GAPBOND_IO_CAP_KEYBOARD_ONLY            blueAPI_IOCapKeyboardOnly           /**<  only a Keyboard present, no Display.  */
#define GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT       blueAPI_IOCapNoIO                   /**<  no input/output capabilities.  */
#define GAPBOND_IO_CAP_KEYBOARD_DISPLAY         blueAPI_IOCapKeyboardDisplay        /**<  BLE: Keyboard and Display present.  */
/** @} End GAPBOND_IO_CAP_DEFINES */

/** @defgroup GAPBOND_PAIRING_STATE_DEFINES GAP Bond Manager Pairing States
 * @{
 */
#define GAPBOND_PAIRING_STATE_STARTED           0x00  //!< Pairing started
#define GAPBOND_PAIRING_STATE_COMPLETE          0x01  //!< Pairing complete
#define GAPBOND_PAIRING_STATE_BONDED            0x02  //!< Devices bonded
/** @} End GAPBOND_PAIRING_STATE_DEFINES */

/** @defgroup GAPBOND_ENCRYPT_STATE_DEFINES GAP Bond Manager Encrypt States
 * @{
 */
#define GAPBOND_ENCRYPT_STATE_ENABLED           0x00 //!< Link Encrypted
#define GAPBOND_ENCRYPT_STATE_DISABLED          0x01 //!< Link Not Encrypted
/** @} End GAPBOND_ENCRYPT_STATE_DEFINES */

/** @defgroup GAPBOND_MITM_DEFINES GAP Bond Mitm Flags
 * @{
 */
#define GAPBOND_AUTH_NO_MITM_NO_BOND            blueAPI_AuthNoMITMRequiredNoStore
#define GAPBOND_AUTH_NO_MITM_YES_BOND           blueAPI_AuthNoMITMRequiredBonding
#define GAPBOND_AUTH_YES_MITM_NO_BOND           blueAPI_AuthMITMRequiredNoStore
#define GAPBOND_AUTH_YES_MITM_YES_BOND          blueAPI_AuthMITMRequiredBonding
#define GAPBOND_AUTH_NO_MITM_GENERAL_BOND       blueAPI_AuthNoMITMRequiredGeneralBonding
#define GAPBOND_AUTH_YES_MITM_GENERAL_BOND      blueAPI_AuthMITMRequiredGeneralBonding

/** @} End GAPBOND_MITM_DEFINES */

/**
  * @}End GAP_BOND_MANAGER_Exported_Constants */

void GAPBondCom_ParaInit(void);
TGAP_STATUS GAPBondCom_SetParameter(uint16_t param, uint8_t len, void *pvalue);
TGAP_STATUS GAPBondCom_GetParameter(uint16_t param, void *pvalue);
void GAPBondCom_InputPassKey(void);
void GAPBondCom_PasskeyEntryDisplayCB(uint16_t handle, uint32_t display_value);
void GAPBondCom_PairStateCB(uint16_t handle, uint8_t state, uint8_t status);
bool GAPBondCom_HandleACLStatusInfo(PBlueAPI_ACLStatusInfo pinfo);
bool GAPBondCom_HandleUserPasskeyReqInd(PBlueAPI_UserPasskeyReqInd pind);
bool GAPBondCom_HandleUserPasskeyNotificationInfo(PBlueAPI_UserPasskeyNotificationInfo pinfo);

#ifdef __cplusplus
}
#endif

#endif /* __GAPBOND_COMMON_H__ */
