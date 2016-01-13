/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file      peripheral.h
* @brief    Head file for Gap peripheral role
* @details
* @author    ranhui
* @date      2015-4-27
* @version   v0.1
* *********************************************************************************************************
*/

/* Define to prevent recursive inclusion **/
#ifndef PERIPHERAL_H
#define PERIPHERAL_H

#ifdef __cplusplus
extern "C"
{
#endif

/** Add Includes here **/
#include <rtl_types.h>
#include <blueapi_types.h>
#include <gap.h>

/** @addtogroup RTK_GAP_MODULE RTK GAP Module
  * @{
  */

/** @defgroup GAP_Peripheral_Role GAP Peripheral Role
  * @{
  */ 

/** @defgroup GAP_Peripheral_Exported_API_Functions GAP Peripheral Exported API Functions
  * @{
  */
extern TGAP_STATUS peripheralSetGapParameter( uint16_t param, uint8_t len, void *pValue );
extern TGAP_STATUS peripheralGetGapParameter( uint16_t param, void *pValue );
extern TGAP_STATUS peripheral_Disconnect(void);
extern TGAP_STATUS peripheral_SendUpdateParam(void);
extern TGAP_STATUS peripheral_StartAdvertising(void);
extern TGAP_STATUS peripheral_StopAdvertising(void);
extern TGAP_STATUS peripheral_Init_StartAdvertising(void);
/** End of GAP_Peripheral_Exported_API_Functions
* @}
*/

///@cond
extern void peripheral_GapParaInit(void);
extern void peripheral_Handle_ServiceRegisterCompleteEvt(void);
bool peripheral_HandleBlueAPIMessage( PBlueAPI_UsMessage pMsg );
///@endcond

extern bool peripheral_StartBtStack(void);
extern void BtStack_Init_Peripheral(void);

#ifdef ANCS_SUPPORT
void peripheral_AncsDiscover(void);
bool peripheral_AncsGetNotificationAttributes(uint32_t NotificationUID, uint8_t *pAttributeIDs, uint8_t AttributeIDsLen);
bool peripheral_AncsGetAppAttributes(uint8_t *AppIdentifier, char *pAttributeIDs, uint8_t AttributeIDsLen);
bool peripheral_AncsPerformNotificationAction(uint32_t NotificationUID, uint8_t ActionID);
#endif
/** End of GAP_Peripheral_Role
* @}
*/

/** End of RTK_GAP_MODULE
* @}
*/

/*-------------------------------------------------------------------
-------------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* PERIPHERAL_H */
