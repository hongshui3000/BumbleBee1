/**
*****************************************************************************************
*     Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
  * @file    gapbondmgr.h
  * @brief   This file contains all the functions prototypes for the GAP bond and pairing 
  *          related functions. 
  * @details
  * @author  ranhui
  * @date    11-5-2015
  * @version v1.0
  * *************************************************************************************
  */

/* Define to prevent recursive inclusion **/
#ifndef GAPBONDMGR_H
#define GAPBONDMGR_H

#ifdef __cplusplus
extern "C"
{
#endif

/** Add Includes here **/
#include <gap.h>
#include <rtl_types.h>
#include <blueapi_types.h>
#include <flash_storage.h>
#include <gapbond_common.h>

/**
 * @defgroup GAPROLES_BONDMGR_EXPORT_APIS GAP Bond Manager Export API Functions
 *
 * @{
 */
extern void GAPBondMgr_ParaInit(void);
extern TGAP_STATUS GAPBondMgr_SetParameter( uint16_t param, uint8_t len, void *pValue );
extern TGAP_STATUS GAPBondMgr_GetParameter( uint16_t param, void *pValue );
extern void GAPBondMgr_SetPairable(void);
extern void GAPBondMgr_InputPassKey(void);
extern void GAPBondMgr_InputOobData(void);
extern bool GAPBondMgr_GetBondedDevice(remote_BD_struct *pBondedDevice);
extern void GAPBondMgr_EraseAllBondings(void);
extern void GAPBondMgr_Pair(void);
/**
* @}
*/

/**
 * @internal
 *
 * @brief        GAP Bond Manager Task event processor.
 *                  This function is called to process all events for the task.
 *                  Events include timers, messages and any other user defined
 *                  events.
 *
 * @param     task_id  - The OSAL assigned task ID.
 * @param     events - events to process.  This is a bit map and can
 *                  contain more than one event.
 *
 * @return      events not processed
 */
extern bool GAPBondMgr_ProcessEvent( PBlueAPI_UsMessage pMsg );
/*-------------------------------------------------------------------
-------------------------------------------------------------------*/

/**
  * @}END GAP_Bond_Manager */

/**
  * @}
  */


#ifdef __cplusplus
}
#endif

#endif /* GAPBONDMGR_H */
