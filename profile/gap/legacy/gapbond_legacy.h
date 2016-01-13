/**
 *****************************************************************************************
 *     Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
 *****************************************************************************************
 * @file    gapbond_legacy.h
 * @brief   This file contains all the functions prototypes for the legacy GAP bond and pairing 
 *          related functions. 
 * @details
 * @author  kyle_xu
 * @date    2015-11-20
 * @version v0.1
 * ****************************************************************************************
 */

/* Define to prevent recursive inclusion **/
#ifndef __GAPBOND_LEGACY_H__
#define __GAPBOND_LEGACY_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** Add Includes here **/
#include <blueapi_types.h>
#include <flash_storage.h>
#include <gapbond_common.h>

#define GAPBOND_BTMODE_21DISABLED       blueAPI_BTMode21Disabled
#define GAPBOND_BTMODE_21ENABLED        blueAPI_BTMode21Enabled

/**
 * @defgroup GAPROLES_BONDMGR_EXPORT_APIS GAP Bond Manager Export API Functions
 *
 * @{
 */
TGAP_STATUS GAPBondlegacy_SetParameter(uint16_t param, uint8_t len, void *pvalue);
TGAP_STATUS GAPBondlegacy_GetParameter(uint16_t param, void *pvalue);

void GAPBondlegacy_SetPairable(void);
void GAPBondlegacy_Pair(uint8_t *remote_bd);
void GAPBondlegacy_InputPassKey(void);
void GAPBondlegacy_InputOOBData(void);
bool GAPBondlegacy_GetBondedDevice(remote_BD_struct *pbonded_device);
void GAPBondlegacy_EraseAllBondings(void);
void GAPBondlegacy_Authorize(uint8_t *remote_bd, TBlueAPI_Cause result);

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
bool GAPBondlegacy_ProcessEvent(PBlueAPI_UsMessage pmsg);

void GAPBondlegacy_ParaInit(void);

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
