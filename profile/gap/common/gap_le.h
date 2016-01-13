/**
*****************************************************************************************
*     Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
  * @file    gap_le.h
  * @brief   This file contains all the constants and functions prototypes for GAP protocol. 
  * @details
  * @author  Ranhui
  * @date    2015-5-22
  * @version v0.1
  * *************************************************************************************
  */

/* Define to prevent recursive inclusion **/
#ifndef __GAP_LE_H__
#define __GAP_LE_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** Add Includes here **/
#include <blueapi_types.h>

///@cond
/* GAP common APIs declared here for all LE roles to use. */
extern void GAP_ParaInit(uint8_t gapRole);
extern TGAP_STATUS GAP_SetParameter(uint16_t param, uint8_t len, void *pValue);
extern TGAP_STATUS GAP_GetParameter(uint16_t param, void *pValue);
extern void GAP_ConnParaUpdateCB(uint16_t handle, uint8_t status);
extern bool GAP_ProcessGapEvent(PBlueAPI_UsMessage pMsg);
///@endcond

/*-------------------------------------------------------------------
-------------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __GAP_LE_H__ */
