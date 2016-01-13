/**
 *********************************************************************************************************
 *               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
 *********************************************************************************************************
 * @file      gapbond_dual.h
 * @brief    Head file for gapbond dualmode role
 * @details
 * @author    kyle_xu
 * @date      2015-11-25
 * @version   v0.1
 *
 **********************************************************************************************************
 */
#ifndef __GAPBOND_DUAL_H__
#define __GAPBOND_DUAL_H__

#include <gapbond_legacy.h>
#include <gapbondmgr.h>

#ifdef __cplusplus
extern "C" {
#endif

TGAP_STATUS GAPBonddual_SetParameter(uint16_t param, uint8_t len, void *pvalue);
TGAP_STATUS GAPBonddual_GetParameter(uint16_t param, void *pvalue);
void GAPBonddual_InputPassKey(void);
void GAPBonddual_InputlegacyOOBData(void);
void GAPBonddual_InputLEOOBData(void);

#ifdef __cplusplus
}
#endif    /*  __cplusplus */

#endif  /* __GAPBOND_DUAL_H__ */
