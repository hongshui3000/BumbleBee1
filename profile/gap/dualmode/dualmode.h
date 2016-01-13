/**
 *********************************************************************************************************
 *               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
 *********************************************************************************************************
 * @file      dualmode.h
 * @brief    Head file for gap dualmode role
 * @details
 * @author    kyle_xu
 * @date      2015-11-25
 * @version   v0.1
 *
 **********************************************************************************************************
 */
#ifndef __DUALMODE_H__
#define __DUALMODE_H__

#include <blueapi_types.h>

#ifdef __cplusplus
extern "C" {
#endif

bool dualmode_StartBtStack(void);
bool dualmode_HandleBlueAPIMessage(PBlueAPI_UsMessage pmsg);


#ifdef __cplusplus
}
#endif    /*  __cplusplus */

#endif  /* __DUALMODE_H__ */
