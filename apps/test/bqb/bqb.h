/**
************************************************************************************************************
*               Copyright(c) 2014, Realtek Semiconductor Corporation. All rights reserved.
************************************************************************************************************
* @file     bqb.h
* @brief    header file of bqb.c.
* @author   tifnan_ge
* @date     2014-05-21
* @version  v0.1
*************************************************************************************************************
*/
#ifndef __BQB__H__
#define __BQB__H__

#include "bqb_database.h"
#include "FreeRTOS.h"
#include "task.h"

#ifdef  __cplusplus
extern "C" {
#endif      /* __cplusplus */

#define GATTDEMO_PRIORITY          (tskIDLE_PRIORITY + 1)   /* Task priorities. */
#define GATTDEMO_STACK_SIZE        2048

/* macros */
#define ADV_DATA_IND_0      0       /**< @brief use advertising data in bBQBAdData0[]*/
#define ADV_DATA_IND_1      1       /**< @brief use advertising data in bBQBAdData1[]*/
#define ADV_DATA_IND_2      2       /**< @brief use advertising data in bBQBAdData2[]*/

/**@brief support prepare write */
#define BQB_PREPARE_WRITE_SUPPPRT 1

extern xTaskHandle stGattBQBHandle;

extern void BQB_Task(void *pParameters);

#ifdef  __cplusplus
}
#endif      /*  __cplusplus */
#endif      /* __BQB__H__ */

