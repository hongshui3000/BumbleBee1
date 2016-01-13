/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file      test_legacy_application.h
* @brief     test legacy application app implementation
* @details   test legacy application app implementation
* @author    kyle_xu
* @date      2015-11-25
* @version   v0.1
* *********************************************************************************************************
*/
#ifndef __TEST_DUAL_APPLICATION__
#define __TEST_DUAL_APPLICATION__

#ifdef __cplusplus
extern "C" {
#endif

/* We can add different services for peripheral role test. */
#define SIMPLE_SERVICE_EN   0
#define DIS_SERVICE_EN      0
#define BAS_SERVICE_EN      0
#define RSC_SERVICE_EN      1
#define CSC_SERVICE_EN      1

#include "bee_message.h"
#include "bterrcod.h"

#if SIMPLE_SERVICE_EN
#include "simple_ble_service.h"
#endif
#if DIS_SERVICE_EN
#include "dis_config.h"
#include "dis.h"
#endif
#if BAS_SERVICE_EN
#include "bas_config.h"
#include "bas.h"
#endif
#if RSC_SERVICE_EN
#include "rsc_service.h"
#endif
#if CSC_SERVICE_EN
#include "csc_service.h"
#endif

extern void AppHandleIODriverMessage(BEE_IO_MSG io_driver_msg_recv);
extern TAppResult AppProfileCallback(uint8_t service_id, void *p_data);
extern void TestProfileInit(void);

#ifdef __cplusplus
}
#endif

#endif

