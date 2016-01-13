/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file      test_peripheral_application.h
* @brief     test_application app implementation
* @details   test_application app implementation
* @author    ethan_su
* @date      2015-06-24
* @version   v0.1
* *********************************************************************************************************
*/
#ifndef _TEST_APPLICATION__
#define _TEST_APPLICATION__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "bee_message.h"

/* We can add different services for peripheral role test. */
#define SIMPLE_SERVICE_EN   0
#define DIS_SERVICE_EN      0
#define BAS_SERVICE_EN      0
#define RSC_SERVICE_EN      1
#define CSC_SERVICE_EN      1

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
extern TAppResult AppProfileCallback(uint8_t serviceID, void* pData);

#ifdef __cplusplus
}
#endif

#endif

