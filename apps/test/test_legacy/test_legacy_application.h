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
#ifndef __TEST_LEGACY_APPLICATION__
#define __TEST_LEGACY_APPLICATION__

#ifdef __cplusplus
extern "C" {
#endif

#include "bee_message.h"
#include "bterrcod.h"

extern void AppHandleIODriverMessage(BEE_IO_MSG io_driver_msg_recv);
extern TAppResult AppProfileCallback(uint8_t service_id, void *p_data);
extern void TestProfileInit(void);

#ifdef __cplusplus
}
#endif

#endif

