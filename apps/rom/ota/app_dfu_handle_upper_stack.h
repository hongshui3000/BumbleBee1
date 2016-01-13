/*
 *  Routines to access hardware
 *
 *  Copyright (c) 2014 Realtek Semiconductor Corp.
 *
 *  This module is a confidential and proprietary property of RealTek and
 *  possession or use of this module requires written permission of RealTek.
 */


#ifndef _APP_DFU_HANDLE_UPPER_STACK_H_
#define  _APP_DFU_HANDLE_UPPER_STACK_H_
#include "rtl_types.h"
#include "blueapi_types.h"
#include "app_dfu_int.h"


void dfu_HandleBlueAPIMessage( TDFU_CB * pDfuCb, PBlueAPI_UsMessage pMsg);



#endif
