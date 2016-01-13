enum { __FILE_NUM__ = 0 };

/*
 *  Routines to access hardware
 *
 *  Copyright (c) 2014 Realtek Semiconductor Corp.
 *
 *  This module is a confidential and proprietary property of RealTek and
 *  possession or use of this module requires written permission of RealTek.
 */

#include "app_dfu_api.h"
#include "app_dfu_int.h"

void dfu_API_Init(TDFU_CB* pDfuCb)
{
    dfu_sm_event(pDfuCb, DFU_EVT_INIT_REG_UPPERSTACK, NULL);
}

