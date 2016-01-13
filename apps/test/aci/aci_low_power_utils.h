/**
*********************************************************************************************************
*               Copyright(c) 2014, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file      aci_low_power_utils.h
* @brief     util fucntions used in ltp aci low power.
* @details   none.
* @author    tifnan
* @date      2014-11-22
* @version   v0.1
* *********************************************************************************************************
*/

#ifndef     _LTP_LOW_POWER_UTILS_H_
#define     _LTP_LOW_POWER_UTILS_H_

#include "ltplib.h"
/** @brief used to save data when bee wakes up and is wait for host ack */
typedef struct _LTPLL_BufMsg
{
    struct _LTPLL_BufMsg* p_next;
    uint8_t* buf_addr;
    uint32_t size;
} LTPLL_BufMsg, *PLTPLL_BufMsg;

/* export functions */
BOOL LTPLL_QueueSave(uint8_t* addr, uint32_t size);
BOOL LTPLL_QueueIsEmpty(void);
uint16_t LTPLL_GetQueueSize(void);
PLTPLL_BufMsg LTPLL_QueueOut(void);

#endif
