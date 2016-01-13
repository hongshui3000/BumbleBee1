/**
************************************************************************************************************
*               Copyright(c) 2014-2015, Realtek Semiconductor Corporation. All rights reserved.
************************************************************************************************************
* @file     board.h
* @brief    
* @author   
* @date     
* @version  
*************************************************************************************************************
*/

#ifndef __BOARD_H__
#define __BOARD_H__

#include "rtl_types.h"
#include "trace.h"
#include "FreeRTOS.h"
#include "task.h"
#include "rtl876x_log_uart.h"
#include "rtl876x_uart.h"
#include "osif.h"
#include "print_howie.h"
#include "rtl876x_rcc.h"

#include "rtl_types.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "trace.h"
#include "rtl876x_gpio.h"

/* Defines ------------------------------------------------------------------*/
#define TEST_Pin            5
#define GPIO_Test_Pin       GPIO_GetPin(TEST_Pin)

//#define TIM_ID                  TIM2
//#define GPIO_Test_Pin       0xffffffff


#endif /* __BOARD_H__ */
