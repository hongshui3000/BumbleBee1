/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

#ifndef _S3C2410_TASK_H_
#define _S3C2410_TASK_H_

/** 
 * \file S3C2410_task.h
 *  Signal definitions for S3C2410 platform specific peudo-task.
 * 
 * \author Santhosh kumar M
 * \date 2006-10-04
 */

#include "platform.h"

/* Signals defined for S3C2410 platform drivers implementation */
#define  UART_TX_ISR_TO_PF_TASK_SIGNAL               5
#define  USB_TX_ISR_TO_PF_TASK_SIGNAL                6
#define  USB_READY_SIGNAL                            7
//#define  LOGUART_TX_ISR_TO_PF_TASK_SIGNAL            8

#ifdef ENABLE_PLATFORM_TASK_HOOK
#define S3C2410_task_send_signal(signal, arg) pf_task_send_signal(signal, arg)
#else
#define S3C2410_task_send_signal(signal, arg)
#endif

#endif /* _S3C2410_TASK_H_ */

