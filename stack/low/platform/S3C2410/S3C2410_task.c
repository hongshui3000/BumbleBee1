/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/** 
 * \file S3C2410_task.c
 *  Pseudo-task for the S3C2410 platform specific drivers. Drivers can
 *  utilize this task for their implementation.
 * 
 * \author Santhosh kumar M
 * \date 2006-10-04
 */
enum { __FILE_NUM__= 105 };

#ifndef NULL_HCI_TRANSPORT

#include "platform.h"
#include "S3C2410_task.h"
#include "S3C2410_transport.h"
#include "uart.h"
#include "logger.h"

#ifdef ENABLE_PLATFORM_TASK_HOOK
//#define S3C2410_task pf_task 

/** 
 * Platform task for S3C2410 Samsung platform. This task is used by the
 * drivers like USB, UART, etc.,
 * 
 * \param signal Type of signal received by this peudo-task.
 * \param arg User provided argument for the signal handler.
 *
 * \return None.
 */
void pf_task (INT32 signal, void* arg)
{
    switch (signal)
    {
        case UART_TX_ISR_TO_PF_TASK_SIGNAL:
			{
#ifdef ENABLE_LOGGER
                MINT_logger_uart_tx();
#endif  // ENABLE_LOGGER 
			}

			break;

        default:
            BZ_ASSERT(0, "Undefined signal was caught by pf_task");
    } // end of switch (signal) 
	
}
#endif /* end of #ifdef ENABLE_PLATFORM_TASK_HOOK */
#endif /* NULL_HCI_TRANSPORT */

