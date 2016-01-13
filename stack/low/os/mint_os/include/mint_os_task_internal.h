/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

#ifndef _H_OS_TASK_
#define _H_OS_TASK_

#include "mint_os.h"
//#include "mint_os_queue_internal.h"

#define     OS_IDLE_PERIOD_TIMER_VAL    (10)


typedef enum    task_event
{
    TASK_NEW_SIGNAL_EVENT,
    TASK_BUSY_TIMEOUT_EVENT,
    TASK_IDLE_TIMEOUT_EVENT,
    TASK_DISPATCHED_EVENT,
    TASK_NO_SIGNAL_EVENT,       /* sent by dispatcher */
    TASK_YIELDED_EVENT

} OS_TASK_EVENT;


typedef enum    task_state
{
    TASK_NO_TASK,
    TASK_IDLE,
    TASK_READY,
    TASK_BUSY,
    TASK_E_IDLE,
    TASK_E_READY,
    TASK_E_BUSY

} OS_TASK_STATE;


/* Task Control Block */
typedef struct  tcb
{
    OS_TASK_FUNCTION    task_function;
    OS_HANDLE           signal_queue;
    OS_TASK_STATE       task_state;
#ifdef _TASK_CALLBACK_DBG_        
    UINT32              dbg_max_time;
    UINT16              dbg_signal_type; 
    UINT16              dbg_int_count;
#endif
} MT_OS_TCB;

/* Task Manager */
typedef struct os_task_mgr
{
    MT_OS_TCB              task_array[OS_TOTAL_TASKS];
    UINT8               max_priority;
} OS_TASK_MGR;

OS_STATUS   OS_task_mgmt_init ( void );

#endif /* _H_OS_TASK_ */

