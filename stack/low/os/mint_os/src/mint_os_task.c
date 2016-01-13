/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/
/********************************* Logger *************************/ 
enum { __FILE_NUM__= 66 };
/********************************* Logger *************************/
#include "FreeRTOS.h"
#include "task.h"
#include "bz_log_defines.h"
#include "mint_os_queue_internal.h"
#include "mint_os_task_internal.h"
#include "mint_os_timer_internal.h"
#include "UartPrintf.h"

#ifdef PROFILE_CALC
#include "profiler.h"
#endif


#include "timer.h"
#include "uart.h"
#include "hci_vendor_defines.h"

OS_TASK_MGR                 task_mgr;
OS_HANDLE                   current_task_handle;

extern  OS_IDLE_FUNCTION    idle_func_handler;

#ifdef _TASK_CALLBACK_DBG_
extern UINT16 dbg_hw_int_count;
#endif

#ifdef _BRUCE_WDG_TIMEOUT_DEBUG
SECTION_SRAM extern UINT8 patch_in_cpu_idle;
SECTION_SRAM extern UINT32 patch_isr_cnts_during_one_callback;
SECTION_SRAM extern UINT16 patch_signal_id;
#endif

/**************************************************************************
 * Function     : OS_task_mgmt_init
 *
 * Description  : This function intialises the timer managment component. It 
 *                  primarily initialises the data structures of task mgr.
 *
 * Parameters   : None
 *
 * Returns      : OK
 *
 * Side Effects : None
 *
 *************************************************************************/

OS_STATUS   OS_task_mgmt_init ( void )
{
    UINT32 index = 0;

    task_mgr.max_priority   = 0;
    current_task_handle     = OS_INVALID_HANDLE;

    /* Initialize fiber data structures */
    for ( index = 0; index < OS_TOTAL_TASKS; index++ )
    {
        task_mgr.task_array[index].signal_queue     = OS_INVALID_HANDLE;
        task_mgr.task_array[index].task_function    = NULL;
        task_mgr.task_array[index].task_state       = TASK_NO_TASK;

#ifdef _TASK_CALLBACK_DBG_    
        task_mgr.task_array[index].dbg_max_time = 0;
        task_mgr.task_array[index].dbg_signal_type = 0;
        task_mgr.task_array[index].dbg_int_count = 0;
#endif
    }
    //OS_INF(log_file,"\nOS_task_mgmt_init : success.");
	OS_INF(OS_TASK_MGMT_INIT_SUCCESS,0,0);

    return ( OK );
}

/**************************************************************************
 * Function     : OS_create_task
 *
 * Description  : This function creates a new task in the system. It initialises
 *                  task state, allocates a queue for storing signals which are 
 *                  sent to this task, creates busy and idle period timers. It 
 *                  is assumed that all the tasks are created during system 
 *                  init. Note that a SIGNAL_BEGIN is queued task queue as part 
 *                  of its initialisation.
 *
 * Parameters   : Task name, task priority, task handler, total task queue size
 *                  and busy period timeout value.
 *
 * Returns      : Handle to the task.
 *
 * Side Effects : None
 *
 *************************************************************************/

OS_HANDLE OS_create_task ( char* name, UINT8 priority, OS_TASK_FUNCTION handler,
                           UINT32 task_queue_size, UINT16 busy_period )
{
    MT_OS_TCB  *tcb     = NULL;

    /* Reset error number */

    /* Check for valid arguments */
    if ( name == NULL )
    {
        //OS_ERR(log_file,"\nOS_create_task : Invalid name");
		OS_ERR(OS_CREATE_TASK_INVALID_NAME,0,0);
        return (OS_INVALID_HANDLE);
    }

    if ( priority >= OS_TOTAL_TASKS )
    {
        //OS_ERR(log_file,"\nOS_create_task : Invalid priority");
		OS_ERR(OS_CREATE_TASK_INVALID_PRIORITY,0,0);
        return (OS_INVALID_HANDLE);
    }

    if ( handler == NULL )
    {
        //OS_ERR(log_file,"\nOS_create_task : Invalid task function");
		OS_ERR(OS_CREATE_TASK_INVALID_TASK_FUNCTION,0,0);
        return (OS_INVALID_HANDLE);
    }

    if ( task_queue_size == 0 || task_queue_size > OS_MAX_QUEUE_ITEMS )
    {
        //OS_INF(log_file,"\nOS_create_task : Invalid task queue size");
		OS_INF(OS_CREATE_TASK_INVALID_TASK_QUEUE_SIZE,0,0);
        return (OS_INVALID_HANDLE);
    }

    if ( busy_period == 0 || busy_period > OS_MAX_BUSY_PERIOD )
    {
        //OS_ERR(log_file,"\nOS_create_task : Invalid busy period");
		OS_ERR(OS_CREATE_TASK_INVALID_BUSY_PERIOD,0,0);
        return (OS_INVALID_HANDLE);
    }

    /* Find the right tcb corresponding to this priority */
    tcb = &(task_mgr.task_array[priority]);

    if ( tcb->task_state != TASK_NO_TASK )
    {
        //OS_ERR(log_file,"\nOS_create_task : task with priority %d already exists",priority);
		OS_ERR(OS_CREATE_TASK_TASK_WITH_PRIORITY_ALREADY_EXISTS,1,priority);
        return (OS_INVALID_HANDLE);
    }

    /* 
     * Initialize tcb with the passed arguments 
     */
    tcb->task_function  = (OS_TASK_FUNCTION)handler;
    tcb->task_state     = TASK_IDLE;
    tcb->signal_queue   = OS_create_queue (task_queue_size, priority);  

    if ( tcb->signal_queue == OS_INVALID_HANDLE )
    {
        tcb->task_state = TASK_NO_TASK;
        //OS_ERR(log_file,"\nOS_create_task : task queue creation failed");
		OS_ERR(OS_CREATE_TASK_TASK_QUEUE_CREATION_FAILED,0,0);
		OS_ERR(GRAY, MINT_OS_TASK_176 , 0, 0);
        return (OS_INVALID_HANDLE);
    }

    /* 
     * Update max_priority with task_mgr.
     * We never need to traverse the whole tcb list. Traversing 0 to 
     * max_priority is the dynamic-range of tasks running in the system.
     */
    if ( priority > task_mgr.max_priority )
    {
        task_mgr.max_priority = priority;
    }

    return (priority);
}

/**************************************************************************
 * Function     : OS_reset_task
 *
 * Description  : This function reset an existing task in the system by
 *                clearing all the pending signals for this task.
 *
 * Parameters   : Handle to the task.
 *
 * Returns      : OK/ERROR
 *
 * Side Effects : None
 *
 *************************************************************************/

OS_STATUS  OS_reset_task ( OS_HANDLE task_handle )
{
#if (MINT_OS_ARG_CHK_EN == 1)
    /* check for valid arguments */
    if ( ( task_handle < 0 ) ||
         ( task_handle > OS_TOTAL_TASKS ) ||
         ( task_mgr.task_array[task_handle].task_state == TASK_NO_TASK ) )
    {
        OS_ERR(log_file,"\nOS_reset_task : Invalid task");
        return (ERROR);
    }
#endif

    /* reset task queue */
    if ( OS_reset_queue (
         task_mgr.task_array[task_handle].signal_queue ) == ERROR )
    {
        OS_ERR(log_file,"\\nOS_reset_task : task queue reset failed");
        return (ERROR);
    }
    return (OK);
}



/**************************************************************************
 * Function     : OS_delete_task
 *
 * Description  : This function deletes an existing task in the system. It lets 
 *                  the task do any task handler specific cleanup by sending it
 *                  the signal SIGNAL_END in caller's context. It also deletes 
 *                  the associated signal queue, busy period timer and idle 
 *                  period timer and reset required task mgr data structures.
 *
 * Parameters   : Handle to the task.
 *
 * Returns      : None
 *
 * Side Effects : None
 *
 *************************************************************************/

OS_STATUS  OS_delete_task ( OS_HANDLE task_handle )
{
#ifndef _DONT_SUPPORT_OS_SHUTDOWN_

    MT_OS_TCB   *tcb = NULL;
    OS_SIGNAL end_signal;

#if (MINT_OS_ARG_CHK_EN == 1)
    /* check for valid arguments */
    if ( ( task_handle < 0 ) ||
         ( task_handle > OS_TOTAL_TASKS ) ||
         ( task_mgr.task_array[task_handle].task_state == TASK_NO_TASK ) )
    {
        //OS_ERR(log_file,"\nOS_delete_task : Invalid task");
		OS_ERR(OS_DELETE_TASK_INVALID_TASK,0,0);
        return (ERROR);
    }
#endif

    end_signal.type     = SIGNAL_END;
    end_signal.param    = NULL;
    end_signal.length   = 0;

    /* Call the task with end signal to allow to it do task specific cleanup. */
    tcb = &(task_mgr.task_array[task_handle]);
    tcb->task_function ( &end_signal );  // dead lock, guochunxia 20090701
    /* reset task function pointer */
    tcb->task_function = (OS_TASK_FUNCTION)NULL;

    /* delete task queue */
    if ( OS_delete_queue ( tcb->signal_queue ) == ERROR )
    {
        //OS_ERR(log_file,"\\nOS_delete_task : task queue delete failed");
		OS_ERR(OS_DELETE_TASK_TASK_QUEUE_DELETE_FAILED,0,0);
        return (ERROR);
    }
    tcb->signal_queue = OS_INVALID_HANDLE;

    /* set task state to undefined state - TASK_NO_TASK */
    tcb->task_state = TASK_NO_TASK;

#endif

    return (OK);
}

/**************************************************************************
 * Function     : OS_send_signal_to_task
 *
 * Description  : Sends the given signal to the given task. It also updates the 
 *                  task state of the task which receives the signal.
 *
 * Parameters   : Handle to the task.
 *
 * Returns      : A valid handle if successful, else OS_INVALID_HANDLE.
 *
 * Side Effects : None
 *
 *************************************************************************/
extern xTaskHandle low_task_handle;
OS_STATUS   OS_send_signal_to_task ( OS_HANDLE task_handle, OS_SIGNAL signal )
{
    MT_OS_TCB      *tcb = NULL;

#if (MINT_OS_ARG_CHK_EN == 1)
    /* check for valid arguments */
    if ( task_handle < 0 || task_handle > OS_TOTAL_TASKS )
    {
        //OS_ERR(log_file,"\nOS_send_signal_to_task : Invalid handle");
		OS_ERR(OS_SEND_SIGNAL_TO_TASK_INVALID_HANDLE,0,0);
        return (ERROR);
    }
#endif

    /* get the tcb for the task */
    tcb = &(task_mgr.task_array[task_handle]);

    /* enqueue signal item to the task's signal queue */
    if ( OS_enqueue_item ( tcb->signal_queue, (OS_ADDRESS)&signal , TRUE)
         == ERROR )
    {
        //OS_INF(log_file,"\nOS_send_signal_to_task : signal enqueue failed");
		OS_INF(OS_SEND_SIGNAL_TO_TASK_SIGNAL_ENQUEUE_FAILED,0,0);
        return (ERROR);
    }
#ifdef USE_FREERTOS
    if (IN_ISR())
    {
        BaseType_t high_pri_task_woken = pdFALSE;
        vTaskNotifyGiveFromISR(low_task_handle, &high_pri_task_woken);
        portYIELD_FROM_ISR(high_pri_task_woken);
    }
    else
    {
        xTaskNotifyGive(low_task_handle);
    }
#endif

    return (OK);
}

/**************************************************************************
 * Function     : OS_send_signal_to_task_head
 *
 * Description  : Sends the given signal to the head of given task. It also 
 *                  updates the task state of the task which receives the 
 *                  signal.
 *
 * Parameters   : Handle to the task.
 *
 * Returns      : A valid handle if successful, else OS_INVALID_HANDLE.
 *
 * Side Effects : None
 *
 *************************************************************************/

OS_STATUS   OS_send_signal_to_task_head ( OS_HANDLE task_handle, OS_SIGNAL signal )
{
    MT_OS_TCB      *tcb = NULL;

#if (MINT_OS_ARG_CHK_EN == 1)
    /* check for valid arguments */
    if ( task_handle < 0 || task_handle > OS_TOTAL_TASKS )
    {
        //OS_ERR(log_file,"\nOS_send_signal_to_task : Invalid handle");
		OS_ERR(OS_SEND_SIGNAL_TO_TASK_INVALID_HANDLE,0,0);
        return (ERROR);
    }
#endif

    /* get the tcb for the task */
    tcb = &(task_mgr.task_array[task_handle]);

    /* enqueue signal item to the task's signal queue */
    if ( OS_enqueue_item ( tcb->signal_queue, (OS_ADDRESS)&signal , FALSE)
         == ERROR )
    {
        //OS_INF(log_file,"\nOS_send_signal_to_task : signal enqueue failed");
		OS_INF(OS_SEND_SIGNAL_TO_TASK_SIGNAL_ENQUEUE_FAILED,0,0);
        return (ERROR);
    }
#ifdef USE_FREERTOS
    if (IN_ISR())
    {
        BaseType_t high_pri_task_woken = pdFALSE;
        vTaskNotifyGiveFromISR(low_task_handle, &high_pri_task_woken);
        portYIELD_FROM_ISR(high_pri_task_woken);
    }
    else
    {
        xTaskNotifyGive(low_task_handle);
    }
#endif

    return (OK);
}

#ifdef USE_FREERTOS
void OS_process_task_signal(void)
{
    while (1)
    {
        OS_HANDLE i;
        OS_HANDLE handle = OS_INVALID_HANDLE;

        for (i = 0; i <= task_mgr.max_priority; ++i)
        {
            MT_OS_TCB *tcb = &(task_mgr.task_array[i]);

            if (tcb->task_state == TASK_NO_TASK)
            {
                continue;
            }

            if (OS_queue_get_num_of_items(tcb->signal_queue) > 0)
            {
                handle = i;
                break;
            }
        }

        WDG_TIMER_RESTART;
        BTON_WDG_TIMER_RESTART;

        if (handle == OS_INVALID_HANDLE)
        {
            break;
        }

        OS_SIGNAL signal;
        MT_OS_TCB *tcb = &task_mgr.task_array[handle];
        if (OS_remove_item(tcb->signal_queue, (OS_ADDRESS) &signal) == OK)
        {
            tcb->task_function(&signal);
        }
    }
}
#else /* ! USE_FREERTOS */
/**************************************************************************
 * Function     : OS_scheduler
 *
 * Description  : 
 *              This function implements a simple scheduler which selects a 
 *              task which should be run and dispatches it in a loop. Note that
 *              this function never returns and should be the last function
 *              that an application should call.
 *
 * Parameters   : Handle to the task.
 *
 * Returns      : A valid handle if successful, else OS_INVALID_HANDLE.
 *
 * Side Effects : None
 *
 *************************************************************************/
void        OS_scheduler ( void )
{
    MT_OS_TCB   *tcb;
    MT_OS_TCB   *curr_tcb;
    OS_SIGNAL   signal;
    OS_STATUS   status = ERROR;
    OS_HANDLE   task_handle_index;

    while ( 1 )
    {
        /* Find next task to run. */

        /* Reset variables which will contain search results */
        current_task_handle = OS_INVALID_HANDLE;

        for ( task_handle_index = 0; task_handle_index <= task_mgr.max_priority;
              task_handle_index++ )
        {
            tcb = &(task_mgr.task_array[task_handle_index]);

            /* Check if its a task */
            if ( tcb->task_state == TASK_NO_TASK )
            {
                continue;
            }

            /* Check if it has a signal pending */
            if ( OS_queue_get_num_of_items ( tcb->signal_queue ) != 0 )
            {
                current_task_handle = task_handle_index;
                break;
            }
        }

        WDG_TIMER_RESTART;          

        BTON_WDG_TIMER_RESTART;

#ifdef _DONT_USE_LOG_UART_TX_INT_
        UINT8 msg = FALSE;
#ifdef _DAPE_TEST_CORRECT_HCI_LOG
        if ((dbg_vendor_log_interface == VENDOR_LOG_PACKET_TYPE_INVALID)
               && (dbg_vendor_log_conn_handle == 0))                   
#endif
        {
            msg = uart_print_buf();
        }

        if ( current_task_handle == OS_INVALID_HANDLE )
        {
            if (( idle_func_handler != NULL ) && !msg)
            {
                idle_func_handler();
            }
            continue;
        }
#else
        /* Check if we found a task ready to get dispatched */
        if ( current_task_handle == OS_INVALID_HANDLE )
        {
            if ( idle_func_handler != NULL )
            {
            
#ifdef _BRUCE_WDG_TIMEOUT_DEBUG
                patch_in_cpu_idle = 1;
#endif            
                idle_func_handler();

#ifdef _BRUCE_WDG_TIMEOUT_DEBUG
                patch_in_cpu_idle = 0;
#endif

            }
            continue;
        }
#endif

        curr_tcb = &(task_mgr.task_array[current_task_handle]);
        status = OS_remove_item ( curr_tcb->signal_queue, (OS_ADDRESS)&signal );

        if (status == OK )
        {
#ifdef _TASK_CALLBACK_DBG_
            reset_vendor_counter();
            dbg_hw_int_count = 0;
#endif

#ifdef _BRUCE_WDG_TIMEOUT_DEBUG
            patch_isr_cnts_during_one_callback = 0;
            patch_signal_id = signal.type; 
#endif

            curr_tcb->task_function ( &signal );

#ifdef _BRUCE_WDG_TIMEOUT_DEBUG
            patch_signal_id = 0xffff;
            patch_isr_cnts_during_one_callback = 0;
#endif

#ifdef _TASK_CALLBACK_DBG_
            UINT32 time = read_vendor_counter_no_display();
            if (time > curr_tcb->dbg_max_time)
            {
                /* we record worse task callback path to help analyze 
                   in the future */
                curr_tcb->dbg_max_time = time;
                curr_tcb->dbg_signal_type = signal.type;
                curr_tcb->dbg_int_count = dbg_hw_int_count;
            }
#endif
        }

    }
}
#endif /* USE_FREERTOS */

