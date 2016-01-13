/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

#ifndef _H_OS_
#define _H_OS_

/* Include bt_fw header files */
#include "bt_fw_errors.h"
#include "bt_fw_signals.h"
#include "bt_fw_types.h"
#include "bt_fw_tasks.h"

/* Macro definitions */

/* Critical section handler primitives */
#define DEF_CRITICAL_SECTION_STORAGE         pf_define_critical
#define MINT_OS_ENTER_CRITICAL()             pf_enter_critical()
#define MINT_OS_EXIT_CRITICAL()              pf_exit_critical()

#define     OS_TOTAL_QUEUES         OS_TOTAL_TASKS
#define     OS_MAX_QUEUE_ITEMS      (100)
#define     OS_MAX_STACK_ITEMS      (300)

#define     OS_TASK_SIG_LOST_MAX_CNT    8 

// deleted by guochunxia for DUAL_APP
//#ifdef DUAL_APP
//#   define     OS_MAX_TIMER_BUCKETS    (16) /* Make sure this is 2^n */
//#else
#define     OS_MAX_TIMER_BUCKETS    (0x40) /* Make sure this is 2^n */
//#endif

#define     OS_MAX_BUSY_PERIOD      (100)

/* Type definitions for OS module */
#undef      OK
#undef      ERROR
#define     OK      ((OS_STATUS)0)
#define     ERROR   ((OS_STATUS)-1)

#undef      TRUE
#undef      FALSE

#define     TRUE    ((OS_BOOL)1)
#define     FALSE   ((OS_BOOL)0)

typedef     unsigned char   OS_BOOL;
typedef     int             OS_STATUS;
typedef     int             OS_HANDLE;
typedef     void*           OS_ADDRESS;

/* Exported data structures */
/**
 * \addtogroup mint_os
 * @{ */

/**
 * Signal structure used to send signals to tasks.
 * (austin: please let the size of OS_SIGNAL be multiple of 4 to avoid 
 *  memory heap error !!)
 *
 * \sa OS_SEND_SIGNAL_TO_TASK.
 */
typedef struct  os_signal
{
    OS_ADDRESS  param;      /**< User defined parameter1 */
    OS_ADDRESS ext_param;   /**< User defined parameter2 */
    UINT16     type;        /**< Type of the signal (eg: SIGNAL_BEGIN) */
    UINT16     length;
    UINT8      fifo_index;
    UINT8      fifo_type;
    UINT8      reserved[2];
} OS_SIGNAL;

/** @} end: mint_os */

#define MEM_POOL_ITEM_SZ sizeof(OS_ADDRESS)
#define OS_Q_ITEM_SIZE   sizeof(OS_SIGNAL)
 
/* Function types */
typedef     void    (*OS_IDLE_FUNCTION) ( void );

/**
 * \addtogroup mint_os
 * @{ */

/**
 * Signature of the MINT OS Task. Any MINT OS task should define a function
 * with this signature.
 *
 * \param[in]  signal Received signal by the task.
 *
 * \return None.
 */
typedef     void    (*OS_TASK_FUNCTION) ( OS_SIGNAL * );
/**
 * Timeout handler signature. Any timer created using MINT OS timer API should
 * define the timeout handler with this signature.
 *
 * \param[in] handle Handle of the timer fired.
 * \param[in] arg User provided argument during the timer creation.
 *
 * \return None.
 */
typedef     void    (*OS_TIMEOUT_HANDLER) ( OS_HANDLE handle, void* arg);

/** @} end: mint_os */


/* Macros for error cases */
#define  OS_INVALID_HANDLE      ((OS_HANDLE)-1)
#define  OS_INVALID_VALUE       -1


#ifdef USE_FREERTOS
#include "FreeRTOS.h"
/**
 * @name Memory management interface.
 *
 * os_heap_init() must be called before any other memory management related
 * functions are called.
 */
/**@{*/

void os_heap_init();

/**
 * @brief Allocate memory from heap.
 *
 * This function can be used only after os_heap_init() is called.\n
 * This function is NOT thread-safe and NOT reentrant.
 *
 * @param bytes  Size to allocate.
 * @param mem_mode  Memory type.
 * @return  Allocated memory address, or \c NULL on error.
 */
#define os_malloc(bytes, mem_mode) pvPortMalloc(mem_mode, bytes)

/**
 * @brief Free memory allocated from heap.
 *
 * This function can be used only after os_heap_init() is called.\n
 * This function is NOT thread-safe and NOT reentrant.
 *
 * @param buf  Buffer to free.
 * @param mem_mode  Memory type.
 */
#define os_free(buf, mem_mode) vPortFree(mem_mode, buf)

#define os_qalloc(bytes) pvPortMalloc(RAM_TYPE_DATA_ON, bytes)
/**@}*/
#else
void *os_malloc(unsigned int bytes, UINT8 mem_mode);
void *os_qalloc(unsigned int bytes);
#endif

/* Init and Shutdown APIs */
OS_STATUS   OS_init ( OS_IDLE_FUNCTION  idle_handler );

/* Task Management APIs */
OS_HANDLE   OS_create_task (char* name, UINT8 priority, OS_TASK_FUNCTION handler,
                            UINT32 task_queue_size, UINT16 busy_period);

OS_STATUS        OS_reset_task ( OS_HANDLE task_handle );

OS_STATUS   OS_delete_task ( OS_HANDLE  );

OS_STATUS   OS_send_signal_to_task ( OS_HANDLE task, OS_SIGNAL signal );

OS_STATUS   OS_send_signal_to_task_head ( OS_HANDLE task, OS_SIGNAL signal );

#ifdef USE_FREERTOS
void OS_process_task_signal(void);
#else
void        OS_scheduler ( void );
#endif

/* Buffer Management APIs */
OS_HANDLE   OS_create_pool ( UINT32 chunk_size, UINT32 num_chunks,
				UINT8 m_alignment, UINT8 m_compensate, RAM_TYPE mem_mode);

OS_STATUS   OS_reset_pool ( OS_HANDLE pool );

OS_STATUS   OS_delete_pool ( OS_HANDLE pool );

OS_ADDRESS  OS_allocate_buffer ( OS_HANDLE pool );

OS_STATUS   OS_deallocate_buffer ( OS_HANDLE pool, OS_ADDRESS address );

UINT32 OS_get_free_buffer(OS_HANDLE pool_id);


#endif /* _H_OS_ */


