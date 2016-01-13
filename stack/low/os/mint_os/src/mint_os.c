/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/
/********************************* Logger *************************/ 
enum { __FILE_NUM__= 62 };
/********************************* Logger *************************/
#include "mint_os.h"
#include "bz_log_defines.h"
#include "mint_os_queue_internal.h"
#ifndef USE_FREERTOS
#include "mint_os_timer_internal.h"
#endif
#include "mint_os_buffer_internal.h"
#include "mint_os_task_internal.h"
#include "platform.h"
#include "le_ll.h"

#ifndef USE_FREERTOS
OS_HANDLE                       timer_task_handle;
#endif
OS_HANDLE                       isr_extended_task_handle;
OS_IDLE_FUNCTION                idle_func_handler;

#ifndef USE_FREERTOS
UINT16                          heap_pointer = 0;
ALIGN(4) UINT8 SECTION_HEAP     heap_memory[OS_MAX_HEAP_MEMORY];

UINT16                          heap_dmem_pointer = 0;
ALIGN(4) UINT8                  heap_dmem_memory[OS_MAX_HEAP_DMEM_MEMORY];

#ifdef _ENABLE_RETENTION_FLOW_FOR_DLPS_
UINT16                          heap_partial_off_pointer = 0;
ALIGN(4) UINT8 SECTION_LOW_HEAP heap_partial_off_memory[OS_MAX_HEAP_MEMORY_PARTIAL_OFF];
#endif

UINT16	                        queue_memory_pointer = 0;
ALIGN(4) UINT8                  queue_memory[OS_MAX_QUEUE_MEMORY];
#endif

extern void isr_extended_task_function(OS_SIGNAL * signal);

/**************************************************************************
 * Function     : OS_init
 *
 * Description  : Initialises the timer, queue, buffer and task management
 *                  components of OS module and creates a timer task which
 *                  interfaces with timer ISR and shoots s/w timers expiring
 *                  at the current time. All the APIs exported by OS module
 *                  available only after a call to this function.
 *
 * Parameters   : None.
 *
 * Returns      : OK
 *
 * Side Effects : None
 *
 *************************************************************************/

OS_STATUS   OS_init ( OS_IDLE_FUNCTION idle_handler )
{
#ifdef USE_FREERTOS
    os_heap_init();
#else
    heap_pointer    = 0;
    heap_dmem_pointer = 0;
    queue_memory_pointer = 0;
    
#ifdef _ENABLE_RETENTION_FLOW_FOR_DLPS_
    heap_partial_off_pointer = 0;
#endif
#endif

    /* 
     * Initialise function which should be called during scheduler
     * idling. On non-OS versions, typically, idle handler would be
     * NULL which makes scheduler do a busy wait, till a OS task gets 
     * an external event and its ready to run again.
     * 
     * On plaforms with OS versions, idle handler function can block 
     * for a message from another platfrom specific task or ISRs, assuming
     * that the OS module tasks have nothing to do and all internal events
     * are triggered by an external event.
     */
    idle_func_handler = idle_handler;

    /* Initialize queue management */
    OS_queue_mgmt_init();

#ifndef USE_FREERTOS
    /* Initialize timer management */
    OS_timer_mgmt_init();
#endif

    /* Initialize buffer management */
    OS_buffer_mgmt_init();

    /* Initialize task management */
    OS_task_mgmt_init();

    /*
     * Create all system tasks here 
     */

    /* Create ISR extended task */
    isr_extended_task_handle = OS_create_task ( ISR_EXTENDED_TASK_NAME, 
                                                ISR_EXTENDED_TASK_PRI, 
                                                isr_extended_task_function, 
                                                ISR_EXTENDED_TASK_Q_SIZE, 5 );

    if ( isr_extended_task_handle == OS_INVALID_HANDLE )
    {
        //OS_ERR(log_file,"\n OS_init : failed!");
		OS_ERR(OS_INIT_FAILED,0,0);
        return (ERROR);
    }

#ifndef USE_FREERTOS
    /* Create timer task */
    timer_task_handle = OS_create_task ( TIMER_TASK_NAME, TIMER_TASK_PRI, 
    OS_timer_task_function, TIMER_TASK_Q_SIZE, 5 );

    if( timer_task_handle == OS_INVALID_HANDLE )
    {
        //OS_ERR(log_file,"\n OS_init : failed!");
		OS_ERR(OS_INIT_FAILED,0,0);
        return (ERROR);
    }
#endif

    //OS_INF(log_file,"\nOS_init : successful.");
	OS_INF(OS_INIT_SUCCESSFUL,0,0);

    return (OK);
}

#ifndef USE_FREERTOS
/**************************************************************************
 * Function     : os_malloc
 *
 * Description  : This provides interface for malloc on platforms where malloc
 *                  is not available. It actually allcoates memory from a 
 *                  statically allocated buffer. It is assumed that the memory 
 *                  is not freed dynamically.
 *
 * Parameters   : Number of bytes of memory required.
 *
 * Returns      : Start address of allocated memory.
 *
 * Side Effects : None
 *
 *************************************************************************/
void*   os_malloc ( unsigned int bytes, UINT8 mem_mode)
{
    void *free_address = NULL;

    /* Check for valid arguments */
    if (bytes == 0 )
    {
        return (NULL);
    }

    if (mem_mode == MEM_SRAM_MODE)
    {
        /* let heap_pointer is 4-byte aligned */
        heap_pointer = (heap_pointer + 3) & ~0x03;

        if ((heap_pointer + bytes) <= OS_MAX_HEAP_MEMORY )
        {
            free_address = (void*)(&heap_memory[heap_pointer]);
            heap_pointer += bytes;
        }
#ifdef _MEMORY_DBG_
        else
        {
            RT_BT_LOG(RED, MINT_OS_186, 0, 0);
        }
#endif        
    }

    if (mem_mode == MEM_DMEM_MODE)
    {
        /* let heap_pointer is 4-byte aligned */
        heap_dmem_pointer = (heap_dmem_pointer + 3) & ~0x03;

        if ((heap_dmem_pointer + bytes) <= OS_MAX_HEAP_DMEM_MEMORY )
        {
            free_address = (void*)(&heap_dmem_memory[heap_dmem_pointer]);
            heap_dmem_pointer += bytes;
        }
#ifdef _MEMORY_DBG_
        else
        {
            RT_BT_LOG(RED, MINT_OS_186, 0, 0);
        }
#endif        
    }

#ifdef _ENABLE_RETENTION_FLOW_FOR_DLPS_
    if (mem_mode == MEM_SRAM_PARTIAL_OFF_MODE)
    {
        /* let heap_pointer is 4-byte aligned */
        heap_partial_off_pointer = (heap_partial_off_pointer + 3) & ~0x03;

        if ((heap_partial_off_pointer + bytes) <= OS_MAX_HEAP_MEMORY_PARTIAL_OFF )
        {
            free_address = (void*)(&heap_partial_off_memory[heap_partial_off_pointer]);
            heap_partial_off_pointer += bytes;
        }
#ifdef _MEMORY_DBG_
        else
        {
            RT_BT_LOG(RED, MINT_OS_186, 0, 0);
        }
#endif        
    }
#endif

    return ( free_address );
}

/**************************************************************************
 * Function     : os_qalloc
 *
 * Description  : This provides interface for malloc on platforms where malloc
 *                  is not available. It actually allcoates memory from a 
 *                  statically allocated buffer. It is assumed that the memory 
 *                  is not freed dynamically.
 *
 * Parameters   : Number of bytes of memory required.
 *
 * Returns      : Start address of allocated memory.
 *
 * Side Effects : None
 *
 *************************************************************************/
void*   os_qalloc( unsigned int bytes )
{
	void *free_address = NULL;
	  
    /* Check for valid arguments */
    if(bytes == 0 )
    {
        return (NULL);
    }

    /* let queue pointer is 4-byte aligned */
    queue_memory_pointer = (queue_memory_pointer + 3) & ~0x03;

    if ((queue_memory_pointer + bytes) <= OS_MAX_QUEUE_MEMORY )
    {
        free_address = (void*)(&queue_memory[queue_memory_pointer]);
        queue_memory_pointer += bytes;
    }    
#ifdef _MEMORY_DBG_
    else
    {
    	RT_BT_LOG(RED, MINT_OS_230, 0, 0);
    }
#endif

    return ( free_address );
}
#endif

