/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/
/********************************* Logger *************************/ 
enum { __FILE_NUM__= 61 };
/********************************* Logger *************************/
#include "bt_fw_os.h"
#include "mint_os_timer_internal.h"
#include "platform.h"
#include "bz_log_defines.h"

/* Buffer Management API */

POOL_ID dma_rx_pool_id;
POOL_ID dma_tx_pool_id;

static UINT8 os_reserved_buffers = 0;

static UINT8 le_os_reserved_buffers = 0;

/**************************************************************************
 * Function   : os_wrapper_create_pool
 *
 * Description: Wrapper for pool creation
 *
 * Parameters : pool id, num of buffer and buffer size.
 *
 * Returns    : API_RESULT
 *
 * Side Effects : None
 *
 *************************************************************************/
API_RESULT   os_wrapper_create_pool ( POOL_ID *p_pool_id, unsigned int num_buf,
                                            unsigned int buf_size, UINT8 m_alignment, 
                                            UINT8 m_compensate, RAM_TYPE mem_mode)
{
    /* extend the buffer size in 4-bytes uint */
    buf_size = (buf_size + 3) & ~0x03;

    *p_pool_id = OS_create_pool ( buf_size, num_buf, m_alignment, m_compensate, mem_mode);

    if ( *p_pool_id == OS_INVALID_HANDLE )
    {
        return BT_ERROR;
    }
    else
    {
        return BT_ERROR_OK;
    }
}


/**************************************************************************
 * Function   : os_wrapper_reset_pool
 *
 * Description: Wrapper for pool reset
 *
 * Parameters : pool id
 *
 * Returns    : API_RESULT
 *
 * Side Effects : None
 *
 *************************************************************************/

API_RESULT    os_wrapper_reset_pool  ( POOL_ID *pool_id )
{
    OS_STATUS   status;

    status  = OS_reset_pool ( *pool_id );

    if ( status == ERROR )
    {
        return BT_ERROR;
    }

    return BT_ERROR_OK;
}


/**************************************************************************
 * Function   : os_wrapper_delete_pool
 *
 * Description: Wrapper for pool deletion
 *
 * Parameters : pool id
 *
 * Returns    : API_RESULT
 *
 * Side Effects : None
 *
 *************************************************************************/

API_RESULT os_wrapper_delete_pool (POOL_ID pool_id )
{
    OS_STATUS   status;

    status  = OS_delete_pool ( pool_id );

    if ( status == ERROR )
    {
        return BT_ERROR;
    }

    return BT_ERROR_OK;
}



/**************************************************************************
 * Function   : os_wrapper_alloc_buffer
 *
 * Description: Wrapper for buffer allocation from a specified pool.
 *
 * Parameters : pool id and address where allocated buffer address is returned.
 *
 * Returns    : API_RESULT
 *
 * Side Effects : None
 *
 *************************************************************************/

API_RESULT    os_wrapper_alloc_buffer ( POOL_ID pool_id, void **pp_buffer )
{
    if ( pp_buffer == NULL )
    {
        return  (BT_ERROR_INVALID_PARAMETER);
    }

    DEF_CRITICAL_SECTION_STORAGE;

    MINT_OS_ENTER_CRITICAL();

    *pp_buffer  = OS_allocate_buffer ( pool_id );

    MINT_OS_EXIT_CRITICAL();

    if ( *pp_buffer == NULL )
    {
		//HCI_LOG_ERROR(LOG_LEVEL_HIGH,"Buffer Allocation Failed Pool: %d",pool_id);
		//HCI_LOG_ERROR(LOG_LEVEL_HIGH, BUFFER_ALLOCATION_FAILED_POOL, 1, pool_id);
        return BT_ERROR;
    }
    else
    {
        return BT_ERROR_OK;
    }
}


/**************************************************************************
 * Function   : os_wrapper_free_buffer
 *
 * Description: Wrapper for freeing buffer from a specified pool. 
 * NOTE       : The linear search may be optimized. Since free buffer count
 *              is USEFUL for only acl data buffers, we CAN do a comparison
 *              for this, and selectively update it. This is a realistic
 *              assumption since flow control is done only for acl packets.
 *
 * Parameters : pool id and buffer address. 
 *
 * Returns    : API_RESULT
 *
 * Side Effects : None
 *
 *************************************************************************/

API_RESULT    os_wrapper_free_buffer ( POOL_ID pool_id, void *p_buffer )
{
    OS_STATUS   status;
    API_RESULT result = BT_ERROR_OK;

    if (p_buffer == NULL)
    {
        return BT_ERROR_OK;
    }

    DEF_CRITICAL_SECTION_STORAGE;
    MINT_OS_ENTER_CRITICAL();    

    status = OS_deallocate_buffer ( pool_id, p_buffer );

    if ( status == ERROR )
    {
        result = BT_ERROR;
    }
    else
    {
	    p_buffer = NULL;
    }
    	
    MINT_OS_EXIT_CRITICAL();    
	
    return result;
}


/**************************************************************************
 * Function   : os_wrapper_get_free_buffer
 *
 * Description: Wrapper for getting the number of free buffers in a pool.
 *
 * Parameters : Pool identifier
 *
 * Returns    : Number of buffers in the pool
 *
 *************************************************************************/

UINT32 os_wrapper_get_free_buffer(POOL_ID pool_id)
{
    return OS_get_free_buffer(pool_id);
}

/**************************************************************************
 * Function   : os_wrapper_create_timer
 *
 * Description: Wrapper for creating a timer
 *
 * Parameters : timer type, address where timer id is returned, timer function,
 *              timer argument and timer period value.
 *
 * Returns    : API_RESULT
 *
 * Side Effects : None
 *
 *************************************************************************/

API_RESULT os_wrapper_create_timer(unsigned char ltimer_type,
        TimerHandle_t *timer_id, TimerCallbackFunction_t timer_func, void* arg,
        unsigned long value)
{
    TickType_t ticks = pdMS_TO_TICKS(value);
    if (ticks <= 0)
    {
        ticks = 1;
    }
    *timer_id = xTimerCreate(NULL, ticks,
            ((OS_TIMER_TYPE) ltimer_type == PERIODIC), arg, timer_func);
    if (*timer_id == NULL)
    {
        return BT_ERROR;
    }
    return BT_ERROR_OK;
}

/**************************************************************************
 * Function   : os_wrapper_delete_timer
 *
 * Description: Wrapper for deleting a timer
 *
 * Parameters : timer id
 *
 * Returns    : API_RESULT
 *
 * Side Effects : None
 *
 *************************************************************************/
API_RESULT os_wrapper_delete_timer(TimerHandle_t* timer_id)
{
    API_RESULT result = BT_ERROR;
    TimerHandle_t timer = *timer_id;
    if (timer != NULL && xTimerDelete(timer, 0) == pdPASS)
    {
        *timer_id = NULL;
        result = BT_ERROR_OK;
    }
    return result;
}

/** 
 * Wrapper for starting a timer.
 * 
 * \param timer_id ID of the timer to start.
 * \param value The timeout value in ms (milli-seconds).
 * 
 * \return API_SUCCESS, on successful operation. API_FAILURE, otherwise.
 */
API_RESULT os_wrapper_start_timer(TimerHandle_t timer_id, unsigned long value)
{
    API_RESULT ret = BT_ERROR;
    TickType_t ticks = pdMS_TO_TICKS(value);
    if (ticks <= 0)
    {
        ticks = 1;
    }
    if (IN_ISR())
    {
        if (xTimerChangePeriodFromISR(timer_id, ticks, NULL) == pdPASS)
        {
            BaseType_t high_pri_task_woken = pdFALSE;
            if (xTimerStartFromISR(timer_id, &high_pri_task_woken) == pdPASS)
            {
                ret = BT_ERROR_OK;
                portYIELD_FROM_ISR(high_pri_task_woken);
            }
        }
    }
    else
    {
        if (xTimerChangePeriod(timer_id, ticks, 0) == pdPASS)
        {
            if (xTimerStart(timer_id, 0) == pdPASS)
            {
                ret = BT_ERROR_OK;
            }
        }
    }
    return ret;
}

/**************************************************************************
 * Function   : os_wrapper_stop_timer
 *
 * Description: Wrapper for stopping a timer
 *
 * Parameters : timer id and address where residual period is returned.
 *
 * Returns    : API_RESULT
 *
 * Side Effects : None
 *
 *************************************************************************/
API_RESULT os_wrapper_stop_timer(TimerHandle_t timer_id, unsigned short *value)
{
    API_RESULT ret = BT_ERROR;
    if (IN_ISR())
    {
        BaseType_t high_pri_task_woken = pdFALSE;
        if (xTimerStopFromISR(timer_id, &high_pri_task_woken) == pdPASS)
        {
            ret = BT_ERROR_OK;
            portYIELD_FROM_ISR(high_pri_task_woken);
        }
    }
    else
    {
        if (xTimerStop(timer_id, 0))
        {
            ret = BT_ERROR_OK;
        }
    }
    return ret;
}

/**************************************************************************
 * Function   : os_wrapper_is_timer_running
 *
 * Description: Checks if a timer is running.
 *
 * Parameters : Timer id and address where residual period is returned.
 *
 * Returns    : TRUE or FALSE
 *
 * Side Effects : None
 *
 *************************************************************************/
UINT32 os_wrapper_is_timer_running(TimerHandle_t timer_handle)
{
    return (UINT32) xTimerIsTimerActive(timer_handle);
}

/**************************************************************************
 * Function   : os_wrapper_create_task
 *
 * Description: Wrapper for creating a task
 *
 * Parameters : address where task id is returned, task name, task priority,
 *              task function, task stack size and busy period value.
 *
 * Returns    : API_RESULT
 *
 * Side Effects : None
 *
 *************************************************************************/

API_RESULT  os_wrapper_create_task ( TASK_ID *task_id, char* name, 
                                     unsigned char priority,
                                     OS_TASK_FUNCTION task_func, 
                                     unsigned int stack_size,
                                     unsigned short busy_period )
{
    if ( task_id == NULL )
    {
        return (BT_ERROR_INVALID_PARAMETER);
    }

    *task_id = OS_create_task ( name, priority, task_func, stack_size, busy_period );

    if ( *task_id == OS_INVALID_HANDLE )
    {
        return BT_ERROR;
    }
    else
    {
        return BT_ERROR_OK;
    }
}


/**************************************************************************
 * Function   : os_wrapper_reset_task
 *
 * Description: Wrapper for resetting a task.
 *
 * Parameters : task id
 *
 * Returns    : API_RESULT
 *
 * Side Effects : None
 *
 *************************************************************************/

API_RESULT  os_wrapper_reset_task ( TASK_ID  task_id )
{
    OS_STATUS   status;

    status = OS_reset_task (task_id );
    if ( status == ERROR )
    {
        return BT_ERROR;
    }
    return BT_ERROR_OK;
}

/**************************************************************************
 * Function   : os_wrapper_delete_task
 *
 * Description: Wrapper for deleting a task.
 *
 * Parameters : task id
 *
 * Returns    : API_RESULT
 *
 * Side Effects : None
 *
 *************************************************************************/

API_RESULT  os_wrapper_delete_task ( TASK_ID  task_id )
{
    OS_STATUS   status;

    status = OS_delete_task (task_id );
    if ( status == ERROR )
    {
        return BT_ERROR;
    }
    return BT_ERROR_OK;
}

/**************************************************************************
 * Function   : os_wrapper_send_signal_to_task
 *
 * Description: Wrapper for sending specified signal to a specified task
 *
 * Parameters : task to which signal is to be sent and the signal.
 *
 * Returns    : API_RESULT
 *
 * Side Effects : None
 *
 *************************************************************************/
API_RESULT  os_wrapper_send_signal_to_task ( TASK_ID task_id, OS_SIGNAL signal )
{
    int status;
	
    status = OS_send_signal_to_task(task_id, signal);
    if (status == ERROR)
    {
        return BT_ERROR;
    }
    return BT_ERROR_OK;
}

/**************************************************************************
 * Function   : os_wrapper_send_signal_to_task_head
 *
 * Description: Wrapper for sending specified signal to the head of a 
 *              specified task
 *
 * Parameters : task to which signal is to be sent and the signal.
 *
 * Returns    : API_RESULT
 *
 * Side Effects : None
 *
 *************************************************************************/
API_RESULT  os_wrapper_send_signal_to_task_head ( TASK_ID task_id, OS_SIGNAL signal )
{
    int status;

    status = OS_send_signal_to_task_head(task_id, signal);
    if (status == ERROR)
    {
        return BT_ERROR;
    }
    return BT_ERROR_OK;
}


INLINE UINT16  os_get_reserved_buffers(void)
{
    return ( (UINT16)os_reserved_buffers);
}

void os_reserve_buffer(void)
{
    DEF_CRITICAL_SECTION_STORAGE;
    MINT_OS_ENTER_CRITICAL();
    os_reserved_buffers++;
    MINT_OS_EXIT_CRITICAL();
}

void os_free_reserved_buffer(void)
{
    DEF_CRITICAL_SECTION_STORAGE;
    MINT_OS_ENTER_CRITICAL();
    if(os_reserved_buffers)
    {
        os_reserved_buffers--;
    }
    MINT_OS_EXIT_CRITICAL();
}

void os_reset_reserved_buffer(void)
{
    DEF_CRITICAL_SECTION_STORAGE;
    MINT_OS_ENTER_CRITICAL();
    os_reserved_buffers = 0;
    MINT_OS_EXIT_CRITICAL();
}


INLINE UINT8  os_read_occupied_le_reserved_buffers(void)
{
    return ( (UINT8)le_os_reserved_buffers);
}


INLINE void os_occupy_le_reserve_buffers_from_isr(UINT8 buffer_num)
{
    le_os_reserved_buffers += buffer_num;
}


INLINE void os_occupy_le_reserve_buffers(UINT8 buffer_num)
{
    DEF_CRITICAL_SECTION_STORAGE;
    MINT_OS_ENTER_CRITICAL();
    le_os_reserved_buffers += buffer_num;
    MINT_OS_EXIT_CRITICAL();
}

INLINE void os_release_one_le_reserved_buffer(void)
{
    DEF_CRITICAL_SECTION_STORAGE;
    MINT_OS_ENTER_CRITICAL();
    if(le_os_reserved_buffers)
    {
        le_os_reserved_buffers--;
    }
    MINT_OS_EXIT_CRITICAL();
}

INLINE void os_reset_le_reserved_buffer(void)
{
    DEF_CRITICAL_SECTION_STORAGE;
    MINT_OS_ENTER_CRITICAL();
    le_os_reserved_buffers = 0;
    MINT_OS_EXIT_CRITICAL();
}

