/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/
/********************************* Logger *************************/ 
enum { __FILE_NUM__= 64 };
/********************************* Logger *************************/
#include "mint_os_queue_internal.h"
#include "bz_log_defines.h"
#include "mem.h"
#include "UartPrintf.h"

OS_QUEUE_MGR        queue_mgr;

UINT32 bm_task_queue_full = 0;
UINT16 task_sig_lost_history[OS_TASK_SIG_LOST_MAX_CNT];
UINT8 task_sig_lost_cnt = 0;

/**************************************************************************
 * Function     : OS_queue_mgmt_init
 *
 * Description  : This function initialises the queue management. 
 *
 * Parameters   : None
 *
 * Returns      : OK if successful, else ERROR;
 *
 * Side Effects : None
 *
 *************************************************************************/

OS_STATUS          OS_queue_mgmt_init ( void )
{
    UINT32 index;

    queue_mgr.num_queues = 0;

    for ( index = 0; index < OS_TOTAL_QUEUES; index++ )
    {
        queue_mgr.queue_array[ index ].start_index  = 0;
        queue_mgr.queue_array[ index ].end_index    = 0;
        queue_mgr.queue_array[ index ].q_length     = OS_INVALID_VALUE;
        queue_mgr.queue_array[ index ].item_array   = NULL;
    }

    //OS_INF(log_file,"\nOS_queue_mgmt_init : success.");
	OS_INF(OS_QUEUE_MGMT_INIT_SUCCESS,0,0);
    
    bm_task_queue_full = 0;
    task_sig_lost_cnt = 0;
    memset(task_sig_lost_history, 0, OS_TASK_SIG_LOST_MAX_CNT*2);
    
    return (OK);
}

/**************************************************************************
 * Function     : OS_create_queue
 *
 * Description  : Allocates required memory dynamically to create a queue of
 *                  "num_items".
 *
 * Parameters   : None
 *
 * Returns      : A valid handle to the queue, if successful, else 
 *                  OS_INVALID_HANDLE
 *
 * Side Effects : None
 *
 *************************************************************************/

OS_HANDLE    OS_create_queue (UINT32 num_items, UINT8 preferred_handle)
{
    OS_HANDLE    handle = 0, search_result = OS_INVALID_HANDLE;

#if (MINT_OS_ARG_CHK_EN == 1) 
    /* Check for valid arguments */
    if ( num_items == 0 || num_items > OS_MAX_QUEUE_ITEMS )
    {
        //OS_ERR(log_file,"\nOS_create_queue : invalid queue size");
		OS_ERR(OS_CREATE_QUEUE_INVALID_QUEUE_SIZE,0,0);
        return (OS_INVALID_HANDLE);
    }
#endif

    do 
    {
        if (preferred_handle < OS_TOTAL_QUEUES)
        {
            if ( queue_mgr.queue_array [ preferred_handle ].q_length == OS_INVALID_VALUE ) 
            {
                search_result = preferred_handle;
                break;
            }          
        }
    
        for ( handle = 0; handle < OS_TOTAL_QUEUES; handle++ )
        {
            if ( queue_mgr.queue_array [ handle ].q_length == OS_INVALID_VALUE )
            {
                search_result = handle;
                break;
            }
        }
    }
    while (0);

    if ( search_result != OS_INVALID_HANDLE )
    {
        queue_mgr.queue_array [ search_result ].max_items = num_items;

       if ((queue_mgr.queue_array [ search_result ].item_array = 
              (OS_SIGNAL *)os_qalloc(num_items * sizeof(OS_SIGNAL))) == NULL)
        {
            //OS_ERR(log_file,"\nOS_create_queue : malloc failed!");
			OS_ERR(OS_CREATE_QUEUE_MALLOC_FAILED,0,0);
            return (OS_INVALID_HANDLE);
        }

        queue_mgr.queue_array [ search_result ].q_length = 0x00;
        queue_mgr.num_queues++;
    }

    return ( search_result );
}


/**************************************************************************
 * Function     : OS_delete_queue
 *
 * Description  : This function frees the memory allocated to the specified 
 *                  queue and reset queue manager data structures.
 *
 * Parameters   : Handle to a queue
 *
 * Returns      : OK
 *
 * Side Effects : None
 *
 *************************************************************************/

OS_STATUS          OS_delete_queue ( OS_HANDLE handle )
{
#if (MINT_OS_ARG_CHK_EN == 1) 
    if ( handle < 0 || handle >= OS_TOTAL_QUEUES )
    {
		OS_ERR(RED, MINT_OS_QUEUE_172, 0, 0);
		OS_ERR(OS_DELETE_QUEUE_INVALID_HANDLE,0,0);
        return (ERROR);
    }
#endif

    queue_mgr.queue_array[handle].start_index   = 0;
    queue_mgr.queue_array[handle].end_index     = 0;    
    queue_mgr.queue_array[handle].item_array    = NULL;
    queue_mgr.queue_array[handle].q_length      = OS_INVALID_VALUE;
    queue_mgr.num_queues--;

    return (OK);
}

/**************************************************************************
 * Function     : OS_reset_queue
 *
 * Description  : This function e specified reset queue data structures.
 *
 * Parameters   : Handle to a queue
 *
 * Returns      : OK
 *
 * Side Effects : None
 *
 *************************************************************************/

OS_STATUS  OS_reset_queue ( OS_HANDLE handle )
{
#if (MINT_OS_ARG_CHK_EN == 1) 
    if ( handle < 0 || handle > OS_TOTAL_QUEUES )
    {
        OS_ERR(log_file,"\nOS_reset_queue : invalid handle");
        return (ERROR);
    }
#endif

    queue_mgr.queue_array[handle].start_index   = 0;
    queue_mgr.queue_array[handle].end_index     = 0;
    queue_mgr.queue_array[handle].q_length      = 0x0;
    return (OK);
}


/**************************************************************************
 * Function     : OS_enqueue_item
 *
 * Description  : This function puts the "item" into a given "queue". Note that
 *                  the SIGNAL pointer by "item" is copied into the queue and 
 *                  the responsibility of free memory of "item" is with the 
 *                  caller. The function also updates the state of the queue  
 *                  after item is inserted into the queue.
 *
 * Parameters   : A queue handle and an item to be appended to the queue.
 *
 * Returns      : OK if successful, else ERROR;
 *
 * Side Effects : None
 *
 *************************************************************************/

OS_STATUS OS_enqueue_item ( OS_HANDLE queue, OS_ADDRESS item , UINT8 tail)
{
    DEF_CRITICAL_SECTION_STORAGE;

#if (MINT_OS_ARG_CHK_EN == 1)       
    /* Check for valid arguments */
    if ( queue < 0 || queue >= OS_TOTAL_QUEUES )
    {
        //OS_ERR(log_file,"\nOS_enqueue_item : invalid handle");
		//RT_BT_LOG(RED, MINT_OS_QUEUE_250, 0, 0);

		OS_ERR(OS_ENQUEUE_ITEM_INVALID_HANDLE,0,0);
        return (ERROR);
    }

    if ( item == NULL )
    {
        //OS_ERR(log_file,"\nOS_enqueue_item : invalid queue item");
		OS_ERR(OS_ENQUEUE_ITEM_INVALID_QUEUE_ITEM,0,0);
        return (ERROR);
    }
#endif

    OS_STATUS ret = ERROR;
    INT32 max_items = queue_mgr.queue_array[queue].max_items;
    MINT_OS_ENTER_CRITICAL();
    do
    {
#if (MINT_OS_ARG_CHK_EN == 1)
        INT32 q_length = queue_mgr.queue_array[queue].q_length;
        if (q_length == OS_INVALID_VALUE)
        {
            //OS_ERR(log_file,"\nOS_enqueue_item : queue state == QUEUE_FREE ");
            OS_ERR(OS_ENQUEUE_ITEM_QUEUE_STATE_QUEUE_FREE,0,0);
            break;
        }
        else if (q_length >= max_items)
        {
            OS_ERR(log_file,"\nOS_enqueue_item : queue state == QUEUE_FULL handle: %x",queue);
            //OS_ERR(RED, OS_ENQUEUE_ITEM_QUEUE_STATE_QUEUE_FULL_HANDLE,1,queue);  //20111024
            bm_task_queue_full |= 1 << queue;

            if (task_sig_lost_cnt < OS_TASK_SIG_LOST_MAX_CNT)
            {
                OS_SIGNAL *sig = (OS_SIGNAL *) item;
                task_sig_lost_history[task_sig_lost_cnt] = sig->type;
                task_sig_lost_cnt++;
            }
            break;
        }
#endif

        if (tail)
        {
            /* enqueue from tail (first in first out) */
            INT32 end_index = queue_mgr.queue_array[queue].end_index;

            memcpy(&(queue_mgr.queue_array[queue].item_array[end_index]), item,
                    sizeof(OS_SIGNAL));

            if ((++end_index) == max_items)
            {
                end_index = 0x00;
            }
            queue_mgr.queue_array[queue].end_index = end_index;
        }
        else
        {
            /* enqueue from head (last in first out) */
            UINT8 start_index = queue_mgr.queue_array[queue].start_index;

            if (start_index == 0)
            {
                start_index = max_items - 1;
            }
            else
            {
                start_index--;
            }

            memcpy(&(queue_mgr.queue_array[queue].item_array[start_index]),
                    item, sizeof(OS_SIGNAL));

            queue_mgr.queue_array[queue].start_index = start_index;
        }

        queue_mgr.queue_array[queue].q_length++;
        ret = OK;
    } while (0);
    MINT_OS_EXIT_CRITICAL();

    return ret;
}


/**************************************************************************
 * Function     : OS_remove_item
 *
 * Description  : This function removes an "item" off the given "queue" and 
 *                  updates the state of the queue.
 *
 * Parameters   : A queue handle and an address where the contents of queued
 *                  item is copied.
 *
 * Returns      : OK if successful, else ERROR;
 *
 * Side Effects : None
 *
 *************************************************************************/

OS_STATUS OS_remove_item ( OS_HANDLE queue, OS_ADDRESS item )
{
    INT32 start_index = OS_INVALID_VALUE;
    DEF_CRITICAL_SECTION_STORAGE;

    MINT_OS_ENTER_CRITICAL();

    start_index = queue_mgr.queue_array [ queue ].start_index;

    memcpy ( item, &(queue_mgr.queue_array [ queue ].item_array [ start_index ]),
             sizeof ( OS_SIGNAL ) );

    if ((++start_index) == queue_mgr.queue_array[ queue ].max_items)
    {
        start_index = 0x00;
    }

    queue_mgr.queue_array [ queue ].start_index = start_index;
    queue_mgr.queue_array[ queue ].q_length--;

    MINT_OS_EXIT_CRITICAL();

    return (OK);
}


/**************************************************************************
 * Function     : OS_is_queue_empty
 *
 * Description  : This function checks if the queue is empty.
 *
 * Parameters   : None
 *
 * Returns      : TRUE if empty, else FALSE.
 *
 * Side Effects : None
 *
 *************************************************************************/

OS_BOOL           OS_is_queue_empty ( OS_HANDLE queue )
{
    return  (OS_BOOL) ( queue_mgr.queue_array[ queue ].q_length == 0x00 );
}


/**************************************************************************
 * Function     : OS_queue_get_num_of_items
 *
 * Description  : This function gives the number of items queued in a given 
 *                  queue.
 *
 * Parameters   : A queue handle
 *
 * Returns      : Number of items in the queue.
 *
 * Side Effects : None
 *
 *************************************************************************/

UINT32   OS_queue_get_num_of_items ( OS_HANDLE queue )
{
#if (MINT_OS_ARG_CHK_EN == 1)

	/* Check for valid arguments */
    if ( queue < 0 || queue >= OS_TOTAL_QUEUES )
    {
        //OS_ERR(log_file,"\nOS_is_queue_empty : invalid handle");
		//RT_BT_LOG(RED, MINT_OS_QUEUE_425, 0, 0);
		OS_ERR(OS_IS_QUEUE_EMPTY_INVALID_HANDLE,0,0);

        return ((UINT32)OS_INVALID_VALUE);
    }

#endif

    return  (queue_mgr.queue_array[ queue ].q_length);
}


