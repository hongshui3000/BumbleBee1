/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/
/********************************* Logger *************************/
enum { __FILE_NUM__= 63 };
/********************************* Logger *************************/
#include "bz_log_defines.h"
#include "mint_os_buffer_internal.h"
#include "mem.h"

OS_POOL_MGR                 pool_mgr;

/**************************************************************************
 * Function     : OS_buffer_mgmt_init
 *
 * Description  : Initialises the free buffer pools.
 *
 * Parameters   : None.
 *
 * Returns      : OK
 *
 * Side Effects : None
 *
 *************************************************************************/

OS_STATUS  OS_buffer_mgmt_init ( void )
{
    INT16  index;

    for ( index = 0; index < OS_MAX_POOLS; index++ )
    {
        pool_mgr.pool_array[ index ].start_ptr = NULL;
        pool_mgr.pool_array[ index ].chunk_size = 0;
        pool_mgr.pool_array[ index ].num_chunks = 0;
        pool_mgr.pool_array[ index ].free_pools = 0;
    }
    pool_mgr.num_pools = 0;

    //OS_INF(log_file,"\nOS_buffer_mgmt_init : success.");
    OS_INF(OS_BUFFER_MGMT_INIT_SUCCESS,0,0);
    return (OK);
}

/**************************************************************************
 * Function     : OS_create_pool
 *
 * Description  : This function creates a buffer pool of size "num_chunks"
 *                  unit(s) where size of one unit is "chunk_size". All
 *                  subsequent calls to allocate memory from this pool return a
 *                  buffer of size "chunk_size". The function allocates memory
 *                  dynamically and initialises its free address stack and
 *                  for the pool returns a handle to the pool.
 *
 * Parameters   : None.
 *
 * Returns      : OK
 *
 * Side Effects : None
 *
 *************************************************************************/
OS_HANDLE   OS_create_pool ( UINT32 chunk_size, UINT32 num_chunks,
                             UINT8 m_alignment, UINT8 m_compensate, RAM_TYPE mem_mode)
{
    INT16  search_result    = OS_INVALID_VALUE;
    UINT16 index;
    UINT32 chunk_index;
    OS_ADDRESS free_address;
    UINT8 compensate_value;

    /* Check for valid arguments */
    if (( chunk_size == 0 ) || ( num_chunks == 0 ))
    {
        //OS_ERR(log_file,"\nPool of size 0 requested!");
        OS_ERR(POOL_OF_SIZE_0_REQUESTED,0,0);
        return (OS_INVALID_HANDLE);
    }

    /* Check for free pool */
    for ( index = 0; index < OS_MAX_POOLS; index++ )
    {
        if ( pool_mgr.pool_array[ index ].start_ptr == NULL )
        {
            search_result = (INT16) index;
            break;
        }
    }

    if ( search_result != OS_INVALID_VALUE )
    {
        MEM_DBG_LOG(BLUE, ALLOCATE_POOL_ID_NUM, 1, search_result);
        if (OS_stack_init ( &(pool_mgr.pool_array [ search_result ].free_stack),
                            num_chunks, RAM_TYPE_DATA_ON ) == ERROR )
        {
            //OS_ERR(log_file,"\nOS_create_pool : memory needed for pool management failed.");
            OS_ERR(OS_CREATE_POOL_MEMORY_NEEDED_FOR_POOL,0,0);
            return (OS_INVALID_HANDLE);
        }

        pool_mgr.pool_array[ search_result ].chunk_size = chunk_size;
        pool_mgr.pool_array[ search_result ].num_chunks = num_chunks;
        pool_mgr.pool_array[ search_result ].free_pools = num_chunks;
        pool_mgr.pool_array[ search_result ].m_alignment = m_alignment;
        pool_mgr.pool_array[ search_result ].m_compensate = m_compensate;

        MEM_DBG_LOG(RED,POOL_SIZE,2,((chunk_size * num_chunks) + m_compensate),((chunk_size * num_chunks) + m_compensate));
        if ( ( pool_mgr.pool_array[ search_result ].start_ptr =
                    ( OS_ADDRESS ) os_malloc ( (chunk_size * num_chunks) + m_compensate, mem_mode ) )
                == NULL )
        {
            //OS_ERR(log_file,"\nOS_create_pool : pool memory allocation failed.");
            OS_ERR(OS_CREATE_POOL_POOL_MEMORY_ALLOCATION_FAILED,0,0);
            return (OS_INVALID_HANDLE);
        }
        free_address = pool_mgr.pool_array[ search_result ].start_ptr;
        if (m_alignment)
        {
            compensate_value = ((UINT32)free_address & (m_alignment - 1));
            if (compensate_value)
            {
                free_address = (OS_ADDRESS)((UINT32)free_address + (UINT32)(m_alignment - compensate_value));
            }
        }
        pool_mgr.pool_array[ search_result ].start_ptr = free_address;

        if ((mem_mode == RAM_TYPE_BUFFER_ON) || (mem_mode == RAM_TYPE_BUFFER_OFF))
        {
            pool_mgr.pool_array[ search_result ].start_ptr = (OS_ADDRESS)(((UINT32)pool_mgr.pool_array[ search_result ].start_ptr) | KSEG1);
        }
        
        MEM_DBG_LOG(BLUE, ALLOCATE_ADDRESS, 1, pool_mgr.pool_array[ search_result ].start_ptr);

        free_address = pool_mgr.pool_array[ search_result ].start_ptr;

        pool_mgr.pool_array[ search_result ].end_ptr
                = (UINT8 *) pool_mgr.pool_array[ search_result ].start_ptr
                  + num_chunks * chunk_size;

        for ( chunk_index = 0; chunk_index < num_chunks; chunk_index++ )
        {
            if (OS_stack_push(&(pool_mgr.pool_array[search_result].free_stack),
                              free_address ) == ERROR )
            {
                OS_ERR(RED, MINT_OS_BUFFER_323, 0, 0);
                OS_ERR(OS_CREATE_POOL_POOL_INITIALISATION_FAILED,0,0);
                return (OS_INVALID_HANDLE);
            }

            free_address = (OS_ADDRESS)
                           ( (INT32)free_address + (INT32)chunk_size );
            /*for alignment*/
            if (m_alignment)
            {
                compensate_value = ((UINT32)free_address & (m_alignment - 1));
                if (compensate_value)
                {
                    free_address = (OS_ADDRESS)((UINT32)free_address + (UINT32)(m_alignment - compensate_value));
                }
            }

        }
    }

    return (search_result);
}


/**************************************************************************
 * Function     : OS_reset_pool
 *
 * Description  : This function frees all the buffers in the pool and
 *                  re-initialises the buffer pool data structures.
 *
 * Parameters   : None.
 *
 * Returns      : OK
 *
 * Side Effects : None
 *
 *************************************************************************/
OS_STATUS   OS_reset_pool ( OS_HANDLE pool )
{
    OS_INF(GRAY, MINT_OS_BUFFER_372, 0, 0);

    UINT32      chunk_size, num_chunks, chunk_index;
    OS_ADDRESS  free_address;
    UINT8       m_alignment, m_compensate, compensate_value;

#if (MINT_OS_ARG_CHK_EN == 1)
    /* Check for valid arguments */
    if ( ( pool <= OS_INVALID_HANDLE ) || ( pool >= OS_MAX_POOLS ) )
    {
        //OS_ERR(log_file,"\nOS_reset_pool : invalid pool handle");
        OS_ERR(OS_RESET_POOL_INVALID_POOL_HANDLE,0,0);
        return (ERROR);
    }
#endif

    if ( pool_mgr.pool_array [ pool ].start_ptr == NULL )
    {
        //OS_ERR(log_file,"\nOS_reset_pool : invalid pool");
        OS_ERR(OS_RESET_POOL_INVALID_POOL,0,0);
        return (ERROR);
    }

    /* Zero out the pool memory */
    chunk_size = pool_mgr.pool_array[ pool ].chunk_size;
    num_chunks = pool_mgr.pool_array[ pool ].num_chunks;
    m_alignment = pool_mgr.pool_array[ pool ].m_alignment;
    m_compensate = pool_mgr.pool_array[ pool ].m_compensate;
    pool_mgr.pool_array[ pool ].free_pools = num_chunks;

    memset( pool_mgr.pool_array[ pool ].start_ptr, 0, (chunk_size * num_chunks + m_compensate));

    /* Reinitialise the pool stack */
    if( OS_stack_reset ( &(pool_mgr.pool_array [ pool ].free_stack) ) == ERROR )
    {
        //OS_ERR(log_file,"\nOS_reset_pool : pool stack reset failed.");
        OS_ERR(OS_RESET_POOL_POOL_STACK_RESET_FAILED,0,0);
        return (ERROR);
    }

    /* Reconstruct the stack */
    free_address = pool_mgr.pool_array[ pool ].start_ptr;
    for ( chunk_index = 0; chunk_index < num_chunks; chunk_index++ )
    {
        if ( OS_stack_push ( &(pool_mgr.pool_array [ pool ].free_stack),
                             free_address ) == ERROR )
        {
            //OS_ERR(log_file,"\nOS_reset_pool : pool reset failed.");
            OS_ERR(OS_RESET_POOL_POOL_RESET_FAILED,0,0);
            return (ERROR);
        }

        free_address = (OS_ADDRESS)((INT32)free_address + (INT32)chunk_size);

        /*for alignment*/
        if (m_alignment)
        {
            compensate_value = ((UINT32)free_address & (m_alignment - 1));
            if (compensate_value)
            {
                free_address = (OS_ADDRESS)((UINT32)free_address + (UINT32)(m_alignment - compensate_value));
            }
        }
    }

    return (OK);
}


/**************************************************************************
 * Function     : OS_delete_pool
 *
 * Description  : This function deallocates the memory allocated to the
 *                  specified pool and resets the pool mgr data structures.
 *
 * Parameters   : None.
 *
 * Returns      : OK
 *
 * Side Effects : None
 *
 *************************************************************************/

OS_STATUS   OS_delete_pool ( OS_HANDLE pool )
{
#if (MINT_OS_ARG_CHK_EN == 1)
    /* Check if pool is valid */
    if ( ( pool <= OS_INVALID_HANDLE ) || ( pool >= OS_MAX_POOLS ) )
    {
        //OS_ERR(log_file,"\nOS_delete_pool : invalid pool handle");
        OS_ERR(OS_DELETE_POOL_INVALID_POOL_HANDLE,0,0);
        return (ERROR);
    }
#endif

    if ( pool_mgr.pool_array [ pool ].start_ptr == NULL )
    {
        //OS_ERR(log_file,"\nOS_delete_pool : invalid pool");
        OS_ERR(OS_DELETE_POOL_INVALID_POOL,0,0);
        return (ERROR);
    }

    if ( OS_stack_cleanup ( &(pool_mgr.pool_array [ pool ].free_stack) )
            == ERROR )
    {
        //OS_ERR(log_file,"\nOS_delete_pool : pool stack cleanup failed");
        OS_ERR(OS_DELETE_POOL_POOL_STACK_CLEANUP_FAILED,0,0);
        return (OS_INVALID_HANDLE);
    }

    pool_mgr.pool_array [ pool ].start_ptr = NULL;
    pool_mgr.pool_array[ pool ].chunk_size = 0;
    pool_mgr.pool_array[ pool ].num_chunks = 0;
    pool_mgr.pool_array[ pool ].free_pools = 0;

    return (OK);
}


/**************************************************************************
 * Function     : OS_allocate_buffer
 *
 * Description  : This function allocates memory of one unit from a specified
 *                  pool. Unit size is fixed for each pool and is assigned
 *                  during pool creation. Allocation of buffer is an inexpensive
 *                  operation as it involves only poping an item from the pool's
 *                  stack.
 *
 * Parameters   : Handle of the pool.
 *
 * Returns      : OK if successful, else NULL;
 *
 * Side Effects : None
 *
 *************************************************************************/

OS_ADDRESS  OS_allocate_buffer (OS_HANDLE pool)
{
    OS_ADDRESS  free_address = NULL;

#if (MINT_OS_ARG_CHK_EN == 1)
    /* Check if pool is valid */
    if(( pool <= OS_INVALID_HANDLE ) || ( pool >= OS_MAX_POOLS ) )
    {
        //OS_ERR(log_file,"\nOS_allocate_buffer : invalid pool handle");
        OS_ERR(OS_ALLOCATE_BUFFER_INVALID_POOL_HANDLE,0,0);
        OS_ERR(GRAY, MINT_OS_BUFFER_542, 1, pool);

        return (NULL);
    }
#endif

    if ( pool_mgr.pool_array [ pool ].start_ptr == NULL )
    {
        //OS_ERR(log_file,"\nOS_allocate_buffer : invalid pool");
        OS_ERR(OS_ALLOCATE_BUFFER_INVALID_POOL,0,0);
        OS_ERR(GRAY, MINT_OS_BUFFER_552, 0, 0);
        return (NULL);
    }

    if (OS_stack_pop ( &(pool_mgr.pool_array [ pool ].free_stack),
                       &free_address ) == ERROR )
    {
        //OS_ERR(log_file,"\nOS_allocate_buffer : alloocation failed!");
        OS_ERR(OS_ALLOCATE_BUFFER_ALLOOCATION_FAILED,0,0);
        RT_BT_LOG(RED, MSG_ALLOC_OS_BUFFER_FAIL, 3,
                  pool,pool_mgr.pool_array[ pool ].free_pools,
                  pool_mgr.pool_array[ pool ].free_stack.top);

        return (NULL);
    }

    pool_mgr.pool_array[pool].free_pools--;

    return (free_address);
}


UINT32 OS_get_free_buffer(OS_HANDLE pool_id)
{
    return pool_mgr.pool_array[pool_id].free_pools ;
}


/**************************************************************************
 * Function     : OS_deallocate_buffer
 *
 * Description  : This function deallocates memory from a specified pool.
 *                  Deallocation of buffer is an inexpensive operation as
 *                  it involves only pushing an item onto the pool's stack.
 *
 * Parameters   : Handle of the pool and address to be freed.
 *
 * Returns      : OK if successful, else ERROR
 *
 * Side Effects : None
 *
 *************************************************************************/
OS_STATUS   OS_deallocate_buffer (OS_HANDLE pool, OS_ADDRESS address )
{
    OS_ADDRESS      start_address = 0;
    OS_ADDRESS      end_address = 0;
    UINT32          chunk_size;
    UINT32          num_chunks;
    OS_BOOL         is_address_freed = FALSE;

    DEF_CRITICAL_SECTION_STORAGE;

    /* Check if pool is valid */
    if ( ( pool <= OS_INVALID_HANDLE ) || ( pool >= OS_MAX_POOLS ) )
    {
        //OS_ERR(log_file,"\nOS_deallocate_buffer : invalid pool handle");
        OS_ERR(OS_DEALLOCATE_BUFFER_INVALID_POOL_HANDLE,0,0);
        OS_ERR(GRAY, MINT_OS_BUFFER_646, 2,
                  pool, address);

        return (ERROR);
    }


    MINT_OS_ENTER_CRITICAL();

    if ( pool_mgr.pool_array [ pool ].start_ptr == NULL )
    {
        MINT_OS_EXIT_CRITICAL();

        //OS_ERR(log_file,"\nOS_allocate_buffer : invalid pool");
        OS_ERR(OS_ALLOCATE_BUFFER_INVALID_POOL,0,0);
        OS_ERR(GRAY, MINT_OS_BUFFER_657, 2,
                  pool, address);

        return (ERROR);
    }

    /* Initialise local variables */
    start_address   = pool_mgr.pool_array [ pool ].start_ptr;
    chunk_size      = pool_mgr.pool_array [ pool ].chunk_size;
    num_chunks      = pool_mgr.pool_array [ pool ].num_chunks;

    end_address = pool_mgr.pool_array [ pool ].end_ptr;

    /* Check if the specified address is in pool memory range */
    if ( ( address < start_address ) || ( address >= end_address ) )
    {
        MINT_OS_EXIT_CRITICAL();

        //OS_ERR(log_file,"\nOS_allocate_buffer : invalid address (out of pool memory range)");
        OS_ERR(OS_ALLOCATE_BUFFER_INVALID_ADDRESS_OUT_OF_POOL_MEMORY_RANGE,0,0);
        RT_BT_LOG(RED, MINT_OS_BUFFER_684, 6, pool, address,
                  start_address, end_address, chunk_size, num_chunks);

        return (ERROR);
    }

    /*
     * The address belongs to the pool and is on its chunk boundary.
     * If its not on the free list, then only we should free it.
     * To make that check, we go down our free stack searching for this
     * address.
     */
    if(OS_stack_search_item (&(pool_mgr.pool_array [ pool ].free_stack),
                             address, &is_address_freed ) == ERROR )
    {
        MINT_OS_EXIT_CRITICAL();

        //OS_ERR(log_file,"\nOS_deallocate_buffer : failed!");
        OS_ERR(OS_DEALLOCATE_BUFFER_FAILED,0,0);
        OS_ERR(GRAY, MINT_OS_BUFFER_737, 0, 0);

        return (ERROR);
    }

    if(is_address_freed == TRUE )
    {
        MINT_OS_EXIT_CRITICAL();

        //OS_ERR(log_file,"\nOS_allocate_buffer : address already freed");
        OS_ERR(OS_ALLOCATE_BUFFER_ADDRESS_ALREADY_FREED,0,0);
        OS_ERR(RED, MINT_OS_BUFFER_756, 4,
                  address, pool,
                  pool_mgr.pool_array[ pool ].free_pools,
                  pool_mgr.pool_array[ pool ].free_stack.top);

        // modified from OK to ERROR by guochunxia
        return (ERROR);
        //return (OK);
    }

    /*
     * The specified address is a valid address of the pool and is
     * not part of free list of addresses belonging to the pool, so we can
     * free it.
     */

#ifndef _DONT_INIT_ALLOCATED_MEMORY_FROM_POOL
    /* Zero out the memory location */
    memset ( address, 0, pool_mgr.pool_array [ pool ].chunk_size );
#endif

    if(pool_mgr.pool_array[ pool].free_pools == pool_mgr.pool_array[ pool].num_chunks)
    {
        OS_ERR(RED, MINT_OS_BUFFER_786, 0, 0);
    }

    /* Put it in the free list */
    if(OS_stack_push ( &(pool_mgr.pool_array [ pool ].free_stack), address )
            == ERROR )
    {
        MINT_OS_EXIT_CRITICAL();

        //OS_ERR(log_file,"\nOS_deallocate_buffer : failed!");
        OS_ERR(GRAY, MINT_OS_BUFFER_794, 0, 0);
        OS_ERR(OS_DEALLOCATE_BUFFER_FAILED,0,0);

        return (ERROR);
    }

    pool_mgr.pool_array[ pool ].free_pools++;
    MINT_OS_EXIT_CRITICAL();

    return (OK);
}

