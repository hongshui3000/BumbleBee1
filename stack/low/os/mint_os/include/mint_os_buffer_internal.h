/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

#ifndef _H_OS_BUFFER_
#define _H_OS_BUFFER_

#include "mint_os_stack_internal.h"
#include "bt_fw_config.h"

typedef struct  os_pool
{
    OS_ADDRESS  start_ptr;
	OS_ADDRESS  end_ptr;  // added by guochunxia
    OS_STACK    free_stack;
    UINT32      chunk_size;
    UINT32      num_chunks;
    INT32       free_pools;
    UINT8       m_alignment;
    UINT8       m_compensate;
} OS_POOL;

typedef struct  os_pool_mgr
{
    OS_POOL     pool_array[OS_MAX_POOLS];
    UINT32      num_pools;

} OS_POOL_MGR;

OS_STATUS  OS_buffer_mgmt_init ( void );

#endif /* _H_OS_BUFFER_ */


