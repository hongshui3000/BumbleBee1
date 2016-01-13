/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

#ifndef _H_OS_QUEUE_
#define _H_OS_QUEUE_

#include "mint_os.h"
#include "bt_fw_config.h"

typedef struct  os_queue
{
    OS_SIGNAL      *item_array;
    INT32           start_index;
    INT32           end_index;
    INT32           max_items;
    INT32           q_length;
} OS_QUEUE;

typedef struct  os_queue_mgr
{
    OS_QUEUE       queue_array[OS_TOTAL_QUEUES];
    INT32          num_queues;
} OS_QUEUE_MGR;

OS_STATUS       OS_queue_mgmt_init ( void );

OS_HANDLE       OS_create_queue(UINT32 num_items, UINT8 preferred_handle);

OS_STATUS       OS_delete_queue ( OS_HANDLE queue );

OS_STATUS       OS_reset_queue ( OS_HANDLE handle );

OS_STATUS       OS_enqueue_item(OS_HANDLE queue, OS_ADDRESS item, UINT8 tail);

OS_STATUS       OS_remove_item ( OS_HANDLE queue, OS_ADDRESS item );

OS_BOOL         OS_is_queue_empty ( OS_HANDLE queue );

UINT32          OS_queue_get_num_of_items ( OS_HANDLE queue );

#endif /* _H_OS_QUEUE_ */

