/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

#ifndef _H_OS_STACK_
#define _H_OS_STACK_

#include    "mint_os.h"
#include    "bt_fw_config.h"

typedef struct  os_stack
{
    UINT32      num_items;
    INT32       top;
    OS_ADDRESS  *item;
} OS_STACK;

OS_STATUS  OS_stack_init ( OS_STACK *stack, UINT32 num_items, RAM_TYPE mem_mode );

OS_STATUS  OS_stack_reset ( OS_STACK *stack );

OS_STATUS  OS_stack_cleanup ( OS_STACK *stack );

OS_STATUS  OS_stack_push ( OS_STACK *stack, OS_ADDRESS  item );

OS_STATUS  OS_stack_pop  ( OS_STACK *stack, OS_ADDRESS *item_ptr );

OS_STATUS  OS_stack_search_item ( OS_STACK *stack, OS_ADDRESS item,
                                  OS_BOOL *result );
OS_BOOL    OS_is_stack_empty ( OS_STACK *stack );

UINT32     OS_stack_get_num_of_items ( OS_STACK *stack );

#endif /* _H_OS_STACK_ */


