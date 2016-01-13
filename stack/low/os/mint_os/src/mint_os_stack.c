/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/
/********************************* Logger *************************/ 
enum { __FILE_NUM__= 65 };
/********************************* Logger *************************/
#include "bz_log_defines.h"
#include "mint_os_stack_internal.h"
#include "mem.h"

//static UINT32 count = 0;
/**************************************************************************
 * Function     : OS_stack_init
 *
 * Description  : Initialises a given stack. 
 *
 * Parameters   : Pointer to a stack and stack size
 *
 * Returns      : OK if successful, else ERROR.
 *
 * Side Effects : None
 *
 *************************************************************************/
OS_STATUS  OS_stack_init ( OS_STACK *stack, UINT32 num_items, RAM_TYPE mem_mode )
{
    /* Check for valid arguments */
    if ( stack == NULL )
    {
        //OS_ERR(log_file,"\nOS_stack_init : stack == NULL ");
		OS_ERR(OS_STACK_INIT_STACK_NULL,0,0);
        return (ERROR);
    }

    if ( num_items > OS_MAX_STACK_ITEMS )
    {
        //OS_ERR(log_file,"\nOS_stack_init : invalid stack size ");
		OS_ERR(OS_STACK_INIT_INVALID_STACK_SIZE,0,0);
        return (ERROR);
    }
    if ((stack->item = (OS_ADDRESS *)os_malloc(sizeof(OS_ADDRESS) * num_items, mem_mode)) == NULL )
	{
		//OS_ERR(log_file,"\nOS_stack_init : memory allocation for stack failed");
		OS_ERR(OS_STACK_INIT_MEMORY_ALLOCATION_FOR_STACK_FAILED,0,0);
		return (ERROR);
	}

    stack->num_items = num_items;
    stack->top = OS_INVALID_VALUE;

    return (OK);
}


/**************************************************************************
 * Function     : OS_stack_reset
 *
 * Description  : Resets a given stack. 
 *
 * Parameters   : Pointer to a stack
 *
 * Returns      : OK if successful, else ERROR.
 *
 * Side Effects : None
 *
 *************************************************************************/

OS_STATUS  OS_stack_reset ( OS_STACK *stack )
{
    /* Check for valid arguments */
    if ( stack == NULL )
    {
        //OS_ERR(log_file,"\nOS_stack_init : stack == NULL ");
		OS_ERR(OS_STACK_INIT_STACK_NULL,0,0);
        return (ERROR);
    }

    /* Reset stack memory */
    memset ( stack->item, 0, stack->num_items );

    /* Reset stack pointer */
    stack->top = OS_INVALID_VALUE;

    return (OK);
}

/**************************************************************************
 * Function     : OS_stack_cleanup
 *
 * Description  : Cleans up a given stack. 
 *
 * Parameters   : Pointer to a stack
 *
 * Returns      : OK if successful, else ERROR.
 *
 * Side Effects : None
 *
 *************************************************************************/

OS_STATUS  OS_stack_cleanup ( OS_STACK *stack )
{
    /* check for valid arguments */
    if ( stack == NULL )
    {
        //OS_ERR(log_file,"\nOS_stack_init : stack == NULL ");
		OS_ERR(OS_STACK_INIT_STACK_NULL,0,0);
        return (ERROR);
    }

    if ( stack->item != NULL )
    {
        stack->item = NULL;
    }

    return (OK);
}


/**************************************************************************
 * Function     : OS_stack_push
 *
 * Description  : Pushes an "item" on the "stack".  
 *
 * Parameters   : Pointer to a stack and address of the item to be pushed.
 *
 * Returns      : OK if successful, else ERROR;
 *
 * Side Effects : None
 *
 *************************************************************************/

OS_STATUS  OS_stack_push ( OS_STACK *stack, OS_ADDRESS  item )
{
#if (MINT_OS_ARG_CHK_EN == 1)
    /* check for valid arguments */
    if ( stack == NULL )
    {
        //OS_ERR(log_file,"\nOS_stack_push : stack == NULL ");
		OS_ERR(OS_STACK_PUSH_STACK_NULL,0,0);
		OS_ERR(GRAY, MINT_OS_STACK_175, 0, 0);
        return (ERROR);
    }

    if ( item == NULL )
    {
        //OS_ERR(log_file,"\nOS_stack_push : stack item == NULL ");
		OS_ERR(OS_STACK_PUSH_STACK_ITEM_NULL,0,0);
		OS_ERR(GRAY, MINT_OS_STACK_183, 0, 0);
        return (ERROR);
    }
#endif

	/* check if stack is full */
    if(stack->top >= ((INT32)stack->num_items-1))
    {
        //OS_INF(log_file,"\nOS_stack_push : stack full!");
		OS_INF(OS_STACK_PUSH_STACK_FULL,0,0);
		OS_INF(RED, MINT_OS_STACK_192, 0, 0);
        return (ERROR);
    }

    /*
     * Increment stack top pointer
     * NOTE : Top pointer points to the most recently occupied entry.
     */
    stack->top++;

    /* push item on the stack */
    stack->item [ stack->top ] = item;
	
	MEM_DBG_LOG(BLUE, PUSH_ADDRESS_TO_STACK, 2, stack->top, stack->item [ stack->top ]);

    return (OK);
}


/**************************************************************************
 * Function     : OS_stack_pop
 *
 * Description  : Pops an "item" off the "stack".  
 *
 * Parameters   : Pointer to a stack and address where the address of the 
 *                  "item" is stored.
 *
 * Returns      : OK if successful, else ERROR.
 *
 * Side Effects : None
 *
 *************************************************************************/

OS_STATUS  OS_stack_pop  ( OS_STACK *stack, OS_ADDRESS *item_ptr )
{
#if (MINT_OS_ARG_CHK_EN == 1)
    /* check for valid arguments */
    if ( stack == NULL )
    {
		OS_ERR(GRAY, MINT_OS_STACK_258, 0, 0);
		OS_ERR(OS_STACK_POP_STACK_NULL,0,0);
        return (ERROR);
    }

    if ( item_ptr == NULL )
    {
		OS_ERR(GRAY, MINT_OS_STACK_265, 0, 0);
		OS_ERR(OS_STACK_POP_ITEM_ADDRESS_NULL,0,0);
        return (ERROR);
    }
#endif

    /* check if stack is empty */
    if(stack->top <= OS_INVALID_VALUE)
    {
        return (ERROR);
    }

	MEM_DBG_LOG(BLUE, POP_STACK_TO_ADDRESS, 2, stack->top, stack->item [ stack->top ]);

    /* Pop item from the stack */
    *item_ptr = stack->item [ stack->top ];

    /* Decrement the stack pointer */
    stack->top--;
  
    return (OK);
}


/**************************************************************************
 * Function     : OS_stack_search_item
 *
 * Description  : Searches the specified item in the stack. This function does
 *                  not modify the stack.
 *
 * Parameters   : A pointer to a stack, the item to be searched and the address
 *                  to a memory location where the function stores the result of 
 *                  the search.
 *
 * Returns      : OK if successful, else ERROR.
 *
 * Side Effects : None
 *
 *************************************************************************/

OS_STATUS    OS_stack_search_item ( OS_STACK *stack, OS_ADDRESS item,
                                    OS_BOOL *result )
{
    INT32   index = 0;
    OS_BOOL found = FALSE;

    /* check for valid arguments */
    if ( stack == NULL )
    {
		OS_ERR(GRAY, MINT_OS_STACK_340, 0, 0);
		OS_ERR(OS_STACK_SEARCH_ITEM_STACK_NULL,0,0);
        return (ERROR);
    }

    for(index = 0; index <= stack->top; index++ )
    {
        if (stack->item[index] == item )
        {
            found = TRUE;
            break;
        }
    }

    *result = found;

    return (OK);
}


/**************************************************************************
 * Function     : OS_is_stack_empty
 *
 * Description  : Makes a check if given stack is empty.
 *
 * Parameters   : Pointer to a stack.
 *
 * Returns      : TRUE if empty, else FALSE.
 *
 * Side Effects : None
 *
 *************************************************************************/
OS_BOOL    OS_is_stack_empty ( OS_STACK *stack )
{
    OS_BOOL     result = FALSE;

    /* check for valid arguments */
    if ( stack == NULL )
    {
        //OS_ERR(log_file,"\nOS_is_stack_empty : stack == NULL ");
		OS_ERR(OS_IS_STACK_EMPTY_STACK_NULL,0,0);
        return (FALSE);
    }

    /* check if stack is empty */
    if ( stack->top <= OS_INVALID_VALUE )
    {
        result = TRUE;
    }

    return (result);
}

/**************************************************************************
 * Function     : OS_stack_get_num_of_items
 *
 * Description  : Gets the count of items on the stack.
 *
 * Parameters   : A pointer to a stack.
 *
 * Returns      : Number of items on the given stack if successful, else ERROR.
 *
 * Side Effects : None
 *
 *************************************************************************/
UINT32     OS_stack_get_num_of_items ( OS_STACK *stack )
{
    /* check for valid arguments */
    if ( stack == NULL )
    {
        //OS_ERR(log_file,"\nOS_get_num_of_items : stack == NULL ");
		OS_ERR(OS_GET_NUM_OF_ITEMS_STACK_NULL,0,0);
        return ((UINT32)ERROR);
    }

    /* check if stack is empty */
    if ( stack->top <= OS_INVALID_VALUE )
    {
        //OS_ERR(log_file, "\nOS_get_num_of_items : stack in invalid state, top = %d", stack->top );
		OS_ERR(OS_GET_NUM_OF_ITEMS_STACK_IN_INVALID_STATE_TOP, 1, stack->top );
        return (OK);
    }

    return ( stack->top + 1 );
}


