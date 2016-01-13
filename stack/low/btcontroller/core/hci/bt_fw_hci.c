/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  Contains the code that implements the
 *  HCI module initialization and shutdown routines.
 */

/********************************* Logger *************************/ 
enum { __FILE_NUM__= 3 };
/********************************* Logger *************************/

/* Includes */
#include "bt_fw_hci_internal.h"

/**
 * HCI Event Buffer Pool Handle
 */
POOL_ID  hci_event_buffer_pool_handle ;
/**
 * HCI Command Buffer Pool Handle
 */
POOL_ID  hci_cmd_buffer_pool_handle ;

/**
 * HCI Command Task Handle
 */
TASK_ID  hci_command_task_handle ;
/**
 * HCI Event Task Handle
 */
TASK_ID  hci_event_task_handle ;

/**
 * Initializes HCI module resources.
 * It creates command and event tasks. It creates buffer pool for
 * HCI events.
 *
 * \param None.
 * \return API_SUCCESS or API_FAILURE.
 *
 */
API_RESULT init_hci(void)
{
    
    /* Initializes Global variables */
    hci_initialize_data();
        
    return API_SUCCESS;
}

/**
 * Deletes tasks and the buffer pools.
 *
 * \param None.
 * \return API_SUCCESS or API_FAILURE.
 *
 */
API_RESULT hci_shutdown(void)
{
#ifndef _DONT_SUPPORT_OS_SHUTDOWN_

    /* Delete the created memory pools */
    if(OS_DELETE_POOL(hci_event_buffer_pool_handle) != BT_ERROR_OK)
    {
		BT_FW_HCI_ERR(OS_DELETE_POOL_FAILED,0,0);
    }

    if(OS_DELETE_POOL(hci_cmd_buffer_pool_handle) != BT_ERROR_OK)
    {
		BT_FW_HCI_ERR(OS_DELETE_POOL_FAILED,0,0);
    }

    /* Delete tasks */
    if(OS_DELETE_TASK(hci_command_task_handle) != BT_ERROR_OK)
    {
		BT_FW_HCI_ERR(OS_DELETE_TASK_FAILED,0,0);
    }

    if(OS_DELETE_TASK(hci_event_task_handle) != BT_ERROR_OK)
    {
		BT_FW_HCI_ERR(OS_DELETE_TASK_FAILED,0,0);
    }
#endif

    return API_SUCCESS;
}

