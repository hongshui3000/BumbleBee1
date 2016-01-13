/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/
/********************************* Logger *************************/ 
enum { __FILE_NUM__= 14 };
/********************************* Logger *************************/

/* OS header files */
#include "bt_fw_os.h"

/* Module specific header files */
#include "hci_td.h"
#include "bt_fw_hci.h"
#include "lmp.h"
#include "lc.h"
#include "bz_log_defines.h"
#ifdef LE_MODE_EN
#include "le_ll.h"
#endif

#ifdef COMPILE_AFH_HOP_KERNEL
extern void hci_ch_as_task(OS_SIGNAL *signal_ptr);
extern OS_HANDLE hci_ch_as_task_handle;
#endif
extern POOL_ID synchronous_data_to_remote_id;


/******************************************************************************
 *                                                                            *
 * Function Name    :hci_td_stack_interface_init
 *
 * Description      :This function initializes the interface between the
 *                      transport driver and the MindTree task system. There are
 *                      a pair of shared tables, which holds the task ids and
 *                      the pool ids that are used across the tasks.
 *
 * Input Params     : None
 *
 * Output Results   : None
 *
 * IMPORTANT NOTE   : Note that this function HAS to be called AFTER the
 *                      tasks and pools have been created. 
 *****************************************************************************/

void hci_td_stack_interface_init(void)
{
    /*
     * Receive Table pool ID's
     * rx_table is the table that is used in the HOST==>HOST CONTROLLER
     * direction. 
     *
     * These tables store pool ids and task handles so that buffers can be
     * allocated in one task, and freed in the other.
     *
     * So, the pool handles stored here are
     * command buffer pool handle
     * acl data pool handle
     * sco data pool handle
     *
     * tx_table is the table that is used in the HOST CONTROLLER ==> HOST
     * direction.
     * So, the pool handles stored here are
     * event buffer pool handle
     * acl data to host pool handle
     * sco data to host pool handle
     */
    rx_table[HCI_CMD_HANDLER_TASK].pool_handle = hci_cmd_buffer_pool_handle;
    rx_table[ACL_DATA_HANDLER_TASK].pool_handle = acl_data_pool_id;
#if defined(SCO_OVER_HCI) || defined(COMPILE_ESCO)
    rx_table[SYNCHRONOUS_DATA_HANDLER_TASK].pool_handle
        = synchronous_data_pool_id;
#endif /* defined(SCO_OVER_HCI) || defined(COMPILE_ESCO) */

#ifdef LE_MODE_EN
    rx_table[LE_ACL_DATA_HANDLER_TASK].pool_handle = le_acl_data_to_ll_pool_id;
#endif

    /* Receive Table Task handles */
    rx_table[HCI_CMD_HANDLER_TASK].task_handle = hci_command_task_handle;
    rx_table[SYNCHRONOUS_DATA_HANDLER_TASK].task_handle = lc_tx_task_handle;
    rx_table[ACL_DATA_HANDLER_TASK].task_handle = lc_tx_task_handle;
#ifdef LE_MODE_EN
    rx_table[LE_ACL_DATA_HANDLER_TASK].task_handle = lc_tx_task_handle;
#endif

    /* Transmit Table pool handles. */
    tx_table[HCI_EVENT_HANDLER_TASK].pool_handle  = hci_event_buffer_pool_handle;
    tx_table[ACL_DATA_HANDLER_TASK].pool_handle = acl_data_to_host_pool_id;
#if defined(SCO_OVER_HCI) || defined(COMPILE_ESCO)
    tx_table[SYNCHRONOUS_DATA_HANDLER_TASK].pool_handle
        = synchronous_data_to_host_pool_id;
#endif /* defined(SCO_OVER_HCI) || defined(COMPILE_ESCO) */

#ifdef LE_MODE_EN
    tx_table[LE_ACL_DATA_HANDLER_TASK].pool_handle = le_acl_data_to_host_pool_id;    
#endif

    /* Transmit Table task handles. */
    tx_table[TRANSPORT_DRIVER_TASK].task_handle = hci_td_task_handle;
}


/******************************************************************************
 *                                                                            *
 * Function Name    : bt_fw_create_buffer_pools
 *
 * Description      : Creates ALL pools used by the system. Also, stores the
 *                      pool information - pool ids and # of buffers in a pool
 *                      info structure. This informatino is used to get the
 *                      number of free buffers for any pool at any point of time.
 *      
 * IMPORTANT NOTE   : The pool info structure is organized in a particular
 *                      way - if you notice, the first entry in this structure
 *                      is the acl pool id. Second is acl pool to host id.
 *                      There is a valid reason for this organization - Mostly,
 *                      the firmware requies the number of free buffers for ONLY
 *                      these pools. This is because flow control is done
 *                      only for ACL buffers. 
 *
 * Input Params     : None
 *
 * Output Results   : None
 *
 *****************************************************************************/
void bt_fw_create_buffer_pools( void )
{
    UINT16 buf_count;
    UINT16 buf_size;
    RAM_TYPE mem_type;
        
    /* Create LC Pools */
    /*
     * ACL Buffer in the host==> host controller direction. When there
     * is ACL data that arrives from the host, THIS buffer is used.
     * Will be ALLOCATED by the Transport driver, and FREED by the
     * MindTree task once the data is consumed.
     */
    buf_count = BT_FW_TOTAL_ACL_PKTS_FROM_HOST;
    buf_size = HCI_ACL_DATA_PKT_SIZE;
#ifndef _ENABLE_RETENTION_FLOW_FOR_DLPS_
    mem_type = RAM_TYPE_BUFFER_ON;
#else
    mem_type = RAM_TYPE_BUFFER_OFF;
#endif
        
	MEM_DBG_LOG(RED, ACL_DATA_TO_CONTROLL_POOL_ID, 0, 0);
    if ((OS_CREATE_POOL( &acl_data_pool_id,
                        buf_count,
                        buf_size,
                        EIGHT_BYTE_ALIGNMENT,
                        EIGHT_BYTES_ALIGN_COMPENDATE_SIZE,
                        mem_type)) != BT_ERROR_OK)
    {
		BT_FW_ERR(ACL_DATA_BUFFER_POOL_CREATION_FAILED,0,0);
        return;
    }
    else
    {
        BT_SW_POOL_LOG(GREEN, MSG_ALLOC_POOL_ACL_DATA_H2C, 3,
                  acl_data_pool_id, buf_count, buf_size);
    }

    /*
     * NOTE: The reason behind storing the acl pool id FIRST is to speed
     * up the linear search during allocing and freeing of the buffer.
     */
    /* LMP PDU buffer memory pool */
    buf_count = BT_FW_PDU_BUFFERS;
    buf_size = LMP_PDU_BUFFER_SIZE;
#ifndef _ENABLE_RETENTION_FLOW_FOR_DLPS_
    mem_type = RAM_TYPE_BUFFER_ON;
#else
    mem_type = RAM_TYPE_BUFFER_OFF;
#endif

    MEM_DBG_LOG(RED, LMP_PDU_BUF_POOL_ID, 0, 0);
    if ((OS_CREATE_POOL ( &lmp_pdu_buffer_pool_handle,
                        buf_count,
                        buf_size,
                        FOUR_BYTE_ALIGNMENT,
                        FOUR_BYTES_ALIGN_COMPENDATE_SIZE,
                        mem_type)) != BT_ERROR_OK)
    {
		BT_FW_ERR(BT_FW_STACK_INIT_LMP_PDU_BUFFER_CREATION_FAILED,0,0);
        return;
    }
    else
    {
        BT_SW_POOL_LOG(GREEN, MSG_ALLOC_POOL_LMP_BUFF, 3,
                  lmp_pdu_buffer_pool_handle, buf_count, buf_size);
    }

    /*
     * Command buffer pool. This pool will be utilized for storing
     * commands when they arrive from the host.
     * The Transport driver ALLOCATES memory using this handle, while
     * the FREEING of memory is done once the command has been executed.
     * Handles are shared across tasks using shared tables.
     */
    buf_count = BT_FW_CMD_BUFFERS;
    buf_size = HCI_CMD_BUFFER_SIZE;
#ifndef _ENABLE_RETENTION_FLOW_FOR_DLPS_
    mem_type = RAM_TYPE_BUFFER_ON;
#else
    mem_type = RAM_TYPE_BUFFER_OFF;
#endif

    MEM_DBG_LOG(RED, CMD_DATA_TO_CONTROLL_POOL_ID, 0, 0);
    if ((OS_CREATE_POOL( &hci_cmd_buffer_pool_handle,
                        buf_count,
                        buf_size,
                        FOUR_BYTE_ALIGNMENT,
                        FOUR_BYTES_ALIGN_COMPENDATE_SIZE,
                        mem_type)) != BT_ERROR_OK)
    {
		BT_FW_ERR(HCI_COMMAND_BUFFER_POOL_CREATION_FAILED,0,0);
        return;
    }
    else
    {
        BT_SW_POOL_LOG(GREEN, MSG_ALLOC_POOL_HCI_CMD, 3,
                  hci_cmd_buffer_pool_handle, buf_count, buf_size);
    }

    /*
     * ACL Buffer in the HOST CONTROLLER ==> HOST direction. Will
     * be ALLOCED by the LC RX Task, when there is data
     * available from the baseband.
     * FREED by the Transport driver, once the data is consumed.
     */
    buf_count = BT_FW_TOTAL_ACL_PKTS_TO_HOST;
    buf_size = HC_TO_HOST_ACL_DATA_PKT_SIZE;
#ifndef _ENABLE_RETENTION_FLOW_FOR_DLPS_
    mem_type = RAM_TYPE_BUFFER_ON;
#else
    mem_type = RAM_TYPE_BUFFER_OFF;
#endif

	MEM_DBG_LOG(RED, ACL_DATA_TO_HOST_POOL_ID, 0, 0);
    if ((OS_CREATE_POOL( &acl_data_to_host_pool_id,
                        buf_count,
                        buf_size,
                        0,
                        0,
                        mem_type)) != BT_ERROR_OK)                      
    {
		BT_FW_ERR(ACL_DATA_BUFFER_POOL_TO_HOST_CREATION_FAILED,0,0);
        return;
    }
    else
    {
        BT_SW_POOL_LOG(GREEN, MSG_ALLOC_POOL_ACL_DATA_C2H, 3,
                  acl_data_to_host_pool_id, buf_count, buf_size);
    }

    /* Event buffer pool:
     * This pool will be utilized for storing events when they are
     * generated by the BT Stack layers (HCI, LMP, LC).
     * The Transport driver FREES memory once the event is sent to
     * the host.
     * Handles are shared across tasks using shared tables.
     */
    buf_count = BT_FW_EVENT_BUFFERS;
    buf_size = HCI_EVENT_BUFFER_SIZE; 
    mem_type = RAM_TYPE_BUFFER_ON;

    MEM_DBG_LOG(RED, EVT_DATA_TO_HOST_POOL_ID, 0, 0);
    if ((OS_CREATE_POOL( &hci_event_buffer_pool_handle,
                        buf_count,
                        buf_size,
                        0,
                        0,
                        mem_type)) != BT_ERROR_OK)
    {
		BT_FW_ERR(HCI_EVENT_BUFFER_POOL_CREATION_FAILED,0,0);
        return;
    }
    else
    {
        BT_SW_POOL_LOG(GREEN, MSG_ALLOC_POOL_EVENT_BUFF, 3,
                  hci_event_buffer_pool_handle, buf_count, buf_size);
    }

    /* 
     * LMP FHS pkt memory pool
     */
    buf_count = BT_FW_FHS_PKT_BUFFERS;
    buf_size = LMP_FHS_PKT_BUFFER_SIZE;  
#ifndef _ENABLE_RETENTION_FLOW_FOR_DLPS_
    mem_type = RAM_TYPE_BUFFER_ON;
#else
    mem_type = RAM_TYPE_BUFFER_OFF;
#endif

    MEM_DBG_LOG(RED, LMP_FHS_PKT_BUF_POOL_ID, 0, 0);
    if ((OS_CREATE_POOL ( &lmp_fhs_pkt_buffer_pool_handle,
                        buf_count,
                        buf_size,
                        0,
                        0,
                        mem_type)) != BT_ERROR_OK)
    {
		BT_FW_ERR(LMP_FHS_PKT_BUFFER_POOL_CREATION_FAILED,0,0);
        return;
    }
    else
    {
        BT_SW_POOL_LOG(GREEN, MSG_ALLOC_POOL_FHS_BUFF, 3,
                  lmp_fhs_pkt_buffer_pool_handle, buf_count, buf_size);
    }

#ifdef LE_MODE_EN
	/* 
	 * LE TX packet pool
     */
    buf_count = LL_POLL_HCI_MAX_TX_ACL_PKT_CNT;
    buf_size = LL_POLL_HCI_MAX_TX_ACL_PKT_SIZE;  
#ifndef _ENABLE_RETENTION_FLOW_FOR_DLPS_
    mem_type = RAM_TYPE_BUFFER_ON;
#else
    mem_type = RAM_TYPE_BUFFER_OFF;
#endif

    MEM_DBG_LOG(RED, LE_TX_POOL_ID, 0, 0);
    if ((OS_CREATE_POOL( &le_acl_data_to_ll_pool_id,
                        buf_count,
                        buf_size,
                        EIGHT_BYTE_ALIGNMENT,
                        EIGHT_BYTES_ALIGN_COMPENDATE_SIZE,
                        mem_type)) != BT_ERROR_OK)
    {
        BT_FW_ERR(LE_MSG_ACL_DATA_H2L_POOL_CREATION_FAILED,0,0);
        return;
    }
    else
    {
        BT_SW_POOL_LOG(GREEN, MSG_ALLOC_POOL_LE_ACL_H2C, 3,
                  le_acl_data_to_ll_pool_id, buf_count, buf_size);
    }

	/* 
	 * LE LLC PDU control pool
     */
    buf_count = LL_POLL_CONTROL_PDU_BUFFER_NUM;
    buf_size = LL_POLL_CONTROL_PDU_BUFFER_SIZE;  
#ifndef _ENABLE_RETENTION_FLOW_FOR_DLPS_
    mem_type = RAM_TYPE_BUFFER_ON;
#else
    mem_type = RAM_TYPE_BUFFER_OFF;
#endif

    MEM_DBG_LOG(RED, LE_LLC_CONTROL_POOL_ID, 0, 0);
    if ((OS_CREATE_POOL( &le_llc_pdu_pool_id,
                        buf_count,
                        buf_size,
                        0,
                        0,
                        mem_type)) != BT_ERROR_OK)
    {
        BT_FW_ERR(LE_MSG_LLC_PDU_POOL_CREATION_FAILED,0,0);
        return;
    }
    else
    {
        BT_SW_POOL_LOG(GREEN, MSG_ALLOC_POOL_LE_LLC_PDU, 3,
                  le_llc_pdu_pool_id, buf_count, buf_size);
    }
     
	/* 
	 * LE Data to Host Pool ID
     */
    buf_count = LL_POLL_HCI_MAX_RX_ACL_PKT_CNT;
    buf_size = LL_POLL_HCI_MAX_RX_ACL_PKT_SIZE;  
#ifndef _ENABLE_RETENTION_FLOW_FOR_DLPS_
    mem_type = RAM_TYPE_BUFFER_ON;
#else
    mem_type = RAM_TYPE_BUFFER_OFF;
#endif

    MEM_DBG_LOG(RED, LE_RX_POOL_ID, 0, 0);
    if ((OS_CREATE_POOL( &le_acl_data_to_host_pool_id,
                        buf_count,
                        buf_size,
                        EIGHT_BYTE_ALIGNMENT,
                        EIGHT_BYTES_ALIGN_COMPENDATE_SIZE,
                        mem_type)) != BT_ERROR_OK)
    {
        BT_FW_ERR(LE_MSG_ACL_DATA_L2H_POOL_CREATION_FAILED,0,0);
        return;
    }
    else
    {
        BT_SW_POOL_LOG(GREEN, MSG_ALLOC_POOL_LE_ACL_C2H, 3,
                  le_acl_data_to_host_pool_id, buf_count, buf_size);
    }          
#endif

#if defined(SCO_OVER_HCI) || defined(COMPILE_ESCO)
    /*
     * SCO Buffer in the host==> host controller direction.
     * ALLOCED by Transport driver, FREED by LC Module after ACK received.
     */
    buf_count = BT_FW_TOTAL_SYNCHRONOUS_PKTS_FROM_HOST;
    buf_size = HCI_SYNCHRONOUS_DATA_PKT_SIZE; 
#ifndef _ENABLE_RETENTION_FLOW_FOR_DLPS_
    mem_type = RAM_TYPE_BUFFER_ON;
#else
    mem_type = RAM_TYPE_BUFFER_OFF;
#endif

    MEM_DBG_LOG(RED, SCO_DATA_TO_CONTROLL_POOL_ID, 0, 0);
    if ((OS_CREATE_POOL( &synchronous_data_pool_id,
                        buf_count,
                        buf_size,
                        FOUR_BYTE_ALIGNMENT,
                        FOUR_BYTES_ALIGN_COMPENDATE_SIZE,
                        mem_type)) != BT_ERROR_OK)
    {
		BT_FW_ERR(SCO_DATA_BUFFER_POOL_CREATION_FAILED,0,0);
        return;
    }
    else
    {
        BT_SW_POOL_LOG(GREEN, MSG_ALLOC_POOL_SCO_DATA_H2C, 3,
                  synchronous_data_pool_id, buf_count, buf_size);
    }
                                   
    /*
     * SCO Buffer in the HOST CONTROLLER ==> HOST direction.
     * ALLOCED by the MindTree task system, FREED by the Transport driver.
     */
    buf_count = BT_FW_TOTAL_SYNCHRONOUS_PKTS_TO_HOST;
    buf_size = HCI_SYNCHRONOUS_DATA_PKT_SIZE; 
#ifndef _ENABLE_RETENTION_FLOW_FOR_DLPS_
    mem_type = RAM_TYPE_BUFFER_ON;
#else
    mem_type = RAM_TYPE_BUFFER_OFF;
#endif

    MEM_DBG_LOG(RED, SCO_DATA_TO_HOST_POOL_ID, 0, 0);
    if ((OS_CREATE_POOL( &synchronous_data_to_host_pool_id,
                        buf_count,
                        buf_size,
                        0,
                        0,
                        mem_type)) != BT_ERROR_OK)
    {
		BT_FW_ERR(SCO_DATA_BUFFER_POOL_TO_HOST_CREATION_FAILED,0,0);
        return;
    }
    else
    {
        BT_SW_POOL_LOG(GREEN, MSG_ALLOC_POOL_SCO_DATA_C2H, 3,
                  synchronous_data_to_host_pool_id, buf_count, buf_size);
    }
#endif /* defined(SCO_OVER_HCI) || defined(COMPILE_ESCO) */
}

/******************************************************************************
 *
 * Function Name    : bt_fw_create_tasks
 *
 * Description      : Creates ALL the internal tasks in the MindTree task
 *                      system. Note that these tasks are not related to any
 *                      specific Operating System. The Transport driver task
 *                      is an internal task in the case where there is no
 *                      OS present. 
 *
 * Input Params     : None
 *
 * Output Results   : None
 *
 *****************************************************************************/

void bt_fw_create_stack_tasks( void )
{
    /* Create HCI Transport Driver Tasks */
    BT_FW_TASKS_PRIORITY task_priority ;

    task_priority = HCI_TD_TASK_PRI ;

    if(OS_CREATE_TASK(&hci_td_task_handle, HCI_TD_TASK_NAME,
                      (UCHAR)task_priority,
                      (OS_TASK_FUNCTION) hci_td_process_signal,
                      HCI_TD_TASK_Q_SIZE, HCI_TD_TASK_BUSY_PERIOD )
                        != BT_ERROR_OK )
    {
		BT_FW_ERR(CREATION_OF_HCI_COMMAND_TASK_FAILED,0,0);
        return;
    }

    /* Creating HCI command task */
    task_priority = HCI_CMD_TASK_PRI ;

    if(OS_CREATE_TASK(&hci_command_task_handle, HCI_CMD_TASK_NAME,
                      (UCHAR)task_priority, (OS_TASK_FUNCTION) HCI_Command_Task,
                      HCI_CMD_TASK_Q_SIZE, HCI_CMD_TASK_BUSY_PERIOD )
                        != BT_ERROR_OK )
    {
		BT_FW_ERR(CREATION_OF_HCI_COMMAND_TASK_FAILED,0,0);
        return;
    }

    /* Creating HCI event task */
    task_priority = HCI_EVENT_TASK_PRI ;

    if(OS_CREATE_TASK(&hci_event_task_handle, HCI_EVENT_TASK_NAME,
                      (UCHAR)task_priority,(OS_TASK_FUNCTION) HCI_Event_Task,
                      HCI_EVENT_TASK_Q_SIZE, HCI_EVENT_TASK_BUSY_PERIOD )
                        != BT_ERROR_OK)
    {
		BT_FW_ERR(CREATION_OF_HCI_COMMAND_TASK_FAILED,0,0);
        return;
    }

    /*
     * Create LMP Tasks 
     */

    /* Creating LMP task */
    task_priority =   LMP_TASK_PRI ;
    if(OS_CREATE_TASK( &lmp_task_handle, LMP_TASK_NAME,
                       (UINT8)task_priority, (OS_TASK_FUNCTION) LMP_Task,
                       LMP_TASK_Q_SIZE, LMP_TASK_BUSY_PERIOD ) != BT_ERROR_OK)
    {
		BT_FW_ERR(CREATION_OF_LMP_TASK_FAILED,0,0);
        return;
    }

    /* 
     * Create LC Tasks 
     */

    /* Creating LC TX task */
    task_priority = LC_TX_TASK_PRI ;

    if(OS_CREATE_TASK(&lc_tx_task_handle,LC_TX_TASK_NAME,
                      (UCHAR)task_priority,
                      (OS_TASK_FUNCTION) lc_tx_task,
                      (UINT32)LC_TX_TASK_Q_SIZE,
                      (UINT16)LC_TX_TASK_BUSY_PERIOD) != BT_ERROR_OK)
    {
        /* For the time being keep it HCI_log only */
		BT_FW_ERR(CREATION_OF_LC_TX_TASK_FAILED,0,0);
        return ;
    }

    /* Creating LC RX task */
    task_priority = LC_RX_TASK_PRI;

    if(OS_CREATE_TASK( &lc_rx_task_handle, LC_RX_TASK_NAME,
                       (UINT8)task_priority,
                       (OS_TASK_FUNCTION)lc_rx_task,
                       LC_RX_TASK_Q_SIZE,
                       LC_RX_TASK_BUSY_PERIOD) != BT_ERROR_OK)
    {
        /* For the time being keep it HCI_log only */
		BT_FW_ERR(CREATION_OF_LC_RX_TASK_FAILED,0,0);
        return ;
    }

#ifdef COMPILE_AFH_HOP_KERNEL
    task_priority = CH_AS_TASK_PRI ;

    if(OS_CREATE_TASK( &hci_ch_as_task_handle, LMP_CH_TASK_NAME,
                       (UINT8)task_priority,
                       (OS_TASK_FUNCTION)hci_ch_as_task,
                       CH_AS_TASK_Q_SIZE,
                       CH_AS_TASK_BUSY_PERIOD) != BT_ERROR_OK)
    {
        /* For the time being keep it HCI_log only */
		BT_FW_ERR(CREATION_OF_CH_AS_TASK_FAILED,0,0);
        return ;
    }
#endif

    return;
}


/******************************************************************************
 *                                                                            *
 * Function Name    : bt_fw_reset_buffer_pools
 * Description      : Resets the buffer pools that are used by the MindTree
 *                      system. This function will be called when the stack
 *                      receives a HCI-RESET command from the host. 
 *                      The order in which the pools are reset is insignificant.
 *
 * Input Params     : None
 *
 * Output Results   : None
 *
 *****************************************************************************/

void bt_fw_reset_buffer_pools(void)
{
    /* Reset the created memory pools */
    if(OS_RESET_POOL(&hci_event_buffer_pool_handle) != BT_ERROR_OK)
    {
		BT_FW_HCI_ERR(OS_RESET_POOL_FAILED,0,0);
    }

    if(OS_RESET_POOL(&hci_cmd_buffer_pool_handle) != BT_ERROR_OK)
    {
		BT_FW_HCI_ERR(OS_RESET_POOL_FAILED,0,0);
    }

    if(OS_RESET_POOL(&lmp_fhs_pkt_buffer_pool_handle) != BT_ERROR_OK)
    {
		LMP_ERR(OS_RESET_POOL_FAILED,0,0);
    }

    if(OS_RESET_POOL(&lmp_pdu_buffer_pool_handle) != BT_ERROR_OK )
    {
		LMP_ERR(OS_RESET_POOL_FAILED,0,0);
    }

    /*
     * ACL buffer in the HOST ==> HOST CONTROLLER direction. When there
     * is ACL data that arrives from the host, THIS buffer is used.
     * Will be ALLOCATED by the Transport driver, and FREED by the
     * MindTree task once the data is CONSUMED.
     */
    if(OS_RESET_POOL(&acl_data_pool_id) != BT_ERROR_OK )
    {
		LC_ERR(FATAL_FAILED_TO_CREATE_ACL_DATA_BUFFER_POOL,0,0);
    }   

#if defined(SCO_OVER_HCI) || defined(COMPILE_ESCO)
    /*
     * SCO Buffer in the HOST ==> HOST CONTROLLER direction.
     * ALLOCED by Transport driver, FREED by MindTree task system.
     */
    if(OS_RESET_POOL(&synchronous_data_pool_id) != BT_ERROR_OK )
    {
		LC_ERR(FATAL_FAILED_TO_CREATE_SCO_DATA_BUFFER_POOL,0,0);
    }   
#endif /* defined(SCO_OVER_HCI) || defined(COMPILE_ESCO) */

    /*
     * ACL Buffer in the HOST CONTROLLER ==> HOST direction. Will
     * be ALLOCED by the MindTree task system, when there is data
     * available from the baseband.
     * FREED by the Transport driver, once the data is consumed.
     */
    if(OS_RESET_POOL(&acl_data_to_host_pool_id) != BT_ERROR_OK )
    {
		LC_ERR(FATAL_FAILED_TO_CREATE_ACL_DATA_BUFFER_POOL_TO_HOST,0,0);
    }

#if defined(SCO_OVER_HCI) || defined(COMPILE_ESCO)
    /*
     * SCO Buffer in the HOST CONTROLLER ==> HOST direction.
     * ALLOCED by the MindTree task system, FREED by the Transport driver.
     */
    if(OS_RESET_POOL(&synchronous_data_to_host_pool_id) != BT_ERROR_OK )
    {
		LC_ERR(FATAL_FAILED_TO_CREATE_SCO_DATA_BUFFER_POOL_TO_HOST,0,0);
    }
#endif /* defined(SCO_OVER_HCI) || defined(COMPILE_ESCO) */

#ifdef LE_MODE_EN
    /* LE LLC PDU buffers pool */
    if (OS_RESET_POOL(&le_llc_pdu_pool_id) != BT_ERROR_OK)
    {
        BT_FW_HCI_ERR(OS_RESET_POOL_FAILED,0,0);
    }
  
    /* LE ACL Data buffers pool (HCI to LL)*/
    if (OS_RESET_POOL(&le_acl_data_to_ll_pool_id) != BT_ERROR_OK)
    {
        BT_FW_HCI_ERR(OS_RESET_POOL_FAILED,0,0);
    }

    /* LE ACL Data buffers pool (LL to HCI)*/
    if (OS_RESET_POOL(&le_acl_data_to_host_pool_id) != BT_ERROR_OK)
    {
        BT_FW_HCI_ERR(OS_RESET_POOL_FAILED,0,0);
    }
#endif

}

/******************************************************************************
 *
 * Function Name    : bt_fw_reset_tasks
 *
 * Description      :  Reset ALL the internal tasks in the BTC MindTree task
 *                      system. Note that these tasks are not related to any
 *                      specific Operating System. The Transport driver task
 *                      is an internal task in the case where there is no
 *                      OS present. 
 *
 * Input Params     : None
 *
 * Output Results   : None
 *
 *****************************************************************************/

void bt_fw_reset_stack_tasks( void )
{
    OS_RESET_TASK(hci_td_task_handle);
    OS_RESET_TASK(hci_command_task_handle);
    OS_RESET_TASK(hci_event_task_handle);
    OS_RESET_TASK(lmp_task_handle);
    OS_RESET_TASK(lc_tx_task_handle);
    OS_RESET_TASK(lc_rx_task_handle);
#ifdef COMPILE_AFH_HOP_KERNEL
    OS_RESET_TASK(hci_ch_as_task_handle);
#endif
    return;
}

/******************************************************************************
 *                                                                            *
 * Function Name    : bt_fw_stack_reset
 *
 * Description      : This function is called when the stack recieves a
 *                      HCI-RESET command from the host. Calls functions that
 *                      initializes the pools and relevant data structures for
 *                      this condition.
 *
 * Input Params     : None
 *
 * Output Results   : None
 *
 *****************************************************************************/

void bt_fw_stack_reset(void)
{
 //   os_reset_heap_pointer();  // deleted by guochunxia 20090701
    bt_fw_reset_buffer_pools();
    bt_fw_reset_stack_tasks();
    hci_td_stack_interface_init();
}

/**************************************************************************
 * Function     : bt_fw_stack_init
 *
 * Description  : This function initializes HCI Transport driver, HCI, LMP
 *                  and LC modules
 *
 * Parameters   : None
 *
 * Returns      : 0
 *
 *************************************************************************/

void bt_fw_stack_init( void )
{
    /* Create Buffer Pools */
    bt_fw_create_buffer_pools();
    /* Create the Tasks */
    bt_fw_create_stack_tasks();
    /* Initialize the Inter-Task Communication Objects */
    hci_td_stack_interface_init();
}

