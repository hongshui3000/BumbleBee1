/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  Contains the code for the HCI command task and HCI event
 *  task routines of HCI module.
 */

/********************************* Logger *************************/
enum { __FILE_NUM__= 11};
/********************************* Logger *************************/

/* Module header file inclusion */
#include "bt_fw_hci_internal.h"
#include "vendor.h"
#include "bt_fw_hci_2_1_cmds.h"
#include "mem.h"
#include "UartPrintf.h"
#include "bz_fw_isoch.h"
#include "hci_vendor_defines.h"
#include "lmp_vendor_defines.h"
#include "bt_fw_acl_q.h"
#ifdef LE_MODE_EN
#include "le_hci_4_0.h"
#endif
#ifdef _3DD_FUNCTION_SUPPORT_
#include "bt_3dd.h"
#endif
#include "bt_fw_hci.h"

/* ====================== Macro Declaration Section ======================= */
#define LOCAL_CLOCK     0x00
#define PICONET_CLOCK   0x01

/* ==================== Structure Declaration Section ===================== */


/* ===================== Variable Declaration Section ===================== */
/**
 * Defines the flag used to check the multiple inquiries from the host.
 * This Flag is set when inquiry command is received and reset when
 * the inquiry complete event is sent.
 */
#ifdef _DAPE_RECORD_COMMAND_POINTER_FOR_EVENT
HCI_CMD_PKT *g_record_hci_cmd_pkt_ptr = NULL;
#endif

UCHAR hci_baseband_cmd_pending = FALSE;
#ifdef _DAPE_DETECT_WHCK_P2P
UINT8 g_whck_running = 0;
UINT8 g_whck_item = 0;
extern UINT16 g_whck_p2p_running_detect_countdown;
extern UINT8 g_p2p_detect_step;
#endif

/**
 * Defines the Received Data Queue used to Queue the packets from
 * host controller to Host.
 */
HCI_DATA_RECV_Q hci_received_data_Q;

UCHAR rem_name_cancel_bd_addr[LMP_BD_ADDR_SIZE]= {0};

/* ================== Static Function Prototypes Section ================== */


#/* ================== For rom code patch function point ================== */
#ifdef _ROM_CODE_PATCHED_
#ifndef _REDUCE_LPS_AFTER_RTL8703B_
#ifdef _CCH_8723_RCP_
PF_ROM_CODE_PATCH_FUNC rcp_hci_handle_remote_name_request_cancel_command = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_hci_execute_link_control_command_packet_case = NULL;
#endif
#endif
PF_ROM_CODE_PATCH_FUNC rcp_hci_command_handle = NULL;
#endif

/* ===================== Function Definition Section ====================== */
/**
 * Receives a signal from the transport driver
 * indicating that it received the complete command packet from
 * the host.The HCI command task receives HCI_CMD_RECD_SIGNAL
 * signal from the transport driver.
 *
 * \param signal_ptr Pointer to OS_SIGNAL.
 *
 * \return SUCCESS or ERROR.
 *
 */
UCHAR HCI_Command_Task(OS_SIGNAL *signal_ptr)
{
    UCHAR cmd_group_opcode ;
    HCI_CMD_PKT *hci_cmd_pkt_ptr ;

    BT_FW_HCI_INF(HCI_COMMAND_TASK_SIGNAL_RECD,0,0);

    hci_cmd_pkt_ptr = (HCI_CMD_PKT *)signal_ptr->param ;
#ifdef _DAPE_RECORD_COMMAND_POINTER_FOR_EVENT
    g_record_hci_cmd_pkt_ptr = hci_cmd_pkt_ptr;
#endif
#ifdef _CCH_LPS_
    if(g_efuse_lps_setting_2.force_exit_sm_en)
    {
//        LPS_DBG_LOG(YELLOW, LPS_LOG_027, 1,sleep_mode_param.bb_sm_sts);
        LC_EXIT_SM_MODE();
    }
#endif


    switch(signal_ptr->type)
    {
        case SIGNAL_BEGIN:
            //BT_FW_HCI_INF(INITILIZING_HCI_MODULE,0,0);
            break;

        case SIGNAL_END :
            hci_shutdown();
            break;

        case HCI_CMD_RECD_SIGNAL :

            /* If we get the null command packet simply reject that packet.*/
            if (hci_cmd_pkt_ptr == NULL)
            {
                if (signal_ptr->length == 0)
                {
                    UINT32 context = (UINT32)signal_ptr->ext_param;
                    UINT16 index = context & 0xFFFF;
                    UINT16 opcode = context >> 16;

                    if (opcode == HCI_SWITCH_ROLE_OPCODE)
                    {
#ifdef COMPILE_ROLE_SWITCH
                        return hci_handle_switch_role_command_bottom_half(index);
#endif
                    }
#ifdef COMPILE_PARK_MODE
                    else if (opcode == HCI_PARK_MODE_OPCODE)
                    {
                        return hci_handle_park_mode_command_bottom_half(index);
                    }
#endif
                }
                HCI_LOG_ERROR(LOG_LEVEL_LOW, HCI_COMAMND_TASK_NULL_COMMAND_PACKET_RECEIVED,0,0);
                return BT_FW_ERROR;
            }

#ifdef _FIX_RACE_CONDITION_ISSUE_OF_NUM_HCI_CMD_PKT_FIELD_
            lmp_self_device_data.num_hci_command_packets--;
#endif

#ifdef TEST_MODE
            if (lmp_self_device_data.test_mode == HCI_LOCAL_LOOPBACK_MODE)
            {
                /* Check whether the cmd received is to be looped back */
                if ((hci_cmd_pkt_ptr->cmd_opcode != HCI_RESET_OPCODE) &&
                        (hci_cmd_pkt_ptr->cmd_opcode !=
                         HCI_SET_CONTROLLER_TO_HOST_FLOW_CONTROL_OPCODE)&&
                        (hci_cmd_pkt_ptr->cmd_opcode !=
                         HCI_HOST_BUFFER_SIZE_OPCODE) &&
                        (hci_cmd_pkt_ptr->cmd_opcode !=
                         HCI_HOST_NUMBER_OF_COMPLETED_PACKETS_OPCODE) &&
                        (hci_cmd_pkt_ptr->cmd_opcode !=
                         HCI_READ_BUFFER_SIZE_OPCODE) &&
                        (hci_cmd_pkt_ptr->cmd_opcode !=
                         HCI_READ_LOOPBACK_MODE_OPCODE) &&
                        (hci_cmd_pkt_ptr->cmd_opcode !=
                         HCI_WRITE_LOOPBACK_MODE_OPCODE))
                {
                    /* Generate a loopback command event to send it
                     * back to host */
                    hci_generate_loopback_command_event(hci_cmd_pkt_ptr);

                    lmp_self_device_data.num_hci_command_packets++;

                    dma_tx_fifo_pkt_free((void * )hci_cmd_pkt_ptr,
                                             HCI_TRANSPORT_CMD_PKT_TYPE);
                    return BT_FW_SUCCESS;
                }
            }
#endif /* TEST_MODE */

#ifdef _SUPPORT_AUTO_DETACH_LINK_
            if (((!IS_USE_FOR_BQB) && (!IS_USE_FOR_MUTE)) &&
                (hci_cmd_pkt_ptr->cmd_opcode == HCI_RESET_OPCODE) &&
                ((UINT32)catch_os_signal.param == 0))
            {
                /* disable scan first. this will avoid watchdog reset when some remote device
                   trying to re-connect us after we do the auto detach procedure.
                   (supervision timeout is restart right after HLC interrupt and we reset timer in
                   HCI reset, which will cause unexpected error in timer link). */
                lmp_self_device_data.scan_enable = 0;
                lc_start_write_scan_mode(0);

                if (auto_detach_terminate_all_remote_links() > 0)
                {
                    memcpy(&catch_os_signal, signal_ptr, sizeof(OS_SIGNAL));
                    auto_detach_enable_link_timer(
                                AUTO_DETACH_PROCESS_HCI_RESET_CMD, 3000);
                    return BT_FW_SUCCESS;
                }
            }
#endif

            /* Get the HCI command group opcode from the command packet */
            // get OGF  cmd_opcode>>10
            cmd_group_opcode = (UCHAR) HCI_GET_GROUP_OPCODE(
                                   hci_cmd_pkt_ptr->cmd_opcode);
            //by liuyong 20090727


            RT_BT_LOG(YELLOW, BT_FW_HCI_TASKS_146, 23,
                      BB_read_native_clock(),
                      hci_cmd_pkt_ptr->cmd_opcode,
                      hci_cmd_pkt_ptr->param_total_length,
                      hci_cmd_pkt_ptr->cmd_parameter[0],
                      hci_cmd_pkt_ptr->cmd_parameter[1],
                      hci_cmd_pkt_ptr->cmd_parameter[2],
                      hci_cmd_pkt_ptr->cmd_parameter[3],
                      hci_cmd_pkt_ptr->cmd_parameter[4],
                      hci_cmd_pkt_ptr->cmd_parameter[5],
                      hci_cmd_pkt_ptr->cmd_parameter[6],
                      hci_cmd_pkt_ptr->cmd_parameter[7],
                      hci_cmd_pkt_ptr->cmd_parameter[8],
                      hci_cmd_pkt_ptr->cmd_parameter[9],
                      hci_cmd_pkt_ptr->cmd_parameter[10],
                      hci_cmd_pkt_ptr->cmd_parameter[11],
                      hci_cmd_pkt_ptr->cmd_parameter[12],
                      hci_cmd_pkt_ptr->cmd_parameter[13],
                      hci_cmd_pkt_ptr->cmd_parameter[14],
                      hci_cmd_pkt_ptr->cmd_parameter[15],
                      hci_cmd_pkt_ptr->cmd_parameter[16],
                      hci_cmd_pkt_ptr->cmd_parameter[17],
                      hci_cmd_pkt_ptr->cmd_parameter[18],
                      hci_cmd_pkt_ptr->cmd_parameter[19]);

#ifdef _ROM_CODE_PATCHED_
            if (rcp_hci_command_handle != NULL)
            {
                if (rcp_hci_command_handle((void *)hci_cmd_pkt_ptr,
                                        &cmd_group_opcode))
                {
                    cmd_group_opcode = RTK_VENDOR_PARAMS_OPCODE_GROUP;
                }
            }
#endif
#ifdef _DAPE_DETECT_WHCK_P2P
            detect_whck_p2p(hci_cmd_pkt_ptr, &cmd_group_opcode);
#endif

            switch(cmd_group_opcode)
            {
                case LINK_CONTROL_CMD_OPCODE_GROUP:
                    hci_execute_link_control_command_packet(hci_cmd_pkt_ptr);
                    break ;

                case LINK_POLICY_CMD_OPCODE_GROUP :
                    hci_execute_link_policy_command_packet(hci_cmd_pkt_ptr);
                    break;

                case HC_AND_BASEBAND_CMD_OPCODE_GROUP :
                    hci_execute_HC_and_baseband_command_packet(hci_cmd_pkt_ptr);
                    break;

                case INFO_PARMS_OPCODE_GROUP :
                    hci_execute_info_params_command_packet(hci_cmd_pkt_ptr);
                    break;

                case STATUS_CMD_OPCODE_GROUP :
                    hci_execute_status_command_packet(hci_cmd_pkt_ptr);
                    break;

#ifdef TEST_MODE
                case TEST_MODE_OPCODE_GROUP :
                    hci_execute_test_mode_command_packet(hci_cmd_pkt_ptr);
                    break;
#endif /* TEST_MODE */

#if defined(RT_VENDOR_CMDS)
                case VENDOR_PARMS_OPCODE_GROUP :
                    hci_handle_vendor_cmds(hci_cmd_pkt_ptr);
                    /* HCI_EXECUTE_VENDOR_COMMAND_PACKET(hci_cmd_pkt_ptr); */
                    break;
#endif//RT_VENDOR_CMDS

#ifdef LE_MODE_EN
                case LE_CTRL_CMD_OPCODE_GROUP :
                    if (IS_BT40)
                    {
                        hci_execute_4_0_hc_le_control_command(hci_cmd_pkt_ptr);
                    }
                    else
                    {
                        /*
                         * Standard BT specification error  : Error code 0x01
                         * specification says you can send either command status or
                         * command complete event.
                         */
                        /* We should send back the same command opcode */
                        hci_generate_command_status_event(
                            hci_cmd_pkt_ptr->cmd_opcode,
                            UNKNOWN_HCI_COMMAND_ERROR);
                    }
                    break;
#endif

#ifdef _ROM_CODE_PATCHED_
                case RTK_VENDOR_PARAMS_OPCODE_GROUP:
                    break;
#endif

                default :
                    /*
                     * Standard BT specification error  : Error code 0x01
                     * specification says you can send either command status or
                     * command complete event.
                     */
                    /* We should send back the same command opcode */
                    hci_generate_command_status_event(
                        hci_cmd_pkt_ptr->cmd_opcode,
                        UNKNOWN_HCI_COMMAND_ERROR);
                    break;
            }

            /* Free Command buffer */
            lmp_self_device_data.num_hci_command_packets++;

            pf_tx_fifo_order_free(signal_ptr->fifo_index, signal_ptr->fifo_type);
#ifdef _DAPE_RECORD_COMMAND_POINTER_FOR_EVENT
            g_record_hci_cmd_pkt_ptr = NULL;
#endif
            break;

        default :
            /* Not the registered signal.  */
            BT_FW_HCI_ERR(HCI_COMMAND_TASK_RECEIVED_UNKNOWN_SIGNAL,0,0);
            break;
    }

    return BT_FW_SUCCESS;
}

/**
 * Receives a signals for delivering the
 * HCI events, ACL data, SCO data to the host.
 *
 * \param signal_ptr Pointer to OS_SIGNAL.
 *
 * \return None.
 *
 */
void HCI_Event_Task(OS_SIGNAL *signal_ptr)
{
    HCI_EVENT_PKT *phci_event;

    BT_FW_HCI_INF(HCI_EVENT_TASK_SIGNAL_RECD,0,0);

    switch(signal_ptr->type)
    {
        case SIGNAL_BEGIN:
            break;

        case SIGNAL_END:
            break;

        case HCI_DELIVER_HCI_EVENT_SIGNAL:
            /* Before delivering the event to the host check the mask. */
            phci_event = (HCI_EVENT_PKT *)signal_ptr->param;
            if (hci_pass_event_through_event_mask(phci_event->event_opcode))
            {
                //by liuyong 20090727
                if (phci_event->event_opcode!=0x13)
                {
                    RT_BT_LOG(WHITE, BT_FW_HCI_TASKS_260, 23,
                              BB_read_native_clock(),
                              phci_event->event_opcode,
                              phci_event->param_total_length,
                              phci_event->event_parameter[0],
                              phci_event->event_parameter[1],
                              phci_event->event_parameter[2],
                              phci_event->event_parameter[3],
                              phci_event->event_parameter[4],
                              phci_event->event_parameter[5],
                              phci_event->event_parameter[6],
                              phci_event->event_parameter[7],
                              phci_event->event_parameter[8],
                              phci_event->event_parameter[9],
                              phci_event->event_parameter[10],
                              phci_event->event_parameter[11],
                              phci_event->event_parameter[12],
                              phci_event->event_parameter[13],
                              phci_event->event_parameter[14],
                              phci_event->event_parameter[15],
                              phci_event->event_parameter[16],
                              phci_event->event_parameter[17],
                              phci_event->event_parameter[18],
                              phci_event->event_parameter[19]);
                }
                hci_td_deliver_event_to_host((UCHAR *)phci_event);
            }
            else
            {
                /* Free the buffer allocated by
                bt_fw_hci_events.c : hci_generate_event() */
                OS_FREE_BUFFER(tx_table[HCI_EVENT_HANDLER_TASK].pool_handle,
                                  signal_ptr->param);
            }
            break;

        case HCI_DELIVER_ACL_DATA_SIGNAL:
#ifdef HS_USB_SEND_EVENT_REORDER_ISSUE_NEW

        if ((g_fun_interface_info.b.hci_sel == PACKAGE_USB_USB)
                    || IS_EVENT_REORDER_FOR_ANY_INTERFACE)
            {
                HCI_ACL_RX_DATA_PKT *pkt = (HCI_ACL_RX_DATA_PKT *) signal_ptr->param;
                ACL_RX_DATA_WAIT_QUEUE *wq = aclq_rx_wait_queue_find(pkt->connection_handle);
                if (wq != NULL)
                {
                    if (wq->send == NULL)
                    {
                        wq->send = (ACL_RX_DATA_WAIT_QUEUE_SEND_FUNC) aclq_send_acl_rx_pkt_in_wait_queue;
                    }
                    aclq_rx_wait_queue_enqueue(wq, pkt);
                    break;
                }
            }
#endif
#ifdef HS_USB_SEND_EVENT_REORDER_ISSUE
            if ((g_fun_interface_info.b.hci_sel == PACKAGE_USB_USB)
                || IS_EVENT_REORDER_FOR_ANY_INTERFACE)
            {
                HCI_ACL_RX_DATA_PKT *acl_pkt = (HCI_ACL_RX_DATA_PKT *) signal_ptr->param;

                if (acl_pkt->connection_handle == hs_recd_conn_handle)
                {
                    /* queue non-ready acl pkt to the waiting listing */
                    acl_pkt->next = NULL;
                    if (hs_wait_queue_acl_head == NULL)
                    {
                        hs_wait_queue_acl_head = acl_pkt;
                    }
                    else
                    {
                        hs_wait_queue_acl_tail->next = acl_pkt;
                    }
                    hs_wait_queue_acl_tail = acl_pkt;
                    return;
                }
            }
#endif
            hci_handle_data_to_host((HCI_ACL_RX_DATA_PKT *)signal_ptr->param);
            break;

        case HCI_HANDLE_DATA_TX_SIGNAL:
            hci_handle_data_tx_completed((UINT16)((UINT32)signal_ptr ->param));
            break;

        case HCI_DELIVER_SYNCHRONOUS_DATA_SIGNAL:
#if defined(SCO_OVER_HCI) || defined(COMPILE_ESCO)
            hci_td_deliver_synchronous_data_to_host(
                (UCHAR *)signal_ptr->param);   // for loop back test
#endif /* SCO_OVER_HCI | COMPILE_ESCO */
            break;

#ifdef HS_USB_SEND_EVENT_REORDER_ISSUE_NEW
        case HCI_DELIVER_LE_ACL_DATA_SIGNAL:
            if (ll_manager.conn_unit.l2h_acl_pkt_list.in_procedure)
            {
                if ((g_fun_interface_info.b.hci_sel == PACKAGE_USB_USB)
                        || IS_EVENT_REORDER_FOR_ANY_INTERFACE)
                {
                    LL_HCI_ACL_DATA_PKT *acl_pkt;
                    LL_HCI_ACL_DATA_PKT *acl_pkt_next;
                    LL_CONN_HANDLE_UNIT *handle;
                    UINT32 pool_id = tx_table[LE_ACL_DATA_HANDLER_TASK].pool_handle;

                    acl_pkt = ll_manager.conn_unit.l2h_acl_pkt_list.pDataHead;

                    for ( ; acl_pkt != NULL; acl_pkt = acl_pkt_next)
                    {
                        acl_pkt_next = acl_pkt->next;
                        acl_pkt->next = NULL;

                        ACL_RX_DATA_WAIT_QUEUE *wq = aclq_rx_wait_queue_find(
                                acl_pkt->connection_handle);
                        if (wq != NULL)
                        {
                            if (wq->send == NULL)
                            {
                                wq->send = (void (*)(void *)) aclq_send_le_acl_rx_pkt_in_wait_queue;
                            }
                            aclq_rx_wait_queue_enqueue_le(wq, acl_pkt);
                            continue;
                        }

                        handle = ll_fw_search_handle_unit_via_conn_handle(
                                acl_pkt->connection_handle);

                        if ((handle != NULL) && handle->connected)
                        {
#ifndef LE_HW_TEST
                            hci_handle_data_to_host((HCI_ACL_RX_DATA_PKT *) acl_pkt);
#else
                            ll_test_receive_rx_data_pkt(acl_pkt);
#endif
                        }
                        else
                        {
                            OS_FREE_BUFFER(pool_id, acl_pkt);

#ifdef _ENABLE_BLE_LL_FLOW_CONTROL_
                            os_release_one_le_reserved_buffer();
#endif

                        }
                    }

                    ll_manager.conn_unit.l2h_acl_pkt_list.pDataHead = NULL;
                    ll_manager.conn_unit.l2h_acl_pkt_list.pDataTail = NULL;
                    ll_manager.conn_unit.l2h_acl_pkt_list.pktcnt = 0;
                    ll_manager.conn_unit.l2h_acl_pkt_list.in_procedure = FALSE;
                }
            }
            break;
#endif

#ifdef SCO_OVER_HCI
#ifdef _FIX_MULTIPLE_SCO_RECORD_PATH_
        case HCI_SEND_SYNCHRONOUS_PACKET_SIGNAL:
            bz_isoch_dequeue_and_send_data_to_host();
            break;
#endif
#endif

        case HCI_DISCONNECT_COMPLETE_SIGNAL:
            lmp_cleanup_after_acl_detach((UINT16)((UINT32)signal_ptr->param));
            break;

#ifdef COMPILE_PARK_MODE
        case HCI_UNPARK_NEW_CONN_TIMEOUT :
            hci_unpark_new_conn_timeout_event
            ((UINT16)((UINT32)signal_ptr->param));
            break;
#endif /* COMPILE_PARK_MODE */

        case HCI_GEN_QOS_COMPLETE_EVENT_SIGNAL:
            hci_generate_QoS_setup_complete_event(
                (UCHAR)((UINT32)signal_ptr->ext_param),
                (UINT16)((UINT32)signal_ptr->param) );
            break;

        case HCI_GEN_FLOW_SPEC_COMPLETE_EVENT_SIGNAL:
            {
                UCHAR status;
                UCHAR ce_index;
                UCHAR flow_direction;

                status = (UCHAR)((UINT32)signal_ptr->ext_param);
                flow_direction = ((UINT16)((UINT32)signal_ptr->ext_param)) >> 8;
                ce_index = (UCHAR)((UINT32)signal_ptr->param);

                hci_generate_flow_spec_complete_event(status,
                                                      flow_direction, ce_index);
            }
            break;

        case HCI_GEN_CONN_PKT_TYPE_CHGD_EVENT_SIGNAL:
            {
                UINT16 ce_index;
                LMP_CONNECTION_ENTITY *ce_ptr;

                ce_index = (UINT16)((UINT32)signal_ptr->param);
                ce_ptr = &lmp_connection_entity[ce_index];

                hci_generate_connection_packet_type_changed_event(
                    HCI_COMMAND_SUCCEEDED,
                    ce_ptr->connection_type.connection_handle,
                    ce_ptr->connection_type.packet_type);
            }
            break;

        case HCI_GEN_ACTIVE_MODE_CHANGED_EVENT_SIGNAL:
            {
                UINT16 ce_index;

                ce_index = (UINT16)((UINT32)signal_ptr->param);

                /* Generate mode change event. */
                hci_generate_mode_change_event(HCI_COMMAND_SUCCEEDED, ce_index,
                                               LP_ACTIVE_MODE, 0x00);
            }
            break;

        case HCI_HOST_EVENT_MASKED_SIGNAL:
            /* Handle the masked event by performing proper action. */
            hci_handle_masked_event(
                (UINT16)((UINT32)signal_ptr->param),
                (UCHAR)((UINT32)signal_ptr->ext_param));
            break;


        default :
            /* Not a registered signal. */
            BT_FW_HCI_ERR(HCI_EVENT_TASK_RECEIVED_UNKNOWN_SIGNAL,0,0);
            break;
    }
}

/**
 * Verifies the Link Control group commands received from
 * the host. It extracts the command opcode from the command packet and
 * analyzes the command packet depending on the type of link control
 * command it calls the corresponding routines.
 *
 * \param hci_cmd_ptr Pointer to HCI_CMD_PKT.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI Error code.
 *
 */
UCHAR hci_execute_link_control_command_packet (HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR ret_error = HCI_COMMAND_SUCCEEDED ;

    /*
     * If the Commands passed to the baseband set this flag.
     * It is used to indicate LC module to delete the command buffer.
     */
    UCHAR sent_event_flag = FALSE ;

    /*
     * The command complete event flag is set to indicate command complete
     * event to be sent to the host.
     *
     * This flag is set for the following Link Control commands.
     * 1.Inquiry cancel.
     * 2.Link Key request reply
     * 3.Link Key request negative reply
     * 4.PIN code request reply
     * 5.PIN code request negative reply
     */
    UCHAR command_complete_event_flag = FALSE;
    /* This variable is a flag to generate CCPT event */
    UCHAR hci_generate_ccpt_event_flag = 0x0;
    UCHAR hci_max_slots_change_event_flag = 0;

    BT_FW_HCI_INF(HCI_LINK_CONTROL_GROUP_COMMAND_FUNCTION,0,0);

#ifdef _MORGAN_ENABLE_WATCH_DOG_TEST_
    WDG_TIMER_TIMEOUT_SOON;
    while (1); /* infinite loop here to wait watch dog timeout */
#endif

    switch(hci_cmd_ptr->cmd_opcode)
    {
    case HCI_INQUIRY_OPCODE :
        /*If previous inquiry is pending reject the current inquiry
        * command. Send command disallowed event error status to
        * the host */
        if (hci_baseband_cmd_pending != TRUE)
        {
            ret_error = hci_handle_inquiry_command(hci_cmd_ptr);
            if (ret_error == HCI_COMMAND_SUCCEEDED)
            {
                BT_FW_HCI_INF(HCI_INQUIRY_COMMAND_SUCCESSFULL,0,0);
                hci_baseband_cmd_pending = TRUE ;

                /* Successfull Command status event is sent to the host
                 * if the baseband indicates the success for inquiry
                 * programing....*/
                sent_event_flag = TRUE ;
                lmp_self_device_data.device_status = LMP_INQUIRY ;
            }
        }
        else
        {
            BT_FW_HCI_INF(HCI_INQUIRY_COMMAND_NOT_ALLOWED,0,0);
            ret_error = COMMAND_DISALLOWED_ERROR ;
        }
        break;

    case HCI_CREATE_CONNECTION_OPCODE :
        if (hci_baseband_cmd_pending != TRUE)
        {
            RT_BT_LOG(GREEN, BT_FW_HCI_TASKS_458, 0, 0);

            ret_error = hci_handle_create_connection_command(hci_cmd_ptr);
            if (ret_error == HCI_COMMAND_SUCCEEDED)
            {
                BT_FW_HCI_INF(HCI_CREATE_CONNECTION_STARTED,0,0);
                sent_event_flag = TRUE ;
                lmp_self_device_data.device_status = LMP_PAGE ;
                hci_baseband_cmd_pending = TRUE ;
            }
            else
            {
                BT_FW_HCI_ERR(HCI_HANDLE_CREATE_CONNECTION_COMMAND_FAILED,0,0);
            }
        }
        else
        {
            ret_error = COMMAND_DISALLOWED_ERROR ;
        }
        break;

    case HCI_ACCEPT_CONNECTION_REQUEST_OPCODE :
        ret_error = hci_handle_accept_connection_request_command(
                        hci_cmd_ptr, &sent_event_flag);
        if (ret_error != HCI_COMMAND_SUCCEEDED)
        {
            BT_FW_HCI_ERR(ACCEPT_CONNECTION_REQUEST_COMMAND_FAILED,0,0);
        }
        break;

    case HCI_REJECT_CONNECTION_REQUEST_OPCODE :
        ret_error = hci_handle_reject_connection_request_command(
                        hci_cmd_ptr, &sent_event_flag);
        if (ret_error != HCI_COMMAND_SUCCEEDED)
        {
            BT_FW_HCI_ERR(REJECT_CONNECTION_REQUEST_COMMAND_FAILED,0,0);
        }
        break;

    case HCI_DISCONNECT_OPCODE :
        RT_BT_LOG(GREEN, BT_FW_HCI_TASKS_499, 0, 0);

        ret_error = hci_handle_disconnect_command(hci_cmd_ptr,
                    &sent_event_flag );
        if (ret_error != HCI_COMMAND_SUCCEEDED)
        {
            BT_FW_HCI_ERR(DISCONNECT_COMMAND_FAILED,0,0);
        }

        break;

#if defined(ADD_SCO) && defined(ENABLE_SCO)
    case HCI_ADD_SCO_CONNECTION_OPCODE:
        ret_error = hci_handle_add_sco_connection_command(hci_cmd_ptr,
                    &sent_event_flag);
        if (ret_error != HCI_COMMAND_SUCCEEDED)
        {
            BT_FW_HCI_ERR(ADD_SCO_CONNECTION_COMMAND_FAILED,0,0);
        }
        break;
#endif /* ADD_SCO */

#ifdef _CCH_RTL8723A_B_CUT
    case HCI_CREATE_CONNECTION_CANCEL_OPCODE:

        memcpy(rem_name_cancel_bd_addr, &hci_cmd_ptr->cmd_parameter[0], LMP_BD_ADDR_SIZE);
        ret_error = lc_kill_paging(rem_name_cancel_bd_addr, CON_REASON_CREATE_CON);

        /* Set this flag for sending command complete event. */
        command_complete_event_flag = 1 ;
        break;
#endif

    case HCI_INQUIRY_CANCEL_OPCODE :
        /*
         * Only if inquiry is already issued. kill it. otherwise
         * return Command disallowed error to the host
         */
        if (hci_baseband_cmd_pending == TRUE)
        {
            lc_kill_inquiry();

            /* Reset the inquiry result table */
            memset(lmp_inquiry_result_data, 0,
                   LMP_MAX_INQUIRY_RESULT_DATA *sizeof(LMP_INQUIRY_RESULT_DATA));

            /*
             * Clear the inquiry reset ,baseband cmd pending flag.
             */
            hci_baseband_cmd_pending = FALSE ;

            lmp_num_inq_resp_received = 0;

        }
        else
        {
            ret_error = COMMAND_DISALLOWED_ERROR ;
        }

        /* Set this flag for sending command complete event. */
        command_complete_event_flag = 1 ;
        break;

    case HCI_PERIODIC_INQUIRY_MODE_OPCODE:

#ifdef COMPILE_PERIODIC_INQUIRY
        /* If previous inquiry is pending reject the current inquiry
         * command. Send command disallowed event error status
         * to the host. */
        if (hci_baseband_cmd_pending != TRUE)
        {
            ret_error = hci_handle_periodic_inquiry_command(hci_cmd_ptr);

            if (ret_error == HCI_COMMAND_SUCCEEDED)
            {
                BT_FW_HCI_INF(HCI_INQUIRY_COMMAND_SUCCESSFULL,0,0);
                /* Inquiry is issued. No more inquiry command */
                /* Command issued for the baseband . Do n't allow
                 * inquiry and page commands*/
                hci_baseband_cmd_pending = TRUE ;
                lmp_self_device_data.device_status = LMP_PERIODIC_INQUIRY;
            }
        }
        else
        {
            BT_FW_HCI_INF(HCI_INQUIRY_COMMAND_NOT_ALLOWED,0,0);
            ret_error = COMMAND_DISALLOWED_ERROR ;
        }
        command_complete_event_flag = TRUE ;
#else /* COMPILE_PERIODIC_INQUIRY */
        ret_error = UNKNOWN_HCI_COMMAND_ERROR;
        command_complete_event_flag = TRUE;
#endif /* COMPILE_PERIODIC_INQUIRY */

        break;

    case HCI_EXIT_PERIODIC_INQUIRY_MODE_OPCODE :
        /*
         *  Before issuing inquiry, if the host sends exit inquiry send
         *  Command disallowed error to the host.
         */
        switch(lmp_self_device_data.device_status)
        {
#ifdef COMPILE_PERIODIC_INQUIRY
            case LMP_PERIODIC_INQUIRY :
                if (lc_kill_periodic_inquiry()!= API_SUCCESS)
                {
                    BT_FW_HCI_ERR(HCI_PERIODIC_INQUIRY_MODE_COMMAND_FAILED,0,0);
                    ret_error = HARDWARE_FAILURE_ERROR;
                }
                else
                {
                    lmp_self_device_data.device_status = LMP_IDLE ;
                    hci_baseband_cmd_pending = FALSE ;

                    /* Reset the inquiry result table */
                    memset(lmp_inquiry_result_data, 0,
                           LMP_MAX_INQUIRY_RESULT_DATA *sizeof(LMP_INQUIRY_RESULT_DATA));

                    lmp_num_inq_resp_received = 0 ;

                }
                break;
#endif /* COMPILE_PERIODIC_INQUIRY */

            default :
#ifdef _DAPE_TEST_KILL_PERIODIC_INQ_IF_TIMER_RUNNING
            if (periodic_inquiry_timer != NULL)
            {
                if (lc_kill_periodic_inquiry()!= API_SUCCESS)
                {
                    BT_FW_HCI_ERR(HCI_PERIODIC_INQUIRY_MODE_COMMAND_FAILED,0,0);
                    ret_error = HARDWARE_FAILURE_ERROR;
                }
                else
                {
                    /* Reset the inquiry result table */
                    memset(lmp_inquiry_result_data, 0,
                           LMP_MAX_INQUIRY_RESULT_DATA *sizeof(LMP_INQUIRY_RESULT_DATA));

                    lmp_num_inq_resp_received = 0 ;
                }
            }
            else
            {
#ifndef _CCH_RS_ISSUE_
                ret_error = COMMAND_DISALLOWED_ERROR;
#endif
            }
#else
#ifndef _CCH_RS_ISSUE_
                ret_error = COMMAND_DISALLOWED_ERROR;
#endif
#endif
                break;
        }
        command_complete_event_flag = TRUE ;
        break;

    case HCI_CHANGE_CONNECTION_PACKET_TYPE_OPCODE :
        ret_error = hci_handle_change_connection_packet_type_command(
                        hci_cmd_ptr, &sent_event_flag, &hci_generate_ccpt_event_flag,
                        &hci_max_slots_change_event_flag);
        if (ret_error != HCI_COMMAND_SUCCEEDED)
        {
            BT_FW_HCI_ERR(HCI_HANDLE_CHANGE_CONN_PKT_COMMAND_FAILED,0,0);
        }
        break;

    case HCI_REMOTE_NAME_REQUEST_OPCODE :
        ret_error = hci_handle_remote_name_request_command(
                        hci_cmd_ptr, &sent_event_flag);
        if (ret_error != HCI_COMMAND_SUCCEEDED)
        {
            BT_FW_HCI_ERR(HCI_REMOTE_NAME_REQUEST_COMMAND_FAILED,0,0);
        }
        break;

    case HCI_REMOTE_NAME_REQUEST_CANCEL_OPCODE :
        command_complete_event_flag = 1;

        memcpy(&(rem_name_cancel_bd_addr[0]),
               &hci_cmd_ptr->cmd_parameter[0],LMP_BD_ADDR_SIZE);
#ifndef _REDUCE_LPS_AFTER_RTL8703B_
#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8723_RCP_
// RCP for add Remote Name Cancel Function
// Parameter:  rem_name_cancel_bd_addr only
// return: ret_error
        if (rcp_hci_handle_remote_name_request_cancel_command != NULL)
        {
            if ( rcp_hci_handle_remote_name_request_cancel_command((void *)&ret_error) )
            {
                break;
            }
        }
#endif
#endif
#endif
#ifndef _CCH_PAGE_CON_
        ret_error = UNKNOWN_HCI_COMMAND_ERROR;

#else
        ret_error = lc_kill_paging(rem_name_cancel_bd_addr, 1);   /*reason 1: remote name req*/
#endif
        break;

    case HCI_READ_REMOTE_SUPPORTED_FEATURES_OPCODE :
        ret_error = hci_handle_remote_supported_features_command(
                        hci_cmd_ptr, &sent_event_flag);
        if (ret_error != HCI_COMMAND_SUCCEEDED)
        {
            BT_FW_HCI_ERR(HCI_READ_REMOTE_SUPPORTED_FEATURES_COMMAND_FAILED,0,0);
        }
        break;

    case HCI_READ_REMOTE_VERSION_INFORMATION_OPCODE :
        ret_error = hci_handle_remote_version_information_command(
                        hci_cmd_ptr, &sent_event_flag);
        if (ret_error != HCI_COMMAND_SUCCEEDED)
        {
            BT_FW_HCI_ERR(HCI_READ_REMOTE_SUPPORTED_FEATURES_COMMAND_FAILED,0,0);
        }
        break;

    case HCI_READ_CLOCK_OFFSET_OPCODE:
        ret_error = hci_handle_clock_offset_command(hci_cmd_ptr,
                    &sent_event_flag);
        if (ret_error != HCI_COMMAND_SUCCEEDED)
        {
            BT_FW_HCI_ERR(HCI_READ_REMOTE_SUPPORTED_FEATURES_COMMAND_FAILED,0,0);
        }
        break;

    case HCI_READ_LMP_HANDLE_OPCODE :
        ret_error = hci_read_lmp_handle_command(hci_cmd_ptr,
                                                &sent_event_flag);
        if (ret_error != HCI_COMMAND_SUCCEEDED)
        {
            BT_FW_HCI_ERR(log_file,"hci read lmp handle command failed\n");
        }
        break;

    default :
#ifndef _REDUCE_LPS_AFTER_RTL8703B_
#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8723_RCP_
// RCP for Undefined HCI Link Control Command: ex: create connection cancel
// Parameter: hci_cmd_ptr
// retrun: ret_error

        if (rcp_hci_execute_link_control_command_packet_case != NULL)
        {
            if ( rcp_hci_execute_link_control_command_packet_case((void *)&ret_error, hci_cmd_ptr) )
            {
                return ret_error;
            }
        }
#endif
#endif
#endif
        ret_error = HCI_HANDLE_1_2_LINK_CONTROL_COMMAND(hci_cmd_ptr);

        if (ret_error == UNKNOWN_HCI_COMMAND_ERROR)
        {
            if (bz_auth_handle_hci_security_commands(hci_cmd_ptr))
            {
                return BT_FW_SUCCESS;
            }

#ifdef VER_CSA4
            if (IS_BT_CSA4)
            {
                ret_error = HCI_HANDLE_CSA4_LC_COMMAND(hci_cmd_ptr);
            }
#endif
#ifdef _SUPPORT_SECURE_CONNECTION_
            if (ret_error == UNKNOWN_HCI_COMMAND_ERROR)
            {
                if (IS_BT_SECURE_CONN)
                {
                    ret_error = HCI_HANDLE_SECURE_CONN_LC_COMMAND(hci_cmd_ptr);
                }
            }
#endif
        }

        if (ret_error != HCI_COMMAND_SUCCEEDED)
        {
            /* Standard BT specification error  : Error code 0x01
            * specification says send either command status or
            * command complete event.
            * Command opcode 0x0000 is a NOP (no operation.)
            */
            /* hci_cmd_ptr->cmd_opcode = HCI_NOP ; */

            command_complete_event_flag = 1;
            break;
        }
        else
        {
            sent_event_flag = TRUE;
        }
        break;
    } /* End of switch statement */

    if (sent_event_flag != TRUE)
    {
        /*For commands queued to the baseband,
         * LC module will send the  command stauts event
         */
        if (command_complete_event_flag) /* Send Command complete event */
        {
            hci_generate_command_complete_event(
                hci_cmd_ptr->cmd_opcode, ret_error,0, NULL);
        }
        else
        {
            /*Send Command status event */
            hci_generate_command_status_event(hci_cmd_ptr->cmd_opcode,
                                              ret_error);

            if (hci_cmd_ptr->cmd_opcode ==
                                HCI_CHANGE_CONNECTION_PACKET_TYPE_OPCODE)
            {
                if (hci_max_slots_change_event_flag == 0x01)
                {
                    UINT16 conn_handle=0,ce_index=0;

                    BT_FW_EXTRACT_16_BITS(conn_handle,
                                          &(hci_cmd_ptr->cmd_parameter[0x00]));

                    if (LMP_GET_CE_INDEX_FROM_CONN_HANDLE(conn_handle,
                                                    &ce_index) != API_FAILURE)
                    {
                        hci_generate_max_slots_change_event(
                            conn_handle,
                            lmp_connection_entity[ce_index].temp_tx_max_slot);

                    }
                    hci_max_slots_change_event_flag = 0x0;
                }
                if (hci_generate_ccpt_event_flag == 0x01)
                {
                    UINT16 conn_handle, packet_type;

                    BT_FW_EXTRACT_16_BITS(conn_handle,
                                        &(hci_cmd_ptr->cmd_parameter[0x00]));
                    BT_FW_EXTRACT_16_BITS(packet_type,
                                        &(hci_cmd_ptr->cmd_parameter[0x02]));

                    hci_generate_connection_packet_type_changed_event(
                        HCI_COMMAND_SUCCEEDED,conn_handle,packet_type);
                    hci_generate_ccpt_event_flag = 0x0;
                }
            }
        }
    }

    return BT_FW_SUCCESS;
}

/**
 * Verifies the Link Policy group commands received
 * from the host. It extracts the command opcode from the command
 * packet and analyzes the command packet depending on the type of
 * Command it calls the corresponding routines.
 *
 * \param hci_cmd_ptr Pointer to HCI_CMD_PKT.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI Error code.
 *
 */
UCHAR hci_execute_link_policy_command_packet(HCI_CMD_PKT *hci_cmd_ptr)
{
    UINT16 ce_index = 0;
    UINT16 connection_handle;
    UCHAR ret_error = HCI_COMMAND_SUCCEEDED ;
    UCHAR command_complete_event_flag = FALSE;
    LMP_CONNECTION_ENTITY *ce_ptr;

    BT_FW_HCI_INF(HCI_LINK_POLICY_GROUP_COMMAND_FUNCTION,0,0);

    /* Validate parameters */
    switch(hci_cmd_ptr ->cmd_opcode)
    {
    case HCI_SNIFF_MODE_OPCODE:
    case HCI_EXIT_SNIFF_MODE_OPCODE:
    case HCI_PARK_MODE_OPCODE:
    case HCI_EXIT_PARK_MODE_OPCODE:
    case HCI_HOLD_MODE_OPCODE:
    case HCI_QOS_SETUP_OPCODE:
    case HCI_WRITE_LINK_POLICY_SETTINGS_OPCODE:
    case HCI_READ_LINK_POLICY_SETTNGS_OPCODE:
    case HCI_FLOW_SPECIFICATION_OPCODE:
    case HCI_ROLE_DISCOVERY_OPCODE:

        /* Extract the connection handle */
        BT_FW_EXTRACT_16_BITS(connection_handle, &(hci_cmd_ptr->cmd_parameter[0]));
        if (LMP_GET_CE_INDEX_FROM_CONN_HANDLE(connection_handle, &ce_index) !=
                API_SUCCESS)
        {
            ce_index = connection_handle;
            ret_error = NO_CONNECTION_ERROR ;
            switch(hci_cmd_ptr->cmd_opcode)
            {
                case HCI_ROLE_DISCOVERY_OPCODE:
                case HCI_WRITE_LINK_POLICY_SETTINGS_OPCODE:
                case HCI_READ_LINK_POLICY_SETTNGS_OPCODE:
                    command_complete_event_flag = TRUE;
                    break;
                default:
                    break;
            }
        }
        break;
    default:
        break;
    }

    ce_ptr = &lmp_connection_entity[ce_index];

    if (ret_error == HCI_COMMAND_SUCCEEDED)
    {
        /* Validate parameters */
        switch(hci_cmd_ptr ->cmd_opcode)
        {
        case HCI_HOLD_MODE_OPCODE:
        case HCI_SNIFF_MODE_OPCODE:
        case HCI_PARK_MODE_OPCODE:
            if (ce_ptr->ce_status != LMP_CONNECTED)
            {
                if (ce_ptr->ce_status == LMP_STANDBY )
                {
                    ret_error = NO_CONNECTION_ERROR ;
                }
                else
                {
                    ret_error = COMMAND_DISALLOWED_ERROR;
                }
            }
            break;

#ifdef COMPILE_SNIFF_MODE
        case HCI_EXIT_SNIFF_MODE_OPCODE:
        {
#ifndef _DAPE_TEST_NEW_HW
            if (ce_ptr->ce_status != LMP_SNIFF_MODE)
#else
            ///// check if is sniff by hw
            BZ_REG_HW_INSTRUCTION_STATUS hw_instruction_status;
            *(UINT16*)&hw_instruction_status = BB_read_instruction_status(ce_index);
            //RT_BT_LOG(BLUE, DAPE_TEST_LOG525, 6,ce_index,
            //                            *(UINT16*)&hw_instruction_status, hw_instruction_status,
            //                            hw_instruction_status.sniff,
            //                            ce_ptr->ce_status, 0);
            if (hw_instruction_status.sniff == 0)
#endif
            {
                //RT_BT_LOG(BLUE, CCH_DBG_006, 2,ce_index, ce_ptr->ce_status);

                BT_FW_HCI_INF(CONN_ENTITY_STATUS_IS_INVALID_CE_STATUS,1,
                              ce_ptr->ce_status);
                ret_error = COMMAND_DISALLOWED_ERROR;
            }
            break;
        }
#else
        case HCI_EXIT_SNIFF_MODE_OPCODE:
            ret_error = UNSUPPORTED_FEATURE_OR_PARAMETER_ERROR;
            break;
#endif

        case HCI_EXIT_PARK_MODE_OPCODE:
#ifdef COMPILE_PARK_MODE
            if (ce_ptr->ce_status != LMP_PARK_MODE)
            {
                BT_FW_HCI_ERR(THIS_DEVICE_IS_NOT_IN_PARK_MODE,0,0);
                ret_error = COMMAND_DISALLOWED_ERROR;
            }
#else
            ret_error = UNSUPPORTED_FEATURE_OR_PARAMETER_ERROR;
#endif
            break;

        case HCI_ROLE_DISCOVERY_OPCODE:
            command_complete_event_flag = TRUE;
            break;
        default:
            break;
        }
    }

    if (ret_error == HCI_COMMAND_SUCCEEDED)
    {
        switch(hci_cmd_ptr ->cmd_opcode)
        {
        case HCI_SNIFF_MODE_OPCODE :
#ifdef COMPILE_SNIFF_MODE
            ret_error = hci_handle_sniff_mode_command(
                            hci_cmd_ptr->cmd_parameter, ce_index);
            if (ret_error != HCI_COMMAND_SUCCEEDED)
            {
                BT_FW_HCI_ERR(HCI_HANDLE_SNIFF_MODE_COMMAND_FAILED,0,0);
            }
#else
            command_complete_event_flag = FALSE ;
            ret_error=UNSUPPORTED_FEATURE_OR_PARAMETER_ERROR;
#endif

            break;

        case HCI_EXIT_SNIFF_MODE_OPCODE :
#ifdef COMPILE_SNIFF_MODE
            ret_error = hci_handle_exit_sniff_mode_command(ce_index);
            if (ret_error != HCI_COMMAND_SUCCEEDED)
            {
                BT_FW_HCI_ERR(HCI_HADLE_EXIT_SNIFF_MODE_COMMAND_FAILED,0,0);
            }
#else
            command_complete_event_flag = FALSE ;
            ret_error=UNSUPPORTED_FEATURE_OR_PARAMETER_ERROR;
#endif
            break;

        case HCI_PARK_MODE_OPCODE :
#ifdef COMPILE_PARK_MODE
            ret_error = hci_handle_park_mode_command(hci_cmd_ptr, ce_index);
            if (ret_error != HCI_COMMAND_SUCCEEDED)
            {
                BT_FW_HCI_ERR(HCI_HADLE_PARK_MODE_COMMAND_FAILED,0,0);
            }
#else
            ret_error = UNSUPPORTED_FEATURE_OR_PARAMETER_ERROR;
#endif /*COMPILE_PARK_MODE*/
            break;

        case HCI_EXIT_PARK_MODE_OPCODE :
#ifdef COMPILE_PARK_MODE
            ret_error = hci_handle_exit_park_mode_command(ce_index);
            if (ret_error != HCI_COMMAND_SUCCEEDED)
            {
                BT_FW_HCI_ERR(HCI_HADLE_EXIT_PARK_MODE_COMMAND_FAILED,0,0);
            }
#else
            command_complete_event_flag = FALSE ;
            ret_error = UNSUPPORTED_FEATURE_OR_PARAMETER_ERROR;
#endif  /* COMPILE_PARK_MODE */
            break;

        case HCI_HOLD_MODE_OPCODE :
#ifdef COMPILE_HOLD_MODE
            ret_error = hci_handle_hold_mode_command(
                                    hci_cmd_ptr->cmd_parameter, ce_index);
            if (ret_error != HCI_COMMAND_SUCCEEDED)
            {
                BT_FW_HCI_ERR(HCI_HADLE_HOLD_MODE_COMMAND_FAILED,0,0);
            }
#else
            command_complete_event_flag = FALSE ;
            ret_error=UNSUPPORTED_FEATURE_OR_PARAMETER_ERROR;
#endif /* COMPILE_HOLD_MODE */
            break;

        case HCI_QOS_SETUP_OPCODE :
            ret_error = hci_handle_qos_setup_command(hci_cmd_ptr,ce_index);
            if (ret_error != HCI_COMMAND_SUCCEEDED)
            {
                BT_FW_HCI_ERR(HCI_HADLE_HOLD_MODE_COMMAND_FAILED,0,0);
            }
            break;

        case HCI_SWITCH_ROLE_OPCODE :
#ifdef COMPILE_ROLE_SWITCH
            ret_error = hci_handle_switch_role_command(hci_cmd_ptr,&ce_index);
            if (ret_error != HCI_COMMAND_SUCCEEDED)
            {
                BT_FW_HCI_ERR(HCI_HANDLE_SWITCH_ROLE_COMMAND_FAILED,0,0);
                HCI_LOG_INFO(LOG_LEVEL_LOW,
                            HCI_HANDLE_SWITCH_ROLE_COMMAND_FAILED,0,0);

            }
#else
            command_complete_event_flag = FALSE ;
            ret_error = UNSUPPORTED_FEATURE_OR_PARAMETER_ERROR;
#endif /* COMPILE_ROLE_SWITCH */
            break;

        case HCI_WRITE_LINK_POLICY_SETTINGS_OPCODE :
            ret_error = hci_handle_write_link_policy_settings_command(
                            hci_cmd_ptr->cmd_parameter, ce_index);
            if (ret_error != HCI_COMMAND_SUCCEEDED)
            {
                BT_FW_HCI_ERR(HCI_HANDLE_WRITE_LINK_POLICY_SETTINGS_COMMAND_FAILED,0,0);

            }
            command_complete_event_flag = TRUE ;
            break;

        case HCI_WRITE_DEFAULT_LINK_POLICY_SETTINGS_OPCODE :
            ret_error = hci_handle_write_default_link_policy_settings_command(
                    hci_cmd_ptr->cmd_parameter);
            if (ret_error != HCI_COMMAND_SUCCEEDED)
            {
                BT_FW_HCI_ERR(HCI_HANDLE_WRITE_LINK_POLICY_SETTINGS_COMMAND_FAILED,0,0);
            }
            command_complete_event_flag = TRUE ;
            break;

        case HCI_READ_DEFAULT_LINK_POLICY_SETTNGS_OPCODE :
            command_complete_event_flag = TRUE ;
            break;

        case HCI_FLOW_SPECIFICATION_OPCODE:
            ret_error = hci_handle_flow_specification_command(
                                            hci_cmd_ptr, ce_index);

            command_complete_event_flag = FALSE; /* modified from TRUE to
                                                    FALSE by guochunxia */
            break;

        case HCI_ROLE_DISCOVERY_OPCODE:
            ret_error = hci_handle_role_discovery_command(hci_cmd_ptr,&ce_index);
            if (ret_error != HCI_COMMAND_SUCCEEDED)
            {
                BT_FW_HCI_ERR(HCI_HANDLE_READ_LINK_POLICY_COMMAND_COMMAND_FAILED,0,0);
            }
            command_complete_event_flag = TRUE ;
            break;

        case HCI_READ_LINK_POLICY_SETTNGS_OPCODE:
            command_complete_event_flag = TRUE ;
            break;

        default :
            ret_error = HCI_HANDLE_1_2_LINK_POLICY_COMMAND(hci_cmd_ptr, ce_index);
            if (ret_error == UNKNOWN_HCI_COMMAND_ERROR)
            {
                /* hci_cmd_ptr->cmd_opcode = HCI_NOP ; */
                ret_error = HCI_HANDLE_2_1_LINK_POLICY_COMMAND
                            (hci_cmd_ptr, &command_complete_event_flag);
                if (ret_error != UNKNOWN_HCI_COMMAND_ERROR)
                {
                    command_complete_event_flag = 0x02;
                }
            }
            break;
        } /*End of switch statement */
    }

    if (command_complete_event_flag == FALSE)
    {
        /* Send command status event*/
        hci_generate_command_status_event(hci_cmd_ptr->cmd_opcode, ret_error);
    }
    else if (command_complete_event_flag == TRUE)
    {
        /* Send command complete event*/
        hci_generate_command_complete_event(
            hci_cmd_ptr->cmd_opcode, ret_error, ce_index, NULL);
    }
    else
    {
        /* Generate no event. */
        /* NOTE: command_complete_event_flag is uchar and can be
         * something other than true/false
         */
    }

    return ret_error;
}


/**
 * Verifies the Host controller and Baseband
 * group of commands received from the host.
 * It extracts the command opcode from the command packet
 * and analyzes the command packet depending on the type of
 * Command it calls the corresponding routines.
 * It sends command complete event for all the HC and baseband
 * commands except for host number of completed packets command.
 *
 * \param hci_cmd_ptr Pointer to HCI_CMD_PKT.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI Error code.
 *
 */
UCHAR hci_execute_HC_and_baseband_command_packet(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR ret_error = HCI_COMMAND_SUCCEEDED ;
    UINT16 ce_index = 0;
    UCHAR sent_event_flag = FALSE ;

    BT_FW_HCI_INF(HCI_HC_AND_BASEBAND_COMMAND_FUNCTION,0,0);

    switch(hci_cmd_ptr ->cmd_opcode)
    {
        case HCI_RESET_OPCODE:
            ret_error = hci_handle_reset_command();
            if (ret_error != HCI_COMMAND_SUCCEEDED)
            {
                BT_FW_HCI_ERR(HCI_HADLE_RESET_COMMAND_FAILED,0,0);
            }

/*  Added by Wallice for RTK LoopBack Mode. 2012/06/04  */
            enable_hci_loopback_mode = 0x00;
/*  End Added by Wallice for RTK LoopBack Mode. 2012/06/04  */
            hci_generate_command_complete_event(HCI_RESET_OPCODE,
                                                ret_error,ce_index, NULL);
            /* Decr. num_hci... it is reset in reset_cmd but will be incremented
                      when this function return.
                      */
            lmp_self_device_data.num_hci_command_packets--;
            return BT_FW_SUCCESS;

        case HCI_SET_EVENT_MASK_OPCODE :
            ret_error =hci_handle_set_event_mask_command(hci_cmd_ptr);
            if (ret_error != HCI_COMMAND_SUCCEEDED)
            {
                BT_FW_HCI_ERR(HCI_HADLE_RESET_COMMAND_FAILED,0,0);
            }
            break;

        case HCI_SET_EVENT_FILTER_OPCODE:
            ret_error = hci_handle_set_event_filter_command(hci_cmd_ptr);
            if (ret_error != HCI_COMMAND_SUCCEEDED)
            {
                BT_FW_HCI_ERR(HCI_HADLE_SET_EVENT_FILTER_COMMAND_FAILED,0,0);
            }
            break;

        case HCI_FLUSH_OPCODE :
            ret_error = hci_handle_flush_command(hci_cmd_ptr, &ce_index);
            if (ret_error == HCI_COMMAND_SUCCEEDED)
            {
                sent_event_flag = TRUE;
            }
            break;

        case HCI_READ_NUM_BROADCAST_RETRANSMISSIONS_OPCODE:
            break;

        case HCI_WRITE_NUM_BROADCAST_RETRANSMISSIONS_OPCODE:
            ret_error = hci_handle_write_num_brcast_retran(hci_cmd_ptr);
            if (ret_error != HCI_COMMAND_SUCCEEDED)
            {
                BT_FW_HCI_ERR(HCI_WRITE_NUM_BROADCAST_RETRANSMISSIONS_FAILED,0,0);
            }
            break;

#ifndef _DISABLE_HCI_HOST_FLOW_CONTROL_
#ifdef ENABLE_SCO
        case HCI_READ_SCO_FLOW_CONTROL_ENABLE_OPCODE:
            break;

        case HCI_WRITE_SCO_FLOW_CONTROL_ENABLE_OPCODE:
            ret_error = hci_handle_write_sco_ctrl_enable(hci_cmd_ptr);
            if (ret_error != HCI_COMMAND_SUCCEEDED)
            {
                BT_FW_HCI_ERR(HCI_WRITE_SCO_FLOW_CONTROL_ENABLE_FAILED,0,0);
            }
            break;
#endif /* ENABLE_SCO */
        case HCI_SET_CONTROLLER_TO_HOST_FLOW_CONTROL_OPCODE:
            ret_error = hci_handle_set_hc_to_host_flow_ctrl(hci_cmd_ptr);
            if (ret_error != HCI_COMMAND_SUCCEEDED)
            {
                BT_FW_HCI_ERR(HCI_SET_HC_TO_HOST_FLOW_CONTROL_FAILED,0,0);
            }
            break;
#endif /* end of #ifndef _DISABLE_HCI_HOST_FLOW_CONTROL_ */

        case HCI_READ_NUMBER_OF_SUPPORTED_IAC_OPCODE:
            break;

        case HCI_READ_CURRENT_IAC_LAP_OPCODE:
            break;

        case HCI_WRITE_CURRENT_IAC_LAP_OPCODE:
            ret_error = hci_handle_write_current_iac_lap(hci_cmd_ptr);
            if (ret_error != HCI_COMMAND_SUCCEEDED)
            {
                BT_FW_HCI_ERR(HCI_WRITE_CURRENT_IAC_LAP_FAILED,0,0);
            }
            break;

        case HCI_READ_TRANSMIT_POWER_LEVEL_OPCODE:
            ret_error=hci_handle_read_transmit_pwr_lvl(hci_cmd_ptr,&ce_index);
            if (ret_error != HCI_COMMAND_SUCCEEDED)
            {
                BT_FW_HCI_ERR(HCI_READ_TRANSMIT_POWER_LEVEL_FAILED,0,0);
            }
            break;

        case HCI_READ_AUTOMATIC_FLUSH_TIMEOUT_OPCODE:
            ret_error = hci_handle_read_automatic_flush_timeout(
                            hci_cmd_ptr, &ce_index);
            if (ret_error != HCI_COMMAND_SUCCEEDED)
            {
                BT_FW_HCI_ERR(HCI_READ_AUTOMATIC_FLUSH_TIMEOUT_FAILED,0,0);
            }
            break;

        case HCI_WRITE_AUTOMATIC_FLUSH_TIMEOUT_OPCODE:
            ret_error = hci_handle_write_automatic_flush_timeout(
                            hci_cmd_ptr, &ce_index);
            if (ret_error != HCI_COMMAND_SUCCEEDED)
            {
                BT_FW_HCI_ERR(HCI_WRITE_AUTOMATIC_FLUSH_TIMEOUT_FAILED,0,0);
            }
            break;

#ifndef _DISABLE_HCI_HOST_FLOW_CONTROL_
        case HCI_HOST_NUMBER_OF_COMPLETED_PACKETS_OPCODE:
            ret_error = hci_handle_host_num_of_comp_pkts(hci_cmd_ptr,
                        &ce_index);
            if (ret_error != HCI_COMMAND_SUCCEEDED)
            {
                hci_generate_command_complete_event(
                    hci_cmd_ptr->cmd_opcode, ret_error, ce_index, NULL);
                BT_FW_HCI_ERR(HCI_HOST_NUMBER_OF_COMPLETED_PACKETS_FAILED,0,0);
            }

            return BT_FW_SUCCESS;
#endif /* end of #ifndef _DISABLE_HCI_HOST_FLOW_CONTROL_ */

        case HCI_WRITE_SCAN_ENABLE_OPCODE :
            ret_error = hci_handle_write_scan_enable_command(
                            hci_cmd_ptr);
            if (ret_error != HCI_COMMAND_SUCCEEDED)
            {
                BT_FW_HCI_ERR(HCI_HADLE_WRITE_SCAN_ENABLE_COMMAND_FAILED,0,0);
            }
            break;

        case HCI_READ_SCAN_ENABLE_OPCODE :
            break;

        case HCI_WRITE_PAGE_SCAN_ACTIVITY_OPCODE :
            ret_error = hci_handle_write_page_scan_activity_command(
                            hci_cmd_ptr);
            if (ret_error != HCI_COMMAND_SUCCEEDED)
            {
                BT_FW_HCI_ERR(HCI_WRITE_PAGE_SCAN_ACTIVITY_COMMAND_FAILED,0,0);
            }
            break;

        case HCI_WRITE_INQUIRY_SCAN_ACTIVITY_OPCODE :
            ret_error =
                hci_handle_write_inquiry_scan_activity_command(hci_cmd_ptr);
            if (ret_error != HCI_COMMAND_SUCCEEDED)
            {
                BT_FW_HCI_ERR(HCI_WRITE_INQUIRY_SCAN_ACTIVITY_COMMAND_FAILED,0,0);
            }
            break;

        case HCI_READ_PAGE_SCAN_ACTIVITY_OPCODE :
            break ;

        case HCI_READ_INQUIRY_SCAN_ACTIVITY_OPCODE :
            break ;

        case HCI_CHANGE_LOCAL_NAME_OPCODE :
            ret_error =
                hci_handle_change_local_name_command(hci_cmd_ptr);
            if (ret_error != HCI_COMMAND_SUCCEEDED)
            {
                BT_FW_HCI_ERR(HCI_CHANGE_LOCAL_NAME_COMMAND_FAILED,0,0);
            }
            break;

        case HCI_WRITE_CONNECTION_ACCEPT_TIMEOUT_OPCODE :
            ret_error = hci_handle_write_connection_accept_timeout_command(
                            hci_cmd_ptr);
            if (ret_error != HCI_COMMAND_SUCCEEDED)
            {
                BT_FW_HCI_ERR(HCI_WRITE_CONNECTION_ACCEPT_TIMEOUT_FAILED,0,0);
            }
            break;

        case HCI_READ_LOCAL_NAME_OPCODE :
            break;

        case HCI_READ_CONNECTION_ACCEPT_TIMEOUT_OPCODE :
            break;

        case HCI_WRITE_PAGE_TIMEOUT_OPCODE :
            ret_error =
                hci_handle_write_page_timeout_command(hci_cmd_ptr);
            if (ret_error != HCI_COMMAND_SUCCEEDED)
            {
                BT_FW_HCI_ERR(HCI_WRITE_PAGE_ACCEPT_TIMEOUT_FAILED,0,0);
            }
            break;

        case HCI_READ_PAGE_TIMEOUT_OPCODE :
            break;

        case HCI_WRITE_CLASS_OF_DEVICE_OPCODE :
            ret_error =
                hci_handle_write_class_of_device_command(hci_cmd_ptr);
            if (ret_error != HCI_COMMAND_SUCCEEDED)
            {
                BT_FW_HCI_ERR(HCI_WRITE_CLASS_OF_DEVICE_FAILED,0,0);
            }

#ifdef _3DD_FUNCTION_SUPPORT_
#ifdef _SUPPORT_CSB_TRANSMITTER_
            if (ret_error == HCI_COMMAND_SUCCEEDED)
            {
                bt_csb_check_and_set_csb_or_rp_mode();
            }
#endif
#endif
            break;

        case HCI_READ_CLASS_OF_DEVICE_OPCODE :
            break;

        case HCI_WRITE_VOICE_SETTING_OPCODE :
            ret_error =
                hci_handle_write_voice_setting_command(hci_cmd_ptr);
            if (ret_error != HCI_COMMAND_SUCCEEDED)
            {
                BT_FW_HCI_ERR(HCI_VOICE_SETTING_COMMAND_FAILED,0,0);
            }
            break;

        case HCI_READ_VOICE_SETTING_OPCODE :
            break;

        case HCI_WRITE_LINK_SUPERVISION_TIMEOUT_OPCODE :
            ret_error = hci_handle_write_link_supervision_timeout_command(
                            hci_cmd_ptr, &ce_index);
            if (ret_error != HCI_COMMAND_SUCCEEDED)
            {
                BT_FW_HCI_ERR(HCI_LINK_SUPERVISION_TIMEOUT_COMMAND_FAILED,0,0);
            }
            break;

        case HCI_READ_LINK_SUPERVISION_TIMEOUT_OPCODE :
            ret_error =
                hci_handle_read_link_supervision_timeout_command(hci_cmd_ptr,
                        &sent_event_flag);
            if (ret_error != HCI_COMMAND_SUCCEEDED)
            {
                BT_FW_HCI_ERR(HCI_READ_LINK_SUPERVISION_TIMEOUT_COMMAND_FAILED,0,0);
            }
            break;

        case HCI_HOST_BUFFER_SIZE_OPCODE:
            ret_error = hci_handle_host_buffer_size_command(hci_cmd_ptr);
            if (ret_error != HCI_COMMAND_SUCCEEDED)
            {
                BT_FW_HCI_ERR(HCI_HADLE_RESET_COMMAND_FAILED,0,0);
            }
            break;

        case HCI_WRITE_PAGE_SCAN_PERIOD_MODE_OPCODE :
            ret_error = hci_handle_write_page_scan_period_mode_command(
                            hci_cmd_ptr);
            if (ret_error != HCI_COMMAND_SUCCEEDED)
            {
                BT_FW_HCI_ERR(HCI_WRITE_PAGE_SCAN_PERIOD_MODE_COMMAND_FAILED,0,0);
            }
            break;

        case HCI_READ_PAGE_SCAN_PERIOD_MODE_OPCODE :
            break;

        case HCI_READ_PAGE_SCAN_MODE_OPCODE :
            break;

        case HCI_WRITE_PAGE_SCAN_MODE_OPCODE :
            ret_error = hci_handle_write_page_scan_mode_command(
                            hci_cmd_ptr, &sent_event_flag);
            if (ret_error != HCI_COMMAND_SUCCEEDED)
            {
                BT_FW_HCI_ERR(HCI_WRITE_PAGE_SCAN_COMMAND_FAILED,0,0);
            }
            else
            {
                ;
            }
            break;

        case HCI_READ_HOLD_MODE_ACTIVITY_OPCODE:
            break;
#ifdef MWS_ENABLE
		case HCI_SET_MWS_CHANNEL_PATAMETERS_OPCODE:
			break;
		case HCI_SET_EXTERNAL_FRAME_CONFIGURATION_OPCODE:
			break;
		case HCI_SET_MWS_SIGNALING:
			break;
		case HCI_SET_MWS_TRANSPORT_LAYER:
			break;
		case HCI_SET_MWS_SCAN_FREQUENCY_TABLE:
			break;
		case HCI_SET_MWS_PATTERN_CONFIGURATION:
			break;
#endif
        case HCI_WRITE_HOLD_MODE_ACTIVITY_OPCODE:
#ifdef COMPILE_HOLD_MODE
            ret_error = hci_handle_write_hold_mode_activity_command(hci_cmd_ptr);
#endif
            break;

        default :
            ret_error = HCI_HANDLE_1_2_HC_BB_COMMAND(hci_cmd_ptr);
            if (ret_error != UNKNOWN_HCI_COMMAND_ERROR)
            {
                break;
            }
            ret_error = HCI_HANDLE_2_1_HC_BB_COMMAND(hci_cmd_ptr,
                        &sent_event_flag);
            if (ret_error != UNKNOWN_HCI_COMMAND_ERROR)
            {
                break;
            }
#ifdef VER_3_0
            if (IS_BT30)
            {
                ret_error = HCI_HANDLE_3_0_HC_BB_COMMAND(hci_cmd_ptr);
                if (ret_error != UNKNOWN_HCI_COMMAND_ERROR)
                {
                    break;
                }
            }
#endif
#ifdef LE_MODE_EN
            if (IS_BT40)
            {
                ret_error = HCI_HANDLE_4_0_HC_BB_COMMAND(hci_cmd_ptr);
                if (ret_error != UNKNOWN_HCI_COMMAND_ERROR)
                {
                    break;
                }
            }
#endif
#ifdef VER_CSA4
            if (IS_BT_CSA4)
            {
                ret_error = HCI_HANDLE_CSA4_HC_BB_COMMAND(hci_cmd_ptr);
                if (ret_error != UNKNOWN_HCI_COMMAND_ERROR)
                {
                    break;
                }
            }
#endif
#ifdef _SUPPORT_SECURE_CONNECTION_
            if (IS_BT_SECURE_CONN)
            {
                ret_error = HCI_HANDLE_SECURE_CONN_HC_BB_COMMAND(hci_cmd_ptr);
                if (ret_error != UNKNOWN_HCI_COMMAND_ERROR)
                {
                    break;
                }
            }
#endif
            if (bz_auth_handle_hci_security_commands(hci_cmd_ptr))
            {
                return BT_FW_SUCCESS;
            }
            break;
    } /*End of switch statement */

    if (ret_error != HCI_COMMAND_SUCCEEDED)
    {
        /*
           Copy connection handle to ce_index assuming it's
           the first parameter
         */
        BT_FW_EXTRACT_16_BITS(ce_index, &(hci_cmd_ptr->cmd_parameter[0]));
    }

    /* Don't send any event to HOST_NUMBER_OF_COMPLETED_PACKETS command */
    if ((sent_event_flag == FALSE) && (hci_cmd_ptr->cmd_opcode !=
                                      HCI_HOST_NUMBER_OF_COMPLETED_PACKETS_OPCODE))
    {
        hci_generate_command_complete_event(hci_cmd_ptr->cmd_opcode,
                                            ret_error,ce_index, NULL);
    }

    return BT_FW_SUCCESS;
}

/**
 * Verifies the Host controller and Baseband group of
 * commands received from the host. It extracts the command opcode
 * from the command packet and analyzes the  command packet depending
 * on the group of command it calls the corresponding routines.
 *
 * It sends command complete event for all the Informational
 * parameters group of HCI commands.
 *
 * \param hci_cmd_ptr Pointer to HCI_CMD_PKT.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI Error code.
 *
 */
UCHAR hci_execute_info_params_command_packet (HCI_CMD_PKT *hci_cmd_ptr)
{
    void *arg = NULL;
    UCHAR ret_error = HCI_COMMAND_SUCCEEDED ;

    BT_FW_HCI_INF(HCI_INFO_PARAMETERS_COMMAND_FUNCTION,0,0);

    switch(hci_cmd_ptr->cmd_opcode)
    {
#ifdef COMPILE_FEATURE_REQ_EXT
        case HCI_READ_LOCAL_EXTENDED_FEATURES_OPCODE:
            arg = (void *)(UINT32) hci_cmd_ptr->cmd_parameter[0];
            break;
#endif

        case HCI_READ_LOCAL_VERSION_INFORMATION_OPCODE: /* Fall through. */
        case HCI_READ_LOCAL_SUPPORTED_FEATURES_OPCODE: /* Fall through. */
        case HCI_READ_LOCAL_SUPPORTED_COMMANDS_OPCODE: /* Fall through. */
        case HCI_READ_BUFFER_SIZE_OPCODE: /* Fall through. */
#ifdef COMPILE_DEPRECATED_COMMANDS
        case HCI_READ_COUNTRY_CODE_OPCODE: /* Fall through. */
#endif
        case HCI_READ_BD_ADDR_OPCODE:
        {
            ret_error = HCI_COMMAND_SUCCEEDED;
        }
        break;

        default:
        {
            ret_error = UNKNOWN_HCI_COMMAND_ERROR;
        }
        break;
    } /*End of switch statement */

    /* Send connection complete event to the host.*/
    hci_generate_command_complete_event(hci_cmd_ptr->cmd_opcode, ret_error, 0,
            arg);

    return BT_FW_SUCCESS ;
}

/**
 * Gets the information about the current state.
 * of the host controller, Link manager and baseband.
 * It sends command complete event for all the commands of this group.
 *
 * \param hci_cmd_ptr Pointer to HCI_CMD_PKT.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI Error code.
 *
 */
UCHAR hci_execute_status_command_packet (HCI_CMD_PKT *hci_cmd_ptr)
{
    UINT16 conn_handle;
    UINT16 ce_index = 0;
    UCHAR ret_val = HCI_COMMAND_SUCCEEDED ;
    UCHAR which_clock;

    BT_FW_HCI_INF(HCI_STATUS_PARAMETERS_COMMAND_FUNCTION,0,0);

    BT_FW_EXTRACT_16_BITS(conn_handle, &(hci_cmd_ptr->cmd_parameter[0]));

    if (
#ifndef _DISABLE_HCI_QOS_CONTROL_
        (hci_cmd_ptr->cmd_opcode == HCI_READ_FAILED_CONTACT_COUNTER_OPCODE) ||
        (hci_cmd_ptr->cmd_opcode == HCI_RESET_FAILED_CONTACT_COUNTER_OPCODE) ||
        (hci_cmd_ptr->cmd_opcode == HCI_GET_LINK_QUALITY_OPCODE) ||
#endif
        (hci_cmd_ptr->cmd_opcode == HCI_READ_RSSI) ||
        (hci_cmd_ptr->cmd_opcode == HCI_READ_AFH_CHANNEL_MAP_OPCODE))
    {
        /* Validate connection handle. */
        /* Obtain the index corresponding to the connection handle */
        if (LMP_GET_CE_INDEX_FROM_CONN_HANDLE(conn_handle, &ce_index)
                != API_SUCCESS)
        {
            ret_val = NO_CONNECTION_ERROR;
            ce_index = conn_handle;

#ifdef LE_MODE_EN
            if ((hci_cmd_ptr->cmd_opcode == HCI_READ_RSSI) && IS_BT40)
            {
                LL_CONN_HANDLE_UNIT *p_handle;
                p_handle = ll_fw_search_handle_unit_via_conn_handle(conn_handle);
                if (p_handle != NULL)
                {
                    ret_val = HCI_COMMAND_SUCCEEDED ;
                    ce_index = LMP_MAX_CONN_HANDLES + p_handle->unit_id;
                }
            }
#endif
        }
    }

    switch(hci_cmd_ptr->cmd_opcode)
    {
#ifndef _DISABLE_HCI_QOS_CONTROL_
        case HCI_READ_FAILED_CONTACT_COUNTER_OPCODE:
            break;

        case HCI_RESET_FAILED_CONTACT_COUNTER_OPCODE:
            if (ret_val == HCI_COMMAND_SUCCEEDED)
            {
                lmp_connection_entity[ce_index].
                failed_contact_counter = 0 ;
            }
            break;

        case HCI_GET_LINK_QUALITY_OPCODE:
            break;
#endif

        case HCI_READ_RSSI:
            break;

        case HCI_READ_CLOCK_OPCODE:
            /*
             * If which_clock is 0, pass ce_index as 0xfe.
             * Otherwise, check if there is connection.
             */
            which_clock = hci_cmd_ptr->cmd_parameter[2];

            if (which_clock == LOCAL_CLOCK)
            {
                ce_index = 0xfe;
            }
            else if (which_clock == PICONET_CLOCK)
            {
                ce_index = 0xff;
                if (LMP_GET_CE_INDEX_FROM_CONN_HANDLE(conn_handle, &ce_index)
                        != API_SUCCESS)
                {
                    ret_val = NO_CONNECTION_ERROR;
                    ce_index = conn_handle;
                }
            }
            else
            {
                /* which_clock is in the range 0x02-0xFF */
                ret_val = INVALID_HCI_COMMAND_PARAMETERS_ERROR;
            }
            break;

        default:
            ret_val = HCI_HANDLE_1_2_STATUS_COMMAND(hci_cmd_ptr, conn_handle);
            if (ret_val != HCI_COMMAND_SUCCEEDED)
            {
                ce_index = conn_handle;
            }
            break;
    }

    /* Send connection complete event to the host.*/
    hci_generate_command_complete_event(
        hci_cmd_ptr->cmd_opcode, ret_val, ce_index, NULL);

    return BT_FW_SUCCESS;
}

#ifdef TEST_MODE

/**
 * Verifies the test mode
 * group of commands received from the host.
 * It extracts the command opcode from the command packet
 * and analyzes the command packet depending on the type of
 * Command it calls the corresponding routines.
 * It sends command complete event for all the test mode commands.
 *
 * \param hci_cmd_ptr Pointer to HCI_CMD_PKT.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI Error code.
 *
 */
UCHAR hci_execute_test_mode_command_packet(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR ret_error = HCI_COMMAND_SUCCEEDED ;
    UINT16 ce_index = 0;

    BT_FW_HCI_INF(HCI_TEST_MODE_COMMAND_FUNCTION,0,0);

    switch(hci_cmd_ptr ->cmd_opcode)
    {
        case HCI_READ_LOOPBACK_MODE_OPCODE:
            if ((ret_error =hci_handle_read_loopback_command(hci_cmd_ptr))
                    != HCI_COMMAND_SUCCEEDED)
            {
                BT_FW_HCI_ERR(HCI_HANDLE_READ_LOOPBACK_COMMAND_FAILED,0,0);
            }
            break;

        case HCI_WRITE_LOOPBACK_MODE_OPCODE:
            ret_error = hci_handle_write_loopback_command(hci_cmd_ptr);
            if (ret_error != HCI_COMMAND_SUCCEEDED)
            {
                BT_FW_HCI_ERR(HCI_HANDLE_WRITE_LOOPBACK_COMMAND_FAILED,0,0);
            }
            break;

        case  HCI_ENABLE_DEVICE_UNDER_TEST_MODE_OPCODE:
            ret_error = hci_handle_enable_dut_command(hci_cmd_ptr);
            if (ret_error != HCI_COMMAND_SUCCEEDED)
            {
                BT_FW_HCI_ERR(HCI_HANDLE_ENABLE_DUT_COMMAND_FAILED,0,0);
            }
            break;

        default:
            if (bz_auth_handle_hci_security_commands(hci_cmd_ptr) == TRUE)
            {
                return BT_FW_SUCCESS;
            }
#ifdef _SUPPORT_SECURE_CONNECTION_
            if (IS_BT_SECURE_CONN)
            {
                ret_error = HCI_HANDLE_SECURE_CONN_TESTING_COMMAND(hci_cmd_ptr);
                if (ret_error != UNKNOWN_HCI_COMMAND_ERROR)
                {
                    break;
                }
            }
#endif
            /* hci_cmd_ptr->cmd_opcode = HCI_NOP ;*/
            /* We should send back the same command opcode */
            ret_error = UNKNOWN_HCI_COMMAND_ERROR;
            break;

    }

    /* Send connection complete event to the host.*/
    hci_generate_command_complete_event(hci_cmd_ptr->cmd_opcode,
                                        ret_error,ce_index, NULL);

    return BT_FW_SUCCESS ;
}
#endif /* TEST_MODE */

#ifdef COMPILE_PARK_MODE

/**
 * Sends the mode change event occured due to
 * new connection timeout during unparking.
 *
 * \param ce_index Connection Entity Index.
 *
 * \return None.
 *
 */
void hci_unpark_new_conn_timeout_event(UINT16 ce_index)
{
    BT_FW_HCI_INF(NEW_CONN_TIMEOUT_MODE_CHANGE_EVENT_TO_THE_HOST,0,0);

    hci_generate_mode_change_event(
        CONNECTION_TIMEOUT_ERROR, ce_index, LP_PARK_MODE, 0 );
}

#endif /* COMPILE_PARK_MODE */

/**
 * hci_send_next_packet. Used for Controller to Host flow control.
 *
 * \param None.
 *
 * \return None.
 *
 */
void hci_send_next_packet(void)
{
    UCHAR *data_buffer_ptr;
    UCHAR *data_header_ptr;
    UINT16 length,allowed_length,total_length;
    HCI_ACL_RX_DATA_PKT *hci_pkt;
    UINT16 connection_handle;
    UINT16 ce_index;
    HCI_DATA_RECV_Q *RxQ = &hci_received_data_Q;

    //RT_BT_LOG(GREEN, BT_FW_HCI_TASKS_1760, 0, 0);

    if ((lmp_self_device_data.flow_control_hc_to_host == 0x01) ||
        (lmp_self_device_data.flow_control_hc_to_host == 0x03))
    {
        BT_FW_HCI_INF(HOST_FLOW_CONTROL_IS_ON,0,0);

        if (lmp_host_info_data.host_total_num_acl_data_pkts != 0)
        {
            if (RxQ->queue_length == 0)
            {
                return;
            }

            if (RxQ->state[RxQ->rd_ptr] == DATA_SENT)
            {
                return;
            }

            hci_pkt = RxQ->data[RxQ->rd_ptr];
            if (RxQ->frag_offset[RxQ->rd_ptr] != 0)
            {
                hci_pkt->packet_boundary_flag = (UINT16)0x1;
            }
            allowed_length = lmp_host_info_data.host_acl_data_pkt_len;
            total_length = RxQ->length[RxQ->rd_ptr];

            RxQ->state[RxQ->rd_ptr] = DATA_SENT;

            BT_FW_HCI_INF(ALLOWED_LENGTH_TOTAL_LENGTH_OF_THIS_PACKET,2,
                          allowed_length,total_length);

            data_buffer_ptr = (UCHAR *) RxQ->data[RxQ->rd_ptr] +
                                        RxQ->frag_offset[RxQ->rd_ptr] + 4;

            if (RxQ->frag_offset[RxQ->rd_ptr] + allowed_length < total_length)
            {
                length = allowed_length;
                hci_pkt->acl_data_total_length = length;
                memcpy(&RxQ->header[RxQ->rd_ptr], hci_pkt, 4);
                data_header_ptr = (UCHAR *)&RxQ->header[RxQ->rd_ptr];
                connection_handle = RxQ->header[RxQ->rd_ptr] & 0x00000FFF;
                if (LMP_GET_CE_INDEX_FROM_CONN_HANDLE(connection_handle,
                                                     &ce_index) == API_SUCCESS)
                {
                    /* Increment the out standing buffer count for data packets
                     * for this connection */
                    /* It will be decremented when we get a num complete from
                     * the host or if the connection gets disconnected
                     */
                    lmp_connection_entity[ce_index].out_standing_data_to_host ++;
                    BT_FW_HCI_INF(INCREMENTING_OUTSTANDING_DATA_FOR_CH_OUTSTANDING_DATA,2,connection_handle,
                                  lmp_connection_entity[ce_index].out_standing_data_to_host);

                }
                BT_FW_HCI_INF(FRAGMENT_LENGTH,1,length);
                hci_td_deliver_acl_data_to_host(data_header_ptr,
                                                data_buffer_ptr);
                RxQ->frag_offset[RxQ->rd_ptr] += length;
            }
            else
            {
                length = total_length - RxQ->frag_offset[RxQ->rd_ptr];
                hci_pkt->acl_data_total_length = length;
                memcpy(&RxQ->header[RxQ->rd_ptr], hci_pkt, 4);
                data_header_ptr = (UCHAR *)&RxQ->header[RxQ->rd_ptr];
                connection_handle = RxQ->header[RxQ->rd_ptr] & 0x00000FFF;
                if (LMP_GET_CE_INDEX_FROM_CONN_HANDLE(
                            connection_handle,&ce_index) == API_SUCCESS)
                {
                    /* Increment the out standing buffer count for data packets
                     * for this connection */
                    /* It will be decremented when we get a num complete from
                     * the host or if the connection gets disconnected
                     */
                    lmp_connection_entity[ce_index].out_standing_data_to_host++;
                    BT_FW_HCI_INF(INCREMENTING_OUTSTANDING_DATA_FOR_CH_OUTSTANDING_DATA,2,connection_handle,
                                  lmp_connection_entity[ce_index].out_standing_data_to_host);
                }
                BT_FW_HCI_INF(FRAGMENT_LENGTH,1,length);
                hci_td_deliver_acl_data_to_host(data_header_ptr,
                                                data_buffer_ptr);
                RxQ->frag_offset[RxQ->rd_ptr] = 0;
            }
            lmp_host_info_data.host_total_num_acl_data_pkts --;
        }
    }
}

/**
 * Handles a received data from remote device (that
 * has to be sent to the host). The data is first queued in
 * a local queue and depending on the flow control from the
 * host controller to host it is given to the Transport for transmission.
 *
 * After a successful transmission the Queued data will be dequed
 * and the associated buffer freed on receiving a singal from the
 * transport of the successful transmission of the packet.
 *
 * \param hci_data_pkt Pointer to HCI_ACL_DATA_PKT.
 *
 * \return None.
 *
 */
void hci_handle_data_to_host(HCI_ACL_RX_DATA_PKT *hci_data_pkt)
{
    UCHAR *data_buffer_ptr,*data_header_ptr;
    UINT16 length;
    UINT8 hci_rx_wptr;

//{{ added by guochunxia --begin
    /* First Queue the received data packet in the device Queue */
    if (hci_received_data_Q.queue_length == REVC_DATA_Q_SIZE)
    {
        BT_FW_HCI_ERR(RECEIVE_QUEUE_IS_FULL_REMOTE_DEVICE_VIOLATED_FLOW_CONTROL,0,0);
        RT_BT_LOG(RED, BT_FW_HCI_TASKS_1882, 0, 0);
        return;
    }
//{{ added by guochunxia -- end

    length = (UINT16) hci_data_pkt->acl_data_total_length;
    hci_rx_wptr = hci_received_data_Q.wr_ptr;

    memcpy(&hci_received_data_Q.header[hci_rx_wptr],hci_data_pkt,4);
    data_header_ptr = (UCHAR *)&hci_received_data_Q.header[hci_rx_wptr];
    data_buffer_ptr = hci_data_pkt->hci_acl_data_pkt;

    BT_FW_HCI_INF(QUEING_DATA_IN_HCI_EVENT_TASK_HEADER,1,data_header_ptr);

    hci_received_data_Q.data[hci_rx_wptr] = hci_data_pkt;
    hci_received_data_Q.length[hci_rx_wptr] = length;
    hci_received_data_Q.state[hci_rx_wptr] = DATA_IDLE;

    hci_received_data_Q.wr_ptr++;
    hci_received_data_Q.wr_ptr %= REVC_DATA_Q_SIZE;
    hci_received_data_Q.queue_length++;

    /* Check if host flow control is turned on , Fragment the
     * packet according to the buffer size in the host
     */
    if ((lmp_self_device_data.flow_control_hc_to_host == 0x01) ||
            (lmp_self_device_data.flow_control_hc_to_host == 0x03))
    {
        hci_send_next_packet();
    }
    else
    {
        /* Send the data to TD for transmission. */
        hci_td_deliver_acl_data_to_host(data_header_ptr,data_buffer_ptr);
    }
}


/**
 * hci_handle_data_tx_completed.
 *
 * \param param (UCHAR *).
 *
 * \return None.
 *
 */
void hci_handle_data_tx_completed(UCHAR param)
{
    //  UCHAR res_cnt_dec = TRUE;
    UCHAR pkt_type;

    pkt_type = param;
    hci_received_data_Q.state[hci_received_data_Q.rd_ptr] = DATA_IDLE;
    if ((lmp_self_device_data.flow_control_hc_to_host == 0x0) ||
            (lmp_self_device_data.flow_control_hc_to_host == 0x2) ||
            (hci_received_data_Q.frag_offset[hci_received_data_Q.rd_ptr] == 0))
    {
#ifdef LE_MODE_EN
        if (pkt_type == HCI_TRANSPORT_LE_ACL_DATA_PKT_TYPE)
        {
            OS_FREE_BUFFER(tx_table[LE_ACL_DATA_HANDLER_TASK].pool_handle,
                  hci_received_data_Q.data[hci_received_data_Q.rd_ptr]);

#ifdef _ENABLE_BLE_LL_FLOW_CONTROL_
            os_release_one_le_reserved_buffer();
#endif

        }
        else
#endif
        {
            OS_FREE_BUFFER(tx_table[ACL_DATA_HANDLER_TASK].pool_handle,
                 hci_received_data_Q.data[hci_received_data_Q.rd_ptr]);
            os_free_reserved_buffer();
            hci_vendor_check_log_data_packet(VENDOR_LOG_PACKET_TYPE_ACL,
                (UINT8 *) hci_received_data_Q.data[hci_received_data_Q.rd_ptr],
                TRUE);
        }
        hci_received_data_Q.data[hci_received_data_Q.rd_ptr] = NULL;
        hci_received_data_Q.rd_ptr++;
        hci_received_data_Q.rd_ptr %= REVC_DATA_Q_SIZE;
        hci_received_data_Q.queue_length --;
    }
    if ((lmp_self_device_data.flow_control_hc_to_host == 0x01) ||
            (lmp_self_device_data.flow_control_hc_to_host == 0x03))
    {
        if (hci_received_data_Q.state[hci_received_data_Q.rd_ptr] == DATA_IDLE)
        {
            hci_send_next_packet();
        }
    }
    return;
}

/**
 * Handles the num completed packets from host.
 *
 * \param num_completed_pkts  Number of completed packets.
 *
 * \return None.
 *
 */
void handle_host_num_of_comp_pkts(UINT16 num_completed_pkts)
{
    /* If the Host flow control is not turned on , Ignore this command */
    if ((lmp_self_device_data.flow_control_hc_to_host == 0x01) ||
            (lmp_self_device_data.flow_control_hc_to_host == 0x03))
    {
        /* Update the host number of completed packets */
        lmp_host_info_data.host_total_num_acl_data_pkts += num_completed_pkts;
        BT_FW_HCI_INF(NOW_HOST_TOTAL_NUM_ACL_DATA_PKTS,1,
                      lmp_host_info_data.host_total_num_acl_data_pkts);
        if (hci_received_data_Q.state[hci_received_data_Q.rd_ptr] == DATA_IDLE)
        {
            hci_send_next_packet();
        }
    }
}

/**
 * Performs the required action because of the masked event.
 *
 * \param ce_index ACL connection index.
 * \param event_opcode masked event opcode.
 *
 * \return TRUE if hanlded, otherwise  FALSE.
 */
BOOLEAN hci_handle_masked_event(UINT16 ce_index, UCHAR event_opcode)
{
    switch (event_opcode)
    {
        case HCI_PIN_CODE_REQUEST_EVENT:
        case HCI_LINK_KEY_REQUEST_EVENT:
        case HCI_IO_CAPABILITY_REQUEST_EVENT:
        case HCI_USER_CONFIRMATION_REQUEST_EVENT:
        case HCI_USER_PASSKEY_REQUEST_EVENT:
        case HCI_REMOTE_OOB_DATA_REQUEST_EVENT:
        case HCI_USER_PASSKEY_NOTIFICATION_EVENT:
            return bz_auth_handle_masked_event(ce_index, event_opcode);
        default:
            break;

    }
    return TRUE;
}
#ifdef _DAPE_DETECT_WHCK_P2P
void detect_whck_p2p(HCI_CMD_PKT *hci_cmd_ptr, UCHAR *cmd_group_opcode)
{
    UINT8 change_opcode = FALSE;

    do
    {
#ifdef _DAPE_NO_REMOTE_NAME_WHEN_WHCK_P2P
        if (hci_cmd_ptr->cmd_opcode == HCI_REMOTE_NAME_REQUEST_OPCODE)
        {
            if ((g_whck_running) && (g_whck_item == 1))
            {
                hci_generate_command_status_event(hci_cmd_ptr->cmd_opcode,
                      COMMAND_DISALLOWED_ERROR);
                change_opcode = TRUE;
                break;
            }
        }
#endif
        if (g_whck_p2p_running_detect_countdown)
        {
          /* If there are any other cmd between these commands, then
             it is not WHCK-testing. Decection ends. */
          if ((hci_cmd_ptr->cmd_opcode != HCI_WRITE_SCAN_ENABLE_OPCODE) &&
             (hci_cmd_ptr->cmd_opcode != HCI_CHANGE_LOCAL_NAME_OPCODE) &&
             (hci_cmd_ptr->cmd_opcode != HCI_VENDOR_ENTER_WHCK))
          {
              g_whck_p2p_running_detect_countdown = 0;
              g_p2p_detect_step = 0;
          }

        }

        if (hci_cmd_ptr->cmd_opcode == HCI_WRITE_SCAN_ENABLE_OPCODE)
        {
            if ((hci_cmd_ptr->cmd_parameter[0] == 2) &&
                (g_whck_p2p_running_detect_countdown == 0))
            {
                g_whck_p2p_running_detect_countdown = 200;
                g_p2p_detect_step |= BIT0;
            }
            else
            {
                if ((hci_cmd_ptr->cmd_parameter[0] == 3) &&
                    (g_whck_p2p_running_detect_countdown) &&
                    (g_p2p_detect_step == 3))
                {
                    auto_enter_whck(1);
                    g_whck_p2p_running_detect_countdown = 0;
                }
            }
        }
        if (hci_cmd_ptr->cmd_opcode == HCI_CHANGE_LOCAL_NAME_OPCODE)
        {
            if (g_whck_p2p_running_detect_countdown)
            {
                g_p2p_detect_step |= BIT1;
            }
        }
    }
    while (0);
    if (change_opcode == TRUE)
    {
        *cmd_group_opcode = RTK_VENDOR_PARAMS_OPCODE_GROUP;
    }
}

void auto_enter_whck(UINT8 whck_item)
{
    g_whck_running = TRUE;
    if (whck_item != 0xFF)
    {
        g_whck_item = whck_item;
    }
    lmp_self_device_data.page_scan_interval = 0x200;
    lmp_self_device_data.page_scan_window = 0x12;
    /* (dape 20131018) do not make inq scan interval too short which will
    result in failure in WHCK P2P hct2.4.01.*/
    lmp_self_device_data.inquiry_scan_interval = 0x600;
    lmp_self_device_data.inquiry_scan_window = 0x12;
    lmp_self_device_data.inquiry_scan_type = HCI_INTERLACED_INQ_SCAN;

}
#endif

