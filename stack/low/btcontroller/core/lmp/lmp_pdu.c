/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  Contains LMP PDU handlers implementation.
 */

/********************************* Logger *************************/
enum { __FILE_NUM__= 53 };
/********************************* Logger *************************/


/* ========================= Include File Section ========================= */
#include "lmp_internal.h"
#include "vendor.h"
#include "hci_vendor_defines.h"
#include "hci_vendor_internal.h"
#include "bz_debug.h"
#include "bt_fw_acl_q.h"
#include "UartPrintf.h"
#include "bt_fw_hci_2_1.h"
#include "lmp_2_1.h"
#include "mem.h"
#include "lmp_pdu_q.h"
#include "lc.h"
#ifdef _SECURE_CONN_TEST_LOG
#include "bz_auth_internal.h"
#endif

/* ==================== Structure declaration Section ===================== */



/* ===================== Variable Declaration Section ===================== */
#ifdef TEST_MODE
TimerHandle_t lmp_tci_timer = NULL;
TimerHandle_t lmp_tester_tci_timer = NULL;
extern UCHAR lc_test_mode_ack_recd;
extern UCHAR lc_tci_pause_flag;
HCI_EVENT_PKT *hci_generate_test_control_event(UINT16 ce_index);
extern UINT8  test_mode_tx_mode_force_ack;
#ifdef _RTL8723B_DUT_MODE_ 
extern UINT16 test_mode_tx_mode_reg_catch;
#endif
#endif
#ifdef _DAPE_NO_ROLE_SW_WHEN_ROLE_SW
extern UINT16 g_role_switch_status;
#endif
#ifdef COMPILE_ESCO
extern LMP_ESCO_CONNECTION_ENTITY lmp_esco_connection_entity
[LMP_MAX_ESCO_CONN_ENTITIES];
#endif
extern LUT_EXTENSION_TABLE lut_ex_table[LC_MAX_NUM_OF_LUT_EX_TABLES];
/**PATCH_PENDING_ENCRY_WHILE_ROLE_SWITCH*/
#ifdef _PENDING_ENCRY_WHILE_ROLE_SWITCH
extern UINT8 g_need_pend_encry;
extern UINT8 g_encry_pending;
extern OS_SIGNAL g_signal_ptr_encry;
#endif
#ifdef _STOP_AFH_TIMER_WHILE_ROLE_SWITCH
extern UINT8 g_stop_afh_timer;
#endif

extern UINT16 lc_get_current_power_value(UCHAR am_addr, UCHAR phy_piconet_id);
#ifdef _DAPE_TEST_AUTO_CONN	
extern SECTION_SRAM LEGACY_WHITE_LIST conn_white_list[LEGACY_MAX_WHITE_LIST];
extern SECTION_SRAM UINT8 num_of_white_list_device;
#endif
/* ================== Static Function Prototypes Section ================== */



/* ===================== Function Definition Section ====================== */

#ifdef TEST_MODE
void lmp_handle_test_control_accepted(LMP_PDU_PKT *lmp_pdu_ptr,
                                      UINT16 ce_index);
void tci_lmp_timeout_handler(TimerHandle_t timer_handle);
#endif

extern void lmp_kill_sco_connection(UINT16 ce_index, UINT16 sco_ce_index,
                                    UCHAR reason);

#/* ================== For rom code patch function point ================== */
#ifdef _ROM_CODE_PATCHED_
#ifndef _REDUCE_LPS_AFTER_RTL8703B_
#ifdef _CCH_8723_RCP_
PF_ROM_CODE_PATCH_FUNC rcp_lmp_handle_max_slot_req_pdu = NULL;
#endif
#endif
#ifdef _DAPE_8723_RCP_
PF_ROM_CODE_PATCH_FUNC rcp_lmp_handle_conn_req_func = NULL;    
#endif
#endif

/**
 * Handles the LMP_accepted pdu from the remote device and takes appropriate
 * action.
 *
 * \param lmp_pdu_ptr Pointer to LMP PDU.
 * \param ce_index ACL Connection entity index.
 *
 * \return TRUE if handled the PDU.
 */
BOOLEAN lmp_handle_accepted_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index)
{
    UCHAR transaction_id ;
    UCHAR pdu_opcode ;

    pdu_opcode = lmp_pdu_ptr->payload_content[1] ;
    transaction_id = (UCHAR)(lmp_pdu_ptr->payload_content[0] & 0x01);

    switch(pdu_opcode)
    {
        case LMP_HOST_CONNECTION_REQ_OPCODE :
            lmp_handle_host_connection_req_accepted(ce_index);
            break;

#ifdef ENABLE_SCO
        case LMP_SCO_LINK_REQ_OPCODE:
            lmp_handle_sco_link_req_accepted(ce_index);
            break;

        case LMP_REMOVE_SCO_LINK_REQ_OPCODE:
#if 1
            {
                UINT16 sco_ce_index;

                if (lmp_get_sco_ce_index_from_ce_index(ce_index, SCO_DISCONNECTING,
                                                       &sco_ce_index) != API_SUCCESS)
                {
                    BZ_ASSERT(0, "I hate obscure error handling!!");
                }

                lmp_kill_sco_connection(ce_index, sco_ce_index,
                                        CONNECTION_TERMINATED_LOCAL_HOST_ERROR);
            }
#endif

            break;      /* NOP */
#endif

#ifdef COMPILE_HOLD_MODE
        case LMP_HOLD_REQ_OPCODE :
            lmp_handle_hold_req_accepted(ce_index);
            break;
#endif /* COMPILE_HOLD_MODE */

#ifdef COMPILE_SNIFF_MODE
        case LMP_SNIFF_REQ_OPCODE :
            lmp_handle_sniff_req_accepted(ce_index);
            break;
        case LMP_UNSNIFF_REQ_OPCODE :
            lmp_handle_unsniff_req_accepted(ce_index);
            break;
#endif /* COMPILE_SNIFF_MODE */

#ifdef COMPILE_PARK_MODE
        case LMP_PARK_REQ_OPCODE :
            lmp_handle_park_req_accepted(transaction_id, ce_index);
            break;
        case LMP_UNPARK_PM_ADDR_REQ_OPCODE : /* Fall Through */
        case LMP_UNPARK_BD_ADDR_REQ_OPCODE :
            lmp_handle_unpark_req_accepted(transaction_id, ce_index);
            break;
#endif /* COMPILE_PARK_MODE */

#ifndef COMPILE_SINGLE_SLOT_PACKETS_ONLY
        case LMP_MAX_SLOT_REQ_OPCODE :
            lmp_handle_max_slot_req_accepted(transaction_id, ce_index);
            break;
#endif

#ifdef COMPILE_ROLE_SWITCH
        case LMP_SWITCH_REQ_OPCODE :
            lmp_handle_switch_req_accepted(transaction_id, ce_index);
            break;
#endif

        case LMP_QoS_REQ_OPCODE :
            lmp_handle_qos_req_accepted(ce_index);
            break ;

#ifdef OPTIONAL_PAGING
        case LMP_PAGE_MODE_REQ_OPCODE:
            lmp_handle_page_mode_accepted(lmp_pdu_ptr,ce_index);
            break;

        case LMP_PAGE_SCAN_MODE_REQ_OPCODE:
            lmp_handle_page_scan_mode_accepted(lmp_pdu_ptr,ce_index);
            break;
#endif /* OPTIONAL_PAGING */
        case LMP_TEST_ACTIVATE_OPCODE: /* fall through */
            break;
        case LMP_TEST_CONTROL_OPCODE: /* fall through */
#ifdef TEST_MODE
            lmp_handle_test_control_accepted(lmp_pdu_ptr,ce_index);
#endif /* TEST_MODE */
            break;
        default:
#if 0
            lmp_send_lmp_not_accepted(ce_index,pdu_opcode,
                                      REMOTE_DEV_TID,
                                      UNKNOWN_LMP_PDU_ERROR);
#endif

            return FALSE;

    }

    return TRUE;
}

#ifdef OPTIONAL_PAGING
/**
 * Handles the LMP_accepted for page mode request pdu.
 *
 * \param lmp_pdu_ptr Pointer to the LMP pdu.
 * \param ce_index ACL Connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_page_mode_accepted(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index)
{
    DEVICE_INFO dev_info ;

#ifdef POWER_CONTROL
    UCHAR parameter_list[LMP_INCR_POWER_REQ_LEN];
    UINT16 rssi ;
#endif

    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    /* Update the device info table. */
    memcpy(dev_info.bd_addr, ce_ptr->bd_addr, LMP_BD_ADDR_SIZE);
    dev_info.page_mode = ce_ptr->optional_page_scheme;
    dev_info.page_mode_settings = ce_ptr->optional_page_setting ;

    /* Update page scheme setting value....*/
    LMP_INF(ADDING_THE_PAGE_MODE_INFO_TO_DEVICE_CACHE,0,0);
    lmp_add_to_device_cache(dev_info);

#ifdef POWER_CONTROL
    if((ce_ptr->feat_page0[2] & LMP_POWER_CONTROL_FEATURE) &&
            (lmp_feature_data.feat_page0[2] & LMP_POWER_CONTROL_FEATURE))
    {
        /*Send power control PDU*/
        if(ce_ptr->rssi_meas_flag == TRUE)
        {
            lmp_send_power_ctrl_pdu(ce_index,REMOTE_DEV_TID);
            ce_ptr->rssi_meas_flag = FALSE;
        }
    }
#endif

    return BT_FW_SUCCESS;
}

/**
 * Handles the LMP_accepted for page scan mode request pdu.
 *
 * \param lmp_pdu_ptr Pointer to the LMP pdu.
 * \param ce_index ACL Connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_page_scan_mode_accepted(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index)
{
    DEVICE_INFO dev_info ;
    HCI_EVENT_PKT *event_pkt ;
    OS_SIGNAL signal_send ;
    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    /*Update the self device database.Self device can go for optioal page scan*/
    lmp_self_device_data.opt_page_scan_flag = TRUE ;

    /*Update the device info table...*/
    memcpy(dev_info.bd_addr, ce_ptr->bd_addr, LMP_BD_ADDR_SIZE);
    dev_info.page_mode = ce_ptroptional_page_scheme;
    dev_info.page_mode_settings = ce_ptr->optional_page_setting;

    lmp_add_to_device_cache(dev_info);

    /*Send command complete event...*/
    if(hci_generate_command_complete_event(
                HCI_WRITE_PAGE_SCAN_MODE_OPCODE,
                HCI_COMMAND_SUCCEEDED,ce_index)
            == API_FAILURE)
    {
        return BT_FW_ERROR ;
    }

    return BT_FW_SUCCESS;
}

#endif /* OPTIONAL_PAGING */

#ifdef COMPILE_ROLE_SWITCH
/**
 * Handles the LMP_accepted for LMP_switch_req pdu.
 *
 * \param transaction_id Transaction ID.
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_switch_req_accepted(UCHAR transaction_id, UINT16 ce_index)
{
    UINT32 native_clock;
    LMP_CONNECTION_ENTITY* ce_ptr;
#ifdef _DAPE_TEST_NO_ROLE_SW_AFTER_DETACH
    if (lmp_connection_entity[ce_index].entity_status == UNASSIGNED)
    {
        return BT_FW_SUCCESS;
    }
#endif
    ce_ptr = &lmp_connection_entity[ce_index];

    switch (ce_ptr->ce_status)
    {
        case LMP_ROLE_SWITCHING:            /* after connection role switch */
        case LMP_CONNECTION_ROLE_SWITCH:    /* during connection role switch */
            break;

        default:
            LMP_INF(INVALID_STATE_ROLE_SWITCH_ACCEPTED_PDU_RECD,0,0);
            return BT_FW_SUCCESS;
    }

    /* Get the current Clock in this piconet */
    lc_get_clock_in_scatternet(&native_clock, ce_ptr->phy_piconet_id);
    native_clock >>= 1;

    /* If the Switch instant is already in the Past */
    if (ce_ptr->switch_instant < native_clock)
    {
        ce_ptr->role_switch_accepted_flag = FALSE;
    }
    else
    {
        ce_ptr->role_switch_accepted_flag = TRUE;
    }

    /* Start the switch. */
    lmp_switch_role_in_scatternet(ce_index);

    return BT_FW_SUCCESS;
}
#endif /* COMPILE_ROLE_SWITCH */

#ifndef COMPILE_SINGLE_SLOT_PACKETS_ONLY
/**
 * Handles the LMP_accepted for LMP_max_slot_req pdu.
 *
 * \param transaction_id Transaction ID.
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_max_slot_req_accepted(UCHAR transaction_id, UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY *ce_ptr;
    UCHAR lcl_max_slot = 5;

    ce_ptr = &lmp_connection_entity[ce_index];

#ifdef ENABLE_LOGGER_LEVEL_2
    LMP_LOG_INFO(LOG_LEVEL_LOW, LAST_MAX_SLOT_REQ_SENT, 1,
                 ce_ptr->last_max_slot_req_sent);
#endif

    if(ce_ptr->last_max_slot_req_sent == LMP_LAST_MAX_SLOT_REQ_SENT_5_SLOT)
    {
        lcl_max_slot = 5;
    }
    else if (ce_ptr->last_max_slot_req_sent == LMP_LAST_MAX_SLOT_REQ_SENT_3_SLOT)
    {
        lcl_max_slot = 3;
    }
    else
    {
        /* This should never happen. */
        lcl_max_slot = 1;
    }

    hci_generate_max_slots_change_event(
        ce_ptr->connection_type.connection_handle,
        lcl_max_slot);

    /* Send connection packet type changed event */
    if (ce_ptr->hci_cmd_bits & CHANGE_CONN_PKT_TYPE_BIT_MASK)
    {
        ce_ptr->hci_cmd_bits &= (~CHANGE_CONN_PKT_TYPE_BIT_MASK);

        hci_generate_connection_packet_type_changed_event(
            HCI_COMMAND_SUCCEEDED,
            ce_ptr->connection_type.connection_handle,
            ce_ptr->connection_type.packet_type);
    }

    if (ce_ptr->last_max_slot_req_sent == LMP_LAST_MAX_SLOT_REQ_SENT_5_SLOT)
    {
        ce_ptr->last_accepted_max_slot = LMP_LAST_ACCEPTED_MAX_SLOT_5;
    }
    else if (ce_ptr->last_max_slot_req_sent == LMP_LAST_MAX_SLOT_REQ_SENT_3_SLOT)
    {
        ce_ptr->last_accepted_max_slot = LMP_LAST_ACCEPTED_MAX_SLOT_3;
    }
    else
    {
        ce_ptr->last_accepted_max_slot = LMP_LAST_ACCEPTED_MAX_SLOT_1;
    }

    lc_update_pkts_allowed(ce_index);


#ifdef COMPILE_CQDDR
    if (!IS_USE_FOR_MUTE)
    {
        lmp_calculate_and_send_preferred_rate_pdu(ce_index);
    }
#endif

    return BT_FW_SUCCESS;
}
#endif

/**
 * Handles LMP_accepted for LMP_qos_req pdu.
 *
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_qos_req_accepted(UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY *ce_ptr;
    ce_ptr = &lmp_connection_entity[ce_index];

#if 0
    /* Check for correct state */
    switch(ce_ptr->ce_status)
    {
        default :
            return BT_FW_ERROR ;
    }
#endif

    /* Send QOS setup complete event...*/
    if(ce_ptr->qos_tn_details == QOS_TN_QOS_SETUP)
    {
        hci_generate_QoS_setup_complete_event(
            HCI_COMMAND_SUCCEEDED,
            ce_index);
    }
    else if( (ce_ptr->qos_tn_details == QOS_TN_FLOW_SPEC_TX_SIDE) ||
             (ce_ptr->qos_tn_details == QOS_TN_FLOW_SPEC_RX_SIDE) )
    {
        hci_generate_flow_spec_complete_event(HCI_COMMAND_SUCCEEDED,
                                              ce_ptr->qos_tn_details, ce_index);
    }

    ce_ptr->qos_tn_details = QOS_TN_INVALID;

    ce_ptr->Tpoll = ce_ptr->qos_tpoll;
    if(ce_ptr->qos_tpoll > 4*3)
    {
        ce_ptr->Tpoll = ce_ptr->qos_tpoll - 8;
    }

#ifdef COMPILE_SNIFF_MODE
    if(ce_ptr->in_sniff_mode == FALSE)
#endif
    {
        BB_start_tpoll(ce_ptr->am_addr, (UINT16) (ce_ptr->Tpoll - 4),
                       ce_ptr->phy_piconet_id);
    }

    return BT_FW_SUCCESS ;
}

/**
 * Handles the LMP_accepted for LMP_host_connection_req pdu.
 *
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_host_connection_req_accepted(UINT16 ce_index)
{
    /* Connection request is accepted (may be role-switch was also completed).
     * Now we may have to proceed with the authentication procedures, if
     * enabled.
     */
#ifdef _SECURE_CONN_TEST_LOG
{
        BZ_AUTH_LINK_PARAMS* auth;
        auth = lmp_connection_entity[ce_index].auth;

        //auth->secure_conn_enabled == 1;
        //auth->len_prime = 8;
        RT_BT_LOG(RED, DAPE_TEST_LOG522, 2,auth->secure_conn_enabled, auth->len_prime);
}
#endif
    /* Handle authentication related prcedures */
    if (bz_auth_initiate_authentication_during_connection(ce_index))
    {
        /* Upon completion of the authentication procedures (may be including
         * encryption), the auth_module will invoke the registered
         * auth_completion (refer bz_auth_register_auth_completed_callback())
         * callback.
         * Then the callback function has to proceed with the ACL connection
         * procedure.
         */
        return BT_FW_SUCCESS;
    }
#ifdef _SECURE_CONN_TEST_LOG
{
        BZ_AUTH_LINK_PARAMS* auth;
        auth = lmp_connection_entity[ce_index].auth;

        //auth->secure_conn_enabled == 1;
        //auth->len_prime = 8;
        RT_BT_LOG(RED, DAPE_TEST_LOG522, 2,auth->secure_conn_enabled, auth->len_prime);
}
#endif
    /* For Set up complete PDU, Put the transaction ID of the self device */
    lmp_decide_to_send_conn_complete_evt(ce_index);

    return BT_FW_SUCCESS;
}

#ifdef COMPILE_HOLD_MODE
/**
 * Handles the LMP_accepted for LMP_hold_req pdu.
 *
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_hold_req_accepted(UINT16 ce_index)
{
    UINT32 cur_clk;
    LMP_CONNECTION_ENTITY *ce_ptr;
    UINT32 instant_28_bit_val;

    ce_ptr = &lmp_connection_entity[ce_index];

    /* Reset the hold interval values */
    ce_ptr->hold_mode_max_interval = otp_str_data.bt_hold_max_interval;
    ce_ptr->hold_mode_min_interval = otp_str_data.bt_hold_min_interval;
    ce_ptr->hold_max_interval_negotiated = 0x00;
    ce_ptr->hold_min_interval_negotiated = 0x00;

    ce_ptr->hold_mode_accepted_flag = TRUE ;
    if(ce_ptr->hold_mode_interval > ce_ptr->hold_neg_max_interval)
    {
        ce_ptr->hold_neg_max_interval = ce_ptr->hold_mode_interval;
    }

    /* Check if the hold-instant has already passed. */
    /* If MSB of clock-current-value and instant-clock are same,
    and instant is in the past, we will not enter hold mode. */

    lc_get_clock_in_scatternet(&cur_clk , ce_ptr->phy_piconet_id);

    instant_28_bit_val = (ce_ptr->hold_instant << 1);

    if (instant_28_bit_val < cur_clk)
    {
        if ((cur_clk & (BT_CLOCK_27_BITS + 1) ) ==
                (instant_28_bit_val & (BT_CLOCK_27_BITS + 1) ))
        {
            LMP_LOG_INFO(LOG_LEVEL_HIGH, HOLD_INSTANT_REQUESTED_HAS_PASSED, 1, instant_28_bit_val);

            hci_generate_mode_change_event(
                INSTANT_PASSED_ERROR,
                ce_index,
                LP_ACTIVE_MODE,
                0);

            lmp_set_ce_status(ce_index, LMP_CONNECTED);

            return BT_FW_SUCCESS ;
        }
        else
        {
            /* Clk wrap around case. Proceed with normal flow. */
        }
    }

    lmp_handle_start_hold_mode(ce_index);

    return BT_FW_SUCCESS ;
}
#endif /* COMPILE_HOLD_MODE */

#ifdef COMPILE_PARK_MODE
/**
 * Handles the LMP_accepted for LMP_unpark_req pdu.
 *
 * \param transaction_id Transaction ID.
 * \param ce_index ACL Connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_unpark_req_accepted(UCHAR transaction_id,
                                     UINT16 ce_index)
{
    UCHAR am_addr ;
    LMP_CONNECTION_ENTITY *ce_ptr;
    UCHAR phy_piconet_id;

    DEF_CRITICAL_SECTION_STORAGE;

    ce_ptr = &lmp_connection_entity[ce_index];
    am_addr = lmp_connection_entity[ce_index].am_addr ;

    /* The slave can send LMP_Accepted for park request.
     * Master shall not send LMP_Accepted for park request.
     */
    if(ce_ptr->ce_status != LMP_PARK_MODE)
    {
        LMP_ERR(RECEIVED_ACCEPETED_FOR_UNPARK_PM_ADDR_WHILE_WE_ARE_NOT_IN_PARK,0,0);
        return BT_FW_ERROR;
    }

    lmp_set_ce_status(ce_index, LMP_CONNECTED);
    ce_ptr->temp_ce_status = LMP_CONNECTED;
    ce_ptr->lmp_expected_pdu_opcode = BIT_MASK_NO_PDU;

    /* Send Mode change event */
    if(ce_ptr->hci_unpark_req_flag != HCI_UNPARK_AUTO_CONTROLLER_INITIATED)
    {
        hci_generate_mode_change_event(HCI_COMMAND_SUCCEEDED, ce_index,
                                       LP_ACTIVE_MODE, 0x00);

        RT_BT_LOG(GRAY, LMP_PDU_673, 1, ce_ptr->hci_unpark_req_flag);
    }

    MINT_OS_ENTER_CRITICAL();
    ce_ptr->unpark_req_flag = LMP_UNPARK_IDLE;
    MINT_OS_EXIT_CRITICAL();

    /* For addressing lut_ex_table */
    phy_piconet_id = ce_ptr->phy_piconet_id;

    {
        UCHAR lcl_lut_index;

        lcl_lut_index = lc_get_lut_index_from_phy_piconet_id(
                            am_addr, phy_piconet_id);

        lut_ex_table[lcl_lut_index].index_in_CE = ce_index;
    }

#if 0
    /* Send Mode change event */
    hci_generate_mode_change_event(HCI_COMMAND_SUCCEEDED, ce_index,
                                   LP_ACTIVE_MODE, 0x00);
#endif

    /* Release the pm address and the ar address */
    lmp_put_pm_addr(ce_ptr->pm_addr, ce_ptr->phy_piconet_id);
    lmp_put_ar_addr(ce_ptr->ar_addr, ce_ptr->phy_piconet_id);

//	LMP_INF(STARTING_THE_TPOLL_TIMER,0,0);

    if(ce_ptr->low_power_disconnect_state == UNPARK_DISCONNECT)
    {
        ce_ptr->low_power_disconnect_state = IDLE_STATE;

        /* Kill ACL connection */
        if(lmp_handle_acl_disconnect(ce_index, ce_ptr->disconnect_reason)
                == API_FAILURE)
        {
            BT_FW_HCI_ERR(LMP_HANDLE_ACL_DISCONNECT_FAILED,0,0);
            return BT_FW_ERROR;
        }
    }
    else
    {
        if(ce_ptr->hci_unpark_req_flag == HCI_UNPARK_AUTO_CONTROLLER_INITIATED)
        {
            /* Park the slave again. */
            UCHAR pm_addr, ar_addr;
            UCHAR parameter_list[LMP_PARK_REQ_LEN];

            lmp_set_auto_unpark_cnt(ce_index);

            ce_ptr->pm_addr = 0;
            if(lmp_get_pm_addr(&pm_addr, ce_ptr->phy_piconet_id) != API_SUCCESS)
            {
                BT_FW_HCI_ERR(log_file, "Error Getting PM address.");
            }
            ce_ptr->ar_addr = 0;
            if(lmp_get_ar_addr(&ar_addr, ce_ptr->phy_piconet_id) != API_SUCCESS)
            {
                BT_FW_HCI_ERR(log_file,"Error Getting AR address.");
            }

            ce_ptr->pm_addr = pm_addr;
            ce_ptr->ar_addr = ar_addr;

#ifdef COMPILE_NESTED_PAUSE_RESUME
            aclq_mark_am_addr_as_paused(ce_ptr->am_addr,
                                        ce_ptr->phy_piconet_id, ACL_PAUSED_PARK);
#else /* COMPILE_NESTED_PAUSE_RESUME */
            aclq_mark_am_addr_as_paused(ce_ptr->am_addr,
                                        ce_ptr->phy_piconet_id);
#endif /* COMPILE_NESTED_PAUSE_RESUME */

            /* Send a LMP_Park_Req PDU to the remote device */
            parameter_list[0] = LMP_PARK_REQ_OPCODE;
            /* Timing control flags */
            parameter_list[2] = ce_ptr->timing_control_flag;
            parameter_list[3] = LSB(ce_ptr->Dbeacon);
            parameter_list[4] = MSB(ce_ptr->Dbeacon);
            parameter_list[5] = LSB(ce_ptr->Tbeacon);
            parameter_list[6] = MSB(ce_ptr->Tbeacon);
            parameter_list[7] = ce_ptr->Nbeacon ;
            parameter_list[8] = ce_ptr->Delta_beacon ;
            parameter_list[9] = ce_ptr->pm_addr ;
            parameter_list[10] = ce_ptr->ar_addr ;
            parameter_list[11] = ce_ptr->NB_sleep ;
            parameter_list[12] = ce_ptr->DB_sleep ;
            parameter_list[13] = ce_ptr->D_access ;
            parameter_list[14] = ce_ptr->T_access ;
            parameter_list[15] = ce_ptr->N_acc_slots ;
            parameter_list[16] = ce_ptr->N_poll ;
            parameter_list[17] = (UCHAR)((ce_ptr->access_scheme << 4) |
                                         ce_ptr->M_access);

            lmp_generate_pdu(ce_index, parameter_list, LMP_PARK_REQ_LEN,
                             SELF_DEV_TID, LMP_PARK_MODE_REQ);

            ce_ptr->lmp_expected_pdu_opcode = BIT_MASK_NO_PDU;

            ce_ptr->hci_unpark_req_flag = HCI_UNPARK_IDLE;

        }
        else
        {
            OS_SIGNAL signal_send;

#ifdef COMPILE_AFH_HOP_KERNEL
#ifdef AFH_PARK
            if((ce_ptr->remote_dev_role == SLAVE) &&
                    ((ce_ptr->link_supervision_timeout == 0) ||
                     (ce_ptr->supto_auto_repark == FALSE)))
            {
                ce_ptr->afh_disabled_for_park = FALSE;
                ce_ptr->park_pending_for_afh = FALSE;

                if (ce_ptr->remote_dev_role == SLAVE)
                {
                    OS_SIGNAL sig_send;
                    sig_send.type = LMP_UPDATE_MAP_SIGNAL;
                    sig_send.param = (OS_ADDRESS)((UINT32)ce_index);
                    OS_SEND_SIGNAL_TO_TASK(lmp_task_handle, sig_send);
                }
            }
#endif
#endif
            signal_send.type = LMP_LLC_CON_SETUP_COMPLETE;
            signal_send.param = (OS_ADDRESS)((UINT32)ce_index);

            ce_ptr->hci_unpark_req_flag = HCI_UNPARK_IDLE;

            if(OS_SEND_SIGNAL_TO_TASK(lmp_task_handle, signal_send)
                    != BT_ERROR_OK)
            {
                LMP_ERR(LOG_LEVEL_HIGH, OS_SEND_SIGNAL_TO_TASK_FAILED,0,0);
            }
        }
    }

    return BT_FW_SUCCESS;
}

/**
 * Handles the LMP_accepted for LMP_park_req pdu.
 *
 * \param transaction_id Transaction ID.
 * \param ce_index ACL Connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_park_req_accepted(UCHAR transaction_id, UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    /* The slave can send LMP_Accepted for park request.
     * Master shall not send LMP_Accepted for park request.
     */
    if(ce_ptr->ce_status != LMP_PARK_MODE_REQ)
    {
//		LMP_INF(GOT_ACCEPTED_FOR_PARK_MODE_WITHOUT_GETTING_A_REQ,0,0);
        return BT_FW_ERROR;
    }

    lmp_start_park_mode_timer(ce_index);

    return BT_FW_SUCCESS ;
}
#endif /* COMPILE_PARK_MODE */

#ifdef COMPILE_SNIFF_MODE
/**
 * Handles the LMP_accepted for LMP_unsniff_req pdu.
 *
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_unsniff_req_accepted(UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY *ce_ptr;

#ifdef ENABLE_LOGGER_LEVEL_2
    LMP_LOG_INFO(LOG_LEVEL_LOW, UNSNIFF_REQ_ACCEPTED_RECEIVED, 0,0);
#endif

    ce_ptr = &lmp_connection_entity[ce_index];

    /* Move to active mode. */
    lmp_exit_sniff_mode(ce_index);

#ifndef _DAPE_TEST_EXIT_SNIFF_EARLIER
    /* Generate mode change event. */
    hci_generate_mode_change_event(HCI_COMMAND_SUCCEEDED, ce_index,
                                   LP_ACTIVE_MODE, 0x00);
#endif

    BB_start_tpoll(ce_ptr->am_addr, ce_ptr->Tpoll, ce_ptr->phy_piconet_id);

#ifdef ENABLE_SCO
    lmp_determine_full_bandwidth();
#endif

    if(ce_ptr->low_power_disconnect_state == UNSNIFF_DISCONNECT)
    {
        ce_ptr->low_power_disconnect_state = IDLE_STATE;

        /* Kill ACL connection */
        if(lmp_handle_acl_disconnect(ce_index, ce_ptr->disconnect_reason)
                == API_FAILURE)
        {
            BT_FW_HCI_ERR(LMP_HANDLE_ACL_DISCONNECT_FAILED,0,0);
            return BT_FW_ERROR;
        }
    }

    ce_ptr->cont_poll_count = 0;

    //RT_BT_LOG(GRAY, LMP_PDU_943, 0, 0);

    return BT_FW_SUCCESS;
}

/**
 * Handles the LMP_accepted for LMP_sniff_req pdu.
 *
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_sniff_req_accepted(UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY *ce_ptr;
    ce_ptr = &lmp_connection_entity[ce_index];

    /* Enter sniff mode. */
    lmp_start_sniff_mode(ce_index);

    /* Send Mode change event to the host.*/
    hci_generate_mode_change_event(
        HCI_COMMAND_SUCCEEDED,
        ce_index,
        LP_SNIFF_MODE,
        ce_ptr->sniff_interval);

    ce_ptr->sniff_max_interval_negotiated = FALSE ;
    ce_ptr->sniff_min_interval_negotiated = FALSE ;

    return BT_FW_SUCCESS ;
}
#endif /* COMPILE_SNIFF_MODE */

/**
 * Handles the LMP_not_accepted pdu from the remote device and routes the
 * control to appropriate handlers based on the opcode rejected.
 *
 * \param lmp_pdu_ptr Pointer to the LMP PDU.
 * \param ce_index ACL connection entity index.
 *
 * \return TRUE, if the operation is successful. FALSE,
 *         otherwise.
 */
BOOLEAN lmp_handle_not_accepted_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index)
{
    UCHAR pdu_opcode ;
    UCHAR reason ;
#ifdef COMPILE_HOLD_MODE
    UCHAR trans_id ;
#endif

    pdu_opcode = lmp_pdu_ptr->payload_content[1] ;
    reason = lmp_pdu_ptr->payload_content[2] ;
#ifdef COMPILE_HOLD_MODE
    trans_id = (UCHAR)(lmp_pdu_ptr->payload_content[0] & 0x01);
#endif

    switch(pdu_opcode)
    {
        case LMP_HOST_CONNECTION_REQ_OPCODE :
            lmp_handle_host_connection_req_not_accepted(reason, ce_index);
            break;
#ifdef ENABLE_SCO
        case LMP_SCO_LINK_REQ_OPCODE :
            lmp_handle_sco_link_req_not_accepted(reason, ce_index);
            break;
#endif /* ENABLE_SCO */

#ifdef COMPILE_HOLD_MODE
        case LMP_HOLD_REQ_OPCODE :
            lmp_handle_hold_req_not_accepted(trans_id, reason, ce_index);
            break;
#endif /* COMPILE_HOLD_MODE */

#ifdef COMPILE_SNIFF_MODE
        case LMP_SNIFF_REQ_OPCODE :
            lmp_handle_sniff_req_not_accepted(reason, ce_index);
            break;

        case LMP_UNSNIFF_REQ_OPCODE :
            lmp_handle_unsniff_req_not_accepted(reason, ce_index);
            break;
#endif /* COMPILE_SNIFF_MODE */

#ifdef COMPILE_PARK_MODE
        case LMP_PARK_REQ_OPCODE :
            lmp_handle_park_req_not_accepted(reason, ce_index);
            break;
#endif /* COMPILE_PARK_MODE */

#ifndef COMPILE_SINGLE_SLOT_PACKETS_ONLY
        case LMP_MAX_SLOT_REQ_OPCODE :
            lmp_handle_max_slot_req_not_accepted(lmp_pdu_ptr, ce_index);
            break;
#endif

#ifdef COMPILE_ROLE_SWITCH
        case LMP_SWITCH_REQ_OPCODE :
            lmp_handle_switch_req_not_accepted(lmp_pdu_ptr, ce_index);
            break;
#endif
#ifdef OPTIONAL_PAGING
        case LMP_PAGE_MODE_REQ_OPCODE:
            lmp_handle_page_mode_req_not_accepted(lmp_pdu_ptr,ce_index);
            break;

        case LMP_PAGE_SCAN_MODE_REQ_OPCODE:
            lmp_handle_page_scan_mode_req_not_accepted(lmp_pdu_ptr,ce_index);
            break;
#endif /* OPTIONAL_PAGING */
        case LMP_QoS_REQ_OPCODE :
            lmp_handle_qos_req_not_accepted(reason, ce_index);
            break ;

        default:
            return FALSE;
    }
    return TRUE ;
}

#ifdef OPTIONAL_PAGING
/**
 * Handles the LMP_not_accepted for page scan mode req pdu.
 *
 * \param lmp_pdu_ptr Pointer to LMP PDU.
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_page_scan_mode_req_not_accepted(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index)
{
    HCI_EVENT_PKT *event_pkt ;
    OS_SIGNAL signal_send ;
    UCHAR reason ;

    reason = lmp_pdu_ptr->payload_content[2];

    /*Update the self device database Self device can go for optioal page scan*/
    lmp_self_device_data.opt_page_scan_flag = FALSE ;

    /*Send command complete event...*/
    if((hci_generate_command_complete_event(
                HCI_WRITE_PAGE_SCAN_MODE_OPCODE,
                reason,ce_index))
            == API_FAILURE)
    {
        return BT_FW_ERROR ;
    }

    return BT_FW_SUCCESS;
}

/**
 * Handles the LMP_not_accepted for page mode req pdu.
 *
 * \param lmp_pdu_ptr Pointer to LMP PDU.
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_page_mode_req_not_accepted(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index)
{
    return BT_FW_SUCCESS;
}

#endif /* OPTIONAL_PAGING */

/**
 * Handles LMP_not_accepted for LMP_qos_req pdu.
 *
 * \param reason Reason for rejection.
 * \param ce_index ACL connection entity.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_qos_req_not_accepted(UCHAR reason, UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

#if 0
    /* Check for correct state */
    switch(ce_ptr->ce_status)
    {
        default :
            return BT_FW_ERROR ;
    }
#endif

    /* Send QOS setup complete event.*/
    if(ce_ptr->qos_tn_details == QOS_TN_QOS_SETUP)
    {
        hci_generate_QoS_setup_complete_event(
            reason,
            ce_index);
    }
    else if( (ce_ptr->qos_tn_details == QOS_TN_FLOW_SPEC_TX_SIDE) ||
             (ce_ptr->qos_tn_details == QOS_TN_FLOW_SPEC_RX_SIDE) )
    {
        hci_generate_flow_spec_complete_event(reason,
                                              ce_ptr->qos_tn_details, ce_index);
    }

    ce_ptr->qos_tn_details = QOS_TN_INVALID;

    return BT_FW_SUCCESS ;
}

#ifdef COMPILE_SNIFF_MODE
/**
 * Handles LMP_not_accepted for LMP_sniff_req pdu.
 *
 * \param reason Reason for rejection.
 * \param ce_index ACL connection entity.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_sniff_req_not_accepted(UCHAR reason, UINT16 ce_index)
{

    switch(lmp_connection_entity[ce_index].ce_status)
    {
        case LMP_SNIFF_MODE_NEG :
        case LMP_SNIFF_MODE :
            /*Send hold mode change event */
            hci_generate_mode_change_event(
                reason,
                ce_index,
                LP_ACTIVE_MODE,
                0X00);

            lmp_set_ce_status(ce_index, LMP_CONNECTED);

            break;
        default :
            break;
    }

#ifndef _CCH_SLOT_OFFSET_
    lmp_put_dsniff(lmp_connection_entity[ce_index].sniff_slot_offset);
#endif

#ifdef _CCH_SLOT_OFFSET_			
    lmp_put_global_slot_offset(ce_index, 0);
#endif

    return BT_FW_SUCCESS ;
}

/**
 * Handles LMP_not_accepted for LMP_unsniff_req pdu.
 *
 * \param reason Reason for rejection.
 * \param ce_index ACL connection entity.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_unsniff_req_not_accepted(UCHAR reason, UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    if(ce_ptr->low_power_disconnect_state != UNSNIFF_DISCONNECT)
    {
        /* Send Mode change event */
        hci_generate_mode_change_event(reason, ce_index, LP_ACTIVE_MODE, 0);
    }

    lmp_set_ce_status(ce_index, LMP_CONNECTED);

    /* Proceed with disconnection if unsniff was issued as part of
       disconnection. */
    if(ce_ptr->low_power_disconnect_state == UNSNIFF_DISCONNECT)
    {
        ce_ptr->low_power_disconnect_state = IDLE_STATE;
        /* Kill ACL connection */
        if(lmp_handle_acl_disconnect(ce_index, ce_ptr->disconnect_reason)
                == API_FAILURE)
        {
            BT_FW_HCI_ERR(LMP_HANDLE_ACL_DISCONNECT_FAILED,0,0);
            return BT_FW_ERROR;
        }
    }

    return BT_FW_SUCCESS;
}
#endif /* COMPILE_SNIFF_MODE */

#ifdef COMPILE_ROLE_SWITCH
/**
 * Handles the LMP_not_accepted for LMP_switch_req pdu.
 *
 * \param lmp_pdu_ptr Pointer to LMP PDU.
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR/A valid
 *         bluetooth error code, otherwise.
 */

UCHAR lmp_handle_switch_req_not_accepted(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index)
{
    UCHAR reason;
    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];
    reason = lmp_pdu_ptr->payload_content[2];

    /**PATCH_PENDING_ENCRY_WHILE_ROLE_SWITCH*/
#ifdef _PENDING_ENCRY_WHILE_ROLE_SWITCH
    if(g_need_pend_encry)
    {
        g_need_pend_encry = FALSE;
        
        if (g_encry_pending == TRUE)
        {
            LMP_PDU_PKT* pdu_pkt;
            pdu_pkt = (LMP_PDU_PKT *)g_signal_ptr_encry.param;
            pduq_queue_pdu(pdu_pkt, pdu_pkt ->piconet,  pdu_pkt->ce_index);
            //encry mode 
            lc_handle_and_check_sw_scheduler((UCHAR)((UINT32)(g_signal_ptr_encry.ext_param)));  
            g_encry_pending = FALSE;
        }
    }
#endif
    lmp_handle_role_switch_failure(ce_index, reason);

    /* Clear cont_poll_count. Though this is set only as master,
    to optimise, we need not check for role here while clearing
    the flag. */
    ce_ptr->cont_poll_count = 0;

    return BT_FW_SUCCESS;
}

#endif /* COMPILE_ROLE_SWITCH */

#ifdef COMPILE_PARK_MODE
/**
 * Handles LMP_not_accepted for LMP_park_req pdu.
 *
 * \param reason Reason for rejection.
 * \param ce_index ACL connection entity.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_park_req_not_accepted(UCHAR reason, UINT16 ce_index)
{
    UCHAR parameter_list[7];
    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    ce_ptr->supto_auto_repark = FALSE;

#ifdef COMPILE_AFH_HOP_KERNEL
#ifdef AFH_PARK
    if ( (ce_ptr->afh_disabled_for_park == TRUE) &&
            (ce_ptr->remote_dev_role == SLAVE) )
    {
        ce_ptr->afh_disabled_for_park = FALSE;
        lmp_update_map(AFH_ENABLE, ce_index, TRUE);
    }
#endif
#endif

    if((ce_ptr->ce_status == LMP_PARK_MODE_REQ) ||
            (ce_ptr->ce_status == LMP_PARK_MODE))
    {
        lmp_set_ce_status(ce_index, LMP_CONNECTED);

        /*
        * Release the PM address which is assigned during connection.
        */
        lmp_put_pm_addr(ce_ptr->pm_addr, ce_ptr->phy_piconet_id);
        lmp_put_ar_addr(ce_ptr->ar_addr, ce_ptr->phy_piconet_id);

        /* Send hold mode change event */
        hci_generate_mode_change_event(
            reason,
            ce_index,
            LP_ACTIVE_MODE,
            0X00);

        if(ce_ptr->remote_dev_role == SLAVE)
        {
            if (ce_ptr->stored_link_supervision_timeout != 0)
            {
                parameter_list[0] = LMP_SUPERVISION_TIMEOUT_OPCODE;
                parameter_list[2] = LSB(ce_ptr->stored_link_supervision_timeout);
                parameter_list[3] = MSB(ce_ptr->stored_link_supervision_timeout);
                lmp_generate_pdu(ce_index, parameter_list,
                                 LMP_SUPERVISION_TIMEOUT_LEN, MASTER_TID,
                                 LMP_NO_STATE_CHANGE);

                LMP_INF(SETTING_SUPERVISION_TO_FOR_SLAVE_TO,1,ce_ptr->stored_link_supervision_timeout);

                /* Restore and start the Sup timer after coming out of park */
                ce_ptr->link_supervision_timeout =
                    ce_ptr->stored_link_supervision_timeout;
                ce_ptr->stored_link_supervision_timeout = 0;

                OS_START_TIMER(
                    ce_ptr->supervision_timeout_handle,
                    (UINT16)(SLOT_VAL_TO_TIMER_VAL(
                                 LMP_SUPERVISION_TIMER_RESOLUTION)));
            }
        }
    }
    return BT_FW_SUCCESS ;
}
#endif /* COMPILE_PARK_MODE */

#ifndef COMPILE_SINGLE_SLOT_PACKETS_ONLY
/**
 * Handles the LMP_not_accepted for LMP_max_slot_req pdu.
 *
 * \param lmp_pdu_ptr Pointer to LMP PDU.
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_max_slot_req_not_accepted(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index)
{
    UCHAR reason ;
    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];
    reason = lmp_pdu_ptr->payload_content[2] ;

    if (ce_ptr->last_max_slot_req_sent == LMP_LAST_MAX_SLOT_REQ_SENT_INVALID)
    {
        /* This case is not possible. */
    }

    if (ce_ptr->last_max_slot_req_sent == LMP_LAST_MAX_SLOT_REQ_SENT_5_SLOT)
    {
        /* 5 slot packet negotiation failed. Now, try for a 3 slot packet */

        lmp_send_max_slot_req_pdu(ce_index, 3);
        ce_ptr->last_max_slot_req_sent = LMP_LAST_MAX_SLOT_REQ_SENT_3_SLOT;
    }
    else if (ce_ptr->last_max_slot_req_sent == LMP_LAST_MAX_SLOT_REQ_SENT_3_SLOT)
    {
        /* 3 slot packet negotiation failed. Allow only single slot packets. */
        //{{ added by guochunxia
        ce_ptr->last_accepted_max_slot = LMP_LAST_ACCEPTED_MAX_SLOT_1;
        //}}
        ce_ptr->last_max_slot_req_sent = LMP_LAST_MAX_SLOT_REQ_SENT_INVALID;
        lc_update_pkts_allowed(ce_index);
    }

    /* Send connection packet type changed event */
    if (ce_ptr->hci_cmd_bits & CHANGE_CONN_PKT_TYPE_BIT_MASK)
    {
        ce_ptr->hci_cmd_bits &= (~CHANGE_CONN_PKT_TYPE_BIT_MASK);

        hci_generate_connection_packet_type_changed_event(
            reason,
            ce_ptr->connection_type.connection_handle,
            ce_ptr->connection_type.packet_type);
    }

    return BT_FW_SUCCESS;
}
#endif /* COMPILE_SINGLE_SLOT_PACKETS_ONLY */

#ifdef COMPILE_HOLD_MODE
/**
 * Handles LMP_not_accepted for LMP_hold_req pdu.
 *
 * \param trans_id Transaction ID.
 * \param reason Reason for rejection.
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_hold_req_not_accepted(UCHAR trans_id, UCHAR reason,
                                       UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    if(trans_id != ce_ptr->remote_dev_role)
    {
        hci_generate_mode_change_event(reason, ce_index, LP_ACTIVE_MODE, 0);
    }

    /*Reset the hold interval values */
    ce_ptr->hold_mode_max_interval = otp_str_data.bt_hold_max_interval;
    ce_ptr->hold_mode_min_interval = otp_str_data.bt_hold_min_interval;
    ce_ptr->hold_max_interval_negotiated = 0x00;
    ce_ptr->hold_min_interval_negotiated = 0x00;

    /* Set the LMP status back to CONNECTED. */
    lmp_set_ce_status(ce_index, LMP_CONNECTED);

#ifdef COMPILE_NESTED_PAUSE_RESUME
    aclq_resume_am_addr(ce_ptr->am_addr,
                        ce_ptr->phy_piconet_id, ACL_PAUSED_HOLD);
#else /* COMPILE_NESTED_PAUSE_RESUME */
    aclq_resume_am_addr(ce_ptr->am_addr,
                        ce_ptr->phy_piconet_id);
#endif /* COMPILE_NESTED_PAUSE_RESUME */

    return BT_FW_SUCCESS ;
}

#endif /* COMPILE_HOLD_MODE */

/**
 * Handles LMP_not_accepted for LMP_host_connection_req pdu.
 *
 * \param reason Reason for rejection.
 * \param ce_index ACL connection entity.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_host_connection_req_not_accepted(UCHAR reason, UINT16 ce_index)
{
    LMP_INF(HOST_CONN_REQ_NOT_ACCEPTED_RESAOSN,1,reason);

    lmp_handle_acl_disconnect(ce_index, reason);
    return BT_FW_SUCCESS ;
}

/**
 * Queues either lmp_features_req or lmp_features_resp PDU.
 *
 * \param ce_index ACL connection entity.
 * \param opcode The opcode of the PDU.
 * \param tid The transaction ID of the PDU.
 *
 * \return None.
 */
void lmp_send_features_req_or_res_pdu(UINT16 ce_index, UCHAR opcode,
                                      LMP_TRAN_ID tid)
{
    UCHAR parameter_list[LMP_FEATURES_REQ_LEN];

    BZ_ASSERT(opcode == LMP_FEATURES_REQ_OPCODE
              || opcode == LMP_FEATURES_RES_OPCODE,
              "I can't send any PDU other than LMP_feature_req/res");
    parameter_list[0] = opcode;

    memcpy(&parameter_list[2], &lmp_feature_data.feat_page0[0],
           BTC_FEATURES_LEN);

    lmp_generate_pdu(ce_index, parameter_list, LMP_FEATURES_REQ_LEN, tid,
                     LMP_NO_STATE_CHANGE);

    return;
}

/**
 * Handles the LMP_features_req pdu from the remote device.
 *
 * \param lmp_pdu_ptr Pointer to LMP PDU.
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_features_request_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index)
{
#ifndef _DAPE_TEST_NO_RESPONSE_TO_FEATURE_REQ
    /* Store the remote features */
    memcpy(&lmp_connection_entity[ce_index].feat_page0[0],
           &lmp_pdu_ptr->payload_content[1], BTC_FEATURES_LEN);
    /* Set Feature response */
    lmp_send_features_req_or_res_pdu(ce_index, LMP_FEATURES_RES_OPCODE,
                                     REMOTE_DEV_TID);
#endif
    return BT_FW_SUCCESS;
}

/**
 * Continues either with the RNR procedure or ACL connection establishment
 * procedure whichever was paused previously to get remote features.
 *
 * \param ce_index ACL Connection entity index.
 *
 * \return None.
 */
void lmp_continue_conn_proc_after_feature_exchange(UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    UCHAR param_list[LMP_NAME_REQ_LEN];

    ce_ptr = &lmp_connection_entity[ce_index];
    BZ_ASSERT(ce_ptr->ce_status == LMP_BB_HL_CONNECTED,
              "We must be in LMP_BB_HL_CONNECTED");
    if (ce_ptr->hci_cmd_bits & REMOTE_NAME_REQ_CON_BIT_MASK)
    {
        /* Did we make the connection for doing Remote_Name_Request?
         */
        if (ce_ptr->hci_cmd_bits & REMOTE_EX_FEA_BIT_MASK)
        {
            /* The BB level connection was initiated by RNR and the extended
             * host features page was read. So generate the
             * Remote_Host_Supported_Features_Notification (RHSFN) event.
             */
            hci_generate_rhsfn_event(ce_ptr->bd_addr, &ce_ptr->features[1][0]);
        }
        
        /* Then initiate RNR.. */
        param_list[0] = LMP_NAME_REQ_OPCODE;
        param_list[2] = 0;
        ce_ptr->name_req_name_offset = 0;
#ifdef _REMOTE_NAME_RES_WRONG
        ce_ptr->name_length_offset_zero = 0;
#endif

        lmp_generate_pdu(ce_index, param_list, LMP_NAME_REQ_LEN, SELF_DEV_TID,
                         LMP_NO_STATE_CHANGE);
        ce_ptr->lmp_expected_pdu_opcode |= lmp_get_opcode_mask(
                                               LMP_NAME_RES_OPCODE, 0x0);
    }
    else if (ce_ptr->remote_dev_role == SLAVE)
    {
        if (!(g_efuse_lps_setting_3.iot_ralink_send_fea_req_later &&
              ce_ptr->got_host_conn_req ))
        {
            /* Its a Create_Connection procedure and we are master */
            param_list[0] = LMP_HOST_CONNECTION_REQ_OPCODE;
            lmp_generate_pdu(ce_index, param_list, LMP_HOST_CONNECTION_REQ_LEN,
                             MASTER_TID, LMP_NO_STATE_CHANGE);
        }
    }
    else
    {
        /* We are slave here */
        /* There is a possiblity that while we are performing feature
         * request transactions, the remote Master might send
         * LMP_host_connection_req PDU. We should not entertain it until
         * completing the feature req transaction because unless otherwise all
         * the feature bits are proper, the authentication module will yield
         * unexpected results. To avoid this problem, the
         * LMP_host_connection_req pdu should be held until feature req
         * transaction completion and then send LMP_accepted for it. Probably
         * we can called lmp_handle_host_connection_request_pdu() from here if
         * it was already received.
         */
        if (!(g_efuse_lps_setting_3.iot_ralink_send_fea_req_later &&
		ce_ptr->got_remote_ext_feature))
        {
        if (ce_ptr->host_con_req_rx_flag == TRUE)
        {
            LMP_PDU_PKT lmp_pdu_pkt;
            /* Reset the feature request related bits that we have affected */
            ce_ptr->hci_cmd_bits &=
                (~(REMOTE_SUP_FEA_BIT_MASK|REMOTE_EX_FEA_BIT_MASK));
            /* Host connection req is pending, so create the pdu
             * and process it.
             */
            lmp_pdu_pkt.payload_content[0] = LMP_HOST_CONNECTION_REQ_OPCODE<<1;
            lmp_pdu_pkt.pdu_length = (UCHAR)(LMP_HOST_CONNECTION_REQ_LEN - 1);
            lmp_handle_host_connection_request_pdu(&lmp_pdu_pkt, ce_index);
            return;
        }
    }
    }

    /* Reset the feature request related bits that we have affected */
    ce_ptr->hci_cmd_bits &= (~(REMOTE_SUP_FEA_BIT_MASK|REMOTE_EX_FEA_BIT_MASK));
}

/**
 * Handles the LMP_features_res pdu from the remote device and takes
 * appropriate action.
 *
 * \param lmp_pdu_ptr Pointer to LMP PDU.
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_features_response_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
                                       UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    UCHAR local_device_feature_bit8;
    
    ce_ptr = &lmp_connection_entity[ce_index];

    if (ce_ptr->hci_cmd_bits & REMOTE_SUP_FEA_TRUE_HOST_BIT_MASK)
    {
        ce_ptr->hci_cmd_bits |= REMOTE_SUP_FEA_BIT_MASK;
    }

    if ((ce_ptr->hci_cmd_bits & REMOTE_SUP_FEA_BIT_MASK) == FALSE)
    {
        /* We don't expect this PDU now */
        lmp_send_lmp_not_accepted(ce_index, LMP_FEATURES_RES_OPCODE,
                                  (UCHAR)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr),
                                  PDU_NOT_ALLOWED_ERROR);
        return BT_FW_SUCCESS;
    }

    /* Store the features */
    memcpy(&ce_ptr->feat_page0[0], &lmp_pdu_ptr->payload_content[1],
           BTC_FEATURES_LEN);

    local_device_feature_bit8 = lmp_feature_data.feat_page0[7];

    if (g_efuse_lps_setting_3.iot_ralink_send_fea_req_later)
    {
        if (ce_ptr->got_remote_feature == 0)
        {
            ce_ptr->got_remote_feature = 1;
			
            if ( (ce_ptr->feat_page0[7] & EXTENDED_FEATURE) &&
                    (local_device_feature_bit8 & EXTENDED_FEATURE) )
            {
                ce_ptr->hci_cmd_bits |= REMOTE_EX_FEA_BIT_MASK;
                ce_ptr->requested_ext_page = 1;

#ifdef _SUPPORT_EXT_FEATURES_PAGE_2_
                ce_ptr->requested_ext_page_bm = 0x06; /* set page 1 and page 2 */
#endif
                
                /* We need to get the page-1 also. It is required for determining
                 * whether the remote device supports SSP or not.
                 */
    
#ifdef COMPILE_FEATURE_REQ_EXT
                lmp_send_features_req_or_res_ext_pdu(ce_index, 1,
                                                     LMP_FEATURES_REQ_EXT_OPCODE, SELF_DEV_TID);
                ce_ptr->lmp_expected_pdu_opcode |= lmp_get_opcode_mask(
                                                       LMP_ESCAPE4_OPCODE, LMP_FEATURES_RES_EXT_OPCODE);
#endif
    
            }
            else
            {
                /* We don't need to fetch extended features page... so lets
                 * proceed with connection.
                 */
                lmp_continue_conn_proc_after_feature_exchange(ce_index);
            }
        }	
        
        if (ce_ptr->hci_cmd_bits & REMOTE_SUP_FEA_TRUE_HOST_BIT_MASK)
        {
            hci_generate_remote_supported_features_complete_event(
                HCI_COMMAND_SUCCEEDED,
                ce_ptr->connection_type.connection_handle,
                ce_ptr->feat_page0);
            ce_ptr->hci_cmd_bits &= (~REMOTE_SUP_FEA_BIT_MASK);
            ce_ptr->hci_cmd_bits &= ~REMOTE_SUP_FEA_TRUE_HOST_BIT_MASK;
        }
    }
    else
    {
        if (ce_ptr->ce_status == LMP_BB_HL_CONNECTED)
        {
            if ( (ce_ptr->feat_page0[7] & EXTENDED_FEATURE) &&
                (local_device_feature_bit8 & EXTENDED_FEATURE) )
            {
                ce_ptr->hci_cmd_bits |= REMOTE_EX_FEA_BIT_MASK;
                ce_ptr->requested_ext_page = 1;

#ifdef _SUPPORT_EXT_FEATURES_PAGE_2_
                ce_ptr->requested_ext_page_bm = 0x06; /* set page 1 and page 2 */
#endif

            /* We need to get the page-1 also. It is required for determining
             * whether the remote device supports SSP or not.
             */

#ifdef COMPILE_FEATURE_REQ_EXT
            lmp_send_features_req_or_res_ext_pdu(ce_index, 1,
                                                 LMP_FEATURES_REQ_EXT_OPCODE, SELF_DEV_TID);
            ce_ptr->lmp_expected_pdu_opcode |= lmp_get_opcode_mask(
                                                   LMP_ESCAPE4_OPCODE, LMP_FEATURES_RES_EXT_OPCODE);
#endif

        }
        else
        {
            /* We don't need to fetch extended features page... so lets
             * proceed with connection.
             */
            lmp_continue_conn_proc_after_feature_exchange(ce_index);
        }
    }	
       
        if (ce_ptr->hci_cmd_bits & REMOTE_SUP_FEA_TRUE_HOST_BIT_MASK)
    {
        hci_generate_remote_supported_features_complete_event(
            HCI_COMMAND_SUCCEEDED,
            ce_ptr->connection_type.connection_handle,
            ce_ptr->feat_page0);
        ce_ptr->hci_cmd_bits &= (~REMOTE_SUP_FEA_BIT_MASK);
            ce_ptr->hci_cmd_bits &= ~REMOTE_SUP_FEA_TRUE_HOST_BIT_MASK;
        }       
    }
    return BT_FW_SUCCESS;
}

/**
 * Handles the LMP_name_req pdu from the remote device.
 *
 * \param lmp_pdu_ptr Pointer to LMP PDU.
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_name_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index)
{
    UCHAR parameter_list[LMP_NAME_RES_LEN];
    UCHAR name_req_name_offset;

    name_req_name_offset = lmp_pdu_ptr->payload_content[1];

    if (name_req_name_offset > lmp_self_device_data.local_name_len)
    {
        lmp_send_lmp_not_accepted(ce_index, LMP_NAME_REQ_OPCODE,
                                  REMOTE_DEV_TID, INVALID_LMP_PARAMETERS_ERROR);
        return BT_FW_SUCCESS;
    }

    parameter_list[0]  = LMP_NAME_RES_OPCODE;
    parameter_list[2]  = name_req_name_offset;   /* Name offset */
    parameter_list[3]  = lmp_self_device_data.local_name_len; /*Name Length*/

    memcpy(&parameter_list[4],
           &lmp_self_device_data.local_name[name_req_name_offset], 14);

    lmp_generate_pdu(ce_index, parameter_list, LMP_NAME_RES_LEN,
                     REMOTE_DEV_TID, LMP_NO_STATE_CHANGE);

    return BT_FW_SUCCESS;
}

/**
 * Handles the LMP_name_res pdu from the remote device.
 *
 * \param lmp_pdu_ptr Pointer to LMP PDU.
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_name_resp_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index)
{
    UCHAR parameter_list[LMP_NAME_REQ_LEN];
    UCHAR name_length;
    UCHAR name_offset;
    UCHAR index;
    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    if (!(ce_ptr->hci_cmd_bits
            & (REMOTE_NAME_REQ_BIT_MASK|REMOTE_NAME_REQ_CON_BIT_MASK)))
    {
        /* We are not expecting this PDU */
        lmp_send_lmp_not_accepted(ce_index, LMP_NAME_RES_OPCODE,
                                  (UCHAR)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr),
                                  PDU_NOT_ALLOWED_ERROR);
        return BT_FW_ERROR;
    }

    /* Ok. We have initiated the transaction and let's proceed further */
    name_offset = lmp_pdu_ptr->payload_content[1];
    name_length = lmp_pdu_ptr->payload_content[2];

#ifdef _REMOTE_NAME_RES_WRONG
    if(name_offset == 0)
    {
        ce_ptr->name_length_offset_zero = name_length;
    }
    if(((name_offset != 0) && (ce_ptr->name_length_offset_zero != name_length)) || \
        (ce_ptr->name_req_name_offset != name_offset))
#else
    if (ce_ptr->name_req_name_offset != name_offset)
#endif
    {
        lmp_send_lmp_not_accepted(ce_index, LMP_NAME_RES_OPCODE, SELF_DEV_TID,
                                  UNSUPPORTED_PARAMETER_VALUE_ERROR);

        if (ce_ptr->hci_cmd_bits & REMOTE_NAME_REQ_BIT_MASK)
        {
            /* Name Request was issued on an existing ACL link */
            hci_generate_remote_name_request_complete_event(
                UNSUPPORTED_PARAMETER_VALUE_ERROR, ce_index);
            ce_ptr->hci_cmd_bits &= (~REMOTE_NAME_REQ_BIT_MASK);
        }
        else if (ce_ptr->hci_cmd_bits & REMOTE_NAME_REQ_CON_BIT_MASK)
        {
            /* Name Request was issued without an ACL link */
            lmp_handle_acl_disconnect(ce_index,
                                      UNSUPPORTED_PARAMETER_VALUE_ERROR);
        }

        return BT_FW_ERROR;
    }

    index = ce_ptr->name_req_name_offset;

    if ((name_offset + 14) >= name_length)  /* Are we done with name_req */
    {
        memcpy(&ce_ptr->device_name[index], &lmp_pdu_ptr->payload_content[3],
               (name_length-index));

        /* Send remote name request complete event. */
        if (ce_ptr->hci_cmd_bits & (REMOTE_NAME_REQ_BIT_MASK))
        {
            ce_ptr->hci_cmd_bits &= ~(REMOTE_NAME_REQ_BIT_MASK);
            hci_generate_remote_name_request_complete_event(
                HCI_COMMAND_SUCCEEDED, ce_index);
        }
        else
        {
            /* Successfully completed RNR */
            lmp_handle_acl_disconnect(ce_index,
                                      CONNECTION_TERMINATED_USER_ERROR);
            ce_ptr->disconnect_reason = HCI_COMMAND_SUCCEEDED;
        }
    }
    else
    {
        /* We are not yet done with the Name_Request... We need to get some
         * more fragments.
         */
        /* Copy the receieved name into the connection entity database */
        memcpy(&ce_ptr->device_name[index],
               &lmp_pdu_ptr->payload_content[3], 14);
        ce_ptr->name_req_name_offset += 14;

        parameter_list[0] = LMP_NAME_REQ_OPCODE;
        parameter_list[2] = ce_ptr->name_req_name_offset;
        lmp_generate_pdu(ce_index, parameter_list, LMP_NAME_REQ_LEN,
                         SELF_DEV_TID, LMP_NO_STATE_CHANGE);
        ce_ptr->lmp_expected_pdu_opcode |= lmp_get_opcode_mask(
                                               LMP_NAME_RES_OPCODE, 0x0);
    }

    return BT_FW_SUCCESS;
}

#ifndef COMPILE_SINGLE_SLOT_PACKETS_ONLY
/**
 * Handles the LMP_max_slot_req pdu from the remote device.
 *
 * \param lmp_pdu_ptr Pointer to LMP PDU.
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_max_slot_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index)
{
    UINT32 max_slot_req_recd;
    UINT32 max_slot_that_can_be_sent;

    max_slot_req_recd = lmp_pdu_ptr->payload_content[1];

    max_slot_that_can_be_sent = lmp_calculate_max_slot(ce_index);
#ifndef _REDUCE_LPS_AFTER_RTL8703B_
#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8723_RCP_
    UINT8 return_status;
    if (rcp_lmp_handle_max_slot_req_pdu != NULL)
    {
        if ( rcp_lmp_handle_max_slot_req_pdu((void *)&return_status, lmp_pdu_ptr, ce_index, &max_slot_that_can_be_sent, &max_slot_req_recd) )
        {
            return return_status;
        }
    }    
#endif	
#endif
#endif
    if (max_slot_that_can_be_sent != 0xFF)
    {
        /*
         * If max-slot that we can send now, is lesser than the recd
         * one, reject. Otherwise accept.
         */
        if (max_slot_that_can_be_sent >= max_slot_req_recd)
        {
            /* Accept. */
            lmp_connection_entity[ce_index].last_max_slot_sent =
                max_slot_req_recd;
			/*update last accept max slot too*/
			lmp_connection_entity[ce_index].last_accepted_max_slot =
                max_slot_req_recd;
            /* Note: Remote dev TiD is hard-coded here.
              We assume that remote device will not mis-behave here.*/
            lmp_send_lmp_accepted(ce_index, LMP_MAX_SLOT_REQ_OPCODE,
                                  REMOTE_DEV_TID, LMP_NO_STATE_CHANGE);
        }
        else
        {

#ifdef _CCH_WHQL_SCO_1_5_13_3_

#ifdef _CCH_RTL8723A_B_CUT
            if( g_efuse_lps_setting_4.whql_test_2sco )
            {				
                lmp_send_lmp_accepted(ce_index, LMP_MAX_SLOT_REQ_OPCODE,
                                  REMOTE_DEV_TID, (LMP_CE_CONN_STATES)LMP_NO_STATE_CHANGE);
            }else
            {
                lmp_send_lmp_not_accepted(ce_index, LMP_MAX_SLOT_REQ_OPCODE,
                                      REMOTE_DEV_TID, UNSPECIFIED_ERROR);
            }		
#else
            lmp_send_lmp_not_accepted(ce_index, LMP_MAX_SLOT_REQ_OPCODE,
                                      REMOTE_DEV_TID, UNSPECIFIED_ERROR);
#endif
	


#else		
            /* Not-accepted. */
            lmp_send_lmp_not_accepted(ce_index, LMP_MAX_SLOT_REQ_OPCODE,
                                      REMOTE_DEV_TID, UNSUPPORTED_PARAMETER_VALUE_ERROR);

#endif
        }
    }

    return BT_FW_SUCCESS;
}

/**
 * Handles the LMP_max_slot pdu from the remote device.
 *
 * \param lmp_pdu_ptr Pointer to LMP PDU.
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_max_slot_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index)
{
    UCHAR max_slot;

    max_slot = lmp_pdu_ptr->payload_content[1];

    if((max_slot < 1) || (max_slot > 5) || ((max_slot & 0x1) == 0))
    {
        LMP_ERR(INVALID_MAX_SLOT_PDU_RECD_JUST_DROP_THIS_PDU,0,0);
        return BT_FW_ERROR;
    }

    lmp_connection_entity[ce_index].last_recd_max_slot = max_slot;

    lc_update_pkts_allowed(ce_index);

    hci_generate_max_slots_change_event(
        lmp_connection_entity[ce_index].connection_type.connection_handle, max_slot);

#ifdef COMPILE_CQDDR
    if (!IS_USE_FOR_MUTE)
    {
        lmp_calculate_and_send_preferred_rate_pdu(ce_index);
    }
#endif

    return BT_FW_SUCCESS ;
}
#endif

/**
 * Handles the LMP_slot_offset pdu from the remote device.
 *
 * \param lmp_pdu_ptr Pointer to LMP PDU.
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_slot_offset_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index)
{
    UINT16 slot_offset ;
    UCHAR bd_addr[LMP_BD_ADDR_SIZE];

    /**PATCH_PENDING_ENCRY_WHILE_ROLE_SWITCH*/
    //need to role swtich, pendig encry
#ifdef _PENDING_ENCRY_WHILE_ROLE_SWITCH
    g_need_pend_encry = TRUE;
#endif
#ifdef _STOP_AFH_TIMER_WHILE_ROLE_SWITCH
    LMP_CONNECTION_ENTITY *ce_ptr;
    ce_ptr = &lmp_connection_entity[ce_index];
    if (ce_ptr->afh_instant_timer_handle != OS_INVALID_HANDLE)
    {
        g_stop_afh_timer = TRUE;
    }
#endif
    /*
     * If the Remote device is Slave and sent the slot offset.
     * Update in its connection entity. And Wait for a Switch Req
     * from the remote device.
     */
    if (lmp_connection_entity[ce_index].remote_dev_role == SLAVE)
    {
        slot_offset = lmp_pdu_ptr->payload_content[1] |
                    (lmp_pdu_ptr->payload_content[2] << 8);
        memcpy(bd_addr, &lmp_pdu_ptr->payload_content[3], LMP_BD_ADDR_SIZE);
        LMP_INF(SLOT_OFFSET_RECD,1,slot_offset);

        /* Slot offset should be within the range of 0-1250 */
        if( slot_offset > 1250 )
        {
            slot_offset %= 1250 ;
        }

        LMP_INF(SLOT_OFFSET_RECEIVED_FROM_THE_REMOTE_DEVICE,1,slot_offset);

        lmp_connection_entity[ce_index].slot_offset = slot_offset ;
        //lmp_connection_entity[ce_index].lmp_expected_pdu_opcode |= lmp_get_opcode_mask(
        //            LMP_SWITCH_REQ_OPCODE, 0x0);
    }
    return BT_FW_SUCCESS ;
}
#ifdef COMPILE_ROLE_SWITCH
/**
 * Handles the LMP_switch_req pdu from the remote device.
 *
 * \param lmp_pdu_ptr Pointer to LMP PDU.
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_switch_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index)
{
    UINT32 switch_instant;
    UINT32 native_clock;
    UCHAR handle_pdu = TRUE;
    LMP_CONNECTION_ENTITY* ce_ptr;
    UCHAR instant_status;
    UINT8 lcl_ce_status;

    ce_ptr = &lmp_connection_entity[ce_index];
    /**PATCH_PENDING_ENCRY_WHILE_ROLE_SWITCH*/
#ifdef _PENDING_ENCRY_WHILE_ROLE_SWITCH
    //need to role swtich, pendig encry
    g_need_pend_encry = TRUE;
#endif
#ifdef _STOP_AFH_TIMER_WHILE_ROLE_SWITCH
    ce_ptr = &lmp_connection_entity[ce_index];
    if (ce_ptr->afh_instant_timer_handle != OS_INVALID_HANDLE)
    {
        g_stop_afh_timer = TRUE;
    }
#endif

/*we don't need to check here**/
#ifndef _STOP_AFH_TIMER_WHILE_ROLE_SWITCH
#ifdef COMPILE_AFH_HOP_KERNEL
    /* Check if the AFH timer is running. */
    if(ce_ptr->afh_instant_timer_handle != NULL)
    {
        LMP_PDU_PKT *lmp_pkt;

        if (ce_ptr->mss_pdu_ptr == NULL)
        {
            OS_ALLOC_BUFFER(lmp_pdu_buffer_pool_handle,(void **)(&lmp_pkt));
        }
        else
        {
            lmp_pkt = ce_ptr->mss_pdu_ptr;
        }

        LMP_LOG_INFO(LOG_LEVEL_HIGH, AFH_MSS_LMP_MSS_PAUSED_AS_AFH_IS_IN_PROGRESS,0,0);

        memcpy(lmp_pkt, lmp_pdu_ptr, sizeof(LMP_PDU_PKT) );

        /* Store the HCI-cmd-ptr in LMP-CE and return.
            We will resume processing this after AFH-instant. */
        ce_ptr->mss_pdu_ptr = lmp_pkt;

        /* Set flag to resume HCI-cmd after AFH-instant. */
        ce_ptr->mss_pdu_pending = TRUE;

        return BT_FW_SUCCESS;
    }
#endif
#endif
    /* Check if MSS is already in progress, on any connection,
       need not be on the same connection. */
    if(lmp_mss_state != LMP_MSS_INIT)
    {
        RT_BT_LOG(GRAY, LMP_PDU_2061, 1, ce_index);

        handle_pdu = FALSE;
    }

    lcl_ce_status = ce_ptr->ce_status;

    switch (lcl_ce_status)
    {
            /* During connection role switch */
        case LMP_BB_HL_CONNECTED:           /* Fall through. */
            /* After connection role switch */
        case LMP_CONNECTED:
            break;

        default:
            handle_pdu = FALSE;
    }

    if( lmp_check_for_role_switch_in_scatternet(ce_index) == FALSE)
    {
        handle_pdu = FALSE;
    }
#ifdef _TMP_CHK_ROLE_SW
        RT_BT_LOG(RED, DAPE_TEST_LOG522, 2,
        bz_auth_is_link_encrypted(ce_index),
        bz_auth_is_master_link_key_in_use(ce_index));
#endif    
#ifndef _ALLOW_ROLE_SW_WHEN_SLAVE_AND_SCO
    if (
        bz_auth_is_link_encrypted(ce_index)
        || bz_auth_is_master_link_key_in_use(ce_index)
#ifdef ENABLE_SCO
        || lmp_self_device_data.total_no_of_sco_conn
#endif
#ifdef COMPILE_ESCO
        || lmp_self_device_data.number_of_esco_connections
#endif
    )
#else
    if (
        bz_auth_is_link_encrypted(ce_index)
        || bz_auth_is_master_link_key_in_use(ce_index)
#ifdef ENABLE_SCO
        || (lmp_self_device_data.total_no_of_sco_conn && ce_ptr->remote_dev_role == SLAVE)
#endif
#ifdef COMPILE_ESCO
        || (lmp_self_device_data.number_of_esco_connections && ce_ptr->remote_dev_role == SLAVE)
#endif
    )

#endif
    {
        handle_pdu = FALSE;
    }
/*we don't need to check here**/
#ifndef _STOP_AFH_TIMER_WHILE_ROLE_SWITCH
#ifdef _DAPE_NO_ROLE_SW_WHEN_AFH_TIMER_RUNNING
    if(ce_ptr->afh_instant_timer_handle != NULL)
    {
        handle_pdu = FALSE;
    }
#endif
#endif
#ifdef _DAPE_NO_ROLE_SW_WHEN_ROLE_SW
    if (g_role_switch_status != 0)
    {
        handle_pdu = FALSE;
    }
#endif
#ifdef _SUPPORT_VER_4_1_
#ifdef _SUPPORT_PCA_ADJUST
    if (IS_BT41)
    {
        if (bt_pca_manager.pca_updating)
        {
            handle_pdu = FALSE;
        }    
    }
#endif
#endif

    if (handle_pdu == FALSE)
    {
#ifdef _STOP_AFH_TIMER_WHILE_ROLE_SWITCH
        if (g_stop_afh_timer)
        {
            //do not need to stop afh timer
            g_stop_afh_timer = FALSE;
        }
#endif
        lmp_send_lmp_not_accepted(ce_index, LMP_SWITCH_REQ_OPCODE,
                                  (UCHAR)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr),
                                  SWITCH_NOT_ALLOWED_ERROR);
        return BT_FW_SUCCESS;
    }

    switch_instant = lmp_pdu_ptr->payload_content[1] |
                     (lmp_pdu_ptr->payload_content[2] << 8) |
                     (lmp_pdu_ptr->payload_content[3] << 16) |
                     (lmp_pdu_ptr->payload_content[4] << 24);

    if (lcl_ce_status == LMP_BB_HL_CONNECTED /* during connection? */
            && ce_ptr->allow_role_switch == 0x00) /* switch not allowed? */
    {
#ifdef _STOP_AFH_TIMER_WHILE_ROLE_SWITCH
        if (g_stop_afh_timer)
        {
            //do not need to stop afh timer
            g_stop_afh_timer = FALSE;
        }
#endif
        LMP_INF(ALLOW_ROLE_SWITH_DURING_CONNECTION_IS_NOT_SET,0,0);
        /* Local device not allow role switch as part of the Connection */
        lmp_send_lmp_not_accepted(ce_index, LMP_SWITCH_REQ_OPCODE,
                                  (UCHAR)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr),
                                  SWITCH_NOT_ALLOWED_ERROR);
        return BT_FW_SUCCESS;
    }

    if ((ce_ptr->link_policy_settings & LMP_LP_MASTER_SLAVE_SWITCH) == 0
            && ce_ptr->ce_status != LMP_BB_HL_CONNECTED) /* after conn? */
    {
#ifdef _STOP_AFH_TIMER_WHILE_ROLE_SWITCH
        if (g_stop_afh_timer)
        {
             //do not need to stop afh timer
            g_stop_afh_timer = FALSE;
        }
#endif
        /* Link policy disallows Role-Switch */
        lmp_send_lmp_not_accepted(ce_index, LMP_SWITCH_REQ_OPCODE,
                                  (UCHAR)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr),
                                  (UCHAR)PDU_NOT_ALLOWED_ERROR);
        return BT_FW_SUCCESS;
    }

    lc_get_clock_in_scatternet(&native_clock, ce_ptr->phy_piconet_id);

    instant_status =
        lc_check_for_clock_wrap_around(native_clock, (switch_instant << 1) );

    if (instant_status == BT_CLOCK_MORE_THAN_12_HRS_AWAY)
    {
#ifdef _STOP_AFH_TIMER_WHILE_ROLE_SWITCH
        if (g_stop_afh_timer)
        {
            //do not need to stop afh timer
            g_stop_afh_timer = FALSE;
        }
#endif
        lmp_send_lmp_not_accepted(ce_index, LMP_SWITCH_REQ_OPCODE,
                                  (UCHAR)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr),
                                  INSTANT_PASSED_ERROR);
        return BT_FW_SUCCESS;
    }

    /* Make sure that instant is even. */
    switch_instant = (UINT32) (switch_instant & (~0x01));
    ce_ptr->switch_instant = switch_instant;
#ifdef _AUTO_ROLE_SW_WHEN_INSTANT_IS_TOO_FAR_AWAY
	UCHAR lmp_send_not_accept = FALSE;
#endif

    if (ce_ptr->remote_dev_role == MASTER)
    {
#ifdef _AUTO_ROLE_SW_WHEN_INSTANT_IS_TOO_FAR_AWAY
        /* dape: If switch_instant is too far away (Here, 400 slots) 
           and if we are slave, the slot_offset we send may have a 
           big difference at the switch_instant. So we send 
           lmp_not_accept pdu at first and then send 
           lmp_switch_req pdu with a switch_instant closer. */
            if ((switch_instant << 1) - native_clock > 800)
            {
#ifdef _STOP_AFH_TIMER_WHILE_ROLE_SWITCH
                if (g_stop_afh_timer)
                {
                    //do not need to stop afh timer
                    g_stop_afh_timer = FALSE;
                }
#endif
                RT_BT_LOG(RED, DAPE_TEST_LOG435, 2,(switch_instant << 1), native_clock);
                lmp_send_lmp_not_accepted(ce_index, LMP_SWITCH_REQ_OPCODE,
                (UCHAR)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr),
                SWITCH_NOT_ALLOWED_ERROR);
                lmp_send_not_accept = TRUE;
                lmp_start_role_switch_during_conn_as_slave(ce_index, 0);
            }
            else
            {
                lmp_send_slot_offset_pdu(ce_index, REMOTE_DEV_TID);
            }
#else
        lmp_send_slot_offset_pdu(ce_index, REMOTE_DEV_TID);
#endif
    }

#ifdef COMPILE_NESTED_PAUSE_RESUME
    aclq_mark_am_addr_as_paused(ce_ptr->am_addr,
                                ce_ptr->phy_piconet_id, ACL_PAUSED_MSS);
#else /* COMPILE_NESTED_PAUSE_RESUME */
    aclq_mark_am_addr_as_paused(ce_ptr->am_addr,
                                ce_ptr->phy_piconet_id);
#endif /* COMPILE_NESTED_PAUSE_RESUME */

    /* Set cont_poll_count as master. */
    if(ce_ptr->remote_dev_role == SLAVE)
    {
        ce_ptr->cont_poll_count = LC_CONT_POLL_SLOT_COUNT;
    }
#ifdef _AUTO_ROLE_SW_WHEN_INSTANT_IS_TOO_FAR_AWAY
	if (lmp_send_not_accept == FALSE)
	{
#endif
    lmp_send_lmp_accepted(ce_index, LMP_SWITCH_REQ_OPCODE,
                          (UCHAR)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr),
                          LMP_NO_STATE_CHANGE);
#ifdef _AUTO_ROLE_SW_WHEN_INSTANT_IS_TOO_FAR_AWAY
    }
#endif

    ce_ptr->role_switch_accepted_flag = TRUE;
    if (lcl_ce_status != LMP_BB_HL_CONNECTED)
    {
        lmp_set_ce_status(ce_index, LMP_ROLE_SWITCHING);
    }


#ifdef _CCH_RS_ISSUE_

    if (( hci_baseband_cmd_pending == TRUE) && (lmp_periodic_inquiry == FALSE) )
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

        hci_generate_inquiry_complete_event(HCI_COMMAND_SUCCEEDED);

    }

#ifdef COMPILE_PERIODIC_INQUIRY

    if (lmp_periodic_inquiry == TRUE)
    {
        if (lc_kill_periodic_inquiry()!= API_SUCCESS)
        {
            BT_FW_HCI_ERR(HCI_PERIODIC_INQUIRY_MODE_COMMAND_FAILED,0,0);
        }
        else
        {
            lmp_self_device_data.device_status = LMP_IDLE ;
            hci_baseband_cmd_pending = FALSE;
 
            /* Reset the inquiry result table */
            memset(lmp_inquiry_result_data, 0,
                   LMP_MAX_INQUIRY_RESULT_DATA *sizeof(LMP_INQUIRY_RESULT_DATA));

            lmp_num_inq_resp_received = 0 ;
        }

        lmp_periodic_inquiry = FALSE;

        RT_BT_LOG(RED, BB_MSG_KILL_PERIODIC_INQUIRY, 0, 0);

       hci_generate_inquiry_complete_event(HCI_COMMAND_SUCCEEDED);		
    }
#endif
#endif

    return BT_FW_SUCCESS;
}

#endif /* COMPILE_ROLE_SWITCH */

/**
 * Handles the LMP_version_req pdu from the remote device.
 *
 * \param lmp_pdu_ptr Pointer to LMP PDU.
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_version_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index)
{
    UCHAR parameter_list[LMP_PDU_MAX_LEN];

    /*
     * Store the remote device version information.
     */
    lmp_connection_entity[ce_index].lmp_version =
                                lmp_pdu_ptr->payload_content[1];
    lmp_connection_entity[ce_index].rem_manuf_name =
                                lmp_pdu_ptr->payload_content[2] |
                                (lmp_pdu_ptr->payload_content[3] << 8);
    lmp_connection_entity[ce_index].lmp_subversion =
                                lmp_pdu_ptr->payload_content[4] |
                                (lmp_pdu_ptr->payload_content[5] << 8);

    /*
     * send the local device version information.
     */
    parameter_list[0] = LMP_VERSION_RES_OPCODE;
#ifdef _DAPE_UNIFY_VERSION_FEATURE_UINT
    parameter_list[2] = lmp_self_device_data.lmp_version;
#else
    if (IS_BT42)
    {
        parameter_list[2] = BT_FW_LMP_VERSION_BT42;            
    }
    else if (IS_BT41)
    {
        parameter_list[2] = BT_FW_LMP_VERSION_BT41;            
    }
    else if (IS_BT40)
    {
        parameter_list[2] = BT_FW_LMP_VERSION_BT40;            
    }
    else if (IS_BT30)
    {
        parameter_list[2] = BT_FW_LMP_VERSION_BT30;            
    }
    else
    {
        parameter_list[2] = BT_FW_LMP_VERSION_BT21_PLUS_EDR;
    }
#endif    
    parameter_list[3] = LSB(otp_str_data.bt_manufacturer_name);
    parameter_list[4] = MSB(otp_str_data.bt_manufacturer_name);
    parameter_list[5] = LSB(BT_FW_LMP_SUBVERSION);
    parameter_list[6] = MSB(BT_FW_LMP_SUBVERSION);

    lmp_generate_pdu(ce_index, parameter_list, LMP_VERSION_RES_LEN,
                     REMOTE_DEV_TID, LMP_NO_STATE_CHANGE);

    return BT_FW_SUCCESS ;
}

/**
 * Handles the LMP_version_res pdu from the remote device.
 *
 * \param lmp_pdu_ptr Pointer to LMP PDU.
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_version_resp_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = & lmp_connection_entity[ce_index];

    /* Store the remote device version information.*/
    ce_ptr->lmp_version = lmp_pdu_ptr->payload_content[1];
    ce_ptr->rem_manuf_name = lmp_pdu_ptr->payload_content[2] |
                            (lmp_pdu_ptr->payload_content[3] << 8);
    ce_ptr->lmp_subversion = lmp_pdu_ptr->payload_content[4] |
                            (lmp_pdu_ptr->payload_content[5] << 8);

    if (ce_ptr->hci_cmd_bits & REMOTE_VER_INFO_BIT_MASK)
    {
        ce_ptr->hci_cmd_bits &= (~REMOTE_VER_INFO_BIT_MASK);

        hci_generate_remote_version_information_complete_event(
            HCI_COMMAND_SUCCEEDED,
            ce_ptr->connection_type.connection_handle,
            ce_index);
    }

    return BT_FW_SUCCESS;
}

/**
 * Handles the LMP_clk_offset_req pdu from the remote device.
 *
 * \param lmp_pdu_ptr Pointer to LMP PDU.
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_clockoffset_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index)
{
    UCHAR parameter_list[LMP_PDU_MAX_LEN];

    if(lmp_connection_entity[ce_index].remote_dev_role == SLAVE)
    {
        /*Return not accepted*/
        lmp_send_lmp_not_accepted(ce_index,LMP_CLKOFFSET_REQ_OPCODE,
                                  REMOTE_DEV_TID,
                                  PDU_NOT_ALLOWED_ERROR);
        return BT_FW_SUCCESS ;
    }

    parameter_list[0] = LMP_CLKOFFSET_RES_OPCODE ;
    /*
     * Get local device clock offset value.
     */
    parameter_list[2] = LSB(lmp_connection_entity[ce_index].clock_offset);
    parameter_list[3] = MSB(lmp_connection_entity[ce_index].clock_offset);
    lmp_generate_pdu(ce_index, parameter_list, LMP_CLKOFFSET_RES_LEN,
                     REMOTE_DEV_TID, LMP_NO_STATE_CHANGE);
    return BT_FW_SUCCESS ;
}

/**
 * Handles the LMP_clk_offset_res pdu from the remote device.
 *
 * \param lmp_pdu_ptr Pointer to LMP PDU.
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_clockoffset_resp_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    /* Store the clock offset of the remote device in connection entity db */
    ce_ptr->clock_offset = lmp_pdu_ptr->payload_content[1] |
                          (lmp_pdu_ptr->payload_content[2] << 8);

    /* Send Read clock offset complete event to the host.*/
    if (ce_ptr->hci_cmd_bits & READ_CLOCK_OFFSET_BIT_MASK)
    {
        ce_ptr->hci_cmd_bits &= (~READ_CLOCK_OFFSET_BIT_MASK);

        hci_generate_clock_offset_complete_event(
            HCI_COMMAND_SUCCEEDED,
            ce_ptr->connection_type.connection_handle,
            ce_ptr->clock_offset);
    }

    return BT_FW_SUCCESS;
}

/**
 * Handles the LMP_supervision_timeout pdu from the remote device.
 *
 * \param lmp_pdu_ptr Pointer to LMP PDU.
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_supervision_timeout_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index)
{
    UINT16 link_super_timeout;

    /* Only master can send this PDU. */
    if(lmp_connection_entity[ce_index].remote_dev_role != MASTER)
    {
        lmp_send_lmp_not_accepted(ce_index,LMP_SUPERVISION_TIMEOUT_OPCODE,
                                  REMOTE_DEV_TID,
                                  PDU_NOT_ALLOWED_ERROR);
        return BT_FW_SUCCESS;
    }
    link_super_timeout = lmp_pdu_ptr->payload_content[1] |
                        (lmp_pdu_ptr->payload_content[2] << 8);

    lmp_connection_entity[ce_index].link_supervision_timeout =
        link_super_timeout;

    LMP_INF(STOPPING_THE_LINK_SUPERVISION_TIMEOUT,0,0);
    OS_STOP_TIMER(lmp_connection_entity[ce_index].supervision_timeout_handle,0);

    if(link_super_timeout != 0)
    {
        LMP_INF(RESTARTING_THE_LINK_SUPERVISION_TIMEOUT,0,0);
        OS_START_TIMER(
            lmp_connection_entity[ce_index].supervision_timeout_handle,
            (UINT16)(SLOT_VAL_TO_TIMER_VAL(LMP_SUPERVISION_TIMER_RESOLUTION)));

        lmp_connection_entity[ce_index].supervision_timeout_flag = TRUE ;

#ifdef _CORRECT_LMP_RESPONSE_TIMER
        if (g_lmp_response_timeout_time < link_super_timeout)
        {
            g_lmp_response_timeout_time = link_super_timeout;
        }
        else
        {
            g_lmp_response_timeout_time = LMP_RESPONSE_TIMEOUT;
        }
#endif
    }

    hci_generate_lsto_change_event(lmp_connection_entity[ce_index].
                                   connection_type.connection_handle,
                                   link_super_timeout);
#ifdef COMPILE_SNIFF_MODE
    lmp_ssr_disable(ce_index);
#endif
    return BT_FW_SUCCESS ;
}

/**
 * Handles the LMP_host_connection_req pdu from the remote device.
 *
 * \param lmp_pdu_ptr Pointer to LMP PDU.
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_host_connection_request_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index)
{
    UCHAR link_type;
    UCHAR generate_event;
    UINT32 class_of_device;
    UCHAR bd_addr[LMP_BD_ADDR_SIZE] ;
    TimerHandle_t host_timeout_timer_handle ;
    LMP_CONNECTION_ENTITY *ce_ptr;
    UINT16 connection_accept_to;

#ifdef _ROM_CODE_PATCHED_
#ifdef _DAPE_8723_RCP_
    UCHAR return_status;
    if (rcp_lmp_handle_conn_req_func != NULL)
    {
        if ( rcp_lmp_handle_conn_req_func((void *)&return_status, ce_index) )
        {
            return return_status;
        }
    }    
#endif	
#endif

    ce_ptr = &lmp_connection_entity[ce_index];

        if((g_efuse_lps_setting_3.iot_ralink_send_fea_req_later)
		&& (ce_ptr->host_con_req_rx_flag == TRUE))
        {
            return BT_FW_SUCCESS;
        }

    ce_ptr->host_con_req_rx_flag = TRUE;

    if ((ce_ptr->hci_cmd_bits
            & (REMOTE_SUP_FEA_BIT_MASK|REMOTE_EX_FEA_BIT_MASK)) != 0x0)
    {
        /* Feature transaction in progress, will handle this pdu at the end
         * of feature transaction
         */         
        return BT_FW_SUCCESS;
    }
    
    ce_ptr->got_host_conn_req = 1;
    if (g_efuse_lps_setting_3.iot_ralink_send_fea_req_later)
    {
        if (ce_ptr->got_remote_feature == 0)
	    {
            lmp_send_features_req_or_res_pdu(ce_index, LMP_FEATURES_REQ_OPCODE,
                                             SELF_DEV_TID);
            ce_ptr->hci_cmd_bits |= REMOTE_SUP_FEA_BIT_MASK;
            ce_ptr->lmp_expected_pdu_opcode  |= lmp_get_opcode_mask(
                                                    LMP_FEATURES_RES_OPCODE, 0x0);
	    }
    }

    link_type = ACL_LINK;

    switch(ce_ptr->ce_status)
    {
            /* Accept host connection request after the hardware level conenction. */
        case LMP_BB_HL_CONNECTED :
            break ;

        default :
            /*Any other states are invalid.*/
            LMP_ERR(INVALID_STATE_HOST_CONN_REQ_PDU_RECD,0,0);
            return BT_FW_SUCCESS ;
    }

    /*
     * This is valid only when the status of the connection entity is CONNECTING
     * for the other states reject the PDU.
     */
    /* Update the connection entity structure with the link_type info */
    ce_ptr->connection_type.link_type = link_type;

    /* Fill in the address string */
    memcpy(bd_addr, ce_ptr->bd_addr, LMP_BD_ADDR_SIZE);

    /* Fill in the class of device parameter */
    class_of_device = ce_ptr->class_of_device;

    generate_event=
        hci_pass_event_through_event_filter(CONNECTION_SETUP_FILTER,
                                            bd_addr, class_of_device);
#ifdef _DAPE_TEST_AUTO_CONN
#if 0
if (g_host_state)  
{
    UINT8 index;
    UINT8 find_result = 1;

	LEGACY_WHITE_LIST *white_list;
	
	for (index = 0; index < LEGACY_MAX_WHITE_LIST_SIZE; index++ )
    {
        white_list = &conn_white_list[index];

        find_result = memcmp(white_list->bd_addr, &ce_ptr->bd_addr[0], 6);
        if ((find_result == 0) && (white_list->enabled))
        {
	        RT_BT_LOG(WHITE, DAPE_TEST_LOG525, 6, white_list->bd_addr[5], 
				white_list->bd_addr[4], 
				white_list->bd_addr[3], 
				white_list->bd_addr[2], 
				white_list->bd_addr[1],
				white_list->bd_addr[0]);
			break;
        }
	}
	if(find_result != 0)
	{
	    RT_BT_LOG(WHITE, DAPE_TEST_LOG527, 6,ce_ptr->bd_addr[5],
			ce_ptr->bd_addr[4],
			ce_ptr->bd_addr[3],
			ce_ptr->bd_addr[2],
			ce_ptr->bd_addr[1],
			ce_ptr->bd_addr[0]);
            lmp_send_lmp_not_accepted(ce_index,LMP_HOST_CONNECTION_REQ_OPCODE,
                                      REMOTE_DEV_TID,
                                      HOST_REJECTED_SECURITY_REASONS_ERROR);
			return BT_FW_SUCCESS;
	}
}
#endif
#endif

    /* Depending upon the return value do the following */
    switch (generate_event)
    {
        case FALSE :
            lmp_send_lmp_not_accepted(ce_index,LMP_HOST_CONNECTION_REQ_OPCODE,
                                      REMOTE_DEV_TID,
                                      HOST_REJECTED_SECURITY_REASONS_ERROR);

            break;

        case HCI_CONNECTION_REQUEST_EVENT :
            connection_accept_to = lmp_self_device_data.conn_accept_timeout;
            /* Dispatch HCI_Connection_Request Event based on the Event Filter*/
            if (!hci_generate_connection_request_event(
                        bd_addr, class_of_device, link_type))
            {
                connection_accept_to = 0x1;
            }

            /* Start a Connection Accept Timeout timer here. If the timer fires
             * then cause an interrupt and handle it to generate an
             * LMP_not_accepted pdu with the reason HOST_TIMEOUT */

            if(OS_CREATE_TIMER(ONESHOT_TIMER, &ce_ptr->conn_accept_timer_handle,
                    lmp_conn_accept_timeout_timer_handler,
                    (void *)((UINT32)ce_ptr->connection_type.connection_handle),
                    0) != BT_ERROR_OK)
            {
                LMP_ERR(HOST_TIMEOUT_TIMER_CREATION_FAILED,0,0);
            }

            host_timeout_timer_handle = ce_ptr->conn_accept_timer_handle;
            if(OS_START_TIMER(host_timeout_timer_handle,connection_accept_to)
                    != BT_ERROR_OK)
            {
                LMP_ERR(OS_START_TIMER_FAILED,0,0);
            }

            lmp_set_ce_status(ce_index, LMP_WAITING_FOR_CONN_ACCEPT);
            //		LMP_INF(CONNECTION_ACCEPT_TIMER_STARTED_CLOCK_VALUE,1,native_clock);

            break;

        case HCI_CONNECTION_COMPLETE_EVENT :
            /* Implies Auto Accept Flag is ON. Generate an LMP_accepted PDU */
            lmp_accept_host_connection_request_pdu(ce_index);

            break;

#ifdef COMPILE_ROLE_SWITCH
        case HCI_CONNECTION_COMPLETE_EVENT_WITH_RS:

            lmp_start_role_switch_during_conn_as_slave(ce_index, bd_addr);

            break;
#endif

        default:
            LMP_ERR(INVALID_CONNECTION_REQUEST,0,0);
            return BT_FW_ERROR;
    }

    return BT_FW_SUCCESS;
}

/**
 * Handles the LMP_setup_complete pdu from the remote device.
 *
 * \param lmp_pdu_ptr Pointer to LMP PDU.
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_setup_complete_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY* ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    switch (ce_ptr->setup_complete_status)
    {
        case BEGIN_SETUP:
            ce_ptr->setup_complete_status = RECD_SETUP_COMPLETE;
            break;

        case SENT_SETUP_COMPLETE:
            ce_ptr->setup_complete_status = RCVD_SENT_SETUP_COMPLETE;
            lmp_decide_to_send_conn_complete_evt(ce_index);
            ce_ptr->setup_complete_status = CONN_COMPLETE_EVENT;
            break;

        case RECD_SETUP_COMPLETE:
        case CONN_COMPLETE_EVENT:
            lmp_send_lmp_not_accepted(ce_index, LMP_SETUP_COMPLETE_OPCODE,
                                      REMOTE_DEV_TID, PDU_NOT_ALLOWED_ERROR);
            break;
    } /* end switch(ce_ptr->setup_complete_status) */

    return BT_FW_SUCCESS;
}


#ifdef COMPILE_CQDDR
#define INVALID_PKT_TYPE 0x00

UINT16 lmp_get_pkt_from_data_rate(UCHAR data_rate, UCHAR ptt_status)
{
    UCHAR bitmask;
    UINT16 packet_type = INVALID_PKT_TYPE;

    if (ptt_status == LMP_PTT_IDLE)
    {
        /* Get BR packet types */
        bitmask = data_rate & 0x07; /* bit0: (0/1) use FEC or not 
                                       bit[2:1]= 0, no preference
                                                 1, use 1-slot pkts
                                                 2, use 3-slot pkts,
                                                 3, use 5-slot pkts */

        if (bitmask & 1)
        {
            packet_type = ALL_BR_PACKETS ^ ALL_FEC_PACKETS;
        }
        else
        {
            //packet_type = ALL_FEC_PACKETS;
            packet_type = ALL_BR_PACKETS; /* allow FEC */
        }

        switch (bitmask >> 1)
        {
            case 0:
                break;
            case 1:
                packet_type &= ALL_BR_ONE_SLOT_PACKETS;
                break;
            case 2:
                packet_type &= (ALL_BR_THREE_SLOT_PACKETS | 
                                ALL_BR_ONE_SLOT_PACKETS);
                break;
            case 3:
                packet_type &= (ALL_BR_FIVE_SLOT_PACKETS | 
                                 ALL_BR_THREE_SLOT_PACKETS | 
                                 ALL_BR_ONE_SLOT_PACKETS);
                break;
        }
        packet_type |= ALL_EDR_PACKETS;
    }
    else if (ptt_status == LMP_PTT_ENABLED)
    {
        /* Get EDR packet types */
        bitmask = data_rate >> 3; /* bit[4:3] = 0, use DM1 pkts
                                                1, use 2Mbps pkts
                                                2, use 3Mbps pkts
                                                3, reserved 
                                     bit[6:5] = 0, no preference
                                                1, use 1-slot pkts
                                                2, use 3-slot pkts,
                                                3, use 5-slot pkts
                                     bit[7], reserbed, 0 */

        switch (bitmask & (BIT1 | BIT0))
        {
            case 0:
                packet_type = HOST_DM1 | ALL_EDR_PACKETS;
                return packet_type;
            case 1:
                packet_type |= ALL_EDR_2MBPS_PACKETS;
                break;
            case 2:
                packet_type |= ALL_EDR_3MBPS_PACKETS;
                break;
        }

        switch ((bitmask >> 2) & 0x03)
        {
            case 0:
                break;
            case 1:
                packet_type &= ALL_EDR_ONE_SLOT_PACKETS;
                break;
            case 2:
                packet_type &= (ALL_EDR_ONE_SLOT_PACKETS | 
                                 ALL_EDR_THREE_SLOT_PACKETS);
                break;
            case 3:
                packet_type &= (ALL_EDR_ONE_SLOT_PACKETS | 
                                 ALL_EDR_THREE_SLOT_PACKETS | 
                                 ALL_EDR_FIVE_SLOT_PACKETS);
                break;
        }

        packet_type = ALL_EDR_PACKETS & ~packet_type;
    }
    else
    {
        packet_type = INVALID_PKT_TYPE;
        LMP_LOG_INFO(LOG_LEVEL_HIGH, NOT_HANDLING_PREFERRED_RATE_PDU_AS_PTT_BIT_IS_BEING_SET_OR_CLEARED,0,0);
    }

    return packet_type;
}

/**
 * Handle the preferred rate pdu from the remote device.
 *
 * \param lmp_pdu_ptr Pointer to LMP PDU.
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
API_RESULT lmp_handle_preferred_data_rate(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    UINT16 packet_type;
    UCHAR data_rate;
    

    ce_ptr = &lmp_connection_entity[ce_index];

    if (ce_ptr->sent_autorate_pdu == FALSE)
    {
        LMP_LOG_INFO(LOG_LEVEL_HIGH, RECEIVED_PREFERRED_DATA_RATE_PDU_ALTHOUGH_AUTO_RATE_PDU_HAS_NOT_BEEN_SENT,0,0);

        return BT_FW_ERROR;
    }

    data_rate = lmp_pdu_ptr->payload_content[1];
    packet_type = lmp_get_pkt_from_data_rate(data_rate, ce_ptr->ptt_status);

    if (packet_type != INVALID_PKT_TYPE)
    {
        ce_ptr->preferred_cqddr_pkt_type = packet_type;
        lc_update_pkts_allowed(ce_index);
    }

    RT_BT_LOG(GREEN, LMP_MSG_PREFER_PACKET_TYPE, 6, 
                            ce_ptr->ptt_status, packet_type, data_rate,
                            ce_ptr->pkts_allowed.status, 
                            ce_ptr->last_accepted_max_slot,
                            ce_ptr->last_max_slot_req_sent);
    return BT_FW_SUCCESS;
}

/**
 * Handle the auto rate pdu from the remote device.
 * Note : This function should be made into a macro.
 * \param lmp_pdu_ptr Pointer to LMP PDU.
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
API_RESULT lmp_handle_auto_data_rate(LMP_PDU_PKT *lmp_pdu_ptr,
                                     UINT16 ce_index)
{
    lmp_connection_entity[ce_index].received_auto_rate_pdu = TRUE;

    /* Queue preferred rate pdu */
    lmp_calculate_and_send_preferred_rate_pdu(ce_index);
    return BT_FW_SUCCESS;
}

#endif /* COMPILE_CQDDR */

#ifdef COMPILE_PARK_MODE
/**
 * Handles the LMP_set_broadcast_scan_window pdu from the remote device.
 *
 * \param lmp_pdu_ptr Pointer to LMP PDU.
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_set_broadcast_scan_window(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index)
{
    UCHAR offset = 0;
    UCHAR tc_flag ;
    UINT16 new_Nbeacon,slots ;

    if (lmp_connection_entity[ce_index].ce_status != LMP_PARK_MODE)
    {
        lmp_send_lmp_not_accepted(ce_index,LMP_SET_BROADCAST_SCAN_WINDOW_OPCODE,
                                  REMOTE_DEV_TID,
                                  PDU_NOT_ALLOWED_ERROR);
        return BT_FW_SUCCESS;
    }

    offset++;
    /*Read timing control flags*/
    tc_flag = lmp_pdu_ptr->payload_content[offset] ;
    offset++;

    /* Check zero bit is 1 or 0
     * If the zero bit is 1 DB is present, otherwise it is optional
     */
    if(tc_flag & 0x01)
    {
        LMP_INF(UNPARK_PM_ADDR_REQ_DB_IS_PRESENT,0,0);
        offset += 2 ; //opcode + TcFlags + Db
    }

    slots = (lmp_pdu_ptr->payload_content[offset + 1] << 8) |
             lmp_pdu_ptr->payload_content[offset];
    new_Nbeacon = (UINT16)(slots / lmp_connection_entity[ce_index].Delta_beacon);
    new_Nbeacon = (UINT16)(new_Nbeacon + lmp_connection_entity[ce_index].Nbeacon);

    LMP_LOG_INFO(LOG_LEVEL_HIGH, NEW_BROADCAST_SCAN_WINDOW_SLOTS, 2, new_Nbeacon,slots);
    LC_SET_BROADCAST_SCAN_WINDOW(new_Nbeacon);
    lmp_connection_entity[ce_index].bc_scan_window = TRUE;
    return BT_FW_SUCCESS;
}

/**
 * Handles the LMP_modify_beacon pdu from the remote device.
 *
 * \param lmp_pdu_ptr Pointer to LMP PDU.
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_modify_beacon(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index)
{
    UCHAR tc_flag ;
    UCHAR offset = 0;

    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    if (ce_ptr->ce_status != LMP_PARK_MODE)
    {
        lmp_send_lmp_not_accepted(ce_index,LMP_MODIFY_BEACON_OPCODE,
                                  (UCHAR)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr),
                                  (UCHAR)PDU_NOT_ALLOWED_ERROR);
        return BT_FW_SUCCESS;
    }
    offset++;
    /*Read timing control flags*/
    tc_flag = lmp_pdu_ptr->payload_content[offset] ;
    offset++;

    /* Check zero bit is 1 or 0
     * If the zero bit is 1 DB is present, otherwise it is optional
     */
    if(tc_flag & 0x01)
    {
        ce_ptr->Dbeacon = lmp_pdu_ptr->payload_content[offset] |
                         (lmp_pdu_ptr->payload_content[offset+1] << 8);
        offset += 2 ;
    }

    ce_ptr->Tbeacon = lmp_pdu_ptr->payload_content[offset] |
                         (lmp_pdu_ptr->payload_content[offset+1] << 8);
    offset += 2 ;
    ce_ptr->Nbeacon = lmp_pdu_ptr->payload_content[offset] ;
    offset++;

    ce_ptr->Delta_beacon = lmp_pdu_ptr->payload_content[offset] ;
    offset++;

    ce_ptr->D_access = lmp_pdu_ptr->payload_content[offset] ;
    offset++;

    ce_ptr->T_access = lmp_pdu_ptr->payload_content[offset] ;
    offset++;

    ce_ptr->N_acc_slots = lmp_pdu_ptr->payload_content[offset] ;
    offset++;

    ce_ptr->N_poll = lmp_pdu_ptr->payload_content[offset];
    offset++;

    ce_ptr->M_access = (UCHAR)(lmp_pdu_ptr->payload_content[offset]  & 0x0f);

    ce_ptr->access_scheme = (UCHAR)((
                                        lmp_pdu_ptr->payload_content[offset] >> 4 ) & 0x0f);

    lc_kill_beacon(ce_index);

    /* Program beacon to the baseband */
    lc_program_beacon(ce_index);

    return BT_FW_SUCCESS ;
}
#endif /* COMPILE_PARK_MODE */

/**
 * Handles the LMP_detach pdu from the remote device.
 *
 * \param lmp_pdu_ptr Pointer to LMP PDU.
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_detach_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    UINT16 detach_time = 0;
    TimerHandle_t detach_connection_timer = NULL;

    ce_ptr = &lmp_connection_entity[ce_index];

    if(ce_ptr->detach_connection_timer_handle != NULL)
    {
#ifdef ENABLE_LOGGER_LEVEL_2
        LMP_LOG_INFO(LOG_LEVEL_LOW,TIMER_NOT_INVALID,0,0);
#endif
        return BT_FW_SUCCESS;
    }

    switch(ce_ptr->ce_status)
    {
        case LMP_STANDBY:
            LMP_INF(INVALID_STATE_DETACH_PDU_IS_RECEIVED_CE_STATUS, 1, ce_ptr->ce_status);
            return BT_FW_ERROR;

        case LMP_BB_HL_CONNECTED: /* Fall through */
            LMP_INF(DETACH_PDU_FOR_REM_NAME_REQ_HLC_IS_CREATING,0,0);
            break;

        case LMP_DISCONNECTING:
            LMP_INF(THE_LINK_IS_ALREADY_DISCONNECTING_DROPING_THIS_PDU,0,0);
            return BT_FW_ERROR;

        default:
            lmp_set_ce_status(ce_index, LMP_DISCONNECTING);
            break;
    }

    ce_ptr->disconnect_reason = lmp_pdu_ptr->payload_content[1];

#ifdef _CCH_PAGE_CON_
    ce_ptr->connect_reason = 0;
#endif	

//	LMP_INF(DETACH_REASON, 1, lmp_pdu_ptr->payload_content[1]);

    /* Stop the connection related timers. */
    lmp_stop_connection_timers(ce_index);

    OS_CREATE_TIMER(ONESHOT_TIMER, &detach_connection_timer,
            lmp_detach_connection_timer_handler, (void *)((UINT32)ce_index), 0);

    ce_ptr->detach_connection_timer_handle = detach_connection_timer;

    /* Start 3*tpoll timer if we are slave, otherwise 6*tpoll timer. */
    detach_time = (UINT16)
                  ((ce_ptr->Tpoll) * 3 * (1 << ce_ptr->remote_dev_role));

    detach_time = SLOT_VAL_TO_TIMER_VAL(detach_time);
    if(OS_START_TIMER(detach_connection_timer, detach_time) != BT_ERROR_OK)
    {
        LMP_ERR(OS_START_TIMER_FAILED,0,0);
    }
    ce_ptr->detach_timer_state = REMOTE_DETACH_STATE;

#ifdef TEST_MODE
    if (lmp_self_device_data.test_mode != HCI_NO_LOOPBACK_MODE)
    {
        lmp_self_device_data.test_mode = HCI_DEVICE_UNDER_TEST_MODE;
    }

#ifdef _RTL8723B_DUT_MODE_
    /* exit dut mode */
    BB_write_baseband_register(RADIO_SELECT_REGISTER, 0x0107);
#endif    
#endif /* TEST_MODE */

    return BT_FW_SUCCESS;
}

#ifdef COMPILE_HOLD_MODE
/**
 * Handles the LMP_hold pdu from the remote device.
 *
 * \param lmp_pdu_ptr Pointer to LMP PDU.
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_hold_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index)
{
    UINT32 clock_value ;
    INT32 hold_instant ;
    UINT32 hold_instant_val ;
    UINT16 hold_time ;
    UINT32 hold_instant_time ;
    UCHAR parameter_list[LMP_HOLD_REQ_OPCODE];
    UINT16 tpoll_val;
    UINT32 native_clock;

    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    /* Calculate the timing control flags */
    hold_time =  lmp_pdu_ptr->payload_content[1] |
                (lmp_pdu_ptr->payload_content[2] << 8);
    hold_instant_val = lmp_pdu_ptr->payload_content[3] |
                      (lmp_pdu_ptr->payload_content[4] << 8) |
                      (lmp_pdu_ptr->payload_content[5] << 16) |
                      (lmp_pdu_ptr->payload_content[6] << 24);

#ifdef COMPILE_CHECK_LOCAL_FEATURES
    if((lmp_feature_data.feat_page0[0] & LMP_HOLD_MODE_FEATURE) == FALSE)
    {
        LMP_ERR(LOCAL_DEVICE_NOT_SUPPORTING_HOLD_FEATURE,0,0);
        return UNSUPPORTED_REMOTE_FEATURE_ERROR;
    }
#endif
#ifdef _SUPPORT_VER_4_1_
#ifdef _SUPPORT_PCA_ADJUST
    if (IS_BT41)
    {
        if (bt_pca_manager.pca_updating)
        {
            return PDU_NOT_ALLOWED_ERROR;
        }    
    }
#endif
#endif

    /* Make sure that the hold instant is even. */
    hold_instant_val = (UINT32) (hold_instant_val & (~0x01));
    ce_ptr->hold_instant = hold_instant_val;

    lc_get_clock_in_scatternet(&clock_value, ce_ptr->phy_piconet_id);

    clock_value >>= 1 ;
    /* M/S forces to the hold mode
     * The hold time and hold instant values are valid.
     */

    hold_instant = hold_instant_val - clock_value;
    tpoll_val = ce_ptr->Tpoll ;
    tpoll_val = (UINT16)(tpoll_val * 6);

    if(ce_ptr->remote_dev_role == MASTER)
    {
        LMP_INF(MASTER_FORCING_HOLD_MODE,0,0);
        /*
         * Master intiated the hold mode, sent LMP_HOLD
         * or Slave device intiated the hold mode, sent LMP_HOLD to Master,
         * Master sent back LMP_HOLD
         */
        if((ce_ptr->ce_status != LMP_CONNECTED) &&
                (ce_ptr->ce_status != LMP_ENTERING_HOLD_MODE))
        {
            lmp_send_lmp_not_accepted(ce_index,LMP_HOLD_OPCODE,
                                      REMOTE_DEV_TID,
                                      PDU_NOT_ALLOWED_ERROR);
            return BT_FW_SUCCESS;
        }
    }
    else
    {
//		LMP_INF(SLAVE_FORCING_HOLD_MODE_SENDING_BACK_HOLD_PDU,0,0);
        /*
         * Remote device is a slave, you are a Master.
         * As a Master, you will receive LMP_hold only when the slave initates
         * the Hold mode and sent LMP_HOLD pdu.
         * Check the received hold_instant is less than  6*Tpoll value,
         * Make it atleast 6*Tpoll value, send it to the remote device.
         * Future transactions will be atleast 6*Tpoll slots.
         */
        if(ce_ptr->ce_status == LMP_CONNECTED)
        {
            if(hold_instant < tpoll_val)
            {
//				LMP_INF(HOLD_INSTANT_IS_6_TPOLL_SELECTING_THE_HOLD_INSTANT_AS_6_TPOLL,0,0);
                hold_instant_time = clock_value + tpoll_val;
            }
            else
            {
                LMP_INF(HOLD_INSTANT_IS_6_TPOLL_SEND_THE_RECEIVED_HOLD_INSTANT_BACK,0,0);
                hold_instant_time = clock_value + hold_instant;
            }
            parameter_list[0] = LMP_HOLD_OPCODE;
            /*
            * hold instnt value should be atleast 6*Tpoll in future.
            * hold instant value is current clock value + 6*Tpoll value.
            */

            /* Make sure that the hold instant is even. */
            hold_instant_time = (UINT32) (hold_instant_time & (~0x01));
            parameter_list[2] = LSB(hold_time);
            parameter_list[3] = MSB(hold_time);
            bt_fw_ultostr(&parameter_list[4], hold_instant_time, 4);
            lmp_generate_pdu(ce_index, parameter_list, LMP_HOLD_LEN,
                             REMOTE_DEV_TID, LMP_NO_STATE_CHANGE);

            ce_ptr->hold_instant = hold_instant_time;
        }
    }

    ce_ptr->hold_mode_accepted_flag = TRUE ;
    ce_ptr->hold_mode_interval = hold_time ;

    if( hold_instant > 0)
    {
        LMP_INF(STARTING_HOLD_INSTANT_TIMER_FOR_SLOTS,1,hold_instant);

        lc_get_clock_in_scatternet(&native_clock ,
                                   ce_ptr->phy_piconet_id);
        native_clock >>= 1 ;

        hold_instant = ce_ptr->hold_instant - native_clock;

        hold_instant = SLOT_VAL_TO_TIMER_VAL(hold_instant);

        lmp_set_ce_status(ce_index, LMP_ENTERING_HOLD_MODE);
    }
    else
    {
        LMP_INF(HOLD_INSTANT_IS_PASSED_NOT_STARTING_THE_HOLD,0,0);
    }

#ifdef COMPILE_NESTED_PAUSE_RESUME
    aclq_mark_am_addr_as_paused(ce_ptr->am_addr,
                                ce_ptr->phy_piconet_id, ACL_PAUSED_HOLD);
#else /* COMPILE_NESTED_PAUSE_RESUME */
    aclq_mark_am_addr_as_paused(ce_ptr->am_addr,
                                ce_ptr->phy_piconet_id);
#endif /* COMPILE_NESTED_PAUSE_RESUME */

    if(ce_ptr->remote_dev_role == MASTER)
    {
        lmp_handle_start_hold_mode(ce_index);
    }

    return BT_FW_SUCCESS;
}

/**
 * Completes the hold mode and sends mode change event to the host.
 *
 * \param ce_index ACL connection entity index.
 * \param transaction_id Transaction ID.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
#ifdef _ENABLE_COMPILE_NESTED_PAUSE_RESUME_
UCHAR lmp_handle_hold_mode_req_complete(UINT16 ce_index, UCHAR transaction_id)
#else
UCHAR lmp_handle_mode_req_complete(UINT16 ce_index, UCHAR transaction_id)
#endif
{
    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    /* Hold Request is not accepted */
    lmp_set_ce_status(ce_index, LMP_CONNECTED);

#ifdef COMPILE_NESTED_PAUSE_RESUME
    aclq_resume_am_addr(ce_ptr->am_addr,
                        ce_ptr->phy_piconet_id, ACL_PAUSED_HOLD);
#else /* COMPILE_NESTED_PAUSE_RESUME */
    aclq_resume_am_addr(ce_ptr->am_addr,
                        ce_ptr->phy_piconet_id);
#endif /* COMPILE_NESTED_PAUSE_RESUME */

    /* Send Mode change event if self device initiated the transaction */
    if(ce_ptr->remote_dev_role != transaction_id)
    {
        /* Send mode change event */
        hci_generate_mode_change_event(
            INVALID_LMP_PARAMETERS_ERROR,
            ce_index,
            LP_ACTIVE_MODE,0X00);
    }
    return BT_FW_SUCCESS ;
}

/**
 * Handles the LMP_hold_req pdu from the remote device.
 *
 * \param lmp_pdu_ptr Pointer to LMP PDU.
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_hold_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index)
{
    UINT16 hold_time ;
    INT32 hold_instant;
    UCHAR transaction_id ;
    UCHAR start_timer_flag = FALSE ;
    UCHAR reason ;
    UCHAR status = HCI_COMMAND_SUCCEEDED;
    UINT32 cur_clk;
    LMP_CONNECTION_ENTITY *ce_ptr;
    UCHAR instant_status;

    ce_ptr = &lmp_connection_entity[ce_index];
    transaction_id = (UCHAR)(lmp_pdu_ptr->payload_content[0] & 0x01);

#ifdef COMPILE_CHECK_LOCAL_FEATURES
    if((lmp_feature_data.feat_page0[0] & LMP_HOLD_MODE_FEATURE) == FALSE)

    {
        LMP_ERR(LOCAL_DEVICE_NOT_SUPPORTING_HOLD_FEATURE,0,0);
        status = UNSUPPORTED_REMOTE_FEATURE_ERROR;
    }
#endif
#ifdef _SUPPORT_VER_4_1_
#ifdef _SUPPORT_PCA_ADJUST
    if (IS_BT41)
    {
        if (bt_pca_manager.pca_updating)
        {
            status = PDU_NOT_ALLOWED_ERROR;
        }    
    }
#endif
#endif

    /*
     * Check the device status : If the device status is CONNECTED state,
     * then allow this PDU else send LMP_NOT_ACCEPTED pdu
     */
    switch(ce_ptr->ce_status)
    {
        case LMP_CONNECTED :         /* Fall through */
        case LMP_ENTERING_HOLD_MODE: /* Fall through */
        case LMP_HOLD_MODE_NEG :
            break;

        default :
            status = PDU_NOT_ALLOWED_ERROR;
            break;
    }

    /*
     * Check this feature is supported from self device features.
     * and Link Policy settings supports this feature.
     */
    if(ce_ptr->remote_dev_role == transaction_id)
    {
        if((ce_ptr->link_policy_settings &
                LMP_LP_MASTER_HOLD_MODE) == 0)
        {
            LMP_ERR(HOLD_MODE_LINK_POLICY_IS_NOT_SET,0,0);
            status = PDU_NOT_ALLOWED_ERROR;
        }
    }

    hold_time =  lmp_pdu_ptr->payload_content[1] |
                (lmp_pdu_ptr->payload_content[2] << 8);
    hold_instant = lmp_pdu_ptr->payload_content[3] |
                  (lmp_pdu_ptr->payload_content[4] << 8) |
                  (lmp_pdu_ptr->payload_content[5] << 16) |
                  (lmp_pdu_ptr->payload_content[6] << 24);

    /* Check if the hold instant is in the past. */
    lc_get_clock_in_scatternet(&cur_clk, ce_ptr->phy_piconet_id);

    instant_status =
        lc_check_for_clock_wrap_around(cur_clk, (hold_instant << 1) );

    if(instant_status == BT_CLOCK_MORE_THAN_12_HRS_AWAY)
    {
        status = INSTANT_PASSED_ERROR;
    }

    if (status == HCI_COMMAND_SUCCEEDED)
    {
        if((reason = lmp_validate_hold_parms(transaction_id, ce_index,
                                             hold_time, &start_timer_flag )) != HCI_COMMAND_SUCCEEDED)
        {
            LMP_INF(HOLD_MODE_PARAMETER_VALIDATION_FAILED,0,0);
            status = reason;
#ifdef _ENABLE_COMPILE_NESTED_PAUSE_RESUME_
            lmp_handle_hold_mode_req_complete(ce_index,transaction_id);
#else
            lmp_handle_mode_req_complete(ce_index,transaction_id);
#endif
        }

        /* Make sure that the instant is even. */
        hold_instant = (UINT32) (hold_instant & (~0x01));

        ce_ptr->hold_instant = hold_instant ;
    }

    if (status != HCI_COMMAND_SUCCEEDED)
    {
        lmp_send_lmp_not_accepted(ce_index, LMP_HOLD_REQ_OPCODE,
                                  transaction_id, status);
        return BT_FW_ERROR;
    }

#ifdef COMPILE_NESTED_PAUSE_RESUME
    aclq_mark_am_addr_as_paused(ce_ptr->am_addr,
                                ce_ptr->phy_piconet_id, ACL_PAUSED_HOLD);
#else /* COMPILE_NESTED_PAUSE_RESUME */
    aclq_mark_am_addr_as_paused(ce_ptr->am_addr,
                                ce_ptr->phy_piconet_id);
#endif /* COMPILE_NESTED_PAUSE_RESUME */

    if(start_timer_flag == TRUE)
    {
        lmp_set_ce_status(ce_index, LMP_HOLD_MODE_NEG);

        ce_ptr->lmp_expected_pdu_opcode |= lmp_get_opcode_mask(
                                               LMP_HOLD_REQ_OPCODE, 0x0);
    }

    return BT_FW_SUCCESS ;
}
#endif /* COMPILE_HOLD_MODE */

#ifdef COMPILE_SNIFF_MODE
/**
 * Handles the LMP_sniff_req pdu from the remote device.
 *
 * \param lmp_pdu_ptr Pointer to LMP PDU.
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_sniff_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index)
{
    UINT16 Tsniff, sniff_slot_offset ;
    UINT16 sniff_attempt, sniff_timeout ;
    UCHAR transaction_id ;
    UCHAR timing_ctl_flag ;
    UCHAR reason ;
    LMP_CONNECTION_ENTITY *ce_ptr;
#ifdef COMPILE_CHECK_LOCAL_FEATURES
    UCHAR local_feature;
#endif

    ce_ptr = &lmp_connection_entity[ce_index];

    transaction_id = (UCHAR)(lmp_pdu_ptr->payload_content[0] & 0x01);
    timing_ctl_flag = lmp_pdu_ptr->payload_content[1];
#ifdef _SUPPORT_VER_4_1_
#ifdef _SUPPORT_PCA_ADJUST
    if (IS_BT41)
    {
        if (bt_pca_manager.pca_updating)
        {
            lmp_send_lmp_not_accepted(ce_index,LMP_SNIFF_REQ_OPCODE,
                                      transaction_id, PDU_NOT_ALLOWED_ERROR);
            return BT_FW_SUCCESS;
        }    
    }
#endif
#endif
#ifdef COMPILE_CHECK_LOCAL_FEATURES
    local_feature = lmp_feature_data.feat_page0[0];

    if((local_feature & LMP_SNIFF_MODE_FEATURE) == FALSE)
    {
        HCI_LOG_ERROR(LOG_LEVEL_LOW, SELF_DEVICE_IS_NOT_SUP_THE_SNIFF_MODE_FEATURE,0,0);
        return UNSUPPORTED_FEATURE_OR_PARAMETER_ERROR;
    }
#endif

    switch(ce_ptr->ce_status)
    {
        case LMP_CONNECTED : /* Fall through. */
        case LMP_SNIFF_MODE_NEG :
            break ;

        default :
            lmp_send_lmp_not_accepted(ce_index,LMP_SNIFF_REQ_OPCODE,
                                      transaction_id, PDU_NOT_ALLOWED_ERROR);
            return BT_FW_SUCCESS;
    }

    if((lmp_feature_data.feat_page0[0] & LMP_SNIFF_MODE_FEATURE) == 0)
    {
        lmp_send_lmp_not_accepted(ce_index,LMP_SNIFF_REQ_OPCODE,
                                  transaction_id,UNSUPPORTED_REMOTE_FEATURE_ERROR);
        return BT_FW_SUCCESS;
    }

    /* Check this feature is supported from self device features.
     * and Link Policy settings supports this feature.
     */
    if(ce_ptr->remote_dev_role == transaction_id)
    {
        if((ce_ptr->link_policy_settings & LMP_LP_MASTER_SNIFF_MODE) == 0)
        {
            lmp_send_lmp_not_accepted(ce_index,LMP_SNIFF_REQ_OPCODE,
                                      transaction_id,PDU_NOT_ALLOWED_ERROR);
            return BT_FW_SUCCESS;
        }
    }
#if 0
    else
    {
        lmp_send_lmp_not_accepted(ce_index,LMP_SNIFF_REQ_OPCODE,
                                  transaction_id, PDU_NOT_ALLOWED_ERROR);
        return BT_FW_SUCCESS;
    }
#endif

    /*
     * Negotiation criteria.
     *
     * Max and Min Sniff intervals are,
     *      a. Received by the host.
     *      b. Default values are Set.
     *
     * Check the Tsniff,
     *      if it is within the range of Max and Min interval.
     *      Send LMP_ACCEPTED
     *
     *      If it is less than Sniff min interval
     *      send LMP_SNIFF_REQ with Tsniff is Sniff min interval,
     *      If you still get value lesser than to
     *      sniff min interval then send LMP_NOT_ACCEPTED.
     *
     *      If it is greater than Sniff max interval
     *      send LMP_SNIFF_REQ with Tsniff is Sniff max interval,
     *      If you still get value greater than to
     *      sniff max interval then send LMP_NOT_ACCEPTED.
     *
     */
    sniff_slot_offset =  lmp_pdu_ptr->payload_content[2] |
                        (lmp_pdu_ptr->payload_content[3] << 8);
    Tsniff = lmp_pdu_ptr->payload_content[4] |
            (lmp_pdu_ptr->payload_content[5] << 8);
    sniff_attempt =  lmp_pdu_ptr->payload_content[6] |
                    (lmp_pdu_ptr->payload_content[7] << 8);
    sniff_timeout = lmp_pdu_ptr->payload_content[8] |
                   (lmp_pdu_ptr->payload_content[9] << 8);

#ifdef _DAPE_CHG_FOR_CLEVO
    UCHAR piconet_id = ce_ptr->phy_piconet_id;
    if ((ce_ptr->remote_dev_role == SLAVE) &&
        (lmp_self_device_data.lc_no_of_connections[piconet_id] > 1) &&
        (Tsniff >= 252))
    {
        lmp_send_lmp_not_accepted(ce_index,LMP_SNIFF_REQ_OPCODE,
                               transaction_id, PDU_NOT_ALLOWED_ERROR);
        return BT_FW_SUCCESS;
    }
#endif

#ifdef _CCH_SNIFF_NEG_TIMEOUT_

    if(g_efuse_rsvd_2.enable_sniff_neg)
    {
        if( ce_ptr->sniff_neg_count < g_efuse_rsvd_2.sniff_neg_time)
        {
            ce_ptr->sniff_neg_count ++;
        }
        else
        {
            ce_ptr->sniff_neg_count = 0;
    	}
        if (ce_ptr->sniff_neg_count == g_efuse_rsvd_2.sniff_neg_time)
        {
            UCHAR temp_slot_num;
            UCHAR force_slot_overlap;

            temp_slot_num = 2;
            if(sniff_attempt > 1)
            {
                temp_slot_num = sniff_attempt << 1;
            }
            if(sniff_timeout > 0)
            {
                temp_slot_num += 2;
            }			
            force_slot_overlap = lmp_force_global_slot_offset(ce_index, 0, Tsniff, 
                             temp_slot_num, sniff_slot_offset);
            RT_BT_LOG(BLUE, CCH_DBG_106, 2,force_slot_overlap, ce_ptr->sniff_neg_count);
            if (force_slot_overlap)
            {
                ce_ptr->sniff_neg_count = 0;
                lmp_send_lmp_accepted(ce_index, LMP_SNIFF_REQ_OPCODE,
                                      transaction_id, LMP_SNIFF_MODE_NEG_TERMINATION);
                /* Store all the sniff parameters in conn entity*/
                ce_ptr->sniff_interval = Tsniff ;
                ce_ptr->sniff_attempt = sniff_attempt;
                ce_ptr->sniff_timeout = sniff_timeout ;
                ce_ptr->sniff_slot_offset = sniff_slot_offset;
                ce_ptr->sniff_max_interval_negotiated = FALSE ;
                ce_ptr->sniff_min_interval_negotiated = FALSE ;
                return BT_FW_SUCCESS;
            }
        }
    }
		
#endif

    if((reason = lmp_validate_sniff_parms(transaction_id,timing_ctl_flag,
                                          ce_index, sniff_slot_offset,
                                          Tsniff,sniff_attempt,sniff_timeout))
            != HCI_COMMAND_SUCCEEDED)
    {
        lmp_send_lmp_not_accepted(ce_index,LMP_SNIFF_REQ_OPCODE,
                                  transaction_id,reason);
        /* Send Mode change event to the host */
        hci_generate_mode_change_event(
            reason,
            ce_index,
            LP_SNIFF_MODE,
            ce_ptr->sniff_interval);

        lmp_set_ce_status(ce_index, LMP_CONNECTED);
        ce_ptr->temp_sniff_interval = 0xffff;
        ce_ptr->temp_sniff_nego_in_progress = FALSE;
        lmp_send_max_slot_pdu_to_all_devices();
    }
    return BT_FW_SUCCESS;
}

/**
 * Handles the LMP_unsniff_req pdu from the remote device.
 *
 * \param lmp_pdu_ptr Pointer to LMP PDU.
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_unsniff_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    /* Note: LMP_SNIFF_INFO_REQUESTING is not allowed here.
       These two nested transactions are not handled. */
    if (ce_ptr->ce_status == LMP_SNIFF_MODE)
    {
        /* Send LMP ACCEPTED to the remote device */
        lmp_send_lmp_accepted(ce_index, LMP_UNSNIFF_REQ_OPCODE,
                              REMOTE_DEV_TID, LMP_NO_STATE_CHANGE);

        /* Exit from sniff mode here. */

        if(ce_ptr->remote_dev_role == SLAVE)
        {
            ce_ptr->cont_poll_count = LC_CONT_POLL_SLOT_COUNT;
            RT_BT_LOG(GRAY, LMP_PDU_3497, 1, ce_index);
        }
        else
        {
#ifdef _CCH_SNIFF_EXIT_EARLY_
            {
                UCHAR am_addr;
                UCHAR phy_piconet_id;
                am_addr = lmp_connection_entity[ce_index].am_addr;
                phy_piconet_id = lmp_connection_entity[ce_index].phy_piconet_id;

                bb_kill_sniff_in_scatternet(am_addr, phy_piconet_id, FALSE);

                BB_start_tpoll(am_addr, ce_ptr->Tpoll, phy_piconet_id);

            }
#endif			

            /* Program sniff-exit-transition bit. */
            lc_program_exit_sniff_transition_mode(ce_index);            
        }
    }
    else
    {
        lmp_send_lmp_not_accepted(ce_index, LMP_UNSNIFF_REQ_OPCODE,
                                  REMOTE_DEV_TID,PDU_NOT_ALLOWED_ERROR);
    }

    return BT_FW_SUCCESS;
}
#endif /* COMPILE_SNIFF_MODE */

#ifdef COMPILE_PARK_MODE
/**
 * Handles the LMP_park_req pdu from the remote device.
 *
 * \param lmp_pdu_ptr Pointer to LMP PDU.
 * \param ce_index ACL connection entity index.
 *
 * \return None.
 */
void lmp_handle_park_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index)
{
    UCHAR reason ;
    UCHAR parameter_list[LMP_PARK_REQ_LEN];
    UCHAR pm_addr ;
    UINT32 clock_val ;
    UCHAR timing_control_flag;
    UCHAR transaction_id = 0;
    UINT16 Tb,min_cal_tb;
    LMP_CONNECTION_ENTITY *ce_ptr;

#ifdef COMPILE_AFH_HOP_KERNEL
    LMP_PDU_PKT *park_lmp_pdu_ptr;
#endif

#ifdef COMPILE_CHECK_LOCAL_FEATURES
    UCHAR local_feature;
#endif

#ifdef COMPILE_CHECK_LOCAL_FEATURES
    local_feature = lmp_feature_data.feat_page0[1];

    if((local_feature & LMP_PARK_MODE_FEATURE) == FALSE)
    {
        HCI_LOG_ERROR(LOG_LEVEL_LOW, SELF_DEVICE_IS_NOT_SUP_THE_PARK_MODE_FEATURE,0,0);
        lmp_send_lmp_not_accepted(ce_index,LMP_PARK_REQ_OPCODE,
                                  transaction_id,
                                  UNSUPPORTED_FEATURE_OR_PARAMETER_ERROR);
        return;
    }
#endif

    ce_ptr = &lmp_connection_entity[ce_index];
    transaction_id = (UCHAR)(lmp_pdu_ptr->payload_content[0] & 0x01);

#ifdef ENABLE_SCO
    /* Check for any SCO connections on this link. */
    if (ce_ptr->no_of_sco_connections != 0)
    {
        lmp_send_lmp_not_accepted(ce_index, LMP_PARK_REQ_OPCODE,
                                  transaction_id, PDU_NOT_ALLOWED_ERROR);
        return;
    }
#endif /* ENABLE_SCO */
#ifdef _SUPPORT_VER_4_1_
#ifdef _SUPPORT_PCA_ADJUST
    if (IS_BT41)
    {
        if (bt_pca_manager.pca_updating)
        {
            lmp_send_lmp_not_accepted(ce_index, LMP_PARK_REQ_OPCODE,
                                      transaction_id, PDU_NOT_ALLOWED_ERROR);
            return;
        }    
    }
#endif
#endif

#ifdef COMPILE_ESCO

/* Check for any Esco connections on this link. */
#ifdef _BRUCE_IMPLEMENTED_NO_OF_ESCO_CONN_
    if (ce_ptr->no_of_esco_connections != 0)
    {
        lmp_send_lmp_not_accepted(ce_index, LMP_PARK_REQ_OPCODE,
                                  transaction_id, PDU_NOT_ALLOWED_ERROR);
        return;
    }
#else
    {
        UCHAR temp_var;
        for (temp_var = 0; temp_var < LMP_MAX_ESCO_CONN_ENTITIES; temp_var++)
        {
            if ( (lmp_esco_connection_entity[temp_var].entity_status == ASSIGNED) &&
                    (lmp_esco_connection_entity[temp_var].ce_index == ce_index) )
            {
                lmp_send_lmp_not_accepted(ce_index,LMP_PARK_REQ_OPCODE,
                                          transaction_id, PDU_NOT_ALLOWED_ERROR);
                return;
            }
        }
    }
#endif
#endif /* COMPILE_ESCO */

    switch(ce_ptr->ce_status)
    {
        case LMP_CONNECTED :
            /* Fall through */
        case LMP_PARK_MODE_REQ :
            break;

        default :
            lmp_send_lmp_not_accepted(ce_index,LMP_PARK_REQ_OPCODE,
                                      (UCHAR)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr),PDU_NOT_ALLOWED_ERROR);

#ifdef COMPILE_AFH_HOP_KERNEL
#ifdef AFH_PARK
            if ( (ce_ptr->afh_disabled_for_park == TRUE) &&
                    (ce_ptr->remote_dev_role == SLAVE) )
            {
                ce_ptr->afh_disabled_for_park = FALSE;
                lmp_update_map(AFH_ENABLE, ce_index, TRUE);
            }
#endif
#endif

            return;
    }

    /* Check link policy settings for remote device initiated transactions. */
    if(ce_ptr->remote_dev_role == transaction_id)
    {
        if((ce_ptr->link_policy_settings & LMP_LP_MASTER_PARK_MODE) == 0)
        {
            lmp_send_lmp_not_accepted(ce_index,LMP_PARK_REQ_OPCODE,
                                      (UCHAR)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr),
                                      PDU_NOT_ALLOWED_ERROR);
            LMP_INF(LINK_POLICY_OR_REMOTE_FEATURE_FAILED_SENT_NOT_ACCEPTED_PDU,0,0);
            return;
        }
    }

    /* Park is not supported if the device is in scatternet configuration. */
    if(ce_ptr->remote_dev_role == transaction_id)
    {
        if(lc_get_no_of_piconets_connected() != 1)
        {
            RT_BT_LOG(GRAY, LMP_PDU_3657, 0, 0);

            lmp_send_lmp_not_accepted(ce_index,LMP_PARK_REQ_OPCODE,
                                      (UCHAR)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr),
                                      UNSPECIFIED_ERROR);

            return;
        }
    }

    /* Only one connection is allowed to be in park state. */
    if(lmp_self_device_data.number_of_parked_dev != 0)
    {
        lmp_send_lmp_not_accepted(ce_index,LMP_PARK_REQ_OPCODE,
                                  (UCHAR)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr),
                                  UNSPECIFIED_ERROR);
        return;
    }

    if(ce_ptr->remote_dev_role == MASTER)
    {
        /* Validate the received parameters. */
        if((reason = lmp_validate_beacon_parameters(lmp_pdu_ptr)) !=
                BT_FW_SUCCESS)
        {
            LMP_INF(INVALID_BEACON_PARAMETERS_RECD_FROM_THE_REMOTE_MASTER,0,0);

            /* Send mode change event */
            if(ce_ptr->remote_dev_role != transaction_id)
            {
                hci_generate_mode_change_event(
                    reason, ce_index, LP_PARK_MODE,
                    ce_ptr->Tbeacon);
            }
            lmp_send_lmp_not_accepted(ce_index,LMP_PARK_REQ_OPCODE,
                                      (UCHAR)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr), reason);

            lmp_set_ce_status(ce_index, LMP_CONNECTED);

            return;
        }

        /*
         * Update the connection entity structure.
         */
        lmp_update_beacon_parameters(lmp_pdu_ptr, ce_index);
        if(ce_ptr->pm_addr == 0)
        {
            /* Master parking the device with pm_addr zero, Master will
             * unpark it using BD_ADDr...
             */
            lmp_park_bd_addr_ce_index = ce_index ;
        }

#ifdef COMPILE_NESTED_PAUSE_RESUME
        aclq_mark_am_addr_as_paused(ce_ptr->am_addr,
                                    ce_ptr->phy_piconet_id, ACL_PAUSED_PARK);
#else /* COMPILE_NESTED_PAUSE_RESUME */
        aclq_mark_am_addr_as_paused(ce_ptr->am_addr,
                                    ce_ptr->phy_piconet_id);
#endif /* COMPILE_NESTED_PAUSE_RESUME */

        /* Send LMP_Accepted. */
        lmp_send_lmp_accepted(ce_index,LMP_PARK_REQ_OPCODE,
                              (UCHAR)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr), LMP_PARK_TRANSITION);
        LMP_INF(DB_TB_NB_DELTAB_DACC_TACC_MACC_NPOLL_NAS, 9,
                ce_ptr->Dbeacon, ce_ptr->Tbeacon, ce_ptr->Nbeacon,
                ce_ptr->Delta_beacon, ce_ptr->D_access,
                ce_ptr->T_access, ce_ptr->M_access,
                ce_ptr->N_poll, ce_ptr->N_acc_slots);

        /*
         * Start a timer for 6*Tpoll slots. When this timer expires
         * put the device in park mode. and then free the am_addr
         * The slave queues LMP_ACCEPTED it starts a timer for 6*Tpoll slots
         * If the baseband-level ack is received before this timer expires
         * it enters the park mode immediatly otherwise it enters park mode
         * when the timer expires.
         * In our implementation since we do n't have the faciliyt to check
         * the ACK received for the which PDU sent. we will go to the park
         * mode once the timer expires.
         */
        lmp_start_park_mode_timer(ce_index);
    }
    else /* if(ce_ptr->remote_dev_role == MASTER) */
    {
        min_cal_tb = MIN_D_acc + (MIN_M_acc * MIN_T_acc) + MIN_N_poll;
        /* Extract the Tb value from remote device */
        Tb = lmp_pdu_ptr->payload_content[4] |
            (lmp_pdu_ptr->payload_content[5] << 8);
        
        if(Tb < min_cal_tb)
        {
            LMP_INF(USING_MIN_VALUES_FOR_PARK_TB,1,min_cal_tb);
            /* Generate the beacon parameters using our minimum possible values*/
            ce_ptr->beacon_max_interval = min_cal_tb;
            ce_ptr->beacon_min_interval = min_cal_tb;
        }
        else
        {
            LMP_INF(USING_SLAVE_SUGGESTED_TB_FOR_PARK_PARAMS,1,Tb);
            /* Use the slave suggested values and generate new beacon params */
            ce_ptr->beacon_max_interval = Tb;
            ce_ptr->beacon_min_interval = Tb;
        }

        /*
         * Master generates beacon parameters, send it to the slave.
         * Slave validates the beacon parameters, If it is acceptable, it
         * sends LMP_ACCEPTED. otherwise it will send LMP_NOT_ACCEPTED.
         */
        lmp_generate_beacon_parameters(ce_index);

        if (ce_ptr->Tbeacon > otp_str_data.bt_beacon_max_interval)
        {
            ce_ptr->Tbeacon = otp_str_data.bt_beacon_max_interval;
        }
        else if (ce_ptr->Tbeacon < otp_str_data.bt_beacon_min_interval)
        {
            ce_ptr->Tbeacon = otp_str_data.bt_beacon_min_interval;
        }

        /* Check for supervision timeout value */
        if((ce_ptr->link_supervision_timeout != 0) &&
                (ce_ptr->link_supervision_timeout <
                 (2*ce_ptr->Tbeacon)+BT_FW_PARK_BEACON_LSTO_MIN_DIFF))
        {
            lmp_send_lmp_not_accepted(ce_index,LMP_PARK_REQ_OPCODE,
                                      (UCHAR)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr),
                                      UNSUPPORTED_FEATURE_OR_PARAMETER_ERROR);

#ifdef COMPILE_NESTED_PAUSE_RESUME
            aclq_resume_am_addr(ce_ptr->am_addr,
                                ce_ptr->phy_piconet_id, ACL_PAUSED_PARK);
#else /* COMPILE_NESTED_PAUSE_RESUME */
            aclq_resume_am_addr(ce_ptr->am_addr,
                                ce_ptr->phy_piconet_id);
#endif /* COMPILE_NESTED_PAUSE_RESUME */

            return;
        }

#ifdef COMPILE_AFH_HOP_KERNEL
        if(ce_ptr->afh_mode == AFH_ENABLE)
        {
            /* AFH is enabled. Disable it and resume park. */
            ce_ptr->rem_park_pending_for_afh = TRUE;
            lmp_update_map(AFH_DISABLE, ce_index, TRUE);

            if (ce_ptr->park_lmp_pdu_ptr == NULL)
            {
                OS_ALLOC_BUFFER(lmp_pdu_buffer_pool_handle, 
                                (void **)(&park_lmp_pdu_ptr));                
            }
            else
            {
                park_lmp_pdu_ptr = ce_ptr->park_lmp_pdu_ptr;
            }
            memcpy(park_lmp_pdu_ptr, lmp_pdu_ptr, sizeof(LMP_PDU_PKT));

            /* Store the command pointer in the LMP_CONNECTION_ENTITY */
            ce_ptr->park_lmp_pdu_ptr = park_lmp_pdu_ptr;
            ce_ptr->afh_disabled_for_park = TRUE;

#ifdef COMPILE_NESTED_PAUSE_RESUME
            aclq_mark_am_addr_as_paused(ce_ptr->am_addr,
                                        ce_ptr->phy_piconet_id, ACL_PAUSED_AFH);
#else /* COMPILE_NESTED_PAUSE_RESUME */
            aclq_mark_am_addr_as_paused(ce_ptr->am_addr,
                                        ce_ptr->phy_piconet_id);
#endif /* COMPILE_NESTED_PAUSE_RESUME */

            return;
        }
#endif /* COMPILE_AFH_HOP_KERNEL */

        ce_ptr->pm_addr = 0;

        if(lmp_get_pm_addr(&pm_addr, ce_ptr->phy_piconet_id) == API_FAILURE)
        {
            LMP_ERR(ERROR_GETTING_PM_ADDRESS,0,0);
        }

        ce_ptr->pm_addr = pm_addr ;
        ce_ptr->ar_addr = 0;

        if(lmp_get_ar_addr(&(ce_ptr->ar_addr), ce_ptr->phy_piconet_id) != API_SUCCESS)
        {
            LMP_ERR(ERROR_GETTING_AR_ADDRESS,0,0);
        }

        /*
         * Master generates beacon parameters, send it to the slave.
         * Slave validates the beacon parameters, If it is acceptable, it
         * sends LMP_ACCEPTED. otherwise it will send LMP_NOT_ACCEPTED.
         */
        lmp_generate_beacon_parameters(ce_index);

        if (ce_ptr->Tbeacon > otp_str_data.bt_beacon_max_interval)
        {
            ce_ptr->Tbeacon = otp_str_data.bt_beacon_max_interval;
        }
        else if (ce_ptr->Tbeacon < otp_str_data.bt_beacon_min_interval)
        {
            ce_ptr->Tbeacon = otp_str_data.bt_beacon_min_interval;
        }

        lc_get_clock_in_scatternet(&clock_val, ce_ptr->phy_piconet_id);

        /* Timing control flag */
        /* Timing change */
        timing_control_flag = 0x00;
        if(clock_val & 0x08000000)
        {
            /* If MSB of master clock is 1 use initialization 2 */
            timing_control_flag |= 0x02;
        }

        /* Access window is persent */
        ce_ptr->timing_control_flag = timing_control_flag;
        /*Debacon value */
        ce_ptr->Dbeacon = LMP_DBEACON;

        /* Dispatch the Park Req PDU */
        parameter_list[0] = LMP_PARK_REQ_OPCODE;
        parameter_list[2] = ce_ptr->timing_control_flag ;
        parameter_list[3] = LSB(ce_ptr->Dbeacon);
        parameter_list[4] = MSB(ce_ptr->Dbeacon);
        parameter_list[5] = LSB(ce_ptr->Tbeacon);
        parameter_list[6] = MSB(ce_ptr->Tbeacon);
        parameter_list[7] = ce_ptr->Nbeacon ;
        parameter_list[8] = ce_ptr->Delta_beacon ;
        parameter_list[9] = ce_ptr->pm_addr ;
        parameter_list[10] = ce_ptr->ar_addr ;
        parameter_list[11] = ce_ptr->NB_sleep ;
        parameter_list[12] = ce_ptr->DB_sleep ;
        parameter_list[13] = ce_ptr->D_access ;
        parameter_list[14] = ce_ptr->T_access ;
        parameter_list[15] = ce_ptr->N_acc_slots ;
        parameter_list[16] = ce_ptr->N_poll ;
        parameter_list[17] = (UCHAR)((ce_ptr->access_scheme << 4) |
                                     ce_ptr->M_access);

        ce_ptr->lmp_expected_pdu_opcode = BIT_MASK_NO_PDU;
        lmp_generate_pdu(ce_index, parameter_list, LMP_PARK_REQ_LEN,
                         (LMP_TRAN_ID)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr),
                         LMP_PARK_MODE_REQ);

#ifdef COMPILE_NESTED_PAUSE_RESUME
        aclq_mark_am_addr_as_paused(ce_ptr->am_addr,
                                    ce_ptr->phy_piconet_id, ACL_PAUSED_PARK);
#else /* COMPILE_NESTED_PAUSE_RESUME */
        aclq_mark_am_addr_as_paused(ce_ptr->am_addr,
                                    ce_ptr->phy_piconet_id);
#endif /* COMPILE_NESTED_PAUSE_RESUME */
    }

    return;
}

/**
 * Handles LMP_unpark_PM_ADDR_req pdu from the remote device.
 *
 * \param lmp_pdu_ptr Pointer to LMP PDU.
 * \param piconet_id Physical Piconet ID.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_unpark_PM_ADDR_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
                                        UINT16 phy_piconet_id)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    UCHAR am_addr;
    UCHAR pm_addr;
    UINT16 ce_index;
    UCHAR tc_flag;
    UCHAR offset = 0;

    offset++;
    /* Read timing control flags*/
    tc_flag = lmp_pdu_ptr->payload_content[offset];
    offset++;

    /* Check zero bit is 1 or 0 */
    if(tc_flag & 0x01)
    {
        LMP_INF(UNPARK_PM_ADDR_REQ_DB_IS_PRESENT,0,0);
        offset += 2 ; /* opcode + TcFlags + Db */
    }

    /* Check the received broadcast packet. */
    pm_addr = lmp_pdu_ptr->payload_content[offset+1];

//	LMP_INF(FIRST_PM_ADDR_RECD_PICONET_ID, 2, pm_addr, piconet_id);

    if(LMP_GET_CE_INDEX_FROM_PM_ADDR(pm_addr, &ce_index, phy_piconet_id)
            != API_SUCCESS)
    {
        pm_addr = lmp_pdu_ptr->payload_content[offset+2];

        if(LMP_GET_CE_INDEX_FROM_PM_ADDR(pm_addr, &ce_index, phy_piconet_id)
                != API_SUCCESS)
        {
            LMP_ERR(PM_ADDR_TO_CE_INDEX_FAILED_PICONET_ID,2,pm_addr, piconet_id);
            return BT_FW_ERROR;
        }
        else
        {
            am_addr = (UCHAR)((lmp_pdu_ptr->payload_content[offset] >> 4)
                              & 0x07);
        }
    }
    else
    {
        am_addr = (UCHAR)(lmp_pdu_ptr->payload_content[offset] & 0x07);
    }

    ce_ptr = &lmp_connection_entity[ce_index];

    LMP_INF(VALID_PM_ADDR_RECD_PM_ADDR_AM_ADDR, 2,pm_addr, am_addr);

    if (ce_ptr->ce_status != LMP_PARK_MODE)
    {
        return BT_FW_SUCCESS;
    }

    if (am_addr != 0)
    {
        ce_ptr->am_addr = am_addr;
        /* Allocate am addr */
        if ((am_addr != 0) && (am_addr < LC_MAX_AM_ADDR))
        {
            lmp_slave_use_am_addr_ppi(am_addr, ce_ptr->phy_piconet_id, ce_index);
        }
    }
    else
    {
        LMP_LOG_INFO(LOG_LEVEL_HIGH,AVOID_AM_ADDR_IS_BECOMING_ZERO,0,0);
    }
    /*
     * LC module will program the LUT extension table with the NULL
     * packet first and then send the LMP_ACCEPTED PDU.
     */

    lmp_self_device_data.unpark_op_code = LMP_UNPARK_PM_ADDR_REQ_OPCODE;

    LMP_INF(UNPARKING_THE_DEVICE,0,0);
    lc_handle_unpark(ce_index);

    return BT_FW_SUCCESS;
}

/**
 * Handles LMP_unpark_BD_ADDR_req pdu from the remote device.
 *
 * \param lmp_pdu_ptr Pointer to the LMP PDU.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_unpark_BD_ADDR_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    UCHAR am_addr;
    UINT16 ce_index = 0;
    UCHAR bd_addr[LMP_BD_ADDR_SIZE];
    UCHAR offset = 0;
    UCHAR tc_flag;

    offset++;
    /* Read timing control flags*/
    tc_flag = lmp_pdu_ptr->payload_content[offset];
    offset++;

    /*Check zero bit is 1 or 0 */
    /* If the zero bit is 1 DB is present, otherwise it is optional */
    if(tc_flag & 0x01)
    {
        LMP_INF(UNPARK_PM_ADDR_REQ_DB_IS_PRESENT,0,0);
        offset += 2 ; //opcode + TcFlags + Db
    }

    /*
     * Validate the received broadcast packet.
     */
    memcpy(bd_addr, &(lmp_pdu_ptr->payload_content[offset+1]),
           LMP_BD_ADDR_SIZE);
    if(memcmp(otp_str_data.bt_bd_addr, bd_addr, LMP_BD_ADDR_SIZE)!= 0)
    {
        memcpy(bd_addr, &(lmp_pdu_ptr->payload_content[offset+7]),
               LMP_BD_ADDR_SIZE);
        if(memcmp(otp_str_data.bt_bd_addr, bd_addr, LMP_BD_ADDR_SIZE) != 0)
        {
            return BT_FW_SUCCESS;
        }
        else
        {
            am_addr = (UCHAR)((lmp_pdu_ptr->payload_content[offset] >> 4)
                              & 0X07);
        }
    }
    else
    {
        am_addr = (UCHAR)(lmp_pdu_ptr->payload_content[offset] & 0x07);
    }
    {
        UINT32 i;
        for(i = 0; i < LMP_MAX_CE_DATABASE_ENTRIES; i++)
        {
            if(lmp_connection_entity[i].ce_status == LMP_PARK_MODE &&
                    lmp_connection_entity[i].entity_status == ASSIGNED &&
                    lmp_connection_entity[i].remote_dev_role == MASTER)
            {
                ce_index = (UINT16)i;
                break;
            }
        }
        if(i == LMP_MAX_CE_DATABASE_ENTRIES)
        {
            LMP_LOG_ERROR(LOG_LEVEL_HIGH, UNABLE_TO_FIND_THE_CE_INDEX,0,0);
            return BT_FW_SUCCESS;
        }
    }

    ce_ptr = &lmp_connection_entity[ce_index];

    if (ce_ptr->ce_status != LMP_PARK_MODE)
    {
        return BT_FW_SUCCESS;
    }

    ce_ptr->am_addr = am_addr;

    if ((am_addr != 0) && (am_addr < LC_MAX_AM_ADDR))
    {
        lmp_slave_use_am_addr_ppi(am_addr, ce_ptr->phy_piconet_id, ce_index);
    }

    /*
     * Lc module will program the LUT extension table with the NULL
     * packet first and then send the LMP_ACCEPTED PDU.
     */
    lmp_self_device_data.unpark_op_code = LMP_UNPARK_BD_ADDR_REQ_OPCODE;

    lc_handle_unpark(ce_index);

    return BT_FW_SUCCESS;
}
#endif /* COMPILE_PARK_MODE */

#ifndef _CCH_REMOVE_USELESS_FUNC_
/**
 * Sends LMP_page_mode pdu to the remote device.
 *
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_send_page_mode_pdu(UINT16 ce_index)
{
    UCHAR parameter_list[LMP_PAGE_MODE_REQ_LEN];
    UCHAR features;

    features = lmp_feature_data.feat_page0[2];

    if(features & LMP_PAGE_SCHEME_FEATURE)
    {
        /* Here Update lmp_connection_entity database only */
        lmp_connection_entity[ce_index].optional_page_scheme =
            OPTIONAL_PAGING_SCHEME_1;
        if(lmp_connection_entity[ce_index].page_scan_repetition_mode == 0)
        {
            lmp_connection_entity[ce_index].optional_page_setting =
                PAGING_SCHEME_SETTING_R1;
        }
        else
        {
            lmp_connection_entity[ce_index].optional_page_setting =
                lmp_connection_entity[ce_index].page_scan_repetition_mode ;
        }
        parameter_list[0] = LMP_PAGE_MODE_REQ_OPCODE;
        parameter_list[2] = lmp_connection_entity[ce_index].optional_page_scheme;
        parameter_list[3] = lmp_connection_entity[ce_index].optional_page_setting;

        lmp_generate_pdu(ce_index, parameter_list, LMP_PAGE_MODE_REQ_LEN,
                         SELF_DEV_TID, LMP_NO_STATE_CHANGE);
    }
    else
    {
        lmp_send_lmp_not_accepted(ce_index,LMP_PAGE_MODE_REQ_OPCODE,
                                  REMOTE_DEV_TID,UNSUPPORTED_REMOTE_FEATURE_ERROR);

    }

    return BT_FW_SUCCESS;
}
#endif

/**
 * Handles the LMP_page_mode_req pdu from the remote device.
 *
 * \param lmp_pdu_ptr Pointer to LMP PDU.
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_page_mode_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index)
{
    DEVICE_INFO dev_info ;
    UCHAR param_valid = TRUE;
    UCHAR paging_scheme;
    UCHAR paging_scheme_settings;
    UCHAR features;

    features = lmp_feature_data.feat_page0[2];

    if(features & LMP_PAGE_SCHEME_FEATURE)
    {
        paging_scheme = lmp_pdu_ptr->payload_content[1];
        paging_scheme_settings = lmp_pdu_ptr->payload_content[2];

        if(paging_scheme == 0)
        {
            if(paging_scheme_settings >= 3)
            {
                LMP_INF(INVALID_PAGING_SCHEME_SETTINGS_FOR_MANDATORY_PAGING_SCHEME,0,0);
                param_valid = FALSE;
            }
        }
        else
        {
            LMP_INF(INVALID_PAGING_SCHEME,1,paging_scheme);
            param_valid = FALSE;
        }

        if((lmp_feature_data.feat_page0[2]  & LMP_PAGE_SCHEME_FEATURE ) &&
                (param_valid == TRUE) &&
                (lmp_self_device_data.opt_page_scan_flag == TRUE))
        {
            lmp_send_lmp_accepted(ce_index,LMP_PAGE_MODE_REQ_OPCODE,
                                  REMOTE_DEV_TID, LMP_NO_STATE_CHANGE);

            /* Update the device info table. */
            memcpy(dev_info.bd_addr, lmp_connection_entity[ce_index].bd_addr,
                   LMP_BD_ADDR_SIZE);
            dev_info.page_mode = paging_scheme;
            dev_info.page_mode_settings = paging_scheme_settings ;
            lmp_add_to_device_cache(dev_info);
        }
        else
        {
            lmp_send_lmp_not_accepted(ce_index,LMP_PAGE_MODE_REQ_OPCODE,
                                      REMOTE_DEV_TID, UNSUPPORTED_REMOTE_FEATURE_ERROR);
        }
    }
    else
    {
        lmp_send_lmp_not_accepted(ce_index,LMP_PAGE_MODE_REQ_OPCODE,
                                  REMOTE_DEV_TID,UNSUPPORTED_REMOTE_FEATURE_ERROR);

    }

    return BT_FW_SUCCESS;
}

/**
 * Handles the LMP_page_scan_mode_req pdu from the remote device.
 *
 * \param lmp_pdu_ptr Pointer to LMP PDU.
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR  lmp_handle_page_scan_mode_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index)
{
    UCHAR paging_scheme;
    UCHAR paging_scheme_settings;
    UCHAR param_valid = TRUE;
    UCHAR features;

    paging_scheme = lmp_pdu_ptr->payload_content[1];
    paging_scheme_settings = lmp_pdu_ptr->payload_content[2];

    features = lmp_feature_data.feat_page0[2];

    if(features & LMP_PAGE_SCHEME_FEATURE)
    {
        if(paging_scheme == 0)
        {
            if(paging_scheme_settings >= 3)
            {
                LMP_INF(INVALID_PAGING_SCHEME_SETTINGS_FOR_MANDATORY_PAGING_SCHEME,0,0);
                param_valid = FALSE;
            }
        }
        else
        {
            LMP_INF(INVALID_PAGING_SCHEME,1,paging_scheme);
            param_valid = FALSE;
        }

        /* When Remote Device sends lmp_page_mode_req, It means that
         * next time remote device will page then local device have to
         * be in optional page scan mode.
         * Check the self device features supports optioanl paging and the host
         * has issued optional page scan mode, then send LMP_ACCEPTED otherwise
         * send LMP_NOT_ACCEPTED */

        if((lmp_feature_data.feat_page0[2]  & LMP_PAGE_SCHEME_FEATURE )
                &&( param_valid == TRUE))
        {
            lmp_send_lmp_accepted(ce_index,LMP_PAGE_SCAN_MODE_REQ_OPCODE,
                                  REMOTE_DEV_TID, LMP_NO_STATE_CHANGE);
        }
        else
        {
            lmp_send_lmp_not_accepted(ce_index,LMP_PAGE_SCAN_MODE_REQ_OPCODE,
                                      REMOTE_DEV_TID, UNSUPPORTED_REMOTE_FEATURE_ERROR);
        }
    }
    else
    {
        lmp_send_lmp_not_accepted(ce_index,LMP_PAGE_SCAN_MODE_REQ_OPCODE,
                                  REMOTE_DEV_TID,UNSUPPORTED_REMOTE_FEATURE_ERROR);

    }

    return BT_FW_SUCCESS;
}

/**
 * Handles the LMP_qos pdu from the remote device.
 *
 * \param lmp_pdu_ptr Pointer to LMP PDU.
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_qos(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    if(ce_ptr->remote_dev_role == MASTER)
    {
        /* Update connection entity.*/
        ce_ptr->Tpoll = lmp_pdu_ptr->payload_content[1] |
                       (lmp_pdu_ptr->payload_content[2] << 8);
        ce_ptr->num_of_BC = lmp_pdu_ptr->payload_content[3];
        LMP_INF(NEW_TPOLL_NEW_NBC,2,ce_ptr->Tpoll, ce_ptr->num_of_BC);

        /* Send QOS setup complete event...*/
        hci_generate_QoS_setup_complete_event(
            HCI_COMMAND_SUCCEEDED,
            ce_index);
    }

    return BT_FW_SUCCESS;
}

/**
 * Handles the LMP_qos_req pdu from the remote device.
 *
 * \param lmp_pdu_ptr Pointer to LMP PDU.
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_qos_req(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index)
{
    UINT16 Tpoll,super_to ;
    UCHAR num_of_BC,trans_id,reason = 0 ;

    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    trans_id = (UCHAR)(lmp_pdu_ptr->payload_content[0] & 0x01);
    Tpoll = lmp_pdu_ptr->payload_content[1] |
            (lmp_pdu_ptr->payload_content[2] << 8);
    num_of_BC = lmp_pdu_ptr->payload_content[3];
    LMP_INF(NEW_TPOLL_NEW_NBC,2,Tpoll,num_of_BC);

    switch(ce_ptr->ce_status)
    {
        case LMP_CONNECTED : /* Fall through. */
#ifdef COMPILE_SNIFF_MODE
        case LMP_SNIFF_MODE : /* Fall through. */
#endif
            break ;

        default :
            lmp_send_lmp_not_accepted(ce_index, LMP_QoS_REQ_OPCODE,
                                      trans_id, PDU_NOT_ALLOWED_ERROR );
            return BT_FW_ERROR;
    }

    if ( (Tpoll == 0x0) || (Tpoll & 0x1) )
    {
        reason = QOS_UNACCEPTABLE_PARAMETER;
    }

    /* Check for sup To */
    super_to = ce_ptr->link_supervision_timeout;

    if((super_to != 0) && (Tpoll >= super_to))
    {
        reason = QOS_UNACCEPTABLE_PARAMETER;
    }

    if ( (Tpoll > LMP_MAX_TPOLL) || (Tpoll < LMP_MIN_TPOLL) )
    {
        reason = QOS_UNACCEPTABLE_PARAMETER;
    }

    RT_BT_LOG(WHITE, LMP_QOS_REQ_ERR_MSG, 5, Tpoll, num_of_BC,
              super_to, reason, ce_ptr->remote_dev_role);

    /* Master is informing the slave with the new Tpoll value. */
    if (ce_ptr->remote_dev_role == SLAVE)
    {
        if(reason != 0)
        {
            /* Failure case. */
            lmp_send_lmp_not_accepted(ce_index, LMP_QoS_REQ_OPCODE,
                                      trans_id, reason );
            return BT_FW_SUCCESS;
        }
        else
        {
            /* Adjust local Tpoll appropriately, */
            ce_ptr->qos_tpoll = Tpoll;
            if(super_to != 0)
            {
                if(Tpoll > (super_to >> 3))
                {
                    Tpoll = (UINT16) (super_to >> 3);
                }
            }

            if (Tpoll >= 4*3)
            {
                Tpoll -= 8;
            }

            if (Tpoll < LMP_MIN_TPOLL)
            {
                Tpoll = LMP_MIN_TPOLL;
            }

            ce_ptr->Tpoll = Tpoll;
        }

#ifdef COMPILE_SNIFF_MODE
        if(ce_ptr->in_sniff_mode == FALSE)
#endif
        {
            BB_start_tpoll(ce_ptr->am_addr, Tpoll, ce_ptr->phy_piconet_id);
        }
    }

    ce_ptr->Tpoll = Tpoll ;
    ce_ptr->num_of_BC = num_of_BC ;
    lmp_send_lmp_accepted(ce_index, LMP_QoS_REQ_OPCODE,
                          trans_id, LMP_NO_STATE_CHANGE);

    /* Send QOS setup complete event to the host...*/
    hci_generate_QoS_setup_complete_event(
        HCI_COMMAND_SUCCEEDED,
        ce_index);

    return BT_FW_SUCCESS;
}

/**
 * Handles the LMP_timing_accuracy_req pdu from the remote device.
 *
 * \param lmp_pdu_ptr Pointer to LMP PDU.
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_timing_accuracy_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index)
{
    UCHAR parameter_list[LMP_TIMING_ACCURACY_RES_LEN];

    /* Check local and remote device supports timing accuracy feature, if
    not send not accepted PDU with reason unsupported feature.*/

    if((lmp_feature_data.feat_page0[0]  & LMP_TIME_ACCURACY_FEATURE)&&
            (lmp_connection_entity[ce_index].feat_page0[0] &
             LMP_TIME_ACCURACY_FEATURE))
    {
        /* Send timing accuracy response PDU */
        parameter_list[0] = LMP_TIMING_ACCURACY_RES_OPCODE;
        parameter_list[2] = LMP_MAX_DRIFT;
        parameter_list[3] = LMP_MAX_JITTER;
        lmp_generate_pdu(ce_index, parameter_list, LMP_TIMING_ACCURACY_RES_LEN,
                         REMOTE_DEV_TID, LMP_NO_STATE_CHANGE);
    }
    else
    {
        lmp_send_lmp_not_accepted(ce_index,LMP_TIMING_ACCURACY_REQ_OPCODE,
                                  REMOTE_DEV_TID,
                                  UNSUPPORTED_REMOTE_FEATURE_ERROR);
    }
    return BT_FW_SUCCESS ;
}

/**
 * Handles the LMP_timing_accuracy_res pdu from the remote device.
 *
 * \param lmp_pdu_ptr Pointer to LMP PDU.
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_timing_accuracy_res_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index)
{
    UCHAR remote_device_drift;

    remote_device_drift = lmp_pdu_ptr->payload_content[1];

    if (remote_device_drift <= 250)
    {
        lmp_connection_entity[ce_index].remote_max_drift =
            lmp_pdu_ptr->payload_content[1];
    }

    return BT_FW_SUCCESS ;
}

#if defined(POWER_CONTROL)
/**
 * Handles the LMP_decr_power_req pdu from the remote device.
 *
 * \param lmp_pdu_ptr Pointer to LMP PDU.
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_decr_power_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index)
{
    UCHAR parameter_list[LMP_NOT_ACCEPTED_LEN];
    UCHAR am_addr;
    UINT16 tx_gain_value;
    UCHAR send_min_power_pdu_flag = FALSE;
    LMP_CONNECTION_ENTITY *ce_ptr;
    UCHAR phy_piconet_id;

    ce_ptr = &lmp_connection_entity[ce_index];
    ce_ptr->inc_pow_pdu_drop_flag = FALSE;

#ifdef POWER_CONTROL

    phy_piconet_id  = lmp_pdu_ptr->piconet;

    am_addr = ce_ptr->am_addr;

    tx_gain_value = lc_get_current_power_value(am_addr,
                    ce_ptr->phy_piconet_id);

    /*
    * Check local device supports power control, If the local device doesen't
    * support send not accepted PDU with reason unsupported feature
    */
    if(lmp_feature_data.feat_page0[2] & LMP_POWER_CONTROL_FEATURE)
    {
        if(LC_CHECK_FOR_DECR_POWER_LEVEL((UCHAR)tx_gain_value))
        {
            lc_program_tx_power(am_addr, ce_index,
                                LC_TX_POWER_LEVEL_DECREMENT, tx_gain_value);

#ifdef TEST_MODE
            test_mode_tx_mode_force_ack = TRUE;

#ifdef _RTL8723B_DUT_MODE_   
            if (lc_is_tx_test_mode)
            {
                /* re-enable tx mode */
                BB_write_baseband_register(RADIO_SELECT_REGISTER, 
                                           test_mode_tx_mode_reg_catch);                        
            }
            else
            {
                /* re-enable loopback mode */
                BB_write_baseband_register(RADIO_SELECT_REGISTER, 0x0127);                    
            }
#endif
#endif
        }
        else
        {
            /* Send MIN power PDU */
            send_min_power_pdu_flag = TRUE;
        }
    }
    else
    {
        lmp_send_lmp_not_accepted(ce_index,LMP_DECR_POWER_REQ_OPCODE,
                                  REMOTE_DEV_TID,
                                  UNSUPPORTED_REMOTE_FEATURE_ERROR);

#ifdef TEST_MODE
        test_mode_tx_mode_force_ack = TRUE;
#endif
    }
#endif /* POWER_CONTROL */

    if ( (send_min_power_pdu_flag == TRUE) &&
            (ce_ptr->dec_pow_pdu_drop_flag == FALSE) )
    {
#ifdef TEST_MODE
        /* Check if the device is in test-mode. */
        if (ce_ptr->test_mode_info.test_state == TEST_STARTED)
        {
            lc_tci_pause_flag = FALSE;
        }
#endif

        parameter_list[0] = LMP_MIN_POWER_OPCODE;

        lmp_generate_pdu(ce_index, parameter_list, LMP_MIN_POWER_LEN,
                         REMOTE_DEV_TID, LMP_NO_STATE_CHANGE);
        ce_ptr->dec_pow_pdu_drop_flag = TRUE;
    }

    return BT_FW_SUCCESS ;
}

/**
 * Handles the LMP_incr_power_req pdu from the remote device.
 *
 * \param lmp_pdu_ptr Pointer to LMP PDU.
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_incr_power_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index)
{
    UCHAR parameter_list[LMP_NOT_ACCEPTED_LEN];
    UCHAR am_addr ;
    UINT16 tx_gain_value ;
    UCHAR send_max_power_pdu_flag = FALSE;
    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];
    ce_ptr->dec_pow_pdu_drop_flag = FALSE;

#ifdef POWER_CONTROL
    am_addr = ce_ptr->am_addr ;

    tx_gain_value = lc_get_current_power_value(am_addr,
                    ce_ptr->phy_piconet_id);

    if(lmp_feature_data.feat_page0[2] & LMP_POWER_CONTROL_FEATURE)
    {
        if(LC_CHECK_FOR_INCR_POWER_LEVEL((UCHAR)tx_gain_value))
        {
            lc_program_tx_power(am_addr, ce_index,
                                LC_TX_POWER_LEVEL_INCREMENT, tx_gain_value);

#ifdef TEST_MODE
            test_mode_tx_mode_force_ack = TRUE;

#ifdef _RTL8723B_DUT_MODE_   
            if (lc_is_tx_test_mode)
            {
                /* re-enable tx mode */
                BB_write_baseband_register(RADIO_SELECT_REGISTER, 
                                           test_mode_tx_mode_reg_catch);                        
            }
            else
            {
                /* re-enable loopback mode */
                BB_write_baseband_register(RADIO_SELECT_REGISTER, 0x0127);                    
            }            
#endif
#endif
        }
        else
        {
            /* Send Max power PDU */
            send_max_power_pdu_flag = TRUE;
        }
    }
    else /* if(lmp_feature_data */
    {
        lmp_send_lmp_not_accepted(ce_index, LMP_INCR_POWER_REQ_OPCODE,
                                  REMOTE_DEV_TID, UNSUPPORTED_REMOTE_FEATURE_ERROR);

#ifdef TEST_MODE
        test_mode_tx_mode_force_ack = TRUE;
#endif
        return BT_FW_SUCCESS;
    }
#endif /* POWER_CONTROL */

    if ( (send_max_power_pdu_flag == TRUE) &&
            (ce_ptr->inc_pow_pdu_drop_flag == FALSE) )
    {
#ifdef TEST_MODE
        /* Check if the device is in test-mode. */
        if (ce_ptr->test_mode_info.test_state == TEST_STARTED)
        {          
            lc_tci_pause_flag = FALSE;
        }
#endif
        {
            parameter_list[0] = LMP_MAX_POWER_OPCODE;
            lmp_generate_pdu(ce_index, parameter_list, LMP_MAX_POWER_LEN,
                             REMOTE_DEV_TID, LMP_NO_STATE_CHANGE);
            ce_ptr->inc_pow_pdu_drop_flag = TRUE;

        }
    }

    return BT_FW_SUCCESS ;
}

/**
 * Handles LMP_min_power pdu from the remote device.
 *
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_min_power_pdu(UINT16 ce_index)
{
#ifdef POWER_CONTROL
    lmp_connection_entity[ce_index].power_ctrl_resp = LMP_MIN_POWER_RCVD;
#endif /* POWER_CONTROL */

    return BT_FW_SUCCESS ;
}

/**
 * Handles LMP_max_power pdu from the remote device.
 *
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_max_power_pdu(UINT16 ce_index)
{

#ifdef POWER_CONTROL
    lmp_connection_entity[ce_index].power_ctrl_resp = LMP_MAX_POWER_RCVD;
#endif /* POWER_CONTROL */

    return BT_FW_SUCCESS ;
}
#endif /* (POWER_CONTROL) */

#ifdef TEST_MODE
/**
 * Handles the LMP_test_activate pdu from the remote device.
 *
 * \param lmp_pdu_ptr Pointer to LMP PDU.
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_test_activate_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index)
{
    UCHAR parameter_list[LMP_NOT_ACCEPTED_LEN];

    //RT_BT_LOG(GRAY, LMP_PDU_5116, 0, 0);

    /* Changed the condition. Perviously this was
     * lmp_connection_entity[ce_index].test_mode_info.test_state != TEST_STARTED.
     * But we have to actually check if this is equal to TEST_INITIALIZED.
     */

    if((lmp_connection_entity[ce_index].ce_status != LMP_CONNECTED) ||
            (lmp_self_device_data.host_enable_test_mode != TRUE) ||
            (lmp_connection_entity[ce_index].remote_dev_role != MASTER) ||
            (lmp_connection_entity[ce_index].test_mode_info.test_state != TEST_INITIALIZED))
    {
        /* Generate Lmp_Not_Accepted pdu */
        parameter_list[0] = LMP_NOT_ACCEPTED_OPCODE;
        parameter_list[2] = LMP_TEST_ACTIVATE_OPCODE;
        parameter_list[3] = PDU_NOT_ALLOWED_ERROR;

        //RT_BT_LOG(GRAY, LMP_PDU_5133, 0, 0);

        lmp_generate_pdu(ce_index, parameter_list, LMP_NOT_ACCEPTED_LEN,
                         REMOTE_DEV_TID, LMP_NO_STATE_CHANGE);

        return BT_FW_SUCCESS;
    }

#ifdef TEST_MODE
    if (lmp_self_device_data.test_mode == HCI_NO_LOOPBACK_MODE)
    {
        pf_switch_hci_dma_parameter(ENTER_TEST_MODE);
    }
#endif

    /* Change state to TEST_MODE */
    lmp_self_device_data.stored_test_mode_state = lmp_self_device_data.test_mode;
    lmp_self_device_data.test_mode = HCI_DEVICE_UNDER_TEST_MODE;

    /* Generate Lmp_Accepted pdu */
    parameter_list[0] = LMP_ACCEPTED_OPCODE;
    parameter_list[2] = LMP_TEST_ACTIVATE_OPCODE ;

    //RT_BT_LOG(GRAY, LMP_PDU_5149, 0, 0);

    lmp_generate_pdu(ce_index, parameter_list, LMP_ACCEPTED_LEN,
                     REMOTE_DEV_TID, LMP_NO_STATE_CHANGE);

    lmp_connection_entity[ce_index].test_mode_info.test_state = TEST_ACTIVATED;

    return BT_FW_SUCCESS;
}

/**
 * Timeout handler for the LMP TCI TIMER. It gives enough time for any packets
 * that are already queued in the baseband to be transmitted over the air.
 *
 * \param timer_handle Timer handle.
 * \param arg User data given during the creation of the timer.
 *
 * \return None.
 */
void tci_lmp_timeout_handler(TimerHandle_t timer_handle)
{
    UINT16 ce_index;
    LMP_PDU_PKT *lmp_pdu_ptr;
    TEST_CONTROL_PARAMS *tcp;
    UCHAR parameter_list[LMP_NOT_ACCEPTED_LEN];
    UCHAR test_scenario;
    UCHAR hoping_mode;
    UCHAR tx_frequency;
    UCHAR rx_frequency;
    UCHAR pwr_control_mode;
    UINT16 packet_type;
    UINT16 pkt_desc;
    UCHAR poll_period;
    UINT16 test_sequence_len;
#if defined(COMPILE_ESCO)
    UINT16 num_of_bytes;
#endif

    /* Hardcoded ce_index as 0 */
    ce_index = 0;

    lmp_pdu_ptr = (LMP_PDU_PKT *) pvTimerGetTimerID(timer_handle);

    /* Delete the timer. */
    OS_DELETE_TIMER(&lmp_tci_timer);
     
    if( (lmp_connection_entity[ce_index].ce_status != LMP_CONNECTED) ||
            (lmp_connection_entity[ce_index].remote_dev_role != MASTER) )
    {
        lmp_test_mode_generate_not_accepted_pdu(lmp_pdu_ptr, PDU_NOT_ALLOWED_ERROR);
        //RT_BT_LOG(GRAY, LMP_PDU_5209, 0, 0);

        /* Free the buffer */
        if(OS_FREE_BUFFER(lmp_pdu_buffer_pool_handle, lmp_pdu_ptr) != BT_ERROR_OK)
        {
            RT_BT_LOG(GRAY, LMP_PDU_5214, 0, 0);
        }
        return;
    }

    switch(lmp_self_device_data.test_mode)
    {
            /*
             * Allow the Test control pdu only in one of these states. This state
             * will be set when the local host gives ENABLE DUT hci command.
             */
        case HCI_DEVICE_UNDER_TEST_MODE:
            /* Fall Thru */
            /*
             * This state will be set when the slave recevies an LMP Test control pdu
             * with the test scenario set to one of the loopback modes - ACL loopback,
             * ACL loopback w/o Whitening, SCO loopback or SCO loopback w/o Whitening.
             */
        case HCI_DUT_LOOPBACK_MODE:
            break;

        default:
            /* Generate Lmp_Not_Accepted pdu  in any other state */
            lmp_test_mode_generate_not_accepted_pdu(lmp_pdu_ptr, PDU_NOT_ALLOWED_ERROR);

            /* Free the buffer */
            if(OS_FREE_BUFFER(lmp_pdu_buffer_pool_handle, lmp_pdu_ptr) != BT_ERROR_OK)
            {
                RT_BT_LOG(GRAY, LMP_PDU_5242, 0, 0);
            }

            return;
    }

    /* Test Control PDU  is allowed only the TEST STARTED & TEST ACTIVATED */
    switch(lmp_connection_entity[ce_index].test_mode_info.test_state)
    {
        case TEST_STARTED:
            /* Fall thru */
        case TEST_ACTIVATED:
            break;

        default:
            /* Generate Lmp_Not_Accepted pdu  in any other state */
            lmp_test_mode_generate_not_accepted_pdu(lmp_pdu_ptr, PDU_NOT_ALLOWED_ERROR);

            /* Free the buffer */
            if(OS_FREE_BUFFER(lmp_pdu_buffer_pool_handle, lmp_pdu_ptr) != BT_ERROR_OK)
            {
                RT_BT_LOG(GRAY, LMP_PDU_5263, 0, 0);
            }
            return;
    }

    tcp = &lmp_connection_entity[ce_index].test_mode_info.tc_params;

    /*
    * Test Control PDU will be sent with all params XOR'ed with 0x55.
    * XOR the pdu parameters with the whitening value (0x55) to get the original values.
    */
    lmp_pdu_ptr->payload_content[1] ^= WHITENING_VALUE;
    lmp_pdu_ptr->payload_content[2] ^= WHITENING_VALUE;
    lmp_pdu_ptr->payload_content[3] ^= WHITENING_VALUE;
    lmp_pdu_ptr->payload_content[4] ^= WHITENING_VALUE;
    lmp_pdu_ptr->payload_content[5] ^= WHITENING_VALUE;
    lmp_pdu_ptr->payload_content[6] ^= WHITENING_VALUE;
    lmp_pdu_ptr->payload_content[7] ^= WHITENING_VALUE;
    lmp_pdu_ptr->payload_content[8] ^= WHITENING_VALUE;
    lmp_pdu_ptr->payload_content[9] ^= WHITENING_VALUE;

    /* Extract the pdu paramters */
    test_scenario     = lmp_pdu_ptr->payload_content[1];
    hoping_mode       = lmp_pdu_ptr->payload_content[2];
    tx_frequency      = lmp_pdu_ptr->payload_content[3];
    rx_frequency      = lmp_pdu_ptr->payload_content[4];
    pwr_control_mode  = lmp_pdu_ptr->payload_content[5];
    poll_period       = lmp_pdu_ptr->payload_content[6];
    packet_type       = lmp_pdu_ptr->payload_content[7] & 0x0f;
    pkt_desc          = (lmp_pdu_ptr->payload_content[7] >> 4) & 0x0f;
    test_sequence_len = lmp_pdu_ptr->payload_content[8] | 
                       (lmp_pdu_ptr->payload_content[9] << 8);

    tcp->test_scenario = test_scenario;
    tcp->hopping_mode = hoping_mode;
    tcp->tx_frequency = tx_frequency;
    tcp->rx_frequency = rx_frequency;
    tcp->power_control_mode = pwr_control_mode;
    tcp->poll_period = poll_period;
    tcp->packet_type_in_pdu = (UCHAR)packet_type;
    tcp->pkt_desc = (UCHAR)pkt_desc;
    tcp->num_packets = test_sequence_len;

#ifdef _RTL8723B_DUT_MODE_
    /* exit dut mode */
    BB_write_baseband_register(RADIO_SELECT_REGISTER, 0x0107);
#endif

    /* Added TEST_MODE_EXIT_TEST_MODE. There was no code to for this case previously.*/
    if ( (test_scenario == TEST_MODE_PAUSE_TEST_MODE) ||
            (test_scenario == TEST_MODE_EXIT_TEST_MODE) )
    {
        /* Generate Lmp_Accepted pdu */
        parameter_list[0] = LMP_ACCEPTED_OPCODE;
        parameter_list[2] = LMP_TEST_CONTROL_OPCODE ;

        lmp_generate_pdu(ce_index, parameter_list, LMP_ACCEPTED_LEN,
                         REMOTE_DEV_TID, LMP_NO_STATE_CHANGE);

        /* Free the buffer */
        if(OS_FREE_BUFFER(lmp_pdu_buffer_pool_handle, lmp_pdu_ptr) != BT_ERROR_OK)
        {
            RT_BT_LOG(GRAY, LMP_PDU_5334, 0, 0);
        }
        return;
    }

    if ((pkt_desc == PACKET_TYPE_DESCRIPTION_BR_ACL_SCO)
#ifdef COMPILE_ESCO
            || (pkt_desc == PACKET_TYPE_DESCRIPTION_BR_eSCO)
#endif
      )
    {
        /* Disable PTT in baseband. */
        lc_disable_ptt_bit(ce_index);
        LMP_LOG_INFO(LOG_LEVEL_HIGH, DUT_DISABLED_PTT,0,0);
    }

    if ((pkt_desc == PACKET_TYPE_DESCRIPTION_EDR_ACL)
#ifdef COMPILE_ESCO
            || (pkt_desc == PACKET_TYPE_DESCRIPTION_EDR_eSCO)
#endif
       )
    {
        /* Enable PTT in baseband. */
        lc_enable_ptt_bit(ce_index);
        LMP_LOG_INFO(LOG_LEVEL_HIGH, DUT_ENABLED_PTT,0,0);
    }

    /*
     * Store the packet type that came in the test control pdu. Used for sending event to
     * the host
     */
    tcp->packet_type_in_pdu = (UCHAR)packet_type;

    switch(pkt_desc)
    {
#ifdef COMPILE_ESCO
        case PACKET_TYPE_DESCRIPTION_BR_eSCO: /* Fall through. */
#endif

#ifdef COMPILE_ESCO
        case PACKET_TYPE_DESCRIPTION_EDR_eSCO:
        {
            DEF_CRITICAL_SECTION_STORAGE;

            MINT_OS_ENTER_CRITICAL();
            
            num_of_bytes = (UINT16) tcp->num_packets;

            /* Warning: amaddr 1 is hard-coded here. */
            BB_write_baseband_register(CONNECTOR_REGISTER, (UINT16) (0x0231));
            BB_write_baseband_register(ESCO_RX_LENGTH_TYPE_REGISTER,
                                       (UINT16) ( (packet_type << 12) | num_of_bytes) );
            BB_write_baseband_register(ESCO_FIFO_FLUSH_LENGTH_AND_TX_PKT_LEN_REGISTER,
                                       ((packet_type & 0x8) << 7));

            BB_write_baseband_register(ESCO_INTERVAL_REGISTER, (UINT16) (0x1200));
            BB_write_baseband_register(ESCO_WINDOW_DESCO_REGISTER, 0x0c06);

#ifdef ENABLE_SCO
            BB_write_baseband_register(SYNC_FIFO_CONFIG_REGISTER, 0x0001);
#endif
            MINT_OS_EXIT_CRITICAL();


            LMP_LOG_INFO(LOG_LEVEL_LOW, PKT_CONFIG_DONE,0,0);
        }
        break;
#endif /* COMPILE_ESCO */

        case PACKET_TYPE_DESCRIPTION_BR_ACL_SCO:
        case PACKET_TYPE_DESCRIPTION_EDR_ACL:
        default:
            break;

    }

    lmp_connection_entity[ce_index].pkts_allowed.status = 0;

    OR_val_with_bb_reg(ENCRYPTION_ENABLE_REGISTER, ENABLE_SCO_PKT_LOOPBACK_TX);

    /* Generate Lmp_Accepted pdu */
    parameter_list[0] = LMP_ACCEPTED_OPCODE;
    parameter_list[2] = LMP_TEST_CONTROL_OPCODE;

    lmp_generate_pdu(ce_index, parameter_list, LMP_ACCEPTED_LEN,
                     REMOTE_DEV_TID, LMP_NO_STATE_CHANGE);

    /* Reset the burst data generation flag to false */
    lmp_connection_entity[ce_index].test_mode_info.dut_burst_data_flag = FALSE;
    {
        switch(tcp->test_scenario)
        {
            case TEST_MODE_TRANSMITTER_TEST_ZERO_PATTERN:
                /* Fall through */
            case TEST_MODE_TRANSMITTER_TEST_ONE_PATTERN:
                /* Fall through */
            case TEST_MODE_TRANSMITTER_TEST_1010_PATTERN:
                /* Fall through */
            case TEST_MODE_PSEUDORANDOM_BIT_SEQUENCE:
                /* Fall through */
            case TEST_MODE_TRANSMITTER_TEST_11110000_PATTERN:
                lc_is_tx_test_mode = TRUE;
                break;

            default:
                lc_is_tx_test_mode = FALSE;
                break;
        }
    }
    lmp_self_device_data.test_mode = HCI_DEVICE_UNDER_TEST_MODE;
    lmp_connection_entity[ce_index].test_mode_info.test_state = TEST_STARTED;

    /* Free the buffer */
    if(OS_FREE_BUFFER(lmp_pdu_buffer_pool_handle, lmp_pdu_ptr) != BT_ERROR_OK)
    {
        RT_BT_LOG(GRAY, LMP_PDU_5522, 0, 0);
    }
}

/**
 * Handles the LMP_test_control pdu from the remote device.
 *
 * \param lmp_pdu_ptr Pointer to LMP PDU.
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_test_control_opcode(LMP_PDU_PKT *lmp_pdu_ptr,
                                     UINT16 ce_index)
{
    LMP_PDU_PKT  *lmp_pkt;

    /*
     * This flag will be set ONLY when we get the LC level ack for the lmp_accepted
     * that we have just sent.
     */
    lc_test_mode_ack_recd = FALSE;

    lc_tci_pause_flag = TRUE;

    /* Copy the lmp_pdu data to another array, as the pdu
    will be freed after returning from here. */
    OS_ALLOC_BUFFER(lmp_pdu_buffer_pool_handle, (void **)(&lmp_pkt));

    memcpy(lmp_pkt , lmp_pdu_ptr , sizeof(LMP_PDU_PKT) );

    /* Start a timer for the least resolution. Meanwhile the BB will
    complete the transmission of any pending packet.*/

    if (lmp_tci_timer == NULL)
    {
        OS_CREATE_TIMER(ONESHOT_TIMER, &lmp_tci_timer,
                tci_lmp_timeout_handler, (void *) lmp_pkt, 0);
    }
    
    /* Create timer with the min resolution */
    OS_START_TIMER(lmp_tci_timer, 15);

    aclq_clear_all_pkts_am_addr(lmp_connection_entity[ce_index].am_addr, FALSE,
                                lmp_connection_entity[ce_index].phy_piconet_id);

    return BT_FW_SUCCESS;
}


/**
 * Handles the test control accepted timer expiry.
 *
 * \param timer_handle The handle of the timer expired.
 * \param arg Arguements passed, if any;None in this case.
 *
 * \return None.
 */
void lmp_handle_test_control_timeout_handler(TimerHandle_t timer_handle)
{
    UINT16 ce_index = 0; /* Hardcoded ce_index as 0 */
    UINT16 sco_pkt_type_reg;
    UINT16 hoping_mode;
    UINT16 channel_reg;
    UINT16 tx_frequency;
    TEST_CONTROL_PARAMS *tcp;

    DEF_CRITICAL_SECTION_STORAGE;

    LMP_LOG_INFO(LOG_LEVEL_HIGH, TESTER_TIMER_FIRED,0,0);

    /* Post signal to delete the timer. */
    {
        OS_SIGNAL sig_send;
        sig_send.type = LMP_DELETE_TIMER_HANDLE_SIGNAL;
        sig_send.param =  (OS_ADDRESS) (lmp_tester_tci_timer);
        OS_SEND_SIGNAL_TO_TASK(lmp_task_handle, sig_send);

        lmp_tester_tci_timer = NULL;
    }

    tcp = &lmp_connection_entity[ce_index].test_mode_info.tc_params;

    if(tcp->test_scenario == TEST_MODE_PAUSE_TEST_MODE)
    {
        return;
    }

    if(tcp->test_scenario == TEST_MODE_EXIT_TEST_MODE)
    {
        /* Switch to 79 hops. */
        MINT_OS_ENTER_CRITICAL();
        sco_pkt_type_reg = BB_read_baseband_register(SCO_PACKET_TYPE_REGISTER);
        HOP_BITS_ISOLATE(sco_pkt_type_reg);
        sco_pkt_type_reg |= TEST_MODE_BASEBAND_HOP_79;
        BB_write_baseband_register(SCO_PACKET_TYPE_REGISTER, sco_pkt_type_reg);

        /* Enable whitening. */
        AND_val_with_bb_reg(SCO_PACKET_TYPE_REGISTER, (UINT16) (~0x0080));

        /* Restore PTT bit in the LUT. */
        if (lmp_connection_entity[ce_index].ptt_status == LMP_PTT_ENABLED)
        {
            lc_enable_ptt_bit(ce_index);
        }
        else
        {
            lc_disable_ptt_bit(ce_index);
        }
        
        MINT_OS_EXIT_CRITICAL();

        LMP_LOG_INFO(LOG_LEVEL_HIGH, DUT_79_HOPS_PROGRAMMED,0,0);
        LMP_LOG_INFO(LOG_LEVEL_HIGH, DUT_WHITENING_ENABLED,0,0);
        return;
    }

    hoping_mode = tcp->hopping_mode;

    switch(hoping_mode)
    {
        case TEST_MODE_SINGLE_FREQUENCY_MODE:
            MINT_OS_ENTER_CRITICAL();
            sco_pkt_type_reg =
                BB_read_baseband_register(SCO_PACKET_TYPE_REGISTER);

            /* Isolate the two bits that are used for hop mode */
            HOP_BITS_ISOLATE(sco_pkt_type_reg);
            sco_pkt_type_reg |= TEST_MODE_BASEBAND_HOP_1;

            BB_write_baseband_register(SCO_PACKET_TYPE_REGISTER,
                                       sco_pkt_type_reg);

            channel_reg = (UINT16) (tcp->rx_frequency);
            tx_frequency = (UINT16) (tcp->tx_frequency);
            tx_frequency = (UINT16) (tx_frequency << 8);
            channel_reg = (UINT16)  (tx_frequency | channel_reg);

            BB_write_baseband_register(CHANNEL_REGISTER, channel_reg);
            
            MINT_OS_EXIT_CRITICAL();

            LMP_LOG_INFO(LOG_LEVEL_HIGH,DUT_SINGLE_HOP_PROGRAMMED,1,channel_reg);
            break;

        case TEST_MODE_HOP_79:
            MINT_OS_ENTER_CRITICAL();
            sco_pkt_type_reg =
                BB_read_baseband_register(SCO_PACKET_TYPE_REGISTER);
            HOP_BITS_ISOLATE(sco_pkt_type_reg);
            sco_pkt_type_reg |= TEST_MODE_BASEBAND_HOP_79;
            BB_write_baseband_register(SCO_PACKET_TYPE_REGISTER,
                                       sco_pkt_type_reg);
            MINT_OS_EXIT_CRITICAL();

            LMP_LOG_INFO(LOG_LEVEL_HIGH, DUT_79_HOPS_PROGRAMMED,0,0);
            break;

        default:
            break;
    }

    if( (tcp->test_scenario == TEST_MODE_TRANSMITTER_TEST_ZERO_PATTERN) ||
        (tcp->test_scenario == TEST_MODE_TRANSMITTER_TEST_ONE_PATTERN) ||
        (tcp->test_scenario == TEST_MODE_TRANSMITTER_TEST_1010_PATTERN) ||
        (tcp->test_scenario == TEST_MODE_PSEUDORANDOM_BIT_SEQUENCE) ||
        (tcp->test_scenario == TEST_MODE_TRANSMITTER_TEST_11110000_PATTERN) )
    {
        /* Disable whitening. */
        OR_val_with_bb_reg(SCO_PACKET_TYPE_REGISTER, 0x0080);

        LMP_LOG_INFO(LOG_LEVEL_HIGH, DUT_WHITENING_DISABLED_FOR_TX_TEST,0,0);
    }
    else
    {
        UINT8 enable;
        if( (tcp->test_scenario == TEST_MODE_ACL_PKTS_WITHOUT_WHITENING) ||
            (tcp->test_scenario == TEST_MODE_SCO_PKTS_WITHOUT_WHITENING) )
        {
            /* Disable whitening. */
            OR_val_with_bb_reg(SCO_PACKET_TYPE_REGISTER, 0x0080);
            enable = 1;
        }
        else
        {
            /* Enable whitening. */
            AND_val_with_bb_reg(SCO_PACKET_TYPE_REGISTER, (UINT16)(~0x0080));
            enable = 0;
        }

        LMP_LOG_INFO(LOG_LEVEL_HIGH, DUT_WHITENING_SWITCH, 1, enable);
    }

    if (tcp->test_scenario == TEST_MODE_EXIT_TEST_MODE)
    {
        /* Program 79 hops */
        sco_pkt_type_reg = BB_read_baseband_register(SCO_PACKET_TYPE_REGISTER);

        /* Clear the two bits for the hop mode in this register.*/
        sco_pkt_type_reg  &= (~0x0060);

        /* Set the hop mode as 79. */
        sco_pkt_type_reg |= 0x20;
        BB_write_baseband_register(SCO_PACKET_TYPE_REGISTER, sco_pkt_type_reg);

        /* Enable whitening. */
        AND_val_with_bb_reg(SCO_PACKET_TYPE_REGISTER, 0xff7f);

        LMP_LOG_INFO(LOG_LEVEL_HIGH, DUT_79_HOPS_PROGRAMMED_EXIT_TESTMODE, 0,0);
    }

    if ((tcp->pkt_desc == PACKET_TYPE_DESCRIPTION_BR_ACL_SCO)
#ifdef COMPILE_ESCO
         || (tcp->pkt_desc == PACKET_TYPE_DESCRIPTION_BR_eSCO)
#endif
      )
    {
        /* Disable PTT in baseband. */
        lc_disable_ptt_bit(ce_index);
        LMP_LOG_INFO(LOG_LEVEL_HIGH, DISABLED_PTT,0,0);
    }

    if ((tcp->pkt_desc == PACKET_TYPE_DESCRIPTION_EDR_ACL)
#ifdef COMPILE_ESCO
            || (tcp->pkt_desc == PACKET_TYPE_DESCRIPTION_EDR_eSCO)
#endif
       )
    {
        /* Enable PTT in baseband. */
        lc_enable_ptt_bit(ce_index);
        LMP_LOG_INFO(LOG_LEVEL_HIGH, ENABLED_PTT,0,0);
    }

    return;
}

/**
 * Handles lmp_accepted for LMP_test_control pdu.
 *
 * \param lmp_pdu_ptr Pointer to LMP PDU.
 * \param ce_index Index to LMP-connection-entity.
 *
 * \return None.
 */
void lmp_handle_test_control_accepted(LMP_PDU_PKT *lmp_pdu_ptr,
                                      UINT16 ce_index)
{
    OS_CREATE_TIMER(ONESHOT_TIMER, &lmp_tester_tci_timer,
            lmp_handle_test_control_timeout_handler, NULL, 0);

    /* Create timer with the min resolution */
    OS_START_TIMER(lmp_tester_tci_timer, 15);

    LMP_LOG_INFO(LOG_LEVEL_HIGH, TESTER_TIMER_CREATED, 1, lmp_tester_tci_timer);
    return;
}


/**
 * Generate LMP_not_accepted for LMP_test_control pdu.
 *
 * \param lmp_pdu_ptr Pointer to LMP PDU.
 * \param reason Reason for rejection.
 *
 * \return None.
 */
void lmp_test_mode_generate_not_accepted_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UCHAR reason)
{
    UCHAR parameter_list[LMP_NOT_ACCEPTED_LEN];
    UINT16 ce_index;

    /* Generate Lmp_Not_Accepted pdu */
    parameter_list[0] = LMP_NOT_ACCEPTED_OPCODE ;
    parameter_list[2] = LMP_TEST_CONTROL_OPCODE ;
    parameter_list[3] = reason;

    RT_BT_LOG(GRAY, LMP_PDU_5831, 1, reason);

    /* This is called only in the slave piconet context.*/
    LMP_GET_CE_INDEX_FROM_AM_ADDR_PPI(
        (UCHAR)lmp_pdu_ptr->am_addr, SCA_PICONET_FIRST, &ce_index);

    lmp_generate_pdu(ce_index, parameter_list, LMP_NOT_ACCEPTED_LEN,
                     REMOTE_DEV_TID, LMP_NO_STATE_CHANGE);
}
#endif /* TEST_MODE */


#ifdef COMPILE_ROLE_SWITCH
/**
 * Starts the role switch operation during connection and it should be called
 * inly by the slave.
 *
 * \param ce_index ACL connection entity index.
 * \param bd_addr BD Address of the other device.
 *
 * \return None.
 */
void lmp_start_role_switch_during_conn_as_slave(UINT16 ce_index,
        UCHAR *bd_addr)
{
    UINT32 native_clock;
    UINT32 switch_instant;
    UCHAR parameter_list[LMP_SWITCH_REQ_LEN];
    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    lmp_send_slot_offset_pdu(ce_index, SLAVE_TID);

    parameter_list[0] = LMP_SWITCH_REQ_OPCODE;

    /*
     *  Switch instant should be atleast 2*Tpoll or 32.
     *  (Whichever is greater)
     */
#ifndef _DAPE_TEST_INCREASE_ROLE_SW_WHEN_ACCEPT_CONN_REQ
    switch_instant = (ce_ptr->Tpoll << 2);
    if (switch_instant < 32)
    {
        switch_instant = 32;
    }
#else	
    UCHAR total_pdus;
    total_pdus = 1;

    UINT8 i;
    for (i = 0; i <= SCA_PICONET_MAX; i++)
    {
        total_pdus += pduq_get_no_of_pdus_piconet(i);
    } 
    switch_instant = (4 + total_pdus) * (ce_ptr->Tpoll);
    if (lmp_self_device_data.number_of_acl_conn > 1)
    {
        if (switch_instant < 320)
        {
            switch_instant = 320;
        }
        /* There are two links and one of them is (e)sco. Almost full bandwidth. */
        if ((lmp_self_device_data.number_of_esco_connections != 0) ||
           (lmp_self_device_data.total_no_of_sco_conn !=0))
        {
            if (switch_instant < 480)
            {
                switch_instant = 480;
            }
        }        
    }
    else
    {
        if (switch_instant < 238)
        {
            switch_instant = 238;
        }
    }
#endif

    lc_get_clock_in_scatternet(&native_clock, ce_ptr->phy_piconet_id);

    native_clock >>= 1 ;
    native_clock += switch_instant;
    native_clock &= (BT_CLOCK_28_BITS >> 1);

    /* Make sure that instant is an even slot.*/
    native_clock &= (~0x01);

    native_clock = (UINT32) (native_clock & (~0x01) );
    ce_ptr->switch_instant = native_clock;

    bt_fw_ultostr(&parameter_list[2], native_clock, 4);

    lmp_generate_pdu(ce_index, parameter_list, LMP_SWITCH_REQ_LEN,
                     SLAVE_TID, LMP_CONNECTION_ROLE_SWITCH);

    return;
}
#endif /* COMPILE_ROLE_SWITCH */
