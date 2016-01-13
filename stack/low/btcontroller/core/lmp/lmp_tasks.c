/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  Contains the code that implements a LMP task and the functionality
 *  supported by the LMP task.
 */

/********************************* Logger *************************/
enum { __FILE_NUM__= 58 };

/********************************* Logger *************************/

/* ========================= Include File Section ========================= */
#include "mem.h"
#include "lmp_internal.h"
#include "lmp_ch_assessment.h"

#include "vendor.h"
#include "btc_defines.h"
#include "led_debug.h"
#include "bt_fw_acl_q.h"
#include "lmp_pdu_q.h"
#include "bz_debug.h"
#include "lmp_2_1.h"
#include "bt_fw_hci_2_1.h"
#ifdef _SECURE_CONN_TEST_LOG
#include "bz_auth_internal.h"
#endif
#ifdef VER_3_0
#include "lmp_3_0.h"
#endif
#ifdef _SUPPORT_PCA_ADJUST
#include "lmp_4_1.h"
#endif

#include "lc.h"

#include "UartPrintf.h"

#ifdef LE_MODE_EN
#include "le_ll.h"
#include "le_hw_reg.h"
#endif

#include "ecdh.h"

#ifdef _ENABLE_RTK_PTA_
#include "pta.h"
#endif

#include "bz_auth_internal.h"
#include "lc_internal.h"

#ifdef _DAPE_TEST_ACL_RX_FIFO_DBG
extern UINT8 lc_sco_pause_status;
#endif
/* ==================== Structure declaration Section ===================== */


/* ================== For rom code patch function point ================== */

#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8723_RCP_
PF_ROM_CODE_PATCH_FUNC  rcp_lmp_handle_hardware_level_connection_end = NULL;
#endif
#endif

/* ===================== Variable Declaration Section ===================== */
/** This variable defers the processing of received unpark indication
    from remote slave till the end of the access window.  */
UINT32 lc_slave_init_unpark_pending = FALSE;

/** Number of responses expected for the inquiry */
UCHAR lmp_num_inq_resp_expected;
/** Number of responses received */
UCHAR lmp_num_inq_resp_received;
UINT16 bc_ce_index;
/** Indicates the baseband status of the periodic inquiry */
UCHAR lmp_periodic_inquiry = FALSE;

#ifdef POWER_CONTROL
TimerHandle_t lmp_power_control_handle = NULL;
#endif
#ifndef _ENABLE_COMPILE_NESTED_PAUSE_RESUME_
extern UCHAR lc_pause_schedule_pkts;
#endif
extern UCHAR g_lc_scan_slot_timer_in_use;

#ifdef COMPILE_PARK_MODE
extern UCHAR lmp_current_nb;
void lmp_select_unpark_device(void);
UINT16 lmp_get_park_ce_index();
extern UINT16 lmp_unpark_ce_index;
extern UINT16 lc_num_unpark_req_trials;
#endif
#ifdef COMPILE_ESCO
extern LMP_ESCO_CONNECTION_ENTITY
lmp_esco_connection_entity[LMP_MAX_ESCO_CONN_ENTITIES];
#endif
extern void lmp_handle_llc_complete_signal(UINT16 ce_index);
extern TimerHandle_t la_period_timer;
extern UINT8 lc_check_and_join_fhs_filter(UINT8 * fhs_src,UINT8 eir);
#ifdef ESCO_DISC_DEBUG
extern UCHAR transaction_id_of_disconnection;
extern UCHAR reason_for_disconnection;
extern UCHAR wait_for_esco_window_end_intr;
#endif

#ifdef COMPILE_ROLE_SWITCH
UCHAR lc_check_scheduler_for_aborting_mss(UCHAR phy_piconet_id);
UCHAR lc_check_for_aborting_mss(UCHAR phy_piconet_id);
INLINE UCHAR lc_check_sniff_for_aborting_mss(void);
void lc_cleanup_mss_abort_at_instant(void);
#endif

#ifdef _DAPE_SNIFF_PKT_TEST
extern UINT32 lc_get_least_sniff_interval(UINT16 *sniff_ce_index, UINT16 *sniff_attempt);
#else
extern UINT32 lc_get_least_sniff_interval(void);
#endif
#ifndef _ENABLE_COMPILE_NESTED_PAUSE_RESUME_
extern UCHAR lc_pause_schedule_pkts;
#endif
void lmp_handle_detach_pdu_sent(UINT16 ce_index);

#ifdef _SSP_DHKEY_CALCULATE_NEW_FLOW_
extern void bz_auth_initiate_auth_stage2(UINT16 ce_index, LMP_TRAN_ID tid);
#endif

/* ================== Static Function Prototypes Section ================== */
void lmp_handle_page_resp_timeout(UCHAR am_addr, UCHAR piconet_id);
#ifdef COMPILE_PARK_MODE
void lmp_handle_park_ncto_signal(UINT16 ce_index);
#endif /* COMPILE_PARK_MODE */

/**
 * Copies new_lsto to link_supervision_timeout variable
 * and starts/stops the link_supervision_timer.
 * Called from pdu ack of link_supervision_pdu.
 *
 * \param ce_index CE Index.
 *
 * \return None.
 */
void lmp_update_supervision_timer(UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY *ce_ptr = &lmp_connection_entity[ce_index];
    UINT16 link_super_timeout = ce_ptr->stored_link_supervision_timeout;

    ce_ptr->link_supervision_timeout = link_super_timeout;

    if (ce_index == INVALID_CE_INDEX)
    {
        return;
    }

    if (OS_STOP_TIMER(ce_ptr->supervision_timeout_handle,0) != BT_ERROR_OK)
    {
        BT_FW_HCI_ERR(UNABLE_TO_STOP_SUPERVISION_TIMEOUT,0,0);
    }

    if ( link_super_timeout != 0)
    {
        OS_START_TIMER(ce_ptr->supervision_timeout_handle, (UINT16)(
                           SLOT_VAL_TO_TIMER_VAL(LMP_SUPERVISION_TIMER_RESOLUTION)));

        /* Adjust Tpoll to be sup_timeout/8 . */
        if ((link_super_timeout >> 3) < (ce_ptr->Tpoll))
        {
            ce_ptr->Tpoll = (UINT16) (link_super_timeout >> 3);
        }
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
}

/* ===================== Function Definition Section ====================== */
/**
 * The LMP task which handles all the LMP related operations.
 *
 * \param signal_ptr Pointer to the OS signal structure.
 *
 * \return None.
 */
void LMP_Task(OS_SIGNAL *signal_ptr)
{
    UCHAR am_addr ;
    UINT32 piconet_id;

    piconet_id = (UINT32)((UINT32)signal_ptr->ext_param);

    switch(signal_ptr->type)
    {
        case SIGNAL_BEGIN:
            break;

        case SIGNAL_END:
            lmp_shutdown();
            break;

        case LMP_AFH_PROC_COMPLETION_SIGNAL:
            break;

        case LMP_HARDWARE_LVL_CONN_RECD_SIGNAL:
            am_addr = (UCHAR)((UINT32)signal_ptr->param) ;
            lmp_handle_hardware_level_connection(am_addr, piconet_id);
            break;

        case LMP_FHS_PKT_RECD_SIGNAL:
            lmp_handle_fhs_packet((LMP_FHS_PKT_RECD*)signal_ptr->param,
                                  (UCHAR) piconet_id);
            break;

        case LMP_PDU_RECD_SIGNAL:
            lmp_handle_incoming_pdu((LMP_PDU_PKT *)signal_ptr->param,
                                    (UCHAR) piconet_id);
            break;

        case LMP_SUPERVISION_TIMEOUT_SIGNAL:
            lmp_supervision_timeout_handler((UINT16)
                                            ((UINT32)signal_ptr->param));
            break;

        case LMP_HANDLE_INQUIRY_TIMEOUT_SIGNAL:
            lmp_handle_inquiry_timeout();
            break;

        case LMP_HANDLE_PAGE_TIMEOUT_SIGNAL:
            lmp_handle_page_timeout((UCHAR)((UINT32)signal_ptr->param),
                                    piconet_id);
            break;

        case LMP_DELETE_TIMER_HANDLE_SIGNAL:
        {
            TimerHandle_t timerid;
            timerid = (TimerHandle_t)(signal_ptr->param);

            if (timerid == NULL)
            {
                break;
            }

            if (OS_DELETE_TIMER(&timerid) != BT_ERROR_OK)
            {
                LMP_ERR(LMP_HANDLE_TIMER_DELETE_TIMER_FAILED,1,timerid);
            }
        }
        break;

        case LMP_PDU_ACK_RECD_SIGNAL:
        {
            UINT32 debug_clock;

            debug_clock = piconet_id & 0xFFFFFFF;
            piconet_id = piconet_id >> 30;

            lmp_handle_pdu_ack_recd((LMP_PDU_PKT *)signal_ptr->param,
                                    (UCHAR) piconet_id, debug_clock);
        }
        break;

#ifdef COMPILE_HOLD_MODE
        case LMP_HOLD_INSTANT_SIGNAL:
            lmp_handle_hold_instant_signal((UINT16)((UINT32)signal_ptr->param));
            break;

        case LMP_COMPLETE_HOLD_MODE_SIGNAL:
            lmp_handle_complete_hold_mode((UINT16)((UINT32)signal_ptr->param));
            break;
#endif /* COMPILE_HOLD_MODE */

#ifdef COMPILE_PARK_MODE
        case LMP_ABNORMAL_EXIT_PARK_SIGNAL:
            lmp_abnormal_exit_park_n_cleanup(
                (UINT16)((UINT32)signal_ptr->param),
                (UCHAR)((UINT32)signal_ptr->ext_param));
            break;

        case LMP_SLAVE_UNPARK_REQ_RECD_SIGNAL:
            if (lc_slave_init_unpark_pending != TRUE)
            {
                lmp_handle_slave_unpark_req((UCHAR)((UINT32)signal_ptr->param),
                                            piconet_id);
            }

            break;

#ifdef COMPILE_AFH_HOP_KERNEL
        case LMP_UPDATE_MAP_SIGNAL:
            lmp_update_map(AFH_ENABLE,
                           (UINT16)((UINT32)signal_ptr->param), FALSE);
            break;
#endif
#endif /* COMPILE_PARK_MODE */

        case LMP_LLC_CON_SETUP_COMPLETE:
            lmp_handle_llc_complete_signal((UINT16)((UINT32)signal_ptr->param));
            break;

        case LMP_HANDLE_PAGESCAN_NCTO_SIGNAL:
            am_addr = (UCHAR)((UINT32)signal_ptr->param);
            piconet_id = (UINT16)((UINT32)signal_ptr->ext_param);
            lmp_handle_page_resp_timeout(am_addr, (UCHAR)piconet_id);
            break;

        case LMP_CALCULATE_DHKEY_SIGNAL:
            bz_auth_calculate_dhkey(
                (BZ_AUTH_LINK_PARAMS_PTR)signal_ptr->param);
            break;

        case LMP_CALCULATE_DHKEY_PARTIAL_SIGNAL:
            mixed_scalar_multiply_state1((UINT8)((UINT32)signal_ptr->param),
                                          TRUE);
            break;

#ifdef _SSP_DHKEY_CALCULATE_NEW_FLOW_
        case LMP_CHK_AUTH_STAGE2_SIGNAL:
            bz_auth_initiate_auth_stage2((UINT16)((UINT32)signal_ptr->param),
                                (LMP_TRAN_ID)((UINT32)signal_ptr->ext_param));
            break;
#endif

#ifdef COMPILE_SNIFF_MODE
        case LMP_INITIATE_SSR_REQ:
        {
            UINT16 ce_index;
            ce_index = (UINT16)((UINT32)signal_ptr->param);
            if (lmp_connection_entity[ce_index].ssr_data.lmp_ssr_state
                    == LMP_SSR_IDLE)
            {
//					LMP_LOG_INFO(LOG_LEVEL_HIGH, TRYING_TO_INITIATE_SSR_REQ,0,0);
                lmp_connection_entity[ce_index].ssr_data.lmp_ssr_state =
                    LMP_SSR_TRIED;
                lmp_initiate_sniff_subrating(ce_index);
            }
        }
        break;
#endif

#ifdef LE_MODE_EN
        case LL_GEN_HCI_EVENT_SIGNAL:
            ll_handle_generate_hci_event_signal(
                (LL_TASK_PARA_U *)&signal_ptr->param);
            break;

        case LL_CONTROL_SIGNAL:
            llc_check_and_schedule_next_pend_pdu(
                (UINT8)((UINT32)signal_ptr->param));
            break;
#endif

        case LMP_START_TIMER_SIGNAL:
            OS_START_TIMER((TimerHandle_t)(signal_ptr->param),
                           (UINT32)(signal_ptr->ext_param));
            break;

        case LMP_LAST_CON_DISCONNECTED_SIGNAL:
            lmp_handle_last_conn_disconnected();
            break;

#ifdef COMPILE_PARK_MODE
        case LMP_PARK_NCTO_SIGNAL:
        {
            UINT16 ce_index;
            ce_index = (UINT16)((UINT32)signal_ptr->param);
            lmp_handle_park_ncto_signal(ce_index);
        }
        break;
#endif

#ifdef COMPILE_SNIFF_MODE
        case LMP_EXIT_SNIFF_MODE:   /* Fall through. */
        case LMP_ENTER_SNIFF_MODE:
            lmp_send_max_slot_pdu_to_all_devices();
            /* Warning: Fall through here. Do not insert break. */
            break; /* add break here due to following signal LMP_SNIFF_NO_ACL_RESTORE_SIGNAL is no use anymore*/

#ifdef _USE_SNIFF_NO_ACL_COUNT_REGISTER
        case LMP_SNIFF_NO_ACL_RESTORE_SIGNAL:
        {
            UCHAR lcl_least_sniff_interval = 0;
#ifdef _DAPE_SNIFF_PKT_TEST
            UINT16 sniff_ce_index;
            UINT16 sniff_attempt;
            lcl_least_sniff_interval = lc_get_least_sniff_interval(&sniff_ce_index, &sniff_attempt);
#else
            lcl_least_sniff_interval = lc_get_least_sniff_interval();
#endif
            /* Program 7 in sniff-no-acl register. */
            BB_write_baseband_register(SNIFF_NO_ACL_COUNT_REGISTER, 0x07);

            if (lcl_least_sniff_interval <
                    LC_MIN_SNIFF_INTERVAL_ALLOW_5_SLOT_PKT)
            {
                /* 5 slot packets are disabled. 1 and 3 slot packets
                   are only enabled. */

                /* Program 5 in sniff-no-acl register. */
                BB_write_baseband_register(SNIFF_NO_ACL_COUNT_REGISTER, 0x05);
            }
            else if (lcl_least_sniff_interval <
                    LC_MIN_SNIFF_INTERVAL_ALLOW_3_SLOT_PKT)
            {
                /* Only 1 slot packets are enabled. All other packets are disabled. */
                /* Program 3 in sniff-no-acl register. */
                BB_write_baseband_register(SNIFF_NO_ACL_COUNT_REGISTER, 0x03);
            }
        }
        break ;
#endif
#endif

        case LMP_GENERIC_SIGNAL:
            ((LMP_GENERIC_SIGNAL_HDLR) signal_ptr->param)(signal_ptr->ext_param);
            break;

        default:
            LMP_ERR(INVALID_SIGNAL_RECEIVED_IN_LMP_TASK,0,0);
            break ;
    }
}

/**
 * Handles auth completed (with or without encryption) call back
 * during connection.
 *
 * \param ce_index ACL connection entity.
 * \param status Status of authentication procedure.
 * \param reason Reason for authentication failure.
 * \param user_data User data provided during callback registration.
 *
 * \return None.
 */
void lmp_auth_completed_callback(UINT16 ce_index, UINT16 status,
                                 UCHAR reason, void* user_data)
{
    lmp_decide_to_send_conn_complete_evt(ce_index);
}

/**
 * Handles encryption paused call back, during role switch procedure.
 *
 * \param ce_index ACL connection entity.
 * \param status Status of encryption pause.
 * \param auth_role Authentication role of the self device
 *     during any transaction.
 * \param enc_proc Encryption procedure used as a result of a generic call.
 * \param user_data User data provided during callback registration.
 *
 * \return None.
 */
void lmp_pause_encryption_callback(UINT16 ce_index, UINT16 status,
                                   UINT8 auth_role, UINT8 enc_proc,
                                   void* user_data)
{
    LMP_CONNECTION_ENTITY* ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    /* Set the pause encryption context */
    ce_ptr->is_enc_paused = TRUE;
    ce_ptr->auth_role = auth_role;
    ce_ptr->enc_proc = enc_proc;

#ifdef COMPILE_ROLE_SWITCH
    /* We must be initiator here and no need to check for "auth_role" */
    if (ce_ptr->ce_status == LMP_ROLE_SWITCHING)
    {
        /* encryption pause procedure started as part of role-switch */
        if (status == HCI_COMMAND_SUCCEEDED)
        {
            /* Since we have successfully disabled encryption on the link,
             * lets start role-switch procedure now.
             */
            hci_start_role_switch(ce_index);
        }
        else
        {
            /* MsbA: Who will change the ce_status from "role_swiching"? */

            /* oh, God! some how we couldn't disable encryption, anyway we
             * can't proceed with role-switch now.
             */
#ifdef _CCH_IOT_CSR_RS_
            ce_ptr->waiting_for_rs_several_times = 0;
#endif
            hci_generate_role_change_event(UNSPECIFIED_ERROR,
                                           ce_ptr->bd_addr, (UCHAR)!ce_ptr->remote_dev_role);
            lmp_set_ce_status(ce_index, LMP_CONNECTED);

            /* Reset the pause encryption context */
            ce_ptr->is_enc_paused = FALSE;
            /* There seem to be no logic in resuming encryption, while the
             * pause itself had failed. So don't restart encryption here.
             */
#ifdef _ENABLE_COMPILE_NESTED_PAUSE_RESUME_
#ifdef COMPILE_NESTED_PAUSE_RESUME
            aclq_resume_am_addr(ce_ptr->am_addr,
                                        ce_ptr->phy_piconet_id, ACL_PAUSED_MSS);
#else /* COMPILE_NESTED_PAUSE_RESUME */
            aclq_resume_am_addr(ce_ptr->am_addr,
                                        ce_ptr->phy_piconet_id);
#endif /* COMPILE_NESTED_PAUSE_RESUME */
#else
            ce_ptr->pause_data_transfer = FALSE;
#endif
        }
    } /* end if (role-switch) */
#endif
}

/**
 * Handles encryption paused, during role switch procedure.
 *
 * \param ce_index ACL connection entity.
 * \param status Status of encryption pause.
 * \param auth_role Authentication role of the self device
 *     during any transaction.
 * \param enc_proc Encryption procedure used as a result of a generic call.
 * \param user_data User data provided during callback registration.
 *
 * \return None.
 */
void lmp_resume_encryption_callback(UINT16 ce_index, UINT16 status,
                                    UINT8 auth_role, UINT8 enc_proc,
                                    void* user_data)
{
    LMP_CONNECTION_ENTITY* ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    /* Reset the pause encryption context */
    ce_ptr->is_enc_paused = FALSE;

#ifdef COMPILE_ROLE_SWITCH
    /* We have completed the encryption resume procedure */
    if (ce_ptr->ce_status == LMP_ROLE_SWITCHING)
    {
        /* let's see if we should generate role-switch-complete */
        if (enc_proc == BZ_AUTH_ENCRYPTION_PROCEDURE_EPR)
        {
            /* We have used EPR, so we have to generate the role-switch
             * event now only. If we had used encryption STOP procedure,
             * then the role-switch would have already been generated.
             * NOTE: Even if role-switch had failed, we should intimate that
             *       only after the completion of the resume encryption. So we
             *       are storing the role-switch status (mss_completion_status).
             */
            if (ce_ptr->mss_completion_status == HCI_COMMAND_SUCCEEDED)
            {
#ifdef _CCH_IOT_CSR_RS_
                ce_ptr->waiting_for_rs_several_times = 0;
#endif
                hci_generate_role_change_event(HCI_COMMAND_SUCCEEDED,
                                               ce_ptr->bd_addr, (UCHAR)!ce_ptr->remote_dev_role);

                lmp_handle_mss_completion_after_connection(ce_index);
#ifdef _ENABLE_COMPILE_NESTED_PAUSE_RESUME_
#ifdef COMPILE_NESTED_PAUSE_RESUME
                aclq_resume_am_addr(ce_ptr->am_addr,
                                            ce_ptr->phy_piconet_id, ACL_PAUSED_MSS);
#else /* COMPILE_NESTED_PAUSE_RESUME */
                aclq_resume_am_addr(ce_ptr->am_addr,
                                            ce_ptr->phy_piconet_id);
#endif /* COMPILE_NESTED_PAUSE_RESUME */
#else
                ce_ptr->pause_data_transfer = FALSE;
#endif
            }
            else
            {
                /* Generates role-swith event (failure) */
                lmp_handle_role_switch_failure(ce_index,
                                               ce_ptr->mss_completion_status);
            }
        }

        ce_ptr->ce_status = LMP_CONNECTED;

    } /* end if (role-switch) */
#endif
}

/**
 * Handle the LLC (Low Level Connection) complete signal.
 *
 * \param ce_index The ACL connection entity index.
 *
 * \return None.
 */
void lmp_handle_llc_complete_signal(UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY *ce_ptr;
#if defined(POWER_CONTROL) && defined(COMPILE_RSSI_REPORTING)
    UINT16 power_ctl_time;
#endif
    UINT32 device_max_slot;

    ce_ptr = &lmp_connection_entity[ce_index];

#ifndef COMPILE_SINGLE_SLOT_PACKETS_ONLY
    lmp_send_max_slot_pdu(ce_index);

    device_max_slot = lmp_calculate_max_slot(ce_index);
    if (device_max_slot != 1)
    {
        lmp_send_max_slot_req_pdu(ce_index, 0xff);
    }
#endif /* COMPILE_SINGLE_SLOT_PACKETS_ONLY */

#if defined POWER_CONTROL && defined (COMPILE_RSSI_REPORTING)
    /* Send power control PDU */
    if ((ce_ptr->feat_page0[2] & LMP_POWER_CONTROL_FEATURE)
        && (lmp_feature_data.feat_page0[2]& LMP_POWER_CONTROL_FEATURE))
    {
        lmp_send_power_ctrl_pdu(ce_index, SELF_DEV_TID);

        power_ctl_time = SLOT_VAL_TO_TIMER_VAL(LMP_POWER_CONTROL_CYCLE);

        /*
         * Create timer if timer not already created, and the radio
         * supports tx-power control.
         */
        if (lmp_power_control_handle == NULL)
        {
            OS_CREATE_TIMER(PERIODIC_TIMER, &lmp_power_control_handle,
                    lmp_power_control_handler, NULL, power_ctl_time);

            OS_START_TIMER(lmp_power_control_handle, power_ctl_time);
        }
    }
#endif /* defined POWER_CONTROL && defined (COMPILE_RSSI_REPORTING) */


#ifdef _CCH_SC_ECDH_P256_TEST_PKT
RT_BT_LOG(YELLOW, CCH_DBG_033, 1,ce_ptr->connection_type.packet_type);
#endif

#ifdef _DAPE_TEST_SEND_PTT_EARLIER
    if (!g_efuse_lps_setting_3.iot_ralink_send_fea_req_later)
#endif
    {
        UCHAR status = TRUE;
        UCHAR temp;

    temp = (UCHAR)(lmp_feature_data.feat_page0[3] & ce_ptr->feat_page0[3]);

    /* Check if the self device or remote device supports EDR. */

#ifdef _CCH_SC_ECDH_P256_TEST_PKT
    if (( (temp & EDR_ACL_2MBPS_FEATURE) == 0)||((ce_ptr->connection_type.packet_type&0x3306) == 0x3306))
#else
    if ( (temp & EDR_ACL_2MBPS_FEATURE) == 0)
#endif
    {
        status = FALSE;
    }

    if ( (status == TRUE) && (ce_ptr->ptt_status == LMP_PTT_IDLE) )
    {
        lmp_generate_lmp_ptt_req_pdu(ce_index, PTT_ENABLE);
    }
}


#ifdef COMPILE_CQDDR
    if (!IS_USE_FOR_MUTE)
    {
        lmp_send_auto_rate_pdu(ce_index);
    }
#endif

#ifdef COMPILE_AFH_HOP_KERNEL
    if (OS_IS_TIMER_RUNNING(la_period_timer) == FALSE)
    {
        OS_START_TIMER(la_period_timer, afh_la_cycle_period);
        LMP_LOG_INFO(LOG_LEVEL_LOW, CH_AS_TIMER_STARTED_FOR_6, 1, afh_la_cycle_period);
    }
#endif

#ifdef COMPILE_NESTED_PAUSE_RESUME
    aclq_resume_am_addr(ce_ptr->am_addr,
                        ce_ptr->phy_piconet_id, ACL_PAUSED_ALL);
#else /* COMPILE_NESTED_PAUSE_RESUME */
    aclq_resume_am_addr(ce_ptr->am_addr,
                        ce_ptr->phy_piconet_id);
#endif /* COMPILE_NESTED_PAUSE_RESUME */

    return;
}

/**
 * Handle the hardware level connection completion and set the connection
 * entity status to LMP_BB_HL_CONNECTED.
 *
 * \param am_addr The Active Member Address.
 * \param piconet_id The Physical Piconet ID.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_hardware_level_connection(UCHAR am_addr, UCHAR phy_piconet_id)
{
    UINT16 ce_index;
    LMP_CONNECTION_ENTITY *ce_ptr;

    if (LMP_GET_CE_INDEX_FROM_AM_ADDR_PPI(am_addr, phy_piconet_id, &ce_index)
            != API_SUCCESS)
    {
        LMP_ERR(HARDWARE_LVL_CONNECTION_CONNECTION_ENTITY_IS_INVALID_AM_ADDR,1, am_addr);
        return BT_FW_ERROR;
    }

    ce_ptr = &lmp_connection_entity[ce_index];
    if (lmp_self_device_data.device_status == LMP_PAGE)
    {
        lmp_self_device_data.device_status = LMP_IDLE;
    }

    switch (ce_ptr->ce_status)
    {
        case LMP_DURING_CONN_REMOTE_NAME_REQ:
            break;

        case LMP_PAGING:
            ce_ptr->paging_completed_flag = TRUE;
            hci_baseband_cmd_pending = FALSE;
            break;

        case LMP_PAGE_SCAN:
            break;

        default:
            LMP_INF(INVALID_CE_STATUS_IN_HLC_HANDLING_CE_INDEX_CE_STATUS,2,
                    ce_index, ce_ptr->ce_status);
            return BT_FW_ERROR;
    }

    lc_update_pkts_allowed(ce_index);

#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8723_RCP_
    if (rcp_lmp_handle_hardware_level_connection_end != NULL)
    {
        if ( rcp_lmp_handle_hardware_level_connection_end((void *)&ce_index) )
        {
            return BT_FW_SUCCESS;
        }
    }
#endif
#endif

    if((ce_ptr->ce_status != LMP_PAGE_SCAN) || (!g_efuse_lps_setting_3.iot_ralink_send_fea_req_later))
    {
        /* Initiate feature exchange just after BB level connection is done */
        lmp_send_features_req_or_res_pdu(ce_index, LMP_FEATURES_REQ_OPCODE,
                                         SELF_DEV_TID);
        ce_ptr->hci_cmd_bits |= REMOTE_SUP_FEA_BIT_MASK;
        ce_ptr->lmp_expected_pdu_opcode  |= lmp_get_opcode_mask(
                                                LMP_FEATURES_RES_OPCODE, 0x0);
    }

    lmp_set_ce_status(ce_index, LMP_BB_HL_CONNECTED);
#ifndef _DAPE_TEST_NEW_HW_BLOCK_ACL_FOR_LE
#ifdef LE_MODE_EN
#ifdef _DAPE_TEST_BLOCK_LEGACY_FOR_LE
    /* dape test for LE slave + inquiry */
    ll_driver_block_legacy_for_le(FALSE);
#endif
#endif
#endif
    return BT_FW_SUCCESS;
}

/**
 * Handle FHS packet received. The FHS packet can be due to Master's inquiry
 * response and Slave page response.
 *
 * \param lmp_fhs_pkt_recd The pointer to FHS packet.
 * \param piconet_id The Logical Piconet ID.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_fhs_packet(LMP_FHS_PKT_RECD *lmp_fhs_pkt_recd,
                            UCHAR log_piconet_id)
{
    /* You receive FHS Packet in the following 2 scenarios,
     * a. Master's  inquiry response
     * b. Slave Page response
     *      If you are a master you receive a response from the slave and then
     *      update the connection entity database.
     */
    switch(lmp_self_device_data.device_status)
    {
        case LMP_INQUIRY :
            RT_BT_LOG(RED,INQUIRY_FHS_PKT_RECEIVED,0,0);

            LMP_INF(INQUIRY_FHS_PKT_RECEIVED,0,0);
            if (lmp_handle_inquiry_fhs_pkt(lmp_fhs_pkt_recd)
                    == BT_FW_ERROR)
            {
                /* return BT_FW_ERROR; */
            }
            break ;

#ifdef COMPILE_PERIODIC_INQUIRY
        case LMP_PERIODIC_INQUIRY :
            LMP_INF(PERIODIC_INQUIRY_FHS_PKT_RECEIVED,0,0);
            if (lmp_periodic_inquiry == TRUE)
            {
                if (lmp_handle_inquiry_fhs_pkt(lmp_fhs_pkt_recd)
                        == BT_FW_ERROR)
                {
                    /* return BT_FW_ERROR; */
                }
            }
            else
            {
                /* LC queues FHS Pkt reception signal to LMP task, There is some
                 * delay in processing the FHS packet in LMP task.. by the time
                 * LC may get signal for the FHS pkt reception and then the
                 * control comes to LMP task which processes the FHS packet and
                 * then kill the inquiry after that if the LMP task receives any
                 * FHS pkt reception simply drop it....
                 */
                LMP_INF(EXTRA_PACKET_RECD_DROPING_IT,0,0);
            }
            break ;
#endif

        default :
            LMP_ERR(INVALID_STATE_FHS_PACKET_RECEIVED,0,0);
            LMP_INF(LMP_SELF_DEVICE_DATA_DEVICE_STATUS,2,lmp_self_device_data.device_status);

            break;
    }
    if (OS_FREE_BUFFER (lmp_fhs_pkt_buffer_pool_handle, lmp_fhs_pkt_recd )
            != BT_ERROR_OK)
    {
        LMP_ERR(LMP_TASKS_OS_FREE_BUFFER_FAILED,0,0);
    }
    return BT_FW_SUCCESS ;
}

/**
 * Handle all the inquiry FHS packets.
 *
 * \param lmp_fhs_pkt_recd The pointer to the FHS packet.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_inquiry_fhs_pkt(LMP_FHS_PKT_RECD *lmp_fhs_pkt_recd)
{
    UCHAR inq_status ;
    UCHAR rssi_flag = FALSE;
    UCHAR eir = FALSE;

    if ((UCHAR)(lmp_fhs_pkt_recd->fhs_pkt[7] & 0x4))
    {
        eir = TRUE;
    }

    /* If number of responses expected should be less than size of the
     * inquiry result table. If it exceeds.
     *   a. We will not return any error.
     *      This is a constraint on the meomory from HC side.
     *   b. We can overwrite the inquiry result table,
     *      but this would result in sending same BD_ADDR response
     *      to the host.
     *
     *      There is an option to save memory or
     *      want to provide a feature to the host.
     * (This is optional feature in the BT specification)
     */

    if (lmp_extract_fhs_packet_to_inquiry_table(lmp_fhs_pkt_recd)
            != BT_FW_SUCCESS)
    {
        return BT_FW_SUCCESS;
    }

    /** Generate the event */
    if (hci_pass_event_through_event_filter(
                INQUIRY_RESULT_FILTER,
                lmp_inquiry_result_data[lmp_inquiry_data_index].bd_addr,
                lmp_inquiry_result_data[lmp_inquiry_data_index].class_of_device))
    {
#ifdef COMPILE_INQ_RES_EVENT_WITH_RSSI
        rssi_flag = FALSE;
        if ((lmp_self_device_data.inquiry_mode == HCI_INQ_RESULT_EVENT_WITH_RSSI) ||
           (lmp_self_device_data.inquiry_mode == HCI_INQ_RESULT_EVENT_WITH_EIR))
        {
            rssi_flag = TRUE;
        }
#endif
        if ((eir == FALSE) ||
           (lmp_self_device_data.inquiry_mode != HCI_INQ_RESULT_EVENT_WITH_EIR))
        {
            hci_generate_inquiry_result_event( lmp_inquiry_data_index,
                                               LMP_NUM_RESPONSES, rssi_flag);
        }
        else
        {
#ifdef COMPILE_RSSI_REPORTING
            hci_generate_eir_event(
                lmp_inquiry_result_data[lmp_inquiry_data_index].bd_addr,
                lmp_inquiry_result_data[lmp_inquiry_data_index].page_scan_repetition_mode,
                lmp_inquiry_result_data[lmp_inquiry_data_index].class_of_device,
                lmp_inquiry_result_data[lmp_inquiry_data_index].clock_offset,
                lmp_inquiry_result_data[lmp_inquiry_data_index].rssi,
                (UCHAR *)lmp_fhs_pkt_recd->recv_data,
                lmp_fhs_pkt_recd->recv_length);
#endif
        }

        if (lmp_num_inq_resp_expected != 0)
        {
            lmp_num_inq_resp_received ++ ;
            LMP_INF(NUMBER_OF_RESPONSES_RECEIVED,1,lmp_num_inq_resp_received);
            LMP_INF(NUMBER_OF_RESPONSES_EXPECTED,1,lmp_num_inq_resp_expected);
        }
    }
    else
    {
        return BT_FW_SUCCESS ;
    }

    /*
    * This value should be less than :
    * Event_buff_size/size of one inquiry result.
    *
    * Size of inquiry result table should be more than the
    * number of responses.
    */
    if ((lmp_num_inq_resp_expected == lmp_num_inq_resp_received) &&
       (lmp_num_inq_resp_expected != 0))
    {
        /* Kill the inquiry */
        lc_kill_inquiry();

        inq_status = 0x00 ; /* Inquiry is completed successfully */

        /* Reset the inquiry command pending flag */
        hci_baseband_cmd_pending  = FALSE ;
        /* Send the inquiry_complete event now */
        hci_generate_inquiry_complete_event(inq_status);

        /*Inquiry result table is reset in inquiry complete event */
        lmp_num_inq_resp_received = 0 ;
        lmp_inquiry_data_index = 0;
        lmp_periodic_inquiry = FALSE ;

        switch(lmp_self_device_data.device_status)
        {
            case LMP_INQUIRY :
                lmp_self_device_data.device_status = LMP_IDLE ;
                break;

            default :
                break;
        }
    }

    lmp_inquiry_data_index++;
    lmp_inquiry_data_index &= (LMP_MAX_INQUIRY_RESULT_DATA - 1) ;

    return BT_FW_SUCCESS ;
}

/**
 * Handle all the page scan FHS packets.
 *
 * \param lmp_fhs_pkt_recd The pointer to the FHS packet.
 * \param phy_piconet_id Physical piconet id of the connection.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_page_scan_fhs_pkt(LMP_FHS_PKT_RECD *lmp_fhs_pkt_recd,
                                   UCHAR phy_piconet_id, UCHAR lut_index,
                                   UCHAR *bd_addr)
{
    UINT16 ce_index;
    UCHAR am_addr;
    LMP_CONNECTION_ENTITY *ce_ptr;

    am_addr = lmp_fhs_pkt_recd->am_addr;

    /* Extract the BD_ADDR */
    lmp_extract_bd_addr_from_fhs_packet(lmp_fhs_pkt_recd->fhs_pkt, bd_addr);

    if (LMP_GET_CE_INDEX_FROM_BD_ADDR(bd_addr, &ce_index) == API_SUCCESS)
    {
        ce_ptr = &lmp_connection_entity[ce_index];

        if (ce_ptr->ce_status == LMP_PAGE_SCAN)
        {
#ifdef _USE_SNIFF_NO_ACL_COUNT_REGISTER
#ifdef COMPILE_SNIFF_MODE
            if (lmp_self_device_data.no_acl_ce_index == ce_index)
            {
                lmp_self_device_data.no_acl_reduced_flag = FALSE;
                lmp_self_device_data.no_acl_ce_index = 0xFF;
            }
#endif
#endif

            LMP_REMOVE_BD_ADDR_FROM_HASH(ce_ptr->bd_addr);

            /* This function releases AM address, connection entity and
             * connection handle. It initializes the conn entity to its
             * default values.*/

#ifdef _CCH_IOT_FTP_PAGE_
            if (lmp_release_am_addr_ppi(am_addr, ce_ptr->phy_piconet_id) != API_SUCCESS)
            {
                LMP_ERR(CAN_NOT_RELEASE_AM_ADDR,1,am_addr);
            }
            else
            {
                lmp_assigned_ce_index--;
            }
#else

            if (lmp_release_am_addr_ppi(am_addr, ce_ptr->phy_piconet_id) != API_SUCCESS)
            {
                LMP_ERR(CAN_NOT_RELEASE_AM_ADDR,1,am_addr);
                return BT_FW_ERROR ;
            }

            /* Connection handle is freed, decrement the assigned ce_index. */

            lmp_assigned_ce_index--;
#endif

        }
#ifdef _DAPE_TEST_DISCONNECT_WHEN_NEW_HLC
        else
        {
            LC_EXIT_SM_MODE();
            lmp_disconnect_links(ce_index, CONNECTION_TIMEOUT_ERROR);
            ce_ptr->disconnect_reason = CONNECTION_TIMEOUT_ERROR;
#ifdef _CCH_PAGE_CON_
            ce_ptr->connect_reason = 0;
#endif
            lmp_cleanup_after_acl_detach(ce_index);
        }
#endif
    }

    /* Copy the BD address to inquiry table */
    if ((ce_index = lmp_allocate_entity_from_ce_database()) == BT_FW_ERROR)
    {
        LMP_ERR(CONNECTION_ENTITY_ALLOCATION_FAILED,0,0);
        return BT_FW_ERROR;
    }

    ce_ptr = &lmp_connection_entity[ce_index];

    ce_ptr->remote_dev_role = MASTER;

    /* Remote device as a master gives am_addr. use that am_Addr */
    ce_ptr->am_addr = am_addr;

    if (phy_piconet_id <= SCA_PICONET_MAX)
    {
        ce_ptr->phy_piconet_id = phy_piconet_id;
    }
    else
    {
        RT_BT_LOG(RED, PICONET_ID_CHECK_MSG, 1, phy_piconet_id);
        ce_ptr->phy_piconet_id = SCA_PICONET_SECOND;
    }

    ce_ptr->Tpoll = otp_str_data.bt_t_poll_slave;

#ifdef COMPILE_AFH_HOP_KERNEL
    /* Disable AFH - clean up */
    BB_disable_afh(am_addr, ce_ptr->phy_piconet_id);
#endif /* COMPILE_AFH_HOP_KERNEL */

    lmp_slave_use_am_addr_ppi(am_addr, ce_ptr->phy_piconet_id, ce_index);

    ce_ptr->remote_dev_role = (UCHAR)MASTER;

    lmp_set_ce_status(ce_index, LMP_PAGE_SCAN);

    lmp_extract_fhs_packet_to_ce(lmp_fhs_pkt_recd, ce_index);

    lc_update_ce_index_to_lut_extn_tbl(ce_index, lut_index);
    return BT_FW_SUCCESS ;
}


/**
 * Check and validate the incoming pdu. Depending on the type of pdu received
 * it routes the received pdu to appropriate function.
 *
 * \param lmp_pdu_ptr The pointer to the LMP PDU packet.
 * \param piconet_id The Physical Piconet ID.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_incoming_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UCHAR phy_piconet_id)
{
    UINT16 ce_index = 0;
    UCHAR pdu_opcode;
    UCHAR status;

#ifdef COMPILE_PARK_MODE
    UCHAR park_pdus = FALSE;
#endif

#if !defined(ENABLE_SCO) || !defined(COMPILE_HOLD_MODE) || \
    !defined(COMPILE_SNIFF_MODE) || !defined(COMPILE_ROLE_SWITCH) || \
    !defined(COMPILE_PARK_MODE) || \
    !defined(POWER_CONTROL)
    UCHAR transaction_id ;

    /* Check the Transaction ID bit. */
    transaction_id = (UCHAR)(lmp_pdu_ptr->payload_content[0] & 0x01);
#endif

    /* AM_ADDR param is in LMP PDU packet
     * Extract the index to CE database from the AM_ADDR */
    pdu_opcode = (UCHAR)(lmp_pdu_ptr->payload_content[0] >> 1);

    lmp_pdu_ptr->piconet = phy_piconet_id;

    UINT32 cur_clk;
    lc_get_clock_in_scatternet(&cur_clk, phy_piconet_id);

#ifdef _DAPE_TEST_ACL_RX_FIFO_DBG
    UINT32 acl_rx_fifo_reg;
    acl_rx_fifo_reg = RD_U32_BZDMA_REG(BZDMA_REG_ACL_RXFIFO_PTR);
    UINT16 acl_rx_wptr = 0;
    UINT16 acl_rx_rptr = 0;
    acl_rx_wptr = (acl_rx_fifo_reg >> 16) & 0x7FF;
    acl_rx_rptr = acl_rx_fifo_reg & 0x7FF;
    if (acl_rx_wptr != acl_rx_rptr)
    {
        //gpio_one_pull_high(1);
        //gpio_one_pull_low(1);
        RT_BT_LOG(RED, DAPE_TEST_LOG213, 2, BZDMA_REG_ACL_RXFIFO_PTR, acl_rx_fifo_reg);
    }
#endif
#ifdef _SECURE_CONN_TEST_LOG
        BZ_AUTH_LINK_PARAMS* auth;
        auth = lmp_connection_entity[ce_index].auth;
///// dape test
//RT_BT_LOG(BLUE, DAPE_TEST_LOG522, 2,auth->super_state, auth->sub_state);
#endif
    //by liuyong 20090727
    if (pdu_opcode != 127)
    {
#ifndef _DAPE_TEST_ACL_RX_FIFO_DBG
        RT_BT_LOG(BLUE, LMP_MSG_RX_NON_EXT_PDU, 15,
                        cur_clk, pdu_opcode,
                        lmp_pdu_ptr->am_addr, phy_piconet_id,
                        lmp_pdu_ptr->payload_content[0] & 0x01,
                        lmp_pdu_ptr->ce_index,
                        lc_sca_manager.pnet[phy_piconet_id].master,
                        lmp_pdu_ptr->payload_content[0],
                        lmp_pdu_ptr->payload_content[1],
                        lmp_pdu_ptr->payload_content[2],
                        lmp_pdu_ptr->payload_content[3],
                        lmp_pdu_ptr->payload_content[4],
                        lmp_pdu_ptr->payload_content[5],
                        lmp_pdu_ptr->payload_content[6],
                        lmp_pdu_ptr->payload_content[7]);
#else
        RT_BT_LOG(BLUE, DAPE_TEST_LMP_MSG_RX_NON_EXT_PDU, 17,
                        cur_clk, pdu_opcode,
                        lmp_pdu_ptr->am_addr, phy_piconet_id,
                        lmp_pdu_ptr->payload_content[0] & 0x01,
                        lmp_pdu_ptr->ce_index,
                        lc_sca_manager.pnet[phy_piconet_id].master,
                        lmp_pdu_ptr->payload_content[0],
                        lmp_pdu_ptr->payload_content[1],
                        lmp_pdu_ptr->payload_content[2],
                        lmp_pdu_ptr->payload_content[3],
                        lmp_pdu_ptr->payload_content[4],
                        lmp_pdu_ptr->payload_content[5],
                        lmp_pdu_ptr->payload_content[6],
                        lmp_pdu_ptr->payload_content[7],
                        lc_sco_pause_status, acl_rx_fifo_reg);

#endif
    }
    else
    {
#ifndef _DAPE_TEST_ACL_RX_FIFO_DBG
        RT_BT_LOG(BLUE, LMP_MSG_RX_EXT_PDU, 16,
                        cur_clk, pdu_opcode,
                        lmp_pdu_ptr->payload_content[1],
                        lmp_pdu_ptr->am_addr,
                        phy_piconet_id,
                        lmp_pdu_ptr->payload_content[0] & 0x01,
                        lmp_pdu_ptr->ce_index,
                        lc_sca_manager.pnet[phy_piconet_id].master,
                        lmp_pdu_ptr->payload_content[0],
                        lmp_pdu_ptr->payload_content[1],
                        lmp_pdu_ptr->payload_content[2],
                        lmp_pdu_ptr->payload_content[3],
                        lmp_pdu_ptr->payload_content[4],
                        lmp_pdu_ptr->payload_content[5],
                        lmp_pdu_ptr->payload_content[6],
                        lmp_pdu_ptr->payload_content[7]);
#else
        RT_BT_LOG(BLUE, DAPE_TEST_LMP_MSG_RX_EXT_PDU, 18,
                        cur_clk, pdu_opcode,
                        lmp_pdu_ptr->payload_content[1],
                        lmp_pdu_ptr->am_addr,
                        phy_piconet_id,
                        lmp_pdu_ptr->payload_content[0] & 0x01,
                        lmp_pdu_ptr->ce_index,
                        lc_sca_manager.pnet[phy_piconet_id].master,
                        lmp_pdu_ptr->payload_content[0],
                        lmp_pdu_ptr->payload_content[1],
                        lmp_pdu_ptr->payload_content[2],
                        lmp_pdu_ptr->payload_content[3],
                        lmp_pdu_ptr->payload_content[4],
                        lmp_pdu_ptr->payload_content[5],
                        lmp_pdu_ptr->payload_content[6],
                        lmp_pdu_ptr->payload_content[7],
                        lc_sco_pause_status, acl_rx_fifo_reg);

#endif
    }

#ifdef _DUMP_LMP_RX_PACKET_
    LMP_LOG_PDU(LOG_LEVEL_HIGH, LOG_PDU_INDEX, LOG_RX_DIR, 20,
                                    (UCHAR *)(lmp_pdu_ptr->payload_content));
#endif

#ifdef COMPILE_PARK_MODE
    switch (pdu_opcode)
    {
        /* Received broadcast PDU */
    case LMP_UNPARK_BD_ADDR_REQ_OPCODE:
        lmp_handle_unpark_BD_ADDR_req_pdu(lmp_pdu_ptr);
        park_pdus = TRUE;
        break;

    case LMP_UNPARK_PM_ADDR_REQ_OPCODE:
        lmp_handle_unpark_PM_ADDR_req_pdu(lmp_pdu_ptr, phy_piconet_id);
        park_pdus = TRUE;
        break;

    case LMP_MODIFY_BEACON_OPCODE :
        ce_index = lmp_get_park_ce_index();

        if (ce_index != INVALID_CE_INDEX)
        {
            lmp_handle_modify_beacon(lmp_pdu_ptr, ce_index);
        }
        park_pdus = TRUE;
        break;

    case LMP_SET_BROADCAST_SCAN_WINDOW_OPCODE :
        ce_index = lmp_get_park_ce_index();
        if (ce_index != INVALID_CE_INDEX)
        {
            lmp_handle_set_broadcast_scan_window(lmp_pdu_ptr, ce_index);
        }
        park_pdus = TRUE;
        break;

    default:
        break;
    }

    if (park_pdus == TRUE)
    {
        if (OS_FREE_BUFFER(lmp_pdu_buffer_pool_handle, lmp_pdu_ptr) !=
                BT_ERROR_OK)
        {
            RT_BT_LOG(GRAY, LMP_TASKS_1241, 0, 0);

            LMP_ERR(LMP_TASKS_OS_FREE_BUFFER_FAILED,0,0);
        }
        return BT_FW_SUCCESS;
    }
#endif /* COMPILE_PARK_MODE */

    ce_index = lmp_pdu_ptr->ce_index;

    if ((ce_index == INVALID_CE_INDEX) ||
            (lmp_connection_entity[ce_index].entity_status != ASSIGNED))
    {
        if (OS_FREE_BUFFER(lmp_pdu_buffer_pool_handle, lmp_pdu_ptr)
                != BT_ERROR_OK)
        {
            RT_BT_LOG(GRAY, LMP_TASKS_1257, 0, 0);
            LMP_ERR(LMP_TASKS_OS_FREE_BUFFER_FAILED,0,0);
        }

        return BT_FW_ERROR;
    }

#ifdef _CCH_IOT_CSR_RS_

    LMP_CONNECTION_ENTITY *ce_ptr;
    ce_ptr = &lmp_connection_entity[ce_index];

    if (ce_ptr->waiting_for_rs_several_times == 2)
    {
        ce_ptr->waiting_for_rs_several_times = 0;
 //       RT_BT_LOG(YELLOW, CCH_DBG_019, 0,0);
        hci_generate_role_change_event(HCI_COMMAND_SUCCEEDED,
                                       ce_ptr->bd_addr, ce_ptr->remote_dev_role ^ 0x01);
#ifndef _ENABLE_COMPILE_NESTED_PAUSE_RESUME_
        ce_ptr->pause_data_transfer = FALSE;
#endif
    }

#endif

    if (lmp_connection_entity[ce_index].ce_status == LMP_DISCONNECTING)
    {
        /* After initiating detach procedure, no incoming PDUs will be
         * entertained.
         */
        if (OS_FREE_BUFFER(lmp_pdu_buffer_pool_handle, lmp_pdu_ptr) != BT_ERROR_OK)
        {
            RT_BT_LOG(GRAY, LMP_TASKS_1276, 0, 0);
        }
        if (lmp_connection_entity[ce_index].pdu_response_timer_running == TRUE)
        {
            OS_STOP_TIMER(lmp_connection_entity[ce_index].lmp_response_timer_handle, 0);
            lmp_connection_entity[ce_index].pdu_response_timer_running = FALSE;
        }
        return BT_FW_SUCCESS;
    }


    lmp_handle_stop_pdu_response_time(ce_index, lmp_pdu_ptr);

    switch(pdu_opcode)
    {
    case LMP_ACCEPTED_OPCODE :
        if (lmp_handle_accepted_pdu(lmp_pdu_ptr, ce_index) == TRUE)
        {
            break;
        }
        if (bz_auth_handle_security_pdus(lmp_pdu_ptr, ce_index) == TRUE)
        {
            return BT_FW_SUCCESS;
        }
        if (lmp_edtm_handle_encapsulated_pdus(lmp_pdu_ptr, ce_index) == TRUE)
        {
            return BT_FW_SUCCESS;
        }
        break;

    case LMP_NOT_ACCEPTED_OPCODE :
        if (lmp_handle_not_accepted_pdu(lmp_pdu_ptr, ce_index) == TRUE)
        {
            break;
        }
        if (bz_auth_handle_security_pdus(lmp_pdu_ptr, ce_index) == TRUE)
        {
            return BT_FW_SUCCESS;
        }
        if (lmp_edtm_handle_encapsulated_pdus(lmp_pdu_ptr, ce_index) == TRUE)
        {
            return BT_FW_SUCCESS;
        }
        break;

    case LMP_FEATURES_REQ_OPCODE :
        lmp_handle_features_request_pdu(lmp_pdu_ptr, ce_index);
        break;

    case LMP_FEATURES_RES_OPCODE :
        lmp_handle_features_response_pdu(lmp_pdu_ptr, ce_index);
        break;

    case LMP_HOST_CONNECTION_REQ_OPCODE :
        lmp_handle_host_connection_request_pdu(lmp_pdu_ptr, ce_index);
        break;

    case LMP_DETACH_OPCODE :
        lmp_handle_detach_pdu(lmp_pdu_ptr, ce_index);
        break;

    case LMP_SETUP_COMPLETE_OPCODE:
        lmp_handle_setup_complete_pdu(lmp_pdu_ptr, ce_index);
        break;

    case LMP_SCO_LINK_REQ_OPCODE :
#ifdef ENABLE_SCO
        lmp_handle_sco_link_request_pdu(lmp_pdu_ptr, ce_index);
#else /* ENABLE_SCO */
        lmp_send_lmp_not_accepted(ce_index, pdu_opcode,
                                  transaction_id, UNSUPPORTED_REMOTE_FEATURE_ERROR);
#endif /* ENABLE_SCO */
        break;

    case LMP_REMOVE_SCO_LINK_REQ_OPCODE :
#ifdef ENABLE_SCO
        lmp_handle_remove_sco_link_request_pdu(lmp_pdu_ptr, ce_index);
#else /* ENABLE_SCO */
        lmp_send_lmp_not_accepted(ce_index, pdu_opcode,
                                  transaction_id, UNSUPPORTED_REMOTE_FEATURE_ERROR);
#endif /* ENABLE_SCO */
        break;

    case LMP_HOLD_OPCODE:
#ifdef COMPILE_HOLD_MODE
        lmp_handle_hold_pdu(lmp_pdu_ptr,ce_index);
#else /* COMPILE_HOLD_MODE */
        lmp_send_lmp_not_accepted(ce_index, pdu_opcode,
                                  transaction_id, UNSUPPORTED_REMOTE_FEATURE_ERROR);
#endif /* COMPILE_HOLD_MODE */
        break;

    case LMP_HOLD_REQ_OPCODE:
#ifdef COMPILE_HOLD_MODE
        lmp_handle_hold_req_pdu(lmp_pdu_ptr,ce_index);
#else /* COMPILE_HOLD_MODE */
        lmp_send_lmp_not_accepted(ce_index, pdu_opcode,
                                  transaction_id, UNSUPPORTED_REMOTE_FEATURE_ERROR);
#endif /* COMPILE_HOLD_MODE */
        break;

    case LMP_SNIFF_REQ_OPCODE:
#ifdef COMPILE_SNIFF_MODE
        lmp_handle_sniff_req_pdu(lmp_pdu_ptr,ce_index);
#else /* COMPILE_SNIFF_MODE */
        lmp_send_lmp_not_accepted(ce_index, pdu_opcode,
                                  transaction_id, UNSUPPORTED_REMOTE_FEATURE_ERROR);
#endif /* COMPILE_SNIFF_MODE */
        break;

    case LMP_UNSNIFF_REQ_OPCODE:
#ifdef COMPILE_SNIFF_MODE
        lmp_handle_unsniff_req_pdu(lmp_pdu_ptr,ce_index);
#else /* COMPILE_SNIFF_MODE */
        lmp_send_lmp_not_accepted(ce_index, pdu_opcode,
                                  transaction_id, UNSUPPORTED_REMOTE_FEATURE_ERROR);
#endif /* COMPILE_SNIFF_MODE */
        break;

    case LMP_NAME_REQ_OPCODE :
        lmp_handle_name_req_pdu(lmp_pdu_ptr,ce_index);
        break;

    case LMP_NAME_RES_OPCODE :
        lmp_handle_name_resp_pdu(lmp_pdu_ptr,ce_index);
        break;

#ifndef COMPILE_SINGLE_SLOT_PACKETS_ONLY
    case LMP_MAX_SLOT_REQ_OPCODE :
        lmp_handle_max_slot_req_pdu(lmp_pdu_ptr,ce_index);
        break;

    case LMP_MAX_SLOT_OPCODE :
        lmp_handle_max_slot_pdu(lmp_pdu_ptr,ce_index);
        break;
#endif

    case LMP_VERSION_REQ_OPCODE :
        lmp_handle_version_req_pdu(lmp_pdu_ptr,ce_index);
        break;

    case LMP_VERSION_RES_OPCODE :
        lmp_handle_version_resp_pdu(lmp_pdu_ptr,ce_index);
        break;

    case LMP_CLKOFFSET_REQ_OPCODE :
        lmp_handle_clockoffset_req_pdu(lmp_pdu_ptr,ce_index);
        break;

    case LMP_CLKOFFSET_RES_OPCODE :
        lmp_handle_clockoffset_resp_pdu(lmp_pdu_ptr,ce_index);
        break;

    case LMP_SLOT_OFFSET_OPCODE :
        lmp_handle_slot_offset_pdu(lmp_pdu_ptr,ce_index);
        break;

    case LMP_SWITCH_REQ_OPCODE :
#ifdef COMPILE_ROLE_SWITCH
        lmp_handle_switch_req_pdu(lmp_pdu_ptr,ce_index);
#else /* COMPILE_ROLE_SWITCH */
        lmp_send_lmp_not_accepted(ce_index, pdu_opcode,
                                  transaction_id, UNSUPPORTED_REMOTE_FEATURE_ERROR);
#endif /* COMPILE_ROLE_SWITCH */
        break;

    case LMP_SUPERVISION_TIMEOUT_OPCODE :
        lmp_handle_supervision_timeout_pdu(lmp_pdu_ptr,ce_index);
        break;

    case LMP_AUTO_RATE_OPCODE :
#ifdef COMPILE_CQDDR
        if (!IS_USE_FOR_MUTE)
        {
            lmp_handle_auto_data_rate(lmp_pdu_ptr, ce_index);
        }
#endif
        break;

    case LMP_PREFERRED_RATE_OPCODE :
#ifdef COMPILE_CQDDR
        if (!IS_USE_FOR_MUTE)
        {
            lmp_handle_preferred_data_rate(lmp_pdu_ptr, ce_index);
        }
#endif
        break;

    case LMP_TIMING_ACCURACY_REQ_OPCODE :
        lmp_handle_timing_accuracy_req_pdu(lmp_pdu_ptr,ce_index);
        break;

    case LMP_TIMING_ACCURACY_RES_OPCODE :
        lmp_handle_timing_accuracy_res_pdu(lmp_pdu_ptr,ce_index);
        break;

    case LMP_PARK_REQ_OPCODE :
#ifdef COMPILE_PARK_MODE
        lmp_handle_park_req_pdu(lmp_pdu_ptr,ce_index);
#else /* COMPILE_PARK_MODE */
        lmp_send_lmp_not_accepted(ce_index, pdu_opcode,
                                  transaction_id, UNSUPPORTED_REMOTE_FEATURE_ERROR);
#endif /* COMPILE_PARK_MODE */
        break;

    case LMP_DECR_POWER_REQ_OPCODE :
#if defined(POWER_CONTROL)
        lmp_handle_decr_power_req_pdu(lmp_pdu_ptr,ce_index);
        //RT_BT_LOG(GREEN, LMP_MSG_RX_DEC_POWER, 0, 0);
#else /* POWER_CONTROL ... */
        lmp_send_lmp_not_accepted(ce_index, pdu_opcode,
                                  transaction_id, UNSUPPORTED_REMOTE_FEATURE_ERROR);
#endif /* POWER_CONTROL ... */
        break;

    case LMP_INCR_POWER_REQ_OPCODE :
#if defined(POWER_CONTROL)
        lmp_handle_incr_power_req_pdu(lmp_pdu_ptr,ce_index);
        //RT_BT_LOG(GREEN, LMP_MSG_RX_INC_POWER, 0, 0);
#else /* POWER_CONTROL ... */
        lmp_send_lmp_not_accepted(ce_index, pdu_opcode,
                                  transaction_id, UNSUPPORTED_REMOTE_FEATURE_ERROR);
#endif /* POWER_CONTROL ... */
        break;

    case LMP_MIN_POWER_OPCODE :
#if defined(POWER_CONTROL)
        lmp_handle_min_power_pdu(ce_index);
#else /* POWER_CONTROL ... */
        lmp_send_lmp_not_accepted(ce_index, pdu_opcode,
                                  transaction_id, UNSUPPORTED_REMOTE_FEATURE_ERROR);
#endif /* POWER_CONTROL ... */
        break;

    case LMP_MAX_POWER_OPCODE :
#if defined(POWER_CONTROL)
        lmp_handle_max_power_pdu(ce_index);
#else /* POWER_CONTROL ... */
        lmp_send_lmp_not_accepted(ce_index, pdu_opcode,
                                  transaction_id, UNSUPPORTED_REMOTE_FEATURE_ERROR);
#endif /* POWER_CONTROL ... */
        break;

    case LMP_PAGE_MODE_REQ_OPCODE:
        lmp_handle_page_mode_req_pdu(lmp_pdu_ptr,ce_index);
        break;

    case LMP_PAGE_SCAN_MODE_REQ_OPCODE:
        lmp_handle_page_scan_mode_req_pdu(lmp_pdu_ptr,ce_index);
        break;

    case LMP_QoS_OPCODE :
        lmp_handle_qos(lmp_pdu_ptr,ce_index);
        break;

    case LMP_QoS_REQ_OPCODE :
        lmp_handle_qos_req(lmp_pdu_ptr,ce_index);
        break;

#ifdef TEST_MODE
    case LMP_TEST_ACTIVATE_OPCODE:
        lmp_handle_test_activate_pdu(lmp_pdu_ptr, ce_index);
        break;

    case LMP_TEST_CONTROL_OPCODE:
        lmp_handle_test_control_opcode(lmp_pdu_ptr, ce_index);
        break;
#endif /* TEST_MODE */

    default:
        status = LMP_HANDLE_1_2_INCOMING_PDU(lmp_pdu_ptr, ce_index);
        if (status == BT_FW_ERROR)
        {
            status = LMP_HANDLE_2_1_INCOMING_PDU(lmp_pdu_ptr, ce_index);
            if (status == BT_FW_ERROR)
            {
                if (bz_auth_handle_security_pdus(lmp_pdu_ptr, ce_index)
                        == TRUE)
                {
                    return BT_FW_SUCCESS;
                }
                if (lmp_edtm_handle_encapsulated_pdus(lmp_pdu_ptr,
                                                      ce_index) == TRUE)
                {
                    return BT_FW_SUCCESS;
                }

#ifdef VER_3_0
                if (IS_BT30)
                {
                    if (LMP_HANDLE_3_0_INCOMING_PDU(lmp_pdu_ptr, ce_index)
                            != BT_FW_ERROR)
                    {
                        return BT_FW_SUCCESS;
                    }
                }
#endif
#ifdef _SUPPORT_VER_4_1_
                if (IS_BT41)
                {
                    if (LMP_HANDLE_4_1_INCOMING_PDU(lmp_pdu_ptr, ce_index)
                            != BT_FW_ERROR)
                    {
                        return BT_FW_SUCCESS;
                    }
                }
#endif

                if (pdu_opcode == LMP_ESCAPE4_OPCODE)
                {
                    UINT8 *pbuf = lmp_pdu_ptr->payload_content;
                    if (pbuf[1] == LMP_ACCEPTED_EXT_OPCODE)
                    {
                        RT_BT_LOG(RED, LMP_TASKS_MSG_ACCEPT_EXT, 2,
                                                 pbuf[2], pbuf[3]);
                        break;
                    }
                    else if (pbuf[1] == LMP_NOT_ACCEPTED_EXT_OPCODE)
                    {
                        RT_BT_LOG(RED, LMP_TASKS_MSG_NOT_ACCEPT_EXT, 2,
                                                 pbuf[2], pbuf[3]);
                        break;
                    }
                }

                RT_BT_LOG(RED, LMP_TASKS_UNKNOWN_PDU, 1, pdu_opcode);
#ifdef _DAPE_TEST_ACL_RX_FIFO_DBG
                // gpio_one_pull_high(1);
                // gpio_one_pull_low(1);
                UINT32 acl_rx_fifo_reg;
                acl_rx_fifo_reg = RD_U32_BZDMA_REG(BZDMA_REG_ACL_RXFIFO_PTR);
                RT_BT_LOG(RED, DAPE_TEST_LOG213, 2,
                          BZDMA_REG_ACL_RXFIFO_PTR, acl_rx_fifo_reg);
#endif
            }
            break;
        }
    }

    if (OS_FREE_BUFFER(lmp_pdu_buffer_pool_handle, lmp_pdu_ptr) != BT_ERROR_OK)
    {
        RT_BT_LOG(GRAY, LMP_TASKS_1571, 0, 0);
    }

    return BT_FW_SUCCESS;
}

/**
 * Disconnect all the links associated with the ACL link given by \a ce_index
 * and then the ACL link itself.
 *
 * \param ce_index Index of the ACL connection entity database.
 * \param reason Reason for disconnection.
 *
 * \return None.
 */
void lmp_disconnect_links(UINT16 ce_index, UCHAR reason)
{
    LC_EXIT_SM_MODE();

#ifdef COMPILE_ESCO
    lmp_disconnect_esco_links(ce_index, reason);
#endif
#ifdef ENABLE_SCO
    lmp_disconnect_sco_links(ce_index, reason);
#endif
    return;
}

/**
 * Free all the resources associated with the detached ACL connection. It
 * doesn't detach the ACL connection itself, it just frees the data structures
 * and generates Disconnection_Complete/Connection_Complete event (if
 * necessary). It has to be called when the am_addr is ready to be freed.
 *
 * \param ce_index Index of the ACL connection entity database.
 *
 * \return None.
 */
void lmp_cleanup_after_acl_detach(UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    UCHAR lut_index;
    LC_PICONET_SCHEDULER *piconet_schd;

    ce_ptr = &lmp_connection_entity[ce_index];

    LC_EXIT_SM_MODE();

#ifdef COMPILE_SNIFF_MODE

    /* Kill Sniff as part of detach cleanup. LC need not be sure that
     * the baseband is in sniff mode here.
     */
    if (ce_ptr->in_sniff_mode == TRUE)
    {
        lut_index = lc_get_lut_index_from_phy_piconet_id(
                        ce_ptr->am_addr, ce_ptr->phy_piconet_id);

        ce_ptr->in_sniff_mode = FALSE;

        lc_exit_sniff_mode(ce_index);

        lc_sniff_cont_count[lut_index - 1] = 0;

#ifdef DETECT_SNIFF_OVERLAP
        if (lmp_self_device_data.no_of_sniff_connections != 0)
        {
            lmp_self_device_data.no_of_sniff_connections--;
        }
#endif
    }
#endif

    ce_ptr->sent_pdu = BIT_MASK_NO_PDU;
    ce_ptr->lmp_expected_pdu_opcode = BIT_MASK_NO_PDU;
    if (ce_ptr->pdu_response_timer_running == TRUE)
    {
        OS_STOP_TIMER(ce_ptr->lmp_response_timer_handle, 0);
        ce_ptr->pdu_response_timer_running = FALSE;
    }

    if (ce_ptr->remote_dev_role != MASTER)
    {

        /* This algorithm is applicable only for MASTER */
        piconet_schd =  &lc_piconet_scheduler[ce_ptr->phy_piconet_id];
        ce_ptr->pdu_response_timer_running = PDU_TIMER_AS_WAIT_FOR_CLEAR_PKT;
        /* Do not schedule more pkt in to this piconet */
        piconet_schd->donot_schedule_pkt++;
        piconet_schd->wait_for_clear_pkt_count = 0x0;
    }
#ifdef _DAPE_RST_AFH_MAP_AFTER_DETACH
    afh_state[ce_ptr->phy_piconet_id]=INITIAL;
    g_afh_times[ce_ptr->phy_piconet_id] = 0;
    afh_recovery_cnt[ce_ptr->phy_piconet_id] = 0;
    afh_recovery_cnt1[ce_ptr->phy_piconet_id] = 0;
    afh_recovery_cnt2[ce_ptr->phy_piconet_id] = 0;
    afh_recovery_cnt3[ce_ptr->phy_piconet_id] = 0;
    afh_recovery_cnt4[ce_ptr->phy_piconet_id] = 0;
    all_on_map_cnt[ce_ptr->phy_piconet_id] = 0;
#endif

    lmp_cleanup_after_acl_detach_if_no_scheduled_pkt(ce_index);
}

/* #define NO_DELAY_BB_FLUSH_FIX */
/**
 * Free all the resources associated with the detached ACL connection,
 * if there are no packets scheduled for this connection.
 *
 * \param ce_index Index of the ACL connection entity database.
 *
 * \return None.
 */
void lmp_cleanup_after_acl_detach_if_no_scheduled_pkt(UINT16 ce_index)
{
    OS_SIGNAL sig_send;
    LC_PICONET_SCHEDULER *piconet_schd = NULL;
    LMP_CONNECTION_ENTITY* ce_ptr = NULL;
    UCHAR piconet_id = 0;

#ifdef _CCH_IOT_FTP_PAGE_
	piconet_id = SCA_PICONET_INVALID;
#endif

    ce_ptr = &lmp_connection_entity[ce_index];

#ifdef COMPILE_AFH_HOP_KERNEL
#ifdef COMPILE_PARK_MODE
    /* free pending lmp park req pdu */
    if (ce_ptr->park_lmp_pdu_ptr != NULL)
    {
        OS_FREE_BUFFER(lmp_pdu_buffer_pool_handle, ce_ptr->park_lmp_pdu_ptr);
        ce_ptr->park_lmp_pdu_ptr = NULL;
    }
#endif

    /* free pending lmp role switch req pdu */
    if (ce_ptr->mss_pdu_ptr != NULL)
    {
        OS_FREE_BUFFER(lmp_pdu_buffer_pool_handle, ce_ptr->mss_pdu_ptr);
        ce_ptr->mss_pdu_ptr = NULL;
    }
#endif

    if (ce_ptr->remote_dev_role != MASTER)
    {
        /* This algorithm is applicable only for MASTER */
        piconet_id =  ce_ptr->phy_piconet_id;
        piconet_schd =  &lc_piconet_scheduler[piconet_id];

#ifndef NO_DELAY_BB_FLUSH_FIX
        RT_BT_LOG(GRAY, LMP_TASKS_1741, 1,
                  (LC_MAX_SCH_INFO - piconet_schd->lc_allowed_pkt_cnt));

        if (piconet_schd->wait_for_clear_pkt_count > MAX_LC_SCHEDULE_PKT_CLEAR_COUNT)
        {
            RT_BT_LOG(GRAY, LMP_TASKS_1745, 0, 0);
            piconet_schd->wait_for_clear_pkt_count = 0x0;
        }
#ifndef _DAPE_TEST_QUICK_DETACH
        else if ((piconet_schd->lc_allowed_pkt_cnt < LC_MAX_SCH_INFO) &&
             ((piconet_schd->lc_scheduled_pkt_info[0].ce_index == ce_index) ||
             (piconet_schd->lc_scheduled_pkt_info[1].ce_index == ce_index)))
#else
        else if ((piconet_schd->lc_allowed_pkt_cnt < LC_MAX_SCH_INFO) &&
             (((piconet_schd->lc_scheduled_pkt_info[0].ce_index == ce_index) &&
             (piconet_schd->lc_scheduled_pkt_info[0].tx_status == LC_TX_SENT))||
             ((piconet_schd->lc_scheduled_pkt_info[1].ce_index == ce_index) &&
              (piconet_schd->lc_scheduled_pkt_info[1].tx_status == LC_TX_SENT))))
#endif
        {
#ifndef _DAPE_TEST_QUICK_DETACH
            UCHAR lut_index;

            lut_index = lc_get_lut_index_from_ce_index(ce_index);
            if (lc_is_tpoll_started[lut_index] == FALSE)
            {
                /* Start Tpoll to ensure NBC Timeout */
                BB_start_tpoll(ce_ptr->am_addr, ce_ptr->Tpoll,
                               ce_ptr->phy_piconet_id);
            }
            piconet_schd->wait_for_clear_pkt_count++;
            OS_START_TIMER(ce_ptr->lmp_response_timer_handle,
                           TIME_WAIT_FOR_CLEAR_PKT);
            return;
#else
            UCHAR am_addr;

            DEF_CRITICAL_SECTION_STORAGE;

            MINT_OS_ENTER_CRITICAL();

            BB_disable_NBC(piconet_id);

            if (piconet_schd->lc_scheduled_pkt_info[0].tx_status == LC_TX_SENT)
            {
                am_addr = piconet_schd->lc_scheduled_pkt_info[0].selected_am_addr;
            }
            else
            {
                am_addr = piconet_schd->lc_scheduled_pkt_info[1].selected_am_addr;
            }

            if (am_addr == 0)
            {
                bzdma_invalid_txcmd(0, 0, 0);
            }
            else
            {
                bzdma_invalid_txcmd(piconet_id + BZDMA_TX_ENTRY_TYPE_PICONET0,
                                    am_addr, 0);
            }
            MINT_OS_EXIT_CRITICAL();
#endif
        }
#endif /* NO_DELAY_BB_FLUSH_FIX */

        ce_ptr->pdu_response_timer_running = 0;

        /* Decrement donot_schedule_pkt count so as to schedule pkt */
        piconet_schd->donot_schedule_pkt--;
    }

    lc_clear_queues(ce_index, TRUE);

    lc_kill_hardware_level_conn_in_scatternet(ce_index);

#ifdef _SUPPORT_PCA_ADJUST
    if (IS_BT41)
    {
        if (ce_ptr->remote_dev_role == MASTER)
        {
            if (bt_pca_manager.clk_adj_ce_index == ce_index)
            {
                bt_pca_manager.clk_adj_id = 0xFF;
            }
        }
        else
        {
            bt_pca_manager.slv_pca_support_bm &= ((~BIT0) << ce_index);
            if (bt_pca_manager.slv_pca_support_bm == 0)
            {
                lmp_cleanup_param_after_clk_adj_procedure();
            }
        }
    }
#endif

    /* Generate appropriate event to the host.*/
    if (ce_ptr->setup_complete_status == CONN_COMPLETE_EVENT)
    {
        hci_generate_disconnection_complete_event(
            HCI_COMMAND_SUCCEEDED,
            ce_ptr->connection_type.connection_handle,
            ce_ptr->disconnect_reason);
#ifdef _CCH_PAGE_CON_
            ce_ptr->connect_reason = 0;
#endif
    }
    else if (ce_ptr->hci_cmd_bits & REMOTE_NAME_REQ_CON_BIT_MASK)
    {
        hci_generate_remote_name_request_complete_event(
            ce_ptr->disconnect_reason,
            ce_index);

#ifdef _CCH_PAGE_CON_
        ce_ptr->connect_reason = 0;
#endif

        ce_ptr->hci_cmd_bits &= (~REMOTE_NAME_REQ_CON_BIT_MASK);
    }
    else if ( (ce_ptr->paging_completed_flag == TRUE) ||
              (ce_ptr->host_con_req_rx_flag == TRUE) )
    {
        hci_generate_connection_complete_event(
            ce_ptr->disconnect_reason,
            ce_ptr->connection_type.connection_handle,
            ce_ptr->bd_addr,
            ce_ptr->connection_type.link_type,
            (UCHAR)bz_auth_get_encryption_mode(ce_index));
        lmp_assigned_ce_index --;
    }

    LMP_REMOVE_BD_ADDR_FROM_HASH(ce_ptr->bd_addr);

    lmp_release_am_addr_ppi(ce_ptr->am_addr, ce_ptr->phy_piconet_id);
#ifdef _USE_SNIFF_NO_ACL_COUNT_REGISTER
#ifdef COMPILE_SNIFF_MODE
    if (lmp_self_device_data.no_acl_ce_index == ce_index)
    {
        lmp_self_device_data.no_acl_ce_index = 0xFF;
        lmp_self_device_data.no_acl_reduced_flag = FALSE;
    }
#endif
#endif

    lc_check_and_enable_scans_in_scatternet();

#ifdef _CCH_IOT_FTP_PAGE_
    if ( (piconet_id == SCA_PICONET_INVALID) ||
        ((piconet_schd != NULL) && (piconet_schd->donot_schedule_pkt == 0x0)))
#else
    if ((piconet_schd != NULL) && (piconet_schd->donot_schedule_pkt == 0x0))
#endif
    {
        /* Now we can resume data transfer */
        sig_send.type = LC_RESUME_DATA_TRANSFER_SIGNAL;
        sig_send.param = (void *)((UINT32)(piconet_id));

        OS_SEND_SIGNAL_TO_TASK (lc_tx_task_handle, sig_send);
    }

    return;
}

void lmp_handle_acl_disconnect_task(void *ce_index, uint32_t reason)
{
    lmp_handle_acl_disconnect((UINT16)(UINT32) ce_index, (UINT8) reason);
}

API_RESULT lmp_pend_acl_disconnect_from_isr(UINT16 ce_index, UCHAR reason)
{
    API_RESULT ret = API_FAILURE;
    BaseType_t high_pri_task_woken = pdFALSE;
    if (xTimerPendFunctionCallFromISR(lmp_handle_acl_disconnect_task,
            (void *)(UINT32) ce_index, reason, &high_pri_task_woken) == pdPASS)
    {
        ret = API_SUCCESS;
    }
    portYIELD_FROM_ISR(high_pri_task_woken);
    return ret;
}

/**
 * Start the detach procedure for ACL link specified by \a ce_index.
 *
 * \param ce_index Index of the ACL connection entity database.
 * \param reason Reason for disconnection.
 *
 * \return API_SUCCESS, if the operation was successful. API_FAILURE,
 *         otherwise.
 */
API_RESULT lmp_handle_acl_disconnect(UINT16 ce_index, UCHAR reason)
{
    LMP_CONNECTION_ENTITY *ce_ptr;
    TimerHandle_t detach_connection_timer = NULL;
    API_RESULT status;
    UCHAR parameter_list[LMP_DETACH_LEN];

    ce_ptr = &lmp_connection_entity[ce_index];
    ce_ptr->disconnect_reason = reason;

#ifdef _CCH_PAGE_CON_
    ce_ptr->connect_reason = 0;
#endif

    if (ce_ptr->detach_connection_timer_handle != NULL)
    {

#ifdef _CCH_IOT_FTP_PAGE_
        RT_BT_LOG(BLUE, CCH_DBG_090, 1,ce_index);
        return API_SUCCESS;
#else
        RT_BT_LOG(RED, CCH_DBG_091, 1,ce_index);
        LC_LOG_INFO(LOG_LEVEL_LOW,DETACH_HANDLE_IS_NOT_INVALID,0,0);
        return API_FAILURE;

#endif
    }

    /* Free all the data buffers and LMP packet buffers */
    lc_free_lmp_pdus(ce_index, FALSE);

#ifdef COMPILE_PARK_MODE
    if (ce_ptr->ce_status == LMP_PARK_MODE)
    {
        OS_SIGNAL signal;

        LMP_INF(PARKED_DEVICE_SUPERVISION_TIMER_EXPIRED,0,0);
        signal.type = LMP_ABNORMAL_EXIT_PARK_SIGNAL;
        signal.param = (OS_ADDRESS)((UINT32)ce_index);
        signal.ext_param = (OS_ADDRESS)((UINT32)reason);
        OS_ISR_SEND_SIGNAL_TO_TASK(lmp_task_handle, signal);

        return API_SUCCESS;
    }
#endif /* COMPILE_PARK_MODE */

    switch(ce_ptr->ce_status)
    {
        case LMP_DISCONNECTING:
            LC_LOG_INFO(LOG_LEVEL_LOW, DETACH_CE_STATUS_IS_ALREADY_DISCONNECTING,0,0);
            return API_FAILURE;

        default:
            lmp_set_ce_status(ce_index, LMP_DISCONNECTING);
            break;
    }

    parameter_list[0] = LMP_DETACH_OPCODE;
    parameter_list[2] = reason;

    lmp_generate_pdu(ce_index, parameter_list, LMP_DETACH_LEN, SELF_DEV_TID,
                     LMP_NO_STATE_CHANGE);

    /* These timers are not required, because the connection is going to be
     * detached anyway.
     */
    lmp_stop_connection_timers(ce_index);

    /* Create a disconnect timer */
    if (OS_CREATE_TIMER(ONESHOT_TIMER, &detach_connection_timer,
            lmp_detach_connection_timer_handler, (void *)((UINT32)ce_index), 0) != BT_ERROR_OK)
    {
        LC_LOG_INFO(LOG_LEVEL_LOW,DETACH_TIMER_CREATION_FAILED,0,0);
        return API_FAILURE;
    }

    ce_ptr->detach_connection_timer_handle = detach_connection_timer;

    {
        UINT32 t;

        UINT32 detach_multiplier = 6;

        detach_multiplier = detach_multiplier * (lmp_self_device_data.number_of_hlc + 1);

        t = detach_multiplier * (SLOT_VAL_TO_TIMER_VAL(ce_ptr->Tpoll) );

        status = OS_START_TIMER(detach_connection_timer, t);

        if (status != BT_ERROR_OK)
        {
            LC_LOG_INFO(LOG_LEVEL_LOW, DETACH_TIMER_STARTING_FAILED, 0, 0);
            return API_FAILURE;
        }

        RT_BT_LOG(GRAY, LMP_TASKS_1953, 1, t);
    }

    ce_ptr->detach_timer_state = SIX_TPOLL_STATE;

#ifdef COMPILE_NESTED_PAUSE_RESUME
    aclq_mark_am_addr_as_paused(ce_ptr->am_addr,
                                ce_ptr->phy_piconet_id, ACL_PAUSED_DETACH);
#else /* COMPILE_NESTED_PAUSE_RESUME */
    aclq_mark_am_addr_as_paused(ce_ptr->am_addr,
                                ce_ptr->phy_piconet_id);
#endif /* COMPILE_NESTED_PAUSE_RESUME */

    return API_SUCCESS;
}

/**
 * Generate and send number of completed packets event to the host.
 *
 * \param ce_index The ACL connection entity index.
 *
 * \return None.
 */
void lmp_handle_num_packets_timer(UINT16 ce_index)
{
    hci_generate_number_of_completed_packets_event();

    return;
}

#ifdef COMPILE_PARK_MODE
/**
 * Exit the park mode and free all the resources associated with the
 * connection. It does also generate the Disconnection_Complete_Event.
 *
 * \param ce_index Index to the ACL connection entity index.
 * \param disconnect_reason Reason for disconnection.
 *
 * \return None.
 */
void lmp_abnormal_exit_park_n_cleanup(UINT16 ce_index, UCHAR disconnect_reason)
{
    LMP_CONNECTION_ENTITY* ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    if (ce_ptr->remote_dev_role == SLAVE)
    {
        lmp_put_am_addr_ppi(ce_ptr->am_addr, ce_ptr->phy_piconet_id);
    }

    lmp_put_pm_addr(ce_ptr->pm_addr, ce_ptr->phy_piconet_id);
    lmp_put_ar_addr(ce_ptr->ar_addr, ce_ptr->phy_piconet_id);

    hci_generate_disconnection_complete_event(
        HCI_COMMAND_SUCCEEDED,
        ce_ptr->connection_type.connection_handle,
        disconnect_reason);

    if (lmp_self_device_data.number_of_parked_dev != 0)
    {
        lmp_self_device_data.number_of_parked_dev --;
    }

    LMP_INF(NUMBER_OF_PARKED_DEVICES,1,lmp_self_device_data.number_of_parked_dev);

    if (lmp_self_device_data.number_of_parked_dev == 0)
    {
        slave_ce_index = INVALID_CE_INDEX;
        lc_kill_beacon(ce_index);
    }

    if (lmp_self_device_data.number_of_parked_dev == 0)
    {
        lc_check_and_enable_scans_in_scatternet();
    }

    LMP_REMOVE_BD_ADDR_FROM_HASH(ce_ptr->bd_addr);

    lmp_release_entity_to_ce_database(ce_index);
    lmp_unpark_ce_index = INVALID_CE_INDEX;

    if ( (lmp_self_device_data.number_of_hlc == 0) &&
            (lmp_self_device_data.number_of_parked_dev == 0) )
    {
        /* Last connection is disconnected. */
        OS_SIGNAL sig_send;

        sig_send.type = LMP_LAST_CON_DISCONNECTED_SIGNAL;
        OS_SEND_SIGNAL_TO_TASK(lmp_task_handle, sig_send);
    }

    return;

}
#endif /* COMPILE_PARK_MODE */

/**
 * Release the AM_ADDR depending on the timeout occured.
 *
 * \param ce_index The ACL connection entity index.
 *
 * \return None.
 */
void lmp_supervision_timeout_handler(UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    LMP_INF(LMP_SUPERVISION_TIMER_AM_ADDR_PICONET_ID,2,
        ce_ptr->am_addr, ce_ptr->piconet_id);

    if (ce_ptr->pdu_response_timer_running == TRUE)
    {
        LMP_INF(STOPPING_PDU_RESP_TIMER_FOR_AM_ADDR,1,ce_ptr->am_addr);
        OS_STOP_TIMER(ce_ptr->lmp_response_timer_handle, 0);
        ce_ptr->pdu_response_timer_running = FALSE;
    }

#ifdef COMPILE_PARK_MODE
    if (ce_ptr->ce_status == LMP_PARK_MODE)
    {
        OS_SIGNAL signal;

        LMP_INF(PARKED_DEVICE_SUPERVISION_TIMER_EXPIRED,0,0);
        signal.type = LMP_ABNORMAL_EXIT_PARK_SIGNAL;
        signal.param = (OS_ADDRESS)((UINT32)ce_index);
        signal.ext_param = (OS_ADDRESS)CONNECTION_TIMEOUT_ERROR;
        OS_SEND_SIGNAL_TO_TASK(lmp_task_handle, signal);

        return;
    }
#endif /* COMPILE_PARK_MODE */

    LC_EXIT_SM_MODE();

    lmp_disconnect_links(ce_index, CONNECTION_TIMEOUT_ERROR);
    ce_ptr->disconnect_reason = CONNECTION_TIMEOUT_ERROR;
#ifdef _CCH_PAGE_CON_
    ce_ptr->connect_reason = 0;
#endif

    lmp_cleanup_after_acl_detach(ce_index);
}

/**
 * Send inquiry complete event after inquiry timeout.
 *
 * \param None.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_inquiry_timeout(void)
{
    LMP_INF(INQUIRY_TIMEOUT_SIGNAL_RECEIVED,0,0);

    /* Reset the inquiry command pending flag */
    hci_baseband_cmd_pending = FALSE;

    switch(lmp_self_device_data.device_status)
    {
#ifdef COMPILE_PERIODIC_INQUIRY
        case LMP_PERIODIC_INQUIRY :
            lmp_periodic_inquiry = FALSE ;         /* Fall through */
#endif
        case LMP_INQUIRY :
            /* Check this event is masked */
            hci_generate_inquiry_complete_event(HCI_COMMAND_SUCCEEDED);

            if (lmp_self_device_data.device_status == LMP_INQUIRY)
            {
                lmp_self_device_data.device_status = LMP_IDLE ;
            }
            break;

        default :
            LMP_ERR(INVALID_STATE_INQUIRY_TIMEOUT_SIGANL,0,0);
            return BT_FW_ERROR;
    }

    lmp_num_inq_resp_received = 0 ;

    /* Kill the inquiry */
    lc_kill_inquiry();

    return BT_FW_SUCCESS ;
}

/**
 * Handle the page timeout signal. It sends the command complete event/Remote
 * name request complete event with PAGE TIMOUT error.
 *
 * \param am_addr The Active Member Address of the device.
 * \param phy_piconet_id The Physical Piconet ID of the device.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_page_timeout(UCHAR am_addr, UCHAR phy_piconet_id)
{
    UINT16 ce_index;
    LMP_CONNECTION_ENTITY* ce_ptr;

    if (LMP_GET_CE_INDEX_FROM_AM_ADDR_PPI(am_addr, phy_piconet_id, &ce_index)
            != API_SUCCESS)
    {
        VER_ERR(PAGE_TIMEOUT_EXTRACTION_OF_CE_INDEX_FAILED,2,am_addr, phy_piconet_id);

        return BT_FW_ERROR;
    }

    ce_ptr = &lmp_connection_entity[ce_index];

    if (lmp_self_device_data.device_status != LMP_PAGE)
    {
        RT_BT_LOG(GRAY, LMP_TASKS_2280, 0, 0);
        return BT_FW_ERROR;
    }

    hci_baseband_cmd_pending = FALSE;
    switch (ce_ptr->ce_status)
    {
        case LMP_DURING_CONN_REMOTE_NAME_REQ:
            hci_generate_remote_name_request_complete_event(
                (UCHAR)PAGE_TIMEOUT_ERROR, ce_index );
            break;

        case LMP_PAGING:
            hci_generate_connection_complete_event(
                (UCHAR)PAGE_TIMEOUT_ERROR,
                ce_ptr->connection_type.connection_handle,
                ce_ptr->bd_addr,
                ce_ptr->connection_type.link_type,
                (UCHAR) bz_auth_get_encryption_mode(ce_index));
            break;
#ifdef _SUPPORT_CSB_RECEIVER_
        case LMP_TRUNCATED_PAGING:
            hci_generate_csa4_truncated_page_complete_event(
                (UCHAR)PAGE_TIMEOUT_ERROR, ce_index );
            break;
#endif
        default:
            LMP_ERR(INVALID_PAGE_TIME_OUT_OCCURED,0,0);
            break;
    }

    LMP_REMOVE_BD_ADDR_FROM_HASH(ce_ptr->bd_addr);

    if (lmp_release_am_addr_ppi(ce_ptr->am_addr,
                                ce_ptr->phy_piconet_id) != API_SUCCESS)
    {
        LMP_ERR(LOG_LEVEL_HIGH, LMP_RELEASE_AM_ADDR_FAILED,0,0);
        return BT_FW_ERROR;
    }

    lmp_assigned_ce_index--;
    lmp_self_device_data.device_status = LMP_IDLE;

    lc_check_and_enable_scans_in_scatternet();

#ifdef COMPILE_DYNAMIC_POLLING
    lc_update_dynamic_polling();
#endif
#ifndef _DAPE_TEST_NEW_HW_BLOCK_ACL_FOR_LE
#ifdef LE_MODE_EN
#ifdef _DAPE_TEST_BLOCK_LEGACY_FOR_LE
    /* dape test for LE slave + inquiry */
    ll_driver_block_legacy_for_le(FALSE);
#endif
#endif
#endif
    return BT_FW_SUCCESS;
}

/**
 * Handle the page response timeout. It frees the resources allocated during
 * page scan FHS packet reception.
 *
 * \param am_addr The Active Member Address of the device.
 * \param piconet_id The Logical Piconet ID of the device.
 *
 * \return None.
 */
void lmp_handle_page_resp_timeout(UCHAR am_addr, UCHAR phy_piconet_id)
{
    UINT16 ce_index = 0 ;
    LMP_CONNECTION_ENTITY *ce_ptr;

    if (LMP_GET_CE_INDEX_FROM_AM_ADDR_PPI(am_addr, phy_piconet_id,
                                         &ce_index) != API_SUCCESS)
    {
        LMP_LOG_INFO(LOG_LEVEL_HIGH, PGRESP_TO_AM_ADDR_TO_CE_INDEX_FAILED,0,0);
        return;
    }

    ce_ptr = &lmp_connection_entity[ce_index];

    if (ce_ptr->ce_status != LMP_PAGE_SCAN)
    {
        LMP_LOG_INFO(LOG_LEVEL_HIGH, PGRESP_TO_NOT_IN_PAGE_SCAN_STATE,0,0);
        return;
    }

    LMP_REMOVE_BD_ADDR_FROM_HASH(ce_ptr->bd_addr);

    if (lmp_release_am_addr_ppi(am_addr, ce_ptr->phy_piconet_id) != API_SUCCESS)
    {
        LMP_ERR(CANT_RELEASE_AM_ADDR,1, am_addr);
        return;
    }

    /* Connection handle is freed, decrement the assigned ce_index. */
    lmp_assigned_ce_index--;

    return;
}


#ifdef COMPILE_ROLE_SWITCH
/**
 * Handle the role switch FHS packets.
 *
 * \param lmp_fhs_pkt_recd Pointer to the FHS packet.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
 #if 0
UCHAR lmp_handle_role_switch_fhs_pkt(LMP_FHS_PKT_RECD *lmp_fhs_pkt_recd)
{
    UINT16 ce_index ;
    UCHAR am_addr ;
    UINT16 phy_piconet_id;

    /* New ce_index is allocated and updated the FHS Packet contents to
     * new ce_index. If the role_switch is not successfull(Not getting ACK
     * for the FHS pkt) then we are restoring the old connection entity in
     * new connection timeout timer handle.
     */
    ce_index = lmp_role_switch_data.ce_index;

    /*Extract the FHS Packet contents... store it in temporary place,
     * if the role switch is completed updatae it in connection entity....
     */
    am_addr = (UCHAR)(lmp_fhs_pkt_recd->fhs_pkt[14] & 0x07);

    lmp_role_switch_data.new_am_addr = am_addr;
    phy_piconet_id = lmp_role_switch_data.new_piconet_id;

    /* Store the previous piconet ID in role switch database and store the
     * newly allocated piconet ID in the connection entity. If the role switch
     * fails restore the same.
     */
    lmp_slave_use_am_addr_ppi(am_addr, phy_piconet_id, ce_index);

    /* Change the status of role switch */
    lmp_set_mss_state(LMP_MSS_M_TO_S_FHS_RECD);

    RT_BT_LOG(GRAY, LMP_TASKS_2471, 0, 0);

    return BT_FW_SUCCESS;
}
#endif
/**
 * Handles the connection procedure after Role-switch has completed during
 * connection.
 *
 * \param ce_index Connection entity index of the ACL link being established.
 *
 * \return None.
 */
void lmp_handle_mss_completion_during_conn_as_slave(UINT16 ce_index)
{
    lmp_accept_host_connection_request_pdu(ce_index);
}

TimerHandle_t reinit_max_slot_after_role_switch_tid_timer = NULL;
void role_switch_reinit_max_slot_timer_callback(TimerHandle_t timer_handle)
{
    UINT8 ce_index = (UINT8)((UINT32)pvTimerGetTimerID(timer_handle));
    LMP_CONNECTION_ENTITY* ce_ptr;

    /* free software timer */
    if (reinit_max_slot_after_role_switch_tid_timer != NULL)
    {
        OS_DELETE_TIMER(&reinit_max_slot_after_role_switch_tid_timer);
    }

////// dape test
#ifdef _DAPE_FIX_P900
    if (lmp_connection_entity[ce_index].entity_status == UNASSIGNED)
    {
        return;
    }
#endif
//////////////

    ce_ptr = &lmp_connection_entity[ce_index];

    /* Re-initialize max-slot. */
    ce_ptr->last_max_slot_req_sent = LMP_LAST_MAX_SLOT_REQ_SENT_INVALID;
    ce_ptr->last_max_slot_sent = LMP_MAX_SLOT_INVALID;

    /* Send max slot PDU */
    lmp_send_max_slot_pdu(ce_index);
    /* Send max slot request */
    lmp_send_max_slot_req_pdu(ce_index, 0xff);
}

/**
 * Handles the connection of MSS procedure, initiated after LM level
 * connection setup is completed.
 *
 * \param ce_index Connection entity index of the ACL link being established.
 *
 * \return None.
 */
void lmp_handle_mss_completion_after_connection(UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY* ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    lmp_set_ce_status(ce_index, LMP_CONNECTED);
    /* Re-invoke the scheduler to restart paused operation because of mss */

#ifndef _ENABLE_COMPILE_NESTED_PAUSE_RESUME_
#ifdef COMPILE_NESTED_PAUSE_RESUME
    aclq_resume_am_addr(ce_ptr->am_addr,
                        ce_ptr->phy_piconet_id, ACL_PAUSED_MSS);
#else /* COMPILE_NESTED_PAUSE_RESUME */
    aclq_resume_am_addr(ce_ptr->am_addr,
                        ce_ptr->phy_piconet_id);
#endif /* COMPILE_NESTED_PAUSE_RESUME */
#endif

#ifndef COMPILE_SINGLE_SLOT_PACKETS_ONLY
    /* use one shot sw timer to delay 1 sec for re-init max slot after role
       switch is successfully */

    if (reinit_max_slot_after_role_switch_tid_timer != NULL)
    {
        OS_DELETE_TIMER(&reinit_max_slot_after_role_switch_tid_timer);
    }

    OS_CREATE_TIMER(ONESHOT_TIMER, &reinit_max_slot_after_role_switch_tid_timer,
            role_switch_reinit_max_slot_timer_callback, (void *)((UINT32)ce_index), 0);

    OS_START_TIMER(reinit_max_slot_after_role_switch_tid_timer, 1000);
#endif

#ifndef _CCH_IOT_CSR_RS_
    LMP_DECIDE_FOR_LMP_AFH_PDU(ce_index);
#endif

    return;
}

TimerHandle_t enable_scan_after_role_switch_tid_timer = NULL;
void role_switch_timer_callback(TimerHandle_t timer_handle)
{
    UINT8 type = (UINT8)((UINT32)pvTimerGetTimerID(timer_handle));

    /* free software timer */
    if (enable_scan_after_role_switch_tid_timer != NULL)
    {
        OS_DELETE_TIMER(&enable_scan_after_role_switch_tid_timer);
    }

    if (type == 1)
    {
        /* Configure & Retrieve the scan */
        lc_handle_scan_mode_command();
    }
    else
    {
        lc_check_and_enable_scans_in_scatternet();
    }
}

/**
 * Handle role change completion signal and generate Role Change Completion
 * event.
 *
 * \param None.
 *
 * \return None.
 */
void lmp_handle_role_change_complete()
{
    UINT16 ce_index;

    LMP_CONNECTION_ENTITY* ce_ptr;

    UCHAR old_am_addr, new_am_addr;
    UCHAR old_piconet_id, new_piconet_id;

#ifdef _CCH_IOT_CSR_RS_QOS_
    UCHAR parameter_list[LMP_QoS_REQ_LEN];
#endif

    ce_index = lmp_role_switch_data.ce_index;

    ce_ptr = &lmp_connection_entity[ce_index];
#ifndef _ENABLE_COMPILE_NESTED_PAUSE_RESUME_
#ifdef COMPILE_NESTED_PAUSE_RESUME
    aclq_resume_am_addr(ce_ptr->am_addr,ce_ptr->phy_piconet_id, ACL_PAUSED_MSS);
#else /* COMPILE_NESTED_PAUSE_RESUME */
    aclq_resume_am_addr(ce_ptr->am_addr,ce_ptr->phy_piconet_id);
#endif /* COMPILE_NESTED_PAUSE_RESUME */
#endif
#ifdef POWER_CONTROL
    ce_ptr->dec_pow_pdu_drop_flag = FALSE;
    ce_ptr->inc_pow_pdu_drop_flag = FALSE;
#endif

    old_piconet_id = lmp_role_switch_data.old_piconet_id;
    new_piconet_id = lmp_role_switch_data.new_piconet_id;

    new_am_addr = lmp_role_switch_data.new_am_addr;
    old_am_addr = lmp_role_switch_data.old_am_addr;

    {
        DEF_CRITICAL_SECTION_STORAGE;
        MINT_OS_ENTER_CRITICAL();
        lmp_set_mss_state(LMP_MSS_INIT);
        MINT_OS_EXIT_CRITICAL();
    }

    /* Assign the new am_addr to the connection entity.*/
    ce_ptr->am_addr = new_am_addr;

    ce_ptr->phy_piconet_id = new_piconet_id;
#ifndef _ENABLE_COMPILE_NESTED_PAUSE_RESUME_
#ifdef COMPILE_NESTED_PAUSE_RESUME
    aclq_resume_am_addr(new_am_addr, new_piconet_id, ACL_PAUSED_MSS);
#else /* COMPILE_NESTED_PAUSE_RESUME */
    aclq_resume_am_addr(new_am_addr, new_piconet_id);
#endif /* COMPILE_NESTED_PAUSE_RESUME */
#endif
    /* Supervision timeout set to def value. */
    ce_ptr->link_supervision_timeout = otp_str_data.bt_supervision_timeout;

    lmp_slave_unuse_am_addr_ppi(old_am_addr, old_piconet_id);
    lmp_slave_use_am_addr_ppi(new_am_addr, new_piconet_id, ce_index);

    /* If we had not paused encryption previously (using EPR), we can do
     * what ever we want :). I mean generate role-change event.
     */
    if (!ce_ptr->is_enc_paused
            || ce_ptr->enc_proc == BZ_AUTH_ENCRYPTION_PROCEDURE_LEGACY)
    {

#ifdef _CCH_IOT_CSR_RS_
        ce_ptr->waiting_for_rs_several_times = 2;
        RT_BT_LOG(YELLOW, CCH_DBG_055, 1,ce_ptr->waiting_for_rs_several_times);
#else
        RT_BT_LOG(YELLOW, CCH_DBG_019, 0,0);
        hci_generate_role_change_event(HCI_COMMAND_SUCCEEDED,
                                       ce_ptr->bd_addr, ce_ptr->remote_dev_role);
#ifdef _ENABLE_COMPILE_NESTED_PAUSE_RESUME_
#ifdef COMPILE_NESTED_PAUSE_RESUME
        aclq_resume_am_addr(new_am_addr, new_piconet_id, ACL_PAUSED_MSS);
#else /* COMPILE_NESTED_PAUSE_RESUME */
        aclq_resume_am_addr(new_am_addr, new_piconet_id);
#endif /* COMPILE_NESTED_PAUSE_RESUME */
#else
        ce_ptr->pause_data_transfer = FALSE;
#endif
#endif
    }

    ce_ptr->remote_dev_role = (UCHAR)(ce_ptr->remote_dev_role ^ 1);

#ifdef _CCH_IOT_CSR_RS_QOS_
    /* Generate LMP_qos_req pdu. */
    parameter_list[0] = LMP_QoS_REQ_OPCODE;
    parameter_list[2] = LSB(ce_ptr->qos_tpoll);
    parameter_list[3] = MSB(ce_ptr->qos_tpoll);
    parameter_list[4] = ce_ptr->num_of_BC;

    lmp_generate_pdu(ce_index, parameter_list, LMP_QoS_REQ_LEN, SELF_DEV_TID,
            LMP_NO_STATE_CHANGE);
#endif

    /* Stop the supervision timer for the old am address */
    LMP_INF(STOPPING_SUPERVISION_TIMER_FOR_AM_ADDR,1,lmp_role_switch_data.old_am_addr);

    if (OS_STOP_TIMER(ce_ptr->supervision_timeout_handle, 0) != BT_ERROR_OK)
    {
        LMP_ERR(UNABLE_TO_STOP_SUPERVISION_TIMEOUT_FOR_AM_ADDR,1,
                lmp_role_switch_data.old_am_addr);
    }

    /* Start the supervision timeout */
    if (ce_ptr->link_supervision_timeout != 0)
    {
        LMP_INF(STARTING_SUPERVISION_TIMER_FOR_AM_ADDR,1,ce_ptr->am_addr);
        if (OS_START_TIMER(ce_ptr->supervision_timeout_handle,
             (UINT16)(SLOT_VAL_TO_TIMER_VAL(LMP_SUPERVISION_TIMER_RESOLUTION)))
                != BT_ERROR_OK)
        {
            LMP_ERR(UNABLE_TO_START_SUP_TO_FOR_AM_ADDR,1,ce_ptr->am_addr);
        }
    }

    init_1_2_connection_entity_after_hlc(ce_index);

    UINT8 type = 0;

    if (g_lc_scan_slot_timer_in_use == 0x1)
    {
        g_lc_scan_slot_timer_in_use = 0x0;
        type = 1;
    }

    /* use one shot sw timer to delay 1 sec the page/inquiry scan enable
       procedure after role switch is successfully */

    if (enable_scan_after_role_switch_tid_timer != NULL)
    {
        OS_DELETE_TIMER(&enable_scan_after_role_switch_tid_timer);
    }

    OS_CREATE_TIMER(ONESHOT_TIMER, &enable_scan_after_role_switch_tid_timer,
            role_switch_timer_callback, (void *)((UINT32)type), 0);

    OS_START_TIMER(enable_scan_after_role_switch_tid_timer, 1000);

    lc_start_tpoll_on_all_connections();

    switch (ce_ptr->ce_status)
    {
        case LMP_BB_HL_CONNECTED:
            break;

        case LMP_ROLE_SWITCHING:
            if (ce_ptr->is_enc_paused)
            {
                if (ce_ptr->auth_role == BZ_AUTH_ROLE_INITIATOR)
                {
                    /* We had paused the encryption previously (using either
                     * EPR or LEGACY method) and now we have to resume/restart
                     * the encryption.
                     * The role-change event would have been generated if we
                     * had used LEGACY method for pausing the encryption.
                     * Otherwise, it will be generated at the end of the
                     * resume encryption procedure
                     * (lmp_resume_encryption_callback()).
                     */
                    bz_auth_resume_encryption(ce_index);
                    return;
                }
                if (ce_ptr->enc_proc == BZ_AUTH_ENCRYPTION_PROCEDURE_EPR)
                {
                    /* We had used EPR to pause the encryption and hence defer
                     * the generation of the role-change event (we wouldn't
                     * have generated the role-change event on top -- in this
                     * case).
                     */
                    ce_ptr->mss_completion_status = HCI_COMMAND_SUCCEEDED;
                    return;
                }
            }

            /* Its time for wrapping up the role-switch */
            lmp_handle_mss_completion_after_connection(ce_index);
            break;

        case LMP_CONNECTION_ROLE_SWITCH:
            lmp_handle_mss_completion_during_conn_as_slave(ce_index);
            break;

        default:
            break;
    }

#ifdef COMPILE_DYNAMIC_POLLING
    lc_update_dynamic_polling();
#endif

/*
#ifdef ENABLE_LOGGER

    {
        UINT8 lc_no_of_conns = 0;
        UINT8 mid = 0;

        if (lc_sca_manager.master_cnt)
        {
            mid = lc_sca_manager.master_id;
            lc_no_of_conns = lmp_self_device_data.lc_no_of_connections[mid];
        }

        RT_BT_LOG(GRAY, LMP_TASKS_0380_1, 4, lc_sca_manager.master_cnt,
                  mid, lc_no_of_conns, lc_sca_manager.bm_slave);
    }

    RT_BT_LOG(GRAY, LMP_TASKS_2822, 6,
              ce_ptr->bd_addr[0], ce_ptr->bd_addr[1], ce_ptr->bd_addr[2],
              ce_ptr->bd_addr[3], ce_ptr->bd_addr[4], ce_ptr->bd_addr[5]);

#endif
*/

    /* Resume data transfer. */
    {
        OS_SIGNAL signal_send ;

        signal_send.type = LC_RESUME_DATA_TRANSFER_SIGNAL;
        /* Resume data transfer for all the piconets */
        signal_send.param = (void*) SCA_PICONET_INVALID;

        if (OS_SEND_SIGNAL_TO_TASK(lc_tx_task_handle, signal_send)
                !=BT_ERROR_OK)
        {
            LMP_ERR(log_file,"OS send signal to task failed.");
        }
    }

#ifdef COMPILE_AFH_HOP_KERNEL
    ce_ptr->last_set_afh_sent_clk = 0xFFFFFFFF;
#endif

#ifdef _ENABLE_RTK_PTA_
    pta_scan_bt_controller_role();
#endif

    return;
}

/**
 * Checks if the MSS instant is far away or fast approaching. If the instant is
 * far away, a timer is started, which will fire close to the instant.
 * Otherwise, MSS procedure is started immediately.
 *
 * \param None.
 *
 * \return None.
 */
void lmp_check_and_switch_role_in_scatternet(void)
{
    UINT32 native_clock = 0x0000;
    UINT16 con_reg;
    UCHAR new_am_addr, old_am_addr;
    UCHAR old_lut_index;
    UCHAR old_piconet_id, new_piconet_id;
    UINT16 ce_index;
    UCHAR instant_status;
    UCHAR ret_val = 0x0;
    LMP_CONNECTION_ENTITY* ce_ptr;

    DEF_CRITICAL_SECTION_STORAGE;

    ce_index = lmp_role_switch_data.ce_index;
#ifdef _DAPE_TEST_NO_ROLE_SW_AFTER_DETACH
    if (lmp_connection_entity[ce_index].entity_status == UNASSIGNED)
    {
        return;
    }
#endif
    old_am_addr = lmp_role_switch_data.old_am_addr;
    old_piconet_id = lmp_role_switch_data.old_piconet_id;
    old_lut_index = lmp_role_switch_data.old_lut_index;

    new_am_addr = lmp_role_switch_data.new_am_addr;
    new_piconet_id = lmp_role_switch_data.new_piconet_id;

    ce_ptr = &lmp_connection_entity[ce_index];

#ifdef COMPILE_AFH_HOP_KERNEL
    /* Do not allow MSS if AFH instant is pending. */
    if (OS_IS_TIMER_RUNNING(ce_ptr->afh_instant_timer_handle) == TRUE)
    {
        if (g_lc_scan_slot_timer_in_use == 0x1)
        {
            g_lc_scan_slot_timer_in_use = 0x0;

            /* Configure & Retrieve the scan */
            lc_handle_scan_mode_command();
        }

        lmp_handle_role_switch_failure(ce_index, DIFFERENT_TRANSACTION_COLLISION);
        RT_BT_LOG(GRAY, LMP_TASKS_2901, 0, 0);

        /* Clear cont_poll_count. Though this is set only as master,
           to optimise, we need not check for role here while clearing
           the flag. */
        ce_ptr->cont_poll_count = 0;
        return;
    }
#endif

    /* Start the switch timer now. */
    ret_val = lmp_start_switch_instant_timer(ce_index);
    switch(ret_val)
    {
        case 0x2:
#ifndef _ENABLE_COMPILE_NESTED_PAUSE_RESUME_
            /* Wait for MSS, Do not pause data transfer now */
#ifdef COMPILE_NESTED_PAUSE_RESUME
            aclq_resume_am_addr(ce_ptr->am_addr,
                                ce_ptr->phy_piconet_id, ACL_PAUSED_MSS);
#else /* COMPILE_NESTED_PAUSE_RESUME */
            aclq_resume_am_addr(ce_ptr->am_addr,
                                ce_ptr->phy_piconet_id);
#endif /* COMPILE_NESTED_PAUSE_RESUME */
            lc_pause_schedule_pkts = 0x0;
#endif
            return;

        case 0x1:
#ifndef _ENABLE_COMPILE_NESTED_PAUSE_RESUME_
            /* Wait for MSS, but pause data transfer now */
#ifdef COMPILE_NESTED_PAUSE_RESUME
            aclq_mark_am_addr_as_paused(ce_ptr->am_addr,
                                        ce_ptr->phy_piconet_id, ACL_PAUSED_MSS);
#else /* COMPILE_NESTED_PAUSE_RESUME */
            aclq_mark_am_addr_as_paused(ce_ptr->am_addr,
                                        ce_ptr->phy_piconet_id);
#endif /* COMPILE_NESTED_PAUSE_RESUME */
            lc_pause_schedule_pkts = 0x1;
#endif
            /* Stop tpoll on all other connections now. */
            lc_stop_tpoll_on_all_connections_except(ce_index);
            return;

        case 0x0:
        default:
#ifndef _ENABLE_COMPILE_NESTED_PAUSE_RESUME_
#ifdef COMPILE_NESTED_PAUSE_RESUME
            aclq_mark_am_addr_as_paused(ce_ptr->am_addr,
                                        ce_ptr->phy_piconet_id, ACL_PAUSED_MSS);
#else /* COMPILE_NESTED_PAUSE_RESUME */
            aclq_mark_am_addr_as_paused(ce_ptr->am_addr,
                                        ce_ptr->phy_piconet_id);
#endif /* COMPILE_NESTED_PAUSE_RESUME */
#endif
            /* proceed with MSS immediately */
            /* Stop tpoll on all other connections now. */
            lc_stop_tpoll_on_all_connections_except(ce_index);
#ifndef _ENABLE_COMPILE_NESTED_PAUSE_RESUME_
            lc_pause_schedule_pkts = 0x1;
#endif
            break;
    }

    /* Program the Baseband registers for TDD Switch */
    if (ce_ptr->remote_dev_role == MASTER)
    {
        /* Program new role as MASTER role. */
        if (new_piconet_id <= SCA_PICONET_MAX)
        {
            OR_val_with_bb_reg(reg_PICONET_INFO[new_piconet_id], BIT7);
        }
        else
        {
            RT_BT_LOG(RED, PICONET_ID_CHECK_MSG, 1, new_piconet_id);

            OR_val_with_bb_reg(PICONET2_INFO_REGISTER, BIT7);
        }

        lmp_set_mss_state(LMP_MSS_S_TO_M_SENT_FHS_PKT);

        /* Move the connection entity to the new am_addr.*/
        lmp_slave_use_am_addr_ppi(new_am_addr, new_piconet_id, ce_index);
    }
    else /* if (ce_ptr->remote_dev_role == MASTER) */
    {
        /* Program new role as SLAVE role. */
        if (new_piconet_id <= SCA_PICONET_MAX)
        {
            AND_val_with_bb_reg(reg_PICONET_INFO[new_piconet_id], ~(BIT7));
        }
        else
        {
            RT_BT_LOG(RED, PICONET_ID_CHECK_MSG, 1, new_piconet_id);

            AND_val_with_bb_reg(PICONET2_INFO_REGISTER, ~(BIT7));
        }

        lmp_set_mss_state(LMP_MSS_M_TO_S_WAITING_FOR_FHS);

        LC_WRITE_PHASE_OFFSET_VALUE((UINT16)(1250 - ce_ptr->slot_offset));

        /* Make sure that LUT has a valid pkt. */
        {
            UINT16 read;
            UINT16 address;

            address = lut_ex_table[old_lut_index].lower_lut_address;

            MINT_OS_ENTER_CRITICAL();

            read = BB_read_baseband_register(address);

            if (read == LC_INVALID_PACKET_TYPE)
            {
                BB_write_baseband_register(address,
                                           LC_MASTER_DEFAULT_PACKET_TYPE);
            }

            MINT_OS_EXIT_CRITICAL();
        }

        RT_BT_LOG(GRAY, LMP_TASKS_2986, 1, ((1250 - ce_ptr->slot_offset)));
    } /* if (ce_ptr->remote_dev_role == MASTER) */

#ifdef _USE_SNIFF_NO_ACL_COUNT_REGISTER
#ifdef COMPILE_SNIFF_MODE
    /* Reduce sniff no-acl to 3 temporarily. */
    BB_write_baseband_register(SNIFF_NO_ACL_COUNT_REGISTER, 0x03);

    lmp_self_device_data.no_acl_reduced_flag = TRUE;
    lmp_self_device_data.no_acl_ce_index = ce_index;
#endif
#endif

    lc_get_clock_in_scatternet(&native_clock, old_piconet_id);

    instant_status = lc_check_for_clock_wrap_around(native_clock,
                     (ce_ptr->switch_instant << 1) );

    if (instant_status == BT_CLOCK_CLK_WRAP_AROUND_CASE)
    {
        native_clock = (native_clock | (BT_CLOCK_27_BITS + 1));
    }

    while(1)
    {
        lc_get_clock_in_scatternet(&native_clock, old_piconet_id);

        if (instant_status == BT_CLOCK_CLK_WRAP_AROUND_CASE)
        {
            native_clock = (native_clock | (BT_CLOCK_27_BITS + 1));
        }

        native_clock = native_clock >> 1;

        if ((native_clock + 1) >= ce_ptr->switch_instant)
        {
            break;
        }
    }

    /* Check if the BB-FIFO has packets for this connection. If yes,
    then abort MSS.This is to avoid flushing of packets in BB-FIFO. */
    {
        UCHAR mss_abort;

        mss_abort = lc_check_for_aborting_mss(old_piconet_id);

        if (mss_abort == TRUE)
        {
            if (g_lc_scan_slot_timer_in_use == 0x1)
            {
                g_lc_scan_slot_timer_in_use = 0x0;
                /* Configure & Retrieve the scan */
                lc_handle_scan_mode_command();
            }
            lc_cleanup_mss_abort_at_instant();
#ifdef _DAPE_TEST_START_TPOLL_WHEN_ABORT_MSS
            lc_start_tpoll_on_all_connections();
#endif
            /* Clear cont_poll_count. Though this is set only as master,
               to optimise, we need not check for role here while clearing
               the flag. */
            ce_ptr->cont_poll_count = 0;
#ifndef _ENABLE_COMPILE_NESTED_PAUSE_RESUME_
            lc_pause_schedule_pkts = 0x0;
#endif
            return;
        }
    }

    BB_stop_tpoll(old_am_addr, old_piconet_id);

    MINT_OS_ENTER_CRITICAL();

    con_reg = BB_read_baseband_register(CONNECTOR_REGISTER);
    con_reg &= 0xE71F;

    if (ce_ptr->remote_dev_role == SLAVE)
    {
        /* Switching from master to slave. */
        con_reg |= (old_am_addr << 5) | (old_piconet_id << 11);
    }
    else if (lmp_mss_state == LMP_MSS_S_TO_M_SENT_FHS_PKT)
    {
        /* Switching from slave to master. */
        con_reg |= (new_am_addr << 5) | (old_piconet_id << 11);
    }

    con_reg |= BB_SET_TDD_BIT;
    con_reg &= 0x3FFF;

    /* for compatible reason, HW set Mss_pnid0 bit to bit15 and
       Mss_pnid1 bit to bit14 */
    if (old_piconet_id != new_piconet_id)
    {
        UINT8 i;
        UINT8 k = 0;
        for (i = 0; i < 4; i++)
        {
            if (i == old_piconet_id)
            {
                continue;
            }

            k++;

            if (i == new_piconet_id)
            {
                break;
            }
        }

        if (k < 3)
        {
            k = 3 - k;
        }
        con_reg |= k << 14;
    }

    BB_write_baseband_register(CONNECTOR_REGISTER, con_reg);

    LC_START_MS_SWITCH();

    MINT_OS_EXIT_CRITICAL();

    {
        UINT16 read;
        UINT16 address;

        address = lut_ex_table[old_lut_index].lower_lut_address;

        MINT_OS_ENTER_CRITICAL();

        read = BB_read_baseband_register(address);

        if ( (read != LC_INVALID_PACKET_TYPE) &&
                (ce_ptr->remote_dev_role == SLAVE) )
        {
            /* Warning. Any CRC pkt in LUT is lost. Anyway, BB is not
               supposed to be transmitting a CRC packet here. */
            BB_write_baseband_register(address, LC_INVALID_PACKET_TYPE);
        }

        MINT_OS_EXIT_CRITICAL();
    }

    /* Clear cont_poll_count. Though this is set only as master,
       to optimise, we need not check for role here while clearing
       the flag. */
    ce_ptr->cont_poll_count = 0;

    RT_BT_LOG(GRAY, MSS_MSG_PICOINFO, 4,
              BB_read_baseband_register(PICONET1_INFO_REGISTER),
              BB_read_baseband_register(PICONET2_INFO_REGISTER),
              BB_read_baseband_register(PICONET3_INFO_REGISTER),
              BB_read_baseband_register(PICONET4_INFO_REGISTER));

    return;
}

/**
 * Starts the role switch procedure in baseband. This function checks of
 * the instant is very far away, and if so, starts a timer to expire very
 * close to the instant. If the instant is fast approaching, then MSS procedure
 * is initiated.
 *
 * \param ce_index The ACL connection entity index.
 *
 * \return None.
 */
void lmp_switch_role_in_scatternet(UINT16 ce_index)
{
    UINT32 native_clock = 0x0000;
    UCHAR new_am_addr, old_am_addr;
    UCHAR old_lut_index, new_lut_index;
    UCHAR old_piconet_id, new_piconet_id;

    LMP_CONNECTION_ENTITY* ce_ptr;
    UCHAR instant_status;

    UINT32 lcl_switch_instant = 0;
#ifdef _DAPE_TEST_NO_ROLE_SW_AFTER_DETACH
    if (lmp_connection_entity[ce_index].entity_status == UNASSIGNED)
    {
        return;
    }
#endif
    g_lc_scan_slot_timer_in_use = 0x0;
    ce_ptr = &lmp_connection_entity[ce_index];
    old_am_addr = ce_ptr->am_addr;

    old_piconet_id = ce_ptr->phy_piconet_id;

    old_lut_index = lc_get_lut_index_from_phy_piconet_id(
                        old_am_addr, old_piconet_id);

    lc_get_clock_in_scatternet(&native_clock, old_piconet_id);

    lcl_switch_instant = (ce_ptr->switch_instant << 1);

#ifdef COMPILE_AFH_HOP_KERNEL
    /* Do not allow MSS if AFH instant is pending. */
    if (OS_IS_TIMER_RUNNING(ce_ptr->afh_instant_timer_handle) == TRUE)
    {
        lmp_handle_role_switch_failure(ce_index,
                                       DIFFERENT_TRANSACTION_COLLISION);
        RT_BT_LOG(GRAY, LMP_TASKS_3195, 0, 0);

        /* Clear cont_poll_count. Though this is set only as master,
           to optimise, we need not check for role here while clearing
           the flag. */
        ce_ptr->cont_poll_count = 0;
        return;
    }
#endif
#ifdef _DAPE_TEST_NO_ROLE_SW_AFTER_DETACH
    if (OS_IS_TIMER_RUNNING(ce_ptr->detach_connection_timer_handle) == TRUE)
    {
        lmp_handle_role_switch_failure(ce_index,
                                       DIFFERENT_TRANSACTION_COLLISION);
        RT_BT_LOG(GRAY, DAPE_TEST_LOG534, 0, 0);

        return;
    }
#endif
    if (lcl_switch_instant < 0x10)
    {
        lcl_switch_instant = lcl_switch_instant + 0x10000000;
    }
    lcl_switch_instant = lcl_switch_instant - 0x10;

    instant_status =
        lc_check_for_clock_wrap_around(native_clock, lcl_switch_instant);

    if (instant_status == BT_CLOCK_MORE_THAN_12_HRS_AWAY)
    {
        lmp_handle_role_switch_failure(ce_index, INSTANT_PASSED_ERROR);
        /* Clear cont_poll_count. Though this is set only as master,
           to optimise, we need not check for role here while clearing
           the flag. */
        ce_ptr->cont_poll_count = 0;
        return;
    }

    lc_kill_scan_mode();

#ifdef COMPILE_AFH_HOP_KERNEL
    lmp_role_switch_data.old_afh_mode = ce_ptr->afh_mode;
#endif
    lmp_role_switch_data.old_am_addr = old_am_addr;
    lmp_role_switch_data.ce_index = ce_index;

    lmp_role_switch_data.old_piconet_id = old_piconet_id;

    lmp_role_switch_data.old_lut_index = old_lut_index;

    lmp_role_switch_data.old_lower_lut_address =
        lut_ex_table[old_lut_index].lower_lut_address;
    lmp_role_switch_data.old_upper_lut_address =
        lut_ex_table[old_lut_index].upper_lut_address;

    new_piconet_id = lc_allocate_piconet_id_for_mss(ce_index);

    new_am_addr = lc_allocate_am_addr_for_mss(ce_index, new_piconet_id);

    new_lut_index = lc_allocate_lut_index_for_mss(ce_index,
                    new_piconet_id, new_am_addr);

    lmp_role_switch_data.new_am_addr = new_am_addr;
    lmp_role_switch_data.new_lut_index = new_lut_index;
    lmp_role_switch_data.new_piconet_id = new_piconet_id;

    lc_update_addresses_in_lut_ex_table_for_mss();

    lmp_check_and_switch_role_in_scatternet();

    LMP_LOG_INFO(LOG_LEVEL_HIGH, LMP_TASK_MSS, 6,
                 old_piconet_id, new_piconet_id,
                 old_am_addr, new_am_addr,
                 old_lut_index, new_lut_index);

    return;
}

#endif /* COMPILE_ROLE_SWITCH */


#ifdef COMPILE_HOLD_MODE
/**
 * Start the hold mode. It instructs the LC Module for starting the hold mode
 * and then sends the Hold Mode Change Event to the host.
 *
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_start_hold_mode(UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY *ce_ptr;
    UCHAR ret_val;

    ce_ptr = &lmp_connection_entity[ce_index];

    if (ce_ptr->ce_status == LMP_HOLD_MODE)
    {
        return BT_FW_SUCCESS;
    }

    ret_val = lc_start_hold_mode(ce_index);

    if (ret_val == API_SUCCESS)
    {
        lmp_set_ce_status(ce_index, LMP_HOLD_MODE);
    }

    ce_ptr->hold_mode_interval_negotiated = TRUE ;
    ce_ptr->hold_mode_accepted_flag = FALSE ;

    return BT_FW_SUCCESS ;
}

/**
 * Handles the hold instant interrupt.It sends the Hold Mode Change
 * Event to the host.
 *
 * \param ce_index ACL connection entity index.
 *
 * \return None.
 */
void lmp_handle_hold_instant_signal(UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY *ce_ptr;
    DEF_CRITICAL_SECTION_STORAGE;

    ce_ptr = &lmp_connection_entity[ce_index];

    /* Send mode change event */
    hci_generate_mode_change_event(
        HCI_COMMAND_SUCCEEDED, ce_index,
        LP_HOLD_MODE, ce_ptr->hold_mode_interval);

#ifdef POWER_SAVE_FEATURE
    LC_PROGRAM_HOLD_SM_MODE(ce_ptr->hold_mode_interval, ce_ptr->hold_instant);
#endif /* POWER_SAVE_FEATURE */

    /* Re-start the sup-to for the connection. */

    MINT_OS_ENTER_CRITICAL();
    lmp_sup_timeout_var[ce_index] = 0;
    MINT_OS_EXIT_CRITICAL();

    lmp_self_device_data.number_of_connections_in_hold_mode++;

    RT_BT_LOG(GRAY, LMP_TASKS_3552, 1, ce_ptr->phy_piconet_id);

    lc_check_and_enable_scans_in_scatternet();

    return;
}

/**
 * Handle the completion of the hold mode. It instructs the LC Module to
 * complete the hold mode and sends the Hold Mode Change Event to the host.
 *
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_complete_hold_mode(UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY *ce_ptr;
    LUT_EXTENSION_TABLE *ex_lut;
    UCHAR lut_index = 0;

    ce_ptr = &lmp_connection_entity[ce_index];

    if (ce_ptr->remote_dev_role == SLAVE)
    {
        lut_index = lc_get_lut_index_from_phy_piconet_id(
                        ce_ptr->am_addr, ce_ptr->phy_piconet_id);
        ex_lut = &lut_ex_table[lut_index];

        lc_check_and_update_pkt_in_lut(LC_MASTER_DEFAULT_PACKET_TYPE,
                                       LC_INVALID_PACKET_TYPE, ex_lut->lower_lut_address);

        ce_ptr->cont_poll_count = 0;

        //RT_BT_LOG(GRAY, LMP_TASKS_3597, 0, 0);
    }
    else
    {
        /* Reset xtol */
        BB_modify_xtol_in_scatternet(0x00, ce_ptr->phy_piconet_id);
    }

    BB_start_tpoll(ce_ptr->am_addr, ce_ptr->Tpoll, ce_ptr->phy_piconet_id);

    /* Generate mode change event */
    hci_generate_mode_change_event(HCI_COMMAND_SUCCEEDED,
                                   ce_index, LP_ACTIVE_MODE, 0);

    lmp_set_ce_status(ce_index, LMP_CONNECTED);
    ce_ptr->temp_ce_status = LMP_CONNECTED;

    if (ce_ptr->low_power_disconnect_state == HOLD_DISCONNECT)
    {
        lmp_handle_acl_disconnect(ce_index, ce_ptr->disconnect_reason);
    }
    else
    {
#ifdef COMPILE_NESTED_PAUSE_RESUME
        aclq_resume_am_addr(ce_ptr->am_addr,
                            ce_ptr->phy_piconet_id, ACL_PAUSED_HOLD);
#else /* COMPILE_NESTED_PAUSE_RESUME */
        aclq_resume_am_addr(ce_ptr->am_addr,
                            ce_ptr->phy_piconet_id);
#endif /* COMPILE_NESTED_PAUSE_RESUME */
    }

    if (lmp_self_device_data.number_of_connections_in_hold_mode != 0)
    {
        lmp_self_device_data.number_of_connections_in_hold_mode--;
    }

    lc_check_and_enable_scans_in_scatternet();

    RT_BT_LOG(GRAY, LMP_TASKS_3638, 1, ce_ptr->phy_piconet_id);

    return BT_FW_SUCCESS ;
}
#endif /* COMPILE_HOLD_MODE */

#ifdef COMPILE_PARK_MODE

/**
 * Forms unpark Request pdu. This is formed in
 * lmp_connection_entity.lmp_pdu_pkt.
 *
 * \param ce_index    CE Index.
 * \param init_timing Can be 1 or 0. 1 to included Db parameter in
 *                    unpark_bd_address pdu.
 *
 * \return None.
 */
void lmp_create_unpark_req_pdu(UINT16 ce_index, UCHAR init_timing)
{
    LMP_PDU_PKT* lmp_pdu_pkt = &(lmp_connection_entity[ce_index].lmp_pdu_pkt);
    UINT32 clock_val ;
    UCHAR offset = 0;
    UCHAR timing_control_flag;
    UCHAR phy_piconet_id;
    UCHAR am_addr;
    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    am_addr = ce_ptr->am_addr;

    phy_piconet_id = ce_ptr->phy_piconet_id;
    lmp_pdu_pkt->piconet = phy_piconet_id;
    lmp_pdu_pkt->am_addr = BC_AM_ADDR;

    /*
     * Check the valid PM_Addr , If it is zero, unpark it using BD_ADDRESS.
     */
    if (ce_ptr->pm_addr == 0)
    {
        timing_control_flag = init_timing;
        if (init_timing == 1)
        {
            offset = 2;
            lmp_pdu_pkt->payload_content[offset] = LSB(ce_ptr->Dbeacon);
            lmp_pdu_pkt->payload_content[offset+1] = MSB(ce_ptr->Dbeacon);
        }
        lmp_pdu_pkt->payload_content[0] =
            (UCHAR)(LMP_UNPARK_BD_ADDR_REQ_OPCODE << 1);
        lmp_pdu_pkt->pdu_length = (UCHAR)(9 + offset);

        lmp_pdu_pkt->payload_content[2 + offset] = am_addr ;
        memcpy(&lmp_pdu_pkt->payload_content[3 + offset],
               &ce_ptr->bd_addr, LMP_BD_ADDR_SIZE);
    }
    else
    {
        timing_control_flag = 0x0;
        lmp_pdu_pkt->payload_content[0] =
            (UCHAR)(LMP_UNPARK_PM_ADDR_REQ_OPCODE << 1);
        lmp_pdu_pkt->pdu_length = (UCHAR)(4);

        lmp_pdu_pkt->payload_content[2] = am_addr ;
        lmp_pdu_pkt->payload_content[3] = ce_ptr->pm_addr;
    }

    clock_val = BB_read_native_clock();

    /* If MSB of master clock is 1 use Initialization 2 */
    if (clock_val & 0x08000000)
    {
        timing_control_flag |= 0x02;
    }
    /* Access window is present */
    timing_control_flag |= 0x04;
    lmp_pdu_pkt->payload_content[1] = timing_control_flag ;

    LMP_LOG_PDU(LOG_LEVEL_HIGH, LOG_PDU_INDEX, LOG_TX_DIR, 20, (UCHAR *)lmp_pdu_pkt);

    return;
}

UCHAR lcl_ar_addr = INVALID_AR_ADDR;
UCHAR lcl_piconet_id = INVALID_PICONET_ID;

/**
 * Handle the unpark request from remote slave.
 *
 * \param ar_addr The Access Request Address.
 * \param piconet_id The Physical Piconet ID.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
void lmp_handle_slave_unpark_req(UCHAR ar_addr, UCHAR piconet_id)
{
    UINT16 ce_index = 0;
    UCHAR am_addr ;
    UCHAR lut_index =0;
    UCHAR phy_piconet_id =0;
    UINT16 upper_lut_address;
    UINT16 lower_lut_address;
    UINT16  lut_contents;
    UINT16 upper_lut_contents;
    DEF_CRITICAL_SECTION_STORAGE;

    UINT16 address;
    LMP_CONNECTION_ENTITY *ce_ptr;

    RT_BT_LOG(GRAY, LMP_TASKS_3864_NEW, 3, ar_addr, piconet_id,
                                           lc_slave_init_unpark_pending);

    if (lc_slave_init_unpark_pending != FALSE)
    {
        ar_addr = lcl_ar_addr;
        piconet_id = lcl_piconet_id;
    }

    if ((ar_addr != INVALID_AR_ADDR) && (piconet_id != INVALID_PICONET_ID) )
    {
        if (LMP_GET_CE_INDEX_FROM_AR_ADDR(ar_addr, &ce_index, piconet_id)
                != API_SUCCESS)
        {
            return;
        }
    }

    RT_BT_LOG(GRAY, LMP_TASKS_3887_NEW, 4,
              ar_addr,piconet_id,lmp_connection_entity[ce_index].am_addr,
              lc_slave_init_unpark_pending);

    if (lc_slave_init_unpark_pending == FALSE)
    {
        lc_slave_init_unpark_pending = TRUE;

        lc_exit_beacon(lmp_connection_entity[ce_index].am_addr);

        lcl_ar_addr = ar_addr;
        lcl_piconet_id = piconet_id;
        return;
    }
    else
    {
        lc_slave_init_unpark_pending = FALSE;
        lcl_ar_addr = INVALID_AR_ADDR;
        lcl_piconet_id = INVALID_PICONET_ID;
    }

    ce_ptr = &lmp_connection_entity[ce_index];

    if (ce_ptr->remote_dev_role != SLAVE)
    {
        return;
    }

    /* Get free Active member address */
    am_addr = ce_ptr->am_addr;

    lut_ex_table[BC_AM_ADDR].index_in_CE = ce_index;

    phy_piconet_id = ce_ptr->phy_piconet_id;

    if (phy_piconet_id <= SCA_PICONET_MAX)
    {
        lut_index = LC_SCA_SLAVE_1_LUT + phy_piconet_id;
    }
    else
    {
        RT_BT_LOG(RED, PICONET_ID_CHECK_MSG, 1, phy_piconet_id);

        lut_index = LC_SCA_SLAVE_2_LUT;
    }

    lc_update_ce_index_to_lut_extn_tbl(ce_index, lut_index);

    if (bz_auth_is_link_encrypted(ce_index))
    {
        bz_auth_enable_link_level_encryption(ce_index, TRUE);
    }

    ce_ptr->supto_auto_repark = FALSE;

    /* Store the PDU, At the start of the beacon instant it is transmitted. */
    if (ce_ptr->unpark_req_flag == LMP_UNPARK_IDLE)
    {
        lmp_create_unpark_req_pdu(ce_index, 1);

        /*
         * Only when the device receives LMP_ACCEPTED for unpark request,
         * it will go to the active mode. till that time it is in LMP_PARK
         * mode only.
         */
        MINT_OS_ENTER_CRITICAL();
        ce_ptr->unpark_req_flag = LMP_UNPARK_HOST_INITIATED;
        MINT_OS_EXIT_CRITICAL();

        /* This function updates the lmp_unpark_ce_index global variable. */
        lmp_unpark_ce_index = ce_index;

        /* Load the pdu into the BB fifo. */
        {
            LMP_PDU_PKT *ppkt;
            ppkt = &lmp_connection_entity[lmp_unpark_ce_index].lmp_pdu_pkt;
            memcpy(&bzdma_tx_buf[phy_piconet_id][0], ppkt->payload_content,
                   ppkt->pdu_length);
            BB_write_baseband_TX_FIFO_scatternet (
                bzdma_tx_buf[phy_piconet_id], ppkt->pdu_length,
                BC_AM_ADDR, phy_piconet_id);
        }

        lut_contents = (BB_DM1 << 12) | BB_LMP_PKT |
                       ce_ptr->lmp_pdu_pkt.pdu_length;
        address = (UINT16) (PICONET_LOOK_UP_TABLE_BASE_ADDR);
        BB_write_baseband_register(address, lut_contents);

        lut_contents = BB_read_baseband_register(address + 2);
        lut_contents &= 0xE1FF;
        lut_contents |= MAX_RADIO_TX_POWER << 9;

#ifdef _SUPPORT_LBT_FUNC_FOR_RF_ADAPTIVITY_
        if (rtl8723_btrf_check_and_enable_lbt(MAX_RADIO_TX_POWER))
        {
            lut_contents |= BIT12;
        }
#endif

        BB_write_baseband_register(address + 2, lut_contents);

        LC_SET_2M_TX_POWER(0,MAX_RADIO_TX_POWER);
        LC_SET_3M_TX_POWER(0,MAX_RADIO_TX_POWER);

        {
            UCHAR lut_index;

            if (ce_ptr->remote_dev_role == SLAVE)
            {
                lut_index = ce_ptr->am_addr;
            }
            else
            {
                if (ce_ptr->phy_piconet_id <= SCA_PICONET_MAX)
                {
                    lut_index = LC_SCA_SLAVE_1_LUT + ce_ptr->phy_piconet_id;
                }
                else
                {
                    RT_BT_LOG(RED, PICONET_ID_CHECK_MSG, 1, ce_ptr->phy_piconet_id);

                    lut_index = LC_SCA_SLAVE_1_LUT;
                }
            }

            lower_lut_address = lut_ex_table[lut_index].lower_lut_address;
            upper_lut_address = lut_ex_table[lut_index].upper_lut_address;
        }

        upper_lut_contents = BB_read_baseband_register(
                                 (UCHAR) upper_lut_address);

        /* Initialize arqn = 0 seqn = 0 and flow = 1 bits & active bit*/
        upper_lut_contents &= 0xFFF8;
        upper_lut_contents |= 0x0101;

#ifdef MASTER_AFH_PARK
        if (ce_ptr->afh_mode == AFH_ENABLE)
        {
            BB_write_afh_map(am_addr, ce_ptr->afh_map);
        }
#endif
        {
            UINT16 read_addr = 0;

            if (ce_ptr->phy_piconet_id <= SCA_PICONET_MAX)
            {
                read_addr = reg_PICONET_INFO[ce_ptr->phy_piconet_id];
            }
            else
            {
                RT_BT_LOG(RED, PICONET_ID_CHECK_MSG, 1, ce_ptr->phy_piconet_id);
                read_addr = PICONET2_INFO_REGISTER;
            }

            AND_val_with_bb_reg_macro(read_addr, (~0x000F) );

            OR_val_with_bb_reg_macro(read_addr, 0x01);

            RT_BT_LOG(GRAY, LMP_TASKS_4077, 2, read_addr, read_addr);
        }

#ifndef PARK_SLAVE_INIT_NO_POLL
        BB_write_baseband_register((UINT16)upper_lut_address,
                                   upper_lut_contents);

        /* Write POLL packet in the lower lut */
        BB_write_baseband_register((UINT16)lower_lut_address,
                                   LC_MASTER_DEFAULT_PACKET_TYPE);

#endif

        lc_exit_beacon(ce_index);

        /* If this var is zero, then we load unpark pdu into the bb fifo in
        end of access window interrupt. */
        lc_num_unpark_req_trials = 1;

        lc_park_setup_lc_level_conn_during_unpark_procedure();
    }

    RT_BT_LOG(GRAY, LMP_SLAVE_UNPARK_1, 3, ce_ptr->unpark_req_flag, ce_index, am_addr);

    if ( (ce_ptr->hci_unpark_req_flag == HCI_UNPARK_AUTO_CONTROLLER_INITIATED) ||
            (ce_ptr->hci_unpark_req_flag == HCI_UNPARK_IDLE) )
    {
        ce_ptr->hci_unpark_req_flag = HCI_UNPARK_REMOTE_INITIATED;
    }
    return;
}
#endif /* COMPILE_PARK_MODE */

#ifdef COMPILE_SNIFF_MODE
/**
 * Handle the BB_ACK for LMP_accepted(for LMP_unsniff_req).
 *
 * \param ce_index ACL Connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_unsniff_accepted_ack_recd(UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    /* Send mode change event */
    hci_generate_mode_change_event(
        HCI_COMMAND_SUCCEEDED,
        ce_index,
        LP_ACTIVE_MODE,
        0X00);

    lmp_exit_sniff_mode(ce_index);

#ifdef ENABLE_SCO
    lmp_determine_full_bandwidth();
#endif

    ce_ptr->cont_poll_count = 0;

    //RT_BT_LOG(GRAY, LMP_TASKS_4175, 0, 0);

    return BT_FW_SUCCESS ;
}

/**
 * Handle the BB_ACK for LMP_accepted(for LMP_sniff_req).
 *
 * \param ce_index ACL Connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_sniff_accepted_ack_recd(UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY *ce_ptr;
    ce_ptr = &lmp_connection_entity[ce_index];

    if (ce_ptr->temp_sniff_interval != 0xffff)
    {
        /* Reset parameters. */
        ce_ptr->temp_sniff_interval = 0xffff;
        ce_ptr->temp_sniff_nego_in_progress = FALSE;
    }


    /* Master will enter sniff mode on receiving BB-ack for lmp_accepted. */
//    if (ce_ptr->remote_dev_role == SLAVE)
    {
        /* Program LMP and LC to enter sniff mode. */
        lmp_start_sniff_mode(ce_index);

        /* Send mode change event */
        hci_generate_mode_change_event(
            HCI_COMMAND_SUCCEEDED,
            ce_index,
            LP_SNIFF_MODE,
            ce_ptr->sniff_interval);
    }
#if 0
    else
    {
        /* Clear sniff tn bit as slave. */
        lc_program_kill_sniff_transition_mode(ce_index);
    }
#endif

    return BT_FW_SUCCESS ;
}
#endif /* COMPILE_SNIFF_MODE */


/**
 * Handle the BB_ACK for LMP_detach pdu.
 *
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_detach_ack_recd(UCHAR ce_index)
{
    LMP_CONNECTION_ENTITY* ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];
    if (ce_ptr->disconnect_reason == CONNECTION_TERMINATED_USER_ERROR &&
            ce_ptr->detach_timer_state != REMOTE_DETACH_STATE)
    {
        ce_ptr->disconnect_reason = CONNECTION_TERMINATED_LOCAL_HOST_ERROR;
#ifdef _CCH_PAGE_CON_
        ce_ptr->connect_reason = 0;
#endif

    }

    if (ce_ptr->detach_timer_state == SIX_TPOLL_STATE)
    {
        //	LMP_INF(DETACH_PDU_ACK_RECD_KILL_THE_HLC,0,0);
        lmp_disconnect_links(ce_index, ce_ptr->disconnect_reason);

        //	LMP_INF(STOPPING_THE_6_TPOLL_DISCONNECT_TIMER,0,0);
        OS_STOP_TIMER(ce_ptr->detach_connection_timer_handle, 0);

        //	LMP_INF(STARTING_DISCONNECT_TIMER_FOR_3_TPOLL,0,0);

        OS_START_TIMER(ce_ptr->detach_connection_timer_handle,
                       (UINT16)(3*SLOT_VAL_TO_TIMER_VAL(ce_ptr->Tpoll)));

        ce_ptr->detach_timer_state = THREE_TPOLL_STATE;
    }
    else
    {
        RT_BT_LOG(GRAY, LMP_TASKS_4261, 2,
                  ce_index, ce_ptr->detach_timer_state);
    }

    return BT_FW_SUCCESS;
}

#ifdef COMPILE_ROLE_SWITCH
/**
 * Handle the BB_ACK for LMP_accepted (for LMP_switch_req).
 *
 * \param ce_index ACL connection entity index.
 *
 * \return None.
 */
void lmp_switch_req_accepted_ack_recd(UINT16 ce_index)
{
#ifdef _DAPE_TEST_NO_ROLE_SW_AFTER_DETACH
    if (lmp_connection_entity[ce_index].entity_status == UNASSIGNED)
    {
        return;
    }
#endif
    lmp_switch_role_in_scatternet(ce_index);

    return;
}


/**
 * Handle the BB_ACK for LMP_not_accepted (for LMP_switch_req).
 *
 * \param ce_index ACL connection entity index.
 *
 * \return None.
 */
void lmp_switch_req_not_accepted_ack_recd(UINT16 ce_index)
{
    /* Clear cont_poll_count. Though this is set only as master,
       to optimise, we need not check for role here while clearing
       the flag. */
    lmp_connection_entity[ce_index].cont_poll_count = 0;

    return;
}
#endif /* COMPILE_ROLE_SWITCH */


/**
 * Handle the BB_ACK for all the LMP PDUs.
 *
 * \param lmp_pdu_pkt Pointer to the LMP PDU packet.
 * \param piconet_id The Physical Piconet ID.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_pdu_ack_recd(LMP_PDU_PKT *lmp_pdu_pkt, UCHAR piconet_id, UINT32 debug_clock)
{
    UINT16 pdu_opcode;
    UCHAR am_addr;
    UINT16 sent_pdu_opcode;
    UINT16 ce_index;
    UCHAR phy_piconet_id;

    phy_piconet_id = (UCHAR)lmp_pdu_pkt->piconet;

    if (phy_piconet_id != piconet_id)
    {
        LMP_LOG_INFO(LOG_LEVEL_HIGH,
                     LMP_TASK_PDU_ACK,
                     piconet_id, phy_piconet_id);
    }
    pdu_opcode = lmp_pdu_pkt->payload_content[0];
    am_addr = (UCHAR)lmp_pdu_pkt->am_addr;
    pdu_opcode = (UINT16)(pdu_opcode >> 1) ;
#ifdef _DAPE_TEST_CHK_CLK
RT_BT_LOG(WHITE, YL_DBG_HEX_5, 5, BB_read_native_clock(), debug_clock,
BB_read_baseband_register(0x22),
BB_read_baseband_register(0x24),
BB_read_baseband_register(0x26));
#endif

    RT_BT_LOG(BLUE, LMP_MSG_PDU_ACK, 6,
          debug_clock, pdu_opcode, lmp_pdu_pkt->payload_content[1], am_addr,
          phy_piconet_id, lmp_pdu_pkt->payload_content[0] & 0x01);

    if (LMP_GET_CE_INDEX_FROM_AM_ADDR_PPI(am_addr, phy_piconet_id, &ce_index)
            != API_SUCCESS)
    {
        RT_BT_LOG(GRAY, LMP_TASKS_4367, 2, am_addr,phy_piconet_id);

        pduq_dequeue_pdu(lmp_pdu_pkt);
        if (OS_FREE_BUFFER(lmp_pdu_buffer_pool_handle, lmp_pdu_pkt) != BT_ERROR_OK)
        {
            RT_BT_LOG(GRAY, LMP_TASKS_4372, 0, 0);
        }
        return BT_FW_ERROR;
    }

#ifdef _USE_SNIFF_NO_ACL_COUNT_REGISTER
#ifdef COMPILE_SNIFF_MODE
    if (lmp_self_device_data.no_acl_ce_index == ce_index)
    {
        OS_SIGNAL signal;

        lmp_self_device_data.no_acl_reduced_flag = FALSE;
        lmp_self_device_data.no_acl_ce_index = 0xFF;

        /* Post the signal here. */
        signal.type = LMP_SNIFF_NO_ACL_RESTORE_SIGNAL;
        OS_SEND_SIGNAL_TO_TASK(lmp_task_handle, signal);
    }
#endif
#endif

    switch(pdu_opcode)
    {
        case LMP_ACCEPTED_OPCODE :
            sent_pdu_opcode = lmp_pdu_pkt->payload_content[1];
            switch(sent_pdu_opcode)
            {
#ifdef COMPILE_HOLD_MODE
                case LMP_HOLD_REQ_OPCODE:
                    if (lmp_handle_start_hold_mode(ce_index) == API_SUCCESS)
                    {
                        /* Change ce_status to LMP_HOLD_MODE */
                        lmp_set_ce_status(ce_index, LMP_HOLD_MODE);
                    }
                    break;
#endif /* COMPILE_HOLD_MODE */

#ifdef COMPILE_SNIFF_MODE
                case LMP_UNSNIFF_REQ_OPCODE :
                    lmp_unsniff_accepted_ack_recd(ce_index);
                    break;

                case LMP_SNIFF_REQ_OPCODE :
                    lmp_sniff_accepted_ack_recd(ce_index);
                    break;
#endif /* COMPILE_SNIFF_MODE */

#ifdef COMPILE_ROLE_SWITCH
                case LMP_SWITCH_REQ_OPCODE :
                    lmp_switch_req_accepted_ack_recd(ce_index);
                    break;
#endif

#ifdef ENABLE_SCO
                case  LMP_SCO_LINK_REQ_OPCODE:
#if 0
                    if (lmp_connection_entity[ce_index].remote_dev_role == MASTER)
                    {
                        lmp_handle_sco_link_req_accepted(ce_index);
                    }
#endif
                    /* Master updates the full-bandwidth flags upon receiving
                     * the LMP_accepted(LMP_sco_link_req) whereas the slave
                     * waits for the BB_ACK of LMP_accepted(LMP_sco_link_req)
                     * to update the same.
                     */
#ifdef COMPILE_SNIFF_MODE
                    lmp_determine_full_bandwidth();
#endif
                    break;
#endif

                default:
                    break;
            }
            break ;

        case LMP_NOT_ACCEPTED_OPCODE :
            sent_pdu_opcode = lmp_pdu_pkt->payload_content[1];
            switch(sent_pdu_opcode)
            {
#ifdef COMPILE_ROLE_SWITCH
                case LMP_SWITCH_REQ_OPCODE :
                    lmp_switch_req_not_accepted_ack_recd(ce_index);
                    break;
#endif

                default:
                    break;
            }
            break;

        case LMP_DETACH_OPCODE :
            lmp_handle_detach_ack_recd((UCHAR)ce_index);
            break;

#ifdef COMPILE_HOLD_MODE
        case LMP_HOLD_OPCODE:
            if (lmp_connection_entity[ce_index].remote_dev_role == SLAVE)
            {
                lmp_handle_start_hold_mode(ce_index);
            }
            break;
#endif

#ifdef POWER_CONTROL
        case LMP_DECR_POWER_REQ_OPCODE: /* Fall through. */
        case LMP_INCR_POWER_REQ_OPCODE:
            lmp_connection_entity[ce_index].power_ctrl_pdu_sent_flag = FALSE;
            break;

        case LMP_MIN_POWER_OPCODE:
            lmp_connection_entity[ce_index].dec_pow_pdu_drop_flag = FALSE;
            break;

        case LMP_MAX_POWER_OPCODE:
            lmp_connection_entity[ce_index].inc_pow_pdu_drop_flag = FALSE;
            break;
#endif

#ifdef ENABLE_SCO
        case LMP_REMOVE_SCO_LINK_REQ_OPCODE:
            //lmp_handle_remove_sco_link_req_ack_recd(ce_index);
            break;
#endif

        case LMP_SUPERVISION_TIMEOUT_OPCODE:
            lmp_update_supervision_timer(ce_index);
#ifdef COMPILE_SNIFF_MODE
            lmp_ssr_disable(ce_index);
#endif
            break;

#ifdef COMPILE_CQDDR
        case LMP_AUTO_RATE_OPCODE:
            if (!IS_USE_FOR_MUTE)
            {
                lmp_connection_entity[ce_index].sent_autorate_pdu = TRUE;
            }
            break;
#endif
#ifdef _DAPE_TEST_EXIT_SNIFF_EARLIER
        case LMP_UNSNIFF_REQ_OPCODE:
            lmp_unsniff_accepted_ack_recd(ce_index);
            break;
#endif
        default:
            LMP_HANDLE_1_2_PDU_ACK_RECD(lmp_pdu_pkt, piconet_id, ce_index);
            break;
    }

    pduq_dequeue_pdu(lmp_pdu_pkt);
    if (OS_FREE_BUFFER(lmp_pdu_buffer_pool_handle, lmp_pdu_pkt) != BT_ERROR_OK)
    {
        RT_BT_LOG(GRAY, LMP_TASKS_4526, 0, 0);
    }

    return BT_FW_SUCCESS;
}

/**
 * Handle LMP PDU sent signal. This signal will be posted whenever a pdu is
 * loaded into BB FIFO for transmission (Basically BB_SEND_PACKET was issued
 * for the PDU).
 *
 * \param lmp_pdu_pkt LMP PDU Packet for which BB_SEND_PACKET was issued.
 *
 * \return None.
 *
 * \note \a lmp_pdu_pkt will be freed by #lmp_handle_pdu_ack_recd handler and
 *          it should not be freed here.
 *
 * \warning Both #LMP_PDU_SENT_SIGNAL and #LC_PDU_ACK_RECD_SIGNAL should be
 *          handled by a single task. It is required because
 *          LC_PDU_SENT_SIGNAL should be handled before LMP_PDU_RECD_SIGNAL.
 *          If it happens the other way then \a lmp_pdu_pkt might be stale.
 */
void lmp_handle_pdu_sent(LMP_PDU_PKT* lmp_pdu_pkt)
{
    UCHAR am_addr;
    UCHAR pdu_opcode;
    UINT16 ce_index;

    am_addr = lmp_pdu_pkt->am_addr;

    pdu_opcode = (UCHAR)(lmp_pdu_pkt->payload_content[0] >> 1);

    if (LMP_GET_CE_INDEX_FROM_AM_ADDR_PPI(am_addr, lmp_pdu_pkt->piconet, &ce_index)
            != API_SUCCESS)
    {
        BZ_ASSERT(0, "Unable to get ce_index from am_addr & piconet_id");
        return;
    }

    switch (pdu_opcode)
    {
        case LMP_ACCEPTED_OPCODE:
        {
            UCHAR accepted_opcode;

            accepted_opcode = lmp_pdu_pkt->payload_content[1];
            switch (accepted_opcode)
            {
#ifdef ENABLE_SCO
                case LMP_SCO_LINK_REQ_OPCODE:
                    lmp_handle_sco_link_req_accepted_sent(ce_index);
                    break;
#endif

                default:
                    break;

            } /* end switch(accepted_opcode) */
            break;
        }

        case LMP_DETACH_OPCODE:
            lmp_handle_detach_pdu_sent(ce_index);
            break;

        default:
            break;
    } /* end switch(pdu_opcode) */
}

#ifdef ESCO_DISC_DEBUG
/**
 * Handle end of ESCO window.
 *
 * \param esco_lt_addr ESCO Active Member Address (a.k.a LT_ADDR).
 * \param logical_piconet_id The Logical Piconet ID.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
int lmp_handle_end_of_esco_window(UINT16 esco_lt_addr,
                                  UCHAR logical_piconet_id)
{
    UINT16 esco_ce_index = 0;
    UINT16 acl_ce_index = 0;

    if (lmp_get_esco_ce_index_from_am_addr(esco_lt_addr,logical_piconet_id,
                                          &esco_ce_index) != API_SUCCESS)
    {
        ESCO_ERR(COULD_NOT_GET_THE_ESCO_CE_INDEX_FROM_LT_ADDR,0,0);
        return BT_FW_ERROR;
    }

    acl_ce_index = lmp_esco_connection_entity[esco_ce_index].ce_index;

    /* Kill the baseband level connection. */
    lc_kill_esco_connection(esco_ce_index);

    /* free the esco buffers */
    reset_esco_ce_buffer(esco_ce_index);

    lmp_put_esco_am_addr(lmp_esco_connection_entity[esco_ce_index].lt_addr,
                         lmp_connection_entity[acl_ce_index].phy_piconet_id);

    lmp_release_esco_entity_to_ce_database(esco_ce_index);

    lmp_update_max_tesco();
    lmp_update_gcd_of_tesco();

#ifndef COMPILE_SINGLE_SLOT_PACKETS_ONLY
    /* Change the max slot value for the acl link*/
    change_max_slot_with_esco(acl_ce_index);
#endif

    lmp_send_lmp_accepted_ext(acl_ce_index,LMP_ESCAPE4_OPCODE,
                              LMP_REMOVE_ESCO_LINK_REQ_OPCODE,
                              transaction_id_of_disconnection);

    if ((hci_generate_disconnection_complete_event(HCI_COMMAND_SUCCEEDED,
            lmp_esco_connection_entity[esco_ce_index].conn_handle,
            reason_for_disconnection)) == API_FAILURE)
    {
        ESCO_ERR(DISCONNECTION_EVENT_GENERATION_FAILED,0,0);
        return BT_FW_ERROR;
    }

    wait_for_esco_window_end_intr=FALSE;
    return BT_FW_SUCCESS;
}
#endif /* ESCO_DISC_DEBUG */

/**
 * Handles PDU_SENT_SIGNAL for lmp_detach_pdu.
 *
 * \param ce_index ACL connection entity index.
 *
 * \return None.
 */
void lmp_handle_detach_pdu_sent(UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    if (ce_ptr->detach_timer_state == SIX_TPOLL_STATE)
    {
        OS_START_TIMER(ce_ptr->detach_connection_timer_handle,
                       (UINT16)(6*SLOT_VAL_TO_TIMER_VAL(ce_ptr->Tpoll)));
    }

    RT_BT_LOG(GRAY, LMP_TASKS_4809, 1, ce_index);

    return;
}

#ifdef COMPILE_PARK_MODE
/**
 * Handles LMP_PARK_NCTO_SIGNAL to lmp-task.
 *
 * \param ce_index ACL connection entity index.
 *
 * \return None.
 */
void lmp_handle_park_ncto_signal(UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    RT_BT_LOG(GRAY, LMP_TASKS_4828, 2,
              ce_index, ce_ptr->hci_unpark_req_flag);

    if (ce_ptr->hci_unpark_req_flag == HCI_UNPARK_REMOTE_INITIATED)
    {
        /* Nothing to do. Remote device is expected to initiate unpark again. */
        ce_ptr->unpark_req_flag = LMP_UNPARK_IDLE;
    }
    else if (ce_ptr->hci_unpark_req_flag == HCI_UNPARK_AUTO_CONTROLLER_INITIATED)
    {
        /* Change the unpark_req_flag to idle. Do not change
           ce_ptr->auto_unpark_cnt, EoAW intr will take care
           of the next attempt to unpark. */
        ce_ptr->unpark_req_flag = LMP_UNPARK_IDLE;
    }
    else if (ce_ptr->hci_unpark_req_flag == HCI_UNPARK_HOST_INITIATED)
    {
        lmp_handle_host_initiated_unpark(ce_index);
    }

    return;
}


/**
 * Handles host initiated exit-park command. This function updates the
 * hci-unpark state machine. Depending on the role, initiates appropriate
 * LC level unpark.
 *
 * \param ce_index ACL connection entity index.
 *
 * \return None.
 */
UCHAR lmp_handle_host_initiated_unpark(UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY *ce_ptr;
    UCHAR ret_val;
    DEF_CRITICAL_SECTION_STORAGE;

    ce_ptr = &lmp_connection_entity[ce_index];

    RT_BT_LOG(GRAY, LMP_TASKS_4869, 2,
              ce_index, ce_ptr->hci_unpark_req_flag);

    if (ce_ptr->remote_dev_role == MASTER)
    {
        /* Program BB to send unpark-req-ind to the master. */
        if (ce_ptr->ar_addr == 0)
        {
            ce_ptr->hci_unpark_req_flag = HCI_UNPARK_IDLE;

            return UNSPECIFIED_ERROR;
        }

        MINT_OS_ENTER_CRITICAL();
        ce_ptr->unpark_req_flag = LMP_UNPARK_HOST_INITIATED;
        lmp_unpark_ce_index = ce_index;
        MINT_OS_EXIT_CRITICAL();

        return HCI_COMMAND_SUCCEEDED;
    }
    else
    {
        ret_val = lmp_handle_master_unpark(ce_index);

        if (ret_val == HCI_COMMAND_SUCCEEDED)
        {
            ce_ptr->supto_auto_repark = FALSE;
        }

        return ret_val;
    }

}

#endif /* COMPILE_PARK_MODE */
