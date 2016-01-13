/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  Contains utility functions for LMP module.
 */

/********************************* Logger *************************/
enum { __FILE_NUM__= 59};

/********************************* Logger *************************/


/* ========================= Include File Section ========================= */
#include "bt_fw_globals.h"
#include "lmp_internal.h"
#include "lc_internal.h"
#include "vendor.h"
#ifdef COMPILE_CHANNEL_ASSESSMENT
#include "lmp_ch_assessment.h"
#endif
#include "hci_vendor_internal.h"
#include "bz_debug.h"
#include "lmp_2_1.h"
#ifdef VER_3_0
#include "lmp_3_0.h"
#endif
#include "bz_auth.h"

#include "mem.h"
#include "bt_fw_acl_q.h"
#include "UartPrintf.h"

//#ifdef _DAPE_TEST_SLOT_OFFSET_FOR_LE
#include "le_hci_4_0.h"
//#endif

#include "otp.h"
#include "bt_fw_os.h"

#ifdef PTA_EXTENSION
#include "lc.h"
#include "pta_meter.h"
#endif
extern OS_HANDLE isr_extended_task_handle;
/* ========================= Preprpcessor definition Section ========================= */
#define LMP_MIN_RSSI_SAMPLES       100

/* ========================= Static function Section ========================= */


/* ==================== Structure declaration Section ===================== */



/* ===================== Variable Declaration Section ===================== */

UINT16 lmp_park_bd_addr_ce_index ;
UCHAR lmp_current_nb = 1;

extern UINT16 lmp_unpark_ce_index ;

extern UCHAR lc_send_poll_pkt_issued_am_addr;

extern void lmp_init_assessment_at_connection();
#ifdef _DAPE_SNIFF_PKT_TEST
extern UINT32 lc_get_least_sniff_interval(UINT16 *sniff_ce_index, UINT16 *sniff_attempt);
#else
extern UINT32 lc_get_least_sniff_interval(void);
#endif


/* ================== For rom code patch function point ================== */
#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8723_RCP_
PF_ROM_CODE_PATCH_FUNC rcp_lmp_validate_sniff_parms = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_lmp_validate_sniff_parms_1 = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_lmp_get_global_slot_offset = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_lmp_update_global_slot_offset = NULL;
#endif

#ifdef _CCH_8821_RCP_
PF_ROM_CODE_PATCH_VOID rcp_lmp_stop_regular_sw_timers = NULL;
PF_ROM_CODE_PATCH_VOID rcp_lmp_start_regular_sw_timers = NULL;

#endif

#endif

/* ================== Static Function Prototypes Section ================== */


/* ===================== Function Definition Section ====================== */
/**
 * Handles the LMP response timeout. It detaches the ACL link.
 *
 * \param timer_handle Timer handle.
 * \param index ACL connection entity index.
 *
 * \return None.
 */
void lmp_response_timeout_handler(TimerHandle_t timer_handle)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    UINT16 ce_index;

    ce_index = (UINT16)((UINT32)pvTimerGetTimerID(timer_handle));
    ce_ptr = &lmp_connection_entity[ce_index];

    if (ce_ptr->pdu_response_timer_running == PDU_TIMER_AS_WAIT_FOR_CLEAR_PKT)
    {
        /* PDU timer is used as wait for clear schedule pkts */
        lmp_cleanup_after_acl_detach_if_no_scheduled_pkt(ce_index);
        return;
    }


    LMP_LOG_INFO(LOG_LEVEL_HIGH, LMP_RESPONSE_TIMER_EXPIRES_CE_INDEX, 2, ce_index, ce_ptr->sent_pdu);
#ifdef _SUPPORT_SECURE_CONNECTION_
    if (ce_ptr->sent_pdu == BIT_MASK_LMP_PING_REQ_OPCODE)
    {
        ce_ptr->pdu_response_timer_running = FALSE;
        //hci_generate_authentication_payload_timeout_expired_event(ce_ptr->connection_type.connection_handle);
        //hci_generate_event(HCI_AUTH_PAYLOAD_TIMEOUT_EXPIRED_EVENT,2);
        return;
    }
#endif

    ce_ptr->disconnect_reason = LMP_RESPONSE_TIMEOUT_ERROR;
#ifdef _CCH_PAGE_CON_
    ce_ptr->connect_reason = 0;
#endif

#ifdef COMPILE_PARK_MODE
    if (ce_ptr->ce_status == LMP_PARK_MODE)
    {
        ce_ptr->low_power_disconnect_state = UNPARK_DISCONNECT;
        hci_handle_exit_park_mode_command(ce_index);

        return;
    }
#endif /* COMPILE_PARK_MODE */

#ifdef COMPILE_SNIFF_MODE
    if (ce_ptr->in_sniff_mode == TRUE)
    {
        ce_ptr->low_power_disconnect_state = UNSNIFF_DISCONNECT;
        hci_handle_exit_sniff_mode_command(ce_index);

        return;
    }
#endif /* COMPILE_SNIFF_MODE */

    ce_ptr->pdu_response_timer_running = FALSE;
    if (lmp_handle_acl_disconnect(ce_index, LMP_RESPONSE_TIMEOUT_ERROR)
            != API_SUCCESS)
    {
        LMP_ERR(LMP_HANDLE_ACL_DISCONNECT_FAILED,0,0);
    }

    return;
}

#ifdef COMPILE_PARK_MODE

/**
 * Sets the auto_unpark_cnt variable depending on Tb and
 * supervision timeout value.
 *
 * \param ce_index ACL connection entity index.
 *
 * \return None.
 */
void lmp_set_auto_unpark_cnt(UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY *ce_ptr;
    UINT16 lcl_sup_to_cnt = 0;
    UINT32 cur_clk, i;
    DEF_CRITICAL_SECTION_STORAGE;

    ce_ptr = &lmp_connection_entity[ce_index];

    lcl_sup_to_cnt = ce_ptr->link_supervision_timeout;

    /* Calc the time from now to the next beacon instant,
       and subtract it from the sup-to. */
    lc_get_clock_in_scatternet(&cur_clk, ce_ptr->phy_piconet_id);

    /* Check if tbeacon unit is clk1 or clk 0 */
    cur_clk %= ce_ptr->Tbeacon;
    cur_clk = ce_ptr->Tbeacon - cur_clk;

    lcl_sup_to_cnt = (UINT16) (lcl_sup_to_cnt - cur_clk);

    ce_ptr->auto_unpark_cnt = (UINT16)
                              ( lcl_sup_to_cnt / (ce_ptr->Tbeacon) );

    for (i=0; i<3; i++)
    {
        if (ce_ptr->auto_unpark_cnt != 0)
        {
            ce_ptr->auto_unpark_cnt--;
        }
    }

    MINT_OS_ENTER_CRITICAL();
    lmp_sup_timeout_var[ce_index] = 0;
    MINT_OS_EXIT_CRITICAL();
}
/**
 * Release the am_addr for parking and signals the LMP_TASK to delete the
 * timer handle.
 *
 * \param timer_handle Timer handle.
 * \param index ACL connection entity index.
 *
 * \return None.
 */
void lmp_park_mode_timer_handler(TimerHandle_t timer_handle)
{
    UINT16 ce_index;
    OS_SIGNAL signal_send;
    LMP_CONNECTION_ENTITY *ce_ptr;
    DEF_CRITICAL_SECTION_STORAGE;

    ce_index = (UINT16)((UINT32)pvTimerGetTimerID(timer_handle));
    LMP_INF(TPOLL_X6_TIMER_EXPIRED_ENTERING_PARK_MODE,0,0);
    ce_ptr = &lmp_connection_entity[ce_index];

    /* Update the connection entity status. */
    ce_ptr->park_mode_negotiated = TRUE ;

    lmp_self_device_data.number_of_parked_dev++;
    lc_start_beacon(ce_index);

    lmp_set_ce_status(ce_index, LMP_PARK_MODE);

    /* Send Mode change event. */
    hci_generate_mode_change_event(
        HCI_COMMAND_SUCCEEDED,
        ce_index,
        LP_PARK_MODE,
        ce_ptr->Tbeacon);

    if ( (ce_ptr->link_supervision_timeout != 0) &&
            (ce_ptr->remote_dev_role == SLAVE) )
    {
        lmp_set_auto_unpark_cnt(ce_index);
    }
    else
    {
        MINT_OS_ENTER_CRITICAL();
        lmp_sup_timeout_var[ce_index] = 0;
        MINT_OS_EXIT_CRITICAL();
    }

    if(ce_ptr->remote_dev_role == MASTER)
    {
        lmp_put_am_addr_ppi(ce_ptr->am_addr, ce_ptr->phy_piconet_id);
    }

    /* Send signal to LMP task to delete the timer  */
    signal_send.type = LMP_DELETE_TIMER_HANDLE_SIGNAL;
    signal_send.param = (OS_ADDRESS) ( ce_ptr->park_mode_timer_handle);

    if(OS_SEND_SIGNAL_TO_TASK(lmp_task_handle, signal_send)
            != BT_ERROR_OK)
    {
        LMP_ERR(LOG_LEVEL_HIGH, OS_SEND_SIGNAL_TO_TASK_FAILED,0,0);
    }

    ce_ptr->park_mode_timer_handle = NULL;

#ifndef COMPILE_SINGLE_SLOT_PACKETS_ONLY
    ce_ptr->last_max_slot_req_sent = LMP_LAST_MAX_SLOT_REQ_SENT_INVALID;
    ce_ptr->last_recd_max_slot = LMP_MAX_SLOT_INVALID;
    ce_ptr->last_max_slot_sent = LMP_MAX_SLOT_INVALID;
#endif

    return;
}
#endif /* COMPILE_PARK_MODE */

/**
 * Handles the Connection Accept timeout (the host exceeds certain amount of
 * time to provide the response).
 *
 * \param timer_handle Timer handle.
 * \param conn_handle_param Connection handle for which some request was sent.
 *
 * \return None.
 */
void lmp_conn_accept_timeout_timer_handler(TimerHandle_t timer_handle)
{
    OS_SIGNAL signal_send;
    UINT16 ce_index;
    UINT16 conn_handle;
#ifdef ENABLE_SCO
    UINT16 sco_ce_index;
#endif

    LMP_CONNECTION_ENTITY *ce_ptr;

    conn_handle = (UINT16)((UINT32)pvTimerGetTimerID(timer_handle));

    RT_BT_LOG(GRAY, LMP_UTILS_309, 2,  timer_handle, conn_handle);

    if (LMP_GET_CE_INDEX_FROM_CONN_HANDLE(conn_handle, &ce_index)
            == API_SUCCESS)
    {
        ce_ptr = &lmp_connection_entity[ce_index];

        switch (ce_ptr->ce_status)
        {
            case LMP_WAITING_FOR_CONN_ACCEPT:
                lmp_set_ce_status(ce_index, LMP_BB_HL_CONNECTED);
                break;

            default:
                BZ_ASSERT(0, "Invalid state conn_accept_timeout received");
                return;
        }

        lmp_send_lmp_not_accepted(ce_index, LMP_HOST_CONNECTION_REQ_OPCODE,
                                  REMOTE_DEV_TID,
                                  CONNECTION_ACCEPT_TIMEOUT_EXCEEDED_ERROR);
        if (lmp_handle_acl_disconnect(ce_index,
                                      CONNECTION_ACCEPT_TIMEOUT_EXCEEDED_ERROR)
                                      != API_SUCCESS)
        {
            LMP_ERR(LMP_HANDLE_ACL_DISCONNECT_FAILED,0,0);
        }
    }
#ifdef ENABLE_SCO
    else if (lmp_get_sco_ce_index_from_conn_handle(conn_handle, &sco_ce_index)
             == API_SUCCESS)
    {
        ce_index = lmp_sco_connection_data[sco_ce_index].conn_entity_index;
        ce_ptr = &lmp_connection_entity[ce_index];
        lmp_send_lmp_not_accepted(ce_index, LMP_SCO_LINK_REQ_OPCODE,
                                  REMOTE_DEV_TID,
                                  CONNECTION_ACCEPT_TIMEOUT_EXCEEDED_ERROR);
        lmp_gen_approp_sco_conn_completion_event(ce_index, sco_ce_index,
                CONNECTION_ACCEPT_TIMEOUT_EXCEEDED_ERROR);
        lmp_sco_connect_restore_ce_status(ce_index);
        lmp_free_sco_conn_entity(sco_ce_index);
    }
#endif
    else
    {
        BZ_ASSERT(0, "Invalid connection handle or timeout");
        return;
    }

    signal_send.type = LMP_DELETE_TIMER_HANDLE_SIGNAL;
    signal_send.param = (OS_ADDRESS) (ce_ptr->conn_accept_timer_handle);
    if (OS_SEND_SIGNAL_TO_TASK(lmp_task_handle, signal_send) != BT_ERROR_OK)
    {
        LMP_ERR(LOG_LEVEL_HIGH, OS_SEND_SIGNAL_TO_TASK_FAILED,0,0);
    }

    ce_ptr->conn_accept_timer_handle = NULL;

    return;
}

/**
 * Extracts the BD_ADDR from the given FHS packet.
 *
 * \param fhs_pkt Pointer to the FHS packet.
 * \param bd_addr The extracted BD_ADDR.
 *
 * \return None.
 */
void lmp_extract_bd_addr_from_fhs_packet(UCHAR *fhs_pkt, UCHAR *bd_addr)
{
    UINT32 fhs_lap;

    fhs_lap = (fhs_pkt[4] >> 2) |
              (fhs_pkt[5] << 6) |
              (fhs_pkt[6] << 14) |
              (fhs_pkt[7] << 22);

    bd_addr[0] = (UINT8)fhs_lap;
    bd_addr[1] = (UINT8)(fhs_lap >> 8);
    bd_addr[2] = (UINT8)(fhs_lap >> 16);
    bd_addr[3] = fhs_pkt[8];
    bd_addr[4] = fhs_pkt[9];
    bd_addr[5] = fhs_pkt[10];
}


/**
 * Extracts the FHS packet contents into the connection entity.
 *
 * \param lmp_fhs_pkt_recd Pointer to the FHS packet.
 * \param ce_index ACL connection entity index.
 *
 * \return None.
 */
void lmp_extract_fhs_packet_to_ce(
    LMP_FHS_PKT_RECD *lmp_fhs_pkt_recd, UINT16 ce_index)
{
    UCHAR temp_byte;
    UINT32 slave_clock ;
    UINT16 clock_offset ;
    UINT32 master_clock ;
    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    /* Extract the BD_ADDR and Copy the BD address to inquiry table */
    lmp_extract_bd_addr_from_fhs_packet(lmp_fhs_pkt_recd->fhs_pkt,
                                        ce_ptr->bd_addr);

    /* Class of Device */
    ce_ptr->class_of_device = (lmp_fhs_pkt_recd->fhs_pkt[13] << 16) |
                              (lmp_fhs_pkt_recd->fhs_pkt[12] << 8) |
                               lmp_fhs_pkt_recd->fhs_pkt[11];

    /* Extracting the Page Scan Repetition Mode */
    temp_byte = lmp_fhs_pkt_recd->fhs_pkt[7];
    ce_ptr->page_scan_repetition_mode = (UCHAR)((temp_byte & 0x30) >> 4);

    /* Page Scan Period Mode (deprecated) */
    ce_ptr->page_scan_period_mode = (UCHAR)(temp_byte >> 6);

    /* Page Scan Mode */
    temp_byte = lmp_fhs_pkt_recd->fhs_pkt[17];
    ce_ptr->page_scan_mode = (UCHAR)(temp_byte >> 5);

    /* Clock offset */
    slave_clock = (lmp_fhs_pkt_recd->fhs_pkt[17]) & 0x1F ;
    slave_clock <<= 8 ;
    slave_clock |= (lmp_fhs_pkt_recd->fhs_pkt[16]);
    slave_clock <<= 8 ;
    slave_clock |= (lmp_fhs_pkt_recd->fhs_pkt[15]);
    slave_clock <<= 5 ;
    slave_clock |= (lmp_fhs_pkt_recd->fhs_pkt[14] >> 3);
    slave_clock = (slave_clock & 0x7FFF);
    master_clock = lmp_fhs_pkt_recd->clk;
    master_clock = ((master_clock >> 2) & 0x7FFF);

    /* Calculating the clock offset */
    clock_offset = (UINT16)((slave_clock - master_clock) & 0x07fff);
    ce_ptr->clock_offset = clock_offset;

    return;
}

#ifdef COMPILE_PARK_MODE
/**
 * Generates the beacon parameters and stores in the connection entity.
 *
 * \param ce_index ACL connection entity index.
 *
 * \return None.
 */
void lmp_generate_beacon_parameters(UINT16 ce_index)
{
    UINT32 clock_val;
    UCHAR timing_control_flag = 0x00;
    UINT16  use_Delta_b,use_Nb,use_M_acc,use_T_acc,use_N_acc_slots,use_D_acc,
         use_N_poll,use_Nb_sleep,use_Db_sleep;
    UINT16 min_cal_tb,default_cal_tb,new_Tb;
    UINT16 additional_slots,slots_to_expand;
    UINT16 park_ce_index = 0;

    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    use_Delta_b = MIN_DELTA_b;
    use_Nb = MIN_Nb;
    use_M_acc = MIN_M_acc;
    use_T_acc = MIN_T_acc;
    use_N_acc_slots = MIN_N_acc_slots;
    use_D_acc = MIN_D_acc;
    use_N_poll = MIN_N_poll;
    use_Nb_sleep = MIN_Nb_sleep;
    use_Db_sleep = MIN_Db_sleep;

    /*
     * For LMP_set_broadcast_scan_window, LMP_modify_beacon,
     * LMP_unpark_BD_ADDR_req and LMP_unpark_PM_ADDR_req the parameter
     * DB is optional. This parameter is only present if bit0 of timing
     * control flags is 1.
     * If the parameter is not included, the position in payload for all
     * parameters following
     * DB are decreased by 2.
     */

    lc_get_clock_in_scatternet(&clock_val, ce_ptr->phy_piconet_id);

    timing_control_flag = 0x00;
    if(clock_val & (BT_CLOCK_27_BITS+1))
    {
        /* If MSB of master clock is 1 use INitialization 2 */
        timing_control_flag |= 0x01; /* Validity bit */
        timing_control_flag |= 0x02;
    }

    ce_ptr->timing_control_flag = timing_control_flag;

    min_cal_tb = MIN_D_acc + (MIN_M_acc * MIN_T_acc) + MIN_N_poll;

    default_cal_tb = DEFAULT_D_acc + (DEFAULT_M_acc * DEFAULT_T_acc) +
                     DEFAULT_N_poll;

    /* If this is the first park request */
    if(lmp_self_device_data.number_of_parked_dev == 0)
    {
        /* Check if our default park parameters is enough to park this device */
        if((default_cal_tb >= ce_ptr->beacon_min_interval) &&
                (default_cal_tb <= ce_ptr->beacon_max_interval))
        {
            use_Delta_b = DEFAULT_DELTA_b;
            use_Nb = DEFAULT_Nb;
            use_M_acc = DEFAULT_M_acc;
            use_T_acc = DEFAULT_T_acc;
            use_N_acc_slots = DEFAULT_N_acc_slots;
            use_D_acc = DEFAULT_D_acc;
            use_N_poll = DEFAULT_N_poll;
            use_Nb_sleep = DEFAULT_Nb_sleep;
            use_Db_sleep = DEFAULT_Db_sleep;
        }
        else
        {
            /* Check if our min values is less than the host max allowed */
            if(min_cal_tb <= ce_ptr->beacon_max_interval)
            {
                LMP_INF(USING_MINIMUM_VALUES_OF_PARK_PARAMS,0,0);

                use_Delta_b = MIN_DELTA_b;
                use_Nb = MIN_Nb;
                use_M_acc = MIN_M_acc;
                use_T_acc = MIN_T_acc;
                use_N_acc_slots = MIN_N_acc_slots;
                use_D_acc = MIN_D_acc;
                use_N_poll = MIN_N_poll;
                use_Nb_sleep = MIN_Nb_sleep;
                use_Db_sleep = MIN_Db_sleep;

                /* Expand our min beacon params */
                additional_slots = (UINT16)
                                   (ce_ptr->beacon_max_interval - min_cal_tb);
                if(additional_slots > 0)
                {
                    LMP_INF(SLOTS_AVALIABLE_TO_EXPAND,1,additional_slots);

                    /* Break up the avaliable slots to expand Beacon Channel,
                     * Access Window and Dacc */
                    slots_to_expand = (UINT16)(additional_slots / 3);

                    /* We will first expand the Delta_b and Nb*/
                    if(slots_to_expand / (2*use_Nb) >= 1)
                    {
                        /* Expand Delta_b */
                        while(use_Delta_b < DEFAULT_DELTA_b)
                        {
                            use_Delta_b +=2;
                            slots_to_expand = (UINT16)(slots_to_expand - (2*use_Nb));
                            if(slots_to_expand < (2*use_Nb))
                            {
                                break;
                            }
                        }

                        /* Lets try to expand Nb */
                        if(slots_to_expand > use_Delta_b)
                        {
                            while(use_Nb < DEFAULT_Nb)
                            {
                                use_Nb ++;
                                slots_to_expand = (UINT16)(slots_to_expand - use_Delta_b);
                                if(slots_to_expand < use_Delta_b)
                                {
                                    break;
                                }
                            }
                        }
                    }

                    /* Try to expand M_acc and T_acc */
                    slots_to_expand = (UINT16)(additional_slots / 3);
                    if(slots_to_expand / (2*use_M_acc) >= 1)
                    {
                        while(use_T_acc < DEFAULT_T_acc)
                        {
                            use_T_acc += 2;
                            slots_to_expand = (UINT16)(slots_to_expand - (2*use_M_acc));
                            if(slots_to_expand < (2*use_M_acc))
                            {
                                break;
                            }
                        }

                        /* Adjust N_acc_slots also */
                        use_N_acc_slots = use_T_acc;

                        /* Lets try to expand Nb */
                        if(slots_to_expand > use_T_acc)
                        {
                            while(use_M_acc < DEFAULT_M_acc)
                            {
                                use_M_acc ++;
                                slots_to_expand = (UINT16)(slots_to_expand - use_T_acc);
                                if(slots_to_expand < use_T_acc)
                                {
                                    break;
                                }
                            }
                        }
                    }

                    /* Now increase D_acc */
                    new_Tb = (UINT16)((use_Nb * use_Delta_b) + (use_M_acc * use_T_acc) +
                                      use_N_poll);
                    if(ce_ptr->beacon_max_interval - new_Tb > DEFAULT_GAP)
                    {
                        LMP_INF(INCREASING_D_ACC,0,0);
                        use_D_acc = (UINT16)((use_Nb * use_Delta_b) + DEFAULT_GAP);
                    }
                    else
                    {
                        use_D_acc = (UINT16)((use_Nb * use_Delta_b) +
                                             (use_M_acc * use_T_acc) + MIN_GAP);
                    }
                }
                else
                {
                    LMP_INF(NO_SLOTS_AVALIABLE_TO_EXPAND_TRAIN,0,0);
                }
            }
            else
            {
                LMP_INF(OUR_MIN_VALUE_DOESNOT_FALL_IN_THE_HOSTRANGE,0,0);
            }
        }
    }
    else
    {
        /* We already have slaves parked.
         * Use the same existing Train to park this slave also
         */
        for(park_ce_index=0; park_ce_index < LMP_MAX_CE_DATABASE_ENTRIES;
                park_ce_index ++)
        {
            if(lmp_connection_entity[park_ce_index].ce_status == LMP_PARK_MODE)
            {
                break;
            }
        }

        use_Delta_b = lmp_connection_entity[park_ce_index].Delta_beacon;
        use_Nb = lmp_connection_entity[park_ce_index].Nbeacon;
        use_M_acc = lmp_connection_entity[park_ce_index].M_access;
        use_T_acc = lmp_connection_entity[park_ce_index].T_access;
        use_N_acc_slots = lmp_connection_entity[park_ce_index].N_acc_slots;
        use_D_acc = lmp_connection_entity[park_ce_index].D_access;
        use_N_poll = lmp_connection_entity[park_ce_index].N_poll;
        use_Nb_sleep = lmp_connection_entity[park_ce_index].NB_sleep;
        use_Db_sleep = lmp_connection_entity[park_ce_index].DB_sleep;
    }

    /* Update the connection entity to use the park params */
    ce_ptr->Nbeacon = (UCHAR)use_Nb;
    ce_ptr->Delta_beacon = (UCHAR)use_Delta_b;
    ce_ptr->NB_sleep = (UCHAR)use_Nb_sleep;
    ce_ptr->DB_sleep = (UCHAR)use_Db_sleep;
    ce_ptr->D_access = (UCHAR)use_D_acc;
    ce_ptr->T_access = (UCHAR)use_T_acc;
    ce_ptr->M_access = (UCHAR)use_M_acc;
    ce_ptr->N_poll = (UCHAR)use_N_poll;
    ce_ptr->N_acc_slots = (UCHAR)use_N_acc_slots;
    ce_ptr->access_scheme = 0;
    ce_ptr->Tbeacon = ce_ptr->beacon_max_interval;

    lmp_current_nb = (UCHAR)(use_Nb + use_N_poll);

    return;
}

/**
 * Update the beacon parameters received from the remote device and stores it
 * in the connection entity.
 *
 * \param lmp_pdu_ptr Pointer to the LMP PDU packet.
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_update_beacon_parameters(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    ce_ptr->Dbeacon = lmp_pdu_ptr->payload_content[2] |
                     (lmp_pdu_ptr->payload_content[3] << 8);

    ce_ptr->Tbeacon = lmp_pdu_ptr->payload_content[4] |
                     (lmp_pdu_ptr->payload_content[5] << 8);

    ce_ptr->Nbeacon = lmp_pdu_ptr->payload_content[6] ;

    ce_ptr->Delta_beacon = lmp_pdu_ptr->payload_content[7] ;
    ce_ptr->pm_addr = lmp_pdu_ptr->payload_content[8] ;
    ce_ptr->ar_addr = lmp_pdu_ptr->payload_content[9] ;
    ce_ptr->NB_sleep = lmp_pdu_ptr->payload_content[10] ;
    ce_ptr->DB_sleep = lmp_pdu_ptr->payload_content[11] ;
    ce_ptr->D_access = lmp_pdu_ptr->payload_content[12] ;
    ce_ptr->T_access = lmp_pdu_ptr->payload_content[13] ;
    ce_ptr->N_acc_slots = lmp_pdu_ptr->payload_content[14] ;
    ce_ptr->N_poll = lmp_pdu_ptr->payload_content[15] ;
    ce_ptr->M_access = (UCHAR)(lmp_pdu_ptr->payload_content[16]  & 0x0f);
    ce_ptr->access_scheme =
        (UCHAR)((lmp_pdu_ptr->payload_content[16] >> 4 ) & 0x0f);

    lmp_current_nb = (UCHAR)(ce_ptr->Nbeacon + ce_ptr->N_poll);

    return BT_FW_SUCCESS;
}

/**
 * Validates the beacon parameters received from the remote device.
 *
 * \param lmp_pdu_ptr Pointer to the LMP PDU packet.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_validate_beacon_parameters(LMP_PDU_PKT *lmp_pdu_ptr)
{
    UCHAR timing_ctrl_flgs,pm_addr,ar_addr,NB_sleep,DB_sleep;
    UINT16 Dbeacon, Tbeacon;
    UCHAR Nbeacon, Delta_beacon, D_access, T_access, N_acc_slots, N_poll;
    UCHAR M_access, access_scheme;

    timing_ctrl_flgs = lmp_pdu_ptr->payload_content[1] ;

    Dbeacon = lmp_pdu_ptr->payload_content[2] |
             (lmp_pdu_ptr->payload_content[3] << 8);

    Tbeacon = lmp_pdu_ptr->payload_content[4] |
             (lmp_pdu_ptr->payload_content[5] << 8);
    Nbeacon = lmp_pdu_ptr->payload_content[6] ;

    Delta_beacon = lmp_pdu_ptr->payload_content[7] ;
    pm_addr = lmp_pdu_ptr->payload_content[8] ;
    ar_addr = lmp_pdu_ptr->payload_content[9] ;
    NB_sleep = lmp_pdu_ptr->payload_content[10] ;
    DB_sleep = lmp_pdu_ptr->payload_content[11] ;

    D_access = lmp_pdu_ptr->payload_content[12] ;
    T_access = lmp_pdu_ptr->payload_content[13] ;
    N_acc_slots = lmp_pdu_ptr->payload_content[14] ;
    N_poll = lmp_pdu_ptr->payload_content[15] ;
    M_access = (UCHAR)(lmp_pdu_ptr->payload_content[16]  & 0x0f);
    access_scheme = (UCHAR)((lmp_pdu_ptr->payload_content[16] >> 4 ) & 0x0f);

    /* Validate the Params sent by the remote device */
    if(((Dbeacon & 0x01) != 0) || ((Tbeacon & 0x01) != 0) ||
            ((Delta_beacon & 0x01) != 0) || ((D_access & 0x01) != 0) ||
            ((T_access & 0x01) != 0))
    {
        return INVALID_LMP_PARAMETERS_ERROR;
    }

    /* Limited range of Beacon parameters are supported. */
    if ( (Tbeacon > otp_str_data.bt_beacon_max_interval) || (Tbeacon < otp_str_data.bt_beacon_min_interval) )
    {
        return UNSUPPORTED_PARAMETER_VALUE_ERROR;
    }

    if(D_access <= ( (Nbeacon - 1) * Delta_beacon))
    {
        return INVALID_LMP_PARAMETERS_ERROR;
    }
    if(Tbeacon < (D_access + (M_access * T_access) + N_poll))
    {
        return INVALID_LMP_PARAMETERS_ERROR;
    }
    if(access_scheme != 0)
    {
        LMP_ERR(INVALID_ACCESS_SCHEME,0,0);
        return INVALID_LMP_PARAMETERS_ERROR;
    }

    if(N_acc_slots > T_access)
    {
        return INVALID_LMP_PARAMETERS_ERROR;
    }

    if (M_access == 0)
    {
        return INVALID_LMP_PARAMETERS_ERROR;
    }

    return BT_FW_SUCCESS;
}
#endif /* COMPILE_PARK_MODE */

/**
 * Convert air mode from one layer representation to another layer.
 *
 * \param air_mode Air mode to converted.
 * \param from The input layer (HCI_LAYER / LMP_LAYER).
 * \param to The output layer (HCI_LAYER / LMP_LAYER).
 *
 * \return The converted air mode, if the operation is successful.
 *         BT_FW_ERROR, otherwise.
 */
UCHAR lmp_convert_air_mode(UCHAR air_mode, UCHAR from, UCHAR to)
{
    UCHAR ret_val;

    if((from == HCI_LAYER) && (to == LMP_LAYER))
    {
        switch(air_mode)
        {
            case AIR_CODING_MU_LAW:
                ret_val = LMP_U_LAW_LOG;
                break;

            case AIR_CODING_A_LAW:
                ret_val = LMP_A_LAW_LOG;
                break;

            case AIR_CODING_CVSD:
                ret_val = LMP_CVSD;
                break;

            default:
                ret_val = LMP_TRANSPARENT_DATA;
                break;
        }
    }
    else if((from == LMP_LAYER) && (to == HCI_LAYER))
    {
        switch(air_mode)
        {
            case LMP_U_LAW_LOG:
                ret_val = AIR_CODING_MU_LAW;
                break;

            case LMP_A_LAW_LOG:
                ret_val = AIR_CODING_A_LAW;
                break;

            case LMP_CVSD:
                ret_val = AIR_CODING_CVSD;
                break;

            default:
                ret_val = LMP_TRANSPARENT_DATA;
                break;
        }
    }
    else
    {
        ret_val = BT_FW_ERROR;
    }

    return ret_val;
}


#ifdef POWER_CONTROL
/**
 * Power control timer handler.
 *
 * \param timer_handle Timer handle.
 * \param index ACL connection entity index.
 *
 * \return None.
 */
void lmp_power_control_handler(TimerHandle_t timer_handle)
{
    UINT16 ce_index;

#ifdef COMPILE_RSSI_REPORTING
    for(ce_index = 0; ce_index < LMP_MAX_CE_DATABASE_ENTRIES; ce_index++)
    {
        LMP_CONNECTION_ENTITY *ce_ptr;

        ce_ptr = &lmp_connection_entity[ce_index];

        if (ce_ptr->ce_status != LMP_CONNECTED)
        {
            /* Disable power control in low power modes */
            continue;
        }

#ifdef TEST_MODE
        /* Do not send if DUT mode has started. */
        if (ce_ptr->test_mode_info.test_state == TEST_ACTIVATED)
        {
            //LMP_LOG_INFO(LOG_LEVEL_HIGH, "Not sending ch-cl-pdu in DUT mode.");

            return;
        }
#endif
        /*(150818) lc_invoke_scheduler is marked by dape...
          copied from 8723B patch which is requested by Austin.*/
        //lc_invoke_scheduler(ce_ptr->phy_piconet_id); 

        lmp_send_power_ctrl_pdu(ce_index, SELF_DEV_TID);
    }
#endif /* COMPILE_RSSI_REPORTING */

    return;
}

/**
 * Sends power control pdu based on the input RSSI value.
 *
 * \param ce_index ACL connection entity index.
 * \param transaction_id Transaction ID.
 * \param rssi Input RSSI value.
 *
 * \return None.
 */
void lmp_send_power_ctrl_pdu(UINT16 ce_index, UCHAR transaction_id)
{
    UCHAR parameter_list[LMP_INCR_POWER_REQ_LEN];
    LMP_CONNECTION_ENTITY *ce_ptr;
    UINT16 rssi = 0;
    INT16 rssi_dbm;

    ce_ptr = &lmp_connection_entity[ce_index];

    if(!(ce_ptr->feat_page0[2] & LMP_POWER_CONTROL_FEATURE)
      || !(lmp_feature_data.feat_page0[2] & LMP_POWER_CONTROL_FEATURE))
    {
        return;
    }

#ifdef COMPILE_PARK_MODE
    if(ce_ptr->ce_status == LMP_PARK_MODE)
    {
        return;
    }
#endif

#ifdef COMPILE_ROLE_SWITCH
    if(lmp_mss_state != LMP_MSS_INIT)
    {
        /* MSS in progress. Do not queue PDUs on this connection. */
        if(lmp_role_switch_data.ce_index == ce_index)
        {
            RT_BT_LOG(GRAY, LMP_UTILS_992, 1, ce_index);
            return;
        }
    }
#endif
#ifdef _NO_SEND_AFH_POWER_CTRL_WHILE_ROLE_SW
#ifdef _ENABLE_COMPILE_NESTED_PAUSE_RESUME_
    UCHAR lut_index;
    lut_index = lc_get_lut_index_from_phy_piconet_id(ce_ptr->am_addr, ce_ptr->phy_piconet_id);
    if(paused_lut_index[lut_index])
    {
         RT_BT_LOG(RED, DAPE_TEST_LOG586, 3, ce_index, LMP_INCR_POWER_REQ_OPCODE, 0);
         return;
    }
#else
    if(ce_ptr->pause_data_transfer)
    {
         RT_BT_LOG(RED, DAPE_TEST_LOG586, 3, ce_index, LMP_INCR_POWER_REQ_OPCODE, 0);
         return;
    }
#endif
#endif
#ifdef TEST_MODE
    if(ce_ptr->test_mode_info.test_state == TEST_STARTED)
    {
        return;
    }
#endif

#ifdef COMPILE_RSSI_REPORTING
    rssi = (UINT16) ce_ptr->rssi;

    if (rssi == 0)
    {
        return;
    }

    rssi_dbm = (rssi << 1) - 90;
    if(rssi_dbm < -90)
    {
        rssi_dbm = -90;
    }

    if(rssi_dbm > 0)
    {
        rssi_dbm = 0;
    }

    LMP_LOG_INFO(LOG_LEVEL_HIGH, LMP_MSG_POWER_CTRL, 8,
                 rssi, rssi_dbm, otp_str_data.bw_rf_max_rssi_dbm,
                 otp_str_data.bw_rf_min_rssi_dbm,
                 ce_ptr->power_ctrl_pdu_sent_flag, ce_ptr->power_ctrl_resp,
                 ce_ptr->rssi_samples_count , ce_ptr->rssi_value_accumulate );
#endif

#ifdef ENABLE_LOGGER_LEVEL_2
    LMP_LOG_INFO(LOG_LEVEL_LOW, NO_OF_RSSI_SAMPLES, 1, ce_ptr->rssi_samples_count);
    LMP_LOG_INFO(LOG_LEVEL_LOW, RSSI_DBM_VAL, 2, ce_index, rssi_dbm);
#endif

    /* Make sure that there are at least LMP_MIN_RSSI_SAMPLES samples. */
    if (ce_ptr->rssi_samples_count < LMP_MIN_RSSI_SAMPLES)
    {
        return;
    }
    else
    {
        ce_ptr->rssi_samples_count = 0;
        ce_ptr->rssi_value_accumulate = 0;
    }

    if(rssi_dbm < otp_str_data.bw_rf_min_rssi_dbm &&
            ce_ptr->power_ctrl_resp != LMP_MAX_POWER_RCVD &&
            ce_ptr->power_ctrl_pdu_sent_flag == FALSE )
    {
        parameter_list[0] = LMP_INCR_POWER_REQ_OPCODE;
        parameter_list[2] = POWER_CONTROL_RESERVED_BYTE_2;
        if (ce_ptr->power_ctrl_resp == LMP_MIN_POWER_RCVD)
        {
            ce_ptr->power_ctrl_resp = LMP_MID_POWER;
        }

#ifdef VER_3_0
        if (IS_BT30)
        {
            UCHAR incr_pwr_step;

            incr_pwr_step = LMP_PWR_CTRL_INC_ONE_STEP;
            if ((otp_str_data.bw_rf_min_rssi_dbm - rssi_dbm) >=
                                                LMP_MAX_POWER_CHANGE)
            {
                incr_pwr_step = LMP_PWR_CTRL_INC_UPTO_MAXIMUM;
            }

            if (lmp_generate_power_ctrl_req_pdu(ce_index, incr_pwr_step)
                    == API_SUCCESS)
            {
                return;
            }
        }
#endif

        lmp_generate_pdu(ce_index, parameter_list, LMP_INCR_POWER_REQ_LEN,
                         (LMP_TRAN_ID)transaction_id, LMP_NO_STATE_CHANGE);

        ce_ptr->power_ctrl_pdu_sent_flag = TRUE;

        //LMP_INF(LMP_MSG_TX_INC_POWER, 0, 0);

        return;
    }

    if(rssi_dbm > otp_str_data.bw_rf_max_rssi_dbm &&
            ce_ptr->power_ctrl_resp != LMP_MIN_POWER_RCVD &&
            ce_ptr->power_ctrl_pdu_sent_flag == FALSE )
    {
        parameter_list[0] = LMP_DECR_POWER_REQ_OPCODE ;
        parameter_list[2] = POWER_CONTROL_RESERVED_BYTE_2;
        if (ce_ptr->power_ctrl_resp == LMP_MAX_POWER_RCVD)
        {
            ce_ptr->power_ctrl_resp = LMP_MID_POWER;
        }

#ifdef VER_3_0
        if (IS_BT30)
        {
            if (lmp_generate_power_ctrl_req_pdu(ce_index,
                                    LMP_PWR_CTRL_DEC_ONE_STEP) == API_SUCCESS)
            {
                return;
            }
        }
#endif

        lmp_generate_pdu(ce_index, parameter_list, LMP_DECR_POWER_REQ_LEN,
                         (LMP_TRAN_ID)transaction_id, LMP_NO_STATE_CHANGE);
        ce_ptr->power_ctrl_pdu_sent_flag = TRUE;

        return;
    }

    return;

}

#endif


/**
 * Decides whether to send Connection_Complete_Event or not. It also sends the
 * event, if necessary.
 *
 * \param ce_index Connection entity index of the ACL link that is being
 *                 established.
 *
 * \return None.
 */
void lmp_decide_to_send_conn_complete_evt(UINT16 ce_index)
{
    UCHAR parameter_list[LMP_PDU_MAX_LEN];
    OS_SIGNAL signal_send;
    UCHAR lcl_setup_comp_sts_changed = FALSE;
    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

#ifdef _CCH_SC_ECDH_P256_TEST_PKT
RT_BT_LOG(YELLOW, CCH_DBG_033, 1,ce_ptr->connection_type.packet_type);
#endif

#ifdef _DAPE_TEST_SEND_PTT_EARLIER
    if (g_efuse_lps_setting_3.iot_ralink_send_fea_req_later)
    {
    UCHAR temp;
    UCHAR status = TRUE;

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
	//RT_BT_LOG(WHITE, DAPE_TEST_LOG522, 2,status, ce_ptr->ptt_status);
    }
#endif

    if((ce_ptr->setup_complete_status == BEGIN_SETUP) ||
            (ce_ptr->setup_complete_status == RECD_SETUP_COMPLETE) )
    {
        LMP_INF(SENDING_SETUP_COMPLETE,0,0);
        parameter_list[0] = LMP_SETUP_COMPLETE_OPCODE;

        lmp_generate_pdu(ce_index,parameter_list, LMP_SETUP_COMPLETE_LEN,
                         SELF_DEV_TID, LMP_NO_STATE_CHANGE);
        if (ce_ptr->setup_complete_status != RECD_SETUP_COMPLETE)
        {
            ce_ptr->lmp_expected_pdu_opcode |=
                lmp_get_opcode_mask(LMP_SETUP_COMPLETE_OPCODE, 0x0);
        }
    }

    if(ce_ptr->setup_complete_status == RCVD_SENT_SETUP_COMPLETE)
    {
        ce_ptr->setup_complete_status = CONN_COMPLETE_EVENT;
        lcl_setup_comp_sts_changed = TRUE;
    }

    switch(ce_ptr->setup_complete_status)
    {
        case BEGIN_SETUP :
            ce_ptr->setup_complete_status = SENT_SETUP_COMPLETE;
            break ;

        case RECD_SETUP_COMPLETE :
            ce_ptr->setup_complete_status = CONN_COMPLETE_EVENT ;
            lcl_setup_comp_sts_changed = TRUE;
            break ;
    }

    /* Send connection complete event */
    if ( (ce_ptr->setup_complete_status == CONN_COMPLETE_EVENT) &&
            (lcl_setup_comp_sts_changed == TRUE) )
    {
        LMP_INF(INSIDE_LMP_AU_RAND_GENERATING_CONN_COMP_EVENT,0,0);
        hci_generate_connection_complete_event(HCI_COMMAND_SUCCEEDED,
                                               ce_ptr->connection_type.connection_handle,
                                               ce_ptr->bd_addr,
                                               ce_ptr->connection_type.link_type,
                                               (UCHAR) bz_auth_get_encryption_mode(ce_index));

        /* Check for sending timing accracy PDU */
        if((lmp_feature_data.feat_page0[0] & LMP_TIME_ACCURACY_FEATURE) &&
                (ce_ptr->feat_page0[0] & LMP_TIME_ACCURACY_FEATURE))
        {
            /* Send timing accuracy response PDU */
            parameter_list[0] = LMP_TIMING_ACCURACY_REQ_OPCODE;
            lmp_generate_pdu(ce_index, parameter_list,
                             LMP_TIMING_ACCURACY_REQ_LEN, SELF_DEV_TID,
                             LMP_NO_STATE_CHANGE);
        }

        lmp_set_ce_status(ce_index, LMP_CONNECTED);

        LMP_DECIDE_FOR_LMP_AFH_PDU(ce_index);

#ifdef _CCH_ADD_QOS_
        if(ce_ptr->remote_dev_role == SLAVE )
        {
        		/* Generate LMP_qos_req pdu. */
        	parameter_list[0] = LMP_QoS_REQ_OPCODE;
            parameter_list[2] = LSB(ce_ptr->qos_tpoll);
            parameter_list[3] = MSB(ce_ptr->qos_tpoll);
        	parameter_list[4] = ce_ptr->num_of_BC;

        	lmp_generate_pdu(ce_index, parameter_list, LMP_QoS_REQ_LEN, SELF_DEV_TID,
                    LMP_NO_STATE_CHANGE);
        }
#endif

#ifdef COMPILE_CHANNEL_ASSESSMENT
        lmp_init_assessment_at_connection();
#endif

        /* Post LLC complete signal to LMP task. */
        signal_send.type = LMP_LLC_CON_SETUP_COMPLETE;
        signal_send.param = (OS_ADDRESS)((UINT32)ce_index);
        OS_ISR_SEND_SIGNAL_TO_TASK(lmp_task_handle, signal_send);
    }

    if (ce_ptr->setup_complete_status == CONN_COMPLETE_EVENT)
    {
        lmp_set_ce_status(ce_index, LMP_CONNECTED);
    }

    return;
}


#ifdef COMPILE_PARK_MODE
/**
 * Selects the connection to be unparked.
 *
 * \param None.
 *
 * \return None.
 */
void lmp_select_unpark_device(void)
{
    UINT16 ce_index ;

    for (ce_index = 0; ce_index < LMP_MAX_CE_DATABASE_ENTRIES ; ce_index++)
    {
        if ((lmp_connection_entity[ce_index].entity_status == ASSIGNED) &&
            ((lmp_connection_entity[ce_index].unpark_req_flag == LMP_UNPARK_HOST_INITIATED) ||
             (lmp_connection_entity[ce_index].unpark_req_flag == LMP_UNPARK_AUTO_CONTROLLER_INITIATED)))
        {
            LMP_INF(SELECTED_CE_INDEX_FOR_UNPARKING_THE_DEVICE,1,ce_index);

            lmp_unpark_ce_index = ce_index ;
            break;
        }
    }
}
#endif /* COMPILE_PARK_MODE */

#ifdef COMPILE_SNIFF_MODE
/**
 * Validate the sniff mode request parameters received from the remote device.
 *
 * \param trans_id Transaction ID.
 * \param timing_ctl_flag Timing control flags.
 * \param ce_index ACL connection entity index.
 * \param sniff_slot_offset Dsniff.
 * \param Tsniff Sniff interval.
 * \param sniff_attempt Sniff attempt.
 * \param sniff_timeout Sniff timeout.
 *
 * \return HCI_COMMAND_SUCCEEDED, if the operation is successful. A valid
 *         bluetooth error code, otherwise.
 */
UCHAR lmp_validate_sniff_parms(UCHAR trans_id, UCHAR timing_ctl_flag,
                               UINT16 ce_index, UINT16 sniff_slot_offset, UINT16 Tsniff,
                               UINT16 sniff_attempt,UINT16 sniff_timeout)
{
    UINT16 super_to ;
    UINT16 new_sniff_interval ;
    UCHAR parameter_list[LMP_SNIFF_REQ_LEN];

    LMP_CONNECTION_ENTITY *ce_ptr;

#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8723_RCP_
// RCP for change the whole function or Just change the input paramter
// Change Parameter: all of the input paramter can be changed

    UCHAR return_status = 0;

    if (rcp_lmp_validate_sniff_parms != NULL)
    {
        if ( rcp_lmp_validate_sniff_parms((void *)&return_status,trans_id, timing_ctl_flag,
                                ce_index,  &sniff_slot_offset, &Tsniff,
                                &sniff_attempt, &sniff_timeout) )
        {
            return return_status;
        }
    }
#endif
#endif

    if( (Tsniff == 0) || (sniff_attempt == 0) )
    {
        return INVALID_LMP_PARAMETERS_ERROR;
    }


    if(Tsniff < ( (sniff_attempt + sniff_timeout) << 1) )
    {
        return INVALID_LMP_PARAMETERS_ERROR;
    }

#ifndef _IGNORE_SNIFF_PARAM_LIMIT
    /*
       We need atleast 4 attempts for every 0x1000 slots of Tsniff, and
       a ceiling of 20d.
     */
    if(Tsniff > 0x5000)
    {
        if(sniff_attempt < 20)
        {
            return UNSUPPORTED_PARAMETER_VALUE_ERROR;
        }
    }
    else if(sniff_attempt < ( ((Tsniff) >> 12) << 2) )
    {
        return UNSUPPORTED_PARAMETER_VALUE_ERROR;
    }
#endif

#ifdef ENABLE_SCO
    if (lc_full_bandwidth_flag == TRUE
            && lmp_self_device_data.sco_pkt_type != HV1)
    {
        return UNSUPPORTED_FEATURE_OR_PARAMETER_ERROR;
    }

#if 0
    /* Sniff request is accepted if there is SCO connection to the other device
     * and needs an sniff attempt of atleast 4.
     */
    if (sniff_attempt < (lmp_self_device_data.total_no_of_sco_conn * 4) )
    {
        return UNSUPPORTED_FEATURE_OR_PARAMETER_ERROR;
    }
#endif

#endif

    /* Check Tsniff is greater than supervision timeout*/
    /* The supervision timer is a software timer, since it is not very
     * accurate keep 50 slots
     * offset to be on the safer side. otherwise as soon as you enter
     * sniff mode supervision timer
     * may expire and connection gets disconnected. */


    ce_ptr = &lmp_connection_entity[ce_index];

    super_to = ce_ptr->link_supervision_timeout;
    LMP_INF(SUPERVISION_TIMEOUT_IN_SLOTS,1,super_to);


#ifdef _CCH_SLOT_OFFSET_
    UCHAR force_slot_overlap;

    UCHAR sniff_slot_offset_temp;
    UINT16 new_sniff_interval_temp;
    UCHAR temp_slot_num;
#ifdef _CCH_SNIFF_NEG_TIMEOUT_
    RT_BT_LOG(BLUE, CCH_DBG_015, 4,ce_ptr->sniff_neg_count, Tsniff, sniff_attempt,sniff_timeout);
#endif
    force_slot_overlap = 1;
    sniff_slot_offset_temp = sniff_slot_offset;
    new_sniff_interval_temp = Tsniff;
    {
#ifndef _DAPE_HID_NEG_ONLY_OFFSET
        if(Tsniff<GLOBAL_SLOT_MIN_ALLOW_SNIFF_INT)
        {
            new_sniff_interval_temp = Tsniff + GLOBAL_SLOT_MIN_ALLOW_SNIFF_INT;
		}
        new_sniff_interval_temp = new_sniff_interval_temp - (new_sniff_interval_temp%GLOBAL_SLOT_MIN_ALLOW_SNIFF_INT);
#endif
        temp_slot_num = 2;
        if(sniff_attempt > 1)
        {
            temp_slot_num = sniff_attempt << 1;
        }
        if(sniff_timeout > 0)
        {
            temp_slot_num += 2;
        }
        
#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8723_RCP_
/* add by josh , to patch PATCH_HID_NEG_ONLY_OFFSET and PATCH_HID_NEG_ATTEMPT */

        UCHAR return_status_1 = 0;

        if (rcp_lmp_validate_sniff_parms_1 != NULL)
        {
            if ( rcp_lmp_validate_sniff_parms_1((void *)&return_status_1, &new_sniff_interval_temp, trans_id, timing_ctl_flag,
                                ce_index,  &sniff_slot_offset, &Tsniff,
                                &sniff_attempt, &sniff_timeout) )
            {
                return return_status_1;
            }
        }
#endif
#endif
        
        if(Tsniff >= GLOBAL_SLOT_MIN_ALLOW_SNIFF_INT)
        {
            force_slot_overlap = lmp_force_global_slot_offset(ce_index, 0, Tsniff,
                        temp_slot_num, sniff_slot_offset);
        }
        else
        {
            force_slot_overlap = 1;
#ifdef _DAPE_ALLOW_SNIFF_WHEN_ONE_LINK
            if (lmp_self_device_data.number_of_hlc < 2)
            {
    	        force_slot_overlap = 0;
            }
#endif
        }

#ifdef _CCH_RTL8723A_B_CUT
// _CCH_8723_A_ECO_

    // cch
    // 20110714 (hw bug)
    // for IOT HID + HFP
    // HID use sniff interval = 20, timeout = 1, will overlap with SCO,
    // and the SNIFF send one redundant POLL at sniff timeout  window, this will cause HID retry

        if ((lmp_self_device_data.total_no_of_sco_conn != 0) ||
            (lmp_self_device_data.adding_new_sco_conn != 0) ||
            (lmp_self_device_data.number_of_esco_connections != 0) ||
            (lmp_self_device_data.adding_new_esco_conn == TRUE) )
        {
            if( ( (sniff_timeout) >0 )
                &&( Tsniff< (GLOBAL_SLOT_MIN_ALLOW_SNIFF_INT<<2))
                &&( (Tsniff%6)!=0) )
            {
                force_slot_overlap = 1;
            }
        }
#endif
        if(force_slot_overlap)
        {
            sniff_slot_offset_temp = lmp_get_global_slot_offset(ce_index, 0,
                        new_sniff_interval_temp, temp_slot_num, 0, 0);
            UCHAR lmp_get_global_slot_offset_fail;
            lmp_get_global_slot_offset_fail = 1;

            if ((sniff_slot_offset_temp) < GLOBAL_SLOT_INTERVAL)
            {
                lmp_get_global_slot_offset_fail = 0;
            }
            if (lmp_get_global_slot_offset_fail)
            {
                lmp_put_global_slot_offset(ce_index, 0);
                return UNSUPPORTED_PARAMETER_VALUE_ERROR;
            }
        }
        RT_BT_LOG(YELLOW, CCH_DBG_022, 1,sniff_slot_offset_temp);
    }
#endif

    if(((Tsniff) > (super_to - 50)) && (super_to != 0 ) )
    {
        if(ce_ptr->sniff_min_interval_negotiated == TRUE)
        {
            LMP_ERR(TSNIFF_IS_GREATER_THAN_SUPERVISION_TIMEOUT_AND_SNIFF_MIN_INTERVAL_NEGOTIATED,0,0);
            return INVALID_LMP_PARAMETERS_ERROR;
        }
        else
        {
            LMP_INF(TSNIFF_IS_GREATER_THAN_LINK_SUPERVISION,0,0);
            new_sniff_interval = (UINT16)(super_to - 50);
            ce_ptr->sniff_min_interval_negotiated = TRUE;
        }
    }
    else
    {
        /*
         * The sniff prarm negotiation is based on the sniff min interval
         * and max interval values
         * given by the host if self device initates the transaction.
        */
        if ((Tsniff < ce_ptr->sniff_min_interval) &&
                (trans_id != ce_ptr->remote_dev_role))
        {
            if(ce_ptr->sniff_min_interval_negotiated == TRUE)
            {
                LMP_ERR(TSNIFF_IS_LESS_THAN_SNIFF_MIN_INTERVAL_AND_SNIFF_MIN_INTERVAL_NEGOTIATED,0,0);
                return INVALID_LMP_PARAMETERS_ERROR ;
            }
            else
            {
                LMP_INF(TSNIFF_IS_LESS_THAN_MINIMUM_SNIFF_VALUE,1, ce_ptr->sniff_min_interval);
                new_sniff_interval = ce_ptr->sniff_min_interval ;
                ce_ptr->sniff_min_interval_negotiated = TRUE;
            }
        }
        else
        {
            /*
                The sniff parm negotiation is based on the sniff min interval and
                max interval values
                given by the host if self device initates the transaction.
            */
            if((Tsniff > ce_ptr->sniff_max_interval) &&
                    (trans_id != ce_ptr->remote_dev_role))
            {
                if(ce_ptr->sniff_max_interval_negotiated)
                {
                    LMP_ERR(TSNIFF_IS_GREATERE_THAN_SNIFF_MAX_INTERVAL_AND_SNIFF_MAX_INTERVAL_NEGOTIATED,0,0);
                    return INVALID_LMP_PARAMETERS_ERROR ;
                }
                else
                {
                    LMP_INF(TSNIFF_IS_GREATER_THAN_MAXIMUM_SNIFF_VALUE, 1, ce_ptr->sniff_max_interval);

                    new_sniff_interval = ce_ptr->sniff_max_interval ;
                    ce_ptr->sniff_max_interval_negotiated = TRUE;
                }
            }
            else
            {
#ifdef _CCH_SLOT_OFFSET_
#ifdef _CCH_SNIFF_OVERLAP_REJ_
                if(!force_slot_overlap)
#endif
                {
#endif
                    LMP_INF(SENDING_ACCEPTED_FOR_SNIFF_REQ_TO_THE_REMOTE_DEV,0,0);
                    lmp_send_lmp_accepted(ce_index, LMP_SNIFF_REQ_OPCODE,
                                          trans_id, LMP_SNIFF_MODE_NEG_TERMINATION);
#ifndef _CCH_SLOT_OFFSET_
                    /* For self dev initiated tn, release the Dsniff. */
                    if ((ce_ptr->remote_dev_role == SLAVE) &&
                            (trans_id != ce_ptr->remote_dev_role))
                    {
                        lmp_put_dsniff(ce_ptr->sniff_slot_offset);
                    }
#endif
                    /* Store all the sniff parameters in conn entity*/
                    ce_ptr->sniff_interval = Tsniff ;
                    ce_ptr->sniff_attempt = sniff_attempt;
                    ce_ptr->sniff_timeout = sniff_timeout ;
                    ce_ptr->sniff_slot_offset = sniff_slot_offset;
                    ce_ptr->sniff_max_interval_negotiated = FALSE ;
                    ce_ptr->sniff_min_interval_negotiated = FALSE ;
#if 0
                    /* As slave, we need to enter sniff mode, on queueing lmp_accepted. */
                    if (ce_ptr->remote_dev_role == MASTER)
                    {
                        /* Program LMP and LC to enter sniff mode. */
                        lmp_start_sniff_mode(ce_index);

                        /* Send Mode change event to the host */
                        hci_generate_mode_change_event(
                            HCI_COMMAND_SUCCEEDED,
                            ce_index,
                            LP_SNIFF_MODE,
                            ce_ptr->sniff_interval);
                    }
#endif
                    return HCI_COMMAND_SUCCEEDED ;
#ifdef _CCH_SLOT_OFFSET_
                }
#endif
            }
        }
    }

    /*
     * To enter sniff mode, The device can negotiate Tsniff and Dsniff.
     * Select new Dsniff as zero. We are always selecting Dsniff zero for
     * negotiation because the received Dsniff may be greater than Tsniff,
     * Tsniff may be link supervision timeout.
     * due to this the connection gets lost.
     */
    parameter_list[0] = LMP_SNIFF_REQ_OPCODE;
    parameter_list[2] = timing_ctl_flag;

    /*
        Dsniff... we have 2 options here,
        1. Always make Dsniff =0 or
        2. Send whatever we receive.
    */
    sniff_slot_offset = 0;

#ifdef _CCH_SLOT_OFFSET_
    sniff_slot_offset = sniff_slot_offset_temp;
    new_sniff_interval = new_sniff_interval_temp;
    //sniff_timeout = 0;
#endif

    RT_BT_LOG(BLUE, CCH_DBG_045, 4,sniff_slot_offset, new_sniff_interval, sniff_attempt, sniff_timeout);

    parameter_list[3] = LSB(sniff_slot_offset);
    parameter_list[4] = MSB(sniff_slot_offset);
    parameter_list[5] = LSB(new_sniff_interval);
    parameter_list[6] = MSB(new_sniff_interval);
    parameter_list[7] = LSB(sniff_attempt);
    parameter_list[8] = MSB(sniff_attempt);
    parameter_list[9] = LSB(sniff_timeout);
    parameter_list[10] = MSB(sniff_timeout);

    lmp_generate_pdu(ce_index, parameter_list, LMP_SNIFF_REQ_LEN,
                     (LMP_TRAN_ID)trans_id, LMP_SNIFF_MODE_NEG);

#ifndef _CCH_SLOT_OFFSET_
    /* For self dev initiated tn, release the Dsniff. */
    if ((ce_ptr->remote_dev_role == SLAVE) &&
            (trans_id != ce_ptr->remote_dev_role))
    {
        lmp_put_dsniff(ce_ptr->sniff_slot_offset);
    }
#endif

    /* Store the parameters in conn entity */
    ce_ptr->sniff_interval = new_sniff_interval ;
    ce_ptr->sniff_attempt = sniff_attempt;
    ce_ptr->sniff_timeout = sniff_timeout ;
    ce_ptr->sniff_slot_offset = sniff_slot_offset;
    ce_ptr->lmp_expected_pdu_opcode |=
        lmp_get_opcode_mask(LMP_SNIFF_REQ_OPCODE, 0x0);

    return HCI_COMMAND_SUCCEEDED ;
}

#endif /* COMPILE_SNIFF_MODE */

#ifdef COMPILE_HOLD_MODE
/**
 * Validates the hold mode parameters received from the remote device.
 *
 * \param trans_id Transaction ID.
 * \param ce_index ACL Connection entity index.
 * \param hold_interval Hold interval.
 * \param start_timer_flag Set to TRUE, if the hold timer is started.
 *
 * \return HCI_COMMAND_SUCCEEDED, if the operation is successful. A valid
 *         bluetooth error code, otherwise.
 */
UCHAR lmp_validate_hold_parms(UCHAR trans_id, UINT16 ce_index,
                              UINT16 hold_interval, UCHAR *start_timer_flag)
{
    UINT16 super_to ;
    UINT16 new_hold_interval = 0;
    UCHAR parameter_list[LMP_HOLD_REQ_LEN];
    UINT32 clock_value;
    UINT32 hold_instant ;
    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];
    LMP_INF(HOLD_INTERVAL,1,hold_interval);

    super_to = ce_ptr->link_supervision_timeout;

    LMP_INF(SUPERVISION_TIMEOUT_IN_SLOTS,1,super_to);

    /* Check hold interval is greater than supervision timeout*/
    if(( ( hold_interval + ce_ptr->Tpoll) > super_to) && (super_to != 0 ))
    {
        if(ce_ptr->hold_min_interval_negotiated == TRUE)
        {
            return INVALID_LMP_PARAMETERS_ERROR;
        }
        else
        {
            if (super_to > 50)
            {
                new_hold_interval = (UINT16)(super_to - 50);
            }
            ce_ptr->hold_min_interval_negotiated = TRUE;
        }
    }
    else
        /*
            The hold param nego is based on the hold min int and hold max int
            values given by the host if self device initiates the transaction.
        */
        if((trans_id != ce_ptr->remote_dev_role) &&
                (hold_interval < ce_ptr->hold_mode_min_interval) )
        {
            if(ce_ptr->hold_min_interval_negotiated == TRUE)
            {
                return INVALID_LMP_PARAMETERS_ERROR ;
            }
            else
            {
                new_hold_interval = ce_ptr->hold_mode_min_interval ;
                ce_ptr->hold_min_interval_negotiated = TRUE;
            }
        }
        else
            /*
                The hold param negotiation is based on the hold min interval and hold
                max interval values
                given by the host if self device initates the transaction.
            */
            if((hold_interval > ce_ptr->hold_mode_max_interval)
                    && (trans_id != ce_ptr->remote_dev_role))
            {
                if(ce_ptr->hold_max_interval_negotiated == TRUE)
                {
                    LMP_ERR(HOLD_INTERVAL_IS_GREATERE_THAN_HOLD_MAX_INTERVAL_AND_HOLD_MAX_INTERVAL_NEGOTIATED,0,0);
                    return INVALID_LMP_PARAMETERS_ERROR ;
                }
                else
                {
                    new_hold_interval = ce_ptr->hold_mode_max_interval ;
                    ce_ptr->hold_max_interval_negotiated = TRUE;
                }
            }
            else
            {
                /* Hold interval is within the acceptable range. */
                LMP_INF(SENDING_ACCEPTED_FOR_HOLD_REQ_TO_THE_REMOTE_DEV,0,0);
                lmp_send_lmp_accepted(ce_index, LMP_HOLD_REQ_OPCODE,
                                      trans_id, LMP_HOLD_MODE_NEG);
                ce_ptr->hold_mode_interval = hold_interval;

                if(hold_interval>ce_ptr->hold_neg_max_interval)
                {
                    ce_ptr->hold_neg_max_interval = hold_interval ;
                }
                *start_timer_flag = FALSE ;

                ce_ptr->hold_mode_accepted_flag = TRUE ;

                /* Reset the hold interval values */
                ce_ptr->hold_mode_max_interval = otp_str_data.bt_hold_max_interval;
                ce_ptr->hold_mode_min_interval = otp_str_data.bt_hold_min_interval;
                ce_ptr->hold_max_interval_negotiated = 0x00;
                ce_ptr->hold_min_interval_negotiated = 0x00;
                return HCI_COMMAND_SUCCEEDED ;
            }

    /*
    Since you are requesting back hold request the hold instant should always be
    atleast 9*Tpoll slots
    */

    lc_get_clock_in_scatternet(&clock_value, ce_ptr->phy_piconet_id);

    clock_value >>= 1 ;

    hold_instant = clock_value +
                   (ce_ptr->Tpoll * HOLD_REQ_INSTANT_MULTIPLIER) ;

    /* Make sure that the instant is even. */
    hold_instant = (UINT32) (hold_instant & (~0x01));

    ce_ptr->hold_instant = hold_instant ;
    parameter_list[0] = LMP_HOLD_REQ_OPCODE ;
    parameter_list[2] = LSB(new_hold_interval);
    parameter_list[3] = MSB(new_hold_interval);
    bt_fw_ultostr(&parameter_list[4], hold_instant, 4);

    lmp_generate_pdu(ce_index, parameter_list, LMP_HOLD_REQ_LEN,
                     (LMP_TRAN_ID)trans_id, LMP_HOLD_MODE_NEG);

    *start_timer_flag = TRUE ;

    /* Store the new hold interval in the conn entity*/
    ce_ptr->hold_mode_interval = new_hold_interval ;
    ce_ptr->lmp_expected_pdu_opcode |=
        lmp_get_opcode_mask(LMP_HOLD_REQ_OPCODE, 0x0);

    return HCI_COMMAND_SUCCEEDED ;

}
#endif /* COMPILE_HOLD_MODE */



/**
 * Start mint_os_timer from task context. This function should be used
 * when starting timer from interrupt context. Use OS_START_TIMER
 * if in task context.
 *
 * \param timerid Timer ID value
 * \param value   Timer timeout duration
 *
 * \return None
 */
void lmp_send_os_start_timer_signal(TimerHandle_t timerid, UINT32 value)
{
    OS_SIGNAL signal;

    signal.type = LMP_START_TIMER_SIGNAL;
    signal.param = (OS_ADDRESS)timerid;
    signal.ext_param = (OS_ADDRESS)value;
    OS_ISR_SEND_SIGNAL_TO_TASK(lmp_task_handle, signal);
}

/**
 * Generates a random number.
 *
 * \param None.
 *
 * \return One byte random number.
 */
UCHAR lmp_generate_random_number(void)
{
    UCHAR rand;
    static UINT32 idum = 0x0L;

    idum = (UINT32)(1664525L * idum + 1013904223L);
    rand = (UCHAR)(idum >> 24);

    return rand;
}

/**
 * Stops the PDU response timer and Link supervision timer of the ACL
 * connection given by the \a ce_index.
 *
 * \param ce_index Index of the ACL connection entity database.
 *
 * \return None.
 */
void lmp_stop_connection_timers(UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY* ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    if(ce_ptr->supervision_timeout_flag == TRUE)
    {
        OS_STOP_TIMER(ce_ptr->supervision_timeout_handle, 0);

        ce_ptr->supervision_timeout_flag = FALSE;
    }

    if(ce_ptr->pdu_response_timer_running == TRUE)
    {
        OS_STOP_TIMER(ce_ptr->lmp_response_timer_handle, 0);
        ce_ptr->pdu_response_timer_running = FALSE;
    }

    return;
}

/**
 * Timeout handler for disconnecting the ACL connection.
 *
 * \param timer_handle Timer handle.
 * \param ce_index ACL connection entity index.
 *
 * \return None.
 */
void lmp_detach_connection_timer_handler(TimerHandle_t timer_handle)
{
    UINT16 ce_index = (UINT16)((UINT32) pvTimerGetTimerID(timer_handle));
    LMP_CONNECTION_ENTITY* ce_ptr;
    OS_SIGNAL sig_send;

    ce_ptr = &lmp_connection_entity[ce_index];
    LMP_INF(DISCONNECT_TIMER_EXPIRED_AM_ADDR_CE_INDEX,2,ce_ptr->am_addr, ce_index);

    if(ce_ptr->disconnect_reason == CONNECTION_TERMINATED_USER_ERROR &&
            ce_ptr->detach_timer_state != REMOTE_DETACH_STATE)
    {
        ce_ptr->disconnect_reason = CONNECTION_TERMINATED_LOCAL_HOST_ERROR;
#ifdef _CCH_PAGE_CON_
        ce_ptr->connect_reason = 0;
#endif

    }

    switch(ce_ptr->detach_timer_state)
    {
        case SIX_TPOLL_STATE:
            /* Ack is not received for lmp_detach yet. */
            lmp_disconnect_links(ce_index, ce_ptr->disconnect_reason);

            /* Generate Disconnection complete event now. */
            sig_send.type = HCI_DISCONNECT_COMPLETE_SIGNAL;
            sig_send.param = (OS_ADDRESS)((UINT32)ce_index);
            OS_SEND_SIGNAL_TO_TASK(hci_event_task_handle, sig_send);

            sig_send.type = LMP_DELETE_TIMER_HANDLE_SIGNAL;
            sig_send.param = (OS_ADDRESS)((UINT32)(ce_ptr->detach_connection_timer_handle));
            OS_SEND_SIGNAL_TO_TASK(lmp_task_handle, sig_send);

            ce_ptr->detach_connection_timer_handle = NULL;

            ce_ptr->detach_timer_state = IDLE_STATE;
            break;

        case REMOTE_DETACH_STATE:
            lmp_disconnect_links(ce_index, ce_ptr->disconnect_reason);
            /* Fall through */

        case THREE_TPOLL_STATE: /* Fall through */
        case SUPERVISION_STATE:
            sig_send.type = HCI_DISCONNECT_COMPLETE_SIGNAL;
            sig_send.param = (OS_ADDRESS)((UINT32)ce_index);
            OS_SEND_SIGNAL_TO_TASK(hci_event_task_handle, sig_send);

            sig_send.type = LMP_DELETE_TIMER_HANDLE_SIGNAL;
            sig_send.param = (OS_ADDRESS)((UINT32)(ce_ptr->detach_connection_timer_handle));
            OS_SEND_SIGNAL_TO_TASK(lmp_task_handle, sig_send);

            ce_ptr->detach_connection_timer_handle = NULL;

            ce_ptr->detach_timer_state = IDLE_STATE;

            break;

        default:
            break;
    }

    return;
}

/**
 * Sends LMP_accepted to the remote device. This function should not be used
 * directly. Use #lmp_send_lmp_accepted, #lmp_send_lmp_accepted_dm1 accessor
 * macros instead.
 *
 * \param ce_index ACL connection entity index
 * \param opcode Opcode for which this LMP_accepted is being sent.
 * \param trans_id Transaction ID.
 * \param ce_status Connection entity status (Main state machine status).
 * \param use_dm1 TRUE, if DM1 has to be used to transmit the PDU. FALSE,
 *                if any packet type (DV/DM1) can be used.
 * \return None.
 */
void lmp_send_lmp_accepted_impl(UINT16 ce_index, UCHAR opcode,
                                UCHAR trans_id,  UINT8 ce_status, UCHAR use_dm1)
{
    UCHAR parameter_list[LMP_ACCEPTED_LEN];

    parameter_list[0] = LMP_ACCEPTED_OPCODE;
    parameter_list[2] = opcode;
    lmp_generate_pdu_impl(ce_index, parameter_list, LMP_ACCEPTED_LEN,
                          (LMP_TRAN_ID)trans_id, ce_status, use_dm1);
}

/**
 * Sends LMP_not_accepted to the remote device.
 *
 * \param ce_index ACL connection entity index.
 * \param opcode Opcode for which this LMP_not_accepted is being sent.
 * \param trans_id Transaction ID.
 * \param reason Reason for rejection.
 *
 * \return None.
 */
void lmp_send_lmp_not_accepted(UINT16 ce_index,UCHAR opcode,UCHAR trans_id,
                               UCHAR reason)
{
    UCHAR parameter_list[LMP_NOT_ACCEPTED_LEN];

    /*send link key to remote device*/
    parameter_list[0] = LMP_NOT_ACCEPTED_OPCODE;
    parameter_list[2] = opcode;
    parameter_list[3] = reason;
    lmp_generate_pdu(ce_index,parameter_list, LMP_NOT_ACCEPTED_LEN,
                     (LMP_TRAN_ID)trans_id, LMP_NO_STATE_CHANGE);
}

#ifdef COMPILE_PARK_MODE
/**
 * Handles the unpark request.
 *
 * \param ce_index ACL connection entity index.
 *
 * \return HCI_COMMAND_SUCCEEDED, if the operation is successful. A valid
 *         bluetooth error code, otherwise.
 */
UCHAR lmp_handle_master_unpark(UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY *ce_ptr;
    UCHAR am_addr;
    UCHAR phy_piconet_id;
    UCHAR lut_index;
    DEF_CRITICAL_SECTION_STORAGE;

    ce_ptr = &lmp_connection_entity[ce_index];

    /* If we are master, start a Master Initiated Unparking procedure */
    /* Get free AM address to assign to this device */
    am_addr = ce_ptr->am_addr;

    if(ce_ptr->unpark_req_flag == LMP_UNPARK_HOST_INITIATED)
    {
        return COMMAND_DISALLOWED_ERROR;
    }

    if(ce_ptr->unpark_req_flag == LMP_UNPARK_AUTO_CONTROLLER_INITIATED)
    {
        ce_ptr->unpark_req_flag = LMP_UNPARK_HOST_INITIATED;
        return HCI_COMMAND_SUCCEEDED;
    }

    lut_ex_table[BC_AM_ADDR].index_in_CE = ce_index;

    phy_piconet_id = ce_ptr->phy_piconet_id;
    lut_index = lc_get_lut_index_from_phy_piconet_id(am_addr, phy_piconet_id);

    if (bz_auth_is_link_encrypted(ce_index))
    {
        bz_auth_enable_link_level_encryption(ce_index, TRUE);
    }

    lmp_create_unpark_req_pdu(ce_index, 0);

    lmp_unpark_ce_index = ce_index;

    /* Kill all scan before starting unpark procedure */
    lc_kill_scan_mode();

    MINT_OS_ENTER_CRITICAL();
    /* Set Flag to send unpark PDU at the beginning of the Beacon. */
    ce_ptr->unpark_req_flag = LMP_UNPARK_HOST_INITIATED;
    MINT_OS_EXIT_CRITICAL();

    return HCI_COMMAND_SUCCEEDED;
}
#endif

#ifndef _CCH_SLOT_OFFSET_
/**
 * Returns unique Dsniff value.
 *
 * \param None.
 *
 * \return Dsniff value.
 */
UINT16 lmp_get_dsniff(void)
{
    UINT16 i;
    UINT16 sniff_val = 0;
    UCHAR lc_dsniffs = lmp_self_device_data.lc_dsniffs;

    for (i = 0; i < 8; i++)
    {
        if (((0x1<<i) & lc_dsniffs) == 0)
        {
            sniff_val = (UINT16)(i << 0x02);
            lmp_self_device_data.lc_dsniffs = (UCHAR)(lc_dsniffs | (1<<i));
            break;
        }
    }

#ifdef ENABLE_LOGGER_LEVEL_2
    LMP_LOG_INFO(LOG_LEVEL_HIGH, GET_DSNIFF, 1, sniff_val);
#endif

    return sniff_val;
}

/**
 * Frees the given Dsniff value so that it can be used for other slaves.
 *
 * \param sniff_val Dsniff value to be freed.
 *
 * \return None.
 */
void lmp_put_dsniff(UINT16 sniff_val)
{
    lmp_self_device_data.lc_dsniffs = (UCHAR)
                                      (lmp_self_device_data.lc_dsniffs & ~(1 << (sniff_val>>2)));

#ifdef ENABLE_LOGGER_LEVEL_2
    LMP_LOG_INFO(LOG_LEVEL_HIGH, PUT_DSNIFF, 1, sniff_val);
#endif

    return;
}

#endif

/**
 * Get the periodic free slot offset for  ce_index/is_esco_ce_index between 0~(GLOBAL_SLOT_INTERVAL-1)  (cch)
 *
 * \param ce_index:	"0~(LMP_MAX_CE_DATABASE_ENTRIES-1)" for acl ce_index
 *           is_esco_ce_index: "0" for "not" (e)sco ce_index, "10~15" for "is" esco_ce_index + LMP_MAX_CE_DATABASE_ENTRIES
 *                           :                                "20~24" for "is" sco_ce_index + 2*LMP_MAX_CE_DATABASE_ENTRIES
 *           interval:		"0~no limit", 0 will equal to GLOBAL_SLOT_INTERVAL
 *           slot_num: 	"0~no limit", 0 will equal to 2
 *
 * \return slot_val: 	"0~(GLOBAL_SLOT_INTERVAL-1)"	for free slot offset,
 *                       	"GLOBAL_SLOT_INTERVAL"		for "no" free slot to be used.
 */

#ifdef _CCH_SLOT_OFFSET_
void global_slot_offset_log()
{
    UCHAR i = 0;
    //RT_BT_LOG(GREEN, CCH_DBG_030, 0, 0);
    for( i = 0; i < GLOBAL_SLOT_USE_NUM; i++)
    {
        if (lmp_self_device_data.global_slot_use_acl_ce_index[i] != 0xFF)
        {
            RT_BT_LOG(GREEN, CCH_DBG_031, 7,
                lmp_self_device_data.global_slot_use_acl_ce_index[i],
                lmp_self_device_data.global_slot_use_ce_index[i],
                lmp_self_device_data.global_slot_use_interval[i],
                lmp_self_device_data.global_slot_use_slot_offset[i],
                lmp_self_device_data.global_slot_use_remote_slot[i],
                lmp_self_device_data.global_slot_use_slot_num[i],
                lmp_self_device_data.global_slot_interval[i]);
        }
    }
}
#ifdef _DAPE_GET_LEGACY_SLOT_OFST_AVOID
void lmp_get_avoid_slot_offset(UCHAR *avoid, UCHAR *slot_ofst)
{
    UINT8 i = 0;
    for (i = 0; i < GLOBAL_SLOT_USE_NUM; i ++)
    {
        if(lmp_self_device_data.global_slot_use_ce_index[i] !=
            lmp_self_device_data.global_slot_use_acl_ce_index[i])
        {
            *avoid = 1;
            *slot_ofst = lmp_self_device_data.global_slot_use_slot_offset[i];
            return;
        }
    }
    *avoid = 0;
    *slot_ofst = 0xFF;
    return;
}
#endif
UINT16 lmp_get_global_slot_offset(UINT16 ce_index, UINT16 is_esco_ce_index, UINT16 interval, UINT16 slot_num, UINT16 slot_min, UINT16 slot_max)
{

    UCHAR slot_val;
    UCHAR slot_in_use[GLOBAL_SLOT_INTERVAL];
#ifdef _MODI_SLOT_OFFSET_AFTER_RTL8821B_TC_
    UCHAR slot_in_use_intreval[GLOBAL_SLOT_INTERVAL];
#endif
    UCHAR global_index;
    UCHAR use_ce_index;
    UINT16 use_interval;
    UCHAR use_slot_offset;
    UCHAR use_slot_num;
    UCHAR use_acl_ce_index;
    UCHAR index_ce_index;
    UCHAR index_interval;
    UCHAR index_slot_num;
    UCHAR use_slot_min;
    UCHAR use_slot_max;
    LMP_CONNECTION_ENTITY *ce_ptr = NULL;
    UCHAR find_slot_fail = 1;
    UCHAR find_slot_val;
    UCHAR temp_max;
    UCHAR temp_use_4 = 0;
    UCHAR is_sco_exist = 0;
    UCHAR slot_overlap;
    UCHAR is_LE = FALSE;

#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8723_RCP_
// RCP for change the whole function or Just change the input paramter
// Change Parameter: all of the input paramter can be changed

    if (rcp_lmp_get_global_slot_offset != NULL)
    {

        if ( rcp_lmp_get_global_slot_offset((void *)&slot_val,
			ce_index, &is_esco_ce_index, &interval, &slot_num, &slot_min, &slot_max) )
        {
            return slot_val;
        }
    }
#endif
#endif

    if (ce_index >= LL_HCI_MIN_CONN_HANDLE)
    {
        is_LE = TRUE;
    }
    //else
    //{
    //    RT_BT_LOG(WHITE, CCH_DBG_035, 4, ce_index, is_esco_ce_index, interval, slot_num);
    //}

// Step 1: Inital slot_in_use=0xff "unused"
// or 0 "unused"

    memset(slot_in_use, 0xFF, GLOBAL_SLOT_INTERVAL);
#ifdef _MODI_SLOT_OFFSET_AFTER_RTL8821B_TC_
    memset(slot_in_use_intreval, 0, GLOBAL_SLOT_INTERVAL);
#endif

// Step 2: Release the same (ce_index, is_esco_ce_index) slot, assume there is only one periodic link in one ce_index"
// Step 3: Check if used global_slot_use_ce_index < GLOBAL_SLOT_USE_NUM
//            if there is no free index, just return with slot_val = GLOBAL_SLOT_INTERVAL;

    global_index = lmp_find_global_slot_index(ce_index, is_esco_ce_index);

    if (global_index >= GLOBAL_SLOT_USE_NUM)
    {
        slot_val = GLOBAL_SLOT_INTERVAL;
        return slot_val;
    }

// Step 4: Find the slot_in_use=use_ce_index by lmp_self_device_data.global_slot_use_ce_index/_interval/_slot_offset/_slot_num
// slot_in_use[index_slot_num] = use_ce_index;  can be replace by slot_in_use[index_slot_num] = 1; "used"
    for (index_ce_index = 0; index_ce_index < GLOBAL_SLOT_USE_NUM; index_ce_index++)
    {
        use_ce_index = lmp_self_device_data.global_slot_use_ce_index[index_ce_index];
        use_acl_ce_index = lmp_self_device_data.global_slot_use_acl_ce_index[index_ce_index];

        if(use_acl_ce_index != 0xff)
        {
            if (use_acl_ce_index < LL_HCI_MIN_CONN_HANDLE)
            {
                slot_overlap = lmp_update_global_slot_offset(use_acl_ce_index, use_ce_index);
            }
//    RT_BT_LOG(BLUE, CCH_DBG_057, 3, use_acl_ce_index, use_ce_index, slot_overlap);
            use_interval =lmp_self_device_data.global_slot_use_interval[index_ce_index];
            use_slot_offset =lmp_self_device_data.global_slot_use_slot_offset[index_ce_index];
            use_slot_num =lmp_self_device_data.global_slot_use_slot_num[index_ce_index];

            if( use_acl_ce_index != use_ce_index )
            {
                is_sco_exist = 1;
                //RT_BT_LOG(WHITE, CCH_DBG_033, 1, is_sco_exist);
            }

            /* added by austin to avoid potential infinite loop */
            if (use_interval == 0)
            {
                use_interval = GLOBAL_SLOT_INTERVAL;
            }

            for(index_interval=use_slot_offset; index_interval<GLOBAL_SLOT_INTERVAL; index_interval+=use_interval)
            {
                UINT8 index_slot_num_upper_bound = index_interval+use_slot_num;
                for(index_slot_num=index_interval; index_slot_num < index_slot_num_upper_bound; index_slot_num++)
                {
                    if(index_slot_num < GLOBAL_SLOT_INTERVAL)
                    {
                        slot_in_use[index_slot_num] = use_acl_ce_index;

#ifdef _MODI_SLOT_OFFSET_AFTER_RTL8821B_TC_
                        slot_in_use_intreval[index_slot_num] =
                            lmp_self_device_data.global_slot_use_interval[index_ce_index];
#endif
                    }
                    else
                    {
                        slot_in_use[index_slot_num - GLOBAL_SLOT_INTERVAL] = use_acl_ce_index;

#ifdef _MODI_SLOT_OFFSET_AFTER_RTL8821B_TC_
                        slot_in_use_intreval[index_slot_num - GLOBAL_SLOT_INTERVAL] =
                            lmp_self_device_data.global_slot_use_interval[index_ce_index];
#endif

                    }
                }
            }
        }
    }

// Step 5: Initial the new link parameter
    if (!is_LE)
    {
        ce_ptr = &lmp_connection_entity[ce_index];
    }
    slot_val = 0xff;

    use_ce_index = ce_index;

    /* use_interval is even */
    use_interval = (interval + 1) & ~0x01;


    if ( (use_interval >= GLOBAL_SLOT_INTERVAL) || (use_interval == 0) || ( ((use_interval%6)!=0) &&(use_interval >6) ) )
    {
        // use_interval must equal to  6*N or GLOBAL_SLOT_INTERVAL
        use_interval = GLOBAL_SLOT_INTERVAL;
    }


    if((slot_num < use_interval) && (slot_num != 0))
    {
        use_slot_num = (slot_num + 1) & ~0x01;
        if (use_slot_num >= 6)
        {
            use_slot_num = 4;
        }
    }
    else
    {
        use_slot_num = 2;
    }

#ifdef _MODI_SLOT_OFFSET_AFTER_RTL8821B_TC_
    if( interval >= 0x200)
    {
        use_slot_num = 2;
    }
#endif

    if (is_LE)
    {
        slot_min = (slot_min + 1) & ~0x01;
        slot_max = (slot_max + 1) & ~0x01;

        if( (slot_max - slot_min) >= use_interval)
        {
            use_slot_min = 0;
            use_slot_max = use_interval;
        }
        else
        {
            use_slot_min = slot_min % use_interval;
            use_slot_max = slot_max % use_interval;
        }
    }
    else
    {
        if(ce_ptr->remote_dev_role == MASTER)
        {
            use_slot_num = use_slot_num + 1;
            if (use_slot_num >= 5)
            {
                use_slot_num = 3;
            }
        }

        use_slot_min = 0;
        use_slot_max = use_interval;
    }

// cch: Reserve slot number 4,5 for SCO. Original is reserve slot 0, but has problem in BQB
//        if is sco link exist, then other link can use slot 4,5
    temp_use_4 = (use_interval <= 0x6) ? 0x01 : 0x00;
    temp_use_4 |= is_sco_exist;
#ifdef _MODI_SLOT_OFFSET_AFTER_RTL8821B_TC_
    temp_use_4 |= is_esco_ce_index;
#endif

#ifdef _CCH_SNIFF_NEG_TIMEOUT_
    temp_use_4 |= g_efuse_rsvd_2.rsvd_for_sco_slot;
#endif

    RT_BT_LOG(WHITE, CCH_DBG_033, 1, temp_use_4);

// Step 6: Find the free slot, if there is no free slot to be used, return slot_val = GLOBAL_SLOT_INTERVAL;
    /* (20120813)modify the for loop criterion as below as chiachun recommands.
        for win8 sco. */
    //for (find_slot_val = 0; find_slot_val < use_interval; find_slot_val += 2)
    for (find_slot_val = 0; find_slot_val < GLOBAL_SLOT_INTERVAL; find_slot_val += 2)
    {
        find_slot_fail = 1;

        if( (!temp_use_4) && ((find_slot_val%6) == 4) )
        {
            continue;
        }

        if( use_slot_min < use_slot_max )
        {
            if ((find_slot_val < use_slot_min) ||
                (find_slot_val>use_slot_max))
            {
                continue;
            }
        }
        if( use_slot_min > use_slot_max )
        {
            if ((find_slot_val <use_slot_min) &&
                (find_slot_val>use_slot_max) )
            {
                continue;
            }
        }

        if(slot_in_use[find_slot_val] == 0xff)
        {
            find_slot_fail = 0;

            for (index_interval=find_slot_val; index_interval<GLOBAL_SLOT_INTERVAL; index_interval+=use_interval)
            {
                temp_max = MIN(GLOBAL_SLOT_INTERVAL,
                               (index_interval+use_slot_num));

                for (index_slot_num=index_interval; index_slot_num<temp_max; index_slot_num++)
                {
                    if(( !g_efuse_lps_setting_3.iot_sco_can_overlap)&&
                       (use_slot_num == 2))
                    {

#ifdef _MODI_SLOT_OFFSET_AFTER_RTL8821B_TC_
                        if(((slot_in_use[index_slot_num] != 0xff) &&
                            (slot_in_use_intreval[index_slot_num] > 6)) &&
                            (use_interval > 6))
#else
                        if((slot_in_use[index_slot_num] != 0xff) ||
                            ( (!temp_use_4) && ((index_slot_num%6) == 4) ) )
#endif
                        {
                            if (( slot_in_use[index_slot_num] != use_ce_index ) &&
                                (slot_in_use[index_slot_num] < LL_HCI_MIN_CONN_HANDLE))
                            {
                                //RT_BT_LOG(BLUE, CCH_DBG_033, 1,index_slot_num);
                                find_slot_fail = 1;
                                break;
                            }
                        }
                    }
                }
            }

            if(find_slot_fail == 0)
            {
                break;
            }
        }
    }


    if (find_slot_fail)
    {
        slot_val = GLOBAL_SLOT_INTERVAL;
        return slot_val;
    }


// Step 7: Write the use_slot information into the free lmp_self_device_data at global_index

    if(is_esco_ce_index >0)
    {
        use_ce_index = is_esco_ce_index;
    }
    else
    {
        use_ce_index = ce_index;
    }

    lmp_self_device_data.global_slot_use_ce_index[global_index] = use_ce_index;
    lmp_self_device_data.global_slot_use_acl_ce_index[global_index] = ce_index;
    lmp_self_device_data.global_slot_use_interval[global_index] = use_interval;
    lmp_self_device_data.global_slot_use_slot_offset[global_index] = find_slot_val;
    lmp_self_device_data.global_slot_use_slot_num[global_index] = use_slot_num;
    lmp_self_device_data.global_slot_interval[global_index] = interval;

// Step 8: Calculate the master slot offset depend on the device role
    if (is_LE)
    {
        slot_val = find_slot_val;
    }
    else
    {
        slot_val = lmp_cal_m2s_clock_offset(ce_index, interval , use_interval, 0);
        slot_val = (slot_val + find_slot_val)%use_interval;
//	RT_BT_LOG(YELLOW, CCH_DBG_026, 1,slot_val);
    }

// global_slot_use_remote_slot is remote master
    lmp_self_device_data.global_slot_use_remote_slot[global_index] = slot_val;

    /* dape added 20120928 for Dual mode slot offset monitor. */
    {
        OS_SIGNAL sig_send;

        /* Send signal to debug */
        sig_send.type = ISR_EXT_SLOT_OFST_LOG;
        OS_ISR_SEND_SIGNAL_TO_TASK(isr_extended_task_handle, sig_send);
    }
// JUST LOG: START
/*
    if (!is_LE)
    {
        RT_BT_LOG(GREEN, CCH_DBG_030, 0, 0);
	for(index_ce_index=0; index_ce_index<GLOBAL_SLOT_USE_NUM; index_ce_index++)
	{
		RT_BT_LOG(GREEN, CCH_DBG_031, 7,
		lmp_self_device_data.global_slot_use_acl_ce_index[index_ce_index],
		lmp_self_device_data.global_slot_use_ce_index[index_ce_index],
		lmp_self_device_data.global_slot_use_interval[index_ce_index],
		lmp_self_device_data.global_slot_use_slot_offset[index_ce_index],
		lmp_self_device_data.global_slot_use_remote_slot[index_ce_index],
		lmp_self_device_data.global_slot_use_slot_num[index_ce_index],
		lmp_self_device_data.global_slot_interval[index_ce_index]);

	}
    }
    */
// JUST LOG: END

    return slot_val;
}

/**
 * Release ce_index/is_esco_ce_index used slot (cch)
 *
 * \param ce_index, is_esco_ce_index: 0 for "not" esco ce_index, 10~15 for "is" esco_ce_index + LMP_MAX_CE_DATABASE_ENTRIES
 *
 * \return None.
 */

void lmp_put_global_slot_offset(UINT16 ce_index, UINT16 is_esco_ce_index)
{
    UCHAR index_ce_index;
    UCHAR use_ce_index;


    if(is_esco_ce_index >0)
    {
        use_ce_index = is_esco_ce_index;
    }
    else
    {
        use_ce_index = ce_index;
    }

//	RT_BT_LOG(GREEN, CCH_DBG_019, 0, 0);
// free the global_slot_offset in this ce_index
    for(index_ce_index=0; index_ce_index<GLOBAL_SLOT_USE_NUM; index_ce_index++)
    {
        if(lmp_self_device_data.global_slot_use_ce_index[index_ce_index] == use_ce_index)
        {
            lmp_self_device_data.global_slot_use_ce_index[index_ce_index] = 0xff;
            lmp_self_device_data.global_slot_use_acl_ce_index[index_ce_index] = 0xff;
            lmp_self_device_data.global_slot_use_interval[index_ce_index] = 0xff;
            lmp_self_device_data.global_slot_use_slot_offset[index_ce_index] = 0xff;
            lmp_self_device_data.global_slot_use_remote_slot[index_ce_index] = 0xff;
            lmp_self_device_data.global_slot_use_slot_num[index_ce_index] = 0xff;
            lmp_self_device_data.global_slot_interval[index_ce_index] = 0xff;
        }
    }

    return;
}

void lmp_disconnect_global_slot_offset(UINT16 ce_index)
{
    UCHAR index_ce_index;


// free the global_slot_offset in this ce_index
    for(index_ce_index=0; index_ce_index<GLOBAL_SLOT_USE_NUM; index_ce_index++)
    {
        if(lmp_self_device_data.global_slot_use_acl_ce_index[index_ce_index] == ce_index)
        {
            lmp_self_device_data.global_slot_use_ce_index[index_ce_index] = 0xff;
            lmp_self_device_data.global_slot_use_acl_ce_index[index_ce_index] = 0xff;
            lmp_self_device_data.global_slot_use_interval[index_ce_index] = 0xff;
            lmp_self_device_data.global_slot_use_slot_offset[index_ce_index] = 0xff;
            lmp_self_device_data.global_slot_use_remote_slot[index_ce_index] = 0xff;
            lmp_self_device_data.global_slot_use_slot_num[index_ce_index] = 0xff;
            lmp_self_device_data.global_slot_interval[index_ce_index] = 0xff;
        }
    }

    return;
}



/**
 * Find ce_index/is_esco_ce_index used global_slot_use_ce_index index (cch)
 * Will release the same ce_index/is_esco_ce_index in global_slot_use_ce_index
 *
 * \param ce_index, is_esco_ce_index: 0 for "not" esco ce_index, 10~15 for "is" esco_ce_index + LMP_MAX_CE_DATABASE_ENTRIES
 *
 * \return index_ce_index: 	"0~(GLOBAL_SLOT_USE_NUM-1)"	for free global_slot_use_ce_index index,
 *                       		"GLOBAL_SLOT_USE_NUM"		for "no" global_slot_use_ce_index index to be used.
 */
UCHAR lmp_find_global_slot_index(UINT16 ce_index, UINT16 is_esco_ce_index)
{
    UCHAR index_ce_index;


    lmp_put_global_slot_offset(ce_index, is_esco_ce_index);

    for(index_ce_index=0; index_ce_index<GLOBAL_SLOT_USE_NUM; index_ce_index++)
    {
        if(lmp_self_device_data.global_slot_use_ce_index[index_ce_index] == 0xff)
        {
//            RT_BT_LOG(WHITE, CCH_DBG_038, 1, index_ce_index);
            return index_ce_index;
        }
    }

//	RT_BT_LOG(WHITE, CCH_DBG_037, 0, 0);
    index_ce_index = GLOBAL_SLOT_USE_NUM;
    return index_ce_index;
}



/**
 * Force the slot offset for global_slot ce_index/is_esco_ce_index into  (cch)
 *
 * \param ce_index:	"0~(LMP_MAX_CE_DATABASE_ENTRIES-1)" for acl ce_index
 *           is_esco_ce_index: "0" for "not" esco ce_index, "10~15" for "is" esco_ce_index + LMP_MAX_CE_DATABASE_ENTRIES
 *           interval:		"0~no limit", 0 will equal to GLOBAL_SLOT_INTERVAL
 *           slot_num: 	"0~no limit", 0 will equal to 2
 *		 slot_val:		"0~no limit", slot_val = slot_val%interval
 *
 * \return force_slot_overlap: 	"0"	for "no" overlap,
 *                       			"1"	for "is" overlap.
 */
UCHAR lmp_force_global_slot_offset(UINT16 ce_index, UINT16 is_esco_ce_index, UINT16 interval, UINT16 slot_num, UINT16 slot_val)
{
    UCHAR slot_in_use[GLOBAL_SLOT_INTERVAL];
    UCHAR global_index;
    UCHAR use_ce_index;
    UINT16 use_interval;
    UCHAR use_slot_offset;
    UCHAR use_slot_num;
    UCHAR index_ce_index;
    UCHAR index_interval;
    UCHAR index_slot_num;

    LMP_CONNECTION_ENTITY *ce_ptr = NULL;

    UCHAR force_slot_overlap;
    UCHAR temp_max;
    UCHAR is_LE = FALSE;

    if (ce_index >= LL_HCI_MIN_CONN_HANDLE)
    {
        is_LE = TRUE;
    }
    if (!is_LE)
    {
        ce_ptr = &lmp_connection_entity[ce_index];
    }

    if (!is_LE)
    {
        RT_BT_LOG(WHITE, CCH_DBG_036, 5, ce_index, is_esco_ce_index, interval, slot_num, slot_val);
    }
    global_index = lmp_find_global_slot_index(ce_index, is_esco_ce_index);

    if(global_index >= GLOBAL_SLOT_USE_NUM)
    {
        force_slot_overlap = 1;
        return force_slot_overlap;
    }

    memset(slot_in_use, 0xFF, GLOBAL_SLOT_INTERVAL);

// find slot in use
    for(index_ce_index=0; index_ce_index<GLOBAL_SLOT_USE_NUM; index_ce_index++)
    {
        use_ce_index = lmp_self_device_data.global_slot_use_acl_ce_index[index_ce_index];

        if(use_ce_index != 0xff)
        {
            use_interval =lmp_self_device_data.global_slot_use_interval[index_ce_index];
            use_slot_offset =lmp_self_device_data.global_slot_use_slot_offset[index_ce_index];
            use_slot_num =lmp_self_device_data.global_slot_use_slot_num[index_ce_index];

            /* added by austin to avoid potential infinite loop */
            if (use_interval == 0)
            {
                use_interval = GLOBAL_SLOT_INTERVAL;
            }

            for(index_interval=use_slot_offset; index_interval<GLOBAL_SLOT_INTERVAL; index_interval+=use_interval)
            {
                UINT8 index_slot_num_upper_bound = (index_interval+use_slot_num);
                for(index_slot_num=index_interval; index_slot_num<index_slot_num_upper_bound; index_slot_num++)
                {
                    if( index_slot_num < GLOBAL_SLOT_INTERVAL )
                    {
                        slot_in_use[index_slot_num] = use_ce_index;
                    }
                    else
                    {
                        slot_in_use[index_slot_num - GLOBAL_SLOT_INTERVAL] = use_ce_index;
                    }

                }
            }
        }
    }


    if(is_esco_ce_index >0)
    {
        use_ce_index = is_esco_ce_index;
    }
    else
    {
        use_ce_index = ce_index;
    }

    use_interval = (interval + 1) & ~0x01;

    if ( (use_interval >= GLOBAL_SLOT_INTERVAL) || (use_interval == 0) || ( ((use_interval%6)!=0) &&(use_interval >6) ) )
    {
        // use_interval must equal to  6*N or GLOBAL_SLOT_INTERVAL
        use_interval = GLOBAL_SLOT_INTERVAL;
    }

    if((slot_num < use_interval) && (slot_num != 0))
    {
        use_slot_num = (slot_num + 1) & ~0x01;

        if (use_slot_num >= 6)
        {
            use_slot_num = 4;
        }
    }
    else
    {
        use_slot_num = 2;
    }

    if (is_LE)
    {
        use_slot_offset = slot_val%use_interval;
    }
    else
    {
        if(ce_ptr->remote_dev_role == MASTER)
        {
            use_slot_num = use_slot_num + 1;
            if (use_slot_num >= 5)
            {
                use_slot_num = 3;
            }
        }

        use_slot_offset = lmp_cal_m2s_clock_offset(ce_index, interval , use_interval, 1);
        use_slot_offset = (slot_val + use_slot_offset)%use_interval;
//	RT_BT_LOG(YELLOW, CCH_DBG_026, 1,use_slot_offset);
    }

    lmp_self_device_data.global_slot_use_acl_ce_index[global_index] = ce_index;
    lmp_self_device_data.global_slot_use_ce_index[global_index] = use_ce_index;
    lmp_self_device_data.global_slot_use_interval[global_index] = use_interval;
    lmp_self_device_data.global_slot_use_slot_offset[global_index] = use_slot_offset;
    lmp_self_device_data.global_slot_use_remote_slot[global_index] = slot_val;
    lmp_self_device_data.global_slot_use_slot_num[global_index] = use_slot_num;
    lmp_self_device_data.global_slot_interval[global_index] = interval;

    force_slot_overlap = 0;


    if (!is_LE)
    {
       for(index_interval=use_slot_offset; index_interval<GLOBAL_SLOT_INTERVAL; index_interval+=use_interval)
       {

           temp_max = GLOBAL_SLOT_INTERVAL;
           if( (index_interval+use_slot_num) < temp_max)
           {
               temp_max = (index_interval+use_slot_num);
           }
           for(index_slot_num=index_interval; index_slot_num<temp_max; index_slot_num++)
           {
               UCHAR temp_slot = index_slot_num;
	       if( index_slot_num >= GLOBAL_SLOT_INTERVAL  )
               {
                   temp_slot =  index_slot_num  - GLOBAL_SLOT_INTERVAL;
               }

               if((slot_in_use[temp_slot] != 0xff)&&(slot_in_use[temp_slot] != ce_index))

               {
                   force_slot_overlap = 1;
                   break;
               }
           }
       }
    }
    /* dape added 20120928 for Dual mode slot offset monitor. */
    {
        OS_SIGNAL sig_send;

        /* Send signal to debug */
        sig_send.type = ISR_EXT_SLOT_OFST_LOG;
        OS_ISR_SEND_SIGNAL_TO_TASK(isr_extended_task_handle, sig_send);
    }

// JUST LOG: START
/*	RT_BT_LOG(GREEN, CCH_DBG_030, 0, 0);
	for(index_ce_index=0; index_ce_index<GLOBAL_SLOT_USE_NUM; index_ce_index++)
	{
		RT_BT_LOG(GREEN, CCH_DBG_031, 7,
		          lmp_self_device_data.global_slot_use_acl_ce_index[index_ce_index],
		          lmp_self_device_data.global_slot_use_ce_index[index_ce_index],
		          lmp_self_device_data.global_slot_use_interval[index_ce_index],
		          lmp_self_device_data.global_slot_use_slot_offset[index_ce_index],
		          lmp_self_device_data.global_slot_use_remote_slot[index_ce_index],
		          lmp_self_device_data.global_slot_use_slot_num[index_ce_index],
		          lmp_self_device_data.global_slot_interval[index_ce_index]);

	}
*/
// JUST LOG: END

    /* move to another position or mark it to avoid timing violation - austin */
    //RT_BT_LOG(WHITE, CCH_DBG_058, 1,force_slot_overlap);
    return force_slot_overlap;
}

UCHAR lmp_cal_m2s_clock_offset(UINT16 ce_index, UINT16 interval , UINT16 use_interval, UCHAR opt_cal_offset)
{
    UCHAR slot_val;
    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    if(ce_ptr->remote_dev_role == SLAVE)
    {
        slot_val = 0;
    }
    else
    {
        UINT32 sniff_master_clock;
        UINT32 sniff_native_clock;
        UINT32 sniff_stom_clock;
        UINT16 clock_offset_temp;

        UINT16 temp1;
        UINT16 temp2;

        DEF_CRITICAL_SECTION_STORAGE;
        MINT_OS_ENTER_CRITICAL();
        lc_get_clock_in_scatternet(&sniff_master_clock, ce_ptr->phy_piconet_id);

        temp1 = NATIVE_CLOCK1_REGISTER;
        temp2 = NATIVE_CLOCK2_REGISTER;

        sniff_native_clock = (BB_read_baseband_register(temp2) << 16) |
        				(BB_read_baseband_register(temp1));

//		RT_BT_LOG(YELLOW, CCH_DBG_025, 3,sniff_master_clock, sniff_native_clock, sniff_stom_clock);

        if (interval != 0)
        {
            sniff_master_clock = sniff_master_clock % (interval << 1);
            sniff_native_clock  = sniff_native_clock % (interval << 1);
        }
        else
        {
            sniff_master_clock = 0;
            sniff_native_clock = 0;
        }

        if (sniff_master_clock >= sniff_native_clock)
        {
            sniff_stom_clock = sniff_master_clock - sniff_native_clock;
        }
        else
        {
            sniff_stom_clock =  (interval<<1) + sniff_master_clock - sniff_native_clock;
        }

        sniff_stom_clock = (sniff_stom_clock + 1) >> 1;

//		RT_BT_LOG(YELLOW, CCH_DBG_025, 3,sniff_master_clock, sniff_native_clock, sniff_stom_clock);

        if(!opt_cal_offset)
        {
            if (sniff_stom_clock & 0x01)
            {
//                RT_BT_LOG(RED, CCH_DBG_019, 0,0);
                sniff_stom_clock = sniff_stom_clock  + 1;
            }
            else
            {
//            RT_BT_LOG(RED, CCH_DBG_019, 0,0);
                sniff_stom_clock = sniff_stom_clock + 2;
            }
        }

        if (use_interval == 0)
        {
            clock_offset_temp = 0;
            slot_val = 0;
        }
        else
        {
            clock_offset_temp = sniff_stom_clock % use_interval;
//		RT_BT_LOG(YELLOW, CCH_DBG_021, 2,sniff_stom_clock, clock_offset_temp);
            slot_val = (use_interval + clock_offset_temp) % use_interval;
        }

        MINT_OS_EXIT_CRITICAL();
    }
    return slot_val;
}

/**
 * Update the slot offset for global_slot ce_index/is_esco_ce_index due to Slave role
 *
 * \param ce_index:	"0~(LMP_MAX_CE_DATABASE_ENTRIES-1)" for acl ce_index
 *           is_esco_ce_index: "0" for "not" esco ce_index, "10~15" for "is" esco_ce_index + LMP_MAX_CE_DATABASE_ENTRIES
 *           interval:		"0~no limit", 0 will equal to GLOBAL_SLOT_INTERVAL
 *           slot_num: 	"0~no limit", 0 will equal to 2
 *		 slot_val:		"0~no limit", slot_val = slot_val%interval
 *
 * \return force_slot_overlap: 	"0"	for "no" overlap,
 *                       			"1"	for "is" overlap.
 */

UCHAR lmp_update_global_slot_offset(UINT16 ce_index, UINT16 is_esco_ce_index)
{
    UCHAR slot_overlap;
    LMP_CONNECTION_ENTITY *ce_ptr;
    UCHAR slot_in_use[GLOBAL_SLOT_INTERVAL];
    UCHAR index_ce_index;
    UCHAR use_ce_index;
    UINT16 use_interval;
    UCHAR use_slot_offset;
    UCHAR use_remote_slot;
    UCHAR use_slot_num;
    UCHAR use_acl_ce_index;
    UINT16 use_orig_interval;

    UCHAR index_interval;
    UCHAR index_slot_num;
    UCHAR temp_max;

    UCHAR global_index;

    UCHAR slot_val;

#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8723_RCP_
// RCP for change the whole function or Just change the input paramter
// Change Parameter: all of the input paramter can be changed

    if (rcp_lmp_update_global_slot_offset != NULL)
    {
        if ( rcp_lmp_update_global_slot_offset((void *)&slot_overlap,
			ce_index, is_esco_ce_index) )
        {
            return slot_overlap;
        }
    }
#endif
#endif

    ce_ptr = &lmp_connection_entity[ce_index];
    slot_overlap = FALSE;

    if(is_esco_ce_index >0)
    {
        use_ce_index = is_esco_ce_index;
    }
    else
    {
        use_ce_index = ce_index;
    }

    global_index = GLOBAL_SLOT_USE_NUM;
    for(index_ce_index=0; index_ce_index<GLOBAL_SLOT_USE_NUM; index_ce_index++)
    {
        if(lmp_self_device_data.global_slot_use_ce_index[index_ce_index] == use_ce_index)
        {
            global_index = index_ce_index;
            break;
        }
    }
//	RT_BT_LOG(GREEN, CCH_DBG_030, 0, 0);

    use_ce_index = lmp_self_device_data.global_slot_use_ce_index[global_index];
    use_acl_ce_index = lmp_self_device_data.global_slot_use_acl_ce_index[global_index];
    use_interval = lmp_self_device_data.global_slot_use_interval[global_index];
    use_slot_offset = lmp_self_device_data.global_slot_use_slot_offset[global_index];
    use_remote_slot = lmp_self_device_data.global_slot_use_remote_slot[global_index];
    use_slot_num = lmp_self_device_data.global_slot_use_slot_num[global_index];
    use_orig_interval = lmp_self_device_data.global_slot_interval[global_index];
//    RT_BT_LOG(GREEN, CCH_DBG_031, 7,use_acl_ce_index, use_ce_index, use_interval, use_slot_offset, use_remote_slot, use_slot_num,use_orig_interval);

    memset(slot_in_use, 0xFF, GLOBAL_SLOT_INTERVAL);

    for(index_ce_index=0; index_ce_index<GLOBAL_SLOT_USE_NUM; index_ce_index++)
    {
        use_ce_index = lmp_self_device_data.global_slot_use_ce_index[index_ce_index];
        if( (use_ce_index != 0xff) && ((use_ce_index !=ce_index) ||(use_ce_index !=is_esco_ce_index) ))
        {
            use_interval =lmp_self_device_data.global_slot_use_interval[index_ce_index];
            use_slot_offset =lmp_self_device_data.global_slot_use_slot_offset[index_ce_index];
            use_slot_num =lmp_self_device_data.global_slot_use_slot_num[index_ce_index];

            for(index_interval=use_slot_offset; index_interval<GLOBAL_SLOT_INTERVAL; index_interval+=use_interval)
            {
                temp_max = GLOBAL_SLOT_INTERVAL;
                if( (index_interval+use_slot_num) < temp_max)
                {
                    temp_max = (index_interval+use_slot_num);
                }
                for(index_slot_num=index_interval; index_slot_num<temp_max; index_slot_num++)
                {
                    slot_in_use[index_slot_num] = 0;
                }
            }
        }
    }
    use_interval = lmp_self_device_data.global_slot_use_interval[global_index];
    use_slot_offset = lmp_self_device_data.global_slot_use_slot_offset[global_index];
    use_remote_slot = lmp_self_device_data.global_slot_use_remote_slot[global_index];
    use_slot_num = lmp_self_device_data.global_slot_use_slot_num[global_index];
    use_orig_interval = lmp_self_device_data.global_slot_interval[global_index];

    if(ce_ptr->remote_dev_role == MASTER)
    {
        slot_val = lmp_cal_m2s_clock_offset(ce_index, use_orig_interval , use_interval, 1);
        use_slot_offset = (use_remote_slot + use_interval - slot_val )%use_interval;
        lmp_self_device_data.global_slot_use_slot_offset[global_index] = use_slot_offset;
    }

    for(index_interval=use_slot_offset; index_interval<GLOBAL_SLOT_INTERVAL; index_interval+=use_interval)
    {
        temp_max = GLOBAL_SLOT_INTERVAL;
        if( (index_interval+use_slot_num) < temp_max)
        {
            temp_max = (index_interval+use_slot_num);
        }
        for(index_slot_num=index_interval; index_slot_num<temp_max; index_slot_num++)
        {
            if(slot_in_use[index_slot_num] == 0)
            {
//                RT_BT_LOG(YELLOW, CCH_DBG_033, 1,index_slot_num);
                    slot_overlap = TRUE;
                    break;
            }
        }
    }

//	RT_BT_LOG(GREEN, CCH_DBG_030, 0, 0);

    use_ce_index = lmp_self_device_data.global_slot_use_ce_index[global_index];
    use_acl_ce_index = lmp_self_device_data.global_slot_use_acl_ce_index[global_index];
    use_interval = lmp_self_device_data.global_slot_use_interval[global_index];
    use_slot_offset = lmp_self_device_data.global_slot_use_slot_offset[global_index];
    use_remote_slot = lmp_self_device_data.global_slot_use_remote_slot[global_index];
    use_slot_num = lmp_self_device_data.global_slot_use_slot_num[global_index];
    use_orig_interval = lmp_self_device_data.global_slot_interval[global_index];
//	RT_BT_LOG(GREEN, CCH_DBG_031, 7,use_acl_ce_index, use_ce_index, use_interval, use_slot_offset, use_remote_slot, use_slot_num,use_orig_interval);

    RT_BT_LOG(WHITE, CCH_DBG_148, 2,slot_overlap, use_slot_offset);
    return slot_overlap;
}
#endif


/**
 * Calculates the max-slot for the connection, based on the packet-types
 * enabled by the host.
 *
 * \param ce_index ACL Connection entity index.
 *
 * \return max_slot The calculated max-slot for the connection.
 */
UCHAR lmp_calculate_max_slot_for_hci(UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY *ce_ptr;
    UINT16 pkt_type;
    UCHAR max_slot = 1;

    ce_ptr = &lmp_connection_entity[ce_index];
    pkt_type = ce_ptr->connection_type.packet_type;

    /* Warning. EDR packets are not considered for this. */
    if ( (pkt_type & DM5_DH5) != 0)
    {
        max_slot = 5;
    }
    else if ( (pkt_type & DM3_DH3) != 0)
    {
        max_slot = 3;
    }

    return max_slot;
}


#ifndef COMPILE_SINGLE_SLOT_PACKETS_ONLY
/**
 * Sends max_slot_pdu to a particular slave, after calculating the
 * acceptable value for it.
 *
 * \param ce_index ACL Connection entity index.
 *
 * \return None.
 */
void lmp_send_max_slot_pdu(UINT16 ce_index)
{
    UCHAR parameter_list[LMP_MAX_SLOT_LEN] = {0};
    UCHAR max_slot;
    UCHAR remote_feature;
    LMP_CONNECTION_ENTITY *ce_ptr;
    UCHAR lcl_feature_bit;
    UCHAR last_sent_max_slot;

    ce_ptr = &lmp_connection_entity[ce_index];

    lcl_feature_bit = lmp_feature_data.feat_page0[0];

    remote_feature = ce_ptr->feat_page0[0];

    parameter_list[0] = LMP_MAX_SLOT_OPCODE ;
    max_slot = (UCHAR) (lmp_calculate_max_slot(ce_index) );

    if (max_slot != 0xff)
    {
        parameter_list[2] = max_slot;

        if (max_slot == 5)
        {
            /* Check if local device supports, and remote device
               supports 5 slot packets. */
            if ( ((remote_feature & LMP_FIVE_SLOT_PACKET_FEATURE) == FALSE ) ||
                    ( ( lcl_feature_bit & LMP_FIVE_SLOT_PACKET_FEATURE) == FALSE ) )
            {
                return;
            }
        }
        else if (max_slot == 3)
        {
            /* Check if local device supports, and remote device
               supports 3 slot packets. */
            if ( ((remote_feature & LMP_THREE_SLOT_PACKET_FEATURE) == FALSE ) ||
                    ( ( lcl_feature_bit & LMP_THREE_SLOT_PACKET_FEATURE) == FALSE ) )
            {
                return;
            }
        }

        if(ce_ptr->last_max_slot_sent == max_slot)
        {
            /* Same max_slot need not be sent again. */
#if 0
            RT_BT_LOG(GRAY, LMP_UTILS_2205, 2,
                      ce_index, ce_ptr->last_max_slot_sent);
#endif
            return;
        }

        lmp_generate_pdu(ce_index, parameter_list, LMP_MAX_SLOT_LEN,
                         SELF_DEV_TID, LMP_NO_STATE_CHANGE);
    }
    last_sent_max_slot = ce_ptr->last_max_slot_sent;

    ce_ptr->last_max_slot_sent = max_slot;

    /* If new-max-slot is lesser than existing max-slot then update
       pkts allowed right away. */
    if( (last_sent_max_slot != LMP_MAX_SLOT_INVALID) && (max_slot < last_sent_max_slot) )
    {
        lc_update_pkts_allowed(ce_index);

        LMP_LOG_INFO(LOG_LEVEL_HIGH, LMP_UTILS_UPDATE_PKT_WHEN_MAX_SLOT, 3,
                     ce_index, max_slot, last_sent_max_slot);
    }
    return;
}

/**
 * Calculates the max-slot for the function based on the state of
 * the connection. This function also sends the max_slot_req_pdu to
 * remote device.
 *
 * \param ce_index ACL Connection entity index.
 * \param max_slot Max-slot to be sent for the connection. Of 0xff is
 *                 passed for this parameter, then appropriate max_slot
 *                 to be sent is calcuated based on the history.
 *
 * \return None.
 */
void lmp_send_max_slot_req_pdu(UINT16 ce_index, UCHAR max_slot)
{
    UCHAR parameter_list[LMP_MAX_SLOT_REQ_LEN] = {0};
    LMP_CONNECTION_ENTITY *ce_ptr;

    UCHAR remote_feature;
    ce_ptr = &lmp_connection_entity[ce_index];

    remote_feature = ce_ptr->feat_page0[0];

    parameter_list[0] = LMP_MAX_SLOT_REQ_OPCODE;

    if (max_slot == 0xff)
    {
        if (ce_ptr->last_max_slot_req_sent == LMP_LAST_MAX_SLOT_REQ_SENT_INVALID)
        {
            /* Check that local device and remote device supports 5 slot packets. */
            if ( ((remote_feature & LMP_FIVE_SLOT_PACKET_FEATURE) != FALSE) &&
                    ( ( lmp_feature_data.feat_page0[0] &
                        LMP_FIVE_SLOT_PACKET_FEATURE) != FALSE ) )
            {
                parameter_list[2] = LMP_MAX_SLOT_5;
                ce_ptr->last_max_slot_req_sent = LMP_LAST_MAX_SLOT_REQ_SENT_5_SLOT;
            }
            else if ( ((remote_feature & LMP_THREE_SLOT_PACKET_FEATURE) != FALSE) &&
                      ( ( lmp_feature_data.feat_page0[0] &
                          LMP_THREE_SLOT_PACKET_FEATURE) != FALSE ) )
            {
                parameter_list[2] = LMP_MAX_SLOT_3;
                ce_ptr->last_max_slot_req_sent = LMP_LAST_MAX_SLOT_REQ_SENT_3_SLOT;
            }
        }
        else if (ce_ptr->last_max_slot_req_sent == LMP_LAST_MAX_SLOT_REQ_SENT_5_SLOT)
        {
            if ( ((remote_feature & LMP_THREE_SLOT_PACKET_FEATURE) != FALSE) &&
                    ( ( lmp_feature_data.feat_page0[0] &
                        LMP_THREE_SLOT_PACKET_FEATURE) != FALSE ) )
            {
                parameter_list[2] = LMP_MAX_SLOT_3;
                ce_ptr->last_max_slot_req_sent = LMP_LAST_MAX_SLOT_REQ_SENT_3_SLOT;
            }
        }
        else
        {
            return;
        }
    }
    else
    {
        parameter_list[2] = max_slot;

        if(max_slot == 5)
        {
            ce_ptr->last_max_slot_req_sent = LMP_LAST_MAX_SLOT_REQ_SENT_5_SLOT;
        }
        else if (max_slot == 3)
        {
            ce_ptr->last_max_slot_req_sent = LMP_LAST_MAX_SLOT_REQ_SENT_3_SLOT;
        }
        else
        {
            /* This will never happen. */
        }
    }

    if( (max_slot == 5) || (max_slot == 3) )
    {
        lmp_generate_pdu(ce_index, parameter_list, LMP_MAX_SLOT_REQ_LEN,
                         SELF_DEV_TID, LMP_NO_STATE_CHANGE);
    }

    return;
}

/**
 * Calculates the max-slot for the function based on the state of
 * the device and the state of the connection.
 *
 * \param ce_index ACL Connection entity index.
 *
 * \return max_slot The calculated max-slot for the connection.
 */
void lmp_send_max_slot_pdu_to_all_devices(void)
{
    UINT32 i;

#ifdef ENABLE_LOGGER_LEVEL_2
    LMP_LOG_INFO(LOG_LEVEL_LOW, LMP_SEND_MAX_SLOT_PDU_TO_ALL_DEVICES_FUNCTION,0,0);
#endif

    for(i = 0; i < LMP_MAX_CE_DATABASE_ENTRIES; i++)
    {
        if ((lmp_connection_entity[i].ce_status != LMP_STANDBY) &&
            (lmp_connection_entity[i].setup_complete_status == CONN_COMPLETE_EVENT))
        {
            /* Connection is present, and LMP level connection
               procedure is complete. */
            lmp_send_max_slot_pdu( (UINT16) i);
            lc_update_pkts_allowed(i);
        }
    }

    return;
}

/**
 * Sends max_slot_req_pdu to all the connections. This function checks the
 * state of each of connection, and calculates the acceptable max_slot
 * for the connection, and sends it.
 *
 * \param None.
 *
 * \return None.
 */
void lmp_send_max_slot_req_pdu_to_all_devices(void)
{
    UINT32 i;

#ifdef ENABLE_LOGGER_LEVEL_2
    LMP_LOG_INFO(LOG_LEVEL_LOW, LMP_SEND_MAX_SLOT_REQ_PDU_TO_ALL_DEVICES_FUNCTION,0,0);
#endif

    for(i = 0; i < LMP_MAX_CONN_HANDLES; i++)
    {
        if(lmp_connection_entity[i].ce_status != LMP_STANDBY)
        {
            /* Connection is present. */
            lmp_send_max_slot_req_pdu( (UINT16) i, 0xff);
        }
    }

    return;
}


/**
 * Calculates the max-slot for the function based on the state of
 * the device and the state of the connection. If invalid-ce-index
 * is passed to this function, it will return the max-slot
 * considering the state of the device, the number of ACL connections,
 * and the number of synchronous connections.
 *
 * \param ce_index ACL Connection entity index.
 *
 * \return max_slot The calculated max-slot for the connection.
 */
UINT32 lmp_calculate_max_slot(UINT32 ce_index)
{
    LMP_CONNECTION_ENTITY *ce_ptr;
    UINT32 max_slot = 1;

    /* adjust the code flow to speed up the max slot decision - austin */
    do
    {
#ifdef ENABLE_SCO
        if ((lmp_self_device_data.total_no_of_sco_conn != 0) ||
            (lmp_self_device_data.adding_new_sco_conn != 0))
        {
            max_slot = 1;
            break;
        }
#endif

#ifdef COMPILE_ESCO
        if (lmp_self_device_data.number_of_esco_connections != 0)
        {
            max_slot = 1;
            break;
        }
#endif

#ifdef _DAPE_TEST_SEND_MAX_SLOT_BEFORE_ESCO_CREATED
        if (lmp_self_device_data.adding_new_esco_conn == TRUE)
        {
            max_slot = 1;
            break;
        }
#endif

        /* Check if the local device supports 5 slot packets. */
        if (lmp_feature_data.feat_page0[0] & LMP_FIVE_SLOT_PACKET_FEATURE)
        {
            /* Check if the remote device supports 5 slot packets. */
            if ( (lmp_connection_entity[ce_index].feat_page0[0] &
                    LMP_FIVE_SLOT_PACKET_FEATURE) != FALSE )
            {
                max_slot = 5;
                break;
            }
        }

        /* Check if the local device supports 3 slot packets. */
        if (lmp_feature_data.feat_page0[0] & LMP_THREE_SLOT_PACKET_FEATURE)
        {
            /* Check if the remote device supports 3 slot packets. */
            if ( (lmp_connection_entity[ce_index].feat_page0[0] &
                    LMP_THREE_SLOT_PACKET_FEATURE) != FALSE )
            {
                max_slot = 3;
            }
        }
    }
    while (0);

    if(ce_index != INVALID_CE_INDEX)
    {
        UINT32 sniff_int;

        ce_ptr = &lmp_connection_entity[ce_index];

#ifdef COMPILE_SNIFF_MODE
        if (max_slot != 1)
        {
            /* Check if any of the other connections are in sniff mode. */

#ifdef _DAPE_SNIFF_PKT_TEST
            UINT16 sniff_ce_index;
            UINT16 sniff_attempt = 0;
            sniff_int = lc_get_least_sniff_interval(&sniff_ce_index, &sniff_attempt);
#else
            sniff_int = lc_get_least_sniff_interval();
#endif

            if (sniff_int != 0)
            {
                if(sniff_int < LC_MIN_SNIFF_INTERVAL_ALLOW_3_SLOT_PKT)
                {
                    max_slot = 1;
                }
                else if (sniff_int < LC_MIN_SNIFF_INTERVAL_ALLOW_5_SLOT_PKT)
                {
                    max_slot = 3;
                }
            }
        }
#endif

#ifdef COMPILE_PARK_MODE
        if (ce_ptr->ce_status == LMP_PARK_MODE)
        {
            max_slot = 0xFF;
        }
#endif
    } /* if ce_index != INVALID_CE_INDEX */

    return max_slot;
}

/**
 * Decides whether the max-slot-req has to be updated for a connection.
 * This function considers the state of LMP and host enabled packets,
 * and decides to send max-slot-req. It also initiates the transaction.
 * the device and the state of the connection.
 *
 * \param ce_index ACL Connection entity index.
 *
 * \return None.
 */
void lmp_decide_and_update_max_slot_req(UINT16 ce_index)
{
    UINT32 hci_max_slot;
    UINT32 cur_max_slot;
    UINT32 lmp_calculated_max_slot;
    LMP_CONNECTION_ENTITY *ce_ptr;

    lmp_calculated_max_slot = lmp_calculate_max_slot(ce_index);

    ce_ptr = &lmp_connection_entity[ce_index];

    if (lmp_calculated_max_slot != 1)
    {
        /* Calculate the HCI max slot for this connection. */
        hci_max_slot = lmp_calculate_max_slot_for_hci(ce_index);

        cur_max_slot = ce_ptr->last_max_slot_req_sent;

        if (hci_max_slot > cur_max_slot)
        {
            /* Need to send max-slot-req. */

            if (hci_max_slot == 5)
            {
                ce_ptr->last_max_slot_req_sent =
                    LMP_LAST_MAX_SLOT_REQ_SENT_INVALID;
            }
            else if (hci_max_slot == 3)
            {
                ce_ptr->last_max_slot_req_sent =
                    LMP_LAST_MAX_SLOT_REQ_SENT_5_SLOT;
            }
            else
            {
                ce_ptr->last_max_slot_req_sent =
                    LMP_LAST_MAX_SLOT_REQ_SENT_3_SLOT;
            }
        }

        if (hci_max_slot == 1)
        {
            /* We need not send max-slot-req of 1. */
            ce_ptr->last_max_slot_req_sent =
                LMP_LAST_MAX_SLOT_REQ_SENT_3_SLOT;
        }
    }
    else
    {
        /* Max-slot-req should not be sent. */
        ce_ptr->last_max_slot_req_sent = LMP_LAST_MAX_SLOT_REQ_SENT_3_SLOT;
    }

    return;
}
#endif /* COMPILE_SINGLE_SLOT_PACKETS_ONLY */

#ifdef COMPILE_CQDDR
/**
 * Check whether CQDDR is enabled in local and remote feature bits.
 *
 * \param  ce_index ACL Connection entity index.
 *
 * \return TRUE if CQDDR is enabled on both devices, else FALSE.
 */
UCHAR lmp_is_cqddr_feature_enabled(UINT16 ce_index)
{
    UCHAR remote_feature, local_feature;

    remote_feature = lmp_connection_entity[ce_index].feat_page0[1];

    local_feature = lmp_feature_data.feat_page0[1];

    /* Check if local device and remote device support CQDDR. */
    if ( ((remote_feature & LMP_CHN_QLTY_FEATURE) == FALSE ) ||
            ( (local_feature  & LMP_CHN_QLTY_FEATURE) == FALSE ) )
    {
        return FALSE;
    }

    return TRUE;
}


/**
 * Sends auto_rate_pdu to a remote device.
 *
 * \param ce_index ACL Connection entity index.
 *
 * \return None.
 */
void lmp_send_auto_rate_pdu(UINT16 ce_index)
{
    UCHAR parameter_list[LMP_AUTO_RATE_LEN] = {0};



    /* Check if local device and remote device support CQDDR. */
    if ( lmp_is_cqddr_feature_enabled(ce_index) == FALSE )
    {
        return;
    }

    parameter_list[0] = LMP_AUTO_RATE_OPCODE;
    lmp_generate_pdu(ce_index, parameter_list, LMP_AUTO_RATE_LEN,
                     SELF_DEV_TID, LMP_NO_STATE_CHANGE);

    return;
}

/**
 * Obtain default best case packet preferances obtained without
 * channel assessment, for preferred_rate pdu.
 *
 * \param ce_index  ACL Connection entity index.
 *
 * \return None.
 */
UCHAR lmp_get_default_data_rate(UINT16 ce_index)
{
    UCHAR ptt_status, max_slot;
    UCHAR data_rate = 0x00;


    ptt_status = lmp_connection_entity[ce_index].ptt_status;

#ifndef COMPILE_SINGLE_SLOT_PACKETS_ONLY
    /* Max slot received or accepted.Both should be same or one invalid */
    max_slot = lmp_connection_entity[ce_index].last_accepted_max_slot;
    max_slot = (max_slot < lmp_connection_entity[ce_index].last_recd_max_slot)
               ? max_slot : lmp_connection_entity[ce_index].last_recd_max_slot;

    if (max_slot == LMP_MAX_SLOT_INVALID)
    {
        return 0x00;
    }

    /* Give packet size as current max_slot */
    switch(max_slot)
    {
        case LMP_MAX_SLOT_1:
            data_rate |= (1 << 1);
            break;
        case LMP_MAX_SLOT_3:
            data_rate |= (2 << 1);
            break;
        case LMP_MAX_SLOT_5:
            data_rate |= (3 << 1);
            break;
    }
#endif

    if (ptt_status == LMP_PTT_IDLE)
    {
        /* Do not use FEC */
        data_rate |= 1;
    }
    else if (ptt_status == LMP_PTT_ENABLED)
    {
        data_rate = data_rate << 1;

        if ( lmp_feature_data.feat_page0[3] & EDR_ACL_3MBPS_FEATURE)
        {
            data_rate |= 2;
        }
        else if ( lmp_feature_data.feat_page0[3] & EDR_ACL_2MBPS_FEATURE)
        {
            data_rate |= 1;
        }

        data_rate = data_rate << 3;
    }
    else
    {
        data_rate = 0x00;
        LMP_LOG_INFO(LOG_LEVEL_HIGH, NOT_GENERATING_PREFERRED_RATE_PARAM_AS_PTT_IS_BEING_SET_CLEARED,0,0);
    }


    return data_rate;
}

/**
 * Sends preferred_rate_pdu to a particular slave, using link quality
 * measurements and default values if not available.
 *
 * \param ce_index  ACL Connection entity index.
 *
 * \return None.
 */
void lmp_calculate_and_send_preferred_rate_pdu(UINT16 ce_index)
{
    UCHAR data_rate;
    if ( lmp_is_cqddr_feature_enabled(ce_index) == FALSE )
    {
        return;
    }

    /* Check if we have received an auto_rate PDU */
    if (lmp_connection_entity[ce_index].received_auto_rate_pdu == FALSE)
    {
        return;
    }

#if 0 /* COMPILE_CHANNEL_ASSESSMENT */
    if (link_quality_available)
    {
        data_rate = lmp_getlink_quality_based_data_rate(ce_index);
    }
    else
#endif
    {
        data_rate = lmp_get_default_data_rate(ce_index);
    }

    if (data_rate != 0x00)
    {
        /* send preferred rate pdu */
        UCHAR parameter_list[LMP_PREFERRED_RATE_LEN];
        parameter_list[0] = LMP_PREFERRED_RATE_OPCODE;
        parameter_list[2] = data_rate;
        lmp_generate_pdu(ce_index, parameter_list, LMP_PREFERRED_RATE_LEN,
                         SELF_DEV_TID, LMP_NO_STATE_CHANGE);
    }
}

#endif /* COMPILE_CQDDR */


/**
 * Handles the last ACL connection disconnection. Any LMP/HCI level cleanup
 * that will have to be done after all the connections are killed will be done
 * in this function.
 *
 * \param None.
 *
 * \return None.
 */
void lmp_handle_last_conn_disconnected(void)
{
    /* Reset all channel-assessment details. */
#ifdef COMPILE_CHANNEL_ASSESSMENT
    if (IS_USE_FOR_BQB)
    {
        afh_la_cycle_period = 10000;
    }
    else
    {
        afh_la_cycle_period = AFH_LA_CYCLE_PERIOD;
    }
#endif

    return;
}

#ifdef COMPILE_ROLE_SWITCH
/**
 * Sends the LMP_slot_offset PDU to remote device.
 *
 * \param ce_index ACL connection entity index.
 * \param tid Transaction ID.
 *
 * \return None.
 */
void lmp_send_slot_offset_pdu(UINT16 ce_index, LMP_TRAN_ID tid)
{
    UINT16 slot_offset;
    UCHAR parameter_list[LMP_SLOT_OFFSET_LEN];
    UCHAR am_addr;
    UCHAR piconet_id;

    DEF_CRITICAL_SECTION_STORAGE;

    BZ_ASSERT(lmp_connection_entity[ce_index].remote_dev_role == MASTER,
              "Only slave can send LMP_slot_offset PDU");

    parameter_list[0] = LMP_SLOT_OFFSET_OPCODE;

    am_addr = lmp_connection_entity[ce_index].am_addr;

    piconet_id = lmp_connection_entity[ce_index].phy_piconet_id;

    MINT_OS_ENTER_CRITICAL();
    BB_write_baseband_register(CONNECTOR_REGISTER,
                               ( (am_addr << 5) | (piconet_id << 11) ));

    slot_offset = LC_GET_PHASE_OFFSET_VALUE();
    MINT_OS_EXIT_CRITICAL();

    LMP_INF(SENDING_SLOT_OFFSET_TO_THE_REMOTE_DEVICE,1,slot_offset);
    parameter_list[2] = LSB(slot_offset);
    parameter_list[3] = MSB(slot_offset);

    memcpy(&parameter_list[4], otp_str_data.bt_bd_addr, LMP_BD_ADDR_SIZE);
    lmp_generate_pdu(ce_index, parameter_list, LMP_SLOT_OFFSET_LEN, tid,
                     LMP_NO_STATE_CHANGE);
}

/**
 * Handles role switch either during connection or after connection.
 *
 * \param ce_index ACL Connection entity index.
 * \param reason Failure reason.
 *
 * \return None.
 */
void lmp_handle_role_switch_failure(UINT16 ce_index, UCHAR reason)
{
    LMP_CONNECTION_ENTITY* ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    /* Role-switch procedure has failed, anyway let's see whether we had
     * already paused the encryption.
     */
    if (ce_ptr->is_enc_paused)
    {
        if (ce_ptr->auth_role == BZ_AUTH_ROLE_INITIATOR)
        {
            /* Being a initiator we have paused the encryption and hence we
             * need to resume it first. It doesn't matter whether we have used
             * the EPR or LEGACY procedure.
             */

            bz_auth_resume_encryption(ce_index);
        }
        if (ce_ptr->enc_proc == BZ_AUTH_ENCRYPTION_PROCEDURE_EPR)
        {
            /* EPR procedure has been used to pause the encryption and hence
             * we have to defer the generation of role-switch until the resume
             * encryption procedure completion. Authentication role doesn't
             * matter here.
             */
            ce_ptr->mss_completion_status = reason;
            return;
        }
    }

    switch (ce_ptr->ce_status)
    {
        case LMP_ROLE_SWITCHING:        /* After connection role switch */
            lmp_set_ce_status(ce_index, LMP_CONNECTED);
            break;

        case LMP_CONNECTION_ROLE_SWITCH: /* During connection role switch */
            lmp_accept_host_connection_request_pdu(ce_index);
            lmp_set_ce_status(ce_index, LMP_BB_HL_CONNECTED);
            break;

        default:
            break;
    }
#ifndef _ENABLE_COMPILE_NESTED_PAUSE_RESUME_
#ifdef COMPILE_NESTED_PAUSE_RESUME
    aclq_resume_am_addr(ce_ptr->am_addr,ce_ptr->phy_piconet_id, ACL_PAUSED_MSS);
#else /* COMPILE_NESTED_PAUSE_RESUME */
    aclq_resume_am_addr(ce_ptr->am_addr,ce_ptr->phy_piconet_id);
#endif /* COMPILE_NESTED_PAUSE_RESUME */
#endif
#ifdef _CCH_IOT_CSR_RS_
    ce_ptr->waiting_for_rs_several_times = 0;
#endif
    hci_generate_role_change_event(reason, ce_ptr->bd_addr,
                                   (UCHAR)(ce_ptr->remote_dev_role ^ 0x01));

    /* Resume data transfer. */
    {
        OS_SIGNAL signal_send ;

        signal_send.type = LC_RESUME_DATA_TRANSFER_SIGNAL;
        /* Resume data transfer for both the piconets */
        signal_send.param = (void*) SCA_PICONET_INVALID;

        if(OS_SEND_SIGNAL_TO_TASK(lc_tx_task_handle, signal_send) != BT_ERROR_OK)
        {
//            LMP_ERR(log_file,"OS send signal to task failed.");
        }
    }
#ifdef _ENABLE_COMPILE_NESTED_PAUSE_RESUME_
#ifdef COMPILE_NESTED_PAUSE_RESUME
    aclq_resume_am_addr(ce_ptr->am_addr,ce_ptr->phy_piconet_id, ACL_PAUSED_MSS);
#else /* COMPILE_NESTED_PAUSE_RESUME */
    aclq_resume_am_addr(ce_ptr->am_addr,ce_ptr->phy_piconet_id);
#endif /* COMPILE_NESTED_PAUSE_RESUME */
#else
    ce_ptr->pause_data_transfer = FALSE;
#endif
    return;
}
#endif /* COMPILE_ROLE_SWITCH */


/**
 * Sets the ce_status of the connection.
 *
 * \param ce_index The index of lmp_connection_entity of the connection.
 * \param new_ce_status The new state of the connection.
 *
 * \return None.
 */
void lmp_set_ce_status(UINT16 ce_index, UINT8 new_ce_status)
{
#ifdef SCNET_DEBUG
    RT_BT_LOG(GRAY, LMP_UTILS_2953, 3, ce_index,
              lmp_connection_entity[ce_index].ce_status, new_ce_status);
#endif

    lmp_connection_entity[ce_index].ce_status = new_ce_status;

    return;
}

/**
 * Checks whether Role switch is allowed for the connection in the current scatternet configuration.
 *
 * \param ce_index The index to lmp_connection_entity database.
 *
 * \return TRUE if the role switch is allowed, FALSE otherwise.
 */
UCHAR lmp_check_for_role_switch_in_scatternet(UINT16 ce_index)
{
    UCHAR new_role;
    UCHAR allowed_status = TRUE;
    UCHAR master_piconet_id = SCA_PICONET_INVALID;
    LMP_CONNECTION_ENTITY *ce_ptr;
    LC_CUR_SCATTERNET_STATE lcl_state;

    ce_ptr = &lmp_connection_entity[ce_index];

#ifdef _BRUCE_TEST_CHK_REMOTE_ROLE_SW
   if(ce_ptr->features[0]!=0x00)
#endif
   {
#ifdef _DAPE_TEST_CHK_REMOTE_ROLE_SW
        UCHAR feature;
        feature = ce_ptr->features[0][0];
        /* Check if the remote features support Role Switch */
        if((feature & LMP_SWITCH_FEATURE) == FALSE)
        {
            RT_BT_LOG(RED, DAPE_TEST_LOG535, 0,0);
            allowed_status = FALSE;
        }
#endif
    }


    new_role = ce_ptr->remote_dev_role;

    lcl_state = lc_current_scatternet_state;

    if(new_role == SLAVE)
    {
        /* Allow when there is only one connection in master piconet. */
        if ((lc_sca_manager.master_cnt > 0) && (lc_sca_manager.slave_cnt > 0))
        {
            UINT8 mpid = lc_sca_manager.master_id;
            if (lmp_self_device_data.lc_no_of_connections[mpid] != 1)
            {
                RT_BT_LOG(GRAY, LMP_UTILS_0380_1, 1, mpid);
                allowed_status = FALSE;
            }
        }
    }
    else if (new_role == MASTER)
    {
        /* Check for availability of am_addr. */
        if ((lc_sca_manager.master_cnt > 0) && (lc_sca_manager.slave_cnt > 0))
        {
            master_piconet_id = lc_get_master_piconet_id();

            if(lmp_self_device_data.lc_no_of_connections[master_piconet_id] ==
                    LMP_MAX_ACTIVE_CONN_ENTITIES)
            {
                RT_BT_LOG(GRAY, LMP_UTILS_3016, 0, 0);
                allowed_status = FALSE;
            }
        }
    }

#ifdef ENABLE_SCO
    if (lmp_self_device_data.total_no_of_sco_conn != 0)
    {
        if (lmp_self_device_data.sco_pkt_type == HV3)
        {
            if(lmp_self_device_data.total_no_of_sco_conn != 1)
            {
                /* If there is a SCO connection on another slave, it
                 * can be only HV3. Only one SCO connection is allowed
                 * in this case. */
                allowed_status = FALSE;

                RT_BT_LOG(GRAY, LMP_UTILS_3034, 0, 0);
            }
        }
        else
        {
            /* If there is a HV2 connection with another device. Reject. */
            allowed_status = FALSE;
            RT_BT_LOG(GRAY, LMP_UTILS_3041, 0, 0);
        }
    }
#endif

    /* Check if scatternet is enabled in config. */
    if (!IS_SUPPORT_SCATTERNET)
    {
        if(lmp_self_device_data.number_of_hlc > 1)
        {
            allowed_status = FALSE;
            RT_BT_LOG(GRAY, LMP_UTILS_3057, 0, 0);
        }
    }

    return allowed_status;
}


/**
 * Returns the master piconet_id of the device.
 *
 * \param None.
 *
 * \return SCA_PICONET_FIRST for the first piconet,
 *         SCA_PICONET_SECOND for the second piconet and
 *         SCA_PICONET_INVALID if not connected as master.
 */
UCHAR lc_get_master_piconet_id(void)
{
    UCHAR piconet_id = SCA_PICONET_INVALID;

    if (lc_sca_manager.master_cnt == 0)
    {
        if (lc_sca_manager.slave_cnt == 0)
        {
            /* SCA_IDLE */
            piconet_id = SCA_PICONET_FIRST;
        }
    }
    else
    {
        piconet_id = lc_sca_manager.master_id;
    }
    return piconet_id;
}


#ifdef COMPILE_ESCO
/**
 * Checks if eSCO connection is allowed in the current device configuration.
 *
 * \param None.
 *
 * \return HCI_COMMAND_SUCCEEDED on success,
 *         approriate error otherwise.
 */
UCHAR lmp_check_for_allowing_new_esco_connection(void)
{
    UCHAR ret_error = HCI_COMMAND_SUCCEEDED;

    /* Do not allow ESCO connection if connected in more than one piconet. */
 #ifndef _CCH_ESCO_SCA_
    if(lc_get_no_of_piconets_connected() != 1)
    {
        ret_error = HOST_REJECTED_LIMITED_RESOURCES_ERROR;
    }
#endif
    return ret_error;
}
#endif /* COMPILE_ESCO */

#ifdef COMPILE_PARK_MODE
UINT16 lmp_get_park_ce_index(void)
{
    UINT16 lcl_temp;
    UINT16 lcl_park_ce_index = INVALID_CE_INDEX;
    LMP_CONNECTION_ENTITY *ce_ptr;

    for(lcl_temp = 0; lcl_temp < LMP_MAX_CE_DATABASE_ENTRIES; lcl_temp++)
    {
        ce_ptr = &lmp_connection_entity[lcl_temp];

        if( (ce_ptr->entity_status == ASSIGNED) &&
                (ce_ptr->ce_status == LMP_PARK_MODE) )
        {
            lcl_park_ce_index = lcl_temp;
            break;
        }
    }

    return lcl_park_ce_index;
}
#endif

#ifdef RTL8723A_B_CUT
extern TimerHandle_t dbg_tid_timer;
#ifdef _IS_ASIC_
extern TIMER_ID g_rtl8723_btrf_thermal_value_timer;
#endif

void lmp_stop_regular_sw_timers(void)
{
#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8821_RCP_

    if (rcp_lmp_stop_regular_sw_timers != NULL)
    {
        rcp_lmp_stop_regular_sw_timers();
    }
#endif
#endif


    /* stop afh corresponding sw timers */
    OS_STOP_TIMER(la_period_timer, 0);
    OS_STOP_TIMER(la_classify_timer, 0);

    if (la_mailbox_timer != NULL)
    {
        OS_DELETE_TIMER(&la_mailbox_timer);
    }

    /* stop background 20sec timer */
    OS_STOP_TIMER(dbg_tid_timer, 0);

#ifdef _IS_ASIC_
    if (otp_str_data.EFuse_ThermalUpdateInterval != 0)
    {
        OS_STOP_TIMER(g_rtl8723_btrf_thermal_value_timer, 0);
    }
#endif

#ifdef PTA_EXTENSION
    if (IS_USE_PTA_METER)
    {
        OS_STOP_TIMER(hTimerIDPtaHciMeter, 0);
        OS_STOP_TIMER(hTimerIDPta50msMeter, 0);
    }
#endif
#ifdef _NEW_MODEM_PSD_SCAN_
    if (g_rtk_afh_modem_psd_internal_enable &&
            otp_str_data.rtk_afh_bt_psd_enable)
    {
        OS_STOP_TIMER(scan_period_timer, 0);
        OS_STOP_TIMER(scan_tdm_timer, 0);
        OS_STOP_TIMER(afh_map_gen_timer,0);
    }
#endif
}

void lmp_start_regular_sw_timers(void)
{

#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8821_RCP_

    if (rcp_lmp_start_regular_sw_timers != NULL)
    {
        rcp_lmp_start_regular_sw_timers();
    }
#endif
#endif

    /* start afh corresponding sw timers */
    OS_START_TIMER(la_period_timer, afh_la_cycle_period);

    /* start background 20sec timer */
    OS_START_TIMER(dbg_tid_timer, 20000);

#ifdef _IS_ASIC_
    if (otp_str_data.EFuse_ThermalUpdateInterval != 0)
    {
        OS_START_TIMER(g_rtl8723_btrf_thermal_value_timer,
                       otp_str_data.EFuse_ThermalUpdateInterval * 1000);
    }
#endif

#ifdef PTA_EXTENSION
    if (IS_USE_PTA_METER)
    {
        OS_START_TIMER(hTimerIDPtaHciMeter, 2000);
        OS_START_TIMER(hTimerIDPta50msMeter, 50);
    }
#endif
}
#endif

