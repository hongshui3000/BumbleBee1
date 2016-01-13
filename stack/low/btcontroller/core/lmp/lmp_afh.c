/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  AFH LMP Layer implementation. All AFH related functions and variables are
 *  defined in this file (except Channel Assessment Algorithm related
 *  entities).
 */


/********************************* Logger *************************/
enum { __FILE_NUM__= 49 };
/********************************* Logger *************************/

/* ========================= Include File Section ========================= */
#include "lmp_internal.h"
#include "lmp_pdu_q.h"
#include "btc_defines.h"
#include "vendor.h"
#ifdef COMPILE_CHANNEL_ASSESSMENT
#include "lmp_ch_assessment.h"
#endif
#include "bt_fw_acl_q.h"
#include "mem.h"
#include "mailbox.h"
#include "otp.h"

#ifdef _ROM_CODE_PATCHED_
#ifdef _DAPE_8723_RCP_
PF_ROM_CODE_PATCH_FUNC rcp_lmp_handle_afh_instant_when_master_func = NULL;
#endif
#endif

#ifdef _STOP_AFH_TIMER_WHILE_ROLE_SWITCH
extern UINT8 g_stop_afh_timer;
#endif

/* ==================== Macro declaration Section ===================== */
#define LMP_AFH_MSS_DEFERRED_DURATION  50

/* ==================== Structure declaration Section ===================== */

/* ===================== Variable Declaration Section ===================== */

/* ================== Static Function Prototypes Section ================== */

/* ===================== Function Definition Section ====================== */
#ifdef COMPILE_AFH_HOP_KERNEL
/**
 * Handles the AFH instant expiration.
 *
 * \param ce_index ACL Connection entity index.
 *
 * \return None.
 */
void lmp_delete_afh_timer(UINT16 ce_index)
{
    OS_SIGNAL sig_send;
    LMP_CONNECTION_ENTITY *ce_ptr;
    DEF_CRITICAL_SECTION_STORAGE;

    ce_ptr = &lmp_connection_entity[ce_index];

    if (ce_ptr->afh_instant_timer_handle == NULL)
    {
        return;
    }

    MINT_OS_ENTER_CRITICAL();
    sig_send.type = LMP_DELETE_TIMER_HANDLE_SIGNAL;
    sig_send.param =  (OS_ADDRESS)(ce_ptr->afh_instant_timer_handle);
    OS_SEND_SIGNAL_TO_TASK(lmp_task_handle,sig_send);
    ce_ptr->afh_instant_timer_handle = NULL;
    MINT_OS_EXIT_CRITICAL();

    return;
}

/**
 * Sends the latest channel information to the remote device. If the
 * current role is master, lmp-set-afh PDU is sent, and the map in
 * the slave is updated. If the current role is slave, then
 * lmp-channel-classification report is sent.
 *
 * \param map The AFH channel map.
 * \param ce_index Index to LMP connection entity.
 * \param afh_mode New AFH mode.
 *
 * \return None.
 */
void lmp_start_afh_map_updation(UCHAR *map, UINT16 ce_index, UCHAR afh_mode)
{
    INT32 result = -1;
    LMP_CONNECTION_ENTITY *ce_ptr;
#ifdef COMPILE_CHECK_LOCAL_FEATURES
    UCHAR local_feature;
#endif

    ce_ptr = &lmp_connection_entity[ce_index];

#ifdef COMPILE_CHECK_LOCAL_FEATURES
    /* Check whether local device suports AFH. */
	local_feature = lmp_feature_data.feat_page0[5];
#ifdef _DAPE_TEST_DEFAULT_MATER_AFH_POWER_CTRL_OFF
	local_feature &= (~AFH_CAPABLE_MASTER);
#endif

	if ((local_feature & AFH_CAPABLE_MASTER) == FALSE)
    {
        return;
    }
#endif

    /* Do not process further if AFH is in disabled state. */
    if (afh_mode == AFH_DISABLE)
    {
        return;
    }

    /* If self device is slave, send channel-classification-pdu */
    if (ce_ptr->remote_dev_role == MASTER)
    {
        lmp_send_ch_cl_pdu(&map[0], ce_index);

        return;
    }

#ifdef COMPILE_ROLE_SWITCH
    /* Check if MSS is in progress, only on the same connection. */
    if ((lmp_mss_state != LMP_MSS_INIT) &&
        (lmp_role_switch_data.ce_index == ce_index))
    {
		RT_BT_LOG(GRAY, LMP_AFH_149, 1, ce_index);
        return;
    }
#endif /* COMPILE_ROLE_SWITCH */

    if ((afh_mode != AFH_DISABLE) &&
        (ce_ptr->afh_instant_timer_handle != NULL))
    {
		RT_BT_LOG(GRAY, LMP_AFH_158, 0, 0);
        return;
    }

    if (afh_mode == ce_ptr->afh_mode)
    {
        if (ce_ptr->waiting_for_set_afh_pdu_ack == TRUE)
        {
            RT_BT_LOG(RED, CCH_DBG_050, 1,ce_index);
            return;
        }
        result = memcmp(&ce_ptr->afh_map[0], &map[0], LMP_AFH_MAP_SIZE);
    }

    if(result != 0)
    {
        memcpy(&ce_ptr->afh_map[0], &map[0],LMP_AFH_MAP_SIZE);
        lmp_update_map(afh_mode, ce_index, FALSE);
    }
    else
    {
        RT_BT_LOG(GRAY, CALCULATED_AND_CURRENT_AFH_MAPS_ARE_SAME_NO_PDU_IS_SENT, 0,0);
#ifdef ENABLE_LOGGER_LEVEL_2
		LMP_LOG_INFO(LOG_LEVEL_LOW, CALCULATED_AND_CURRENT_AFH_MAPS_ARE_SAME_NO_PDU_IS_SENT, 0,0);
#endif
    }

    return;
}


void lmp_update_map_bottom_half(UCHAR afh_mode, UINT16 ce_index)
{
    if (lmp_calculate_afh_instant(ce_index) == API_FAILURE)
    {
        return;
    }

    lmp_send_set_afh_pdu(ce_index, afh_mode);
}

void lmp_update_map_bottom_half_task(void *afh_mode, uint32_t ce_index)
{
    lmp_update_map_bottom_half((UCHAR)(UINT32) afh_mode, (UINT16) ce_index);
}

/**
 * Initiate the AFH updating. It checks data transfer progress, changes
 * connection entity status according to afh, sends lmp packets to all
 * required slaves if data transfer is not in progress. This function is
 * called only as MASTER.
 *
 * \param afh_mode AFH_ENABLE or AFH_DISABLE.
 * \param ce_index ACL Connection entity index.
 * \param force_flag Whether to override all standard checks
 *                   and force updating map.
 *
 * \note This function should be called only when self-device is MASTER.
 *
 * \return None.
 */
void lmp_update_map(UCHAR afh_mode, UINT16 ce_index, UCHAR force_flag)
{
    UCHAR remote_feature_slave;
    LMP_CONNECTION_ENTITY *ce_ptr;

#ifdef COMPILE_CHECK_LOCAL_FEATURES
    UCHAR local_feature;
#endif

    ce_ptr = &lmp_connection_entity[ce_index];

#ifdef COMPILE_CHECK_LOCAL_FEATURES
    /* Check whether local device suports AFH. */
	local_feature = lmp_feature_data.feat_page0[5];

	if ((local_feature & AFH_CAPABLE_MASTER) == FALSE)
    {
        return;
    }
#endif

    if (afh_mode != AFH_DISABLE)
    {
        if(ce_ptr->afh_instant_timer_handle != NULL)
        {
			LMP_LOG_INFO(LOG_LEVEL_HIGH, AFH_TIMER_ALREADY_RUNNING_NOT_RESTARTED, 0,0);
            return;
        }
    }

    /* Check that the last lmp-set-afh was sent at least
       LMP_MIN_DISTANCE_BETWEEN_SET_AFH_IN_SECONDS */
    if (ce_ptr->last_set_afh_sent_clk != 0xFFFFFFFF)
    {
        UINT32 current_clock;
        current_clock = BB_read_native_clock();

        if( (current_clock - ce_ptr->last_set_afh_sent_clk) <
          (LMP_MIN_DISTANCE_BETWEEN_SET_AFH_IN_SECONDS *
            LMP_CH_ASS_1_SEC_CLK0_TICKS) )
        {
            /* Note: Clock wrap around is not considered here, as
               there is nothing seriously lost, if IUT violates this 10
               seconds interval.*/
			RT_BT_LOG(GRAY, LMP_AFH_244, 0, 0);
            if(force_flag == FALSE)
            {
                return;
            }
        }
    }

    remote_feature_slave = ce_ptr->feat_page0[4];

    if (remote_feature_slave & AFH_CAPABLE_SLAVE)
    {
        UINT8 i;

#ifdef _CCH_CHK_LMP_RESOURCE_
        /* move flag check here to avoid the inconsistency between afh timer
           and afh map generator - austin */

        if (ce_ptr->waiting_for_set_afh_pdu_ack == TRUE)
        {
            RT_BT_LOG(RED, CCH_DBG_050, 1,ce_index);
            return;
        }
#endif

#ifdef _BRUCE_TEST_PLC_FIX_AFH
        const UCHAR bruce_afh_map[10] = {
        0xF0,0x0D,0xFF,0x00,0x00,0xD0,0xF3,0x00,0x00,0xFF};
        memcpy(&ce_ptr->afh_map[0], &bruce_afh_map[0], LMP_AFH_MAP_SIZE);
#endif

        if ((afh_mode == AFH_ENABLE) && (ce_ptr->afh_mode == AFH_ENABLE))
        {
            for (i = 0; i < LMP_AFH_MAP_SIZE; i++)
            {
                if (ce_ptr->old_afh_map[i] ^ ce_ptr->afh_map[i])
                {
                    break;
                }
            }

            if (i == LMP_AFH_MAP_SIZE)
            {
                /* all channels are still the same.
                   do not need to update channel map via lmp - austin */
                return;
            }
        }
#ifdef _SUPPORT_VER_4_1_
#ifdef _SUPPORT_PCA_ADJUST
        if (IS_BT41)
        {
            if (bt_pca_manager.pca_updating)
            {
                RT_BT_LOG(RED, DAPE_TEST_LOG581, 1,ce_index);
                return;
            }
        }
#endif
#endif
#ifdef _NO_SEND_AFH_POWER_CTRL_WHILE_ROLE_SW
#ifdef _ENABLE_COMPILE_NESTED_PAUSE_RESUME_
        UCHAR lut_index;
        lut_index = lc_get_lut_index_from_phy_piconet_id(ce_ptr->am_addr, ce_ptr->phy_piconet_id);
        if(paused_lut_index[lut_index])
        {
             RT_BT_LOG(RED, DAPE_TEST_LOG586, 3, ce_index, LMP_SET_AFH_OPCODE, 0);
             return;
        }
#else
        if(ce_ptr->pause_data_transfer)
        {
             RT_BT_LOG(RED, DAPE_TEST_LOG586, 3, ce_index, LMP_SET_AFH_OPCODE, 0);
             return;
        }
#endif
#endif

        if (IN_ISR())
        {
            BaseType_t high_pri_task_woken = pdFALSE;
            xTimerPendFunctionCallFromISR(lmp_update_map_bottom_half_task,
                    (void *)(UINT32) afh_mode, ce_index, &high_pri_task_woken);
            portYIELD_FROM_ISR(high_pri_task_woken);
        }
        else
        {
            lmp_update_map_bottom_half(afh_mode, ce_index);
        }
    }

    return;
}

/**
 * Handles the LMP_set_AFH pdu. This PDU is always received as slave.
 *
 * \param lmp_pdu_ptr Pointer to LMP PDU packet.
 * \param ce_index Index to the connection entity table.
 *
 * \return None.
 */
void lmp_handle_set_afh_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index)
{
	UINT32 afh_instant;
	UINT32 native_clock;
	UINT32 afh_instant_time_val;
	UCHAR local_slave_feature;
	UCHAR remote_master_feature;
	UCHAR afh_mode;
	LMP_CONNECTION_ENTITY *ce_ptr;
	UCHAR instant_status;
	UCHAR phy_piconet_id;

    //RT_BT_LOG(WHITE, LMP_AFH_302, 0, 0);

	ce_ptr = &lmp_connection_entity[ce_index];

	phy_piconet_id = ce_ptr->phy_piconet_id;

	local_slave_feature = lmp_feature_data.feat_page0[4];

	remote_master_feature = ce_ptr->feat_page0[5];

	if ((ce_ptr->remote_dev_role == MASTER) &&
		((local_slave_feature & AFH_CAPABLE_SLAVE) &&
		(remote_master_feature & AFH_CAPABLE_MASTER))
#ifdef _SUPPORT_VER_4_1_
#ifdef _SUPPORT_PCA_ADJUST
        && ((!IS_BT41)||(!bt_pca_manager.pca_updating))
#endif
#endif
    )
	{
        afh_instant = lmp_pdu_ptr->payload_content[1] |
                      (lmp_pdu_ptr->payload_content[2] << 8) |
                      (lmp_pdu_ptr->payload_content[3] << 16) |
                      (lmp_pdu_ptr->payload_content[4] << 24);

		lc_get_clock_in_scatternet(&native_clock, phy_piconet_id);

		afh_mode = lmp_pdu_ptr->payload_content[5];
        if(afh_mode == AFH_ENABLE)
        {
           lmp_pdu_ptr->payload_content[5 + LMP_AFH_MAP_SIZE] = (UCHAR)
                    (lmp_pdu_ptr->payload_content[5+LMP_AFH_MAP_SIZE] & 0x7F);

            afh_instant = (UINT32) (afh_instant & (~0x1));

            /* Check and start Ch as. timer. */
            if (OS_IS_TIMER_RUNNING(la_period_timer) == FALSE)
            {
                OS_START_TIMER(la_period_timer, afh_la_cycle_period);

#ifdef ENABLE_LOGGER_LEVEL_2
				LMP_LOG_INFO(LOG_LEVEL_LOW, CH_AS_TIMER_STARTED_FOR_1, 1, afh_la_cycle_period);
#endif
            }
        }
        else
        {
#ifdef COMPILE_CHANNEL_ASSESSMENT
            ce_ptr->afh_ch_cl_reporting_mode = AFH_DISABLE;
#endif
        }

		ce_ptr->afh_mode = afh_mode;
        memcpy(&ce_ptr->afh_map[0], &lmp_pdu_ptr->payload_content[6],
                LMP_AFH_MAP_SIZE);

        instant_status = lc_check_for_clock_wrap_around(
                            native_clock, (afh_instant << 1));

        if(instant_status == BT_CLOCK_MORE_THAN_12_HRS_AWAY)
        {
			LMP_LOG_INFO(LOG_LEVEL_HIGH, AFH_INST_12_HOURS_AWAY_ASSUMING_INSTANT_PASSED,0,0);

        /* cch: fix receive AFH Disable LMP PDU */
	    	if(afh_mode == AFH_ENABLE)  // cch: add for afh disable lmp
    		{
    			BB_write_afh_map(ce_ptr->am_addr, ce_ptr->phy_piconet_id,
    				ce_ptr->afh_map);

    			RT_BT_LOG(GRAY, LMP_AFH_371, 3, ce_index,
    				ce_ptr->am_addr, ce_ptr->phy_piconet_id);
        /* cch: fix receive AFH Disable LMP PDU */
    		}
    		else
    		{
    			BB_disable_afh(ce_ptr->am_addr, ce_ptr->phy_piconet_id);
    		}
            return;
        }
        else if (instant_status == BT_CLOCK_CLK_WRAP_AROUND_CASE)
        {
            /* Calculate absolute difference. */
            afh_instant_time_val =
                (((afh_instant << 1) | (BT_CLOCK_27_BITS + 1)) - native_clock);

			LMP_LOG_INFO(LOG_LEVEL_HIGH, AFH_INST_CLK_WARP_AROUND_CASE,0,0);
        }
        else
        {
            afh_instant_time_val = ((afh_instant << 1)  - native_clock);
        }

        afh_instant_time_val =
            (UINT32) (SLOT_VAL_TO_TIMER_VAL(afh_instant_time_val >> 1));

        lmp_delete_afh_timer(ce_index);

        OS_CREATE_TIMER(ONESHOT_TIMER, &ce_ptr->afh_instant_timer_handle,
                lmp_handle_afh_instant_timer, (void *)((UINT32)ce_index), 0);

        OS_START_TIMER(ce_ptr->afh_instant_timer_handle,
                    afh_instant_time_val);
    }
    else
    {
         lmp_send_lmp_not_accepted(ce_index,LMP_SET_AFH_OPCODE,
                    (UCHAR)(lmp_pdu_ptr->payload_content[0] & 0x01),
                                             PDU_NOT_ALLOWED_ERROR);
    }

    return;
}

/**
 * Handles the AFH instant timer expiry both as master and slave. Here
 * Firmware updates new parameters of AFH map to baseband and LC module and
 * apply recovery mechanism as master, if required.
 *
 * \param timer_handle Timer handle of this timer.
 * \param arg The user data passed to this timer.
 *
 * \return None.
 */
void lmp_handle_afh_instant_timer(TimerHandle_t timer_handle)
{
    UCHAR remote_slave_features;
    UCHAR count;
    UCHAR am_addr;
    UCHAR temp_afh_map[LMP_AFH_MAP_SIZE];
    UCHAR state;

#ifdef COMPILE_AFH_HOP_KERNEL
#ifdef COMPILE_PARK_MODE
#ifdef AFH_PARK
    OS_SIGNAL signal;
#endif
#endif
#endif

    LMP_CONNECTION_ENTITY *ce_ptr;
    UINT16 ce_index = (UINT16)((UINT32) pvTimerGetTimerID(timer_handle));

    ce_ptr = &lmp_connection_entity[ce_index];
    am_addr = ce_ptr->am_addr;

#ifdef _BRUCE_TEST_PLC_FIX_AFH
    const UCHAR bruce_afh_map[10] = {
    0xF0,0x0D,0xFF,0x00,0x00,0xD0,0xF3,0x00,0x00,0xFF};
    memcpy(&ce_ptr->afh_map[0], &bruce_afh_map[0], LMP_AFH_MAP_SIZE);
#endif

    if(timer_handle != ce_ptr->afh_instant_timer_handle)
    {
		LC_LOG_ERROR(LOG_LEVEL_HIGH,TIMER_HANDLE_IS_INVALID_REQUIRED,1,
			ce_ptr->afh_instant_timer_handle);
        return;
    }
#ifdef _STOP_AFH_TIMER_WHILE_ROLE_SWITCH
    if ((lmp_mss_state != LMP_MSS_INIT) || g_stop_afh_timer)
#else
	if (lmp_mss_state != LMP_MSS_INIT)
#endif
    {
        /* Restart the timer for 50ms, and return. */
        if(OS_START_TIMER(timer_handle, LMP_AFH_MSS_DEFERRED_DURATION) !=
                                                                BT_ERROR_OK)
        {
			AFH_ERR(INSTANT_TIMER_START_FAILED,0,0);
        }

        return;
    }

    /* Check and start Ch as. timer. */
    if (OS_IS_TIMER_RUNNING(la_period_timer) == FALSE)
    {
        OS_START_TIMER(la_period_timer, afh_la_cycle_period);

#ifdef ENABLE_LOGGER_LEVEL_2
		LMP_LOG_INFO(LOG_LEVEL_LOW, CH_AS_TIMER_STARTED_FOR_2, 1, afh_la_cycle_period);
#endif

    }

    if(ce_ptr->need_to_send_ch_cls_req == TRUE)
    {
        /* If sending of channel classification request is pending
           then send it. this will happen during connection as master */
        ce_ptr->need_to_send_ch_cls_req = FALSE;
		lmp_send_ch_cl_req_pdu(AFH_ENABLE, ce_index);
    }

    lmp_delete_afh_timer(ce_index);

    if(ce_ptr->afh_mode == AFH_DISABLE)
    {
		BB_disable_afh(am_addr, ce_ptr->phy_piconet_id);

        state = 0;
    }
    else if(ce_ptr->remote_dev_role == MASTER)
    {
#ifdef TMP_PATCH_AFH_ONLY_ONE_CHANNEL_WHEN_SLV
        {
            UINT16 reg_value;
            reg_value = BB_read_baseband_register(SCO_PACKET_TYPE_REGISTER);
            reg_value |= TEST_MODE_BASEBAND_HOP_1;
            BB_write_baseband_register(SCO_PACKET_TYPE_REGISTER, reg_value);

            UINT16 ch_num;
            ch_num = 0;
            BB_write_baseband_register(0xC4, (ch_num << 8) | ch_num);
            RT_BT_LOG(YELLOW, DAPE_TEST_LOG293, 1, 8989);
        }
#else
		BB_write_afh_map(am_addr, ce_ptr->phy_piconet_id, ce_ptr->afh_map);
#endif

#if 0
		RT_BT_LOG(GRAY, LMP_AFH_511, 3, ce_index,
			ce_ptr->am_addr, ce_ptr->phy_piconet_id);
#endif
        state = 1;
    }
    else /* ce_ptr->... */
    {
       /*
        * If ack is not received then start recovery mechanism
        */
        remote_slave_features = ce_ptr->feat_page0[4];

        /* Check for ack received */
        if( (remote_slave_features & AFH_CAPABLE_SLAVE) &&
                        (ce_ptr->afh_ce_status != LMP_AFH_UPDATING))
        {
            BB_write_afh_map(am_addr, ce_ptr->phy_piconet_id, ce_ptr->afh_map);
#if 0
			RT_BT_LOG(GRAY, LMP_AFH_537, 3, ce_index,
				ce_ptr->am_addr, ce_ptr->phy_piconet_id);
#endif
            state = 2;
        }
        else if( (ce_ptr->afh_ce_status == LMP_AFH_UPDATING) &&
                     (ce_ptr->waiting_for_set_afh_pdu_ack == TRUE))
        {

#ifdef ENABLE_LOGGER_LEVEL_2
			LC_LOG_ERROR(LOG_LEVEL_LOW,AFH_RECOVERY,0,0);
#endif

            for(count = 0; count < LMP_AFH_MAP_SIZE; count++)
            {
                temp_afh_map[count] =
                	(UCHAR)(ce_ptr->afh_map[count] |
                        ce_ptr->old_afh_map[count]);
                temp_afh_map[count] =
                        (UCHAR)(temp_afh_map[count] |
                        ce_ptr->last_acked_afh_map[count]);
            }
            BB_write_afh_map(am_addr, ce_ptr->phy_piconet_id, temp_afh_map);
#if 0
            RT_BT_LOG(GRAY, LMP_AFH_566, 3, ce_index,
                ce_ptr->am_addr, ce_ptr->phy_piconet_id);
#endif
            state = 3;
        }
        else
        {
#ifdef ENABLE_LOGGER_LEVEL_2
			LC_LOG_ERROR(LOG_LEVEL_LOW, NOT_CHANGING_AFH_MODE,0,0);
#endif
            state = 4;
        }
    }

#ifdef _ROM_CODE_PATCHED_
#ifdef _DAPE_8723_RCP_
    if (rcp_lmp_handle_afh_instant_when_master_func != NULL)
    {
        rcp_lmp_handle_afh_instant_when_master_func((void *)&ce_index);
    }
#endif
#endif

    UINT32 cur_clock;
    lc_get_clock_in_scatternet(&cur_clock, ce_ptr->phy_piconet_id);
    RT_BT_LOG(WHITE, LMP_AFH_962, 16, cur_clock,
        ce_index, ce_ptr->remote_dev_role, ce_ptr->afh_mode,
        ce_ptr->last_set_afh_sent_clk,
        ce_ptr->afh_map[0], ce_ptr->afh_map[1], ce_ptr->afh_map[2],
        ce_ptr->afh_map[3], ce_ptr->afh_map[4], ce_ptr->afh_map[5],
        ce_ptr->afh_map[6], ce_ptr->afh_map[7], ce_ptr->afh_map[8],
        ce_ptr->afh_map[9], state);

#ifdef COMPILE_PARK_MODE
#ifdef AFH_PARK
    if(ce_ptr->remote_dev_role == SLAVE)
    {
        /* Check if park is pending. */
        if (ce_ptr->park_pending_for_afh == TRUE)
        {
            /* Post signal to hci-command task to resume the park procedure. */
            signal.type = HCI_CMD_RECD_SIGNAL;
            signal.length = 0x0;
            signal.param = NULL;
            UINT32 context = (HCI_PARK_MODE_OPCODE << 16) | ce_index;
            signal.ext_param = (OS_ADDRESS)context;

            /* Send the Signal to HCI Task to process the command Pkt */
            OS_SEND_SIGNAL_TO_TASK(
                            rx_table[HCI_CMD_HANDLER_TASK].task_handle,signal);
        }
        else
        {
            if (ce_ptr->rem_park_pending_for_afh == TRUE)
            {
                /* Post signal to LMP task to resume the park procedure. */
                signal.type = LMP_PDU_RECD_SIGNAL;
				signal.param = ce_ptr->park_lmp_pdu_ptr;
				signal.ext_param = (OS_ADDRESS)((UINT32)ce_ptr->phy_piconet_id);

				OS_SEND_SIGNAL_TO_TASK(lmp_task_handle, signal);
                ce_ptr->rem_park_pending_for_afh = FALSE;
            }
        }
    }
#endif /* AFH_PARK */
#endif /* COMPILE_PARK_MODE */

    /* Check if MSS HCI cmd is pending. */
    if(ce_ptr->mss_cmd_pending == TRUE)
    {
        /* Post signal to HCI-cmd-task to start command processing. */
        OS_SIGNAL signal;
        signal.type = HCI_CMD_RECD_SIGNAL;
        signal.length = 0x0;
        signal.param = NULL;
        UINT32 context = (HCI_SWITCH_ROLE_OPCODE << 16) | ce_index;
        signal.ext_param = (OS_ADDRESS)context;

        /* Send the Signal to HCI Task. */
        OS_SEND_SIGNAL_TO_TASK(
                        rx_table[HCI_CMD_HANDLER_TASK].task_handle,signal);

        /* Clear the flag that says that MSS is pending. */
        ce_ptr->mss_cmd_pending = FALSE;
    }

    if(ce_ptr->mss_pdu_pending == TRUE)
    {
        /* Post signal to LMP-task to start PDU processing. */
        OS_SIGNAL signal;
        signal.type = LMP_PDU_RECD_SIGNAL;
        signal.length = 0x0;
        signal.param = ce_ptr->mss_pdu_ptr;

		/* Physical piconet ID */
		signal.ext_param = (OS_ADDRESS)((UINT32)(ce_ptr->phy_piconet_id));

        /* Send the Signal to LMP Task. */
        OS_SEND_SIGNAL_TO_TASK(lmp_task_handle, signal);

        /* Clear the flag that says that MSS is pending. */
        ce_ptr->mss_pdu_pending = FALSE;
    }

    return;
}

/**
 * Handles the BB_ACK for LMP_set_AFH. It updates Baseband and LC module
 * params if required or just sets LMP flags. This function sends AFH
 * procedure completion conditionally.
 *
 * \param ce_index Connection entity index of the connection.
 *
 * \return None.
 */
void lmp_handle_ack_received_for_set_afh_pdu(UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY *ce_ptr;

    UINT32 piconet_clock = 0;

	ce_ptr = &lmp_connection_entity[ce_index];

	lc_get_clock_in_scatternet(&piconet_clock, ce_ptr->phy_piconet_id);

#ifdef ENABLE_LOGGER_LEVEL_2
	LC_LOG_INFO(LOG_LEVEL_LOW, ACK_RECVD_FOR_SET_AFH, 1, ce_ptr->am_addr);

#endif

    ce_ptr->waiting_for_set_afh_pdu_ack = FALSE;

    memcpy(&ce_ptr->last_acked_afh_map[0], &ce_ptr->afh_map[0],
                LMP_AFH_MAP_SIZE);
    switch(ce_ptr->afh_ce_status)
    {
        case LMP_AFH_ENABLING: /* Fall through. */
#ifdef COMPILE_NESTED_PAUSE_RESUME
			aclq_resume_am_addr(ce_ptr->am_addr,
				ce_ptr->phy_piconet_id, ACL_PAUSED_AFH);
#else /* COMPILE_NESTED_PAUSE_RESUME */
			aclq_resume_am_addr(ce_ptr->am_addr,
				ce_ptr->phy_piconet_id);
#endif /* COMPILE_NESTED_PAUSE_RESUME */

		case LMP_AFH_UPDATING:
            ce_ptr->afh_ce_status = LMP_AFH_ENABLED;
            break;

        case LMP_AFH_DISABLING:
            ce_ptr->afh_ce_status = LMP_AFH_DISABLED;

#ifdef COMPILE_NESTED_PAUSE_RESUME
			aclq_resume_am_addr(ce_ptr->am_addr,
				ce_ptr->phy_piconet_id, ACL_PAUSED_AFH);
#else /* COMPILE_NESTED_PAUSE_RESUME */
			aclq_resume_am_addr(ce_ptr->am_addr, ce_ptr->phy_piconet_id);
#endif /* COMPILE_NESTED_PAUSE_RESUME */
			break;

        default:
            break;
    }

    if(ce_ptr->afh_instant_timer_handle == NULL)
    {
		if(ce_ptr->afh_mode == AFH_ENABLE)
        {
            BB_write_afh_map(ce_ptr->am_addr,
                ce_ptr->phy_piconet_id, ce_ptr->afh_map);
        }
        else
        {
			BB_disable_afh(ce_ptr->am_addr, ce_ptr->phy_piconet_id);
		}
    }

     return;
}

/**
 * Calculate the AFH instant for the piconet. This function considers Tpoll,
 * sniff interval and hold intervals for all the slaves in the piconet. This
 * function also starts instant timer and sets flag for the same.
 *
 * \param piconet_id The Logical Piconet ID.
 * \param ce_index ACL Connection entity index.
 *
 * \return None.
 */
UCHAR lmp_calculate_afh_instant(UINT16 ce_index)
{
	UINT32 native_clock;
	UINT32 afh_instant_offset;
	UINT32 afh_instant;
	UINT32 no_of_enqueued_pdus, curr_tpoll;
	LMP_CONNECTION_ENTITY *ce_ptr;

	UCHAR piconet_id;

	ce_ptr = &lmp_connection_entity[ce_index];

	piconet_id = ce_ptr->phy_piconet_id;

	/* AFH instant offset should atleast be tpoll*6 */
    no_of_enqueued_pdus =  pduq_get_no_of_pdus_piconet(piconet_id) + 1;
	curr_tpoll = ce_ptr->Tpoll;
	afh_instant_offset =  no_of_enqueued_pdus * curr_tpoll * 6;

#ifdef COMPILE_SNIFF_MODE
    if (ce_ptr->in_sniff_mode == TRUE)
    {
        UINT32 temp;

        temp = ce_ptr->sniff_interval * 3;
        if (temp > afh_instant_offset)
        {
            afh_instant_offset = temp;
        }

        if (ce_ptr->ssr_data.lmp_ssr_state == LMP_SSR_ACTIVE)
        {
            temp = ce_ptr->ssr_data.tsniff * 3;

            if (temp > afh_instant_offset)
            {
                afh_instant_offset = temp;
            }

            temp = ce_ptr->ssr_data.rem_max_ssr * ce_ptr->sniff_interval * 3;
            if( temp > afh_instant_offset)
            {
                afh_instant_offset = temp;
            }
        }
    }
#endif /* COMPILE_SNIFF_MODE */

#ifdef COMPILE_HOLD_MODE
    if(ce_ptr->ce_status == LMP_HOLD_MODE)
    {
        if(ce_ptr->hold_mode_interval > afh_instant_offset)
        {
            afh_instant_offset = ce_ptr->hold_mode_interval + ce_ptr->Tpoll;
        }
    }
#endif /* COMPILE_HOLD_MODE */

    if(afh_instant_offset < LMP_MIN_AFH_SLOTS)
    {
        afh_instant_offset = LMP_MIN_AFH_SLOTS;
    }

#ifdef _ADAPTED_ADJUST_AFH_INSTANT_
    {
        /* added by austin */

        UINT8 ch_index;
        UINT8 good_channels = 0;
        UINT8 byte_idx;
        UINT8 bit_num;

        for (ch_index = 0; ch_index < LMP_MAX_CHANNELS; ch_index++)
        {
            byte_idx = ch_index >> 3;
            bit_num = ch_index & 0x07;
            if (ce_ptr->old_afh_map[byte_idx] & (1 << bit_num))
            {
                good_channels++;
            }
        }

        if (good_channels == 0)
        {
            /* avoid exception */
            good_channels = 20;
        }

        /* consider the interference, we maybe adjust the instant
           to be double or trible */
        afh_instant_offset *= LMP_MAX_CHANNELS / good_channels;
    }
#endif

	LC_EXIT_SM_MODE();

	lc_get_clock_in_scatternet(&native_clock, ce_ptr->phy_piconet_id);

    native_clock = native_clock >> 1;
    afh_instant = native_clock + afh_instant_offset;

    ce_ptr->afh_instant =  afh_instant;

    /* Make afh instant an even value */
    ce_ptr->afh_instant++;
    ce_ptr->afh_instant &= BT_CLOCK_27_BITS;
    ce_ptr->afh_instant &= (~0x01);

    /* Run the afh instant timer to reach afh instant */
    afh_instant_offset = (UINT32)
        (SLOT_VAL_TO_TIMER_VAL(afh_instant_offset));

    if(OS_CREATE_TIMER(ONESHOT_TIMER, &ce_ptr->afh_instant_timer_handle,
            lmp_handle_afh_instant_timer, (void *)((UINT32)ce_index), 0) != BT_ERROR_OK )
    {
		AFH_ERR(CLASSIFICTION_TIMER_FAILED,0,0);
    }

    if(OS_START_TIMER(ce_ptr->afh_instant_timer_handle,
                                        afh_instant_offset)!= BT_ERROR_OK)
    {
		AFH_ERR(INSTANT_TIMER_START_FAILED,0,0);
    }

	LMP_LOG_INFO(LOG_LEVEL_HIGH, AFH_INSTANT_CALCULATION_AFH_OFFSET, 1,
		afh_instant_offset);

    return API_SUCCESS;
}

/**
 * Sends LMP_set_AFH PDU to the 1.2 AFH supporting slaves. It takes map and
 * instant from #lmp_self_device_data.
 *
 * \param ce_index Connection entity index of the connection.
 *
 * \return None.
 */
void lmp_send_set_afh_pdu(UINT16 ce_index, UINT8 afh_mode)
{
    UINT32 afh_instant;
    UCHAR parameter_list[LMP_SET_AFH_LEN];
    LMP_CONNECTION_ENTITY *ce_ptr;

    //RT_BT_LOG(GREEN, LMP_MSG_SET_AFH_PDU, 0, 0);

    ce_ptr = &lmp_connection_entity[ce_index];
    ce_ptr->afh_mode = afh_mode;

    /* Take Calculated AFH instant from lmp self device data */
    afh_instant = ce_ptr->afh_instant;

    parameter_list[0] = LMP_SET_AFH_OPCODE;
    memcpy(&parameter_list[2], &afh_instant, 4);
    parameter_list[6] = afh_mode;

    /* Update connection entity status for this connection */
    if (afh_mode == AFH_ENABLE)
    {
#ifdef _ENABLE_MAILBOX_
        //back up for BT channel
       {
            EFUSE_POW_PARA_S_TYPE efuse_pow_para;
            efuse_pow_para.d16 = otp_str_data.power_seq_param;
            if (efuse_pow_para.b.force_mask_wifi_ch_en)
            {
                mailbox_set_afh_map(g_wifi_ch, ce_ptr->afh_map);
            }
        }
#endif

        memcpy(&parameter_list[7], &ce_ptr->afh_map[0], LMP_AFH_MAP_SIZE);

        /* Check if no of channels is less than 20.*/
    }
    else
    {
        memset(&parameter_list[7],0xFF,LMP_AFH_MAP_SIZE);
    }

    /* map[9] should be 0x7F */
    parameter_list[6+LMP_AFH_MAP_SIZE] &= 0x7F;

    /* Store old map for recovery seqnence. */
    if (afh_mode == AFH_ENABLE)
    {
        memcpy(&ce_ptr->old_afh_map[0], &ce_ptr->afh_map[0],
                LMP_AFH_MAP_SIZE);
    }

    /* Set lmp module parameters. */
    ce_ptr->waiting_for_set_afh_pdu_ack = TRUE;

    lmp_generate_pdu(ce_index, parameter_list, LMP_SET_AFH_LEN, MASTER_TID,
            LMP_NO_STATE_CHANGE);

    ce_ptr->last_set_afh_sent_clk = BB_read_native_clock();

        UINT8 pre_afh_ce_status =  ce_ptr->afh_ce_status;
        UINT8 err = FALSE;

        if (afh_mode == AFH_ENABLE)
        {
            if (pre_afh_ce_status == LMP_AFH_DISABLED)
            {
                ce_ptr->afh_ce_status = LMP_AFH_ENABLING;
            }
            else if (pre_afh_ce_status == LMP_AFH_ENABLED)
            {
                ce_ptr->afh_ce_status = LMP_AFH_UPDATING;
            }
            else
            {
                err = TRUE;
            }
        }
        else
        {
            /* afh mode is disabled */
            if (pre_afh_ce_status == LMP_AFH_ENABLED)
            {
                ce_ptr->afh_ce_status = LMP_AFH_DISABLING;
            }
            else
            {
                err = TRUE;
            }
        }

        if (err)
        {
            RT_BT_LOG(RED, MSG_SET_AFH_PDU_STATE_ERR, 2,
                            afh_mode, pre_afh_ce_status);
        }
}

/**
 * Decides to send AFH PDUs to the 1.2 AFH supporting slaves during
 * connection setup. It takes map and instant from #lmp_self_device_data.
 *
 * \param ce_index Connection entity index of the connection.
 *
 * \return None.
 */
void lmp_decide_for_lmp_afh_pdu(UINT16 ce_index)
{
	LMP_CONNECTION_ENTITY *ce_ptr;

#ifdef COMPILE_CHECK_LOCAL_FEATURES
	UCHAR local_feature;
	/* Check whether local device suports AFH. */

    //RT_BT_LOG(RED, LMP_AFH_1018, 0, 0);

	ce_ptr = &lmp_connection_entity[ce_index];

	local_feature = lmp_feature_data.feat_page0[5];

	if( (local_feature & AFH_CAPABLE_MASTER) == FALSE)
	{
		return;
	}
#endif

	/* Check whether remote device suports AFH and we are master */
	if((ce_ptr->afh_mode == AFH_DISABLE) &&
		(ce_ptr->feat_page0[4] & AFH_CAPABLE_SLAVE) &&
		(ce_ptr->remote_dev_role == SLAVE))
	{
		/* If AFH update is pending for the whole piconet , then
		* we just do nothing, when the pending procedure is taken up
		* the map will be updated accordingly
		*/

        lmp_generate_afh_map(ce_ptr->phy_piconet_id, &ce_ptr->afh_map[0]);
		lmp_start_afh_map_updation(&ce_ptr->afh_map[0],
			ce_index, AFH_ENABLE);

		if( (lmp_feature_data.feat_page0[5] & AFH_CLASSIFICATION_MASTER) &&
			(ce_ptr->feat_page0[4] & AFH_CLASSIFICATION_SLAVE))
		{
			/* When updation for the AFH map will complete then we need to send
			* channel classification req pdu because we can send
			* classification req only to a afh enabled slave */
			ce_ptr->need_to_send_ch_cls_req = TRUE;
		}
	}

	return;
}

/**
 * Handle the channel classification request PDU.
 *
 * \param lmp_pdu_ptr Pointer to the #LMP_PDU_PKT.
 * \param ce_index Connection entity index of the connection.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR
 *         otherwise.
 */
UCHAR lmp_handle_channel_classification_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index)
{
	UCHAR local_slave_feature;
	UCHAR remote_master_feature;
    UINT32 afh_ch_max_interval_period;

#ifdef COMPILE_CHANNEL_ASSESSMENT
	UCHAR map[LMP_AFH_MAP_SIZE];
#endif /* COMPILE_CHANNEL_ASSESSMENT */
	LMP_CONNECTION_ENTITY *ce_ptr;

    //RT_BT_LOG(RED, LMP_AFH_1093, 0, 0);

	ce_ptr = &lmp_connection_entity[ce_index];

	local_slave_feature = lmp_feature_data.feat_page0[4];

	remote_master_feature = ce_ptr->feat_page0[5];

    /* Validate for local and remote features */
    /* Validate Pdu is recieved from master */
    if( (ce_ptr->remote_dev_role == MASTER) &&
        ( (local_slave_feature & AFH_CLASSIFICATION_SLAVE) &&
          (remote_master_feature & AFH_CLASSIFICATION_MASTER) ))
    {
        /* Send one lmp_channel_classification_pdu immediately, with all
           channels as unknown. */
        lmp_generate_afh_map(ce_ptr->phy_piconet_id, &map[0]);

        /*
         * Store max and min interval and afh_reporting_mode
         * in connection entity
         */
        ce_ptr->afh_ch_cl_reporting_mode = lmp_pdu_ptr->payload_content[2];

        ce_ptr->afh_min_interval = lmp_pdu_ptr->payload_content[3] |
                                   (lmp_pdu_ptr->payload_content[4] << 8);

        ce_ptr->afh_max_interval = lmp_pdu_ptr->payload_content[5] |
                                   (lmp_pdu_ptr->payload_content[6] << 8);

        /* Unit of max-interval is slots, convert them into ms. */
        afh_ch_max_interval_period = (ce_ptr->afh_max_interval * 5) / 8;

        if (afh_ch_max_interval_period > 300)
        {
            afh_ch_max_interval_period -= 200;
        }

        if (afh_la_cycle_period > afh_ch_max_interval_period)
        {
            afh_la_cycle_period = afh_ch_max_interval_period;
            OS_STOP_TIMER(la_period_timer, 0);
            OS_START_TIMER(la_period_timer, afh_la_cycle_period);
#ifdef ENABLE_LOGGER_LEVEL_2
            RT_TMP_L0G("afh_la_cycle_period modified to %x",
                afh_la_cycle_period);
#endif
        }

        if(ce_ptr->afh_ch_cl_reporting_mode == AFH_ENABLE)
        {
            lmp_send_ch_cl_pdu(&map[0], ce_index);
        }
    }
    else
    {
        /* Send not accepted to remote device */
         lmp_send_lmp_not_accepted_ext(ce_index, LMP_ESCAPE4_OPCODE,
                               LMP_CHANNEL_CLASSIFICATION_REQ_OPCODE,
                               REMOTE_DEV_TID,
                               UNSUPPORTED_FEATURE_OR_PARAMETER_ERROR);
    }

    return BT_FW_SUCCESS;
}

/**
 * Handle the channel classification PDU.
 *
 * \param lmp_pdu_ptr The pointer to the #LMP_PDU_PKT.
 * \param ce_index Connection entity index of the connection.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR
 *         otherwise.
 */
UCHAR lmp_handle_channel_classification_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index)
{
	UCHAR local_feature;
 	UCHAR am_address;
	UCHAR remote_feature;

#ifndef COMPILE_CHANNEL_ASSESSMENT
	UCHAR count1, count2, temp_var, n_min = 0;
#endif /* COMPILE_CHANNEL_ASSESSMENT */

	LMP_CONNECTION_ENTITY *ce_ptr;

	//RT_BT_LOG(RED, LMP_AFH_1220, 0, 0);

	ce_ptr = &lmp_connection_entity[ce_index];

	local_feature = lmp_feature_data.feat_page0[5];

	remote_feature = ce_ptr->feat_page0[4];

    /* Validate for local and remote features */
    /* Validate PDU is received from slave */
	if( (ce_ptr->remote_dev_role == SLAVE) &&
		( (local_feature & AFH_CLASSIFICATION_MASTER) &&
		(remote_feature & AFH_CLASSIFICATION_SLAVE) ))
	{
#ifdef COMPILE_CHANNEL_ASSESSMENT
		/* Save the slave classification report in LUT */
		am_address = ce_ptr->am_addr;

		/* Store the map in CE. */
		memcpy(ce_ptr->last_recd_ch_cl_map,
			&lmp_pdu_ptr->payload_content[2], LMP_AFH_MAP_SIZE);
            ce_ptr->new_ch_cl_map = 0x1;
#endif
	}
        else
        {
            /*Send not accepted to remote device */
            lmp_send_lmp_not_accepted_ext(ce_index,LMP_ESCAPE4_OPCODE,
                                       LMP_CHANNEL_CLASSIFICATION_OPCODE,
                                       REMOTE_DEV_TID, PDU_NOT_ALLOWED_ERROR);
        }

    return BT_FW_SUCCESS;

}

/**
 * Sends the Channel Classification request PDU.
 *
 * \param afh_mode AFH_ENABLE or AFH_DISABLE.
 * \param ce_index Connection entity index of the connection.
 *
 * \return None.
 */
void lmp_send_ch_cl_req_pdu(UCHAR afh_mode, UINT16 ce_index)
{
    UCHAR parameter_list[LMP_CHANNEL_CLASSIFICATION_REQ_LEN];

    //RT_BT_LOG(RED, LMP_AFH_1272, 0, 0);

    parameter_list[0] = LMP_ESCAPE4_OPCODE;
    parameter_list[2] = LMP_CHANNEL_CLASSIFICATION_REQ_OPCODE;
    parameter_list[3] = afh_mode;
    parameter_list[4] = LSB(lmp_self_device_data.cl_rep_min_interval);
    parameter_list[5] = MSB(lmp_self_device_data.cl_rep_min_interval);
    parameter_list[6] = LSB(lmp_self_device_data.cl_rep_max_interval);
    parameter_list[7] = LSB(lmp_self_device_data.cl_rep_max_interval);

    lmp_generate_pdu(ce_index, parameter_list,
            LMP_CHANNEL_CLASSIFICATION_REQ_LEN, MASTER_TID,
            LMP_NO_STATE_CHANGE);
}

/**
 * Sends the Channel Classification PDU.
 *
 * \param afh_map AFH map.
 * \param ce_index Connection entity index of the connection.
 *
 * \return None.
 */
void lmp_send_ch_cl_pdu(UCHAR *afh_map,UINT16 ce_index)
{
    UCHAR parameter_list[LMP_CHANNEL_CLASSIFICATION_LEN];
    UCHAR cls_map[LMP_AFH_MAP_SIZE];
    INT32 result;
    UINT32 current_clk;
    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    if(ce_ptr->afh_ch_cl_reporting_mode == AFH_DISABLE)
    {
        return;
    }

#ifdef COMPILE_PARK_MODE
    if(ce_ptr->ce_status == LMP_PARK_MODE)
    {
        return;
    }
#endif

#ifdef TEST_MODE
    /* Do not send if DUT mode has started. */
    if (ce_ptr->test_mode_info.test_state == TEST_ACTIVATED)
    {
        //LMP_LOG_INFO(LOG_LEVEL_HIGH, "Not sending ch-cl-pdu in DUT mode.");

        return;
    }
#endif
#ifdef _NO_SEND_AFH_POWER_CTRL_WHILE_ROLE_SW
#ifdef _ENABLE_COMPILE_NESTED_PAUSE_RESUME_
    UCHAR lut_index;
    lut_index = lc_get_lut_index_from_phy_piconet_id(ce_ptr->am_addr, ce_ptr->phy_piconet_id);
    if(paused_lut_index[lut_index])
    {
         RT_BT_LOG(RED, DAPE_TEST_LOG586, 3, ce_index, LMP_ESCAPE4_OPCODE, LMP_CHANNEL_CLASSIFICATION_OPCODE);
         return;
    }
#else
    if(ce_ptr->pause_data_transfer)
    {
         RT_BT_LOG(RED, DAPE_TEST_LOG586, 3, ce_index, LMP_ESCAPE4_OPCODE, LMP_CHANNEL_CLASSIFICATION_OPCODE);
         return;
    }
#endif
#endif
    current_clk = BB_read_native_clock();
    if(ce_ptr->last_set_afh_sent_clk != 0xFFFFFFFF)
    {
        if( (current_clk - ce_ptr->last_set_afh_sent_clk) <
          (ce_ptr->afh_min_interval << 1))
        {
            /* Note: Clock wrap around is not considered here, as
               there is nothing seriously lost, if IUT violates this 10
               seconds interval.*/

            return;
        }
    }
    /*
     * When assessment feature is on, afh_map is in the channel
     * classification PDU's map format (pair classification).
     */
#ifdef COMPILE_CHANNEL_ASSESSMENT
    lmp_make_map_for_ch_cl_pdu(afh_map, &cls_map[0]);
#endif /* COMPILE_CHANNEL_ASSESSMENT */

    result = memcmp(ce_ptr->last_recd_ch_cl_map,&cls_map[0],LMP_AFH_MAP_SIZE);

    if(result != 0)
    {
        memcpy(ce_ptr->last_recd_ch_cl_map,&cls_map[0],LMP_AFH_MAP_SIZE);
        parameter_list[0] = LMP_ESCAPE4_OPCODE;
        parameter_list[2] = LMP_CHANNEL_CLASSIFICATION_OPCODE;

        memcpy(&parameter_list[3],&cls_map[0],LMP_AFH_MAP_SIZE);

        lmp_generate_pdu(ce_index, parameter_list,
                LMP_CHANNEL_CLASSIFICATION_LEN, SLAVE_TID,
                LMP_NO_STATE_CHANGE);

        ce_ptr->last_set_afh_sent_clk = current_clk;
    }
    else
    {
		AFH_INF(CLASSIFICATION_HAS_NOT_CHANGE_FROM_LAST_TIME,0,0);
    }

    return;
}

/*
 *
This parameter contains
40 2-bit fields.
The nth (numbering from 0)
such field defines the classification
of channels 2n
and 2n+1, other than the
39th field which just contains
the classification of
channel 78.
Each field interpreted as
an integer whose values
indicate:
0 = unknown
1 = good
2 = reserved
3 = bad
 *
 */
/**
 * Convert channel classification into the channel classification pair
 * to send it into the channel classification PDU.
 *
 * \param afh_map The input map.
 * \param classified_map The output map.
 *
 * \return None.
 */
void lmp_make_map_for_ch_cl_pdu(UCHAR *afh_map, UCHAR *classified_map)

{
    UCHAR byte_index, bit_index, ch_status;
    UCHAR ch_index;
    UCHAR good_channel_value;

    memset(classified_map, 0x0, LMP_AFH_MAP_SIZE);

    if (lmp_self_device_data.afh_channel_assessment_mode == AFH_DISABLE)
    {
        /* Report good channel pair as 0x0 => Unknown */
        good_channel_value = 0x0;
    }
    else
    {
        /* Report good channel pair as 0x01 => Good */
        good_channel_value = 0x01;
    }

   /* 0 = unknown 1 = good 2 = reserved 3 = bad */
    for (ch_index = 0; ch_index < (LMP_MAX_CHANNELS - 2); ch_index += 2)
    {
        bit_index = ch_index & 0x07;
        byte_index = ch_index >> 3;

        ch_status = (afh_map[byte_index] >> bit_index) & 0x03;

        if (ch_status == 0x03)
        {
            /* GOOD Pair channel. */
            classified_map[byte_index] |=  (good_channel_value << (bit_index));
        }
        else
        {
            /* BAD Pair */
            classified_map[byte_index] |= (0x03 << (bit_index));
        }
    }

    /* for channel 78 */
    if (afh_map[9] & (1 << 6))
    {
        classified_map[9] |= good_channel_value << 6;
    }
    else
    {
        classified_map[9] |= 0x03 << 6;
    }
    return;
}

#endif /* COMPILE_AFH_HOP_KERNEL */

