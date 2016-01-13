/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  Contains LMP module initialization routines implementation.
 */

/********************************* Logger *************************/
enum { __FILE_NUM__= 46};
/********************************* Logger *************************/

/* ========================= Include File Section ========================= */
#include "lmp_internal.h"
#include "bz_debug.h"
#include "bt_fw_acl_q.h"
#include "mem.h"
#include "UartPrintf.h"
#include "lmp_2_1.h"
#ifdef _SECURE_CONN_TEST_LOG
#include "bz_auth_internal.h"
#endif

/* ==================== Structure declaration Section ===================== */

/* ===================== Variable Declaration Section ===================== */
TASK_ID lmp_task_handle;                /**< LMP Task handle */
POOL_ID lmp_pdu_buffer_pool_handle;     /**< PDU buffers pool handle */
POOL_ID lmp_fhs_pkt_buffer_pool_handle; /**< FHS packet buffers pool handle */
CACHE_TABLE bd_addr_cache[MAX_CACHE];   /**< Device Information Cache */
UCHAR bd_addr_cache_start = INVALID_INDEX;
UCHAR bd_addr_cache_free = INVALID_INDEX;
#ifdef _CORRECT_LMP_RESPONSE_TIMER
UINT32 g_lmp_response_timeout_time = 30000;
#endif
#if defined(TEST_MODE)
extern LMP_PDU_PKT *test_mode_lmp_pdu;
#endif
#ifdef _DAPE_TEST_ACL_RX_FIFO_DBG
extern UINT8 lc_sco_pause_status;
#endif

/* ================== Static Function Prototypes Section ================== */


/* ================== For rom code patch function point ================== */
#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8723_RCP_
PF_ROM_CODE_PATCH_FUNC rcp_lmp_exit_sniff_mode = NULL;
#endif
#ifdef _CCH_8821_RCP_
PF_ROM_CODE_PATCH_FUNC rcp_lmp_release_am_addr_ppi = NULL;
#endif

#endif

/* ===================== Function Definition Section ====================== */
/**
 * Initializes the LMP module. The main data structures that are initialized
 * are #LMP_CONNECTION_ENTITY, #LMP_SELF_DEVICE_DATA, afh related structures,
 * #LMP_SCO_CONNECTION_DATA, etc.,
 *
 * \param None.
 *
 * \return API_SUCCESS, if the operation is successful. API_FAILURE, otherwise.
 */
API_RESULT lmp_init(void)
{
    bz_auth_init();
    lmp_initialize_host_controller();

    return API_SUCCESS;
}

/**
 * Shutdown the LMP module. It deletes the LMP related buffer pools and
 * #LMP_Task.
 *
 * \param None.
 *
 * \return API_SUCCESS, if the operation is successful. API_FAILURE, otherwise.
 */
API_RESULT lmp_shutdown(void)
{
#ifndef _DONT_SUPPORT_OS_SHUTDOWN_

    OS_DELETE_POOL(lmp_fhs_pkt_buffer_pool_handle);
    OS_DELETE_TASK(lmp_task_handle);
    OS_DELETE_POOL(lmp_pdu_buffer_pool_handle);

#endif

    return API_SUCCESS;
}

/**
 * Reset the LMP module. It reinitializes the host controller (same as
 * #lmp_init).
 *
 * \param None.
 *
 * \return API_SUCCESS, if the operation is successful. API_FAILURE, otherwise.
 */
API_RESULT lmp_module_reset(void)
{
    lmp_initialize_host_controller();

    return API_SUCCESS;
}

/**
 * Generate LMP pdu and deliver it to the remote device. It simply queues the
 * pdu to the LC module and it in turn delivers the PDU to remote device.
 *
 * \param ce_index The index of the Connection entity database corresponding
 *                 to the link.
 * \param param_list LMP PDU parameter list. Element0 will be the opcode,
 *                   Element1 will be our role(Master/Slave), Element2 and
 *                   the rest will be specific to the PDU to be sent.
 * \param pdu_len Length of the PDU (Use predefined macros like
 *                #LMP_ACCEPTED_LEN - if length has to be hard coded, then it
 *                should be Bluetooth spec defined length of that PDU plus
 *                one).
 * \param tran_id Transaction ID to be used for this PDU.
 * \param ce_status The status with which the main LM state machine has to be
 *                  updated.
 * \param use_dm1 TRUE, if DM1 has to be used to transmit the PDU. FALSE,
 *                if any packet type (DV/DM1) can be used.
 *
 * \return API_SUCCESS, if the operation is successful. API_FAILURE, otherwise.
 *
 * \note This function should not be used directly. Use either
 *       #lmp_generate_pdu or #lmp_generate_pdu_dm1.
 */
API_RESULT lmp_generate_pdu_impl(UINT16 ce_index, UCHAR* param_list,
                                 UCHAR pdu_len, LMP_TRAN_ID tran_id, UINT8 ce_status,
                                 UCHAR use_dm1)
{
    LMP_PDU_PKT* pdu_ptr;
    LMP_CONNECTION_ENTITY* ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];
    BZ_ASSERT(ce_ptr->am_addr < 8, "Invalid am_addr");

    if (ce_status != LMP_NO_STATE_CHANGE)
    {
        lmp_set_ce_status(ce_index, ce_status);
    }

    if ((ce_ptr->detach_timer_state != IDLE_STATE)
            && (param_list[0] != LMP_DETACH_OPCODE))
    {
        /* After initiating detach procedure, no LMP PDUs (other than
         * LMP_detach, ofcourse) will be queued for transmission.
         */
        return API_SUCCESS;
    }

    /* Allocate LMP PDU buffer from corresponding buffer pool */
    if (OS_ALLOC_BUFFER(lmp_pdu_buffer_pool_handle, (void **)(&pdu_ptr))
            != BT_ERROR_OK)
    {
        LMP_LOG_INFO(LOG_LEVEL_HIGH, PDU_BUFFER_CREATION_FAILED, 0, 0);
        return API_FAILURE;
    }


    /* Determine transaction ID */
    switch (tran_id)
    {
        case MASTER:
        case SLAVE:
            param_list[1] = (UCHAR)tran_id;
            break;
        case SELF_DEV_TID:
            param_list[1] = (UCHAR)(!ce_ptr->remote_dev_role);
            break;
        case REMOTE_DEV_TID:
            param_list[1] = ce_ptr->remote_dev_role;
            break;
        default:
            BZ_ASSERT(0, "lmp_generate_pdu: Invalid TID");
    }

    /* Populate PDU buffer */
    pdu_ptr->payload_content[0] =
        (UCHAR)((param_list[0]<<1) | (param_list[1] & 0x01));

    if (pdu_len >= 2)
    {
        memcpy(&pdu_ptr->payload_content[1], &param_list[2], pdu_len-2);
    }

    pdu_ptr->pdu_length = (UCHAR) (pdu_len - 1);
    pdu_ptr->am_addr = ce_ptr->am_addr;
    pdu_ptr->use_dm1 = use_dm1;
    pdu_ptr->pdu_sent = FALSE;

#ifdef POWER_CONTROL
    DEF_CRITICAL_SECTION_STORAGE;
    LMP_PDU_PKT* pdu_tmp = NULL;
    MINT_OS_ENTER_CRITICAL();

    if (test_mode_lmp_pdu != NULL)
    {
        pdu_tmp = test_mode_lmp_pdu;
        test_mode_lmp_pdu = NULL;
    }

#ifdef _RTL8723B_DUT_MODE_
    {
        UINT8 opcode1;
        UINT8 opcode2;

        opcode1 = pdu_ptr->payload_content[0] >> 1;
        opcode2 = pdu_ptr->payload_content[1];

        if ((opcode1 == LMP_MAX_POWER_OPCODE) ||
            (opcode1 == LMP_MIN_POWER_OPCODE)
#ifdef VER_3_0
            || (IS_BT30 && ((opcode1 == LMP_ESCAPE4_OPCODE) &&
            (opcode2 == LMP_POWER_CTRL_RES_OPCODE)))
#endif
        )
        {
            /* exit dut mode */
            BB_write_baseband_register(RADIO_SELECT_REGISTER, 0x0107);
        }
    }
#endif

    /* this is a back-door solution to help send lmp pkt fast in
       Tx mode of DUT for RF test */
    if ((lmp_self_device_data.test_mode == HCI_DEVICE_UNDER_TEST_MODE) &&
        (lc_is_tx_test_mode == TRUE) &&
        (ce_ptr->test_mode_info.test_state == TEST_STARTED ))
    {
        UINT8 opcode1;
        UINT8 opcode2;

        opcode1 = pdu_ptr->payload_content[0] >> 1;
        opcode2 = pdu_ptr->payload_content[1];
        if ((opcode1 == LMP_MAX_POWER_OPCODE) ||
            (opcode1 == LMP_MIN_POWER_OPCODE)
#ifdef VER_3_0
            || (IS_BT30 && ((opcode1 == LMP_ESCAPE4_OPCODE) &&
            (opcode2 == LMP_POWER_CTRL_RES_OPCODE)))
#endif
        )
        {
            test_mode_lmp_pdu = pdu_ptr;
            MINT_OS_EXIT_CRITICAL();

            if (pdu_tmp != NULL)
            {
                OS_FREE_BUFFER(lmp_pdu_buffer_pool_handle, (void*)pdu_tmp);
            }
            return API_SUCCESS;
        }
    }

    MINT_OS_EXIT_CRITICAL();

    if (pdu_tmp != NULL)
    {
        OS_FREE_BUFFER(lmp_pdu_buffer_pool_handle, (void*)pdu_tmp);
    }
#endif

    /* Deliver the LMP PDU to LC module for transmission */
    if (lc_handle_lmp_pdus(ce_index, pdu_ptr, ce_ptr->phy_piconet_id)
            != API_SUCCESS)
    {
        if (OS_FREE_BUFFER(lmp_pdu_buffer_pool_handle, (void*)pdu_ptr) != BT_ERROR_OK)
        {
            RT_BT_LOG(GRAY, LMP_196, 0, 0);
        }
        LMP_LOG_ERROR(LOG_LEVEL_HIGH, ERROR_GENERATE_PDU_FAILED,0,0);
        return API_FAILURE;
    }

    lmp_handle_start_pdu_response_time(ce_index, param_list);

    UINT32 cur_clk;
    lc_get_clock_in_scatternet(&cur_clk, ce_ptr->phy_piconet_id);

#ifdef _DAPE_TEST_ACL_RX_FIFO_DBG
    UINT32 acl_rx_fifo_reg;
    acl_rx_fifo_reg = RD_U32_BZDMA_REG(BZDMA_REG_ACL_RXFIFO_PTR);
#endif
#ifdef _DAPE_CHK_MAX_SLOT
    //RT_BT_LOG(BLUE, DAPE_TEST_LOG207, 1, ce_ptr->pkts_allowed);
#endif
#ifdef _SECURE_CONN_TEST_LOG
        BZ_AUTH_LINK_PARAMS* auth;
        auth = lmp_connection_entity[ce_index].auth;
///// dape test
//RT_BT_LOG(BLUE, DAPE_TEST_LOG522, 2,auth->super_state, auth->sub_state);
#endif
    //by liuyong 20090727
#ifdef _DAPE_TEST_CHK_CLK
{
RT_BT_LOG(WHITE, YL_DBG_HEX_5, 5, BB_read_native_clock(), cur_clk,
BB_read_baseband_register(0x22),
BB_read_baseband_register(0x24),
BB_read_baseband_register(0x26));
}
#endif

    if (param_list[0]!=127)
    {
#ifndef _DAPE_TEST_ACL_RX_FIFO_DBG
        RT_BT_LOG(GREEN, LMP_MSG_SENT_NON_EXT_PDU, 15,
                        cur_clk, param_list[0], pdu_ptr->am_addr,
                        ce_ptr->phy_piconet_id, param_list[1], ce_index,
                        ce_ptr->remote_dev_role,
                        pdu_ptr->payload_content[0],
                        pdu_ptr->payload_content[1],
                        pdu_ptr->payload_content[2],
                        pdu_ptr->payload_content[3],
                        pdu_ptr->payload_content[4],
                        pdu_ptr->payload_content[5],
                        pdu_ptr->payload_content[6],
                        pdu_ptr->payload_content[7]);
#else
        RT_BT_LOG(GREEN, DAPE_TEST_LMP_MSG_SENT_NON_EXT_PDU, 17,
                        cur_clk, param_list[0], pdu_ptr->am_addr,
                        ce_ptr->phy_piconet_id, param_list[1], ce_index,
                        ce_ptr->remote_dev_role,
                        pdu_ptr->payload_content[0],
                        pdu_ptr->payload_content[1],
                        pdu_ptr->payload_content[2],
                        pdu_ptr->payload_content[3],
                        pdu_ptr->payload_content[4],
                        pdu_ptr->payload_content[5],
                        pdu_ptr->payload_content[6],
                        pdu_ptr->payload_content[7],
                        lc_sco_pause_status, acl_rx_fifo_reg);

#endif
    }
    else
    {
#ifndef _DAPE_TEST_ACL_RX_FIFO_DBG
        RT_BT_LOG(GREEN, LMP_MSG_SENT_EXT_PDU, 16,
                        cur_clk, param_list[0], param_list[2], pdu_ptr->am_addr,
                        ce_ptr->phy_piconet_id, param_list[1], ce_index,
                        ce_ptr->remote_dev_role,
                        pdu_ptr->payload_content[0],
                        pdu_ptr->payload_content[1],
                        pdu_ptr->payload_content[2],
                        pdu_ptr->payload_content[3],
                        pdu_ptr->payload_content[4],
                        pdu_ptr->payload_content[5],
                        pdu_ptr->payload_content[6],
                        pdu_ptr->payload_content[7]);
#else
        RT_BT_LOG(GREEN, DAPE_TEST_LMP_MSG_SENT_EXT_PDU, 18,
                        cur_clk, param_list[0], param_list[2], pdu_ptr->am_addr,
                        ce_ptr->phy_piconet_id, param_list[1], ce_index,
                        ce_ptr->remote_dev_role,
                        pdu_ptr->payload_content[0],
                        pdu_ptr->payload_content[1],
                        pdu_ptr->payload_content[2],
                        pdu_ptr->payload_content[3],
                        pdu_ptr->payload_content[4],
                        pdu_ptr->payload_content[5],
                        pdu_ptr->payload_content[6],
                        pdu_ptr->payload_content[7],
                        lc_sco_pause_status, acl_rx_fifo_reg);
#endif
    }
#ifdef _DUMP_LMP_TX_PACKET_
    LMP_LOG_PDU(LOG_LEVEL_HIGH, LOG_PDU_INDEX, LOG_TX_DIR, 20, (UCHAR *)(pdu_ptr->payload_content));
#endif
    return API_SUCCESS;
}

/**
 * Returns the corresponding local defined bit mask
 *
 * \param pdu_opcode  PDU opcode
 * \param pdu_ext_opcode Extended PDU opcode in the case of ext PDU's
 *
 * \return Returns the opcode bit mask value
 */

UINT32 lmp_get_opcode_mask(UCHAR pdu_opcode, UCHAR pdu_ext_opcode)
{
    UINT32 opcode_mask = BIT_MASK_NO_PDU;

    switch(pdu_opcode)
    {
        case LMP_NAME_RES_OPCODE:
            opcode_mask = BIT_MASK_LMP_NAME_RES_OPCODE;
            break;
        case LMP_FEATURES_RES_OPCODE:
            opcode_mask = BIT_MASK_LMP_FEATURES_RES_OPCODE;
            break;
        case LMP_VERSION_RES_OPCODE:
            opcode_mask = BIT_MASK_LMP_VERSION_RES_OPCODE;
            break;
        case LMP_CLKOFFSET_RES_OPCODE:
            opcode_mask = BIT_MASK_LMP_CLKOFFSET_RES_OPCODE;
            break;
        case LMP_SLOT_OFFSET_OPCODE:
            opcode_mask = BIT_MASK_LMP_SLOT_OFFSET_OPCODE;
            break;
        case LMP_SETUP_COMPLETE_OPCODE:
            opcode_mask = BIT_MASK_LMP_SETUP_COMPLETE_OPCODE;
            break;
        case LMP_MAX_POWER_OPCODE:
            opcode_mask = BIT_MASK_LMP_MAX_POWER_OPCODE;
            break;
        case LMP_MIN_POWER_OPCODE:
            opcode_mask = BIT_MASK_LMP_MIN_POWER_OPCODE;
            break;
        case LMP_SNIFF_REQ_OPCODE:
            opcode_mask = BIT_MASK_LMP_SNIFF_REQ_OPCODE;
            break;
        case LMP_HOLD_OPCODE:
            opcode_mask = BIT_MASK_LMP_HOLD_OPCODE;
            break;
        case LMP_HOLD_REQ_OPCODE:
            opcode_mask = BIT_MASK_LMP_HOLD_REQ_OPCODE;
            break;
        case LMP_PARK_REQ_OPCODE:
            opcode_mask = BIT_MASK_LMP_PARK_REQ_OPCODE;
            break;
        case LMP_SCO_LINK_REQ_OPCODE:
            opcode_mask = BIT_MASK_LMP_SCO_LINK_REQ_OPCODE;
            break;
        case LMP_HOST_CONNECTION_REQ_OPCODE:
            opcode_mask = BIT_MASK_LMP_HOST_CONNECTION_REQ_OPCODE;
            break;
        case LMP_VERSION_REQ_OPCODE:
            opcode_mask = BIT_MASK_LMP_VERSION_REQ_OPCODE ;
            break;
        case LMP_FEATURES_REQ_OPCODE:
            opcode_mask = BIT_MASK_LMP_FEATURES_REQ_OPCODE;
            break;
        case LMP_NAME_REQ_OPCODE:
            opcode_mask = BIT_MASK_LMP_NAME_REQ_OPCODE;
            break;
        case LMP_CLKOFFSET_REQ_OPCODE:
            opcode_mask = BIT_MASK_LMP_CLKOFFSET_REQ_OPCODE;
            break;
        case LMP_MAX_SLOT_REQ_OPCODE:
            opcode_mask = BIT_MASK_LMP_MAX_SLOT_REQ_OPCODE;
            break;
        case LMP_SWITCH_REQ_OPCODE:
            opcode_mask = BIT_MASK_LMP_SWITCH_REQ_OPCODE;
            break;
        case LMP_QoS_REQ_OPCODE:
            opcode_mask = BIT_MASK_LMP_QoS_REQ_OPCODE;
            break;
        case LMP_PAGE_SCAN_MODE_REQ_OPCODE:
            opcode_mask = BIT_MASK_LMP_PAGE_SCAN_MODE_REQ_OPCODE;
            break;
        case LMP_PAGE_MODE_REQ_OPCODE:
            opcode_mask = BIT_MASK_LMP_PAGE_MODE_REQ_OPCODE;
            break;
        case LMP_REMOVE_SCO_LINK_REQ_OPCODE:
            opcode_mask = BIT_MASK_LMP_REMOVE_SCO_LINK_REQ_OPCODE;
            break;
        case LMP_ESCAPE4_OPCODE:
            switch(pdu_ext_opcode)
            {
                case LMP_FEATURES_REQ_EXT_OPCODE:
                    opcode_mask = BIT_MASK_LMP_FEATURES_REQ_EXT_OPCODE;
                    break;
                case LMP_FEATURES_RES_EXT_OPCODE:
                    opcode_mask = BIT_MASK_LMP_FEATURES_RES_EXT_OPCODE;
                    break;
                case LMP_SNIFF_SUBRATING_REQ_OPCODE:
                    opcode_mask = BIT_MASK_LMP_SNIFF_SUBRATING_REQ_OPCODE;
                    break;
                case LMP_SNIFF_SUBRATING_RES_OPCODE:
                    opcode_mask = BIT_MASK_LMP_SNIFF_SUBRATING_RES_OPCODE;
                    break;
                case LMP_ESCO_LINK_REQ_OPCODE:
                    opcode_mask = BIT_MASK_LMP_ESCO_LINK_REQ_OPCODE;
                    break;
                case LMP_REMOVE_ESCO_LINK_REQ_OPCODE:
                    opcode_mask = BIT_MASK_LMP_REMOVE_ESCO_LINK_REQ_OPCODE;
                    break;
#ifdef _SUPPORT_SECURE_CONNECTION_
                case LMP_PING_REQ_OPCODE:
                    opcode_mask = BIT_MASK_LMP_PING_REQ_OPCODE;
                    break;
                case LMP_PING_RES_OPCODE:
                    opcode_mask = BIT_MASK_LMP_PING_RES_OPCODE;
                    break;

#endif
                default:
                    break;
            }
            break;
        default:
            break;
    }

    return opcode_mask;
}


/**
 * Starts the response timer for the self device or the remote device initiated
 * request pdus.
 *
 * \param ce_index The index of the Connection entity database corresponding
 * \param parameter_list The parameters used to form the PDU.
 *
 * \return API_SUCCESS, if the operation is successful. API_FAILURE, otherwise.
 */

UCHAR lmp_handle_start_pdu_response_time(UINT16 ce_index, UCHAR* parameter_list)
{
    UCHAR pdu_opcode = parameter_list[0];
    UCHAR trans_id = parameter_list[1];
    UCHAR self_dev_trans = TRUE;
    LMP_CONNECTION_ENTITY *ce_ptr;
    UCHAR am_addr;

    ce_ptr = &lmp_connection_entity[ce_index];

    am_addr = ce_ptr->am_addr ;
    if (trans_id == ce_ptr->remote_dev_role)
    {
        self_dev_trans = FALSE ;
    }

    /* Self device initiated transaction */
    if (self_dev_trans == TRUE)
    {
        switch(pdu_opcode)
        {
            case LMP_HOST_CONNECTION_REQ_OPCODE : /* Fall through */
            case LMP_VERSION_REQ_OPCODE : /* Fall through */
            case LMP_FEATURES_REQ_OPCODE: /* Fall through */
            case LMP_NAME_REQ_OPCODE : /* Fall through */
            case LMP_CLKOFFSET_REQ_OPCODE : /* Fall through */
            case LMP_MAX_SLOT_REQ_OPCODE : /* Fall through */

#ifdef COMPILE_ROLE_SWITCH
            case LMP_SWITCH_REQ_OPCODE : /* Fall through */
#endif

            case LMP_QoS_REQ_OPCODE : /* Fall through */

#ifdef _CCH_IOT_CSR_RS_QOS_
                if(ce_ptr->waiting_for_rs_several_times != 0)
                {
                  RT_BT_LOG(GRAY, CCH_DBG_083, 0,0);
                  break;
                }
#endif

            case LMP_PAGE_SCAN_MODE_REQ_OPCODE : /* Fall through */
            case LMP_PAGE_MODE_REQ_OPCODE : /* Fall through */
#ifdef ENABLE_SCO
            case LMP_SCO_LINK_REQ_OPCODE : /* Fall through */
            case LMP_REMOVE_SCO_LINK_REQ_OPCODE : /* Fall through */
#endif

#ifdef COMPILE_HOLD_MODE
            case LMP_HOLD_REQ_OPCODE : /* Fall through */
#endif /* COMPILE_HOLD_MODE */

#ifdef COMPILE_SNIFF_MODE
            case LMP_SNIFF_REQ_OPCODE : /* Fall through */
            case LMP_UNSNIFF_REQ_OPCODE : /* Fall through */
#endif /* COMPILE_SNIFF_MODE */

#ifdef COMPILE_PARK_MODE
            case LMP_PARK_REQ_OPCODE :
#endif /* COMPILE_PARK_MODE */

                /* Store the PDU opcode sent to the remote device....*/
                ce_ptr->sent_pdu |= lmp_get_opcode_mask(pdu_opcode,0x0);
                ce_ptr->sent_lmp_pdu_opcode = pdu_opcode;
                ce_ptr->sent_lmp_pdu_ext_opcode = 0xFF;

                /* Check already the response timer is running for the same
                 * connection. Start the timer if you initiate the transaction
                 */
                if (ce_ptr->pdu_response_timer_running == TRUE)
                {
                    OS_STOP_TIMER(ce_ptr->lmp_response_timer_handle, 0);
                }

                if (ce_ptr->sent_pdu != BIT_MASK_NO_PDU)
                {
#ifdef _CORRECT_LMP_RESPONSE_TIMER
                    OS_START_TIMER(ce_ptr->lmp_response_timer_handle,
                               g_lmp_response_timeout_time);
#else
                    OS_START_TIMER(ce_ptr->lmp_response_timer_handle,
                               LMP_RESPONSE_TIMEOUT);
#endif
                    ce_ptr->pdu_response_timer_running = TRUE;
                }
                break;

            case LMP_ESCAPE4_OPCODE:
            {
                UCHAR pdu_ext_opcode = parameter_list[2];
                switch(pdu_ext_opcode)
                {
#ifdef COMPILE_FEATURE_REQ_EXT
                    case LMP_FEATURES_REQ_EXT_OPCODE:
#endif
                    case LMP_SNIFF_SUBRATING_REQ_OPCODE:
#ifdef COMPILE_ESCO
                    case LMP_ESCO_LINK_REQ_OPCODE:
                    case LMP_REMOVE_ESCO_LINK_REQ_OPCODE:
#endif  /* COMPILE_ESCO */
#ifdef _SUPPORT_SECURE_CONNECTION_
                    case LMP_PING_REQ_OPCODE:
#endif
                        /* Check already the response timer is running for the
                         * same connection. Start the timer if you initiate the
                                             * transaction.
                                             */
                        ce_ptr->sent_pdu |= lmp_get_opcode_mask(pdu_opcode,
                                                                pdu_ext_opcode);
                        ce_ptr->sent_lmp_pdu_opcode = pdu_opcode;
                        ce_ptr->sent_lmp_pdu_ext_opcode = pdu_ext_opcode;
#ifdef _SUPPORT_SECURE_CONNECTION_
                        if (pdu_ext_opcode != LMP_PING_REQ_OPCODE)
#endif
                        {
                        if (ce_ptr->pdu_response_timer_running == TRUE)
                        {
                            OS_STOP_TIMER(ce_ptr->lmp_response_timer_handle, 0);
                        }
                        if (ce_ptr->sent_pdu != BIT_MASK_NO_PDU)
                        {
#ifdef _CORRECT_LMP_RESPONSE_TIMER
                            OS_START_TIMER(ce_ptr->lmp_response_timer_handle,
                                       g_lmp_response_timeout_time);
#else
                            OS_START_TIMER(ce_ptr->lmp_response_timer_handle,
                                       LMP_RESPONSE_TIMEOUT);
#endif
                            ce_ptr->pdu_response_timer_running = TRUE ;
                        }
                        }
#ifdef _SUPPORT_SECURE_CONNECTION_
                        else
                        {
                            if (ce_ptr->pdu_response_timer_running == FALSE)
                            {
#ifdef _CORRECT_LMP_RESPONSE_TIMER
                                OS_START_TIMER(ce_ptr->lmp_response_timer_handle,
                                           g_lmp_response_timeout_time);
#else
                                OS_START_TIMER(ce_ptr->lmp_response_timer_handle,
                                           LMP_RESPONSE_TIMEOUT);
#endif
                                ce_ptr->pdu_response_timer_running = TRUE ;

                            }
                        }
#endif

                        break;

                    default:
                        break;
                }
            }

            default :
                break;
        }
    }
    else
    {
        /* Remote device initiated transaction */
        switch(pdu_opcode)
        {
#ifdef ENABLE_SCO
            case LMP_SCO_LINK_REQ_OPCODE :
#endif

#ifdef COMPILE_HOLD_MODE
            case LMP_HOLD_REQ_OPCODE :
#endif /* COMPILE_HOLD_MODE */

#ifdef COMPILE_SNIFF_MODE
            case LMP_SNIFF_REQ_OPCODE :
#endif /* COMPILE_SNIFF_MODE */

#ifdef COMPILE_PARK_MODE
            case LMP_PARK_REQ_OPCODE :
#endif /* COMPILE_PARK_MODE */

                /* Store the PDU opcode sent to the remote device....*/
                ce_ptr->sent_pdu |= lmp_get_opcode_mask(pdu_opcode, 0x0);
                ce_ptr->sent_lmp_pdu_opcode = pdu_opcode;
                ce_ptr->sent_lmp_pdu_ext_opcode = 0xFF;

                /* Check already the response timer is running for the same
                * connection. Start the timer if you initiate the transaction
                */
                if (ce_ptr->pdu_response_timer_running == TRUE)
                {
                    OS_STOP_TIMER(ce_ptr->lmp_response_timer_handle, 0);
                }
                if (ce_ptr->sent_pdu != BIT_MASK_NO_PDU)
                {
#ifdef _CORRECT_LMP_RESPONSE_TIMER
                    OS_START_TIMER(ce_ptr->lmp_response_timer_handle,
                               g_lmp_response_timeout_time);
#else
                    OS_START_TIMER(ce_ptr->lmp_response_timer_handle,
                               LMP_RESPONSE_TIMEOUT);
#endif
                    ce_ptr->pdu_response_timer_running = TRUE;
                }
                break;

            case LMP_ESCAPE4_OPCODE:
            {
                UCHAR pdu_ext_opcode = parameter_list[2];
                switch(pdu_ext_opcode)
                {
#ifdef COMPILE_ESCO
                    case LMP_ESCO_LINK_REQ_OPCODE:
                    case LMP_REMOVE_ESCO_LINK_REQ_OPCODE:
#endif  /* COMPILE_ESCO */
                        /* Check already the response timer is running for
                         * the same connection. Start the timer if you
                         * initiate the transaction
                                             */
                        ce_ptr->sent_pdu |= lmp_get_opcode_mask(pdu_opcode,
                                                                pdu_ext_opcode);
                        ce_ptr->sent_lmp_pdu_opcode = pdu_opcode;
                        ce_ptr->sent_lmp_pdu_ext_opcode = pdu_ext_opcode;

                        if (ce_ptr->pdu_response_timer_running == TRUE)
                        {
                            OS_STOP_TIMER(ce_ptr->lmp_response_timer_handle, 0);
                        }
                        if (ce_ptr->sent_pdu != BIT_MASK_NO_PDU)
                        {
#ifdef _CORRECT_LMP_RESPONSE_TIMER
                            OS_START_TIMER(ce_ptr->lmp_response_timer_handle,
                               g_lmp_response_timeout_time);
#else
                            OS_START_TIMER(ce_ptr->lmp_response_timer_handle,
                                       LMP_RESPONSE_TIMEOUT);
#endif
                            ce_ptr->pdu_response_timer_running = TRUE;
                        }

                        break;

                    default:
                        break;

                }
            }
            break;

            default :
                break;
        }
    }

    return API_SUCCESS;
}

/**
 * Stops the response timer for the self device or the remote device initiated
 * request pdus.
 *
 * \param ce_index The index of the Connection entity database corresponding
 *                 to the link.
 * \param pdu_pkt The pointer to the LMP PDU packet which contains the PDU
 *                information(#LMP_PDU_PKT).
 *
 * \return API_SUCCESS, if the operation is successful. API_FAILURE, otherwise.
 */
UCHAR lmp_handle_stop_pdu_response_time(UINT16 ce_index, LMP_PDU_PKT *pdu_pkt)
{
    UCHAR recd_pdu_opcode ;
    UCHAR temp_var ;
    UCHAR trans_id;
    UCHAR self_dev_trans;
    UCHAR am_addr;

    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    am_addr = ce_ptr->am_addr;

    trans_id = (UCHAR)(pdu_pkt->payload_content[0] & 0x01);
    recd_pdu_opcode = (UCHAR)((pdu_pkt->payload_content[0] >> 1) & 0xFF);

    if (trans_id == ce_ptr->remote_dev_role)
    {
        self_dev_trans = FALSE ;
    }

    ce_ptr->lmp_expected_pdu_opcode &= (~(lmp_get_opcode_mask(
            recd_pdu_opcode, 0x0)));

    switch(recd_pdu_opcode)
    {
        case LMP_ACCEPTED_OPCODE :
            temp_var = pdu_pkt->payload_content[1];
            ce_ptr->sent_pdu &= (~(lmp_get_opcode_mask(temp_var, 0x0)));
            switch (pdu_pkt->payload_content[1])
            {
                case LMP_HOLD_REQ_OPCODE:
                    ce_ptr->lmp_expected_pdu_opcode &= (~(lmp_get_opcode_mask(
                            LMP_HOLD_REQ_OPCODE, 0x0)));
                    break;
                case LMP_SNIFF_REQ_OPCODE:
                    ce_ptr->lmp_expected_pdu_opcode &= (~(lmp_get_opcode_mask(
                            LMP_SNIFF_REQ_OPCODE, 0x0)));
                    break;
                case LMP_SWITCH_REQ_OPCODE:
                    ce_ptr->lmp_expected_pdu_opcode &= (~(lmp_get_opcode_mask(
                            LMP_SWITCH_REQ_OPCODE, 0x0)));
                    break;
                default:
                    break;
            }

            break;
        case LMP_NOT_ACCEPTED_OPCODE :
            temp_var = pdu_pkt->payload_content[1];
            if ((temp_var != LMP_UNSNIFF_REQ_OPCODE) &&
                    (temp_var != LMP_REMOVE_SCO_LINK_REQ_OPCODE))
            {
                ce_ptr->sent_pdu &= (~(lmp_get_opcode_mask(temp_var, 0x0)));
            }
            switch (pdu_pkt->payload_content[1])
            {
                case LMP_HOLD_REQ_OPCODE:
                    ce_ptr->lmp_expected_pdu_opcode &= (~(lmp_get_opcode_mask(
                            LMP_HOLD_REQ_OPCODE, 0x0)));
                    break;
                case LMP_SNIFF_REQ_OPCODE:
                    ce_ptr->lmp_expected_pdu_opcode &= (~(lmp_get_opcode_mask(
                            LMP_SNIFF_REQ_OPCODE, 0x0)));
                    break;
                case LMP_SWITCH_REQ_OPCODE:
                    ce_ptr->lmp_expected_pdu_opcode &= (~(lmp_get_opcode_mask(
                            LMP_SLOT_OFFSET_OPCODE, 0x0)));
                    ce_ptr->lmp_expected_pdu_opcode &= (~(lmp_get_opcode_mask(
                            LMP_SWITCH_REQ_OPCODE, 0x0)));
                    break;
                case LMP_PARK_REQ_OPCODE:
                    ce_ptr->lmp_expected_pdu_opcode &= (~(lmp_get_opcode_mask(
                            LMP_PARK_REQ_OPCODE, 0x0)));
                    break;
                case LMP_SCO_LINK_REQ_OPCODE:
                    ce_ptr->lmp_expected_pdu_opcode &= (~(lmp_get_opcode_mask(
                            LMP_SCO_LINK_REQ_OPCODE, 0x0)));
                    break;
                default:
                    break;
            }

            break;
        case LMP_SETUP_COMPLETE_OPCODE:
            ce_ptr->sent_pdu &= (~(lmp_get_opcode_mask(
                                       LMP_HOST_CONNECTION_REQ_OPCODE, 0x0)));
            break;
        case LMP_VERSION_RES_OPCODE:
            ce_ptr->sent_pdu &= (~(lmp_get_opcode_mask(
                                       LMP_VERSION_REQ_OPCODE, 0x0)));
            break;
        case LMP_FEATURES_RES_OPCODE:
            ce_ptr->sent_pdu &= (~(lmp_get_opcode_mask(
                                       LMP_FEATURES_REQ_OPCODE, 0x0)));
            break;
        case LMP_NAME_RES_OPCODE:
            ce_ptr->sent_pdu &= (~(lmp_get_opcode_mask(
                                       LMP_NAME_REQ_OPCODE, 0x0)));
            break;
        case LMP_CLKOFFSET_RES_OPCODE:
            ce_ptr->sent_pdu &= (~(lmp_get_opcode_mask(
                                       LMP_CLKOFFSET_REQ_OPCODE, 0x0)));
            break;
        case LMP_SLOT_OFFSET_OPCODE:
            ce_ptr->sent_pdu &= (~(lmp_get_opcode_mask(
                                       LMP_SWITCH_REQ_OPCODE, 0x0)));
            break;
        case LMP_HOLD_OPCODE:
            ce_ptr->sent_pdu &= (~(lmp_get_opcode_mask(
                                       LMP_HOLD_OPCODE, 0x0)));
            break;
        case LMP_HOLD_REQ_OPCODE:
            ce_ptr->sent_pdu &= (~(lmp_get_opcode_mask(
                                       LMP_HOLD_REQ_OPCODE, 0x0)));
            break;
        case LMP_PARK_REQ_OPCODE:
            ce_ptr->sent_pdu &= (~(lmp_get_opcode_mask(
                                       LMP_PARK_REQ_OPCODE, 0x0)));
            break;
        case LMP_SNIFF_REQ_OPCODE:
            ce_ptr->sent_pdu &= (~(lmp_get_opcode_mask(
                                       LMP_SNIFF_REQ_OPCODE, 0x0)));
            break;
        case LMP_SCO_LINK_REQ_OPCODE:
            ce_ptr->sent_pdu &= (~(lmp_get_opcode_mask(
                                       LMP_SCO_LINK_REQ_OPCODE, 0x0)));
            break;

        case LMP_ESCAPE4_OPCODE:
            ce_ptr->lmp_expected_pdu_opcode &= (~(lmp_get_opcode_mask(
                    LMP_ESCAPE4_OPCODE,
                    pdu_pkt->payload_content[1])));
            switch(pdu_pkt->payload_content[1])
            {
                case LMP_ACCEPTED_EXT_OPCODE:
                case LMP_NOT_ACCEPTED_EXT_OPCODE:
                    switch(pdu_pkt->payload_content[2])
                    {
                        case LMP_ESCAPE1_OPCODE:
                        case LMP_ESCAPE2_OPCODE:
                        case LMP_ESCAPE3_OPCODE:
                            break;

                        case LMP_ESCAPE4_OPCODE:
                        {
                            switch(pdu_pkt->payload_content[3])
                            {
                                case LMP_SNIFF_SUBRATING_REQ_OPCODE:
                                    ce_ptr->sent_pdu &= (~(lmp_get_opcode_mask(
                                                               LMP_ESCAPE4_OPCODE,
                                                               LMP_SNIFF_SUBRATING_REQ_OPCODE)));

                                    ce_ptr->lmp_expected_pdu_opcode &=
                                        (~(lmp_get_opcode_mask(
                                               LMP_ESCAPE4_OPCODE,
                                               LMP_SNIFF_SUBRATING_RES_OPCODE)));
                                    break;

#ifdef COMPILE_ESCO
                                case LMP_ESCO_LINK_REQ_OPCODE:
#endif /* COMPILE _ESCO */
                                    ce_ptr->sent_pdu &= (~(lmp_get_opcode_mask(
                                                               LMP_ESCAPE4_OPCODE,
                                                               pdu_pkt->payload_content[3])));
                                    break;
#ifdef COMPILE_ESCO
                                case LMP_REMOVE_ESCO_LINK_REQ_OPCODE:
                                    if (pdu_pkt->payload_content[1]
                                            != LMP_NOT_ACCEPTED_OPCODE)
                                    {
                                        ce_ptr->sent_pdu &= (~(lmp_get_opcode_mask(
                                                                   LMP_ESCAPE4_OPCODE,
                                                                   pdu_pkt->payload_content[3])));
                                    }
                                    break;
#endif /* COMPILE _ESCO */

                                default:
                                    break;
                            }
                        }
                    }
                    break;

#ifdef COMPILE_ESCO
                case LMP_ESCO_LINK_REQ_OPCODE:
                    ce_ptr->sent_pdu &= (~(lmp_get_opcode_mask(
                                               LMP_ESCAPE4_OPCODE, LMP_ESCO_LINK_REQ_OPCODE)));
                    break;
#endif /* COMPILE _ESCO */
                case LMP_FEATURES_RES_EXT_OPCODE:
                    ce_ptr->sent_pdu &= (~(lmp_get_opcode_mask(
                                               LMP_ESCAPE4_OPCODE,
                                               LMP_FEATURES_REQ_EXT_OPCODE)));
                    break;
                case LMP_SNIFF_SUBRATING_RES_OPCODE:
                    ce_ptr->sent_pdu &= (~(lmp_get_opcode_mask(
                                               LMP_ESCAPE4_OPCODE,
                                               LMP_SNIFF_SUBRATING_REQ_OPCODE)));
                    break;
#ifdef _SUPPORT_SECURE_CONNECTION_
                case LMP_PING_RES_OPCODE:
                    ce_ptr->sent_pdu &= (~(lmp_get_opcode_mask(
                                               LMP_ESCAPE4_OPCODE,
                                               LMP_PING_REQ_OPCODE)));
#ifdef _SECURE_CONN_TEST_LOG
//RT_BT_LOG(BLUE, DAPE_TEST_LOG207, 1, ce_ptr->sent_pdu);
#endif

#endif
                default:
                    break;
            }
        default :
            break;
    }

    /* Stop PDU Timer
     * if (sent_pdu == BIT_MASK_NO_PDU && expected_pdu == BIT_MASK_NO_PDU)
     */

    if ((ce_ptr->sent_pdu == BIT_MASK_NO_PDU) &&
            (ce_ptr->lmp_expected_pdu_opcode== BIT_MASK_NO_PDU))
    {
        /* Stop the response timer. */
        if (ce_ptr->pdu_response_timer_running == TRUE)
        {
            OS_STOP_TIMER(ce_ptr->lmp_response_timer_handle, 0);
            ce_ptr->pdu_response_timer_running = FALSE;
        }
    }
    return BT_FW_SUCCESS ;
}

/**
 * Get the \a ce_index corresponding to the \a bd_addr provided. It uses the
 * hash mechanism to search the BD_ADDR to CE_INDEX table
 * (#LMP_BD_TBL_TO_CE_INDEX_MAP).
 *
 * \param bd_addr BD_ADDR (Bluetooth Device Address) of the device.
 * \param ce_index The address of a variable through which the ce_index is
 *                 returned.
 *
 * \return API_SUCCESS, if the operation is successful. API_FAILURE, otherwise.
 */

API_RESULT lmp_get_CE_index_from_BD_ADDR(UCHAR *bd_addr, UINT16* ce_index)
{
    UINT32 temp;

    *ce_index = INVALID_CE_INDEX;

    for (temp = 0; temp < LMP_MAX_CE_DATABASE_ENTRIES; temp++)
    {
        if (lmp_connection_entity[temp].entity_status == ASSIGNED)
        {
            /* Compare the bd address. */
            if ( memcmp(lmp_connection_entity[temp].bd_addr, bd_addr, LMP_BD_ADDR_SIZE) == 0)
            {
                /* bd address match. */
                *ce_index = (UINT16) temp;
                return API_SUCCESS;
            }
        }
    }

    return API_FAILURE;
}

/**
 * Get the \a ce_index corresponding to the \a am_addr provided. It uses the
 * hash mechanism to search the AM_ADDR to CE_INDEX table
 * (#LMP_AM_ADDR_TO_CE_INDEX_TABLE).
 *
 * \param am_addr AM_ADDR (Active Member Address - a.k.a LT_ADDR) of the device.
 * \param piconet_id The ID representing the piconet of the device.
 * \param ce_index The address of a variable through which the ce_index is
 *                 returned.
 *
 * \return API_SUCCESS, if the operation is successful. API_FAILURE, otherwise.
 */
API_RESULT lmp_get_CE_index_from_AM_ADDR_PPI(UCHAR am_addr, UCHAR phy_piconet_id,
        UINT16* ce_index)
{
    *ce_index = INVALID_CE_INDEX;

    if (phy_piconet_id >= LMP_MAX_PICONETS_SUPPORTED)
    {
        return API_FAILURE;
    }

    if ((am_addr < LC_MAX_AM_ADDR) && (am_addr != BC_AM_ADDR))
    {
        LMP_AM_ADDR_TO_CE_INDEX_TABLE *table;

        table = &lmp_am_addr_to_ce_index_table_ppi[am_addr-1][phy_piconet_id];

        if ((table->status == UNASSIGNED) ||
            !lmp_am2ce_is_acl_ce_index(table->ce_index))
        {
            return API_FAILURE;
        }

        *ce_index = (UINT16)lmp_am2ce_demangle_acl_ce_index(table->ce_index);
        return API_SUCCESS;
    }

    return API_FAILURE;
}

/**
 * Get the \a ce_index corresponding to the \a conn_handle provided. It uses
 * the hash mechanism to search the BD_ADDR to CE_INDEX table
 * (#LMP_CONN_HANDLE_TO_CE_INDEX_TABLE).
 *
 * \param conn_handle Connection handle representing the connection.
 * \param ce_index The address of a variable through which the ce_index is
 *                 returned.
 *
 * \return API_SUCCESS, if the operation is successful. API_FAILURE, otherwise.
 */
API_RESULT lmp_get_CE_index_from_conn_handle(UINT16 conn_handle, UINT16* ce_index)
{
    *ce_index = INVALID_CE_INDEX;

    if ((conn_handle > 0) &&
       (conn_handle <= (LMP_MAX_CE_DATABASE_ENTRIES + LMP_MAX_BC_CONNECTIONS)))
    {
        if (lmp_ch_to_ce_index_table[conn_handle-1].status == UNASSIGNED)
        {
            return API_FAILURE;
        }
        else
        {
            *ce_index = lmp_ch_to_ce_index_table[conn_handle-1].ce_index ;
            if (*ce_index == INVALID_CE_INDEX)
            {
                return API_FAILURE;
            }
            return API_SUCCESS ;
        }
    }
    return API_FAILURE ;
}

#ifdef COMPILE_PARK_MODE

/**
 * Get the \a ce_index corresponding to the \a ar_addr provided. It uses the
 * hash mechanism to search the AR_ADDR to CE_INDEX table
 * (#LMP_AR_ADDR_TO_CE_INDEX_TABLE).
 *
 * \param ar_addr AR_ADDR (Access Request Address) of the device.
 * \param ce_index The address of a variable through which the ce_index is
 *                 returned.
 * \param piconet_id The ID representing the piconet of the device.
 *
 * \return API_SUCCESS, if the operation is successful. API_FAILURE, otherwise.
 */
API_RESULT lmp_get_CE_index_from_AR_ADDR(UCHAR ar_addr, UINT16* ret_ce_index,
        UCHAR phy_piconet_id)
{
    *ret_ce_index = INVALID_CE_INDEX;

    UINT16 ce_index;
    for(ce_index = 0; ce_index < LMP_MAX_CE_DATABASE_ENTRIES; ce_index++)
    {
        if (lmp_connection_entity[ce_index].entity_status == ASSIGNED &&
                lmp_connection_entity[ce_index].ar_addr == ar_addr &&
                lmp_connection_entity[ce_index].phy_piconet_id == phy_piconet_id)
        {
            *ret_ce_index = ce_index;
            return API_SUCCESS;
        }
    }
    return API_FAILURE;
}

/**
 * Get the \a ce_index corresponding to the \a pm_addr provided. It uses the
 * hash mechanism to search the PM_ADDR to CE_INDEX table
 * (#LMP_PM_ADDR_TO_CE_INDEX_TABLE).
 *
 * \param pm_addr PM_ADDR (Parked Member Address) of the device.
 * \param ce_index The address of a variable through which the ce_index is
 *                 returned.
 * \param piconet_id The ID representing the piconet of the device.
 *
 * \return API_SUCCESS, if the operation is successful. API_FAILURE, otherwise.
 */
API_RESULT lmp_get_CE_index_from_PM_ADDR(UCHAR pm_addr, UINT16* ret_ce_index,
        UCHAR piconet_id)
{
    UINT16 ce_index;

    *ret_ce_index = INVALID_CE_INDEX;

    for(ce_index = 0; ce_index < LMP_MAX_CE_DATABASE_ENTRIES; ce_index++)
    {
        if (lmp_connection_entity[ce_index].entity_status == ASSIGNED &&
                lmp_connection_entity[ce_index].pm_addr == pm_addr &&
                lmp_connection_entity[ce_index].phy_piconet_id == piconet_id)
        {
            *ret_ce_index = ce_index;
            return API_SUCCESS;
        }
    }
    return API_FAILURE;
}
#endif /* COMPILE_PARK_MODE */

/**
 * Allocate the connection entity from connection entity database and AM_ADDR
 * from the AM_ADDR to connection entity table. It also creates Tpoll timer
 * associated with the connection.
 *
 * \param am_addr The address of a variable through which the newly allocated
 *                am_addr (Active Member Address) is returned.
 * \param ce_index The address of a variable through which the newly allocated
 *                 ce_index (Connection Entity Index) is returned.
 *
 * \return API_SUCCESS, if the operation is successful. API_FAILURE, otherwise.
 */
UCHAR lmp_allocate_am_addr(UCHAR *am_addr, UINT16 *ce_index)
{
    UCHAR temp_am_addr;
    UINT16 temp_ce_index;
    UCHAR phy_piconet_id;

    phy_piconet_id = lc_allocate_piconet_id_for_paging();
    if (phy_piconet_id == SCA_PICONET_INVALID)
    {
        return UNSPECIFIED_ERROR;
    }

    if (lmp_get_am_addr_ppi(&temp_am_addr, phy_piconet_id) == API_SUCCESS)
    {
#ifdef COMPILE_NESTED_PAUSE_RESUME
        aclq_resume_am_addr(temp_am_addr, phy_piconet_id, ACL_PAUSED_ALL);
#else /* COMPILE_NESTED_PAUSE_RESUME */
        aclq_resume_am_addr(temp_am_addr, phy_piconet_id);
#endif /* COMPILE_NESTED_PAUSE_RESUME */

        aclq_mark_am_addr_flow_go(temp_am_addr, phy_piconet_id);

        if ((temp_ce_index = lmp_allocate_entity_from_ce_database())
                != BT_FW_ERROR)
        {
            lmp_am_addr_to_ce_index_table_ppi[temp_am_addr-1][phy_piconet_id].ce_index
                    = lmp_am2ce_mangle_acl_ce_index(temp_ce_index);

            lmp_connection_entity[temp_ce_index].phy_piconet_id = phy_piconet_id;

            *am_addr = temp_am_addr;
            *ce_index = temp_ce_index;

            return API_SUCCESS;
        }
        else
        {
            lmp_put_am_addr_ppi(temp_am_addr, phy_piconet_id);
        }
    }

    return MAX_NUMBER_OF_CONNECTIONS_ERROR;
}

/**
* Releases the am_addr of the connection.
*
* \param am_addr The am_addr of the connection.
* \param phy_piconet_id Phy piconet ID of the connection.
*
* \return API_SUCCESS on successfully releasing, and API_FAILURE on failure.
*/
API_RESULT lmp_release_am_addr_ppi(UCHAR am_addr, UINT16 phy_piconet_id)
{
    UINT16 ce_index;

#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8821_RCP_
    UCHAR return_status;
    if (rcp_lmp_release_am_addr_ppi != NULL)
    {
        if (rcp_lmp_release_am_addr_ppi((void *)&return_status,
			            am_addr, phy_piconet_id))
        {
            return return_status;
        }
    }
#endif
#endif

    if ((am_addr > 0) && (am_addr < LC_MAX_AM_ADDR))
    {
        LMP_AM_ADDR_TO_CE_INDEX_TABLE *table;
        table = &lmp_am_addr_to_ce_index_table_ppi[am_addr-1][phy_piconet_id];
        ce_index = table->ce_index;

        if ((table->status == UNASSIGNED) ||
            (!lmp_am2ce_is_acl_ce_index(ce_index)))
        {
            /* Unassigned or Already released */
            RT_BT_LOG(GRAY, LMP_1173, 0, 0);
            return API_FAILURE;
        }
        else
        {
            ce_index = lmp_am2ce_demangle_acl_ce_index(ce_index);
            lmp_put_am_addr_ppi(am_addr, phy_piconet_id);
            lmp_release_entity_to_ce_database(ce_index);

            return API_SUCCESS;
        }
    } /* end if (am_addr is within bounds) */

    return API_FAILURE;
}

/**
* Remove the BD_ADDR from the BD_ADDR to Connecion Entity hash table
* (#LMP_BD_TBL_TO_CE_INDEX_MAP).
*
* \param bd_addr The BD_ADDR of the entry to be removed from the table.
*
* \return None.
*/
void lmp_remove_bd_addr_from_hash(UCHAR *bd_addr)
{
#ifdef _CCH_PAGE_CON_
    memset(bd_addr, 0x00, LMP_BD_ADDR_SIZE);
#endif


    return;
}

#ifdef COMPILE_PARK_MODE
/**
 * Allocate a free PM_ADDR.
 *
 * \param pm_addr The Parked Mode Address.
 * \param log_piconet_id The Physical Piconet ID.
 *
 * \return API_SUCCESS, if the operation is successful. API_FAILURE, otherwise.
 */
API_RESULT lmp_get_pm_addr(UCHAR *ret_pm_addr, UINT16 phy_piconet_id)
{
    UINT16 ce_index;
    UCHAR pm_addr;
    LMP_CONNECTION_ENTITY *ce_ptr;

    if (phy_piconet_id >= LMP_MAX_PICONETS_SUPPORTED)
    {
        return API_FAILURE;
    }

    pm_addr = 1;
    while(1)
    {
        for(ce_index = 0; ce_index < LMP_MAX_CE_DATABASE_ENTRIES; ce_index++)
        {
            ce_ptr = &lmp_connection_entity[ce_index];

            if (ce_ptr->entity_status == ASSIGNED &&
                    ce_ptr->pm_addr == pm_addr &&
                    ce_ptr->phy_piconet_id == phy_piconet_id)
            {
                pm_addr += 1;
                if (pm_addr == 0)
                {
                    return API_FAILURE;
                }
                ce_index = (UINT16)-1;
                continue;
            }
        }
        *ret_pm_addr = pm_addr;
        return API_SUCCESS;
    }
}

/**
 * Release the \a pm_addr (Parked Mode Address).
 *
 * \param pm_addr The Parked Mode Address.
 * \param log_piconet_id The Physical Piconet ID.
 *
 * \return API_SUCCESS, if the operation is successful. API_FAILURE, otherwise.
 */
API_RESULT lmp_put_pm_addr(UCHAR pm_addr, UINT16 phy_piconet_id)
{
    UINT16 ce_index;
    LMP_CONNECTION_ENTITY *ce_ptr;

    if (phy_piconet_id >= LMP_MAX_PICONETS_SUPPORTED)
    {
        return API_FAILURE;
    }

    for(ce_index = 0; ce_index < LMP_MAX_CE_DATABASE_ENTRIES; ce_index++)
    {
        ce_ptr = &lmp_connection_entity[ce_index];

        if (ce_ptr->entity_status == ASSIGNED &&
                ce_ptr->pm_addr == pm_addr &&
                ce_ptr->phy_piconet_id == phy_piconet_id)
        {
            ce_ptr->pm_addr = 0;
            return API_SUCCESS;
        }
    }
    return API_FAILURE;
}

/**
 * Allocate a free AR_ADDR.
 *
 * \param ar_addr The Access Request Address.
 * \param log_piconet_id The Physical Piconet ID.
 *
 * \return API_SUCCESS, if the operation is successful. API_FAILURE, otherwise.
 */
API_RESULT lmp_get_ar_addr(UCHAR *ret_ar_addr, UINT16 phy_piconet_id)
{
    UINT16 ce_index;
    UCHAR ar_addr;
    LMP_CONNECTION_ENTITY *ce_ptr;

    if (phy_piconet_id >= LMP_MAX_PICONETS_SUPPORTED)
    {
        return API_FAILURE;
    }

    ar_addr = 1;

    for(ce_index = 0; ce_index < LMP_MAX_CE_DATABASE_ENTRIES; ce_index++)
    {
        ce_ptr = &lmp_connection_entity[ce_index];

        if (ce_ptr->entity_status == ASSIGNED &&
                ce_ptr->ar_addr == ar_addr &&
                ce_ptr->phy_piconet_id == phy_piconet_id)
        {
            ar_addr += 1;
            if (ar_addr == 0)
            {
                return API_FAILURE;
            }
            ce_index = (UINT16)-1;
            continue;
        }
    }
    *ret_ar_addr = ar_addr;
    return API_SUCCESS;
}


/**
 * Release the \a ar_addr (Access Request Address).
 *
 * \param ar_addr The Access Request Address.
 * \param phy_piconet_id The Physical Piconet ID.
 *
 * \return API_SUCCESS, if the operation is successful. API_FAILURE, otherwise.
 */
API_RESULT lmp_put_ar_addr(UCHAR ar_addr, UINT16 phy_piconet_id)
{
    UINT16 ce_index;
    LMP_CONNECTION_ENTITY *ce_ptr;

    if (phy_piconet_id >= LMP_MAX_PICONETS_SUPPORTED)
    {
        return API_FAILURE;
    }

    for(ce_index = 0; ce_index < LMP_MAX_CE_DATABASE_ENTRIES; ce_index++)
    {
        ce_ptr = &lmp_connection_entity[ce_index];

        if (ce_ptr->entity_status == ASSIGNED &&
                ce_ptr->ar_addr == ar_addr &&
                ce_ptr->phy_piconet_id == phy_piconet_id)
        {
            ce_ptr->ar_addr = 0;
            return API_SUCCESS;
        }
    }
    return API_FAILURE;
}
#endif /* COMPILE_PARK_MODE */

/**
* Allocate a free AM_ADDR.
*
* \param am_addr Returned Active Member Address.
* \param log_piconet_id Logical Piconet ID.
*
* \return API_SUCCESS, if the operation is successful. API_FAILURE, otherwise.
*/
API_RESULT lmp_get_am_addr_ppi(UCHAR* am_addr, UINT16 phy_piconet_id)
{
    UCHAR index;
    LMP_AM_ADDR_TO_CE_INDEX_TABLE *table;

    if (phy_piconet_id < LMP_MAX_PICONETS_SUPPORTED)
    {
        for (index = 0; index < (LC_MAX_AM_ADDR - 1); index++)
        {
            table = &lmp_am_addr_to_ce_index_table_ppi[index][phy_piconet_id];
            if (table->status == UNASSIGNED)
            {
                table->status = ASSIGNED;
                *am_addr = (UCHAR)(index + 1);

                return API_SUCCESS;
            }
        }
    }

    return API_FAILURE;
}


/**
* Releases the \a am_addr.
*
* \param am_addr The Active Member Address.
* \param log_piconet_id The Logical Piconet ID.
*
* \return API_SUCCESS, if the operation is successful. API_FAILURE, otherwise.
*/
API_RESULT lmp_put_am_addr_ppi(UCHAR am_addr, UINT16 phy_piconet_id)
{
    LMP_AM_ADDR_TO_CE_INDEX_TABLE *table;

    if ((phy_piconet_id >= LMP_MAX_PICONETS_SUPPORTED) ||
        (am_addr == 0) || (am_addr >= LC_MAX_AM_ADDR))
    {
        return API_FAILURE;
    }

    table = &lmp_am_addr_to_ce_index_table_ppi[am_addr-1][phy_piconet_id];

    if (table->status == ASSIGNED)
    {
        table->status = UNASSIGNED;
        return API_SUCCESS;
    }
    else
    {
        return API_FAILURE; /* Already released */
    }
}

/**
* Marks the am_addr and phy_piconet_id combination as used.
*
* \param am_addr The am_addr of the connection.
* \param phy_piconet_id Phy piconet ID of the connection.
*
* \return None.
*/
void lmp_slave_use_am_addr_ppi(UCHAR am_addr, UINT16 phy_piconet_id,
                               UINT16 acl_ce_index)
{
    LMP_AM_ADDR_TO_CE_INDEX_TABLE *table;

    if ((phy_piconet_id >= LMP_MAX_PICONETS_SUPPORTED) ||
        (am_addr == 0) || (am_addr >= LC_MAX_AM_ADDR))
    {
        return;
    }

    table = &lmp_am_addr_to_ce_index_table_ppi[am_addr-1][phy_piconet_id];

    //if (table->status == UNASSIGNED)
    {
        table->status = ASSIGNED;
        table->ce_index = lmp_am2ce_mangle_acl_ce_index(acl_ce_index);
    }
    return;
}

/**
* Marks the am_addr and phy_piconet_id combination as unused.
*
* \param am_addr The am_addr of the connection.
* \param phy_piconet_id Phy piconet ID of the connection.
*
* \return None.
*/
void lmp_slave_unuse_am_addr_ppi(UCHAR am_addr, UINT16 phy_piconet_id)
{
    LMP_AM_ADDR_TO_CE_INDEX_TABLE *table;

    if ((phy_piconet_id >= LMP_MAX_PICONETS_SUPPORTED) ||
        (am_addr == 0) || (am_addr >= LC_MAX_AM_ADDR))
    {
        return;
    }

    table = &lmp_am_addr_to_ce_index_table_ppi[am_addr-1][phy_piconet_id];

    //if ((table->status == ASSIGNED) &&
    //    lmp_am2ce_is_acl_ce_index(table->ce_index))
    {
        table->status = UNASSIGNED;
    }
}

#ifdef COMPILE_ESCO
/**
* Marks the esco_am_addr and phy_piconet_id combination as used.
*
* \param esco_am_addr The am_addr of the connection.
* \param phy_piconet_id Phy piconet ID of the connection.
*
* \return None.
*/
void lmp_slave_use_esco_am_addr_ppi(UCHAR esco_am_addr, UINT16 phy_piconet_id,
                                    UINT16 esco_ce_index)
{
    LMP_AM_ADDR_TO_CE_INDEX_TABLE *table;

    if ((phy_piconet_id >= LMP_MAX_PICONETS_SUPPORTED) ||
        (esco_am_addr == 0) || (esco_am_addr >= LC_MAX_AM_ADDR))
    {
        return;
    }

    table = &lmp_am_addr_to_ce_index_table_ppi[esco_am_addr-1][phy_piconet_id];

    //if (table->status == UNASSIGNED)
    {
        table->status = ASSIGNED;
        table->ce_index = lmp_am2ce_mangle_esco_ce_index(esco_ce_index);
    }
}

/**
* Marks esco_the am_addr and phy_piconet_id combination as unused.
*
* \param esco_am_addr The am_addr of the connection.
* \param phy_piconet_id Phy piconet ID of the connection.
*
* \return None.
*/
void lmp_slave_unuse_esco_am_addr_ppi(UCHAR esco_am_addr, UINT16 phy_piconet_id)
{
    LMP_AM_ADDR_TO_CE_INDEX_TABLE *table;

    if ((phy_piconet_id >= LMP_MAX_PICONETS_SUPPORTED) ||
        (esco_am_addr == 0) || (esco_am_addr >= LC_MAX_AM_ADDR))
    {
        return;
    }

    table = &lmp_am_addr_to_ce_index_table_ppi[esco_am_addr-1][phy_piconet_id];

    //if ((table->status == ASSIGNED) &&
    //    lmp_am2ce_is_esco_ce_index(table->ce_index))
    {
        table->status = UNASSIGNED;
    }
}
#endif /* COMPILE_ESCO */

#ifdef COMPILE_SNIFF_MODE
#ifndef _CCH_SLOT_OFFSET_
/**
 * Obtain the slot offset from the derived clock value and time interval.
 *
 * \param time_interval Time interval in terms of slots.
 * \param init_proc Initialization procedure used in timing control flags.
 * \param slot_offset The Slot offset vale (Dsniff).
 * \param remote_dev_role Role of the remote device.
 * \param piconet_id The Logical Piconet ID.
 *
 * \return API_SUCCESS, if the operation is successful. API_FAILURE, otherwise.
 */
API_RESULT lmp_get_slot_offset(UINT16 ce_index, UCHAR *init_proc)
{
    UINT32 clock_value;
    UINT16 time_interval;
    UINT16 *slot_offset;
    UCHAR remote_dev_role;
    UINT16 phy_piconet_id;

    time_interval = lmp_connection_entity[ce_index].sniff_interval;

    slot_offset = &(lmp_connection_entity[ce_index].sniff_slot_offset);
    remote_dev_role = lmp_connection_entity[ce_index].remote_dev_role;
    phy_piconet_id = lmp_connection_entity[ce_index].phy_piconet_id;

    /* Extract the value of the current clock value from the BB registers */
    lc_get_clock_in_scatternet(&clock_value, (UCHAR)phy_piconet_id);

    /* If the MSB ie clock_value(26) bit is 0 follow INIT 1 else follow INIT 2*/
    *init_proc = (UCHAR)((clock_value & 0x08000000) >> 27);

    clock_value >>= 1;
    /* Calculate the slot offset by taking only the first 25 bits that is
    * clock_value(0) to clock_value(25)
    */
    *slot_offset = (UINT16)(clock_value % time_interval);

    /* To Check is it is valid to give 0 all the time */
    *slot_offset = 0;

    return API_SUCCESS;
}
#endif
#endif /* COMPILE_SNIFF_MODE */

/**
 * Add the device information to the device cache table.
 *
 * \param device_info Device information.
 *
 * \return API_SUCCESS, if the operation is successful. API_FAILURE, otherwise.
 */
API_RESULT lmp_add_to_device_cache(DEVICE_INFO device_info)
{
    CACHE_TABLE *node = NULL;
    CACHE_TABLE *temp_node = NULL;
    UCHAR start = 0;
    UCHAR index;
    UCHAR temp_cache_start = 0;
    UCHAR temp_cache_free = 0;
    UCHAR last_node = 0;

    if ((bd_addr_cache_free == INVALID_INDEX) ||
        (bd_addr_cache_start == INVALID_INDEX))
    {
        LMP_ERR(CACHE_TABLE_IS_NOT_PROPER,0,0);
        return API_FAILURE;
    }

    /* Check if the BD Address is already in the cache table */
    temp_node = &bd_addr_cache[bd_addr_cache_start];
    temp_cache_start = bd_addr_cache_start;

    while((temp_node->state == CACHE_TAKEN) )
    {
        if (memcmp(temp_node->bd_addr,device_info.bd_addr,
                  LMP_BD_ADDR_SIZE) == 0)
        {
            node = temp_node;
            break;
        }

        if (temp_node->next_index == INVALID_INDEX)
        {
            /*If we have reached the end of the list */
            break;
        }
        else
        {
            /* Move to the next Node */
            temp_node = &bd_addr_cache[temp_node->next_index];
            start ++;
        }
    }

    /* If the BD address does not exist , Select a new node to add */
    if (node == NULL)
    {
        node = &bd_addr_cache[bd_addr_cache_free];
    }

    /* Update the Device Info */
    memcpy(node->bd_addr, device_info.bd_addr, LMP_BD_ADDR_SIZE);
    node->clock_offset = device_info.clock_offset;
    node->page_mode = device_info.page_mode;
    node->page_mode_settings = device_info.page_mode_settings;
    node->state = CACHE_TAKEN;

    temp_cache_start = node->cur_index;

    if (start == 0)
    {
        /* If first node do nothing */
    }
    else if ((start == MAX_CACHE - 1) || (last_node == 1))
    {
        bd_addr_cache[node->prev_index].next_index = INVALID_INDEX;
        node->next_index = bd_addr_cache[bd_addr_cache_start].cur_index;
        bd_addr_cache[bd_addr_cache_start].prev_index = node->cur_index;
        node->prev_index = INVALID_INDEX;
    }
    else
    {
        bd_addr_cache[node->prev_index].next_index =
            bd_addr_cache[node->next_index].cur_index;
        bd_addr_cache[node->next_index].prev_index =
            bd_addr_cache[node->prev_index].cur_index;
        node->next_index = bd_addr_cache[bd_addr_cache_start].cur_index;
        bd_addr_cache[bd_addr_cache_start].prev_index = node->cur_index;
        node->prev_index = INVALID_INDEX;
    }

    /* Update the next free node */
    temp_cache_free = temp_cache_start;
    index = 0;
    temp_node = &bd_addr_cache[temp_cache_start];

    while(temp_node->state == CACHE_TAKEN)
    {
        if (index == MAX_CACHE - 1)
        {
            last_node = 1;
            break;
        }
        index ++;
        temp_node = &bd_addr_cache[temp_node->next_index];
    }

    temp_cache_free = temp_node->cur_index;
    bd_addr_cache_free = temp_cache_free;
    bd_addr_cache_start = temp_cache_start;
    return API_SUCCESS;
}

/**
 * Get the device information associated with the BD Address.
 *
 * \param bd_addr BD_ADDR of the device.
 * \param device_info Device information corresponding to the BD_ADDR.
 *
 * \return API_SUCCESS, if the operation is successful. API_FAILURE, otherwise.
 */
API_RESULT lmp_get_device_info(UCHAR *bd_addr, DEVICE_INFO *device_info)
{

    CACHE_TABLE *node;

    if (device_info == NULL)
    {
        LMP_ERR(INVALID_DEVICE_INFO_POINTER,0,0);
        return API_FAILURE;
    }

    node = &bd_addr_cache[bd_addr_cache_start];

    while(node->state == CACHE_TAKEN)
    {
        if (memcmp(bd_addr, node->bd_addr, LMP_BD_ADDR_SIZE) == 0)
        {
            memcpy(device_info->bd_addr, node->bd_addr, LMP_BD_ADDR_SIZE);
            device_info->clock_offset = node->clock_offset;
            device_info->page_mode = node->page_mode;
            device_info->page_mode_settings  = node->page_mode_settings;
            return API_SUCCESS;
        }

        if (node->next_index == INVALID_INDEX)
        {
            break;
        }
        else
        {
            node = &bd_addr_cache[node->next_index];
        }
    }

    return API_FAILURE;
}


#ifdef COMPILE_PARK_MODE
/**
 * Start the lmp_park_mode_timer. Timeout handler: lmp_park_mode_timer_handler.
 *
 * \param ce_index Connection entity index.
 *
 * \return None.
 */
void lmp_start_park_mode_timer(UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY *ce_ptr;
    UINT16 timer_val;
    ce_ptr = &lmp_connection_entity[ce_index];

    timer_val = (UINT16) ((ce_ptr->Tpoll) * 6);
    timer_val = SLOT_VAL_TO_TIMER_VAL(timer_val);

    if (ce_ptr->park_mode_timer_handle == NULL)
    {
        if (OS_CREATE_TIMER(ONESHOT_TIMER, &ce_ptr->park_mode_timer_handle,
                lmp_park_mode_timer_handler, (void *)((UINT32)ce_index), timer_val)
                != BT_ERROR_OK )
        {
            LMP_ERR(TIMER_CREATION_FAILED_1,0,0);
        }
    }

    if (OS_START_TIMER(ce_ptr->park_mode_timer_handle, timer_val) != BT_ERROR_OK)
    {
        LMP_ERR(OS_START_TIMER_FAILED,0,0);
    }

    return;
}
#endif

#ifdef COMPILE_SNIFF_MODE
/**
 * Programs the sniff mode to the LMP. This will be called
 * when LMP level negotiation of sniff mode parameters is completed.
 *
 * \param ce_index Index to lmp-connection-entity database for the connection
 *
 * \return API_SUCCESS on successful programming.
 */
API_RESULT lmp_start_sniff_mode(UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];
    lmp_set_ce_status(ce_index, LMP_SNIFF_MODE);

#ifdef DETECT_SNIFF_OVERLAP
    lmp_self_device_data.no_of_sniff_connections++;
#endif

    ce_ptr->in_sniff_mode = TRUE;

#ifdef _CCH_SNIFF_NEG_TIMEOUT_
    ce_ptr->sniff_neg_count = 0x00;
#endif

    lc_start_sniff_mode(ce_index, 0);

    lmp_self_device_data.num_of_sniff_connections++;

#ifdef ENABLE_SCO
    lmp_determine_full_bandwidth();
#endif

#if 0 /* (josh) diable following code due to lmp_send_max_slot_pdu_to_all_devices() already done in above lc_start_sniff_mode(ce_index, 0) */
#ifndef COMPILE_SINGLE_SLOT_PACKETS_ONLY
    lmp_send_max_slot_pdu_to_all_devices();
#endif
#endif

    lc_update_pkts_allowed(ce_index);

    return API_SUCCESS;
}

/**
 * Programs the LMP to exit from sniff mode. This will be called
 * when LMP level negotiation of exit-sniff omde is completed successfully.
 *
 * \param am_addr Logical transport address of the connection
 * \param piconet_id Physical piconet ID of the connection.
 *
 * \return API_SUCCESS on success.
 */
API_RESULT lmp_exit_sniff_mode(UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY *ce_ptr;
    UCHAR lut_index= 0;
    UCHAR phy_piconet_id;

#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8723_RCP_
    if (rcp_lmp_exit_sniff_mode != NULL)
    {
        if ( rcp_lmp_exit_sniff_mode((void*)(&ce_index)) )
        {
            return API_SUCCESS;
        }
    }
#endif
#endif

    ce_ptr = &lmp_connection_entity[ce_index];
    phy_piconet_id = ce_ptr->phy_piconet_id;

    lut_index = lc_get_lut_index_from_phy_piconet_id(
                    ce_ptr->am_addr, ce_ptr->phy_piconet_id);

    ce_ptr->temp_ce_status = LMP_CONNECTED;


#ifndef _CCH_RTL8723A_B_CUT
    lmp_set_ce_status(ce_index, LMP_CONNECTED);
#else
// _CCH_8723_A_ECO_
    // cch
    // 20110715 (fw bug)
    // for IOT ISSC eSCO + Sniff/Unsniff ce_status error
    // Need Return 1;
    // to replace the whole function

    if( ce_ptr->ce_status ==  LMP_SNIFF_MODE)
    {
        lmp_set_ce_status(ce_index, LMP_CONNECTED);
    }
    else if( ce_ptr->ce_status ==  LMP_ADDING_SCO_LINK_DURING_SNIFF)
    {
        lmp_set_ce_status(ce_index, LMP_ADDING_SCO_LINK);
    }
    else if( ce_ptr->ce_status == LMP_ADDING_ESCO_LINK)
    {
        lmp_set_ce_status(ce_index, LMP_ADDING_ESCO_LINK);
    }
#endif

    ce_ptr->in_sniff_mode = FALSE;

#ifdef _CCH_SNIFF_NEG_TIMEOUT_
    ce_ptr->sniff_neg_count = 0x00;
#endif

    lc_exit_sniff_mode(ce_index);

    if (ce_ptr->low_power_disconnect_state != UNSNIFF_DISCONNECT)
    {

#if 0 /* (josh) diable following code due to lmp_send_max_slot_pdu(ce_index) already done in above lc_exit_sniff_mode(ce_index) */
#ifndef COMPILE_SINGLE_SLOT_PACKETS_ONLY
        lmp_send_max_slot_pdu(ce_index);
#endif
#endif
        lc_update_pkts_allowed(ce_index);

#ifndef COMPILE_SINGLE_SLOT_PACKETS_ONLY
        lmp_decide_and_update_max_slot_req(ce_index);
        lmp_send_max_slot_req_pdu(ce_index, 0xff);
#endif
    }

    /* Reset SSR Variables */
    lmp_reset_ssr_parameters(ce_index);

    return API_SUCCESS;
}
#endif


#if defined(ENABLE_SCO) && defined(COMPILE_SNIFF_MODE)
/**
 * Determines whether the bandwidth of the device is completely occupied
 * or not and sets various full bandwidth flags accordingly. This function has
 * to be called whenever a portion of the bandwidth is reserved or freed (eg.
 * after establishing/killing a SCO link, after entering/exiting into/from
 * sniff mode, etc.).
 *
 * \param None.
 *
 * \return None.
 */
void lmp_determine_full_bandwidth(void)
{
    UCHAR num_sco_conns;
    UINT16 sco_pkt_type;
    DEF_CRITICAL_SECTION_STORAGE;

    MINT_OS_ENTER_CRITICAL();

    lc_acl_full_bandwidth_flag = FALSE;

    lc_full_bandwidth_flag = FALSE;

    num_sco_conns = lmp_self_device_data.total_no_of_sco_conn;
    sco_pkt_type = lmp_self_device_data.sco_pkt_type;

    /* Determine whether the entire physical channel is occupied by the SCO
     * logical transports.
     */
    switch (num_sco_conns)
    {
        case 1:
            if (sco_pkt_type == HV1)
            {
                lc_full_bandwidth_flag = TRUE;
            }
            break;

        case 2:
            if (sco_pkt_type == HV2)
            {
                lc_full_bandwidth_flag = TRUE;
            }
            break;

        case 3:
            lc_full_bandwidth_flag = TRUE;
            break;

        default:
            /* Invalid case. */
            break;
    }

    if (lc_full_bandwidth_flag == TRUE)
    {
        /* The entire physical channel is occupied and it implies that
         * the entire ACL bandwidth is also occupied.
         */
        lc_acl_full_bandwidth_flag = TRUE;
    }
#ifdef COMPILE_SNIFF_MODE
    else
    {
        int ce_index;
        LMP_CONNECTION_ENTITY* ce_ptr;

        /* When in sniff mode the available ACL bandwidth is only the sniff
         * window. If the sniff window is completely overlapped with the SCO
         * slots then we would have no ACL bandwidth left even though the
         * entire physical channel is not occupied.
         * Lets find if we have some spare ACL bandwidth.
         */
        for (ce_index = 0; ce_index < LMP_MAX_CE_DATABASE_ENTRIES; ce_index++)
        {
            ce_ptr = &lmp_connection_entity[ce_index];

            /* If we have at least a single ACL link in sniff mode and its
             * sniff windows is completely occupied, the ACL link might not
             * have any ACL bandwidth at all.
             */
            if (ce_ptr->entity_status == TRUE
                    && ce_ptr->in_sniff_mode == TRUE
                    && ce_ptr->sniff_attempt < 3)
            {

#ifndef _CCH_NO_PAUSE_SCO_
                if (num_sco_conns > 0)
#else
		   if (  (num_sco_conns > 0) && (lmp_self_device_data.number_of_acl_conn > 3) )
#endif
                {
                    lc_acl_full_bandwidth_flag = TRUE;
                    break;
                }
            } /* end if (connection_in_sniff && sniff_attempts < 3) */
        } /* end for(each_connection_entity) */
    }
#endif /* COMPILE_SNIFF_MODE */

    if (lc_acl_full_bandwidth_flag == TRUE)
    {
        BB_set_slave_full_bandwidth_flag();
        if (lmp_self_device_data.number_of_acl_conn > 3)
        {
            BB_set_multi_slave_full_bandwidth_flag();
        }
        else
        {
            BB_set_single_slave_full_bandwidth_flag();
        }
    }
    else
    {
#ifdef DISABLE_SLAVE_SCO_OVERRIDE
        BB_clear_slave_full_bandwidth_flag();
#endif
        BB_clear_single_slave_full_bandwidth_flag();
        BB_clear_multi_slave_full_bandwidth_flag();
    }

    MINT_OS_EXIT_CRITICAL();

#ifdef ENABLE_LOGGER_LEVEL_2
    LMP_LOG_INFO(LOG_LEVEL_LOW, LC_FULL_BANDWIDTH_FLAG, 1, lc_full_bandwidth_flag);
    LMP_LOG_INFO(LOG_LEVEL_LOW, LC_ACL_FULL_BANDWIDTH_FLAG, 1, lc_acl_full_bandwidth_flag);
    LMP_LOG_INFO(LOG_LEVEL_LOW, SCO_PKT_TYPE_REG, 1,
                 BB_read_baseband_register(SCO_PACKET_TYPE_REGISTER));
    LMP_LOG_INFO(LOG_LEVEL_LOW, SYNC_FIFO_CONFIG_REG, 1,
                 BB_read_baseband_register(SYNC_FIFO_CONFIG_REGISTER));
#endif

    return;
}
#endif /* ENABLE_SCO && COMPILE_SNIFF_MODE */

/**
 * Accepts the ACL connection request as slave. It also initiates either the
 * connection completion procedure or authentication procedure.
 *
 * \param ce_index ACL connection entity index.
 *
 * \return None.
 */
void lmp_accept_host_connection_request_pdu(UINT16 ce_index)
{
    /* As slave, we have decided to accept the ACL connection and sent
     * accepted PDU. (Role-Switch could have also happened)
     */
    lmp_send_lmp_accepted(ce_index, LMP_HOST_CONNECTION_REQ_OPCODE,
                          MASTER_TID, LMP_NO_STATE_CHANGE);

    /* Before completing the connection establishment procedure with
     * LMP_setup_complete_pdu, let's check whether we need to initiate
     * authentication or not (eg. the host has set auth_enable = TRUE).
     * If we start authentication procedure, we have to wait until it
     * completes before proceeding with the connection completion
     * procedure.
     */
    if (!bz_auth_initiate_authentication_during_connection(ce_index))
    {
        /* Host hasn't asked us to perform authentication during connection
         * (or SSP is allowed at both sides and we can't initiate
         * authentication during connection), so let's march towards
         * completing the ACL connection establishment procedure.
         */
        lmp_decide_to_send_conn_complete_evt(ce_index);
        lmp_connection_entity[ce_index].lmp_expected_pdu_opcode |=
            lmp_get_opcode_mask( LMP_SETUP_COMPLETE_OPCODE, 0x0);
    }
}

