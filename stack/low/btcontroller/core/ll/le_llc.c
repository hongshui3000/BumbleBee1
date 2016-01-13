enum { __FILE_NUM__= 204 };

#include "le_ll.h"
#include "le_ll_driver.h"
#include "bt_fw_types.h"
#include "mem.h"
#include "bt_fw_acl_q.h"
#include "bt_fw_hci_spec_defines.h"
#include "bt_fw_hci_internal.h"
#include "common_utils.h"
#include "lmp.h"
#include "mint_os.h"
#include "le_hci_4_0.h"

#ifdef _ROM_CODE_PATCHED_
PF_ROM_CODE_PATCH_FUNC rcp_ll_update_instant_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_llc_sent_llc_pdu_ack_recd_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_ll_generate_conn_para_req_func = NULL;
#endif

extern OS_HANDLE isr_extended_task_handle;

/**************************************************************************
 * Function     : llc_timeout_callback
 *
 * Description  : This function is a callback function when llc procedure timer
 *                or terminated timer is expired.
 *
 * Parameters   : timer_handle: the ID of llc procedure or terminated timer
 *                index: the argument of callback function
 *
 * Returns      : None
 *
 *************************************************************************/
void llc_timeout_callback(TimerHandle_t timer_handle)
{
    UINT32 temp = (UINT32)pvTimerGetTimerID(timer_handle);
    UINT8 conn_entry = temp & 0xFF;
    LL_CONN_HANDLE_UNIT *phandle;

    if (timer_handle == NULL)
    {
        return;
    }

	DEF_CRITICAL_SECTION_STORAGE;
    MINT_OS_ENTER_CRITICAL();

    phandle = &ll_manager.conn_unit.handle[conn_entry];

    /* delete timer in invalid argument */
    if ((!phandle->connected) || (phandle->llc_timer != timer_handle))
    {
        OS_DELETE_TIMER(&timer_handle);
        MINT_OS_EXIT_CRITICAL();
        return;
    }

#ifdef USE_FREERTOS
    OS_STOP_TIMER(phandle->llc_timer, NULL);
    OS_STOP_TIMER(phandle->sup_timer, NULL);
#else
    /* free software timer */
    if (phandle->llc_timer != NULL)
    {
        OS_DELETE_TIMER(&phandle->llc_timer);
    }

    /* free supervision timer */
    if (phandle->sup_timer != NULL)
    {
        OS_DELETE_TIMER(&phandle->sup_timer);
    }
#endif

    phandle->kill_link_reason = LL_KILL_REASON_LLC_TIMEOUT;

    switch (phandle->llc.type)
    {
        case LLC_PROCEDURE_TYPE_FEATURE_EXCHANGE:
            /* Send remote used features complete Event */
            hci_generate_le_read_remote_used_features_complete_event(
                                       LL_RESPONSE_TIMEOUT_ERROR, phandle);
            break;

        case LLC_PROCEDURE_TYPE_VERSION_EXCHANGE:
            /* Send remote version information complete Event */
            hci_generate_remote_version_information_complete_event(
                            LL_RESPONSE_TIMEOUT_ERROR, phandle->conn_handle,
                            phandle->unit_id + LMP_MAX_CE_DATABASE_ENTRIES);
            break;

        case LLC_PROCEDURE_TYPE_ENCRYPTION_START:
            if (ll_manager.conn_unit.master)
            {
                UCHAR event_parameter[4];
                /* Send hci encryption change Event */
                event_parameter[0] = LL_RESPONSE_TIMEOUT_ERROR;
                event_parameter[1] = phandle->conn_handle & 0xFF;
                event_parameter[2] = phandle->conn_handle >> 8;
                event_parameter[3] = 0;
                hci_generate_event(HCI_ENCRYPTION_CHANGE_EVENT, event_parameter, 4);
            }
            break;

        case LLC_PROCEDURE_TYPE_ENCRYPTION_PAUSE:
            {
                UCHAR event_parameter[HCI_ENCRYPTION_KEY_REFRESH_COMPLETE_EVENT_LEN];
                /* Send hci encryption key refresh Event */
                event_parameter[0] = LL_RESPONSE_TIMEOUT_ERROR;
                event_parameter[1] = phandle->conn_handle & 0xFF;
                event_parameter[2] = phandle->conn_handle >> 8;
                hci_generate_event(HCI_ENCRYPTION_KEY_REFRESH_COMPLETE_EVENT,
                        event_parameter, HCI_ENCRYPTION_KEY_REFRESH_COMPLETE_EVENT_LEN);
            }
            break;

        case LLC_PROCEDURE_TYPE_LE_PING_REQ:
            break;
    }

    /* kill the connection */
    ll_driver_kill_connection(conn_entry);

    MINT_OS_EXIT_CRITICAL();

    LL_LOG_TRACE(RED, LE_MSG_LLC_PROCEDURE_TIMEOUT, 1, conn_entry);
}

#ifdef _BT4_1_LE_PING_FEATURE_SUPPORT_
/**************************************************************************
 * Function     : llc_ping_req_callback
 *
 * Description  : This function is a callback function when llc ping timer
 *                is expired (timeout = auth_payload_timeout - wait_rsp_time).
 *
 * Parameters   : timer_handle: the ID of llc procedure or terminated timer
 *                index: the argument of callback function
 *
 * Returns      : None
 *
 *************************************************************************/
void llc_ping_req_callback(TIMER_ID timer_handle, void *index)
{
    UINT32 temp = (UINT32)index;
    UINT8 conn_entry = temp & 0xFF;
    LL_CONN_HANDLE_UNIT *phandle;

    if (timer_handle == OS_INVALID_HANDLE)
    {
        return;
    }

    phandle = &ll_manager.conn_unit.handle[conn_entry];

    /* delete timer in invalid argument */
    if ((!phandle->connected) || (phandle->ping_req_timer != timer_handle))
    {
        OS_DELETE_TIMER(&timer_handle);
        return;
    }

    /* use the LLC procedure */
    if (phandle->llc.type != LLC_PROCEDURE_TYPE_NONE)
    {
        /* we need to wait the completion of previous LLC procedure */
        phandle->llc.pend_ping_req = TRUE;
    }
    else
    {
        llc_schedule_le_ping_procedure(phandle);
    }
}

/**************************************************************************
 * Function     : llc_ping_req_restart_timer
 *
 * Description  : This function is used for restarting ping timer.
 *
 * Parameters   : entry: connectin entry
 *
 * Returns      : None
 *
 *************************************************************************/

void llc_ping_req_restart_timer(UINT8 entry)
{
    LL_CONN_HANDLE_UNIT *pHandle = &ll_manager.conn_unit.handle[entry];

    /* check ping timer is being used or not */
    if (pHandle->ping_req_timer != OS_INVALID_HANDLE)
    {
        /* stop the timer if we have the body of supervision timer */
        OS_DELETE_TIMER(&pHandle->ping_req_timer);
    }

    /* create a timer for ping and install the callback function */
    OS_CREATE_TIMER((UCHAR)ONESHOT_TIMER, &pHandle->ping_req_timer,
                   (OS_TIMEOUT_HANDLER)llc_ping_req_callback,
                   (void *)((UINT32)entry), 0);

    /* start a ping request timer */
    OS_START_TIMER(pHandle->ping_req_timer, (pHandle->auth_payload_timeout - pHandle->auth_payload_timeout / 10) * 10);
}

/**************************************************************************
 * Function     : llc_ping_wait_rsp_callback
 *
 * Description  : This function is a callback function when llc ping timer
 *                is expired (timeout = wait ping rsp time).
 *
 * Parameters   : timer_handle: the ID of llc procedure or terminated timer
 *                index: the argument of callback function
 *
 * Returns      : None
 *
 *************************************************************************/
void llc_ping_wait_rsp_callback(TIMER_ID timer_handle, void *index)
{
    UINT32 temp = (UINT32)index;
    UINT8 conn_entry = temp & 0xFF;
    LL_CONN_HANDLE_UNIT *phandle;

    if (timer_handle == OS_INVALID_HANDLE)
    {
        return;
    }

    phandle = &ll_manager.conn_unit.handle[conn_entry];

    /* delete timer in invalid argument */
    if ((!phandle->connected) || (phandle->ping_req_timer != timer_handle))
    {
        OS_DELETE_TIMER(&timer_handle);
        return;
    }

    hci_generate_authentication_payload_timeout_expired_event(phandle->conn_handle);

    ll_start_timer(conn_entry, LL_TIMER_TYPE_PING);
}

/**************************************************************************
 * Function     : llc_ping_start_wait_rsp_timer
 *
 * Description  : This function is used for starting wait rsp timer.
 *
 * Parameters   : entry: connectin entry
 *
 * Returns      : None
 *
 *************************************************************************/
void llc_ping_start_wait_rsp_timer(UINT8 entry)
{
    LL_CONN_HANDLE_UNIT *pHandle = &ll_manager.conn_unit.handle[entry];

    /* check ping timer is being used or not */
    if (pHandle->ping_req_timer != OS_INVALID_HANDLE)
    {
        /* stop the timer if we have the body of supervision timer */
        OS_DELETE_TIMER(&pHandle->ping_req_timer);
    }

    /* create a timer for ping and install the callback function */
    OS_CREATE_TIMER((UCHAR)ONESHOT_TIMER, &pHandle->ping_req_timer,
                   (OS_TIMEOUT_HANDLER)llc_ping_wait_rsp_callback,
                   (void *)((UINT32)entry), 0);

    /* start a ping request timer */
    OS_START_TIMER(pHandle->ping_req_timer, pHandle->auth_payload_timeout);
}
#endif /* _BT4_1_LE_PING_FEATURE_SUPPORT_ */

/**************************************************************************
 * Function     : llc_schedule_encryption_procedure
 *
 * Description  : This function is used to schedule start or restart encryption
 *                procedure. It shall be used in master role.
 *
 * Parameters   : pHandle: the pointer of connection entity
 *
 * Returns      : TRUE is Successfully / FALSE is Failure
 *
 *************************************************************************/
UINT8 llc_schedule_encryption_procedure(LL_CONN_HANDLE_UNIT *pHandle)
{
    LL_CTRL_PDU_PAYLOAD *pTxPkt;
    UINT8 entry = pHandle->unit_id;

    /* pause acl data path */
    pHandle->pause_acl_tx = TRUE;

    /* we can start to send LL_ENC_REQ to slave now */
    if (pHandle->encrypted)
    {
        /* restart encryption */
        pTxPkt = llc_generate_ll_pause_enc_req();
        llc_append_pdu_to_tx_list(pTxPkt, entry);
        pHandle->encrypt_blk.pause_enc_state =
                        LLC_PAUSE_ENC_STATE_M2S_PAUSE_ENC_REQ;
        pHandle->llc.type = LLC_PROCEDURE_TYPE_ENCRYPTION_PAUSE;
    }
    else
    {
        /* start encryption */
        pTxPkt = llc_generate_ll_enc_req(entry);
        llc_append_pdu_to_tx_list(pTxPkt, entry);
        pHandle->llc.type = LLC_PROCEDURE_TYPE_ENCRYPTION_START;
    }
    pHandle->llc.pend_start_encrypt = FALSE;

    /* start the LLC procedure timer */
    ll_start_timer(entry, LL_TIMER_TYPE_LLC);

    return TRUE;
}


/**************************************************************************
 * Function     : llc_schedule_version_exchange_procedure
 *
 * Description  : This function is used to schedule version exchange procedure.
 *
 * Parameters   : pHandle: the pointer of connection entity
 *
 * Returns      : TRUE is Successfully / FALSE is Failure
 *
 *************************************************************************/
UINT8 llc_schedule_version_exchange_procedure(LL_CONN_HANDLE_UNIT *pHandle)
{
    UINT8 entry = pHandle->unit_id;

    pHandle->llc.sent_version_ind = TRUE;

    llc_append_pdu_to_tx_list(llc_generate_ll_version_ind(), entry);

    /* we can start to send LL_FEATURE_REQ to host now */
    pHandle->llc.type = LLC_PROCEDURE_TYPE_VERSION_EXCHANGE;
    pHandle->llc.pend_version_ex = FALSE;

    /* start the LLC procedure timer */
    ll_start_timer(entry, LL_TIMER_TYPE_LLC);

#ifdef _FIX_BQB_LL_TP_SEC_MAS_BV_12_C_ISSUE_
    if (pHandle->llc.get_version_ind == TRUE)
    {
        llc_check_sent_llc_pdu_request(pHandle, LL_VERSION_IND);
    }
#endif

    return TRUE;
}

/**************************************************************************
 * Function     : llc_schedule_feature_exchange_procedure
 *
 * Description  : This function is used to schedule feature exchange procedure.
 *                It shall be used in master role.
 *
 * Parameters   : pHandle: the pointer of connection entity
 *
 * Returns      : TRUE is Successfully / FALSE is Failure
 *
 *************************************************************************/
UINT8 llc_schedule_feature_exchange_procedure(LL_CONN_HANDLE_UNIT *pHandle)
{
    LL_CTRL_PDU_PAYLOAD *pTxPkt;
    UINT8 entry = pHandle->unit_id;
#ifdef _SUPPORT_VER_4_1_
    if (ll_manager.conn_unit.master)
    {
        pTxPkt = llc_generate_ll_feature_req();
    }
    else
    {
        pTxPkt = llc_generate_ll_slave_feature_req();
    }
#else
    pTxPkt = llc_generate_ll_feature_req();
#endif
    llc_append_pdu_to_tx_list(pTxPkt, entry);

    /* we can start to send LL_FEATURE_REQ to host now */
    pHandle->llc.type = LLC_PROCEDURE_TYPE_FEATURE_EXCHANGE;
    pHandle->llc.pend_feature_ex = FALSE;

    /* start the LLC procedure timer */
    ll_start_timer(entry, LL_TIMER_TYPE_LLC);

    return TRUE;
}

/**************************************************************************
 * Function     : llc_schedule_connection_update_procedure
 *
 * Description  : This function is used to schedule connection update
 *                procedure. It shall be used in master role.
 *
 * Parameters   : pHandle: the pointer of connection entity
 *
 * Returns      : TRUE is Successfully / FALSE is Failure
 *
 *************************************************************************/
UINT8 llc_schedule_connection_update_procedure(LL_CONN_HANDLE_UNIT *pHandle)
{
    LL_CTRL_PDU_PAYLOAD *pTxPkt;
    UINT8 entry = pHandle->unit_id;
    UINT8 result = FALSE;

    /* before previous connection update is finished, we can not do
       next connection update rocedure (hw limit) */

    if (ll_manager.conn_unit.conn_updt_entry == LL_MAX_CONNECTION_UNITS)
    {
        /* generate CONNECTION_UPDATE_REQ PDU to fire this command */
        pTxPkt = llc_generate_ll_connection_update_req(entry);
        llc_append_pdu_to_tx_list(pTxPkt, entry);

        /* we can start new LLC procedure for connection update now */
        pHandle->llc.type = LLC_PROCEDURE_TYPE_CONN_UPDT;
        pHandle->llc.pend_conn_updt = FALSE;
        ll_manager.conn_unit.conn_updt_entry = entry;

        /* TODO: fire connection update command and further handling */
        ll_driver_connection_update(entry, pHandle->conn_updt_blk.instant);

        result = TRUE;
    }
#ifdef LE_ISSUE_INSTR_IN_ORDER_ON_COMPLT
    else
    {
        pHandle->llc.pend_conn_updt = TRUE;
    }
#endif

    return result;
}

/**************************************************************************
 * Function     : llc_schedule_channel_map_update_procedure
 *
 * Description  : This function is used to schedule channel map update
 *                procedure. It shall be used in master role.
 *
 * Parameters   : pHandle: the pointer of connection entity
 *
 * Returns      : TRUE is Successfully / FALSE is Failure
 *
 *************************************************************************/
UINT8 llc_schedule_channel_map_update_procedure(LL_CONN_HANDLE_UNIT *pHandle)
{
    LL_CTRL_PDU_PAYLOAD *pTxPkt;
    UINT8 entry = pHandle->unit_id;
    UINT8 result = FALSE;

    /* before previous channel map update is finished, we can not do
       next channel map update procedure (hw limit) */

    if (ll_manager.conn_unit.chm_updt_entry == LL_MAX_CONNECTION_UNITS)
    {
        pHandle->llc.pend_ch_map_updt = FALSE;
        pHandle->llc.type = LLC_PROCEDURE_TYPE_CHMAP_UPDT;
        ll_manager.conn_unit.chm_updt_entry = entry;

        pTxPkt = llc_generate_ll_channel_map_req(entry);
        llc_append_pdu_to_tx_list(pTxPkt, entry);

        /* TODO: fire channel update command and further handling */
        ll_driver_channel_map_update(entry, pHandle->ch_map_blk.instant);

        result = TRUE;
    }
#ifdef LE_ISSUE_INSTR_IN_ORDER_ON_COMPLT
    else
    {
        pHandle->llc.pend_ch_map_updt = TRUE;
    }
#endif

    return result;
}
#ifdef _BT4_1_SUPPORT_LL_CONN_PARAM_REQ
/**************************************************************************
 * Function     : llc_schedule_connection_parameter_procedure
 *
 * Description  : This function is used to schedule connection parameter request
 *                procedure.
 *
 * Parameters   : pHandle: the pointer of connection entity
 *
 * Returns      : TRUE is Successfully / FALSE is Failure
 *
 *************************************************************************/
UINT8 llc_schedule_connection_parameter_procedure(LL_CONN_HANDLE_UNIT *pHandle)
{
    LL_CTRL_PDU_PAYLOAD *pTxPkt;
    UINT8 entry = pHandle->unit_id;
    UINT8 result = FALSE;
    if (pHandle->llc.type == LLC_PROCEDURE_TYPE_NONE)
    {
        pHandle->llc.type = LLC_PROCEDURE_TYPE_CONN_PARAM_REQ;
    }
    else
    {
        RT_BT_LOG(RED, DAPE_TEST_LOG588, 2, entry, pHandle->llc.type);
        return FALSE;
    }
    /* generate CONNECTION_PARAM_REQ PDU to fire this command */
    ll_decide_connection_parameters(entry);
    pTxPkt = llc_generate_ll_connection_parameter_req(entry);
    llc_append_pdu_to_tx_list(pTxPkt, entry);

    /* we can start new LLC procedure for connection update now */
    pHandle->llc.type = LLC_PROCEDURE_TYPE_CONN_PARAM_REQ;
    pHandle->llc.pend_conn_param_req = FALSE;

    result = TRUE;

    return result;
}
#endif

#ifdef _BT4_1_LE_PING_FEATURE_SUPPORT_
/**************************************************************************
 * Function     : llc_schedule_le_ping_procedure
 *
 * Description  : This function is used to schedule le ping procedure.
 *
 * Parameters   : pHandle: the pointer of connection entity
 *
 * Returns      : TRUE is Successfully / FALSE is Failure
 *
 *************************************************************************/
UINT8 llc_schedule_le_ping_procedure(LL_CONN_HANDLE_UNIT *pHandle)
{
    LL_CTRL_PDU_PAYLOAD *pTxPkt;
    UINT8 entry = pHandle->unit_id;

    if (pHandle->llc.type == LLC_PROCEDURE_TYPE_NONE)
    {
        pHandle->llc.type = LLC_PROCEDURE_TYPE_LE_PING_REQ;
    }
    else
    {
        RT_BT_LOG(RED, DAPE_TEST_LOG588, 2, entry, pHandle->llc.type);
        return FALSE;
    }

    /* generate LL_PING_REQ PDU to fire this command */
    pTxPkt = llc_generate_ll_ping_req(entry);
    llc_append_pdu_to_tx_list(pTxPkt, entry);

    /* start LE ping timer for waiting LL_PING_RSP */
    llc_ping_start_wait_rsp_timer(entry);

    return TRUE;
}
#endif

#ifdef _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_
/**
 * @brief Schedule new data length update procedure.
 *
 * It will not start a new one if there is already a data length update
 * procedure processing.
 * @param chu  LL_CONN_HANDLE_UNIT instance.
 * @param state  State in data length update procedure.
 * @return  \c TRUE if successful; \c FALSE on error.
 */
BOOLEAN llc_schedule_data_length_update_procedure(LL_CONN_HANDLE_UNIT *chu,
        LLC_DATA_LEN_UPDATE_STATE state)
{
    LL_CTRL_PDU_PAYLOAD *pdu;
    UINT8 conn_entry = chu->unit_id;

    if (chu->llc.type == LLC_PROCEDURE_TYPE_NONE)
    {
#ifdef _BT4_2_DLE_CHECK_REMOTE_FEATURE_
        if (!chu->support_dle)
        {
            /* skip this flow if remote device and local device both are not
               support data length extension (austin) */
            return FALSE;
        }
#endif

        chu->llc.type = LLC_PROCEDURE_TYPE_DATA_LENGTH_UPDATE;
    }
    else
    {
        return FALSE;
    }

    switch (state)
    {
    case LLC_DATA_LEN_UPDATE_STATE_START_REQ:
        pdu = llc_generate_ll_length_req(chu);
        break;
    case LLC_DATA_LEN_UPDATE_STATE_RECV_REQ:
    case LLC_DATA_LEN_UPDATE_STATE_RECV_REQ_WITH_CONFLICT:
        pdu = llc_generate_ll_length_rsp(chu);
        break;
    default:
        return FALSE;
    }
    if (pdu == NULL)
    {
        chu->llc.type = LLC_PROCEDURE_TYPE_NONE;
        return FALSE;
    }
    llc_append_pdu_to_tx_list(pdu, chu->unit_id);
    chu->llc.pend_data_len_updt = FALSE;
    chu->data_len_updt.state = state;

    ll_start_timer(conn_entry, LL_TIMER_TYPE_LLC);
    return TRUE;
}

BOOLEAN llc_finish_data_length_update_procedure(LL_CONN_HANDLE_UNIT *chu)
{
    if (chu->llc.type != LLC_PROCEDURE_TYPE_DATA_LENGTH_UPDATE)
    {
        return FALSE;
    }
    chu->llc.type = LLC_PROCEDURE_TYPE_NONE;
    chu->data_len_updt.state = LLC_DATA_LEN_UPDATE_STATE_END;
#ifdef USE_FREERTOS
    OS_STOP_TIMER(chu->llc_timer, NULL);
#else
    OS_DELETE_TIMER(&chu->llc_timer);
#endif

    if (ll_fw_update_data_length(chu))
    {
        hci_generate_le_data_length_change_event(chu);
    }
#ifdef _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_INFO_
    ll_fw_print_data_length(chu);
#endif
    return TRUE;
}
#endif /* _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_ */

void llc_stop_procedure_by_ll_unknown_rsp(LL_CONN_HANDLE_UNIT *chu,
        enum LL_CTRL_PDU_OPCODES opcode)
{
    LL_CTRL_PDU_PAYLOAD *pdu = llc_generate_ll_unknown_rsp(opcode);
    if (pdu != NULL)
    {
        llc_append_pdu_to_tx_list(pdu, chu->unit_id);
        llc_check_sent_llc_pdu_request(chu, LL_UNKNOWN_RSP);
    }
}

void llc_stop_procedure_by_ll_reject_ind(LL_CONN_HANDLE_UNIT *chu, UINT8 err_code)
{
    LL_CTRL_PDU_PAYLOAD *pdu = llc_generate_ll_reject_ind(err_code);
    if (pdu != NULL)
    {
        llc_append_pdu_to_tx_list(pdu, chu->unit_id);
        llc_check_sent_llc_pdu_request(chu, LL_REJECT_IND);
    }
}

void llc_stop_procedure_by_ll_reject_ind_ext(LL_CONN_HANDLE_UNIT *chu,
        enum LL_CTRL_PDU_OPCODES rej_opcode, UINT8 err_code)
{
    LL_CTRL_PDU_PAYLOAD *pdu = llc_generate_ll_reject_ind_ext(rej_opcode,
            err_code);
    if (pdu != NULL)
    {
        llc_append_pdu_to_tx_list(pdu, chu->unit_id);
        llc_check_sent_llc_pdu_request(chu, LL_REJECT_IND_EXT);
    }
}

/**************************************************************************
 * Function     : llc_check_and_schedule_next_pend_pdu
 *
 * Description  : This function is used to check and schedule any pending
 *                LLC procedure
 *
 * Parameters   : conn_entry: the index of connection entry
 *
 * Returns      : None
 *
 *************************************************************************/
void llc_check_and_schedule_next_pend_pdu(UINT8 conn_entry)
{
    LL_CONN_HANDLE_UNIT *pHandle;

    pHandle = &ll_manager.conn_unit.handle[conn_entry];

    if (pHandle->is_checking_llc_procedure)
    {
        pHandle->is_checking_llc_procedure = FALSE;
    }

    if ((pHandle->llc.type != LLC_PROCEDURE_TYPE_NONE) ||
        !pHandle->connected)
    {
        return;
    }

    /* check any pending start or restart encryption procedure for schedule */
    if (pHandle->llc.pend_start_encrypt)
    {
        llc_schedule_encryption_procedure(pHandle);
        return;
    }

    /* check any pending version exchange procedure for schedule */
    if (pHandle->llc.pend_version_ex)
    {
        llc_schedule_version_exchange_procedure(pHandle);
        return;
    }

    /* check any pending feature exchange procedure for schedule */
    if (pHandle->llc.pend_feature_ex)
    {
        llc_schedule_feature_exchange_procedure(pHandle);
        return;
    }

    /* check any pending channel map update procedure for schedule */
    if (pHandle->llc.pend_ch_map_updt)
    {
        if (llc_schedule_channel_map_update_procedure(pHandle) == TRUE)
        {
            return;
        }
    }

    /* check any pending connection update procedure for schedule */
    if (pHandle->llc.pend_conn_updt)
    {
        if (llc_schedule_connection_update_procedure(pHandle) == TRUE)
        {
            return;
        }
    }
#ifdef _BT4_1_SUPPORT_LL_CONN_PARAM_REQ
    /* check any pending connection parameter request procedure for schedule */
    if (pHandle->llc.pend_conn_param_req)
    {
        if (llc_schedule_connection_parameter_procedure(pHandle) == TRUE)
        {
            return;
        }
    }
#endif

#ifdef _BT4_1_LE_PING_FEATURE_SUPPORT_
    if (pHandle->llc.pend_ping_req)
    {
        if (llc_schedule_le_ping_procedure(pHandle) == TRUE)
        {
            return;
        }
    }
#endif

#ifdef _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_
    if (pHandle->llc.pend_data_len_updt)
    {
        if (llc_schedule_data_length_update_procedure(pHandle,
                LLC_DATA_LEN_UPDATE_STATE_START_REQ))
        {
            return;
        }
    }
#endif
}


/**************************************************************************
 * Function     : llc_append_pdu_to_tx_list
 *
 * Description  : This function is used to append a LLC pdu to the pending
 *                LLC tx list.
 *
 * Parameters   : llc_pdu: the pointer of llc pdu
 *                conn_entry: the index of connection entry
 *
 * Returns      : None
 *
 *************************************************************************/
void llc_append_pdu_to_tx_list(LL_CTRL_PDU_PAYLOAD *llc_pdu, UINT8 conn_entry)
{
    LE_ACL_PKT_LIST_MANAGE *pacl_pkt_list;
    LL_CONN_HANDLE_UNIT *phandle;

    if (llc_pdu == NULL)
    {
        return;
    }

    DEF_CRITICAL_SECTION_STORAGE;

    MINT_OS_ENTER_CRITICAL();

    phandle = &ll_manager.conn_unit.handle[conn_entry];
    pacl_pkt_list = &phandle->tx_pend_ctrl_pkt_list;

    if (pacl_pkt_list->pCtrlHead == NULL)
    {
        pacl_pkt_list->pCtrlHead = llc_pdu;
    }
    else
    {
        pacl_pkt_list->pCtrlTail->pNext = llc_pdu;
    }
    pacl_pkt_list->pCtrlTail = llc_pdu;
    pacl_pkt_list->pktcnt++;

    MINT_OS_EXIT_CRITICAL();
}


/**************************************************************************
 * Function     : llc_append_pdu_list_to_list
 *
 * Description  : This function is used to append LLC pdu list to the free list.
 *                (It still does not return memory entities to the pool)
 *
 * Parameters   : llc_pdu_list: the llc pdu list for free
 *
 * Returns      : None
 *
 *************************************************************************/
void llc_append_pdu_list_to_list(LE_ACL_PKT_LIST_MANAGE *llc_pdu_list,
                                 UINT8 conn_entry, UINT8 policy,
                                 UINT8 list_type)
{
    LE_ACL_PKT_LIST_MANAGE *pacl_pkt_list;
    LL_CONN_HANDLE_UNIT *phandle;

    if (llc_pdu_list->pktcnt == 0)
    {
        return;
    }

    DEF_CRITICAL_SECTION_STORAGE;

    MINT_OS_ENTER_CRITICAL();

    phandle = &ll_manager.conn_unit.handle[conn_entry];

    if (list_type == LL_LINKED_LIST_TYPE_TX_PEND)
    {
        pacl_pkt_list = &phandle->tx_pend_ctrl_pkt_list;
    }
#ifndef USE_NEW_LE_SCHEDULER
    else if (list_type == LL_LINKED_LIST_TYPE_TX_SCHED)
    {
        pacl_pkt_list = &phandle->tx_sched_ctrl_pkt_list;
    }
#endif
    else
    {
        pacl_pkt_list = &ll_manager.conn_unit.llc_free_pkt_list;
    }

    if (policy == LL_LINKED_LIST_INSERT_POLICY_HEAD)
    {
        if (pacl_pkt_list->pCtrlHead == NULL)
        {
            pacl_pkt_list->pCtrlTail = llc_pdu_list->pCtrlTail;
        }
        else
        {
            llc_pdu_list->pCtrlTail->pNext = pacl_pkt_list->pCtrlHead;
        }
        pacl_pkt_list->pCtrlHead = llc_pdu_list->pCtrlHead;
    }
    else
    {
        if (pacl_pkt_list->pCtrlHead == NULL)
        {
            pacl_pkt_list->pCtrlHead = llc_pdu_list->pCtrlHead;
        }
        else
        {
            pacl_pkt_list->pCtrlTail->pNext = llc_pdu_list->pCtrlHead;
        }
        pacl_pkt_list->pCtrlTail = llc_pdu_list->pCtrlTail;
    }
    pacl_pkt_list->pktcnt += llc_pdu_list->pktcnt;

    MINT_OS_EXIT_CRITICAL();
}

/**************************************************************************
 * Function     : llc_append_pdu_list_to_free_list
 *
 * Description  : This function is used to append LLC pdu list to the free list.
 *                (It still does not return memory entities to the pool)
 *
 * Parameters   : llc_pdu_list: the llc pdu list for free
 *
 * Returns      : None
 *
 *************************************************************************/
void llc_append_pdu_list_to_free_list(LE_ACL_PKT_LIST_MANAGE *llc_pdu_list)
{
    LE_ACL_PKT_LIST_MANAGE *pllc_free_list;

    if (llc_pdu_list->pktcnt == 0)
    {
        return;
    }

    DEF_CRITICAL_SECTION_STORAGE;

    MINT_OS_ENTER_CRITICAL();

    pllc_free_list = &ll_manager.conn_unit.llc_free_pkt_list;

    if (pllc_free_list->pCtrlHead == NULL)
    {
        pllc_free_list->pCtrlHead = llc_pdu_list->pCtrlHead;
    }
    else
    {
        pllc_free_list->pCtrlTail->pNext = llc_pdu_list->pCtrlHead;
    }
    pllc_free_list->pCtrlTail = llc_pdu_list->pCtrlTail;
    pllc_free_list->pktcnt += llc_pdu_list->pktcnt;

    MINT_OS_EXIT_CRITICAL();
}

/**************************************************************************
 * Function     : llc_free_pdu_in_free_list
 *
 * Description  : This function is used to free LLC pdu in the free list. (It
 *                can return memory entities to the pool)
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
void llc_free_pdu_in_free_list(void)
{
    LE_ACL_PKT_LIST_MANAGE *pllc_free_list;
    LL_CTRL_PDU_PAYLOAD *ppkt;
    LL_CTRL_PDU_PAYLOAD *pnext;

    DEF_CRITICAL_SECTION_STORAGE;

    MINT_OS_ENTER_CRITICAL();

    pllc_free_list = &ll_manager.conn_unit.llc_free_pkt_list;
    ppkt = pllc_free_list->pCtrlHead;

    /* init the LLC pdu free list */
    pllc_free_list->pCtrlHead = NULL;
    pllc_free_list->pCtrlTail = NULL;
    pllc_free_list->pktcnt = 0;

    MINT_OS_EXIT_CRITICAL();

    while (ppkt != NULL)
    {
        pnext = ppkt->pNext;
        ppkt->pNext = NULL;

        OS_FREE_BUFFER(le_llc_pdu_pool_id, (void*)ppkt);

        ppkt = pnext;
    }
}

/**************************************************************************
 * Function     : llc_generate_pdu
 *
 * Description  : This function is used to generate a LLC control pdu.
 *                The pdu body is from the le_llc_pdu_pool_id in SRAM.
 *
 * Parameters   : opcode: the opcode of LL control pdu
 *                ppPkt: the pointer of LL control pdu payload
 *
 * Returns      : API_FAILURE or API_SUCCESS
 *
 *************************************************************************/
UINT8 llc_generate_pdu(UINT8 opcode, LL_CTRL_PDU_PAYLOAD **ppPkt)
{
    LL_CTRL_PDU_PAYLOAD *pPkt;
    if (opcode > LL_CTRL_PDU_MAX)
    {
        RT_BT_LOG(RED, LE_MSG_INVALID_LLC_PDU, 1, opcode);
        return API_FAILURE;
    }
    if(OS_ALLOC_BUFFER(le_llc_pdu_pool_id, (void **)&pPkt) != BT_ERROR_OK)
    {
        RT_BT_LOG(RED, LE_MSG_NO_FREE_LE_CONTROL_PDU_IN_POOL, 0, 0);
        return API_FAILURE;
    }

    pPkt->OpCode = opcode;
    pPkt->pNext = NULL;
    *ppPkt = pPkt;

    return API_SUCCESS;
}

/**************************************************************************
 * Function     : llc_generate_ll_connection_update_req
 *
 * Description  : This function is used to generate a LL_CONNECTION_UPDATE_REQ.
 *                The pdu body is from the le_llc_pdu_pool_id in SRAM. The
 *                slave shall not send this PDU.
 *
 * Parameters   : conn_entry: the connection entry id
 *
 * Returns      : the pointer of LL control pdu payload or NULL
 *
 *************************************************************************/
LL_CTRL_PDU_PAYLOAD* llc_generate_ll_connection_update_req(UINT8 conn_entry)
{
    LL_CTRL_PDU_PAYLOAD *pPkt = NULL;
    LL_CONN_UPDT_BLK *pConnUpdtBlk = NULL;
    UINT16 conn_counter;
    UINT16 instant;

    if (conn_entry > LL_MAX_CONNECTION_UNITS)
    {
        //RT_BT_LOG(RED, LE_MSG_INVALID_CONN_ENTRY, 1, conn_entry);
        return NULL;
    }

    if (API_FAILURE == llc_generate_pdu(LL_CONNECTION_UPDATE_REQ, &pPkt))
    {
        return NULL;
    }

    pConnUpdtBlk = &ll_manager.conn_unit.handle[conn_entry].conn_updt_blk;
    conn_counter = ll_manager.conn_unit.handle[conn_entry].conn_counter;
    instant = conn_counter + LL_CONN_UPDT_REQ_RELATIVE_INSTANT +
              ll_manager.conn_unit.handle[conn_entry].slave_latency;
#ifdef _DAPE_LONGER_UPDATE_INSTANT_WHEN_SLAVE_LATENCY
    if (ll_manager.conn_unit.handle[conn_entry].slave_latency)
    {
        instant = conn_counter + LL_CONN_UPDT_REQ_RELATIVE_INSTANT_W_SLV_LATENCY *
              ll_manager.conn_unit.handle[conn_entry].slave_latency * ll_manager.conn_unit.connection_cnts;
    }
#endif
#ifdef _ROM_CODE_PATCHED_
    if (rcp_ll_update_instant_func != NULL)
    {
        rcp_ll_update_instant_func(&instant, conn_entry,
                                   LL_UPDATE_REASON_CONN_PARAM);
    }
#endif

    pConnUpdtBlk->instant = instant;

    /* fill CtrlData of LL_CONNECTION_UPDATE_REQ */
    pPkt->conn_updt_req.WinSize = pConnUpdtBlk->tx_win_size;
    pPkt->conn_updt_req.u1WinOffset[0] = pConnUpdtBlk->tx_win_offset & 0xFF;
    pPkt->conn_updt_req.u1WinOffset[1] = pConnUpdtBlk->tx_win_offset >> 8;
    pPkt->conn_updt_req.u1Interval[0] = pConnUpdtBlk->ce_interval & 0xFF;
    pPkt->conn_updt_req.u1Interval[1] = pConnUpdtBlk->ce_interval >> 8;
    pPkt->conn_updt_req.u1Latency[0] = pConnUpdtBlk->slave_latency & 0xFF;
    pPkt->conn_updt_req.u1Latency[1] = pConnUpdtBlk->slave_latency >> 8;
    pPkt->conn_updt_req.u1Timeout[0] = pConnUpdtBlk->supervision_to & 0xFF;
    pPkt->conn_updt_req.u1Timeout[1] = pConnUpdtBlk->supervision_to >> 8;
    pPkt->conn_updt_req.u1Instant[0] = instant & 0xFF;
    pPkt->conn_updt_req.u1Instant[1] = instant >> 8;

    LL_LOG_TRACE(GREEN, LE_MSG_GEN_LL_CONN_UPDT_REQ_NEW, 14,
                conn_entry, conn_counter, instant,

                ll_manager.conn_unit.handle[conn_entry].ce_interval,
                pConnUpdtBlk->ce_interval,

                ll_manager.conn_unit.handle[conn_entry].ce_length,
                pConnUpdtBlk->ce_length,

                ll_manager.conn_unit.handle[conn_entry].slave_latency,
                pConnUpdtBlk->slave_latency,

                ll_manager.conn_unit.handle[conn_entry].supervision_to,
                pConnUpdtBlk->supervision_to,

                pConnUpdtBlk->tx_win_size,  pConnUpdtBlk->tx_win_size_ofst,
                pConnUpdtBlk->tx_win_offset);

    return pPkt;
}

/**************************************************************************
 * Function     : llc_generate_ll_channel_map_req
 *
 * Description  : This function is used to generate a LL_CHANNEL_MAP_REQ.
 *                The pdu body is from the le_llc_pdu_pool_id in SRAM. The
 *                slave shall not send this PDU.
 *
 * Parameters   : conn_entry: the connection entry id
 *
 * Returns      : the pointer of LL control pdu payload or NULL
 *
 *************************************************************************/
LL_CTRL_PDU_PAYLOAD* llc_generate_ll_channel_map_req(UINT8 conn_entry)
{
    LL_CTRL_PDU_PAYLOAD *pPkt = NULL;
    UINT16 conn_counter;
    UINT16 instant;

    if (conn_entry > LL_MAX_CONNECTION_UNITS)
    {
        //RT_BT_LOG(RED, LE_MSG_INVALID_CONN_ENTRY, 1, conn_entry);
        return NULL;
    }

    if (API_FAILURE == llc_generate_pdu(LL_CHANNEL_MAP_REQ, &pPkt))
    {
        return NULL;
    }

    conn_counter = ll_manager.conn_unit.handle[conn_entry].conn_counter;
    instant = conn_counter + LL_CH_MAP_REQ_RELATIVE_INSTANT +
              ll_manager.conn_unit.handle[conn_entry].slave_latency;
#ifdef _DAPE_TEST_CHG_UPDATE_INSTANT_WHEN_SLAVE_LATENCY
    if (ll_manager.conn_unit.handle[conn_entry].slave_latency)
    {
        instant = conn_counter + LL_CH_MAP_REQ_RELATIVE_INSTANT_W_SLV_LATENCY *
              ll_manager.conn_unit.handle[conn_entry].slave_latency;
    }
#endif

#ifdef _ROM_CODE_PATCHED_
    if (rcp_ll_update_instant_func != NULL)
    {
        rcp_ll_update_instant_func(&instant, conn_entry,
                                   LL_UPDATE_REASON_CH_MAP);
    }
#endif

    ll_manager.conn_unit.handle[conn_entry].ch_map_blk.instant = instant;

    /* fill CtrlData of LL_CHANNEL_MAP_REQ */
    pPkt->ch_map_req.u1ChM[0] = ll_manager.conn_unit.updt_ch_map[0];
    pPkt->ch_map_req.u1ChM[1] = ll_manager.conn_unit.updt_ch_map[1];
    pPkt->ch_map_req.u1ChM[2] = ll_manager.conn_unit.updt_ch_map[2];
    pPkt->ch_map_req.u1ChM[3] = ll_manager.conn_unit.updt_ch_map[3];
    pPkt->ch_map_req.u1ChM[4] = ll_manager.conn_unit.updt_ch_map[4];
    pPkt->ch_map_req.u1Instant[0] = instant & 0xFF;
    pPkt->ch_map_req.u1Instant[1] = instant >> 8;

    LL_LOG_TRACE(GREEN, LE_MSG_GEN_LL_CH_MAP_REQ, 8, conn_entry,
                pPkt->u1CtrData[0], pPkt->u1CtrData[1], pPkt->u1CtrData[2],
                pPkt->u1CtrData[3], pPkt->u1CtrData[4], pPkt->u1CtrData[5],
                pPkt->u1CtrData[6]);

    return pPkt;
}

/**************************************************************************
 * Function     : llc_generate_ll_terminate_req
 *
 * Description  : This function is used to generate a LL_TERMINATE_REQ.
 *                The pdu body is from the le_llc_pdu_pool_id in SRAM.
 *
 * Parameters   : err_code: the error code
 *
 * Returns      : the pointer of LL control pdu payload or NULL
 *
 *************************************************************************/
LL_CTRL_PDU_PAYLOAD* llc_generate_ll_terminate_req(UINT8 err_code)
{
    LL_CTRL_PDU_PAYLOAD *pPkt = NULL;

    if (API_FAILURE == llc_generate_pdu(LL_TERMINATE_IND, &pPkt))
    {
        return NULL;
    }

    /* fill CtrlData of LL_TERMINATE_IND */
    pPkt->terminate.ErrCode = err_code;

    LL_LOG_TRACE(GREEN, LE_MSG_GEN_LL_TERMINATE_REQ, 1, err_code);

    return pPkt;
}

/**************************************************************************
 * Function     : llc_generate_ll_enc_req
 *
 * Description  : This function is used to generate a LL_ENC_REQ.
 *                The pdu body is from the le_llc_pdu_pool_id in SRAM.
 *                This PDU is sent by the master.
 *
 * Parameters   : conn_entry: the connection entry id
 *
 * Returns      : the pointer of LL control pdu payload or NULL
 *
 *************************************************************************/
LL_CTRL_PDU_PAYLOAD* llc_generate_ll_enc_req(UINT8 conn_entry)
{
    LL_CTRL_PDU_PAYLOAD *pPkt = NULL;
    LL_ENCRYPT_BLK *pEnc_blk;

    if (conn_entry > LL_MAX_CONNECTION_UNITS)
    {
        //RT_BT_LOG(RED, LE_MSG_INVALID_CONN_ENTRY, 1, conn_entry);
        return NULL;
    }

    if (API_FAILURE == llc_generate_pdu(LL_ENC_REQ, &pPkt))
    {
        return NULL;
    }

    pEnc_blk = &ll_manager.conn_unit.handle[conn_entry].encrypt_blk;

    /* fill CtrlData of LL_ENC_REQ */
    memcpy(pPkt->enc_req.u1Rand, pEnc_blk->rand, 8);
    pPkt->enc_req.u1EDIV[0] = pEnc_blk->ediv[0];
    pPkt->enc_req.u1EDIV[1] = pEnc_blk->ediv[1];
    memcpy(pPkt->enc_req.u1SKDm, pEnc_blk->skd_m, 8);
    memcpy(pPkt->enc_req.u1IVm, pEnc_blk->iv_m, 4);

    LL_LOG_TRACE(GREEN, LE_MSG_GEN_LL_ENC_REQ, 23, conn_entry,
                pPkt->u1CtrData[0], pPkt->u1CtrData[1], pPkt->u1CtrData[2],
                pPkt->u1CtrData[3], pPkt->u1CtrData[4], pPkt->u1CtrData[5],
                pPkt->u1CtrData[6], pPkt->u1CtrData[7], pPkt->u1CtrData[8],
                pPkt->u1CtrData[9], pPkt->u1CtrData[10], pPkt->u1CtrData[11],
                pPkt->u1CtrData[12], pPkt->u1CtrData[13], pPkt->u1CtrData[14],
                pPkt->u1CtrData[15], pPkt->u1CtrData[16], pPkt->u1CtrData[17],
                pPkt->u1CtrData[18], pPkt->u1CtrData[19], pPkt->u1CtrData[20],
                pPkt->u1CtrData[21]);

    return pPkt;
}

/**************************************************************************
 * Function     : llc_generate_ll_enc_rsp
 *
 * Description  : This function is used to generate a LL_ENC_RSP.
 *                The pdu body is from the le_llc_pdu_pool_id in SRAM.
 *
 * Parameters   : conn_entry: the connection entry id
 *
 * Returns      : the pointer of LL control pdu payload or NULL
 *
 *************************************************************************/
LL_CTRL_PDU_PAYLOAD* llc_generate_ll_enc_rsp(UINT8 conn_entry)
{
    LL_CTRL_PDU_PAYLOAD *pPkt = NULL;
    LL_ENCRYPT_BLK *pEnc_blk;

    if (conn_entry > LL_MAX_CONNECTION_UNITS)
    {
        //RT_BT_LOG(RED, LE_MSG_INVALID_CONN_ENTRY, 1, conn_entry);
        return NULL;
    }

    if (API_FAILURE == llc_generate_pdu(LL_ENC_RSP, &pPkt))
    {
        return NULL;
    }

    pEnc_blk = &ll_manager.conn_unit.handle[conn_entry].encrypt_blk;

    /* fill CtrlData of LL_ENC_RSP */
    memcpy(pPkt->enc_rsp.u1SKDs, pEnc_blk->skd_s, 8);
    memcpy(pPkt->enc_rsp.u1IVs, pEnc_blk->iv_s, 4);

    LL_LOG_TRACE(GREEN, LE_MSG_GEN_LL_ENC_RSP, 13, conn_entry,
                pPkt->u1CtrData[0], pPkt->u1CtrData[1], pPkt->u1CtrData[2],
                pPkt->u1CtrData[3], pPkt->u1CtrData[4], pPkt->u1CtrData[5],
                pPkt->u1CtrData[6], pPkt->u1CtrData[7], pPkt->u1CtrData[8],
                pPkt->u1CtrData[9], pPkt->u1CtrData[10], pPkt->u1CtrData[11]);

    return pPkt;
}

/**************************************************************************
 * Function     : llc_generate_ll_start_enc_req
 *
 * Description  : This function is used to generate a LL_START_ENC_REQ.
 *                The pdu body is from the le_llc_pdu_pool_id in SRAM.
 *
 * Parameters   : None
 *
 * Returns      : the pointer of LL control pdu payload or NULL
 *
 *************************************************************************/
LL_CTRL_PDU_PAYLOAD* llc_generate_ll_start_enc_req(void)
{
    LL_CTRL_PDU_PAYLOAD *pPkt = NULL;

    if (API_FAILURE == llc_generate_pdu(LL_START_ENC_REQ, &pPkt))
    {
        return NULL;
    }

    LL_LOG_TRACE(GREEN, LE_MSG_GEN_LL_START_ENC_REQ, 0, 0);

    return pPkt;
}

/**************************************************************************
 * Function     : llc_generate_ll_start_enc_rsp
 *
 * Description  : This function is used to generate a LL_START_ENC_RSP.
 *                The pdu body is from the le_llc_pdu_pool_id in SRAM.
 *
 * Parameters   : None
 *
 * Returns      : the pointer of LL control pdu payload or NULL
 *
 *************************************************************************/
LL_CTRL_PDU_PAYLOAD* llc_generate_ll_start_enc_rsp(void)
{
    LL_CTRL_PDU_PAYLOAD *pPkt = NULL;

    if (API_FAILURE == llc_generate_pdu(LL_START_ENC_RSP, &pPkt))
    {
        return NULL;
    }

    LL_LOG_TRACE(GREEN, LE_MSG_GEN_LL_START_ENC_RSP, 0, 0);

    return pPkt;
}


/**************************************************************************
 * Function     : llc_generate_ll_unknown_rsp
 *
 * Description  : This function is used to generate a LL_UNKNOWN_RSP.
 *                The pdu body is from the le_llc_pdu_pool_id in SRAM.
 *
 * Parameters   : unknown_type: the unknwon type
 *
 * Returns      : the pointer of LL control pdu payload or NULL
 *
 *************************************************************************/
LL_CTRL_PDU_PAYLOAD* llc_generate_ll_unknown_rsp(UINT8 unknown_type)
{
    LL_CTRL_PDU_PAYLOAD *pPkt = NULL;

    if (API_FAILURE == llc_generate_pdu(LL_UNKNOWN_RSP, &pPkt))
    {
        return NULL;
    }

    /* fill CtrlData of LL_UNKNOWN_RSP */
    pPkt->unknown_rsp.UnknownType = unknown_type;

    LL_LOG_TRACE(GREEN, LE_MSG_GEN_LL_UNKNOWN_RSP, 1, pPkt->u1CtrData[0]);

    return pPkt;
}

/**************************************************************************
 * Function     : llc_generate_ll_feature_req
 *
 * Description  : This function is used to generate a LL_FEATURE_REQ.
 *                The pdu body is from the le_llc_pdu_pool_id in SRAM.
 *
 * Parameters   : None
 *
 * Returns      : the pointer of LL control pdu payload or NULL
 *
 *************************************************************************/
LL_CTRL_PDU_PAYLOAD* llc_generate_ll_feature_req(void)
{
    LL_CTRL_PDU_PAYLOAD *pPkt = NULL;

    if (API_FAILURE == llc_generate_pdu(LL_FEATURE_REQ, &pPkt))
    {
        return NULL;
    }

    /* fill CtrlData of LL_FEATURE_REQ */
    memset(pPkt->feature_req.u1FeatureSet, 0, 8);
#ifdef _DAPE_UNIFY_VERSION_FEATURE_UINT
    memcpy(&pPkt->feature_req.u1FeatureSet[0], &ll_manager.ll_feature_support[0], 8);
#else
    pPkt->feature_rsp.u1FeatureSet[0] = (
            ll_manager.conn_unit.support_enc
#ifdef _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_
            | (ll_manager.conn_unit.support_dle << 5)
#endif
#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_
            | (ll_manager.support_ll_privacy << 6)
#endif
            );
    /* if support bt4.1, then directly enable these two features bits
       since these two are very easy. */
    if (IS_BT41)
    {
        pPkt->feature_rsp.u1FeatureSet[0] |= (SUPPORT_REJ_IND_EXT |
                                               SUPPORT_LE_PING | SUPPORT_SLV_INIT_FEATURE_REQ);
    }
#endif
    LL_LOG_TRACE(GREEN, LE_MSG_GEN_LL_FEATURE_REQ, 8,
                pPkt->u1CtrData[0], pPkt->u1CtrData[1], pPkt->u1CtrData[2],
                pPkt->u1CtrData[3], pPkt->u1CtrData[4], pPkt->u1CtrData[5],
                pPkt->u1CtrData[6], pPkt->u1CtrData[7]);

    return pPkt;
}

/**************************************************************************
 * Function     : llc_generate_ll_feature_rsp
 *
 * Description  : This function is used to generate a LL_FEATURE_RSP.
 *                The pdu body is from the le_llc_pdu_pool_id in SRAM.
 *
 * Parameters   : conn_entry: the connection entry id
 *
 * Returns      : the pointer of LL control pdu payload or NULL
 *
 *************************************************************************/
LL_CTRL_PDU_PAYLOAD* llc_generate_ll_feature_rsp(UINT8 conn_entry)
{
    LL_CTRL_PDU_PAYLOAD *pPkt = NULL;

    if (conn_entry > LL_MAX_CONNECTION_UNITS)
    {
        //RT_BT_LOG(RED, LE_MSG_INVALID_CONN_ENTRY, 1, conn_entry);
        return NULL;
    }


    if (API_FAILURE == llc_generate_pdu(LL_FEATURE_RSP, &pPkt))
    {
        return NULL;
    }

    /* fill CtrlData of LL_FEATURE_RSP */
    memset(pPkt->feature_rsp.u1FeatureSet, 0, 8);
#ifdef _DAPE_UNIFY_VERSION_FEATURE_UINT
    memcpy(&pPkt->feature_rsp.u1FeatureSet[0], &ll_manager.ll_feature_support[0], 8);
#else
    pPkt->feature_rsp.u1FeatureSet[0] = (
            ll_manager.conn_unit.handle[conn_entry].support_enc
#ifdef _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_
            | (ll_manager.conn_unit.support_dle << 5)
#endif
            );
    /* if support bt4.1, then directly enable these two features bits
       since these three are very easy. */
    if (IS_BT41)
    {
        pPkt->feature_rsp.u1FeatureSet[0] |= (SUPPORT_REJ_IND_EXT| SUPPORT_LE_PING | SUPPORT_SLV_INIT_FEATURE_REQ);
    }
#endif
    LL_LOG_TRACE(GREEN, LE_MSG_GEN_LL_FEATURE_RSP, 9, conn_entry,
                pPkt->u1CtrData[0], pPkt->u1CtrData[1], pPkt->u1CtrData[2],
                pPkt->u1CtrData[3], pPkt->u1CtrData[4], pPkt->u1CtrData[5],
                pPkt->u1CtrData[6], pPkt->u1CtrData[7]);

    return pPkt;
}

/**************************************************************************
 * Function     : llc_generate_ll_pause_enc_req
 *
 * Description  : This function is used to generate a LL_PAUSE_ENC_REQ.
 *                The pdu body is from the le_llc_pdu_pool_id in SRAM.
 *
 * Parameters   : None
 *
 * Returns      : the pointer of LL control pdu payload or NULL
 *
 *************************************************************************/
LL_CTRL_PDU_PAYLOAD* llc_generate_ll_pause_enc_req(void)
{
    LL_CTRL_PDU_PAYLOAD *pPkt = NULL;

    if (API_FAILURE == llc_generate_pdu(LL_PAUSE_ENC_REQ, &pPkt))
    {
        return NULL;
    }

    LL_LOG_TRACE(GREEN, LE_MSG_GEN_LL_PAUSE_ENC_REQ, 0, 0);

    return pPkt;
}

/**************************************************************************
 * Function     : llc_generate_ll_pause_enc_rsp
 *
 * Description  : This function is used to generate a LL_PAUSE_ENC_RSP.
 *                The pdu body is from the le_llc_pdu_pool_id in SRAM.
 *
 * Parameters   : None
 *
 * Returns      : the pointer of LL control pdu payload or NULL
 *
 *************************************************************************/
LL_CTRL_PDU_PAYLOAD* llc_generate_ll_pause_enc_rsp(void)
{
    LL_CTRL_PDU_PAYLOAD *pPkt = NULL;

    if (API_FAILURE == llc_generate_pdu(LL_PAUSE_ENC_RSP, &pPkt))
    {
        return NULL;
    }

    LL_LOG_TRACE(GREEN, LE_MSG_GEN_LL_PAUSE_ENC_RSP, 0, 0);

    return pPkt;
}

/**************************************************************************
 * Function     : llc_generate_ll_version_ind
 *
 * Description  : This function is used to generate a LL_VERSION_IND.
 *                The pdu body is from the le_llc_pdu_pool_id in SRAM.
 *
 * Parameters   : None
 *
 * Returns      : the pointer of LL control pdu payload or NULL
 *
 *************************************************************************/
LL_CTRL_PDU_PAYLOAD* llc_generate_ll_version_ind(void)
{
    LL_CTRL_PDU_PAYLOAD *pPkt = NULL;

    if (API_FAILURE == llc_generate_pdu(LL_VERSION_IND, &pPkt))
    {
        return NULL;
    }
#ifdef _DAPE_UNIFY_VERSION_FEATURE_UINT
        pPkt->version_ind.VersNr = ll_manager.ll_version;
#else
    /* fill CtrlData of LL_VERSION_IND */
    if (IS_BT42)
    {
        pPkt->version_ind.VersNr = BT_FW_LL_VERSION_BT42;
    }
    else if (IS_BT41)
    {
        pPkt->version_ind.VersNr = BT_FW_LL_VERSION_BT41;
    }
    else
    {
        pPkt->version_ind.VersNr = BT_FW_LL_VERSION_BT40;
    }
#endif
    pPkt->version_ind.u1CompId[0] = BT_FW_MANUFACTURER_NAME & 0xFF;
    pPkt->version_ind.u1CompId[1] = BT_FW_MANUFACTURER_NAME >> 8;
    pPkt->version_ind.u1SubVersNr[0] = FW_VERSION_NUMBER & 0xFF;
    pPkt->version_ind.u1SubVersNr[1] = FW_VERSION_NUMBER >> 8;

    LL_LOG_TRACE(GREEN, LE_MSG_GEN_LL_VERSION_IND, 5,
                pPkt->u1CtrData[0], pPkt->u1CtrData[1], pPkt->u1CtrData[2],
                pPkt->u1CtrData[3], pPkt->u1CtrData[4]);

    return pPkt;
}

/**************************************************************************
 * Function     : llc_generate_ll_reject_ind
 *
 * Description  : This function is used to generate a LL_REJECT_IND.
 *                The pdu body is from the le_llc_pdu_pool_id in SRAM.
 *
 * Parameters   : err_code: the error code
 *
 * Returns      : the pointer of LL control pdu payload or NULL
 *
 *************************************************************************/
LL_CTRL_PDU_PAYLOAD* llc_generate_ll_reject_ind(UINT8 err_code)
{
    LL_CTRL_PDU_PAYLOAD *pPkt = NULL;

    if (API_FAILURE == llc_generate_pdu(LL_REJECT_IND, &pPkt))
    {
        return NULL;
    }

    /* fill CtrlData of LL_REJECT_IND */
    pPkt->reject_ind.ErrCode = err_code;

    LL_LOG_TRACE(GREEN, LE_MSG_GEN_LL_REJECT_IND, 1, pPkt->u1CtrData[0]);

    return pPkt;
}


LL_CTRL_PDU_PAYLOAD *llc_generate_ll_reject_ind_ext(UINT8 rej_opcode,
        UINT8 err_code)
{
    LL_CTRL_PDU_PAYLOAD *pdu;

    if (API_FAILURE == llc_generate_pdu(LL_REJECT_IND_EXT, &pdu))
    {
        return NULL;
    }

    pdu->reject_ind_ext.rej_opcode = rej_opcode;
    pdu->reject_ind_ext.err_code = err_code;

    LL_LOG_TRACE(GREEN, LE_MSG_GEN_LL_REJECT_IND_EXT, 2,
            pdu->reject_ind_ext.rej_opcode, pdu->reject_ind_ext.err_code);
    return pdu;
}

/**************************************************************************
 * Function     : llc_handle_ll_connection_update_req
 *
 * Description  : This function is used to handle an LL_CONNECTION_UPDATE_REQ.
 *
 * Parameters   : pdata: the pointer of payload body
 *                len:   the length of payload body
 *                entry: the connection entry index
 *
 * Returns      : None
 *
 *************************************************************************/
void llc_handle_ll_connection_update_req(UINT8 *pdata, UINT8 len, UINT8 entry)
{
    LL_CTRL_PDU_PAYLOAD *pRxPdu = (LL_CTRL_PDU_PAYLOAD *)pdata;
    LL_CTRL_PDU_PAYLOAD *pTxPkt;
    LL_CONN_UPDT_BLK *pConnUpdt;
    UINT16 connEventCount;

    if (ll_manager.conn_unit.master || (len != LL_CONNECTION_UPDATE_REQ_LEN))
    {
        /* when we are the master or the payload length is mismatched,
           we can not accept this request.
           therefore, we will send a LL_UNKNOWN_RSP to the slave */
        pTxPkt = llc_generate_ll_unknown_rsp(LL_CONNECTION_UPDATE_REQ);
        llc_append_pdu_to_tx_list(pTxPkt, entry);
        return;
    }

    /* update current connection counter from hw register */
    connEventCount = RD_LE_REG(LE_REG_CONN_COUNTER);
    ll_manager.conn_unit.handle[entry].conn_counter = connEventCount;

    pConnUpdt = &ll_manager.conn_unit.handle[entry].conn_updt_blk;
    pConnUpdt->instant = *(UINT16*)pRxPdu->conn_updt_req.u1Instant;

    /* Check the instant in the past or not.
     * NOTE: Don't change the codes unless you know what you're doing.
     * For example, make sure compiler generate correct instructions like sltiu
     * for the comparison with 32767.
     */
    if (((UINT16) (pConnUpdt->instant - connEventCount) >= 32767U)
#ifdef _LE_SUPPORT_CSA3_LE_ERRATA_
            || (pConnUpdt->instant == connEventCount)
#endif
            )
    {
        /* the instant in the past. The LL of the slave shall consider the
           connection to be lost, the LL shall exit the connection state,
           and transition to the standby state and shall notify the Host */

        ll_manager.conn_unit.handle[entry].kill_link_reason =
                                                  LL_KILL_REASON_UPDT_PAST;
        /* kill the connection */
        ll_driver_kill_connection(entry);

		LL_LOG_TRACE(RED, DAPE_TEST_LOG284, 3,entry, connEventCount, pConnUpdt->instant);
    }
    else
    {
        ll_manager.conn_unit.conn_updt_entry = entry;
        pConnUpdt->tx_win_size = pRxPdu->conn_updt_req.WinSize;
        pConnUpdt->tx_win_offset = *(UINT16*)pRxPdu->conn_updt_req.u1WinOffset;
        pConnUpdt->ce_interval = *(UINT16*)pRxPdu->conn_updt_req.u1Interval;
        pConnUpdt->slave_latency = *(UINT16*)pRxPdu->conn_updt_req.u1Latency;
        pConnUpdt->supervision_to = *(UINT16*)pRxPdu->conn_updt_req.u1Timeout;
        // dape added
        pConnUpdt->ce_length = (pConnUpdt->ce_interval -1) << 1;

        /* because we are the slave and the LLC procedure shall be initiated in
           the LL at a time per connection oer device */
        ll_driver_connection_update(entry, pConnUpdt->instant);

        LL_LOG_TRACE(GREEN, DAPE_TEST_LOG280, 9,
               entry, connEventCount, pConnUpdt->instant,
               pConnUpdt->tx_win_size,
               pConnUpdt->tx_win_offset,
               pConnUpdt->ce_interval,
               pConnUpdt->slave_latency,
               pConnUpdt->supervision_to,
               pConnUpdt->ce_length);
    }
}

/**************************************************************************
 * Function     : llc_handle_ll_channel_map_req
 *
 * Description  : This function is used to handle an LL_CHANNEL_MAP_REQ.
 *
 * Parameters   : pdata: the pointer of payload body
 *                len:   the length of payload body
 *                entry: the connection entry index
 *
 * Returns      : None
 *
 *************************************************************************/
void llc_handle_ll_channel_map_req(UINT8 *pdata, UINT8 len, UINT8 entry)
{
    LL_CTRL_PDU_PAYLOAD *pRxPdu = (LL_CTRL_PDU_PAYLOAD *)pdata;
    LL_CTRL_PDU_PAYLOAD *pTxPkt;
    LL_CH_MAP_BLK *pChMap;
    UINT16 connEventCount;

    //LL_LOG_TRACE(WHITE, DAPE_TEST_LOG277, 2, len, entry);

    if (ll_manager.conn_unit.master || (len != LL_CHANNEL_MAP_REQ_LEN))
    {
        /* when we are the master or the payload length is mismatched,
           we can not accept this request.
           therefore, we will send a LL_UNKNOWN_RSP to the slave */
        pTxPkt = llc_generate_ll_unknown_rsp(LL_CHANNEL_MAP_REQ);
        llc_append_pdu_to_tx_list(pTxPkt, entry);
        return;
    }

    /* update current connection counter from hw register */
    connEventCount = RD_LE_REG(LE_REG_CONN_COUNTER);
    ll_manager.conn_unit.handle[entry].conn_counter = connEventCount;

    pChMap = &ll_manager.conn_unit.handle[entry].ch_map_blk;
    pChMap->instant = *(UINT16*)pRxPdu->ch_map_req.u1Instant;

    /* Check the instant in the past or not.
     * NOTE: Don't change the codes unless you know what you're doing.
     * For example, make sure compiler generate correct instructions like sltiu
     * for the comparison with 32767.
     */
    if (((UINT16 ) (pChMap->instant - connEventCount) >= 32767U)
#ifdef _LE_SUPPORT_CSA3_LE_ERRATA_
            || (pChMap->instant == connEventCount)
#endif
            )
    {
        /* the instant in the past. The LL of the slave shall consider the
           connection to be lost, the LL shall exit the connection state,
           and transition to the standby state and shall notify the Host */
        ll_manager.conn_unit.handle[entry].kill_link_reason =
                                                    LL_KILL_REASON_UPDT_PAST;

        /* kill the connection */
        ll_driver_kill_connection(entry);

        LL_LOG_TRACE(RED, DAPE_TEST_LOG283, 3, entry, connEventCount, pChMap->instant);
    }
    else
    {
        ll_manager.conn_unit.chm_updt_entry = entry;
        ll_manager.conn_unit.updt_ch_map[0] = pRxPdu->ch_map_req.u1ChM[0];
        ll_manager.conn_unit.updt_ch_map[1] = pRxPdu->ch_map_req.u1ChM[1];
        ll_manager.conn_unit.updt_ch_map[2] = pRxPdu->ch_map_req.u1ChM[2];
        ll_manager.conn_unit.updt_ch_map[3] = pRxPdu->ch_map_req.u1ChM[3];
        ll_manager.conn_unit.updt_ch_map[4] = pRxPdu->ch_map_req.u1ChM[4];

        /* because we are the slave and the LLC procedure shall be initiated in
           the LL at a time per connection per device */
        ll_driver_channel_map_update(entry, pChMap->instant);

        LL_LOG_TRACE(GREEN, DAPE_TEST_LOG279, 8,
               entry, connEventCount, pChMap->instant,
               pRxPdu->ch_map_req.u1ChM[0],
               pRxPdu->ch_map_req.u1ChM[1],
               pRxPdu->ch_map_req.u1ChM[2],
               pRxPdu->ch_map_req.u1ChM[3],
               pRxPdu->ch_map_req.u1ChM[4]);
    }
}

/**************************************************************************
 * Function     : llc_handle_ll_terminate_ind
 *
 * Description  : This function is used to handle an LL_TERMINATE_IND.
 *
 * Parameters   : pdata: the pointer of payload body
 *                len:   the length of payload body
 *                entry: the connection entry index
 *
 * Returns      : None
 *
 *************************************************************************/
void llc_handle_ll_terminate_ind(UINT8 *pdata, UINT8 len, UINT8 entry)
{
    LL_CTRL_PDU_PAYLOAD *pRxPdu = (LL_CTRL_PDU_PAYLOAD *)pdata;
    LL_CTRL_PDU_PAYLOAD *pTxPkt;
    UINT8 remote_err_code;
    LL_CONN_HANDLE_UNIT *handle;
    OS_SIGNAL sig_send;
    LL_TASK_PARA_U task_param;

    if (len != LL_TERMINATE_IND_LEN)
    {
        /* if the payload length is mismatched, we can not accept this request.
           therefore, we will send a LL_UNKNOWN_RSP to the slave */
        pTxPkt = llc_generate_ll_unknown_rsp(LL_TERMINATE_IND);
        llc_append_pdu_to_tx_list(pTxPkt, entry);
        return;
    }

    remote_err_code = pRxPdu->terminate.ErrCode;

    handle = &ll_manager.conn_unit.handle[entry];

    LL_LOG_TRACE(WHITE, LE_MSG_RECEIVE_TERMINATE, 2, entry, remote_err_code);

    if (handle->connected)
    {
        /* TODO: kill the connection and release relative resources */
        /* free software timer */
#ifdef USE_FREERTOS
        OS_STOP_TIMER(handle->llc_timer, NULL);
        OS_STOP_TIMER(handle->sup_timer, NULL);
#else
        if (handle->llc_timer != NULL)
        {
            OS_DELETE_TIMER(&handle->llc_timer);
        }

        /* free supervision timer */
        if (handle->sup_timer != NULL)
        {
            OS_DELETE_TIMER(&handle->sup_timer);
        }
#endif

        /* TODO: notify the Host and report the error code */
        task_param.Dword = 0;
        task_param.lmp_s.sub_type = LL_TASK_HANDLE_DISCONN;
        task_param.lmp_s.conn_entry = entry;
        task_param.lmp_s.status = HCI_COMMAND_SUCCEEDED;
        task_param.lmp_s.reason = remote_err_code;
        sig_send.type = LL_GEN_HCI_EVENT_SIGNAL;
        sig_send.param = (OS_ADDRESS)task_param.Dword;

        OS_SEND_SIGNAL_TO_TASK (lmp_task_handle, sig_send);
    }
}

/**************************************************************************
 * Function     : llc_handle_ll_enc_req
 *
 * Description  : This function is used to handle an LL_ENC_REQ.
 *
 * Parameters   : pdata: the pointer of payload body
 *                len:   the length of payload body
 *                entry: the connection entry index
 *
 * Returns      : None
 *
 *************************************************************************/
void llc_handle_ll_enc_req(UINT8 *pdata, UINT8 len, UINT8 entry)
{
    LL_CTRL_PDU_PAYLOAD *pTxPkt;
    LL_CTRL_PDU_PAYLOAD *pRxPdu = (LL_CTRL_PDU_PAYLOAD *)pdata;
    LL_ENCRYPT_BLK *pEnc_blk;
    LL_TASK_PARA_U task_param;
    OS_SIGNAL sig_send;
    LL_CONN_HANDLE_UNIT *pHandle;

    pHandle = &ll_manager.conn_unit.handle[entry];

    LL_LOG_TRACE(YELLOW, DAPE_TEST_LOG267, 5,
		         ll_manager.conn_unit.master,
		         entry, len,
		         pHandle->support_enc,
		         pHandle->encrypted);

    if (ll_manager.conn_unit.master || (len != LL_ENC_REQ_LEN))
    {
        /* when we are the master, we can not accept this request.
           therefore, we will send a LL_UNKNOWN_RSP to the slave */
        pTxPkt = llc_generate_ll_unknown_rsp(LL_ENC_REQ);
        llc_append_pdu_to_tx_list(pTxPkt, entry);
        return;
    }

    if (!pHandle->support_enc)
    {
        /* we don't support encryption so send LL_REJECT_IND to the master */
        pTxPkt = llc_generate_ll_reject_ind(UNSUPPORTED_REMOTE_FEATURE_ERROR);
        llc_append_pdu_to_tx_list(pTxPkt, entry);
        return;
    }

    /* discard any acl user data packets */
    pHandle->pause_acl_tx = TRUE;
    pHandle->discard_acl_rx = TRUE;

    if (pHandle->encrypted)
    {
        /* restart encryption */
        pHandle->llc.type = LLC_PROCEDURE_TYPE_ENCRYPTION_PAUSE;
    }
    else
    {
        /* start encryption */
        pHandle->llc.type = LLC_PROCEDURE_TYPE_ENCRYPTION_START;
    }

#ifdef _TP_SEC_MAS_BV12C_LOWER_TESTER
    {
        llc_schedule_version_exchange_procedure(pHandle);
    }
#endif
#ifdef _TP_SEC_MAS_BV13C_LOWER_TESTER
    {
        pTxPkt = llc_generate_ll_slave_feature_req();
        llc_append_pdu_to_tx_list(pTxPkt, entry);
    }
#endif

    /* start the LLC procedure timer */
    ll_start_timer(entry, LL_TIMER_TYPE_LLC);

    /* send LL_ENC_RSP to the master */
    pTxPkt = llc_generate_ll_enc_rsp(entry);
    llc_append_pdu_to_tx_list(pTxPkt, entry);
#ifdef _TP_SEC_MAS_BV14C_LOWER_TESTER
    {
        llc_schedule_version_exchange_procedure(pHandle);
    }
#endif

    /* get master's paramters from LL_ENC_REQ */
    pEnc_blk = &ll_manager.conn_unit.handle[entry].encrypt_blk;
    memcpy(pEnc_blk->rand, pRxPdu->enc_req.u1Rand, 8);
    pEnc_blk->ediv[0] = pRxPdu->enc_req.u1EDIV[0];
    pEnc_blk->ediv[1] = pRxPdu->enc_req.u1EDIV[1];
    memcpy(pEnc_blk->skd_m, pRxPdu->enc_req.u1SKDm, 8);
    memcpy(pEnc_blk->iv_m, pRxPdu->enc_req.u1IVm, 4);

    /* send a LE_Long_Term_Key_Request event to the Host */
    task_param.Dword = 0;
    task_param.lmp_s.sub_type = LL_TASK_HANDLE_LONG_TERM_KEY_REQUEST_REPLY;
    task_param.lmp_s.conn_entry = entry;
    sig_send.type = LL_GEN_HCI_EVENT_SIGNAL;
    sig_send.param = (OS_ADDRESS)task_param.Dword;
    OS_SEND_SIGNAL_TO_TASK (lmp_task_handle, sig_send);

    pHandle->long_term_key_got = 0;
    pEnc_blk->start_enc_state = LLC_START_ENC_STATE_BEGIN;
}

/**************************************************************************
 * Function     : llc_handle_ll_enc_rsp
 *
 * Description  : This function is used to handle an LL_ENC_RSP.
 *
 * Parameters   : pdata: the pointer of payload body
 *                len:   the length of payload body
 *                entry: the connection entry index
 *
 * Returns      : None
 *
 *************************************************************************/
void llc_handle_ll_enc_rsp(UINT8 *pdata, UINT8 len, UINT8 entry)
{
    LL_CTRL_PDU_PAYLOAD *pTxPkt;
    LL_CTRL_PDU_PAYLOAD *pRxPdu = (LL_CTRL_PDU_PAYLOAD *)pdata;
    LL_ENCRYPT_BLK *pEnc_blk;
    LL_CONN_HANDLE_UNIT *pHandle;

    pHandle = &ll_manager.conn_unit.handle[entry];

    LL_LOG_TRACE(GREEN, DAPE_TEST_LOG260, 6,
		ll_manager.conn_unit.master,
		entry, len,
		pHandle->support_enc,
		pHandle->encrypt_blk.start_enc_state,
		pHandle->install_key);

    if (!ll_manager.conn_unit.master || (len != LL_ENC_RSP_LEN))
    {
        /* when we are the slave, we can not accept this request.
           therefore, we will send a LL_UNKNOWN_RSP to the master */
        pTxPkt = llc_generate_ll_unknown_rsp(LL_ENC_RSP);
        llc_append_pdu_to_tx_list(pTxPkt, entry);
        llc_check_sent_llc_pdu_request(pHandle, LL_UNKNOWN_RSP);
        return;
    }

    if (!pHandle->support_enc)
    {
        /* we don't support encryption so send LL_REJECT_IND to the slave */
        pTxPkt = llc_generate_ll_reject_ind(UNSUPPORTED_REMOTE_FEATURE_ERROR);
        llc_append_pdu_to_tx_list(pTxPkt, entry);
        llc_check_sent_llc_pdu_request(pHandle, LL_REJECT_IND);
        return;
    }

    /* get slave's paramters from LL_ENC_REQ */
    pEnc_blk = &pHandle->encrypt_blk;
    memcpy(pEnc_blk->skd_s, pRxPdu->enc_rsp.u1SKDs, 8);
    memcpy(pEnc_blk->iv_s, pRxPdu->enc_rsp.u1IVs, 4);

    /* discard any acl user data packets */
    pHandle->discard_acl_rx = TRUE;

    /* TODO: We have Long_Term_Key as the key. and full SKD as the plain text
       input. So we can calculate the session key, too */
    ll_driver_get_session_key((UINT16*)pEnc_blk->ltk,
                              (UINT16*)pEnc_blk->skd_m,
                              (UINT16*)pEnc_blk->sesskey);

    /* Put here to guarantee session key is calculated. */
    pEnc_blk->start_enc_state = LLC_START_ENC_STATE_BEGIN;

    if ((pHandle->encrypt_blk.start_enc_state == LLC_START_ENC_STATE_BEGIN) &&
        (!pHandle->install_key))
    {
        ll_driver_install_encryption_info(LL_MASTER, entry,
                                          (UINT16*)pEnc_blk->iv_m,
                                          (UINT16*)pEnc_blk->sesskey);
        pHandle->install_key = 1;
    }
}

/**************************************************************************
 * Function     : llc_handle_ll_start_enc_req
 *
 * Description  : This function is used to handle an LL_START_ENC_REQ.
 *
 * Parameters   : pdata: the pointer of payload body
 *                len:   the length of payload body
 *                entry: the connection entry index
 *
 * Returns      : None
 *
 *************************************************************************/
void llc_handle_ll_start_enc_req(UINT8 *pdata, UINT8 len, UINT8 entry)
{
    LL_CTRL_PDU_PAYLOAD *pTxPkt;
    LL_ENCRYPT_BLK *pEnc_blk;
    LL_CONN_HANDLE_UNIT *handle;

    handle = &ll_manager.conn_unit.handle[entry];

    if (!ll_manager.conn_unit.master || (len != LL_START_ENC_REQ_LEN))
    {
        /* when we are the slave, we can not accept this request.
           therefore, we will send a LL_UNKNOWN_RSP to the master */
        pTxPkt = llc_generate_ll_unknown_rsp(LL_START_ENC_REQ);
        llc_append_pdu_to_tx_list(pTxPkt, entry);
        return;
    }

    if (!handle->support_enc)
    {
        /* TODO: send LL_REJECT_IND to the master */
        pTxPkt = llc_generate_ll_reject_ind(UNSUPPORTED_REMOTE_FEATURE_ERROR);
        llc_append_pdu_to_tx_list(pTxPkt, entry);
        llc_check_sent_llc_pdu_request(handle, LL_REJECT_IND);
        return;
    }

    /* this shall be the way 1 of three way handshake */

    /* TODO: we shall start to encrpt the tx/rx data path then send encrpted
       LL_START_ENC_RSP */
    pEnc_blk = &handle->encrypt_blk;
    if (LLC_START_ENC_STATE_BEGIN == pEnc_blk->start_enc_state)
    {
        /* TODO: we shall start to encrpt the tx data path then
           send encrpted LL_START_ENC_RSP */
        /* we can start to send LL_ENC_REQ to slave now */
        pTxPkt = llc_generate_ll_start_enc_rsp();
        llc_append_pdu_to_tx_list(pTxPkt, entry);
        pEnc_blk->start_enc_state = LLC_START_ENC_STATE_M2S_START_ENC_RSP;

        return;
    }

    /* TODO: we may send LL_REJECT_IND or LL_TERMINATE ? */
    pTxPkt = llc_generate_ll_reject_ind(ENCRYPTION_MODE_NOT_ACCEPTABLE_ERROR);
    llc_append_pdu_to_tx_list(pTxPkt, entry);
    llc_check_sent_llc_pdu_request(handle, LL_REJECT_IND);
}

/**************************************************************************
 * Function     : llc_handle_ll_start_enc_rsp
 *
 * Description  : This function is used to handle an LL_START_ENC_RSP.
 *
 * Parameters   : pdata: the pointer of payload body
 *                len:   the length of payload body
 *                entry: the connection entry index
 *
 * Returns      : None
 *
 *************************************************************************/
void llc_handle_ll_start_enc_rsp(UINT8 *pdata, UINT8 len, UINT8 entry)
{
    LL_CTRL_PDU_PAYLOAD *pTxPkt;
    LL_CONN_HANDLE_UNIT *pHandle;
    LL_ENCRYPT_BLK *pEnc_blk;
    LL_TASK_PARA_U task_param;
    OS_SIGNAL sig_send;

    pHandle = &ll_manager.conn_unit.handle[entry];

    if (len != LL_START_ENC_RSP_LEN)
    {
        /* when we are the slave, we can not accept this request.
           therefore, we will send a LL_UNKNOWN_RSP to the master */
        pTxPkt = llc_generate_ll_unknown_rsp(LL_START_ENC_RSP);
        llc_append_pdu_to_tx_list(pTxPkt, entry);
        return;
    }

    if (!ll_manager.conn_unit.handle[entry].support_enc)
    {
        /* TODO: send LL_REJECT_IND to the master */
        pTxPkt = llc_generate_ll_reject_ind(UNSUPPORTED_REMOTE_FEATURE_ERROR);
        llc_append_pdu_to_tx_list(pTxPkt, entry);
        llc_check_sent_llc_pdu_request(pHandle, LL_REJECT_IND);
        return;
    }

    pEnc_blk = &ll_manager.conn_unit.handle[entry].encrypt_blk;

    if (ll_manager.conn_unit.master)
    {
        /* this shall be the way 3 of three way handshake for master */
        if (LLC_START_ENC_STATE_M2S_START_ENC_RSP == pEnc_blk->start_enc_state)
        {
            pEnc_blk->start_enc_state = LLC_START_ENC_STATE_END;

            /* Encryption is enable */
            pHandle->encrypted = TRUE;

            if (LLC_PAUSE_ENC_STATE_END == pEnc_blk->pause_enc_state)
            {
                /* finish restart encryption procedure. we can stop and kill
                    llc procedure response timer */
#ifdef USE_FREERTOS
                OS_STOP_TIMER(pHandle->llc_timer, NULL);
#else
                OS_DELETE_TIMER(&pHandle->llc_timer);
#endif

                /* resume acl user data packets */
                pHandle->pause_acl_tx = FALSE;
                pHandle->discard_acl_rx = FALSE;

                pEnc_blk->pause_enc_state = LLC_PAUSE_ENC_STATE_IDLE;

                pHandle->llc.type = LLC_PROCEDURE_TYPE_NONE;

                /* generate Encryption_Key_Refresh_Complete Event to host */
                task_param.Dword = 0;
                task_param.lmp_s.sub_type = LL_TASK_HANDLE_ENCRYPTION_KEY_REFRESH;
                task_param.lmp_s.conn_entry = entry;
                task_param.lmp_s.status = HCI_COMMAND_SUCCEEDED;
                sig_send.type = LL_GEN_HCI_EVENT_SIGNAL;
                sig_send.param = (OS_ADDRESS)task_param.Dword;
                OS_SEND_SIGNAL_TO_TASK (lmp_task_handle, sig_send);
            }
            else
            {
                /* finish start encryption procedure. we can stop and kill
                    llc procedure response timer */
#ifdef USE_FREERTOS
                OS_STOP_TIMER(pHandle->llc_timer, NULL);
#else
                OS_DELETE_TIMER(&pHandle->llc_timer);
#endif

                /* resume acl user data packets */
                pHandle->pause_acl_tx = FALSE;
                pHandle->discard_acl_rx = FALSE;

                pHandle->llc.type = LLC_PROCEDURE_TYPE_NONE;

                LL_LOG_TRACE(WHITE, DAPE_TEST_LOG264, 1,
                             pEnc_blk->start_enc_state);

                /* TODO: we shall send Encryption_Change_Event to Host */
                task_param.Dword = 0;
                task_param.lmp_s.sub_type = LL_TASK_HANDLE_ENCRYPTION_CHANGE_EVENT;
                task_param.lmp_s.reason = 0x01;
                task_param.lmp_s.conn_entry = entry;
                task_param.lmp_s.status = HCI_COMMAND_SUCCEEDED;
                sig_send.type = LL_GEN_HCI_EVENT_SIGNAL;
                sig_send.param = (OS_ADDRESS)task_param.Dword;
                OS_SEND_SIGNAL_TO_TASK (lmp_task_handle, sig_send);
            }
            return;
        }
    }
    else
    {
        /* this shall be the way 2 of three way handshake for slave */
        if (LLC_START_ENC_STATE_S2M_START_ENC_REQ == pEnc_blk->start_enc_state)
        {
#ifndef _ADD_LLC_HANDLE_START_ENC_RES_ACK_RECD_
            pEnc_blk->start_enc_state = LLC_START_ENC_STATE_END;
#endif

            /* TODO: we shall start to encrpt the tx data path then send
               encrpted LL_START_ENC_RSP */

            pTxPkt = llc_generate_ll_start_enc_rsp();
            llc_append_pdu_to_tx_list(pTxPkt, entry);

#ifdef _ADD_LLC_HANDLE_START_ENC_RES_ACK_RECD_
            llc_check_sent_llc_pdu_request(pHandle, LL_START_ENC_RSP);
#endif

            /* Encryption is enable */
            pHandle->encrypted = TRUE;

#ifndef _ADD_LLC_HANDLE_START_ENC_RES_ACK_RECD_
            if (LLC_PAUSE_ENC_STATE_END == pEnc_blk->pause_enc_state)
            {
                /* finish restart encryption procedure. we can stop and kill
                    llc procedure response timer */
                OS_DELETE_TIMER(&pHandle->llc_timer);

                /* resume acl user data packets */
                pHandle->pause_acl_tx = FALSE;
                pHandle->discard_acl_rx = FALSE;

                pEnc_blk->pause_enc_state = LLC_PAUSE_ENC_STATE_IDLE;

                pHandle->llc.type = LLC_PROCEDURE_TYPE_NONE;

                /* send Encryption_Key_Refresh_Complete Event to Host */
                task_param.Dword = 0;
                task_param.lmp_s.sub_type = LL_TASK_HANDLE_ENCRYPTION_KEY_REFRESH;
                task_param.lmp_s.conn_entry = entry;
                task_param.lmp_s.status = HCI_COMMAND_SUCCEEDED;
                sig_send.type = LL_GEN_HCI_EVENT_SIGNAL;
                sig_send.param = (OS_ADDRESS)task_param.Dword;
                OS_SEND_SIGNAL_TO_TASK (lmp_task_handle, sig_send);
            }
            else
            {
                /* finish start encryption procedure. we can stop and kill
                    llc procedure response timer */
                OS_DELETE_TIMER(&pHandle->llc_timer);

                /* resume acl user data packets */
                pHandle->pause_acl_tx = FALSE;
                pHandle->discard_acl_rx = FALSE;

                pHandle->llc.type = LLC_PROCEDURE_TYPE_NONE;

                /* TODO: we shall send Encryption_Change_Event to Host */
                task_param.Dword = 0;
                task_param.lmp_s.sub_type = LL_TASK_HANDLE_ENCRYPTION_CHANGE_EVENT;
                task_param.lmp_s.reason = 0x01;
                task_param.lmp_s.conn_entry = entry;
                task_param.lmp_s.status = HCI_COMMAND_SUCCEEDED;
                sig_send.type = LL_GEN_HCI_EVENT_SIGNAL;
                sig_send.param = (OS_ADDRESS)task_param.Dword;
                OS_SEND_SIGNAL_TO_TASK (lmp_task_handle, sig_send);
            }
#endif
            return;
        }
    }

    /* TODO: we may send LL_REJECT_IND or LL_TERMINATE ? */
    pTxPkt = llc_generate_ll_reject_ind(ENCRYPTION_MODE_NOT_ACCEPTABLE_ERROR);
    llc_append_pdu_to_tx_list(pTxPkt, entry);
    llc_check_sent_llc_pdu_request(pHandle, LL_REJECT_IND);
}

/**************************************************************************
 * Function     : llc_handle_ll_unknown_rsp
 *
 * Description  : This function is used to handle an LL_UNKNOWN_RSP.
 *
 * Parameters   : pdata: the pointer of payload body
 *                len:   the length of payload body
 *                entry: the connection entry index
 *
 * Returns      : None
 *
 *************************************************************************/
void llc_handle_ll_unknown_rsp(UINT8 *pdata, UINT8 len, UINT8 entry)
{
    LL_CTRL_PDU_PAYLOAD *pRxPdu = (LL_CTRL_PDU_PAYLOAD *)pdata;
    LL_CTRL_PDU_PAYLOAD *pTxPkt;
#ifdef _SUPPORT_VER_4_1_
    LL_CONN_HANDLE_UNIT *chu = &ll_manager.conn_unit.handle[entry];
#endif
    UINT8 unknown_type;

    if (len != LL_UNKNOWN_RSP_LEN)
    {
        /* if the payload length is mismatched, we can not accept this request.
           therefore, we will send a LL_UNKNOWN_RSP to the slave */
        pTxPkt = llc_generate_ll_unknown_rsp(LL_UNKNOWN_RSP);
        llc_append_pdu_to_tx_list(pTxPkt, entry);
        return;
    }

    unknown_type = pRxPdu->unknown_rsp.UnknownType;
    switch (unknown_type)
    {
#ifdef _SUPPORT_VER_4_1_
        case LL_SLAVE_FEATURE_REQ_IND:
            if (chu->llc.type == LLC_PROCEDURE_TYPE_FEATURE_EXCHANGE)
            {
                chu->llc.type = LLC_PROCEDURE_TYPE_NONE;
#ifdef USE_FREERTOS
                OS_STOP_TIMER(chu->llc_timer, NULL);
#else
                OS_DELETE_TIMER(&chu->llc_timer);
#endif
            }
            if (chu->hci_cmd_bm.remote_feature)
            {
                chu->hci_cmd_bm.remote_feature = 0;
                hci_generate_le_read_remote_used_features_complete_event(
                                                UNSUPPORTED_REMOTE_FEATURE_ERROR, chu);
            }
            break;

#ifdef _BT4_1_LE_PING_FEATURE_SUPPORT_
        case LL_PING_REQ:
            if (chu->llc.type == LLC_PROCEDURE_TYPE_LE_PING_REQ)
            {
                chu->llc.type = LLC_PROCEDURE_TYPE_NONE;

                /* restart LE ping timer */
                llc_ping_req_restart_timer(entry);
            }
            break;
#endif

#endif /* _SUPPORT_VER_4_1_ */

#ifdef _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_
        case LL_LENGTH_REQ:
            if (chu->llc.type == LLC_PROCEDURE_TYPE_DATA_LENGTH_UPDATE)
            {
                chu->llc.type = LLC_PROCEDURE_TYPE_NONE;
#ifdef USE_FREERTOS
                OS_STOP_TIMER(chu->llc_timer, NULL);
#else
                OS_DELETE_TIMER(&chu->llc_timer);
#endif
#ifdef _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_TRACE_
                RT_BT_LOG(YELLOW, YL_DBG_DEC_2, 2, 141014, entry);
#endif
            }
            break;
#endif
        default:
            break;
    }
}

/**************************************************************************
 * Function     : llc_handle_ll_feature_req
 *
 * Description  : This function is used to handle an LL_FEATURE_REQ.
 *
 * Parameters   : pdata: the pointer of payload body
 *                len:   the length of payload body
 *                entry: the connection entry index
 *
 * Returns      : None
 *
 *************************************************************************/
void llc_handle_ll_feature_req(UINT8 *pdata, UINT8 len, UINT8 entry)
{
    LL_CTRL_PDU_PAYLOAD *pRxPdu = (LL_CTRL_PDU_PAYLOAD *)pdata;
    LL_CTRL_PDU_PAYLOAD *pTxPkt;
    LL_CONN_HANDLE_UNIT *pHandle;

    pHandle = &ll_manager.conn_unit.handle[entry];

    if (ll_manager.conn_unit.master || (len != LL_FEATURE_REQ_LEN))
    {
        /* when we are the master, we can not accept this request.
           therefore, we will send a LL_UNKNOWN_RSP to the slave */
        pTxPkt = llc_generate_ll_unknown_rsp(LL_FEATURE_REQ);
        llc_append_pdu_to_tx_list(pTxPkt, entry);
        return;
    }

    memcpy(pHandle->remote_info.le_features,
           pRxPdu->feature_req.u1FeatureSet, 8);

    /* the FeatureSet field shall be set to featureSet_used */
    if ((pHandle->remote_info.le_features[0] & BIT0) &&
            ll_manager.conn_unit.support_enc)
    {
        pHandle->support_enc = TRUE;
    }
    else
    {
        pHandle->support_enc = FALSE;
    }

#ifdef _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_
    pHandle->support_dle = ((pHandle->remote_info.le_features[0] & BIT5)
            && ll_manager.conn_unit.support_dle);
#endif

    pHandle->llc.type = LLC_PROCEDURE_TYPE_FEATURE_EXCHANGE;

    /* start the LLC procedure timer */
    ll_start_timer(entry, LL_TIMER_TYPE_LLC);

    /* send a LL_FEATURE_RSP to the master */
    pTxPkt = llc_generate_ll_feature_rsp(entry);
    llc_append_pdu_to_tx_list(pTxPkt, entry);
    llc_check_sent_llc_pdu_request(pHandle, LL_FEATURE_RSP);
}

/**************************************************************************
 * Function     : llc_handle_ll_feature_rsp
 *
 * Description  : This function is used to handle an LL_FEATURE_RSP.
 *
 * Parameters   : pdata: the pointer of payload body
 *                len:   the length of payload body
 *                entry: the connection entry index
 *
 * Returns      : None
 *
 *************************************************************************/
void llc_handle_ll_feature_rsp(UINT8 *pdata, UINT8 len, UINT8 entry)
{
    LL_CTRL_PDU_PAYLOAD *pRxPdu = (LL_CTRL_PDU_PAYLOAD *)pdata;
    LL_CTRL_PDU_PAYLOAD *pTxPkt;
    LL_CONN_HANDLE_UNIT *pHandle;

    pHandle = &ll_manager.conn_unit.handle[entry];
#ifdef _SUPPORT_VER_4_1_
    if (((!IS_BT41) && (!IS_BT42) && (!ll_manager.conn_unit.master))
                                            ||(len != LL_FEATURE_RSP_LEN))
#else
    if (!ll_manager.conn_unit.master || (len != LL_FEATURE_RSP_LEN))
#endif
    {
        /* when we are the slave, we can not accept this request.
           therefore, we will send a LL_UNKNOWN_RSP to the slave */
        pTxPkt = llc_generate_ll_unknown_rsp(LL_FEATURE_RSP);
        llc_append_pdu_to_tx_list(pTxPkt, entry);
        return;
    }

    memcpy(pHandle->remote_info.le_features,
           pRxPdu->feature_rsp.u1FeatureSet, 8);

    /* the FeatureSet field shall be set to featureSet_used */
    if (pHandle->remote_info.le_features[0] & BIT0)
    {
        pHandle->support_enc = TRUE;

        if (!ll_manager.conn_unit.support_enc)
        {
            LL_LOG_TRACE(RED, LE_MSG_LLC_FEATURE_SET_VIOLATE, 1, entry);
        }
    }
    else
    {
        pHandle->support_enc = FALSE;
    }

#ifdef _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_
    pHandle->support_dle = ((pHandle->remote_info.le_features[0] & BIT5)
            && ll_manager.conn_unit.support_dle);
#endif

    /* TODO: finish the feature exchange procedure. we can stop and kill
       llc procedure response timer */
#ifdef USE_FREERTOS
    OS_STOP_TIMER(pHandle->llc_timer, NULL);
#else
    OS_DELETE_TIMER(&pHandle->llc_timer);
#endif
    pHandle->llc.type = LLC_PROCEDURE_TYPE_NONE;

    if (pHandle->hci_cmd_bm.remote_feature)
    {
        /* TODO: we sell send a LE Read Remote Used Features Complete Event
           to the Host after the controller has completed the procedure to
           determine the remote features */
        hci_generate_le_read_remote_used_features_complete_event(
                                        HCI_COMMAND_SUCCEEDED, pHandle);
        pHandle->hci_cmd_bm.remote_feature = 0;
    }
}

/**************************************************************************
 * Function     : llc_handle_ll_pause_enc_req
 *
 * Description  : This function is used to handle an LL_PAUSE_ENC_REQ.
 *
 * Parameters   : pdata: the pointer of payload body
 *                len:   the length of payload body
 *                entry: the connection entry index
 *
 * Returns      : None
 *
 *************************************************************************/
void llc_handle_ll_pause_enc_req(UINT8 *pdata, UINT8 len, UINT8 entry)
{
    LL_CTRL_PDU_PAYLOAD *pTxPkt;
    LL_ENCRYPT_BLK *pEnc_blk;
    LL_CONN_HANDLE_UNIT *pHandle;

    pHandle = &ll_manager.conn_unit.handle[entry];

    if (ll_manager.conn_unit.master || (len != LL_PAUSE_ENC_REQ_LEN))
    {
        /* when we are the master, we can not accept this request.
           therefore, we will send a LL_UNKNOWN_RSP to the master */
        pTxPkt = llc_generate_ll_unknown_rsp(LL_PAUSE_ENC_REQ);
        llc_append_pdu_to_tx_list(pTxPkt, entry);
        return;
    }

    if (!pHandle->support_enc)
    {
        /* TODO: send LL_REJECT_IND to the master */
        pTxPkt = llc_generate_ll_reject_ind(UNSUPPORTED_REMOTE_FEATURE_ERROR);
        llc_append_pdu_to_tx_list(pTxPkt, entry);
        llc_check_sent_llc_pdu_request(pHandle, LL_REJECT_IND);
        return;
    }

    /* this shall be the way 1 of three way handshake in the pause procedure */
    pEnc_blk = &pHandle->encrypt_blk;

    if ((LLC_START_ENC_STATE_END == pEnc_blk->start_enc_state) &&
            (LLC_PAUSE_ENC_STATE_IDLE == pEnc_blk->pause_enc_state))
    {
        /* TODO: we shall start to de-encrpt the rx data path then send
           encrpted LL_PAUSE_ENC_RSP to the master */

        /* restart encryption */
        pHandle->llc.type = LLC_PROCEDURE_TYPE_ENCRYPTION_PAUSE;

        /* start the LLC procedure timer */
        ll_start_timer(entry, LL_TIMER_TYPE_LLC);

        /* discard any acl user data packets */
        pHandle->pause_acl_tx = TRUE;
        pHandle->discard_acl_rx = TRUE;

        pTxPkt = llc_generate_ll_pause_enc_rsp();
        llc_append_pdu_to_tx_list(pTxPkt, entry);
        pEnc_blk->pause_enc_state = LLC_PAUSE_ENC_STATE_S2M_PAUSE_ENC_RSP;
        return;
    }

    /* TODO: we may send LL_REJECT_IND or LL_TERMINATE ? */
    pTxPkt = llc_generate_ll_reject_ind(ENCRYPTION_MODE_NOT_ACCEPTABLE_ERROR);
    llc_append_pdu_to_tx_list(pTxPkt, entry);
    llc_check_sent_llc_pdu_request(pHandle, LL_REJECT_IND);
}

/**************************************************************************
 * Function     : llc_handle_ll_pause_enc_rsp
 *
 * Description  : This function is used to handle an LL_PAUSE_ENC_RSP.
 *
 * Parameters   : pdata: the pointer of payload body
 *                len:   the length of payload body
 *                entry: the connection entry index
 *
 * Returns      : None
 *
 *************************************************************************/
void llc_handle_ll_pause_enc_rsp(UINT8 *pdata, UINT8 len, UINT8 entry)
{
    LL_CTRL_PDU_PAYLOAD *pTxPkt;
    LL_ENCRYPT_BLK *pEnc_blk;
    LL_CONN_HANDLE_UNIT *pHandle;

    pHandle = &ll_manager.conn_unit.handle[entry];

    if (len != LL_PAUSE_ENC_RSP_LEN)
    {
        /* when we are the master, we can not accept this request.
           therefore, we will send a LL_UNKNOWN_RSP to the master */
        pTxPkt = llc_generate_ll_unknown_rsp(LL_PAUSE_ENC_RSP);
        llc_append_pdu_to_tx_list(pTxPkt, entry);
        return;
    }

    if (!ll_manager.conn_unit.handle[entry].support_enc)
    {
        /* TODO: send LL_REJECT_IND to the master */
        pTxPkt = llc_generate_ll_reject_ind(UNSUPPORTED_REMOTE_FEATURE_ERROR);
        llc_append_pdu_to_tx_list(pTxPkt, entry);
        llc_check_sent_llc_pdu_request(pHandle, LL_REJECT_IND);
        return;
    }

    pEnc_blk = &ll_manager.conn_unit.handle[entry].encrypt_blk;

    if (ll_manager.conn_unit.master)
    {
        /* this shall be the way 2 of three way handshake for master */
        if (LLC_PAUSE_ENC_STATE_M2S_PAUSE_ENC_REQ == pEnc_blk->pause_enc_state)
        {
            /* Unencryption is enabled */

            /* discard any acl user data packets */
            pHandle->discard_acl_rx = TRUE;

            /* TODO: we shall start to unencrpt the tx/rx data path then send
               unencrpted LL_PAUSE_ENC_RSP */
            pTxPkt = llc_generate_ll_pause_enc_rsp();
            llc_append_pdu_to_tx_list(pTxPkt, entry);

            pEnc_blk->pause_enc_state = LLC_PAUSE_ENC_STATE_END;
            pEnc_blk->start_enc_state = LLC_START_ENC_STATE_IDLE;
            pHandle->install_key = 0;

            /* TODO: we shall send LL_ENC_REQ to start re-encrpt procedure */
            pTxPkt = llc_generate_ll_enc_req(entry);
            llc_append_pdu_to_tx_list(pTxPkt, entry);
            return;
        }
    }
    else
    {
        /* this shall be the way 3 of three way handshake for slave */
        if (LLC_PAUSE_ENC_STATE_S2M_PAUSE_ENC_RSP == pEnc_blk->pause_enc_state)
        {
            pEnc_blk->pause_enc_state = LLC_PAUSE_ENC_STATE_END;
            pEnc_blk->start_enc_state = LLC_START_ENC_STATE_IDLE;
            pHandle->install_key = 0;

            /* Unencryption is enabled */

            /* TODO: we shall start to unencrpt the tx data path */
            return;
        }
    }

    /* TODO: we may send LL_REJECT_IND or LL_TERMINATE ? */
    pTxPkt = llc_generate_ll_reject_ind(ENCRYPTION_MODE_NOT_ACCEPTABLE_ERROR);
    llc_append_pdu_to_tx_list(pTxPkt, entry);
    llc_check_sent_llc_pdu_request(pHandle, LL_REJECT_IND);
}

/**************************************************************************
 * Function     : llc_handle_ll_version_ind
 *
 * Description  : This function is used to handle an LL_VERSION_IND.
 *
 * Parameters   : pdata: the pointer of payload body
 *                len:   the length of payload body
 *                entry: the connection entry index
 *
 * Returns      : None
 *
 *************************************************************************/
void llc_handle_ll_version_ind(UINT8 *pdata, UINT8 len, UINT8 entry)
{
    LL_CTRL_PDU_PAYLOAD *pRxPdu = (LL_CTRL_PDU_PAYLOAD *)pdata;
    LL_CTRL_PDU_PAYLOAD *pTxPkt;
    LL_CONN_REMOTE_INFO *pRemInfo;
    LL_CONN_HANDLE_UNIT *pHandle;

    pHandle = &ll_manager.conn_unit.handle[entry];

#ifdef _FIX_BQB_LL_TP_SEC_SLA_BI_05_C_ISSUE_
    if (!ll_manager.conn_unit.master &&
        (pHandle->encrypt_blk.start_enc_state == LLC_START_ENC_STATE_BEGIN))
    {
        /* receive unexpected pdu -
           Test that a slave IUT which has started the encryption procedure does
           not respond to an LL_VERSION_IND but instead drops the link.*/
        pHandle->kill_link_reason = LL_KILL_REASON_MIC_ERROR;

        /* need to kill le connection */
        ll_driver_kill_connection(entry);
        return;
    }
#endif

    if (len != LL_VERSION_IND_LEN)
    {
        /* the length is mismatch.
           therefore, we will send a LL_UNKNOWN_RSP to the master */
        pTxPkt = llc_generate_ll_unknown_rsp(LL_VERSION_IND);
        llc_append_pdu_to_tx_list(pTxPkt, entry);
        return;
    }

    /* save the information of LL_VERSION_IND to local from remote device */
    pRemInfo = &pHandle->remote_info;
    pRemInfo->vers_nr = pRxPdu->version_ind.VersNr;
    pRemInfo->comp_id = *(UINT16*)pRxPdu->version_ind.u1CompId;
    pRemInfo->subvers_nr = *(UINT16*)pRxPdu->version_ind.u1SubVersNr;

    pHandle->llc.get_version_ind = TRUE;

    /* check we have already sent an LL_VERSION_IND pdu to the remote device */
    if (!pHandle->llc.sent_version_ind)
    {
#ifdef _FIX_BQB_LL_TP_SEC_MAS_BV_12_C_ISSUE_
        if (pHandle->llc.type != LLC_PROCEDURE_TYPE_NONE)
        {
            /* we need to wait the completion of previous LLC procedure */
            pHandle->llc.pend_version_ex = TRUE;
            return;
        }
#endif

        pHandle->llc.type = LLC_PROCEDURE_TYPE_VERSION_EXCHANGE;

        /* start the LLC procedure timer */
        ll_start_timer(entry, LL_TIMER_TYPE_LLC);

        pHandle->llc.sent_version_ind = TRUE;
        pTxPkt = llc_generate_ll_version_ind();
        llc_append_pdu_to_tx_list(pTxPkt, entry);
        llc_check_sent_llc_pdu_request(pHandle, LL_VERSION_IND);
    }
    else
    {
        /* TODO: finish the feature exchange procedure. we can stop and kill
           llc procedure response timer */
#ifdef USE_FREERTOS
        OS_STOP_TIMER(pHandle->llc_timer, NULL);
#else
        OS_DELETE_TIMER(&pHandle->llc_timer);
#endif
        pHandle->llc.type = LLC_PROCEDURE_TYPE_NONE;
    }

    /* check we have already owe an REMOTE Version Information Complete Event
       to Host */
    if (ll_manager.conn_unit.handle[entry].hci_cmd_bm.remote_version)
    {
        /*Send remote version information complete Event */
        hci_generate_remote_version_information_complete_event(
                                HCI_COMMAND_SUCCEEDED,
                                pHandle->conn_handle,
                                entry + LMP_MAX_CE_DATABASE_ENTRIES);
        ll_manager.conn_unit.handle[entry].hci_cmd_bm.remote_version = 0;
    }
}

void llc_stop_procedure_when_rejected(LL_CONN_HANDLE_UNIT *chu, UINT8 err_code)
{
    UINT8 llc_type = chu->llc.type;

    if (llc_type == LLC_PROCEDURE_TYPE_NONE)
    {
        return;
    }

#ifdef _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_
    if (llc_type == LLC_PROCEDURE_TYPE_DATA_LENGTH_UPDATE
            && chu->data_len_updt.state == LLC_DATA_LEN_UPDATE_STATE_RECV_REQ_WITH_CONFLICT)
    {
        /* In this case, master rejects slave's LL_LENGTH_REQ. Because slave
         * still accepts master's LL_LENGTH_REQ, the data length update
         * procedure should keep going.
         */
        return;
    }
#endif

    if (llc_type == LLC_PROCEDURE_TYPE_ENCRYPTION_START
            || llc_type == LLC_PROCEDURE_TYPE_ENCRYPTION_PAUSE)
    {
        /* resume any acl user data packets */
        chu->pause_acl_tx = FALSE;
        chu->discard_acl_rx = FALSE;

        LL_TASK_PARA_U task_param = { .Dword = 0 };
        task_param.lmp_s.sub_type = LL_TASK_HANDLE_ENCRYPTION_CHANGE_EVENT;
        task_param.lmp_s.reason = 0x00;
        task_param.lmp_s.conn_entry = chu->unit_id;
        task_param.lmp_s.status = err_code;

        OS_SIGNAL sig_send = {
                .type = LL_GEN_HCI_EVENT_SIGNAL,
                .param = (OS_ADDRESS) task_param.Dword
        };
        OS_SEND_SIGNAL_TO_TASK (lmp_task_handle, sig_send);
    }

#ifdef USE_FREERTOS
    OS_STOP_TIMER(chu->llc_timer, NULL);
#else
    OS_DELETE_TIMER(&chu->llc_timer);
#endif
    chu->llc.type = LLC_PROCEDURE_TYPE_NONE;
}

/**************************************************************************
 * Function     : llc_handle_ll_reject_ind
 *
 * Description  : This function is used to handle an LL_REJECT_IND.
 *
 * Parameters   : pdata: the pointer of payload body
 *                len:   the length of payload body
 *                entry: the connection entry index
 *
 * Returns      : None
 *
 *************************************************************************/
void llc_handle_ll_reject_ind(UINT8 *pdata, UINT8 len, UINT8 entry)
{
    LL_CTRL_PDU_PAYLOAD *pRxPdu = (LL_CTRL_PDU_PAYLOAD *) pdata;
    LL_CONN_HANDLE_UNIT *pHandle = &ll_manager.conn_unit.handle[entry];

    if (len != LL_REJECT_IND_LEN)
    {
        /* we can not accept this request.
           therefore, we will send a LL_UNKNOWN_RSP to the master */
        LL_CTRL_PDU_PAYLOAD *pTxPkt = llc_generate_ll_unknown_rsp(LL_REJECT_IND);
        llc_append_pdu_to_tx_list(pTxPkt, entry);
        return;
    }

    llc_stop_procedure_when_rejected(pHandle, pRxPdu->reject_ind.ErrCode);
}

void llc_handle_ll_todo_pdu(UINT8 *pdata, UINT8 len, UINT8 entry)
{
    LL_CTRL_PDU_PAYLOAD *pCtrlPayload = (LL_CTRL_PDU_PAYLOAD *)pdata;
    UINT8 opcode = pCtrlPayload->OpCode;
    LL_CTRL_PDU_PAYLOAD *pTxPkt;

    LL_LOG_TRACE(YELLOW, LE_MSG_INVALID_LLC_PDU, 1, pCtrlPayload->OpCode);

    /* generate LL_UNKNOWN_RES*/
    pTxPkt = llc_generate_ll_unknown_rsp(opcode);
    llc_append_pdu_to_tx_list(pTxPkt, entry);
}

#ifdef _SUPPORT_VER_4_1_
/**************************************************************************
 * Function     : llc_generate_ll_ping_req
 *
 * Description  : This function is used to generate an LL_PING_REQ.
 *
 * Parameters   : entry: the connection entry index
 *
 * Returns      : None
 *
 *************************************************************************/
LL_CTRL_PDU_PAYLOAD* llc_generate_ll_ping_req(UINT8 conn_entry)
{
    LL_CTRL_PDU_PAYLOAD *pdu;

    if (API_FAILURE == llc_generate_pdu(LL_PING_REQ, &pdu))
    {
        return NULL;
    }

    LL_LOG_TRACE(GREEN, LE_MSG_GEN_LL_PING_REQ, 1, conn_entry);
    return pdu;
}
/**************************************************************************
 * Function     : llc_generate_ll_ping_rsp
 *
 * Description  : This function is used to generate an LL_PING_RSP.
 *
 * Parameters   : entry: the connection entry index
 *
 * Returns      : None
 *
 *************************************************************************/
LL_CTRL_PDU_PAYLOAD* llc_generate_ll_ping_rsp(UINT8 conn_entry)
{
    LL_CTRL_PDU_PAYLOAD *pdu;

    if (API_FAILURE == llc_generate_pdu(LL_PING_RSP, &pdu))
    {
        return NULL;
    }

    LL_LOG_TRACE(GREEN, LE_MSG_GEN_LL_PING_RSP, 1, conn_entry);
    return pdu;
}

/**************************************************************************
 * Function     : llc_generate_ll_slave_feature_req
 *
 * Description  : This function is used to generate a LL_SLAVE_FEATURE_REQ.
 *                The pdu body is from the le_llc_pdu_pool_id in SRAM.
 *
 * Parameters   : None
 *
 * Returns      : the pointer of LL control pdu payload or NULL
 *
 *************************************************************************/
LL_CTRL_PDU_PAYLOAD* llc_generate_ll_slave_feature_req(void)
{
    LL_CTRL_PDU_PAYLOAD *pPkt = NULL;

    if (API_FAILURE == llc_generate_pdu(LL_SLAVE_FEATURE_REQ_IND, &pPkt))
    {
        return NULL;
    }

    /* fill CtrlData of LL_FEATURE_REQ */
    memset(pPkt->feature_req.u1FeatureSet, 0, 8);
#ifdef _DAPE_UNIFY_VERSION_FEATURE_UINT
    memcpy(&pPkt->feature_req.u1FeatureSet[0], &ll_manager.ll_feature_support[0], 8);
#else
    pPkt->feature_rsp.u1FeatureSet[0] = (
            ll_manager.conn_unit.support_enc
#ifdef _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_
            | (ll_manager.conn_unit.support_dle << 5)
#endif
            );
    /* if support bt4.1, then directly enable these two features bits
       since these two are very easy. */
    if (IS_BT41)
    {
        pPkt->feature_rsp.u1FeatureSet[0] |= (SUPPORT_REJ_IND_EXT |
                                               SUPPORT_LE_PING | SUPPORT_SLV_INIT_FEATURE_REQ);
    }
#endif
    LL_LOG_TRACE(GREEN, LE_MSG_GEN_LL_SLV_FEATURE_REQ, 8,
                pPkt->u1CtrData[0], pPkt->u1CtrData[1], pPkt->u1CtrData[2],
                pPkt->u1CtrData[3], pPkt->u1CtrData[4], pPkt->u1CtrData[5],
                pPkt->u1CtrData[6], pPkt->u1CtrData[7]);

    return pPkt;
}
#ifdef _BT4_1_SUPPORT_LL_CONN_PARAM_REQ
/**************************************************************************
 * Function     : llc_generate_ll_connection_parameter_req
 *
 * Description  : This function is used to generate a LL_CONNECTION_PARAM_REQ.
 *                The pdu body is from the le_llc_pdu_pool_id in SRAM. The
 *                slave shall not send this PDU.
 *
 * Parameters   : conn_entry: the connection entry id
 *
 * Returns      : the pointer of LL control pdu payload or NULL
 *
 *************************************************************************/
LL_CTRL_PDU_PAYLOAD* llc_generate_ll_connection_parameter_req(UINT8 conn_entry)
{
    LL_CTRL_PDU_PAYLOAD *pPkt = NULL;
    LL_CONN_PARAM_REQ_BLK *pConnParamReqBlk = NULL;

    if (conn_entry > LL_MAX_CONNECTION_UNITS)
    {
        return NULL;
    }

    if (API_FAILURE == llc_generate_pdu(LL_CONNECTION_PARAM_REQ, &pPkt))
    {
        return NULL;
    }

    pConnParamReqBlk = &ll_manager.conn_unit.handle[conn_entry].conn_param_req_blk;

    /* fill CtrlData of LL_CONNECTION_PARAM_REQ */
    htole16(pConnParamReqBlk->Interval_Min, pPkt->conn_param_req.Interval_Min);
    htole16(pConnParamReqBlk->Interval_Max, pPkt->conn_param_req.Interval_Max);
    htole16(pConnParamReqBlk->Latency, pPkt->conn_param_req.Latency);
    htole16(pConnParamReqBlk->Timeout, pPkt->conn_param_req.Timeout);
    pPkt->conn_param_req.PreferredPeriodicity = pConnParamReqBlk->PreferredPeriodicity;
    htole16(pConnParamReqBlk->RefConnEventCount, pPkt->conn_param_req.ReferenceConnEventCount);
    htole16(pConnParamReqBlk->Offset0, pPkt->conn_param_req.Offset0);
    htole16(pConnParamReqBlk->Offset1, pPkt->conn_param_req.Offset1);
    htole16(pConnParamReqBlk->Offset2, pPkt->conn_param_req.Offset2);
    htole16(pConnParamReqBlk->Offset3, pPkt->conn_param_req.Offset3);
    htole16(pConnParamReqBlk->Offset4, pPkt->conn_param_req.Offset4);
    htole16(pConnParamReqBlk->Offset5, pPkt->conn_param_req.Offset5);

#ifdef _ROM_CODE_PATCHED_
    if (rcp_ll_generate_conn_para_req_func != NULL)
    {
        if(rcp_ll_generate_conn_para_req_func((void*)&conn_entry, pPkt))
        {
            return pPkt;
        }
    }
#endif

    LL_LOG_TRACE(GREEN, LE_MSG_GEN_LL_CONN_PARAM_REQ, 16,
        conn_entry,
        pConnParamReqBlk->Interval_Min,
        pConnParamReqBlk->Interval_Max,
        ll_manager.conn_unit.handle[conn_entry].ce_interval,

        pConnParamReqBlk->Latency,
        ll_manager.conn_unit.handle[conn_entry].slave_latency,

        pConnParamReqBlk->Timeout,
        ll_manager.conn_unit.handle[conn_entry].supervision_to,

        pConnParamReqBlk->PreferredPeriodicity,
        pConnParamReqBlk->RefConnEventCount,
        pConnParamReqBlk->Offset0, pConnParamReqBlk->Offset1,
        pConnParamReqBlk->Offset2, pConnParamReqBlk->Offset3,
        pConnParamReqBlk->Offset4, pConnParamReqBlk->Offset5);

    return pPkt;
}

#endif

/**************************************************************************
 * Function     : llc_handle_ll_slave_feature_req
 *
 * Description  : This function is used to handle an LL_SLAVE_FEATURE_REQ.
 *
 * Parameters   : pdata: the pointer of payload body
 *                len:   the length of payload body
 *                entry: the connection entry index
 *
 * Returns      : None
 *
 *************************************************************************/
void llc_handle_ll_slave_feature_req(UINT8 *pdata, UINT8 len, UINT8 entry)
{
    LL_CTRL_PDU_PAYLOAD *pRxPdu = (LL_CTRL_PDU_PAYLOAD *)pdata;
    LL_CTRL_PDU_PAYLOAD *pTxPkt;
    LL_CONN_HANDLE_UNIT *pHandle;

    pHandle = &ll_manager.conn_unit.handle[entry];
#ifdef _TP_SEC_SLV_BI06C_LOWER_TESTER
    return;
#endif
#ifndef _TP_SEC_SLV_BV23C_LOWER_TESTER
    if (!ll_manager.conn_unit.master || (len != LL_SLAVE_FEATURE_REQ_LEN) ||
        ((!IS_BT41) && (!IS_BT42)))
#endif
    {
        /* when we are the slave, we can not accept this request.
           therefore, we will send a LL_UNKNOWN_RSP to the master */
        pTxPkt = llc_generate_ll_unknown_rsp(LL_SLAVE_FEATURE_REQ_IND);
        llc_append_pdu_to_tx_list(pTxPkt, entry);
        return;
    }

    memcpy(pHandle->remote_info.le_features,
           pRxPdu->feature_req.u1FeatureSet, 8);

    /* the FeatureSet field shall be set to featureSet_used */
    if ((pHandle->remote_info.le_features[0] & BIT0) &&
            ll_manager.conn_unit.support_enc)
    {
        pHandle->support_enc = TRUE;
    }
    else
    {
        pHandle->support_enc = FALSE;
    }

    pHandle->llc.type = LLC_PROCEDURE_TYPE_FEATURE_EXCHANGE;

    /* start the LLC procedure timer */
    ll_start_timer(entry, LL_TIMER_TYPE_LLC);

    /* send a LL_FEATURE_RSP to the slave */
    pTxPkt = llc_generate_ll_feature_rsp(entry);
    llc_append_pdu_to_tx_list(pTxPkt, entry);
    llc_check_sent_llc_pdu_request(pHandle, LL_FEATURE_RSP);
}
/**************************************************************************
 * Function     : llc_handle_ll_connection_param_req
 *
 * Description  : This function is used to handle an LL_CONNECTION_PARAM_REQ.
 *
 * Parameters   : pdata: the pointer of payload body
 *                len:   the length of payload body
 *                entry: the connection entry index
 *
 * Returns      : None
 *
 *************************************************************************/
void llc_handle_ll_connection_param_req(UINT8 *pdata, UINT8 len, UINT8 entry)
{
    LL_CONN_HANDLE_UNIT *pHandle;
    LL_CTRL_PDU_PAYLOAD *pTxPkt;

    pHandle = &ll_manager.conn_unit.handle[entry];

    pTxPkt = llc_generate_ll_unknown_rsp(LL_CONNECTION_PARAM_REQ);
    llc_append_pdu_to_tx_list(pTxPkt, entry);

    return;
}
/**************************************************************************
 * Function     : llc_handle_ll_connection_param_rsp
 *
 * Description  : This function is used to handle an LL_CONNECTION_PARAM_RSP.
 *
 * Parameters   : pdata: the pointer of payload body
 *                len:   the length of payload body
 *                entry: the connection entry index
 *
 * Returns      : None
 *
 *************************************************************************/
void llc_handle_ll_connection_param_rsp(UINT8 *pdata, UINT8 len, UINT8 entry)
{
    LL_CONN_HANDLE_UNIT *pHandle;
    LL_CTRL_PDU_PAYLOAD *pTxPkt;

    pHandle = &ll_manager.conn_unit.handle[entry];

    pTxPkt = llc_generate_ll_unknown_rsp(LL_CONNECTION_PARAM_RSP);
    llc_append_pdu_to_tx_list(pTxPkt, entry);

    return;
}
/**************************************************************************
 * Function     : llc_handle_ll_reject_ind_ext
 *
 * Description  : This function is used to handle an LL_REJECT_IND_EXT.
 *
 * Parameters   : pdata: the pointer of payload body
 *                len:   the length of payload body
 *                entry: the connection entry index
 *
 * Returns      : None
 *
 *************************************************************************/
void llc_handle_ll_reject_ind_ext(UINT8 *pdata, UINT8 len, UINT8 entry)
{
    LL_CTRL_PDU_PAYLOAD *pdu = (LL_CTRL_PDU_PAYLOAD *) pdata;
    LL_CONN_HANDLE_UNIT *pHandle = &ll_manager.conn_unit.handle[entry];

    if (len != LL_REJECT_IND_EXT_LEN)
    {
        llc_stop_procedure_by_ll_unknown_rsp(pHandle, LL_REJECT_IND_EXT);
        return;
    }

    llc_stop_procedure_when_rejected(pHandle, pdu->reject_ind_ext.err_code);
}
/**************************************************************************
 * Function     : llc_handle_ll_ping_req
 *
 * Description  : This function is used to handle an LL_PING_REQ.
 *
 * Parameters   : pdata: the pointer of payload body
 *                len:   the length of payload body
 *                entry: the connection entry index
 *
 * Returns      : None
 *
 *************************************************************************/
void llc_handle_ll_ping_req(UINT8 *pdata, UINT8 len, UINT8 entry)
{
    LL_CTRL_PDU_PAYLOAD *pTxPkt;

    if ((len != LL_PING_REQ_LEN) || ((!IS_BT41) && (!IS_BT42)))
    {
        pTxPkt = llc_generate_ll_unknown_rsp(LL_PING_REQ);
        llc_append_pdu_to_tx_list(pTxPkt, entry);
        return;
    }

    /* send a LL_PING_RSP to the remote */
    pTxPkt = llc_generate_ll_ping_rsp(entry);
    llc_append_pdu_to_tx_list(pTxPkt, entry);
}
/**************************************************************************
 * Function     : llc_handle_ll_ping_rsp
 *
 * Description  : This function is used to handle an LL_PING_RSP.
 *
 * Parameters   : pdata: the pointer of payload body
 *                len:   the length of payload body
 *                entry: the connection entry index
 *
 * Returns      : None
 *
 *************************************************************************/
void llc_handle_ll_ping_rsp(UINT8 *pdata, UINT8 len, UINT8 entry)
{
    LL_CTRL_PDU_PAYLOAD *pTxPkt;

    if ((len != LL_PING_RSP_LEN) || ((!IS_BT41) && (!IS_BT42)))
    {
        pTxPkt = llc_generate_ll_unknown_rsp(LL_PING_RSP);
        llc_append_pdu_to_tx_list(pTxPkt, entry);
        return;
    }

#ifdef _BT4_1_LE_PING_FEATURE_SUPPORT_
    /* restart LE ping timer */
    llc_ping_req_restart_timer(entry);
#endif

}

#endif /* _SUPPORT_VER_4_1_ */

#ifdef _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_
/**
 * @brief Generate LLC_LENGTH_REQ PDU.
 *
 * This functions fills the contents of PDU with data members in \p chu. You
 * need to update those values before calling this function.
 * @param chu  Connection handle unit.
 * @return  A LL_CTRL_PDU_PAYLOAD instance if success; \c NULL on error.
 */
LL_CTRL_PDU_PAYLOAD *llc_generate_ll_length_req(LL_CONN_HANDLE_UNIT *chu)
{
    LL_CTRL_PDU_PAYLOAD *pdu;

    if (API_FAILURE == llc_generate_pdu(LL_LENGTH_REQ, &pdu))
    {
        return NULL;
    }

    UINT16 max_rx_size = chu->data_len_updt.local_max_rx_size;
    UINT16 max_rx_time = chu->data_len_updt.local_max_rx_time;
    UINT16 max_tx_size = chu->data_len_updt.local_max_tx_size;
    UINT16 max_tx_time = chu->data_len_updt.local_max_tx_time;
    htole16(max_rx_size, pdu->len_rsp.max_rx_octets);
    htole16(max_rx_time, pdu->len_rsp.max_rx_time);
    htole16(max_tx_size, pdu->len_rsp.max_tx_octets);
    htole16(max_tx_time, pdu->len_rsp.max_tx_time);

    LL_LOG_TRACE(GREEN, LE_MSG_GEN_LL_LENGTH_REQ, 4, max_rx_size, max_rx_time,
            max_tx_size, max_tx_time);
    return pdu;
}

LL_CTRL_PDU_PAYLOAD *llc_generate_ll_length_rsp(LL_CONN_HANDLE_UNIT *chu)
{
    LL_CTRL_PDU_PAYLOAD *pdu;

    if (API_FAILURE == llc_generate_pdu(LL_LENGTH_RSP, &pdu))
    {
        return NULL;
    }

    UINT16 max_rx_size = chu->data_len_updt.local_max_rx_size;
    UINT16 max_rx_time = chu->data_len_updt.local_max_rx_time;
    UINT16 max_tx_size = chu->data_len_updt.local_max_tx_size;
    UINT16 max_tx_time = chu->data_len_updt.local_max_tx_time;
    htole16(max_rx_size, pdu->len_rsp.max_rx_octets);
    htole16(max_rx_time, pdu->len_rsp.max_rx_time);
    htole16(max_tx_size, pdu->len_rsp.max_tx_octets);
    htole16(max_tx_time, pdu->len_rsp.max_tx_time);

    LL_LOG_TRACE(GREEN, LE_MSG_GEN_LL_LENGTH_RSP, 4, max_rx_size, max_rx_time,
            max_tx_size, max_tx_time);
    return pdu;
}

void llc_handle_ll_length_req(UINT8 *data, UINT8 len, UINT8 entry)
{
    LL_CTRL_PDU_PAYLOAD *pdu = (LL_CTRL_PDU_PAYLOAD *) data;
    LL_CONN_HANDLE_UNIT *chu = &ll_manager.conn_unit.handle[entry];
    LLC_DATA_LEN_UPDATE_STATE state = LLC_DATA_LEN_UPDATE_STATE_RECV_REQ;

    if ((len != LL_LENGTH_REQ_LEN) || (!ll_manager.conn_unit.support_dle))
    {
        llc_stop_procedure_by_ll_unknown_rsp(chu, LL_LENGTH_REQ);
        return;
    }

    if (chu->llc.type == LLC_PROCEDURE_TYPE_DATA_LENGTH_UPDATE)
    {
        /* If conflict occurs, master rejects slave's LL_LENGTH_REQ and slave
         * accepts master's LL_LENGTH_REQ.
         */
        if (ll_manager.conn_unit.master)
        {
            LL_CTRL_PDU_PAYLOAD *rej_pdu = llc_generate_ll_reject_ind_ext(
                    LL_LENGTH_REQ, LMP_ERROR_TRANSACTION_COLLISION_ERROR);
            if (rej_pdu != NULL)
            {
                llc_append_pdu_to_tx_list(rej_pdu, entry);
            }
            LL_LOG_TRACE(RED, LE_MSG_LL_LENGTH_REQ_CONFLICT, 1, entry);
            return;
        }
        else
        {
            /* Prepare to schedule a new length update procedure. */
            chu->llc.type = LLC_PROCEDURE_TYPE_NONE;

            state = LLC_DATA_LEN_UPDATE_STATE_RECV_REQ_WITH_CONFLICT;
        }
    }
    else if (chu->llc.type != LLC_PROCEDURE_TYPE_NONE)
    {
        LL_CTRL_PDU_PAYLOAD *rej_pdu = llc_generate_ll_reject_ind_ext(
                LL_LENGTH_REQ, DIFFERENT_TRANSACTION_COLLISION);
        if (rej_pdu != NULL)
        {
            llc_append_pdu_to_tx_list(rej_pdu, entry);
        }
        return;
    }

    /* Clear our pending/delayed data length update request because we use the
     * data length update procedure initiated by the remote device.
     */
    chu->llc.delay_data_len_updt = FALSE;
    chu->llc.pend_data_len_updt = FALSE;

    UINT16 remote_max_rx_size = letoh16(pdu->len_req.max_rx_octets);
    UINT16 remote_max_rx_time = letoh16(pdu->len_req.max_rx_time);
    UINT16 remote_max_tx_size = letoh16(pdu->len_req.max_tx_octets);
    UINT16 remote_max_tx_time = letoh16(pdu->len_req.max_tx_time);

#ifdef _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_TRACE_
    RT_BT_LOG(YELLOW, YL_DBG_DEC_5, 5, 14101500, remote_max_rx_size,
            remote_max_rx_time, remote_max_tx_size, remote_max_tx_time);
#endif

    if (!ll_fw_validate_ll_length_param(remote_max_rx_size, remote_max_rx_time)
            || !ll_fw_validate_ll_length_param(remote_max_tx_size, remote_max_tx_time))
    {
        llc_stop_procedure_by_ll_reject_ind_ext(chu, LL_LENGTH_REQ,
                INVALID_LMP_PARAMETERS_ERROR);
        return;
    }

    chu->data_len_updt.remote_max_rx_size = remote_max_rx_size;
    chu->data_len_updt.remote_max_rx_time = remote_max_rx_time;
    chu->data_len_updt.remote_max_tx_size = remote_max_tx_size;
    chu->data_len_updt.remote_max_tx_time = remote_max_tx_time;

    llc_schedule_data_length_update_procedure(chu, state);
}

void llc_handle_ll_length_rsp(UINT8 *data, UINT8 len, UINT8 entry)
{
    LL_CTRL_PDU_PAYLOAD *pdu = (LL_CTRL_PDU_PAYLOAD *) data;
    LL_CONN_HANDLE_UNIT *chu = &ll_manager.conn_unit.handle[entry];

    if ((len != LL_LENGTH_RSP_LEN) || !ll_manager.conn_unit.support_dle)
    {
        llc_stop_procedure_by_ll_unknown_rsp(chu, LL_LENGTH_RSP);
        return;
    }

    if (chu->llc.type != LLC_PROCEDURE_TYPE_DATA_LENGTH_UPDATE)
    {
        LL_CTRL_PDU_PAYLOAD *rej_pdu = llc_generate_ll_reject_ind_ext(
                LL_LENGTH_RSP, PDU_NOT_ALLOWED_ERROR);
        if (rej_pdu != NULL)
        {
            llc_append_pdu_to_tx_list(rej_pdu, entry);
        }
        LL_LOG_TRACE(RED, LE_MSG_LL_LENGTH_RSP_NOT_ALLOWED, 1, entry);
        return;
    }

    UINT16 remote_max_rx_size = letoh16(pdu->len_req.max_rx_octets);
    UINT16 remote_max_rx_time = letoh16(pdu->len_req.max_rx_time);
    UINT16 remote_max_tx_size = letoh16(pdu->len_req.max_tx_octets);
    UINT16 remote_max_tx_time = letoh16(pdu->len_req.max_tx_time);

#ifdef _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_TRACE_
    RT_BT_LOG(YELLOW, YL_DBG_DEC_5, 5, 14101501, remote_max_rx_size,
            remote_max_rx_time, remote_max_tx_size, remote_max_tx_time);
#endif

    if (!ll_fw_validate_ll_length_param(remote_max_rx_size, remote_max_rx_time)
            || !ll_fw_validate_ll_length_param(remote_max_tx_size, remote_max_tx_time))
    {
        llc_stop_procedure_by_ll_reject_ind_ext(chu, LL_LENGTH_RSP,
                INVALID_LMP_PARAMETERS_ERROR);
        return;
    }

    chu->data_len_updt.remote_max_rx_size = remote_max_rx_size;
    chu->data_len_updt.remote_max_rx_time = remote_max_rx_time;
    chu->data_len_updt.remote_max_tx_size = remote_max_tx_size;
    chu->data_len_updt.remote_max_tx_time = remote_max_tx_time;

    llc_finish_data_length_update_procedure(chu);
}

#define LLC_HANDLE_LL_LENGTH_REQ llc_handle_ll_length_req
#define LLC_HANDLE_LL_LENGTH_RSP llc_handle_ll_length_rsp
#else
#define LLC_HANDLE_LL_LENGTH_REQ llc_handle_ll_todo_pdu
#define LLC_HANDLE_LL_LENGTH_RSP llc_handle_ll_todo_pdu
#endif /* _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_ */

/*************************************************************************
* The function table of HCI LE Control Command Handling
*************************************************************************/
void (*(llc_handle_ll_ctrl_pdu[])) (UINT8 *pdata, UINT8 len, UINT8 entry) =
{
    llc_handle_ll_connection_update_req,
    llc_handle_ll_channel_map_req,
    llc_handle_ll_terminate_ind,
    llc_handle_ll_enc_req,
    llc_handle_ll_enc_rsp,
    llc_handle_ll_start_enc_req,
    llc_handle_ll_start_enc_rsp,
    llc_handle_ll_unknown_rsp,
    llc_handle_ll_feature_req,
    llc_handle_ll_feature_rsp,
    llc_handle_ll_pause_enc_req,
    llc_handle_ll_pause_enc_rsp,
    llc_handle_ll_version_ind,
    llc_handle_ll_reject_ind,
#ifdef _SUPPORT_VER_4_1_
    llc_handle_ll_slave_feature_req,
    llc_handle_ll_connection_param_req,
    llc_handle_ll_connection_param_rsp,
    llc_handle_ll_reject_ind_ext,
    llc_handle_ll_ping_req,
    llc_handle_ll_ping_rsp,
#endif
    [0x14] = LLC_HANDLE_LL_LENGTH_REQ,
    [0x15] = LLC_HANDLE_LL_LENGTH_RSP,
};

/**************************************************************************
 * Function     : llc_parser_ctrl_pdu
 *
 * Description  : This function is used to parser an LL Control pdu.
 *
 * Parameters   : pdata: the pointer of payload body
 *                len:   the length of payload body
 *                entry: the connection entry index
 *
 * Returns      : None
 *
 *************************************************************************/
void llc_parser_ctrl_pdu(UINT8 *payload, UINT8 len, UINT8 conn_entry)
{
    LL_CTRL_PDU_PAYLOAD *pCtrlPayload = (LL_CTRL_PDU_PAYLOAD *)payload;
    UINT8 opcode;
    LL_CTRL_PDU_PAYLOAD *pTxPkt;

    opcode = pCtrlPayload->OpCode;

    if (conn_entry > LL_MAX_CONNECTION_UNITS)
    {
        LL_LOG_TRACE(RED, LE_MSG_INVALID_CONN_ENTRY, 2, conn_entry);
        return;
    }

    if (opcode > LL_CTRL_PDU_MAX)
    {
        LL_LOG_TRACE(YELLOW, LE_MSG_INVALID_LLC_PDU, 1, opcode);

        /* generate LL_UNKNOWN_RES*/
        pTxPkt = llc_generate_ll_unknown_rsp(opcode);
        llc_append_pdu_to_tx_list(pTxPkt, conn_entry);
    }
    else
    {
        LL_LOG_TRACE(WHITE, DAPE_TEST_LOG274, 15,
                            opcode,
                            ll_manager.conn_unit.master,
                            conn_entry, len,
                            ll_manager.conn_unit.handle[conn_entry].encrypt_blk.pause_enc_state,
                            ll_manager.conn_unit.handle[conn_entry].encrypt_blk.start_enc_state,
                            ll_manager.conn_unit.handle[conn_entry].conn_counter,
                            payload[0], payload[1], payload[2], payload[3],
                            payload[4], payload[5], payload[6], payload[7]);

        llc_handle_ll_ctrl_pdu[opcode](payload, len, conn_entry);
    }
}

#ifdef _ADD_LLC_HANDLE_START_ENC_RES_ACK_RECD_
void llc_handle_sent_ll_start_enc_rsp_ack_recd_task_bg(void *pargument)
{
    LL_CONN_HANDLE_UNIT *pHandle = (LL_CONN_HANDLE_UNIT*)pargument;
    LL_ENCRYPT_BLK *pEnc_blk;
    LL_TASK_PARA_U task_param;
    OS_SIGNAL sig_send;

    pEnc_blk = &pHandle->encrypt_blk;

    if (!ll_manager.conn_unit.master)
    {
        /* this shall be the way 2 of three way handshake for slave */
        if (LLC_START_ENC_STATE_S2M_START_ENC_REQ == pEnc_blk->start_enc_state)
        {
            pEnc_blk->start_enc_state = LLC_START_ENC_STATE_END;

            if (LLC_PAUSE_ENC_STATE_END == pEnc_blk->pause_enc_state)
            {
                /* finish restart encryption procedure. we can stop and kill
                    llc procedure response timer */
                //OS_DELETE_TIMER(&pHandle->llc_timer);

                /* resume acl user data packets */
                pHandle->pause_acl_tx = FALSE;
                pHandle->discard_acl_rx = FALSE;

                pEnc_blk->pause_enc_state = LLC_PAUSE_ENC_STATE_IDLE;

                pHandle->llc.type = LLC_PROCEDURE_TYPE_NONE;

                /* send Encryption_Key_Refresh_Complete Event to Host */
                task_param.Dword = 0;
                task_param.lmp_s.sub_type = LL_TASK_HANDLE_ENCRYPTION_KEY_REFRESH;
                task_param.lmp_s.conn_entry = pHandle->unit_id;
                task_param.lmp_s.status = HCI_COMMAND_SUCCEEDED;
                sig_send.type = LL_GEN_HCI_EVENT_SIGNAL;
                sig_send.param = (OS_ADDRESS)task_param.Dword;
                OS_SEND_SIGNAL_TO_TASK (lmp_task_handle, sig_send);
            }
            else
            {
                /* finish start encryption procedure. we can stop and kill
                    llc procedure response timer */
                //OS_DELETE_TIMER(&pHandle->llc_timer);

                /* resume acl user data packets */
                pHandle->pause_acl_tx = FALSE;
                pHandle->discard_acl_rx = FALSE;

                pHandle->llc.type = LLC_PROCEDURE_TYPE_NONE;

                /* TODO: we shall send Encryption_Change_Event to Host */
                task_param.Dword = 0;
                task_param.lmp_s.sub_type = LL_TASK_HANDLE_ENCRYPTION_CHANGE_EVENT;
                task_param.lmp_s.reason = 0x01;
                task_param.lmp_s.conn_entry = pHandle->unit_id;
                task_param.lmp_s.status = HCI_COMMAND_SUCCEEDED;
                sig_send.type = LL_GEN_HCI_EVENT_SIGNAL;
                sig_send.param = (OS_ADDRESS)task_param.Dword;
                OS_SEND_SIGNAL_TO_TASK (lmp_task_handle, sig_send);
            }

#ifdef HS_USB_SEND_EVENT_REORDER_ISSUE_NEW
            /* To ensure no data to be sent before HCI_ENCRYPTION_CHANGE_EVENT
             * or HCI_ENCRYPTION_KEY_REFRESH_COMPLETE_EVENT is sent.
             */
            if ((g_fun_interface_info.b.hci_sel == PACKAGE_USB_USB)
                    || IS_EVENT_REORDER_FOR_ANY_INTERFACE)
            {
                aclq_rx_wait_queue_get(pHandle->conn_handle);
            }
#endif
        }
    }
}

void llc_handle_sent_ll_start_enc_rsp_ack_recd(LL_CONN_HANDLE_UNIT *pHandle)
{
    LL_ENCRYPT_BLK *pEnc_blk;
    OS_SIGNAL sig_send;

    pEnc_blk = &pHandle->encrypt_blk;

    if (!ll_manager.conn_unit.master)
    {
        /* this shall be the way 2 of three way handshake for slave */
        if (LLC_START_ENC_STATE_S2M_START_ENC_REQ == pEnc_blk->start_enc_state)
        {
            /* go to background to handle it !! */
            sig_send.type = ISR_EXT_BOTTOM_HALF_CALLBACK_PROCESS;
            sig_send.param = (OS_ADDRESS)llc_handle_sent_ll_start_enc_rsp_ack_recd_task_bg;
            sig_send.ext_param = (OS_ADDRESS)pHandle;
            OS_SEND_SIGNAL_TO_TASK (isr_extended_task_handle, sig_send);
        }
    }
}
#endif

void llc_handle_sent_llc_pdu_ack_recd(LL_CONN_HANDLE_UNIT *pHandle, UINT8 OpCode)
{
#ifdef _ROM_CODE_PATCHED_
    if (rcp_llc_sent_llc_pdu_ack_recd_func != NULL)
    {
        if (rcp_llc_sent_llc_pdu_ack_recd_func(pHandle, OpCode))
        {
            return;
        }
    }
#endif

    switch (OpCode)
    {
#ifdef _ADD_LLC_HANDLE_START_ENC_RES_ACK_RECD_
    case LL_START_ENC_RSP:
        llc_handle_sent_ll_start_enc_rsp_ack_recd(pHandle);
        break;
#endif

    default:
        break;
    }

    pHandle->llc.wait_ack = FALSE;
    pHandle->llc.type = LLC_PROCEDURE_TYPE_NONE;
#ifdef USE_FREERTOS
    OS_STOP_TIMER(pHandle->llc_timer, NULL);
#else
    OS_DELETE_TIMER(&pHandle->llc_timer);
#endif
}

void llc_check_sent_llc_pdu_request(LL_CONN_HANDLE_UNIT *pHandle, UINT8 OpCode)
{
    if (pHandle->llc.type != LLC_PROCEDURE_TYPE_NONE)
    {
        pHandle->llc.wait_sent_pdu_type = OpCode;
        pHandle->llc.wait_ack = TRUE;
    }
}

