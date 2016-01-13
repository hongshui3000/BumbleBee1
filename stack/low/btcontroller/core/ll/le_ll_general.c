enum { __FILE_NUM__= 206 };

#include "le_ll.h"
#include "le_ll_driver.h"
#include "mem.h"
#include "le_hci_4_0.h"
// dape added
#ifdef _CCH_SLOT_OFFSET_
#ifdef _DAPE_TEST_SLOT_OFFSET_FOR_LE
#include "lmp.h"
#endif
#endif
#ifdef _DAPE_TEST_SLOT_OFFSET_FOR_LE
extern UINT16 g_le_conn_interval_changed;
extern UCHAR  g_le_conn_interval_set;
#endif

#ifdef _ROM_CODE_PATCHED_
#ifdef _DAPE_8723_RCP_
PF_ROM_CODE_PATCH_FUNC rcp_ll_update_slot_decision_func = NULL;
#ifdef _BT4_1_SUPPORT_LL_CONN_PARAM_REQ
PF_ROM_CODE_PATCH_FUNC rcp_ll_decide_connection_parameters_func = NULL;
#endif
#endif
#endif

#ifdef _ENABLE_BLE_LL_FLOW_CONTROL_
UINT8 g_le_enter_flow_stop_flag = FALSE;
UINT8 g_le_flow_stop_threshold_pkt_cnt = (LL_POLL_HCI_MAX_RX_ACL_PKT_CNT >> 2);
UINT8 g_le_flow_go_threshold_pkt_cnt = ((LL_POLL_HCI_MAX_RX_ACL_PKT_CNT >> 2) * 3);
#endif

UINT8 ll_fw_aa_index[4] = {0,0,0,0};

/**************************************************************************
 * Function     : ll_fw_search_handle_unit_via_conn_handle
 *
 * Description  : This function is used to search handle unit via connection
 *                handle
 *
 * Parameters   : conn_handle: connection handle of one LE piconet link
 *
 * Returns      : the pointer of connection handle unit or NULL
 *
 *************************************************************************/
void ll_fw_init_conn_entry(UINT8 entry_idx)
{
    LL_CONN_HANDLE_UNIT *pHandle;

    if (entry_idx < LL_MAX_CONNECTION_UNITS)
    {
        /* init connection unit */
        pHandle = &ll_manager.conn_unit.handle[entry_idx];
        memset((UINT8*)pHandle, 0, sizeof(LL_CONN_HANDLE_UNIT));
        pHandle->unit_id = entry_idx;

        pHandle->conn_handle = LL_HCI_MIN_CONN_HANDLE + entry_idx;

#if defined(_SCHEDULE_BLE_MISC_PACKET_) && defined(_NEW_BZDMA_FROM_V8_)
#ifndef USE_NEW_LE_SCHEDULER
        pHandle->tx_resent_misc_pkt = NULL;
        pHandle->tx_sched_misc_pkt_list.DWord = LL_MISC_ACL_PKT_LIST_RESET_VALUE;
#endif
        pHandle->tx_pend_misc_pkt_list.DWord = LL_MISC_ACL_PKT_LIST_RESET_VALUE;
        pHandle->h2l_free_misc_pkt_list.DWord = LL_MISC_ACL_PKT_LIST_RESET_VALUE;
#endif
    }
}

#ifdef _ROM_CODE_PATCHED_
BOOLEAN (*rcp_ll_fw_fill_remote_conn_entry_param)(LL_CONN_HANDLE_UNIT *chu) = NULL;
#endif

/**************************************************************************
 * Function     : ll_fw_fill_remote_conn_entry_param
 *
 * Description  : This function is used to fill the parameters of remote
 *                connection entry
 *
 * Parameters   : role: LL_MASTER or LL_SLAVE
 *                conn_entry: connection entry of one LE piconet link
 *
 * Returns      : None
 *
 *************************************************************************/
void ll_fw_fill_remote_conn_entry_param(UINT8 role, UINT8 conn_entry)
{
    LL_CONN_HANDLE_UNIT *handle;
    LL_INITIATOR_UNIT *initiator = &ll_manager.initiator_unit;

    if (role == LL_MASTER)
    {
        if (conn_entry >= LL_MAX_CONNECTION_UNITS)
        {
            return;
        }

        ll_manager.conn_unit.bmActiveHandle |= BIT(conn_entry);

        handle = &ll_manager.conn_unit.handle[conn_entry];
        if (handle->connected == 1)
        {
            /* connection update ? */
        }

        /* init handle parameters */
        memset(((UINT8*)handle) + 4, 0, sizeof(LL_CONN_HANDLE_UNIT) - 4);

        /* move information from initiating state */
        handle->connected = 1;
        handle->conn_counter = 0;
        handle->support_enc = 1;

        handle->ce_interval = initiator->conn_interval;
        handle->slave_latency = initiator->conn_latency;
        handle->supervision_to = initiator->supervision_to;
        handle->ce_length = initiator->ce_len;
        handle->bmChMap[0] = initiator->conn_ch_map[0];
        handle->bmChMap[1] = initiator->conn_ch_map[1];
        handle->bmChMap[2] = initiator->conn_ch_map[2];
        handle->bmChMap[3] = initiator->conn_ch_map[3];
        handle->bmChMap[4] = initiator->conn_ch_map[4];

        /* fill IVm */
        handle->encrypt_blk.iv_m[0] = 0x24;
        handle->encrypt_blk.iv_m[1] = 0xAB;
        handle->encrypt_blk.iv_m[2] = 0xDC;
        handle->encrypt_blk.iv_m[3] = 0xBA;

        /* fill SKBm */
        handle->encrypt_blk.skd_m[0] = 0x13;
        handle->encrypt_blk.skd_m[1] = 0x02;
        handle->encrypt_blk.skd_m[2] = 0xF1;
        handle->encrypt_blk.skd_m[3] = 0xE0;
        handle->encrypt_blk.skd_m[4] = 0xDF;
        handle->encrypt_blk.skd_m[5] = 0xCE;
        handle->encrypt_blk.skd_m[6] = 0xBD;
        handle->encrypt_blk.skd_m[7] = 0xAC;
#ifndef _DAPE_GET_LE_SLV_ADDR_FROM_HW_WHEN_INIT
        handle->remote_info.u2addr[0] = initiator->u2rem_addr[0];
        handle->remote_info.u2addr[1] = initiator->u2rem_addr[1];
        handle->remote_info.u2addr[2] = initiator->u2rem_addr[2];
        handle->remote_info.addr_type = initiator->remote_addr_type;

#else

        BOOLEAN isPeerAddrSet = FALSE;
#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_
        if (ll_manager.address_resolution_enable && !ll_manager.initiator_unit.filter_policy)
        {
            handle->remote_info.u2addr[0] = initiator->u2rem_addr[0];
            handle->remote_info.u2addr[1] = initiator->u2rem_addr[1];
            handle->remote_info.u2addr[2] = initiator->u2rem_addr[2];
            handle->remote_info.addr_type = initiator->remote_addr_type;

            /* remote addr in slave mode is set in ll_adv_handle_connection_req() */
            isPeerAddrSet = TRUE;
        }
#endif
        if (!isPeerAddrSet)
        {
            handle->remote_info.u2addr[0] = RD_LE_REG(LE_REG_CONN_REQ_REMOTE_BD_ADDR_0);
            handle->remote_info.u2addr[1] = RD_LE_REG(LE_REG_CONN_REQ_REMOTE_BD_ADDR_1);
            handle->remote_info.u2addr[2] = RD_LE_REG(LE_REG_CONN_REQ_REMOTE_BD_ADDR_2);
#ifdef _GET_LE_SLV_ADDR_TYPE_FROM_HW         
            handle->remote_info.addr_type = RD_LE_REG(LE_REG_CONN_REQ_REMOTE_BD_ADDR_TYPE) & BIT0;
#else
#ifdef _DAPE_GET_LE_SLV_ADDR_TYPE_FROM_FW
            if (initiator->filter_policy!= 0)
            {
                UINT8 addr_type = 0xFF;
                UINT8 entry_id = ll_driver_search_dev_type_from_list(LL_WHITE_LIST_TYPE,
                &addr_type, handle->remote_info.addr);
                if (entry_id != LL_MAX_WHITE_LIST_SIZE)
                {
                    handle->remote_info.addr_type = addr_type;
                }
                else
                {
                    RT_BT_LOG(RED, DAPE_TEST_LOG563, 6,
                    handle->remote_info.addr[5], handle->remote_info.addr[4],
                    handle->remote_info.addr[3], handle->remote_info.addr[2],
                    handle->remote_info.addr[1], handle->remote_info.addr[0]);
                }
            }
            else
            {
                handle->remote_info.addr_type = initiator->remote_addr_type;
            }
#endif
#endif
        }

        /*RT_BT_LOG(WHITE, YL_DBG_HEX_4, 4,
        handle->remote_info.addr_type,
handle->remote_info.u2addr[0],
handle->remote_info.u2addr[1],
handle->remote_info.u2addr[2]);*/
#endif

        handle->cur_tx_power_index = LE_DEAULT_HW_TX_INDEX;

#ifndef USE_FREERTOS
        handle->sup_timer = NULL;
        handle->llc_timer = NULL;
#endif

        handle->conn_created = 1;
        handle->conn_update_le_slot_updated = 1;

#if defined(_SCHEDULE_BLE_MISC_PACKET_) && defined(_NEW_BZDMA_FROM_V8_)
#ifndef USE_NEW_LE_SCHEDULER
        handle->tx_resent_misc_pkt = NULL;
        handle->tx_sched_misc_pkt_list.DWord = LL_MISC_ACL_PKT_LIST_RESET_VALUE;
#endif
        handle->tx_pend_misc_pkt_list.DWord = LL_MISC_ACL_PKT_LIST_RESET_VALUE;
        handle->h2l_free_misc_pkt_list.DWord = LL_MISC_ACL_PKT_LIST_RESET_VALUE;
#endif
    }
    else
    {
        ll_manager.conn_unit.bmActiveHandle |= 0x01;
        ll_manager.conn_unit.master = 0;

        handle = &ll_manager.conn_unit.handle[0];

        /* init handle parameters */
        memset(((UINT8*)handle) + 4, 0, sizeof(LL_CONN_HANDLE_UNIT) - 4);

        /* move information from advertising state */
        handle->connected = 1;
        handle->conn_counter = 0;
        handle->support_enc = 1;

        /* fill IVs */
        handle->encrypt_blk.iv_s[0] = 0xBE;
        handle->encrypt_blk.iv_s[1] = 0xBA;
        handle->encrypt_blk.iv_s[2] = 0xAF;
        handle->encrypt_blk.iv_s[3] = 0xDE;

        /* fill SKDs */
        handle->encrypt_blk.skd_s[0] = 0x79;
        handle->encrypt_blk.skd_s[1] = 0x68;
        handle->encrypt_blk.skd_s[2] = 0x57;
        handle->encrypt_blk.skd_s[3] = 0x46;
        handle->encrypt_blk.skd_s[4] = 0x35;
        handle->encrypt_blk.skd_s[5] = 0x24;
        handle->encrypt_blk.skd_s[6] = 0x13;
        handle->encrypt_blk.skd_m[7] = 0x02;

        handle->cur_tx_power_index = LE_DEAULT_HW_TX_INDEX;

#ifndef USE_FREERTOS
        handle->sup_timer = NULL;
        handle->llc_timer = NULL;
#endif

        handle->conn_created = 1;

#if defined(_SCHEDULE_BLE_MISC_PACKET_) && defined(_NEW_BZDMA_FROM_V8_)
#ifndef USE_NEW_LE_SCHEDULER
        handle->tx_resent_misc_pkt = NULL;
        handle->tx_sched_misc_pkt_list.DWord = LL_MISC_ACL_PKT_LIST_RESET_VALUE;
#endif
        handle->tx_pend_misc_pkt_list.DWord = LL_MISC_ACL_PKT_LIST_RESET_VALUE;
        handle->h2l_free_misc_pkt_list.DWord = LL_MISC_ACL_PKT_LIST_RESET_VALUE;
#endif
    }

#ifdef _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_
    handle->data_len_updt.local_max_tx_size = ll_manager.conn_unit.init_max_tx_size;
    handle->data_len_updt.local_max_tx_time = ll_manager.conn_unit.init_max_tx_time;
    if (IS_BT42)
    {
        handle->data_len_updt.local_max_rx_size = LE_CONN_MAX_RX_SIZE_MAX;
        handle->data_len_updt.local_max_rx_time = LE_CONN_MAX_RX_TIME_MAX;
    }
    else
    {
        handle->data_len_updt.local_max_rx_size = LE_CONN_MAX_RX_SIZE_MIN;
        handle->data_len_updt.local_max_rx_time = LE_CONN_MAX_RX_TIME_MIN;
    }
    handle->data_len_updt.remote_max_tx_size = LE_CONN_MAX_TX_SIZE_MIN;
    handle->data_len_updt.remote_max_tx_time = LE_CONN_MAX_TX_TIME_MIN;
    handle->data_len_updt.remote_max_rx_size = LE_CONN_MAX_RX_SIZE_MIN;
    handle->data_len_updt.remote_max_rx_time = LE_CONN_MAX_RX_TIME_MIN;
#endif

#ifdef _ROM_CODE_PATCHED_
    if (rcp_ll_fw_fill_remote_conn_entry_param != NULL)
    {
        if (rcp_ll_fw_fill_remote_conn_entry_param(handle))
        {
            return;
        }
    }
#endif

#ifdef _SUPPORT_LBT_FUNC_FOR_RF_ADAPTIVITY_
    ll_driver_enabe_ce_lbt(conn_entry, LE_DEAULT_HW_TX_INDEX);
#endif

#ifdef _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_
    ll_fw_update_data_length(handle);
    if (IS_BT42)
    {
        ll_driver_enable_data_extension(conn_entry, TRUE);
        if (handle->data_len_updt.local_max_rx_size != LE_CONN_MAX_RX_SIZE_MIN
                || handle->data_len_updt.local_max_rx_time != LE_CONN_MAX_RX_TIME_MIN
                || handle->data_len_updt.local_max_tx_size != LE_CONN_MAX_TX_SIZE_MIN
                || handle->data_len_updt.local_max_tx_time != LE_CONN_MAX_TX_TIME_MIN)
        {
            if (role == LL_MASTER)
            {
                ll_fw_pend_data_length_update_procedure(handle);
            }
            else
            {
                /* Slave delays the data length update procedure until a data PDU
                 * is received and parsed.
                 */
                handle->llc.delay_data_len_updt = TRUE;
            }
        }
#ifdef _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_INFO_
        ll_fw_print_data_length(handle);
#endif
    }
#endif
}

/**************************************************************************
 * Function     : ll_fw_reset_remote_conn_entry_param
 *
 * Description  : This function is used to reset the parameters of remote
 *                connection entry
 *
 * Parameters   : conn_entry: connection entry of one LE piconet link
 *
 * Returns      : None
 *
 *************************************************************************/
void ll_fw_reset_remote_conn_entry_param(UINT8 conn_entry)
{
    LL_CONN_HANDLE_UNIT *handle;

    if (conn_entry < LL_MAX_CONNECTION_UNITS)
    {
        if (!(ll_manager.conn_unit.bmActiveHandle & (1 << conn_entry)))
        {
            return;
        }

        ll_manager.conn_unit.bmActiveHandle &= ~ (1 << conn_entry);

        if (ll_manager.conn_unit.conn_updt_entry == conn_entry)
        {
            ll_manager.conn_unit.conn_updt_entry = LL_MAX_CONNECTION_UNITS;
        }
        if (ll_manager.conn_unit.chm_updt_entry == conn_entry)
        {
            ll_manager.conn_unit.chm_updt_entry = LL_MAX_CONNECTION_UNITS;
        }

        handle = &ll_manager.conn_unit.handle[conn_entry];

#ifdef USE_FREERTOS
        OS_STOP_TIMER(handle->sup_timer, NULL);
        OS_STOP_TIMER(handle->llc_timer, NULL);
#else
        /* free supervision timer */
        if (handle->sup_timer != NULL)
        {
            OS_DELETE_TIMER(&handle->sup_timer);
        }

        /* free supervision timer */
        if (handle->llc_timer != NULL)
        {
            OS_DELETE_TIMER(&handle->llc_timer);
        }
#endif

        /* free pkt list */
        ll_fw_free_all_pkts_in_conn_entry(conn_entry);

        /* init handle parameters */
        memset(((UINT8*)handle) + 4, 0, sizeof(LL_CONN_HANDLE_UNIT) - 4);
        handle->connected = 0;
#ifndef USE_FREERTOS
        handle->sup_timer = NULL;
        handle->llc_timer = NULL;
#endif
#ifndef _SCHEDULE_BLE_MISC_PACKET_
        handle->tx_resent_data_pkt = NULL;
#endif

#if defined(_SCHEDULE_BLE_MISC_PACKET_) && defined(_NEW_BZDMA_FROM_V8_)
#ifndef USE_NEW_LE_SCHEDULER
        handle->tx_resent_misc_pkt = NULL;
        handle->tx_sched_misc_pkt_list.DWord = LL_MISC_ACL_PKT_LIST_RESET_VALUE;
#endif
        handle->tx_pend_misc_pkt_list.DWord = LL_MISC_ACL_PKT_LIST_RESET_VALUE;
        handle->h2l_free_misc_pkt_list.DWord = LL_MISC_ACL_PKT_LIST_RESET_VALUE;
#endif

        if (ll_manager.conn_unit.bmActiveHandle == 0)
        {
            ll_manager.conn_unit.enable = 0;
            ll_manager.conn_unit.master = 0;
            ll_manager.conn_unit.connection_cnts = 0;
            ll_manager.conn_unit.ce_interval_same = 0;
            ll_manager.conn_unit.ce_interval_min = 0xFFFF;

#ifdef _DAPE_TEST_SLOT_OFFSET_FOR_LE
            g_le_conn_interval_changed = 0;
            g_le_conn_interval_set = 0;
#endif
        }
    }
}


/**************************************************************************
 * Function     : ll_fw_search_handle_unit_via_conn_handle
 *
 * Description  : This function is used to search handle unit via connection
 *                handle
 *
 * Parameters   : conn_handle: connection handle of one LE piconet link
 *
 * Returns      : the pointer of connection handle unit or NULL
 *
 *************************************************************************/
LL_CONN_HANDLE_UNIT *ll_fw_search_handle_unit_via_conn_handle(UINT16 conn_handle)
{
    UINT8 entry_id;

    if ((conn_handle < LL_HCI_MIN_CONN_HANDLE) ||
        (conn_handle > LL_HCI_MAX_CONN_HANDLE))
    {
        return NULL;
    }

    entry_id = conn_handle - LL_HCI_MIN_CONN_HANDLE;

    if (ll_manager.conn_unit.bmActiveHandle & (1 << entry_id))
    {
        return &ll_manager.conn_unit.handle[entry_id];
    }

    return NULL;
}

/**************************************************************************
 * Function     : ll_fw_free_all_pkts_in_conn_entry
 *
 * Description  : This function is used to free all le acl pkts or llc pdus
 *                in the all lists from the LE connection entity
 *
 * Parameters   : conn_entry: connection entry id of one LE piconet link
 *
 * Returns      : None
 *
 *************************************************************************/
void ll_fw_free_all_pkts_in_conn_entry(UINT8 conn_entry)
{
    LL_CONN_HANDLE_UNIT *phandle;
    phandle = &ll_manager.conn_unit.handle[conn_entry];

    if (!phandle->connected)
    {
        return;
    }

#ifdef USE_NEW_LE_SCHEDULER
    le_sched_pkt_fifo_flush_entry(conn_entry);
#endif

    /* append pending LE llc pdu tx packet to free list */
    if (phandle->tx_pend_ctrl_pkt_list.pCtrlHead != NULL)
    {
        llc_append_pdu_list_to_free_list(&phandle->tx_pend_ctrl_pkt_list);
    }

#ifndef USE_NEW_LE_SCHEDULER
    /* append scheduled LE llc pdu tx packet to free list */
    if (phandle->tx_sched_ctrl_pkt_list.pCtrlHead != NULL)
    {
        llc_append_pdu_list_to_free_list(&phandle->tx_sched_ctrl_pkt_list);
    }
#endif

    /* free any llc pdu in the free list */
    llc_free_pdu_in_free_list();

#ifndef _SCHEDULE_BLE_MISC_PACKET_
    /* append resent LE ACL data tx packet to free list */
    if (phandle->tx_resent_data_pkt != NULL)
    {
        LE_ACL_PKT_LIST_MANAGE resent_list;
        resent_list.pDataHead = phandle->tx_resent_data_pkt;
        resent_list.pDataTail = phandle->tx_resent_data_pkt;
        resent_list.pktcnt = 1;

        ll_append_acl_tx_pkt_list_to_list(&resent_list,
                                          conn_entry,
                                          LL_LINKED_LIST_INSERT_POLICY_TAIL,
                                          LL_LINKED_LIST_TYPE_TX_FREE);

        phandle->tx_resent_data_pkt = NULL;
    }
#endif

    /* append pending LE ACL data tx packet to free list */
    if (phandle->tx_pend_data_pkt_list.pDataHead != NULL)
    {
        ll_append_acl_tx_pkt_list_to_list(&phandle->tx_pend_data_pkt_list,
                                          conn_entry,
                                          LL_LINKED_LIST_INSERT_POLICY_TAIL,
                                          LL_LINKED_LIST_TYPE_TX_FREE);

        memset(&phandle->tx_pend_data_pkt_list, 0, sizeof(LE_ACL_PKT_LIST_MANAGE));
    }

#ifndef _SCHEDULE_BLE_MISC_PACKET_
    /* append scheduled LE ACL data tx packet to free list */
    if (phandle->tx_sched_data_pkt_list.pDataHead != NULL)
    {
        ll_append_acl_tx_pkt_list_to_list(&phandle->tx_sched_data_pkt_list,
                                          conn_entry,
                                          LL_LINKED_LIST_INSERT_POLICY_TAIL,
                                          LL_LINKED_LIST_TYPE_TX_SCHED);

        memset(&phandle->tx_sched_data_pkt_list, 0, sizeof(LE_ACL_PKT_LIST_MANAGE));
    }
#endif

   /* free LE ACL data tx packet */
    if (phandle->h2l_free_pkt_list.pktcnt > 0)
    {
        ll_free_acl_tx_pkts_in_free_list(conn_entry);
    }

#if defined(_SCHEDULE_BLE_MISC_PACKET_) && defined(_NEW_BZDMA_FROM_V8_)
#ifndef USE_NEW_LE_SCHEDULER
    /* append resent LE ACL data misc tx packet to free list */
    if (phandle->tx_resent_misc_pkt != NULL)
    {
        LE_ACL_MISC_PKT_LIST_MANAGE resent_misc_list;
        resent_misc_list.MiscHeadId = phandle->tx_resent_misc_pkt->MyNodeId;
        resent_misc_list.MiscTailId = phandle->tx_resent_misc_pkt->MyNodeId;
        resent_misc_list.pktcnt = 1;

        ll_append_acl_tx_misc_pkt_list_to_list(&resent_misc_list,
                                          conn_entry,
                                          LL_LINKED_LIST_INSERT_POLICY_TAIL,
                                          LL_LINKED_LIST_TYPE_TX_FREE);

        phandle->tx_resent_misc_pkt = NULL;
    }
#endif

    /* append pending LE ACL data misc tx packet to free list */
    if (phandle->tx_pend_misc_pkt_list.pktcnt > 0)
    {
        ll_append_acl_tx_misc_pkt_list_to_list(&phandle->tx_pend_misc_pkt_list,
                                          conn_entry,
                                          LL_LINKED_LIST_INSERT_POLICY_TAIL,
                                          LL_LINKED_LIST_TYPE_TX_FREE);

        phandle->tx_pend_misc_pkt_list.DWord = LL_MISC_ACL_PKT_LIST_RESET_VALUE;
    }

#ifndef USE_NEW_LE_SCHEDULER
    /* append scheduled LE ACL data misc tx packet to free list */
    if (phandle->tx_sched_misc_pkt_list.pktcnt > 0)
    {
        ll_append_acl_tx_misc_pkt_list_to_list(&phandle->tx_sched_misc_pkt_list,
                                          conn_entry,
                                          LL_LINKED_LIST_INSERT_POLICY_TAIL,
                                          LL_LINKED_LIST_TYPE_TX_FREE);
        phandle->tx_sched_misc_pkt_list.DWord = LL_MISC_ACL_PKT_LIST_RESET_VALUE;
    }
#endif

    /* free LE ACL data misc tx packet */
    if (phandle->h2l_free_misc_pkt_list.pktcnt > 0)
    {
        ll_free_acl_tx_misc_pkts_in_free_list(conn_entry);
    }
#endif
}


/**************************************************************************
 * Function     : ll_fw_get_free_conn_entry
 *
 * Description  : This function is used to get one free connection entry
 *
 * Parameters   : None
 *
 * Returns      : valid connection entry id or LL_MAX_CONNECTION_UNITS
 *
 *************************************************************************/
UINT8 ll_fw_get_free_conn_entry(void)
{
    UINT8 entry_id;

    for (entry_id = 0; entry_id < LL_MAX_CONNECTION_UNITS; entry_id++)
    {
        if (!(ll_manager.conn_unit.bmActiveHandle & (1 << entry_id)))
        {
            ll_manager.conn_unit.bmActiveHandle |= 1 << entry_id;
            break;
        }
    }

    return entry_id;
}

/**************************************************************************
 * Function     : ll_fw_init_access_address_index
 *
 * Description  : This function is used to initiate an access address index.
 *                It is a seed and generated via HW random registers
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
void ll_fw_init_access_address_index(void)
{
    UINT32 temp;

    LL_DRIVER_GEN_32BIT_RANDOM(temp);
    ll_fw_aa_index[0] = temp;
    ll_fw_aa_index[1] = temp >> 8;
    ll_fw_aa_index[2] = temp >> 16;
    ll_fw_aa_index[3] = temp >> 24;
}

/**************************************************************************
 * Function     : ll_fw_generate_access_address
 *
 * Description  : This function is used to generate an access address.
 *                FW uses a pre-defined value plus some by an access address
 *                index to meet access address constraint.
 *
 * Parameters   : None
 *
 * Returns      : 32 bit Access Address
 *
 *************************************************************************/
UINT32 ll_fw_generate_access_address(void)
{
    UINT16 random;
    UINT32 access_address;
    UINT8 x;
    UINT8 y;
    UINT8 x_shift[4] = {26, 20, 29, 16};
    UINT8 y_shift[4] = {0, 1, 12, 4};
    UINT32 aa_base[4] = {LL_FW_ACCESS_ADDRESS_INIT0,
                         LL_FW_ACCESS_ADDRESS_INIT1,
                         LL_FW_ACCESS_ADDRESS_INIT2,
                         LL_FW_ACCESS_ADDRESS_INIT3};
    ll_fw_init_access_address_index();
    LL_DRIVER_GEN_16BIT_RANDOM(random);
    random &= 0x03;

    x = (ll_fw_aa_index[random] >> 4) & 0x07;
    y = ll_fw_aa_index[random] & 0x0F;
    ll_fw_aa_index[random] += 19;

    access_address = aa_base[random];
    access_address |= x << x_shift[random];
    access_address |= y << y_shift[random];
    if (y & 0x01)
    {
        access_address = ~access_address;
    }

    return access_address;
}

/**************************************************************************
 * Function     : ll_fw_compute_advertiser_h2h_min_duration
 *
 * Description  : This function is used to compute a minimum duration of
 *                head to head period of advertising pdu from an given
 *                advertising pdu length or scan response pdu length.
 *                (It also adds the RF setting time and FW adjusted time)
 *
 * Parameters   : None
 *
 * Returns      : durtion (unit: us)
 *
 *************************************************************************/
UINT16 ll_fw_compute_advertiser_h2h_min_duration(void)
{
    LL_ADVERTISER_UNIT *adv = &ll_manager.adv_unit;
    UINT16 duration_us;
    UINT16 max_trx_byet_cnt;
    UINT16 adv_pkt_size = LL_ADV_CH_PDU_SIZE_MIN + adv->adv_len;
    UINT16 scan_rsp_pkt_size = LL_SCAN_RSP_SIZE_MIN + adv->scan_rsp_len;

    switch (adv->pdu_type)
    {
    case LL_ADV_PDU_TYPE_ADV_IND:
        max_trx_byet_cnt = adv_pkt_size + LL_SCAN_REQ_SIZE + scan_rsp_pkt_size;
        max_trx_byet_cnt = MAX(max_trx_byet_cnt, adv_pkt_size + LL_CONNECTION_REQ_SIZE);
        duration_us = (max_trx_byet_cnt << 3) + (2 * LL_T_IFS);
        break;

    case LL_ADV_PDU_TYPE_ADV_DISCOVER_IND:
        max_trx_byet_cnt = adv_pkt_size + LL_SCAN_REQ_SIZE + scan_rsp_pkt_size;
        duration_us = (max_trx_byet_cnt << 3) + (2 * LL_T_IFS);
        break;

    case LL_ADV_PDU_TYPE_ADV_DIRECT_IND:
        max_trx_byet_cnt = LL_ADV_DIRECT_SIZE + LL_CONNECTION_REQ_SIZE;
        duration_us = (max_trx_byet_cnt << 3) + LL_T_IFS;
        break;

    case LL_ADV_PDU_TYPE_ADV_NONCONN_IND:
        max_trx_byet_cnt = adv_pkt_size;
        duration_us = max_trx_byet_cnt << 3;
        break;

    default:
        duration_us = 0;
        LL_LOG_TRACE(RED, LE_MSG_ERR_ADV_PDU_TYPE, 1, adv->pdu_type);
        return duration_us;
    }

    duration_us += LL_RF_TX_SETTLING_TIME + LL_ADV_H2H_FW_ADJ_OFFSET;

    return duration_us;
}


/**************************************************************************
 * Function     : ll_fw_check_multiple_states_valid
 *
 * Description  : This function is used to check current single or multiple
 *                states. To check the combination is valid or not then return
 *                the result
 *
 * Parameters   : unconn_state: current unconnected state
 *                type: current type (advertising pdu type or scan type)
 *
 * Returns      : result: TRUE or FALSE
 *
 *************************************************************************/
UINT8 ll_fw_check_multiple_states_valid(UINT8 unconn_state, UINT8 type)
{
    UINT8 result = FALSE;

    switch (unconn_state)
    {
    case LL_STATE_ADVERTISING:
        if (ll_manager.adv_unit.enable)
        {
            /* the advertising is enabled. Command Disallowed error
               code shall be used */
            break;
        }

        switch (type)
        {
        case LL_HCI_ADV_TYPE_ADV_NONCONN_IND:
            if (!(LL_MULTI_STATES_SUPPORTED_BITMAP_LOW_WORD & BIT0))
            {
                /* check non-connectable advertising state supported ? */
                break;
            }

            if (ll_manager.scan_unit.enable)
            {
                if (ll_manager.scan_unit.active_scan)
                {
                    if (!(LL_MULTI_STATES_SUPPORTED_BITMAP_LOW_WORD & BIT12))
                    {
                        /* check non-connectable advertising state and active
                           scanning state combination supported ? */
                        break;
                    }
                }
                else
                {
                    if (!(LL_MULTI_STATES_SUPPORTED_BITMAP_LOW_WORD & BIT8))
                    {
                        /* check non-connectable advertising state and passive
                           scanning state combination supported ? */
                        break;
                    }
                }
            }

            if (ll_manager.initiator_unit.enable)
            {
                if (!(LL_MULTI_STATES_SUPPORTED_BITMAP_LOW_WORD & BIT16))
                {
                    /* check non-connectable advertising state and initiating
                       state combination supported ? */
                    break;
                }
            }

            if (ll_manager.conn_unit.enable ||
                (ll_manager.conn_unit.bmActiveHandle > 0))
            {
                if (ll_manager.conn_unit.master)
                {
                    if (!(LL_MULTI_STATES_SUPPORTED_BITMAP_LOW_WORD & BIT18))
                    {
                        /* check non-connectable advertising state and master
                           role combination supported ? */
                        break;
                    }
                }
                else
                {
                    if (!(LL_MULTI_STATES_SUPPORTED_BITMAP_LOW_WORD & BIT20))
                    {
                        /* check non-connectable advertising state and slave
                           role combination supported ? */
                        break;
                    }
                }
            }
            result = TRUE;
            break;

        case LL_HCI_ADV_TYPE_ADV_DISCOVER_IND:
            if (!(LL_MULTI_STATES_SUPPORTED_BITMAP_LOW_WORD & BIT1))
            {
                /* check discoverable advertising state supported ? */
                break;
            }

            if (ll_manager.scan_unit.enable)
            {
                if (ll_manager.scan_unit.active_scan)
                {
                    if (!(LL_MULTI_STATES_SUPPORTED_BITMAP_LOW_WORD & BIT13))
                    {
                        /* check discoverable advertising state and active
                           scanning state combination supported ? */
                        break;
                    }
                }
                else
                {
                    if (!(LL_MULTI_STATES_SUPPORTED_BITMAP_LOW_WORD & BIT9))
                    {
                        /* check discoverable advertising state and passive
                           scanning state combination supported ? */
                        break;
                    }
                }
            }

            if (ll_manager.initiator_unit.enable)
            {
                if (!(LL_MULTI_STATES_SUPPORTED_BITMAP_LOW_WORD & BIT17))
                {
                    /* check discoverable advertising state and initiating
                       state combination supported ? */
                    break;
                }
            }

            if (ll_manager.conn_unit.enable ||
                (ll_manager.conn_unit.bmActiveHandle > 0))
            {
                if (ll_manager.conn_unit.master)
                {
                    if (!(LL_MULTI_STATES_SUPPORTED_BITMAP_LOW_WORD & BIT19))
                    {
                        /* check discoverable advertising state and master
                           role combination supported ? */
                        break;
                    }
                }
                else
                {
                    if (!(LL_MULTI_STATES_SUPPORTED_BITMAP_LOW_WORD & BIT21))
                    {
                        /* check discoverable advertising state and slave
                           role combination supported ? */
                        break;
                    }
                }
            }
            result = TRUE;
            break;

        case LL_HCI_ADV_TYPE_ADV_IND:
            if (!(LL_MULTI_STATES_SUPPORTED_BITMAP_LOW_WORD & BIT2) ||
                !(LL_MULTI_STATES_SUPPORTED_BITMAP_LOW_WORD & BIT7))
            {
                /* check connectable advertising state and connection state in
                   the slave role supported ? */
                break;
            }

            if (ll_manager.scan_unit.enable)
            {
                if (ll_manager.scan_unit.active_scan)
                {
                    if (!(LL_MULTI_STATES_SUPPORTED_BITMAP_LOW_WORD & BIT14))
                    {
                        /* check connectable advertising state and active
                           scanning state combination supported ? */
                        break;
                    }
                }
                else
                {
                    if (!(LL_MULTI_STATES_SUPPORTED_BITMAP_LOW_WORD & BIT10))
                    {
                        /* check connectable advertising state and passive
                           scanning state combination supported ? */
                        break;
                    }
                }
            }

            if (ll_manager.initiator_unit.enable ||
                ll_manager.conn_unit.enable ||
                (ll_manager.conn_unit.bmActiveHandle > 0))
            {
                /* such combination is not allowed */
                break;
            }

            result = TRUE;
            break;

        case LL_HCI_ADV_TYPE_ADV_DIRECT_IND:
            if (!(LL_MULTI_STATES_SUPPORTED_BITMAP_LOW_WORD & BIT3) ||
                !(LL_MULTI_STATES_SUPPORTED_BITMAP_LOW_WORD & BIT7))
            {
                /* check direct advertising state and the
                   connection state in the slave role supported ? */
                break;
            }

            if (ll_manager.scan_unit.enable)
            {
                if (ll_manager.scan_unit.active_scan)
                {
                    if (!(LL_MULTI_STATES_SUPPORTED_BITMAP_LOW_WORD & BIT15))
                    {
                        /* check directed advertising state and active
                           scanning state combination supported ? */
                        break;
                    }
                }
                else
                {
                    if (!(LL_MULTI_STATES_SUPPORTED_BITMAP_LOW_WORD & BIT11))
                    {
                        /* check directed advertising state and passive
                           scanning state combination supported ? */
                        break;
                    }
                }
            }

            if (ll_manager.initiator_unit.enable ||
                ll_manager.conn_unit.enable ||
                (ll_manager.conn_unit.bmActiveHandle > 0))
            {
                /* such combination is not allowed */
                break;
            }
            result = TRUE;
            break;

#ifdef _DAPE_LOW_DUTY_ADV
        case LL_HCI_ADV_TYPE_ADV_LOW_DUTY_DIRECT_IND:
            if (!(LL_MULTI_STATES_SUPPORTED_BITMAP_LOW_WORD & BIT29) ||
                !(LL_MULTI_STATES_SUPPORTED_BITMAP_LOW_WORD & BIT7))
            {
                /* check low duty direct advertising state and the
                   connection state in the slave role supported ? */
                break;
            }

            if (ll_manager.scan_unit.enable)
            {
                if (ll_manager.scan_unit.active_scan)
                {
                    if (!(LL_MULTI_STATES_SUPPORTED_BITMAP_LOW_WORD & BIT31))
                    {
                        /* check directed advertising state and active
                           scanning state combination supported ? */
                        break;
                    }
                }
                else
                {
                    if (!(LL_MULTI_STATES_SUPPORTED_BITMAP_LOW_WORD & BIT30))
                    {
                        /* check directed advertising state and passive
                           scanning state combination supported ? */
                        break;
                    }
                }
            }

            if (ll_manager.initiator_unit.enable ||
                ll_manager.conn_unit.enable ||
                (ll_manager.conn_unit.bmActiveHandle > 0))
            {
                /* such combination is not allowed */
                break;
            }
            result = TRUE;
            break;
#endif

        default:
            break;
        }
        break;

    case LL_STATE_SCANNING:
        if (ll_manager.scan_unit.enable)
        {
            /* the advertising is enabled. Command Disallowed error
               code shall be used */
            break;
        }

        if (type)
        {
            /* active scan */
            if (!(LL_MULTI_STATES_SUPPORTED_BITMAP_LOW_WORD & BIT5))
            {
                /* check active scanning state supported ? */
                break;
            }

            if (ll_manager.adv_unit.enable)
            {
                if (ll_manager.adv_unit.pdu_type == LL_HCI_ADV_TYPE_ADV_NONCONN_IND)
                {
                    if (!(LL_MULTI_STATES_SUPPORTED_BITMAP_LOW_WORD & BIT12))
                    {
                        /* check active scanning state and non-connectable
                           advertising state combination supported ? */
                        break;
                    }
                }
                else if (ll_manager.adv_unit.pdu_type == LL_HCI_ADV_TYPE_ADV_DISCOVER_IND)
                {
                    if (!(LL_MULTI_STATES_SUPPORTED_BITMAP_LOW_WORD & BIT13))
                    {
                        /* check active scanning state and discoverable
                           advertising state combination supported ? */
                        break;
                    }
                }
                else if (ll_manager.adv_unit.pdu_type == LL_HCI_ADV_TYPE_ADV_IND)
                {
                    if (!(LL_MULTI_STATES_SUPPORTED_BITMAP_LOW_WORD & BIT14))
                    {
                        /* check active scanning state and connectable
                           advertising state combination supported ? */
                        break;
                    }
                }
                else if (ll_manager.adv_unit.hci_pdu_type == LL_HCI_ADV_TYPE_ADV_DIRECT_IND)
                {
                    if (!(LL_MULTI_STATES_SUPPORTED_BITMAP_LOW_WORD & BIT15))
                    {
                        /* check active scanning state and directed
                           advertising state combination supported ? */
                        break;
                    }
                }
#ifdef _DAPE_LOW_DUTY_ADV
                else if (ll_manager.adv_unit.hci_pdu_type == LL_HCI_ADV_TYPE_ADV_LOW_DUTY_DIRECT_IND)
                {
                    if (!(LL_MULTI_STATES_SUPPORTED_BITMAP_LOW_WORD & BIT31))
                    {
                        /* check active scanning state and directed
                           advertising state combination supported ? */
                        break;
                    }
                }
#endif
            }


            if (ll_manager.initiator_unit.enable)
            {
                if (!(LL_MULTI_STATES_SUPPORTED_BITMAP_LOW_WORD & BIT23))
                {
                    /* check active scanning state and initiating
                       state combination supported ? */
                    break;
                }
            }

            if (ll_manager.conn_unit.enable ||
                (ll_manager.conn_unit.bmActiveHandle > 0))
            {
                if (ll_manager.conn_unit.master)
                {
                    if (!(LL_MULTI_STATES_SUPPORTED_BITMAP_LOW_WORD & BIT25))
                    {
                        /* check active scanning state and master role
                           combination supported ? */
                        break;
                    }
                }
                else
                {
                    if (!(LL_MULTI_STATES_SUPPORTED_BITMAP_LOW_WORD & BIT27))
                    {
                        /* check active scanning state and slave role
                           combination supported ? */
                        break;
                    }
                }
            }
        }
        else
        {
            /* passive scan */
            if (!(LL_MULTI_STATES_SUPPORTED_BITMAP_LOW_WORD & BIT4))
            {
                /* check passive scanning state supported ? */
                break;
            }

            if (ll_manager.adv_unit.enable)
            {
                if (ll_manager.adv_unit.pdu_type == LL_HCI_ADV_TYPE_ADV_NONCONN_IND)
                {
                    if (!(LL_MULTI_STATES_SUPPORTED_BITMAP_LOW_WORD & BIT8))
                    {
                        /* check passive scanning state and non-connectable
                           advertising state combination supported ? */
                        break;
                    }
                }
                else if (ll_manager.adv_unit.pdu_type == LL_HCI_ADV_TYPE_ADV_DISCOVER_IND)
                {
                    if (!(LL_MULTI_STATES_SUPPORTED_BITMAP_LOW_WORD & BIT9))
                    {
                        /* check passive scanning state and discoverable
                           advertising state combination supported ? */
                        break;
                    }
                }
                else if (ll_manager.adv_unit.pdu_type == LL_HCI_ADV_TYPE_ADV_IND)
                {
                    if (!(LL_MULTI_STATES_SUPPORTED_BITMAP_LOW_WORD & BIT10))
                    {
                        /* check passive scanning state and connectable
                           advertising state combination supported ? */
                        break;
                    }
                }
                else if (ll_manager.adv_unit.hci_pdu_type == LL_HCI_ADV_TYPE_ADV_DIRECT_IND)
                {
                    if (!(LL_MULTI_STATES_SUPPORTED_BITMAP_LOW_WORD & BIT11))
                    {
                        /* check passive scanning state and directed
                           advertising state combination supported ? */
                        break;
                    }
                }
#ifdef _DAPE_LOW_DUTY_ADV
                else if (ll_manager.adv_unit.hci_pdu_type == LL_HCI_ADV_TYPE_ADV_LOW_DUTY_DIRECT_IND)
                {
                    if (!(LL_MULTI_STATES_SUPPORTED_BITMAP_LOW_WORD & BIT30))
                    {
                        /* check passive scanning state and directed
                           advertising state combination supported ? */
                        break;
                    }
                }
#endif
            }

            if (ll_manager.initiator_unit.enable)
            {
                if (!(LL_MULTI_STATES_SUPPORTED_BITMAP_LOW_WORD & BIT22))
                {
                    /* check passive scanning state and initiating
                       state combination supported ? */
                    break;
                }
            }

            if (ll_manager.conn_unit.enable ||
                (ll_manager.conn_unit.bmActiveHandle > 0))
            {
                if (ll_manager.conn_unit.master)
                {
                    if (!(LL_MULTI_STATES_SUPPORTED_BITMAP_LOW_WORD & BIT24))
                    {
                        /* check passive scanning state and master role
                           combination supported ? */
                        break;
                    }
                }
                else
                {
                    if (!(LL_MULTI_STATES_SUPPORTED_BITMAP_LOW_WORD & BIT26))
                    {
                        /* check passive scanning state and slave role
                           combination supported ? */
                        break;
                    }
                }
            }
        }
        result = TRUE;
        break;

    case LL_STATE_INITIATING:
        if (ll_manager.initiator_unit.enable)
        {
            /* the initiator is enabled. Command Disallowed error
               code shall be used */
            break;
        }

        if (!(LL_MULTI_STATES_SUPPORTED_BITMAP_LOW_WORD & BIT6))
        {
            /* check the initiating state supported. connection state in the
               master role supported is also supported */
            break;
        }

        if (ll_manager.adv_unit.enable)
        {
            if (ll_manager.adv_unit.pdu_type == LL_HCI_ADV_TYPE_ADV_NONCONN_IND)
            {
                if (!(LL_MULTI_STATES_SUPPORTED_BITMAP_LOW_WORD & BIT16))
                {
                   /* check the initiating state and non-connectable
                      advertising state combination supported */
                    break;
                }
            }
            else if (ll_manager.adv_unit.pdu_type == LL_HCI_ADV_TYPE_ADV_DISCOVER_IND)
            {
                if (!(LL_MULTI_STATES_SUPPORTED_BITMAP_LOW_WORD & BIT17))
                {
                   /* check the initiating state and discoverable
                      advertising state combination supported */
                    break;
                }
            }
        }

        if (ll_manager.scan_unit.enable)
        {
            if (ll_manager.scan_unit.active_scan)
            {
                if (!(LL_MULTI_STATES_SUPPORTED_BITMAP_LOW_WORD & BIT27))
                {
                   /* check the initiating state and active scanning
                      state combination supported */
                    break;
                }
            }
            else
            {
                if (!(LL_MULTI_STATES_SUPPORTED_BITMAP_LOW_WORD & BIT26))
                {
                   /* check the initiating state and passive scanning
                      state combination supported */
                    break;
                }
            }
        }

        if (ll_manager.conn_unit.enable ||
            (ll_manager.conn_unit.bmActiveHandle > 0))
        {
            if (ll_manager.conn_unit.master)
            {
                if (!(LL_MULTI_STATES_SUPPORTED_BITMAP_LOW_WORD & BIT28))
                {
                    /* check initiating state and master role combination
                       supported. master role and master role combination
                       is also supported */
                    break;
                }
            }
            else
            {
                /* disallow combination */
                break;
            }
        }

        result = TRUE;
        break;

    default:
        break;
    }

    return result;
}

/**************************************************************************
 * Function     : ll_fw_get_gcd
 *
 * Description  : This function is used to get gcd(a, b)
 *
 * Parameters   : a, b: the value of which we want to get their gcd
 *
 * Returns      : result: gcd(a,b)
 *
 *************************************************************************/
UINT32 ll_fw_get_gcd(UINT32 a, UINT32 b)
{
    UINT32 temp;

    while (b)
    {
        temp = b;
        b = a % b;
        a = temp;
    }

    return a;
}

/**************************************************************************
 * Function     : ll_fw_check_all_slave_interval
 *
 * Description  : This function is used for LE master to update all slave's intervals.
 *
 * Parameters   : NONE
 *
 * Returns      : NONE
 *
 *************************************************************************/
void ll_fw_check_all_slave_interval(void)
{
    UINT8  i;
	UINT16 bm_conn_entry = RD_LE_REG(LE_REG_STATUS_CAM_VALID);
    LL_CONN_HANDLE_UNIT *handle;
    //UINT32 ce_interval = ll_manager.initiator_unit.conn_interval;
    UINT32 ce_interval = 0xFFFFFFFF;
    ll_manager.conn_unit.ce_interval_same = TRUE;
    ll_manager.conn_unit.ce_interval_min = 0xFFFF;
    //ll_manager.conn_unit.ce_interval_second_min = 0xFFFFFFFF;
    if (ll_manager.initiator_unit.enable)
    {
        ce_interval = ll_manager.initiator_unit.conn_interval;
        if (ce_interval < ll_manager.conn_unit.ce_interval_min)
        {
        ll_manager.conn_unit.ce_interval_min = ce_interval;
	}
    }
    for (i = 0; i < LL_MAX_CONNECTION_UNITS; i++)
    {
        if (bm_conn_entry & (1 << i))
        {
            handle = &ll_manager.conn_unit.handle[i];

            /* Check if ce_interval(now is same as previous) is the same as current entry. */
            if ((ll_manager.conn_unit.ce_interval_same == TRUE) &&
                (ce_interval != 0xFFFFFFFF))
            {
                if (ce_interval == handle->ce_interval)
                {
                    ll_manager.conn_unit.ce_interval_same = TRUE;
                }
                else
                {
                    ll_manager.conn_unit.ce_interval_same = FALSE;
                }
            }
            ce_interval = handle->ce_interval;

            if (ce_interval < ll_manager.conn_unit.ce_interval_min)
            {
                /*if (ll_manager.conn_unit.ce_interval_min == 0xFFFFFFFF)
                {
                    ll_manager.conn_unit.ce_interval_second_min = ce_interval;
                }
                else
                {
                    ll_manager.conn_unit.ce_interval_second_min = ll_manager.conn_unit.ce_interval_min;
                }*/
                ll_manager.conn_unit.ce_interval_min = ce_interval;
            }
        }
    }

    if ((ll_manager.conn_unit.connection_cnts > 1) ||
        ((ll_manager.conn_unit.connection_cnts != 0) && (ll_manager.initiator_unit.enable)))
    {
        RT_BT_LOG(BLUE, DAPE_TEST_LOG427, 6, BB_read_native_clock(),
            ll_manager.initiator_unit.enable,
            ce_interval,
        ll_manager.initiator_unit.conn_interval,ll_manager.conn_unit.ce_interval_min,
    	ll_manager.conn_unit.ce_interval_same
	//ll_manager.conn_unit.ce_interval_second_min);
    	);
    }
}

/**************************************************************************
 * Function     : ll_fw_get_adv_hit_counter_n
 *
 * Description  : This function is used for LE initiator to get the counter when
 *                it receives the connectable adv pdu.
 *
 * Parameters   : None.
 *
 * Returns      : adv_hit_counter_n
 *
 *************************************************************************/
UINT16 ll_fw_get_adv_hit_counter_n(void)
{
    UINT16 reg_clk;
    UINT16 adv_hit_counter_n;
    reg_clk = RD_LE_REG(LE_REG_SLAVE_CLOCK_HIGH);
    adv_hit_counter_n = (reg_clk >> 2) & (0x3FF);
    if (adv_hit_counter_n > 624)
    {
        adv_hit_counter_n -= 624;
    }
    return adv_hit_counter_n;
}

/**************************************************************************
 * Function     : ll_fw_conn_update
 *
 * Description  : This function is used for LE master to do connection update.
 *
 * Parameters   :  UINT8 conn_entry: connection entry which is going to do connection update.
 *                 UINT8 tx_win_size: Tx Window Size of connection update.
 *                 UINT16 tx_win_offset: Tx Window Size Offset of connection update.
 *                 UINT16 conn_interval: New Connection Interval.
 *                 UINT16 conn_latency: New slave latency,
 *                 UINT16 supervision_timeout: New Supervision timeout
 *
 * Returns      : Update Result. 0: Success. Others: generate connection update fail.
 *
 *************************************************************************/
UINT8 ll_fw_conn_update(UINT8 conn_entry, UINT8 tx_win_size, UINT16 tx_win_offset,
	UINT16 conn_interval, UINT16 conn_latency, UINT16 supervision_timeout)
{
    UINT8 errcode = INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    LL_CONN_HANDLE_UNIT *pHandle;

	pHandle = &ll_manager.conn_unit.handle[conn_entry];

    do
    {
        if (!ll_manager.conn_unit.enable || !ll_manager.conn_unit.master)
        {
            /* not setup a connection or we are not master role */
            errcode = COMMAND_DISALLOWED_ERROR;
            break;
        }

        if (conn_latency > LL_CONN_LATENCY_MAX)
        {
            /* In fact, the valid range of this field is between 0x0000 and
               0x03E8 in BT4.0 HCI spec. But the LL spec defines slave latency
               can not be larger than 500.  Therefore, FW sets maximum value is
               500 here - austin */
            break;
        }

        if ((supervision_timeout < LL_SUPERVISION_TO_MIN) ||
            (supervision_timeout > LL_SUPERVISION_TO_MAX) ||
            (supervision_timeout <= (conn_interval >> 3)))
        {
            /* supervision_timeout (unit:10ms) in milliseconds shall be larger
               than the conn_interval_max (unit:1.25ms) in milliseconds */
            break;
        }

        if (conn_latency != 0)
        {
            UINT16 conn_intv_val;
            conn_intv_val = (supervision_timeout << 3) / (conn_latency + 1);
            if (conn_interval > conn_intv_val)
            {
                /* slave latency shall be an integer beteen 0 and
                   (supervision timeout / conn_interval) - 1 */
                break;
            }
        }

        if (pHandle != NULL)
        {
            if (pHandle->llc.pend_conn_updt)
            {
                /* one connection req update is processing. we should disallow
                   this commaand (or pend this command) ? - austin */
                errcode = COMMAND_DISALLOWED_ERROR;
                break;
            }

            /* update connection parameters*/
            pHandle->conn_updt_blk.ce_interval_min = conn_interval;
            pHandle->conn_updt_blk.ce_interval_max = conn_interval;
            pHandle->conn_updt_blk.ce_interval = conn_interval;
            pHandle->conn_updt_blk.slave_latency = conn_latency;
            pHandle->conn_updt_blk.supervision_to = supervision_timeout;
            pHandle->conn_updt_blk.ce_length_min = (conn_interval-1)<< 1;
            pHandle->conn_updt_blk.ce_length_max = (conn_interval-1)<< 1;
            pHandle->conn_updt_blk.ce_length = pHandle->conn_updt_blk.ce_length_min;

            /* compute transmit window offset (0 to connInterval)*/
            pHandle->conn_updt_blk.tx_win_offset = tx_win_offset;

            /* compute transmit window size (10 ms ~ (connInterval - 1.25ms) */
            pHandle->conn_updt_blk.tx_win_size = tx_win_size;

            pHandle->conn_updt_blk.tx_win_size_ofst = 0;

            errcode = HCI_COMMAND_SUCCEEDED;


            /* TODO: prepare connection update procedure */

            if (pHandle->llc.type != LLC_PROCEDURE_TYPE_NONE)
            {
                /* need to wait after previous LLC procedure has completed */
                pHandle->llc.pend_conn_updt = TRUE;
            }
            else
            {
                llc_schedule_connection_update_procedure(pHandle);
            }

            return errcode;
        }
        else
        {
            /* this code shall be re-name "UNKNOWN_CONNECTION_IDENTIFIER" */
            errcode = NO_CONNECTION_ERROR;
        }
    }
    while (0);

	return errcode;
}
/**************************************************************************
 * Function     : ll_fw_get_slot_offset
 *
 * Description  : This function is used for le master to calculate the slot
 *                offset
 *
 * Parameters   : conn_entry: the connection entry
 *                role: master(1) or slave(0)
 *
 * Returns      : result: NONE
 *
 *************************************************************************/
void ll_fw_get_slot_offset(UINT8 conn_entry, UINT8 role)
{
    UINT32 rd_value = 0;
    UINT8  dw_addr;
    UINT16 conn_interval;
    UINT16 conn_interval_slot;
    UINT32 real_anchor_point = 0;
	UINT32 full_real_anchor_point = 0;
    UINT32 anchor_point_slot = 0;

    LL_CONN_HANDLE_UNIT *phandle;

    phandle = &ll_manager.conn_unit.handle[conn_entry];
#ifdef _DAPE_MOVE_LE_SLAVE_PARAMETER_TO_CAM
    UINT8 cam_entry = 0;
    if (role)
    {
        cam_entry = conn_entry;
    }
    dw_addr = LE_CAM_ENTRY_BASE(cam_entry) + LE_CAM_ADDR_1;
    conn_interval = ll_driver_read_cam(dw_addr) >> 16;
    dw_addr = LE_CAM_ENTRY_BASE(cam_entry) + LE_CAM_ADDR_0;
    rd_value = ll_driver_read_cam(dw_addr);
    real_anchor_point = (rd_value>>4) & (UINT32)(0x1FFFF);

#else
    // Master //
    if (role)
    {
        dw_addr = LE_CAM_ENTRY_BASE(conn_entry) + LE_CAM_ADDR_1;
        conn_interval = ll_driver_read_cam(dw_addr) >> 16;
        dw_addr = LE_CAM_ENTRY_BASE(conn_entry) + LE_CAM_ADDR_0;
        rd_value = ll_driver_read_cam(dw_addr);
        real_anchor_point = (rd_value>>4) & (UINT32)(0x1FFFF);
    }
    else
    {
        conn_interval = RD_LE_REG(LE_REG_SLAVE_CE_INTERVAL);
    }

#endif
    conn_interval_slot = conn_interval<<1;
    anchor_point_slot = phandle->anchor_point_by_intr >> 1;

    if (role)
    {
        full_real_anchor_point = (real_anchor_point<<1) | (anchor_point_slot & (0xFFFC0000));
        phandle->slot_offset =  full_real_anchor_point % conn_interval_slot;
        phandle->slot_offset += LE_LEGACY_COMPENSATE_SLOT;
        if (conn_entry ==ll_manager.conn_unit.first_connection_entry)
        {
            //ll_manager.conn_unit.first_slot_offset_for_le = (real_anchor_point<<1) % conn_interval_slot;
            ll_manager.conn_unit.first_slot_offset_for_le = phandle->slot_offset;
            //ll_manager.conn_unit.current_slot_offset_for_le = phandle->slot_offset;
        }
    }
    else
    {
        phandle->slot_offset = anchor_point_slot % conn_interval_slot;
    }

///// dape test only
RT_BT_LOG(WHITE, DAPE_TEST_LOG431, 7, conn_entry, role, full_real_anchor_point, real_anchor_point,
              anchor_point_slot, phandle->slot_offset, conn_interval_slot);
}
#ifdef _CCH_SLOT_OFFSET_
#ifdef _DAPE_TEST_SLOT_OFFSET_FOR_LE
/**************************************************************************
 * Function     : ll_fw_compute_slot_offset_and_occupy
 *
 * Description  : This function is used for le master to compute the slot
 *                offset and occupy slots for le connection
 *
 * Parameters   : conn_entry: the connection entry
 *                role: master(1) or slave(0)
 *
 * Returns      : result: NONE
 *
 *************************************************************************/
void ll_fw_compute_slot_offset_and_occupy(UINT8 conn_entry, UINT8 role)
{
    UINT32 rd_value = 0;
    UINT8  dw_addr;
    UINT16 conn_interval;
    UINT16 conn_interval_slot;
    UINT32 anchor_point;
    UINT32 real_anchor_point = 0;
    UINT32 ahchor_point_slot;
    UINT16 le_slot_offset;
    LL_CONN_HANDLE_UNIT *phandle;

    phandle = &ll_manager.conn_unit.handle[conn_entry];

#ifdef _DAPE_MOVE_LE_SLAVE_PARAMETER_TO_CAM
    UINT8 cam_entry = 0;
    if (role)
    {
        cam_entry = conn_entry;
    }
    dw_addr = LE_CAM_ENTRY_BASE(cam_entry) + LE_CAM_ADDR_1;
    conn_interval = ll_driver_read_cam(dw_addr) >> 16;
    dw_addr = LE_CAM_ENTRY_BASE(cam_entry) + LE_CAM_ADDR_0;
    rd_value = ll_driver_read_cam(dw_addr);
    real_anchor_point = (rd_value>>4) & (UINT32)(0x1FFFF);

#else
    // Master //
    if (role)
    {
        dw_addr = LE_CAM_ENTRY_BASE(conn_entry) + LE_CAM_ADDR_1;
        conn_interval = ll_driver_read_cam(dw_addr) >> 16;
        dw_addr = LE_CAM_ENTRY_BASE(conn_entry) + LE_CAM_ADDR_0;
        rd_value = ll_driver_read_cam(dw_addr);
        real_anchor_point = (rd_value>>4) & (UINT32)(0x1FFFF);
    }
    else
    {
        conn_interval = RD_LE_REG(LE_REG_SLAVE_CE_INTERVAL);
    }
#endif
    anchor_point = phandle->anchor_point_by_intr;
    ahchor_point_slot = anchor_point>>1;

    conn_interval_slot = conn_interval<<1;

    if (conn_interval_slot > GLOBAL_SLOT_INTERVAL)
    {
        le_slot_offset = ahchor_point_slot % GLOBAL_SLOT_INTERVAL;
    }
    else
    {
        if (conn_interval_slot == 0)
        {
            le_slot_offset = 0; /* avoid exception */
        }
        else
        {
            le_slot_offset = ahchor_point_slot % conn_interval_slot;
        }
    }
    le_slot_offset = (le_slot_offset + 1) & ~0x01;
    if (conn_entry ==ll_manager.conn_unit.first_connection_entry)
    {
        ll_manager.conn_unit.current_slot_offset_for_le = le_slot_offset;
    }
    lmp_force_global_slot_offset(conn_entry + LL_HCI_MIN_CONN_HANDLE,
		                         0, conn_interval_slot, 2,
		                         le_slot_offset + LE_LEGACY_COMPENSATE_SLOT);
/*  dape: originally is lmp_force_global_slot_offset(conn_entry + LL_HCI_MIN_CONN_HANDLE,
		                         0, conn_interval_slot, 2, le_slot_offset);
    change to le_slot_offset+1 is because that le's timing is 23us earlier than legacy.
    so the le_slot_offset should be added by one.

*/
/// dape test
//RT_BT_LOG(BLUE, DAPE_TEST_LOG430, 5, real_anchor_point, ahchor_point_slot,
//ll_manager.conn_unit.first_slot_offset_for_le, le_slot_offset, conn_interval_slot);
}
#endif
#endif
/**************************************************************************
 * Function     : ll_fw_get_update_slot
 *
 * Description  : This function is used for le master to get le slot offset
 *                occupy slots for le connection and if its slot offset
 *                is collision with SCO, then it will do connection update.
 *
 * Parameters   : conn_entry: the connection entry
 *
 * Returns      : result: NONE
 *
 *************************************************************************/
void ll_fw_get_update_slot(UINT8 entry, UINT8 type)
{
    LL_CONN_HANDLE_UNIT *phandle;

    phandle = &ll_manager.conn_unit.handle[entry];

#ifdef _ROM_CODE_PATCHED_
#ifdef _DAPE_8723_RCP_
    if (rcp_ll_update_slot_decision_func != NULL)
    {
        if (rcp_ll_update_slot_decision_func((void*)(&entry)), type)
        {
            return;
        }
    }
#endif
#endif

#ifdef _CCH_SLOT_OFFSET_
#ifdef _DAPE_TEST_SLOT_OFFSET_FOR_LE
    ll_fw_compute_slot_offset_and_occupy(entry, ll_manager.conn_unit.master);
#endif
#endif
    ll_fw_get_slot_offset(entry, ll_manager.conn_unit.master);

    if (type == 2)
    {
        return;
    }
#ifdef _DAPE_GET_LEGACY_SLOT_OFST_AVOID
    UCHAR avoid = 0;
    UCHAR slot_ofst_avoid = 0xFF;
    lmp_get_avoid_slot_offset(&avoid, &slot_ofst_avoid);
#endif

#ifdef _DAPE_AUTO_CONN_UPDT_WHEN_SLOT_CONFLICT_AFTER_CONN_UPDT
RT_BT_LOG(GREEN, DAPE_TEST_LOG548, 10, BB_read_native_clock(), entry,
phandle->conn_counter, phandle->updated_conn_counter,
phandle->ce_interval, phandle->slot_offset, phandle->slave_latency,
phandle->self_updt, avoid, slot_ofst_avoid);
#endif
    if (ll_manager.conn_unit.master && IS_LE_AUTO_CONN_UPDATE_EN)
    {
        LL_INITIATOR_UNIT *initiator = &ll_manager.initiator_unit;
        if (((phandle->slot_offset != ll_manager.conn_unit.current_slot_offset_for_le) &&
                (entry != ll_manager.conn_unit.first_connection_entry) &&
                ll_manager.conn_unit.ce_interval_same)
#ifdef _CCH_SLOT_OFFSET_
#ifdef _DAPE_TEST_SLOT_OFFSET_FOR_LE
                ||((phandle->ce_interval % 3 == 0) &&
                   ((((phandle->slot_offset) % 6 == LE_SLOT_AVOID_FOR_SCO)
#ifdef _DAPE_GET_LEGACY_SLOT_OFST_AVOID
                       && (avoid == 0))
                    || ((avoid == 1) &&
                      ((phandle->slot_offset) % 6 == (slot_ofst_avoid % 6)))
#endif
                    )
#ifdef _DAPE_AUTO_CONN_UPDT_WHEN_SLOT_CONFLICT_AFTER_CONN_UPDT
                   &&(phandle->self_updt == 0)
#endif
                   )
#endif
#endif
           )
        {
            UINT8  update_result = 0;
            UINT16 new_tx_win_offset = 0;
            UINT16 new_slot_offset = 0;
            if ((entry != ll_manager.conn_unit.first_connection_entry) &&
                    (ll_manager.conn_unit.ce_interval_same))
            {
                /* current slot offset is bigger than wanted slot offset*/
                if (phandle->slot_offset >= ll_manager.conn_unit.current_slot_offset_for_le)
                {
                    new_tx_win_offset =  (ll_manager.conn_unit.ce_interval_min << 1) -
                                         (phandle->slot_offset - ll_manager.conn_unit.current_slot_offset_for_le);
                }
                else
                {
                    new_tx_win_offset =
                        (ll_manager.conn_unit.current_slot_offset_for_le - phandle->slot_offset);
                }
            }
#ifdef _CCH_SLOT_OFFSET_
#ifdef _DAPE_TEST_SLOT_OFFSET_FOR_LE
            else
            {
                new_slot_offset = lmp_get_global_slot_offset(
                                      entry + LL_HCI_MIN_CONN_HANDLE, 0,
                                      phandle->ce_interval<<1, 2, 0, GLOBAL_SLOT_INTERVAL);
                if (new_slot_offset > phandle->slot_offset)
                {
                    new_tx_win_offset = new_slot_offset - phandle->slot_offset;
                }
                else
                {
                    new_tx_win_offset = (phandle->ce_interval<<1) -
                                        (phandle->slot_offset - new_slot_offset);
                }
            }
#endif
#endif
            new_tx_win_offset = new_tx_win_offset >> 1;
            ll_driver_fill_conn_win_size_offset(0);
            if (type == 0)
            {
                update_result = ll_fw_conn_update(entry, initiator->tx_win_size,
                                              new_tx_win_offset, initiator->conn_interval,
                                              initiator->conn_latency, initiator->supervision_to);
            }
            else
            {
                update_result = ll_fw_conn_update(entry, phandle->conn_updt_blk.tx_win_size,
                                              new_tx_win_offset,
                                              phandle->conn_updt_blk.ce_interval,
                                              phandle->conn_updt_blk.slave_latency,
                                              phandle->conn_updt_blk.supervision_to);
            }

            if (!update_result)
            {
#ifdef _DAPE_AUTO_CONN_UPDT_WHEN_SLOT_CONFLICT_AFTER_CONN_UPDT
                if (type == 1)
                {
                    phandle->self_updt = 1;
                }
#endif
                ll_manager.conn_unit.current_slot_offset_for_le = phandle->slot_offset;
                RT_BT_LOG(YELLOW, DAPE_TEST_LOG546, 5, type,
                phandle->slot_offset, new_slot_offset,
                phandle->ce_interval, new_tx_win_offset);
            }
        }
    }
}
/**************************************************************************
 * Function     : ll_fw_get_all_anchor_point
 *
 * Description  : This function is used to get all the anchor point
 *                of a master
 *
 * Parameters   : anchor_point_slot_max: the farest anchor point
 *                anchor_point_slot_min: the closest anchor point
 *                next_anchor_point_slot_min: the closest anchor point
 *                                            in the next ce interval.
 *                num_le_conn: the number of le connection
 *                empty_entry: the entry that is not used.
 *                cur_clock_slot: the native clock
 * Returns      : result: NONE
 *
 *************************************************************************/
void ll_fw_get_all_anchor_point(UINT32 *anchor_point_slot_max,
UINT32 *anchor_point_slot_min, UINT32 *next_anchor_point_slot_min,
UINT8 *num_le_conn, UINT16 *empty_entry, UINT32 cur_clock_slot)
{
    UINT8  i;
    UINT32 rd_value;
    UINT8  dw_addr;
    UCHAR  cam_state;
    UINT16 bm_conn_entry = RD_LE_REG(LE_REG_STATUS_CAM_VALID);
    LL_CONN_HANDLE_UNIT *handle;

    for (i = 0; i < LL_MAX_CONNECTION_UNITS; i++)
    {
        /* We shold refer to HW register because CAM is only valid when CAM_VALID==1.*/
        if (bm_conn_entry & (1 << i))
        {
            /* Get Anchor point of each slave */
            dw_addr = LE_CAM_ENTRY_BASE(i) + LE_CAM_ADDR_0;
            rd_value = ll_driver_read_cam(dw_addr);
            cam_state = rd_value & (UINT32)(0xF);
            if (cam_state == 3)
            {
                handle = &ll_manager.conn_unit.handle[i];
                handle->anchor_point_slot = (((rd_value>>4) & (UINT32)(0x1FFFF)) << 1)
                                            | (cur_clock_slot & (0xFFFC0000));
                handle->next_anchor_point_slot = handle->anchor_point_slot + (handle->ce_interval<<1);
                if (handle->anchor_point_slot > *anchor_point_slot_max)
                {
                    *anchor_point_slot_max = handle->anchor_point_slot;
                }
                if (handle->anchor_point_slot < *anchor_point_slot_min)
                {
                    *anchor_point_slot_min = handle->anchor_point_slot;
                }
                if (handle->next_anchor_point_slot < *next_anchor_point_slot_min)
                {
                    *next_anchor_point_slot_min = handle->next_anchor_point_slot;
                }
            }
            (*num_le_conn)++;
        }
        else
        {
            if (i < *empty_entry)
            {
                *empty_entry = i;
            }
        }
    }
}

#ifdef _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_
BOOLEAN ll_fw_validate_ll_length_param(UINT16 size, UINT16 time)
{
    if (size < LE_CONN_MAX_TX_SIZE_MIN || size > LE_CONN_MAX_TX_SIZE_MAX
            || time < LE_CONN_MAX_TX_TIME_MIN || time > LE_CONN_MAX_TX_TIME_MAX)
    {
        return FALSE;
    }
    return TRUE;
}

/**
 * @brief Update effective data length parameters.
 * @param chu  Connection handle unit.
 * @return  \c TRUE if effective data length parameters are changed; otherwise,
 * \c FALSE.
 */
BOOLEAN ll_fw_update_data_length(LL_CONN_HANDLE_UNIT *chu)
{
    UINT16 old_max_tx_size = chu->data_len_updt.max_tx_size;
    UINT16 old_max_tx_time = chu->data_len_updt.max_tx_time;
    UINT16 old_max_rx_size = chu->data_len_updt.max_rx_size;
    UINT16 old_max_rx_time = chu->data_len_updt.max_rx_time;
    UINT16 max_tx_size = MIN(chu->data_len_updt.local_max_tx_size,
            chu->data_len_updt.remote_max_rx_size);
    UINT16 max_rx_size = MIN(chu->data_len_updt.local_max_rx_size,
            chu->data_len_updt.remote_max_tx_size);
    UINT16 max_tx_time = MIN(chu->data_len_updt.local_max_tx_time,
            chu->data_len_updt.remote_max_rx_time);
    UINT16 max_rx_time = MIN(chu->data_len_updt.local_max_rx_time,
            chu->data_len_updt.remote_max_tx_time);
    BOOLEAN tx_changed = (old_max_tx_size != max_tx_size
            || old_max_tx_time != max_tx_time);

    chu->data_len_updt.max_tx_size = max_tx_size;
    chu->data_len_updt.max_rx_size = max_rx_size;
    chu->data_len_updt.max_tx_time = max_tx_time;
    chu->data_len_updt.max_rx_time = max_rx_time;
    chu->data_len_updt.tx_changed = tx_changed;
    return (tx_changed || old_max_rx_size != max_rx_size
            || old_max_rx_time != max_rx_time);
}

/**
 * @brief Pend a data length update procedure.
 * @param chu  Connection handle unit instance.
 * @return  \c TRUE if success, \c FALSE on error.
 */
BOOLEAN ll_fw_pend_data_length_update_procedure(LL_CONN_HANDLE_UNIT *chu)
{
    chu->llc.pend_data_len_updt = TRUE;
    OS_SIGNAL signal = {
            .type = LL_CONTROL_SIGNAL,
            .param = (OS_ADDRESS) ((UINT32) chu->unit_id)
    };
    return (OS_ISR_SEND_SIGNAL_TO_TASK(
            rx_table[LE_ACL_DATA_HANDLER_TASK].task_handle, signal)
            == BT_ERROR_OK);
}

#ifdef _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_INFO_
void ll_fw_print_data_length(LL_CONN_HANDLE_UNIT *chu)
{
    LL_LOG_TRACE(GREEN, LE_MSG_LOCAL_DATA_LEN_INFO, 4,
            chu->data_len_updt.local_max_tx_size,
            chu->data_len_updt.local_max_tx_time,
            chu->data_len_updt.local_max_rx_size,
            chu->data_len_updt.local_max_rx_time);
    LL_LOG_TRACE(GREEN, LE_MSG_REMOTE_DATA_LEN_INFO, 4,
            chu->data_len_updt.remote_max_tx_size,
            chu->data_len_updt.remote_max_tx_time,
            chu->data_len_updt.remote_max_rx_size,
            chu->data_len_updt.remote_max_rx_time);
    LL_LOG_TRACE(GREEN, LE_MSG_DATA_LEN_INFO, 4,
            chu->data_len_updt.max_tx_size,
            chu->data_len_updt.max_tx_time,
            chu->data_len_updt.max_rx_size,
            chu->data_len_updt.max_rx_time);
}
#endif
#endif /* _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_ */
#ifdef _BT4_1_SUPPORT_LL_CONN_PARAM_REQ
void ll_decide_connection_parameters(UINT8 entry)
{
    LL_CONN_PARAM_REQ_BLK *pConnParamReqBlk = NULL;
    pConnParamReqBlk = &ll_manager.conn_unit.handle[entry].conn_param_req_blk;
    pConnParamReqBlk->PreferredPeriodicity = 0; /* temporarily no preferred. */
    memset(&pConnParamReqBlk->RefConnEventCount, 0xFF, 14); /* To set the values to invalid. */
#ifdef _ROM_CODE_PATCHED_
    if (rcp_ll_decide_connection_parameters_func != NULL)
    {
        if(rcp_ll_decide_connection_parameters_func((void*)&entry))
        {
            return;
        }
    }
#endif
    /* To be continued... */
}
#endif
