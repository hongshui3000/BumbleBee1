enum { __FILE_NUM__= 201 };

#include "bt_fw_types.h"
#include "le_ll.h"
#include "le_hw_reg.h"
#include "mem.h"
#include "bt_fw_hci_spec_defines.h"
#include "bt_fw_hci.h"
#include "bt_fw_hci_internal.h"
#include "le_ll_driver.h"
#include "le_hci_4_0.h"
#include "bzdma.h"
#include "hci_td.h"
#include "dma_usb.h"
#include "lc_internal.h"
#include "lmp_vendor_defines.h"
#include "mint_os.h"
#include "gpio.h"
#ifdef LE_HW_TEST
#include "le_ll_test.h"
#endif
#ifdef CONFIG_TV_FEATURE
#include "tv.h"
#endif

#ifdef _DAPE_TEST_SLOT_OFFSET_FOR_LE
extern UINT16 g_le_conn_interval_changed;
#endif
#ifdef _ROM_CODE_PATCHED_
PF_ROM_CODE_PATCH_FUNC rcp_ll_handle_adv_channel_pdu = NULL;
#endif

extern UINT8 g_block_legacy_for_le_slot_num;
extern UINT8 g_le_use_interval_slot_pair;
LL_MANAGER ll_manager;
POOL_ID le_llc_pdu_pool_id;         /* LE LLC PDU buffers pool */
POOL_ID le_acl_data_to_ll_pool_id;  /* LE ACL Data buffers pool (HCI to LL)*/
POOL_ID le_acl_data_to_host_pool_id;/* LE ACL Data buffers pool (LL to HCI)*/
LL_DEV_ADDR_LIST_ENTRY le_white_entry[LL_MAX_WHITE_LIST_SIZE];
LL_DEV_ADDR_LIST_ENTRY le_black_entry[LL_MAX_BLACK_LIST_SIZE];
#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_
UINT8 le_resolving_list_size = LL_MAX_HW_RESOLVING_LIST_SZ;
SECTION_LOW_BSS LL_RESOLVING_LIST_ENTRY le_resolving_list[LL_MAX_HW_RESOLVING_LIST_SZ];
TimerHandle_t resolvable_private_address_timeout_timer = NULL;
#endif
SECTION_SRAM UINT8 ll_adv_data_tx_buf[LL_ADV_DATA_LEN_MAX + 1];
SECTION_SRAM UINT8 ll_scan_res_data_tx_buf[LL_SCAN_RESPONSE_DATA_LEN_MAX + 1];
SECTION_SRAM UINT8 ll_hw_rxfifo_buf[LL_HW_RXBUFF_SIZE];
FW_BT_RX_INT_FIFO_S fw_bt_rx_int_fifo;

UINT8 le_tx_power_max_value = 0x44;
UINT8 le_tx_power_max_index = 6;

#ifdef _ADD_RX_BACKUP_STATUS_INSIDE_BLE_RX_PKT_
UINT8 le_rx_backup_status_cnts = 0;
#endif

#if defined(_SCHEDULE_BLE_MISC_PACKET_) && defined(_NEW_BZDMA_FROM_V8_)
LL_MISC_ACL_DATA_PKT_NODE le_tx_misc_node[LL_MISC_ACL_DATA_PKT_MAX_NODES];
LE_ACL_MISC_PKT_LIST_MANAGE le_tx_misc_free_list;
#ifdef _NEW_BZDMA_FROM_V8_PROGRAMMABLE_CONFIG_SETTING_
BZDMA_BLEONLY_TX_DESC_FRAGMENT *pll_misc_acl_pkt_node_all_frag_base = NULL;
#ifdef _SUPPORT_LE_HCI_TX_PKT_FRAGMENT_REASSEMBLE_FUNC_
LL_HCI_ACL_DATA_FRAGMENT_INFO *pll_misc_acl_pkt_node_all_frag_info_base = NULL;
#endif
#endif
#endif

extern OS_HANDLE isr_extended_task_handle;

/* violate with core spec, but we can disable it via patch .. (austin) */
UINT8 g_le_auto_enable_event_mask_for_new_spec = TRUE;

/**************************************************************************
 * Function     : ll_fw_init
 *
 * Description  : This function is used to initiate fw global varibles for LL
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
void ll_fw_init(void)
{
    UINT8 i;
    LE_REG_S_INT_IMR *pint_imr;
    LE_REG_S_LE_EXT_MISR *pext_int_imr;

    /* init global variable to zero */
    memset(&ll_manager, 0, sizeof(ll_manager));

    /* init hci part */
    ll_manager.le_event_mask[0] = LE_DEFAULT_EVENT_LSB_MASK;
    ll_manager.cur_mode = LL_MODE_NORMAL;
    ll_manager.adv_tx_power_idx = LE_DEAULT_ADV_TX_INDEX;
#ifdef _DAPE_UNIFY_VERSION_FEATURE_UINT
    if (IS_BT50)
    {
        ll_manager.ll_version =  BT_FW_LL_VERSION_BT50;
    }
    else if (IS_BT42)
    {
        ll_manager.ll_version =  BT_FW_LL_VERSION_BT42;
    }
    else if (IS_BT41)
    {
        ll_manager.ll_version =  BT_FW_LL_VERSION_BT41;
    }
    else
    {
        ll_manager.ll_version =  BT_FW_LL_VERSION_BT40;
    }
#endif

    /* copy local public device address */
    memcpy(ll_manager.local_public_address, otp_str_data.bt_bd_addr, 6);

    /* init advertising unit */
    ll_manager.adv_unit.enable = 0;
    ll_manager.adv_unit.local_addr_type = LL_DEFAULT_ADV_ADDR_TYPE;
    ll_manager.adv_unit.direct_addr_type = LL_DEFAULT_ADV_ADDR_TYPE;
    ll_manager.adv_unit.filter_conn_req = 0;
    ll_manager.adv_unit.filter_scan_req = 0;
    ll_manager.adv_unit.pdu_type = LL_DEFAULT_ADV_TYPE;
    ll_manager.adv_unit.hci_pdu_type = LL_HCI_ADV_TYPE_ADV_IND;
    ll_manager.adv_unit.ch_map = LL_DEFAULT_ADV_CH_MAP;
    ll_manager.adv_unit.interval_min = LL_ADV_INTERVAL_MIN;
    ll_manager.adv_unit.interval_max = LL_ADV_INTERVAL_MAX;
    ll_manager.adv_unit.interval = LL_DEFAULT_ADV_INTERVAL;
    ll_manager.adv_unit.adv_buf = NON_DCATCH_PTR(ll_adv_data_tx_buf);
    ll_manager.adv_unit.adv_len = 0;
    ll_manager.adv_unit.scan_rsp_buf = NON_DCATCH_PTR(ll_scan_res_data_tx_buf);
    ll_manager.adv_unit.scan_rsp_len = 0;
#ifdef _NEW_BLE_HW_SPEC_FROM_150320_
    ll_manager.adv_unit.h2h_period_en = 0;
#endif

    /* init scanning unit */
    ll_manager.scan_unit.enable = 0;
    ll_manager.scan_unit.local_addr_type = LL_DEFAULT_SCAN_ADDR_TYPE;
    ll_manager.scan_unit.filter_policy = 0;
    ll_manager.scan_unit.filter_duplicate = 0;
    ll_manager.scan_unit.active_scan = 0;
    ll_manager.scan_unit.ch_map = LL_DEFAULT_SCAN_CH_MAP;
    ll_manager.scan_unit.interval = LL_DEFAULT_SCAN_INTERVAL;
    ll_manager.scan_unit.window = LL_DEFAULT_SCAN_WINDOW;

    /* init initiator unit */
    ll_manager.initiator_unit.enable = 0;
    ll_manager.initiator_unit.local_addr_type = LL_ADDR_TYPE_PUBLIC;
    ll_manager.initiator_unit.remote_addr_type = LL_ADDR_TYPE_PUBLIC;
    ll_manager.initiator_unit.filter_policy = 0;
    ll_manager.initiator_unit.ch_map = LL_DEFAULT_INIT_CH_MAP;
    ll_manager.initiator_unit.scan_interval = LL_DEFAULT_INITIATOR_SCAN_INTERVAL;
    ll_manager.initiator_unit.scan_window = LL_DEFAULT_INITIATOR_SCAN_WINDOW;
    ll_manager.initiator_unit.supervision_to = LL_DEFAULT_SUPERVISION_TO;
    ll_manager.initiator_unit.conn_latency = LL_DEFAULT_CONN_LATENCY;
    ll_manager.initiator_unit.conn_interval_min = LL_CONN_INTERVAL_MIN;
    ll_manager.initiator_unit.conn_interval_max = LL_CONN_INTERVAL_MAX;
    ll_manager.initiator_unit.conn_interval = LL_DEFAULT_CONN_INTERVAL;
    ll_manager.initiator_unit.ce_len_min = LL_CE_LENGTH_MIN;
    ll_manager.initiator_unit.ce_len_max = LL_CE_LENGTH_MAX;
    ll_manager.initiator_unit.tx_win_offset = LL_TX_WIN_OFFSET_MIN;
    ll_manager.initiator_unit.tx_win_size = LL_TX_WIN_SIZE_MAX;
    ll_manager.initiator_unit.ce_len = LL_DEFAULT_CE_LENGTH;
    ll_manager.initiator_unit.conn_ch_map[0] = 0xFF;
    ll_manager.initiator_unit.conn_ch_map[1] = 0xFF;
    ll_manager.initiator_unit.conn_ch_map[2] = 0xFF;
    ll_manager.initiator_unit.conn_ch_map[3] = 0xFF;
    ll_manager.initiator_unit.conn_ch_map[4] = 0x1F;
    ll_manager.initiator_unit.hop_increment = LL_HOP_INCREMENT_MIN;

    /* init connection unit */
    ll_manager.conn_unit.enable = 0;
    ll_manager.conn_unit.support_enc = 1;
#ifdef _DAPE_UNIFY_VERSION_FEATURE_UINT
    if (IS_BT40)
    {
        ll_manager.ll_feature_support[0] |= SUPPORT_LE_ENCRYPTION;
    }
    if (IS_BT41)
    {
        ll_manager.ll_feature_support[0] |= (SUPPORT_REJ_IND_EXT |
                                               SUPPORT_LE_PING | SUPPORT_SLV_INIT_FEATURE_REQ);
    }
#endif

    if (IS_BT42)
    {
#ifdef _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_
#ifdef _DAPE_UNIFY_VERSION_FEATURE_UINT
        ll_manager.ll_feature_support[0] |= SUPPORT_LE_DATA_PKT_LEN_EXT;
#endif

        ll_manager.conn_unit.support_dle = 1;

        if (ll_manager.conn_unit.support_dle &&
                        g_le_auto_enable_event_mask_for_new_spec)
        {
            ll_manager.le_event_mask[0] |= LE_DATA_LENGTH_CHANGE_EVENT_MASK;
        }
#endif
#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_
#ifdef _DAPE_UNIFY_VERSION_FEATURE_UINT
        ll_manager.ll_feature_support[0] |= (SUPPORT_LL_PRIVACY | SUPPORT_EXT_SCANNER_FILTER_POLICIES);
#endif

        ll_manager.support_ll_privacy = 1;
        ll_manager.comp_rpa_lbd_opt_enable = 1;
        ll_manager.force_bt40_opt = 1;

        if (ll_manager.support_ll_privacy &&
                        g_le_auto_enable_event_mask_for_new_spec)
        {
            ll_manager.le_event_mask[1] |= (LE_ENHANCED_CONNECTION_COMPLETE_EVENT_MASK >> 8);
            ll_manager.le_event_mask[1] |= (LE_DIRECT_ADVERTISING_REPORT_EVENT_MASK >> 8);
        }

        switch (LL_MAX_RESOLVING_LIST_SIZE)
        {
            case 4:
                UPDATE_LE_REG(LE_REG_PRIVACY_MISC1, 0x3, 0x0);
                break;

            case 8:
                UPDATE_LE_REG(LE_REG_PRIVACY_MISC1, 0x3, 0x1);
                break;

            case 16:
            default:
                le_resolving_list_size = LL_MAX_HW_RESOLVING_LIST_SZ;
                UPDATE_LE_REG(LE_REG_PRIVACY_MISC1, 0x3, 0x2);
                break;
        }

#endif
    }
#ifdef _SUPPORT_VER_5_0_
    if (IS_BT50)
    {
        ll_manager.ll_feature_support[1] |= ((SUPPORT_2MBPS_PHY >> 8)| (SUPPORT_STABLE_MODULATION_IDX >> 8));
        ll_manager.support_le_2mbps = 1;

        if (ll_manager.support_le_2mbps &&
                        g_le_auto_enable_event_mask_for_new_spec)
        {
            ll_manager.le_event_mask[1] |= (LE_PHY_UPDATE_COMPLETE_EVENT_MASK >> 8);
        }        
    }
#endif
    ll_manager.conn_unit.conn_updt_entry = LL_MAX_CONNECTION_UNITS;
    ll_manager.conn_unit.chm_updt_entry = LL_MAX_CONNECTION_UNITS;
    ll_manager.conn_unit.ce_interval_same = 0;
    ll_manager.conn_unit.ce_interval_min = 0xFFFF;
#ifdef _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_
    ll_manager.conn_unit.init_max_tx_size = LE_CONN_MAX_TX_SIZE_MIN;
    ll_manager.conn_unit.init_max_tx_time = LE_CONN_MAX_TX_TIME_MIN;
#endif

    for (i = 0; i < LL_MAX_CONNECTION_UNITS; i++)
    {
        ll_manager.conn_unit.handle[i].unit_id = i;
#ifdef USE_FREERTOS
        OS_CREATE_TIMER(ONESHOT_TIMER, &ll_manager.conn_unit.handle[i].sup_timer,
                ll_supervision_timeout_callback, (void *)((UINT32) i | (LL_TIMER_TYPE_SUPERVISION << 8)), 0);
        OS_CREATE_TIMER(ONESHOT_TIMER, &ll_manager.conn_unit.handle[i].llc_timer,
                llc_timeout_callback, (void *)((UINT32) i | (LL_TIMER_TYPE_LLC << 8)), 0);
#else
        ll_manager.conn_unit.handle[i].sup_timer = NULL;
        ll_manager.conn_unit.handle[i].llc_timer = NULL;
#endif
        ll_manager.conn_unit.handle[i].conn_handle = LL_HCI_MIN_CONN_HANDLE + i;

#if defined(_SCHEDULE_BLE_MISC_PACKET_) && defined(_NEW_BZDMA_FROM_V8_)
#ifndef USE_NEW_LE_SCHEDULER
        ll_manager.conn_unit.handle[i].tx_resent_misc_pkt = NULL;
        ll_manager.conn_unit.handle[i].tx_sched_misc_pkt_list.DWord = LL_MISC_ACL_PKT_LIST_RESET_VALUE;
#endif
        ll_manager.conn_unit.handle[i].tx_pend_misc_pkt_list.DWord = LL_MISC_ACL_PKT_LIST_RESET_VALUE;
        ll_manager.conn_unit.handle[i].h2l_free_misc_pkt_list.DWord = LL_MISC_ACL_PKT_LIST_RESET_VALUE;
#endif

#ifdef _BT4_1_LE_PING_FEATURE_SUPPORT_
        ll_manager.conn_unit.handle[i].ping_req_timer = OS_INVALID_HANDLE;
        ll_manager.conn_unit.handle[i].auth_payload_timeout = LL_AUTHENTICATED_PAYLOAD_TIMEOUT;
#endif
    }

    /* init white list entries and relative list */
    for (i = 0; i < LL_MAX_WHITE_LIST_SIZE; i++)
    {
        le_white_entry[i].DWord[0] = 0;
        le_white_entry[i].DWord[1] = 0;
        le_white_entry[i].next = i + 1;
    }
    ll_manager.white_list.free_list = 0;                      /* full list */
    ll_manager.white_list.list_head = LL_MAX_WHITE_LIST_SIZE; /* empty list */
    ll_manager.white_list.list_tail = LL_MAX_WHITE_LIST_SIZE; /* empty list */
    ll_manager.white_list.entry = le_white_entry;

    /* init black list entries and relative list */
    for (i = 0; i < LL_MAX_BLACK_LIST_SIZE; i++)
    {
        le_black_entry[i].DWord[0] = 0;
        le_black_entry[i].DWord[1] = 0;
        le_black_entry[i].next = i + 1;
    }
    ll_manager.black_list.free_list = 0;                      /* full list */
    ll_manager.black_list.list_head = LL_MAX_BLACK_LIST_SIZE; /* empty list */
    ll_manager.black_list.list_tail = LL_MAX_BLACK_LIST_SIZE; /* empty list */
    ll_manager.black_list.entry = le_black_entry;

#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_
    ll_manager.RPA_timer = &resolvable_private_address_timeout_timer;
    if (*ll_manager.RPA_timer != NULL)
    {
        OS_DELETE_TIMER(ll_manager.RPA_timer);
    }
    ll_manager.address_resolution_enable = FALSE;
    ll_manager.local_IRK_idx = LL_MAX_RESOLVING_LIST_SIZE;

    /* init resolving list */
    for (i = 0; i < LL_MAX_RESOLVING_LIST_SIZE; i++)
    {
        memset(le_resolving_list[i].DWord, 0, 13 * sizeof(UINT32));
        le_resolving_list[i].next = i + 1;
    }
    ll_manager.resolving_list.free_list = 0;                          /* full list */
    ll_manager.resolving_list.list_head = LL_MAX_RESOLVING_LIST_SIZE; /* empty list */
    ll_manager.resolving_list.list_tail = LL_MAX_RESOLVING_LIST_SIZE; /* empty list */
    ll_manager.resolving_list.item = le_resolving_list;
#endif /* _BT4_2_PRIVACY_FEATURE_SUPPORT_ */

    /* init interrupt mask of sw */
    ll_manager.int_imr = 0xFFFF;
    pint_imr = (LE_REG_S_INT_IMR *)&ll_manager.int_imr;
    pint_imr->event_end_int_mask = 0;
    pint_imr->event_begin_int_mask = 0;
    pint_imr->pkt_rx_mask = 1;
    pint_imr->pkt_tx_mask = 1;
    pint_imr->cam_int_mask = 1;
    pint_imr->rx_th_int_mask = 0;
    pint_imr->scan_int_mask = 0;
    pint_imr->conn_int_mask = 0;
    pint_imr->adv_int_mask = 0;
    pint_imr->tx_thr_int_mask = 0;
    pint_imr->conn_updt_ce_e_mask = 0;
    pint_imr->conn_updt_tw_s_mask = 0;
    pint_imr->chm_updt_ce_s_mask = 0;
    pint_imr->hit_conn_adv_mask = 0;

#ifdef _DAPE_EN_8821_MP_LE_SCAN_INTR
    ll_manager.ext_int_imr = 0x0000;
    pext_int_imr = (LE_REG_S_LE_EXT_MISR *)&ll_manager.ext_int_imr;
    pext_int_imr->conn_early_en = 1;
    pext_int_imr->scan_early_en = 1;
    pext_int_imr->scan_early_time = 2; /* 2 slots */
#endif
#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_
    if (ll_manager.support_ll_privacy)
    {
        pext_int_imr->scan_req_en = 1;
    }
#endif

#if defined(_SCHEDULE_BLE_MISC_PACKET_) && defined(_NEW_BZDMA_FROM_V8_)
    /* init the free list */
#ifdef _NEW_BZDMA_FROM_V8_PROGRAMMABLE_CONFIG_SETTING_
    UINT8 offset = 0;

    if (pll_misc_acl_pkt_node_all_frag_base == NULL)
    {
        pll_misc_acl_pkt_node_all_frag_base =
            (BZDMA_BLEONLY_TX_DESC_FRAGMENT *)os_malloc (
                        (sizeof(BZDMA_BLEONLY_TX_DESC_FRAGMENT)*
                                        LL_MISC_ACL_DATA_PKT_MAX_NODES *
                                        bzdma_supported_le_max_frag_num),
                                        RAM_TYPE_DATA_ON);
    }
#ifdef _SUPPORT_LE_HCI_TX_PKT_FRAGMENT_REASSEMBLE_FUNC_
    if (pll_misc_acl_pkt_node_all_frag_info_base == NULL)
    {
        pll_misc_acl_pkt_node_all_frag_info_base =
                (LL_HCI_ACL_DATA_FRAGMENT_INFO *) os_malloc(
                        sizeof (LL_HCI_ACL_DATA_FRAGMENT_INFO)
                        * LL_MISC_ACL_DATA_PKT_MAX_NODES
                        * bzdma_supported_le_max_frag_num,
                        RAM_TYPE_DATA_ON);
    }
#endif
#endif

    for (i = 0; i < LL_MISC_ACL_DATA_PKT_MAX_NODES; i++)
    {
        le_tx_misc_node[i].MyNodeId = i;
        le_tx_misc_node[i].NextNodeId = i + 1;
#ifdef _NEW_BZDMA_FROM_V8_PROGRAMMABLE_CONFIG_SETTING_
        le_tx_misc_node[i].pFrags = &pll_misc_acl_pkt_node_all_frag_base[offset];
#ifdef _SUPPORT_LE_HCI_TX_PKT_FRAGMENT_REASSEMBLE_FUNC_
        le_tx_misc_node[i].frag_info = &pll_misc_acl_pkt_node_all_frag_info_base[offset];
#endif
        offset += bzdma_supported_le_max_frag_num;
#endif
    }
    le_tx_misc_free_list.MiscHeadId = 0;
    le_tx_misc_free_list.MiscTailId = LL_MISC_ACL_DATA_PKT_MAX_NODES - 1;
    le_tx_misc_free_list.pktcnt = LL_MISC_ACL_DATA_PKT_MAX_NODES;
#endif

#ifdef USE_NEW_LE_SCHEDULER
    le_sched_pkt_fifo_init();
#endif

    /* init rx fifo manager */
    ll_manager.event_manager.event_type = LL_CHANNEL_TYPE_ADVERTISING;

    memset(&fw_bt_rx_int_fifo, 0, sizeof(fw_bt_rx_int_fifo));

#ifdef _LE_AUTO_REPORT_RSSI_AND_LOGIN_INOUT_FUNC_
    memset(&ll_rssi_manager, 0, sizeof(LL_CONN_HANDLE_RSSI_MANAGER));
#endif
    g_block_legacy_for_le_slot_num = ((otp_str_data.bt_le_fw_policy >> 21) & 0x07);
    g_le_use_interval_slot_pair = ((otp_str_data.bt_func_support_policy_ext >> 18) & 0x0F);

#ifdef LE_ISSUE_INSTR_IN_ORDER_ON_COMPLT
    le_instr_queue_init();
#endif

#ifdef _ENABLE_BLE_LL_FLOW_CONTROL_
    os_reset_le_reserved_buffer();

    g_le_enter_flow_stop_flag = FALSE;
    g_le_flow_stop_threshold_pkt_cnt = LL_POLL_HCI_MAX_RX_ACL_PKT_CNT >> 2;
    g_le_flow_go_threshold_pkt_cnt =  (LL_POLL_HCI_MAX_RX_ACL_PKT_CNT >> 2) * 3;
#endif
}

/**************************************************************************
 * Function     : ll_hw_init
 *
 * Description  : This function is used to initiate HW registers for LL
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
void ll_hw_init(UINT8 dlps_flow)
#else
void ll_hw_init(void)
#endif
{
    LE_REG_S_SET ll_reg;
    UINT8 i;

    /* mask all interrupts of ll controller */
    WR_LE_REG(LE_REG_INT_IMR, 0xFFFF);

    /* enable whitening/dewhiening and set set rx pkt filter */
    WR_LE_REG(LE_REG_RX_PKT_ERR_CTRL, BIT8);

    /* avoid to enable rf test mode */
    WR_LE_REG(LE_REG_RF_TEST_CONTROL, 0);

    /* dape added - suggested by kevin to avoid hw bug */
    ll_reg.value = RD_LE_REG(LE_REG_CE_END_CTRL);
    ll_reg.ce_end_ctrl.slave_ce_len_end = TRUE;
    WR_LE_REG(LE_REG_CE_END_CTRL, ll_reg.value);

    /* let random number be updated periodically */
    WR_LE_REG(LE_REG_RANDOM_NUM_CTRL, 0x04);

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
    // by store/restore
    if(dlps_flow == FALSE)
#endif
    {
        /* disable slave latency */
        ll_reg.value = RD_LE_REG(LE_REG_SLAVE_WIN_WIDENING_L);
        ll_reg.slave_win_widen_l.sub_en = FALSE;
#ifdef _NEW_BLE_HW_SPEC_FROM_150320_
        ll_reg.slave_win_widen_l.adv_h2h_en =
                    ll_manager.adv_unit.h2h_period_en ? TRUE : FALSE;
#endif
        WR_LE_REG(LE_REG_SLAVE_WIN_WIDENING_L, ll_reg.value);

        /* set default power to maximum */
        ll_reg.value = RD_LE_REG(LE_REG_SLAVE_CONN_WIN_SIZE);
        ll_reg.slave_win_size.slave_tx_power = LE_DEAULT_HW_TX_POWER;
        WR_LE_REG(LE_REG_SLAVE_CONN_WIN_SIZE, ll_reg.value);

        /* set advertising channel tx power */
        ll_reg.value = RD_LE_REG(LE_REG_CBK_CONTROL);
        ll_reg.cbk_ctrl.adv_tx_power = LE_DEAULT_ADV_TX_POWER;
#ifdef _SUPPORT_LBT_FUNC_FOR_RF_ADAPTIVITY_
        if (rtl8723_btrf_check_and_enable_lbt(LE_DEAULT_ADV_TX_INDEX))
        {
            /* enable lbt_en_lut bit */
            ll_reg.cbk_ctrl.lbt_en_adv = TRUE;
        }
#endif
        WR_LE_REG(LE_REG_CBK_CONTROL, ll_reg.value);

        /* write local public device address to HW */
        WR_LE_REG(LE_REG_DEVA_PUBLIC_LOCAL_L, ll_manager.u2local_public_addr[0]);
        WR_LE_REG(LE_REG_DEVA_PUBLIC_LOCAL_M, ll_manager.u2local_public_addr[1]);
        WR_LE_REG(LE_REG_DEVA_PUBLIC_LOCAL_H, ll_manager.u2local_public_addr[2]);

        /* set my slave clock accuracy */
        ll_reg.value = RD_LE_REG(LE_REG_SLAVE_CH_MAP_H);
        ll_reg.slave_ch_map_h.local_sca = LL_LOCAL_SCA;
        WR_LE_REG(LE_REG_SLAVE_CH_MAP_H, ll_reg.value);
    }

    /* init ce early interrupt time */
    WR_LE_REG(LE_REG_INT_CE_EARLY_INT_TIME, otp_str_data.le_ce_early_int_time);

#ifdef _NEW_BLE_HW_SPEC_FROM_150320_
    /* init ce early guard duration (trx abort boundary to next ce anchor point) */
    UINT16 guard_time_us = otp_str_data.le_ce_early_int_time & 0x3ff;
    UINT16 guard_time_slot = (otp_str_data.le_ce_early_int_time >> 10) & 0x01;
    guard_time_us += 200;
    if (guard_time_us > 624)
    {
        guard_time_us -= 625;
        guard_time_slot += 1;
    }
    //guard_time_slot += g_efuse_lps_setting_2.early_quard_time_slot_add;
    if(guard_time_slot > 7)
    {
        guard_time_slot = 7;
    }
    WR_LE_REG(LE_REG_CE_EARLY_GUARD_TIME, (guard_time_slot << 10) | guard_time_us);
#endif

#ifdef _DAPE_ENABLE_LE_EARLY_TX_TOGGLE
        otp_str_data.le_tx_turnaround_delay |= BIT14;
#endif
    /* adjust the timing for tx on early and tx2rx delay */
    WR_LE_REG(LE_REG_DELAY_TX_TURNAROUND, otp_str_data.le_tx_turnaround_delay);

    /* adjust the timing for rx2tx delay */
    WR_LE_REG(LE_REG_DELAY_RX_TURNAROUND, otp_str_data.le_rx_turnaround_delay);

    /* adjust the timing for tx on delay and rx on delay */
    WR_LE_REG(LE_REG_DELAY_TRX_ON, otp_str_data.le_trx_on_delay);

    /* adjust the timing for tx on extend and rx timeout delay */
    WR_LE_REG(LE_REG_DELAY_CE_RX_TIMEOUT, otp_str_data.le_ce_rx_timeout_delay);

    /* adjust rx search to value */
    WR_LE_REG(LE_REG_DELAY_RX_SEARCH_TIMEOUT,
                                    otp_str_data.le_rx_search_timeout_value);

    /* adjust le clock compensate */
    WR_LE_REG(LE_REG_CLK_COMPENSATE, otp_str_data.le_clock_compensate);

#ifdef _ADD_RX_BACKUP_STATUS_INSIDE_BLE_RX_PKT_
    /* enable ble rx backup register in rx fifo */
    if (le_rx_backup_status_cnts > 5)
    {
        le_rx_backup_status_cnts = 5;
    }
    ll_reg.value = RD_LE_REG(LE_REG_MODEM_CONTROL);
    ll_reg.modem_ctrl.backup_rpt_en = (1 << le_rx_backup_status_cnts) - 1;
    WR_LE_REG(LE_REG_MODEM_CONTROL, ll_reg.value);
#endif

#ifdef _LE_IRQ_MASK_EN_
    /* configure le early interrupt time */
    WR_LE_REG(LE_REG_PRE_MASK_TIME, LL_CE_BEGIN_IRQ_MASK_SETTING);

    /* adjust le early interrupt irq mask */
    UINT32 reg_value;
    reg_value = VENDOR_READ(0x34);
    reg_value &= ~(0x1F << 16);
    reg_value |=  (0x1F << 16); /* mask log uart/timer/gpio/bzdma/hcidma int */
    VENDOR_WRITE(0x34, reg_value);
#endif

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
    // by store/restore
    if(dlps_flow == FALSE)
#endif
    {
        /* dape added for scan priority control. Set FALSE for not stop scanning */
	/* at the end of scan window if no scan rsp is received*/
        ll_reg.value = RD_LE_REG(LE_REG_SCAN_CONTROL);
        ll_reg.scan_ctrl.scan_pri_ctrl= FALSE;
        WR_LE_REG(LE_REG_SCAN_CONTROL, ll_reg.value);

#ifdef _DAPE_TEST_NEW_HW_BLOCK_ACL_FOR_LE
        if(BLOCK_LEGACY_WHEN_LE)
        {
            ll_driver_block_legacy_slot_for_le(g_block_legacy_for_le_slot_num);
        }
#endif

        /* initialize the white list content in the CAM */
        for (i = 0; i < (LL_MAX_WHITE_LIST_SIZE << 1); i++)
        {
            ll_driver_write_cam(LE_CAM_WHITE_LIST_BASE + i, 0);
        }

        /* initialize the black list content in the CAM */
        for (i = 0; i < (LL_MAX_BLACK_LIST_SIZE << 1); i++)
        {
            ll_driver_write_cam(LE_CAM_BLACK_LIST_BASE + i, 0);
        }

#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_
        /* initialize valid register of the resolving list */
        WR_LE_REG(LE_REG_LOCAL_IRK_VALID_L, 0x0);
        WR_LE_REG(LE_REG_LOCAL_IRK_VALID_H, 0x0);
        WR_LE_REG(LE_REG_PEER_IRK_VALID_L, 0x0);
        WR_LE_REG(LE_REG_PEER_IRK_VALID_H, 0x0);
        WR_LE_REG(LE_REG_PEER_ADDR_VALID_L, 0x0);
        WR_LE_REG(LE_REG_PEER_ADDR_VALID_H, 0x0);

        if (ll_manager.comp_rpa_lbd_opt_enable)
        {
            LE_REG_S_SET reg;
            reg.value = RD_LE_REG(LE_REG_PRIVACY_MISC0);
            reg.le_privacy_misc0.comp_rpa_lbd_opt = 1;
            WR_LE_REG(LE_REG_PRIVACY_MISC0, reg.value);
        }

        if (ll_manager.force_bt40_opt)
        {
            LE_REG_S_SET reg;
            reg.value = RD_LE_REG(LE_REG_PRIVACY_MISC1);
            reg.le_privacy_misc1.force_bt40_opt = 1;
            WR_LE_REG(LE_REG_PRIVACY_MISC1, reg.value);
        }
#endif /* _BT4_2_PRIVACY_FEATURE_SUPPORT_ */

#ifdef _NEW_BZDMA_FROM_V8_
        /* initialize the read pointer of all entries in the CAM */
        for (i = 0; i < LL_MAX_CONNECTION_UNITS; i++)
        {
            ll_driver_write_bzdma_rptr_to_cam(i, 0);
        }
#endif
    }

    /*---------------------------------*/
    /* clear and initialize interrupts */
    /*---------------------------------*/
    /* clear rx threshold contriol interrupt and init relative register */
    RD_LE_REG(LE_REG_STATUS_RX_THRES_STATUS);
    ll_reg.value = 0;
    /* comment by Austin, FW expects to process Rx-th per-packet.
       So change the threshold from 6 to 1.*/
    ll_reg.status_rx_thres_ctrl.word_cnt_thres = 1; /*original: LE_ALLOW_RX_WORD_CNT_THRESHOLD;*/
    ll_reg.status_rx_thres_ctrl.thres_en = 1;
    WR_LE_REG(LE_REG_STATUS_RX_THRES_CTRL, ll_reg.value);

    /* clear tx acked threshold contriol interrupt and init relative register */
    ll_reg.value = RD_LE_REG(LE_REG_STATUS_TX_THR_STATUS);
    ll_reg.status_tx_thres.acked_tx_thr = LE_ALLOW_TX_WORD_CNT_THRESHOLD;
    WR_LE_REG(LE_REG_STATUS_TX_THR_STATUS, ll_reg.value);

#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_INFO_
    if (ll_manager.support_ll_privacy)
    {
        /* received scan_req into rx fifo */
        UPDATE_LE_REG(LE_REG_RX_PKT_ERR_CTRL, BIT7, BIT7);
    }
#endif

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
    // by store/restore
    if(dlps_flow == FALSE)
#endif
    {
        /* dape test for LE slave + inquiry */
        ll_reg.value = RD_LE_REG(LE_REG_CONN_UPD_ENTRY);
#ifndef _DAPE_TEST_NEW_HW_LE_CONN_STATE_HIGHER_THAN_LEGACY_ACL
        ll_reg.conn_upd_entry.high_pri_as_no_crc_ok = TRUE;
#else
        ll_reg.conn_upd_entry.high_pri_as_no_crc_ok = FALSE;
#endif
        /* block_legacy should be set to TRUE when doing legacy inquiry & paging */
        ll_reg.conn_upd_entry.blk_legacy_one_slot = FALSE;
#ifdef _DAPE_TEST_NEW_HW_BLOCK_ACL_FOR_LE
        ll_reg.conn_upd_entry.blk_legacy_one_slot = TRUE;
#endif
        ll_reg.conn_upd_entry.confl_avoid = TRUE;
        WR_LE_REG(LE_REG_CONN_UPD_ENTRY, ll_reg.value);
    }

    /* clear ce end interrupt (todo: clean stack) */
    RD_LE_REG(LE_REG_STATUS_CE_END_EVENT);

    /* clear ce start interrupt */
    RD_LE_REG(LE_REG_STATUS_CE_BEGIN_STATUS);

    /* clear exit connection state interrupt */
    RD_LE_REG(LE_REG_STATUS_CONN_STATUS);

    /* clear exit scanning state interrupt, trx packet done interrupts,
       connection update, channel map update and hit connected adv pdu
       interrupts */
    WR_LE_REG(LE_REG_INT_MISR, (BIT2 | BIT3 | BIT7 | BIT11 |
                                BIT12 | BIT13 | BIT14 | BIT15));

    /* clear exit advertising state interrupt */
    RD_LE_REG(LE_REG_STATUS_ADV_STATUS);

    /* set IMR */
    WR_LE_REG(LE_REG_INT_IMR, ll_manager.int_imr);
#ifdef _DAPE_EN_8821_MP_LE_SCAN_INTR
    WR_LE_REG(LE_REG_EXT_MISR, ll_manager.ext_int_imr);
#endif
#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_MSG_
    LL_LOG_TRACE(BLUE, DAPE_TEST_LOG213, 2, LE_REG_EXT_MISR, RD_LE_REG(LE_REG_EXT_MISR));
#endif

    /* Set LE TX retry number to 1 to fast close CE for flow control. */
//    WR_LE_REG(LE_REG_RX_NAK_NUM, 1);

#ifdef _ENABLE_BLE_LL_FLOW_CONTROL_
    ll_driver_set_le_flow_stop(FALSE);
#endif
}

/**************************************************************************
 * Function     : ll_init
 *
 * Description  : This function is used to initiate relative settings for LL
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
void ll_init(void)
{
    ll_fw_init();

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
    ll_hw_init(FALSE);
#else
    ll_hw_init();
#endif

    /* init access address index (enable it after random gen) */
    ll_fw_init_access_address_index();

#ifdef LE_HW_TEST
    ll_test_main_function();
#endif
}

/**************************************************************************
 * Function     : ll_supervision_timeout_callback
 *
 * Description  : This function is a callback function when supervision timer
 *                is expired temporarity
 *
 * Parameters   : timer_handle: the handle of supervision timer
 *                index: the argument of callback function
 *
 * Returns      : None
 *
 *************************************************************************/
void ll_supervision_timeout_callback(TimerHandle_t timer_handle)
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
    if ((!phandle->connected) || (phandle->sup_timer != timer_handle))
    {
        OS_DELETE_TIMER(&timer_handle);
        MINT_OS_EXIT_CRITICAL();
        return;
    }

#ifdef USE_FREERTOS
    OS_STOP_TIMER(phandle->sup_timer, NULL);
    OS_STOP_TIMER(phandle->llc_timer, NULL);
#else
    /* free software timer */
    if (phandle->sup_timer != NULL)
    {
        OS_DELETE_TIMER(&phandle->sup_timer);
    }

    /* free llc timer */
    if (phandle->llc_timer != NULL)
    {
        OS_DELETE_TIMER(&phandle->llc_timer);
    }
#endif

    phandle->kill_link_reason = LL_KILL_REASON_SUPERVISION_TIMEOUT;

    /* kill the connection */
    ll_driver_kill_connection(conn_entry);

    MINT_OS_EXIT_CRITICAL();

    LL_LOG_TRACE(RED, LE_MSG_SUPERVISION_TIMEOUT, 8,
                        conn_entry, phandle->supervision_to,
                        phandle->ce_count_rx_any_pkt,
                        phandle->ce_count_no_crc_ok,
                        phandle->ce_count_restart_sup,
                        phandle->conn_counter,
                        phandle->ce_count_no_any_pkt_received,
                        phandle->ce_count_crc_ok);

#ifdef _DAPE_TEST_CHK_LE_MASTER_LEGACY_SLV_LE_DISCONNECT
    SET_BT_GPIO_OUTPUT_HIGH(0);
    SET_BT_GPIO_OUTPUT_LOW(0);
#endif
}


/**************************************************************************
 * Function     : ll_handle_rxfifo_in_task
 *
 * Description  : This function is used to handle rx fifo in the task.
 *                It can fire the BZDMA to move data from acl rx fifo to
 *                dedicated memory of SRAM.
 *
 * Parameters   : param: the argument of rx task
 *
 * Returns      : None
 *
 *************************************************************************/
void ll_handle_rxfifo_in_task(void *param)
{
    BZDMA_RX_DESC_SEGMENT rxdesc;
    LL_RX_TASK_PARA_S *prx_task_param = (LL_RX_TASK_PARA_S *)param;
    UINT16 fifo_byte_cnt;
    UINT8 type;
    UINT8 conn_entry;
    UINT16 ce_counter = 0;
    UINT16 dma_byte_cnt;
    UINT16 incomplete_len = 0;

    if (prx_task_param->fifo_word_cnt == 0)
    {
        return;
    }

    fifo_byte_cnt = prx_task_param->fifo_word_cnt << 1;
    type = prx_task_param->sub_type;
    conn_entry = prx_task_param->conn_entry;

    switch (type)
    {
    case LL_HANDLE_RECD_ADV_PACKET_TYPE:
        {
            /* do not know why rx threshold interrupt is invalid to
               pop adv packet in rx fifo, but we assume only scan can receive
               more packets in the fifo */

            /* move data from rxfifo to SRAM via BZDMA */

            while (fifo_byte_cnt)
            {
                /* decide the length for fragment */
                if (fifo_byte_cnt > (LL_HW_RXBUFF_SIZE - incomplete_len))
                {
                    dma_byte_cnt = LL_HW_RXBUFF_SIZE - incomplete_len;
                }
                else
                {
                    dma_byte_cnt = fifo_byte_cnt;
                }

                /* remainder length */
                fifo_byte_cnt -= dma_byte_cnt;

                if (incomplete_len > 0)
                {
                    /* copy incomplete data from the tail to the head of
                       local buffer */
                    memcpy(ll_hw_rxfifo_buf,
                       &ll_hw_rxfifo_buf[LL_HW_RXBUFF_SIZE - incomplete_len],
                       incomplete_len);
                }

                /* fire bzdma to copy data from acl rx fifo to memory */
                rxdesc.addr = (UINT32)ll_hw_rxfifo_buf + incomplete_len;
                rxdesc.len = dma_byte_cnt;
                rxdesc.flush = FALSE;
                bzdma_send_burst_rxcmd_and_wait_complete(&rxdesc, 1,
                            BZDMA_RX_PID_ACL, 0, TRUE);

                dma_byte_cnt += incomplete_len;

                if (fifo_byte_cnt > 0)
                {
                    /* this mean we need fragment it */

                    incomplete_len =
                            ll_pre_parser_adv_channel_pdu(ll_hw_rxfifo_buf,
                                                        dma_byte_cnt);
                    dma_byte_cnt -= incomplete_len;
                }
                else
                {
                    incomplete_len = 0;
                }

                ll_parser_adv_channel_pdu(ll_hw_rxfifo_buf, dma_byte_cnt);
            }
        }
        break;

    case LL_HANDLE_RECD_DATA_PACKET_TYPE:
        {
            LL_CONN_HANDLE_UNIT *chu = &ll_manager.conn_unit.handle[conn_entry];

            /* move data from rxfifo to SRAM via BZDMA */
            rxdesc.addr = (UINT32)ll_hw_rxfifo_buf;
            rxdesc.len = fifo_byte_cnt;
            rxdesc.flush = FALSE;
            bzdma_send_burst_rxcmd_and_wait_complete(&rxdesc, 1,
                    BZDMA_RX_PID_ACL, 0, TRUE);

            ce_counter = chu->conn_counter;
            if (chu->rx_pkt_mic_err)
            {
                chu->kill_link_reason = LL_KILL_REASON_MIC_ERROR;
                type = LL_HANDLE_RECD_DATA_PACKET_MIC_ERROR_TYPE;

                /* need to kill le connection */
                ll_driver_kill_connection(conn_entry);

                break;
            }

#ifdef _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_
            if (chu->llc.delay_data_len_updt
#ifdef _BT4_2_DLE_CHECK_REMOTE_FEATURE_
                    && chu->support_dle
#endif
                    )
            {
                chu->llc.delay_data_len_updt = FALSE;
                ll_fw_pend_data_length_update_procedure(chu);
            }
#endif
        }
        ll_parser_data_channel_pdu(ll_hw_rxfifo_buf, fifo_byte_cnt, conn_entry);
        break;

    default:
        BB_read_baseband_RX_FIFO_and_flush(fifo_byte_cnt, TRUE);
        break;
    }

    LL_LOG_TRACE(WHITE, LE_MSG_HANDLE_RX_PKT_IN_TASK, 6,
                 type, prx_task_param->int_src, fifo_byte_cnt, conn_entry, ce_counter,
                 BB_read_native_clock());
}

void ll_start_timer(UINT8 conn_entry, UINT8 timer_type)
{
    LL_CONN_HANDLE_UNIT *phandle;
    UINT32 argument;
    UINT16 timeout;

    phandle = &ll_manager.conn_unit.handle[conn_entry];

    if (!phandle->connected)
    {
        return;
    }

    argument = conn_entry | (timer_type << 8);

    if (timer_type == LL_TIMER_TYPE_SUPERVISION)
    {
        timeout = phandle->supervision_to;

#ifndef USE_FREERTOS
        if (phandle->sup_timer != NULL)
        {
            /* stop the timer if we have the body of supervision timer */
            OS_DELETE_TIMER(&phandle->sup_timer);
        }
#endif

        /* record some parameters for help debug */
        phandle->ce_count_no_crc_ok = 0;
        phandle->ce_count_rx_any_pkt = 0;
        phandle->ce_count_crc_ok = 0;
        phandle->ce_count_no_any_pkt_received = 0;

        phandle->ce_count_restart_sup = phandle->conn_counter;

#ifndef USE_FREERTOS
        /* create a supervision timer and install the callback function */
        OS_CREATE_TIMER(ONESHOT_TIMER, &phandle->sup_timer,
                ll_supervision_timeout_callback, (void *)((UINT32)argument), 0);
#endif
        OS_START_TIMER(phandle->sup_timer, timeout * 10);
    }
    else
    {
#ifndef USE_FREERTOS
        if (phandle->llc_timer != NULL)
        {
            /* stop the timer if we have the body of supervision timer */
            OS_DELETE_TIMER(&phandle->llc_timer);
        }

        /* create a supervision timer and install the callback function */
        OS_CREATE_TIMER(ONESHOT_TIMER, &phandle->llc_timer,
                llc_timeout_callback, (void *)((UINT32)argument), 0);
#endif

        switch (timer_type)
        {
            case LL_TIMER_TYPE_TERMINATE:
            timeout = phandle->supervision_to;
                break;

#ifdef _BT4_1_LE_PING_FEATURE_SUPPORT_
            case LL_TIMER_TYPE_PING:
                if (phandle->auth_payload_timeout > LL_LLC_PROCEDURE_RESPONSE_TIMEOUT)
                {
                    timeout = 0;
                }
                else
                {
                    timeout = LL_LLC_PROCEDURE_RESPONSE_TIMEOUT - phandle->auth_payload_timeout;
                }
                break;
#endif

            default:
            timeout = LL_LLC_PROCEDURE_RESPONSE_TIMEOUT;
                break;
        }

        OS_START_TIMER(phandle->llc_timer, timeout * 10);
    }
}

/**************************************************************************
 * Function     : ll_handle_acl_disconnect
 *
 * Description  : This function is used to disconnect LE acl link
 *
 * Parameters   : conn_handle: the connection handle
 *                reason: the reason code why disconnect
 *                from_host: initaitor is from the host
 *
 * Returns      : BT_FW_ERROR or BT_FW_SUCCESS
 *
 *************************************************************************/
UINT8 ll_handle_acl_disconnect(UINT16 conn_handle, UCHAR reason, UCHAR from_host)
{
    UINT8 conn_entry;
    UINT8 result = BT_FW_ERROR;

    if ((conn_handle < LL_HCI_MIN_CONN_HANDLE) ||
            (conn_handle > LL_HCI_MAX_CONN_HANDLE))
    {
        return BT_FW_ERROR;
    }

	DEF_CRITICAL_SECTION_STORAGE;
    MINT_OS_ENTER_CRITICAL();
    conn_entry = conn_handle - LL_HCI_MIN_CONN_HANDLE;

    if (ll_manager.conn_unit.handle[conn_entry].connected &&
            (ll_manager.conn_unit.bmActiveHandle & (1 << conn_entry)))
    {
        ll_manager.conn_unit.handle[conn_entry].llc.pend_terminate_ind = TRUE;
        ll_manager.conn_unit.handle[conn_entry].llc.loc_terminate_reason = reason;
        if (from_host)
        {
            ll_manager.conn_unit.handle[conn_entry].llc.loc_term_from_host = TRUE;
        }
        else
        {
            ll_manager.conn_unit.handle[conn_entry].llc.loc_term_from_host = FALSE;
        }
        ll_driver_disconnect_connection(conn_entry, reason);

        result = BT_FW_SUCCESS;
    }
    MINT_OS_EXIT_CRITICAL();

    LL_LOG_TRACE(RED, LE_MSG_ACL_DISCONNECT_CMD, 3,
                        conn_handle, conn_entry, reason);

    return result;
}

/**************************************************************************
 * Function     : ll_append_acl_rx_pkt_to_hci_list
 *
 * Description  : This function is used to append LE ACL rx packet to
 *                HCI rx packet list (LL to HCI)
 *
 * Parameters   : acl_pkt: the pointer of LE acl rx packet
 *
 * Returns      : None
 *
 *************************************************************************/
void ll_append_acl_rx_pkt_to_hci_list(LL_HCI_ACL_DATA_PKT *acl_pkt)
{
    LE_ACL_PKT_LIST_MANAGE *pacl_pkt_list;

    if (acl_pkt == NULL)
    {
        return;
    }

    DEF_CRITICAL_SECTION_STORAGE;

    MINT_OS_ENTER_CRITICAL();

    pacl_pkt_list = &ll_manager.conn_unit.l2h_acl_pkt_list;

    if (pacl_pkt_list->pDataHead == NULL)
    {
        pacl_pkt_list->pDataHead = acl_pkt;
    }
    else
    {
        pacl_pkt_list->pDataTail->next = acl_pkt;
    }
    pacl_pkt_list->pDataTail = acl_pkt;
    pacl_pkt_list->pktcnt++;

    MINT_OS_EXIT_CRITICAL();
}

/**************************************************************************
 * Function     : ll_append_acl_rx_pkt_list_to_hci_list
 *
 * Description  : This function is used to append LE ACL rx packet list to
 *                HCI rx packet list (LL to HCI)
 *
 * Parameters   : acl_rx_list: the pointer of LE acl rx packet list
 *
 * Returns      : None
 *
 *************************************************************************/
void ll_append_acl_rx_pkt_list_to_hci_list(LE_ACL_PKT_LIST_MANAGE *acl_rx_list)
{
    LE_ACL_PKT_LIST_MANAGE *pacl_pkt_list;

    if (acl_rx_list->pktcnt == 0)
    {
        return;
    }

    DEF_CRITICAL_SECTION_STORAGE;

    MINT_OS_ENTER_CRITICAL();

    pacl_pkt_list = &ll_manager.conn_unit.l2h_acl_pkt_list;

    if (pacl_pkt_list->pDataHead == NULL)
    {
        pacl_pkt_list->pDataHead = acl_rx_list->pDataHead;
    }
    else
    {
        pacl_pkt_list->pDataTail->next = acl_rx_list->pDataHead;
    }
    pacl_pkt_list->pDataTail = acl_rx_list->pDataTail;
    pacl_pkt_list->pktcnt += acl_rx_list->pktcnt;

    MINT_OS_EXIT_CRITICAL();
}

/**************************************************************************
 * Function     : ll_append_acl_tx_pkt_to_pend_list
 *
 * Description  : This function is used to append LE ACL tx packet to
 *                pending list (HCI to LL)
 *
 * Parameters   : acl_pkt: the pointer of LE acl tx packet list
 *                conn_entry: the connection entry
 *
 * Returns      : None
 *
 *************************************************************************/
void ll_append_acl_tx_pkt_to_pend_list(LL_HCI_ACL_DATA_PKT *acl_pkt,
                                       UINT8 conn_entry)
{
    LE_ACL_PKT_LIST_MANAGE *pacl_pkt_list;
    LL_CONN_HANDLE_UNIT *phandle;
    LL_HCI_ACL_DATA_PKT *psrc_head_dbg;
    LL_HCI_ACL_DATA_PKT *psrc_tail_dbg;

    if (acl_pkt == NULL)
    {
        return;
    }

    DEF_CRITICAL_SECTION_STORAGE;

    MINT_OS_ENTER_CRITICAL();

    phandle = &ll_manager.conn_unit.handle[conn_entry];
    pacl_pkt_list = &phandle->tx_pend_data_pkt_list;

    psrc_head_dbg = pacl_pkt_list->pDataHead;
    psrc_tail_dbg = pacl_pkt_list->pDataTail;

    if (pacl_pkt_list->pDataHead == NULL)
    {
        pacl_pkt_list->pDataHead = acl_pkt;
    }
    else
    {
        pacl_pkt_list->pDataTail->next = acl_pkt;
    }
    pacl_pkt_list->pDataTail = acl_pkt;
    pacl_pkt_list->pktcnt++;

    MINT_OS_EXIT_CRITICAL();

#if 0
    LL_LOG_TRACE(WHITE, LE_MSG_INSERT_ACL_TX_PKT_TO_PEND_LIST, 7,
                 conn_entry, acl_pkt, psrc_head_dbg, psrc_tail_dbg,
                 pacl_pkt_list->pDataHead, pacl_pkt_list->pDataTail, pacl_pkt_list->pktcnt);
#endif
}

/**************************************************************************
 * Function     : ll_append_acl_tx_pkt_list_to_list
 *
 * Description  : This function is used to append LE ACL tx packet list to
 *                list (HCI to LL)
 *
 * Parameters   : acl_tx_list: the pointer of LE acl tx packet list
 *                conn_entry: the connection entry
 *                policy: the append policy
 *                list_type: the type of target list
 *
 * Returns      : None
 *
 *************************************************************************/
void ll_append_acl_tx_pkt_list_to_list(LE_ACL_PKT_LIST_MANAGE *acl_tx_list,
                                       UINT8 conn_entry, UINT8 policy,
                                       UINT8 list_type)
{
    LE_ACL_PKT_LIST_MANAGE *pacl_pkt_list;
    LL_CONN_HANDLE_UNIT *phandle;
    LL_HCI_ACL_DATA_PKT *psrc_head_dbg;
    LL_HCI_ACL_DATA_PKT *psrc_tail_dbg;

#if 0
    LL_LOG_TRACE(WHITE, LE_MSG_INSERT_ACL_TX_PKT_LIST_TO_LIST_PARAM, 6,
                 conn_entry, policy, list_type,
                 acl_tx_list->pDataHead, acl_tx_list->pDataTail, acl_tx_list->pktcnt);
#endif

    if (acl_tx_list->pktcnt == 0)
    {
        return;
    }

    DEF_CRITICAL_SECTION_STORAGE;

    MINT_OS_ENTER_CRITICAL();

    phandle = &ll_manager.conn_unit.handle[conn_entry];

    if (list_type == LL_LINKED_LIST_TYPE_TX_PEND)
    {
        pacl_pkt_list = &phandle->tx_pend_data_pkt_list;
    }
#ifndef _SCHEDULE_BLE_MISC_PACKET_
    else if (list_type == LL_LINKED_LIST_TYPE_TX_SCHED)
    {
        pacl_pkt_list = &phandle->tx_sched_data_pkt_list;
    }
#endif
    else
    {
        pacl_pkt_list = &phandle->h2l_free_pkt_list;
    }

    psrc_head_dbg = pacl_pkt_list->pDataHead;
    psrc_tail_dbg = pacl_pkt_list->pDataTail;

    if (policy == LL_LINKED_LIST_INSERT_POLICY_HEAD)
    {
        if (pacl_pkt_list->pDataHead == NULL)
        {
            pacl_pkt_list->pDataTail = acl_tx_list->pDataTail;
        }
        else
        {
            acl_tx_list->pDataTail->next = pacl_pkt_list->pDataHead;
        }
        pacl_pkt_list->pDataHead = acl_tx_list->pDataHead;
    }
    else
    {
        if (pacl_pkt_list->pDataHead == NULL)
        {
            pacl_pkt_list->pDataHead = acl_tx_list->pDataHead;
        }
        else
        {
            pacl_pkt_list->pDataTail->next = acl_tx_list->pDataHead;
        }
        pacl_pkt_list->pDataTail = acl_tx_list->pDataTail;
    }

    pacl_pkt_list->pktcnt += acl_tx_list->pktcnt;

    MINT_OS_EXIT_CRITICAL();

#if 0
    LL_LOG_TRACE(WHITE, LE_MSG_INSERT_ACL_TX_PKT_LIST_TO_LIST_OUTPUT, 5,
                 psrc_head_dbg, psrc_tail_dbg, pacl_pkt_list->pDataHead,
                 pacl_pkt_list->pDataTail, pacl_pkt_list->pktcnt);
#endif
}

/**************************************************************************
 * Function     : ll_free_acl_tx_pkts_in_free_list
 *
 * Description  : This function is used to free LE ACL tx packets in a free list
 *
 * Parameters   : conn_entry: the connection entry
 *
 * Returns      : None
 *
 *************************************************************************/
void ll_free_acl_tx_pkts_in_free_list(UINT8 conn_entry)
{
    LL_CONN_HANDLE_UNIT *phandle;
    LL_HCI_ACL_DATA_PKT *ppkt;
    LL_HCI_ACL_DATA_PKT *pnext;
    UINT8 pkt_cnt;

    DEF_CRITICAL_SECTION_STORAGE;

    MINT_OS_ENTER_CRITICAL();

    phandle = &ll_manager.conn_unit.handle[conn_entry];
    ppkt = phandle->h2l_free_pkt_list.pDataHead;
    pkt_cnt = phandle->h2l_free_pkt_list.pktcnt;

    /* init the acl data pdu free list */
    phandle->h2l_free_pkt_list.pDataHead = NULL;
    phandle->h2l_free_pkt_list.pDataTail = NULL;
    phandle->h2l_free_pkt_list.pktcnt = 0;

    MINT_OS_EXIT_CRITICAL();

    while (ppkt != NULL)
    {
        pnext = ppkt->next;
        ppkt->next = NULL;

#ifndef LE_HW_TEST
        if (dma_tx_fifo_pkt_free((void*)ppkt,
                           HCI_TRANSPORT_LE_ACL_DATA_PKT_TYPE) != BT_ERROR_OK)
#else
        if(OS_FREE_BUFFER(rx_table[LE_ACL_DATA_HANDLER_TASK].pool_handle,
                          (void *)ppkt) != BT_ERROR_OK)
#endif
        {
            LL_LOG_TRACE(RED, LE_MSG_FREE_H2L_ACL_DATA_ERROR, 2,
                         conn_entry, (UINT32)ppkt);
        }
        ppkt = pnext;
    }

    phandle->hc_complete_pkts += pkt_cnt;

#ifndef LE_HW_TEST
    /* generate number of complete event to hci host */
    hci_generate_number_of_completed_packets_event();
#endif
}

#if defined(_SCHEDULE_BLE_MISC_PACKET_) && defined(_NEW_BZDMA_FROM_V8_)
/**************************************************************************
 * Function     : ll_get_le_tx_misc_node
 *
 * Description  : This function is used to get a tx misc node from free list
 *
 * Parameters   : None
 *
 * Returns      : Node Id
 *
 *************************************************************************/
UINT8 ll_get_le_tx_misc_node(void)
{
    UINT8 next_id;
    UINT8 cur_head;

    DEF_CRITICAL_SECTION_STORAGE;

    MINT_OS_ENTER_CRITICAL();

    cur_head = le_tx_misc_free_list.MiscHeadId;

    if (cur_head != LL_MISC_ACL_DATA_PKT_MAX_NODES)
    {
        /* pop one node from the head */
        next_id = le_tx_misc_node[cur_head].NextNodeId;
        if (next_id == LL_MISC_ACL_DATA_PKT_MAX_NODES)
        {
            le_tx_misc_free_list.MiscTailId = LL_MISC_ACL_DATA_PKT_MAX_NODES;
        }
        le_tx_misc_free_list.MiscHeadId = next_id;
        le_tx_misc_free_list.pktcnt--;
        le_tx_misc_node[cur_head].NextNodeId = LL_MISC_ACL_DATA_PKT_MAX_NODES;
    }

    MINT_OS_EXIT_CRITICAL();

    if (cur_head == LL_MISC_ACL_DATA_PKT_MAX_NODES)
    {
        RT_BT_LOG(RED, MSG_BEE_NO_FREE_NODE, 0, 0);
    }

    return cur_head;
}

/**************************************************************************
 * Function     : ll_free_le_tx_misc_node
 *
 * Description  : This function is used to free a tx misc node back free list
 *
 * Parameters   : node_id : assigned node id
 *
 * Returns      : Node
 *
 *************************************************************************/
void ll_free_le_tx_misc_node(UINT8 node_id)
{
    DEF_CRITICAL_SECTION_STORAGE;

    MINT_OS_ENTER_CRITICAL();

    if ((node_id < LL_MISC_ACL_DATA_PKT_MAX_NODES) &&
        (le_tx_misc_free_list.pktcnt < LL_MISC_ACL_DATA_PKT_MAX_NODES))
    {
        if (le_tx_misc_free_list.MiscHeadId == LL_MISC_ACL_DATA_PKT_MAX_NODES)
        {
            le_tx_misc_free_list.MiscHeadId = node_id;
        }
        else
        {
            le_tx_misc_node[le_tx_misc_free_list.MiscTailId].NextNodeId = node_id;
        }

        le_tx_misc_free_list.MiscTailId = node_id;
        le_tx_misc_node[node_id].NextNodeId = LL_MISC_ACL_DATA_PKT_MAX_NODES;
        le_tx_misc_free_list.pktcnt++;
    }

    MINT_OS_EXIT_CRITICAL();
}

#ifdef _SUPPORT_LE_HCI_TX_PKT_FRAGMENT_REASSEMBLE_FUNC_
void ll_hci_acl_data_pkt_list_reset(LE_ACL_PKT_LIST_MANAGE *list)
{
    list->pDataHead = NULL;
    list->pDataTail = NULL;
    list->pktcnt = 0;
}

void ll_hci_acl_data_pkt_list_move(LE_ACL_PKT_LIST_MANAGE *dest, LE_ACL_PKT_LIST_MANAGE *src)
{
    dest->pDataHead = src->pDataHead;
    dest->pDataTail = src->pDataTail;
    dest->pktcnt = src->pktcnt;
    ll_hci_acl_data_pkt_list_reset(src);
}

void ll_hci_acl_data_pkt_list_add_tail(LE_ACL_PKT_LIST_MANAGE *list,
        LL_HCI_ACL_DATA_PKT *pkt)
{
    pkt->next = NULL;
    if (list->pDataHead == NULL)
    {
        list->pDataHead = pkt;
    }
    else
    {
        list->pDataTail->next = pkt;
    }

    list->pDataTail = pkt;
    ++list->pktcnt;
}

LL_HCI_ACL_DATA_PKT *ll_hci_acl_data_pkt_list_pop(LE_ACL_PKT_LIST_MANAGE *list)
{
    LL_HCI_ACL_DATA_PKT *pkt = list->pDataHead;
    if (pkt != NULL)
    {
        list->pDataHead = pkt->next;
        pkt->next = NULL;
        --list->pktcnt;
        if (list->pktcnt == 0)
        {
            list->pDataTail = NULL;
        }
    }
    return pkt;
}

void ll_restructure_acl_tx_misc_pkts_to_list(LE_ACL_PKT_LIST_MANAGE *list, UINT8 head_id)
{
    ll_hci_acl_data_pkt_list_reset(list);

    UINT8 cur = head_id;
    while (cur != LL_MISC_ACL_DATA_PKT_MAX_NODES)
    {
        LL_MISC_ACL_DATA_PKT_NODE *node = &le_tx_misc_node[cur];
        if (node->Src == LL_MISC_ACL_DATA_PKT_NODE_TYPE_HCI)
        {
            UINT8 i;
            for (i = 0; i < node->FragCount; i++)
            {
                LL_HCI_ACL_DATA_PKT *pkt = node->frag_info[i].pkt;
                if (list->pDataTail == pkt)
                {
                    continue;
                }
                ll_hci_acl_data_pkt_list_add_tail(list, pkt);
            }
        }

        ll_free_le_tx_misc_node(cur);
        cur = node->NextNodeId;
    }
}

void ll_restructure_free_acl_tx_misc_pkts_to_list(LE_ACL_PKT_LIST_MANAGE *list, UINT8 head_id)
{
    ll_hci_acl_data_pkt_list_reset(list);

    UINT8 cur = head_id;
    while (cur != LL_MISC_ACL_DATA_PKT_MAX_NODES)
    {
        LL_MISC_ACL_DATA_PKT_NODE *node = &le_tx_misc_node[cur];
        if (node->Src == LL_MISC_ACL_DATA_PKT_NODE_TYPE_HCI)
        {
            UINT8 i;
            for (i = 0; i < node->FragCount; i++)
            {
                LL_HCI_ACL_DATA_PKT *pkt = node->frag_info[i].pkt;
                pkt->bytes_acked += node->frag_info[i].len;
                if (pkt->bytes_acked < pkt->acl_data_total_length)
                {
                    continue;
                }
                ll_hci_acl_data_pkt_list_add_tail(list, pkt);
            }
        }

        ll_free_le_tx_misc_node(cur);
        cur = node->NextNodeId;
    }
}
#else
void ll_restructure_acl_tx_misc_pkts_to_list(LE_ACL_PKT_LIST_MANAGE *plist, UINT8 node_id)
{
    LL_HCI_ACL_DATA_PKT *phci_acl_pkt;
    LL_MISC_ACL_DATA_PKT_NODE *pNode;
    UINT8 next;
    UINT8 head_id = node_id;
    UINT8 i;

    /* init local packet list managerment */
    plist->pDataHead = NULL;
    plist->pDataTail = NULL;
    plist->pktcnt = 0;

    while (head_id != LL_MISC_ACL_DATA_PKT_MAX_NODES)
    {
        pNode = &le_tx_misc_node[head_id];
        next = pNode->NextNodeId;

        if (pNode->Src == LL_MISC_ACL_DATA_PKT_NODE_TYPE_HCI)
        {
            for (i = 0; i < pNode->FragCount; i++)
            {
                phci_acl_pkt = (LL_HCI_ACL_DATA_PKT *)
                    ((pNode->pFrags[i].start_addr | LL_BUFF_RAM_BASE) - 4);

                phci_acl_pkt->next = NULL;

                if (plist->pDataHead == NULL)
                {
                    plist->pDataHead = phci_acl_pkt;
                }
                else
                {
                    plist->pDataTail->next = phci_acl_pkt;
                }

                plist->pDataTail = phci_acl_pkt;
                plist->pktcnt++;
            }
        }

        ll_free_le_tx_misc_node(head_id);

        head_id = next;
    }
}
#endif /* _SUPPORT_LE_HCI_TX_PKT_FRAGMENT_REASSEMBLE_FUNC_ */

/**************************************************************************
 * Function     : ll_append_acl_tx_misc_pkt_to_pend_list
 *
 * Description  : This function is used to append LE ACL tx MISC packet to
 *                pending list (HCI to LL)
 *
 * Parameters   : Node_Id : the id of le misc node
 *                conn_entry : the connection entry
 *
 * Returns      : None
 *
 *************************************************************************/
void ll_append_acl_tx_misc_pkt_to_pend_list(UINT8 Node_Id, UINT8 conn_entry)
{
    LE_ACL_MISC_PKT_LIST_MANAGE *plist;

    DEF_CRITICAL_SECTION_STORAGE;

    if (Node_Id == LL_MISC_ACL_DATA_PKT_MAX_NODES)
    {
        return;
    }

    plist = &ll_manager.conn_unit.handle[conn_entry].tx_pend_misc_pkt_list;

    /* add node to the tail of pending list */
    MINT_OS_ENTER_CRITICAL();
    if (plist->MiscHeadId == LL_MISC_ACL_DATA_PKT_MAX_NODES)
    {
        plist->MiscHeadId = Node_Id;
    }
    else
    {
        le_tx_misc_node[plist->MiscTailId].NextNodeId = Node_Id;
    }
    plist->MiscTailId = Node_Id;
    plist->pktcnt++;
    MINT_OS_EXIT_CRITICAL();
}

/**************************************************************************
 * Function     : ll_append_acl_tx_misc_pkt_list_to_list
 *
 * Description  : This function is used to append LE ACL tx misc packet list to
 *                list (HCI to LL)
 *
 * Parameters   : acl_tx_list: the pointer of LE acl tx misc packet list
 *                conn_entry: the connection entry
 *                policy: the append policy
 *                list_type: the type of target list
 *
 * Returns      : None
 *
 *************************************************************************/
void ll_append_acl_tx_misc_pkt_list_to_list(LE_ACL_MISC_PKT_LIST_MANAGE *acl_tx_list,
                                       UINT8 conn_entry, UINT8 policy,
                                       UINT8 list_type)
{
    LE_ACL_MISC_PKT_LIST_MANAGE *plist;
    LL_CONN_HANDLE_UNIT *phandle;

    if (acl_tx_list->pktcnt == 0)
    {
        return;
    }

    phandle = &ll_manager.conn_unit.handle[conn_entry];

    if (list_type == LL_LINKED_LIST_TYPE_TX_PEND)
    {
        plist = &phandle->tx_pend_misc_pkt_list;
    }
#ifndef USE_NEW_LE_SCHEDULER
    else if (list_type == LL_LINKED_LIST_TYPE_TX_SCHED)
    {
        plist = &phandle->tx_sched_misc_pkt_list;
    }
#endif
    else
    {
        plist = &phandle->h2l_free_misc_pkt_list;
    }

    DEF_CRITICAL_SECTION_STORAGE;

    MINT_OS_ENTER_CRITICAL();

    if (policy == LL_LINKED_LIST_INSERT_POLICY_HEAD)
    {
        /* AddFirst */
        if (plist->MiscHeadId == LL_MISC_ACL_DATA_PKT_MAX_NODES)
        {
            /* target list is empty */
            plist->MiscTailId = acl_tx_list->MiscTailId;
        }
        else
        {
            le_tx_misc_node[acl_tx_list->MiscTailId].NextNodeId = plist->MiscHeadId;
        }
        plist->MiscHeadId = acl_tx_list->MiscHeadId;
    }
    else
    {
        /* AddLast */
        if (plist->MiscHeadId == LL_MISC_ACL_DATA_PKT_MAX_NODES)
        {
            /* target list is empty */
            plist->MiscHeadId = acl_tx_list->MiscHeadId;
        }
        else
        {
            le_tx_misc_node[plist->MiscTailId].NextNodeId = acl_tx_list->MiscHeadId;
        }
        plist->MiscTailId = acl_tx_list->MiscTailId;
    }

    plist->pktcnt += acl_tx_list->pktcnt;

    MINT_OS_EXIT_CRITICAL();
}

/**************************************************************************
 * Function     : ll_free_acl_tx_misc_pkts_in_free_list
 *
 * Description  : This function is used to free LE ACL tx packets in a free list
 *
 * Parameters   : conn_entry: the connection entry
 *
 * Returns      : None
 *
 *************************************************************************/
void ll_free_acl_tx_misc_pkts_in_free_list(UINT8 conn_entry)
{
    LL_CONN_HANDLE_UNIT *phandle;
    LE_ACL_PKT_LIST_MANAGE hci_acl_pkt_list;
    UINT8 head_id;

    DEF_CRITICAL_SECTION_STORAGE;

    phandle = &ll_manager.conn_unit.handle[conn_entry];

    MINT_OS_ENTER_CRITICAL();
    head_id = phandle->h2l_free_misc_pkt_list.MiscHeadId;
    phandle->h2l_free_misc_pkt_list.DWord = LL_MISC_ACL_PKT_LIST_RESET_VALUE;
    MINT_OS_EXIT_CRITICAL();

#ifdef _SUPPORT_LE_HCI_TX_PKT_FRAGMENT_REASSEMBLE_FUNC_
    ll_restructure_free_acl_tx_misc_pkts_to_list(&hci_acl_pkt_list, head_id);
#else
    ll_restructure_acl_tx_misc_pkts_to_list(&hci_acl_pkt_list, head_id);
#endif

    if (hci_acl_pkt_list.pktcnt > 0)
    {
        ll_append_acl_tx_pkt_list_to_list(&hci_acl_pkt_list, conn_entry,
                                          LL_LINKED_LIST_INSERT_POLICY_TAIL,
                                          LL_LINKED_LIST_TYPE_TX_FREE);
    }

    if (phandle->h2l_free_pkt_list.pktcnt > 0)
    {
        ll_free_acl_tx_pkts_in_free_list(conn_entry);
    }
}


/**************************************************************************
 * Function     : ll_reassemble_misc_tx_pkt_from_acl_pend_list
 *
 * Description  : This function is used to reassemble LE ACL misc tx packets
 *                from pending acl packet list
 *
 * Parameters   : conn_entry : the connection entry
 *                max_tx_size : maximum tx packet size
 *
 * Returns      : None
 *
 *************************************************************************/
#ifdef _SUPPORT_LE_HCI_TX_PKT_FRAGMENT_REASSEMBLE_FUNC_
void ll_reassemble_misc_tx_pkt_from_acl_pend_list(UINT8 conn_entry,
        UINT8 max_tx_size)
{
    LE_ACL_PKT_LIST_MANAGE pkt_list;
    LL_CONN_HANDLE_UNIT *chu = &ll_manager.conn_unit.handle[conn_entry];
    DEF_CRITICAL_SECTION_STORAGE;

    MINT_OS_ENTER_CRITICAL();
    ll_hci_acl_data_pkt_list_move(&pkt_list, &chu->tx_pend_data_pkt_list);
    chu->tx_pend_data_pkt_list.in_procedure = FALSE;
    MINT_OS_EXIT_CRITICAL();

    UINT8 is_1st_aclu_frag = FALSE;
    LL_HCI_ACL_DATA_PKT *pkt = pkt_list.pDataHead;
    while (pkt != NULL)
    {
        UINT8 node_id = ll_get_le_tx_misc_node();
        if (node_id >= LL_MISC_ACL_DATA_PKT_MAX_NODES)
        {
            /* no free misc node, but the list is not empty !!
             * we must insert back original pneding list (austin)
             */
            ll_append_acl_tx_pkt_list_to_list(&pkt_list, conn_entry,
                    LL_LINKED_LIST_INSERT_POLICY_HEAD,
                    LL_LINKED_LIST_TYPE_TX_PEND);
            return;
        }

        LL_MISC_ACL_DATA_PKT_NODE *node = &le_tx_misc_node[node_id];
        UINT8 index;
        UINT16 total_len = 0;
        if (pkt->packet_boundary_flag == L_CH_L2CAP_NON_FLUSH
                && pkt->frag_offs == 0)
        {
            is_1st_aclu_frag = TRUE;
        }
#ifdef _NEW_BZDMA_FROM_V8_PROGRAMMABLE_CONFIG_SETTING_
        for (index = 0; index < bzdma_supported_le_max_frag_num; ++index)
#else
        for (index = 0; index < LL_MISC_ACL_DATA_PKT_MAX_FRAGS; ++index)
#endif
        {
            LL_HCI_ACL_DATA_FRAGMENT_INFO *fi = &node->frag_info[index];
            UINT16 pkt_bytes_to_send = pkt->acl_data_total_length - pkt->frag_offs;

            /* Fragment only if bytes of a packet to send > max_tx_size.
             * In other cases (e.g. the assembly of two packets slightly
             * exceeds max_tx_size), we favor assembly only to avoid the
             * overhead of fragmentation.
             */
            UINT8 frag_len;
            if (pkt_bytes_to_send > max_tx_size) /* Packets need fragmentation. */
            {
                frag_len = max_tx_size;
            }
            else
            {
                frag_len = pkt_bytes_to_send;
            }

            total_len += frag_len;
            if (total_len > max_tx_size)
            {
                break;
            }

            fi->pkt = pkt;
            fi->offs = pkt->frag_offs;
            fi->len = frag_len;

            node->pFrags[index].DWord = 0;
            node->pFrags[index].start_addr = (UINT32) &pkt->hci_acl_data_pkt[pkt->frag_offs];
            node->pFrags[index].len = frag_len;
            node->pFrags[index].isLast = FALSE;
            pkt->frag_offs += frag_len;

            if (pkt_bytes_to_send > max_tx_size) /* Packets need fragmentation. */
            {
                if (fi->offs + frag_len < pkt->acl_data_total_length)
                {
                    continue;
                }
            }

            pkt = pkt->next;
            ll_hci_acl_data_pkt_list_pop(&pkt_list);
            if (pkt == NULL
                    || pkt->packet_boundary_flag == L_CH_L2CAP_NON_FLUSH)
            {
                ++index;
                break;
            }
        }

        if (index > 0)
        {
            node->pFrags[index - 1].isLast = TRUE;
            node->is1stFrag = is_1st_aclu_frag;
            node->FragCount = index;
            ll_append_acl_tx_misc_pkt_to_pend_list(node_id, conn_entry);
            is_1st_aclu_frag = FALSE;
        }
        else
        {
            ll_free_le_tx_misc_node(node_id);
        }
    }
}
#else
void ll_reassemble_misc_tx_pkt_from_acl_pend_list(UINT8 conn_entry,  UINT8 max_tx_size)
{
    LL_CONN_HANDLE_UNIT *phandle;
    LL_HCI_ACL_DATA_PKT *ppkt;
    UINT8 is_1st_aclu_frag = FALSE;;
    UINT8 index = 0;
    UINT16 len = 0;
    LL_MISC_ACL_DATA_PKT_NODE *pNode = NULL;
    LL_HCI_ACL_DATA_PKT *ppkt_pre = NULL;
    LL_HCI_ACL_DATA_PKT *ppkt_tail;
    UINT8 Node_Id = 0;
    UINT8 pkt_count;

    DEF_CRITICAL_SECTION_STORAGE;

    phandle = &ll_manager.conn_unit.handle[conn_entry];

    MINT_OS_ENTER_CRITICAL();
    ppkt = phandle->tx_pend_data_pkt_list.pDataHead;
    ppkt_tail = phandle->tx_pend_data_pkt_list.pDataTail;
    pkt_count = phandle->tx_pend_data_pkt_list.pktcnt;
    phandle->tx_pend_data_pkt_list.pDataHead = NULL;
    phandle->tx_pend_data_pkt_list.pDataTail = NULL;
    phandle->tx_pend_data_pkt_list.pktcnt = 0;
    phandle->tx_pend_data_pkt_list.in_procedure = FALSE;
    MINT_OS_EXIT_CRITICAL();

    while (ppkt != NULL)
    {
        if (ppkt->packet_boundary_flag == L_CH_L2CAP_NON_FLUSH)
        {
            /* first pkt of h2c le-u pkt */
            if (index > 0)
            {
                /* aggregate previous le-u pkts.
                   because new first le-u pkt is scheduled.  */
                pNode->pFrags[index - 1].isLast = TRUE;
                pNode->is1stFrag = is_1st_aclu_frag ? TRUE : FALSE;
                pNode->FragCount = index;
                ll_append_acl_tx_misc_pkt_to_pend_list(Node_Id, conn_entry);
                index = 0;
                len = 0;
            }

            is_1st_aclu_frag = TRUE;
        }

        len += ppkt->acl_data_total_length;

#ifdef _NEW_BZDMA_FROM_V8_PROGRAMMABLE_CONFIG_SETTING_
        if ((len > max_tx_size) || (index == bzdma_supported_le_max_frag_num))
#else
        if ((len > max_tx_size) || (index == LL_MISC_ACL_DATA_PKT_MAX_FRAGS))
#endif
        {
            if ((pNode == NULL) || (index == 0))
            {
                LL_LOG_TRACE(RED, YL_DBG_HEX_4, 4, 0xdead, len, max_tx_size, index);
                return;
            }

            /* aggregate previous le-u pkts.
               because the total packet length or used fragment numbers are
               overflow */
            pNode->pFrags[index - 1].isLast = TRUE;
            pNode->is1stFrag = is_1st_aclu_frag ? TRUE : FALSE;
            pNode->FragCount = index;
            ll_append_acl_tx_misc_pkt_to_pend_list(Node_Id, conn_entry);

            len = ppkt->acl_data_total_length;
            index = 0;
            is_1st_aclu_frag = FALSE;
        }

        if (index == 0)
        {
            Node_Id = ll_get_le_tx_misc_node();

            if (Node_Id == LL_MISC_ACL_DATA_PKT_MAX_NODES)
            {
                LE_ACL_PKT_LIST_MANAGE list_tmp;

                /* no free misc node, but the list is not empty !!
                   we must insert back original pneding list (austin) */

                if (ppkt_pre != NULL)
                {
                    /* cut valid list */
                    ppkt_pre->next = NULL;
                }

                list_tmp.pDataHead = ppkt;
                list_tmp.pDataTail = ppkt_tail;
                list_tmp.pktcnt = pkt_count;
                ll_append_acl_tx_pkt_list_to_list(&list_tmp, conn_entry,
                                              LL_LINKED_LIST_INSERT_POLICY_HEAD,
                                              LL_LINKED_LIST_TYPE_TX_PEND);
                return;
            }

            pNode = &le_tx_misc_node[Node_Id];
        }

        /* fill one fragment for bzdma ble tx command */
        pNode->pFrags[index].DWord = 0;
        pNode->pFrags[index].start_addr = (UINT32)ppkt->hci_acl_data_pkt;
        pNode->pFrags[index].len = ppkt->acl_data_total_length;
        pNode->pFrags[index].isLast = FALSE;

        index++;
        pkt_count--;

        ppkt_pre = ppkt;
        ppkt = ppkt->next;
    }

    if (index > 0)
    {
        /* aggregate previous le-u pkts.
           because the pending data pkt list is empty */
        pNode->pFrags[index - 1].isLast = TRUE;
        pNode->is1stFrag = is_1st_aclu_frag ? TRUE : FALSE;
        pNode->FragCount = index;
        ll_append_acl_tx_misc_pkt_to_pend_list(Node_Id, conn_entry);
    }
}
#endif

/**************************************************************************
 * Function     : ll_reassemble_misc_tx_pkt_from_acl_pend_list
 *
 * Description  : This function is used to reassemble LE ACL misc tx packets
 *                from pending acl packet list
 *
 * Parameters   : conn_entry : the connection entry
 *                max_tx_size : maximum tx packet size
 *
 * Returns      : None
 *
 *************************************************************************/
void ll_reassemble_misc_tx_pkt_from_acl_misc_pend_list(UINT8 conn_entry,  UINT8 max_tx_size)
{
    LL_CONN_HANDLE_UNIT *phandle;
    LE_ACL_PKT_LIST_MANAGE hci_acl_pkt_list;
    UINT8 head_id;
    UINT8 change_misc_pkt_size;

    DEF_CRITICAL_SECTION_STORAGE;

    phandle = &ll_manager.conn_unit.handle[conn_entry];

    MINT_OS_ENTER_CRITICAL();
    head_id = phandle->tx_pend_misc_pkt_list.MiscHeadId;
    change_misc_pkt_size = phandle->data_len_updt.tx_changed;
    if (change_misc_pkt_size)
    {
        if (head_id != LL_MISC_ACL_DATA_PKT_MAX_NODES)
        {
            /* cut original pending list */
            phandle->tx_pend_misc_pkt_list.DWord = LL_MISC_ACL_PKT_LIST_RESET_VALUE;
            change_misc_pkt_size = TRUE;
        }
        phandle->data_len_updt.tx_changed = FALSE;
    }
    MINT_OS_EXIT_CRITICAL();

    if (change_misc_pkt_size == TRUE)
    {
        ll_restructure_acl_tx_misc_pkts_to_list(&hci_acl_pkt_list, head_id);

        if (hci_acl_pkt_list.pktcnt > 0)
        {
            /* it shall be insert the head of tx data pending list because
               these packets are rescheduling */
            ll_append_acl_tx_pkt_list_to_list(&hci_acl_pkt_list, conn_entry,
                                              LL_LINKED_LIST_INSERT_POLICY_HEAD,
                                              LL_LINKED_LIST_TYPE_TX_PEND);
        }
    }

    ll_reassemble_misc_tx_pkt_from_acl_pend_list(conn_entry, max_tx_size);
}
#endif



/**
* Validates the parameters of the ACL packet received from the host,
* and queues it in the baseband if there is a free scheduler entry
* available.
*
* \param pkt Pointer to the ACL packet received from the host.
*
* \return None.
*/
void ll_handle_host_data_pkt(LL_HCI_ACL_DATA_PKT *pkt)
{
    UINT16 connection_handle;
    UCHAR packet_boundry_flag;
    UCHAR broad_cast_flag;
    UCHAR drop = FALSE;
    UCHAR *pkt_buf = NULL;
    LL_CONN_HANDLE_UNIT *handle;
    UINT8 reason = 0;

    /* Extract boundry flag and broadcast flag for the packet */
    packet_boundry_flag = (UCHAR)pkt->packet_boundary_flag;
    broad_cast_flag = (UCHAR)pkt->broadcast_flag;

    /*Extract connection handle from the packet */
    connection_handle = (UINT16) pkt->connection_handle;

    do
    {
        /* Check for BC flag */
        if( (broad_cast_flag != LC_DATA_ACTIVE_FLAG))
        {
            /* LL shall be unicast */
            drop = TRUE;
            reason = 1;
            break;
        }

        /* Check for PB flag */
        if ((packet_boundry_flag != L_CH_L2CAP_NON_FLUSH) &&
                (packet_boundry_flag != L_CH_L2CAP_CONT))
        {
            drop = TRUE;
            reason = 2;
            break;
        }

        /* Get the connection handle */
        handle = ll_fw_search_handle_unit_via_conn_handle(connection_handle);

        if ((handle == NULL) || !handle->connected)
        {
            drop = TRUE;
            reason = 3;
            break;
        }

        /* append le acl pkt to list */
        pkt->next = NULL;
#ifdef _SUPPORT_LE_HCI_TX_PKT_FRAGMENT_REASSEMBLE_FUNC_
        pkt->bytes_acked = 0;
        pkt->frag_offs = 0;
#endif
        ll_append_acl_tx_pkt_to_pend_list(pkt, handle->unit_id);

#if defined(_NEW_BLE_HW_SPEC_FROM_150320_) && defined(_LE_WAKE_FROM_SLAVE_LATENCY_)
        /* if I am slave, notify hw to wakeup from slave latency */
        if (ll_manager.conn_unit.en_wake_from_slave_latency &&
                                            !ll_manager.conn_unit.master)
        {
            ll_driver_wake_from_slave_latency();
        }
#endif
    }
    while (0);

    if (drop == TRUE)
    {
        /* Free the packet buffer and send H\W error event to host */
#ifdef LE_HW_TEST
        OS_FREE_BUFFER(rx_table[LE_ACL_DATA_HANDLER_TASK].pool_handle,
                          (void *)pkt);
#else
        dma_tx_fifo_pkt_free((void *)pkt,
                          HCI_TRANSPORT_LE_ACL_DATA_PKT_TYPE);
#endif

        LL_LOG_TRACE(RED, LE_MSG_HOST_DATA_ERR, 4, reason,
                     packet_boundry_flag, broad_cast_flag, connection_handle);

#ifdef _DAPE_TEST_DONT_SEND_ERR_EVENT_TO_HOST
        if (IS_HARDWARE_ERR_EVENT_TO_HOST_WHEN_HOST_ACL)
#endif
        {
            OS_ALLOC_BUFFER(tx_table[HCI_EVENT_HANDLER_TASK].pool_handle,
                            (void **)(&pkt_buf));

            pkt_buf[0] = 0x10;
            pkt_buf[1] = 0x01;
            pkt_buf[2] = PF_HCI_TRANSPORT_HW_ERROR_WRONG_PARAM_FOR_ACL_DATA;
            hci_td_deliver_event_to_host(pkt_buf);
            //RT_BT_LOG(RED, EVENT_SENT_TO_HCI_TD_TASK_EVENT_TYPE_PARAM_LENGTH,2,pkt_buf[2],pkt_buf[1]);
        }
    }
}

/**************************************************************************
 * Function     : ll_handle_rx_data_pdu
 *
 * Description  : This function is used to generate a LE ACL rx packet after
 *                do a memory copy from a common memory then append this packet
 *                to a ll-to-hci list
 *
 * Parameters   : payload: the pointer of payload from a common memory
 *                len: the copied length
 *                frag1st: the first or continuous packet
 *                entry: the entry id of this connection
 *
 * Returns      : None
 *
 *************************************************************************/
void ll_handle_rx_data_pdu(UINT8 *payload, UINT8 len, UINT8 frag1st, UINT8 entry)
{
    LL_HCI_ACL_DATA_PKT *RxPkt;
    UINT16 conn_handle;

    if (entry > LL_MAX_CONNECTION_UNITS)
    {
        LL_LOG_TRACE(RED, LE_MSG_INVALID_CONN_ENTRY, 1, entry);
#ifdef _ENABLE_BLE_LL_FLOW_CONTROL_
        os_release_one_le_reserved_buffer();
#endif
        return;
    }

    conn_handle = ll_manager.conn_unit.handle[entry].conn_handle;

    if (ll_manager.conn_unit.handle[entry].discard_acl_rx)
    {
        /* fix SEC/MAS/BI-07 and SEC/MAS/BI-09 - different from the test spec.
           this will be the correct handling in the future spec - austin */
        if (ll_manager.conn_unit.handle[entry].encrypt_blk.start_enc_state ==
                                                LLC_START_ENC_STATE_BEGIN)
        {
            if (ll_manager.conn_unit.handle[entry].encrypt_blk.pause_enc_state ==
                                                LLC_PAUSE_ENC_STATE_END)
            {
                UCHAR event_parameter[HCI_ENCRYPTION_KEY_REFRESH_COMPLETE_EVENT_LEN];
                /* expect to do restart procedure */
                /* Send hci encryption key refresh Event */
                event_parameter[0] = CONNECTION_TERMINATED_DUE_TO_MIC_FAILURE;
                event_parameter[1] = conn_handle & 0xFF;
                event_parameter[2] = conn_handle >> 8;
                hci_generate_event(HCI_ENCRYPTION_KEY_REFRESH_COMPLETE_EVENT,
                        event_parameter,
                        HCI_ENCRYPTION_KEY_REFRESH_COMPLETE_EVENT_LEN);
            }
            else
            {
                UCHAR event_parameter[4];
                /* expect to do start procedure */
                event_parameter[0] = CONNECTION_TERMINATED_DUE_TO_MIC_FAILURE;
                event_parameter[1] = conn_handle & 0xFF;
                event_parameter[2] = conn_handle >> 8;
                event_parameter[3] = 0; /* encryption_enabled */
                hci_generate_event(HCI_ENCRYPTION_CHANGE_EVENT,
                        event_parameter, 4);
            }

            /* send ll_terminate pdu to remote device */
            ll_handle_acl_disconnect(conn_handle,
                                    CONNECTION_TERMINATED_DUE_TO_MIC_FAILURE,
                                    FALSE);
        }

        /* discard acl user data during the encryption pause precedure */

#ifdef _ENABLE_BLE_LL_FLOW_CONTROL_
        os_release_one_le_reserved_buffer();
#endif

        return;
    }

#ifdef  _FRAGMENT_BLE_ACLU_RX_DATA_PACKET_
    UINT16 offset = 0;
    UINT16 pkt_len;

    /* start to fragment the acl-u data packet */
    while (len != 0)
    {
        if (len >= LL_POLL_HCI_MAX_RX_ACL_PAYLOAD_SIZE)
        {
            pkt_len = LL_POLL_HCI_MAX_RX_ACL_PAYLOAD_SIZE;
            len -= LL_POLL_HCI_MAX_RX_ACL_PAYLOAD_SIZE;
        }
        else
        {
            pkt_len = len;
            len = 0;
        }

        if(OS_ALLOC_BUFFER(tx_table[LE_ACL_DATA_HANDLER_TASK].pool_handle,
                       (void**)&(RxPkt)) != BT_ERROR_OK)
        {
            /* TODO: show message */
#ifdef _ENABLE_BLE_LL_FLOW_CONTROL_
            os_release_one_le_reserved_buffer();
#endif
            return;
        }

        if ((frag1st == TRUE) && (offset == 0))
        {
            /* only first le-u packet and first fragment is TRUE first hci ble
              acl packet */
            RxPkt->packet_boundary_flag = 0x02;
        }
        else
        {
            RxPkt->packet_boundary_flag = 0x01;
        }

        RxPkt->connection_handle = conn_handle;
        RxPkt->broadcast_flag = 0;
        RxPkt->acl_data_total_length = pkt_len;
        memcpy(RxPkt->hci_acl_data_pkt, payload + offset, pkt_len);
        RxPkt->next = NULL;

        ll_append_acl_rx_pkt_to_hci_list(RxPkt);

        offset += pkt_len;

#ifdef _ENABLE_BLE_LL_FLOW_CONTROL_
        if (len > 0)
        {
            os_occupy_le_reserve_buffers(1);
        }
#endif
    }
#else

    if(OS_ALLOC_BUFFER(tx_table[LE_ACL_DATA_HANDLER_TASK].pool_handle,
                       (void**)&(RxPkt)) != BT_ERROR_OK)
    {
        /* TODO: show message */
#ifdef _ENABLE_BLE_LL_FLOW_CONTROL_
        os_release_one_le_reserved_buffer();
#endif
        return;
    }

    RxPkt->connection_handle = conn_handle;
    RxPkt->packet_boundary_flag = (frag1st == TRUE) ? 0x02: 0x01;
    RxPkt->broadcast_flag = 0;
    RxPkt->acl_data_total_length = len;
    memcpy(RxPkt->hci_acl_data_pkt, payload, len);
    RxPkt->next = NULL;
    ll_append_acl_rx_pkt_to_hci_list(RxPkt);

#endif
}

/**************************************************************************
 * Function     : ll_handle_generate_hci_event_signal
 *
 * Description  : This function is used to generate a HCI event signal
 *
 * Parameters   : param: the pointer of LL task's parameters
 *
 * Returns      : None
 *
 *************************************************************************/
void ll_handle_generate_hci_event_signal(LL_TASK_PARA_U *param)
{
    LL_CONN_HANDLE_UNIT *phandle;

    phandle = &ll_manager.conn_unit.handle[param->lmp_s.conn_entry];

    switch (param->lmp_s.sub_type)
    {
    case LL_TASK_HANDLE_CONN_COMP:
        hci_generate_le_connection_complete_event(param->lmp_s.status, phandle);
        break;

#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_
    case LL_TASK_HANDLE_ENHANCED_CONN_COMP:
        hci_generate_le_enhanced_connection_complete_event(param->lmp_s.status, phandle);
        break;
#endif /* _BT4_2_PRIVACY_FEATURE_SUPPORT_ */

    case LL_TASK_HANDLE_DISCONN:
        hci_generate_disconnection_complete_event(param->lmp_s.status,
                                    phandle->conn_handle, param->lmp_s.reason);
        break;

    case LL_TASK_HANDLE_LONG_TERM_KEY_REQUEST_REPLY:
        hci_generate_le_long_term_key_request_event(phandle);
        break;

    case LL_TASK_HANDLE_ENCRYPTION_KEY_REFRESH:
        {
            UCHAR event_parameter[HCI_ENCRYPTION_KEY_REFRESH_COMPLETE_EVENT_LEN];
            event_parameter[0] = param->lmp_s.status;
            event_parameter[1] = phandle->conn_handle & 0xFF;
            event_parameter[2] = phandle->conn_handle >> 8;
            hci_generate_event(HCI_ENCRYPTION_KEY_REFRESH_COMPLETE_EVENT,
                    event_parameter,
                    HCI_ENCRYPTION_KEY_REFRESH_COMPLETE_EVENT_LEN);
        }
        break;

    case LL_TASK_HANDLE_ENCRYPTION_CHANGE_EVENT:
        {
            UCHAR event_parameter[4];
            event_parameter[0] = param->lmp_s.status;;
            event_parameter[1] = phandle->conn_handle & 0xFF;
            event_parameter[2] = phandle->conn_handle >> 8;
            event_parameter[3] = param->lmp_s.reason; /* encryption_enabled */
            hci_generate_event(HCI_ENCRYPTION_CHANGE_EVENT, event_parameter, 4);
        }
        break;

    case LL_TASK_HANDLE_CONN_UPDT_COMP:
        hci_generate_le_connection_update_complete_event(param->lmp_s.status,
                                                         phandle);
        break;

#ifdef _BT4_2_LE_SECURE_CONNECTIONS_SUPPORT_
    case LL_TASK_HANDLE_READ_LOCAL_P256_PUBLIC_KEY_COMP:
        hci_generate_le_read_local_p256_public_key_complete_event();
        break;

    case LL_TASK_HANDLE_GENERATE_DHKEY_COMP:
        hci_generate_le_generate_dhkey_complete_event();
        break;
#endif

    default:
        break;
    }
}

/**************************************************************************
 * Function     : ll_scanner_handle_adv_ch_pdu
 *
 * Description  : This function is used to handle an advertising channel pdu
 *                in scanner role
 *
 * Parameters   : pHdr: the pointer of the header of advertising channel pdu
 *
 * Returns      : gen_adv_report(TRUE or FALSE)
 *
 *************************************************************************/
#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_
UINT8 ll_scanner_handle_adv_ch_pdu(UINT8 *pHdr, UINT8 resolving_list_idx)
#else
UINT8 ll_scanner_handle_adv_ch_pdu(UINT8 *pHdr)
#endif
{
    LE_HW_ADVERTISING_CH_RX_PKT_S *pRxPkt;
    UINT8 pdu_type;
    UINT8 pdu_len;
    UINT8 entry_id;
    UINT8 gen_adv_report = FALSE;
    UINT16 cam_addr;

    pRxPkt = (LE_HW_ADVERTISING_CH_RX_PKT_S *)pHdr;
    pdu_type = pRxPkt->Header.PDU_Type;
    pdu_len = pRxPkt->Header.Length;

    if ((pdu_type == LL_ADV_PDU_TYPE_SCAN_REQ) ||
            (pdu_type == LL_ADV_PDU_TYPE_CONNECT_REQ) ||
            (pdu_type > LL_ADV_PDU_TYPE_MAX))
    {
        return FALSE;
    }

    if (!ll_manager.scan_unit.filter_duplicate)
    {
        /* we need to generate le_adv_report_event to host */

        /* TODO: we need to check any rx pkts have not been recorded in the
           list if HW has bugs */
        return TRUE;
    }

    UINT8 type = pRxPkt->Header.TxAdd;
    UINT8* addr = pRxPkt->u1RemoteAddr;

#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_
    if (resolving_list_idx < LL_MAX_RESOLVING_LIST_SIZE)
    {
        type = ll_manager.resolving_list.item[resolving_list_idx].type;
        addr = ll_manager.resolving_list.item[resolving_list_idx].addr;
    }
#endif

    /* we need to check any duplicated packet and mark the address in the CAM
     * filter policy = 0x0, 0x2 => disable white list
     * filter policy = 0x1, 0x3 => enable white list
     */
    if (ll_manager.scan_unit.filter_policy % 2)
    {
        /* check white list */
        entry_id = ll_driver_search_dev_from_list(LL_WHITE_LIST_TYPE,
                   type, addr);

        if (entry_id != LL_MAX_WHITE_LIST_SIZE)
        {
            LL_DEV_ADDR_LIST_ENTRY *wl_entry = &ll_manager.white_list.entry[entry_id];
            /* this entry is found in the white list */
            if (!wl_entry->duplicated)
            {
                if (!ll_manager.scan_unit.active_scan
                        || (pdu_type != LL_ADV_PDU_TYPE_ADV_IND
                                && pdu_type != LL_ADV_PDU_TYPE_ADV_DISCOVER_IND))
                {
                    wl_entry->duplicated = TRUE;

                    /*  FW sets this entry to be duplicated in the CAM */
                    cam_addr = LE_CAM_WHITE_LIST_BASE + (entry_id << 1);
                    ll_driver_write_cam(cam_addr + 1, wl_entry->DWord[1]);
                    gen_adv_report = TRUE;
                }
                else if (!wl_entry->gen_report)
                {
                    wl_entry->gen_report = 1;
                    gen_adv_report = TRUE;
                }
            }
        }
    }
    else
    {
        /* check black list */
        entry_id = ll_driver_search_dev_from_list(LL_BLACK_LIST_TYPE,
                   type, addr);

        if (entry_id == LL_MAX_BLACK_LIST_SIZE)
        {
            if (!ll_manager.scan_unit.active_scan
                    || (pdu_type != LL_ADV_PDU_TYPE_ADV_IND
                            && pdu_type != LL_ADV_PDU_TYPE_ADV_DISCOVER_IND))
            {
                /* FW enables this entry in the CAM and set duplicated flag
                 in the CAM */
#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_
                ll_driver_add_dev_to_list(LL_BLACK_LIST_TYPE, type, addr, NULL, NULL);
#else
                ll_driver_add_dev_to_list(LL_BLACK_LIST_TYPE, type, addr);
#endif
            }
            else
            {
                /* When receiving ADV_IND or ADV_DISCOVER_IND (scannable) at
                 * first time, add it to blacklist only but not CAM to let
                 * subsequent SCAN_RSPs receivable.
                 */
                ll_dev_addr_list_center_add_list_entry(LL_BLACK_LIST_TYPE,
                        type, addr);
            }
            gen_adv_report = TRUE;
        }
        else
        {
            if (ll_manager.scan_unit.active_scan
                    && pdu_type == LL_ADV_PDU_TYPE_SCAN_RSP)
            {
                UINT16 cam_addr = LE_CAM_BLACK_LIST_BASE + (entry_id << 1);
                LL_DEV_ADDR_LIST_ENTRY *entry = &ll_manager.black_list.entry[entry_id];
                ll_driver_write_cam(cam_addr, entry->DWord[0]);
                ll_driver_write_cam(cam_addr + 1, entry->DWord[1]);
                gen_adv_report = TRUE;
            }
        }
    }

    return gen_adv_report;
}

/**************************************************************************
 * Function     : ll_scanner_generate_le_adv_report
 *
 * Description  : This function is used to generate an LE advertising report
 *                in scanner role
 *
 * Parameters   : pHdr: the pointer of the header (scanned rx pkt)
 *                pHWStaus: the pointer of the HW status (scanned rx pkt)
 *
 * Returns      : None
 *
 *************************************************************************/
#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_
void ll_scanner_generate_le_adv_report(UINT8 *pHdr, UINT8 *pHWStaus, UINT8 resolving_list_idx)
#else
void ll_scanner_generate_le_adv_report(UINT8 *pHdr, UINT8 *pHWStaus)
#endif
{
    LE_HW_ADVERTISING_CH_RX_PKT_S *pRxPkt;
    LE_HW_RX_PKT_TAIL_S *pTail;
    UINT8 pdu_type;
    UINT8 data_buf_size;
    LL_ADV_REPORT_UNIT *rpt;
    UINT8 index;
    INT8 rssi_rd;
    UINT8 num_of_reports;

    if (IS_USE_FOR_MUTE)
    {
        num_of_reports = 1;
    }
    else
    {
        num_of_reports = (LE_MAX_NUM_OF_ADV_REPORT_OPTION + 1);
    }

    pRxPkt = (LE_HW_ADVERTISING_CH_RX_PKT_S *)pHdr;
    pTail = (LE_HW_RX_PKT_TAIL_S *)pHWStaus;

    pdu_type = pRxPkt->Header.PDU_Type;

    switch (pdu_type)
    {
        case LL_ADV_PDU_TYPE_ADV_DIRECT_IND:
            data_buf_size = 0;
            break;

        case LL_ADV_PDU_TYPE_ADV_IND:
        case LL_ADV_PDU_TYPE_ADV_DISCOVER_IND:
        case LL_ADV_PDU_TYPE_ADV_NONCONN_IND:
        case LL_ADV_PDU_TYPE_SCAN_RSP:
            data_buf_size = pRxPkt->Header.Length - 6;
            break;

        default:
            return;
    }

#ifndef _ADD_RX_BACKUP_STATUS_INSIDE_BLE_RX_PKT_
    rssi_rd = lc_calculate_log_from_rssi(pTail->rssi);
#else
    rssi_rd = lc_calculate_log_from_rssi(*((UINT16*)&pTail->rssi + le_rx_backup_status_cnts));
#endif

#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_
    if (ll_rssi_manager.msft_monitor_rssi_mode ==
                            LE_MSFT_MONITOR_RSSI_VIA_CONN_HANDLE_MODE_MSFT)
    {
        if (rssi_app_msft_check_rssi_monitor_condition(pHdr, rssi_rd) == FALSE)
        {
            return;
        }
    }
#endif

    /* fw can queue the advertising report in one LE Advertising Report event */
    rpt = (LL_ADV_REPORT_UNIT *)&ll_manager.scan_unit.report;
    index = rpt->num_of_reports;
    rpt->event_type[index] = pRxPkt->Header.PDU_Type;

    BOOLEAN isAddrCopied = rpt->is_direct_adv_report = FALSE;

#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_

    if (ll_manager.address_resolution_enable)
    {
        if (resolving_list_idx < LL_MAX_RESOLVING_LIST_SIZE)
        {
            LE_REG_S_SET reg;
            reg.value = RD_LE_REG(LE_REG_PRIVACY_MISC0);

            /* match only one entries in resolving list, and AdvA is RPA */
            if (!pTail->multi_match && reg.le_privacy_misc0.fw_peer_rpa_val)
            {
                /* peer IRK is non-zero => RPA is resolved and give identity addr back to host */
                if (ll_manager.resolving_list.item[resolving_list_idx].valid_peer_IRK)
                {
                    rpt->addr_type[index] = ll_manager.resolving_list.item[resolving_list_idx].type ? LL_ADDR_TYPE_RPA_RANDOM: LL_ADDR_TYPE_RPA_PUBLIC;
                    memcpy(&rpt->addr[index][0], ll_manager.resolving_list.item[resolving_list_idx].addr, 6);
                    isAddrCopied = TRUE;
                }
            }
        }
    }

    if (ll_manager.support_ll_privacy)
    {
        if (pTail->mic_err) /* errata 6247, scan_rcv_nreso_inita in adv channel */
        {
            /* check event mask */
            UINT16 le_event_mask = ll_manager.le_event_mask[0] | (ll_manager.le_event_mask[1] << 8);
            if (le_event_mask & LE_DIRECT_ADVERTISING_REPORT_EVENT_MASK)
            {
                rpt->direct_addr_type[index] = 0x1;
                memcpy(&rpt->direct_addr[index][0], pRxPkt->AdvDirInd.InitA.u1Addr, 6);
                rpt->is_direct_adv_report = TRUE;
            }
        }
    }

#endif /* _BT4_2_PRIVACY_FEATURE_SUPPORT_ */

    if (!isAddrCopied)
    {
        rpt->addr_type[index] = pRxPkt->Header.TxAdd;
        memcpy(rpt->addr[index], pRxPkt->AdvInd.AdvA.u1Addr, 6);
    }

    if (!rpt->is_direct_adv_report)
    {
        rpt->data_len[index] = data_buf_size;
        if (data_buf_size > 0)
        {
            memcpy(rpt->data_buf[index], pRxPkt->AdvInd.u1AdvData, data_buf_size);
        }
    }

    rpt->rssi[index] = *(UINT8*)&rssi_rd;
    rpt->num_of_reports++;

    /* if _DAPE_TEST_FOR_MUTE or IS_USE_FOR_MUTE is on, num_of_reports is 1*/
    if (rpt->num_of_reports == num_of_reports)
    {
        hci_generate_le_advertising_report_event();
        rpt->num_of_reports = 0;
    }
}

LL_ADV_HANDLER ll_adv_handler[LL_ADV_PDU_TYPE_MAX + 1] = {
#ifdef CONFIG_TV_POWERON
        [LL_ADV_PDU_TYPE_ADV_NONCONN_IND] = ll_vendor_adv_handle_rtk_cmds,
#endif
        NULL
};

/**************************************************************************
 * Function     : ll_adv_handle_routine
 *
 * Description  : This function is used to handle the routine parser for
 *                advertising channel pdus
 *
 * Parameters   : pHdr: the pointer of the header in rx pkt
 *                len: the length of rx pkt
 *                min_payload_len: the minimum payload length
 *                send_adv_rpt: is it possible to send le advertising report
 *
 * Returns      : actual size of LL_ADV_IND PDU
 *
 *************************************************************************/
UINT8 ll_adv_handle_routine(UINT8 *pHdr, UINT16 len,
                                 UINT8 min_payload_len, UINT8 send_adv_rpt)
{
    LE_HW_ADVERTISING_CH_RX_PKT_S *pRxPkt;
    UINT8 offset;
    UINT8 size;

    pRxPkt = (LE_HW_ADVERTISING_CH_RX_PKT_S *)pHdr;

    offset = LL_ADV_CH_PDU_HEADER_SIZE + pRxPkt->Header.Length;
    if (offset & 0x01)
    {
        /* If the offset is odd, HW can add one pad between the tail of the
           payload and the head of the HW status */
        offset++;
    }
    size = offset + LE_HW_RX_PKT_TAIL_SIZE;

#ifdef _ADD_RX_BACKUP_STATUS_INSIDE_BLE_RX_PKT_
    size += le_rx_backup_status_cnts << 1;
#endif

    if (size > len)
    {
        LL_LOG_TRACE(RED, LE_MSG_ADV_CH_RX_PKT_LEN_ERROR, 4,
                     pRxPkt->Header.PDU_Type, pRxPkt->Header.Length, size, len);

        return len;
    }

    if (pRxPkt->Header.Length < min_payload_len)
    {
        LL_LOG_TRACE(RED, LE_MSG_ADV_CH_RX_PDU_LEN_ERROR, 2,
                     pRxPkt->Header.PDU_Type, pRxPkt->Header.Length);
        return size;
    }

    if (ll_adv_handler[pRxPkt->Header.PDU_Type] != NULL)
    {
        LE_HW_RX_PKT_TAIL_S *hw_status = (LE_HW_RX_PKT_TAIL_S *) (pHdr + offset);
        ll_adv_handler[pRxPkt->Header.PDU_Type](pRxPkt, hw_status, &send_adv_rpt);
    }

#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_
    UINT8 idx = ll_driver_store_rx_resolvable_private_address(pHdr, offset);
#endif

    if (send_adv_rpt)
    {
        if (ll_manager.scan_unit.enable)
        {
#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_
            if (ll_scanner_handle_adv_ch_pdu(pHdr, idx))
            {
                ll_scanner_generate_le_adv_report(pHdr, pHdr + offset, idx);
            }
#else
            if (ll_scanner_handle_adv_ch_pdu(pHdr))
            {
                ll_scanner_generate_le_adv_report(pHdr, pHdr + offset);
            }
#endif
        }
    }
    return size;
}

/**************************************************************************
 * Function     : ll_adv_handle_adv_ind
 *
 * Description  : This function is used to handle an LL_ADV_IND PDU
 *
 * Parameters   : pHdr: the pointer of the header in rx pkt
 *                len: the length of rx pkt
 *
 * Returns      : actual size of LL_ADV_IND PDU
 *
 *************************************************************************/
UINT8 ll_adv_handle_adv_ind(UINT8 *pHdr, UINT16 len)
{
    return ll_adv_handle_routine(pHdr, len, LL_ADV_CH_PAYLOAD_LEN_MIN, TRUE);
}

/**************************************************************************
 * Function     : ll_adv_handle_adv_direct_ind
 *
 * Description  : This function is used to handle an LL_ADV_DIRECT_IND PDU
 *
 * Parameters   : pHdr: the pointer of the header in rx pkt
 *                len: the length of rx pkt
 *
 * Returns      : actual size of LL_ADV_DIRECT_IND PDU
 *
 *************************************************************************/
UINT8 ll_adv_handle_adv_direct_ind(UINT8 *pHdr, UINT16 len)
{
    return ll_adv_handle_routine(pHdr, len, LL_ADV_DIRECT_PL_SIZE, TRUE);
}

/**************************************************************************
 * Function     : ll_adv_handle_adv_nonconn_ind
 *
 * Description  : This function is used to handle an LL_ADV_NONCONN_IND PDU
 *
 * Parameters   : pHdr: the pointer of the header in rx pkt
 *                len: the length of rx pkt
 *
 * Returns      : actual size of LL_ADV_NONCONN_IND PDU
 *
 *************************************************************************/
UINT8 ll_adv_handle_adv_nonconn_ind(UINT8 *pHdr, UINT16 len)
{
    return ll_adv_handle_routine(pHdr, len, LL_ADV_CH_PAYLOAD_LEN_MIN, TRUE);
}

/**************************************************************************
 * Function     : ll_adv_handle_scan_req
 *
 * Description  : This function is used to handle an LL_SCAN_REQ PDU
 *
 * Parameters   : pHdr: the pointer of the header in rx pkt
 *                len: the length of rx pkt
 *
 * Returns      : actual size of LL_SCAN_REQ PDU
 *
 *************************************************************************/
UINT8 ll_adv_handle_scan_req(UINT8 *pHdr, UINT16 len)
{
    return ll_adv_handle_routine(pHdr, len, LL_SCAN_REQ_PL_SIZE, FALSE);
}

/**************************************************************************
 * Function     : ll_adv_handle_scan_rsp
 *
 * Description  : This function is used to handle an LL_SCAN_RSP PDU
 *
 * Parameters   : pHdr: the pointer of the header in rx pkt
 *                len: the length of rx pkt
 *
 * Returns      : actual size of LL_SCAN_RSP PDU
 *
 *************************************************************************/
UINT8 ll_adv_handle_scan_rsp(UINT8 *pHdr, UINT16 len)
{
    return ll_adv_handle_routine(pHdr, len, LL_SCAN_RSP_PL_SIZE_MIN, TRUE);
}

/**************************************************************************
 * Function     : ll_adv_handle_connection_req
 *
 * Description  : This function is used to handle an LL_CONNECTION_REQ PDU
 *
 * Parameters   : pHdr: the pointer of the header in rx pkt
 *                len: the length of rx pkt
 *
 * Returns      : actual size of LL_CONNECTION_REQ PDU
 *
 *************************************************************************/
UINT8 ll_adv_handle_connection_req(UINT8 *pHdr, UINT16 len)
{
    LE_HW_ADVERTISING_CH_RX_PKT_S *pRxPkt;
    UINT8 offset;
    UINT8 size;

	DEF_CRITICAL_SECTION_STORAGE;

    pRxPkt = (LE_HW_ADVERTISING_CH_RX_PKT_S *)pHdr;

    offset = LL_ADV_CH_PDU_HEADER_SIZE + pRxPkt->Header.Length;
    if (offset & 0x01)
    {
        /* If the offset is odd, HW can add one pad between the tail of the
           payload and the head of the HW status */
        offset++;
    }
    size = offset + LE_HW_RX_PKT_TAIL_SIZE;

#ifdef _ADD_RX_BACKUP_STATUS_INSIDE_BLE_RX_PKT_
    size += le_rx_backup_status_cnts << 1;
#endif

    if (size > len)
    {
        LL_LOG_TRACE(RED, LE_MSG_ADV_CH_RX_PKT_LEN_ERROR, 4,
                     pRxPkt->Header.PDU_Type, pRxPkt->Header.Length, size, len);

        return len;
    }

    if (pRxPkt->Header.Length != LL_CONNECTION_REQ_PL_SIZE)
    {
        LL_LOG_TRACE(RED, LE_MSG_ADV_CH_RX_PDU_LEN_ERROR, 2,
                     pRxPkt->Header.PDU_Type, pRxPkt->Header.Length);
        return size;
    }

    /* update connection req parameters */
    LL_CONN_HANDLE_UNIT *phandle = &ll_manager.conn_unit.handle[0];

    BOOLEAN isAddrCopied = FALSE;

#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_

    UINT8 idx = ll_driver_store_rx_resolvable_private_address(pHdr, offset);

    if (idx < LL_MAX_RESOLVING_LIST_SIZE)
    {
        /* find matching entry => store identity address to remote address */
        phandle->remote_info.addr_type = ll_manager.resolving_list.item[idx].type ? LL_ADDR_TYPE_RPA_RANDOM : LL_ADDR_TYPE_RPA_PUBLIC;
        memcpy(phandle->remote_info.u2addr, ll_manager.resolving_list.item[idx].addr, 6);
        isAddrCopied = TRUE;
    }

#endif /* _BT4_2_PRIVACY_FEATURE_SUPPORT_ */

    if (!isAddrCopied)
    {
        phandle->remote_info.addr_type = pRxPkt->Header.TxAdd ? 1 : 0;
        memcpy(phandle->remote_info.u2addr, &pRxPkt->ConnReq.InitA, 6);
    }

    phandle->ce_interval = pRxPkt->ConnReq.u1Interval[0] |
                            (pRxPkt->ConnReq.u1Interval[1] << 8);
    phandle->slave_latency = pRxPkt->ConnReq.u1Latency[0] |
                            (pRxPkt->ConnReq.u1Latency[1] << 8);
    phandle->supervision_to =  pRxPkt->ConnReq.u1Timeout[0] |
                              (pRxPkt->ConnReq.u1Timeout[1] << 8);
    // dape added
    phandle->ce_length = (phandle->ce_interval - 1) << 1;

    ll_manager.conn_unit.masterSCA =  pRxPkt->ConnReq.SCA;

    MINT_OS_ENTER_CRITICAL();
    phandle->recv_conn_req = TRUE;

    if (phandle->connected)
    {
        LL_TASK_PARA_U task_param;
        OS_SIGNAL sig_send;

        /* generate connection complete event to host */
        task_param.Dword = 0;
#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_
        if (ll_manager.address_resolution_enable)
        {
            UINT16 le_event_mask = ll_manager.le_event_mask[0] | (ll_manager.le_event_mask[1] << 8);
            if (((le_event_mask & LE_ENHANCED_CONNECTION_COMPLETE_EVENT_MASK) > 0) ||
                ((le_event_mask & (LE_ENHANCED_CONNECTION_COMPLETE_EVENT_MASK | LE_CONNECTION_COMPLATE_EVENT_MASK)) == 0))
            {
                task_param.lmp_s.sub_type = LL_TASK_HANDLE_ENHANCED_CONN_COMP;
            }
        }
        else
        {
            task_param.lmp_s.sub_type = LL_TASK_HANDLE_CONN_COMP;
        }
#else
        task_param.lmp_s.sub_type = LL_TASK_HANDLE_CONN_COMP;
#endif
        task_param.lmp_s.conn_entry = 0;
        task_param.lmp_s.status = HCI_COMMAND_SUCCEEDED;
        sig_send.type = LL_GEN_HCI_EVENT_SIGNAL;
        sig_send.param = (OS_ADDRESS)task_param.Dword;
        OS_SEND_SIGNAL_TO_TASK (lmp_task_handle, sig_send);

        /* create and start the supervision timer */
        ll_start_timer(0, LL_TIMER_TYPE_SUPERVISION);
    }

    MINT_OS_EXIT_CRITICAL();

    return size;
}

/**************************************************************************
 * Function     : ll_adv_handle_adv_discover_ind
 *
 * Description  : This function is used to handle an LL_DISCOVER_IND PDU
 *
 * Parameters   : pHdr: the pointer of the header in rx pkt
 *                len: the length of rx pkt
 *
 * Returns      : actual size of LL_DISCOVER_IND PDU
 *
 *************************************************************************/
UINT8 ll_adv_handle_adv_discover_ind(UINT8 *pHdr, UINT16 len)
{
    return ll_adv_handle_routine(pHdr, len, LL_ADV_CH_PAYLOAD_LEN_MIN, TRUE);
}

/**************************************************************************
 * Function     : ll_adv_handle_unknown_pdu
 *
 * Description  : This function is used to handle an unknown PDU in advertising
 *                channel
 *
 * Parameters   : pHdr: the pointer of the header in rx pkt
 *                len: the length of rx pkt
 *
 * Returns      : actual size of unknown PDU
 *
 *************************************************************************/
UINT8 ll_adv_handle_unknown_pdu(UINT8 *pHdr, UINT16 len)
{
    LE_HW_ADVERTISING_CH_RX_PKT_S *pRxPkt;
    UINT8 offset;
    UINT8 size;

    pRxPkt = (LE_HW_ADVERTISING_CH_RX_PKT_S *)pHdr;

    offset = LL_ADV_CH_PDU_HEADER_SIZE + pRxPkt->Header.Length;
    if (offset & 0x01)
    {
        /* If the offset is odd, HW can add one pad between the tail of the
           payload and the head of the HW status */
        offset++;
    }
    size = offset + LE_HW_RX_PKT_TAIL_SIZE;

#ifdef _ADD_RX_BACKUP_STATUS_INSIDE_BLE_RX_PKT_
    size += le_rx_backup_status_cnts << 1;
#endif

    LL_LOG_TRACE(YELLOW, LE_MSG_ADV_CH_RX_UNKNOWN_PDU, 2,
                 pRxPkt->Header.PDU_Type, size);
    if (size > len)
    {
        LL_LOG_TRACE(RED, LE_MSG_ADV_CH_RX_PKT_LEN_ERROR, 4,
                     pRxPkt->Header.PDU_Type, pRxPkt->Header.Length, size, len);
        return len;
    }

    return size;
}

/*************************************************************************
* The function table of LL Handle Advertising Channel PDUs
*************************************************************************/
UINT8 (*(ll_handle_adv_channel_pdu[])) (UINT8 *pHdr, UINT16 len) = {
    ll_adv_handle_adv_ind,
    ll_adv_handle_adv_direct_ind,
    ll_adv_handle_adv_nonconn_ind,
    ll_adv_handle_scan_req,
    ll_adv_handle_scan_rsp,
    ll_adv_handle_connection_req,
    ll_adv_handle_adv_discover_ind
};

/**************************************************************************
 * Function     : ll_pre_parser_adv_channel_pdu
 *
 * Description  : This function is used to previous parser an advertising
 *                channel PDU stream in normal mode
 *
 * Parameters   : pdata: the pointer of the header in rx pkt
 *                len: the length of rx pkt
 *
 * Returns      : incomplete pdu length
 *
 *************************************************************************/
UINT16 ll_pre_parser_adv_channel_pdu(UINT8 *pdata, UINT16 len)
{
    LE_HW_ADVERTISING_CH_RX_PKT_S *pRxPkt;
    UINT16 offset = 0;
    UINT8 hw_pdu_len;

    while (len != 0)
    {
        pRxPkt = (LE_HW_ADVERTISING_CH_RX_PKT_S *)(pdata + offset);
        hw_pdu_len = LL_ADV_CH_PDU_HEADER_SIZE + pRxPkt->Header.Length;
        if (hw_pdu_len & 0x01)
        {
            /* If the pdu length is odd, HW can add one pad between the tail of the
               payload and the head of the HW status */
            hw_pdu_len++;
        }
        hw_pdu_len += LE_HW_RX_PKT_TAIL_SIZE;

#ifdef _ADD_RX_BACKUP_STATUS_INSIDE_BLE_RX_PKT_
        hw_pdu_len += le_rx_backup_status_cnts << 1;
#endif

        offset += hw_pdu_len;

        if (len < hw_pdu_len)
        {
            break;
        }
        len -= hw_pdu_len;
    }

    return len;
}


/**************************************************************************
 * Function     : ll_parser_adv_channel_pdu
 *
 * Description  : This function is used to parser an advertising channel PDU
 *                stream in normal mode
 *
 * Parameters   : pdata: the pointer of the header in rx pkt
 *                len: the length of rx pkt
 *
 * Returns      : None
 *
 *************************************************************************/
void ll_parser_adv_channel_pdu(UINT8 *pdata, UINT16 len)
{

#ifdef _ROM_CODE_PATCHED_
    if (rcp_ll_handle_adv_channel_pdu != NULL)
    {
        if (rcp_ll_handle_adv_channel_pdu((void *)pdata, len))
        {
            return;
        }
    }
#endif

    LL_ADVERTISING_CH_PDU_HEADER *pAdvHdr;
    UINT16 offset = 0;
    UINT8 pdu_type;
    UINT8 hw_pdu_len;

    while (len != 0)
    {
        pAdvHdr = (LL_ADVERTISING_CH_PDU_HEADER *)(pdata + offset);
        pdu_type = pAdvHdr->PDU_Type;

        if (pdu_type > LL_ADV_PDU_TYPE_MAX)
        {
            hw_pdu_len = ll_adv_handle_unknown_pdu(pdata + offset, len);
        }
        else
        {
            hw_pdu_len = ll_handle_adv_channel_pdu[pdu_type](pdata + offset,
                                                             len);
        }

#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_INFO_
        if (ll_manager.debug_privacy_enable)
        {
            LL_LOG_TRACE(YELLOW, MSG_LE_PRIVACY_PEER_RPA, 4,
                ll_manager.local_IRK_idx, RD_LE_REG(LE_REG_PEER_RESOLVABLE_ADDR_H),
                RD_LE_REG(LE_REG_PEER_RESOLVABLE_ADDR_M), RD_LE_REG(LE_REG_PEER_RESOLVABLE_ADDR_L));
            LL_LOG_TRACE(YELLOW, MSG_LE_PRIVACY_DUMP_DEBUG, 2, LE_REG_PRIVACY_PEER_RULE_PASS, RD_LE_REG(LE_REG_PRIVACY_PEER_RULE_PASS));
            LL_LOG_TRACE(YELLOW, MSG_LE_PRIVACY_DUMP_DEBUG, 2, LE_REG_PRIVACY_LOCAL_RULE_PASS, RD_LE_REG(LE_REG_PRIVACY_LOCAL_RULE_PASS));
        }
#endif

        LL_LOG_TRACE(WHITE, LE_MSG_ADV_RX_DATA_CONTENT, 4,
                     pdu_type, offset, hw_pdu_len, len);

        offset += hw_pdu_len;
        len -= hw_pdu_len;
    }

    /* check any le_adv_report(s) are pending in the FW queue. If yes,
       we compose a LE_Advertising_Repot event to host and clear the queue */
    if (ll_manager.scan_unit.enable &&
            (ll_manager.scan_unit.report.num_of_reports > 0))
    {
        hci_generate_le_advertising_report_event();
        ll_manager.scan_unit.report.num_of_reports = 0;
    }
}

/**************************************************************************
 * Function     : ll_parser_data_channel_pdu
 *
 * Description  : This function is used to parser a data channel PDU
 *                in normal mode
 *
 * Parameters   : pdata: the pointer of the header in rx pkt
 *                len: the length of rx pkt
 *                entry: the id of the connection entry
 *
 * Returns      : None
 *
 *************************************************************************/
void ll_parser_data_channel_pdu(UINT8 *pdata, UINT16 len, UINT8 entry)
{
    LE_HW_DATA_CH_RX_PKT_S *pDataHdr;
    LE_HW_RX_PKT_TAIL_S *pHWStaus;
    LE_HW_RX_PKT_TAIL_S hw_status;
    UINT16 offset = 0;
    UINT8 llid;
    UINT8 payload_len;
    UINT8 hw_pdu_len;
    OS_SIGNAL sig_send;
    UINT8 acl_pkt_cnts;

    acl_pkt_cnts = ll_manager.conn_unit.l2h_acl_pkt_list.pktcnt;

    while (len != 0)
    {
        pDataHdr = (LE_HW_DATA_CH_RX_PKT_S *)(pdata + offset);
        llid = pDataHdr->Header.llid;
        payload_len = pDataHdr->Header.length;

        hw_pdu_len = LL_DATA_CH_PDU_HEADER_SIZE +
                     LE_HW_RX_PKT_TAIL_SIZE + payload_len;

        offset += LL_DATA_CH_PDU_HEADER_SIZE + payload_len;

        if (offset & 0x01)
        {
            offset++;
            hw_pdu_len++;
        }

        pHWStaus = (LE_HW_RX_PKT_TAIL_S *)(pdata + offset);
        hw_status.rx_status = pHWStaus->rx_status;

        LL_CONN_HANDLE_UNIT *p_handle;
        p_handle = &ll_manager.conn_unit.handle[entry];
#ifndef _ADD_RX_BACKUP_STATUS_INSIDE_BLE_RX_PKT_
        p_handle->last_rssi = pHWStaus->rssi;
#else
        p_handle->last_rssi = *((UINT16*)&pHWStaus->rssi + le_rx_backup_status_cnts);
#endif

        //RT_BT_LOG(BLUE, DAPE_TEST_LOG525, 6,
        //	pHWStaus->rx_status, p_handle->last_rssi, entry, 0,0,0);

        offset += LE_HW_RX_PKT_TAIL_SIZE;

#ifdef _ADD_RX_BACKUP_STATUS_INSIDE_BLE_RX_PKT_
        offset += le_rx_backup_status_cnts << 1;
#endif

        do
        {
            if (len < hw_pdu_len)
            {
                hw_pdu_len = len;
                break;
            }

            if (hw_status.rx_status & 0x3F)
            {
                LL_LOG_TRACE(RED, LE_MSG_DATA_CH_INVALID_RX_PKT, 4,
                             entry, pDataHdr->u2Header, hw_status.rx_status,
                             p_handle->conn_counter);
                break;
            }

            if (hw_status.decrypted)
            {
                payload_len -= LL_DATA_CH_MIC_SIZE;
#ifdef _BT4_1_LE_PING_FEATURE_SUPPORT_
                /* received data packet with valid mic,
                 * restart the LE ping timer if auth_payload_timeout is written
                 */
                llc_ping_req_restart_timer(entry);
#endif
            }

            switch (llid)
            {
            case LL_LLID_CTRL_PDU:
                llc_parser_ctrl_pdu(pDataHdr->buf, payload_len, entry);

#ifdef _ENABLE_BLE_LL_FLOW_CONTROL_
                /* release buffer because llc pdu can not occupy the aclu
                   buffer */
                os_release_one_le_reserved_buffer();
#endif
                break;

            case LL_LLID_DATA_PDU_START:
                ll_handle_rx_data_pdu(pDataHdr->buf, payload_len, TRUE, entry);
                break;

            case LL_LLID_DATA_PDU_CONT:
                ll_handle_rx_data_pdu(pDataHdr->buf, payload_len, FALSE, entry);
                break;

            default:
                break;
            }
        }
        while (0);

        len -= hw_pdu_len;
    }

    /* we can send acl data pkt to host */
    if (ll_manager.conn_unit.l2h_acl_pkt_list.in_procedure == FALSE)
    {
        if (acl_pkt_cnts < ll_manager.conn_unit.l2h_acl_pkt_list.pktcnt)
        {
            sig_send.type = HCI_TD_LE_ACL_DATA_TO_HOST_SIGNAL;
            if (OS_SEND_SIGNAL_TO_TASK(hci_td_task_handle, sig_send)
                    != BT_ERROR_OK)
            {
#ifdef ENABLE_LOGGER_LEVEL_2
                LC_LOG_ERROR(LOG_LEVEL_LOW, "OS send signal to task failed\n");
#endif
            }
            else
            {
                ll_manager.conn_unit.l2h_acl_pkt_list.in_procedure = TRUE;
            }
        }
    }
}

/**************************************************************************
 * Function     : ll_master_connection_complete_process_second_part
 *
 * Description  : This function is used to send related events to
 *                host when LE initiator has create a connection
 *
 * Parameters   : entry: the id of the connection entry
 *
 * Returns      : None
 *
 *************************************************************************/
void ll_handle_master_connection_complete_process_second_part(UINT8 hw_conn_ent)
{
    LL_TASK_PARA_U task_param;
    OS_SIGNAL sig_send;

    /* generate connection complete event to host */
    task_param.Dword = 0;

#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_
    if (ll_manager.address_resolution_enable)
    {
        UINT16 le_event_mask = ll_manager.le_event_mask[0] | (ll_manager.le_event_mask[1] << 8);

        if (((le_event_mask & LE_ENHANCED_CONNECTION_COMPLETE_EVENT_MASK) > 0) ||
            ((le_event_mask & (LE_ENHANCED_CONNECTION_COMPLETE_EVENT_MASK | LE_CONNECTION_COMPLATE_EVENT_MASK)) == 0))
        {
            task_param.lmp_s.sub_type = LL_TASK_HANDLE_ENHANCED_CONN_COMP;
        }
    }
    else
    {
        task_param.lmp_s.sub_type = LL_TASK_HANDLE_CONN_COMP;
    }
#else
    task_param.lmp_s.sub_type = LL_TASK_HANDLE_CONN_COMP;
#endif

    task_param.lmp_s.conn_entry = hw_conn_ent;
    task_param.lmp_s.status = HCI_COMMAND_SUCCEEDED;
    sig_send.type = LL_GEN_HCI_EVENT_SIGNAL;
    sig_send.param = (OS_ADDRESS)task_param.Dword;
    OS_SEND_SIGNAL_TO_TASK (lmp_task_handle, sig_send);

#ifdef _DAPE_TEST_SLOT_OFFSET_FOR_LE
    if (IS_LE_CONN_INTV_AUTO_UPDATE_EN)
    {
        if (g_le_conn_interval_changed & (1 << hw_conn_ent))
        {
            /* generate connection update complete event to host */
            task_param.Dword = 0;
            task_param.lmp_s.sub_type = LL_TASK_HANDLE_CONN_UPDT_COMP;
            task_param.lmp_s.conn_entry = hw_conn_ent;
            task_param.lmp_s.status = HCI_COMMAND_SUCCEEDED;
            sig_send.type = LL_GEN_HCI_EVENT_SIGNAL;
            sig_send.param = (OS_ADDRESS)task_param.Dword;
            OS_SEND_SIGNAL_TO_TASK (lmp_task_handle, sig_send);
        }
    }
#endif

}
