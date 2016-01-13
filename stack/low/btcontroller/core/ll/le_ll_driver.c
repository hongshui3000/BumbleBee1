enum { __FILE_NUM__= 205 };

#include "le_ll.h"
#include "le_ll_driver.h"
#include "le_hw_reg.h"
#include "bzdma.h"
#include "mem.h"
#include "common_utils.h"
#include "bb_driver.h"
#include "le_ll_test.h"
#ifdef _CCH_SLOT_OFFSET_
#ifdef _DAPE_TEST_SLOT_OFFSET_FOR_LE
#include "le_hci_4_0.h"
#include "lmp.h"
#endif
#endif


/* The Table of Translation from RF Channel to LE Channel Index */
const UINT8 ll_rf_channel_to_le_channel_idx[40] = {
    37,  0,  1,  2,  3,  4,  5,  6,  7,  8,     // RF channel 0~9
     9, 10, 38, 11, 12, 13, 14, 15, 16, 17,     // RF channel 10~19
    18, 19, 20, 21, 22, 23, 24, 25, 26, 27,     // RF channel 20~29
    28, 29, 30, 31, 32, 33, 34, 35, 36, 39      // RF channel 30~39
};

/* The Table of Translation from LE Channel Index to RF Channel */
const UINT8 ll_le_channel_idx_to_rf_channel[40] = {
     1,  2,  3,  4,  5,  6,  7,  8,  9, 10,     // Data channel 0~9
    11, 13, 14, 15, 16, 17, 18, 19, 20, 21,     // Data channel 10~19
    22, 23, 24, 25, 26, 27, 28, 29, 30, 31,     // Data channel 20~29
    32, 33, 34, 35, 36, 37, 38,  0, 12, 39      // Data and Adv channel 30~39
};

/* The Table of the Length every valid LLC Ctrl PDU */
const UINT8 ll_ctrl_pdu_length[] = {
    LL_CONNECTION_UPDATE_REQ_LEN,       /* opcode = 0x00 */
    LL_CHANNEL_MAP_REQ_LEN,             /* opcode = 0x01 */
    LL_TERMINATE_IND_LEN,               /* opcode = 0x02 */
    LL_ENC_REQ_LEN,                     /* opcode = 0x03 */
    LL_ENC_RSP_LEN,                     /* opcode = 0x04 */
    LL_START_ENC_REQ_LEN,               /* opcode = 0x05 */
    LL_START_ENC_RSP_LEN,               /* opcode = 0x06 */
    LL_UNKNOWN_RSP_LEN,                 /* opcode = 0x07 */
    LL_FEATURE_REQ_LEN,                 /* opcode = 0x08 */
    LL_FEATURE_RSP_LEN,                 /* opcode = 0x09 */
    LL_PAUSE_ENC_REQ_LEN,               /* opcode = 0x0A */
    LL_PAUSE_ENC_RSP_LEN,               /* opcode = 0x0B */
    LL_VERSION_IND_LEN,                 /* opcode = 0x0C */
    LL_REJECT_IND_LEN,                  /* opcode = 0x0D */
#ifdef _SUPPORT_VER_4_1_
    LL_SLAVE_FEATURE_REQ_LEN,           /* opcode = 0x0E */
    LL_CONNECTION_PARAM_REQ_LEN,        /* opcode = 0x0F */
    LL_CONNECTION_PARAM_RSP_LEN,        /* opcode = 0x10 */
    LL_REJECT_IND_EXT_LEN,              /* opcode = 0x11 */
    LL_PING_REQ_LEN,                    /* opcode = 0x12 */
    LL_PING_RSP_LEN,                    /* opcode = 0x13 */
#endif
#ifdef _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_
    [0x14] = LL_LENGTH_REQ_LEN,
    [0x15] = LL_LENGTH_RSP_LEN,
#endif
};

#ifdef _ROM_CODE_PATCHED_
PF_ROM_CODE_PATCH_FUNC rcp_ll_data_cmd_decision = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_ll_driver_wake_from_slave_latency = NULL;
#endif

#ifdef _CHECK_LE_CAM_ACCESS_
UINT8 g_ll_driver_write_cam_warning_flag = 0;
#endif

/**************************************************************************
 * Function     : ll_driver_read_cam
 *
 * Description  : This function is used to read CAM of LE Controller HW.
 *
 * Parameters   : cam_addr: the address of CAM.
 *
 * Returns      : 32 bit read data from CAM
 *
 *************************************************************************/
UINT32 ll_driver_read_cam(UINT16 cam_addr)
{
    UINT32_S rdata;
#ifdef _INFINITE_LOOP_PROTECT_
    UINT32 count = 0;
#endif

    DEF_CRITICAL_SECTION_STORAGE;

    MINT_OS_ENTER_CRITICAL();

    /* set address of CAM and enable read command (bit15 = 0) */
    WR_LE_REG(LE_REG_CAM_ACCESS, cam_addr & 0x3FFF);

    /* polling cam_int flag (bit4) until it is completed */
    while (1)
    {
        if (RD_LE_REG(LE_REG_INT_MISR) & (1 << 4))
        {
#ifdef _INFINITE_LOOP_PROTECT_
            if (count > 1000000)
            {
                RT_BT_LOG(RED, MSG_LL_CAM_READ_ERR, 1, cam_addr);
                break;
            }
            count++;
#endif
            break;
        }
    }

    /* the CAM indication flag will be clear automatically after read data port*/

    /* copy value from read data port of CAM */
    rdata.u2Byte[0] = RD_LE_REG(LE_REG_CAM_DATA_L);
    rdata.u2Byte[1] = RD_LE_REG(LE_REG_CAM_DATA_H);

    MINT_OS_EXIT_CRITICAL();

    return rdata.u4Byte;
}

/**************************************************************************
 * Function     : ll_driver_write_cam
 *
 * Description  : This function is used to write CAM of LE Controller HW
 *
 * Parameters   : cam_addr: the address of CAM
 *                wdata: 32 bit data for write
 *
 * Returns      : BT_ERROR_OK or BT_ERROR_xxx
 *
 *************************************************************************/
UINT8 ll_driver_write_cam(UINT16 cam_addr, UINT32 wdata)
{
#ifdef _INFINITE_LOOP_PROTECT_
    UINT32 count = 0;
    UINT8 ret = BT_ERROR_OK;
#endif

    DEF_CRITICAL_SECTION_STORAGE;

    MINT_OS_ENTER_CRITICAL();

    /* fill data to write data port of CAM */
    WR_LE_REG(LE_REG_CAM_DATA_L, wdata & 0xFFFF);
    WR_LE_REG(LE_REG_CAM_DATA_H, wdata >> 16);

    /* set address of CAM and enable write command (bit15 = 1) */
    WR_LE_REG(LE_REG_CAM_ACCESS, (cam_addr & 0x3FFF) | 0x8000);

    /* polling cam_int flag (bit4) until it is completed */
    while (1)
    {
        if (RD_LE_REG(LE_REG_INT_MISR) & (1 << 4))
        {
#ifdef _INFINITE_LOOP_PROTECT_
            if (count > 1000000)
            {
                RT_BT_LOG(RED, MSG_LL_CAM_READ_ERR, 1, cam_addr, wdata);
                break;
            }
            count++;
#endif

            break;
        }
    }

#ifdef _CHECK_LE_CAM_ACCESS_
    /* to check the write process is valid or not */
    if (RD_LE_REG(LE_REG_CAM_ACCESS) & BIT14)
    {
        LL_LOG_TRACE(RED, MSG_LL_CAM_ACCESS_WARNNING, 2, cam_addr, wdata);
        g_ll_driver_write_cam_warning_flag = 1;
        ret = BT_ERROR_INVALID_OPERATION;
    }
#endif

    /* the CAM indication flag will be clear automatically after read data port*/
    RD_LE_REG(LE_REG_CAM_DATA_L);

    MINT_OS_EXIT_CRITICAL();

    return ret;
}

/**************************************************************************
 * Function     : ll_driver_set_le_instruction
 *
 * Description  : This function is used to set LE instruction.
 *
 * Parameters   : insctruction_code: instruction code
 *
 * Returns      : BT_ERROR_OK or BT_ERROR
 *
 *************************************************************************/
UINT8 ll_driver_set_le_instruction(UINT8 insctruction_code)
{
    LE_REG_S_SET reg;

    if (insctruction_code > LE_HW_INSTRUCTION_MAX)
    {
        /* Need to fix FW where call this function */
        LL_LOG_TRACE(RED, LE_MSG_HW_INSTRUCTION_CODE_ERROR, 1, insctruction_code);
        return BT_ERROR;
    }

    WR_LE_REG(LE_REG_INSTRUCTION, insctruction_code);

    if (insctruction_code == LE_HW_INSTRUCTION_CONN_CANCEL)
    {
        reg.value = RD_LE_REG(LE_REG_INSTRUCTION);
        if (!reg.instruction.conn_cancel_fail)
        {
            ll_manager.initiator_unit.enable = 0;
        }
    }
    return BT_ERROR_OK;
}

/**************************************************************************
 * Function     : ll_driver_advertising_schedule_bzdma
 *
 * Description  : This function is used to schedule a tx command of BZDMA for
 *                non-empty data of payload part in advertising pdu or
 *                non-empty data of payload part in scan response pdu.
 *                This fucntion shall be called before we fire HW advertising
 *                request instruction.
 *
 * Parameters   : le_pdu_type: LE pdu type of BZDMA tx command
 *
 * Returns      : BT_ERROR_OK or BT_ERROR
 *************************************************************************/
UINT8 ll_driver_advertising_schedule_bzdma(UINT8 le_pdu_type)
{
    LL_ADVERTISER_UNIT *adv = &ll_manager.adv_unit;
#ifdef _NEW_BZDMA_FROM_V8_
    BZDMA_BLEONLY_ADV_TX_DESC_ENTRY_STATUS *pTxEnt;
    UINT8 cur_frag_idx = 0;
    UINT8 i;
#else
    BZDMA_TX_DESC_ENTRY_STATUS *pTxEnt;
#endif

    UINT8 tx_id;
    UINT8 cur_seg_idx = 0;

    /* if llid_type is zero, it means the advertising data legnth of advertising
       pdu and scanning data length of scan response pdu both are zero. */
    if (le_pdu_type != 0)
    {
        if (le_pdu_type > BZDMA_LE_PDU_TYPE_MAX)
        {
            /* TODO: Need to check FW if this case is occured */
            LL_LOG_TRACE(RED, LE_MSG_INVALID_TXCMD_LLID, 1, le_pdu_type);
            return BT_ERROR;
        }

#ifdef _NEW_BZDMA_FROM_V8_
        /* get free tx entry */
        tx_id = bzdma_get_ble_dedicated_free_tx_entry(BZDMA_LEONLY_TX_ENTRY_TYPE_ADV0);
        if (tx_id == BZDMA_TX_DESC_ENTRY_NUM)
        {
            /* TODO: Need to schedule it to queue if this case is occured */
            //LL_LOG_TRACE(RED, LE_MSG_NO_FREE_TXCMD_OF_ADV_TYPE, 0, 0);
            return BT_ERROR;
        }

        pTxEnt = &Bzdma_Manager.BleAdvTxEntSta;
        pTxEnt->le_pdu_cmd = le_pdu_type;

        /* init tx descriptors */
        for (i = 0; i < (BZDMA_LEONLY_TX_ENTRY_MAX_SEGS_ADV0 *
                        BZDMA_LEONLY_TX_ENTRY_MAX_FRAGS_ADV0); i++)
        {
            pTxEnt->pTxFragDesc[i].DWord = 0;
        }
        pTxEnt->pTxSegDesc[0].isLast = FALSE;
        pTxEnt->pTxSegDesc[1].isLast = FALSE;

        /* prepare bzdma txcmd for non-empty data of advertising pdu */
        if (adv->adv_len > 0)
        {
            pTxEnt->pTxFragDesc[cur_frag_idx].len = adv->adv_len;
            pTxEnt->pTxFragDesc[cur_frag_idx].start_addr = (UINT32)adv->adv_buf;
            pTxEnt->pTxFragDesc[cur_frag_idx].isLast = TRUE;
            pTxEnt->pTxSegDesc[cur_seg_idx].len = adv->adv_len;
            cur_frag_idx += BZDMA_LEONLY_TX_ENTRY_MAX_FRAGS_ADV0;
            cur_seg_idx++;
        }

        /* prepare bzdma txcmd for non-empty data of scan response pdu */
        if (adv->scan_rsp_len > 0)
        {
            pTxEnt->pTxFragDesc[cur_frag_idx].len = adv->scan_rsp_len;
            pTxEnt->pTxFragDesc[cur_frag_idx].start_addr = (UINT32)adv->scan_rsp_buf;
            pTxEnt->pTxFragDesc[cur_frag_idx].isLast = TRUE;
            pTxEnt->pTxSegDesc[cur_seg_idx].len = adv->scan_rsp_len;
            cur_frag_idx += BZDMA_LEONLY_TX_ENTRY_MAX_FRAGS_ADV0;
            cur_seg_idx++;
        }

        /* set last flag */
        pTxEnt->pTxSegDesc[cur_seg_idx - 1].isLast = TRUE;

        /* send tx command */
        bzdma_send_ble_txcmd(tx_id);
#else

        /* get free tx entry */
        tx_id = bzdma_get_dedicated_free_tx_entry(BZDMA_TX_ENTRY_TYPE_LE_ADV);
        if (tx_id == BZDMA_TX_DESC_ENTRY_NUM)
        {
            /* TODO: Need to schedule it to queue if this case is occured */
            //LL_LOG_TRACE(RED, LE_MSG_NO_FREE_TXCMD_OF_ADV_TYPE, 0, 0);
            return BT_ERROR;
        }

        pTxEnt = &Bzdma_Manager.TxEntSta[tx_id];
        pTxEnt->le_pdu_cmd = le_pdu_type;

        /* init tx descriptors */
        pTxEnt->pTxDesc[0].DWord0 = 0;
        pTxEnt->pTxDesc[0].DWord1 = 0;
        pTxEnt->pTxDesc[1].DWord0 = 0;
        pTxEnt->pTxDesc[1].DWord1 = 0;

        /* prepare bzdma txcmd for non-empty data of advertising pdu */
        if (adv->adv_len > 0)
        {
            pTxEnt->pTxDesc[cur_seg_idx].len = adv->adv_len;
            pTxEnt->pTxDesc[cur_seg_idx].start_addr = (UINT32)adv->adv_buf;
            cur_seg_idx++;
        }

        /* prepare bzdma txcmd for non-empty data of scan response pdu */
        if (adv->scan_rsp_len > 0)
        {
            pTxEnt->pTxDesc[cur_seg_idx].len = adv->scan_rsp_len;
            pTxEnt->pTxDesc[cur_seg_idx].start_addr = (UINT32)adv->scan_rsp_buf;
            cur_seg_idx++;
        }

        /* set last flag */
        pTxEnt->pTxDesc[cur_seg_idx - 1].isLast = TRUE;

        /* send tx command */
        bzdma_send_txcmd(tx_id);
#endif
    }

    return BT_ERROR_OK;
}

/**************************************************************************
 * Function     : ll_driver_data_channel_schedule_bzdma
 *
 * Description  : This function is used to schedule a tx command of BZDMA for
 *                transmit burst LE data packets in data channel pdu (user data
 *                or LL control pdu).
 *
 * Parameters   : conn_entry: The connection entry id
 *
 * Returns      : BT_ERROR_OK or BT_ERROR
 *************************************************************************/
#ifdef USE_NEW_LE_SCHEDULER
UINT8 ll_driver_data_channel_schedule_bzdma(UINT8 conn_entry, UINT8 is_terminate)
{
    UINT8 pkt_num = 0;
    LL_CONN_HANDLE_UNIT *phandle = &ll_manager.conn_unit.handle[conn_entry];

    if (is_terminate == FALSE)
    {
        /* check if we need to pause (?) le acl data path */
        if (!phandle->pause_acl_tx)
        {
#if defined(_SCHEDULE_BLE_MISC_PACKET_) && defined(_NEW_BZDMA_FROM_V8_)
            ll_reassemble_misc_tx_pkt_from_acl_misc_pend_list(conn_entry,
                    phandle->data_len_updt.max_tx_size);
#endif
        }
    }

    LE_ACL_PKT_LIST_MANAGE *ctrl_pkt_list = &phandle->tx_pend_ctrl_pkt_list;
    LE_ACL_MISC_PKT_LIST_MANAGE *misc_pkt_list = &phandle->tx_pend_misc_pkt_list;

    if (ctrl_pkt_list->pktcnt == 0 && misc_pkt_list->pktcnt == 0)
    {
        return BT_ERROR_OK;
    }

    if (Bzdma_Manager.bmFreeBleTxEnt & (1 << BZDMA_LEONLY_TX_ENTRY_TYPE_DATA))
    {
        /* no enable bzdma tx command, quit now */
        return BT_ERROR;
    }

    /* schedule any pending llc pdu */
    while (ctrl_pkt_list->pktcnt > 0)
    {
        if (Bzdma_Manager.BleTxEntSta[conn_entry].free_segs == 0
                || le_sched_pkt_fifo_is_full(conn_entry))
        {
            /* no free space in the fifo, quit now */
            goto sched_end;
        }

        LL_CTRL_PDU_PAYLOAD *llc_pkt = ctrl_pkt_list->pCtrlHead;
        ctrl_pkt_list->pCtrlHead = llc_pkt->pNext;
        if (ctrl_pkt_list->pCtrlHead == NULL)
        {
            ctrl_pkt_list->pCtrlTail = NULL;
        }
        --ctrl_pkt_list->pktcnt;

        llc_pkt->pNext = NULL;
        le_sched_pkt_fifo_enqueue(conn_entry, LE_SCHED_PKT_TYPE_CTRL_PDU,
                llc_pkt);

        BZDMA_BLEONLY_TX_DESC_FRAGMENT TxFragCache = { .DWord = 0 };
        TxFragCache.start_addr = (UINT32) llc_pkt;
        if (llc_pkt->OpCode > LL_CTRL_PDU_MAX)
        {
            LL_LOG_TRACE(RED, LE_MSG_SEND_INVALID_LLC_PDU_TYPE, 3,
                            pkt_num, llc_pkt, llc_pkt->OpCode);
        }
        TxFragCache.len = ll_ctrl_pdu_length[llc_pkt->OpCode];
        TxFragCache.isLast = TRUE;

        bzdma_send_packet_to_ble_data_ring_fifo(conn_entry, LL_LLID_CTRL_PDU,
                &TxFragCache, 1);
        pkt_num++;
    }

    /* schedule any pending acl packets */
#if defined(_SCHEDULE_BLE_MISC_PACKET_) && defined(_NEW_BZDMA_FROM_V8_)
    while (misc_pkt_list->pktcnt > 0)
    {
        if (Bzdma_Manager.BleTxEntSta[conn_entry].free_segs == 0
                || le_sched_pkt_fifo_is_full(conn_entry))
        {
            /* no free space in the fifo, quit now */
            goto sched_end;
        }

        UINT8 misc_pkt_id = misc_pkt_list->MiscHeadId;
        misc_pkt_list->MiscHeadId = le_tx_misc_node[misc_pkt_id].NextNodeId;
        if (misc_pkt_list->MiscHeadId == LL_MISC_ACL_DATA_PKT_MAX_NODES)
        {
            misc_pkt_list->MiscTailId = LL_MISC_ACL_DATA_PKT_MAX_NODES;
        }
        --misc_pkt_list->pktcnt;

        le_tx_misc_node[misc_pkt_id].NextNodeId = LL_MISC_ACL_DATA_PKT_MAX_NODES;
        le_sched_pkt_fifo_enqueue(conn_entry, LE_SCHED_PKT_TYPE_MISC_PKT,
                &le_tx_misc_node[misc_pkt_id]);

        UINT8 llid;
        if (le_tx_misc_node[misc_pkt_id].is1stFrag)
        {
            llid = LL_LLID_DATA_PDU_START;
        }
        else
        {
            llid = LL_LLID_DATA_PDU_CONT;
        }

        bzdma_send_packet_to_ble_data_ring_fifo(conn_entry, llid,
                le_tx_misc_node[misc_pkt_id].pFrags,
                le_tx_misc_node[misc_pkt_id].FragCount);
        pkt_num++;
    }
#endif /* _SCHEDULE_BLE_MISC_PACKET_ && _NEW_BZDMA_FROM_V8_ */

sched_end:
    if (pkt_num > 0)
    {
        /* accumulate scheduled tx packet counts */
        phandle->tx_sched_pkts_one_ce += pkt_num;
    }

    return BT_ERROR_OK;
}
#else /* ! USE_NEW_LE_SCHEDULER */
UINT8 ll_driver_data_channel_schedule_bzdma(UINT8 conn_entry, UINT8 is_terminate)
{
    LL_CONN_HANDLE_UNIT *phandle;
    LE_ACL_PKT_LIST_MANAGE *pctrl_sched_list;
    LL_CTRL_PDU_PAYLOAD *pllc_pkt_tmp;
#ifndef _SCHEDULE_BLE_MISC_PACKET_
    LE_ACL_PKT_LIST_MANAGE *pdata_sched_list;
    LL_HCI_ACL_DATA_PKT *pdata_pkt_tmp;
    LL_HCI_ACL_DATA_PKT *resent_pkt;
#endif
#ifndef _NEW_BZDMA_FROM_V8_
    BZDMA_TX_DESC_SEGMENT *pTxDesc;
    BZDMA_TX_DESC_SEGMENT TxDescCache;
    UINT8 tx_id;
    UINT8 tx_entry_type = BZDMA_TX_ENTRY_TYPE_LE_DATA_BURST0;
#else
    BZDMA_BLEONLY_TX_DESC_FRAGMENT TxFragCache[1];
    UINT8 llid;
#endif
    UINT8 pkt_num = 0;

    phandle = &ll_manager.conn_unit.handle[conn_entry];

    if (is_terminate == FALSE)
    {
        /* append pending ctrl pdu to schedule list */
        llc_append_pdu_list_to_list(&phandle->tx_pend_ctrl_pkt_list,
                                    conn_entry, LL_LINKED_LIST_INSERT_POLICY_TAIL,
                                    LL_LINKED_LIST_TYPE_TX_SCHED);

        /* init tx_pend_ctrl_pkt_list */
        phandle->tx_pend_ctrl_pkt_list.pCtrlHead = NULL;
        phandle->tx_pend_ctrl_pkt_list.pCtrlTail = NULL;
        phandle->tx_pend_ctrl_pkt_list.pktcnt = 0;

        /* check if we need to pause (?) le acl data path */
        if (!phandle->pause_acl_tx)
        {
#if defined(_SCHEDULE_BLE_MISC_PACKET_) && defined(_NEW_BZDMA_FROM_V8_)
#ifdef _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_
            UINT8 max_tx_size = phandle->data_len_updt.max_tx_size;
#else
            UINT8 max_tx_size = 27;
#endif
            ll_reassemble_misc_tx_pkt_from_acl_misc_pend_list(conn_entry, max_tx_size);

            /* append pending le acl misc data pkts to schedule list */
            ll_append_acl_tx_misc_pkt_list_to_list(&phandle->tx_pend_misc_pkt_list,
                                              conn_entry, LL_LINKED_LIST_INSERT_POLICY_TAIL,
                                              LL_LINKED_LIST_TYPE_TX_SCHED);

            /* init tx_pend_misc_pkt_list */
            phandle->tx_pend_misc_pkt_list.DWord = LL_MISC_ACL_PKT_LIST_RESET_VALUE;
#endif

#ifndef _SCHEDULE_BLE_MISC_PACKET_
            /* append pending le acl data pkts to schedule list */
            ll_append_acl_tx_pkt_list_to_list(&phandle->tx_pend_data_pkt_list,
                                              conn_entry, LL_LINKED_LIST_INSERT_POLICY_TAIL,
                                              LL_LINKED_LIST_TYPE_TX_SCHED);

            /* init tx_pend_data_pkt_list */
            phandle->tx_pend_data_pkt_list.pDataHead = NULL;
            phandle->tx_pend_data_pkt_list.pDataTail = NULL;
            phandle->tx_pend_data_pkt_list.pktcnt = 0;
#endif
        }
    }

    pctrl_sched_list = &phandle->tx_sched_ctrl_pkt_list;
    pllc_pkt_tmp = pctrl_sched_list->pCtrlHead;
#ifndef _SCHEDULE_BLE_MISC_PACKET_
    pdata_sched_list = &phandle->tx_sched_data_pkt_list;
    pdata_pkt_tmp = pdata_sched_list->pDataHead;
    resent_pkt = phandle->tx_resent_data_pkt;
#endif

#if defined(_SCHEDULE_BLE_MISC_PACKET_) && defined(_NEW_BZDMA_FROM_V8_)
    LE_ACL_MISC_PKT_LIST_MANAGE *pdata_sched_misc_list;
    LL_MISC_ACL_DATA_PKT_NODE *present_node = phandle->tx_resent_misc_pkt;
    UINT8 misc_pkt_id_tmp;

    pdata_sched_misc_list = &phandle->tx_sched_misc_pkt_list;
    misc_pkt_id_tmp = pdata_sched_misc_list->MiscHeadId;

    if ((pctrl_sched_list->pktcnt == 0)
#ifdef _SCHEDULE_BLE_MISC_PACKET_
            && (present_node == NULL) && (pdata_sched_misc_list->pktcnt == 0)
#else
            && (resent_pkt == NULL) && (pdata_sched_list->pktcnt == 0)
#endif
            )
    {
        return BT_ERROR_OK;
    }
#else
    if ((pctrl_sched_list->pktcnt == 0) &&
        (pdata_sched_list->pktcnt == 0) &&
        (resent_pkt == NULL))
    {
        return BT_ERROR_OK;
    }
#endif

#ifndef _NEW_BZDMA_FROM_V8_
    do
    {
        if (!IS_TOGGLE_LL_DATA_CMD)
        {
            break;
        }

#ifdef _ROM_CODE_PATCHED_
        if (rcp_ll_data_cmd_decision != NULL)
        {
            tx_entry_type = rcp_ll_data_cmd_decision(&conn_entry);
            break;
        }
#endif

        /* get free tx entry */
        if (ll_manager.event_manager.used_txcmd_cnt == 2)
        {
            return BT_ERROR;
        }
        else
        {
            if (ll_manager.event_manager.used_txcmd_cnt == 1)
            {
                /* update current used tx command */
                ll_manager.event_manager.cur_txcmd_id =
                           !ll_manager.event_manager.cur_txcmd_id;
            }
            tx_entry_type = BZDMA_TX_ENTRY_TYPE_LE_DATA_BURST0 +
                           ll_manager.event_manager.cur_txcmd_id;
        }
    }
    while (0);

    tx_id = bzdma_get_dedicated_free_tx_entry(tx_entry_type);
    if (tx_id == BZDMA_TX_DESC_ENTRY_NUM)
    {
        /* TODO: Need to schedule it to queue if this case is occured */
        //LL_LOG_TRACE(RED, LE_MSG_NO_FREE_TXCMD_OF_LE_DATA_TYPE, 0, 0);
        return BT_ERROR;
    }

    /* update the history of connection entry to use tx command */
    ll_manager.event_manager.pre_tx_entry = ll_manager.event_manager.cur_tx_entry;
    ll_manager.event_manager.cur_tx_entry = conn_entry;
    ll_manager.event_manager.used_txcmd_cnt++;

    pTxDesc = &Bzdma_Manager.TxEntSta[tx_id].pTxDesc[0];

    /* schedule previous unacked packet */
    if (resent_pkt != NULL)
    {
        TxDescCache.DWord0 = 0;
        TxDescCache.DWord1 = 0;
        TxDescCache.start_addr = (UINT32)resent_pkt->hci_acl_data_pkt;
        TxDescCache.len = resent_pkt->acl_data_total_length;
        if (resent_pkt->packet_boundary_flag == L_CH_L2CAP_NON_FLUSH)
        {
            TxDescCache.llid = LL_LLID_DATA_PDU_START;
        }
        else
        {
            TxDescCache.llid = LL_LLID_DATA_PDU_CONT;
        }
        pTxDesc[pkt_num].DWord0 = TxDescCache.DWord0;
        pTxDesc[pkt_num].DWord1 = TxDescCache.DWord1;
        pkt_num++;
    }

    /* schedule any pending llc pdu */
    while (pllc_pkt_tmp != NULL)
    {
        if (pkt_num == BZDMA_TX_ENTRY_MAX_SEGS_LE_DATA_BURST0)
        {
            /* the maximum tx pkt count we can support */
            break;
        }

        TxDescCache.DWord0 = 0;
        TxDescCache.DWord1 = 0;
        TxDescCache.start_addr = (UINT32)pllc_pkt_tmp;
        if (pllc_pkt_tmp->OpCode > LL_CTRL_PDU_MAX)
        {
            LL_LOG_TRACE(RED, LE_MSG_SEND_INVALID_LLC_PDU_TYPE, 3,
                            pkt_num, pllc_pkt_tmp, pllc_pkt_tmp->OpCode);
        }
        TxDescCache.len = ll_ctrl_pdu_length[pllc_pkt_tmp->OpCode];
        TxDescCache.llid = LL_LLID_CTRL_PDU;

        pTxDesc[pkt_num].DWord0 = TxDescCache.DWord0;
        pTxDesc[pkt_num].DWord1 = TxDescCache.DWord1;
        pllc_pkt_tmp = pllc_pkt_tmp->pNext;
        pkt_num++;
    }

    /* schedule any pending acl packets */
    while (pdata_pkt_tmp != NULL)
    {
        if (pkt_num == BZDMA_TX_ENTRY_MAX_SEGS_LE_DATA_BURST0)
        {
            /* the maximum tx pkt count we can support */
            break;
        }

        TxDescCache.DWord0 = 0;
        TxDescCache.DWord1 = 0;
        TxDescCache.start_addr = (UINT32)pdata_pkt_tmp->hci_acl_data_pkt;
        TxDescCache.len = pdata_pkt_tmp->acl_data_total_length;

        if (pdata_pkt_tmp->packet_boundary_flag == L_CH_L2CAP_NON_FLUSH)
        {
            TxDescCache.llid = LL_LLID_DATA_PDU_START;
        }
        else
        {
            TxDescCache.llid = LL_LLID_DATA_PDU_CONT;
        }

        pTxDesc[pkt_num].DWord0 = TxDescCache.DWord0;
        pTxDesc[pkt_num].DWord1 = TxDescCache.DWord1;

        pdata_pkt_tmp = pdata_pkt_tmp->next;
        pkt_num++;
    }

    if (pkt_num > 0)
    {
        pTxDesc[pkt_num - 1].isLast = TRUE;

        /* accumulate scheduled tx packet counts */
        phandle->tx_sched_pkts_one_ce += pkt_num;

        /* send tx command */
        Bzdma_Manager.TxEntSta[tx_id].le_conn_entry = conn_entry;
        bzdma_send_txcmd(tx_id);
    }
#else
    if (Bzdma_Manager.bmFreeBleTxEnt & (1 << BZDMA_LEONLY_TX_ENTRY_TYPE_DATA))
    {
        /* no enable bzdma tx command, quit now */
        return BT_ERROR;
    }

    /* schedule previous unacked packet (misc packet) */
#if defined(_SCHEDULE_BLE_MISC_PACKET_) && defined(_NEW_BZDMA_FROM_V8_)
    if (present_node != NULL)
    {
        if (present_node->is1stFrag)
        {
            llid = LL_LLID_DATA_PDU_START;
        }
        else
        {
            llid = LL_LLID_DATA_PDU_CONT;
        }
        if (!bzdma_send_packet_to_ble_data_ring_fifo(conn_entry, llid,
                present_node->pFrags, present_node->FragCount))
        {
            goto end;
        }
        pkt_num++;
    }
#endif

#ifndef _SCHEDULE_BLE_MISC_PACKET_
    /* schedule previous unacked packet */
    if (resent_pkt != NULL)
    {
        TxFragCache[0].DWord = 0;
        TxFragCache[0].start_addr = (UINT32)resent_pkt->hci_acl_data_pkt;
        TxFragCache[0].len = resent_pkt->acl_data_total_length;
        TxFragCache[0].isLast = TRUE;

        if (resent_pkt->packet_boundary_flag == L_CH_L2CAP_NON_FLUSH)
        {
            llid = LL_LLID_DATA_PDU_START;
        }
        else
        {
            llid = LL_LLID_DATA_PDU_CONT;
        }
        if (!bzdma_send_packet_to_ble_data_ring_fifo(conn_entry, llid,
                TxFragCache, 1))
        {
            goto end;
        }
        pkt_num++;
    }
#endif

    /* schedule any pending llc pdu */
    while (pllc_pkt_tmp != NULL)
    {
        TxFragCache[0].DWord = 0;
        TxFragCache[0].start_addr = (UINT32)pllc_pkt_tmp;
        if (pllc_pkt_tmp->OpCode > LL_CTRL_PDU_MAX)
        {
            LL_LOG_TRACE(RED, LE_MSG_SEND_INVALID_LLC_PDU_TYPE, 3,
                            pkt_num, pllc_pkt_tmp, pllc_pkt_tmp->OpCode);
        }
        TxFragCache[0].len = ll_ctrl_pdu_length[pllc_pkt_tmp->OpCode];
        TxFragCache[0].isLast = TRUE;
        llid = LL_LLID_CTRL_PDU;

        if (!bzdma_send_packet_to_ble_data_ring_fifo(conn_entry, llid,
                TxFragCache, 1))
        {
            goto end;
        }

        pllc_pkt_tmp = pllc_pkt_tmp->pNext;
        pkt_num++;
    }

    /* schedule any pending acl packets */
#if defined(_SCHEDULE_BLE_MISC_PACKET_) && defined(_NEW_BZDMA_FROM_V8_)
    while (misc_pkt_id_tmp != LL_MISC_ACL_DATA_PKT_MAX_NODES)
    {
#ifdef _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_TRACE_
        int i;
        LL_MISC_ACL_DATA_PKT_NODE * const pkt = &le_tx_misc_node[misc_pkt_id_tmp];
        RT_BT_LOG(YELLOW, YL_DBG_HEX_7, 7, 0x14102400, misc_pkt_id_tmp,
                pkt->MyNodeId, pkt->NextNodeId, pkt->FragCount, pkt->Src, pkt->is1stFrag);
        for (i = 0; i < pkt->FragCount; ++i)
        {
            UCHAR *data = (UCHAR *)(pkt->pFrags[i].start_addr | LL_BUFF_RAM_BASE);
            RT_BT_LOG(YELLOW, YL_DBG_HEX_5, 5, 0x14102402, data[0], data[1], data[2], data[3]);
        }
#endif

        if (le_tx_misc_node[misc_pkt_id_tmp].is1stFrag)
        {
            llid = LL_LLID_DATA_PDU_START;
        }
        else
        {
            llid = LL_LLID_DATA_PDU_CONT;
        }
        if (!bzdma_send_packet_to_ble_data_ring_fifo(conn_entry, llid,
                le_tx_misc_node[misc_pkt_id_tmp].pFrags,
                le_tx_misc_node[misc_pkt_id_tmp].FragCount))
        {
            goto end;
        }

        misc_pkt_id_tmp = le_tx_misc_node[misc_pkt_id_tmp].NextNodeId;

        pkt_num++;
    }
#ifdef _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_TRACE_
    RT_BT_LOG(YELLOW, YL_DBG_DEC_2, 2, 14102401, pkt_num);
#endif
#endif /* _SCHEDULE_BLE_MISC_PACKET_ && _NEW_BZDMA_FROM_V8_ */

#ifndef _SCHEDULE_BLE_MISC_PACKET_
    while (pdata_pkt_tmp != NULL)
    {
        TxFragCache[0].DWord = 0;
        TxFragCache[0].start_addr = (UINT32)pdata_pkt_tmp->hci_acl_data_pkt;
        TxFragCache[0].len = pdata_pkt_tmp->acl_data_total_length;
        TxFragCache[0].isLast = TRUE;

        if (pdata_pkt_tmp->packet_boundary_flag == L_CH_L2CAP_NON_FLUSH)
        {
            llid = LL_LLID_DATA_PDU_START;
        }
        else
        {
            llid = LL_LLID_DATA_PDU_CONT;
        }
        if (!bzdma_send_packet_to_ble_data_ring_fifo(conn_entry, llid,
                TxFragCache, 1))
        {
            goto end;
        }

        pdata_pkt_tmp = pdata_pkt_tmp->next;
        pkt_num++;
    }
#endif

end:
    if (pkt_num > 0)
    {
        /* accumulate scheduled tx packet counts */
        phandle->tx_sched_pkts_one_ce += pkt_num;
    }
#endif
    return BT_ERROR_OK;
}
#endif /* USE_NEW_LE_SCHEDULER */


/**************************************************************************
 * Function     : ll_driver_enable_advertising
 *
 * Description  : This function is used to enable an advertising block command.
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
/*
 AE Start                                            AE End      AE Start
    |  adv_ch(0) |        adv_ch(1)        |  adv_ch(2) |    |   | adv_ch(0)
    T---+        T---+ R-------+ T-------+ T---+        |    |   T---+ +-----+
    |Adv|        |Adv| |ScanReq| |ScanRsp| |Adv|        |    |   |Adv| |ConnR|
----+---+--------+---+-+-------+-+-------+-+---+--------+----+\\\+---+-+-----+
    |<--     --->|                         |            |    |   |
    adv_h2h_period                                           |   |
    |                                                        |   |
    |<-----------------   adv_interval    ------------------>|<->|
                                                            adv_delay(0~10ms)
*/
#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_LE_
void ll_driver_enable_advertising(UCHAR in_lps)
#else
void ll_driver_enable_advertising(void)
#endif
{
    LL_ADVERTISER_UNIT *adv = &ll_manager.adv_unit;
    UINT8 le_pdu_type = 0;
    UINT16 adv_h2h_period;

    if (adv->adv_len > 0)
    {
        le_pdu_type += BZDMA_LE_PDU_TYPE_ADV_DATA_ONLY;
    }

    switch (adv->pdu_type)
    {
    case LL_ADV_PDU_TYPE_ADV_IND:
    case LL_ADV_PDU_TYPE_ADV_DISCOVER_IND:
        if (adv->scan_rsp_len > 0)
        {
            le_pdu_type += BZDMA_LE_PDU_TYPE_SCAN_DATA_ONLY;
        }
        break;

    case LL_ADV_PDU_TYPE_ADV_DIRECT_IND:
    case LL_ADV_PDU_TYPE_ADV_NONCONN_IND:
        break;

    default:
        LL_LOG_TRACE(RED, LE_MSG_ERR_ADV_PDU_TYPE, 1, adv->pdu_type);
        return;
    }

    /* schedule any non-zero advertising data via TX command of BZDMA */
    if (BT_ERROR == ll_driver_advertising_schedule_bzdma(le_pdu_type))
    {
        return;
    }

    /* ---------------------------------------------------------*/
    /*  Set Instruction Parameters and Fire Block Command       */
    /*----------------------------------------------------------*/
    LE_REG_S_SET reg;

    /* set adv_h2h_period (unit: 625us), adv_chm, filter policy, packet type */
    adv_h2h_period = ll_fw_compute_advertiser_h2h_min_duration();
    adv_h2h_period = LL_DRIVER_TRANSLATE_625US_UNIT(adv_h2h_period);
#ifdef _NEW_BLE_HW_SPEC_FROM_150320_
    if (!ll_manager.adv_unit.h2h_period_en)
    {
        adv_h2h_period += 1;
        /* Warning!!!!! FW should set adv_h2h_period >=3 when type != 3(non-connectable)
           when h2h_period_en is set to 0. Jira BTFWHW-71. */
        if (adv_h2h_period < 3)
        {
            adv_h2h_period = 3;
        }
    }
#endif

    reg.value = RD_LE_REG(LE_REG_ADV_CONTROL);
    reg.adv_ctrl.adv_h2h_period = adv_h2h_period;
    reg.adv_ctrl.adv_chm = adv->ch_map;
    reg.adv_ctrl.filter_conn_req = adv->filter_conn_req;
    reg.adv_ctrl.filter_scan_req = adv->filter_scan_req;
    reg.adv_ctrl.pkt_type = adv->pdu_type;
    WR_LE_REG(LE_REG_ADV_CONTROL, reg.value);

    /* set local address type */
    reg.value = RD_LE_REG(LE_REG_DEVA_TYPE);
#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_

    /* own addr type: 0 => public, 1 => random,
     *                2 => RPA, otherwise public,
     *                3 => RPA, otherwise random
     */
    reg.deva_type.local_type_adv = adv->local_addr_type % 2;

    if (adv->local_addr_type > LL_ADDR_TYPE_RANDOM)
    {
        UPDATE_LE_REG(LE_REG_PRIVACY_MISC0, BIT14, BIT14);
    }

#else
    reg.deva_type.local_type_adv = adv->local_addr_type;
#endif /* _BT4_2_PRIVACY_FEATURE_SUPPORT_ */

#ifdef _DAPE_LOW_DUTY_ADV
    if (adv->hci_pdu_type != LL_HCI_ADV_TYPE_ADV_DIRECT_IND)
#else
    if (adv->pdu_type != LL_ADV_PDU_TYPE_ADV_DIRECT_IND)
#endif
    {
        /* set adv_interval */
        adv->interval = adv->interval_min;
        WR_LE_REG(LE_REG_ADV_INTERVAL, adv->interval);

        /* TODO: choose a better value of advertising interval
                (current set minimum value) - austin */
    }
#ifndef _DAPE_LOW_DUTY_ADV
    else
#else
    if (adv->pdu_type == LL_ADV_PDU_TYPE_ADV_DIRECT_IND || adv->local_addr_type > LL_ADDR_TYPE_RANDOM)
#endif
    {
        /* set remote address and address type for direct adv pdu */
        reg.deva_type.remote_type = adv->direct_addr_type;
        WR_LE_REG(LE_REG_DEVA_REMOTE_L, adv->u2dir_addr[0]);
        WR_LE_REG(LE_REG_DEVA_REMOTE_M, adv->u2dir_addr[1]);
        WR_LE_REG(LE_REG_DEVA_REMOTE_H, adv->u2dir_addr[2]);
#ifdef _DAPE_LOW_DUTY_ADV
        if (adv->hci_pdu_type != LL_HCI_ADV_TYPE_ADV_LOW_DUTY_DIRECT_IND)
#endif
        {
        /* set the timeout value of direct advertising pdu */
        WR_LE_REG(LE_REG_STATUS_ADV_TIMEOUT_CTRL, LL_DIRECT_ADV_TIMEOUT_MAX);
        }
        /* TODO: maybe choose a better value of direct advertising timeout
                (current set maximum value 1.28 sec) - austin */
    }

    WR_LE_REG(LE_REG_DEVA_TYPE, reg.value);

    if ((adv->pdu_type == LL_ADV_PDU_TYPE_ADV_DIRECT_IND) ||
       (adv->pdu_type == LL_ADV_PDU_TYPE_ADV_IND))
    {
        /* update tx win size offset to 0 */
        reg.value = RD_LE_REG(LE_REG_CONN_UPD_WIN_SIZE);
        reg.conn_upd_win_size.conn_win_size_offset = 0;
        WR_LE_REG(LE_REG_CONN_UPD_WIN_SIZE, reg.value);

        /* dape added. set ce_length back to minimum*/
        WR_LE_REG(LE_REG_CONN_CE_LEN, LL_DEFAULT_CE_LENGTH);

        /* disable slave latency */
        reg.value = RD_LE_REG(LE_REG_SLAVE_WIN_WIDENING_L);
        reg.slave_win_widen_l.sub_en = FALSE;
        WR_LE_REG(LE_REG_SLAVE_WIN_WIDENING_L, reg.value);

#ifdef _NEW_BZDMA_FROM_V8_
        ll_driver_write_bzdma_rptr_to_cam(0, 0);
        Bzdma_Manager.BleTxEntSta[0].seg_rptr = 0;
        Bzdma_Manager.BleTxEntSta[0].seg_wptr = 0;
#ifdef _NEW_BZDMA_FROM_V8_PROGRAMMABLE_CONFIG_SETTING_
        Bzdma_Manager.BleTxEntSta[0].free_segs = bzdma_supported_le_max_seg_num;
#else
        Bzdma_Manager.BleTxEntSta[0].free_segs = BZDMA_LEONLY_TX_ONE_ENTRY_MAX_SEGS;
#endif
#endif
    }
#ifdef _DAPE_LOW_DUTY_ADV
    reg.value = RD_LE_REG(LE_REG_CBK_CONTROL);
    if (adv->hci_pdu_type == LL_HCI_ADV_TYPE_ADV_LOW_DUTY_DIRECT_IND)
    {
        reg.cbk_ctrl.low_duty_adv_en = TRUE;
    }
    else
    {
        reg.cbk_ctrl.low_duty_adv_en = FALSE;
    }
    WR_LE_REG(LE_REG_CBK_CONTROL, reg.value);
#endif

    /* fire advertising request */
    ll_driver_set_le_instruction(LE_HW_INSTRUCTION_ADV_REQ);

#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_
    ll_driver_store_tx_resolvable_private_address();
#endif

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_LE_
    if(!in_lps)
#endif
    {
        LL_LOG_TRACE(WHITE, LE_MSG_EN_ADV_MODE, 10, adv->adv_len,
                    adv->hci_pdu_type, adv_h2h_period,
                    adv->interval, adv->direct_addr[0], adv->direct_addr[1],
                    adv->direct_addr[2], adv->direct_addr[3],
                    adv->direct_addr[4], adv->direct_addr[5]);

    }
}

/**************************************************************************
 * Function     : ll_driver_disable_advertising
 *
 * Description  : This function is used to disable an advertising procedure.
 *                Then we need to wait cancel completion interrupt to check the
 *                reason. (Maybe the FSM of HW enters connection state of slave
 *                role)
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
void ll_driver_disable_advertising_imp(void)
{
    /* fire advertising cancel instruction */
    ll_driver_set_le_instruction(LE_HW_INSTRUCTION_ADV_CANCEL);
}
void (*ll_driver_disable_advertising)(void) = ll_driver_disable_advertising_imp;

/**************************************************************************
 * Function     : ll_driver_enable_scanning
 *
 * Description  : This function is used to enable a scanning block command.
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
/*
                                    Active Scan
  |           scan_ch[N]          |              |         scan_ch[N+1]
  | R---+ T-------+ R-------+     |   R---+      | R---+ T-------+ R-------+
  | |Adv| |ScanReq| |ScanRsp|...  |   |Adv|...   | |Adv| |ScanReq| |ScanRsp|...
--+-+---+-+-------+-+-------+-----+---+---+------+-+---+-+-------+-+-------+--
  |<----     scan_window      --->|              |<-------  scan_window   ---
  |<----     scan_interval    ------------------>|<-------  scan_interval ---

                                    Passive Scan
  |           scan_ch[N]          |              |         scan_ch[N+1]
  | R---+                         |   R---+      | R---+
  | |Adv|...                      |   |Adv|...   | |Adv|...
--+-+---+-------------------------+---+---+------+-+---+----------------------
  |<----     scan_window      --->|              |<-------  scan_window   ---
  |<----     scan_interval    ------------------>|<-------  scan_interval ---
*/
#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_LE_
void ll_driver_enable_scanning(UCHAR in_lps)
#else
void ll_driver_enable_scanning(void)
#endif
{
    LL_SCANNER_UNIT *scan = &ll_manager.scan_unit;
    LE_REG_S_SET reg;

    /* set scan interval */
    WR_LE_REG(LE_REG_SCAN_INTERVAL, scan->interval);

    /* set scan window */
    WR_LE_REG(LE_REG_SCAN_WINDOW, scan->window);

    reg.value = RD_LE_REG(LE_REG_DEVA_TYPE);
#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_

    /* own addr type: 0 => public, 1 => random,
     *                2 => RPA, otherwise public,
     *                3 => RPA, otherwise random
     */
    reg.deva_type.local_type_scan = scan->local_addr_type % 2;

#else
    reg.deva_type.local_type_scan = scan->local_addr_type;
#endif /* _BT4_2_PRIVACY_FEATURE_SUPPORT_ */

#ifdef _USE_NEW_BLE_SCANNER_OPTION_TO_RX_DIR_IND_
    reg.deva_type.scan_rx_dir_ind_opt = 1; //option 1
#endif
    WR_LE_REG(LE_REG_DEVA_TYPE, reg.value);

    /* set duplicate filter, scan type, scan channel map, scan filter policy */
    reg.value = RD_LE_REG(LE_REG_SCAN_CONTROL);
    reg.scan_ctrl.scan_pri_ctrl = TRUE;
    reg.scan_ctrl.dup_filter = scan->filter_duplicate;
    reg.scan_ctrl.active_scan = scan->active_scan;
#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_LE_
    if(in_lps)
    {
        while(1)
        {
            scan->ch_bit_in_use ++;

            scan->ch_bit_in_use = (scan->ch_bit_in_use < 3)? (scan->ch_bit_in_use):0;

            if((scan->ch_map)&(1<<(scan->ch_bit_in_use)))
            {
               break;
            }
        }

        reg.scan_ctrl.scan_chm = 1<<(scan->ch_bit_in_use);
    }
    else
#endif
    {
        reg.scan_ctrl.scan_chm = scan->ch_map;
    }

#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_

    /* scanner filter policy:
     * 0x0 and 0x2 => white list disable
     * 0x1 and 0x3 => white list enable
     */
    reg.scan_ctrl.scan_filter = scan->filter_policy % 2;

#else
    reg.scan_ctrl.scan_filter = scan->filter_policy;
#endif
#ifdef _DAPE_TEST_FIX_LE_SCAN_CH
    reg.scan_ctrl.scan_chm = 1;
#endif
    WR_LE_REG(LE_REG_SCAN_CONTROL, reg.value);

#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_

    if (scan->filter_policy > 0x1)
    {
        reg.value = RD_LE_REG(LE_REG_PRIVACY_MISC0);
        reg.le_privacy_misc0.ext_scan_filter_policy = 1;
        WR_LE_REG(LE_REG_PRIVACY_MISC0, reg.value);
    }

    if (scan->local_addr_type > LL_ADDR_TYPE_RANDOM)
    {
        reg.value = RD_LE_REG(LE_REG_PRIVACY_MISC0);
        reg.le_privacy_misc0.tx_lreso_adr_sel = 1;
        WR_LE_REG(LE_REG_PRIVACY_MISC0, reg.value);
    }

#endif /* _BT4_2_PRIVACY_FEATURE_SUPPORT_ */

    /* fire scanning request */
    ll_driver_set_le_instruction(LE_HW_INSTRUCTION_SCAN_REQ);
}

/**************************************************************************
 * Function     : ll_driver_disable_scanning
 *
 * Description  : This function is used to disable a scanning procedure.
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
void ll_driver_disable_scanning(void)
{
    /* fire scan cancel instruction */
    ll_driver_set_le_instruction(LE_HW_INSTRUCTION_SCAN_CANCEL);
}

/**************************************************************************
 * Function     : ll_driver_create_connection
 *
 * Description  : This function is used to send a connection_req pdu in the
 *                advertising channel after enable a block command to create
 *                a LE piconet connection. HW will send a connection_req to
 *                the dedicated advertiser from initiating state to enter
 *                connection state of master role.
 *
 * Parameters   : conn_entry: the entry index for create connection
 *
 * Returns      : None
 *
 *************************************************************************/
/*                                  conn_win_size
                                   |<----------->|
    R---+   T-------+   |          | T----+      |             T----+
    |Adv|   |ConnReq|   |          | | Tx |      |             | Tx |
----+---+---+-------+---+----------+-+----+------+------+------+----+---
                    |<->|<-------->| |<--  ce_length  ->|      |
                  1.25ms conn_win_offset                       |
                                     |<--  conn_interval    -->|

 [payload part of connection request]
 +--------+--------+----+-------+-------------+---------------+------------+
 | InitA  |  AdvA  | AA |CRCInit|   WinSize   |  WinOffset    |  Interval  |
 |  (6B)  |  (6B)  |(4B)|  (3B) |    (1B)     |    (2B)       |    (2B)    |...
 |lbd_addr|rbd_addr| AA |       |conn_win_size|conn_win_offset| ce_interval|
 +--------+--------+----+-------+-------------+---------------+------------+
   +-------------------+--------------+------+----------------+---------+
   |      Latency      |      TO      | ChM  |      Hop       |   SCA   |
 ..|       (2B)        |     (2B)     | (5B) |    (5bits)     | (3bits) |
   |slave_conn_latency |supervision_to|ch_map| Hop_incremental|local_sca|
   +-------------------+--------------+------+----------------+---------+
*/
void ll_driver_create_connection(UINT8 conn_entry)
{
    LL_INITIATOR_UNIT *initiator = &ll_manager.initiator_unit;
    LE_REG_S_SET reg;
    UINT16 random16;

    /* set scan_interval */
    WR_LE_REG(LE_REG_CONN_INTERVAL, initiator->scan_interval);

    /* set scan_windows */
    WR_LE_REG(LE_REG_CONN_WINDOW, initiator->scan_window);

    /* set filter policy */
    reg.value = 0;
    reg.cbk_ctrl.init_filt = initiator->filter_policy;
    reg.cbk_ctrl.adv_tx_power = LE_DEAULT_ADV_TX_POWER;
    WR_LE_REG(LE_REG_CBK_CONTROL, reg.value);

    /* set 32-bit access address */
    WR_LE_REG(LE_REG_CONN_AA_L, initiator->access_address & 0xFFFF);
    WR_LE_REG(LE_REG_CONN_AA_H, initiator->access_address >> 16);

    /* set conn_win_size */
    WR_LE_REG(LE_REG_CONN_WIN_SIZE, initiator->tx_win_size);

    /* set conn_win_offset */
    WR_LE_REG(LE_REG_CONN_WIN_OFFSET, initiator->tx_win_offset);

    /* set conn_interval */
    WR_LE_REG(LE_REG_CONN_CE_INTERVAL, initiator->conn_interval);

    /* set ce_length */
    WR_LE_REG(LE_REG_CONN_CE_LEN, initiator->ce_len);

    /* set slave_conn_latency and advertising channel map */
    reg.conn_latency.rsvd = 0;
    reg.conn_latency.slave_latency = initiator->conn_latency;
    reg.conn_latency.conn_req_chm = initiator->ch_map;
    WR_LE_REG(LE_REG_CONN_LATENCY, reg.value);

    /* set supervision timeout */
    WR_LE_REG(LE_REG_CONN_SUPERVISION_TO, initiator->supervision_to);

    /* set channel map (ch 0~31) */
    WR_LE_REG(LE_REG_CONN_CH_MAP_L, initiator->conn_ch_map[0] |
                                   (initiator->conn_ch_map[1] << 8));
    WR_LE_REG(LE_REG_CONN_CH_MAP_M, initiator->conn_ch_map[2] |
                                   (initiator->conn_ch_map[3] << 8));

    /* generate hop_incremental (5 ~ 16) */
    LL_DRIVER_GEN_16BIT_RANDOM(random16);
    random16 = random16 % 12;
    random16 += 5;
    ll_manager.initiator_unit.hop_increment = random16;

    /* set channel map (ch 32~36), hop_incremental, conn_entry */
    reg.conn_ch_map_h.ch_map = initiator->conn_ch_map[4];
    reg.conn_ch_map_h.hop_inc = random16;
    reg.conn_ch_map_h.conn_entry = conn_entry;
    WR_LE_REG(LE_REG_CONN_CH_MAP_H, reg.value);

    /* set master SCA */
    reg.value = RD_LE_REG(LE_REG_SLAVE_CH_MAP_H);
    reg.slave_ch_map_h.local_sca = initiator->masterSCA;
    WR_LE_REG(LE_REG_SLAVE_CH_MAP_H, reg.value);

    /* set local address type, peer address type */
    reg.value = RD_LE_REG(LE_REG_DEVA_TYPE);
#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_

    /* addr type:
     * 0 => public, 1 => random, 2 => RPA, otherwise public, 3 => RPA, otherwise random
     */
    reg.deva_type.local_type_init = initiator->local_addr_type % 2;
    reg.deva_type.remote_type = initiator->remote_addr_type % 2;

    if (initiator->local_addr_type > LL_ADDR_TYPE_RANDOM)
    {
        UPDATE_LE_REG(LE_REG_PRIVACY_MISC0, BIT14, BIT14);
    }

#else
    reg.deva_type.local_type_init = initiator->local_addr_type;
    reg.deva_type.remote_type = initiator->remote_addr_type;
#endif /* _BT4_2_PRIVACY_FEATURE_SUPPORT_ */
    WR_LE_REG(LE_REG_DEVA_TYPE, reg.value);

    /* set peer address */
    WR_LE_REG(LE_REG_DEVA_REMOTE_L, initiator->u2rem_addr[0]);
    WR_LE_REG(LE_REG_DEVA_REMOTE_M, initiator->u2rem_addr[1]);
    WR_LE_REG(LE_REG_DEVA_REMOTE_H, initiator->u2rem_addr[2]);

    /* update tx win size offset to LL_CONN_TX_WIN_SIZE_OFFSET */
    reg.value = RD_LE_REG(LE_REG_CONN_UPD_WIN_SIZE);
    reg.conn_upd_win_size.conn_win_size_offset = LL_CONN_TX_WIN_SIZE_OFFSET;
    WR_LE_REG(LE_REG_CONN_UPD_WIN_SIZE, reg.value);

#ifdef _NEW_BZDMA_FROM_V8_
    ll_driver_write_bzdma_rptr_to_cam(conn_entry, 0);
    Bzdma_Manager.BleTxEntSta[conn_entry].seg_rptr = 0;
    Bzdma_Manager.BleTxEntSta[conn_entry].seg_wptr = 0;
#ifdef _NEW_BZDMA_FROM_V8_PROGRAMMABLE_CONFIG_SETTING_
    Bzdma_Manager.BleTxEntSta[conn_entry].free_segs = bzdma_supported_le_max_seg_num;
#else
    Bzdma_Manager.BleTxEntSta[conn_entry].free_segs = BZDMA_LEONLY_TX_ONE_ENTRY_MAX_SEGS;
#endif
#endif

    /* fire connection request instruction */
    ll_driver_set_le_instruction(LE_HW_INSTRUCTION_CONN_REQ);

    RT_BT_LOG(GREEN, DAPE_TEST_LOG231, 15,
          conn_entry, initiator->access_address, initiator->tx_win_size,
          initiator->tx_win_offset, initiator->conn_interval, initiator->conn_latency,
          initiator->supervision_to, initiator->conn_ch_map[0], initiator->conn_ch_map[1],
          initiator->conn_ch_map[2], initiator->conn_ch_map[3], initiator->conn_ch_map[4],
          random16, initiator->masterSCA, ll_manager.initiator_unit.hop_increment);
}

/**************************************************************************
 * Function     : ll_driver_create_connection_cancel
 *
 * Description  : This function is used to cancel a connection_req task if
 *                HW does not send it to the air. If HW already sends it to
 *                the air, this command can become instruction fail because
 *                HW will enter connection state from initiating state.
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
void ll_driver_create_connection_cancel(void)
{
    LL_INITIATOR_UNIT *initiator = &ll_manager.initiator_unit;

    if (initiator->enable)
    {
#if 0
        if (initiator->entry_id > LL_MAX_CONNECTION_ENTRY_ID)
        {
            LL_LOG_TRACE(RED, LE_MSG_INVALID_CONN_ENTRY, 1, initiator->entry_id);

            /* do not return, but FW needs to check it
               if this case is triggered */
        }
#endif

        /* fire connection request instruction */
        ll_driver_set_le_instruction(LE_HW_INSTRUCTION_CONN_CANCEL);
    }
}

#ifdef LE_ISSUE_INSTR_IN_ORDER_ON_COMPLT
typedef enum LE_INSTR_QUEUE_TYPE_
{
    LE_INSTR_QUEUE_CONN_KILL,
    LE_INSTR_QUEUE_TYPE_NUM
} LE_INSTR_QUEUE_TYPE;

LE_INSTR_ENTITY le_instr_queue_buf[LE_INSTR_QUEUE_TYPE_NUM][LL_MAX_CONNECTION_UNITS];

ARRAY_QUEUE_DEFINE_TYPE(LE_INSTR_ENTITY, LE_INSTR_QUEUE);
LE_INSTR_QUEUE le_instr_queue[LE_INSTR_QUEUE_TYPE_NUM];
UINT32 le_instr_pending_req_map[LE_INSTR_QUEUE_TYPE_NUM];

void le_instr_queue_init()
{
    int i;
    for (i = 0; i < LE_INSTR_QUEUE_TYPE_NUM; ++i)
    {
        ARRAY_QUEUE_INIT(&le_instr_queue[i], le_instr_queue_buf[i],
                LL_MAX_CONNECTION_UNITS);
    }
    memset(le_instr_pending_req_map, 0, sizeof (le_instr_pending_req_map));
}

static inline BOOLEAN le_instr_queue_is_empty(LE_INSTR_QUEUE_TYPE type)
{
    return ARRAY_QUEUE_IS_EMPTY(&le_instr_queue[type]);
}

static inline LE_INSTR_ENTITY le_instr_queue_front(LE_INSTR_QUEUE_TYPE type)
{
    return ARRAY_QUEUE_FRONT(&le_instr_queue[type]);
}

static inline BOOLEAN le_instr_queue_has_pending_req(LE_INSTR_QUEUE_TYPE type,
        UINT8 conn_entry)
{
    return (le_instr_pending_req_map[type] & (1 << conn_entry));
}

static inline void le_instr_queue_set_pending_req(LE_INSTR_QUEUE_TYPE type,
        UINT8 conn_entry)
{
    le_instr_pending_req_map[type] |= (1 << conn_entry);
}

static inline void le_instr_queue_clear_pending_req(LE_INSTR_QUEUE_TYPE type,
        UINT8 conn_entry)
{
    le_instr_pending_req_map[type] &= ~(1 << conn_entry);
}

void le_instr_queue_enqueue(LE_INSTR_QUEUE_TYPE type, LE_INSTR_ENTITY instr)
{
    if (!le_instr_queue_has_pending_req(type, instr.conn.conn_entry))
    {
        ARRAY_QUEUE_ENQUEUE(&le_instr_queue[type], instr);
        le_instr_queue_set_pending_req(type, instr.conn.conn_entry);
    }
}

LE_INSTR_ENTITY le_instr_queue_dequeue(LE_INSTR_QUEUE_TYPE type)
{
    LE_INSTR_ENTITY instr = { .conn = { .conn_entry = -1 } };
    ARRAY_QUEUE_DEQUEUE(&le_instr_queue[type], &instr);
    if (instr.conn.conn_entry >= 0)
    {
        le_instr_queue_clear_pending_req(type, instr.conn.conn_entry);
    }
    return instr;
}
#endif /* LE_ISSUE_INSTR_IN_ORDER_ON_COMPLT */

/**************************************************************************
 * Function     : ll_driver_kill_connection
 *
 * Description  : This function is used to kill an connection entry. This
 *                action will kill the state machine of HW and any connection
 *                information in local side after this command is completed.
 *
 * Parameters   : conn_entry_id: connection entry id
 *
 * Returns      : None
 *
 *************************************************************************/
void ll_driver_issue_kill_connection(UINT8 conn_entry_id)
{
    LE_REG_S_SET reg;

    if (conn_entry_id > LL_MAX_CONNECTION_ENTRY_ID)
    {
        //LL_LOG_TRACE(RED, LE_MSG_INVALID_CONN_ENTRY, 1, conn_entry_id);
        return;
    }

    /* set conn_entry */
    reg.value = RD_LE_REG(LE_REG_CANCEL_ENTRY);
    reg.cancel_entry.kill_entry = conn_entry_id;
    WR_LE_REG(LE_REG_CANCEL_ENTRY, reg.value);

    /* fire connection kill instruction */
    ll_driver_set_le_instruction(LE_HW_INSTRUCTION_CONN_KILL);
}

#ifdef LE_ISSUE_INSTR_IN_ORDER_ON_COMPLT
LE_INSTR_ENTITY ll_driver_issue_next_kill_connection()
{
    DEF_CRITICAL_SECTION_STORAGE;
    LE_INSTR_ENTITY instr;
    LE_INSTR_ENTITY next_instr = { .conn = { .conn_entry = -1 } };

    MINT_OS_ENTER_CRITICAL();
    instr = le_instr_queue_dequeue(LE_INSTR_QUEUE_CONN_KILL);
    if (!le_instr_queue_is_empty(LE_INSTR_QUEUE_CONN_KILL))
    {
        next_instr = le_instr_queue_front(LE_INSTR_QUEUE_CONN_KILL);
    }
    MINT_OS_EXIT_CRITICAL();
    if (next_instr.conn.conn_entry >= 0)
    {
        ll_driver_issue_kill_connection(next_instr.conn.conn_entry);
    }
    return instr;
}

void ll_driver_kill_connection_imp(UINT8 conn_entry_id)
{
    DEF_CRITICAL_SECTION_STORAGE;
    BOOLEAN need_issue_instr;
    LE_INSTR_ENTITY instr = { .conn = { .conn_entry = conn_entry_id } };

    MINT_OS_ENTER_CRITICAL();
    need_issue_instr = le_instr_queue_is_empty(LE_INSTR_QUEUE_CONN_KILL);
    le_instr_queue_enqueue(LE_INSTR_QUEUE_CONN_KILL, instr);
    MINT_OS_EXIT_CRITICAL();
    if (need_issue_instr)
    {
        ll_driver_issue_kill_connection(conn_entry_id);
    }
}
void (*ll_driver_kill_connection)(UINT8) = ll_driver_kill_connection_imp;
#else /* ! LE_ISSUE_INSTR_IN_ORDER_ON_COMPLT */
void (*ll_driver_kill_connection)(UINT8) = ll_driver_issue_kill_connection;
#endif /* LE_ISSUE_INSTR_IN_ORDER_ON_COMPLT */

/**************************************************************************
 * Function     : ll_driver_disconnect_connection
 *
 * Description  : This function is used to request HW to send a LL_TERMINATE_IND
 *                for remote device in connection state after enable a block
 *                command for disconnect the connection. This action will kill
 *                the state machine of HW and any connection information in
 *                local side after this command is completed when HW receives
 *                remote device's Acknowledgement.
 *                (If no acknowledgement is received until the terminate timer
 *                 is expired, FW need to kill the connection)
 *
 * Parameters   : conn_entry_id: connection entry id
 *
 * Returns      : None
 *
 *************************************************************************/
void ll_driver_issue_disconnect_connection(UINT8 conn_entry_id, UINT8 reason)
{
    LE_REG_S_SET reg;

    if (conn_entry_id > LL_MAX_CONNECTION_ENTRY_ID)
    {
        //LL_LOG_TRACE(RED, LE_MSG_INVALID_CONN_ENTRY, 1, conn_entry_id);
        return;
    }

    /* set conn_entry */
    reg.value = RD_LE_REG(LE_REG_CANCEL_ENTRY);
    reg.cancel_entry.disconn_entry = conn_entry_id;
    WR_LE_REG(LE_REG_CANCEL_ENTRY, reg.value);

    /* set terminate reason */
    reg.value = RD_LE_REG(LE_REG_STATUS_TERMINATE_REASON);
    reg.status_term_reason.local_term_reason = reason;
    WR_LE_REG(LE_REG_STATUS_TERMINATE_REASON, reg.value);

    /* fire connection disconnet instruction */
    ll_driver_set_le_instruction(LE_HW_INSTRUCTION_DISCONN_REQ);
}
void (*ll_driver_disconnect_connection)(UINT8, UINT8) = ll_driver_issue_disconnect_connection;

/**************************************************************************
 * Function     : ll_driver_connection_update
 *
 * Description  : This function is used to enable a block command to HW for
 *                update connection timing with assigned connection entry.
 *                This functiona shall be used after we transmit a
 *                LL_CONNECTION_UPDATE_REQ then receive the acknowledgement
 *                from remote device, or we receive a LL_CONNECTION_UPDATE_REQ
 *                then response an acknowledgement to remote device.
 *
 * Parameters   : conn_entry_id: connection entry id
 *                instant: the instant when start the connection
 *                         update procedure
 *
 * Returns      : None
 *
 *************************************************************************/
/*                                  conn_win_size
                                   |<----------->|
 +---+   +---+          |          | +----+      |              +---+  +---+
 |m2s|   |s2m|          |          | |m2s |      |              |m2s|  |s2m|
-+---+---+---+----------+----------+-+----+------+------+-------+---+--+---+--
                        |<-------->| |<---- ce_len ---->|       |
                        | conn_win_offset                       |
 <- conn_interval_old ->|            |<--  conn_interval_new -->|

                    [payload part of connection update request]
    +--------+--------------+---------------+----------+-------------------+
    | Opcode |   WinSize    |  WinOffset    | Interval |      Latency      |
    |  (1B)  |    (1B)      |    (2B)       |   (2B)   |       (2B)        |...
    |  0x00  |conn_win_size |conn_win_offset| conn_inv |slave_conn_latency |
    +--------+--------------+---------------+----------+-------------------+
      +--------------+-----------+
      |      TO      |  Instant  |
    ..|     (2B)     |   (2B)    |
      |supervision_to|upd_instant|
      +--------------+-----------+
*/
void ll_driver_issue_connection_update(UINT8 conn_entry_id, UINT32 instant)
{
    LL_CONNECTION_UNIT *conn = &ll_manager.conn_unit;
    LL_CONN_HANDLE_UNIT *ll_handle;
    LL_CONN_UPDT_BLK *updt_blk;
    LE_REG_S_SET reg;

    if (conn_entry_id > LL_MAX_CONNECTION_ENTRY_ID)
    {
        //LL_LOG_TRACE(RED, LE_MSG_INVALID_CONN_ENTRY, 1, conn_entry_id);
        return;
    }

    ll_handle = &conn->handle[conn_entry_id];

    if (ll_handle->connected)
    {
        if (!conn->master)
        {
            /* disable slave latency */
            reg.value = RD_LE_REG(LE_REG_SLAVE_WIN_WIDENING_L);
            reg.slave_win_widen_l.sub_en = FALSE;
            WR_LE_REG(LE_REG_SLAVE_WIN_WIDENING_L, reg.value);
        }

        updt_blk = &ll_handle->conn_updt_blk;

        /* set conn_win_size */
        // dape added
        reg.value = RD_LE_REG(LE_REG_CONN_UPD_WIN_SIZE);
        reg.conn_upd_win_size.conn_win_size = updt_blk->tx_win_size;
        if (conn->master)
        {
#if defined (_DAPE_GET_SLOT_FOR_LE_UPDT) || defined (_DAPE_KEEP_SLOT_OFST_SAME_WHEN_CONN_UPDT)
            reg.conn_upd_win_size.conn_win_size_offset = updt_blk->tx_win_size_ofst;
#else
            reg.conn_upd_win_size.conn_win_size_offset = 0;
#endif
        }
        else
        {
            reg.conn_upd_win_size.conn_win_size_offset = 0;
        }
        WR_LE_REG(LE_REG_CONN_UPD_WIN_SIZE, reg.value);

        /* set conn_win_offset */
        WR_LE_REG(LE_REG_CONN_UPD_WIN_OFFSET, updt_blk->tx_win_offset);

        /* set conn_inv */
        WR_LE_REG(LE_REG_CONN_UPD_CE_INTERVAL, updt_blk->ce_interval);

        /* set ce_len */
        WR_LE_REG(LE_REG_CONN_UPD_CE_LEN, updt_blk->ce_length);

        /* set slave_conn_latency */
        WR_LE_REG(LE_REG_CONN_UPD_LATENCY, updt_blk->slave_latency);

        /* set supervision_to */
        WR_LE_REG(LE_REG_CONN_UPD_SUPERVISION_TO, updt_blk->supervision_to);

        /* set upt_instant */
        WR_LE_REG(LE_REG_CONN_UPD_INSTANCE, instant);

        /* set conn_upt_ent */
        reg.value = RD_LE_REG(LE_REG_CONN_UPD_ENTRY);
        reg.conn_upd_entry.updt_ce_minus_one = TRUE;
        reg.conn_upd_entry.conn_upd_entry = conn_entry_id;
        WR_LE_REG(LE_REG_CONN_UPD_ENTRY, reg.value);

        /* fire connection update instruction */
        ll_driver_set_le_instruction(LE_HW_INSTRUCTION_CONN_UPDATE);
#ifdef _DAPE_PRINT_MORE_LOG_INFO
        LL_LOG_TRACE(YELLOW, DAPE_TEST_LOG542, 3,
                            conn_entry_id, RD_LE_REG(LE_REG_CONN_UPD_WIN_SIZE),
                            RD_LE_REG(LE_REG_CONN_UPD_WIN_OFFSET));
#endif

    }
    else
    {
        LL_LOG_TRACE(YELLOW, LE_MSG_SET_UNCONNECTED_CONN_ENTRY, 2,
                            conn_entry_id, LE_HW_INSTRUCTION_CONN_UPDATE);
    }
}
void (*ll_driver_connection_update)(UINT8, UINT32) = ll_driver_issue_connection_update;

/**************************************************************************
 * Function     : ll_driver_channel_map_update
 *
 * Description  : This function is used to enable a block command to HW for
 *                update connection timing with assigned connection entry.
 *                This functiona shall be used after we transmit a
 *                LL_CHANNEL_UPDATE_REQ then receive the acknowledgement
 *                from remote device, or we receive a LL_CHANNEL_UPDATE_REQ
 *                then response an acknowledgement to remote device.
 *
 * Parameters   : conn_entry_id: connection entry id
 *                instant: the instant when start the channel map
 *                         update procedure
 *
 * Returns      : None
 *
 *************************************************************************/
/*
  [payload part of channel map update request]
    +--------+---------+-----------+
    | Opcode |   ChM   |  Instant  |
    |  (1B)  |   (5B)  |   (2B)    |
    |  0x00  |  ch_map |upd_instant|
    +--------+---------+-----------+
*/
void ll_driver_issue_channel_map_update(UINT8 conn_entry_id, UINT32 instant)
{
    LL_CONNECTION_UNIT *conn = &ll_manager.conn_unit;
    LE_REG_S_SET reg;

    if (conn_entry_id > LL_MAX_CONNECTION_ENTRY_ID)
    {
        //LL_LOG_TRACE(RED, LE_MSG_INVALID_CONN_ENTRY, 1, conn_entry_id);
        return;
    }

    if (conn->handle[conn_entry_id].connected)
    {
        if (!conn->master)
        {
            /* disable slave latency */
            reg.value = RD_LE_REG(LE_REG_SLAVE_WIN_WIDENING_L);
            reg.slave_win_widen_l.sub_en = FALSE;
            WR_LE_REG(LE_REG_SLAVE_WIN_WIDENING_L, reg.value);
        }

        /* set upt_instant */
        WR_LE_REG(LE_REG_CH_UPD_INSTANT, instant);

        /* fill channel update map to HW register */
        WR_LE_REG(LE_REG_CH_UPD_MAP_L,
                                ll_manager.conn_unit.updt_ch_map[0] |
                               (ll_manager.conn_unit.updt_ch_map[1] << 8));
        WR_LE_REG(LE_REG_CH_UPD_MAP_M,
                                ll_manager.conn_unit.updt_ch_map[2] |
                               (ll_manager.conn_unit.updt_ch_map[3] << 8));

        /* fill channel update map and set ch_upt_ent */
        reg.value = RD_LE_REG(LE_REG_CH_UPD_MAP_H);
        reg.ch_upd_map_h.ch_upd_entry = conn_entry_id;
        reg.ch_upd_map_h.ch_map = ll_manager.conn_unit.updt_ch_map[4];
        WR_LE_REG(LE_REG_CH_UPD_MAP_H, reg.value);

        /* fire connection update instruction */
        ll_driver_set_le_instruction(LE_HW_INSTRUCTION_CHANNEL_UPDATE);
    }
    else
    {
        LL_LOG_TRACE(YELLOW, LE_MSG_SET_UNCONNECTED_CONN_ENTRY, 2,
                            conn_entry_id, LE_HW_INSTRUCTION_CHANNEL_UPDATE);
    }
}
void (*ll_driver_channel_map_update)(UINT8, UINT32) = ll_driver_issue_channel_map_update;


UINT8 ll_dev_addr_list_center_add_list_entry(UINT8 list_type, UINT8 addr_type,
        UINT8 *addr)
{
    LL_DEV_ADDR_LIST_CENTER *list;
    UINT8 max_list_size;
    switch (list_type)
    {
    case LL_WHITE_LIST_TYPE:
        list = &ll_manager.white_list;
        max_list_size = LL_MAX_WHITE_LIST_SIZE;
        break;
    case LL_BLACK_LIST_TYPE:
        list = &ll_manager.black_list;
        max_list_size = LL_MAX_BLACK_LIST_SIZE;
        break;
    default:
        return -1;
    }

    UINT8 free_head = list->free_list;
    if (free_head != max_list_size)
    {
        /* cut one entry from free list */
        list->free_list = list->entry[free_head].next;

        /* init entry content */
        list->entry[free_head].DWord[0] = 0;
        list->entry[free_head].DWord[1] = 0;
        list->entry[free_head].next = max_list_size;

        /* insert current free device entry to the tail of the white list */
        if (list->list_head == max_list_size)
        {
            list->list_head = free_head;
        }
        else
        {
            list->entry[list->list_tail].next = free_head;
        }
        list->list_tail = free_head;

        list->entry[free_head].valid = TRUE;
        list->entry[free_head].duplicated = (list_type == LL_BLACK_LIST_TYPE);

        /* update device address and address type to white entry */
        list->entry[free_head].type = addr_type;
        memcpy(list->entry[free_head].addr, addr, 6);
    }
    return free_head;
}

#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_
UINT8 ll_dev_addr_list_center_add_resolve_list_entry(UINT8 addr_type,
        UINT8 *addr, UINT8 *local_irk, UINT8 *peer_irk)
{
    LL_DEV_ADDR_LIST_CENTER *list = &ll_manager.resolving_list;
    UINT8 free_head = list->free_list;
    if (free_head != LL_MAX_RESOLVING_LIST_SIZE)
    {
        /* cut one entry from free list */
        list->free_list = list->item[free_head].next;

        /* init entry content */
        memset(list->item[free_head].DWord, 0, 10 * sizeof(UINT32));
        list->item[free_head].next = LL_MAX_RESOLVING_LIST_SIZE;

        /* insert current free device entry to the tail of the resolving list */
        if (list->list_head == LL_MAX_RESOLVING_LIST_SIZE)
        {
            list->list_head = free_head;
        }
        else
        {
            list->item[list->list_tail].next = free_head;
        }
        list->list_tail = free_head;

        /* update device address and address type to white entry */
        list->item[free_head].type = addr_type;
        memcpy(list->item[free_head].addr, addr, 6);

        /* update Local IRK and Peer IRK */
        memcpy(list->item[free_head].Local_IRK, local_irk, 16);
        memcpy(list->item[free_head].Peer_IRK, peer_irk, 16);

        /* set valid bit of each irk and entire entry */
        UINT32 zero_irk[4] = {0,0,0,0};
        list->item[free_head].valid_local_IRK = (memcmp(zero_irk, local_irk, 16) != 0);
        list->item[free_head].valid_peer_IRK = (memcmp(zero_irk, peer_irk, 16) != 0);
        list->item[free_head].valid = TRUE;
    }
    return free_head;
}
#endif

/**************************************************************************
 * Function     : ll_driver_add_dev_to_list
 *
 * Description  : This function is used to add a device unit to white list, black list
 *                or resolving list. FW can record the device unit and list information
 *                then enable one entry and fill new address information in the
 *                CAM of HW if the list is not full.
 *
 * Parameters   : list_type: LL_WHITE_LIST_TYPE, LL_BLACK_LIST_TYPE or LL_RESOLVING_LIST_TYPE
 *                addr_type: the type of address (public or random)
 *                addr:      the pointer of device address
 *                local_irk: Local IRK for resovling list used
 *                peer_irk:  Peer IRK for resovling list used
 *
 * Returns      : device unit index or LL_MAX_XXX_LIST_SIZE
 *
 *************************************************************************/
#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_
UINT8 ll_driver_add_dev_to_list(UINT8 list_type, UINT8 addr_type, UINT8 *addr, UINT8 *local_irk, UINT8 *peer_irk)
#else
UINT8 ll_driver_add_dev_to_list(UINT8 list_type,UINT8 addr_type,UINT8 * addr)
#endif
{
    LL_DEV_ADDR_LIST_CENTER *pglist = NULL;
    UINT8 free_head;
    UINT8 max_list_size = 0;
    UINT16 cam_addr = 0;

    switch (list_type)
    {
        case LL_WHITE_LIST_TYPE:
            pglist = &ll_manager.white_list;
            free_head = ll_dev_addr_list_center_add_list_entry(LL_WHITE_LIST_TYPE, addr_type, addr);
            max_list_size = LL_MAX_WHITE_LIST_SIZE;
            cam_addr = LE_CAM_WHITE_LIST_BASE;
            break;
        case LL_BLACK_LIST_TYPE:
            pglist = &ll_manager.black_list;
            free_head = ll_dev_addr_list_center_add_list_entry(LL_BLACK_LIST_TYPE, addr_type, addr);
            max_list_size = LL_MAX_BLACK_LIST_SIZE;
            cam_addr = LE_CAM_BLACK_LIST_BASE;
            break;

#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_
        case LL_RESOLVING_LIST_TYPE:
            pglist = &ll_manager.resolving_list;
            free_head = ll_dev_addr_list_center_add_resolve_list_entry(addr_type, addr, local_irk, peer_irk);
            max_list_size = LL_MAX_RESOLVING_LIST_SIZE;
            cam_addr = LE_CAM_RESOLVING_LIST_BASE;
            break;
#endif

        default:
            return -1;
    }

    if (free_head == max_list_size)
    {
        LL_LOG_TRACE(RED, LE_MSG_NO_FREE_DEV_UNIT_ADD_TO_LIST, 8, list_type,
            addr_type, addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

        return max_list_size;
    }

    LL_LOG_TRACE(WHITE, LE_MSG_ADD_DEV_LIST_TYPE, 9, free_head, list_type, addr_type, addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_
    if (list_type == LL_RESOLVING_LIST_TYPE)
    {
        LL_RESOLVING_LIST_ENTRY *entry = &pglist->item[free_head];
        /* set hw register for valid bit */
        UINT8 shift = free_head / 16, idx = free_head % 16;
        UPDATE_LE_REG(LE_REG_LOCAL_IRK_VALID_HF(shift), BIT(idx), entry->valid_local_IRK ? BIT(idx) : 0x0);
        UPDATE_LE_REG(LE_REG_PEER_IRK_VALID_HF(shift), BIT(idx), entry->valid_peer_IRK ? BIT(idx) : 0x0);
        UPDATE_LE_REG(LE_REG_PEER_ADDR_VALID_HF(shift), BIT(idx), BIT(idx));

#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_MSG_
        LL_LOG_TRACE(BLUE, DAPE_TEST_LOG213, 2, LE_REG_LOCAL_IRK_VALID_HF(shift), RD_LE_REG(LE_REG_LOCAL_IRK_VALID_HF(shift)));
        LL_LOG_TRACE(BLUE, DAPE_TEST_LOG213, 2, LE_REG_PEER_IRK_VALID_HF(shift), RD_LE_REG(LE_REG_PEER_IRK_VALID_HF(shift)));
        LL_LOG_TRACE(BLUE, DAPE_TEST_LOG213, 2, LE_REG_PEER_ADDR_VALID_HF(shift), RD_LE_REG(LE_REG_PEER_ADDR_VALID_HF(shift)));
#endif

        /* write resolving list unit content to cam */
        cam_addr += (free_head * LL_HW_RESOLVING_LIST_ENTRY_SIZE);
        int i;
        for (i = 0;i < LL_HW_RESOLVING_LIST_ENTRY_SIZE;i ++)
            ll_driver_write_cam(cam_addr + i, entry->DWord[i]);
    }
#endif /* _BT4_2_PRIVACY_FEATURE_SUPPORT_ */

    if (list_type == LL_WHITE_LIST_TYPE || list_type == LL_BLACK_LIST_TYPE)
    {
        LL_DEV_ADDR_LIST_ENTRY *entry = &pglist->entry[free_head];
        /* write white list unit content to cam */
        cam_addr += free_head << 1;
        ll_driver_write_cam(cam_addr, entry->DWord[0]);
        ll_driver_write_cam(cam_addr + 1, entry->DWord[1]);

#ifdef LE_HW_TEST
        {
            LL_DEV_ADDR_LIST_ENTRY addr_entry_tmp;
            addr_entry_tmp.DWord[0] = ll_driver_read_cam(cam_addr);
            addr_entry_tmp.DWord[1] = ll_driver_read_cam(cam_addr + 1);

            LL_LOG_TRACE(GRAY, LE_MSG_CONFIRM_DEV_LIST, 6, list_type, cam_addr,
               entry->DWord[0], entry->DWord[1],
               addr_entry_tmp.DWord[0], addr_entry_tmp.DWord[1]);
        }
#endif
    }

    return free_head;
}

/**************************************************************************
 * Function     : ll_driver_remove_dev_from_list
 *
 * Description  : This function is used to remove a device unit to white list, black list
 *                or resolving list. FW can update the list information then disable
 *                one corresponding entry in the CAM of HW if the the address
 *                unit is found in the list.
 *
 * Parameters   : list_type: LL_WHITE_LIST_TYPE, LL_BLACK_LIST_TYPE or LL_RESOLVING_LIST_TYPE
 *                addr_type: the type of address (public or random)
 *                addr:      the pointer of device address
 *
 * Returns      : None
 *
 *************************************************************************/
#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_
BOOLEAN ll_driver_remove_dev_from_list(UINT8 list_type, UINT8 addr_type, UINT8 *addr)
#else
void ll_driver_remove_dev_from_list(UINT8 list_type,UINT8 addr_type,UINT8 * addr)
#endif
{
    UINT8 list_head;
    UINT8 list_tail;
    UINT8 node;
    UINT8 pre_node;
    UINT32 addr_buf_w[2] = {0, 0};
    LL_DEV_ADDR_LIST_CENTER *pglist = NULL;
    UINT8 max_list_size = 0;
    UINT16 cam_addr = 0;
    BOOLEAN is_found = FALSE;

    switch (list_type)
    {
        case LL_WHITE_LIST_TYPE:
            pglist = &ll_manager.white_list;
            max_list_size = LL_MAX_WHITE_LIST_SIZE;
            cam_addr = LE_CAM_WHITE_LIST_BASE;
            break;
        case LL_BLACK_LIST_TYPE:
            pglist = &ll_manager.black_list;
            max_list_size = LL_MAX_BLACK_LIST_SIZE;
            cam_addr = LE_CAM_BLACK_LIST_BASE;
            break;

#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_
        case LL_RESOLVING_LIST_TYPE:
            pglist = &ll_manager.resolving_list;
            max_list_size = LL_MAX_RESOLVING_LIST_SIZE;
            cam_addr = LE_CAM_RESOLVING_LIST_BASE;
            break;
#endif

        default:
            break;
    }

    list_head = pglist->list_head;
    list_tail = pglist->list_tail;
    pre_node = max_list_size;

    if (list_head == max_list_size)
    {
#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_
        return is_found;
#else
        return;
#endif
    }

    node = list_head;

    /* copy device address to local buffer for fast comparision */
    memcpy((UINT8*)addr_buf_w, addr, 6);

#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_

    if (list_type == LL_RESOLVING_LIST_TYPE)
    {
        while (node != max_list_size)
        {
            /* find matched device unit (type and address field) in the list */
            if ((pglist->item[node].type == addr_type) &&
                (pglist->item[node].DWord[8] == addr_buf_w[0]) &&
                ((pglist->item[node].DWord[9] & 0xFFFF) == addr_buf_w[1]))
            {
                if (node == list_head)
                {
                    pglist->list_head = pglist->item[node].next;
                }
                else
                {
                    /* link two nodes */
                    pglist->item[pre_node].next = pglist->item[node].next;
                }

                if (node == list_tail)
                {
                    pglist->list_tail = pre_node;
                }

                /* link back removed device entry to free list */
                pglist->item[node].next = pglist->free_list;
                pglist->free_list = node;

                /* let the entry be invalid */
                pglist->item[node].valid = FALSE;
                pglist->item[node].valid_local_IRK = FALSE;
                pglist->item[node].valid_peer_IRK = FALSE;

                /* remove device entry in the list from HW, set valid bitmap to 0 */
                UINT8 shift = node / 16, idx = node % 16;
                UPDATE_LE_REG(LE_REG_LOCAL_IRK_VALID_HF(shift), BIT(idx), 0x0);
                UPDATE_LE_REG(LE_REG_PEER_IRK_VALID_HF(shift), BIT(idx), 0x0);
                UPDATE_LE_REG(LE_REG_PEER_ADDR_VALID_HF(shift), BIT(idx), 0x0);
                is_found = TRUE;
                break;
            }
            pre_node = node;
            node = pglist->item[node].next;
        }
    }

#endif /* _BT4_2_PRIVACY_FEATURE_SUPPORT_ */

    if (list_type == LL_WHITE_LIST_TYPE || list_type == LL_BLACK_LIST_TYPE)
    {
        while (node != max_list_size)
        {
            /* find matched device unit (type and address field) in the list */
            if ((pglist->entry[node].type == addr_type) &&
                (pglist->entry[node].DWord[0] == addr_buf_w[0]) &&
                ((pglist->entry[node].DWord[1] & 0xFFFF) == addr_buf_w[1]))
            {
                if (node == list_head)
                {
                    pglist->list_head = pglist->entry[node].next;
                }
                else
                {
                    /* link two nodes */
                    pglist->entry[pre_node].next = pglist->entry[node].next;
                }

                if (node == list_tail)
                {
                    pglist->list_tail = pre_node;
                }

                /* link back removed device entry to free list */
                pglist->entry[node].next = pglist->free_list;
                pglist->free_list = node;

                /* let the entry be invalid */
                pglist->entry[node].valid = FALSE;
                pglist->entry[node].duplicated = FALSE;
                pglist->entry[node].gen_report = 0;

                /* remove device entry in the list from HW */
                cam_addr += node << 1;
                ll_driver_write_cam(cam_addr + 1, 0);
                is_found = TRUE;
                break;
            }
            pre_node = node;
            node = pglist->entry[node].next;
        }
    }

#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_
    return is_found;
#endif
}

/**************************************************************************
 * Function     : ll_driver_reset_dev_list
 *
 * Description  : This function is used to reset a white list or black list.
 *                FW can disable all used entries in the CAM of HW and initiate
 *                the white list or black list.
 *
 * Parameters   : list_type: LL_WHITE_LIST_TYPE, LL_BLACK_LIST_TYPE or LL_RESOLVING_LIST_TYPE
 *
 * Returns      : None
 *
 *************************************************************************/
void ll_driver_reset_dev_list(UINT8 list_type)
{
    LL_DEV_ADDR_LIST_CENTER *pglist = NULL;
    UINT8 max_list_size = 0;
    UINT16 cam_base = 0;
    UINT16 cam_addr = 0;
    UINT8 node;

    switch (list_type)
    {
        case LL_WHITE_LIST_TYPE:
            pglist = &ll_manager.white_list;
            max_list_size = LL_MAX_WHITE_LIST_SIZE;
            cam_base = LE_CAM_WHITE_LIST_BASE;
            break;

        case LL_BLACK_LIST_TYPE:
            pglist = &ll_manager.black_list;
            max_list_size = LL_MAX_BLACK_LIST_SIZE;
            cam_base = LE_CAM_BLACK_LIST_BASE;
            break;

#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_
        case LL_RESOLVING_LIST_TYPE:
            pglist = &ll_manager.resolving_list;
            max_list_size = LL_MAX_RESOLVING_LIST_SIZE;
            cam_base = LE_CAM_RESOLVING_LIST_BASE;
            break;
#endif

        default:
            break;
    }

    node = pglist->list_head;

#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_

    if (list_type == LL_RESOLVING_LIST_TYPE)
    {
        while (node != max_list_size)
        {
            /* let the entry be invalid */
            pglist->item[node].valid = FALSE;
            pglist->item[node].valid_local_IRK = FALSE;
            pglist->item[node].valid_peer_IRK = FALSE;

            /* let HW's list entry be invalid, set valid bitmap to 0 */
            WR_LE_REG(LE_REG_LOCAL_IRK_VALID_L, 0x0);
            WR_LE_REG(LE_REG_LOCAL_IRK_VALID_H, 0x0);
            WR_LE_REG(LE_REG_PEER_IRK_VALID_L, 0x0);
            WR_LE_REG(LE_REG_PEER_IRK_VALID_H, 0x0);
            WR_LE_REG(LE_REG_PEER_ADDR_VALID_L, 0x0);
            WR_LE_REG(LE_REG_PEER_ADDR_VALID_H, 0x0);

            /* check next node */
            node = pglist->item[node].next;
        }
    }

#endif /* _BT4_2_PRIVACY_FEATURE_SUPPORT_ */

    if (list_type == LL_WHITE_LIST_TYPE || list_type == LL_BLACK_LIST_TYPE)
    {
        while (node != max_list_size)
        {
            /* let the entry be invalid */
            pglist->entry[node].valid = FALSE;
            pglist->entry[node].duplicated = FALSE;
            pglist->entry[node].gen_report = 0;

            /* let HW's list entry be invalid */
            cam_addr = cam_base + (node << 1);
            ll_driver_write_cam(cam_addr, 0);
            ll_driver_write_cam(cam_addr + 1, 0);

            /* check next node */
            node = pglist->entry[node].next;
        }
    }

    if (pglist->list_head != max_list_size)
    {
        /* chain the head of the free list to the tail of the used list */
        if (list_type == LL_RESOLVING_LIST_TYPE)
            pglist->item[pglist->list_tail].next = pglist->free_list;
        else
            pglist->entry[pglist->list_tail].next = pglist->free_list;

        /* move the used list to the free list */
        pglist->free_list = pglist->list_head;

        /* init head and tail of the used list */
        pglist->list_head = max_list_size;
        pglist->list_tail = max_list_size;
    }
}

/**************************************************************************
 * Function     : ll_driver_search_dev_from_list
 *
 * Description  : This function is used to search a device unit from white list
 *                or black list.
 *
 * Parameters   : list_type: LL_WHITE_LIST_TYPE, LL_BLACK_LIST_TYPE or LL_RESOLVING_LIST_TYPE
 *                addr_type: the type of address (public or random)
 *                addr:      the pointer of device address
 *
 * Returns      : device address unit id
 *
 *************************************************************************/
UINT8 ll_driver_search_dev_from_list(UINT8 list_type, UINT8 addr_type, UINT8 *addr)
{
    LL_DEV_ADDR_LIST_CENTER *pglist = NULL;
    UINT8 max_list_size = 0;
    UINT8 node;
    UINT32 addr_buf_w[2] = {0, 0};

    /* copy device address to local buffer for fast comparision */
    memcpy((UINT8*)addr_buf_w, addr, 6);

    switch (list_type)
    {
        case LL_WHITE_LIST_TYPE:
            pglist = &ll_manager.white_list;
            max_list_size = LL_MAX_WHITE_LIST_SIZE;
            break;

        case LL_BLACK_LIST_TYPE:
            pglist = &ll_manager.black_list;
            max_list_size = LL_MAX_BLACK_LIST_SIZE;
            break;

#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_
        case LL_RESOLVING_LIST_TYPE:
            pglist = &ll_manager.resolving_list;
            max_list_size = LL_MAX_RESOLVING_LIST_SIZE;
            break;
#endif

        default:
            break;
    }

    node = pglist->list_head;

#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_

    if (list_type == LL_RESOLVING_LIST_TYPE)
    {
        while (node != max_list_size)
        {
            /* find matched device unit (type and address field) in the list */
            if ((pglist->item[node].type == addr_type) &&
                (pglist->item[node].DWord[8] == addr_buf_w[0]) &&
                ((pglist->item[node].DWord[9] & 0xFFFF) == addr_buf_w[1]))
            {
                break;
            }
            node = pglist->item[node].next;
        }
    }

#endif /* _BT4_2_PRIVACY_FEATURE_SUPPORT_ */

    if (list_type == LL_WHITE_LIST_TYPE || list_type == LL_BLACK_LIST_TYPE)
    {
        while (node != max_list_size)
        {
            /* find matched device unit (type and address field) in the list */
            if ((pglist->entry[node].type == addr_type) &&
                (pglist->entry[node].DWord[0] == addr_buf_w[0]) &&
                ((pglist->entry[node].DWord[1] & 0xFFFF) == addr_buf_w[1]))
            {
                break;
            }
            node = pglist->entry[node].next;
        }
    }

    return node;
}

/**************************************************************************
 * Function     : ll_driver_clear_all_duplicated_flags_in_list
 *
 * Description  : This function is used to clear all duplicated flags in the
 *                white list or black list. This operation can be used when
 *                we disable scan procedure after enable duplicate filter.
 *
 * Parameters   : list_type: LL_WHITE_LIST_TYPE or LL_BLACK_LIST_TYPE
 *
 * Returns      : None
 *
 *************************************************************************/
void ll_driver_clear_all_duplicated_flags_in_list(UINT8 list_type)
{
    LL_DEV_ADDR_LIST_CENTER *pglist;
    UINT8 node;
    UINT16 cam_addr;

    if (list_type == LL_BLACK_LIST_TYPE)
    {
        ll_driver_reset_dev_list(LL_BLACK_LIST_TYPE);
        return;
    }

    pglist = &ll_manager.white_list;
    node = pglist->list_head;

    while (node != LL_MAX_WHITE_LIST_SIZE)
    {
        if (pglist->entry[node].duplicated)
        {
            pglist->entry[node].duplicated = FALSE;
            pglist->entry[node].gen_report = 0;

            /* clear duplicated flag of HW's list entry to zero */
            cam_addr = LE_CAM_WHITE_LIST_BASE + (node << 1);
            ll_driver_write_cam(cam_addr + 1, pglist->entry[node].DWord[1]);
        }
        node = pglist->entry[node].next;
    }
}

/**************************************************************************
 * Function     : ll_driver_get_session_key
 *
 * Description  : This function is used to calculate the session key via
 *                128 bit key data (LTK) and 128 bit plain text (SKD) via HW
 *                encryption enginee.
 *
 * Parameters   : key: the pointer of 128 bit key data
 *                text: the pointer of 128 bit plain text
 *                session_key: the pointer of 128 bit calculated key data
 *
 * Returns      : None
 *
 *************************************************************************/
void ll_driver_get_session_key(UINT16 *key , UINT16 *text, UINT16 *session_key)
{
    UINT8 i;
    LE_REG_S_SET reg;

    for (i = 0; i < 8; i++)
    {
        WR_LE_REG(LE_REG_AES_ENC_KEY_HF(i), key[i]);
        WR_LE_REG(LE_REG_AES_ENC_DATA_HF(i), text[i]);
    }

    reg.value = RD_LE_REG(LE_REG_AES_FW_ENC_CTRL);
    reg.aes_fw_enc_ctrl.enc_start = 1;
    WR_LE_REG(LE_REG_AES_FW_ENC_CTRL, reg.value);

    while (1)
    {
        reg.value = RD_LE_REG(LE_REG_AES_FW_ENC_CTRL);
        if (reg.aes_fw_enc_ctrl.enc_done)
        {
            break;
        }
    }

    for (i = 0; i < 8; i++)
    {
        session_key[i] = RD_LE_REG(LE_REG_AES_ENC_DATA_HF(i));
    }

    LL_LOG_TRACE(YELLOW, LE_MSG_CHECK_SESSION_KEY_INPUT, 16,
             key[7],key[6],key[5],key[4],key[3],key[2],key[1],key[0],
             text[7],text[6],text[5],text[4],text[3],text[2],text[1],text[0]);

    LL_LOG_TRACE(YELLOW, LE_MSG_CHECK_SESSION_KEY_OUTPUT, 8,
             session_key[7], session_key[6], session_key[5], session_key[4],
             session_key[3], session_key[2], session_key[1], session_key[0]);
}

/**************************************************************************
 * Function     : ll_driver_install_encryption_info
 *
 * Description  : This function is used to install the 128 bit session key
 *                and 64 bit iv to HW encryption enginee.
 *
 * Parameters   : master_role: master or slave role in the connection state
 *                conn_entry: connection entry id (it is valid in the master)
 *                iv: the pointer of 64 bit initialization vector
 *                sess_key: the pointer of 128 bit session key data
 *
 * Returns      : None
 *
 *************************************************************************/
void ll_driver_install_encryption_info(UINT8 master_role, UINT8 conn_entry,
                                       UINT16 *iv, UINT16 *sess_key)
{
    UINT8 i;
    UINT16 addr_iv;
    UINT32_S temp;

#ifdef _CHECK_LE_CAM_ACCESS_
    UINT32 count = 0;

    while (1)
    {
        g_ll_driver_write_cam_warning_flag = 0;
#endif

#ifdef _DAPE_MOVE_LE_SLAVE_PARAMETER_TO_CAM
        if (!master_role)
        {
            /* hw always use entry 0 to store the information of slave link now */
            conn_entry = 0;
        }

        /* fill IV to the correct engry in the CAM */
        for (i = 0; i < 2; i++)
        {
            addr_iv = LE_CAM_ENTRY_BASE(conn_entry) + LE_CAM_ADDR_IV(i);
            temp.u2Byte[0] = iv[i << 1];
            temp.u2Byte[1] = iv[(i << 1) + 1];
            ll_driver_write_cam(addr_iv, temp.u4Byte);
        }

        /* fill Session Key to the correct engry in the CAM */
        for (i = 0; i < 4; i++)
        {
            addr_iv = LE_CAM_ENTRY_BASE(conn_entry) + LE_CAM_ADDR_AES_SES_KEY(i);
            temp.u2Byte[0] = sess_key[i << 1];
            temp.u2Byte[1] = sess_key[(i << 1) + 1];
            ll_driver_write_cam(addr_iv, temp.u4Byte);
        }

#else
        if (master_role)
        {
            /* fill IV to the correct engry in the CAM */
            for (i = 0; i < 2; i++)
            {
                addr_iv = LE_CAM_ENTRY_BASE(conn_entry) + LE_CAM_ADDR_IV(i);
                temp.u2Byte[0] = iv[i << 1];
                temp.u2Byte[1] = iv[(i << 1) + 1];
                ll_driver_write_cam(addr_iv, temp.u4Byte);
            }

            /* fill Session Key to the correct engry in the CAM */
            for (i = 0; i < 4; i++)
            {
                addr_iv = LE_CAM_ENTRY_BASE(conn_entry) + LE_CAM_ADDR_AES_SES_KEY(i);
                temp.u2Byte[0] = sess_key[i << 1];
                temp.u2Byte[1] = sess_key[(i << 1) + 1];
                ll_driver_write_cam(addr_iv, temp.u4Byte);
            }


        }
        else
        {
            /* fill IV to the slave registers */
            for (i = 0; i < 4; i++)
            {
                WR_LE_REG(LE_REG_SLAVE_IV(i), iv[i]);
            }

            /* fill Session Key to the slave registers */
            for (i = 0; i < 8; i++)
            {
                WR_LE_REG(LE_REG_SLAVE_SESSION_KEY(i), sess_key[i]);
            }
        }
#endif

#ifdef _CHECK_LE_CAM_ACCESS_
        if (g_ll_driver_write_cam_warning_flag == 0)
        {
            break;
        }

        if (count > 1000000)
        {
            RT_BT_LOG(RED, MSG_LL_INSTALL_KEY_FAILED, 0, 0);
            break;
        }
        count++;
    }
#endif

    LL_LOG_TRACE(YELLOW, LE_MSG_INSTALL_SESSION_KEY_1, 6,
             master_role, conn_entry, iv[3], iv[2], iv[1], iv[0]);

    LL_LOG_TRACE(YELLOW, LE_MSG_INSTALL_SESSION_KEY_2, 8,
             sess_key[7], sess_key[6], sess_key[5], sess_key[4],
             sess_key[3], sess_key[2], sess_key[1], sess_key[0]);
}

/**************************************************************************
 * Function     : ll_driver_set_random_address
 *
 * Description  : This function is used to set local random address
 *
 * Parameters  : None
 *
 * Returns       : None
 *
 *************************************************************************/
void ll_driver_set_random_address(void)
{
    WR_LE_REG(LE_REG_DEVA_RANDOM_LOCAL_L, ll_manager.u2local_random_addr[0]);
    WR_LE_REG(LE_REG_DEVA_RANDOM_LOCAL_M, ll_manager.u2local_random_addr[1]);
    WR_LE_REG(LE_REG_DEVA_RANDOM_LOCAL_H, ll_manager.u2local_random_addr[2]);

#if 0
    RT_BT_LOG(GREEN, DAPE_TEST_LOG259, 3, ll_manager.u2local_random_addr[2],
                                          ll_manager.u2local_random_addr[1],
                                          ll_manager.u2local_random_addr[0]);
#endif
}

#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_
/**************************************************************************
 * Function     : ll_driver_set_address_resolution_enable
 *
 * Description  : This function is used to enable / disable address resolution
 *
 * Parameters  : isEnable, 0: disable, 1: enable
 *
 * Returns       : None
 *
 *************************************************************************/
void ll_driver_set_address_resolution_enable(BOOLEAN isEnable)
{
    LE_REG_S_SET reg;
    reg.value = RD_LE_REG(LE_REG_PRIVACY_MISC0);
    reg.le_privacy_misc0.Addr_Resol_En = isEnable;
    reg.le_privacy_misc0.re_gen_rand = isEnable;
    reg.le_privacy_misc0.rand_num_sel_local = !isEnable;
    reg.le_privacy_misc0.rand_num_sel_peer = !isEnable;
    WR_LE_REG(LE_REG_PRIVACY_MISC0, reg.value);

    ll_manager.address_resolution_enable = isEnable;
}

/**************************************************************************
 * Function     : ll_driver_set_resolvable_address_regenerate
 *
 * Description  : This function is used to re-generate RPA
 *
 * Parameters  : None
 *
 * Returns       : None
 *
 *************************************************************************/
void ll_driver_set_resolvable_address_regenerate(void)
{
    LE_REG_S_SET reg;
    reg.value = RD_LE_REG(LE_REG_PRIVACY_MISC0);
    reg.le_privacy_misc0.re_gen_rand = 1;
    reg.le_privacy_misc0.rand_num_sel_local = 0;
    reg.le_privacy_misc0.rand_num_sel_local = 0;
    WR_LE_REG(LE_REG_PRIVACY_MISC0, reg.value);
#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_MSG_
    LL_LOG_TRACE(GREEN, MSG_LE_PRIVACY_REGEN_RPA, 4,
        RD_LE_REG(LE_REG_LOCAL_RAND_NUM_H), RD_LE_REG(LE_REG_LOCAL_RAND_NUM_L),
        RD_LE_REG(LE_REG_PEER_RAND_NUM_H), RD_LE_REG(LE_REG_PEER_RAND_NUM_L));
#endif
}

void ll_driver_store_tx_resolvable_private_address(void)
{
    if (ll_manager.address_resolution_enable == FALSE)
    {
#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_MSG_
        LL_LOG_TRACE(RED, MSG_LE_PRIVACY_NOT_ENABLE, 0, 0);
#endif
        return;
    }

    LE_REG_S_SET reg;
    reg.value = RD_LE_REG(LE_REG_PRIVACY_MISC0);

    UINT32 cnt = 10000;
    do {
        reg.value = RD_LE_REG(LE_REG_PRIVACY_MISC0);
        cnt --;
    } while (!reg.le_privacy_misc0.tx_seriden_done && cnt);

#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_INFO_
    if (ll_manager.debug_privacy_enable)
    {
        LL_LOG_TRACE(BLUE, DAPE_TEST_LOG213, 2, LE_REG_PRIVACY_MISC0, RD_LE_REG(LE_REG_PRIVACY_MISC0));
        LL_LOG_TRACE(BLUE, DAPE_TEST_LOG213, 2, LE_REG_LOCAL_IRK_VALID_L, RD_LE_REG(LE_REG_LOCAL_IRK_VALID_L));
        LL_LOG_TRACE(BLUE, DAPE_TEST_LOG213, 2, LE_REG_LOCAL_IRK_VALID_H, RD_LE_REG(LE_REG_LOCAL_IRK_VALID_H));
        LL_LOG_TRACE(BLUE, DAPE_TEST_LOG213, 2, LE_REG_PEER_IRK_VALID_L, RD_LE_REG(LE_REG_PEER_IRK_VALID_L));
        LL_LOG_TRACE(BLUE, DAPE_TEST_LOG213, 2, LE_REG_PEER_IRK_VALID_H, RD_LE_REG(LE_REG_PEER_IRK_VALID_H));
        LL_LOG_TRACE(BLUE, DAPE_TEST_LOG213, 2, LE_REG_PEER_ADDR_VALID_L, RD_LE_REG(LE_REG_PEER_ADDR_VALID_L));
        LL_LOG_TRACE(BLUE, DAPE_TEST_LOG213, 2, LE_REG_PEER_ADDR_VALID_H, RD_LE_REG(LE_REG_PEER_ADDR_VALID_H));
    }
#endif

    if (reg.le_privacy_misc0.tx_seriden_val && reg.le_privacy_misc0.tx_seriden_done)
    {
        ll_manager.local_IRK_idx = reg.le_privacy_misc0.tx_seriden_idx;

        if (reg.le_privacy_misc0.fw_local_rpa_val)
        {
            ll_manager.resolving_list.item[ll_manager.local_IRK_idx].local_rpa[0] = RD_LE_REG(LE_REG_LOCAL_RESOLVABLE_ADDR_L);
            ll_manager.resolving_list.item[ll_manager.local_IRK_idx].local_rpa[1] = RD_LE_REG(LE_REG_LOCAL_RESOLVABLE_ADDR_M);
            ll_manager.resolving_list.item[ll_manager.local_IRK_idx].local_rpa[2] = RD_LE_REG(LE_REG_LOCAL_RESOLVABLE_ADDR_H);

#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_INFO_
            if (ll_manager.show_resolvable_addr)
            {
                LL_LOG_TRACE(BLUE, MSG_LE_PRIVACY_LOCAL_RPA, 4, ll_manager.local_IRK_idx, ll_manager.resolving_list.item[ll_manager.local_IRK_idx].local_rpa[2],
                    ll_manager.resolving_list.item[ll_manager.local_IRK_idx].local_rpa[1], ll_manager.resolving_list.item[ll_manager.local_IRK_idx].local_rpa[0]);
            }
#endif
        }

        if (reg.le_privacy_misc0.fw_peer_rpa_val)
        {
            ll_manager.resolving_list.item[ll_manager.local_IRK_idx].peer_rpa[0] = RD_LE_REG(LE_REG_PEER_RESOLVABLE_ADDR_L);
            ll_manager.resolving_list.item[ll_manager.local_IRK_idx].peer_rpa[1] = RD_LE_REG(LE_REG_PEER_RESOLVABLE_ADDR_M);
            ll_manager.resolving_list.item[ll_manager.local_IRK_idx].peer_rpa[2] = RD_LE_REG(LE_REG_PEER_RESOLVABLE_ADDR_H);

#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_INFO_
            if (ll_manager.show_resolvable_addr)
            {
                LL_LOG_TRACE(BLUE, MSG_LE_PRIVACY_PEER_RPA, 4, ll_manager.local_IRK_idx, ll_manager.resolving_list.item[ll_manager.local_IRK_idx].peer_rpa[2],
                    ll_manager.resolving_list.item[ll_manager.local_IRK_idx].peer_rpa[1], ll_manager.resolving_list.item[ll_manager.local_IRK_idx].peer_rpa[0]);
            }
#endif
        }
    }
}

UINT8 ll_driver_store_rx_resolvable_private_address(UINT8 *pHdr, UINT8 offset)
{
    UINT8 idx = LL_MAX_RESOLVING_LIST_SIZE;

    if (ll_manager.address_resolution_enable == FALSE)
    {
#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_MSG_
        LL_LOG_TRACE(RED, MSG_LE_PRIVACY_NOT_ENABLE, 0, 0);
#endif
        return idx;
    }

    LE_HW_RX_PKT_TAIL_S *pTail = (LE_HW_RX_PKT_TAIL_S *) (pHdr + offset);

    /*
     * successfully resolved case should be matched the following rules:
     * 1. in adv channel (pTail->adv_channel == 1)
     * 2. privacy rule passed (pTail->rx_channel & BIT(5) == 1)
     * 3. not passed by scan filter policy = 0x0 or 0x2 (pTail->empty_pdu == 0)
     */
    if (pTail->adv_channel && (pTail->rx_channel & BIT(5)) && !pTail->empty_pdu)
    {
        LE_REG_S_SET reg;
        reg.value = RD_LE_REG(LE_REG_PRIVACY_MISC0);

#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_INFO_
        if (ll_manager.debug_privacy_enable)
        {
            LL_LOG_TRACE(YELLOW, MSG_LE_PRIVACY_RX_STATUS, 1, pTail->rx_status);
            LL_LOG_TRACE(YELLOW, DAPE_TEST_LOG213, 2, LE_REG_PRIVACY_MISC0, reg.value);
            LL_LOG_TRACE(YELLOW, DAPE_TEST_LOG213, 2, LE_REG_LOCAL_IRK_VALID_L, RD_LE_REG(LE_REG_LOCAL_IRK_VALID_L));
            LL_LOG_TRACE(YELLOW, DAPE_TEST_LOG213, 2, LE_REG_LOCAL_IRK_VALID_H, RD_LE_REG(LE_REG_LOCAL_IRK_VALID_H));
            LL_LOG_TRACE(YELLOW, DAPE_TEST_LOG213, 2, LE_REG_PEER_IRK_VALID_L, RD_LE_REG(LE_REG_PEER_IRK_VALID_L));
            LL_LOG_TRACE(YELLOW, DAPE_TEST_LOG213, 2, LE_REG_PEER_IRK_VALID_H, RD_LE_REG(LE_REG_PEER_IRK_VALID_H));
            LL_LOG_TRACE(YELLOW, DAPE_TEST_LOG213, 2, LE_REG_PEER_ADDR_VALID_L, RD_LE_REG(LE_REG_PEER_ADDR_VALID_L));
            LL_LOG_TRACE(YELLOW, DAPE_TEST_LOG213, 2, LE_REG_PEER_ADDR_VALID_H, RD_LE_REG(LE_REG_PEER_ADDR_VALID_H));
        }
#endif

        /* bit[4:0] of rx_channel */
        idx = pTail->rx_channel & 0x1f;

        if (reg.le_privacy_misc0.fw_peer_rpa_val)
        {
            /* copy RPA to resolving list */
            ll_manager.resolving_list.item[idx].peer_rpa[0] = RD_LE_REG(LE_REG_PEER_RESOLVABLE_ADDR_L);
            ll_manager.resolving_list.item[idx].peer_rpa[1] = RD_LE_REG(LE_REG_PEER_RESOLVABLE_ADDR_M);
            ll_manager.resolving_list.item[idx].peer_rpa[2] = RD_LE_REG(LE_REG_PEER_RESOLVABLE_ADDR_H);

#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_INFO_
            if (ll_manager.show_resolvable_addr)
            {
                LL_LOG_TRACE(YELLOW, MSG_LE_PRIVACY_PEER_RPA, 4, idx, ll_manager.resolving_list.item[idx].peer_rpa[2],
                    ll_manager.resolving_list.item[idx].peer_rpa[1], ll_manager.resolving_list.item[idx].peer_rpa[0]);
            }
#endif
        }

        LE_HW_ADVERTISING_CH_RX_PKT_S *pRxPkt = (LE_HW_ADVERTISING_CH_RX_PKT_S *)pHdr;
        BOOLEAN check_local_rpa = FALSE;

        if (pRxPkt->Header.PDU_Type == LL_ADV_PDU_TYPE_ADV_DIRECT_IND
            || pRxPkt->Header.PDU_Type == LL_ADV_PDU_TYPE_SCAN_REQ
            || pRxPkt->Header.PDU_Type == LL_ADV_PDU_TYPE_CONNECT_REQ)
        {
            check_local_rpa = TRUE;
        }

        if (reg.le_privacy_misc0.fw_local_rpa_val && check_local_rpa)
        {
            ll_manager.resolving_list.item[idx].local_rpa[0] = RD_LE_REG(LE_REG_LOCAL_RESOLVABLE_ADDR_L);
            ll_manager.resolving_list.item[idx].local_rpa[1] = RD_LE_REG(LE_REG_LOCAL_RESOLVABLE_ADDR_M);
            ll_manager.resolving_list.item[idx].local_rpa[2] = RD_LE_REG(LE_REG_LOCAL_RESOLVABLE_ADDR_H);

#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_INFO_
            if (ll_manager.show_resolvable_addr)
            {
                LL_LOG_TRACE(YELLOW, MSG_LE_PRIVACY_LOCAL_RPA, 4, idx, ll_manager.resolving_list.item[idx].local_rpa[2],
                    ll_manager.resolving_list.item[idx].local_rpa[1], ll_manager.resolving_list.item[idx].local_rpa[0]);
            }
#endif
        }
    }

    return idx;
}

void ll_driver_dump_hw_resolving_list(void)
{
#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_MSG_
    UINT32 cam_addr = LE_CAM_RESOLVING_LIST_BASE;
    UINT32 i = 0, j = 0;
    UINT8 type = 0;
    UINT16 addr[3] = {0,0,0};
    UINT32 lirk[4] = {0,0,0,0};
    UINT32 pirk[4] = {0,0,0,0};
    const UINT8 lirkSZ = 4, pirkSZ = 4;

    for (i = 0;i < LL_MAX_RESOLVING_LIST_SIZE;i ++)
    {
        cam_addr = LE_CAM_RESOLVING_LIST_BASE + (i * LL_HW_RESOLVING_LIST_ENTRY_SIZE);

        for (j = 0;j < LL_HW_RESOLVING_LIST_ENTRY_SIZE;j ++)
        {
            UINT32 tmp = ll_driver_read_cam(cam_addr + j);

            if (j < lirkSZ)
                lirk[j] = tmp;
            else if (j < (lirkSZ + pirkSZ))
                pirk[j - 4] = tmp;
            else if (j == (lirkSZ + pirkSZ))
                *(UINT32 *)&addr[0] = tmp;
            else
            {
                addr[2] = (UINT16)(tmp & 0x0000FFFF);
                type = (tmp >> 16) & BIT(0);
            }
        }

        if ((RD_LE_REG(LE_REG_PEER_ADDR_VALID_HF(i / 16)) & BIT(i % 16)) > 0)
        {
            LL_LOG_TRACE(GREEN, MSG_RESOLVING_LIST_DUMP, 14,
                i, cam_addr, lirk[3], lirk[2], lirk[1], lirk[0], pirk[3], pirk[2], pirk[1], pirk[0], addr[2], addr[1], addr[0], type);
        }
    }
#endif /* _BT4_2_PRIVACY_FEATURE_SUPPORT_MSG_ */
}

#endif /* _BT4_2_PRIVACY_FEATURE_SUPPORT_ */

/**************************************************************************
 * Function     : ll_driver_fill_conn_win_size_offset
 *
 * Description  : This function is used to set conn_win_size_offset
 *
 * Parameters   : conn_win_size_offset (unit:1.25ms)
 *
 * Returns      : None
 *
 *************************************************************************/
void ll_driver_fill_conn_win_size_offset(UINT8 conn_win_size_offset)
{
    LE_REG_S_SET reg;

    /* update tx win size offset to LL_CONN_TX_WIN_SIZE_OFFSET */
    reg.value = RD_LE_REG(LE_REG_CONN_UPD_WIN_SIZE);
//RT_BT_LOG(BLUE, DAPE_TEST_LOG213, 2, LE_REG_CONN_UPD_WIN_SIZE, reg.value);
    reg.conn_upd_win_size.conn_win_size_offset = conn_win_size_offset;
    WR_LE_REG(LE_REG_CONN_UPD_WIN_SIZE, reg.value);

////
#if 0
    UINT16 dape_reg;
    dape_reg = RD_LE_REG(LE_REG_CONN_UPD_WIN_SIZE);
    RT_BT_LOG(BLUE, DAPE_TEST_LOG213, 2, LE_REG_CONN_UPD_WIN_SIZE, dape_reg);
#endif
}

/**************************************************************************
 * Function     : ll_driver_block_legacy_for_le
 *
 * Description  : This function is used to enable or disable the blocking
 *                of legacy schedule for le
 *
 * Parameters   : block_en : enable or disable the legacy schedule block
 *
 * Returns      : None
 *
 *************************************************************************/
void ll_driver_block_legacy_for_le(UINT8 block_en)
{
    if (IS_BT40)
    {
        LE_REG_S_SET ll_reg;
        ll_reg.value = RD_LE_REG(LE_REG_CONN_UPD_ENTRY);
        if (ll_reg.conn_upd_entry.blk_legacy_one_slot != block_en)
        {
            ll_reg.conn_upd_entry.blk_legacy_one_slot = block_en;
            WR_LE_REG(LE_REG_CONN_UPD_ENTRY, ll_reg.value);
        }
    }
}
/**************************************************************************
 * Function     : ll_driver_block_legacy_slot_for_le
 *
 * Description  : This function is used to set the number of slot before le
 *                anchor point to stop trx legacy traffic.
 *
 * Parameters   : slot : the slot number
 *
 * Returns      : None
 *
 *************************************************************************/
void ll_driver_block_legacy_slot_for_le(UINT8 slot_number)
{
    if (IS_BT40)
    {
        if (slot_number) /* HW(0557 &8723b)
                            has some problem when slot_number = 0. */
        {
            LE_REG_S_SET ll_reg;

            DEF_CRITICAL_SECTION_STORAGE;
            MINT_OS_ENTER_CRITICAL();
            ll_reg.value = RD_LE_REG(LE_REG_NO_ACL);
            ll_reg.le_no_acl.le_no_acl_slot = slot_number;
            WR_LE_REG(LE_REG_NO_ACL, ll_reg.value);
            MINT_OS_EXIT_CRITICAL();
        }
    }
}

/**************************************************************************
 * Function     : ll_driver_set_le_flow_stop
 *
 * Description  : This function is used to set the link layer flow stop
 *
 * Parameters   : on : TRUE is flow stop on, and FALSE is flow stop off
 *
 * Returns      : None
 *
 *************************************************************************/
void ll_driver_set_le_flow_stop(UINT8 on)
{
    LE_REG_S_SET reg;
    reg.value = RD_LE_REG(LE_REG_CE_END_CTRL);
    if (on)
    {
        reg.ce_end_ctrl.fw_flow_ctrl = TRUE;
    }
    else
    {
        reg.ce_end_ctrl.fw_flow_ctrl = FALSE;
    }
    WR_LE_REG(LE_REG_CE_END_CTRL, reg.value);
}

/**************************************************************************
 * Function     : ll_driver_set_le_link_tx_power
 *
 * Description  : This function is used to set the tx power for ble conneciton
 *
 * Parameters   : conn_entry: connection entry number
 *                tx_gain: the tx gain value
 *
 * Returns      : None
 *
 *************************************************************************/
void ll_driver_set_le_link_tx_power(UINT8 conn_entry, UINT8 tx_gain)
{
    UINT16 addr;
    LE_CAM_ADDR_14_S row;
    addr = LE_CAM_ENTRY_BASE(conn_entry) + LE_CAM_ADDR_14;
    row.DWord = ll_driver_read_cam(addr);
    row.tx_power = tx_gain;
    ll_driver_write_cam(addr, row.DWord);
}

#ifndef _GET_LE_SLV_ADDR_TYPE_FROM_HW
#ifdef _DAPE_GET_LE_SLV_ADDR_TYPE_FROM_FW

/**************************************************************************
 * Function     : ll_driver_search_dev_type_from_list
 *
 * Description  : This function is used to search a device unit from white list
 *                or black list.
 *
 * Parameters   : list_type: LL_WHITE_LIST_TYPE or LL_BLACK_LIST_TYPE
 *                addr_type: the type of address (public or random)
 *                addr:      the pointer of device address
 *
 * Returns      : device address unit id
 *
 *************************************************************************/
UINT8 ll_driver_search_dev_type_from_list(UINT8 list_type, UINT8 *addr_type, UINT8 *addr)
{
    LL_DEV_ADDR_LIST_CENTER *pglist;
    UINT8 max_list_size;
    UINT8 node;
    UINT32 addr_buf_w[2] = {0, 0};

    if (list_type == LL_WHITE_LIST_TYPE)
    {
        pglist = &ll_manager.white_list;
        max_list_size = LL_MAX_WHITE_LIST_SIZE;
    }
    else
    {
        pglist = &ll_manager.black_list;
        max_list_size = LL_MAX_BLACK_LIST_SIZE;
    }

    node = pglist->list_head;

    /* copy device address to local buffer for fast comparision */
    memcpy((UINT8*)addr_buf_w, addr, 6);

    while (node != max_list_size)
    {
        /* find matched device unit (type and address field) in the list */
        if (//(pglist->entry[node].type == addr_type) &&
            (pglist->entry[node].DWord[0] == addr_buf_w[0]) &&
            ((pglist->entry[node].DWord[1] & 0xFFFF) == addr_buf_w[1]))
        {
            *addr_type = pglist->entry[node].type;

            break;
        }
        node = pglist->entry[node].next;
    }
    return node;
}

#endif
#endif
#ifdef _NEW_BZDMA_FROM_V8_
UINT8 ll_driver_read_bzdma_rptr_from_cam(UINT8 conn_entry)
{
    UINT16 addr;
    UINT8 hw_rptr;

    addr = LE_CAM_ENTRY_BASE(conn_entry) + LE_CAM_ADDR_14;
    hw_rptr = ll_driver_read_cam(addr) >> 28;
    return hw_rptr;
}

void ll_driver_write_bzdma_rptr_to_cam(UINT8 conn_entry, UINT8 new_rptr)
{
    UINT16 addr;
    UINT32 temp;

    addr = LE_CAM_ENTRY_BASE(conn_entry) + LE_CAM_ADDR_14;
    temp = ll_driver_read_cam(addr);
    ll_driver_write_cam(addr, (temp & 0x0fffffff) | ((new_rptr & 0x0f) << 28));
}

#ifdef _SUPPORT_LBT_FUNC_FOR_RF_ADAPTIVITY_
void ll_driver_enabe_ce_lbt(UINT8 conn_entry, UINT8 tx_power_index)
{
    UINT16 addr;
    UINT32 temp;

    addr = LE_CAM_ENTRY_BASE(conn_entry) + LE_CAM_ADDR_5;
    temp = ll_driver_read_cam(addr);

    if (rtl8723_btrf_check_and_enable_lbt(tx_power_index))
    {
        if (temp & BIT0)
        {
            return;
        }

        /* enable lbt_en_ce bit */
        temp |= BIT0;
    }
    else
    {
        if (!(temp & BIT0))
        {
            return;
        }

        /* disable lbt_en_ce bit */
        temp &= ~BIT0;
    }

    ll_driver_write_cam(addr, temp);
}
#endif

#ifdef _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_
void ll_driver_enable_data_extension(UINT8 conn_entry, UINT8 enable)
{
    UINT16 addr;
    UINT32 temp;

    addr = LE_CAM_ENTRY_BASE(conn_entry) + LE_CAM_ADDR_14;
    temp = ll_driver_read_cam(addr);

    if (enable)
    {
        temp |= BIT27;
    }
    else
    {
        temp &= ~BIT27;
    }

    ll_driver_write_cam(addr, temp);
}
#endif /* end of #ifdef _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_ */

UINT32 ll_driver_read_slave_clock(void)
{
    UINT32 clk;
    clk = (RD_LE_REG(LE_REG_SLAVE_CLOCK_HIGH) & 0x03) << 16; /* bit[17:16]*/
    clk |= RD_LE_REG(LE_REG_SLAVE_CLOCK_LOW);                /* bit[15:0] */
    return clk;
}

UINT32 ll_get_slave_clock_diff(UINT32 exp_clk, UINT32 cur_clk)
{
    UINT32 diff;

    if (exp_clk >= cur_clk)
    {
        /* (msb)-- exp_clk ---------- cur_clk --(lsb) BB_TIMER
                       |<------diff------>|                 */

        diff = exp_clk - cur_clk;
    }
    else
    {
        /* (msb)----cur_clk ---------- exp_clk -----(lsb) BB_TIMER
                |diff->|                   |<--diff-|       */
        diff = 0x00040000 - cur_clk + exp_clk;
    }

    diff &= 0x0003FFFF; /* avoid wrap around */

    return diff;
}

#if defined(_NEW_BLE_HW_SPEC_FROM_150320_) && defined(_LE_WAKE_FROM_SLAVE_LATENCY_)
void ll_driver_wake_from_slave_latency(void)
{
    LE_CAM_ADDR_0_S ent0_data;
    LE_CAM_ADDR_4_S ent4_data;
    LE_CAM_ADDR_15_S ent15_data;
    LE_CAM_ADDR_16_S ent16_data;
    LE_CAM_ADDR_17_S ent17_data;

    UINT16 le_slave_ww_val_slot_end_new;
    UINT16 le_slave_ww_val_slot_end;
    //UINT16 le_slave_ww_val_us_end;
    UINT16 addr;
    UINT8 last_hop = 0;
    UINT16 ce_cnt = 0;
    UINT32 anchor_point_with_ww = 0;                  /* unit : slots */
    UINT32 anchor_point = 0;                          /* unit : slots */
    UINT32 cur_clk = 0;                               /* unit : slots */
    UINT32 time_diff;
    UINT32 time_diff_with_ww;
    UINT16 ce_intv_slots;                             /* unit : slots */
    int    ce_cnt_diff = 0;
    int    ww_slots = 0;                              /* unit : slots*/
    int    slave_laten_cycle_slots = 0;

    UINT8 last_hop_new = 0;
    UINT16 ce_cnt_new = 0;
    UINT32 anchor_point_with_ww_new = 0;              /* unit : slots */
    UINT32 anchor_point_new = 0;                      /* unit : slots */

    UINT8 cause = 7;

    if (0 == ll_manager.conn_unit.handle[0].slave_latency)
    {
        /* no slave latency !! */
        return;
    }

 #ifdef _ROM_CODE_PATCHED_
    if (rcp_ll_driver_wake_from_slave_latency != NULL)
    {
        if (rcp_ll_driver_wake_from_slave_latency((void *)0))
        {
            return;
        }
    }
#endif

    ce_intv_slots = ll_manager.conn_unit.handle[0].ce_interval << 1;
    slave_laten_cycle_slots = ce_intv_slots * (1 + ll_manager.conn_unit.handle[0].slave_latency);
    if (slave_laten_cycle_slots > 0x1FFFF)
    {
        /* hardware only has 18 bits (slots: 0.625 ms) to count cur_clk and anchor pointer */
        slave_laten_cycle_slots = 0x1FFFF;
        //LL_LOG_TRACE(YELLOW, LOG_ERROR_CONN_INTERVAL_LATENCY_TOO_BIG, 2, ce_intv_slots,
        //    ll_manager.conn_unit.handle[0].slave_latency);
    }

    do {
        addr = LE_CAM_ENTRY_BASE(0) + LE_CAM_ADDR_17;
        ent17_data.DWord = ll_driver_read_cam(addr);
        anchor_point = ent17_data.ap_int;               /* post anchor point */

        cur_clk = ll_driver_read_slave_clock();

        addr = LE_CAM_ENTRY_BASE(0) + LE_CAM_ADDR_0;
        ent0_data.DWord = ll_driver_read_cam(addr);
        last_hop = ent0_data.last_hop;                  /* previous last hop ch */

        addr = LE_CAM_ENTRY_BASE(0) + LE_CAM_ADDR_15;
        ent15_data.DWord = ll_driver_read_cam(addr);
        //le_slave_ww_val_us_end = (UINT16)(ent15_data.ww_val) & 0x3FF;
        le_slave_ww_val_slot_end = (UINT16)(ent15_data.ww_val>>10) & 0xFFFF;

        addr = LE_CAM_ENTRY_BASE(0) + LE_CAM_ADDR_16;
        ent16_data.DWord = ll_driver_read_cam(addr);

        anchor_point_with_ww = (ent0_data.anchor_point << 1) | ent16_data.ap_backup_b0;

        time_diff = ll_get_slave_clock_diff(anchor_point, cur_clk);
        if (time_diff <= ce_intv_slots)
        {
            /* not in slave latency period !! */
            cause = 2;
            break;
        }

        time_diff_with_ww = ll_get_slave_clock_diff(anchor_point, cur_clk);
        time_diff_with_ww -= (le_slave_ww_val_slot_end+1);
        if (time_diff > slave_laten_cycle_slots ||
            time_diff_with_ww > slave_laten_cycle_slots)
        {
            /* anchor point or ap_ww is earilier than cur_clk*/
            cause = 3;
            break;
        }

        if (time_diff < time_diff_with_ww)
        {
            /* ap is earlier than ap_ww  */
            cause = 4;
            break;
        }

        ww_slots = ll_get_slave_clock_diff(anchor_point, anchor_point_with_ww);
        if (time_diff < ww_slots)
        {
            /*check to make sure cur_clk is not between ap_ww and ap */
            cause = 5;
            break;
        }

        addr = LE_CAM_ENTRY_BASE(0) + LE_CAM_ADDR_4;
        ent4_data.DWord = ll_driver_read_cam(addr);
        ce_cnt = ent4_data.ce_count;                    /* post ce counter */

        ce_cnt_diff = time_diff_with_ww / ce_intv_slots;        /* the difference of ce counter */

        // protect hardware for at least 1 slots between new ap_with_ww and current time
        if ((time_diff_with_ww % ce_intv_slots) < 1)
        {
            ce_cnt_diff--;
            if (ce_cnt_diff <= 0)
            {
                cause = 1;
                break;
            }
        }

        /* shorten window widening by percent, make it 1 slot bigger to increase rx success */
        le_slave_ww_val_slot_end_new = le_slave_ww_val_slot_end + 1 -
            le_slave_ww_val_slot_end  * ce_cnt_diff / (1 + ll_manager.conn_unit.handle[0].slave_latency);

        /* update new parameters */
        anchor_point_new = ll_get_slave_clock_diff(anchor_point, ce_cnt_diff * ce_intv_slots);
        anchor_point_with_ww_new = ll_get_slave_clock_diff(anchor_point_new, le_slave_ww_val_slot_end_new);
        ce_cnt_new =(ce_cnt >= ce_cnt_diff)?(ce_cnt - ce_cnt_diff):(ce_cnt - ce_cnt_diff + 0xFFFF);
        last_hop_new = (last_hop + ((37 - ent0_data.hop_inc) * ce_cnt_diff)) % 37;

        /* write new parameters to CAM, CAM15 will be caculate by HW, no need to update it */
        ent0_data.last_hop = last_hop_new;
        ent0_data.anchor_point = anchor_point_with_ww_new >> 1; /* slot to slot pair */
        ent4_data.ce_count = ce_cnt_new;
        ent16_data.ap_backup_b0 = anchor_point_with_ww_new & 0x01;
        ent17_data.ap_int = anchor_point_new;

        addr = LE_CAM_ENTRY_BASE(0) + LE_CAM_ADDR_0;
        if (BT_ERROR_OK != ll_driver_write_cam(addr, ent0_data.DWord))
        {
            break;
        }
        addr = LE_CAM_ENTRY_BASE(0) + LE_CAM_ADDR_4;
        if (BT_ERROR_OK != ll_driver_write_cam(addr, ent4_data.DWord))
        {
            break;
        }
        addr = LE_CAM_ENTRY_BASE(0) + LE_CAM_ADDR_16;
        if (BT_ERROR_OK != ll_driver_write_cam(addr, ent16_data.DWord))
        {
            break;
        }
        addr = LE_CAM_ENTRY_BASE(0) + LE_CAM_ADDR_17;
        if (BT_ERROR_OK != ll_driver_write_cam(addr, ent17_data.DWord))
        {
            break;
        }

        /* write some new parameters to ll register */
        WR_LE_REG(LE_REG_ANCHOR_POINT_RPT1, anchor_point_with_ww_new & 0xffff);
        WR_LE_REG(LE_REG_ANCHOR_POINT_RPT2, anchor_point_with_ww_new >> 16);
        cause = 0;
    }
    while (0);

    if(3 < cause )
    {
        LL_LOG_TRACE(YELLOW, LE_MSG_SLA_LATENCY_WAKEUP, 10,
                 cause, cur_clk, anchor_point_with_ww, anchor_point_with_ww_new,
                 anchor_point, anchor_point_new, last_hop, last_hop_new,
                 ce_cnt, ce_cnt_new);
    }
#if 0 /* read CAM out and test whether write success.*/
    if (cause == 0)
    {
        UINT32 Cam0 = 0;
        UINT32 Cam4 = 0;
        UINT32 Cam16 = 0;
        UINT32 Cam17 = 0;

        addr = LE_CAM_ENTRY_BASE(0) + LE_CAM_ADDR_0;
        Cam0 = ll_driver_read_cam(addr);

        addr = LE_CAM_ENTRY_BASE(0) + LE_CAM_ADDR_4;
        Cam4 = ll_driver_read_cam(addr);

        addr = LE_CAM_ENTRY_BASE(0) + LE_CAM_ADDR_16;
        Cam16 = ll_driver_read_cam(addr);

        addr = LE_CAM_ENTRY_BASE(0) + LE_CAM_ADDR_17;
        Cam17 = ll_driver_read_cam(addr);
        LL_LOG_TRACE(YELLOW, YL_DBG_HEX_8, 8, Cam0, ent0_data.DWord, Cam4, ent4_data.DWord,
            Cam16, ent16_data.DWord, Cam17, ent17_data.DWord);
    }
#endif
}
#endif
#endif /* end of #ifdef _NEW_BZDMA_FROM_V8_ */

