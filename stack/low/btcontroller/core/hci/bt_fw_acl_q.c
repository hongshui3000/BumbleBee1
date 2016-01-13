/***************************************************************************
Copyright (C) Mindtree Ltd.
This module is a confidential and proprietary property of MindTree and
a possession or use of this module requires written permission of MindTree.
***************************************************************************/

/**
* \file
*  ACL Queue implementation.
*
* \author Muthu Subramanian K
*/

/********************************* Logger *************************/
enum { __FILE_NUM__= 2 };
/********************************* Logger *************************/

/* Includes */
#include "lc_internal.h"        /* For lc flags like LC_ACTIVE_BC_FLAG  */
#include "lmp.h"                /* For lmp_structs like self_dev_data   */
#include "lc.h"
#include "bt_fw_hci_defines.h"
#include "bt_fw_hci_external_defines.h"
#include "bt_fw_types.h"
#include "bt_fw_acl_q.h"
#include "vendor.h"             /* Included for send_num_complete...    */
#include "mem.h"
#include "bt_fw_hci_2_1.h"
#include "bzdma.h"
#include "bt_fw_hci_internal.h"
#include "array_map.h"

/* the head of acl pkt queue list (received from host) */
HCI_ACL_DATA_PKT *acl_q_head = NULL;
/* the tail of acl pkt queue list (received from host) */
HCI_ACL_DATA_PKT *acl_q_tail = NULL;

#ifdef _NEW_HCI_DMA_DESIGN_FOR_ACL_SNIFF_SCHEDULE_
/* the head of acl pkt queue list in sniff mode (received from host) */
HCI_ACL_DATA_PKT *acl_q_head_in_sniff = NULL;
/* the tail of acl pkt queue list in sniff mode (received from host) */
HCI_ACL_DATA_PKT *acl_q_tail_in_sniff = NULL;
#endif

UCHAR failed_lut_index[MAX_NO_OF_LUT_INDICES] = {0};
UCHAR paused_lut_index[MAX_NO_OF_LUT_INDICES] = {0};
UCHAR flow_enabled_lut_index[MAX_NO_OF_LUT_INDICES];

HCI_ACL_DATA_ZERO_PKT aclq_zero_length_pkt[MAX_PICONET_CNT];

UINT16 g_lmp_min_sniff_intv_to_skip_data_tx_opt = 20;
UINT32 g_lc_min_slots_to_skip_mark_as_bad_for_sniff = 4;

#ifdef HS_USB_SEND_EVENT_REORDER_ISSUE
UINT16 hs_recd_conn_handle = 0xBEEF;

/* the head of acl waiting queue list (received from air) */
HCI_ACL_RX_DATA_PKT *hs_wait_queue_acl_head = NULL;
/* the tail of acl waiting queue list (received from air) */
HCI_ACL_RX_DATA_PKT *hs_wait_queue_acl_tail = NULL;
#endif

#ifdef _ROM_CODE_PATCHED_
PF_ROM_CODE_PATCH_FUNC rcp_aclq_get_next_acl_pkt = NULL;
#endif
#ifdef _DAPE_TEST_SEND_ZERO_LEN_PKT_BY_VENDOR_CMD
extern UINT8 g_send_zero_len;
#endif

/* --------------------------- Static Functions --------------------------*/
UCHAR aclq_is_am_addr_schedulable(UCHAR am_addr, UCHAR phy_piconet_id,
        HCI_ACL_DATA_PKT *pkt, UCHAR aclq_pkt_nbc_oldest_count);

void check_flush_occured_event_generation(UINT16 ce_index);

#ifdef HS_USB_SEND_EVENT_REORDER_ISSUE
/**
 * send hci acl packet to waiting queue
 *
 * \param acl_pkt : pointer of hci acl packet
 *
 * \return None.
 */
void aclq_send_acl_pkt_in_wait_queue(HCI_ACL_RX_DATA_PKT *acl_pkt)
{
    HCI_ACL_RX_DATA_PKT *acl_pkt_next;
    UINT16 ce_index;

    while (acl_pkt != NULL)
    {
        acl_pkt_next = acl_pkt->next;
        acl_pkt->next = NULL;

        if ((LMP_GET_CE_INDEX_FROM_CONN_HANDLE(acl_pkt->connection_handle,
                                               &ce_index) != API_SUCCESS) ||
             (lmp_connection_entity[ce_index].entity_status != ASSIGNED))
        {
            /* free the queue packets when the link is not alive or the
               connection handle is mismatched */
            if (OS_FREE_BUFFER(tx_table[ACL_DATA_HANDLER_TASK].pool_handle,
                               acl_pkt) == BT_ERROR_OK)
            {
                os_free_reserved_buffer();
            }
        }
        else
        {
            /* send acl-u pkt to the host */
            hci_handle_data_to_host(acl_pkt);
        }
        acl_pkt = acl_pkt_next;
    }
}
#endif /* HS_USB_SEND_EVENT_REORDER_ISSUE */

#ifdef HS_USB_SEND_EVENT_REORDER_ISSUE_NEW
ARRAY_MAP_DEFINE_TYPE(ACL_RX_DATA_WAIT_QUEUE *, ACL_RX_DATA_WAIT_QUEUE_MAP);

BOOLEAN acl_rx_data_wait_queue_map_bsearch(
        ACL_RX_DATA_WAIT_QUEUE_MAP *map, UINT16 key, int *index)
{
#define ACL_RX_DATA_WAIT_QUEUE_MAP_CMP(item, key) \
    ((int) ((item)->conn_handle) - (int) key)

    return ARRAY_MAP_BSEARCH(map, key, ACL_RX_DATA_WAIT_QUEUE_MAP_CMP, index);
}

#define ACLQ_RX_WAIT_QUEUE_NUM 8

ACL_RX_DATA_WAIT_QUEUE aclq_rx_wq_buf[ACLQ_RX_WAIT_QUEUE_NUM];
ACL_RX_DATA_WAIT_QUEUE *aclq_rx_wq[ACLQ_RX_WAIT_QUEUE_NUM];

ACL_RX_DATA_WAIT_QUEUE_MAP aclq_rx_wq_map;

void aclq_rx_wait_queue_init()
{
    int i;
    for (i = 0; i < ACLQ_RX_WAIT_QUEUE_NUM; ++i)
    {
        aclq_rx_wq[i] = &aclq_rx_wq_buf[i];
    }
    ARRAY_MAP_INIT(&aclq_rx_wq_map, aclq_rx_wq, ACLQ_RX_WAIT_QUEUE_NUM);
}

ACL_RX_DATA_WAIT_QUEUE *aclq_rx_wait_queue_find(UINT16 handle)
{
    return ARRAY_MAP_FIND(&aclq_rx_wq_map, handle,
            acl_rx_data_wait_queue_map_bsearch);
}

ACL_RX_DATA_WAIT_QUEUE *aclq_rx_wait_queue_get(UINT16 handle)
{
#define ACL_RX_DATA_WAIT_QUEUE_MAP_INIT(item, key) \
    do { \
        (item)->conn_handle = (key); \
        (item)->data.head = NULL; \
        (item)->data.tail = NULL; \
        (item)->send = NULL; \
    } while (0)

    return ARRAY_MAP_INSERT(&aclq_rx_wq_map, handle,
            acl_rx_data_wait_queue_map_bsearch,
            ACL_RX_DATA_WAIT_QUEUE_MAP_INIT);
}

void aclq_rx_wait_queue_free(UINT16 handle)
{
#define ACL_RX_DATA_WAIT_QUEUE_MAP_FLZR(item)

        ARRAY_MAP_REMOVE(&aclq_rx_wq_map, handle,
                acl_rx_data_wait_queue_map_bsearch,
                ACL_RX_DATA_WAIT_QUEUE_MAP_FLZR);
}

void aclq_rx_wait_queue_enqueue(ACL_RX_DATA_WAIT_QUEUE *queue,
        HCI_ACL_RX_DATA_PKT *pkt)
{
    pkt->next = NULL;
    if (queue->data.tail != NULL)
    {
        queue->data.tail->next = pkt;
    }
    else
    {
        queue->data.head = pkt;
    }
    queue->data.tail = pkt;
}

void aclq_rx_wait_queue_enqueue_le(ACL_RX_DATA_WAIT_QUEUE *queue,
        LL_HCI_ACL_DATA_PKT *pkt)
{
    pkt->next = NULL;
    if (queue->le_data.tail != NULL)
    {
        queue->le_data.tail->next = pkt;
    }
    else
    {
        queue->le_data.head = pkt;
    }
    queue->le_data.tail = pkt;
}

HCI_ACL_RX_DATA_PKT *aclq_rx_wait_queue_dequeue(ACL_RX_DATA_WAIT_QUEUE *queue)
{
    HCI_ACL_RX_DATA_PKT *pkt = queue->data.head;
    if (pkt != NULL)
    {
        queue->data.head = pkt->next;
        pkt->next = NULL;
        if (queue->data.head == NULL)
        {
            queue->data.tail = NULL;
        }
    }
    return pkt;
}

HCI_ACL_RX_DATA_PKT *aclq_rx_wait_queue_dequeue_all(ACL_RX_DATA_WAIT_QUEUE *queue)
{
    HCI_ACL_RX_DATA_PKT *pkt = queue->data.head;
    queue->data.head = NULL;
    queue->data.tail = NULL;
    return pkt;
}

/**
 * send hci acl packet to waiting queue
 *
 * \param acl_pkt : pointer of hci acl packet
 *
 * \return None.
 */
void aclq_send_acl_rx_pkt_in_wait_queue(HCI_ACL_RX_DATA_PKT *acl_pkt)
{
    HCI_ACL_RX_DATA_PKT *acl_pkt_next;
    UINT16 ce_index;

    while (acl_pkt != NULL)
    {
        acl_pkt_next = acl_pkt->next;
        acl_pkt->next = NULL;

        if ((LMP_GET_CE_INDEX_FROM_CONN_HANDLE(acl_pkt->connection_handle,
                                               &ce_index) != API_SUCCESS) ||
             (lmp_connection_entity[ce_index].entity_status != ASSIGNED))
        {
            /* free the queue packets when the link is not alive or the
               connection handle is mismatched */
            if (OS_FREE_BUFFER(tx_table[ACL_DATA_HANDLER_TASK].pool_handle,
                               acl_pkt) == BT_ERROR_OK)
            {
                os_free_reserved_buffer();
            }
        }
        else
        {
            /* send acl-u pkt to the host */
            hci_handle_data_to_host(acl_pkt);
        }
        acl_pkt = acl_pkt_next;
    }
}

void aclq_send_le_acl_rx_pkt_in_wait_queue(LL_HCI_ACL_DATA_PKT *acl_pkt)
{
    LL_HCI_ACL_DATA_PKT *acl_pkt_next;
    LL_CONN_HANDLE_UNIT *conn_unit;

    while (acl_pkt != NULL)
    {
        acl_pkt_next = acl_pkt->next;
        acl_pkt->next = NULL;

        conn_unit = ll_fw_search_handle_unit_via_conn_handle(
                acl_pkt->connection_handle);
        if (conn_unit == NULL || !conn_unit->connected)
        {
            /* free the queue packets when the link is not alive or the
               connection handle is mismatched */
            if (OS_FREE_BUFFER(tx_table[LE_ACL_DATA_HANDLER_TASK].pool_handle,
                               acl_pkt) == BT_ERROR_OK)
            {
                os_free_reserved_buffer();
            }
        }
        else
        {
            /* send acl-u pkt to the host */
            hci_handle_data_to_host((HCI_ACL_RX_DATA_PKT *) acl_pkt);
        }
        acl_pkt = acl_pkt_next;
    }
}
#endif /* HS_USB_SEND_EVENT_REORDER_ISSUE_NEW */

/**
 * Checks for generation of flush occured, enhanced_flush_complete and
 * command complete for hci_flush commands.
 *
 * \param pkt_flushed Array of pkt_flushed flags (for each CE index).
 *
 * \return None.
 */
void check_flush_occured_event_generation(UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY *ce_ptr;
    UCHAR gen_fls = TRUE;

    ce_ptr = &lmp_connection_entity[ce_index];

    /* Generation of enhanced_flush_complete */
    if (ce_ptr->enhanced_flush == TRUE)
    {
        if (aclq_acl_check_flushed(
                    ce_ptr->am_addr, ce_ptr->phy_piconet_id) == TRUE)
        {
            hci_generate_enhanced_flush_event(
                ce_ptr->connection_type.connection_handle);
            ce_ptr->enhanced_flush = FALSE;
            gen_fls = FALSE;
        }
    }

    RT_BT_LOG(GRAY, BT_MSG_GENERATE_FLUSH, 1, gen_fls);

    /* Generation of flush_complete event */
    if (ce_ptr->flush_running == TRUE)
    {
        if (aclq_acl_check_flushed(
                    ce_ptr->am_addr, ce_ptr->phy_piconet_id) == FALSE)
        {
            hci_generate_command_complete_event(
                HCI_FLUSH_OPCODE,
                HCI_COMMAND_SUCCEEDED,
                ce_index/*(UINT16)j*/,
                NULL);
            ce_ptr->flush_running = FALSE;
        }
    }

    if (gen_fls == TRUE)
    {
        hci_generate_flush_occured_event(
            ce_ptr->connection_type.connection_handle);
    }

    return;
}
/**
* Updates the continue fragments with the 'start-fragment' time,
* since that is the time that the ACL-U packet transmission started.
*
* \param pkt HCI_ACL_DATA_PKT pointer.
* \param ce_index Connection Entity Index.
*
* \return None.
*/
void aclq_update_flush_time(HCI_ACL_DATA_PKT *ppkt,UINT16 ce_index)
{
    HCI_ACL_DATA_PKT_WS *pkt = ppkt->ws;

    switch(ppkt->packet_boundary_flag)
    {
        case L_CH_L2CAP_NON_FLUSH:
        case L_CH_L2CAP_START:
            lmp_connection_entity[ce_index].aclq_start_flush_time =
                pkt->flush_time;
            break;

        case L_CH_L2CAP_CONT:
            pkt->flush_time =
                lmp_connection_entity[ce_index].aclq_start_flush_time;
            break;

        default:
            break;
    }
}


/**
* [External interface] Initialize the ACL Queue.
*
* \param None.
* \return None.
*/
void aclq_init(void)
{
    UCHAR i;

    acl_q_head = NULL;
    acl_q_tail = NULL;

#ifdef _NEW_HCI_DMA_DESIGN_FOR_ACL_SNIFF_SCHEDULE_
    acl_q_head_in_sniff = NULL;
    acl_q_tail_in_sniff = NULL;
#endif

    for(i = 0; i < LC_MAX_NUM_OF_LUT_EX_TABLES; i++)
    {
        failed_lut_index[i] = FALSE;
        paused_lut_index[i] = FALSE;
        flow_enabled_lut_index[i] = TRUE;
    }

#ifdef BROADCAST_DATA
    lmp_self_device_data.bc_conn_handle = 0xFFFF;
    lmp_self_device_data.park_bc_conn_handle = 0xFFFF;
#endif

    for (i = 0; i < MAX_PICONET_CNT; i++)
    {
        memset((void *)&aclq_zero_length_pkt[i], 0, sizeof(HCI_ACL_DATA_ZERO_PKT));
        aclq_zero_length_pkt[i].packet_boundary_flag = L_CH_L2CAP_CONT;
        aclq_zero_length_pkt[i].broadcast_flag = LC_DATA_ACTIVE_FLAG;
        aclq_zero_length_pkt[i].ws.phy_piconet_id = i;
    }

#ifdef HS_USB_SEND_EVENT_REORDER_ISSUE_NEW
    aclq_rx_wait_queue_init();
#endif
#ifdef HS_USB_SEND_EVENT_REORDER_ISSUE
    hs_recd_conn_handle = 0xBEEF;
    hs_wait_queue_acl_head = NULL;
    hs_wait_queue_acl_tail = NULL;
#endif
}

/**
* [External interface] Queue the ACL packet for transmission.
* Do not free the pointer until the transmission is complete.
*
* \param pkt the acl data packet.
*
* \return API_SUCCESS or API_FAILURE.
*/
API_RESULT aclq_queue_acl_pkt(HCI_ACL_DATA_PKT *ppkt)
{
    UINT16 ce_index;
    HCI_ACL_DATA_PKT_WS *pkt = ppkt->ws;
    UINT16 conn_handle = ppkt->connection_handle;

    /* Initialize Packet data */
    memset(((UINT8*)pkt) + 4, 0, sizeof(HCI_ACL_DATA_PKT_WS) - 4);

    /* check BC flag and get connection entity, am_addr and piconet id */

    switch(ppkt->broadcast_flag)
    {
    case LC_DATA_ACTIVE_FLAG:
        if (LMP_GET_CE_INDEX_FROM_CONN_HANDLE(conn_handle,
                                              &ce_index)!=API_SUCCESS)
        {
            return API_FAILURE;
        }

        if (lmp_ch_to_ce_index_table[conn_handle - 1].status != DEDICATED)
        {
            lmp_ch_to_ce_index_table[conn_handle - 1].status = DEDICATED;
        }

        pkt->am_addr = lmp_connection_entity[ce_index].am_addr;

        pkt->phy_piconet_id = lmp_connection_entity[ce_index].phy_piconet_id;

        aclq_update_flush_time(ppkt,ce_index);
        break;

#ifdef BROADCAST_DATA
    case LC_ACTIVE_BC_FLAG:
        pkt->am_addr = BC_AM_ADDR;
        pkt->phy_piconet_id = lc_get_master_piconet_id();

        if (pkt->phy_piconet_id == SCA_PICONET_INVALID)
        {
            return API_FAILURE;
        }
        lmp_self_device_data.bc_conn_handle = conn_handle;
        break;

    case LC_PNET_BC_FLAG:
        pkt->am_addr = BC_AM_ADDR;
        pkt->phy_piconet_id = lc_get_master_piconet_id();
        if (pkt->phy_piconet_id == SCA_PICONET_INVALID)
        {
            return API_FAILURE;
        }
        lmp_self_device_data.park_bc_conn_handle = conn_handle;
        break;
#endif

    default:
        return API_FAILURE;
    }


    /* Add to Queue */
    ppkt->next = NULL;

    if (acl_q_head == NULL)
    {
        acl_q_head = ppkt;
    }
    else
    {
        acl_q_tail->next = ppkt;
    }
    acl_q_tail = ppkt;

    return API_SUCCESS;
}

/**
* [External interface] Get the next ACL packet for transmission
* for scheduling.
*
* \param am_addr address to return am_addr.
* \param length address to return the length.
* \param frag_pointer address to return the offset to start from.
* \param pkt_type address to return the packet type to use for transmission.
*
* \return pkt_type pointer to the acl packet or NULL.
*/

HCI_ACL_DATA_PKT *aclq_get_next_acl_pkt(UCHAR *am_addr,
                                        UINT16 *length,
                                        UINT16 *frag_pointer,
                                        UINT16 *pkt_type_lut,
                                        UCHAR piconet_id)
{
    HCI_ACL_DATA_PKT *ppkt;
    HCI_ACL_DATA_PKT *ppkt1;
    UINT16 avail_length;
    UINT16 ce_index = INVALID_CE_INDEX;
    UCHAR aclq_pkt_nbc_oldest_count;
    UCHAR flush_to_iteration_count;
    HCI_ACL_DATA_PKT_WS *pkt;
    HCI_ACL_DATA_PKT_WS *pkt1;
    UINT8 nothing = FALSE;
    UINT8 is_send_zero_length_acl_pkt = FALSE;

    flush_to_iteration_count = 0x0;

    while (1)
    {
        /* start_of_get_next_acl_pkt */

        ppkt = NULL;
        aclq_pkt_nbc_oldest_count = 0x0;

        if (flush_to_iteration_count > 0x2)
        {
            /* This is to Avoid infinite loop */
            /* flush_to_iteration_count == 0x0, First Iteration */
            /* flush_to_iteration_count == 0x1, Second Iteration
             * 1. Give chance for another link as there are pending ack from bb
             * and zero length pkt already sent for this link.
             */

            /* flush_to_iteration_count == 0x2, Third Iteration, one more
             * iteration
             */
            nothing = TRUE;
            break;
        }

        if (aclq_is_acl_queue_empty())
        {
            nothing = TRUE;
            break;
        }

        ppkt1 = acl_q_head;

        while (ppkt1 != NULL)
        {
            /* check any acl pkt (tx failed) in the acl_q list and get the
               oldest retry count */

            pkt1 = ppkt1->ws;

            if ((pkt1->phy_piconet_id == piconet_id) &&
                (pkt1->acl_pkt_tx_failed == 0x1))
            {
                if (pkt1->acl_pkt_nbc_old_count > aclq_pkt_nbc_oldest_count)
                {
                    aclq_pkt_nbc_oldest_count = pkt1->acl_pkt_nbc_old_count;
                }
            }
            ppkt1 = ppkt1->next;
        }

#ifdef _ROM_CODE_PATCHED_
        if (rcp_aclq_get_next_acl_pkt != NULL)
        {
            /* schedule any valid packet in different policy via patch */
            rcp_aclq_get_next_acl_pkt((void*)&ppkt, &pkt, piconet_id);
        }
#endif

        if (ppkt == NULL)
        {
            /* Check for any in transmission pkts that can be queued. */
            ppkt = acl_q_head;

            while (ppkt != NULL)
            {
                pkt = ppkt->ws;

                if ((pkt->in_transmission == TRUE) &&
                    (pkt->phy_piconet_id == piconet_id) &&
                    (aclq_is_am_addr_schedulable(pkt->am_addr, pkt->phy_piconet_id,
                                        ppkt, aclq_pkt_nbc_oldest_count) == TRUE))
                {
                    if ((pkt->frame_pkt_count == 0x1) &&
                       ((pkt->tx_length1 + pkt->tx_length2) <
                         ppkt->acl_data_total_length))
                    {
                        /* This pkt is not fully scheduled for transmission. */
                        /* Let us schedule the remaining portion of the data */
                        break;
                    }
                }
                ppkt = ppkt->next;
            }
        } /* end of if (pkt == NULL) */

        if (ppkt == NULL)
        {
            /* Check for the active am address pkt for schedule if any */
            ppkt = acl_q_head;

            while (ppkt != NULL)
            {
                pkt = ppkt->ws;
                if ((pkt->in_transmission == FALSE) &&
                    (pkt->phy_piconet_id == piconet_id) &&
                    (aclq_is_am_addr_failed(pkt->am_addr, piconet_id) == FALSE) &&
                    (aclq_is_am_addr_schedulable(pkt->am_addr,
                                         pkt->phy_piconet_id, ppkt,
                                         aclq_pkt_nbc_oldest_count) == TRUE))
                {
                    break;
                }
                ppkt = ppkt->next;
            }
        } /* end of if (pkt == NULL) */

        if (ppkt == NULL)
        {
            /* If there are none for active am_addr then
            * get the 1st in_transmission = FALSE packet
            */
            ppkt = acl_q_head;

            while (ppkt != NULL)
            {
                pkt = ppkt->ws;
                if ((pkt->in_transmission == FALSE) &&
                    (pkt->phy_piconet_id == piconet_id) &&
                    (aclq_is_am_addr_schedulable(pkt->am_addr,
                                         pkt->phy_piconet_id, ppkt,
                                         aclq_pkt_nbc_oldest_count) == TRUE))
                {
                    break;
                }
                ppkt = ppkt->next;
            }
        }  /* end of if (pkt == NULL) */

        if (ppkt == NULL)
        {
            nothing = TRUE;
            break;
        }

        /* Let us calculate the length of data pending for transmission */
        avail_length = ppkt->acl_data_total_length -
                       (pkt->tx_length1 + pkt->tx_length2);

        if (pkt->am_addr != BC_AM_ADDR)
        {
            ppkt1 = ppkt->next;

            //UINT16 max_hci_acl_data_size = otp_str_data.bt_read_buffer_size;
            UINT16 max_hci_acl_data_size = 1021;

            while ((ppkt1 != NULL) && (avail_length < max_hci_acl_data_size))
            {
                pkt1 = ppkt1->ws;

                if ((pkt1->am_addr == pkt->am_addr) &&
                    (pkt1->phy_piconet_id == pkt->phy_piconet_id))
                {
                    if (ppkt1->packet_boundary_flag == L_CH_L2CAP_CONT)
                    {
                        avail_length += (ppkt1->acl_data_total_length -
                                        (pkt1->tx_length1 + pkt1->tx_length2));

                        if (avail_length >= max_hci_acl_data_size)
                        {
                            break;
                        }
                    }
                    else
                    {
                        /* There is a L2CAP start pkt, so don't try to include
                         * more pkts
                         */
                        break;
                    }
                }
                ppkt1 = ppkt1->next;
            }
        } /* end of if (pkt->am_addr != BC_AM_ADDR) */

        if (pkt->am_addr != BC_AM_ADDR)
        {
            LMP_GET_CE_INDEX_FROM_CONN_HANDLE((UINT16)(ppkt->connection_handle),
                                              &ce_index);
            if (ce_index == INVALID_CE_INDEX)
            {
                aclq_clear_all_pkts_am_addr(pkt->am_addr, FALSE, piconet_id);
                continue;
            }

            LMP_CONNECTION_ENTITY *ce_ptr;
            ce_ptr = &lmp_connection_entity[ce_index];

            /*************************************************************/
            /* Let us handle this case here itself and return zero length pkt */
            if (ce_ptr->aclq_resch_flag == RESCHEDULE_FLAG_ZERO_LENGTH)
            {
                is_send_zero_length_acl_pkt = TRUE;
                break;
            }

            /*************************************************************/
            /*
             * Check for flush timeout pkt.
             * Take appropriate actions.
             */
            if (lc_is_pkt_timeout(ppkt, ce_ptr) == TRUE)
            {
                if (((pkt->acl_pkt_tx_failed == 0x0) &&
                    (pkt->ackd_length >= (pkt->tx_length1 + pkt->tx_length2)))
#ifdef _DAPE_TEST_SEND_ZERO_LEN_PKT_BY_VENDOR_CMD
                     &&(g_send_zero_len == 0)
#endif
                    )
                {
                    /* There is no pending fragment */
                    /* So we can flush this pkt safely */
                    pkt->in_transmission = TRUE;
                    aclq_ack_acl_pkt(ppkt,
                                 (ppkt->acl_data_total_length - pkt->ackd_length),
                                 0x1, TRUE);

                    /* Let us try to schedule another pkt */
                    continue;
                }
                else
                {
                    if (pkt->acl_pkt_tx_failed == 0x1)
                    {
                        pkt->acl_pkt_nbc_old_count = 0x0;
                        pkt->acl_pkt_tx_failed = 0x0;
                    }

                    /* There are pending fragment waiting for ack, so send
                     * zero length pkt.
                     */
                    if ((ce_ptr->is_last_sent_zero_l_l2cap == 0x0)
#ifdef _DAPE_TEST_SEND_ZERO_LEN_PKT_BY_VENDOR_CMD
                        ||(g_send_zero_len)
#endif
                    )
                    {
#ifdef _DAPE_TEST_SEND_ZERO_LEN_PKT_BY_VENDOR_CMD
                        g_send_zero_len = 0;
#endif

                        /* Let us send zero length pkt now. */
                        is_send_zero_length_acl_pkt = TRUE;
                        break;
                    }
                    else
                    {
                        /* Let us try to schedule another pkt */
                        flush_to_iteration_count++;
                        continue;
                    }
                }
            } /* end of if ((lc_is_pkt_timeout(pkt, &lmp_connection_enti... */

            if (pkt->acl_pkt_tx_failed == 0x1)
            {
                ppkt1 = acl_q_head;

                while (ppkt1 != NULL)
                {
                    pkt1 = ppkt1->ws;
                    if ((pkt1->phy_piconet_id == piconet_id) &&
                            (pkt1->acl_pkt_tx_failed == 0x1))
                    {
                        pkt1->acl_pkt_nbc_old_count++;
                    }
                    ppkt1 = ppkt1->next;
                }

                avail_length = pkt->failed_frame_length;
                *pkt_type_lut = pkt->selected_pkt_type;
            }
            else
            {
                lc_decide_packet_type(&avail_length, pkt_type_lut, ce_index);
            }
        }
        else
        {
            /* broadcast pkt */
            RT_BT_LOG(GRAY, LC_SCHED_MSG_BROADCAST, 6,
                    ppkt, ppkt->connection_handle, ppkt->packet_boundary_flag,
                    ppkt->broadcast_flag,ppkt->acl_data_total_length,
                    avail_length);
#ifdef SECURE_CONN_BROADCAST_CHK
RT_BT_LOG(WHITE, DAPE_TEST_LOG551, 11, ppkt->acl_data_total_length,
ppkt->hci_acl_data_pkt[0],
ppkt->hci_acl_data_pkt[1],
ppkt->hci_acl_data_pkt[2],
ppkt->hci_acl_data_pkt[3],
ppkt->hci_acl_data_pkt[4],
ppkt->hci_acl_data_pkt[5],
ppkt->hci_acl_data_pkt[6],
ppkt->hci_acl_data_pkt[7],
ppkt->hci_acl_data_pkt[8],
ppkt->hci_acl_data_pkt[9]);
#endif
            if (avail_length > LC_MAX_NUMBER_OF_BYTES_IN_BC_PACKET)
            {
                avail_length = LC_MAX_NUMBER_OF_BYTES_IN_BC_PACKET;
            }
            *pkt_type_lut = BB_DM1_LUT;
        } /* end of if (pkt->am_addr != BC_AM_ADDR) */

        *length = avail_length;
        *am_addr = pkt->am_addr;

        pkt->in_transmission = TRUE;
        pkt->frame_pkt_count = 1;

        if (pkt->tx_length1 == 0x0)
        {
            *frag_pointer = 0x0;
            pkt->tx_length1 = MIN(avail_length, ppkt->acl_data_total_length);
            avail_length -= pkt->tx_length1;
            pkt->total_frame_length = pkt->tx_length1;
        }
        else
        {
            UINT16 left_len;

            *frag_pointer = pkt->tx_length1;

            if (pkt->tx_length2 != 0x0)
            {
                pkt->tx_length1 += pkt->tx_length2;
                pkt->tx_length2 = 0x0;
            }

            left_len = ppkt->acl_data_total_length - pkt->tx_length1;
            pkt->tx_length2 = MIN(avail_length, left_len);
            avail_length -= pkt->tx_length2;
            pkt->total_frame_length += pkt->tx_length2;

        } /* end of if (pkt->tx_length1 == 0x0) */

        ppkt1 = ppkt->next;

        while ((ppkt1 != NULL) && (avail_length > 0))
        {
            pkt1 = ppkt1->ws;
            if ((pkt1->am_addr == pkt->am_addr) &&
                    (pkt1->phy_piconet_id == pkt->phy_piconet_id) &&
                    (ppkt1->packet_boundary_flag == L_CH_L2CAP_CONT))
            {
                /* This should be the start of the pkt */
                pkt1->tx_length1 = MIN(avail_length,
                                      ppkt1->acl_data_total_length);
                avail_length -= pkt1->tx_length1;
                pkt->total_frame_length += pkt1->tx_length1;

                pkt1->in_transmission = TRUE;
                pkt1->frame_pkt_count = 1;
                pkt->frame_pkt_count += 1;
            }
            ppkt1 = ppkt1->next;
        } /* end of while ((pkt1 != NULL) && (avail_length > 0)) */

        if (avail_length != 0x0)
        {
            RT_BT_LOG(GRAY, BT_FW_ACL_Q_714, 1, avail_length);
            /* This error should not happen, if occurred then handle it,
            * correcting the acl length
            * */
            *length -= avail_length;
        }

        pkt->acl_pkt_nbc_old_count = 0x0;
        pkt->acl_pkt_tx_failed = 0x0;
        pkt->selected_pkt_type = *pkt_type_lut;
        pkt->failed_frame_length = *length;

        break;
    }

    if (nothing == TRUE)
    {
        return NULL;
    }

    if (is_send_zero_length_acl_pkt &&
        (ce_index != INVALID_CE_INDEX))
    {
        /* Queue zero length pkt */

        LMP_CONNECTION_ENTITY *ce_ptr;
        ce_ptr = &lmp_connection_entity[ce_index];

        ppkt = aclq_get_zero_length_pkt(piconet_id);
        HCI_ACL_DATA_ZERO_PKT *zero_pkt;
        zero_pkt = (HCI_ACL_DATA_ZERO_PKT *)ppkt;
        zero_pkt->ws.am_addr = ce_ptr->am_addr;
        ppkt->connection_handle = ce_ptr->connection_type.connection_handle;
        *am_addr = ce_ptr->am_addr;
        *length = 0x0;
        *frag_pointer = 0x0;
        *pkt_type_lut = BB_DM1_LUT;
        ce_ptr->aclq_resch_flag = RESCHEDULE_FLAG_NONE;
        ce_ptr->is_last_sent_zero_l_l2cap = 0x1;
#ifdef _DAPE_TEST_SEND_ZERO_LEN_PKT_BY_VENDOR_CMD
        RT_BT_LOG(WHITE, DAPE_TEST_LOG556, 0, 0);
#endif

    }

    return ppkt;
}

/**
* Writes the packet to the BB_FIFO. This function takes care of re-assembly
*  of ACL packets done in the Controller, and combines the buffers and
*  writes the packets into BB-FIFO.
*
* \param pkt The ACL packet pointer.
* \param length The length of the packet.
* \param am_addr The am_addr of the connection.
* \param piconet_id Whether to use which piconet id
*
* \return length_written The number of bytes written into the BB FIFO.
*/
UINT16 aclq_write_to_baseband_TX_FIFO(HCI_ACL_DATA_PKT* ppkt,
                                      UINT16 length, UCHAR am_addr, UCHAR piconet_id)
{
    HCI_ACL_DATA_PKT* ppkt1;
    UCHAR* buf;
    UINT16 buf_length;
    UINT16 length_written = 0x0;
    UINT8 ori_seg = 0;
    UINT8 cur_seg = 0;
    UINT8 i;
    BZDMA_TX_DESC_SEGMENT TxDescTmp[BZDMA_UNICAST_ACL_TX_SEG];
    BZDMA_TX_DESC_SEGMENT *pTxDesc;
    UINT16 dma_total_len = 0;

    HCI_ACL_DATA_PKT_WS *pkt;
    HCI_ACL_DATA_PKT_WS *pkt1;

    /* TODO: Please split
    * BB_write_baseband_TX_FIFO_aclu/BB_write_baseband_TX_FIFO_aclu_scatternet
    * (Please only one interface)
    * to 2 part
    * 1: Header part
    * 2: Data loading
    */

    /* TODO: Write header part */
    /* Handle zero length acl pkt */

    LC_PICONET_SCHEDULER *piconet_sched;
    LC_SCHEDULED_PKT *schd;
    piconet_sched = &lc_piconet_scheduler[piconet_id];
    schd = &piconet_sched->lc_scheduled_pkt_info[piconet_sched->wptr];

    if ((length == 0) ||
        (aclq_is_zero_length_pkt(ppkt, piconet_id) == TRUE) ||
        (ppkt == NULL))
    {
        schd->txdesc_used_cnt = 0;
        return 0x00;
    }

    pkt = ppkt->ws;

    pTxDesc = schd->txdesc;

    if (pkt->tx_length2 != 0x0)
    {
        buf = &ppkt->hci_acl_data_pkt[pkt->tx_length1];
        buf_length = pkt->tx_length2;
    }
    else
    {
        buf = &ppkt->hci_acl_data_pkt[pkt->ackd_length];
        buf_length = (pkt->tx_length1 - pkt->ackd_length);
    }

    TxDescTmp[ori_seg].DWord0 = 0;
    TxDescTmp[ori_seg].DWord1 = 0;
    TxDescTmp[ori_seg].start_addr = (UINT32)buf;

    if ((dma_total_len + buf_length) > length)
    {
        TxDescTmp[ori_seg].len = length;
        dma_total_len = length;
    }
    else
    {
        TxDescTmp[ori_seg].len = buf_length;
        dma_total_len += buf_length;
    }

    length_written += buf_length;

    ppkt1 = ppkt->next;

    while ((ppkt1 != NULL) && (length_written < length))
    {
        pkt1 = ppkt1->ws;;

        if ((pkt1->am_addr == pkt->am_addr) &&
            (pkt1->phy_piconet_id == pkt->phy_piconet_id) &&
            (ppkt1->packet_boundary_flag == L_CH_L2CAP_CONT))
        {
            buf = &ppkt1->hci_acl_data_pkt[pkt1->ackd_length];
            buf_length = (pkt1->tx_length1 - pkt1->ackd_length);

            if (buf_length > 0)
            {
                ori_seg++;

                if (ori_seg >= BZDMA_UNICAST_ACL_TX_SEG)
                {
                    RT_BT_LOG(GRAY, BT_FW_ACL_Q_0380_1, 6,
                              BZDMA_UNICAST_ACL_TX_SEG-1, ori_seg, length,
                              length_written, buf_length, ppkt1);
                }

                TxDescTmp[ori_seg].DWord0 = 0;
                TxDescTmp[ori_seg].DWord1 = 0;
                TxDescTmp[ori_seg].start_addr = (UINT32)buf;
                TxDescTmp[ori_seg].len = buf_length;

                if ((dma_total_len + buf_length) > length)
                {
                    TxDescTmp[ori_seg].len = length - dma_total_len;
                    dma_total_len = length;
                }
                else
                {
                    TxDescTmp[ori_seg].len = buf_length;
                    dma_total_len += buf_length;
                }
            }

            length_written += buf_length;
        }
        ppkt1 = ppkt1->next;
    }

    if (length != length_written)
    {
        RT_BT_LOG(GRAY, BT_FW_ACL_Q_814, 2, length, length_written);
    }

    UINT8 *cur_data_buf;
    UINT8 *next_data_buf;

    /* workaround for BZDMA in legacy Tx Command */
    for (i = 0; i < ori_seg; i++)
    {
        if (TxDescTmp[i].len & 0x1) /* cur segment length is odd */
        {
            cur_data_buf = (UINT8*)NON_DCATCH(TxDescTmp[i].start_addr);

            /* need copy 1 byte from next segment */
            next_data_buf = (UINT8*)NON_DCATCH(TxDescTmp[i + 1].start_addr);
            cur_data_buf[TxDescTmp[i].len] = next_data_buf[0];
            TxDescTmp[i].len++;
            TxDescTmp[i+1].len--;
            TxDescTmp[i+1].start_addr++;

            pTxDesc[cur_seg].DWord0 = TxDescTmp[i].DWord0;
            pTxDesc[cur_seg].DWord1 = TxDescTmp[i].DWord1;

            if (TxDescTmp[i+1].len == 0)
            {
                i++;
            }
        }
        else
        {
            pTxDesc[cur_seg].DWord0 = TxDescTmp[i].DWord0;
            pTxDesc[cur_seg].DWord1 = TxDescTmp[i].DWord1;
        }
        cur_seg++;
    }

    if ((ori_seg == 0) || (TxDescTmp[ori_seg].len != 0))
    {
        /* only one segmnet or last segment has non-zero length data */
        pTxDesc[cur_seg].DWord0 = TxDescTmp[ori_seg].DWord0;
        pTxDesc[cur_seg].DWord1 = TxDescTmp[ori_seg].DWord1;
        pTxDesc[cur_seg].isLast = 1;
        cur_seg++;
    }
    else
    {
        /* last segment has zero length data, so last segment is previous one */
        pTxDesc[cur_seg - 1].isLast = 1;
    }

    schd->txdesc_used_cnt = cur_seg;

    return length_written;
}

/**
* [External interface] Used to ack a fragrament of the acl_pkt.
* This function will dequeue (but not free) the acl_pkt if complete
* packet has been transmitted.
*
* \param pkt Pointer to the acl_data_pkt.
* \param length length to be acked.
* \param is_to_generate_nce indicate whther to generate num complete or not.
*
* \return TRUE or FALSE If TRUE the calling function should free
*                       the acl_pkt.
*/
UCHAR aclq_ack_acl_pkt(HCI_ACL_DATA_PKT *ppkt, UINT16 length,
                       UCHAR is_to_generate_nce, UCHAR is_timeout)
{
    HCI_ACL_DATA_PKT* ppkt1;
    UCHAR frame_pkt_count;
    UINT16 ackd_length;
    UINT16 ce_index = INVALID_CE_INDEX;
    UCHAR bc_flag;
    UCHAR num_completed_pkts = 0x0;
    HCI_ACL_DATA_PKT_WS *pkt;
    HCI_ACL_DATA_PKT_WS *pkt1;

    if (aclq_is_acl_queue_empty() == TRUE)
    {
        return FALSE;
    }

    /* Handle zero length acl pkt */
    if (aclq_is_one_of_zero_length_pkt(ppkt) == TRUE)
    {
        return FALSE;
    }

    pkt = ppkt->ws;

    if (pkt->in_transmission != TRUE)
    {
        return FALSE;
    }

    pkt->ackd_length = (UINT16)(pkt->ackd_length + length);

    LMP_GET_CE_INDEX_FROM_CONN_HANDLE((UINT16)(ppkt->connection_handle),
                                      &ce_index);

    LMP_CONNECTION_ENTITY *ce_ptr = NULL;

    if (ce_index == INVALID_CE_INDEX)
    {
        RT_BT_LOG(GRAY, BT_FW_ACL_Q_866, 1, ppkt->connection_handle);
    }
    else
    {
        ce_ptr = &lmp_connection_entity[ce_index];

        /*
         * Check for flush timeout pkt.
         * Take appropriate actions.
         */
        if ((is_timeout == TRUE) ||
            (lc_is_pkt_timeout(ppkt, ce_ptr) == TRUE))
        {
            if (pkt->ackd_length >= (pkt->tx_length1 + pkt->tx_length2))
            {
                /* There is no pending fragment */
                /* So we can flush this pkt safely */

                /* Flush all the continous pkts */
                ppkt1 = ppkt->next;
                pkt->ackd_length = ppkt->acl_data_total_length;
                pkt->frame_pkt_count = 1;

                while (ppkt1 != NULL)
                {
                    pkt1 = ppkt1->ws;

                    if ((pkt1->am_addr == pkt->am_addr) &&
                        (pkt1->phy_piconet_id == pkt->phy_piconet_id))
                    {
                        if (ppkt1->packet_boundary_flag == L_CH_L2CAP_CONT)
                        {
                            pkt->ackd_length += ppkt1->acl_data_total_length;
                            pkt->frame_pkt_count++;
                        }
                        else
                        {
                            /* There is a L2CAP start pkt, so  don't try to
                             * delete this pkt
                             */
                            break;
                        }
                    }
                    ppkt1 = ppkt1->next;
                }

                if (ce_ptr->is_last_sent_zero_l_l2cap == 0x0)
                {
                    /* Send Zero length l2cap pkt */
                    ce_ptr->aclq_resch_flag = RESCHEDULE_FLAG_ZERO_LENGTH;
                }

                check_flush_occured_event_generation(ce_index);
            }
        }
    }

    if (pkt->ackd_length < ppkt->acl_data_total_length)
    {
        return FALSE;
    }

    pkt->acl_pkt_nbc_old_count = 0x0;
    frame_pkt_count = pkt->frame_pkt_count;
    ackd_length = pkt->ackd_length;

    while ((ppkt != NULL) && (ackd_length > 0x0))
    {
        pkt = ppkt->ws;

        /* This for iteration */
        pkt->ackd_length = ackd_length;

        bc_flag = (UCHAR) ppkt->broadcast_flag;

        ppkt1 = ppkt->next;

        while ((ppkt1 != NULL) && (frame_pkt_count > 1))
        {
            pkt1 = ppkt1->ws;

            if ((pkt1->am_addr == pkt->am_addr) &&
                (pkt1->phy_piconet_id == pkt->phy_piconet_id) &&
                (ppkt1->packet_boundary_flag == L_CH_L2CAP_CONT))
            {
                frame_pkt_count--;
                break;
            }
            ppkt1 = ppkt1->next;
        }

        if (pkt->ackd_length >= ppkt->acl_data_total_length)
        {
            ackd_length -= ppkt->acl_data_total_length;

            aclq_dequeue_acl_pkt(ppkt);

            /* Update and Send num complete */
            /* Free the buffer */

            /*************************************************************/
            if (dma_tx_fifo_pkt_free((void * )ppkt,
                        HCI_TRANSPORT_ACL_DATA_PKT_TYPE) != BT_ERROR_OK)
            {
                RT_BT_LOG(GRAY, BT_FW_ACL_Q_962, 0, 0);
            }
            num_completed_pkts++;

            switch(bc_flag)
            {
                case LC_DATA_ACTIVE_FLAG:
                    ce_ptr->hc_num_of_completed_packets++;
                    break;
#ifdef BROADCAST_DATA
                case LC_ACTIVE_BC_FLAG:
                    lmp_self_device_data.asb_num_of_completed_packets++;
                    break;
                case LC_PNET_BC_FLAG:
                    lmp_self_device_data.psb_num_of_completed_packets++;
                    break;
#endif
                default:
                    break;
            }
            /*************************************************************/
        }
        else
        {
            ackd_length = 0x0;
        }
        ppkt = ppkt1;
    }

    if ((is_to_generate_nce == 0x1) && (num_completed_pkts > 0x0))
    {
        SEND_NUM_COMPLETED_SIGNAL_TO_HOST(ce_index);
    }
    return TRUE;
}

/**
* [Internal interface] Used to dequeue a queued acl_pkt.
* Can be used as external interface also.
*
* \param fpkt Pointer to the acl_data_pkt to dequeue.
*
* \return None.
*/
void aclq_dequeue_acl_pkt(HCI_ACL_DATA_PKT *fpkt)
{
    HCI_ACL_DATA_PKT *ppkt;
    HCI_ACL_DATA_PKT *pkt;

    ppkt = NULL;
    pkt = acl_q_head;
    while (pkt != NULL)
    {
        if (pkt == fpkt)
        {
            if (ppkt == NULL)
            {
                /* fpkt is in the head */
                acl_q_head = acl_q_head->next;
                pkt->next = NULL;
                if (acl_q_head == NULL)
                {
                    /* No more pkt in the queue, make both head & tail NULL */
                    acl_q_tail = NULL;
                }
                return;
            }

            ppkt->next = pkt->next;
            pkt->next = NULL;

            if (pkt == acl_q_tail)
            {
                /* fpkt is in the tail, set tail to previous pkt */
                acl_q_tail = ppkt;
                acl_q_tail->next = NULL;
            }
            break;
        }
        ppkt = pkt;
        pkt = pkt->next;
    }
}

/**
* [External interface] Resets acl_pkt which is in transmission.
* This function will mark the am_addr as failed and keep the ackd length
* so fragments from ackd length onwards will be resent when this packet
* is scheduled.
* The pkt has been NBC timed out.
*
* \param pkt Pointer to the acl_data_pkt.
*
* \return None.
*/
//#define NO_NBC_ACL_QUEUE_FIX
void aclq_reset_nbc_acl_pkt(HCI_ACL_DATA_PKT *ppkt, UCHAR phy_piconet_id,
                            UINT16 failed_pkt_length, UINT16 failed_pkt_type)
{
    HCI_ACL_DATA_PKT_WS *pkt = ppkt->ws;

#ifdef COMPILE_SNIFF_MODE
    UINT16 ce_index;

    LMP_CONNECTION_ENTITY *ce_ptr = NULL;

    LMP_GET_CE_INDEX_FROM_CONN_HANDLE((UINT16)(ppkt->connection_handle),
                                      &ce_index);

    if (ce_index != INVALID_CE_INDEX)
    {
        ce_ptr = &lmp_connection_entity[ce_index];
    }
    else
    {
        //RT_BT_LOG(GRAY, BT_FW_ACL_Q_INVALID, 0, 0);
        //return;
    }

    if ((ce_index != INVALID_CE_INDEX) &&
            (ce_ptr->in_sniff_mode == TRUE) &&
            (ce_ptr->sniff_interval >
             LMP_MIN_SNIFF_INTERVAL_TO_SKIP_DATA_TX_OPT))
    {
        /* Do not mark this link as invalid */
    }
    else if ((ce_index != INVALID_CE_INDEX) &&
             (ce_ptr->in_sniff_mode == TRUE) &&
             ((ce_ptr->next_instant_in_nat_clk -
               BB_read_native_clock()) <=
              LC_MIN_SLOTS_TO_SKIP_MARK_AS_BAD_FOR_SNIFF))
    {
        /* Do not mark this link as invalid, sniff instant is approaching */
    }
    else
#endif
    {
        aclq_mark_am_addr_as_failed(pkt->am_addr, phy_piconet_id);
    }
    if (aclq_is_zero_length_pkt(ppkt, phy_piconet_id) == FALSE)
    {
        aclq_reset_acl_pkt_wo_failed(ppkt,phy_piconet_id);
#ifndef NO_NBC_ACL_QUEUE_FIX
        pkt->failed_frame_length = failed_pkt_length;
        pkt->selected_pkt_type = failed_pkt_type;
        pkt->acl_pkt_tx_failed = 0x1;
        pkt->acl_pkt_nbc_old_count = 0x0;
#endif
    }
}

/**
* [External interface] Resets acl_pkt which is in transmission.
* This function will mark the am_addr as failed and keep the ackd length
* so fragments from ackd length onwards will be resent when this packet
* is scheduled.
*
* \param pkt Pointer to the acl_data_pkt.
*
* \return None.
*/
#if 0 /* no one use function aclq_reset_acl_pkt, hence we comment it */
void aclq_reset_acl_pkt(HCI_ACL_DATA_PKT *ppkt, UCHAR phy_piconet_id)
{
    HCI_ACL_DATA_PKT_WS *pkt = ppkt->ws;

    aclq_mark_am_addr_as_failed(pkt->am_addr, phy_piconet_id);
    aclq_reset_acl_pkt_wo_failed(ppkt,phy_piconet_id);
}
#endif
/**
* [External interface] Resets acl_pkt which is in transmission.
* This function will NOT mark the am_addr as failed and keep the ackd length
* so fragments from ackd length onwards will be resent when this packet
* is scheduled.
*
* \param pkt Pointer to the acl_data_pkt.
*
* \return None.
*/
void aclq_reset_acl_pkt_wo_failed(HCI_ACL_DATA_PKT *ppkt, UCHAR phy_piconet_id)
{
    HCI_ACL_DATA_PKT_WS *pkt;
    UINT8 am_addr;

    if ((aclq_is_zero_length_pkt(ppkt, phy_piconet_id) == TRUE) ||
        (ppkt == NULL))
    {
        return;
    }

    pkt = ppkt->ws;

    am_addr = pkt->am_addr;
    phy_piconet_id = pkt->phy_piconet_id;

    /* Keep the acked length as such */
    /* We can reset the entire packet for this am_addr */
    /* Mark all the pkts in this link as in Transmission == FALSE */
    while (ppkt != NULL)
    {
        pkt = ppkt->ws;

        if ((pkt->am_addr == am_addr) &&
             (pkt->phy_piconet_id == phy_piconet_id))
        {
            pkt->in_transmission = FALSE;
            pkt->tx_length1 = pkt->ackd_length;
            pkt->tx_length2 = 0x0;
            pkt->total_frame_length = 0x0;
            pkt->frame_pkt_count = 0x0;
        }
        ppkt = ppkt->next;
    }
}

/**
* [Internal interface] Returns TRUE if acl_queue is empty.
* Can be used as external interface also.
*
* \param None..
*
* \return TRUE or FALSE.
*/
UCHAR aclq_is_acl_queue_empty()
{
    if (acl_q_head == NULL)
    {
        return TRUE;
    }
    return FALSE;
}

/**
* [Internal interface] Mark the am_addr as failed (i.e. low-priority).
* Can be used as external interface also.
*
* \param am_addr AM Address.
*
* \return None.
*/
void aclq_mark_am_addr_as_failed(UCHAR am_addr, UCHAR piconet_id)
{
    UCHAR lut_index;

    lut_index = lc_get_lut_index_from_phy_piconet_id(am_addr, piconet_id);

    failed_lut_index[lut_index] = TRUE;
}

/**
* Returns number of acl packets in the aclq list
*
* \param am_addr AM Address.
* \param piconet piconet id.
* \param *length total length.
* \param near_end count from near end or not.
*
*
* \return number of acl packets.
*/
UINT16 aclq_get_no_of_pkts_am_addr(UCHAR am_addr, UCHAR piconet,
                                            UINT32 *length, UINT8 near_end)
{
    HCI_ACL_DATA_PKT *pkt;
    HCI_ACL_DATA_PKT_WS *ws;

    UINT16 num_of_pkts = 0;
    UINT32 total_acl_length = 0;

    pkt = acl_q_head;

    while (pkt != (HCI_ACL_DATA_PKT *)NULL)
    {
        ws = pkt->ws;

        if ((ws->am_addr == am_addr) && (ws->phy_piconet_id == piconet))
        {
            num_of_pkts++;
            total_acl_length += pkt->acl_data_total_length;
        }
        else if (near_end)
        {
            break;
        }

        pkt = pkt->next;
    }

    *length = total_acl_length;
    return num_of_pkts;
}

/**
* [External interface] Returns TRUE if am_addr is marked as low-prioriy.
* Can be used as external interface also.
*
* \param am_addr AM Address.
*
* \return TRUE or FALSE.
*/
UCHAR aclq_is_am_addr_failed(UCHAR am_addr, UCHAR phy_piconet_id)
{
    UCHAR lut_index;

    lut_index = lc_get_lut_index_from_phy_piconet_id(am_addr, phy_piconet_id);

    return failed_lut_index[lut_index];
}

/**
* [External interface] Mark the am_addr as active (i.e. normal priority).
*
* \param am_addr AM Address.
*
* \return None.
*/
void aclq_mark_am_addr_as_active(UCHAR am_addr, UCHAR phy_piconet_id)
{
    UCHAR lut_index;

    lut_index = lc_get_lut_index_from_phy_piconet_id(am_addr, phy_piconet_id);

    failed_lut_index[lut_index] = FALSE;
}

/**
* [External interface] Mark the am_addr as received flow zero.
*
* \param am_addr AM Address.
*
* \return None.
*/
void aclq_mark_am_addr_flow_stop(UCHAR am_addr, UCHAR phy_piconet_id)
{
    UCHAR lut_index;

    lut_index = lc_get_lut_index_from_phy_piconet_id(am_addr, phy_piconet_id);

    flow_enabled_lut_index[lut_index] = FALSE;
    //RT_BT_LOG(GRAY, BT_FW_ACL_Q_1263, 1, lut_index);
}

/**
* [External interface] Mark the am_addr as received flow one.
*
* \param am_addr AM Address.
*
* \return None.
*/
void aclq_mark_am_addr_flow_go(UCHAR am_addr, UCHAR phy_piconet_id)
{
    OS_SIGNAL sig_send;

    UCHAR lut_index;

    lut_index = lc_get_lut_index_from_phy_piconet_id(am_addr, phy_piconet_id);

    flow_enabled_lut_index[lut_index] = TRUE;
    //RT_BT_LOG(GRAY, BT_FW_ACL_Q_1288, 1, lut_index);

    sig_send.type = LC_RESUME_DATA_TRANSFER_SIGNAL;
    sig_send.param = (void*)((UINT32) phy_piconet_id);

    OS_SEND_SIGNAL_TO_TASK(lc_tx_task_handle, sig_send);

    return;
}

/**
* [External interface] Mark the am_addr as paused. So none of the acl data
* packets for this am_address will be transmitted until resume_am_addr is
* called.
*
* \param am_addr AM Address.
* \param phy_piconet_id Piconet ID of the connection.
* \param flag The feature that is pausing.
*
* \return None.
*/
#ifdef COMPILE_NESTED_PAUSE_RESUME
void aclq_mark_am_addr_as_paused(UCHAR am_addr, UCHAR phy_piconet_id,
                                 UINT16 flag)
#else /* COMPILE_NESTED_PAUSE_RESUME */
void aclq_mark_am_addr_as_paused(UCHAR am_addr, UCHAR phy_piconet_id)
#endif /* COMPILE_NESTED_PAUSE_RESUME */
{
    UCHAR lut_index;

    lut_index = lc_get_lut_index_from_phy_piconet_id(am_addr, phy_piconet_id);

#ifdef COMPILE_NESTED_PAUSE_RESUME
    paused_lut_index[lut_index] |= flag;
#else /* COMPILE_NESTED_PAUSE_RESUME */
    paused_lut_index[lut_index] = TRUE;
#endif /* COMPILE_NESTED_PAUSE_RESUME */

    return;
}

/**
* [External interface] Returns TRUE if the not recieved L2CAP flow zero
* on am address.
*
* \param am_addr AM Address.
*
* \return TRUE or FALSE.
*/
UCHAR aclq_is_am_addr_flow_enabled(UCHAR am_addr, UCHAR phy_piconet_id)
{
    UCHAR ret_val;
    UCHAR lut_index;

    lut_index = lc_get_lut_index_from_phy_piconet_id(am_addr, phy_piconet_id);

    ret_val = flow_enabled_lut_index[lut_index];

    return ret_val;
}

/**
* [External interface] Resume data transfer for the paused am_address.
*
* \param am_addr AM Address.
*
* \return None.
*/
#ifdef COMPILE_NESTED_PAUSE_RESUME
void aclq_resume_am_addr(UCHAR am_addr, UCHAR phy_piconet_id, UINT16 flag)
#else /* COMPILE_NESTED_PAUSE_RESUME */
void aclq_resume_am_addr(UCHAR am_addr, UCHAR phy_piconet_id)
#endif /* COMPILE_NESTED_PAUSE_RESUME */
{
    OS_SIGNAL sig_send;

    UCHAR lut_index;

    lut_index = lc_get_lut_index_from_phy_piconet_id(am_addr, phy_piconet_id);

#ifdef COMPILE_NESTED_PAUSE_RESUME
    paused_lut_index[lut_index] &= ~flag;
#else /* COMPILE_NESTED_PAUSE_RESUME */
    paused_lut_index[lut_index] = FALSE;
#endif /* COMPILE_NESTED_PAUSE_RESUME */

    /**
    * Send Signal to invoke_scheduler
    * See also: lmp_resume_data_transfer
    */
    sig_send.type = LC_RESUME_DATA_TRANSFER_SIGNAL;
    sig_send.param = (void*)((UINT32) phy_piconet_id);
    OS_SEND_SIGNAL_TO_TASK(lc_tx_task_handle, sig_send);

    return;
}

/**
* Returns TRUE if the am_addr is not paused,
* nor has received L2CAP flow zero.
*
* \param am_addr AM Address.
*
* \return TRUE or FALSE.
*/
UCHAR aclq_is_am_addr_schedulable(UCHAR am_addr, UCHAR phy_piconet_id,
        HCI_ACL_DATA_PKT *ppkt, UCHAR aclq_pkt_nbc_oldest_count)
{
    UCHAR ret_val = FALSE;
    UCHAR lut_index;
    UCHAR temp_var;
#ifndef _ENABLE_COMPILE_NESTED_PAUSE_RESUME_
    UCHAR pause_status = FALSE;
#endif
    HCI_ACL_DATA_PKT_WS *pkt = ppkt->ws;

    if (am_addr == BC_AM_ADDR)
    {
        return TRUE;
    }

    for (temp_var = 0; temp_var < LMP_MAX_CE_DATABASE_ENTRIES; temp_var++)
    {
        LMP_CONNECTION_ENTITY *ce_ptr;

        ce_ptr = &lmp_connection_entity[temp_var];

        if ((ce_ptr->entity_status != UNASSIGNED) &&
            (ce_ptr->am_addr == am_addr) &&
            (ce_ptr->phy_piconet_id == phy_piconet_id))
        {
#ifdef COMPILE_SNIFF_MODE
            if ((ce_ptr->in_sniff_mode == TRUE) &&
                (ce_ptr->sniff_interval >
                                LMP_MIN_SNIFF_INTERVAL_TO_SKIP_DATA_TX_OPT))
            {
                pkt->acl_pkt_nbc_old_count = 0x0;
#ifndef _ENABLE_COMPILE_NESTED_PAUSE_RESUME_
                pause_status = ce_ptr->pause_data_transfer;
#endif
                continue;
            }
#endif
            if (ce_ptr->aclq_resch_flag == RESCHEDULE_FLAG_ACL)
            {
                if ((pkt->acl_pkt_tx_failed == 0x1) &&
                    (pkt->acl_pkt_nbc_old_count >= aclq_pkt_nbc_oldest_count))
                {
                    return TRUE;
                }
            }
#ifndef _ENABLE_COMPILE_NESTED_PAUSE_RESUME_
            pause_status = ce_ptr->pause_data_transfer;
#endif
            break;
        }
    }

    lut_index = lc_get_lut_index_from_phy_piconet_id(am_addr, phy_piconet_id);
#ifdef _ENABLE_COMPILE_NESTED_PAUSE_RESUME_
    if (!paused_lut_index[lut_index] && flow_enabled_lut_index[lut_index])
#else
    if (!paused_lut_index[lut_index] && flow_enabled_lut_index[lut_index] &&
            (pause_status == FALSE))
#endif
    {
        ret_val = TRUE;
    }

    return ret_val;
}

/**
* [Internal interface] Get the first available acl_pkt for the am_address.
* Used for flushing & RE-Queuing.
*
* \param am_addr AM Address.
*
* \return HCI_ACL_DATA_PKT* pointer to the acl_packet.
*/
HCI_ACL_DATA_PKT *aclq_get_acl_pkt_am_addr(UCHAR am_addr, UCHAR piconet_id)
{
    HCI_ACL_DATA_PKT *ppkt;
    HCI_ACL_DATA_PKT_WS *pkt;

    ppkt = acl_q_head;

    while (ppkt != (HCI_ACL_DATA_PKT *)NULL)
    {
        pkt = ppkt->ws;

        if ((pkt->am_addr == am_addr) && (pkt->phy_piconet_id == piconet_id))
        {
            return ppkt;
        }
        ppkt = ppkt->next;
    }
    return NULL;
}

/**
* [External interface] Re-Queue all the acl-pkts on a different
* am_addr. Useful for role-switch.
*
* \param old_am_addr Old AM Address.
* \param new_am_addr New AM Address.
*
* \return None.
*/
void aclq_requeue_all_pkts(void)
{
    HCI_ACL_DATA_PKT *ppkt;
    UCHAR old_piconet_id;
    UCHAR new_piconet_id;
    HCI_ACL_DATA_PKT_WS *pkt;
    UCHAR old_lut_index;
    UCHAR new_lut_index;
    UCHAR old_am_addr;
    UCHAR new_am_addr;

    old_am_addr = lmp_role_switch_data.old_am_addr;
    new_am_addr = lmp_role_switch_data.new_am_addr;

    old_piconet_id = lmp_role_switch_data.old_piconet_id;
    new_piconet_id = lmp_role_switch_data.new_piconet_id;

    old_lut_index = lmp_role_switch_data.old_lut_index;
    new_lut_index = lmp_role_switch_data.new_lut_index;

    if ((old_am_addr == new_am_addr) &&
        (old_piconet_id == new_piconet_id))
    {
        return;
    }

    while (1)
    {
        ppkt = aclq_get_acl_pkt_am_addr(old_am_addr, old_piconet_id);

        if (ppkt == (HCI_ACL_DATA_PKT *)NULL)
        {
            break;
        }

        pkt = ppkt->ws;
        pkt->am_addr = new_am_addr;
        pkt->phy_piconet_id = new_piconet_id;
    }
#ifdef _ENABLE_COMPILE_NESTED_PAUSE_RESUME_
    if (paused_lut_index[old_lut_index])
    {
        paused_lut_index[new_lut_index] = paused_lut_index[old_lut_index];
    }
#else
    if (paused_lut_index[old_lut_index] == TRUE)
    {
        paused_lut_index[new_lut_index] = TRUE;
    }
#endif
    flow_enabled_lut_index[new_lut_index] = TRUE;

    paused_lut_index[old_lut_index] = FALSE;
    failed_lut_index[old_lut_index] = FALSE;
    flow_enabled_lut_index[old_lut_index] = TRUE;

    return;
}

/**
* [External interface] Dequeue and free all packets for an am_address.
*
* \param am_addr AM Address.
* \param generate_num_complete If TRUE generates num_complete events to
*                              host.
*
* \return TRUE if an acl-packet was dequeued.
*/
UCHAR aclq_clear_all_pkts_am_addr(UCHAR am_addr, UCHAR generate_num_complete,
                                  UCHAR piconet_id)
{
    HCI_ACL_DATA_PKT *pkt;
    UINT16 ce_index = INVALID_CE_INDEX;
    UCHAR get_ce_index = 1;
    UCHAR ret_status = FALSE;
#ifdef BROADCAST_DATA
    UCHAR bc_flag = LC_ACTIVE_BC_FLAG;
#endif

    while (1)
    {
        pkt = aclq_get_acl_pkt_am_addr(am_addr, piconet_id);

        if (pkt == NULL)
        {
            break;
        }

        aclq_dequeue_acl_pkt(pkt);
#ifdef BROADCAST_DATA
        bc_flag = (UCHAR) pkt->broadcast_flag;
#endif
        ret_status = TRUE;
        if (get_ce_index == 1)
        {
            LMP_GET_CE_INDEX_FROM_CONN_HANDLE((UINT16)pkt->connection_handle,
                                               &ce_index);
            get_ce_index = 0;
        }

        if (dma_tx_fifo_pkt_free((void * )pkt,
                            HCI_TRANSPORT_ACL_DATA_PKT_TYPE) != BT_ERROR_OK)
        {
            LC_LOG_INFO(LOG_LEVEL_LOW,OS_FREE_BUFFER_FAILED,0,0);
        }

        if (generate_num_complete == TRUE)
        {
            if (ce_index != INVALID_CE_INDEX)
            {
                lmp_connection_entity[ce_index].hc_num_of_completed_packets++;
            }
#ifdef BROADCAST_DATA
            else if (am_addr == BC_AM_ADDR)
            {
                if (bc_flag == LC_ACTIVE_BC_FLAG)
                {
                    lmp_self_device_data.asb_num_of_completed_packets++;
                }
                else
                {
                    lmp_self_device_data.psb_num_of_completed_packets++;
                }
            }
#endif
            SEND_NUM_COMPLETED_SIGNAL_TO_HOST(ce_index);
        }
    }

    return ret_status;
}

/**
* Mark all ACL packets for a particular am_addr for flushing.
* The packets will be flushed when scheduling for transmission.
*
* \param am_addr AM Address.
*
* \return TRUE if one packet was marked FALSE otherwise.
*/
UCHAR aclq_mark_all_for_flush(UCHAR am_addr, UCHAR piconet_id)
{
    HCI_ACL_DATA_PKT *ppkt;
    UCHAR status = FALSE;
    HCI_ACL_DATA_PKT_WS *pkt;

    ppkt = acl_q_head;
    while (ppkt != (HCI_ACL_DATA_PKT *)NULL)
    {
        pkt = ppkt->ws;

        if ((pkt->am_addr == am_addr) && (pkt->phy_piconet_id == piconet_id))
        {
            /* Mark for flush */
            pkt->flush_time = FORCE_FLUSH_CLOCK;
            status = TRUE;
        }
        ppkt = ppkt->next;
    }
    return status;
}

/**
* [External interface] Set the acl_pkts to FORCE_FLUSH.
*
* \param am_addr AM Address.
* \param packet_boundary_flag Packet boundary of the pkt.
*
* \return None.
*/
void aclq_acl_enhanced_flush(UCHAR am_addr, UCHAR packet_boundary_flag, UCHAR piconet_id)
{
    HCI_ACL_DATA_PKT *ppkt = acl_q_head;
    HCI_ACL_DATA_PKT_WS *pkt;

    while (ppkt != NULL)
    {
        pkt = ppkt->ws;

        if ((pkt->am_addr == am_addr) &&
            (ppkt->packet_boundary_flag == packet_boundary_flag) &&
            (pkt->phy_piconet_id == piconet_id))
        {
            pkt->flush_time = FORCE_FLUSH_CLOCK;
        }
        ppkt = ppkt->next;
    }
}

/**
* [External interface] Check if enhanced flush packets are pending
*
* \param am_addr AM Address.
*
* \return TRUE if enhance_flush is pending.
*/
UCHAR aclq_acl_check_flushed(UCHAR am_addr, UCHAR piconet_id)
{
    HCI_ACL_DATA_PKT *ppkt = acl_q_head;
    HCI_ACL_DATA_PKT_WS *pkt;

    while (ppkt != NULL)
    {
        pkt = ppkt->ws;

        if ((pkt->am_addr == am_addr) &&
                (pkt->flush_time == FORCE_FLUSH_CLOCK) &&
                (pkt->phy_piconet_id == piconet_id))
        {
            return TRUE;
        }
        ppkt = ppkt->next;
    }
    return FALSE;
}

/**
* [External interface] Returns the pointer to the static
* zero length acl packet.
* The pointer should not be tried to be freed.
*
* \param None.
*
* \return Pointer to ACL pkt (static/global).
*/

HCI_ACL_DATA_PKT *aclq_get_zero_length_pkt(UINT8 piconet_id)
{
    return (HCI_ACL_DATA_PKT*)&aclq_zero_length_pkt[piconet_id];
}

/**
* [External interface] Returns TRUE if zero length pkt.
*
* \param HCI_ACL_DATA_PKT* pkt
*
* \return TRUE if pkt is pointing to zero length pkt otherwize returns FALSE.
*
*/
UCHAR aclq_is_zero_length_pkt(HCI_ACL_DATA_PKT* pkt, UINT8 piconet_id)
{
    if (pkt == (HCI_ACL_DATA_PKT*)&aclq_zero_length_pkt[piconet_id])
    {
        return TRUE;
    }
    return FALSE;
}


UCHAR aclq_is_one_of_zero_length_pkt(HCI_ACL_DATA_PKT* pkt)
{
    if (((UINT32)pkt >= (UINT32)&aclq_zero_length_pkt[0]) &&
        ((UINT32)pkt <= (UINT32)&aclq_zero_length_pkt[MAX_PICONET_CNT - 1]))
    {
        UINT32 tmp = (UINT32)pkt - (UINT32)&aclq_zero_length_pkt[0];
        if (tmp % sizeof(HCI_ACL_DATA_ZERO_PKT))
        {
            return FALSE;
        }

        return TRUE;
    }
    return FALSE;
}


/* ---------------------------- End of bt_fw_acl_q.c ---------------------- */


