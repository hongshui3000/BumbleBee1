/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  ACL Queue interface.
 *
 * \author Muthu Subramanian K
 */

/** \addtogroup hci_external
 *  @{ */
#ifndef _BT_FW_ACL_Q_H_
#define _BT_FW_ACL_Q_H_

#include "le_ll.h"

void aclq_dequeue_acl_pkt(HCI_ACL_DATA_PKT *fpkt);
UINT16 aclq_get_no_of_pkts_am_addr(UCHAR am_addr, UCHAR piconet,
                                            UINT32 * length, UINT8 near_end);
UCHAR aclq_is_acl_queue_empty(void);

void aclq_mark_am_addr_as_failed(UCHAR am_addr, UCHAR phy_piconet_id);
UCHAR aclq_is_am_addr_failed(UCHAR am_addr, UCHAR phy_piconet_id);

#ifdef COMPILE_NESTED_PAUSE_RESUME
void aclq_mark_am_addr_as_paused(UCHAR am_addr, UCHAR phy_piconet_id,
                                                              UINT16 flag);
void aclq_resume_am_addr(UCHAR am_addr, UCHAR phy_piconet_id, UINT16 flag);
#else /* COMPILE_NESTED_PAUSE_RESUME */
void aclq_mark_am_addr_as_paused(UCHAR am_addr, UCHAR phy_piconet_id);
void aclq_resume_am_addr(UCHAR am_addr, UCHAR phy_piconet_id);
#endif /* COMPILE_NESTED_PAUSE_RESUME */

/* External interfaces */
void aclq_init(void);

void aclq_mark_am_addr_flow_stop(UCHAR am_addr, UCHAR phy_piconet_id);
void aclq_mark_am_addr_flow_go(UCHAR am_addr, UCHAR phy_piconet_id);
UCHAR aclq_is_am_addr_flow_enabled(UCHAR am_addr, UCHAR phy_piconet_id);

API_RESULT aclq_queue_acl_pkt(HCI_ACL_DATA_PKT *pkt);
UCHAR aclq_ack_acl_pkt(HCI_ACL_DATA_PKT * ppkt, UINT16 length,
                       UCHAR is_to_generate_nce, UCHAR is_timeout);

HCI_ACL_DATA_PKT *aclq_get_next_acl_pkt(UCHAR *am_addr,UINT16 *length,
             UINT16 *frag_pointer, UINT16 *pkt_type_lut, UCHAR phy_piconet_id);

void aclq_mark_am_addr_as_active(UCHAR am_addr, UCHAR phy_piconet_id);

UCHAR aclq_clear_all_pkts_am_addr(UCHAR am_addr,
            UCHAR generate_num_complete, UCHAR piconet_id);

UCHAR aclq_mark_all_for_flush(UCHAR am_addr, UCHAR piconet_id);

void aclq_requeue_all_pkts(void);

void aclq_acl_enhanced_flush(UCHAR am_addr, UCHAR packet_boundary_flag,
                                                        UCHAR piconet_id);

UCHAR aclq_acl_check_flushed(UCHAR am_addr, UCHAR piconet_id);

HCI_ACL_DATA_PKT *aclq_get_zero_length_pkt(UINT8 piconet_id);
UCHAR aclq_is_zero_length_pkt(HCI_ACL_DATA_PKT* pkt, UINT8 piconet_id);

UCHAR aclq_is_one_of_zero_length_pkt(HCI_ACL_DATA_PKT* pkt);
#if 0 /* no one use function aclq_reset_acl_pkt, hence we comment it */
void aclq_reset_acl_pkt(HCI_ACL_DATA_PKT *pkt, UCHAR phy_piconet_id);
#endif
void aclq_reset_acl_pkt_wo_failed(HCI_ACL_DATA_PKT *pkt, UCHAR phy_piconet_id);
void aclq_reset_nbc_acl_pkt(HCI_ACL_DATA_PKT *pkt, UCHAR phy_piconet_id,
        UINT16 failed_pkt_length, UINT16 failed_pkt_type);

void aclq_log_information(void);

UINT16 aclq_write_to_baseband_TX_FIFO(HCI_ACL_DATA_PKT* pkt,
    UINT16 length, UCHAR am_addr, UCHAR piconet_id);

#ifdef HS_USB_SEND_EVENT_REORDER_ISSUE
void aclq_send_acl_pkt_in_wait_queue(HCI_ACL_RX_DATA_PKT *acl_pkt);

extern UINT16 hs_recd_conn_handle;
extern HCI_ACL_RX_DATA_PKT *hs_wait_queue_acl_head;
extern HCI_ACL_RX_DATA_PKT *hs_wait_queue_acl_tail;
#endif /* HS_USB_SEND_EVENT_REORDER_ISSUE */

#ifdef HS_USB_SEND_EVENT_REORDER_ISSUE_NEW
typedef struct HCI_ACL_RX_DATA_PKT_QUEUE_
{
    HCI_ACL_RX_DATA_PKT *head;
    HCI_ACL_RX_DATA_PKT *tail;
} HCI_ACL_RX_DATA_PKT_QUEUE;

typedef struct LL_HCI_ACL_RX_DATA_PKT_QUEUE_
{
    LL_HCI_ACL_DATA_PKT *head;
    LL_HCI_ACL_DATA_PKT *tail;
} LL_HCI_ACL_RX_DATA_PKT_QUEUE;

typedef void (*ACL_RX_DATA_WAIT_QUEUE_SEND_FUNC)(void *);
typedef struct ACL_RX_DATA_WAIT_QUEUE_
{
    UINT16 conn_handle;
    union
    {
        HCI_ACL_RX_DATA_PKT_QUEUE data;
        LL_HCI_ACL_RX_DATA_PKT_QUEUE le_data;
    };

    void (*send)(void *);
} ACL_RX_DATA_WAIT_QUEUE;

void aclq_rx_wait_queue_init();
ACL_RX_DATA_WAIT_QUEUE *aclq_rx_wait_queue_find(UINT16 handle);
ACL_RX_DATA_WAIT_QUEUE *aclq_rx_wait_queue_get(UINT16 handle);
void aclq_rx_wait_queue_free(UINT16 handle);
void aclq_rx_wait_queue_enqueue(ACL_RX_DATA_WAIT_QUEUE *queue, HCI_ACL_RX_DATA_PKT *pkt);
void aclq_rx_wait_queue_enqueue_le(ACL_RX_DATA_WAIT_QUEUE *queue, LL_HCI_ACL_DATA_PKT *pkt);
HCI_ACL_RX_DATA_PKT *aclq_rx_wait_queue_dequeue(ACL_RX_DATA_WAIT_QUEUE *queue);
HCI_ACL_RX_DATA_PKT *aclq_rx_wait_queue_dequeue_all(ACL_RX_DATA_WAIT_QUEUE *queue);
#define aclq_rx_wait_queue_dequeue_all_le(queue) ((LL_HCI_ACL_DATA_PKT *) aclq_rx_wait_queue_dequeue_all(queue))

void aclq_send_acl_rx_pkt_in_wait_queue(HCI_ACL_RX_DATA_PKT *acl_pkt);
void aclq_send_le_acl_rx_pkt_in_wait_queue(LL_HCI_ACL_DATA_PKT *acl_pkt);
#endif /* HS_USB_SEND_EVENT_REORDER_ISSUE_NEW */

#endif /* _BT_FW_ACL_Q_H_ */

/** @} end: hci_external */
