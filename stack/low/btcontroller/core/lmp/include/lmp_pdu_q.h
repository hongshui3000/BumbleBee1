/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  LMP PDU Queue interface.
 *
 * \author Muthu Subramanian K
 */

/** \addtogroup lmp_external
 *  @{ */
#ifndef _LMP_PDU_Q_H_
#define _LMP_PDU_Q_H_

void pduq_init(void);
void pduq_queue_pdu(LMP_PDU_PKT *pkt, UCHAR piconet, UINT16 ce_index);
LMP_PDU_PKT *pduq_get_next_pdu_active(UCHAR *am_addr, UCHAR piconet);
LMP_PDU_PKT *pduq_get_next_pdu(UCHAR *am_addr, UCHAR piconet);
#if 0 /* no one use function pduq_reset_pdu, hence we comment it */
void pduq_reset_pdu(LMP_PDU_PKT *pkt);
#endif
void pduq_reset_nbc_pdu(LMP_PDU_PKT *pkt);
void pduq_reset_pdu_wo_failed(LMP_PDU_PKT *pkt);
void pduq_dequeue_pdu(LMP_PDU_PKT *fpkt);
LMP_PDU_PKT *pduq_get_pdu_am_addr(UCHAR am_addr, UCHAR piconet);
UCHAR pduq_clear_all_pdus_am_addr(UCHAR am_addr, UCHAR piconet, UCHAR all_flag);
void pduq_requeue_all_pdus(void);
UINT16 pduq_get_no_of_pdus_piconet(UCHAR piconet);
UINT16 pduq_get_no_of_pdus_am_addr(UCHAR am_addr, UCHAR piconet,
                                              UINT8 near_end);
#endif

/** @} end: lmp_external */
