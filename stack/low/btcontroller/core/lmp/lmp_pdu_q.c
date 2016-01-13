/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/
/********************************* Logger *************************/
enum { __FILE_NUM__= 55 };
/********************************* Logger *************************/
/**
 * \file
 *  LMP-PDU Queue implementation.
 *  This is stripped down version of ACL_Q (see bt_fw_acl_q.[ch])
 *
 * \author Muthu Subramanian K
 */

/* Includes */
#include "lmp_internal.h"
#include "bt_fw_acl_q.h"
#include "lmp_pdu_q.h"

#ifdef _ROM_CODE_PATCHED_
PF_ROM_CODE_PATCH_FUNC rcp_pduq_get_next_active_pdu;
#endif

/* Enable this define for debugging */
LMP_PDU_PKT *pdu_q_head = NULL;
LMP_PDU_PKT *pdu_q_tail = NULL;

#ifdef _NEW_HCI_DMA_DESIGN_FOR_ACL_SNIFF_SCHEDULE_
LMP_PDU_PKT *pdu_q_head_in_sniff = NULL;
LMP_PDU_PKT *pdu_q_tail_in_sniff = NULL;
#endif

/**
 * Initialize the LMP PDU Queue.
 *
 * \param None.
 * \return None.
 */
void pduq_init(void)
{
    pdu_q_head = NULL;
    pdu_q_tail = NULL;

#ifdef _NEW_HCI_DMA_DESIGN_FOR_ACL_SNIFF_SCHEDULE_
    pdu_q_head_in_sniff = NULL;
    pdu_q_tail_in_sniff = NULL;
#endif
}

/**
 * Queue the PDU packet for transmission.
 * Do not free the pointer until the transmission is complete.
 *
 * \param pkt the PDU data packet.
 * \param piconet Piconet id.
 *
 * \return None.
 */
void pduq_queue_pdu(LMP_PDU_PKT *pkt, UCHAR piconet, UINT16 ce_index)
{
    LMP_PDU_PKT* pkt1;

    if (pkt == NULL)
    {
        return;
    }

    /* Initialize Packet data */
    pkt->in_transmission = FALSE;
    pkt->next = NULL;
    pkt->piconet = piconet;
    pkt->ce_index = ce_index;
    pkt->nbc_count = 0x0;

    /* Ensure that the nbc_count is same as existing PDU for this link if any */
    pkt1 = pdu_q_head;
    while (pkt1 != NULL)
    {
        if ((pkt1->am_addr == pkt->am_addr) &&
            (pkt1->piconet == pkt->piconet))
        {
            pkt->nbc_count = pkt1->nbc_count;
            break;
        }
        pkt1 = pkt1->next;
    }

    /* Add to Queue */
    if (pdu_q_head == NULL)
    {
        pdu_q_head = pkt;
    }
    else
    {
        pdu_q_tail->next = pkt;
    }
    pdu_q_tail = pkt;
}

/**
 * Returns the next PDU packet for an active
 * am_addr. Returns NULL if there is none for an active am_addr or
 * if there is no active am_addr.
 *
 * \param am_addr address to return am_addr.
 * \param piconet Piconet id.
 *
 * \return Pointer to the pdu packet or NULL.
 */
LMP_PDU_PKT *pduq_get_next_pdu_active(UCHAR *am_addr, UCHAR piconet)
{
    LMP_PDU_PKT *pkt;

    pkt = pdu_q_head;

    while (pkt != NULL)
    {
		if ((pkt->in_transmission == FALSE) &&
			(pkt->piconet == piconet) &&
			(aclq_is_am_addr_failed(pkt->am_addr, pkt->piconet) == FALSE))
		{
            *am_addr = pkt->am_addr;
            pkt->in_transmission = TRUE;
            break;
        }
        pkt = pkt->next;
    }

    return pkt;
}

/**
 * Get the next PDU packet for transmission.
 * Tries for active am_addr first and then for 'any' am_addr.
 * Returns NULL if queue is empty (or if available ones are in
 * transmission).
 *
 * \param am_addr address to return am_addr.
 * \param piconet Piconet id.
 *
 * \return Pointer to the lmp packet or NULL.
 */
LMP_PDU_PKT *pduq_get_next_pdu(UCHAR *am_addr, UCHAR piconet)
{
    LMP_PDU_PKT *pkt;
    LMP_PDU_PKT *pkt1;

#ifdef _ROM_CODE_PATCHED_
    if (rcp_pduq_get_next_active_pdu != NULL)
    {
        if (rcp_pduq_get_next_active_pdu((void*)am_addr, piconet, &pkt))
        {
            return pkt;
        }
    }
#endif

    /* Try to get a pdu for active am_addr */
    pkt = pduq_get_next_pdu_active(am_addr, piconet);
    if (pkt != NULL)
    {
        if (pkt->nbc_count != 0x0)
        {
            pkt->nbc_count = 0x0;
            pkt1 = pkt->next;
            while (pkt1 != NULL)
            {
                if ((pkt1->am_addr == pkt->am_addr) &&
                    (pkt1->piconet == pkt->piconet))
                {
                    pkt1->nbc_count = pkt->nbc_count;
                }
                pkt1 = pkt1->next;
            }
        }
        return pkt;
    }

    /*
       If there are none for active am_addr then
       get the 1st in_transmission = FALSE packet
     */
    /* Skip bad connections in sniff mode. */
    pkt = pdu_q_head;
    while (pkt != NULL)
    {
        LMP_CONNECTION_ENTITY *ce_ptr;
        ce_ptr = &lmp_connection_entity[pkt->ce_index];

        if ((pkt->in_transmission == FALSE) &&
           (pkt->piconet == piconet) &&
           (pkt->nbc_count < LMP_MAX_NBC_TIMEOUT)
#ifdef COMPILE_SNIFF_MODE
           && (ce_ptr->in_sniff_mode == FALSE)
#endif
           )
        {
            *am_addr = pkt->am_addr;
            pkt->in_transmission = TRUE;
            break;
        }
        pkt = pkt->next;
    }

    return pkt;
}

/**
 * Used to dequeue and ack a queued pdu_pkt.
 * Can be used as external interface also.
 *
 * \param fpkt Pointer to the pdu_pkt to dequeue.
 *
 * \return None.
 */
void pduq_dequeue_pdu(LMP_PDU_PKT *fpkt)
{
    LMP_PDU_PKT *ppkt; /* previous packet */
    LMP_PDU_PKT *pkt; /* current packet */

    if (fpkt == NULL)
    {
        return;
    }

    ppkt = NULL;
    pkt = pdu_q_head;
    while (pkt != NULL)
    {
        if (pkt == fpkt)
        {
            if (ppkt == NULL)
            {
                pdu_q_head = pdu_q_head->next;
            }
            else
            {
                ppkt->next = pkt->next;
            }
            pkt->next = NULL;

            if (pdu_q_head == NULL)
            {
                pdu_q_tail = NULL;
            }
            else if (fpkt == pdu_q_tail)
            {
                pdu_q_tail = ppkt;
            }
            break;
        }
        ppkt = pkt;
        pkt = pkt->next;
    }
}

/**
 * Gets the number of PDUs currently in pdu queue for a
 * particular piconet.
 *
 * \param piconet Piconet id.
 *
 * \return Number of enqueued PDUs for a piconet.
 */
UINT16 pduq_get_no_of_pdus_piconet(UCHAR piconet)
{
    UINT16 num_of_pdus = 0;
    LMP_PDU_PKT *pkt;

    pkt = pdu_q_head;
    while (pkt != (LMP_PDU_PKT *)NULL)
    {
        if (pkt->piconet == piconet)
        {
            num_of_pdus++;
        }
        pkt = pkt->next;
    }
    return num_of_pdus;
}

/**
 * Gets the number of PDUs currently in pdu queue for a
 * particular am addr in a particular piconet.
 *
 * \param am_addr AM Address.
 * \param piconet Piconet id.
 * \param near_end valid counter from near end
 *
 * \return Number of enqueued PDUs for an AM addr in a particular piconet.
 */
UINT16 pduq_get_no_of_pdus_am_addr(UCHAR am_addr, UCHAR piconet,
                                              UINT8 near_end)
{
    LMP_PDU_PKT *pkt;
    UINT16 num_of_pdus = 0;

    pkt = pdu_q_head;

    while (pkt != (LMP_PDU_PKT *)NULL)
    {
        if ((pkt->am_addr == am_addr) && (pkt->piconet == piconet))
        {
            num_of_pdus++;
        }
        else if (near_end)
        {
            break;
        }

        pkt = pkt->next;
    }
    return num_of_pdus;
}


/**
 * Get the first available pdu_pkt for the am_address.
 * Used for flushing & RE-Queuing.
 *
 * \param am_addr AM Address.
 * \param piconet Piconet id.
 *
 * \return Pointer to the pdu_packet.
 */
LMP_PDU_PKT *pduq_get_pdu_am_addr(UCHAR am_addr, UCHAR piconet)
{
    LMP_PDU_PKT *pkt;
    pkt = pdu_q_head;
    while (pkt != (LMP_PDU_PKT *)NULL)
    {
        if ((pkt->am_addr == am_addr) && (pkt->piconet == piconet))
        {
            return pkt;
        }
        pkt = pkt->next;
    }
    return NULL;
}

/**
 * Re-Queue all the pdu-pkts on a different
 * am_addr & piconet. Useful for role-switch.
 *
 * \param old_am_addr Old AM Address.
 * \param old_piconet Old Piconet id.
 * \param new_am_addr New AM Address.
 * \param new_piconet New Piconet id.
 *
 * \return None.
 */
void pduq_requeue_all_pdus()
{
	LMP_PDU_PKT *pkt;

	UCHAR old_am_addr, new_am_addr;
	UCHAR old_piconet, new_piconet;

	old_am_addr = lmp_role_switch_data.old_am_addr;
	new_am_addr = lmp_role_switch_data.new_am_addr;

	old_piconet = lmp_role_switch_data.old_piconet_id;
	new_piconet = lmp_role_switch_data.new_piconet_id;

    if ((old_am_addr == new_am_addr) && (old_piconet == new_piconet))
    {
        return;
    }

    while (1)
    {
        pkt = pduq_get_pdu_am_addr(old_am_addr, old_piconet);
        if (pkt == (LMP_PDU_PKT *)NULL)
        {
            break;
        }
        pkt->am_addr = new_am_addr;
        pkt->piconet = new_piconet;
    }

    return;
}

/**
 * Resets pdu_pkt which is in transmission.
 * This function will mark the am_addr as failed. This pdu
 * will be re-rescheduled later.
 *
 * \param pkt Pointer to the pdu_pkt.
 *
 * \return None.
 */
#if 0 /* no one use function pduq_reset_pdu, hence we comment it */
void pduq_reset_pdu(LMP_PDU_PKT *pkt)
{
    LMP_PDU_PKT* pkt1;

    aclq_mark_am_addr_as_failed(pkt->am_addr, pkt->piconet);

    pkt->in_transmission = FALSE;

#ifdef COMPILE_SNIFF_MODE
    if ((lmp_connection_entity[pkt->ce_index].in_sniff_mode == TRUE) &&
        (lmp_connection_entity[pkt->ce_index].sniff_interval >
         LMP_MIN_SNIFF_INTERVAL_TO_SKIP_DATA_TX_OPT))
    {
        pkt->nbc_count = LMP_MAX_NBC_TIMEOUT;
        pkt1 = pkt->next;
        while (pkt1 != NULL)
        {
            if ((pkt1->am_addr == pkt->am_addr) &&
                (pkt1->piconet == pkt->piconet))
            {
                pkt1->nbc_count = pkt->nbc_count;
            }
            pkt1 = pkt1->next;
        }
    }
#endif
    return;
}
#endif
/**
 * Resets pdu_pkt which is in transmission upon NBC Time Out.
 * This function will mark the am_addr as failed. This pdu
 * will be re-rescheduled later.
 *
 * \param pkt Pointer to the pdu_pkt.
 *
 * \return None.
 */
void pduq_reset_nbc_pdu(LMP_PDU_PKT *pkt)
{
    LMP_PDU_PKT *pkt1;
    LMP_CONNECTION_ENTITY *ce_ptr;

	pkt->in_transmission = FALSE;
    ce_ptr = &lmp_connection_entity[pkt->ce_index];

#ifdef COMPILE_SNIFF_MODE
    if ((ce_ptr->in_sniff_mode == TRUE) &&
            ((ce_ptr->next_instant_in_nat_clk -
                BB_read_native_clock()) <=
                    LC_MIN_SLOTS_TO_SKIP_MARK_AS_BAD_FOR_SNIFF))
    {
        /* Do not mark this link as invalid, sniff instant is approaching */
    }
    else if ((ce_ptr->in_sniff_mode == TRUE) &&
        (ce_ptr->sniff_interval > LMP_MIN_SNIFF_INTERVAL_TO_SKIP_DATA_TX_OPT))
    {
        pkt->nbc_count = LMP_MAX_NBC_TIMEOUT;
    }
    else
#endif
    {
        aclq_mark_am_addr_as_failed(pkt->am_addr, pkt->piconet);
        pkt->nbc_count++;
    }

    pkt1 = pkt->next;
    while (pkt1 != NULL)
    {
        if ((pkt1->am_addr == pkt->am_addr) &&
            (pkt1->piconet == pkt->piconet))
        {
            pkt1->nbc_count = pkt->nbc_count;
        }
        pkt1 = pkt1->next;
    }
	return;
}

/**
 * Resets pdu_pkt which is in transmission.
 * This function will NOT mark the am_addr as failed. This pdu
 * will be re-rescheduled later.
 *
 * \param pkt Pointer to the pdu_pkt.
 *
 * \return None.
 */
void pduq_reset_pdu_wo_failed(LMP_PDU_PKT *pkt)
{
    pkt->in_transmission = FALSE;
}

/**
 * Dequeue and free all packets for an am_address.
 *
 * \param am_addr AM Address.
 * \param piconet Piconet id.
 * \param all_flag if TRUE then all PDUs including in_transmission
 *                 ones will be freed.
 *
 * \return TRUE if a pdu-packet was dequeued.
 */
UCHAR pduq_clear_all_pdus_am_addr(UCHAR am_addr, UCHAR piconet, UCHAR all_flag)
{
    LMP_PDU_PKT *pkt, *npkt;
    UCHAR ret_status = FALSE;
    pkt = pdu_q_head;

    while (pkt != (LMP_PDU_PKT *)NULL)
    {
        npkt = pkt->next;

        if ((pkt->am_addr == am_addr) &&
            (pkt->piconet == piconet))
        {
            if ((pkt->in_transmission == FALSE) || (all_flag == TRUE))
            {
                pduq_dequeue_pdu(pkt);
                if (OS_FREE_BUFFER(lmp_pdu_buffer_pool_handle, pkt) != BT_ERROR_OK)
                {
					RT_BT_LOG(GRAY, LMP_PDU_Q_487, 0, 0);
                }
                ret_status = TRUE;
            }
        }

        pkt = npkt;
    }

    return ret_status;
}

/* ---------------------------- End of lmp_pdu_q.c ---------------------- */

