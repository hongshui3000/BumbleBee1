/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/** 
 * \file
 *  LMP SCO interface. This file defines the public interface which can be used
 *  by the other layers (HCI, LC, LMP).
 * 
 * \author Santhosh kumar M
 * \date 2007-02-01
 */

/** \addtogroup lmp_external
 *  @{ */
#ifndef _LMP_SCO_H_
#define _LMP_SCO_H_

#include "bt_fw_types.h"
#include "bt_fw_config.h"
#include "lmp_defines.h"

#ifdef ENABLE_SCO
/** 
 * Sets Transaction Status (\a status) of SCO entity (\a sco_ce_ptr).
 * 
 * \param sco_ce_ptr SCO connection entity pointer for which the transaction
 *                   status has to be set.
 * \param status Transaction status to be set (#LMP_TRANSACTION_STATUS).
 * 
 * \return None.
 */
#define lmp_sco_set_trans_status(sco_ce_ptr, status)                        \
    (sco_ce_ptr)->tr_status = (status)

/** 
 * Gets the Transaction Status (#LMP_TRANSACTION_STATUS) associated with the
 * SCO connection entity pointed by \a sco_ce_ptr.
 * 
 * \param sco_ce_ptr SCO connection entity pointer.
 * 
 * \return Transaction status of the SCO connection entity (\a sco_ce_ptr).
 */
#define lmp_sco_get_trans_status(sco_ce_ptr)                                \
    (sco_ce_ptr)->tr_status

/** 
 * Allocates Temporary SCO Connection entity (which is used during SCO change
 * connection packet type negotiation).
 * 
 * \param ce_index ACL Connection entity index to which the allocated
 *                 Temporary SCO connection entity has to be associated.
 * 
 * \return None.
 */
#define lmp_allocate_temp_sco_conn_entity(ce_index)                         \
    (lmp_sco_connection_data[LMP_MAX_SCO_CONNECTIONS].sco_conn_status       \
     != SCO_FREE ? FALSE: (lmp_sco_connection_data[LMP_MAX_SCO_CONNECTIONS].\
         sco_conn_status = SCO_CONNECTED, lmp_sco_connection_data[          \
         LMP_MAX_SCO_CONNECTIONS].conn_entity_index = ce_index, TRUE))

/** 
 * Frees the Temporary SCO connection entiy.
 * 
 * \param None.
 *
 * \return None.
 */
#define lmp_free_temp_sco_conn_entity()                                 \
    lmp_sco_connection_data[LMP_MAX_SCO_CONNECTIONS].sco_conn_status    \
        = SCO_FREE

API_RESULT lmp_get_sco_ce_index_from_conn_handle(const UINT16 conn_handle,
                                                 UINT16* sco_ce_index);
API_RESULT lmp_get_sco_ce_index_from_ce_index(const UINT16 ce_index,
        const LMP_SCO_STATE status, UINT16* sco_ce_index);
API_RESULT lmp_get_sco_ce_index_from_sco_handle(const UCHAR sco_handle,
        const UINT16 ce_index, UINT16* sco_ce_index);
UCHAR lmp_initiate_new_sco_connection(UINT16 ce_index, UINT16 max_latency,
        UINT16 allowed_pkt_types, UCHAR hci_air_mode, UCHAR gen_conn_complete);
UCHAR lmp_handle_new_sco_conn_req(UINT16 sco_ce_index, UINT16 max_latency,
        UINT16 allowed_pkt_types, UCHAR hci_air_mode, UCHAR gen_conn_complete);
UCHAR lmp_generate_sco_params_for_change_pkt_type(UINT16 sco_ce_index,
        UINT16 allowed_pkt_types, UINT16 max_latency,
        UINT16 preferred_pkt_type);
UCHAR lmp_allocate_sco_conn_entity(UINT16* sco_ce_index, UINT16 ce_index);
void lmp_free_sco_conn_entity(const UINT16 sco_ce_index);
void lmp_sco_connect_update_ce_status(const UINT16 ce_index);
void lmp_sco_connect_restore_ce_status(const UINT16 ce_index);
//#ifdef SCO_DETACH_WORKAROUND_REMIND
UCHAR lmp_is_remove_sco_link_possible(UINT16 ce_index);
//#endif
void lmp_send_sco_link_request_pdu(UINT16 ce_index, UINT16 sco_ce_index,
        LMP_TRAN_ID tid);
void lmp_disconnect_sco_links(UINT16 ce_index, UCHAR reason);
API_RESULT lmp_handle_sco_disconnect(UINT16 conn_handle, UCHAR reason,
                                     UCHAR* sent_cmd_status);
void lmp_gen_approp_sco_conn_completion_event(UINT16 ce_index,
        UINT16 sco_ce_index, UCHAR status);
#endif /* ENABLE_SCO */
#endif /* _LMP_SCO_H_ */

/** @} end: lmp_external */
