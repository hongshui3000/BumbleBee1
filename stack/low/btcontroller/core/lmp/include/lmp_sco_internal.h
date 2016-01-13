/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/** 
 * \file
 *  LMP Internal SCO interface. This file defines the interface which is
 *  exposed only to the LMP Layer and no other layers should use this
 *  interface.
 * 
 * \author Santhosh kumar M
 * \date 2007-02-01
 */

/** \addtogroup lmp_internal
 *   @{ */
#ifndef _LMP_SCO_INTERNAL_H_
#define _LMP_SCO_INTERNAL_H_

#include "bt_fw_types.h"
#include "lmp_defines.h"

#ifdef ENABLE_SCO
void lmp_init_sco_connection_table(void);
void lmp_init_sco_connection_entity(UCHAR index);
void lmp_sco_connection_data_cleanup_after_detach(int sco_ce_index);
UCHAR lmp_handle_remove_sco_link_req_ack_recd(UINT16 ce_index);
UCHAR lmp_handle_remove_sco_link_request_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index);
UCHAR lmp_handle_sco_link_req_not_accepted(UCHAR reason, UINT16 ce_index);
UCHAR lmp_handle_sco_link_req_accepted(UINT16 ce_index);
UCHAR lmp_handle_sco_link_req_accepted_sent(UINT16 ce_index);
UCHAR lmp_handle_sco_link_request_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index);
#endif /* ENABLE_SCO */
#endif /* _LMP_SCO_INTERNAL_H_ */

/** @} end: lmp_internal */
