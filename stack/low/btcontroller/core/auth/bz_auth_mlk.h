/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file bz_auth_mlk.h
 *  Master Link Key related internal interface definition.
 *
 * \author Rajan PS, Santhosh kumar M
 * \date 2007-10-23
 */

#ifndef _BZ_AUTH_MLK_H_
#define _BZ_AUTH_MLK_H_

#ifdef COMPILE_BROADCAST_ENCRYPTION
/* ========================= Include File Section ========================= */
#include "bt_fw_types.h"
#include "bz_auth_internal.h"

/**
 * \addtogroup bz_auth_internal Authentication Internal Interface
 * @{ */


/* ====================== Macro Declaration Section ======================= */


/* ==================== Data Types Declaration Section ==================== */


/* ================ Exported Variables Declaration Section ================ */


/* ============================= API Section ============================== */
UCHAR bz_auth_handle_master_link_key(HCI_CMD_PKT* hci_cmd_ptr);
void bz_auth_handle_master_link_key_enable_completion(UINT16 ce_index,
        UCHAR status);
void bz_auth_handle_master_link_key_reauth_completion(UINT16 ce_index,
        UCHAR status);
void bz_auth_handle_master_link_key_disable_completion(UINT16 ce_index,
        UCHAR status);
void bz_auth_handle_master_link_key_enable_enc_completion(UINT16 ce_index,
        UCHAR status);
void bz_auth_handle_master_link_key_disable_enc_completion(UINT16 ce_index,
        UCHAR status);
void bz_auth_handle_mlk_link_disconnected(UINT16 ce_index,
        BZ_AUTH_MLK_STATE link_mlk_state);
void bz_auth_initiate_mlk_procedure_if_pending(UINT16 ce_index, UCHAR status);
void bz_auth_handle_use_semi_permanent_key_accepted_pdu(
        LMP_PDU_PKT* lmp_pdu_ptr, UINT16 ce_index, OUT BOOLEAN* can_free_pdu);
void bz_auth_handle_temp_pdu_not_accepted_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index, OUT BOOLEAN* can_free_pdu);
void bz_auth_handle_use_semi_permanent_key_not_accepted_pdu(
        LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index, OUT BOOLEAN* can_free_pdu);
void bz_auth_handle_temp_rand_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index,
        OUT BOOLEAN* can_free_pdu);
void bz_auth_handle_temp_key_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index,
        OUT BOOLEAN* can_free_pdu);
void bz_auth_handle_use_semi_permanent_key_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index, OUT BOOLEAN* can_free_pdu);
#endif /* COMPILE_BROADCAST_ENCRYPTION */

/** @} end: bz_auth_internal */

#endif /* !_BZ_AUTH_MLK_H_ */

