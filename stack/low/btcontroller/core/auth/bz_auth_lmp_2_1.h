/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file bz_auth_lmp_2_1.h
 *  BT2.1 LMP Authentication related \a internal interface definition.
 *
 * \author Rajan PS, Santhosh kumar M
 * \date 2007-10-23
 */

#ifndef _BZ_AUTH_LMP_2_1_H_
#define _BZ_AUTH_LMP_2_1_H_

/* ========================= Include File Section ========================= */
#include "bt_fw_types.h"
#include "bz_auth_internal.h"
#include "bz_auth_lmp.h"

/* ====================== Macro Declaration Section ======================= */


/* ==================== Data Types Declaration Section ==================== */


/* ================ Exported Variables Declaration Section ================ */


/* ============================= API Section ============================== */
void bz_auth_handle_io_capability_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index, OUT BOOLEAN* can_free_pdu);
void bz_auth_handle_io_capability_res_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index, OUT BOOLEAN* can_free_pdu);
void bz_auth_handle_numeric_comparison_failed_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index, OUT BOOLEAN* can_free_pdu);
void bz_auth_handle_oob_failed_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index,
        OUT BOOLEAN* can_free_pdu);
void bz_auth_handle_keypress_notification_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index, OUT BOOLEAN* can_free_pdu);
void bz_auth_handle_passkey_failed_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index, OUT BOOLEAN* can_free_pdu);
void bz_auth_handle_pause_encryption_req_not_accepted_pdu(
        LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index, OUT BOOLEAN* can_free_pdu);
void bz_auth_handle_io_capability_req_not_accepted_pdu(
        LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index, OUT BOOLEAN* can_free_pdu);
void bz_auth_handle_pause_encryption_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index, OUT BOOLEAN* can_free_pdu);
void bz_auth_handle_resume_encryption_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index, OUT BOOLEAN* can_free_pdu);
void bz_auth_handle_simple_pairing_confirm_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index, OUT BOOLEAN* can_free_pdu);
void bz_auth_handle_simple_pairing_number_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index, OUT BOOLEAN* can_free_pdu);
void bz_auth_handle_dhkey_check_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index, OUT BOOLEAN* can_free_pdu);
void bz_auth_handle_simple_pairing_number_accepted_pdu(
        LMP_PDU_PKT* lmp_pdu_ptr, UINT16 ce_index, OUT BOOLEAN* can_free_pdu);
void bz_auth_handle_dhkey_check_accepted_pdu(LMP_PDU_PKT* lmp_pdu_ptr,
        UINT16 ce_index, OUT BOOLEAN* can_free_pdu);
void bz_auth_handle_simple_pairing_confirm_not_accepted_pdu(
        LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index, OUT BOOLEAN* can_free_pdu);
void bz_auth_handle_simple_pairing_number_not_accepted_pdu(
        LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index, OUT BOOLEAN* can_free_pdu);
void bz_auth_handle_dhkey_check_not_accepted_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index, OUT BOOLEAN* can_free_pdu);
#endif /* _BZ_AUTH_LMP_2_1_H_ */

