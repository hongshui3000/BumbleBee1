/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file bz_auth_lmp.h
 *  LMP Authentication related \a internal interface definition.
 *
 * \author Rajan PS, Santhosh kumar M
 * \date 2007-10-23
 */

#ifndef _BZ_AUTH_LMP_H_
#define _BZ_AUTH_LMP_H_

/* ========================= Include File Section ========================= */
#include "bt_fw_types.h"
#include "bz_auth_internal.h"
#include "bz_auth_lmp_2_1.h"

/* ====================== Macro Declaration Section ======================= */
#ifdef _CCH_IOT_RALINK_
#define IS_LMP_PDU_TID_DIFFERENT \
        (!g_efuse_lps_setting_3.iot_ralink_tid_no_check &&  \
         (LMP_GET_TRANSACTION_ID(lmp_pdu_ptr) != (active_tid)))

#define IS_LMP_PDU_TID_THE_SAME  \
        (!g_efuse_lps_setting_3.iot_ralink_tid_no_check &&  \
         (LMP_GET_TRANSACTION_ID(lmp_pdu_ptr) == (active_tid)))

#define IS_AUTH_SELF_INIT       \
        (!g_efuse_lps_setting_3.iot_ralink_tid_no_check &&  \
         bz_auth_is_self_initiated(ce_ptr, lmp_pdu_ptr))

#define IS_AUTH_REMOTE_INIT     \
        (!g_efuse_lps_setting_3.iot_ralink_tid_no_check &&  \
         bz_auth_is_remote_initiated(ce_ptr, lmp_pdu_ptr))
#else
#define IS_LMP_PDU_TID_DIFFERENT \
    (LMP_GET_TRANSACTION_ID(lmp_pdu_ptr) != (active_tid))
#define IS_LMP_PDU_TID_THE_SAME  \
    (LMP_GET_TRANSACTION_ID(lmp_pdu_ptr) == (active_tid))
#define IS_AUTH_SELF_INIT       \
    bz_auth_is_self_initiated(ce_ptr, lmp_pdu_ptr)
#define IS_AUTH_REMOTE_INIT       \
    bz_auth_is_remote_initiated(ce_ptr, lmp_pdu_ptr)

#endif

#define bz_auth_break_if_different_transaction(active_tid)      \
{                                                               \
    if (IS_LMP_PDU_TID_DIFFERENT)                               \
    {                                                           \
        lmp_error_code = PDU_NOT_ALLOWED_ERROR;                 \
        break;                                                  \
    }                                                           \
}

#define bz_auth_break_if_same_transaction(active_tid)           \
{                                                               \
    if (IS_LMP_PDU_TID_THE_SAME)                                \
    {                                                           \
        lmp_error_code = PDU_NOT_ALLOWED_ERROR;                 \
        break;                                                  \
    }                                                           \
}

#define bz_auth_break_if_self_initiated_transaction         \
{                                                           \
    if (IS_AUTH_SELF_INIT)     \
    {                                                       \
        lmp_error_code = PDU_NOT_ALLOWED_ERROR;             \
        break;                                              \
    }                                                       \
}                                                           

#define bz_auth_break_if_remote_initiated_transaction       \
{                                                           \
    if (IS_AUTH_REMOTE_INIT)   \
    {                                                       \
        lmp_error_code = PDU_NOT_ALLOWED_ERROR;             \
        break;                                              \
    }                                                       \
}                                                           

#define bz_auth_return_if_different_transaction(active_tid)     \
{                                                               \
    if (IS_LMP_PDU_TID_DIFFERENT)                               \
    {                                                           \
        BZ_ASSERT(0, "Unexpected TID");                         \
        return;                                                 \
    }                                                           \
}

#define bz_auth_return_if_self_initiated_transaction            \
{                                                               \
    if (IS_AUTH_SELF_INIT)                                      \
    {                                                           \
        BZ_ASSERT(0, "Unexpected TID");                         \
        return;                                                 \
    }                                                           \
}                                                               

#define bz_auth_return_if_remote_initiated_transaction          \
{                                                               \
    if (IS_AUTH_REMOTE_INIT)                                    \
    {                                                           \
        BZ_ASSERT(0, "Unexpected TID");                         \
        return;                                                 \
    }                                                           \
}                                                               

#define bz_auth_return_if_error_or_self_initiated_transaction(opcode)   \
{                                                                       \
    if (IS_AUTH_SELF_INIT                  \
            && lmp_error_code == HCI_COMMAND_SUCCEEDED)                 \
    {                                                                   \
        lmp_error_code = PDU_NOT_ALLOWED_ERROR;                         \
    }                                                                   \
    if (lmp_error_code != HCI_COMMAND_SUCCEEDED)                        \
    {                                                                   \
        bz_auth_send_not_accepted_pdu(ce_index, (opcode),               \
                tid, lmp_error_code);                                   \
        return;                                                         \
    }                                                                   \
}

#define bz_auth_return_if_error_or_self_initiated_transaction_ext(op, ext_op) \
{                                                                       \
    if (IS_AUTH_SELF_INIT                  \
            && lmp_error_code == HCI_COMMAND_SUCCEEDED)                 \
    {                                                                   \
        lmp_error_code = PDU_NOT_ALLOWED_ERROR;                         \
    }                                                                   \
    if (lmp_error_code != HCI_COMMAND_SUCCEEDED)                        \
    {                                                                   \
        bz_auth_send_not_accepted_ext_pdu(ce_index, (op), (ext_op),     \
                tid, lmp_error_code);                                   \
        return;                                                         \
    }                                                                   \
}

/**
 * Returns whether the authentication module is dealing with during ACL
 * connection authentication procedure or not.
 *
 * \param auth Authentication parameters associated with the ACL link.
 *
 * \return TRUE, if the ACL connection is in progress. FALSE, otherwise.
 */
#define bz_auth_is_during_conn(auth)                                \
    ((auth)->super_state == UNAUTHENTICATED_DURING_CONN             \
        || (auth)->super_state == AUTHENTICATING_DURING_CONN        \
        || (auth)->super_state == AUTHENTICATED_DURING_CONN         \
        || (auth)->super_state == ENCRYPTING_DURING_CONN            \
        || (auth)->super_state == ENCRYPTED_DURING_CONN)


/* ==================== Data Types Declaration Section ==================== */


/* ================ Exported Variables Declaration Section ================ */


/* ============================= API Section ============================== */
void bz_auth_handle_stop_encryption_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index, OUT BOOLEAN* can_free_pdu);

#endif /* _BZ_AUTH_LMP_H_ */

