/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file bz_auth_hci_2_1.h
 *  BT2.1 HCI Authentication related \a internal interface definition.
 *
 * \author Rajan PS, Santhosh kumar M
 * \date 2007-10-23
 */

#ifndef _BZ_AUTH_HCI_2_1_H_
#define _BZ_AUTH_HCI_2_1_H_

/* ========================= Include File Section ========================= */
#include "bt_fw_types.h"
#include "bz_auth_internal.h"
#include "bz_auth_hci.h"

/* ====================== Macro Declaration Section ======================= */


/* ==================== Data Types Declaration Section ==================== */


/* ================ Exported Variables Declaration Section ================ */


/* ============================= API Section ============================== */
/* 2.1 Event Generation API */
void bz_auth_generate_encryption_key_refresh_complete_event(UINT16 ce_index,
        UCHAR status);
void bz_auth_generate_io_cap_request_event(UINT16 ce_index);
void bz_auth_generate_io_cap_response_event(UINT16 ce_index,
        UCHAR* remote_io_cap);
void bz_auth_generate_user_confirmation_request_event(UINT16 ce_index,
        UINT32 numeric_check_value);
void bz_auth_generate_user_passkey_request_event(UINT16 ce_index);
void bz_auth_generate_remote_oob_data_request_event(UINT16 ce_index);
void bz_auth_generate_simple_pairing_complete_event(UINT16 ce_index,
        UCHAR status);
void bz_auth_generate_user_passkey_notification_event(UINT16 ce_index,
        UINT32 passkey);
void bz_auth_generate_keypress_notification_event(UINT16 ce_index,
        UCHAR notification_type);

/* 2.1 Command handlers API */
UCHAR bz_auth_handle_refresh_encryption(HCI_CMD_PKT* hci_cmd_ptr);
UCHAR bz_auth_handle_key_press_notification(HCI_CMD_PKT* hci_cmd_ptr);
UCHAR bz_auth_handle_io_cap_request_reply(HCI_CMD_PKT* hci_cmd_ptr);
UCHAR bz_auth_handle_io_cap_request_neg_reply(HCI_CMD_PKT* hci_cmd_ptr);
UCHAR bz_auth_handle_user_confirmation_request_reply(HCI_CMD_PKT* hci_cmd_ptr);
UCHAR bz_auth_handle_user_confirmation_request_neg_reply(
        HCI_CMD_PKT* hci_cmd_ptr);
UCHAR bz_auth_handle_user_passkey_request_reply(HCI_CMD_PKT* hci_cmd_ptr);
UCHAR bz_auth_handle_user_passkey_request_neg_reply(HCI_CMD_PKT* hci_cmd_ptr);
UCHAR bz_auth_handle_remote_oob_data_request_reply(HCI_CMD_PKT* hci_cmd_ptr);
UCHAR bz_auth_handle_remote_oob_data_request_neg_reply(
        HCI_CMD_PKT* hci_cmd_ptr);
UCHAR bz_auth_perform_io_cap_request_neg_reply_action(UINT16 ce_index,
        UINT16 reason);
UCHAR bz_auth_perform_user_confirmation_request_neg_reply_action(
        UINT16 ce_index);
UCHAR bz_auth_perform_user_passkey_request_neg_reply_action(UINT16 ce_index);
UCHAR bz_auth_perform_remote_oob_data_request_neg_reply_action(UINT16 ce_index);

#endif /* _BZ_AUTH_HCI_2_1_H_ */

