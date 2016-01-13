/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file bz_auth_hci.h
 *  HCI Authentication related \a internal interface definition.
 *
 * \author Rajan PS, Santhosh kumar M
 * \date 2007-08-21
 */

#ifndef _BZ_AUTH_HCI_H_
#define _BZ_AUTH_HCI_H_

/* ========================= Include File Section ========================= */
#include "bt_fw_types.h"
#include "bz_auth_internal.h"
#include "bz_auth_hci_2_1.h"

/* ====================== Macro Declaration Section ======================= */


/* ==================== Data Types Declaration Section ==================== */


/* ================ Exported Variables Declaration Section ================ */
extern BOOLEAN bz_auth_is_remote_oob_data_expected(
        BZ_AUTH_LINK_PARAMS* auth);


/* ============================= API Section ============================== */
BOOLEAN bz_auth_validate_conn_handle(HCI_CMD_PKT* hci_cmd_ptr,
        OUT UINT16 *pce_index);
BOOLEAN bz_auth_validate_bd_addr(HCI_CMD_PKT* hci_cmd_ptr,
        OUT UINT16 *pce_index);
UINT16 perform_sanity_checks_for_reply_commands(HCI_CMD_PKT* hci_cmd_ptr,
        BOOLEAN (*is_the_cmd_expected)(BZ_AUTH_LINK_PARAMS*),
        BOOLEAN (*is_valid_params)(HCI_CMD_PKT*),
        OUT UINT16* pce_index, OUT LMP_CONNECTION_ENTITY** pce_ptr);
void bz_auth_enter_link_key_requested_state(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth, UINT8 sub_state);
void bz_auth_generate_pin_code_request_event(UINT16 ce_index);
void bz_auth_generate_authentication_complete_event(UINT16 ce_index,
        UCHAR status);
void bz_auth_generate_link_key_notification_event(UINT16 ce_index);
void bz_auth_generate_change_connection_link_key_event(UINT16 ce_index,
        UCHAR status);
void bz_auth_generate_encryption_change_event(UINT16 ce_index,
        UCHAR status, UINT8 enc_mode);
void bz_auth_generate_command_complete_event(HCI_CMD_PKT* hci_cmd_ptr,
        UCHAR status, UINT16 ce_index);
#endif /* _BZ_AUTH_HCI_H_ */

