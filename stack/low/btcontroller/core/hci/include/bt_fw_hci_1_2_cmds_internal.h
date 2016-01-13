/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  BT1.2 HCI Layer internal interface.
 */

/** \addtogroup hci_internal
 *  @{ */
#ifndef _HCI_1_2_CMDS_INTERNAL_H_
#define _HCI_1_2_CMDS_INTERNAL_H_

/* Function Prototype Declarations */

UCHAR hci_handle_write_inquiry_scan_type_command(HCI_CMD_PKT *hci_cmd_ptr);

UCHAR hci_handle_write_inquiry_mode_command(HCI_CMD_PKT *hci_cmd_ptr);

UCHAR hci_handle_write_page_scan_type_command(HCI_CMD_PKT *hci_cmd_ptr);

UCHAR hci_handle_setup_sync_conn_command(HCI_CMD_PKT *hci_cmd_ptr,
        UINT16* ce_index, HCI_LINK_TYPE* link_type,
        UCHAR gen_conn_complete, SYNC_LINK_STATUS* sync_link);

UCHAR hci_handle_accept_sync_conn_req_command(HCI_CMD_PKT *hci_cmd_ptr,
        UINT16* ce_index, HCI_LINK_TYPE* link_type, UCHAR gen_conn_complete);

UCHAR hci_handle_reject_sync_conn_req_command(HCI_CMD_PKT *hci_cmd_ptr,
        UINT16* ce_index, HCI_LINK_TYPE* link_type);

UCHAR hci_handle_set_afh_host_channel_classification_command(
                                              HCI_CMD_PKT *hci_cmd_ptr);

UCHAR hci_handle_write_afh_channel_assessment_command(
                                              HCI_CMD_PKT *hci_cmd_ptr);

UCHAR   hci_handle_read_afh_channel_map_command(HCI_CMD_PKT *hci_cmd_ptr, 
                                                UINT16 conn_handle);

API_RESULT hci_generate_synchronous_conn_complete_event(UINT16 ce_index,
        UINT16 esco_ce_index, HCI_LINK_TYPE link_type, UCHAR status);

API_RESULT hci_generate_synchronous_conn_changed_event(UCHAR status,
        UINT16 esco_ce_index);

#ifdef COMPILE_FEATURE_REQ_EXT
void hci_generate_remote_ext_features_complete_event(UCHAR status,
        UINT16 connection_handle, UCHAR page, UCHAR max_supported_page,
        UCHAR* remote_features);
#endif

UCHAR hci_handle_read_remote_ext_features_command(HCI_CMD_PKT *hci_cmd_ptr,
                                              UCHAR *sent_cmd_status);

#endif /* _HCI_1_2_CMDS_INTERNAL_H_ */

/** @} end: hci_internal */
