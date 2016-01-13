/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

#ifndef __HCI_H__
#define __HCI_H__

#include "bt_fw_types.h"

#include "bt_fw_os.h"
#include "bt_fw_hci_defines.h"
#include "bt_fw_hci_1_2_cmds.h"

/*Required for host number of completed packtes command */
extern HCI_CMD_PKT hci_special_cmd_buffer;

/*Transport driver will use this handle for allocation of the command buffer */
extern POOL_ID hci_cmd_buffer_pool_handle;

/*Transport driver will use this handle for de allocation of the Event buffer */
extern POOL_ID hci_event_buffer_pool_handle;

/* Comamnd and Event Task handles */
extern TASK_ID hci_command_task_handle;
extern TASK_ID hci_event_task_handle;
extern UCHAR hci_baseband_cmd_pending;


/*APIs prototypes*/

API_RESULT hci_init(void);

API_RESULT hci_shutdown(void);


//void firmware_soft_reset(void);

UCHAR HCI_Command_Task(OS_SIGNAL *signal_ptr);

void HCI_Event_Task(OS_SIGNAL *signal_ptr);

HCI_EVENT_PKT *hci_generate_command_status_event(UINT16 hci_cmd_opcode,
                                                    UCHAR cmd_status);

HCI_EVENT_PKT *hci_generate_command_complete_event(UINT16 hci_cmd_opcode,
                                                       UCHAR cmd_status,
                                                           UINT16 ce_index);

HCI_EVENT_PKT *hci_generate_hardware_error_event(UCHAR hardware_code);

HCI_EVENT_PKT *hci_generate_number_of_completed_packets_event(void);

HCI_EVENT_PKT *hci_generate_inquiry_complete_event(UCHAR inquiry_status);

HCI_EVENT_PKT *hci_generate_inquiry_result_event(UCHAR inquiry_data_index,
                                 UCHAR num_responses , UCHAR rssi_flag);

HCI_EVENT_PKT *hci_generate_connection_complete_event(UCHAR status,
                               UINT16 conn_handle, UCHAR* bd_addr, 
                                   UCHAR link_type, UCHAR encryption_mode);

HCI_EVENT_PKT* hci_generate_connection_request_event(UCHAR  *bd_addr,
                               UINT32 class_of_device, UCHAR link_type);

HCI_EVENT_PKT* hci_generate_disconnection_complete_event(UCHAR status,
                               UINT16 connection_handle, UCHAR reason);

HCI_EVENT_PKT* hci_generate_mode_change_event(UCHAR status, 
                               UINT16 connection_handle, UCHAR current_mode,
                                   UINT16 interval);

HCI_EVENT_PKT* hci_generate_remote_name_request_complete_event(UCHAR status,
                               UINT16 ce_index);

HCI_EVENT_PKT* hci_generate_remote_supported_features_complete_event(
                               UCHAR status, UINT16 connection_handle,
                                   UCHAR *remote_features);

HCI_EVENT_PKT* hci_generate_remote_version_information_complete_event(
                               UCHAR status,UINT16 connection_handle,
                                   UINT16 ce_index);

HCI_EVENT_PKT* hci_generate_clock_offset_complete_event(UCHAR  status,
                               UINT16 connection_handle,
                                   UINT16 clock_offset);

HCI_EVENT_PKT* hci_generate_QoS_setup_complete_event(UCHAR status,
                               UINT16 ce_index);

HCI_EVENT_PKT* hci_generate_role_change_event(UCHAR status, UCHAR *bd_addr,
                               UCHAR new_role);

HCI_EVENT_PKT* hci_generate_max_slots_change_event(UINT16 connection_handle,
                               UCHAR max_slots);

HCI_EVENT_PKT* hci_generate_connection_packet_type_changed_event(UCHAR status,
                               UINT16 connection_handle,UINT16 packet_type);

__inline UINT32 bt_fw_strtoul(const UCHAR *buffer, UCHAR bytes, UCHAR base);

__inline void bt_fw_ultostr(UCHAR *buffer, UINT32 number, UCHAR bytes);

UCHAR hci_pass_event_through_event_mask(UCHAR event_opcode);

UCHAR hci_pass_event_through_event_filter(UCHAR filter_type,UCHAR *bd_addr,
                               UINT32 class_of_device);

UCHAR hci_start_role_switch(UINT16 ce_index);

#ifdef AUTH

HCI_EVENT_PKT *hci_generate_authentication_complete_event(UINT16 conn_handle,
                               UCHAR cmd_status);

HCI_EVENT_PKT *hci_generate_encryption_change_event(UINT16 conn_handle, 
                               UCHAR cmd_status,UCHAR encryption_mode);

HCI_EVENT_PKT *hci_generate_change_connection_link_key_complete_event(
                               UINT16 conn_handle, UCHAR cmd_status);

HCI_EVENT_PKT *hci_generate_master_link_key_complete_event(
                               UINT16 conn_handle, UCHAR cmd_status,
                                   UCHAR key_type);

HCI_EVENT_PKT *hci_generate_return_link_key_event(UCHAR *key_buffer, 
                               UCHAR num_key);

HCI_EVENT_PKT *hci_generate_pincode_request_event(UCHAR *bd_addr);

HCI_EVENT_PKT *hci_generate_linkkey_request_event(UCHAR *bd_addr);

HCI_EVENT_PKT *hci_generate_link_key_notification_event(UCHAR *bd_addr, 
                               UCHAR *key, UCHAR key_type);
#endif /* AUTH */

#define HCI_HANDLE_1_2_LINK_CONTROL_COMMAND        \
            hci_handle_1_2_link_control_command

#define HCI_HANDLE_1_2_LINK_POLICY_COMMAND         \
            hci_handle_1_2_link_policy_command

#define HCI_HANDLE_1_2_HC_BB_COMMAND               \
            hci_handle_1_2_hc_bb_command

#define HCI_HANDLE_1_2_STATUS_COMMAND              \
            hci_handle_1_2_status_command

#define HCI_GENERATE_1_2_COMMAND_COMPLETE_EVENT    \
            hci_generate_1_2_command_complete_event

#endif /*__HCI_H__ */

