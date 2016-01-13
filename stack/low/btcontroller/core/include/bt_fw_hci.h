/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  HCI Layer external interface.
 */

/** \addtogroup hci_external
 *  @{ */
#ifndef __HCI_H__
#define __HCI_H__

#include "bt_fw_types.h"
#include "bt_fw_hci_defines.h"

#include "bt_fw_os.h"

#include "bt_fw_hci_1_2_cmds.h"

#include "bt_fw_hci_2_1_cmds.h"

#ifdef VER_3_0
#include "bt_fw_hci_3_0_cmds.h"
#endif

#ifdef VER_CSA4
#include "bt_fw_hci_csa4_cmds_evts.h"
#endif
#ifdef _SUPPORT_SECURE_CONNECTION_
#include "bt_fw_hci_secure_conn_cmds_evts.h"
#endif
#define BT_FW_EXTRACT_32_BITS(val, buffer) \
{                                                 \
    val = (buffer)[3];                            \
    val <<= 8;                                    \
    val |= (buffer)[2];                           \
    val <<= 8;                                    \
    val |= (buffer)[1];                           \
    val <<= 8;                                    \
    val |= (buffer)[0];                           \
}

#define BT_FW_EXTRACT_16_BITS(val, buffer) \
{                                                 \
    val = (UINT16)(buffer)[1];                    \
    val = (UINT16)(val << 8);                     \
    val = (UINT16)(val | (buffer)[0]);            \
}

#define BT_FW_EXTRACT_32_BITS_WA(val, buffer) \
{                                          \
    val = *((UINT32 *)(buffer));           \
}
    
#define BT_FW_EXTRACT_16_BITS_WA(val, buffer) \
{                                          \
    val = *((UINT16 *)(buffer));           \
}
#define BT_FW_EXTRACT_24_BITS(val, buffer) \
{                                          \
    val   = (buffer)[2];                   \
    val <<= 8;                             \
    val  |= (buffer)[1];                   \
    val <<= 8;                             \
    val  |= (buffer)[0];                   \
}

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

API_RESULT init_hci(void);

API_RESULT hci_shutdown(void);

//void firmware_soft_reset(void);

UCHAR HCI_Command_Task(OS_SIGNAL *signal_ptr);

void HCI_Event_Task(OS_SIGNAL *signal_ptr);

void hci_generate_command_status_event(UINT16 hci_cmd_opcode,
                                                    UCHAR cmd_status);

void hci_generate_command_complete_event(UINT16 hci_cmd_opcode, UCHAR cmd_status, UINT16 ce_index, void *arg);

void hci_generate_number_of_completed_packets_event(void);

void hci_generate_inquiry_complete_event(UCHAR inquiry_status);

void hci_generate_inquiry_result_event(UCHAR inquiry_data_index,
                                        UCHAR num_responses, UCHAR rssi_flag);

void hci_generate_connection_complete_event(UCHAR status,
                                UINT16 conn_handle, UCHAR* bd_addr, 
                                UCHAR link_type, UCHAR encryption_mode);

BOOLEAN hci_generate_connection_request_event(UCHAR  *bd_addr,
                                UINT32 class_of_device, UCHAR link_type);

void hci_generate_disconnection_complete_event(UCHAR status,
                                UINT16 connection_handle, UCHAR reason);

void hci_generate_mode_change_event(UCHAR status, 
                                UINT16 ce_index, UCHAR current_mode,
                                UINT16 interval);

void hci_generate_remote_name_request_complete_event(UCHAR status,
                                UINT16 ce_index);

void hci_generate_remote_supported_features_complete_event(
                                UCHAR status, UINT16 connection_handle,
                                UCHAR *remote_features);

void hci_generate_remote_version_information_complete_event(
                                UCHAR status,UINT16 connection_handle,
                                UINT16 ce_index);

void hci_generate_clock_offset_complete_event(
                                UCHAR  status, UINT16 connection_handle,
                                UINT16 clock_offset);

void hci_generate_QoS_setup_complete_event(
                                UCHAR status,
                                UINT16 ce_index);

void hci_generate_role_change_event(
                                UCHAR status, UCHAR *bd_addr,
                                UCHAR new_role);

void hci_generate_max_slots_change_event(
                                UINT16 connection_handle,
                                UCHAR max_slots);

void hci_generate_connection_packet_type_changed_event(
                                UCHAR status, UINT16 connection_handle,
                                UINT16 packet_type);

void hci_generate_hw_error_event(UCHAR hardware_code);

UINT32 bt_fw_strtoul(const UCHAR *buffer, UCHAR bytes, UCHAR base);

void bt_fw_ultostr(UCHAR *buffer, UINT32 number, UCHAR bytes);

UCHAR hci_pass_event_through_event_mask(UCHAR event_opcode);

#ifdef VER_CSA4
UCHAR hci_pass_event_through_event_mask_page2(UCHAR event_opcode);
#endif

UCHAR hci_pass_event_through_event_filter(UCHAR filter_type,UCHAR *bd_addr,
                               UINT32 class_of_device);

UCHAR hci_start_role_switch(UINT16 ce_index);

#ifdef COMPILE_HOLD_MODE
UCHAR hci_handle_write_hold_mode_activity_command(
                                HCI_CMD_PKT *hci_cmd_pkt_ptr);
#endif

void hci_generate_flow_spec_complete_event(UCHAR status, 
                        UCHAR flow_direction, UINT16 ce_index);

UCHAR hci_handle_flow_specification_command(
        HCI_CMD_PKT *hci_cmd_ptr, UINT16 ce_index);

#ifdef VER_CSA4
#define HCI_HANDLE_CSA4_LC_COMMAND                   \
            hci_handle_csa4_link_controller_commands
#define HCI_HANDLE_CSA4_HC_BB_COMMAND                   \
            hci_handle_csa4_hc_bb_commands
#define HCI_HANDLE_CSA4_STATUS_COMMAND                  \
            hci_handle_csa4_status_commands
#define HCI_GENERATE_CSA4_COMMAND_COMPLETE_EVENT        \
            hci_generate_csa4_command_complete_event
#else
#define HCI_HANDLE_CSA4_LC_COMMAND(p1)                  \
            UNKNOWN_HCI_COMMAND_ERROR
#define HCI_HANDLE_CSA4_HC_BB_COMMAND(p1)               \
            UNKNOWN_HCI_COMMAND_ERROR
#define HCI_HANDLE_CSA4_STATUS_COMMAND(p1)              \
            UNKNOWN_HCI_COMMAND_ERROR            
#define HCI_GENERATE_CSA4_COMMAND_COMPLETE_EVENT(p1,p2,p3) \
            0xFF
#endif
#ifdef _SUPPORT_SECURE_CONNECTION_
#define HCI_HANDLE_SECURE_CONN_LC_COMMAND                   \
            hci_handle_secure_conn_link_controller_commands
#define HCI_HANDLE_SECURE_CONN_HC_BB_COMMAND                   \
            hci_handle_secure_conn_hc_bb_commands
#define HCI_HANDLE_SECURE_CONN_TESTING_COMMAND                   \
            hci_handle_secure_conn_testing_commands
#define HCI_GENERATE_SECURE_CONN_COMMAND_COMPLETE_EVENT        \
            hci_generate_secure_conn_command_complete_event
#else
#define HCI_HANDLE_SECURE_CONN_LC_COMMAND(p1)                  \
            UNKNOWN_HCI_COMMAND_ERROR
#define HCI_HANDLE_SECURE_CONN_HC_BB_COMMAND(p1)               \
            UNKNOWN_HCI_COMMAND_ERROR
#define HCI_HANDLE_SECURE_CONN_TESTING_COMMAND(p1)                  \
            UNKNOWN_HCI_COMMAND_ERROR
#define HCI_GENERATE_SECURE_CONN_COMMAND_COMPLETE_EVENT(p1,p2,p3) \
            0xFF
#endif

#ifdef LE_MODE_EN
#define HCI_HANDLE_4_0_HC_BB_COMMAND                   \
            hci_handle_4_0_hc_bb_command
#define HCI_GENERATE_4_0_COMMAND_COMPLETE_EVENT        \
            hci_generate_4_0_command_complete_event
#else
#define HCI_HANDLE_4_0_HC_BB_COMMAND(p1)                   \
            UNKNOWN_HCI_COMMAND_ERROR
#define HCI_GENERATE_4_0_COMMAND_COMPLETE_EVENT(p1,p2,p3)  \
            0xFF
#endif

#ifdef VER_3_0 
#define HCI_HANDLE_3_0_HC_BB_COMMAND                   \
            hci_handle_3_0_hc_bb_command
#define HCI_HANDLE_3_0_STATUS_COMMAND                  \
            hci_handle_3_0_status_command
#define HCI_GENERATE_3_0_COMMAND_COMPLETE_EVENT        \
            hci_generate_3_0_command_complete_event
#else
#define HCI_HANDLE_3_0_HC_BB_COMMAND(p1)                   \
            UNKNOWN_HCI_COMMAND_ERROR
#define HCI_HANDLE_3_0_STATUS_COMMAND(p1)                  \
            UNKNOWN_HCI_COMMAND_ERROR
#define HCI_GENERATE_3_0_COMMAND_COMPLETE_EVENT(p1,p2,p3) \
            0
#endif

#define HCI_HANDLE_2_1_HC_BB_COMMAND                   \
            hci_handle_2_1_hc_bb_command
#define HCI_HANDLE_2_1_LINK_POLICY_COMMAND             \
            hci_handle_2_1_link_policy_command
#define HCI_GENERATE_2_1_COMMAND_COMPLETE_EVENT        \
            hci_generate_2_1_command_complete_event

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

/** @} end: hci_external */
