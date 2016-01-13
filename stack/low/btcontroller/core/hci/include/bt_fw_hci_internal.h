/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  HCI Layer internal interface.
 */

/** \addtogroup hci_internal
 *  @{ */
#ifndef __HCI_INTERNAL_H__
#define __HCI_INTERNAL_H__

/* Main level header files */
#include "bt_fw_common.h"
#include "bt_fw_globals.h"

/* Module header files */
#include "lmp.h"
#include "lc.h"
#include "hci_td.h"

/*Local header files */
#include "bt_fw_hci_defines.h"

/* External variables */

#ifdef _DAPE_RECORD_COMMAND_POINTER_FOR_EVENT
extern HCI_CMD_PKT *g_record_hci_cmd_pkt_ptr;
#endif
/* Command and Event buffer pool handles */
extern POOL_ID hci_event_buffer_pool_handle;
extern POOL_ID hci_cmd_buffer_pool_handle;

/* Command and Event Task handles */
extern TASK_ID hci_command_task_handle;
extern TASK_ID hci_event_task_handle;
extern UCHAR bt_fw_page_cmd_issued;

extern UCHAR hci_baseband_cmd_pending;

#define REVC_DATA_Q_SIZE            ACL_RX_DES_NUM

typedef enum recv_data_state
{
    DATA_IDLE =0,
    DATA_SENT
}RECV_DATA_STATE;

typedef struct hci_recv_data_q
{
    HCI_ACL_RX_DATA_PKT *data[REVC_DATA_Q_SIZE];
    UINT32 header[REVC_DATA_Q_SIZE];
    UINT16 length[REVC_DATA_Q_SIZE];
    UINT16 frag_offset[REVC_DATA_Q_SIZE];
    RECV_DATA_STATE state[REVC_DATA_Q_SIZE];
    UINT16 queue_length;
    UCHAR wr_ptr;
    UCHAR rd_ptr;

}HCI_DATA_RECV_Q;

/* Command Group Opcode */

typedef enum
{
    LINK_CONTROL_CMD_OPCODE_GROUP       = 0X01,
    LINK_POLICY_CMD_OPCODE_GROUP        = 0X02,
    HC_AND_BASEBAND_CMD_OPCODE_GROUP    = 0X03,

#ifdef TEST_MODE
    TEST_MODE_OPCODE_GROUP              = 0x06,
#endif /*TEST_MODE */

    VENDOR_PARMS_OPCODE_GROUP           = 0x3F,

    RTK_VENDOR_PARAMS_OPCODE_GROUP      = 0x66,

    INFO_PARMS_OPCODE_GROUP	            = 0X04,

	STATUS_CMD_OPCODE_GROUP	            = 0X05

#ifdef LE_MODE_EN
    ,LE_CTRL_CMD_OPCODE_GROUP           = 0x08
#endif
}HCI_CMD_OPCODE_GROUP ;

/* Function prototypes */

UCHAR    HCI_Command_Task(OS_SIGNAL *signal_ptr);
void     HCI_Event_Task(OS_SIGNAL *signal_ptr);
void     hci_initialize_data(void);

/* HCI commands handling function prototypes */

UCHAR hci_execute_link_control_command_packet (HCI_CMD_PKT *hci_cmd_ptr);
UCHAR hci_execute_link_policy_command_packet (HCI_CMD_PKT *hci_cmd_ptr);
UCHAR hci_execute_HC_and_baseband_command_packet(HCI_CMD_PKT *hci_cmd_ptr);
UCHAR hci_execute_info_params_command_packet (HCI_CMD_PKT *hci_cmd_ptr);
UCHAR hci_execute_status_command_packet(HCI_CMD_PKT *hci_cmd_ptr);

/*Link Control commands */

UCHAR hci_handle_create_connection_command(HCI_CMD_PKT *hci_cmd_ptr);

UCHAR hci_handle_accept_connection_request_command(
                                             HCI_CMD_PKT *hci_cmd_pkt, UCHAR *);

UCHAR hci_handle_reject_connection_request_command(HCI_CMD_PKT *hci_cmd_ptr,
        UCHAR* sent_cmd_status);

UCHAR hci_handle_add_sco_connection_command(HCI_CMD_PKT *hci_cmd_ptr,
        UCHAR* sent_event_flag);

UCHAR hci_handle_disconnect_command(HCI_CMD_PKT *hci_cmd_ptr,
                                             UCHAR *baseband_cmd_flag);

UCHAR hci_handle_remote_name_request_command(HCI_CMD_PKT *hci_cmd_ptr,
                                             UCHAR * baseband_cmd_flag);

UCHAR hci_handle_remote_supported_features_command(HCI_CMD_PKT *hci_cmd_ptr,
                                             UCHAR * sent_event_flag);

UCHAR hci_handle_remote_version_information_command(HCI_CMD_PKT *hci_cmd_ptr,
                                             UCHAR * sent_event_flag);

UCHAR hci_handle_clock_offset_command(HCI_CMD_PKT *hci_cmd_ptr,
                                             UCHAR * sent_event_flag);

UCHAR hci_read_lmp_handle_command(HCI_CMD_PKT *hci_cmd_ptr,
                                             UCHAR * sent_event_flag);

UCHAR hci_handle_inquiry_command(HCI_CMD_PKT *hci_cmd_ptr);

#ifdef COMPILE_PERIODIC_INQUIRY
UCHAR hci_handle_periodic_inquiry_command(HCI_CMD_PKT *hci_cmd_ptr);
#endif

/*Link Policy commands */

UCHAR hci_handle_hold_mode_command(UCHAR *hci_cmd_ptr, UINT16 ce_index);

UCHAR hci_handle_sniff_mode_command(UCHAR *hci_cmd_ptr, UINT16 ce_index);

UCHAR hci_handle_exit_sniff_mode_command(UINT16 ce_index);

UCHAR hci_handle_park_mode_command(HCI_CMD_PKT *hci_cmd_ptr, UINT16 ce_index);

UCHAR hci_handle_exit_park_mode_command(UINT16 ce_index);

UCHAR hci_handle_qos_setup_command(HCI_CMD_PKT *hci_cmd_ptr, UINT16 ce_index);

UCHAR hci_handle_write_link_policy_settings_command(UCHAR *hci_cmd_ptr,
                                        UINT16 ce_index);
UCHAR hci_handle_write_default_link_policy_settings_command(UCHAR *hci_cmd_ptr);
                                        
UCHAR hci_handle_role_discovery_command(HCI_CMD_PKT *hci_cmd_ptr,
                                        UINT16 *ce_index);                                        

UCHAR hci_handle_park_mode_command_bottom_half(UINT16 ce_index);

#ifdef COMPILE_ROLE_SWITCH
UCHAR hci_handle_switch_role_command_bottom_half(UINT16 ce_index);
#endif

UCHAR hci_handle_switch_role_command(HCI_CMD_PKT * hci_cmd_ptr,UINT16 * ce_idx);

/* Host controller and baseband commands */

UCHAR hci_module_reset(void);

UCHAR hci_handle_set_event_filter_command(HCI_CMD_PKT *hci_cmd_ptr);

UCHAR hci_handle_set_event_mask_command(HCI_CMD_PKT *hci_cmd_ptr);

UCHAR hci_handle_reset_command(void);

UCHAR hci_handle_write_scan_enable_command( HCI_CMD_PKT *hci_cmd_pkt_ptr);

UCHAR hci_handle_write_page_scan_activity_command(
                                            HCI_CMD_PKT *hci_cmd_pkt_ptr );

UCHAR hci_handle_write_inquiry_scan_activity_command(
                                            HCI_CMD_PKT *hci_cmd_pkt_ptr);

UCHAR hci_handle_write_connection_accept_timeout_command(
                                 HCI_CMD_PKT *hci_cmd_pkt_ptr);

UCHAR hci_handle_change_local_name_command(HCI_CMD_PKT *hci_cmd_pkt_ptr);

UCHAR hci_handle_write_page_timeout_command(HCI_CMD_PKT *hci_cmd_pkt_ptr);

UCHAR hci_handle_write_class_of_device_command(HCI_CMD_PKT *hci_cmd_pkt_ptr);

UCHAR hci_handle_write_voice_setting_command(HCI_CMD_PKT *hci_cmd_pkt_ptr);

UCHAR hci_handle_write_link_supervision_timeout_command(
                                HCI_CMD_PKT *hci_cmd_ptr, UINT16 *ce_index);

UCHAR hci_handle_write_page_scan_period_mode_command(
                                HCI_CMD_PKT *hci_cmd_pkt_ptr);

UCHAR hci_handle_write_page_scan_mode_command(
                                HCI_CMD_PKT *hci_cmd_pkt_ptr,
                                    UCHAR *sent_event_flag);

UCHAR hci_handle_change_connection_packet_type_command(
	          HCI_CMD_PKT *hci_cmd_ptr, UCHAR *sent_event_flag, 
	          UCHAR *hci_generate_ccpt_event_flag,
	          UCHAR *hci_max_slots_change_event_flag);

UCHAR hci_handle_flush_command(HCI_CMD_PKT *hci_cmd_ptr, UINT16 *ce_index);

UCHAR hci_handle_write_num_brcast_retran(HCI_CMD_PKT *hci_cmd_ptr);

#ifndef _DISABLE_HCI_HOST_FLOW_CONTROL_
UCHAR hci_handle_write_sco_ctrl_enable(HCI_CMD_PKT *hci_cmd_ptr);

UCHAR hci_handle_set_hc_to_host_flow_ctrl(HCI_CMD_PKT *hci_cmd_ptr);
#endif

UCHAR hci_handle_write_current_iac_lap(HCI_CMD_PKT *hci_cmd_ptr);

UCHAR hci_handle_read_transmit_pwr_lvl(HCI_CMD_PKT *hci_cmd_ptr, UINT16 *ce_index);

UCHAR hci_handle_read_automatic_flush_timeout(
                                HCI_CMD_PKT *hci_cmd_ptr, UINT16 *ce_index);

UCHAR hci_handle_write_automatic_flush_timeout(
                                HCI_CMD_PKT *hci_cmd_ptr,UINT16 *ce_index);

#ifndef _DISABLE_HCI_HOST_FLOW_CONTROL_
UCHAR hci_handle_host_num_of_comp_pkts(HCI_CMD_PKT *hci_cmd_ptr,
                                UINT16 *ce_index);

void hci_reset_host_num_complete(UINT16 conn_handle, UINT16 ce_index);
#endif

/* Status parameters. */
void hci_handle_data_to_host(HCI_ACL_RX_DATA_PKT *hci_data_pkt);

void hci_handle_data_tx_completed(UCHAR param);

void handle_host_num_of_comp_pkts(UINT16 num_completed_pkts);

UCHAR hci_start_role_switch(UINT16 ce_index);

/*Event Generation prototypes */
void hci_generate_command_status_event(
                                    UINT16 hci_cmd_opcode, UCHAR cmd_status);

void hci_generate_command_complete_event(
                                    UINT16 hci_cmd_opcode, UCHAR cmd_status,
                                    UINT16 ce_index, void *arg);

void hci_generate_number_of_completed_packets_event(void);

void hci_generate_inquiry_complete_event(UCHAR inquiry_status);

void hci_generate_inquiry_result_event(
                                    UCHAR inquiry_data_index, 
                                    UCHAR num_responses,
                                    UCHAR rssi_flag );

void hci_generate_connection_complete_event(
                                    UCHAR status, UINT16 conn_handle,
                                    UCHAR* bd_addr, UCHAR link_type,
                                    UCHAR encryption_mode );

void hci_generate_disconnection_complete_event(UCHAR status, 
                                    UINT16 connection_handle,UCHAR reason);

void hci_generate_mode_change_event(
                                    UCHAR status, UINT16 ce_index,
                                    UCHAR current_mode, UINT16 interval );

void hci_generate_remote_name_request_complete_event (
                                    UCHAR  status, UINT16 ce_index);

void hci_generate_remote_supported_features_complete_event (
                                    UCHAR  status,
                                    UINT16 connection_handle,
                                    UCHAR *remote_features);

void hci_generate_remote_version_information_complete_event(
                                    UCHAR  status, UINT16 connection_handle, 
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

#ifdef TEST_MODE
void hci_generate_loopback_command_event(HCI_CMD_PKT* hci_cmd_pkt_ptr);

void hci_generate_test_mode_connection_complete_event(
                                UCHAR status, UINT16 conn_handle, UCHAR* bd_addr, 
                                UCHAR link_type, UCHAR encryption_mode);
#endif /* TEST_MODE */

UCHAR   hci_handle_host_buffer_size_command(HCI_CMD_PKT *hci_cmd_pkt_ptr);

void hci_unpark_new_conn_timeout_event(UINT16 ce_index);

API_RESULT hci_generate_qos_params(UINT16 ce_index, UCHAR flow_direction);

UCHAR hci_handle_read_link_supervision_timeout_command(
        HCI_CMD_PKT *hci_cmd_ptr, UCHAR *sent_evnet_flag);



#ifdef TEST_MODE

UCHAR hci_execute_test_mode_command_packet(HCI_CMD_PKT *hci_cmd_ptr);

UCHAR hci_handle_read_loopback_command(HCI_CMD_PKT *hci_cmd_ptr);

UCHAR hci_handle_write_loopback_command(HCI_CMD_PKT *hci_cmd_ptr);

UCHAR hci_handle_enable_dut_command(HCI_CMD_PKT *hci_cmd_ptr);

#endif /* TEST_MODE */

extern UINT32 hci_calculate_tpoll_token_rate(UINT32 token_rate);
extern UINT32 hci_get_available_bw(void);
extern void hci_queue_QoS_complete_event(UINT16 ce_index, UCHAR error_code);

void hci_queue_flow_spec_complete_event(UINT16 ce_index, 
        UCHAR flow_direction, UCHAR error_code);

UCHAR hci_validate_sniff_params(UCHAR *hci_cmd_ptr, UINT16 ce_index);
UCHAR hci_validate_hold_params(UCHAR *hci_cmd_ptr, UINT16 ce_index);
UCHAR hci_validate_park_params(UCHAR *hci_cmd_ptr, UINT16 ce_index);
UCHAR hci_validate_inquiry_cmd_params(HCI_CMD_PKT *hci_cmd_ptr);
UCHAR hci_validate_periodic_inquiry_cmd_params(HCI_CMD_PKT *hci_cmd_ptr);
UCHAR hci_validate_create_connection_cmd_params(HCI_CMD_PKT *hci_cmd_ptr);
UCHAR hci_validate_switch_role_params(UCHAR* hci_cmd_ptr);

UCHAR hci_is_full_bandwidth(void);

BOOLEAN hci_generate_event(UCHAR event_code, void *event_param, UCHAR event_param_len);
BOOLEAN hci_handle_masked_event(UINT16 ce_index, UCHAR event_opcode);

UCHAR lmp_calculate_min_tpoll_from_qos_and_flow_spec(UINT16 ce_index);
#ifdef _DAPE_DETECT_WHCK_P2P
void detect_whck_p2p(HCI_CMD_PKT *hci_cmd_ptr, UCHAR *cmd_group_opcode);
void auto_enter_whck(UINT8 whck_item);
#endif

#endif /* __HCI_INTERNAL_H__ */

/** @} end: hci_internal */
