/***************************************************************************
 Copyright (C) Realtek
 This module is a confidential and proprietary property of Realtek and
 a possession or use of this module requires written permission of Realtek.
 ***************************************************************************/

/**
 * \file
 *  le_hci_4_0.h (Low Energy Host Controller Interface 4.0 Definition)
 *
 * \author
 */

/** \addtogroup Low Energy HCI 4.0
 *  @{ */
#ifndef _LE_HCI_4_0_H_
#define _LE_HCI_4_0_H_
#include "bt_fw_hci_external_defines.h"
#include "DataType.h"
#include "le_ll.h"
#include "bt_fw_config.h"

#define LL_HCI_MIN_CONN_HANDLE  \
             (LMP_MAX_CE_DATABASE_ENTRIES + LMP_MAX_BC_CONNECTIONS + \
              LMP_MAX_SCO_CONNECTIONS + 1)
#define LL_HCI_MAX_CONN_HANDLE  \
             (LL_HCI_MIN_CONN_HANDLE + LL_MAX_CONNECTION_UNITS - 1)

#define LE_CONN_COMPLETE_EVENT_PARA_SIZE                        19
#define LE_CONN_UPDT_COMPLETE_EVENT_PARA_SIZE                   10
#define LE_READ_REMOTE_USED_FEATURES_COMPLETE_EVENT_PARA_SIZE   12
#define LE_LONG_TERM_KEY_REQUEST_EVENT_PARA_SIZE                13
#define LE_READ_LOCAL_P256_PUBLIC_KEY_COMPLETE_PARA_SIZE        66
#define LE_GENERATE_DHKEY_COMPLETE_EVENT_PARA_SIZE              34
#define LE_ENHANCED_CONN_COMPLETE_EVENT_PARA_SIZE               31


/* LE Advertising Type in LE HCI Advertising parameters */
enum LL_HCI_ADV_TYPE_SET {
    LL_HCI_ADV_TYPE_ADV_IND,           //0
    LL_HCI_ADV_TYPE_ADV_DIRECT_IND,    //1
    LL_HCI_ADV_TYPE_ADV_DISCOVER_IND,  //2
    LL_HCI_ADV_TYPE_ADV_NONCONN_IND,   //3
#ifdef _DAPE_LOW_DUTY_ADV
    LL_HCI_ADV_TYPE_ADV_LOW_DUTY_DIRECT_IND,   //4
    LL_HCI_ADV_TYPE_ADV_MAX = LL_HCI_ADV_TYPE_ADV_LOW_DUTY_DIRECT_IND,
#else
    LL_HCI_ADV_TYPE_ADV_MAX = LL_HCI_ADV_TYPE_ADV_NONCONN_IND,
#endif
};


/* LE Event Type in LE Advertising Report Event */
enum LL_HCI_ADV_REPORT_EVENT_TYPE {
    LL_HCI_ADV_REPORT_EVENT_TYPE_ADV_IND,
    LL_HCI_ADV_REPORT_EVENT_TYPE_ADV_DIRECT_IND,
    LL_HCI_ADV_REPORT_EVENT_TYPE_ADV_DISCOVER_IND,
    LL_HCI_ADV_REPORT_EVENT_TYPE_ADV_NONCONN_IND,
    LL_HCI_ADV_REPORT_EVENT_TYPE_SCAN_RSP,
    LL_HCI_ADV_REPORT_EVENT_TYPE_MAX = LL_HCI_ADV_REPORT_EVENT_TYPE_SCAN_RSP,
    LL_HCI_ADV_REPORT_EVENT_TYPE_INVALID = 0xFF,
};


/* LE Advertising Filter Policy in LE HCI Advertising parameters */
enum LL_HCI_ADV_FILTER_POLICY_SET {
    LL_HCI_ADV_FILT_POLICY_ANY_SCAN_ANY_CONN,
    LL_HCI_ADV_FILT_POLICY_WHITE_LIST_SCAN_ANY_CONN,
    LL_HCI_ADV_FILT_POLICY_ANY_SCAN_WHITE_LIST_CONN,
    LL_HCI_ADV_FILT_POLICY_WHITE_LIST_SCAN_WHITE_LIST_CONN,
    LL_HCI_ADV_FILT_POLICY_MAX = LL_HCI_ADV_FILT_POLICY_WHITE_LIST_SCAN_WHITE_LIST_CONN,
};

/* LE Packet Payload Type in Test Mode */
enum LL_TEST_PKT_TYPE_SET {
    LL_TEST_PKT_TYPE_PRBS9,
    LL_TEST_PKT_TYPE_REPEAT_11110000,
    LL_TEST_PKT_TYPE_REPEAT_10101010,
    LL_TEST_PKT_TYPE_PRBS15,
    LL_TEST_PKT_TYPE_ALL_1,
    LL_TEST_PKT_TYPE_ALL_0,
    LL_TEST_PKT_TYPE_REPEAT_00001111,
    LL_TEST_PKT_TYPE_REPEAT_01010101,
    LL_TEST_PKT_TYPE_MAX = LL_TEST_PKT_TYPE_REPEAT_01010101
};

#ifndef LE_HW_TEST
#define LL_HCI_GEN_COMMAND_COMPLETE_EVENT(evt_buf, return_param_len) \
    hci_generate_event(HCI_COMMAND_COMPLETE_EVENT, evt_buf, 3 + (return_param_len))

#define LL_HCI_GEN_LE_MEGA_EVENT(evt_buf, evt_param_len) \
    hci_generate_event(HCI_LE_MEGA_EVENT, evt_buf, evt_param_len)
#else
#define LL_HCI_GEN_COMMAND_COMPLETE_EVENT(return_para_len)
#define LL_HCI_GEN_LE_MEGA_EVENT(event_para_len)
#endif

/**
 * @brief Advertising data header.
 *
 * Data with length #len - 2 octets are appended to this header to form a AD
 * structure.
 */
typedef EIR_HEADER AD_HEADER;

#define AD_DATA_SIZE(p) EIR_DATA_SIZE(p)
#define NEXT_AD_HEADER(p) NEXT_EIR_HEADER(p)

#define ad_header_find(buf, buf_size, type) eir_header_find(buf, buf_size, type)

#define AD_TYPE_MANUFACTURER_DATA EIR_TYPE_MANUFACTURER_DATA

/**
 * @brief Manufacturer specific data header.
 */
typedef EIR_MANUFACTURER_DATA_HEADER AD_MANUFACTURER_DATA_HEADER;

#define AD_MANUFACTURER_DATA_GET_COMPANY_ID(o) \
    EIR_MANUFACTURER_DATA_GET_COMPANY_ID(o)

/*---------------------------------------------------------------------------*/
/*  The Data Structure of LE Meta Event (the address must be 2-byte aligned) */
/*---------------------------------------------------------------------------*/
/* The Structure of LE Connection Complete Event. Refer 7.7.65.1 on page 797
   in BT 4.0 HCI spec */
typedef struct LE_CONNECTION_COMPLETE_EVENT_PARA_S_ {
    UINT8 Subevent_Code;        /* subevent code (0x01) */
    UINT8 Status;               /* report status */
    UINT16 Connection_Handle;   /* connection handle */
    UINT8 Role;                 /* connection is master or slave */
    UINT8 Peer_Address_Type;    /* peer uses public or random device address */
    UINT16 Peer_Address[3];     /* device Address of the peer device */
#ifndef _BT4_2_PRIVACY_FEATURE_SUPPORT_
    UINT16 Conn_Interval;       /* connection interval for this connection */
    UINT16 Conn_Latency;        /* connection latency for this connection */
    UINT16 Supervision_Timeout; /* connection supervision timeout */
    UINT8 Master_Clock_Accuracy; /* master clock accuracy */
#endif
} LE_CONNECTION_COMPLETE_EVENT_PARA_S;


/* The Structure of LE Advertising Report Event. Refer 7.7.65.2 on page 800 in
    BT 4.0 HCI spec */
typedef struct LE_ADVERTISING_REPORT_EVENT_PARA_S_ {
    UINT8 Subevent_Code;        /* subevent code (0x02) */
    UINT8 Num_Reports;          /* report status */
    UINT8 Event_Type[0];        /* start pointer of Event_Type[0] */
} LE_ADVERTISING_REPORT_EVENT_PARA_S;

/* The Structure of LE Connection Update Complete Event. Refer 7.7.65.3 on page
    801 in BT 4.0 HCI spec */
typedef struct LE_CONNECTION_UPDATE_COMPLETE_EVENT_PARA_S_ {
    UINT8 Subevent_Code;        /* subevent code (0x03) */
    UINT8 Status;               /* report status */
    UINT16 Connection_Handle;   /* connection handle */
    UINT16 Conn_Interval;       /* connection interval for this connection */
    UINT16 Conn_Latency;        /* connection latency for this connection */
    UINT16 Supervision_Timeout; /* connection supervision timeout */
} LE_CONNECTION_UPDATE_COMPLETE_EVENT_PARA_S;

/* The Structure of LE Read Remote Used Features Complete Event. Refer 7.7.65.4
    on page 803 in BT 4.0 HCI spec */
typedef struct LE_READ_REMOTE_USED_FEATURES_COMPLETE_EVENT_PARA_S_ {
    UINT8 Subevent_Code;        /* subevent code (0x04) */
    UINT8 Status;               /* report status */
    UINT16 Connection_Handle;   /* connection handle */
    UINT16 u2LE_Features[4];    /* Bit Mask List of used LE features */
} LE_READ_REMOTE_USED_FEATURES_COMPLETE_EVENT_PARA_S;

/* The Structure of LE Long Term Key Request Event. Refer 7.7.65.5 on page 804
    in BT 4.0 HCI spec */
typedef struct LE_LONG_TERM_KEY_REQUEST_EVENT_PARA_S_ {
    UINT8 Subevent_Code;                /* subevent code (0x05) */
    UINT8 u1Connection_Handle[2];       /* connection handle */
    UINT8 u1Random_Number[8];           /* 64 bit random number */
    UINT8 u1Encrypted_Diversifier[2];   /* 16 bit encrypted diversifier */
} LE_LONG_TERM_KEY_REQUEST_EVENT_PARA_S;

#ifdef _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_
typedef struct LE_DATA_LENGTH_CHANGE_EVT_PARAM_
{
    UINT8 subevt_code;
    UINT8 conn_handle[2];
    UINT8 max_tx_octets[2];
    UINT8 max_tx_time[2];
    UINT8 max_rx_octets[2];
    UINT8 max_rx_time[2];
} LE_DATA_LENGTH_CHANGE_EVT_PARAM;
#endif

extern UINT8 (*(hci_le_control_command[])) (HCI_CMD_PKT *);

UCHAR hci_handle_4_0_hc_bb_command(HCI_CMD_PKT * hci_cmd_ptr);
UCHAR hci_generate_4_0_command_complete_event( UINT16 cmd_opcode,
                                               UCHAR *pkt_param,
                                               UINT16 ext_param );
UCHAR hci_4_0_handle_remote_version_information_command(UINT16 conn_handle,
                                            LL_CONN_HANDLE_UNIT * * pp_handle);
UINT8 hci_pass_event_through_le_event_mask(UCHAR subevent_opcode);
UINT8 hci_handle_le_invalid_command(HCI_CMD_PKT *hci_cmd_ptr);
UINT8 hci_handle_le_set_event_mask(HCI_CMD_PKT *hci_cmd_ptr);
UINT8 hci_handle_le_read_buffer_size(HCI_CMD_PKT *hci_cmd_ptr);
UINT8 hci_handle_le_read_local_supported_features(HCI_CMD_PKT *hci_cmd_ptr);
UINT8 hci_handle_le_set_random_address(HCI_CMD_PKT *hci_cmd_ptr);
UINT8 hci_handle_le_set_advertising_parameters(HCI_CMD_PKT *hci_cmd_ptr);
UINT8 hci_handle_le_read_advertising_channel_tx_power(HCI_CMD_PKT *hci_cmd_ptr);
UINT8 hci_handle_le_set_advertising_data(HCI_CMD_PKT *hci_cmd_ptr);
UINT8 hci_handle_le_set_scan_response_data(HCI_CMD_PKT *hci_cmd_ptr);
UINT8 hci_handle_le_set_advertising_enable(HCI_CMD_PKT *hci_cmd_ptr);
UINT8 hci_handle_le_set_scan_parameters(HCI_CMD_PKT *hci_cmd_ptr);
UINT8 hci_handle_le_set_scan_enable(HCI_CMD_PKT *hci_cmd_ptr);
UINT8 hci_handle_le_create_connection(HCI_CMD_PKT *hci_cmd_ptr);
UINT8 hci_handle_le_create_connection_cancel(HCI_CMD_PKT *hci_cmd_ptr);
UINT8 hci_handle_le_read_white_list_size(HCI_CMD_PKT *hci_cmd_ptr);
UINT8 hci_handle_le_clear_white_list(HCI_CMD_PKT *hci_cmd_ptr);
UINT8 hci_handle_le_add_device_to_white_list(HCI_CMD_PKT *hci_cmd_ptr);
UINT8 hci_handle_le_remove_device_from_white_list(HCI_CMD_PKT *hci_cmd_ptr);
UINT8 hci_handle_le_connection_update(HCI_CMD_PKT *hci_cmd_ptr);
UINT8 hci_handle_le_set_host_channel_classification(HCI_CMD_PKT *hci_cmd_ptr);
UINT8 hci_handle_le_read_channel_map(HCI_CMD_PKT *hci_cmd_ptr);
UINT8 hci_handle_le_read_remote_used_features(HCI_CMD_PKT *hci_cmd_ptr);
UINT8 hci_handle_le_encrypt(HCI_CMD_PKT *hci_cmd_ptr);
UINT8 hci_handle_le_rand(HCI_CMD_PKT *hci_cmd_ptr);
UINT8 hci_handle_le_start_encryption(HCI_CMD_PKT *hci_cmd_ptr);
UINT8 hci_handle_le_long_term_key_request_reply(HCI_CMD_PKT *hci_cmd_ptr);
UINT8 hci_handle_le_long_term_key_requested_negative_reply(HCI_CMD_PKT *hci_cmd_ptr);
UINT8 hci_handle_le_read_supported_states(HCI_CMD_PKT *hci_cmd_ptr);
UINT8 hci_handle_le_receiver_test(HCI_CMD_PKT *hci_cmd_ptr);
UINT8 hci_handle_le_transmitter_test(HCI_CMD_PKT *hci_cmd_ptr);
UINT8 hci_handle_le_test_end(HCI_CMD_PKT *hci_cmd_ptr);
UINT8 hci_execute_4_0_hc_le_control_command(HCI_CMD_PKT *hci_cmd_ptr);

void hci_generate_le_advertising_report_event(void);
void hci_generate_le_command_complete_event_head(UINT16 hci_cmd_opcode, UCHAR *event_parameter);
#ifdef _BT5_0_LE2MBPS_SUPPORT_
UCHAR hci_handle_le_receiver_test_general(HCI_CMD_PKT *hci_cmd_ptr, UINT8 enhanced);
UCHAR hci_handle_le_transmitter_test_general(HCI_CMD_PKT *hci_cmd_ptr, UINT8 enhanced);
UINT8 hci_handle_le_read_phy_command(HCI_CMD_PKT *hci_cmd_ptr);
UINT8 hci_handle_le_set_default_phy_command(HCI_CMD_PKT *hci_cmd_ptr);
UINT8 hci_handle_le_set_phy_command(HCI_CMD_PKT *hci_cmd_ptr);
UINT8 hci_handle_le_enhanced_rx_test_command(HCI_CMD_PKT *hci_cmd_ptr);
UINT8 hci_handle_le_enhanced_tx_test_command(HCI_CMD_PKT *hci_cmd_ptr);

#endif
#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_
void hci_generate_le_connection_complete_basic(UINT8 status, LL_CONN_HANDLE_UNIT *pHandle, BOOLEAN isEnhanced);
#define hci_generate_le_connection_complete_event(status, pHandle) hci_generate_le_connection_complete_basic(status, pHandle, FALSE)
#define hci_generate_le_enhanced_connection_complete_event(status, pHandle) hci_generate_le_connection_complete_basic(status, pHandle, TRUE)
#else
void hci_generate_le_connection_complete_event(UINT8 status, LL_CONN_HANDLE_UNIT * pHandle);
#endif
void hci_generate_le_connection_update_complete_event(UINT8 status,
                                            LL_CONN_HANDLE_UNIT * pHandle);
void hci_generate_le_long_term_key_request_event(LL_CONN_HANDLE_UNIT * pHandle);
void hci_generate_le_read_remote_used_features_complete_event(UINT8 status,
                                                 LL_CONN_HANDLE_UNIT * pHandle);
#ifdef _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_
void hci_generate_le_data_length_change_event(LL_CONN_HANDLE_UNIT *chu);
#endif

#ifdef _BT4_2_LE_SECURE_CONNECTIONS_SUPPORT_
#include <bz_auth_internal.h>
void hci_generate_le_read_local_p256_public_key_complete_event(void);
void hci_generate_le_generate_dhkey_complete_event(void);
#endif

#endif
