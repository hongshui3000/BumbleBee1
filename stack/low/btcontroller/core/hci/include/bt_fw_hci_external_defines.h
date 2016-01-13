/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/
/**
 * \file
 * Contains definitions for HCI Comand opcodes, Event codes and
 * Error codes. This file also contains HCI comand, Event, ACL data and
 * SCO data structure definitions. 
 */
/** \addtogroup hci_external
 *  @{ */
#ifndef __HCI_EXTERNAL_DEFINES_H__
#define __HCI_EXTERNAL_DEFINES_H__
/* Main level include header files */
#include "bt_fw_hci_spec_defines.h"
#include "bt_fw_types.h"
#include "bt_fw_config.h"

#ifdef TEST_MODE
/*
 * The Test Control Master HCI command parameters structure.
 * Used by the Master device when the TCM HCI command is received.
 * Master sends TCMaster PDU. Once the remote sends LMP Accepted,
 * the params are taken from this structure, and programmed into the
 * baseband according to the test scenario.
 */
typedef struct
{
    UCHAR test_scenario;
    UCHAR hopping_mode;
    UCHAR tx_frequency;
    UCHAR rx_frequency;
    UCHAR power_control_mode;
    UCHAR poll_period;
    UINT16 num_packets;
    UINT16 pkt_desc; /* packet_type_description */
    UCHAR packet_type_in_pdu; /* Packet type that came in Test ctrl pdu */
    UCHAR test_pattern;
}TEST_CONTROL_PARAMS;

typedef struct
{
    UCHAR test_state;
    UCHAR dut_burst_data_flag;
    TEST_CONTROL_PARAMS tc_params;
}TEST_MODE_INFO;
#endif /* TEST_MODE */

/** 
 * Link Type 
 */
typedef enum
{
    INVALID_LINK_TYPE =                               -1,
    SCO_LINK =                                      0x00,
    ACL_LINK =                                      0x01,
    ESCO_LINK =                                     0x02
}HCI_LINK_TYPE;
/**
 * Event Filters. 
 */
typedef enum
{
    CLEAR_ALL_FILTERS =                             0x00,
    INQUIRY_RESULT_FILTER =                         0x01,
    CONNECTION_SETUP_FILTER =                       0x02
}HCI_EVENT_FILTERS;
/**
 * Link type for the synchronous links through v1.2 commands 
 */
typedef enum
{
    NONE,
    NEW_SYNC_LINK,
    NEW_SCO_LINK,  // not used
    NEW_ESCO_LINK,  // not used
    EXISTING_ESCO_LINK
}SYNC_LINK_STATUS;
/**
 * Command packet 
 */
typedef struct
{
    UINT16 cmd_opcode;
    /* Length of parameter not the number of paramenters */
    UCHAR param_total_length;
    UCHAR cmd_parameter[HCI_MAX_CMD_PARAM_LEN];
}HCI_CMD_PKT;
/**
 * Event packet  
 */
typedef struct
{
    UCHAR event_opcode;
    UCHAR param_total_length;
    UCHAR event_parameter[HCI_MAX_EVENT_PARAM_LEN];
}HCI_EVENT_PKT;

typedef struct HCI_ACL_DATA_PKT_WS_
{
    UINT32 flush_time;

    /* The length of data scheduled for the first LC Schedule PKT */
    UINT16 tx_length1;
    /* The length of data scheduled for the second LC Schedule PKT */
    UINT16 tx_length2;
    /*
     * If tx_length2 is zero, then write data of length tx_length1
     * starting from hci_acl_data_pkt[0x00] to BB TX FIFO.
     *
     * If tx_length2 is no zero, then write data of length tx_length2
     * starting from hci_acl_data_pkt[tx_length1] to BB TX_FIFO.
     * */

    /* Hold the value of total length of data scheduled for transmission
     * including the other pkts fragments.
     * */
    UINT16 total_frame_length;

    /* The ackd_length will be incremented upon reception of BB ACK.
     * If the ackd_length >= acl_data_total_length, then  the transmission of
     * this pkt is completed and can generate num_complete.
     * Transfer the (akd_length - acl_data_total_length) to next pkt.
     * Also increment the frame_pkt_count of next pkt by (frame_pkt_count-1).
     * Tranverse the list and apply the same logic.
     */
    UINT16 ackd_length;

    /* Hold the information nof number of pkts involved in the scheduled
     * transmission in the scope of this pkt.  The default value is 0x1.
     * 1 => Only this pkt is involved.
     * 2 => This pkt + one more pkts involved.
     * 3 => This pkt + 2 more pkts involved.
     * ......
     */
    UCHAR  frame_pkt_count;
    UCHAR  am_addr;
    UCHAR  in_transmission;
    UCHAR  phy_piconet_id;

    /* This pkt type, and length will be used when schedule
     * this pkt after NBC Timeout */
    UCHAR  acl_pkt_tx_failed;
    UCHAR  acl_pkt_nbc_old_count;
    UINT16  selected_pkt_type;
    UINT16 failed_frame_length;
    UINT16 dummy;
} HCI_ACL_DATA_PKT_WS;

#define HCI_ACL_8BYTE_ALIGN    ((HCI_ACL_DATA_PAYLOAD_SIZE+4) & 0x07)

/**
 * ACL Rx data packet 
 */
typedef struct HCI_ACL_RX_DATA_PKT_
{
    /* HCI Spec defined ACL DATA PKT */
    UINT32 connection_handle : 12 ;
    UINT32 packet_boundary_flag : 2;
    UINT32 broadcast_flag : 2 ;
    UINT32 acl_data_total_length : 16 ;
    UCHAR  hci_acl_data_pkt[HCI_ACL_RX_DATA_PAYLOAD_SIZE];
    struct HCI_ACL_RX_DATA_PKT_ *next; /* point to the next HCI ACL pkt */    
}HCI_ACL_RX_DATA_PKT;

/**
 * ACL data packet 
 */
typedef struct HCI_ACL_DATA_PKT_
{
    /* HCI Spec defined ACL DATA PKT */
    UINT32 connection_handle : 12 ;
    UINT32 packet_boundary_flag : 2;
    UINT32 broadcast_flag : 2 ;
    UINT32 acl_data_total_length : 16 ;
    UCHAR  hci_acl_data_pkt[HCI_ACL_DATA_PAYLOAD_SIZE];
#if (HCI_ACL_8BYTE_ALIGN != 0)
    UINT8 dummy[8 - HCI_ACL_8BYTE_ALIGN];
#endif
    struct HCI_ACL_DATA_PKT_ *next; /* point to the next HCI ACL pkt */
    HCI_ACL_DATA_PKT_WS *ws; /* point to the workspace in the DMEM */
}HCI_ACL_DATA_PKT;

typedef struct HCI_ACL_DATA_ZERO_PKT_
{
    UINT32 connection_handle : 12 ;
    UINT32 packet_boundary_flag : 2;
    UINT32 broadcast_flag : 2 ;
    UINT32 acl_data_total_length : 16 ;
    HCI_ACL_DATA_PKT_WS ws;
}HCI_ACL_DATA_ZERO_PKT;

void hci_send_host_event_masked_signal(UINT16 ce_index, UCHAR event_opcode);

typedef struct HCI_CONNECTION_COMPLETE_EVT_PARAM_
{
    UINT8 status;
    UINT8 conn_handle[2];
    UINT8 bd_addr[6];
    UINT8 link_type;
    UINT8 enc_enabled;
} HCI_CONNECTION_COMPLETE_EVT_PARAM;

typedef struct HCI_DISCONNECTION_COMPLETE_EVT_PARAM_
{
    UINT8 status;
    UINT8 conn_handle[2];
    UINT8 reason;
} HCI_DISCONNECTION_COMPLETE_EVT_PARAM;

typedef struct HCI_ENCRYPTION_CHANGE_EVT_PARAM_
{
    UCHAR status;
    UCHAR conn_handle[2];
    UCHAR enc_enabled;
} HCI_ENCRYPTION_CHANGE_EVT_PARAM;

#ifdef _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_
typedef struct HCI_LE_SET_DATA_LENGTH_CMD_PARAM_
{
    UINT8 conn_handle[2];
    UINT8 tx_octets[2];
    UINT8 tx_time[2];
} HCI_LE_SET_DATA_LENGTH_CMD_PARAM;

typedef struct HCI_LE_WRITE_DEFAULT_DATA_LENGTH_CMD_PARAM_
{
    UINT8 tx_octets[2];
    UINT8 tx_time[2];
} HCI_LE_WRITE_DEFAULT_DATA_LENGTH_CMD_PARAM;
#endif

/**
 * @struct EIR_HEADER_
 * @brief Extended inquiry response data header.
 *
 * Data with length #len - 2 octets are appended to this header to form a EIR
 * structure.
 */
typedef struct _PACKED_ EIR_HEADER_
{
    UINT8 len; /**< EIR data length. */
    UINT8 type; /**< EIR type. */
} EIR_HEADER;

#define EIR_DATA_SIZE(p) \
    (((EIR_HEADER *)(p))->len + sizeof (((EIR_HEADER *)(p))->len))
#define NEXT_EIR_HEADER(p) ((EIR_HEADER *) ((UCHAR *)(p) + EIR_DATA_SIZE(p)))

#define EIR_TYPE_MANUFACTURER_DATA 0xff

EIR_HEADER *eir_header_find(void *buf, UINT32 buf_size, UINT8 type);

/**
 * @struct EIR_MANUFACTURER_DATA_HEADER_
 * @brief Manufacturer specific data header.
 */
typedef struct _PACKED_ EIR_MANUFACTURER_DATA_HEADER_
{
    EIR_HEADER head; /**< EIR data header. */
    UINT8 company_id[2]; /**< Company ID. */
} EIR_MANUFACTURER_DATA_HEADER;

#define EIR_MANUFACTURER_DATA_GET_COMPANY_ID(o) (letoh16((o)->company_id))

#define BROADCOM_COMPANY_ID     0x000F
#define RTK_COMPANY_ID          0x005D

typedef struct _PACKED_ EIR_LEGACY_3D_INFO_
{
    EIR_MANUFACTURER_DATA_HEADER manu_head;
    UINT8 fixed0;
    UINT8 is_3d_supported : 1;
    UINT8 reserved0 : 1;
    UINT8 send_sync_train : 1;
    UINT8 remote_paired: 1;
    UINT8 showroom_mode : 1;
    UINT8 remote_pairable_mode : 1;
    UINT8 reserved1 : 1;
    UINT8 test_mode : 1;
    UINT8 path_loss_threshold;
} EIR_LEGACY_3D_INFO;

#endif  /* __HCI_EXTERNAL_DEFINES_H__ */
/** @} end: hci_external */
