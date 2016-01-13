/***************************************************************************
 Copyright (C) Realtek Semiconductor Corp.
 This module is a confidential and proprietary property of Realtek and
 a possession or use of this module requires written permission of Realtek.
 ***************************************************************************/

/**
 * \file
 *  CSA4 HCI Layer interface.
 *  
 *  \author Austin Chen
 *  
 */

/** \addtogroup hci_internal
 *  @{ */
#ifndef _BT_FW_HCI_SECURE_CONN_CMDS_EVENTS_H_
#define _BT_FW_HCI_SECURE_CONN_CMDS_EVENTS_H_

#include "DataType.h"

/* Function declarations */

/*----------------------------------------------------------------------------*/
/*  The Data Structure of SECURE-CONN HCI Command Packet                             */
/*----------------------------------------------------------------------------*/
/*================================*/
/*     Link Control Commands      */
/*================================*/
/* The Structure of Truncated Page Command Packet (OCF = 0x003F). 
   Refer BT SECURE_CONN HCI spec */

/*=====================================*/
/*  Controller and Baseband Commands   */
/*=====================================*/
/* The Structure of Read Secure Connection Host Support Command Packet (OCF = 0x0079). 
   Refer BT Secure Connection HCI spec */
typedef struct SECURE_CONN_HCI_READ_SECURE_CONN_HOST_SUPPORT_CMD_PKT_S_ {
    UINT16 opcode;                  /* octet[1:0]: opcpde */
    UINT8 length;                   /* octet[2]: the length of command parameters */ 
} SECURE_CONN_HCI_READ_SECURE_CONN_HOST_SUPPORT_CMD_PKT_S;

/* The Structure of Write Secure Connection Host Support Command Packet (OCF = 0x007A). 
   Refer BT Secure Connection HCI spec */
typedef struct SECURE_CONN_HCI_WRITE_SECURE_CONN_HOST_SUPPORT_CMD_PKT_S_ {
    UINT16 opcode;                  /* octet[1:0]: opcpde */
    UINT8 length;                   /* octet[2]: the length of command parameters */ 
    UINT8 Secure_Conn_Host_Support; /* octet[3]: Secure Connection Host Support */
} SECURE_CONN_HCI_WRITE_SECURE_CONN_HOST_SUPPORT_CMD_PKT_S;

/* The Structure of Read Authentication Payload Timeout Command Packet (OCF = 0x007B). 
   Refer BT Secure Connection HCI spec */
typedef struct SECURE_CONN_HCI_READ_AUTH_PAYLOAD_TO_CMD_PKT_S_ {
    UINT16 opcode;                  /* octet[1:0]: opcpde */
    UINT8 length;                   /* octet[2]: the length of command parameters */ 
    UINT8 bConn_handle[2];          /* octet[4:3]: Connection Handle */
} SECURE_CONN_HCI_READ_AUTH_PAYLOAD_TO_CMD_PKT_S;

/* The Structure of Write Authentication Payload Timeout Command Packet (OCF = 0x007C). 
   Refer BT Secure Connection HCI spec */
typedef struct SECURE_CONN_HCI_WRITE_AUTH_PAYLOAD_TO_CMD_PKT_S_ {
    UINT16 opcode;                  /* octet[1:0]: opcpde */
    UINT8 length;                   /* octet[2]: the length of command parameters */ 
    UINT8 bConn_handle[2];          /* octet[4:3]: Connection Handle */
    UINT8 bAuth_Payload_TO[2];       /* octet[6:5]: Authentication Payload TO */
} SECURE_CONN_HCI_WRITE_AUTH_PAYLOAD_TO_CMD_PKT_S;

/* The Structure of Read Local OOB Extended Data Command Packet (OCF = 0x007D). 
   Refer BT Secure Connection HCI spec */
typedef struct SECURE_CONN_HCI_READ_LOCAL_OOB_EXT_DATA_CMD_PKT_S_ {
    UINT16 opcode;                  /* octet[1:0]: opcpde */
    UINT8 length;                   /* octet[2]: the length of command parameters */ 
} SECURE_CONN_HCI_READ_LOCAL_OOB_EXT_DATA_CMD_PKT_S;

/*=====================================*/
/*  Testing Commands   */
/*=====================================*/
/* The Structure of Write Secure Connection Test Mode Command Packet (OCF = 0x000A). 
   Refer BT Secure Connection HCI spec */
typedef struct SECURE_CONN_HCI_WRITE_SC_TEST_MODE_CMD_PKT_S_ {
    UINT16 opcode;                  /* octet[1:0]: opcpde */
    UINT8 length;                   /* octet[2]: the length of command parameters */ 
    UINT8 bConn_handle[2];          /* octet[4:3]: Connection Handle */
    UINT8 DM1_ACLU_mode;            /* octet[5]: DM1 ACL-U Mode */ 
    UINT8 esco_loopback_mode;       /* octet[6]: eSCO loopback mode */ 

} SECURE_CONN_HCI_WRITE_SC_TEST_MODE_CMD_PKT_S;



/*----------------------------------------------------------------------------*/
/*  The Data Structure of Secure Conn HCI Event (the address must be 2-byte aligned) */
/*----------------------------------------------------------------------------*/
/* The Structure of Authenticated Payload Timeout Expired Event (Event Code = 0x4E). 
   Refer BT Secure Connection HCI spec */
typedef struct SECURE_CONN_AUTH_PAYLOAD_TO_EXPIRED_EVT_PARA_S_ {
    union {
        UINT16 Connection_Handle;   /* offset[1:0]: connection handle */
        UINT8 bConnection_Handle[2];
    };
} SECURE_CONN_AUTH_PAYLOAD_TO_EXPIRED_EVT_PARA_S;

UCHAR hci_handle_read_secure_conn_host_support_command(HCI_CMD_PKT *hci_cmd_ptr);
UCHAR hci_handle_write_secure_conn_host_support_command(HCI_CMD_PKT *hci_cmd_ptr);
UCHAR hci_handle_read_auth_payload_timeout(HCI_CMD_PKT *hci_cmd_ptr);
UCHAR hci_handle_write_auth_payload_timeout(HCI_CMD_PKT *hci_cmd_ptr);
UCHAR hci_handle_read_local_oob_ext_data(HCI_CMD_PKT *hci_cmd_ptr);
UCHAR hci_handle_write_secure_conn_test_mode_command(HCI_CMD_PKT *hci_cmd_ptr);


UCHAR hci_handle_secure_conn_link_controller_commands(HCI_CMD_PKT *hci_cmd_ptr);
UCHAR hci_handle_secure_conn_hc_bb_commands(HCI_CMD_PKT *hci_cmd_ptr);
UCHAR hci_handle_secure_conn_testing_commands(HCI_CMD_PKT *hci_cmd_ptr);
UCHAR hci_generate_secure_conn_command_complete_event( UINT16 cmd_opcode,
                                               UCHAR *pkt_param, 
                                               UINT16 ce_index );
void hci_generate_authentication_payload_timeout_expired_event(UINT16 conn_handle);


#endif /* _BT_FW_HCI_SECURE_CONN_CMDS_EVENTS_H_ */

/** @} end: hci_internal */

