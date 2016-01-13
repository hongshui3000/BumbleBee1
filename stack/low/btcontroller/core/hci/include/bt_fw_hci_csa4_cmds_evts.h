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
#ifndef _BT_FW_HCI_CSA4_CMDS_EVENTS_H_
#define _BT_FW_HCI_CSA4_CMDS_EVENTS_H_

#include "DataType.h"

/* Function declarations */

/*----------------------------------------------------------------------------*/
/*  The Data Structure of CSA4 HCI Command Packet                             */
/*----------------------------------------------------------------------------*/
/*================================*/
/*     Link Control Commands      */
/*================================*/
/* The Structure of Truncated Page Command Packet (OCF = 0x003F). 
   Refer BT CSA4 CSB HCI spec */
typedef struct CSB_HCI_TRUNCATED_PAGE_CMD_PKT_S_ {
    UINT16 opcode;                  /* octet[1:0]: opcpde */
    UINT8 length;                   /* octet[2]: the length of command parameters */
    
    UINT8 bBD_ADDR[6];              /* octet[8:3]: BD_ADDR of the Device to page */
    UINT8 Page_Scan_Repeition_Mode; /* octet[9]: 0=R0, 1=R1, 2=R2 */
    UINT16 Clock_Offset;            /* octet[11:10]: 
                                       bit[15]: clock_offset is valid or not 
                                       bit[14:0]: Bit 16-2 of CLKslave-CLKmaster */
} CSB_HCI_TRUNCATED_PAGE_CMD_PKT_S;

/* The Structure of Truncated Page Cancel Command Packet (OCF = 0x0040). 
   Refer BT CSA4 CSB HCI spec */
typedef struct CSB_HCI_TRUNCATED_PAGE_CANCEL_CMD_PKT_S_ {
    UINT16 opcode;                  /* octet[1:0]: opcpde */
    UINT8 length;                   /* octet[2]: the length of command parameters */
    
    UINT8 bBD_ADDR[6];              /* octet[8:3]: BD_ADDR of the Device to page */
} CSB_HCI_TRUNCATED_PAGE_CANCEL_CMD_PKT_S;

/* The Structure of Set Connectionless Slave Broadcast Command Packet (OCF = 0x0041). 
   Refer BT CSA4 CSB HCI spec */
typedef struct CSB_HCI_SET_CSB_CMD_PKT_S_ {
    UINT16 opcode;                  /* octet[1:0]: opcpde */
    UINT8 length;                   /* octet[2]: the length of command parameters */
    
    UINT8 Enable;                   /* octet[3]: enable or disable */
    UINT8 LT_Addr;                  /* octet[4]: LT_ADDR used for CSB */
    UINT8 LPO_Allowed;              /* octet[5]: low power osc is allowed ? */
    UINT16 Packet_Type;             /* octet[7:6]: support packet type */
    UINT16 Interval_Min;            /* octet[9:8]: minimum CSB interval */
    UINT16 Interval_Max;            /* octet[11:10]: maximum CSB interval */
    UINT16 CSB_supervisionTO;       /* octet[13:12]: CSB Timeout Duration in slots */
} CSB_HCI_SET_CSB_CMD_PKT_S;

/* The Structure of Set Connectionless Slave Broadcast Receive Command Packet (OCF = 0x0042). 
   Refer BT CSA4 CSB HCI spec */
typedef struct CSB_HCI_SET_CSB_RECEIVE_CMD_PKT_S_ {
    UINT16 opcode;                  /* octet[1:0]: opcpde */
    UINT8 length;                   /* octet[2]: the length of command parameters */
    
    UINT8 Enable;                   /* octet[3]: enable or disable */
    union {
        UINT8 bBD_Addr[6];          /* octet[9:4]: BD_ADDR of CSB transmitter */
        UINT16 wBD_Addr[3];
    };
    UINT8 LT_Addr;                  /* octet[10]: LT_ADDR used for CSB */
    UINT8 bInterval[2];             /* octet[12:11]: CSB interval */
    UINT8 bClock_Offset[4];         /* octet[16:13]: (CLKslave ¡V CLKmaster) modulo 2^27 */
    UINT8 bNext_CSB_Clock[4];       /* octet[20:17]: CLK for next CSB instant */
    UINT8 bCSB_supervisionTO[2];    /* octet[22:21]: CSB Timeout Duration in slots */
    UINT8 Remote_Timing_Accuracy;   /* octet[23]: Timing accuracy of the master in ppm */
    UINT8 Skip;                     /* octet[24]: Number of CSB instants to skip after
                                       successfully receiving a broadcast packet */
    UINT8 bPacket_Type[2];          /* octet[26:25]: support packet type */
    UINT8 bAFH_Ch_Map[10];          /* octet[36:27]: afh channel map */
} CSB_HCI_SET_CSB_RECEIVE_CMD_PKT_S;

/* The Structure of Start Synchronization Train Command Packet (OCF = 0x0043). 
   Refer BT CSA4 CSB HCI spec */
typedef struct CSB_HCI_START_SYNC_TRAIN_CMD_PKT_S_ {
    UINT16 opcode;                  /* octet[1:0]: opcpde */
    UINT8 length;                   /* octet[2]: the length of command parameters */    
} CSB_HCI_START_SYNC_TRAIN_CMD_PKT_S;

/* The Structure of Receive Synchronization Train Command Packet (OCF = 0x0044). 
   Refer BT CSA4 CSB HCI spec */
typedef struct CSB_HCI_RECEIVE_SYNC_TRAIN_CMD_PKT_S_ {
    UINT16 opcode;                  /* octet[1:0]: opcpde */
    UINT8 length;                   /* octet[2]: the length of command parameters */
    
    UINT8 bBD_Addr[6];              /* octet[8:3]: BD_ADDR of CSB transmitter */
    UINT8 bSync_scanTO[2];          /* octet[10:9]: Duration in slots to search 
                                                for the synchronization train */
    UINT8 bSync_Scan_Window[2];     /* octet[12:11]: Sync Scan Window */                                                    
    UINT8 bSync_Scan_Interval[2];   /* octet[14:13]: Sync Scan Interval */
} CSB_HCI_RECEIVE_SYNC_TRAIN_CMD_PKT_S;

/*=====================================*/
/*  Controller and Baseband Commands   */
/*=====================================*/
/* The Structure of Set Reserved LT_ADDR Command Packet (OCF = 0x0074). 
   Refer BT CSA4 CSB HCI spec */
typedef struct CSB_HCI_SET_RESERVED_LT_ADDR_CMD_PKT_S_ {
    UINT16 opcode;                  /* octet[1:0]: opcpde */
    UINT8 length;                   /* octet[2]: the length of command parameters */

    UINT8 LT_Addr;                  /* octet[3]: LT_ADDR to reserved for CSB */
} CSB_HCI_SET_RESERVED_LT_ADDR_CMD_PKT_S;

/* The Structure of Delete Reserved LT_ADDR Command Packet (OCF = 0x0075). 
   Refer BT CSA4 CSB HCI spec */
typedef struct CSB_HCI_DELETE_RESERVED_LT_ADDR_CMD_PKT_S_ {
    UINT16 opcode;                  /* octet[1:0]: opcpde */
    UINT8 length;                   /* octet[2]: the length of command parameters */

    UINT8 LT_Addr;                  /* octet[3]: LT_ADDR to reserved for CSB */
} CSB_HCI_DELETE_RESERVED_LT_ADDR_CMD_PKT_S;

/* The Structure of Set Connectionless Slave Broadcast Data Command Packet (OCF = 0x0076). 
   Refer BT CSA4 CSB HCI spec */
typedef struct CSB_HCI_SET_CSB_DATA_CMD_PKT_S_ {
    UINT16 opcode;                  /* octet[1:0]: opcpde */
    UINT8 length;                   /* octet[2]: the length of command parameters */

    UINT8 LT_Addr;                  /* octet[3]: LT_ADDR on which to send CSB data */
    UINT8 Fragment;                 /* octet[4]: fragment status */
    UINT8 Data_Length;              /* octet[5]: Length of the Data field */
    UINT8 Data[0];                  /* octet[~6]: Data to send in future CSB packets */    
} CSB_HCI_SET_CSB_DATA_CMD_PKT_S;

/* The Structure of Read Synchronization Train Parameters Command Packet (OCF = 0x0077). 
   Refer BT CSA4 CSB HCI spec */
typedef struct CSB_HCI_READ_SYNC_TRAIN_PARAMS_CMD_PKT_S_ {
    UINT16 opcode;                  /* octet[1:0]: opcpde */
    UINT8 length;                   /* octet[2]: the length of command parameters */ 
} CSB_HCI_READ_SYNC_TRAIN_PARAMS_CMD_PKT_S;

/* The Structure of Write Synchronization Train Parameters Command Packet (OCF = 0x0078). 
   Refer BT CSA4 CSB HCI spec */
typedef struct CSB_HCI_WRITE_SYNC_TRAIN_PARAMS_CMD_PKT_S_ {
    UINT16 opcode;                  /* octet[1:0]: opcpde */
    UINT8 length;                   /* octet[2]: the length of command parameters */ 

    UINT8 bInterval_Min[2];         /* octet[4:3]: minimum Sync Train interval */
    UINT8 bInterval_Max[2];         /* octet[6:5]: maximum Sync Train interval */
    


    UINT8 bSync_TrainTO[4];         /* octet[10:7]: Duration in slots to continue 
                                       sending for the synchronization train */
    UINT8 Service_Data;             /* octet[11]: host provided value 
                                       for octet 27 of STP payload body */
} CSB_HCI_WRITE_SYNC_TRAIN_PARAMS_CMD_PKT_S;

/*================================*/
/*     Status Commands            */
/*================================*/
/* The Structure of Set Triggered Clock Capture Command Packet (OCF = 0x000D). 
   Refer BT CSA4 CSB HCI spec */
typedef struct CSB_HCI_SET_TRIGGERED_CLOCK_CAPTURE_CMD_PKT_S_ {
    UINT16 opcode;                  /* octet[1:0]: opcpde */
    UINT8 length;                   /* octet[2]: the length of command parameters */ 

    UINT8 bConnecton_Handle[2];     /* octet[4:3]: used to identify a connection */
    UINT8 Enable;                   /* octet[5]: disable or enable triggered clock capturing 
                                          on the specified Connection Handle */
    UINT8 Which_Clock;              /* octet[6]: Local or Piconet clock */
    UINT8 LPO_Allowed;              /* octet[7]: low power osc is allowed ? */
    UINT8 Num_Clk_Cap_To_Filter;    /* octet[8]: Number of triggered clock 
                                       captures filtered between sending a 
                                       Triggered Clock Capture event to the Host */
} CSB_HCI_SET_TRIGGERED_CLOCK_CAPTURE_CMD_PKT_S;



/*----------------------------------------------------------------------------*/
/*  The Data Structure of CSA4 HCI Event (the address must be 2-byte aligned) */
/*----------------------------------------------------------------------------*/
/* The Structure of Triggered Clock Capture Event (Event Code = 0x4E). 
   Refer BT CSA4 CSB HCI spec */
typedef struct CSB_TRIGGERED_CLOCK_CAPTURE_EVT_PARA_S_ {
    union {
        UINT16 Connection_Handle;   /* offset[1:0]: connection handle */
        UINT8 bConnection_Handle[2];
    };
    UINT8 Which_Clock;              /* offset[2]: local or piconet clock */
    UINT8 bClock[4];                /* offset[6:3]: bluetooth clock of the device
                                       requested with bits 1 and 0 set to 0b */
    UINT8 bSlot_Offset[2];          /* offset[8:7]: slot offset (0 ~ 1249 us) */   
} CSB_TRIGGERED_CLOCK_CAPTURE_EVT_PARA_S;

/* The Structure of Synchronization Train Complete Event (Event Code = 0x4F). 
   Refer BT CSA4 CSB HCI spec */
typedef struct CSB_SYNC_TRAIN_COMPLETE_EVT_PARA_S_ {
    UINT8 Status;                   /* offset[0]: event status */ 
} CSB_SYNC_TRAIN_COMPLETE_EVT_PARA_S;

/* The Structure of Synchronization Train Received Event (Event Code = 0x50).
   Refer BT CSA4 CSB HCI spec */
typedef struct CSB_SYNC_TRAIN_RECEIVED_EVT_PARA_S_ {
    UINT8 Status;                   /* offset[0]: event status */ 
    UINT8 bBD_ADDR[6];              /* offset[7:1]: BD Address */
    UINT8 bClock_Offset[4];         /* offset[11:8]: (CLKslave - CLKmaster) mod 2^27 */
    UINT8 bAFH_Ch_Map[10];          /* offset[21:12]: afh channel map */
    UINT8 LT_Addr;                  /* offset[22]: LT Address */
    UINT8 bNext_Broadcast_Inst[4];  /* offset[26:23]: clk of next broadtcast on this channel */
    UINT8 bCSB_Interval[2];         /* offset[28:27]: interval between CSB instant in slots */
    UINT8 Service_Data;             /* offset[29]: value from octet 27 of STP */
} CSB_SYNC_TRAIN_RECEIVED_EVT_PARA_S;

/* The Structure of Connectionless Slave Broadcast Receive Event (Event Code = 0x51).
   Refer BT CSA4 CSB HCI spec */
typedef struct CSB_CSB_RECEIVED_EVT_PARA_S_ {
    union {
        UINT8 bBD_ADDR[6];          /* offset[5:0]: BD Address */
        UINT16 wBD_ADDR[3];
    };
    UINT8 LT_Addr;                  /* offset[6]: LT Address */
    UINT8 bCLK[4];                  /* offset[10:7]: clock when CSB data was received */
    UINT8 bOffset[4];               /* offset[13:11]: (CLKslave - CLKmaster) mod 2^27 */
    UINT8 Receive_Status;           /* offset[14]: Packet received status */
    UINT8 Fragment;                 /* offset[15]: fragment status */
    UINT8 Data_Length;              /* offset[16]: Length of Data Field */    
    UINT8 Data[0];                  /* offset[~:17]: Data received from a CSB packet.*/
} CSB_CSB_RECEIVED_EVT_PARA_S;


/* The Structure of Connectionless Slave Broadcast Timeout Event (Event Code = 0x52).
   Refer BT CSA4 CSB HCI spec */
typedef struct CSB_CSB_TIMEOUT_EVT_PARA_S_ {
    union {
        UINT8 bBD_ADDR[6];          /* offset[5:0]: BD Address */
        UINT16 wBD_ADDR[3];
    };
    UINT8 LT_Addr;                  /* offset[6]: LT Address */
} CSB_CSB_TIMEOUT_EVT_PARA_S;

/* The Structure of Truncated Page Complete Event (Event Code = 0x53).
   Refer BT CSA4 CSB HCI spec */
typedef struct CSB_TRUNCATED_PAGE_COMPLETE_EVT_PARA_S_ {
    UINT8 Status;                   /* offset[0]: the status of truncated_page command */
    UINT8 bBD_ADDR[6];              /* offset[6:1]: BD Address */
} CSB_TRUNCATED_PAGE_COMPLETE_EVT_PARA_S;

/* The Structure of Truncated Page Complete Event (Event Code = 0x54).
   Refer BT CSA4 CSB HCI spec */
typedef struct CSB_SLAVE_PAGE_RESPONSE_TIMEOUT_EVT_PARA_S_ {
} CSB_SLAVE_PAGE_RESPONSE_TIMEOUT_EVT_PARA_S;

/* The Structure of Connectionless Slave Broadcast Channel Map Change Event(Event Code = 0x55).
   Refer BT CSA4 CSB HCI spec */
typedef struct CSB_CSB_CH_MAP_CHANGE_EVT_PARA_S_ {    
    UINT8 bChannel_Map[10];        /* offset[9:0]: Channel Map */
} CSB_CSB_CH_MAP_CHANGE_EVT_PARA_S;

/* The Structure of Inquiry Response Notification Event (Event Code = 0x56).
   Refer BT CSA4 CSB HCI spec */
typedef struct CSB_INQUIRY_RESP_NOTIFICATION_EVT_PARA_S_ {
    UINT8 bLAP[3];                /* offset[2:0]: LAP from which the IAC was derived */
    UINT8 rssi;                   /* offset[3]: Range: -127 to +20, signed integer
                                     , (2¡¦s complement), Units: dBm */
} CSB_INQUIRY_RESP_NOTIFICATION_EVT_PARA_S;


UCHAR hci_handle_truncated_page(HCI_CMD_PKT *hci_cmd_ptr);
UCHAR hci_handle_truncated_page_cancel(HCI_CMD_PKT *hci_cmd_ptr);
UCHAR hci_handle_set_connectionless_slave_broadcast(HCI_CMD_PKT *hci_cmd_ptr);
UCHAR hci_handle_set_connectionless_slave_broadcast_receive(HCI_CMD_PKT *hci_cmd_ptr);
UCHAR hci_handle_start_synchronization_train_command(HCI_CMD_PKT *hci_cmd_ptr);
UCHAR hci_handle_receive_synchronization_train_command(HCI_CMD_PKT *hci_cmd_ptr);
UCHAR hci_handle_set_reserved_lt_addr_command(HCI_CMD_PKT *hci_cmd_ptr);
UCHAR hci_handle_delete_reserved_lt_addr_command(HCI_CMD_PKT *hci_cmd_ptr);
UCHAR hci_handle_set_connectionless_slave_broadcast_data(HCI_CMD_PKT *hci_cmd_ptr);
UCHAR hci_handle_read_synchronization_train_parameters(HCI_CMD_PKT *hci_cmd_ptr);
UCHAR hci_handle_write_synchronization_train_parameters(HCI_CMD_PKT *hci_cmd_ptr);
UCHAR hci_handle_set_triggered_clock_capture(HCI_CMD_PKT *hci_cmd_ptr);
UCHAR hci_handle_csa4_link_controller_commands(HCI_CMD_PKT *hci_cmd_ptr);
UCHAR hci_handle_csa4_hc_bb_commands(HCI_CMD_PKT *hci_cmd_ptr);
UCHAR hci_handle_csa4_status_commands(HCI_CMD_PKT *hci_cmd_ptr);

UCHAR hci_generate_csa4_command_complete_event( UINT16 cmd_opcode,
                                               UCHAR *pkt_param, 
                                               UINT16 ce_index );
void hci_generate_csa4_triggered_clock_capture_event(void);
void hci_generate_csa4_sychronization_train_complete_event(UINT8 status);

#ifdef _SUPPORT_CSB_RECEIVER_
void hci_generate_csa4_sychronization_train_received_event(UINT8 status);
void hci_generate_csa4_connectionless_slave_broadcast_receive_event(void);
#endif
void hci_generate_csa4_connectionless_slave_broadcast_timeout_event(UINT8 rx);

#ifdef _SUPPORT_CSB_RECEIVER_
void hci_generate_csa4_truncated_page_complete_event(UINT8 status, UINT16 ce_index);
#endif
void hci_generate_csa4_slave_page_response_timeout_event(void);
void hci_generate_csa4_connectionless_slave_broadcast_channel_map_change_event(void);
void hci_generate_csa4_inquiry_response_notification_event(void);
#endif /* _BT_FW_HCI_CSA4_CMDS_EVENTS_H_ */

/** @} end: hci_internal */

