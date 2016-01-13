/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  HCI_TD Header file.
 */

/** \addtogroup hci_td HCI Transport Driver Module
 *  @{ */
#ifndef _HCI_TD_H_
#define _HCI_TD_H_

#include "bt_fw_types.h"
#include "bt_fw_os.h"
#include "bz_log_defines.h"
#include "hci_td_api.h"
#include "bt_fw_config.h"
#include "platform.h"
#include "le_ll.h"

#define TD_PKT_QUEUE_SIZE 16 /* Make sure that this is 2^n */ 

#define HCI_TD_MAX_BUFFER (HCI_ACL_DATA_PKT_SIZE + 12)

#define     HCI_TD_CMD_PKT_SIGNAL           2
#define     HCI_TD_ACL_DATA_PKT_SIGNAL      3
#define     HCI_TD_SCO_DATA_PKT_SIGNAL      4

/*
 * Task type will be used as an index into the RX_TABLE and TX_TABLE
 * to get the task ids and the buffer pool ids
 */
#define     HCI_TD_MAX_HANDLER_TASKS        4

typedef enum
{
    /*Receive Table indexes */
    HCI_CMD_HANDLER_TASK    = 0,
    ACL_DATA_HANDLER_TASK   = 1,
    SYNCHRONOUS_DATA_HANDLER_TASK = 2,
#if defined(LE_MODE_EN)
    LE_ACL_DATA_HANDLER_TASK = 3,    
#endif

    /*Transmit Table indexes */
    TRANSPORT_DRIVER_TASK   = 0,
    HCI_EVENT_HANDLER_TASK	= 0

} HCI_TD_TASK_TYPE;

typedef struct
{
    UINT32  task_handle;
    UINT32  pool_handle;
} HCI_TD_RX_TABLE, HCI_TD_TX_TABLE;


typedef enum hci_td_state
{
    HCI_TD_READY =0,
    HCI_TD_TRANSMITTING
}HCI_TD_STATE;

#define SCO_TX_BUF_SIZE 1020

#if 0
void hci_td_handle_received_bytes(UCHAR* buf, UINT16 len);
UCHAR recovered_from_lost_sync(void);
#endif

void hci_td_handle_received_packet(HCI_TRANSPORT_PKT_TYPE pkt_type,
                                   UCHAR *buffer, UINT16 length, UINT32 fifo_index);
void hci_td_tx_complete(UCHAR *buffer, UINT8 type);
#if 0
void hci_td_enque_pkt(HCI_TRANSPORT_PKT_TYPE pkt_type, UCHAR *buffer,
                      UINT16 length);
#endif

void hci_td_tx_packet(HCI_TRANSPORT_PKT_TYPE pkt_type, UCHAR *buffer,
                      UINT16 length);

UCHAR hci_td_process_signal(OS_SIGNAL *signal);

#ifdef HS_USB_SEND_EVENT_REORDER_ISSUE_NEW
/**
 * @brief Check if a HCI event needs to guarantee the following ACL data are in
 * order.
 * @param evt  HCI event to check.
 * @param status  Return status of event if yes.
 * @param conn_handle  Return connection handle of event if yes.
 * @return  \c TRUE if the correct order needs to guarantee; otherwise,
 * \c FALSE.
 */
extern BOOLEAN (*hci_td_check_hci_event_need_in_order)(HCI_EVENT_PKT *evt, UINT8 *status, UINT16 *conn_handle);
#endif

extern HCI_TD_RX_TABLE            rx_table[HCI_TD_MAX_HANDLER_TASKS];
extern HCI_TD_TX_TABLE            tx_table[HCI_TD_MAX_HANDLER_TASKS];
extern OS_HANDLE hci_td_task_handle;

#endif /* _HCI_TD_H_ */

/** @} */

