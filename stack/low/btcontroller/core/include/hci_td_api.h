/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

#ifndef _HCI_TD_API_H_
#define _HCI_TD_API_H_

#include "bt_fw_types.h"
#include "bt_fw_hci_defines.h"

API_RESULT hci_td_init(void);

API_RESULT hci_td_deliver_event_to_host( UCHAR * event_ptr);
// added by guochunxia for test 20090407
//API_RESULT hci_td_deliver_acl_to_host_test( UCHAR * acl_ptr);

API_RESULT hci_td_deliver_acl_data_to_host(UCHAR *data_header, UCHAR *pkt_ptr);

void hci_td_handle_tx_completed(UCHAR *buffer, UINT16 len, UINT8 type);

void hci_td_notify_data_received(void);

#if defined(SCO_OVER_HCI) || defined(COMPILE_ESCO)

API_RESULT hci_td_deliver_synchronous_data_to_host(UCHAR *pkt_ptr);

#endif  /* SCO_OVER_HCI || COMPILE_ESCO */

#endif /* _HCI_TD_API_H_ */

