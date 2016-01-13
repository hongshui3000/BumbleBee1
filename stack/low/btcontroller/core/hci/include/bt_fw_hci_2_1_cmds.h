/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  BT2_1 HCI Layer interface.
 *  
 *  \author Muthu Subramanian K
 *  
 */

/** \addtogroup hci_internal
 *  @{ */
#ifndef _HCI_2_1_CMDS_H_
#define _HCI_2_1_CMDS_H_

UCHAR hci_handle_2_1_hc_bb_command(HCI_CMD_PKT *hci_cmd_ptr, 
                                   UCHAR *sent_event);
UCHAR hci_handle_2_1_link_policy_command(HCI_CMD_PKT *hci_cmd_ptr, 
                                   UCHAR *sent_event_flag);
void hci_check_and_enable_eir_trans(void);

#endif // _HCI_2_1_CMDS_H_

/** @} end: hci_internal */
