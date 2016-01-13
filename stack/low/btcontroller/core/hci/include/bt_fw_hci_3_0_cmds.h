/***************************************************************************
 Copyright (C) MindTree Consulting Ltd.
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

#ifdef VER_3_0

#ifndef __BT_FW_HCI_3_0_CMDS_H__
#define __BT_FW_HCI_3_0_CMDS_H__

/* Function declarations */


UCHAR hci_handle_3_0_hc_bb_command(HCI_CMD_PKT *hci_cmd_ptr);
UCHAR hci_handle_3_0_status_command(HCI_CMD_PKT *hci_cmd_ptr);
UCHAR hci_generate_3_0_command_complete_event( UINT16 cmd_opcode,
                                               UCHAR *pkt_param, 
                                               UINT16 ce_index );

#endif /* __BT_FW_HCI_3_0_CMDS_H__ */

#endif

/** @} end: hci_internal */
