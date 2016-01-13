/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  HCI Layer external interface for V2_1.
 *
 * \author Muthu Subramanian K
 *
 */

/** \addtogroup hci_external
 *  @{ */

#ifndef __HCI_2_1_H__
#define __HCI_2_1_H__

void hci_generate_lsto_change_event(UINT16 conn_handle, UINT16 to_value);
void hci_generate_enhanced_flush_event(UINT16 conn_handle);
void hci_generate_sniff_subrating_event(UCHAR status, UINT16 ce_index);
void hci_generate_eir_event(UCHAR bd_addr[6],UCHAR page_scan_rep_mode,
                            UINT32 cod, UINT16 clk_off,
                            UCHAR rssi, UCHAR *eir_data, UCHAR eir_len);
UCHAR hci_generate_2_1_command_complete_event( UINT16 cmd_opcode,
                                          UCHAR *pkt_param, UINT16 ext_param);
void hci_generate_rhsfn_event(UCHAR* bd_addr, UCHAR* remote_host_features);
void hci_check_and_enable_eir_recv(void);
#endif

/** @} end: hci_external */
