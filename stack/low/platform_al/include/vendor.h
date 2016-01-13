/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

#ifndef __VENDOR_H__
#define __VENDOR_H__

extern UINT16 lc_switch_temp_flag;

#ifdef TEST_MODE
void lmp_test_mode_generate_not_accepted_pdu(LMP_PDU_PKT *, UCHAR );
#endif//TEST_MODE

#define WHITENING_STATUS    0x0010


#define TEST_CONTROL_EVENT_TYPE      0x01

/*
 * Special bit that enables the usage of SCO packet types [HV1, HV2, HV3] 
 * even without the presense of an actual SCO link. this feature is required
 * for SCO packet loopback for Test control master command.
 */
#define ENABLE_SCO_PKT_LOOPBACK_TX          0x0020

/*
 * Disable the special bit that is used for SCO packet loopback once the
 * test is over.
 */
#define DISABLE_SCO_PKT_LOOPBACK_TX       0xffdf

#define     BASEBAND_HOP_79         0x30
#ifdef _DAPE_TEST_GEN_FAKE_ESCO_DATA
void gen_esco_data_packet(UINT8 change);
#endif

#if defined(RT_VENDOR_CMDS)
UCHAR hci_handle_vendor_cmds(HCI_CMD_PKT *hci_cmd_ptr);
HCI_EVENT_PKT * hci_vendor_generate_command_complete_event(UINT16 hci_cmd_opcode, UCHAR cmd_status, void *arg);
#endif //defined(RT_VENDOR_CMDS)

#define  LMP_HANDLE_MSS_FAIL_AT_FHS_ID()                                
#define  LMP_HANDLE_MSS_FAIL_AT_POLL_NULL(read)                             
#define  LMP_HANDLE_MSS_FAIL_NCTO(read)                
#define LC_CHECK_FOR_INCR_PKT_RX()
#define INIT_CONTROL_POLL_NULL()
#define LC_HANDLE_HEC_ERR(tx_pkt_type)

#endif /* __VENDOR_H__ */

