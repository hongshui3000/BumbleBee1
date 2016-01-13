/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  BT1.2 HCI Layer interface.
 */

/** \addtogroup hci_internal
 *  @{ */
#ifndef _HCI_1_2_CMDS_H_
#define _HCI_1_2_CMDS_H_

/* Module Specific Includes */
#include "bt_fw_hci_1_2_cmds_internal.h"

#if defined(ENABLE_SCO) || defined(COMPILE_ESCO)
/** 
 * Defines Parameters given by the host to create/re-negotiate a synchronous
 * link.
 */
typedef struct
{
    UINT32 tx_bandwidth;    /**< Transmission bandwidth */
    UINT32 rx_bandwidth;    /**< Receive bandwidth */
    UINT16 max_latency;     /**< Maximum allowed latency */
    UINT16 voice_setting;   /**< Voice Settings */
    UINT16 pkt_types;       /**< Allowed packet types */
    UCHAR  retx_effort;     /**< Retransmission effort */
} HCI_SYNC_PARAMS;

/** 
 * Checks whether the \a hci_air_mode is supported or not.
 * 
 * \param air_mode_bitmap All supported air modes (should be in the format
 *                        returned by #lmp_extract_air_mode_bitmap).
 * \param hci_air_mode Air mode to be tested (should be in the format defined
 *                     in \a Voice_Settings parameter of Setup sync command).
 * 
 * \return 1, if the air_mode is supported. 0, otherwise.
 */
#define lmp_is_air_mode_supported(air_mode_bitmap, hci_air_mode)        \
                            ((air_mode_bitmap) & (1<<(hci_air_mode)))
#endif


/* Function Prototype Declarations */

#if defined(ENABLE_SCO) || defined(COMPILE_ESCO)
UCHAR hci_validate_sync_params(UINT16 ce_index, const HCI_SYNC_PARAMS* host_params);

#if defined(ENABLE_SCO) 
UCHAR hci_validate_sco_params(UINT16 ce_index, const HCI_SYNC_PARAMS * host_params);
UCHAR hci_initiate_new_sco_link(UINT16 ce_index, HCI_SYNC_PARAMS * host_params, UCHAR gen_conn_complete);
void hci_release_sco_resources(UINT16 ce_index, UINT16 sco_ce_index, UCHAR reason);
UCHAR hci_handle_new_sco_conn_request(UINT16 ce_index, HCI_SYNC_PARAMS * host_params, UCHAR gen_conn_complete);
#endif

#ifdef COMPILE_ESCO
UCHAR hci_validate_esco_params(UINT16 ce_index, const HCI_SYNC_PARAMS * host_params);
void hci_release_esco_resources(UINT16 ce_index, UINT16 esco_ce_index,
                                UCHAR reason);
#endif

#endif

UCHAR hci_handle_new_sync_link(UINT16 ce_index, HCI_SYNC_PARAMS * host_params, HCI_LINK_TYPE * p_link_type, UCHAR gen_conn_complete);

UCHAR hci_handle_1_2_link_control_command(HCI_CMD_PKT *hci_cmd_ptr);

UCHAR hci_handle_1_2_link_policy_command(HCI_CMD_PKT *hci_cmd_ptr, UINT16 ce_index);

UCHAR hci_handle_1_2_hc_bb_command(HCI_CMD_PKT *hci_cmd_ptr);

UCHAR hci_handle_1_2_status_command(HCI_CMD_PKT *hci_cmd_ptr, UINT16 conn_handle);

UCHAR hci_generate_1_2_command_complete_event( UINT16 cmd_opcode,
                                          UCHAR *pkt_param, UINT16 ext_param);

#ifdef COMPILE_ESCO
UCHAR hci_initiate_new_esco_link(UINT16 ce_index, HCI_SYNC_PARAMS * host_params);
UCHAR hci_handle_esco_renegotiation(UINT16 esco_ce_index, HCI_SYNC_PARAMS * host_params);
#endif

#endif // _HCI_1_2_CMDS_H_

/** @} end: hci_internal */
