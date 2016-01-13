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
#ifndef __HCI_DEFINES_H__
#define __HCI_DEFINES_H__
/* Main level include header files */
#include "bt_fw_hci_external_defines.h"
#include "bz_auth.h"
#include "lmp_external_defines.h"
#include "bt_fw_config.h"

extern UCHAR rem_name_cancel_bd_addr[LMP_BD_ADDR_SIZE];

#ifdef _MONITOR_ACLU_TRX_THROUGHPUT_
typedef struct
{
    UINT32 total_tx_len;
    UINT32 total_rx_len;
} HCI_CONN_HANDLE_ACLU_MEASURE_UNIT;

typedef struct
{
    HCI_CONN_HANDLE_ACLU_MEASURE_UNIT unit[LMP_MAX_CONN_HANDLES];
    UINT16 count;
} HCI_CONN_HANDLE_ACLU_MEASURE_UNIT;

extern HCI_CONN_HANDLE_ACLU_MEASURE_UNIT hci_conn_handle_aclu_measure;
#endif

#endif  /* __HCI_DEFINES_H__ */
/** @} end: hci_external */
