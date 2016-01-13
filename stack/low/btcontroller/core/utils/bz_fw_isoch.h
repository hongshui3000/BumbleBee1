/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

#ifndef _BZ_FW_ISOCH_H_
#define _BZ_FW_ISOCH_H_

/**
 * \file bz_fw_isoch.h.
 *  Provides interfaces for the intermediate Isochronous SCO queue
 *  for firmware.
 * \author Akshat Kumar
 * \date 2008-01-25
 */

#ifdef SCO_OVER_HCI

#include "bt_fw_types.h"
#include "lmp.h"

/************************ Defines and macros ******************************/

/*********************** Structure Definations ****************************/

/*********************** Function Declarations ****************************/
extern UINT8 bz_isoch_conns;
void bz_isoch_init_q(void);
SCO_ISOCH_MAP* bz_isoch_add_sco_queues(UINT16 sync_conn_handle, UINT8 ce_index);
API_RESULT bz_isoch_remove_sco_queues(UINT16 sync_conn_handle, UINT8 ce_index);
SECTION_ISR API_RESULT bz_isoch_write_sco_tx_data(LMP_SCO_CONNECTION_DATA* sco_ce_ptr);
API_RESULT bz_isoch_send_data_to_host( UINT16 conn_handle,
        UCHAR erroneous_data_reporting, UCHAR fifo_num,
        UINT16 length, UCHAR data_status);
#ifdef _FIX_MULTIPLE_SCO_RECORD_PATH_
void bz_isoch_free_all_packets_in_sub_rx_queue(UINT8 queue_id);
void bz_isoch_free_all_packets_in_rx_queue(void);
void bz_isoch_dequeue_and_send_data_to_host(void);
#endif

#endif /* SCO_OVER_HCI */
#endif /* _BZ_FW_ISOCH_H_ */

