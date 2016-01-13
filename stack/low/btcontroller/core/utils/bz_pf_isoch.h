/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

#ifndef _BZ_PF_ISOCH_H_
#define _BZ_PF_ISOCH_H_

/**
 * \file bz_pf_isoch.h
 *  Defines the functions to be used by the platform specific USB driver .
 *  Provides interfaces for the intermediate Isochronous SCO queue.
 * \author Akshat Kumar
 * \date 2008-01-25
 */

#ifdef SCO_OVER_HCI

#include "bt_fw_types.h"

/************************ Defines and macros ******************************/
#define ISOCH_SINGLE_RX_PKT_LENGTH 24  // single rx pkt is 24 (becayse 9*3-3 ?)  
#define ISOCH_SCO_MAX_CONNS 3          // support maximum 3 sco link 
#define ISOCH_RX_QUEUE_MAX_PKTCNT  16  // shall be 2^n

/*********************** Structure Definations ****************************/
typedef struct {
    UINT16 *sco_pkt_buff;
    UINT16 sco_pkt_wr_ptr;
    UINT16 sco_pkt_rd_ptr;
    void   *scoch_map; /* help handle by FW for catch parameters */
}ISOCH_SCO_TX_QUEUE;

typedef struct {
    void* sco_rx_pkt;
    UINT16 fragment_length;
    UINT8 fragment_status; 
    void   *scoch_map; /* help handle by FW for ctach parameters */
}ISOCH_SCO_RX_QUEUE;

typedef struct SCO_TX_ENTRY {
    UINT8 *sco_payload;
    UINT16 length;
    UINT16 remain_len;
    UINT32 free_addr;
#ifdef _SCO_SEND_SINE_TONE_PKT_AND_COMPARE
    UINT8 start_index;
    UINT8 cmp_index;
    UINT8 count;
#endif	
    void   *next;
}SCO_TX_ENTRY_TYPE;

typedef struct SCO_TX_ENTRY_MANAGER {
    SCO_TX_ENTRY_TYPE *header;
    SCO_TX_ENTRY_TYPE *ender;
    SCO_TX_ENTRY_TYPE *wait_head;  
    SCO_TX_ENTRY_TYPE *wait_end;     
    UINT16 offset;
    UINT16 pkt_num;
    
#ifdef _SCO_SEND_SINE_TONE_PKT_   
    UINT16 *tone_src;
    UINT8 tone_repeat_mask;
    UINT8 sine_tx_index;
#endif
}SCO_TX_ENTRY_MANAGER_TYPE;

typedef struct {
    ISOCH_SCO_TX_QUEUE isoch_tx_q;
    ISOCH_SCO_RX_QUEUE isoch_rx_q;
    UINT16 conn_handle;

    /* add by austin */
    UINT16 active:1;         /* this map is active ? */    
    UINT16 sco_ch_id:2;      /* sco channel 0 ~ (ISOCH_SCO_MAX_CONNS - 1) */
    UINT16 host_data_8bit:1; /* host data format is 8 bit */
    UINT16 air_data_8bit:1;  /* air data format is 8 bit */
    UINT16 no_transparent:1; /* dont transparent the codec converter path */
    UINT16 cur_id:2;         /* current id */  
    UINT16 rsvd:8;           /* resvered */   
        
    UINT32 next_instant;
    SCO_TX_ENTRY_MANAGER_TYPE sco_tx_entry_manager; 
}SCO_ISOCH_MAP;

/* this structure is used to help handle two discontinus segments in a 
   tx ring buffer - austin */
typedef struct ISOCH_SCO_TX_QUEUE_INFO_ {
    UINT32 seg_pkt_add[6];
    UINT16 seg_max_len[6];  /* the segment maximum length (unit: half) */    
} ISOCH_SCO_TX_QUEUE_INFO;

typedef struct ISOCH_SCO_HCI_RX_SUB_QUEUE_ {
    UINT32 ring_array[ISOCH_RX_QUEUE_MAX_PKTCNT];   /* the ring array record 
                                                        the pointer of pkt */
    UINT8 wptr;                                     /* write pointer */
    UINT8 rptr;                                     /* read pointer */
    UINT8 pend_pktcnt;                              /* pending packet count */
    UINT8 active;
} ISOCH_SCO_HCI_RX_SUB_QUEUE;

typedef struct ISOCH_SCO_HCI_RX_QUEUE_MANAGER_ {
    ISOCH_SCO_HCI_RX_SUB_QUEUE rxq[ISOCH_SCO_MAX_CONNS];
    UINT8 cur_enqueue_ptr;
    UINT8 cur_dequeue_ptr;
    UINT8 pend_pktcnt;
} ISOCH_SCO_HCI_RX_QUEUE_MANAGER;

/*********************** Function Declarations ****************************/

INLINE UCHAR pf_hci_transport_isoch_get_pkt_length(void);

INLINE void pf_hci_transport_isoch_usb_ready(void);

INLINE UINT8 pf_hci_transport_is_isoch_ups_ready(void);

void* pf_hci_transport_isoch_get_handle(UINT16 sync_conn_handle);

BOOLEAN pf_hci_transport_isoch_dma_queue_rx_fifo_data(void* isoch_handle, 
                                                 UINT16 length, UINT16 con_handle);

void* pf_hci_transport_isoch_get_tx_entry(UINT16 sync_conn_handle);

void pf_hci_init_free_sco_tx_entry(void);

BOOLEAN pf_hci_transport_isoch_queue_pkt(void * pkt);
    
SECTION_ISR BOOLEAN pf_hci_transport_degueue_tx_pkt(
             SCO_ISOCH_MAP * iso_ch_map, UINT16 *tx_length, UINT8 * segm_index, 
             ISOCH_SCO_TX_QUEUE_INFO * pInfo, UINT8 lag);

void pf_hci_free_tx_pkt(UINT8 sco_number);

BOOLEAN pf_hci_transport_free_tx_buf(UINT16 sync_conn_handle);

void pf_hci_transport_reset_isoch_queue(void);

#endif /* SCO_OVER_HCI */
#endif /* _BZ_PF_ISOCH_H_ */

