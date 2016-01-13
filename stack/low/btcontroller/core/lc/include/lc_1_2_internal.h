/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  Contains the BT1.2 LC module internal API.
 */

/** \addtogroup lc_internal
 *   @{ */
#ifndef _LC_1_2_INTERNAL_H_

#define _LC_1_2_INTERNAL_H_

#define MAX_PKTS_IN_SCHEDULER   (1)
#define HIGH_WATER_MARK         (100)
#define LOW_WATER_MARK          (3)
#define FIFO_PACKET             0
#define DEFAULT_PACKET          1  /* when no data in fifo, use default pkt*/

#define ESCO_STOP_RX            0x0080
#define ESCO_ENABLE_RX          0xFF7F

#define ESCO_ACK                1
#define ESCO_NACK               2
#define ESCO_REXMIT             3

#define ESCO_FIFO_SIZE          (512)


#ifdef COMPILE_ESCO
#define LC_DISABLE_ESCO_RETX        \
            lc_disable_esco_retx
#define LC_RESTORE_ESCO_RETX       \
            lc_restore_esco_retx        
            
#define LC_RETURN_TO_CURRENT_ESCO() \
        lc_return_to_current_esco()  
#else
#define LC_DISABLE_ESCO_RETX(ce_index)
#define LC_RESTORE_ESCO_RETX(ce_index)
#define LC_RETURN_TO_CURRENT_ESCO()
#endif

#define LC_MIN_VAL_DESCO 4

#ifdef COMPILE_DYNAMIC_POLLING
#define LC_TPOLL_OFFSET  2
#else
#define LC_TPOLL_OFFSET  8
#endif

#if defined COMPILE_ESCO || defined SCO_OVER_HCI
void lc_handle_synchronous_data_pkt(HCI_SYNC_DATA_PKT *synchronous_pkt);
#endif //#if defined(SCO_OVER_HCI)

#ifdef COMPILE_ESCO
void lc_make_esco_connection(UINT16 esco_ce_index, UCHAR lt_addr);
void lc_kill_esco_connection(UINT16 esco_ce_index);
UCHAR lc_queue_esco_data_pkt(HCI_SYNC_DATA_PKT *acl_pkt, UINT16 esco_ce_index);
void lc_disable_esco_retx(UINT16 acl_ce_index);
void lc_restore_esco_retx(UINT16 acl_ce_index);
void lc_reset_esco_scheduler(UCHAR acl_ce_index, UCHAR esco_ce_index);
void lc_reset_esco_fifo_new(void);

typedef enum
{
    ESCO_IDLE=0,                    /* init state */
    ESCO_LOADED,                    /* eSCO user data is loaded */
    ESCO_DEFAULT_PACKET             /* no eSCO user data. use default tx pkt */
}ESCO_FIFO_LOAD_STATUS;

/* This data structure is for each node in the e-SCO scheduler;s queue */    
typedef struct esco_scheduler_node
{
    ESCO_WINDOW_STATUS active;     /* Indicates if the Current node is active */
    UINT16 esco_ce_index;          /* The ce_index of the ESCO CE */
    UCHAR ack_received;            /* Status if ACK is received */
    UCHAR packet_received;                                               
    UINT32 esco_instant;           /* The instant at which this e-SCO node
                                    * will be scheduled 
                                    */
}ESCO_SCHEDULER_NODE;    

/* This Structure defines the ESCO scheduler */    
typedef struct esco_scheduler
{    
    ESCO_SCHEDULER_NODE esco_scheduler_queue;
}ESCO_SCHEDULER;


void lc_init_esco_scheduler(void);
UINT32 lc_esco_get_largest_instant(void);
UCHAR lc_esco_get_least_instant(void);
UINT32 lc_get_clock_diff(UINT32 exp_clk, UINT32 cur_clk, UINT8 lsb_num);

#ifdef _CCH_SC_ECDH_P256_MIC_FLAG
void lc_handle_esco_rx_interrupt(UCHAR esco_lt_addr, UCHAR piconet_id,
                                 UINT16 packet_header, UINT16 payload_header);
#else
void lc_handle_esco_rx_interrupt(UCHAR esco_lt_addr, UCHAR piconet_id, 
                                UINT16 packet_header);
#endif
void lc_handle_esco_rx_packet(UINT16 packet_length, UINT16 status);
void lc_return_to_current_esco(void);
void lc_handle_esco_instant_interrupt_new(UCHAR esco_lt_addr, UINT16 ce_index);
void lc_handle_esco_window_expiry_interrupt_new(UCHAR esco_lt_addr,
                                            UCHAR piconet_id);
void lc_handle_esco_free_pkts_callback_new(UCHAR esco_ce_index);
void lc_load_esco_fifo_new(UCHAR esco_ce_index, UINT8 lag);
void lc_add_to_esco_scheduler_new(UCHAR esco_ce_index);
UINT8 lc_esco_scheduler_new(UCHAR esco_ce_index);


#endif /* COMPILE_ESCO */

#ifdef COMPILE_AFH_HOP_KERNEL

void lc_handle_mss_with_afh(UINT16 ce_index);

#define LC_HANDLE_MSS_WITH_AFH          lc_handle_mss_with_afh

#else
#define LC_HANDLE_HEC_ERR(tx_pkt_type)

#define LC_HANDLE_MSS_WITH_AFH(ce_index)
#endif

#endif /* #ifndef _LC_1_2_INTERNAL_H_*/

/** @} end: lc_internal */
