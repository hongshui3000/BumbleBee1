
/***************************************************************************
 Copyright (C) MindTree Consulting Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  Structures and Macro definitions specific to BT3.0 LMP layer. It also
 *  contains the function interface.
 */

/** \addtogroup lmp_internal
 *  @{ */
#ifndef __LMP_4_1_H__
#define __LMP_4_1_H__

#ifdef _SUPPORT_VER_4_1_
UCHAR lmp_handle_4_1_incoming_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index);



/* Functions */
#ifdef _SUPPORT_PCA_ADJUST

void lmp_handle_lmp_clk_adj_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index);
void lmp_handle_lmp_clk_adj_ack_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index);
void lmp_handle_lmp_clk_adj_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index);
        
API_RESULT lmp_generate_clk_adj_pdu(UINT32 instant, 
           UINT8 clk_adj_slots, INT16 clk_adj_us, UINT8 clk_adj_mode);
void lmp_generate_clk_adj_ack_pdu(UINT16 ce_index, UINT16 clk_adj_id);
void lmp_generate_clk_adj_req_pdu(UINT16 ce_index, INT16 clk_adj_us,
    UINT8 clk_adj_slots, UINT8 clk_adj_period);
UINT8 lmp_validate_clk_adj_param(INT16 clk_adj_us, UINT16 clk_adj_slots, UINT8 self_role);
void lmp_cleanup_param_after_clk_adj_procedure(void);

/*==========================================================*/
/*          SW Structure for Piconet_Adjust Unit            */
/*==========================================================*/
typedef struct BT_PCA_MANAGER_S_ {    
    UINT32 clk_offset_for_update;   /* the clk offset between current & instant. */
    UINT32 instant;                 /* this is the instant that controller will do clk_adj (clk[27:0])*/
    UINT16 slv_pca_support_bm;      /* the bit-map of the status of all slaves that support PCA.
                                       check it when we send lmp_clk_adj pdu. once it support the
                                       recording bit in bm = 1 and set that bit back to 0 when
                                       the lmp_clk_ack is received from that slave. */
    INT16 clk_adj_us;               /* the us of the updation of this time. */
                                       
    UINT8 clk_adj_id;               /* the adj id that is updating. */
    UINT8 pca_updating:1;           /* is piconet clk updating in procedure...? */
    UINT8 pca_clk_req_sent:1;       /* has pca master sent clk_adj before? */
    UINT8 pca_clk_req_nbc_to:1;
    UINT8 rsvd:5;
    UINT8 tpoll_cnt_after_clk_req;  /* this is the count tx_interrupt happened after we send LMP_clk_Adj pdu. */
    UINT8 clk_req_retry_th;         /* this is the count that we resend LMP_clk_Adj pdu*/                                       
    UINT8 clk_adj_slots;            /* the slot of the updation of this time. */

    /******************************/
    /********* SLAVE PART *********/
    /******************************/
    UINT8 clk_adj_ce_index;         /* the ce_index that is doing clk_update. If this ce_index disconn, 
                                       then we should set clk_adj_id to default. */
                                       
} BT_PCA_MANAGER_S;
extern BT_PCA_MANAGER_S bt_pca_manager;
#endif


#endif /*_SUPPORT_VER_4_1_*/

#endif
/** @} end: lmp_internal */


