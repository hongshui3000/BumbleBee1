
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
#ifdef VER_3_0

#ifndef __LMP_3_0_H__
#define __LMP_3_0_H__

/* Feature bit */
#define LMP_EPC_FEATURE                     0x04

#define LMP_EPC_DB_STEP_SIZE                2
#define LMP_EPC_INITIAL_TX_DB               0

/** Maximum power increase or decrease */
#define LMP_MAX_POWER_CHANGE                15


#define EPC_POWER_MOD_NOT_SUPPORTED         0
#define EPC_POWER_SGL_STEP_INC_DEC          1
#define EPC_MAX_POWER_REACHED               2
#define EPC_MIN_POWER_REACHED               3


/**
 * LMP Power incr/decr/incr by 15db parameter values.
 */
#define LMP_PWR_CTRL_DEC_ONE_STEP           0
#define LMP_PWR_CTRL_INC_ONE_STEP           1
#define LMP_PWR_CTRL_INC_UPTO_MAXIMUM          2



#define LMP_PWR_ALL_MOD_MAX                 0x2A    /* 101010b */
#define LMP_PWR_ALL_MOD_MIN                 0x3F    /* 111111b */

/** This enum will have as many entries as number of 
 *  modulation schemes, and will serve to give their order 
 *  in epc_power_table.modulation_power_data[]. 
 */
enum modulation_scheme
{
//#ifdef GSFK_SUPPORTED
    GSFK,
//#endif
//#ifdef DQPSK_SUPPORTED
    DQPSK,
//#endif
//#ifdef EIGHT_DPSK_SUPPORTED
    EIGHT_DPSK
//#endif
};

/* Functions */
UCHAR lmp_handle_3_0_incoming_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index);
void lmp_init_epc(void);

/* Helper functions */
API_RESULT lmp_generate_power_ctrl_req_pdu(UINT16 ce_index, 
                                           UCHAR power_adjustment);
void lmp_generate_power_ctrl_res_pdu(UINT16 ce_index, UCHAR response_parameter);
CHAR lmp_epc_get_cur_pwr_level(UINT16 ce_index);
CHAR lmp_epc_get_max_pwr_level(void);
void lmp_init_epc_connection_entity(UINT16 ce_index);

#endif /*__LMP_3_0_H__*/

#endif
/** @} end: lmp_internal */


