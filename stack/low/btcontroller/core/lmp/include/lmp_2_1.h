
/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  Structures and Macro definitions specific to BT1.2 LMP layer. It also
 *  contains the function interface.
 */

/** \addtogroup lmp_internal
 *  @{ */
#ifndef LMP_2_1_H_
#define LMP_2_1_H_

#include "lmp_spec_defines.h"
#include "bt_fw_types.h"
#include "lmp.h"

/**
 * This value is multiplied to the sniff_interval (tsniff)
 * and checked for min_timeout value.
 * See lc_isr.c (check_enter_ssr)
 */
#define LMP_MIN_SSR_TSNIFF_MUL                      2

/**
 * This value is multiplied to the sniff_interval (tsniff)
 * for calculating SSR_Instant.
 */
#define LMP_SSR_INSTANT_MUL                         2

/**
 * Minimum number of sniff attempts - if this condition
 * is not satisfied ssr_instant is multiplied by 2.
 */
#define LMP_SSR_INSTANT_MIN_ATTEMPT                 4

/**
 * After ssr_instant calculations - it is checked for 
 * this minimum value.
 */
#define LMP_MIN_SSR_INSTANT_OFFSET                  0x500

/* Functions */
UCHAR lmp_handle_2_1_incoming_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index);
/* Helper functions */
UCHAR lmp_initiate_sniff_subrating(UINT16 ce_index);
void lmp_generate_sniff_subrating_res_pdu(UINT16 ce_index);
void lmp_generate_sniff_subrating_req_pdu(UINT16 ce_index);
void lmp_reset_ssr_parameters(UINT16 ce_index);
void lmp_ssr_disable(UINT16 ce_index);

#endif /*LMP_2_1_H_*/
/** @} end: lmp_internal */
