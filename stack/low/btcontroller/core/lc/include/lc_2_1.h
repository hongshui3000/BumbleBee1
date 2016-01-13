/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  Contains the BT2.1 LC module external/internal API.
 */

/** \addtogroup lc_external
 *   @{ */
#ifndef LC_2_1_H_
#define LC_2_1_H_

void lc_exit_ssr_mode(UINT16 ce_index);
void lc_enter_ssr_mode(UINT16 ce_index);
void lc_write_bc_pwr_level(UCHAR val);

#endif /*LC_2_1_H_*/

/** @} end: lc_internal */
