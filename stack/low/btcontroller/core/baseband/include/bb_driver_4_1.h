/***************************************************************************
 Copyright (C) MindTree Consulting Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  Baseband driver interface.
 */

/** \addtogroup bb_driver Baseband Driver Module
 *  @{ */
#ifndef __BB_4_1_H__
#define __BB_4_1_H__

#ifdef _SUPPORT_VER_4_1_
void  BB_set_piconet_adjust(UCHAR piconet_id, UINT32 clk_slot, UINT16 clk_cnt, UINT32 instant);

void BB_prepare_clk_adj_broadcast(UCHAR piconet_id);

#endif /** @} end: bb_driver */
#endif
