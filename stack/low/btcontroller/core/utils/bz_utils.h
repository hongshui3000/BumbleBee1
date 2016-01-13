/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/** 
 * \file bz_utils.h
 *  Generic BlueWiz utilities interface.
 * 
 * \author Santhosh kumar M
 * \date 2006-05-06
 */

/** 
 * \addtogroup bz_utils Bluewiz Utilities
 * @{ */

/** 
 * \addtogroup bz_generic_utils Generic Utilities
 * @{ */

#ifndef __BZ_UTILS_H__
#define __BZ_UTILS_H__

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */

/* --------------------------------- Function Prototypes ------------------ */
void bz_spin_for_nclk0_ticks(int nticks);
#ifdef _FIX_CROSSBAR_ERROR_DEBUG_
void print_timer_info();
#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __BZ_UTILS_H__ */

/**  @} end: bz_generic_utils */
/**  @} end: bz_utils */

