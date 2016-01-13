/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

#ifndef _BZ_DEBUG_H_
#define _BZ_DEBUG_H_
/** 
 * \file bz_debug.h
 *  BlueWiz Debug utlities Interface.
 * 
 * \author Santhosh kumar M
 * \date 2006-10-08
 */

/**
 * \addtogroup bz_debug BlueWiz Debug Utilities
 * @{ */

#include "platform.h"

#define BZ_ASSERT(cond, msg)        /* null_macro */
#define BZ_ASSERT_NOL(cond)         /* null_macro */
#define SET_LA_DEBUG_PROBE()        /* null_macro */
#define RESET_LA_DEBUG_PROBE()      /* null_macro */
#define TRIGGER_LA_DEBUG_PROBE()    /* null_macro */
#define X_BZ_ASSERT(cond, msg)      /* null macro - for explorative testing */
#define BZ_STATIC                   static

/** @} end: bz_debug */

#endif /* _BZ_DEBUG_H_ */

