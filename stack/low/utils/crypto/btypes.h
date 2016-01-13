/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/** 
 * \file btypes.h
 *  Basic types definition. This file should be modified based on the target
 *  platform.
 * 
 * \author Santhosh kumar M
 * \date 2007-08-20
 */

#ifndef _BTYPES_H_
#define _BTYPES_H_

/* ========================= Include File Section ========================= */


/* ====================== Macro Declaration Section ======================= */
#ifndef IN
#define IN              /*@in@*/
#endif
#ifndef OUT
#define OUT             /*@out@*/
#endif
#ifndef INOUT
#define INOUT           IN OUT
#endif

/* ==================== Data Types Declaration Section ==================== */
typedef unsigned char u8;
typedef unsigned short int u16;
typedef unsigned int u32;
typedef unsigned long long u64;
typedef signed char s8;
typedef signed short int s16;
typedef signed int s32;
typedef signed long long s64;

/* ================ Exported Variables Declaration Section ================ */


/* ============================= API Section ============================== */



#endif /* _BTYPES_H_ */

