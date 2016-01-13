/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

#ifndef _COMPILER_H_
#define _COMPILER_H_

/**
 * \file compiler.h
 *  Defines the compiler abstraction. It defines an interface to enable the
 *  firmware to uniformly access the non-standard(C89) compiler features. It
 *  may not be possible to abstract all the non-standard features (like __irq
 *  or __attribute__((interrupt("IRQ")))) required by the firmware, but this
 *  abstraction tries to capture the possible interface.
 *
 * \author Santhosh kumar M
 * \date 2007-12-28
 */

#define ALIGN_UINT16     /* null_macro */
#define ALIGN_UINT32     /* null_macro */
#define ALIGN(size)      __attribute__((aligned(size)))
#define INLINE inline    /* null_macro */


#define BUILD_BUG_ON_ZERO(e)  (sizeof(struct { int _; int:-!!(e); }) - sizeof(struct { int _; }))
#define SAME_TYPE(a, b)  __builtin_types_compatible_p(typeof(a), typeof(b))
#define MUST_BE_ARRAY(a)  BUILD_BUG_ON_ZERO(SAME_TYPE((a), &(*a)))

#define ARRAY_SIZE(a) ((sizeof(a) / sizeof(*a)) + MUST_BE_ARRAY(a))

#define ASSERT_CONCAT_(a, b) a##b
#define ASSERT_CONCAT(a, b) ASSERT_CONCAT_(a, b)
#define STATIC_ASSERT(e, m) \
    enum { ASSERT_CONCAT(static_assert_, __COUNTER__) = 1/(!!(e)) }


#endif /* _COMPILER_H_ */

