/**
************************************************************************************************************
*               Copyright(c) 2014-2015, Realtek Semiconductor Corporation. All rights reserved.
************************************************************************************************************
* @file     compiler_abstraction.h
* @brief    Compiler specific intrinsics
* @author   lory_xu
* @date     2015-03
* @version  v0.1
*************************************************************************************************************
*/

#ifndef __COMPILER_ABSTRACTION_H__
#define __COMPILER_ABSTRACTION_H__

/** @addtogroup Platform
  * @brief Platform Manual
  * @{
  */

/** @addtogroup Platform_Toolchain Toolchain
  * @{
  */

/** @addtogroup Platform_Toolchain_Compiler Compiler Support
  * @{
  */

/** @brief Define compilor specific symbol */

#if defined ( __CC_ARM )
#define __ASM                   __asm                                      /*!< asm keyword for ARM Compiler          */
#define __INLINE                __inline                                   /*!< inline keyword for ARM Compiler       */
#define __STATIC_INLINE         static __inline
#define _PACKED_                __attribute__ ((packed))
#define PACKED(structOrUnion)   structOrUnion __attribute__ ((packed))
#define SECTION(_name)          __attribute__ ((__section__(_name)))
#define SECTION_ZI(_name)       __attribute__ ((section(_name),zero_init))
#define barrier()               __memory_changed()
#elif defined ( __ICCARM__ )
#define __ASM                   __asm                                      /*!< asm keyword for IAR Compiler          */
#define __INLINE                inline                                     /*!< inline keyword for IAR Compiler. Only available in High optimization mode! */
#define __STATIC_INLINE         static inline
#define _PACKED_                __packed
#define PACKED(structOrUnion)   __packed structOrUnion
#define SECTION(_name)          @_name
#define SECTION_ZI(_name)       @_name
#elif defined ( __GNUC__ )
#define __ASM                   __asm                                      /*!< asm keyword for GNU Compiler          */
#define __INLINE                inline                                     /*!< inline keyword for GNU Compiler       */
#define __STATIC_INLINE         static inline
#define _PACKED_                __attribute__ ((packed))
#define  PACKED(structOrUnion)  structOrUnion __attribute__ ((packed))
#define SECTION(_name)          __attribute__ ((__section__(_name)))
#define SECTION_ZI(_name)       __attribute__ ((section(_name),zero_init))
#define barrier()               __asm volatile ("": : :"memory")
#endif


#if defined(__GNUC__) && 0
#define VA_START(v,l) __builtin_va_start(v,l)
#define VA_ARG(v,l)   __builtin_va_arg(v,l)
#define VA_END(v)     __builtin_va_end(v)
#define VA_LIST       __builtin_va_list
#else /* defined(__GNUC__) */
#ifndef _STDARG_H
#include <stdarg.h>
#endif
#define VA_START va_start
#define VA_ARG   va_arg
#define VA_END   va_end
#define VA_LIST  va_list
#endif /* defined(__GNUC__) */


/** End of Platform_Toolchain_Compiler
  * @}
  */

/** End of Platform_Toolchain
  * @}
  */

/** End of Platform
  * @}
  */

#endif /* __COMPILER_ABSTRACTION_H__ */
