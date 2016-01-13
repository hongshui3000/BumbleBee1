/************************************************************************************************************
*               Copyright(c) 2014-2015, Realtek Semiconductor Corporation. All rights reserved.
************************************************************************************************************
* @file     rtl_types.h
* @brief    Basic types definition.
* @author   lory_xu
* @date     2014-05
* @version  v0.1
*************************************************************************************************************
*/

#ifndef __RTL_TYPES_H__
#define __RTL_TYPES_H__

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stddef.h>

#include "compiler_abstraction.h"

/** @addtogroup Platform
  * @{
  */

/** @addtogroup Platform_Basic Basic Definition
  * @{
  */

/** @addtogroup Platform_Basic_Data_Type Data Type
  * @brief Basic Data Type Definition
  * @{
  */

#ifndef CONST
#define CONST const
#endif

#ifndef EXTERN
#define EXTERN extern
#endif

#ifndef INLINE
#define INLINE  inline
#endif

#ifndef STATIC
#define STATIC static
#endif

#ifndef NULL
#define NULL    ((void*)0)
#endif

#ifndef FALSE
#define FALSE   0
#endif

#ifndef TRUE
#define TRUE    1
#endif

#ifndef IN
#define IN
#endif

#ifndef OUT
#define OUT
#endif

#ifndef INOUT
#define INOUT
#endif

typedef unsigned char                   BOOL;
typedef unsigned char                   BOOLEAN;

typedef void                            VOID, *PVOID, ** PPVOID;

typedef unsigned char                   UCHAR;
typedef unsigned char                   UINT8;
typedef unsigned short                  UINT16;
typedef unsigned int                    UINT32;
typedef unsigned long long              UINT64;

typedef signed char                     CHAR;
typedef signed char                     BYTE, INT8;
typedef signed short                    INT16;
typedef signed int                      INT32;
typedef signed long long                INT64;


typedef  const uint8_t*                 LPCBYTE;
typedef  unsigned short*                LPWORD;




typedef        unsigned char    UCHAR, uint8_t, * PUCHAR, * PBYTE;
typedef  const uint8_t             * PCBYTE;
typedef        unsigned char     * LPBYTE;
typedef  const uint8_t              * LPCBYTE;

typedef        char              * LPSTR;
typedef        unsigned int     UINT, * PUINT;
typedef        long             LONG, * PLONG;
typedef        long              * LPLONG;
typedef        unsigned long    ULONG, DWORD, * PULONG, * PDWORD;
typedef        unsigned long     * LPDWORD;
typedef  const unsigned long  * PCUINT32;
typedef        unsigned short   WORD, USHORT, * PUSHORT, * PWORD;
typedef  const void * PCVOID;
typedef  void  * LPVOID;
typedef  const void   * LPCVOID;


typedef  const char        * LPCSTR;

typedef  unsigned int           HANDLE;



/**
 *  general handle definition
 *  size of THandle is fixed to 4 bytes
 */
typedef union _handle
{
	uint8_t   bHandle;
	uint16_t   wHandle;
	uint32_t  dwHandle;
	PVOID  pHandle;
	PVOID lpHandle;
} THandle, *PHandle;
typedef THandle  *LPHandle;


/** @brief  RAM Type Definition */
typedef enum tagRAM_TYPE {
    RAM_TYPE_DATA_OFF       = 0,
    RAM_TYPE_DATA_ON        = 1,
    RAM_TYPE_BUFFER_OFF     = 2,
    RAM_TYPE_BUFFER_ON      = 3,

    RAM_TYPE_NUM            = 4
} RAM_TYPE;


#ifndef max
#define max(a,b)            (((a) > (b)) ? (a) : (b))
#endif

#ifndef min
#define min(a,b)            (((a) < (b)) ? (a) : (b))
#endif

/** Get local uint16_t from external 2 uint8_t, Big-Endian format STANDARD NETWORK uint8_t ORDER */
#define BE_EXTRN2WORD(p) ((*((p)+1)) & 0xff) + ((*(p)) << 8)

/** Get local uint32_t from external 4 uint8_t, Big-Endian format STANDARD NETWORK uint8_t ORDER */
#define BE_EXTRN2DWORD(p) ((uint32_t)(*((p)+3)) & 0xff) + ((uint32_t)(*((p)+2)) << 8) \
                      + ((uint32_t)(*((p)+1)) << 16)  + ((uint32_t)(*((p)+0)) << 24)

/** PUT external 2 CHARS from local int16_t, Big-Endian format STANDARD NETWORK uint8_t ORDER */
#define BE_WORD2EXTRN(p,w)                 \
   {*((p)+1) = (uint8_t)((w) & 0xff);         \
    *(p)     = /*lint -e(572,778)*/ (uint8_t)(((w)>>8) & 0xff);}

/** PUT external 4 uint8_t from local uint32_t, Big-Endian format STANDARD NETWORK uint8_t ORDER */
#define BE_DWORD2EXTRN(p,w)                   \
   {*((p)+3) = (uint8_t)((w) & 0xff);          \
    *((p)+2) = /*lint -e(572,778)*/ (uint8_t)(((w)>>8) & 0xff);     \
    *((p)+1) = /*lint -e(572,778)*/ (uint8_t)(((w)>>16) & 0xff);    \
    *((p)+0) = /*lint -e(572,778)*/ (uint8_t)(((w)>>24) & 0xff);}

/** Get local uint16_t from external 2 uint8_t, Little-Endian format */
#define LE_EXTRN2WORD(p) (((*(p)) & 0xff) + ((*((p)+1)) << 8))

/** Get local uint32_t from external 4 uint8_t, Little-Endian format  */
#define LE_EXTRN2DWORD(p) (((uint32_t)(*((p)+0)) & 0xff) + ((uint32_t)(*((p)+1)) << 8) \
                   + ((uint32_t)(*((p)+2)) << 16)  + ((uint32_t)(*((p)+3)) << 24))

/** PUT external 2 uint8_t from local uint16_t, Little-Endian Format */
#define LE_WORD2EXTRN(p,w)                 \
   {*((uint8_t *)p)     = (uint8_t)((uint16_t)(w) & 0xff);      \
    *(((uint8_t *)p)+1) = /*lint -e(572,778)*/ (uint8_t)(((uint16_t)(w)>>8) & 0xff);}

/** PUT external 4 uint8_t from local uint32_t, Little endian Format */
#define LE_DWORD2EXTRN(p,w)                      \
   {*((uint8_t *)p)     = (uint8_t)((uint32_t)(w) & 0xff);          \
    *(((uint8_t *)p)+1) = (uint8_t)(((uint32_t)(w)>>8) & 0xff);     \
    *(((uint8_t *)p)+2) = (uint8_t)(((uint32_t)(w)>>16) & 0xff);    \
    *(((uint8_t *)p)+3) = (uint8_t)(((uint32_t)(w)>>24) & 0xff);}

#define DebuggerBreak(void)
#define assert( _expr )
#define UNUSED_PARAMETER(para)



/**
 * @brief Function type for patch.
 *
 * Parameters can be any number of arbitrary types.
 *
 * Must add PATCH_POINTER_SECTION when define patch pointer
 */
typedef void (*VoidPatchFun) ();
typedef uint32_t (*U32PatchFun) ();
typedef uint16_t (*U16PatchFun) ();
typedef uint8_t  (*U8PatchFun) ();
typedef int32_t (*S32PatchFun) ();
typedef int16_t (*S16PatchFun) ();
typedef int8_t  (*S8PatchFun) ();
typedef bool (*BOOLPatchFun) ();
typedef uint8_t*(*PU8PatchFun) ();
typedef void*(*PVOIDPatchFun) ();

/** End of Platform_Basic_Data_Type
  * @}
  */

/** End of Platform_Basic
  * @}
  */

/** End of Platform
  * @}
  */

#endif /* __RTL_TYPES_H__ */

