/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file	   pool.h
* @brief	   Pool definitions (Buffer count)
* @details   
*
* @author		gordon
* @date		 	2015-06-29
* @version  		v0.1
*/

#if !defined(__POOL_H)
#define      __POOL_H

#include <flags.h>

#if !defined(__BASETYPE_H)
#include <rtl_types.h>
#endif


/** workspace preceeding/following protocol stack buffers: */

#define POOL_INTERNAL_WORKSPACE_LENGTH    0x40

/** Global Pool */


#define GLOBAL_SHORT_BUFFER_LENGTH    0x80

#define GLOBAL_SHORT_BUFFER_COUNT     8

#define GLOBAL_MIDDLE_BUFFER_LENGTH   0x100
#define GLOBAL_MIDDLE_BUFFER_COUNT    4

#define GLOBAL_LONG_BUFFER_LENGTH     0x800
#define GLOBAL_LONG_BUFFER_COUNT      2



/** D-channel Pool (not X.31/D and D3 transp.) */

/** first the length defines */

#define DC_SHORT_BUFFER_LENGTH        0x10

#define DC_LONG_BUFFER_LENGTH         (0x100+POOL_INTERNAL_WORKSPACE_LENGTH)

#define DC_EXTENDED_BUFFER_LENGTH     0x7e8 /**< (2024)*/

#if defined (_NDIS_)||defined(F_OSWIN)
#define DC_CONTROL_BUFFER_LENGTH      0x10 + 32
#else
#define DC_CONTROL_BUFFER_LENGTH      0x10
#endif

/** and now target dependend COUNTs */

/** default branch */
#define DC_SHORT_BUFFER_COUNT         8


#define DC_EXT_UPSTREAM_BUFFER_COUNT   0
#define DC_EXT_DOWNSTREAM_BUFFER_COUNT 0

#define DC_CONTROL_BUFFER_COUNT       16


/** X.31/D-channel, D3 transp. pool */
/** common defines */
#define DC_X31_CONTROL_BUFFER_LENGTH       16
#define DC_X31_CONTROL_BUFFER_COUNT        16
#define DC_X31_EXTENDED_BUFFER_LENGTH      0
#define DC_X31_EXT_UPSTREAM_BUFFER_COUNT   0
#define DC_X31_EXT_DOWNSTREAM_BUFFER_COUNT 0


/** default branch */
#define DC_X31_SHORT_BUFFER_LENGTH         16
#define DC_X31_SHORT_BUFFER_COUNT          8

#define DC_X31_UPSTREAM_BUFFER_COUNT       16
#define DC_X31_DOWNSTREAM_BUFFER_COUNT     8
#define DC_X31_LONG_BUFFER_LENGTH          0x200+POOL_INTERNAL_WORKSPACE_LENGTH



/** B-channel Pool */
/** default branch */
#define BC_SHORT_BUFFER_LENGTH        0x40
#define BC_SHORT_BUFFER_COUNT         16
#define BC_LONG_BUFFER_LENGTH         (0x800+POOL_INTERNAL_WORKSPACE_LENGTH)
#define BC_UPSTREAM_BUFFER_COUNT      12
#define BC_DOWNSTREAM_BUFFER_COUNT     8
#define BC_CONTROL_BUFFER_LENGTH      0x10
#define BC_CONTROL_BUFFER_COUNT       16
#define BC_EXTENDED_BUFFER_LENGTH      0
#define BC_EXT_UPSTREAM_BUFFER_COUNT   0
#define BC_EXT_DOWNSTREAM_BUFFER_COUNT 0


/** defaults for additional channel */

/** BlueTooth */

#if ((defined(BT_SYS_SHORT_BUFFER_COUNT) && defined(BT_SYS_MIDDLE_BUFFER_COUNT) && defined(BT_SYS_LONG_BUFFER_COUNT))  && \
     ((BT_SYS_SHORT_BUFFER_COUNT  > 0) || (BT_SYS_MIDDLE_BUFFER_COUNT > 0) || (BT_SYS_LONG_BUFFER_COUNT   > 0)))

#define BT_SYS_SHORT_BUFFER_SIZE        BT_SYS_SHORT_BUFFER_BYTE_COUNT
#define BT_SYS_MIDDLE_BUFFER_SIZE       BT_SYS_MIDDLE_BUFFER_BYTE_COUNT
#define BT_SYS_LONG_BUFFER_SIZE         BT_SYS_LONG_BUFFER_BYTE_COUNT

#else

#undef BT_SYS_SHORT_BUFFER_COUNT
#undef BT_SYS_MIDDLE_BUFFER_COUNT
#undef BT_SYS_LONG_BUFFER_COUNT


#define BT_SYS_SHORT_BUFFER_COUNT      64
#define BT_SYS_SHORT_BUFFER_SIZE     0x48
#define BT_SYS_MIDDLE_BUFFER_COUNT     14
#define BT_SYS_MIDDLE_BUFFER_SIZE   0x120
#define BT_SYS_LONG_BUFFER_COUNT        4
#define BT_SYS_LONG_BUFFER_SIZE     0x200


#endif

#endif  /**< #if !defined(__POOL_H) */


