/**********************************************************************!KA****
 *
 * $Header: /var/lib/cvs/sw/inc/trace_binary_mod.h,v 1.1 2013/11/21 14:47:24 mn Exp $
 *
 * File:        $RCSfile: trace_binary_mod.h,v $
 * Version:     $Name: P_BLB1290_V1_0 $
 *
 * Archive:     $Source: /var/lib/cvs/sw/inc/trace_binary_mod.h,v $
 * Revision:    $Revision: 1.1 $
 * Date:        $Date: 2013/11/21 14:47:24 $
 * Author:      $Author: mn $
 *
 * ---------------------------------------------------------------------------
 * !MODULE      [  ]
 * ---------------------------------------------------------------------------
 * !FILE        [  ]
 * !PROGRAM     [  ]
 * !VERSION     [$Name: P_BLB1290_V1_0 $]
 * !GROUP       [  ]
 * !AUTHOR      [$Author: mn $]
 * ---------------------------------------------------------------------------
 *
 *          Copyright (c)           2013 Stollmann E+V GmbH
 *                                  Mendelssohnstr. 15
 *                                  22761 Hamburg
 *                                  Phone: 040/89088-0
 *          The content of this file is Stollmann E+V GmbH confidential
 *          All Rights Reserved
 *
 * ---------------------------------------------------------------------------
 * !DESCRIPTION
 *
 *         Binary Trace
 * ---------------------------------------------------------------------------
 * !INDEX
 *  ...
 * ---------------------------------------------------------------------------
 * !CONTENTS
 * ---------------------------------------------------------------------------
 * !INCLUDE_REFERENCES
 * ---------------------------------------------------------------------------
 * !HISTORY
 *  Date      Author          Comment
 *  tt.mm.jj                  Initial revision
 *  tt.mm.jj
 * ---------------------------------------------------------------------------
 *
 ************************************************************************!KA*/

#if !defined(__TRACE_BINARY_MOD_H)
#define      __TRACE_BINARY_MOD_H

#include <efuse_config.h>
extern OTP_STRUCT otp_str_data;



/** BDADDR */
#define TRACE_BDADDR1                      BTRACE_BDADDR1
#define TRACE_BDADDR2                      BTRACE_BDADDR2

#define HCI_TRACE_MASK_TRACE        0x00000001 
#define HCI_TRACE_MASK_ERROR        0x00000002
#define HCI_TRACE_MASK_MESSAGE      0x00000004 


#define L2CAP_TRACE_MASK_TRACE      0x00000010 
#define L2CAP_TRACE_MASK_ERROR      0x00000020
//#define L2CAP_TRACE_MASK_MESSAGE      0x00000040

#define GATT_TRACE_MASK_TRACE               0x00000100
#define GATT_TRACE_MASK_ERROR               0x00000200
//#define GATT_TRACE_MASK_MESSAGE               0x00000400

#define BTSECMAN_TRACE_MASK_TRACE      0x00001000
#define BTSECMAN_TRACE_MASK_ERROR      0x00002000
//#define BTSECMAN_TRACE_MASK_MESSAGE      0x00004000

#define BLUEFACE_TRACE_MASK_TRACE       0x00010000
#define BLUEFACE_TRACE_MASK_ERROR       0x00020000
//#define BLUEFACE_TRACE_MASK_MESSAGE  0x00040000

#define BLUEAPI_TRACE_MASK_TRACE      0x00100000
#define BLUEAPI_TRACE_MASK_ERROR      0x00200000
#define BLUEAPI_TRACE_MASK_MESSAGE 0x00400000

#define APPL_TRACE_MASK_TRACE      0x01000000
#define APPL_TRACE_MASK_ERROR      0x02000000
#define PATCH_TRACE_MASK_TRACE     0x04000000
#define PATCH_TRACE_MASK_ERROR     0x08000000

#define RFCOMM_TRACE_MASK_TRACE      0x10000000
#define RFCOMM_TRACE_MASK_ERROR      0x20000000
#define SDP_TRACE_MASK_TRACE         0x40000000
#define SDP_TRACE_MASK_ERROR         0x80000000

#if 0
#define PATCH_TRACE_BINARY_UPSTREAM     if (PATCH_TRACE_MASK_MESSAGE& otp_str_data.gEfuse_UpperStack_s.traceMask)   \
        BTRACE_BINARY_UPSTREAM
#define PATCH_TRACE_BINARY_DOWNSTREAM    if (PATCH_TRACE_MASK_MESSAGE& otp_str_data.gEfuse_UpperStack_s.traceMask)   \
        BTRACE_BINARY_UPSTREAM

#define APPL_TRACE_BINARY_UPSTREAM      if (APPL_TRACE_MASK_MESSAGE& otp_str_data.gEfuse_UpperStack_s.traceMask)   \
        BTRACE_BINARY_UPSTREAM
#define APPL_TRACE_BINARY_DOWNSTREAM    if (APPL_TRACE_MASK_MESSAGE& otp_str_data.gEfuse_UpperStack_s.traceMask)   \
        BTRACE_BINARY_UPSTREAM
#endif
#define BLUEAPI_TRACE_BINARY_UPSTREAM    if (BLUEAPI_TRACE_MASK_MESSAGE& otp_str_data.gEfuse_UpperStack_s.traceMask)   \
        BTRACE_BINARY_UPSTREAM
#define BLUEAPI_TRACE_BINARY_DOWNSTREAM  if (BLUEAPI_TRACE_MASK_MESSAGE & otp_str_data.gEfuse_UpperStack_s.traceMask)   \
        BTRACE_BINARY_DOWNSTREAM

#define BLUEFACE_TRACE_BINARY_UPSTREAM     if (BLUEFACE_TRACE_MASK_MESSAGE & otp_str_data.gEfuse_UpperStack_s.traceMask)\
        BTRACE_BINARY_UPSTREAM
#define BLUEFACE_TRACE_BINARY_DOWNSTREAM   if (BLUEFACE_TRACE_MASK_MESSAGE & otp_str_data.gEfuse_UpperStack_s.traceMask)\
        BTRACE_BINARY_DOWNSTREAM

#define BTSECMAN_TRACE_BINARY_UPSTREAM      if (BTSECMAN_TRACE_MASK_MESSAGE& otp_str_data.gEfuse_UpperStack_s.traceMask)   \
        BTRACE_BINARY_UPSTREAM
#define BTSECMAN_TRACE_BINARY_DOWNSTREAM    if (BTSECMAN_TRACE_MASK_MESSAGE& otp_str_data.gEfuse_UpperStack_s.traceMask)   \
        BTRACE_BINARY_UPSTREAM

#define GATT_TRACE_BINARY_UPSTREAM      if (GATT_TRACE_MASK_MESSAGE& otp_str_data.gEfuse_UpperStack_s.traceMask)   \
        BTRACE_BINARY_UPSTREAM
#define GATT_TRACE_BINARY_DOWNSTREAM    if (GATT_TRACE_MASK_MESSAGE& otp_str_data.gEfuse_UpperStack_s.traceMask)   \
        BTRACE_BINARY_UPSTREAM
        
#define L2CAP_TRACE_BINARY_UPSTREAM       if (L2CAP_TRACE_MASK_MESSAGE& otp_str_data.gEfuse_UpperStack_s.traceMask)   \
        BTRACE_BINARY_UPSTREAM
#define L2CAP_TRACE_BINARY_DOWNSTREAM    if (L2CAP_TRACE_MASK_MESSAGE& otp_str_data.gEfuse_UpperStack_s.traceMask)   \
        BTRACE_BINARY_UPSTREAM

#define HCI_TRACE_BINARY_UPSTREAM     if (HCI_TRACE_MASK_MESSAGE& otp_str_data.gEfuse_UpperStack_s.traceMask)   \
        BTRACE_BINARY_UPSTREAM
#define HCI_TRACE_BINARY_DOWNSTREAM     if (HCI_TRACE_MASK_MESSAGE& otp_str_data.gEfuse_UpperStack_s.traceMask)   \
        BTRACE_BINARY_UPSTREAM



/****************************************************************************/
/* PATCH                                                                    */
/****************************************************************************/

#define PATCH_TRACE_PRINTF_0(level, pFormat)                                                   \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                               \
        BTRACE_PRINTF_0(level, pFormat)
#define PATCH_TRACE_PRINTF_1(level, pFormat, Arg1)                                             \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                               \
        BTRACE_PRINTF_1(level, pFormat, Arg1)
#define PATCH_TRACE_PRINTF_2(level, pFormat, Arg1, Arg2)                                       \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                             \
        BTRACE_PRINTF_2(level, pFormat, Arg1, Arg2)
#define PATCH_TRACE_PRINTF_3(level, pFormat, Arg1, Arg2, Arg3)                                 \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                \
        BTRACE_PRINTF_3(level, pFormat, Arg1, Arg2, Arg3)
#define PATCH_TRACE_PRINTF_4(level, pFormat, Arg1, Arg2, Arg3, Arg4)                           \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                \
        BTRACE_PRINTF_4(level, pFormat, Arg1, Arg2, Arg3, Arg4)
#define PATCH_TRACE_PRINTF_5(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5)                     \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                               \
        BTRACE_PRINTF_5(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5)
#define PATCH_TRACE_PRINTF_6(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6)               \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                               \
        BTRACE_PRINTF_6(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6)
#define PATCH_TRACE_PRINTF_7(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6, Arg7)         \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                               \
        BTRACE_PRINTF_7(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6, Arg7)
#define PATCH_TRACE_PRINTF_8(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6, Arg7, Arg8)   \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                               \
        BTRACE_PRINTF_8(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6, Arg7, Arg8)


/****************************************************************************/
/* APPLICATION                                                                    */
/****************************************************************************/
#define APPL_TRACE_PRINTF_0(level, pFormat)                                                   \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                 \
        BTRACE_PRINTF_0(level, pFormat)
#define APPL_TRACE_PRINTF_1(level, pFormat, Arg1)                                             \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                 \
        BTRACE_PRINTF_1(level, pFormat, Arg1)
#define APPL_TRACE_PRINTF_2(level, pFormat, Arg1, Arg2)                                       \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                 \
        BTRACE_PRINTF_2(level, pFormat, Arg1, Arg2)
#define APPL_TRACE_PRINTF_3(level, pFormat, Arg1, Arg2, Arg3)                                 \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                 \
        BTRACE_PRINTF_3(level, pFormat, Arg1, Arg2, Arg3)
#define APPL_TRACE_PRINTF_4(level, pFormat, Arg1, Arg2, Arg3, Arg4)                           \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                \
        BTRACE_PRINTF_4(level, pFormat, Arg1, Arg2, Arg3, Arg4)
#define APPL_TRACE_PRINTF_5(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5)                     \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                              \
        BTRACE_PRINTF_5(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5)
#define APPL_TRACE_PRINTF_6(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6)               \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                \
        BTRACE_PRINTF_6(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6)
#define APPL_TRACE_PRINTF_7(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6, Arg7)         \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                               \
        BTRACE_PRINTF_7(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6, Arg7)
#define APPL_TRACE_PRINTF_8(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6, Arg7, Arg8)   \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                 \
        BTRACE_PRINTF_8(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6, Arg7, Arg8)

/****************************************************************************/
/* BlueAPI                                                                  */
/****************************************************************************/
#define BLUEAPI_TRACE_PRINTF_0(level, pFormat)                                                   \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                  \
        BTRACE_PRINTF_0(level, pFormat)
#define BLUEAPI_TRACE_PRINTF_1(level, pFormat, Arg1)                                             \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                 \
        BTRACE_PRINTF_1(level, pFormat, Arg1)
#define BLUEAPI_TRACE_PRINTF_2(level, pFormat, Arg1, Arg2)                                       \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                              \
        BTRACE_PRINTF_2(level, pFormat, Arg1, Arg2)
#define BLUEAPI_TRACE_PRINTF_3(level, pFormat, Arg1, Arg2, Arg3)                                 \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                  \
        BTRACE_PRINTF_3(level, pFormat, Arg1, Arg2, Arg3)
#define BLUEAPI_TRACE_PRINTF_4(level, pFormat, Arg1, Arg2, Arg3, Arg4)                           \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                               \
        BTRACE_PRINTF_4(level, pFormat, Arg1, Arg2, Arg3, Arg4)
#define BLUEAPI_TRACE_PRINTF_5(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5)                     \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                               \
        BTRACE_PRINTF_5(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5)
#define BLUEAPI_TRACE_PRINTF_6(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6)               \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                               \
        BTRACE_PRINTF_6(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6)
#define BLUEAPI_TRACE_PRINTF_7(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6, Arg7)         \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                 \
        BTRACE_PRINTF_7(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6, Arg7)
#define BLUEAPI_TRACE_PRINTF_8(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6, Arg7, Arg8)   \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                  \
        BTRACE_PRINTF_8(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6, Arg7, Arg8)

#define BLUEAPI_TRACE_BDADDR1              BTRACE_BDADDR1
#define BLUEAPI_TRACE_BDADDR2              BTRACE_BDADDR2

/****************************************************************************/
/* BlueFace                                                                 */
/****************************************************************************/
#define BLUEFACE_TRACE_PRINTF_0(level, pFormat)                                                   \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                 \
        BTRACE_PRINTF_0(level, pFormat)
#define BLUEFACE_TRACE_PRINTF_1(level, pFormat, Arg1)                                             \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                 \
        BTRACE_PRINTF_1(level, pFormat, Arg1)
#define BLUEFACE_TRACE_PRINTF_2(level, pFormat, Arg1, Arg2)                                       \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                 \
        BTRACE_PRINTF_2(level, pFormat, Arg1, Arg2)
#define BLUEFACE_TRACE_PRINTF_3(level, pFormat, Arg1, Arg2, Arg3)                                 \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                 \
        BTRACE_PRINTF_3(level, pFormat, Arg1, Arg2, Arg3)
#define BLUEFACE_TRACE_PRINTF_4(level, pFormat, Arg1, Arg2, Arg3, Arg4)                           \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                  \
        BTRACE_PRINTF_4(level, pFormat, Arg1, Arg2, Arg3, Arg4)
#define BLUEFACE_TRACE_PRINTF_5(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5)                     \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                  \
        BTRACE_PRINTF_5(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5)
#define BLUEFACE_TRACE_PRINTF_6(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6)               \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                 \
        BTRACE_PRINTF_6(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6)
#define BLUEFACE_TRACE_PRINTF_7(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6, Arg7)         \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                  \
        BTRACE_PRINTF_7(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6, Arg7)
#define BLUEFACE_TRACE_PRINTF_8(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6, Arg7, Arg8)   \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                               \
        BTRACE_PRINTF_8(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6, Arg7, Arg8)


/****************************************************************************/
/* BtSecMan                                                                 */
/****************************************************************************/

#define BTSECMAN_TRACE_PRINTF_0(level, pFormat)                                                   \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                  \
        BTRACE_PRINTF_0(level, pFormat)
#define BTSECMAN_TRACE_PRINTF_1(level, pFormat, Arg1)                                             \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                 \
        BTRACE_PRINTF_1(level, pFormat, Arg1)
#define BTSECMAN_TRACE_PRINTF_2(level, pFormat, Arg1, Arg2)                                       \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                 \
        BTRACE_PRINTF_2(level, pFormat, Arg1, Arg2)
#define BTSECMAN_TRACE_PRINTF_3(level, pFormat, Arg1, Arg2, Arg3)                                 \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                \
        BTRACE_PRINTF_3(level, pFormat, Arg1, Arg2, Arg3)
#define BTSECMAN_TRACE_PRINTF_4(level, pFormat, Arg1, Arg2, Arg3, Arg4)                           \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                               \
        BTRACE_PRINTF_4(level, pFormat, Arg1, Arg2, Arg3, Arg4)
#define BTSECMAN_TRACE_PRINTF_5(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5)                     \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                \
        BTRACE_PRINTF_5(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5)
#define BTSECMAN_TRACE_PRINTF_6(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6)               \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                 \
        BTRACE_PRINTF_6(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6)
#define BTSECMAN_TRACE_PRINTF_7(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6, Arg7)         \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                \
        BTRACE_PRINTF_7(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6, Arg7)
#define BTSECMAN_TRACE_PRINTF_8(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6, Arg7, Arg8)   \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                             \
        BTRACE_PRINTF_8(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6, Arg7, Arg8)


/****************************************************************************/
/* GATT                                                                     */
/****************************************************************************/

#define GATT_TRACE_PRINTF_0(level, pFormat)                                                   \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                 \
        BTRACE_PRINTF_0(level, pFormat)
#define GATT_TRACE_PRINTF_1(level, pFormat, Arg1)                                             \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                               \
        BTRACE_PRINTF_1(level, pFormat, Arg1)
#define GATT_TRACE_PRINTF_2(level, pFormat, Arg1, Arg2)                                       \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                \
        BTRACE_PRINTF_2(level, pFormat, Arg1, Arg2)
#define GATT_TRACE_PRINTF_3(level, pFormat, Arg1, Arg2, Arg3)                                 \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                  \
        BTRACE_PRINTF_3(level, pFormat, Arg1, Arg2, Arg3)
#define GATT_TRACE_PRINTF_4(level, pFormat, Arg1, Arg2, Arg3, Arg4)                           \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                              \
        BTRACE_PRINTF_4(level, pFormat, Arg1, Arg2, Arg3, Arg4)
#define GATT_TRACE_PRINTF_5(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5)                     \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                 \
        BTRACE_PRINTF_5(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5)
#define GATT_TRACE_PRINTF_6(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6)               \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                 \
        BTRACE_PRINTF_6(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6)
#define GATT_TRACE_PRINTF_7(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6, Arg7)         \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                \
        BTRACE_PRINTF_7(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6, Arg7)
#define GATT_TRACE_PRINTF_8(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6, Arg7, Arg8)   \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                  \
        BTRACE_PRINTF_8(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6, Arg7, Arg8)


/****************************************************************************/
/* RFCOMM                                                                    */
/****************************************************************************/

#define RFCOMM_TRACE_PRINTF_0(level, pFormat)                                                   \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                 \
        BTRACE_PRINTF_0(level, pFormat)
#define RFCOMM_TRACE_PRINTF_1(level, pFormat, Arg1)                                             \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                               \
        BTRACE_PRINTF_1(level, pFormat, Arg1)
#define RFCOMM_TRACE_PRINTF_2(level, pFormat, Arg1, Arg2)                                       \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                 \
        BTRACE_PRINTF_2(level, pFormat, Arg1, Arg2)
#define RFCOMM_TRACE_PRINTF_3(level, pFormat, Arg1, Arg2, Arg3)                                 \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                 \
        BTRACE_PRINTF_3(level, pFormat, Arg1, Arg2, Arg3)
#define RFCOMM_TRACE_PRINTF_4(level, pFormat, Arg1, Arg2, Arg3, Arg4)                           \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                 \
        BTRACE_PRINTF_4(level, pFormat, Arg1, Arg2, Arg3, Arg4)
#define RFCOMM_TRACE_PRINTF_5(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5)                     \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                \
        BTRACE_PRINTF_5(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5)
#define RFCOMM_TRACE_PRINTF_6(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6)               \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                 \
        BTRACE_PRINTF_6(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6)
#define RFCOMM_TRACE_PRINTF_7(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6, Arg7)         \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                 \
        BTRACE_PRINTF_7(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6, Arg7)
#define RFCOMM_TRACE_PRINTF_8(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6, Arg7, Arg8)   \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                 \
        BTRACE_PRINTF_8(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6, Arg7, Arg8)


/****************************************************************************/
/* SDP                                                                    */
/****************************************************************************/

#define SDP_TRACE_PRINTF_0(level, pFormat)                                                   \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                 \
        BTRACE_PRINTF_0(level, pFormat)
#define SDP_TRACE_PRINTF_1(level, pFormat, Arg1)                                             \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                               \
        BTRACE_PRINTF_1(level, pFormat, Arg1)
#define SDP_TRACE_PRINTF_2(level, pFormat, Arg1, Arg2)                                       \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                 \
        BTRACE_PRINTF_2(level, pFormat, Arg1, Arg2)
#define SDP_TRACE_PRINTF_3(level, pFormat, Arg1, Arg2, Arg3)                                 \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                 \
        BTRACE_PRINTF_3(level, pFormat, Arg1, Arg2, Arg3)
#define SDP_TRACE_PRINTF_4(level, pFormat, Arg1, Arg2, Arg3, Arg4)                           \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                 \
        BTRACE_PRINTF_4(level, pFormat, Arg1, Arg2, Arg3, Arg4)
#define SDP_TRACE_PRINTF_5(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5)                     \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                \
        BTRACE_PRINTF_5(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5)
#define SDP_TRACE_PRINTF_6(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6)               \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                 \
        BTRACE_PRINTF_6(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6)
#define SDP_TRACE_PRINTF_7(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6, Arg7)         \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                 \
        BTRACE_PRINTF_7(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6, Arg7)
#define SDP_TRACE_PRINTF_8(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6, Arg7, Arg8)   \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                 \
        BTRACE_PRINTF_8(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6, Arg7, Arg8)

/****************************************************************************/
/* L2CAP                                                                    */
/****************************************************************************/

#define L2CAP_TRACE_PRINTF_0(level, pFormat)                                                   \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                 \
        BTRACE_PRINTF_0(level, pFormat)
#define L2CAP_TRACE_PRINTF_1(level, pFormat, Arg1)                                             \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                               \
        BTRACE_PRINTF_1(level, pFormat, Arg1)
#define L2CAP_TRACE_PRINTF_2(level, pFormat, Arg1, Arg2)                                       \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                 \
        BTRACE_PRINTF_2(level, pFormat, Arg1, Arg2)
#define L2CAP_TRACE_PRINTF_3(level, pFormat, Arg1, Arg2, Arg3)                                 \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                 \
        BTRACE_PRINTF_3(level, pFormat, Arg1, Arg2, Arg3)
#define L2CAP_TRACE_PRINTF_4(level, pFormat, Arg1, Arg2, Arg3, Arg4)                           \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                 \
        BTRACE_PRINTF_4(level, pFormat, Arg1, Arg2, Arg3, Arg4)
#define L2CAP_TRACE_PRINTF_5(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5)                     \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                \
        BTRACE_PRINTF_5(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5)
#define L2CAP_TRACE_PRINTF_6(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6)               \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                 \
        BTRACE_PRINTF_6(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6)
#define L2CAP_TRACE_PRINTF_7(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6, Arg7)         \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                 \
        BTRACE_PRINTF_7(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6, Arg7)
#define L2CAP_TRACE_PRINTF_8(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6, Arg7, Arg8)   \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                 \
        BTRACE_PRINTF_8(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6, Arg7, Arg8)

/****************************************************************************/
/* HCI                                                                      */
/****************************************************************************/



#define HCI_TRACE_PRINTF_0(level, pFormat)                                                   \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                \
        BTRACE_PRINTF_0(level, pFormat)
#define HCI_TRACE_PRINTF_1(level, pFormat, Arg1)                                             \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                 \
        BTRACE_PRINTF_1(level, pFormat, Arg1)
#define HCI_TRACE_PRINTF_2(level, pFormat, Arg1, Arg2)                                       \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                              \
        BTRACE_PRINTF_2(level, pFormat, Arg1, Arg2)
#define HCI_TRACE_PRINTF_3(level, pFormat, Arg1, Arg2, Arg3)                                 \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                \
        BTRACE_PRINTF_3(level, pFormat, Arg1, Arg2, Arg3)
#define HCI_TRACE_PRINTF_4(level, pFormat, Arg1, Arg2, Arg3, Arg4)                           \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                 \
        BTRACE_PRINTF_4(level, pFormat, Arg1, Arg2, Arg3, Arg4)
#define HCI_TRACE_PRINTF_5(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5)                     \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                               \
        BTRACE_PRINTF_5(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5)
#define HCI_TRACE_PRINTF_6(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6)               \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                               \
        BTRACE_PRINTF_6(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6)
#define HCI_TRACE_PRINTF_7(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6, Arg7)         \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                                 \
        BTRACE_PRINTF_7(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6, Arg7)
#define HCI_TRACE_PRINTF_8(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6, Arg7, Arg8)   \
    if (level&otp_str_data.gEfuse_UpperStack_s.traceMask)                              \
        BTRACE_PRINTF_8(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6, Arg7, Arg8)


#endif
