/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file        btcommon.h
* @brief      BT Protocol Modules: Common BT Definitions
* @details   
*
* @author   	gordon
* @date      	2015-07-10
* @version	v0.1
*/

#ifndef __BTCOMMON_H
#define __BTCOMMON_H

#include <btglbdef.h>
#include <flags.h>
#include <rtl_types.h>

#ifndef BT_US_WRITE_OFFSET_COUNT
#error("required count BT_US_WRITE_OFFSET_COUNT not defined - update flags")
#endif
#ifndef BT_DS_WRITE_OFFSET_COUNT
#error("required count BT_DS_WRITE_OFFSET_COUNT not defined - update flags")
#endif


/** Header Length for L2CAP */
#define L2CAP_HDR_LENGTH 4

/** set up the old offset used before introduction of flags based counts
     we use the greater value of down and upstream offset  */
#if BT_US_WRITE_OFFSET_COUNT > BT_DS_WRITE_OFFSET_COUNT
#define DATACB_INIT_OFFSET          BT_US_WRITE_OFFSET_COUNT
#else
#define DATACB_INIT_OFFSET          BT_DS_WRITE_OFFSET_COUNT
#endif


/** check minimal offset (ERTM and RFCOMM are *NOT* checked) */
#if ((BT_L1_HCI_DS_OFFSET + ACL_HDR_LENGTH + L2CAP_HDR_LENGTH) > BT_DS_WRITE_OFFSET_COUNT)
#error("BT_DS_WRITE_OFFSET_COUNT too small")
#endif



/** Get int16_t from 2 CHARS, Low-Endian format */
#define CHAR2SHORT(p) (((*(p)) & 0xff) + ((*((p)+1)) << 8))
#define SHORT2CHAR(p,w)                 \
    *((PBYTE)p)     = (uint8_t)((w) & 0xff);      \
    *(((PBYTE)p)+1) = /*lint -e(572,778)*/ (uint8_t)(((w)>>8) & 0xff)

/** Get LONG from 4 CHARS, Low-Endian format */
#define CHAR2LONG(p) (((uint32_t)(*((p)+0)) & 0xff) + ((uint32_t)(*((p)+1)) << 8) \
                   + ((uint32_t)(*((p)+2)) << 16)  + ((uint32_t)(*((p)+3)) << 24))

#define LONG2CHAR(p,w)                      \
    *((PBYTE)p)     = (uint8_t)((w) & 0xff);          \
    *(((PBYTE)p)+1) = (uint8_t)(((w)>>8) & 0xff);     \
    *(((PBYTE)p)+2) = (uint8_t)(((w)>>16) & 0xff);    \
    *(((PBYTE)p)+3) = (uint8_t)(((w)>>24) & 0xff);

/** Inquiry Access Codes:  bt spec 1.0b bluetooth assigned numbers, p 1012 */
#define GIAC_IDX        0
#define LIAC_IDX        1
#define DIAC_FIRST_IDX  2
#define DIAC_LAST_IDX   63
/** PSM Values */
#define PSM_SDP                0x01
#define PSM_RFCOMM             0x03
#define PSM_TCSBIN             0x05
#define PSM_TCSBINCORDLESS     0x07
#define PSM_BNEP               0x0F
#define PSM_HID_CONTROL        0x11
#define PSM_HID_INTERRUPT      0x13
#define PSM_AVCTP              0x17
#define PSM_AVDTP              0x19
#define PSM_UDI_CPLANE         0x1D
#define PSM_ATT                0x1F

/** LMP Feature Definitions, BT Core Spec. p. 234 */
#define LMP_FEAT_3SLOT            0x01 /**< now offset 0 */
#define LMP_FEAT_5SLOT            0x02
#define LMP_FEAT_ENCRYPT          0x04
#define LMP_FEAT_SLOTOFFSET       0x08
#define LMP_FEAT_TIMINGACCURACY   0x10
#define LMP_FEAT_SWITCH           0x20
#define LMP_FEAT_HOLD             0x40
#define LMP_FEAT_SNIFF            0x80

#define LMP_FEAT_PARK             0x01 /**< now offset 1 */
#define LMP_FEAT_RSSI             0x02
#define LMP_FEAT_CHANNELQUALITY   0x04
#define LMP_FEAT_SCOLINK          0x08
#define LMP_FEAT_HV2              0x10
#define LMP_FEAT_HV3              0x20
#define LMP_FEAT_ULAW             0x40
#define LMP_FEAT_ALAW             0x80

#define LMP_FEAT_CVSD             0x01 /**< now offset 2 */
#define LMP_FEAT_PAGINGSCHEME     0x02
#define LMP_FEAT_POWERCONTROL     0x04

/** now offset 3 */
#define LMP_FEAT_EDR_2MBPS        0x02
#define LMP_FEAT_EDR_3MBPS        0x04

#define LMP_FEAT_BYTE3_EV3        0x80

/** now offset 4 */
#define LMP_FEAT_BYTE4_EV4        0x01
#define LMP_FEAT_BYTE4_EV5        0x02

#define LMP_FEAT_LOWENERGY        0x40
#define LMP_FEAT_3SLOT_EDR        0x80

/** now offset 5 */
#define LMP_FEAT_5SLOT_EDR        0x01

/** now offset 6 */
#define LMP_FEAT_SSP              0x08

/** now offset 7 */
#define LMP_FEAT_EXT_FEAT         0x80

/** ext. features offset 0 */
#define LMP_EXTFEAT_SSP_HOST      0x01

#define PSM_UPFTEST             0x1231  /**< special PSM used for UnPlugFest test cases       */
#define PSM_UPFTEST_2           0x4461  /**< special PSM used for UnPlugFest test cases       */
#define PSM_UPFTEST_ERR         0x4461  /**< special PSM used for UnPlugFest test cases       */

#endif /* defined (__BTCOMMON_H) */

