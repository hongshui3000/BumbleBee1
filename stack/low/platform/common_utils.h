/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

#ifndef _COMMON_UTILS_H_
#define _COMMON_UTILS_H_

#include "DataType.h"
#include "common_defs.h"

/** 
 * \file common_utils.h
 *  Defines utility macros which are generic and can be used by both the
 *  platform and the core firmware.
 * 
 * \author Santhosh kumar M
 * \date 2007-12-30
 */

#define SWAP(x)    ((((x) & 0xFF) << 8) | (((x) >> 8) & 0xFF))
#define GEN_BIT_MASK(x)   ( ((UINT32)0x0001) << (x))

#define BITMASK_26      (0x03FFFFFF)
#define BITMASK_27      (0x07FFFFFF)
#define BITMASK_28      (0x0FFFFFFF)

/* Bit manipulation macros */
#define SET_BIT(val, mask)          ((val) |=  (mask))

#define CLR_BIT(val, mask)          ((val)  &= ~(mask))

#define TOGGLE_BIT(val, mask)       ((val) |=  (mask)); \
                                    ((val)  &= ~(mask))

#define EXTRACT_BIT(val, bit_num)   (((val) >> (bit_num)) & 1)

#define CHK_BIT(val, mask)          ((val) & (mask))

#define MERGE_BIT(oval, val, mask) ((oval) ^ (((oval) ^ (val)) & (mask)))

#define BITLEN_ALIGN_BYTE(len) (((len) + 0x7) >> 3)

/**
 * Convert 2 bytes from big-endian byte order to host byte order.
 * @param data  2 byte data in big-endian byte order.
 * @return  Data converted in host byte order.
 */
static inline UINT16 betoh16(UINT8 data[])
{
    return ((data[0] << 8) | data[1]);
}

/**
 * Convert 2 bytes from little-endian byte order to host byte order.
 * @param data  2 byte data in little-endian byte order.
 * @return  Data converted in host byte order.
 */
static inline UINT16 letoh16(UINT8 data[])
{
    return (data[0] | (data[1] << 8));
}

#ifndef UNIT_TEST
/**
 * Convert 2 bytes from host byte order to little-endian byte order.
 * @param val  2 byte data in host byte order.
 * @param buf  Buffer to store data converted to little-endian.
 */
static inline void htole16(UINT16 val, UINT8 buf[])
{
    buf[0] = LSB(val);
    buf[1] = MSB(val);
}

/**
 * Convert 4 bytes from host byte order to little-endian byte order.
 * @param val  4 byte data in host byte order.
 * @param buf  Buffer to store data converted to little-endian.
 */
static inline void htole32(UINT32 val, UINT8 buf[])
{
    buf[0] = (val & 0xFF);
    buf[1] = ((val >> 8) & 0xFF);
    buf[2] = ((val >> 16) & 0xFF);
    buf[3] = ((val >> 24) & 0xFF);
}
#endif
#endif /* _COMMON_UTILS_H_ */

