/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       crc16btx.h
* @brief     Cyclic Redundancy Check (CRC) Implementation for BTX,
*  			Polynomial: X**0 + X**2 + X**15 + X16
*
* @author   	gordon
* @date      	2015-07-13
* @version	v0.1
*/

#if ! defined (__CRC16BTX_H)
#define       __CRC16BTX_H

/** includes */
#include <rtl_types.h>


/** defines and macros */
#define BTXFCS_INIT      0x0000  /**< Initial FCS value */
#define BTXFCS_GOOD      0x0000  /**< Good final FCS value */


/** types */
/** global variables */
/** methods */
uint16_t btxfcs( uint16_t fcs,
             uint8_t  *cp,
             uint16_t len );

#endif  /**< #if ! defined (__CRC16BTX_H) */

/** End of CRC16BTX.H */

