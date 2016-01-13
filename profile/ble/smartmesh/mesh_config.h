/**
*****************************************************************************************
*     Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
  * @file     mesh_config.h
  * @brief    Head file for mesh configuration.
  * @details  Data types and external functions declaration.
  * @author   bill
  * @date     2015-8-27
  * @version  v1.0
  * *************************************************************************************
  */

/* Define to prevent recursive inclusion */
#ifndef _MESH_CONFIG_H
#define _MESH_CONFIG_H

#ifdef __cplusplus
extern "C"  {
#endif      /* __cplusplus */

/* Add Includes here */
#include "rtl_types.h"

// Mesh debug information show switch
#define MESH_DEBUG                              1

// Mesh stored message parameters
#define MAX_NUMBER_OF_MESH_MESSAGE              0x20 //!< The number of stored mesh advertise packets
#define MAX_NUMBER_OF_MESH_FORWARD_RECORD       0x20 //!< The number of devices to check duplicate packets

// Mesh time parameters
#define MESH_SCAN_PERIOD                        1000 //!< in units of millisecond
#define MESH_AD_PERIOD                          160 //!< in units of millisecondoriginal 160
#define MESH_BEACON_PERIOD                      10000 //!< in units of millisecondoriginal 160
#define MESH_FORWARD_TABLE_REFRESH_PERIOD       60000 //!< in units of millisecond, uint32_t

// Scan interval and window (units of 625us, 160=100ms)
#define MESH_SCAN_INTERVAL                      0xC0 /* 120ms */
#define MESH_SCAN_WINDOW                        0xB0 /* 110ms */

// Advertising interval (units of 625us, 160=100ms)
#define MESH_ADVERTISING_INTERVAL_MIN           0xA0 /* 100ms */
#define MESH_ADVERTISING_INTERVAL_MAX           0xA0 /* 100ms */

// Timers used by Mesh layers
#define MESH_SCAN_TIMER_ID                      1
#define MESH_ADV_TIMER_ID                       2
#define MESH_FORWORD_TABLE_REFRESH_TIMER_ID     3
#define MESH_ADV_BEACON_TIMER_ID                4

// Sequence number error range in case of overflow
#define MESH_SEQ_NUM_OVERFLOW_ERROR             (0x00ffffff - 100)

// Maximum number of group addresses stored in the flash
#define MESH_GROUP_ADDRESS_MAX_NUMBER           10

// application key default value
#define MESH_APP_KEY                            {0x63, 0x96, 0x47, 0x71, 0x73, 0x4f, 0xbd, 0x76, 0xe3, 0xb4, 0x05, 0x19, 0xd1, 0xd9, 0x4a, 0x48}

// net key default value
#define MESH_NET_KEY                            {0x7d, 0xd7, 0x36, 0x4c, 0xd8, 0x42, 0xad, 0x18, 0xc1, 0x7c, 0x2b, 0x82, 0x0c, 0x84, 0xc3, 0xd6}

//for beacon test only
//#define MESH_NET_KEY                            {0xef, 0xb2, 0x25, 0x5e, 0x64, 0x22, 0xd3, 0x30, 0x08, 0x8e, 0x09, 0xbb, 0x01, 0x5e, 0xd7, 0x07}



// network source address default value
#define MESH_SRC                                0x0405

// network TTL default value
#define MESH_TTL                                7

// network iv index
#define MESH_IV_INDEX                           2
//#define MESH_IV_INDEX                           0x0102

// network sequence number default value
#define MESH_SEQ                                0x010203

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _MESH_CONFIG_H */

