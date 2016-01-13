/**
*****************************************************************************************
*     Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
  * @file     mesh_beacon.h
  * @brief    Head file for mesh beacon.
  * @details  Data types and external functions declaration.
  * @author   bill
  * @date     2015-8-27
  * @version  v1.0
  * *************************************************************************************
  */

/* Define to prevent recursive inclusion */
#ifndef _MESH_BEACON_H
#define _MESH_BEACON_H

#ifdef __cplusplus
extern "C"  {
#endif      /* __cplusplus */

/* Add Includes here */
#include "rtl_types.h"

#define MESH_SERVICE_UUID 0x7FD1

enum {
    BEACON_TYPE_UNPROVISIONED_NODE = 0x01,
    BEACON_TYPE_SECNWK = 0x02
};

typedef struct
{
    uint16_t service_uuid; 
} _PACKED_ TMeshBeaconHeader;

typedef struct
{
    uint8_t   device_uuid[16];
} _PACKED_ TMeshBeaconUnprovisionedNode;

typedef struct
{
    uint8_t kr_nid[8]; //!< Key Refresh Signaling Bit + Network ID
    uint16_t civi; //!< Current IV Index
    uint32_t cmac; //!< Authenticates the packet contents, computed with the Network Key over the [KR, NID, CIVI] fields
} _PACKED_ TMeshBeaconSecNwk;

typedef struct
{
    TMeshBeaconHeader header;
    uint8_t beacon_type;
    union
    {
        TMeshBeaconUnprovisionedNode  UnprovisionedNode;
        TMeshBeaconSecNwk sec_nwk;
    }p;
} TMeshBeacon;

void MeshBeaconReceive(uint8_t* pbuffer, uint8_t offset, uint8_t len);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _MESH_BEACON_H */

