/**
*****************************************************************************************
*     Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
  * @file     mesh_bearer.h
  * @brief    Head file for mesh bearer layer.
  * @details  Data types and external functions declaration.
  * @author   bill
  * @date     2015-8-27
  * @version  v1.0
  * *************************************************************************************
  */

/* Define to prevent recursive inclusion */
#ifndef _MESH_BEARER_H
#define _MESH_BEARER_H

#ifdef __cplusplus
extern "C"  {
#endif      /* __cplusplus */

/* Add Includes here */
#include "rtl_types.h"
#include "mesh_config.h"
#include "mesh_beacon.h"
#include "mesh_network.h"

/** advertising bearer */
#define MESH_SIZE_OFFSET                        0
#define MESH_FLAG_OFFSET                        1
#define MESH_NWK_OFFSET                         2
#define MESH_BEACON_OFFSET                         2

/** mesh bearer type */
#define MESH_AD_BEARER
#define MESH_GATT_BEARER

typedef enum 
{
    MESH_AD_TYPE_MESH,
    MESH_AD_TYPE_MESH_BEACON
} TMeshAdType;

/* Add all public types here */
typedef struct
{
    uint8_t length;
    uint8_t flag;
    union
    {
        TMeshNwkPkt mesh_nwk_pkt;
        TMeshBeacon mesh_beacon;
    };
} TMeshAdPkt;

bool MeshBearerSend(TMeshAdType ad_type, uint8_t* pbuffer, uint8_t offset, uint8_t data_len);
void MeshBearerAdvertise(void);
bool MeshDataRamPoolInit(void);
void * MeshDataRamPoolBufferGet(void);
void MeshDataRamPoolBufferRelease(void * pbuffer);
void MeshBearerInit(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _MESH_BEARER_H */

