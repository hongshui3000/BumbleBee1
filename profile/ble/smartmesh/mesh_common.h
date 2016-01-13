/**
*****************************************************************************************
*     Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
  * @file     mesh_common.h
  * @brief    Head file for mesh common part.
  * @details  Data types and external functions declaration.
  * @author   bill
  * @date     2015-9-14
  * @version  v1.0
  * *************************************************************************************
  */

/* Define to prevent recursive inclusion */
#ifndef _MESH_COMMON_H
#define _MESH_COMMON_H


#define DEBUG_MESH_PACKT_ONLY

#ifdef __cplusplus
extern "C"  {
#endif      /* __cplusplus */

/* Add Includes here */
#include "mesh_config.h"
#include "mesh_api.h"
#include "mesh_security.h"
#include "mesh_bearer.h"
#include "mesh_network.h"
#include "mesh_transport.h"
#include "mesh_application.h"

#define MESH_FLASH_PARAMS_OFFSET                0
#define MESH_FLASH_PARAMS_APPKEY_OFFSET         (MESH_FLASH_PARAMS_OFFSET)
#define MESH_FLASH_PARAMS_NETKEY_OFFSET         (MESH_FLASH_PARAMS_APPKEY_OFFSET + sizeof(TMeshFlashAppKey))
#define MESH_FLASH_PARAMS_SRC_OFFSET            (MESH_FLASH_PARAMS_NETKEY_OFFSET + sizeof(TMeshFlashNetKey))
#define MESH_FLASH_PARAMS_TTL_OFFSET            (MESH_FLASH_PARAMS_SRC_OFFSET + sizeof(TMeshFlashSrc))
#define MESH_FLASH_PARAMS_IV_INDEX_OFFSET       (MESH_FLASH_PARAMS_TTL_OFFSET + sizeof(TMeshFlashTtl))
#define MESH_FLASH_PARAMS_FUT_OFFSET            (MESH_FLASH_PARAMS_IV_INDEX_OFFSET + sizeof(TMeshFlashIVindex))
#define MESH_FLASH_PARAMS_SEQ_OFFSET            (MESH_FLASH_PARAMS_FUT_OFFSET + sizeof(TMeshFlashFut))
#define MESH_FLASH_PARAMS_GROUP_ADDR_OFFSET     (MESH_FLASH_PARAMS_SEQ_OFFSET + sizeof(TMeshFlashSeq))
#define MESH_FLASH_PARAMS_LENGTH                (MESH_FLASH_PARAMS_GROUP_ADDR_OFFSET + sizeof(TMeshFlashGroupAddr))

#if ((2*MESH_GROUP_ADDRESS_MAX_NUMBER)%4 == 0)
#define MESH_GROUP_ADDRESS_PADDING_LENGTH       3
#else
#define MESH_GROUP_ADDRESS_PADDING_LENGTH       1
#endif

typedef struct
{
    uint8_t appKey[16];
    uint8_t used;
    uint8_t padding[3];
} TMeshFlashAppKey;

typedef struct
{
    uint8_t netKey[16];
    uint8_t used;
    uint8_t padding[3];

} TMeshFlashNetKey;

typedef struct
{
    uint16_t src;
    uint8_t used;
    uint8_t padding[1];
} TMeshFlashSrc;

typedef struct
{
    uint8_t ttl;
    uint8_t used;
    uint8_t padding[2];
} TMeshFlashTtl;

typedef struct
{
    uint16_t IVindex;
    uint8_t used;
    uint8_t padding[1];
} TMeshFlashIVindex;

typedef struct
{
    uint8_t fut;
    uint8_t used;
    uint8_t padding[2];
} TMeshFlashFut;

typedef struct
{
    uint32_t seq;
    uint8_t used;
    uint8_t padding[3];
} TMeshFlashSeq;

typedef struct
{
    uint16_t group_addr[MESH_GROUP_ADDRESS_MAX_NUMBER];
    uint8_t used; //!< The counter of the group addresses
    uint8_t padding[MESH_GROUP_ADDRESS_PADDING_LENGTH];
} TMeshFlashGroupAddr;

void MeshHandleBtStatusMsg(TMeshQueueMsg * pmsg);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _MESH_COMMON_H */

