/**
*****************************************************************************************
*     Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
  * @file     mesh_api.h
  * @brief    Head file for mesh application interface.
  * @details  Data types and external functions declaration.
  * @author   bill
  * @date     2015-10-19
  * @version  v1.0
  * *************************************************************************************
  */

/* Define to prevent recursive inclusion */
#ifndef _MESH_API_H
#define _MESH_API_H

#ifdef __cplusplus
extern "C"  {
#endif      /* __cplusplus */

/* Add Includes here */
#include "rtl_types.h"
#include "blueAPI_types.h"
#include "FreeRTOS.h"
#include "queue.h"

#define MESH_AD_BEARER_HEADER_SIZE              2 //!< length + flag
#define MESH_NWK_HEADER_SIZE                    9
#define MESH_APP_PAYLOAD_OFFSET                 (MESH_AD_BEARER_HEADER_SIZE + MESH_NWK_HEADER_SIZE)

#define EVENT_MESH                              80

typedef enum 
{
    MESH_BT_STATUS_UPDATE,
    MESH_SCAN_TIMEOUT,
    MESH_ADVERTISE_TIMEOUT,
    MESH_FORWARD_TABLE_REFRESH_TIMEOUT
} TMeshQueueMsgType;

typedef struct
{
    uint16_t type;
    uint16_t sub_type;
    union
    {
        uint32_t parm;
        void *pBuf;
    };
} TMeshQueueMsg;

typedef enum
{
    MESH_FLASH_PARAMS_APPKEY,
    MESH_FLASH_PARAMS_NETKEY,
    MESH_FLASH_PARAMS_SRC,
    MESH_FLASH_PARAMS_TTL,
    MESH_FLASH_PARAMS_IV_INDEX,
    MESH_FLASH_PARAMS_FUT,
    MESH_FLASH_PARAMS_SEQ,
    MESH_FLASH_PARAMS_GROUP_ADDR
} TMeshFlashParams;


typedef void (*pfMeshReceive) (uint16_t src, uint8_t* pbuffer, uint8_t len, uint8_t ttl, uint8_t pri, uint32_t seq, uint16_t dst);

uint8_t MeshSendQueueMessage(TMeshQueueMsg * pmsg);
uint8_t MeshSendQueueMessageFromISR(TMeshQueueMsg *pmsg, long *pxHigherPriorityTaskWoken);
void MeshHandleQueueMsg(TMeshQueueMsg * pmsg);

void MeshAppKeySet(uint8_t * papp_key, uint8_t length);
void MeshAppKeyGet(uint8_t * papp_key);
void MeshNetKeySet(uint8_t * pnet_key, uint8_t length);
void MeshNetKeyGet(uint8_t * pnet_key);
void MeshIVindexSet(uint16_t value);
uint16_t MeshIVindexGet(void);
void MeshSrcSet(uint16_t new_src);
uint16_t MeshSrcGet(void);
bool MeshNwkGroupAddrAdd(uint16_t addr);
bool MeshNwkGroupAddrDelete(uint16_t addr);
void MeshNwkGroupAddrClear(void);
uint8_t MeshNwkGroupAddrDump(uint16_t addr[], uint8_t num);
void MeshSeqSet(uint32_t new_seq);
uint32_t MeshSeqGet(void);
void MeshTTLSet(uint8_t new_ttl);
uint8_t MeshTTLGet(void);
void MeshFutSet(uint8_t new_fut);
uint8_t MeshFutGet(void);

void MeshNetworkIDGet(uint8_t * network_id);
void MeshIVzeroGet(uint8_t * iv_zero);
void MeshNetworkIVGet(uint8_t * network_iv);
void MeshPriKeyGet(uint8_t * pri_key);

void MeshFlashStore(TMeshFlashParams param, uint8_t used);
void MeshFlashClear(void);

void MeshBearerHandleLEScanInfo(PBlueAPI_LEScanInfo pLeScanInfo);

void * MeshDataRamPoolBufferGet(void);
bool MeshAppSend(uint16_t dst_addr, uint8_t pri, uint8_t* pbuffer, uint8_t offset, uint8_t data_len);
void MeshBeaconSecSend(uint8_t kr);
void MeshBeaconUnprovisionedSend(uint8_t device_uuid[]);

void MeshGapInit(void);
void MeshInit(xQueueHandle event_queue_handle, xQueueHandle mesh_queue_handle, pfMeshReceive fun);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _MESH_API_H */

