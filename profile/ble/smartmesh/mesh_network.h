/**
*****************************************************************************************
*     Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
  * @file     mesh_network.h
  * @brief    Head file for mesh network layer.
  * @details  Data types and external functions declaration.
  * @author   bill
  * @date     2015-8-27
  * @version  v1.0
  * *************************************************************************************
  */

/* Define to prevent recursive inclusion */
#ifndef _MESH_NETWORK_H
#define _MESH_NETWORK_H

#ifdef __cplusplus
extern "C"  {
#endif      /* __cplusplus */

/* Add Includes here */
#include "rtl_types.h"

#define MESH_NWK_HEADER_SIZE                    9
#define MESH_NWK_ENC_OFFSET                     7 //!< The encryption of network layer starts from the destincation address!
#define MESH_NWK_PRIVACY_HEADER_OFFSET          1
#define MESH_NWK_PRIVACY_HEADER_SIZE            6
#define MESH_NWK_PKT_MIN_SIZE                   (MESH_NWK_HEADER_SIZE + 4) // open interval

#define MESH_MIN_TTL                            1
#define MESH_BROADCAST_ADDRESS                  0xffff

typedef enum _TMeshAddrType
{
    MESH_LOCAL_ADDRESS_TYPE,
    MESH_DEVICE_ADDRESS_TYPE,
    MESH_VIRTUAL_ADDRESS_TYPE,
    MESH_GROUP_ADDRESS_TYPE,
    MESH_BROADCAST_ADDRESS_TYPE
} TMeshAddrType;

/** mesh network packet format */
typedef struct 
{
    uint8_t nid: 5; //!< least significant 5 bits of network identity
    uint8_t ivi: 3; //!< least significant 3 bits of initialization vector index
    uint8_t ttl: 6; //!< time to live
    uint8_t fut: 1; //!< future
    uint8_t pri: 1; //!< priority
    uint8_t seq[3]; //!< sequence number
    uint16_t src; //!< source address
    uint16_t dst; //!< destination address
    uint8_t app_payload_nwk_mic[16+4]; //!< app payload with a varying length of 1 ~ 16 bytes and 4-bytes network message integrity check (MIC)
} _PACKED_ TMeshNwkPkt;

typedef struct
{
    uint16_t num:  15;
    uint16_t used: 1;
    uint16_t sr_addr;
    uint32_t seq_num;
} TMeshForwardRecord;

typedef struct _TGroupAddrElement
{
    uint16_t group_addr;
    struct _TGroupAddrElement * next;     
} TGroupAddrElement;

void MeshNwkForwardTableRefresh(void);
void MeshEncryptionKeyGet(uint8_t * pEncryption_key);
void MeshNetIDGet(uint8_t * pNID);
TGroupAddrElement * MeshNwkGroupAddrFind(uint16_t addr);
void MeshNwkGroupAddrLoad(uint16_t addr[], uint8_t num);
void MeshGenerateNetworkID(void);
void MeshNwkSend(uint16_t dest_addr, uint8_t pri, uint8_t* pbuffer, uint8_t offset, uint8_t data_len);
void MeshNwkReceive(uint8_t* pbuffer, uint8_t len);
void MeshNwkInit(void);
void MeshGeneratePriKey(void);
void MeshNwkKeysRefresh(void);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _MESH_NETWORK_H */

