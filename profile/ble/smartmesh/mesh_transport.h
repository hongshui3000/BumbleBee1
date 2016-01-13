/**
*****************************************************************************************
*     Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
  * @file     mesh_transport.h
  * @brief    Head file for mesh transport layer.
  * @details  Data types and external functions declaration.
  * @author   bill
  * @date     2015-8-27
  * @version  v1.0
  * *************************************************************************************
  */

/* Define to prevent recursive inclusion */
#ifndef _MESH_TRANSPORT_H
#define _MESH_TRANSPORT_H

#ifdef __cplusplus
extern "C"  {
#endif      /* __cplusplus */

/* Add Includes here */
#include "rtl_types.h"

typedef struct
{
    uint8_t app_payload_app_mic[12+4]; //!< variable length app payload, Message Integrity Check for application with a length of 4 Bytes
} TMeshTransportMsg;

void MeshTransportSend(uint16_t dest_addr, uint8_t pri, uint8_t* pbuffer, uint8_t offset, uint8_t data_len);
void MeshTransportReceive(uint16_t src, uint8_t* pbuffer, uint8_t len, uint8_t ttl, uint8_t pri, uint32_t seq, uint16_t dst);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _MESH_TRANSPORT_H */

