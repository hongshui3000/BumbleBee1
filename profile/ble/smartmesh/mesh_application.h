/**
*****************************************************************************************
*     Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
  * @file     mesh_application.h
  * @brief    Head file for mesh application layer.
  * @details  Data types and external functions declaration.
  * @author   bill
  * @date     2015-8-27
  * @version  v1.0
  * *************************************************************************************
  */

/* Define to prevent recursive inclusion */
#ifndef _MESH_APPLICATION_H
#define _MESH_APPLICATION_H

#ifdef __cplusplus
extern "C"  {
#endif      /* __cplusplus */

/* Add Includes here */
#include "rtl_types.h"

#define MESH_APP_PAYLOAD_MAX_SIZE           12

typedef struct
{
    uint8_t op_tid_params[MESH_APP_PAYLOAD_MAX_SIZE]; //!< variable length application message, composed of Operation Code, Transaction Identifier, and Application Parameters
} TMeshAppMsg;

void MeshAppRegister(pfMeshReceive fun);
void MeshAppReceive(uint16_t src, uint8_t* pbuffer, uint8_t len, uint8_t ttl, uint8_t pri, uint32_t seq, uint16_t dst);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _MESH_APPLICATION_H */

