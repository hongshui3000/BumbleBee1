/**
*****************************************************************************************
*     Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
  * @file     mesh.h
  * @brief    Head file for mesh interface.
  * @details  Data types and external functions declaration.
  * @author   bill
  * @date     2015-10-10
  * @version  v1.0
  * *************************************************************************************
  */

/* Define to prevent recursive inclusion */
#ifndef _MESH_H
#define _MESH_H

#ifdef __cplusplus
extern "C"  {
#endif      /* __cplusplus */

/* Add Includes here */
#include "rtl_types.h"

void MeshReceive(uint16_t src, uint8_t* pbuffer, uint8_t len, uint8_t ttl, uint8_t pri, uint32_t seq, uint16_t dst);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _MESH_H */

