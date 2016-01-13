/**
*****************************************************************************************
*     Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
  * @file     mesh_security.h
  * @brief    Head file for mesh security.
  * @details  Data types and external functions declaration.
  * @author   ranhui
  * @date     2015-8-27
  * @version  v1.0
  * *************************************************************************************
  */

/* Define to prevent recursive inclusion */
#ifndef _MESH_SECURITY_H
#define _MESH_SECURITY_H

#ifdef __cplusplus
extern "C"  {
#endif      /* __cplusplus */

/* Add Includes here */
#include "rtl_types.h"

void MeshShowKey(uint8_t * key, uint8_t len);
void MeshGenerateEncryptionKey(void);
void MeshGenerateIVzero(void);
void MeshGenerateNetworkIV(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _MESH_SECURITY_H */

