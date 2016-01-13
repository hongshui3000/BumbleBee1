/**
*****************************************************************************************
*     Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
  * @file     mesh.c
  * @brief    Source file for mesh interface.
  * @details  Data types and external functions declaration.
  * @author   bill
  * @date     2015-10-10
  * @version  v1.0
  * *************************************************************************************
  */

/* Add Includes here */
#include <string.h>
#include "mesh_api.h"
#include "smdlmp.h"
#include "test_mesh_uart.h"

void MeshReceive(uint16_t src, uint8_t* pbuffer, uint8_t len, uint8_t ttl, uint8_t pri, uint32_t seq, uint16_t dst)
{
    testMeshCmdPrint( NULL,
                         "Received a mesh message from 0x%04x, seq = 0x%06x, priority = %d, ttl = %d, length = %d, dst = 0x%04x, msg = 0x",
                         src,
                         seq,
                         pri,
                         ttl,
                         len,
                         dst);
    for(uint8_t loop = 0; loop < len; loop++)
    {
        testMeshCmdPrint( NULL,
                         "%02x",
                         pbuffer[loop]);
    }
    testMeshCmdPrint( NULL, "\r\n");
    
    smdlp_ReceiveMsg(src, pbuffer, len, dst);
}
