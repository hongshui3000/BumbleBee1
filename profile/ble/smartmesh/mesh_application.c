/**
*****************************************************************************************
*     Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
  * @file     mesh_application.c
  * @brief    Source file for mesh application layer.
  * @details  Data types and external functions declaration.
  * @author   bill
  * @date     2015-8-27
  * @version  v1.0
  * *************************************************************************************
  */

/* Add Includes here */
#include <string.h>
#include "mesh_common.h"
#include "mesh_application.h"
#include "smdlmp.h"

pfMeshReceive MeshReceiveCb = NULL;

void MeshAppRegister(pfMeshReceive fun)
{
    MeshReceiveCb = fun;
}

bool MeshAppSend(uint16_t dst_addr, uint8_t pri, uint8_t* pbuffer, uint8_t offset, uint8_t data_len) // uint8_t offset
{
    //TMeshAdPkt mesh_pkt;

    if(data_len > MESH_APP_PAYLOAD_MAX_SIZE)
    {
        data_len = MESH_APP_PAYLOAD_MAX_SIZE;
    }
    
    //memcpy(mesh_pkt.mesh_nwk_pkt.app_payload_nwk_mic, pbuffer, data_len);
    MeshTransportSend(dst_addr, pri, pbuffer, offset, data_len);
    return TRUE;
}

void MeshAppReceive(uint16_t src, uint8_t* pbuffer, uint8_t len, uint8_t ttl, uint8_t pri, uint32_t seq, uint16_t dst)
{
    if (MeshReceiveCb != NULL)
    {
        MeshReceiveCb(src, pbuffer, len, ttl, pri, seq, dst);
    }
}
