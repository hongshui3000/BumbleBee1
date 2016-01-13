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

/* Add Includes here */
#include "mesh_common.h"
#include "mesh_transport.h"
#include <string.h>
#include "ccm.h"
#include "trace.h"

//extern uint32_t seq; 
//extern uint16_t src;
extern uint8_t networkIV[16];

uint8_t appKey[16] = MESH_APP_KEY;
uint8_t appNonce[13];

void MeshAppKeySet(uint8_t * papp_key, uint8_t length)
{
    if (length < 16)
    {
        memset(appKey, 0, 16);
        memcpy(appKey, papp_key, length);
    }
    else
    {
        memcpy(appKey, papp_key, 16);
    }
}

void MeshAppKeyGet(uint8_t * papp_key)
{
    memcpy(papp_key, appKey, 16);
}

void MeshGenerateAppNonce(uint8_t pri, uint32_t seq, uint16_t src)
{    
    appNonce[0] = (pri & 0x01) << 7;
    appNonce[1] = (seq >> 16) & 0xff;
    appNonce[2] = (seq >> 8) & 0xff;
    appNonce[3] = seq & 0xff;
    BE_WORD2EXTRN(appNonce + 4, src);
    memcpy(appNonce + 6, networkIV + 9, 7);

    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "AppNonce = ", 0);
    MeshShowKey(appNonce, 13);
}

void MeshAppEncrypt(uint8_t app_msg[], uint8_t len)
{
    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "AppMsg len = %d, Payload = ", 1, len);
    MeshShowKey(app_msg, len);
    
    if(0 != mesh_security_ccm_encrypt_and_tag( (mesh_security_ccm_context *) &appKey, len, appNonce, 13, NULL, 0, app_msg, app_msg, app_msg + len, 4))
    {
        DBG_BUFFER(MODULE_APP, LEVEL_ERROR, "MeshAppEncrypt: failed!", 0);
        return;
    }

    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "EncAppMsg = ", 0);
    MeshShowKey(app_msg, len);
    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "AppMIC = ", 0);
    MeshShowKey(app_msg + len, 4);
}

bool MeshAppDecrypt(uint8_t app_msg[], uint8_t len)
{
    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "EncAppMsg = ", 0);
    MeshShowKey(app_msg, len);
    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "AppMIC = ", 0);
    MeshShowKey(app_msg + len, 4);
   
    if(0 != mesh_security_ccm_auth_decrypt( (mesh_security_ccm_context *) &appKey, len, appNonce, 13, NULL, 0, app_msg, app_msg, app_msg + len, 4))
    {
        DBG_BUFFER(MODULE_APP, LEVEL_ERROR, "MeshAppDecrypt: failed!", 0);
        return FALSE;
    }

    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "AppMsg len = %d, Payload = ", 1, len);
    MeshShowKey(app_msg, len);

    return TRUE;
}

void MeshTransportSend(uint16_t dest_addr, uint8_t pri, uint8_t* pbuffer, uint8_t offset, uint8_t data_len)
{
    /** process of dest address */
    // Not clear in spec!
    TMeshAddrType dest_addr_type = MESH_DEVICE_ADDRESS_TYPE;
    /** encrypt the app message */
    
    if (MESH_VIRTUAL_ADDRESS_TYPE == dest_addr_type)
    {
        
    }
    else
    {
        MeshGenerateAppNonce(pri, MeshSeqGet(), MeshSrcGet());
        MeshAppEncrypt(pbuffer + offset, data_len);
        MeshNwkSend(dest_addr, pri, pbuffer, offset, data_len + 4);
    }
}

void MeshTransportReceive(uint16_t src, uint8_t* pbuffer, uint8_t len, uint8_t ttl, uint8_t pri, uint32_t seq, uint16_t dst)
{
    //decrypt the app message
    MeshGenerateAppNonce(pri, seq, src);
    if (FALSE == MeshAppDecrypt(pbuffer, len - 4))
    {
        return;
    }
    // send to application layer
    MeshAppReceive(src, pbuffer, len - 4, ttl, pri, seq, dst);
}

