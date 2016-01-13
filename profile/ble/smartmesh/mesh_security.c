/**
*****************************************************************************************
*     Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
  * @file     mesh_security.c
  * @brief    Source file for mesh security.
  * @details  Data types and external functions declaration.
  * @author   bill
  * @date     2015-8-27
  * @version  v1.0
  * *************************************************************************************
  */

/* Add Includes here */
#include "mesh_common.h"
#include "mesh_security.h"
#include <string.h>
#include "aes_cmac.h"
#include "ccm.h"
#include "trace.h"

extern uint8_t netKey[16];
extern uint8_t EncryptionKey[16];

static uint8_t IVzero[16];
uint16_t IVindex = MESH_IV_INDEX;
uint8_t networkIV[16];

extern void traceBinary(uint8_t Mid, uint8_t pkt_type, uint16_t DataLength, uint8_t *pBinaryData);
void MeshShowKey(uint8_t * key, uint8_t len)
{
    traceBinary(0, 0xf0, len, key);
    
#if 0
    for(uint8_t i =0; i< len; i++)
    {
        DBG_BUFFER(MODULE_APP, LEVEL_INFO, "%02x", 1, key[i]);
    }
#endif
}


void MeshGenerateEncryptionKey(void)
{
    uint8_t smit[4] = {'s', 'm', 'n', 't'}; 
    uint8_t smiz[4] = {'s', 'm', 'n', 'k'};
    uint8_t mac[16];
    
    AES_CMAC(netKey, smit, 4, mac);
    AES_CMAC(mac, smiz, 4, EncryptionKey);
    
    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "EncryptionKey = ", 0);
    MeshShowKey(EncryptionKey, 16);
}


void MeshGenerateIVzero(void)
{
    uint8_t smit[4] = {'s', 'm', 'i', 't'}; 
    uint8_t smiz[4] = {'s', 'm', 'i', 'z'};
    uint8_t mac[16];
    
    AES_CMAC(netKey, smit, 4, mac);
    AES_CMAC(mac, smiz, 4, IVzero);
    
    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "IVzero = ", 0);
    MeshShowKey(IVzero, 16);
}

void MeshIVzeroGet(uint8_t * iv_zero)
{
    memcpy(iv_zero, IVzero, 16);
}

/** This function has to be called once after netKey or IVindex is set! */
void MeshGenerateNetworkIV(void)
{
    uint8_t smiv[4] = {'s', 'm', 'i', 'v'};
    uint8_t iv_zero_index[18];
    uint8_t mac[16];

    memcpy(iv_zero_index, IVzero, 16);
    BE_WORD2EXTRN(iv_zero_index + 16, IVindex);

    AES_CMAC(netKey, iv_zero_index, 18, mac);
    AES_CMAC(mac, smiv, 4, networkIV);

    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "NetworkIV = ", 0);
    MeshShowKey(networkIV, 16);
}

void MeshNetworkIVGet(uint8_t * network_iv)
{
    memcpy(network_iv, networkIV, 16);
}

void MeshIVindexSet(uint16_t value)
{
    IVindex = value;
    /** This function has to be called once after netKey or IVindex is set! */
    MeshGenerateNetworkIV();
}

uint16_t MeshIVindexGet(void)
{
    return IVindex;
}

