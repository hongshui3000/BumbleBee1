/**
*****************************************************************************************
*     Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
  * @file     mesh_beacon.c
  * @brief    Source file for mesh beacon.
  * @details  Data types and external functions declaration.
  * @author   bill
  * @date     2015-8-27
  * @version  v1.0
  * *************************************************************************************
  */

/* Add Includes here */
#include "mesh_common.h"
#include "mesh_bearer.h"
#include "mesh_beacon.h"
#include "aes_cmac.h"
#include "trace.h"
#include <string.h>

void MeshBeaconReceive(uint8_t* pbuffer, uint8_t offset, uint8_t len)
{
    TMeshBeacon*pmesh_beacon_msg = (TMeshBeacon *)(pbuffer + offset);
    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "MeshBeaconReceive: service_uuid(0x%x), beacon_type(%d)", 2, 
                                                                    pmesh_beacon_msg->header.service_uuid, 
                                                                    pmesh_beacon_msg->beacon_type);
    //for test only
//    return;
    
    switch(pmesh_beacon_msg->beacon_type)
    {
        case BEACON_TYPE_UNPROVISIONED_NODE:
        {
            TMeshBeaconUnprovisionedNode *pMeshBeaconUnprovisionedNode = (TMeshBeaconUnprovisionedNode *)&pmesh_beacon_msg->p.UnprovisionedNode;
            (void)pMeshBeaconUnprovisionedNode;
            break;
        }
        case BEACON_TYPE_SECNWK:
        {
            TMeshBeaconSecNwk *pMeshBeaconSecNwk = (TMeshBeaconSecNwk *)&pmesh_beacon_msg->p.sec_nwk;

//            uint8_t testRxBeacon[14] = {0x7f, 0x28, 0x9a, 0x8c, 0x32, 0x18, 0x8a, 0x3e, 0x01, 0x02, 0xa6, 0xc2, 0x3b, 0xef};

            uint8_t encryption_key[16];
            uint8_t temp_mac[16];
            uint8_t mac[16];
            uint8_t smnb[4] = {'s', 'm', 'n', 'b'};
            uint8_t netID[16];
            MeshEncryptionKeyGet(encryption_key);
            MeshNetIDGet(netID);
            uint16_t ivi = MeshIVindexGet();
            uint16_t CIVI = 0;

            uint8_t kr = (pMeshBeaconSecNwk->kr_nid[0]&0x80)?1:0;
            CIVI = LE_EXTRN2WORD((uint8_t*)&pMeshBeaconSecNwk->civi);


            DBG_BUFFER(MODULE_APP, LEVEL_INFO, "RX  KR= %d", 1, kr);
            DBG_BUFFER(MODULE_APP, LEVEL_INFO, "RX  CIVI= 0x%x", 1, CIVI);

            uint8_t kr_nid_civi[19];
            memcpy(kr_nid_civi, &kr, 1);
            memcpy(kr_nid_civi+1, netID, 16);
            memcpy(kr_nid_civi+1+16, &CIVI, 2);
            DBG_BUFFER(MODULE_APP, LEVEL_INFO, "kr_nid_civi = ", 0);
            MeshShowKey(kr_nid_civi, 19);            

            AES_CMAC(encryption_key, kr_nid_civi, 19, temp_mac);

            DBG_BUFFER(MODULE_APP, LEVEL_INFO, "Temp Mac = ", 0);
            MeshShowKey(temp_mac, 16);//pointer to cmac, offset 10

            AES_CMAC(temp_mac, smnb, 4, mac);

            DBG_BUFFER(MODULE_APP, LEVEL_INFO, "Received Beacon Mac = ", 0);
            MeshShowKey((uint8_t*)pMeshBeaconSecNwk+ 10, 4);//pointer to cmac, offset 10

            DBG_BUFFER(MODULE_APP, LEVEL_INFO, "Calc Beacon Mac = ", 0);
            MeshShowKey(mac, 16);
            
            if(memcmp((uint8_t*)pMeshBeaconSecNwk+ 10, mac+12, 4) == 0)
            {
                 DBG_BUFFER(MODULE_APP, LEVEL_INFO, "MAC MATCH", 0);               
            }
            else
            {
                 DBG_BUFFER(MODULE_APP, LEVEL_ERROR, "MAC dis-match", 0);
            }
            
            break;
        }
        default:
            break;

    }

    
   
}


void MeshBeaconSecSend(uint8_t kr)
{

	
    uint8_t encryption_key[16];
    uint8_t temp_mac[16];
    uint8_t mac[16];
    uint8_t smnb[4] = {'s', 'm', 'n', 'b'};
    uint8_t netID[16];
    MeshEncryptionKeyGet(encryption_key);
    MeshNetIDGet(netID);
    uint16_t CIVI = MeshIVindexGet();
    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "CIVI = 0x%x", 1, CIVI);

    uint8_t kr_nid_civi[19];
    memcpy(kr_nid_civi, &kr, 1);
    memcpy(kr_nid_civi+1, netID, 16);
    BE_WORD2EXTRN(kr_nid_civi+1+16, CIVI);

    
    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "kr_nid_civi = ", 0);
    MeshShowKey(kr_nid_civi, 19);            

    AES_CMAC(encryption_key, kr_nid_civi, 19, temp_mac);

    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "Temp Mac = ", 0);
    MeshShowKey(temp_mac, 16);//pointer to cmac, offset 10

    AES_CMAC(temp_mac, smnb, 4, mac);


    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "Calc Beacon Mac = ", 0);
    MeshShowKey(mac, 16);

    uint8_t * pbuffer;
    TMeshBeacon *pMeshBeacon;

    pbuffer = MeshDataRamPoolBufferGet();
    if (pbuffer == NULL)
    {
        return ;
    }

    pMeshBeacon = (TMeshBeacon *) (pbuffer + 2);

    pMeshBeacon->header.service_uuid = MESH_SERVICE_UUID;
    pMeshBeacon->beacon_type = BEACON_TYPE_SECNWK;
    netID[8] &=0x7f; 
    if(kr)
    {
        netID[8] |= 0x80;
    }
    else
    {
        netID[8] &=0x7F;
    }
    
    memcpy(pMeshBeacon->p.sec_nwk.kr_nid, &netID[8], 8); 
    BE_WORD2EXTRN((uint8_t*)&pMeshBeacon->p.sec_nwk.civi, CIVI);
    pMeshBeacon->p.sec_nwk.cmac = LE_EXTRN2DWORD(&mac[12]);

    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "TX Beacon Data = ", 0);
    MeshShowKey((uint8_t *)pMeshBeacon, 17);//pointer to cmac, offset 10

   
    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "MeshBeaconSecSend", 0);
    MeshBearerSend(MESH_AD_TYPE_MESH_BEACON, pbuffer, 2, 17);


    return;

}

void MeshBeaconUnprovisionedSend(uint8_t device_uuid[])
{

    uint8_t * pbuffer;
    TMeshBeacon *pMeshBeacon;

    pbuffer = MeshDataRamPoolBufferGet();
    if (pbuffer == NULL)
    {
        return ;
    }

    pMeshBeacon = (TMeshBeacon *) (pbuffer + 2);
    pMeshBeacon->header.service_uuid = MESH_SERVICE_UUID;

    pMeshBeacon->beacon_type = BEACON_TYPE_UNPROVISIONED_NODE;
    memcpy(pMeshBeacon->p.UnprovisionedNode.device_uuid, device_uuid, 16);

    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "MeshBeaconUnprovisionedSend", 0);
    MeshBearerSend(MESH_AD_TYPE_MESH_BEACON, pbuffer, 2, 2+1+16);//service_uuid(2), beacon_type(1),device_uuid(16)

    return;
  
}

void MeshBeaconTest()
{
    uint8_t testRxBeacon[14] = {0x7f, 0x28, 0x9a, 0x8c, 0x32, 0x18, 0x8a, 0x3e, 0x01, 0x02, 0xa6, 0xc2, 0x3b, 0xef};
    TMeshBeaconSecNwk meshBeacon;
    
    memcpy(&meshBeacon, testRxBeacon, 14);

    uint8_t encryption_key[16];
    uint8_t temp_mac[16];
    uint8_t mac[16];
    uint8_t smnb[4] = {'s', 'm', 'n', 'b'};
    uint8_t netID[16];
    MeshEncryptionKeyGet(encryption_key);
    MeshNetIDGet(netID);
    uint16_t ivi = MeshIVindexGet();
    uint16_t CIVI = 0;

    uint8_t kr = meshBeacon.kr_nid[0]&0x80;
    CIVI = LE_EXTRN2WORD((uint8_t*)&meshBeacon.civi);
        

    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "RX  KR= %d", 1, kr);
    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "RX  CIVI= 0x%x", 1, CIVI);

    uint8_t kr_nid_civi[19];
    memcpy(kr_nid_civi, &kr, 1);
    memcpy(kr_nid_civi+1, netID, 16);
    memcpy(kr_nid_civi+1+16, &CIVI, 2);
    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "kr_nid_civi = ", 0);
    MeshShowKey(kr_nid_civi, 19);            

    AES_CMAC(encryption_key, kr_nid_civi, 19, temp_mac);

    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "Temp Mac = ", 0);
    MeshShowKey(temp_mac, 16);//pointer to cmac, offset 10

    AES_CMAC(temp_mac, smnb, 4, mac);

    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "Received Beacon Mac = ", 0);
    MeshShowKey((uint8_t*)&meshBeacon+ 10, 4);//pointer to cmac, offset 10

    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "Calc Beacon Mac = ", 0);
    MeshShowKey(mac, 16);
    
    if(memcmp((uint8_t*)&meshBeacon+ 10, mac+12, 4) == 0)
    {
         DBG_BUFFER(MODULE_APP, LEVEL_INFO, "MAC MATCH", 0);               
    }
    else
    {
         DBG_BUFFER(MODULE_APP, LEVEL_ERROR, "MAC dis-match", 0);
    }

}

