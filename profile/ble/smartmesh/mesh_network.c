/**
*****************************************************************************************
*     Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
  * @file     mesh_network.c
  * @brief    Source file for mesh network layer.
  * @details  Data types and external functions declaration.
  * @author   bill
  * @date     2015-8-27
  * @version  v1.0
  * *************************************************************************************
  */

/* Add Includes here */
#include "FreeRTOS.h"
#include "timers.h"
#include "mesh_common.h"
#include "mesh_network.h"
#include <string.h>
#include "aes.h"
#include "aes_cmac.h"
#include "ccm.h"
#include "trace.h"

extern uint8_t networkIV[16];
extern uint16_t IVindex;

static uint8_t fut;
static uint8_t ttl = MESH_TTL;
static uint32_t seq = MESH_SEQ;
static uint16_t src = MESH_SRC;
TGroupAddrElement * groupAddrList = NULL; // static

uint8_t netKey[16] = MESH_NET_KEY;
uint8_t EncryptionKey[16];
uint8_t networkID[16];
uint8_t priKey[16];
uint8_t privacyCounter[16]; 
uint8_t PECB[16];
uint8_t nwkNonce[13];

TimerHandle_t hMeshForwardTableRefreshTimer;
TMeshForwardRecord MeshForwardTable[MAX_NUMBER_OF_MESH_FORWARD_RECORD] = {0};

void MeshNetKeySet(uint8_t * pnet_key, uint8_t length)
{
    if (length < 16)
    {
        memset(netKey, 0, 16);
        memcpy(netKey, pnet_key, length);
    }
    else
    {
        memcpy(netKey, pnet_key, 16);
    }
    MeshNwkKeysRefresh();
}

void MeshNetKeyGet(uint8_t * pnet_key)
{
    memcpy(pnet_key, netKey, 16);
}

void MeshEncryptionKeyGet(uint8_t * pEncryption_key)
{
    memcpy(pEncryption_key, EncryptionKey, 16);
}


void MeshNetIDGet(uint8_t * pNID)
{
    memcpy(pNID, networkID, 16);
}

void MeshTTLSet(uint8_t new_ttl)
{
    ttl = new_ttl & 0x3f;
}

uint8_t MeshTTLGet(void)
{
    return ttl;
}

void MeshSrcSet(uint16_t new_src)
{
    if ((new_src != 0) && (new_src != 0xffff))
    {
        src = new_src & 0x7fff;
    }
}

uint16_t MeshSrcGet(void)
{
    return src;
}

void MeshSeqSet(uint32_t new_seq)
{
    seq = new_seq & 0xffffff;
}

uint32_t MeshSeqGet(void)
{
    return seq;
}

void MeshFutSet(uint8_t new_fut)
{
        fut = new_fut & 0x01;
}

uint8_t MeshFutGet(void)
{
    return fut;
}

TGroupAddrElement * MeshNwkGroupAddrFind(uint16_t addr)
{
    TGroupAddrElement * pgroup_addr;

    if (0x03 != (addr >> 14) || (0xffff == addr))
        return NULL;

    pgroup_addr = groupAddrList;
    while(pgroup_addr != NULL)
    {
        if (addr == pgroup_addr->group_addr)
        {
            return pgroup_addr;
        }
        else
        {
            pgroup_addr = pgroup_addr->next;
        }
    }

    return NULL;
}

bool MeshNwkGroupAddrAdd(uint16_t addr)
{
    if (0x03 != (addr >> 14) || (0xffff == addr))
        return FALSE;
        
    if (NULL == MeshNwkGroupAddrFind(addr))
    {
        TGroupAddrElement * pgroup_addr;
        pgroup_addr = (TGroupAddrElement *) pvPortMalloc(sizeof(TGroupAddrElement), RAM_TYPE_DATA_ON);
        if (NULL == pgroup_addr)
        {
            DBG_BUFFER(MODULE_APP, LEVEL_ERROR, "MeshNwkGroupAddrAdd: Failed to allocate memory for the new group addr!", 0);
            return FALSE;
        }
        else
        {
            pgroup_addr->group_addr = addr;
            pgroup_addr->next = groupAddrList;
            groupAddrList = pgroup_addr;
            return TRUE;
        }
    }
    
    return FALSE;
}

bool MeshNwkGroupAddrDelete(uint16_t addr)
{
    TGroupAddrElement * pgroup_addr, * plast;
    
    plast = NULL;
    pgroup_addr = groupAddrList;
    while(pgroup_addr != NULL)
    {
        if (addr == pgroup_addr->group_addr)
        {
            break;
        }
        else
        {
            plast = pgroup_addr;
            pgroup_addr = pgroup_addr->next;
        }
    }

    if (pgroup_addr != NULL)
    {
        if (pgroup_addr == groupAddrList)
        {
            groupAddrList = pgroup_addr->next;
        }
        else
        {
            plast->next = pgroup_addr->next;
        }
        vPortFree(pgroup_addr, RAM_TYPE_DATA_ON);

        return TRUE;
    }
    
    return FALSE;
}

uint8_t MeshNwkGroupAddrDump(uint16_t addr[], uint8_t num)
{
    TGroupAddrElement * pgroup_addr = groupAddrList;
    uint8_t count =0;

    while((pgroup_addr != NULL) && (count < num))
    {
        addr[count] = pgroup_addr->group_addr;
        count += 1;
        pgroup_addr = pgroup_addr->next;
    }

    return count;
}

void MeshNwkGroupAddrLoad(uint16_t addr[], uint8_t num)
{
    for (uint8_t loop = 0; loop < num; loop++)
    {
        MeshNwkGroupAddrAdd(addr[loop]);
    }
}

void MeshNwkGroupAddrClear(void)
{
    TGroupAddrElement * pgroup_addr = groupAddrList;
    while(pgroup_addr != NULL)
    {
        groupAddrList = pgroup_addr->next;
        vPortFree(pgroup_addr, RAM_TYPE_DATA_ON);
        pgroup_addr = groupAddrList;
    }
}

void MeshGenerateNetworkID(void)
{
    uint8_t smdt[4] = {'s', 'm', 'd', 't'}; 
    uint8_t smid[4] = {'s', 'm', 'i', 'd'};
    uint8_t mac[16];
    
    AES_CMAC(netKey, smdt, 4, mac);
    AES_CMAC(mac, smid, 4, networkID);
    
    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "NetworkID = ", 0);
    MeshShowKey(networkID, 16);
}

void MeshNetworkIDGet(uint8_t * network_id)
{
    memcpy(network_id, networkID, 16);
}

void MeshGeneratePriKey(void)
{
    uint8_t smpt[4] = {'s', 'm', 'p', 't'}; 
    uint8_t smpk[4] = {'s', 'm', 'p', 'k'};
    uint8_t mac[16];
    
    AES_CMAC(netKey, smpt, 4, mac);
    AES_CMAC(mac, smpk, 4, priKey);
    
    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "PriKey = ", 0);
    MeshShowKey(priKey, 16);
}

void MeshPriKeyGet(uint8_t * pri_key)
{
    memcpy(pri_key, priKey, 16);
}

void MeshGenerateNwkNonce(uint8_t *pnonce_src)//uint8_t pri, uint8_t fut, uint8_t ttl, uint32_t seq, uint16_t src
{
    memcpy(nwkNonce, pnonce_src, 6);
    memcpy(nwkNonce + 6, networkIV + 9, 7);
    
    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "NwkNonce = ", 0);
    MeshShowKey(nwkNonce, 13);
}

void MeshNwkEncrypt(uint8_t nwk_msg[], uint8_t len)
{
    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "NwkMsg = ", 0);
    MeshShowKey(nwk_msg, len);
    
    if (0 != mesh_security_ccm_encrypt_and_tag( (mesh_security_ccm_context *) &EncryptionKey, len, nwkNonce, 13, NULL, 0, nwk_msg, nwk_msg, nwk_msg + len, 4))
    {
        DBG_BUFFER(MODULE_APP, LEVEL_ERROR, "MeshNwkEncrypt: failed!", 0);
        return;
    }

    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "NwkEncMsg = ", 0);
    MeshShowKey(nwk_msg, len);
    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "NwkMIC = ", 0);
    MeshShowKey(nwk_msg + len, 4);
}

bool MeshNwkDecrypt(uint8_t nwk_msg[], uint8_t len)
{
    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "NwkEncMsg = ", 0);
    MeshShowKey(nwk_msg, len);
    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "NwkMIC = ", 0);
    MeshShowKey(nwk_msg + len, 4);
    
    if (0 != mesh_security_ccm_auth_decrypt( (mesh_security_ccm_context *) &EncryptionKey, len, nwkNonce, 13, NULL, 0, nwk_msg, nwk_msg, nwk_msg + len, 4))
    {
        DBG_BUFFER(MODULE_APP, LEVEL_ERROR, "MeshNwkDecrypt: failed!", 0);
        return FALSE;
    }

    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "NwkMsg = ", 0);
    MeshShowKey(nwk_msg, len);

    return TRUE;
}

void MeshGeneratePrivacyCounter(uint8_t nwk_msg[], uint8_t len)
{
    if (len < 7)
    {
        DBG_BUFFER(MODULE_APP, LEVEL_ERROR, "MeshGeneratePrivacyCounter: failed!", 0);
        return;
    }
    
    /** Attention that the expression in the spec isn't equal to the sample data */
    memcpy(privacyCounter, networkIV, 9);
    memcpy(privacyCounter + 9, nwk_msg, 7); // If the length of app payload eqauls zero, then the length of nwk message will be less than 7! 

    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "PrivacyCounter = ", 0);
    MeshShowKey(privacyCounter, 16);
}

void MeshNwkObfuscate(uint8_t privacy_header[])
{
    AES128_ECB_encrypt(privacyCounter, priKey, PECB);
    for(uint8_t loop = 0; loop < MESH_NWK_PRIVACY_HEADER_SIZE; loop++)
    {
        privacy_header[loop] = privacy_header[loop] ^ PECB[loop];
    }

    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "PECB = ", 0);
    MeshShowKey(PECB, 16);
    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "PrivacyHeader = ", 0);
    MeshShowKey(privacy_header, MESH_NWK_PRIVACY_HEADER_SIZE);
}

void MeshNwkSend(uint16_t dest_addr, uint8_t pri, uint8_t* pbuffer, uint8_t offset, uint8_t data_len)
{
    uint8_t *pnwk_msg = pbuffer + offset - MESH_NWK_HEADER_SIZE;
    TMeshNwkPkt *pmesh_nwk_msg = (TMeshNwkPkt *) pnwk_msg;
    
    uint8_t *pnwk_privacy_header = pnwk_msg + MESH_NWK_PRIVACY_HEADER_OFFSET;
    uint8_t *pnwk_enc_data = (uint8_t *) &pmesh_nwk_msg->dst;

    pmesh_nwk_msg->ivi = IVindex & 0x07;
    pmesh_nwk_msg->nid = networkID[15] & 0x1f;

    pmesh_nwk_msg->pri = pri; //((pri & 0x01) << 7) | ((fut & 0x01) << 6) | ( ttl & 0x3f);
    pmesh_nwk_msg->fut = fut;
    pmesh_nwk_msg->ttl = ttl;
    pmesh_nwk_msg->seq[0] = (seq >> 16) & 0xff;
    pmesh_nwk_msg->seq[1] = (seq >> 8) & 0xff;
    pmesh_nwk_msg->seq[2] = seq & 0xff;

    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "IVI = %02x", 1, pmesh_nwk_msg->ivi);
    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "NID = %02x", 1, pmesh_nwk_msg->nid);
    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "IVINID = %02x", 1, pmesh_nwk_msg->ivi|pmesh_nwk_msg->nid);
    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "PRI = %02x", 1, pmesh_nwk_msg->pri);
    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "FUT = %02x", 1, pmesh_nwk_msg->fut);
    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "TTL = %02x", 1, pmesh_nwk_msg->ttl);
    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "PFT = %02x", 1, pmesh_nwk_msg->pri|pmesh_nwk_msg->fut|pmesh_nwk_msg->ttl);
    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "SEQ = %06x", 1, seq );
    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "SRC = %04x", 1, src);
    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "DST = %04x", 1, dest_addr);
    
    BE_WORD2EXTRN((uint8_t *)&pmesh_nwk_msg->src, src); // big endian 
    BE_WORD2EXTRN(pnwk_enc_data, dest_addr);

    /** Encrypt app payload and dest addr */
    MeshGenerateNwkNonce(pnwk_privacy_header);
    MeshNwkEncrypt(pnwk_enc_data, data_len + sizeof(dest_addr));

    /** Obfuscate the network header */
    MeshGeneratePrivacyCounter(pnwk_enc_data, data_len + sizeof(dest_addr));
    MeshNwkObfuscate(pnwk_privacy_header);
    
    if (0x00ffffff <= seq)
    {
        seq = 0;
    }
    else
    {
        seq += 1;
    }

    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "NwkMsg = ", 0);
    MeshShowKey(pnwk_msg, MESH_NWK_HEADER_SIZE + data_len + 4);

    if (0x0000 == dest_addr) // loop back, for test
    {
        MeshNwkReceive(pnwk_msg, MESH_NWK_HEADER_SIZE + data_len + 4);
        MeshDataRamPoolBufferRelease(pbuffer);
    }
    else
    {
        // send to mesh bearer
        MeshBearerSend(MESH_AD_TYPE_MESH, pbuffer, offset - MESH_NWK_HEADER_SIZE, MESH_NWK_HEADER_SIZE + data_len + 4);
    }
}


void MeshNwkForwardTableRefresh(void)
{
    uint8_t loop;

    /** refresh all the used records */
    for (loop = 0; loop < MAX_NUMBER_OF_MESH_FORWARD_RECORD; loop++)
    {
        if (MeshForwardTable[loop].used == TRUE)
        {
            if (0 == MeshForwardTable[loop].num) // The record in the forward table is outdated!
            {
                MeshForwardTable[loop].used = FALSE; // clear the record!
            }
            else
            {
                MeshForwardTable[loop].num = 0; // clear the counter!
            }
        }
    }
}

bool MeshNwkDuplicateCheck(uint16_t sr_addr, uint32_t seq_num)
{
    uint8_t loop;

    /** look for the record matching the source device */
    for (loop = 0; loop < MAX_NUMBER_OF_MESH_FORWARD_RECORD; loop++)
    {
        if ((MeshForwardTable[loop].used == TRUE) && (MeshForwardTable[loop].sr_addr == sr_addr))
        {
            if ((MeshForwardTable[loop].seq_num < seq_num) || ((MeshForwardTable[loop].seq_num - seq_num) > MESH_SEQ_NUM_OVERFLOW_ERROR)) // In case the sequence number is overflowed!
            {
                MeshForwardTable[loop].seq_num = seq_num;
                if (MeshForwardTable[loop].num < 0x7fff) // Is there a necessary to check overflow in refresh interval?
                {
                    MeshForwardTable[loop].num += 1;
                }
                return FALSE;
            }
            else
            {
                return TRUE;
            }
        }
    }
    
    /** allocate a new record to the new source device  */
    if (loop == MAX_NUMBER_OF_MESH_FORWARD_RECORD)
    {
        for (loop = 0; loop < MAX_NUMBER_OF_MESH_FORWARD_RECORD; loop++)
        {
            if (MeshForwardTable[loop].used == FALSE)
            {
                MeshForwardTable[loop].num = 1; // The first message from this device.
                MeshForwardTable[loop].used = TRUE;
                MeshForwardTable[loop].sr_addr = sr_addr;
                MeshForwardTable[loop].seq_num = seq_num;
                return FALSE;
            }
        }

        if (loop == MAX_NUMBER_OF_MESH_FORWARD_RECORD)
        {
            return TRUE; // There is no record space for the new device! For the sake of preventing flood, the packets from this device should not be relayed!
        }
    }

    return TRUE;
}

void vTimerForwardTableRefreshTimeoutCb(TimerHandle_t pxTimer)
{
    TMeshQueueMsg forward_table_timeout_msg;
    forward_table_timeout_msg.type = MESH_FORWARD_TABLE_REFRESH_TIMEOUT;
    if (!MeshSendQueueMessage(&forward_table_timeout_msg))
    {
          
    }
    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "vTimerForwardTableRefreshTimeoutCb: Forward table refresh timer expires!", 0);
}

/*
uint32_t lastSeqNum = 0;
uint16_t neighborDeviceNum = 0;
typedef struct _TNeighborDeviceInfo
{
    uint8_t remote_BD[BLUE_API_BD_SIZE];
    int8_t rssi;
    struct _TNeighborDeviceInfo * pnext; //struct _TNeighborDeviceInfo
} TNeighborDeviceInfo;

TNeighborDeviceInfo * neighborDeviceListHeader = NULL;
*/

void MeshNwkReceive(uint8_t* pbuffer, uint8_t len)
{
    uint8_t bIvUpdating = FALSE;
    TMeshNwkPkt *pmesh_nwk_msg = (TMeshNwkPkt *)pbuffer;
    if (len <= MESH_NWK_PKT_MIN_SIZE)
    {
        DBG_BUFFER(MODULE_APP, LEVEL_ERROR, "MeshNwkReceive: Received mesh message, len = %d is too small", 1, len);
        return;
    }
    
    // verify the network id
    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "MeshNwkReceive: Received mesh message, nid = 0x%02x, ivi = %d", 2, pmesh_nwk_msg->nid, pmesh_nwk_msg->ivi);

    if (pmesh_nwk_msg->nid != (networkID[15] & 0x1f))
    {
        DBG_BUFFER(MODULE_APP, LEVEL_ERROR, "MeshNwkReceive mismatch: local nid = 0x%02x", 1, (networkID[15] & 0x1f));
        return;
    }

    // process the initialization vector bit
    //if (pmesh_nwk_msg->ivi != (IVindex & 0x07))//may update IVindex, but should check dest address
    uint16_t IVindexLast = IVindex;
    uint8_t IVILast = IVindex & 0x07;
    
    if(pmesh_nwk_msg->ivi > IVILast)
    {
        IVindex += (pmesh_nwk_msg->ivi -  IVILast);
        bIvUpdating = TRUE;
    }
    else 
    if(pmesh_nwk_msg->ivi < IVILast)
    {
        IVindex += (pmesh_nwk_msg->ivi +8 -IVILast);
        bIvUpdating = TRUE;
    }

    //need to update NetworkIV            
    if(TRUE == bIvUpdating)
    {
        //MeshIVindexSet(IVindex);
        MeshGenerateNetworkIV();
    }
        
    uint8_t *pnwk_privacy_header = (uint8_t *)pmesh_nwk_msg + MESH_NWK_PRIVACY_HEADER_OFFSET;
    uint8_t *pnwk_enc_data = (uint8_t *) &pmesh_nwk_msg->dst;
    uint16_t src_addr, dst_addr;
    uint32_t seq_num;
    // When the message is a broadcast, this buffer is used to save the nwk payload in case that the payload is decrypted by the transport layer before relayed.
    //uint8_t nwk_payload[16];
    uint8_t * prelay_buffer;
    TMeshNwkPkt * prelay_mesh_nwk_msg;
    // inverse nwk obfuscation, just repeat the same operations as the trasmit function
    MeshGeneratePrivacyCounter(pnwk_enc_data, len - MESH_NWK_ENC_OFFSET - 4);
    MeshNwkObfuscate(pnwk_privacy_header);

    // obtain the src and seq etc
    src_addr = BE_EXTRN2WORD((uint8_t *)&pmesh_nwk_msg->src);
    seq_num = pmesh_nwk_msg->seq[2] + (pmesh_nwk_msg->seq[1] << 8) + (pmesh_nwk_msg->seq[0] << 16);
    
    /** decrypt the app payload and dest */
    MeshGenerateNwkNonce(pnwk_privacy_header);

    if (FALSE == MeshNwkDecrypt(pnwk_enc_data, len - MESH_NWK_ENC_OFFSET - 4))
    {
        if(TRUE == bIvUpdating)
        {
            MeshIVindexSet(IVindexLast);
        }
        return;
    }
    else
    {
        if(TRUE == bIvUpdating)
        {
            //if decrypt successfully, we can use new IVindex
            MeshFlashStore(MESH_FLASH_PARAMS_IV_INDEX, TRUE);
        }
    }
    
    dst_addr = BE_EXTRN2WORD((uint8_t *)&pmesh_nwk_msg->dst);

    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "MeshNwkReceive: Received mesh message, src_addr = 0x%04x, seq_num = 0x%06x, dest_addr = 0x%04x, TTL = %d!", 4, src_addr, seq_num, dst_addr, pmesh_nwk_msg->ttl);
    if (0x0000 == dst_addr) // for loop-back test
    {
        MeshTransportReceive(src_addr, pmesh_nwk_msg->app_payload_nwk_mic, len - MESH_NWK_HEADER_SIZE - 4, pmesh_nwk_msg->ttl, pmesh_nwk_msg->pri, seq_num, dst_addr);
        return;
    }
#if 1
    if (src_addr == src) // Message sent by myself should be ignored before checking duplicate pkt! But this message can be used to calculate the number of neighbor devices!
    {
        DBG_BUFFER(MODULE_APP, LEVEL_ERROR, "MeshNwkReceive: Received mesh message from myself, src_addr(0x%04x), local src(0x%04x)", 2, src_addr, src);

/*
        TNeighborDeviceInfo *pDevice;
        if (seq_num != lastSeqNum)
        {
            DBG_BUFFER(MODULE_APP, LEVEL_INFO, "AppHandleLEScanInfo: Number of neighbor relay devices = %d!", 1, neighborDeviceNum);
            // clear statistic data 
            pDevice = neighborDeviceListHeader;
            while (pDevice != NULL)
            {
                neighborDeviceListHeader = pDevice-> pnext;
                vPortFree(pDevice, RAM_TYPE_DATA_ON);
                pDevice = neighborDeviceListHeader;
            }
            neighborDeviceNum = 0;
            lastSeqNum = seq_num;
        }

        pDevice = neighborDeviceListHeader;
        while (pDevice != NULL)
        {
            if (0 == memcmp(pDevice->remote_BD, pLeScanInfo->remote_BD, BLUE_API_BD_SIZE))
            {
                pDevice->rssi = pLeScanInfo->rssi;
                break;
            }
        }
        if (pDevice == NULL)
        {
            neighborDeviceNum += 1;
            pDevice = (TNeighborDeviceInfo *) pvPortMalloc(sizeof(TNeighborDeviceInfo), RAM_TYPE_DATA_ON);
            if(pDevice == NULL)
            {
                DBG_BUFFER(MODULE_APP, LEVEL_ERROR, "AppHandleLEScanInfo: failed to allocate memory for neighbor device!",0);
                return;
            }
 
            memcpy(&pDevice->remote_BD, pLeScanInfo->remote_BD, BLUE_API_BD_SIZE);
            pDevice->rssi = pLeScanInfo->rssi;

            pDevice->pnext = neighborDeviceListHeader;
            neighborDeviceListHeader = pDevice;
        }
*/
        return;
    }
    
    if (MeshNwkDuplicateCheck(src_addr, seq_num)) // Message is duplicate.
    {
        DBG_BUFFER(MODULE_APP, LEVEL_ERROR, "MeshNwkReceive: Message is duplicate",0);
        return;
    }
#endif
    
    if (dst_addr == src) // Message is sent to me!
    {
        MeshTransportReceive(src_addr, pmesh_nwk_msg->app_payload_nwk_mic, len - MESH_NWK_HEADER_SIZE - 4, pmesh_nwk_msg->ttl, pmesh_nwk_msg->pri, seq_num, dst_addr);
        //update IV index
        return;
    }

    if (pmesh_nwk_msg->ttl > MESH_MIN_TTL) // Message should be relayed, and check if TTL is valid.
    {
        //should not relay packet if IV update
        if(TRUE == bIvUpdating)
        {
            DBG_BUFFER(MODULE_APP, LEVEL_ERROR, "bIvUpdating is TRUE, not relay packets",0);
        }
        else
        {
            // Store the payload in case of being decrypted by the transport layer!
            //memcpy(nwk_payload, pmesh_nwk_msg->app_payload_nwk_mic, len - MESH_NWK_HEADER_SIZE - 4);
            prelay_buffer = MeshDataRamPoolBufferGet();
            if (prelay_buffer == NULL)
            {
                DBG_BUFFER(MODULE_APP, LEVEL_ERROR, "MeshNwkReceive: Not enough space to relay mesh message!", 0);
            }
            else
            {
                prelay_mesh_nwk_msg = (TMeshNwkPkt *)(prelay_buffer + MESH_NWK_OFFSET);
                memcpy(prelay_mesh_nwk_msg, pmesh_nwk_msg, len);
                prelay_mesh_nwk_msg->ttl -= 1; // Reduce TTL by one.

                // Re-encrypt the relayed mesh message
                MeshGenerateNwkNonce((uint8_t *)prelay_mesh_nwk_msg + MESH_NWK_PRIVACY_HEADER_OFFSET);
                MeshNwkEncrypt((uint8_t *) &prelay_mesh_nwk_msg->dst, len - MESH_NWK_ENC_OFFSET - 4);
                // Re-obfuscate the relayed mesh message
                MeshGeneratePrivacyCounter((uint8_t *) &prelay_mesh_nwk_msg->dst, len - MESH_NWK_ENC_OFFSET - 4);
                MeshNwkObfuscate((uint8_t *)prelay_mesh_nwk_msg + MESH_NWK_PRIVACY_HEADER_OFFSET);
                // Send the mesh message                
                MeshBearerSend(MESH_AD_TYPE_MESH, prelay_buffer, MESH_NWK_OFFSET, len);
            }
        }
    }
    
    if (0x03 == (dst_addr >> 14)) // Group or Broadcast Message should be processed, and may be relayed!
    {
        if ((dst_addr == MESH_BROADCAST_ADDRESS) || (NULL != MeshNwkGroupAddrFind(dst_addr)))
        {
            MeshTransportReceive(src_addr, pmesh_nwk_msg->app_payload_nwk_mic, len - MESH_NWK_HEADER_SIZE - 4, pmesh_nwk_msg->ttl, pmesh_nwk_msg->pri, seq_num, dst_addr);
            //update IV index
        }
    }
}

void MeshNwkInit(void)
{
    hMeshForwardTableRefreshTimer = xTimerCreate("MeshForwardTableRefreshTimer",       // Just a text name, not used by the kernel.
                                                   MESH_FORWARD_TABLE_REFRESH_PERIOD / portTICK_PERIOD_MS,   // The timer period in ticks.
                                                   pdTRUE,        // The timers will auto-reload themselves when they expire.
                                                   ( void *) MESH_FORWORD_TABLE_REFRESH_TIMER_ID,   // Assign each timer a unique id equal to its array index.
                                                   vTimerForwardTableRefreshTimeoutCb // Each timer calls the same callback when it expires.
                                                  );
    xTimerStart(hMeshForwardTableRefreshTimer, 0);
}

void MeshNwkKeysRefresh(void)
{
    MeshGenerateEncryptionKey();
    /** This function has to be called once after netKey is set! */
    MeshGenerateIVzero();
    /** This function has to be called once after netKey or IVindex is set! */
    MeshGenerateNetworkIV();
    /** This function has to be called once after netKey is set! */
    MeshGenerateNetworkID();
    MeshGeneratePriKey();
}


