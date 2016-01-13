/**
*****************************************************************************************
*     Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
  * @file     mesh_bearer.c
  * @brief    Source file for mesh bearer layer.
  * @details  Data types and external functions declaration.
  * @author   bill
  * @date     2015-8-27
  * @version  v1.0
  * *************************************************************************************
  */

/* Add Includes here */
#include "FreeRTOS.h"
#include "queue.h"
#include "timers.h"
#include "blueapi.h"
#include "observer_broadcaster.h"
#include "mesh_common.h"
#include "mesh_bearer.h"
#include "mesh_beacon.h"

uint16_t meshDataRamPoolId = 0xFFFF;

//TimerHandle_t hMeshScanTimer;
TimerHandle_t hMeshAdvertiseTimer;
TimerHandle_t hMeshBeaconTimer;
xQueueHandle hMeshMessageQueueHandle; // Queue used to put mesh adv pkts

void vTimerMeshScanTimeoutCb(xTimerHandle pxTimer)
{	
    TMeshQueueMsg scan_timeout_msg;
    scan_timeout_msg.type = MESH_SCAN_TIMEOUT;
    if (!MeshSendQueueMessage(&scan_timeout_msg))
    {
          
    }
#ifndef DEBUG_MESH_PACKT_ONLY
    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "vTimerMeshScanTimeoutCb: Scan timer expires!", 0);
#endif
}

void vTimerMeshAdvertiseTimeoutCb(TimerHandle_t pxTimer)
{
    if (xTimerIsTimerActive( pxTimer ) != pdFALSE)
    {
    }
    else
    {
        TMeshQueueMsg adv_timeout_msg;
        adv_timeout_msg.type = MESH_ADVERTISE_TIMEOUT;
        if (!MeshSendQueueMessage(&adv_timeout_msg))
        {
              
        }
        DBG_BUFFER(MODULE_APP, LEVEL_INFO, "vTimerMeshAdvertiseTimeoutCb: Advertise timer expires!", 0);        
    }
}


void vTimerMeshBeaconAdvertiseTimeoutCb(TimerHandle_t pxTimer)
{
  
}

#if 0
static void MeshBearerHandleLEScanInfo(PBlueAPI_LEScanInfo pLeScanInfo)
{
    uint8_t buffer[32];
    uint8_t pos = 0;

    while (pos < pLeScanInfo->dataLength)
    {
        /* Length of the AD structure. */
        uint8_t length = pLeScanInfo->data[pos++];
        uint8_t type;

        if (length < 0x02 || length > 0x1F || (pos + length) > 0x1F)
            continue;
        
        /* Copy the AD Data to buffer. */
        memcpy(buffer, pLeScanInfo->data + pos + 1, length -1);
        /* AD Type, one octet. */
        type = pLeScanInfo->data[pos];
        
        DBG_BUFFER(MODULE_APP, LEVEL_INFO, "  AD Structure Info: AD type = 0x%x, AD Data Length = %d", 2, type, length -1);

        switch (type)
        {
            case GAP_ADTYPE_FLAGS:
            {
                /* (flags & 0x01) -- LE Limited Discoverable Mode */
                /* (flags & 0x02) -- LE General Discoverable Mode */
                /* (flags & 0x04) -- BR/EDR Not Supported */
                /* (flags & 0x08) -- Simultaneous LE and BR/EDR to Same Device Capable (Controller) */
                /* (flags & 0x10) -- Simultaneous LE and BR/EDR to Same Device Capable (Host) */
                uint8_t flags = pLeScanInfo->data[pos +1];
                DBG_BUFFER(MODULE_APP, LEVEL_INFO, "  AD Data: Flags = 0x%x", 1, flags);
            }
                break;

            case GAP_ADTYPE_16BIT_MORE:
            case GAP_ADTYPE_16BIT_COMPLETE:
            case GAP_ADTYPE_SERVICES_LIST_16BIT:
            {
                uint16_t *pUUID = (uint16_t *)(buffer);
                uint8_t i = length - 1;

                while (i >= 2)
                {
                    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "  AD Data: UUID16 List Item %d = 0x%x", 2, i/2, *pUUID++);
                    i -= 2;
                }
            }
                break;

            case GAP_ADTYPE_32BIT_MORE:
            case GAP_ADTYPE_32BIT_COMPLETE:
            {
                uint32_t *pUUID = (uint32_t *)(buffer);
                uint8_t    i     = length - 1;

                while (i >= 4)
                {
                    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "  AD Data: UUID32 List Item %d = 0x%x", 2, i/4, *pUUID++);
                    i -= 4;
                }
            }
                break;

            case GAP_ADTYPE_128BIT_MORE:
            case GAP_ADTYPE_128BIT_COMPLETE:
            case GAP_ADTYPE_SERVICES_LIST_128BIT:
            {
                uint32_t *pUUID = (uint32_t *)(buffer);
                DBG_BUFFER(MODULE_APP, LEVEL_INFO, "  AD Data: UUID128 value: 0x%8.8x%8.8x%8.8x%8.8x", 4,
                            pUUID[3], pUUID[2], pUUID[1], pUUID[0]);
            }
                break;

            case GAP_ADTYPE_LOCAL_NAME_SHORT:
            case GAP_ADTYPE_LOCAL_NAME_COMPLETE:
            {
                buffer[length -1] = '\0';
                DBG_BUFFER(MODULE_APP, LEVEL_INFO, "  AD Data: Local Name", 0);
            }
                break;

            case GAP_ADTYPE_POWER_LEVEL:
            {
                DBG_BUFFER(MODULE_APP, LEVEL_INFO, "  AD Data: TX power = 0x%x", 1, pLeScanInfo->data[pos + 1]);
            }
            break;

            case GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE:
            {
                uint16_t *pMin = (uint16_t *)(buffer);
                uint16_t *pMax = pMin +1;
                DBG_BUFFER(MODULE_APP, LEVEL_INFO, "  AD Data: Slave conn interval range, 0x%x - 0x%x", 2, *pMin, *pMax);
            }
                break;

            case GAP_ADTYPE_SERVICE_DATA:

                break;
                
            case GAP_ADTYPE_APPEARANCE:

                break;
            case GAP_ADTYPE_MESH_NETWORK:

                break;
                
                
            default:
            {
                uint8_t i = 0;

                for (i = 0; i < (length-1); i++)
                {
                    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "  AD Data: Unhandled Data = 0x%x", 1, pLeScanInfo->data[pos + i]);
                }
            }
                break;
        }

        pos += length;
    }
}
#endif

void MeshBearerHandleLEScanInfo(PBlueAPI_LEScanInfo pLeScanInfo)
{
    if((pLeScanInfo->dataLength >= MESH_AD_BEARER_HEADER_SIZE) && (pLeScanInfo->data[MESH_SIZE_OFFSET] >= (MESH_AD_BEARER_HEADER_SIZE - 1)))
    {
        if (pLeScanInfo->data[MESH_FLAG_OFFSET] == GAP_ADTYPE_MESH_NETWORK)
        {
            DBG_BUFFER(MODULE_APP, LEVEL_INFO,
                                            "MeshBearerHandleLEScanInfo: remote_BD = 0x%02x%02x%02x%02x%02x%02x,remote_BD_type = %d, advType = %d, rssi = %d, length = %d", 
                                            10, 
                                            pLeScanInfo->remote_BD[5],
                                            pLeScanInfo->remote_BD[4],
                                            pLeScanInfo->remote_BD[3],
                                            pLeScanInfo->remote_BD[2],
                                            pLeScanInfo->remote_BD[1],
                                            pLeScanInfo->remote_BD[0],
                                            pLeScanInfo->remote_BD_type,
                                            pLeScanInfo->advType,
                                            pLeScanInfo->rssi,
                                            pLeScanInfo->dataLength);
            if(pLeScanInfo->advType == blueAPI_LEAdvEventTypeNonConnectable)
            {
                MeshNwkReceive(pLeScanInfo->data + MESH_NWK_OFFSET, pLeScanInfo->data[MESH_SIZE_OFFSET] - 1);
            }
            else
            {
                DBG_BUFFER(MODULE_APP, LEVEL_ERROR, "MeshNwkReceive: Ignore adv packet, advertising type error", 0);
            }
        }
        else if (pLeScanInfo->data[MESH_FLAG_OFFSET] == GAP_ADTYPE_SERVICE_DATA)
        {
            // mesh beacon
            DBG_BUFFER(MODULE_APP, LEVEL_INFO,
                                            "MeshBeaconReceive: remote_BD = 0x%02x%02x%02x%02x%02x%02x,remote_BD_type = %d, advType = %d, rssi = %d, length = %d", 
                                            10, 
                                            pLeScanInfo->remote_BD[5],
                                            pLeScanInfo->remote_BD[4],
                                            pLeScanInfo->remote_BD[3],
                                            pLeScanInfo->remote_BD[2],
                                            pLeScanInfo->remote_BD[1],
                                            pLeScanInfo->remote_BD[0],
                                            pLeScanInfo->remote_BD_type,
                                            pLeScanInfo->advType,
                                            pLeScanInfo->rssi,
                                            pLeScanInfo->dataLength);
                
            MeshBeaconReceive(pLeScanInfo->data, MESH_BEACON_OFFSET, pLeScanInfo->data[MESH_SIZE_OFFSET] - 1);
        }
        else
        {
            // common advertising packet
#ifndef DEBUG_MESH_PACKT_ONLY            
            DBG_BUFFER(MODULE_APP, LEVEL_INFO, "Common Advertising packets", 0);
      //      MeshShowKey(pLeScanInfo->data, pLeScanInfo->dataLength);
#endif
        }
    }
}

void MeshBearerSendAdv(TMeshAdPkt *pmesh_ad_pkt, UBaseType_t num_mesh_message)
{
    //ob_StopScan();
    obSetGapParameter(GAPPRRA_ADVERT_DATA, pmesh_ad_pkt->length + 1, pmesh_ad_pkt);
    ob_StartAdvertising();
    if (pdFAIL == xTimerStart(hMeshAdvertiseTimer, 0))
    {
        DBG_BUFFER(MODULE_APP, LEVEL_ERROR, "!!! MeshBearerSendAdv: Failed to start adv timer !!!", 0);
    }
    DBG_BUFFER(MODULE_APP, LEVEL_INFO, "MeshBearerSendAdv: Mesh packet is relayed! Adv queue size = %d, length = %d, data = ", 2, num_mesh_message, pmesh_ad_pkt->length+1);
    MeshShowKey((uint8_t *)pmesh_ad_pkt, pmesh_ad_pkt->length+1);
    // release the mesh data buffer after the mesh pkt has been sent!
    MeshDataRamPoolBufferRelease(pmesh_ad_pkt);
}

bool MeshBearerSend(TMeshAdType ad_type, uint8_t* pbuffer, uint8_t offset, uint8_t data_len)
{
    bool ret = TRUE;
    uint8_t adv_state;
    UBaseType_t num_mesh_message;
    TMeshAdPkt *pmesh_ad_pkt = (TMeshAdPkt *) pbuffer;
    
    if (MESH_AD_TYPE_MESH == ad_type)
    {
        pmesh_ad_pkt->flag = GAP_ADTYPE_MESH_NETWORK;
    }
    else
    {
        pmesh_ad_pkt->flag = GAP_ADTYPE_SERVICE_DATA;
    }
    pmesh_ad_pkt->length = data_len + 1; // just include flag
    
    obGetGapParameter(GAPPRRA_ROLE_STATE, &adv_state);
    adv_state = ob_AdvStateGet(adv_state);
    num_mesh_message = uxQueueMessagesWaiting(hMeshMessageQueueHandle);
    if ((num_mesh_message == 0) && (adv_state == OB_STATE_IDLE))
    {
        MeshBearerSendAdv(pmesh_ad_pkt, num_mesh_message);
    }
    else
    {
        if (errQUEUE_FULL == xQueueSend(hMeshMessageQueueHandle, &pmesh_ad_pkt, 0)) // Should write operation be blocked?
        {
            DBG_BUFFER(MODULE_APP, LEVEL_ERROR, "!!! MeshBearerSend: Queue of mesh advertise packets is full !!!", 0);
            ret = FALSE;
        }
        else
        {
            DBG_BUFFER(MODULE_APP, LEVEL_INFO, "MeshBearerSend: Mesh advertise packet is pushed into the queue, queue size = %d!", 1, num_mesh_message + 1);
        }
    }
    return ret;
}

void MeshBearerAdvertise(void)
{
    UBaseType_t num_mesh_message;
    //num_mesh_message = uxQueueMessagesWaitingFromISR(hMeshMessageQueueHandle);
    num_mesh_message = uxQueueMessagesWaiting(hMeshMessageQueueHandle);
    if(num_mesh_message > 0)
    {
        TMeshAdPkt *pmesh_ad_pkt;
        //if (xQueueReceiveFromISR(hMeshMessageQueueHandle, &mesh_message, 0) == pdPASS)
        if (xQueueReceive(hMeshMessageQueueHandle, &pmesh_ad_pkt, 0) == pdPASS)
        {
            MeshBearerSendAdv(pmesh_ad_pkt, num_mesh_message);
        }
        else
        {
            DBG_BUFFER(MODULE_APP, LEVEL_ERROR, "MeshBearerAdvertise: Failed to queue out mesh advertise packet!",0);
        }
    }
    else
    {
        //ob_StartScan();
    }
}

bool MeshDataRamPoolInit(void)
{
    bool ret;
    uint8_t * mesh_data_ram_pool;
    uint16_t pool_size;
    pool_size = (((31+3)&~3)+4)*MAX_NUMBER_OF_MESH_MESSAGE;

    mesh_data_ram_pool = pvPortMalloc(pool_size, RAM_TYPE_DATA_OFF);
    if (mesh_data_ram_pool == NULL)
    {
        DBG_BUFFER(MODULE_APP, LEVEL_ERROR, "MeshDataRamPoolInit: failed to allocate memory for mesh pool", 0);
        return FALSE;
    }
    
    ret = blueAPI_DataRamPoolInit(mesh_data_ram_pool, pool_size);
    if (ret == FALSE)
    {
        DBG_BUFFER(MODULE_APP, LEVEL_ERROR, "MeshDataRamPoolInit: failed to init mesh pool", 0);
        return FALSE;
    }

    meshDataRamPoolId = blueAPI_DataRamPoolCreate(31, MAX_NUMBER_OF_MESH_MESSAGE);
    if (meshDataRamPoolId == 0xFFFF)
    {
        DBG_BUFFER(MODULE_APP, LEVEL_ERROR, "MeshDataRamPoolInit: failed to create mesh pool", 0);
        return FALSE;
    }

    return TRUE;
}

void * MeshDataRamPoolBufferGet(void)
{
    return blueAPI_DataRamPoolBufferGet(meshDataRamPoolId, 31);
}

void MeshDataRamPoolBufferRelease(void * pbuffer)
{
    blueAPI_DataRamPoolBufferRelease(meshDataRamPoolId, pbuffer);
}

void MeshBearerInit(void)
{
    MeshDataRamPoolInit();
    //hMeshMessageQueueHandle = xQueueCreate(MAX_NUMBER_OF_MESH_MESSAGE, sizeof(TMeshAdPkt));
    hMeshMessageQueueHandle = xQueueCreate(MAX_NUMBER_OF_MESH_MESSAGE, sizeof(void *));

#if 0    
    hMeshScanTimer = xTimerCreate("meshScanTimer",
                                    MESH_SCAN_PERIOD / portTICK_PERIOD_MS,
                                    pdTRUE,
                                    (void*)MESH_SCAN_TIMER_ID,
                                    vTimerMeshScanTimeoutCb
                                    );
    xTimerStart(hMeshScanTimer, 0);
#endif
    hMeshAdvertiseTimer = xTimerCreate("meshAdvertiseTimer",       // Just a text name, not used by the kernel.
                                       MESH_AD_PERIOD / portTICK_PERIOD_MS,   // The timer period in ticks.
                                       pdFALSE,        // The timers will auto-reload themselves when they expire.
                                       ( void *) MESH_ADV_TIMER_ID,   // Assign each timer a unique id equal to its array index.
                                       vTimerMeshAdvertiseTimeoutCb // Each timer calls the same callback when it expires.
                                      );

    if ( hMeshAdvertiseTimer == NULL )
    {
        // The timer was not created.
        DBG_BUFFER(MODULE_APP, LEVEL_ERROR, "Advertise timer is not created", 0);
    }
    else
    {
        // The timer was created.
        DBG_BUFFER(MODULE_APP, LEVEL_INFO, "Advertise timer is created", 0);
    }


    hMeshBeaconTimer = xTimerCreate("meshBeaconTimer",       // Just a text name, not used by the kernel.
                                       MESH_BEACON_PERIOD / portTICK_PERIOD_MS,   // The timer period in ticks.
                                       pdFALSE,        // The timers will auto-reload themselves when they expire.
                                       ( void *) MESH_ADV_BEACON_TIMER_ID,   // Assign each timer a unique id equal to its array index.
                                       vTimerMeshAdvertiseTimeoutCb // Each timer calls the same callback when it expires.
                                      );

    if ( hMeshBeaconTimer == NULL )
    {
        // The timer was not created.
        DBG_BUFFER(MODULE_APP, LEVEL_ERROR, "Beacon timer is not created", 0);
    }
    else
    {
        // The timer was created.
        DBG_BUFFER(MODULE_APP, LEVEL_INFO, "Beacon timer is created", 0);
    }
    



    
}

void MeshGapInit(void)
{
    // scan parameters
    uint16_t scan_interval = MESH_SCAN_INTERVAL;
    uint16_t scan_window = MESH_SCAN_WINDOW;
    
    //advertising parameters
    uint16_t adv_interval_min = MESH_ADVERTISING_INTERVAL_MIN;
    uint16_t adv_interval_max = MESH_ADVERTISING_INTERVAL_MIN;

    //Set Scan parameters
    obSetGapParameter(GAPPARA_SCANINTERVAL, sizeof(scan_interval), &scan_interval);
    obSetGapParameter(GAPPARA_SCANWINDOW, sizeof(scan_window), &scan_window);

    //Set advertising parameters
    obSetGapParameter(GAPPRRA_ADV_INTERVAL_MIN, sizeof(adv_interval_min), &adv_interval_min);
    obSetGapParameter(GAPPRRA_ADV_INTERVAL_MAX, sizeof(adv_interval_max), &adv_interval_max);
}

