/**
*****************************************************************************************
*     Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
  * @file     mesh_common.c
  * @brief    Source file for mesh common part.
  * @details  Data types and external functions declaration.
  * @author   bill
  * @date     2015-9-14
  * @version  v1.0
  * *************************************************************************************
  */

/* Add Includes here */
#include "FreeRTOS.h"
#include "queue.h"
#include "timers.h"
#include <string.h>
#include "gap.h"
#include "observer_broadcaster.h"
#include "mesh_common.h"
#include "rtl876x_flash_storage.h"

static xQueueHandle hEventQueueHandle;
static xQueueHandle hMeshQueueHandle;

uint8_t MeshSendQueueMessage(TMeshQueueMsg * pmsg)
{
    portBASE_TYPE SendQueueResult;
    uint8_t Event;

    /* Send MSG to APP task */
    SendQueueResult = xQueueSend(hMeshQueueHandle, pmsg, 0xFFFF);
    if (SendQueueResult != pdPASS)
    {
        return pdFALSE;
    }

    /* Send EVENT to notify APP task */
    Event = EVENT_MESH;
    SendQueueResult = xQueueSend(hEventQueueHandle, &Event, 0xFFFF);
    if (SendQueueResult != pdPASS)
    {
        return pdFALSE;
    }

    return pdTRUE;

}

uint8_t MeshSendQueueMessageFromISR(TMeshQueueMsg *pmsg, long *pxHigherPriorityTaskWoken)
{
    portBASE_TYPE SendQueueResult;
    uint8_t Event;

    /* Send MSG to APP task */
    SendQueueResult = xQueueSendFromISR(hMeshQueueHandle, pmsg, pxHigherPriorityTaskWoken);
    if (SendQueueResult != pdPASS)
    {
        return pdFALSE;
    }

    /* Send EVENT to notify APP task */
    Event = EVENT_MESH;
    SendQueueResult = xQueueSendFromISR(hEventQueueHandle, &Event, pxHigherPriorityTaskWoken);
    if (SendQueueResult != pdPASS)
    {
        return pdFALSE;
    }

    return pdTRUE;
}

void MeshHandleQueueMsg(TMeshQueueMsg * pmsg)
{
    switch(pmsg->type)
    {
        case MESH_BT_STATUS_UPDATE:
            MeshHandleBtStatusMsg(pmsg);
            break;
        case MESH_SCAN_TIMEOUT:
            //ob_StopScan();
            break;
        case MESH_ADVERTISE_TIMEOUT:
            ob_StopAdvertising();
            break;
        case MESH_FORWARD_TABLE_REFRESH_TIMEOUT:
            MeshNwkForwardTableRefresh();
            break;
        default:
            DBG_BUFFER(MODULE_APP, LEVEL_ERROR, "MeshHandleQueueMsg: mesh queue message unknown type: %d", 1, pmsg->type);      
            break;
    }
}

void MeshHandleBtStatusMsg(TMeshQueueMsg * pmsg)
{
    BT_STACK_MSG BtStackMsg;
    memcpy(&BtStackMsg, &pmsg->parm, sizeof(pmsg->parm));
    
    switch(pmsg->sub_type)
    {
        case BT_MSG_TYPE_CONN_STATE_CHANGE:
            {
                bool ad_free_flag = FALSE;
                bool scan_free_flag = FALSE;
                uint8_t state;
                
                state = ob_ScanStateGet(BtStackMsg.msgData.gapConnStateChange.newState);
                switch (state)
                {
                    case OB_STATE_IDLE:
                        scan_free_flag = TRUE;
                        break;
                    case OB_STATE_START:
                        break;
                    case OB_STATE_STOP:
                        break;
                    case OB_STATE_ACTIVE:
                        break;
                    default:
                        break;
                }

                state = ob_AdvStateGet(BtStackMsg.msgData.gapConnStateChange.newState);
                switch (state)
                {
                    case OB_STATE_IDLE:
                        ad_free_flag = TRUE;
                        break;
                    case OB_STATE_START:
                        break;
                    case OB_STATE_STOP:
                        break;
                    case OB_STATE_ACTIVE:
                        break;
                    default:
                        break;
                }

                if (ad_free_flag == TRUE)
                {
                    MeshBearerAdvertise();
                }
                
                if (scan_free_flag == TRUE)
                {
                    ob_StartScan();
                }
            }
            break;
        default:
            break;
    }
}

void MeshFlashStore(TMeshFlashParams param, uint8_t used)
{
    uint32_t ret;
    switch (param)
    {
        case MESH_FLASH_PARAMS_APPKEY:
            {
                TMeshFlashAppKey flash_app_key;
                MeshAppKeyGet(flash_app_key.appKey);
                flash_app_key.used = used;
                ret = fs_save_vendor_data((void *)&flash_app_key, sizeof(TMeshFlashAppKey), MESH_FLASH_PARAMS_APPKEY_OFFSET);
            }
            break;
        case MESH_FLASH_PARAMS_NETKEY:
            {
                TMeshFlashNetKey flash_net_key;
                MeshNetKeyGet(flash_net_key.netKey);
                flash_net_key.used = used;
                ret = fs_save_vendor_data((void *)&flash_net_key, sizeof(TMeshFlashNetKey), MESH_FLASH_PARAMS_NETKEY_OFFSET);
            }
            break;
        case MESH_FLASH_PARAMS_SRC:
            {
                TMeshFlashSrc flash_src;
                flash_src.src = MeshSrcGet();
                flash_src.used = used;
                ret = fs_save_vendor_data((void *)&flash_src, sizeof(TMeshFlashSrc), MESH_FLASH_PARAMS_SRC_OFFSET);
            }
            break;
        case MESH_FLASH_PARAMS_TTL:
            {
                TMeshFlashTtl flash_ttl;
                flash_ttl.ttl = MeshTTLGet();
                flash_ttl.used = used;
                ret = fs_save_vendor_data((void *)&flash_ttl, sizeof(TMeshFlashTtl), MESH_FLASH_PARAMS_TTL_OFFSET);
            }
            break;
        case MESH_FLASH_PARAMS_IV_INDEX:
            {
                TMeshFlashIVindex flash_iv_index;
                flash_iv_index.IVindex = MeshIVindexGet();
                flash_iv_index.used = used;
                ret = fs_save_vendor_data((void *)&flash_iv_index, sizeof(TMeshFlashIVindex), MESH_FLASH_PARAMS_IV_INDEX_OFFSET);
            }
            break;
        case MESH_FLASH_PARAMS_FUT:
            {
                TMeshFlashFut flash_fut;
                flash_fut.fut = MeshFutGet();
                flash_fut.used = used;
                ret = fs_save_vendor_data((void *)&flash_fut, sizeof(TMeshFlashFut), MESH_FLASH_PARAMS_FUT_OFFSET);
            }
            break;
        case MESH_FLASH_PARAMS_SEQ:
            {
                TMeshFlashSeq flash_seq;
                flash_seq.seq= MeshSeqGet();
                flash_seq.used = used;
                ret = fs_save_vendor_data((void *)&flash_seq, sizeof(TMeshFlashSeq), MESH_FLASH_PARAMS_SEQ_OFFSET);
            }
            break;
        case MESH_FLASH_PARAMS_GROUP_ADDR:
            {
                TMeshFlashGroupAddr flash_group_addr; 
                if (used != 0)
                {
                    uint8_t count;
                    count = MeshNwkGroupAddrDump(flash_group_addr.group_addr, MESH_GROUP_ADDRESS_MAX_NUMBER);
                    flash_group_addr.used = count;
                }
                else
                {
                    flash_group_addr.used = 0;
                }
                ret = fs_save_vendor_data((void *)&flash_group_addr, sizeof(TMeshFlashGroupAddr), MESH_FLASH_PARAMS_GROUP_ADDR_OFFSET);
            }
            break;
        default:
            break;
    }

    if (ret != 0)
    {
        DBG_BUFFER(MODULE_APP, LEVEL_ERROR, "MeshFlashStore: failed to store mesh related information, params = %d, cause = %d", 2, param, ret);  
    }
}

void MeshFlashClear(void)
{
    uint8_t data[MESH_FLASH_PARAMS_LENGTH] = {0};
    fs_save_vendor_data((void *)data, MESH_FLASH_PARAMS_LENGTH, MESH_FLASH_PARAMS_OFFSET);
}

void MeshFlashRestore(void)
{
    uint32_t ret;
    TMeshFlashAppKey flash_app_key;
    TMeshFlashNetKey flash_net_key;
    TMeshFlashSrc flash_src;
    TMeshFlashTtl flash_ttl;
    TMeshFlashIVindex flash_iv_index;
    TMeshFlashFut flash_fut;
    TMeshFlashSeq flash_seq;
    TMeshFlashGroupAddr flash_group_addr;

    // load app key
    ret = fs_load_vendor_data((void *)&flash_app_key, sizeof(TMeshFlashAppKey), MESH_FLASH_PARAMS_APPKEY_OFFSET);
    if ((ret == 0) && (TRUE == flash_app_key.used))
    {
        MeshAppKeySet(flash_app_key.appKey, 16);
    }

    // load net key
    ret = fs_load_vendor_data((void *)&flash_net_key, sizeof(TMeshFlashNetKey), MESH_FLASH_PARAMS_NETKEY_OFFSET);
    if ((ret == 0) && (TRUE == flash_net_key.used))
    {
        MeshNetKeySet(flash_net_key.netKey, 16);
    }
    
    // load src
    ret = fs_load_vendor_data((void *)&flash_src, sizeof(TMeshFlashSrc), MESH_FLASH_PARAMS_SRC_OFFSET);
    if ((ret == 0) && (TRUE == flash_src.used))
    {
        MeshSrcSet(flash_src.src);
    }

    // load ttl
    ret = fs_load_vendor_data((void *)&flash_ttl, sizeof(TMeshFlashTtl), MESH_FLASH_PARAMS_TTL_OFFSET);
    if ((ret == 0) && (TRUE == flash_ttl.used))
    {
        MeshTTLSet(flash_ttl.ttl);
    }

    // load iv index
    ret = fs_load_vendor_data((void *)&flash_iv_index, sizeof(TMeshFlashIVindex), MESH_FLASH_PARAMS_IV_INDEX_OFFSET);
    if ((ret == 0) && (TRUE == flash_iv_index.used))
    {
        MeshIVindexSet(flash_iv_index.IVindex);
    }

    // load fut
    ret = fs_load_vendor_data((void *)&flash_fut, sizeof(TMeshFlashFut), MESH_FLASH_PARAMS_FUT_OFFSET);
    if ((ret == 0) && (TRUE == flash_fut.used))
    {
        MeshFutSet(flash_fut.fut);
    }

    // load seq
    ret = fs_load_vendor_data((void *)&flash_seq, sizeof(TMeshFlashSeq), MESH_FLASH_PARAMS_SEQ_OFFSET);
    if ((ret == 0) && (TRUE == flash_seq.used))
    {
        MeshSeqSet(flash_seq.seq);
    }

    // load group addresses
    ret = fs_load_vendor_data((void *)&flash_group_addr, sizeof(TMeshFlashGroupAddr), MESH_FLASH_PARAMS_GROUP_ADDR_OFFSET);
    if ((ret == 0) && (0 != flash_group_addr.used))
    {
        MeshNwkGroupAddrLoad(flash_group_addr.group_addr, flash_group_addr.used);
    }

}

void MeshInit(xQueueHandle event_queue_handle, xQueueHandle mesh_queue_handle, pfMeshReceive fun)
{
    hEventQueueHandle = event_queue_handle;
    hMeshQueueHandle = mesh_queue_handle;
        
    MeshBearerInit();
    MeshNwkInit();
    // restore mesh related information from flash when start
    MeshFlashRestore();
    MeshNwkKeysRefresh();
    MeshAppRegister(fun);
}

