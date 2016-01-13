/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       blueapi_osif.c
* @brief     btmos init and blueapi buf manager
* @details   
*
* @author   	gordon
* @date      	2015-06-26
* @version	v0.1
*/

#include <flags.h>
#include <rtl_types.h>
#include <blueface.h>
#include <btglbdef.h>
#include <bt_api.h>
#include <os_message.h>
#include <os_pool.h>
#include <os_mem.h>
#include <blueapi_types.h>
#include <sdplib.h>
#include <sdp_code.h>
#include <blueapi_def.h>
#include <blueapi_osif.h>
#include <upper_stack_global.h>

#define TRACE_MODULE_ID  MID_BT_HDP

void  blueAPIEntry(PBlueAPI_Data pBlueAPIData)
{
    MESSAGE_T msg;

    if (osMessageReceive(blueAPIQueueID, &msg) == 0)
    {
        switch (msg.Command)
        {
        case BLUE_API_MSG:
            blueAPI_Handle_Command((PBlueAPI_DsMessage)(msg.MData.DataCB.BufferAddress));
            break;

        default:
            BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_ERROR,
                "!!!blueAPIEntry: unknown message cmd(0x%X)", msg.Command);

            DebuggerBreak();
            break;
        }
    }
}

/**
* @brief  init bt mos
* 
* @param
* @return
*
*/
void blueAPIInit(void)
{
    pBlueAPIData = osMemoryClearAllocate(RAM_TYPE_DATA_ON, sizeof(TBlueAPI_Data));

    BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE, "blueAPI: init instance...");

    pBlueAPIData->plinkDescriptorTableDon = osMemoryAllocate(RAM_TYPE_DATA_ON,
        sizeof(TBlueAPI_LinkDescriptor)*otp_str_data.gEfuse_UpperStack_s.num_link_don);
    pBlueAPIData->plinkDescriptorTableDoff = osMemoryAllocate(RAM_TYPE_DATA_OFF,
        sizeof(TBlueAPI_LinkDescriptor)*otp_str_data.gEfuse_UpperStack_s.num_link_doff);

    pBlueAPIData->pMCLDescriptorTableDon = osMemoryAllocate(RAM_TYPE_DATA_ON,
        sizeof(TBlueAPI_MCL)*otp_str_data.gEfuse_UpperStack_s.num_link_don);
    pBlueAPIData->pMCLDescriptorTableDoff = osMemoryAllocate(RAM_TYPE_DATA_OFF,
        sizeof(TBlueAPI_MCL)*otp_str_data.gEfuse_UpperStack_s.num_link_doff);

    pBlueAPIData->pMDLDescriptorTableDon = osMemoryAllocate(RAM_TYPE_DATA_ON,
        sizeof(TBlueAPI_MDL)*otp_str_data.gEfuse_UpperStack_s.num_link_don);
    pBlueAPIData->pMDLDescriptorTableDoff = osMemoryAllocate(RAM_TYPE_DATA_OFF,
        sizeof(TBlueAPI_MDL)*otp_str_data.gEfuse_UpperStack_s.num_link_doff);

    osCoroutineCreate(&blueAPIQueueID, (TCoroutineEntry)blueAPIEntry,(void *)pBlueAPIData);

    osPoolCreate(&pBlueAPIData->usBlueAPIPoolID, RAM_TYPE_DATA_OFF, TRUE, sizeof(TBlueAPI_UsMessage),
        (otp_str_data.gEfuse_UpperStack_s.num_link_don + otp_str_data.gEfuse_UpperStack_s.num_link_doff) * 3 + 3, 4);

    osPoolCreate(&pBlueAPIData->dsBlueAPIPoolID, RAM_TYPE_DATA_OFF, TRUE, sizeof(TBlueAPI_DsMessage),
        (otp_str_data.gEfuse_UpperStack_s.num_link_don + otp_str_data.gEfuse_UpperStack_s.num_link_doff) * 3 + 2, 4);

    PBlueAPI_DS pBlueAPI_DS = &pBlueAPIData->DataStore;

    uint16_t nvSize;
    uint16_t offset = offsetof(TBlueAPI_DSNVData, dataDyn);

    /* blueAPIStore_GetNVDataSize() need these values */
    pBlueAPI_DS->peerCount     = 4;
    pBlueAPI_DS->entriesSCount = 16;
    pBlueAPI_DS->entriesLCount = 8;
    nvSize                     = blueAPIStore_GetNVDataSize(pBlueAPIData);

    if ((pBlueAPI_DS->pNVData = osMemoryClearAllocate(RAM_TYPE_DATA_ON, nvSize)) == 0)
    {
        BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE, "!!BTMOS: failed to allocate %d bytes", nvSize);

        /* send ActInfo when application registers */
        pBlueAPIData->stateCause = blueAPI_CauseInvalidParameter;
        return;
    }

    /*updata point of pPeers, pEntriesS, pEntriesL*/
    pBlueAPI_DS->pPeers = (PBlueAPI_DSPeer)((uint8_t *)(pBlueAPI_DS->pNVData) + offset);
    offset += sizeof(TBlueAPI_DSPeer) * pBlueAPI_DS->peerCount;

    pBlueAPI_DS->pEntriesS = (PBlueAPI_DSEntryS)((uint8_t *)(pBlueAPI_DS->pNVData) + offset);
    offset += sizeof(TBlueAPI_DSEntryS) * pBlueAPI_DS->entriesSCount;

    pBlueAPI_DS->pEntriesL = (PBlueAPI_DSEntryL)((uint8_t *)(pBlueAPI_DS->pNVData) + offset);
    offset  += sizeof(TBlueAPI_DSEntryL) * pBlueAPI_DS->entriesLCount;

    blueAPI_Main(pBlueAPIData);
    BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE, "blueAPI: ...done!");
}


