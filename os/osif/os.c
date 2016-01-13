/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       os.c
* @brief     OS interface without SRTX-Task
* @details   
*
* @author   	gordon
* @date      	2015-06-29
* @version	v0.1
*/

#include <flags.h>
#include <pool.h>
#include <os.h>
#include <osif.h>
#include <os_pool.h>
#include <os_queue.h>
#include <os_mem.h>
#include <os_intr.h>
#include <os_message.h>
#include <bluemgr.h>
#include <btsm_api.h>
#include <hci_api.h>
#include <l2c_api.h>
#include <sdp_api.h>
#include <gatt_api.h>
#include <blueapi_osif.h>
#include <swtimer.h>
#include <upper_stack_global.h>
#include <trace_binary.h>

#define TRACE_MODULE_ID      MID_OS


void createBTSystemPool(void)
{
	/*create system short buf*/
#if (BT_SYS_SHORT_BUFFER_COUNT > 0)
	if (osPoolCreate(&BTSystemPoolID, RAM_TYPE_DATA_OFF, TRUE, BT_SYS_SHORT_BUFFER_SIZE,
        BT_SYS_SHORT_BUFFER_COUNT, 4))
	{
		DebuggerBreak();
		return;
	}
#endif

	/*create system middle buf*/
#if (BT_SYS_MIDDLE_BUFFER_COUNT > 0)
#if (BT_SYS_SHORT_BUFFER_COUNT > 0)
	osPoolExtend(BTSystemPoolID, BT_SYS_MIDDLE_BUFFER_SIZE, BT_SYS_MIDDLE_BUFFER_COUNT);
#else
	if (osPoolCreate(&BTSystemPoolID, RAM_TYPE_DATA_OFF, TRUE, BT_SYS_MIDDLE_BUFFER_SIZE,
        BT_SYS_MIDDLE_BUFFER_COUNT, 4))
	{
		DebuggerBreak();
		return;
	}
#endif
#endif

	/*create system long buf*/
#if (BT_SYS_LONG_BUFFER_COUNT > 0)
#if (BT_SYS_SHORT_BUFFER_COUNT > 0) || (BT_SYS_MIDDLE_BUFFER_COUNT > 0)
	osPoolExtend(BTSystemPoolID, BT_SYS_LONG_BUFFER_SIZE, BT_SYS_LONG_BUFFER_COUNT);
#else
	if (osPoolCreate(&BTSystemPoolID, RAM_TYPE_DATA_OFF, TRUE, BT_SYS_LONG_BUFFER_SIZE,
        BT_SYS_LONG_BUFFER_COUNT, 4))
	{
		DebuggerBreak();
		return;
	}
#endif
#endif /* (BT_SYS_LONG_BUFFER_COUNT > 0) */
}

bool CreateBTDataStreamPool(void)
{
	uint8_t 				PoolID;

    if(otp_str_data.gEfuse_UpperStack_s.DownstreamLongBufferCount)
	{
		if (osPoolCreate(&PoolID, RAM_TYPE_DATA_OFF, TRUE, otp_str_data.gEfuse_UpperStack_s.DownstreamLongBufferLength,
            otp_str_data.gEfuse_UpperStack_s.DownstreamLongBufferCount, 4))
		{
			return(FALSE);
		}
        DownstreamPoolID = PoolID;
	}

    if (otp_str_data.gEfuse_UpperStack_s.UpsteamShortBufferCount)
    {
        if (osPoolCreate(&PoolID, RAM_TYPE_DATA_OFF, TRUE, otp_str_data.gEfuse_UpperStack_s.UpsteamShortBufferLength,
            otp_str_data.gEfuse_UpperStack_s.UpsteamShortBufferCount, 4))
        {
            return (FALSE);
        }
        UpstreamPoolID = PoolID;

    }

    if (otp_str_data.gEfuse_UpperStack_s.UpstreamLongBufferCount)
    {
        if (UpstreamPoolID == 0xFF)
        {
            if (osPoolCreate(&PoolID, RAM_TYPE_DATA_OFF, TRUE, otp_str_data.gEfuse_UpperStack_s.UpstreamLongBufferLength,
                otp_str_data.gEfuse_UpperStack_s.UpstreamLongBufferCount, 4))
            {
                return (FALSE);
            }
            UpstreamPoolID = PoolID;
        }
        else
        {
            if (osPoolExtend(UpstreamPoolID, otp_str_data.gEfuse_UpperStack_s.UpstreamLongBufferLength,
                otp_str_data.gEfuse_UpperStack_s.UpstreamLongBufferCount))
                return (FALSE);
        }
    }

    return TRUE;
}

extern void hciActivateStack(void);

void StartModuleStack(void)
{
    btsmInit(); 
    
    lblueFaceInit();
    hciInit();
    l2cInit();
    sdpInit();
    gattInit();

    hciActivateStack();

    blueAPIInit();
}

int osInit(void)
{
    PMessageQueueElement    pMessageQueueElement;
    PEntryQueueElement      pEntryQueueElement;
    uint16_t                i;

    efuse_config_load();

    os.FreeMessageQueueSize = 18;
    os.FreeEntryQueueSize = 18;

    if ((otp_str_data.gEfuse_UpperStack_s.att_max_mtu_size   > 23) || (otp_str_data.gEfuse_UpperStack_s.num_l2c_chan_don != 0)||(otp_str_data.gEfuse_UpperStack_s.num_l2c_chan_doff != 0)) //23 is ATT_MTU_SIZE_LE_DEFAULT
    {
        //os.QueueTableSize = 14 + otp_str_data.gEfuse_UpperStack_s.num_link_don + otp_str_data.gEfuse_UpperStack_s.num_link_doff + (otp_str_data.gEfuse_UpperStack_s.num_link_don + otp_str_data.gEfuse_UpperStack_s.num_link_doff) * 2; //14+HCI+ L2CAP Fragment queue*2
        os.OsQueueTableSize = 32;
    }
    else
    {
        //os.QueueTableSize = 14 + otp_str_data.gEfuse_UpperStack_s.num_link_don + otp_str_data.gEfuse_UpperStack_s.num_link_doff;//14+HCI+ L2CAP*2
        os.OsQueueTableSize = 32;
    }

    if (otp_str_data.gEfuse_UpperStack_s.bqb_en)
    {
        otp_str_data.gEfuse_UpperStack_s.gatt_max_service_count = 12;
    }
    else
    {
        otp_str_data.gEfuse_UpperStack_s.gatt_max_service_count = 8;
    }

    os.OsQueueTable = osMemoryClearAllocate(RAM_TYPE_DATA_ON, sizeof(TOsQueue) * os.OsQueueTableSize);

    pMessageQueueElement = osMemoryClearAllocate(RAM_TYPE_DATA_OFF, sizeof(TMessageQueueElement) * os.FreeMessageQueueSize);
    pEntryQueueElement = osMemoryClearAllocate(RAM_TYPE_DATA_OFF, sizeof(TEntryQueueElement) * os.FreeEntryQueueSize);

    for (i = 0; i < os.FreeMessageQueueSize; i++)
    {
        osQueueIn(&os.FreeMessageQueue, pMessageQueueElement++);
    }

    for (i = 0; i < os.FreeEntryQueueSize; i++)
    {
        osQueueIn(&os.FreeEntryQueue, pEntryQueueElement++);
    }

    for (i = 0; i < os.OsQueueTableSize; i++)
    {
        os.OsQueueTable[i].Flags = OS_QUEUE_FREE;
    }

    createBTSystemPool();
    CreateBTDataStreamPool();

    swInit();
    traceInit();

    StartModuleStack();

    return RET_OK;
}

void osStart(void)
{
    osReschedule();
    return;
}
