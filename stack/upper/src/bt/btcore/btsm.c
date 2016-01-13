/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       btsm.c
* @brief     bt security manager
* @details   
*
* @author   	gordon
* @date      	2015-06-29
* @version	v0.1
*/

#include <flags.h>
#include <rtl_types.h>
#include <message.h>
#include <btsm.h>
#include <btsmprot.h>
#include <upper_stack_global.h>
#include <os_message.h>
#include <os_mem.h>

#define TRACE_MODULE_ID     MID_BTSECMAN

void  btsmEntry(TBtSM  *lpBtSM)
{
    if (osMessageReceive(btsmQueueID, &pBtSM->message) == 0)
    {
        btsmHandleMessage();
    }
}
/**
* @brief  init bt security module
* 
* @param  
* @return 
*
*/
int  btsmInit(void)
{
	pBtSM = osMemoryClearAllocate(RAM_TYPE_DATA_ON, sizeof(TBtSM));

	if (osCoroutineCreate(&btsmQueueID, (TCoroutineEntry)btsmEntry, (void *)pBtSM))
	{
		return(FALSE);
	}

	osMessageQueueCreate(&pBtSM->deferredQueueID);
	osMessageQueueCreate(&pBtSM->cryptQueueID);

	pBtSM->cryptHandle = LE_CRYPT_HANDLE_IDLE;
	pBtSM->LE_maxKeySize = 16;
	pBtSM->LE_fixedDisplayValue = 0xFFFFFFFF;

    pBtSM->plinksDon = osMemoryAllocate(RAM_TYPE_DATA_ON, sizeof(TSECLINK)*otp_str_data.gEfuse_UpperStack_s.num_link_don);
    pBtSM->plinksDoff = osMemoryAllocate(RAM_TYPE_DATA_OFF, sizeof(TSECLINK)*otp_str_data.gEfuse_UpperStack_s.num_link_doff);

	return(TRUE);
}

