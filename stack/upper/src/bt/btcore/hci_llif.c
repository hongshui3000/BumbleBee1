/**
*****************************************************************
*	Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************
* @file       hci_llif.c
* @brief     hci layer interface
* @details   
*
* @author   	gordon
* @date      	2015-06-25
* @version	v0.1
*/

#include <flags.h>
#include <os_message.h>
#include <os_pool.h>
#include <os_queue.h>
#include <os_intr.h>
#include <message.h>
#include <hci_code.h>
#include <hci_api.h>

#include <trace_binary.h>
#include <hci.h>
#include <blueapi_api.h>
#include <upper_stack_global.h>
#include <hci_llif.h>

#if UPPER_STACK_USE_VIRTUAL_HCI
#include <comapi.h>
#endif

#if UPPER_STACK_USE_NORMAL_HCI
#include <hci_transport_uart.h>
#endif

#define TRACE_MODULE_ID     MID_BT_HCI

#define HCI_LLAPI_RESULT_SUCCESS      0
#define HCI_LLAPI_RESULT_NO_RESOURCE  1
#define HCI_LLAPI_RESULT_UNKNOWN      2

/**
* @brief  write data to bt controller
* 		queue out data from llDeferredQueue
*		use com write to write data to controller
* 
* @param
*
* @return 
*
*/  
void hciLLTryToWrite(void)
{
	int s;

	s = osInterruptDisable();
	if (pHCI->llDeferredQueue.Count)
	{
		PHciLLQueueData pHciLLQueueData;
		uint8_t *          pPacket;
		uint32_t           Result = HCI_LLAPI_RESULT_UNKNOWN;
        uint16_t            length;

		pHciLLQueueData = (PHciLLQueueData)pHCI->llDeferredQueue.First;
		pPacket         = pHciLLQueueData->DataCB.BufferAddress +
		                  pHciLLQueueData->DataCB.Offset;

#if UPPER_STACK_USE_NORMAL_HCI
        length = pHciLLQueueData->DataCB.Length - 1;
        HCI_TRACE_PRINTF_2(HCI_TRACE_MASK_TRACE, "HCI_TX:pPacket(%p), length(%d)",
                           pPacket, length);
        switch (pPacket[0])
#endif

#if UPPER_STACK_USE_VIRTUAL_HCI
        HCI_TRACE_PRINTF_2(HCI_TRACE_MASK_TRACE, "HCI_TX:pPacket(%p), length(%d)",
                           pPacket, pHciLLQueueData->DataCB.Length);
        switch (pHciLLQueueData->DataCB.Pkt_type)
#endif
        {
        case CMD_PKT:
            HCI_TRACE_BINARY_DOWNSTREAM(TRACE_PKT_TYPE_BT_SNOOP_HCI_CMD, length, &pPacket[1]);
            break;
        case ACL_PKT:
            HCI_TRACE_BINARY_DOWNSTREAM(TRACE_PKT_TYPE_BT_SNOOP_HCI_ACL_TX, length, &pPacket[1]);
            break;
        default:
            break;
        }
#if UPPER_STACK_USE_VIRTUAL_HCI
        pHCI->bWritePending = TRUE;

        TComResult ApiResult = comWrite(pHciLLQueueData->DataCB.Pkt_type,
                                 (void *)pPacket,
                                 pHciLLQueueData->DataCB.Length);
        pHCI->bWritePending = FALSE;
#endif

#if UPPER_STACK_USE_NORMAL_HCI
		TComResult ApiResult = comWrite((HCOM)pHCI->llHandle, (void *)pPacket, pHciLLQueueData->DataCB.Length);
#endif

		if (ApiResult == comResultNoResources)
		{
			Result = HCI_LLAPI_RESULT_NO_RESOURCE;
		}
		else if (ApiResult == comResultSuccess)
		{
			Result = HCI_LLAPI_RESULT_SUCCESS;
		}

		if (Result != HCI_LLAPI_RESULT_NO_RESOURCE)
		{
			pHciLLQueueData = (PHciLLQueueData)osQueueOut(&pHCI->llDeferredQueue);
			osQueueIn(&pHCI->llDeferredQueueFree, pHciLLQueueData);

			if (Result != HCI_LLAPI_RESULT_SUCCESS)
			{
				HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_TRACE,"!! llApiWrite failed (%d)", Result);
				DebuggerBreak();
				osBufferRelease(pHciLLQueueData->DataCB.BufferAddress);
			}
		}
	}
	osInterruptEnable(s);
}

/**
* @brief  release buf and try to write again
*		 release buf maybe call callback again
*
* @param  pBuffer: transmit data buf
*
* @return  
*
*/
void hciLLReleaseBuffer(uint8_t * pBuffer)
{
	uint8_t * p;

	/*p is point to buffer address*/
	memcpy((uint8_t *) &p, ((uint8_t *) pBuffer) - sizeof(uint8_t *), sizeof(uint8_t *));
	if (p)
	{
		osBufferRelease(p);
	}

#if UPPER_STACK_USE_VIRTUAL_HCI
    if(pHCI->bWritePending == TRUE)
    {
        HCI_TRACE_PRINTF_0(HCI_TRACE_MASK_ERROR, "!!!hciLLReleaseBuffer: bWritePending");
        return;
    }
#endif

	hciLLTryToWrite();    /* try to write again */
}

/**
* @brief  handle com open complete 
* 
* @param
*
* @return  
*
*/
BOOL hciLLApiHandleOpenCompleted(void)
{
	pHCI->status	= HCI_STATUS_INITIALIZING;
	pHCI->initPhase = HCI_INIT_STATE_REREAD_LOCAL_VERSION - 1;
	hciNextInitCommand();
	hciLaterEntry();
	return(TRUE);
}

/**
* @brief  handle error from hci layer
* 
* @param
*
* @return  
*
*/
BOOL hciLLApiHandleErrorIndication(void)
{
	HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_ERROR, "!!HCI: ERROR IND %d", BT_ERROR_IND_NOSYNC);

	hciNotInSync();
	hciLaterEntry();
	
	return(TRUE);
}

/**
* @brief  handle data indicate from hci layer
* 
* @param  pLlBuffer: data buff
* @param  Length: buff length
*
* @return  
*
*/
#if UPPER_STACK_USE_VIRTUAL_HCI
BOOL hciLLApiHandleDataIndication(PHCI pHCI, uint8_t * pLlBuffer, uint32_t Length, uint8_t type)
#endif

#if UPPER_STACK_USE_NORMAL_HCI
BOOL hciLLApiHandleDataIndication(uint8_t * pLlBuffer, uint32_t Length)
#endif
{
	uint8_t *    pBuffer;
	MESSAGE_T Message;
	uint16_t      PoolID;
	uint16_t      Offset;
    uint8_t BtSnoopyType = 0;

	switch (pLlBuffer[0])              /* First byte is packet type */
	{
	case ACL_PKT:
        BtSnoopyType = TRACE_PKT_TYPE_BT_SNOOP_HCI_ACL_RX;
		PoolID = UpstreamPoolID;
		Offset = pHCI->ReadOffset - 1;
		break;

	case EVENT_PKT:
        BtSnoopyType = TRACE_PKT_TYPE_BT_SNOOP_HCI_EVT;
		PoolID = BTSystemPoolID;
		Offset = 0;
		break;

	default:
		DebuggerBreak();
		return(FALSE);
	}

	if (osBufferGet(PoolID, Offset + Length, (PVOID *)&pBuffer))
	{
		return(FALSE);
	}
	memcpy((PVOID)&pBuffer[Offset], (PVOID)pLlBuffer, Length);
    
    HCI_TRACE_BINARY_UPSTREAM(BtSnoopyType, (Length - 1), &pLlBuffer[1]);

	Message.Command        = PH_DATA_IND;
	Message.MData.DataCBExt.DataCB.BufferAddress = pBuffer;
	Message.MData.DataCBExt.DataCB.Offset        = Offset;
#if UPPER_STACK_USE_VIRTUAL_HCI
    Message.MData.DataCBExt.DataCB.Pkt_type = type;
#endif

    Message.MData.DataCBExt.DataCB.Length        = Length;             /* with packet type */
    Message.MData.DataCBExt.DataCB.Flag          = DATA_CB_RELEASE;
    Message.MData.DataCBExt.Handle.lpHandle      = (PVOID)pLlBuffer;  /* save for response */

    osMessageSend(hciQueueID, &Message);

    return(TRUE);
}


/**
* @brief  call back by hci layer
* 
* @param  pContext
* @param  Event: show what happened
* @param  EventResult: need to be comResultSucess
* @param  pBuffer: recv data or trans data
* @param  Length: length of pBuffer
*
* @return  
*
*/
#if UPPER_STACK_USE_VIRTUAL_HCI
TComResult hciComApiCallBack(void      *pContext,
                             TComEvent  Event,
                             uint32_t   EventResult,
                             void       *pBuffer,
                             uint32_t   Length,
                             uint8_t pType)
#endif

#if UPPER_STACK_USE_NORMAL_HCI
TComResult hciComApiCallBack(void      *pContext,
                                   TComEvent  Event,
                                   uint32_t   EventResult,
                                   void       *pBuffer,
                                   uint32_t   Length)
#endif
{
	TComResult Result = comResultSuccess;
	PHCI       pHCI = (PHCI)pContext;

	if (EventResult != comResultSuccess)
	{
		HCI_TRACE_PRINTF_2(HCI_TRACE_MASK_TRACE,"hciComApiCallBack: event %d result %d", Event, EventResult);
		DebuggerBreak();
	}

	switch (Event)
	{
	case comeDataIndication:	/*recv data from hci layer*/
        if (EventResult == comResultSuccess)
        {
#if UPPER_STACK_USE_VIRTUAL_HCI
            if (hciLLApiHandleDataIndication(pHCI, (uint8_t *)pBuffer, (uint32_t)Length, pType) == FALSE)
            {
                Result = comResultNoResources;
                hciLLResponse(pBuffer);
                HCI_TRACE_PRINTF_0(HCI_TRACE_MASK_ERROR, "!!hciComApiCallBack: hciLLApiHandleDataIndication failed");
            }
#endif

#if UPPER_STACK_USE_NORMAL_HCI
            if (hciLLApiHandleDataIndication((uint8_t *)pBuffer, (uint32_t)Length) == FALSE)
            {
                Result = comResultNoResources;
            }
#endif
        }
        else
        {
            HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_ERROR, "!!comeDataIndication: EventResult = 0x%x", EventResult);
            DebuggerBreak();
        }
        break;

	case comeDataTransmitted:	/*data trans to hci layer completed*/
		hciLLReleaseBuffer((uint8_t *)pBuffer);
		break;

	case comeError:				/*com error*/
		hciLLApiHandleErrorIndication();
		break;

	case comeOpenCompleted:		/*com open completed*/
		if (EventResult == comResultSuccess)
		{
			hciLLApiHandleOpenCompleted();
		}
		else
		{
			blueAPI_Handle_HCI_ACT_CONF(pHCI->localBdAddr, hciStatus(HCI_ERR_TIMEOUT));
		}
		break;

	case comeCloseCompleted:	/*com close completed*/
		break;

    default:
        break;
	}
	return(Result);
}

/**
* @brief  open lower layer
* 
* @param
* @return  
*
*/
void hciLLOpen(void)
{
	int i;

	pHCI->llDeferredQueueFree.First        = NULL;
	pHCI->llDeferredQueueFree.Last         = NULL;
	pHCI->llDeferredQueueFree.Count = 0;

	for (i=0; i < HCI_LLAPI_QUEUE_SIZE; i++)
	{
		osQueueIn(&pHCI->llDeferredQueueFree, &pHCI->llDeferredQueueData[i]);
	}
#if UPPER_STACK_USE_VIRTUAL_HCI
    TComResult Result = comOpen( hciComApiCallBack, (void *)pHCI);
#endif

#if UPPER_STACK_USE_NORMAL_HCI
    TComResult Result = comOpen("", "", hciComApiCallBack, (void *)pHCI, (HCOM *)&pHCI->llHandle);
#endif

    if (Result != comResultPending)
	{
		HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_TRACE,"!! comOpenOpen failed (%d)", Result);
		DebuggerBreak();
	}
}

/**
 * @brief  close interface
 *
* @param
*
* @return
*
*/
void hciLLClose(void)
{
	TComResult Result;
#if UPPER_STACK_USE_NORMAL_HCI
    Result = comClose((HCOM)pHCI->llHandle);
#endif

#if UPPER_STACK_USE_VIRTUAL_HCI
    Result = comClose();
#endif
    if (Result != comResultPending)
	{
		HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_TRACE,"!! comClose failed (%d)", Result);
		DebuggerBreak();
	}
}

/**
* @brief  try to write msg from hci layer
* 
* @param  pMsg:  msg
*
* @return 
*
*/  
void hciLLWrite(MESSAGE_P pMsg)
{
	PHciLLQueueData pHciLLQueueData;

	pHciLLQueueData = (PHciLLQueueData)osQueueOut(&pHCI->llDeferredQueueFree);
	if (pHciLLQueueData != (PHciLLQueueData)0)
	{
		int   s;
		uint8_t * p;

		p = pMsg->MData.DataCB.BufferAddress + pMsg->MData.DataCB.Offset;
		/*Copy buff address to the place before offset*/
		memcpy((PVOID)(p - sizeof(uint8_t *)), (PVOID)&pMsg->MData.DataCB.BufferAddress, sizeof(uint8_t *));

		pHciLLQueueData->DataCB = pMsg->MData.DataCB;

		s = osInterruptDisable();
		osQueueIn(&pHCI->llDeferredQueue, pHciLLQueueData);
		osInterruptEnable(s);

		hciLLTryToWrite();
	}
	else
	{
		HCI_TRACE_PRINTF_0(HCI_TRACE_MASK_TRACE,"!! hciLLWrite: no resource");
		DebuggerBreak();

		osBufferRelease(pMsg->MData.DataCB.BufferAddress);
	}
}

#if UPPER_STACK_USE_VIRTUAL_HCI
void hciLLResponse(PVOID pBuffer)
{
    TComResult Result;
    HCI_TRACE_PRINTF_0(HCI_TRACE_MASK_TRACE, "hciLLResponse");

    Result = comResponse((uint8_t *)pBuffer);

    if (Result != comResultSuccess)
    {
        HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_ERROR, "!!comResponse (%d)", Result);
        DebuggerBreak();
    }

}
#endif



