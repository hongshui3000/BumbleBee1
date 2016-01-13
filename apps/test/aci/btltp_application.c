enum { __FILE_NUM__ = 0 };
/**
*********************************************************************************************************
*               Copyright(c) 2014, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file      btltp_FreeRTOS.c
* @brief     aci iterface implementation.
* @details   none.
* @author    Tifnan
* @date      2014-10-17
* @version   v0.1
* *********************************************************************************************************
*/

#include <blueapi_types.h>
#include <blueapi.h>
#include "aci_if.h"
#include "btltp.h"
//#include "dlps_platform.h"
#include "aci_low_power.h"
#include "aci_low_power_utils.h"
#include "trace.h"
#include "aci_service_handle.h"
#include "btltp_uart.h"
#include <os_mem.h>

/* task  */
#define LTP_PRIORITY                    (tskIDLE_PRIORITY + 2)   /* Task priorities. */
#define LTP_TASK_STACK_SIZE             0x0600
#define TX_TASK_STACK_SIZE              0x200

#define MAX_NUMBER_OF_RX_EVENT          0x20
#define MAX_NUMBER_OF_MESSAGE           0x20
#define MAX_NUMBER_OF_TX_DATA           12
#define MAX_NUMBER_OF_TX_REL            MAX_NUMBER_OF_TX_DATA
#define MAX_NUMBER_OF_RX_DATA           12

ACI_TCB AciTcb;
TBTLtp  BtLtp;
PBTLtp  P_BtLtp = NULL;

//SRAM_OFF_BD_DATA_SECTION
//uint8_t RxBuffer[RX_BUFFER_SIZE];
//SRAM_OFF_BD_DATA_SECTION
//uint8_t TXBuffer[TX_BUFFER_SIZE];
//SRAM_OFF_BD_DATA_SECTION
//uint8_t RXHandleBuffer[RX_HANDLE_BUFFER_SIZE];

xTaskHandle LTPTaskHandle = NULL;
xTaskHandle LTPTxAssistHandle = NULL;
extern uint32_t g_eflash_dbg;
void ltpTask(void *pParameters);

/**
 * @brief send event to ltp task.
 *
 * @param pEvent, pointer to the event to be sent.
 * @return send result.
 * @retval pdPASS--send successfully.
 *         errQUEUE_FULL-- queue is full.
*/
portBASE_TYPE ltpSendEvent(const unsigned char *pEvent)
{
    portBASE_TYPE ReturnValue;

    ReturnValue = xQueueSend(P_BtLtp->p_aci_tcb->QueueHandleEvent, pEvent, 0);
    return (ReturnValue);
}

/**
 * @brief handle message from upper stack.
 *
 * @param pMsg --message pointer from upper stack.
 * @return none.
 * @retal void
*/
void ltpHandleBlueAPIMessage(PBlueAPI_UsMessage pMsg)
{
    switch (pMsg->Command)
    {
    case blueAPI_EventActInfo: /*-------------------------------------------*/
        P_BtLtp->LTP_US_OfflinePoolID = pMsg->p.ActInfo.systemPoolID;
        break;

    default: /*-------------------------------------------------------------*/
        break;
    }

    BTLTPHandleBLUE_API_MSG(P_BtLtp, (uint8_t *)pMsg, 0);

    blueAPI_BufferRelease(pMsg);

}


/**
 * @brief callback function, upper stack will call it to send message to ltp.
 *
 * @param pMsg --message pointer from upper stack.
 * @return none.
 * @retal void
*/
void ltpBlueAPICallback(PBlueAPI_UsMessage pMsg)
{
    unsigned char Event = LTP_EVENT_BLUEAPI_MESSAGE;

    if (xQueueSend(P_BtLtp->p_aci_tcb->QueueHandleMessage, &pMsg, 0) == errQUEUE_FULL)
    {

        DBG_BUFFER(MODULE_LTP, LEVEL_ERROR, "ltpBlueAPICallback: MessageQueue full", 0);

        blueAPI_BufferRelease(pMsg);
    }
    else
    {
        if (ltpSendEvent(&Event) == errQUEUE_FULL)     /* signal event to GATTDEMO task */
        {
            DBG_BUFFER(MODULE_LTP, LEVEL_ERROR, "ltpBlueAPICallback: EventQueue full", 0);
        }
    }
}


/**
 * @brief time out routine, not used now .
 *
 * @param xTimer --time handle.
 * @return none.
 * @retal void
*/
#if F_LTPLIB_ASYNC_ASSEMBLY_SUPPORT
void ltpTimeoutRoutine(xTimerHandle xTimer)
{
    P_BtLtp->p_aci_tcb->ltpTimerMS = 0;
    LTPLibHandleTimeout(&P_BtLtp->LTPLib, (TLTPTimerID)P_BtLtp->p_aci_tcb->ltpTimerID);
}

/**
 * @brief start lp timer, not used in ltp now.
 *
 * @param TimerID time id.
 * @param TimerMS the time setting of this timer.
 * @return none.
 * @retal   void.
*/
void ltpTimerStart(TLTPTimerID TimerID, int TimerMS)        /* tifnan:not used now */
{
    if (P_BtLtp->p_aci_tcb->ltpTimerMS != 0)     /* timer started */
    {
        xTimerStop(P_BtLtp->p_aci_tcb->ltpTimerHandle, 0);
    }
    P_BtLtp->p_aci_tcb->ltpTimerID = TimerID;
    P_BtLtp->p_aci_tcb->ltpTimerMS = TimerMS;
    xTimerChangePeriod(P_BtLtp->p_aci_tcb->ltpTimerHandle, (TimerMS / portTICK_RATE_MS), 0);
    xTimerStart(P_BtLtp->p_aci_tcb->ltpTimerHandle, 0);

    return;
}
#endif

/**
 * @brief dlps callback function, used when bee is to enter dlps .
 *
 * @param none.
 * @return the check result.
 * @retal  TRUE -- can enter dlps.
            FALSE --can not enter dlps.
*/
#if 0
BOOL LtpDlpsEnterCheck(void)
{
    /* check all queues are all empty */
    if (MAX_NUMBER_OF_RX_EVENT != uxQueueSpacesAvailable(P_BtLtp->p_aci_tcb->QueueHandleEvent)
            && MAX_NUMBER_OF_TX_DATA != uxQueueSpacesAvailable(P_BtLtp->p_aci_tcb->QueueHandleTxData)
            && MAX_NUMBER_OF_RX_DATA != uxQueueSpacesAvailable(P_BtLtp->p_aci_tcb->QueueHandleRxData)
            && MAX_NUMBER_OF_TX_REL != uxQueueSpacesAvailable(P_BtLtp->p_aci_tcb->QueueHandleTxRel)
            && MAX_NUMBER_OF_MESSAGE != uxQueueSpacesAvailable(P_BtLtp->p_aci_tcb->QueueHandleMessage))
    {
        DBG_BUFFER(MODULE_LTP, LEVEL_INFO, "Ltp queue is not empty!", 0);
        return FALSE;
    }
    else
    {
        return TRUE;
    }
}
#endif
/**
 * @brief call this fucntion will start to send data through ltp.
 * @param p_buf pointer to the buffer start address.
 * @param buf_len the length of the buffer.
 * @return none.
 * @retal   void.
*/
void LtpWrite(uint8_t *p_buf, uint32_t buf_len)
{
    TLTPData tx_data;
    unsigned char ltpEvent  = LTP_EVENT_UART_TX;
    tx_data.pBuffer       = p_buf;
    tx_data.Length        = buf_len;

    /* host support low power mode & host in low power mode */
#if 0
    if (HostPwrMode == LTPLL_LPWR_ON)
    {
        if(LTPLL_IsHostSleep())
        {
            DBG_BUFFER(MODULE_LTP, LEVEL_INFO, "LtpWrite: host in lps, store buffer: 0x%x, length: %d", \
                       2, p_buf, buf_len);
            /* save data */
            LTPLL_QueueSave(p_buf, buf_len);
            /* wake up host firstly */
            LTPLL_WakeUpHost();
            /* stop moniter timer, can not enter dlps when wait for ack from host */
            if (MoniterTimer)
            {
                xTimerStop(MoniterTimer, 5);
                MonitorTimeout = 0;
            }
            LTPLL_SetPowerStatus(LTPLL_STATE_W4ACK); 
        }
        else if(LTPLL_GetPowerStatus() == LTPLL_STATE_PRE_SLEEP)
        {
            /* bee is wait host goio low!!!! just save data */
            LTPLL_QueueSave(p_buf, buf_len);
        }
        else
        {
            //send directly
            if (pdFALSE == xQueueSend(P_BtLtp->p_aci_tcb->QueueHandleTxData, &tx_data, 0))
            {
                DBG_BUFFER(MODULE_LTP, LEVEL_ERROR, "LtpWrite:xQueueSend fail", 0);
            }
        }
    }
    else    /* host not support low power mode, or host in wake status, send directly */
    {
#endif
        if (pdFALSE == xQueueSend(P_BtLtp->p_aci_tcb->QueueHandleTxData, &tx_data, 0))
        {
            DBG_BUFFER(MODULE_LTP, LEVEL_ERROR, "LtpWrite:xQueueSend fail", 0);
        }
        ltpSendEvent(&ltpEvent);
#if 0
    }
#endif
    return;
}


/**
 * @brief release ltp buffer when ltp command has executed completely.
 *
 * @param p_buf the buffer start address to release, not used now!!
 * @return none.
 * @retal   void.
*/
void LtpBufferRelease(void* p_buf)
{
    TLTPData RxData;

    if (xQueueReceive(P_BtLtp->p_aci_tcb->QueueHandleRxData, &RxData, 0) == pdPASS)
    {
        if (RxData.pBuffer == &P_BtLtp->p_aci_tcb->p_rx_buf[P_BtLtp->p_aci_tcb->RxReadIndex])
        {
            uint32_t RxDataLength;

            taskDISABLE_INTERRUPTS();
            P_BtLtp->p_aci_tcb->RxDataLength -= RxData.Length;
            RxDataLength           = P_BtLtp->p_aci_tcb->RxDataLength;
            taskENABLE_INTERRUPTS();

            P_BtLtp->p_aci_tcb->RxReadIndex += RxData.Length;
            P_BtLtp->p_aci_tcb->RxReadIndex &= (RX_BUFFER_SIZE - 1);

            if (P_BtLtp->p_aci_tcb->RxDataIndication)   /* waiting for response */
            {
                P_BtLtp->p_aci_tcb->RxDataIndication -= RxData.Length;
            }

            if (P_BtLtp->p_aci_tcb->RxDataIndication == 0 &&   /* no response pending and */
                    RxDataLength != 0)                         /* still data available */
            {
                uint8_t event = LTP_EVENT_UART_RX;
                if ( errQUEUE_FULL == ltpSendEvent(&event) )
                    DBG_BUFFER(MODULE_LTP, LEVEL_ERROR, "ltpSendEvent fail", 0);
            }

        }
        else
        {
            DBG_BUFFER(MODULE_LTP, LEVEL_ERROR, "LtpBufferRelease: Wrong buffer", 0);
        }
    }
    else
    {
        DBG_BUFFER(MODULE_LTP, LEVEL_INFO, "LtpBufferRelease: No RxData", 0);
    }

    return;
}

/**
 * @brief init ltp module, call this function before calling other ltp functions .
 *
 * @param none.
 * @return the init result.
 * @retal  0 -- init ltp failed.
            1 -- init ltp successfully.
*/
uint8_t ltpInit(void)
{
    //g_eflash_dbg = 0;
    P_BtLtp = &BtLtp;
    /* memset */
    memset(P_BtLtp, 0, sizeof(TBTLtp));
    P_BtLtp->p_aci_tcb = & AciTcb;
    memset(P_BtLtp->p_aci_tcb, 0, sizeof(ACI_TCB));
    P_BtLtp->p_aci_tcb->p_rx_buf = osMemoryClearAllocate(RAM_TYPE_DATA_ON, RX_BUFFER_SIZE);
    memset(P_BtLtp->p_aci_tcb->p_rx_buf, 0, RX_BUFFER_SIZE);
    P_BtLtp->p_aci_tcb->p_tx_buf = osMemoryClearAllocate(RAM_TYPE_DATA_ON, TX_BUFFER_SIZE);
    memset(P_BtLtp->p_aci_tcb->p_tx_buf, 0, TX_BUFFER_SIZE);
    P_BtLtp->p_aci_tcb->p_rx_handle_buf = osMemoryClearAllocate(RAM_TYPE_DATA_ON, RX_HANDLE_BUFFER_SIZE);
    memset(P_BtLtp->p_aci_tcb->p_rx_handle_buf, 0, RX_HANDLE_BUFFER_SIZE);

    /* allocate failed */
    if (NULL == P_BtLtp
            || NULL == P_BtLtp->p_aci_tcb
            || NULL == P_BtLtp->p_aci_tcb->p_rx_buf
            || NULL == P_BtLtp->p_aci_tcb->p_tx_buf
            || NULL == P_BtLtp->p_aci_tcb->p_rx_handle_buf)
    {
        DBG_BUFFER(MODULE_LTP, LEVEL_ERROR, "Ltp ram allocated failed!", 0);
        return 0;
    }

    DBG_BUFFER(MODULE_LTP, LEVEL_INFO, "Ltp ram allocated successfully!", 0);
    memset(&(P_BtLtp->p_aci_tcb->tx_mem_tcb), 0, sizeof(TxMemTCB));
    P_BtLtp->p_aci_tcb->tx_mem_tcb.free_size = TX_BUFFER_SIZE;

    /* tasks and queues */
    P_BtLtp->p_aci_tcb->Handle = LTPTaskHandle;
    P_BtLtp->p_aci_tcb->P_RxBuffer = &P_BtLtp->p_aci_tcb->p_rx_buf[0];
    P_BtLtp->p_aci_tcb->QueueHandleEvent   = xQueueCreate(MAX_NUMBER_OF_RX_EVENT, sizeof(unsigned char));
    P_BtLtp->p_aci_tcb->QueueHandleTxData  = xQueueCreate(MAX_NUMBER_OF_TX_DATA, sizeof(TLTPData));
    P_BtLtp->p_aci_tcb->QueueHandleRxData  = xQueueCreate(MAX_NUMBER_OF_RX_DATA, sizeof(TLTPData));
    P_BtLtp->p_aci_tcb->QueueHandleTxRel  = xQueueCreate(MAX_NUMBER_OF_TX_REL, sizeof(TLTPData)); /* tx release */
    P_BtLtp->p_aci_tcb->QueueHandleMessage = xQueueCreate(MAX_NUMBER_OF_MESSAGE, sizeof(PBlueAPI_DsMessage));
#if F_LTPLIB_ASYNC_ASSEMBLY_SUPPORT
    P_BtLtp->p_aci_tcb->ltpTimerHandle = xTimerCreate("TIM_LTP", (10 / portTICK_RATE_MS), pdFALSE, 0, ltpTimeoutRoutine);
#endif


    for (uint16_t i = 0; i < BT_GATT_SERVER_MAX_SERVICES_COUNT; i++)
    {
        P_BtLtp->gattServiceTable[i].self_idx = i;
    }
    fs_init(P_BtLtp);

    return 1;
}

uint8_t ltpTaskInit(void)
{
    xTaskCreate(ltpTask, "BTLTP", LTP_TASK_STACK_SIZE / sizeof(portSTACK_TYPE), NULL, LTP_PRIORITY, &LTPTaskHandle);

    //xTaskCreate(TxAssistTask, "TxAssist", TX_TASK_STACK_SIZE / sizeof(portSTACK_TYPE), NULL, LTP_PRIORITY - 1, &LTPTxAssistHandle);
    return 1;
}

/**
 * @brief ltp task implementation .
 *
 * @param pParameters --task parameters, no used in ltp task.
 * @return none.
 * @retal void
*/
void ltpTask(void *pParameters)
{
    int  loop;
    char Event;

    /* init uart or spi */
    ltpPeripheralInit();
    P_BtLtp->State = btltpStateInit;

    for (loop = 0; loop < BTLTP_QUEUE_ELEMENT_COUNT; loop++)
    {
        ltpQueueIn(&P_BtLtp->FreeElementQueue, &P_BtLtp->ElementPool[loop]);
    }

    for (loop = 0; loop < BTLTP_ACTION_POOL_SIZE; loop++)
    {
        P_BtLtp->ActionPool[loop].Action = btltpActionNotUsed;
    }
    P_BtLtp->pBufferAction = NULL; /* no action pending */
    LTPLibInitialize(&P_BtLtp->LTPLib,
                     (LTP_TGT_APPHANDLE)P_BtLtp,
                     0,
                     BTLTP_MAX_MSG_SIZE,
                     0
                    );

    blueAPI_RegisterReq(P_BtLtp, (void *)ltpBlueAPICallback);

    P_BtLtp->State = btltpStateIdle; /* not use !!*/

    while (TRUE)
    {
        if (xQueueReceive(P_BtLtp->p_aci_tcb->QueueHandleEvent, &Event, portMAX_DELAY) == pdPASS)
        {
            switch (Event)
            {
            case LTP_EVENT_UART_RX:          /* RxData available */
                {
                    uint32_t RxDataLength;
                    uint16_t RxReadIndex;

                    taskENTER_CRITICAL();
                    /* skip data filed in handling */
                    RxDataLength = P_BtLtp->p_aci_tcb->RxDataLength - P_BtLtp->p_aci_tcb->RxDataIndication;
                    taskEXIT_CRITICAL();
                    RxReadIndex  = P_BtLtp->p_aci_tcb->RxReadIndex + P_BtLtp->p_aci_tcb->RxDataIndication;
                    RxReadIndex &= (RX_BUFFER_SIZE - 1);

                    while (RxDataLength)
                    {
                        TLTPData RxData;

                        /* exceed rx buffer tail */
                        if ((RxReadIndex + RxDataLength) > RX_BUFFER_SIZE)
                        {
                            RxData.Length = RX_BUFFER_SIZE - RxReadIndex;
                        }
                        else
                        {
                            RxData.Length = RxDataLength;
                        }
                        RxData.pBuffer = &P_BtLtp->p_aci_tcb->p_rx_buf[RxReadIndex];
                        //DBG_BUFFER(MODULE_LTP, LEVEL_TRACE, "LTP_EVENT_UART_RX: RxData.pBuffer = [0x%8x], RxData.Length= 0x%4x\t",\
                        //2, RxData.pBuffer, RxData.Length);
                        if (xQueueSend(P_BtLtp->p_aci_tcb->QueueHandleRxData, &RxData, 0) == pdPASS)
                        {
                            P_BtLtp->p_aci_tcb->RxDataIndication += RxData.Length;
                            /* LTPLibHandleReceiveData return when all data in RxData has been copied */
                            if (!LTPLibHandleReceiveData(&P_BtLtp->LTPLib, RxData.pBuffer, RxData.Length, 0))
                            {
                                LtpBufferRelease(RxData.pBuffer);
                            }

                            RxDataLength -= RxData.Length;
                            RxReadIndex  += RxData.Length;
                            RxReadIndex  &= (RX_BUFFER_SIZE - 1);
                        }
                        else
                        {
                            DBG_BUFFER(MODULE_LTP, LEVEL_ERROR, "LTP_EVENT_UART_RX, xQueueSend fail", 0);
                            break;
                        }
                    }
                    break;
                }
            case LTP_EVENT_UART_TX:                    /* transmit data */
                ltpStartTransmit();
                break;
            case LTP_EVENT_UART_TX_COMPLETED:          /* transmit completed */
                {
                    THandle Handle;
                    TLTPData data;

                    if (xQueueReceive(P_BtLtp->p_aci_tcb->QueueHandleTxRel, &data, 0) == pdPASS)
                    {
                        DBG_BUFFER(MODULE_LTP, LEVEL_INFO, "QueueHandleTxRel cmd:0x%x", 1, \
                                   P_BtLtp->p_aci_tcb->p_tx_buf[P_BtLtp->p_aci_tcb->tx_mem_tcb.tx_blk_idx]);
                        P_BtLtp->p_aci_tcb->tx_mem_tcb.tx_blk_idx += data.Length;
                        P_BtLtp->p_aci_tcb->tx_mem_tcb.free_size += data.Length;

                        if (P_BtLtp->p_aci_tcb->tx_mem_tcb.tx_blk_idx > TX_BUFFER_SIZE)
                        {
                            P_BtLtp->p_aci_tcb->tx_mem_tcb.tx_blk_idx = data.Length;
                            P_BtLtp->p_aci_tcb->tx_mem_tcb.tx_un_used_size = 0;
                        }
                        else if (P_BtLtp->p_aci_tcb->tx_mem_tcb.tx_blk_idx == TX_BUFFER_SIZE)
                        {
                            P_BtLtp->p_aci_tcb->tx_mem_tcb.tx_blk_idx = 0;
                            P_BtLtp->p_aci_tcb->tx_mem_tcb.tx_un_used_size = 0;
                        }

                        DBG_BUFFER(MODULE_LTP, LEVEL_INFO, "Free2:tx idx = 0x%4x, free idx = 0x%4x\t, tx_un_used_size = 0x%4x\t", 3, \
                                   P_BtLtp->p_aci_tcb->tx_mem_tcb.tx_blk_idx, \
                                   P_BtLtp->p_aci_tcb->tx_mem_tcb.tx_free_blk_idx, \
                                   P_BtLtp->p_aci_tcb->tx_mem_tcb.tx_un_used_size);
                    }
                    else
                    {
                        DBG_BUFFER(MODULE_LTP, LEVEL_ERROR, "QueueHandleTxRel recieve fail: ", 0);
                    }


                    if (P_BtLtp->pBufferAction->Action >= btltpActionReset && 
												P_BtLtp->pBufferAction->Action <  btltpActionReleaseBuffer)
                    {
                        Handle.lpHandle       = (PVOID)P_BtLtp->pBufferAction;
                        BTLTPBufferCallback(Handle);
                        P_BtLtp->pBufferAction->Action = btltpActionNotUsed;
                        P_BtLtp->pBufferAction = NULL;
                    }
			
                    ltpStartTransmit();
                    break;
                }
            case LTP_EVENT_BLUEAPI_MESSAGE:            /* BlueAPI */
                {
                    PBlueAPI_UsMessage pMsg;

                    while (xQueueReceive(P_BtLtp->p_aci_tcb->QueueHandleMessage, &pMsg, 0) == pdPASS)
                    {
                        ltpHandleBlueAPIMessage(pMsg);
                    }
                    break;
                }

            default:
                DBG_BUFFER(MODULE_LTP, LEVEL_ERROR, "ltpTask: Unknown event (%d)", 1, Event);
                break;
            }
        }
        else
        {
            DBG_BUFFER(MODULE_LTP, LEVEL_ERROR, "ltpTask: xQueueReceive fail", 0);
        }
    }
}

