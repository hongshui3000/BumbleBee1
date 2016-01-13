/**
*********************************************************************************************************
*               Copyright(c) 2014, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file      aci_if.h
* @brief     some macro/struct/functions declaration of aci interface
* @details   none.
* @author    tifnan
* @date      2014-10-17
* @version   v0.1
* *********************************************************************************************************
*/

#ifndef     _ACI_IF_H_
#define     _ACI_IF_H_

/****************************************************************************
 * includes
 ****************************************************************************/
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <timers.h>

#include <ltplib.h>
#include <stm32f10x.h>
/* Ltp data struct */
typedef struct _TData
{
    uint8_t *pBuffer;
    uint32_t Length;
} TLTPData;


/** @brief tcb to manage tx buffer */
typedef struct
{
    uint16_t    tx_blk_idx;          /* sending block index [0 -- TX_BUFFER_SIZE_MASK](block is sending now ) */
    uint16_t    tx_free_blk_idx;     /* free tx block index [0----TX_BUFFER_SIZE_MASK] */
    uint16_t    tx_un_used_size;     /* size of block which is discarded */
    uint16_t    free_size;           /* bytes can be used */
} TxMemTCB, *PTxMemTCB;

/** @brief aci control struct */
typedef struct _ACI_TCB
{
    TLTPData                  TxData;
    xTaskHandle               Handle;              /* task handle */
    xQueueHandle              QueueHandleEvent;    /* task queue */
    xQueueHandle              QueueHandleMessage;  /* message queue */
    xQueueHandle              QueueHandleTxData;   /* Tx queue */
    xQueueHandle              QueueHandleRxData;   /* Rx queue */
    xQueueHandle              QueueHandleTxRel;    /* tx buffer release queue */
    uint32_t                     RxDataIndication;    /* pending responses */
    uint32_t                     RxDataLength;
    uint32_t                     RxOffset;
    uint16_t                      RxReadIndex;
    uint16_t                      RxWriteIndex;
    // for F_LTPLIB_ASYNC_ASSEMBLY_SUPPORT
#if F_LTPLIB_ASYNC_ASSEMBLY_SUPPORT
    xTimerHandle              ltpTimerHandle;
    int                       ltpTimerMS;
    TLTPTimerID               ltpTimerID;         /* only one timer */
#endif
    PBYTE                     p_rx_buf;           /* pointer to the rx buffer allocated dynamically */
    PBYTE                     p_tx_buf;           /* pointer to the tx buffer allocated dynamically */
    PBYTE                     p_rx_handle_buf;    /* buffer address, handle uart rx data */
    PBYTE                     P_RxBuffer;         /* for rx */
    TxMemTCB                  tx_mem_tcb;         /* manage tx bufer */


    USART_InitTypeDef         USART_InitStructure;
    DMA_InitTypeDef           DMA_Rx_InitStructure;
    DMA_InitTypeDef           DMA_Tx_InitStructure;
    TIM_TimeBaseInitTypeDef   TIM_TimeBaseInitStructure;  /* RxD */
    EXTI_InitTypeDef          EXTI_InitStructure;         /* RxD */
    BOOL                      RxDisabled;
    BOOL                      TxDisabled;
} ACI_TCB, *P_ACI_TCB;

/* export functions */
uint8_t ltpInit(void);
uint8_t ltpTaskInit(void);
#endif

