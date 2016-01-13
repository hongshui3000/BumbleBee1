enum { __FILE_NUM__ = 0 };
//#include "board.h"
//#include "rtl876x_rcc.h"
//#include "rtl876x_uart.h"
//#include "rtl876x_pinmux.h"
//#include "rtl876x_gpio.h"
#include <stm32f10x.h>
#include "btltp.h"
#include "aci_low_power.h"
#include "trace.h"

//uint8_t RxTriggerLevel = 14;
#define LTP_TIM_PRESCALER        40      /* 40 bits timeout */

#define LTP_BAUDRATE             115200  /* baudrate */
const unsigned char ltpEventUartRx          = LTP_EVENT_UART_RX;
const unsigned char ltpEventUartTx          = LTP_EVENT_UART_TX;
const unsigned char ltpEventUartTxCompleted = LTP_EVENT_UART_TX_COMPLETED;
/****************************************************************************/
/**
 * @brief send to ltp task to release tx buffer space.
 *
 * @param p_tx_buf, pointer to the tx bufer to be released.
 * @return void.
*/
portBASE_TYPE ltpSendTxBufReleaseMsg(TLTPData *p_tx_buf)
{
    portBASE_TYPE ReturnValue;

    ReturnValue = xQueueSend(P_BtLtp->p_aci_tcb->QueueHandleTxRel, p_tx_buf, 0);

    return (ReturnValue);
}

/**
 * @brief send event to ltp task in isr.
 *
 * @param pEvent, pointer to the event to b sent.
 * @return send result.
 * @retval pdPASS--send successfully.
 *         errQUEUE_FULL-- queue is full.
*/
portBASE_TYPE ltpSendEventFromISR(const unsigned char *pEvent)
{
    portBASE_TYPE ReturnValue;
    portBASE_TYPE TaskWoken = pdFALSE;

    ReturnValue = xQueueSendFromISR(P_BtLtp->p_aci_tcb->QueueHandleEvent, pEvent, &TaskWoken);
    portYIELD_FROM_ISR(TaskWoken);

    return (ReturnValue);
}

portBASE_TYPE ltpSendEventMsg(const unsigned char *pEvent)
{
    portBASE_TYPE ReturnValue;

    ReturnValue = xQueueSend(P_BtLtp->p_aci_tcb->QueueHandleEvent, pEvent, 0);
    return (ReturnValue);
}

/**
 * @brief tx task, tx bytes in polling mode.
 *
 * @param pParameters, not used.
 * @return void.
*/
void TxAssistTask(void *pParameters)
{
#if 0
    uint32_t i = 0;
    uint8_t *pBuffer = NULL;
    uint8_t event = 0;
    uint16_t blk_count = 0;
    TLTPData data = {NULL, 0};

    while (TRUE)
    {
        if (xQueueReceive(P_BtLtp->p_aci_tcb->QueueHandleTxData, &data, portMAX_DELAY) == pdPASS)
        {
            pBuffer = data.pBuffer;
            blk_count = data.Length / 16;

            for (i = 0; i < blk_count; i++)
            {
                while (UART_GetFlagState(UART, UART_FLAG_THR_EMPTY) != SET);
                UART_SendData(UART, pBuffer, 16);
                pBuffer += 16;
            }

            //the ramain data
            blk_count = data.Length % 16;

            if (blk_count)
            {
                while (UART_GetFlagState(UART, UART_FLAG_THR_EMPTY) != SET);
                UART_SendData(UART, pBuffer, blk_count);
            }

            if (pdFALSE == ltpSendTxBufReleaseMsg(&data))
            {
                DBG_BUFFER(MODULE_LTP, LEVEL_ERROR, "ltpSendTxBufReleaseMsg fail", 0);
            }
            event = LTP_EVENT_UART_TX_COMPLETED;
            if (pdFALSE == ltpSendEventMsg(&event))
            {
                DBG_BUFFER(MODULE_LTP, LEVEL_ERROR, "ltpSendEvent fail", 0);
            }

            /* reset moniter timer */
            if (MoniterTimer)
            {
                xTimerReset(MoniterTimer, 5);
                MonitorTimeout = 0;
            }
        }
        else
        {
            DBG_BUFFER(MODULE_LTP, LEVEL_ERROR, "TxAssistTask: xQueueReceive fail", 0);
        }
    }
#endif
}


/****************************************************************************/
/* Disable Rx                                                               */
/****************************************************************************/
#if ACI_EN
#else
static void ltpRxDisable(void)
{
  GPIO_SetBits(GPIOA, GPIO_Pin_12);    /* CTS */
  P_BtLtp->p_aci_tcb->RxDisabled = true;
}


/****************************************************************************/
/* Enable Rx                                                                */
/****************************************************************************/

static void ltpRxEnable(void)
{
  GPIO_ResetBits(GPIOA, GPIO_Pin_12);    /* CTS */
  P_BtLtp->p_aci_tcb->RxDisabled = false;
}

static void ltpHandleRTS(void)
{
  uint8_t RTS;

  RTS = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11);
  if (RTS)
  {
    P_BtLtp->p_aci_tcb->TxDisabled = true;
  }
  else
  {
    P_BtLtp->p_aci_tcb->TxDisabled = false;
    if (P_BtLtp->p_aci_tcb->TxData.pBuffer != (uint8_t *)0)
    {
      ltpSendEventFromISR(&ltpEventUartTx);
    }
  }
}
#endif
/**
 * @brief update rx data length when received data from uart or spi.
 *
 * @param none.
 * @return none.
 * @retval void.
*/
void LtpRxDataLengthUpdate(void)
{
    uint16_t RxOffset;
    uint16_t Length;
    
    RxOffset = RX_BUFFER_SIZE - DMA_GetCurrDataCounter(DMA1_Channel5);
    //RxOffset = P_BtLtp->p_aci_tcb->P_RxBuffer - &P_BtLtp->p_aci_tcb->p_rx_buf[0]; /* tifnan: num of char received */

    /* will not occur in our uart framework!!! */
    if (P_BtLtp->p_aci_tcb->RxOffset == RxOffset)
    {
        if (P_BtLtp->p_aci_tcb->RxDataLength == RX_BUFFER_SIZE)  /* overrun */
        {
            DBG_BUFFER(MODULE_LTP, LEVEL_ERROR, "LtpRxDataLengthUpdate: Rx overrun", 0);
            P_BtLtp->p_aci_tcb->RxDataLength = 0;
            Length = RX_BUFFER_SIZE;
        }
        else
        {
            return;       /* no data */
        }
    }
    else
    {
        /* [p_aci_tcb->RxOffset----RxBufferSize-1] + [P_BtLtp->p_aci_tcb->p_rx_buf[0]----RxOffset] */
        if (P_BtLtp->p_aci_tcb->RxOffset > RxOffset)
        {
            Length = RX_BUFFER_SIZE - P_BtLtp->p_aci_tcb->RxOffset + RxOffset;
        }
        /* [p_aci_tcb->RxOffset ---- RxOffset] */
        else
        {
            Length = RxOffset - P_BtLtp->p_aci_tcb->RxOffset;
        }

        /* update new P_BtLtp->p_aci_tcb->RxOffset */
        P_BtLtp->p_aci_tcb->RxOffset = RxOffset;
    }

    if ((Length + P_BtLtp->p_aci_tcb->RxDataLength) > RX_BUFFER_SIZE)   /* Rx overrun */
    {
        DBG_BUFFER(MODULE_LTP, LEVEL_ERROR, "LtpRxDataLengthUpdate: Rx overrun (%d)", 1, \
                   Length + P_BtLtp->p_aci_tcb->RxDataLength);

        P_BtLtp->p_aci_tcb->RxDataLength  = RX_BUFFER_SIZE;
        P_BtLtp->p_aci_tcb->RxWriteIndex += Length;
        P_BtLtp->p_aci_tcb->RxWriteIndex &= (RX_BUFFER_SIZE - 1);
        P_BtLtp->p_aci_tcb->RxReadIndex   = P_BtLtp->p_aci_tcb->RxWriteIndex;
    }
    else
    {
        P_BtLtp->p_aci_tcb->RxDataLength += Length;         /* update length */
        P_BtLtp->p_aci_tcb->RxWriteIndex += Length;
        P_BtLtp->p_aci_tcb->RxWriteIndex &= (RX_BUFFER_SIZE - 1);
    }
#if ACI_EN
#else
    if (P_BtLtp->p_aci_tcb->RxDataLength >= RX_DISABLE_COUNT &&
      P_BtLtp->p_aci_tcb->RxDisabled == false)
    {
        ltpRxDisable();
    }
#endif
}

/**
 * @brief uart interrupt handle ISR.
 *
 * @param none.
 * @return none.
 * @retval void.
*/
#if 0
void LtpDataUartIrqHandle(void)
{
    /* read interrupt status */
    UINT32 int_status;
    uint16_t len = 0;
    uint8_t event = LTP_EVENT_UART_RX;
    /* read interrupt id */
    int_status = UART_GetIID(UART);
    /* disable interrupt */
    UART_INTConfig(UART, UART_INT_RD_AVA | UART_INT_LINE_STS, DISABLE);

    switch (int_status)
    {
    /* tx fifo empty */
    case UART_INT_ID_TX_EMPTY:
        break;

    /* rx data valiable */
    case UART_INT_ID_RX_LEVEL_REACH:
        if ((P_BtLtp->p_aci_tcb->P_RxBuffer - &P_BtLtp->p_aci_tcb->p_rx_buf[0] + RxTriggerLevel)\
                <= RX_BUFFER_SIZE)
        {
            UART_ReceiveData(UART, P_BtLtp->p_aci_tcb->P_RxBuffer, RxTriggerLevel);
            P_BtLtp->p_aci_tcb->P_RxBuffer += RxTriggerLevel;
        }
        else
        {
            len = RX_BUFFER_SIZE - (P_BtLtp->p_aci_tcb->P_RxBuffer - &P_BtLtp->p_aci_tcb->p_rx_buf[0]);
            UART_ReceiveData(UART, P_BtLtp->p_aci_tcb->P_RxBuffer, len);
            P_BtLtp->p_aci_tcb->P_RxBuffer = &P_BtLtp->p_aci_tcb->p_rx_buf[0];
            UART_ReceiveData(UART, P_BtLtp->p_aci_tcb->P_RxBuffer, RxTriggerLevel - len);
            P_BtLtp->p_aci_tcb->P_RxBuffer += (RxTriggerLevel - len);
        }

        /* update rx data length */
        LtpRxDataLengthUpdate();
        /* notify ltp task */
        if (pdFALSE == ltpSendEventFromISR(&event))
        {
            DBG_BUFFER(MODULE_LTP, LEVEL_ERROR, "ltpSendEventFromISR fail", 0);
        }

        break;

    /* rx time out */
    case UART_INT_ID_RX_TMEOUT:
        /* read out all bytes in fifo */
        while (UART_GetFlagState(UART, UART_FLAG_RX_DATA_RDY) == SET)
        {
            if (P_BtLtp->p_aci_tcb->P_RxBuffer - &P_BtLtp->p_aci_tcb->p_rx_buf[0] == RX_BUFFER_SIZE)
            {
                P_BtLtp->p_aci_tcb->P_RxBuffer = &P_BtLtp->p_aci_tcb->p_rx_buf[0];
            }
            UART_ReceiveData(UART, P_BtLtp->p_aci_tcb->P_RxBuffer, 1);
            P_BtLtp->p_aci_tcb->P_RxBuffer++;
        }

        /* update rx data length */
        LtpRxDataLengthUpdate();
        /* notify ltp task */
        if (pdFALSE == ltpSendEventFromISR(&event))
        {
            DBG_BUFFER(MODULE_LTP, LEVEL_ERROR, "ltpSendEventFromISR fail", 0);
        }

        break;

    /* receive line status interrupt */
    case UART_INT_ID_LINE_STATUS:
        DBG_BUFFER(MODULE_APP, LEVEL_ERROR, "Line status error!!!!\n", 0);
        break;

    default:
        break;
    }

    /* enable interrupt again */
    UART_INTConfig(UART, UART_INT_RD_AVA | UART_INT_LINE_STS, ENABLE);
}
#endif


/****************************************************************************/
/* EXTI - start bit                                                         */
/****************************************************************************/

static void ltpEXTICommand(FunctionalState NewState)
{
  P_BtLtp->p_aci_tcb->EXTI_InitStructure.EXTI_LineCmd = NewState;
  EXTI_Init(&P_BtLtp->p_aci_tcb->EXTI_InitStructure);
}

/****************************************************************************/
/* Start character timeout                                                  */
/****************************************************************************/

static void ltpStartCharacterTimeout(void)
{
  TIM_DeInit(TIM4);
  TIM_ARRPreloadConfig(TIM4, ENABLE);
  TIM_UpdateRequestConfig(TIM4, ENABLE);

  P_BtLtp->p_aci_tcb->TIM_TimeBaseInitStructure.TIM_Prescaler         = LTP_TIM_PRESCALER - 1;
  P_BtLtp->p_aci_tcb->TIM_TimeBaseInitStructure.TIM_CounterMode       = TIM_CounterMode_Up;
  P_BtLtp->p_aci_tcb->TIM_TimeBaseInitStructure.TIM_Period            = HSE_VALUE / P_BtLtp->p_aci_tcb->USART_InitStructure.USART_BaudRate;
  P_BtLtp->p_aci_tcb->TIM_TimeBaseInitStructure.TIM_ClockDivision     = TIM_CKD_DIV1;
  P_BtLtp->p_aci_tcb->TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM4, &P_BtLtp->p_aci_tcb->TIM_TimeBaseInitStructure);
  TIM_SetCompare1(TIM4, P_BtLtp->p_aci_tcb->TIM_TimeBaseInitStructure.TIM_Period);

  TIM_SelectMasterSlaveMode(TIM4, TIM_SlaveMode_Reset);
  TIM_SelectInputTrigger(TIM4, TIM_TS_TI1FP1);
  TIM_ITConfig(TIM4, TIM_IT_CC1, ENABLE);
  TIM_Cmd(TIM4, ENABLE);
}

void ltpStartTransmit(void)
{
  if (P_BtLtp->p_aci_tcb->TxData.pBuffer == (uint8_t)0 &&
      xQueueReceive(P_BtLtp->p_aci_tcb->QueueHandleTxData, &P_BtLtp->p_aci_tcb->TxData, 0) == pdPASS)
  {
    P_BtLtp->p_aci_tcb->DMA_Tx_InitStructure.DMA_MemoryBaseAddr = (uint32_t)P_BtLtp->p_aci_tcb->TxData.pBuffer;
    P_BtLtp->p_aci_tcb->DMA_Tx_InitStructure.DMA_BufferSize     = P_BtLtp->p_aci_tcb->TxData.Length;
    DMA_Init(DMA1_Channel4, &P_BtLtp->p_aci_tcb->DMA_Tx_InitStructure);

    DMA_ITConfig(DMA1_Channel4, (DMA_IT_TC | DMA_IT_TE), ENABLE);
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
    DMA_Cmd(DMA1_Channel4, ENABLE);
  }
}

/****************************************************************************/
/* EXTI11 (RxD) interrupt                                                   */
/****************************************************************************/

void ltpEXTI15_10_IRQHandler(void)
{
  if (EXTI_GetFlagStatus(EXTI_Line10) == SET)   /* RxD-pin: start bit */
  {
    EXTI_ClearFlag(EXTI_Line10);
    ltpEXTICommand(DISABLE);

    ltpStartCharacterTimeout();
  }
#if ACI_EN
#else
  if (EXTI_GetFlagStatus(EXTI_Line11) == SET)   /* RTS */
  {
    EXTI_ClearFlag(EXTI_Line11);
    ltpHandleRTS();
  }
#endif
}

/****************************************************************************/
/* RxData available                                                         */
/****************************************************************************/

static void ltpHandleRxInterrupt(void)
{
  LtpRxDataLengthUpdate();

  ltpSendEventFromISR(&ltpEventUartRx);
}

/****************************************************************************/
/* TIM4 character timeout interrupt                                         */
/****************************************************************************/

void TIM4_IRQHandler(void)
{
  ltpEXTICommand(ENABLE);
  TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);
  TIM_Cmd(TIM4, DISABLE);

  ltpHandleRxInterrupt();
}

/****************************************************************************/
/* DMA Rx interrupt                                                         */
/****************************************************************************/

void DMAChannel5_IRQHandler(void)
{
  DMA_ClearFlag(DMA1_FLAG_GL5 | DMA1_FLAG_TC5 | DMA1_FLAG_HT5 | DMA1_FLAG_TE5);

  ltpHandleRxInterrupt();
}


/****************************************************************************/
/* DMA Tx interrupt                                                         */
/****************************************************************************/

void DMAChannel4_IRQHandler(void)
{
  DMA_ClearFlag(DMA1_FLAG_GL4 | DMA1_FLAG_TC4 | DMA1_FLAG_HT4 | DMA1_FLAG_TE4);
  USART_DMACmd(USART1, USART_DMAReq_Tx, DISABLE);
  DMA_Cmd(DMA1_Channel4, DISABLE);
  if (pdFALSE == ltpSendTxBufReleaseMsg(&P_BtLtp->p_aci_tcb->TxData))
  {
       DBG_BUFFER(MODULE_LTP, LEVEL_ERROR, "ltpSendTxBufReleaseMsg fail", 0);
  }
  P_BtLtp->p_aci_tcb->TxData.pBuffer = (uint8_t)0;
  ltpSendEventFromISR(&ltpEventUartTxCompleted);
}

void ltpPeripheralInit(void)
{
#if 0
    //pinmux and pad config
    Pinmux_Config(UART_TX_PIN, DATA_UART_TX);
    Pinmux_Config(UART_RX_PIN, DATA_UART_RX);

    Pad_Config(UART_TX_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_DISABLE, PAD_OUT_LOW);
    Pad_Config(UART_RX_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_LOW);

    RCC_PeriphClockCmd(APBPeriph_UART, APBPeriph_UART_CLOCK, ENABLE);
    //uart init
    UART_InitTypeDef uartInitStruct;
    UART_StructInit(&uartInitStruct);

    uartInitStruct.rxTriggerLevel = UART_RX_FIFO_TRIGGER_LEVEL_14BYTE;
    UART_Init(UART, &uartInitStruct);
    //enable line status interrupt and rx data avaliable interrupt
    UART_INTConfig(UART, UART_INT_RD_AVA | UART_INT_LINE_STS, ENABLE);

    /*  Enable UART IRQ  */
    NVIC_ClearPendingIRQ(UART_IRQ);
    NVIC_SetPriority(UART_IRQ, 0);
    NVIC_EnableIRQ(UART_IRQ);
#endif
  NVIC_InitTypeDef  NVIC_InitStructure;
  GPIO_InitTypeDef  GPIO_InitStructure;
#if ACI_EN
#else
  EXTI_InitTypeDef  EXTI_InitStructure;         /* RTS */
#endif
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

  GPIO_PinRemapConfig(GPIO_Remap_USART1, DISABLE);
  GPIO_PinRemapConfig(GPIO_Remap_TIM4, DISABLE);

  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_10;              /* PA10 - RxD */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;              /* PA9  - TxD */
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;              /* PB6  - TIM4 ch 1 */
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
#if ACI_EN
#else
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;             /* PA12 - CTS */
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOA, GPIO_Pin_12);

  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;             /* PA11 - RTS */
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  EXTI_InitStructure.EXTI_Line    = EXTI_Line11;           /* RTS */
  EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource11);
#endif
  USART_DeInit(USART1);

  P_BtLtp->p_aci_tcb->USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
  P_BtLtp->p_aci_tcb->USART_InitStructure.USART_StopBits            = USART_StopBits_1;
  P_BtLtp->p_aci_tcb->USART_InitStructure.USART_Parity              = USART_Parity_No ;
  P_BtLtp->p_aci_tcb->USART_InitStructure.USART_BaudRate            = LTP_BAUDRATE;
  P_BtLtp->p_aci_tcb->USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  P_BtLtp->p_aci_tcb->USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &P_BtLtp->p_aci_tcb->USART_InitStructure);

  DMA_DeInit(DMA1_Channel5);                /* Rx channel */
  P_BtLtp->p_aci_tcb->DMA_Rx_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
  P_BtLtp->p_aci_tcb->DMA_Rx_InitStructure.DMA_MemoryBaseAddr     = (uint32_t)P_BtLtp->p_aci_tcb->p_rx_buf;
  P_BtLtp->p_aci_tcb->DMA_Rx_InitStructure.DMA_DIR                = DMA_DIR_PeripheralSRC;
  P_BtLtp->p_aci_tcb->DMA_Rx_InitStructure.DMA_BufferSize         = RX_BUFFER_SIZE;
  P_BtLtp->p_aci_tcb->DMA_Rx_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
  P_BtLtp->p_aci_tcb->DMA_Rx_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
  P_BtLtp->p_aci_tcb->DMA_Rx_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  P_BtLtp->p_aci_tcb->DMA_Rx_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
  P_BtLtp->p_aci_tcb->DMA_Rx_InitStructure.DMA_Mode               = DMA_Mode_Circular;
  P_BtLtp->p_aci_tcb->DMA_Rx_InitStructure.DMA_Priority           = DMA_Priority_Medium;
  P_BtLtp->p_aci_tcb->DMA_Rx_InitStructure.DMA_M2M                = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel5, &P_BtLtp->p_aci_tcb->DMA_Rx_InitStructure);

  DMA_ITConfig(DMA1_Channel5, (DMA_IT_TC | DMA_IT_HT | DMA_IT_TE), ENABLE);
  USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
  DMA_Cmd(DMA1_Channel5, ENABLE);

  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_KERNEL_INTERRUPT_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  DMA_DeInit(DMA1_Channel4);                /* Tx channel */
  P_BtLtp->p_aci_tcb->DMA_Tx_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
  P_BtLtp->p_aci_tcb->DMA_Tx_InitStructure.DMA_MemoryBaseAddr     = 0;
  P_BtLtp->p_aci_tcb->DMA_Tx_InitStructure.DMA_DIR                = DMA_DIR_PeripheralDST;
  P_BtLtp->p_aci_tcb->DMA_Tx_InitStructure.DMA_BufferSize         = 0;
  P_BtLtp->p_aci_tcb->DMA_Tx_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
  P_BtLtp->p_aci_tcb->DMA_Tx_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
  P_BtLtp->p_aci_tcb->DMA_Tx_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  P_BtLtp->p_aci_tcb->DMA_Tx_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
  P_BtLtp->p_aci_tcb->DMA_Tx_InitStructure.DMA_Mode               = DMA_Mode_Normal;
  P_BtLtp->p_aci_tcb->DMA_Tx_InitStructure.DMA_Priority           = DMA_Priority_Medium;
  P_BtLtp->p_aci_tcb->DMA_Tx_InitStructure.DMA_M2M                = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel4, &P_BtLtp->p_aci_tcb->DMA_Tx_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_KERNEL_INTERRUPT_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  P_BtLtp->p_aci_tcb->EXTI_InitStructure.EXTI_Line    = EXTI_Line10;
  P_BtLtp->p_aci_tcb->EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
  P_BtLtp->p_aci_tcb->EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  ltpEXTICommand(ENABLE);
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource10);

  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_KERNEL_INTERRUPT_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_KERNEL_INTERRUPT_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
  USART_Cmd(USART1, ENABLE);
#if ACI_EN
#else
  ltpRxEnable();
#endif
}

