/**********************************************************************!KA****
 *
 * $Header: /var/lib/cvs/sw/src/system/srp/src/uart/comuart.c,v 1.2 2013/10/10 13:46:33 mn Exp $
 *
 * File:        $RCSfile: comuart.c,v $
 * Version:     $Name: P_SRP1290_V1_0 $
 *
 * Archive:     $Source: /var/lib/cvs/sw/src/system/srp/src/uart/comuart.c,v $
 * Revision:    $Revision: 1.2 $
 * Date:        $Date: 2013/10/10 13:46:33 $
 * Author:      $Author: mn $
 *
 * ---------------------------------------------------------------------------
 * !MODULE      [  ]
 * ---------------------------------------------------------------------------
 * !FILE        [  ]
 * !PROGRAM     [  ]
 * !VERSION     [$Name: P_SRP1290_V1_0 $]
 * !GROUP       [  ]
 * !AUTHOR      [$Author: mn $]
 * ---------------------------------------------------------------------------
 *
 *          Copyright (c)           2013 Stollmann E+V GmbH
 *                                  Mendelssohnstr. 15d
 *                                  22761 Hamburg
 *                                  Phone: 040/89088-0
 *          The content of this file is Stollmann E+V GmbH confidential
 *          All Rights Reserved
 *
 * ---------------------------------------------------------------------------
 * !DESCRIPTION
 *
 *         Example Uart implementation using blueMod+SR
 *         Use USART3:
 *         DMA Rx cahnnel 3
 *         DMA Tx channel 2
 *         PC11 (USART3 RxD) connected to PC6 (TIM3 Ch 1) for character timeout
 *         CSR
 *            RST    PB12
 *            RTS    PB13
 *            CTS    PB14
 * ---------------------------------------------------------------------------
 * !INDEX
 *  ...
 * ---------------------------------------------------------------------------
 * !CONTENTS
 * ---------------------------------------------------------------------------
 * !INCLUDE_REFERENCES
 * ---------------------------------------------------------------------------
 * !HISTORY
 *  Date      Author          Comment
 *  tt.mm.jj                  Initial revision
 *  tt.mm.jj
 ************************************************************************!KA*/


#include <stm32f10x.h>

#include "comuart.h"

TComUart tComUart;


/****************************************************************************/
/* Start character timeout                                                  */
/****************************************************************************/

static void comUartStartCharacterTimeout(void)
{
  TIM_DeInit(TIM3);
  TIM_ARRPreloadConfig(TIM3, ENABLE);
  TIM_UpdateRequestConfig(TIM3, ENABLE);

  tComUart.TIM_TimeBaseInitStructure.TIM_Prescaler         = COM_TIM_PRESCALER - 1;
  tComUart.TIM_TimeBaseInitStructure.TIM_CounterMode       = TIM_CounterMode_Up;
  tComUart.TIM_TimeBaseInitStructure.TIM_Period            = HSE_VALUE / tComUart.USART_InitStructure.USART_BaudRate;
  tComUart.TIM_TimeBaseInitStructure.TIM_ClockDivision     = TIM_CKD_DIV1;
  tComUart.TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM3, &tComUart.TIM_TimeBaseInitStructure);
  TIM_SetCompare1(TIM3, tComUart.TIM_TimeBaseInitStructure.TIM_Period);

  TIM_SelectMasterSlaveMode(TIM3, TIM_SlaveMode_Reset);
  TIM_SelectInputTrigger(TIM3, TIM_TS_TI1FP1);
  TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);
  TIM_Cmd(TIM3, ENABLE);
}


/****************************************************************************/
/* EXTI - start bit                                                         */
/****************************************************************************/

static void comUartEXTICommand(FunctionalState NewState)
{
  tComUart.EXTI_InitStructure.EXTI_LineCmd = NewState;
  EXTI_Init(&tComUart.EXTI_InitStructure);
}


/****************************************************************************/
/* Start transmit                                                           */
/****************************************************************************/

void comUartChangeBaudrate(void)
{
  tComUart.USART_InitStructure.USART_BaudRate = tComUart.BaudRate;
  USART_Init(USART3, &tComUart.USART_InitStructure);
}


/****************************************************************************/
/* Start transmit                                                           */
/****************************************************************************/

void comUartStartTransmit(void)
{
  if (tComUart.TxData.pBuffer != (uint8_t)0)
  {
    tComUart.DMA_Tx_InitStructure.DMA_MemoryBaseAddr = (uint32_t)tComUart.TxData.pBuffer;
    tComUart.DMA_Tx_InitStructure.DMA_BufferSize     = tComUart.TxData.Length;
    DMA_Init(DMA1_Channel2, &tComUart.DMA_Tx_InitStructure);

    DMA_ITConfig(DMA1_Channel2, (DMA_IT_TC | DMA_IT_TE), ENABLE);
    USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
    DMA_Cmd(DMA1_Channel2, ENABLE);
  }
}


/****************************************************************************/
/* Disable Rx                                                               */
/****************************************************************************/

void comUartRxDisable(void)
{
  GPIO_SetBits(GPIOB, GPIO_Pin_14);    /* CSR CTS */
  tComUart.RxDisabled = true;
}


/****************************************************************************/
/* Enable Rx                                                                */
/****************************************************************************/

void comUartRxEnable(void)
{
  GPIO_ResetBits(GPIOB, GPIO_Pin_14);    /* CSR CTS */
  tComUart.RxDisabled = false;
}


/****************************************************************************/
/* Handle RTS                                                               */
/****************************************************************************/

static void comUartHandleRTS(void)
{
  uint8_t RTS;

  RTS = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13);
  if (RTS)
  {
    tComUart.TxDisabled = true;
  }
  else
  {
    tComUart.TxDisabled = false;
    if (tComUart.TxData.pBuffer != (uint8_t *)0)
    {
      tComUart.StartTransmit();
    }
  }
}


/****************************************************************************/
/* EXTI6 (RxD) interrupt                                                    */
/****************************************************************************/

void EXTI9_5_IRQHandler(void)
{
  if (EXTI_GetFlagStatus(EXTI_Line6) == SET)   /* TIM3 ch1: start bit */
  {
    EXTI_ClearFlag(EXTI_Line6);
    comUartEXTICommand(DISABLE);

    comUartStartCharacterTimeout();
  }
}


/****************************************************************************/
/* EXTI11 (RxD) interrupt                                                   */
/****************************************************************************/

#if defined(SRP0007)     /* Stollmann only: internal test */
void ltpEXTI15_10_IRQHandler(void);
#endif
void EXTI15_10_IRQHandler(void)
{
  if (EXTI_GetFlagStatus(EXTI_Line13) == SET)   /* CSR RTS */
  {
    EXTI_ClearFlag(EXTI_Line13);
    comUartHandleRTS();
  }
#if defined(SRP0007)
  ltpEXTI15_10_IRQHandler();
#endif
}


/****************************************************************************/
/* TIM3 character timeout interrupt                                         */
/****************************************************************************/

void TIM3_IRQHandler(void)
{
  comUartEXTICommand(ENABLE);
  TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
  TIM_Cmd(TIM3, DISABLE);

  tComUart.DataReceived();
}


/****************************************************************************/
/* DMA Rx interrupt                                                         */
/****************************************************************************/

void DMAChannel3_IRQHandler(void)
{
  DMA_ClearFlag(DMA1_FLAG_GL3 | DMA1_FLAG_TC3 | DMA1_FLAG_HT3 | DMA1_FLAG_TE3);

  tComUart.DataReceived();
}


/****************************************************************************/
/* DMA Tx interrupt                                                         */
/****************************************************************************/

void DMAChannel2_IRQHandler(void)
{
  DMA_ClearFlag(DMA1_FLAG_GL2 | DMA1_FLAG_TC2 | DMA1_FLAG_HT2 | DMA1_FLAG_TE2);
  USART_DMACmd(USART3, USART_DMAReq_Tx, DISABLE);
  DMA_Cmd(DMA1_Channel2, DISABLE);

  tComUart.TransmitCompleted();
}

/****************************************************************************/
/* Uart init                                                                */
/****************************************************************************/

void comUartInit(void)
{
  NVIC_InitTypeDef  NVIC_InitStructure;
  GPIO_InitTypeDef  GPIO_InitStructure;
  EXTI_InitTypeDef  EXTI_InitStructure;         /* CSR RTS */

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3 | RCC_APB1Periph_TIM3, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);

  GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);
  GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);

  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_11;              /* PC11 - RxD */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;             /* PC10 - TxD */
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;              /* PC6 - TIM3 ch 1 */
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;             /* PB12 - CSR RST */
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOB, GPIO_Pin_12);

  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_13;             /* PB13 - CSR RTS I */
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  EXTI_InitStructure.EXTI_Line    = EXTI_Line13;           /* CSR RTS */
  EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_14;             /* PB14 - CSR CTS O */
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOB, GPIO_Pin_14);

  USART_DeInit(USART3);

  tComUart.USART_InitStructure.USART_BaudRate            = COM_INIT_BAUDRATE;
  tComUart.USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
  tComUart.USART_InitStructure.USART_StopBits            = USART_StopBits_1;
  tComUart.USART_InitStructure.USART_Parity              = USART_Parity_No ;
  tComUart.USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  tComUart.USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART3, &tComUart.USART_InitStructure);

  DMA_DeInit(DMA1_Channel3);                /* Rx channel */
  tComUart.DMA_Rx_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;
  tComUart.DMA_Rx_InitStructure.DMA_MemoryBaseAddr     = (uint32_t)tComUart.RxBuffer;
  tComUart.DMA_Rx_InitStructure.DMA_DIR                = DMA_DIR_PeripheralSRC;
  tComUart.DMA_Rx_InitStructure.DMA_BufferSize         = sizeof(tComUart.RxBuffer);
  tComUart.DMA_Rx_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
  tComUart.DMA_Rx_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
  tComUart.DMA_Rx_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  tComUart.DMA_Rx_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
  tComUart.DMA_Rx_InitStructure.DMA_Mode               = DMA_Mode_Circular;
  tComUart.DMA_Rx_InitStructure.DMA_Priority           = DMA_Priority_Medium;
  tComUart.DMA_Rx_InitStructure.DMA_M2M                = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel3, &tComUart.DMA_Rx_InitStructure);

  DMA_ITConfig(DMA1_Channel3, (DMA_IT_TC | DMA_IT_HT | DMA_IT_TE), ENABLE);
  USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
  DMA_Cmd(DMA1_Channel3, ENABLE);

  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_KERNEL_INTERRUPT_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  DMA_DeInit(DMA1_Channel2);                /* Tx channel */
  tComUart.DMA_Tx_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;
  tComUart.DMA_Tx_InitStructure.DMA_MemoryBaseAddr     = 0;
  tComUart.DMA_Tx_InitStructure.DMA_DIR                = DMA_DIR_PeripheralDST;
  tComUart.DMA_Tx_InitStructure.DMA_BufferSize         = 0;
  tComUart.DMA_Tx_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
  tComUart.DMA_Tx_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
  tComUart.DMA_Tx_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  tComUart.DMA_Tx_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
  tComUart.DMA_Tx_InitStructure.DMA_Mode               = DMA_Mode_Normal;
  tComUart.DMA_Tx_InitStructure.DMA_Priority           = DMA_Priority_Medium;
  tComUart.DMA_Tx_InitStructure.DMA_M2M                = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel2, &tComUart.DMA_Tx_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_KERNEL_INTERRUPT_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  tComUart.EXTI_InitStructure.EXTI_Line    = EXTI_Line6;
  tComUart.EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
  tComUart.EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  comUartEXTICommand(ENABLE);
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource6);

  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_KERNEL_INTERRUPT_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource13);

  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_KERNEL_INTERRUPT_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_KERNEL_INTERRUPT_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
  USART_Cmd(USART3, ENABLE);
}

