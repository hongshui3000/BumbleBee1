/**********************************************************************!MA*
 *
 * $Header: /var/lib/cvs/sw/src/system/srp/src/target/hw_srp_freertos.c,v 1.14 2014/03/19 14:43:34 ka Exp $
 *
 * File:        $RCSfile: hw_srp_freertos.c,v $
 * Version:     $Name:
 *
 * Archive:     $Source: /var/lib/cvs/sw/src/system/srp/src/target/hw_srp_freertos.c,v $
 *
 * ------------------------------------------------------------------------
 * !MODULE      [  ]
 * ------------------------------------------------------------------------
 * !FILE        [  ]
 * !PROGRAM     [  ]
 * !VERSION     [$Name:]
 * !GROUP       [  ]
 * ------------------------------------------------------------------------
 *
 *          Copyright (c)           2011 Stollmann E+V GmbH
 *                                  Mendelssohnstr. 15
 *                                  22761 Hamburg
 *                                  Phone: 040/89088-0
 *          The content of this file is Stollmann E+V GmbH confidential
 *          All Rights Reserved
 *
 * ------------------------------------------------------------------------
 * !DESCRIPTION
 *           BlueMod+SR interface
 *
 * ------------------------------------------------------------------------
 * !INDEX
 *  ...
 * ------------------------------------------------------------------------
 * !CONTENTS
 * ------------------------------------------------------------------------
 * !INCLUDE_REFERENCES
 * ------------------------------------------------------------------------
 * !HISTORY
 *  Date      Author          Comment
 *  tt.mm.jj                  Initial revision
 *  tt.mm.jj
 **********************************************************************!HE*/
enum { __FILE_NUM__ = 0 };
/****************************************************************************/
/* includes                                                                 */
/****************************************************************************/

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <stm32f10x.h>
#include <string.h>
#include <stextif.h>

#include "hw_srp.h"

#include <cycle_queue.h>
#include "trace.h"
#include <rtl_types.h>
#include <common_defs.h>

static uint8_t hwVersion;

const uint8_t firmwareVersion[] = "BLB0D00 V1.000 "__DATE__" "__TIME__;

/****************************************************************************/
/* getInfopageBaseAddress                                                   */
/* used by BtStack                                                          */
/****************************************************************************/

void *getInfopageBaseAddress(unsigned char idx)
{
  switch(idx)
  {
    case 1:
      return (void*)FEP_INFOPAGE1_BASE;
    case 2:
      return (void*)FEP_INFOPAGE2_BASE;
  }
  return((void *)0);
}


/****************************************************************************/
/* hwBluetoothResetActivate                                                 */
/****************************************************************************/

static void hwBluetoothResetActivate(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

  switch (hwVersion)
  {
    case 3:
      GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_12;   /* PB12 VregEnable AI */
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
      GPIO_Init(GPIOB, &GPIO_InitStructure);
      break;
    case 4:
      GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_15;   /* PB15 AI */
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
      GPIO_Init(GPIOB, &GPIO_InitStructure);
      GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0;    /* PB0 AI */
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
      GPIO_Init(GPIOB, &GPIO_InitStructure);
      break;
    default:
      GPIO_ResetBits(GPIOB, GPIO_Pin_12);
      break;
  }
}


/****************************************************************************/
/* hwBluetoothResetDeactivate                                               */
/****************************************************************************/

static void hwBluetoothResetDeactivate(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  switch (hwVersion)
  {
    case 3:
      GPIO_ResetBits(GPIOB, GPIO_Pin_12);
      GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;
      GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
      GPIO_Init(GPIOB, &GPIO_InitStructure);
      break;
    case 4:
      GPIO_ResetBits(GPIOB, GPIO_Pin_0);
      GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
      GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
      GPIO_Init(GPIOB, &GPIO_InitStructure);
      GPIO_SetBits(GPIOB, GPIO_Pin_15);
      GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_15;
      GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
      GPIO_Init(GPIOB, &GPIO_InitStructure);
      break;
    default:
      GPIO_SetBits(GPIOB, GPIO_Pin_12);
      break;
  }
}


/****************************************************************************/
/* hwBluetoothReset                                                         */
/****************************************************************************/

void hwBluetoothReset(void)
{
  hwBluetoothResetActivate();
  vTaskDelay(portTICK_RATE_MS * 10);                       /* 10 ms delay */
  hwBluetoothResetDeactivate();
}


/****************************************************************************/
/* Detect BlueMOD+SR version                                                */
/****************************************************************************/

void hwDetectHwVersion(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  volatile int     Delay;
  uint8_t          hwConf = 0;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0;   /* PC0 */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_1;   /* PC1 */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_2;   /* PC2 */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  for (Delay = 0; Delay < 100; Delay++) {}

  hwConf  = (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0) << 0);
  hwConf |= (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1) << 1);
  hwConf |= (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2) << 2);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, DISABLE);

  switch (hwConf)
  {
    case 6:
      hwVersion = 3; /* HW V3.0 */
      break;
    case 5:
      hwVersion = 4; /* HW V3.1 */
      break;
    case 7:
      hwVersion = 2; /* in fact is it the version 1 or 2 */
      break;
    default:
      hwVersion = 0;
      break;
  }
}


/****************************************************************************/
/* Apple Coprocessor interface routines                                     */
/****************************************************************************/

int stAuthCoProcInit( void )
{
  return(-1);          /* Not supported */
}

int stAuthCoProcWrite( uint8_t registerAddress, void * pData, uint8_t size )
{
  return(-1);          /* Not supported */
}

int stAuthCoProcRead( uint8_t registerAddress, void * pData, uint8_t size )
{
  return(-1);          /* Not supported */
}


/****************************************************************************/
/* EEPROM ST-Micro M2464                                                    */
/****************************************************************************/

#define DEVICE_ADDRESS_ST_2464                    0xA2
#define DEVICE_PAGE_SIZE_ST_2464                  0x20

#define I2C_CLOCK                   380000

/****************************************************************************/
/* NVRAM delay                                                              */
/****************************************************************************/

static void stNvramDelay(void)
{
  volatile int i;
  for (i=0; i<0x10000; i++) {}
}

/****************************************************************************/
/* NVRAM ST-M2464 reset                                                     */
/****************************************************************************/

static void stNvramM2464Reset(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  int               i;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE);

  GPIO_ResetBits(GPIOB, GPIO_Pin_10 | GPIO_Pin_11);
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;             /* PB10 - scl = 1 */
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;             /* PB11 - sda */
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_OD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Software reset */
  stNvramDelay();

  for (i = 0; i < 9; i++)
  {
    GPIO_SetBits(GPIOB, GPIO_Pin_10 | GPIO_Pin_11);
    stNvramDelay();
    GPIO_ResetBits(GPIOB, GPIO_Pin_10 | GPIO_Pin_11);
    stNvramDelay();
  }
  GPIO_SetBits(GPIOB, GPIO_Pin_11);
  GPIO_SetBits(GPIOB, GPIO_Pin_10);
  stNvramDelay();
  GPIO_ResetBits(GPIOB, GPIO_Pin_11);
  GPIO_ResetBits(GPIOB, GPIO_Pin_10);
  stNvramDelay();
}

/****************************************************************************/
/* NVRAM acknowlegde polling                                                */
/* return value:  0 ok.                                                     */
/*              !=0 error                                                   */
/****************************************************************************/

static int stNvramAcknowledgePolling(uint8_t I2C_Direction)
{
  int RetryCount;

  RetryCount = 100;                     /* 100 ms */
  do
  {
    uint16_t sr1;

    I2C_GenerateSTART(I2C2, ENABLE);      /* Send START */
    while (((I2C2->SR1 & I2C_SR1_SB) != I2C_SR1_SB) ||                         /* Start bit generated */
           ((I2C2->SR2 & (I2C_SR2_MSL | I2C_SR2_BUSY)) != (I2C_SR2_MSL | I2C_SR2_BUSY))) {} /* master */

    I2C_Send7bitAddress(I2C2, DEVICE_ADDRESS_ST_2464, I2C_Direction);
    do
    {
      sr1 = I2C2->SR1;
      if (sr1 & I2C_SR1_AF)
        break;
    } while (!(sr1 & I2C_SR1_ADDR));
    if (sr1 & I2C_SR1_ADDR)
      break;

    if (sr1 & I2C_SR1_AF)
    {
      I2C2->SR1 &=~I2C_SR1_AF;
      I2C_GenerateSTOP(I2C2, ENABLE);      /* Send STOP */

      vTaskDelay(portTICK_RATE_MS * 1);
    }
  } while (--RetryCount);
  (void)I2C2->SR2;

  if (RetryCount == 0)
    return(-1);                       /* timeout */
  else
    return(0);
}

/****************************************************************************/
/* I2C wait for assertion of SR1 TxE, if timeout assume reception of NAK    */
/****************************************************************************/

static int stNvramCWaitForTXENak(void)
{
  int RetryCount = 100;                     /* 100 ms */

  while (!(I2C2->SR1 & I2C_SR1_TXE))
  {
    vTaskDelay(portTICK_RATE_MS * 1);
    RetryCount--;
    if (RetryCount == 0)
      return(-1);
  }

  return(0);
}

/****************************************************************************/
/* NVRAM init                                                               */
/* return value:  0 ok.                                                     */
/*              !=0 error                                                   */
/****************************************************************************/

int stNvramInit(uint32_t size)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  I2C_InitTypeDef   I2C_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);     /* I2C2 */

  stNvramM2464Reset();

  GPIO_SetBits(GPIOB, GPIO_Pin_10);
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;             /* PB10 - scl = 1 */
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;             /* PB10 - scl */
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;             /* PB11 - sda */
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_OD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  I2C_DeInit(I2C2);

  I2C_InitStructure.I2C_ClockSpeed          = I2C_CLOCK;
  I2C_InitStructure.I2C_Mode                = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle           = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1         = DEVICE_ADDRESS_ST_2464;
  I2C_InitStructure.I2C_Ack                 = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_Cmd(I2C2, ENABLE);
  I2C_Init(I2C2, &I2C_InitStructure);

  return(0);
}

/****************************************************************************/
/* NVRAM read                                                               */
/* return value:  0 ok.                                                     */
/*              !=0 error                                                   */
/****************************************************************************/

int stNvramRead(void *pData, uint32_t Size)
{
  uint32_t EEpromAddr;
  uint32_t Length;
  uint8_t  *pBuffer = (uint8_t *)pData;

  EEpromAddr = 0;                       /* Read address = 0 */
  while (Size)
  {
    if (Size > DEVICE_PAGE_SIZE_ST_2464)
      Length = DEVICE_PAGE_SIZE_ST_2464;
    else
      Length = Size;

    Size -= Length;

    if (stNvramAcknowledgePolling(I2C_Direction_Transmitter))
      return(-1);                         /* timeout */

    if (stNvramCWaitForTXENak())
      return(-1);                         /* timeout */

    I2C2->DR = (uint8_t)(EEpromAddr >> 8);    /* Send Read address */
    while (!(I2C2->SR1 & I2C_SR1_BTF)) {}
    while (!(I2C2->SR1 & I2C_SR1_TXE)) {}

    I2C2->DR = (uint8_t)(EEpromAddr & 0xFF);  /* Send Read address */
    while (!(I2C2->SR1 & I2C_SR1_BTF)) {}
    while (!(I2C2->SR1 & I2C_SR1_TXE)) {}

    stNvramAcknowledgePolling(I2C_Direction_Receiver);

    EEpromAddr += Length;

    while (Length)
    {
      if (Length == 1)
      {
        I2C2->CR1 &=~I2C_CR1_ACK;
      }

      while (!(I2C2->SR1 & I2C_SR1_RXNE)) {}

      *pBuffer++ = I2C2->DR;
      Length--;
    }
    I2C2->CR1 |= I2C_CR1_ACK;
  }

  I2C_GenerateSTOP(I2C2, ENABLE);      /* Send STOP */
  stNvramDelay();

  return(0);
}

/****************************************************************************/
/* NVRAM write                                                              */
/* return value:  0  ok.                                                    */
/*              !=0 error                                                   */
/****************************************************************************/

int stNvramWrite(void *pData, uint32_t Size)
{
  uint32_t EEpromAddr;
  uint32_t Length;
  uint8_t  *pBuffer = (uint8_t *)pData;

  EEpromAddr = 0;                       /* Read address = 0 */
  while (Size)
  {
    if (Size > DEVICE_PAGE_SIZE_ST_2464)
      Length = DEVICE_PAGE_SIZE_ST_2464;
    else
      Length = Size;

    Size -= Length;

    if (stNvramAcknowledgePolling(I2C_Direction_Transmitter))
      return(-1);                         /* timeout */

    if (stNvramCWaitForTXENak())
      return(-1);                         /* timeout */

    I2C2->DR = (uint8_t)(EEpromAddr >> 8);    /* Send Read address */
    while (!(I2C2->SR1 & I2C_SR1_BTF)) {}
    while (!(I2C2->SR1 & I2C_SR1_TXE)) {}

    I2C2->DR = (uint8_t)(EEpromAddr & 0xFF);  /* Send Read address */
    while (!(I2C2->SR1 & I2C_SR1_BTF)) {}
    while (!(I2C2->SR1 & I2C_SR1_TXE)) {}

    EEpromAddr += Length;

    while (Length)
    {
      I2C2->DR = *pBuffer++;
      Length--;

      while (!(I2C2->SR1 & I2C_SR1_BTF)) {}
      while (!(I2C2->SR1 & I2C_SR1_TXE)) {}
    }

    I2C_GenerateSTOP(I2C2, ENABLE);      /* Send STOP */
    stNvramDelay();

    if (stNvramAcknowledgePolling(I2C_Direction_Transmitter))
      return(-1);                         /* timeout */
  }

  I2C_GenerateSTOP(I2C2, ENABLE);      /* Send STOP */
  stNvramDelay();

  return(0);
}

void stNvramTest()
{
    int temp_data[8] = {0};
    int full_0_data[8] = {0};
    int full_1_data[8] = {1,1,1,1,1,1,1,1};
    int i;
    int errno;
    errno = stNvramRead((void *)temp_data,8);
    if(temp_data[0] > 0)
    {
        stNvramDelay();
        DBG_BUFFER(MODULE_APP, LEVEL_INFO, ">> TEST_INFO   Data stored in test page >0: <<\r\n>>> TEST_INFO", 0);
        for(i = 0; i < 8; i++)
            DBG_BUFFER( MODULE_APP, LEVEL_INFO,"   %d",1, temp_data[i]);
        DBG_BUFFER( MODULE_APP, LEVEL_INFO,"<<\r\n>>> TEST_INFO   Store data to test page 2. <<\r\n>",0);
        //memcpy((void *)&temp_data,(void *)&test_full_0_data,sizeof(test_data_struct));
        errno = stNvramWrite((void *)full_0_data,8);
    }
    else
    {
        stNvramDelay();
        DBG_BUFFER(MODULE_APP, LEVEL_INFO, ">> TEST_INFO   Data stored in test page >0: <<\r\n>>> TEST_INFO", 0);
        for(i = 0; i < 8; i++)
            DBG_BUFFER( MODULE_APP, LEVEL_INFO,"   %d",1, temp_data[i]);
        DBG_BUFFER( MODULE_APP, LEVEL_INFO,"<<\r\n>>> TEST_INFO   Store data to test page 1. <<\r\n>",0);
        //memcpy((void *)&temp_data,(void *)&test_full_0_data,sizeof(test_data_struct));
        errno = stNvramWrite((void *)full_1_data,8);
    }
    DBG_BUFFER(MODULE_APP, LEVEL_INFO, ">> TEST_INFO   errno = %d <<\r\n>", 1, errno);
    return;
}

/****************************************************************************/
/* Trace interface                                                          */
/* Called by the stack                                                      */
/****************************************************************************/
/* USART2 baudrate 921000 8N1                                               */
/* DMA1 channel 7                                                           */
/****************************************************************************/

#define TRACE_QUEUE_LENGTH_EVENT      0x80
#define TRACE_QUEUE_LENGTH_BUFFER     0x80

#define TRACE_EVENT_TX                0
#define TRACE_EVENT_TX_COMPLETED      1

typedef struct _TTraceTaskBuffer
{
  char     *Pointer;
  uint16_t Length;
} TTraceTaskBuffer;

typedef struct _TTraceTaskData
{
  xTaskHandle       Handle;               /* task handle */
  xQueueHandle      QueueHandleEvent;     /* task queue */
  xQueueHandle      QueueHandleBuffer;    /* buffer queue */
  DMA_InitTypeDef   DMA_Tx_InitStructure;
  TTraceTaskBuffer  Buffer;               /* actual buffer */
} TTraceTaskData;

static TTraceTaskData trace;



/****************************************************************************/
/* DMA interrupt                                                            */
/****************************************************************************/

void DMAChannel7_IRQHandler(void)
{
  portBASE_TYPE TaskWoken = pdFALSE;
  uint8_t       Event;

  DMA_ClearFlag(DMA1_FLAG_GL7 | DMA1_FLAG_TC7 | DMA1_FLAG_HT7 | DMA1_FLAG_TE7);
  USART_DMACmd(USART2, USART_DMAReq_Tx, DISABLE);
  DMA_Cmd(DMA1_Channel7, DISABLE);

  Event = TRACE_EVENT_TX_COMPLETED;
  xQueueSendFromISR(trace.QueueHandleEvent, &Event, &TaskWoken);
  portYIELD_FROM_ISR(TaskWoken);
}


/****************************************************************************/
/* Start transmit                                                           */
/****************************************************************************/

static void traceStartTransmit(void)
{
#if 0
  if (trace.Buffer.Pointer == (char *)0 &&
      xQueueReceive(trace.QueueHandleBuffer, &trace.Buffer, 0) == pdPASS)
  {
    trace.DMA_Tx_InitStructure.DMA_MemoryBaseAddr = (uint32_t)trace.Buffer.Pointer;
    trace.DMA_Tx_InitStructure.DMA_BufferSize     = trace.Buffer.Length;
    DMA_Init(DMA1_Channel7, &trace.DMA_Tx_InitStructure);

    DMA_ITConfig(DMA1_Channel7, (DMA_IT_TC | DMA_IT_TE), ENABLE);
    USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
    DMA_Cmd(DMA1_Channel7, ENABLE);
  }
#else
	uint16_t queueSize = CycQueueSize();

	if(trace.Buffer.Pointer == (char *)0 && (queueSize > 0))
	{
		trace.Buffer.Pointer = (char*)(cyc_buffer + pRead);
		trace.DMA_Tx_InitStructure.DMA_MemoryBaseAddr = (uint32_t)(cyc_buffer + pRead);
		trace.DMA_Tx_InitStructure.DMA_BufferSize	  = queueSize;
		DMA_Init(DMA1_Channel7, &trace.DMA_Tx_InitStructure);

		DMA_ITConfig(DMA1_Channel7, (DMA_IT_TC | DMA_IT_TE), ENABLE);
		USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
		DMA_Cmd(DMA1_Channel7, ENABLE);
	}

#endif
}


/****************************************************************************/
/* Trace task                                                               */
/****************************************************************************/

static void traceTask(void *pParameters)
{
  while (1)
  {
    uint8_t Event;
    

    if (xQueueReceive(trace.QueueHandleEvent, &Event, portMAX_DELAY) == pdPASS)
    {
      switch (Event)
      {
        case TRACE_EVENT_TX:            /* new trace output */
          traceStartTransmit();
          break;
        case TRACE_EVENT_TX_COMPLETED:  /* transmit completed */
#if 0			
          if (trace.Buffer.Pointer != (char *)0)
          {
            osMemoryFree(trace.Buffer.Pointer);    /* release buffer */
            trace.Buffer.Pointer = (char *)0;
          }
#else
          if (trace.Buffer.Pointer != (char *)0)
          {
            trace.Buffer.Pointer = (char *)0;		  
					UpdateQueueRead(trace.DMA_Tx_InitStructure.DMA_BufferSize);
          }
#endif
          traceStartTransmit();
          break;
      }
    }
  }
}


/****************************************************************************/
/* Trace output (binary)                                                    */
/****************************************************************************/

int stTraceOutput(void *pData, uint16_t Length)
{
#if 0
  TTraceTaskBuffer Buffer;
  int              ReturnValue = -1;               /* out of memory */

  Buffer.Length  = Length;
  Buffer.Pointer = osMemoryClearAllocate(Buffer.Length);    /* allocate buffer */

  if (Buffer.Pointer != (char *)0)
  {
    memcpy(Buffer.Pointer, pData, Buffer.Length);  /* copy data */

    if (xQueueSend(trace.QueueHandleBuffer, &Buffer, 0) == pdPASS)
    {
      uint8_t Event;

      Event = TRACE_EVENT_TX;
      xQueueSend(trace.QueueHandleEvent, &Event, 0);
      ReturnValue = 0;      /* ok. */
    }
    else
    {
      osMemoryFree(Buffer.Pointer);
      ReturnValue = -2;     /* queue full */
    }
  }

  return(ReturnValue);

#else


		if(CycQueueWrite(pData, Length))
		{
			uint8_t Event;
			Event = TRACE_EVENT_TX;
			xQueueSend(trace.QueueHandleEvent, &Event, 0);
			return 0;
		}else
		{
			return -2;
		}

#endif
}


/****************************************************************************/
/* Trace init                                                               */
/****************************************************************************/

int stTraceInit(void)
{
  NVIC_InitTypeDef  NVIC_InitStructure;
  GPIO_InitTypeDef  GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA, ENABLE);

  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;             /* PA2 - TxD */
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  USART_DeInit(USART2);

  USART_InitStructure.USART_BaudRate            = 921600;
  USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits            = USART_StopBits_1;
  USART_InitStructure.USART_Parity              = USART_Parity_No ;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode                = USART_Mode_Tx;
  USART_Init(USART2, &USART_InitStructure);
  USART_Cmd(USART2, ENABLE);

  DMA_DeInit(DMA1_Channel7);                /* Tx channel */
  trace.DMA_Tx_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;
  trace.DMA_Tx_InitStructure.DMA_MemoryBaseAddr     = 0;
  trace.DMA_Tx_InitStructure.DMA_DIR                = DMA_DIR_PeripheralDST;
  trace.DMA_Tx_InitStructure.DMA_BufferSize         = 0;
  trace.DMA_Tx_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
  trace.DMA_Tx_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
  trace.DMA_Tx_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  trace.DMA_Tx_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
  trace.DMA_Tx_InitStructure.DMA_Mode               = DMA_Mode_Normal;
  trace.DMA_Tx_InitStructure.DMA_Priority           = DMA_Priority_Medium;
  trace.DMA_Tx_InitStructure.DMA_M2M                = DMA_M2M_Disable;
//  DMA_Init(DMA1_Channel7, &trace.DMA_Tx_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_KERNEL_INTERRUPT_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  xTaskCreate(traceTask, "trace", 0x80, NULL, (tskIDLE_PRIORITY + 1), &trace.Handle);
  trace.QueueHandleEvent  = xQueueCreate(TRACE_QUEUE_LENGTH_EVENT, sizeof(uint8_t));
  //trace.QueueHandleBuffer = xQueueCreate(TRACE_QUEUE_LENGTH_BUFFER, sizeof(TTraceTaskBuffer));

  return(0);
}
