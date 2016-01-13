/**********************************************************************!KA****
 *
 * $Header: /var/lib/cvs/sw/src/system/srp/src/uart/comuart.h,v 1.2 2013/10/17 09:30:05 mn Exp $
 *
 * File:        $RCSfile: comuart.h,v $
 * Version:     $Name: P_SRP1290_V1_0 $
 *
 * Archive:     $Source: /var/lib/cvs/sw/src/system/srp/src/uart/comuart.h,v $
 * Revision:    $Revision: 1.2 $
 * Date:        $Date: 2013/10/17 09:30:05 $
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
 *         Example ComApi Uart implementation
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


#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <stm32f10x.h>
#include <stdio.h>           /* printf */
#include <stdint.h>
#include <stdbool.h>



#include <hw_srp.h>          /* BlueMOD+SR */


/****************************************************************************/
/* Com macros                                                               */
/****************************************************************************/

#define COM_TIM_PRESCALER        40      /* 40 bits timeout */

#define COM_INIT_BAUDRATE        115200  /* initial baudrate */
#define COM_MAX_BAUDRATE         921600  /* max. baudrate */

#define COM_RX_BUFFER_SIZE       0x800   /* Rx buffer size */
#define COM_RX_BUFFER_SIZE_MASK  0x7FF   /* mask */
#define COM_RX_ENABLE_COUNT      0x300   /* Enable Rx */
#define COM_RX_DISABLE_COUNT     0x400   /* Disable Rx */
#define COM_RX_ENABLE_COUNT      0x300   /* Enable Rx */


/****************************************************************************/
/* data                                                                     */
/****************************************************************************/

typedef void (*TCallBack) (void);

typedef struct _TData
{
  uint8_t  *pBuffer;
  uint32_t Length;
} TComData;

typedef struct _TComUart
{
  TComData                  TxData;
  uint32_t                  BaudRate;
  USART_InitTypeDef         USART_InitStructure;
  DMA_InitTypeDef           DMA_Rx_InitStructure;
  DMA_InitTypeDef           DMA_Tx_InitStructure;
  TIM_TimeBaseInitTypeDef   TIM_TimeBaseInitStructure;  /* RxD */
  EXTI_InitTypeDef          EXTI_InitStructure;         /* RxD */
  bool                      RxDisabled;
  bool                      TxDisabled;

  uint32_t                  RxDataLength;
  uint32_t                  RxOffset;
  uint16_t                  RxReadIndex;
  uint16_t                  RxWriteIndex;
  uint8_t                   RxBuffer[COM_RX_BUFFER_SIZE];

  TCallBack                 DataReceived;
  TCallBack                 StartTransmit;
  TCallBack                 TransmitCompleted;
} TComUart;
typedef TComUart *PComUart;


/****************************************************************************/
/* Prototypes                                                               */
/****************************************************************************/

/* comuart.c */
void comUartChangeBaudrate(void);
void comUartStartTransmit(void);
void comUartRxDisable(void);
void comUartRxEnable(void);
void comUartInit(void);
