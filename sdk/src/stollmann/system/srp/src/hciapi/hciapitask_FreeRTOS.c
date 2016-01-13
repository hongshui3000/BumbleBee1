enum { __FILE_NUM__ = 0 };
/**********************************************************************!KA****
 *
 * $Header: /var/lib/cvs/sw/src/system/srp/src/hciapi/hciapitask_FreeRTOS.c,v 1.11 2013/12/16 08:38:02 mn Exp $
 *
 * File:        $RCSfile: hciapitask_FreeRTOS.c,v $
 * Version:     $Name: P_SRP1290_V1_0 $
 *
 * Archive:     $Source: /var/lib/cvs/sw/src/system/srp/src/hciapi/hciapitask_FreeRTOS.c,v $
 * Revision:    $Revision: 1.11 $
 * Date:        $Date: 2013/12/16 08:38:02 $
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
 *         Example HCI Api Task implementation
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
#include <timers.h>

#include <stdio.h>           /* printf */
#include <string.h>
#include "trace.h"
#include <common_defs.h>
#include <comapi.h>
#include <hci_code.h>
#include <hw_srp.h>          /* BlueMOD+SR */
#include <os_mem.h>

//#include <trace_binary.h>
#define HCIAPI_MID         BTRACE_MODULE_ID_A0
#define TRACE_MODULE_ID    HCIAPI_MID

#include <comuart.h>
#include "hciapi_csr.h"
#include "hciapi.h"

       THciApi  tHciApi;
extern TComUart tComUart;


/****************************************************************************/
/* HCIAPI macros                                                            */
/****************************************************************************/

#define HCIAPI_PRIORITY          (tskIDLE_PRIORITY + 5)   /* Task priorities. */
#define HCIAPI_STACK_SIZE        0x400


/****************************************************************************/
/* Events                                                                   */
/****************************************************************************/

#define HCIAPI_EVENT_UART_RX              0x01      /* data available */
#define HCIAPI_EVENT_UART_TX              0x02      /* transmit request */
#define HCIAPI_EVENT_UART_TX_COMPLETED    0x03      /* transmit completed */
#define HCIAPI_EVENT_OPEN                 0x04      /* comOpen */
#define HCIAPI_EVENT_CLOSE                0x05      /* comClose */

const unsigned char hciApiEventUartRx             = HCIAPI_EVENT_UART_RX;
const unsigned char hciApiEventUartTx             = HCIAPI_EVENT_UART_TX;
const unsigned char hciApiEventUartTxCompleted    = HCIAPI_EVENT_UART_TX_COMPLETED;
const unsigned char hciApiEventOpen               = HCIAPI_EVENT_OPEN;
const unsigned char hciApiEventClose              = HCIAPI_EVENT_CLOSE;


/****************************************************************************/
/* Start timer                                                              */
/****************************************************************************/

void hciApiTimerStart(int MSecond)
{
  xTimerChangePeriod(tHciApi.TimerHandle, (MSecond / portTICK_RATE_MS), (0/portTICK_RATE_MS));
  xTimerStart(tHciApi.TimerHandle, (0/portTICK_RATE_MS));
}


/****************************************************************************/
/* Stop timer                                                               */
/****************************************************************************/

void hciApiTimerStop(void)
{
  xTimerStop(tHciApi.TimerHandle, (0/portTICK_RATE_MS));
}


/****************************************************************************/
/* Send event                                                               */
/****************************************************************************/

static portBASE_TYPE hciApiSendEvent(const unsigned char *pEvent)
{
  portBASE_TYPE ReturnValue;

  ReturnValue = xQueueSend(tHciApi.QueueHandleEvent, pEvent, 0);
  return(ReturnValue);
}


/****************************************************************************/
/* Send event from interrupt                                                */
/****************************************************************************/

portBASE_TYPE hciApiSendEventFromISR(const unsigned char *pEvent)
{
  portBASE_TYPE ReturnValue;
  portBASE_TYPE TaskWoken;

  ReturnValue = xQueueSendFromISR(tHciApi.QueueHandleEvent, pEvent, &TaskWoken);
  portYIELD_FROM_ISR(TaskWoken);
  return(ReturnValue);
}


/****************************************************************************/
/* Update length UART Rx                                                    */
/****************************************************************************/

static void hciApiUartRxDataLengthUpdate(void)
{
  uint16_t RxOffset;
  uint16_t Length;

  RxOffset = COM_RX_BUFFER_SIZE - DMA_GetCurrDataCounter(DMA1_Channel3);

  if (tComUart.RxOffset == RxOffset)
  {
    if (tComUart.RxDataLength == COM_RX_BUFFER_SIZE)  /* overrun */
    {
      DBG_BUFFER(MODULE_APP, LEVEL_ERROR,"hciApiUartRxDataLengthUpdate: Rx overrun - data length = %4.4x",1, tComUart.RxDataLength);

      tComUart.RxDataLength = 0;
      Length = COM_RX_BUFFER_SIZE;
    }
    else
    {
      return;       /* no data */
    }
  }
  else
  {
    if (tComUart.RxOffset > RxOffset)
    {
      Length = COM_RX_BUFFER_SIZE - tComUart.RxOffset + RxOffset;
    }
    else
    {
      Length = RxOffset - tComUart.RxOffset;
    }

    tComUart.RxOffset = RxOffset;
  }

  if ((Length + tComUart.RxDataLength) > COM_RX_BUFFER_SIZE)   /* Rx overrun */
  {
    DBG_BUFFER(MODULE_APP, LEVEL_ERROR,"hciApiUartRxDataLengthUpdate: Rx overrun - data length = %4.4x",1,tComUart.RxDataLength);

    tComUart.RxDataLength  = COM_RX_BUFFER_SIZE;
    tComUart.RxWriteIndex += Length;
    tComUart.RxWriteIndex &= COM_RX_BUFFER_SIZE_MASK;
    tComUart.RxReadIndex   = tComUart.RxWriteIndex;
  }
  else
  {
    tComUart.RxDataLength += Length;         /* update length */
    tComUart.RxWriteIndex += Length;
    tComUart.RxWriteIndex &= COM_RX_BUFFER_SIZE_MASK;
  }

  if (tComUart.RxDataLength >= COM_RX_DISABLE_COUNT &&
      tComUart.RxDisabled == false)
  {
    comUartRxDisable();
  }
}


/****************************************************************************/
/* RxData available                                                         */
/****************************************************************************/

static void hciApiDataReceived(void)
{
  hciApiUartRxDataLengthUpdate();

  hciApiSendEventFromISR(&hciApiEventUartRx);
}


/****************************************************************************/
/* Start transmit                                                           */
/****************************************************************************/

static void hciApiStartTransmit(void)
{
  hciApiSendEventFromISR(&hciApiEventUartTx);
}


/****************************************************************************/
/* Transmit completed                                                       */
/****************************************************************************/

static void hciApiTransmitCompleted(void)
{
  hciApiSendEventFromISR(&hciApiEventUartTxCompleted);
}


/****************************************************************************/
/* Start transmit                                                           */
/****************************************************************************/

static void hciApiTransmitData(void)
{
  if (tComUart.TxData.pBuffer == (uint8_t)0 &&
      xQueueReceive(tHciApi.QueueHandleTxData, &tComUart.TxData, 0) == pdPASS)
  {
    comUartStartTransmit();
  }
}


/****************************************************************************/
/* Change state                                                             */
/****************************************************************************/

const char hciApiStateTextIdle[]         = "Idle";
const char hciApiStateTextAutobaudTest[] = "AutobaudTest";
const char hciApiStateTextInitializing[] = "Initializing";
const char hciApiStateTextInitComplete[] = "InitComplete";
const char hciApiStateTextReady[]        = "Ready";

/* example using RAMDATA */
uint8_t hciApiBuffer[0x20];
#if 0
static uint8_t *hciApiStateText(THciApiState State)
{
  const char *pText;

  switch (State)
  {
    case hciApiStateIdle:
      pText = hciApiStateTextIdle;
      break;
    case hciApiStateAutobaudTest:
      pText = hciApiStateTextAutobaudTest;
      break;
    case hciApiStateInitializing:
      pText = hciApiStateTextInitializing;
      break;
    case hciApiStateInitComplete:
      pText = hciApiStateTextInitComplete;
      break;
    case hciApiStateReady:
      pText = hciApiStateTextReady;
      break;
    default:
      pText = NULL;
      break;
  }
  memset(hciApiBuffer, 0, sizeof(hciApiBuffer));
  memcpy(hciApiBuffer, pText, strlen(pText));
  return(hciApiBuffer);   /* return RAM address */
}
#endif
void hciApiChangeState(THciApiState NewState)
{
  //BTRACE_PRINTF_2("hciApiChangeState: %s -> %s",
                  //BTRACE_RAMDATA1(hciApiStateText(tHciApi.State)),
                  //BTRACE_RAMDATA2(hciApiStateText(NewState)));
  tHciApi.State = NewState;
}


/****************************************************************************/
/* Timout routine                                                           */
/****************************************************************************/

static void hciApiTimoutRoutine(xTimerHandle xTimer)
{
  switch (tHciApi.State)
  {
    case hciApiStateAutobaudTest:
      if (sendCSRMessageSyncHwLayer() == false)
      {
        tHciApi.CallBack(tHciApi.pContext, comeOpenCompleted,
                         comResultTimeout, (void *)NULL, 0);
        DBG_BUFFER(MODULE_APP, LEVEL_INFO, "hciApiTimoutRoutine: comeOpenCompleted state=%d result=%d",2,
                        tHciApi.State, comResultTimeout);
      }
      break;
    case hciApiStateInitializing:
      tHciApi.CallBack(tHciApi.pContext, comeOpenCompleted,
                       comResultTimeout, (void *)NULL, 0);
      DBG_BUFFER(MODULE_APP, LEVEL_INFO, "hciApiTimoutRoutine: comeOpenCompleted state=%d result=%d",2,
                      tHciApi.State, comResultTimeout);
      break;
    default:
      break;
  }
}


/****************************************************************************/
/* Send data                                                                */
/****************************************************************************/

void hciApiDataSend(uint8_t *pBuffer, uint32_t Length)
{
  TComData TxData;

  TxData.pBuffer = pBuffer;
  TxData.Length  = Length;
  xQueueSend(tHciApi.QueueHandleTxData, &TxData, 0);

  hciApiSendEvent(&hciApiEventUartTx);
}


/****************************************************************************/
/* Send data indication                                                     */
/****************************************************************************/

static void hciApiSendDataIndication(void)
{
  TComResult Result;

  vTaskSuspendAll();
  Result = tHciApi.CallBack(tHciApi.pContext, comeDataIndication,
                            comResultSuccess,
                            (void *)tHciApi.RxData.pBuffer, tHciApi.RxData.Length);
  //DBG_BUFFER(MODULE_APP, LEVEL_INFO, "hciApiSendDataIndication: comeDataIndication buffer=%8.8x length=%4.4x",
                  //2, tHciApi.RxData.pBuffer, tHciApi.RxData.Length);
  if (Result == comResultNoResources)
  {
    tHciApi.DataIndicationBlocked = true;
    if (tHciApi.DataIndication == 0)
    {
      /* No response pending - should not happened */
      DBG_BUFFER(MODULE_APP, LEVEL_ERROR,"hciApiSendDataIndication: first comeDataIndication blocked",0);
    }
  }
  else
  {
    if (xQueueSend(tHciApi.QueueHandleRxData, &tHciApi.RxData, 0) == pdPASS)
    {

      tHciApi.DataIndicationBlocked = false;

      tHciApi.DataIndication += tHciApi.RxData.Length;

      tHciApi.RxData.Length   = 0;
      tHciApi.RxData.pBuffer  = NULL;
    }
    else
    {
      DBG_BUFFER(MODULE_APP, LEVEL_ERROR,"hciApiSendDataIndication: Not enough queue space",0);
    }
  }
  xTaskResumeAll();
}


/****************************************************************************/
/* Handle Rx packet                                                         */
/****************************************************************************/

static void hciApiHandlePacket(void)
{
  if (tHciApi.State == hciApiStateInitComplete)
  {
    tHciApi.CallBack(tHciApi.pContext, comeOpenCompleted,
                     comResultSuccess, (void *)NULL, 0);
    DBG_BUFFER(MODULE_APP, LEVEL_ERROR,"hciApiHandlePacket: comeOpenCompleted result=%d",1, comResultSuccess);

    hciApiChangeState(hciApiStateReady);
  }

  if (tHciApi.State == hciApiStateReady)
  {
    hciApiSendDataIndication();
  }
  else
  {
    if (tHciApi.RxData.pBuffer[0] == EVENT_PKT)
    {
      hciCsrHandleEventPacket(tHciApi.RxData.pBuffer, tHciApi.RxData.Length);
    }
    else
    {
      DBG_BUFFER(MODULE_APP, LEVEL_ERROR,"hciApiHandlePacket: Not event packet received",0);
    }

    osMemoryFree(tHciApi.RxData.pBuffer);    /* release buffer */
    tHciApi.RxData.pBuffer = NULL;
    tHciApi.RxData.Length  = 0;
  }
}


/****************************************************************************/
/* Get buffer                                                               */
/****************************************************************************/

static bool hciApiGetBuffer(void)
{
  tHciApi.RxData.pBuffer = (uint8_t *)osMemoryClearAllocate(RAM_TYPE_DATA_ON, tHciApi.HCIDataLength + tHciApi.HCIHeaderLength);
  if (tHciApi.RxData.pBuffer == NULL)
  {
    return(false);
  }
  tHciApi.RxData.Length = tHciApi.HCIPos;   /* packet type included */
  memcpy((void *)tHciApi.RxData.pBuffer, (void *)tHciApi.HCIBuffer, tHciApi.HCIPos);

  return(true);
}


/****************************************************************************/
/* Packet assembler                                                         */
/****************************************************************************/

static void hciApiPacketAssembler(void)
{
  uint32_t RxDataLength;
  uint32_t DataLength;
  uint8_t  RxChar;

  if (tHciApi.DataIndicationBlocked)
    return;

  taskENTER_CRITICAL();
  RxDataLength = tComUart.RxDataLength;
  taskEXIT_CRITICAL();

  DataLength = 0;

  while (RxDataLength)
  {
    if (tHciApi.HCIPos == 0 || tHciApi.HCIPos < tHciApi.HCIHeaderLength)
    {
      RxChar = tComUart.RxBuffer[tComUart.RxReadIndex];
      tComUart.RxReadIndex++;
      tComUart.RxReadIndex &= COM_RX_BUFFER_SIZE_MASK;
      RxDataLength--;
      DataLength++;

      if (tHciApi.HCIPos == 0)                 /* packet type */
      {
        tHciApi.HCIBuffer[0] = RxChar;
        tHciApi.HCIPos++;

        switch (RxChar)
        {
          case ACL_PKT:
            tHciApi.HCIHeaderLength = ACL_HDR_LENGTH;
            break;
          case EVENT_PKT:
            tHciApi.HCIHeaderLength = EVENT_HDR_LENGTH;
            break;
          default:
            DBG_BUFFER(MODULE_APP, LEVEL_ERROR,"hciApiPacketAssembler: Unknown HCI packet indicator (%x)", 1, RxChar);

            tHciApi.CallBack(tHciApi.pContext, comeError,
                             comResultUnspecified, (void *)NULL, 0);
            DBG_BUFFER(MODULE_APP, LEVEL_ERROR,"hciApiPacketAssembler: comeError result=%d", 1,comResultUnspecified);
            tHciApi.HCIPos = 0;
            break;
        }
      }
      else
      {
        tHciApi.HCIBuffer[tHciApi.HCIPos++] = RxChar;

        if (tHciApi.HCIPos == tHciApi.HCIHeaderLength)
        { /* get length and upstream buffer */
          switch (tHciApi.HCIBuffer[0])
          {
            case ACL_PKT:
              tHciApi.HCIDataLength = CHAR2SHORT(&tHciApi.HCIBuffer[3]);
              if (tHciApi.HCIDataLength == 0)
              {
                tHciApi.HCIPos = 0;
              }
              break;
            case EVENT_PKT:
              tHciApi.HCIDataLength = (uint16_t)tHciApi.HCIBuffer[2];
              break;
          }

          if (tHciApi.HCIDataLength && (hciApiGetBuffer() == false))
          {
            break;
          }
        }
      }
    }
    else
    {
      uint32_t CopyLength;

      if ((tHciApi.RxData.pBuffer == NULL) && (hciApiGetBuffer() == false))
      {
        break;
      }
      CopyLength = RxDataLength;              /* try to copy all available data  */
      if (CopyLength > tHciApi.HCIDataLength) /* enough space in upstream buffer?*/
      {
        CopyLength = tHciApi.HCIDataLength;
      }
      if (CopyLength > COM_RX_BUFFER_SIZE - tComUart.RxReadIndex) /* index overflow ?*/
      {
        CopyLength = COM_RX_BUFFER_SIZE - tComUart.RxReadIndex;
      }

      /* copy data from interrupt buffer to buffer */
      if (CopyLength)
      {
        memcpy((void *)&tHciApi.RxData.pBuffer[tHciApi.RxData.Length],
               (void *)&tComUart.RxBuffer[tComUart.RxReadIndex], CopyLength);
      }

      tHciApi.RxData.Length += CopyLength;
      RxDataLength          -= CopyLength;
      DataLength            += CopyLength;
      tComUart.RxReadIndex  += CopyLength;
      tComUart.RxReadIndex  &= COM_RX_BUFFER_SIZE_MASK;
      tHciApi.HCIDataLength -= CopyLength;


      if (tHciApi.HCIDataLength == 0)
      {
        tHciApi.HCIPos = 0;
        hciApiHandlePacket();
        if (tHciApi.DataIndicationBlocked)
        {
          comUartRxDisable();
          break;
        }
      }
    }
  }

  taskENTER_CRITICAL();
  tComUart.RxDataLength -= DataLength;

  if (tComUart.RxDisabled == true)                 /* flow control */
  {
    if (!tHciApi.DataIndicationBlocked &&
        tComUart.RxDataLength < COM_RX_ENABLE_COUNT)
    {
      comUartRxEnable();
    }
  }
  taskEXIT_CRITICAL();
}


/****************************************************************************/
/* TASK                                                                     */
/****************************************************************************/

static void hciApiTask(void *pParameters)
{
  char Event;

  tHciApi.State = hciApiStateIdle;

  while (true)
  {
    if (xQueueReceive(tHciApi.QueueHandleEvent, &Event, portMAX_DELAY) == pdPASS)
    {
      switch (Event)
      {
        case HCIAPI_EVENT_UART_RX:                    /* RxData available */
          hciApiPacketAssembler();
          break;
        case HCIAPI_EVENT_UART_TX:                    /* transmit data */
          hciApiTransmitData();
          break;
        case HCIAPI_EVENT_UART_TX_COMPLETED:          /* transmit completed */
          if (tHciApi.State == hciApiStateReady)
          {
            tHciApi.CallBack(tHciApi.pContext, comeDataTransmitted, comResultSuccess,
                             (void *)tComUart.TxData.pBuffer, tComUart.TxData.Length);
            //DBG_BUFFER(MODULE_APP, LEVEL_INFO, "hciApiTask: comeDataTransmitted buffer=%8.8x length=%4.4x",
                            //2, tComUart.TxData.pBuffer, tComUart.TxData.Length);
            tComUart.TxData.pBuffer = (uint8_t)0;
            hciApiTransmitData();
          }
          else
          {
            if (tHciApi.State == hciApiStateInitComplete)
            {
              hciApiTimerStop();
              vTaskDelay(portTICK_RATE_MS * 10);      /* 10 ms delay */
              comUartChangeBaudrate();
            }

            osMemoryFree(tComUart.TxData.pBuffer);       /* release buffer */
            tComUart.TxData.pBuffer = NULL;
            tComUart.TxData.Length  = 0;
          }
          break;
        case HCIAPI_EVENT_OPEN:             /* Open */
          hwBluetoothReset();
          hciApiChangeState(hciApiStateAutobaudTest);
          sendCSRMessageSyncHwLayer();
          break;
        case HCIAPI_EVENT_CLOSE:            /* Close */
          tHciApi.CallBack(tHciApi.pContext, comeCloseCompleted,
                           comResultSuccess, (void *)NULL, 0);
          DBG_BUFFER(MODULE_APP, LEVEL_ERROR,"hciApiTask: comeCloseCompleted result=%d",1, comResultSuccess);
          break;
        default:
          DBG_BUFFER(MODULE_APP, LEVEL_ERROR,"hciApiTask: Unknown event (%x)", 1,Event);
          break;
      }
    }
  }
}


/****************************************************************************/
/* HCIAPI Init                                                              */
/****************************************************************************/

void hciApiInit(void)
{
  xTaskCreate(hciApiTask, "HciApi", HCIAPI_STACK_SIZE/sizeof(portSTACK_TYPE),
                                    NULL, HCIAPI_PRIORITY, &tHciApi.Handle);

  tHciApi.QueueHandleEvent  = xQueueCreate(32, sizeof(unsigned char));
  tHciApi.QueueHandleTxData = xQueueCreate(16, sizeof(TComData));
  tHciApi.QueueHandleRxData = xQueueCreate(16, sizeof(TComData));
  tHciApi.TimerHandle = xTimerCreate("HciAPiTimer", (50 / portTICK_RATE_MS), pdFALSE, 0, hciApiTimoutRoutine);
}


/****************************************************************************/
/* Open - called by COMAPI Client                                           */
/****************************************************************************/
/* Function name  : comOpen                                                 */
/* Description    : opens a communication channel                           */
/* Return type    : TComResult       see definition of enum                 */
/* Argument       : pszDeviceName    the name of device to open             */
/* Argument       : pszPortName      specific channel within the device     */
/* Argument       : callBack         pointer to client callback function    */
/* Argument       : callBackContext  a client specific context pointer,     */
/* will be used as argument for the callback                                */
/* Argument       : phCom            pointer to a var of type HCOM used as id*/
/* in all function calls                                                    */
/****************************************************************************/

COMENTRY comOpen(char        *pszDeviceName,
                 char        *pszParameter,
                 PComCallBack callBack,
                 void        *callBackContext,
                 HCOM        *phCom)
{
  if (tHciApi.CallBack != (PComCallBack)0)
  {
    return(comResultInvalidState);
  }
  else
  {
    tComUart.DataReceived      = hciApiDataReceived;
    tComUart.StartTransmit     = hciApiStartTransmit;
    tComUart.TransmitCompleted = hciApiTransmitCompleted;

    comUartInit();

    tHciApi.CallBack = callBack;
    tHciApi.pContext = callBackContext;

    *phCom = (HCOM)&tHciApi;

    hciApiSendEvent(&hciApiEventOpen);

    return(comResultPending);
  }
}


/****************************************************************************/
/* Close - called by COMAPI Client                                          */
/****************************************************************************/
/* Function name  : comClose                                                */
/* Description    : close the communication channel identified by hCom      */
/* Return type    : TComResult              see definition of enum          */
/* Argument       : hCom                    the handle provided by comOpen()*/
/****************************************************************************/

COMENTRY comClose (HCOM hCom)
{
  hciApiSendEvent(&hciApiEventClose);
  return(comResultSuccess);
}


/****************************************************************************/
/* Write - called by COMAPI Client                                          */
/****************************************************************************/
/* Function name : comWrite                                                 */
/* Description   : writes uBufferLength chars to the communication channel  */
/* Return type   : TComResult              see definition of enum           */
/* Argument      : hCom                    the handle provided by comOpen() */
/* Argument      : pvBuffer                pointer to a buffer containing the data*/
/* Argument      : uBufferLength           number of byte to write          */
/****************************************************************************/

COMENTRY comWrite (HCOM hCom, uint8_t *pvBuffer, uint32_t uBufferLength)
{
  TComData TxData;

  TxData.pBuffer = pvBuffer;
  TxData.Length  = uBufferLength;
  if (xQueueSend(tHciApi.QueueHandleTxData, &TxData, 0) == pdPASS)
  {
    xQueueSend(tHciApi.QueueHandleEvent, &hciApiEventUartTx, 0);
    return(comResultSuccess);
  }
  else
  {
    return(comResultNoResources);
  }
}


/****************************************************************************/
/* confirma last indicated RX buffer                                        */
/****************************************************************************/
/* Function name : comResponse                                              */
/* Description   : Indicate consumptuion of last signaled RX buffer         */
/* Return type   : TComResul               see definition of enum           */
/* Argument      : hCom                    the handle provided by comOpen() */
/* Argument      : pvBuffer                pointer to a buffer containing the data*/
/****************************************************************************/

COMENTRY comResponse (HCOM hCom, uint8_t *pvBuffer)
{
  TComData RxData;

  vTaskSuspendAll();
  if (xQueueReceive(tHciApi.QueueHandleRxData, &RxData, 0) == pdPASS)
  {
    if (RxData.pBuffer == (uint8_t *)pvBuffer)
    {
      tHciApi.DataIndication -= RxData.Length;

      osMemoryFree(RxData.pBuffer);    /* release buffer */

      if (tComUart.RxDataLength)    /* data in buffer? */
        hciApiSendEvent(&hciApiEventUartRx);
    }
    else
    {
      DBG_BUFFER(MODULE_APP, LEVEL_ERROR, "comResponse: wrong buffer (%8.8x expected %8.8x)",2, pvBuffer, RxData.pBuffer);
    }
  }
  else
  {
    DBG_BUFFER(MODULE_APP, LEVEL_ERROR, "comResponse: Not enough queue space", 0);
  }

  if (tHciApi.DataIndicationBlocked)
  {
    if (tHciApi.RxData.pBuffer == NULL)      /* Should not happen */
    {
      DBG_BUFFER(MODULE_APP, LEVEL_ERROR, "comResponse: DataIndication blocked without data",0);
    }
    else
    {
      hciApiSendDataIndication();
    }
  }
  xTaskResumeAll();

  return(comResultSuccess);
}

