/**
 * Copyright (c) 2015, Realsil Semiconductor Corporation. All rights reserved.
 *
 */

#include <stdint.h>
#include <string.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include <timers.h>
#include <os.h>
#include <os_sched.h>
#include <osif.h>

#ifndef DEFAULT_STACK_SIZE
#define DEFAULT_STACK_SIZE   0x600
#endif

#define UPPER_TASK_PRIORITY        (tskIDLE_PRIORITY + 4)

#define XTIMER_ID(QID, TID, CID)    (((QID) << 24) | ((TID) << 16) | (CID))
#define XTIMER_QID(XID)             ((char)((XID) >> 24))
#define XTIMER_TID(XID)             ((char)((XID) >> 16))
#define XTIMER_CID(XID)             ((short)(XID))

static xSemaphoreHandle osifTaskSemaphore;

static int osifFlags;
static int osifInterruptNest;


/****************************************************************************/
/* Initialize task schedule signal                                          */
/****************************************************************************/
void osifInitScheduleSignal(uint32_t Signal)
{
    /* Create semaphore */
    vSemaphoreCreateBinary(osifTaskSemaphore);
}

/****************************************************************************/
/* Send singal to trigger task schedule                                     */
/****************************************************************************/
void osifSendScheduleSignal(uint32_t Singal)
{
  if (osifInterruptNest)
  {
    signed portBASE_TYPE TaskWoken = pdFALSE;

    xSemaphoreGiveFromISR(osifTaskSemaphore, &TaskWoken);
    portEND_SWITCHING_ISR(TaskWoken);
  }
  else
  {
    xSemaphoreGive(osifTaskSemaphore);
  }
}

/****************************************************************************/
/* Receive singal to do task schedule                                       */
/****************************************************************************/
void osifReceiveScheduleSignal(uint32_t Singal)
{
    /*
     * Task schedule should not happen in ISR context,
     * so take the semaphore safely here.
     */
    xSemaphoreTake(osifTaskSemaphore, portMAX_DELAY);
}

/****************************************************************************/
/* Create os level task routine                                             */
/****************************************************************************/
void osifTaskCreate(const char *TaskName, void (*TaskRoutine)(void *), void *TaskParameter,
    uint16_t StackSize, uint16_t TaskPriority, void *TaskHandle)
{
    xTaskCreate(TaskRoutine, (const char *)TaskName, StackSize / sizeof(portSTACK_TYPE), TaskParameter, TaskPriority, NULL);
}


/****************************************************************************/
/* Init critical section                                                    */
/****************************************************************************/
void *osifInitCriticalSection(void)
{
    return NULL;
}

/****************************************************************************/
/* Enter critical section                                                   */
/****************************************************************************/
int osifEnterCriticalSection(void *pParam)
{
  int Flags = osifFlags;

  if (osifFlags == 0)
  {
    taskENTER_CRITICAL();
  }
  osifFlags++;

  return Flags;
}

/****************************************************************************/
/* Exit critical section                                                    */
/****************************************************************************/
void osifExitCriticalSection(void *pParam, int Flags)
{
  osifFlags--;

  if (osifFlags == 0)
  {
    taskEXIT_CRITICAL();
  }
}

/****************************************************************************/
/* Enter interrupt                                                          */
/****************************************************************************/
void osifEnterInterrupt(void)
{
  osifInterruptNest++;
}

/****************************************************************************/
/* Exit interrupt                                                           */
/****************************************************************************/
void osifExitInterrupt(void)
{
  osifInterruptNest--;
}

/****************************************************************************/
/* Allocate memory                                                          */
/****************************************************************************/
void *osifMemoryAllocate(RAM_TYPE RamType, size_t Size)
{
    return((void *)pvPortMalloc(RamType, Size));
}

/****************************************************************************/
/* Allocate zero memory                                                     */
/****************************************************************************/
void *osifMemoryClearAllocate(RAM_TYPE RamType, size_t Size)
{
    void *p;

    p = osifMemoryAllocate(RamType, Size);
    if (p)
    {
        memset(p, 0, Size);
    }

    return p;
}

/****************************************************************************/
/* Allocate aligned but not freed memory                                    */
/****************************************************************************/
void *osifMemoryAlignAllocate(RAM_TYPE RamType, size_t Size, uint8_t Alignment)
{
    void *Address;

    /* Alignment 0 means using FreeRTOS default alignment */
    if (Alignment != 0 && Alignment != 4 && Alignment != 8)
    {
        return (void *)0;
    }

    if (Alignment > portBYTE_ALIGNMENT)
    {
        Size = (Size + (Alignment - portBYTE_ALIGNMENT) + Alignment - 1) & ~(Alignment - 1);
    }

    Address = pvPortMalloc(RamType, Size);

    if (Alignment > portBYTE_ALIGNMENT)
    {
        return (void *)(((uint32_t)Address + Alignment - portBYTE_ALIGNMENT) & ~(Alignment - 1));
    }
    else
    {
        return Address;
    }
}

/****************************************************************************/
/* Free memory                                                          */
/****************************************************************************/
void osifMemoryFree(void *pBlock)
{
    vPortFree(pBlock);
}

/****************************************************************************/
/* Get system time in milliseconds                                          */
/****************************************************************************/
uint32_t osifGetSystemTime(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_RATE_MS);
}

/****************************************************************************/
/* Default timer task callback                                              */
/****************************************************************************/
static void DefaultTimerCallback(xTimerHandle xTimer)
{
    /* Did nothing here */
}

/****************************************************************************/
/* Get software timer IDs                                                   */
/****************************************************************************/
void osifGetTimerID(void **pHandle, uint8_t *pQueueID, uint8_t *pTimerID, uint16_t *pTimerChannel)
{
    portTickType xTimerID;

    if (pHandle == NULL || *pHandle == NULL)
        return;

    xTimerID = (portTickType)pvTimerGetTimerID((xTimerHandle)*pHandle);

    *pQueueID = XTIMER_QID(xTimerID);
    *pTimerID = XTIMER_TID(xTimerID);
    *pTimerChannel = XTIMER_CID(xTimerID);
}

/****************************************************************************/
/* Start software timer                                                     */
/****************************************************************************/
void osifStartTimer(void **pHandle, uint8_t QueueID, uint8_t TimerID,
        uint16_t TimerChannel, uint32_t IntervalInMS, void (*pTimerCallback)(void *))
{
    portTickType xTimerID = XTIMER_ID(QueueID, TimerID, TimerChannel);

    if (pHandle == NULL)
    {
        return;
    }

    if (pTimerCallback == NULL)
    {
        pTimerCallback = DefaultTimerCallback;
    }

    if (*pHandle == NULL)
    {
        *pHandle = xTimerCreate(NULL, (portTickType)IntervalInMS, pdFALSE, (void *)xTimerID, pTimerCallback);
        if (*pHandle == NULL)
        {
            return;
        }
    }

    if (xTimerStart((xTimerHandle)*pHandle, (portTickType)0) == pdFAIL)
    {
        return;
    }
}

/****************************************************************************/
/* Restart software timer                                                   */
/****************************************************************************/
void osifRestartTimer(void **pHandle, uint8_t QueueID, uint8_t TimerID,
        uint16_t TimerChannel, uint32_t IntervalInMS, void (*pTimerCallback)(void *))
{
    portTickType xTimerID = XTIMER_ID(QueueID, TimerID, TimerChannel);

    if (pHandle == NULL)
    {
        return;
    }

    if (pTimerCallback == NULL)
    {
        pTimerCallback = DefaultTimerCallback;
    }

    if (*pHandle == NULL)
    {
        *pHandle = xTimerCreate(NULL, (portTickType)IntervalInMS, pdFALSE, (void *)xTimerID, pTimerCallback);
        if (*pHandle == NULL)
        {
            return;
        }
    }

    if (xTimerChangePeriod((xTimerHandle)*pHandle, (portTickType)IntervalInMS, (portTickType)0) == pdFAIL)
    {
        return;
    }
}

/****************************************************************************/
/* Stop software timer                                                      */
/****************************************************************************/
void osifStopTimer(void **pHandle)
{
    if (pHandle == NULL || *pHandle == NULL)
        return;

    if (xTimerStop((xTimerHandle)*pHandle, (portTickType)0) == pdFAIL)
    {
        return;
    }
}

/****************************************************************************/
/* Delete software timer                                                    */
/****************************************************************************/
void osifDeleteTimer(void **pHandle)
{
    if (pHandle == NULL || *pHandle == NULL)
        return;

    if (xTimerDelete((xTimerHandle)*pHandle, (portTickType)0) == pdFAIL)
    {
        return;
    }

    *pHandle = NULL;
}


/****************************************************************************/
/* Main task                                                                */
/****************************************************************************/
static void UpperStackTask(void *pParameters)
{
    while (1)
    {
        osifReceiveScheduleSignal(0);
        // TODO: should register the schedule.
        osReschedule();
    }
}


int osifInit(void)
{
    osifInitScheduleSignal(0);

    /* Start upper stack tasks */
    osifTaskCreate("UpperStackTask", UpperStackTask, NULL /* TaskParameter */,
        DEFAULT_STACK_SIZE + 0x600, UPPER_TASK_PRIORITY, NULL /* TaskHandle */);

    osInit();

    return 0;
}

int osifStart(void)
{
    osStart();
    return 0;
}
