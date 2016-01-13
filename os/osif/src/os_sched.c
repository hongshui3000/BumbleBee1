/**
 * Copyright (c) 2015, Realsil Semiconductor Corporation. All rights reserved.
 *
 */

#include <stdint.h>
#include <os.h>
#include <os_message.h>
#include <os_sched.h>
#include <os_intr.h>
#include <osif.h>
#include <upper_stack_global.h>

uint32_t osGetSystemTime(void)
{
    return osifGetSystemTime();
}

int osCoroutineCreate(uint8_t *pQueueHandle, TCoroutineEntry CoroutineEntry, void *CoroutineParameter)
{
    int Error = osMessageQueueCreate(pQueueHandle);

    if (Error == RET_OK)
    {
        uint8_t QueueID = *pQueueHandle - OS_FIRST_QUEUE_ID;

        os.OsQueueTable[QueueID].Entry       = CoroutineEntry;
        os.OsQueueTable[QueueID].Parameter   = CoroutineParameter;
        os.OsQueueTable[QueueID].Flags      |= OS_ENTRY_EXISTS;
    }

    return Error;
}

void osTaskCreate(const char *TaskName, void (*TaskRoutine)(void *), void *TaskParameter,
    uint16_t StackSize, uint16_t TaskPriority, void *TaskHandle)
{
    osifTaskCreate(TaskName, TaskRoutine, TaskParameter, StackSize, TaskPriority, TaskHandle);
}

void osTriggerSchedule(void)
{
    if (os.Rescheduling)
    {
        return;
    }

    osifSendScheduleSignal(0);
}

void osReschedule(void)
{
    PEntryQueueElement  pEntryQueueElement;
    POsQueue            pOsQueue;
    TCoroutineEntry     CoroutineEntry;
    void *              Parameter;
    int                 s;

    s = osInterruptDisable();
    if (os.Rescheduling == TRUE)
    {
        osInterruptEnable(s);
        return;
    }
    os.Rescheduling = TRUE;

    while (os.EntryQueue.Count)
    {
        pEntryQueueElement = (PEntryQueueElement)osQueueOut(&os.EntryQueue);
        osInterruptEnable(s);
        pOsQueue        = pEntryQueueElement->pOsQueue;
        CoroutineEntry  = pOsQueue->Entry;
        Parameter       = pOsQueue->Parameter;
        s = osInterruptDisable();
        osQueueIn(&os.FreeEntryQueue, pEntryQueueElement);
        osInterruptEnable(s);

        CoroutineEntry(Parameter);

        s = osInterruptDisable();
    }

    os.Rescheduling = FALSE;

    osInterruptEnable(s);
}
