/**
 * Copyright (c) 2015, Realsil Semiconductor Corporation. All rights reserved.
 *
 */

#ifndef __OS_SCHED_H
#define __OS_SCHED_H

#include <stdint.h>
#include <os_queue.h>
#include <os_message.h>

#ifdef __cplusplus
extern "C" {
#endif

#define OS_QUEUE_FREE            0x00
#define OS_QUEUE_CREATED         0x01
#define OS_ENTRY_EXISTS          0x10

/* Coroutine entry definition  */
typedef void (*TCoroutineEntry)(void *Parameter);

typedef struct tagOsQueue
{
    uint16_t        Flags;
    TCoroutineEntry Entry;
    void *          Parameter;
    QUEUE_T         MessageQueue;
} TOsQueue, *POsQueue;

typedef struct tagMessageQueueElement
{
    struct tagMessageQueueElement   *Next;
    MESSAGE_T                       Message;
} TMessageQueueElement, *PMessageQueueElement;

typedef struct tagEntryQueueElement
{
    struct tagEntryQueueElement *Next;
    POsQueue                    pOsQueue;
} TEntryQueueElement, *PEntryQueueElement;

typedef struct tagOS
{
    uint8_t     Rescheduling;           /* Mark as Task is under rescheduling */

    uint8_t     OsQueueTableSize;       /* Size of OsQueueTable */
    uint8_t     FreeMessageQueueSize;   /* Size of FreeMessageQueue */
    uint8_t     FreeEntryQueueSize;     /* Size of FreeEntryQueue */

    TOsQueue    *OsQueueTable;          /* OsQueue Table for message queue */
    QUEUE_T     FreeMessageQueue;       /* Cache free MessageQueueElement that will be queued in MessageQueue */
    QUEUE_T     FreeEntryQueue;         /* Cache free EntryQueueElement that will be queued in EntryQueue */
    QUEUE_T     EntryQueue;             /* Os Queue with Coroutine Entry will be queued here if messages received */
} TOS;

uint32_t osGetSystemTime(void);

int osCoroutineCreate(uint8_t *pQueueHandle, TCoroutineEntry CoroutineEntry, void *CoroutineParameter);

void osTaskCreate(const char *TaskName, void (*TaskRoutine)(void *), void *TaskParameter,
    uint16_t StackSize, uint16_t TaskPriority, void *TaskHandle);

void osTriggerSchedule(void);

void osReschedule(void);

#ifdef __cplusplus
}
#endif

#endif /* __OS_SCHED_H */
