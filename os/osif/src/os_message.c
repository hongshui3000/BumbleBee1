/**
 * Copyright (c) 2015, Realsil Semiconductor Corporation. All rights reserved.
 *
 */

#include <stdint.h>
#include <os.h>
#include <os_queue.h>
#include <os_sched.h>
#include <os_message.h>
#include <os_intr.h>
#include <upper_stack_global.h>

int osMessageQueueHandleCheck(uint8_t QueueHandle)
{
    if (QueueHandle >= OS_FIRST_QUEUE_ID + os.OsQueueTableSize ||
        QueueHandle < OS_FIRST_QUEUE_ID)
    {
        return ER_QID;
    }

    return RET_OK;
}

int osMessageQueueCreate(uint8_t *pQueueHandle)
{
    uint8_t QueueID;

    for (QueueID = 0; QueueID < os.OsQueueTableSize; QueueID++)
    {
        if (!(os.OsQueueTable[QueueID].Flags & OS_QUEUE_CREATED))
        {
            os.OsQueueTable[QueueID].Flags |= OS_QUEUE_CREATED;
            *pQueueHandle = QueueID + OS_FIRST_QUEUE_ID;
            break;
        }
    }

    if (QueueID == os.OsQueueTableSize)
    {
        return ER_QUE;
    }
    else
    {
        return RET_OK;
    }
}

int osMessageQueueElementCountGet(uint8_t QueueHandle, uint16_t *pElementCount)
{
    uint8_t QueueID;

    if (osMessageQueueHandleCheck(QueueHandle))
    {
        return ER_QID;
    }
    QueueID = QueueHandle - OS_FIRST_QUEUE_ID;

    if (!(os.OsQueueTable[QueueID].Flags & OS_QUEUE_CREATED))
    {
        return ER_QUE;
    }

    *pElementCount = os.OsQueueTable[QueueID].MessageQueue.Count;

    return RET_OK;
}

int osMessageSend(uint8_t QueueHandle, MESSAGE_P pMessage)
{
    uint8_t                 QueueID;
    POsQueue                pOsQueue;
    PMessageQueueElement    pMessageQueueElement;
    PEntryQueueElement      pEntryQueueElement;
    int                     s;
    bool                    TriggerRescheduling = false;

    if (osMessageQueueHandleCheck(QueueHandle))
    {
        return ER_QID;
    }
    QueueID = QueueHandle - OS_FIRST_QUEUE_ID;

    if (!(os.OsQueueTable[QueueID].Flags & OS_QUEUE_CREATED))
    {
        return ER_QID;
    }

    pOsQueue = &os.OsQueueTable[QueueID];

    s = osInterruptDisable();
    pMessageQueueElement = (PMessageQueueElement)osQueueOut(&os.FreeMessageQueue);
    osInterruptEnable(s);
    if (pMessageQueueElement == (PMessageQueueElement)0)
    {
        return ER_ZWS;
    }
    pMessageQueueElement->Message = *pMessage; /* copy message */
    s = osInterruptDisable();
    osQueueIn(&pOsQueue->MessageQueue, pMessageQueueElement);
    osInterruptEnable(s);

    if (pOsQueue->Flags & OS_ENTRY_EXISTS)
    {
        s = osInterruptDisable();
        pEntryQueueElement = (PEntryQueueElement)osQueueOut(&os.FreeEntryQueue);
        osInterruptEnable(s);
        if (pEntryQueueElement == (PEntryQueueElement)0)
        {
            return ER_ZWS;
        }
        pEntryQueueElement->pOsQueue = pOsQueue;
        s = osInterruptDisable();

        if (os.EntryQueue.Count == 0)
        {
            TriggerRescheduling = true;
        }
        osQueueIn(&os.EntryQueue, pEntryQueueElement);
        osInterruptEnable(s);
    }

    if (TriggerRescheduling)
    {
        osTriggerSchedule();
    }

    return RET_OK;
}

int osMessageReceive(uint8_t QueueHandle, MESSAGE_P pMessage)
{
    uint8_t                 QueueID;
    POsQueue                pOsQueue;
    PMessageQueueElement    pMessageQueueElement;
    int                     s;

    if (osMessageQueueHandleCheck(QueueHandle))
    {
        return ER_QID;
    }
    QueueID = QueueHandle - OS_FIRST_QUEUE_ID;

    if (!(os.OsQueueTable[QueueID].Flags & OS_QUEUE_CREATED))
    {
        return ER_QID;
    }

    pOsQueue = &os.OsQueueTable[QueueID];
    s = osInterruptDisable();
    if (pOsQueue->MessageQueue.Count == 0)
    {
        osInterruptEnable(s);
        return ER_ZWS;
    }
    pMessageQueueElement = (PMessageQueueElement)osQueueOut(&pOsQueue->MessageQueue);
    osInterruptEnable(s);

    if (pMessageQueueElement == (PMessageQueueElement)0)
    {
        return ER_ZWS;
    }

    *pMessage = pMessageQueueElement->Message; /* Message copy */

    s = osInterruptDisable();
    osQueueIn(&os.FreeMessageQueue, pMessageQueueElement); /* MessageQueueElement free */
    osInterruptEnable(s);

    return RET_OK;
}
