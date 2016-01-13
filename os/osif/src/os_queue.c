/**
 * Copyright (C) 2015 Realsil Semiconductor Corporation. All rights reserved.
 *
 */

#include <os_queue.h>

/**
 * @brief  Enqueue an element to the queue
 *
 * @param  pQueue: queue header
 * @param  pElement: queue element
 * @return
 */
void osQueueIn(QUEUE_P pQueue, void *pElement)
{
    ELEMENT_P pQueueElement = (ELEMENT_P)pElement;

    pQueueElement->Next = (ELEMENT_P)0;

    if (pQueue->Last == (ELEMENT_P)0)
    {
        pQueue->First = pQueueElement;
    }
    else
    {
        pQueue->Last->Next = pQueueElement;
    }

    pQueue->Last = pQueueElement;
    pQueue->Count++;
}

/**
 * @brief  Dequeue an element from the queue
 *
 * @param  pQueue: queue header
 * @return  first element from the queue
 */
void *osQueueOut(QUEUE_P pQueue)
{
    ELEMENT_P pFirst = pQueue->First;

    if (pFirst != (ELEMENT_P)0)
    {
        pQueue->First = pFirst->Next;
        if (pQueue->First == (ELEMENT_P)0)
        {
            pQueue->Last = (ELEMENT_P)0;
        }
        pQueue->Count--;
    }

    return pFirst;
}

/**
 * @brief  Insert an element to the queue
 *
 * @param  pQueue: queue header
 * @param  pElement: the element to insert behind
 * @param  pNewElement: the inserted element
 * @return
 */
void osQueueInsert(QUEUE_P pQueue, void *pElement, void *pNewElement)
{
    ELEMENT_P pQueueElement = (ELEMENT_P)pElement;
    ELEMENT_P pNewQueueElement = (ELEMENT_P)pNewElement;

    /* if pQueue is empty, insert pNewElement directly */
    if (pQueue->First == (ELEMENT_P)0)
    {
        pQueue->First = pNewQueueElement;
        pQueue->Last = pNewQueueElement;
        pNewQueueElement->Next = (ELEMENT_P)0;
    }
    /* if pElement is null, insert pNewElement as first element */
    else if (pQueueElement == (ELEMENT_P)0)
    {
        pNewQueueElement->Next = pQueue->First;
        pQueue->First = pNewQueueElement;
    }
    /* caller should make sure that pElement belongs to pQueue */
    else
    {
        pNewQueueElement->Next = pQueueElement->Next;
        pQueueElement->Next = pNewQueueElement;
    }

    pQueue->Count++;
}

/**
 * @brief  Delete an element from the queue
 *
 * @param  pQueue: queue header
 * @param  pElement: the deleted element
 * @return
 */
void osQueueDelete(QUEUE_P pQueue, void *pElement)
{
    ELEMENT_P pQueueElement = (ELEMENT_P)pElement;
    ELEMENT_P pCurrentElement = pQueue->First;
    ELEMENT_P pNextElement;

    /* queue is empty */
    if (pCurrentElement == (ELEMENT_P)0)
    {
        return;
    }

    /* first element is the one to be deleted */
    if (pCurrentElement == pQueueElement)
    {
        pQueue->First = pCurrentElement->Next;
        if (pQueue->First == (ELEMENT_P)0)
        {
            pQueue->Last = (ELEMENT_P)0;
        }

        pQueue->Count--;
        return;
    }

    while ((pNextElement = pCurrentElement->Next) != (ELEMENT_P)0 &&
            pNextElement != pQueueElement)
    {
        pCurrentElement = pNextElement;
    }

    if (pNextElement == (ELEMENT_P)0)
    {
        return;
    }

    pCurrentElement->Next = pNextElement->Next;
    if (pCurrentElement->Next == (ELEMENT_P)0)
    {
        pQueue->Last = pCurrentElement;
    }
    pQueue->Count--;
}
