enum { __FILE_NUM__ = 0 };




#include "app_queue.h"

void AppQueueIn(QUEUE_P QueuePtr, void *pQueueElement)
{
    ELEMENT_P QueueElementPtr = (ELEMENT_P)pQueueElement;
    ELEMENT_P LastPtr;

    if ((LastPtr = QueuePtr->Last) == (ELEMENT_P)0)    /* if queue is empty,  */
        QueuePtr->First = QueueElementPtr;    /* q->first = q->last = new entry */
    else                                    /* if it is not empty, new entry  */
        LastPtr->Next  = QueueElementPtr;     /* is next from last entry        */
    QueuePtr->Last = QueueElementPtr;
    QueueElementPtr->Next = (ELEMENT_P)0;
    QueuePtr->ElementCount++;               /* increment element count        */
}

void *AppQueueOut(QUEUE_P QueuePtr)
{
    ELEMENT_P FirstPtr;

    if ((FirstPtr = QueuePtr->First) != (ELEMENT_P)0)
    {
        /* if queue not empty and    */
        /* it is the last entry      */
        if ((QueuePtr->First = FirstPtr->Next) == (ELEMENT_P)0)
            QueuePtr->Last = (ELEMENT_P)0;           /* set queue empty           */
        QueuePtr->ElementCount--;                  /* decrement element count   */
    }
    return (FirstPtr);
}

void AppQueueInsert(QUEUE_P QueuePtr, void *pQueueElement, void *pNewQueueElement)
{
    ELEMENT_P QueueElementPtr = (ELEMENT_P)pQueueElement;
    ELEMENT_P NewQueueElementPtr = (ELEMENT_P)pNewQueueElement;
    ELEMENT_P NextPtr;

    if (QueueElementPtr == (ELEMENT_P)0)          /* insert first element    */
    {
        NextPtr = QueuePtr->First;
        QueuePtr->First = NewQueueElementPtr;
        NewQueueElementPtr->Next = NextPtr;
        if (QueuePtr->Last == (ELEMENT_P)0)         /* queue is empty ?        */
            QueuePtr->Last = NewQueueElementPtr;
        QueuePtr->ElementCount++;                   /* increment element count */
        return;
    }
    /* if ele is last ele -> new ele is last ele */
    if (QueueElementPtr->Next == (ELEMENT_P)0)
    {
        AppQueueIn(QueuePtr, NewQueueElementPtr);
        return;
    }

    NextPtr = QueueElementPtr->Next;
    QueueElementPtr->Next = NewQueueElementPtr;
    NewQueueElementPtr->Next = NextPtr;
    QueuePtr->ElementCount++;                     /* increment element count */
    return;
}

void AppQueueDelete(QUEUE_P QueuePtr, void *pQueueElement)
{
    ELEMENT_P QueueElementPtr = (ELEMENT_P)pQueueElement;
    ELEMENT_P NextElementPtr;
    ELEMENT_P NextPtr;

    if ((NextElementPtr = QueuePtr->First) == (ELEMENT_P)0)
        return;                                  /* queue is empty             */

    if (NextElementPtr == QueueElementPtr)     /* if first is the elem to be */
    {
        /* deleted set first to next  */
        /* only one element ?         */
        if ((QueuePtr->First = NextElementPtr->Next) == (ELEMENT_P)0)
            QueuePtr->Last = (ELEMENT_P)0;         /* set queue empty            */
        QueuePtr->ElementCount--;                /* decrement element count    */
        return;
    }
    while ((NextPtr = NextElementPtr->Next) != (ELEMENT_P)0 &&
            NextPtr != QueueElementPtr)
        NextElementPtr = NextPtr;
    if (NextPtr == (ELEMENT_P)0)
    {
        return;                                  /* not found                  */
    }
    NextElementPtr->Next = NextPtr->Next;
    if (NextElementPtr->Next == (ELEMENT_P)0)
        QueuePtr->Last = NextElementPtr;
    QueuePtr->ElementCount--;                  /* decrement element count    */
    return;
}


