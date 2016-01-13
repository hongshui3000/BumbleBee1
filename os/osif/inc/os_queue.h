/**
 * Copyright (C) 2015 Realsil Semiconductor Corporation. All rights reserved.
 *
 */

#ifndef __OS_QUEUE_H
#define __OS_QUEUE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct tagRawQueueElement
{
    struct tagRawQueueElement *Next;
} TRawQueueElement;

typedef TRawQueueElement    ELEMENT_T;
typedef TRawQueueElement   *ELEMENT_P;

typedef struct tagRawQueue
{
    ELEMENT_P   First;
    ELEMENT_P   Last;
    uint32_t    Count;
} QUEUE_T, *QUEUE_P;

void  osQueueIn(QUEUE_P pQueue, void *pElement);
void *osQueueOut(QUEUE_P pQueue);
void  osQueueInsert(QUEUE_P pQueue, void *pElement, void *pNewElement);
void  osQueueDelete(QUEUE_P pQueue, void *pElement);

#ifdef __cplusplus
}
#endif

#endif  /* __OS_QUEUE_H */
