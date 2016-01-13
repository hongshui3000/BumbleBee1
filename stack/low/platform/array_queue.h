/***************************************************************************
 Copyright (C) RealTek Ltd.
 This module is a confidential and proprietary property of RealTek and
 a possession or use of this module requires written permission of RealTek.
 ***************************************************************************/

#ifndef ARRAY_QUEUE_H_
#define ARRAY_QUEUE_H_

#include "DataType.h"

#define ARRAY_QUEUE_DEFINE_TYPE(type, name) \
typedef struct \
{ \
    type *item; \
    UINT32 size; \
    UINT32 front; \
    UINT32 back; \
    UINT32 len; \
} name

#define ARRAY_QUEUE_INIT(q, buf, siz) \
do { \
    (q)->item = (buf); \
    (q)->size = (siz); \
    (q)->front = 0; \
    (q)->back = 0; \
    (q)->len = 0; \
} while (0)

#define ARRAY_QUEUE_IS_EMPTY(q) ((q)->len == 0)

#define ARRAY_QUEUE_IS_FULL(q) ((q)->len == (q)->size)

#define ARRAY_QUEUE_FRONT(q) ((q)->item[(q)->front])

#define ARRAY_QUEUE_ENQUEUE(q, data) \
({ \
    BOOLEAN succeeded; \
    if (ARRAY_QUEUE_IS_FULL(q)) \
    { \
        succeeded = FALSE; \
    } \
    else \
    { \
        (q)->item[(q)->back] = (data); \
        (q)->back = ((q)->back + 1) % (q)->size; \
        ++(q)->len; \
        succeeded = TRUE; \
    } \
    succeeded; \
})

#define ARRAY_QUEUE_DEQUEUE(q, data) \
({ \
    BOOLEAN succeeded; \
    if (ARRAY_QUEUE_IS_EMPTY(q)) \
    { \
        succeeded = FALSE; \
    } \
    else \
    { \
        *(data) = (q)->item[(q)->front]; \
        (q)->front = ((q)->front + 1) % (q)->size; \
        --(q)->len; \
        succeeded = TRUE; \
    } \
    succeeded; \
})

#endif /* ARRAY_QUEUE_H_ */
