/**
 * Copyright (c) 2015, Realsil Semiconductor Corporation. All rights reserved.
 *
 */

#ifndef __OS_POOL_H
#define __OS_POOL_H

#include <stdint.h>
#include <rtl_types.h>
#include <rtl_types.h>
#include <os_queue.h>



#ifdef __cplusplus
extern "C" {
#endif

#define OS_FIRST_POOL_ID        0x10
#define OS_POOL_ILLEGAL_HANDLE  0xFF

/* Maximum number of pools created by osPoolCreate */
#define OS_POOL_TABLE_SIZE      24

typedef void (*TBufferCallBack)(uint32_t Handle);

typedef struct tagPoolSlot
{
    struct tagPoolSlot  *Next;
    uint8_t             PoolId;
    uint16_t            BufferSize;
    QUEUE_T             BufferQueue;
} TPoolSlot, *PPoolSlot;

#define OS_BUFFER_FREE           0x01
#define OS_BUFFER_CALLBACK       0x02

typedef struct tagPoolBuffer
{
    struct tagPoolBuffer    *Next;
    PPoolSlot               pSlot;
    uint16_t                Flags;
    void                    *Data;
    TBufferCallBack         pCallBack;
    uint32_t                Handle;
} TPoolBuffer, *PPoolBuffer;

typedef struct tagMiniPoolBuffer
{
    struct tagPoolBuffer    *Next;
    PPoolSlot               pSlot;
    uint16_t                Flags;
    void                    *Data;
} TMiniPoolBuffer, *PMiniPoolBuffer;

/**
 * OS Pool 32 bit flags are divided into 4 sections.
 * bit  0 ~  7: set pool status.
 * bit  8 ~ 15: set pool memory type.
 * bit 16 ~ 23: reserved.
 * bit 24 ~ 31: set pool buffer alignment.
 */
#define OS_POOL_CREATED         0x01
#define OS_POOL_CALLBACK        0x02

#define OS_POOL_MEM_TYPE_SET(flag, type)    ( (flag) |= (((uint8_t)(type)) << 8)         )
#define OS_POOL_MEM_TYPE_GET(flag, type)    ( (type) = (RAM_TYPE)(((flag) >> 8) & 0xFF) )

#define OS_POOL_MEM_ALIGN_SET(flag, align)  ( (flag) |= (((uint8_t)(align)) << 24)       )
#define OS_POOL_MEM_ALIGN_GET(flag, align)  ( (align) = (uint8_t)(((flag) >> 24) & 0xFF) )
typedef struct tagOsPool
{
    uint32_t    Flags;
    QUEUE_T     SlotQueue;
} TOsPool, *POsPool;

int osPoolCreate(uint8_t *pPoolHandle, RAM_TYPE RamType, uint8_t SetCallback, uint16_t BufferSize,
    uint16_t BufferCount, uint8_t BufferAlignment);

int osPoolExtend(uint8_t PoolHandle, uint16_t BufferSize, uint16_t BufferCount);

uint16_t osPoolFree(uint8_t PoolHandle, uint16_t BufferSize);

int osBufferGet(uint8_t PoolHandle, uint16_t BufferSize, void **ppBuffer);

int osBufferGetCallBack(uint8_t PoolHandle, uint16_t BufferSize, void **ppBuffer,
    TBufferCallBack Callback, uint32_t Handle);

int osBufferRelease(void *pBuffer);

int osBufferCallBackSet(void *pBuffer, TBufferCallBack Callback, uint32_t Handle);

int osBufferLength(void *pBuffer, uint16_t *pBufferSize);

void osPoolDump(void);

#ifdef __cplusplus
}
#endif

#endif  /* __OS_POOL_H */
