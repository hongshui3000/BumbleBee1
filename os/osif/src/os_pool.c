/**
 * Copyright (c) 2015, Realsil Semiconductor Corporation. All rights reserved.
 *
 */

#include <rtl_types.h>
#include <os.h>
#include <os_mem.h>
#include <os_pool.h>
#include <os_intr.h>
#include <trace_binary.h>

#define TRACE_MODULE_ID     MID_BT_HCI

TOsPool OsPoolTable[OS_POOL_TABLE_SIZE];

int osPoolHandleCheck(uint8_t PoolHandle)
{
    if (PoolHandle >= OS_FIRST_POOL_ID + OS_POOL_TABLE_SIZE ||
        PoolHandle < OS_FIRST_POOL_ID)
    {
        BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_ERROR,
               "!!!osPoolHandleCheck: PoolHandle= 0x%x", PoolHandle);
        return ER_PID;
    }

    return RET_OK;
}

int osPoolSlotCreate(uint8_t PoolId, RAM_TYPE RamType, uint8_t SetCallback, uint16_t BufferSize,
    uint16_t BufferCount, uint8_t BufferAlignment)
{
    POsPool     pPool;
    PPoolSlot   pSlot;
    PPoolSlot   pPrevSlot, pCurrSlot;
    PPoolBuffer pPoolBuffer;
    void        *pBuffer, *pData;
    size_t      HeaderLength, DataLength;
    uint16_t    i;

    pSlot = (PPoolSlot)osMemoryAlignAllocate(RAM_TYPE_DATA_ON, sizeof(TPoolSlot), 0);
    if (pSlot == NULL)
    {
        BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_ERROR, "!!!osPoolSlotCreate: pSlot is NULL");
        return ER_MEM;
    }

    /* TPoolBuffer uses default alignment */
    HeaderLength = SetCallback ? sizeof(TPoolBuffer) : sizeof(TMiniPoolBuffer);
    pBuffer = (void *)osMemoryAlignAllocate(RAM_TYPE_DATA_ON, HeaderLength * BufferCount, 0);
    if (pBuffer == NULL)
    {
        BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_ERROR, "!!!osPoolSlotCreate: pBuffer is NULL");
        osMemoryFree(pSlot);
        return ER_MEM;
    }

    /* Data Buffer uses caller required alignment */
    DataLength = (2 * sizeof(void *) + BufferSize + BufferAlignment -1) & ~(BufferAlignment - 1);
    pData = (void *)osMemoryAlignAllocate(RamType, DataLength * BufferCount, BufferAlignment);
    if (pData == NULL)
    {
        BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_ERROR, "!!!osPoolSlotCreate: pData is NULL");
        osMemoryFree(pBuffer);
        osMemoryFree(pSlot);
        return ER_MEM;
    }

    for (i = 0; i < BufferCount; i++)
    {
        /* sizeof(TPoolBuffer) will be default alignment */
        pPoolBuffer          = (PPoolBuffer)((uint8_t *)pBuffer + HeaderLength * i);
        pPoolBuffer->Data    = (uint8_t *)pData + DataLength * i;
        pPoolBuffer->pSlot   = pSlot;
        pPoolBuffer->Flags   = OS_BUFFER_FREE;
        pPoolBuffer->Next    = NULL;
        memcpy(pPoolBuffer->Data, &pPoolBuffer, sizeof(void *));

        osQueueIn(&pSlot->BufferQueue, pPoolBuffer);
    }

    pPool = &OsPoolTable[PoolId];
    pPrevSlot = (PPoolSlot)0;
    pCurrSlot = (PPoolSlot)pPool->SlotQueue.First;
    /* sort slots in ascending buffer size order */
    while (pCurrSlot != (PPoolSlot)0)
    {
        if (pCurrSlot->BufferSize >= BufferSize)
        {
            break;
        }
        pPrevSlot = pCurrSlot;
        pCurrSlot = (PPoolSlot)pCurrSlot->Next;
    }

    pSlot->BufferSize   = BufferSize;
    pSlot->PoolId       = PoolId;

    osQueueInsert(&pPool->SlotQueue, pPrevSlot, pSlot);

    return RET_OK;
}

int osPoolCreate(uint8_t *pPoolHandle, RAM_TYPE RamType, uint8_t SetCallback, uint16_t BufferSize,
    uint16_t BufferCount, uint8_t BufferAlignment)
{
    POsPool pPool;
    uint8_t PoolId;

    if (BufferSize == 0 || BufferCount == 0)
    {
        BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_ERROR, "!!!osPoolCreate: error param");
        return ER_PARAM;
    }

    for (PoolId = 0; PoolId < OS_POOL_TABLE_SIZE; PoolId++)
    {
        if (!(OsPoolTable[PoolId].Flags & OS_POOL_CREATED))
        {
            pPool = &OsPoolTable[PoolId];
            pPool->Flags = 0;
            break;
        }
    }

    if (PoolId == OS_POOL_TABLE_SIZE)
    {
        BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_ERROR, "!!!osPoolCreate: No PoolId available"); 
        return ER_PID;
    }

    if (osPoolSlotCreate(PoolId, RamType, SetCallback, BufferSize, BufferCount, BufferAlignment))
    {
        BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_ERROR, "!!!osPoolCreate: osPoolSlotCreate failed");
        *pPoolHandle = 0;
        return ER_MEM;
    }

    pPool->Flags |= OS_POOL_CREATED;
    if (SetCallback != false)
    {
        pPool->Flags |= OS_POOL_CALLBACK;
    }
    OS_POOL_MEM_TYPE_SET(pPool->Flags, RamType);
    OS_POOL_MEM_ALIGN_SET(pPool->Flags, BufferAlignment);

    *pPoolHandle = PoolId + OS_FIRST_POOL_ID;

    return RET_OK;
}

int osPoolExtend(uint8_t PoolHandle, uint16_t BufferSize, uint16_t BufferCount)
{
    POsPool pPool;
    uint8_t PoolId;
    RAM_TYPE RamType;
    uint8_t BufferAlignment;
    uint8_t SetCallback;

    if (BufferSize == 0 || BufferCount == 0)
    {
        BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_ERROR, "!!!osPoolExtend: invalid param");
        return ER_PARAM;
    }

    if (osPoolHandleCheck(PoolHandle))
    {
        BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_ERROR, "!!!osPoolExtend: PoolHandle error");
        return ER_PID;
    }

    PoolId = PoolHandle - OS_FIRST_POOL_ID;

    pPool = &OsPoolTable[PoolId];
    if (!(pPool->Flags & OS_POOL_CREATED))
    {
        BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_ERROR, "!!!osPoolExtend: pPool Flags error");
        return ER_PID;
    }

    OS_POOL_MEM_TYPE_GET(pPool->Flags, RamType);
    OS_POOL_MEM_ALIGN_GET(pPool->Flags, BufferAlignment);
    SetCallback = pPool->Flags & OS_POOL_CALLBACK ? TRUE: FALSE;

    if (osPoolSlotCreate(PoolId, RamType, SetCallback, BufferSize, BufferCount, BufferAlignment))
    {
        BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_ERROR, "!!!osPoolExtend: osPoolSlotCreate failed");
        return (ER_MEM);
    }

    return (RET_OK);
}

uint16_t osPoolFree(uint8_t PoolHandle, uint16_t BufferSize)
{
    POsPool     pPool;
    PPoolSlot   pSlot;
    uint16_t    Count = 0;
    int        s;

    if (osPoolHandleCheck(PoolHandle))
    {
        BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_ERROR, "!!!osPoolFree: PoolHandle error");
        return Count;
    }

    pPool = &OsPoolTable[PoolHandle - OS_FIRST_POOL_ID];
    if (!(pPool->Flags & OS_POOL_CREATED))
    {
        BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_ERROR, "!!!osPoolFree: pPool Flags error");
        return Count;
    }
    pSlot = (PPoolSlot)pPool->SlotQueue.First;
    s = osInterruptDisable();
    while (pSlot != NULL)
    {
        if (pSlot->BufferSize >= BufferSize)
        {
            Count += pSlot->BufferQueue.Count;
        }
        pSlot = (PPoolSlot)pSlot->Next;
    }
    osInterruptEnable(s);

    return Count;
}

int osBufferCallBackSet(void *pBuffer, TBufferCallBack Callback, uint32_t Handle)
{
    PPoolBuffer pPoolBuffer;
    POsPool     pPool;

    memcpy(&pPoolBuffer, (uint8_t *)pBuffer - 2*sizeof(void *), sizeof(void *));

    pPool = &OsPoolTable[pPoolBuffer->pSlot->PoolId];
    if (!(pPool->Flags & OS_POOL_CALLBACK))
    {
        BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_ERROR, "!!!osBufferCallBackSet: No callback capability");
        return ER_PARAM;
    }

    pPoolBuffer->pCallBack  = Callback;
    pPoolBuffer->Handle     = Handle;
    pPoolBuffer->Flags     &= ~OS_BUFFER_FREE;
    if (pPoolBuffer->pCallBack != (TBufferCallBack)0)
    {
        pPoolBuffer->Flags |= OS_BUFFER_CALLBACK;
    }
    else
    {
        pPoolBuffer->Flags &= ~OS_BUFFER_CALLBACK;
    }

    return RET_OK;
}

int osBufferGetInternal(uint8_t PoolHandle, uint16_t BufferSize, void **ppBuffer,
    TBufferCallBack Callback, uint32_t Handle)
{
    PPoolBuffer pPoolBuffer;
    PPoolSlot   pSlot;
    POsPool     pPool;
    int         s;
    uint16_t    BufferCount;

    *ppBuffer = NULL;

    if (BufferSize == 0)
    {
        BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_ERROR, "!!!osBufferGetInternal: BufferSize == 0");
        return ER_PARAM;
    }

    if (osPoolHandleCheck(PoolHandle))
    {
        BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_ERROR, "!!!osBufferGetInternal: PoolHandle error");
        return ER_PID;
    }

    pPool = &OsPoolTable[PoolHandle - OS_FIRST_POOL_ID];
    if (!(pPool->Flags & OS_POOL_CREATED))
    {
        BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_ERROR, "!!!osBufferGetInternal: Pool not created");
        return ER_PID;
    }

    if (!(pPool->Flags & OS_POOL_CALLBACK) && Callback)
    {
        BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_ERROR, "!!!osBufferGetInternal: No callback capability");
        return ER_PARAM;
    }

    pSlot = (PPoolSlot)pPool->SlotQueue.First;
    s = osInterruptDisable();

    while (pSlot != (PPoolSlot)0)
    {
        BufferCount = pSlot->BufferQueue.Count;
        if (pSlot->BufferSize >= BufferSize && BufferCount)
        {
            break;
        }
        pSlot = (PPoolSlot)pSlot->Next;
    }

    if (pSlot == (PPoolSlot)0)
    {
        osInterruptEnable(s);
        return ER_MEM;
    }

    pPoolBuffer = (PPoolBuffer)osQueueOut(&pSlot->BufferQueue);

    osInterruptEnable(s);

    pPoolBuffer->Flags &= ~OS_BUFFER_FREE;
    if (pPoolBuffer->pCallBack != (TBufferCallBack)0)
    {
        pPoolBuffer->Flags      |= OS_BUFFER_CALLBACK;
        pPoolBuffer->pCallBack   = Callback;
        pPoolBuffer->Handle      = Handle;
    }
    else
    {
        pPoolBuffer->Flags &= ~OS_BUFFER_CALLBACK;
    }

    memcpy(pPoolBuffer->Data, &pPoolBuffer, sizeof(void *));

    *ppBuffer = (uint8_t *)pPoolBuffer->Data + 2*sizeof(void *);

    return RET_OK;
}

int osBufferGet(uint8_t PoolHandle, uint16_t BufferSize, void **ppBuffer)
{
    return osBufferGetInternal(PoolHandle, BufferSize, ppBuffer, (TBufferCallBack)0, 0);
}

int osBufferGetCallBack(uint8_t PoolHandle, uint16_t BufferSize, void **ppBuffer,
    TBufferCallBack Callback, uint32_t Handle)
{
    return osBufferGetInternal(PoolHandle, BufferSize, ppBuffer, Callback, Handle);
}

#if F_BT_L2C_ENHANCED_FEATURE_SUPPORT
int osBufferLength(void *pBuffer, uint16_t *pBufferSize)
{
    PPoolSlot   pSlot;
    PPoolBuffer pPoolBuffer;

    if (pBuffer == NULL)
    {
        BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_ERROR, "!!!osBufferLength: pBuffer is NULL");
        return ER_MEM;
    }

    memcpy(&pPoolBuffer, (uint8_t *)pBuffer - 2*sizeof(void *), sizeof(void *));

    if (pPoolBuffer->Flags & OS_BUFFER_FREE)
    {
        BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_ERROR, "!!!osBufferLength: pBuffe(%p) is freed", pBuffer);
        return ER_MEM;
    }

    pSlot = pPoolBuffer->pSlot;
    *pBufferSize = pSlot->BufferSize;

    return RET_OK;
}
#endif

int osBufferRelease(void *pBuffer)
{
    PPoolBuffer pPoolBuffer;
    PPoolSlot   pSlot;
    int         s;
    uint16_t    Flags;
    uint32_t    Handle;
    TBufferCallBack pCallBack;

    if (pBuffer == NULL)
    {
        BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_ERROR, "!!!osBufferRelease: pBuffer is NULL");
        return ER_MEM;
    }

    memcpy(&pPoolBuffer, (uint8_t *)pBuffer - 2*sizeof(void *), sizeof(void *));

    if (pPoolBuffer->Flags & OS_BUFFER_FREE)
    {
        BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_ERROR, "!!!osBufferRelease: pBuffer(%p) is freed", pBuffer);
        return ER_MEM;
    }
    pSlot = pPoolBuffer->pSlot;

    /* save buffer callback specific data for potential call */
    Flags  = pPoolBuffer->Flags;
    if (Flags & OS_BUFFER_CALLBACK)
    {
        Handle = pPoolBuffer->Handle;
        pCallBack = pPoolBuffer->pCallBack;
        pPoolBuffer->pCallBack = (TBufferCallBack)0;
    }

    pPoolBuffer->Flags |= OS_BUFFER_FREE;
    pPoolBuffer->Flags &= ~OS_BUFFER_CALLBACK;

    s = osInterruptDisable();
    osQueueIn(&pSlot->BufferQueue, pPoolBuffer);
    osInterruptEnable(s);

    if (Flags & OS_BUFFER_CALLBACK)
    {
        pCallBack(Handle);
    }

    return RET_OK;
}

void osPoolDump(void)
{
    uint8_t PoolId;
    PPoolSlot pSlot;

    for (PoolId = 0; PoolId < OS_POOL_TABLE_SIZE; PoolId++)
    {
        BLUEAPI_TRACE_PRINTF_3(BLUEAPI_TRACE_MASK_ERROR,
            "OsPool[%d]: Flags(0x%x), Slot Count(%d)",
            PoolId,
            OsPoolTable[PoolId].Flags,
            OsPoolTable[PoolId].SlotQueue.Count
            );

        pSlot = (PPoolSlot)OsPoolTable[PoolId].SlotQueue.First;
        while (pSlot != (PPoolSlot)0)
        {
            BLUEFACE_TRACE_PRINTF_4(BLUEAPI_TRACE_MASK_ERROR,
                "pSlot(%p), PoolId(0x%x), Buffer Size(%d), Buffer Count(%d)",
                pSlot,
                pSlot->PoolId,
                pSlot->BufferSize,
                pSlot->BufferQueue.Count
                );

            pSlot = (PPoolSlot)pSlot->Next;
        }
    }
}
