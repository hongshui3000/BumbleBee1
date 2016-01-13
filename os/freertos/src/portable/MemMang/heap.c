/***************************************************************************
 Copyright (C) RealTek Ltd.
 This module is a confidential and proprietary property of RealTek and
 a possession or use of this module requires written permission of RealTek.
 ***************************************************************************/
/********************************* Logger *************************/
enum { __FILE_NUM__= 147 };
/********************************* Logger *************************/

#include <bt_fw_config.h>
#include <le_ll.h>
#include "heap_imp.h"
#include "section_config.h"
#include <mint_os.h>
#ifdef UNIT_TEST
#include <unit_test.h> /* Work only if UNIT_TEST defined */
#endif
#include "FreeRTOS.h"
#include "task.h"


MEM_ON_MANAGER mem_mgr_buf_on;
MEM_ON_MANAGER mem_mgr_data_on;

#define OS_POOL_SIZE_EXPAND(s, n, a, c) OS_POOL_MEM_SIZE(s, n, a, c)
uint8_t heap_buf_on[MEM_BLOCK_SIZE_ALIGN(OS_MAX_HEAP_MEMORY)] ALIGN(MEM_BLOCK_ALIGNMENT) SRAM_ON_BF_BSS_SECTION;
#undef OS_POOL_SIZE_EXPAND
#define OS_POOL_SIZE_EXPAND(s, n, a, c) OS_POOL_STACK_MEM_SIZE(s, n, a, c)
uint8_t heap_data_on[MEM_BLOCK_SIZE_ALIGN(OS_MAX_HEAP_DMEM_MEMORY + OS_MAX_QUEUE_MEMORY + 1024)] ALIGN(MEM_BLOCK_ALIGNMENT) SRAM_ON_BD_HEAP_SECTION;
#undef OS_POOL_SIZE_EXPAND


MEM_OFF_MANAGER mem_mgr_buf_off;
MEM_OFF_MANAGER mem_mgr_data_off;

#define OS_POOL_SIZE_EXPAND(s, n, a, c) (n)
MEM_BLOCK_OFF_DESC heap_buf_off_desc[OS_MAX_HEAP_DESC_PARTIAL_OFF] ALIGN(MEM_BLOCK_ALIGNMENT) SRAM_ON_BF_BSS_SECTION;
#undef OS_POOL_SIZE_EXPAND
#define OS_POOL_SIZE_EXPAND(s, n, a, c) OS_POOL_MEM_SIZE(s, n, a, c)
uint8_t heap_buf_off[MEM_BLOCK_SIZE_ALIGN(OS_MAX_HEAP_MEMORY_PARTIAL_OFF)] ALIGN(MEM_BLOCK_ALIGNMENT) SRAM_OFF_BF_BSS_SECTION;
#undef OS_POOL_SIZE_EXPAND

MEM_BLOCK_OFF_DESC heap_data_off_desc[OS_MAX_HEAP_DESC_DMEM_OFF] ALIGN(MEM_BLOCK_ALIGNMENT) SRAM_ON_BD_HEAP_SECTION;
uint8_t heap_data_off[MEM_BLOCK_SIZE_ALIGN(OS_MAX_HEAP_DMEM_MEMORY_OFF + 1024)] ALIGN(MEM_BLOCK_ALIGNMENT) SRAM_OFF_BD_HEAP_SECTION;


typedef struct MEM_REGION_TYPE_MAP_
{
    void *start_addr;
    RAM_TYPE type;
} MEM_REGION_TYPE_MAP;
MEM_REGION_TYPE_MAP mem_type_map[RAM_TYPE_NUM + 2]; /* Additional space for extending heap */
uint8_t mem_type_map_len;

void mem_region_type_register(void *start_addr, RAM_TYPE mem_type)
{
    if (mem_type_map_len >= ARRAY_SIZE(mem_type_map))
    {
        return;
    }

    uint8_t insert_i;
    for (insert_i = 0; insert_i < mem_type_map_len; ++insert_i)
    {
        if (start_addr > mem_type_map[insert_i].start_addr)
        {
            break;
        }
    }
    uint8_t j;
    for (j = mem_type_map_len; j > insert_i; --j)
    {
        mem_type_map[j] = mem_type_map[j - 1];
    }
    mem_type_map[insert_i].start_addr = start_addr;
    mem_type_map[insert_i].type = mem_type;
    ++mem_type_map_len;
}

RAM_TYPE mem_region_type_search(void *start_addr)
{
    uint8_t i;
    for (i = 0; i < mem_type_map_len; ++i)
    {
        if (start_addr >= mem_type_map[i].start_addr)
        {
            return mem_type_map[i].type;
        }
    }
    return RAM_TYPE_NUM;
}

void mem_on_manager_init(MEM_ON_MANAGER *mgr, RAM_TYPE mem_type, uint8_t *heap,
        size_t heap_size)
{
    size_t aligned_heap = MEM_BLOCK_SIZE_ALIGN((size_t) heap);
    size_t aligned_heap_size = heap_size - (aligned_heap - (size_t) heap);
    mgr->free_head = (MEM_BLOCK *) aligned_heap;
    mgr->free_head->next = NULL;
    mgr->free_head->size = aligned_heap_size;
    mgr->free_bytes = aligned_heap_size;
    mem_region_type_register(heap, mem_type);
}

void mem_on_manager_extend(MEM_ON_MANAGER *mgr, RAM_TYPE mem_type,
        uint8_t *heap, size_t heap_size)
{
    MEM_BLOCK **prev = &mgr->free_head;
    MEM_BLOCK *cur = mgr->free_head;
    while (cur != NULL && (uint8_t *) cur < heap)
    {
        prev = &cur->next;
        cur = cur->next;
    }

    size_t aligned_heap = MEM_BLOCK_SIZE_ALIGN((size_t) heap);
    size_t aligned_heap_size = heap_size - (aligned_heap - (size_t) heap);
    MEM_BLOCK *blk = (MEM_BLOCK *) aligned_heap;
    MEM_BLOCK *next = *prev;
    *prev = blk;
    blk->next = next;
    blk->size = aligned_heap_size;
    mgr->free_bytes += aligned_heap_size;
    mem_region_type_register(heap, mem_type);
}

void mem_off_manager_init(MEM_OFF_MANAGER *mgr, RAM_TYPE mem_type,
        uint8_t *heap, size_t heap_size, MEM_BLOCK_OFF_DESC *desc,
        size_t desc_num)
{
    size_t aligned_heap = MEM_BLOCK_SIZE_ALIGN((size_t) heap);
    size_t aligned_heap_size = heap_size - (aligned_heap - (size_t) heap);
    mgr->desc = desc;
    mgr->desc_num = desc_num;
    mgr->free_end = 1;
    mgr->used_beg = desc_num;
    mgr->free_bytes = aligned_heap_size;

    mgr->desc[0].addr = (void *) aligned_heap;
    mgr->desc[0].size = aligned_heap_size;
    mem_region_type_register(heap, mem_type);
}

/**
 * @brief Binary search for the memory block with the given address.
 *
 * This function returns the index of the memory block with the given address,
 * \p addr, if it is found; otherwise, it returns the index where you can insert
 * a new memory block. The policy of inserting the new memory block is
 * suggested by \p move_right. If \p move_right is \c TRUE, the new memory
 * block is inserted by moving blocks starting from \p index rightward. If \p
 * move_right is \c FALSE, the new memory block is inserted by moving blocks
 * before \p index (inclusive) leftward. \p move_right is just a indication to
 * decide what value \p index should be. This functions doesn't perform the
 * actual insertion.
 *
 * @param[in] mgr  Off-Memory manager
 * @param[in] addr  Address
 * @param[in] beg  Beginning index to search (inclusive)
 * @param[in] end  Ending index to search (exclusive)
 * @param[out] index  Return index if found, or insert position
 * @param[in] move_right  Policy of inserting the new memory block
 * @return \c TRUCE if found; otherwise, \c FALSE.
 */
BOOLEAN mem_off_manager_bsearch(MEM_OFF_MANAGER *mgr, void *addr,
        int beg, int end, int *index, BOOLEAN move_right)
{
    if (beg >= end)
    {
        *index = end + move_right - 1;
        return FALSE;
    }

    ptrdiff_t cmp;
    int low = beg;
    int high = end - 1;
    while (low < high)
    {
        int med = (low + high) / 2;
        cmp = (uint8_t *) mgr->desc[med].addr - (uint8_t *) addr;
        if (cmp < 0)
        {
            low = med + 1;
        }
        else if (cmp > 0)
        {
            high = med - 1;
        }
        else
        {
            *index = med;
            return TRUE;
        }
    }

    cmp = (uint8_t *) mgr->desc[low].addr - (uint8_t *) addr;
    if (cmp == 0)
    {
        *index = low;
        return TRUE;
    }
    else if (cmp < 0)
    {
        *index = low + move_right;
    }
    else
    {
        *index = low + move_right - 1;
    }
    return FALSE;
}

void mem_off_manager_extend(MEM_OFF_MANAGER *mgr, RAM_TYPE mem_type,
        uint8_t *heap, size_t heap_size)
{
    int index;
    if (mgr->free_end >= mgr->desc_num
            || mem_off_manager_bsearch(mgr, heap, 0, mgr->free_end, &index, TRUE))
    {
        return;
    }

    int i;
    for (i = mgr->free_end; i > index; --i)
    {
        mgr->desc[i] = mgr->desc[i - 1];
    }
    size_t aligned_heap = MEM_BLOCK_SIZE_ALIGN((size_t) heap);
    size_t aligned_heap_size = heap_size - (aligned_heap - (size_t) heap);
    mgr->desc[index].addr = (void *) aligned_heap;
    mgr->desc[index].size = aligned_heap_size;
    mgr->free_end += 1;
    mgr->free_bytes += aligned_heap_size;
    mem_region_type_register(heap, mem_type);
}

void os_heap_init()
{
    mem_type_map_len = 0;
    mem_on_manager_init(&mem_mgr_buf_on, RAM_TYPE_BUFFER_ON, heap_buf_on,
            sizeof (heap_buf_on));
    mem_on_manager_init(&mem_mgr_data_on, RAM_TYPE_DATA_ON, heap_data_on,
            sizeof (heap_data_on));
    mem_off_manager_init(&mem_mgr_buf_off, RAM_TYPE_BUFFER_OFF, heap_buf_off,
            sizeof (heap_buf_off), heap_buf_off_desc,
            ARRAY_SIZE(heap_buf_off_desc));
    mem_off_manager_init(&mem_mgr_data_off, RAM_TYPE_DATA_OFF, heap_data_off,
            sizeof (heap_data_off), heap_data_off_desc,
            ARRAY_SIZE(heap_data_off_desc));
}

void *os_malloc_on(size_t size, MEM_ON_MANAGER *mgr)
{
    uint8_t *allocated = NULL;
    if ((size & MEM_BLOCK_ALLOC_BIT) || size <= 0)
    {
        return NULL;
    }
    vTaskSuspendAll();
    {
        size += MEM_BLOCK_HEADER_SIZE;
        size = MEM_BLOCK_SIZE_ALIGN(size);
        if (size > mgr->free_bytes)
        {
            goto alloc_exit;
        }

        MEM_BLOCK **prev = &mgr->free_head;
        MEM_BLOCK *blk = mgr->free_head;
        while (1)
        {
            if (blk == NULL)
            {
                goto alloc_exit;
            }
            if (blk->size >= size)
            {
                break;
            }
            prev = &blk->next;
            blk = blk->next;
        }

        allocated = (uint8_t *) blk + MEM_BLOCK_HEADER_SIZE;
        /* Split the block if it's too large. */
        if (blk->size - size >= MEM_BLOCK_MIN_SIZE)
        {
            MEM_BLOCK *new_blk = (MEM_BLOCK *) ((UCHAR *) blk + size);
            new_blk->next = blk->next;
            new_blk->size = blk->size - size;
            blk->size = size;

            *prev = new_blk;
        }
        else
        {
            *prev = blk->next;
        }
        mgr->free_bytes -= blk->size;

        blk->size |= MEM_BLOCK_ALLOC_BIT; /* Turn on allocated bit. */
        blk->next = NULL;
    }
alloc_exit:
    xTaskResumeAll();

#if (configUSE_MALLOC_FAILED_HOOK == 1)
    if (allocated == NULL)
    {
        extern void vApplicationMallocFailedHook(void);
        vApplicationMallocFailedHook();
    }
#endif
    return allocated;
}

void os_free_on(void *buf, MEM_ON_MANAGER *mgr)
{
    if (buf == NULL)
    {
        return;
    }

    MEM_BLOCK *blk = (MEM_BLOCK *) ((UCHAR *) buf - MEM_BLOCK_HEADER_SIZE);
    configASSERT(blk->size & MEM_BLOCK_ALLOC_BIT);
    configASSERT(blk->next == NULL);
    if (!(blk->size & MEM_BLOCK_ALLOC_BIT) || blk->next != NULL)
    {
        // TODO: memory free fail
        return;
    }

    blk->size &= ~MEM_BLOCK_ALLOC_BIT;
    vTaskSuspendAll();
    {
        mgr->free_bytes += blk->size;

        MEM_BLOCK *prev = NULL;
        MEM_BLOCK *cur = mgr->free_head;
        while (cur != NULL && cur < blk)
        {
            prev = cur;
            cur = cur->next;
        }

        /* Merge next block if contiguous and cur != NULL. */
        if ((uint8_t *) blk + blk->size == (uint8_t *) cur)
        {
            blk->size += cur->size;
            blk->next = cur->next;
        }
        else
        {
            blk->next = cur;
        }

        if (prev != NULL)
        {
            /* Merge previous block if contiguous. */
            if ((uint8_t *) prev + prev->size == (uint8_t *) blk)
            {
                prev->size += blk->size;
                prev->next = blk->next;
            }
            else
            {
                prev->next = blk;
            }
        }
        else
        {
            mgr->free_head = blk;
        }
    }
    xTaskResumeAll();
}

void *os_malloc_off(size_t size, MEM_OFF_MANAGER *mgr)
{
    void *allocated_addr = NULL;

    if ((size & MEM_BLOCK_ALLOC_BIT) || size <= 0)
    {
        return NULL;
    }
    vTaskSuspendAll();
    {
        if (mgr->free_end <= 0)
        {
            goto alloc_exit;
        }

        size = MEM_BLOCK_SIZE_ALIGN(size);
        if (size > mgr->free_bytes)
        {
            goto alloc_exit;
        }

        int desc_i;
        MEM_BLOCK_OFF_DESC *const desc = mgr->desc;
        for (desc_i = 0; ; ++desc_i)
        {
            if (desc_i >= mgr->free_end)
            {
                goto alloc_exit;
            }
            if (desc[desc_i].size >= size)
            {
                break;
            }
        }

        MEM_BLOCK_OFF_DESC allocated = desc[desc_i];

        /* Check if there's empty descriptor available for "used" group */
        int index;
        BOOLEAN found = mem_off_manager_bsearch(mgr, allocated.addr,
                mgr->used_beg, mgr->desc_num, &index, FALSE);
        configASSERT(!found);
        if (found)
        {
            // TODO: shouldn't enter here
            goto alloc_exit;
        }

        /* Split the block if it's too large. */
        if (allocated.size - size >= MEM_BLOCK_OFF_MIN_SIZE)
        {
            if (mgr->used_beg <= mgr->free_end) /* No more empty desc */
            {
                // TODO: raise warning
                goto alloc_exit;
            }

            desc[desc_i].addr = (uint8_t *) allocated.addr + size;
            desc[desc_i].size = allocated.size - size;
            allocated.size = size;
        }
        else
        {
            int i;
            for (i = desc_i; i < mgr->free_end - 1; ++i)
            {
                desc[i] = desc[i + 1];
            }
            --mgr->free_end;
        }
        mgr->free_bytes -= allocated.size;

        /* Insert the allocated descriptor into "used" group */
        int i;
        for (i = mgr->used_beg - 1; i < index; ++i)
        {
            desc[i] = desc[i + 1];
        }
        allocated_addr = allocated.addr;
        desc[index].addr = allocated_addr;
        desc[index].size = (allocated.size | MEM_BLOCK_ALLOC_BIT); /* Turn on allocated bit. */
        --mgr->used_beg;
    }
alloc_exit:
    xTaskResumeAll();

    return allocated_addr;
}

void os_free_off(void *buf, MEM_OFF_MANAGER *mgr)
{
    if (buf == NULL)
    {
        return;
    }

    vTaskSuspendAll();
    do
    {
        int index;
        BOOLEAN found = mem_off_manager_bsearch(mgr, buf, mgr->used_beg,
                mgr->desc_num, &index, FALSE);
        configASSERT(found);
        if (!found)
        {
            // TODO: shouldn't go here
            break;
        }

        MEM_BLOCK_OFF_DESC *const desc = mgr->desc;
        MEM_BLOCK_OFF_DESC allocated = desc[index];
        configASSERT(allocated.size & MEM_BLOCK_ALLOC_BIT);
        if (!(allocated.size & MEM_BLOCK_ALLOC_BIT))
        {
            // TODO: memory free fail
            break;
        }

        allocated.size &= ~MEM_BLOCK_ALLOC_BIT;
        mgr->free_bytes += allocated.size;

        /* Remove the descriptor from "used" group */
        {
            int i;
            for (i = index; i > mgr->used_beg; --i)
            {
                desc[i] = desc[i - 1];
            }
            ++mgr->used_beg;
        }

        /* Insert the descriptor into "free" group */
        found = mem_off_manager_bsearch(mgr, buf, 0, mgr->free_end, &index, TRUE);
        configASSERT(!found);
        if (found)
        {
            // TODO: free fail
            break;
        }

        int offset;
        /* Merge next block if contiguous */
        if (index < mgr->free_end
                && (uint8_t *) allocated.addr + allocated.size == (uint8_t *) desc[index].addr)
        {
            allocated.size += desc[index].size;
            offset = 0;
        }
        else
        {
            offset = 1;
        }

        if (index > 0)
        {
            /* Merge previous block if contiguous. */
            if ((uint8_t *) desc[index - 1].addr + desc[index - 1].size == (uint8_t *) allocated.addr)
            {
                allocated.addr = desc[index - 1].addr;
                allocated.size += desc[index - 1].size;
                index -= 1;
                offset -= 1;
            }
        }

        if (offset > 0) /* not merge next, not merge previous */
        {
            int i;
            for (i = mgr->free_end; i > index; --i)
            {
                desc[i] = desc[i - 1];
            }
            ++mgr->free_end;
        }
        else if (offset < 0) /* merge next, merge previous */
        {
            int i;
            for (i = index + 1; i < mgr->free_end - 1; ++i)
            {
                desc[i] = desc[i + 1];
            }
            --mgr->free_end;
        }
        desc[index] = allocated;
    } while (0);
    xTaskResumeAll();
}

typedef void *(*MEM_OP_MALLOC_FUNC)(size_t, void *);
typedef void (*MEM_OP_FREE_FUNC)(void *, void *);

typedef struct MEM_OP_
{
    void *mgr;
    MEM_OP_MALLOC_FUNC malloc;
    MEM_OP_FREE_FUNC free;
} MEM_OP;

#define MEM_OP_ON_INITIALIZER(mgr) { (mgr), (MEM_OP_MALLOC_FUNC) os_malloc_on, (MEM_OP_FREE_FUNC) os_free_on }
#define MEM_OP_OFF_INITIALIZER(mgr) { (mgr), (MEM_OP_MALLOC_FUNC) os_malloc_off, (MEM_OP_FREE_FUNC) os_free_off }

MEM_OP mem_op[RAM_TYPE_NUM] = {
        [RAM_TYPE_DATA_OFF] = MEM_OP_OFF_INITIALIZER(&mem_mgr_data_off),
        [RAM_TYPE_DATA_ON] = MEM_OP_ON_INITIALIZER(&mem_mgr_data_on),
        [RAM_TYPE_BUFFER_OFF] = MEM_OP_OFF_INITIALIZER(&mem_mgr_buf_off),
        [RAM_TYPE_BUFFER_ON] = MEM_OP_ON_INITIALIZER(&mem_mgr_buf_on),
};

void *pvPortMalloc(RAM_TYPE ram_type, size_t size)
{
    return mem_op[ram_type].malloc(size, mem_op[ram_type].mgr);
}

void vPortFree(void *buf)
{
    RAM_TYPE ram_type = mem_region_type_search(buf);
    mem_op[ram_type].free(buf, mem_op[ram_type].mgr);
}
