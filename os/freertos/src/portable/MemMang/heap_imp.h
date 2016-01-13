/***************************************************************************
 Copyright (C) RealTek Ltd.
 This module is a confidential and proprietary property of RealTek and
 a possession or use of this module requires written permission of RealTek.
 ***************************************************************************/

#ifndef HEAP_IMP_H_
#define HEAP_IMP_H_

#include "DataType.h"
#include "FreeRTOS.h"

/**
 * @name Memory interface
 * @{
 */
/**
 * @struct MEM_BLOCK_
 * @brief Memory block header.
 *
 * Each partial-on memory block has a header MEM_BLOCK, which links to the next
 * partial-on memory block in ascending address order.
 */
typedef struct MEM_BLOCK_
{
    /** Pointer to the next memory block; \c NULL, if the last one. */
    struct MEM_BLOCK_ *next;
    /** Size of memory block (including MEM_BLOCK). */
    size_t size;
} MEM_BLOCK;
/**@} */

typedef struct MEM_ON_MANAGER_
{
    MEM_BLOCK *free_head;
    size_t free_bytes;
} MEM_ON_MANAGER;


typedef struct MEM_BLOCK_OFF_DESC_
{
    void *addr;
    size_t size;
} MEM_BLOCK_OFF_DESC;

/**
 * @name Memory interface
 * @{
 */
/**
 * @struct MEM_BLOCK_OFF_MANAGER_
 * @brief Partial-off memory manager
 *
 * This manager maintains descriptors on partial-on memory to point to memory
 * buffers allocated on partial-off memory. The structure of descriptors is
 * like this:
@verbatim
+------+------+-------+-------+-------+------+------+
| free | free | empty | empty | empty | used | used |
+------+------+-------+-------+-------+------+------+
                  ^                      ^             ^
                  +- free_end            +- used_beg   +- desc_num
@endverbatim
 * Descriptors are managed in an array.
 */
typedef struct MEM_OFF_MANAGER_
{
    MEM_BLOCK_OFF_DESC *desc;
    size_t desc_num;

    int free_end;
    int used_beg;

    size_t free_bytes;
} MEM_OFF_MANAGER;
/**@} */

extern MEM_ON_MANAGER mem_mgr_data_on;
extern MEM_ON_MANAGER mem_mgr_buf_on;
extern MEM_OFF_MANAGER mem_mgr_buf_off;
extern MEM_OFF_MANAGER mem_mgr_data_off;

#define MEM_BLOCK_ALLOC_BIT     (((size_t) 1) << ((sizeof (size_t) * 8) - 1))

#define MEM_BLOCK_ALIGNMENT         portBYTE_ALIGNMENT
#define MEM_BLOCK_ALIGNMENT_MASK    portBYTE_ALIGNMENT_MASK
#define MEM_BLOCK_SIZE_ALIGN(s) (((s) + (MEM_BLOCK_ALIGNMENT - 1)) & ~(MEM_BLOCK_ALIGNMENT_MASK))

#define MEM_BLOCK_HEADER_SIZE   MEM_BLOCK_SIZE_ALIGN(sizeof (MEM_BLOCK))
#define MEM_BLOCK_MIN_SIZE      (2 * MEM_BLOCK_HEADER_SIZE)
#define MEM_BLOCK_OFF_MIN_SIZE  (2 * MEM_BLOCK_SIZE_ALIGN(sizeof (MEM_BLOCK_OFF_DESC)))

#endif /* HEAP_IMP_H_ */
