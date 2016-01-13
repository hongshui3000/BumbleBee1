/**
 * Copyright (c) 2015, Realsil Semiconductor Corporation. All rights reserved.
 *
 */

#ifndef __OS_MEM_H
#define __OS_MEM_H

#include <stdint.h>
#include <rtl_types.h>

#ifdef __cplusplus
extern "C" {
#endif

void *osMemoryAllocate(RAM_TYPE RamType, size_t Size);

void *osMemoryClearAllocate(RAM_TYPE RamType, size_t Size);

void *osMemoryAlignAllocate(RAM_TYPE RamType, size_t Size, uint8_t Alignment);

void osMemoryFree(void *pBlock);


#ifdef __cplusplus
 }
#endif

#endif /* __OS_MEM_H */
