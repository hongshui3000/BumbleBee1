/**
 * Copyright (c) 2015, Realsil Semiconductor Corporation. All rights reserved.
 *
 */

#include <stdint.h>
#include <rtl_types.h>
#include <osif.h>

void *osMemoryAllocate(RAM_TYPE RamType, size_t Size)
{

    return osifMemoryAllocate(RamType, Size);
}

void *osMemoryClearAllocate(RAM_TYPE RamType, size_t Size)
{
    return osifMemoryClearAllocate(RamType, Size);
}

void *osMemoryAlignAllocate(RAM_TYPE RamType, size_t Size, uint8_t Alignment)
{
    return osifMemoryAlignAllocate(RamType, Size, Alignment);
}

void osMemoryFree(void *pBlock)
{
    osifMemoryFree(pBlock);
}
