//DataType.h

#ifndef _DATATYPE_H_
#define _DATATYPE_H_

#include <stddef.h>
#include "rtl_types.h"
#include "common_defs.h"


#ifdef IS_BTSOC
#define KSEG1                      0
#define NON_DCATCH(ptr)            ((UINT32)((UINT32)(ptr)))
#else
#define KSEG1                      0xA0000000
#define NON_DCATCH(ptr)            ((UINT32)((UINT32)(ptr) | KSEG1 | 0x100000))
#endif
#define NON_DCATCH_PTR(ptr)        ((UINT8*)NON_DCATCH(ptr))

#define RD_32BIT_IO(base, offset) \
        (*(volatile UINT32*)((base) + (offset)))
#define RD_16BIT_IO(base, offset) \
        (*((volatile UINT16*)((base) + (offset))))
#define RD_8BIT_IO(base, offset) \
        (*((volatile UINT8*)((base) + (offset))))
#define WR_32BIT_IO(base, offset, val) \
        (*((volatile UINT32*)((base) + (offset))) = (val))
#define WR_16BIT_IO(base, offset, val) \
        (*((volatile UINT16*)((base) + (offset))) = (val))
#define WR_8BIT_IO(base, offset, val) \
        (*((volatile UINT8*)((base) + (offset))) = (val))

#define RD_REG_MMIO(type, addr) \
    (*((volatile type*) (addr)))
#define WR_REG_MMIO(type, addr, val) \
    (*((volatile type*) (addr)) = (val))


typedef union UINT32_S_ {
    UINT32 u4Byte;
    UINT16 u2Byte[2];
    UINT8 u1Byte[4];
} UINT32_S;

typedef union INT32_S_ {
    INT32 s4Byte;
    INT16 s2Byte[2];
    INT8 s1Byte[4];
} INT32_S;

typedef struct UINT32_BM_S_
{
    UINT32 bit0:1;
    UINT32 bit1:1;
    UINT32 bit2:1;
    UINT32 bit3:1;
    UINT32 bit4:1;
    UINT32 bit5:1;
    UINT32 bit6:1;
    UINT32 bit7:1;
    UINT32 bit8:1;
    UINT32 bit9:1;    
    UINT32 bit10:1;
    UINT32 bit11:1;
    UINT32 bit12:1;
    UINT32 bit13:1;
    UINT32 bit14:1;
    UINT32 bit15:1;
    UINT32 bit16:1;
    UINT32 bit17:1;
    UINT32 bit18:1;
    UINT32 bit19:1;       
    UINT32 bit20:1;
    UINT32 bit21:1;
    UINT32 bit22:1;
    UINT32 bit23:1;
    UINT32 bit24:1;
    UINT32 bit25:1;
    UINT32 bit26:1;
    UINT32 bit27:1;
    UINT32 bit28:1;
    UINT32 bit29:1;  
    UINT32 bit30:1;
    UINT32 bit31:1;
} UINT32_BM_S;

typedef struct UINT16_BM_S_
{
    UINT16 bit0:1;
    UINT16 bit1:1;
    UINT16 bit2:1;
    UINT16 bit3:1;
    UINT16 bit4:1;
    UINT16 bit5:1;
    UINT16 bit6:1;
    UINT16 bit7:1;
    UINT16 bit8:1;
    UINT16 bit9:1;    
    UINT16 bit10:1;
    UINT16 bit11:1;
    UINT16 bit12:1;
    UINT16 bit13:1;
    UINT16 bit14:1;
    UINT16 bit15:1;
} UINT16_BM_S;

typedef struct UINT8_BM_S_
{
    UINT8 bit0:1;
    UINT8 bit1:1;
    UINT8 bit2:1;
    UINT8 bit3:1;
    UINT8 bit4:1;
    UINT8 bit5:1;
    UINT8 bit6:1;
    UINT8 bit7:1;
} UINT8_BM_S;

#endif // _DATATYPE_H_

