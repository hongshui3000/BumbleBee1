//TODO other cpu clk?

#include "rtl_delay.h"

// for cpu 40MHz in fpga
void delayMS(uint32_t t)
{
    // tmp
    t = t*2700;

    uint32_t i;
    for(i=0;i<t;++i)
    {
        __asm volatile("nop");
        __asm volatile("nop");
        __asm volatile("nop");
        __asm volatile("nop");
        __asm volatile("nop");
        __asm volatile("nop");
        __asm volatile("nop");
        __asm volatile("nop");
        __asm volatile("nop");
        __asm volatile("nop");
    }
}

void delayUS(uint32_t t)
{
    // tmp
    t = t*2;

    uint32_t i;
    for(i=0;i<t;++i)
    {
        __asm volatile("nop");
        __asm volatile("nop");
        __asm volatile("nop");
        __asm volatile("nop");
        __asm volatile("nop");
        __asm volatile("nop");
        __asm volatile("nop");
        __asm volatile("nop");
        __asm volatile("nop");
        __asm volatile("nop");
    }
}

