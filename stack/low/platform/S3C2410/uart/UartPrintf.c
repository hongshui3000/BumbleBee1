#if defined(FOR_SIMULATION)

#include "UartPrintf.h"
#include "print.h"
#include "uart.h"

void MyOutput(void *arg, UINT8 *s, INT32 l)
{
    INT32 i;

    // special termination call
    if ((l==1) && (s[0] == '\0')) return;

    for (i=0; i< l; i++)
    {
        (*((volatile UINT32*)(0xb000a180)))=s[i];
    }
}

void UARTPrintf(UINT8 *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);

    GenPrint(MyOutput, 0, fmt, ap);
    va_end(ap);

    return;
}

void COLOR_UARTPrintf(UINT8 color, UINT8 *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);

    GenPrint(MyOutput, 0, fmt, ap);
    va_end(ap);

    return;
}

#endif 


