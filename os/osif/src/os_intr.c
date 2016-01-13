/**
 * Copyright (c) 2015, Realsil Semiconductor Corporation. All rights reserved.
 *
 */

#include <osif.h>

int osInterruptDisable(void)
{
    return osifEnterCriticalSection(NULL);
}

void osInterruptEnable(int s)
{
    osifExitCriticalSection(NULL, s);
}
