/**
 * Copyright (c) 2015, Realsil Semiconductor Corporation. All rights reserved.
 *
 */

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include <timers.h>
#include <osif.h>
#include "stdint.h"

/****************************************************************************/
/* Init routine                                                             */
/****************************************************************************/
void stInitFreeRTOS(void)
{
    osifInit();

    osifStart();
}
