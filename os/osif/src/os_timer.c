/**
 * Copyright (c) 2015, Realsil Semiconductor Corporation. All rights reserved.
 *
 */

#include <stdint.h>
#include <osif.h>

void osGetTimerID(void **pHandle, uint8_t *pQueueID, uint8_t *pTimerID, uint16_t *pTimerChannel)
{
    osifGetTimerID(pHandle, pQueueID, pTimerID, pTimerChannel);
}

void osStartTimer(void **pHandle, uint8_t QueueID, uint8_t TimerID,
        uint16_t TimerChannel, uint32_t IntervalInMS, void (*pTimerCallback)(void *))
{
    osifStartTimer(pHandle, QueueID, TimerID, TimerChannel, IntervalInMS, pTimerCallback);
}

void osRestartTimer(void **pHandle, uint8_t QueueID, uint8_t TimerID,
        uint16_t TimerChannel, uint32_t IntervalInMS, void (*pTimerCallback)(void *))
{
    osifRestartTimer(pHandle, QueueID, TimerID, TimerChannel, IntervalInMS, pTimerCallback);
}

void osStopTimer(void **pHandle)
{
    osifStopTimer(pHandle);
}

void osDeleteTimer(void **pHandle)
{
    osifDeleteTimer(pHandle);
}
