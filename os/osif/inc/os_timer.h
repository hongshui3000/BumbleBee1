/**
 * Copyright (c) 2015, Realsil Semiconductor Corporation. All rights reserved.
 *
 */

#ifndef __OS_TIMER_H
#define __OS_TIMER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void osGetTimerID(void **pHandle, uint8_t *pQueueID, uint8_t *pTimerID, uint16_t *pTimerChannel);

void osStartTimer(void **pHandle, uint8_t QueueID, uint8_t TimerID,
        uint16_t TimerChannel, uint32_t IntervalInMS, void (*pTimerCallback)(void *));

void osRestartTimer(void **pHandle, uint8_t QueueID, uint8_t TimerID,
        uint16_t TimerChannel, uint32_t IntervalInMS, void (*pTimerCallback)(void *));

void osStopTimer(void **pHandle);

void osDeleteTimer(void **pHandle);

#ifdef __cplusplus
}
#endif

#endif /* __OS_TIMER_H */
