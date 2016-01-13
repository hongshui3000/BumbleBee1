/**
 * Copyright (c) 2015, Realsil Semiconductor Corporation. All rights reserved.
 *
 */

#ifndef __OSIF_H
#define __OSIF_H

#include <stdint.h>
#include <stdlib.h>
#include <rtl_types.h>


#ifdef __cplusplus
extern "C" {
#endif

/** OSIF is now ready, OsifActive must be called */
typedef void (* TOsifReady)                (void);

/** called after TOsifReady */
typedef void (* TOsifActive)            (void);

int osifInit(void);

int osifStart(void);

/* OS task & schedule interfaces */
uint32_t osifGetSystemTime(void);
void osifInitScheduleSignal(uint32_t Signal);
void osifSendScheduleSignal(uint32_t Signal);
void osifReceiveScheduleSignal(uint32_t Singal);
void osifTaskCreate(const char *TaskName, void (*TaskRoutine)(void *), void *TaskParameter,
    uint16_t StackSize, uint16_t TaskPriority, void *TaskHandle);


/* OS memory management interfaces */
void *osifMemoryAllocate(RAM_TYPE RamType, size_t Size);
void *osifMemoryClearAllocate(RAM_TYPE RamType, size_t Size);
void *osifMemoryAlignAllocate(RAM_TYPE RamType, size_t Size, uint8_t Alignment);
void osifMemoryFree(void *pBlock);

/* OS software timer interfaces */
void osifGetTimerID(void **pHandle, uint8_t *pQueueID, uint8_t *pTimerID, uint16_t *pTimerChannel);
void osifStartTimer(void **pHandle, uint8_t QueueID, uint8_t TimerID,
        uint16_t TimerChannel, uint32_t IntervalInMS, void (*pTimerCallback)(void *));
void osifRestartTimer(void **pHandle, uint8_t QueueID, uint8_t TimerID,
        uint16_t TimerChannel, uint32_t IntervalInMS, void (*pTimerCallback)(void *));
void osifStopTimer(void **pHandle);
void osifDeleteTimer(void **pHandle);

/* OS interrupt & critical section interfaces */
void osInitCriticalSection( void);
int osifEnterCriticalSection(void *pParam);
void osifExitCriticalSection(void *pParam, int Flags);
void osifEnterInterrupt(void);
void osifExitInterrupt(void);

#ifdef __cplusplus
}
#endif

#endif /* __OSIF_H */
