/**
 * Copyright (c) 2015, Realsil Semiconductor Corporation. All rights reserved.
 *
 */

#ifndef __OS_MESSAGE_H
#define __OS_MESSAGE_H

#include <stdint.h>
#include <message.h>

#ifdef __cplusplus
extern "C" {
#endif

#define OS_FIRST_QUEUE_ID           0x10
#define OS_QUEUE_ILLEGAL_HANDLE     0xFF

int osMessageQueueCreate(uint8_t *pQueueHandle);

int osMessageQueueElementCountGet(uint8_t QueueHandle, uint16_t *pElementCount);

int osMessageSend(uint8_t QueueHandle, MESSAGE_P pMessage);

int osMessageReceive(uint8_t QueueHandle, MESSAGE_P pMessage);

#ifdef __cplusplus
 }
#endif

#endif /* __OS_MESSAGE_H */
