/**
 * Copyright (c) 2015, Realsil Semiconductor Corporation. All rights reserved.
 *
 */

#ifndef __OS_H
#define __OS_H

#include <stdint.h>
#include <rtl_types.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RET_OK   0x0000
#define ER_MEM   0x0001         /**< no memory available          */
#define ER_QID   0x0002         /**< queue id error               */
#define ER_PID   0x0003         /**< partition id error           */
#define ER_QUE   0x0004         /**< no queues available          */
#define ER_POOL  0x0005         /**< pool id error                */
#define ER_ZWS   0x0006
#define ER_RESNF 0x0007         /**< resource not free            */
#define ER_NIMPL 0x0008         /**< function not supported       */
#define ER_PARAM 0x0009         /**< wrong parameter              */

int  osInit(void);
void osStart(void);

#ifdef __cplusplus
 }
#endif

#endif /* __OS_H */
