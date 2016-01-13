/**
 * Copyright (c) 2015, Realsil Semiconductor Corporation. All rights reserved.
 *
 */

#ifndef __OS_INTR_H
#define __OS_INTR_H

#ifdef __cplusplus
extern "C" {
#endif

int osInterruptDisable(void);

void osInterruptEnable(int s);

#ifdef __cplusplus
 }
#endif

#endif /* __OS_INTR_H */

