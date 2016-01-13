/**
*****************************************************************
*	Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************
* @file       swtimer.h
* @brief      CAPI Manager general definitions
* @details   
*
* @author   	gordon
* @date      	2015-07-13
* @version	v0.1
*/ 
#if ! defined (__SWTIMER_H)
#define __SWTIMER_H

#include <osif.h>

#if defined (__cplusplus)
extern "C" {
#endif

void swTimerCallBack(void * xTimer);


int swInit(void);

#if defined (__cplusplus)
 }
#endif


#endif
 








