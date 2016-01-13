/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       bluemgr.h
* @brief     External BTMAN functions
* @details   
*
* @author   	gordon
* @date      	2015-07-13
* @version	v0.1
*/

#ifndef __BLUEMGR_H
#define __BLUEMGR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <os_message.h>

blueFaceStatus lblueFaceRegister           (LPblueFaceReg pReg);
void lblueFaceInit(void);

#ifdef __cplusplus
}
#endif

#endif
