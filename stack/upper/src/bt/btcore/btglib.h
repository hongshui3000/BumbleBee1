/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       btglib.c
* @brief     
* @details   
*
* @author   	gordon
* @date      	2015-06-25
* @version	v0.1
*/

#ifndef __BTGLIB_H
#define __BTGLIB_H

#include <flags.h>
#include <os_message.h>

/** Prototyp Section */
void btgSendConResp(uint8_t hTargetQueue, uint16_t cid, uint16_t status, PBtConRespPSpecifc pExt);
void btgSendDiscReq(uint8_t hTargetQueue, uint16_t cid, uint16_t cause, BOOL holdLink);

#endif /**< __BTGLIB_H */
