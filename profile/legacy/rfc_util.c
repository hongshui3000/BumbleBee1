 /**
 ********************************************************************************************************
 Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
 *********************************************************************************************************
 * @file	   	rfc_util.c
 * @brief	rfcomm param neg save and load file
 * @details   
 *
 * @author	gordon
 * @date	  	2015-06-23
 * @version  	v0.1
 */

#include <flags.h>
//#include <basetype.h>
//#include <os.h>
//#include <blueface.h>
#include <rfc_code.h>
//#include "btcommon.h"
#include "rfcomm.h"

/**
* @brief  rfcomm load param neg
* 
* @param  tc
* @param buf 
* @param len
*
* @return  
*
*/
int rfcLoadPN(TrfcChannel * tc, uint8_t *buf, uint16_t len)
{
    if (len != 8)
	{
    	return 1; /* too short, too long */
	}

    tc->dlci             = buf[0] & 0x3f;
    tc->frameType        = buf[1] & 0x0f;
    tc->convergenceLayer = (buf[1] >> 4) & 0x0f;
    tc->priority         = buf[2] & 0x3f;
    tc->T1               = buf[3];
    tc->frameSize        = (UINT16) buf[4] + ((UINT16) buf[5] << 8);
    tc->N2               = buf[6];
    tc->windowSize       = buf[7] & 0x07;
    return 0;
}

/**
* @brief  rfcomm save param neg
* 
* @param  tc
* @param buf 
*
* @return  
*
*/
void rfcSavePN(TrfcChannel * tc, uint8_t *buf)
{
    buf[0] = tc->dlci;
    buf[1] = tc->frameType + (tc->convergenceLayer << 4);
    buf[2] = tc->priority;
    buf[3] = tc->T1;
    buf[4] = (UINT8)(tc->frameSize & 0xff);
    buf[5] = (UINT8)(tc->frameSize >> 8);
    buf[6] = tc->N2;
    buf[7] = tc->windowSize;
}

