/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       stextif.h
* @brief      Bluetooth stack library external interface routines
* @details   
*
* @author   	gordon
* @date      	2015-07-13
* @version	v0.1
*/

#ifndef __STEXTIF_H
#define __STEXTIF_H

#include <stdint.h>
#include <comapi.h>

#ifdef __cplusplus
extern "C" {
#endif



/** NVRAM interface routines */

int stNvramInit( uint32_t size )
{
	return 0;
}
int stNvramRead( void * pData, uint32_t size )
{
	return 0;
}
int stNvramWrite( void * pData, uint32_t size )
{
	return 0;
}


/** Trace interface routines */

int stTraceInit( void )
{
	return 0;
}
int stTraceOutput( void * pData, uint16_t size )
{
	return 0;
}



/****************************************************************************/
/* Open - called by COMAPI Client                                           */
/****************************************************************************/
/* Function name  : comOpen                                                 */
/* Description    : opens a communication channel                           */
/* Return type    : TComResult       see definition of enum                 */
/* Argument       : pszDeviceName    the name of device to open             */
/* Argument       : pszPortName      specific channel within the device     */
/* Argument       : callBack         pointer to client callback function    */
/* Argument       : callBackContext  a client specific context pointer,     */
/* will be used as argument for the callback                                */
/* Argument       : phCom            pointer to a var of type HCOM used as id*/
/* in all function calls                                                    */
/****************************************************************************/
COMENTRY comOpen(
    PComCallBack callBack,
    void        *callBackContext)
{
    //OpenLowerStack(callBack, callBackContext);

    return (comResultPending);
}


/****************************************************************************/
/* Close - called by COMAPI Client                                          */
/****************************************************************************/
/* Function name  : comClose                                                */
/* Description    : close the communication channel identified by hCom      */
/* Return type    : TComResult              see definition of enum          */
/* Argument       : hCom                    the handle provided by comOpen()*/
/****************************************************************************/

COMENTRY comClose ()
{
    //CloseLowerStack();

    return (comResultSuccess);
}


/****************************************************************************/
/* Write - called by COMAPI Client                                          */
/****************************************************************************/
/* Function name : comWrite                                                 */
/* Description   : writes uBufferLength chars to the communication channel  */
/* Return type   : TComResult              see definition of enum           */
/* Argument      : hCom                    the handle provided by comOpen() */
/* Argument      : pvBuffer                pointer to a buffer containing the data*/
/* Argument      : uBufferLength           number of byte to write          */
/****************************************************************************/

COMENTRY comWrite (uint8_t pkt_type, uint8_t *pvBuffer, uint32_t uBufferLength)
{
    COMENTRY Result = comResultSuccess;
    
    //Result =  (COMENTRY)SendPktToLowerStack(pkt_type, pvBuffer, uBufferLength);
    return Result;

}


COMENTRY comResponse (uint8_t *pvBuffer)
{
    COMENTRY Result = comResultSuccess;
    
    return Result;
}


#ifdef __cplusplus
}
#endif

#endif /**< __STEXTIF_H */

