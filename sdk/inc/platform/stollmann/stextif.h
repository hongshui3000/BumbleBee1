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



#ifdef __cplusplus
extern "C" {
#endif



/** NVRAM interface routines */

int stNvramInit( uint32_t size );
int stNvramRead( void * pData, uint32_t size );
int stNvramWrite( void * pData, uint32_t size );


/** Trace interface routines */

int stTraceInit( void );
int stTraceOutput( void * pData, uint16_t size );


/** Apple Coprocessor interface routines */

int stAuthCoProcInit( void );
int stAuthCoProcWrite( uint8_t registerAddress, void * pData, uint8_t size );
int stAuthCoProcRead( uint8_t registerAddress, void * pData, uint8_t size );



#ifdef __cplusplus
}
#endif

#endif /**< __STEXTIF_H */

