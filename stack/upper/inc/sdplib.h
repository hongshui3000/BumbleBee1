/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       sdplib.h
* @brief     sdp using util
* @details   
*
* @author   	gordon
* @date      	2015-07-13
* @version	v0.1
*/
#ifndef __SDPLIB_H
#define __SDPLIB_H
#include <rtl_types.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SDP_CREATE_DES_MAX_NESTING  7

/** get address of nth element , start index is 1. ds points to des-sequence, dse is end pointer*/
/** if not accessible, return NULL                                                              */
uint8_t * sdpAccessElement(uint8_t * ds, uint8_t * dse, uint16_t ix);

/** return value field from encoded element at p, pe is limit pointer for p. p is either s-int, */
/** uint, uuid or boolean.                                                                      */
uint32_t sdpGetDValue(uint8_t * p, uint8_t * pe);

/** extract string value into buf as cstring from element at p. pe is end pointer               */
/** in case of any error return empty string                                                    */
/** truncate string if too long to fit                                                          */
uint16_t sdpGetString(uint8_t * p, uint8_t * pe, uint8_t * buf, uint16_t buflen);

/** find attribute value "aVal" in list of attribute value pairs seq,eseq. If found returns     */
/** pointer to value, else NULL                                                                 */
uint8_t * sdpFindAttribute(uint8_t * seq, uint8_t * eseq, uint32_t aVal);

/** find an attribute element record with ID "sdpAttribute" in SDP record "pSdpRecord"          */
/** return pointer to structure if found, else NULL                                             */
uint8_t *
sdpAccessAttributeElement(uint8_t * pSdpRecord, uint16_t lenSdpRecord, uint32_t sdpAttribute);

int sdpCompareUUID(uint8_t * el1, uint8_t * el2);
int compareUUID(uint8_t * el1, int len1, uint8_t * el2, int len2);

uint8_t * sdpDecodeElement(uint8_t * element, uint8_t * elemEnd, LPWORD plen, uint8_t * ptyp);
uint32_t  sdpDesHeader(uint8_t * buf, uint32_t bp, uint8_t typ, uint32_t len);

/** extract UUID value from encodec element at p. pe is limit pointer for p. p must be        , */
/** SDP_TYPE_UUID. UUID is extracted and expanded into buf (16 byte storage)                    */
/** returns pointer to buf for correct operation or NULL in case of any error or problem        */
uint8_t * sdpGetUUIDValue(uint8_t * p, uint8_t * pe, uint8_t * buf);

/** expand UUID value from 16 / 32 bit to full 128 bit format into user supplied buffer at buf  */
/** return pointer to buf                                                                       */
uint8_t * sdpExpandUUIDValue(uint32_t uuid, uint8_t * buf);

/** compresses a standard 128 bit UUID to a 16 / 32 bit UUID if possible                        */
/** returns 32 bit standard UUID if compression is possible, otherwhise -1                      */
uint32_t sdpCompressUUIDValue(uint8_t * buf);
uint32_t sdpCompressUUIDValue128(uint8_t * buf, uint8_t * uuid);

#ifdef __cplusplus
}
#endif

#endif
