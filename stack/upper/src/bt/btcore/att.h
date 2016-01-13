/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file        att.h
* @brief      Attribute protocol (ATT) definitions
* @details   
*
* @author   	gordon
* @date      	2015-07-10
* @version	v0.1
*/

#ifndef __ATT_H
#define __ATT_H


#include <rtl_types.h>
#include <att_code.h>


/** transaction timeout in mseconds */
#define ATT_TRANSACTION_TIMEOUT_VALUE    30000   

/** ATT PDU definitions */

/** generic ATT PDU*/
typedef PACKED(struct) _ATTPDUGeneric       
{
	uint8_t          Opcode;
	uint8_t          Param[1];
} TATTPDUGeneric, * PATTPDUGeneric;
typedef TATTPDUGeneric   * LPATTPDUGeneric;

/** Error Response*/
typedef PACKED(struct) _ATTErrorResp         
{
	uint8_t          Opcode;
	uint8_t          ReqOpcode;
	uint8_t          Handle[2];
	uint8_t          ErrorCode;
} TATTErrorResp, * PATTErrorResp;
typedef TATTErrorResp   * LPATTErrorResp;

/** Exchange MTU Request*/
typedef PACKED(struct) _ATTExMTUReq          
{
	uint8_t          Opcode;
	uint8_t          ClientRxMTU[2];
} TATTExMTUReq, * PATTExMTUReq;
typedef TATTExMTUReq   * LPATTExMTUReq;

/** Exchange MTU Response*/
typedef PACKED(struct) _ATTExMTUResp          
{
	uint8_t          Opcode;
	uint8_t          ServerRxMTU[2];
} TATTExMTUResp, * PATTExMTUResp;
typedef TATTExMTUResp   * LPATTExMTUResp;

/**  Read by Group Type Request */
typedef PACKED(struct) _ATTReadGroupTypeReq  
{
	uint8_t          Opcode;
	uint8_t          StartingHandle[2];
	uint8_t          EndingHandle[2];
	uint8_t          AttGroupType[1];
} TATTReadGroupTypeReq, * PATTReadGroupTypeReq;
typedef TATTReadGroupTypeReq   * LPATTReadGroupTypeReq;

/** Read by Group Type Response */
typedef PACKED(struct) _ATTReadGroupTypeResp   
{
	uint8_t          Opcode;
	uint8_t          Length;            /**< length of single list element */
	uint8_t          AttDataList[1];
} TATTReadGroupTypeResp, * PATTReadGroupTypeResp;
typedef TATTReadGroupTypeResp   * LPATTReadGroupTypeResp;

/** Find by Type Value Request */
typedef PACKED(struct) _ATTFindTypeValueReq   
{
	uint8_t          Opcode;
	uint8_t          StartingHandle[2];
	uint8_t          EndingHandle[2];
	uint8_t          AttType[2];
	uint8_t          AttValue[1];
} TATTFindTypeValueReq, * PATTFindTypeValueReq;
typedef TATTFindTypeValueReq   * LPATTFindTypeValueReq;

/** Find by Type Value Response */
typedef PACKED(struct) _ATTFindTypeValueResp   
{
	uint8_t          Opcode;
	uint8_t          HandlesInfoList[1];
} TATTFindTypeValueResp, * PATTFindTypeValueResp;
typedef TATTFindTypeValueResp   * LPATTFindTypeValueResp;

/** Read by Type Request */
typedef PACKED(struct) _ATTReadTypeReq        
{
	uint8_t          Opcode;
	uint8_t          StartingHandle[2];
	uint8_t          EndingHandle[2];
	uint8_t          AttType[1];
} TATTReadTypeReq, * PATTReadTypeReq;
typedef TATTReadTypeReq   * LPATTReadTypeReq;

/** Read by Type Response */
typedef PACKED(struct) _ATTReadTypeResp        
{
	uint8_t          Opcode;
	uint8_t          Length;            /**< length of each handle/value pair */
	uint8_t          AttDataList[1];
} TATTReadTypeResp, * PATTReadTypeResp;
typedef TATTReadTypeResp   * LPATTReadTypeResp;

/** Find Information Request */
typedef PACKED(struct) _ATTFindInfoReq          
{
	uint8_t          Opcode;
	uint8_t          StartingHandle[2];
	uint8_t          EndingHandle[2];
} TATTFindInfoReq, * PATTFindInfoReq;
typedef TATTFindInfoReq   * LPATTFindInfoReq;

/** Find Information Response */
typedef PACKED(struct) _ATTFindInfoResp        
{
	uint8_t          Opcode;
	uint8_t          Format;
	uint8_t          InfoData[1];
} TATTFindInfoResp, * PATTFindInfoResp;
typedef TATTFindInfoResp   * LPATTFindInfoResp;

#define ATT_FIND_INFO_RESP_FORMAT_UUID_16BIT    0x01
#define ATT_FIND_INFO_RESP_FORMAT_UUID_128BIT   0x02

/** Read Request */
typedef PACKED(struct) _ATTReadReq             
{
	uint8_t          Opcode;
	uint8_t          AttHandle[2];
} TATTReadReq, * PATTReadReq;
typedef TATTReadReq   * LPATTReadReq;

/** Read Response */
typedef PACKED(struct) _ATTReadResp             
{
	uint8_t          Opcode;
	uint8_t          AttValue[1];
} TATTReadResp, * PATTReadResp;
typedef TATTReadResp   * LPATTReadResp;

/** Read Blob Request */
typedef PACKED(struct) _ATTReadBlobReq          
{
	uint8_t          Opcode;
	uint8_t          AttHandle[2];
	uint8_t          ValueOffset[2];
} TATTReadBlobReq, * PATTReadBlobReq;
typedef TATTReadBlobReq   * LPATTReadBlobReq;

/** Read Blob Response */
typedef PACKED(struct) _ATTReadBlobResp        
{
	uint8_t          Opcode;
	uint8_t          PartAttValue[1];
} TATTReadBlobResp, * PATTReadBlobResp;
typedef TATTReadBlobResp   * LPATTReadBlobResp;

/** Read Multiple Request */
typedef PACKED(struct) _ATTReadMultiReq         
{
	uint8_t          Opcode;
	uint8_t          SetOfHandles[2];
} TATTReadMultiReq, * PATTReadMultiReq;
typedef TATTReadMultiReq   * LPATTReadMultiReq;

/** Read Multiple Response */
typedef PACKED(struct) _ATTReadMultiResp        
{
	uint8_t          Opcode;
	uint8_t          SetOfValues[1];
} TATTReadMultiResp, * PATTReadMultiResp;
typedef TATTReadMultiResp   * LPATTReadMultiResp;

/** Write Request */
typedef PACKED(struct) _ATTWriteReq            
{
	uint8_t          Opcode;
	uint8_t          AttHandle[2];
	uint8_t          AttValue[1];
} TATTWriteReq, * PATTWriteReq;
typedef TATTWriteReq   * LPATTWriteReq;

/** Write Response */
typedef PACKED(struct) _ATTWriteResp           
{
	uint8_t          Opcode;
} TATTWriteResp, * PATTWriteResp;
typedef TATTWriteResp   * LPATTWriteResp;

typedef TATTWriteReq TATTWriteCmd, * PATTWriteCmd;  /**< Write Command */
typedef TATTWriteCmd   * LPATTWriteCmd;

/** Prepare Write Request */
typedef PACKED(struct) _ATTPrepareWriteReq         
{
	uint8_t          Opcode;
	uint8_t          AttHandle[2];
	uint8_t          ValueOffset[2];
	uint8_t          PartAttValue[1];
} TATTPrepareWriteReq, * PATTPrepareWriteReq;
typedef TATTPrepareWriteReq   * LPATTPrepareWriteReq;

typedef TATTPrepareWriteReq  TATTPrepareWriteResp, * PTATTPrepareWriteResp;
typedef TATTPrepareWriteResp  * LPATTPrepareWriteResp;

/** Execute Write Request */
typedef PACKED(struct) _ATTExecuteWriteReq         
{
	uint8_t          Opcode;
	uint8_t          Flags;
} TATTExecuteWriteReq, * PATTExecuteWriteReq;
typedef TATTExecuteWriteReq   * LPATTExecuteWriteReq;

/** Execute Write Response */
typedef PACKED(struct) _ATTExecuteWriteResp        
{
	uint8_t          Opcode;
} TATTExecuteWriteResp, * PATTExecuteWriteResp;
typedef TATTExecuteWriteResp   * LPATTExecuteWriteResp;

/** Handle Notification */
typedef PACKED(struct) _ATTHandleValueNotif         
{
	uint8_t          Opcode;
	uint8_t          AttHandle[2];
	uint8_t          AttValue[1];
} TATTHandleValueNotif, * PATTHandleValueNotif;
typedef TATTHandleValueNotif   * LPATTHandleValueNotif;
                                                 
typedef TATTHandleValueNotif TATTHandleValueInd, * PATTHandleValueInd;/**< Indication */
typedef TATTHandleValueInd     * LPATTHandleValueInd;

/** Handle Vaule Confirmation */
typedef PACKED(struct) _ATTHandleValueConf          
{
	uint8_t          Opcode;
} TATTHandleValueConf, * PATTHandleValueConf;
typedef TATTHandleValueConf   * LPATTHandleValueConf;

/** ATT PDU overlay */
typedef PACKED(union) _ATTPDU                
{
	uint8_t                     Opcode;
	TATTPDUGeneric           Generic;
	TATTErrorResp            ErrorResp;
	TATTExMTUReq             ExMTUReq;
	TATTExMTUResp            ExMTUResp;
	TATTReadGroupTypeReq     ReadGroupTypeReq;
	TATTReadGroupTypeResp    ReadGroupTypeResp;
	TATTFindTypeValueReq     FindTypeValueReq;
	TATTFindTypeValueResp    FindTypeValueResp;
	TATTReadTypeReq          ReadTypeReq;
	TATTReadTypeResp         ReadTypeResp;
	TATTFindInfoReq          FindInfoReq;
	TATTFindInfoResp         FindInfoResp;
	TATTReadReq              ReadReq;
	TATTReadResp             ReadResp;
	TATTReadBlobReq          ReadBlobReq;
	TATTReadBlobResp         ReadBlobResp;
	TATTReadMultiReq         TATTReadMultiReq;
	TATTReadMultiResp        TATTReadMultiResp;
} TATTPDU, * PATTPDU;
typedef TATTPDU   * LPATTPDU;

/**
* all GATT server PDU handling routines return a uint32_t:
* bits 0..7  :  ATT error code
*      8..15 :  reserved ( = 0 )
*      16..31:  attribute handle to be returned in ATT_ERROR_RESPONSE
*               if ATT error code is nonzero
*/
#define ATT_HANDLE2DWORD(dw, h)    dw &= 0x0000FFFF; \
                                   dw |= ((uint16_t)h << 16)
#define ATT_DWORD2HANDLE(dw)       (uint16_t)((0xFFFF0000 & dw) >> 16)
#define ATT_ERRCODE2DWORD(dw, e)   dw &= 0xFFFFFF00; \
                                   dw |= (uint8_t)h
#define ATT_DWORD2ERRCODE(dw)      (uint8_t)dw


#endif /**< __ATT_H */

