/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file        att.h
* @brief      ATT (Attribute Protocol) internal definitions
* @details   
*
* @author   	gordon
* @date      	2015-07-10
* @version	v0.1
*/

#ifndef __ATTDEF_H
#define __ATTDEF_H

#if !defined(__FLAGS_H)
#include <flags.h>
#endif

#if !defined(__GATTDEF_H)
#include <gattdef.h>
#endif

#if !defined(__ATT_H)
#include <att.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**--- ATT.C ---*/
void attTimerStop(PGATTL2CChannel pL2CChannel, BOOL ServerTimer );
void attSendATT_ERROR_RESPONSE(PGATTL2CChannel pL2CChannel,
                                               uint8_t  bReqOpcode,
                                               uint16_t  wHandle,
                                               uint8_t  bErrorCode );

void attSendATT_EXCHANGE_MTU_REQUEST(PGATTL2CChannel pL2CChannel );
uint16_t attSendATT_READ_BY_GROUP_TYPE_REQUEST( PGATTL2CChannel pL2CChannel,
                                            uint16_t   wStartingHandle,
                                            uint16_t   wEndingHandle,
                                            uint8_t * pAttGroupType,
                                            int    iAttGroupTypeLength );
uint16_t attSendATT_FIND_BY_TYPE_VALUE_REQUEST( PGATTL2CChannel pL2CChannel,
                                            uint16_t            wStartingHandle,
                                            uint16_t            wEndingHandle,
                                            int             iUUIDLength,
                                            uint8_t *          pUUID);
uint16_t attSendATT_READ_BY_TYPE_REQUEST( PGATTL2CChannel pL2CChannel,
                                      uint16_t            wStartingHandle,
                                      uint16_t            wEndingHandle,
                                      int             iUUIDLength,
                                      uint8_t *          pUUID);
uint16_t attSendATT_FIND_INFO_REQUEST( PGATTL2CChannel pL2CChannel,
                                   uint16_t            wStartingHandle,
                                   uint16_t            wEndingHandle );
uint16_t attSendATT_READ_REQUEST( PGATTL2CChannel pL2CChannel,
                              uint16_t            wHandle );
uint16_t attSendATT_READ_BLOB_REQUEST( PGATTL2CChannel pL2CChannel,
                                   uint16_t            wHandle,
                                   uint16_t            wValueOffset );
uint16_t attSendATT_WRITE_REQ_CMD( PGATTL2CChannel pL2CChannel,
                               BOOL            Cmd,
                               uint16_t            wHandle,
                               uint8_t *          pBuffer,
                               LPWORD          pwOffset,
                               LPWORD          pwLength
                             );


uint16_t attSendATT_PREPARE_WRITE_REQ( PGATTL2CChannel pL2CChannel,
                                   uint16_t            wWriteOffset,
                                   uint16_t            wHandle,
                                   uint8_t *          pBuffer,
                                   LPWORD          pwOffset,
                                   LPWORD          pwLength
                                 );
uint16_t attSendATT_EXECUTE_WRITE_REQ( PGATTL2CChannel pL2CChannel,
                                   uint8_t            bFlags
                                 );
void attSendATT_PREPARE_WRITE_RESPONSE( PGATTL2CChannel  pL2CChannel,
                                        uint16_t   wHandle,
                                        uint16_t   wWriteOffset,
                                        int    iSize,
                                        uint8_t * pValue );
void attSendATT_EXECUTE_WRITE_RESPONSE( PGATTL2CChannel pL2CChannel );

typedef void (* TATTRespFunction) ( PGATTL2CChannel  pL2CChannel,
                                    int    iCnt,
                                    int    iSize,
                                    uint8_t * pList );

void attSendATT_READ_BY_GROUP_TYPE_RESPONSE(
                                PGATTL2CChannel  pL2CChannel,
                                int    iCnt,
                                int    iSize,
                                uint8_t * pList );
void attSendATT_FIND_BY_TYPE_VALUE_RESPONSE(
                                PGATTL2CChannel  pL2CChannel,
                                int    iCnt,
                                int    iSize,
                                uint8_t * pList );
void attSendATT_READ_BY_TYPE_RESPONSE(
                                PGATTL2CChannel  pL2CChannel,
                                int    iCnt,
                                int    iSize,
                                uint8_t * pList );
void attSendATT_FIND_INFO_RESPONSE(
                                PGATTL2CChannel  pL2CChannel,
                                int    iCnt,
                                int    iSize,
                                uint8_t * pList );
void attSendATT_READx_RESPONSE( PGATTL2CChannel  pL2CChannel,
                                uint8_t   bOpCode,
                                int    iSize,
                                uint8_t * pValue );
int  attSendATT_HANDLE_VALUE_NOTIF_IND( PGATTL2CChannel pL2CChannel,
                                        BOOL   Notify,
                                        uint8_t * pBuffer, LPWORD pwOffsetPayload,
                                        uint16_t   wHandle,
                                        int  * piLength,
                                        uint8_t * pValue );

BOOL attDataReceived( PGATTL2CChannel pL2CChannel,
                                                uint8_t * pBuffer, int iLength );
void attSendATT_GENERIC_PDU( PGATTL2CChannel pL2CChannel, uint8_t  bOpcode, int iLength , 
                                 uint8_t parm1, uint8_t parm2, uint16_t parm3,
                                 int param4, uint8_t * param5);
#ifdef __cplusplus
}
#endif

#endif /* __ATTDEF_H */

