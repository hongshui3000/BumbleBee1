/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       rfc_api.h
* @brief     
* @details   
*
* @author  	gordon
* @date      	2015-07-13
* @version	v0.1
*/

#ifndef __RFC_API_H
#define __RFC_API_H

#include <flags.h>
#include <rtl_types.h>

#include <rfcomm.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    RFC_SUCCESS,
    RFC_INVALID_PARA,
    RFC_INVALID_STATE,
    RFC_NO_RESOURCE,
    RFC_WRONG_STATE,
    RFC_UNEXPECTED,
}RfcResult;


typedef enum
{
    RFC_CONNECT_IND,          /**< rfcomm connect indicate */
    RFC_CONNECT_RSP,          /**< rfcomm  connect response */
    RFC_CONNECT_COMPLETE,     /**< rfcomm  connect successful */
    RFC_AUTHORIZATION_IND,
    
    RFC_DISC_IND,             /**< rfcomm  disconnect indicate */
    RFC_DISC_RSP,              /**< rfcomm  disconnect response*/

    RFC_FLOW_IND,
    RFC_DATA_IND,             /**< rfcomm  data indicate */
}RfcMsgType;

typedef struct TRfcProMsg 
{	
	uint16_t  status;  
	uint8_t handle;
	uint8_t dlci;                          /**< protocol service multiplexer, 0 for unused */
	uint8_t  uuid;                           /**<  */
	TBdAddr bd; 	
	uint16_t frameSize;
	bool creditBased; 
	uint8_t outgoing;
	bool fc;	/*1 blocked, 0 unblocked*/
	uint16_t dataLength;
	uint8_t *buf;
} TRfcProMsg;
/** Connect RESPONSE */
typedef struct                  /**< RFCOMM specific part of ConResp  */
{
    uint16_t frameSize;
    uint8_t mscStatus;
    uint8_t maxCredits;
} TBtConRespRfc;
/*
typedef struct
{
    uint16_t    cid;
    uint16_t    usQueueID;
    uint8_t      remote_BD[BLUE_API_BD_SIZE];
} TBlueAPI_L2cConInd;
typedef TRfcConInd * PRfcConInd;

typedef struct
{
    uint16_t    cid;
    uint16_t    usQueueID;
    uint8_t      remote_BD[BLUE_API_BD_SIZE];
} TBlueAPI_L2cConInd;
typedef TRfcFlowInd * PRfcFlowInd;

typedef struct
{
    uint16_t    cid;
    uint16_t    usQueueID;
    uint8_t      remote_BD[BLUE_API_BD_SIZE];
} TBlueAPI_L2cConInd;
typedef TRfcDataInd * PRfcDataInd;
*/
//typedef void (* pRfcProCallback)( RfcMsgType msg_type,void* buf );
typedef void (* pRfcProCallback)( RfcMsgType msg_type,TRfcProMsg msg);

/** @brief  protocol description */
typedef struct _TRfcProDesc 
{
    uint8_t  server_channel;             /**< protocol service multiplexer, 0 for unused */
    uint16_t  uuid;                           /**<  */
    pRfcProCallback cb;                     /**< point to callback function */
} TRfcProDesc,*PRfcProDesc;



/*APIs provided to upstream*/
RfcResult rfcRegister( uint16_t  uuid, uint8_t* pChanneID,pRfcProCallback callback);
RfcResult rfcHandleUpFlowReq(uint8_t handle, uint8_t status, uint8_t sbreak);
//void rfchandleUpConReq(UINT8 * bd,UINT8 flowControlMode,UINT8 serverChannel, UINT16 frameSize,UINT8 status,UINT8 maxCredits, UINT16 uuid);
RfcResult rfcHandleUpConReq(uint8_t* bd, uint8_t serverChannel, uint16_t frameSize, uint8_t flowStatus,  uint8_t maxCredits,uint16_t uuid);
RfcResult rfcHandleUpDiscReq(uint16_t handle);
RfcResult rfcHandleUpDataReq( uint8_t * bd, uint8_t dlci,uint8_t* pbuffer,uint32_t length);
RfcResult rfcHandleUpDataResp(uint8_t handle);
void rfcSendConResp(UINT16 cid,               /* Identifier for this connection */
									UINT16 channel,            /* protocol specific channel info */
									UINT16 status,             /* result of the connect indication  */
									TBtConRespRfc   rfc);
									//PBtConRespPSpecifc p       /* protocol specific part         */);
void rfcNotifyUpstream(RfcMsgType msg_type,UINT16 status,TrfcChannel * pRfcChannel,PBlueAPI_L2cDataInd pL2cDataInd);

#ifdef __cplusplus
}
#endif

#endif
