/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       rfcomm.h
* @brief     rfcomm layer
* @details   
*
* @author   	gordon
* @date      	2015-06-29
* @version	v0.1
*/

#ifndef __RFCOMM_H
#define __RFCOMM_H

//#include <bt_msg.h>
#ifdef __cplusplus
extern "C" {
#endif
#include <rfc_code.h>
#include <efuse_config.h> 
#include <btglbdef.h>
#include <bterrcod.h>
#include <sdp_code.h>
#include <mpa.h>
#include <stdint.h>
#include <FreeRTOS.h>
#include <queue.h>
//#include "timers.h"

extern OTP_STRUCT otp_str_data;

#define RFCOMM_HEADER_SIZE (RFCOMM_SIZE_UIH + 1)

/** RFCOMM maximum frame size */
#ifndef BT_RFCOMM_MTU_COUNT
#error("required count BT_RFCOMM_MTU_COUNT not defined - update flags")
#else
#if (BT_RFCOMM_MTU_COUNT == 0)
#undef BT_RFCOMM_MTU_COUNT
#define BT_RFCOMM_MTU_COUNT (BT_DS_PDU_L2C_BYTE_COUNT - RFCOMM_HEADER_SIZE)
#else

#if BT_RFCOMM_MTU_COUNT > (BT_DS_PDU_L2C_BYTE_COUNT - RFCOMM_HEADER_SIZE)
#error("BT_RFCOMM_MTUT does not fit into BT_DS_PDU_L2C_BYTE")
#endif /**< BT_RFCOMM_MTU_COUNT > (BT_DS_PDU_L2C_BYTE_COUNT - RFCOMM_SIZE_UIH - 1) */
#if BT_RFCOMM_MTU_COUNT > (BT_US_PDU_L2C_BYTE_COUNT - RFCOMM_HEADER_SIZE)
#error("BT_RFCOMM_MTUT does not fit into BT_US_PDU_L2C_BYTE")
#endif /**< BT_RFCOMM_MTU_COUNT > (BT_US_PDU_L2C_BYTE_COUNT - RFCOMM_SIZE_UIH - 1) */

#endif /**< (BT_RFCOMM_MTU_COUNT == 0) */
#endif /**< BT_RFCOMM_MTU_COUNT */


#define EA_BIT                        1 /**< Extension Bit */

/** Timer values etc */
#define RFCOMM_T1_ID                  1 /**< for this id, timeout is tc->handle */
#define RFCOMM_T1_TIMEOUT            10 /**< Seconds */
#define RFCOMM_T1_TIMEOUT_EXTENDED   60 /**< Seconds */

#define RFCOMM_T2_ID                  2 /**< timeout during wait for MSC_IND after UA_IND */
#define RFCOMM_T2_TIMEOUT            10 /**< Seconds */

#define RFCOMM_LAST_ID                RFCOMM_T2_ID
#define RFCOMM_DSQUEUE_NUM	3

typedef enum 
{
	dlciIdle,
	dlciIndicated,
	dlciOpening,
	dlciConfiging, 
	dlciConfigured,
	dlciConfigIndicated,
	dlciConfigAccepted,
	dlciConnecting,
	dlciConnected,
	dlciDisconnecting
} TdlciState;

typedef enum 
{
	linkIdle, 
	linkConnecting, 
	linkConnected, 
	linkDisconnectRequested
} TAcllinkState;

/** Bluetooth device address */
#define BD_ADDR_SIZE      6
typedef uint8_t TBdAddr[BD_ADDR_SIZE];

/*move from blueface.h in stack*/
/** RFCOMM IF Message Argument Definitions */
#if defined(PRAGMA_PACK)
#pragma pack(1)
#endif
//#define  PACKED(structOrUnion) __packed structOrUnion
typedef PACKED(struct)
{
    uint8_t dlci;
    uint8_t baudrate;
    uint8_t oc3;
    uint8_t flctl;
    uint8_t xon;
    uint8_t xoff;
    uint8_t pm1;
    uint8_t pm2;
} Trpn;
typedef Trpn * LPrpn;
//typedef Trpn FAR * LPrpn;
//typedef CONST Trpn FAR * LPCrpn;

#if defined(PRAGMA_PACK)
#pragma pack()
#endif

typedef struct
{
	/** first part MUST be same as DATA_CB_T: !!!! */
//	uint16_t      Flag;
	uint16_t		Length;
	uint16_t      Offset;               /**< offset to data in data buffer */
	uint16_t      payloadLength;               /**< length of data */
	uint8_t	 *BufferAddress;       /**< buffer address */
	/** extension   */
	uint16_t      Channel;              /**< Channel Identifier */
//	uint16_t      TotalLength;          /**< total length of data (more-bit) */
} TRfcUpDataMsg;

/** DLCI description structure */
typedef struct _TrfcChannel
{
	/** General Administrative Components */
	bool used;
	//struct _TRFC * pRFC;   /**< Back pointer to RFCOMM session instance data */

	/** DLCI Status Data */
	bool initiator;
	uint8_t dlci;             /**< connection dlci */
	uint8_t handle;			/**< index: from 1*/
	TdlciState state;
	/** Link Status Data (only for DLCI==0) */
	bool linitiator;       /**< initiator of the linh */
	TAcllinkState lstate;     /**< state of l2cap link */
	uint16_t channel;          /**< l2cap connection id */
	uint16_t uuid;             /**< uuid */
	TBdAddr bd;            /**< bd of l2cap connection */
	uint8_t mscHandshake;     /**< Signals: Received MSC_COMMAND (Bit 0), RECEIVED MSC_RESPONSE (Bit 1) */

	uint8_t authenticationActive;

	/** RFCOMM Link Configuration Data (only for DLCI != 0 */
	struct _TrfcChannel * cc; /**< Back pointer to control channel for this channel */
	uint8_t frameType;        /**< UIH / UI / I, default UIH */
	uint8_t convergenceLayer; /**< 0..3 for Convergence Layers 1..4, default 0 */
	uint8_t priority;
	uint8_t T1;               /**< T1 in units of 10 ms */
	uint16_t frameSize;
	uint8_t N2;               /**< from GSM 07.10. not used under RFCOMM */
	uint8_t windowSize;       /**< from GSM 07.10. not used under RFCOMM except CreditBased */
	uint16_t mtuSize;          /**< Max MTU Size for Remote Peer */

	/** Buffer usage */
	uint16_t maxUpstreamBuffers;

	/** MSC Flow Control */
	uint8_t usFlow;           /**< Value of last MSC command SENT : 1 Upstream flow is BLOCKED, 0 unblock */
	uint8_t usFlowBreak;      /**< Value of last MSC command SENT : Break status byte */
	uint8_t usFlowActive;     /**< Bit 0: Currently exists is an open MSC */
	                       /**< Bit 1: Open MSC Req stored in usFlow / usFlowBreak */

	uint8_t dsFlow;           /**< Value of last MSC command RECEIVED : Downstream flow is BLOCKED */

	/** Credit Based Flow Control */
	uint16_t remainingCredits; /**< Credit based flow control: current downstream credits remaining */
	uint16_t backCredits;      /**< Credits to return to remote side */

	/** Remote Line Status */
	uint8_t rls;              /**< Remote Line Status */
	Trpn rpn;            /**< Parameters from RPN command */
	//uint8_t dsQueueID;        /**< Downstream Queue for Data */
	xQueueHandle			dsQueueID;	  /**< Downstream Queue for Data */
	uint16_t				dsPoolID;/*stack buffer pool*/

    void * TimerHandleT1;
    void * TimerHandleT2;
//	xTimerHandle				TimerHandleT1;/*wait for response*/
//	xTimerHandle				TimerHandleT2;/*wait for msc*/
} TrfcChannel;
typedef TrfcChannel * PTrfcChannel;

typedef struct _TRFC
{
    uint16_t writeOffset;
    bool creditSupport;    /**< should credit based be used ? */
    unsigned short  QueueID;              /**< own (input) queue     */
	
#if (F_BT_L2C_ENHANCED_FEATURE_SUPPORT) && 0
    uint8_t * segP;              /**< pointer into buffer used for PDU assembly */
    uint16_t   segOffs;           /**< offset used in buffer */
    uint16_t   segLen;            /**< currently used length in assembly buffer */
    uint16_t   usPool;            /**< id of upstream pool */
#endif
//    TrfcChannel cl[RFCOMM_MAX_DLCI];
    TrfcChannel *rfcchannelDon;
    TrfcChannel *rfcchannelDoff;
} TRFC, * PRFC;


/** Peer to Peer Message Generators  */
void rfcSendSABM(TrfcChannel * tc, uint8_t command, uint8_t poll);
void rfcSendUA(TrfcChannel * tc, uint8_t command, uint8_t poll);
void rfcSendDM(TrfcChannel * cc, uint8_t dlci, uint8_t command, uint8_t poll);
void rfcSendDISC(TrfcChannel * tc, uint8_t command, uint8_t poll);
void rfcSendUIH(TrfcChannel * tc, uint8_t typ, uint8_t cr, uint8_t * p, uint16_t len);
void rfcSendFrame(TrfcChannel * cc, uint8_t dlci, uint8_t ctl, uint8_t cr, uint8_t poll, uint8_t * pl, uint16_t pllen, int highPrio);
void rfcSendMSC(TrfcChannel * tc, uint8_t cr, uint8_t dlci, uint8_t status, uint8_t sbreak);

/** Message Generators Downstream */
//void rfcSendLDiscResp(uint16_t channel, uint16_t status);

/** Message Generators Upstream */
//void rfcSendUConInd(uint8_t dlci, uint8_t *bd, uint8_t handle, uint16_t frameSize, bool creditBased, uint8_t outgoing);
//void rfcSendUFlowInd(uint8_t handle, uint8_t status, uint8_t command, uint8_t sbreak);
//void rfcSendURpn(uint8_t handle, LPrpn rpn, uint8_t len);
//void rfcSendLDiscReq(uint16_t channel);                       /* POSOP replace with Macro or direct call */

/** Utility Functions */
int rfcLoadPN(TrfcChannel * tc, uint8_t *buf, uint16_t len);
void rfcSavePN(TrfcChannel * tc, uint8_t * buf);
void rfcFreeChannel(TrfcChannel * tc);
void rfcDownstreamWork(void);

void rfcResetTimer(uint8_t timerID, TrfcChannel * tc);
void rfcSetTimer(uint8_t timerID, TrfcChannel * tc, uint16_t seconds);
//void rfcT1TimerCallback(xTimerHandle xTimer);
//void rfcT2TimerCallback(xTimerHandle xTimer);
void rfcHandleTimer(UINT8 chan,UINT8 id);

/** Channel Management Functions */
TrfcChannel * rfcAllocateDLCI(void);
void rfcInitDLCI(TrfcChannel * tc, uint8_t dlci);
TrfcChannel * rfcFindDLCI(uint8_t dlci, uint16_t channel);
TrfcChannel * rfcFindHandle(uint16_t cid);
TrfcChannel * rfcFindBd(uint8_t * bd, uint8_t dlci);
//void rfcCloseChannels(TrfcChannel * cc, uint16_t status);
void rfcCloseChannels(TrfcChannel * cc,uint16_t status);
void rfcDisconnectIfNoUserChannels(TrfcChannel * cc);

/** State Machine */
void rfcCast(TrfcChannel * tc, uint8_t state);
void rfcLCast(TrfcChannel * cc , uint8_t state);

/** Handlers from Upper Layer */
//void rfcHandleRfcErrorReq(MESSAGE_T * msg);

/** Handlers for Messages from Peer */
//void rfcHandleUIH(TrfcChannel * tc, MESSAGE_T *pmsg, uint8_t cr, uint8_t poll);
void rfcHandleUIH(TrfcChannel * tc, PBlueAPI_L2cDataInd pL2cDataInd, uint8_t cr, uint8_t poll);
void rfcHandleSABM(TrfcChannel *tc, uint8_t cr, uint8_t poll);
void rfcHandleUA(TrfcChannel *tc, uint8_t cr, uint8_t poll);
void rfcHandleDISC(TrfcChannel *tc, uint8_t cr, uint8_t poll);
void rfcHandleDM(TrfcChannel *tc, uint8_t cr);


void rfcAllocateBuffers(TrfcChannel * tc, uint16_t maxCredits);


//void rfcHandleRfcFlowReq(uint8_t handle, uint8_t status, uint8_t sbreak);
//void rfcHandleRfcRpnResp(uint8_t handle, uint8_t * rpn, uint16_t len);

/*handle stack messages from lowstream*/
void rfcHandleL2cConRsp(PBlueAPI_L2cConRsp pL2cConRsp);
void rfcHandleL2cConComplete(PBlueAPI_L2cConActInd pL2cConActInd);
void rfcHandleL2cConInd (PBlueAPI_L2cConInd pL2cConInd);
void rfcHandleL2cDataInd(PBlueAPI_L2cDataInd pL2cDataInd);
void rfcHandleL2cDiscInd(PBlueAPI_L2cDiscInd pL2cDiscInd);
void rfcHandleL2cDiscRsp(PBlueAPI_L2cDiscConf pL2cDiscConf);
void rfcHandleSecRegRsp(PBlueAPI_L2cSecurityRegisterRsp prsp);
void rfcHandleAuthorizationInd(PBlueAPI_UserAuthorizationReqInd pind);

/*
void rfcHandleSECMAN_AUTHENTICATION_RESP(  TBdAddr bd,
																  UINT16    ref,
																  UINT16    channelId,
																  UINT16    uuid,
																  UINT8    outgoing,
																  UINT16    cause);
*/
//void rfcSendSecManAuthenticationInd(PTrfcChannel tc, uint8_t outgoing, uint8_t active);
void rfcAuthenticationRspCallBack(PBlueAPI_RFCAuthenticationRsp prsp);
void rfcCallBack(void* buf, ProtocolUsMsg l2c_msg);
bool rfcInit(void);

#ifdef __cplusplus
}
#endif

#endif
