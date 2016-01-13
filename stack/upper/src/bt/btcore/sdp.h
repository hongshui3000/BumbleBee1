/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file 	  sdp.h
* @brief   SDP Protocol Layer: Header File with Common Definitions
* @details	 
*
* @author  gordon
* @date 	   2015-07-13
* @version	   v0.1
*/

#ifndef __SDP_H
#define __SDP_H

#ifdef __cplusplus
extern "C" {
#endif

/** Max. No Of Handles in one SDP_SEARCH_ATTRIBUTE_RESPONSE */
#define SDP_MAX_HDL                     4    /**< must be changed for embedded qualification */


/** we need one implicit service for the database state */
//#define BT_SDP_SERVER_MAX_SERVICES (BT_SDP_SERVER_MAX_SERVICES_COUNT+1)
#define BT_SDP_SERVER_MAX_SERVICES (5+1)


#define SDP_SERVICE_OFFSET              ((uint32_t)0x10000)   /**< User Services start at handle 0x10000 */
#define SDP_TIMEOUT_SERVER              60                 /**< Server waits for requests */
#define SDP_TIMEOUT_CLIENT              10                 /**< Client waits for answer */
#define SDP_CLIENT_CHANNEL              0                  /**< Index in desc table for client channel */

/** frame size used by SDP server for local work buffer on stack */
#if (BT_SDP_SERVER_BUFFER_BYTE_COUNT == 0) || (BT_SDP_SERVER_BUFFER_BYTE_COUNT > BT_DS_PDU_L2C_BYTE_COUNT)
#if (BT_SDP_SERVER_BUFFER_BYTE_COUNT > BT_DS_PDU_L2C_BYTE_COUNT)
#pragma message("BT_SDP_SERVER_BUFFER_BYTE_COUNT > BT_DS_PDU_L2C_BYTE_COUNT not allowed, set BT_SDP_SERVER_BUFFER_BYTE_COUNT to BT_DS_PDU_L2C_BYTE_COUNT")
#endif

#undef BT_SDP_SERVER_BUFFER_BYTE_COUNT                  /**< and framesize for uplink channel */
#define BT_SDP_SERVER_BUFFER_BYTE_COUNT   BT_DS_PDU_L2C_BYTE_COUNT
#endif /**< (BT_SDP_SERVER_BUFFER_BYTE_COUNT == 0) */

/** common structure of sdp header */

/**
*	offset             contents
*	0                 sdp code   ( 1 byte )
*     1                 trans id   ( 2 byte )
*     3                 par length ( 2 byte , network byte order !)
*     5              payload starts here
*/
#define SDP_HEADER_LENGTH               5
#define SDP_RESPONSE_OVERHEAD  (5 + 2 + 17)   /**< 5 for sdp header, 2 for AttrListByteCount, max 17 for continuation state */

/** Channel Descriptor */
typedef struct
{
    uint16_t chan;             /**< l2cap channel number */
    BOOL incoming;         /**< incoming = TRUE --> server channel */
    BOOL used;             /**< descriptor in use flag */
    uint8_t self;             /**< own index */
    uint16_t id;               /**< current transaction id */
    TBdAddr bd;            /**< bd of remote party */
    uint16_t mtuSize;          /**< mtu size of remote peer */
    uint16_t inMtuSize;        /**< used MTU Size for Local Peer */
    uint16_t flag;             /**< mem flag for outstanding command */
    uint8_t nextCode;         /**< Code of next expected SDP message */
    uint8_t * cmdp;           /**< args for outstanding command */
    uint16_t offset;           /**< offset of data in cmdp */
    uint8_t * respp;          /**< current assembled response */
    uint16_t index;
    uint8_t segmentationMode; /**< 0: normal reassembly mode, 1: do not reassemble */
    uint8_t continuationLen;  /**< unsegmented mode: length of last continuation state */
    uint8_t continuationState[16];/**< unsegmented mode: last continuation state, max 16 byte */
    void * TimerHandle;
} TsdpChanDesc;

/** Instance Data */
typedef struct _TSDP 
{
    uint8_t usSDPAttrDataPool;

    uint16_t readOffset;
    uint16_t writeOffset;

    uint16_t dbState;
    
    uint8_t mainRecord[32];                  /**< 32 bytes should be sufficient to hold the main record */
//    TsdpChanDesc channels[BT_SDP_MAX_CHANNELS_COUNT];
    TsdpChanDesc *pchannelsDon;
    TsdpChanDesc *pchannelsDoff;

    uint8_t * services[BT_SDP_SERVER_MAX_SERVICES];    /**< Pointer to service records, including offsets */
    uint16_t   offsets [BT_SDP_SERVER_MAX_SERVICES];    /**< corresponding offsets */

    uint8_t bufTmp[BT_SDP_SERVER_BUFFER_BYTE_COUNT];   /**< buffer used locally by SDP server for temporary operations */

    BOOL eirRunning;                                /**< indicates that EIR operation is currently in progress */
    BOOL eirPending;                                /* indicates that a new EIR operation is pending */
    uint8_t shortName[32];                             /* buffer for short version of friendly name */
    uint16_t VendorID;
    uint16_t ProductID;
    uint16_t ProductVersion;
    uint16_t IDSource;
    char *extraEIRData;
} TSDP, * PSDP;
typedef const TSDP *PCSDP;

/** Message Generators Peer */
void sdpSend(uint16_t chan, uint8_t * buf, uint16_t pos);
void sdpError(uint16_t chan, uint16_t id, uint8_t error);
void sdpServiceSearchRequest(uint16_t chan, uint16_t id, LPCBYTE sid, uint16_t sidlen,
                             uint16_t maxcnt, LPCBYTE cstate, uint8_t cstateLen);

void sdpServiceSearchResponse(uint16_t chan, uint16_t id, uint16_t maxIndex,
                              PCUINT32 hbuf, uint16_t hcnt, uint16_t follow);

void sdpServiceAttributeRequest(uint16_t chan, uint16_t id, uint32_t handle, LPCBYTE aid, uint16_t aidlen,
                                uint16_t maxcnt, LPCBYTE cstate, uint8_t cstateLen);

void sdpServiceAttributeResponse(uint16_t chan, uint16_t id, uint8_t sdpCode,
                                 LPCBYTE aid, uint16_t aLen, uint16_t cState);

void sdpServiceSearchAttributeRequest(uint16_t chan, uint16_t id,
                                      LPCBYTE sid, uint16_t sidlen,
                                      uint16_t maxcnt,
                                      LPCBYTE aid, uint16_t aidlen,
                                      LPCBYTE cstate, uint8_t cstateLen);
void sdpNewEIR(void);

/** Message Generators DOWNSTREAM */
//void sdpSendLDiscResp(uint16_t lcid, uint16_t status);

/** Handling functions from PEER */
void sdpHandleServiceSearchRequest(CONST TsdpChanDesc * tc, uint8_t * buf, uint16_t id, uint16_t blen);
void sdpHandleServiceSearchResponse(TsdpChanDesc * tc, uint8_t * buf, uint16_t id, uint16_t blen);
void sdpHandleServiceAttributeRequest(CONST TsdpChanDesc * tc, uint8_t * buf, uint16_t id, uint16_t blen);
void sdpHandleServiceAttributeResponse(TsdpChanDesc * tc, uint8_t * buf, uint16_t id, uint16_t blen);
void sdpHandleServiceSearchAttributeRequest(CONST TsdpChanDesc * tc, uint8_t * buf, uint16_t id, uint16_t blen);
void sdpHandleServiceSearchAttributeResponse(TsdpChanDesc * tc, uint8_t * buf, uint16_t id, uint16_t blen);

/** Service Database Access */
uint8_t * sdpDbSearchAttribute(uint32_t handle, uint16_t attribute, LPWORD aLen, LPWORD nextAtributeP);
BOOL   sdpDbServiceSearchOne(uint8_t * elem, uint32_t handle);
uint8_t   sdpDbServiceSearchList(uint8_t * sList, uint8_t * sEnd, uint32_t lhandle);
uint8_t   sdpDbServiceSearch(uint8_t * pattern, uint8_t * pe, uint16_t index, uint16_t totalMax, uint32_t* phandle, uint16_t maxcount, LPWORD pmaxindex);

/** Utility Functions */
uint16_t   sdpGetValue(uint8_t * p, uint8_t * pe);
void   sdpResetTimer(uint8_t timerID, TsdpChanDesc *cp);
void   sdpSetTimer(uint8_t timerID, TsdpChanDesc *cp, uint16_t seconds);
uint16_t   sdpCreateAttributeList(uint8_t * buf, uint16_t bufsize, uint16_t index, uint16_t bpos,
                              uint16_t maxByteCount, uint8_t * attrList, uint32_t handle, uint8_t deleteEmpty);
void   sdpMemCpy(uint8_t * dbuf, uint16_t dix, LPCBYTE sbuf, uint16_t scnt, uint16_t dindex, uint16_t dsize);
void   sdpCloseLink(TsdpChanDesc * tc, uint16_t status, uint16_t holdLink);
void   sdpCleanClient(TsdpChanDesc * tc);

/** Channel Descriptor Management */
TsdpChanDesc * sdpGetChannel(uint16_t chan);
TsdpChanDesc * sdpGetChannelByBd(LPCBYTE bd);
TsdpChanDesc * sdpAllocateChannel(uint16_t chan, BOOL incoming);
void sdpAttributeConf(TsdpChanDesc * tc);
void sdpSearchConf(TsdpChanDesc * tc);

#ifdef __cplusplus
}
#endif

#endif
