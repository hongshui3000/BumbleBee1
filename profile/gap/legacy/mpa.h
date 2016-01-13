/**
 *****************************************************************************************
 *               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
 *****************************************************************************************
 * @file       mpa.h
 * @brief     Head file for multi profile adapter layer
 * @detail
 * @author   kyle_xu
 * @date      2015-11-18
 * @version   v0.1
 * ****************************************************************************************
 */

#ifndef __MPA_H__
#define __MPA_H__

#include <blueapi_types.h>

#ifdef  __cplusplus
extern "C" {
#endif      /* __cplusplus */

#define DATA_CB_RELEASE        0x0001  /**< release buffer after sending */

/** @brief  l2cap upstream message which will be handled in protocol layer */
typedef enum
{
    L2CAP_CONNECT_IND,              /**< l2cap connect indicate */
    L2CAP_CONNECT_RSP,              /**< l2cap connect response */
    L2CAP_CONNECT_COMPLETE,         /**< l2cap connect successful */
    L2CAP_DATA_IND,                 /**< l2cap data indicate */
    L2CAP_DISC_IND,                 /**< l2cap disconnect indicate */
    L2CAP_DISC_RSP,                 /**< l2cap disconnect response*/
    PROTOCOL_SECURITY_REGISTER_RSP, /**< protocol security register response */
    PROTOCOL_AUTHORIZATION_IND,     /**< protocol authorization request indicate*/
}ProtocolUsMsg;

/* protocol callback function to handle l2cap upstream message */
typedef void (* pProtocolCallBack)(void* buf, ProtocolUsMsg msg);

/** @brief  SCO upstream message which will be handled in protocol layer */
typedef enum
{
    SCO_CONNECT_IND,          /**< SCO connect indicate */
    SCO_CONNECT_RSP,          /**< SCO connect response */
    SCO_CONNECT_COMPLETE,     /**< SCO connect successful */
    SCO_DISC_IND,             /**< SCO disconnect indicate */
}SCOUsMsg;

/* protocol callback function to handle SCO upstream message */
typedef void (* pSCOCallBack)(void* buf, SCOUsMsg sco_msg);

/* rfcomm callback function to handle rfcomm authentication response message */
typedef void (* pRFCAuthenticationRsp)(PBlueAPI_RFCAuthenticationRsp prsp);

/** @brief  protocol description */
typedef struct _TProtocolDes 
{
    uint16_t            psm;        /**< protocol service multiplexer, 0 for unused */
    //uint16_t  id;                           /**< queue ID for protocol layer, 0~LAST_PROTOCOL_QUEUE_ID */
    pProtocolCallBack   callback;   /**< point to callback function */
} TProtocolDesc;

/* init mpa layer */
bool mpa_Init(void);
/* allocate descripor for protocol */
int  mpa_RegisterProtocol(uint16_t psm, pProtocolCallBack callback);
/* send l2cap connect request */
void mpa_Sendl2cConReq(uint16_t psm, uint16_t uuid, uint16_t protocol_id, uint16_t mtu_size, uint8_t* remote_bd);
/* send l2cap disconnect request */
void mpa_Sendl2cDiscReq(uint16_t cid);
/* send l2cap disconnect confirm */
void mpa_Sendl2cDiscConf(uint16_t cid);
/* send l2cap connect confirm */
void mpa_Sendl2cConConf(int status, uint16_t cid);
/* send l2cap data request */
void mpa_Sendl2cDataReq(void* buf, uint8_t offset, uint16_t cid, uint16_t length);
/* register/deregister protocol */
void mpa_Sendl2cProtocolRegister(uint16_t psm, uint16_t listen_id, uint8_t action);
/* register/deregister security for protocol */
void mpa_SendL2cSecurityRegister(uint8_t active,
                                             uint8_t outgoing,
                                             uint8_t psm,
                                             uint16_t server_channel,
                                             uint16_t uuid,
                                             bool authentication,
                                             bool authorization,
                                             bool encryption,
                                             bool mitm
                                             );
/* send authentication request for rfcomm connection */
void mpa_SendRFCAuthenticationReq(uint8_t *bd,
                                                uint16_t channel,
                                                uint16_t dlci,
                                                uint16_t uuid,
                                                uint8_t outgoing,
                                                uint8_t active
                                                );
/* register SCO callback */
void mpa_RegisterSCOCb(pSCOCallBack pfunc);
/* register rfcomm authentication callback */
void mpa_RegisterRFCAuthenticationCb(pRFCAuthenticationRsp pfunc);
/* send SCO connect request */
void mpa_SendSCOConReq(uint8_t *remote_bd,
                                   uint32_t tx_bandwidth,
                                   uint32_t rx_bandwidth,
                                   uint16_t max_latency,
                                   uint16_t voice_setting,
                                   uint8_t  retrans_effort,
                                   uint16_t packet_type
                                   );
/* send SCO connect confirm */
void mpa_SendSCOConConf(uint8_t *remote_bd,
                                    uint32_t tx_bandwidth,
                                    uint32_t rx_bandwidth,
                                    uint16_t max_latency,
                                    uint16_t voice_setting,
                                    uint8_t  retrans_effort,
                                    uint16_t packet_type,
                                    TBlueAPI_Cause cause
                                    );
/* send SCO disconnect request */
void mpa_SendSCODiscReq(uint8_t *remote_bd);

/* handle blueAPI upstream message */
void mpa_HandleMessage(PBlueAPI_UsMessage pmsg);

#ifdef  __cplusplus
}
#endif      /*  __cplusplus */
#endif      /*  __MPA_H__ */

