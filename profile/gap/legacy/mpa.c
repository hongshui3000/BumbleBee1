enum { __FILE_NUM__ = 0 };

/**
 ***************************************************************************************
 *               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
 ***************************************************************************************
 * @file      mpa.c
 * @brief    This file provides the multi profile adapter layer functions
 * @details  Multi profile adapter is used to deliver l2cap message to protocol layer
 * @author  kyle_xu
 * @date     2015-11-18
 * @version  v0.1
 ***************************************************************************************
 */

#include <mpa.h>
#include <string.h>
#include <blueapi.h>
#include <trace.h>
#include <common_defs.h>

/** max supported protocol number, must be less than OS_FIRST_QUEUE_ID in upper stack */
#define MAX_PROTOCOL_NUM       5

/** protocol descriptor array, which is used to find callback function when handle l2cap blueapi message */
TProtocolDesc ProtocolDes[MAX_PROTOCOL_NUM];
pSCOCallBack SCOCallBack = NULL;
pRFCAuthenticationRsp RFCAuthencationCallBack = NULL;

/**
 * @brief  init mpa layer
 * @param  none
 * @return  none
 * @retval   void
 */
bool mpa_Init(void)
{
    memset(ProtocolDes, 0, MAX_PROTOCOL_NUM*sizeof(TProtocolDesc));

    return true;
}

/**
 * @brief register callback function to send sco events to profile
 * @param pfunc  callback function.
 * @return void
 */
void mpa_RegisterSCOCb(pSCOCallBack pfunc)
{
    SCOCallBack = pfunc;
}

/**
 * @brief register rfcomm authentication response callback function
 * @param pfunc  callback function.
 * @return void
 */
void mpa_RegisterRFCAuthenticationCb(pRFCAuthenticationRsp pfunc)
{
    RFCAuthencationCallBack = pfunc;
}

/**
 * @brief  allocate descripor for protocol
 *   
 *   find unused protocol desciptor in array with given psm, if found, set psm and callback fucntion,
 *   if the given psm is already allocated, update callback function
 *
 * @param  psm      protocol service multiplexer
 * @param  callback  callback function to handle l2cap message
 * @return  int
 * @retval   -1    failed with no free descriptor
 * @retval   uint  protocol descriptor id for the given psm
 * @note
 */

int mpa_RegisterProtocol(uint16_t psm, pProtocolCallBack callback)
{
    uint8_t index = 0;
    uint8_t used_index = MAX_PROTOCOL_NUM;
    uint8_t free_index = MAX_PROTOCOL_NUM;

    for (index = 0; index < MAX_PROTOCOL_NUM; index++)
    {
        if (ProtocolDes[index].psm == 0)
        {
            free_index = index;
            break;
        }
        else if (ProtocolDes[index].psm == psm)
        {
            used_index = index;
            break;
        }
    }

    if (free_index != MAX_PROTOCOL_NUM)
    {
        ProtocolDes[free_index].psm = psm;
        ProtocolDes[free_index].callback = callback;

        return free_index;
    }
    else if (used_index != MAX_PROTOCOL_NUM)
    {
        ProtocolDes[used_index].callback = callback;
        return used_index;
    }
    else
    {
        return -1;
    }
}

/**
 * @brief  register/deregister protocol to decide whether listen on the l2cap connection with given psm or not
 * @param  psm        psm need to pay attention to when l2cap connect
 * @param  listen_id  protocol queue id which will handle the connection with given psm
 * @param  action     1: register, 0: degister
 * @return  none
 * @retval   void
 */
void mpa_Sendl2cProtocolRegister(uint16_t psm, uint16_t listen_id, uint8_t action)
{
    blueAPI_L2cProtocolRegister(psm, listen_id, action);
}

/**
 * @brief  register/deregister security setting on protocol
 * @param  active                1: register, 0: degister
 * @param  outgoing            1: set for outgoing connection, 0: set for incoming connection
 * @param  psm                  protocol service multiplexer
 * @param  server_channel   local server channel for rfcomm, for other protocol set to 0
 * @param  uuid                 upper profile uuid for rfcomm, upper protocol uuid(except rfcomm) for l2cap
 * @param  authentication   authenticaion needed
 * @param  authorization    authorization needed
 * @param  encryption       encrytion needed
 * @param  mitm                 mitm needed
 * @return  none
 * @retval   void
 */
void mpa_SendL2cSecurityRegister(uint8_t active,
                                             uint8_t outgoing,
                                             uint8_t psm,
                                             uint16_t server_channel,
                                             uint16_t uuid,
                                             bool authentication,
                                             bool authorization,
                                             bool encryption,
                                             bool mitm
                                             )
{
    blueAPI_L2cSecurityRegister(active, outgoing, psm, server_channel, uuid, authentication, authorization, encryption, mitm);
}

/**
 * @brief  send authentication request to security manager for rfcomm connection
 * @param  bd                   remote bd address
 * @param  channel            l2cap channel id which is remembered for response
 * @param  dlci                  dlci of rfcomm channel
 * @param  uuid                 upper profile uuid for rfcomm
 * @param  outgoing          outgoing of rfcomm connection
 * @param  active               1 to start authentication, 0 to cancel authentication
 * @return  none
 * @retval   void
 */
void mpa_SendRFCAuthenticationReq(uint8_t *bd,
                                                uint16_t channel,
                                                uint16_t dlci,
                                                uint16_t uuid,
                                                uint8_t outgoing,
                                                uint8_t active
                                                )
{
    blueAPI_RFCAuthenticationReq(bd, channel, dlci, uuid, outgoing, active);
}

/**
 * @brief  send l2cap connect request
 * @param  psm            psm to connect
 * @param  uuid            uuid for upper layer
 * @param  protocol_id  protocol descriptor id for this connection
 * @param  mtu_size     mtu size protocol supported
 * @param  remote_bd  remote bt address
 * @return  none
 * @retval   void
 */
void mpa_Sendl2cConReq(uint16_t psm, uint16_t uuid, uint16_t protocol_id, uint16_t mtu_size, uint8_t* remote_bd)
{
    blueAPI_L2cConReq(psm, uuid, remote_bd, protocol_id, mtu_size);
}

/**
 * @brief  send l2cap disconnect request
 * @param  cid  local cid for the l2cap channel to be disconnected
 * @return  none
 * @retval   void
 */
void mpa_Sendl2cDiscReq(uint16_t cid)
{
    blueAPI_L2cDiscReq(cid);
}

/**
 * @brief  send l2cap disconnect confirm
 * @param  cid  local cid for the l2cap channel to be disconnected
 * @return  none
 * @retval   void
 */
void mpa_Sendl2cDiscConf(uint16_t cid)
{
    blueAPI_L2cDiscConf(cid);
}

/**
 * @brief  send l2cap connect confirm
 * @param  status  0: ok to connect, otherwise fail to connect
 * @param  cid       local cid for the l2cap channel want to connect
 * @return  none
 * @retval   void
 */
void mpa_Sendl2cConConf(int status, uint16_t cid)
{
    blueAPI_L2cConConf(status, cid);
}

/**
 * @brief  send l2cap data request
 * @param  buf     point to data buffer
 * @param  offset offset of exact data in the buffer 
 * @param  cid      local cid for the l2cap channel
 * @param  length data length
 * @return  none
 * @retval   void
 */
void mpa_Sendl2cDataReq(void* buf, uint8_t offset, uint16_t cid, uint16_t length)
{
    blueAPI_L2cDataReq(buf, offset, cid, length);
}

/**
 * @brief  send SCO connect request
 * @param  remote_bd  remote bt address
 * @param  tx_bandwidth  transmit bandwidth
 * @param  rx_bandwidth  receive bandwidth
 * @param  voice_setting  voice setting
 * @param  retrans_effort  retansmit effort
 * @param  packet_type  transmit packet type
 * @return  none
 * @retval   void
 */
void mpa_SendSCOConReq(uint8_t *remote_bd,
                                   uint32_t tx_bandwidth,
                                   uint32_t rx_bandwidth,
                                   uint16_t max_latency,
                                   uint16_t voice_setting,
                                   uint8_t  retrans_effort,
                                   uint16_t packet_type
                                   )
{
    blueAPI_SCOConReq(remote_bd, tx_bandwidth, rx_bandwidth, max_latency, voice_setting, retrans_effort, packet_type);
}

/**
 * @brief  send SCO connect confirm
 * @param  remote_bd  remote bt address
 * @param  tx_bandwidth  transmit bandwidth
 * @param  rx_bandwidth  receive bandwidth
 * @param  voice_setting  voice setting
 * @param  retrans_effort  retansmit effort
 * @param  packet_type  transmit packet type
 * @param  cause    accept sco connect or not
 * @return  none
 * @retval   void
 */
void mpa_SendSCOConConf(uint8_t *remote_bd,
                                    uint32_t tx_bandwidth,
                                    uint32_t rx_bandwidth,
                                    uint16_t max_latency,
                                    uint16_t voice_setting,
                                    uint8_t  retrans_effort,
                                    uint16_t packet_type,
                                    TBlueAPI_Cause cause
                                    )
{
    blueAPI_SCOConConf(remote_bd, tx_bandwidth, rx_bandwidth, max_latency,
                       voice_setting, retrans_effort, packet_type, cause
                       );
}

/**
 * @brief  send SCO disconnect request
 * @param  remote_bd  remote bt address
 * @return  none
 * @retval   void
 */
void mpa_SendSCODiscReq(uint8_t *remote_bd)
{
    blueAPI_SCODiscReq(remote_bd);
}

/**
 * @brief  handle l2cap connect response
 * @param  pcon_rsp l2cap connect response blueAPI message
 * @return  none
 * @retval   void
 */
void mpa_HandleL2cConRsp(PBlueAPI_L2cConRsp pcon_rsp)
{
    pProtocolCallBack protocol_callback;

    if ((pcon_rsp->usQueueID < MAX_PROTOCOL_NUM) && (ProtocolDes[pcon_rsp->usQueueID].psm))
    {
        if (pcon_rsp->status)
        {
            DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "L2cap connect response error: 0x%x", 1, pcon_rsp->status);
        }

        if (ProtocolDes[pcon_rsp->usQueueID].callback != NULL)
        {
            protocol_callback = (pProtocolCallBack)ProtocolDes[pcon_rsp->usQueueID].callback;
            protocol_callback(pcon_rsp, L2CAP_CONNECT_RSP);
        }
    }
}

/**
 * @brief  handle l2cap connect indicate
 * @param  pcon_ind l2cap connect indicate blueAPI message
 * @return  none
 * @retval   void
 */
void mpa_HandleL2cConInd(PBlueAPI_L2cConInd pcon_ind)
{
    pProtocolCallBack protocol_callback;

    if ((pcon_ind->usQueueID < MAX_PROTOCOL_NUM) && (ProtocolDes[pcon_ind->usQueueID].psm))
    {
        if (ProtocolDes[pcon_ind->usQueueID].callback != NULL)
        {
            protocol_callback = (pProtocolCallBack)ProtocolDes[pcon_ind->usQueueID].callback;
            protocol_callback(pcon_ind, L2CAP_CONNECT_IND);
        }
    }
}

/**
 * @brief  handle l2cap connect active indicate
 * @param  pact_ind l2cap connect active indicate blueAPI message
 * @return  none
 * @retval   void
 */
void mpa_HandleL2cConActInd(PBlueAPI_L2cConActInd pact_ind)
{
    pProtocolCallBack protocol_callback;

    if ((pact_ind->usQueueID < MAX_PROTOCOL_NUM) && (ProtocolDes[pact_ind->usQueueID].psm))
    {
        if (ProtocolDes[pact_ind->usQueueID].callback != NULL)
        {
            protocol_callback = (pProtocolCallBack)ProtocolDes[pact_ind->usQueueID].callback;
            protocol_callback(pact_ind, L2CAP_CONNECT_COMPLETE);
        }
    }
}

/**
 * @brief  handle l2cap data indicate
 * @param  pdata_ind l2cap data indicate blueAPI message
 * @return  none
 * @retval   void
 */
void mpa_HandleL2cDataInd(PBlueAPI_L2cDataInd pdata_ind)
{
    pProtocolCallBack protocol_callback;

    if ((pdata_ind->usQueueID < MAX_PROTOCOL_NUM) && (ProtocolDes[pdata_ind->usQueueID].psm))
    {
        if (ProtocolDes[pdata_ind->usQueueID].callback != NULL)
        {
            protocol_callback = (pProtocolCallBack)ProtocolDes[pdata_ind->usQueueID].callback;
            protocol_callback(pdata_ind, L2CAP_DATA_IND);
        }
    }

    if ((pdata_ind->buf) && (pdata_ind->flag & DATA_CB_RELEASE))
    {
        blueAPI_BufferRelease(pdata_ind->buf);
    }
}

/**
 * @brief  handle l2cap disconnect indicate
 * @param  pdisc_ind l2cap disconnect indicate blueAPI message
 * @return  none
 * @retval   void
 */
void mpa_HandleL2cDiscInd(PBlueAPI_L2cDiscInd pdisc_ind)
{
    pProtocolCallBack protocol_callback;

    if ((pdisc_ind->usQueueID < MAX_PROTOCOL_NUM) && (ProtocolDes[pdisc_ind->usQueueID].psm))
    {
        if (ProtocolDes[pdisc_ind->usQueueID].callback != NULL)
        {
            protocol_callback = (pProtocolCallBack)ProtocolDes[pdisc_ind->usQueueID].callback;
            protocol_callback(pdisc_ind, L2CAP_DISC_IND);
        }
    }
}

/**
 * @brief  handle l2cap security register response
 * @param  prsp point to l2cap security register response blueAPI message
 * @return  none
 * @retval   void
 */
void mpa_HandleL2cSecurityRegisterRsp(PBlueAPI_L2cSecurityRegisterRsp prsp)
{
    uint8_t i = 0;

    for (i = 0; i < MAX_PROTOCOL_NUM; i++)
    {
        if ((ProtocolDes[i].psm == prsp->psm) && (ProtocolDes[i].callback != NULL))
        {
            ProtocolDes[i].callback(prsp, PROTOCOL_SECURITY_REGISTER_RSP);
        }
    }
}

/**
 * @brief  handle rfcomm authentication response
 * @param  prsp point to rfcomm authentication response blueAPI message
 * @return  none
 * @retval   void
 */
void mpa_HandleRFCAuthenticationRsp(PBlueAPI_RFCAuthenticationRsp prsp)
{
    if (RFCAuthencationCallBack)
    {
        RFCAuthencationCallBack(prsp);
    }
}

/**
 * @brief  handle authorization request indication
 * @param  pind point to authorization request indication blueAPI message
 * @return  none
 * @retval   void
 */
void mpa_HandleUserAuthorizationReqInd(PBlueAPI_UserAuthorizationReqInd pind)
{
    uint8_t i = 0;

    for (i = 0; i < MAX_PROTOCOL_NUM; i++)
    {
        if ((ProtocolDes[i].psm == pind->psm) && (ProtocolDes[i].callback != NULL))
        {
            ProtocolDes[i].callback(pind, PROTOCOL_AUTHORIZATION_IND);
        }
    }
}

/**
 * @brief  handle l2cap disconnect response
 * @param  pdisc_rsp l2cap disconnect response blueAPI message
 * @return  none
 * @retval   void
 */
void mpa_HandleL2cDiscRsp(PBlueAPI_L2cDiscRsp pdisc_rsp)
{
    pProtocolCallBack protocol_callback;

    if ((pdisc_rsp->usQueueID < MAX_PROTOCOL_NUM) && (ProtocolDes[pdisc_rsp->usQueueID].psm))
    {
        if (ProtocolDes[pdisc_rsp->usQueueID].callback != NULL)
        {
            protocol_callback = (pProtocolCallBack)ProtocolDes[pdisc_rsp->usQueueID].callback;
            protocol_callback(pdisc_rsp, L2CAP_DISC_RSP);
        }
    }
}

/**
 * @brief  handle SCO connect indicate
 * @param  pcon_ind SCO connect indicate blueAPI message
 * @return  none
 * @retval   void
 */
void mpa_HandleSCOConInd(PBlueAPI_SCOConInd pcon_ind)
{
    if (SCOCallBack)
    {
        SCOCallBack(pcon_ind, SCO_CONNECT_IND);
    }
}

/**
 * @brief  handle SCO connect response
 * @param  pcon_ind SCO connect respnose blueAPI message
 * @return  none
 * @retval   void
 */
void mpa_HandleSCOConRsp(PBlueAPI_SCOConRsp pcon_rsp)
{
    if (SCOCallBack)
    {
        SCOCallBack(pcon_rsp, SCO_CONNECT_RSP);
    }
}

/**
 * @brief  handle SCO connect active indicate
 * @param  pact_ind SCO connect active indicate blueAPI message
 * @return  none
 * @retval   void
 */
void mpa_HandleSCOConActInd(PBlueAPI_SCOConActInd pact_ind)
{
    if (SCOCallBack)
    {
        SCOCallBack(pact_ind, SCO_CONNECT_COMPLETE);
    }
}

/**
 * @brief  handle SCO disconnect indicate
 * @param  pcon_ind SCO disconnect indicate blueAPI message
 * @return  none
 * @retval   void
 */
void mpa_HandleSCODiscInd(PBlueAPI_SCODiscInd pdisc_ind)
{
    if (SCOCallBack)
    {
        SCOCallBack(pdisc_ind, SCO_DISC_IND);
    }

}

/**
 * @brief  handle blueAPI upstream message, only legacy l2cap related messages are handled here
 * @param  pmsg point to blueAPI upstream message
 * @return  none
 * @retval   void
 */
void mpa_HandleMessage(PBlueAPI_UsMessage pmsg)
{
    switch (pmsg->Command)
    {
    case blueAPI_EventL2cConRsp:
        mpa_HandleL2cConRsp(&pmsg->p.L2cConRsp);
        break;

    case blueAPI_EventL2cConInd:
        mpa_HandleL2cConInd(&pmsg->p.L2cConInd);
        break;

    case blueAPI_EventL2cConActInd:
        mpa_HandleL2cConActInd(&pmsg->p.L2cConActInd);
        break;

    case blueAPI_EventL2cDataInd:
        mpa_HandleL2cDataInd(&pmsg->p.L2cDataInd);
        break;

    case blueAPI_EventL2cDiscInd:
        mpa_HandleL2cDiscInd(&pmsg->p.L2cDiscInd);
        break;

    case blueAPI_EventL2cDiscRsp:
        mpa_HandleL2cDiscRsp(&pmsg->p.L2cDiscRsp);
        break;

    case blueAPI_EventSCOConInd:
        mpa_HandleSCOConInd(&pmsg->p.SCOConInd);
        break;

    case blueAPI_EventL2cSecurityRegisterRsp:
        mpa_HandleL2cSecurityRegisterRsp(&pmsg->p.L2cSecurityRegisterRsp);
        break;

    case blueAPI_EventRFCAuthenticationRsp:
        mpa_HandleRFCAuthenticationRsp(&pmsg->p.RFCAuthenticaionRsp);
        break;

    case blueAPI_EventUserAuthorizationReqInd:
        mpa_HandleUserAuthorizationReqInd(&pmsg->p.UserAuthorizationReqInd);
        break;

    case blueAPI_EventSCOConRsp:
        mpa_HandleSCOConRsp(&pmsg->p.SCOConRsp);
        break;

    case blueAPI_EventSCOConActInd:
        mpa_HandleSCOConActInd(&pmsg->p.SCOConActInd);
        break;

    case blueAPI_EventSCODiscInd:
        mpa_HandleSCODiscInd(&pmsg->p.SCODiscInd);
        break;

    default:
        break;
    }
}

