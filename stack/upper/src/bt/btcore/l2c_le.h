/**
*****************************************************************
*	Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************
* @file       l2c_le.h
* @brief     Bluetooth L2CAP Layer (Low Energy)
* @details   
*
* @author   	gordon
* @date      	2015-07-09
* @version	v0.1
*/

#include <flags.h>
#include <bt_msg.h>
#if (F_BT_LOW_ENERGY)

#if !defined(__L2C_LE_H)
#define      __L2C_LE_H

#ifdef __cplusplus
extern "C" {
#endif

#define LE_FIXED_CHANNEL_DESCRIPTOR   0xFFFFFFFF  /**< dummy pointer for fixed channels */

#if F_BT_LE_BT41_SUPPORT

#define LE_PSM_MAX 0xFF
#define L2CAP_US_WRITE_OFFSET    24

typedef struct
{
    uint16_t le_psm;
    uint16_t scid;
    uint16_t mtu;
    uint16_t mps;
    uint16_t initialCredits;
} T_L2CAP_LE_ConReq_para, *P_L2CAP_LE_ConReq_para;

typedef struct                  /* parameter set of L2CAP LE Credit Based Connection Response */
{
    uint16_t dcid;
    uint16_t mtu;
    uint16_t mps;
    uint16_t initialCredits;
    uint16_t result;
} T_L2CAP_LE_ConResp_para, *P_L2CAP_LE_ConResp_para;

typedef struct                  /* parameter set of L2CAP LE Flow Control Credit */
{
    uint16_t cid;
    uint16_t credits;
} T_L2CAP_LE_FlowCtrol_para, *P_L2CAP_LE_FlowCtrol_para;

void l2cSendL2CAP_LE_CREDIT_BASED_CONNECTION_RESPONSE(P_ACLPOOLDESC hciDesc, P_L2CAP_CHANNEL pChan, uint16_t dcid, uint16_t mtu, uint16_t mps,
        uint16_t initialCredits, uint16_t result);

void l2cSend_CreateLEDataChannelInd(uint16_t channel, uint16_t handle);
void l2cSend_LEDataChannelParameterInfo(uint16_t handle, uint16_t channel, uint16_t mtu, uint16_t mps, uint16_t credits);
void l2cSend_CreateLEDataChannelRsp(uint16_t channel, uint16_t handle, uint16_t cause);
void l2cSend_DisconnectLEDataChannelRsp(uint16_t channel, uint16_t handle, uint16_t cause);
void l2cSend_DisconnectLEDataChannelInd (uint16_t channel, uint16_t handle, uint16_t cause);

void l2cHandleL2CAP_LE_CREDIT_BASED_CONNECTION_REQUEST(P_ACLPOOLDESC hciDesc, T_L2CAP_Command *l2cap_cmd, uint8_t * pPar);
void l2cHandleL2CAP_LE_CREDIT_BASED_CONNECTION_RESPONSE(P_ACLPOOLDESC hciDesc, T_L2CAP_Command *l2cap_cmd, uint8_t * pPar);
void l2cHandleL2CAP_LE_FLOW_CONTROL_CREDIT(P_ACLPOOLDESC hciDesc, T_L2CAP_Command *l2cap_cmd, uint8_t * pPar);
void l2cCloseLEChannel(P_L2CAP_CHANNEL pChan, uint16_t cause);
void l2cHandleUpstreamLEFrame(P_L2CAP_CHANNEL pChan, MESSAGE_P pMsg);
#endif

BOOL l2cLESearchLCid(uint16_t LCid);

void l2cSendL2CAP_CONNECTION_PARAMETER_UPDATE_RESPONSE(P_L2CAP_CHANNEL pChan, P_ACLPOOLDESC pHciDesc, uint16_t result);
void l2cHandleL2CAP_CONNECTION_PARAMETER_UPDATE_REQUEST(P_ACLPOOLDESC pHciDesc, uint8_t * pPar);
void l2cHandleL2CAP_CONNECTION_PARAMETER_UPDATE_RESPONSE(P_ACLPOOLDESC pHciDesc, uint8_t * pPar);

void l2cLESendDataInd(P_ACLPOOLDESC pHciDesc, uint16_t LCid, MESSAGE_P pMsg);
void l2cLEHandleTimeout(P_ACLPOOLDESC pHciDesc);

void l2cLEStartRTXTimeout(P_ACLPOOLDESC pHciDesc, uint16_t secs);
void l2cLEStopRTXTimeout(P_ACLPOOLDESC pHciDesc);

#ifdef __cplusplus
}
#endif

#endif
#endif

