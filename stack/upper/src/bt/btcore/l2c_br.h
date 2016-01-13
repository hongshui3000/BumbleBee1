/**
*****************************************************************
*	Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************
* @file       l2c_br.h
* @brief     Bluetooth L2CAP Layer (Low Energy)
* @details   
*
* @author   	gordon
* @date      	2015-07-09
* @version	v0.1
*/

#include <flags.h>
#include <bt_msg.h>

#if !defined(__L2C_BR_H)
#define      __L2C_BR_H

#ifdef __cplusplus
extern "C" {
#endif

#define L2CAP_CONFIG_PARA_MTU                   0x01
#define L2CAP_CONFIG_PARA_FLUSH_TIMEOUT         0x02
#define L2CAP_CONFIG_PARA_QOS                   0x03
#define L2CAP_CONFIG_PARA_FCandR                0x04
#define L2CAP_CONFIG_PARA_FCS                   0x05

/** L2CAP to L2CAP SIGNALING parameter field definition */
typedef struct                  /**< parameter set of L2CAP Connect Request */
{
	uint16_t psm;                   /**< Protocol/Service Multiplexer */
	uint16_t scid;                  /**< Source CID */
} T_L2CAP_ConReq_para, *P_L2CAP_ConReq_para;

typedef struct                  /**< parameter set of L2CAP Config Request */
{
	uint16_t dcid;                  /**< Destination CID */
	uint16_t flags;                 /**< Configure Flags */
} T_L2CAP_Config_para, *P_L2CAP_Config_para;


typedef struct                  /**< parameter set of L2CAP Connect Response */
{
	uint16_t dcid;                  /**< Destination CID */
	uint16_t scid;                  /**< Source CID */
	uint16_t result;                /**< result of the con resp */
	uint16_t status;                /**< additional error codes */
} T_L2CAP_ConResp_para, *P_L2CAP_ConResp_para;

typedef struct                  /**< parameter set of L2CAP Configure Response */
{
    uint16_t scid;                  /**< Source CID */
    uint16_t flags;                 /**< Configure Flags */
    uint16_t result;                /**< result of the con resp */
} T_L2CAP_ConfigResp_para, *P_L2CAP_ConfigResp_para;

void l2cStartCONFIGTimeout(P_L2CAP_CHANNEL pChan, uint16_t ticks);
void l2cStopCONFIGTimeout(P_L2CAP_CHANNEL pChan);

BOOL l2cCheckConfPara(LP_L2C_Conf_para pConfPara);
void l2cCheckMode(P_L2CAP_CHANNEL pChan);
void l2cSendLHciWriteAutomaticFlushTimeout(P_ACLPOOLDESC pHciDesc);

void l2cSendL2CAP_CONFIGURE_REQUEST(P_L2CAP_CHANNEL pChan);
void l2cSendL2CAP_CONFIGURE_RESPONSE(P_L2CAP_CHANNEL pChan, uint16_t flags, uint16_t result, PCBYTE options, uint16_t optLen);
void l2cSendL2CAP_CONNECTION_RESPONSE(P_L2CAP_CHANNEL pChan, P_ACLPOOLDESC pHciDesc, uint16_t lcid, uint16_t rcid,
                                    uint16_t result, uint16_t status);
void l2cSendL2CAP_INFORMATION_REQUEST(P_L2CAP_CHANNEL pChan, uint16_t infoType);
void l2cSendL2CAP_ECHO_REQUEST(P_L2CAP_CHANNEL pChan);

void l2cHandleL2CAP_CONNECTION_REQUEST(P_ACLPOOLDESC hciDesc, T_L2CAP_Command *l2cap_cmd, uint8_t * pPar);
void l2cHandleL2CAP_CONNECTION_RESPONSE(T_L2CAP_Command *l2cap_cmd, uint8_t * pPar);
void l2cHandleL2CAP_CONFIGURE_REQUEST(P_ACLPOOLDESC hciDesc, T_L2CAP_Command *l2cap_cmd, uint8_t * pPar);
void l2cHandleL2CAP_CONFIGURE_RESPONSE(T_L2CAP_Command *l2cap_cmd, uint8_t * pPar);
void l2cHandleL2CAP_ECHO_REQUEST(P_ACLPOOLDESC hciDesc, T_L2CAP_Command *l2cap_cmd);
void l2cHandleL2CAP_INFORMATION_REQUEST(P_ACLPOOLDESC hciDesc, T_L2CAP_Command *l2cap_cmd, uint8_t * pPar);
void l2cHandleL2CAP_INFORMATION_RESPONSE(P_L2CAP_CHANNEL pChan, P_ACLPOOLDESC hciDesc, T_L2CAP_Command *l2cap_cmd, uint8_t * pPar);

void l2cTearDownChan(P_L2CAP_CHANNEL pChan);
void l2cRemoveHCI(P_ACLPOOLDESC pHciDesc, uint8_t CloseId);
#ifdef __cplusplus
}

#endif
#endif

