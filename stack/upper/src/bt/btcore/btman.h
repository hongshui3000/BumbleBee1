/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       btman.c
* @brief      BTA BT stack definition
* @details   
*
* @author  	gordon
* @date      	2015-06-29
* @version	v0.1
*/

#ifndef __BTA_H
#define __BTA_H

#include <os_queue.h>
#include <os_message.h>
#include <bt_msg.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BLUEFACE_CHECK_LINK_VALID() (pLink == NULL || pLink < &app->bta->links[0] || \
                                    pLink > &app->bta->links[BT_MAX_LINK - 1] || \
                                    (pLink->magic[0] != 'L') || (pLink->magic[1] != 'I'))

/** check data continuity: data format: 8 digits terminated CR/LF for example: "00000001\n00000002\n"etc */
#define T_CHECK_UPSTREAM_DATA     0     /**< check data continuity in upstream */
#define T_CHECK_DOWNSTREAM_DATA   0     /**< check data continuity in downstream */

typedef struct
{
    BOOL            used;           /**< entry is in use */
    BOOL            indicated;      /**< already sent ACT_IND */
    BOOL            security;       /**< this app is real security app */
    BAPPCONTEXT     context;          /**<pBlueAPIdata*/
    struct _TBTA    *bta;           /**< back pointer to containing stack */
} TbtApplication, *PbtApplication;
typedef TbtApplication  * LPbtApplication;
typedef CONST TbtApplication  * LPCbtApplication;

typedef struct
{
    uint8_t                 magic[2];       /**< 'L', 'I' */
    BOOL                    used;
    LPbtApplication         app;            /**< back pointer to containing app */
    uint8_t                 selfIndex;      /**< index of this entry in bta table (back reference) al*/
    BLINKCONTEXT            context;        /**< application context for this link */
    TBdAddr                 bd;             /**< BD of connected party */
    uint16_t                psm;
    enum {
        link_Idle,                          /**< link is idle */
        link_Connecting,                    /**< link is connecting outgoing */
        link_Connected,                     /**< link is connected */
        link_Disconnecting,                 /**< link is disconnecting */
        link_Incoming,                      /**< link is connecting incoming (app layer negotating) */
        link_Incoming_Acp,                  /**< link is connecting incoming (app layer accepted) */
        link_Incoming_Disc                  /**< link is connecting incoming (app layer negotating) and disc_ind */
    } state;                                /**< link status */
    uint16_t                handle;         /**< link identifier of lower layer */
    BOOL                    delayedDisc;    /**< Disconnect requested during connecting */
    BOOL                    activeAuth;     /**< active authentication in progress */
    BOOL                    outgoing;       /**< link is from outgoing call */
    BOOL                    alerting;       /**< in state incoming: link is alerting */
    enum {
        ENCRYPT_IDLE,
        ENCRYPT_REQUIRED,
        ENCRYPT_REQUESTED,
        ENCRYPT_ACTIVATED
    } encryptState;                         /**< encryption state */

#if (T_CHECK_UPSTREAM_DATA)
    uint32_t    DUcrrVal;
    uint32_t    DUprevVal;
#endif
#if (T_CHECK_DOWNSTREAM_DATA)
    uint32_t    DDcrrVal;
    uint32_t    DDprevVal;
#endif

#if (F_BT_L2C_EXT_FEATURE_SUPPORT ||  F_BT_L2C_ENHANCED_FEATURE_SUPPORT)
    BOOL        dataExt;
    uint16_t    totalLength;
#endif
    TCON_IND_P  connIndParams;      /**< link specific paramaters (used to sending of CON_IND too) */
} TbtLink;
typedef TbtLink  * LPbtLink;
typedef CONST TbtLink  * LPCbtLink;

/**BTA Instance data*/
typedef struct _TBTA
{
    uint8_t         btReqQueueID;

    /** Properties of the local adapter */
    TBdAddr         localBd;
    uint16_t        localCause;

    /** descriptors for applications, links etc */
    LPbtApplication activeApp;
    LPbtLink        activeLink;

    /** the security managing application */
    LPbtApplication secApp;

    TbtApplication  app;

    TbtLink         *pLinksDon;
    TbtLink         *pLinksDoff;
} TBTA, * PBTA;
typedef CONST TBTA * PCBTA;
typedef TBTA  * LPBTA;
typedef CONST TBTA  * LPCBTA;

/** link and application management procedures */
LPbtLink blueFaceAllocateLink(void);
void blueFaceDeallocateLink(LPbtLink pLink);
uint8_t blueFacePSM2hQueue(uint16_t psm);
LPbtLink blueFaceFindLinkByHandle(uint16_t handle, uint16_t psm);
LPbtLink blueFaceFindLinkByBD(LPCBYTE bd);
#if (F_BT_SCO)
LPbtLink blueFaceFindLinkByBDandPSM(LPCBYTE bd, uint16_t psm);
#endif
void blueFaceReleaseActiveLinkSemaphore(void);
void blueFaceHandleBtConInd_2(uint16_t psm, PBtConInd  pConInd );

#ifdef __cplusplus
}
#endif

#endif
