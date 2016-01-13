/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       blueapi_link.c
* @brief     link related operation
* @details   
*
* @author   gordon
* @date      2015-06-26
* @version  v0.1
*/

#include <flags.h>
#include <rtl_types.h>
#include <blueface.h>
#include <btglbdef.h>
#include <bt_api.h>
#include <blueapi_types.h>
#include <sdp_code.h>
#include <blueapi_def.h>

/* File Id for COM_InternalEventInfo */
#define BLUE_API_SOURCE_FILE_ID     0x03
#define TRACE_MODULE_ID  MID_BT_HDP
extern const char pBlueAPIDumpPrefix[];

/**
 * @brief  blueapid handle link open indicate
 *
 * @param  pBlueAPIdata
 * @param  pLinkListenInd
 *
 * @return
 *
 */
PBlueAPI_MCL blueAPI_Handle_LinkOpenInd(PBlueAPI_Data pBlueAPIdata, PBlueAPI__BT__LinkOpenInd pLinkOpenInd)
{
    PBlueAPI_LinkDescriptor pLinkContext = pLinkOpenInd->pLinkContext;
    PBlueAPI_MCL            pMCL         = pLinkContext->pMCL;
    PBlueAPI_MDL            pMDL;

    TRACE_DUMP_LINK(pLinkContext, pBlueAPIDumpPrefix);

    switch (pMCL->state)
    {
    case blueAPI_DS_DataConnecting:
        blueAPI_MCLSetState(pMCL, blueAPI_DS_DataConnected);

        switch (pMCL->DS_CommandInProgress)
        {
        case blueAPI_EventConnectMDLReq:
            pMDL = pLinkContext->pMDL;
            blueAPI_Send_ConnectMDLRsp(pMCL->DS_CommandMsg,
                                       pMCL,
                                       pMDL->remoteBd,
                                       pMDL->remoteBdType,
                                       pMDL->local_MDL_ID,
                                       blueAPI_CauseSuccess
                                       );

            blueAPI_Send_ConnectMDLInfo(pLinkContext);
            pMDL->pendingCause = blueAPI_CauseSuccess;

            if (pLinkContext->pMCL->bdType != blueAPI_RemoteBDTypeClassic)
            {
                blueAPI_Send_LEConnectionParameterInfo(pLinkContext);
            }
            break;

        case blueAPI_EventSDPDiscoveryReq:
            blueAPI_MDLSubFlags(pLinkContext->pMDL, BLUE_API_MDL_FLAG_NOT_ACTIVATED);
            pBlueAPIdata->pSDPLinkContext = pLinkContext;

            pBlueAPIdata->SM_SDP_ServiceUUID = UUID_PNPINFORMATION;
            if (!pMCL->DS_CommandMsg->p.SDPDiscoveryReq.remote_DID_Discovery)
            {
                pBlueAPIdata->SM_SDP_ServiceUUID = (uint16_t)pMCL->DS_CommandMsg->p.SDPDiscoveryReq.remote_MDEP_DataType;
            }
            blueAPI_Send_BT_SDP_SEARCH_REQ(pBlueAPIdata, pBlueAPIdata->SM_SDP_ServiceUUID, 0);
            break;

        case blueAPI_EventGATTSDPDiscoveryReq:
            blueAPI_MDLSubFlags(pLinkContext->pMDL, BLUE_API_MDL_FLAG_NOT_ACTIVATED);
            pBlueAPIdata->pSDPLinkContext = pLinkContext;

            if (pMCL->DS_CommandMsg->p.GATTSDPDiscoveryReq.remote_DID_Discovery)
            {
                pBlueAPIdata->SM_SDP_ServiceUUID = UUID_PNPINFORMATION;
                blueAPI_Send_BT_SDP_SEARCH_REQ(pBlueAPIdata, pBlueAPIdata->SM_SDP_ServiceUUID, 0);
            }
            else
            {
                pBlueAPIdata->SM_SDP_ServiceUUID = UUID_ATT;
                blueAPI_Send_BT_SDP_SEARCH_REQ(pBlueAPIdata,
                                               pBlueAPIdata->SM_SDP_ServiceUUID,
                                               pMCL->DS_CommandMsg->p.GATTSDPDiscoveryReq.remote_GATT_UUID
                                               );
            }
            break;

        case blueAPI_EventAuthReq:
            blueAPI_MDLSubFlags(pLinkContext->pMDL, BLUE_API_MDL_FLAG_NOT_ACTIVATED);
            pBlueAPIdata->pSDPLinkContext = pLinkContext;
            btApiBT_AUTH_REQ(pLinkContext->handle);
            break;

        default:
#if SEND_INT_EVENT_INFO            
            blueAPI_Send_InternalEventInfo(blueAPI_InternalEventInvalidRemoteEvent,
                                           BLUE_API_GENERATE_EVENT_ID,
                                           blueAPI_CauseInvalidState
                                           );
#endif
            break;
        }
        break;

    case blueAPI_DS_DataListening:
        blueAPI_MCLSetState(pMCL,blueAPI_DS_DataConnected);
        pMDL = pLinkContext->pMDL;

        if(pLinkOpenInd->cause == blueAPI_CauseSuccess)
        {
            blueAPI_Send_ConnectMDLInfo(pLinkContext);

            if (pLinkContext->pMCL->bdType != blueAPI_RemoteBDTypeClassic)
            {
                blueAPI_Send_LEConnectionParameterInfo(pLinkContext);
            }
            pMDL->pendingCause = blueAPI_CauseSuccess;
        }
        else /* link is open, but with invalid L2CAP configuration */
        {
            BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE, "blueAPI: enforce instant disconnect");
            pMDL->pendingCause = blueAPI_CauseInvalidParameter;
            blueAPI_ChannelDisconnect(pBlueAPIdata, pLinkContext, FALSE);
        }
        break;

    default:
#if SEND_INT_EVENT_INFO        
        blueAPI_Send_InternalEventInfo(blueAPI_InternalEventInvalidRemoteEvent,
                                       BLUE_API_GENERATE_EVENT_ID,
                                       blueAPI_CauseInvalidState
                                       );
#endif
        break;
    }

    return pMCL;
}

/**
 * @brief  data link close indicate
 *
 * @param  pBlueAPIdata
 * @param  pLinkCloseInd
 *
 * @return
 *
 */
BOOL blueAPI_Handle_LinkCloseIndData(PBlueAPI_Data pBlueAPIdata, PBlueAPI__BT__LinkCloseInd pLinkCloseInd)
{
    PBlueAPI_LinkDescriptor pLinkContext   = pLinkCloseInd->pLinkContext;
    PBlueAPI_MCL            pMCL           = pLinkContext->pMCL;
    BOOL                    releaseChannel = TRUE;
    TBlueAPI_Cause          indCause       = blueAPI_CauseConnectionDisconnect;

    if (pLinkCloseInd->cause != blueAPI_CauseSuccess)
    {
        indCause = pLinkCloseInd->cause;
    }

    BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE, "blueAPI: data channel disconnected");

    switch(pMCL->state)
    {
    case blueAPI_DS_DataConnecting:
        /* we initiated a connect_req for a data channel */
        if ((pLinkContext->linkState == blueAPI_Connecting) ||
             /* or a LE connection timeout */
            ((pMCL->mclType == blueAPI_MCLType_GATT) &&
             (pMCL->bdType != blueAPI_RemoteBDTypeClassic) &&
             (pLinkContext->linkState == blueAPI_Disconnecting))
            )
        {
            /* this connection attempt failed */
            switch (pMCL->DS_CommandInProgress)
            {
            case blueAPI_EventConnectMDLReq:
                if (pMCL->mclType == blueAPI_MCLType_GATT)
                {
                    if (pMCL->bdType != blueAPI_RemoteBDTypeClassic)
                    {
                        blueAPI_TimerStop(pMCL, blueAPI_TimerID_LEScanTO);
                    }
                    blueAPI_Send_ConnectMDLRsp(pMCL->DS_CommandMsg,
                                               pMCL,
                                               pLinkContext->pMDL->remoteBd,
                                               pLinkContext->pMDL->remoteBdType,
                                               pLinkContext->pMDL->local_MDL_ID,
                                               indCause
                                               );
                    /* we can delete this MDL since it was never active*/
                    blueAPI_Send_DeleteMDLInfo( pLinkContext->pMDL);
                    blueAPI_MDLRelease(pLinkContext->pMDL);
                }
                break;

            case blueAPI_EventAuthReq:
            case blueAPI_EventSDPDiscoveryReq:
            case blueAPI_EventGATTSDPDiscoveryReq:
                blueAPI_SDP_Disconnect(pBlueAPIdata, pLinkContext, indCause);
                releaseChannel = TRUE;
                break;

            default:
#if SEND_INT_EVENT_INFO                  
                blueAPI_Send_InternalEventInfo(blueAPI_InternalEventInvalidRemoteEvent,
                                               BLUE_API_GENERATE_EVENT_ID,
                                               blueAPI_CauseInvalidState
                                               );
#endif
                break;
            }
            break;
        }
        else
        {
            /* ==> pLinkContext->linkState!=blueAPI_Connecting   */
            /* ==> it can't be this channel so there must be at least one  more data channel connected  */
            /* ==> no break */
        }

    case blueAPI_DS_DataListening:
        if ((pMCL->mclType == blueAPI_MCLType_GATT) &&
            (pLinkContext->linkState == blueAPI_Connecting)
            )
        {
            BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE,
                                   "blueAPI: incoming RFCOMM/GATT channel in progress disconnected"
                                   );

            blueAPI_Send_DeleteMDLInfo(pLinkContext->pMDL);
            blueAPI_MDLRelease(pLinkContext->pMDL);
            blueAPI_MCLSetState(pMCL, blueAPI_DS_Idle);
            break;
        }
        else
        {
            /* we initiated a listen_req for an other data channel  */
            /* ==> it can't be this channel so there must be at least one more data channel connected */
            /* no break */
        }

    case blueAPI_DS_DataConnected:
        releaseChannel = FALSE; /* needed for BT_DISC_RESP later */

        /* at least one data channel is connected */
        if(!blueAPI_MCLInTransition(pMCL))
        {
            /* we are in a stable state => start action */
            pLinkContext->pMDL->pendingCause = indCause;
            blueAPI_Send_DisconnectMDLInd(pMCL, pLinkContext->pMDL, indCause);
        }
        else if(pMCL->US_CommandInProgress == blueAPI_EventDisconnectMDLInd)
        {
            PBlueAPI_DisconnectMDLInd pProgress = (PBlueAPI_DisconnectMDLInd)&pMCL->US_CommandData;

            if(pProgress->local_MDL_ID != pLinkContext->pMDL->local_MDL_ID)
            {
                blueAPI_MDLAddFlags(pLinkContext->pMDL, BLUE_API_MDL_FLAG_DISCONNECT_IND);
                pLinkContext->pMDL->pendingCause = indCause;
            }
            else
            {
                BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE,
                                       "blueAPI: COM_DisconnectMDLInd Local_MDL_ID[%d] already pending",
                                       pProgress->local_MDL_ID
                                       );
            }
        }
        else if (pMCL->mclType == blueAPI_MCLType_SDP)
        {
            blueAPI_SDP_Disconnect(pBlueAPIdata, pBlueAPIdata->pSDPLinkContext, indCause);
            releaseChannel = TRUE;
        }
        else
        {
            /* mark connected MDL to become disconnected later */
            blueAPI_MDLAddFlags(pLinkContext->pMDL,BLUE_API_MDL_FLAG_DISCONNECT_IND);
            pLinkContext->pMDL->pendingCause = indCause;
        }

        switch (pMCL->mclType)
        {
        case blueAPI_MCLType_GATT:
            /* set fake state for RFCOMM, GATT, L2CAP connections */
            blueAPI_MCLSetState(pMCL, blueAPI_DS_DataDisconnecting);
            break;

        default:
            if (pMCL->state == blueAPI_DS_DataConnected)
            {
                blueAPI_MCLSetState(pMCL, blueAPI_DS_Idle);
            }
            break;
        }
        break;

    case blueAPI_DS_DataDisconnecting:
    {
        /* we initiated a disconnect_req for a data channel */
        PBlueAPI_MDL pMDL;

        if (pMCL->DS_CommandInProgress==blueAPI_EventDisconnectMDLReq)
        {
            /* local disconnectMDLReq without delete MDL                    */
            PBlueAPI_DisconnectMDLReq pProgress = &pMCL->DS_CommandMsg->p.DisconnectMDLReq;
            pMDL = blueAPI_MDLFindByLocal_MDL_ID(pBlueAPIdata,pProgress->local_MDL_ID);

            if (!pMDL)
            {
                BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE,
                                       "!!blueAPI: unhandled 9[%d]!!",
                                       pProgress->local_MDL_ID
                                       );
#if SEND_INT_EVENT_INFO
                blueAPI_Send_InternalEventInfo(blueAPI_InternalEventInvalidRemoteEvent,
                                               BLUE_API_GENERATE_EVENT_ID,
                                               blueAPI_CauseInvalidParameter
                                               );
#endif
            }
            else if (pMDL == pLinkContext->pMDL)
            {
                /* this is our link => continue */

                blueAPI_Send_DisconnectMDLRsp(pMCL->DS_CommandMsg,
                                              pMCL,
                                              pMDL->local_MDL_ID,
                                              blueAPI_CauseSuccess
                                              );

                if (pMCL->mclType == blueAPI_MCLType_GATT)
                {
                    /* GATT have no reconnect, delete the MDL now */
                    blueAPI_Send_DeleteMDLInfo(pMDL);
                    blueAPI_MDLRelease(pMDL);
                }

                blueAPI_MCLSetState(pMCL, blueAPI_DS_Idle);
            }
            else
            {
                /* an other link is disconnected while we are disconnecting  */
                blueAPI_MDLAddFlags(pLinkContext->pMDL, BLUE_API_MDL_FLAG_DISCONNECT_IND);
                pLinkContext->pMDL->pendingCause = indCause;
                releaseChannel = FALSE; /* needed for BT_DISC_RESP later */
            }
        }
        else if ((pMCL->DS_CommandInProgress == blueAPI_EventAuthReq)
                || (pMCL->DS_CommandInProgress == blueAPI_EventSDPDiscoveryReq)
                || (pMCL->DS_CommandInProgress == blueAPI_EventGATTSDPDiscoveryReq)
                )
        {
            blueAPI_SDP_Disconnect(pBlueAPIdata, pBlueAPIdata->pSDPLinkContext, pBlueAPIdata->SM_SDP_DS_Cause);
            releaseChannel = TRUE;
        }
        else
        {
            pMDL=pLinkContext->pMDL;

            if(pMDL&&pMDL->pendingCause == blueAPI_CauseInvalidParameter)
            {
                /* a reconnect attept failed due to changed L2CAP config     */
                BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE,
                                       "blueAPI: incomming reconnect Local_MDL[%d] failed",
                                       pLinkContext->pMDL->local_MDL_ID
                                       );
                blueAPI_Send_DeleteMDLInfo(pMDL);
                blueAPI_MDLRelease(pMDL);
                blueAPI_MCLSetState(pMCL, blueAPI_DS_Idle);
            }
            else
            {
#if SEND_INT_EVENT_INFO              
                blueAPI_Send_InternalEventInfo(blueAPI_InternalEventInvalidRemoteEvent,
                                               BLUE_API_GENERATE_EVENT_ID,
                                               blueAPI_CauseInvalidParameter
                                               );
#endif
            }
        }
        break;
    }

    default:
#if SEND_INT_EVENT_INFO          
        blueAPI_Send_InternalEventInfo(blueAPI_InternalEventInvalidRemoteEvent,
                                       BLUE_API_GENERATE_EVENT_ID,
                                       blueAPI_CauseInvalidState
                                       );
#endif
        break;
    }

    return releaseChannel;
}

/**
 * @brief  blueapid indicate link closed
 *
 * @param  pBlueAPIdata
 * @param  pLinkCloseInd
 *
 * @return
 *
 */
PBlueAPI_MCL blueAPI_Handle_LinkCloseInd(PBlueAPI_Data pBlueAPIdata, PBlueAPI__BT__LinkCloseInd pLinkCloseInd)
{
    PBlueAPI_LinkDescriptor pLinkContext = pLinkCloseInd->pLinkContext;
    PBlueAPI_MCL pMCL                    = pLinkContext->pMCL;
    BOOL releaseChannel                  = TRUE;

    releaseChannel = blueAPI_Handle_LinkCloseIndData(pBlueAPIdata, pLinkCloseInd);

    if (releaseChannel)
    {
        btApiBT_DISC_RESP(pLinkContext->handle);
        blueAPI_ChannelRelease(pBlueAPIdata, pLinkContext);
    }
    return pMCL;
}

/**
 * @brief  blueapid handle bt event
 *
 * @param  pBlueAPIdata
 * @param  BT_event: bt event num
 * @param  pBTEvent: pointer to bt event buf
 *
 * @return
 *
 */
void blueAPI_Handle_BT_Event(PBlueAPI_Data pBlueAPIdata, TBlueAPI__BT_Event BT_event, PBlueAPI__BT_Event pBTEvent)
{
    PBlueAPI_MCL pMCL = NULL;

    BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE, "blueAPI: enter BT event (%d)", BT_event);

    pBlueAPIdata->SM_Active++;

    switch (BT_event)
    {
    case blueAPI_Event_BT__LinkCloseInd:
        pMCL = blueAPI_Handle_LinkCloseInd(pBlueAPIdata, (PBlueAPI__BT__LinkCloseInd)pBTEvent);
        break;

    case blueAPI_Event_BT__LinkOpenInd:
        pMCL = blueAPI_Handle_LinkOpenInd(pBlueAPIdata, (PBlueAPI__BT__LinkOpenInd)pBTEvent);
        break;

    default:
#if SEND_INT_EVENT_INFO      
        blueAPI_Send_InternalEventInfo(blueAPI_InternalEventInvalidRemoteEvent,
                                     BLUE_API_GENERATE_EVENT_ID,
                                     blueAPI_CauseInvalidState
                                     );
#endif
        break;
    }

    pBlueAPIdata->SM_Active--;

    if (pMCL)
    {
        blueAPI_CheckMCLDisconnect(pBlueAPIdata, pMCL);
    }

    if ((!pBlueAPIdata->SM_Active) &&
        ((pBlueAPIdata->state & BLUE_API_STATE_READY) == BLUE_API_STATE_READY)
        )
    {
        TRACE_DUMP_SYSTEM(pBlueAPIdata); /* exit Handle_BT_Event */
    }

    BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE, "blueAPI: exit BT event (%d)", BT_event);
}
