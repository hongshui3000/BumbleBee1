enum { __FILE_NUM__ = 0 };

#include <blueapi_types.h>
#include <blueapi.h>
#include "btltp.h"
#include <ltplib.h>
#include "aci_service_handle.h"
#include "trace.h"
#if BREDR_SUPPORT
#include "btltp_br.h"
#endif

#define LTP_SOURCE_FILE_ID 0x81


STATIC void LTPHandle_CreateMDLInd(PBTLtp pBTLtp, PBlueAPI_CreateMDLInd pCOM_CreateMDLInd)
{
    uint8_t pOpt[2];
    pOpt[0] = pCOM_CreateMDLInd->remote_BD_type;
    pOpt[1] = blueAPI_LinkConfigGATT;

    /* Allocate Context for this MDL */
    BTLTPAllocateMDLContext(pBTLtp,
                            (uint8_t)pCOM_CreateMDLInd->local_MDL_ID,
                            0x01,  /* for upper stack changes  */
                            blueAPI_LinkConfigGATT
                           );

    LTPLibSendCreateMDLInd(&pBTLtp->LTPLib,
                           (BTLTP_DEFAULT_COPMSK |
                            LTP_CREATE_MDL_IND_OPT_MASK_BD_TYPE|
                            LTP_CREATE_MDL_IND_OPT_MASK_LINK_TYPE
                           ),
                           pOpt,
                           pCOM_CreateMDLInd->remote_BD,
                           (uint8_t)pCOM_CreateMDLInd->local_MDL_ID
                          );
}

STATIC void LTPHandle_ConnectMDLInfo(PBTLtp pBTLtp, PBlueAPI_ConnectMDLInfo pCOM_ConnectMDLInfo)
{
    uint8_t              pOpt[3];
    PBTLtpMDLContext  pMDLContext;
    uint8_t copmsk = BTLTP_DEFAULT_COPMSK; 

    if (pCOM_ConnectMDLInfo->maxTPDUSize  > BT_MAX_MTU_SIZE)
    {
        DBG_BUFFER(MODULE_LTP, LEVEL_ERROR,  "!!LTP: ConnectMDLInfo TPDU size not supported (%d>%d)", 2, \
                   pCOM_ConnectMDLInfo->maxTPDUSize, BT_MAX_MTU_SIZE);

        LTPLibSendInternalEventInfo(&pBTLtp->LTPLib, 0, NULL, LTP_CAUSE_RESOURCE_ERROR, LTP_INTERNAL_EVENT_INVALID_DATA_RECEIVED, LTP_GENERATE_EVENT_ID);
    }
    
    /* Update Context for this MDL */
    pMDLContext = BTLTPFindMDLContext(pBTLtp, (uint8_t)pCOM_ConnectMDLInfo->local_MDL_ID);
    if (pMDLContext != NULL)
    {
        pMDLContext->flags             |= LTP_MDL_CONNECTED;
#if BREDR_SUPPORT
        pMDLContext->maxUsCredits       = pCOM_ConnectMDLInfo->maxTPDUdsCredits;
#endif
        pMDLContext->collectedUsCredits = 0;
        pMDLContext->pendingDataConfs   = 0;
        pMDLContext->dsDataOffset = pCOM_ConnectMDLInfo->dsDataOffset;
        pMDLContext->dsPoolID = pCOM_ConnectMDLInfo->dsPoolID;
    }
    
#if BREDR_SUPPORT
   pOpt[0] = (uint8_t)pCOM_ConnectMDLInfo->linkConfigType;
   pOpt[1] = pCOM_ConnectMDLInfo->maxTPDUdsCredits;
   copmsk = BTLTP_DEFAULT_COPMSK |LTP_CONNECT_MDL_INFO_OPT_MASK_LINK_TYPE |
                            LTP_CONNECT_MDL_INFO_OPT_MASK_MAX_TPDU_US_CREDITS |
                            LTP_CONNECT_MDL_INFO_OPT_MASK_MAX_TPDU_DS_CREDITS;
#endif

    LTPLibSendConnectMDLInfo(&pBTLtp->LTPLib,
                             copmsk,
                             pOpt,
                             (uint8_t)pCOM_ConnectMDLInfo->local_MDL_ID,
                             pCOM_ConnectMDLInfo->maxTPDUSize,
                             pCOM_ConnectMDLInfo->maxTPDUSize
                            );
}

STATIC void LTPHandle_DisconnectMDLInd(PBTLtp pBTLtp, PBlueAPI_DisconnectMDLInd pCOM_DisconnectMDLInd)
{
    PBTLtpMDLContext pMDLContext;

    /* Update Context for this MDL */
    pMDLContext = BTLTPFindMDLContext(pBTLtp, (uint8_t)pCOM_DisconnectMDLInd->local_MDL_ID);
    if (pMDLContext != NULL)
    {
        pMDLContext->flags &= ~(LTP_MDL_CONNECTED);
    }

    LTPLibSendDisconnectMDLInd(&pBTLtp->LTPLib,
                               BTLTP_DEFAULT_COPMSK,
                               NULL,
                               BTLTPConvertCOMtoLTPcause(pCOM_DisconnectMDLInd->cause),
                               (uint8_t)pCOM_DisconnectMDLInd->local_MDL_ID
                              );
}

STATIC void LTPHandle_ConnectMDLRsp(PBTLtp pBTLtp, PBlueAPI_ConnectMDLRsp pCOM_ConnectMDLRsp)
{
   PBTLtpMDLContext pContext;

   if (pCOM_ConnectMDLRsp->local_MDL_ID != 0x00)
   {
     pContext = BTLTPFindMDLContext(pBTLtp, (uint8_t)pCOM_ConnectMDLRsp->local_MDL_ID);
   }
#if 0
   else
   {
     pContext = BTLTPFindMDLContextByMDEPID(pBTLtp, pCOM_ConnectMDLRsp->local_MDEP_ID);
   }
#endif
   if ((pContext != NULL) && (pContext->flags & LTP_MDL_GATT))
   {
     LTPLibSendConnectGATTMDLRsp(&pBTLtp->LTPLib,
                                 BTLTP_DEFAULT_COPMSK,
                                 NULL,
                                 BTLTPConvertCOMtoLTPcause(pCOM_ConnectMDLRsp->cause),
                                 pCOM_ConnectMDLRsp->remote_BD,
                                 pCOM_ConnectMDLRsp->remote_BD_type,
                                 (uint8_t)pCOM_ConnectMDLRsp->local_MDL_ID,
                                 1
                                 );

     /* context was allocated in ConnectMDLReq and no CreateMDLInd was sent
      * -> no DeleteMDLInfo -> free context now */
     if (pCOM_ConnectMDLRsp->local_MDL_ID == 0x00)
     {
       pContext->flags = LTP_MDL_UNUSED;
     }
   }
#if BREDR_SUPPORT
   else
  {

     uint8_t pOpt[1];

     pOpt[0] = (uint8_t)pCOM_ConnectMDLRsp->local_MDL_ID;
     //pOpt[1] = pCOM_ConnectMDLRsp->local_MDEP_ID;

     LTPLibSendConnectMDLRsp(&pBTLtp->LTPLib,
                             BTLTP_DEFAULT_COPMSK |                    \
                             LTP_CONNECT_MDL_RSP_OPT_MASK_LOC_MDL_ID | \
                             LTP_CONNECT_MDL_RSP_OPT_MASK_LOC_MDEP_ID,
                             pOpt,
                             BTLTPConvertCOMtoLTPcause(pCOM_ConnectMDLRsp->cause),
                             pCOM_ConnectMDLRsp->remote_BD
                            );
   }
#endif
}

STATIC void LTPHandle_DisconnectMDLRsp(PBTLtp pBTLtp, PBlueAPI_DisconnectMDLRsp pCOM_DisconnectMDLRsp)
{
    LTPLibSendDisconnectMDLRsp(&pBTLtp->LTPLib,
                               BTLTP_DEFAULT_COPMSK,
                               NULL,
                               BTLTPConvertCOMtoLTPcause(pCOM_DisconnectMDLRsp->cause),
                               (uint8_t)pCOM_DisconnectMDLRsp->local_MDL_ID
                              );
}

STATIC void LTPHandle_DeleteMDLInfo(PBTLtp pBTLtp, PBlueAPI_DeleteMDLInfo pCOM_DeleteMDLInfo)
{
    PBTLtpMDLContext pMDLContext;

    /* remove Credit Context for this MDL */
    pMDLContext = BTLTPFindMDLContext(pBTLtp, (uint8_t)pCOM_DeleteMDLInfo->local_MDL_ID);
    if (pMDLContext != NULL)
    {
        pMDLContext->flags = LTP_MDL_UNUSED;
    }

    LTPLibSendDeleteMDLInfo(&pBTLtp->LTPLib,
                            BTLTP_DEFAULT_COPMSK,
                            NULL,
                            (uint8_t)pCOM_DeleteMDLInfo->local_MDL_ID
                           );
}

STATIC void LTPHandle_AuthResultRequestInd(PBTLtp pBTLtp, PBlueAPI_AuthResultRequestInd pCOM_AuthResultRequestInd)
{
  if (pCOM_AuthResultRequestInd->remote_BD_Type != blueAPI_RemoteBDTypeClassic)
  {
    uint8_t pOpt[2];
    uint8_t pos = 0;

    NETSHORT2CHAR(&pOpt[pos], pCOM_AuthResultRequestInd->restartHandle);
    pos += 2;

    LTPLibSendAuthResultRequestExtInd(&pBTLtp->LTPLib,
                                      (BTLTP_DEFAULT_COPMSK |
                                       LTP_AUTH_RESULT_REQUEST_EXT_IND_OPT_MASK_RESTART_HANDLE
                                      ),
                                      pOpt,
                                      pCOM_AuthResultRequestInd->remote_BD,
                                      pCOM_AuthResultRequestInd->remote_BD_Type,
                                      pCOM_AuthResultRequestInd->keyType
                                     );
    }
#if BREDR_SUPPORT  
  else
  {
    LTPLibSendAuthResultRequestInd(&pBTLtp->LTPLib,
                                   BTLTP_DEFAULT_COPMSK,
                                   NULL,
                                   pCOM_AuthResultRequestInd->remote_BD
                                   );
  }
#endif
}

STATIC void LTPHandle_RegisterRsp(PBTLtp pBTLtp, PBlueAPI_RegisterRsp pCOM_RegisterRsp)
{
    if (pCOM_RegisterRsp->cause == blueAPI_CauseSuccess)
    {
        BTLTPCheckForActInfo(pBTLtp);
    }
}

STATIC void LTPHandle_PairableModeSetRsp(PBTLtp pBTLtp, PBlueAPI_PairableModeSetRsp pCOM_PairableModeSetRsp)
{
    LTPLibSendPairableModeSetRsp(&pBTLtp->LTPLib,
                                 BTLTP_DEFAULT_COPMSK,
                                 NULL,
                                 BTLTPConvertCOMtoLTPcause(pCOM_PairableModeSetRsp->cause)
                                );
}

STATIC void LTPHandle_UserPasskeyReqInd(PBTLtp pBTLtp, PBlueAPI_UserPasskeyReqInd pCOM_UserPasskeyReqInd)
{
    LTPLibSendPasskeyRequestInd(&pBTLtp->LTPLib,
                                BTLTP_DEFAULT_COPMSK,
                                NULL,
                                pCOM_UserPasskeyReqInd->remote_BD
                               );
}

STATIC void LTPHandle_UserPasskeyReqReplyRsp(PBTLtp pBTLtp, PBlueAPI_UserPasskeyReqReplyRsp pCOM_UserPasskeyReqReplyRsp)
{

    LTPLibSendPasskeyReqReplyRsp(&pBTLtp->LTPLib,
                                 BTLTP_DEFAULT_COPMSK,
                                 NULL,
                                 BTLTPConvertCOMtoLTPcause(pCOM_UserPasskeyReqReplyRsp->cause)
                                );
}

STATIC void LTPHandle_UserPasskeyNotificationInfo(PBTLtp pBTLtp, PBlueAPI_UserPasskeyNotificationInfo pCOM_UserPasskeyNotificationInfo)
{
    LTPLibSendPasskeyNotificationInfo(&pBTLtp->LTPLib,
                                      BTLTP_DEFAULT_COPMSK,
                                      NULL,
                                      pCOM_UserPasskeyNotificationInfo->remote_BD,
                                      pCOM_UserPasskeyNotificationInfo->displayValue
                                     );
}

STATIC void LTPHandle_RemoteOOBDataReqInd(PBTLtp pBTLtp, PBlueAPI_RemoteOOBDataReqInd pCOM_RemoteOOBDataReqInd)
{
    LTPLibSendRemoteOOBRequestInd(&pBTLtp->LTPLib,
                                  BTLTP_DEFAULT_COPMSK,
                                  NULL,
                                  pCOM_RemoteOOBDataReqInd->remote_BD
                                 );
}

STATIC void LTPHandle_AuthResultInd(PBTLtp pBTLtp, PBlueAPI_AuthResultInd pCOM_AuthResultInd)
{
  if (pCOM_AuthResultInd->remote_BD_Type != blueAPI_RemoteBDTypeClassic)
  {
    LTPLibSendAuthResultExtInd(&pBTLtp->LTPLib,
                               BTLTP_DEFAULT_COPMSK,
                               NULL,
                               BTLTPConvertCOMtoLTPcause(pCOM_AuthResultInd->cause),
                               pCOM_AuthResultInd->remote_BD,
                               pCOM_AuthResultInd->remote_BD_Type,
                               pCOM_AuthResultInd->keyType,
                               pCOM_AuthResultInd->linkKey,
                               pCOM_AuthResultInd->linkKeyLength
                               );
  }
#if BREDR_SUPPORT
  else
  {
    LTPLibSendAuthResultInd(&pBTLtp->LTPLib,
                            BTLTP_DEFAULT_COPMSK,
                            NULL,
                            BTLTPConvertCOMtoLTPcause(pCOM_AuthResultInd->cause),
                            pCOM_AuthResultInd->remote_BD,
                            pCOM_AuthResultInd->linkKey,
                            pCOM_AuthResultInd->keyType,
                            NULL
                           );
  }
#endif
}

STATIC void LTPHandle_ActInfo(PBTLtp pBTLtp, PBlueAPI_ActInfo pCOM_ActInfo)
{

    memcpy(pBTLtp->ownBDAddress        , pCOM_ActInfo->local_BD, 6);
    memcpy(pBTLtp->ActInfoVersionString, pCOM_ActInfo->version, BLUE_API_VERSION_LENGTH);

    pBTLtp->ActInfoCause = (uint8_t)BTLTPConvertCOMtoLTPcause(pCOM_ActInfo->cause);

    pBTLtp->ActInfoFlags |= LTP_ACT_INFO_FLAG_ACT_INFO;

    BTLTPCheckForActInfo(pBTLtp);
}

STATIC void LTPHandle_InternalEventInfo(PBTLtp pBTLtp, PBlueAPI_InternalEventInfo pCOM_InternalEventInfo)
{
    LTPLibSendInternalEventInfo(&pBTLtp->LTPLib,
                                BTLTP_DEFAULT_COPMSK,
                                NULL,
                                BTLTPConvertCOMtoLTPcause(pCOM_InternalEventInfo->cause),
                                pCOM_InternalEventInfo->eventType,
                                pCOM_InternalEventInfo->eventInfo
                               );
}

STATIC void LTPHandle_DeviceConfigSetRsp(PBTLtp pBTLtp, PBlueAPI_DeviceConfigSetRsp pCOM_DeviceConfigSetRsp)
{
    switch (pCOM_DeviceConfigSetRsp->opCode)
    {
    case blueAPI_DeviceConfigStore:
        LTPLibSendDeviceConfigStoreSetRsp(&pBTLtp->LTPLib,
                                          BTLTP_DEFAULT_COPMSK,
                                          NULL,
                                          pCOM_DeviceConfigSetRsp->cause
                                         );
        break;


    case blueAPI_DeviceConfigSecurity:
        LTPLibSendDeviceConfigSecuritySetRsp(&pBTLtp->LTPLib,
                                             BTLTP_DEFAULT_COPMSK,
                                             NULL,
                                             pCOM_DeviceConfigSetRsp->cause
                                            );
        break;

    case blueAPI_DeviceConfigDeviceName:
        LTPLibSendDeviceConfigDeviceNameSetRsp(&pBTLtp->LTPLib,
                                               BTLTP_DEFAULT_COPMSK,
                                               NULL,
                                               pCOM_DeviceConfigSetRsp->cause
                                              );

        break;
    case blueAPI_DeviceConfigAppearance:
        LTPLibSendDeviceConfigAppearanceSetRsp(&pBTLtp->LTPLib,
                                               BTLTP_DEFAULT_COPMSK,
                                               NULL,
                                               pCOM_DeviceConfigSetRsp->cause
                                              );

        break;
    case blueAPI_DeviceConfigPerPrefConnParam:
        LTPLibSendDeviceConfigPerPrefConnParamSetRsp(&pBTLtp->LTPLib,
                BTLTP_DEFAULT_COPMSK,
                NULL,
                pCOM_DeviceConfigSetRsp->cause
                                                    );

        break;

    default: /*-------------------------------------------------------------*/
#if BREDR_SUPPORT
        LTPHandle_BREDR_DeviceConfigSetRsp(pBTLtp, pCOM_DeviceConfigSetRsp);
#endif
        break;
    }
}

STATIC void LTPHandle_MCLStatusInfo(PBTLtp pBTLtp, PBlueAPI_MCLStatusInfo pCOM_MCLStatusInfo)
{
    LTPLibSendMCLStatusInfo(&pBTLtp->LTPLib,
                            BTLTP_DEFAULT_COPMSK,
                            NULL,
                            pCOM_MCLStatusInfo->remote_BD,
                            (uint8_t)pCOM_MCLStatusInfo->local_MCL_ID,
                            pCOM_MCLStatusInfo->status
                           );
}

STATIC void LTPHandle_ACLStatusInfo(PBTLtp pBTLtp, PBlueAPI_ACLStatusInfo pCOM_ACLStatusInfo)
{
    uint8_t pOpt[3];
    uint8_t pos = 0;
    uint8_t copmsk = (BTLTP_DEFAULT_COPMSK | LTP_ACL_STATUS_INFO_OPT_MASK_BD_TYPE);

    pOpt[pos++] = pCOM_ACLStatusInfo->remote_BD_type;

    if (0)
    {}
#if BREDR_SUPPORT
    else if (pCOM_ACLStatusInfo->status == blueAPI_ACLConnectedSniffSubrate)
    {
        LTPLibSendACLSniffSubrateInfo(&pBTLtp->LTPLib,
                                  BTLTP_DEFAULT_COPMSK,
                                  NULL,
                                  pCOM_ACLStatusInfo->remote_BD,
                                  pCOM_ACLStatusInfo->p.sniffSubrate.maxTxLatency,
                                  pCOM_ACLStatusInfo->p.sniffSubrate.maxRxLatency,
                                  pCOM_ACLStatusInfo->p.sniffSubrate.minRemoteTimeout,
                                  pCOM_ACLStatusInfo->p.sniffSubrate.minLocalTimeout
                                  );
    }
#endif
    else
    {
        switch (pCOM_ACLStatusInfo->status)
        {
#if BREDR_SUPPORT
        case blueAPI_ACLConnectedSniff: /*--------------------------------------*/
            NETSHORT2CHAR(&pOpt[pos], pCOM_ACLStatusInfo->p.sniff.interval); pos+=2;
            copmsk |= LTP_ACL_STATUS_INFO_OPT_MASK_SNIFF_INTERVAL;
            break;
#endif           
        case blueAPI_ACLAuthenticationStarted:
        case blueAPI_ACLAuthenticationSuccess:
        case blueAPI_ACLAuthenticationFailure:
        case blueAPI_ACLConnectionEncrypted:
        case blueAPI_ACLConnectionNotEncrypted:
            pOpt[pos++] = pCOM_ACLStatusInfo->p.auth.keyType;
            pOpt[pos++] = pCOM_ACLStatusInfo->p.auth.keySize;
            copmsk |= (LTP_ACL_STATUS_INFO_OPT_MASK_KEY_TYPE |
                       LTP_ACL_STATUS_INFO_OPT_MASK_KEY_SIZE);
            break;

        default:
            break;
        }

        LTPLibSendACLStatusInfo(&pBTLtp->LTPLib,
                                copmsk,
                                pOpt,
                                pCOM_ACLStatusInfo->remote_BD,
                                pCOM_ACLStatusInfo->status
                               );
    }
}

STATIC void LTPHandle_GATTServiceRegisterRsp(PBTLtp pBTLtp, PBlueAPI_GATTServiceRegisterRsp pServiceRegisterRsp)
{
    uint8_t short_serviceHandle;
#if ACI_EN
    short_serviceHandle = pBTLtp->service_register_idx;
    pBTLtp->gattServiceTable[short_serviceHandle].serviceHandle = pServiceRegisterRsp->serviceHandle;
#else
    short_serviceHandle = BTLTPAllocateFindGATTServiceHandle(pBTLtp, pServiceRegisterRsp->serviceHandle);
#endif

    LTPLibSendGATTServiceRegisterRsp(&pBTLtp->LTPLib,
                                     BTLTP_DEFAULT_COPMSK,
                                     NULL,
                                     pServiceRegisterRsp->cause,
                                     pServiceRegisterRsp->subCause,
                                     short_serviceHandle
                                    );
}


STATIC void LTPHandle_GATTAttributeUpdateRsp(PBTLtp pBTLtp, uint8_t * pBuffer,
        PBlueAPI_GATTAttributeUpdateRsp pAttributeUpdateRsp)
{
    uint8_t serviceHandle;
    uint16_t pos    = 0;
    uint16_t offset = offsetof(TBlueAPI_UsMessage, p.GATTAttributeUpdateRsp.list) + pAttributeUpdateRsp->gap;
    TBlueAPI_GATTAttributeUpdateRsp  attributeUpdateRsp = *pAttributeUpdateRsp;
    uint16_t msgLen = LTPLibInsertHeader(&pBTLtp->LTPLib,
                                     pBuffer, &offset,
                                     pAttributeUpdateRsp->count * (6 + 1),
                                     LTP_GATT_ATTRIBUTE_UPDATE_RSP,
                                     BTLTP_DEFAULT_COPMSK,
                                     NULL,
                                     &pos
                                    );
#if ACI_EN
    serviceHandle = BTACIFindGATTServiceHandle(pBTLtp, pAttributeUpdateRsp->serviceHandle);
#else
    serviceHandle = BTLTPAllocateFindGATTServiceHandle(pBTLtp, pAttributeUpdateRsp->serviceHandle);
#endif

    if (msgLen > 0)
    {
        pBuffer[pos++] = attributeUpdateRsp.cause;
        NETSHORT2CHAR(&pBuffer[pos], attributeUpdateRsp.subCause);
        pos += 2;
        pBuffer[pos++] = serviceHandle;
        NETLONG2CHAR(&pBuffer[pos], (uint32_t)attributeUpdateRsp.requestHandle);
        pos += 4;
        NETSHORT2CHAR(&pBuffer[pos], attributeUpdateRsp.attribIndex);
        pos += 2;

        uint8_t* pTxBuffer = BTLTPTgtSendBufferAlloc(pBTLtp->LTPLib.AppHandle, msgLen);
        memcpy(pTxBuffer, pBuffer + offset, msgLen);
        BTLTPTgtSendLTPMessage(pBTLtp, pTxBuffer, 0, msgLen);
    }

    return;
}

STATIC void LTPHandle_GATTAttributeUpdateStatusInd(PBTLtp pBTLtp, PBlueAPI_GATTAttributeUpdateStatusInd pAttributeUpdateStatusInd)
{
    uint8_t serviceHandle;
#if ACI_EN
    serviceHandle = BTACIFindGATTServiceHandle(pBTLtp, pAttributeUpdateStatusInd->serviceHandle);
#else
    serviceHandle = BTLTPAllocateFindGATTServiceHandle(pBTLtp, pAttributeUpdateStatusInd->serviceHandle);
#endif

    LTPLibSendGATTAttributeUpdateStatusInd(&pBTLtp->LTPLib,
                                           BTLTP_DEFAULT_COPMSK,
                                           NULL,
                                           pAttributeUpdateStatusInd->cause,
                                           pAttributeUpdateStatusInd->subCause,
                                           serviceHandle,
                                           pAttributeUpdateStatusInd->requestHandle,
                                           pAttributeUpdateStatusInd->attribIndex,
                                           pAttributeUpdateStatusInd->remote_BD,
                                           pAttributeUpdateStatusInd->remote_BD_Type
                                          );
}

STATIC void LTPHandle_GATTAttributeReadInd(PBTLtp pBTLtp, PBlueAPI_GATTAttributeReadInd pAttributeReadInd)
{
    uint8_t serviceHandle;
#if ACI_EN
    serviceHandle = BTACIFindGATTServiceHandle(pBTLtp, pAttributeReadInd->serviceHandle);
#else
    serviceHandle = BTLTPAllocateFindGATTServiceHandle(pBTLtp, pAttributeReadInd->serviceHandle);
#endif

    LTPLibSendGATTAttributeReadInd(&pBTLtp->LTPLib,
                                   BTLTP_DEFAULT_COPMSK,
                                   NULL,
                                   (uint8_t)pAttributeReadInd->local_MDL_ID,
                                   serviceHandle,
                                   pAttributeReadInd->attribIndex,
                                   pAttributeReadInd->readOffset
                                  );
}

STATIC void LTPHandle_GATTAttributeWriteInd(PBTLtp pBTLtp, uint8_t * pBuffer,
        TBlueAPI_Command command,
        PBlueAPI_GATTAttributeWriteInd pAttributeWriteInd)
{
    uint8_t serviceHandle;
    uint16_t pos    = 0;
    uint16_t offset = offsetof(TBlueAPI_UsMessage, p.GATTAttributeWriteInd.data) + pAttributeWriteInd->gap;
    uint16_t local_MDL_ID = pAttributeWriteInd->local_MDL_ID;
    uint16_t attribIndex  = pAttributeWriteInd->attribIndex;
    uint16_t writeOffset  = pAttributeWriteInd->writeOffset;
    uint16_t handle       = pAttributeWriteInd->handle;
    uint16_t msgLen = LTPLibInsertHeader(&pBTLtp->LTPLib,
                                     pBuffer, &offset,
                                     pAttributeWriteInd->attribLength,
                                     (command == blueAPI_EventGATTAttributeWriteInd) ? LTP_GATT_ATTRIBUTE_WRITE_IND
                                     : LTP_GATT_ATTRIBUTE_WRITE_COMMAND_INFO,
                                     BTLTP_DEFAULT_COPMSK,
                                     NULL,
                                     &pos
                                    );
#if ACI_EN
    serviceHandle = BTACIFindGATTServiceHandle(pBTLtp, pAttributeWriteInd->serviceHandle);
#else
    serviceHandle = BTLTPAllocateFindGATTServiceHandle(pBTLtp, pAttributeWriteInd->serviceHandle);
#endif

    if (msgLen > 0)
    {
        pBuffer[pos++] = (uint8_t)local_MDL_ID;
        pBuffer[pos++] = serviceHandle;
        NETSHORT2CHAR(&pBuffer[pos], attribIndex);
        pos += 2;
        NETSHORT2CHAR(&pBuffer[pos], writeOffset);
        pos += 2;
        NETSHORT2CHAR(&pBuffer[pos], handle);
        pos += 2;
        /* memory copy to tx buffer */
        uint8_t* pTxBuffer = BTLTPTgtSendBufferAlloc(pBTLtp->LTPLib.AppHandle, msgLen);
        memcpy(pTxBuffer, pBuffer + offset, msgLen);
        BTLTPTgtSendLTPMessage(pBTLtp, pTxBuffer, 0, msgLen);
    }

    return;
}

STATIC void LTPHandle_GATTCCCDInfo(PBTLtp pBTLtp, uint8_t * pBuffer,
                                   PBlueAPI_GATTCCCDInfo pCCCDInfo)
{
    uint16_t local_MDL_ID  = pCCCDInfo->local_MDL_ID;
    uint8_t serviceHandle;

    uint16_t pos    = 0;
    uint16_t offset = offsetof(TBlueAPI_UsMessage, p.GATTCCCDInfo.data) + pCCCDInfo->gap;
    uint16_t msgLen = LTPLibInsertHeader(&pBTLtp->LTPLib,
                                     pBuffer, &offset,
                                     (pCCCDInfo->count * 4),
                                     LTP_GATT_CCCD_INFO,
                                     BTLTP_DEFAULT_COPMSK,
                                     NULL,
                                     &pos
                                    );
#if ACI_EN
    serviceHandle = BTACIFindGATTServiceHandle(pBTLtp, pCCCDInfo->serviceHandle);
#else
    serviceHandle = BTLTPAllocateFindGATTServiceHandle(pBTLtp, pCCCDInfo->serviceHandle);
#endif

    if (msgLen > 0)
    {
        pBuffer[pos++] = (uint8_t)local_MDL_ID;
        pBuffer[pos++] = serviceHandle;

        /* memory copy to tx buffer */
        uint8_t* pTxBuffer = BTLTPTgtSendBufferAlloc(pBTLtp->LTPLib.AppHandle, msgLen);
        memcpy(pTxBuffer, pBuffer + offset, msgLen);
        BTLTPTgtSendLTPMessage(pBTLtp, pTxBuffer, 0, msgLen);
    }

    return;
}

STATIC void LTPHandle_GATTDiscoveryRsp(PBTLtp pBTLtp, PBlueAPI_GATTDiscoveryRsp pDiscoveryRsp)
{
    LTPLibSendGATTDiscoveryRsp(&pBTLtp->LTPLib,
                               BTLTP_DEFAULT_COPMSK,
                               NULL,
                               pDiscoveryRsp->cause,
                               pDiscoveryRsp->subCause,
                               (uint8_t)pDiscoveryRsp->local_MDL_ID,
                               pDiscoveryRsp->discoveryType
                              );
}

STATIC void LTPHandle_GATTDiscoveryInd(PBTLtp pBTLtp, uint8_t * pBuffer,
                                       PBlueAPI_GATTDiscoveryInd pDiscoveryInd)
{
    uint16_t pos    = 0;
    uint16_t offset = offsetof(TBlueAPI_UsMessage, p.GATTDiscoveryInd.list) + pDiscoveryInd->gap;
    TBlueAPI_GATTDiscoveryInd discoveryInd = *pDiscoveryInd;
    uint16_t msgLen = LTPLibInsertHeader(&pBTLtp->LTPLib,
                                     pBuffer, &offset,
                                     (pDiscoveryInd->elementCount * pDiscoveryInd->elementLength),
                                     LTP_GATT_DISCOVERY_IND,
                                     BTLTP_DEFAULT_COPMSK,
                                     NULL,
                                     &pos
                                    );

    if (msgLen > 0)
    {
        pBuffer[pos++] = discoveryInd.cause;
        NETSHORT2CHAR(&pBuffer[pos], discoveryInd.subCause);
        pos += 2;
        pBuffer[pos++] = (uint8_t)discoveryInd.local_MDL_ID;
        pBuffer[pos++] = discoveryInd.discoveryType;
        pBuffer[pos++] = (uint8_t)discoveryInd.elementLength;

        /* memory copy to tx buffer */
        uint8_t* pTxBuffer = BTLTPTgtSendBufferAlloc(pBTLtp->LTPLib.AppHandle, msgLen);
        memcpy(pTxBuffer, pBuffer + offset, msgLen);
        BTLTPTgtSendLTPMessage(pBTLtp, pTxBuffer, 0, msgLen);
    }

    return;
}

STATIC void LTPHandle_GATTAttributeReadRsp(PBTLtp pBTLtp, uint8_t * pBuffer,
        PBlueAPI_GATTAttributeReadRsp pAttributeReadRsp)
{
    uint16_t pos    = 0;
    uint16_t offset = offsetof(TBlueAPI_UsMessage, p.GATTAttributeReadRsp.handlesData) + pAttributeReadRsp->gap;
    TBlueAPI_GATTAttributeReadRsp  attributeReadRsp = *pAttributeReadRsp;
    uint16_t msgLen = LTPLibInsertHeader(&pBTLtp->LTPLib,
                                     pBuffer, &offset,
                                     pAttributeReadRsp->totalLength,
                                     LTP_GATT_ATTRIBUTE_READ_RSP,
                                     BTLTP_DEFAULT_COPMSK,
                                     NULL,
                                     &pos
                                    );

    if (msgLen > 0)
    {
        pBuffer[pos++] = attributeReadRsp.cause;
        NETSHORT2CHAR(&pBuffer[pos], attributeReadRsp.subCause);
        pos += 2;
        pBuffer[pos++] = (uint8_t)attributeReadRsp.local_MDL_ID;
        pBuffer[pos++] = attributeReadRsp.readType;
        NETSHORT2CHAR(&pBuffer[pos], attributeReadRsp.readOffset);
        pos += 2;
        NETSHORT2CHAR(&pBuffer[pos], attributeReadRsp.attribLength);
        pos += 2;
        pBuffer[pos++] = (uint8_t)attributeReadRsp.nbrOfHandles;

        /* memory copy to tx buffer */
        uint8_t* pTxBuffer = BTLTPTgtSendBufferAlloc(pBTLtp->LTPLib.AppHandle, msgLen);
        memcpy(pTxBuffer, pBuffer + offset, msgLen);
        BTLTPTgtSendLTPMessage(pBTLtp, pTxBuffer, 0, msgLen);
    }

    return;
}

STATIC void LTPHandle_GATTAttributeWriteRsp(PBTLtp pBTLtp, PBlueAPI_GATTAttributeWriteRsp pAttributeWriteRsp)
{
    LTPLibSendGATTAttributeWriteRsp(&pBTLtp->LTPLib,
                                    BTLTP_DEFAULT_COPMSK,
                                    NULL,
                                    pAttributeWriteRsp->cause,
                                    pAttributeWriteRsp->subCause,
                                    (uint8_t)pAttributeWriteRsp->local_MDL_ID,
                                    pAttributeWriteRsp->writeType
                                   );
}

STATIC void LTPHandle_GATTAttributeInd(PBTLtp pBTLtp, uint8_t * pBuffer,
                                       PBlueAPI_GATTAttributeInd pAttributeInd)
{
    uint16_t pos    = 0;
    uint16_t local_MDL_ID = pAttributeInd->local_MDL_ID;
    uint16_t attribHandle = pAttributeInd->attribHandle;
    uint16_t offset = offsetof(TBlueAPI_UsMessage, p.GATTAttributeInd.data) + pAttributeInd->gap;
    uint16_t msgLen = LTPLibInsertHeader(&pBTLtp->LTPLib,
                                     pBuffer, &offset,
                                     pAttributeInd->attribLength,
                                     LTP_GATT_ATTRIBUTE_IND,
                                     BTLTP_DEFAULT_COPMSK,
                                     NULL,
                                     &pos
                                    );

    if (msgLen > 0)
    {
        pBuffer[pos++] = (uint8_t)local_MDL_ID;
        NETSHORT2CHAR(&pBuffer[pos], attribHandle);
        pos += 2;

        /* memory copy to tx buffer */
        uint8_t* pTxBuffer = BTLTPTgtSendBufferAlloc(pBTLtp->LTPLib.AppHandle, msgLen);
        memcpy(pTxBuffer, pBuffer + offset, msgLen);
        BTLTPTgtSendLTPMessage(pBTLtp, pTxBuffer, 0, msgLen);
    }

    return;
}

STATIC void LTPHandle_GATTAttributeNotificationInfo(PBTLtp pBTLtp, uint8_t * pBuffer,
        PBlueAPI_GATTAttributeNotificationInfo pAttributeNotificationInfo)
{
    uint16_t pos    = 0;
    uint16_t local_MDL_ID = pAttributeNotificationInfo->local_MDL_ID;
    uint16_t attribHandle = pAttributeNotificationInfo->attribHandle;
    uint16_t offset = offsetof(TBlueAPI_UsMessage, p.GATTAttributeNotificationInfo.data) + pAttributeNotificationInfo->gap;
    uint16_t msgLen = LTPLibInsertHeader(&pBTLtp->LTPLib,
                                     pBuffer, &offset,
                                     pAttributeNotificationInfo->attribLength,
                                     LTP_GATT_ATTRIBUTE_NOTIFICATION_INFO,
                                     BTLTP_DEFAULT_COPMSK,
                                     NULL,
                                     &pos
                                    );

    if (msgLen > 0)
    {
        pBuffer[pos++] = (uint8_t)local_MDL_ID;
        NETSHORT2CHAR(&pBuffer[pos], attribHandle);
        pos += 2;

        /* memory copy to tx buffer */
        uint8_t* pTxBuffer = BTLTPTgtSendBufferAlloc(pBTLtp->LTPLib.AppHandle, msgLen);
        memcpy(pTxBuffer, pBuffer + offset, msgLen);
        BTLTPTgtSendLTPMessage(pBTLtp, pTxBuffer, 0, msgLen);
    }

    return;
}

STATIC void LTPHandle_GATTSecurityRsp(PBTLtp pBTLtp, PBlueAPI_GATTSecurityRsp pSecurityRsp)
{
    LTPLibSendGATTSecurityRsp(&pBTLtp->LTPLib,
                              BTLTP_DEFAULT_COPMSK,
                              NULL,
                              pSecurityRsp->cause,
                              (uint8_t)pSecurityRsp->local_MDL_ID,
                              pSecurityRsp->keyType,
                              pSecurityRsp->keySize
                             );
}

STATIC void LTPHandle_GATTServerStoreInd(PBTLtp pBTLtp, PBlueAPI_GATTServerStoreInd pServerStoreInd)
{
    LTPLibSendGATTServerStoreInd(&pBTLtp->LTPLib,
                                 BTLTP_DEFAULT_COPMSK,
                                 NULL,
                                 pServerStoreInd->opCode,
                                 pServerStoreInd->remote_BD,
                                 pServerStoreInd->remote_BD_Type,
                                 pServerStoreInd->restartHandle,
                                 pServerStoreInd->data,
                                 pServerStoreInd->dataLength
                                );
}

STATIC void LTPHandle_GATTMtuSizeInfo(PBTLtp pBTLtp, PBlueAPI_GATTMtuSizeInfo pMtuSizeInfo)
{
    LTPLibSendGATTMtuSizeInfo(&pBTLtp->LTPLib,
                              BTLTP_DEFAULT_COPMSK,
                              NULL,
                              (uint8_t)pMtuSizeInfo->local_MDL_ID,
                              pMtuSizeInfo->mtuSize
                             );
}

STATIC void LTPHandle_LEAdvertiseRsp(PBTLtp pBTLtp, PBlueAPI_LEAdvertiseRsp pAdvertiseRsp)
{
    LTPLibSendLEAdvertiseRsp(&pBTLtp->LTPLib,
                             BTLTP_DEFAULT_COPMSK,
                             NULL,
                             pAdvertiseRsp->cause,
                             pAdvertiseRsp->advMode
                            );
}

STATIC void LTPHandle_SetRandomAddressRsp(PBTLtp pBTLtp, PBlueAPI_SetRandomAddressRsp pRandomAddressRsp)
{
    LTPLibSendSetRandomAddressRsp(&pBTLtp->LTPLib,
                                  BTLTP_DEFAULT_COPMSK,
                                  NULL,
                                  pRandomAddressRsp->cause,
                                  pRandomAddressRsp->subCause
                                 );
}

STATIC void LTPHandle_LEAdvertiseParameterSetRsp(PBTLtp pBTLtp, PBlueAPI_LEAdvertiseParameterSetRsp pAdvertiseParameterSetRsp)
{
    LTPLibSendLEAdvertiseParameterSetRsp(&pBTLtp->LTPLib,
                                         BTLTP_DEFAULT_COPMSK,
                                         NULL,
                                         pAdvertiseParameterSetRsp->cause
                                        );
}

STATIC void LTPHandle_LEAdvertiseDataSetRsp(PBTLtp pBTLtp, PBlueAPI_LEAdvertiseDataSetRsp pAdvertiseDataSetRsp)
{
    LTPLibSendLEAdvertiseDataSetRsp(&pBTLtp->LTPLib,
                                    BTLTP_DEFAULT_COPMSK,
                                    NULL,
                                    pAdvertiseDataSetRsp->cause,
                                    pAdvertiseDataSetRsp->dataType
                                   );
}

STATIC void LTPHandle_LEScanRsp(PBTLtp pBTLtp, PBlueAPI_LEScanRsp pScanRsp)
{
    LTPLibSendLEScanRsp(&pBTLtp->LTPLib,
                        BTLTP_DEFAULT_COPMSK,
                        NULL,
                        pScanRsp->cause
                       );
}

STATIC void LTPHandle_LEScanInfo(PBTLtp pBTLtp, PBlueAPI_LEScanInfo pScanInfo)
{
    LTPLibSendLEScanInfo(&pBTLtp->LTPLib,
                         BTLTP_DEFAULT_COPMSK,
                         NULL,
                         pScanInfo->remote_BD,
                         pScanInfo->remote_BD_type,
                         pScanInfo->advType,
                         pScanInfo->rssi,
                         pScanInfo->data,
                         pScanInfo->dataLength
                        );
}

STATIC void LTPHandle_LEModifyWhitelistRsp(PBTLtp pBTLtp, PBlueAPI_LEModifyWhitelistRsp pModifyWhitelistRsp)
{
    LTPLibSendLEModifyWhitelistRsp(&pBTLtp->LTPLib,
                                   BTLTP_DEFAULT_COPMSK,
                                   NULL,
                                   pModifyWhitelistRsp->cause,
                                   pModifyWhitelistRsp->operation
                                  );
}

STATIC void LTPHandle_LEConnectionUpdateRsp(PBTLtp pBTLtp, PBlueAPI_LEConnectionUpdateRsp pConnectionUpdateRsp)
{
    LTPLibSendLEConnectionUpdateRsp(&pBTLtp->LTPLib,
                                    BTLTP_DEFAULT_COPMSK,
                                    NULL,
                                    pConnectionUpdateRsp->cause,
                                    (uint8_t)pConnectionUpdateRsp->local_MDL_ID
                                   );
}

STATIC void LTPHandle_LEConnectionUpdateInd(PBTLtp pBTLtp, PBlueAPI_LEConnectionUpdateInd pConnectionUpdateInd)
{
    LTPLibSendLEConnectionUpdateInd(&pBTLtp->LTPLib,
                                    BTLTP_DEFAULT_COPMSK,
                                    NULL,
                                    (uint8_t)pConnectionUpdateInd->local_MDL_ID,
                                    pConnectionUpdateInd->connIntervalMin,
                                    pConnectionUpdateInd->connIntervalMax,
                                    pConnectionUpdateInd->connLatency,
                                    pConnectionUpdateInd->supervisionTimeout
                                   );
}

STATIC void LTPHandle_LEConnectionParameterInfo(PBTLtp pBTLtp, PBlueAPI_LEConnectionParameterInfo pConnectionParameterInfo)
{
    LTPLibSendLEConnectionParameterInfo(&pBTLtp->LTPLib,
                                        BTLTP_DEFAULT_COPMSK,
                                        NULL,
                                        (uint8_t)pConnectionParameterInfo->local_MDL_ID,
                                        pConnectionParameterInfo->connInterval,
                                        pConnectionParameterInfo->connLatency,
                                        pConnectionParameterInfo->supervisionTimeout
                                       );
}
#if 0
STATIC void LTPHandle_SetBleTxPowerRsp(PBTLtp pBTLtp, PBlueAPI_SetBleTxPowerRsp pSetBleTxPowerRsp)
{
    LTPLibSendSetBleTxPowerRsp(&pBTLtp->LTPLib,
                               pSetBleTxPowerRsp->tx_power_index,
                               pSetBleTxPowerRsp->cause,
                               pSetBleTxPowerRsp->subCause
                              );
}

STATIC void LTPHandle_SetDataLengthRsp(PBTLtp pBTLtp, PBlueAPI_SetDataLengthRsp pSetDataLengthRsp)
{
    LTPLibSendSetDataLengthRsp(&pBTLtp->LTPLib,
                               (uint8_t)pSetDataLengthRsp->local_MDL_ID,
                               pSetDataLengthRsp->cause
                              );
}

STATIC void LTPHandle_DataLengthChangeInfo(PBTLtp pBTLtp, PBlueAPI_DataLengthChangeInfo pDataLengthChangeInfo)
{
    LTPLibSendDataLengthChangeInfo(&pBTLtp->LTPLib,
                                   (uint8_t)pDataLengthChangeInfo->local_MDL_ID,
                                   pDataLengthChangeInfo->MaxTxOctets,
                                   pDataLengthChangeInfo->MaxTxTime,
                                   pDataLengthChangeInfo->MaxRxOctets,
                                   pDataLengthChangeInfo->MaxRxTime
                                  );
}

STATIC void LTPHandle_GATTAttributeExecuteWriteRsp(PBTLtp pBTLtp, PBlueAPI_GATTAttributeExecuteWriteRsp pGATTAttributeExecuteWriteRsp)
{
    LTPLibSendGATTAttributeExecuteWriteRsp(&pBTLtp->LTPLib,
                                   (uint8_t)pGATTAttributeExecuteWriteRsp->local_MDL_ID,
                                   pGATTAttributeExecuteWriteRsp->cause,
                                   pGATTAttributeExecuteWriteRsp->subCause
                                  );
}

STATIC void LTPHandle_GATTAttributePrepareWriteInd(PBTLtp pBTLtp, uint8_t * pBuffer,PBlueAPI_GATTAttributeWriteInd pAttributeWriteInd)
{
    uint8_t serviceHandle;
    uint16_t pos    = 0;
    uint16_t offset = offsetof(TBlueAPI_UsMessage, p.GATTAttributeWriteInd.data) + pAttributeWriteInd->gap;
    uint16_t local_MDL_ID = pAttributeWriteInd->local_MDL_ID;
    uint16_t attribIndex  = pAttributeWriteInd->attribIndex;
    uint16_t writeOffset  = pAttributeWriteInd->writeOffset;
    uint16_t handle       = pAttributeWriteInd->handle;
    uint16_t msgLen = LTPLibInsertExtendHeader(&pBTLtp->LTPLib,
                                     pBuffer, &offset,
                                     pAttributeWriteInd->attribLength,
                                     LTP_SUB_GATT_ATTRIBUTE_PREPARE_WRITE_IND,
                                     BTLTP_DEFAULT_COPMSK,
                                     NULL,
                                     &pos
                                    );
#if ACI_EN
    serviceHandle = BTACIFindGATTServiceHandle(pBTLtp, pAttributeWriteInd->serviceHandle);
#else
    serviceHandle = BTLTPAllocateFindGATTServiceHandle(pBTLtp, pAttributeWriteInd->serviceHandle);
#endif

    if (msgLen > 0)
    {
        pBuffer[pos++] = (uint8_t)local_MDL_ID;
        pBuffer[pos++] = serviceHandle;
        NETSHORT2CHAR(&pBuffer[pos], attribIndex);
        pos += 2;
        NETSHORT2CHAR(&pBuffer[pos], writeOffset);
        pos += 2;
        NETSHORT2CHAR(&pBuffer[pos], handle);
        pos += 2;
        /* memory copy to tx buffer */
        uint8_t* pTxBuffer = BTLTPTgtSendBufferAlloc(pBTLtp->LTPLib.AppHandle, msgLen);
        memcpy(pTxBuffer, pBuffer + offset, msgLen);
        BTLTPTgtSendLTPMessage(pBTLtp, pTxBuffer, 0, msgLen);
    }

    return;
}

STATIC void LTPHandle_GATTAttributePrepareWriteRsp(PBTLtp pBTLtp, uint8_t * pBuffer,PBlueAPI_GATTAttributePrepareWriteRsp pGATTAttributePrepareWriteRsp)
{
    uint16_t pos    = 0;
    uint16_t offset = offsetof(TBlueAPI_UsMessage, p.GATTAttributePrepareWriteRsp.data) + pGATTAttributePrepareWriteRsp->gap;
    uint16_t local_MDL_ID = pGATTAttributePrepareWriteRsp->local_MDL_ID;
    uint8_t cause  = pGATTAttributePrepareWriteRsp->cause;
    uint16_t subCause = pGATTAttributePrepareWriteRsp->subCause;
    uint16_t writeOffset  = pGATTAttributePrepareWriteRsp->writeOffset;
    uint16_t msgLen = LTPLibInsertExtendHeader(&pBTLtp->LTPLib,
                                     pBuffer, &offset,
                                     pGATTAttributePrepareWriteRsp->attribLength,
                                     LTP_SUB_GATT_ATTRIBUTE_PREPARE_WRITE_RSP,
                                     BTLTP_DEFAULT_COPMSK,
                                     NULL,
                                     &pos
                                    );
    if (msgLen > 0)
    {
        pBuffer[pos++] = (uint8_t)local_MDL_ID;
        pBuffer[pos++] = cause;
        NETSHORT2CHAR(&pBuffer[pos], subCause);
        pos += 2;
        NETSHORT2CHAR(&pBuffer[pos], writeOffset);
        pos += 2;
        /* memory copy to tx buffer */
        uint8_t* pTxBuffer = BTLTPTgtSendBufferAlloc(pBTLtp->LTPLib.AppHandle, msgLen);
        memcpy(pTxBuffer, pBuffer + offset, msgLen);
        BTLTPTgtSendLTPMessage(pBTLtp, pTxBuffer, 0, msgLen);
    }

    return;
}

STATIC void LTPHandle_GATTAttributeExecuteWriteInd(PBTLtp pBTLtp, PBlueAPI_GATTAttributeExecuteWriteInd pGATTAttributeExecuteWriteInd)
{
    LTPLibSendGATTAttributeExecuteWriteInd(&pBTLtp->LTPLib,
                                   BTLTP_DEFAULT_COPMSK, NULL,
                                   (uint8_t)pGATTAttributeExecuteWriteInd->local_MDL_ID,
                                   pGATTAttributeExecuteWriteInd->flags
                                  );
}
#endif
void BTLTPHandleBLUE_API_MSG(PBTLtp pBTLtp, uint8_t * pBuffer, uint16_t offset)
{
    PBlueAPI_UsMessage pCOMMsg       = (PBlueAPI_UsMessage)(pBuffer + offset);

    switch (pCOMMsg->Command)
    {
    case blueAPI_EventRegisterRsp: /*-----------------------------------*/
        LTPHandle_RegisterRsp(pBTLtp, &pCOMMsg->p.RegisterRsp);
        break;
    case blueAPI_EventCreateMDLInd: /*----------------------------------*/
        LTPHandle_CreateMDLInd(pBTLtp, &pCOMMsg->p.CreateMDLInd);
        break;
    case blueAPI_EventConnectMDLInfo: /*--------------------------------*/
        LTPHandle_ConnectMDLInfo(pBTLtp, &pCOMMsg->p.ConnectMDLInfo);
        break;
    case blueAPI_EventDisconnectMDLInd: /*------------------------------*/
        LTPHandle_DisconnectMDLInd(pBTLtp, &pCOMMsg->p.DisconnectMDLInd);
        break;
    case blueAPI_EventConnectMDLRsp: /*---------------------------------*/
        LTPHandle_ConnectMDLRsp(pBTLtp, &pCOMMsg->p.ConnectMDLRsp);
        break;
    case blueAPI_EventDisconnectMDLRsp: /*------------------------------*/
        LTPHandle_DisconnectMDLRsp(pBTLtp, &pCOMMsg->p.DisconnectMDLRsp);
        break;
    case blueAPI_EventDeleteMDLInfo: /*---------------------------------*/
        LTPHandle_DeleteMDLInfo(pBTLtp, &pCOMMsg->p.DeleteMDLInfo);
        break;
    case blueAPI_EventMCLStatusInfo: /*---------------------------------*/
        LTPHandle_MCLStatusInfo(pBTLtp, &pCOMMsg->p.MCLStatusInfo);
        break;
    case blueAPI_EventACLStatusInfo: /*---------------------------------*/
        LTPHandle_ACLStatusInfo(pBTLtp, &pCOMMsg->p.ACLStatusInfo);
        break;
    case blueAPI_EventDeviceConfigSetRsp: /*----------------------------*/
        LTPHandle_DeviceConfigSetRsp(pBTLtp, &pCOMMsg->p.DeviceConfigSetRsp);
        break;
    case blueAPI_EventPairableModeSetRsp: /*----------------------------*/
        LTPHandle_PairableModeSetRsp(pBTLtp, &pCOMMsg->p.PairableModeSetRsp);
        break;
    case blueAPI_EventUserPasskeyReqInd: /*-----------------------------*/
        LTPHandle_UserPasskeyReqInd(pBTLtp, &pCOMMsg->p.UserPasskeyReqInd);
        break;
    case blueAPI_EventUserPasskeyReqReplyRsp: /*------------------------*/
        LTPHandle_UserPasskeyReqReplyRsp(pBTLtp, &pCOMMsg->p.UserPasskeyReqReplyRsp);
        break;
    case blueAPI_EventUserPasskeyNotificationInfo: /*-------------------*/
        LTPHandle_UserPasskeyNotificationInfo(pBTLtp, &pCOMMsg->p.UserPasskeyNotificationInfo);
        break;
    case blueAPI_EventRemoteOOBDataReqInd: /*---------------------------*/
        LTPHandle_RemoteOOBDataReqInd(pBTLtp, &pCOMMsg->p.RemoteOOBDataReqInd);
        break;
    case blueAPI_EventAuthResultInd: /*---------------------------------*/
        LTPHandle_AuthResultInd(pBTLtp, &pCOMMsg->p.AuthResultInd);
        break;

    case blueAPI_EventAuthResultRequestInd: /*--------------------------*/
        LTPHandle_AuthResultRequestInd(pBTLtp, &pCOMMsg->p.AuthResultRequestInd);
        break;

    case blueAPI_EventActInfo: /*---------------------------------------*/
        LTPHandle_ActInfo(pBTLtp, &pCOMMsg->p.ActInfo);
        break;
    case blueAPI_EventInternalEventInfo: /*-----------------------------*/
        LTPHandle_InternalEventInfo(pBTLtp, &pCOMMsg->p.InternalEventInfo);
        break;

    case blueAPI_EventGATTServiceRegisterRsp: /*------------------------*/
        LTPHandle_GATTServiceRegisterRsp(pBTLtp, &pCOMMsg->p.GATTServiceRegisterRsp);
        break;
    case blueAPI_EventGATTAttributeUpdateRsp: /*------------------------*/
        LTPHandle_GATTAttributeUpdateRsp(pBTLtp, pBuffer, &pCOMMsg->p.GATTAttributeUpdateRsp);
        break;
    case blueAPI_EventGATTAttributeUpdateStatusInd: /*------------------*/
        LTPHandle_GATTAttributeUpdateStatusInd(pBTLtp, &pCOMMsg->p.GATTAttributeUpdateStatusInd);
        break;
    case blueAPI_EventGATTAttributeReadInd: /*--------------------------*/
        LTPHandle_GATTAttributeReadInd(pBTLtp, &pCOMMsg->p.GATTAttributeReadInd);
        break;
    case blueAPI_EventGATTAttributeWriteInd: /*-------------------------*/
    case blueAPI_EventGATTAttributeWriteCommandInfo: /*-----------------*/
        LTPHandle_GATTAttributeWriteInd(pBTLtp, pBuffer, (TBlueAPI_Command)pCOMMsg->Command, (PBlueAPI_GATTAttributeWriteInd)&pCOMMsg->p.GATTAttributeWriteInd);
        break;
    case blueAPI_EventGATTCCCDInfo: /*----------------------------------*/
        LTPHandle_GATTCCCDInfo(pBTLtp, pBuffer, &pCOMMsg->p.GATTCCCDInfo);
        break;

    case blueAPI_EventGATTDiscoveryRsp: /*------------------------------*/
        LTPHandle_GATTDiscoveryRsp(pBTLtp, &pCOMMsg->p.GATTDiscoveryRsp);
        break;
    case blueAPI_EventGATTDiscoveryInd: /*------------------------------*/
        LTPHandle_GATTDiscoveryInd(pBTLtp, pBuffer, &pCOMMsg->p.GATTDiscoveryInd);
        break;
    case blueAPI_EventGATTAttributeReadRsp: /*--------------------------*/
        LTPHandle_GATTAttributeReadRsp(pBTLtp, pBuffer, &pCOMMsg->p.GATTAttributeReadRsp);
        break;
    case blueAPI_EventGATTAttributeWriteRsp: /*-------------------------*/
        LTPHandle_GATTAttributeWriteRsp(pBTLtp, &pCOMMsg->p.GATTAttributeWriteRsp);
        break;
    case blueAPI_EventGATTAttributeInd: /*------------------------------*/
        LTPHandle_GATTAttributeInd(pBTLtp, pBuffer, &pCOMMsg->p.GATTAttributeInd);
        break;
    case blueAPI_EventGATTAttributeNotificationInfo: /*-----------------*/
        LTPHandle_GATTAttributeNotificationInfo(pBTLtp, pBuffer, &pCOMMsg->p.GATTAttributeNotificationInfo);
        break;

    case blueAPI_EventGATTSecurityRsp: /*-------------------------------*/
        LTPHandle_GATTSecurityRsp(pBTLtp, &pCOMMsg->p.GATTSecurityRsp);
        break;

    case blueAPI_EventGATTServerStoreInd: /*----------------------------*/
        LTPHandle_GATTServerStoreInd(pBTLtp, &pCOMMsg->p.GATTServerStoreInd);
        break;

    case blueAPI_EventGATTMtuSizeInfo: /*-------------------------------*/
        LTPHandle_GATTMtuSizeInfo(pBTLtp, &pCOMMsg->p.GATTMtuSizeInfo);
        break;

    case blueAPI_EventLEAdvertiseRsp: /*--------------------------------*/
        LTPHandle_LEAdvertiseRsp(pBTLtp, &pCOMMsg->p.LEAdvertiseRsp);
        break;

    case blueAPI_EventSetRandomAddressRsp: /*--------------------------------*/
        LTPHandle_SetRandomAddressRsp(pBTLtp, &pCOMMsg->p.SetRandomAddressRsp);
        break;

    case blueAPI_EventLEAdvertiseParameterSetRsp: /*--------------------*/
        LTPHandle_LEAdvertiseParameterSetRsp(pBTLtp, &pCOMMsg->p.LEAdvertiseParameterSetRsp);
        break;
    case blueAPI_EventLEAdvertiseDataSetRsp: /*-------------------------*/
        LTPHandle_LEAdvertiseDataSetRsp(pBTLtp, &pCOMMsg->p.LEAdvertiseDataSetRsp);
        break;
    case blueAPI_EventLEScanRsp: /*-------------------------------------*/
        LTPHandle_LEScanRsp(pBTLtp, &pCOMMsg->p.LEScanRsp);
        break;
    case blueAPI_EventLEScanInfo: /*------------------------------------*/
        LTPHandle_LEScanInfo(pBTLtp, &pCOMMsg->p.LEScanInfo);
        break;
    case blueAPI_EventLEModifyWhitelistRsp: /*--------------------------*/
        LTPHandle_LEModifyWhitelistRsp(pBTLtp, &pCOMMsg->p.LEModifyWhitelistRsp);
        break;
    case blueAPI_EventLEConnectionUpdateRsp: /*-------------------------*/
        LTPHandle_LEConnectionUpdateRsp(pBTLtp, &pCOMMsg->p.LEConnectionUpdateRsp);
        break;
    case blueAPI_EventLEConnectionUpdateInd: /*-------------------------*/
        LTPHandle_LEConnectionUpdateInd(pBTLtp, &pCOMMsg->p.LEConnectionUpdateInd);
        break;
    case blueAPI_EventLEConnectionParameterInfo: /*---------------------*/
        LTPHandle_LEConnectionParameterInfo(pBTLtp, &pCOMMsg->p.LEConnectionParameterInfo);
        break;
#if 0
    case blueAPI_EventSetBleTxPowerRsp: /*---------------------*/
        LTPHandle_SetBleTxPowerRsp(pBTLtp, &pCOMMsg->p.SetBleTxPowerRsp);
        break;
    case blueAPI_EventSetDataLengthRsp: /*---------------------*/
        LTPHandle_SetDataLengthRsp(pBTLtp, &pCOMMsg->p.SetDataLengthRsp);
        break;
    case blueAPI_EventDataLengthChangeInfo: /*---------------------*/
        LTPHandle_DataLengthChangeInfo(pBTLtp, &pCOMMsg->p.DataLengthChangeInfo);
        break;
    case blueAPI_EventGATTAttributePrepareWriteInd:
        LTPHandle_GATTAttributePrepareWriteInd(pBTLtp, pBuffer, &pCOMMsg->p.GATTAttributeWriteInd);
        break;
    case blueAPI_EventGATTAttributePrepareWriteRsp:
        LTPHandle_GATTAttributePrepareWriteRsp(pBTLtp, pBuffer, &pCOMMsg->p.GATTAttributePrepareWriteRsp);
        break;
    case blueAPI_EventGATTAttributeExecuteWriteInd:
        LTPHandle_GATTAttributeExecuteWriteInd(pBTLtp, &pCOMMsg->p.GATTAttributeExecuteWriteInd);
        break;
    case blueAPI_EventGATTAttributeExecuteWriteRsp:
        LTPHandle_GATTAttributeExecuteWriteRsp(pBTLtp, &pCOMMsg->p.GATTAttributeExecuteWriteRsp);
        break;
#endif
    default: /*---------------------------------------------------------*/
#if BREDR_SUPPORT
        BTLTPHandleBREDRBLUE_API_MSG(pBTLtp, pBuffer, offset);
#else
        DBG_BUFFER(MODULE_LTP, LEVEL_ERROR, "BTLTPHandleBLUE_API_MSG, unknown pCOMMsg->Command = 0x%x", 1, pCOMMsg->Command);
#endif
        break;
    }

    return;
}

