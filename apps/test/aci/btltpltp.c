enum { __FILE_NUM__ = 0 };

#include <blueapi_types.h>
#include <blueapi.h>
#include "btltp.h"
#include "gatt.h"
#include "trace.h"
#include "aci_service_handle.h"
#if BREDR_SUPPORT
#include "btltp_br.h"
#endif

#define LTP_SOURCE_FILE_ID 0x82
//extern void  stSetTraceLevel(uint32_t traceLevel);
//extern uint32_t g_eflash_dbg;
void BTLTPHandleExitReq(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint8_t * pPara)
{
    uint8_t *       pInsert = pBTLtp->pMsgBuffer;
    uint16_t         pos     = 0;

    DBG_BUFFER(MODULE_LTP, LEVEL_INFO,  "LTP: >>ExitReq", 0);

    /* prepare confirmation                                                 */
    pInsert[pos++] = LTP_EXIT_RSP;
    pInsert[pos++] = 0x00;                                    /* copmsk     */
    NETSHORT2CHAR(&pInsert[pos], LTP_EXIT_RSP_LENGTH);
    pos += 2; /* msg length */
    pInsert[pos++] = LTP_CAUSE_NOT_SUPPORTED;

    /* send LTP messages                                                    */
    BTLTPTgtSendLTPMessage(pBTLtp, pBTLtp->pMsgBuffer, 0, pos);
}

void BTLTPHandleCreateMDLConf(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara)
{
   TBlueAPI_LinkConfigType linkConfigType   = blueAPI_LinkConfigGATT;
   uint16_t                 pos              = 0;
   uint8_t                 local_MDL_ID     = 0;                  /* mandatory */
   uint8_t                 maxTPDUusCredits = 0;                  /* default   */
   uint8_t                 cause;


   /* read mandatory parameters                                             */
   cause        = pPara[pos++];

   if(cause!=LTP_CAUSE_NOT_SUPPORTED)
   {  local_MDL_ID = pPara[pos++];
   }

   /* read optional parameters                                              */
   pos = 0;
   if(copmsk&LTP_CREATE_MDL_CNF_OPT_MASK_LINK_TYPE)
   {  linkConfigType = (TBlueAPI_LinkConfigType)pOpt[pos++];
   }
   if (copmsk & LTP_CREATE_MDL_CNF_OPT_MASK_MAX_TPDU_US_CREDITS)
   {  maxTPDUusCredits = pOpt[pos++];
   }
   if(linkConfigType   == blueAPI_LinkConfigGATT)
   {
        blueAPI_CreateMDLConf(local_MDL_ID,
                         maxTPDUusCredits,
                         BTLTPConvertLTPtoCOMcause(cause)
                         );
   }
#if 0
   else
   {
       blueAPI_CreateLegacyMDLConf(local_MDL_ID,
                         linkConfigType,
                         maxTPDUusCredits,
                         BTLTPConvertLTPtoCOMcause(cause)
                         );
   }
#endif
}

void BTLTPHandleDisconnectMDLConf(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara)
{
    uint8_t          local_MDL_ID;
    uint16_t          pos          = 0;


    /* read mandatory parameters                                             */
    local_MDL_ID = pPara[pos++];


    /* read optional parameters                                              */
    /* non */

    blueAPI_DisconnectMDLConf(
        local_MDL_ID
    );
}

void BTLTPHandleDisconnectMDLReq(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara)
{
    uint8_t          local_MDL_ID = 0;                             /* mandatory */
    uint16_t          pos          = 0;
    uint8_t          cause;


    /* read mandatory parameters                                             */
    cause        = pPara[pos++];

    if (cause != LTP_CAUSE_NOT_SUPPORTED)
    {
        local_MDL_ID = pPara[pos++];
    }

    /* read optional parameters                                              */
    /* non */

    blueAPI_DisconnectMDLReq(
        local_MDL_ID,
        BTLTPConvertLTPtoCOMcause(cause)
    );
}

void BTLTPHandleResetReq(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara)
{
    BOOL result = FALSE;

    DBG_BUFFER(MODULE_LTP, LEVEL_INFO, "LTP: >>ResetReq", 0);

    pBTLtp->pBufferAction = BTLTPAllocateAction(pBTLtp);
    if (pBTLtp->pBufferAction)
    {
        pBTLtp->pBufferAction->Action = btltpActionReset;

        result = LTPLibSendResetRsp(&pBTLtp->LTPLib, BTLTP_DEFAULT_COPMSK, NULL, LTP_CAUSE_SUCCESS);

        if (!result)
        {
            pBTLtp->pBufferAction->Action = btltpActionNotUsed;
            pBTLtp->pBufferAction         = NULL;
        }
    }

    if (!result)
    {
        THandle      handle;
        TBTLtpAction directAction;
        directAction.Action = btltpActionReset;

        handle.lpHandle = (PVOID)&directAction;
        BTLTPBufferCallback(handle);
    }
}

void BTLTPHandlePasskeyRequestCnf(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara)
{
    uint16_t   pos = 0;
    uint8_t   cause;                                               /* mandatory */
    uint8_t * rem_BD = NULL;                                       /* mandatory */


    /* read mandatory parameters                                             */
    cause = pPara[pos++];

    if (cause != LTP_CAUSE_NOT_SUPPORTED)
    {
        rem_BD = &pPara[pos];
        pos += 6;
    }

    /* read optional parameters                                              */
    /* non */

    blueAPI_UserPasskeyReqConf(
        rem_BD,
        BTLTPConvertLTPtoCOMcause(cause)
    );
}

void BTLTPHandleOOBRequestCnf(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara)
{
    uint16_t   pos = 0;
    uint8_t * rem_BD = NULL;                                       /* mandatory */
    uint8_t * C      = NULL;                                       /* mandatory */
    uint8_t   cause;                                               /* mandatory */


    /* read mandatory parameters                                             */
    cause = pPara[pos++];

    if (cause != LTP_CAUSE_NOT_SUPPORTED)
    {
        rem_BD = &pPara[pos];
        pos += 6;
        C      = &pPara[pos];
        pos += 16;
    }
    /* read optional parameters                                              */
    /* non */

    blueAPI_RemoteOOBDataReqConf(
        rem_BD,
        C,
        BTLTPConvertLTPtoCOMcause(cause)
    );
}

void BTLTPHandleAuthResultExtCnf(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara)
{
    uint16_t   pos = 0;
    uint8_t   cause;
    uint8_t * rem_BD      = NULL;
    uint8_t   rem_BD_Type = blueAPI_RemoteBDTypeClassic;


    /* read mandatory parameters                                             */
    cause = pPara[pos++];

    if (cause != LTP_CAUSE_NOT_SUPPORTED)
    {
        rem_BD      = &pPara[pos];
        pos += 6;
        rem_BD_Type = pPara[pos++];
    }

    /* read optional parameters                                              */
    /* non */

    blueAPI_AuthResultConf(
        rem_BD,
        (TBlueAPI_RemoteBDType)rem_BD_Type,
        BTLTPConvertLTPtoCOMcause(cause)
    );
}

void BTLTPHandleAuthResultRequestExtCnf(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara)
{
    uint16_t          pos            = 0;
    uint8_t *        rem_BD         = NULL;
    uint8_t          rem_BD_Type    = blueAPI_RemoteBDTypeClassic;
    uint8_t *        linkKey        = NULL;
    uint8_t          linkKeyLength  = 0;
    uint8_t          keyType        = 0;
    uint8_t          cause;
    uint16_t          restartHandle  = 0x0000;                      /* optional  */


    /* read mandatory parameters                                             */
    cause = pPara[pos++];

    if (cause != LTP_CAUSE_NOT_SUPPORTED)
    {
        rem_BD        = &pPara[pos];
        pos += 6;
        rem_BD_Type   = pPara[pos++];
        keyType       = pPara[pos++];
        linkKeyLength = lenPara - pos;
        linkKey       = &pPara[pos];
        pos += linkKeyLength;
    }

    /* read optional parameters                                              */
    pos = 0;

    if (copmsk & LTP_AUTH_RESULT_REQUEST_EXT_CNF_OPT_MASK_RESTART_HANDLE)
    {
        restartHandle = NETCHAR2SHORT(&pOpt[pos]);
        pos += 2;
    }

    blueAPI_AuthResultRequestConf(
        rem_BD,
        (TBlueAPI_RemoteBDType)rem_BD_Type,
        linkKeyLength,
        linkKey,
        (TBlueAPI_LinkKeyType)keyType,
        restartHandle,
        BTLTPConvertLTPtoCOMcause(cause)
    );
}

void BTLTPHandlePairableModeSetReq(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara)
{
    uint16_t pos = 0;
    BOOL                   enablePairableMode;
    TBlueAPI_AuthRequirements AuthRequirements;
    TBlueAPI_IOCapabilities   IOCapabilities;
    BOOL                   remoteOOBDataPresent;
#if BREDR_SUPPORT
    BOOL                   isBluetoothMode = false;
    TBlueAPI_BluetoothMode bluetoothMode;
#endif
    


    /* read mandatory parameters                                             */
    enablePairableMode   = (BOOL)pPara[pos++];
    AuthRequirements     = (TBlueAPI_AuthRequirements)pPara[pos++];
    IOCapabilities       = (TBlueAPI_IOCapabilities)pPara[pos++];
    remoteOOBDataPresent = (BOOL)pPara[pos++];

    /* read optional parameters                                              */
    /* non */
#if BREDR_SUPPORT
    if (copmsk & LTP_PAIRABLE_MODE_SET_REQ_OPT_MASK_BLUETOOTH_MODE)
    {
        isBluetoothMode = true;
	 pos = 0;
        bluetoothMode = (TBlueAPI_BluetoothMode)pOpt[pos];
    }

    if(isBluetoothMode)
    {
        blueAPI_ExtendPairableModeSetReq(
            enablePairableMode,
            bluetoothMode,
            AuthRequirements,
            IOCapabilities,
            remoteOOBDataPresent
        );
    }
    else
#endif
    {
        blueAPI_PairableModeSetReq(
            enablePairableMode,
            AuthRequirements,
            IOCapabilities,
            remoteOOBDataPresent
        );
    }
}

void BTLTPHandlePasskeyReqReplyReq(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara)
{
    uint16_t              pos       = 0;
    TBlueAPI_Cause    cause;
    uint8_t *            rem_BD;
    uint32_t             passKey;


    /* read mandatory parameters                                             */
    cause    = (TBlueAPI_Cause)pPara[pos++];
    rem_BD   = &pPara[pos];
    pos += 6;
    passKey  = NETCHAR2LONG(&pPara[pos]);
    pos += 4;

    /* read optional parameters                                              */
    /* non */

    blueAPI_UserPasskeyReqReplyReq(
        rem_BD,
        passKey,
        cause
    );
}
#define LTP_CauseServiceDatabaseNotExist 0xF1
#define LTP_CauseServiceDatabaseError 0xF2

void BTLTPHandleGATTServiceRegisterReq(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt,
                                       uint16_t lenPara, uint8_t * pPara)
{
    uint16_t pos = 0;
#if ACI_EN
    uint32_t pService;
    PGattServiceTable pServiceTable;
    uint8_t cause;
#endif
    uint16_t nbrOfAttrib;
    uint8_t serviceID = 0;


#if ACI_EN
    /* read mandatory parameters                                             */
    pService     = NETCHAR2LONG(&pPara[pos]);
#endif
    pos += 4;
    nbrOfAttrib = NETCHAR2SHORT(&pPara[pos]);

    /* read optional parameters                                              */
    pos = 0;

    if (copmsk & LTP_GATT_SERVICE_REGISTER_REQ_OPT_MASK_SERVICE_ID)
    {
        serviceID = pOpt[pos];
    }


#if ACI_EN
    pServiceTable = BTACILookupGATTServiceTableByHostService(pBTLtp, pService);
    if (pServiceTable == NULL)
    {
        cause = LTP_CauseServiceDatabaseNotExist;
        DBG_BUFFER(MODULE_LTP, LEVEL_INFO, "BTLTPHandleGATTServiceRegisterReq 0xF1 0x%x!", 1, pService);
        LTPLibSendGATTServiceRegisterRsp(&pBTLtp->LTPLib,
                                         BTLTP_DEFAULT_COPMSK,
                                         NULL,
                                         cause,
                                         0,
                                         0
                                        );
    }
    else
    {
        uint8_t data = *(uint8_t*)pServiceTable->pService;
        if ((nbrOfAttrib == pServiceTable->nbrOfAttrib) && (pServiceTable->pService != NULL) && (data != 0xFF))
        {
            pBTLtp->service_register_idx = pServiceTable->self_idx;
            blueAPI_GATTServiceRegisterReq(
                nbrOfAttrib,
                pServiceTable->pService
            );
        }
        else
        {
            cause = LTP_CauseServiceDatabaseError;
            LTPLibSendGATTServiceRegisterRsp(&pBTLtp->LTPLib,
                                             BTLTP_DEFAULT_COPMSK,
                                             NULL,
                                             cause,
                                             0,
                                             0
                                            );
        }
    }
#else

#if BREDR_SUPPORT
{
    PVOID pSer = NULL;
    pSer = blueAPI_GATTServiceGet((GATTServiceID)serviceID, &nbrOfAttrib);

    if (pSer != NULL)
    {
	    blueAPI_GATTServiceRegisterReq(nbrOfAttrib,
                             pSer
                             );
    }
}
#else
    nbrOfAttrib = GattdFindMeProfileSize / sizeof(TAttribAppl);
    blueAPI_GATTServiceRegisterReq(
        nbrOfAttrib,
        (void*)GattdFindMeProfile
    );
#endif
#endif
}

void BTLTPHandleGATTAttributeUpdateReq(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt,
                                       uint16_t lenPara, uint8_t * pPara)
{
    uint16_t pos = 0;
    uint8_t serviceHandle;
    uint16_t attribIndex;
    union
    {
        uint32_t   d;
        void   *p;
    } requestHandle;

    uint16_t wOffset = 0;
    uint16_t ds_pool_id = 0;
    uint8_t* pBuffer = NULL;

    /* read mandatory parameters                                             */
    serviceHandle   = pPara[pos++];
    requestHandle.d = NETCHAR2LONG(&pPara[pos]);
    pos += 4;
    attribIndex     = NETCHAR2SHORT(&pPara[pos]);
    pos += 2;

    /* read optional parameters                                              */
    /* none */
    PBTLtpMDLContext p_mdl_context = BTLTPFindAMDLConnected(pBTLtp);

    if (p_mdl_context)
    {
        wOffset = p_mdl_context->dsDataOffset + 3;
        ds_pool_id = p_mdl_context->dsPoolID;
    }

    if (blueAPI_CauseSuccess == blueAPI_BufferGet(ds_pool_id,  lenPara - pos, wOffset, (void**)&pBuffer))
    {
        memcpy(pBuffer + wOffset, pPara + pos, lenPara - pos);
        if (!blueAPI_GATTAttributeUpdateReq(pBuffer,
                                            BTLTPLookupGATTServiceHandle(pBTLtp, serviceHandle),
                                            requestHandle.p,
                                            attribIndex,
                                            (lenPara - pos),
                                            wOffset
                                           ))
        {
            blueAPI_BufferRelease(pBuffer);
        }
    }
    else
    {
        DBG_BUFFER(MODULE_LTP, LEVEL_ERROR, "BTLTPHandleGATTAttributeUpdateReq blueAPI_BufferGet failed!", 0);
    }

    return;
}

void BTLTPHandleGATTAttributeUpdateStatusCnf(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt,
        uint16_t lenPara, uint8_t * pPara)
{
    uint16_t pos = 0;
    uint8_t cause;
    uint8_t serviceHandle = 0x00;
    uint16_t attribIndex   = 0x00;
    union
    {
        uint32_t   d;
        void   *p;
    } requestHandle;

    requestHandle.d = 0;

    /* read mandatory parameters                                             */
    cause           = pPara[pos++];
    if (cause != LTP_CAUSE_NOT_SUPPORTED)
    {
        serviceHandle   = pPara[pos++];
        requestHandle.d = NETCHAR2LONG(&pPara[pos]);
        pos += 4;
        attribIndex     = NETCHAR2SHORT(&pPara[pos]);
        pos += 2;
    }

    /* read optional parameters                                              */
    /* none */

    blueAPI_GATTAttributeUpdateStatusConf(
        BTLTPLookupGATTServiceHandle(pBTLtp, serviceHandle),
        requestHandle.p,
        attribIndex
    );
}

void BTLTPHandleGATTAttributeReadCnf(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt,
                                     uint16_t lenPara, uint8_t * pPara)
{
    uint16_t pos = 0;
    uint8_t cause;
    uint16_t subCause       = 0x0000;
    uint8_t loc_MDL_ID     = 0x00;
    uint8_t serviceHandle  = 0x00;
    uint16_t attribIndex    = 0x00;

    PBTLtpMDLContext pMDLContext = NULL;
    uint16_t wOffset = 0;
    void* pBuffer = NULL;
    uint16_t ds_pool_id = 0;

    /* read mandatory parameters                                             */
    cause           = pPara[pos++];
    if (cause != LTP_CAUSE_NOT_SUPPORTED)
    {
        subCause      = NETCHAR2SHORT(&pPara[pos]);
        pos += 2;
        loc_MDL_ID    = pPara[pos++];
        serviceHandle = pPara[pos++];
        attribIndex   = NETCHAR2SHORT(&pPara[pos]);
        pos += 2;
    }

    /* read optional parameters                                              */
    /* none */
    pMDLContext = BTLTPFindMDLContext(pBTLtp, loc_MDL_ID);


    if (NULL != pMDLContext)
    {
        wOffset = pMDLContext->dsDataOffset;
        ds_pool_id = pMDLContext->dsPoolID;
    }

    /* none */
    if (blueAPI_CauseSuccess == blueAPI_BufferGet(ds_pool_id,  lenPara - pos, wOffset, (void**)&pBuffer))
    {
        memcpy(((uint8_t *)pBuffer) + wOffset, pPara + pos, lenPara - pos);
        if (!blueAPI_GATTAttributeReadConf(pBuffer,
                                           loc_MDL_ID,
                                           BTLTPLookupGATTServiceHandle(pBTLtp, serviceHandle),
                                           (TBlueAPI_Cause)cause,
                                           subCause,
                                           attribIndex,
                                           (lenPara - pos),
                                           wOffset
                                          ))
        {
            blueAPI_BufferRelease(pBuffer);
        }
    }
    else
    {
        DBG_BUFFER(MODULE_LTP, LEVEL_ERROR, "BTLTPHandleGATTAttributeReadCnf blueAPI_BufferGet failed!", 0);
    }

    return;
}

void BTLTPHandleGATTAttributeWriteCnf(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt,
                                      uint16_t lenPara, uint8_t * pPara)
{
    uint16_t pos = 0;
    uint8_t cause;
    uint16_t subCause       = 0x0000;
    uint8_t loc_MDL_ID     = 0x00;
    uint8_t serviceHandle  = 0x00;
    uint16_t attribIndex    = 0x0000;

    /* read mandatory parameters                                             */
    cause           = pPara[pos++];
    if (cause != LTP_CAUSE_NOT_SUPPORTED)
    {
        subCause      = NETCHAR2SHORT(&pPara[pos]);
        pos += 2;
        loc_MDL_ID    = pPara[pos++];
        serviceHandle = pPara[pos++];
        attribIndex   = NETCHAR2SHORT(&pPara[pos]);
        pos += 2;
    }

    /* read optional parameters                                              */
    /* none */

    blueAPI_GATTAttributeWriteConf(
        loc_MDL_ID,
        BTLTPLookupGATTServiceHandle(pBTLtp, serviceHandle),
        (TBlueAPI_Cause)cause,
        subCause,
        attribIndex
    );
}

void BTLTPHandleGATTServerStoreCnf(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt,
                                   uint16_t lenPara, uint8_t * pPara)
{
    uint16_t pos = 0;
    uint8_t cause;
    uint8_t opCode         = 0x00;
    uint8_t * rem_BD       = NULL;
    uint8_t rem_BD_Type    = 0x00;
    uint16_t restartHandle  = 0x0000;

    /* read mandatory parameters                                             */
    cause           = pPara[pos++];
    if (cause != LTP_CAUSE_NOT_SUPPORTED)
    {
        opCode        = pPara[pos++];
        rem_BD        = &pPara[pos];
        pos += 6;
        rem_BD_Type   = pPara[pos++];
        restartHandle = NETCHAR2SHORT(&pPara[pos]);
        pos += 2;
    }

    /* read optional parameters                                              */
    /* none */

    blueAPI_GATTServerStoreConf(
        (TBlueAPI_GATTStoreOpCode)opCode,
        rem_BD,
        (TBlueAPI_RemoteBDType)rem_BD_Type,
        restartHandle,
        (lenPara - pos),
        (pPara + pos),
        (TBlueAPI_Cause)cause
    );
}

void BTLTPHandleConnectGATTMDLReq(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt,
                                  uint16_t lenPara, uint8_t * pPara)
{
    uint16_t pos = 0;
    uint8_t * rem_BD;
    uint8_t rem_BD_Type;
    uint16_t scanInterval;
    uint16_t scanWindow;
    uint16_t scanTimeout;
    uint16_t connIntervalMin;
    uint16_t connIntervalMax;
    uint16_t connLatency;
    uint16_t supervisionTimeout;
    uint8_t local_BD_Type;
    uint16_t CE_Length;

    /* read mandatory parameters                                             */
    rem_BD              = &pPara[pos];
    pos += 6;
    rem_BD_Type         = pPara[pos++];
    //loc_MDEP_ID         = pPara[pos++];
    pos++;
    scanInterval        = NETCHAR2SHORT(&pPara[pos]);
    pos += 2;
    scanWindow          = NETCHAR2SHORT(&pPara[pos]);
    pos += 2;
    scanTimeout         = NETCHAR2SHORT(&pPara[pos]);
    pos += 2;
    connIntervalMin     = NETCHAR2SHORT(&pPara[pos]);
    pos += 2;
    connIntervalMax     = NETCHAR2SHORT(&pPara[pos]);
    pos += 2;
    connLatency         = NETCHAR2SHORT(&pPara[pos]);
    pos += 2;
    supervisionTimeout  = NETCHAR2SHORT(&pPara[pos]);
    pos += 2;
    local_BD_Type       = pPara[pos++];
    CE_Length           = NETCHAR2SHORT(&pPara[pos]);
    pos += 2;

    /* read optional parameters                                              */
    /* none */

    /* Allocate Context for this loc_MDEP_ID */
    BTLTPAllocateMDLContext(pBTLtp, 0x00, 0x01, blueAPI_LinkConfigGATT);

    blueAPI_ConnectGATTMDLReq(
        rem_BD,
        (TBlueAPI_RemoteBDType)rem_BD_Type,
        (TBlueAPI_LocalBDType)local_BD_Type,
        scanInterval,
        scanWindow,
        scanTimeout,
        connIntervalMin,
        connIntervalMax,
        connLatency,
        supervisionTimeout,
        CE_Length
    );
}

void BTLTPHandleGATTDiscoveryReq(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt,
                                 uint16_t lenPara, uint8_t * pPara)
{
    uint16_t pos = 0;
    uint8_t loc_MDL_ID;
    uint8_t discoveryType;
    uint16_t startHandle;
    uint16_t endHandle;
    uint16_t uuid16 = 0;
    uint8_t * pUUID128 = NULL;

    /* read mandatory parameters                                             */
    loc_MDL_ID    = pPara[pos++];
    discoveryType = pPara[pos++];
    startHandle   = NETCHAR2SHORT(&pPara[pos]);
    pos += 2;
    endHandle     = NETCHAR2SHORT(&pPara[pos]);
    pos += 2;

    if (lenPara == (pos + 2))
    {
        uuid16      = NETCHAR2SHORT(&pPara[pos]);
        pos += 2;
    }
    else
    {
        pUUID128    = &pPara[pos];
    }

    /* read optional parameters                                              */
    /* none */

    blueAPI_GATTDiscoveryReq(
        loc_MDL_ID,
        (TBlueAPI_GATTDiscoveryType)discoveryType,
        startHandle,
        endHandle,
        uuid16,
        pUUID128
    );
}

void BTLTPHandleGATTDiscoveryCnf(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt,
                                 uint16_t lenPara, uint8_t * pPara)
{
    uint16_t pos = 0;
    uint8_t cause;
    uint8_t loc_MDL_ID     = 0x00;
    uint8_t discoveryType  = 0x00;
    uint16_t startHandle    = 0x0000;
    uint16_t endHandle      = 0x0000;

    /* read mandatory parameters                                             */
    cause           = pPara[pos++];
    if (cause != LTP_CAUSE_NOT_SUPPORTED)
    {
        loc_MDL_ID    = pPara[pos++];
        discoveryType = pPara[pos++];
        startHandle   = NETCHAR2SHORT(&pPara[pos]);
        pos += 2;
        endHandle     = NETCHAR2SHORT(&pPara[pos]);
        pos += 2;
    }

    /* read optional parameters                                              */
    /* none */

    blueAPI_GATTDiscoveryConf(
        loc_MDL_ID,
        (TBlueAPI_GATTDiscoveryType)discoveryType,
        startHandle,
        endHandle
    );
}

void BTLTPHandleGATTAttributeReadReq(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt,
                                     uint16_t lenPara, uint8_t * pPara)
{
    uint16_t pos = 0;
    uint8_t loc_MDL_ID;
    uint8_t readType;
    uint16_t readOffset;
    uint16_t startHandle;
    uint16_t endHandle;
    uint16_t uuid16 = 0;
    uint8_t * pUUID128 = NULL;

    /* read mandatory parameters                                             */
    loc_MDL_ID  = pPara[pos++];
    readType    = pPara[pos++];
    readOffset  = NETCHAR2SHORT(&pPara[pos]);
    pos += 2;
    startHandle = NETCHAR2SHORT(&pPara[pos]);
    pos += 2;
    endHandle   = NETCHAR2SHORT(&pPara[pos]);
    pos += 2;

    if (lenPara == (pos + 2))
    {
        uuid16    = NETCHAR2SHORT(&pPara[pos]);
        pos += 2;
    }
    else
    {
        pUUID128  = &pPara[pos];
    }

    /* read optional parameters                                              */
    /* none */

    blueAPI_GATTAttributeReadReq(
        loc_MDL_ID,
        (TBlueAPI_GATTReadType)readType,
        readOffset,
        startHandle,
        endHandle,
        uuid16,
        pUUID128
    );
}

void BTLTPHandleGATTAttributeWriteReq(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt,
                                      uint16_t lenPara, uint8_t * pPara)
{
    uint16_t pos = 0;
    uint8_t loc_MDL_ID;
    uint8_t writeType;
    uint16_t attribHandle;

    PBTLtpMDLContext pMDLContext = NULL;
    uint16_t wOffset = 0;
    void* pBuffer = NULL;
    uint16_t ds_pool_id = 0;

    /* read mandatory parameters                                             */
    loc_MDL_ID    = pPara[pos++];
    writeType     = pPara[pos++];
    attribHandle  = NETCHAR2SHORT(&pPara[pos]);
    pos += 2;

    /* read optional parameters */
    pMDLContext = BTLTPFindMDLContext(pBTLtp, loc_MDL_ID);

    if (NULL != pMDLContext)
    {
        wOffset = pMDLContext->dsDataOffset + 3;
        ds_pool_id = pMDLContext->dsPoolID;
    }

    if (blueAPI_CauseSuccess == blueAPI_BufferGet(ds_pool_id,  lenPara - pos, wOffset, (void**)&pBuffer))
    {
        memcpy(((uint8_t *)pBuffer) + wOffset, pPara + pos, lenPara - pos);
        if (!blueAPI_GATTAttributeWriteReq(pBuffer,
                                           loc_MDL_ID,
                                           (TBlueAPI_GATTWriteType)writeType,
                                           attribHandle,
                                           (lenPara - pos),
                                           wOffset
                                          ))
        {
            blueAPI_BufferRelease(pBuffer);
        }
    }
    else
    {
        DBG_BUFFER(MODULE_LTP, LEVEL_ERROR, "BTLTPHandleGATTAttributeWriteReq blueAPI_BufferGet failed!", 0);
    }

    return;
}

void BTLTPHandleGATTAttributeCnf(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt,
                                 uint16_t lenPara, uint8_t * pPara)
{
    uint16_t pos = 0;
    uint8_t cause;
    uint8_t loc_MDL_ID = 0x00;

    /* read mandatory parameters                                             */
    cause         = pPara[pos++];
    if (cause != LTP_CAUSE_NOT_SUPPORTED)
    {
        loc_MDL_ID  = pPara[pos++];
    }

    /* read optional parameters                                              */
    /* none */

    blueAPI_GATTAttributeConf(
        loc_MDL_ID
    );
}

void BTLTPHandleGATTSecurityReq(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt,
                                uint16_t lenPara, uint8_t * pPara)
{
    uint16_t pos = 0;
    uint8_t loc_MDL_ID;
    uint16_t requirements;
    uint8_t minKeySize;

    /* read mandatory parameters                                             */
    loc_MDL_ID    = pPara[pos++];
    requirements  = NETCHAR2SHORT(&pPara[pos]);
    pos += 2;
    minKeySize    = pPara[pos++];

    /* read optional parameters                                              */
    /* none */

    blueAPI_GATTSecurityReq(
        loc_MDL_ID,
        requirements,
        minKeySize
    );
}

void BTLTPHandleLEAdvertiseReq(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt,
                               uint16_t lenPara, uint8_t * pPara)
{
    uint16_t pos = 0;
    uint8_t advMode;


    /* read mandatory parameters                                             */
    advMode     = pPara[pos++];

    /* read optional parameters                                              */
    /* none */

    blueAPI_LEAdvertiseReq(
        (TBlueAPI_LEAdvMode)advMode);

}

void BTLTPHandleSetRandomAddressReq(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt,
                                    uint16_t lenPara, uint8_t * pPara)
{
    uint16_t pos = 1;
    uint8_t * rem_BD;

    /* read mandatory parameters                                             */
    rem_BD      = &pPara[pos];
    pos += 6;

    /* read optional parameters                                              */
    /* none */

    blueAPI_SetRandomAddressReq(rem_BD);

}

void BTLTPHandleLEAdvertiseParameterSetReq(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt,
        uint16_t lenPara, uint8_t * pPara)
{
    uint16_t pos = 0;
    uint8_t advType;
    uint8_t filterScanReq;
    uint8_t filterConnectReq;
    uint16_t minAdvInterval;
    uint16_t maxAdvInterval;
    uint8_t local_BD_Type;
    uint8_t * rem_BD;
    uint8_t rem_BD_Type;

    /* read mandatory parameters                                             */
    advType           = pPara[pos++];
    filterScanReq     = pPara[pos++];
    filterConnectReq  = pPara[pos++];
    minAdvInterval    = NETCHAR2SHORT(&pPara[pos]);
    pos += 2;
    maxAdvInterval    = NETCHAR2SHORT(&pPara[pos]);
    pos += 2;
    local_BD_Type     = pPara[pos++];
    rem_BD            = &pPara[pos];
    pos += 6;
    rem_BD_Type = pPara[pos++];

    /* read optional parameters                                              */
    /* none */

    blueAPI_LEAdvertiseParameterSetReq(
        (TBlueAPI_LEAdvType)advType,
        (TBlueAPI_LEFilterPolicy)filterScanReq,
        (TBlueAPI_LEFilterPolicy)filterConnectReq,
        minAdvInterval,
        maxAdvInterval,
        (TBlueAPI_LocalBDType)local_BD_Type,
        rem_BD,
        (TBlueAPI_RemoteBDType)rem_BD_Type
    );
}

void BTLTPHandleLEAdvertiseDataSetReq(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt,
                                      uint16_t lenPara, uint8_t * pPara)
{
    uint16_t pos = 0;
    uint8_t dataType = 0;

    /* read mandatory parameters                                             */
    dataType = pPara[pos++];

    /* read optional parameters                                              */
    /* none */

    blueAPI_LEAdvertiseDataSetReq(
        (TBlueAPI_LEDataType)dataType,
        (lenPara - pos),
        (pPara + pos)
    );

}

void BTLTPHandleLEScanReq(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt,
                          uint16_t lenPara, uint8_t * pPara)
{
    uint16_t pos = 0;
    uint8_t scanMode;
    uint16_t scanInterval;
    uint16_t scanWindow;
    uint8_t filterPolicy;
    uint8_t filterDuplicates;
    uint8_t localBDType;

    /* read mandatory parameters                                             */
    scanMode          = pPara[pos++];
    scanInterval      = NETCHAR2SHORT(&pPara[pos]);
    pos += 2;
    scanWindow        = NETCHAR2SHORT(&pPara[pos]);
    pos += 2;
    filterPolicy      = pPara[pos++];
    filterDuplicates  = pPara[pos++];
    localBDType  = pPara[pos++];

    /* read optional parameters                                              */
    /* none */

    blueAPI_LEScanReq(
        (TBlueAPI_LEScanMode)scanMode,
        scanInterval,
        scanWindow,
        (TBlueAPI_LEFilterPolicy)filterPolicy,
        (TBlueAPI_LocalBDType)localBDType,
        filterDuplicates
    );
} /* BTLTPHandleLEScanReq */

void BTLTPHandleLEModifyWhitelistReq(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt,
                                     uint16_t lenPara, uint8_t * pPara)
{
    uint16_t pos = 0;
    uint8_t whitelistOp;
    uint8_t * rem_BD;
    uint8_t rem_BD_Type;

    /* read mandatory parameters                                             */
    whitelistOp = pPara[pos++];
    rem_BD      = &pPara[pos];
    pos += 6;
    rem_BD_Type = pPara[pos++];

    /* read optional parameters                                              */
    /* none */

    blueAPI_LEModifyWhitelistReq(
        (TBlueAPI_LEWhitelistOp)whitelistOp,
        rem_BD,
        (TBlueAPI_RemoteBDType)rem_BD_Type
    );
}

void BTLTPHandleLEConnectionUpdateReq(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt,
                                      uint16_t lenPara, uint8_t * pPara)
{
    uint16_t pos = 0;
    uint8_t loc_MDL_ID;
    uint16_t connIntervalMin;
    uint16_t connIntervalMax;
    uint16_t connLatency;
    uint16_t supervisionTimeout;

    /* read mandatory parameters                                             */
    loc_MDL_ID          = pPara[pos++];
    connIntervalMin     = NETCHAR2SHORT(&pPara[pos]);
    pos += 2;
    connIntervalMax     = NETCHAR2SHORT(&pPara[pos]);
    pos += 2;
    connLatency         = NETCHAR2SHORT(&pPara[pos]);
    pos += 2;
    supervisionTimeout  = NETCHAR2SHORT(&pPara[pos]);
    pos += 2;

    /* read optional parameters                                              */
    /* none */

    blueAPI_LEConnectionUpdateReq(
        loc_MDL_ID,
        connIntervalMin,
        connIntervalMax,
        connLatency,
        supervisionTimeout
    );
}

void BTLTPHandleLEConnectionUpdateCnf(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt,
                                      uint16_t lenPara, uint8_t * pPara)
{
    uint16_t pos = 0;
    uint8_t cause;
    uint8_t loc_MDL_ID = 0x00;

    /* read mandatory parameters                                             */
    cause         = pPara[pos++];
    if (cause != LTP_CAUSE_NOT_SUPPORTED)
    {
        loc_MDL_ID  = pPara[pos++];
    }

    /* read optional parameters                                              */
    /* none */

    blueAPI_LEConnectionUpdateConf(
        loc_MDL_ID,
        (TBlueAPI_Cause)cause
    );
}

void BTLTPHandleDeviceNameSetReq(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara)
{
    uint8_t dev_name[40];
    uint16_t pos = 1;
    memcpy(dev_name, pPara + pos, lenPara - pos);
    blueAPI_DeviceConfigDeviceNameSetReq( dev_name);
}

void BTLTPHandleDeviceConfigSecuritySetReq(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara)
{
    uint16_t                    pos = 1;
    uint32_t                   leFixedDisplayValue;


    leFixedDisplayValue = NETCHAR2LONG(&pPara[pos]);


    blueAPI_DeviceConfigSecuritySetReq(leFixedDisplayValue);
}

void BTLTPHandleDeviceConfigStoreSetReq(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara)
{
    uint16_t                    pos = 1;
    TBlueAPI_StoreBondModes storeBondMode;
    uint8_t                    storeBondSize;

    /* read mandatory parameters                                             */
    storeBondMode       = (TBlueAPI_StoreBondModes)pPara[pos++];
    storeBondSize       = pPara[pos++];

    blueAPI_DeviceConfigStoreSetReq(storeBondMode, storeBondSize);
}

void BTLTPHandleDeviceConfigAppearanceSetReq(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara)
{
    uint16_t                    pos = 1;
    uint16_t                    appearance;

    /* read mandatory parameters                                             */
    appearance     = NETCHAR2SHORT(&pPara[pos]);

    blueAPI_DeviceConfigAppearanceSetReq(appearance);
}

void BTLTPHandleDeviceConfigPerPrefConnParamSetReq(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara)
{
    uint16_t                    pos = 1;
    uint16_t connIntervalMin;
    uint16_t connIntervalMax;
    uint16_t slaveLatency;
    uint16_t supervisionTimeout;

    /* read mandatory parameters                                             */
    connIntervalMin     = NETCHAR2SHORT(&pPara[pos]);
    pos += 2;
    connIntervalMax     = NETCHAR2SHORT(&pPara[pos]);
    pos += 2;
    slaveLatency         = NETCHAR2SHORT(&pPara[pos]);
    pos += 2;
    supervisionTimeout  = NETCHAR2SHORT(&pPara[pos]);
    pos += 2;

    blueAPI_DeviceConfigPerPrefConnParamSetReq(connIntervalMin, connIntervalMax, slaveLatency, supervisionTimeout);
}
#if 0
void BTLTPHandleSetLETxPowerReq(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara)
{
    uint16_t                    pos = 1;
    uint8_t                    tx_power;

    /* read mandatory parameters                                             */
    tx_power       = pPara[pos];

    blueAPI_SetBleTxPowerReq(tx_power);
}

void BTLTPHandleSetDataLengthReq(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara)
{
    uint16_t                    pos = 1;
    uint16_t                    txOctets;
    uint16_t                    txTime;
    uint8_t                    loc_MDL_ID = 0x00;

    /* read mandatory parameters                                             */
    loc_MDL_ID       = pPara[pos++];
    txOctets     = NETCHAR2SHORT(&pPara[pos]);
    pos += 2;
    txTime       = NETCHAR2SHORT(&pPara[pos]);

    blueAPI_SetDataLengthReq(loc_MDL_ID, txOctets, txTime);
}
#endif

void BTLTPHandleDownloadServiceDatabaseReq(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara)
{
    uint16_t    pos = 1;
    uint8_t    cmd;
    uint32_t host_service = 0;
    PGattServiceTable pServiceTable = NULL;
    LoadDatabaseCause    cause = Load_CauseSuccess;
    uint16_t    receive_pos;

    cmd = pPara[pos++];
    host_service = NETCHAR2LONG(&pPara[pos]);
    pos += 4;
    DBG_BUFFER(MODULE_LTP, LEVEL_INFO,  "BTLTPHandleDownloadServiceDatabaseReq cmd %d 0x%x\n", 2, cmd,host_service);
    switch (cmd)
    {
    case ACI_SERVICE_DATABASE_START:
        {
            uint16_t  database_len;
            database_len = NETCHAR2SHORT(&pPara[pos]);
            pos += 2;
            pServiceTable = BTACILookupGATTServiceTableByHostService(pBTLtp, host_service);
            if (pServiceTable == NULL)
            {
                pServiceTable = BTACIAllocateGATTServiceHandle(pBTLtp);
                if (pServiceTable == NULL)
                {
                    DBG_BUFFER(MODULE_LTP, LEVEL_ERROR,  "BTLTPHandleDownloadServiceDatabaseReq too much\n", 0);
                    cause = Load_CauseServiceTooMuch;
                }
                else
                {

                    pServiceTable->pService = ACI_GetBuffer(database_len);
                    if (pServiceTable->pService == NULL)
                    {
                        DBG_BUFFER(MODULE_LTP, LEVEL_ERROR,  "BTLTPHandleDownloadServiceDatabaseReq allocate failed\n", 0);
                        fs_clear_all_ServiceTable(pBTLtp);
                        cause = Load_CauseServiceNoResource;
                    }
                    else
                    {
                        pServiceTable->isUsed = TRUE;
                        pServiceTable->host_service = host_service;
                        pServiceTable->database_length = database_len;
                        pServiceTable->receive_length = lenPara - pos;
                        memcpy(pServiceTable->pService, pPara + pos, pServiceTable->receive_length);
                        if (pServiceTable->receive_length == pServiceTable->database_length)
                        {
                            pServiceTable->nbrOfAttrib = pServiceTable->database_length / sizeof(TAttribAppl);
                            fs_save_New_ServiceTable(pServiceTable);
                        }
                    }
                }
            }
            else
            {
                DBG_BUFFER(MODULE_LTP, LEVEL_ERROR,  "BTLTPHandleDownloadServiceDatabaseReq service exist\n", 0);
                cause = Load_CauseServiceExist;
            }
        }
        break;
    case ACI_SERVICE_DATABASE_CONTINUE:
        {
            pServiceTable = BTACILookupGATTServiceTableByHostService(pBTLtp, host_service);
            if (pServiceTable == NULL)
            {
                DBG_BUFFER(MODULE_LTP, LEVEL_ERROR,  "BTLTPHandleDownloadServiceDatabaseReq not exist\n", 0);
                cause = Load_CauseServiceNotExist;
            }
            else
            {
                receive_pos = pServiceTable->receive_length;
                memcpy(((uint8_t *)pServiceTable->pService + receive_pos), pPara + pos, lenPara - pos);
                pServiceTable->receive_length += (lenPara - pos);
            }
        }
        break;
    case ACI_SERVICE_DATABASE_END:
        {
            pServiceTable = BTACILookupGATTServiceTableByHostService(pBTLtp, host_service);
            if (pServiceTable == NULL)
            {
                DBG_BUFFER(MODULE_LTP, LEVEL_ERROR,  "BTLTPHandleDownloadServiceDatabaseReq not exist\n", 0);
                cause = Load_CauseServiceNotExist;
            }
            else
            {
                receive_pos = pServiceTable->receive_length;
                memcpy(((uint8_t *)pServiceTable->pService + receive_pos), pPara + pos, lenPara - pos);
                pServiceTable->receive_length += (lenPara - pos);
                if (pServiceTable->receive_length != pServiceTable->database_length)
                {
                    ACI_FreeBuffer(pServiceTable->pService, pServiceTable->database_length);
                    pServiceTable->isUsed = FALSE;
                    pServiceTable->host_service = 0;
                    pServiceTable->database_length = 0;
                    pServiceTable->receive_length = 0;
                    pServiceTable->pService = NULL;
                    cause = Load_CauseLengthError;
                    DBG_BUFFER(MODULE_LTP, LEVEL_ERROR,  "BTLTPHandleDownloadServiceDatabaseReq len check failed\n", 0);

                }
                else
                {
                    pServiceTable->nbrOfAttrib = pServiceTable->database_length / sizeof(TAttribAppl);
                    DBG_BUFFER(MODULE_LTP, LEVEL_INFO,  "Download OK %d\n", 1, pServiceTable->nbrOfAttrib);
                    fs_save_New_ServiceTable(pServiceTable);
                }
            }
        }
        break;
    default:
        cause = Load_CauseInvalidPDU;
        break;
    }

    LTPLibSendDownloadServiceDatabaseRsp(&pBTLtp->LTPLib, BTLTP_DEFAULT_COPMSK, NULL, cause);
}


void BTLTPHandleClearServiceDatabaseReq(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara)
{
    TBlueAPI_Cause cause = blueAPI_CauseSuccess;
    DBG_BUFFER(MODULE_LTP, LEVEL_INFO, "BTLTPHandleClearServiceDatabaseReq !", 0);
    fs_clear_all_ServiceTable(pBTLtp);
    LTPLibSendClearServiceDatabaseRsp(&pBTLtp->LTPLib, BTLTP_DEFAULT_COPMSK, NULL, cause);
}

void BTLTPHandleSetTraceLevelReq(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara)
{
    TBlueAPI_Cause cause = blueAPI_CauseSuccess;
	uint16_t           pos = 1;
	uint8_t           level;
	UINT           stack_level;
	UINT32 debug[4];
    debug[0] = 0;
    debug[1] = 0;
    debug[2] = 0;
    debug[3] = 0;

    level        = pPara[pos++];
    stack_level  = NETCHAR2LONG(&pPara[pos]);
    pos += 4;
	DBG_BUFFER(MODULE_LTP, LEVEL_INFO, "BTLTPHandleSetTraceLevelReq level 0x%x stack_level 0x%x", 2, level, stack_level);
    if(level >= 3)
    {
        //g_eflash_dbg = 0x02;
    }
	for(uint8_t i = 0; i< level;i++)
	{
    	debug[i] = 0xffffffff;
	}
    set_debug_mask(debug);
    //stSetTraceLevel(stack_level);
    LTPLibSendSetTraceLevelRsp(&pBTLtp->LTPLib, BTLTP_DEFAULT_COPMSK, NULL, cause);
}
#if 0
void BTLTPHandleGATTAttributePrepareWriteReq(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara)
{
    uint16_t pos = 1;
    uint8_t loc_MDL_ID;
    uint16_t attribHandle;
    uint16_t writeOffset;

    PBTLtpMDLContext pMDLContext = NULL;
    uint16_t wOffset = 0;
    void* pBuffer = NULL;
    uint16_t ds_pool_id = 0;

    /* read mandatory parameters                                             */
    loc_MDL_ID    = pPara[pos++];
    attribHandle  = NETCHAR2SHORT(&pPara[pos]);
    pos += 2;
    writeOffset   = NETCHAR2SHORT(&pPara[pos]);
    pos += 2;

    /* read optional parameters */
    pMDLContext = BTLTPFindMDLContext(pBTLtp, loc_MDL_ID);

    if (NULL != pMDLContext)
    {
        wOffset = pMDLContext->dsDataOffset + 3;
        ds_pool_id = pMDLContext->dsPoolID;
    }
    else
    {
        return;
    }

    if (blueAPI_CauseSuccess == blueAPI_BufferGet(ds_pool_id,  lenPara - pos, wOffset, (void**)&pBuffer))
    {
        memcpy(((uint8_t *)pBuffer) + wOffset, pPara + pos, lenPara - pos);
        if (!blueAPI_GATTAttributePrepareWriteReq(pBuffer,
                                           loc_MDL_ID,
                                           attribHandle,
                                           (lenPara - pos),
                                           writeOffset,
                                           wOffset
                                          ))
        {
            blueAPI_BufferRelease(pBuffer);
        }
    }
    else
    {
        DBG_BUFFER(MODULE_LTP, LEVEL_ERROR, "blueAPI_GATTAttributePrepareWriteReq blueAPI_BufferGet failed!", 0);
    }

    return;
}

void BTLTPHandleGATTAttributeExecuteWriteReq(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara)
{
    uint16_t pos = 1;
    uint8_t loc_MDL_ID;
    uint8_t flags;

    loc_MDL_ID    = pPara[pos++];
    flags    = pPara[pos++];
    blueAPI_GATTAttributeExecuteWriteReq(loc_MDL_ID, flags);
}

void BTLTPHandleGATTAttributePrepareWriteCnf(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara)
{
    uint16_t pos = 1;
    uint8_t cause;
    uint16_t subCause       = 0x0000;
    uint8_t loc_MDL_ID     = 0x00;
    uint8_t serviceHandle  = 0x00;
    uint16_t attribIndex    = 0x0000;

    PBTLtpMDLContext pMDLContext = NULL;
    uint16_t wOffset = 0;
    void* pBuffer = NULL;
    uint16_t ds_pool_id = 0;

    /* read mandatory parameters                                             */
    cause           = pPara[pos++];
    subCause      = NETCHAR2SHORT(&pPara[pos]);
    pos += 2;
    loc_MDL_ID    = pPara[pos++];
    serviceHandle = pPara[pos++];
    attribIndex   = NETCHAR2SHORT(&pPara[pos]);
    pos += 2;

    /* read optional parameters */
    pMDLContext = BTLTPFindMDLContext(pBTLtp, loc_MDL_ID);

    if (NULL != pMDLContext)
    {
        wOffset = pMDLContext->dsDataOffset;
        ds_pool_id = pMDLContext->dsPoolID;
    }

    if (blueAPI_CauseSuccess == blueAPI_BufferGet(ds_pool_id,  lenPara - pos, wOffset, (void**)&pBuffer))
    {
        memcpy(((uint8_t *)pBuffer) + wOffset, pPara + pos, lenPara - pos);
        if (!blueAPI_GATTAttributePrepareWriteConf(pBuffer,
                                           loc_MDL_ID,
                                           BTLTPLookupGATTServiceHandle(pBTLtp, serviceHandle),
                                           (TBlueAPI_Cause)cause,
                                           subCause,
                                           attribIndex,
                                           (lenPara - pos),
                                           wOffset
                                          ))
        {
            blueAPI_BufferRelease(pBuffer);
        }
    }
    else
    {
        DBG_BUFFER(MODULE_LTP, LEVEL_ERROR, "BTLTPHandleGATTAttributePrepareWriteCnf blueAPI_BufferGet failed!", 0);
    }
  
    return;
}

void BTLTPHandleGATTAttributeExecuteWriteCnf(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara)
{
    uint16_t pos = 1;
    uint8_t loc_MDL_ID;
    uint8_t cause;
    uint16_t subCause;
    uint16_t handle;

    loc_MDL_ID    = pPara[pos++];
    cause    = pPara[pos++];
    subCause = NETCHAR2SHORT(&pPara[pos]);
    pos += 2;
    handle = NETCHAR2SHORT(&pPara[pos]);
    pos += 2;
    blueAPI_GATTAttributeExecuteWriteConf(loc_MDL_ID, (TBlueAPI_Cause)cause, subCause, handle);
}
#endif
