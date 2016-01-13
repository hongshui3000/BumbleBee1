enum { __FILE_NUM__ = 0 };


#include <blueapi_types.h>
#include "btltp.h"
#include "trace.h"


PBTLtpMDLContext BTLTPAllocateMDLContext(PBTLtp pBTLtp, uint8_t local_MDL_ID, uint8_t local_MDEP_ID, TBlueAPI_LinkConfigType linkConfigType)
{
    uint16_t loop;
    PBTLtpMDLContext pUnusedContext = NULL;
    BOOL isGATT = (linkConfigType == blueAPI_LinkConfigGATT);
    if (!isGATT && (local_MDL_ID == 0))
    {
        return NULL;
    }

    for (loop = 0; loop < BLUE_API_MDL_COUNT; loop++)
    {
        PBTLtpMDLContext pContext = &pBTLtp->MDLContextPool[loop];
        if (pContext->flags & LTP_MDL_ALLOCATED)
        {
            if (isGATT &&
                (pContext->flags & LTP_MDL_GATT) &&
                    (pContext->local_MDL_ID == 0x00) &&
                    (pContext->local_MDEP_ID == local_MDEP_ID)
               )
            {
                pContext->local_MDL_ID = local_MDL_ID;
                return pContext;
            }
            else if (pContext->local_MDL_ID == local_MDL_ID)
            {
                return pContext;
            }
        }
        else if (pUnusedContext == NULL)
        {
            pUnusedContext = pContext;
        }
    }

    if (pUnusedContext != NULL)
    {
        memset(pUnusedContext, 0, sizeof(TBTLtpMDLContext));
        pUnusedContext->flags           = LTP_MDL_ALLOCATED;
        pUnusedContext->local_MDL_ID    = local_MDL_ID;
        pUnusedContext->local_MDEP_ID   = local_MDEP_ID;
        if (isGATT)
        {
            pUnusedContext->flags        |= LTP_MDL_GATT;
        }

        return pUnusedContext;
    }

    DBG_BUFFER(MODULE_LTP, LEVEL_ERROR, "!!!LTP: out of MDL context elements", 0);
    return NULL;
}


PBTLtpMDLContext BTLTPFindMDLContext(PBTLtp pBTLtp, uint8_t local_MDL_ID)
{
    uint16_t loop;

    for (loop = 0; loop < BLUE_API_MDL_COUNT; loop++)
    {
        PBTLtpMDLContext pContext = &pBTLtp->MDLContextPool[loop];
        if ((pContext->flags & LTP_MDL_ALLOCATED) &&
                (pContext->local_MDL_ID == local_MDL_ID)
           )
        {
            return pContext;
        }
    }

    DBG_BUFFER(MODULE_LTP, LEVEL_ERROR, "!!!LTP: no MDL context found for MDL %d", 1, local_MDL_ID);
    return NULL;
}

PBTLtpMDLContext BTLTPFindMDLContextByMDEPID(PBTLtp pBTLtp, uint8_t local_MDEP_ID)
{
  uint16_t loop;

  for (loop = 0; loop < BLUE_API_MDL_COUNT; loop++)
  {
     PBTLtpMDLContext pContext = &pBTLtp->MDLContextPool[loop];
     if ((pContext->flags & LTP_MDL_ALLOCATED) &&
         (pContext->local_MDL_ID == 0x00) &&
         (pContext->local_MDEP_ID == local_MDEP_ID)
        )
     {
       return pContext;
     }
  }

  return NULL;
}

uint8_t BTLTPCountMDLConnected(PBTLtp pBTLtp)
{
    uint16_t loop;
    uint8_t count = 0;

    for (loop = 0; loop < BLUE_API_MDL_COUNT; loop++)
    {
        PBTLtpMDLContext pContext = &pBTLtp->MDLContextPool[loop];

        if ((pContext->flags & (LTP_MDL_ALLOCATED | LTP_MDL_CONNECTED)) == (LTP_MDL_ALLOCATED | LTP_MDL_CONNECTED))
        {
            count++;
        }
    }

    return count;
}

PBTLtpMDLContext BTLTPFindAMDLConnected(PBTLtp pBTLtp)
{
    uint16_t loop;

    for (loop = 0; loop < BLUE_API_MDL_COUNT; loop++)
    {
        PBTLtpMDLContext pContext = &pBTLtp->MDLContextPool[loop];

        if ((pContext->flags & (LTP_MDL_ALLOCATED | LTP_MDL_CONNECTED)) == (LTP_MDL_ALLOCATED | LTP_MDL_CONNECTED))
        {
            return pContext;
        }
    }

    return NULL;
}

PBTLtpAction BTLTPAllocateAction(PBTLtp pBTLtp)
{
    uint16_t loop;

    for (loop = 0; loop < BTLTP_ACTION_POOL_SIZE; loop++)
    {
        if (pBTLtp->ActionPool[loop].Action == btltpActionNotUsed)
        {
            return &pBTLtp->ActionPool[loop];
        }
    }

    DBG_BUFFER(MODULE_LTP, LEVEL_ERROR, "!!!LTP: out of action queue elements", 0);
    return NULL;
}

uint8_t BTLTPConvertCOMtoLTPcause(TBlueAPI_Cause cause)
{
    switch (cause)
    {
    case blueAPI_CauseUnspecified:
        return LTP_CAUSE_UNSPECIFIED;
    default:
        return (uint8_t) cause;
    }
}

TBlueAPI_Cause BTLTPConvertLTPtoCOMcause(uint8_t cause)
{
    switch (cause)
    {
    case LTP_CAUSE_NOT_SUPPORTED:
        return blueAPI_CauseNotSupported;
    default:
        return (TBlueAPI_Cause)cause;
    }
}

void BTLTPCheckForActInfo(PBTLtp pBTLtp)
{
    uint8_t pOpt[4];
    uint16_t pos = 0;


    if (pBTLtp->ActInfoFlags == LTP_ACT_INFO_FLAG_ALL)
    {
        NETSHORT2CHAR(&pOpt[pos], pBTLtp->LTPLib.ReceiveMaxLength);
        pos += 2;
        NETSHORT2CHAR(&pOpt[pos], pBTLtp->LTPLib.ReceiveMaxLength);
        pos += 2;

        DBG_BUFFER(MODULE_LTP, LEVEL_INFO, "LTP: send ActInfo cause(%d)", 1, pBTLtp->ActInfoCause);

        LTPLibSendActInfo(&pBTLtp->LTPLib,
                          BTLTP_DEFAULT_COPMSK | \
                          LTP_ACT_INFO_OPT_MASK_MAX_RX_MESSAGE_LENGTH | \
                          LTP_ACT_INFO_OPT_MASK_MAX_TX_MESSAGE_LENGTH,
                          pOpt,
                          pBTLtp->ActInfoCause,
                          LTP_VERSION,
                          pBTLtp->ownBDAddress,
                          pBTLtp->ActInfoVersionString
                         );
    }
}

