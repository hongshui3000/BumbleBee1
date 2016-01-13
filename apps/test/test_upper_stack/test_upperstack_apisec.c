#include "gatttest.h"
#include <blueapi.h>
#include <string.h>
#include "test_cmd.h"
#include "blueapi_lib.h"

/* ************************************************************************* */
STATIC void test_upperstack_Bin2Hex(uint8_t * input, uint8_t * output, uint16_t length)
{
    uint16_t pos = 0;
    uint16_t i   = 0;
    uint8_t  nib;

    /* multiple-octet values written in hexadecimal notation have
     * the most significant octet towards the left and
     * the least significant octet towards the right
     */
    for (i = 0; i < length; i++)
    {
        nib           = (input[i] >> 4) & 0x0F;
        output[pos++] = (nib < 0x0A) ? ('0' + nib) : ('a' - 0x0A + nib);

        nib           =  input[i] & 0x0F;
        output[pos++] = (nib < 0x0A) ? ('0' + nib) : ('a' - 0x0A + nib);
    }

    output[pos] = '\0';
}

/* *************************************************************************
 * command devcfg
 * ************************************************************************* */
TTestResult test_upperstack_CmdDevCfg(PGATTTest pGattTest, TTestParseResult *pParseResult)
{
    switch (pParseResult->dwParameter[0])
    {
    case 0:
        blueAPI_DeviceConfigDeviceNameSetReq((uint8_t *)pParseResult->pParameter[1]);

        break;

    case 1:
        blueAPI_DeviceConfigAppearanceSetReq(pParseResult->dwParameter[1]);

        break;

    case 2:
        blueAPI_DeviceConfigSecuritySetReq(
                                           (pParseResult->dwParameter[1] != 0) ? pParseResult->dwParameter[1] | BLUE_API_USE_LE_FIXED_DISPLAY_VALUE
                                           : 0
                                          );

        break;

    case 3:
        blueAPI_DeviceConfigStoreSetReq((TBlueAPI_StoreBondModes)pParseResult->dwParameter[1],
                                        pParseResult->dwParameter[2]
                                       );
        break;
    case 4:
        blueAPI_DeviceConfigPerPrefConnParamSetReq(
                pParseResult->dwParameter[1],
                pParseResult->dwParameter[2],
                pParseResult->dwParameter[3],
                pParseResult->dwParameter[4]
                                                  );

    default:
        return test_ResultError;
    }

    return test_ResultOk;
} /* gattdCmdNvShow */
#if 0
TTestResult test_upperstack_CmdDevCfgGet(PGATTTest pGattTest, TTestParseResult *pParseResult)
{
    switch (pParseResult->dwParameter[0])
    {
    case 0:
        blueAPI_DeviceConfigDeviceNameGetReq();

        break;

    case 1:
        blueAPI_DeviceConfigAppearanceGetReq();

        break;

    case 2:
        blueAPI_DeviceConfigPerPrefConnParamGetReq();

    default:
        return test_ResultError;
    }

    return test_ResultOk;
} /* gattdCmdNvShow */
#endif
/* *************************************************************************
 * command bndlist
 * ************************************************************************* */
TTestResult test_upperstack_CmdBndList(PGATTTest pGattTest, TTestParseResult *pParseResult)
{
    TTestResult result = test_ResultError;
#if (GATTDSTORE_ENTRY_COUNT)
    bool displayAll;
    TBlueAPI_RemoteBDType remote_BD_Type = blueAPI_RemoteBDTypeAny;
    int idx = pParseResult->dwParameter[0];

    if (pParseResult->iParameterCount == 0)
        displayAll = TRUE;
    else
    {
        if (idx > (GATTDEMO_MAX_LINKS - 1))
            return result;

        if (!pGattTest->linkTable[idx].RemoteBdValid)
        {
            test_upperstack_CmdPrint(pGattTest, "idx = %d,remote bd is not valid", idx);
            return (result);
        }
    }

    if (pParseResult->iParameterCount == 2)
    {
        remote_BD_Type = (TBlueAPI_RemoteBDType)pParseResult->dwParameter[1];
    }

    {
        uint16_t          i;

        for (i = 0; i < GATTDSTORE_ENTRY_COUNT; i++)
        {
            PGATTDStoreEntry pSearch = &pGattTest->extStore[i];

            if (((pSearch->used == GATTDSTORE_SEC_ENTRY) || (pSearch->used == GATTDSTORE_CCC_ENTRY)) &&
                    (displayAll == TRUE) || (memcmp(pSearch->bd, pGattTest->linkTable[idx].RemoteBd, 6) == 0) &&
                    (remote_BD_Type == blueAPI_RemoteBDTypeAny) || (pSearch->bdType == remote_BD_Type)
               )
            {
                if (pSearch->used == GATTDSTORE_SEC_ENTRY)
                {
                    test_upperstack_CmdPrint( pGattTest, "Key: remote_BD=%s bdType=%d keyType=%d dataLen=%d\r\n",
                                              blueAPI_StringizeBd(pSearch->bd),
                                              pSearch->bdType,
                                              pSearch->p.sec.keyType,
                                              pSearch->p.sec.dataLen
                                              );
                }
                if (pSearch->used == GATTDSTORE_CCC_ENTRY)
                {
                    test_upperstack_CmdPrint( pGattTest, "CCC info: remote_BD=%s bdType=%d dataLen=%d \r\n",
                                              blueAPI_StringizeBd(pSearch->bd),
                                              pSearch->bdType,
                                              pSearch->p.gatt.dataLen
                                              );
                }
            }
        }

    }
#endif

    result = test_ResultOk;

    return result;
} /* gattdCmdBndList */


/* *************************************************************************
 * command bnddel
 * ************************************************************************* */
TTestResult test_upperstack_CmdBndDel(PGATTTest pGattTest, TTestParseResult *pParseResult)
{
    TTestResult result = test_ResultError;
#if (GATTDSTORE_ENTRY_COUNT)
    bool removeAll = FALSE;
    TBlueAPI_RemoteBDType remote_BD_Type = blueAPI_RemoteBDTypeAny;
    int idx = pParseResult->dwParameter[0];

    if (pParseResult->iParameterCount == 0)
        removeAll = TRUE;
    else
    {
        if (idx > (GATTDEMO_MAX_LINKS - 1))
            return test_ResultError;

        if (!pGattTest->linkTable[idx].RemoteBdValid)
        {
            test_upperstack_CmdPrint(pGattTest, "idx = %d,remote bd is not valid", idx);
            return (test_ResultError);
        }
    }
    if (pParseResult->iParameterCount == 2)
    {
        remote_BD_Type = (TBlueAPI_RemoteBDType)pParseResult->dwParameter[1];
    }


    {
        uint16_t          i;

        for (i = 0; i < GATTDSTORE_ENTRY_COUNT; i++)
        {
            PGATTDStoreEntry pSearch = &pGattTest->extStore[i];

            if (((pSearch->used == GATTDSTORE_SEC_ENTRY) || (pSearch->used == GATTDSTORE_CCC_ENTRY)) &&
                    (removeAll == TRUE) || (memcmp(pSearch->bd, pGattTest->linkTable[idx].RemoteBd, 6) == 0) &&
                    (remote_BD_Type == blueAPI_RemoteBDTypeAny) || (pSearch->bdType == remote_BD_Type)
               )
            {
                pSearch->used = GATTDSTORE_FREE_ENTRY;
            }
        }

#if (GATTDSTORE_NVRAM_ADDRESS)
        nvramUpdate(GATTDSTORE_NVRAM_ADDRESS, sizeof(pGattTest->extStore), (uint8_t *)&pGATTDemo->extStore);
#endif /* (GATTDSTORE_NVRAM_ADDRESS) */
    }
#endif
    result = test_ResultOk;
    return result;
} /* gattdCmdBndDel */


#if (F_BT_LE_PRIVACY_MODE)
/* *************************************************************************
 * command privacy
 * ************************************************************************* */
TTestResult test_upperstack_CmdPrivacyMode(PGATTTest pGattTest, TTestParseResult *pParseResult)
{
    blueAPI_LEPrivacyModeReq(NULL,
                             (TBlueAPI_LEPrivacyMode)pParseResult->dwParameter[0]
                            );

    return test_ResultOk;
}
#endif /* (F_BT_LE_PRIVACY_MODE) */







/* *************************************************************************
 * command authmode
 * ************************************************************************* */
TTestResult test_upperstack_CmdPairableMode(PGATTTest pGattTest, TTestParseResult *pParseResult)
{
    bool    result = test_ResultValueOutOfRange;



    if ((pParseResult->iParameterCount < 4) || (pParseResult->iParameterCount > 5))
    {
        return test_ResultWrongNumberOfParameters;
    }



    if (pParseResult->dwParameter[0] > 1 ||
            pParseResult->dwParameter[1] > (uint16_t)blueAPI_AuthMITMRequiredBonding ||
            pParseResult->dwParameter[2] > (uint16_t)blueAPI_IOCapKeyboardDisplay ||
            pParseResult->dwParameter[3] > 1
       )
    {
        return test_ResultValueOutOfRange;

    }

    result = blueAPI_PairableModeSetReq(pParseResult->dwParameter[0],
                                        (TBlueAPI_AuthRequirements)pParseResult->dwParameter[1],
                                        (TBlueAPI_IOCapabilities)pParseResult->dwParameter[2],
                                        pParseResult->dwParameter[3]
                                       );

    if (result)
    {
        test_upperstack_CmdPrint(pGattTest, "--> PairableModeSetReq: pairable=%d authreq=%s iocap=%s oobdata=%d \r\n",
                                 pParseResult->dwParameter[0],
                                 blueAPI_AuthRequirementsString((TBlueAPI_AuthRequirements)pParseResult->dwParameter[1]),
                                 blueAPI_IOCapabilitiesString((TBlueAPI_IOCapabilities)pParseResult->dwParameter[2]),
                                 pParseResult->dwParameter[3]
                                 );
    }

    return (result) ? test_ResultOk : test_ResultError;
} /* gattdCmdPairableMode */






/* *************************************************************************
 * command authkeyb
 * ************************************************************************* */
TTestResult test_upperstack_CmdAuthKeyboard(PGATTTest pGattTest, TTestParseResult *pParseResult)
{
    bool            result;
    TBlueAPI_Cause  cause   = blueAPI_CauseReject;
    int idx = pParseResult->dwParameter[0];

    if (idx > (GATTDEMO_MAX_LINKS - 1))
        return (test_ResultError);

    if (!pGattTest->linkTable[idx].RemoteBdValid)
    {
        test_upperstack_CmdPrint(pGattTest, "idx = %d,remote bd is not valid", idx);
        return (test_ResultError);
    }

    if (pParseResult->iParameterCount == 2)
    {
        cause   = blueAPI_CauseAccept;
    }

    result = blueAPI_UserPasskeyReqReplyReq(pGattTest->linkTable[idx].RemoteBd,
                                            pParseResult->dwParameter[1],
                                            cause
                                           );
    if (result)
    {
        test_upperstack_CmdPrint( pGattTest,
                                  "idx = %d--> UserPasskeyReqReplyReq: remote_BD=%s, passKey=0x%x, cause=%s\r\n",
                                  idx, blueAPI_StringizeBd(pGattTest->linkTable[idx].RemoteBd),
                                  pParseResult->dwParameter[1],
                                  blueAPI_CauseString(cause)
                                  );
    }

    return (result) ? test_ResultOk : test_ResultError;
} /* gattdCmdAuthKeyboard */


/* *************************************************************************
 * command authsetoob
 * ************************************************************************* */
TTestResult test_upperstack_CmdAuthSetOOB(PGATTTest pGattTest, TTestParseResult *pParseResult)
{
    bool            result;
    TBlueAPI_Cause  cause   = blueAPI_CauseReject;
    uint8_t         Cbin[16];
    uint8_t         Cstr[32 + 1];

    int idx = pParseResult->dwParameter[0];

    if (idx > (GATTDEMO_MAX_LINKS - 1))
        return (test_ResultError);

    if (!pGattTest->linkTable[idx].RemoteBdValid)
    {
        test_upperstack_CmdPrint(pGattTest, "idx = %d,remote bd is not valid", idx);
        return (test_ResultError);
    }

    memset(Cbin, 0x00, 16);


    if (pParseResult->iParameterCount > 1)
    {
        ASCIIHexToBin((uint8_t *)pParseResult->pParameter[1], Cbin, 16);



        cause = blueAPI_CauseAccept;
    }

    test_upperstack_Bin2Hex(Cbin, Cstr, 16);


    result = blueAPI_RemoteOOBDataReqConf(pGattTest->linkTable[idx].RemoteBd, Cbin, cause
                                         );
    if (result)
    {
        test_upperstack_CmdPrint( pGattTest,
                                  "idx = %d--> RemoteOOBDataReqConf: remote_BD=%s, C=[%s], cause=%s\r\n",
                                  idx, blueAPI_StringizeBd(pGattTest->linkTable[idx].RemoteBd),
                                  Cstr,
                                  blueAPI_CauseString(cause)
                                  );
    }

    return (result) ? test_ResultOk : test_ResultError;
} /* gattdCmdAuthSetOOB */




#if (F_BT_LE_PRIVACY_MODE)
/* ************************************************************************* */
void test_upperstack_Handle_LEPrivacyModeRsp(PGATTTest pGattTest, PBlueAPI_LEPrivacyModeRsp pPrivacyRsp)
{
    test_upperstack_CmdPrint(pGattTest, "<-- LEPrivacyModeRsp: bd=%s bdType=%d cause=%s\r\n",
                             blueAPI_StringizeBd(pPrivacyRsp->local_BD),
                             pPrivacyRsp->local_BD_type,
                             blueAPI_CauseString(pPrivacyRsp->cause)
                             );
} /* gattdHandle_LEPrivacyModeRsp */
#endif /* (F_BT_LE_PRIVACY_MODE) */



/* ************************************************************************* */
void test_upperstack_Handle_AuthResultInd(PGATTTest pGattTest, PBlueAPI_AuthResultInd pAuthResultInd)
{
    TBlueAPI_Cause  cause = blueAPI_CauseSuccess;

    test_upperstack_CmdPrint(pGattTest, "<-- AuthResultInd: bd=%s bdType=%d keyType=%d cause=%s\r\n",
                             blueAPI_StringizeBd(pAuthResultInd->remote_BD),
                             pAuthResultInd->remote_BD_Type,
                             pAuthResultInd->keyType,
                             blueAPI_CauseString((TBlueAPI_Cause)pAuthResultInd->cause)
                             );

#if (GATTDSTORE_ENTRY_COUNT)
    {
        PGATTDStoreEntry  pEntry = NULL;
        uint16_t          i;

        for (i = 0; i < GATTDSTORE_ENTRY_COUNT; i++)
        {
            PGATTDStoreEntry pSearch = &pGattTest->extStore[i];

            if ((pSearch->used == GATTDSTORE_SEC_ENTRY) &&
                    (memcmp(pSearch->bd, pAuthResultInd->remote_BD, 6) == 0) &&
                    (pSearch->bdType == pAuthResultInd->remote_BD_Type)
               )
            {
                if (pAuthResultInd->keyType == pSearch->p.sec.keyType)
                {
                    pEntry = pSearch;
                    break;
                }
            }
            else if ((pSearch->used == GATTDSTORE_FREE_ENTRY) &&
                     (pEntry == NULL)
                    )
            {
                pEntry = pSearch;
            }
        }

        if (pEntry)
        {
            pEntry->used          = GATTDSTORE_SEC_ENTRY;
            memcpy(pEntry->bd, pAuthResultInd->remote_BD, 6);
            pEntry->bdType        = pAuthResultInd->remote_BD_Type;
            pEntry->p.sec.keyType = pAuthResultInd->keyType;
            pEntry->p.sec.dataLen = pAuthResultInd->linkKeyLength;
            memcpy(pEntry->p.sec.data, pAuthResultInd->linkKey, pEntry->p.sec.dataLen);

#if (GATTDSTORE_NVRAM_ADDRESS)
            nvramUpdate(GATTDSTORE_NVRAM_ADDRESS, sizeof(pGattTest->extStore), (uint8_t *)&pGATTDemo->extStore);
#endif /* (GATTDSTORE_NVRAM_ADDRESS) */
        }
        else
        {
            cause = blueAPI_CauseResourceError;
        }
    }
#endif /* (GATTDSTORE_ENTRY_COUNT) */


    blueAPI_AuthResultConf(pAuthResultInd->remote_BD,
                           (TBlueAPI_RemoteBDType)pAuthResultInd->remote_BD_Type,
                           cause
                          );
} /* gattdHandle_AuthResultInd */


/* ************************************************************************* */
void test_upperstack_Handle_AuthResultRequestInd(PGATTTest pGattTest, PBlueAPI_AuthResultRequestInd pAuthResultRequestInd)
{
#if (GATTDSTORE_ENTRY_COUNT)
    uint16_t          i;
    PGATTDStoreEntry  pEntry = NULL;
    uint16_t          restartHandle = 0x0000;
    uint8_t           BD0[6]        = { 0, 0, 0, 0, 0, 0 };
    bool              nullBD;
#endif /* (GATTDSTORE_ENTRY_COUNT) */

    test_upperstack_CmdPrint(pGattTest, "<-- AuthResultRequestInd: bd=%s bdType=%d keyType=%d restart=0x%x\r\n",
                             blueAPI_StringizeBd(pAuthResultRequestInd->remote_BD),
                             pAuthResultRequestInd->remote_BD_Type,
                             pAuthResultRequestInd->keyType,
                             pAuthResultRequestInd->restartHandle
                             );

#if (GATTDSTORE_ENTRY_COUNT)
    nullBD = (memcmp(pAuthResultRequestInd->remote_BD, BD0, BLUE_API_BD_SIZE) == 0);

    for (i = pAuthResultRequestInd->restartHandle; i < GATTDSTORE_ENTRY_COUNT; i++)
    {
        PGATTDStoreEntry pSearch = &pGattTest->extStore[i];

        if ((pSearch->used == GATTDSTORE_SEC_ENTRY) &&
                ((memcmp(pSearch->bd, pAuthResultRequestInd->remote_BD, 6) == 0) || nullBD) &&
                ((pSearch->bdType == pAuthResultRequestInd->remote_BD_Type) ||
                 (pAuthResultRequestInd->remote_BD_Type == blueAPI_RemoteBDTypeAny)
                ))
        {
            if (pAuthResultRequestInd->keyType == pSearch->p.sec.keyType)
            {
                if (pEntry == NULL)
                {
                    pEntry        = pSearch;
                    restartHandle = 0x0000;
                }
                else
                {
                    restartHandle = i;
                    break;
                }
            }
        }
    }

    if (pEntry)
    {
        blueAPI_AuthResultRequestConf(pEntry->bd,
                                      pEntry->bdType,
                                      pEntry->p.sec.dataLen,
                                      pEntry->p.sec.data,
                                      pEntry->p.sec.keyType,
                                      restartHandle,
                                      blueAPI_CauseAccept
                                     );
    }
    else
#endif /* (GATTDSTORE_ENTRY_COUNT) */
    {
        blueAPI_AuthResultRequestConf(pAuthResultRequestInd->remote_BD,
                                      pAuthResultRequestInd->remote_BD_Type,
                                      0, NULL,
                                      pAuthResultRequestInd->keyType,
                                      0x0000,
                                      blueAPI_CauseReject
                                     );
    }
} /* gattdHandle_AuthResultInd */

/* ************************************************************************* */
void test_upperstack_Handle_PairableModeSetRsp(PGATTTest pGattTest, PBlueAPI_PairableModeSetRsp pPairableModeSetRsp)
{
    test_upperstack_CmdPrint(pGattTest, "<-- PairableModeSetRsp: cause=%s\r\n",
                             blueAPI_CauseString(pPairableModeSetRsp->cause)
                             );
} /* gattdHandle_PairableModeSetRsp */


/* ************************************************************************* */
void test_upperstack_Handle_UserPasskeyReqInd(PGATTTest pGattTest, PBlueAPI_UserPasskeyReqInd pUserPasskeyReqInd)
{
    test_upperstack_CmdPrint(pGattTest, "<-- UserPasskeyReqInd: bd=%s\r\n",
                             blueAPI_StringizeBd(pUserPasskeyReqInd->remote_BD)
                             );

    blueAPI_UserPasskeyReqConf(pUserPasskeyReqInd->remote_BD,
                               blueAPI_CauseAccept
                              );
} /* gattdHandle_UserPasskeyReqInd */


/* ************************************************************************* */
void test_upperstack_Handle_UserPasskeyNotificationInfo(PGATTTest pGattTest, PBlueAPI_UserPasskeyNotificationInfo pUserPasskeyNotificationInfo)
{
    test_upperstack_CmdPrint(pGattTest, "<-- UserPasskeyNotificationInfo: bd=%s value=%06d\r\n",
                             blueAPI_StringizeBd(pUserPasskeyNotificationInfo->remote_BD),
                             pUserPasskeyNotificationInfo->displayValue
                             );
} /* gattdHandle_UserPasskeyNotificationInfo */


/* ************************************************************************* */
void test_upperstack_Handle_UserPasskeyReqReplyRsp(PGATTTest pGattTest, PBlueAPI_UserPasskeyReqReplyRsp pUserPasskeyReqReplyRsp)
{
    test_upperstack_CmdPrint(pGattTest, "<-- UserPasskeyReqReplyRsp: cause=%s\r\n",
                             blueAPI_CauseString(pUserPasskeyReqReplyRsp->cause)
                             );
} /* gattdHandle_UserPasskeyReqReplyRsp */


/* ************************************************************************* */
void test_upperstack_Handle_RemoteOOBDataReqInd(PGATTTest pGattTest, PBlueAPI_RemoteOOBDataReqInd pRemoteOOBDataReqInd)
{
    test_upperstack_CmdPrint(pGattTest, "<-- RemoteOOBDataReqInd: bd=%s\r\n",
                             blueAPI_StringizeBd(pRemoteOOBDataReqInd->remote_BD)
                             );
} /* gattdHandle_RemoteOOBDataReqInd */
