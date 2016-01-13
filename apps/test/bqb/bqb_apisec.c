enum { __FILE_NUM__= 0 };

/**********************************************************************!KA****
 *
 * $Header: /var/lib/cvs/sw/src/app/demo/gattdemo/gatdbapisec.c,v 1.22.2.1 2013/10/17 09:51:18 mn Exp $
 *
 * File:        $RCSfile: gatdbapisec.c,v $
 * Version:     $Name: P_SRP1290_V1_0 $
 *
 * Archive:     $Source: /var/lib/cvs/sw/src/app/demo/gattdemo/gatdbapisec.c,v $
 * Revision:    $Revision: 1.22.2.1 $
 * Date:        $Date: 2013/10/17 09:51:18 $
 * Author:      $Author: mn $
 *
 * ---------------------------------------------------------------------------
 * !MODULE      [  ]
 * ---------------------------------------------------------------------------
 * !FILE        [  ]
 * !PROGRAM     [  ]
 * !VERSION     [$Name: P_SRP1290_V1_0 $]
 * !GROUP       [  ]
 * !AUTHOR      [$Author: mn $]
 * ---------------------------------------------------------------------------
 *
 *          Copyright (c)           2011 Stollmann E+V GmbH
 *                                  Mendelssohnstr. 15
 *                                  22761 Hamburg
 *                                  Phone: 040/89088-0
 *          The content of this file is Stollmann E+V GmbH confidential
 *          All Rights Reserved
 *
 * ---------------------------------------------------------------------------
 * !DESCRIPTION
 *
 *         GATTDEMO / BlueAPI security related interfacing
 * ---------------------------------------------------------------------------
 * !INDEX
 *  ...
 * ---------------------------------------------------------------------------
 * !CONTENTS
 * ---------------------------------------------------------------------------
 * !INCLUDE_REFERENCES
 * ---------------------------------------------------------------------------
 * !HISTORY
 *  Date      Author          Comment
 *  tt.mm.jj                  Initial revision
 *  tt.mm.jj
 * ---------------------------------------------------------------------------
 *
 ************************************************************************!KA*/

#include "bqb_demo.h"
#include <string.h>
#include <blueapi.h>
#include "blueapi_lib.h"

uint8_t SecAUT14 = 0;    /* by passing TC_SEC_AUT_BV_14_C */

/* ************************************************************************* */
STATIC void gattdBin2Hex(uint8_t * input, uint8_t * output, uint16_t length)
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
TGATTDemoResult BQB_CmdDevCfg(PGATTDemo pGATTDemo, TGATTDemoParseResult *pParseResult)
{
    switch (pParseResult->dwParameter[0])
    {
    case 0:
        /*blueAPI_DeviceConfigDeviceSetReq(NULL,
                                         (TBlueAPI_DeviceNameType)pParseResult->dwParameter[1],
                                         (uint8_t *)pParseResult->pParameter[2]); */   /* tifnan: for  stack change*/
        break;


    case 1:
        blueAPI_DeviceConfigSecuritySetReq(
                                           //(TBlueAPI_StoreBondModes)pParseResult->dwParameter[1],
                                           (pParseResult->dwParameter[3] != 0) ? (BLUE_API_USE_LE_FIXED_DISPLAY_VALUE | 123456) : 0
                                          );
        break;

    default:
        return gattdResultError;
    }

    return gattdResultOk;
} /* gattdCmdNvShow */




#if (F_BT_LE_PRIVACY_MODE)
/* *************************************************************************
 * command privacy
 * ************************************************************************* */
TGATTDemoResult BQB_CmdPrivacyMode(PGATTDemo pGATTDemo, TGATTDemoParseResult *pParseResult)
{
    /* blueAPI_LEPrivacyModeReq(NULL,
                             (TBlueAPI_LEPrivacyMode)pParseResult->dwParameter[0]
                            ); */ /* tifnan: for stack changes */

    return gattdResultOk;
}
#endif /* (F_BT_LE_PRIVACY_MODE) */

/* *************************************************************************
 * command authmode
 * ************************************************************************* */
TGATTDemoResult BQB_CmdPairableMode(PGATTDemo pGATTDemo, TGATTDemoParseResult *pParseResult)
{
    bool    result;

    //char *btmode_str[3] = { "bt2.0", "bt2.1", "bt2.1dbg" };

    if ((pParseResult->iParameterCount < 4) || (pParseResult->iParameterCount > 5))
    {
        return gattdResultWrongNumberOfParameters;
    }

    if (pParseResult->dwParameter[0] > 1 ||
            pParseResult->dwParameter[1] > (uint16_t)blueAPI_AuthMITMRequiredBonding ||
            pParseResult->dwParameter[2] > (uint16_t)blueAPI_IOCapKeyboardDisplay ||
            pParseResult->dwParameter[3] > 1
       )
    {
        return gattdResultValueOutOfRange;
    }

    result = blueAPI_PairableModeSetReq(pParseResult->dwParameter[0],
                                        (TBlueAPI_AuthRequirements)pParseResult->dwParameter[1],
                                        (TBlueAPI_IOCapabilities)pParseResult->dwParameter[2],
                                        pParseResult->dwParameter[3]
                                       );

    if (result)
    {
        BQB_CmdPrint(pGATTDemo, "--> PairableModeSetReq: pairable=%d authreq=%s iocap=%s oobdata=%d \r\n%s",
                     pParseResult->dwParameter[0],
                     blueAPI_AuthRequirementsString((TBlueAPI_AuthRequirements)pParseResult->dwParameter[1]),
                     blueAPI_IOCapabilitiesString((TBlueAPI_IOCapabilities)pParseResult->dwParameter[2]),
                     pParseResult->dwParameter[3],
                     pGATTDemo->CmdIF.cPrompt );
    }

    return (result) ? gattdResultOk : gattdResultError;
} /* gattdCmdPairableMode */


TGATTDemoResult BQB_AutoChangeAuthMode(PGATTDemo pGATTDemo, TGATTDemoParseResult *pParseResult)
{
	if (0 == pParseResult->dwParameter[0])
    {
        SecAUT14 = 0;
        BQB_CmdPrint(pGATTDemo, "disable auto change authmode func successfully\r\n");
    }
    else if (1 == pParseResult->dwParameter[0])
    {
        SecAUT14 = 1;
        BQB_CmdPrint(pGATTDemo, "enable auto change authmode func successfully\r\n");
    }
    else
    {
        BQB_CmdPrint(pGATTDemo, "invalid parameters!\r\n");
        return gattdResultError;
    }
    
    return gattdResultOk;
}




/* *************************************************************************
 * command authkeyb
 * ************************************************************************* */
TGATTDemoResult BQB_CmdAuthKeyboard(PGATTDemo pGATTDemo, TGATTDemoParseResult *pParseResult)
{
    bool            result;
    TBlueAPI_Cause  cause   = blueAPI_CauseReject;
    int idx = pParseResult->dwParameter[0];

    if (idx > (BQB_MAX_LINKS - 1))
    {
        return (gattdResultError);
    }

    if (!pGATTDemo->linkTable[idx].RemoteBdValid)
    {
        BQB_CmdPrint(pGATTDemo, "idx = %d,remote bd is not valid", idx);
        return (gattdResultError);
    }

    if (pParseResult->iParameterCount == 2)
    {
        cause   = blueAPI_CauseAccept;
    }

    result = blueAPI_UserPasskeyReqReplyReq(pGATTDemo->linkTable[idx].RemoteBd,
                                            pParseResult->dwParameter[1],
                                            cause
                                           );
    if (result)
    {
        BQB_CmdPrint( pGATTDemo,
                      "idx = %d--> UserPasskeyReqReplyReq: remote_BD=%s, passKey=0x%x, cause=%s\r\n%s",
                      idx, blueAPI_StringizeBd(pGATTDemo->linkTable[idx].RemoteBd),
                      pParseResult->dwParameter[1],
                      blueAPI_CauseString(cause),
                      pGATTDemo->CmdIF.cPrompt );
    }

    return (result) ? gattdResultOk : gattdResultError;
} /* gattdCmdAuthKeyboard */


/* *************************************************************************
 * command authsetoob
 * ************************************************************************* */
TGATTDemoResult BQB_CmdAuthSetOOB(PGATTDemo pGATTDemo, TGATTDemoParseResult *pParseResult)
{
    bool            result;
    TBlueAPI_Cause  cause   = blueAPI_CauseReject;
    uint8_t         Cbin[16];
    uint8_t         Cstr[32 + 1];

    int idx = pParseResult->dwParameter[0];

    if (idx > (BQB_MAX_LINKS - 1))
    {
        return (gattdResultError);
    }

    if (!pGATTDemo->linkTable[idx].RemoteBdValid)
    {
        BQB_CmdPrint(pGATTDemo, "idx = %d,remote bd is not valid", idx);
        return (gattdResultError);
    }

    memset(Cbin, 0x00, 16);


    if (pParseResult->iParameterCount > 1)
    {
        ASCIIHexToBin((uint8_t *)pParseResult->pParameter[1], Cbin, 16);


        cause = blueAPI_CauseAccept;
    }

    gattdBin2Hex(Cbin, Cstr, 16);


    result = blueAPI_RemoteOOBDataReqConf(pGATTDemo->linkTable[idx].RemoteBd, Cbin, cause
                                         );
    if (result)
    {
        BQB_CmdPrint( pGATTDemo,
                      "idx = %d--> RemoteOOBDataReqConf: remote_BD=%s, C=[%s], cause=%s\r\n%s",
                      idx, blueAPI_StringizeBd(pGATTDemo->linkTable[idx].RemoteBd),
                      Cstr,
                      blueAPI_CauseString(cause),
                      pGATTDemo->CmdIF.cPrompt );
    }

    return (result) ? gattdResultOk : gattdResultError;
} /* gattdCmdAuthSetOOB */




#if (F_BT_LE_PRIVACY_MODE)
/* ************************************************************************* */
void BQB_Handle_LEPrivacyModeRsp(PGATTDemo pGATTDemo, PBlueAPI_LEPrivacyModeRsp pPrivacyRsp)
{
    BQB_CmdPrint(pGATTDemo, "<-- LEPrivacyModeRsp: bd=%s bdType=%d cause=%s\r\n%s",
                 blueAPI_StringizeBd(pPrivacyRsp->local_BD),
                 pPrivacyRsp->local_BD_type,
                 blueAPI_CauseString(pPrivacyRsp->cause),
                 pGATTDemo->CmdIF.cPrompt );
} /* gattdHandle_LEPrivacyModeRsp */
#endif /* (F_BT_LE_PRIVACY_MODE) */



/* ************************************************************************* */
void BQB_Handle_AuthResultInd(PGATTDemo pGATTDemo, PBlueAPI_AuthResultInd pAuthResultInd)
{
    TBlueAPI_Cause  cause = blueAPI_CauseSuccess;

    BQB_CmdPrint(pGATTDemo, "<-- AuthResultInd: bd=%s bdType=%d keyType=%d cause=%s\r\n%s",
                 blueAPI_StringizeBd(pAuthResultInd->remote_BD),
                 pAuthResultInd->remote_BD_Type,
                 pAuthResultInd->keyType,
                 blueAPI_CauseString((TBlueAPI_Cause)pAuthResultInd->cause),
                 pGATTDemo->CmdIF.cPrompt );
    
    blueAPI_AuthResultConf(pAuthResultInd->remote_BD,
                           (TBlueAPI_RemoteBDType)pAuthResultInd->remote_BD_Type,
                           cause
                          );
}


/* ************************************************************************* */
void BQB_Handle_PairableModeSetRsp(PGATTDemo pGATTDemo, PBlueAPI_PairableModeSetRsp pPairableModeSetRsp)
{
    BQB_CmdPrint(pGATTDemo, "<-- PairableModeSetRsp: cause=%s\r\n%s",
                 blueAPI_CauseString(pPairableModeSetRsp->cause),
                 pGATTDemo->CmdIF.cPrompt );
} /* gattdHandle_PairableModeSetRsp */


/* ************************************************************************* */
void BQB_Handle_UserPasskeyReqInd(PGATTDemo pGATTDemo, PBlueAPI_UserPasskeyReqInd pUserPasskeyReqInd)
{
    BQB_CmdPrint(pGATTDemo, "<-- UserPasskeyReqInd: bd=%s\r\n%s",
                 blueAPI_StringizeBd(pUserPasskeyReqInd->remote_BD),
                 pGATTDemo->CmdIF.cPrompt );

    blueAPI_UserPasskeyReqConf(pUserPasskeyReqInd->remote_BD,
                               blueAPI_CauseAccept
                              );
} /* gattdHandle_UserPasskeyReqInd */


/* ************************************************************************* */
void BQB_Handle_UserPasskeyNotificationInfo(PGATTDemo pGATTDemo, PBlueAPI_UserPasskeyNotificationInfo pUserPasskeyNotificationInfo)
{
    BQB_CmdPrint(pGATTDemo, "<-- UserPasskeyNotificationInfo: bd=%s value=%06d\r\n%s",
                 blueAPI_StringizeBd(pUserPasskeyNotificationInfo->remote_BD),
                 pUserPasskeyNotificationInfo->displayValue,
                 pGATTDemo->CmdIF.cPrompt );
} /* gattdHandle_UserPasskeyNotificationInfo */


/* ************************************************************************* */
void BQB_Handle_UserPasskeyReqReplyRsp(PGATTDemo pGATTDemo, PBlueAPI_UserPasskeyReqReplyRsp pUserPasskeyReqReplyRsp)
{
    BQB_CmdPrint(pGATTDemo, "<-- UserPasskeyReqReplyRsp: cause=%s\r\n%s",
                 blueAPI_CauseString(pUserPasskeyReqReplyRsp->cause),
                 pGATTDemo->CmdIF.cPrompt );
} /* gattdHandle_UserPasskeyReqReplyRsp */


/* ************************************************************************* */
void BQB_Handle_RemoteOOBDataReqInd(PGATTDemo pGATTDemo, PBlueAPI_RemoteOOBDataReqInd pRemoteOOBDataReqInd)
{
    BQB_CmdPrint(pGATTDemo, "<-- RemoteOOBDataReqInd: bd=%s\r\n%s",
                 blueAPI_StringizeBd(pRemoteOOBDataReqInd->remote_BD),
                 pGATTDemo->CmdIF.cPrompt );
} /* gattdHandle_RemoteOOBDataReqInd */
