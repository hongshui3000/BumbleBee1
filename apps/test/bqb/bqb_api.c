enum { __FILE_NUM__= 0 };

/**********************************************************************!KA****
 *
 * $Header: /var/lib/cvs/sw/src/app/demo/gattdemo/gatdbapi.c,v 1.65.2.1 2013/10/17 09:51:18 mn Exp $
 *
 * File:        $RCSfile: gatdbapi.c,v $
 * Version:     $Name: P_SRP1290_V1_0 $
 *
 * Archive:     $Source: /var/lib/cvs/sw/src/app/demo/gattdemo/gatdbapi.c,v $
 * Revision:    $Revision: 1.65.2.1 $
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
 *          Copyright (c)           2012 Stollmann E+V GmbH
 *                                  Mendelssohnstr. 15
 *                                  22761 Hamburg
 *                                  Phone: 040/89088-0
 *          The content of this file is Stollmann E+V GmbH confidential
 *          All Rights Reserved
 *
 * ---------------------------------------------------------------------------
 * !DESCRIPTION
 *
 *         GATTDEMO BlueAPI interfacing
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
#include <blueapi.h>
#include "bqb.h"
#include <string.h>
#include "blueapi_lib.h"

static uint8_t AdvDataIndex = ADV_DATA_IND_0;
//extern void SetMtuSizeOnlyForBQB(uint16_t mtuSize);

/*----------------------------------------------------------------------------
 * convert BlueAPI subCause to string
 * --------------------------------------------------------------------------*/
static const char * gattdSubCause2Str( uint16_t wSubCause )
{
    return ( BQB_ErrorCode2Str(wSubCause) );
}


/*--------------------------------------------------------------------------*/
/*---------------------------- commands ------------------------------------*/
/*--------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
 * command reg
 * --------------------------------------------------------------------------*/

TGATTDemoResult BQB_Register(PGATTDemo pGATTDemo,
                             TGATTDemoParseResult *pParseResult)
{
    TGATTDemoResult Result = gattdResultError;

    if (
        blueAPI_RegisterReq( (void *)pGATTDemo,
                             (void *)BQB_BlueAPICallback)
    )
    {
        BQB_CmdPrint( pGATTDemo, "--> RegisterReq\r\n%s", pGATTDemo->CmdIF.cPrompt );
        Result = gattdResultOk;
    }
    return ( Result );
}

/*----------------------------------------------------------------------------
 * command rel
 * --------------------------------------------------------------------------*/

TGATTDemoResult BQB_Release(PGATTDemo pGATTDemo,
                            TGATTDemoParseResult *pParseResult)
{
    TGATTDemoResult Result = gattdResultError;



    if (
        blueAPI_ReleaseReq()
    )
    {
        BQB_CmdPrint( pGATTDemo, "--> ReleaseReq\r\n%s", pGATTDemo->CmdIF.cPrompt );
        Result = gattdResultOk;
    }

    return ( Result );
}



/*----------------------------------------------------------------------------
 * command sauth
 * --------------------------------------------------------------------------*/

TGATTDemoResult BQB_GATTSecurityReq(PGATTDemo pGATTDemo,
                                    TGATTDemoParseResult *pParseResult)
{
    bool result = false;
    int idx = pParseResult->dwParameter[0];

    if (idx > (BQB_MAX_LINKS - 1))
        return (gattdResultError);

    if (pGATTDemo->linkTable[idx].local_MDL_ID_Valid)
    {
        result = blueAPI_GATTSecurityReq(pGATTDemo->linkTable[idx].local_MDL_ID,
                                         pParseResult->dwParameter[1],
                                         pParseResult->dwParameter[2]
                                        );
    }

    if (result)
    {
        BQB_CmdPrint( pGATTDemo, "--> GATTSecurityReq requirements=0x%x minKeySize=%d\r\n%s",
                      pParseResult->dwParameter[0],
                      pParseResult->dwParameter[1],
                      pGATTDemo->CmdIF.cPrompt
                    );
    }
    else
        BQB_CmdPrint(pGATTDemo, "idx= %d --> GATTSecurityReq failed ", pParseResult->dwParameter[0]);

    return result ? gattdResultOk : gattdResultError;
}


/* change gatt mtu size */
TGATTDemoResult BQB_ChangeMtuSize(PGATTDemo pGATTDemo, TGATTDemoParseResult *pParseResult)
{
    uint16_t mtu_size = pParseResult->dwParameter[1];
    int idx = pParseResult->dwParameter[0];

    if (idx > (BQB_MAX_LINKS - 1))
    {
        return (gattdResultError);
    }
    
    if(0x17 <= mtu_size && 0xb8 >= mtu_size)
    {
        //otp_str_data.gEfuse_UpperStack_s.att_max_mtu_size = mtu_size;
        //SetMtuSizeOnlyForBQB(mtu_size);
        BQB_CmdPrint(pGATTDemo, "change mtu size to 0x%x successfully\r\n", mtu_size);
    }
    else
    {
        BQB_CmdPrint(pGATTDemo, "invalid mtu size\r\n");
        return gattdResultError;
    }
    return gattdResultOk;
}

/*----------------------------------------------------------------------------
 * command srvreg
 * --------------------------------------------------------------------------*/

TGATTDemoResult BQB_ServiceRegister(PGATTDemo pGATTDemo,
                                    TGATTDemoParseResult *pParseResult)
{
    PGATTDService   pService;
    TGATTDemoResult Result = gattdResultError;
    uint8_t service_off = 0;
    uint8_t service_index = 0;
    uint8_t *p_service = NULL;

    switch (pParseResult->dwParameter[0])
    {
    case 0:
        if ( (pGATTDemo->iServiceCount >= BQB_GATT_SUPPORT_SERVICE_MAX) ||
                pGATTDemo->Service[pGATTDemo->iServiceCount].Used )
        {
            return ( Result );
        }
        
        pService       = &pGATTDemo->Service[pGATTDemo->iServiceCount];
        pService->Used = true;
        pService->ServiceID = Service_Info[pGATTDemo->iServiceCount].ser_id;
        
        for (service_index = 0; service_index < pGATTDemo->iServiceCount; service_index++)
        {
            service_off += Service_Info[service_index].num_attr;
        }
        if (SMALL_DATABASE == BQB_GATT_Database_Index)
        {
            p_service = (uint8_t*)SmallDatabase + service_off * sizeof(TAttribAppl);
        }
        else if (LARGE_DATABASE1 == BQB_GATT_Database_Index)
        {
            p_service = (uint8_t*)LargeDatabase1 + service_off * sizeof(TAttribAppl);
        }
        else
        {
            BQB_CmdPrint(pGATTDemo, "BQB_ServiceRegister: invalid database index!!");
        }

        if ( blueAPI_GATTServiceRegisterReq(
#if (GATTDEMO_APPLIC_PROVIDED_SERVICES)
                    Service_Info[pGATTDemo->iServiceCount].num_attr,        /* nbrOfAttrib */
                    p_service           /* pService  */
#else
                    0, NULL
#endif
                )
           )
        {
            BQB_CmdPrint( pGATTDemo,
#if (GATTDEMO_APPLIC_PROVIDED_SERVICES)
                          "--> GATTServiceRegisterReq: pService=0x%x, nbrOfAttrib=%d, srvChanged=%d\r\n%s",
                          p_service,
                          Service_Info[pGATTDemo->iServiceCount].num_attr,
                          pParseResult->dwParameter[1],
#else
                          "--> GATTServiceRegisterReq: ServiceID=%d\r\n%s", pService->ServiceID,
#endif
                          pGATTDemo->CmdIF.cPrompt );

            Result = gattdResultOk;
        }
        break;



    default:
        break;
    }


    return ( Result );
}

/*----------------------------------------------------------------------------
 * command srvrel
 * --------------------------------------------------------------------------*/

TGATTDemoResult BQB_ServiceRelease(PGATTDemo pGATTDemo,
                                   TGATTDemoParseResult *pParseResult)
{
    TGATTDemoResult Result = gattdResultError;
    int i = pParseResult->dwParameter[0];

    if (
        pGATTDemo->Service[i].Used &&
        (pGATTDemo->Service[i].pServiceHandle != NULL)
    )
    {
        if (blueAPI_GATTServiceReleaseReq( pGATTDemo->Service[i].pServiceHandle))
        {
            BQB_CmdPrint( pGATTDemo, "--> GATTServiceReleaseReq: serviceHandle=0x%x\r\n%s",
                          pGATTDemo->Service[i].pServiceHandle,
                          pGATTDemo->CmdIF.cPrompt );
            Result = gattdResultOk;
        }
    }
    return (Result);
}

/*----------------------------------------------------------------------------
 * get attribute value and send update message
 * --------------------------------------------------------------------------*/
static TGATTDemoResult BQB_AttribUpdateSend(PGATTDemo pGATTDemo,
        TGATTDemoParseResult *pParseResult, bool InitCall )
{
    PGATTDService   pService;
    TGATTDemoResult Result = gattdResultError;
    PGATTLink pLink = BQB_LinkFindByRole(pGATTDemo, BQB_CON_ROLE_SLAVE);
    if (pLink == NULL)
    {
        Result = gattdResultError;
        BQB_CmdPrint(pGATTDemo, "gattdAttribUpdateSend:not find the slave device\r\n");
        return (Result);
    }

    if (InitCall)
    {
        /* determine service to be used */
#if (!GATTDEMO_APPLIC_PROVIDED_SERVICES)                                        /* */
        if ( pParseResult->iParameterCount > 2 )
            pGATTDemo->iUpdateServiceIndex = pParseResult->dwParameter[2];
        else
            pGATTDemo->iUpdateServiceIndex = GATTDEMO_MAIN_SERVICE_INDEX;
#endif
    }
    //pService = &pGATTDemo->Service[pGATTDemo->iUpdateServiceIndex];
    pService = &pGATTDemo->Service[pParseResult->dwParameter[2]];

    if (
        pService->Used &&
        (pService->pServiceHandle != NULL)
    )
    {
        uint16_t     wAttribIndex;
        uint16_t     wLength;
        uint8_t    * pData;

        if (InitCall)
        {
            /*if (pParseResult->dwParameter[0] == 0)
            {
                wAttribIndex = BQB_NOTIFICATION_VALUE_INDEX;
            }
            else
            {
                wAttribIndex = pParseResult->dwParameter[0];
            }*/
            wAttribIndex = pParseResult->dwParameter[3];
        }
        else
        {
            wAttribIndex = pGATTDemo->iUpdateAttribIndex;
        }
        pGATTDemo->iUpdateAttribIndex = wAttribIndex;

        /* service/attribute specific read operation */
        if ( !pLink->Connected ||  /* tifnan: find the attribute */
                (BQB_AttribGet(pGATTDemo, pService, wAttribIndex, 0, &wLength, &pData) == GATT_SUCCESS))
        {
            union
            {
                uint32_t d;
                void    *p;
            } requestHandle;
            uint8_t   * pBuffer = NULL;
            uint16_t    wOffset = pGATTDemo->wDsDataOffset + 3;

            if ( !pLink->Connected )
            {
                /* not connected with client, update req. may be used to find out     */
                /* if the attribute is configured for indications or notifications .. */
                wLength = 0;
            }
            else
            {
                if ( pData == NULL )
                {
                    /* value is directly accessible by BT stack */
                    wLength = 0;
                }
                else
                {
                    /* copy attribute value to buffer position that allows re-usage by stack */
                    /* without copying ..                                                   */
                    if ( blueAPI_BufferGet(
                                pGATTDemo->wDsPoolId,
                                wLength,
                                wOffset,
                                (void **)&pBuffer) == blueAPI_CauseSuccess)
                    {
                        memcpy( pBuffer + wOffset, pData, wLength );
                    }
                    else
                    {
                        return ( gattdResultError );
                    }
                }
            }

            requestHandle.d = pGATTDemo->wUpdReqHandle;
            if ( blueAPI_GATTAttributeUpdateReq(pBuffer,
                                                pService->pServiceHandle,
                                                requestHandle.p,
                                                wAttribIndex,
                                                wLength,
                                                wOffset
                                               )
               )
            {
                BQB_CmdPrint( pGATTDemo,
                              "--> GATTAttributeUpdateReq: serviceHandle=0x%x, attribIndex=%d\r\n%s",
                              pService->pServiceHandle, wAttribIndex,
                              pGATTDemo->CmdIF.cPrompt );
                pGATTDemo->wUpdReqHandle++;
                pGATTDemo->iUpdateSent++;
                Result = gattdResultOk;
            }
            else
            {
                BQB_CmdPrint( pGATTDemo,
                              "!!! illegal parameter (e.g. offset too small)\r\n%s",
                              pGATTDemo->CmdIF.cPrompt );
                if ( pBuffer != NULL )
                    blueAPI_BufferRelease( pBuffer);
            }
        }
    }

    return ( Result );
}

/*----------------------------------------------------------------------------
 * command update
 * --------------------------------------------------------------------------*/

TGATTDemoResult BQB_AttribUpdate(PGATTDemo pGATTDemo,
                                 TGATTDemoParseResult *pParseResult)
{
    int iSent, iUpdateCnt;
    TGATTDemoResult Result = gattdResultOk;
    PGATTLink pLink = BQB_LinkFindByRole(pGATTDemo, BQB_CON_ROLE_SLAVE);
    if (pLink == NULL)
    {
        Result = gattdResultError;
        BQB_CmdPrint(pGATTDemo, "gattdAttribUpdate:not find the slave device\r\n");
        return (Result);
    }
    if ( pLink->Connected )
        pGATTDemo->iUpdateCnt = pParseResult->dwParameter[1];
    else
    {
        pGATTDemo->iUpdateCnt = 1;

    }
    if ( pGATTDemo->iUpdateCnt == 0 )           /* repetition count */
    {
        pGATTDemo->iUpdateCnt = 1;
    }
    iUpdateCnt = pGATTDemo->iUpdateCnt;
    pGATTDemo->iUpdateSent      = 0;
    pGATTDemo->iUpdateConfirmed = 0;
    iSent = 0;

    do
    {
        /* call may modify pGATTDemo->iUpdateCnt value (e.g. more than one */
        /* value in notification ..):                                      */
        Result = BQB_AttribUpdateSend(pGATTDemo, pParseResult, (iSent == 0 ? true : false) );
        iUpdateCnt = pGATTDemo->iUpdateCnt;
        iSent++;
    }
    while ( (Result == gattdResultOk) &&
            (iSent < iUpdateCnt) && (iSent < pLink->wDsCredits) );

    return ( Result );
}

/*----------------------------------------------------------------------------
 * continue update sequence
 * --------------------------------------------------------------------------*/

int BQB_AttribUpdateContinue( PGATTDemo pGATTDemo )
{
    TGATTDemoResult Result = BQB_AttribUpdateSend( pGATTDemo, NULL, false );

    return ( Result == gattdResultOk ? 0 : -1 );
}

/* XXXXMJMJ reasonable place for type <<Flags>> .. definitions needed !!!!: */
/* XXXXMJMJ (sdp_code.h is not appropriate !!!) */
static uint8_t bBQBAdData0[] =
{
    /* Core spec. Vol. 3, Part C, Chapter 18 */
    /* Flags */
    0x02,             /* length     */                              /* TP/ADV/BV-03-C */
    //XXXXMJMJ 0x01, 0x06,      /* type="flags", data="bit 1: LE General Discoverable Mode", BR/EDR not supp. */
    0x01, 0x02,      /* type="flags", data="bit 1: LE General Discoverable Mode" */

    /* Service */
    0x03,             /* length     */                                  /* TP/ADV/BV-01-C */
    0x02,            /* type="More 16-bit UUIDs available" */
    LO_WORD(0x181f),
    HI_WORD(0x181f),

    /* local name */                                /* TP/ADV/BV-02-C */
    0x04,               /* length     */
    0x08,
    0x52, 0x54, 0x4b,  /* RTK */

    /* place holder for Local Name, filled by BT stack. if not present */
    /* BT stack appends Local Name.                                    */
#if 0
    0x05,             /* length     */
    0x09,            /* type="Complete local name" */
    0x53, 0x54, 0x4F, 0x30,     /* STO0 */
#endif

    /* Manufacturer Specific Data */              /* TP/ADV/BV-04-C */
    0x03,           /* length     */
    0xff,
    0x5D, 0x00,     /* RealTech*/

    /* TX Power */
    0x02,
    0x0A,
    0x80,                                           /* TP/ADV/BV-05-C */

    /* AD Type - Slave Connection Interval Range */     /* TP/ADV/BV-08-C */
    0x05,
    0x12,
    0x06, 0x00, 0x80, 0xc0,

    /* service solicication */
    0x03,                                           /* TP/ADV/BV-09-C */
    0x14,
    0x00,
    0x18

#if 0
    , 0x04, 0xFF, 0x00, 0x0D, 0xBE, 0xEF /* manufactor (TI) specific data */
#endif
};

/* for passing bqb gap adv cases */
static const uint8_t bBQBAdData1[] =
{
    /* service data */
    0x05,                                                   /* TP/ADV/BV-10-C */
    0x16,
    0x00,
    0x18,
    0xBE,
    0xEF,

    /* type="Appearance" */                                             /* TP/ADV/BV-11-C */
    0x03,             /* length     */
    0x19,
    0x41, 0x04,      /* Running Walking Sensor: In-Shoe */

    /* AD Type - Public Target Address */       /* TP/ADV/BV-12-C */
    0x07,
    0x17,
    0x00, 0xe0, 0x77, 0x88, 0x99, 0xaa,

    /* AD Type - Random Target Address */       /* TP/ADV/BV-13-C */
    0x07,
    0x18,
    0x20, 0xe0, 0x77, 0x88, 0x99, 0x02
};

/* for passing bqb gap adv cases */
static const uint8_t bBQBAdData2[] =
{
    //0x05, 0x12, 0x00, 0x20, 0x00, 0x40,   /* slave con param */

    /* AD Type - LE Bluetooth Device Address */                         /* TP/ADV/BV-15-C */
    0x07,       /* length */
    0x1B,
    0x87, 0x99, 0x23, 0x4c, 0xe0, 0x00,
    //0x00, 0xe0, 0x4c, 0x23, 0x99, 0x87,

    /* LE role */                                               /* TP/ADV/BV-16-C */
    0x02,         /* length     */
    0x1c,
    0x02,   /* Peripheral and Central Role supported,Peripheral Role preferred for connection establishment*/

    /* AD Type Advertising Interval */                              /* TP/ADV/BV-14-C */
    0x03,   /* length */
    0x1A,
    0x23,
    0x88
};


static const uint8_t bBQBScanRespData[] =
{
    /* Core spec. Vol. 3, Part C, Chapter 18 */

    0x03,             /* length     */
    0x02,            /* type="More 16-bit UUIDs available" */

    LO_WORD(0x181f),
    HI_WORD(0x181f)
};


/*----------------------------------------------------------------------------
 * command ade
 * --------------------------------------------------------------------------*/

STATIC TGATTDemoResult BQB_SendLEAdvertiseReq(PGATTDemo               pGATTDemo,
        TBlueAPI_LEAdvMode      advMode,
        uint8_t *                  remote_BD,
        TBlueAPI_RemoteBDType   remote_BD_type)
{
    bool               result;

    result = blueAPI_LEAdvertiseReq(advMode);
    if (result)
    {
        BQB_CmdPrint( pGATTDemo, "--> LEAdvertiseReq: advMode=%d, remote_BD=[%s], type=%d\r\n%s",
                      advMode,
                      blueAPI_StringizeBd(remote_BD),
                      remote_BD_type,
                      pGATTDemo->CmdIF.cPrompt );
    }

    return result ? gattdResultOk : gattdResultError;
}

extern TGATTDBdAddr gattdTestRemoteBd[];
TGATTDemoResult BQB_CmdSetAdvertisingEnable(PGATTDemo pGATTDemo,
        TGATTDemoParseResult *pParseResult)
{
    TBlueAPI_LEAdvMode advMode = blueAPI_LEAdvModeDisabled;

    if (1 == pParseResult->dwParameter[0])
    {
        advMode = blueAPI_LEAdvModeEnabled;
    }
    else if (0 ==  pParseResult->dwParameter[0])
    {
        advMode = blueAPI_LEAdvModeDisabled;

    }
    else
    {
        advMode = blueAPI_LEAdvModeDirectedHighDuty;        /* tifnan add for test */
        return ( BQB_SendLEAdvertiseReq(pGATTDemo,
                                        advMode,
                                        (uint8_t*)&gattdTestRemoteBd[0],
                                        blueAPI_RemoteBDTypeLEPublic)
               );

    }
    return ( BQB_SendLEAdvertiseReq(pGATTDemo,
                                    advMode,
                                    NULL,
                                    blueAPI_RemoteBDTypeLEPublic)
           );

}

/*----------------------------------------------------------------------------
 * command addata
 * --------------------------------------------------------------------------*/

STATIC TGATTDemoResult BQB_SendLEAdvertiseDataSetReq(PGATTDemo            pGATTDemo,
        TBlueAPI_LEDataType  dataType,
        uint8_t                 dataLength,
        uint8_t *               data)
{
    bool result;

    result = blueAPI_LEAdvertiseDataSetReq(dataType,
                                           dataLength,
                                           data
                                          );
    if (result)
    {
        BQB_CmdPrint( pGATTDemo, "--> LEAdvertiseDataSetReq: dataType=%d, dataLength=%d\r\n%s",
                      dataType, dataLength,
                      pGATTDemo->CmdIF.cPrompt );
    }

    return result ? gattdResultOk : gattdResultError;
}

TGATTDemoResult BQB_SetAdvertisingData(PGATTDemo pGATTDemo, TGATTDemoParseResult *pParseResult)
{
    if (ADV_DATA_IND_0 == AdvDataIndex) /* for passing TP/ADV/BV-01/02/03/04/05/08/09-C cases */
    {
        return  BQB_SendLEAdvertiseDataSetReq(pGATTDemo,
                                              blueAPI_LEDataTypeAdvertisingData,
                                              sizeof(bBQBAdData0),
                                              (uint8_t *)bBQBAdData0);
    }
    else if (ADV_DATA_IND_1 == AdvDataIndex) /* for passing TP/ADV/BV-10/11/12/13-C cases */
    {
        return  BQB_SendLEAdvertiseDataSetReq(pGATTDemo,
                                              blueAPI_LEDataTypeAdvertisingData,
                                              sizeof(bBQBAdData1),
                                              (uint8_t *)bBQBAdData1);
    }
    else if (ADV_DATA_IND_2 == AdvDataIndex) /* for passing TP/ADV/BV-14/15/16-C cases */
    {
        return  BQB_SendLEAdvertiseDataSetReq(pGATTDemo,
                                              blueAPI_LEDataTypeAdvertisingData,
                                              sizeof(bBQBAdData2),
                                              (uint8_t *)bBQBAdData2);
    }
    return gattdResultOk;
}

/*----------------------------------------------------------------------------
 * command adp
 * --------------------------------------------------------------------------*/

TGATTDemoResult BQB_CmdSetAdvertisingParameters(PGATTDemo pGATTDemo,
        TGATTDemoParseResult *pParseResult)
{
    int idx = pParseResult->dwParameter[0];
    bool result;

    if (idx > (BQB_MAX_LINKS - 1))
    {
        return gattdResultError;
    }

    if ((pParseResult->dwParameter[1] == blueAPI_LEAdvTypeDirectedLowDuty) || (pParseResult->dwParameter[1] == blueAPI_LEAdvTypeDirectedHighDuty))
    {
        if (!pGATTDemo->linkTable[idx].RemoteBdValid)
        {
            BQB_CmdPrint(pGATTDemo, "idx = %d,remote bd is not valid", idx);
            return gattdResultError;
        }
    }

    if (pParseResult->dwParameter[1] == blueAPI_LEAdvTypeNonConnectable)
    {
        result = blueAPI_LEAdvertiseParameterSetReq(
                 (TBlueAPI_LEAdvType)pParseResult->dwParameter[1],
                 blueAPI_LEFilterAny,
                 blueAPI_LEFilterAny,
                 0xA5,
                 0xD0,
                 blueAPI_LocalBDTypeLEPublic,
                 NULL,
                 blueAPI_RemoteBDTypeLEPublic
                                                   );
    }
    else if (pParseResult->dwParameter[1] == blueAPI_LEAdvTypeScannable)
    {
        result = blueAPI_LEAdvertiseParameterSetReq(
                 (TBlueAPI_LEAdvType)pParseResult->dwParameter[1],
                 blueAPI_LEFilterAny,
                 blueAPI_LEFilterAny,
                 0xA4,
                 0xE0,
                 blueAPI_LocalBDTypeLEPublic,
                 NULL,
                 blueAPI_RemoteBDTypeLEPublic
                                                   );
    }
    else
    {
        result = blueAPI_LEAdvertiseParameterSetReq(
                 (TBlueAPI_LEAdvType)pParseResult->dwParameter[1],
                 blueAPI_LEFilterAny,
                 blueAPI_LEFilterAny,
                 0x20, /* 20ms */
                 0x30, /* 30ms */
                 blueAPI_LocalBDTypeLEPublic,
                 ((pParseResult->dwParameter[1] == blueAPI_LEAdvTypeDirectedLowDuty)|| (pParseResult->dwParameter[1] == blueAPI_LEAdvTypeDirectedHighDuty)) ?
                 pGATTDemo->linkTable[idx].RemoteBd : NULL,
                 blueAPI_RemoteBDTypeLEPublic
                                                   );
    }

    if (result)
    {
        BQB_CmdPrint( pGATTDemo, "--> LEAdvertiseParameterSetReq\r\n%s", pGATTDemo->CmdIF.cPrompt );
    }

    return result ? gattdResultOk : gattdResultError;
}

/*----------------------------------------------------------------------------
 * command add
 * --------------------------------------------------------------------------*/

TGATTDemoResult BQB_CmdSetDirectedAdvertising(PGATTDemo pGATTDemo,
        TGATTDemoParseResult *pParseResult)
{
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

    return (BQB_SendLEAdvertiseReq(pGATTDemo,
                                   blueAPI_LEAdvModeDirectedHighDuty,
                                   pGATTDemo->linkTable[idx].RemoteBd,
                                   blueAPI_RemoteBDTypeLEPublic)
           );
}


/*----------------------------------------------------------------------------
 * command con
 * --------------------------------------------------------------------------*/

TGATTDemoResult BQB_ConnectReq(PGATTDemo pGATTDemo,
                               TGATTDemoParseResult *pParseResult)
{
    TGATTDemoResult Result = gattdResultError;
    int   idx = pParseResult->dwParameter[0];

    if (idx > (BQB_MAX_LINKS - 1))
    {
        return gattdResultError;
    }

    if ( pGATTDemo->linkTable[idx].RemoteBdValid)
    {
        TBlueAPI_RemoteBDType  bdType;
        uint8_t *              pRemoteBd = (pParseResult->dwParameter[2] == 0) ?
                                           pGATTDemo->linkTable[idx].RemoteBd : NULL;

        switch (pParseResult->dwParameter[1])
        {
        case 0:
            bdType = blueAPI_RemoteBDTypeLEPublic;
            break;

        case 1:
            bdType = blueAPI_RemoteBDTypeLERandom;
            break;

        case 8:
            bdType = blueAPI_RemoteBDTypeLEResolved;
            break;

        default:
            bdType = blueAPI_RemoteBDTypeClassic;
            break;
        }

        if ( blueAPI_ConnectGATTMDLReq(pRemoteBd,
                                       bdType,
                                       blueAPI_LocalBDTypeLEPublic,
                                       0x10,    /* scanInterval       */
                                       0x10,    /* scanWindow         */
                                       1000,    /* scanTimeout 10s    */
                                       8,       /* connIntervalMin    */
                                       8,       /* connIntervalMax    */
                                       0,       /* connLatency        */
                                       300,      /* supervisionTimeout */
                                       14
                                      ) )
        {
            Result = gattdResultOk;
            pGATTDemo->linkTable[idx].role = BQB_CON_ROLE_MASTER;
            BQB_CmdPrint( pGATTDemo, "--> GATTConnectReq: remote_BD=[%s], type=%d\r\n%s",
                          blueAPI_StringizeBd(pRemoteBd), bdType,
                          pGATTDemo->CmdIF.cPrompt );
        }
    }

    return ( Result );
}

/*----------------------------------------------------------------------------
 * command disc
 * --------------------------------------------------------------------------*/

TGATTDemoResult BQB_DisconnectReq(PGATTDemo pGATTDemo,
                                  TGATTDemoParseResult *pParseResult)
{
    TGATTDemoResult Result = gattdResultError;
    int idx = pParseResult->dwParameter[0];

    if (idx > (BQB_MAX_LINKS - 1))
    {
        return (gattdResultError);
    }
    if (pGATTDemo->linkTable[idx].local_MDL_ID_Valid )
    {
        if ( blueAPI_DisconnectMDLReq( pGATTDemo->linkTable[idx].local_MDL_ID,
                                       blueAPI_CauseConnectionDisconnect )
           )
        {
            Result = gattdResultOk;
            BQB_CmdPrint( pGATTDemo, "--> GATTDisconnectReq: local_MDL_ID=%d\r\n%s",
                          pGATTDemo->linkTable[idx].local_MDL_ID,
                          pGATTDemo->CmdIF.cPrompt );
        }
    }

    return ( Result );
}




/*----------------------------------------------------------------------------
 * send GATTDiscoveryReq through BlueAPI
 * --------------------------------------------------------------------------*/
static TGATTDemoResult BQB_SendGATTDiscoveryReq(
    PGATTDemo                    pGATTDemo,
    TBlueAPI_GATTDiscoveryType   discoveryType,
    TGATTDemoParseResult        *pParseResult)
{
    uint16_t        wUUID = 0;
    uint8_t *       pUuid128 = NULL;
    TGATTDemoResult Result = gattdResultError;
    uint8_t uuid128[16] = {0};
    uint8_t i = 0;
    uint8_t j = 0;
    uint8_t k = 0;

    int idx = pParseResult->dwParameter[0];

    if (idx > BQB_MAX_LINKS - 1)
    {
        return gattdResultError;
    }

    if ( pGATTDemo->linkTable[idx].Connected )
    {
        switch ( discoveryType )
        {
        default:
            wUUID = 0;
            break;

        case blueAPI_GATTDiscoveryServices:
        case blueAPI_GATTDiscoveryServiceByUUID:
            if (pParseResult->dwParameter[4] == 0)  /* all services */
            {
                wUUID = pParseResult->dwParameter[4];   /* 0 */
            }
            else if (pParseResult->dwParameter[3] == 0 && pParseResult->dwParameter[4] != 0) /* 16 bit uuid*/
            {
                wUUID = pParseResult->dwParameter[4];
            }
            else if (pParseResult->dwParameter[3] == 1 && pParseResult->dwParameter[4] != 0) /* 128 bit uuid*/
            {
                for (i = 4; i > 0; i--) /* 128 bit uuid 32bit X 4 */
                {
                    for (j = 0; j < 4; j++) /* 32bit  = 8bit X 4 */
                    {
                        uuid128[k++] = pParseResult->dwParameter[4 + i - 1] >> (j * 8);
                    }
                }
                pUuid128 = &uuid128[0];
            }
            else
            {
                BQB_CmdPrint( pGATTDemo, " parameter error!!");
            }

            /* set handle range defaults: */
            if ( pParseResult->dwParameter[1] == 0 )
            {
                pParseResult->dwParameter[1] = 1;
            }
            if ( pParseResult->dwParameter[2] == 0 )
            {
                pParseResult->dwParameter[2] = 0xFFFF;
            }
            break;
        }

        pGATTDemo->linkTable[idx].wEndingHandle = pParseResult->dwParameter[2];

        if ( blueAPI_GATTDiscoveryReq(pGATTDemo->linkTable[idx].local_MDL_ID,
                                      discoveryType,
                                      pParseResult->dwParameter[1], /* start */
                                      pParseResult->dwParameter[2], /* end   */
                                      wUUID,     /* uuid16 */
                                      pUuid128   /* pUuid128 */
                                     )
           )
        {
            Result = gattdResultOk;
            if (NULL == pUuid128)
            {
                BQB_CmdPrint( pGATTDemo,
                              "idx = %d --> GATTDiscoveryReq: local_MDL_ID=%d, type=%d, start=0x%04x, end=0x%04x, UUID16=0x%04x\r\n%s",
                              idx, pGATTDemo->linkTable[idx].local_MDL_ID, discoveryType,
                              pParseResult->dwParameter[1], /* start */
                              pParseResult->dwParameter[2], /* end   */
                              wUUID,     /* uuid16 */
                              pGATTDemo->CmdIF.cPrompt );
            }
            else
            {
                BQB_CmdPrint( pGATTDemo,
                              "idx = %d --> GATTDiscoveryReq: local_MDL_ID=%d, type=%d, start=0x%04x, end=0x%04x, UUID128=0x%02x%02x\
          %02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\r\n%s",
                              idx, pGATTDemo->linkTable[idx].local_MDL_ID, discoveryType,
                              pParseResult->dwParameter[1], /* start */
                              pParseResult->dwParameter[2], /* end   */
                              uuid128[15], uuid128[14], uuid128[13], uuid128[12], uuid128[11], uuid128[10], uuid128[9], uuid128[8], uuid128[7],
                              uuid128[6], uuid128[5], uuid128[4], uuid128[3], uuid128[2], uuid128[1], uuid128[0],
                              pGATTDemo->CmdIF.cPrompt );
            }
        }
    }

    return ( Result );
}

/*----------------------------------------------------------------------------
 * command srvdis
 * --------------------------------------------------------------------------*/

TGATTDemoResult BQB_ServiceDiscovery(PGATTDemo pGATTDemo,
                                     TGATTDemoParseResult *pParseResult)
{
    TGATTDemoResult Result;
    Result = BQB_SendGATTDiscoveryReq(
                 pGATTDemo,
                 (pParseResult->dwParameter[4] != 0) ?
                 blueAPI_GATTDiscoveryServiceByUUID : blueAPI_GATTDiscoveryServices,
                 pParseResult);

    return ( Result );
}

/*----------------------------------------------------------------------------
 * command reldis
 * --------------------------------------------------------------------------*/

TGATTDemoResult BQB_RelationshipDiscovery(PGATTDemo pGATTDemo,
        TGATTDemoParseResult *pParseResult)
{
    return ( BQB_SendGATTDiscoveryReq( pGATTDemo,
                                       blueAPI_GATTDiscoveryRelationship,
                                       pParseResult ) );
}

/*----------------------------------------------------------------------------
 * command chardis
 * --------------------------------------------------------------------------*/

TGATTDemoResult BQB_CharacteristicDiscovery(PGATTDemo pGATTDemo,
        TGATTDemoParseResult *pParseResult)
{
    return ( BQB_SendGATTDiscoveryReq( pGATTDemo,
                                       blueAPI_GATTDiscoveryCharacteristics,
                                       pParseResult ) );
}

/*----------------------------------------------------------------------------
 * command charddis
 * --------------------------------------------------------------------------*/

TGATTDemoResult BQB_CharacDescriptorDiscovery(PGATTDemo pGATTDemo,
        TGATTDemoParseResult *pParseResult)
{
    return ( BQB_SendGATTDiscoveryReq( pGATTDemo,
                                       blueAPI_GATTDiscoveryCharacDescriptors,
                                       pParseResult ) );
}

/*----------------------------------------------------------------------------
 * command lsuuid
 * --------------------------------------------------------------------------*/

TGATTDemoResult BQB_ListUUIDs(PGATTDemo pGATTDemo,
                              TGATTDemoParseResult *pParseResult)
{
    /* same procedure on ATT layer: */
    return ( BQB_CharacDescriptorDiscovery( pGATTDemo, pParseResult) );
}

/*----------------------------------------------------------------------------
 * command read (using handle)
 * --------------------------------------------------------------------------*/

TGATTDemoResult BQB_AttribRead(PGATTDemo pGATTDemo,
                               TGATTDemoParseResult *pParseResult)
{
    TGATTDemoResult Result = gattdResultError;
    int idx = pParseResult->dwParameter[0];

    if (idx > (BQB_MAX_LINKS - 1))
    {
        return (gattdResultError);
    }

    if (pGATTDemo->linkTable[idx].Connected )
    {
        if ( blueAPI_GATTAttributeReadReq(pGATTDemo->linkTable[idx].local_MDL_ID,
                                          blueAPI_GATTReadTypeBasic,
                                          pParseResult->dwParameter[2],  /* readOffset */
                                          pParseResult->dwParameter[1],  /* handle     */
                                          0,          /* endHandle   */
                                          0,          /* UUID16      */
                                          NULL        /* pUUID128    */
                                         )
           )
        {
            Result = gattdResultOk;
            BQB_CmdPrint( pGATTDemo,
                          "idx = %d--> GATTAttributeReadReq: local_MDL_ID=%d, type=%d, readOffset=%d, handle=0x%04x\r\n%s",
                          idx, pGATTDemo->linkTable[idx].local_MDL_ID, blueAPI_GATTReadTypeBasic,
                          pParseResult->dwParameter[2], /* readOffset */
                          pParseResult->dwParameter[1], /* handle     */
                          pGATTDemo->CmdIF.cPrompt );
        }
    }

    return ( Result );
}

/* multi read */
#if 0
TGATTDemoResult BQB_AttribReadMulti(PGATTDemo pGATTDemo, TGATTDemoParseResult *pParseResult)
{
    uint16_t num = 2;
    uint16_t handles[4];
    TGATTDemoResult Result = gattdResultError;
    int idx = pParseResult->dwParameter[0];
    uint8_t i = 0;

    /* max 4 handles */
    if(pParseResult->iParameterCount > 5)
    {
        return (gattdResultError);
    }
    if(pParseResult->iParameterCount > 2)
    {
        num = pParseResult->iParameterCount - 1;
        for(i = 0; i < num; i++)
        {
            handles[i] = pParseResult->dwParameter[i + 1];
        }
    }

    if(idx > (BQB_MAX_LINKS-1))
    {
        return (gattdResultError);
    }
    
    if (pGATTDemo->linkTable[idx].Connected )
    {
        if ( blueAPI_GATTAttributeReadMultipleReq(pGATTDemo->linkTable[idx].local_MDL_ID, num, handles))
        {
            Result = gattdResultOk;
            BQB_CmdPrint( pGATTDemo,
                           "idx = %d--> GATTAttributeReadMultipleReq: local_MDL_ID=%d, num=%d\r\n%s",
                           idx, pGATTDemo->linkTable[idx].local_MDL_ID, num,
                           pGATTDemo->CmdIF.cPrompt );
        }
    }
    return( Result );
}
#endif

/*----------------------------------------------------------------------------
 * command read (using UUID)
 * --------------------------------------------------------------------------*/

TGATTDemoResult BQB_AttribReadUUID(PGATTDemo pGATTDemo,
                                   TGATTDemoParseResult *pParseResult)
{
    uint16_t        wUUID = 0;
    uint8_t *       pUuid128 = NULL;
    TGATTDemoResult Result = gattdResultError;
    int idx = pParseResult->dwParameter[0];
    uint8_t uuid128[16] = {0};
    uint8_t i = 0;
    uint8_t j = 0;
    uint8_t k = 0;

    if (idx > (BQB_MAX_LINKS - 1))
    {
        return gattdResultError;
    }

    if (pGATTDemo->linkTable[idx].Connected )
    {
        if (pParseResult->dwParameter[3] == 0 && pParseResult->dwParameter[4] != 0) /* 16 bit uuid*/
        {
            wUUID = pParseResult->dwParameter[4];
        }
        else if (pParseResult->dwParameter[3] == 1 && pParseResult->dwParameter[4] != 0) /* 128 bit uuid*/
        {
            for (i = 4; i > 0; i--) /* 128 bit uuid 32bit X 4 */
            {
                for (j = 0; j < 4; j++) /* 32bit  = 8bit X 4 */
                {
                    uuid128[k++] = pParseResult->dwParameter[4 + i - 1] >> (j * 8);
                }
            }
            pUuid128 = &uuid128[0];
        }
        else
        {
            BQB_CmdPrint( pGATTDemo, " parameter error!!");
        }

        /* set handle range defaults: */
        if ( pParseResult->dwParameter[1] == 0 )
        {
            pParseResult->dwParameter[1] = 1;
        }
        if ( pParseResult->dwParameter[2] == 0 )
        {
            pParseResult->dwParameter[2] = 0xFFFF;
        }

        if ( blueAPI_GATTAttributeReadReq(pGATTDemo->linkTable[idx].local_MDL_ID,
                                          blueAPI_GATTReadTypeByUUID,
                                          0,  /* readOffset */
                                          pParseResult->dwParameter[1],  /* startHandle */
                                          pParseResult->dwParameter[2],  /* endHandle   */
                                          wUUID,          /* UUID16      */
                                          pUuid128        /* pUUID128    */
                                         )
           )
        {
            Result = gattdResultOk;
            if (NULL == pUuid128)
            {
                BQB_CmdPrint( pGATTDemo,
                              "idx = %d--> GATTAttributeReadReq: local_MDL_ID=%d, type=%d, start=0x%04x, end=0x%04x, UUID16=0x%04x\r\n%s",
                              idx, pGATTDemo->linkTable[idx].local_MDL_ID, blueAPI_GATTReadTypeByUUID,
                              pParseResult->dwParameter[1], /* start */
                              pParseResult->dwParameter[2], /* end   */
                              wUUID,
                              pGATTDemo->CmdIF.cPrompt );
            }
            else
            {
                BQB_CmdPrint( pGATTDemo,
                              "idx = %d --> GATTDiscoveryReq: local_MDL_ID=%d, type=%d, start=0x%04x, end=0x%04x, UUID128=0x%02x%02x"
                              "%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\r\n%s",
                              idx, pGATTDemo->linkTable[idx].local_MDL_ID, blueAPI_GATTReadTypeByUUID,
                              pParseResult->dwParameter[1], /* start */
                              pParseResult->dwParameter[2], /* end   */
                              uuid128[15], uuid128[14], uuid128[13], uuid128[12], uuid128[11], uuid128[10], uuid128[9], uuid128[8], uuid128[7],
                              uuid128[6], uuid128[5], uuid128[4], uuid128[3], uuid128[2], uuid128[1], uuid128[0],
                              pGATTDemo->CmdIF.cPrompt );
            }
        }
    }

    return ( Result );
}

/*----------------------------------------------------------------------------
 * command write
 * --------------------------------------------------------------------------*/

TGATTDemoResult BQB_AttribWrite(PGATTDemo pGATTDemo,
                                TGATTDemoParseResult *pParseResult)
{
    TGATTDemoResult Result = gattdResultError;
    int idx = pParseResult->dwParameter[0];
    uint16_t hanlde = pParseResult->dwParameter[1];
    uint16_t length = pParseResult->dwParameter[3];
    void* pBuffer = NULL;
    uint16_t wOffset = pGATTDemo->wDsDataOffset + 3;
    uint8_t* pData = NULL;

    if (idx > (BQB_MAX_LINKS - 1))
    {
        return gattdResultError;
    }

    if ( pGATTDemo->linkTable[idx].Connected )
    { 
        pData = &DesValV2D_9[0];

        /* copy attribute value to buffer position that allows re-usage by stack */
        /* without copying ..                                                   */
        if ( blueAPI_BufferGet(pGATTDemo->wDsPoolId, length, wOffset, (void **)&pBuffer) == blueAPI_CauseSuccess )
        {
            TBlueAPI_GATTWriteType   writeType = (pParseResult->dwParameter[2] == 1) ?
                                                 blueAPI_GATTWriteTypeCommand : blueAPI_GATTWriteTypeRequest;

            memcpy( ((uint8_t *)pBuffer) + wOffset, pData, length );

            if (blueAPI_GATTAttributeWriteReq(
                        pBuffer,
                        pGATTDemo->linkTable[idx].local_MDL_ID,
                        writeType,
                        hanlde,  /* handle */
                        length,                       /* length */
                        wOffset                        /* offset */
                    )
               )
            {
                Result = gattdResultOk;
                BQB_CmdPrint( pGATTDemo,
                              "idx= %d--> GATTAttributeWriteReq: local_MDL_ID=%d, type=%d, handle=0x%04x, length=%d\r\n%s",
                              idx, pGATTDemo->linkTable[idx].local_MDL_ID, writeType,
                              pParseResult->dwParameter[1], /* handle */
                              length,
                              pGATTDemo->CmdIF.cPrompt );
            }
            else
            {
                blueAPI_BufferRelease(pBuffer);
            }
        }
    }

    return ( Result );
}

TGATTDemoResult BQB_AttribWriteEx(PGATTDemo pGATTDemo, TGATTDemoParseResult *pParseResult)
{
    TGATTDemoResult Result = gattdResultError;
    int idx = pParseResult->dwParameter[0];

    if (idx > (BQB_MAX_LINKS - 1))
    {
        return gattdResultError;
    }

    if ( pGATTDemo->linkTable[idx].Connected )
    {
        uint16_t     wLength = 0;
        uint8_t*     pData = NULL;
        void*        pBuffer;
        uint16_t     wOffset = pGATTDemo->wDsDataOffset + 3;

        /* service/attribute specific write data setup */
        wLength = pParseResult->dwParameter[3];
        pData  = (uint8_t*)&pParseResult->dwParameter[4];
        if (wLength != 0 && pData != NULL)
        {
            /* copy attribute value to buffer position that allows re-usage by stack */
            /* without copying ..                                                   */
            if ( blueAPI_BufferGet(
                        pGATTDemo->wDsPoolId,
                        wLength,
                        wOffset,
                        (void **)&pBuffer) == blueAPI_CauseSuccess )
            {
                TBlueAPI_GATTWriteType   writeType = (pParseResult->dwParameter[2] == 1) ?
                                                     blueAPI_GATTWriteTypeCommand : blueAPI_GATTWriteTypeRequest;

                memcpy( ((uint8_t *)pBuffer) + wOffset, pData, wLength );

                if ( blueAPI_GATTAttributeWriteReq(
                            pBuffer,
                            pGATTDemo->linkTable[idx].local_MDL_ID,
                            writeType,
                            pParseResult->dwParameter[1],  /* handle */
                            wLength,                       /* length */
                            wOffset                        /* offset */
                        )
                   )
                {
                    Result = gattdResultOk;
                    BQB_CmdPrint( pGATTDemo,
                                  "idx= %d--> GATTAttributeWriteReq: local_MDL_ID=%d, type=%d, handle=0x%04x, length=%d\r\n%s",
                                  idx, pGATTDemo->linkTable[idx].local_MDL_ID, writeType,
                                  pParseResult->dwParameter[1], /* handle */
                                  wLength,
                                  pGATTDemo->CmdIF.cPrompt );
                }
                else
                {
                    blueAPI_BufferRelease(pBuffer);
                }
            }
        }
    }

    return ( Result );
}


#if (BQB_PREPARE_WRITE_SUPPPRT)
/* prepare write */
TGATTDemoResult BQB_AttribPrepareWrite(PGATTDemo pGATTDemo, TGATTDemoParseResult *pParseResult)
{
    TGATTDemoResult Result = gattdResultError;
    int idx = pParseResult->dwParameter[0];
    pGATTDemo->pwrite_handle = pParseResult->dwParameter[1];
    pGATTDemo->pwrite_attri_len = pParseResult->dwParameter[3];
  
    if(idx > (BQB_MAX_LINKS-1))
    {
        return (gattdResultError);
    }

    if ( pGATTDemo->linkTable[idx].Connected )
    {
        uint16_t     wLength = 0;
        uint8_t      *pData = NULL; 
        void         *pBuffer = NULL;
        uint16_t     wOffset = pGATTDemo->wDsDataOffset + 1;
        uint16_t     writeOffset = pParseResult->dwParameter[2];
        
        pData = &DesValV2D_9[0];

        //wLength = pGATTDemo->pwrite_attri_len;
        wLength = 18;   //for mtu_size = 23
        
        /* service/attribute specific write data setup */
        if ( blueAPI_BufferGet(pGATTDemo->wDsPoolId,
                               wLength,
                               wOffset,
                               (void **)&pBuffer) == blueAPI_CauseSuccess )
        {
            memcpy( ((uint8_t *)pBuffer) + wOffset, pData, wLength );

            if ( blueAPI_GATTAttributePrepareWriteReq(
                               pBuffer,
                               pGATTDemo->linkTable[idx].local_MDL_ID,
                               pGATTDemo->pwrite_handle,        /* handle */
                               wLength,                         /* length */
                               writeOffset,
                               wOffset                          /* offset */
                                          )
            )
            {
                Result = gattdResultOk;
                BQB_CmdPrint( pGATTDemo,
                    "idx= %d--> GATTAttributePrepareWriteReq: local_MDL_ID=%d,handle=0x%x length=%d valueOffset=%d\r\n%s",
                    idx,
                    pGATTDemo->linkTable[idx].local_MDL_ID,
                    pGATTDemo->pwrite_handle,
                    wLength,
                    writeOffset,
                    pGATTDemo->CmdIF.cPrompt );
            }
            else
            {
                blueAPI_BufferRelease(pBuffer);
                BQB_CmdPrint( pGATTDemo," blueAPI_GATTAttributePrepareWriteReq failed ");
            }
     
        }
        else
        {
            BQB_CmdPrint( pGATTDemo," blueAPI_BufferGet failed ");
        }
    }

    return( Result );
}

TGATTDemoResult BQB_SendPrepareWrite(PGATTDemo pGATTDemo,
                                          uint16_t local_MDL_ID,
                                          uint16_t handle,
                                          uint16_t writeOffset,
                                          uint16_t length)
{
  TGATTDemoResult Result = gattdResultError;
  PGATTLink pLink = NULL;
  
  pLink = BQB_LinkFindByLocal_MDL_ID(pGATTDemo, local_MDL_ID);
  
  if (pLink->Connected)
  {
    uint16_t     wLength;
    uint8_t      *pData = &DesValV2D_9[0];
    void       * pBuffer;
    uint16_t     wOffset = pGATTDemo->wDsDataOffset + 1;

    wLength = length;
    /* service/attribute specific write data setup */
    if ( blueAPI_BufferGet(
                           pGATTDemo->wDsPoolId,
                           wLength,
                           wOffset,
                           (void **)&pBuffer) == blueAPI_CauseSuccess )
    {
      memcpy( ((uint8_t *)pBuffer)+wOffset, pData+writeOffset, wLength );

      if ( blueAPI_GATTAttributePrepareWriteReq(
                               pBuffer,
                               local_MDL_ID,
                               handle,  /* handle */
                               wLength,                       /* length */
                               writeOffset,
                               wOffset                        /* offset */
                                          )
         )
      {
        Result = gattdResultOk;
        BQB_CmdPrint( pGATTDemo,
            "-> GATTAttributePrepareWriteReq: local_MDL_ID=%d, length=%d valueOffset=%d\r\n%s",
            local_MDL_ID, 
            wLength,
            writeOffset,
            pGATTDemo->CmdIF.cPrompt );
      }
      else
      {
        blueAPI_BufferRelease(pBuffer);
      }
     
    }
  }

  return( Result );
}

/*----------------------------------------------------------------------------
 * command ewrite
 * --------------------------------------------------------------------------*/
TGATTDemoResult BQB_SendExecuteWrite(PGATTDemo pGATTDemo,uint16_t local_MDL_ID, uint8_t flags)
{
  TGATTDemoResult Result = gattdResultError;
  PGATTLink pLink = NULL;
  pLink = BQB_LinkFindByLocal_MDL_ID(pGATTDemo,local_MDL_ID);

  if ( pLink->Connected )
  {
    if(blueAPI_GATTAttributeExecuteWriteReq(local_MDL_ID, flags))
    {
      Result = gattdResultOk;
      BQB_CmdPrint( pGATTDemo,
              "--> GATTAttributeExecuteWriteReq: local_MDL_ID=%d, flags=%d\r\n%s",
              local_MDL_ID,flags,
              pGATTDemo->CmdIF.cPrompt );
    }
  }

  return( Result );
}

/* execute write, user call it through uart command */
TGATTDemoResult BQB_AttribExecuteWrite(PGATTDemo pGATTDemo, TGATTDemoParseResult *pParseResult)
{
    int idx = pParseResult->dwParameter[0];
    uint8_t flag = pParseResult->dwParameter[1];
    
    if (idx > (BQB_MAX_LINKS - 1))
    {
        return gattdResultError;
    }

    if (pGATTDemo->linkTable[idx].local_MDL_ID_Valid)
    {
        BQB_SendExecuteWrite(pGATTDemo, pGATTDemo->linkTable[idx].local_MDL_ID, flag);
    }

    return gattdResultOk;
}

#endif


/*----------------------------------------------------------------------------
 * command scan
 * --------------------------------------------------------------------------*/
TGATTDemoResult BQB_CmdScanEnable(PGATTDemo pGATTDemo,
                                  TGATTDemoParseResult *pParseResult)
{
    bool                result;
    TBlueAPI_LEScanMode scanMode = blueAPI_LEScanDisabled;

    if (pParseResult->dwParameter[0])
    {
        if (pParseResult->dwParameter[1])
        {
            scanMode = blueAPI_LEScanActive;
        }
        else
        {
            scanMode = blueAPI_LEScanPassive;
        }
    }

    result = blueAPI_LEScanReq(scanMode,
                               0x0080,
                               0x0020,
                               (TBlueAPI_LEFilterPolicy)pParseResult->dwParameter[3],
                               blueAPI_LocalBDTypeLEPublic,
                               pParseResult->dwParameter[2]
                              );
    if (result)
    {
        BQB_CmdPrint( pGATTDemo, "--> LEScanReq: scanMode=%d\r\n%s",
                      scanMode,
                      pGATTDemo->CmdIF.cPrompt );
    }

    return result ? gattdResultOk : gattdResultError;
}

/*----------------------------------------------------------------------------
 * command conupdreq
 * --------------------------------------------------------------------------*/

TGATTDemoResult BQB_ConUpdateReq(PGATTDemo pGATTDemo,
                                 TGATTDemoParseResult *pParseResult)
{
    TGATTDemoResult result = gattdResultOk;
    int idx = pParseResult->dwParameter[0];

    if (idx > (BQB_MAX_LINKS - 1))
    {
        return gattdResultError;
    }

    if (pGATTDemo->linkTable[idx].local_MDL_ID_Valid)
    {
        result = (TGATTDemoResult)blueAPI_LEConnectionUpdateReq(pGATTDemo->linkTable[idx].local_MDL_ID,
                                               pParseResult->dwParameter[1],
                                               pParseResult->dwParameter[2],
                                               pParseResult->dwParameter[3],
                                               pParseResult->dwParameter[4]
                                              );
    }

    if (result)
    {
        BQB_CmdPrint( pGATTDemo, "--> LEConnectionUpdateReq: local_MDL_ID=%d\r\n%s",
                      pGATTDemo->linkTable[idx].local_MDL_ID,
                      pGATTDemo->CmdIF.cPrompt );
    }

    return result ? gattdResultOk : gattdResultError;
}

/*----------------------------------------------------------------------------
 * command conupdresp
 * --------------------------------------------------------------------------*/

TGATTDemoResult BQB_ConUpdateResp(PGATTDemo pGATTDemo,
                                  TGATTDemoParseResult *pParseResult)
{
    TGATTDemoResult result = gattdResultOk;
    int idx = pParseResult->dwParameter[0];

    TBlueAPI_Cause  cause = pParseResult->dwParameter[1] ?
                            blueAPI_CauseAccept : blueAPI_CauseReject;
    if (idx > (BQB_MAX_LINKS - 1))
    {
        return (gattdResultError);
    }

    if (pGATTDemo->linkTable[idx].local_MDL_ID_Valid)
    {
        result = (TGATTDemoResult)blueAPI_LEConnectionUpdateConf(pGATTDemo->linkTable[idx].local_MDL_ID,
                                                cause
                                               );
    }

    if (result)
    {
        BQB_CmdPrint( pGATTDemo,
                      "--> LEConnectionUpdateConf: local_MDL_ID=%d, cause=%s\r\n%s",
                      pGATTDemo->linkTable[idx].local_MDL_ID,
                      blueAPI_CauseString(cause),
                      pGATTDemo->CmdIF.cPrompt );
    }

    return result ? gattdResultOk : gattdResultError;
}

/*----------------------------------------------------------------------------
 * command addwl
 * --------------------------------------------------------------------------*/

STATIC TGATTDemoResult BQB_SendLEModifyWhitelistReq(PGATTDemo pGATTDemo,
        TBlueAPI_LEWhitelistOp  operation,
        uint8_t *                  remote_BD,
        TBlueAPI_RemoteBDType   remote_BD_type)
{
    bool result;

    result = blueAPI_LEModifyWhitelistReq(operation,
                                          remote_BD,
                                          remote_BD_type
                                         );
    if (result)
    {
        BQB_CmdPrint( pGATTDemo,
                      "--> LEModifyWhitelistReq: op=%d, remote_BD=%s, type=%d\r\n%s",
                      operation,
                      blueAPI_StringizeBd(remote_BD),
                      remote_BD_type,
                      pGATTDemo->CmdIF.cPrompt );
    }

    return result ? gattdResultOk : gattdResultError;
}


TGATTDemoResult BQB_CmdAddToWhitelist(PGATTDemo pGATTDemo,
                                      TGATTDemoParseResult *pParseResult)
{
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

    return ( BQB_SendLEModifyWhitelistReq(pGATTDemo,
                                          blueAPI_LEWhitelistOpAdd,
                                          pGATTDemo->linkTable[idx].RemoteBd,
                                          blueAPI_RemoteBDTypeLEPublic)
           );
}

/*----------------------------------------------------------------------------
 * command remwl
 * --------------------------------------------------------------------------*/

TGATTDemoResult BQB_CmdRemoveFromWhitelist(PGATTDemo pGATTDemo,
        TGATTDemoParseResult *pParseResult)
{
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

    return (BQB_SendLEModifyWhitelistReq(pGATTDemo,
                                         blueAPI_LEWhitelistOpRemove,
                                         pGATTDemo->linkTable[idx].RemoteBd,
                                         blueAPI_RemoteBDTypeLEPublic)
           );
}

/*----------------------------------------------------------------------------
 * command clearwl
 * --------------------------------------------------------------------------*/

TGATTDemoResult BQB_CmdClearWhitelist(PGATTDemo pGATTDemo,
                                      TGATTDemoParseResult *pParseResult)
{
    return ( BQB_SendLEModifyWhitelistReq(pGATTDemo,
                                          blueAPI_LEWhitelistOpClear,
                                          NULL,
                                          blueAPI_RemoteBDTypeLEPublic)
           );
}

/**
 * @brief bqb send indication/notifiction.
 *
 * @param  pGATTDemo -- pointer to global struct GATTDemo.
 * @param  pParseResult -- pointer to the command parameters.
 * @return the service index.
 * @retval send indication/notification result.
*/
TGATTDemoResult BQB_SendNotInd(PGATTDemo pGATTDemo,
                               TGATTDemoParseResult *pParseResult)
{
    //otp_str_data.gEfuse_Bqb_s.bqb_is_notification = pParseResult->dwParameter[1];  /* for notification or indication */
    TGATTDemoResult Result = gattdResultError;
    int idx = pParseResult->dwParameter[0];

    if (idx > (BQB_MAX_LINKS - 1))
    {
        return (gattdResultError);
    }

    Result = BQB_AttribUpdateSend(pGATTDemo, pParseResult, true);
    return Result;
}

#if (GATTDEMO_TESTMODE_SUPPORT)
/*----------------------------------------------------------------------------
 * command testmode
 * --------------------------------------------------------------------------*/

TGATTDemoResult BQB_CmdTestmode(PGATTDemo pGATTDemo,
                                TGATTDemoParseResult *pParseResult)
{
    TGATTDemoResult Result = gattdResultError;

    if ( pParseResult->iParameterCount > 0 )
    {
        TBlueAPI_Testmode  testmode;

        testmode = (pParseResult->dwParameter[0] == 1) ?
                   blueAPI_TestmodeHCIDIRECT : blueAPI_TestmodeBEDUTM;
        if ( blueAPI_TestmodeReq(NULL,  testmode ) )
        {
            Result = gattdResultOk;
            BQB_CmdPrint( pGATTDemo,
                          "--> TestmodeReq: mode=%d\r\n%s",
                          testmode,
                          pGATTDemo->CmdIF.cPrompt );
        }
    }

    return ( Result );
}
#endif /* (GATTDEMO_TESTMODE_SUPPORT) */

TGATTDemoResult BQB_CmdConLEChan(PGATTDemo pGATTDemo,
                                  TGATTDemoParseResult *pParseResult)
{
    bool            result = gattdResultOk;
    int idx = pParseResult->dwParameter[0];
    uint16_t le_psm = 0x45;
    uint16_t credit = 5;
    uint16_t mtu = 100;
    uint16_t mps = 23;
      
    if(idx > (BQB_MAX_LINKS-1))
        return (gattdResultError);

    if(pParseResult->iParameterCount>1)
        le_psm = pParseResult->dwParameter[1];  
    if(pParseResult->iParameterCount>2)
        mtu = pParseResult->dwParameter[2];
    if(pParseResult->iParameterCount>3)
        mps = pParseResult->dwParameter[3];
    if(pParseResult->iParameterCount>4)
        credit = pParseResult->dwParameter[4];
      
    if(pGATTDemo->linkTable[idx].local_MDL_ID_Valid)
    {
        result = blueAPI_CreateLEDataChannelReq( pGATTDemo->linkTable[idx].local_MDL_ID,
                                                le_psm,
                                                mtu,
                                                mps,
                                                credit,
                                                0
                                                );
    }
    
    if (result)
    {
        BQB_CmdPrint( pGATTDemo,
                        "--> CreateLEDataChannelReq: local_MDL_ID=0x%04x\r\n",
                        pGATTDemo->linkTable[idx].local_MDL_ID
                        );
    }
    
    return result ? gattdResultOk : gattdResultError;
}

TGATTDemoResult BQB_CmdDiscLEChan(PGATTDemo pGATTDemo,
                                  TGATTDemoParseResult *pParseResult)
{
    TGATTDemoResult Result = gattdResultOk;
    int idx = pParseResult->dwParameter[0];
    int channel = pParseResult->dwParameter[1];
    PGATTLink pLink;
    PGATTLEChannel pChan;
    
    if(idx > (BQB_MAX_LINKS-1))
        return (gattdResultError);

    pLink = &pGATTDemo->linkTable[idx];
    pChan = BQB_LEChanFind(pGATTDemo,pLink,channel);
    if(pChan == NULL)
        return (gattdResultError);
    if(pLink->Connected)
    {
        if((pChan->isDataChanConnected))
        {
            Result = (TGATTDemoResult)blueAPI_DisconnectLEDataChannelReq( pGATTDemo->linkTable[idx].local_MDL_ID,
                                          pChan->channel);
        }
    }
  
    if (Result)
    {
        BQB_CmdPrint( pGATTDemo,
                                "--> DisconnectLEDataChannelReq: local_MDL_ID=0x%04x, channel=0x%04x\r\n",
                                pGATTDemo->linkTable[idx].local_MDL_ID,
                                pChan->channel
                                );
    }

    return( Result );
}

TGATTDemoResult BQB_CmdCredit(PGATTDemo pGATTDemo,
                                  TGATTDemoParseResult *pParseResult)
{
    TGATTDemoResult Result = gattdResultOk;
    int idx = pParseResult->dwParameter[0];
    int channel = pParseResult->dwParameter[1];
    int credit = 2;
    PGATTLink pLink;
    PGATTLEChannel pChan;
    if(idx > (BQB_MAX_LINKS-1))
        return (gattdResultError);

    pLink = &pGATTDemo->linkTable[idx];
    if(pParseResult->iParameterCount >= 3)
        credit = pParseResult->dwParameter[2];
    pChan = BQB_LEChanFind(pGATTDemo,pLink,channel);
    if(pChan == NULL)
        return (gattdResultError);
    if(pLink->Connected)
    {
        if((pChan->isDataChanConnected))
        {
            Result = (TGATTDemoResult)blueAPI_SendLEFlowControlCreditReq( pGATTDemo->linkTable[idx].local_MDL_ID,
                                          pChan->channel, credit);
        }
    }
  
    if (Result)
    {
        BQB_CmdPrint( pGATTDemo,
                   "--> SendLEFlowControlCreditReq: local_MDL_ID=0x%04x, channel=0x%04x credit=%d\r\n",
                   pGATTDemo->linkTable[idx].local_MDL_ID,
                   pChan->channel,
                   credit
                   );
    }

    return( Result );
}

TGATTDemoResult BQB_CmdLEData(PGATTDemo pGATTDemo,
                                  TGATTDemoParseResult *pParseResult)
{
    TGATTDemoResult Result = gattdResultOk;
    int idx = pParseResult->dwParameter[0];
    int channel = pParseResult->dwParameter[1];
    int dataIdx = pParseResult->dwParameter[2];
    PGATTLink pLink;
    PGATTLEChannel pChan;
    uint8_t *      pBuffer;
    uint8_t *      value;
    uint16_t       wLength = 4;
    uint16_t       wOffset = pGATTDemo->wDsDataOffset + 1;
    uint8_t test[4] = {0x01,0x02,0x03,0x04};
    uint8_t test_6[60] = {0x01,0x01,0x01,0x01,0x01,0x02,0x02,0x02,0x02,0x02,
                              0x03,0x03,0x03,0x03,0x03,0x04,0x04,0x04,0x04,0x04,
                              0x05,0x05,0x05,0x05,0x05,0x06,0x06,0x06,0x06,0x06,
                              0x07,0x07,0x07,0x07,0x07,0x08,0x08,0x08,0x08,0x08,
                              0x09,0x09,0x09,0x09,0x09,0x00,0x00,0x00,0x00,0x00,
                              0x01,0x01,0x01,0x01,0x01,0x02,0x02,0x02,0x02,0x02};

    if(idx > (BQB_MAX_LINKS-1))
        return (gattdResultError);

    pLink = &pGATTDemo->linkTable[idx];

    pChan = BQB_LEChanFind(pGATTDemo,pLink,channel);
    if(dataIdx == 0)
    {
        wLength = 1;
        value = test;
    }
    else
    {
        wLength = 60;
        value = test_6;
    }
    if(pChan == NULL)
        return (gattdResultError);
    if(pLink->Connected)
    {
        if((pChan->isDataChanConnected))
        {
            if ( blueAPI_BufferGet(
                           pGATTDemo->wDsPoolId,
                           wLength,
                           wOffset,
                           (void **)&pBuffer) == blueAPI_CauseSuccess )
            {
                memcpy( pBuffer+wOffset, value, wLength );
                Result = (TGATTDemoResult)blueAPI_LEDataReq( pBuffer,pGATTDemo->linkTable[idx].local_MDL_ID,
                                          pChan->channel, wLength,wOffset);
            }
            else 
            {
                BQB_CmdPrint( pGATTDemo,
                   "--> SendLEData:get buffer failed\r\n"
                   );
            }
        }
    }
  
    if (Result)
    {
        BQB_CmdPrint( pGATTDemo,
                   "--> SendLEData: local_MDL_ID=0x%04x, channel=0x%04x\r\n",
                   pGATTDemo->linkTable[idx].local_MDL_ID,
                   pChan->channel
                   );
    }

    return( Result );
}

TGATTDemoResult BQB_CmdLESec(PGATTDemo pGATTDemo,
                                  TGATTDemoParseResult *pParseResult)
{
    TGATTDemoResult Result = gattdResultOk;
    int le_psm = pParseResult->dwParameter[0];
    int mode = pParseResult->dwParameter[2];
    int keysize = pParseResult->dwParameter[3];

    Result = (TGATTDemoResult)blueAPI_LEPsmSecuritySetReq(le_psm,pParseResult->dwParameter[1],
                                       (TBlueAPI_LESecurityMode)mode,keysize);


    return( Result );
}

TGATTDemoResult BQB_CmdScBqb(PGATTDemo pGATTDemo,
        TGATTDemoParseResult *pParseResult)
{
    TGATTDemoResult Result = gattdResultOk;
    int test_case = pParseResult->dwParameter[0];
        
    //blueAPI_UserDefined(&test_case);
     
    return ( Result );
}




/*--------------------------------------------------------------------------*/
/*-------------------------- BlueAPI messages ------------------------------*/
/*--------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_RegisterRsp
 * --------------------------------------------------------------------------*/

static bool BQB_Handle_RegisterRsp( PGATTDemo            pGATTDemo,
                                    PBlueAPI_RegisterRsp pRegisterRsp )
{
    BQB_CmdPrint( pGATTDemo, "<-- RegisterRsp: cause=%s\r\n%s",
                  blueAPI_CauseString(pRegisterRsp->cause),
                  pGATTDemo->CmdIF.cPrompt );

    if ( pRegisterRsp->cause == blueAPI_CauseSuccess )
    {
        /* init. service specific data */
        BQB_ServiceInitData(pGATTDemo); /* tifnan: not always needed */
    }
    else
    {
    }

    return ( true );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_ReleaseRsp
 * --------------------------------------------------------------------------*/

static bool BQB_Handle_ReleaseRsp( PGATTDemo            pGATTDemo,
                                   PBlueAPI_ReleaseRsp  pReleaseRsp )
{
    BQB_CmdPrint( pGATTDemo, "<-- ReleaseRsp: cause=%s\r\n%s",
                  blueAPI_CauseString(pReleaseRsp->cause),
                  pGATTDemo->CmdIF.cPrompt );

    if ( pReleaseRsp->cause == blueAPI_CauseSuccess )
    {

    }

    return ( true );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_ActInfo
 * --------------------------------------------------------------------------*/

static bool BQB_Handle_ActInfo( PGATTDemo pGATTDemo, PBlueAPI_ActInfo pActInfo )
{
    BQB_CmdPrint( pGATTDemo,
                  "<-- ActInfo: cause=%s, version=%s, BD=%s\r\n%s",
                  blueAPI_CauseString(pActInfo->cause),
                  pActInfo->version,
                  blueAPI_StringizeBd(pActInfo->local_BD),
                  pGATTDemo->CmdIF.cPrompt );

    return ( true );
}


static void BQB_Handle_DeviceConfigSetRsp( PGATTDemo pGATTDemo, PBlueAPI_DeviceConfigSetRsp pDevCfgSetRsp )
{
    BQB_CmdPrint(pGATTDemo, "<-- DeviceConfigSetRsp: opCode=%d cause=%s\r\n%s",
                 pDevCfgSetRsp->opCode,
                 blueAPI_CauseString(pDevCfgSetRsp->cause),
                 pGATTDemo->CmdIF.cPrompt );
}

static bool BQB_Handle_CreateMDLInd( PGATTDemo pGATTDemo,
                                     PBlueAPI_CreateMDLInd pCreateMDLInd )
{
    TBlueAPI_Cause cause;
    PGATTLink pLink;
    BQB_CmdPrint( pGATTDemo, "<-- CreateMDLInd: remote_BD=[%s], remote_BD_type=%d, local_MDL_ID=%04x\r\n%s",
                  blueAPI_StringizeBd(pCreateMDLInd->remote_BD),
                  pCreateMDLInd->remote_BD_type,
                  pCreateMDLInd->local_MDL_ID,
                  pGATTDemo->CmdIF.cPrompt );
    pLink = BQB_LinkFindByBD(pGATTDemo, pCreateMDLInd->remote_BD);
    if (pLink)
    {
        if ( pLink->local_MDL_ID_Valid)
        {
            cause = blueAPI_CauseReject;
            BQB_CmdPrint( pGATTDemo,
                          "!! reject, the connection is already exist.\r\n%s",
                          pGATTDemo->CmdIF.cPrompt );
        }
        else
        {
            cause = blueAPI_CauseAccept;
            pLink->local_MDL_ID_Valid = true;
            pLink->local_MDL_ID       = pCreateMDLInd->local_MDL_ID;
        }
    }
    else
    {
        pLink = BQB_LinkAllocate(pGATTDemo, pCreateMDLInd->remote_BD);
        if (pLink)
        {
            cause = blueAPI_CauseAccept;
            pLink->local_MDL_ID_Valid = true;
            pLink->local_MDL_ID       = pCreateMDLInd->local_MDL_ID;
        }
        else
        {
            cause = blueAPI_CauseReject;
            BQB_CmdPrint( pGATTDemo,
                          "!! reject, no rescource.\r\n" );
        }
    }


    if ( blueAPI_CreateMDLConf( pCreateMDLInd->local_MDL_ID,
                                1,    /* XXXXMJMJ maxTPDUusCredits */
                                cause )
       )
    {
        BQB_CmdPrint( pGATTDemo, "--> CreateMDLConf: local_MDL_ID=%04x, cause=%s\r\n%s",
                      pCreateMDLInd->local_MDL_ID,
                      blueAPI_CauseString(cause),
                      pGATTDemo->CmdIF.cPrompt );
    }
    return ( true );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_ConnectMDLRsp
 * --------------------------------------------------------------------------*/

static bool BQB_Handle_ConnectMDLRsp( PGATTDemo pGATTDemo,
                                      PBlueAPI_ConnectMDLRsp pConnectMDLRsp )
{
    BQB_CmdPrint( pGATTDemo, "<-- ConnectMDLRsp: cause=%s\r\n%s",
                  blueAPI_CauseString(pConnectMDLRsp->cause),
                  pGATTDemo->CmdIF.cPrompt );

    return ( true );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_MCLStatusInfo
 * --------------------------------------------------------------------------*/

static bool BQB_Handle_MCLStatusInfo( PGATTDemo pGATTDemo,
                                      PBlueAPI_MCLStatusInfo pMCLStatusInfo )
{
    BQB_CmdPrint( pGATTDemo, "<-- MCLStatusInfo: status=%s, local_MCL_ID=%04x\r\n%s",
                  blueAPI_MCLStatusString(pMCLStatusInfo->status),
                  pMCLStatusInfo->local_MCL_ID,
                  pGATTDemo->CmdIF.cPrompt );

    return ( true );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_ConnectMDLInfo
 * --------------------------------------------------------------------------*/

static bool BQB_Handle_ConnectMDLInfo( PGATTDemo pGATTDemo,
                                       PBlueAPI_ConnectMDLInfo pConnectMDLInfo )
{
    PGATTLink pLink;
    BQB_CmdPrint( pGATTDemo, "<-- ConnectMDLInfo: dsPoolID=%04x, local_MDL_ID=%04x\r\n%s",
                  pConnectMDLInfo->dsPoolID,
                  pConnectMDLInfo->local_MDL_ID,
                  pGATTDemo->CmdIF.cPrompt );
    pLink = BQB_LinkFindByLocal_MDL_ID(pGATTDemo, pConnectMDLInfo->local_MDL_ID);
    if (pLink)
    {
        pLink->Connected     = true;
        pLink->wMTUSize      = pConnectMDLInfo->maxTPDUSize;
        pLink->wDsCredits    = pConnectMDLInfo->maxTPDUdsCredits;
        if (pLink->role != BQB_CON_ROLE_MASTER)
        {
            pLink->role = BQB_CON_ROLE_SLAVE;
        }
    }

    pGATTDemo->wDsPoolId     = pConnectMDLInfo->dsPoolID;
    pGATTDemo->wDsDataOffset = pConnectMDLInfo->dsDataOffset;
    pGATTDemo->wUpdReqHandle = 0x0012;    /* tifnan: bqb in large database 1, for notification */


#if (GATTDEMO_CCCD_COUNT)
    /* init. array of CCCD attribute index/value pairs */
    BQB_CCCDInitTable( pGATTDemo );
#endif

    if ( pGATTDemo->ConnectedInd != NULL )
    {
        (*pGATTDemo->ConnectedInd)( (void *)pGATTDemo );
    }

    return ( true );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_DisconnectMDLRsp
 * --------------------------------------------------------------------------*/

static bool BQB_Handle_DisconnectMDLRsp( PGATTDemo pGATTDemo,
        PBlueAPI_DisconnectMDLRsp pDisconnectMDLRsp )
{
    BQB_CmdPrint( pGATTDemo, "<-- DisconnectMDLRsp: cause=%s, local_MDL_ID=%04x\r\n%s",
                  blueAPI_CauseString(pDisconnectMDLRsp->cause),
                  pDisconnectMDLRsp->local_MDL_ID,
                  pGATTDemo->CmdIF.cPrompt );

    return ( true );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_DisconnectMDLInd
 * --------------------------------------------------------------------------*/

static bool BQB_Handle_DisconnectMDLInd( PGATTDemo pGATTDemo,
        PBlueAPI_DisconnectMDLInd pDisconnectMDLInd )
{
    BQB_CmdPrint( pGATTDemo, "<-- DisconnectMDLInd: cause=%s, local_MDL_ID=%04x\r\n%s",
                  blueAPI_CauseString(pDisconnectMDLInd->cause),
                  pDisconnectMDLInd->local_MDL_ID,
                  pGATTDemo->CmdIF.cPrompt );

    if ( blueAPI_DisconnectMDLConf( pDisconnectMDLInd->local_MDL_ID )
       )
    {
        BQB_CmdPrint( pGATTDemo, "--> DisconnectMDLConf: local_MDL_ID=%04x\r\n%s",
                      pDisconnectMDLInd->local_MDL_ID,
                      pGATTDemo->CmdIF.cPrompt );
    }
    return ( true );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_DeleteMDLInfo
 * --------------------------------------------------------------------------*/

static bool BQB_Handle_DeleteMDLInfo( PGATTDemo pGATTDemo,
                                      PBlueAPI_DeleteMDLInfo pDeleteMDLInfo )
{
    PGATTLink pLink;
    BQB_CmdPrint( pGATTDemo, "<-- DeleteMDLInfo: local_MDL_ID=%04x\r\n%s",
                  pDeleteMDLInfo->local_MDL_ID,
                  pGATTDemo->CmdIF.cPrompt );
    pLink = BQB_LinkFindByLocal_MDL_ID(pGATTDemo, pDeleteMDLInfo->local_MDL_ID);
    /* cleanup connection data */
    if (pLink)
    {
        pLink->Connected          = false;
        pLink->local_MDL_ID_Valid = false;
        pLink->local_MDL_ID       = 0;
        pLink->role = BQB_CON_ROLE_UNDEFINED;
        BQB_LEChanClear(pGATTDemo,pLink);
    }

    if (pGATTDemo->advertiseOnDisc)
    {
        BQB_SendLEAdvertiseReq(pGATTDemo, blueAPI_LEAdvModeEnabled,
                               NULL,
                               blueAPI_RemoteBDTypeLEPublic);
    }

    return ( true );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_InternalEventInfo
 * --------------------------------------------------------------------------*/

static bool BQB_Handle_InternalEventInfo( PGATTDemo pGATTDemo,
        PBlueAPI_InternalEventInfo pInternalEventInfo )
{
    BQB_CmdPrint( pGATTDemo, "<-- InternalEventInfo: Type=%s, Info=%08x, cause=%s\r\n%s",
                  blueAPI_InternalEventTypeString(pInternalEventInfo->eventType),
                  pInternalEventInfo->eventInfo,
                  blueAPI_CauseString(pInternalEventInfo->cause),
                  pGATTDemo->CmdIF.cPrompt );

    return ( true );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_ACLStatusInfo
 * --------------------------------------------------------------------------*/

static bool BQB_Handle_ACLStatusInfo( PGATTDemo pGATTDemo,
                                      PBlueAPI_ACLStatusInfo pACLStatusInfo )
{
    BQB_CmdPrint( pGATTDemo, "<-- ACLStatusInfo: BD=[%s] BDType=[%d] status=[%s] ",
                  blueAPI_StringizeBd(pACLStatusInfo->remote_BD),
                  pACLStatusInfo->remote_BD_type,
                  blueAPI_ACLStatusString(pACLStatusInfo->status) );

    switch (pACLStatusInfo->status)
    {


    case blueAPI_ACLAuthenticationStarted:
    case blueAPI_ACLAuthenticationSuccess:
    case blueAPI_ACLAuthenticationFailure:
    case blueAPI_ACLConnectionEncrypted:
    case blueAPI_ACLConnectionNotEncrypted:
        BQB_CmdPrint( pGATTDemo, "keyType=[%d] keySize=[%d]\r\n%s",
                      pACLStatusInfo->p.auth.keyType,
                      pACLStatusInfo->p.auth.keySize,
                      pGATTDemo->CmdIF.cPrompt );
        break;

    case blueAPI_ACLAddressResolved:
        BQB_CmdPrint( pGATTDemo, "resolvedBD=[%s] resolvedBDType=[%d]\r\n%s",
                      blueAPI_StringizeBd(pACLStatusInfo->p.resolve.remote_BD),
                      pACLStatusInfo->p.resolve.remote_BD_type,
                      pGATTDemo->CmdIF.cPrompt );
        break;

    case blueAPI_ACLConnectedLinkStatus:
        BQB_CmdPrint( pGATTDemo, "qual=[%d] rssi=[%d] failedC=[%d] txpwr=[%d] arssi=[%d]\r\n%s",
                      pACLStatusInfo->p.linkStatus.linkQuality,
                      pACLStatusInfo->p.linkStatus.rssi,
                      pACLStatusInfo->p.linkStatus.failedContacts,
                      pACLStatusInfo->p.linkStatus.txPower,
                      pACLStatusInfo->p.linkStatus.absoluteRssi,
                      pGATTDemo->CmdIF.cPrompt );
        break;

    default:
        BQB_CmdPrint( pGATTDemo, "\r\n%s", pGATTDemo->CmdIF.cPrompt );
        break;
    }

    return ( true );
}

static void BQB_Handle_GATTMtuSizeInfo(PGATTDemo pGATTDemo,
        PBlueAPI_GATTMtuSizeInfo  pMtuSizeInfo)
{
    PGATTLink pLink;
    BQB_CmdPrint( pGATTDemo, "<-- MtuSizeInfo: mtuSize=0x%x\r\n%s",
                              pMtuSizeInfo->mtuSize,
                              pGATTDemo->CmdIF.cPrompt );

    pLink = BQB_LinkFindByLocal_MDL_ID(pGATTDemo, pMtuSizeInfo->local_MDL_ID);
    if (pLink)
    {
        pLink->wMTUSize = pMtuSizeInfo->mtuSize;
    }
    else
    {
        BQB_CmdPrint( pGATTDemo, "<-- BQB_Handle_GATTMtuSizeInfo:not find the link\r\n%s",
                                  pGATTDemo->CmdIF.cPrompt );
    }
}


/*----------------------------------------------------------------------------
 * handle blueAPI_Event_GATTServiceRegisterRsp
 * --------------------------------------------------------------------------*/

static bool BQB_Handle_GATTServiceRegisterRsp(
    PGATTDemo                       pGATTDemo,
    PBlueAPI_GATTServiceRegisterRsp pGATTServiceRegisterRsp )
{
    BQB_CmdPrint( pGATTDemo, "<-- GATTServiceRegisterRsp: service=0x%x, cause=%s\r\n%s",
                  pGATTServiceRegisterRsp->serviceHandle,
                  blueAPI_CauseString(pGATTServiceRegisterRsp->cause),
                  pGATTDemo->CmdIF.cPrompt );

    if ( pGATTServiceRegisterRsp->cause == blueAPI_CauseSuccess )
    {
        TGATTDemoParseResult  ParseResult;

        /* save stack supplied service handle */
        pGATTDemo->Service[pGATTDemo->iServiceCount].pServiceHandle =
            pGATTServiceRegisterRsp->serviceHandle;
        pGATTDemo->iServiceCount++;

        if (pGATTDemo->iServiceCount == BQB_ServiceNum)
        {
            pGATTDemo->initLEServer    = true;
            pGATTDemo->advertiseOnDisc = true;
            if (pGATTDemo->initLEServer)
            {
                BQB_SendLEModifyWhitelistReq(pGATTDemo,
                                             blueAPI_LEWhitelistOpClear,
                                             NULL,
                                             blueAPI_RemoteBDTypeLEPublic
                                            );
            }
        }
        else
        {
            /* register next service */
            ParseResult.dwParameter[0] = 0;
            ParseResult.dwParameter[1] = 0;
            BQB_ServiceRegister( pGATTDemo, &ParseResult );
        }
    }
    else
    {
        pGATTDemo->Service[pGATTDemo->iServiceCount].Used           = false;
        pGATTDemo->Service[pGATTDemo->iServiceCount].pServiceHandle = NULL;
    }

    return ( true );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_GATTAttributeUpdateRsp
 * --------------------------------------------------------------------------*/

static bool BQB_Handle_GATTAttributeUpdateRsp(
    PGATTDemo                       pGATTDemo,
    PBlueAPI_GATTAttributeUpdateRsp pGATTAttributeUpdateRsp )
{
    bool  Error = false;

    BQB_CmdPrint( pGATTDemo,
                  "<-- GATTAttributeUpdateRsp: subCause=0x%x(%s), reqHandle=0x%x\r\n%s",
                  pGATTAttributeUpdateRsp->subCause,
                  gattdSubCause2Str(pGATTAttributeUpdateRsp->subCause),
                  pGATTAttributeUpdateRsp->requestHandle,
                  pGATTDemo->CmdIF.cPrompt );

    if ( pGATTAttributeUpdateRsp->cause == blueAPI_CauseSuccess )
    {
        pGATTDemo->iUpdateConfirmed++;
        if ( pGATTDemo->iUpdateCnt > pGATTDemo->iUpdateSent )
        {
            /* notification request accepted, continue with next one .. */
            if ( BQB_AttribUpdateContinue( pGATTDemo ) < 0 )
            {
                Error = true;
            }
        }
    }
    else if ( pGATTAttributeUpdateRsp->subCause == GATT_ERR_OUT_OF_RESOURCE )
    {
        Error = true;
    }
    else if ( (pGATTAttributeUpdateRsp->subCause == GATT_ERR_NOTIF_IND_CFG) &&
              (pGATTAttributeUpdateRsp->count > 0)
            )
    {
        /* list of BDs that configured the CCCD for notifications/indications */
        /* is appended */
        PBlueAPI_GATTAttributeUpdateListElement  pElement;
        int   i;

        pElement =
            (PBlueAPI_GATTAttributeUpdateListElement)(pGATTAttributeUpdateRsp->list + pGATTAttributeUpdateRsp->gap);
        for ( i = 0; i < pGATTAttributeUpdateRsp->count ; i++ )
        {
            BQB_CmdPrint( pGATTDemo, " BD=[%s], type=0x%x\r\n%s",
                          blueAPI_StringizeBd(pElement->remote_BD),
                          pElement->remote_BD_Type,
                          pGATTDemo->CmdIF.cPrompt );
            pElement++;
        }
    }

    if ( Error )
    {

        pGATTDemo->iUpdateCnt = 0;
    }
    else
    {
        /* some services need to do some post update (sequence) processing (RACP ..): */
        if ( (pGATTDemo->iUpdateCnt == pGATTDemo->iUpdateConfirmed) ||
                (pGATTAttributeUpdateRsp->cause != blueAPI_CauseSuccess)
           )
        {
            BQB_ServiceUpdateCallback( pGATTDemo,     /* tifnan: not always needed */
                                       pGATTAttributeUpdateRsp->subCause, pGATTAttributeUpdateRsp->attribIndex );
        }
    }

    return ( true );
}


/*----------------------------------------------------------------------------
 * handle blueAPI_Event_GATTAttributeUpdateStatusInd
 * --------------------------------------------------------------------------*/

static bool BQB_Handle_GATTAttributeUpdateStatusInd(
    PGATTDemo                             pGATTDemo,
    PBlueAPI_GATTAttributeUpdateStatusInd pGATTAttributeUpdateStatusInd )
{
    BQB_CmdPrint( pGATTDemo,
                  "<-- GATTAttributeUpdateStatusInd: subCause=0x%x(%s), reqHandle=0x%x, BD=[%s]\r\n%s",
                  pGATTAttributeUpdateStatusInd->subCause,
                  gattdSubCause2Str(pGATTAttributeUpdateStatusInd->subCause),
                  pGATTAttributeUpdateStatusInd->requestHandle,
                  blueAPI_StringizeBd(pGATTAttributeUpdateStatusInd->remote_BD),
                  pGATTDemo->CmdIF.cPrompt );

    if ( blueAPI_GATTAttributeUpdateStatusConf(
            pGATTAttributeUpdateStatusInd->serviceHandle,
            pGATTAttributeUpdateStatusInd->requestHandle,
            pGATTAttributeUpdateStatusInd->attribIndex
                                              )
       )
    {
        BQB_CmdPrint( pGATTDemo, "--> GATTAttributeUpdateStatusConf\r\n%s",
                      pGATTDemo->CmdIF.cPrompt );
    }

    /* check if update sequence should be continued */
    if ( pGATTAttributeUpdateStatusInd->cause == blueAPI_CauseSuccess )
    {
        pGATTDemo->iUpdateConfirmed++;
        if ( pGATTDemo->iUpdateCnt > pGATTDemo->iUpdateSent )
        {
            BQB_AttribUpdateContinue( pGATTDemo );
        }
    }
    else
    {
        pGATTDemo->iUpdateCnt = 0;
    }

#if 0
    /* some services need to do some post update (sequence) processing (RACP ..): */
    if ( pGATTDemo->iUpdateCnt == 0 )
    {
        BQB_ServiceUpdateCallback( pGATTDemo,
                                   pGATTAttributeUpdateStatusInd->subCause,
                                   pGATTAttributeUpdateStatusInd->attribIndex );
    }
#endif
    return ( true );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_GATTAttributeReadInd
 * --------------------------------------------------------------------------*/

static bool BQB_Handle_GATTAttributeReadInd(
    PGATTDemo                     pGATTDemo,
    PBlueAPI_GATTAttributeReadInd pGATTAttributeReadInd )
{
    PGATTDService  pService;
    uint8_t *      pBuffer;
    uint16_t       wLength = 0;
    uint16_t       wOffset = pGATTDemo->wDsDataOffset;
    uint8_t *      pData   = NULL;
    uint16_t       wCause  = GATT_ERR_ILLEGAL_PARAMETER;

    BQB_CmdPrint( pGATTDemo,
                  "<-- GATTAttributeReadInd: service=0x%x, idx=%d, offset=%d\r\n%s",
                  pGATTAttributeReadInd->serviceHandle,
                  pGATTAttributeReadInd->attribIndex,
                  pGATTAttributeReadInd->readOffset,
                  pGATTDemo->CmdIF.cPrompt );

    pService = BQB_ServiceFind( pGATTDemo, pGATTAttributeReadInd->serviceHandle );
    if ( pService != (PGATTDService)NULL )
    {
        /* service/attribute specific read operation.                          */
        /* check for authorization etc. has been done by GATT layer already .. */
        wCause  = BQB_AttribGet( pGATTDemo, pService, pGATTAttributeReadInd->attribIndex,
                                 pGATTAttributeReadInd->readOffset, &wLength, &pData );
    }

    /* send response */
    if ( wCause == GATT_SUCCESS )
    {
        /* copy attribute value to buffer position that allows re-usage by stack */
        /* without copying ..                                                   */
        if ( blueAPI_BufferGet(
                    pGATTDemo->wDsPoolId,
                    wLength,
                    wOffset,
                    (void **)&pBuffer) == blueAPI_CauseSuccess )
        {
            memcpy( pBuffer + wOffset, pData, wLength );
        }
        else
        {
            wCause = GATT_ERR_OUT_OF_RESOURCE;
        }
    }
    if ( wCause != GATT_SUCCESS )
    {
        pBuffer = NULL;
        wLength = 0;
    }

    if ( blueAPI_GATTAttributeReadConf(pBuffer,
                                       pGATTAttributeReadInd->local_MDL_ID,
                                       pService->pServiceHandle,
                                       (wCause != GATT_SUCCESS) ?
                                       blueAPI_CauseLowerLayerError : blueAPI_CauseSuccess,
                                       wCause,
                                       pGATTAttributeReadInd->attribIndex,
                                       wLength,
                                       wOffset
                                      )
       )
    {
        BQB_CmdPrint( pGATTDemo,
                      "--> GATTAttributeReadConf: service=0x%x, subCause=%s, length=%d\r\n%s",
                      pGATTAttributeReadInd->serviceHandle,
                      gattdSubCause2Str(wCause),
                      wLength,
                      pGATTDemo->CmdIF.cPrompt );
    }
    else
    {
        BQB_CmdPrint( pGATTDemo,
                      "!!! illegal parameter (e.g. offset too small)\r\n%s",
                      pGATTDemo->CmdIF.cPrompt );
        if ( pBuffer != NULL )
            blueAPI_BufferRelease(pBuffer);
    }

    return ( true );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_GATTAttributeWriteInd (triggered by ATT_WRITE_REQUEST)
 * and blueAPI_Event_GATTAttributeWriteCommandIngo (triggered by
 * ATT_WRITE_COMMAND). both have the same parameters.
 * --------------------------------------------------------------------------*/

static bool BQB_Handle_GATTAttributeWriteIndCommandInfo(
    PGATTDemo                      pGATTDemo,
    TBlueAPI_Command               Command,
    PBlueAPI_GATTAttributeWriteInd pGATTAttributeWriteInd )
{
    PGATTDService           pService;
    TBQBGATTDWriteIndPostProc  WriteIndPostProc = NULL;
    uint16_t                wCause  = GATT_ERR_ILLEGAL_PARAMETER;
    static const char writeIndTxt[]         = "GATTAttributeWriteInd";
    static const char writeCommandInfoTxt[] = "GATTAttributeWriteCommandInfo";
    char * pTxt;

    if ( Command == blueAPI_EventGATTAttributeWriteCommandInfo )
        pTxt = (char *)writeCommandInfoTxt;
    else
        pTxt = (char *)writeIndTxt;


    BQB_CmdPrint( pGATTDemo,
                  "<-- %s: service=0x%x, idx=%d, length=%d\r\n%s",
                  pTxt,
                  pGATTAttributeWriteInd->serviceHandle,
                  pGATTAttributeWriteInd->attribIndex,
                  pGATTAttributeWriteInd->attribLength,
                  pGATTDemo->CmdIF.cPrompt );

    pService = BQB_ServiceFind( pGATTDemo, pGATTAttributeWriteInd->serviceHandle );
    if ( pService != (PGATTDService)NULL )
    {
        /* service/attribute specific write operation.                         */
        /* check for authorization etc. has been done by GATT layer already .. */
        wCause  = BQB_AttribPut(
                      pGATTDemo, pService, pGATTAttributeWriteInd->attribIndex,
                      pGATTAttributeWriteInd->attribLength,
                      pGATTAttributeWriteInd->data + pGATTAttributeWriteInd->gap,
                      &WriteIndPostProc
                  );
    }

    /* send response */
    if ( Command == blueAPI_EventGATTAttributeWriteInd )
    {
        if ( blueAPI_GATTAttributeWriteConf(pGATTAttributeWriteInd->local_MDL_ID,
                                            pService->pServiceHandle,
                                            (wCause != GATT_SUCCESS) ?
                                            blueAPI_CauseLowerLayerError : blueAPI_CauseSuccess,
                                            wCause,
                                            pGATTAttributeWriteInd->attribIndex
                                           )
           )
        {
            BQB_CmdPrint( pGATTDemo,
                          "--> GATTAttributeWriteConf: service=0x%x, subCause=%s\r\n%s",
                          pGATTAttributeWriteInd->serviceHandle,
                          gattdSubCause2Str(wCause),
                          pGATTDemo->CmdIF.cPrompt );
        }
    }

    /* for services with RACP some post processing may be needed: */
    if ( WriteIndPostProc != 0 )
    {
        //temp 1127
         (*WriteIndPostProc)( pGATTDemo,
                             pGATTAttributeWriteInd->attribLength,
                             pGATTAttributeWriteInd->data + pGATTAttributeWriteInd->gap );
    }

    return ( true );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_GATTCCCDInfo
 * --------------------------------------------------------------------------*/

static bool BQB_Handle_GATTCCCDInfo( PGATTDemo             pGATTDemo,
                                     PBlueAPI_GATTCCCDInfo pGATTCCCDInfo )
{
    PGATTDService pService;

    BQB_CmdPrint( pGATTDemo,
                  "<-- GATTCCCDInfo: service=0x%x, count=%d\r\n%s",
                  pGATTCCCDInfo->serviceHandle,
                  pGATTCCCDInfo->count,
                  pGATTDemo->CmdIF.cPrompt );

    pService = BQB_ServiceFind( pGATTDemo, pGATTCCCDInfo->serviceHandle );
    if ( (pService != (PGATTDService)NULL) &&
            ((pService->ServiceID == BQB_SERVICE_APP_DEFINED))
       )
    {
        int        i;
        uint16_t   wAttribIndex, wCCCBits;
        uint16_t * pWord;

        pWord = (uint16_t *)&pGATTCCCDInfo->data[pGATTCCCDInfo->gap];
        for ( i = 0; i < pGATTCCCDInfo->count; i++ )
        {
            wAttribIndex = *(pWord++);
            wCCCBits     = *(pWord++);
            BQB_CmdPrint( pGATTDemo, "  attribIndex=%d, cccBits=0x%04x\r\n%s",
                          wAttribIndex, wCCCBits,
                          pGATTDemo->CmdIF.cPrompt );
#if (GATTDEMO_CCCD_COUNT)
            /* check if CCCD should be stored */
            {
                int  j;

                for ( j = 0; j < GATTDEMO_CCCD_COUNT; j++ )
                {
                    if ( pGATTDemo->AttrIdxCCCD[ j ].wAttribIndex == wAttribIndex )
                    {
                        pGATTDemo->AttrIdxCCCD[ j ].wCCCBits = wCCCBits;
                        break;
                    }
                }
            }
#endif
        }
    }

    return ( true );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_GATTDiscoveryRsp
 * --------------------------------------------------------------------------*/

static bool BQB_Handle_GATTDiscoveryRsp(PGATTDemo                 pGATTDemo,
                                        PBlueAPI_GATTDiscoveryRsp pGATTDiscoveryRsp)
{
    BQB_CmdPrint( pGATTDemo, "<-- GATTDiscoveryRsp: subCause=0x%x(%s)\r\n%s",
                  pGATTDiscoveryRsp->subCause,
                  gattdSubCause2Str(pGATTDiscoveryRsp->subCause),
                  pGATTDemo->CmdIF.cPrompt );

    return ( true );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_GATTDiscoveryInd
 * --------------------------------------------------------------------------*/

static bool BQB_Handle_GATTDiscoveryInd(PGATTDemo                 pGATTDemo,
                                        PBlueAPI_GATTDiscoveryInd pGATTDiscoveryInd)
{
    int        i, j;
    uint16_t   wEndingHandle = 0;   /* used for continuation search */
    PGATTLink pLink;
    BQB_CmdPrint( pGATTDemo,
                  "<-- GATTDiscoveryInd: subCause=0x%x(%s), type=%d, count=%d\r\n%s",
                  pGATTDiscoveryInd->subCause,
                  gattdSubCause2Str(pGATTDiscoveryInd->subCause),
                  pGATTDiscoveryInd->discoveryType,
                  pGATTDiscoveryInd->elementCount,
                  pGATTDemo->CmdIF.cPrompt );
    pLink = BQB_LinkFindByLocal_MDL_ID(pGATTDemo, pGATTDiscoveryInd->local_MDL_ID);
    if (pLink == NULL)
    {
        BQB_CmdPrint(pGATTDemo, "the Local_MDL_ID is not valid\r\n");
        return true;
    }
    if ( pGATTDiscoveryInd->subCause == GATT_SUCCESS )
    {
        uint32_t dwUUID128[4];

        switch ( pGATTDiscoveryInd->discoveryType )
        {
        case blueAPI_GATTDiscoveryServices:
            /*--- discovery of all primary services ---*/
            if ( pGATTDiscoveryInd->elementLength == sizeof(TBlueAPI_GATTGenericElement16) )
            {
                /* 2 handles + 16 bit UUID */
                PBlueAPI_GATTServicesElement16   pElement16;

                pElement16 = (PBlueAPI_GATTServicesElement16)(pGATTDiscoveryInd->list
                             + pGATTDiscoveryInd->gap);
                for ( i = 0; i < pGATTDiscoveryInd->elementCount; i++ )
                {
                    /* XXXX some output may be lost due to limited buffer pool ... */
                    BQB_CmdPrint( pGATTDemo,
                                  " range: 0x%04x-0x%04x, service=<0x%04x> (%s)\r\n%s",
                                  pElement16->attHandle,
                                  pElement16->endGroupHandle,
                                  pElement16->UUID16,
                                  BQB_UUID16ToString( pElement16->UUID16 ),
                                  pGATTDemo->CmdIF.cPrompt );
                    wEndingHandle = pElement16->endGroupHandle;
                    pElement16++;
                }
            }
            else
            {
                /* 2 handles + 128 bit UUID */
                PBlueAPI_GATTServicesElement128  pElement128;

                pElement128 = (PBlueAPI_GATTServicesElement128)(pGATTDiscoveryInd->list
                              + pGATTDiscoveryInd->gap);
                for ( i = 0; i < pGATTDiscoveryInd->elementCount; i++ )
                {
                    /* XXXX some output may be lost due to limited buffer pool ... */
                    memcpy( dwUUID128, &pElement128->UUID128[0], sizeof(dwUUID128));
                    BQB_CmdPrint( pGATTDemo,
                                  " range: 0x%04x-0x%04x, service=<0x%08x%08x%08x%08x>\r\n%s",
                                  pElement128->attHandle,
                                  pElement128->endGroupHandle,
                                  dwUUID128[3], dwUUID128[2], dwUUID128[1], dwUUID128[0],
                                  pGATTDemo->CmdIF.cPrompt );

                    wEndingHandle = pElement128->endGroupHandle;
                    pElement128++;
                }
            }
            break;

        case blueAPI_GATTDiscoveryServiceByUUID:
            /*--- discovery of primary services by UUID ---*/
            {
                /* 2 handles */
                PBlueAPI_GATTServiceByUUIDElement  pElementUUID;

                pElementUUID = (PBlueAPI_GATTServiceByUUIDElement)(pGATTDiscoveryInd->list
                               + pGATTDiscoveryInd->gap);
                for ( i = 0; i < pGATTDiscoveryInd->elementCount; i++ )
                {
                    /* XXXX some output may be lost due to limited buffer pool ... */
                    BQB_CmdPrint( pGATTDemo,
                                  " range: 0x%04x-0x%04x\r\n%s",
                                  pElementUUID->attHandle,
                                  pElementUUID->endGroupHandle,
                                  pGATTDemo->CmdIF.cPrompt );
                    wEndingHandle = pElementUUID->endGroupHandle;
                    pElementUUID++;
                }
            }
            break;

        case blueAPI_GATTDiscoveryRelationship:
            /*--- discovery of relationships        ---*/
            if ( pGATTDiscoveryInd->elementLength == sizeof(TBlueAPI_GATTRelationshipElement16) )
            {
                /* 3 handles + 16 bit UUID */
                PBlueAPI_GATTRelationshipElement16   pElement16;

                pElement16 = (PBlueAPI_GATTRelationshipElement16)(pGATTDiscoveryInd->list
                             + pGATTDiscoveryInd->gap);
                for ( i = 0; i < pGATTDiscoveryInd->elementCount; i++ )
                {
                    /* XXXX some output may be lost due to limited buffer pool ... */
                    BQB_CmdPrint( pGATTDemo,
                                  " decl=0x%04x, range: 0x%04x-0x%04x, service=<0x%04x> (%s)\r\n%s",
                                  pElement16->declHandle,
                                  pElement16->attHandle,
                                  pElement16->endGroupHandle,
                                  pElement16->UUID16,
                                  BQB_UUID16ToString( pElement16->UUID16 ),
                                  pGATTDemo->CmdIF.cPrompt );
                    wEndingHandle = pElement16->declHandle;
                    pElement16++;
                }
            }
            else
            {
                /* 3 handles + 128 bit UUID */
                PBlueAPI_GATTRelationshipElement128  pElement128;

                pElement128 = (PBlueAPI_GATTRelationshipElement128)(pGATTDiscoveryInd->list
                              + pGATTDiscoveryInd->gap);
                for ( i = 0; i < pGATTDiscoveryInd->elementCount; i++ )
                {
                    /* XXXX some output may be lost due to limited buffer pool ... */
                    memcpy( dwUUID128, &pElement128->UUID128[0], sizeof(dwUUID128));
                    BQB_CmdPrint( pGATTDemo,
                                  " decl=0x%04x, range: 0x%04x-0x%04x, service=<0x%08x%08x%08x%08x>\r\n%s",
                                  pElement128->declHandle,
                                  pElement128->attHandle,
                                  pElement128->endGroupHandle,
                                  dwUUID128[3], dwUUID128[2], dwUUID128[1], dwUUID128[0],
                                  pGATTDemo->CmdIF.cPrompt );

                    /* XXXXMJMJ: for type=GATT_TYPE_DISCOVERY_RELATION not yet fully
                     *  implemented: applic. can retrieve the 128 bit UUID thru
                     *  GATT_ATTRIBUTE_READ_REQ with the appropriate attribute handle
                     *  (just as on ATT layer ...) and an updated search with start handle
                     *  increased by 1 in a new GATT_DISCOVERY_REQ.
                     *  work-around check:
                     */
                    for ( j = 0; j < UUID_128BIT_SIZE; j++ )
                    {
                        if ( pElement128->UUID128[j] != 0 )
                            break;
                    }
                    if ( j == UUID_128BIT_SIZE )
                    {
                        /* UUID received is 0 */
                        BQB_CmdPrint( pGATTDemo,
                                      " !! get 128 bit UUID with command \"read x%04x\" !!\r\n%s",
                                      pElement128->attHandle,
                                      pGATTDemo->CmdIF.cPrompt );
                    }
                    /* end of work-around check */

                    wEndingHandle = pElement128->declHandle;
                    pElement128++;
                }
            }
            break;

        case blueAPI_GATTDiscoveryCharacteristics:
            /*--- discovery of all characteristics of a service ---*/
            if ( pGATTDiscoveryInd->elementLength == sizeof(TBlueAPI_GATTCharacteristicsElement16) )
            {
                /* declaration handle, properties, value handle + 16 bit UUID */
                PBlueAPI_GATTCharacteristicsElement16   pElement16;

                pElement16 = (PBlueAPI_GATTCharacteristicsElement16)(pGATTDiscoveryInd->list
                             + pGATTDiscoveryInd->gap);
                for ( i = 0; i < pGATTDiscoveryInd->elementCount; i++ )
                {
                    /* XXXX some output may be lost due to limited buffer pool ... */
                    BQB_CmdPrint( pGATTDemo,
                                  " decl hndl=0x%04x, prop=0x%02x, value hndl=0x%04x, UUID=<0x%04x> (%s)\r\n%s",
                                  pElement16->declHandle,
                                  pElement16->properties,
                                  pElement16->valueHandle,
                                  pElement16->UUID16,
                                  BQB_UUID16ToString( pElement16->UUID16 ),
                                  pGATTDemo->CmdIF.cPrompt );
                    wEndingHandle = pElement16->declHandle;

                    /* save characteristic value handle/UUID in local array */
                    BQB_HandleUUIDSave( pGATTDemo, pLink->idx, pElement16->valueHandle, pElement16->UUID16 );
                    pElement16++;
                }
            }
            else
            {
                /* declaration handle, properties, value handle + 128 bit UUID */
                PBlueAPI_GATTCharacteristicsElement128  pElement128;

                pElement128 = (PBlueAPI_GATTCharacteristicsElement128)(pGATTDiscoveryInd->list
                              + pGATTDiscoveryInd->gap);
                for ( i = 0; i < pGATTDiscoveryInd->elementCount; i++ )
                {
                    /* XXXX some output may be lost due to limited buffer pool ... */
                    memcpy( dwUUID128, &pElement128->UUID128[0], sizeof(dwUUID128));
                    BQB_CmdPrint( pGATTDemo,
                                  " decl hndl=0x%04x, prop=0x%02x, value hndl=0x%04x, UUID=<0x%08x%08x%08x%08x>\r\n%s",
                                  pElement128->declHandle,
                                  pElement128->properties,
                                  pElement128->valueHandle,
                                  dwUUID128[3], dwUUID128[2], dwUUID128[1], dwUUID128[0],
                                  pGATTDemo->CmdIF.cPrompt );

                    wEndingHandle = pElement128->declHandle;
                    pElement128++;
                }
            }
            break;

        case blueAPI_GATTDiscoveryCharacDescriptors:
            /*--- discovery of characteristic descriptors of a service ---*/
            /*--- (or list of UUIDs ..) ---*/
            if ( pGATTDiscoveryInd->elementLength == sizeof(TBlueAPI_GATTCharacDescriptorsElement16) )
            {
                /* handle, properties, 16 bit UUID */
                PBlueAPI_GATTCharacDescriptorsElement16   pElement16;

                pElement16 = (PBlueAPI_GATTCharacDescriptorsElement16)(pGATTDiscoveryInd->list
                             + pGATTDiscoveryInd->gap);
                for ( i = 0; i < pGATTDiscoveryInd->elementCount; i++ )
                {
                    /* XXXX some output may be lost due to limited buffer pool ... */
                    BQB_CmdPrint( pGATTDemo,
                                  " handle=0x%04x, UUID=<0x%04x> (%s)\r\n%s",
                                  pElement16->handle,
                                  pElement16->UUID16,
                                  BQB_UUID16ToString( pElement16->UUID16 ),
                                  pGATTDemo->CmdIF.cPrompt );
                    wEndingHandle = pElement16->handle;

                    /* save characteristic descriptor handle/UUID in local array */
                    BQB_HandleUUIDSave( pGATTDemo, pLink->idx, pElement16->handle, pElement16->UUID16 );

                    pElement16++;
                }
            }
            else
            {
                /* handle, 128 bit UUID */
                PBlueAPI_GATTCharacDescriptorsElement128  pElement128;

                pElement128 = (PBlueAPI_GATTCharacDescriptorsElement128)(pGATTDiscoveryInd->list
                              + pGATTDiscoveryInd->gap);
                for ( i = 0; i < pGATTDiscoveryInd->elementCount; i++ )
                {
                    /* XXXX some output may be lost due to limited buffer pool ... */
                    memcpy( dwUUID128, &pElement128->UUID128[0], sizeof(dwUUID128));
                    BQB_CmdPrint( pGATTDemo,
                                  " handle=0x%04x, UUID=<0x%08x%08x%08x%08x>\r\n%s",
                                  pElement128->handle,
                                  dwUUID128[3], dwUUID128[2], dwUUID128[1], dwUUID128[0],
                                  pGATTDemo->CmdIF.cPrompt );

                    wEndingHandle = pElement128->handle;
                    pElement128++;
                }
            }
            break;
        }

        if ( wEndingHandle >= pLink->wEndingHandle )
            pLink->wEndingHandle = 0;   /* discovery complete */
    }
    else
    {
        /* no more results found */
        pLink->wEndingHandle = 0;     /* discovery complete */
    }

    if ( blueAPI_GATTDiscoveryConf(pGATTDiscoveryInd->local_MDL_ID,
                                   (TBlueAPI_GATTDiscoveryType)pGATTDiscoveryInd->discoveryType,
                                   0,  /* starting handle: continue search */
                                   pLink->wEndingHandle
                                  )
       )
    {
        BQB_CmdPrint( pGATTDemo,
                      "idx= %d--> GATTDiscoveryConf: start=0x%04x, end=0x%04x\r\n%s",
                      pLink->idx, 0,
                      pLink->wEndingHandle,
                      pGATTDemo->CmdIF.cPrompt );
    }

    return ( true );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_GATTAttributeReadRsp
 * --------------------------------------------------------------------------*/

static bool BQB_Handle_GATTAttributeReadRsp(PGATTDemo  pGATTDemo,
        PBlueAPI_GATTAttributeReadRsp pGATTAttributeReadRsp)
{
    PGATTLink pLink;
    BQB_CmdPrint( pGATTDemo, "<-- GATTAttributeReadRsp: subCause=0x%x(%s), length=%d\r\n%s",
                  pGATTAttributeReadRsp->subCause,
                  gattdSubCause2Str(pGATTAttributeReadRsp->subCause),
                  pGATTAttributeReadRsp->totalLength,
                  pGATTDemo->CmdIF.cPrompt );
    pLink = BQB_LinkFindByLocal_MDL_ID(pGATTDemo, pGATTAttributeReadRsp->local_MDL_ID);
    if (pLink == NULL)
    {
        BQB_CmdPrint( pGATTDemo, "Error: local_MDL_ID is not valid\r\n");
        return (true);
    }

    if ( pGATTAttributeReadRsp->subCause == GATT_SUCCESS )
    {
        if ( pGATTAttributeReadRsp->totalLength > 0 )
        {
            int    i;
            int    iValueSize;
            int    iTruncated;   /* nbr. of bytes that have been truncated in last */
            /* handle/value pair (ReadByTypeResp only ..)     */
            uint16_t * pHandle;
            uint8_t  * pValue;

            /* only OK if attribute values have same size: !!!!! */
            iValueSize = pGATTAttributeReadRsp->attribLength;
            iTruncated = pGATTAttributeReadRsp->nbrOfHandles * (sizeof(uint16_t) + iValueSize) -
                         pGATTAttributeReadRsp->totalLength;
            pHandle    = (uint16_t *)(pGATTAttributeReadRsp->handlesData +   /* first handle */
                                      pGATTAttributeReadRsp->gap);
            pValue     = ((uint8_t *)pHandle) +                    /* first attrib. value */
                         (pGATTAttributeReadRsp->nbrOfHandles * sizeof(uint16_t));

            for ( i = 0; i < pGATTAttributeReadRsp->nbrOfHandles; i++ )
            {
                if ( (iTruncated != 0) && (i == (pGATTAttributeReadRsp->nbrOfHandles - 1)) )
                    iValueSize -= iTruncated;

                /* service/attribute specific display of value: */
                BQB_AttribDisplay( pGATTDemo, pLink->idx, *pHandle, iValueSize, pValue );
                pHandle++;
                pValue += iValueSize;
            }
        }
    }

    return ( true );
}
#if 0
static bool BQB_Handle_GATTAttributeReadMultipleRsp(PGATTDemo  pGATTDemo,
                                  PBlueAPI_GATTAttributeReadMultipleRsp pGATTAttributeReadMultiRsp)
{
    PGATTLink pLink;
    BQB_CmdPrint( pGATTDemo, "<-- GATTAttributeReadMultipleRsp: subCause=0x%x(%s), length=%d\r\n%s",
                            pGATTAttributeReadMultiRsp->subCause,
                            gattdSubCause2Str(pGATTAttributeReadMultiRsp->subCause),
                            pGATTAttributeReadMultiRsp->totalLength,
                            pGATTDemo->CmdIF.cPrompt );
    pLink = BQB_LinkFindByLocal_MDL_ID(pGATTDemo, pGATTAttributeReadMultiRsp->local_MDL_ID);
    
    if(pLink == NULL)
    {
        BQB_CmdPrint( pGATTDemo, "Error: local_MDL_ID is not valid\r\n");
        return (true);
    }
  
    if ( pGATTAttributeReadMultiRsp->subCause == GATT_SUCCESS )
    {
        if ( pGATTAttributeReadMultiRsp->totalLength > 0 )
        {
            int length = pGATTAttributeReadMultiRsp->totalLength;
            uint8_t  * pValue = pGATTAttributeReadMultiRsp->SetOfValues + pGATTAttributeReadMultiRsp->gap;
      
            BQB_HexDump( pGATTDemo, "value=", length, pValue );
        }
    }

    return ( true );
}
#endif

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_GATTAttributeWriteRsp
 * --------------------------------------------------------------------------*/

static bool BQB_Handle_GATTAttributeWriteRsp(PGATTDemo  pGATTDemo,
        PBlueAPI_GATTAttributeWriteRsp pGATTAttributeWriteRsp)
{
    BQB_CmdPrint( pGATTDemo, "<-- GATTAttributeWriteRsp: subCause=0x%x(%s)\r\n%s",
                  pGATTAttributeWriteRsp->subCause,
                  gattdSubCause2Str(pGATTAttributeWriteRsp->subCause),
                  pGATTDemo->CmdIF.cPrompt );

    return ( true );
}

 #if (BQB_PREPARE_WRITE_SUPPPRT)
static bool BQB_Handle_GATTAttributePrepareWriteInd(
                        PGATTDemo                    pGATTDemo,
                        TBlueAPI_Command                 Command,
                        PBlueAPI_GATTAttributeWriteInd pGATTAttributeWriteInd )
{
    uint16_t       wCause  = GATT_SUCCESS;
    uint8_t *      pBuffer;
    uint16_t       wLength = 0;
    uint16_t       wOffset = pGATTDemo->wDsDataOffset;
    uint8_t *      pData   = NULL;
    PGATTPrepareWrite pPrepare = NULL;

    BQB_CmdPrint( pGATTDemo,
                     "<--GATTAttributePrepareWriteInd : service=0x%x, idx=%d, length=%d valueOffset=%d\r\n%s",
                     pGATTAttributeWriteInd->serviceHandle,
                     pGATTAttributeWriteInd->attribIndex,
                     pGATTAttributeWriteInd->attribLength,
                     pGATTAttributeWriteInd->writeOffset,
                     pGATTDemo->CmdIF.cPrompt );

    wLength = pGATTAttributeWriteInd->attribLength;
    pData = pGATTAttributeWriteInd->data + pGATTAttributeWriteInd->gap;

    //the first prepare write
    if(0 == pGATTDemo->queueIdx)
    {
        pGATTDemo->queue_local_MDL_ID = pGATTAttributeWriteInd->local_MDL_ID;
    }
  
    if(pGATTDemo->queue_local_MDL_ID == pGATTAttributeWriteInd->local_MDL_ID)
    {
        if(pGATTDemo->queueIdx == BQB_MAX_PREPARE_QUEUE)
        {
            wCause = (ATT_ERR|ATT_ERR_PREPARE_QUEUE_FULL);
        }
        else
        {
            pPrepare = &pGATTDemo->prepareQueue[pGATTDemo->queueIdx];
            pPrepare->attribIndex = pGATTAttributeWriteInd->attribIndex;
            pPrepare->serviceHandle = pGATTAttributeWriteInd->serviceHandle;
            pPrepare->handle = pGATTAttributeWriteInd->handle;
            pPrepare->attribLength = pGATTAttributeWriteInd->attribLength;
            pPrepare->writeOffset = pGATTAttributeWriteInd->writeOffset;
            memcpy( pPrepare->data , pData, wLength );
            pGATTDemo->queueIdx++;      
        }
    }
    else
    {
        wCause = (ATT_ERR|ATT_ERR_MIN_APPLIC_CODE);
    }
  
    BQB_HexDump( pGATTDemo, "value=", wLength, pData);

    if ( wCause == GATT_SUCCESS )
    {
        if ( blueAPI_BufferGet(
                              pGATTDemo->wDsPoolId,
                              wLength,
                              wOffset,
                              (void **)&pBuffer) == blueAPI_CauseSuccess )
        {
            memcpy( pBuffer+wOffset, pData, wLength );
        }
        else
        {
            wCause = GATT_ERR_OUT_OF_RESOURCE;
        }
    }
    if ( wCause != GATT_SUCCESS )
    {
        pBuffer = NULL;
        wLength = 0;
    }   
     
    if ( blueAPI_GATTAttributePrepareWriteConf(pBuffer,
                                       pGATTAttributeWriteInd->local_MDL_ID,
                                       pGATTAttributeWriteInd->serviceHandle,
                                       (wCause != GATT_SUCCESS) ?
                                           blueAPI_CauseLowerLayerError : blueAPI_CauseSuccess,
                                       wCause,
                                       pGATTAttributeWriteInd->attribIndex,
                                       wLength,
                                       wOffset
                                      )
           )
    {
        BQB_CmdPrint( pGATTDemo,
                           "--> GATTAttributeWriteConf: service=0x%x, subCause=%s\r\n%s",
                           pGATTAttributeWriteInd->serviceHandle,
                           gattdSubCause2Str(wCause),
                           pGATTDemo->CmdIF.cPrompt );
    }
    else
    {
        BQB_CmdPrint( pGATTDemo,
                     "!!! illegal parameter (e.g. offset too small)\r\n%s",
                     pGATTDemo->CmdIF.cPrompt );
        if (NULL != pBuffer)
        {
            blueAPI_BufferRelease(pBuffer);
        }
    }
        
    return( true );
}

static void BQB_Handle_GATTAttributePrepareWriteRsp(PGATTDemo pGATTDemo,
                                          PBlueAPI_GATTAttributePrepareWriteRsp pPrepareWriteRsp)
{
    int length = pPrepareWriteRsp->attribLength;    /* the length of the prepare write */
    int writeOffset = pPrepareWriteRsp->writeOffset;    /* offset of the prepare write */
    uint8_t *value =  pPrepareWriteRsp->data + pPrepareWriteRsp->gap;
    BQB_CmdPrint( pGATTDemo, "<-- PrepareWriteRsp: cause=%s subcase = %s, writeOffset=%d attribLength=%d\r\n%s",
                            blueAPI_CauseString((TBlueAPI_Cause)pPrepareWriteRsp->cause),
                            gattdSubCause2Str(pPrepareWriteRsp->subCause),
                            pPrepareWriteRsp->writeOffset,
                            pPrepareWriteRsp->attribLength,
                            pGATTDemo->CmdIF.cPrompt );
    BQB_HexDump( pGATTDemo, "value=", length, value );

    /* error occurs */
    if(gattdResultOk != pPrepareWriteRsp->cause)
    {
        return;
    }

    writeOffset += length;
    
    if(writeOffset < pGATTDemo->pwrite_attri_len)
    {
        if(writeOffset + 18 > pGATTDemo->pwrite_attri_len)
        {
            length = pGATTDemo->pwrite_attri_len - writeOffset;
        }
        else
        {
            length = 18;    //mtu = 23
        }
        BQB_SendPrepareWrite(pGATTDemo, pPrepareWriteRsp->local_MDL_ID, 
                                    pGATTDemo->pwrite_handle, writeOffset, length); 
    }
    else
    {
        //BQB_SendExecuteWrite(pGATTDemo,pPrepareWriteRsp->local_MDL_ID, 0x01); 
        BQB_CmdPrint( pGATTDemo, "Please send execute write command......\r\n");
    }
}

static void BQB_Handle_GATTAttributeExecuteWriteRsp(PGATTDemo pGATTDemo,
                                          PBlueAPI_GATTAttributeExecuteWriteRsp pExecuteWriteRsp)
{
    BQB_CmdPrint( pGATTDemo, "<-- ExecuteWriteRsp: cause=%s, subcause=%s\r\n%s",
                            blueAPI_CauseString(pExecuteWriteRsp->cause),
                            gattdSubCause2Str(pExecuteWriteRsp->subCause),
                            pGATTDemo->CmdIF.cPrompt );
    pGATTDemo->pwrite_attri_len = 0;
    pGATTDemo->pwrite_handle = 0;
}

//only used for large databse 1
TAttribAppl* BQB_GetAttribute(PGATTDemo pGATTDemo, void* p_service_handle, uint16_t attr_index)
{
    uint8_t i = 0;
    uint8_t service_id = 0;
    uint16_t service_off = 0;

    for(service_id = 0; service_id < pGATTDemo->iServiceCount; service_id++)
    {
        if(pGATTDemo->Service[service_id].pServiceHandle == p_service_handle)
        {
            break;
        }
    }

    /* find valid sercice id */
    if(0 != service_id)
    {
        for(i = 0; i < service_id; i++ )
        {
            service_off+= Service_Info[i].num_attr;
        }
    }
    else
    {
        return NULL;
    }

    return (TAttribAppl*)(&LargeDatabase1[0] + service_off + attr_index);
}


/* gatt serve: write prepare write data in queue to reald attribute */
uint16_t  BQB_WritePrepareQueueData( PGATTDemo pGATTDemo, uint16_t local_MDL_ID,uint16_t *handle)
{
    uint16_t  wCause  = GATT_SUCCESS;
    PGATTPrepareWrite pPrepareWrite;
    TAttribAppl* p_attrib;
    //int length;
    uint8_t *pData;

    //for reliable write, use different way to judge invalid length
    uint8_t attr_type_num = 0;  //reliable write handles num

    uint8_t i = 0;
    uint8_t j = 0;
    uint8_t k = 0;
    uint8_t attr_map[6] = {0};  //attribute map
    PGATTPrepareWrite p1 = NULL, p2 = NULL;
    BOOL is_same_handle = FALSE;
    uint16_t attr_len =0;
    uint16_t write_len = 0;
    uint16_t write_off = 0;

    //the first handle
    p1 = &pGATTDemo->prepareQueue[0];
    attr_map[0] |= (1 << 0);
    attr_type_num++;
    
    //max 6 queue
    for(i = 1; i < pGATTDemo->queueIdx; i++)
    {
        p1 = &pGATTDemo->prepareQueue[i];

        //find if there is attribute that is same with p1
        for(j = 0; j < i; j++)
        {
            p2 = &pGATTDemo->prepareQueue[j];

            //the same atttribute
            if(p1->serviceHandle == p2->serviceHandle
                && p1->attribIndex == p2->attribIndex)
            {
                is_same_handle = TRUE;

                //find map
                for(k = 0; k < attr_type_num; k++)
                {
                    //add this hanlde to map already exsit
                    if((1 << j) & attr_map[k])
                    {
                        attr_map[k] |= (1 << i);
                    }
                    
                    if(TRUE == is_same_handle)
                    {
                        break;
                    }
                }
            }

            if(TRUE == is_same_handle)
            {
                break;
            }
        }
        
        //new attribute 
        if(FALSE == is_same_handle)
        {
            attr_map[++attr_type_num] |= (1 << i);
        }
        else
        {
            is_same_handle = FALSE;
        }
        
    }

    
    for(i = 0; i < attr_type_num; i++)
    {
        //find attribute in database
        for(j = 0; j < 6; j++)
        {
            if((1 << j) & attr_map[i])
            {
                p1 = &pGATTDemo->prepareQueue[j];
                //find attribute in database
                p_attrib = BQB_GetAttribute(pGATTDemo, p1->serviceHandle, p1->attribIndex);
                attr_len = p_attrib->wValueLen;
                break;
            }
        }

        //check the length and offset of the attribute
        for(j = 0; j < 6; j++)
        {
            if((1 << j) & attr_map[i])
            {
                p1 = &pGATTDemo->prepareQueue[j];
                write_len+= p1->attribLength;
                if(p1->writeOffset > write_off)
                {
                    write_off = p1->writeOffset;
                }
            }
        }

        if(attr_len < write_off)
        {
            wCause = ATT_ERR|ATT_ERR_INVALID_OFFSET;
            *handle = p1->handle;
            break;
        }
        
        //here, we find the length and offset of an attriute, check error
        if(attr_len < write_len)
        {
            wCause = ATT_ERR|ATT_ERR_INVALID_VALUE_SIZE;
            *handle = p1->handle;
            break;
        }

    }

#if 0
    //for reliable write
    uint16_t write_len =0;

    for(i = 0; i < pGATTDemo->queueIdx; i++)
    {
        pPrepareWrite = &pGATTDemo->prepareQueue[i];
        p_attrib = BQB_GetAttribute(pGATTDemo, pPrepareWrite->serviceHandle, pPrepareWrite->attribIndex);
        length = p_attrib->wValueLen;   /* real?attribute length */

        /* prepare write write handle */
        if(length !=0)
        {
            write_len += pPrepareWrite->attribLength;

            if(pPrepareWrite->attribLength > length || write_len > length)
            {
                wCause = ATT_ERR|ATT_ERR_INVALID_VALUE_SIZE;
                *handle = pPrepareWrite->handle;
                break;
            }
            
            if(pPrepareWrite->writeOffset > length)
            {
                wCause = ATT_ERR|ATT_ERR_INVALID_OFFSET;
                *handle = pPrepareWrite->handle;
                break;
            }
        }
        else
        {
            wCause = ATT_ERR|ATT_ERR_OUT_OF_RANGE;
            *handle = pPrepareWrite->handle;
            break;
        }
        
        if(NULL == p_attrib->pValueContext)
        {
            wCause = ATT_ERR|ATT_ERR_OUT_OF_RANGE;
            *handle = pPrepareWrite->handle;
            break;
        }   
    }
#endif
 
    if(wCause == GATT_SUCCESS)
    {       
        for(i = 0;i < pGATTDemo->queueIdx; i++)
        {
            pPrepareWrite = &pGATTDemo->prepareQueue[i];
            p_attrib = BQB_GetAttribute(pGATTDemo, pPrepareWrite->serviceHandle, pPrepareWrite->attribIndex);
            
            if(p_attrib)
            {
                pData = p_attrib->pValueContext;
                //length = p_attrib->wValueLen;
                memcpy((pData + pPrepareWrite->writeOffset), pPrepareWrite->data, pPrepareWrite->attribLength); 
            }
            else
            {
                wCause =  GATT_ERR_ILLEGAL_HANDLE;
            }
        }           
    }

    return( wCause );
}

static void BQB_Handle_GATTAttributeExecuteWriteInd(PGATTDemo pGATTDemo,
                                          PBlueAPI_GATTAttributeExecuteWriteInd pExecuteWriteInd)
{
    uint16_t  wCause = blueAPI_CauseSuccess;
    uint8_t   flags = pExecuteWriteInd->flags;
    uint16_t  handle = 0;
    BQB_CmdPrint( pGATTDemo, "<-- ExecuteWriteInd: MDL=%d flags=%d\r\n%s",
                            pExecuteWriteInd->local_MDL_ID,
                            pExecuteWriteInd->flags,
                            pGATTDemo->CmdIF.cPrompt );
    
    if(pGATTDemo->queue_local_MDL_ID == pExecuteWriteInd->local_MDL_ID)
    {
        if(flags == 0)
        {
            pGATTDemo->queueIdx = 0;
            pGATTDemo->queue_local_MDL_ID = 0;
        }
        else
        {
            /* write prepare write value i queue to real attribute */
            wCause = BQB_WritePrepareQueueData(pGATTDemo, pExecuteWriteInd->local_MDL_ID, &handle);
            
            if(GATT_SUCCESS != wCause)
            {
                BQB_CmdPrint(pGATTDemo, "Write prepare write data in queue to attribute failed, cause=%s\r\n", gattdSubCause2Str(wCause));
            }
            
            pGATTDemo->queueIdx = 0;
            pGATTDemo->queue_local_MDL_ID = 0;
        }
    }
    else
    {
        wCause = (ATT_ERR|ATT_ERR_MIN_APPLIC_CODE);
    }
  
    blueAPI_GATTAttributeExecuteWriteConf(pExecuteWriteInd->local_MDL_ID,
                                    (wCause != GATT_SUCCESS) ?
                                    blueAPI_CauseLowerLayerError : blueAPI_CauseSuccess,
                                    wCause,
                                    handle
                                   );
}

#endif

/*----------------------------------------------------------------------------
 * handle blueAPI_EventGATTAttributeInd and
 * blueAPI_EventGATTAttributeNotificationInfo
 * --------------------------------------------------------------------------*/

static bool BQB_Handle_GATTAttributeNotifInd(PGATTDemo  pGATTDemo,
        TBlueAPI_Command Command,
        PBlueAPI_GATTAttributeInd pGATTAttributeInd)
{
    bool  Notify = (Command == blueAPI_EventGATTAttributeNotificationInfo);
    PGATTLink pLink;
    BQB_CmdPrint( pGATTDemo, "<-- GATTAttributeInd: notif=%d, handle=0x%x\r\n%s",
                  Notify,
                  pGATTAttributeInd->attribHandle,
                  pGATTDemo->CmdIF.cPrompt );
    pLink = BQB_LinkFindByLocal_MDL_ID(pGATTDemo, pGATTAttributeInd->local_MDL_ID);
    if (pLink == NULL)
    {
        BQB_CmdPrint(pGATTDemo, "error:local_MDL_ID is not valid");
        return (true);
    }
    /* service/attribute specific display of value: */    /* tifnan: for add for bqb */
    /*BQB_AttribDisplay(pGATTDemo, pLink->idx, pGATTAttributeInd->attribHandle,
                                   pGATTAttributeInd->attribLength,
                                   pGATTAttributeInd->data+pGATTAttributeInd->gap
                      );*/
    if ( !Notify )
    {
        /* send response/ack */
        if ( blueAPI_GATTAttributeConf(pGATTAttributeInd->local_MDL_ID
                                      )
           )
        {
            BQB_CmdPrint( pGATTDemo, "--> GATTAttributeConf\r\n%s",
                          pGATTDemo->CmdIF.cPrompt );
        }
    }

    return ( true );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_EventGATTSecurityRsp
 * --------------------------------------------------------------------------*/
static void BQB_Handle_GATTSecurityRsp(PGATTDemo pGATTDemo,
                                       PBlueAPI_GATTSecurityRsp pSecurityRsp)
{
    BQB_CmdPrint( pGATTDemo, "<-- GATTSecurityRsp: keyType=%d keySize=%d cause=%s\r\n%s",
                  pSecurityRsp->keyType,
                  pSecurityRsp->keySize,
                  blueAPI_CauseString(pSecurityRsp->cause),
                  pGATTDemo->CmdIF.cPrompt );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_EventGATTServerStoreInd
 * --------------------------------------------------------------------------*/
static void BQB_Handle_GATTServerStoreInd(PGATTDemo pGATTDemo,
        PBlueAPI_GATTServerStoreInd pStoreInd)
{
    uint16_t        restart   = 0x0000;
    uint8_t         dataLen   = 0;
    uint8_t *       pData     = NULL;
    TBlueAPI_Cause  cause     = blueAPI_CauseReject;

    BQB_CmdPrint( pGATTDemo, "<-- GATTServerStoreInd: opCode=%d bd=%s bdType=%d restart=0x%x dataLen=%d\r\n%s",
                  pStoreInd->opCode,
                  blueAPI_StringizeBd(pStoreInd->remote_BD),
                  pStoreInd->remote_BD_Type,
                  pStoreInd->restartHandle,
                  pStoreInd->dataLength,
                  pGATTDemo->CmdIF.cPrompt );


    blueAPI_GATTServerStoreConf((TBlueAPI_GATTStoreOpCode)pStoreInd->opCode,
                                pStoreInd->remote_BD,
                                (TBlueAPI_RemoteBDType)pStoreInd->remote_BD_Type,
                                restart,
                                dataLen,
                                pData,
                                cause
                               );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_EventLEAdvertiseRsp
 * --------------------------------------------------------------------------*/
static void BQB_Handle_LEAdvertiseRsp(PGATTDemo pGATTDemo,
                                      PBlueAPI_LEAdvertiseRsp pAdvertiseRsp)
{
    BQB_CmdPrint( pGATTDemo, "<-- LEAdvertiseRsp: advMode=%d cause=%s\r\n%s",
                  pAdvertiseRsp->advMode,
                  blueAPI_CauseString(pAdvertiseRsp->cause),
                  pGATTDemo->CmdIF.cPrompt );

    if (pGATTDemo->initLEServer)
    {
        if (pAdvertiseRsp->advMode == blueAPI_LEAdvModeEnabled)
        {
            pGATTDemo->initLEServer = false;
        }
    }

}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_LEAdvertiseParameterRsp
 * --------------------------------------------------------------------------*/
static void BQB_Handle_LEAdvertiseParameterRsp(PGATTDemo pGATTDemo,
        PBlueAPI_LEAdvertiseParameterSetRsp pAdvertiseParameterSetRsp)
{
    BQB_CmdPrint( pGATTDemo, "<-- LEAdvertiseParameterRsp: cause=%s\r\n%s",
                  blueAPI_CauseString(pAdvertiseParameterSetRsp->cause),
                  pGATTDemo->CmdIF.cPrompt );

    if (pGATTDemo->initLEServer)
    {
        if (ADV_DATA_IND_0 == AdvDataIndex)
        {
            BQB_SendLEAdvertiseDataSetReq(pGATTDemo,
                                          blueAPI_LEDataTypeAdvertisingData,
                                          sizeof(bBQBAdData0),
                                          (uint8_t *)bBQBAdData0
                                         );
        }
        else if (ADV_DATA_IND_1 == AdvDataIndex)
        {
            BQB_SendLEAdvertiseDataSetReq(pGATTDemo,
                                          blueAPI_LEDataTypeAdvertisingData,
                                          sizeof(bBQBAdData1),
                                          (uint8_t *)bBQBAdData1
                                         );
        }
        else if (ADV_DATA_IND_2 == AdvDataIndex)
        {
            BQB_SendLEAdvertiseDataSetReq(pGATTDemo,
                                          blueAPI_LEDataTypeAdvertisingData,
                                          sizeof(bBQBAdData1),
                                          (uint8_t *)bBQBAdData1
                                         );

        }
        else
        {
            BQB_CmdPrint(pGATTDemo, " invalid adv data index \r\n");
        }
    }
}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_LEAdvertiseDataSetRsp
 * --------------------------------------------------------------------------*/
static void BQB_Handle_LEAdvertiseDataSetRsp(PGATTDemo pGATTDemo,
        PBlueAPI_LEAdvertiseDataSetRsp pAdvertiseDataSetRsp)
{
    BQB_CmdPrint( pGATTDemo, "<-- LEAdvertiseDataSetRsp: dataType=%d cause=%s\r\n%s",
                  pAdvertiseDataSetRsp->dataType,
                  blueAPI_CauseString(pAdvertiseDataSetRsp->cause),
                  pGATTDemo->CmdIF.cPrompt );

    if (pGATTDemo->initLEServer)
    {
        if (pAdvertiseDataSetRsp->dataType == blueAPI_LEDataTypeAdvertisingData)
        {
            BQB_SendLEAdvertiseDataSetReq(pGATTDemo,
                                          blueAPI_LEDataTypeScanResponseData,
                                          sizeof(bBQBScanRespData),
                                          (uint8_t *)bBQBScanRespData
                                         );
        }
        else
        {
            BQB_SendLEAdvertiseReq(pGATTDemo,
                                   blueAPI_LEAdvModeEnabled,
                                   NULL,
                                   blueAPI_RemoteBDTypeLEPublic
                                  );
        }
    }

}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_LEScanRsp
 * --------------------------------------------------------------------------*/
static void BQB_Handle_LEScanRsp(PGATTDemo pGATTDemo,
                                 PBlueAPI_LEScanRsp pScanRsp)
{
    BQB_CmdPrint( pGATTDemo, "<-- LEScanRsp: cause=%s\r\n%s",
                  blueAPI_CauseString(pScanRsp->cause),
                  pGATTDemo->CmdIF.cPrompt );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_LEScanInfo
 * --------------------------------------------------------------------------*/
static void BQB_Handle_LEScanInfo(PGATTDemo pGATTDemo,
                                  PBlueAPI_LEScanInfo pScanInfo)
{
    uint8_t       buffer[32];
    uint8_t       pos        = 0;

    BQB_CmdPrint( pGATTDemo, "<-- LEScanInfo: bd=%s bdtype=%d event=0x%x rssi=0x%x\r\n",
                  blueAPI_StringizeBd(pScanInfo->remote_BD),
                  pScanInfo->remote_BD_type,
                  pScanInfo->advType,
                  pScanInfo->rssi);

    while (pos < pScanInfo->dataLength)
    {
        uint8_t length = pScanInfo->data[pos++];
        //uint8_t type;

        if (length < 0x02 || length > 0x1F || (pos + length) > 0x1F)
            continue;

        memcpy(buffer, pScanInfo->data + pos + 1, length - 1);
        //type = pScanInfo->data[pos];
        pos += length;
    }


}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_LEModifyWhitelistRsp
 * --------------------------------------------------------------------------*/
static void BQB_Handle_LEModifyWhitelistRsp(PGATTDemo pGATTDemo,
        PBlueAPI_LEModifyWhitelistRsp pModifyWhitelistRsp)
{
    BQB_CmdPrint( pGATTDemo, "<-- LEModifyWhitelistRsp: op=%d cause=%s\r\n%s",
                  pModifyWhitelistRsp->operation,
                  blueAPI_CauseString(pModifyWhitelistRsp->cause),
                  pGATTDemo->CmdIF.cPrompt );

    if (pGATTDemo->initLEServer)
    {
        if (pModifyWhitelistRsp->operation == blueAPI_LEWhitelistOpClear)
        {
            BQB_SendLEModifyWhitelistReq(pGATTDemo,
                                         blueAPI_LEWhitelistOpAdd,
                                         pGATTDemo->linkTable[0].RemoteBd, //fixme later
                                         blueAPI_RemoteBDTypeLEPublic
                                        );
        }
        else
        {
            if ( blueAPI_LEAdvertiseParameterSetReq(blueAPI_LEAdvTypeUndirected,
                                                    blueAPI_LEFilterAny,
                                                    blueAPI_LEFilterAny,
                                                    0x20, /* 20ms */
                                                    0x30, /* 30ms */
                                                    blueAPI_LocalBDTypeLEPublic,
                                                    NULL,
                                                    blueAPI_RemoteBDTypeLEPublic
                                                   )
               )
            {
                BQB_CmdPrint( pGATTDemo, "--> LEAdvertiseParameterSetReq\r\n%s",
                              pGATTDemo->CmdIF.cPrompt );
            }
        }
    }

}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_LEConnectionUpdateRsp
 * --------------------------------------------------------------------------*/
static void BQB_Handle_LEConnectionUpdateRsp(PGATTDemo pGATTDemo,
        PBlueAPI_LEConnectionUpdateRsp pConnectionUpdateRsp)
{
    BQB_CmdPrint( pGATTDemo, "<-- LEConnectionUpdateRsp: MDL=%d cause=%s\r\n%s",
                  pConnectionUpdateRsp->local_MDL_ID,
                  blueAPI_CauseString(pConnectionUpdateRsp->cause),
                  pGATTDemo->CmdIF.cPrompt );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_LEConnectionUpdateInd
 * --------------------------------------------------------------------------*/
static void BQB_Handle_LEConnectionUpdateInd(PGATTDemo pGATTDemo,
        PBlueAPI_LEConnectionUpdateInd pConnectionUpdateInd)
{
    BQB_CmdPrint( pGATTDemo, "<-- LEConnectionUpdateInd: MDL=%d Interval=%d-%d Latency=%d Supervision=%d\r\n%s",
                  pConnectionUpdateInd->local_MDL_ID,
                  pConnectionUpdateInd->connIntervalMin,
                  pConnectionUpdateInd->connIntervalMax,
                  pConnectionUpdateInd->connLatency,
                  pConnectionUpdateInd->supervisionTimeout,
                  pGATTDemo->CmdIF.cPrompt );
}

/*----------------------------------------------------------------------------
 * handle gattdHandle_LEConnectionParameterInfo
 * --------------------------------------------------------------------------*/
static void BQB_Handle_LEConnectionParameterInfo(PGATTDemo pGATTDemo,
        PBlueAPI_LEConnectionParameterInfo pConnectionParameterInfo)
{
    BQB_CmdPrint( pGATTDemo, "<-- LEConnectionParameterInfo: MDL=%d Interval=%d Latency=%d Supervision=%d\r\n%s",
                  pConnectionParameterInfo->local_MDL_ID,
                  pConnectionParameterInfo->connInterval,
                  pConnectionParameterInfo->connLatency,
                  pConnectionParameterInfo->supervisionTimeout,
                  pGATTDemo->CmdIF.cPrompt );
}

static void BQB_Handle_CreateLEDataChannelRsp(PGATTDemo pGATTDemo,
                                          PBlueAPI_CreateLEDataChannelRsp pCreateChanRsp)
{
    PGATTLink pLink;
    PGATTLEChannel pChan;
    BQB_CmdPrint( pGATTDemo, "<-- CreateLEDataChannelRsp: local_MDL_ID 0x%04x channel 0x%04x cause=%s subCause=0x%04x(%s)\r\n",
                            pCreateChanRsp->local_MDL_ID,
                            pCreateChanRsp->channel,
                            blueAPI_CauseString(pCreateChanRsp->cause),
                            pCreateChanRsp->subCause,
                            gattdSubCause2Str(pCreateChanRsp->subCause)
                            );
    if(pCreateChanRsp->cause == blueAPI_CauseSuccess)
    {
        pLink = BQB_LinkFindByLocal_MDL_ID(pGATTDemo,pCreateChanRsp->local_MDL_ID);
        if(pLink)
        {
            pChan = BQB_LEChanAllocate(pGATTDemo,pLink,pCreateChanRsp->channel);
            if(pChan)
            {
                pChan->isDataChanConnected = true;
            }
            else{
                    BQB_CmdPrint( pGATTDemo, "<-- CreateLEDataChannelRsp: allocate failed\r\n");
                }
        }
    }
  
}
static void BQB_Handle_CreateLEDataChannelInd(PGATTDemo pGATTDemo,
                                          PBlueAPI_CreateLEDataChannelInd pCreateChanInd)
{
    PGATTLink pLink;
    PGATTLEChannel pChan;
    BQB_CmdPrint( pGATTDemo, "<-- CreateLEDataChannelInd: local_MDL_ID 0x%04x channel 0x%04x \r\n",
                              pCreateChanInd->local_MDL_ID,
                              pCreateChanInd->channel
                              );
    pLink = BQB_LinkFindByLocal_MDL_ID(pGATTDemo,pCreateChanInd->local_MDL_ID);
    if(pLink)
    {
        pChan = BQB_LEChanAllocate(pGATTDemo,pLink,pCreateChanInd->channel);
        if(pChan)
        {
            pChan->isDataChanConnected = true;
            blueAPI_CreateLEDataChannelConf(pLink->local_MDL_ID,
                                      pCreateChanInd->channel,
                                      210,
                                      23,
                                      3,
                                      0,
                                      blueAPI_CauseAccept);
        
        }

    }
}
static void BQB_Handle_DisconnectLEDataChannelRsp(PGATTDemo pGATTDemo,
                                          PBlueAPI_DisconnectLEDataChannelRsp pDiscChanRsp)
{
    PGATTLink pLink;
    PGATTLEChannel pChan;
    BQB_CmdPrint( pGATTDemo, "<-- DisconnectLEDataChannelRsp: local_MDL_ID 0x%04x channel 0x%04x cause=%d subcause=0x%04x(%s)\r\n",
                              pDiscChanRsp->local_MDL_ID,
                              pDiscChanRsp->channel,
                              pDiscChanRsp->cause,
                              pDiscChanRsp->subCause,
                              gattdSubCause2Str(pDiscChanRsp->subCause)
                              );
    pLink = BQB_LinkFindByLocal_MDL_ID(pGATTDemo,pDiscChanRsp->local_MDL_ID);
    if(pLink)
    {
        pChan = BQB_LEChanFind(pGATTDemo,pLink,pDiscChanRsp->channel);
      
        BQB_LEChanRelease(pGATTDemo,pChan);
    }

}
static void BQB_Handle_DisconnectLEDataChannelInd(PGATTDemo pGATTDemo,
                                          PBlueAPI_DisconnectLEDataChannelInd pDiscChanInd)
{
    PGATTLink pLink;
    PGATTLEChannel pChan;
    BQB_CmdPrint( pGATTDemo, "<-- DisconnectLEDataChannelInd: local_MDL_ID 0x%04x channel 0x%x subcause 0x%04x(%s) \r\n",
                              pDiscChanInd->local_MDL_ID,
                              pDiscChanInd->channel,
                              pDiscChanInd->subCause,
                              gattdSubCause2Str(pDiscChanInd->subCause));
    pLink = BQB_LinkFindByLocal_MDL_ID(pGATTDemo,pDiscChanInd->local_MDL_ID);
    if(pLink)
    {
        pChan = BQB_LEChanFind(pGATTDemo,pLink,pDiscChanInd->channel);
        BQB_LEChanRelease(pGATTDemo,pChan);
        blueAPI_DisconnectLEDataChannelConf(pLink->local_MDL_ID,pDiscChanInd->channel,
                                      blueAPI_CauseAccept);
    }

}

static void BQB_Handle_SendLEFlowControlCreditRsp(PGATTDemo pGATTDemo,
                                          PBlueAPI_SendLEFlowControlCreditRsp pFlowRsp)
{
    BQB_CmdPrint( pGATTDemo, "<-- SendLEFlowControlCreditRsp: local_MDL_ID 0x%04x cause=%d subcause 0x%04x\r\n",
                              pFlowRsp->local_MDL_ID,
                              pFlowRsp->cause,
                              pFlowRsp->subCause
                              );

}

static void BQB_Handle_LEDataInd(PGATTDemo pGATTDemo,
                                          PBlueAPI_LEDataInd pDataInd)
{
    int length = pDataInd->valueLength;
    uint8_t *value =  pDataInd->data + pDataInd->gap;

    BQB_CmdPrint( pGATTDemo, "<-- LEDataInd: local_MDL_ID 0x%04x channel 0x%04x valueLength %d gap %d\r\n",
                              pDataInd->local_MDL_ID,
                              pDataInd->channel,
                              pDataInd->valueLength,
                              pDataInd->gap
                              );
    BQB_HexDump( pGATTDemo, "value=", length, value );
    
    blueAPI_LEDataConf(pDataInd->local_MDL_ID,pDataInd->channel,
                                      blueAPI_CauseSuccess);
}

static void BQB_Handle_LEDataRsp(PGATTDemo pGATTDemo,
                                          PBlueAPI_LEDataRsp pDataRsp)
{
    BQB_CmdPrint( pGATTDemo, "<-- LEDataRsp: local_MDL_ID 0x%04x channel 0x%04x cause %d subcause 0x%04x\r\n",
                              pDataRsp->local_MDL_ID,
                              pDataRsp->channel,
                              pDataRsp->cause,
                              pDataRsp->subCause
                              );
}

static void BQB_Handle_LEDataChannelParameterInfo(PGATTDemo pGATTDemo,
                                          PBlueAPI_LEDataChannelParameterInfo pConInfo)
{
    BQB_CmdPrint( pGATTDemo, "<-- LEDataChannelParameterInfo: local_MDL_ID 0x%04x channel 0x%04x remote_mtu %d remote_mps %d remote_credits %d maxDsCredits %d\r\n",
                              pConInfo->local_MDL_ID,
                              pConInfo->channel,
                              pConInfo->remote_mtu,
                              pConInfo->remote_mps,
                              pConInfo->remote_initial_credits,
                              pConInfo->maxDsCredits
                              );
}

static void BQB_Handle_LEDataChannelCreditsAlertInfo(PGATTDemo pGATTDemo,
                                          PBlueAPI_LEDataChannelCreditsAlertInfo pCreditInfo)
{
    BQB_CmdPrint( pGATTDemo, "<-- LEDataChannelCreditsAlertInfo: local_MDL_ID 0x%04x channel 0x%04x \r\n",
                              pCreditInfo->local_MDL_ID,
                              pCreditInfo->channel);
    blueAPI_SendLEFlowControlCreditReq( pCreditInfo->local_MDL_ID,
                                          pCreditInfo->channel, 5);
}

static void BQB_Handle_LEDataChannelDeleteInfo(PGATTDemo pGATTDemo,
                                          PBlueAPI_LEDataChannelDeleteInfo pDiscInfo)
{
    BQB_CmdPrint( pGATTDemo, "<-- LEDataChannelDeleteInfo: local_MDL_ID 0x%04x channel 0x%04x \r\n",
                              pDiscInfo->local_MDL_ID,
                              pDiscInfo->channel
                              );
}

static void BQB_Handle_LEPsmSecuritySetRsp(PGATTDemo pGATTDemo,
                                          PBlueAPI_LEPsmSecuritySetRsp pSecRsp)
{
    BQB_CmdPrint( pGATTDemo, "<-- LEPsmSecuritySetRsp: cause %d\r\n",
                              pSecRsp->cause
                              );
}
#if 0
static void BQB_Handle_DeviceConfigAppearanceGetRsp(PGATTDemo pGATTDemo,
                                          PBlueAPI_DeviceConfigAppearanceGetRsp pAppRsp)
{
    BQB_CmdPrint( pGATTDemo, "<-- DeviceConfigAppearanceGetRsp: cause %d appearance = 0x%04x\r\n",
                              pAppRsp->cause,
                              pAppRsp->appearance
                              );
}
static void BQB_Handle_DeviceConfigDeviceNameGetRsp(PGATTDemo pGATTDemo,
                                          PBlueAPI_DeviceConfigDeviceNameGetRsp pNameRsp)
{
    uint8_t *value =  pNameRsp->deviceName + pNameRsp->gap;
    
    BQB_CmdPrint( pGATTDemo, "<-- DeviceConfigDeviceNameGetRsp: cause %d lenth=%d gap=%d\r\n",
                              pNameRsp->cause,
                              pNameRsp->nameLength,
                              pNameRsp->gap
                              );
    BQB_HexDump( pGATTDemo, "DeviceName=", pNameRsp->nameLength, value );
}
static void BQB_Handle_DeviceConfigPerPrefConnParamGetRsp(PGATTDemo pGATTDemo,
                                          PBlueAPI_DeviceConfigPerPrefConnParamGetRsp pPerRsp)
{
    BQB_CmdPrint( pGATTDemo, "<-- DeviceConfigPerPrefConnParamGetRsp: cause %d connIntervalMin=0x%04x connIntervalMax=0x%04x slaveLatency=0x%04x supervisionTimeout=0x%04x\r\n",
                              pPerRsp->cause,
                              pPerRsp->conn.connIntervalMin,
                              pPerRsp->conn.connIntervalMax,
                              pPerRsp->conn.slaveLatency,
                              pPerRsp->conn.supervisionTimeout
                              );
}
#endif
/*----------------------------------------------------------------------------
 * BlueAPI message handler.
 * Returns non-zero value if calling routine may release message buffer.
 * --------------------------------------------------------------------------*/


void BQB_HandleBlueAPIMessage( PGATTDemo pGATTDemo, PBlueAPI_UsMessage pMsg )

{
    bool      ReleaseBuffer = true;
    switch ( pMsg->Command )
    {
    default:
        BQB_CmdPrint( pGATTDemo, "gattdemo: unhandled 0x%x: %s\r\n%s",
                      pMsg->Command,
                      blueAPI_CommandString((TBlueAPI_Command)pMsg->Command),
                      pGATTDemo->CmdIF.cPrompt );
        break;

    case blueAPI_EventRegisterRsp:
        ReleaseBuffer = BQB_Handle_RegisterRsp( pGATTDemo, &pMsg->p.RegisterRsp );
        break;

    case blueAPI_EventReleaseRsp:
        ReleaseBuffer = BQB_Handle_ReleaseRsp( pGATTDemo, &pMsg->p.ReleaseRsp );
        break;

    case blueAPI_EventActInfo:
        ReleaseBuffer = BQB_Handle_ActInfo( pGATTDemo, &pMsg->p.ActInfo );
        break;

    case blueAPI_EventDeviceConfigSetRsp:
        BQB_Handle_DeviceConfigSetRsp(pGATTDemo, &pMsg->p.DeviceConfigSetRsp);
        break;

    case blueAPI_EventCreateMDLInd:
        ReleaseBuffer = BQB_Handle_CreateMDLInd(
                            pGATTDemo, &pMsg->p.CreateMDLInd );
        break;

    case blueAPI_EventConnectMDLRsp:
        ReleaseBuffer = BQB_Handle_ConnectMDLRsp(
                            pGATTDemo, &pMsg->p.ConnectMDLRsp );
        break;

    case blueAPI_EventMCLStatusInfo:
        ReleaseBuffer = BQB_Handle_MCLStatusInfo(
                            pGATTDemo, &pMsg->p.MCLStatusInfo );
        break;

    case blueAPI_EventConnectMDLInfo:
        ReleaseBuffer = BQB_Handle_ConnectMDLInfo(
                            pGATTDemo, &pMsg->p.ConnectMDLInfo );
        break;

    case blueAPI_EventDisconnectMDLRsp:
        ReleaseBuffer = BQB_Handle_DisconnectMDLRsp(
                            pGATTDemo, &pMsg->p.DisconnectMDLRsp );
        break;

    case blueAPI_EventDisconnectMDLInd:
        ReleaseBuffer = BQB_Handle_DisconnectMDLInd(
                            pGATTDemo, &pMsg->p.DisconnectMDLInd );
        break;

    case blueAPI_EventDeleteMDLInfo:
        ReleaseBuffer = BQB_Handle_DeleteMDLInfo(
                            pGATTDemo, &pMsg->p.DeleteMDLInfo );
        break;

    case blueAPI_EventInternalEventInfo:
        ReleaseBuffer = BQB_Handle_InternalEventInfo(
                            pGATTDemo, &pMsg->p.InternalEventInfo );
        break;

    case blueAPI_EventACLStatusInfo:
        ReleaseBuffer = BQB_Handle_ACLStatusInfo(
                            pGATTDemo, &pMsg->p.ACLStatusInfo );
        break;

    case blueAPI_EventGATTServiceRegisterRsp:
        ReleaseBuffer = BQB_Handle_GATTServiceRegisterRsp(
                            pGATTDemo, &pMsg->p.GATTServiceRegisterRsp );
        break;

    case blueAPI_EventGATTAttributeUpdateRsp:
        ReleaseBuffer = BQB_Handle_GATTAttributeUpdateRsp(
                            pGATTDemo, &pMsg->p.GATTAttributeUpdateRsp );
        break;

    case blueAPI_EventGATTAttributeUpdateStatusInd:
        ReleaseBuffer = BQB_Handle_GATTAttributeUpdateStatusInd(
                            pGATTDemo, &pMsg->p.GATTAttributeUpdateStatusInd );
        break;

    case blueAPI_EventGATTAttributeReadInd:
        ReleaseBuffer = BQB_Handle_GATTAttributeReadInd(
                            pGATTDemo, &pMsg->p.GATTAttributeReadInd );
        break;

    case blueAPI_EventGATTAttributeWriteInd:
    case blueAPI_EventGATTAttributeWriteCommandInfo:
        ReleaseBuffer = BQB_Handle_GATTAttributeWriteIndCommandInfo(
                            pGATTDemo, (TBlueAPI_Command)pMsg->Command,
                            &pMsg->p.GATTAttributeWriteInd );
        break;

    case blueAPI_EventGATTCCCDInfo:
        ReleaseBuffer = BQB_Handle_GATTCCCDInfo(
                            pGATTDemo, &pMsg->p.GATTCCCDInfo );
        break;



    case blueAPI_EventGATTDiscoveryRsp:
        ReleaseBuffer = BQB_Handle_GATTDiscoveryRsp(
                            pGATTDemo, &pMsg->p.GATTDiscoveryRsp );
        break;

    case blueAPI_EventGATTDiscoveryInd:
        ReleaseBuffer = BQB_Handle_GATTDiscoveryInd(
                            pGATTDemo, &pMsg->p.GATTDiscoveryInd );
        break;



    case blueAPI_EventGATTAttributeReadRsp:
        ReleaseBuffer = BQB_Handle_GATTAttributeReadRsp(
                            pGATTDemo, &pMsg->p.GATTAttributeReadRsp );
        break;
#if 0
    case blueAPI_EventGATTAttributeReadMultipleRsp:
        ReleaseBuffer = BQB_Handle_GATTAttributeReadMultipleRsp(
                                       pGATTDemo, &pMsg->p.GATTAttributeReadMultipleRsp );
      break;
#endif
    case blueAPI_EventGATTAttributeWriteRsp:
        ReleaseBuffer = BQB_Handle_GATTAttributeWriteRsp(
                            pGATTDemo, &pMsg->p.GATTAttributeWriteRsp );
        break;

    case blueAPI_EventGATTAttributeInd:
    case blueAPI_EventGATTAttributeNotificationInfo:
        ReleaseBuffer = BQB_Handle_GATTAttributeNotifInd(
                            pGATTDemo, (TBlueAPI_Command)pMsg->Command,
                            &pMsg->p.GATTAttributeInd );
        break;


    case blueAPI_EventGATTSecurityRsp:
        BQB_Handle_GATTSecurityRsp(pGATTDemo, &pMsg->p.GATTSecurityRsp);
        break;

    case blueAPI_EventGATTServerStoreInd:
        BQB_Handle_GATTServerStoreInd(pGATTDemo, &pMsg->p.GATTServerStoreInd);
        break;

    case blueAPI_EventGATTMtuSizeInfo:
        BQB_Handle_GATTMtuSizeInfo(pGATTDemo, &pMsg->p.GATTMtuSizeInfo);
        break;

    case blueAPI_EventLEAdvertiseRsp:
        BQB_Handle_LEAdvertiseRsp(pGATTDemo, &pMsg->p.LEAdvertiseRsp);
        break;

    case blueAPI_EventLEAdvertiseParameterSetRsp:
        BQB_Handle_LEAdvertiseParameterRsp(pGATTDemo, &pMsg->p.LEAdvertiseParameterSetRsp);
        break;

    case blueAPI_EventLEAdvertiseDataSetRsp:
        BQB_Handle_LEAdvertiseDataSetRsp(pGATTDemo, &pMsg->p.LEAdvertiseDataSetRsp);
        break;

    case blueAPI_EventLEScanRsp:
        BQB_Handle_LEScanRsp(pGATTDemo, &pMsg->p.LEScanRsp);
        break;

    case blueAPI_EventLEScanInfo:
        BQB_Handle_LEScanInfo(pGATTDemo, &pMsg->p.LEScanInfo);
        break;

    case blueAPI_EventLEModifyWhitelistRsp:
        BQB_Handle_LEModifyWhitelistRsp(pGATTDemo, &pMsg->p.LEModifyWhitelistRsp);
        break;

    case blueAPI_EventLEConnectionUpdateRsp:
        BQB_Handle_LEConnectionUpdateRsp(pGATTDemo, &pMsg->p.LEConnectionUpdateRsp);
        break;

    case blueAPI_EventLEConnectionUpdateInd:
        BQB_Handle_LEConnectionUpdateInd(pGATTDemo, &pMsg->p.LEConnectionUpdateInd);
        break;

    case blueAPI_EventLEConnectionParameterInfo:
        BQB_Handle_LEConnectionParameterInfo(pGATTDemo, &pMsg->p.LEConnectionParameterInfo);
        break;

#if (F_BT_LE_PRIVACY_MODE)
    case blueAPI_EventLEPrivacyModeRsp:
        BQB_Handle_LEPrivacyModeRsp(pGATTDemo, &pMsg->p.LEPrivacyModeRsp);
        break;
#endif /* (F_BT_LE_PRIVACY_MODE) */

    case blueAPI_EventAuthResultInd:
        BQB_Handle_AuthResultInd(pGATTDemo, &pMsg->p.AuthResultInd);
        break;

    case blueAPI_EventPairableModeSetRsp:
        BQB_Handle_PairableModeSetRsp(pGATTDemo, &pMsg->p.PairableModeSetRsp);
        break;

    case blueAPI_EventUserPasskeyReqInd:
        BQB_Handle_UserPasskeyReqInd(pGATTDemo, &pMsg->p.UserPasskeyReqInd);
        break;

    case blueAPI_EventUserPasskeyNotificationInfo:
        BQB_Handle_UserPasskeyNotificationInfo(pGATTDemo, &pMsg->p.UserPasskeyNotificationInfo);
        break;


    case blueAPI_EventUserPasskeyReqReplyRsp:
        BQB_Handle_UserPasskeyReqReplyRsp(pGATTDemo, &pMsg->p.UserPasskeyReqReplyRsp);
        break;

    case blueAPI_EventRemoteOOBDataReqInd:
        BQB_Handle_RemoteOOBDataReqInd(pGATTDemo, &pMsg->p.RemoteOOBDataReqInd);
        break;

#if (BQB_PREPARE_WRITE_SUPPPRT)
    case blueAPI_EventGATTAttributePrepareWriteInd:
        ReleaseBuffer = BQB_Handle_GATTAttributePrepareWriteInd(
                                         pGATTDemo, (TBlueAPI_Command)pMsg->Command,
                                         (PBlueAPI_GATTAttributeWriteInd)&pMsg->p.GATTAttributeWriteInd );
        break;

    case blueAPI_EventGATTAttributePrepareWriteRsp:
        BQB_Handle_GATTAttributePrepareWriteRsp(pGATTDemo, &pMsg->p.GATTAttributePrepareWriteRsp);
        break;
   
    case blueAPI_EventGATTAttributeExecuteWriteRsp:
        BQB_Handle_GATTAttributeExecuteWriteRsp(pGATTDemo, &pMsg->p.GATTAttributeExecuteWriteRsp);
        break;
      
    case blueAPI_EventGATTAttributeExecuteWriteInd:
        BQB_Handle_GATTAttributeExecuteWriteInd(pGATTDemo, &pMsg->p.GATTAttributeExecuteWriteInd);
        break;
#endif
    case blueAPI_EventCreateLEDataChannelRsp:
        BQB_Handle_CreateLEDataChannelRsp(pGATTDemo, &pMsg->p.CreateLEDataChannelRsp);
        break;
    
    case blueAPI_EventCreateLEDataChannelInd:
        BQB_Handle_CreateLEDataChannelInd(pGATTDemo, &pMsg->p.CreateLEDataChannelInd);
        break;
    case blueAPI_EventDisconnectLEDataChannelRsp:
        BQB_Handle_DisconnectLEDataChannelRsp(pGATTDemo, &pMsg->p.DisconnectLEDataChannelRsp);
        break;
      
    case blueAPI_EventDisconnectLEDataChannelInd:
        BQB_Handle_DisconnectLEDataChannelInd(pGATTDemo, &pMsg->p.DisconnectLEDataChannelInd);
        break;

    case blueAPI_EventSendLEFlowControlCreditRsp:
        BQB_Handle_SendLEFlowControlCreditRsp(pGATTDemo, &pMsg->p.SendLEFlowControlCreditRsp);
        break;
    case blueAPI_EventLEDataInd:
        BQB_Handle_LEDataInd(pGATTDemo, &pMsg->p.LEDataInd);
        break;
    case blueAPI_EventLEDataRsp:
        BQB_Handle_LEDataRsp(pGATTDemo, &pMsg->p.LEDataRsp);
        break;
    case blueAPI_EventLEDataChannelParameterInfo:
        BQB_Handle_LEDataChannelParameterInfo(pGATTDemo, &pMsg->p.LEDataChannelParameterInfo);
        break;
    case blueAPI_EventLEDataChannelCreditsAlertInfo:
        BQB_Handle_LEDataChannelCreditsAlertInfo(pGATTDemo, &pMsg->p.LEDataChannelCreditsAlertInfo);
        break;
    case blueAPI_EventLEDataChannelDeleteInfo:
        BQB_Handle_LEDataChannelDeleteInfo(pGATTDemo, &pMsg->p.LEDataChannelDeleteInfo);
        break;
    case blueAPI_EventLEPsmSecuritySetRsp:
        BQB_Handle_LEPsmSecuritySetRsp(pGATTDemo, &pMsg->p.LEPsmSecuritySetRsp);
        break;
#if 0
    case blueAPI_EventDeviceConfigAppearanceGetRsp:
        BQB_Handle_DeviceConfigAppearanceGetRsp(pGATTDemo, &pMsg->p.DeviceConfigAppearanceGetRsp);
        break;
    case blueAPI_EventDeviceConfigDeviceNameGetRsp:
        BQB_Handle_DeviceConfigDeviceNameGetRsp(pGATTDemo, &pMsg->p.DeviceConfigDeviceNameGetRsp);
        break;
    case blueAPI_EventDeviceConfigPerPrefConnParamGetRsp:
        BQB_Handle_DeviceConfigPerPrefConnParamGetRsp(pGATTDemo, &pMsg->p.DeviceConfigPerPrefConnParamGetRsp);
        break;
#endif

    }

#if (F_SBSP_API)
    return ( ReleaseBuffer ? 1 : 0 );
#else
    if (ReleaseBuffer)
        blueAPI_BufferRelease(pMsg);
#endif
}

/*----------------------------------------------------------------------------
 * link
 * --------------------------------------------------------------------------*/
PGATTLink BQB_LinkAllocate(PGATTDemo pGATTDemo, uint8_t * pBD)
{
    int i;
    PGATTLink pLink;
    for (i = 0; i < BQB_MAX_LINKS; i++)
    {
        pLink = &pGATTDemo->linkTable[i];
        if (pLink->isUsed && ((memcmp(pLink->RemoteBd, pBD, BLUE_API_BD_SIZE) == 0)))
        {
            return pLink;
        }
    }

    for (i = 0; i < BQB_MAX_LINKS; i++)
    {
        pLink = &pGATTDemo->linkTable[i];
        if (!pLink->isUsed)
        {
            memset(pLink, 0, sizeof(*pLink));
            memcpy(pLink->RemoteBd, pBD, 6);
            pLink->isUsed = true;
            pLink->RemoteBdValid = true;
            pLink->idx = i;
            return pLink;
        }
    }
    return NULL;
}

void BQB_LinkRelease(PGATTDemo pGATTDemo, PGATTLink pLink)
{
    if (pLink)
    {
        pLink->isUsed = false;
        memset(pLink, 0, sizeof(TGATTLink));
    }
}

PGATTLink BQB_LinkFindByLocal_MDL_ID(PGATTDemo pGATTDemo, uint16_t local_MDL_ID)
{
    int i;
    PGATTLink pLink;
    for (i = 0; i < BQB_MAX_LINKS; i++)
    {
        pLink = &pGATTDemo->linkTable[i];
        if (pLink->isUsed && (pLink->local_MDL_ID == local_MDL_ID))
        {
            return pLink;
        }
    }
    return NULL;
}

PGATTLink BQB_LinkFindByBD(PGATTDemo pGATTDemo, uint8_t * pBD)
{
    int i;
    PGATTLink pLink;
    for (i = 0; i < BQB_MAX_LINKS; i++)
    {
        pLink = &pGATTDemo->linkTable[i];
        if (pLink->isUsed && ((memcmp(pLink->RemoteBd, pBD, BLUE_API_BD_SIZE) == 0)))
        {
            return pLink;
        }
    }
    return NULL;
}

PGATTLink BQB_LinkFindByRole(PGATTDemo pGATTDemo, uint16_t role)
{
    int i;
    PGATTLink pLink;
    for (i = 0; i < BQB_MAX_LINKS; i++)
    {
        pLink = &pGATTDemo->linkTable[i];
        if (pLink->isUsed && (pLink->role == role))
        {
            return pLink;
        }
    }
    return NULL;
}

PGATTLEChannel BQB_LEChanAllocate(PGATTDemo pGATTDemo,PGATTLink pLink,uint16_t channel)
{
    int i;
    PGATTLEChannel pChan;
    for (i = 0; i < GATTDEMO_MAX_LE_CHANNELS; i++)
    {
        pChan= &pLink->LEChannel[i];
        if(pChan->isUsed &&(pChan->channel == channel))
        {
            return pChan;
        }
    }

    for (i = 0; i < GATTDEMO_MAX_LE_CHANNELS; i++)
    {
        pChan= &pLink->LEChannel[i];
        if(!pChan->isUsed)
        {
            pChan->isUsed = true;
            pChan->isDataChanConnected = false;
            pChan->channel = channel;

            return pChan;
        }
    }
    return NULL;
}

void BQB_LEChanClear(PGATTDemo pGATTDemo,PGATTLink pLink)
{
    int i;
    PGATTLEChannel pChan;

    if(pLink)
    {
        for (i = 0; i < GATTDEMO_MAX_LE_CHANNELS; i++)
        {
            pChan = &pLink->LEChannel[i];
            memset(pChan,0,sizeof(TGATTLEChannel));
        }   
    }
}

void BQB_LEChanRelease(PGATTDemo pGATTDemo,PGATTLEChannel pChan)
{
    if(pChan)
    {
        pChan->isUsed = false;
        memset(pChan,0,sizeof(TGATTLEChannel));
    }
}

PGATTLEChannel BQB_LEChanFind(PGATTDemo pGATTDemo,PGATTLink pLink, uint16_t channel)
{
    int i;
    PGATTLEChannel pChan;
    for (i = 0; i < GATTDEMO_MAX_LE_CHANNELS; i++)
    {
        pChan = &pLink->LEChannel[i];
        if(pChan->isUsed && (pChan->channel == channel))
        {
            return pChan;
        }
    }
    return NULL;
}


/* select database, call it before "srvreg" command !!!!!!!!!!!!!!!*/
TGATTDemoResult BQB_SelectDB(PGATTDemo pGATTDemo, TGATTDemoParseResult *pParseResult)
{
    if (0 == pParseResult->dwParameter[0])
    {
        BQB_SelectDatabase(SMALL_DATABASE);
        BQB_CmdPrint(pGATTDemo, "select small db ok\r\n");
    }
    else if (1 == pParseResult->dwParameter[0])
    {
        BQB_SelectDatabase(LARGE_DATABASE1);
        BQB_CmdPrint(pGATTDemo, "select large db1 ok\r\n");
    }
    else
    {
        BQB_CmdPrint(pGATTDemo, "invalid db index!\r\n");
        return gattdResultError;
    }
    BQB_ServiceInfoInit();
    return gattdResultOk;
}

/* select adv data array index */
TGATTDemoResult BQB_SelectAdvDataIndex(PGATTDemo pGATTDemo, TGATTDemoParseResult *pParseResult)
{
    if (0 == pParseResult->dwParameter[0])
    {
        AdvDataIndex = ADV_DATA_IND_0;
        BQB_CmdPrint(pGATTDemo, "select bBQBAdData0[] ok\r\n");
    }
    else if (1 == pParseResult->dwParameter[0])
    {
        AdvDataIndex = ADV_DATA_IND_1;
        BQB_CmdPrint(pGATTDemo, "select bBQBAdData1[] ok\r\n");
    }
    else if (2 == pParseResult->dwParameter[0])
    {
        AdvDataIndex = ADV_DATA_IND_2;
        BQB_CmdPrint(pGATTDemo, "select bBQBAdData2[] ok\r\n");
    }
    else
    {
        BQB_CmdPrint(pGATTDemo, "invalid adv data index!\r\n");
        return gattdResultError;
    }
    return gattdResultOk;
}

/* set advertising discovery mode, advertising data must use bBQBAdData0[] */
TGATTDemoResult BQB_SetDiscoveryMode(PGATTDemo pGATTDemo, TGATTDemoParseResult *pParseResult)
{
    if (0 == pParseResult->dwParameter[0])  /* Non-Discoverable */
    {
        bBQBAdData0[2] = 0x04;  /* cssv4 FLAGS */   /* BR/EDR not support */
    }
    else if (1 == pParseResult->dwParameter[0]) /* Limitted-Discoverable, BR/EDR not support */
    {
        bBQBAdData0[2] = 0x05;
    }
    else if (2 == pParseResult->dwParameter[0]) /* General-Discoverable, BR/EDR not support */
    {
        bBQBAdData0[2] = 0x06;
    }
    else
    {
        BQB_CmdPrint(pGATTDemo, "invalid adv discoverable mdoe pam\r\n");
        return gattdResultError;
    }
    return gattdResultOk;
}


