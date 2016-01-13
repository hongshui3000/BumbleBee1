enum { __FILE_NUM__= 0 };

/**********************************************************************!KA****
 *
 * $Header: /var/lib/cvs/sw/src/app/demo/gattdemo/gattdutl.c,v 1.30.2.1 2013/10/17 09:51:18 mn Exp $
 *
 * File:        $RCSfile: gattdutl.c,v $
 * Version:     $Name: P_SRP1290_V1_0 $
 *
 * Archive:     $Source: /var/lib/cvs/sw/src/app/demo/gattdemo/gattdutl.c,v $
 * Revision:    $Revision: 1.30.2.1 $
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
 *         GATTDEMO utilities
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
#include <bterrcod.h>

#define CASE2STR(x) case x : pStr = (char *)#x; break
static const char cUnknownErrorCode[] = "Unknown error code";

#if !defined(UUID_GATT)
#define UUID_GAP                        0x1800
#define UUID_GATT                       0x1801
#endif

/*----------------------------------------------------------------------------
 * partial HCI error code to string conversion
 * --------------------------------------------------------------------------*/

static const char * BQB_HCIErrorCode2Str( uint8_t bErrorCode )
{
    const char * pStr = NULL;

    switch ( bErrorCode )
    {
        CASE2STR(HCI_ERR_NOCONNECTION);
        CASE2STR(HCI_ERR_AUTHENTICATION_FAILED);
        CASE2STR(HCI_ERR_KEY_MISSING);
        CASE2STR(HCI_ERR_CONNECTION_TIMEOUT);
        CASE2STR(HCI_ERR_COMMAND_DISALLOWED);
        CASE2STR(HCI_ERR_INVALID_HCI_PARAMETER_VALUE);
        CASE2STR(HCI_ERR_OTHER_END_TERMINATE_13);
        CASE2STR(HCI_ERR_OTHER_END_TERMINATE_14);
        CASE2STR(HCI_ERR_OTHER_END_TERMINATE_15);
        CASE2STR(HCI_ERR_CONNECTION_TERMINATE_LOCALLY);
        CASE2STR(HCI_ERR_PARING_NOT_ALLOWED);
        CASE2STR(HCI_ERR_UNACCEPTABLE_CONNECTION_INTERVAL);
        CASE2STR(HCI_ERR_DIRECTED_ADVERTISING_TIMEOUT);
        CASE2STR(HCI_ERR_CONNECTION_TERMINATED_DUE_TO_MIC_FAILURE);
        CASE2STR(HCI_ERR_CONNECTION_FAILED_TO_BE_ESTABLISHED);
    default:
        pStr = "HCI_ERR";
        break;
    }

    return ( pStr );
}

/*----------------------------------------------------------------------------
 * partial L2CAP error code to string conversion
 * --------------------------------------------------------------------------*/

static const char * BQB_L2CAPErrorCode2Str( uint8_t bErrorCode )
{
    const char * pStr = NULL;

    switch ( bErrorCode )
    {
        CASE2STR(L2CAP_ERR_REFUS_INV_PSM);
        CASE2STR(L2CAP_ERR_REFUS_SEC_BLOCK);
        CASE2STR(L2CAP_ERR_REFUS_NO_RESOURCE);
		CASE2STR(L2CAP_ERR_ILLEGAL_PARAMETER);
		CASE2STR(L2CAP_ERR_INSUFFICIENT_AUTHENTICATION);
		CASE2STR(L2CAP_ERR_INSUFFICIENT_AUTHORIZATION);
		CASE2STR(L2CAP_ERR_INSUFFICIENT_KEY_SIZE);
		CASE2STR(L2CAP_ERR_INSUFFICIENT_ENCRYPTION);
		CASE2STR(L2CAP_ERR_NO_RESOURCE);
		CASE2STR(L2CAP_ERR_CREDITS_EXCEED_RANGE);
    default:
        pStr = "L2CAP_ERR";
        break;
    }

    return ( pStr );
}

/*----------------------------------------------------------------------------
 * ATT error code to string conversion (XXXXMJMJ -> common lib ..)
 * --------------------------------------------------------------------------*/

static const char * BQB_ATTErrorCode2Str( uint8_t bErrorCode )
{
    const char * pStr = NULL;

    if ( bErrorCode >= 0x80 )
    {
        pStr = "applic. error code";
    }
    else
    {
        switch ( bErrorCode )
        {
            /*CASE2STR(ATT_OK);*/
            CASE2STR(ATT_ERR_INVALID_HANDLE);
            CASE2STR(ATT_ERR_READ_NOT_PERMITTED);
            CASE2STR(ATT_ERR_WRITE_NOT_PERMITTED);
            CASE2STR(ATT_ERR_INVALID_PDU);
            CASE2STR(ATT_ERR_INSUFFICIENT_AUTHENTICATION);
            CASE2STR(ATT_ERR_UNSUPPORTED_REQUEST);
            CASE2STR(ATT_ERR_INVALID_OFFSET);
            CASE2STR(ATT_ERR_INSUFFICIENT_AUTHORIZATION);
            CASE2STR(ATT_ERR_PREPARE_QUEUE_FULL);
            CASE2STR(ATT_ERR_ATTR_NOT_FOUND);
            CASE2STR(ATT_ERR_ATTR_NOT_LONG);
            CASE2STR(ATT_ERR_INSUFFICIENT_KEY_SIZE);
            CASE2STR(ATT_ERR_INVALID_VALUE_SIZE);
            CASE2STR(ATT_ERR_UNLIKELY);
            CASE2STR(ATT_ERR_INSUFFICIENT_ENCRYPTION);
            CASE2STR(ATT_ERR_UNSUPPORTED_GROUP_TYPE);
            CASE2STR(ATT_ERR_INSUFFICIENT_RESOURCES);
        default:
            pStr = cUnknownErrorCode;
            break;
        }
    }

    return ( pStr );
}

/*----------------------------------------------------------------------------
 * GATT error code to string conversion
 * --------------------------------------------------------------------------*/

static const char * BQB_GATTErrorCode2Str( uint16_t wCause )
{
    const char * pStr = NULL;

    switch ( wCause )
    {
        /*CASE2STR(GATT_SUCCESS);*/
        CASE2STR(GATT_ERR_OUT_OF_RESOURCE);
        CASE2STR(GATT_ERR_UNSUPPORTED);
        CASE2STR(GATT_ERR_ILLEGAL_ROLE);
        CASE2STR(GATT_ERR_ILLEGAL_STATE);
        CASE2STR(GATT_ERR_ILLEGAL_CODING);
        CASE2STR(GATT_ERR_ILLEGAL_HANDLE);
        CASE2STR(GATT_ERR_ILLEGAL_PARAMETER);
        CASE2STR(GATT_ERR_INTERNAL);
        CASE2STR(GATT_ERR_NOT_ALLOWED);
        CASE2STR(GATT_ERR_NOTIF_IND_NOT_CFG);
        CASE2STR(GATT_ERR_NOTIF_IND_CFG);
        CASE2STR(GATT_ERR_NOTIF_IND_CONF_PD);
        CASE2STR(GATT_ERR_TIMEOUT);
        CASE2STR(GATT_ERR_LINK_DEACTIVATED);
    default:
        pStr = cUnknownErrorCode;
        break;
    }

    return ( pStr );
}

/*----------------------------------------------------------------------------
 * error code to string conversion (XXXXMJMJ -> common lib ..)
 * --------------------------------------------------------------------------*/

const char * BQB_ErrorCode2Str( uint16_t wCause )
{
    const char  * pStr = NULL;

    if ( wCause == GATT_SUCCESS )
    {
        pStr = "SUCCESS";
    }
    else if ((wCause & 0xFF00) == GATT_ERR)
    {
        /* code from GATT/profile layer */
        pStr = BQB_GATTErrorCode2Str(wCause);
    }
    else if ((wCause & 0xFF00) == ATT_ERR)
    {
        /* code from ATT layer */
        pStr = BQB_ATTErrorCode2Str(wCause & 0x00FF);
    }
    else if ((wCause & 0xFF00) == L2CAP_ERR)
    {
        /* code from L2CAP layer */
        pStr = BQB_L2CAPErrorCode2Str(wCause & 0x00FF);
    }
    else if ((wCause & 0xFF00) == HCI_ERR)
    {
        /* code from HCI layer */
        pStr = BQB_HCIErrorCode2Str(wCause & 0x00FF);
    }
    else
    {
        pStr = cUnknownErrorCode;
    }

    return ( pStr );
}

/*----------------------------------------------------------------------------
 * hex dump of data
 * --------------------------------------------------------------------------*/

void BQB_HexDump( PGATTDemo pGATTDemo, char *pTxt, int iSize, uint8_t * pValue )
{
    int               i, iLength, iTotal;
    char            * pNext  = pGATTDemo->CmdIF.cBuffer;

    iLength = sprintf( pNext, "%s", pTxt );
    iTotal  = iLength;

    if ( (iSize > 0) && (iTotal < (MAX_BUFFER_LENGTH - iLength - 3 - 1)) )
    {
        pNext   += iLength;
        for ( i = 0; i < iSize; i++ )
        {
            iLength = sprintf( pNext, " %2.2x", *(pValue + i));
            iTotal += iLength;
            if ( iTotal < (MAX_BUFFER_LENGTH - iLength - 1) )
                pNext += iLength;
            else
                break;
        }
        BQB_UserIFSendString( pGATTDemo, pGATTDemo->CmdIF.cBuffer, NULL );
    }
    BQB_CmdPrint( pGATTDemo, "\r\n%s", pGATTDemo->CmdIF.cPrompt );
}

/*----------------------------------------------------------------------------
 * convert SFLOAT value to string and uint16_t value.
 * returns 0xFFFF as uint16_t value if the exponent is negative.
 * --------------------------------------------------------------------------*/

char * BQB_SFLOAT2StringAndWORD( uint16_t * pwSFLOATValue )
{
    static char cBuf[32];
    char        cSignM, cSignE;
    union
    {
        signed char c;
        uint8_t     b;
    }           exponent;
    union
    {
        int         i;
        uint32_t    d;
    }           mantissa;
    uint16_t  wValue = *pwSFLOATValue;

    exponent.b = (wValue & 0xF000) >> 12;
    if ( exponent.b & 0x08 )
        exponent.b |= 0xF0;
    if ( exponent.c < 0 )
    {
        /* print routine can't handle negative values .. */
        cSignE     = '-';
        exponent.c = -exponent.c;
    }
    else
    {
        cSignE = '+';
    }

    mantissa.d = (wValue & 0x0FFF);
    if ( mantissa.d & 0x0800 )
        mantissa.d |= 0xFFFFF000;
    if ( mantissa.i < 0 )
    {
        /* print routine can't handle negative values .. */
        cSignM     = '-';
        mantissa.i = -mantissa.i;
    }
    else
    {
        cSignM = '+';
    }

    sprintf( cBuf, "%c%d**(%c%d)", cSignM, mantissa.i, cSignE, exponent.c );

    /* return uint16_t value */
    if ( cSignM == '-' )
    {
        /* negative mantissa */
        *pwSFLOATValue = 0xFFFF;
    }
    else
    {
        int      i;
        uint16_t wSFLOATValue = mantissa.i;

        if ( cSignE == '-' )
        {
            /* negative exponent */
            for ( i = 0; i < exponent.c; i++ )
                wSFLOATValue = wSFLOATValue / 10;
        }
        else
        {
            for ( i = 0; i < exponent.c; i++ )
                wSFLOATValue = wSFLOATValue * 10;
        }
        *pwSFLOATValue = wSFLOATValue;
    }

    return ( cBuf );
}

/*----------------------------------------------------------------------------
 * print SFLOAT value
 * --------------------------------------------------------------------------*/

void BQB_PrintSFLOAT( PGATTDemo pGATTDemo, char *pName, uint16_t wValue )
{
    uint16_t  wSFLOATValue = wValue;

    BQB_CmdPrint( pGATTDemo, "%s = %s\r\n%s",
                  pName, BQB_SFLOAT2StringAndWORD(&wSFLOATValue),
                  pGATTDemo->CmdIF.cPrompt );
}

/*----------------------------------------------------------------------------
 * print time stamp value
 * --------------------------------------------------------------------------*/

void BQB_PrintTimestamp( PGATTDemo pGATTDemo, uint8_t * pTimestamp, char * pTxt )
{
    uint16_t wYYYY = LE_EXTRN2WORD(pTimestamp);

    BQB_CmdPrint( pGATTDemo, "  %s = %04d-%02d-%02d %02d:%02d:%02d\r\n%s",
                  pTxt, wYYYY, pTimestamp[2], pTimestamp[3],
                  pTimestamp[4], pTimestamp[5], pTimestamp[6],
                  pGATTDemo->CmdIF.cPrompt );
}

/*----------------------------------------------------------------------------
 * convert 16 bit UUID to string
 * --------------------------------------------------------------------------*/

char * BQB_UUID16ToString( uint16_t wUUID16 )
{
    switch ( wUUID16 )
    {
    default:
        return ("Unknown UUID");

    /*--- general GAP/GATT UUIDs ---*/
    case UUID_GAP:
        return ("Generic Access Profile");

    case UUID_GATT:
        return ("Generic Attribute Profile");
    case GATT_UUID_PRIMARY_SERVICE:
        return ("Primary Service");
    case GATT_UUID_SECONDARY_SERVICE:
        return ("Secondary Service");
    case GATT_UUID_INCLUDE:
        return ("Include");
    case GATT_UUID_CHARACTERISTIC:
        return ("Characteristic");

    /* GATT characteristic descriptors */
    case GATT_UUID_CHAR_EXTENDED_PROP:
        return ("Characteristic Extended Properties");
    case GATT_UUID_CHAR_USER_DESCR:
        return ("Characteristic User Description");
    case GATT_UUID_CHAR_CLIENT_CONFIG:
        return ("Client Characteristic Configuration");
    case GATT_UUID_CHAR_SERVER_CONFIG:
        return ("Server Characteristic Configuration");
    case GATT_UUID_CHAR_FORMAT:
        return ("Characteristic Format");
    case GATT_UUID_CHAR_AGGR_FORMAT:
        return ("Characteristic Aggregate Format");

    /* GATT characteristic types */
    case GATT_UUID_CHAR_DEVICE_NAME:
        return ("Device Name");
    case GATT_UUID_CHAR_APPEARANCE:
        return ("Appearance");
    case GATT_UUID_CHAR_PER_PRIV_FLAG:
        return ("Peripheral Privacy Flag");
    case GATT_UUID_CHAR_RECONN_ADDRESS:
        return ("Reconnection Address");
    case GATT_UUID_CHAR_PER_PREF_CONN_PARAM:
        return ("Peripheral Preferred Connection Parameters");
    case GATT_UUID_CHAR_SERVICE_CHANGED:
        return ("Service Changed");

    }
}


/*----------------------------------------------------------------------------
 * get UUID based on handle
 * --------------------------------------------------------------------------*/

uint16_t BQB_UUIDGet( PGATTDemo pGATTDemo, uint16_t idx, uint16_t wHandle)
{
    int       i;
    uint16_t  wUUID16 = 0;

    for ( i = 0; i < pGATTDemo->linkTable[idx].iHandleCnt; i++ )
    {
        if ( pGATTDemo->linkTable[idx].HandleUUID[i].wHandle == wHandle )
        {
            wUUID16 = pGATTDemo->linkTable[idx].HandleUUID[i].wUUID;
        }
    }

#if 0
    if ( wUUID16 == 0 )
    {
        BQB_CmdPrint( pGATTDemo, " : handle not found in local table !\r\n%s",
                      pGATTDemo->CmdIF.cPrompt );
    }
#endif

    return ( wUUID16 );
}





/*----------------------------------------------------------------------------
 * find service descriptor based on GATT server handle
 * --------------------------------------------------------------------------*/

PGATTDService BQB_ServiceFind( PGATTDemo pGATTDemo, void * pServiceHandle )
{
    int  i;

    for ( i = 0; i < BQB_GATT_SUPPORT_SERVICE_MAX; i++ )
    {
        if (pGATTDemo->Service[i].Used
                && (pGATTDemo->Service[i].pServiceHandle == pServiceHandle))
        {
            return (&pGATTDemo->Service[i]);
        }
    }
    return NULL;
}

/****************************************************************************/
/* convert Hex string to Binary                                             */
/****************************************************************************/

bool ASCIIHexToBin(uint8_t *pHex, uint8_t *pBin, uint16_t Length)
{
    int i;
    char c;
    char out = 0;

    if ( ! pHex || ! pBin )
    {
        return false;
    }


    while (Length)
    {
        for (i = 0; i < 2; i++) /* 2 digits only */
        {
            c = *pHex++;
            if ((c >= '0') && (c <= '9'))
                c -= '0';
            else if ((c >= 'A') && (c <= 'F'))
                c -= ('A' - 0x0A);
            else if ((c >= 'a') && (c <= 'f'))
                c -= ('a' - 0x0A);
            else
            {
                /*  invalid character */
                *pBin = '\0';
                return (false);
            }

            if ((i & 0x01) == 0)
                out = c << 4;
            else
                out |= c;
        }
        *pBin++ = out;
        Length--;
    }
    return (true);
}



#if (GATTDEMO_CCCD_COUNT) /* [ */
/*----------------------------------------------------------------------------
 * init. table of CCC descriptors
 * --------------------------------------------------------------------------*/

void BQB_CCCDInitTable( PGATTDemo pGATTDemo )
{
    int      i;
    uint16_t wAttribIndex[GATTDEMO_CCCD_COUNT] = GATTDEMO_CCCD_IDX;

    for ( i = 0; i < GATTDEMO_CCCD_COUNT; i++ )
    {
        pGATTDemo->AttrIdxCCCD[ i ].wAttribIndex = wAttribIndex[ i ];
        pGATTDemo->AttrIdxCCCD[ i ].wCCCBits     = 0;
    }
}

/*----------------------------------------------------------------------------
 * check if CCC descriptors are properly set
 * --------------------------------------------------------------------------*/

uint16_t BQB_CCCDCheckValue( PGATTDemo pGATTDemo, int iStart, int iEnd )
{
    int      i;
    uint16_t wCause = GATT_SUCCESS;

    if ( (iStart < 0) || (iEnd < 0) )
    {
        /* check all CCCDs */
        iStart = 0;
        iEnd   = GATTDEMO_CCCD_COUNT - 1;
    } /* else range only */

    /* XXXXMJMJ rudimentary check only ... : */
    for ( i = iStart; i <= iEnd; i++ )
    {
        if ( pGATTDemo->AttrIdxCCCD[ i ].wCCCBits == 0 )
        {

            wCause = ATT_ERR | GLC_ERR_CCCD_IMPROPERLY_CONFIGURED;

            break;
        }
    }

    return ( wCause );
}
#endif /* ] (GATTDEMO_CCCD_COUNT) */

/*----------------------------------------------------------------------------
 * display attribute value
 * --------------------------------------------------------------------------*/
void  BQB_AttribDisplay(PGATTDemo pGATTDemo, uint16_t idx, uint16_t wHandle, int iSize, uint8_t * pValue)
{
    uint16_t  wUUID16;
    BQB_CmdPrint( pGATTDemo, " handle=0x%x, size=%d", wHandle, iSize );

    wUUID16 = BQB_UUIDGet( pGATTDemo, idx, wHandle );
    switch ( wUUID16 )
    {
    default:
        BQB_HexDump( pGATTDemo, " : value=", iSize, pValue );
        break;
    }
}

