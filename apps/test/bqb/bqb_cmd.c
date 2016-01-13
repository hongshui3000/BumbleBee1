enum { __FILE_NUM__= 0 };

/**********************************************************************!KA****
 *
 * $Header: /var/lib/cvs/sw/src/app/demo/gattdemo/gattdcmd.c,v 1.56.2.2 2013/12/06 12:54:12 mn Exp $
 *
 * File:        $RCSfile: gattdcmd.c,v $
 * Version:     $Name: P_SRP1290_V1_0 $
 *
 * Archive:     $Source: /var/lib/cvs/sw/src/app/demo/gattdemo/gattdcmd.c,v $
 * Revision:    $Revision: 1.56.2.2 $
 * Date:        $Date: 2013/12/06 12:54:12 $
 * Author:      $Author: mn $
 *
 * ---------------------------------------------------------------------------
 * !MODULE      [  ]
 * ---------------------------------------------------------------------------
 * !FILE        [  ]
 * !PROGRAM     [  ]
 * !VERSION     [$Name: P_SRP1290_V1_0 $]
 * !AUTHOR      [$Author: mn $]
 * !GROUP       [  ]
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
 *         GATTDEMO (BlueAPI/GATT API demo) command handling
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

/*-- non-BlueAPI/GATT related command handlers, others are in --*/
/*-- gatdbapi.c and gatdbapisec.c                             --*/
static TGATTDemoResult BQB_CmdHelp(PGATTDemo pGATTDemo,
                                   TGATTDemoParseResult *pParseResult);
static TGATTDemoResult BQB_CmdRemoteBdSet(PGATTDemo pGATTDemo,
        TGATTDemoParseResult *pParseResult);
static TGATTDemoResult BQB_CmdRemoteBdList(PGATTDemo pGATTDemo,
        TGATTDemoParseResult *pParseResult);
static TGATTDemoResult BQB_CmdRemoteBdRemove(PGATTDemo pGATTDemo,
        TGATTDemoParseResult *pParseResult);

static TGATTDemoResult BQB_CmdShortOutput(PGATTDemo pGATTDemo,
        TGATTDemoParseResult *pParseResult);

static TGATTDemoResult BQB_CmdHandleUUIDDisplay(PGATTDemo pGATTDemo,
        TGATTDemoParseResult *pParseResult);


const char cGATTDRoleServerTxt[] = "Server";
const char cGATTDRoleClientTxt[] = "Client";

extern int _strcmp(const char *cs, const char *ct);

/*----------------------------------------------------
 * command table
 * --------------------------------------------------*/

const TGATTDemoCmdTableEntry BQB_CmdTable[] =
{
    {
        "help",
        "help\n\r",
        "Show help text\n\r",
        BQB_CmdHelp
    },

    /* BlueAPI/GATT related commands */
    {
        "reg",
        "reg\n\r",
        "Register to BlueAPI\n\r",

        BQB_Register
    },
    
    {
        "rel",
        "rel\n\r",
        "Release from BlueAPI\n\r",

        BQB_Release
    },

    /*  
        [index of BD]   remote bluetooth device address
            exp: 0x00 0x1b 0xdc 0x0 0xb5 0x93  */
    {
        "rembd",
        "rembd [index of BD]\n\r",
        "Set remote BD (hard coded or based on inquiry result)\n\r",
        BQB_CmdRemoteBdSet
    },
    
    {
        "bdlist",
        "bdlist \n\r",
        "Show register BD list\n\r",
        BQB_CmdRemoteBdList
    },
    
    {
        "bdrm",
        "bdrm <index>\n\r",
        "Remove BD from the list\n\r",
        BQB_CmdRemoteBdRemove
    },
    
    {
        "devcfg",
        "devcfg <opCode> <...>\n\r",
        "device config\r\n",
        BQB_CmdDevCfg
    },


#if (F_BT_LE_PRIVACY_MODE)
    {
        "privacy",
        "privacy <mode>\n\r",
        "Control LE Privacy Mode\n\r",
        BQB_CmdPrivacyMode
    },
#endif /* (F_BT_LE_PRIVACY_MODE) */

    /*  
        [index]             link index
        [requirements]      0x01--force MITM, 0x10--force pairing
        [keysize]           if 0, 16bit keysize */
    {
        "sauth",
        "sauth <index> <requirements> <keysize>\n\r",
        "Secman authentication request\n\r",
        BQB_GATTSecurityReq
    },

    /*  
        [pair]      0--disable pair, 1--enable pair.
        [authreq]  0--NoMITMRequiredNoStore, 1--MITMRequiredNoStore, 2-- NoMITMRequiredBonding, 3--MITMRequiredBonding
        [iocap]     0-- DisplayOnly, 1--DisplayYesNo, 2--KeyboardOnly, 3--NoIO, 4--KeyboardDisplay
        [oob]       0-- remote oob not present, 1--remote oob present           */
    {
        "authmode",
        "authmode <pair> <authreq> <iocap> <oob>\n\r",
        "Set pairable mode\n\r",
        BQB_CmdPairableMode
    },

    /*  
        [index]   link index.
        [6digits]  6 digits pin code  */
    {
        "authkeyb",
        "authkeyb <index>  [6digits]\n\r",
        "SSP passkey entry\n\r",
        BQB_CmdAuthKeyboard
    },

    {
        "authsetoob",
        "authsetoob [C]\n\r",
        "SSP set remote OOB data\n\r",
        BQB_CmdAuthSetOOB
    },

    {
        "srvreg",
        "srvreg [gatt/hdp/spp] [changed]\n\r",
        "Register service(s)\n\r",
        BQB_ServiceRegister
    },
    
    {
        "srvrel",
        "srvrel\n\r",
        "Release service(s)\n\r",
        BQB_ServiceRelease
    },

    /*
        [index]   characteristic value index in service.
        [count]  repetion count, default 1  */
    {
        "update",
        "update [index [count]]\n\r",
        "Attribute value update\n\r",
        BQB_AttribUpdate
    },

    /* [enable] 1--enable advertising, 0--disable advertising */
    {
        "ade",
        "ade [enable]\n\r",
        "enable/disable advertising\n\r",
        BQB_CmdSetAdvertisingEnable
    },

    {
        "adp",
        "adp <index> <adv-type>\n\r",
        "set advertising parameters\n\r",
        BQB_CmdSetAdvertisingParameters
    },

    {
        "addirect",
        "addirect <index> \n\r",
        "directed advertising to remote BD\n\r",
        BQB_CmdSetDirectedAdvertising
    },

    /*
         [adv_data_arry_index] 0-- bBQBAdData0, 1-- bBQBAdData1, 2-- bBQBAdData2 */
    {
        "addata",
        "addata <adv_data_arry_index> \n\r",
        "Set advertising data\n\r",
        BQB_SetAdvertisingData
    },

    /* select adv discovery mode[0--2] */
    {
        "dismode",
        "dismode [mode]\n\r",
        "set advertising discovering mode 0--Non-Discoverable 1--Limitted-Discoverable[], 2--General-Discoverable\n\r",
        BQB_SetDiscoveryMode
    },

    /* 
        [index] link index
       [type] 0--LE_pub (default), 1--LE_rand, 2:BR/EDR
       [whitelist] 0 -- remote bd addr 1--remote bd from white list */
    {
        "con",
        "con <index> [bdtype] [whitelist]\n\r",
        "Connect\n\r",
        BQB_ConnectReq
    },

    /* [index] link index */
    {
        "disc",
        "disc <index> \n\r",
        "Disconnect\n\r",
        BQB_DisconnectReq
    },

    /* 
        [index]         link index
        [start]         start handle
       [end]        end hande
       [uuid_type]  0 --16bit 1--128bit
       [UUID16_128]  16bit uuid or 128bit uuid
         exp: 0x1800(16bit uuid)
            0xa4a50000 0x00000000 0x01234567 0x89abcdef(128bit uuid) */
    {
        "srvdis",
        "srvdis <index> [start] [end] [uuid_type] [UUID16_128]\n\r",
        "Service discovery\n\r",
        BQB_ServiceDiscovery
    },

    /* 
        [index] link index
       [start] start handle
       [end]   end hande    */
    {
        "reldis",
        "reldis <index> [start] [end]\n\r",
        "Relationship discovery\n\r",
        BQB_RelationshipDiscovery
    },

    /*
         [index] link index
         [start] start handle
         [end]   end hande
         [uuid_type] 0 --16bit 1--128bit
         [UUID16_128] 16-bit uuid or 128bit uuid */
    {
        "chardis",
        "chardis <index> [start] [end] [uuid_type] [UUID16_128]\n\r",
        "Characteristic discovery\n\r",
        BQB_CharacteristicDiscovery
    },

    /*
         [index] link index
         [start] start handle
         [end]   end hande
         [uuid_type] 0 --16bit 1--128bit
         [UUID16_128] 16-bit uuid or 128bit uuid */
    {
        "charddis",
        "charddis <index> [start] [end] [uuid_type] [UUID16_128]\n\r",
        "Characteristic descriptor discovery\n\r",
        BQB_CharacDescriptorDiscovery
    },

    /*
         [index] link index
         [start] start handle
         [end]   end hande       */
    {
        "lsuuid",
        "lsuuid <index> [start] [end]\n\r",
        "List of UUIDs\n\r",
        BQB_ListUUIDs
    },

     /*
         [index]  link index
         [handle] attribute handle
         [offset] read offset        */
    {
        "read",
        "read <index> [handle] [offset]\n\r",
        "Read attribute\n\r",
        BQB_AttribRead
    },
#if 0
    /*
         [index]  link index
         [handle1] attribute handle1
         ...... 
         [handle4] attribute handle2    */
    {
        "mread",
        "mread <index> [handle1] [handle2].....\n\r",
        "multiple read, max 4 handles \n\r",
        BQB_AttribReadMulti
    },
#endif
   /*
         [index] link index
         [start] start handle
         [end]   end hande
         [uuid_type] 0 --16bit 1--128bit
         [UUID16_128] 16-bit uuid or 128bit uuid */
    {
        "readu",
        "readu <index> [start] [end] [uuid-type] [UUID16_128]\n\r",
        "Read attribute using UUID\n\r",
        BQB_AttribReadUUID
    },

     /*
         [index]     link index
         [handle]  attribute handle
         [type]    1--write command 0--write request
         [len]       the length of attribute value to write */
    {
        "write",
        "write <index> [handle] [type] [len]\n\r",
        "Gatt attribute write\n\r",
        BQB_AttribWrite
    },

    /*
         [index]     link index
         [handle]  attribute handle
         [type]    1--write command 0--write request
         [len]       the length of attribute value to write
         [var1]  the first byte
         [var2]  the second byte 
         ......                                                 */
    {
        "wr",
        "write <index> [handle] [type] [len] [val1] [val2]...... \n\r",
        "Write attribute\n\r",
         BQB_AttribWriteEx
    },

#ifdef BQB_PREPARE_WRITE_SUPPPRT
    /*
         [index]    link index
         [handle]  attribute handle
         [offset]  prepare/reliable write offset
         [len]      the length of attribute value to write */

    {
        "pwrite",
        "pwrite <index> [handle] [offset][len]\n\r",
        "Gatt prepare write\n\r",
        BQB_AttribPrepareWrite
    },

    /*
         [index] link index
         [flag]  1--do execute write, 0--cancel execute write */
    {
        "ewrite",
        "ewrite <index> [flag]\n\r",
        "Gatt execute write\n\r",
        BQB_AttribExecuteWrite
    },
#endif

    /* [db_index] 0--small database 1--large database */
    {
        "seldb",
        "seldb [db_index]\n\r",
        "Select bqb database\n\r",
        BQB_SelectDB
    },

    /* auto change authmode from No-MITM to MITM
        [enable] 0--disable, 1--enable 
            TC_SEC_AUT_BV_14_C needed 
      note: autochgauthmode 1 firstly and autochgauthmode 0 after testing releated case!!!!!! */
    {
        "autochgauthmode",
        "autochgauthmode [enable]\n\r",
        "enable authmode auto changed \n\r",
        BQB_AutoChangeAuthMode
    },

     {
        "chgmtusize",
        "chgmtusize [mtusize]\n\r",
        "change gatt mtu size \n\r",
        BQB_ChangeMtuSize
    },

    /* [adv_data_index] 0--bBQBAdData[] 1--bBQBAdData1[]£¬ 2--bBQBAdData2[]*/
    {
        "seladvdata",
        "seladvdata [adv_data_index]\n\r",
        "Select advertising data\n\r",
        BQB_SelectAdvDataIndex
    },

    /* [index] link index */
    {
        "lshndl",
        "lshndl <index>\n\r",
        "List of locally stored handle/UUID pairs\n\r",
        BQB_CmdHandleUUIDDisplay
    },

    /*
         [enable]       1--enable scan, 0--disable scan
         [active]       0--passive scna, 1--active scan
         [dups]         0--do not filter duplicated advertising events, 1--Filter duplicated advertising events
         [whitelist]    0--do not use white list to fiter, 1-- use white list to fiter bluetooth device */
    {
        "scan",
        "scan [enable] [active] [dups] [whitelist]\n\r",
        "enable LE scan\n\r",
        BQB_CmdScanEnable
    },

    /*
         [index]        link index
         [accept]       0--reject connection parameter update, 1--accept connection parameter update */
    {
        "conupdresp",
        "conupdresp <index> [accept]\n\r",
        "LE connection update\n\r",
        BQB_ConUpdateResp
    },

    /*
         [index]        link index */
    {
        "addwl",
        "addwl <index>\n\r",
        "add remote BD to whitelist\n\r",
        BQB_CmdAddToWhitelist
    },

     /*
         [index]        link index */
    {
        "remwl",
        "remwl index>\n\r",
        "remove remote BD from whitelist\n\r",
        BQB_CmdRemoveFromWhitelist
    },
    
    {
        "clearwl",
        "clearwl\n\r",
        "clear whitelist\n\r",
        BQB_CmdClearWhitelist
    },

    /* miscellaneous */
    {
        "short",
        "short [value]\n\r",
        "Short output: 0-off, 1-on\n\r",
        BQB_CmdShortOutput
    },

    /*
         [index]            link index
         [notify]           0--indication, 1--notification
         [service_index]    service index
         [atrr_index]       attribute index */
    {
        "notind",
        "notify/indication <index> [notify--1] [service_index] [atrr_index]\n\r",
        "notify/indication bqb server send indication/notification\n\r",
        BQB_SendNotInd
    },

    /*
         [index]            link index
         [minInt]           minimum connection interval
         [maxInt]           maximum connection interval
         [latency]          connection latency
         [supervision]      supervision timeout             */
    {
        "conupdreq",
        "conupdreq <index> <minInt> <maxInt> <latency> <supervision>\n\r",
        "LE connection update\n\r",
        BQB_ConUpdateReq
    },


    {
        "conle",
        "conle [id]\n\r",
        "conle: connect LE data channel\n\r",
        BQB_CmdConLEChan
    },

    {
        "discle",
        "discle [id] [channel]\n\r",
        "discle: disconnect LE data channel\n\r",
        BQB_CmdDiscLEChan
    },
    {
        "credit",
        "credit [id] [channel] [credits]\n\r",
        "credit: send LE Flow Control Credit\n\r",
        BQB_CmdCredit
    },
    {
        "ledata",
        "ledata [id] [channel] \n\r",
        "ledata: send LE Data\n\r",
        BQB_CmdLEData
    },
    {
        "leSec",
        "leSec [le_psm] [active] [mode] [keysize] \n\r",
        "leSec: \n\r",
        BQB_CmdLESec
    },
    {
        "scbqb",
        "scbqb [test case]\n\r",
        "Sample: scbqb 0 or latency 100 \n\r",
        BQB_CmdScBqb
    },


    /* MUST be at the end: */
    {
        0,
        0,
        0,
        0
    }
};

/****************************************************************************/
/* Show help                                                                */
/****************************************************************************/

static void BQB_CmdHelpShow( PGATTDemo pGATTDemo )
{
}

/*----------------------------------------------------
 * clear command line buffer
 * --------------------------------------------------*/

static void BQB_CmdClearCommand( PGATTDemo pGATTDemo )
{
    pGATTDemo->CmdIF.iCommandLength = 0;
    memset( pGATTDemo->CmdIF.cCommandLine, 0,
            sizeof(pGATTDemo->CmdIF.cCommandLine) );
}

/*----------------------------------------------------
 * command help
 * --------------------------------------------------*/

static TGATTDemoResult BQB_CmdHelp( PGATTDemo pGATTDemo,
                                    TGATTDemoParseResult *pParseResult )
{
    BQB_CmdHelpShow(pGATTDemo);
    return ( gattdResultOk );
}

/*----------------------------------------------------------------------------
 * command rembd, access to BDs in:
 * - list of hard coded BDs  (reverse order !!!)
 * - list of inquiry results
 * ---------------------------------------------------------------------------*/

#define TEST_BD_LIST_IDX   0xFFFF    /* list all hard coded BDs  */
#define TEST_BD_START_IDX  0x1000    /* offset to hard coded BDs */
TGATTDBdAddr gattdTestRemoteBd[] = /* first entry is default BD: */
{
    { 0x87, 0x99, 0x23, 0x4C, 0xe0, 0x00 },    /* PTS LE Dongle 2 */
    { 0xF4, 0xBD, 0xF8, 0xA5, 0x0D, 0xBC },    /* LE patch board 2 / SN 0174 */
    { 0x4D, 0xD9, 0xF8, 0xA5, 0x0D, 0xBC },    /* LE patch board 1 / SN 0176 */
    { 0xFA, 0xC4, 0xF8, 0xA5, 0x0D, 0xBC },    /* LE patch board 3 / (ore1) */
    { 0x4D, 0xD1, 0xF8, 0xA5, 0x0D, 0xBC },    /* LE patch board 4 / (ore2) */
    { 0x1D, 0xB6, 0x05, 0xDC, 0x1B, 0x00 },    /* PTS LE Dongle 1 */
    { 0x00, 0x00, 0x00, 0xEE, 0xFF, 0xC0 },    /* TI USB dongle */
    { 0x01, 0x00, 0x00, 0xEE, 0xFF, 0xC0 },    /* TI keyfob     */
    { 0x76, 0x9C, 0x14, 0x25, 0x80, 0x00 },    /* BR/EDR P25/G2 1 */
    { 0x55, 0x9C, 0x14, 0x25, 0x80, 0x00 },    /* BR/EDR P25/G2 2 */
    { 0x5C, 0x9C, 0x14, 0x25, 0x80, 0x00 },    /* BR/EDR P25/G2 3 */
    { 0xCE, 0x9C, 0x14, 0x25, 0x80, 0x00 }     /* BR/EDR P25/G2 4 */

};

static TGATTDemoResult BQB_CmdRemoteBdSet( PGATTDemo pGATTDemo,
        TGATTDemoParseResult *pParseResult )
{
    int   i, iCnt;
    TGATTDemoResult Result = gattdResultOk;
    PGATTLink pLink = NULL;

    if ( pParseResult->iParameterCount > 0 )
    {
#if (TEST_BD_START_IDX)
        iCnt = sizeof(gattdTestRemoteBd) / BLUE_API_BD_SIZE;

        if ( pParseResult->dwParameter[0] == TEST_BD_LIST_IDX )
        {
            /* list all hard coded BDs (output without flow control, */
            /* partial loss possible ...):                           */

            for ( i = 0; i < iCnt; i++ )
            {
                BQB_CmdPrint( pGATTDemo,
                              "RemoteBd[%d]=[%02x:%02x:%02x:%02x:%02x:%02x]\r\n", i,
                              gattdTestRemoteBd[i][5], gattdTestRemoteBd[i][4],
                              gattdTestRemoteBd[i][3], gattdTestRemoteBd[i][2],
                              gattdTestRemoteBd[i][1], gattdTestRemoteBd[i][0] );
            }
            BQB_CmdPrint( pGATTDemo,
                          "Use \"rembd [0x%02x+index]\" to set BD !\r\n", TEST_BD_START_IDX );
            return ( Result );
        }
        else if ( (pParseResult->dwParameter[0] >= TEST_BD_START_IDX) &&
                  (pParseResult->dwParameter[0] < (TEST_BD_START_IDX + iCnt))
                )
        {
            /* use hard coded BD */
            pParseResult->dwParameter[0] -= TEST_BD_START_IDX;
            pLink = BQB_LinkAllocate(pGATTDemo, gattdTestRemoteBd[pParseResult->dwParameter[0]]);
        }
        else
#endif
        {
            if ( pParseResult->iParameterCount >= 6 )
            {
                TGATTDBdAddr     RemoteBd;
                /* yes, reverse input ... */
                RemoteBd[0] = pParseResult->dwParameter[5];
                RemoteBd[1] = pParseResult->dwParameter[4];
                RemoteBd[2] = pParseResult->dwParameter[3];
                RemoteBd[3] = pParseResult->dwParameter[2];
                RemoteBd[4] = pParseResult->dwParameter[1];
                RemoteBd[5] = pParseResult->dwParameter[0];
                pLink = BQB_LinkAllocate(pGATTDemo, RemoteBd);
            }
            else
            {
                BQB_CmdPrint( pGATTDemo, "No inquiry performed !\r\n" );
            }
        }
    }

    if ( pLink )
    {
        BQB_CmdPrint( pGATTDemo,
                      "RemoteBd=[%02x:%02x:%02x:%02x:%02x:%02x] idx=%d\r\n",
                      pLink->RemoteBd[5], pLink->RemoteBd[4],
                      pLink->RemoteBd[3], pLink->RemoteBd[2],
                      pLink->RemoteBd[1], pLink->RemoteBd[0], pLink->idx);
    }
    else
    {
        BQB_CmdPrint( pGATTDemo,
                      "Set bd failed\r\n");
    }

    return ( Result );
}

static TGATTDemoResult BQB_CmdRemoteBdList( PGATTDemo pGATTDemo,
        TGATTDemoParseResult *pParseResult )
{
    int   i;
    PGATTLink pLink = NULL;
    for (i = 0; i < BQB_MAX_LINKS; i++)
    {
        pLink = &pGATTDemo->linkTable[i];
        if (pLink->isUsed)
        {
            BQB_CmdPrint( pGATTDemo,
                          "idx=%d RemoteBd=[%02x:%02x:%02x:%02x:%02x:%02x] connect = %d role=%d\r\n",
                          pLink->idx, pLink->RemoteBd[5], pLink->RemoteBd[4],
                          pLink->RemoteBd[3], pLink->RemoteBd[2],
                          pLink->RemoteBd[1], pLink->RemoteBd[0], pLink->Connected, pLink->role);
        }
    }

    return ( gattdResultOk );
}

static TGATTDemoResult BQB_CmdRemoteBdRemove( PGATTDemo pGATTDemo,
        TGATTDemoParseResult *pParseResult )
{
    PGATTLink pLink = NULL;
    int idx ;
    if (pParseResult->iParameterCount == 1)
    {
        idx = pParseResult->dwParameter[0];
    }
    else
    {
        return gattdResultError;
    }

    if (idx > (BQB_MAX_LINKS - 1))
    {
        return (gattdResultError);
    }

    pLink = &pGATTDemo->linkTable[idx];
    BQB_LinkRelease(pGATTDemo, pLink);
    BQB_CmdPrint( pGATTDemo, "remove idx=%d \r\n", idx);

    return ( gattdResultOk );
}


/*----------------------------------------------------------------------------
 * command lshndl
 * --------------------------------------------------------------------------*/

static TGATTDemoResult BQB_CmdHandleUUIDDisplay( PGATTDemo pGATTDemo,
        TGATTDemoParseResult *pParseResult )
{
    int   i;
    int idx = pParseResult->dwParameter[0];

    if (idx > (BQB_MAX_LINKS - 1))
    {
        return (gattdResultError);
    }

    if ( pGATTDemo->linkTable[idx].iHandleCnt <= 0 )
    {
        BQB_CmdPrint( pGATTDemo, "List is empty!\r\n" );
    }
    else
    {
        for ( i = 0; i < pGATTDemo->linkTable[idx].iHandleCnt; i++ )
        {
            BQB_CmdPrint( pGATTDemo, "handle=0x%04x, UUID=<0x%04x> (%s)\r\n",
                          pGATTDemo->linkTable[idx].HandleUUID[i].wHandle,
                          pGATTDemo->linkTable[idx].HandleUUID[i].wUUID,
                          BQB_UUID16ToString( pGATTDemo->linkTable[idx].HandleUUID[i].wUUID ) );
        }
    }

    return ( gattdResultOk );
}




/*----------------------------------------------------------------------------
 * command short
 *---------------------------------------------------------------------------*/

static TGATTDemoResult BQB_CmdShortOutput(PGATTDemo pGATTDemo,
        TGATTDemoParseResult *pParseResult)
{
    if ( pParseResult->iParameterCount == 0 )
        pGATTDemo->ShortOutput = true;
    else
        pGATTDemo->ShortOutput = pParseResult->dwParameter[0] ? true : false;

    return ( gattdResultOk );
}

/****************************************************************************/
/* Send result                                                              */
/****************************************************************************/

static void BQB_CmdSendResult(PGATTDemo pGATTDemo, TGATTDemoResult Result)
{
    switch (Result)
    {
    case gattdResultError:
        BQB_UserIFSendString(pGATTDemo, "Error\n\r", NULL);
        break;
    case gattdResultCommandNotFound:
        BQB_UserIFSendString(pGATTDemo, "Command not found\n\r", NULL);
        break;
    case gattdResultWrongNumberOfParameters:
        BQB_UserIFSendString(pGATTDemo, "Wrong number of parameters\n\r", NULL);
        break;
    case gattdResultWrongParameter:
        BQB_UserIFSendString(pGATTDemo, "Wrong parameter\n\r", NULL);
        break;
    case gattdResultValueOutOfRange:
        BQB_UserIFSendString(pGATTDemo, "Value out of range\n\r", NULL);
        break;
    default:
        break;
    }
}

/*----------------------------------------------------------------------------
 * flush buffer with echo characters
 * -------------------------------------------------------------------------*/

static void BQB_CmdFlushEchoChars( PGATTDemo pGATTDemo, PGATTDemoCmdIF pCmdIF )
{
    if ( pCmdIF->iTxLength > 0 )
    {
        BQB_UserIFSendChar(pGATTDemo, 0);     /* flush buffer with echo data */
        pCmdIF->iTxLength   = 0;
        pCmdIF->pTxBuffer   = NULL;
    }
}

/****************************************************************************/
/* Check, if a character is a white space                                   */
/****************************************************************************/

static bool BQB_CmdIsWhiteSpace(char c)
{
    return (((c >= 9) && (c <= 13)) || (c == 32));
}


/****************************************************************************/
/* Skip white spaces in buffer                                              */
/****************************************************************************/

static char *BQB_CmdSkipSpaces(char *buffer)
{
    char *p = buffer;

    while (BQB_CmdIsWhiteSpace(*p)) /* white space */
    {
        p++;
    }
    return p;
}


/****************************************************************************/
/* Find end of a word                                                       */
/****************************************************************************/

static char *BQB_CmdFindEndOfWord(char *buffer)
{
    char *p = buffer;

    while (!BQB_CmdIsWhiteSpace(*p) && (*p != '\0'))
    {
        p++;
    }
    return p;
}


/****************************************************************************/
/* Read ASCII string and convert to uint32_t                                   */
/****************************************************************************/

static uint32_t BQB_CmdString2uint32_t(char *p)
{
    uint32_t result = 0;
    char     ch;
    bool     hex = false;

    /* check if value is dec */
    if (p[0] == 'x')
    {
        hex = true;
        p = &p[1];
    }
    else if ((p[0] == '0') && (p[1] == 'x'))
    {
        hex = true;
        p = &p[2];
    }

    for (;;)
    {
        ch = *( p++ ) | 0x20;               /* convert to lower case */

        if ( hex)                           /* dec value */
        {
            /* hex value */
            if ((ch >= 'a') && (ch <= 'f'))
            {
                ch -= ('a' - 10);
            }
            else if ((ch >= '0') && (ch <= '9'))
            {
                ch -= '0';
            }
            else
            {
                break;
            }
            result =  (result << 4);
            result += (ch & 0x0f);
        }
        else
        {
            if ( ch < '0' || ch > '9' )
                break;  /* end of string reached */
            result = 10 * result + ch - '0';
        }
    }
    return (result);
}

/*----------------------------------------------------------------------------
 * execute cmd.
 * -------------------------------------------------------------------------*/

static TGATTDemoResult BQB_CmdExecute( PGATTDemo pGATTDemo,
                                       TGATTDemoParseResult * pParseResult )
{
    int              i = 0;
    TGATTDemoResult  Result = gattdResultCommandNotFound;

    /* find command in table */
    while (BQB_CmdTable[i].pCommand != NULL)
    {
        if ( strcmp((const char *)BQB_CmdTable[i].pCommand,
                     (const char *)pParseResult->pCommand) == 0 )
        {
            /* command found */

            /* check if user wants help */
            if (pParseResult->iParameterCount && *pParseResult->pParameter[0] == '?')
            {
                BQB_UserIFSendString(pGATTDemo, BQB_CmdTable[i].pOption, NULL);
                Result = gattdResultOk;
            }
            else
            {
                /* execute command function */
                Result = BQB_CmdTable[i].Func(pGATTDemo, pParseResult);
            }
            /* exit while */
            break;
        }
        i++;
    };

    return ( Result );
}

/*----------------------------------------------------------------------------
 * Parse a command line and return the found
 * command and parameters in "pParseResult"
 * -------------------------------------------------------------------------*/

static TGATTDemoResult BQB_CmdParse( PGATTDemo pGATTDemo,
                                     TGATTDemoParseResult * pParseResult )
{
    int              i;
    TGATTDemoResult  Result;
    PGATTDemoCmdIF   pCmdIF;
    char            *p, *q;

    pCmdIF  = &pGATTDemo->CmdIF;

    /* clear all results */
    Result = gattdResultOk;
    pParseResult->pCommand            = NULL;
    pParseResult->pRestOfCommandLine  = NULL;
    pParseResult->iParameterCount     = 0;
    for (i = 0 ; i < MAX_PARAMETERS; i++)
    {
        pParseResult->pParameter[i]     = NULL;
        pParseResult->dwParameter[i]    = 0;
    }

    /* Parse line */
    p = pCmdIF->cCommandLine;

    /*ignore leading spaces */
    p = BQB_CmdSkipSpaces(p);
    if ( *p == '\0')                      /* empty command line ? */
    {
        Result = gattdResultEmptyCommandLine;
    }
    else
    {
        /* find end of word */
        q = BQB_CmdFindEndOfWord( p);
        if (p == q)                        /* empty command line ? */
        {
            Result = gattdResultEmptyCommandLine;
        }
        else                                /* command found */
        {
            pParseResult->pCommand = p;
            *q = '\0';                        /* mark end of command */
            p = q + 1;

            /* parse parameters */
            if ( *p != '\0')                  /* end of line ? */
            {
                int j;

                pParseResult->pRestOfCommandLine = p;

                j = 0;
                do
                {
                    uint32_t d;
                    /* ignore leading spaces */
                    p = BQB_CmdSkipSpaces(p);
                    d = BQB_CmdString2uint32_t(p);

                    pParseResult->pParameter[j]    = p;
                    pParseResult->dwParameter[j++] = d;

                    if ( j >= MAX_PARAMETERS)
                    {
                        break;
                    }

                    /* find next parameter */
                    p  = BQB_CmdFindEndOfWord( p);
                    *p++ = '\0';                        /* mark end of parameter */
                }
                while (*p != '\0');
                pParseResult->iParameterCount = j;
            }
        }
    }

    return ( Result );
}

/*----------------------------------------------------------------------------
 * collect cmd characters.
 * returns:
 * - true:  caller may release data buffer,
 * - false: caller may NOT release data buffer (data was forwarded ..)
 * -------------------------------------------------------------------------*/

bool BQB_CmdCollect( PGATTDemo pGATTDemo, char * pData, int iLength )
{
    TGATTDemoParseResult ParseResult;
    PGATTDemoCmdIF       pCmdIF;
    char                 c;

    pCmdIF              = &pGATTDemo->CmdIF;
    pCmdIF->iTxLength   = -iLength;       /* < 0: tx buffer must be allocated */
    pCmdIF->pTxBuffer   = NULL;

    /* discard rx data as long help text output is in progress */
    while (!pCmdIF->HelpTextInProgress && iLength--)
    {
        c = *pData++;
        if (c != 0x0)                       /* not ESC character received */
        {
            switch (c)                        /* Normal handling */
            {
            case '\n':
            case '\r':                      /* end of line */
                BQB_CmdFlushEchoChars( pGATTDemo, pCmdIF );
                pCmdIF->SendPrompt = true;

                BQB_UserIFSendString(pGATTDemo, pCmdIF->cCrLf, NULL);
                if (pCmdIF->iCommandLength > 0)  /* at least one character in command line ? */
                {
                    TGATTDemoResult Result;

                    pCmdIF->cCommandLine[pCmdIF->iCommandLength] = '\0';
                    Result = BQB_CmdParse(pGATTDemo, &ParseResult);
                    if (Result == gattdResultOk)
                        Result = BQB_CmdExecute(pGATTDemo, &ParseResult);

                    if (Result != gattdResultOk)
                        BQB_CmdSendResult(pGATTDemo, Result);
                }

                if (pCmdIF->SendPrompt)
                    BQB_UserIFSendString(pGATTDemo, pCmdIF->cPrompt, NULL);

                BQB_CmdClearCommand( pGATTDemo );

                /* maybe more than 1 cmd in pData: */
                pCmdIF->iTxLength   = -iLength;
                pCmdIF->pTxBuffer   = NULL;
                break;
            case '\b':                        /* backspace */
                if (pCmdIF->iCommandLength > 0)
                {
                    pCmdIF->iCommandLength--;
                    pCmdIF->cCommandLine[pCmdIF->iCommandLength] = '\0';
                }
                if ( pCmdIF->iTxLength < 0 )
                    pCmdIF->iTxLength -= 2;     /* OK only if BS is first char ... */
                BQB_UserIFSendChar(pGATTDemo, c);
                BQB_UserIFSendChar(pGATTDemo, ' ');
                BQB_UserIFSendChar(pGATTDemo, c);
                break;
            default:
                /* Put character in command buffer */
                if (pCmdIF->iCommandLength < MAX_COMMAND_LINE)
                {
                    BQB_UserIFSendChar(pGATTDemo, c);
                    pCmdIF->cCommandLine[pCmdIF->iCommandLength++] = c;
                }
                break;
            }
        }
    }

    BQB_CmdFlushEchoChars( pGATTDemo, pCmdIF );

    return ( true );
}

/*----------------------------------------------------------------------------
 * init. cmd set
 * -------------------------------------------------------------------------*/

void BQB_CmdInit( PGATTDemo pGATTDemo )
{
    uint8_t i = 0;
    memset( &pGATTDemo->CmdIF, 0, sizeof(TGATTDemoCmdIF) );
    pGATTDemo->CmdIF.cCrLf[0]   = '\r';
    pGATTDemo->CmdIF.cCrLf[1]   = '\n';
    pGATTDemo->CmdIF.cPrompt[0] = '>';

    /* setup default remote BD */
    for (i = 0; i < BQB_MAX_LINKS; i++)
    {
        memset( &pGATTDemo->linkTable[i], 0, sizeof(TGATTLink));
    }

    /* service used for update command */
    pGATTDemo->iUpdateServiceIndex = 1;  /* offset in database1 V6 handle 0x0052, for notification */

    /* send greeting msg */
    BQB_CmdPrint(pGATTDemo, ">> BQB_Mode/ <<\r\n%s", pGATTDemo->CmdIF.cPrompt );
}


