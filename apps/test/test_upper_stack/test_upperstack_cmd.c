#include "gatttest.h"
#include "test_upperstack_cmd.h"
#include "test_upperstack_api.h"
#include <string.h>
#include "test_cmd.h"
#include <FreeRTOS.h>

TGATTTest GattTest;
PGATTTest g_pGattTest = &GattTest;


  /* a single command table entry */
typedef struct _TGATTDemoCmdTableEntry
{
  char              * pCommand;
  char              * pOption;
  char              * pHelp;
  TTestCmdFunc    Func;
} TGATTDemoCmdTableEntry;

/*----------------------------------------------------
 * command table
 * --------------------------------------------------*/

const TGATTDemoCmdTableEntry test_upperstack_CmdTable[] =
{
    {
        "reset",
        "reset\n\r",
        "reset system\n\r",
        test_upperstack_Reset
    },
    /* BlueAPI/GATT related commands */
    {
        "reg",
        "reg\n\r",
        "Register to BlueAPI\n\r",
        test_upperstack_Register
    },
    {
        "rel",
        "rel\n\r",
        "Release from BlueAPI\n\r",
        test_upperstack_Release
    },

  {
        "setreadlen",
        "setreadlen <len>\n\r",
        "setreadlen n \n\r",
        test_upperstack_CmdSetReadDataLength
    },
#if 0    
    {
        "traceLevel",
        "traceLevel <level>\n\r",
        "Set the trace level\n\r",
        test_upperstack_CmdTraceLevel
    },
#endif
    {
        "rembd",
        "rembd <BD>\n\r",
        "Set remote BD\n\r",
        test_upperstack_CmdRemoteBdSet
    },

    {
        "bdlist",
        "bdlist \n\r",
        "Show register BD list\n\r",
        test_upperstack_CmdRemoteBdList
    },
    {
        "bdrm",
        "bdrm <index of BD>\n\r",
        "Remove BD from the list\n\r",
        test_upperstack_CmdRemoteBdRemove
    },
    {
        "devcfg",
        "devcfg <opCode> <...>\n\r",
        "device config\r\n",
        test_upperstack_CmdDevCfg
    },
#if 0
    {
        "devcfgget",
        "devcfgget <opCode>\n\r",
        "device config get\r\n",
        test_upperstack_CmdDevCfgGet
    },
#endif
    {
        "bndlist",
        "bndlist\n\r",
        "Show bonds\n\r",
        test_upperstack_CmdBndList
    },
    {
        "bnddel",
        "bnddel\n\r",
        "Remove bond with rembd\n\r",
        test_upperstack_CmdBndDel
    },


#if (F_BT_LE_PRIVACY_MODE)
    {
        "privacy",
        "privacy <mode>\n\r",
        "Control LE Privacy Mode\n\r",
        test_upperstack_CmdPrivacyMode
    },
#endif /* (F_BT_LE_PRIVACY_MODE) */

    {
        "sauth",
        "sauth  <index of BD> <requirements> <keysize>\n\r",
        "Secman authentication request\n\r",
        test_upperstack_GATTSecurityReq
    },

    {
        "authmode",
        "authmode <pair> <authreq> <iocap> <oob>\n\r",
        "Set pairable mode\n\r",
        test_upperstack_CmdPairableMode
    },

    {
        "authkeyb",
        "authkeyb <index of BD> [6digits]\n\r",
        "SSP passkey entry\n\r",
        test_upperstack_CmdAuthKeyboard
    },
    {
        "authsetoob",
        "authsetoob <index of BD> [C]\n\r",
        "SSP set remote OOB data\n\r",
        test_upperstack_CmdAuthSetOOB
    },
    {
        "srvreg",
        "srvreg\n\r",
        "Register service(s)\n\r",
        test_upperstack_ServiceRegister
    },
    {
        "srvrel",
        "srvrel\n\r",
        "Release service(s)\n\r",
        test_upperstack_ServiceRelease
    },
    
    {
        "update",
        "update [attribute index [count] [service index ] [datalen]]\n\r",
        "Attribute value update\n\r",
        test_upperstack_AttribUpdate
    },

    {
        "reporttx",
        "reporttx \n\r",
        "report tx information for debug\n\r",
        test_upperstack_report_tx
    },

    {
        "reporttxclear",
        "reporttxclear\n\r",
        "clear tx information\n\r",
        test_upperstack_report_tx_clear
    },
    
    {
        "reportrx",
        "reportrx\n\r",
        "Report rx information\n\r",
        test_upperstack_report_rx
    },

    {
        "reportrxsetpktlen",
        "reportrxsetpktlen\n\r",
        "set rx packet length to check\n\r",
        test_upperstack_report_rx_set_pkt_len
    },
        
    {
        "reportrxclear",
        "reportrxclear\n\r",
        "Clear variable for report rx\n\r",
        test_upperstack_report_rx_clear
    },      

    {
        "reportrxdisplay",
        "reportrxdisplay\n\r",
        "reportrxdisplay 0/1\n\r",
        test_upperstack_report_rx_display
    },      
    {
        "ade",
        "ade [0|1]\n\r",
        "enable/disable advertising\n\r",
        test_upperstack_CmdSetAdvertisingEnable
    },
    {
        "adp",
        "adp <adv-type>\n\r",
        "set advertising parameters\n\r",
        test_upperstack_CmdSetAdvertisingParameters
    },
    {
        "addirect",
        "addirect\n\r",
        "directed advertising to remote BD\n\r",
        test_upperstack_CmdSetDirectedAdvertising
    },
    {
        "addata",
        "addata\n\r",
        "Set advertising data\n\r",
        test_upperstack_SetAdvertisingData
    },

    {
        "con",
        "con  <index of BD> [RemoteBdtype] [whitelist] [LocalBdtype] [conInterval] [slaveLantency] [supervisionTimeout]\n\r",
        "Connect, type=0:LE_pub (default), 1:LE_rand\n\r",
        test_upperstack_ConnectReq
    },
    {
        "disc",
        "disc <index of BD>\n\r",
        "Disconnect\n\r",
        test_upperstack_DisconnectReq
    },


    {
        "srvdis",
        "srvdis  <index of BD> [start] [end] [UUID16]\n\r",
        "Service discovery\n\r",
        test_upperstack_ServiceDiscovery
    },
    {
        "reldis",
        "reldis <index of BD> [start] [end]\n\r",
        "Relationship discovery\n\r",
        test_upperstack_RelationshipDiscovery
    },
    {
        "chardis",
        "chardis  <index of BD> [start] [end]\n\r",
        "Characteristic discovery\n\r",
        test_upperstack_CharacteristicDiscovery
    },
    {
        "charddis",
        "charddis <index of BD> [start] [end]\n\r",
        "Characteristic descriptor discovery\n\r",
        test_upperstack_CharacDescriptorDiscovery
    },
    {
        "lsuuid",
        "lsuuid  <index of BD> [start] [end]\n\r",
        "List of UUIDs\n\r",
        test_upperstack_ListUUIDs
    },
    {
        "read",
        "read  <index of BD> [handle] [offset]\n\r",
        "Read attribute\n\r",
        test_upperstack_AttribRead
    },
    {
        "readu",
        "readu <index of BD> [start] [end] [UUID16]\n\r",
        "Read attribute using UUID\n\r",
        test_upperstack_AttribReadUUID
    },
#if 0
    {
    	"readm",
    	"readm <index of BD> [handle1] [handle2] [handle3] [handle4] \n\r",
    	"Read multiple attribute\n\r",
    	test_upperstack_AttribReadMulti
    },
#endif
    {
        "write",
        "write <index of BD> [handle] [value/index/..] [type]\n\r",
        "Write attribute  type=2:write command, other:write request\n\r",
        test_upperstack_AttribWrite
    },
#if (GATTDEMO_PREPARE_WRITE_SUPPPRT)
    {
    	"pwrite",
    	"pwrite <index of BD> \n\r",
    	"Prepare Write attribute\n\r",
    	test_upperstack_AttribPrepareWrite
    },
#endif
    {
        "lshndl",
        "lshndl <index of BD>\n\r",
        "List of locally stored handle/UUID pairs\n\r",
        test_upperstack_CmdHandleUUIDDisplay
    },


    /* GLC specific commands */

    {
        "racpinit",
        "racpinit\n\r",
        "Init. RACP\n\r",
        test_upperstack_GLC_RACPInit
    },
    {
        "racppar",
        "racppar [nbr. of recs] [init. seqnbr]  [seqnbr delta] [special] [reqnum]\n\r",
        "Set RACP parameters\n\r",
        test_upperstack_GLC_RACPSetParam
    },


    {
        "racp",
        "racp <index of BD> [opcode] [operator] [filter type] [operand ..]\n\r",
        "Write to RACP\n\r",
        test_upperstack_GLC_RACPWrite
    },



    {
        "scan",
        "scan [enable] [active] [dups] [whitelist] [localBdType]\n\r",
        "enable LE scan\n\r",
        test_upperstack_CmdScanEnable
    },
    {
        "conupdreq",
        "conupdreq  <index of BD> <minInt> <maxInt> <latency> <supervision>\n\r",
        "LE connection update\n\r",
        test_upperstack_ConUpdateReq
    },
    {
        "conupdresp",
        "conupdresp <index of BD> [0|1]\n\r",
        "LE connection update\n\r",
        test_upperstack_ConUpdateResp
    },
    {
        "addwl",
        "addwl <index of BD>\n\r",
        "add remote BD to whitelist\n\r",
        test_upperstack_CmdAddToWhitelist
    },
    {
        "remwl",
        "remwl <index of BD>\n\r",
        "remove remote BD from whitelist\n\r",
        test_upperstack_CmdRemoveFromWhitelist
    },
    {
        "clearwl",
        "clearwl\n\r",
        "clear whitelist\n\r",
        test_upperstack_CmdClearWhitelist
    },
#if 0
    {
        "vendorsvp",
        "vendorsvp <index of BD>\n\r",
        "vendorsvp \n\r",
        test_upperstack_CmdVendorSetVoicePara
    },
    {
        "settxpower",
        "settxpower <txPower>\n\r",
        "settxpower \n\r",
        test_upperstack_CmdVendorSetBleTxPower
    },
#endif
	{
		"randomaddr",
		"randomaddr\n\r",
		"randomaddr \n\r",
		test_upperstack_CmdSetRandomAddress
	},

    {
        "dump",
        "dump N\n\r",
        "dump N\n\r",
        test_upperstack_dump
    },

    /* miscellaneous */
    {
        "short",
        "short [value]\n\r",
        "Short output: 0-off, 1-on\n\r",
        test_upperstack_CmdShortOutput
    },

    {
    	"conle",
    	"conle [index of BD] [le_psm] [mtu] [mps] [credit] [creditsIncrease]\n\r",
    	"conle: connect LE data channel\n\r",
    	test_upperstack_CmdConLEChan
  	},

  	{
    	"discle",
    	"discle [index of BD] [channel]\n\r",
    	"discle: disconnect LE data channel\n\r",
    	test_upperstack_CmdDiscLEChan
  	},
  	{
		"credit",
		"credit [index of BD] [channel] [credits]\n\r",
		"credit: send LE Flow Control Credit\n\r",
		test_upperstack_CmdCredit
  	},
  	{
		"ledata",
		"ledata [index of BD] [channel] [dataNum] [dataLength] \n\r",
		"ledata: send LE Data\n\r",
		test_upperstack_CmdLEData
  	},
  	{
		"leSec",
		"leSec [le_psm] [active] [mode] [keysize] \n\r",
		"leSec: \n\r",
		test_upperstack_CmdLESec
  	},
#if 0
      	{
		"dlps",
		"dlps n \n\r",
		"dlps: \n\r",
		test_upperstack_dlps
  	},

  	{
		"ota",
		"ota \n\r",
		"ota: \n\r",
		test_upperstack_ota
  	},
#endif
        {
            "buftest",
            "buftest [id]\n\r",
            "buftest: 0-test data ram pool, 1-test buf ram pool,2-test BT4.1 use data ram pool \n\r",
            test_upperstack_Buftest
        },
#if 0        
        {
            "setdatalen",
            "setdatalen [index of BD][txOctets][txTime]\n\r",
            "Sample: setdatalen 0 x20, x30 \n\r",
            test_upperstack_SetDataLength
        },
#endif

    /* MUST be at the end: */
    {
        0,
        0,
        0,
        0
    }
};


void test_upperstack_CmdInit()
{
    int i;

    //g_pGattTest = pvPortMalloc(sizeof(TGATTTest), RAM_TYPE_DATA_ON);
    memset(g_pGattTest, 0, sizeof(TGATTTest));

    /* setup default remote BD */
    for (i = 0; i < GATTDEMO_MAX_LINKS; i++)
    {
        memset( &g_pGattTest->linkTable[i], 0, sizeof(TGATTLink));
    }

    /* service used for update command */
    g_pGattTest->iUpdateServiceIndex = GATTDEMO_MAIN_SERVICE_INDEX;
    g_pGattTest->iReadDataLength = 20;
    g_pGattTest->bEnterDlps = false;
    
    test_module_register(test_upperstack_CmdExecute, g_pGattTest);

    /* send greeting msg */
    test_upperstack_CmdPrint( g_pGattTest, ">> test_upperstack / <<\r\n");
}


/*----------------------------------------------------------------------------
 * execute cmd.
 * -------------------------------------------------------------------------*/

TTestResult test_upperstack_CmdExecute(PGATTTest pGattTest,
        TTestParseResult * pParseResult )
{
    int              i = 0;
    TTestResult  Result = test_ResultCommandNotFound;
    pGattTest = (PGATTTest) pGattTest;

    /* find command in table */
    while (test_upperstack_CmdTable[i].pCommand != NULL)
    {
        if ( strcmp((const char *)test_upperstack_CmdTable[i].pCommand,
                    (const char *)pParseResult->pCommand) == 0 )
        {
            /* command found */

            /* check if user wants help */
            if (pParseResult->iParameterCount && *pParseResult->pParameter[0] == '?')
            {
                test_upperstack_UserIFSendString(pGattTest, test_upperstack_CmdTable[i].pOption, NULL);
                Result = test_ResultOk;
            }
            else
            {
                /* execute command function */
                Result = test_upperstack_CmdTable[i].Func(pGattTest, pParseResult);
            }
            /* exit while */
            break;
        }
        i++;
    };

    return ( Result );
}

