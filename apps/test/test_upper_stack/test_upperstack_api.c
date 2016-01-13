enum { __FILE_NUM__= 0 };

//#include "rtl876x.h"
#include "gatttest.h"
//#include "flags.h"
#include <blueapi.h>
#include <FreeRTOS.h>
#include <string.h>
//#include "ota_api.h"
#include "test_cmd.h"
#include "blueapi_lib.h"
//#include "dlps_platform.h"
#include "upper_stack_dump_api.h"
//#include "device_name.h"
//#include "hal_wdg.h"
#include "trace.h"
#include "os_mem.h"
#include <common_defs.h>

//#include <trace_binary.h>
#define TRACE_MODULE_ID     MID_BT_HCI


//extern uint32_t gTotalHciTxCount;
//extern uint32_t gTotalHciTxFreeCount;
//extern uint32_t gTotalHciRxCount;
//extern uint32_t gTotalHciRxRespCount;

extern uint8_t gRxDisplayOn;
extern uint8_t gRxPacketLen;
extern uint32_t gTestDataMax;
UINT16 appDataRamPoolId_test = 0xFFFF;


#ifdef DEBUG_UPPERSTACK_TX_THROUGHPUT
uint32_t gNotofication = 0;
#endif

#ifdef DEBUG_UPPERSTACK_SCHEDULE_TX_DATA
uint32_t gTxTime = 0;

uint32_t gUpdateTime1 = 0;
uint32_t gUpdateTime2 = 0;
uint32_t gUpdateTime3 = 0;
uint32_t gUpdateTime4 = 0;



extern  void reset_vendor_counter(void);
extern UINT32 read_vendor_counter_no_display(void);
#endif




extern const TAttribAppl test_upperstack_Profile[];
extern const int iGattTestProfileSize;
#if (GLC_SRV_INCLUDE_TEST)
extern const UINT8 test_0[60];
#endif

#define TEST_BD_LIST_IDX   0xFFFF    /* list all hard coded BDs  */
#define TEST_BD_START_IDX  0x1000    /* offset to hard coded BDs */

static const TGATTDBdAddr test_upperstack_TestRemoteBd[] = /* first entry is default BD: */
{
    { 0x88, 0x99, 0x23, 0x4C, 0xe0, 0x00 },    /* PTS LE Dongle 2 */
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

extern PGATTLink test_upperstack_LinkFindByRole(PGATTTest pGattTest, uint16_t role);
extern PGATTLink test_upperstack_LinkFindByBD(PGATTTest pGattTest, uint8_t * pBD);
extern PGATTLink test_upperstack_LinkFindByLocal_MDL_ID(PGATTTest pGattTest, uint16_t local_MDL_ID);
extern void test_upperstack_LinkRelease(PGATTTest pGattTest, PGATTLink pLink);
extern PGATTDService gattTestServiceFind( PGATTTest pGattTest, void * pServiceHandle );
extern PGATTLink test_upperstack_LinkAllocate(PGATTTest pGattTest, uint8_t * pBD);

//extern void blueAPI_dump_os(void);
//extern void *pvPortMalloc( size_t xSize,RAM_TYPE ramType ) ;
//extern void upper_stack_dlps_enter(void);
//extern void upper_stack_dlps_exit(void);
extern void osRescheduleTriggerCallBack(void);


/*----------------------------------------------------------------------------
 * convert BlueAPI subCause to string
 * --------------------------------------------------------------------------*/

static const char * test_upperstack_SubCause2Str( uint16_t wSubCause )
{
    return ( test_upperstack_ErrorCode2Str(wSubCause) );
}


TTestResult test_upperstack_CmdRemoteBdSet( PGATTTest pGattTest,
        TTestParseResult *pParseResult )
{
    int   i = 0;
    int iCnt = 0;
    TTestResult Result = test_ResultOk;
    PGATTLink pLink = NULL;

    if ( pParseResult->iParameterCount > 0 )
    {
#if (TEST_BD_START_IDX)
        iCnt = sizeof(test_upperstack_TestRemoteBd) / BLUE_API_BD_SIZE;

        if ( pParseResult->dwParameter[0] == TEST_BD_LIST_IDX )
        {
            /* list all hard coded BDs (output without flow control, */
            /* partial loss possible ...):                           */

            for ( i = 0; i < iCnt; i++ )
            {
                test_upperstack_CmdPrint( pGattTest,
                                          "RemoteBd[%d]=[%02x:%02x:%02x:%02x:%02x:%02x]\r\n", i,
                                          test_upperstack_TestRemoteBd[i][5], test_upperstack_TestRemoteBd[i][4],
                                          test_upperstack_TestRemoteBd[i][3], test_upperstack_TestRemoteBd[i][2],
                                          test_upperstack_TestRemoteBd[i][1], test_upperstack_TestRemoteBd[i][0] );
            }
            test_upperstack_CmdPrint( pGattTest,
                                      "Use \"rembd [0x%02x+index]\" to set BD !\r\n", TEST_BD_START_IDX );
            return ( Result );
        }
        else if ( (pParseResult->dwParameter[0] >= TEST_BD_START_IDX) &&
                  (pParseResult->dwParameter[0] < (TEST_BD_START_IDX + iCnt))
                )
        {
            /* use hard coded BD */
            pParseResult->dwParameter[0] -= TEST_BD_START_IDX;
            pLink = test_upperstack_LinkAllocate(pGattTest, test_upperstack_TestRemoteBd[pParseResult->dwParameter[0]]);
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
                pLink = test_upperstack_LinkAllocate(pGattTest, RemoteBd);
            }
            else
            {
                test_upperstack_CmdPrint( pGattTest, "No inquiry performed !\r\n" );
            }
        }
    }

    if ( pLink )
    {
        test_upperstack_CmdPrint( pGattTest,
                                  "RemoteBd=[%02x:%02x:%02x:%02x:%02x:%02x] idx=%d\r\n",
                                  pLink->RemoteBd[5], pLink->RemoteBd[4],
                                  pLink->RemoteBd[3], pLink->RemoteBd[2],
                                  pLink->RemoteBd[1], pLink->RemoteBd[0], pLink->idx);
    }
    else
    {
        test_upperstack_CmdPrint( pGattTest,
                                  "Set bd failed\r\n");
    }

    return ( Result );
}

TTestResult test_upperstack_CmdRemoteBdList( PGATTTest pGattTest,
        TTestParseResult *pParseResult )
{
    int   i;
    PGATTLink pLink = NULL;
    for (i = 0; i < GATTDEMO_MAX_LINKS; i++)
    {
        pLink = &pGattTest->linkTable[i];
        if (pLink->isUsed)
        {
            test_upperstack_CmdPrint( pGattTest,
                                      "idx=%d RemoteBd=[%02x:%02x:%02x:%02x:%02x:%02x] connect = %d role=%d\r\n",
                                      pLink->idx, pLink->RemoteBd[5], pLink->RemoteBd[4],
                                      pLink->RemoteBd[3], pLink->RemoteBd[2],
                                      pLink->RemoteBd[1], pLink->RemoteBd[0], pLink->Connected, pLink->role);
        }
    }

    return ( test_ResultOk );
}

TTestResult test_upperstack_CmdRemoteBdRemove( PGATTTest pGattTest,
        TTestParseResult *pParseResult )
{
    PGATTLink pLink = NULL;
    int idx ;
    if (pParseResult->iParameterCount == 1)
        idx = pParseResult->dwParameter[0];
    else
        return test_ResultError;

    if (idx > (GATTDEMO_MAX_LINKS - 1))
        return (test_ResultError);

    pLink = &pGattTest->linkTable[idx];
    test_upperstack_LinkRelease(pGattTest, pLink);

    test_upperstack_CmdPrint( pGattTest,
                              "remove idx=%d \r\n", idx);

    return ( test_ResultOk );
}



/*----------------------------------------------------------------------------
 * command lshndl
 * --------------------------------------------------------------------------*/

TTestResult test_upperstack_CmdHandleUUIDDisplay( PGATTTest pGattTest,
        TTestParseResult *pParseResult )
{
    int   i;
    int idx = pParseResult->dwParameter[0];

    if (idx > (GATTDEMO_MAX_LINKS - 1))
        return (test_ResultError);

    if ( pGattTest->linkTable[idx].iHandleCnt <= 0 )
    {
        test_upperstack_CmdPrint( pGattTest, "List is empty!\r\n" );
    }
    else
    {
        for ( i = 0; i < pGattTest->linkTable[idx].iHandleCnt; i++ )
        {
            test_upperstack_CmdPrint( pGattTest, "handle=0x%04x, UUID=<0x%04x> (%s)\r\n",
                                      pGattTest->linkTable[idx].HandleUUID[i].wHandle,
                                      pGattTest->linkTable[idx].HandleUUID[i].wUUID,
                                      test_upperstack_UUID16ToString( pGattTest->linkTable[idx].HandleUUID[i].wUUID ) );
        }
    }

    return ( test_ResultOk );
}



/*----------------------------------------------------------------------------
 * command short
 *---------------------------------------------------------------------------*/

TTestResult test_upperstack_CmdShortOutput(PGATTTest pGattTest,
        TTestParseResult *pParseResult)
{
    if ( pParseResult->iParameterCount == 0 )
        pGattTest->ShortOutput = true;
    else
        pGattTest->ShortOutput = pParseResult->dwParameter[0] ? true : false;

    return ( test_ResultOk );
}

uint8_t * gDataRamPoolTest = NULL;

TTestResult test_upperstack_Buftest(PGATTTest pGattTest,
                            TTestParseResult *pParseResult)
{
    UINT16 appDataRamPoolId = 0xFFFF;
    //UINT16 appBufRamPoolId = 0xFFFF;
	UINT16 poolSize = 3500;
    
    UINT8 *pAppBuffer = NULL;
    TTestResult Result = test_ResultError;
    BOOL bRet = FALSE;
    int idx = pParseResult->dwParameter[0];
	//gDataRamPoolTest = pvPortMalloc(poolSize, RAM_TYPE_DATA_OFF);
	gDataRamPoolTest = osMemoryAllocate(RAM_TYPE_DATA_OFF, poolSize);
    if(idx == 0)
    {

        bRet = blueAPI_DataRamPoolInit(gDataRamPoolTest, poolSize);
        if(bRet == TRUE)
        {
            test_upperstack_CmdPrint( pGattTest, "<-- gattdbuftest: blueAPI_DataRamPoolInit success, \r\n");
        }

        appDataRamPoolId = blueAPI_DataRamPoolCreate(48, 2);
        blueAPI_DataRamPoolExtend(appDataRamPoolId, 70, 2);
        if (appDataRamPoolId != 0xFFFF)
        {
            test_upperstack_CmdPrint( pGattTest, "<-- gattdbuftest: blueAPI_DataRamPoolCreate success, appPoolId(0x%x)\r\n", appDataRamPoolId);
        }
     

        pAppBuffer = blueAPI_DataRamPoolBufferGet(appDataRamPoolId, 48);
        if (pAppBuffer != NULL)
        {
            test_upperstack_CmdPrint( pGattTest, "<-- gattdbuftest: blueAPI_DataRamPoolBufferGet success,pAppBuffer=%p\r\n",
                          pAppBuffer);
            Result = test_ResultOk;

        }
        blueAPI_DataRamPoolBufferRelease(appDataRamPoolId, pAppBuffer);
        test_upperstack_CmdPrint( pGattTest, "<-- gattdbuftest: blueAPI_DataRamPoolBufferRelease\r\n");
    }
    #if 0
    else if(idx == 1)
    {
        appBufRamPoolId = blueAPI_BufRamPoolCreate(120, 12);
        if (appBufRamPoolId != 0xFFFF)
        {
            test_upperstack_CmdPrint( pGattTest, "<-- gattdbuftest: blueAPI_BufRamPoolCreate success, appPoolId(0x%x)\r\n", appBufRamPoolId);
        }

        pAppBuffer = blueAPI_BufRamPoolBufferGet(appBufRamPoolId, 120);
        if (pAppBuffer != NULL)
        {
            test_upperstack_CmdPrint( pGattTest, "<-- gattdbuftest: blueAPI_BufRamPoolBufferGet success,pAppBuffer=%p\r\n",
                          pAppBuffer);
            Result = test_ResultOk;

        }
        blueAPI_BufRamPoolBufferRelease(appBufRamPoolId, pAppBuffer);
        test_upperstack_CmdPrint( pGattTest, "<-- gattdbuftest: blueAPI_BufRamPoolBufferRelease\r\n"); 
    }
	else
	{
		bRet = blueAPI_DataRamPoolInit(gDataRamPoolTest, poolSize);
        if(bRet == TRUE)
        {
            test_upperstack_CmdPrint( pGattTest, "<-- gattdbuftest: blueAPI_DataRamPoolInit success, \r\n");
        }

        appDataRamPoolId_test = blueAPI_DataRamPoolCreate(1050, 3);
        if (appDataRamPoolId_test != 0xFFFF)
        {
            test_upperstack_CmdPrint( pGattTest, "<-- gattdbuftest: blueAPI_DataRamPoolCreate success, appPoolId(0x%x)\r\n", appDataRamPoolId_test);
        }
	
	}
    #endif
    return ( Result );
}
/*--------------------------------------------------------------------------*/
/*---------------------------- commands ------------------------------------*/
/*--------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
 * command reg
 * --------------------------------------------------------------------------*/

TTestResult test_upperstack_Reset(PGATTTest pGattTest,
        TTestParseResult *pParseResult)
{
    TTestResult Result = test_ResultOk;
    //NVIC_SystemReset();
    return Result;
}

TTestResult test_upperstack_Register(PGATTTest pGattTest,
        TTestParseResult *pParseResult)
{
    TTestResult Result = test_ResultError;

    if (
        blueAPI_RegisterReq((void *)pGattTest,
                             (void *)test_upperstack_BlueAPICallback)
    )
    {
        test_upperstack_CmdPrint( pGattTest, "--> RegisterReq\r\n" );
        Result = test_ResultOk;
    }

#if (GATTDSTORE_ENTRY_COUNT)
    /* clear NVRAM store */
    memset(pGattTest->extStore, 0x00, sizeof(pGattTest->extStore));
#if (GATTDSTORE_NVRAM_ADDRESS)
    if (pParseResult->dwParameter[0] == 1)
    {
        nvramUpdate(GATTDSTORE_NVRAM_ADDRESS, sizeof(pGattTest->extStore), (uint8_t *)&pGattTest->extStore);
    }
    else
    {
        nvramRead(GATTDSTORE_NVRAM_ADDRESS, sizeof(pGattTest->extStore), (uint8_t *)&pGattTest->extStore);
    }
#endif /* (GATTDSTORE_NVRAM_ADDRESS) */
#endif /* (GATTDSTORE_ENTRY_COUNT) */


    return ( Result );
}

/*----------------------------------------------------------------------------
 * command rel
 * --------------------------------------------------------------------------*/

TTestResult test_upperstack_Release(PGATTTest pGattTest,
                                        TTestParseResult *pParseResult)
{
    TTestResult Result = test_ResultError;



    if (
        blueAPI_ReleaseReq()
    )
    {
        test_upperstack_CmdPrint( pGattTest, "--> ReleaseReq\r\n" );
        Result = test_ResultOk;
    }

    return ( Result );
}



/*----------------------------------------------------------------------------
 * command sauth
 * --------------------------------------------------------------------------*/

TTestResult test_upperstack_GATTSecurityReq(PGATTTest pGattTest,
        TTestParseResult *pParseResult)
{
    bool result = false;
    int idx = pParseResult->dwParameter[0];

    if (idx > (GATTDEMO_MAX_LINKS - 1))
        return (test_ResultError);

    if (pGattTest->linkTable[idx].local_MDL_ID_Valid)
    {
        result = blueAPI_GATTSecurityReq(pGattTest->linkTable[idx].local_MDL_ID,
                                         pParseResult->dwParameter[1],
                                         pParseResult->dwParameter[2]
                                        );
    }

    if (result)
    {
        test_upperstack_CmdPrint( pGattTest, "idx= %d--> GATTSecurityReq requirements=0x%x minKeySize=%d\r\n",
                                  pParseResult->dwParameter[0],
                                  pParseResult->dwParameter[1],
                                  pParseResult->dwParameter[2]
                                );
    }
    else
        test_upperstack_CmdPrint(pGattTest, "idx= %d --> GATTSecurityReq failed ", pParseResult->dwParameter[0]);

    return result ? test_ResultOk : test_ResultError;
}


/*----------------------------------------------------------------------------
 * command srvreg
 * --------------------------------------------------------------------------*/

TTestResult test_upperstack_ServiceRegister(PGATTTest pGattTest,
        TTestParseResult *pParseResult)
{
    PGATTDService   pService;
    TTestResult Result = test_ResultError;


    switch (pParseResult->dwParameter[0])
    {
    case 0:
        if ( (pGattTest->iServiceCount >= GATTTEST_MAX_SERVICES) ||
                pGattTest->Service[pGattTest->iServiceCount].Used )
        {
            return ( Result );
        }
        pService       = &pGattTest->Service[pGattTest->iServiceCount];
        pService->Used = true;
        pService->ServiceID = gattTestGetNextServiceID( pGattTest->iServiceCount );

        if ( blueAPI_GATTServiceRegisterReq(
                    iGattTestProfileSize / sizeof(TAttribAppl), /* nbrOfAttrib */
                    (uint8_t *)&test_upperstack_Profile                /* pService  */
                )
           )
        {
            test_upperstack_CmdPrint( pGattTest,
                                      "--> GATTServiceRegisterReq: pService=0x%x, nbrOfAttrib=%d, srvChanged=%d\r\n",
                                      &test_upperstack_Profile,
                                      iGattTestProfileSize / sizeof(TAttribAppl),
                                      pParseResult->dwParameter[1]);

            Result = test_ResultOk;
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

TTestResult test_upperstack_ServiceRelease(PGATTTest pGattTest,
        TTestParseResult *pParseResult)
{
    TTestResult Result = test_ResultError;
    int i = pParseResult->dwParameter[0];

    if (
        pGattTest->Service[i].Used &&
        (pGattTest->Service[i].pServiceHandle != NULL)
    )
    {
        if ( blueAPI_GATTServiceReleaseReq( pGattTest->Service[i].pServiceHandle )
           )
        {
            test_upperstack_CmdPrint( pGattTest, "--> GATTServiceReleaseReq: serviceHandle=0x%x\r\n",
                                      pGattTest->Service[i].pServiceHandle);
            Result = test_ResultOk;
        }
    }

    return ( Result );
}

/*----------------------------------------------------------------------------
 * get attribute value and send update message
 * --------------------------------------------------------------------------*/
static TTestResult test_upperstack_AttribUpdateSend(PGATTTest pGattTest,
        TTestParseResult *pParseResult, bool InitCall )
{
    PGATTDService   pService;
    TTestResult Result = test_ResultError;
    PGATTLink pLink = test_upperstack_LinkFindByRole(pGattTest, GATTDEMO_CON_ROLE_SLAVE);
    if (pLink == NULL)
    {
        Result = test_ResultError;
        test_upperstack_CmdPrint(pGattTest, "test_upperstack_AttribUpdateSend:not find the slave device\r\n");
        return (Result);
    }

#ifdef DEBUG_UPPERSTACK_SCHEDULE_TX_DATA
    gTxTime = read_vendor_counter_no_display();
    if(pGattTest->iUpdateSent == 0)
    {
        gUpdateTime1 = read_vendor_counter_no_display();
    }
    else
    if(pGattTest->iUpdateSent == 1)
    {
        gUpdateTime2 = read_vendor_counter_no_display();
    }
    else
    if(pGattTest->iUpdateSent == 2)
    {
        gUpdateTime3 = read_vendor_counter_no_display();
    }
    else        
    if(pGattTest->iUpdateSent == 3)
    {
        gUpdateTime4 = read_vendor_counter_no_display();
    }
#endif


    if ( InitCall )
    {

        if ( pParseResult->iParameterCount > 2 )
            pGattTest->iUpdateServiceIndex = pParseResult->dwParameter[2];
        else
            pGattTest->iUpdateServiceIndex = GATTDEMO_MAIN_SERVICE_INDEX;

       if ( pParseResult->iParameterCount > 3 )
            pGattTest->iUpdateDataLength = pParseResult->dwParameter[3];
       else
            pGattTest->iUpdateDataLength = 20;

    }
    pService = &pGattTest->Service[pGattTest->iUpdateServiceIndex];

    if (
        pService->Used &&
        (pService->pServiceHandle != NULL)
    )
    {
        uint16_t     wAttribIndex;
        uint16_t     wLength;
        uint8_t    * pData;

        if ( InitCall )
        {
            if ( pParseResult->dwParameter[0] == 0 )
                wAttribIndex = GATT_CHAR1_VALUE_INDEX;
            else
                wAttribIndex = pParseResult->dwParameter[0];
        }
        else
        {
            wAttribIndex = pGattTest->iUpdateAttribIndex;
        }
        pGattTest->iUpdateAttribIndex = wAttribIndex;

        /* service/attribute specific read operation */
        if ( !pLink->Connected ||
                (gattTestAttribGet( pGattTest, pService, wAttribIndex, 0, &wLength, &pData )
                 == GATT_SUCCESS)
           )
        {
            union
            {
                uint32_t d;
                void    *p;
            } requestHandle;
            uint8_t   * pBuffer = NULL;
            uint16_t    wOffset = pGattTest->wDsDataOffset + 3;

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
                                pGattTest->wDsPoolId,
                                wLength,
                                wOffset,
                                (void **)&pBuffer) == blueAPI_CauseSuccess )
                    {
                        memcpy( pBuffer + wOffset, pData, wLength );
                    }
                    else
                    {
                        return ( test_ResultError );
                    }
                }
            }

            requestHandle.d = pGattTest->wUpdReqHandle;
            if ( blueAPI_GATTAttributeUpdateReq(pBuffer,
                                                pService->pServiceHandle,
                                                requestHandle.p,
                                                wAttribIndex,
                                                wLength,
                                                wOffset
                                               )
               )
            {
#if 0
                test_upperstack_CmdPrint( pGattTest,
                                          "--> GATTAttributeUpdateReq: serviceHandle=0x%x, attribIndex=%d\r\n",
                                          pService->pServiceHandle, wAttribIndex);
#endif
                pGattTest->wUpdReqHandle++;
                pGattTest->iUpdateSent++;
                Result = test_ResultOk;
            }
            else
            {
                test_upperstack_CmdPrint( pGattTest,
                                          "!!! illegal parameter (e.g. offset too small)\r\n"
                                          );
                if ( pBuffer != NULL )
                    blueAPI_BufferRelease(pBuffer);
            }
        }
    }

    return ( Result );
}

/*----------------------------------------------------------------------------
 * command update
 * --------------------------------------------------------------------------*/

TTestResult test_upperstack_AttribUpdate(PGATTTest pGattTest,
        TTestParseResult *pParseResult)
{
    int             iSent, iUpdateCnt;
    TTestResult Result = test_ResultOk;
    PGATTLink pLink = test_upperstack_LinkFindByRole(pGattTest, GATTDEMO_CON_ROLE_SLAVE);
    if (pLink == NULL)
    {
        Result = test_ResultError;
        test_upperstack_CmdPrint(pGattTest, "test_upperstack_AttribUpdate:not find the slave device\r\n");
        return (Result);
    }
    
#ifdef DEBUG_UPPERSTACK_TX_THROUGHPUT    
    gNotofication = 1;
#endif
    
    if ( pLink->Connected )
        pGattTest->iUpdateCnt = pParseResult->dwParameter[1];
    else
        pGattTest->iUpdateCnt = 1;
    if ( pGattTest->iUpdateCnt == 0 )           /* repetition count */
        pGattTest->iUpdateCnt = 1;

    iUpdateCnt = pGattTest->iUpdateCnt;
    pGattTest->iUpdateSent      = 0;
    pGattTest->iUpdateConfirmed = 0;
    iSent = 0;
    
#ifdef    DEBUG_UPPERSTACK_SCHEDULE_TX_DATA
    reset_vendor_counter();
    
    gUpdateTime1 = 0;
    gUpdateTime2 = 0;
    gUpdateTime3 = 0;
    gUpdateTime4 = 0;
#endif    
    
    test_upperstack_CmdPrint(pGattTest,
    "iUpdateCnt = %d, wDsCredits = %d\r\n",
    iUpdateCnt,
    pLink->wDsCredits);

	DBG_BUFFER(MODULE_APP, LEVEL_INFO, "TX throughput test start\n\n", 0);

    
    do
    {
        /* call may modify pGattTest->iUpdateCnt value (e.g. more than one */
        /* value in notification ..):                                      */
        Result = test_upperstack_AttribUpdateSend(pGattTest, pParseResult,
                 (iSent == 0 ? true : false) );
        iUpdateCnt = pGattTest->iUpdateCnt;
        iSent++;
    }
    while ( (Result == test_ResultOk) &&
            (iSent < iUpdateCnt) && (iSent < pLink->wDsCredits) );

    return ( Result );
}

TTestResult test_upperstack_report_tx(PGATTTest pGattTest,
        TTestParseResult *pParseResult)
{
    test_upperstack_CmdPrint(pGattTest,
            "report_tx: iUpdateCnt = %d, iUpdateSent = %d, iUpdateConfirmed = %d\r\n",
            pGattTest->iUpdateCnt,
            pGattTest->iUpdateSent,
            pGattTest->iUpdateConfirmed
	         //,gTotalHciTxCount,
            //gTotalHciTxFreeCount
            );
	return test_ResultOk;
}

TTestResult test_upperstack_report_tx_clear(PGATTTest pGattTest,
        TTestParseResult *pParseResult)
{
    pGattTest->iUpdateCnt = 0;
    pGattTest->iUpdateSent = 0;
    pGattTest->iUpdateConfirmed = 0;
    //gTotalHciTxCount = 0;
    //gTotalHciTxFreeCount = 0;
	return test_ResultOk;
}

extern uint32_t gTotalNotificationCount;
TTestResult test_upperstack_report_rx(PGATTTest pGattTest,
        TTestParseResult *pParseResult)
{
    test_upperstack_CmdPrint(pGattTest,
            "report_rx: gTotalNotificationCount = %d\r\n",
            gTotalNotificationCount
	          //,gTotalHciRxCount,
            //gTotalHciRxRespCount
            );
	return test_ResultOk;
}

TTestResult test_upperstack_report_rx_set_pkt_len(PGATTTest pGattTest,
        TTestParseResult *pParseResult)
{
       gRxPacketLen = pParseResult->dwParameter[0];
	return test_ResultOk;
}


TTestResult test_upperstack_report_rx_clear(PGATTTest pGattTest,
        TTestParseResult *pParseResult)
{
    gTotalNotificationCount = 0;
    //gTotalHciRxCount = 0;
    //gTotalHciRxRespCount = 0;
	return test_ResultOk;

}

TTestResult test_upperstack_report_rx_display(PGATTTest pGattTest,
        TTestParseResult *pParseResult)
{
    gRxDisplayOn = pParseResult->dwParameter[0];
	return test_ResultOk;
}




/*----------------------------------------------------------------------------
 * continue update sequence
 * --------------------------------------------------------------------------*/

int test_upperstack_AttribUpdateContinue( PGATTTest pGattTest )
{
    TTestResult Result = test_upperstack_AttribUpdateSend( pGattTest, NULL, false );

    return ( Result == test_ResultOk ? 0 : -1 );
}




static const uint8_t bGattTestAdData[] =
{
   /* Core spec. Vol. 3, Part C, Chapter 18 */
    /* Flags */
    0x02,             /* length     */
    //XXXXMJMJ 0x01, 0x06,      /* type="flags", data="bit 1: LE General Discoverable Mode", BR/EDR not supp. */
    0x01, 0x02,      /* type="flags", data="bit 1: LE General Discoverable Mode" */

    /* Service */

    0x03,             /* length     */
    0x02,            /* type="More 16-bit UUIDs available" */

    LO_WORD(GATT_UUID_GLUCOSE),
    HI_WORD(GATT_UUID_GLUCOSE)



    /* place holder for Local Name, filled by BT stack. if not present */
    /* BT stack appends Local Name.                                    */

    ,
    0x0E,             /* length     */
    0x09,            /* type="Complete local name" */
    'T', 'E', 'S', 'T','_','U','P','S','S','T','A','C','K'     /* STO0 */
    ,
    0x03,             /* length     */
    0x19,            /* type="Appearance" */
    0x00, 0x04      /* Generic Glucose Meter */


};


static const uint8_t bGattTestScanRespData[] =
{
    /* Core spec. Vol. 3, Part C, Chapter 18 */

    0x03,             /* length     */
    0x02,            /* type="More 16-bit UUIDs available" */

    LO_WORD(GATT_UUID_GLUCOSE),
    HI_WORD(GATT_UUID_GLUCOSE)
};


/*----------------------------------------------------------------------------
 * command ade
 * --------------------------------------------------------------------------*/

STATIC TTestResult test_upperstack_SendLEAdvertiseReq(PGATTTest               pGattTest,
        TBlueAPI_LEAdvMode      advMode,
        uint8_t *                  remote_BD,
        TBlueAPI_RemoteBDType   remote_BD_type)
{
    bool               result;

    result = blueAPI_LEAdvertiseReq(advMode);

	
    if (result)
    {
        test_upperstack_CmdPrint( pGattTest, "--> LEAdvertiseReq: advMode=%d, remote_BD=[%s], type=0x%x\r\n",
                                  advMode,
                                  blueAPI_StringizeBd(remote_BD),
                                  remote_BD_type
                                  );
    }

    return result ? test_ResultOk : test_ResultError;
}


TTestResult test_upperstack_CmdSetAdvertisingEnable(PGATTTest pGattTest,
        TTestParseResult *pParseResult)
{
    TBlueAPI_LEAdvMode advMode = blueAPI_LEAdvModeDisabled;

    if (pParseResult->dwParameter[0])
    {
        advMode = blueAPI_LEAdvModeEnabled;
    }

    return ( test_upperstack_SendLEAdvertiseReq(pGattTest,
             advMode,
             NULL,
             blueAPI_RemoteBDTypeLEPublic)
           );
}

/*----------------------------------------------------------------------------
 * command addata
 * --------------------------------------------------------------------------*/

STATIC TTestResult test_upperstack_SendLEAdvertiseDataSetReq(PGATTTest            pGattTest,
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
        test_upperstack_CmdPrint( pGattTest, "--> LEAdvertiseDataSetReq: dataType=%d, dataLength=%d\r\n",
                                  dataType, dataLength
                                  );
    }

    return result ? test_ResultOk : test_ResultError;
}

TTestResult test_upperstack_SetAdvertisingData(PGATTTest pGattTest,
        TTestParseResult *pParseResult)
{
    return ( test_upperstack_SendLEAdvertiseDataSetReq(pGattTest,
             blueAPI_LEDataTypeAdvertisingData,
             sizeof(bGattTestAdData),
             (uint8_t *)bGattTestAdData)
           );
}

/*----------------------------------------------------------------------------
 * command adp
 * --------------------------------------------------------------------------*/

TTestResult test_upperstack_CmdSetAdvertisingParameters(PGATTTest pGattTest,
        TTestParseResult *pParseResult)
{
    int idx = pParseResult->dwParameter[1];
    bool result;
	TBlueAPI_LocalBDType localBdType;

    if (idx > (GATTDEMO_MAX_LINKS - 1))
        return (test_ResultError);

    if ((pParseResult->dwParameter[0] == blueAPI_LEAdvTypeDirectedLowDuty) || (pParseResult->dwParameter[0] == blueAPI_LEAdvTypeDirectedHighDuty))
    {
        if (!pGattTest->linkTable[idx].RemoteBdValid)
        {
            test_upperstack_CmdPrint(pGattTest, "idx = %d,remote bd is not valid", idx);
            return (test_ResultError);
        }
    }

	switch (pParseResult->dwParameter[2])
    {
        case 0:
            localBdType = blueAPI_LocalBDTypeLEPublic;
            break;

        case 1:
            localBdType = blueAPI_LocalBDTypeLERandom;
            break;

        default:
            localBdType = blueAPI_LocalBDTypeLEPublic;
            break;
    }

    result = blueAPI_LEAdvertiseParameterSetReq((TBlueAPI_LEAdvType)pParseResult->dwParameter[0],
             blueAPI_LEFilterAny,
             blueAPI_LEFilterAny,
             0x00A0, /* 20ms */
             0x00B0, /* 30ms */
             localBdType,
             (pParseResult->dwParameter[0] == blueAPI_LEAdvTypeDirectedLowDuty || pParseResult->dwParameter[0] == blueAPI_LEAdvTypeDirectedHighDuty) ?
             pGattTest->linkTable[idx].RemoteBd : NULL,
             blueAPI_RemoteBDTypeLEPublic
                                               );
    if (result)
    {
        test_upperstack_CmdPrint( pGattTest, "--> LEAdvertiseParameterSetReq\r\n");
    }

    return result ? test_ResultOk : test_ResultError;
}

/*----------------------------------------------------------------------------
 * command add
 * --------------------------------------------------------------------------*/

TTestResult test_upperstack_CmdSetDirectedAdvertising(PGATTTest pGattTest,
        TTestParseResult *pParseResult)
{
    int idx = pParseResult->dwParameter[0];

    if (idx > (GATTDEMO_MAX_LINKS - 1))
        return (test_ResultError);

    if (!pGattTest->linkTable[idx].RemoteBdValid)
    {
        test_upperstack_CmdPrint(pGattTest, "idx = %d,remote bd is not valid", idx);
        return (test_ResultError);
    }

    return ( test_upperstack_SendLEAdvertiseReq(pGattTest,
             blueAPI_LEAdvModeDirectedHighDuty,
             pGattTest->linkTable[idx].RemoteBd,
             blueAPI_RemoteBDTypeLEPublic)
           );
}

/*----------------------------------------------------------------------------
 * command con
 * --------------------------------------------------------------------------*/

TTestResult test_upperstack_ConnectReq(PGATTTest pGattTest,
        TTestParseResult *pParseResult)
{
    TTestResult Result = test_ResultError;
    int   idx = pParseResult->dwParameter[0];
    uint16_t conInterval = 0x80;
    uint16_t slaveLantency = 0;  
    uint16_t supervisionTimeout = 1000;
    

    if (idx > (GATTDEMO_MAX_LINKS - 1))
        return (test_ResultError);

    if ( pGattTest->linkTable[idx].RemoteBdValid)
    {
        TBlueAPI_RemoteBDType  bdType;
		TBlueAPI_LocalBDType  localBdType;
        uint8_t *              pRemoteBd = (pParseResult->dwParameter[2] == 0) ?
                                           pGattTest->linkTable[idx].RemoteBd : NULL;

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

        switch (pParseResult->dwParameter[3])
        {
        case 0:
            localBdType = blueAPI_LocalBDTypeLEPublic;
            break;

        case 1:
            localBdType = blueAPI_LocalBDTypeLERandom;
            break;

        default:
            localBdType = blueAPI_LocalBDTypeLEPublic;
            break;
        }

        if(pParseResult->iParameterCount >= 5)
            conInterval = pParseResult->dwParameter[4];

        if(pParseResult->iParameterCount >= 6)
            slaveLantency = pParseResult->dwParameter[5];
        
        if(pParseResult->iParameterCount >= 7)
            supervisionTimeout = pParseResult->dwParameter[6];        
      
        if ( blueAPI_ConnectGATTMDLReq(pRemoteBd,
                                       bdType,
                                       localBdType,
                                       0x60,    /* scanInterval       */
                                       0x60,    /* scanWindow         */
                                       1000,    /* scanTimeout 10s    */
                                       conInterval,       /* connIntervalMin    */
                                       conInterval,       /* connIntervalMax    */
                                       slaveLantency,       /* connLatency        */
                                       supervisionTimeout,      /* supervisionTimeout */
                                       2*conInterval-2
                                      ) )
        {
            Result = test_ResultOk;
            pGattTest->linkTable[idx].role = GATTDEMO_CON_ROLE_MASTER;
            test_upperstack_CmdPrint( pGattTest, "--> GATTConnectReq: remote_BD=[%s], type=0x%04x\r\n",
                                      blueAPI_StringizeBd(pRemoteBd), bdType
                                      );
        }
    }

    return ( Result );
}

/*----------------------------------------------------------------------------
 * command disc
 * --------------------------------------------------------------------------*/

TTestResult test_upperstack_DisconnectReq(PGATTTest pGattTest,
        TTestParseResult *pParseResult)
{
    TTestResult Result = test_ResultError;
    int idx = pParseResult->dwParameter[0];

    if (idx > (GATTDEMO_MAX_LINKS - 1))
        return (test_ResultError);

    if (pGattTest->linkTable[idx].local_MDL_ID_Valid )
    {
        if ( blueAPI_DisconnectMDLReq( pGattTest->linkTable[idx].local_MDL_ID,
                                       blueAPI_CauseConnectionDisconnect )
           )
        {
            Result = test_ResultOk;
            test_upperstack_CmdPrint( pGattTest, "--> GATTDisconnectReq: local_MDL_ID=0x%04x\r\n",
                                      pGattTest->linkTable[idx].local_MDL_ID
                                      );
        }
    }

    return ( Result );
}




/*----------------------------------------------------------------------------
 * send GATTDiscoveryReq through BlueAPI
 * --------------------------------------------------------------------------*/

static TTestResult test_upperstack_SendGATTDiscoveryReq(
    PGATTTest                    pGattTest,
    TBlueAPI_GATTDiscoveryType   discoveryType,
    TTestParseResult        *pParseResult)
{
    uint16_t        wUUID;
    uint8_t *       pUuid128 = NULL;
    TTestResult Result = test_ResultError;
    int idx = pParseResult->dwParameter[0];

    if (idx > (GATTDEMO_MAX_LINKS - 1))
        return (test_ResultError);

    if ( pGattTest->linkTable[idx].Connected )
    {
        switch ( discoveryType )
        {
        default:
            wUUID = 0;
            break;

        case blueAPI_GATTDiscoveryServices:
        case blueAPI_GATTDiscoveryServiceByUUID:
            wUUID = pParseResult->dwParameter[3];



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

        pGattTest->linkTable[idx].wEndingHandle = pParseResult->dwParameter[2];

        if ( blueAPI_GATTDiscoveryReq(pGattTest->linkTable[idx].local_MDL_ID,
                                      discoveryType,
                                      pParseResult->dwParameter[1], /* start */
                                      pParseResult->dwParameter[2], /* end   */
                                      wUUID,     /* uuid16 */
                                      pUuid128   /* pUuid128 */
                                     )
           )
        {
            Result = test_ResultOk;
            test_upperstack_CmdPrint( pGattTest,
                                      "idx = %d --> GATTDiscoveryReq: local_MDL_ID=0x%04x, type=%d, start=0x%04x, end=0x%04x, UUID16=0x%04x\r\n",
                                      idx, pGattTest->linkTable[idx].local_MDL_ID, discoveryType,
                                      pParseResult->dwParameter[1], /* start */
                                      pParseResult->dwParameter[2], /* end   */
                                      wUUID
                                      );
        }
    }

    return ( Result );
}

/*----------------------------------------------------------------------------
 * command srvdis
 * --------------------------------------------------------------------------*/

TTestResult test_upperstack_ServiceDiscovery(PGATTTest pGattTest,
        TTestParseResult *pParseResult)
{
    TTestResult Result;

    Result = test_upperstack_SendGATTDiscoveryReq(
                 pGattTest,
                 (pParseResult->dwParameter[3] != 0) ?
                 blueAPI_GATTDiscoveryServiceByUUID : blueAPI_GATTDiscoveryServices,
                 pParseResult);

    return ( Result );
}

/*----------------------------------------------------------------------------
 * command reldis
 * --------------------------------------------------------------------------*/

TTestResult test_upperstack_RelationshipDiscovery(PGATTTest pGattTest,
        TTestParseResult *pParseResult)
{
    return ( test_upperstack_SendGATTDiscoveryReq( pGattTest,
             blueAPI_GATTDiscoveryRelationship,
             pParseResult ) );
}

/*----------------------------------------------------------------------------
 * command chardis
 * --------------------------------------------------------------------------*/

TTestResult test_upperstack_CharacteristicDiscovery(PGATTTest pGattTest,
        TTestParseResult *pParseResult)
{
    return ( test_upperstack_SendGATTDiscoveryReq( pGattTest,
             blueAPI_GATTDiscoveryCharacteristics,
             pParseResult ) );
}

/*----------------------------------------------------------------------------
 * command charddis
 * --------------------------------------------------------------------------*/

TTestResult test_upperstack_CharacDescriptorDiscovery(PGATTTest pGattTest,
        TTestParseResult *pParseResult)
{
    return ( test_upperstack_SendGATTDiscoveryReq( pGattTest,
             blueAPI_GATTDiscoveryCharacDescriptors,
             pParseResult ) );
}

/*----------------------------------------------------------------------------
 * command lsuuid
 * --------------------------------------------------------------------------*/

TTestResult test_upperstack_ListUUIDs(PGATTTest pGattTest,
        TTestParseResult *pParseResult)
{
    /* same procedure on ATT layer: */
    return ( test_upperstack_CharacDescriptorDiscovery( pGattTest, pParseResult) );
}

/*----------------------------------------------------------------------------
 * command read (using handle)
 * --------------------------------------------------------------------------*/

TTestResult test_upperstack_AttribRead(PGATTTest pGattTest,
        TTestParseResult *pParseResult)
{
    TTestResult Result = test_ResultError;
    int idx = pParseResult->dwParameter[0];

    if (idx > (GATTDEMO_MAX_LINKS - 1))
        return (test_ResultError);


    if (pGattTest->linkTable[idx].Connected )
    {
        if ( blueAPI_GATTAttributeReadReq(pGattTest->linkTable[idx].local_MDL_ID,
                                          blueAPI_GATTReadTypeBasic,
                                          pParseResult->dwParameter[2],  /* readOffset */
                                          pParseResult->dwParameter[1],  /* handle     */
                                          0,          /* endHandle   */
                                          0,          /* UUID16      */
                                          NULL        /* pUUID128    */
                                         )
           )
        {
            Result = test_ResultOk;
            test_upperstack_CmdPrint( pGattTest,
                                      "idx = %d--> GATTAttributeReadReq: local_MDL_ID=0x%04x, type=%d, readOffset=%d, handle=0x%04x\r\n",
                                      idx, pGattTest->linkTable[idx].local_MDL_ID, blueAPI_GATTReadTypeBasic,
                                      pParseResult->dwParameter[2], /* readOffset */
                                      pParseResult->dwParameter[1]
                                      );
        }
    }

    return ( Result );
}

/*----------------------------------------------------------------------------
 * command read (using UUID)
 * --------------------------------------------------------------------------*/

TTestResult test_upperstack_AttribReadUUID(PGATTTest pGattTest,
        TTestParseResult *pParseResult)
{
    uint16_t        wUUID;
    uint8_t *       pUUID128 = NULL;
    TTestResult Result = test_ResultError;
    int idx = pParseResult->dwParameter[0];

    if (idx > (GATTDEMO_MAX_LINKS - 1))
        return (test_ResultError);

    if (pGattTest->linkTable[idx].Connected )
    {
        /* set handle range defaults: */
        if ( pParseResult->dwParameter[1] == 0 )
        {
            pParseResult->dwParameter[1] = 1;
        }
        if ( pParseResult->dwParameter[2] == 0 )
        {
            pParseResult->dwParameter[2] = 0xFFFF;
        }

        wUUID = pParseResult->dwParameter[3];


        if ( blueAPI_GATTAttributeReadReq(pGattTest->linkTable[idx].local_MDL_ID,
                                          blueAPI_GATTReadTypeByUUID,
                                          0,  /* readOffset */
                                          pParseResult->dwParameter[1],  /* startHandle */
                                          pParseResult->dwParameter[2],  /* endHandle   */
                                          wUUID,          /* UUID16      */
                                          pUUID128        /* pUUID128    */
                                         )
           )
        {
            Result = test_ResultOk;
            test_upperstack_CmdPrint( pGattTest,
                                      "idx = %d--> GATTAttributeReadReq: local_MDL_ID=0x%04x, type=%d, start=0x%04x, end=0x%04x, UUID16=0x%04x\r\n",
                                      idx, pGattTest->linkTable[idx].local_MDL_ID, blueAPI_GATTReadTypeByUUID,
                                      pParseResult->dwParameter[1], /* start */
                                      pParseResult->dwParameter[2], /* end   */
                                      wUUID
                                      );
        }
    }

    return ( Result );
}
#if 0
/*----------------------------------------------------------------------------
 * command read (using handles)
 * --------------------------------------------------------------------------*/
TTestResult test_upperstack_AttribReadMulti(PGATTTest pGattTest,
                                          TTestParseResult *pParseResult)
{
  	uint16_t num = 2;
  	uint16_t handles[4];
  	TTestResult Result = test_ResultError;
  	int idx = pParseResult->dwParameter[0];

  	handles[0]=0x31;
  	handles[1]=0x32;

  	if(pParseResult->iParameterCount>5)
  		return (test_ResultError);
  	if(pParseResult->iParameterCount>2)
  	{
    	int i;
    	num = pParseResult->iParameterCount - 1;
		for(i=0;i<num;i++)
		{
	  		handles[i] = pParseResult->dwParameter[i+1];
		}
  	}

  	if(idx > (GATTDEMO_MAX_LINKS-1))
  		return (test_ResultError);
	
  	if (pGattTest->linkTable[idx].Connected )
  	{
    	if ( blueAPI_GATTAttributeReadMultipleReq(pGattTest->linkTable[idx].local_MDL_ID,
                                     num,handles
                                    ))
    	{
      		Result = test_ResultOk;
      		test_upperstack_CmdPrint( pGattTest,
                           "idx = %d--> GATTAttributeReadMultipleReq: local_MDL_ID=0x%04x, num=%d\r\n",
                           idx,pGattTest->linkTable[idx].local_MDL_ID, num
                           );
    	}
  	}
  	return( Result );
}
#endif
/*----------------------------------------------------------------------------
 * command write
 * --------------------------------------------------------------------------*/

TTestResult test_upperstack_AttribWrite(PGATTTest pGattTest,
        TTestParseResult *pParseResult)
{
    TTestResult Result = test_ResultError;
    int idx = pParseResult->dwParameter[0];

    if (idx > (GATTDEMO_MAX_LINKS - 1))
        return (test_ResultError);

    if ( pGattTest->linkTable[idx].Connected )
    {
        uint16_t     wLength;
        uint8_t    * pData;
        void       * pBuffer;
        uint16_t     wOffset = pGattTest->wDsDataOffset + 3;

        /* service/attribute specific write data setup */
        if ( test_upperstack_AttribGetWriteData( pGattTest, pParseResult,
                &wLength, &pData ) == GATT_SUCCESS )
        {
            /* copy attribute value to buffer position that allows re-usage by stack */
            /* without copying ..                                                   */
            if ( blueAPI_BufferGet(
                        pGattTest->wDsPoolId,
                        wLength,
                        wOffset,
                        (void **)&pBuffer) == blueAPI_CauseSuccess )
            {
                TBlueAPI_GATTWriteType   writeType = (pParseResult->dwParameter[3] == 2) ?
                                                     blueAPI_GATTWriteTypeCommand : blueAPI_GATTWriteTypeRequest;

                memcpy( ((uint8_t *)pBuffer) + wOffset, pData, wLength );

                if ( blueAPI_GATTAttributeWriteReq(
                            pBuffer,
                            pGattTest->linkTable[idx].local_MDL_ID,
                            writeType,
                            pParseResult->dwParameter[1],  /* handle */
                            wLength,                       /* length */
                            wOffset                        /* offset */
                        )
                   )
                {
                    Result = test_ResultOk;
                    test_upperstack_CmdPrint( pGattTest,
                                              "idx= %d--> GATTAttributeWriteReq: local_MDL_ID=0x%04x, type=%d, handle=0x%04x, length=%d\r\n",
                                              idx, pGattTest->linkTable[idx].local_MDL_ID, writeType,
                                              pParseResult->dwParameter[1], /* handle */
                                              wLength
                                              );
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

/*----------------------------------------------------------------------------
 * command pwrite
 * --------------------------------------------------------------------------*/
#if (GATTDEMO_PREPARE_WRITE_SUPPPRT)
TTestResult test_upperstack_AttribPrepareWrite(PGATTTest pGattTest,
                                          TTestParseResult *pParseResult)
{
  	TTestResult Result = test_ResultError;
  	int idx = pParseResult->dwParameter[0];
  
  	if(idx > (GATTDEMO_MAX_LINKS-1))
  		return (test_ResultError);

  	if ( pGattTest->linkTable[idx].Connected )
  	{
    	uint16_t     wLength;
		uint8_t      *pData = (uint8_t *)test_0;
    	void       * pBuffer;
    	uint16_t     wOffset = pGattTest->wDsDataOffset + 3;
		uint16_t     writeOffset = 0;

    	wLength = 18;
    	/* service/attribute specific write data setup */
    	if ( blueAPI_BufferGet(pGattTest->wDsPoolId,
                               wLength,
                               wOffset,
                               (void **)&pBuffer) == blueAPI_CauseSuccess )
    	{
      		memcpy( ((uint8_t *)pBuffer)+wOffset, pData, wLength );

      		if ( blueAPI_GATTAttributePrepareWriteReq(
                               pBuffer,
                               pGattTest->linkTable[idx].local_MDL_ID,
                               0x3d,  /* handle */
                               wLength,                       /* length */
				               writeOffset,
                               wOffset                        /* offset */
                                          )
         	)
      		{
        		Result = test_ResultOk;
        		test_upperstack_CmdPrint( pGattTest,
            		"idx= %d--> GATTAttributePrepareWriteReq: local_MDL_ID=0x%04x,handle=0x%04x length=%d valueOffset=%d\r\n",
            		idx,pGattTest->linkTable[idx].local_MDL_ID, 0x3d,
            		wLength,
            		writeOffset
            		);
      		}
      		else
      		{
        		blueAPI_BufferRelease(pBuffer);
      		}
     
    	}
  	}

  	return( Result );
}

TTestResult test_upperstack_SendPrepareWrite(PGATTTest pGattTest,
	                                      uint16_t local_MDL_ID,
                                          uint16_t writeOffset,
                                          uint16_t length)
{
  TTestResult Result = test_ResultError;
  PGATTLink pLink = NULL;
  pLink = test_upperstack_LinkFindByLocal_MDL_ID(pGattTest,local_MDL_ID);
  if ( pLink->Connected )
  {
    uint16_t     wLength;
	uint8_t      *pData = (uint8_t *)test_0;
    void       * pBuffer;
    uint16_t     wOffset = pGattTest->wDsDataOffset + 1;

    wLength = length;
    /* service/attribute specific write data setup */
    if ( blueAPI_BufferGet(
                           pGattTest->wDsPoolId,
                           wLength,
                           wOffset,
                           (void **)&pBuffer) == blueAPI_CauseSuccess )
    {
      memcpy( ((uint8_t *)pBuffer)+wOffset, pData+writeOffset, wLength );

      if ( blueAPI_GATTAttributePrepareWriteReq(
                               pBuffer,
                               local_MDL_ID,
                               0x3d,  /* handle */
                               wLength,                       /* length */
				               writeOffset,
                               wOffset                        /* offset */
                                          )
         )
      {
        Result = test_ResultOk;
        test_upperstack_CmdPrint( pGattTest,
            "-> GATTAttributePrepareWriteReq: local_MDL_ID=0x%04x, length=%d valueOffset=%d\r\n",
            local_MDL_ID, 
            wLength,
            writeOffset
            );
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

TTestResult test_upperstack_SendExecuteWrite(PGATTTest pGattTest,
                                          uint16_t local_MDL_ID,
                                          uint8_t flags)
{
  TTestResult Result = test_ResultOk;
  PGATTLink pLink = NULL;
  pLink = test_upperstack_LinkFindByLocal_MDL_ID(pGattTest,local_MDL_ID);

  if ( pLink->Connected )
  {
    if(blueAPI_GATTAttributeExecuteWriteReq(
                                   local_MDL_ID,
                                   flags
                                  ))
    {
      Result = test_ResultOk;
      test_upperstack_CmdPrint( pGattTest,
              "--> GATTAttributeExecuteWriteReq: local_MDL_ID=0x%04x, flags=0x%02x\r\n",
              local_MDL_ID,flags
              );
    }
  }

  return( Result );
}
#endif


/*----------------------------------------------------------------------------
 * command scan
 * --------------------------------------------------------------------------*/
TTestResult test_upperstack_CmdScanEnable(PGATTTest pGattTest,
        TTestParseResult *pParseResult)
{
    bool                result;
    TBlueAPI_LEScanMode scanMode = blueAPI_LEScanDisabled;
	TBlueAPI_LocalBDType localBdType;

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
	switch (pParseResult->dwParameter[4])
    {
        case 0:
            localBdType = blueAPI_LocalBDTypeLEPublic;
            break;

        case 1:
            localBdType = blueAPI_LocalBDTypeLERandom;
            break;

        default:
            localBdType = blueAPI_LocalBDTypeLEPublic;
            break;
    }

    result = blueAPI_LEScanReq(scanMode,
                               0x0080,
                               0x0020,
                               (TBlueAPI_LEFilterPolicy)pParseResult->dwParameter[3],
                               localBdType,
                               pParseResult->dwParameter[2]
                              );
    if (result)
    {
        test_upperstack_CmdPrint( pGattTest, "--> LEScanReq: scanMode=%d\r\n",
                                  scanMode
                                  );
    }

    return result ? test_ResultOk : test_ResultError;
}

/*----------------------------------------------------------------------------
 * command conupdreq
 * --------------------------------------------------------------------------*/

TTestResult test_upperstack_ConUpdateReq(PGATTTest pGattTest,
        TTestParseResult *pParseResult)
{
    bool result = test_ResultError;
    int idx = pParseResult->dwParameter[0];

    if (idx > (GATTDEMO_MAX_LINKS - 1))
        return (test_ResultError);

    if (pGattTest->linkTable[idx].local_MDL_ID_Valid)
    {
        result = blueAPI_LEConnectionUpdateReq(pGattTest->linkTable[idx].local_MDL_ID,
                                               pParseResult->dwParameter[1],
                                               pParseResult->dwParameter[2],
                                               pParseResult->dwParameter[3],
                                               pParseResult->dwParameter[4]
                                              );
    }

    if (result)
    {
        test_upperstack_CmdPrint( pGattTest, "--> LEConnectionUpdateReq: local_MDL_ID=0x%04x\r\n",
                                  pGattTest->linkTable[idx].local_MDL_ID
                                  );
    }

    return result ? test_ResultOk : test_ResultError;
}

/*----------------------------------------------------------------------------
 * command conupdresp
 * --------------------------------------------------------------------------*/

TTestResult test_upperstack_ConUpdateResp(PGATTTest pGattTest,
        TTestParseResult *pParseResult)
{
    bool            result = FALSE;
    int idx = pParseResult->dwParameter[0];

    TBlueAPI_Cause  cause = pParseResult->dwParameter[1] ?
                            blueAPI_CauseAccept : blueAPI_CauseReject;
    if (idx > (GATTDEMO_MAX_LINKS - 1))
        return (test_ResultError);

    if (pGattTest->linkTable[idx].local_MDL_ID_Valid)
    {
        result = blueAPI_LEConnectionUpdateConf(pGattTest->linkTable[idx].local_MDL_ID,
                                                cause
                                               );
    }

    if (result)
    {
        test_upperstack_CmdPrint( pGattTest,
                                  "--> LEConnectionUpdateConf: local_MDL_ID=0x%04x, cause=%s\r\n",
                                  pGattTest->linkTable[idx].local_MDL_ID,
                                  blueAPI_CauseString(cause)
                                  );
    }

    return result ? test_ResultOk : test_ResultError;
}

/*----------------------------------------------------------------------------
 * command addwl
 * --------------------------------------------------------------------------*/

STATIC TTestResult test_upperstack_SendLEModifyWhitelistReq(PGATTTest pGattTest,
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
        test_upperstack_CmdPrint( pGattTest,
                                  "--> LEModifyWhitelistReq: op=%d, remote_BD=%s, type=%d\r\n",
                                  operation,
                                  blueAPI_StringizeBd(remote_BD),
                                  remote_BD_type
                                  );
    }

    return result ? test_ResultOk : test_ResultError;
}


TTestResult test_upperstack_CmdAddToWhitelist(PGATTTest pGattTest,
        TTestParseResult *pParseResult)
{
    int idx = pParseResult->dwParameter[0];

    if (idx > (GATTDEMO_MAX_LINKS - 1))
        return (test_ResultError);

    if (!pGattTest->linkTable[idx].RemoteBdValid)
    {
        test_upperstack_CmdPrint(pGattTest, "idx = %d,remote bd is not valid", idx);
        return (test_ResultError);
    }

    return ( test_upperstack_SendLEModifyWhitelistReq(pGattTest,
             blueAPI_LEWhitelistOpAdd,
             pGattTest->linkTable[idx].RemoteBd,
             blueAPI_RemoteBDTypeLEPublic)
           );
}

/*----------------------------------------------------------------------------
 * command remwl
 * --------------------------------------------------------------------------*/

TTestResult test_upperstack_CmdRemoveFromWhitelist(PGATTTest pGattTest,
        TTestParseResult *pParseResult)
{
    int idx = pParseResult->dwParameter[0];

    if (idx > (GATTDEMO_MAX_LINKS - 1))
        return (test_ResultError);

    if (!pGattTest->linkTable[idx].RemoteBdValid)
    {
        test_upperstack_CmdPrint(pGattTest, "idx = %d,remote bd is not valid", idx);
        return (test_ResultError);
    }

    return ( test_upperstack_SendLEModifyWhitelistReq(pGattTest,
             blueAPI_LEWhitelistOpRemove,
             pGattTest->linkTable[idx].RemoteBd,
             blueAPI_RemoteBDTypeLEPublic)
           );
}

/*----------------------------------------------------------------------------
 * command clearwl
 * --------------------------------------------------------------------------*/

TTestResult test_upperstack_CmdClearWhitelist(PGATTTest pGattTest,
        TTestParseResult *pParseResult)
{
    return ( test_upperstack_SendLEModifyWhitelistReq(pGattTest,
             blueAPI_LEWhitelistOpClear,
             NULL,
             blueAPI_RemoteBDTypeLEPublic)
           );
}
#if 0
TTestResult test_upperstack_CmdVendorSetVoicePara(PGATTTest pGattTest,
        TTestParseResult *pParseResult)
{

    bool result = test_ResultError;
    uint8_t ExtData[24] = {0x01, 0x02, 0x03, 0x04, 0x01, 0x02, 0x03, 0x04, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x05, 0x06, 0x07, 0x08, 0x05, 0x06, 0x07, 0x08};
    uint8_t extLen = sizeof(ExtData);


    int idx = pParseResult->dwParameter[0];

    if (idx > (GATTDEMO_MAX_LINKS - 1))
        return (test_ResultError);

    if ( pGattTest->linkTable[idx].Connected )
    {


        result = blueAPI_VendorSetVoiceParaReq(
                     pGattTest->linkTable[idx].local_MDL_ID,
                     0x01,
                     0x10,
                     0x00,
                     0x05,
                     0x06,
                     extLen,
                     ExtData
                 );
        if (result)
        {
            test_upperstack_CmdPrint( pGattTest, "--> test_upperstack_CmdVendorSetVoicePara\r\n");
        }
    }

    return result ? test_ResultOk : test_ResultError;
}
#endif
#if 0
TTestResult test_upperstack_CmdVendorSetBleTxPower(PGATTTest pGattTest,
        TTestParseResult *pParseResult)
{

    bool result = test_ResultError;
#ifdef RTL8762AX_VB
    result = blueAPI_SetBleTxPowerReq(
                     pParseResult->dwParameter[0]
                 );
    if (result)
    {
        test_upperstack_CmdPrint( pGattTest, "--> test_upperstack_CmdSetBleTxPower\r\n");
    }
#endif
    return result ? test_ResultOk : test_ResultError;
}
#endif

TTestResult test_upperstack_CmdSetRandomAddress(PGATTTest pGattTest,
        TTestParseResult *pParseResult)
{

    bool result = test_ResultError;
    uint8_t bd[6] = {0x0,0x01,0x02,0x03,0x04,0x05};    

 	result = blueAPI_SetRandomAddressReq(bd);
    if (result)
    {
        test_upperstack_CmdPrint( pGattTest, "--> test_upperstack_CmdSetRandomAddress\r\n");
    }

    return result ? test_ResultOk : test_ResultError;
}

TTestResult test_upperstack_CmdConLEChan(PGATTTest pGattTest,
                                  TTestParseResult *pParseResult)
{
	bool			result = test_ResultOk;
	int idx = pParseResult->dwParameter[0];
	uint16_t le_psm = 0x45;
	uint16_t credit = 8;
	uint16_t mtu = 1050;
	uint16_t mps = 46;
	uint16_t creditsIncrease = 4;
	  
	if(idx > (GATTDEMO_MAX_LINKS-1))
	  	return (test_ResultError);

	if(pParseResult->iParameterCount>1)
		le_psm = pParseResult->dwParameter[1];	
	if(pParseResult->iParameterCount>2)
		mtu = pParseResult->dwParameter[2];
	if(pParseResult->iParameterCount>3)
		mps = pParseResult->dwParameter[3];
	if(pParseResult->iParameterCount>4)
		credit = pParseResult->dwParameter[4];
	if(pParseResult->iParameterCount>5)
		creditsIncrease = pParseResult->dwParameter[5];
	  
	if(pGattTest->linkTable[idx].local_MDL_ID_Valid)
	{
	  	result = blueAPI_CreateLEDataChannelReq( pGattTest->linkTable[idx].local_MDL_ID,
	  	                                    	le_psm,
	  	                                    	mtu,
	  	                                    	mps,
												credit,
												creditsIncrease
												);
	}
	
	if (result)
	{
	  	test_upperstack_CmdPrint( pGattTest,
					 	"--> CreateLEDataChannelReq: local_MDL_ID=0x%04x\r\n",
					 	pGattTest->linkTable[idx].local_MDL_ID
					 	);
	}
	
	return result ? test_ResultOk : test_ResultError;
}

TTestResult test_upperstack_CmdDiscLEChan(PGATTTest pGattTest,
                                  TTestParseResult *pParseResult)
{
  	bool Result = test_ResultOk;
  	int idx = pParseResult->dwParameter[0];
  	int channel = pParseResult->dwParameter[1];
  	PGATTLink pLink;
  	PGATTLEChannel pChan;
	
  	if(idx > (GATTDEMO_MAX_LINKS-1))
		return (test_ResultError);

  	pLink = &pGattTest->linkTable[idx];
  	pChan = test_upperstack_LEChanFind(pGattTest,pLink,channel);
  	if(pChan == NULL)
  		return (test_ResultError);
  	if(pLink->Connected)
  	{
    	if((pChan->isDataChanConnected))
    	{
	  		Result = blueAPI_DisconnectLEDataChannelReq( pGattTest->linkTable[idx].local_MDL_ID,
										  pChan->channel);
    	}
  	}
  
  	if (Result)
  	{
		test_upperstack_CmdPrint( pGattTest,
				   				"--> DisconnectLEDataChannelReq: local_MDL_ID=0x%04x, channel=0x%04x\r\n",
				   				pGattTest->linkTable[idx].local_MDL_ID,
				   				pChan->channel
				   				);
  	}

  	return( Result ? test_ResultOk : test_ResultError );
}

TTestResult test_upperstack_CmdCredit(PGATTTest pGattTest,
                                  TTestParseResult *pParseResult)
{
  	bool Result = test_ResultOk;
  	int idx = pParseResult->dwParameter[0];
  	int channel = pParseResult->dwParameter[1];
  	int credit = 2;
  	PGATTLink pLink;
  	PGATTLEChannel pChan;
  	if(idx > (GATTDEMO_MAX_LINKS-1))
		return (test_ResultError);

  	pLink = &pGattTest->linkTable[idx];
  	if(pParseResult->iParameterCount >= 3)
  		credit = pParseResult->dwParameter[2];
  	pChan = test_upperstack_LEChanFind(pGattTest,pLink,channel);
  	if(pChan == NULL)
  		return (test_ResultError);
  	if(pLink->Connected)
  	{
    	if((pChan->isDataChanConnected))
    	{
	  		Result = blueAPI_SendLEFlowControlCreditReq( pGattTest->linkTable[idx].local_MDL_ID,
										  pChan->channel, credit);
    	}
  	}
  
  	if (Result)
  	{
		test_upperstack_CmdPrint( pGattTest,
				   "--> SendLEFlowControlCreditReq: local_MDL_ID=0x%04x, channel=0x%04x credit=%d\r\n",
				   pGattTest->linkTable[idx].local_MDL_ID,
				   pChan->channel,
				   credit
				   );
  	}

  	return( Result ? test_ResultOk : test_ResultError );
}

TTestResult test_upperstack_CmdLEData(PGATTTest pGattTest,
                                  TTestParseResult *pParseResult)
{
  	bool Result = test_ResultOk;
  	int idx = pParseResult->dwParameter[0];
  	int channel = pParseResult->dwParameter[1];
  	int dataNum = 1;
	int dataLength = 1;
  	PGATTLink pLink;
  	PGATTLEChannel pChan;
	int i;
  	uint8_t *      pBuffer = NULL;
  	uint8_t *      value;
  	uint16_t       wOffset = pGattTest->wDsDataOffset+1;
  	uint8_t test[1050] = {0};

  	if(idx > (GATTDEMO_MAX_LINKS-1))
		return (test_ResultError);
	if(pParseResult->iParameterCount>= 3)
		dataNum = pParseResult->dwParameter[2];
	if(pParseResult->iParameterCount>= 4)
		dataLength = pParseResult->dwParameter[3];
  	if(dataLength > 1050)
		return (test_ResultError);
	

  	pLink = &pGattTest->linkTable[idx];

  	pChan = test_upperstack_LEChanFind(pGattTest,pLink,channel);

	for(i = 0; i < dataLength; i++)
    {
    	test[i] = gTestDataMax;
		gTestDataMax++;
    }
    value    = (uint8_t *)test;

  	if(pChan == NULL)
  		return (test_ResultError);

	if ( pLink->Connected )
        pGattTest->iUpdateCnt = dataNum;
    else
        pGattTest->iUpdateCnt = 1;
    if ( pGattTest->iUpdateCnt == 0 )           /* repetition count */
        pGattTest->iUpdateCnt = 1;

    pGattTest->iUpdateSent      = 0;
    pGattTest->iUpdateConfirmed = 0;
	pGattTest->iUpdateDataLength = dataLength;

  	if(pLink->Connected)
  	{
  		for(i = 0; i < pGattTest->iUpdateCnt; i++)
    	{ 
    		if((pChan->isDataChanConnected))
    		{
    	    	if(pChan->maxDsCredit > 0)
    	    	{
					if(appDataRamPoolId_test != 0xFFFF)
					{
						pBuffer = blueAPI_DataRamPoolBufferGet(appDataRamPoolId_test, dataLength + wOffset);
					}
					else
					{
						if ( blueAPI_BufferGet(
                           		pGattTest->wDsPoolId,
                           		dataLength,
                           		wOffset,
                           		(void **)&pBuffer) != blueAPI_CauseSuccess )
						{
                           	pBuffer = NULL;
						}
					}
       				if ( pBuffer != NULL)
       				{
       					   	 			/*	test_upperstack_CmdPrint( pGattTest,
				   			"--> SendLEData:get buffer success i= %d \r\n",
				   			i);*/
         				memcpy( pBuffer+wOffset, value, dataLength );
	     				Result = blueAPI_LEDataReq( pBuffer,pGattTest->linkTable[idx].local_MDL_ID,
										  pChan->channel, dataLength,wOffset);
						pChan->maxDsCredit--;
						pGattTest->iUpdateSent++;
       				}
	   				else 
	   				{
	   	 				test_upperstack_CmdPrint( pGattTest,
				   			"--> SendLEData:get buffer failed i= %d maxDsCredit %d\r\n",
				   			i,
				   			pChan->maxDsCredit);
						break;
	   				}
	   			}
				else
				{
				    test_upperstack_CmdPrint( pGattTest,
				   			"--> SendLEData: maxDsCredit = 0 i = %d\r\n",
				   			i);
					break;
				}
    	    }
			else
			{
				test_upperstack_CmdPrint( pGattTest,
				   			"--> SendLEData: channel is not connected\r\n");
				break;
			}
    	
    	}
  	}
  
  	if (Result)
  	{
		/*test_upperstack_CmdPrint( pGattTest,
				   "--> SendLEData: local_MDL_ID=0x%04x, channel=0x%04x\r\n",
				   pGattTest->linkTable[idx].local_MDL_ID,
				   pChan->channel
				   );*/
  	}

  	return( Result ? test_ResultOk : test_ResultError );
}

TTestResult test_upperstack_CmdLESec(PGATTTest pGattTest,
                                  TTestParseResult *pParseResult)
{
  	bool Result = test_ResultOk;
  	int le_psm = pParseResult->dwParameter[0];
  	int mode = pParseResult->dwParameter[2];
  	int keysize = pParseResult->dwParameter[3];

  	Result = blueAPI_LEPsmSecuritySetReq(le_psm,pParseResult->dwParameter[1],
  	                                   (TBlueAPI_LESecurityMode)mode,keysize);


  	return( Result ? test_ResultOk : test_ResultError );
}
#if 0
TTestResult test_upperstack_dlps(PGATTTest pGattTest,
                                  TTestParseResult *pParseResult)
{
    bool result = test_ResultOk;
    uint8_t bEnterDlps = pParseResult->dwParameter[0];

    pGattTest->bEnterDlps = bEnterDlps;

    switch(bEnterDlps)
    {
        case 0:
        LPS_MODE_Pause();
        test_upperstack_CmdPrint( pGattTest,
                                "LPS_MODE_Pause\r\n"
                                );
            break;
        case 1:
        LPS_MODE_Resume();
        test_upperstack_CmdPrint( pGattTest,
				   "LPS_MODE_Resume\r\n"
				   );            
            break;
        case 2:
        upper_stack_dlps_enter();
        test_upperstack_CmdPrint( pGattTest,
				   "upper_stack_dlps_enter\r\n"
				   );
            break;
        case 3:
        upper_stack_dlps_exit();
        test_upperstack_CmdPrint( pGattTest,
				   "upper_stack_dlps_exit\r\n"
				   );
            break;
        default:
            break;
                
    }


    
    return( result ? test_ResultOk : test_ResultError );
}

TTestResult test_upperstack_ota(PGATTTest pGattTest,
                                  TTestParseResult *pParseResult)
{
    TTestResult result = test_ResultOk;
    dfuSetOtaMode(TRUE);
    HalWatchDogConfig(0,5,1);
    HalWatchDogEnable();
    return( result );
}
#endif
#if 0
extern void stSetTraceLevel(uint32_t traceLevel);

TTestResult test_upperstack_CmdTraceLevel(PGATTTest pGattTest,
        TTestParseResult *pParseResult)
{
    int level = pParseResult->dwParameter[0];
    stSetTraceLevel(level);

    return ( test_ResultOk );
}
#endif
TTestResult test_upperstack_CmdSetReadDataLength(PGATTTest pGattTest,
        TTestParseResult *pParseResult)
{
    int read_data_len = pParseResult->dwParameter[0];
    if(pGattTest != NULL)
    {
        pGattTest->iReadDataLength = read_data_len;
        return ( test_ResultOk );
    }
    else
    {
        return ( test_ResultError);
    }

}
#if 0
TTestResult test_upperstack_SetDataLength(PGATTTest pGattTest,
        TTestParseResult *pParseResult)

{
    TTestResult Result = test_ResultError;
    int idx = pParseResult->dwParameter[0];

    if (idx > (GATTDEMO_MAX_LINKS - 1))
        return (test_ResultError);


    if (pGattTest->linkTable[idx].Connected )
    {
        if ( blueAPI_SetDataLengthReq(
                                          pGattTest->linkTable[idx].local_MDL_ID,                                         
                                          pParseResult->dwParameter[1],  /* txOctets */
                                          pParseResult->dwParameter[2]   /* txTIme     */                                        
                                         )
           )
        {
            Result = test_ResultOk;
            test_upperstack_CmdPrint( pGattTest,
                                      "idx = %d--> blueAPI_SetDataLengthReq: local_MDL_ID=0x%04x, txOctets=0x%x, txTime=0x%x\r\n",
                                      idx, pGattTest->linkTable[idx].local_MDL_ID, 
                                      pParseResult->dwParameter[1],
                                      pParseResult->dwParameter[2]
                                      );
        }
    }

    return ( Result );
}
#endif

TTestResult test_upperstack_dump(PGATTTest pGattTest,
        TTestParseResult *pParseResult)
{
    TTestResult result = test_ResultOk;

    int moduleID = pParseResult->dwParameter[0];

    switch(moduleID)
    {
#if 0
        case 1:
            blueAPI_dump_os();
            break;

        case 2:
            blueAPI_dump_BlueAPIUsMessageLength();
            blueAPI_dump_BlueAPIDsMessageLength();
            break;
        case 3:
            blueAPI_dump_memory();
            break;
        case 4:
            blueAPI_dump_messageOffset();
            break;
#endif
        case 1:
            blueAPI_dump_ProtocolData();
            break;
        case 2:
            blueAPI_dump_all_pools();
            break;
#if 0
        case 7:
			{
            	uint16_t totalDataOffRamSize = 
            	blueAPI_dump_DataRamOff_Module();
				DBG_BUFFER(MODULE_APP, LEVEL_ERROR, "blueAPI_dump_DataRamOff_Module: totalDataOffRamSize = %d\n\n", 1, totalDataOffRamSize);
        	}
            break;
        case 8:
			{
            	uint16_t totalDataOnRamSize = 
            	blueAPI_dump_DataRamOn_Module();

            	DBG_BUFFER(MODULE_APP, LEVEL_ERROR, "blueAPI_dump_DataRamOn_Module: totalDataOnRamSize = %d\n\n", 1, totalDataOnRamSize);

        	}
            
            break;
        case 9:
			{
            	uint16_t totalBufOffRamSize = 
            	blueAPI_dump_BufRamOff_Module();             
				DBG_BUFFER(MODULE_APP, LEVEL_ERROR, "blueAPI_dump_BufRamOff_Module: totalBufOffRamSize = %d\n\n", 1, totalBufOffRamSize);
        	}
            break;
        case 10:
            osRescheduleTriggerCallBack();
            break;
        case 11:
            DBG_DIRECT("[Memory Dump] Data RAM Address: 0x20000000~0x2000FFFF\n");
            DumpRawMemory((UINT32*)0x20000000, 64*1024);
            DumpMemoryHEX386(0x20000000, 64*1024);
            DBG_DIRECT("Memory Dump Done.\n");
            break;
#endif            
        case 0xFF:
            //blueAPI_dump_os();
            //blueAPI_dump_BlueAPIUsMessageLength();
            //blueAPI_dump_BlueAPIDsMessageLength();
            //blueAPI_dump_memory();
            //blueAPI_dump_messageOffset();
            blueAPI_dump_ProtocolData();
            blueAPI_dump_all_pools();
            break;   

        default:
            test_upperstack_CmdPrint(pGattTest, "test_upperstack_dump invalid id(0x%x)", moduleID);
            result = test_ResultWrongParameter;
            break;
    }       

    return result;    
}

/*--------------------------------------------------------------------------*/
/*-------------------------- BlueAPI messages ------------------------------*/
/*--------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_RegisterRsp
 * --------------------------------------------------------------------------*/

static bool test_upperstack_Handle_RegisterRsp( PGATTTest            pGattTest,
        PBlueAPI_RegisterRsp pRegisterRsp )
{
    test_upperstack_CmdPrint( pGattTest, "<-- RegisterRsp: cause=%s\r\n",
                              blueAPI_CauseString(pRegisterRsp->cause)
                              );

    if ( pRegisterRsp->cause == blueAPI_CauseSuccess )
    {
        /* save application handle */


        /* init. service specific data */
        gattTestServiceInitData( pGattTest );

    }
    else
    {

    }

    return ( true );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_ReleaseRsp
 * --------------------------------------------------------------------------*/

static bool test_upperstack_Handle_ReleaseRsp( PGATTTest            pGattTest,
        PBlueAPI_ReleaseRsp  pReleaseRsp )
{
    test_upperstack_CmdPrint( pGattTest, "<-- ReleaseRsp: cause=%s\r\n",
                              blueAPI_CauseString(pReleaseRsp->cause)
                              );


    return ( true );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_ActInfo
 * --------------------------------------------------------------------------*/

static bool test_upperstack_Handle_ActInfo( PGATTTest pGattTest, PBlueAPI_ActInfo pActInfo )
{
    test_upperstack_CmdPrint( pGattTest,
                              "<-- ActInfo: cause=%s, version=%s, BD=%s\r\n",
                              blueAPI_CauseString(pActInfo->cause),
                              pActInfo->version,
                              blueAPI_StringizeBd(pActInfo->local_BD)
                              );

    return ( true );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_DeviceConfigSetRsp
 * --------------------------------------------------------------------------*/

static void test_upperstack_Handle_DeviceConfigSetRsp( PGATTTest pGattTest, PBlueAPI_DeviceConfigSetRsp pDevCfgSetRsp )
{
    test_upperstack_CmdPrint(pGattTest, "<-- DeviceConfigSetRsp: opCode=%d cause=%s\r\n",
                             pDevCfgSetRsp->opCode,
                             blueAPI_CauseString(pDevCfgSetRsp->cause)
                             );
}


static bool test_upperstack_Handle_CreateMDLInd( PGATTTest pGattTest,
        PBlueAPI_CreateMDLInd pCreateMDLInd )
{
    TBlueAPI_Cause cause;
    PGATTLink pLink;
    test_upperstack_CmdPrint( pGattTest, "<-- CreateMDLInd: remote_BD=[%s], remote_BD_type=%d, local_MDL_ID=0x%04x\r\n",
                              blueAPI_StringizeBd(pCreateMDLInd->remote_BD),
                              pCreateMDLInd->remote_BD_type,
                              pCreateMDLInd->local_MDL_ID
                              );
    pLink = test_upperstack_LinkFindByBD(pGattTest, pCreateMDLInd->remote_BD);
    if (pLink)
    {
        if ( pLink->local_MDL_ID_Valid)
        {
            cause = blueAPI_CauseReject;
            test_upperstack_CmdPrint( pGattTest,
                                      "!! reject, the connection is already exist.\r\n");
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
        pLink = test_upperstack_LinkAllocate(pGattTest, pCreateMDLInd->remote_BD);
        if (pLink)
        {
            cause = blueAPI_CauseAccept;
            pLink->local_MDL_ID_Valid = true;
            pLink->local_MDL_ID       = pCreateMDLInd->local_MDL_ID;
        }
        else
        {
            cause = blueAPI_CauseReject;
            test_upperstack_CmdPrint( pGattTest,
                                      "!! reject, no rescource.\r\n" );
        }
    }


    if ( blueAPI_CreateMDLConf( pCreateMDLInd->local_MDL_ID,
                                1,    /* XXXXMJMJ maxTPDUusCredits */
                                cause )
       )
    {
        test_upperstack_CmdPrint( pGattTest, "--> CreateMDLConf: local_MDL_ID=0x%04x, cause=%s\r\n",
                                  pCreateMDLInd->local_MDL_ID,
                                  blueAPI_CauseString(cause)
                                  );
    }
    return ( true );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_ConnectMDLRsp
 * --------------------------------------------------------------------------*/

static bool test_upperstack_Handle_ConnectMDLRsp( PGATTTest pGattTest,
        PBlueAPI_ConnectMDLRsp pConnectMDLRsp )
{
    test_upperstack_CmdPrint( pGattTest, "<-- ConnectMDLRsp: cause=%s\r\n",
                              blueAPI_CauseString(pConnectMDLRsp->cause)
                              );

    return ( true );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_MCLStatusInfo
 * --------------------------------------------------------------------------*/

static bool test_upperstack_Handle_MCLStatusInfo( PGATTTest pGattTest,
        PBlueAPI_MCLStatusInfo pMCLStatusInfo )
{
    test_upperstack_CmdPrint( pGattTest, "<-- MCLStatusInfo: status=%s, local_MCL_ID=0x%04x\r\n",
                              blueAPI_MCLStatusString(pMCLStatusInfo->status),
                              pMCLStatusInfo->local_MCL_ID
                              );

    return ( true );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_ConnectMDLInfo
 * --------------------------------------------------------------------------*/

static bool test_upperstack_Handle_ConnectMDLInfo( PGATTTest pGattTest,
        PBlueAPI_ConnectMDLInfo pConnectMDLInfo )
{
    PGATTLink pLink;
    test_upperstack_CmdPrint( pGattTest, "<-- ConnectMDLInfo: dsPoolID=%04x, local_MDL_ID=0x%04x\r\n",
                              pConnectMDLInfo->dsPoolID,
                              pConnectMDLInfo->local_MDL_ID
                              );
    pLink = test_upperstack_LinkFindByLocal_MDL_ID(pGattTest, pConnectMDLInfo->local_MDL_ID);
    if (pLink)
    {
        pLink->Connected     = true;
        pLink->wMTUSize      = pConnectMDLInfo->maxTPDUSize;
        pLink->wDsCredits    = pConnectMDLInfo->maxTPDUdsCredits;
        if (pLink->role != GATTDEMO_CON_ROLE_MASTER)
            pLink->role = GATTDEMO_CON_ROLE_SLAVE;
    }

    pGattTest->wDsPoolId     = pConnectMDLInfo->dsPoolID;
    pGattTest->wDsDataOffset = pConnectMDLInfo->dsDataOffset;
    pGattTest->wUpdReqHandle = 0;

#if (GATTDEMO_CCCD_COUNT)
    /* init. array of CCCD attribute index/value pairs */
    test_upperstack_CCCDInitTable( pGattTest );
#endif

    if ( pGattTest->ConnectedInd != NULL )
    {
        (*pGattTest->ConnectedInd)( (void *)pGattTest );
    }

    return ( true );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_DisconnectMDLRsp
 * --------------------------------------------------------------------------*/

static bool test_upperstack_Handle_DisconnectMDLRsp( PGATTTest pGattTest,
        PBlueAPI_DisconnectMDLRsp pDisconnectMDLRsp )
{
    test_upperstack_CmdPrint( pGattTest, "<-- DisconnectMDLRsp: cause=%s, local_MDL_ID=0x%04x\r\n",
                              blueAPI_CauseString(pDisconnectMDLRsp->cause),
                              pDisconnectMDLRsp->local_MDL_ID
                              );

    return ( true );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_DisconnectMDLInd
 * --------------------------------------------------------------------------*/

static bool test_upperstack_Handle_DisconnectMDLInd( PGATTTest pGattTest,
        PBlueAPI_DisconnectMDLInd pDisconnectMDLInd )
{
    test_upperstack_CmdPrint( pGattTest, "<-- DisconnectMDLInd: cause=%s, local_MDL_ID=0x%04x\r\n",
                              blueAPI_CauseString(pDisconnectMDLInd->cause),
                              pDisconnectMDLInd->local_MDL_ID
                              );

    if ( blueAPI_DisconnectMDLConf( pDisconnectMDLInd->local_MDL_ID )
       )
    {
        test_upperstack_CmdPrint( pGattTest, "--> DisconnectMDLConf: local_MDL_ID=0x%04x\r\n",
                                  pDisconnectMDLInd->local_MDL_ID
                                  );
    }
    return ( true );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_DeleteMDLInfo
 * --------------------------------------------------------------------------*/

static bool test_upperstack_Handle_DeleteMDLInfo( PGATTTest pGattTest,
        PBlueAPI_DeleteMDLInfo pDeleteMDLInfo )
{
    PGATTLink pLink;
    test_upperstack_CmdPrint( pGattTest, "<-- DeleteMDLInfo: local_MDL_ID=0x%04x\r\n",
                              pDeleteMDLInfo->local_MDL_ID
                              );
    pLink = test_upperstack_LinkFindByLocal_MDL_ID(pGattTest, pDeleteMDLInfo->local_MDL_ID);
    /* cleanup connection data */
    if (pLink)
    {
        pLink->Connected          = false;
        pLink->local_MDL_ID_Valid = false;
        pLink->local_MDL_ID       = 0;
        pLink->role = GATTDEMO_CON_ROLE_UNDEFINED;
		test_upperstack_LEChanClear(pGattTest,pLink);
    }
    #if (GATTDEMO_PREPARE_WRITE_SUPPPRT)
    if (pDeleteMDLInfo->local_MDL_ID == pGattTest->queue_local_MDL_ID)
    {
  	  	pGattTest->queueIdx = 0;
  	  	pGattTest->queue_local_MDL_ID = 0;
    }
    #endif

    if (pGattTest->advertiseOnDisc)
    {
        test_upperstack_SendLEAdvertiseReq(pGattTest, blueAPI_LEAdvModeEnabled,
                                           NULL,
                                           blueAPI_RemoteBDTypeLEPublic);
    }

    return ( true );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_InternalEventInfo
 * --------------------------------------------------------------------------*/

static bool test_upperstack_Handle_InternalEventInfo( PGATTTest pGattTest,
        PBlueAPI_InternalEventInfo pInternalEventInfo )
{
    test_upperstack_CmdPrint( pGattTest, "<-- InternalEventInfo: Type=%s, Info=0x%08x, cause=%s\r\n",
                              blueAPI_InternalEventTypeString(pInternalEventInfo->eventType),
                              pInternalEventInfo->eventInfo,
                              blueAPI_CauseString(pInternalEventInfo->cause)
                              );

    return ( true );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_ACLStatusInfo
 * --------------------------------------------------------------------------*/

static bool test_upperstack_Handle_ACLStatusInfo( PGATTTest pGattTest,
        PBlueAPI_ACLStatusInfo pACLStatusInfo )
{
    test_upperstack_CmdPrint( pGattTest, "<-- ACLStatusInfo: BD=[%s] BDType=[%d] status=[%s] ",
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
        test_upperstack_CmdPrint( pGattTest, "keyType=[0x%04x] keySize=[%d]\r\n",
                                  pACLStatusInfo->p.auth.keyType,
                                  pACLStatusInfo->p.auth.keySize
                                  );
        break;

    case blueAPI_ACLAddressResolved:
        test_upperstack_CmdPrint( pGattTest, "resolvedBD=[%s] resolvedBDType=[%d]\r\n",
                                  blueAPI_StringizeBd(pACLStatusInfo->p.resolve.remote_BD),
                                  pACLStatusInfo->p.resolve.remote_BD_type
                                  );
        break;

    case blueAPI_ACLConnectedLinkStatus:
        test_upperstack_CmdPrint( pGattTest, "qual=[%d] rssi=[%d] failedC=[%d] txpwr=[%d] arssi=[%d]\r\n",
                                  pACLStatusInfo->p.linkStatus.linkQuality,
                                  pACLStatusInfo->p.linkStatus.rssi,
                                  pACLStatusInfo->p.linkStatus.failedContacts,
                                  pACLStatusInfo->p.linkStatus.txPower,
                                  pACLStatusInfo->p.linkStatus.absoluteRssi
                                  );
        break;

    default:
        test_upperstack_CmdPrint( pGattTest, "\r\n" );
        break;
    }

    return ( true );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_GATTServiceRegisterRsp
 * --------------------------------------------------------------------------*/

static bool test_upperstack_Handle_GATTServiceRegisterRsp(
    PGATTTest                       pGattTest,
    PBlueAPI_GATTServiceRegisterRsp pGATTServiceRegisterRsp )
{
    test_upperstack_CmdPrint( pGattTest, "<-- GATTServiceRegisterRsp: service=0x%x, cause=%s\r\n",
                              pGATTServiceRegisterRsp->serviceHandle,
                              blueAPI_CauseString(pGATTServiceRegisterRsp->cause)
                              );

    if ( pGATTServiceRegisterRsp->cause == blueAPI_CauseSuccess )
    {
        TTestParseResult  ParseResult;

        /* save stack supplied service handle */
        pGattTest->Service[pGattTest->iServiceCount].pServiceHandle =
            pGATTServiceRegisterRsp->serviceHandle;
        pGattTest->iServiceCount++;

        if ( pGattTest->iServiceCount == GATTTEST_MAX_SERVICES )
        {
            pGattTest->initLEServer    = true;
            pGattTest->advertiseOnDisc = true;
            if (pGattTest->initLEServer)
            {
                test_upperstack_SendLEModifyWhitelistReq(pGattTest,
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
            test_upperstack_ServiceRegister( pGattTest, &ParseResult );
        }
    }
    else
    {
        pGattTest->Service[pGattTest->iServiceCount].Used           = false;
        pGattTest->Service[pGattTest->iServiceCount].pServiceHandle = NULL;
    }

    return ( true );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_GATTAttributeUpdateRsp
 * --------------------------------------------------------------------------*/

static bool test_upperstack_Handle_GATTAttributeUpdateRsp(
    PGATTTest                       pGattTest,
    PBlueAPI_GATTAttributeUpdateRsp pGATTAttributeUpdateRsp )
{
    bool  Error = false;
    
#if 0
    test_upperstack_CmdPrint( pGattTest,
                              "<-- GATTAttributeUpdateRsp: subCause=0x%x(%s), reqHandle=0x%x\r\n",
                              pGATTAttributeUpdateRsp->subCause,
                              test_upperstack_SubCause2Str(pGATTAttributeUpdateRsp->subCause),
                              pGATTAttributeUpdateRsp->requestHandle
                              );
#endif

    if ( pGATTAttributeUpdateRsp->cause == blueAPI_CauseSuccess )
    {
        pGattTest->iUpdateConfirmed++;
        if ( pGattTest->iUpdateCnt > pGattTest->iUpdateSent )
        {
            /* notification request accepted, continue with next one .. */
            if ( test_upperstack_AttribUpdateContinue( pGattTest ) < 0 )
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
            test_upperstack_CmdPrint( pGattTest, " BD=[%s], type=0x%x\r\n",
                                      blueAPI_StringizeBd(pElement->remote_BD),
                                      pElement->remote_BD_Type
                                      );
            pElement++;
        }
    }

    if ( Error )
    {
        test_upperstack_CmdPrint( pGattTest, " Error=0x%x\r\n",
                                      pGATTAttributeUpdateRsp->subCause
                                      );

        pGattTest->iUpdateCnt = 0;
    }
    else
    {
        /* some services need to do some post update (sequence) processing (RACP ..): */
        if ( (pGattTest->iUpdateCnt == pGattTest->iUpdateConfirmed) ||
                (pGATTAttributeUpdateRsp->cause != blueAPI_CauseSuccess)
           )
        {
            gattTestServiceUpdateCallback( pGattTest,
                                           pGATTAttributeUpdateRsp->subCause, pGATTAttributeUpdateRsp->attribIndex );
        }
    }

    return ( true );
}


/*----------------------------------------------------------------------------
 * handle blueAPI_Event_GATTAttributeUpdateStatusInd
 * --------------------------------------------------------------------------*/

static bool test_upperstack_Handle_GATTAttributeUpdateStatusInd(
    PGATTTest                             pGattTest,
    PBlueAPI_GATTAttributeUpdateStatusInd pGATTAttributeUpdateStatusInd )
{
    test_upperstack_CmdPrint( pGattTest,
                              "<-- GATTAttributeUpdateStatusInd: subCause=0x%x(%s), reqHandle=0x%x, BD=[%s]\r\n",
                              pGATTAttributeUpdateStatusInd->subCause,
                              test_upperstack_SubCause2Str(pGATTAttributeUpdateStatusInd->subCause),
                              pGATTAttributeUpdateStatusInd->requestHandle,
                              blueAPI_StringizeBd(pGATTAttributeUpdateStatusInd->remote_BD)
                              );

    if ( blueAPI_GATTAttributeUpdateStatusConf(
            pGATTAttributeUpdateStatusInd->serviceHandle,
            pGATTAttributeUpdateStatusInd->requestHandle,
            pGATTAttributeUpdateStatusInd->attribIndex
                                              )
       )
    {
        test_upperstack_CmdPrint( pGattTest, "--> GATTAttributeUpdateStatusConf\r\n");
    }

    /* check if update sequence should be continued */
    if ( pGATTAttributeUpdateStatusInd->cause == blueAPI_CauseSuccess )
    {
        pGattTest->iUpdateConfirmed++;
        if ( pGattTest->iUpdateCnt > pGattTest->iUpdateSent )
        {
            test_upperstack_AttribUpdateContinue( pGattTest );
        }
        else
        {
        	DBG_BUFFER(MODULE_APP, LEVEL_INFO, "TX throughput test end\n\n", 0);
        }
    }
    else
    {
        pGattTest->iUpdateCnt = 0;
    }

#if 0
    /* some services need to do some post update (sequence) processing (RACP ..): */
    if ( pGATTDemo->iUpdateCnt == 0 )
    {
        gattTestServiceUpdateCallback( pGATTDemo,
                                       pGATTAttributeUpdateStatusInd->subCause,
                                       pGATTAttributeUpdateStatusInd->attribIndex );
    }
#endif
    return ( true );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_GATTAttributeReadInd
 * --------------------------------------------------------------------------*/

static bool test_upperstack_Handle_GATTAttributeReadInd(
    PGATTTest                     pGattTest,
    PBlueAPI_GATTAttributeReadInd pGATTAttributeReadInd )
{
    PGATTDService  pService;
    uint8_t *      pBuffer;
    uint16_t       wLength = 0;
    uint16_t       wOffset = pGattTest->wDsDataOffset;
    uint8_t *      pData   = NULL;
    uint16_t       wCause  = GATT_ERR_ILLEGAL_PARAMETER;

    test_upperstack_CmdPrint( pGattTest,
                              "<-- GATTAttributeReadInd: service=0x%x, idx=%d, offset=%d\r\n",
                              pGATTAttributeReadInd->serviceHandle,
                              pGATTAttributeReadInd->attribIndex,
                              pGATTAttributeReadInd->readOffset
                              );

    pService = gattTestServiceFind( pGattTest, pGATTAttributeReadInd->serviceHandle );
    if ( pService != (PGATTDService)NULL )
    {
        /* service/attribute specific read operation.                          */
        /* check for authorization etc. has been done by GATT layer already .. */
        wCause  = gattTestAttribGet( pGattTest, pService, pGATTAttributeReadInd->attribIndex,
                                     pGATTAttributeReadInd->readOffset, &wLength, &pData );
    }

    /* send response */
    if ( wCause == GATT_SUCCESS )
    {
        /* copy attribute value to buffer position that allows re-usage by stack */
        /* without copying ..                                                   */
        if ( blueAPI_BufferGet(
                    pGattTest->wDsPoolId,
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
        test_upperstack_CmdPrint( pGattTest,
                                  "--> GATTAttributeReadConf: service=0x%x, subCause=%s, length=%d\r\n",
                                  pGATTAttributeReadInd->serviceHandle,
                                  test_upperstack_SubCause2Str(wCause),
                                  wLength
                                  );
    }
    else
    {
        test_upperstack_CmdPrint( pGattTest,
                                  "!!! illegal parameter (e.g. offset too small)\r\n");
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
 #if (GATTDEMO_PREPARE_WRITE_SUPPPRT)
static bool test_upperstack_Handle_GATTAttributePrepareWriteInd(
						PGATTTest 					 pGattTest,
					    TBlueAPI_Command				 Command,
						PBlueAPI_GATTAttributeWriteInd pGATTAttributeWriteInd )
{
  	uint16_t		 wCause  = GATT_SUCCESS;
  	uint8_t *      pBuffer;
  	uint16_t       wLength = 0;
  	uint16_t       wOffset = pGattTest->wDsDataOffset;
 	uint8_t *      pData   = NULL;
  	PGATTPrepareWrite pPrepare = NULL;

  	test_upperstack_CmdPrint( pGattTest,
                     "<--GATTAttributePrepareWriteInd : service=0x%x, idx=%d, length=%d valueOffset=%d\r\n",
                     pGATTAttributeWriteInd->serviceHandle,
                     pGATTAttributeWriteInd->attribIndex,
                     pGATTAttributeWriteInd->attribLength,
                     pGATTAttributeWriteInd->writeOffset
                     );

  	wLength = pGATTAttributeWriteInd->attribLength;
  	pData = pGATTAttributeWriteInd->data + pGATTAttributeWriteInd->gap;
  	if(pGattTest->queueIdx == 0)
  	{
  		pGattTest->queue_local_MDL_ID = pGATTAttributeWriteInd->local_MDL_ID;
  	}
  
  	if(pGattTest->queue_local_MDL_ID == pGATTAttributeWriteInd->local_MDL_ID)
  	{
    	if(pGattTest->queueIdx == GATTDEMO_MAX_PREPARE_QUEUE)
    	{
      		wCause = (ATT_ERR|ATT_ERR_PREPARE_QUEUE_FULL);
    	}
    	else
    	{
      		pPrepare = &pGattTest->prepareQueue[pGattTest->queueIdx];
	  		pPrepare->attribIndex = pGATTAttributeWriteInd->attribIndex;
	  		pPrepare->serviceHandle = pGATTAttributeWriteInd->serviceHandle;
	  		pPrepare->handle = pGATTAttributeWriteInd->handle;
	  		pPrepare->attribLength = pGATTAttributeWriteInd->attribLength;
	  		pPrepare->writeOffset = pGATTAttributeWriteInd->writeOffset;
	  		memcpy( pPrepare->data , pData, wLength );
	  		pGattTest->queueIdx++;	
    	}
  	}
  	else
  	{
    	wCause = (ATT_ERR|ATT_ERR_MIN_APPLIC_CODE);
  	}
  
  	test_upperstack_HexDump( pGattTest, "value=", wLength, pData);

  	if ( wCause == GATT_SUCCESS )
  	{
    	if ( blueAPI_BufferGet(
                              pGattTest->wDsPoolId,
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
		test_upperstack_CmdPrint( pGattTest,
						   "--> GATTAttributeWriteConf: service=0x%x, subCause=%s\r\n",
						   pGATTAttributeWriteInd->serviceHandle,
						   test_upperstack_SubCause2Str(wCause)
						   );
  	}
  	else
  	{
    	test_upperstack_CmdPrint( pGattTest,
                     "!!! illegal parameter (e.g. offset too small)\r\n");
    	if ( pBuffer != NULL )
      		blueAPI_BufferRelease(pBuffer);
  	}
		
  	return( true );
}
#endif

static bool test_upperstack_Handle_GATTAttributeWriteIndCommandInfo(
                          PGATTTest                      pGattTest,
                          TBlueAPI_Command               Command,
                          PBlueAPI_GATTAttributeWriteInd pGATTAttributeWriteInd )
{
    PGATTDService           pService;
    TGATTDWriteIndPostProc  WriteIndPostProc = NULL;
    uint16_t                wCause  = GATT_ERR_ILLEGAL_PARAMETER;
    static const char writeIndTxt[]         = "GATTAttributeWriteInd";
    static const char writeCommandInfoTxt[] = "GATTAttributeWriteCommandInfo";
    char * pTxt;

    if ( Command == blueAPI_EventGATTAttributeWriteCommandInfo )
        pTxt = (char *)writeCommandInfoTxt;
    else
        pTxt = (char *)writeIndTxt;


    test_upperstack_CmdPrint( pGattTest,
                              "<-- %s: service=0x%x, idx=%d, length=%d\r\n",
                              pTxt,
                              pGATTAttributeWriteInd->serviceHandle,
                              pGATTAttributeWriteInd->attribIndex,
                              pGATTAttributeWriteInd->attribLength
                              );

    pService = gattTestServiceFind( pGattTest, pGATTAttributeWriteInd->serviceHandle );
    if ( pService != (PGATTDService)NULL )
    {
        /* service/attribute specific write operation.                         */
        /* check for authorization etc. has been done by GATT layer already .. */
        wCause  = gattTestAttribPut(
                      pGattTest, pService, pGATTAttributeWriteInd->attribIndex,
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
            test_upperstack_CmdPrint( pGattTest,
                                      "--> GATTAttributeWriteConf: service=0x%x, subCause=%s\r\n",
                                      pGATTAttributeWriteInd->serviceHandle,
                                      test_upperstack_SubCause2Str(wCause)
                                      );
        }
    }

    /* for services with RACP some post processing may be needed: */
    if ( WriteIndPostProc != 0 )
    {
        (*WriteIndPostProc)( pGattTest,
                             pGATTAttributeWriteInd->attribLength,
                             pGATTAttributeWriteInd->data + pGATTAttributeWriteInd->gap );
    }

    return ( true );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_GATTCCCDInfo
 * --------------------------------------------------------------------------*/

static bool test_upperstack_Handle_GATTCCCDInfo( PGATTTest             pGattTest,
        PBlueAPI_GATTCCCDInfo pGATTCCCDInfo )
{
    PGATTDService pService;

    test_upperstack_CmdPrint( pGattTest,
                              "<-- GATTCCCDInfo: service=0x%x, count=%d\r\n",
                              pGATTCCCDInfo->serviceHandle,
                              pGATTCCCDInfo->count
                              );

    pService = gattTestServiceFind( pGattTest, pGATTCCCDInfo->serviceHandle );
    if ( (pService != (PGATTDService)NULL) &&
            ((pService->ServiceID == blueAPI_ServiceApplicationDefined) ||
             (pService->ServiceID == blueAPI_ServiceGLS))
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
            test_upperstack_CmdPrint( pGattTest, "  attribIndex=%d, cccBits=0x%04x\r\n",
                                      wAttribIndex, wCCCBits
                                      );
#if (GATTDEMO_CCCD_COUNT)
            /* check if CCCD should be stored */
            {
                int  j;

                for ( j = 0; j < GATTDEMO_CCCD_COUNT; j++ )
                {
                    if ( pGattTest->AttrIdxCCCD[ j ].wAttribIndex == wAttribIndex )
                    {
                        pGattTest->AttrIdxCCCD[ j ].wCCCBits = wCCCBits;
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

static bool test_upperstack_Handle_GATTDiscoveryRsp(PGATTTest                 pGattTest,
        PBlueAPI_GATTDiscoveryRsp pGATTDiscoveryRsp)
{
    test_upperstack_CmdPrint( pGattTest, "<-- GATTDiscoveryRsp: subCause=0x%x(%s)\r\n",
                              pGATTDiscoveryRsp->subCause,
                              test_upperstack_SubCause2Str(pGATTDiscoveryRsp->subCause)
                              );

    return ( true );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_GATTDiscoveryInd
 * --------------------------------------------------------------------------*/

static bool test_upperstack_Handle_GATTDiscoveryInd(PGATTTest                 pGattTest,
        PBlueAPI_GATTDiscoveryInd pGATTDiscoveryInd)
{
    int        i, j;
    uint16_t   wEndingHandle = 0;   /* used for continuation search */
    PGATTLink pLink;
    test_upperstack_CmdPrint( pGattTest,
                              "<-- GATTDiscoveryInd: subCause=0x%x(%s), type=%d, count=%d\r\n",
                              pGATTDiscoveryInd->subCause,
                              test_upperstack_SubCause2Str(pGATTDiscoveryInd->subCause),
                              pGATTDiscoveryInd->discoveryType,
                              pGATTDiscoveryInd->elementCount
                              );
    pLink = test_upperstack_LinkFindByLocal_MDL_ID(pGattTest, pGATTDiscoveryInd->local_MDL_ID);
    if (pLink == NULL)
    {
        test_upperstack_CmdPrint(pGattTest, "the Local_MDL_ID is not valid\r\n");
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
                    test_upperstack_CmdPrint( pGattTest,
                                              " range: 0x%04x-0x%04x, service=<0x%04x> (%s)\r\n",
                                              pElement16->attHandle,
                                              pElement16->endGroupHandle,
                                              pElement16->UUID16,
                                              test_upperstack_UUID16ToString( pElement16->UUID16 )
                                              );
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
                    test_upperstack_CmdPrint( pGattTest,
                                              " range: 0x%04x-0x%04x, service=<0x%08x%08x%08x%08x>\r\n",
                                              pElement128->attHandle,
                                              pElement128->endGroupHandle,
                                              dwUUID128[3], dwUUID128[2], dwUUID128[1], dwUUID128[0]
                                              );

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
                    test_upperstack_CmdPrint( pGattTest,
                                              " range: 0x%04x-0x%04x\r\n",
                                              pElementUUID->attHandle,
                                              pElementUUID->endGroupHandle
                                              );
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
                    test_upperstack_CmdPrint( pGattTest,
                                              " decl=0x%04x, range: 0x%04x-0x%04x, service=<0x%04x> (%s)\r\n",
                                              pElement16->declHandle,
                                              pElement16->attHandle,
                                              pElement16->endGroupHandle,
                                              pElement16->UUID16,
                                              test_upperstack_UUID16ToString( pElement16->UUID16 )
                                              );
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
                    test_upperstack_CmdPrint( pGattTest,
                                              " decl=0x%04x, range: 0x%04x-0x%04x, service=<0x%08x%08x%08x%08x>\r\n",
                                              pElement128->declHandle,
                                              pElement128->attHandle,
                                              pElement128->endGroupHandle,
                                              dwUUID128[3], dwUUID128[2], dwUUID128[1], dwUUID128[0]
                                              );

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
                        test_upperstack_CmdPrint( pGattTest,
                                                  " !! get 128 bit UUID with command \"read x%04x\" !!\r\n",
                                                  pElement128->attHandle
                                                  );
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
                    test_upperstack_CmdPrint( pGattTest,
                                              " decl hndl=0x%04x, prop=0x%02x, value hndl=0x%04x, UUID=<0x%04x> (%s)\r\n",
                                              pElement16->declHandle,
                                              pElement16->properties,
                                              pElement16->valueHandle,
                                              pElement16->UUID16,
                                              test_upperstack_UUID16ToString( pElement16->UUID16 )
                                              );
                    wEndingHandle = pElement16->declHandle;

                    /* save characteristic value handle/UUID in local array */
                    test_upperstack_HandleUUIDSave( pGattTest, pLink->idx,
                                                    pElement16->valueHandle, pElement16->UUID16 );

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
                    test_upperstack_CmdPrint( pGattTest,
                                              " decl hndl=0x%04x, prop=0x%02x, value hndl=0x%04x, UUID=<0x%08x%08x%08x%08x>\r\n",
                                              pElement128->declHandle,
                                              pElement128->properties,
                                              pElement128->valueHandle,
                                              dwUUID128[3], dwUUID128[2], dwUUID128[1], dwUUID128[0]
                                              );

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
                    test_upperstack_CmdPrint( pGattTest,
                                              " handle=0x%04x, UUID=<0x%04x> (%s)\r\n",
                                              pElement16->handle,
                                              pElement16->UUID16,
                                              test_upperstack_UUID16ToString( pElement16->UUID16 )
                                              );
                    wEndingHandle = pElement16->handle;

                    /* save characteristic descriptor handle/UUID in local array */
                    test_upperstack_HandleUUIDSave( pGattTest, pLink->idx,
                                                    pElement16->handle, pElement16->UUID16 );

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
                    test_upperstack_CmdPrint( pGattTest,
                                              " handle=0x%04x, UUID=<0x%08x%08x%08x%08x>\r\n",
                                              pElement128->handle,
                                              dwUUID128[3], dwUUID128[2], dwUUID128[1], dwUUID128[0]
                                              );

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
        test_upperstack_CmdPrint( pGattTest,
                                  "idx= %d--> GATTDiscoveryConf: start=0x%04x, end=0x%04x\r\n",
                                  pLink->idx, 0,
                                  pLink->wEndingHandle
                                  );
    }

    return ( true );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_GATTAttributeReadRsp
 * --------------------------------------------------------------------------*/

static bool test_upperstack_Handle_GATTAttributeReadRsp(PGATTTest  pGattTest,
        PBlueAPI_GATTAttributeReadRsp pGATTAttributeReadRsp)
{
    PGATTLink pLink;
    test_upperstack_CmdPrint( pGattTest, "<-- GATTAttributeReadRsp: subCause=0x%x(%s), length=%d\r\n",
                              pGATTAttributeReadRsp->subCause,
                              test_upperstack_SubCause2Str(pGATTAttributeReadRsp->subCause),
                              pGATTAttributeReadRsp->totalLength
                              );
    pLink = test_upperstack_LinkFindByLocal_MDL_ID(pGattTest, pGATTAttributeReadRsp->local_MDL_ID);
    if (pLink == NULL)
    {
        test_upperstack_CmdPrint( pGattTest, "Error: local_MDL_ID is not valid\r\n");
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
                test_upperstack_AttribDisplay( pGattTest, pLink->idx, *pHandle, iValueSize, pValue );
                pHandle++;
                pValue += iValueSize;
            }
        }
    }

  return( true );
}
#if 0
static bool test_upperstack_Handle_GATTAttributeReadMultipleRsp(PGATTTest  pGattTest,
                                  PBlueAPI_GATTAttributeReadMultipleRsp pGATTAttributeReadMultiRsp)
{
  	PGATTLink pLink;
  	test_upperstack_CmdPrint( pGattTest, "<-- GATTAttributeReadMultipleRsp: subCause=0x%x(%s), length=%d\r\n",
                            pGATTAttributeReadMultiRsp->subCause,
                            test_upperstack_SubCause2Str(pGATTAttributeReadMultiRsp->subCause),
                            pGATTAttributeReadMultiRsp->totalLength
                            );
  	pLink = test_upperstack_LinkFindByLocal_MDL_ID(pGattTest,pGATTAttributeReadMultiRsp->local_MDL_ID);
  	if(pLink == NULL)
  	{
    	test_upperstack_CmdPrint( pGattTest, "Error: local_MDL_ID is not valid\r\n");
	 	return (true);
  	}
  
  	if ( pGATTAttributeReadMultiRsp->subCause == GATT_SUCCESS )
  	{
    	if ( pGATTAttributeReadMultiRsp->totalLength > 0 )
    	{
      		int length = pGATTAttributeReadMultiRsp->totalLength;
      		uint8_t  * pValue = pGATTAttributeReadMultiRsp->SetOfValues+pGATTAttributeReadMultiRsp->gap;
	  
	  		test_upperstack_HexDump( pGattTest, "value=", length, pValue );
    	}
  	}

    return ( true );
}
#endif

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_GATTAttributeWriteRsp
 * --------------------------------------------------------------------------*/

static bool test_upperstack_Handle_GATTAttributeWriteRsp(PGATTTest  pGattTest,
        PBlueAPI_GATTAttributeWriteRsp pGATTAttributeWriteRsp)
{
    test_upperstack_CmdPrint( pGattTest, "<-- GATTAttributeWriteRsp: subCause=0x%x(%s)\r\n",
                              pGATTAttributeWriteRsp->subCause,
                              test_upperstack_SubCause2Str(pGATTAttributeWriteRsp->subCause)
                              );

    return ( true );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_EventGATTAttributeInd and
 * blueAPI_EventGATTAttributeNotificationInfo
 * --------------------------------------------------------------------------*/
uint32_t gTotalNotificationCount = 0;

uint8_t gRxDisplayOn = 0;
uint8_t gRxPacketLen = 120;

static bool test_upperstack_Handle_GATTAttributeNotifInd(PGATTTest  pGattTest,
        TBlueAPI_Command Command,
        PBlueAPI_GATTAttributeInd pGATTAttributeInd)
{
    bool  Notify = (Command == blueAPI_EventGATTAttributeNotificationInfo);
    PGATTLink pLink;
    BOOL bRxError = FALSE;
#if 0
    test_upperstack_CmdPrint( pGattTest, "<-- GATTAttributeInd: notif=%d, handle=0x%x\r\n",
                              Notify,
                              pGATTAttributeInd->attribHandle
                              );
#endif
    gTotalNotificationCount++;
    pLink = test_upperstack_LinkFindByLocal_MDL_ID(pGattTest, pGATTAttributeInd->local_MDL_ID);
    if (pLink == NULL)
    {
        test_upperstack_CmdPrint(pGattTest, "error:local_MDL_ID is not valid");
        return (true);
    }
    /* service/attribute specific display of value: */
#if 1 
    static uint8_t iRxValue = 0;
		uint8_t i;
    for(i = 0; i<pGATTAttributeInd->attribLength; i++)
    {
        uint8_t RxTemp = *(pGATTAttributeInd->data + pGATTAttributeInd->gap + i);
        if(RxTemp != iRxValue)
        {
            bRxError = TRUE; 
        }
        iRxValue++;
    }
    
    if(gTotalNotificationCount == 10000||gRxDisplayOn||pGATTAttributeInd->attribLength != gRxPacketLen||bRxError)
    {
        test_upperstack_AttribDisplay(pGattTest, pLink->idx, pGATTAttributeInd->attribHandle,
                                  pGATTAttributeInd->attribLength,
                                  pGATTAttributeInd->data + pGATTAttributeInd->gap
                                 );
    }  

    bRxError = FALSE;
#endif
    if ( !Notify )
    {
        /* send response/ack */
        if ( blueAPI_GATTAttributeConf(pGATTAttributeInd->local_MDL_ID
                                      )
           )
        {
            test_upperstack_CmdPrint( pGattTest, "--> GATTAttributeConf\r\n");
        }
    }

    return ( true );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_EventGATTSecurityRsp
 * --------------------------------------------------------------------------*/
static void test_upperstack_Handle_GATTSecurityRsp(PGATTTest pGattTest,
        PBlueAPI_GATTSecurityRsp pSecurityRsp)
{
    test_upperstack_CmdPrint( pGattTest, "<-- GATTSecurityRsp: keyType=%d keySize=%d cause=%s\r\n",
                              pSecurityRsp->keyType,
                              pSecurityRsp->keySize,
                              blueAPI_CauseString(pSecurityRsp->cause)
                              );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_EventGATTServerStoreInd
 * --------------------------------------------------------------------------*/
static void test_upperstack_Handle_GATTServerStoreInd(PGATTTest pGattTest,
        PBlueAPI_GATTServerStoreInd pStoreInd)
{
    uint16_t        restart   = 0x0000;
    uint8_t         dataLen   = 0;
    uint8_t *       pData     = NULL;
    TBlueAPI_Cause  cause     = blueAPI_CauseReject;
#if (GATTDSTORE_ENTRY_COUNT)
    uint8_t       entryType = GATTDSTORE_FREE_ENTRY;
    bool          changed   = false;
#endif

    test_upperstack_CmdPrint( pGattTest, "<-- GATTServerStoreInd: opCode=%d bd=%s bdType=%d restart=0x%x dataLen=%d\r\n",
                              pStoreInd->opCode,
                              blueAPI_StringizeBd(pStoreInd->remote_BD),
                              pStoreInd->remote_BD_Type,
                              pStoreInd->restartHandle,
                              pStoreInd->dataLength
                              );

#if (GATTDSTORE_ENTRY_COUNT)
    switch (pStoreInd->opCode)
    {
    case blueAPI_GATTStoreOpGetCCCBits: /*----------------------------------*/
    case blueAPI_GATTStoreOpSetCCCBits: /*----------------------------------*/
    case blueAPI_GATTStoreOpGetAllCCCBits: /*-------------------------------*/
    case blueAPI_GATTStoreOpDeleteAllCCCBits: /*----------------------------*/
        entryType = GATTDSTORE_CCC_ENTRY;
        break;

    default:
        break;
    }

    switch (pStoreInd->opCode)
    {
    case blueAPI_GATTStoreOpGetCCCBits: /*----------------------------------*/
        {
            uint16_t idx;

            for (idx = 0; idx < GATTDSTORE_ENTRY_COUNT; idx++)
            {
                PGATTDStoreEntry pSearch = &pGattTest->extStore[idx];

                if ((pSearch->used == entryType) &&
                        (memcmp(pSearch->bd, pStoreInd->remote_BD, BLUE_API_BD_SIZE) == 0) &&
                        (pSearch->bdType == pStoreInd->remote_BD_Type)
                   )
                {
                    dataLen = pSearch->p.gatt.dataLen;
                    pData   = pSearch->p.gatt.data;
                    cause   = blueAPI_CauseSuccess;
                    break;
                }
            }
        }
        break;

    case blueAPI_GATTStoreOpSetCCCBits: /*----------------------------------*/
        {
            uint16_t idx;
            PGATTDStoreEntry pEntry = NULL;

            // TODO: check if a bond exists

            for (idx = 0; idx < GATTDSTORE_ENTRY_COUNT; idx++)
            {
                PGATTDStoreEntry pSearch = &pGattTest->extStore[idx];

                if ((pSearch->used == GATTDSTORE_FREE_ENTRY) &&
                        (pEntry == NULL)
                   )
                {
                    pEntry = pSearch;
                }
                else if ((pSearch->used == entryType) &&
                         (memcmp(pSearch->bd, pStoreInd->remote_BD, BLUE_API_BD_SIZE) == 0) &&
                         (pSearch->bdType == pStoreInd->remote_BD_Type)
                        )
                {
                    pEntry = pSearch;
                    break;
                }
            }

            if (pEntry)
            {
                if (pStoreInd->dataLength > 0)
                {
                    pEntry->used            = entryType;
                    memcpy(pEntry->bd, pStoreInd->remote_BD, BLUE_API_BD_SIZE);
                    pEntry->bdType          = pStoreInd->remote_BD_Type;
                    pEntry->p.gatt.dataLen  = pStoreInd->dataLength;
                    memcpy(pEntry->p.gatt.data, pStoreInd->data, pEntry->p.gatt.dataLen);
                }
                else
                {
                    pEntry->used = GATTDSTORE_FREE_ENTRY;
                }

                cause   = blueAPI_CauseSuccess;
                changed = true;
            }
        }
        break;

    case blueAPI_GATTStoreOpGetAllCCCBits: /*-------------------------------*/
        {
            uint16_t idx = pStoreInd->restartHandle;

            while (idx < GATTDSTORE_ENTRY_COUNT)
            {
                PGATTDStoreEntry pSearch = &pGattTest->extStore[idx++];

                if (pSearch->used == GATTDSTORE_CCC_ENTRY)
                {
                    memcpy(pStoreInd->remote_BD, pSearch->bd, BLUE_API_BD_SIZE);
                    pStoreInd->remote_BD_Type = pSearch->bdType;

                    restart = (idx < GATTDSTORE_ENTRY_COUNT) ? idx : 0x0000;
                    dataLen = pSearch->p.gatt.dataLen;
                    pData   = pSearch->p.gatt.data;
                    cause   = blueAPI_CauseSuccess;
                    break;
                }
            }

            cause = blueAPI_CauseSuccess;
        }
        break;

    case blueAPI_GATTStoreOpDeleteAllCCCBits: /*----------------------------*/
        {
            uint16_t idx;

            for (idx = 0; idx < GATTDSTORE_ENTRY_COUNT; idx++)
            {
                PGATTDStoreEntry pSearch = &pGattTest->extStore[idx];

                if (pSearch->used == GATTDSTORE_CCC_ENTRY)
                {
                    pSearch->used = GATTDSTORE_FREE_ENTRY;
                    changed       = true;
                }
            }

            cause   = blueAPI_CauseSuccess;
        }
        break;

    default: /*-------------------------------------------------------------*/
        break;
    }

#if (GATTDSTORE_NVRAM_ADDRESS)
    if (changed)
    {
        nvramUpdate(GATTDSTORE_NVRAM_ADDRESS, sizeof(pGattTest->extStore), (uint8_t *)&pGattTest->extStore);
    }
#endif /* (GATTDSTORE_NVRAM_ADDRESS) */
#endif /* (GATTDSTORE_ENTRY_COUNT) */


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
 * handle blueAPI_EventGATTMtuSizeInfo
 * --------------------------------------------------------------------------*/

static void test_upperstack_Handle_GATTMtuSizeInfo(PGATTTest pGattTest,
        PBlueAPI_GATTMtuSizeInfo  pMtuSizeInfo)
{
    PGATTLink pLink;
    test_upperstack_CmdPrint( pGattTest, "<-- MtuSizeInfo: mtuSize=0x%x\r\n",
                              pMtuSizeInfo->mtuSize);

    pLink = test_upperstack_LinkFindByLocal_MDL_ID(pGattTest, pMtuSizeInfo->local_MDL_ID);
    if (pLink)
        pLink->wMTUSize = pMtuSizeInfo->mtuSize;
    else
        test_upperstack_CmdPrint( pGattTest, "<-- test_upperstack_Handle_GATTMtuSizeInfo:not find the link\r\n");
}

/*----------------------------------------------------------------------------
 * handle blueAPI_EventLEAdvertiseRsp
 * --------------------------------------------------------------------------*/
static void test_upperstack_Handle_LEAdvertiseRsp(PGATTTest pGattTest,
        PBlueAPI_LEAdvertiseRsp pAdvertiseRsp)
{
    test_upperstack_CmdPrint( pGattTest, "<-- LEAdvertiseRsp: advMode=%d cause=%s\r\n",
                              pAdvertiseRsp->advMode,
                              blueAPI_CauseString(pAdvertiseRsp->cause)
                              );

    if (pGattTest->initLEServer)
    {
        if (pAdvertiseRsp->advMode == blueAPI_LEAdvModeEnabled)
        {
            pGattTest->initLEServer = false;
        }
    }

}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_LEAdvertiseParameterRsp
 * --------------------------------------------------------------------------*/
static void test_upperstack_Handle_LEAdvertiseParameterRsp(PGATTTest pGattTest,
        PBlueAPI_LEAdvertiseParameterSetRsp pAdvertiseParameterSetRsp)
{
    test_upperstack_CmdPrint( pGattTest, "<-- LEAdvertiseParameterRsp: cause=%s\r\n",
                              blueAPI_CauseString(pAdvertiseParameterSetRsp->cause)
                              );

    if (pGattTest->initLEServer)
    {
        test_upperstack_SendLEAdvertiseDataSetReq(pGattTest,
                blueAPI_LEDataTypeAdvertisingData,
                sizeof(bGattTestAdData),
                (uint8_t *)bGattTestAdData
                                                 );
    }

}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_LEAdvertiseDataSetRsp
 * --------------------------------------------------------------------------*/
static void test_upperstack_Handle_LEAdvertiseDataSetRsp(PGATTTest pGattTest,
        PBlueAPI_LEAdvertiseDataSetRsp pAdvertiseDataSetRsp)
{
    test_upperstack_CmdPrint( pGattTest, "<-- LEAdvertiseDataSetRsp: dataType=%d cause=%s\r\n",
                              pAdvertiseDataSetRsp->dataType,
                              blueAPI_CauseString(pAdvertiseDataSetRsp->cause)
                              );

    if (pGattTest->initLEServer)
    {
        if (pAdvertiseDataSetRsp->dataType == blueAPI_LEDataTypeAdvertisingData)
        {
            test_upperstack_SendLEAdvertiseDataSetReq(pGattTest,
                    blueAPI_LEDataTypeScanResponseData,
                    sizeof(bGattTestScanRespData),
                    (uint8_t *)bGattTestScanRespData
                                                     );
        }
        else
        {
            test_upperstack_SendLEAdvertiseReq(pGattTest,
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
static void test_upperstack_Handle_LEScanRsp(PGATTTest pGattTest,
        PBlueAPI_LEScanRsp pScanRsp)
{
    test_upperstack_CmdPrint( pGattTest, "<-- LEScanRsp: cause=%s\r\n",
                              blueAPI_CauseString(pScanRsp->cause)
                              );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_LEScanInfo
 * --------------------------------------------------------------------------*/
static void test_upperstack_Handle_LEScanInfo(PGATTTest pGattTest,
        PBlueAPI_LEScanInfo pScanInfo)
{
    uint8_t       buffer[32];
    uint8_t       pos        = 0;

    test_upperstack_CmdPrint( pGattTest, "<-- LEScanInfo: bd=%s bdtype=%d event=0x%x rssi=0x%x\r\n",
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

static void test_upperstack_Handle_LEModifyWhitelistRsp(PGATTTest pGattTest,
        PBlueAPI_LEModifyWhitelistRsp pModifyWhitelistRsp)
{
    test_upperstack_CmdPrint( pGattTest, "<-- LEModifyWhitelistRsp: op=%d cause=%s\r\n",
                              pModifyWhitelistRsp->operation,
                              blueAPI_CauseString(pModifyWhitelistRsp->cause)
                              );

    if (pGattTest->initLEServer)
    {
        if (pModifyWhitelistRsp->operation == blueAPI_LEWhitelistOpClear)
        {
            test_upperstack_SendLEModifyWhitelistReq(pGattTest,
                    blueAPI_LEWhitelistOpAdd,
                    pGattTest->linkTable[0].RemoteBd, //fixme later
                    blueAPI_RemoteBDTypeLEPublic
                                                    );
        }
        else
        {
            if ( blueAPI_LEAdvertiseParameterSetReq(blueAPI_LEAdvTypeUndirected,
                                                    blueAPI_LEFilterAny,
                                                    blueAPI_LEFilterAny,
                                                    0xA0, /* 20ms */
                                                    0xB0, /* 30ms */
                                                    blueAPI_LocalBDTypeLEPublic,
                                                    NULL,
                                                    blueAPI_RemoteBDTypeLEPublic
                                                   )
               )
            {
                test_upperstack_CmdPrint( pGattTest, "--> LEAdvertiseParameterSetReq\r\n");
            }
        }
    }

}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_LEConnectionUpdateRsp
 * --------------------------------------------------------------------------*/
static void test_upperstack_Handle_LEConnectionUpdateRsp(PGATTTest pGattTest,
        PBlueAPI_LEConnectionUpdateRsp pConnectionUpdateRsp)
{
    test_upperstack_CmdPrint( pGattTest, "<-- LEConnectionUpdateRsp: MDL=0x%04x cause=%s\r\n",
                              pConnectionUpdateRsp->local_MDL_ID,
                              blueAPI_CauseString(pConnectionUpdateRsp->cause)
                              );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_LEConnectionUpdateInd
 * --------------------------------------------------------------------------*/
static void test_upperstack_Handle_LEConnectionUpdateInd(PGATTTest pGattTest,
        PBlueAPI_LEConnectionUpdateInd pConnectionUpdateInd)
{
    test_upperstack_CmdPrint( pGattTest, "<-- LEConnectionUpdateInd: MDL=0x%04x Interval=%d-%d Latency=%d Supervision=%d\r\n",
                              pConnectionUpdateInd->local_MDL_ID,
                              pConnectionUpdateInd->connIntervalMin,
                              pConnectionUpdateInd->connIntervalMax,
                              pConnectionUpdateInd->connLatency,
                              pConnectionUpdateInd->supervisionTimeout
                              );
}

/*----------------------------------------------------------------------------
 * handle test_upperstack_Handle_LEConnectionParameterInfo
 * --------------------------------------------------------------------------*/
static void test_upperstack_Handle_LEConnectionParameterInfo(PGATTTest pGattTest,
        PBlueAPI_LEConnectionParameterInfo pConnectionParameterInfo)
{
    test_upperstack_CmdPrint( pGattTest, "<-- LEConnectionParameterInfo: MDL=0x%04x Interval=%d Latency=%d Supervision=%d\r\n",
                              pConnectionParameterInfo->local_MDL_ID,
                              pConnectionParameterInfo->connInterval,
                              pConnectionParameterInfo->connLatency,
                              pConnectionParameterInfo->supervisionTimeout
                              );
}

#if (GATTDEMO_PREPARE_WRITE_SUPPPRT)
static void test_upperstack_Handle_GATTAttributePrepareWriteRsp(PGATTTest pGattTest,
                                          PBlueAPI_GATTAttributePrepareWriteRsp pPrepareWriteRsp)
{
  	int length = pPrepareWriteRsp->attribLength;
  	int writeOffset = pPrepareWriteRsp->writeOffset;
  	uint8_t *value =  pPrepareWriteRsp->data + pPrepareWriteRsp->gap;
  	test_upperstack_CmdPrint( pGattTest, "<-- PrepareWriteRsp: cause=%s writeOffset=%d attribLength=%d\r\n",
                            blueAPI_CauseString((TBlueAPI_Cause)pPrepareWriteRsp->cause),
                            pPrepareWriteRsp->writeOffset,
                            pPrepareWriteRsp->attribLength
                            );
  	test_upperstack_HexDump( pGattTest, "value=", length, value );

  	writeOffset += length;
  	if(writeOffset<60)
  	{
    	if(writeOffset+18>60)
    	{
      		length = 60-writeOffset;
    	}
		else
		{
      		length = 18;
		}
		test_upperstack_SendPrepareWrite(pGattTest, pPrepareWriteRsp->local_MDL_ID,
		                  writeOffset,length); 
  	}
  	else
  	{
    	test_upperstack_SendExecuteWrite(pGattTest,pPrepareWriteRsp->local_MDL_ID,0x01);    
  	}
}

static void test_upperstack_Handle_GATTAttributeExecuteWriteRsp(PGATTTest pGattTest,
                                          PBlueAPI_GATTAttributeExecuteWriteRsp pExecuteWriteRsp)
{
  	test_upperstack_CmdPrint( pGattTest, "<-- ExecuteWriteRsp: cause=%s\r\n",
                            blueAPI_CauseString(pExecuteWriteRsp->cause)
                            );
}

static void test_upperstack_Handle_GATTAttributeExecuteWriteInd(PGATTTest pGattTest,
                                          PBlueAPI_GATTAttributeExecuteWriteInd pExecuteWriteInd)
{
  	uint16_t  wCause = blueAPI_CauseSuccess;
  	uint8_t         flags = pExecuteWriteInd->flags;
  	uint16_t        handle = 0;
  	test_upperstack_CmdPrint( pGattTest, "<-- ExecuteWriteInd: MDL=0x%04x flags=%d\r\n",
                            pExecuteWriteInd->local_MDL_ID,
                            pExecuteWriteInd->flags
                            );
  	if(pGattTest->queue_local_MDL_ID == pExecuteWriteInd->local_MDL_ID)
  	{
  		if(flags == 0)
  		{
      		pGattTest->queueIdx = 0;
	  		pGattTest->queue_local_MDL_ID = 0;
    	}
    	else
    	{
      		wCause = gattTestAttribPrepareWrite(pGattTest,pExecuteWriteInd->local_MDL_ID, &handle);
	  		pGattTest->queueIdx = 0;
	  		pGattTest->queue_local_MDL_ID = 0;
    	}
  	}
  	else
  		wCause = (ATT_ERR|ATT_ERR_MIN_APPLIC_CODE);
  
  	blueAPI_GATTAttributeExecuteWriteConf(pExecuteWriteInd->local_MDL_ID,
                                    (wCause != GATT_SUCCESS) ?
                                    blueAPI_CauseLowerLayerError : blueAPI_CauseSuccess,
                                    wCause,
                                    handle
                                   );
}
#endif
#if 0
static void test_upperstack_Handle_VendorSetVoicePara(PGATTTest pGattTest,
        PBlueAPI_VendorSetVoiceParaRsp pVendorSetVoiceParaRsp)
{
    test_upperstack_CmdPrint( pGattTest, "<-- VendorSetVoiceParaRsp: MDL=0x%04x \r\n",
                              pVendorSetVoiceParaRsp->local_MDL_ID,
                              blueAPI_CauseString(pVendorSetVoiceParaRsp->cause)
                              );

}
#endif
#ifdef RTL8762AX_VB
static void test_upperstack_Handle_SetBleTxPowerRsp(PGATTTest pGattTest,
        PBlueAPI_SetBleTxPowerRsp pSetBleTxPowerRsp)
{
    test_upperstack_CmdPrint( pGattTest, "<-- SetBleTxPowerRsp: cause=%s tx_power_index = %d subCause=0x%x\r\n",
                              blueAPI_CauseString(pSetBleTxPowerRsp->cause),
                              pSetBleTxPowerRsp->tx_power_index,
                              pSetBleTxPowerRsp->subCause
                              );

}
#endif

static void test_upperstack_Handle_SetRandomAddressRsp(PGATTTest pGattTest,
        PBlueAPI_SetRandomAddressRsp pRandomRsp)
{
    test_upperstack_CmdPrint( pGattTest, "<-- SetRandomAddressRsp: cause=%s subCause=0x%04x\r\n",
                              blueAPI_CauseString(pRandomRsp->cause),
                              pRandomRsp->subCause
                              );

}

static void test_upperstack_Handle_CreateLEDataChannelRsp(PGATTTest pGattTest,
                                          PBlueAPI_CreateLEDataChannelRsp pCreateChanRsp)
{
  	PGATTLink pLink;
  	PGATTLEChannel pChan;
  	test_upperstack_CmdPrint( pGattTest, "<-- CreateLEDataChannelRsp: local_MDL_ID 0x%04x channel 0x%04x cause=%s subCause=0x%04x\r\n",
  	                        pCreateChanRsp->local_MDL_ID,
  	                        pCreateChanRsp->channel,
                            blueAPI_CauseString(pCreateChanRsp->cause),
                            pCreateChanRsp->subCause
                            );
  	if(pCreateChanRsp->cause == blueAPI_CauseSuccess)
  	{
  		pLink = test_upperstack_LinkFindByLocal_MDL_ID(pGattTest,pCreateChanRsp->local_MDL_ID);
  		if(pLink)
  		{
    		pChan = test_upperstack_LEChanAllocate(pGattTest,pLink,pCreateChanRsp->channel);
			if(pChan)
			{
  				pChan->isDataChanConnected = true;
			}
			else{
				  	test_upperstack_CmdPrint( pGattTest, "<-- CreateLEDataChannelRsp: allocate failed\r\n");
				}
  		}
  	}
  
}
static void test_upperstack_Handle_CreateLEDataChannelInd(PGATTTest pGattTest,
                                          PBlueAPI_CreateLEDataChannelInd pCreateChanInd)
{
	PGATTLink pLink;
	PGATTLEChannel pChan;
	test_upperstack_CmdPrint( pGattTest, "<-- CreateLEDataChannelInd: local_MDL_ID 0x%04x channel 0x%04x \r\n",
							  pCreateChanInd->local_MDL_ID,
							  pCreateChanInd->channel
							  );
	pLink = test_upperstack_LinkFindByLocal_MDL_ID(pGattTest,pCreateChanInd->local_MDL_ID);
	if(pLink)
	{
	  	pChan = test_upperstack_LEChanAllocate(pGattTest,pLink,pCreateChanInd->channel);
	  	if(pChan)
	  	{
	    	pChan->isDataChanConnected = true;
	  		blueAPI_CreateLEDataChannelConf(pLink->local_MDL_ID,
	  	                              pCreateChanInd->channel,
	  	                              1050,
	  	                              46,
	  	                              8,
	  	                              4,
	  	                              blueAPI_CauseAccept);
  		
	  	}

	}
}
static void test_upperstack_Handle_DisconnectLEDataChannelRsp(PGATTTest pGattTest,
                                          PBlueAPI_DisconnectLEDataChannelRsp pDiscChanRsp)
{
	PGATTLink pLink;
	PGATTLEChannel pChan;
	test_upperstack_CmdPrint( pGattTest, "<-- DisconnectLEDataChannelRsp: local_MDL_ID 0x%04x channel 0x%04x cause=%d subcause=0x%04x\r\n",
							  pDiscChanRsp->local_MDL_ID,
							  pDiscChanRsp->channel,
							  pDiscChanRsp->cause,
							  pDiscChanRsp->subCause
							  );
	pLink = test_upperstack_LinkFindByLocal_MDL_ID(pGattTest,pDiscChanRsp->local_MDL_ID);
	if(pLink)
	{
	  	pChan = test_upperstack_LEChanFind(pGattTest,pLink,pDiscChanRsp->channel);
	  
	  	test_upperstack_LEChanRelease(pGattTest,pChan);
	}

}
static void test_upperstack_Handle_DisconnectLEDataChannelInd(PGATTTest pGattTest,
                                          PBlueAPI_DisconnectLEDataChannelInd pDiscChanInd)
{
	PGATTLink pLink;
	PGATTLEChannel pChan;
	test_upperstack_CmdPrint( pGattTest, "<-- DisconnectLEDataChannelInd: local_MDL_ID 0x%04x channel 0x%x subcause 0x%04x \r\n",
							  pDiscChanInd->local_MDL_ID,
							  pDiscChanInd->channel,
							  pDiscChanInd->subCause);
	pLink = test_upperstack_LinkFindByLocal_MDL_ID(pGattTest,pDiscChanInd->local_MDL_ID);
	if(pLink)
	{
	  	pChan = test_upperstack_LEChanFind(pGattTest,pLink,pDiscChanInd->channel);
	  	test_upperstack_LEChanRelease(pGattTest,pChan);
	  	blueAPI_DisconnectLEDataChannelConf(pLink->local_MDL_ID,pDiscChanInd->channel,
	  	                              blueAPI_CauseAccept);
	}

}

static void test_upperstack_Handle_SendLEFlowControlCreditRsp(PGATTTest pGattTest,
                                          PBlueAPI_SendLEFlowControlCreditRsp pFlowRsp)
{
	test_upperstack_CmdPrint( pGattTest, "<-- SendLEFlowControlCreditRsp: local_MDL_ID 0x%04x channel 0x%x cause=%d subcause 0x%04x\r\n",
							  pFlowRsp->local_MDL_ID,
							  pFlowRsp->channel,
							  pFlowRsp->cause,
							  pFlowRsp->subCause
							  );

}

static void test_upperstack_Handle_LEDataInd(PGATTTest pGattTest,
                                          PBlueAPI_LEDataInd pDataInd)
{
	int length = pDataInd->valueLength;
	uint8_t *value =  pDataInd->data + pDataInd->gap;

	gTotalNotificationCount++;

    if(gTotalNotificationCount == 10000)
    {
        //test_upperstack_HexDump( pGattTest, "value=", length, value );
    }

	test_upperstack_CmdPrint( pGattTest, "<-- LEDataInd: local_MDL_ID 0x%04x channel 0x%04x valueLength %d gap %d\r\n",
							  pDataInd->local_MDL_ID,
							  pDataInd->channel,
							  pDataInd->valueLength,
							  pDataInd->gap
							  );
	test_upperstack_HexDump( pGattTest, "value=", length, value );
	
	blueAPI_LEDataConf(pDataInd->local_MDL_ID,pDataInd->channel,
	  	                              blueAPI_CauseSuccess);
}
int test_upperstack_LEDataContinue( PGATTTest pGattTest, PGATTLEChannel pChan, uint16_t  local_MDL_ID )
{
    bool Result;
	int i;
  	uint8_t *      pBuffer;
  	uint8_t *      value;
  	uint16_t       wOffset = pGattTest->wDsDataOffset+1;
	int dataLength = pGattTest->iUpdateDataLength;
  	uint8_t test[1050] = {0};

	for(i = 0; i < pGattTest->iUpdateDataLength; i++)
    {
    	test[i] = gTestDataMax;
		gTestDataMax++;
    }
    value    = (uint8_t *)test;

  	if(pChan == NULL)
  		return (test_ResultError);

  	if(pChan->isDataChanConnected)
  	{
  		while((pGattTest->iUpdateCnt>pGattTest->iUpdateSent) && (pChan->maxDsCredit >0))
    	{  	
    		if(appDataRamPoolId_test != 0xFFFF)
			{
				pBuffer = blueAPI_DataRamPoolBufferGet(appDataRamPoolId_test, dataLength + wOffset);
			}
			else
			{
				if ( blueAPI_BufferGet(
                           		pGattTest->wDsPoolId,
                           		dataLength,
                           		wOffset,
                           		(void **)&pBuffer) != blueAPI_CauseSuccess )
				{
                	pBuffer = NULL;
				}
					}
       		if ( pBuffer != NULL)
       		{
       					   	 	
         		memcpy( pBuffer+wOffset, value, dataLength );
	     		Result = blueAPI_LEDataReq( pBuffer,local_MDL_ID,
										  pChan->channel, dataLength,wOffset);
				pChan->maxDsCredit--;
				pGattTest->iUpdateSent++;
       		}
	   		else 
	   		{
	   	 		test_upperstack_CmdPrint( pGattTest,
				   			"--> SendLEData:get buffer failed maxDsCredit %d\r\n",
				   			pChan->maxDsCredit);
				break;
	   		}
	   	}
    	
  	}
  
  	if (Result)
  	{
		/*test_upperstack_CmdPrint( pGattTest,
				   "--> SendLEData: local_MDL_ID=0x%04x, channel=0x%04x\r\n",
				   local_MDL_ID,
				   pChan->channel
				   );*/
  	}

  	return( Result ? test_ResultOk : test_ResultError );
}

static void test_upperstack_Handle_LEDataRsp(PGATTTest pGattTest,
                                          PBlueAPI_LEDataRsp pDataRsp)
{
	PGATTLink pLink;
	PGATTLEChannel pChan;
	bool Error = FALSE;
	/*test_upperstack_CmdPrint( pGattTest, "<-- LEDataRsp: local_MDL_ID 0x%04x channel 0x%04x cause %d subcause 0x%04x\r\n",
							  pDataRsp->local_MDL_ID,
							  pDataRsp->channel,
							  pDataRsp->cause,
							  pDataRsp->subCause
							  );*/

	if ( pDataRsp->cause == blueAPI_CauseSuccess )
    {
        pGattTest->iUpdateConfirmed++;
		pLink = test_upperstack_LinkFindByLocal_MDL_ID(pGattTest,pDataRsp->local_MDL_ID);
	    if(pLink)
	    {
	  	 	pChan = test_upperstack_LEChanFind(pGattTest,pLink,pDataRsp->channel);
			if(pChan != NULL)
			{
				pChan->maxDsCredit++;
				/*test_upperstack_CmdPrint( pGattTest, "<-- LEDataRsp: maxDsCredit %d\r\n",
							  pChan->maxDsCredit
							  );*/
				if ( pGattTest->iUpdateCnt > pGattTest->iUpdateSent )
        		{
            		if ( test_upperstack_LEDataContinue( pGattTest , pChan, pDataRsp->local_MDL_ID) != test_ResultOk )
            		{
                		Error = true;
            		}
        		}
			}
		}
        
    }
    else 
    {
        test_upperstack_CmdPrint( pGattTest, " Error=0x%x\r\n",
                                      pDataRsp->subCause
                                      );

        pGattTest->iUpdateCnt = 0;
    }
	if(Error)
	{
		test_upperstack_CmdPrint( pGattTest, "test_upperstack_LEDataContinue error\r\n"
                                      );
	}

}

static void test_upperstack_Handle_LEDataChannelParameterInfo(PGATTTest pGattTest,
                                          PBlueAPI_LEDataChannelParameterInfo pConInfo)
{
	PGATTLink pLink;
	PGATTLEChannel pChan;
	test_upperstack_CmdPrint( pGattTest, "<-- LEDataChannelParameterInfo: local_MDL_ID 0x%04x channel 0x%04x remote_mtu %d remote_mps %d remote_credits %d maxDsCredits %d\r\n",
							  pConInfo->local_MDL_ID,
							  pConInfo->channel,
							  pConInfo->remote_mtu,
							  pConInfo->remote_mps,
							  pConInfo->remote_initial_credits,
							  pConInfo->maxDsCredits
							  );

	pLink = test_upperstack_LinkFindByLocal_MDL_ID(pGattTest,pConInfo->local_MDL_ID);
	if(pLink)
	{
	  	pChan = test_upperstack_LEChanFind(pGattTest,pLink,pConInfo->channel);
		if(pChan != NULL)
			pChan->maxDsCredit = pConInfo->maxDsCredits;
	}
}

static void test_upperstack_Handle_LEDataChannelCreditsAlertInfo(PGATTTest pGattTest,
                                          PBlueAPI_LEDataChannelCreditsAlertInfo pCreditInfo)
{
	test_upperstack_CmdPrint( pGattTest, "<-- LEDataChannelCreditsAlertInfo: local_MDL_ID 0x%04x channel 0x%04x \r\n",
							  pCreditInfo->local_MDL_ID,
							  pCreditInfo->channel);
	blueAPI_SendLEFlowControlCreditReq( pCreditInfo->local_MDL_ID,
										  pCreditInfo->channel, 5);
}

static void test_upperstack_Handle_LEDataChannelDeleteInfo(PGATTTest pGattTest,
                                          PBlueAPI_LEDataChannelDeleteInfo pDiscInfo)
{
	test_upperstack_CmdPrint( pGattTest, "<-- LEDataChannelDeleteInfo: local_MDL_ID 0x%04x channel 0x%04x \r\n",
							  pDiscInfo->local_MDL_ID,
							  pDiscInfo->channel
							  );
}

static void test_upperstack_Handle_LEPsmSecuritySetRsp(PGATTTest pGattTest,
                                          PBlueAPI_LEPsmSecuritySetRsp pSecRsp)
{
	test_upperstack_CmdPrint( pGattTest, "<-- LEPsmSecuritySetRsp: cause %d\r\n",
							  pSecRsp->cause
							  );
}
#if 0
static void test_upperstack_Handle_DeviceConfigAppearanceGetRsp(PGATTTest pGattTest,
                                          PBlueAPI_DeviceConfigAppearanceGetRsp pAppRsp)
{
	test_upperstack_CmdPrint( pGattTest, "<-- DeviceConfigAppearanceGetRsp: cause %d appearance = 0x%04x\r\n",
							  pAppRsp->cause,
							  pAppRsp->appearance
							  );
}
static void test_upperstack_Handle_DeviceConfigDeviceNameGetRsp(PGATTTest pGattTest,
                                          PBlueAPI_DeviceConfigDeviceNameGetRsp pNameRsp)
{
	uint8_t *value =  pNameRsp->deviceName + pNameRsp->gap;
	
	test_upperstack_CmdPrint( pGattTest, "<-- DeviceConfigDeviceNameGetRsp: cause %d lenth=%d gap=%d\r\n",
							  pNameRsp->cause,
							  pNameRsp->nameLength,
							  pNameRsp->gap
							  );
	test_upperstack_HexDump( pGattTest, "DeviceName=", pNameRsp->nameLength, value );
}
static void test_upperstack_Handle_DeviceConfigPerPrefConnParamGetRsp(PGATTTest pGattTest,
                                          PBlueAPI_DeviceConfigPerPrefConnParamGetRsp pPerRsp)
{
	test_upperstack_CmdPrint( pGattTest, "<-- DeviceConfigPerPrefConnParamGetRsp: cause %d connIntervalMin=0x%04x connIntervalMax=0x%04x slaveLatency=0x%04x supervisionTimeout=0x%04x\r\n",
							  pPerRsp->cause,
							  pPerRsp->conn.connIntervalMin,
							  pPerRsp->conn.connIntervalMax,
							  pPerRsp->conn.slaveLatency,
							  pPerRsp->conn.supervisionTimeout
							  );
}

static void test_upperstack_Handle_SetDataLengthRsp(PGATTTest pGattTest,
                                          PBlueAPI_SetDataLengthRsp pSetDataLengthRsp)
{
	test_upperstack_CmdPrint( pGattTest, "<-- SetDataLengthRsp: local_MDL_ID= %d, cause = %d",
							  pSetDataLengthRsp->local_MDL_ID,
							  pSetDataLengthRsp->cause
							  );
}

static void test_upperstack_Handle_DataLengthChangeInd(PGATTTest pGattTest,
                                          PBlueAPI_DataLengthChangeInfo pDataLengthChangeInfo)
{
	test_upperstack_CmdPrint( pGattTest, 
	                                            "<-- DataLengthChangeInfo: local_MDL_ID= %d, MaxTxOctets = %d, MaxTxTime = %d, MaxRxOctets = %d, MaxRxTime = %d",
							  pDataLengthChangeInfo->local_MDL_ID,
							  pDataLengthChangeInfo->MaxTxOctets,
							  pDataLengthChangeInfo->MaxTxTime,
							  pDataLengthChangeInfo->MaxRxOctets,
							  pDataLengthChangeInfo->MaxRxTime							  
							  );
}
#endif


/*----------------------------------------------------------------------------
 * BlueAPI message handler.
 * Returns non-zero value if calling routine may release message buffer.
 * --------------------------------------------------------------------------*/


void test_upperstack_HandleBlueAPIMessage( PGATTTest pGattTest, PBlueAPI_UsMessage pMsg )

{
    bool      ReleaseBuffer = true;
    switch ( pMsg->Command )
    {
    default:
        test_upperstack_CmdPrint( pGattTest, "test_upperstack_emo: unhandled 0x%x: %s\r\n",
                                  pMsg->Command,
                                  blueAPI_CommandString((TBlueAPI_Command)pMsg->Command)
                                  );
        break;

    case blueAPI_EventRegisterRsp:
        ReleaseBuffer = test_upperstack_Handle_RegisterRsp( pGattTest, &pMsg->p.RegisterRsp );
        break;

    case blueAPI_EventReleaseRsp:
        ReleaseBuffer = test_upperstack_Handle_ReleaseRsp( pGattTest, &pMsg->p.ReleaseRsp );
        break;

    case blueAPI_EventActInfo:
        ReleaseBuffer = test_upperstack_Handle_ActInfo( pGattTest, &pMsg->p.ActInfo );
        break;

    case blueAPI_EventDeviceConfigSetRsp:
        test_upperstack_Handle_DeviceConfigSetRsp(pGattTest, &pMsg->p.DeviceConfigSetRsp);
        break;

    case blueAPI_EventCreateMDLInd:
        ReleaseBuffer = test_upperstack_Handle_CreateMDLInd(
                            pGattTest, &pMsg->p.CreateMDLInd );
        break;

    case blueAPI_EventConnectMDLRsp:
        ReleaseBuffer = test_upperstack_Handle_ConnectMDLRsp(
                            pGattTest, &pMsg->p.ConnectMDLRsp );
        break;

    case blueAPI_EventMCLStatusInfo:
        ReleaseBuffer = test_upperstack_Handle_MCLStatusInfo(
                            pGattTest, &pMsg->p.MCLStatusInfo );
        break;

    case blueAPI_EventConnectMDLInfo:
        ReleaseBuffer = test_upperstack_Handle_ConnectMDLInfo(
                            pGattTest, &pMsg->p.ConnectMDLInfo );
        break;

    case blueAPI_EventDisconnectMDLRsp:
        ReleaseBuffer = test_upperstack_Handle_DisconnectMDLRsp(
                            pGattTest, &pMsg->p.DisconnectMDLRsp );
        break;

    case blueAPI_EventDisconnectMDLInd:
        ReleaseBuffer = test_upperstack_Handle_DisconnectMDLInd(
                            pGattTest, &pMsg->p.DisconnectMDLInd );
        break;

    case blueAPI_EventDeleteMDLInfo:
        ReleaseBuffer = test_upperstack_Handle_DeleteMDLInfo(
                            pGattTest, &pMsg->p.DeleteMDLInfo );
        break;

    case blueAPI_EventInternalEventInfo:
        ReleaseBuffer = test_upperstack_Handle_InternalEventInfo(
                            pGattTest, &pMsg->p.InternalEventInfo );
        break;

    case blueAPI_EventACLStatusInfo:
        ReleaseBuffer = test_upperstack_Handle_ACLStatusInfo(
                            pGattTest, &pMsg->p.ACLStatusInfo );
        break;

    case blueAPI_EventGATTServiceRegisterRsp:
        ReleaseBuffer = test_upperstack_Handle_GATTServiceRegisterRsp(
                            pGattTest, &pMsg->p.GATTServiceRegisterRsp );
        break;

    case blueAPI_EventGATTAttributeUpdateRsp:
        ReleaseBuffer = test_upperstack_Handle_GATTAttributeUpdateRsp(
                            pGattTest, &pMsg->p.GATTAttributeUpdateRsp );
        break;

    case blueAPI_EventGATTAttributeUpdateStatusInd:
        ReleaseBuffer = test_upperstack_Handle_GATTAttributeUpdateStatusInd(
                            pGattTest, &pMsg->p.GATTAttributeUpdateStatusInd );
        break;

    case blueAPI_EventGATTAttributeReadInd:
        ReleaseBuffer = test_upperstack_Handle_GATTAttributeReadInd(
                            pGattTest, &pMsg->p.GATTAttributeReadInd );
        break;

    case blueAPI_EventGATTAttributeWriteInd:
    case blueAPI_EventGATTAttributeWriteCommandInfo:
        ReleaseBuffer = test_upperstack_Handle_GATTAttributeWriteIndCommandInfo(
                            pGattTest, (TBlueAPI_Command)pMsg->Command,
                            &pMsg->p.GATTAttributeWriteInd );
        break;

    case blueAPI_EventGATTCCCDInfo:
        ReleaseBuffer = test_upperstack_Handle_GATTCCCDInfo(
                            pGattTest, &pMsg->p.GATTCCCDInfo );
        break;



    case blueAPI_EventGATTDiscoveryRsp:
        ReleaseBuffer = test_upperstack_Handle_GATTDiscoveryRsp(
                            pGattTest, &pMsg->p.GATTDiscoveryRsp );
        break;

    case blueAPI_EventGATTDiscoveryInd:
        ReleaseBuffer = test_upperstack_Handle_GATTDiscoveryInd(
                            pGattTest, &pMsg->p.GATTDiscoveryInd );
        break;



    case blueAPI_EventGATTAttributeReadRsp:
        ReleaseBuffer = test_upperstack_Handle_GATTAttributeReadRsp(
                            pGattTest, &pMsg->p.GATTAttributeReadRsp );
        break;
#if 0
    case blueAPI_EventGATTAttributeReadMultipleRsp:
        ReleaseBuffer = test_upperstack_Handle_GATTAttributeReadMultipleRsp(
								       pGattTest, &pMsg->p.GATTAttributeReadMultipleRsp );
      break;
#endif
    case blueAPI_EventGATTAttributeWriteRsp:
        ReleaseBuffer = test_upperstack_Handle_GATTAttributeWriteRsp(
                            pGattTest, &pMsg->p.GATTAttributeWriteRsp );
        break;

    case blueAPI_EventGATTAttributeInd:
    case blueAPI_EventGATTAttributeNotificationInfo:
        ReleaseBuffer = test_upperstack_Handle_GATTAttributeNotifInd(
                            pGattTest, (TBlueAPI_Command)pMsg->Command,
                            &pMsg->p.GATTAttributeInd );
        break;


    case blueAPI_EventGATTSecurityRsp:
        test_upperstack_Handle_GATTSecurityRsp(pGattTest, &pMsg->p.GATTSecurityRsp);
        break;

    case blueAPI_EventGATTServerStoreInd:
        test_upperstack_Handle_GATTServerStoreInd(pGattTest, &pMsg->p.GATTServerStoreInd);
        break;

    case blueAPI_EventGATTMtuSizeInfo:
        test_upperstack_Handle_GATTMtuSizeInfo(pGattTest, &pMsg->p.GATTMtuSizeInfo);
        break;

    case blueAPI_EventLEAdvertiseRsp:
        test_upperstack_Handle_LEAdvertiseRsp(pGattTest, &pMsg->p.LEAdvertiseRsp);
        break;

    case blueAPI_EventLEAdvertiseParameterSetRsp:
        test_upperstack_Handle_LEAdvertiseParameterRsp(pGattTest, &pMsg->p.LEAdvertiseParameterSetRsp);
        break;

    case blueAPI_EventLEAdvertiseDataSetRsp:
        test_upperstack_Handle_LEAdvertiseDataSetRsp(pGattTest, &pMsg->p.LEAdvertiseDataSetRsp);
        break;

    case blueAPI_EventLEScanRsp:
        test_upperstack_Handle_LEScanRsp(pGattTest, &pMsg->p.LEScanRsp);
        break;

    case blueAPI_EventLEScanInfo:
        test_upperstack_Handle_LEScanInfo(pGattTest, &pMsg->p.LEScanInfo);
        break;

    case blueAPI_EventLEModifyWhitelistRsp:
        test_upperstack_Handle_LEModifyWhitelistRsp(pGattTest, &pMsg->p.LEModifyWhitelistRsp);
        break;

    case blueAPI_EventLEConnectionUpdateRsp:
        test_upperstack_Handle_LEConnectionUpdateRsp(pGattTest, &pMsg->p.LEConnectionUpdateRsp);
        break;

    case blueAPI_EventLEConnectionUpdateInd:
        test_upperstack_Handle_LEConnectionUpdateInd(pGattTest, &pMsg->p.LEConnectionUpdateInd);
        break;

    case blueAPI_EventLEConnectionParameterInfo:
        test_upperstack_Handle_LEConnectionParameterInfo(pGattTest, &pMsg->p.LEConnectionParameterInfo);
        break;

#if (F_BT_LE_PRIVACY_MODE)
    case blueAPI_EventLEPrivacyModeRsp:
        test_upperstack_Handle_LEPrivacyModeRsp(pGattTest, &pMsg->p.LEPrivacyModeRsp);
        break;
#endif /* (F_BT_LE_PRIVACY_MODE) */

    case blueAPI_EventAuthResultInd:
        test_upperstack_Handle_AuthResultInd(pGattTest, &pMsg->p.AuthResultInd);
        break;

    case blueAPI_EventAuthResultRequestInd:
        test_upperstack_Handle_AuthResultRequestInd(pGattTest, &pMsg->p.AuthResultRequestInd);
        break;

    case blueAPI_EventPairableModeSetRsp:
        test_upperstack_Handle_PairableModeSetRsp(pGattTest, &pMsg->p.PairableModeSetRsp);
        break;

    case blueAPI_EventUserPasskeyReqInd:
        test_upperstack_Handle_UserPasskeyReqInd(pGattTest, &pMsg->p.UserPasskeyReqInd);
        break;

    case blueAPI_EventUserPasskeyNotificationInfo:
        test_upperstack_Handle_UserPasskeyNotificationInfo(pGattTest, &pMsg->p.UserPasskeyNotificationInfo);
        break;


    case blueAPI_EventUserPasskeyReqReplyRsp:
        test_upperstack_Handle_UserPasskeyReqReplyRsp(pGattTest, &pMsg->p.UserPasskeyReqReplyRsp);
        break;

    case blueAPI_EventRemoteOOBDataReqInd:
        test_upperstack_Handle_RemoteOOBDataReqInd(pGattTest, &pMsg->p.RemoteOOBDataReqInd);
        break;
#if 0
    case blueAPI_EventVendorSetVoiceParaRsp:
        test_upperstack_Handle_VendorSetVoicePara(pGattTest, &pMsg->p.VendorSetVoiceParaRsp);

        break;
#endif
#ifdef RTL8762AX_VB
	case blueAPI_EventSetBleTxPowerRsp:
        test_upperstack_Handle_SetBleTxPowerRsp(pGattTest, &pMsg->p.SetBleTxPowerRsp);
        break;
#endif
	case blueAPI_EventSetRandomAddressRsp:
        test_upperstack_Handle_SetRandomAddressRsp(pGattTest, &pMsg->p.SetRandomAddressRsp);
        break;

#if (GATTDEMO_PREPARE_WRITE_SUPPPRT)
	case blueAPI_EventGATTAttributePrepareWriteInd:
	    ReleaseBuffer = test_upperstack_Handle_GATTAttributePrepareWriteInd(
										 pGattTest, (TBlueAPI_Command)pMsg->Command,
										 &pMsg->p.GATTAttributeWriteInd );
		break;

    case blueAPI_EventGATTAttributePrepareWriteRsp:
	    test_upperstack_Handle_GATTAttributePrepareWriteRsp(pGattTest, &pMsg->p.GATTAttributePrepareWriteRsp);
	    break;
   
    case blueAPI_EventGATTAttributeExecuteWriteRsp:
	    test_upperstack_Handle_GATTAttributeExecuteWriteRsp(pGattTest, &pMsg->p.GATTAttributeExecuteWriteRsp);
	    break;
	  
	case blueAPI_EventGATTAttributeExecuteWriteInd:
	    test_upperstack_Handle_GATTAttributeExecuteWriteInd(pGattTest, &pMsg->p.GATTAttributeExecuteWriteInd);
	    break;
#endif

    case blueAPI_EventCreateLEDataChannelRsp:
	  	test_upperstack_Handle_CreateLEDataChannelRsp(pGattTest, &pMsg->p.CreateLEDataChannelRsp);
	  	break;
	
    case blueAPI_EventCreateLEDataChannelInd:
	  	test_upperstack_Handle_CreateLEDataChannelInd(pGattTest, &pMsg->p.CreateLEDataChannelInd);
	  	break;
	case blueAPI_EventDisconnectLEDataChannelRsp:
	  	test_upperstack_Handle_DisconnectLEDataChannelRsp(pGattTest, &pMsg->p.DisconnectLEDataChannelRsp);
	  	break;
	  
	case blueAPI_EventDisconnectLEDataChannelInd:
	  	test_upperstack_Handle_DisconnectLEDataChannelInd(pGattTest, &pMsg->p.DisconnectLEDataChannelInd);
	  	break;

	case blueAPI_EventSendLEFlowControlCreditRsp:
	  	test_upperstack_Handle_SendLEFlowControlCreditRsp(pGattTest, &pMsg->p.SendLEFlowControlCreditRsp);
	  	break;
	case blueAPI_EventLEDataInd:
	  	test_upperstack_Handle_LEDataInd(pGattTest, &pMsg->p.LEDataInd);
	  	break;
	case blueAPI_EventLEDataRsp:
	  	test_upperstack_Handle_LEDataRsp(pGattTest, &pMsg->p.LEDataRsp);
	  	break;
	case blueAPI_EventLEDataChannelParameterInfo:
	  	test_upperstack_Handle_LEDataChannelParameterInfo(pGattTest, &pMsg->p.LEDataChannelParameterInfo);
	  	break;
	case blueAPI_EventLEDataChannelCreditsAlertInfo:
	  	test_upperstack_Handle_LEDataChannelCreditsAlertInfo(pGattTest, &pMsg->p.LEDataChannelCreditsAlertInfo);
	  	break;
	case blueAPI_EventLEDataChannelDeleteInfo:
	  	test_upperstack_Handle_LEDataChannelDeleteInfo(pGattTest, &pMsg->p.LEDataChannelDeleteInfo);
	  	break;
	case blueAPI_EventLEPsmSecuritySetRsp:
	  	test_upperstack_Handle_LEPsmSecuritySetRsp(pGattTest, &pMsg->p.LEPsmSecuritySetRsp);
	  	break;
#if 0
	case blueAPI_EventDeviceConfigAppearanceGetRsp:
		test_upperstack_Handle_DeviceConfigAppearanceGetRsp(pGattTest, &pMsg->p.DeviceConfigAppearanceGetRsp);
		break;
	case blueAPI_EventDeviceConfigDeviceNameGetRsp:
		test_upperstack_Handle_DeviceConfigDeviceNameGetRsp(pGattTest, &pMsg->p.DeviceConfigDeviceNameGetRsp);
		break;
	case blueAPI_EventDeviceConfigPerPrefConnParamGetRsp:
		test_upperstack_Handle_DeviceConfigPerPrefConnParamGetRsp(pGattTest, &pMsg->p.DeviceConfigPerPrefConnParamGetRsp);
		break;
        case blueAPI_EventSetDataLengthRsp:
            test_upperstack_Handle_SetDataLengthRsp(pGattTest, &pMsg->p.SetDataLengthRsp);
            break;
        case blueAPI_EventDataLengthChangeInfo:
            test_upperstack_Handle_DataLengthChangeInd(pGattTest, &pMsg->p.DataLengthChangeInfo);
            break;
#endif

    }


    if (ReleaseBuffer )
        blueAPI_BufferRelease(pMsg);

}
