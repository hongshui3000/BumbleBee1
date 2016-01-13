
#include "gatttest.h"
#include <rtl_types.h>
#include <string.h>
#include "test_cmd.h"
#include "test_upperstack_api.h"


#if !defined (offsetof)
/* offset of field m in a struct s */
#define offsetof(s, m)  (unsigned long) ((char *) &(((s *) 0)->m) - (char *) 0)
#endif

static void test_upperstack_GLC_RACPSendResp( PGATTTest pGattTest, UINT8 RespCode, bool Retry );



static TIMESTAMP Timestamp1 =
{
    LO_WORD(2011), HI_WORD(2011),  /* YY:YY */
    10,        /* MM */
    6,         /* DD */
    11,        /* HH */
    26,        /* MM */
    15         /* SS */
};

static TIMESTAMP Timestamp2 =
{
    LO_WORD(2011), HI_WORD(2011),  /* YY:YY */
    10,        /* MM */
    7,         /* DD */
    12,        /* HH */
    26,        /* MM */
    15         /* SS */
};



#define CASE2STR(x) case x : pStr = (char *)#x; break

/*----------------------------------------------------------------------------
 * RACP opcode to string conversion
 * --------------------------------------------------------------------------*/

static char * test_upperstack_OpCode2Str( UINT8 OpCode )
{
    char  * pStr;

    switch ( OpCode )
    {
        CASE2STR(GLC_RACP_OPCODE_REPORT_RECS);
        CASE2STR(GLC_RACP_OPCODE_DELETE_RECS);
        CASE2STR(GLC_RACP_OPCODE_ABORT_OPERATION);
        CASE2STR(GLC_RACP_OPCODE_REPORT_NBR_OF_RECS);
        CASE2STR(GLC_RACP_OPCODE_NBR_OF_RECS_RESP);
        CASE2STR(GLC_RACP_OPCODE_RESP_CODE);
    default:
        pStr = "Unknown OpCode";
        break;
    }

    return ( pStr );
}

/*----------------------------------------------------------------------------
 * RACP operator to string conversion
 * --------------------------------------------------------------------------*/

static char * test_upperstack_Operator2Str( UINT8 Operator )
{
    char  * pStr;

    switch ( Operator )
    {
        CASE2STR(GLC_RACP_OPERATOR_NULL);
        CASE2STR(GLC_RACP_OPERATOR_ALL_RECS);
        CASE2STR(GLC_RACP_OPERATOR_LT_EQ);
        CASE2STR(GLC_RACP_OPERATOR_GT_EQ);
        CASE2STR(GLC_RACP_OPERATOR_RANGE);
        CASE2STR(GLC_RACP_OPERATOR_FIRST);
        CASE2STR(GLC_RACP_OPERATOR_LAST);
    default:
        pStr = "Unknown Operator";
        break;
    }

    return ( pStr );
}

/*----------------------------------------------------------------------------
 * RACP filter type to string conversion
 * --------------------------------------------------------------------------*/

static char * test_upperstack_FilterType2Str( UINT8 FilterType )
{
    char  * pStr;

    switch ( FilterType )
    {
        CASE2STR(GLC_RACP_FILTER_TYPE_SEQ_NBR);
        CASE2STR(GLC_RACP_FILTER_TYPE_TIME);
    default:
        pStr = "Unknown Filter Type";
        break;
    }

    return ( pStr );
}


/*----------------------------------------------------------------------------
 * RACP response code to string conversion
 * --------------------------------------------------------------------------*/

static char * test_upperstack_RespCode2Str( UINT8 RespCode )
{
    char  * pStr;

    switch ( RespCode )
    {
        CASE2STR(GLC_RACP_RESP_SUCCESS);
        CASE2STR(GLC_RACP_RESP_OPCODE_NOT_SUPPORTED);
        CASE2STR(GLC_RACP_RESP_INVALID_OPERATOR);
        CASE2STR(GLC_RACP_RESP_OPERATOR_NOT_SUPPORTED);
        CASE2STR(GLC_RACP_RESP_INVALID_OPERAND);
        CASE2STR(GLC_RACP_RESP_NO_RECS_FOUND);
        CASE2STR(GLC_RACP_RESP_ABORT_UNSUCCESSFUL);
        CASE2STR(GLC_RACP_RESP_PROC_NOT_COMPLETED);
        CASE2STR(GLC_RACP_RESP_OPERAND_NOT_SUPPORTED);
    default:
        pStr = "Unknown Response Code";
        break;
    }

    return ( pStr );
}

/*----------------------------------------------------------------------------
 * display RACP response
 * --------------------------------------------------------------------------*/

static void  test_upperstack_GLC_RACPDisplayResp( PGATTTest pGattTest, bool PromptCrLf,
        PGLCControlPoint pValue )
{

    test_upperstack_CmdPrint( pGattTest, "RACP response: ReqOpCode=0x%x (%s)\r\n ",
                              pValue->Operand[0], test_upperstack_OpCode2Str(pValue->Operand[0])
                              );
    test_upperstack_CmdPrint( pGattTest, "               RespCode=0x%x (%s)\r\n ",
                              pValue->Operand[1], test_upperstack_RespCode2Str(pValue->Operand[1])
                              );
}

/*----------------------------------------------------------------------------
 * display RACP nbr. of stored records response
 * --------------------------------------------------------------------------*/

static void  test_upperstack_GLC_RACPDisplayNbrOfRecsResp( PGATTTest pGattTest,
        bool PromptCrLf, PGLCControlPoint pValue )
{
 
    test_upperstack_CmdPrint( pGattTest, "RACP response: Nbr. of stored recs. = %d\r\n ",
                              LE_EXTRN2WORD(pValue->Operand) );
}

/*----------------------------------------------------------------------------
 * display RACP filter related data
 * --------------------------------------------------------------------------*/

static void  test_upperstack_GLC_RACPDisplayFilterData( PGATTTest pGattTest,
        bool PromptCrLf, PGLCControlPoint pValue )
{
    PGLCRACP pRACP      = &pGattTest->RACP;
    UINT8    FilterType = pValue->Operand[0];

 

    if ( ((pRACP->Value.Operator == GLC_RACP_OPERATOR_LT_EQ) ||
            (pRACP->Value.Operator == GLC_RACP_OPERATOR_GT_EQ) ||
            (pRACP->Value.Operator == GLC_RACP_OPERATOR_RANGE))
       )
    {
        test_upperstack_CmdPrint( pGattTest, "  Filter data: Type=0x%x (%s)\r\n ",
                                  FilterType, test_upperstack_FilterType2Str(FilterType)
                                  );

        if ( FilterType == GLC_RACP_FILTER_TYPE_SEQ_NBR )
        {
            if ( (pValue->Operator == GLC_RACP_OPERATOR_LT_EQ) ||
                    (pValue->Operator == GLC_RACP_OPERATOR_GT_EQ)
               )
            {
                test_upperstack_CmdPrint( pGattTest, "    SeqNbr=0x%x\r\n ",
                                          LE_EXTRN2WORD(&pValue->Operand[1])
                                          );
            }
            else if ( pValue->Operator == GLC_RACP_OPERATOR_RANGE )
            {
                test_upperstack_CmdPrint( pGattTest, "    SeqNbr range=[0x%x - 0x%x]\r\n ",
                                          LE_EXTRN2WORD(&pValue->Operand[1]),
                                          LE_EXTRN2WORD(&pValue->Operand[1 + sizeof(UINT16)])
                                          );
            }
        }
        else if ( FilterType == GLC_RACP_FILTER_TYPE_TIME )
        {
            //XXXXMJMJ
        }
    }
}






/*----------------------------------------------------------------------------
 * command racpinit:  init. RACP
 * --------------------------------------------------------------------------*/

TTestResult test_upperstack_GLC_RACPInit(PGATTTest pGattTest,
        TTestParseResult *pParseResult)
{
    gattTestServiceInitData( pGattTest );
    return ( test_ResultOk );
}

/*----------------------------------------------------------------------------
 * command racppar:  set RACP parameters
 *   racppar [nbr. of recs] [init. seqnbr]  [seqnbr delta] [special] [reqnum]
 * --------------------------------------------------------------------------*/

TTestResult test_upperstack_GLC_RACPSetParam(PGATTTest pGattTest,
        TTestParseResult *pParseResult)
{
    PGLCRACP pRACP = &pGattTest->RACP;
    int index      = pParseResult->dwParameter[4];

    if (index >= GLC_MAX_RACP_REQS)
        index = GLC_MAX_RACP_REQS - 1;

    if (index > pRACP->iReqNumMax)
        pRACP->iReqNumMax = index;

    pRACP->iReqNum = 0;

    pRACP->wNbrOfRecs[index] = pParseResult->dwParameter[0];
    pRACP->wSeqNbr[index]    = pParseResult->dwParameter[1];

    if ( pParseResult->dwParameter[2] == 0 )
        pParseResult->dwParameter[2] = 1;

    pRACP->iSeqNbrDelta[index]      = pParseResult->dwParameter[2];
    pRACP->iSpecialBehavior[index]  = pParseResult->dwParameter[3];

    return ( test_ResultOk );
}



/*----------------------------------------------------------------------------
 * command racp :  write to RACP
 *   racp [opcode] [operator] [filter type] [operand ..]
 * --------------------------------------------------------------------------*/

#define GLC_FILTER_XXXXMJMJ  0xAB /* test case 4.14.6 TP/SPE/BI-06-C [Unsupported Filter Type] hack */
#define GLC_USER_TIMESTAMP_SIZE   7
TTestResult test_upperstack_GLC_RACPWrite(PGATTTest pGattTest,
        TTestParseResult *pParseResult)
{
    PGLCRACP  pRACP;
    UINT8     OpCode, Operator, FilterType;
    uint8_t * pDst;
    TTestResult Result = test_ResultError;

    OpCode     = pParseResult->dwParameter[1];
    Operator   = pParseResult->dwParameter[2];
    FilterType = pParseResult->dwParameter[3];

    if ( (FilterType != GLC_RACP_FILTER_TYPE_TIME)
            && (FilterType != GLC_FILTER_XXXXMJMJ)
       )
    {
        FilterType = GLC_RACP_FILTER_TYPE_SEQ_NBR;
    }

    pRACP    = &pGattTest->RACP;

    if ( (pRACP->wHandle != 0)
       )
    {
        uint8_t bTimestamp[GLC_USER_TIMESTAMP_SIZE];

        pRACP->iLength        = 2 * sizeof(UINT8);
        pRACP->Value.OpCode   = OpCode;
        pRACP->Value.Operator = Operator;

        switch ( OpCode )
        {
        default:
            break;

        case GLC_RACP_OPCODE_REPORT_RECS:
        case GLC_RACP_OPCODE_DELETE_RECS:
        case GLC_RACP_OPCODE_REPORT_NBR_OF_RECS:
            pDst = &pRACP->Value.Operand[1];
            if ( (Operator == GLC_RACP_OPERATOR_LT_EQ) ||
                    (Operator == GLC_RACP_OPERATOR_GT_EQ)
               )
            {
                /* insert filter type + seq.nbr/time */
                pRACP->Value.Operand[0] = FilterType;
                pRACP->iLength         += sizeof(UINT8);
                if ( (FilterType == GLC_FILTER_XXXXMJMJ) ||
                        (FilterType == GLC_RACP_FILTER_TYPE_SEQ_NBR)
                   )
                {
                    LE_WORD2EXTRN( pDst, (uint16_t)pParseResult->dwParameter[4] );
                    pRACP->iLength += sizeof(UINT16);
                }
                else
                {
                    /* user facing time */
                    if (pParseResult->iParameterCount == 5)
                    {
                        /* convert ASCII input to binary: */
                        ASCIIHexToBin((uint8_t *)pParseResult->pParameter[4],
                                      bTimestamp, sizeof(bTimestamp));
                        memcpy( pDst, bTimestamp, sizeof(bTimestamp) );
                        pRACP->iLength += sizeof(bTimestamp);
                    }
                    else
                    {
                        memcpy( pDst, Timestamp1, sizeof(TIMESTAMP) );
                        pRACP->iLength += sizeof(TIMESTAMP);
                    }
                }
            }
            else if ( Operator == GLC_RACP_OPERATOR_RANGE )
            {
                /* insert filter type + seq.nbr/time pair */
                pRACP->Value.Operand[0] = FilterType;
                pRACP->iLength         += sizeof(UINT8);
                if ( FilterType == GLC_RACP_FILTER_TYPE_SEQ_NBR )
                {
                    LE_WORD2EXTRN( pDst, (uint16_t)pParseResult->dwParameter[4] );
                    pDst += sizeof(UINT16);
                    LE_WORD2EXTRN( pDst, (uint16_t)pParseResult->dwParameter[5] );
                    pRACP->iLength += 2 * sizeof(UINT16);
                }
                else
                {
                    /* user facing time */
                    /* start of range */
                    if (pParseResult->iParameterCount >= 5)
                    {
                        /* convert ASCII input to binary: */
                        ASCIIHexToBin((uint8_t *)pParseResult->pParameter[4],
                                      bTimestamp, sizeof(bTimestamp));
                        memcpy( pDst, bTimestamp, sizeof(bTimestamp) );
                        pRACP->iLength += sizeof(bTimestamp);
                    }
                    else
                    {
                        memcpy( pDst, Timestamp1, sizeof(TIMESTAMP) );
                        pRACP->iLength += sizeof(TIMESTAMP);
                    }
                    pDst += sizeof(TIMESTAMP);

                    /* end of range */
                    if (pParseResult->iParameterCount == 6)
                    {
                        /* convert ASCII input to binary: */
                        ASCIIHexToBin((uint8_t *)pParseResult->pParameter[5],
                                      bTimestamp, sizeof(bTimestamp));
                        memcpy( pDst, bTimestamp, sizeof(bTimestamp) );
                        pRACP->iLength += sizeof(bTimestamp);
                    }
                    else
                    {
                        memcpy( pDst, Timestamp2, sizeof(TIMESTAMP) );
                        pRACP->iLength += sizeof(TIMESTAMP);
                    }
                }
            }
            break;
        }

        /* write to RACP */
        pParseResult->dwParameter[1] = pRACP->wHandle;
        pParseResult->dwParameter[2] = 0;
        pParseResult->dwParameter[3] = 0;
        if ( (Result = test_upperstack_AttribWrite( pGattTest, pParseResult )) == test_ResultOk )
        {

        }

        /* reset opcode (potential role switch in next connection, new      */
        /* connection does not clear opcode => RACP request would fail ...) */
        pRACP->Value.OpCode = GLC_RACP_OPCODE_RESERVED;
    }

    return ( Result );
}

/*----------------------------------------------------------------------------
 * handle RACP response
 * --------------------------------------------------------------------------*/

void  test_upperstack_GLC_RACPHandleResp( PGATTTest pGattTest, PGLCControlPoint pValue )
{
    /* XXXXMJMJ: timer must be stopped too when:
     * - connection is shutdown
     * - GLC measurement and context notification is received (and must be restarted!!)
     */


    if ( pValue->OpCode == GLC_RACP_OPCODE_NBR_OF_RECS_RESP )
    {
        test_upperstack_GLC_RACPDisplayNbrOfRecsResp( pGattTest, true, pValue );
    }
    else
    {
        test_upperstack_GLC_RACPDisplayResp( pGattTest, true, pValue );
    }
}

/*----------------------------------------------------------------------------
 * send RACP response
 * --------------------------------------------------------------------------*/

static void test_upperstack_GLC_RACPSendResp( PGATTTest pGattTest, UINT8 RespCode, bool Retry )
{
    TTestParseResult ParseResult;
    PGLCRACP             pRACP = &pGattTest->RACP;

    if ( pRACP->iSpecialBehavior[pRACP->iReqNum] == 2 )
    {
        /* no rsp: test case 4.9.1  TP/SPE/BI-01-I [RACP Procedure Timeout] hack  */
    }
    else
    {
        pRACP->Value.Operand[0] = pRACP->Value.OpCode;    /* req. opcode */
        pRACP->Value.Operand[1] = RespCode;
        pRACP->Value.OpCode     = GLC_RACP_OPCODE_RESP_CODE;
        pRACP->Value.Operator   = GLC_RACP_OPERATOR_NULL;

        pRACP->iLength = offsetof(TGLCControlPoint, Operand) + 2 * sizeof(UINT8);

        if ( !Retry )
            test_upperstack_GLC_RACPDisplayResp( pGattTest, false, &pRACP->Value );

        /* send indication to client */
        ParseResult.iParameterCount = 2;
        ParseResult.dwParameter[0]  = GATT_SVC_GLS_RACP_INDEX;
        ParseResult.dwParameter[1]  = 0;
        if ( test_upperstack_AttribUpdate( pGattTest, &ParseResult ) != test_ResultOk )
        {
            /* no tx buffer (cannot happen if tx windows size is not exceeded) .. */

            return;
        }
    }

    pRACP->Value.OpCode = GLC_RACP_OPCODE_RESERVED; //XXXXMJMJ move to ATT_HANDLE_VALUE_CONFIRMATION !!

    if ((pRACP->iReqNumMax != 0) &&
            (pRACP->iReqNum < pRACP->iReqNumMax)
       )
    {
        pRACP->iReqNum++;
        test_upperstack_CmdPrint( pGattTest, "exec racppar %d %d %d %d\r\n",
                                  pGattTest->RACP.wNbrOfRecs[pRACP->iReqNum],
                                  pGattTest->RACP.wSeqNbr[pRACP->iReqNum],
                                  pGattTest->RACP.iSeqNbrDelta[pRACP->iReqNum],
                                  pGattTest->RACP.iSpecialBehavior[pRACP->iReqNum]
                                  );
    }
}

/*----------------------------------------------------------------------------
 * send RACP nbr. of stored records response
 * --------------------------------------------------------------------------*/

static void test_upperstack_GLC_RACPReportNbrOfRecs( PGATTTest pGattTest )
{
    TTestParseResult ParseResult;
    PGLCRACP             pRACP = &pGattTest->RACP;

    test_upperstack_GLC_RACPDisplayFilterData( pGattTest, false, &pRACP->Value );

    /* setup rsp */
    pRACP->Value.OpCode     = GLC_RACP_OPCODE_NBR_OF_RECS_RESP;
    pRACP->Value.Operator   = GLC_RACP_OPERATOR_NULL;
    LE_WORD2EXTRN( pRACP->Value.Operand, pRACP->wNbrOfRecs[pRACP->iReqNum] );

    pRACP->iLength = offsetof(TGLCControlPoint, Operand) + sizeof(UINT16);

    test_upperstack_GLC_RACPDisplayNbrOfRecsResp( pGattTest, false, &pRACP->Value );

    /* send indication to client */
    ParseResult.iParameterCount = 2;
    ParseResult.dwParameter[0]  = GATT_SVC_GLS_RACP_INDEX;
    ParseResult.dwParameter[1]  = 0;
    if ( test_upperstack_AttribUpdate( pGattTest, &ParseResult ) != test_ResultOk )
    {
        //XXXXMJMJ
    }

    pRACP->Value.OpCode = GLC_RACP_OPCODE_RESERVED; //XXXXMJMJ move to ATT_HANDLE_VALUE_CONFIRMATION !!
}

/*----------------------------------------------------------------------------
 * send stored records
 * --------------------------------------------------------------------------*/

static void test_upperstack_GLC_RACPReportRecs( PGATTTest pGattTest )
{
    TTestParseResult ParseResult;
    PGLCRACP             pRACP = &pGattTest->RACP;
    UINT8                respCode = GLC_RACP_RESP_SUCCESS;

    test_upperstack_GLC_RACPDisplayFilterData( pGattTest, false, &pRACP->Value );

    /* setup rsp XXXXMJM put all plausi stuff in one routine !!!! */
    if ( pRACP->Value.Operator == GLC_RACP_OPERATOR_NULL )
    {
        respCode = GLC_RACP_RESP_INVALID_OPERATOR;
    }
    else if (pRACP->Value.Operator > GLC_RACP_OPERATOR_LAST)
    {
        respCode = GLC_RACP_RESP_OPERATOR_NOT_SUPPORTED;
    }
    else if (((pRACP->Value.Operator == GLC_RACP_OPERATOR_ALL_RECS) ||
              (pRACP->Value.Operator == GLC_RACP_OPERATOR_FIRST) ||
              (pRACP->Value.Operator == GLC_RACP_OPERATOR_LAST)
             ) &&
             (pRACP->Value.Operand[0] != GLC_RACP_FILTER_TYPE_RESERVED)
            )
    {
        respCode = GLC_RACP_RESP_INVALID_OPERAND;
    }
    else if ( (pRACP->Value.Operator >= GLC_RACP_OPERATOR_LT_EQ) &&
              (pRACP->Value.Operator <= GLC_RACP_OPERATOR_RANGE)
            )
    {
        if (pRACP->Value.Operand[0] == GLC_RACP_FILTER_TYPE_RESERVED)
        {
            respCode = GLC_RACP_RESP_INVALID_OPERAND;
        }
        else if (pRACP->Value.Operand[0] > GLC_RACP_FILTER_TYPE_TIME)
        {
            respCode = GLC_RACP_RESP_OPERAND_NOT_SUPPORTED;
        }
        else if ((pRACP->Value.Operator == GLC_RACP_OPERATOR_RANGE) &&
                 (LE_EXTRN2WORD(&pRACP->Value.Operand[1]) > LE_EXTRN2WORD(&pRACP->Value.Operand[1 + sizeof(UINT16)]))
                )
        {
            respCode = GLC_RACP_RESP_INVALID_OPERAND;
        }
    }

    if (pRACP->wNbrOfRecs[pRACP->iReqNum] == 0)
    {
        respCode = GLC_RACP_RESP_NO_RECS_FOUND;
    }

    if (respCode == GLC_RACP_RESP_SUCCESS)
    {
        /* send (sequence of notifications) to client */
        ParseResult.iParameterCount = 2;
        ParseResult.dwParameter[0]  = GATT_SVC_GLS_MEASUREMENT_INDEX;
        ParseResult.dwParameter[1]  = pRACP->wNbrOfRecs[pRACP->iReqNum];
        if ( test_upperstack_AttribUpdate( pGattTest, &ParseResult ) != test_ResultOk )
        {
            //XXXXMJMJ
        }
    }
    else
    {
        test_upperstack_GLC_RACPSendResp( pGattTest, respCode, false );
    }
}

/*----------------------------------------------------------------------------
 * delete stored records
 * --------------------------------------------------------------------------*/

static void test_upperstack_GLC_RACPDeleteRecs( PGATTTest pGattTest )
{
    PGLCRACP   pRACP = &pGattTest->RACP;

    test_upperstack_GLC_RACPDisplayFilterData( pGattTest, false, &pRACP->Value );

    pRACP->wNbrOfRecs[pRACP->iReqNum] = 0;

    /* setup rsp */
    test_upperstack_GLC_RACPSendResp( pGattTest, GLC_RACP_RESP_SUCCESS, false );
}

/*----------------------------------------------------------------------------
 * abort operation
 * --------------------------------------------------------------------------*/

static void test_upperstack_GLC_RACPAbortOperation( PGATTTest pGattTest )
{
    /* XXXXMJMJ this is a hack: iUpdateCnt is used for ABORTED operation and */
    /* the ABORT operation too !!!: */
    pGattTest->iUpdateCnt = 0;

    /* setup rsp */
    test_upperstack_GLC_RACPSendResp( pGattTest, GLC_RACP_RESP_SUCCESS, false );
}


/*----------------------------------------------------------------------------
 * handle RACP procedure complete callback
 * --------------------------------------------------------------------------*/

void  test_upperstack_GLC_RACPProcComplete( PGATTTest pGattTest, uint16_t wCause )
{
    PGLCRACP  pRACP = &pGattTest->RACP;

    if ( GLC_RACP_OPERATION_ACTIVE( pRACP->Value.OpCode ) )
    {
        if ( wCause == GATT_ERR_NOTIF_IND_NOT_CFG )
        {
            pRACP->Value.OpCode = GLC_RACP_OPCODE_RESERVED; //XXXXMJMJ move to ATT_HANDLE_VALUE_CONFIRMATION !!
        }
        else
        {
            test_upperstack_GLC_RACPSendResp( pGattTest,
                                              (wCause == GATT_SUCCESS) ? GLC_RACP_RESP_SUCCESS : GLC_RACP_RESP_INTERNAL,
                                              false);
        }
    }
}

/*----------------------------------------------------------------------------
 * handle RACP write (request)
 * --------------------------------------------------------------------------*/

void  test_upperstack_GLC_RACPHandleReq( PGATTTest pGattTest, uint16_t wLength, uint8_t * pValue )
{
    PGLCRACP  pRACP = &pGattTest->RACP;

    /*
    if ( *pValue == GLC_RACP_OPCODE_ABORT_OPERATION)
      DebuggerBreak();
    */

    //XXXXMJMJ superfluous due to gattdGLC_RACPOperationAllowed() !!!!??:
    if ( GLC_RACP_OPERATION_ACTIVE(pRACP->Value.OpCode) &&
            (*pValue != GLC_RACP_OPCODE_ABORT_OPERATION)
       )
    {
        //XXXXMJMJ ATT_WRITE_RERQUEST must be rejected !!!
        return;
    }

    memset( &pGattTest->RACP.Value, 0, sizeof(TGLCControlPoint) );
    memcpy( &pGattTest->RACP.Value, pValue, wLength );
    pGattTest->RACP.iLength = wLength;

    test_upperstack_CmdPrint( pGattTest, "RACP request: OpCode=0x%x (%s)\r\n ",
                              pRACP->Value.OpCode, test_upperstack_OpCode2Str(pRACP->Value.OpCode)
                              );

    if ( pRACP->iLength > sizeof(UINT8) )
    {
        test_upperstack_CmdPrint( pGattTest, "  Operator=0x%x (%s)\r\n ",
                                  pRACP->Value.Operator,
                                  test_upperstack_Operator2Str(pRACP->Value.Operator)
                                  );
    }

    switch ( pRACP->Value.OpCode )
    {
    default:
        test_upperstack_GLC_RACPSendResp( pGattTest, GLC_RACP_RESP_OPCODE_NOT_SUPPORTED, false );
        break;

    case GLC_RACP_OPCODE_REPORT_NBR_OF_RECS:
        test_upperstack_GLC_RACPReportNbrOfRecs( pGattTest );
        break;

    case GLC_RACP_OPCODE_REPORT_RECS:
        test_upperstack_GLC_RACPReportRecs( pGattTest );
        break;

    case GLC_RACP_OPCODE_DELETE_RECS:
        test_upperstack_GLC_RACPDeleteRecs( pGattTest );
        break;

    case GLC_RACP_OPCODE_ABORT_OPERATION:
        test_upperstack_GLC_RACPAbortOperation( pGattTest );
        break;
    }
}

/*----------------------------------------------------------------------------
 * check if RACP operation is currently allowed
 * --------------------------------------------------------------------------*/

bool  test_upperstack_GLC_RACPOperationAllowed( PGATTTest pGattTest, uint8_t bOpCode )
{

    return ( (bOpCode == GLC_RACP_OPCODE_ABORT_OPERATION ) ||
             !(GLC_RACP_OPERATION_ACTIVE( pGattTest->RACP.Value.OpCode ))
           );
}
