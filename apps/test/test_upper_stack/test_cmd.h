#ifndef _TEST_CMD_H_
#define _TEST_CMD_H_

#include "gatttest.h"


#define MAX_COMMAND_LINE   80           /* max. length of command line in bytes */
#define MAX_BUFFER_LENGTH  512          /* max. length of buffer in bytes */
#define MAX_RESULT_STRING  256          /* max. length of result string */
#define MAX_PARAMETERS     10            /* max. number of parameters that the parser will scan */
#define MAX_HELP_TXT_TX_WSIZE   2       /* max. tx window size for help text output */


/*----------------------------------------------------------------------------
 * cmd interface data structure
 *---------------------------------------------------------------------------*/

typedef struct _TestCmdIF
{
    char   cCommandLine[MAX_COMMAND_LINE];
    char   cBuffer[MAX_BUFFER_LENGTH];  /* work buffer */
    int    iCommandLength;              /* accumulated length of command       */
    char * pTxBuffer;                   /* buffer for echo of input            */
    int    iTxLength;                   /* < 0: tx buffer must be allocated    */
    int    iHelpTextSentIdx;            /* index of last help text string sent */
    bool   HelpTextInProgress;
    bool   SendPrompt;
    char   cPrompt[2];
    char   cCrLf[3];
} TTestCmdIF, * PTestCmdIF;


/*----------------------------------------------------------------------------
 * user cmd handler definitions
 *---------------------------------------------------------------------------*/
/*--------------------------------------------------------------------*/


typedef struct _TestModule  /* main structure */
{

  TTestCmdIF   CmdIF;


} TTestModule, *PTestModule;


/****************************************************************************/
/* define result codes of command collector                                 */
/****************************************************************************/

typedef enum _TTestResult
{
    test_ResultOk,
    test_ResultError,
    test_ResultEmptyCommandLine,
    test_ResultCommandNotFound,
    test_ResultWrongParameter,
    test_ResultWrongNumberOfParameters,
    test_ResultValueOutOfRange
} TTestResult;



/****************************************************************************/
/* This is the structure where the command line parser puts its result      */
/****************************************************************************/

typedef struct _TTestParseResult
{
    char      * pCommand;                    /* pointer to command */
    char      * pRestOfCommandLine;          /* pointer to rest of command line for manual inspection */
    int         iParameterCount;             /* number of found parameters */
    uint32_t    dwParameter[MAX_PARAMETERS]; /* automatically parsed parameters */
    char      * pParameter[MAX_PARAMETERS];  /* automatically parsed parameters */
} TTestParseResult, *PTestParseResult;


  /* functions that can be called from command table in gattdcmd.c */
typedef TTestResult (*TTestCmdFunc)(PGATTTest pGattTest,
                                            TTestParseResult *pParseResult);



bool test_CmdCollect( PTestModule pTestModule, char * pData, int iLength );

void test_CmdInit( PTestModule pTestModule );

BOOL test_module_register(TTestCmdFunc func, void *pParameter);

#endif
