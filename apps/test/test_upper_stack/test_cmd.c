

#include "rtl_types.h"
#include <FreeRTOS.h>
#include <string.h>
#include <os_mem.h>

#include "test_cmd.h"
#include "test_transport_uart.h"
#include "app_queue.h"

#define test_CmdPrint(a, b...)             transport_uart_print(b)
#define test_UserIFSendString(a, b, c)     transport_uart_print("%s", b)
#define test_UserIFSendChar(a, b)          if (b != 0) transport_uart_print("%c", b)

//extern void *pvPortMalloc( size_t xSize,RAM_TYPE ramType ) ;

static QUEUE_T gAppTestCmdCallbackQueue;

typedef struct _ModuleCallBackFunc {
	struct TTestCmdFunc *pNext;
	TTestCmdFunc CheckFunc;
       void *pParameter;
}TModuleCallBackFunc;


BOOL test_module_register(TTestCmdFunc func, void *pParameter)
{
    if(func == NULL)
        return FALSE;

    TModuleCallBackFunc *item = (TModuleCallBackFunc *)osMemoryAllocate(RAM_TYPE_DATA_ON, sizeof(TModuleCallBackFunc));
    if(item == NULL)
        return FALSE;

    item->CheckFunc = func;
    item->pParameter = pParameter;   
    
    AppQueueIn(&gAppTestCmdCallbackQueue, item);

    return TRUE;
}


TTestResult test_module_execte_callback(TTestParseResult *pParseResult)
{
    TTestResult Result = test_ResultOk;
    TModuleCallBackFunc *item;
    TTestCmdFunc func;
    ELEMENT_P CurrentPtr = gAppTestCmdCallbackQueue.First;
    UINT16 ItemIndex = 0;



    for(ItemIndex = 0; ItemIndex < gAppTestCmdCallbackQueue.ElementCount; ++ItemIndex)
    {
        item = (TModuleCallBackFunc *)CurrentPtr;
        func = (TTestCmdFunc)((UINT32)(item->CheckFunc) | 0x1);  //thumb
        Result = func(item->pParameter, pParseResult);
        CurrentPtr = CurrentPtr->Next;
    }
    return Result;
}



/*----------------------------------------------------
 * clear command line buffer
 * --------------------------------------------------*/

static void test_CmdClearCommand( PTestModule pTestModule )
{
    pTestModule->CmdIF.iCommandLength = 0;
    memset( pTestModule->CmdIF.cCommandLine, 0,
            sizeof(pTestModule->CmdIF.cCommandLine) );
}


/****************************************************************************/
/* Send result                                                              */
/****************************************************************************/

static void test_CmdSendResult(PTestModule pTestModule, TTestResult Result)
{
    switch (Result)
    {
    case test_ResultError:
        test_UserIFSendString(pTestModule, "Error\n\r", NULL);
        break;
    case test_ResultCommandNotFound:
        test_UserIFSendString(pTestModule, "Command not found\n\r", NULL);
        break;
    case test_ResultWrongNumberOfParameters:
        test_UserIFSendString(pTestModule, "Wrong number of parameters\n\r", NULL);
        break;
    case test_ResultWrongParameter:
        test_UserIFSendString(pTestModule, "Wrong parameter\n\r", NULL);
        break;
    case test_ResultValueOutOfRange:
        test_UserIFSendString(pTestModule, "Value out of range\n\r", NULL);
        break;
    default:
        break;
    }
}

/*----------------------------------------------------------------------------
 * flush buffer with echo characters
 * -------------------------------------------------------------------------*/

static void test_CmdFlushEchoChars( PTestModule pTestModule, PTestCmdIF pCmdIF )
{
    if ( pCmdIF->iTxLength > 0 )
    {
        test_UserIFSendChar(pTestModule, 0);     /* flush buffer with echo data */
        pCmdIF->iTxLength   = 0;
        pCmdIF->pTxBuffer   = NULL;
    }
}

/****************************************************************************/
/* Check, if a character is a white space                                   */
/****************************************************************************/

static bool test_CmdIsWhiteSpace(char c)
{
    return (((c >= 9) && (c <= 13)) || (c == 32));
}


/****************************************************************************/
/* Skip white spaces in buffer                                              */
/****************************************************************************/

static char *test_CmdSkipSpaces(char *buffer)
{
    char *p = buffer;

    while (test_CmdIsWhiteSpace(*p)) /* white space */
    {
        p++;
    }
    return p;
}


/****************************************************************************/
/* Find end of a word                                                       */
/****************************************************************************/

static char *test_CmdFindEndOfWord(char *buffer)
{
    char *p = buffer;

    while (!test_CmdIsWhiteSpace(*p) && (*p != '\0'))
    {
        p++;
    }
    return p;
}


/****************************************************************************/
/* Read ASCII string and convert to uint32_t                                   */
/****************************************************************************/

static uint32_t test_CmdString2uint32_t(char *p)
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
 * Parse a command line and return the found
 * command and parameters in "pParseResult"
 * -------------------------------------------------------------------------*/

static TTestResult test_CmdParse( PTestModule pTestModule,
        TTestParseResult * pParseResult )
{
    int              i;
    TTestResult  Result;
    PTestCmdIF   pCmdIF;
    char            *p, *q;

    pCmdIF  = &pTestModule->CmdIF;

    /* clear all results */
    Result = test_ResultOk;
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
    p = test_CmdSkipSpaces(p);
    if ( *p == '\0')                      /* empty command line ? */
    {
        Result = test_ResultEmptyCommandLine;
    }
    else
    {
        /* find end of word */
        q = test_CmdFindEndOfWord( p);
        if (p == q)                        /* empty command line ? */
        {
            Result = test_ResultEmptyCommandLine;
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
                    p = test_CmdSkipSpaces(p);
                    d = test_CmdString2uint32_t(p);

                    pParseResult->pParameter[j]    = p;
                    pParseResult->dwParameter[j++] = d;

                    if ( j >= MAX_PARAMETERS)
                    {
                        break;
                    }

                    /* find next parameter */
                    p  = test_CmdFindEndOfWord( p);
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

bool test_CmdCollect( PTestModule pTestModule, char * pData, int iLength )
{
    TTestParseResult ParseResult;
    PTestCmdIF       pCmdIF;
    char                 c;

    pCmdIF              = &pTestModule->CmdIF;
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
                test_CmdFlushEchoChars( pTestModule, pCmdIF );
                pCmdIF->SendPrompt = true;

                test_UserIFSendString(pTestModule, pCmdIF->cCrLf, NULL);
                if (pCmdIF->iCommandLength > 0)  /* at least one character in command line ? */
                {
                    TTestResult Result;

                    pCmdIF->cCommandLine[pCmdIF->iCommandLength] = '\0';
                    Result = test_CmdParse(pTestModule, &ParseResult);
                    if (Result == test_ResultOk)
                        Result = test_module_execte_callback(&ParseResult);

                    if (Result != test_ResultOk)
                        test_CmdSendResult(pTestModule, Result);
                }

                if (pCmdIF->SendPrompt)
                    test_UserIFSendString(pTestModule, pCmdIF->cPrompt, NULL);

                test_CmdClearCommand( pTestModule );

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
                test_UserIFSendChar(pTestModule, c);
                test_UserIFSendChar(pTestModule, ' ');
                test_UserIFSendChar(pTestModule, c);
                break;
            default:
                /* Put character in command buffer */
                if (pCmdIF->iCommandLength < MAX_COMMAND_LINE)
                {
                    test_UserIFSendChar(pTestModule, c);
                    pCmdIF->cCommandLine[pCmdIF->iCommandLength++] = c;
                }
                break;
            }
        }
    }

    test_CmdFlushEchoChars( pTestModule, pCmdIF );

    return ( true );
}

/*----------------------------------------------------------------------------
 * init. cmd set
 * -------------------------------------------------------------------------*/

void test_CmdInit( PTestModule pTestModule )
{
    memset( &pTestModule->CmdIF, 0, sizeof(TTestCmdIF) );
    pTestModule->CmdIF.cCrLf[0]   = '\r';
    pTestModule->CmdIF.cCrLf[1]   = '\n';
    pTestModule->CmdIF.cPrompt[0] = '>';


}


