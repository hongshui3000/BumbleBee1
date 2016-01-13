/**********************************************************************!KA****
 *
 * $Header: /var/lib/cvs/sw/src/app/demo/gattdemo/gattdcmd.h,v 1.4 2013/04/19 14:09:38 mj Exp $
 *
 * File:        $RCSfile: gattdcmd.h,v $
 * Version:     $Name: P_SRP1290_V1_0 $
 *
 * Archive:     $Source: /var/lib/cvs/sw/src/app/demo/gattdemo/gattdcmd.h,v $
 * Revision:    $Revision: 1.4 $
 * Date:        $Date: 2013/04/19 14:09:38 $
 * Author:      $Author: mj $
 *
 * ---------------------------------------------------------------------------
 * !MODULE      [  ]
 * ---------------------------------------------------------------------------
 * !FILE        [  ]
 * !PROGRAM     [  ]
 * !VERSION     [$Name: P_SRP1290_V1_0 $]
 * !GROUP       [  ]
 * !AUTHOR      [$Author: mj $]
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
 *         GATTDEMO command definitions
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

#ifndef __BQB_CMD_H
#define __BQB_CMD_H


/****************************************************************************/
/* define result codes of command collector                                 */
/****************************************************************************/

typedef enum _TGATTDemoResult
{
    gattdResultOk,
    gattdResultError,
    gattdResultEmptyCommandLine,
    gattdResultCommandNotFound,
    gattdResultWrongParameter,
    gattdResultWrongNumberOfParameters,
    gattdResultValueOutOfRange
} TGATTDemoResult;

#define MAX_COMMAND_LINE   80           /* max. length of command line in bytes */
#define MAX_BUFFER_LENGTH  256          /* max. length of buffer in bytes */
#define MAX_RESULT_STRING  256          /* max. length of result string */
#define MAX_PARAMETERS     10            /* max. number of parameters that the parser will scan */
#define MAX_HELP_TXT_TX_WSIZE   2       /* max. tx window size for help text output */


/****************************************************************************/
/* This is the structure where the command line parser puts its result      */
/****************************************************************************/

typedef struct _TGATTDemoParseResult
{
    char      * pCommand;                    /* pointer to command */
    char      * pRestOfCommandLine;          /* pointer to rest of command line for manual inspection */
    int         iParameterCount;             /* number of found parameters */
    uint32_t    dwParameter[MAX_PARAMETERS]; /* automatically parsed parameters */
    char      * pParameter[MAX_PARAMETERS];  /* automatically parsed parameters */
} TGATTDemoParseResult;


/*----------------------------------------------------------------------------
 * cmd interface data structure
 *---------------------------------------------------------------------------*/

typedef struct _GATTDemoCmdIF
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
} TGATTDemoCmdIF, * PGATTDemoCmdIF;


#endif /* __BQB_CMD_H */

