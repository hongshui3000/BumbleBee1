/**********************************************************************!KA****
 *
 * $Header: /var/lib/cvs/sw/src/system/srp/src/hciapi/hciapi.h,v 1.2 2013/10/22 11:46:06 mn Exp $
 *
 * File:        $RCSfile: hciapi.h,v $
 * Version:     $Name: P_SRP1290_V1_0 $
 *
 * Archive:     $Source: /var/lib/cvs/sw/src/system/srp/src/hciapi/hciapi.h,v $
 * Revision:    $Revision: 1.2 $
 * Date:        $Date: 2013/10/22 11:46:06 $
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
 *          Copyright (c)           2013 Stollmann E+V GmbH
 *                                  Mendelssohnstr. 15d
 *                                  22761 Hamburg
 *                                  Phone: 040/89088-0
 *          The content of this file is Stollmann E+V GmbH confidential
 *          All Rights Reserved
 *
 * ---------------------------------------------------------------------------
 * !DESCRIPTION
 *
 *         Example HCI Api Task implementation
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
 ************************************************************************!KA*/


#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <timers.h>

#include <comapi.h>


/****************************************************************************/
/* State                                                                    */
/****************************************************************************/

typedef enum tHciApiState
{
  hciApiStateIdle,
  hciApiStateAutobaudTest,
  hciApiStateInitializing,
  hciApiStateInitComplete,
  hciApiStateReady,
  hciApiStateCount
} THciApiState;


/****************************************************************************/
/* Macro Definitions                                                        */
/****************************************************************************/

/* Get int16_t from 2 CHARS, Low-Endian format */
#define CHAR2SHORT(p) (((*(p)) & 0xff) + ((*((p)+1)) << 8))
#define SHORT2CHAR(p,w)                 \
    *((uint8_t *)p)     = (uint8_t)((w) & 0xff);      \
    *(((uint8_t *)p)+1) = /*lint -e(572,778)*/ (uint8_t)(((w)>>8) & 0xff)

/* Get LONG from 4 CHARS, Low-Endian format */
#define CHAR2LONG(p) (((uint32_t)(*((p)+0)) & 0xff) + ((uint32_t)(*((p)+1)) << 8) \
                   + ((uint32_t)(*((p)+2)) << 16)  + ((uint32_t)(*((p)+3)) << 24))

#define LONG2CHAR(p,w)                      \
    *((uint8_t *)p)     = (uint8_t)((w) & 0xff);          \
    *(((uint8_t *)p)+1) = (uint8_t)(((w)>>8) & 0xff);     \
    *(((uint8_t *)p)+2) = (uint8_t)(((w)>>16) & 0xff);    \
    *(((uint8_t *)p)+3) = (uint8_t)(((w)>>24) & 0xff);

#define HCI_OFFSET                      4 /* Nbr of free Bytes for HCI Packet Type Header */


/****************************************************************************/
/* HCI header                                                               */
/****************************************************************************/

#define EVENT_HDR_LENGTH 3 /* packet type (1), event type (1), length (1) */
#define ACL_HDR_LENGTH   5 /* packet type (1), handle (2), length (2) */


/****************************************************************************/
/* data                                                                     */
/****************************************************************************/

typedef struct _THciApi
{
  THciApiState    State;
  xTaskHandle     Handle;                /* task handle */
  xQueueHandle    QueueHandleEvent;      /* task queue */
  xQueueHandle    QueueHandleTxData;     /* Tx queue */
  xQueueHandle    QueueHandleRxData;     /* Rx queue */
  xTimerHandle    TimerHandle;           /* Timer */

  void            *pContext;
  PComCallBack    CallBack;

  uint16_t        DataIndication;        /* pending responses */
  bool            DataIndicationBlocked;

  TComData        RxData;

  uint16_t        HCIPos;
  uint16_t        HCIHeaderLength;
  uint16_t        HCIDataLength;
  uint8_t         HCIBuffer[ACL_HDR_LENGTH];     /* temp. space */
} THciApi;
typedef THciApi *PHciApi;


/****************************************************************************/
/* Prototypes                                                               */
/****************************************************************************/

/* hciapitask_FreeRTOS.C */
void hciApiFatalError(int Line);
void hciApiTimerStart(int MSecond);
void hciApiTimerStop(void);
void hciApiChangeState(THciApiState NewState);
void hciApiDataSend(uint8_t *pBuffer, uint32_t Length);
