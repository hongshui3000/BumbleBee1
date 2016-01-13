enum { __FILE_NUM__ = 0 };
/**********************************************************************!MA*
*
* $Header: /var/lib/cvs/sw/src/system/srp/src/hciapi/hciapi_csr.c,v 1.2 2013/11/22 11:21:07 mn Exp $
*
* File:        $RCSfile: hciapi_csr.c,v $
* Version:     $Name:
*
* Archive:     $Source: /var/lib/cvs/sw/src/system/srp/src/hciapi/hciapi_csr.c,v $
*
* ------------------------------------------------------------------------
* !MODULE      [  ]
* ------------------------------------------------------------------------
* !FILE        [  ]
* !PROGRAM     [  ]
* !VERSION     [$Name:]
* !GROUP       [  ]
* ------------------------------------------------------------------------
*
*          Copyright (c)           2006 Stollmann E+V GmbH
*                                  Mendelssohnstr. 15
*                                  22761 Hamburg
*                                  Phone: 040/89088-0
 *          The content of this file is Stollmann E+V GmbH confidential
*          All Rights Reserved
*
* ------------------------------------------------------------------------
* !DESCRIPTION
*
*       Support BlueMod+SR only
*       upload hardware specific patch files over HCI interface
*
* ------------------------------------------------------------------------
* !INDEX
*  ...
* ------------------------------------------------------------------------
* !CONTENTS
* ------------------------------------------------------------------------
* !INCLUDE_REFERENCES
* ------------------------------------------------------------------------
* !HISTORY
*  Date      Author          Comment
*  tt.mm.jj                  Initial revision
*  tt.mm.jj
* ------------------------------------------------------------------------
*
**********************************************************************!HE*/

/****************************************************************************
* includes
****************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <comuart.h>
#include "trace.h"
#include <common_defs.h>
#include <os_mem.h>
//#include <trace_binary.h>
#define HCIAPI_MID         BTRACE_MODULE_ID_A0
#define TRACE_MODULE_ID    HCIAPI_MID

#include "hciapi_csr.h"
#include <hci_code.h>
#include "hci_code_vendor.h"
#include <bt_infop.h>

#include "hciapi.h"

extern THciApi  tHciApi;
extern TComUart tComUart;

static int SyncTries;
static PInfoPageDataFormat  hciInfoPageDataFormat;


#define BT_SEQ_NO_FOR_INIT_PSKEYS  BT_INFOPAGE_NEXT_FREE_IDX

extern void *getInfopageBaseAddress(unsigned char idx);

/****************************************************************************/
/* FUNCTION NAME: infopageGetNextDataPacket                                 */
/* ------------------------------------------------------------------------ */
/* INPUT PARAMETER:                                                         */
/* ---------------------                                                    */
/* IN nInfopageIdx      : infopage index                                    */
/* IN fwID              : version of used firmware                          */
/* IN pDataRecord       : pointer to data packet                            */
/*                        if set to NULL - return first record              */
/*                        else get next after this pointer                  */
/* ------------------------------------------------------------------------ */
/* RETURN PARAMETER TYPE:                                                   */
/* ---------------------                                                    */
/* PInfoPageDataFormat  : pointer to next data record                       */
/****************************************************************************/

static PInfoPageDataFormat infopageGetNextDataPacket(uint8_t nInfopageIdx, uint16_t fwID, PInfoPageDataFormat pDataRecord)
{
  PBTInfoPage         pBTInfoPage         = NULL;
  PInfoPageDataFormat pInfoPageDataFormat = NULL;

  pBTInfoPage = (PBTInfoPage)getInfopageBaseAddress(nInfopageIdx);

  if ( NULL == pBTInfoPage )
    return NULL;

  if ( pBTInfoPage->ModuleHeader.hardwareType != BT_INFOPAGE_V2_00 )
    return NULL;

  if ( NULL == pDataRecord )
  {
    pInfoPageDataFormat = (PInfoPageDataFormat)&pBTInfoPage->firstRecord;
  }
  else
  {
    /* ?does the current record belong to the mentioned infopage */
    if ( (uint8_t *)pDataRecord < ((uint8_t *)&pBTInfoPage->firstRecord)
      || (((uint8_t *)pBTInfoPage) + pBTInfoPage->GlobalHeader.Length) <= (uint8_t *)pDataRecord )
    {
      return NULL;
    }
    pInfoPageDataFormat = pDataRecord;
  }

  while ( (NULL != pInfoPageDataFormat && pInfoPageDataFormat->fwID != fwID && pInfoPageDataFormat->fwID != 0x0000)
    || (pInfoPageDataFormat == pDataRecord) )
  {
    /* jump to the next data record */
    pInfoPageDataFormat = (PInfoPageDataFormat)(((uint8_t *)pInfoPageDataFormat)+sizeof(TInfoPageDataFormat)-1+pInfoPageDataFormat->lengthOfParam);

    /* ?does the record belong to the mentioned infopage */
    if ( (((uint8_t *)pBTInfoPage) + pBTInfoPage->GlobalHeader.Length) <= (uint8_t *)pInfoPageDataFormat )
    {
      pInfoPageDataFormat = NULL;
      break;
    }
  };

  return pInfoPageDataFormat;
}


/************************ Util Functions  ********************/

static void hciCSRSendDsCommand(uint16_t command, uint8_t *buf, uint16_t length)
{
  uint8_t *pBuffer;
  uint16_t pos;

  pBuffer = (uint8_t *)osMemoryClearAllocate(RAM_TYPE_DATA_ON, length + HCI_OFFSET);
  if (pBuffer == NULL)
  {
    return;
  }

  pos = 0;
  pBuffer[pos++] = CMD_PKT;
  SHORT2CHAR(pBuffer+pos, command); pos += 2;
  pBuffer[pos++] = (uint8_t)length;
  if (length)
    memcpy((void *)(pBuffer + pos), (void *)buf, length);

  hciApiDataSend(pBuffer, length + HCI_OFFSET);
}

static void hciCSRCommandNoParameter(uint16_t command)
{
  hciCSRSendDsCommand(command, NULL, 0);
}


#define N_MAX_SYNC_TRIES  5

/****************************************************************************/
/* sendCSRMessageSyncHwLayer                                                */
/* ------------------------------------------------------------------------ */
/* used for autobauding                                                     */
/****************************************************************************/

bool sendCSRMessageSyncHwLayer(void)
{
  uint8_t *mbuf;
  uint16_t pos = 0;

  if ( SyncTries > N_MAX_SYNC_TRIES )
  {
    SyncTries = 0;
    return false;
  }

  mbuf = (uint8_t *)osMemoryClearAllocate(RAM_TYPE_DATA_ON, 12);
  if (mbuf == NULL)
  {
    return false;
  }

  /* CSR 8811 A08 */
  mbuf[pos++] = 0x81;
  mbuf[pos++] = 0x70;
  mbuf[pos++] = 0x51;
  mbuf[pos++] = 0x1F;
  mbuf[pos++] = 0x68;
  mbuf[pos++] = 0x34;
  mbuf[pos++] = 0x91;
  mbuf[pos++] = 0x80;
  mbuf[pos++] = 0xD9;
  mbuf[pos++] = 0x8F;
  mbuf[pos++] = 0x48;
  mbuf[pos++] = 0x34;

  hciApiTimerStart(50);

  hciApiDataSend(mbuf, 12);

  return true;
}

/****************************************************************************
* sendCSRVenSpecDbgCmdBCCMD
****************************************************************************/

static void sendCSRVenSpecDbgCmdBCCMD(
                                      uint16_t msgType, uint16_t seqNo, uint16_t varID,
                                      const uint16_t *lpPayload,  uint16_t len  /* in uint16_t */,
                                      const uint16_t *lpPayload2, uint16_t len2 /* in uint16_t */
                                     )
{
  uint8_t  *mbuf;
  uint16_t pos;
  uint8_t  fill = 0;
  uint8_t  i;
  PCsrHciExtCommand pCsrHciExtCommand = NULL;

  if( (len+len2) < 4 ) /* the payload must be at least 4 uint16_t long */
    fill = 4 - (len+len2);

  mbuf = (uint8_t *)osMemoryClearAllocate(RAM_TYPE_DATA_ON, offsetof(TCsrHciExtCommand,payload.bccmd.msgPayload.payload) + (len+len2+fill)*sizeof(uint16_t));
  if (mbuf == NULL)
  {
    return;
  }

  pCsrHciExtCommand = (PCsrHciExtCommand)mbuf;

  pCsrHciExtCommand->header.hciType             = CMD_PKT;
  pCsrHciExtCommand->header.hciOpCode           = HCI_CSR_EXTN_PACKET;
  pCsrHciExtCommand->header.totalLength         = (uint8_t)(1 + (5+len+len2+fill)*sizeof(uint16_t));
  pCsrHciExtCommand->header.payloadDescriptor   = ( HCI_CSR_FIRST_FRAGMENT | HCI_CSR_LAST_FRAGMENT | HCI_CSR_VDBG_CHNL_ID_BCCMD);
  pCsrHciExtCommand->payload.bccmd.msgHeader.msgType    = msgType;
  pCsrHciExtCommand->payload.bccmd.msgHeader.msgLength  = (len+len2) + fill + HCI_CSR_VDBG_HEADER_SIZE;
  pCsrHciExtCommand->payload.bccmd.msgHeader.msgSeqNo   = seqNo;
  pCsrHciExtCommand->payload.bccmd.msgHeader.msgVarId   = varID;
  pCsrHciExtCommand->payload.bccmd.msgHeader.msgStatus  = HCI_CSR_VDBG_STATUS_OK;

  pos = offsetof(TCsrHciExtCommand,payload.bccmd.msgPayload.payload);
  for(i=0; i < len; i++)
  {
    SHORT2CHAR(mbuf+pos, lpPayload[i]);         pos+=2;
  }
  for(i=0; i < len2; i++)
  {
    SHORT2CHAR(mbuf+pos, lpPayload2[i]);        pos+=2;
  }
  while(fill--)
  {
    SHORT2CHAR(mbuf+pos, 0x0000);               pos+=2;
  }

  hciApiDataSend(mbuf, pos);
}


/****************************************************************************
* setCsrPSKey
****************************************************************************/

static void setCsrPSKey(uint16_t seqNo, uint16_t psKey, const uint16_t *lpPayload, uint16_t len /* in uint16_t */)
{
  uint16_t psKeyCmd[3];

  psKeyCmd[0] = psKey;
  psKeyCmd[1] = len;
  psKeyCmd[2] = HCI_CSR_PSKEY_STORAGE_DEFAULT;

  sendCSRVenSpecDbgCmdBCCMD( HCI_CSR_SETREQ, seqNo, HCI_CSR_BCCMDVARID_PSKEY_READWRITE, psKeyCmd, 3, lpPayload, len);
}


/****************************************************************************
* setCsrPSKeyWORD
****************************************************************************/

static void setCsrPSKeyWORD(uint16_t seqNo, uint16_t psKey, uint16_t val)
{
  uint16_t psKeyCmd[4];

  psKeyCmd[0] = psKey;
  psKeyCmd[1] = 1; /* len */
  psKeyCmd[2] = HCI_CSR_PSKEY_STORAGE_DEFAULT;
  psKeyCmd[3] = val;

  sendCSRVenSpecDbgCmdBCCMD( HCI_CSR_SETREQ, seqNo, HCI_CSR_BCCMDVARID_PSKEY_READWRITE, psKeyCmd, 4, NULL, 0);
}


/****************************************************************************
* initPhaseGetNextPskey
* returns TRUE if command was sent
****************************************************************************/

static bool initPhaseGetNextPskey(uint16_t seqNo)
{
  while(true)
  {
    switch ( seqNo )
    {
      case BT_SEQ_NO_FOR_INIT_PSKEYS:
        setCsrPSKeyWORD(seqNo+1, HCI_CSR_PSKEY_HOSTIO_USE_HCI_EXTN_CCFC, true);
        return true;

      case BT_SEQ_NO_FOR_INIT_PSKEYS+1:
        /* set PCM bus to slave and activate tri-state mode in PCM-OUT */
        {
          uint16_t csrPCMConfig32[2] = {0x0080, 0x0042};
          setCsrPSKey(seqNo+1, HCI_CSR_PSKEY_PCM_CONFIG32, csrPCMConfig32, 2);
        }
        return true;

      case BT_SEQ_NO_FOR_INIT_PSKEYS+2:
        /* change max acl packet size */
        setCsrPSKeyWORD(seqNo+1, HCI_CSR_PSKEY_H_HC_FC_MAX_ACL_PKT_LEN, 339);  /* Todo */
        return true;

      case BT_SEQ_NO_FOR_INIT_PSKEYS+3:
        /* change max count of acl packets */
        setCsrPSKeyWORD(seqNo+1, HCI_CSR_PSKEY_H_HC_FC_MAX_ACL_PKTS, 10);  /* Todo */
        return true;

      case BT_SEQ_NO_FOR_INIT_PSKEYS+81:
        /* disable H4DS in BB */
        setCsrPSKeyWORD(seqNo+1, HCI_CSR_PSKEY_HOST_INTERFACE, 3 /* H4 */);
        return true;

      case BT_SEQ_NO_FOR_INIT_PSKEYS+99:
        /* !!! HCI_CSR_PSKEY_UART_BAUDRATE must be the last sent cmd because if the command is completed a WARM_BOOT is generated */
        /* change baudrate */
        {
          uint16_t csrBC7upBR[2];
          csrBC7upBR[0] = (uint16_t)(COM_MAX_BAUDRATE >> 16);
          csrBC7upBR[1] = (uint16_t)(COM_MAX_BAUDRATE & 0xFFFF);
          setCsrPSKey(seqNo+1, HCI_CSR_PSKEY_UART_BITRATE, csrBC7upBR, 2);
        }
        return true;

      case BT_SEQ_NO_FOR_INIT_PSKEYS+100:
        return false;
    }
    seqNo++;
  } /* while(true) */
}


/****************************************************************************
* hciCsrPatchesAndPSKEYs
* returns TRUE if command was sent
****************************************************************************/

bool hciCsrPatchesAndPSKEYs(uint16_t seqNo)
{
  PInfoPageDataFormat       pInfoPageDataFormat;
  PCsrInfopageDataFormat    pCsrInfopageDataFormat;

  if ( seqNo == 0 || seqNo >= BT_INFOPAGE_NEXT_FREE_IDX )
  {
    return false;
  }

  pInfoPageDataFormat = hciInfoPageDataFormat;

  switch ( seqNo )
  {
    case BT_INFOPAGE2_IDX: /* get all parameter from infopage2 */
      pInfoPageDataFormat = infopageGetNextDataPacket(BT_INFOPAGE2_IDX, 0x2031/*Todo hciRevision*/, pInfoPageDataFormat);
      if ( NULL != pInfoPageDataFormat )
        break;
      seqNo = BT_INFOPAGE1_IDX;
      /* no break */

    case BT_INFOPAGE1_IDX: /* get all parameter from infopage1 */
      pInfoPageDataFormat = infopageGetNextDataPacket(BT_INFOPAGE1_IDX, 0x2031/*Todo hciRevision*/, pInfoPageDataFormat);
      break;

    default:
      return false;
  }

  hciInfoPageDataFormat = (PVOID)pInfoPageDataFormat;

  if ( NULL == pInfoPageDataFormat )  /* ?is patch process done */
  {
    return false;
  }

  pCsrInfopageDataFormat = (PCsrInfopageDataFormat)(&pInfoPageDataFormat->param[0]);

  switch ( pCsrInfopageDataFormat->pskeyId )
  {
    default:;
  }

  setCsrPSKey(seqNo, pCsrInfopageDataFormat->pskeyId, (const uint16_t*)&pCsrInfopageDataFormat->param[0],  pInfoPageDataFormat->lengthOfParam/sizeof(uint16_t) - 1 /* pskeyId */);

  return true;
}


/***************************************************************************
* handleCSRVenSpecDbgEventBCCMD
****************************************************************************/

void handleCSRVenSpecDbgEventBCCMD(uint8_t *lpPar, uint8_t len, bool *pbVendorSpecificInitPhaseActive)
{
  uint16_t pos,
           type,
           seqNo,
           varID;

  pos    = 1;

  type    = CHAR2SHORT(lpPar+pos); pos += 2;
  pos += 2;                                                        /* skip length field */
  seqNo   = CHAR2SHORT(lpPar+pos); pos += 2;
  varID   = CHAR2SHORT(lpPar+pos); pos += 2;

  pos += 2;                                                        /* jump over status */

  if((*lpPar & (HCI_CSR_FIRST_FRAGMENT | HCI_CSR_LAST_FRAGMENT)) == (HCI_CSR_FIRST_FRAGMENT | HCI_CSR_LAST_FRAGMENT))
  {
    if(type == HCI_CSR_GETRESP)
    {
      switch(varID)
      {
      case HCI_CSR_BCCMDVARID_CONFIG_UART:
        break;
      case HCI_CSR_BCCMDVARID_PSKEY_READWRITE:
        {
          uint16_t PSkey = CHAR2SHORT(lpPar+pos);
          switch(PSkey)
          {

          case HCI_CSR_PSKEY_UART_BAUDRATE:
          case HCI_CSR_PSKEY_UART_BITRATE:
            sendCSRVenSpecDbgCmdBCCMD(HCI_CSR_SETREQ, 0, HCI_CSR_BCCMDVARID_WARM_RESET, NULL, 0, NULL, 0);
            tComUart.BaudRate = COM_MAX_BAUDRATE; /* after this warm reset is the max uart baudrate active */
            hciApiChangeState(hciApiStateInitComplete);
            return ;

          default:
            break;
          }
        }
        break;
      default:
        break;
      } /* varID */
    } /* GET_RESP */
  }

  if ( *pbVendorSpecificInitPhaseActive )
  {
    if ( 0 < seqNo && seqNo < BT_INFOPAGE_NEXT_FREE_IDX )
    {
      if ( hciCsrPatchesAndPSKEYs(seqNo) == FALSE )  /* hciCsrPatchesAndPSKEYs returns false if done */
        seqNo = BT_SEQ_NO_FOR_INIT_PSKEYS;
    }
    if ( BT_SEQ_NO_FOR_INIT_PSKEYS <= seqNo )
    {
      if ( initPhaseGetNextPskey(seqNo) == FALSE )   /* initPhaseGetNextPskey returns false if done */
      {
        *pbVendorSpecificInitPhaseActive = FALSE;
      }
    }
  }

  return;
}


/*****************************************************************************/
/* Handle command complete                                                   */
/*****************************************************************************/

static void hciCSRHandleCommandComplete(uint8_t *pPacket, uint16_t pos, uint16_t Length)
{
  uint16_t Response;
/*  uint8_t  NumHCI; */

  /*NumHCI   =*/ pPacket[pos++];
  Response = CHAR2SHORT(pPacket+pos); pos += 2;

  switch (Response)
  {
    case HCI_RESET:
      if (tHciApi.State == hciApiStateInitializing)
      {
        hciCsrPatchesAndPSKEYs(2/*start with infopage2*/); /* hciCsrPatches returns true if any command transmitted */
      }
      break; /* HCI_RESET */

    default:
      DBG_BUFFER(MODULE_APP, LEVEL_ERROR,"hciCSRHandleCommandComplete: unexpected response",1, Response);
      break;
  }
}


/*****************************************************************************/
/* Handle commansd status                                                    */
/*****************************************************************************/

static void hciCSRHandleCommandStatus(uint8_t *pPacket, uint16_t pos, uint16_t Length)
{
  uint16_t Response;
/*  uint8_t  Status; */
/*  uint8_t  NumHCI; */

  /*Status   =*/ pPacket[pos++];
  /*NumHCI   =*/ pPacket[pos++];
  Response = CHAR2SHORT(pPacket+pos); pos += 2;

  switch (Response)
  {
    case HCI_NOOP:
      if (tHciApi.State == hciApiStateAutobaudTest)
      {
        hciApiTimerStop();

        hciApiTimerStart(1000);      /* max. 1 second */
        tHciApi.State = hciApiStateInitializing;
        hciCSRCommandNoParameter(HCI_RESET);
      }
      break; /* HCI_NOOP */

    default:
      DBG_BUFFER(MODULE_APP, LEVEL_ERROR,"hciCSRHandleCommandStatus: unexpected response",1, Response);
      break;
  }
}


/*****************************************************************************/
/* Handle event packet                                                       */
/*****************************************************************************/

void hciCsrHandleEventPacket(uint8_t *pPacket, uint16_t len)
{
  uint8_t  pos = 1;
  uint16_t Length;
  uint8_t  Event;

  Event  = pPacket[pos++];
  Length = pPacket[pos++];

  switch (Event)
  {
    case HCI_COMMAND_COMPLETE:
      hciCSRHandleCommandComplete(pPacket, pos, len);
      break; /* command complete */

    case HCI_COMMAND_STATUS:
      hciCSRHandleCommandStatus(pPacket, pos, len);
      break; /* command status */

    case HCI_VENDOR_SPECIFIC_DBG_EVENT:
      if (tHciApi.State == hciApiStateInitializing)
      {
        if( (pPacket[pos] & 0x3F) == HCI_CSR_VDBG_CHNL_ID_BCCMD)
        {
          bool bVendorSpecificInitPhaseActive = true;

          handleCSRVenSpecDbgEventBCCMD(&pPacket[pos], Length, &bVendorSpecificInitPhaseActive);
          if (bVendorSpecificInitPhaseActive == false)
          {
            /* Do nothing */
          }
        }
      }
      break; /* command status */

    default:
      DBG_BUFFER(MODULE_APP, LEVEL_ERROR,"hciCsrHandleEventPacket: unexpected event", 1,Event);
      break;
  }
}
