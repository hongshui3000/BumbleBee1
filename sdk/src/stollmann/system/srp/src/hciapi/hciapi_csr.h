/**********************************************************************!MA*
 *
 * $Header: /var/lib/cvs/sw/src/system/srp/src/hciapi/hciapi_csr.h,v 1.1 2013/10/17 09:28:16 mn Exp $
 *
 * File:        $RCSfile: hciapi_csr.h,v $
 * Version:     $Name:
 *
 * Archive:     $Source: /var/lib/cvs/sw/src/system/srp/src/hciapi/hciapi_csr.h,v $
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
 *          Copyright (c)           2013 Stollmann E+V GmbH
 *                                  Mendelssohnstr. 15D
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

#if !defined(__HCI_CSR_H)
#define      __HCI_CSR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "rtl_types.h"
/******************************************************************************/
/* Infopage stuff                                                             */
/******************************************************************************/

#define  __BASETYPE_H          /* no basetype.h */


	


#define IN
#define FARFUNC
#define FAR


/****************************************************************************
 * structure definitions
 ****************************************************************************/

/*** TCsrPSKEY ***/
typedef PACKED(struct) tCsrPSKEY {
  WORD  key;                      /* pskey identifier                                         */
  WORD  length;                   /* pskey identifier                                         */
  WORD  stores;                   /* destination story                                        */
  WORD  value[1];                 /* value if any                                             */
} TCsrPSKEY, *PCsrPSKEY;

/*** TCsrBCCMDHeader ***/
typedef PACKED(struct) tCsrBCCMDHeader {
  WORD  msgType;                  /* msg type eg. SET_REQ                                     */
  WORD  msgLength;                /* length of whole BCCMD massage in words                   */
  WORD  msgSeqNo;                 /* sequence number                                          */
  WORD  msgVarId;                 /* variable identifier                                      */
  WORD  msgStatus;                /* status                                                   */
} TCsrBCCMDHeader, *PCsrBCCMDHeader;

/*** TCsrBCCMDPayload ***/
typedef PACKED(union) tCsrBCCMDPayload {
  WORD        payload[1];
  TCsrPSKEY   pskey;
} TCsrBCCMDPayload, *PCsrBCCMDPayload;

/*** TCsrBCCMD ***/
typedef PACKED(struct) tCsrBCCMD {
  TCsrBCCMDHeader   msgHeader;
  TCsrBCCMDPayload  msgPayload;
} TCsrBCCMD, *PCsrBCCMD;

/*** TCsrHciExtCommandHeader ***/
typedef PACKED(struct) tCsrHciExtCommandHeader {
  BYTE        hciType;            /* HCI_CMD                                                  */
  WORD        hciOpCode;          /* HCI specific operation code = HCI_CSR_EXTN_PACKET=0xFC00 */
  BYTE        totalLength;        /* length of parameters below                               */
  BYTE        payloadDescriptor;  /* fragmentation information                                */
} TCsrHciExtCommandHeader, *PCsrHciExtCommandHeader;

/*** TCsrHciExtCommandPayload ***/
typedef PACKED(union) tCsrHciExtCommandPayload {
  BYTE        payload[1];
  TCsrBCCMD   bccmd;
} TCsrHciExtCommandPayload, *PCsrHciExtCommandPayload;

/*** TCsrHciExtCommand ***/
typedef PACKED(struct) tCsrHciExtCommand {
  TCsrHciExtCommandHeader   header;
  TCsrHciExtCommandPayload  payload;
} TCsrHciExtCommand, *PCsrHciExtCommand;



/*** TCsrInfopageDataFormat ***/
typedef PACKED(struct) tCsrInfopageDataFormat {
  WORD        pskeyId;
  WORD        param[1];
} TCsrInfopageDataFormat, *PCsrInfopageDataFormat;


/****************************************************************************
 * function declarations
 ****************************************************************************/
void handleCSRVenSpecDbgEventBCCMD(uint8_t *lpPar, uint8_t len, bool * pbVendorSpecificInitPhaseActive);
void sendCSRVenSpecDbgCmdBCCMD(
                                uint16_t msgType, uint16_t seqNo, uint16_t varID,
                                const uint16_t *lpPayload,  uint16_t len  /* in uint16_t */,
                                const uint16_t *lpPayload2, uint16_t len2 /* in uint16_t */
                              );

void switchBaudrateCSR            (int rate);

bool sendCSRMessageSyncHwLayer    (void);
bool hciCsrPatchesAndPSKEYs       (uint16_t seqNo);

void hciCsrHandleEventPacket      (uint8_t *pPacket, uint16_t Length);


#ifdef __cplusplus
}
#endif

#endif /* !defined(__HCI_CSR_H) */
