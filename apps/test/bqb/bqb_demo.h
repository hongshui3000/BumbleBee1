/**********************************************************************!MA*
 *
 * $Header: /var/lib/cvs/sw/src/app/demo/gattdemo/gattdemo.h ,v 1.62.2.1 2013/10/17 09:51:18 mn Exp $
 *
 * File:        $RCSfile: gattdemo.h ,v $
 * Version:     $Name:
 *
 * Archive:     $Source: /var/lib/cvs/sw/src/app/demo/gattdemo/gattdemo.h ,v $
 * Revision:    $Revision: 1.62.2.1 $
 * Date:        $Date: 2013/10/17 09:51:18 $
 * Author:      $Author: mn $
 *
 * ------------------------------------------------------------------------
 * !MODULE      [  ]
 * ------------------------------------------------------------------------
 * !FILE        [  ]
 * !PROGRAM     [  ]
 * !VERSION     [  ]
 * !GROUP       [  ]
 * !AUTHOR      [  ]
 * ------------------------------------------------------------------------
 *
 *          Copyright (c)           2011 Stollmann E+V GmbH
 *                                  Mendelssohnstrasse 15
 *                                  22761 Hamburg
 *                                  Phone: 040/89088-0
 *          The content of this file is Stollmann E+V GmbH confidential
 *          All Rights Reserved
 *
 * ------------------------------------------------------------------------
 * !DESCRIPTION
 *          GATTDEMO definitions.
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

#ifndef __BQB_DEMO_H
#define __BQB_DEMO_H

#include <blueapi_types.h>
#include <bterrcod.h>
#include "bqb_cmd.h"
#include "bqb.h"
#include <gatt.h>


/*--- operation mode(s) ---*/
#define GATTDEMO_APPLIC_PROVIDED_SERVICES   1  /* 1: use own service definitions    */
/* 0: use BT-stack built-in services */

#define GATTSRV_GLS 1

typedef uint8_t     TIMESTAMP[7];
typedef uint8_t     SFLOAT[2];         /* 4 bit (MSB) exponent, 12 bit mantissa */
/*--- common definitions for all services/characteristics ---*/


#define SFLOAT_VALUE_NaN             0x07ff    /* not a number             */
#define SFLOAT_VALUE_NRes            0x0800    /* not at this resolution   */
#define SFLOAT_VALUE_PlusINFINITY    0x07fe    /* + INFINITY               */
#define SFLOAT_VALUE_MinusINFINITY   0x0802    /* - INFINITY               */
#define SFLOAT_VALUE_RFU             0x0801    /* reserved for future use  */

/* --- */
#define BYTE_0(x)   LO_WORD(x)
#define BYTE_1(x)   HI_WORD(x)
#define BYTE_2(x)   ((uint8_t)((x & 0xFF0000) >> 16))
#define BYTE_3(x)   ((uint8_t)((x & 0xFF000000) >> 24))


/*--- services defined in build process (must be used mutually exclusive!): ---*/

/*--- fatal error handler ---*/
#define GATTDEMO_FATAL_ERROR()  DebuggerBreak();

/*--- UART device ---*/
#define BQB_USART_DEVICE_NBR     0



#define BQB_BD_BUFFER_SIZE   256  /* size of buffer for BD inquiry results */

/*--- attribute handle to (16 bit) UUID mapping ---*/
#if !defined(BQB_MAX_HANDLE_UUID)
#define  BQB_MAX_HANDLE_UUID   16
#endif
typedef struct _GATTDHandleUUID
{
    uint16_t  wHandle;
    uint16_t  wUUID;
} TGATTDHandleUUID, * PGATTDHandleUUID;


/* BD address */
typedef uint8_t TGATTDBdAddr[BLUE_API_BD_SIZE];
typedef TGATTDBdAddr  * PGATTDBdAddr;


/*----------------------------------------------------------------------------
 * main instance data structure
 *---------------------------------------------------------------------------*/

/* OSIF target specific internal data structure, will be setup by */
/* library routine. Never change the content of this structure !! */
typedef struct _GATTDTgt        //XXXXMJMJ needed ????
{
    uint16_t    wQueueID;
} TGATTDTgt, * PGATTDTgt;

/* common service related data */
typedef struct _GATTDService
{
    bool                    Used;
    BQB_GATT_ServiceId      ServiceID;
    void *                  pServiceHandle;  /* service handle provided by GATT server */
} TGATTDService, * PGATTDService;


#if defined(GATTDEMO_MAIN_SERVICE_INDEX)
/* defined in service/profile specific header file */
#else
#define GATTDEMO_MAIN_SERVICE_INDEX  0     /* service used in update command */
#endif




/* attribute index / client characteristic configuration descriptor (CCCD) pair */
typedef struct _GATTDAttrIdxCCCD
{
    uint16_t    wAttribIndex;      /* index of CCCD in struct gattdProfile[] */
    uint16_t    wCCCBits;
} TGATTDAttrIdxCCCD, * PGATTDAttrIdxCCCD;

#define GATTDEMO_MAX_LE_CHANNELS  3
typedef struct _GATTLEChannel
{
    bool             isUsed;
    bool             isDataChanConnected;
    uint16_t         channel;
} TGATTLEChannel, *PGATTLEChannel;


#define BQB_MAX_LINKS  4
#define BQB_CON_ROLE_UNDEFINED 0
#define BQB_CON_ROLE_MASTER 1
#define BQB_CON_ROLE_SLAVE 2
typedef struct _GATTLink
{
    bool             isUsed;
    bool             Connected;
    uint16_t         role;  //0:undefined  1:master  2:slave
    TGATTDBdAddr     RemoteBd;        /* current remote BD */
    bool             RemoteBdValid;
    uint16_t         idx;
    uint16_t         local_MDL_ID;
    bool             local_MDL_ID_Valid;

    uint16_t         wDsCredits;      /* downstream (tx) credits for WriteCommand */
    /* and Notifications                        */
    uint16_t         wMTUSize;          /* ATT layer MTU size */

    uint16_t         wEndingHandle;   /* current discovery ending handle */
    int            iHandleCnt;
    TGATTDHandleUUID HandleUUID[BQB_MAX_HANDLE_UUID];
    TGATTLEChannel   LEChannel[GATTDEMO_MAX_LE_CHANNELS];
} TGATTLink, *PGATTLink;


/* external data store definitions */
#define GATTDSTORE_ENTRY_COUNT    0
#define GATTDSTORE_NVRAM_ADDRESS  0x1000


/* some services perform specific init. when connection is established: */
typedef void (* TGATTDConnectedInd)(void * pGATTDemo);

#define BQB_MAX_PREPARE_QUEUE  6
#define BQB_MAX_PREPARE_VALUE_LENGTH 18 //MTU size = 23

#if (BQB_PREPARE_WRITE_SUPPPRT)
typedef struct _GATTPrepareWrite
{
  void *         serviceHandle;
  uint16_t       attribIndex;
  uint16_t       handle;
  uint16_t       writeOffset;        /* write offset in attribute */
  uint16_t       attribLength;
  uint8_t        data[BQB_MAX_PREPARE_VALUE_LENGTH];
} TGATTPrepareWrite, *PGATTPrepareWrite;
#endif

/*--------------------------------------------------------------------*/
typedef struct _GATTDemo  /* main structure */
{
    /* cmd interface data */
    TGATTDemoCmdIF   CmdIF;


    bool             initLEServer;
    bool             advertiseOnDisc;    /* true: restart advertising when disconnected */


#if (BQB_PREPARE_WRITE_SUPPPRT)
      //fixme: now just support one link for prepareQueue
      TGATTPrepareWrite  prepareQueue[BQB_MAX_PREPARE_QUEUE];
      int                queueIdx;
      uint16_t           queue_local_MDL_ID;
      //save handle to prepare write
      uint16_t pwrite_handle;
      uint16_t pwrite_attri_len;
#endif

    int              iServiceCount;
    TGATTDService    Service[BQB_GATT_SUPPORT_SERVICE_MAX];
    uint16_t         wUpdReqHandle;   /* attribute update request handle */
    /* (NOT attribute handle !!!)      */



    uint8_t          bCCCBits[sizeof(uint16_t)];  /* current CCCBits written .. */

    /* inquiry result data (Bluetooth device addresses ..) */
    int              iBDCount;        /* nbr. of BD addresses        */
    uint8_t          BDBuffer[BQB_BD_BUFFER_SIZE];  /* buffer storing BD addresses */

    /* connection related data */
    TGATTLink        linkTable[BQB_MAX_LINKS];

    uint16_t         wDsPoolId;       /* buffer pool id for connection related  */
    /* downstream data                        */
    uint16_t         wDsDataOffset;   /* data offset in downstream data buffer  */

    TGATTDConnectedInd  ConnectedInd;    /* <> NULL: called when connected */


    /* attribute handle to (16 bit) UUID mapping */



#if (GATTDEMO_CCCD_COUNT)
    /* array of CCCD attribute index/value pairs (needed to allow application */
    /* to verify that all CCCD bits are properly set by client).              */
    /* XXXXMJMJ OK only for one service !!! if needed for more array must be  */
    /* moved to service specific sub-struct TGATTDService Service[] ... !!!   */
    TGATTDAttrIdxCCCD    AttrIdxCCCD[GATTDEMO_CCCD_COUNT];
#endif

    /* attribute value update sequence control */
    int              iUpdateServiceIndex;
    int              iUpdateAttribIndex;
    int              iUpdateCnt;
    int              iUpdateSent;
    int              iUpdateConfirmed;


    /* service specifics */

    //TGLCRACP         RACP;    /* Record Access Control Point related data */




    /* miscellaneous */
    bool             ShortOutput;     /* true: reduce output (buffer limitations ..) */


} TGATTDemo, * PGATTDemo;


/*----------------------------------------------------------------------------
 * user cmd handler definitions
 *---------------------------------------------------------------------------*/

/* functions that can be called from command table in gattdcmd.c */
typedef TGATTDemoResult (*TGATTDemoCmdFunc)(PGATTDemo pGATTDemo,
        TGATTDemoParseResult *pParseResult);

/* a single command table entry */
typedef struct _TGATTDemoCmdTableEntry
{
    char              * pCommand;
    char              * pOption;
    char              * pHelp;
    TGATTDemoCmdFunc    Func;
} TGATTDemoCmdTableEntry;


/*------ attribute write indication post processing (needed for some -----*/
/*------ services that have a Record Access Control Point (RACP)     -----*/

typedef void (* TBQBGATTDWriteIndPostProc)(PGATTDemo pGATTDemo, uint16_t wLength, uint8_t * pValue);


/*---------------------------------------------------------------------------
 * prototypes of internal routines
 *--------------------------------------------------------------------------*/

bool   BQB_CmdCollect( PGATTDemo pGATTDemo, char * pData, int iLength );

/*-- gattdutl.c: GATT utilities --*/
const char * BQB_ErrorCode2Str( uint16_t wCause );
extern void BQB_HexDump(PGATTDemo pGATTDemo, char *pTxt, int iSize, uint8_t * pValue);
char * BQB_SFLOAT2StringAndWORD( uint16_t * pwSFLOATValue );
void   BQB_PrintSFLOAT( PGATTDemo pGATTDemo, char *pName, uint16_t wValue );
void   BQB_PrintTimestamp( PGATTDemo pGATTDemo, uint8_t * pTimestamp, char * pTxt );
char * BQB_UUID16ToString( uint16_t wUUID16 );

PGATTDService BQB_ServiceFind( PGATTDemo pGATTDemo, void * pServiceHandle );

#if (GATTDEMO_CCCD_COUNT)
void   BQB_CCCDInitTable( PGATTDemo pGATTDemo );
uint16_t  BQB_CCCDCheckValue( PGATTDemo pGATTDemo, int iStart, int iEnd );
#endif


/*-- gatdbapi.c: BlueAPI interface routines --*/
TGATTDemoResult BQB_Register(PGATTDemo pGATTDemo,
                             TGATTDemoParseResult *pParseResult);

TGATTDemoResult BQB_Release(PGATTDemo pGATTDemo,
                            TGATTDemoParseResult *pParseResult);
TGATTDemoResult BQB_Inquiry(PGATTDemo pGATTDemo,
                            TGATTDemoParseResult *pParseResult);
TGATTDemoResult BQB_NameDiscovery(PGATTDemo pGATTDemo,
                                  TGATTDemoParseResult *pParseResult);
TGATTDemoResult BQB_GATTSecurityReq(PGATTDemo pGATTDemo,
                                    TGATTDemoParseResult *pParseResult);
TGATTDemoResult BQB_ServiceRegister(PGATTDemo pGATTDemo,
                                    TGATTDemoParseResult *pParseResult);
TGATTDemoResult BQB_ServiceRelease(PGATTDemo pGATTDemo,
                                   TGATTDemoParseResult *pParseResult);
TGATTDemoResult BQB_AttribUpdate(PGATTDemo pGATTDemo,
                                 TGATTDemoParseResult *pParseResult);
int             BQB_AttribUpdateContinue( PGATTDemo pGATTDemo );



TGATTDemoResult BQB_CmdSetAdvertisingEnable(PGATTDemo pGATTDemo,
        TGATTDemoParseResult *pParseResult);
TGATTDemoResult BQB_CmdSetAdvertisingParameters(PGATTDemo pGATTDemo,
        TGATTDemoParseResult *pParseResult);
TGATTDemoResult BQB_CmdSetDirectedAdvertising(PGATTDemo pGATTDemo,
        TGATTDemoParseResult *pParseResult);
TGATTDemoResult BQB_SetAdvertisingData(PGATTDemo pGATTDemo,
                                       TGATTDemoParseResult *pParseResult);



TGATTDemoResult BQB_ConUpdateReq(PGATTDemo pGATTDemo,
                                 TGATTDemoParseResult *pParseResult);
TGATTDemoResult BQB_ConUpdateResp(PGATTDemo pGATTDemo,
                                  TGATTDemoParseResult *pParseResult);
TGATTDemoResult BQB_CmdAddToWhitelist(PGATTDemo pGATTDemo,
                                      TGATTDemoParseResult *pParseResult);
TGATTDemoResult BQB_CmdRemoveFromWhitelist(PGATTDemo pGATTDemo,
        TGATTDemoParseResult *pParseResult);
TGATTDemoResult BQB_CmdClearWhitelist(PGATTDemo pGATTDemo,
                                      TGATTDemoParseResult *pParseResult);


TGATTDemoResult BQB_CmdConLEChan(PGATTDemo pGATTDemo,
                                          TGATTDemoParseResult *pParseResult);
TGATTDemoResult BQB_CmdDiscLEChan(PGATTDemo pGATTDemo,
                                          TGATTDemoParseResult *pParseResult);
TGATTDemoResult BQB_CmdCredit(PGATTDemo pGATTDemo,
                                          TGATTDemoParseResult *pParseResult);
TGATTDemoResult BQB_CmdLEData(PGATTDemo pGATTDemo,
                                          TGATTDemoParseResult *pParseResult);
TGATTDemoResult BQB_CmdLESec(PGATTDemo pGATTDemo,
                                          TGATTDemoParseResult *pParseResult);
TGATTDemoResult BQB_CmdScBqb(PGATTDemo pGATTDemo,
                                          TGATTDemoParseResult *pParseResult);

TGATTDemoResult BQB_ConnectReq(PGATTDemo pGATTDemo, TGATTDemoParseResult *pParseResult);
TGATTDemoResult BQB_DisconnectReq(PGATTDemo pGATTDemo, TGATTDemoParseResult *pParseResult);
TGATTDemoResult BQB_ServiceDiscovery(PGATTDemo pGATTDemo, TGATTDemoParseResult *pParseResult);
TGATTDemoResult BQB_RelationshipDiscovery(PGATTDemo pGATTDemo, TGATTDemoParseResult *pParseResult);
TGATTDemoResult BQB_CharacteristicDiscovery(PGATTDemo pGATTDemo, TGATTDemoParseResult *pParseResult);
TGATTDemoResult BQB_CharacDescriptorDiscovery(PGATTDemo pGATTDemo, TGATTDemoParseResult *pParseResult);
TGATTDemoResult BQB_ListUUIDs(PGATTDemo pGATTDemo, TGATTDemoParseResult *pParseResult);
TGATTDemoResult BQB_AttribRead(PGATTDemo pGATTDemo, TGATTDemoParseResult *pParseResult);
TGATTDemoResult BQB_AttribReadUUID(PGATTDemo pGATTDemo, TGATTDemoParseResult *pParseResult);
TGATTDemoResult BQB_AttribWrite(PGATTDemo pGATTDemo, TGATTDemoParseResult *pParseResult);
TGATTDemoResult BQB_AttribWriteEx(PGATTDemo pGATTDemo, TGATTDemoParseResult *pParseResult);
TGATTDemoResult BQB_CmdScanEnable(PGATTDemo pGATTDemo, TGATTDemoParseResult *pParseResult);

void BQB_HandleBlueAPIMessage            (PGATTDemo pGATTDemo, PBlueAPI_UsMessage pMsg);
void BQB_Handle_AuthResultInd            (PGATTDemo pGATTDemo, PBlueAPI_AuthResultInd pAuthResultInd);
void BQB_Handle_PairableModeSetRsp       (PGATTDemo pGATTDemo, PBlueAPI_PairableModeSetRsp pPairableModeSetRsp);
void BQB_Handle_UserPasskeyReqInd        (PGATTDemo pGATTDemo, PBlueAPI_UserPasskeyReqInd pUserPasskeyReqInd);
void BQB_Handle_UserPasskeyNotificationInfo(PGATTDemo pGATTDemo, PBlueAPI_UserPasskeyNotificationInfo pUserPasskeyNotificationInfo);
void BQB_Handle_UserPasskeyReqReplyRsp   (PGATTDemo pGATTDemo, PBlueAPI_UserPasskeyReqReplyRsp pUserPasskeyReqReplyRsp);
void BQB_Handle_RemoteOOBDataReqInd      (PGATTDemo pGATTDemo, PBlueAPI_RemoteOOBDataReqInd pRemoteOOBDataReqInd);

/* security specific stuff: gatdbapisec.c */
TGATTDemoResult BQB_CmdDevCfg            (PGATTDemo pGATTDemo, TGATTDemoParseResult *pParseResult);

TGATTDemoResult BQB_CmdNvClear           (PGATTDemo pGATTDemo, TGATTDemoParseResult *pParseResult);
TGATTDemoResult BQB_CmdNvShow            (PGATTDemo pGATTDemo, TGATTDemoParseResult *pParseResult);
TGATTDemoResult BQB_CmdPrivacyMode       (PGATTDemo pGATTDemo, TGATTDemoParseResult *pParseResult);
TGATTDemoResult BQB_CmdBrMode            (PGATTDemo pGATTDemo, TGATTDemoParseResult *pParseResult);
TGATTDemoResult BQB_CmdBrAuth            (PGATTDemo pGATTDemo, TGATTDemoParseResult *pParseResult);
TGATTDemoResult BQB_CmdPairableMode      (PGATTDemo pGATTDemo, TGATTDemoParseResult *pParseResult);
TGATTDemoResult BQB_CmdAuthConfirm       (PGATTDemo pGATTDemo, TGATTDemoParseResult *pParseResult);
TGATTDemoResult BQB_CmdAuthKeypress      (PGATTDemo pGATTDemo, TGATTDemoParseResult *pParseResult);
TGATTDemoResult BQB_CmdAuthKeyboard      (PGATTDemo pGATTDemo, TGATTDemoParseResult *pParseResult);
TGATTDemoResult BQB_CmdAuthSetOOB        (PGATTDemo pGATTDemo, TGATTDemoParseResult *pParseResult);
TGATTDemoResult BQB_CmdAuthPin           (PGATTDemo pGATTDemo, TGATTDemoParseResult *pParseResult);

/*----------------------------------------------------------------------------
 * link
 * --------------------------------------------------------------------------*/
extern PGATTLink BQB_LinkAllocate(PGATTDemo pGATTDemo, uint8_t * pBD);
extern void BQB_LinkRelease(PGATTDemo pGATTDemo, PGATTLink pLink);
extern PGATTLink BQB_LinkFindByLocal_MDL_ID(PGATTDemo pGATTDemo, uint16_t local_MDL_ID);
extern PGATTLink BQB_LinkFindByBD(PGATTDemo pGATTDemo, uint8_t * pBD);
extern PGATTLink BQB_LinkFindByRole(PGATTDemo pGATTDemo, uint16_t role);

extern PGATTLEChannel BQB_LEChanAllocate(PGATTDemo pGATTDemo,PGATTLink pLink,uint16_t channel);
extern void BQB_LEChanClear(PGATTDemo pGATTDemo,PGATTLink pLink);
extern void BQB_LEChanRelease(PGATTDemo pGATTDemo,PGATTLEChannel pChan);
extern PGATTLEChannel BQB_LEChanFind(PGATTDemo pGATTDemo,PGATTLink pLink, uint16_t channel); 

/* print */
extern int sprintf(char *msg, const char *fmt, ...);
extern void  BQB_AttribDisplay(PGATTDemo pGATTDemo, uint16_t idx, uint16_t wHandle, int iSize, uint8_t * pValue);
//void  gattdServiceInitData( PGATTDemo pGATTDemo );

/****************************************************************************/
/* Macros                                                                   */
/****************************************************************************/
#define STATIC                     static
#define BQB_CmdPrint(a, b...)             BQB_Print(b)
#define BQB_UserIFSendString(a, b, c)     BQB_Print("%s", b)
#define BQB_UserIFSendChar(a, b)          if (b != 0) BQB_Print("%c", b)

#define ASCIIHexToBin              BQB_ASCIIHexToBin
bool BQB_ASCIIHexToBin(uint8_t *pHex, uint8_t *pBin, uint16_t Length);
extern int BQB_Print(IN  char *fmt, ...);
/****************************************************************************/
/* Prototypes                                                               */
/****************************************************************************/

/* export functions */
extern void   BQB_CmdInit(PGATTDemo pGATTDemo);
extern void BQB_BlueAPICallback(PBlueAPI_UsMessage pMsg);
extern void BQB_Init(void);
extern uint16_t BQB_GetUuidByHandle(PGATTDemo pGATTDemo, uint16_t idx, uint16_t wHandle);
extern uint16_t  BQB_AttribGetWriteData(PGATTDemo pGATTDemo, TGATTDemoParseResult *pParseResult,
                                        uint16_t * pwLength, uint8_t **ppValue);
extern void BQB_HandleUUIDSave(PGATTDemo pGATTDemo, uint16_t idx, uint16_t wHandle, uint16_t wUUID16);
extern void BQB_ServiceInitData(PGATTDemo pGATTDemo);
extern void BQB_ServiceUpdateCallback(PGATTDemo pGATTDemo, uint16_t wCause, uint16_t wAttribIndex);
extern uint16_t BQB_AttribPut(PGATTDemo pGATTDemo, PGATTDService pService, uint16_t iAttribIndex,
                              uint16_t wLength, uint8_t * pValue, TBQBGATTDWriteIndPostProc * pWriteIndPostProc);
extern uint16_t BQB_AttribGet(PGATTDemo pGATTDemo, PGATTDService pService, uint16_t iAttribIndex,
                              int iOffset, uint16_t* pwLength, uint8_t** ppValue);

extern TGATTDemoResult BQB_SendNotInd(PGATTDemo pGATTDemo, TGATTDemoParseResult *pParseResult);
TGATTDemoResult BQB_SelectDB(PGATTDemo pGATTDemo, TGATTDemoParseResult *pParseResult);
TGATTDemoResult BQB_SelectAdvDataIndex(PGATTDemo pGATTDemo, TGATTDemoParseResult *pParseResult);
TGATTDemoResult BQB_SetDiscoveryMode(PGATTDemo pGATTDemo, TGATTDemoParseResult *pParseResult);

#ifdef BQB_PREPARE_WRITE_SUPPPRT
TGATTDemoResult BQB_AttribPrepareWrite(PGATTDemo pGATTDemo, TGATTDemoParseResult *pParseResult);
TGATTDemoResult BQB_SendPrepareWrite(PGATTDemo pGATTDemo, uint16_t local_MDL_ID, uint16_t handle, 
                                                    uint16_t writeOffset, uint16_t length);
TGATTDemoResult BQB_SendExecuteWrite(PGATTDemo pGATTDemo,uint16_t local_MDL_ID, uint8_t flags);
TGATTDemoResult BQB_AttribExecuteWrite(PGATTDemo pGATTDemo, TGATTDemoParseResult *pParseResult);
TGATTDemoResult BQB_AttribReadMulti(PGATTDemo pGATTDemo, TGATTDemoParseResult *pParseResult);
TGATTDemoResult BQB_AutoChangeAuthMode(PGATTDemo pGATTDemo, TGATTDemoParseResult *pParseResult);
TGATTDemoResult BQB_ChangeMtuSize(PGATTDemo pGATTDemo, TGATTDemoParseResult *pParseResult);

#endif

#endif  /* __BQB_DEMO_H */

