 


#if !defined (__GATTDEMO_H)
#define  __GATTDEMO_H


/*
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
*/
#include "rtl_types.h"


#include <blueapi_types.h>


#include <bterrcod.h>


#include "gattdcmd.h"
#include "blueapi_types.h"


#include <gatt.h>
#include "gulcose_uuid.h"

#include "test_transport_uart.h"



#define GATTSRV_GLS 1
#define GATTDEMO_PREPARE_WRITE_SUPPPRT 1

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



/*------ service specifics: -------*/



#include "gattdglc.h"              /* Glucose */



#include "gattsvc_dis.h"           /* Device Information (common to all profiles ..) */




/*--- services defined in build process (must be used mutually exclusive!): ---*/

/*--- fatal error handler ---*/
#define GATTDEMO_FATAL_ERROR()  DebuggerBreak();

/*--- UART device ---*/
#define GATTDEMO_USART_DEVICE_NBR     0

/* max. number of <handle,UUID> pairs the client demo app. can store */
#define  GATTDEMO_MAX_HANDLE_UUID   16


#define GATTDEMO_BD_BUFFER_SIZE   256  /* size of buffer for BD inquiry results */

/*--- attribute handle to (16 bit) UUID mapping ---*/

typedef struct _GATTDHandleUUID
{
  uint16_t  wHandle;
  uint16_t  wUUID;
} TGATTDHandleUUID, * PGATTDHandleUUID;


/* BD address */
typedef uint8_t TGATTDBdAddr[BLUE_API_BD_SIZE];
typedef TGATTDBdAddr  * PGATTDBdAddr;

extern int sprintf(char *msg, const char *fmt, ...);
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
  TBlueAPI_GATTServiceID  ServiceID;
  void *                  pServiceHandle;  /* service handle provided by GATT server */
} TGATTDService, * PGATTDService;

#if defined(GATTDEMO_MAX_SERVICES)
  /* defined in service/profile specific header file */
#else
#define GATTTEST_MAX_SERVICES        1     /* all services in one array .. */
#endif

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
	bool			 isUsed;
	bool			 isDataChanConnected;
	uint16_t		 channel;
	uint16_t         maxDsCredit;
} TGATTLEChannel, *PGATTLEChannel;

#define GATTDEMO_MAX_LINKS  4
#define GATTDEMO_CON_ROLE_UNDEFINED 0
#define GATTDEMO_CON_ROLE_MASTER 1
#define GATTDEMO_CON_ROLE_SLAVE 2
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
  uint16_t		   wMTUSize;		  /* ATT layer MTU size */

  uint16_t         wEndingHandle;   /* current discovery ending handle */
  int 			 iHandleCnt;
  TGATTDHandleUUID HandleUUID[GATTDEMO_MAX_HANDLE_UUID];
  TGATTLEChannel   LEChannel[GATTDEMO_MAX_LE_CHANNELS];
} TGATTLink, *PGATTLink;


  /* external data store definitions */
#define GATTDSTORE_ENTRY_COUNT    4
#define GATTDSTORE_NVRAM_ADDRESS  0

#if (GATTDSTORE_ENTRY_COUNT)
typedef struct _GATTDStoreEntrySEC
{
  TBlueAPI_LinkKeyType    keyType;
  uint8_t                 dataLen;
  uint8_t                 data[28];
} TGATTDStoreEntrySEC;

typedef struct _GATTDStoreEntryGATT
{
  uint8_t                 dataLen;
  uint8_t                 data[32];
} TGATTDStoreEntryGATT;

#define GATTDSTORE_FREE_ENTRY    0
#define GATTDSTORE_SEC_ENTRY     1
#define GATTDSTORE_CCC_ENTRY     2

typedef struct _GATTDStoreEntry
{
  uint8_t                 used;
  uint8_t                 bd[6];
  TBlueAPI_RemoteBDType   bdType;
  union
  {
    TGATTDStoreEntrySEC   sec;
    TGATTDStoreEntryGATT  gatt;
  } p;
} TGATTDStoreEntry, * PGATTDStoreEntry;

#endif /* GATTDEMO_EXT_STORE */



#define GATTDEMO_MAX_PREPARE_QUEUE  4
#define GATTDEMO_MAX_PREPARE_VALUE_LENGTH 18

#if (GATTDEMO_PREPARE_WRITE_SUPPPRT)

typedef struct _GATTPrepareWrite
{
  void *         serviceHandle;
  uint16_t       attribIndex;
  uint16_t       handle;
  uint16_t       writeOffset;        /* write offset in attribute */
  uint16_t       attribLength;
  uint8_t        data[GATTDEMO_MAX_PREPARE_VALUE_LENGTH];
} TGATTPrepareWrite, *PGATTPrepareWrite;
#endif


/* some services perform specific init. when connection is established: */
typedef void (* TGATTDConnectedInd)(void * PGATTTest);



typedef struct _TTestParseResult TTestParseResult;
typedef enum _TTestResult TTestResult;

/*--------------------------------------------------------------------*/
typedef struct _GATTTest  /* main structure */
{
  /* cmd interface data */



  bool             initLEServer;
  bool             advertiseOnDisc;    /* true: restart advertising when disconnected */


  /*-- BlueAPI/Blueface API data --*/



  int              iServiceCount;
  TGATTDService    Service[GATTTEST_MAX_SERVICES];
  uint16_t         wUpdReqHandle;   /* attribute update request handle */
                                    /* (NOT attribute handle !!!)      */



  uint8_t          bCCCBits[sizeof(uint16_t)];  /* current CCCBits written .. */

  /* connection related data */
  TGATTLink        linkTable[GATTDEMO_MAX_LINKS];

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

#if (GATTDEMO_PREPARE_WRITE_SUPPPRT)
  //fixme: now just support one link for prepareQueue
  TGATTPrepareWrite	 prepareQueue[GATTDEMO_MAX_PREPARE_QUEUE];
  int 				 queueIdx;
  uint16_t			 queue_local_MDL_ID;				 
#endif

  /* attribute value update sequence control */
  int              iUpdateServiceIndex;
  int              iUpdateAttribIndex;
  int              iUpdateCnt;
  int              iUpdateSent;
  int              iUpdateConfirmed;

    int              iUpdateDataLength;

    int              iReadDataLength;
    
  /* service specifics */

  TGLCRACP         RACP;    /* Record Access Control Point related data */

#if (GATTDSTORE_ENTRY_COUNT)
  TGATTDStoreEntry  extStore[GATTDSTORE_ENTRY_COUNT];
#endif


  /* miscellaneous */
  bool             ShortOutput;     /* true: reduce output (buffer limitations ..) */

 bool bEnterDlps;   


} TGATTTest, *PGATTTest;



void gattTestInit( void );

/*------ attribute write indication post processing (needed for some -----*/
/*------ services that have a Record Access Control Point (RACP)     -----*/

typedef void (* TGATTDWriteIndPostProc)
                           (PGATTTest PGATTTest, uint16_t wLength, uint8_t * pValue);


/*---------------------------------------------------------------------------
 * prototypes of internal routines
 *--------------------------------------------------------------------------*/

 void   test_upperstack_ApplicStart( PGATTTest PGATTTest );




  /*-- gattdutl.c: GATT utilities --*/
const char * test_upperstack_ErrorCode2Str( uint16_t wCause );
void         test_upperstack_HexDump( PGATTTest PGATTTest, char *pTxt, int iSize, uint8_t * pValue );
char * test_upperstack_SFLOAT2StringAndWORD( uint16_t * pwSFLOATValue );
void   test_upperstack_PrintSFLOAT( PGATTTest PGATTTest, char *pName, uint16_t wValue );
void   test_upperstack_PrintTimestamp( PGATTTest PGATTTest, uint8_t * pTimestamp, char * pTxt );
char * test_upperstack_UUID16ToString( uint16_t wUUID16 );
uint16_t   test_upperstack_UUIDGet( PGATTTest PGATTTest,uint16_t idx, uint16_t wHandle );


PGATTDService gattServiceFind( PGATTTest PGATTTest, void * pServiceHandle );

#if (GATTDEMO_CCCD_COUNT)
void   test_upperstack_CCCDInitTable( PGATTTest pGattTest );
uint16_t   test_upperstack_CCCDCheckValue( PGATTTest PGATTTest, int iStart, int iEnd );
#endif


void test_upperstack_HandleBlueAPIMessage            (PGATTTest PGATTTest, PBlueAPI_UsMessage pMsg);


void test_upperstack_Handle_AuthResultInd            (PGATTTest PGATTTest, PBlueAPI_AuthResultInd pAuthResultInd);
void test_upperstack_Handle_AuthResultRequestInd     (PGATTTest PGATTTest, PBlueAPI_AuthResultRequestInd pAuthResultRequestInd);


void test_upperstack_Handle_PairableModeSetRsp       (PGATTTest PGATTTest, PBlueAPI_PairableModeSetRsp pPairableModeSetRsp);
void test_upperstack_Handle_UserPasskeyReqInd        (PGATTTest PGATTTest, PBlueAPI_UserPasskeyReqInd pUserPasskeyReqInd);
void test_upperstack_Handle_UserPasskeyNotificationInfo(PGATTTest PGATTTest, PBlueAPI_UserPasskeyNotificationInfo pUserPasskeyNotificationInfo);
void test_upperstack_Handle_UserPasskeyReqReplyRsp   (PGATTTest PGATTTest, PBlueAPI_UserPasskeyReqReplyRsp pUserPasskeyReqReplyRsp);

void test_upperstack_Handle_RemoteOOBDataReqInd      (PGATTTest PGATTTest, PBlueAPI_RemoteOOBDataReqInd pRemoteOOBDataReqInd);
#else
//void test_upperstack_HandleBlueFaceMessage( PGATTTest pGattTest, LPblueFaceMsg pMsg );
#endif


/*-- profile/service specific module: gattdtst.c, gattdbpm.c ... --*/

void  test_upperstack_HandleUUIDSave( PGATTTest PGATTTest,uint16_t idx, uint16_t wHandle, uint16_t wUUID16 );
void  test_upperstack_AttribDisplay( PGATTTest PGATTTest,uint16_t idx, uint16_t wHandle,
                                                   int iSize, uint8_t * pValue );
uint16_t  test_upperstack_AttribGetWriteData( PGATTTest PGATTTest, TTestParseResult *pParseResult,
                                              uint16_t * pwLength, uint8_t * *ppValue );

TBlueAPI_GATTServiceID  gattTestGetNextServiceID( int iServiceCount );
uint16_t  gattTestAttribGet( PGATTTest PGATTTest, PGATTDService pService, int iAttribIndex,
                                   int iOffset, uint16_t * pwLength, uint8_t * *ppValue );
uint16_t  gattTestAttribPut( PGATTTest PGATTTest, PGATTDService pService, int iAttribIndex,
                                   uint16_t wLength, uint8_t * pValue,
                                   TGATTDWriteIndPostProc * pWriteIndPostProc);
uint16_t  gattTestAttribPrepareWrite( PGATTTest PGATTTest, uint16_t local_MDL_ID,uint16_t *handle);

void  gattTestServiceUpdateCallback( PGATTTest PGATTTest,
                                               uint16_t wCause, uint16_t wAttribIndex );
void  gattTestServiceInitData( PGATTTest PGATTTest );




  /*--- prototypes of GLC specific routines ---*/
  /* gatdglcp.c */

TTestResult test_upperstack_GLC_RACPInit(PGATTTest PGATTTest,
                                          TTestParseResult *pParseResult);
TTestResult test_upperstack_GLC_RACPSetParam(PGATTTest PGATTTest,
                                          TTestParseResult *pParseResult);
void  test_upperstack_GLC_RACPProcComplete( PGATTTest PGATTTest, uint16_t wCause );
void  test_upperstack_GLC_RACPHandleReq( PGATTTest PGATTTest, uint16_t wLength, uint8_t * pValue );
bool  test_upperstack_GLC_RACPOperationAllowed( PGATTTest PGATTTest, uint8_t bOpCode );


TTestResult test_upperstack_GLC_RACPWrite(PGATTTest PGATTTest,
                                          TTestParseResult *pParseResult);
void  test_upperstack_GLC_RACPHandleResp( PGATTTest PGATTTest, PGLCControlPoint pValue );


 

void test_upperstack_CPTimerStart( PGATTTest PGATTTest, unsigned char ucIDMajor );
void test_upperstack_CPTimerStop( PGATTTest PGATTTest, unsigned char ucIDMajor );

PGATTLEChannel test_upperstack_LEChanAllocate(PGATTTest PGATTTest,PGATTLink pLink,uint16_t channel);
void test_upperstack_LEChanClear(PGATTTest PGATTTest,PGATTLink pLink);
void test_upperstack_LEChanRelease(PGATTTest PGATTTest,PGATTLEChannel pChan);
PGATTLEChannel test_upperstack_LEChanFind(PGATTTest PGATTTest,PGATTLink pLink, uint16_t channel); 

 
/****************************************************************************/
/* Macros                                                                   */
/****************************************************************************/

#define STATIC                     static

#define test_upperstack_CmdPrint(a, b...)             transport_uart_print(b)
#define test_upperstack_UserIFSendString(a, b, c)     transport_uart_print("%s", b)
#define test_upperstack_UserIFSendChar(a, b)          if (b != 0) transport_uart_print("%c", b)

#define ASCIIHexToBin              test_upperstack_ASCIIHexToBin
bool test_upperstack_ASCIIHexToBin(uint8_t *pHex, uint8_t *pBin, uint16_t Length);


/****************************************************************************/
/* Prototypes                                                               */
/****************************************************************************/

void test_upperstack_Init(void);
void test_upperstack_BlueAPICallback( PBlueAPI_UsMessage pMsg );





