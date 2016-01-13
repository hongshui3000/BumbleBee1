#ifndef  _BTLTP_H_
#define  _BTLTP_H_

#include <string.h>
#include <blueapi_types.h>
#include <ltplib.h>
#include "aci_if.h"
#include "gatt.h"
//#include <flags.h>
#include <common_defs.h>

#define LTP_VERSION                           0x10          /* LTP V1.0 */
#define ACI_EN                                0
#define USE_IN_BUMBLE                         1
#define BREDR_SUPPORT                         1

/* buffer size */
#define RX_BUFFER_SIZE                        0x400
#define RX_DISABLE_COUNT                      0x200
#define TX_BUFFER_SIZE                        0x200
#define RX_HANDLE_BUFFER_SIZE                 0x400

#define BTLTP_DEFAULT_COPMSK                  0x80          /* enable CRC  */

#define BTLTP_ACTION_POOL_SIZE                (2)  /* enough */
#define BT_GATT_SERVER_MAX_SERVICES_COUNT     8
#define BLUE_API_MDL_COUNT 4

#define LTP_ACT_INFO_FLAG_ACT_INFO            0x01
#define LTP_ACT_INFO_FLAG_ALL                 (LTP_ACT_INFO_FLAG_ACT_INFO)

#if BREDR_SUPPORT
#define BT_MAX_MTU_SIZE                       	335		
#define BTLTP_US_BUFFER_SIZE                  (BT_MAX_MTU_SIZE + 32)
#define BTLTP_MAX_MSG_SIZE                    361		
#else
#define BT_MAX_MTU_SIZE                       	244		
#define BTLTP_US_BUFFER_SIZE                  (BT_MAX_MTU_SIZE + 32)
#define BTLTP_MAX_MSG_SIZE                    270		
#endif

#define BTLTP_QUEUE_ELEMENT_COUNT             10

/******************************** the define of events ltp used **********************************/

#define LTP_EVENT_UART_RX               0x01      /* data available */
#define LTP_EVENT_UART_TX               0x02      /* transmit request */
#define LTP_EVENT_UART_TX_COMPLETED     0x03      /* transmit completed */
#define LTP_EVENT_BLUEAPI_MESSAGE       0x04      /* BlueAPI message */

/****************************************************************************/
/* state                                                                    */
/****************************************************************************/
typedef enum _TBTLtpState
{
    btltpStateInit,
    btltpStateRegistering,
    btltpStateIdle,
    btltpStateReleasing,
    btltpStateCount
} TBTLtpState;

/****************************************************************************/
/* Tx Buffer callback handling                                              */
/****************************************************************************/

typedef enum
{
    btltpActionNotUsed,
    btltpActionExit,
    btltpActionReset,
    btltpActionSendDataConf,
    btltpActionSendDIDDeviceConf,
    btltpActionSendSPPEndpointConf,
    btltpActionReleaseBuffer
} TBTLtpActionCommand;

typedef struct
{
    PVOID pBuffer;
    uint32_t  serviceHandle;
} TBTLtpServiceAction;

typedef union
{
    uint8_t *              pReleaseBuffer;
    uint16_t                MDL_ID;
    TBTLtpServiceAction serviceAction;
} TBTLtpActionData;

typedef struct
{
    TBTLtpActionCommand Action;
    TBTLtpActionData    p;
} TBTLtpAction;
typedef TBTLtpAction * PBTLtpAction;

#define LTP_MDL_UNUSED           0x00
#define LTP_MDL_ALLOCATED        0x01  /* MDL and MDEP valid/set */
#define LTP_MDL_CONNECTED        0x02  /* ConnectMDLInfo .. DisconnectMDLInd */
#define LTP_MDL_DATARSP_ERROR    0x04  /* got DataRsp with cause != success */
#define LTP_MDL_GATT             0x08  /* GATT context, reply with ConnectGATTMDLRsp */

typedef struct
{
    uint16_t             dsDataOffset;
    uint16_t             dsPoolID;
    uint8_t             flags;                  /* LTP_MDL_* */
    uint8_t             local_MDL_ID;           /* local MDL ID */
    uint8_t             local_MDEP_ID;          /* local MDEP ID */
    uint8_t             maxUsCredits;           /* max us credits (from ConnectMDLInfo) */
    uint8_t             collectedUsCredits;     /* credits need to be sent US */
    uint8_t             pendingDataConfs;       /* DataConf need to be sent DS */
    TBlueAPI_FrameType   returnCreditPacketType; /* packet type that shall be used to return Credits */
} TBTLtpMDLContext;
typedef TBTLtpMDLContext * PBTLtpMDLContext;

/****************************************************************************/
/* Instance data                                                            */
/****************************************************************************/

typedef struct
{
    void *  serviceHandle;
    void * pService;
    BOOL isUsed;
    uint8_t self_idx;
    uint16_t nbrOfAttrib;
    uint16_t database_length;
    uint16_t receive_length;
    uint32_t   host_service;
} TGattServiceTable;
typedef TGattServiceTable * PGattServiceTable;

typedef struct _TBTLtp
{
    uint8_t *              p_send_buffer;    /* for saving the address of tx buffer */
    LTP_QUEUE_T         FreeElementQueue;

    TBTLtpState         State;

    TBTLtpMDLContext    MDLContextPool[BLUE_API_MDL_COUNT];

    /* Buffer callback Action Handling                                       */
    TBTLtpAction        ActionPool[BTLTP_ACTION_POOL_SIZE];
    PBTLtpAction        pBufferAction;
    /* LTP re-assemble                                                       */
    uint16_t                LTP_US_OfflinePoolID;
    uint8_t *              pMsgBuffer;
    TLTPLib             LTPLib;
    TLTPElement         ElementPool[BTLTP_QUEUE_ELEMENT_COUNT];
    /* COM interface                                                         */
    uint8_t                ActInfoFlags;
    uint8_t                ownBDAddress[6];
    uint8_t                ActInfoCause;
    uint8_t                ActInfoVersionString[BLUE_API_VERSION_LENGTH];

    uint8_t                service_register_idx;
    TGattServiceTable   gattServiceTable[BT_GATT_SERVER_MAX_SERVICES_COUNT]; //use for ACI
    void *              gattServiceHandle[BT_GATT_SERVER_MAX_SERVICES_COUNT];//use for LTP

    P_ACI_TCB           p_aci_tcb;
} TBTLtp;
typedef TBTLtp * PBTLtp;

/* for ltp, will change later */

extern const TAttribAppl GattdFindMeProfile[];
extern const int gattSvcFindMeNbrOfAttrib;
extern const int GattdFindMeProfileSize;
extern PBTLtp  P_BtLtp;

/****************************************************************************/
/* Prototypes                                                               */
/****************************************************************************/

/* btltpcom.c */
void BTLTPHandleBLUE_API_MSG(PBTLtp pBTLtp, uint8_t * pBuffer, uint16_t offset);

/* btltpltp.c */
void BTLTPHandleResetReq                    (PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara);
void BTLTPHandleDisconnectMDLReq            (PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara);
void BTLTPHandleDisconnectMDLConf           (PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara);
void BTLTPHandleCreateMDLConf               (PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara);
void BTLTPHandleExitReq                     (PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint8_t * pPara);
void BTLTPHandlePasskeyRequestCnf           (PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara);
void BTLTPHandleOOBRequestCnf               (PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara);
void BTLTPHandleAuthResultExtCnf            (PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara);
void BTLTPHandleAuthResultRequestExtCnf     (PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara);
void BTLTPHandlePairableModeSetReq          (PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara);
void BTLTPHandlePasskeyReqReplyReq          (PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara);
void BTLTPHandleGATTServiceRegisterReq      (PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara);
void BTLTPHandleGATTAttributeUpdateReq      (PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara);
void BTLTPHandleGATTAttributeUpdateStatusCnf(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara);
void BTLTPHandleGATTAttributeReadCnf        (PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara);
void BTLTPHandleGATTAttributeWriteCnf       (PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara);
void BTLTPHandleGATTServerStoreCnf          (PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara);
void BTLTPHandleConnectGATTMDLReq           (PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara);
void BTLTPHandleGATTDiscoveryReq            (PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara);
void BTLTPHandleGATTDiscoveryCnf            (PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara);
void BTLTPHandleGATTAttributeReadReq        (PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara);
void BTLTPHandleGATTAttributeWriteReq       (PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara);
void BTLTPHandleGATTAttributeCnf            (PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara);
void BTLTPHandleGATTSecurityReq             (PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara);
void BTLTPHandleLEAdvertiseReq              (PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara);
void BTLTPHandleLEAdvertiseParameterSetReq  (PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara);
void BTLTPHandleLEAdvertiseDataSetReq       (PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara);
void BTLTPHandleLEScanReq                   (PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara);
void BTLTPHandleLEModifyWhitelistReq        (PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara);
void BTLTPHandleLEConnectionUpdateReq       (PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara);
void BTLTPHandleLEConnectionUpdateCnf       (PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara);

void BTLTPHandleSetRandomAddressReq         (PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara);
void BTLTPHandleDeviceNameSetReq               (PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara);
void BTLTPHandleDeviceConfigSecuritySetReq  (PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara);
void BTLTPHandleDeviceConfigAppearanceSetReq(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara);
void BTLTPHandleDeviceConfigStoreSetReq     (PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara);
void BTLTPHandleDeviceConfigPerPrefConnParamSetReq(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara);
void BTLTPHandleSetLETxPowerReq             (PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara);
void BTLTPHandleSetDataLengthReq            (PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara);
void BTLTPHandleDownloadServiceDatabaseReq  (PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara);
void BTLTPHandleClearServiceDatabaseReq     (PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara);
void BTLTPHandleSetTraceLevelReq            (PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara);
void BTLTPHandleGATTAttributePrepareWriteReq(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara);
void BTLTPHandleGATTAttributeExecuteWriteReq(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara);
void BTLTPHandleGATTAttributePrepareWriteCnf(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara);
void BTLTPHandleGATTAttributeExecuteWriteCnf(PBTLtp pBTLtp, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara);

/* btltputil.c */
PBTLtpMDLContext     BTLTPAllocateMDLContext(PBTLtp pBTLtp, uint8_t local_MDL_ID, uint8_t local_MDEP_ID, TBlueAPI_LinkConfigType linkConfigType);
PBTLtpMDLContext     BTLTPFindMDLContext        (PBTLtp pBTLtp, uint8_t local_MDL_ID);
PBTLtpMDLContext     BTLTPFindMDLContextByMDEPID(PBTLtp pBTLtp, uint8_t local_MDEP_ID);
PBTLtpMDLContext     BTLTPFindAMDLConnected(PBTLtp pBTLtp);
uint8_t                 BTLTPCountMDLConnected     (PBTLtp pBTLtp);
uint8_t                 BTLTPConvertCOMtoLTPcause  (TBlueAPI_Cause cause);
TBlueAPI_Cause       BTLTPConvertLTPtoCOMcause  (uint8_t cause);
PBTLtpAction         BTLTPAllocateAction        (PBTLtp pBTLtp);
void                 BTLTPCheckForActInfo       (PBTLtp pBTLtp);

/* btltp_FreeRTOS.c  */
extern void LtpBufferRelease(void *pBuffer);
extern void LtpWrite(uint8_t *p_buf, uint32_t buf_len);
extern void ltpBlueAPICallback(PBlueAPI_UsMessage pMsg);
extern BOOL LtpDlpsEnterCheck(void);

#endif
