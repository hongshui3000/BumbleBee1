
#ifndef _UPPER_STACK_GLOBAL_H_
#define _UPPER_STACK_GLOBAL_H_

#include <stdint.h>
#include <rtl_types.h>
#include <blueapi_def.h>

typedef unsigned char    UCHAR, uint8_t, * PUCHAR, * PBYTE;
typedef unsigned short  uint16_t;
typedef void   * PVOID;
typedef struct _MESSAGE_T MESSAGE_T;
typedef struct tagOS TOS;
typedef struct tagBtSM TBtSM;
typedef struct _THCI THCI;
typedef struct _TL2C TL2C;
typedef struct _TGATT TGATT;
typedef struct _TBTA TBTA;
typedef CONST TBTA * PCBTA;
typedef struct _TSDP TSDP;


typedef struct tagStackAclPool TStackAclPool, * PTStackAclPool;



extern char gHexBuffer[];

extern char gStrBdAddressBuf[];


extern char gGattDeviceName[];
extern char gGattAppearance[];
extern char gGattPerPrefConnParam[];

extern TOS os;


#if 0

extern uint16_t gDataOnSize;
extern uint16_t gDataOffSize;
extern uint16_t gBufOffSize;

#endif

extern uint8_t gL2cDsFragmentationSupport;

/**
    global configurations before stack start up.
*/

extern TBtSM *pBtSM;
extern THCI  *pHCI;
extern TL2C  *pL2c;
extern TGATT *pGATT;
extern TBTA  *pBTA;
extern TBlueAPI_Data *pBlueAPIData;
extern TSDP *pSDP;

extern uint8_t sdpQueueID;
extern uint8_t gattQueueID;
extern uint8_t l2cQueueID;
extern uint8_t hciQueueID;
extern uint8_t btsmQueueID;
extern uint8_t btaQueueID;
extern uint8_t blueAPIQueueID;

extern uint8_t UpstreamPoolID;
extern uint8_t DownstreamPoolID;
extern uint8_t BTSystemPoolID;

extern uint8_t gGattUsWriteOffset;

//stprint.c
extern uint8_t  *g_printHexPtr;
extern uint8_t    g_printHex[];


extern PVOID pOSIFLock;


#if 0

extern uint8_t *pDataRamPartialOnHeap;
extern uint8_t *DataRamPartialOnHeapBottom;
extern uint8_t *DataRamPartialOnHeapLastMem;

extern uint8_t* pDataRamPartialOffHeap;
extern uint8_t *DataRamPartialOffHeapBottom;
extern uint8_t *DataRamPartialOffHeapLastMem;

extern uint8_t* pBufRamPartialOffHeap;
extern uint8_t *BufRamPartialOffHeapBottom;
extern uint8_t *BufRamPartialOffHeapLastMem;
#endif
extern uint8_t *pUserDataRamPartialOnHeap;
extern uint8_t *UserDataRamPartialOnHeapBottom;
extern uint8_t *UserDataRamPartialOnHeapLastMem;

extern uint8_t* pUserDataRamPartialOffHeap;
extern uint8_t *UserDataRamPartialOffHeapBottom;
extern uint8_t *UserDataRamPartialOffHeapLastMem;
#if 0
extern uint8_t* pUserBufRamPartialOffHeap;
extern uint8_t *UserBufRamPartialOffHeapBottom;
extern uint8_t *UserBufRamPartialOffHeapLastMem;


#endif


extern uint8_t gGattUsWriteOffset;
extern uint8_t    gAdvertisingChannelMap;

#define BLUEAPI_ENABLE_ACL_INFO (0x01FF & ~0x0020)
#endif



