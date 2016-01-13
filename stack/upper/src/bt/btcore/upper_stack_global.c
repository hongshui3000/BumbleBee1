#include <upper_stack_global.h>
#include <message.h>
#include <hci.h>
#include <gattdef.h>


//blueapi_util.c

//btsmutil.c
char gHexBuffer[32 + 1] = {0};

//cutlbt.c
char gStrBdAddressBuf[20] = {0};


//gattutil.c
char gGattDeviceName[GATT_DEVICE_NAME_LENGTH] = {0};
char gGattAppearance[2] = {0};
char gGattPerPrefConnParam[8] = {0};

//os.c
TOS os = {0};


#if 0

//os_mem.c
uint16_t gDataOnSize = 0;
uint16_t gDataOffSize = 0;
uint16_t gBufOffSize = 0;

//flags.h

//gatt.c GATT BQB




uint8_t *pDataRamPartialOnHeap = NULL;
uint8_t *DataRamPartialOnHeapBottom = NULL;
uint8_t *DataRamPartialOnHeapLastMem = NULL;

uint8_t* pDataRamPartialOffHeap = NULL;
uint8_t *DataRamPartialOffHeapBottom = NULL;
uint8_t *DataRamPartialOffHeapLastMem = NULL;

uint8_t* pBufRamPartialOffHeap = NULL;
uint8_t *BufRamPartialOffHeapBottom = NULL;
uint8_t *BufRamPartialOffHeapLastMem = NULL;

#endif
uint8_t gL2cDsFragmentationSupport = TRUE;

uint8_t *pUserDataRamPartialOnHeap = NULL;
uint8_t *UserDataRamPartialOnHeapBottom = NULL;
uint8_t *UserDataRamPartialOnHeapLastMem = NULL;

uint8_t* pUserDataRamPartialOffHeap = NULL;
uint8_t *UserDataRamPartialOffHeapBottom = NULL;
uint8_t *UserDataRamPartialOffHeapLastMem = NULL;
#if 0
uint8_t* pUserBufRamPartialOffHeap = NULL;
uint8_t *UserBufRamPartialOffHeapBottom = NULL;
uint8_t *UserBufRamPartialOffHeapLastMem = NULL;

#endif

uint8_t gGattUsWriteOffset = BT_GATT_US_WRITE_OFFSET_COUNT;

//uint16_t gBlueAPIEnableAclInfo = 0x01FF;    //(BLUE_API_ENABLE_ACL_INFO_ALL & ~BLUE_API_ENABLE_ACL_INFO_CONNECTION_ENCRYPTED)

uint8_t gAdvertisingChannelMap = LE_ADVERTISING_CHANNEL_ALL;

TSDP  *pSDP  = NULL;
TBtSM *pBtSM = NULL;
THCI  *pHCI  = NULL;
TL2C  *pL2c  = NULL;
TGATT *pGATT = NULL;
TBTA  *pBTA  = NULL;
TBlueAPI_Data *pBlueAPIData = NULL;

uint8_t sdpQueueID = 0;
uint8_t gattQueueID = 0;
uint8_t l2cQueueID = 0;
uint8_t hciQueueID = 0;
uint8_t btsmQueueID = 0;
uint8_t btaQueueID = 0;
uint8_t blueAPIQueueID = 0;

uint8_t UpstreamPoolID = 0xFF;
uint8_t DownstreamPoolID = 0xFF;
uint8_t BTSystemPoolID = 0xFF;

