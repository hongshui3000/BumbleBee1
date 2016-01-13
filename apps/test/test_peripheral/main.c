/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
**********************************************************************************************************
* @file     main.c
* @brief    This is the entry of user code which the main function resides in.
* @details
* @author   Ethan
* @date     2015-07-02
* @version  v0.2
*********************************************************************************************************
*/
//#include "rtl876x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "application.h"
//#include "dlps_platform.h"
#include "peripheral.h"
#include "gap.h"
#include "gapbondmgr.h"
#include "profile_api.h"
#include "test_peripheral_application.h"
#include "simple_ble_service.h"
#include "trace.h"

#include <stdio.h>

/*
********************************************************
* parameter for btstack
*
*
*********************************************************
*/
// What is the advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL_MIN            0x20 /* 20ms */
#define DEFAULT_ADVERTISING_INTERVAL_MAX            0x30 /* 30ms */


/* Service IDs definition. */
#if SIMPLE_SERVICE_EN
uint8_t gSimpleProfileServiceId;
#endif
#if DIS_SERVICE_EN
uint8_t gDisServiceId;
#endif
#if BAS_SERVICE_EN
uint8_t gBasServiceId;
#endif
#if RSC_SERVICE_EN
uint8_t gRscServiceId;
#endif
#if CSC_SERVICE_EN
uint8_t gCscServiceId;
#endif


// GAP - SCAN RSP data (max size = 31 bytes)
static uint8_t scanRspData[] =
{
    0x03,           /* length     */
    0x03,           /* type="More 16-bit UUIDs available, service uuid 0xA00A" */
    0x0A,
    0xA0,
    0x03,           /* length     */
    0x19,           /* type="Appearance" */
    0x00, 0x00,     /* Unknown */
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8_t advertData[] =
{
    /* Core spec. Vol. 3, Part C, Chapter 18 */
    /* Flags */
    0x02,            /* length     */
    //XXXXMJMJ 0x01, 0x06,      /* type="flags", data="bit 1: LE General Discoverable Mode", BR/EDR not supp. */
    0x01, GAP_ADTYPE_FLAGS_LIMITED|GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,      /* type="flags", data="bit 1: LE General Discoverable Mode" */
    /* Service */
    0x03,           /* length     */
    0x03,           /* type="More 16-bit UUIDs available, service uuid 0xA00A" */
    0x0A,
    0xA0,
    0x03,           /* length     */
    0x19,           /* type="Appearance" */
    0x00, 0x00,     /* Unknown */
    0x0A,           /* length     */
    0x09,           /* type="Complete local name" */
//    0x42, 0x65, 0x65, 0x5F, 0x6D, 0x6F, 0x75, 0x73, 0x65  /* Bee_mouse */
    'B', 'e', 'e', '_', 'p', 'e', 'r', 'i', 'p' /* Bee_perip */
};


/******************************************************************
 * @fn          Initial gap parameters
 * @brief      Initialize peripheral and gap bond manager related parameters
 *
 * @return     void
 */
void BtStack_Init_Gap(void)
{
    //device name and device appearance
    uint8_t DeviceName[GAP_DEVICE_NAME_LEN] = "Bee_perip";
    uint16_t Appearance = GAP_GATT_APPEARANCE_UNKNOWN;

    //default start adv when bt stack initialized
    uint8_t  advEnableDefault = TRUE;

    //advertising parameters
    uint8_t  advEventType = GAP_ADTYPE_ADV_IND;
    uint8_t  advDirectType = PEER_ADDRTYPE_PUBLIC;
    uint8_t  advDirectAddr[B_ADDR_LEN] = {0};
    uint8_t  advChanMap = GAP_ADVCHAN_ALL;
    uint8_t  advFilterPolicy = GAP_FILTER_POLICY_ALL;
    uint16_t advIntMin = DEFAULT_ADVERTISING_INTERVAL_MIN;
    uint16_t advIntMax = DEFAULT_ADVERTISING_INTERVAL_MIN;

    //GAP Bond Manager parameters
    uint8_t pairMode = GAPBOND_PAIRING_MODE_PAIRABLE;
    uint8_t mitm = GAPBOND_AUTH_YES_MITM_YES_BOND;
    uint8_t ioCap = GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT;
    uint8_t oobEnable = FALSE;
    uint32_t passkey = 0; // passkey "000000"

    uint8_t bUseFixedPasskey = TRUE;

    //Set device name and device appearance
    peripheralSetGapParameter(GAPPRRA_DEVICE_NAME, GAP_DEVICE_NAME_LEN, DeviceName);
    peripheralSetGapParameter(GAPPRRA_APPEARANCE, sizeof(Appearance), &Appearance);

    peripheralSetGapParameter( GAPPRRA_ADV_ENABLE_DEFAULT, sizeof ( advEnableDefault ), &advEnableDefault );

    //Set advertising parameters
    peripheralSetGapParameter( GAPPRRA_ADV_EVENT_TYPE, sizeof ( advEventType ), &advEventType );
    peripheralSetGapParameter( GAPPRRA_ADV_DIRECT_ADDR_TYPE, sizeof ( advDirectType ), &advDirectType );
    peripheralSetGapParameter( GAPPRRA_ADV_DIRECT_ADDR, sizeof ( advDirectAddr ), advDirectAddr );
    peripheralSetGapParameter( GAPPRRA_ADV_CHANNEL_MAP, sizeof ( advChanMap ), &advChanMap );
    peripheralSetGapParameter( GAPPRRA_ADV_FILTER_POLICY, sizeof ( advFilterPolicy ), &advFilterPolicy );

    peripheralSetGapParameter(GAPPRRA_ADV_INTERVAL_MIN, sizeof(advIntMin), &advIntMin);
    peripheralSetGapParameter(GAPPRRA_ADV_INTERVAL_MAX, sizeof(advIntMax), &advIntMax);

    peripheralSetGapParameter( GAPPRRA_ADVERT_DATA, sizeof( advertData ), advertData );
    peripheralSetGapParameter( GAPPRRA_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );

    // Setup the GAP Bond Manager
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8_t ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8_t ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8_t ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_OOB_ENABLED, sizeof ( uint8_t ), &oobEnable );

    GAPBondMgr_SetParameter( GAPBOND_PASSKEY, sizeof ( uint32_t ), &passkey );
    GAPBondMgr_SetParameter(GAPBOND_FIXED_PASSKEY_ENABLE, sizeof ( uint8_t ), &bUseFixedPasskey);
}

/******************************************************************
 * @fn          Initial profile
 * @brief      Add simple profile service and register callbacks 
 *
 * @return     void
 */
void BtProfile_Init(void)
{
#if RSC_SERVICE_EN
    uint16_t instantSpeed = 10;
    uint8_t instantCadence = 5;
    uint16_t instantStrideLength = 3;
    uint32_t totalDistance = 100;
    uint8_t rsc_sens_location = RSC_SENSOR_LOC_TOP_OF_SHOE;
#endif
#if CSC_SERVICE_EN
    uint32_t wheelRevolution = 10;
    uint16_t wheelEventTime = 5;
    uint16_t crankRevolution = 8;
    uint16_t crankEventTime = 12;
    uint8_t csc_flag = CSC_INC_ALL_PRESENTS;
#endif

#if SIMPLE_SERVICE_EN
    gSimpleProfileServiceId = SimpBleService_AddService(AppProfileCallback);
#endif
#if DIS_SERVICE_EN
    gDisServiceId = DIS_AddService(AppProfileCallback);
#endif
#if BAS_SERVICE_EN
    gBasServiceId = BAS_AddService(AppProfileCallback);
#endif
#if RSC_SERVICE_EN
    gRscServiceId = RSC_AddService(AppProfileCallback);
#endif
#if CSC_SERVICE_EN
    gCscServiceId = CSC_AddService(AppProfileCallback);
#endif

#if RSC_SERVICE_EN
    /* Service Initiate -- Running Speed and Cadence service */
    RSC_SetParameter(RSC_PARAM_SPEED, 2, (uint8_t*)&instantSpeed);
    RSC_SetParameter(RSC_PARAM_CADENCE, 1, (uint8_t*)&instantCadence);
    RSC_SetParameter(RSC_PARAM_STRIDE_LENGTH,2,  (uint8_t*)&instantStrideLength);
    RSC_SetParameter(RSC_PARAM_TOTALDISTANCE, 4, (uint8_t*)&totalDistance);
    RSC_SetParameter(RSC_PARAM_SENSOR_LOC, 1, &rsc_sens_location);
        
#endif
#if CSC_SERVICE_EN
    /* Service Initiate -- Cycling Speed and Cadence service */
    CSC_SetParameter(CSC_PARAM_INC_FLAG, 1, &csc_flag);
    CSC_SetParameter(CSC_PARAM_WHEEL_REVOL, 4, (uint8_t*)&wheelRevolution);
    CSC_SetParameter(CSC_PARAM_WHEEL_EVT_TIME, 2, (uint8_t*)&wheelEventTime);
    CSC_SetParameter(CSC_PARAM_CRANK_REVOL, 2, (uint8_t*)&crankRevolution);
    CSC_SetParameter(CSC_PARAM_CRANK_EVT_TIME, 2, (uint8_t*)&crankEventTime);
#endif

    ProfileAPI_RegisterCB(AppProfileCallback);
}

/**
* @brief    Board_Init() contains the initialization of pinmux settings and pad settings.
*
*               All the pinmux settings and pad settings shall be initiated in this function.
*               But if legacy driver is used, the initialization of pinmux setting and pad setting
*               should be peformed with the IO initializing.
*
* @return  void
*/
void Board_Init(void)
{

}

/**
* @brief    Driver_Init() contains the initialization of peripherals.
*
*               Both new architecture driver and legacy driver initialization method can be used.
*
* @return  void
*/
void Driver_Init(void)
{
    //m24c64_Init();
    //stNvramInit(4096);
}


/**
* @brief    PwrMgr_Init() contains the setting about power mode.
*
* @return  void
*/
void PwrMgr_Init(void)
{
}


/**
* @brief  Task_Init() contains the initialization of all the tasks.
*
*           There are four tasks are initiated.
*           Lowerstack task and upperstack task are used by bluetooth stack.
*           Application task is task which user application code resides in.
*           Emergency task is reserved.
*
* @return  void
*/
void Task_Init(void)
{
    void lowerstack_task_init();
    void upperstack_task_init();
    void emergency_task_init();
    application_task_init();
}


/**
* @brief  main() is the entry of user code.
*
*
* @return  void
*/
int test_peripheral_main(void)
{
    Board_Init();
    Driver_Init();
    BtStack_Init_Peripheral();
    BtStack_Init_Gap();
    BtProfile_Init();
    PwrMgr_Init();
    Task_Init();
    //vTaskStartScheduler();

    return 0;
}

void hard_fault_handler_c(unsigned int * hardfault_args)  
{
    unsigned int stacked_r0;  
    unsigned int stacked_r1;  
    unsigned int stacked_r2;  
    unsigned int stacked_r3;  
    unsigned int stacked_r12;  
    unsigned int stacked_lr;  
    unsigned int stacked_pc;  
    unsigned int stacked_psr;  

    stacked_r0 = ((unsigned long) hardfault_args[0]);  
    stacked_r1 = ((unsigned long) hardfault_args[1]);  
    stacked_r2 = ((unsigned long) hardfault_args[2]);  
    stacked_r3 = ((unsigned long) hardfault_args[3]);  

    stacked_r12 = ((unsigned long) hardfault_args[4]);  
    stacked_lr = ((unsigned long) hardfault_args[5]);  
    stacked_pc = ((unsigned long) hardfault_args[6]);  
    stacked_psr = ((unsigned long) hardfault_args[7]);  

    printf ("[Hard fault handler]\n");  
    printf ("R0 = %x\n", stacked_r0);  
    printf ("R1 = %x\n", stacked_r1);  
    printf ("R2 = %x\n", stacked_r2);  
    printf ("R3 = %x\n", stacked_r3);  
    printf ("R12 = %x\n", stacked_r12);  
    printf ("LR = %x\n", stacked_lr);  
    printf ("PC = %x\n", stacked_pc);  
    printf ("PSR = %x\n", stacked_psr);  

    while(1)
    {
        ;
    }
}  

