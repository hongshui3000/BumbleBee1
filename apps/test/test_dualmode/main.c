/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
**********************************************************************************************************
* @file     main.c
* @brief    This is the entry of user code which the main function resides in.
* @details
* @author   kyle_xu
* @date     2015-11-30
* @version  v0.2
*********************************************************************************************************
*/
#include "FreeRTOS.h"
#include "task.h"

#include "legacy.h"
#include "peripheral.h"
#include "gapbond_dual.h"
#include "cod.h"
#include "profile_common.h"
#include "profile_api.h"

#include "application.h"
#include "test_dual_application.h"

#include <stdio.h>

/*
********************************************************
* parameter for btstack
*
*
*********************************************************
*/
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

#define DEFAULT_PAGESCAN_WINDOW             0x12
#define DEFAULT_PAGESCAN_INTERVAL           0x800
#define DEFAULT_PAGE_TIMEOUT                0x2000
#define DEFAULT_SUPVISIONTIMEOUT            0x7D00
#define DEFAULT_INQUIRYSCAN_WINDOW          0x12
#define DEFAULT_INQUIRYSCAN_INTERVAL        0x1000

#define DEFAULT_ADVERTISING_INTERVAL_MIN    0x20 /* 20ms */
#define DEFAULT_ADVERTISING_INTERVAL_MAX    0x30 /* 30ms */

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8_t ScanRspData[] =
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
static uint8_t AdvertData[] =
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
void GAP_Init(void)
{
    uint8_t gatt_name[GAP_DEVICE_NAME_LEN] = "Bee_perip";
    uint16_t appearance = GAP_GATT_APPEARANCE_UNKNOWN;
    uint8_t  adv_enable_default = TRUE;

    uint8_t  adv_event_type = GAP_ADTYPE_ADV_IND;
    uint8_t  adv_direct_type = PEER_ADDRTYPE_PUBLIC;
    uint8_t  adv_direct_addr[B_ADDR_LEN] = {0};
    uint8_t  adv_channel_map = GAP_ADVCHAN_ALL;
    uint8_t  adv_filter_policy = GAP_FILTER_POLICY_ALL;
    uint16_t adv_interval_min = DEFAULT_ADVERTISING_INTERVAL_MIN;
    uint16_t adv_interval_max = DEFAULT_ADVERTISING_INTERVAL_MIN;

    uint8_t  legacy_name[GAP_DEVICE_NAME_LEN] = "legacy";
    uint32_t class_of_device = SERVICE_CLASS_AUDIO |
                               MAJOR_DEVICE_CLASS_AUDIO |
                               MINOR_DEVICE_CLASS_HANDSFREE;

    uint8_t device_role = GAP_DEVICEROLE_DONTCARE;
    uint8_t link_policy = GAP_LINKPOLICY_ENABLE_ROLESWITCH | GAP_LINKPOLICY_ENABLE_SNIFFMODE;
    uint16_t supervision_timeout = DEFAULT_SUPVISIONTIMEOUT;

    uint8_t radio_mode = GAP_RADIOMODE_VISIABLE_CONNECTABLE;
    bool limited_discoverable = false;

    uint8_t pagescan_type      = GAP_PAGESCAN_TYPE_STANDARD;
    uint16_t pagescan_interval = DEFAULT_PAGESCAN_INTERVAL;
    uint16_t pagescan_window   = DEFAULT_PAGESCAN_WINDOW;
    uint16_t page_timeout      = DEFAULT_PAGE_TIMEOUT;

    uint8_t inquiryscan_type      = GAP_INQUIRYSCAN_TYPE_STANDARD;
    uint16_t inquiryscan_window   = DEFAULT_INQUIRYSCAN_WINDOW;
    uint16_t inquiryscan_interval = DEFAULT_INQUIRYSCAN_INTERVAL;
    uint8_t inquiry_mode = GAP_INQUIRYMODE_EXTENDEDRESULT;

    uint8_t pair_mode = GAPBOND_PAIRING_MODE_PAIRABLE;
    uint8_t mitm = GAPBOND_AUTH_YES_MITM_YES_BOND;
    uint8_t io_cap = GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT;
    uint8_t oob_enable = FALSE;
    uint32_t passkey = 0;
    uint8_t bt_mode = GAPBOND_BTMODE_21ENABLED;
    uint8_t fixed_passkey = TRUE;

    legacy_SetGapParameter(GAPPRRA_DEVICE_NAME, sizeof(legacy_name), legacy_name);
    legacy_SetGapParameter(GAPPARA_CLASS_OF_DEVICE, sizeof(uint32_t), &class_of_device);

    legacy_SetGapParameter(GAPPARA_DEVICE_ROLE, sizeof(uint8_t), &device_role);
    legacy_SetGapParameter(GAPPARA_LINK_POLICY, sizeof(uint8_t), &link_policy);
    legacy_SetGapParameter(GAPPARA_SUPERVISIONTIMEOUT, sizeof(uint16_t), &supervision_timeout);

    legacy_SetGapParameter(GAPPARA_RADIOMODE, sizeof(uint8_t), &radio_mode);
    legacy_SetGapParameter(GAPPARA_LIMITEDDISCOVERABLE, sizeof(bool), &limited_discoverable);

    legacy_SetGapParameter(GAPPARA_PAGESCAN_TYPE, sizeof(uint8_t), &pagescan_type);
    legacy_SetGapParameter(GAPPARA_PAGESCAN_INTERVAL, sizeof(uint16_t), &pagescan_interval);
    legacy_SetGapParameter(GAPPARA_PAGESCAN_WINDOW, sizeof(uint16_t), &pagescan_window);
    legacy_SetGapParameter(GAPPARA_PAGETIMEOUT, sizeof(uint16_t), &page_timeout);

    legacy_SetGapParameter(GAPPARA_INQUIRYSCAN_TYPE, sizeof(uint8_t), &inquiryscan_type);
    legacy_SetGapParameter(GAPPARA_INQUIRYSCAN_INTERVAL, sizeof(uint16_t), &inquiryscan_interval);
    legacy_SetGapParameter(GAPPARA_INQUIRYSCAN_WINDOW, sizeof(uint16_t), &inquiryscan_window);
    legacy_SetGapParameter(GAPPARA_INQUIRYMODE, sizeof(uint8_t), &inquiry_mode);

    peripheralSetGapParameter(GAPPRRA_DEVICE_NAME, sizeof(gatt_name), gatt_name);
    peripheralSetGapParameter(GAPPRRA_APPEARANCE, sizeof(appearance), &appearance);
    peripheralSetGapParameter(GAPPRRA_ADV_ENABLE_DEFAULT, sizeof(adv_enable_default), &adv_enable_default );

    peripheralSetGapParameter(GAPPRRA_ADV_EVENT_TYPE, sizeof(adv_event_type), &adv_event_type);
    peripheralSetGapParameter(GAPPRRA_ADV_DIRECT_ADDR_TYPE, sizeof(adv_direct_type), &adv_direct_type);
    peripheralSetGapParameter(GAPPRRA_ADV_DIRECT_ADDR, sizeof(adv_direct_addr), adv_direct_addr);
    peripheralSetGapParameter(GAPPRRA_ADV_CHANNEL_MAP, sizeof(adv_channel_map), &adv_channel_map);
    peripheralSetGapParameter(GAPPRRA_ADV_FILTER_POLICY, sizeof(adv_filter_policy), &adv_filter_policy);
    peripheralSetGapParameter(GAPPRRA_ADV_INTERVAL_MIN, sizeof(adv_interval_min), &adv_interval_min);
    peripheralSetGapParameter(GAPPRRA_ADV_INTERVAL_MAX, sizeof(adv_interval_max), &adv_interval_max);
    peripheralSetGapParameter(GAPPRRA_ADVERT_DATA, sizeof(AdvertData), AdvertData);
    peripheralSetGapParameter(GAPPRRA_SCAN_RSP_DATA, sizeof(ScanRspData), ScanRspData);

    GAPBonddual_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pair_mode);
    GAPBonddual_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
    GAPBonddual_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &io_cap);
    GAPBonddual_SetParameter(GAPBOND_OOB_ENABLED, sizeof(uint8_t), &oob_enable);
    GAPBonddual_SetParameter(GAPBOND_PASSKEY, sizeof(uint32_t), &passkey);
    GAPBonddual_SetParameter(GAPBOND_BTMODE, sizeof(uint8_t), &bt_mode);
    GAPBonddual_SetParameter(GAPBOND_FIXED_PASSKEY_ENABLE, sizeof(uint8_t), &fixed_passkey);
}

/******************************************************************
 * @fn          Initial profile
 * @brief      Add simple profile service and register callbacks 
 *
 * @return     void
 */
void Profile_Init(void)
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

    TestProfileInit();
    legacy_RegisterCB(AppProfileCallback);
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
int Testdual_main(void)
{
    Board_Init();
    Driver_Init();
    legacy_Init();
    BtStack_Init_Peripheral();
    GAP_Init();
    Profile_Init();
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

