/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
**********************************************************************************************************
* @file     main.c
* @brief    This is the entry of user code which the main function resides in.
* @details
* @author   kyle_xu
* @date     2015-07-02
* @version  v0.2
*********************************************************************************************************
*/
#include "FreeRTOS.h"
#include "task.h"

#include "legacy.h"
#include "gapbond_legacy.h"
#include "cod.h"
#include "profile_common.h"

#include "application.h"
#include "test_application.h"
#include "avrcp_app_demo.h"
#include "a2dp_demo.h"
#include "dip.h"

#include <stdio.h>
/*
********************************************************
* parameter for btstack
*
*
*********************************************************
*/

#define DEFAULT_PAGESCAN_WINDOW         0x12
#define DEFAULT_PAGESCAN_INTERVAL       0x800
#define DEFAULT_PAGE_TIMEOUT            0x2000
#define DEFAULT_SUPVISIONTIMEOUT        0x7D00
#define DEFAULT_INQUIRYSCAN_WINDOW      0x12
#define DEFAULT_INQUIRYSCAN_INTERVAL    0x1000


/******************************************************************
 * @fn          Initial gap parameters
 * @brief      Initialize peripheral and gap bond manager related parameters
 *
 * @return     void
 */
void GAP_Init(void)
{
    uint8_t  device_name[GAP_DEVICE_NAME_LEN] = "test1_avrcp";
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
    uint8_t mitm = GAPBOND_AUTH_NO_MITM_GENERAL_BOND;
    uint8_t io_cap = GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT;
    uint8_t oob_enable = FALSE;
    uint32_t passkey = 0;
    uint8_t bt_mode = GAPBOND_BTMODE_21ENABLED;

    legacy_SetGapParameter(GAPPRRA_DEVICE_NAME, sizeof(device_name), device_name);
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

    GAPBondlegacy_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pair_mode);
    GAPBondlegacy_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
    GAPBondlegacy_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &io_cap);
    GAPBondlegacy_SetParameter(GAPBOND_OOB_ENABLED, sizeof(uint8_t), &oob_enable);
    GAPBondlegacy_SetParameter(GAPBOND_PASSKEY, sizeof(uint32_t), &passkey);
    GAPBondlegacy_SetParameter(GAPBOND_BTMODE, sizeof(uint8_t), &bt_mode);
}

/******************************************************************
 * @fn          Initial profile
 * @brief      Add simple profile service and register callbacks 
 *
 * @return     void
 */
void Profile_Init(void)
{
//  TestProfileInit();
//  dip_Init(0x1111, 0x0001, 0x1122, 0x2233);
    void UARTInit(void);
    UARTInit();
    App_Init();
    avrcp_app_init();
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
int TestA2dpAvrcp_main(void)
{
    Board_Init();
    Driver_Init();
    legacy_Init();
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

