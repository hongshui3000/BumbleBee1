enum { __FILE_NUM__= 0 };

#include "gatttest.h"
#include "gattdcmd.h"

#include <blueapi.h>

//#include "section_config.h"


#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <os_mem.h>



//#include "ota_api.h"
#include "test_cmd.h"
#include "test_transport_uart.h"
//#include "hal_vector_table.h"
//#include "hal_platform.h"
//#include "dlps_platform.h"
#include "test_upperstack_cmd.h"
#include "trace.h"
//#include "rtl876x_rcc.h"

#define TRACE_MODULE_ID     MID_BT_APPL

extern PGATTTest g_pGattTest;

/****************************************************************************/
/* Events                                                                   */
/****************************************************************************/


#define DEMO_EVENT_BLUEAPI_MSG         0x02


 
#define GATTTEST_PRIORITY          (tskIDLE_PRIORITY + 1)   /* Task priorities. */
#define GATTTEST_STACK_SIZE        1024*2

xTaskHandle gattTestTaskHandle;

#define MAX_NUMBER_OF_RX_EVENT     0x20
#define MAX_NUMBER_OF_MESSAGE      0x20



xQueueHandle test_upperstack_QueueHandleEvent;
static xQueueHandle test_upperstack_QueueHandleMessage;
xQueueHandle test_upperstack_QueueHandleTransportRx;

TTestModule * g_pTestModule;

/*
//OTA TEST
#define OTA_IO_GROUP GPIO_GROUP_C
#define OTA_IO_INDEX 7

VOID OTA_ISR_HANDLER(VOID *Data)
{



    GPIO_MaskInterrupt(OTA_IO_GROUP*8 + OTA_IO_INDEX);
    dfuSetOtaMode(TRUE);    
    NVIC_SystemReset();
}
  
VOID OTA_Interrupt_IO()
{
    GPIO_Initialize_Param dlps_isr_int_param;
    IRQ_HANDLE OTA_INT_IrqHandle;

    
    dlps_isr_int_param.GPIOGroup = GPIO_GROUP_C;
    dlps_isr_int_param.GPIOIndex = 7;
    dlps_isr_int_param.GPIOMode  = GPIO_MODE_INT;
    dlps_isr_int_param.INTConfiguration.INT_Enabler  = GPIO_INT_ENABLE;
    dlps_isr_int_param.INTConfiguration.INT_Level    = GPIO_INT_EDGE_SENSITIVE;
    dlps_isr_int_param.INTConfiguration.INT_Polarity = GPIO_INT_POLARITY_ACTIVE_LOW;
    dlps_isr_int_param.INTConfiguration.INT_Debounce = GPIO_INT_DEBOUNCE_ENABLE;   
    GPIO_Initialize(&dlps_isr_int_param);
    OTA_INT_IrqHandle.IrqNum   = GPIO_IRQ;
    OTA_INT_IrqHandle.IrqFun   = (IRQ_FUN)OTA_ISR_HANDLER;
    OTA_INT_IrqHandle.Priority = 0;
    OTA_INT_IrqHandle.GPIO_Irq_Num = dlps_isr_int_param.INTPinIndex;

    // Modified GPIO register function
    GPIO_RegisterIRQ(&OTA_INT_IrqHandle);
}

 VOID DLPS_Status_IO()
{
    GPIO_Initialize_Param dlps_status_pin_param;
    dlps_status_pin_param.GPIOGroup = GPIO_GROUP_C;
    dlps_status_pin_param.GPIOIndex = 8;
    dlps_status_pin_param.GPIOMode  = GPIO_MODE_IN;
    GPIO_Initialize(&dlps_status_pin_param);
}

*/

/****************************************************************************/
/* Fatal error                                                              */
/****************************************************************************/

static void test_upperstack_FatalError(int Line)
{
//  printf("GATTD: Fatal error (line %d)\n\r", Line);
}





/****************************************************************************/
/* BlueAPI Callback.                                                        */
/****************************************************************************/

void test_upperstack_BlueAPICallback(PBlueAPI_UsMessage pMsg)
{
    unsigned char Event = DEMO_EVENT_BLUEAPI_MSG;

    if (xQueueSend(test_upperstack_QueueHandleMessage, &pMsg, 0) == errQUEUE_FULL)
    {
        test_upperstack_FatalError(__LINE__);

        blueAPI_BufferRelease(pMsg);
    }
    else if (xQueueSend(test_upperstack_QueueHandleEvent, &Event, 0) == errQUEUE_FULL) 
    {
        test_upperstack_FatalError(__LINE__);
    }
}
#if 0
//#ifdef CONFIG_DLPS_EN
BOOL DLPS_CheckTestUpperTask()
{
    if (g_pGattTest->bEnterDlps == true)
    {
    	DBG_BUFFER(MODULE_APP, LEVEL_INFO, "DLPS_CheckTestUpperTask true\n\n", 0);
        return true;
    }
    else
    {
    	DBG_BUFFER(MODULE_APP, LEVEL_INFO, "DLPS_CheckTestUpperTask flase\n\n", 0);
        return false;
    }
}

extern VOID ActiveTime_Restart(UINT16 restart_val);
void TestUpperTask_DLPS_Exit()
{
//    g_pGattTest->bEnterDlps = false;
	DBG_BUFFER(MODULE_APP, LEVEL_INFO, "TestUpperTask_DLPS_Exit\n\n", 0);
    ActiveTime_Restart(200);    
}
//#endif
#endif
/****************************************************************************/
/* TASK                                                                     */
/****************************************************************************/
//#define DEBUG_TEST_SCHEDULE_TASK
#ifdef DEBUG_TEST_SCHEDULE_TASK
 
#include <trace_binary.h>
#define TRACE_MODULE_ID     MID_BT_HCI
 
    uint32_t  test7 = 0;
    uint32_t  test8= 0;
#endif

static void test_upperstack_Task(void *pParameters)
{
    char RxChar;
    char Event;

    test_upperstack_QueueHandleEvent   = xQueueCreate((MAX_NUMBER_OF_MESSAGE + MAX_NUMBER_OF_RX_EVENT),
                                         sizeof(unsigned char));
    test_upperstack_QueueHandleMessage = xQueueCreate(MAX_NUMBER_OF_MESSAGE,
                                         sizeof(PBlueAPI_UsMessage));
    
		//RCC_PeriphClockCmd(APBPeriph_UART, APBPeriph_UART_CLOCK, ENABLE);
    test_upperstack_UARTInit();

    test_CmdInit(g_pTestModule);
    test_upperstack_CmdInit();

//    OTA_Interrupt_IO();
//#ifdef CONFIG_DLPS_EN
    //LPS_MODE_Set(LPM_DLPS_MODE);
    //LPS_MODE_Pause();

/*
    if (FALSE == DLPS_ENTER_CHECK_CB_REG(DLPS_CheckTestUpperTask))
    {

    }
*/
    //DLPS_INTERRUPT_CONTROL_CB_REG(TestUpperTask_DLPS_Exit, DLPS_EXIT);

//#endif
    

    while (true)
    {
        if (xQueueReceive(test_upperstack_QueueHandleEvent, &Event, portTICK_RATE_MS * 1000) == pdPASS)
        {
#ifdef DEBUG_TEST_SCHEDULE_TASK
//        reset_vendor_counter();
        test7= read_vendor_counter_no_display();
#endif
            if (Event & DEMO_EVENT_UART_RX)      /* User interface */
            {
                while (xQueueReceive(test_upperstack_QueueHandleTransportRx, &RxChar, 0) == pdPASS)
                {
                    test_CmdCollect(g_pTestModule, &RxChar, sizeof(RxChar));
                }
            }
            if (Event & DEMO_EVENT_BLUEAPI_MSG)  /* BlueAPI */
            {
                PBlueAPI_UsMessage pMsg;

                while (xQueueReceive(test_upperstack_QueueHandleMessage, &pMsg, 0) == pdPASS)
                {
                    test_upperstack_HandleBlueAPIMessage(g_pGattTest, pMsg);
                }
            }

        //osRescheduleTriggerCallBack();

#ifdef DEBUG_TEST_SCHEDULE_TASK
        test8 = read_vendor_counter_no_display();
        HCI_TRACE_PRINTF_2(HCI_TRACE_MASK_ERROR, "t7:%08x, t8:%08x", test7, test8);
#endif
            
        }
    }
}


/****************************************************************************/
/* GATT Init                                                                */
/****************************************************************************/
void gattTestInit( void )
{
    g_pTestModule = osMemoryAllocate(RAM_TYPE_DATA_ON, sizeof(TTestModule));

    
    xTaskCreate(test_upperstack_Task, "gattTest", GATTTEST_STACK_SIZE / sizeof(portSTACK_TYPE),
                NULL, GATTTEST_PRIORITY, &gattTestTaskHandle);
}

