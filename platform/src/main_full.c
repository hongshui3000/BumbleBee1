#include "rtl_types.h"
#include "trace.h"
#include "FreeRTOS.h"
#include "task.h"
#include "rtl876x_log_uart.h"
#include "rtl876x_uart.h"
#include "osif.h"
#include "print_howie.h"
#include "rtl876x_rcc.h"

#include "rtl_types.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "trace.h"
#include "rtl876x_gpio.h"
#include "board.h"
#include "rtl876x.h"
#include "rtl876x_tim.h"

/* task handle & queue handle */
xTaskHandle IODemoTaskHandle;
xQueueHandle DemoIOMessageQueue;
xQueueHandle DemoIOEventQueue;

/* functions declaration */
void IO_DemoTask(void* param);
/* queue size & stak size & task priority */
#define IO_DEMO_EVENT_QUEUE_SIZE        0x10
#define IO_DEMO_MESSAGE_QUEUE_SIZE      0x10
#define IO_DEMO_TASK_STACK_SIZE         1024
#define CLOCK_DEMO_TASK_STACK_SIZE         1024
#define IO_DEMO_TASK_PRIORITY           (tskIDLE_PRIORITY + 1)
#define CLOCK_DEMO_TASK_PRIORITY         (tskIDLE_PRIORITY + 1)

#define IO_DEMO_EVENT_UART_RX           0x11


__STATIC_INLINE void hal_enable_bt_bus_clock(void)
{
    //set PERI_ON reg 0x244[0] = 1, bt bus clock enable
    HAL_WRITE32(SYSTEM_REG_BASE, 0x244, 0x1);
}

__STATIC_INLINE void hal_enable_bt_bus_function(void)
{
    //set PERI_ON reg 0x210[2] = 1, bt bus function enable
    HAL_WRITE32(SYSTEM_REG_BASE, 0x210, 0x4);
}

void hal_setup_hardware(void)
{
    /* Enable buffer RAM */
    hal_enable_bt_bus_clock();
    hal_enable_bt_bus_function();
}

void LOGUARTDriverInit(void)
{
    LOG_UART_InitTypeDef logUartInitStruct;


    LOG_UART_StructInit(&logUartInitStruct);
    LOG_UART_Init(LOG_UART, &logUartInitStruct);

}
#define UART_IRQ    Data_Uart_IRQn
void UART_INIT(void)
{
    UART_DeInit(UART);
		/* turn on UART clock */
    RCC_PeriphClockCmd(APBPeriph_UART, APBPeriph_UART_CLOCK, ENABLE);
    
    /******************* uart init *****************************/
    UART_InitTypeDef uartInitStruct;  
    UART_StructInit(&uartInitStruct); 
		uartInitStruct.rxTriggerLevel= UART_RX_FIFO_TRIGGER_LEVEL_14BYTE;
    /* default 115200 baudrate */
    UART_Init(UART, &uartInitStruct); 
		UART_INTConfig(UART, UART_INT_RD_AVA, ENABLE);  
    /*  Enable UART IRQ  */
    NVIC_ClearPendingIRQ((IRQn_Type)UART_IRQ);
    NVIC_SetPriority((IRQn_Type)UART_IRQ, 3);
    NVIC_EnableIRQ((IRQn_Type)UART_IRQ);
}



int main(void)
{
    hal_setup_hardware();
    /* Init Log Uart */
    LOGUARTDriverInit();
		UART_INIT();
		print_howie("========== %d\n",__LINE__);
		//AutoTestInit();	
		//test_timer0();
		//test_timer1();
		//test_timer2();
		//test_timer3();
		//test_timer4();
		//test_timer5();
		//test_timer6();
		//test_timer7();
		//test_timer1_pwm();
		//test_pwm();
		while(1);

	
		
		xTaskCreate(IO_DemoTask, "IO_Demo", IO_DEMO_TASK_STACK_SIZE / sizeof(portSTACK_TYPE), NULL, IO_DEMO_TASK_PRIORITY, &IODemoTaskHandle);
		


    vTaskStartScheduler();

    return 0;
}

void IO_DemoTask(void* param)
{
		uint8_t event = 0;
	  uint8_t rxByte = 0;
    DemoIOEventQueue = xQueueCreate(IO_DEMO_EVENT_QUEUE_SIZE, sizeof(char));
    DemoIOMessageQueue = xQueueCreate(IO_DEMO_MESSAGE_QUEUE_SIZE, sizeof(uint16_t));
		
		test_timer1_pwm();
		while(1)
		{
			
					
//					vTaskDelay(20 / portTICK_RATE_MS);
//					TIMER1_LOAD_COUNT2 = 1;
//					TIM1->LoadCount = 197;
//					vTaskDelay(20 / portTICK_RATE_MS);
//					TIMER1_LOAD_COUNT2 = 1;
//					TIM1->LoadCount = 197;
//					vTaskDelay(20 / portTICK_RATE_MS);
//					TIMER1_LOAD_COUNT2 = 99;
//					TIM1->LoadCount = 99;
			
				if (xQueueReceive(DemoIOEventQueue, &event, portMAX_DELAY) == pdPASS)//portMAX_DELAY
				{
							
							if (event == IO_DEMO_EVENT_UART_RX) 
							{
									 while (xQueueReceive(DemoIOMessageQueue, &rxByte, portMAX_DELAY) == pdPASS)
									 {
									   
									     if(rxByte == 0x11)
											 {
												    //print_howie("have a tst! LINE = %d  filename = %s\n",__LINE__,__FILE__); 
														print_howie("===TIM_GetINTStatus = 0x%x\n",TIM_GetINTStatus(TIM5));
														
														
											 }
											 else if(rxByte == 0x12)
											 {
												
											 }
									 }
							}
				}
		    
		}
}

//		unsigned int *p;
//		p=(unsigned int *)0x40002064;
//		
//		for(int i=0;i<10;i++){
//			print_howie("===0x%x===0x%x=====\n",p,*p);
//			p++;
//		}
//		p = (unsigned int *)0x40002068;
//		print_howie("===0x%x===0x%x=====\n",p,*p);
//		while(1)
//		{
//			print_howie("===0x%x===0x%x=====\n",p,*p);
//		}
