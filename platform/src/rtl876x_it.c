/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
**********************************************************************************************************
* @file		
* @brief	
* @details
* @author	
* @date 	
* @version	
*********************************************************************************************************
*/

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
#include "board.h"
#include "rtl876x_tim.h"

#define IO_DEMO_EVENT_UART_RX           0x11

extern xQueueHandle DemoIOMessageQueue;
extern xQueueHandle DemoIOEventQueue;	
extern uint32_t gTimer1PWM;;

void BTMAC_Handler(void)
{
	//print_howie("========LINE ========= %d\n",__LINE__);
	Timer1IntrHandler();
}


void GPIO_0_Handler(void)//
{
	print_howie("========LINE ========= %d\n",__LINE__);
	GPIO_MaskINTConfig(GPIO_Test_Pin, ENABLE);
    
  // add user code here

    
  GPIO_ClearINTPendingBit(GPIO_Test_Pin);
  GPIO_MaskINTConfig(GPIO_Test_Pin, DISABLE);

}
void GPIO_1_Handler(void)//
{

}

void Gpio5IntrHandler(void)//
{
	print_howie("========LINE ========= %d\n",__LINE__);
	GPIO_MaskINTConfig(GPIO_Test_Pin, ENABLE);
    
  // add user code here
    
  GPIO_ClearINTPendingBit(GPIO_Test_Pin);
  GPIO_MaskINTConfig(GPIO_Test_Pin, DISABLE);

}

void Timer0IntrHandler(void)
{
	print_howie("========LINE ========= %d\n",__LINE__);
	TIM_ClearINT(TIM0);    
}

void Timer1IntrHandler(void)
{
	print_howie("========LINE ========= %d\n",__LINE__);
	TIM_ClearINT(TIM1);    
}

void Timer2_Handler(void)
{
	print_howie("========LINE ========= %d\n",__LINE__);
	TIM_ClearINT(TIM2);    
}
void Timer3IntrHandler(void)
{
	print_howie("========LINE ========= %d\n",__LINE__);
	TIM_ClearINT(TIM3);    
}
void Timer4IntrHandler(void)
{
	print_howie("========LINE ========= %d\n",__LINE__);
	TIM_ClearINT(TIM4);    
}
void Timer5IntrHandler(void)
{
	print_howie("========LINE ========= %d\n",__LINE__);
	TIM_ClearINT(TIM5);
}
void Timer6IntrHandler(void)
{
	print_howie("========LINE ========= %d\n",__LINE__);
	TIM_ClearINT(TIM6);    
}
void Timer7IntrHandler(void)
{
	print_howie("========LINE ========= %d\n",__LINE__);
	TIM_ClearINT(TIM7);    
}




void Data_Uart_Handler(void)
{
    uint32_t int_status = 0;
    uint8_t event = 0;
    uint8_t rxByte = 0;
    portBASE_TYPE TaskWoken = pdFALSE;
    
    /* read interrupt id */
    int_status = UART_GetIID(UART);
    /* disable interrupt */
    UART_INTConfig(UART, UART_INT_RD_AVA | UART_INT_LINE_STS, DISABLE);
    
    switch(int_status)
    {
        /* tx fifo empty, not enable */
        case UART_INT_ID_TX_EMPTY:
        break;

        /* rx data valiable */
        case UART_INT_ID_RX_LEVEL_REACH:
            /* rx trigger is 14, not possible */
            break;
        
        case UART_INT_ID_RX_TMEOUT:
            while(UART_GetFlagState(UART, UART_FLAG_RX_DATA_RDY) == SET)
            {
                rxByte = UART_ReceiveByte(UART);
            }
            event = IO_DEMO_EVENT_UART_RX;
            /* send message to DemoIO task */
            xQueueSendFromISR(DemoIOMessageQueue, &rxByte, &TaskWoken);//DemoIOEventQueue
            xQueueSendFromISR(DemoIOEventQueue, &event, &TaskWoken);

        break;
        
        /* receive line status interrupt */
        case UART_INT_ID_LINE_STATUS:
            //DBG_BUFFER(MODULE_APP, LEVEL_ERROR, "Line status error!!!!\n", 0);
        break;      

        default:
        break;
    }
    
    /* enable interrupt again */
    //UART_INTConfig(UART, UART_INT_RD_AVA | UART_INT_LINE_STS, ENABLE);
		UART_INTConfig(UART, UART_INT_RD_AVA, ENABLE);
    portEND_SWITCHING_ISR(TaskWoken);
    
    return;
}
void RTCIntrHandler(void)
{

}

