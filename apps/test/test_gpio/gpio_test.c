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



void test_gpio_output(void)
{
		RCC_PeriphClockCmd(APBPeriph_GPIO, APBPeriph_GPIO_CLOCK, ENABLE);
		GPIO_InitTypeDef GPIO_InitStruct;
   
    GPIO_InitStruct.GPIO_Pin  = GPIO_Test_Pin;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_ITCmd = DISABLE;   
    GPIO_Init(&GPIO_InitStruct);	
//		GPIO_SetBits(GPIO_Test_Pin);
//		GPIO_ResetBits(GPIO_Test_Pin);
}

void test_gpio_input(void)
{
		RCC_PeriphClockCmd(APBPeriph_GPIO, APBPeriph_GPIO_CLOCK, ENABLE);
		GPIO_InitTypeDef GPIO_InitStruct;
   
    GPIO_InitStruct.GPIO_Pin  = GPIO_Test_Pin;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_ITCmd = DISABLE;
    
    GPIO_Init(&GPIO_InitStruct);
		print_howie("========LINE ========= %d\n",__LINE__);
		print_howie("===GPIO_ReadInputDataBit = 0x%x\n",GPIO_ReadInputDataBit(GPIO_Test_Pin));
}
//#define GPIO0_IRQ    9
void test_gpio_interrupt(void)
{
		RCC_PeriphClockCmd(APBPeriph_GPIO, APBPeriph_GPIO_CLOCK, ENABLE);
    GPIO_InitTypeDef GPIO_InitStruct;
   
    GPIO_InitStruct.GPIO_Pin  = GPIO_Test_Pin;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_ITCmd = ENABLE;
    GPIO_InitStruct.GPIO_ITTrigger = GPIO_INT_Trigger_EDGE;
    GPIO_InitStruct.GPIO_ITPolarity = GPIO_INT_POLARITY_ACTIVE_LOW;
    GPIO_InitStruct.GPIO_ITDebounce = GPIO_INT_DEBOUNCE_ENABLE;  
    GPIO_Init(&GPIO_InitStruct);
    GPIO_INTConfig(GPIO_Test_Pin, ENABLE);
    GPIO_MaskINTConfig(GPIO_Test_Pin, DISABLE);
	    /*  Enable GPIO0 IRQ  */
    NVIC_ClearPendingIRQ((IRQn_Type)GPIO_0_IRQn);
    NVIC_SetPriority((IRQn_Type)GPIO_0_IRQn, 3);
    NVIC_EnableIRQ((IRQn_Type)GPIO_0_IRQn);//GPIO_0_IRQn == 9
}

void test_gpio5_interrupt(void)
{
		RCC_PeriphClockCmd(APBPeriph_GPIO, APBPeriph_GPIO_CLOCK, ENABLE);
    GPIO_InitTypeDef GPIO_InitStruct;
   
    GPIO_InitStruct.GPIO_Pin  = GPIO_Test_Pin;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_ITCmd = ENABLE;
    GPIO_InitStruct.GPIO_ITTrigger = GPIO_INT_Trigger_EDGE;
    GPIO_InitStruct.GPIO_ITPolarity = GPIO_INT_POLARITY_ACTIVE_LOW;
    GPIO_InitStruct.GPIO_ITDebounce = GPIO_INT_DEBOUNCE_ENABLE;  
    GPIO_Init(&GPIO_InitStruct);
    GPIO_INTConfig(GPIO_Test_Pin, ENABLE);
    GPIO_MaskINTConfig(GPIO_Test_Pin, DISABLE);
	    /*  Enable GPIO0 IRQ  */
    NVIC_ClearPendingIRQ((IRQn_Type)GPIO5_IRQ);
    NVIC_SetPriority((IRQn_Type)GPIO5_IRQ, 3);
    NVIC_EnableIRQ((IRQn_Type)GPIO5_IRQ);//GPIO5_IRQ == 68
}

