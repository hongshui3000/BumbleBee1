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
#include "rtl876x_pwm.h"
#include "board.h"
#include "rtl876x.h"



void test_pwm(void)
{
		RCC_PeriphClockCmd(APBPeriph_TIMER, APBPeriph_TIMER_CLOCK, ENABLE);
		TIM_TimeBaseInitTypeDef TIM_InitStruct;
		PWM_InitTypeDef PWM_InitStruct;	
	
    TIM_StructInit(&TIM_InitStruct);
		TIM_InitStruct.TIM_ClockSrc = TIM_CLOCK_10MHZ;
		TIM_InitStruct.TIM_Period = 10;
		TIM_InitStruct.TIM_Mode = 1;
		TIM_TimeBaseInit(TIM2, &TIM_InitStruct);
			
		/*  Enable TIMER2 IRQ  */
//    NVIC_ClearPendingIRQ((IRQn_Type)Timer2_IRQn);
//    NVIC_SetPriority((IRQn_Type)Timer2_IRQn, 3);
//    NVIC_EnableIRQ((IRQn_Type)Timer2_IRQn);//GPIO_0_IRQn == 9

//		TIM_ClearINT(TIM2);
//		TIM_INTConfig(TIM2,ENABLE);
//		TIM_Cmd(TIM2, ENABLE);
	
		PWM_InitStruct.PWM_Period = 20;
    PWM_InitStruct.PWM_Duty = 1;
		PWM_InitStruct.PWM_TIMIndex = 2;
		PWM_Init(PWM0, &PWM_InitStruct);
		PWM_Cmd(PWM0, ENABLE);

}

