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
#include "rtl876x_tim.h"
#include "board.h"
#include "rtl876x.h"

uint32_t gTimer1PWM;



void test_timer2(void)
{
		RCC_PeriphClockCmd(APBPeriph_TIMER, APBPeriph_TIMER_CLOCK, ENABLE);
		TIM_TimeBaseInitTypeDef TIM_InitStruct;
	
    TIM_StructInit(&TIM_InitStruct);
		TIM_InitStruct.TIM_ClockSrc = TIM_CLOCK_10MHZ;
		TIM_InitStruct.TIM_Period = 1000*10000;
		TIM_InitStruct.TIM_Mode = 1;
		TIM_TimeBaseInit(TIM2, &TIM_InitStruct);
			
		/*  Enable TIMER2 IRQ  */
    NVIC_ClearPendingIRQ((IRQn_Type)Timer2_IRQn);
    NVIC_SetPriority((IRQn_Type)Timer2_IRQn, 3);
    NVIC_EnableIRQ((IRQn_Type)Timer2_IRQn);//GPIO_0_IRQn == 9

		TIM_ClearINT(TIM2);
		TIM_INTConfig(TIM2,ENABLE);
		TIM_Cmd(TIM2, ENABLE);

}

void test_timer1_pwm(void)
{
#if 1
		RCC_PeriphClockCmd(APBPeriph_TIMER, APBPeriph_TIMER_CLOCK, ENABLE);
		TIM_TimeBaseInitTypeDef TIM_InitStruct;
	
    TIM_StructInit(&TIM_InitStruct);
		TIM_InitStruct.TIM_ClockSrc = TIM_CLOCK_10MHZ;
		TIM_InitStruct.TIM_Period = 0xaa;
		TIM_InitStruct.TIM_Mode = 1;
		TIM_TimeBaseInit(TIM1, &TIM_InitStruct);
			/*  Enable TIMER2 IRQ  */
    NVIC_ClearPendingIRQ((IRQn_Type)BTMAC_IRQn);
    NVIC_SetPriority((IRQn_Type)BTMAC_IRQn, 3);
    NVIC_EnableIRQ((IRQn_Type)BTMAC_IRQn);
	
		TIM1->ControlReg |= (1<<3);
		TIMER1_LOAD_COUNT2 = 0xaa>>2;
	
		TIM_ClearINT(TIM1);
		TIM_INTConfig(TIM1,ENABLE);
		TIM_Cmd(TIM1, ENABLE);
	#endif
	
#if 0
			RCC_PeriphClockCmd(APBPeriph_TIMER, APBPeriph_TIMER_CLOCK, ENABLE);
		TIM_TimeBaseInitTypeDef TIM_InitStruct;
	
    TIM_StructInit(&TIM_InitStruct);
		TIM_InitStruct.TIM_ClockSrc = TIM_CLOCK_10MHZ;
		TIM_InitStruct.TIM_Period = 0xaa;
		TIM_InitStruct.TIM_Mode = 1;
		TIM_TimeBaseInit(TIM1, &TIM_InitStruct);
		TIM1->ControlReg |= (1<<3);
		TIMER1_LOAD_COUNT2 = 0xaa>>4;
			
		/*  Enable TIMER2 IRQ  */
    NVIC_ClearPendingIRQ((IRQn_Type)BTMAC_IRQn);
    NVIC_SetPriority((IRQn_Type)BTMAC_IRQn, 3);
    NVIC_EnableIRQ((IRQn_Type)BTMAC_IRQn);//GPIO_0_IRQn == 9

		TIM_ClearINT(TIM1);
		TIM_INTConfig(TIM1,ENABLE);
		TIM_Cmd(TIM1, ENABLE);
#endif
	
}



void test_timer3(void)
{
		RCC_PeriphClockCmd(APBPeriph_TIMER, APBPeriph_TIMER_CLOCK, ENABLE);
		TIM_TimeBaseInitTypeDef TIM_InitStruct;
	
    TIM_StructInit(&TIM_InitStruct);
		TIM_InitStruct.TIM_ClockSrc = TIM_CLOCK_10MHZ;
		TIM_InitStruct.TIM_Period = 5;
		TIM_InitStruct.TIM_Mode = 1;
		TIM_TimeBaseInit(TIM3, &TIM_InitStruct);
			
		/*  Enable TIMER3 IRQ  */
    NVIC_ClearPendingIRQ((IRQn_Type)TIMER3_IRQ);
    NVIC_SetPriority((IRQn_Type)TIMER3_IRQ, 3);
    NVIC_EnableIRQ((IRQn_Type)TIMER3_IRQ);//GPIO_0_IRQn == 9

		TIM_ClearINT(TIM3);
		TIM_INTConfig(TIM3,ENABLE);
		TIM_Cmd(TIM3, ENABLE);

}

void test_timer4(void)
{
		RCC_PeriphClockCmd(APBPeriph_TIMER, APBPeriph_TIMER_CLOCK, DISABLE);
		RCC_PeriphClockCmd(APBPeriph_TIMER, APBPeriph_TIMER_CLOCK, ENABLE);
		TIM_TimeBaseInitTypeDef TIM_InitStruct;
	
    TIM_StructInit(&TIM_InitStruct);
		TIM_InitStruct.TIM_ClockSrc = TIM_CLOCK_10MHZ;
		TIM_InitStruct.TIM_Period = 1000*10000;
		TIM_InitStruct.TIM_Mode = 1;
		TIM_TimeBaseInit(TIM4, &TIM_InitStruct);
			
		/*  Enable TIMER5 IRQ  */
    NVIC_ClearPendingIRQ((IRQn_Type)TIMER4_IRQ);
    NVIC_SetPriority((IRQn_Type)TIMER4_IRQ, 3);
    NVIC_EnableIRQ((IRQn_Type)TIMER4_IRQ);//GPIO_0_IRQn == 9

		TIM_ClearINT(TIM4);
		TIM_INTConfig(TIM4,ENABLE);
		TIM_Cmd(TIM4, ENABLE);

}
void test_timer5(void)
{
		RCC_PeriphClockCmd(APBPeriph_TIMER, APBPeriph_TIMER_CLOCK, DISABLE);
		RCC_PeriphClockCmd(APBPeriph_TIMER, APBPeriph_TIMER_CLOCK, ENABLE);
		TIM_TimeBaseInitTypeDef TIM_InitStruct;
	
    TIM_StructInit(&TIM_InitStruct);
		TIM_InitStruct.TIM_ClockSrc = TIM_CLOCK_10MHZ;
		TIM_InitStruct.TIM_Period = 1000*10000;
		TIM_InitStruct.TIM_Mode = 1;
		TIM_TimeBaseInit(TIM5, &TIM_InitStruct);
			
		/*  Enable TIMER5 IRQ  */
    NVIC_ClearPendingIRQ((IRQn_Type)TIMER5_IRQ);
    NVIC_SetPriority((IRQn_Type)TIMER5_IRQ, 3);
    NVIC_EnableIRQ((IRQn_Type)TIMER5_IRQ);//GPIO_0_IRQn == 9

		TIM_ClearINT(TIM5);
		TIM_INTConfig(TIM5,ENABLE);
		TIM_Cmd(TIM5, ENABLE);

}

void test_timer6(void)
{
		RCC_PeriphClockCmd(APBPeriph_TIMER, APBPeriph_TIMER_CLOCK, DISABLE);
		RCC_PeriphClockCmd(APBPeriph_TIMER, APBPeriph_TIMER_CLOCK, ENABLE);
		TIM_TimeBaseInitTypeDef TIM_InitStruct;
	
    TIM_StructInit(&TIM_InitStruct);
		TIM_InitStruct.TIM_ClockSrc = TIM_CLOCK_10MHZ;
		TIM_InitStruct.TIM_Period = 1000*10000;
		TIM_InitStruct.TIM_Mode = 1;
		TIM_TimeBaseInit(TIM6, &TIM_InitStruct);
			
		/*  Enable TIMER6 IRQ  */
    NVIC_ClearPendingIRQ((IRQn_Type)TIMER6_IRQ);
    NVIC_SetPriority((IRQn_Type)TIMER6_IRQ, 3);
    NVIC_EnableIRQ((IRQn_Type)TIMER6_IRQ);//GPIO_0_IRQn == 9

		TIM_ClearINT(TIM6);
		TIM_INTConfig(TIM6,ENABLE);
		TIM_Cmd(TIM6, ENABLE);

}

void test_timer7(void)
{
		RCC_PeriphClockCmd(APBPeriph_TIMER, APBPeriph_TIMER_CLOCK, DISABLE);
		RCC_PeriphClockCmd(APBPeriph_TIMER, APBPeriph_TIMER_CLOCK, ENABLE);
		TIM_TimeBaseInitTypeDef TIM_InitStruct;
	
    TIM_StructInit(&TIM_InitStruct);
		TIM_InitStruct.TIM_ClockSrc = TIM_CLOCK_10MHZ;
		TIM_InitStruct.TIM_Period = 1000*10000;
		TIM_InitStruct.TIM_Mode = 1;
		TIM_TimeBaseInit(TIM7, &TIM_InitStruct);
			
		/*  Enable TIMER5 IRQ  */
    NVIC_ClearPendingIRQ((IRQn_Type)TIMER7_IRQ);
    NVIC_SetPriority((IRQn_Type)TIMER7_IRQ, 3);
    NVIC_EnableIRQ((IRQn_Type)TIMER7_IRQ);//GPIO_0_IRQn == 9

		TIM_ClearINT(TIM7);
		TIM_INTConfig(TIM7,ENABLE);
		TIM_Cmd(TIM7, ENABLE);

}

void test_timer0(void)
{
		RCC_PeriphClockCmd(APBPeriph_TIMER, APBPeriph_TIMER_CLOCK, ENABLE);
		TIM_TimeBaseInitTypeDef TIM_InitStruct;
	
    TIM_StructInit(&TIM_InitStruct);
		TIM_InitStruct.TIM_ClockSrc = TIM_CLOCK_10MHZ;
		TIM_InitStruct.TIM_Period = 10000000;
		TIM_InitStruct.TIM_Mode = 1;
		TIM_TimeBaseInit(TIM0, &TIM_InitStruct);
			
		/*  Enable TIMER0 IRQ  */
    NVIC_ClearPendingIRQ((IRQn_Type)BTMAC_IRQn);
    NVIC_SetPriority((IRQn_Type)BTMAC_IRQn, 3);
    NVIC_EnableIRQ((IRQn_Type)BTMAC_IRQn);//GPIO_0_IRQn == 9

		TIM_ClearINT(TIM0);
		TIM_INTConfig(TIM0,ENABLE);
		TIM_Cmd(TIM0, ENABLE);
}

void test_timer1(void)
{
		RCC_PeriphClockCmd(APBPeriph_TIMER, APBPeriph_TIMER_CLOCK, ENABLE);
		TIM_TimeBaseInitTypeDef TIM_InitStruct;
	
    TIM_StructInit(&TIM_InitStruct);
		TIM_InitStruct.TIM_ClockSrc = TIM_CLOCK_10MHZ;
		TIM_InitStruct.TIM_Period = 10000000;
		TIM_InitStruct.TIM_Mode = 1;
		TIM_TimeBaseInit(TIM1, &TIM_InitStruct);
			
		/*  Enable TIMER1 IRQ  */
    NVIC_ClearPendingIRQ((IRQn_Type)BTMAC_IRQn);
    NVIC_SetPriority((IRQn_Type)BTMAC_IRQn, 3);
    NVIC_EnableIRQ((IRQn_Type)BTMAC_IRQn);//GPIO_0_IRQn == 9

		TIM_ClearINT(TIM1);
		TIM_INTConfig(TIM1,ENABLE);
		TIM_Cmd(TIM1, ENABLE);
}

