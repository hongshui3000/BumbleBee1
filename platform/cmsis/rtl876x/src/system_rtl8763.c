#include "rtl_types.h"
#include "rtl876x.h"
#include "pingpong_buffer.h"
#include "trace.h"

uint32_t SystemCoreClock = PLATFORM_CLOCK;

int main(void);

void SystemInit(void)
{
    DBG_FWsim_Log("SystemInit\n");

    /* PingPong Buffer Init */
    PPB_Init();

    /* Enable the memory management fault , Bus Fault, Usage Fault exception */
    //SCB->SHCSR |= (SCB_SHCSR_MEMFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk | SCB_SHCSR_USGFAULTENA_Msk);

    /* Ensure all priority bits are assigned as preemption priority bits. */
    NVIC_SetPriorityGrouping( 0 );

    #if (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 10*2) |                 /* set CP10 Full Access */
                   (3UL << 11*2)  );               /* set CP11 Full Access */
    #endif

    #ifdef UNALIGNED_SUPPORT_DISABLE
    SCB->CCR |= SCB_CCR_UNALIGN_TRP_Msk;
    #endif

    // relocate vector table to internal ram
    // updates also VTOR
    // TODO
}

void ProgramStart(void)
{
    DBG_FWsim_Log("ProgramStart\n");

    main();

    DBG_FWsim_Log("while(1)\n");
    while(1);
}

///////////////////////////////////////////////////////////////

// ISR handler (exception)
void NMI_Handler(void)
{
    while(1);
}

void HardFault_Handler_C(uint32_t * hardfault_args, uint32_t lr_value)
{
    (void)hardfault_args[0];    /* R0    */
    (void)hardfault_args[1];    /* R1    */
    (void)hardfault_args[2];    /* R2    */
    (void)hardfault_args[3];    /* R3    */
    (void)hardfault_args[4];    /* R12   */
    (void)hardfault_args[5];    /* LR    */
    (void)hardfault_args[6];    /* PC    */
    (void)hardfault_args[7];    /* PSR   */
    (void)SCB->CFSR;            /* CFSR  */
    (void)SCB->HFSR;            /* HFSR  */
    (void)SCB->DFSR;            /* DFSR  */
    (void)SCB->AFSR;            /* AFSR  */
    (void)SCB->MMFAR;           /* MMFAR */
    (void)SCB->BFAR;            /* BFAR  */

    //  printf("[HardFault]\n");
    //  printf("- Stack frame:\n");
    //  printf(" R0 = %x\n", stacked_r0);
    //  printf(" R1= %x\n", stacked_r1);
    //  printf(" R2 = %x\n", stacked_r2);
    //  printf(" R3 = %x\n", stacked_r3);
    //  printf(" R12 = %x\n", stacked_r12);
    //  printf(" LR = %x\n", stacked_lr);
    //  printf(" PC = %x\n", stacked_pc);
    //  printf(" PSR = %x\n", stacked_psr);
    //  printf("- FSR/:\n");
    //  printf(" CFSR = %x\n", cfsr);
    //  printf(" HFSR = %x\n", SCB->HFSR); //0xE000ED2C
    //  printf(" DFSR = %x\n", SCB->DFSR);
    //  printf(" AFSR = %x\n", SCB->AFSR);
    //
    //  if(cfsr & 0x0080)
    //      printf(" MMFAR = %x\n", memmanage_fault_address);
    //  if(cfsr & 0x8000)
    //      printf(" BFAR = %x\n", bus_fault_address);
    //  printf("- Misc\n");
    //  printf(" LR/EXC_RETURN = %x\n", lr_value);


    while(1);  //endless loop
}

void MemManage_Handler(void)
{
    SCB->CFSR;   //0xE000ED28 byte[0]
    SCB->MMFAR;  //0xE000ED34
    while(1);
}

void BusFault_Handler(void)
{
    SCB->CFSR;   //0xE000ED29 byte[1]
    SCB->BFAR;  //0xE000ED38
    while(1);
}

void UsageFault_Handler(void)
{
    SCB->CFSR;   //0xE000ED2A byte[2..3]
    while(1);
}

void DebugMon_Handler(void)
{
    SCB->DFSR;   //0xE000ED30
    while(1);
}


void SVC_Handler_C(unsigned int * svc_args)
{
    (void)((char *)svc_args[6])[-2]; /* SVC number */
    (void)svc_args[0];  /* R0 */
    (void)svc_args[1];  /* R1 */
    (void)svc_args[2];  /* R2 */
    (void)svc_args[3];  /* R3 */

    // ... other processing

    // Return result (e.g. sum of first two arguments)
    svc_args[0] = svc_args[0] + svc_args[1];

    return;
}

// ISR handler (interrupt)
void system_Handler(void)
{
    while(1);
}

void WDG_Handler_C(uint32_t * hardfault_args, uint32_t lr_value)
{
    (void)hardfault_args[0];    /* R0  */
    (void)hardfault_args[1];    /* R1  */
    (void)hardfault_args[2];    /* R2  */
    (void)hardfault_args[3];    /* R3  */
    (void)hardfault_args[4];    /* R12 */
    (void)hardfault_args[5];    /* LR  */
    (void)hardfault_args[6];    /* PC  */
    (void)hardfault_args[7];    /* PSR */

//  printf("[HardFault]\n");
//  printf("- Stack frame:\n");
//  printf(" R0 = %x\n", stacked_r0);
//  printf(" R1= %x\n", stacked_r1);
//  printf(" R2 = %x\n", stacked_r2);
//  printf(" R3 = %x\n", stacked_r3);
//  printf(" R12 = %x\n", stacked_r12);
//  printf(" LR = %x\n", stacked_lr);
//  printf(" PC = %x\n", stacked_pc);
//  printf(" PSR = %x\n", stacked_psr);

    // TODO WDG_SystemReset?
    NVIC_SystemReset();
}

void Default_Handler(void)
{
    while(1);
}


extern uint32_t __Vectors;
void Peripheral_Handler(void)
{
    uint32_t PeriIrqStatus, CheckIndex, ExactIrqStatus, TableIndex;
    PeriIrqStatus = PERIPHINT->STATUS;

    /* For GPIO interrupt */
    UINT32 gpio_int = 0x0;
    UINT32 index = 0;
    //Save exact IRQ status
    ExactIrqStatus = PeriIrqStatus & (PERIPHINT->EN);

    //Check exact IRQ function
    for(CheckIndex = 0;CheckIndex<32;CheckIndex++) 
    {
        if (ExactIrqStatus & BIT(CheckIndex)) 
        {
            if (CheckIndex == 0 ||
                CheckIndex == 1 ||
                CheckIndex == 2 ||
                CheckIndex == 3 ||
                CheckIndex == 4 ||
                CheckIndex == 5 ||
                CheckIndex == 6 ||
                CheckIndex == 7 ||
                CheckIndex == 8 ||
                CheckIndex == 9   
            )
            {
                IRQ_FUN pFun = (IRQ_FUN)*(&__Vectors+ CheckIndex + 48);
                pFun();
            }
            else if (CheckIndex == 16)
            {
                gpio_int = GPIO->INTSTATUS;
                for (index = 6; index < 32; index++)
                {  
                    if(gpio_int&BIT(index))
                    {
                        IRQ_FUN pFun = (IRQ_FUN)*(&__Vectors+(52 + index));
                        pFun();
                    }
                }
            }
            else
                ;
        }
    }
		    //Clear sub-rout IRQ
    HAL_WRITE32(PERI_INT_REG_BASE, 0, PeriIrqStatus);
}

