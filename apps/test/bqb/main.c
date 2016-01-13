enum { __FILE_NUM__= 0 };

//#include "section_config.h"
//#include "flash_ota.h"
#include "trace.h"
//#include "symboltable_uuid.h"
#include "bqb_demo.h"
#include <stdio.h>

int bqb_main(void) 
{   
    DBG_DIRECT("Bee stack BQB\n");
    
     BQB_Init();
     vTaskStartScheduler();
    
     return 0;
}

void io_assert_failed(uint8_t* file, uint32_t line)
{
    DBG_DIRECT("io driver parameters error! file_name: %s, line: %d", file, line);
    
    for(;;);
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

