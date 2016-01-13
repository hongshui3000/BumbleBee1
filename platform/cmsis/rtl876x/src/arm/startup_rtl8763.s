Stack_Size      EQU     0x0
                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp    EQU     0x10020000 // BB3 SRAM 128K

Heap_Size       EQU     0x0
                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit

                PRESERVE8
                THUMB

; Vector Table Mapped to Address 0 at Reset
                AREA    |.vectors_table|, DATA, READONLY, ALIGN=8
				EXPORT  __Vectors
                EXPORT  __Vectors_End
                EXPORT  __Vectors_Size

__Vectors       DCD     __initial_sp              ; Top of Stack
                DCD     Reset_Handler             ; Reset Handler
                DCD     NMI_Handler               ; NMI Handler
                DCD     HardFault_Handler         ; Hard Fault Handler
                DCD     MemManage_Handler         ; MPU Fault Handler
                DCD     BusFault_Handler          ; Bus Fault Handler
                DCD     UsageFault_Handler        ; Usage Fault Handler
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     SVC_Handler               ; SVCall Handler
                DCD     DebugMon_Handler          ; Debug Monitor Handler
                DCD     0                         ; Reserved
                DCD     PendSV_Handler            ; PendSV Handler
                DCD     SysTick_Handler           ; SysTick Handler

                ; External Interrupts
                ; sync from BumbleBee3_Address_Mapping_20151116.xls
                DCD     system_Handler             ;[0]
                DCD     WDG_Handler                ;[1]  Watch dog global insterrupt
                DCD     BTMAC_Handler              ;[2]  See Below Table ( an Extension of interrupt )
                DCD     USB_Handler                ;[3]  USB IP interrupt
                DCD     Timer2_Handler             ;[4]  Timer2 global interrupt
                DCD     SDIO_Host_Handler          ;[5]  SDIO Host interrupt
                DCD     I2S0_Handler               ;[6]  I2S0 interrupt
                DCD     I2S1_Handler               ;[7]  I2S1 interrupt
                DCD     Data_Uart1_Handler         ;[8]  Data_Uart1 interrupt (used for DSP)
                DCD     GPIO_0_Handler             ;[9]  GPIO 0 interrupt
                DCD     GPIO_1_Handler             ;[10] GPIO 1 interrupt
                DCD     Log_Uart_Handler           ;[11] Log_Uart interrupt (Log0)
                DCD     Data_Uart_Handler          ;[12] Data_Uart interrupt
                DCD     RTC_Handler                ;[13] Realtime counter interrupt
                DCD     SPI0_Handler               ;[14] SPI0 interrupt
                DCD     SPI1_Handler               ;[15] SPI1 interrupt
                DCD     I2C0_Handler               ;[16] I2C0 interrupt
                DCD     I2C1_Handler               ;[17] I2C1 interrupt
                DCD     ADC_Handler                ;[18] ADC global interrupt
                DCD     Peripheral_Handler         ;[19] See Below Table ( an Extension of interrupt )
                DCD     GDMA0_Channel0_Handler     ;[20] RTK-DMA0 channel 0 global interrupt
                DCD     GDMA0_Channel1_Handler     ;[21] RTK-DMA0 channel 1 global interrupt
                DCD     GDMA0_Channel2_Handler     ;[22] RTK-DMA0 channel 2 global interrupt
                DCD     GDMA0_Channel3_Handler     ;[23] RTK-DMA0 channel 3 global interrupt
                DCD     GDMA0_Channel4_Handler     ;[24] RTK-DMA0 channel 4 global interrupt
                DCD     GDMA0_Channel5_Handler     ;[25] RTK-DMA0 channel 5 global interrupt
                DCD     keyscan_Handler            ;[26] keyscan global interrupt
                DCD     qdecode_Handler            ;[27] qdecode global interrupt
                DCD     IR_Handler                 ;[28] IR module global interrupt
                DCD     DSP_Handler                ;[29] DSP interrupt
                DCD     GDMA0_Channel6             ;[30]
                DCD     GDMA0_Channel7             ;[31]
				;Peripheral Interrupts not special interrupt
				DCD     SPIFLASHIntrHandler         ;48
                DCD     Gpio2IntrHandler            ;49
                DCD     Gpio3IntrHandler            ;50
                DCD     Gpio4IntrHandler            ;51
                DCD     Gpio5IntrHandler            ;52
                DCD     Timer3IntrHandler           ;53
                DCD     Timer4IntrHandler           ;54
                DCD     Timer5IntrHandler           ;55
                DCD     Timer6IntrHandler           ;56
                DCD     Timer7IntrHandler           ;57
                DCD     Gpio6IntrHandler            ;58
                DCD     Gpio7IntrHandler
                DCD     Gpio8IntrHandler
                DCD     Gpio9IntrHandler
                DCD     Gpio10IntrHandler
                DCD     Gpio11IntrHandler
                DCD     Gpio12IntrHandler
                DCD     Gpio13IntrHandler
                DCD     Gpio14IntrHandler
                DCD     Gpio15IntrHandler
                DCD     Gpio16IntrHandler
                DCD     Gpio17IntrHandler
                DCD     Gpio18IntrHandler
                DCD     Gpio19IntrHandler
                DCD     Gpio20IntrHandler
                DCD     Gpio21IntrHandler
                DCD     Gpio22IntrHandler
                DCD     Gpio23IntrHandler
                DCD     Gpio24IntrHandler
                DCD     Gpio25IntrHandler
                DCD     Gpio26IntrHandler
                DCD     Gpio27IntrHandler
                DCD     Gpio28IntrHandler
                DCD     Gpio29IntrHandler
                DCD     Gpio30IntrHandler
                DCD     Gpio31IntrHandler
				DCD		Timer0IntrHandler
				DCD 	Timer1IntrHandler
__Vectors_End

__Vectors_Size  EQU     __Vectors_End - __Vectors

                AREA    |.vectors_table_text|, CODE, READONLY, ALIGN=8


; Reset Handler

Reset_Handler   PROC
                EXPORT  Reset_Handler             [WEAK]
                IMPORT  SystemInit
                IMPORT  ProgramStart

                ldr     r0, =__initial_sp
                mov     sp, r0

#if 1
                ; NVIC disable Interrupt
                ldr r0,=0xE000E180
                ldr r1,=0xFFFFFFFF
                str r1, [r0]
#endif

                IMPORT  |Load$$DATA_RAM_ON$$RW$$Base|
                IMPORT  |Image$$DATA_RAM_ON$$RW$$Base|
                IMPORT  |Image$$DATA_RAM_ON$$RW$$Length|
                IMPORT  |Load$$DATA_RAM_OFF$$RW$$Base|
                IMPORT  |Image$$DATA_RAM_OFF$$RW$$Base|
                IMPORT  |Image$$DATA_RAM_OFF$$RW$$Length|
                IMPORT  |Image$$DATA_RAM_ON$$ZI$$Base|
                IMPORT  |Image$$DATA_RAM_ON$$ZI$$Length|
                IMPORT  |Image$$DATA_RAM_OFF$$ZI$$Base|
                IMPORT  |Image$$DATA_RAM_OFF$$ZI$$Length|

                ; Rom data copy(on area)
                ldr r0,=|Load$$DATA_RAM_ON$$RW$$Base|
                ldr r1,=|Image$$DATA_RAM_ON$$RW$$Base|
                ldr r2,=|Image$$DATA_RAM_ON$$RW$$Length|
                cmp r2, #0
                beq end_of_copy_loop1
copy_loop1
                ldr r3, [r0]
                adds r0, r0, #4
                str r3, [r1]
                adds r1, r1, #4
                subs r2, r2, #4
                bne copy_loop1
end_of_copy_loop1

                ; Ram Bss Clear(on area)
                ldr r0,=|Image$$DATA_RAM_ON$$ZI$$Base|
                ldr r1,=|Image$$DATA_RAM_ON$$ZI$$Length|
                cmp r1, #0
                beq end_of_clear_loop1
                ldr r2,=0x0
clear_loop1
                str r2, [r0]
                adds r0, r0, #4
                subs r1, r1, #4
                bne clear_loop1
end_of_clear_loop1

                ; Rom data copy(off area)
                ldr r0,=|Load$$DATA_RAM_OFF$$RW$$Base|
                ldr r1,=|Image$$DATA_RAM_OFF$$RW$$Base|
                ldr r2,=|Image$$DATA_RAM_OFF$$RW$$Length|
                cmp r2, #0
                beq end_of_copy_loop2
copy_loop2
                ldr r3, [r0]
                adds r0, r0, #4
                str r3, [r1]
                adds r1, r1, #4
                subs r2, r2, #4
                bne copy_loop2
end_of_copy_loop2

                ; Ram Bss Clear(off area)
                ldr r0,=|Image$$DATA_RAM_OFF$$ZI$$Base|
                ldr r1,=|Image$$DATA_RAM_OFF$$ZI$$Length|
                cmp r1, #0
                beq end_of_clear_loop2
                ldr r2,=0x0
clear_loop2
                str r2, [r0]
                adds r0, r0, #4
                subs r1, r1, #4
                bne clear_loop2
end_of_clear_loop2


;ready for using C code
                ldr     r0, =SystemInit
                blx     r0
                ldr     r0, =ProgramStart
                bx      r0
                ENDP

; Exception Handlers
HardFault_Handler PROC
                IMPORT  HardFault_Handler_C
                TST     LR, #4
                ITE     EQ
                MRSEQ   R0, MSP
                MRSNE   R0, PSP
                MOV     R1, LR
                B       HardFault_Handler_C
                ENDP

;Todo
;SVC_Handler     PROC
                ;IMPORT  SVC_Handler_C
                ;TST     LR, #4
                ;ITE     EQ
                ;MRSEQ   R0, MSP
                ;MRSNE   R0, PSP
                ;B       SVC_Handler_C
                ;ENDP

WDG_Handler PROC
                IMPORT  WDG_Handler_C
                TST     LR, #4
                ITE     EQ
                MRSEQ   R0, MSP
                MRSNE   R0, PSP
                MOV     R1, LR
                B       WDG_Handler_C
                ENDP

Default_Handler PROC
                EXPORT  NMI_Handler                      [WEAK]
                EXPORT  SVC_Handler                      [WEAK]
                EXPORT  PendSV_Handler                   [WEAK]
                EXPORT  SysTick_Handler                  [WEAK]
                EXPORT  system_Handler                   [WEAK]
                EXPORT  BTMAC_Handler                    [WEAK]
                EXPORT  USB_Handler                      [WEAK]
                EXPORT  Timer2_Handler                   [WEAK]
                EXPORT  SDIO_Host_Handler                [WEAK]
                EXPORT  I2S0_Handler                     [WEAK]
                EXPORT  I2S1_Handler                     [WEAK]
                EXPORT  Data_Uart1_Handler               [WEAK]
                EXPORT  GPIO_0_Handler                   [WEAK]
                EXPORT  GPIO_1_Handler                   [WEAK]
                EXPORT  Log_Uart_Handler                 [WEAK]
                EXPORT  Data_Uart_Handler                [WEAK]
                EXPORT  RTC_Handler                      [WEAK]
                EXPORT  SPI0_Handler                     [WEAK]
                EXPORT  SPI1_Handler                     [WEAK]
                EXPORT  I2C0_Handler                     [WEAK]
                EXPORT  I2C1_Handler                     [WEAK]
                EXPORT  ADC_Handler                      [WEAK]
                EXPORT  Peripheral_Handler               [WEAK]
                EXPORT  GDMA0_Channel0_Handler           [WEAK]
                EXPORT  GDMA0_Channel1_Handler           [WEAK]
                EXPORT  GDMA0_Channel2_Handler           [WEAK]
                EXPORT  GDMA0_Channel3_Handler           [WEAK]
                EXPORT  GDMA0_Channel4_Handler           [WEAK]
                EXPORT  GDMA0_Channel5_Handler           [WEAK]
                EXPORT  keyscan_Handler                  [WEAK]
                EXPORT  qdecode_Handler                  [WEAK]
                EXPORT  IR_Handler                       [WEAK]
                EXPORT  DSP_Handler                      [WEAK]
                EXPORT  MemManage_Handler                [WEAK]
                EXPORT  BusFault_Handler                 [WEAK]
                EXPORT  UsageFault_Handler               [WEAK]
                EXPORT  DebugMon_Handler                 [WEAK]
				;Peripheral Interrupts not special interrupt
				EXPORT  SPIFLASHIntrHandler         [WEAK]
                EXPORT  Timer3IntrHandler           [WEAK]
                EXPORT  Timer4IntrHandler           [WEAK]
                EXPORT  Timer5IntrHandler           [WEAK]
                EXPORT  Timer6IntrHandler           [WEAK]
                EXPORT  Timer7IntrHandler           [WEAK]

                ;EXPORT  Gpio0IntrHandler           [WEAK]
                ;EXPORT  Gpio1IntrHandler           [WEAK]
                EXPORT  Gpio2IntrHandler            [WEAK]
                EXPORT  Gpio3IntrHandler            [WEAK]
                EXPORT  Gpio4IntrHandler            [WEAK]
                EXPORT  Gpio5IntrHandler            [WEAK]
                EXPORT  Gpio6IntrHandler            [WEAK]
                EXPORT  Gpio7IntrHandler            [WEAK]
                EXPORT  Gpio8IntrHandler            [WEAK]
                EXPORT  Gpio9IntrHandler            [WEAK]
                EXPORT  Gpio10IntrHandler           [WEAK]
                EXPORT  Gpio11IntrHandler           [WEAK]
                EXPORT  Gpio12IntrHandler           [WEAK]
                EXPORT  Gpio13IntrHandler           [WEAK]
                EXPORT  Gpio14IntrHandler           [WEAK]
                EXPORT  Gpio15IntrHandler           [WEAK]
                EXPORT  Gpio16IntrHandler           [WEAK]
                EXPORT  Gpio17IntrHandler           [WEAK]
                EXPORT  Gpio18IntrHandler           [WEAK]
                EXPORT  Gpio19IntrHandler           [WEAK]
                EXPORT  Gpio20IntrHandler           [WEAK]
                EXPORT  Gpio21IntrHandler           [WEAK]
                EXPORT  Gpio22IntrHandler           [WEAK]
                EXPORT  Gpio23IntrHandler           [WEAK]
                EXPORT  Gpio24IntrHandler           [WEAK]
                EXPORT  Gpio25IntrHandler           [WEAK]
                EXPORT  Gpio26IntrHandler           [WEAK]
                EXPORT  Gpio27IntrHandler           [WEAK]
                EXPORT  Gpio28IntrHandler           [WEAK]
                EXPORT  Gpio29IntrHandler           [WEAK]
                EXPORT  Gpio30IntrHandler           [WEAK]
                EXPORT  Gpio31IntrHandler           [WEAK]
				EXPORT  Timer0IntrHandler           [WEAK]
				EXPORT  Timer1IntrHandler           [WEAK]
NMI_Handler
SVC_Handler
PendSV_Handler
SysTick_Handler
system_Handler
BTMAC_Handler
USB_Handler
Timer2_Handler
SDIO_Host_Handler
I2S0_Handler
I2S1_Handler
Data_Uart1_Handler
GPIO_0_Handler
GPIO_1_Handler
Log_Uart_Handler
Data_Uart_Handler
RTC_Handler
SPI0_Handler
SPI1_Handler
I2C0_Handler
I2C1_Handler
ADC_Handler
Peripheral_Handler
GDMA0_Channel0_Handler
GDMA0_Channel1_Handler
GDMA0_Channel2_Handler
GDMA0_Channel3_Handler
GDMA0_Channel4_Handler
GDMA0_Channel5_Handler
keyscan_Handler
qdecode_Handler
IR_Handler
DSP_Handler
GDMA0_Channel6
GDMA0_Channel7
MemManage_Handler
BusFault_Handler
UsageFault_Handler
DebugMon_Handler
;Peripheral Interrupts not special interrupt
SPIFLASHIntrHandler
Timer3IntrHandler
Timer4IntrHandler
Timer5IntrHandler
Timer6IntrHandler
Timer7IntrHandler

;Gpio0IntrHandler
;Gpio1IntrHandler
Gpio2IntrHandler
Gpio3IntrHandler
Gpio4IntrHandler
Gpio5IntrHandler
Gpio6IntrHandler
Gpio7IntrHandler
Gpio8IntrHandler
Gpio9IntrHandler
Gpio10IntrHandler
Gpio11IntrHandler
Gpio12IntrHandler
Gpio13IntrHandler
Gpio14IntrHandler
Gpio15IntrHandler
Gpio16IntrHandler
Gpio17IntrHandler
Gpio18IntrHandler
Gpio19IntrHandler
Gpio20IntrHandler
Gpio21IntrHandler
Gpio22IntrHandler
Gpio23IntrHandler
Gpio24IntrHandler
Gpio25IntrHandler
Gpio26IntrHandler
Gpio27IntrHandler
Gpio28IntrHandler
Gpio29IntrHandler
Gpio30IntrHandler
Gpio31IntrHandler
Timer0IntrHandler
Timer1IntrHandler
                B       .
                ENDP


                ALIGN

; User Initial Stack & Heap
                IF      :DEF:__MICROLIB

                EXPORT  __initial_sp
                EXPORT  __heap_base
                EXPORT  __heap_limit

                ELSE

                IMPORT  __use_two_region_memory
                EXPORT  __user_initial_stackheap

__user_initial_stackheap PROC
                LDR     R0, =  Heap_Mem
                LDR     R1, =(Stack_Mem + Stack_Size)
                LDR     R2, = (Heap_Mem +  Heap_Size)
                LDR     R3, = Stack_Mem
                BX      LR
                ENDP

                ALIGN

                ENDIF


                END

