//==================================================================
//File Name : rlx4181.s
//Purpose	: provides rlx4181 register access codes
//===================================================================

#include "include/eregdef.h"	// processor register definitions
#include "include/rosboot.h"	// osboot definitions

#define INTR_STACKSIZE 0x80 //0x800			//128 bytes			

#ifndef	LANGUAGE_ASSEMBLY
#define	LANGUAGE_ASSEMBLY
#endif

//===================================================================
//_intr_stack: stack used at interrupt time
//===================================================================
	.bss
    .align  4							//force next address to multiple of 0x80
	.globl  _intr_stack
_intr_stack:
	.space	INTR_STACKSIZE

//===================================================================
//begining of access codes
//===================================================================
	.text


//===================================================================
//uint32 Rlx4181EnableInterrupt(void)
//===================================================================
    .globl	Rlx4181EnableInterrupt
	.ent	Rlx4181EnableInterrupt
Rlx4181EnableInterrupt:
    .set noreorder
	
	// FOR 0360, bluewiz0: 8 usb0: 9 rf0: 10 gpio0: 11 timer0: 12 uart0: 13
	// FOR 0380, bluewiz0: 2 hci dma: 3 bz dma: 4 gpio0: 5 timer0: 6 uart0: 7

	mfc0	t1,C0_SR
	nop
 	and     t1, (0xfffffffd) //enable im[7:0] for 0380
//	and     t1, (0xffffdffd) //enable im[7:0] for 0380
	// check, bf, df, ff
	or		t1,(0x01)
	mtc0	t1, C0_SR
	nop	

	jr		ra
	nop

	.set    reorder
    .end	Rlx4181EnableInterrupt

#ifdef _ENABLE_BTON_POWER_SAVING_
//===================================================================
//uint32 DisableIntForLPS(void)
//===================================================================
    .globl	DisableIntForLPS
	.ent	DisableIntForLPS
DisableIntForLPS:
    .set noreorder
	
	// FOR 0380, bluewiz0: 2 hci dma: 3 bz dma: 4 gpio0: 5 timer0: 6 uart0: 7

	mfc0	t1,C0_SR
	nop
	and     t1, (0xffff03fd) //enable im[7:0] for 0380
	or		t1,(0x800)       //[15:8] <- bluewiz0: 10 hci dma: 11 bz dma: 12 gpio0: 13 timer0: 14 uart0: 15
	mtc0	t1, C0_SR
	nop	

	jr		ra
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop

	.set    reorder
    .end	DisableIntForLPS

#ifdef _ENABLE_RETENTION_FLOW_FOR_DLPS_
//===================================================================
//uint32 DisableIntForDLPS(void)
//===================================================================
    .globl	DisableIntForDLPS
	.ent	DisableIntForDLPS
DisableIntForDLPS:
    .set noreorder
	
	// FOR 0380, bluewiz0: 2 hci dma: 3 bz dma: 4 gpio0: 5 timer0: 6 uart0: 7

	mfc0	t1,C0_SR
	nop
	and     t1, (0xffff03fd) //enable im[7:0] for 0380
	//or		t1,(0x000)       //[15:8] <- bluewiz0: 10 hci dma: 11 bz dma: 12 gpio0: 13 timer0: 14 uart0: 15
	mtc0	t1, C0_SR
	nop	

#if defined(_CHECK_STACK_SIZE_B4_ENTER_DLPS_)
  la      t0, _intr_stack
  sw      sp, 104(t0)
#endif

	jr		ra
	nop

	.set    reorder
    .end	DisableIntForDLPS
#endif

//===================================================================
//uint32 EnableIntForLPS(void)
//===================================================================
    .globl	EnableIntForLPS
	.ent	EnableIntForLPS
EnableIntForLPS:
    .set noreorder
	
	// FOR 0360, bluewiz0: 8 usb0: 9 rf0: 10 gpio0: 11 timer0: 12 uart0: 13
	// FOR 0380, bluewiz0: 2 hci dma: 3 bz dma: 4 gpio0: 5 timer0: 6 uart0: 7

	mfc0	t1,C0_SR
	nop
	and     t1, (0xfffffffd) //enable im[7:0] for 0380
	or		t1,(0xff00)
	mtc0	t1, C0_SR
	nop	

	jr		ra
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop

	.set    reorder
    .end	EnableIntForLPS
#endif

//===================================================================
//uint32 OSCPUSaveSR
//===================================================================
	.globl	OSCPUSaveSR
	.ent	OSCPUSaveSR
OSCPUSaveSR:
	mfc0	v0, C0_SR
	nop
	li	t1, 0xfffffffe

	and	t1, v0, t1
	mtc0	t1, C0_SR
	nop
	
	jr		ra
	.end	OSCPUSaveSR
	
//===================================================================
//uint32 OSCPURestoreSR
//===================================================================
	.globl	OSCPURestoreSR
	.ent	OSCPURestoreSR
OSCPURestoreSR:
	mtc0	a0, C0_SR
	nop
__end_OSCPURestoreSR__:
	jr		ra
	.end	OSCPURestoreSR

//===================================================================
//uint32 rlx4181_ReadCause(void)
//===================================================================
	.globl	Rlx4181ReadCause
	.ent	Rlx4181ReadCause
Rlx4181ReadCause:
	mfc0	v0, C0_CAUSE
	jr		ra
	.end	Rlx4181ReadCause

//===================================================================
//uint32 rlx4181_ReadEPC(void)
//===================================================================
	.globl	Rlx4181ReadEPC
	.ent	Rlx4181ReadEPC
Rlx4181ReadEPC:
	mfc0	v0, C0_EPC
	jr		ra
	.end	Rlx4181ReadEPC

//===================================================================
//void rlx4081_sleep(void)
//===================================================================
	.globl	rlx4081sleep
	.ent	rlx4081sleep
rlx4081sleep:
	sleep
	jr		ra
	.end	rlx4081sleep

//===================================================================
//uint32 rlx4181_ReadSP(void)
//===================================================================
	.globl	Rlx4181ReadSP
	.ent	Rlx4181ReadSP
Rlx4181ReadSP:
	mfc0	v0, sp
	jr		ra
	.end	Rlx4181ReadSP

#ifdef _ENABLE_RETENTION_FLOW_FOR_DLPS_
//===================================================================
//void EnterDlpsHandler
//===================================================================
#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_    
    .extern dlps_save_cpu_to_partial_on_then_enter_deep_lps_mode
#endif
    .globl  EnterDlpsHandler
    .ent    EnterDlpsHandler
    
EnterDlpsHandler:
    .set	noreorder
    .set    noat

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_ 
    SAVE_ALL;    
    la		k0, dlps_save_cpu_to_partial_on_then_enter_deep_lps_mode
    jal		k0
    nop   
    nop
    nop   
    nop
    nop   
    nop    
#endif    
    jr      ra
    nop
    .end    EnterDlpsHandler
#endif

//===================================================================
//void rlx4081_nop_delay(void)
//===================================================================
	.globl	rlx4081NopDelay
	.ent	rlx4081NopDelay
rlx4081NopDelay:
	jr		ra
	nop
	.end	rlx4081NopDelay



#ifdef _CP3_COUNTER_SUPPORTED_
//===================================================================
//void rlx4081_cp3_counter_init(void)
//===================================================================
	.globl	rlx4081Cp3CntInit
	.ent	rlx4081Cp3CntInit
rlx4081Cp3CntInit:
	// t0~t7: $8~$15
	
    /*  Enable CP3 */
	mfc0    t1, $12
	nop
	la      t2, 0x80000000
	or      t1, t2
	mtc0    t1, $12
	nop
	nop

    /* Stop all counters */
	ctc3    zero, $0

    /* Clear all counters */
	mtc3    zero, $8
	mtc3    zero, $9
	mtc3    zero, $10
	mtc3    zero, $11
	mtc3    zero, $12
	mtc3    zero, $13
	mtc3    zero, $14
	mtc3    zero, $15

    /* Start instrcution counter */
	la      t1, 0x11131210
	ctc3    t1, $0

	jr		ra
	nop
	.end	rlx4081Cp3CntInit

//===================================================================
//uint32 rlx4081_Cp3_Stop_Cnt0(void)
//===================================================================
	.globl	rlx4081Cp3StopCnt0
	.ent	rlx4081Cp3StopCnt0
rlx4081Cp3StopCnt0:
    /* Stop all counters */
	ctc3    $0, $0
	mfc3	v0, $8
	nop
	jr		ra
	nop
	.end	rlx4081Cp3StopCnt0

//===================================================================
//uint32 rlx4081_Cp3_Stop_Cnt1(void)
//===================================================================
	.globl	rlx4081Cp3StopCnt1
	.ent	rlx4081Cp3StopCnt1
rlx4081Cp3StopCnt1:
	mfc3	v0, $10
	nop
	jr		ra
	nop
	.end	rlx4081Cp3StopCnt1

//===================================================================
//uint32 rlx4081_Cp3_Stop_Cnt2(void)
//===================================================================
	.globl	rlx4081Cp3StopCnt2
	.ent	rlx4081Cp3StopCnt2
rlx4081Cp3StopCnt2:
	mfc3	v0, $12
	nop
	jr		ra
	nop
	.end	rlx4081Cp3StopCnt2

//===================================================================
//uint32 rlx4081_Cp3_Stop_Cnt3(void)
//===================================================================
	.globl	rlx4081Cp3StopCnt3
	.ent	rlx4081Cp3StopCnt3
rlx4081Cp3StopCnt3:
	mfc3	v0, $14
	nop
	jr		ra
	nop
	.end	rlx4081Cp3StopCnt3
#endif

#undef	LANGUAGE_ASSEMBLY

