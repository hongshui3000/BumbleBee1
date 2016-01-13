
#ifndef	LANGUAGE_ASSEMBLY
#define	LANGUAGE_ASSEMBLY
#endif

//==================================================================
// for 0380 CPU configuration:
//		RLX4181/KINMEM	----Yes
//		DMEM	----Yes (64KB)
//		IMEM	----No
//		DCACHE	----No
//		ICACHE	----Yes(8KB)
//Compiler enviroment:
//		rsdk-elf-gcc
//		rsdk-elf-ar
//		rsdk-elf-ld
//		all run in GNU situation
//===================================================================

#include "include/eregdef.h"	// processor register definitions
#include "include/rosboot.h"	// osboot definitions
#include "include/initial.h"

/*===================================================================
**
** Entry Point:  The osboot entry point label is "_osboot".  When
**               linking osboot into application code this label should
**               be pass in to the linker (usually via a -e _osboot) as
**               the entry point for the program.
**
**               Note that there are three _osboot labels below, each
**               under conditional preprocess logic for the different
**               types of build environments.  Only one will ever be
**               valid for a particular environment.
**
**===================================================================*/

/*-------------------------------------------------------------------
**  ROM(SR.BEV=1) exception handlers:
**
**  BFC00000	reset
**  BFC00100    UTLB Miss (TLB equiped processors only)
**				SMMU is adopted, so NO TLB exception will happen.
**  BFC00180    General Exception Address
**-------------------------------------------------------------------
*/

/*-------------------------------------------------------------------
**  RAM(SR.BEV=0) exception handlers:
**
**  80000000    UTLB Miss (TLB equiped processors only)
**				SMMU is adopted, so NO TLB exception will happen.
**				So, we could arrange code start from 0x8000_0000.
**  80000080    General Exception Address
**-------------------------------------------------------------------
*/

    //-----------------------------------
    //_osboot will be set at 0x8000_0000
    //-----------------------------------
    .globl	_osboot
    .section ".osboot", "ax"
    .set	noreorder
    .ent	_osboot	
_osboot:

    //---sys_clk=40MHz, BW_enble=1, USB_SIE_EN=1---
    //---usbphy_rst=1, modem_rst=1				---
    //---sysclk:32K-->40MHz, BW_clk_en=1		---
#ifdef _BOOT_32K_SWITCH_40M_
    //(*((volatile UINT32*)(0xb000a000))) = 0x32f;
    la	t0, 0xb000a000
    la	t1, VENDOR_00_VAL1
    sw	t1, 0(t0)
    nop
#endif//_BOOT_32K_SWITCH_40M_

    //-------------------------------------------------------------------
    //to distiguish it is boot-on reset or TLB exception
    //-------------------------------------------------------------------
    mfc0	k0, C0_CAUSE
    nop
    and	k0, 0x7f
    bne	k0, zero, bt_interrupt
    nop

    //--------------------------------------------------------------------
    //After reset, SR.IEc=0, no interrupt will be responsed.
    //In RAM version, RDC supplied ROM code(0xbfc0_0000) is a "LOOP", so
    //after probe download RAM code, SR.IEc=0 at this time.
    //--------------------------------------------------------------------

    //-----------------------------------
    //enable 6 external INT and 2 SW INT
    //-----------------------------------
    mfc0	s0, C0_SR
    nop
#ifdef USE_DMEM
    //enable CP3
    or		s0, s0, 0x80000000 //  SR_IMASK
#endif

    //enable interrupt 0 ~ 7
    or		s0, s0, 0x0000ff00 

    //------------------------------------------------------
    //After reset, SR.BEV=1 and exception entry=0xbfc0_0080.
    //Set SR.BEV=0, now exception entry=0x8000_0080.
    //------------------------------------------------------

    and		s0, s0, ~SR_BEV  // BEV = 0;exception 0x8000-0080; BEV = 1;exception 0xbfc0-0180

    //--------------------------------------------------------------
    //SR.KUc=0 Adjust the current SR to kernel mode(0x8000_0000) 
    //SR.IEc=0 Disable Interrupr 
    //--------------------------------------------------------------
    and		s0, 0xFFFFFFFC
    mtc0	s0, C0_SR
    nop

    //-----------------------------------------------------
    //More boot steps are arranged at following code.
    //Do not exceed 0x8000_0080(exception entry)
    //-----------------------------------------------------
    b	boot
    nop

    //--------------------------------------------------
    //0x8000_0080
    //we put "jump gen_exception" at the address
    //--------------------------------------------------
    .global exception_label
    .align	7				//0x8000_0080
exception_label:
    //jump to interrupt handler
    j	bt_interrupt
    nop


//	.set	reorder
//	.set	at
//========================================================================
//boot:Performs processor initialization, and starts user's application.
//========================================================================
    .align	4				//0x8000_0090
boot:
    .set	noreorder
    .set	reorder

    la		ra, os_init
    jr		ra

    .end	_osboot
/*-------------------------------------------------------------------
**
**  os_init: Initialize application stack pointers.
**			This entry does not return, but instead transitions to
**           the start of the users application, in our case, to
**			 to our ROS starting point.
**
**   syntax: users sp = os_init( a0, a1, a2 )
**
**		r4/a0 = application entry point
**			(usually start in crt0.s)
**		r5/a1 = application high water mark/end address.
**			(the highest address referenced in the download
**			 process)
**		r6/a2 = minimum stack size requested.  This will be the
**			size of	the largest BSS section named ".mstack"
**			found in the loaded files.  Normally, this is
**			not provided by a user and will simply be 0, in
**			which case, the default APPL_STACKSIZE is used.
**
**-------------------------------------------------------------------
*/
    .global os_init
    .ent	os_init
os_init:
    .set	noreorder

    //-----------------------------------------------------------------
    //After reset, all valid bits for IMEM, DMEM, ICACHE, DCACHE
    //will be clear by LMI.
    //So, we need not flush CACHE here. But still supplied code here.
    //-----------------------------------------------------------------
#ifdef USE_DMEM
    //declare Dmem range
    la		t0, __dmem_start_addr 
    la		t1, 0x7fffffff 
    and		t0, t0, t1 //physical base address of DMEM
    mtc3	t0, $4
    nop
    la		t0, __dmem_end_addr	
    and		t0, t0, t1
    mtc3	t0, $5
    nop

    //enable Dmem
    mfc0	t0, C0_CCTL
    nop
    and		t0, 0xfffffbff
    mtc0	t0, C0_CCTL		//Dmemon-->0
    nop
    or		t0, 0x00000400
    mtc0	t0, C0_CCTL		//Dmemon-->1
    nop
#else
    //do nothing
#endif//USE_DMEM     



#ifdef SPI_FLASH_BOOT
//---------------------------------------------------
// [1] Copy SPI_init code to SRAM (0xa010,0000)
// [2] Run SPI_init code
// [3] Return here and clear SPI_init code in SRAM
//--------------------------------------------------- 
	//[1] copy spi init code
	la    a0,    _spi_start
    la    a1,    _spi_end
    la    a2,    _spi_lma
    nop
_spi_code_copy:
    lw    t1,     0x0(a2)
    nop
    sw    t1,     0x0(a0)
    addu  a0,     4
    addu  a2,     4
_spi_code_copy_check:
    bltu  a0,     a1,    _spi_code_copy
    nop
    la    t0,	  spic_init //jump to a 32bit address, this will change PC			     
    jal	  t0
    nop
   
	//[3]return hear and clear spi init code in SRAM        
_spi_init_return_add:
	la    a0,    _spi_start						//0x14C
    la    a1,    _spi_end						//0x150
    nop											//0x154
_spi_code_clear:
    sw    $0,     0x0(a0)
    addu  a0,     4
_spi_code_clear_check:
    bltu  a0,     a1,    _spi_code_clear
    nop	

#ifdef SPI_CRITICAL_CODE_IN_SRAM
    /* added by austin to move fast code of flash to SRAM */
#if !defined(_NEW_8812B_CPU_PLATFORM_)
    la    a0,    0x80050000						// copied start address in flash 						
    la    a1,    0x80056000						// copied end address in flash    
#else
    la    a0,    0x80060000						// copied start address in flash 						
    la    a1,    0x80066000						// copied end address in flash   
#endif
    la    a2, 	 0x8010A000						// the target address of sram
    nop											
_spi_fw_code_copy_to_sram:
    lw    t1,     0x0(a0)
    nop
    sw    t1,     0x0(a2)
    addu  a0,     4
    addu  a2,     4
    bltu  a0,     a1,    _spi_fw_code_copy_to_sram
    nop	
#endif
#endif//SPI_FLASH_BOOT	

#ifdef _ENABLE_RETENTION_FLOW_FOR_DLPS_
    //-------------------------------------------------------------------
    //to distiguish it is boot-on reset or DLPS reset
    //(check bit 0 of BTON Register 0xA0 "0xb000a0a0")  
    //-------------------------------------------------------------------
    la  t0, 0xb000a0a0
    la  t1, 0x00000001
    lw  t2, 0(t0)
    nop
    and t1, t2
    beq	t1, zero, _tag_ori_boot_flow_
    nop

    /* clear indication about retention */
    and t2, 0xfffffffe
    sw  t2, 0(t0)

    j   ResumeFromDlps
    nop
    
_tag_ori_boot_flow_:	
#endif

	//------------------------------------------
	//linker supplied global data pointer _gp
	//_gp=ALIGN(16) between .sdata and .sbss
	//in link script file. used to init gp reg
	//------------------------------------------
	.extern _gp
	la		gp,_gp

#ifdef _SUPPORT_MEMORY_TEST_SIMULATION_ONLY_
    la    t0, 0xb000a0a0
    lw    t2, 0(t0)    
    nop
    and   t2, 0x1fffffff /* clear bit 31, 30, 29 */
    sw    t2, 0(t0)

    /*------------------------------*/
    /*          rom test            */
    /*------------------------------*/
    la    a0, __rom_start_addr        /* copied start address of rom */						
    la    a1, __rom_end_addr          /* copied end address of rom */  
    and   a1, 0xfffffffc
    la    a2, 0

_add_checksum_of_rom_mem_:    
    lw    t1,    0(a0)
    nop
    addu  a2,    t1 
    addu  a0,    4
    bltu  a0,    a1,  _add_checksum_of_rom_mem_    
    nop

    lw    t1,    0(a0)
    nop
    beq	  t1,    a2,   _is_rom_test_pass_
    nop       
    or    t2,    0x20000000       
    sw    t2,    0(t0)                 /* set bit 29 */     
_is_rom_test_pass_:  


    /*------------------------------*/
    /*          ram test            */
    /*------------------------------*/   
    la    a1, 0x800
    la    a0, __sram0_start_addr       /* copied start of top address in partial-on sram */

_is_partial_on_sram_top_test_:    
    la    a2, 0x55555555
    nop
    sw    a2, 0(a0)     
    lw    t1, 0(a0)
    nop    
    bne   a2, t1, _is_sram_test_fail_
    nop
    
    la    a2, 0xaaaaaaaa
    nop
    sw    a2, 0(a0)     
    lw    t1, 0(a0)
    nop    
    bne   a2, t1, _is_sram_test_fail_
    nop

    add   a0,  4
    sub   a1,  4
    beqz  a1,  _is_partial_on_sram_top_test_
    nop

        
    la    a1, 0x800
    la    a0, __sram0_end_addr   /* copied end of bottom address in partial-on sram */ 

_is_partial_on_sram_end_test_:    
    and   a0, 0xfffffffc
    
    la    a2, 0x55555555
    nop    
    sw    a2, 0(a0)     
    lw    t1, 0(a0)
    nop    
    bne   a2, t1, _is_sram_test_fail_
    nop
    
    la    a2, 0xaaaaaaaa
    nop
    sw    a2, 0(a0)     
    lw    t1, 0(a0)
    nop    
    bne   a2, t1, _is_sram_test_fail_
    nop

    sub   a1,  4
    beqz  a1,  _is_partial_on_sram_end_test_
    nop

    la    a1, 0x800    
    la    a0, __sram1_start_addr    /* copied start of top address in partial-off sram */						

_is_partial_off_sram_top_test_:    
    la    a2, 0x55555555
    nop
    sw    a2, 0(a0)     
    lw    t1, 0(a0)
    nop    
    bne   a2, t1, _is_sram_test_fail_
    nop
    
    la    a2, 0xaaaaaaaa
    nop
    sw    a2, 0(a0)     
    lw    t1, 0(a0)
    nop    
    bne   a2, t1, _is_sram_test_fail_
    nop

    add   a0,  4
    sub   a1,  4
    beqz  a1,  _is_partial_off_sram_top_test_
    nop


    la    a1, 0x800
    la    a0, __sram1_end_addr    /* copied end of bottom address in partial-off sram */ 

_is_partial_off_sram_end_test_:    
    and   a0, 0xfffffffc
    
    la    a2, 0x55555555
    nop    
    sw    a2, 0(a0)     
    lw    t1, 0(a0)
    nop    
    bne   a2, t1, _is_sram_test_fail_
    nop
    
    la    a2, 0xaaaaaaaa
    nop
    sw    a2, 0(a0)     
    lw    t1, 0(a0)
    nop    
    bne   a2, t1, _is_sram_test_fail_
    nop

    sub   a1,  4
    beqz  a1,  _is_partial_off_sram_end_test_
    nop 
    
    b     _is_sram_test_pass_
    nop
    
_is_sram_test_fail_:      
    or    t2,    0x40000000       
    sw    t2,    0(t0)                 /* set bit 30 */

_is_sram_test_pass_: 

    /*------------------------------*/
    /*         dmem test            */
    /*------------------------------*/   
    la    a1, 0x800
    la    a0, __dmem0_start_addr       /* copied start of top address in partial-on dmem */

_is_partial_on_dmem_top_test_:    
    la    a2, 0x55555555
    nop
    sw    a2, 0(a0)     
    lw    t1, 0(a0)
    nop    
    bne   a2, t1, _is_dmem_test_fail_
    nop
    
    la    a2, 0xaaaaaaaa
    nop
    sw    a2, 0(a0)     
    lw    t1, 0(a0)
    nop    
    bne   a2, t1, _is_dmem_test_fail_
    nop

    add   a0,  4
    sub   a1,  4
    beqz  a1,  _is_partial_on_dmem_top_test_
    nop

        
    la    a1, 0x800
    la    a0, __dmem0_end_addr      /* copied end of bottom address in partial-on dmem */ 

_is_partial_on_dmem_end_test_:    
    and   a0, 0xfffffffc
    
    la    a2, 0x55555555
    nop    
    sw    a2, 0(a0)     
    lw    t1, 0(a0)
    nop    
    bne   a2, t1, _is_dmem_test_fail_
    nop
    
    la    a2, 0xaaaaaaaa
    nop
    sw    a2, 0(a0)     
    lw    t1, 0(a0)
    nop    
    bne   a2, t1, _is_dmem_test_fail_
    nop

    sub   a1,  4
    beqz  a1,  _is_partial_on_dmem_end_test_
    nop

    la    a1, 0x800    
    la    a0, __dmem1_start_addr      /* copied start of top address in partial-off dmem */						

_is_partial_off_dmem_top_test_:    
    la    a2, 0x55555555
    nop
    sw    a2, 0(a0)     
    lw    t1, 0(a0)
    nop    
    bne   a2, t1, _is_dmem_test_fail_
    nop
    
    la    a2, 0xaaaaaaaa
    nop
    sw    a2, 0(a0)     
    lw    t1, 0(a0)
    nop    
    bne   a2, t1, _is_dmem_test_fail_
    nop

    add   a0,  4
    sub   a1,  4
    beqz  a1,  _is_partial_off_dmem_top_test_
    nop


    la    a1, 0x800
    la    a0, __dmem1_end_addr     /* copied end of bottom address in partial-off dmem */ 

_is_partial_off_dmem_end_test_:    
    and   a0, 0xfffffffc
    
    la    a2, 0x55555555
    nop    
    sw    a2, 0(a0)     
    lw    t1, 0(a0)
    nop    
    bne   a2, t1, _is_dmem_test_fail_
    nop
    
    la    a2, 0xaaaaaaaa
    nop
    sw    a2, 0(a0)     
    lw    t1, 0(a0)
    nop    
    bne   a2, t1, _is_dmem_test_fail_
    nop

    sub   a1,  4
    beqz  a1,  _is_partial_off_dmem_end_test_
    nop 
    b     _is_sram_test_pass_
    nop
    
_is_dmem_test_fail_:      
    or    t2,    0x80000000       
    sw    t2,    0(t0)                 /* set bit 31 */

_is_dmem_test_pass_: 
#endif /* end of _SUPPORT_MEMORY_TEST_SIMULATION_ONLY_ */


//{{{added by liuyong 20090927
#ifdef USE_DMEM
	//----------------------------------------------------------
	// clear dmem to zero
	// (only clean the bss section)
	//----------------------------------------------------------
    //la		t0, __RAM_DATA_SDAT_START_ADD	//dmem begin add
    la 		t0, _gp
    la		t1, __dmem_end_addr			//dmem end add
    la		t2, 0x00000000
2:
    sw      t2, 0(t0)
    addu	t0, t0, 4
    blt		t0, t1, 2b
    nop
#endif	
//}}}added by liuyong 20090927	
	
	//----------------------------------------------------------
	//copy __RAM_DATA_SDAT_END_ADD - __RAM_DATA_SDAT_START_ADD
	//bytes from __ROM_DATA_SDAT_START_ADD to
	//__RAM_DATA_SDAT_START_ADD
	//----------------------------------------------------------
    la		t0, __ROM_DATA_SDAT_START_ADD		//src
    la		t1, __RAM_DATA_SDAT_START_ADD		//dst
    la		t2, __RAM_DATA_SDAT_END_ADD			
1:
    lw      t3, 0(t0)
    nop
    sw      t3, 0(t1)
    addu	t0, t0, 4
    addu	t1, t1, 4
    blt		t1, t2, 1b
    nop
	
	.set	reorder

	//---------------------------------------------------------
	// set up initial program start, stack, and heap values
	// application linked in with osboot
	// APPL_STACK_SIZE=4K. Is 4K stack size enough? (austin)
	//---------------------------------------------------------
    la      t0,__dmem_end_addr
    and     t0, 0xFFFFFFF8      //align it(8 byte aligned???) 
    move    s7,t0		        //save users sp in s7

	//-----------------------------------------------------
	// We are about to start the application
	//-----------------------------------------------------
	.set	noreorder
	.extern main
#if defined(_CHECK_STACK_SIZE_B4_ENTER_DLPS_) && defined(_ENABLE_RETENTION_FLOW_FOR_DLPS_)	
	.extern g_dmem_end_addr;
    la    t0, __dmem_end_addr
    la    t1, g_dmem_end_addr
    sw    t0, 0(t1)
#endif
    la      t1, main
    move	sp,s7
    move	ra,r0			//clear out ra for stack walking
    jal		t1				//use jump here so ra remains 0, jump to the application start address
    nop		

	//------------------------------
	//never come here
	//------------------------------
    rfe
    .set	reorder
    .end	os_init

	//-----------------------------------
	//bt_interrupt is started here
	//-----------------------------------
    .extern BtIntrHandler
    .globl	bt_interrupt
    .ent	bt_interrupt
bt_interrupt:
    .set	noreorder
    .set    noat	

    SAVE_ALL
    la		k0, BtIntrHandler	    // general exception handler in C
    jal		k0
    nop

    RESTORE_ALL
    mfc0	k1, C0_EPC              // load EPC
    nop

    jr		k1                      // return from exception
    rfe                                
    .end	bt_interrupt

#ifdef _ENABLE_RETENTION_FLOW_FOR_DLPS_
    //-----------------------------------
    //CPU resume function from deep LPS mode
    //-----------------------------------
#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_    
    .extern dlps_restore_io_mem_from_partial_on
#endif
    .globl  ResumeFromDlps
    .ent    ResumeFromDlps
ResumeFromDlps:
    .set    noreorder
    .set    noat    

    la  k0, _intr_stack;
    nop;
    
    /* because we can not damage original stack memory content in partial on 
       area and it was kept before enter DLPS mode, we must get previous sp 
       first before link C code */
    lw  sp, 104(k0)
    nop

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
    la  t1, dlps_restore_io_mem_from_partial_on
    jal t1
    nop
#endif    

    RESTORE_ALL

    jr      ra
    nop
    nop                             
    .end    ResumeFromDlps
#endif


#ifdef EXCEPTION_REPORT_SUPPORT
    .extern ErrExcptHandler
    .globl	err_exception
    .ent	err_exception
err_exception:
    .set	noreorder
    la		k0, ErrExcptHandler     // general exception handler in C
    jal		k0
    nop
    .end	err_exception
#endif//EXCEPTION_REPORT_SUPPORT

#ifdef SPI_FLASH_BOOT
    .globl spic_init
    .section ".spi_sec", "ax"
    .set noreorder
    .ent spic_init
spic_init:
//######################################################################
//#### Switch to quad channel ####
    //---Setup SPIC---
    la $3, 0xb0008000
#ifndef SPI_ONE_BIT_MODE
    sw $0, 0x8($3)                  //Disable SPIC         
    li $2, 0x200
    sw $2, 0x120($3)                //single channel, blocking write
    li $2, 0x30000
    sw $2, 0x11c($3)                //SPI Address byte = 3, read dummy cycle =0
    li $2, 0x1
    sw $2, 0x118($3)                //SPI Address byte = 1 in user mode
    sw $2, 0x10($3)                 //set SER
    sw $2, 0x8($3)                  //Enable SPIC

    //---Write enable---
    li $2, 0x6
    sb $2, 0x60($3)                 //WREN

check_wren_busy:
    lw $4, 0x28($3)
    li $2, 0x6
    bne $2, $4, check_wren_busy
    nop

    //---Write Status---
    sw $0, 0x8($3)                  //Disable SPIC
    li $2, 0x1
    sb $2, 0x60($3)                 //WRSR
    //   li $2, 0x40
    li $2, 0x7c                     //QUAD_bit=1, BP3,BP2,BP1,BP0=1 for block sw protect
    sb $2, 0x60($3)                 //QUAD Enable
    li $2, 0x1
    sw $2, 0x8($3)                  //Enable SPIC

    //---Read Status---
check_wr_busy:
    lw $4, 0x28($3)
    li $2, 0x6
    bne $2, $4, check_wr_busy
    nop

    //---Setup SPIC---
    sw $0, 0x8($3)                  //Disable SPIC
    li $2, 0x300
    sw $2, 0x0($3)                  //EEPROM mode
    li $2, 0x1
    sw $2, 0x4($3)                  //read one byte back
    sw $2, 0x8($3)                  //Enable SPIC

read_flash_status:
    li $2, 0x5                      //0x78
    sb $2, 0x60($3)                 //RDSR		//0x7C


check_rd_busy:
    lw $4, 0x28($3)                 //0x80
    li $2, 0xe                      //0x84
    bne $2, $4, check_rd_busy       //0x88
    nop                             //0x8C

    lw $2, 0x60($3)                 //0x90
    li $4, 0x3                      //0x94
    beq $2, $4, read_flash_status   //0x98
    nop                             //0x9C

#endif /* end of SPI_ONE_BIT_MODE */

    sw $0, 0x8($3)                  //Disable SPIC	//0xA0

    sw $0, 0x0($3)                  //SPI mode 0	//0xA4

#if (SPI_CLK_DIV == 1)
    li $2, 0x2                      //BAUD Rate = 1/4 ssclk
#elif (SPI_CLK_DIV == 2)
    li $2, 0x4                      //BAUD Rate = 1/8 ssclk
#else
    li $2, 0x1                      //BAUD Rate = 1/2 ssclk
#endif

    sw $2, 0x14($3)                 //Set BAUD Rate
#ifndef SPI_ONE_BIT_MODE

    li $2, 0x21
    sw $2, 0x110($3)                //setup FIFO depth = 16, SO pin at pin 1

#if (SPI_CLK_DIV == 1)
    li $2, 0x3001a                  //26 SPIC clock
#elif (SPI_CLK_DIV == 2)     
    li $2, 0x30032                  //50 SPIC clock
#else 
    li $2, 0x3000e                  //14 SPIC clock
#endif
    sw $2, 0x11c($3)                //SPI Address byte = 3, read dummy cycle = 6

    li $2, 0x210
    sw $2, 0x120($3)                //quad address/data, blocking write

#endif /* end of SPI_ONE_BIT_MODE */
    li $2, 0x1
    sw $2, 0x8($3)                  //Enable SPIC

    jr $31
    nop

    .end spic_init
#endif// SPI_FLASH_BOOT
#undef	LANGUAGE_ASSEMBLY

