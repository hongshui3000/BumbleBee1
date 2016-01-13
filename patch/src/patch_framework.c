#include "rtl_types.h"
#include "section_config.h"

/*
Patch Rules:

a) Function pointer prototype: refer to rtl_types.h
e.g. VoidPatchFun pPatch_SystemInit = NULL;

b) Patch function pointer Naming: pPatch_XXX_YYY or pPatch_YYY
   XXX: Module Name; YYY: ROM function name.
e.g. pPatch_SystemInit

c) Patch Function section: PATCH_FLASH_TEXT_SECTION
e.g.
  PATCH_FLASH_TEXT_SECTION
  VOID PatchPointerInit_Framework(VOID){}

d) Patch RO Data: PATCH_FLASH_RODATA_SECTION
e.g. PATCH_FLASH_RODATA_SECTION const char gPatchSampleTable[32] ={0,1,2,3};

e) Patch RW BSS (zero-initialize): use one of the following sections:
	<1> SRAM_OFF_BD_BSS_SECTION
	<2> SRAM_ON_BD_BSS_SECTION
	<3> SRAM_OFF_BF_BSS_SECTION
	<4> SRAM_ON_BF_BSS_SECTION
e.g.
  SRAM_OFF_BF_BSS_SECTION
  UINT32 MAX_Duty_10Mhz_Value = 200;

f) Patch RW Data (Nonzero-initialize): NOT Allowed!!!
   Use Patch RW BSS instead, and initialize it in one of the following functions:
     <2> PatchDataInit_Framework();
     <3> PatchDataInit_IODriver();
     <4> PatchDataInit_LowerStack();
     <5> PatchDataInit_UpperStack();

g) All patch funtion pointers and patch code Should be put in the following files:
     patch_application.c, patch_framework.c, patch_iodriver.c,  patch_lowerstack.c ,  patch_upperstack.c

h) Patch function pointer initialize: in one of the following functions:
      <2> PatchPointerInit_Framework();
      <3> PatchPointerInit_IODriver();
      <4> PatchPointerInit_LowerStack();
      <5> PatchPointerInit_UpperStack();
e.g.
	VOID PatchPointerInit_Framework(VOID)
	{
		pPatch_SystemInit = NULL;
		pPatch_UserIrqFunTableInit = Patch_UserIrqFunTableInit;
	}

*/

BOOLPatchFun pPatch_xtal_40mhz_calibration PATCH_POINTER_SECTION;
VoidPatchFun pPatch_use_rcosc PATCH_POINTER_SECTION;
VoidPatchFun pPatch_rcosc_cal PATCH_POINTER_SECTION;
VoidPatchFun pPatch_set_xtal_then_pll PATCH_POINTER_SECTION;
VoidPatchFun pPatch_late_init PATCH_POINTER_SECTION;
BOOLPatchFun pPatch_set_pad_ctrl_by_pinmux PATCH_POINTER_SECTION;
VoidPatchFun pPatch_LOP_setting PATCH_POINTER_SECTION;
VoidPatchFun pPatch_SystemInit PATCH_POINTER_SECTION;
VoidPatchFun pPatch_UserIrqFunTableInit PATCH_POINTER_SECTION;
VoidPatchFun pPatch_HalInitPlatformLogUart PATCH_POINTER_SECTION;
VoidPatchFun pPatch_main PATCH_POINTER_SECTION;
VoidPatchFun pPatch_prvHeapInit PATCH_POINTER_SECTION;

VoidPatchFun pPatch_vTaskStartScheduler PATCH_POINTER_SECTION;
VoidPatchFun pPatch_prvIdleTask PATCH_POINTER_SECTION;
VoidPatchFun pPatch_vApplicationIdleHook PATCH_POINTER_SECTION;
VoidPatchFun pPatch_vTaskSwitchContext PATCH_POINTER_SECTION;
VoidPatchFun pPatch_vListInsert PATCH_POINTER_SECTION;
VoidPatchFun pPatch_uxListRemove PATCH_POINTER_SECTION;

BOOLPatchFun pPatch_CycQueueWrite PATCH_POINTER_SECTION;
U16PatchFun pPatch_CycQueueReadandPrint PATCH_POINTER_SECTION;

U32PatchFun  pPatch_DLPS_CheckNextTimeout PATCH_POINTER_SECTION;
BOOLPatchFun pPatch_DLPS_ENTER_CHECK_CB_REG PATCH_POINTER_SECTION;
BOOLPatchFun pPatch_DLPS_ENTER_CHECK PATCH_POINTER_SECTION;
BOOLPatchFun pPatch_DLPS_ENTER_QUERY_CB_REG PATCH_POINTER_SECTION;
BOOLPatchFun pPatch_DLPS_ENTER_QUERY PATCH_POINTER_SECTION;
BOOLPatchFun pPatch_DLPS_INTERRUPT_CONTROL_CB_REG PATCH_POINTER_SECTION;
VoidPatchFun pPatch_DLPS_INTERRUPT_CONTROL_CB_UNREG PATCH_POINTER_SECTION;
BOOLPatchFun pPatch_DLPS_INTERRUPT_CONTROL PATCH_POINTER_SECTION;
BOOLPatchFun pPatch_DLPS_STACK_REG PATCH_POINTER_SECTION;
BOOLPatchFun pPatch_DLPS_STACK_STORE PATCH_POINTER_SECTION;
VoidPatchFun pPatch_DLPS_STACK_STORE_ROLLBACK PATCH_POINTER_SECTION;
VoidPatchFun pPatch_DLPS_STACK_RESTORE PATCH_POINTER_SECTION;
BOOLPatchFun pPatch_DLPS_BUFFER_REG PATCH_POINTER_SECTION;
BOOLPatchFun pPatch_DLPS_BUFFER_UNREG PATCH_POINTER_SECTION;
VoidPatchFun pPatch_DLPS_BUFFER_STORE_ROLLBACK PATCH_POINTER_SECTION;
BOOLPatchFun pPatch_DLPS_BUFFER_STORE PATCH_POINTER_SECTION;
VoidPatchFun pPatch_DLPS_BUFFER_RESTORE PATCH_POINTER_SECTION;
VoidPatchFun pPatch_DLPS_RESTORE_SPECIAL PATCH_POINTER_SECTION;
VoidPatchFun pPatch_LPS_MODE_ForceEnter PATCH_POINTER_SECTION;
BOOLPatchFun pPatch_DLPS_QUEUEHANDLE_REG PATCH_POINTER_SECTION;
VoidPatchFun pPatch_DLPS_QUEUEHANDLE_UNREG PATCH_POINTER_SECTION;
BOOLPatchFun pPatch_DLPS_QUEUEHANDLE_CHECK PATCH_POINTER_SECTION;


VoidPatchFun pPatch_LOG_RAW PATCH_POINTER_SECTION;
VoidPatchFun pPatch_LOG_BUFFER PATCH_POINTER_SECTION;
VoidPatchFun pPatch_LOG_DATA PATCH_POINTER_SECTION;
VoidPatchFun pPatch_prvSetupHardware PATCH_POINTER_SECTION;
VoidPatchFun pPatch_fw_sim PATCH_POINTER_SECTION;

VoidPatchFun pPatch_RECORD_ADD PATCH_POINTER_SECTION;
VoidPatchFun pPatch_RECORD_DUMP PATCH_POINTER_SECTION;

VoidPatchFun pPatch_vApplicationStackOverflowHook PATCH_POINTER_SECTION;
BOOLPatchFun pPatch_HalHardFaultHandler PATCH_POINTER_SECTION;

// [eflash]
VoidPatchFun pPatch_Flash_Set_TimingParameters PATCH_POINTER_SECTION;
U32PatchFun  pPatch_FTL_init PATCH_POINTER_SECTION;
U32PatchFun  pPatch_FTL_read PATCH_POINTER_SECTION;
U32PatchFun  pPatch_FTL_write PATCH_POINTER_SECTION;
U32PatchFun  pPatch_FTL_get_error PATCH_POINTER_SECTION;
U32PatchFun  pPatch_FTL_ioctl PATCH_POINTER_SECTION;
U32PatchFun  pPatch_FMC_Init PATCH_POINTER_SECTION;
U32PatchFun  pPatch_FMC_Init_from_DLPS PATCH_POINTER_SECTION;
U32PatchFun  pPatch_FMC_Read PATCH_POINTER_SECTION;
U32PatchFun  pPatch_FMC_Write PATCH_POINTER_SECTION;
U32PatchFun  pPatch_FMC_Erase_Page PATCH_POINTER_SECTION;
U32PatchFun  pPatch_FMC_Erase_Mass PATCH_POINTER_SECTION;
U32PatchFun  pPatch_FMC_ioctl PATCH_POINTER_SECTION;

U32PatchFun  pPatch_ota_reprot_target_fw_information PATCH_POINTER_SECTION;
U32PatchFun  pPatch_ota_update PATCH_POINTER_SECTION;
U32PatchFun  pPatch_ota_check_crc PATCH_POINTER_SECTION;
U16PatchFun  pPatch_ota_get_bank_size PATCH_POINTER_SECTION;
VoidPatchFun pPatch_ota_fw_active_reset PATCH_POINTER_SECTION;
U32PatchFun  pPatch_ota_reset PATCH_POINTER_SECTION;


