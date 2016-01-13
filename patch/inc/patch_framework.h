#ifndef _PATCH_FRAMEWORK_H_
#define _PATCH_FRAMEWORK_H_

#include "rtl_types.h"
#include "section_config.h"

extern BOOLPatchFun pPatch_xtal_40mhz_calibration;
extern VoidPatchFun pPatch_use_rcosc;
extern VoidPatchFun pPatch_rcosc_cal;
extern VoidPatchFun pPatch_set_xtal_then_pll;
extern VoidPatchFun pPatch_late_init;
extern BOOLPatchFun pPatch_set_pad_ctrl_by_pinmux;
extern VoidPatchFun pPatch_LOP_setting;
extern VoidPatchFun pPatch_SystemInit;
//extern VoidPatchFun pPatch_HalCpuClkConfig;
//extern U32PatchFun pPatch_HalGetCpuClk;
extern VoidPatchFun pPatch_UserIrqFunTableInit;
extern VoidPatchFun pPatch_HalInitPlatformLogUart;
extern VoidPatchFun pPatch_main;
extern VoidPatchFun pPatch_prvHeapInit;

extern VoidPatchFun pPatch_vTaskStartScheduler;
extern VoidPatchFun pPatch_prvIdleTask;
extern VoidPatchFun pPatch_vApplicationIdleHook;
extern VoidPatchFun pPatch_vTaskSwitchContext;
extern VoidPatchFun pPatch_vListInsert;
extern U32PatchFun pPatch_uxListRemove;

extern BOOLPatchFun pPatch_CycQueueWrite;
extern U16PatchFun pPatch_CycQueueReadandPrint;

extern U32PatchFun  pPatch_DLPS_CheckNextTimeout;
extern BOOLPatchFun pPatch_DLPS_ENTER_CHECK_CB_REG;
extern BOOLPatchFun pPatch_DLPS_ENTER_CHECK;
extern BOOLPatchFun pPatch_DLPS_ENTER_QUERY_CB_REG;
extern BOOLPatchFun pPatch_DLPS_ENTER_QUERY;
extern BOOLPatchFun pPatch_DLPS_INTERRUPT_CONTROL_CB_REG;
extern VoidPatchFun pPatch_DLPS_INTERRUPT_CONTROL_CB_UNREG;
extern BOOLPatchFun pPatch_DLPS_INTERRUPT_CONTROL;
extern BOOLPatchFun pPatch_DLPS_STACK_REG;
extern BOOLPatchFun pPatch_DLPS_STACK_STORE;
extern VoidPatchFun pPatch_DLPS_STACK_STORE_ROLLBACK;
extern VoidPatchFun pPatch_DLPS_STACK_RESTORE;
extern BOOLPatchFun pPatch_DLPS_BUFFER_REG;
extern BOOLPatchFun pPatch_DLPS_BUFFER_UNREG;
extern VoidPatchFun pPatch_DLPS_BUFFER_STORE_ROLLBACK;
extern BOOLPatchFun pPatch_DLPS_BUFFER_STORE;
extern VoidPatchFun pPatch_DLPS_BUFFER_RESTORE;
extern VoidPatchFun pPatch_DLPS_RESTORE_SPECIAL;
extern VoidPatchFun pPatch_LPS_MODE_ForceEnter;
extern BOOLPatchFun pPatch_DLPS_QUEUEHANDLE_REG;
extern VoidPatchFun pPatch_DLPS_QUEUEHANDLE_UNREG;
extern BOOLPatchFun pPatch_DLPS_QUEUEHANDLE_CHECK;

extern VoidPatchFun pPatch_LOG_RAW;
extern VoidPatchFun pPatch_LOG_BUFFER;
extern VoidPatchFun pPatch_LOG_DATA;

extern VoidPatchFun pPatch_prvSetupHardware;
extern VoidPatchFun pPatch_fw_sim;

extern VoidPatchFun pPatch_RECORD_ADD;
extern VoidPatchFun pPatch_RECORD_DUMP;

extern VoidPatchFun pPatch_vApplicationStackOverflowHook;
extern BOOLPatchFun pPatch_HalHardFaultHandler;


// eflash
extern VoidPatchFun pPatch_Flash_Set_TimingParameters;
extern U32PatchFun  pPatch_FTL_init;
extern U32PatchFun  pPatch_FTL_read;
extern U32PatchFun  pPatch_FTL_write;
extern U32PatchFun  pPatch_FTL_get_error;
extern U32PatchFun  pPatch_FTL_ioctl;

extern U32PatchFun  pPatch_FMC_Init;
extern U32PatchFun  pPatch_FMC_Init_from_DLPS;
extern U32PatchFun  pPatch_FMC_Read;
extern U32PatchFun  pPatch_FMC_Write;
extern U32PatchFun  pPatch_FMC_Erase_Page;
extern U32PatchFun  pPatch_FMC_Erase_Mass;
extern U32PatchFun  pPatch_FMC_ioctl;

extern U32PatchFun  pPatch_ota_reprot_target_fw_information;
extern U32PatchFun  pPatch_ota_update;
extern U32PatchFun  pPatch_ota_check_crc;
extern U16PatchFun  pPatch_ota_get_bank_size;
extern VoidPatchFun pPatch_ota_fw_active_reset;
extern U32PatchFun  pPatch_ota_reset;

#endif

