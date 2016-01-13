#ifndef _PATCH_IODRIVER_H_
#define _PATCH_IODRIVER_H_

#include "rtl_types.h"


//uart patch function pointer
extern VoidPatchFun pPatch_Uart_Hal_UART_Initialize_OPs;
extern VoidPatchFun pPatch_Uart_ReInit;

extern VoidPatchFun pPatch_Pinmux_Config_ROM;
extern VoidPatchFun pPatch_Pad_Config_ROM;
extern VoidPatchFun pPatch_System_WakeUp_Pin_Enable_ROM;

extern VoidPatchFun pPatch_HCI_UART_Pin_Config;
extern VoidPatchFun pPatch_System_WakeUp_For_Hci_Uart;

#endif

