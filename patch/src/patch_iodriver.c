#include "rtl_types.h"
#include "section_config.h"

/**@brief patch function pointer for uart */
VoidPatchFun pPatch_Uart_Hal_UART_Initialize_OPs PATCH_POINTER_SECTION;
VoidPatchFun pPatch_Uart_ReInit PATCH_POINTER_SECTION;

VoidPatchFun pPatch_Pinmux_Config_ROM PATCH_POINTER_SECTION;
VoidPatchFun pPatch_Pad_Config_ROM PATCH_POINTER_SECTION;
VoidPatchFun pPatch_System_WakeUp_Pin_Enable_ROM PATCH_POINTER_SECTION;

VoidPatchFun pPatch_HCI_UART_Pin_Config PATCH_POINTER_SECTION;
VoidPatchFun pPatch_System_WakeUp_For_Hci_Uart PATCH_POINTER_SECTION;

