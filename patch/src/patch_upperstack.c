#include "rtl_types.h"
#include "section_config.h"


/**
    patch function pointers defined here
*/

VoidPatchFun pPatch_UpperStack_init PATCH_POINTER_SECTION;
VoidPatchFun pPatch_UpperStack_reinit PATCH_POINTER_SECTION;

//patch function pointer for blueAPI
BOOLPatchFun pPatch_upper_stack_blueAPI_PutMessage PATCH_POINTER_SECTION;

//patch function pointer for OS
U32PatchFun pPatch_upper_stack_osMessageSend PATCH_POINTER_SECTION;
U32PatchFun pPatch_upper_stack_osMessageReceive PATCH_POINTER_SECTION;
U32PatchFun pPatch_upper_stack_osBufferGet PATCH_POINTER_SECTION;
U32PatchFun pPatch_upper_stack_osBufferRelease PATCH_POINTER_SECTION;


//add more here
BOOLPatchFun pPatch_upper_stack_blueAPI_UserDefined PATCH_POINTER_SECTION;

U32PatchFun pPatch_upper_stack_blueFacePutMessage PATCH_POINTER_SECTION;
U32PatchFun pPatch_upper_stack_blueFacePutMessage PATCH_POINTER_SECTION;
BOOLPatchFun pPatch_upper_stack_blueAPI_AppPoolInit PATCH_POINTER_SECTION;
U16PatchFun  pPatch_upper_stack_blueAPI_AppPoolCreate PATCH_POINTER_SECTION;
BOOLPatchFun pPatch_upper_stack_blueAPI_AppPoolExtend PATCH_POINTER_SECTION;
PVOIDPatchFun pPatch_upper_stack_blueAPI_AppBufferAlloc PATCH_POINTER_SECTION;
VoidPatchFun   pPatch_upper_stack_blueAPI_AppBufferRelease PATCH_POINTER_SECTION;



BOOLPatchFun  pPatch_upper_stack_blueAPI_HciInterfaceInit PATCH_POINTER_SECTION;
BOOLPatchFun  pPatch_upper_stack_blueAPI_SendHciComamand PATCH_POINTER_SECTION;

PVOIDPatchFun pPatch_upper_stack_blueAPI_CmdBufferGet PATCH_POINTER_SECTION;
VoidPatchFun  pPatch_upper_stack_blueAPI_CmdBufferRelease PATCH_POINTER_SECTION;
VoidPatchFun  pPatch_upper_stack_blueAPI_EvtBufferRelease PATCH_POINTER_SECTION;

BOOLPatchFun pPatch_upper_stack_dfu_HandleBlueAPIMessage PATCH_POINTER_SECTION;

BOOLPatchFun pPatch_upper_stack_blueAPI_BTCallBack PATCH_POINTER_SECTION;
BOOLPatchFun pPatch_upper_stack_blueAPI_Handle_Command PATCH_POINTER_SECTION;
BOOLPatchFun pPatch_upper_stack_blueFaceEntry_Handle_Command PATCH_POINTER_SECTION;
BOOLPatchFun pPatch_upper_stack_blueFaceHandleGATTDownSideMessage PATCH_POINTER_SECTION;
BOOLPatchFun pPatch_upper_stack_blueFaceHandleGATTUpSideMessage PATCH_POINTER_SECTION;
BOOLPatchFun pPatch_upper_stack_blueFaceHandlePhDataReq PATCH_POINTER_SECTION;
BOOLPatchFun pPatch_upper_stack_gattHandleMessage PATCH_POINTER_SECTION;
BOOLPatchFun pPatch_upper_stack_gattHandleLE_MESSAGE_CONF PATCH_POINTER_SECTION;
BOOLPatchFun pPatch_upper_stack_gattHandleLE_MESSAGE_IND PATCH_POINTER_SECTION;
BOOLPatchFun pPatch_upper_stack_l2cEntry PATCH_POINTER_SECTION;
BOOLPatchFun pPatch_upper_stack_hciHandleMessage PATCH_POINTER_SECTION;
BOOLPatchFun pPatch_upper_stack_hciLEProcessEventPacket PATCH_POINTER_SECTION;
BOOLPatchFun pPatch_upper_stack_hciProcessEventPacket PATCH_POINTER_SECTION;
BOOLPatchFun pPatch_upper_stack_btsmHandleMessage PATCH_POINTER_SECTION;
BOOLPatchFun pPatch_upper_stack_btsmHandleLE_MESSAGE_REQ PATCH_POINTER_SECTION;
BOOLPatchFun pPatch_upper_stack_btsmHandleLE_MESSAGE_CONF PATCH_POINTER_SECTION;
BOOLPatchFun pPatch_upper_stack_btsmHandleLE_MESSAGE_IND PATCH_POINTER_SECTION;
BOOLPatchFun pPatch_upper_stack_l2cHandleSigCmd PATCH_POINTER_SECTION;
VoidPatchFun pPatch_upper_stack_l2cFragmentLEData PATCH_POINTER_SECTION;
BOOLPatchFun pPatch_upper_stack_attDataReceived PATCH_POINTER_SECTION;
VoidPatchFun pPatch_upper_stack_l2cFragmentDATA_REQ PATCH_POINTER_SECTION;
VoidPatchFun pPatch_upper_stack_l2cChanRelease PATCH_POINTER_SECTION;;
VoidPatchFun pPatch_upper_stack_l2cSendL2CAPMessage PATCH_POINTER_SECTION;
BOOLPatchFun pPatch_upper_stack_l2cChangeState PATCH_POINTER_SECTION;
BOOLPatchFun pPatch_upper_stack_hciLLApiHandleDataIndication PATCH_POINTER_SECTION;
VoidPatchFun pPatch_upper_stack_l2cHandleUpstreamLEFrame PATCH_POINTER_SECTION;
BOOLPatchFun pPatch_upper_stack_blueAPI_TgtSendEvent PATCH_POINTER_SECTION;

BOOLPatchFun  pPatch_upper_stack_dlps_enter PATCH_POINTER_SECTION;
BOOLPatchFun  pPatch_upper_stack_dlps_exit PATCH_POINTER_SECTION;
BOOLPatchFun  pPatch_upper_stack_dlps_check PATCH_POINTER_SECTION;


/**@brief ota */
VoidPatchFun pPatch_dfuInit PATCH_POINTER_SECTION;
BOOLPatchFun pPatch_dfu_timer_init PATCH_POINTER_SECTION;


