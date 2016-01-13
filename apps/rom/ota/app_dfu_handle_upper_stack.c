enum { __FILE_NUM__ = 0 };

/*
 *  Routines to access hardware
 *
 *  Copyright (c) 2014 Realtek Semiconductor Corp.
 *
 *  This module is a confidential and proprietary property of RealTek and
 *  possession or use of this module requires written permission of RealTek.
 */

#include "app_dfu_int.h"
#include "blueapi_types.h"
#include "blueapi.h"
#include "bterrcod.h"
#include "otp.h"



#include "app_dfu_handle_upper_stack.h"
#include "patch_upperstack.h"


#include <trace_binary.h>
#define TRACE_MODULE_ID     MID_BT_APPL

#if UPPER_STACK_USE_VIRTUAL_HCI
#include "flash_ota.h"
#endif


extern const int iDfuTargetAdvDataSize;
extern const uint8_t dfuTargetAdData[];

extern uint32_t ota_reprot_target_fw_information(uint16_t signature, uint16_t *pOrigFwVersion,uint32_t* pOffset);


bool dfu_Handle_RegisterRsp( TDFU_CB *pDfuCb,
                                    PBlueAPI_RegisterRsp pRegisterRsp )
{
    if ( pRegisterRsp->cause == blueAPI_CauseSuccess )
    {

    }
    else
    {
        //do reset
    }

    return ( true );
}


/*----------------------------------------------------------------------------
 * handle blueAPI_Event_ActInfo
 * --------------------------------------------------------------------------*/

bool dfu_Handle_ActInfo( TDFU_CB * pDfuCb, PBlueAPI_ActInfo pActInfo )
{

    dfu_sm_event(pDfuCb, DFU_EVT_UPPERSTACK_ACTIVE, NULL);
    return ( true );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_DeviceConfigSetRsp
 * --------------------------------------------------------------------------*/

void dfu_Handle_DeviceConfigSetRsp( TDFU_CB * pDfuCb, PBlueAPI_DeviceConfigSetRsp pDevCfgSetRsp )
{

}


bool dfu_Handle_CreateMDLInd( TDFU_CB * pDfuCb,
                                     PBlueAPI_CreateMDLInd pCreateMDLInd )
{

    TBlueAPI_Cause cause = blueAPI_CauseAccept;

    if ( blueAPI_CreateMDLConf( pCreateMDLInd->local_MDL_ID,
                                1,    /* XXXXMJMJ maxTPDUusCredits */
                                cause )
       )
    {

    }
    else
    {
        APPL_TRACE_PRINTF_0(APPL_TRACE_MASK_ERROR,
                            "!!!blueAPI_CreateMDLConf: Fail"
                           );

    }


    return ( true );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_ConnectMDLRsp
 * --------------------------------------------------------------------------*/

bool dfu_Handle_ConnectMDLRsp( TDFU_CB * pDfuCb,
                                      PBlueAPI_ConnectMDLRsp pConnectMDLRsp )
{
    return ( true );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_MCLStatusInfo
 * --------------------------------------------------------------------------*/

bool dfu_Handle_MCLStatusInfo( TDFU_CB * pDfuCb,
                                      PBlueAPI_MCLStatusInfo pMCLStatusInfo )
{

    return ( true );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_ConnectMDLInfo
 * --------------------------------------------------------------------------*/

bool dfu_Handle_ConnectMDLInfo( TDFU_CB * pDfuCb,
                                       PBlueAPI_ConnectMDLInfo pConnectMDLInfo )
{

    pDfuCb->wMTUSize = pConnectMDLInfo->maxTPDUSize;
    pDfuCb->wDsCredits = pConnectMDLInfo->maxTPDUdsCredits;
    pDfuCb->wDsPoolId     = pConnectMDLInfo->dsPoolID;
    pDfuCb->wDsDataOffset = pConnectMDLInfo->dsDataOffset;


    //send event to dfu sm
    dfu_sm_event(pDfuCb, DFU_EVT_LINK_CONNECTED, NULL);

    return ( true );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_DisconnectMDLRsp
 * --------------------------------------------------------------------------*/

bool dfu_Handle_DisconnectMDLRsp( TDFU_CB * pDfuCb,
        PBlueAPI_DisconnectMDLRsp pDisconnectMDLRsp )
{


    return ( true );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_DisconnectMDLInd
 * --------------------------------------------------------------------------*/

bool dfu_Handle_DisconnectMDLInd( TDFU_CB * pDfuCb,
        PBlueAPI_DisconnectMDLInd pDisconnectMDLInd )
{

    if ( blueAPI_DisconnectMDLConf( pDisconnectMDLInd->local_MDL_ID )
       )
    {
        
    }
    return ( true );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_DeleteMDLInfo
 * --------------------------------------------------------------------------*/

bool dfu_Handle_DeleteMDLInfo( TDFU_CB * pDfuCb,
                                      PBlueAPI_DeleteMDLInfo pDeleteMDLInfo )
{


    pDfuCb->local_MDL_ID = 0;
    pDfuCb->local_MDL_ID_Valid = 0;

    //
    dfu_sm_event(pDfuCb, DFU_EVT_LINK_DISCONNECTED, NULL);
    return ( true );
}


/*----------------------------------------------------------------------------
 * handle blueAPI_Event_GATTServiceRegisterRsp
 * --------------------------------------------------------------------------*/

bool dfu_Handle_GATTServiceRegisterRsp(
    TDFU_CB *                       pDfuCb,
    PBlueAPI_GATTServiceRegisterRsp pGATTServiceRegisterRsp )
{

    if ( pGATTServiceRegisterRsp->cause == blueAPI_CauseSuccess )
    {
        pDfuCb->pServiceHandle = pGATTServiceRegisterRsp->serviceHandle;
        dfu_sm_event(pDfuCb, DFU_EVT_DFU_SERVICE_REGISTERED, NULL);
    }
    else
    {

    }

    return ( true );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_GATTAttributeUpdateRsp
 * --------------------------------------------------------------------------*/

bool dfu_Handle_GATTAttributeUpdateRsp(
    TDFU_CB *                       pDfuCb,
    PBlueAPI_GATTAttributeUpdateRsp pGATTAttributeUpdateRsp )
{

    return ( true );
}


/*----------------------------------------------------------------------------
 * handle blueAPI_Event_GATTAttributeUpdateStatusInd
 * --------------------------------------------------------------------------*/

bool dfu_Handle_GATTAttributeUpdateStatusInd(
    TDFU_CB *                             pDfuCb,
    PBlueAPI_GATTAttributeUpdateStatusInd pGATTAttributeUpdateStatusInd )
{
    if ( blueAPI_GATTAttributeUpdateStatusConf(
            pGATTAttributeUpdateStatusInd->serviceHandle,
            pGATTAttributeUpdateStatusInd->requestHandle,
            pGATTAttributeUpdateStatusInd->attribIndex
                                              )
       )
    {
    }

    return ( true );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_GATTAttributeReadInd
 * --------------------------------------------------------------------------*/

bool dfu_Handle_GATTAttributeReadInd(
    TDFU_CB *                     pDfuCb,
    PBlueAPI_GATTAttributeReadInd pGATTAttributeReadInd )
{


    return ( true );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_GATTAttributeWriteInd (triggered by ATT_WRITE_REQUEST)
 * and blueAPI_Event_GATTAttributeWriteCommandIngo (triggered by
 * ATT_WRITE_COMMAND). both have the same parameters.
 * --------------------------------------------------------------------------*/

bool dfu_Handle_GATTAttributeWriteIndCommandInfo(
    TDFU_CB *                      pDfuCb,
    TBlueAPI_Command               Command,
    PBlueAPI_GATTAttributeWriteInd pGATTAttributeWriteInd )
{
    static const char writeIndTxt[]  TRACE_DATA       = "GATTAttributeWriteInd";
    static const char writeCommandInfoTxt[] TRACE_DATA = "GATTAttributeWriteCommandInfo";
    char * pTxt = NULL;
    if ( Command == blueAPI_EventGATTAttributeWriteCommandInfo )
        pTxt = (char *)writeCommandInfoTxt;
    else
        pTxt = (char *)writeIndTxt;


    APPL_TRACE_PRINTF_4( APPL_TRACE_MASK_TRACE,
                         "<-- dfu_Handle_GATTAttributeWriteIndCommandInfo: %s: service=0x%x, idx=%d, length=%d",
                         pTxt,
                         pGATTAttributeWriteInd->serviceHandle,
                         pGATTAttributeWriteInd->attribIndex,
                         pGATTAttributeWriteInd->attribLength
                       );


    if ( Command == blueAPI_EventGATTAttributeWriteInd )
    {
        if ( blueAPI_GATTAttributeWriteConf(pGATTAttributeWriteInd->local_MDL_ID,
                                            pDfuCb->pServiceHandle,
                                            blueAPI_CauseSuccess,
                                            GATT_SUCCESS,
                                            pGATTAttributeWriteInd->attribIndex
                                           )
           )
        {

        }
    }

    dfuServiceAttribPut(pDfuCb,
                                  pGATTAttributeWriteInd->attribIndex,
                                  pGATTAttributeWriteInd->attribLength,
                                  pGATTAttributeWriteInd->data + pGATTAttributeWriteInd->gap
                                 );

    return ( true );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_GATTCCCDInfo
 * --------------------------------------------------------------------------*/

bool dfu_Handle_GATTCCCDInfo( TDFU_CB *             pDfuCb,
                                     PBlueAPI_GATTCCCDInfo pGATTCCCDInfo )
{
    uint16_t   wAttribIndex, wCCCBits;
    uint16_t * pWord;

    if (0 != pGATTCCCDInfo->count)
    {
        pWord = (uint16_t *)&pGATTCCCDInfo->data[pGATTCCCDInfo->gap];

        wAttribIndex = *(pWord++);
        wCCCBits     = *(pWord++);
        APPL_TRACE_PRINTF_2( APPL_TRACE_MASK_TRACE,
                             "--> dfu_Handle_GATTCCCDInfo: wAttribIndex =0x%x, wCCCBits=0x%x",
                             wAttribIndex,
                             wCCCBits
                           );
    }

    dfu_sm_event(pDfuCb, DFU_EVT_NOTIFICATION_ENABLED, NULL);



    return ( true );
}


/*----------------------------------------------------------------------------
 * handle blueAPI_EventGATTSecurityRsp
 * --------------------------------------------------------------------------*/
void dfu_Handle_GATTSecurityRsp(TDFU_CB * pDfuCb,
                                       PBlueAPI_GATTSecurityRsp pSecurityRsp)
{

}

/*----------------------------------------------------------------------------
 * handle blueAPI_EventGATTServerStoreInd
 * --------------------------------------------------------------------------*/
void dfu_Handle_GATTServerStoreInd(TDFU_CB * pDfuCb,
        PBlueAPI_GATTServerStoreInd pStoreInd)
{
    uint16_t        restart   = 0x0000;
    uint8_t         dataLen   = 0;
    uint8_t *       pData     = NULL;
    TBlueAPI_Cause  cause     = blueAPI_CauseReject;


    blueAPI_GATTServerStoreConf((TBlueAPI_GATTStoreOpCode)pStoreInd->opCode,
                                pStoreInd->remote_BD,
                                (TBlueAPI_RemoteBDType)pStoreInd->remote_BD_Type,
                                restart,
                                dataLen,
                                pData,
                                cause
                               );
}


/*----------------------------------------------------------------------------
 * handle blueAPI_EventGATTMtuSizeInfo
 * --------------------------------------------------------------------------*/

void dfu_Handle_GATTMtuSizeInfo(TDFU_CB * pDfuCb,
                                       PBlueAPI_GATTMtuSizeInfo  pMtuSizeInfo)
{

    pDfuCb->wMTUSize = pMtuSizeInfo->mtuSize;
}

/*----------------------------------------------------------------------------
 * handle blueAPI_EventLEAdvertiseRsp
 * --------------------------------------------------------------------------*/
void dfu_Handle_LEAdvertiseRsp(TDFU_CB * pDfuCb,
                                      PBlueAPI_LEAdvertiseRsp pAdvertiseRsp)
{
    dfu_sm_event(pDfuCb, DFU_EVT_ADV_ENABLED, NULL);
}

void dfu_Handle_SetRandomAddressRsp(TDFU_CB * pDfuCb,
                                      PBlueAPI_SetRandomAddressRsp pSetRandomAddressRsp)
{
    if(otp_str_data.gEfuse_UpperStack_s.ota_use_randon_address)
    {
        blueAPI_LEAdvertiseParameterSetReq(blueAPI_LEAdvTypeUndirected,
                                           blueAPI_LEFilterAny,
                                           blueAPI_LEFilterAny,
                                           0x00A0, /* 20ms */
                                           0x00B0, /* 30ms */
                                           blueAPI_LocalBDTypeLERandom,
                                           NULL,
                                           blueAPI_RemoteBDTypeLEPublic
                                          );

    }
 
}



/*----------------------------------------------------------------------------
 * handle blueAPI_Event_LEAdvertiseParameterRsp
 * --------------------------------------------------------------------------*/
void dfu_Handle_LEAdvertiseParameterRsp(TDFU_CB * pDfuCb,
        PBlueAPI_LEAdvertiseParameterSetRsp pAdvertiseParameterSetRsp)
{
    if(otp_str_data.gEfuse_UpperStack_s.ota_adv_with_image_version)
    {
        blueAPI_LEAdvertiseDataSetReq(blueAPI_LEDataTypeAdvertisingData,
                                      iDfuTargetAdvDataSize,
                                      (uint8_t*)dfuTargetAdData);
    }
    else
    {
        uint8_t advData[31] = {0};
        uint16_t KernelImageVersion = 0;
        uint16_t UserlImageVersion = 0;
        uint32_t Offset = 0;
        
        memcpy(advData, dfuTargetAdData, iDfuTargetAdvDataSize);
        ota_reprot_target_fw_information(Signature_KERNEL, &KernelImageVersion, &Offset);
        ota_reprot_target_fw_information(Signature_USER, &UserlImageVersion, &Offset);
        advData[iDfuTargetAdvDataSize] = 0x05;
        advData[iDfuTargetAdvDataSize+1] = 0xFF;
        
        memcpy(&advData[iDfuTargetAdvDataSize+2], &KernelImageVersion, 2);
        memcpy(&advData[iDfuTargetAdvDataSize+4], &UserlImageVersion, 2);
        
        blueAPI_LEAdvertiseDataSetReq(blueAPI_LEDataTypeAdvertisingData,
                                      iDfuTargetAdvDataSize+6,
                                      advData);

    }
 




}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_LEAdvertiseDataSetRsp
 * --------------------------------------------------------------------------*/
void dfu_Handle_LEAdvertiseDataSetRsp(TDFU_CB * pDfuCb,
        PBlueAPI_LEAdvertiseDataSetRsp pAdvertiseDataSetRsp)
{
        blueAPI_LEAdvertiseReq(blueAPI_LEAdvModeEnabled
                              );
}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_LEConnectionUpdateRsp
 * --------------------------------------------------------------------------*/
void dfu_Handle_LEConnectionUpdateRsp(TDFU_CB * pDfuCb,
        PBlueAPI_LEConnectionUpdateRsp pConnectionUpdateRsp)
{

}

/*----------------------------------------------------------------------------
 * handle blueAPI_Event_LEConnectionUpdateInd
 * --------------------------------------------------------------------------*/
void dfu_Handle_LEConnectionUpdateInd(TDFU_CB * pDfuCb,
        PBlueAPI_LEConnectionUpdateInd pConnectionUpdateInd)
{

}

/*----------------------------------------------------------------------------
 * handle dfu_Handle_LEConnectionParameterInfo
 * --------------------------------------------------------------------------*/
void dfu_Handle_LEConnectionParameterInfo(TDFU_CB * pDfuCb,
        PBlueAPI_LEConnectionParameterInfo pConnectionParameterInfo)
{

}


/*----------------------------------------------------------------------------
 * BlueAPI message handler.
 * Returns non-zero value if calling routine may release message buffer.
 * --------------------------------------------------------------------------*/


void dfu_HandleBlueAPIMessage( TDFU_CB * pDfuCb, PBlueAPI_UsMessage pMsg )
{
    bool      ReleaseBuffer = true;

    if(pPatch_upper_stack_dfu_HandleBlueAPIMessage)
    {
				if(pPatch_upper_stack_dfu_HandleBlueAPIMessage(pDfuCb, pMsg))
        {
            return;
        }
    }

    APPL_TRACE_PRINTF_1(APPL_TRACE_MASK_TRACE, 
                                            "dfu_HandleBlueAPIMessage 0x%x",
                                            pMsg->Command                                  
                                            );
    APPL_TRACE_BINARY_UPSTREAM(TRACE_PKT_TYPE_APPL, pMsg->Length,  (uint8_t*)&pMsg);   


    
    switch ( pMsg->Command )
    {
    default:
        APPL_TRACE_PRINTF_1(APPL_TRACE_MASK_ERROR,
                            "<-- dfu_HandleBlueAPIMessage: Unknown Command(%d)",
                            pMsg->Command
                           );

        break;

    case blueAPI_EventRegisterRsp:
        ReleaseBuffer = dfu_Handle_RegisterRsp( pDfuCb, &pMsg->p.RegisterRsp );
        break;

    case blueAPI_EventActInfo:
        ReleaseBuffer = dfu_Handle_ActInfo( pDfuCb, &pMsg->p.ActInfo );
        break;

    case blueAPI_EventDeviceConfigSetRsp:
        dfu_Handle_DeviceConfigSetRsp(pDfuCb, &pMsg->p.DeviceConfigSetRsp);
        break;

    case blueAPI_EventCreateMDLInd:
        ReleaseBuffer = dfu_Handle_CreateMDLInd(
                            pDfuCb, &pMsg->p.CreateMDLInd );
        break;

    case blueAPI_EventConnectMDLRsp:
        ReleaseBuffer = dfu_Handle_ConnectMDLRsp(
                            pDfuCb, &pMsg->p.ConnectMDLRsp );
        break;

    case blueAPI_EventMCLStatusInfo:
        ReleaseBuffer = dfu_Handle_MCLStatusInfo(
                            pDfuCb, &pMsg->p.MCLStatusInfo );
        break;

    case blueAPI_EventConnectMDLInfo:
        ReleaseBuffer = dfu_Handle_ConnectMDLInfo(
                            pDfuCb, &pMsg->p.ConnectMDLInfo );
        break;

    case blueAPI_EventDisconnectMDLRsp:
        ReleaseBuffer = dfu_Handle_DisconnectMDLRsp(
                            pDfuCb, &pMsg->p.DisconnectMDLRsp );
        break;

    case blueAPI_EventDisconnectMDLInd:
        ReleaseBuffer = dfu_Handle_DisconnectMDLInd(
                            pDfuCb, &pMsg->p.DisconnectMDLInd );
        break;

    case blueAPI_EventDeleteMDLInfo:
        ReleaseBuffer = dfu_Handle_DeleteMDLInfo(
                            pDfuCb, &pMsg->p.DeleteMDLInfo );
        break;

    case blueAPI_EventGATTServiceRegisterRsp:
        ReleaseBuffer = dfu_Handle_GATTServiceRegisterRsp(
                            pDfuCb, &pMsg->p.GATTServiceRegisterRsp );
        break;

    case blueAPI_EventGATTAttributeUpdateRsp:
        ReleaseBuffer = dfu_Handle_GATTAttributeUpdateRsp(
                            pDfuCb, &pMsg->p.GATTAttributeUpdateRsp );
        break;

    case blueAPI_EventGATTAttributeUpdateStatusInd:
        ReleaseBuffer = dfu_Handle_GATTAttributeUpdateStatusInd(
                            pDfuCb, &pMsg->p.GATTAttributeUpdateStatusInd );
        break;

    case blueAPI_EventGATTAttributeReadInd:
        ReleaseBuffer = dfu_Handle_GATTAttributeReadInd(
                            pDfuCb, &pMsg->p.GATTAttributeReadInd );
        break;

    case blueAPI_EventGATTAttributeWriteInd:
    case blueAPI_EventGATTAttributeWriteCommandInfo:
        ReleaseBuffer = dfu_Handle_GATTAttributeWriteIndCommandInfo(
                            pDfuCb, (TBlueAPI_Command)pMsg->Command,
                            &pMsg->p.GATTAttributeWriteInd );
        break;

    case blueAPI_EventGATTCCCDInfo:
        ReleaseBuffer = dfu_Handle_GATTCCCDInfo(
                            pDfuCb, &pMsg->p.GATTCCCDInfo );
        break;

    case blueAPI_EventGATTSecurityRsp:
        dfu_Handle_GATTSecurityRsp(pDfuCb, &pMsg->p.GATTSecurityRsp);
        break;

    case blueAPI_EventGATTServerStoreInd:
        dfu_Handle_GATTServerStoreInd(pDfuCb, &pMsg->p.GATTServerStoreInd);
        break;

    case blueAPI_EventGATTMtuSizeInfo:
        dfu_Handle_GATTMtuSizeInfo(pDfuCb, &pMsg->p.GATTMtuSizeInfo);
        break;

    case blueAPI_EventSetRandomAddressRsp:
        dfu_Handle_SetRandomAddressRsp(pDfuCb, &pMsg->p.SetRandomAddressRsp);
        break;

    case blueAPI_EventLEAdvertiseRsp:
        dfu_Handle_LEAdvertiseRsp(pDfuCb, &pMsg->p.LEAdvertiseRsp);
        break;

    case blueAPI_EventLEAdvertiseParameterSetRsp:
        dfu_Handle_LEAdvertiseParameterRsp(pDfuCb, &pMsg->p.LEAdvertiseParameterSetRsp);
        break;

    case blueAPI_EventLEAdvertiseDataSetRsp:
        dfu_Handle_LEAdvertiseDataSetRsp(pDfuCb, &pMsg->p.LEAdvertiseDataSetRsp);
        break;

    case blueAPI_EventLEConnectionUpdateRsp:
        dfu_Handle_LEConnectionUpdateRsp(pDfuCb, &pMsg->p.LEConnectionUpdateRsp);
        break;

    case blueAPI_EventLEConnectionUpdateInd:
        dfu_Handle_LEConnectionUpdateInd(pDfuCb, &pMsg->p.LEConnectionUpdateInd);
        break;

    case blueAPI_EventLEConnectionParameterInfo:
        dfu_Handle_LEConnectionParameterInfo(pDfuCb, &pMsg->p.LEConnectionParameterInfo);
        break;

    }

    if (ReleaseBuffer )
        blueAPI_BufferRelease(pMsg);

}


