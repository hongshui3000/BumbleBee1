enum { __FILE_NUM__ = 0 };

/*
 *  Routines to access hardware
 *
 *  Copyright (c) 2014 Realtek Semiconductor Corp.
 *
 *  This module is a confidential and proprietary property of RealTek and
 *  possession or use of this module requires written permission of RealTek.
 */

#include "app_dfu_act.h"
#include "app_dfu_int.h"

#include "blueapi.h"
#include "gatt.h"
#include "flags.h"
#include "otp.h"


#if UPPER_STACK_USE_VIRTUAL_HCI
#include "flash_ota.h"
#endif

#include <trace_binary.h>
#define TRACE_MODULE_ID     MID_BT_APPL

extern const int iDfuServiceSize;
extern const TAttribAppl DFU_Service[];

extern uint16_t ota_get_bank_size(uint16_t signature);
extern uint32_t ota_reprot_target_fw_information(uint16_t signature, uint16_t *pOrigFwVersion,uint32_t* pOffset);
extern uint32_t ota_update(uint16_t signature, uint32_t offset, uint32_t length, void *pVoid);
extern uint32_t ota_reset(uint16_t signature);
extern void ota_fw_active_reset(void);
extern uint32_t ota_check_crc(uint16_t signature);


void dfu_act_process_evt_init_upperstack(TDFU_CB *pDfuCb,
        TDFU_INIT_DATA *pDfuData)
{
    //register upper stack
    if (
        blueAPI_RegisterReq( (void *)pDfuCb,
                             (void *)dfu_BlueAPICallback)
    )
    {

    }
    else
    {
    }
}

void dfu_act_process_evt_upperstack_active(TDFU_CB *pDfuCb,
        TDFU_INIT_DATA *pDfuData)
{
    //register dfu service

    if (blueAPI_GATTServiceRegisterReq(
                iDfuServiceSize / sizeof(TAttribAppl), /* nbrOfAttrib */
                (uint8_t *)&DFU_Service                /* pService  */
            ))
    {

    }
    else
    {
    }



}

void dfu_act_process_evt_dfu_service_registered(TDFU_CB *pDfuCb,
        TDFU_INIT_DATA *pDfuData)
{
    //set advertising parameters
    if(otp_str_data.gEfuse_UpperStack_s.ota_use_randon_address)
    {
        blueAPI_SetRandomAddressReq(otp_str_data.gEfuse_UpperStack_s.ota_adv_random_address);
    }
    else
    {
        blueAPI_LEAdvertiseParameterSetReq(blueAPI_LEAdvTypeUndirected,
                                           blueAPI_LEFilterAny,
                                           blueAPI_LEFilterAny,
                                           0x00A0, /* 20ms */
                                           0x00B0, /* 30ms */
                                           blueAPI_LocalBDTypeLEPublic,
                                           NULL,
                                           blueAPI_RemoteBDTypeLEPublic
                                          );

    }


}
void dfu_act_process_evt_adv_enabled(TDFU_CB *pDfuCb,
                                     TDFU_INIT_DATA *pDfuData)
{
    //wait for connection


}

void dfu_act_process_evt_link_connected(TDFU_CB *pDfuCb,
                                        TDFU_INIT_DATA *pDfuData)
{
    APPL_TRACE_PRINTF_0(APPL_TRACE_MASK_TRACE, "dfu_act_process_evt_link_connected");
    
    xTimerStop(pDfuCb->dfu_wait4_connTimerHandle, 0);
    
    xTimerStart(pDfuCb->dfu_image_transferTimerHandle, 0);

#if(UPPERSTACK_OTA_SUPPORT)
    pDfuCb->nMaxKernelImageSizeSupport = ota_get_bank_size(Signature_KERNEL) << 2; //N*4
    pDfuCb->nMaxUserImageSizeSupport = ota_get_bank_size(Signature_USER) << 2; //N*4

#endif
}

void dfu_act_process_evt_enable_notification(TDFU_CB *pDfuCb,
        TDFU_INIT_DATA *pDfuData)
{

    APPL_TRACE_PRINTF_0(APPL_TRACE_MASK_TRACE, "dfu_act_process_evt_enable_notification");
}

void dfu_act_notify_rx_cp_req(TDFU_CB *pDfuCb,        TDFU_INIT_DATA *pDfuData)
{
    if (pDfuCb->nCurNotificationAct == DFU_SUBACT_RX_FW_CP_REQ_REPORT_TARGET_INFO)
    {

#if(UPPERSTACK_OTA_SUPPORT)
        if(pDfuCb->nSignature == Signature_KERNEL)
        {
            ota_reprot_target_fw_information(pDfuCb->nSignature, &pDfuCb->nOrigKernelImageVersion, (uint32_t*)&pDfuCb->nCurOffSet);
        }
        else
        {
            ota_reprot_target_fw_information(pDfuCb->nSignature, &pDfuCb->nOrigUserImageVersion, (uint32_t*)&pDfuCb->nCurOffSet);
        }
#endif

        dfu_act_notify_report_target_image_information(pDfuCb, pDfuData);
    }
    else if (pDfuCb->nCurNotificationAct == DFU_SUBACT_RX_FW_CP_REQ_START_DFU)
    {
#if(UPPERSTACK_OTA_SUPPORT)

        if (ota_update(pDfuCb->nSignature, 0, pDfuCb->nOffSet*4, (uint8_t*)&pDfuCb->nOffSet) == 0)
        {
            pDfuCb->nCurOffSet += pDfuCb->nOffSet*4;
        }
        else
        {
            ota_reset(pDfuCb->nSignature);
            ota_fw_active_reset();
            
        }
#else
        pDfuCb->nCurOffSet += pDfuCb->nOffSet*4;
#endif

        dfu_act_notify_start_dfu(pDfuCb, pDfuData);
    }
}

void dfu_act_notify_report_target_image_information(TDFU_CB *pDfuCb,
        TDFU_INIT_DATA *pDfuData)
{

    uint8_t dfuNotification[DFU_NOTIFY_LENGTH_REPORT_TARGET_INFO] = {0};
    uint8_t *pDfuNotification = dfuNotification;
    
    APPL_TRACE_PRINTF_2(APPL_TRACE_MASK_TRACE, 
        "dfu_act_notify_report_target_image_information, nOrigKernelImageVersion(0x%x), nCurOffSet = %d",
        pDfuCb->nOrigKernelImageVersion,
        pDfuCb->nCurOffSet
        );
    pDfuNotification[0] = DFU_OPCODE_NOTIFICATION;
    pDfuNotification[1] = DFU_OPCODE_REPORT_TARGET_INFO;
    pDfuNotification[2] = DFU_ARV_SUCCESS;
    
    if(pDfuCb->nSignature == Signature_KERNEL)
    {
        LE_WORD2EXTRN(&pDfuNotification[3], pDfuCb->nOrigKernelImageVersion);
    }
    else if(pDfuCb->nSignature == Signature_USER)
    {
        LE_WORD2EXTRN(&pDfuNotification[3], pDfuCb->nOrigUserImageVersion);
    }
    
    LE_DWORD2EXTRN(&pDfuNotification[5], pDfuCb->nCurOffSet);

    dfu_act_send_notification(pDfuCb, (uint8_t*)&dfuNotification, DFU_NOTIFY_LENGTH_REPORT_TARGET_INFO);


}
void dfu_act_notify_start_dfu(TDFU_CB *pDfuCb,
                              TDFU_INIT_DATA *pDfuData)
{
    TDFUNotification dfuNotification;
    APPL_TRACE_PRINTF_0(APPL_TRACE_MASK_TRACE, "dfu_act_notify_start_dfu");

    dfuNotification.opCode = DFU_OPCODE_NOTIFICATION;
    dfuNotification.reqOpCode = DFU_OPCODE_START_DFU;
    dfuNotification.respValue = DFU_ARV_SUCCESS;

    dfu_act_send_notification(pDfuCb, (uint8_t*)&dfuNotification, DFU_NOTIFY_LENGTH_ARV);
}

void dfu_act_notify_fw_rx_cmpl(TDFU_CB *pDfuCb,
                               TDFU_INIT_DATA *pDfuData)
{
    TDFUNotification dfuNotification;
    APPL_TRACE_PRINTF_0(APPL_TRACE_MASK_TRACE, "dfu_act_notify_fw_rx_cmpl");
    dfuNotification.opCode = DFU_OPCODE_NOTIFICATION;
    dfuNotification.reqOpCode = DFU_OPCODE_RECEIVE_FW_IMAGE_INFO;
    dfuNotification.respValue = DFU_ARV_SUCCESS;
    dfu_act_send_notification(pDfuCb, (uint8_t*)&dfuNotification, DFU_NOTIFY_LENGTH_ARV);


}
void dfu_act_notify_valid(TDFU_CB *pDfuCb,
                          TDFU_INIT_DATA *pDfuData)

{
    TDFUNotification dfuNotification;
    uint8_t nCrcResult = 0;
    APPL_TRACE_PRINTF_0(APPL_TRACE_MASK_TRACE, "dfu_act_notify_valid");
    DBG_DIRECT("dfu_act_notify_valid");
#if(UPPERSTACK_OTA_SUPPORT)
    nCrcResult = ota_check_crc(pDfuCb->nSignature);
#endif

    if (nCrcResult == 0)
    {
        dfuNotification.respValue = DFU_ARV_SUCCESS;
    }
    else
    {
        APPL_TRACE_PRINTF_0(APPL_TRACE_MASK_ERROR, "dfu_act_notify_valid, CRC error");
        dfuNotification.respValue = DFU_ARV_FAIL_CRC_ERROR;
    }

    dfuNotification.opCode = DFU_OPCODE_NOTIFICATION;
    dfuNotification.reqOpCode = DFU_OPCODE_VALID_FW;

    dfu_act_send_notification(pDfuCb, (uint8_t*)&dfuNotification, DFU_NOTIFY_LENGTH_ARV);
}



void dfu_act_reset_and_activate(TDFU_CB *pDfuCb,
                                TDFU_INIT_DATA *pDfuData)

{
     
    APPL_TRACE_PRINTF_3(APPL_TRACE_MASK_ERROR, "dfu_act_reset_and_activate: nCurOffSet =%d, nImageTotalLength= %d, ec(%d)",
                        pDfuCb->nCurOffSet,
                        pDfuCb->nImageTotalLength,
                        pDfuCb->nTotalErrorCount
                        );
#if 0    
    DBG_DIRECT("dfu_act_reset_and_activate: nCurOffSet =%d, nImageTotalLength= %d, ec(%d)",
                        pDfuCb->nCurOffSet,
                        pDfuCb->nImageTotalLength,
                        pDfuCb->nTotalErrorCount
                        );
#endif

#if(UPPERSTACK_OTA_SUPPORT)
    ota_fw_active_reset();
#endif
}


void dfu_act_system_reset(TDFU_CB *pDfuCb,
                          TDFU_INIT_DATA *pDfuData)

{
    APPL_TRACE_PRINTF_0(APPL_TRACE_MASK_TRACE, "dfu_act_reset_and_activate");
}

void dfu_act_send_notification(TDFU_CB *pDfuCb,
                               uint8_t* pData,
                               int dataLen)

{

    uint8_t     wLength = dataLen;
    if ( dataLen != 0 && pData != NULL)
    {

        uint8_t   * pBuffer = NULL;
        uint16_t    wOffset = pDfuCb->wDsDataOffset + 3;

        if ( pData == NULL )
        {
            /* value is directly accessible by BT stack */
            wLength = 0;
        }
        else
        {
            /* copy attribute value to buffer position that allows re-usage by stack */
            /* without copying ..                                                   */
            if ( blueAPI_BufferGet(
                        pDfuCb->wDsPoolId,
                        wLength,
                        wOffset,
                        (void **)&pBuffer) == blueAPI_CauseSuccess )
            {
                memcpy( pBuffer + wOffset, pData, wLength );
            }
            else
            {

                return ;
            }
        }


        if ( blueAPI_GATTAttributeUpdateReq(pBuffer,
                                            pDfuCb->pServiceHandle,
                                            pDfuCb->pRequestHandle,
                                            INDEX_DFU_CONTROL_POINT_CHAR_VALUE,
                                            wLength,
                                            wOffset
                                           )
           )
        {



        }
        else
        {
            if ( pBuffer != NULL )
                blueAPI_BufferRelease(pBuffer);
        }
    }
}






