enum { __FILE_NUM__ = 0 };

/*
 *  Routines to access hardware
 *
 *  Copyright (c) 2014 Realtek Semiconductor Corp.
 *
 *  This module is a confidential and proprietary property of RealTek and
 *  possession or use of this module requires written permission of RealTek.
 */

#include "app_dfu_api.h"
#include "app_dfu_int.h"
#include "dfu_service.h"
#include "app_dfu_act.h"

#include "gatt.h"
#include "bterrcod.h"
#include "flags.h"
#include "aes.h"

#if UPPER_STACK_USE_VIRTUAL_HCI
#include "flash_ota.h"
#endif



#include <trace_binary.h>
#define TRACE_MODULE_ID     MID_BT_APPL

extern uint32_t ota_update(uint16_t signature, uint32_t offset, uint32_t length, void *pVoid);
extern uint32_t ota_reset(uint16_t signature);
extern void ota_fw_active_reset(void);


const TAttribAppl DFU_Service[] =
{

    /*-------------------------- DFU Service ---------------------------*/
    /* <<Primary Service>>, .. */
    {
        (ATTRIB_FLAG_VALUE_INCL | ATTRIB_FLAG_LE),  /* wFlags     */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_PRIMARY_SERVICE),
            HI_WORD(GATT_UUID_PRIMARY_SERVICE),
            LO_WORD(GATT_UUID_DFU_SERVICE),               /* service UUID */
            HI_WORD(GATT_UUID_DFU_SERVICE)
        },
        UUID_16BIT_SIZE,                            /* bValueLen     */
        NULL,                                       /* pValueContext */
        GATT_PERM_READ                              /* wPermissions  */
    },



    /* <<Characteristic>>, .. */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_WRITE_NO_RSP/* characteristic properties */
            /* characteristic UUID not needed here, is UUID of next attrib. */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },
    /*--- DFU packet characteristic value ---*/
    {
        ATTRIB_FLAG_VALUE_APPL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_DFU_PACKET),
            HI_WORD(GATT_UUID_DFU_PACKET),
        },
        2,                                          /* bValueLen */
        NULL,
        GATT_PERM_WRITE                 /* wPermissions */
    }

    ,
    /* <<Characteristic>>, .. */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            (GATT_CHAR_PROP_WRITE |                   /* characteristic properties */
            GATT_CHAR_PROP_NOTIFY)
            /* characteristic UUID not needed here, is UUID of next attrib. */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },
    /*--- DFU Control Point value ---*/
    {
        ATTRIB_FLAG_VALUE_APPL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_DFU_CONTROL_POINT),
            HI_WORD(GATT_UUID_DFU_CONTROL_POINT)
        },
        0,                                          /* bValueLen, 0 : variable length */
        NULL,
        GATT_PERM_WRITE                  /* wPermissions */
    },
    /* client characteristic configuration */
    {
        (ATTRIB_FLAG_VALUE_INCL |                   /* wFlags */
        ATTRIB_FLAG_CCCD_APPL),
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHAR_CLIENT_CONFIG),
            HI_WORD(GATT_UUID_CHAR_CLIENT_CONFIG),
            /* NOTE: this value has an instantiation for each client, a write to */
            /* this attribute does not modify this default value:                */
            LO_WORD(GATT_CLIENT_CHAR_CONFIG_DEFAULT), /* client char. config. bit field */
            HI_WORD(GATT_CLIENT_CHAR_CONFIG_DEFAULT)
        },
        2,                                          /* bValueLen */
        NULL,
        (GATT_PERM_READ | GATT_PERM_WRITE)          /* wPermissions */
    }

};

const int iDfuServiceSize = sizeof(DFU_Service);


const uint8_t dfuTargetAdData[] =
{
    /* Flags */
    0x02,             /* length     */
    //XXXXMJMJ 0x01, 0x06,      /* type="flags", data="bit 1: LE General Discoverable Mode", BR/EDR not supp. */
    0x01, 0x02,      /* type="flags", data="bit 1: LE General Discoverable Mode" */

    /* Service */
    0x03,             /* length     */
    0x02,            /* type="More 16-bit UUIDs available" */
    LO_WORD(GATT_UUID_DFU_SERVICE),
    HI_WORD(GATT_UUID_DFU_SERVICE),

    /*Complete local name*/
    0x0A,             /* length     */
    0x09,            /* type="Complete local name" */
    0x42, 0x65, 0x65, 0x54, 0x61, 0x72, 0x67, 0x65, 0x74 /* BeeTarget */
    
#if 0
        , 0x05, 0xFF, 0x00, 0x0D, 0xBE, 0xEF /* manufactor specific data, kernel image version(2bytes) + user image version(2Bytes) */
#endif

};

const int iDfuTargetAdvDataSize = sizeof(dfuTargetAdData);



extern uint8_t *g_pOtaTempBufferHead;
extern uint16_t g_OtaTempBufferUsedSize;
#define OTA_TEMP_BUFFER_SIZE 20*100

extern aes_context*g_pAesCtx; 


uint16_t dfu_Service_CCCDCheckValue( TDFU_CB* pDfuCb )
{

    uint16_t wCause = GATT_SUCCESS;



    if ( pDfuCb->AttrIdxCCCD.wCCCBits == 0 )
    {

        wCause = ATT_ERR | DFU_ERR_CCCD_IMPROPERLY_CONFIGURED;


    }


    return ( wCause );
}



void  dfu_Service_HandleControlPointReq( TDFU_CB *pDfuCb, uint16_t wLength, uint8_t * pValue )
{
    TDFUControlPoint dfuControlPoint;

    dfuControlPoint.opCode = * pValue;
    uint8_t *p = pValue + 1;

    APPL_TRACE_PRINTF_2(APPL_TRACE_MASK_TRACE, "dfu_Service_HandleControlPointReq: opCode=0x%x, wLength=%d",
                        dfuControlPoint.opCode,
                        wLength
                       );

    if (dfuControlPoint.opCode >= DFU_OPCODE_MAX || dfuControlPoint.opCode <= DFU_OPCODE_MIN )
    {

        return;
    }

    switch (dfuControlPoint.opCode)
    {
    case DFU_OPCODE_START_DFU:
        if (wLength == DFU_LENGTH_START_DFU + 4)// 4 bytes is pending for encrypt
        {
        

            if(otp_str_data.gEfuse_UpperStack_s.ota_with_encryption_data == TRUE)
            {
                aes_decrypt(g_pAesCtx, p, p);
            }

            dfuControlPoint.p.StartDfu.nOffSet = LE_EXTRN2WORD(p);
            p += 2;
            dfuControlPoint.p.StartDfu.nSignature = LE_EXTRN2WORD(p);
            p += 2;
            dfuControlPoint.p.StartDfu.nImageVersion = LE_EXTRN2WORD(p);
            p += 2;
            dfuControlPoint.p.StartDfu.nCRC16 = LE_EXTRN2WORD(p);
            p += 2;
            dfuControlPoint.p.StartDfu.nImageLength = LE_EXTRN2WORD(p);
            p += 2;
            dfuControlPoint.p.StartDfu.nReserved = LE_EXTRN2WORD(p);

            APPL_TRACE_PRINTF_6(APPL_TRACE_MASK_TRACE,
                                "DFU_OPCODE_START_DFU: nOffSet=0x%x, nSignature=0x%x, nImageVersion=0x%x, nCRC16=0x%x,nImageLength=0x%x*4Bytes, nReserved=0x%x",
                                dfuControlPoint.p.StartDfu.nOffSet,
                                dfuControlPoint.p.StartDfu.nSignature,
                                dfuControlPoint.p.StartDfu.nImageVersion,
                                dfuControlPoint.p.StartDfu.nCRC16,
                                dfuControlPoint.p.StartDfu.nImageLength,
                                dfuControlPoint.p.StartDfu.nReserved
                               );
            pDfuCb->nOffSet = dfuControlPoint.p.StartDfu.nOffSet;
            pDfuCb->nSignature = dfuControlPoint.p.StartDfu.nSignature;
            pDfuCb->nImageVersion = dfuControlPoint.p.StartDfu.nImageVersion;
            pDfuCb->nCRC16 = dfuControlPoint.p.StartDfu.nCRC16;
            pDfuCb->nImageLength = dfuControlPoint.p.StartDfu.nImageLength;
            pDfuCb->nReserved = dfuControlPoint.p.StartDfu.nReserved;

            pDfuCb->nImageTotalLength = (dfuControlPoint.p.StartDfu.nImageLength <<2) + (pDfuCb->nOffSet<<2);

            pDfuCb->nCurNotificationAct = DFU_SUBACT_RX_FW_CP_REQ_START_DFU;
            dfu_sm_event(pDfuCb, DFU_EVT_RX_FW_CP_REQ, NULL);

        }
        break;

    case DFU_OPCODE_RECEIVE_FW_IMAGE_INFO:
        if(wLength == DFU_LENGTH_RECEIVE_FW_IMAGE_INFO)
        {
            pDfuCb->nSignature = LE_EXTRN2WORD(p);
            p += 2;
            pDfuCb->nCurOffSet = LE_EXTRN2DWORD(p);
            APPL_TRACE_PRINTF_2(APPL_TRACE_MASK_TRACE, "DFU_OPCODE_RECEIVE_FW_IMAGE_INFO: nSignature = 0x%x, nCurOffSet = %d", 
                pDfuCb->nSignature,
                pDfuCb->nCurOffSet
                );      
#if UPPERSTACK_OTA_SUPPORT
    //do nothing
#else
            pDfuCb->nCurOffSet = 12;
#endif
        }
        else
        {
            APPL_TRACE_PRINTF_0(APPL_TRACE_MASK_ERROR, "DFU_OPCODE_RECEIVE_FW_IMAGE_INFO: invalid length");
        }

        break;

    case DFU_OPCODE_VALID_FW:

        if (wLength == DFU_LENGTH_VALID_FW)
        {
            pDfuCb->nSignature = LE_EXTRN2WORD(p);
            APPL_TRACE_PRINTF_1(APPL_TRACE_MASK_TRACE, "DFU_OPCODE_VALID_FW: nSignature = 0x%x", pDfuCb->nSignature);
            
            dfu_sm_event(pDfuCb, DFU_EVT_VALID_FW, NULL);
        }
        else
        {
            APPL_TRACE_PRINTF_0(APPL_TRACE_MASK_ERROR, "DFU_OPCODE_VALID_FW: invalid length");
        }
        break;

    case DFU_OPCODE_ACTIVE_IMAGE_RESET:
        //notify bootloader to reset and use new image
        dfu_sm_event(pDfuCb, DFU_EVT_ACTIVE_RESET, NULL);
        break;


    case DFU_OPCODE_SYSTEM_RESET:
        //notify bootloader to reset
        break;

    case DFU_OPCODE_REPORT_TARGET_INFO:
        if (wLength == DFU_LENGTH_REPORT_TARGET_INFO)
        {
            pDfuCb->nSignature = LE_EXTRN2WORD(p);
            pDfuCb->nCurNotificationAct = DFU_SUBACT_RX_FW_CP_REQ_REPORT_TARGET_INFO;
            dfu_sm_event(pDfuCb, DFU_EVT_RX_FW_CP_REQ, NULL);
        }
        else
        {
            APPL_TRACE_PRINTF_0(APPL_TRACE_MASK_ERROR, "DFU_OPCODE_REPORT_TARGET_INFO: invalid length");
        }
        break;

    case DFU_OPCODE_PKT_RX_NOTIFICATION_REQ:
        if (wLength == DFU_LENGTH_PKT_RX_NOTIFICATION_REQ)
        {
            dfuControlPoint.p.PktRxNotifyReq.PacketNum = LE_EXTRN2WORD(p);
            APPL_TRACE_PRINTF_1(APPL_TRACE_MASK_TRACE, "DFU_OPCODE_PKT_RX_NOTIFICATION_REQ: PacketNum=0x%x",
                                dfuControlPoint.p.PktRxNotifyReq.PacketNum
                               );
        }
        else
        {
            APPL_TRACE_PRINTF_0(APPL_TRACE_MASK_ERROR, "DFU_OPCODE_PKT_RX_NOTIFICATION_REQ: invalid length");
        }
        break;

    default:
        {
            APPL_TRACE_PRINTF_1(APPL_TRACE_MASK_ERROR, "dfu_Service_HandleControlPointReq: Unknown Opcode=0x%x",
                                dfuControlPoint.opCode
                               );
        }

        break;

    }
}


void  dfu_Service_HandlePacketReq( TDFU_CB *pDfuCb, uint16_t wLength, uint8_t * pValue )
{

     APPL_TRACE_PRINTF_3(APPL_TRACE_MASK_TRACE, "patch_upper_dfu_Service_HandlePacketReq: wLength=%d, nCurOffSet =%d, nImageTotalLength= %d",
                        wLength,
                        pDfuCb->nCurOffSet,
                        pDfuCb->nImageTotalLength
                       );
    //UINT32 iTime1, iTime2 = 0;
    if (pDfuCb->nCurOffSet+ g_OtaTempBufferUsedSize + wLength > pDfuCb->nImageTotalLength)
    {
        dfu_sm_event(pDfuCb, DFU_EVT_SYSTEM_RESET, NULL);
    }
    else
    {
        if(wLength >= 16)
        {
//          reset_vendor_counter();
//          iTime1 = read_vendor_counter_no_display();


            if(otp_str_data.gEfuse_UpperStack_s.ota_with_encryption_data == TRUE)
            {
                aes_decrypt(g_pAesCtx, pValue, pValue);
            }

//          iTime2 = read_vendor_counter_no_display();
            
        }
        
        memcpy(g_pOtaTempBufferHead + g_OtaTempBufferUsedSize, pValue, wLength);        
        g_OtaTempBufferUsedSize += wLength;

        if(g_OtaTempBufferUsedSize == OTA_TEMP_BUFFER_SIZE ||
            pDfuCb->nCurOffSet + g_OtaTempBufferUsedSize == pDfuCb->nImageTotalLength
            )
        {

            if(ota_update(pDfuCb->nSignature, pDfuCb->nCurOffSet, g_OtaTempBufferUsedSize, g_pOtaTempBufferHead)==0)
            {

            }
            else
            {
                //eflash write fail, we should restart ota procedure.
                ota_reset(pDfuCb->nSignature);
                ota_fw_active_reset();        
            }
            pDfuCb->nCurOffSet += g_OtaTempBufferUsedSize;
            g_OtaTempBufferUsedSize = 0;
        }

    }

}



uint16_t  dfuServiceAttribPut( void* pDfuCb, int iAttribIndex,
                               uint16_t wLength, uint8_t * pValue)
{
    TDFU_CB* pDFU_CB = (TDFU_CB*)pDfuCb;
    if (iAttribIndex == INDEX_DFU_CONTROL_POINT_CHAR_VALUE)
    {
        dfu_Service_HandleControlPointReq((TDFU_CB*)pDfuCb, wLength, pValue);
    }
    else if (iAttribIndex == INDEX_DFU_PACKET_VALUE)
    {
        dfu_Service_HandlePacketReq((TDFU_CB*)pDfuCb, wLength, pValue);
    }
    else
    {
        APPL_TRACE_PRINTF_3(APPL_TRACE_MASK_ERROR, 
            "!!!dfuServiceAttribPut Fail: iAttribIndex=%d, nCurOffSet =%d, ec(%d)",
            iAttribIndex,
            pDFU_CB->nCurOffSet,
            pDFU_CB->nTotalErrorCount
            );
        
        pDFU_CB->nTotalErrorCount++;
        return 1;

    }
    return 0;
}



