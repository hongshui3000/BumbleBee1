enum { __FILE_NUM__ = 0 };

#include <FreeRTOS.h>
#include <task.h>
#include <blueapi_types.h>
#include "blueapi.h"
#include <ltplib.h>
#include "aci_if.h"
#include "btltp.h"
#include "trace.h"
#include <stm32f10x.h>
#if BREDR_SUPPORT
#include "btltp_br.h"
#endif
//#include "rtl876x.h"
//#include "hal_wdg.h"

extern TBTLtp  BtLtp;

#if F_LTPLIB_ASYNC_ASSEMBLY_SUPPORT
void ltpTimerStart(TLTPTimerID TimerID, int TimerMS);
#endif
void BTLTPBufferCallback(THandle Handle)
{
    PBTLtpAction         pAction    = (PBTLtpAction)Handle.lpHandle;
    TBTLtpActionCommand  thisAction = pAction->Action;

    pAction->Action = btltpActionNotUsed;

    switch (thisAction)
    {
    case btltpActionReset: /*---------------------------------------------*/

        DBG_BUFFER(MODULE_LTP, LEVEL_INFO, "LTP: reset (wait for WD to kick in)", 0);
        #if 0
        /* wait for last char of ResetRsp (buffercallback is executed on txempty, NOT on txcomplete) */
        vTaskDelay(20 / portTICK_RATE_MS);                     /* 20 ms delay */
        HalWatchDogConfig(0, 5, 1);
        HalWatchDogEnable();  /* system reset */
		    #else
		    vTaskDelay(portTICK_RATE_MS * 20);                       /* 20 ms delay */
        SCB->AIRCR = 0x05FA0000 | SCB_AIRCR_SYSRESETREQ;
		    #endif
        break;
    case btltpActionSendDataConf: /*--------------------------------------*/
      {
        PBTLtpMDLContext pMDLContext = BTLTPFindMDLContext(&BtLtp, pAction->p.MDL_ID);
        //BOOL             ret;

        if (pMDLContext != NULL)
        {
          pMDLContext->pendingDataConfs++;
          while (pMDLContext->pendingDataConfs > 0)
          {
#if 0
            ret = blueAPI_DataConf(//NULL,
                                   //tBTLtp.blueAPIHandle,
                                   pAction->p.MDL_ID,
                                   blueAPI_CauseSuccess
                                  );

            /* if msg fails, keep number of failed dataConfs and retry later */
            if (ret == FALSE)
            {
              DBG_BUFFER(MODULE_LTP, LEVEL_ERROR, "!!LTP: failed to send [%d] COM_DataConf(s)",1, 
                                        pMDLContext->pendingDataConfs
                                        );
              break;
            }
#endif
            pMDLContext->pendingDataConfs--;
          }
        }
        LTPLibTriggerLTPProccess(&BtLtp.LTPLib);
      }
      break;
    case btltpActionSendDIDDeviceConf: /*---------------------------------*/
      blueAPI_DIDDeviceConf(//pAction->p.serviceAction.pBuffer,
                            //tBTLtp.blueAPIHandle,
                            pAction->p.serviceAction.serviceHandle,
                            blueAPI_CauseSuccess
                           );
      break;

    case btltpActionSendSPPEndpointConf: /*-------------------------------*/
      blueAPI_SDPEndpointConf(//pAction->p.serviceAction.pBuffer,
                              //tBTLtp.blueAPIHandle,
                              pAction->p.serviceAction.serviceHandle,
                              blueAPI_CauseSuccess
                              );
      break;

    default: /*-----------------------------------------------------------*/
        break;
    }
}

/****************************************************************************/
/* BOOL BTLTPTgtHandleLTPMessage                                            */
/* (                                                                        */
/*    LTP_TGT_APPHANDLE AppHandle : Handle to identity Application Context  */
/*    uint8_t *            pMsgBuffer: pointer to message buffer               */
/*    uint8_t              cmd       : identifier for LTP-command to be handled*/
/*    uint8_t              copmsk    : copmsk of LPT-command to be handled     */
/*    uint8_t *            pOpt      : pointer to optional parameters of LTP-  */
/*                                  command to be handled, or NULL in case  */
/*                                  of no optional parameters included      */
/*    uint16_t              lenPara   : length of mandatory parameters of LTP-  */
/*                                  command to be handled                   */
/*    uint8_t *            pPara     : pointer to mandatory parameters of LTP- */
/*                                  command to be handled, or NULL in case  */
/*                                  of no mandatory parameters included     */
/* )                                                                        */
/****************************************************************************/
static VOID BTLTPTgtHandleLTPSubMessage(PBTLtp pBTLtp, uint8_t subCmd, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara)
{
    uint8_t command = subCmd;

    switch (command)
    {
    case LTP_SUB_SET_RANDOM_ADDRESS_REQ:
        BTLTPHandleSetRandomAddressReq(pBTLtp, copmsk, pOpt, lenPara, pPara);
        break;
    case LTP_SUB_DEVICE_CONFIG_DEVICE_NAME_SET_REQ:
        BTLTPHandleDeviceNameSetReq(pBTLtp, copmsk, pOpt, lenPara, pPara);
        break;
    case LTP_SUB_DEVICE_CONFIG_SECURITY_SET_REQ:
        BTLTPHandleDeviceConfigSecuritySetReq(pBTLtp, copmsk, pOpt, lenPara, pPara);
        break;
    case LTP_SUB_DEVICE_CONFIG_STORE_SET_REQ:
        BTLTPHandleDeviceConfigStoreSetReq(pBTLtp, copmsk, pOpt, lenPara, pPara);
        break;
    case LTP_SUB_DEVICE_CONFIG_APPEARANCE_SET_REQ:
        BTLTPHandleDeviceConfigAppearanceSetReq(pBTLtp, copmsk, pOpt, lenPara, pPara);
        break;
    case LTP_SUB_DEVICE_CONFIG_PER_PREF_CONN_PARAM_SET_REQ:
        BTLTPHandleDeviceConfigPerPrefConnParamSetReq(pBTLtp, copmsk, pOpt, lenPara, pPara);
        break;
#if 0
    case LTP_SUB_SET_LE_TX_POWER_REQ:
        BTLTPHandleSetLETxPowerReq(pBTLtp, copmsk, pOpt, lenPara, pPara);
        break;
    case LTP_SUB_SET_DATA_LENGTH_REQ:
        BTLTPHandleSetDataLengthReq(pBTLtp, copmsk, pOpt, lenPara, pPara);
        break;
#endif
    case LTP_SUB_DOWNLOAD_SERVICE_DATABASE_REQ:
        BTLTPHandleDownloadServiceDatabaseReq(pBTLtp, copmsk, pOpt, lenPara, pPara);
        break;
    case LTP_SUB_CLEAR_SERVICE_DATABASE_REQ:
        BTLTPHandleClearServiceDatabaseReq(pBTLtp, copmsk, pOpt, lenPara, pPara);
        break;
	case LTP_SUB_SET_TRACE_LEVEL_REQ:
        BTLTPHandleSetTraceLevelReq(pBTLtp, copmsk, pOpt, lenPara, pPara);
        break;
#if 0
    case LTP_SUB_GATT_ATTRIBUTE_PREPARE_WRITE_REQ:
        BTLTPHandleGATTAttributePrepareWriteReq(pBTLtp, copmsk, pOpt, lenPara, pPara);
        break;
    case LTP_SUB_GATT_ATTRIBUTE_EXECUTE_WRITE_REQ:
        BTLTPHandleGATTAttributeExecuteWriteReq(pBTLtp, copmsk, pOpt, lenPara, pPara);
        break;
    case LTP_SUB_GATT_ATTRIBUTE_PREPARE_WRITE_CNF:
        BTLTPHandleGATTAttributePrepareWriteCnf(pBTLtp, copmsk, pOpt, lenPara, pPara);
        break;
    case LTP_SUB_GATT_ATTRIBUTE_EXECUTE_WRITE_CNF:
        BTLTPHandleGATTAttributeExecuteWriteCnf(pBTLtp, copmsk, pOpt, lenPara, pPara);
        break;
#endif
    default:
        break;
    }
}

BOOL BTLTPTgtHandleLTPMessage(LTP_TGT_APPHANDLE AppHandle, uint8_t * pMsgBuffer, uint8_t cmd, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara)
{
    PBTLtp pBTLtp      = (PBTLtp)AppHandle;
    uint8_t command = cmd;

    pBTLtp->pMsgBuffer = NULL;

    switch (command)
    {
    case LTP_CREATE_MDL_CNF: /*-------------------------------------------*/
        BTLTPHandleCreateMDLConf(pBTLtp, copmsk, pOpt, lenPara, pPara);
        break;
    case LTP_DISCONNECT_MDL_REQ: /*---------------------------------------*/
        BTLTPHandleDisconnectMDLReq(pBTLtp, copmsk, pOpt, lenPara, pPara);
        break;
    case LTP_DISCONNECT_MDL_CNF: /*---------------------------------------*/
        BTLTPHandleDisconnectMDLConf(pBTLtp, copmsk, pOpt, lenPara, pPara);
        break;
    case LTP_EXIT_REQ: /*-------------------------------------------------*/
        BTLTPHandleExitReq(pBTLtp, copmsk, pOpt, pPara);
        break;
    case LTP_RESET_REQ: /*------------------------------------------------*/
        BTLTPHandleResetReq(pBTLtp, copmsk, pOpt, lenPara, pPara);
        break;
    case LTP_PASSKEY_REQUEST_CNF: /*--------------------------------------*/
        BTLTPHandlePasskeyRequestCnf(pBTLtp, copmsk, pOpt, lenPara, pPara);
        break;
    case LTP_REMOTE_OOB_REQUEST_CNF: /*-----------------------------------*/
        BTLTPHandleOOBRequestCnf(pBTLtp, copmsk, pOpt, lenPara, pPara);
        break;
    case LTP_AUTH_RESULT_EXT_CNF: /*--------------------------------------*/
        BTLTPHandleAuthResultExtCnf(pBTLtp, copmsk, pOpt, lenPara, pPara);
        break;
    case LTP_AUTH_RESULT_REQUEST_EXT_CNF: /*------------------------------*/
        BTLTPHandleAuthResultRequestExtCnf(pBTLtp, copmsk, pOpt, lenPara, pPara);
        break;
    case LTP_PAIRABLE_MODE_SET_REQ: /*------------------------------------*/
        BTLTPHandlePairableModeSetReq(pBTLtp, copmsk, pOpt, lenPara, pPara);
        break;
    case LTP_PASSKEY_REQ_REPLY_REQ: /*------------------------------------*/
        BTLTPHandlePasskeyReqReplyReq(pBTLtp, copmsk, pOpt, lenPara, pPara);
        break;

    case LTP_GATT_SERVICE_REGISTER_REQ: /*--------------------------------*/
        BTLTPHandleGATTServiceRegisterReq(pBTLtp, copmsk, pOpt, lenPara, pPara);
        break;
    case LTP_GATT_ATTRIBUTE_UPDATE_REQ: /*--------------------------------*/
        BTLTPHandleGATTAttributeUpdateReq(pBTLtp, copmsk, pOpt, lenPara, pPara);
        break;
    case LTP_GATT_ATTRIBUTE_UPDATE_STATUS_CNF: /*-------------------------*/
        BTLTPHandleGATTAttributeUpdateStatusCnf(pBTLtp, copmsk, pOpt, lenPara, pPara);
        break;
    case LTP_GATT_ATTRIBUTE_READ_CNF: /*----------------------------------*/
        BTLTPHandleGATTAttributeReadCnf(pBTLtp, copmsk, pOpt, lenPara, pPara);
        break;
    case LTP_GATT_ATTRIBUTE_WRITE_CNF: /*---------------------------------*/
        BTLTPHandleGATTAttributeWriteCnf(pBTLtp, copmsk, pOpt, lenPara, pPara);
        break;
    case LTP_GATT_SERVER_STORE_CNF: /*------------------------------------*/
        BTLTPHandleGATTServerStoreCnf(pBTLtp, copmsk, pOpt, lenPara, pPara);
        break;

    case LTP_CONNECT_GATT_MDL_REQ: /*-------------------------------------*/
        BTLTPHandleConnectGATTMDLReq(pBTLtp, copmsk, pOpt, lenPara, pPara);
        break;
    case LTP_GATT_DISCOVERY_REQ: /*---------------------------------------*/
        BTLTPHandleGATTDiscoveryReq(pBTLtp, copmsk, pOpt, lenPara, pPara);
        break;
    case LTP_GATT_DISCOVERY_CNF: /*---------------------------------------*/
        BTLTPHandleGATTDiscoveryCnf(pBTLtp, copmsk, pOpt, lenPara, pPara);
        break;
    case LTP_GATT_ATTRIBUTE_READ_REQ: /*----------------------------------*/
        BTLTPHandleGATTAttributeReadReq(pBTLtp, copmsk, pOpt, lenPara, pPara);
        break;
    case LTP_GATT_ATTRIBUTE_WRITE_REQ: /*---------------------------------*/
        BTLTPHandleGATTAttributeWriteReq(pBTLtp, copmsk, pOpt, lenPara, pPara);
        break;
    case LTP_GATT_ATTRIBUTE_CNF: /*---------------------------------------*/
        BTLTPHandleGATTAttributeCnf(pBTLtp, copmsk, pOpt, lenPara, pPara);
        break;

    case LTP_GATT_SECURITY_REQ: /*----------------------------------------*/
        BTLTPHandleGATTSecurityReq(pBTLtp, copmsk, pOpt, lenPara, pPara);
        break;

    case LTP_LE_ADVERTISE_REQ: /*-----------------------------------------*/
        BTLTPHandleLEAdvertiseReq(pBTLtp, copmsk, pOpt, lenPara, pPara);
        break;
    case LTP_LE_ADVERTISE_PARAMETER_SET_REQ: /*---------------------------*/
        BTLTPHandleLEAdvertiseParameterSetReq(pBTLtp, copmsk, pOpt, lenPara, pPara);
        break;
    case LTP_LE_ADVERTISE_DATA_SET_REQ: /*--------------------------------*/
        BTLTPHandleLEAdvertiseDataSetReq(pBTLtp, copmsk, pOpt, lenPara, pPara);
        break;
    case LTP_LE_SCAN_REQ: /*----------------------------------------------*/
        BTLTPHandleLEScanReq(pBTLtp, copmsk, pOpt, lenPara, pPara);
        break;
    case LTP_LE_MODIFY_WHITELIST_REQ: /*----------------------------------*/
        BTLTPHandleLEModifyWhitelistReq(pBTLtp, copmsk, pOpt, lenPara, pPara);
        break;
    case LTP_LE_CONNECTION_UPDATE_REQ: /*---------------------------------*/
        BTLTPHandleLEConnectionUpdateReq(pBTLtp, copmsk, pOpt, lenPara, pPara);
        break;
    case LTP_LE_CONNECTION_UPDATE_CNF: /*---------------------------------*/
        BTLTPHandleLEConnectionUpdateCnf(pBTLtp, copmsk, pOpt, lenPara, pPara);
        break;
    case LTP_EXTEND_COMMAND:
        BTLTPTgtHandleLTPSubMessage(pBTLtp, pPara[0], copmsk, pOpt, lenPara, pPara);
        break;

    default: /*-----------------------------------------------------------*/
#if BREDR_SUPPORT
        BTLTPHandleBREDRLtpMsg(pBTLtp, command, copmsk, pOpt, lenPara, pPara);
#else
        DBG_BUFFER(MODULE_LTP, LEVEL_ERROR, "BTLTPTgtHandleLTPMessage, unknown command = 0x%x", 1, command);
#endif
        break;
    }

    return TRUE;
}

/****************************************************************************/
/* BOOL BTLTPTgtSendLTPMessage                                              */
/* (                                                                        */
/*    LTP_TGT_APPHANDLE AppHandle : Handle to identity Application Context  */
/*    uint8_t *            pMsg      : pointer to of LTP msg buffer to be send */
/*    uint16_t              offset                                              */
/*    uint16_t              dataLen                                             */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* FALSE in case the message could be send successfully,                    */
/* TRUE in case the message could not be send but was dumped                */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This function is used to send an LTP message to an Application with the  */
/* BT_LTP_Sendxxx functions of this library                                 */
/****************************************************************************/
BOOL BTLTPTgtSendLTPMessage(LTP_TGT_APPHANDLE AppHandle, uint8_t * pBuffer, uint16_t offset, uint16_t dataLen)
{
    PBTLtp    pBTLtp   = (PBTLtp)AppHandle;
    THandle   Handle;

    if (pBTLtp->State == btltpStateIdle)
    {
        /* put real buffer address and buffer callback in front of message */
        LtpWrite((uint8_t *)(pBuffer + offset), dataLen);
        return (TRUE);
    }
    else
    {
        pBTLtp->p_aci_tcb->tx_mem_tcb.tx_free_blk_idx = pBTLtp->p_aci_tcb->tx_mem_tcb.tx_blk_idx = 0;

        if (pBTLtp->pBufferAction)
        {
            Handle.lpHandle       = (PVOID)pBTLtp->pBufferAction;
            pBTLtp->pBufferAction = NULL;

            BTLTPBufferCallback(Handle);
        }

        DBG_BUFFER(MODULE_LTP, LEVEL_ERROR, "btLTP: did not send message (configurator active)", 0);

        return TRUE;
    }
}

/****************************************************************************/
/* uint8_t * BTLTPTgtSendBfferAlloc                                            */
/* (                                                                        */
/*    LTP_TGT_APPHANDLE AppHandle : Handle to identity Application Context  */
/*    uint16_t    len                 : size of buffer to be allocated (bytes)  */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* pointer to allocated memory in case of success                           */
/* NULL pointer in case of an error                                         */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This target specific function is used to allocate buffers for LTP        */
/* messages that are send to an Application with the BT_LTP_Sendxxx         */
/* functions of this library.                                               */
/****************************************************************************/
uint8_t * BTLTPTgtSendBufferAlloc(LTP_TGT_APPHANDLE AppHandle, uint16_t len)
{
    uint8_t * p_buf = NULL;
    PBTLtp    pBTLtp   = (PBTLtp)AppHandle;

    if (NULL == pBTLtp->p_aci_tcb)
    {
        return NULL;
    }

    /* free index > tx index  */
    if ((pBTLtp->p_aci_tcb->tx_mem_tcb.tx_free_blk_idx >= pBTLtp->p_aci_tcb->tx_mem_tcb.tx_blk_idx)
            && (pBTLtp->p_aci_tcb->tx_mem_tcb.tx_un_used_size != pBTLtp->p_aci_tcb->tx_mem_tcb.free_size))
    {
        /* [A---tx_idx----free_idx----B], have enough serial ram in  [free_idx----B]  */
        if ((pBTLtp->p_aci_tcb->tx_mem_tcb.tx_free_blk_idx + len) <= TX_BUFFER_SIZE)

        {
            p_buf = pBTLtp->p_aci_tcb->p_tx_buf + pBTLtp->p_aci_tcb->tx_mem_tcb.tx_free_blk_idx;
            pBTLtp->p_aci_tcb->tx_mem_tcb.tx_free_blk_idx += len;
            pBTLtp->p_aci_tcb->tx_mem_tcb.free_size -= len;
            /* if reach tx buffer size, return to index 0 */
            pBTLtp->p_aci_tcb->tx_mem_tcb.tx_free_blk_idx &= (TX_BUFFER_SIZE - 1);
        }
        /* [A---tx_idx----free_idx----B], have enough serial ram in  [A---tx_idx],
            discard [free_idx----B] */
        else if ((pBTLtp->p_aci_tcb->tx_mem_tcb.tx_free_blk_idx + len) > TX_BUFFER_SIZE
                 && pBTLtp->p_aci_tcb->tx_mem_tcb.tx_blk_idx >= len)
        {
            p_buf = pBTLtp->p_aci_tcb->p_tx_buf;
            pBTLtp->p_aci_tcb->tx_mem_tcb.tx_un_used_size =
                TX_BUFFER_SIZE - pBTLtp->p_aci_tcb->tx_mem_tcb.tx_free_blk_idx;
            pBTLtp->p_aci_tcb->tx_mem_tcb.tx_free_blk_idx = len;
            pBTLtp->p_aci_tcb->tx_mem_tcb.free_size -= len;
        }
        else
        {
            p_buf = NULL;   /* no enough free size */
            DBG_BUFFER(MODULE_LTP, LEVEL_ERROR, "Have no serial ram,free index:0x%x, tx index:0x%x, unused size:0x%x, free size:0x%x",
                       4, pBTLtp->p_aci_tcb->tx_mem_tcb.tx_free_blk_idx, pBTLtp->p_aci_tcb->tx_mem_tcb.tx_blk_idx, \
                       pBTLtp->p_aci_tcb->tx_mem_tcb.tx_un_used_size, pBTLtp->p_aci_tcb->tx_mem_tcb.free_size);
        }
    }
    /* free index < tx index */
    else if (pBTLtp->p_aci_tcb->tx_mem_tcb.tx_free_blk_idx < pBTLtp->p_aci_tcb->tx_mem_tcb.tx_blk_idx)
    {
        /* [A---free_idx----tx_idx----B],  have enough ram in  [free_idx----tx_idx]  */
        if ((pBTLtp->p_aci_tcb->tx_mem_tcb.tx_free_blk_idx + len) < pBTLtp->p_aci_tcb->tx_mem_tcb.tx_blk_idx)

        {
            p_buf = pBTLtp->p_aci_tcb->p_tx_buf + pBTLtp->p_aci_tcb->tx_mem_tcb.tx_free_blk_idx;
            pBTLtp->p_aci_tcb->tx_mem_tcb.tx_free_blk_idx += len;
            pBTLtp->p_aci_tcb->tx_mem_tcb.free_size -= len;
        }
        else
        {
            p_buf = NULL;   /* no enough free size */
            DBG_BUFFER(MODULE_LTP, LEVEL_ERROR, "Have no serial ram,free index:0x%x, tx index:0x%x, unused size:0x%x, free size:0x%x",
                       4, pBTLtp->p_aci_tcb->tx_mem_tcb.tx_free_blk_idx, pBTLtp->p_aci_tcb->tx_mem_tcb.tx_blk_idx, \
                       pBTLtp->p_aci_tcb->tx_mem_tcb.tx_un_used_size, pBTLtp->p_aci_tcb->tx_mem_tcb.free_size);
        }
    }
    else    /* no possiable */
    {
        p_buf = NULL;   /* no enough free size */
        DBG_BUFFER(MODULE_LTP, LEVEL_ERROR, "Have no serial ram", 0);
    }

    DBG_BUFFER(MODULE_LTP, LEVEL_INFO, "BufferAlloc:tx_idx == 0x%4x, free_idx = 0x%4x", 2, pBTLtp->p_aci_tcb->tx_mem_tcb.tx_blk_idx, \
               pBTLtp->p_aci_tcb->tx_mem_tcb.tx_free_blk_idx);

    return p_buf;
}

/****************************************************************************/
/* void BTLTPTgtReceiveBufferRelease                                        */
/* (                                                                        */
/*    LTP_TGT_APPHANDLE AppHandle : Handle to identity Application Context  */
/*    uint8_t *            pBuffer   : pointer to receive buffer to be released*/
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* non                                                                      */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This target specific function is used to released buffers for LTP        */
/* messages that are received and consumed by the 'LTPLibHandleReceiveData' */
/* function of this library.                                                */
/****************************************************************************/

void BTLTPTgtReceiveBufferRelease(LTP_TGT_APPHANDLE AppHandle, uint8_t * pBuffer)
{
    //UNUSED_PARAMETER(AppHandle);

    LtpBufferRelease((void *)pBuffer);
}

/****************************************************************************/
/* uint8_t * BTLTPTgtAssemblyBufferAlloc                                       */
/* (                                                                        */
/*    LTP_TGT_APPHANDLE AppHandle : Handle to identity Application Context  */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* pointer to allocated memory in case of success                           */
/* NULL pointer in case of an error                                         */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This target specific function is used to allocate buffers for LTP-       */
/* message assembly that is processed by functions of this library.         */
/****************************************************************************/
uint8_t * BTLTPTgtAssemblyBufferAlloc(LTP_TGT_APPHANDLE AppHandle)
{
    PBTLtp pBTLtp  = (PBTLtp)AppHandle;

    return pBTLtp->p_aci_tcb->p_rx_handle_buf;
}

/****************************************************************************/
/* PLTPElement BTLTPTgtQueueElementAlloc                                    */
/* (                                                                        */
/*    LTP_TGT_APPHANDLE AppHandle : Handle to identity Application Context  */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* pointer to allocated queue element in case of success                    */
/* NULL pointer in case of an error                                         */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This target specific function is used to allocate queue elements for LTP-*/
/* message assembly that is processed by functions of this library.         */
/****************************************************************************/
PLTPElement BTLTPTgtQueueElementAlloc(LTP_TGT_APPHANDLE AppHandle)
{
    PBTLtp      pBTLtp   = (PBTLtp)AppHandle;
    PLTPElement pElement;

    pElement = ltpQueueOut(&pBTLtp->FreeElementQueue);

    if (!pElement)
    {
        DBG_BUFFER(MODULE_LTP, LEVEL_ERROR, "btLTP: no element in BTLTPTgtQueueElementAlloc", 0);
    }

    return pElement;
}

/****************************************************************************/
/* void BTLTPTgtQueueElementRelease                                         */
/* (                                                                        */
/*    LTP_TGT_APPHANDLE AppHandle : Handle to identity Application Context  */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* non                                                                      */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This target specific function is used to release queue elements for LTP- */
/* message assembly that is processed by functions of this library.         */
/****************************************************************************/
void BTLTPTgtQueueElementRelease(LTP_TGT_APPHANDLE AppHandle, PLTPElement pLTPElement)
{
    PBTLtp pBTLtp   = (PBTLtp)AppHandle;

    ltpQueueIn(&pBTLtp->FreeElementQueue, pLTPElement);
}

#if F_LTPLIB_ASYNC_ASSEMBLY_SUPPORT
/****************************************************************************/
/* void BTLTPTgtTriggerTimer                                                */
/* (                                                                        */
/*    LTP_TGT_APPHANDLE AppHandle : Handle to identify application context  */
/*    TLTPTimerID       timerID   : ID to identify addressed timer          */
/*    uint16_t              timeout_ms: new timeout for addressed timer in ms   */
/* )                                                                        */
/****************************************************************************/
void BTLTPTgtTriggerTimer(LTP_TGT_APPHANDLE AppHandle, TLTPTimerID timerID, uint16_t timeout_ms)
{
    ltpTimerStart(timerID, timeout_ms);
}

/****************************************************************************/
/* void BTLTPTgtHandleAsyncLTP_RESET_REQ                                    */
/* (                                                                        */
/*    LTP_TGT_APPHANDLE AppHandle : Handle to identify application context  */
/* )                                                                        */
/****************************************************************************/
void BTLTPTgtHandleAsyncLTP_RESET_REQ(LTP_TGT_APPHANDLE AppHandle)
{
    PBTLtp pBTLtp  = (PBTLtp)AppHandle;

    BTLTPHandleResetReq(pBTLtp, 0x00, NULL, 0, NULL);
}
#endif /* F_LTPLIB_ASYNC_ASSEMBLY_SUPPORT */

