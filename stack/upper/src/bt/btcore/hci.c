/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       hci.c
* @brief     handle hci related info(command, event, data etc.)
* @details   
*
* @author  gordon
* @date      2015-06-26
* @version	v0.1
*/

#include <flags.h>
#include <os_message.h>
#include <os_pool.h>
#include <os_mem.h>
#include <message.h>
#include <btcommon.h>
#include <hci.h>
#include <hci_api.h>
#include <hci_code.h>
#include <hci_le.h>

#include <hci_llif.h>
#if F_BT_BREDR_SUPPORT
#include <hci_br.h>
#endif
#include <btsm_api.h>
#include <l2c_api.h>
#include <blueapi_api.h>
#include <upper_stack_global.h>

#if UPPER_STACK_USE_VIRTUAL_HCI
#include <comapi.h>
#endif

#if UPPER_STACK_USE_NORMAL_HCI
#include <hci_transport_uart.h>
#endif

#define TRACE_MODULE_ID             MID_BT_HCI




/* needed for hciChangeConnPacketType */
#if (BT_DS_PDU_L2C_BYTE_COUNT > BT_US_PDU_L2C_BYTE_COUNT)
#define BT_MAX_PACKET_SIZE BT_DS_PDU_L2C_BYTE_COUNT
#else
#define BT_MAX_PACKET_SIZE BT_US_PDU_L2C_BYTE_COUNT
#endif


/**
* @brief  init hci
* 
* @param
*
* @return  init result
*
*/
BOOL hciInit(void)
{
    int i;

    pHCI = osMemoryClearAllocate(RAM_TYPE_DATA_ON, sizeof(THCI));

    if (osCoroutineCreate(&hciQueueID, (TCoroutineEntry)hciEntry, pHCI))
    {
        DebuggerBreak();
        return(FALSE);
    }

    osMessageQueueCreate(&pHCI->hciQueueID);
    osMessageQueueCreate(&pHCI->hciTransQueueID);       /* HCI command queue for complex commands (e.g. CREATE_CON) */
    osMessageQueueCreate(&pHCI->hciTransQueueID2);      /* HCI command queue for DISC commands */

    pHCI->phciDescDon = osMemoryAllocate(RAM_TYPE_DATA_ON, sizeof(ThciHandleDesc)*otp_str_data.gEfuse_UpperStack_s.num_link_don);
    pHCI->phciDescDoff = osMemoryAllocate(RAM_TYPE_DATA_OFF, sizeof(ThciHandleDesc)*otp_str_data.gEfuse_UpperStack_s.num_link_doff);

    for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_don; i++)
    {
        pHCI->phciDescDon[i].used            = FALSE;
        pHCI->phciDescDon[i].i               = i;    /* self index */
        pHCI->phciDescDon[i].currentRole     = HCI_ROLE_UNKNOWN;
        pHCI->phciDescDon[i].usAclCount      = 0;    /* there are no open packets in upstream direction */
        pHCI->phciDescDon[i].dsAclTotal      = 999;  /* something non blocking */
        pHCI->phciDescDon[i].dsAclCount      = 0;    /* there are no pending hci blocks */
#if F_BT_BREDR_SUPPORT
        pHCI->phciDescDon[i].piconetType     = HCI_PICONET_MASTER_DONTCARE;
        pHCI->phciDescDon[i].link_policy     = 0x00;
        pHCI->phciDescDon[i].encryptionStatus = HCI_ENCRYPTION_STATE_OFF;
#endif
        if (osMessageQueueCreate(&pHCI->phciDescDon[i].dsAclQueueID))
        {
            DebuggerBreak();
            return(FALSE);
        }
    } /* for */

    for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_doff; i++)
    {
        pHCI->phciDescDoff[i].used            = FALSE;
        pHCI->phciDescDoff[i].usAclCount      = 0;    /* there are no open packets in upstream direction */
        pHCI->phciDescDoff[i].dsAclTotal      = 999;  /* something non blocking */
        pHCI->phciDescDoff[i].dsAclCount      = 0;    /* there are no pending hci blocks */
        pHCI->phciDescDoff[i].i               = i;    /* self index */
        pHCI->phciDescDoff[i].currentRole     = HCI_ROLE_UNKNOWN;
#if F_BT_BREDR_SUPPORT
        pHCI->phciDescDoff[i].piconetType     = HCI_PICONET_MASTER_DONTCARE;
        pHCI->phciDescDoff[i].link_policy     = 0x00;
        pHCI->phciDescDoff[i].encryptionStatus = HCI_ENCRYPTION_STATE_OFF;
#endif
        if (osMessageQueueCreate(&pHCI->phciDescDoff[i].dsAclQueueID))
        {
            DebuggerBreak();
            return(FALSE);
        }
    } /* for */
    
    pHCI->status        = HCI_STATUS_IDLE;
    pHCI->manuName      = COMPID_UNINITIALIZED;
    pHCI->ReadOffset     = DATACB_INIT_OFFSET;
    pHCI->WriteOffset    = BT_L1_HCI_DS_OFFSET; /* updated in SET_PARAMETER_CONF */

#if F_BT_BREDR_SUPPORT    
    pHCI->saveScanEnable   = 0xFF;
    pHCI->crrScanEnable    = 0xFF;
    pHCI->piconetType   = HCI_PICONET_MASTER_DONTCARE;
    pHCI->link_supervision_timeout = 0x7D00; /* default 20 secs */
    pHCI->pageScanActivity.interval = PAGE_SCAN_ACTIVITY_INTERVAL;
    pHCI->pageScanActivity.window   = PAGE_SCAN_ACTIVITY_WINDOW;
    pHCI->scanRepetitionModeReq = 0xFF;   /* reserved value */
    pHCI->pageTimeout           = BT_HCI_PAGE_TIMEOUT_DEFAULT_COUNT; /* default value */
#endif

    return(TRUE);
}

/**
 * @brief  shut down all hci activities
 *
* @param 
*
* @return
*
*/
void hciShutDown(void)
{
    int i;
    ThciHandleDesc * hd = NULL;

    //HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_TRACE,"hciShutDown acl: %d", pHCI->aclHandleCnt);

    for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_don; i++)
    {
        hd = &pHCI->phciDescDon[i];
        if (hd->used)
        {
#if F_BT_BREDR_SUPPORT
            l2cHandleHciDiscInd(hd->handle, hciStatus(HCI_ERR_OUTOFSYNC), hd->conType);
#endif
            hd->used = FALSE;
        }
    }
    for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_doff; i++)
    {
        hd = &pHCI->phciDescDoff[i];
        if (hd->used)
        {
#if F_BT_BREDR_SUPPORT
            l2cHandleHciDiscInd(hd->handle, hciStatus(HCI_ERR_OUTOFSYNC), hd->conType);
#endif
            hd->used = FALSE;
        }
    }

    /*clear handle cnt*/
#if F_BT_BREDR_SUPPORT
    pHCI->aclHandleCnt = 0;
#endif
    pHCI->leAclHandleCnt = 0;
    
    hciConfigureNoConnectionsInd(CLEAR_ALL_CONNECTIONS, NULL, 0);
}

/**
* @brief      handle Fatal error condition
*
* @param
*
* @return
*
*/
void hciFatal(void)
{
    /* remove all active links from the view of the upper layers */
    hciShutDown();
    blueAPI_Handle_HCI_ACT_CONF(pHCI->localBdAddr, hciStatus(HCI_ERR_TIMEOUT));
    /* reinitialize */
    pHCI->status = HCI_STATUS_RESETTING;

}

void hciDeactReq(void)
{
    HCI_TRACE_PRINTF_0(HCI_TRACE_MASK_TRACE,"hciHandleDEACTREQ");

    hciLLClose();
    hciShutDown();          /* declare all open links gone */
    hciLaterEntry();
}

/**
* @brief	 HCI link lost synchronisation, close all links, reset controller
*
* @param
*
* @return
*
*/
void hciNotInSync(void)
{
    if (pHCI->status != HCI_STATUS_READY)
    {
        return;
    }

    HCI_TRACE_PRINTF_0(HCI_TRACE_MASK_ERROR, "!!HCI Out of Sync Restarting");
    hciFatal();
}

/****************************************************************************/
/* hciBufferCallback: Upstream Flow Control                                 */
/****************************************************************************/
#if F_BT_BREDR_SUPPORT
void hciBufferCallback(uint32_t handle)
{
    LPThciHandleDesc hd = (LPThciHandleDesc)handle;

    if (!hd->usAclCount)
    {
        HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_ERROR,"!!hciBufferCallback: release without allocated buffer  cnt %x ?", hd->usAclCount);
        return;
    }
    hd->usAclCount--;

    /* Notifiy controller that packet(s) are locally free'd if there are less that 2 remaining buffers */
    pHCI->usAclHandshake++;
    /* @@@@@ this feature is currently disabled: we have to note the handles for which
    the packets were received and we have to check at connection closure (delete handle)
    if there are any open packets (unacked)
    */
    /*gordon!!here we maybe need to op*/
    if ((HCI_ACL_CNT - pHCI->usAclHandshake) < 2 || 1)
    {
        if (pHCI->capabilities & HCI_CAP_HOST_FLOW)
        {
            hciCommandHostNumberOfCompletedPackets(hd->handle, pHCI->usAclHandshake);
        }
        pHCI->usAclHandshake = 0;
    }
} /* hciBufferCallback */
#endif
/**
* @brief  handle hci event
* 
* @param
*
* @return  
*
*/
void hciHandlePhDataEventInd(PHCI pHCI)
{
    uint8_t *p = pHCI->Message.MData.DataCB.BufferAddress + pHCI->Message.MData.DataCB.Offset;
    uint8_t *p_for_release = pHCI->Message.MData.DataCB.BufferAddress;
    uint16_t release_flag = pHCI->Message.MData.DataCB.Flag;
    hciProcessEventPacket(p);
    if (release_flag  & DATA_CB_RELEASE)
    {
        osBufferRelease(p_for_release);
    }
}

/**
* @brief  handle acl data indicate
* 
* @param
*
* @return  
*
*/
void hciHandlePhDataAclInd(PHCI pHCI)
{
    uint16_t handle;
    ThciHandleDesc * hd;

    if (pHCI->initPhase == 0)
    {
        /* discard all data received before baseband initialisation complete */
        HCI_TRACE_PRINTF_0(HCI_TRACE_MASK_TRACE, "hciHandlePhDataAclInd() discards ACL data");
        return;
    }

    handle = CHAR2SHORT(pHCI->Message.MData.DataCB.BufferAddress + pHCI->Message.MData.DataCB.Offset) & 0xfff;
    hd     = hciFindHandle(pHCI, handle);

    if (!hd)
    {
        HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_ERROR, "!!hciHandlePhDataAclInd: handle %.4X not found", handle);
        if (pHCI->Message.MData.DataCB.Flag & DATA_CB_RELEASE)
        {
            osBufferRelease(pHCI->Message.MData.DataCB.BufferAddress);
        }
        return;
    }
#if F_BT_BREDR_SUPPORT
    hd->usAclCount++;
#endif
    hciProcessAclPacket(pHCI,
                        pHCI->Message.MData.DataCB.BufferAddress,
                        pHCI->Message.MData.DataCB.Offset,
                        pHCI->Message.MData.DataCB.Length);
}

/**
* @brief  handle hci data ind
* 
* @param
*
* @return  
*
*/
void hciHandlePhDataIndExt(PHCI pHCI)
{
    DATA_CB_T DataCB;

#if UPPER_STACK_USE_NORMAL_HCI
    uint8_t *  pPacket;

    comResponse((HCOM)pHCI->llHandle, (uint8_t *)pHCI->Message.MData.DataCBExt.Handle.lpHandle);

    pPacket = pHCI->Message.MData.DataCBExt.DataCB.BufferAddress +
                    pHCI->Message.MData.DataCBExt.DataCB.Offset++;
    pHCI->Message.MData.DataCBExt.DataCB.Length--;
#endif
    DataCB = pHCI->Message.MData.DataCBExt.DataCB;
    pHCI->Message.MData.DataCB = DataCB;

#if UPPER_STACK_USE_NORMAL_HCI
    switch (*pPacket++)
#endif
#if UPPER_STACK_USE_VIRTUAL_HCI
    switch (pHCI->Message.MData.DataCBExt.DataCB.Pkt_type)
#endif
    {
    case ACL_PKT:
        hciHandlePhDataAclInd(pHCI);
        break;
    case EVENT_PKT:
        hciHandlePhDataEventInd(pHCI);
#if UPPER_STACK_USE_VIRTUAL_HCI
        hciLLResponse(pHCI->Message.MData.DataCBExt.Handle.lpHandle);
#endif
        break;
    default:
#if UPPER_STACK_USE_VIRTUAL_HCI
        HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_ERROR,
                       "!!hciHandlePhDataIndExt: unknown packet type: %d",pHCI->Message.MData.DataCBExt.DataCB.Pkt_type);
#endif
        DebuggerBreak();
        break;
    }
}

/**
* @brief  update hci free buffer count 
* 
* @param
*
* @return  
*
*/
void hciGlobalFreeBufferCountUpdated(void)
{
    uint8_t i;

    for (i=0; i < otp_str_data.gEfuse_UpperStack_s.num_link_don && 0 < pHCI->dsGlobalFreeBufferCount; i++)
    {
        /* ?acl link not used -> assuming that pending acl buffers were completed after disconnecting */
        if ( !pHCI->phciDescDon[i].used )
        {
//#if F_BT_BREDR_SUPPORT
            if ( pHCI->phciDescDon[i].dsAclCount > 0 )
            {
                if ( pHCI->dsGlobalFreeBufferCount <= pHCI->phciDescDon[i].dsAclCount )
                {
                    pHCI->phciDescDon[i].dsAclCount -= pHCI->dsGlobalFreeBufferCount;
                    pHCI->dsGlobalFreeBufferCount = 0;
                }
                else
                {
                    pHCI->dsGlobalFreeBufferCount -= pHCI->phciDescDon[i].dsAclCount;
                    pHCI->phciDescDon[i].dsAclCount = 0;
                }
            }
//#endif
            if ( pHCI->phciDescDon[i].leDsAclCount > 0 )
            {
                if ( pHCI->dsGlobalFreeBufferCount <= pHCI->phciDescDon[i].leDsAclCount )
                {
                    pHCI->phciDescDon[i].leDsAclCount -= pHCI->dsGlobalFreeBufferCount;
                    pHCI->dsGlobalFreeBufferCount = 0;
                }
                else
                {
                    pHCI->dsGlobalFreeBufferCount -= pHCI->phciDescDon[i].leDsAclCount;
                    pHCI->phciDescDon[i].leDsAclCount = 0;
                }
            }
        }
    }

    for (i=0; i < otp_str_data.gEfuse_UpperStack_s.num_link_doff && 0 < pHCI->dsGlobalFreeBufferCount; i++)
    {
        /* ?acl link not used -> assuming that pending acl buffers were completed after disconnecting */
        if ( !pHCI->phciDescDoff[i].used )
        {
//#if F_BT_BREDR_SUPPORT
            if ( pHCI->phciDescDoff[i].dsAclCount > 0 )
            {
                if ( pHCI->dsGlobalFreeBufferCount <= pHCI->phciDescDoff[i].dsAclCount )
                {
                    pHCI->phciDescDoff[i].dsAclCount -= pHCI->dsGlobalFreeBufferCount;
                    pHCI->dsGlobalFreeBufferCount = 0;
                }
                else
                {
                    pHCI->dsGlobalFreeBufferCount -= pHCI->phciDescDoff[i].dsAclCount;
                    pHCI->phciDescDoff[i].dsAclCount = 0;
                }
            }
//#endif
            if ( pHCI->phciDescDoff[i].leDsAclCount > 0 )
            {
                if ( pHCI->dsGlobalFreeBufferCount <= pHCI->phciDescDoff[i].leDsAclCount )
                {
                    pHCI->phciDescDoff[i].leDsAclCount -= pHCI->dsGlobalFreeBufferCount;
                    pHCI->dsGlobalFreeBufferCount = 0;
                }
                else
                {
                    pHCI->dsGlobalFreeBufferCount -= pHCI->phciDescDoff[i].leDsAclCount;
                    pHCI->phciDescDoff[i].leDsAclCount = 0;
                }
            }
        }
    }

  /* the code shall necer be reached */
    if (pHCI->dsGlobalFreeBufferCount)
    {
        HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_ERROR, "!!hciGlobalFreeBufferCountUpdated(rest %d)", pHCI->dsGlobalFreeBufferCount);
        assert(FALSE);
    }
}

/**
* @brief  update after handle hci msg
* 
* @param  
*
* @return  
*
*/
void hciLaterEntry(void)
{
    int  i;
    BOOL any;

    /* First we handle HCI commands */
    while (pHCI->numHCI)
    {
        /* if there is an ongoing transaction and if we have a patch requiring platform,
        we break the loop and wait until the transaction is finished
        */
        if (pHCI->transaction)
        {
            if (osMessageReceive(pHCI->hciQueueID, &pHCI->Message))
            {
                break;
            }
        }
        else
        {
            if (osMessageReceive(pHCI->hciTransQueueID2, &pHCI->Message))
            {
                if (osMessageReceive(pHCI->hciTransQueueID, &pHCI->Message))
                {
                    if (osMessageReceive(pHCI->hciQueueID, &pHCI->Message))
                    {
                        break;
                    }
                }
            }
        }

        if(pHCI->Message.Command == PH_DATA_REQ)
        {
            uint16_t commandCode = CHAR2SHORT(pHCI->Message.MData.DataCB.BufferAddress + pHCI->Message.MData.DataCB.Offset + 1);

            /* decrement sending rights for non vendor specific commands, vendor specific do not consume numHCI rights */
            /* CSR: by setting of PSKEY HCI_CSR_PSKEY_HOSTIO_USE_HCI_EXTN_CCFC a COMMAND_COMPLETE is generated for each vendor specific command.
            The value of HCI_CSR_PSKEY_HOSTIO_USE_HCI_EXTN_CCFC is valid after BB init phase, also after CSR WARM_BOOT */
            if ((commandCode & ~0x3ff) != ((uint16_t)(HCI_VENDOR) << 10)) /*not vendor command*/
            {
                pHCI->numHCI--;
            }

            if (
                commandCode == HCI_DISCONNECT
#if F_BT_BREDR_SUPPORT
                || commandCode == HCI_WRITE_SCAN_ENABLE
                || commandCode == HCI_SET_EVENT_FILTER
                /* with bd args */
                || commandCode == HCI_CREATE_CONNECTION
                || commandCode == HCI_REMOTE_NAME_REQUEST
                /* with handle arg */
                || commandCode == HCI_INQUIRY
                || commandCode == HCI_SET_CONNECTION_ENCRYPTION

                || commandCode == HCI_SNIFF_MODE
                || commandCode == HCI_EXIT_SNIFF_MODE
#endif
                || commandCode == HCI_LE_CREATE_CONNECTION
                || commandCode == HCI_LE_CONNECTION_UPDATE
                || commandCode == HCI_LE_READ_REMOTE_USED_FEATURES
                || commandCode == HCI_LE_START_ENCRYPTION
                )
            {
                pHCI->transaction = commandCode;
            }
#if F_BT_BREDR_SUPPORT
            if (
                commandCode == HCI_LINK_KEY_REQUEST_REPLY ||
                commandCode == HCI_LINK_KEY_REQUEST_NEG_REPLY ||
                commandCode == HCI_PIN_CODE_REQUEST_REPLY ||
                commandCode == HCI_PIN_CODE_REQUEST_NEG_REPLY)
            {
                memcpy(pHCI->keyBdAddr, pHCI->Message.MData.DataCB.BufferAddress + pHCI->Message.MData.DataCB.Offset + 4, BD_ADDR_SIZE);
            } /* Command Code using BD, non blocked */

            if (commandCode == HCI_CREATE_CONNECTION ||
                commandCode == HCI_REMOTE_NAME_REQUEST)
            {
                memcpy(pHCI->transactionBdAddr, pHCI->Message.MData.DataCB.BufferAddress + pHCI->Message.MData.DataCB.Offset + 4, BD_ADDR_SIZE);
            } /* Command Code using BD */
            else 
            if (   commandCode == HCI_DISCONNECT
                || commandCode == HCI_SET_CONNECTION_ENCRYPTION
                || commandCode == HCI_SNIFF_MODE
                || commandCode == HCI_EXIT_SNIFF_MODE
                || commandCode == HCI_LE_CONNECTION_UPDATE
                || commandCode == HCI_LE_READ_REMOTE_USED_FEATURES
                || commandCode == HCI_LE_START_ENCRYPTION
                )
            {
                pHCI->transactionHandle = CHAR2SHORT(pHCI->Message.MData.DataCB.BufferAddress + pHCI->Message.MData.DataCB.Offset + 4);
            } /*Command Code using handle*/
#else
            if (commandCode == HCI_DISCONNECT
                || commandCode == HCI_LE_CONNECTION_UPDATE
                || commandCode == HCI_LE_READ_REMOTE_USED_FEATURES
                || commandCode == HCI_LE_START_ENCRYPTION
            )
            {
                pHCI->transactionHandle = CHAR2SHORT(pHCI->Message.MData.DataCB.BufferAddress + pHCI->Message.MData.DataCB.Offset + 4);
            } /*Command Code using handle*/
#endif
        } /* PH_DATA_REQ */

        hciLLWrite(&pHCI->Message);
    } /* while (pHCI->numHCI) */

    /* Calculate actual no of open blocks per connection (all conn are equal) */
#if F_BT_BREDR_SUPPORT
    if (pHCI->aclHandleCnt || pHCI->leAclHandleCnt)
#else
    if (pHCI->leAclHandleCnt)
#endif
    {
        /* Check for all handles if there is a chance to transmit downstream data */
        any = TRUE;
        while (any)
        {
        any = FALSE;

            /* right to transmit exists *///fixme
            for (i = 0;i < otp_str_data.gEfuse_UpperStack_s.num_link_don; i++)
            {
                ThciHandleDesc * hd = &pHCI->phciDescDon[(i + pHCI->startOffset) % otp_str_data.gEfuse_UpperStack_s.num_link_don];

                if (!hd->used)
                    continue;
#if F_BT_BREDR_SUPPORT
                if (hd->conType == BT_CONNECTION_TYPE_BR_ACL && hd->dsAclCount >= hd->dsAclTotal)
                    continue;
#endif
                if (hd->conType == BT_CONNECTION_TYPE_LE && hd->leDsAclCount >= hd->leDsAclTotal)
                    continue;

                if (osMessageReceive(hd->dsAclQueueID, &pHCI->Message) == 0)
                {
                    pHCI->startOffset = (i + pHCI->startOffset + 1) % otp_str_data.gEfuse_UpperStack_s.num_link_don;
                    hciLLWrite(&pHCI->Message);
#if F_BT_BREDR_SUPPORT
                    if (hd->conType == BT_CONNECTION_TYPE_BR_ACL)
                    {
                        hd->dsAclCount++;
                    }
                    else 
                    if (hd->conType == BT_CONNECTION_TYPE_LE)
                    {
#endif
                        hd->leDsAclCount++;
#if F_BT_BREDR_SUPPORT
                    }
#endif
                    any = TRUE;
                    break;
                } /* Message Receive */
            } /* for */

            /* right to transmit exists */
            for (i = 0;i < otp_str_data.gEfuse_UpperStack_s.num_link_doff; i++)
            {
                ThciHandleDesc * hd = &pHCI->phciDescDoff[(i + pHCI->startOffset_off) % otp_str_data.gEfuse_UpperStack_s.num_link_doff];

                if (!hd->used)
                    continue;
#if F_BT_BREDR_SUPPORT
                if (hd->conType == BT_CONNECTION_TYPE_BR_ACL && hd->dsAclCount >= hd->dsAclTotal)
                    continue;
#endif
                if (hd->conType == BT_CONNECTION_TYPE_LE && hd->leDsAclCount >= hd->leDsAclTotal)
                    continue;

                if (osMessageReceive(hd->dsAclQueueID, &pHCI->Message) == 0)
                {
                    pHCI->startOffset_off= (i + pHCI->startOffset_off+ 1) % otp_str_data.gEfuse_UpperStack_s.num_link_doff;

                    hciLLWrite(&pHCI->Message);
#if F_BT_BREDR_SUPPORT
                    if (hd->conType == BT_CONNECTION_TYPE_BR_ACL)
                    {
                        hd->dsAclCount++;
                    }
                    else if (hd->conType == BT_CONNECTION_TYPE_LE)
                    {
#endif
                        hd->leDsAclCount++;
#if F_BT_BREDR_SUPPORT
                    }
#endif
                    any = TRUE;
                    break;
                } /* Message Receive */
            } /* for */
        } /* while */
    } /* any open handle */
}

/**
* @brief  hci entry
* 
* @param  pHCI
*
* @return  
*
*/
void  hciEntry(THCI  *lpHCI)
{
    pHCI = (PHCI)(lpHCI);

    if (osMessageReceive(hciQueueID, &pHCI->Message) ==0)
    {
        hciHandleMessage();
    }

    hciLaterEntry();
}

/**
* @brief  handle hci message
* 
* @param
*
* @return  
*
*/
void hciHandleMessage(void)
{
    switch (pHCI->Message.Command) 
    {
    case PH_DATA_IND:
        hciHandlePhDataIndExt(pHCI);
        break;

    case HCI_DATA_REQ:
        hciHandleUpDataReq();
        break;

    default:
        HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_ERROR,
                            "!!hciHandleMessage: unknown command %X", pHCI->Message.Command);
        break;
    }
} /* hciHandleMessage */


void hciProcessVendorCommandComplete(uint8_t * fp, uint16_t pos, uint16_t len, uint16_t status)
{

}



/**
* @brief  handle command complete event 
* 
* @param  fp: point to packet(without packet type)
* @param  pos: 2: without event type and length
* @param  len: event length
*
* @return  
*
*/
void hciProcessCommandComplete(uint8_t * fp, uint16_t pos, uint16_t len)
{
    uint16_t response;
    uint8_t status;

    pHCI->numHCI  = fp[pos++]; /*get num of hci command which allowed to send to controller*/
    response      = CHAR2SHORT(fp+pos);  pos += 2; /*command opcode*/

    if (pHCI->initPhase == 0 && response != HCI_RESET)
    {
        /* discard all data received before hci_command_complete(HCI_RESET) or HCI_COMMAND_STATUS(HCI_NOOP) */
        HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_TRACE,
                            "hciProcessCommandComplete() discards hci_command_complete(0x%X) received before baseband initialisation completed",
                            response
                            );
        return;
    }

    status = len >= 4 ? fp[pos++] : HCI_SUCCESS; /*get status*/

    switch (response)
    {
    case HCI_NOOP: /* nothing more to do */
        break;

    case HCI_READ_LOCAL_VERSION_INFORMATION:
        if (pHCI->status == HCI_STATUS_INITIALIZING && status == HCI_SUCCESS)
        {

            pHCI->hciVersion    = fp[pos++];//fixme
            pHCI->hciRevision   = CHAR2SHORT(fp+pos); pos += 2;
            pHCI->lmpVersion    = fp[pos++];
            pHCI->manuName      = CHAR2SHORT(fp+pos); pos += 2;
            pHCI->lmpSubVersion = CHAR2SHORT(fp+pos); pos += 2;
            HCI_TRACE_PRINTF_5(HCI_TRACE_MASK_TRACE,
                                "HCI: Local Version hci 0x%x rev 0x%x lmp 0x%x manu 0x%x sub 0x%x",
                                pHCI->hciVersion,
                                pHCI->hciRevision,
                                pHCI->lmpVersion,
                                pHCI->manuName,
                                pHCI->lmpSubVersion);

            pHCI->capabilities |= HCI_CAP_HOST_FLOW;
            hciNextInitCommand();
        }
        break;

    case HCI_READ_BD_ADDR:
        if (pHCI->status == HCI_STATUS_INITIALIZING && status == HCI_SUCCESS)
        {
            memcpy(pHCI->localBdAddr, fp+pos, BD_ADDR_SIZE);
            hciNextInitCommand();
        }
        break;

    case HCI_READ_LOCAL_SUPPORTED_FEATURES:
        if (pHCI->status == HCI_STATUS_INITIALIZING && status == HCI_SUCCESS)
        {
#if F_BT_BREDR_SUPPORT
            memcpy(pHCI->localFeatures, fp+pos, sizeof(pHCI->localFeatures));
#endif
            hciNextInitCommand();
        }
        break;
//#if F_BT_BREDR_SUPPORT
    case HCI_READ_BUFFER_SIZE:
        if (pHCI->status == HCI_STATUS_INITIALIZING && status == HCI_SUCCESS)
        {
            uint16_t dsAclTotal;
            uint16_t dsAclBlocksPerLink;
            uint16_t i;
            uint16_t totolLink = otp_str_data.gEfuse_UpperStack_s.num_link_don+otp_str_data.gEfuse_UpperStack_s.num_link_doff;

            pHCI->dsAclSize = CHAR2SHORT(fp+pos);   pos += 2 + 1; /*get acl data packet length*/
            dsAclTotal= CHAR2SHORT(fp+pos);         pos += 2 + 2; /*get total number acl data packet*/

            HCI_TRACE_PRINTF_2(HCI_TRACE_MASK_TRACE, "HCI: READ_BUFFER_SIZE acl %d aclCnt %d",
                                pHCI->dsAclSize, dsAclTotal);

            if (dsAclTotal < totolLink)
            {
                /* configuration problem: BT_MAX_CONNECTION_COUNT is bigger than count of available ACL buffers */
                HCI_TRACE_PRINTF_0(HCI_TRACE_MASK_ERROR, "!!HCI: too few ACL buffers");
                DebuggerBreak();
            }

            dsAclBlocksPerLink = dsAclTotal / totolLink;
            dsAclTotal %= totolLink;    /* here is the rest: increment tx rights for first links in the table */

            for (i=0; i < otp_str_data.gEfuse_UpperStack_s.num_link_don; i++)
            {
                pHCI->phciDescDon[i].dsAclTotal = dsAclBlocksPerLink;
                if (dsAclTotal)
                {
                    pHCI->phciDescDon[i].dsAclTotal++;
                    dsAclTotal--;
                }
            }

            for (i=0; i < otp_str_data.gEfuse_UpperStack_s.num_link_doff; i++)
            {
                pHCI->phciDescDoff[i].dsAclTotal = dsAclBlocksPerLink;
                if (dsAclTotal)
                {
                    pHCI->phciDescDoff[i].dsAclTotal++;
                    dsAclTotal--;
                }
            }

            hciNextInitCommand();
        }
        break; /* HCI_READ_BUFFER_SIZE */
//#endif
#if F_BT_BREDR_SUPPORT
    case HCI_WRITE_SCAN_ENABLE:
        pHCI->transaction = 0;
        if (pHCI->status == HCI_STATUS_INITIALIZING && status == HCI_SUCCESS)
        {
            hciNextInitCommand();
        }
        else
        {
            if( pHCI->internScanEnable == FALSE )
            {
                blueAPI_Handle_HCI_WRITE_SCAN_ENABLE(hciStatus(status));
            }
        }
        break; /* HCI_WRITE_SCAN_ENABLE */
#endif
    case HCI_SET_EVENT_MASK:

#if F_BT_BREDR_SUPPORT
    case HCI_WRITE_AUTHENTICATION_ENABLE:
    case HCI_HOST_BUFFER_SIZE:
    case HCI_SET_HOST_CONTROLLER_TO_HOST_FLOW:
    case HCI_CHANGE_LOCAL_NAME:
    case HCI_WRITE_CONNECTION_ACCEPT_TIMEOUT:
    case HCI_WRITE_PAGE_SCAN_ACTIVITY:
    case HCI_WRITE_PAGE_TIMEOUT:

    case HCI_WRITE_PAGE_SCAN_TYPE:
    case HCI_READ_INQUIRY_SCAN_TYPE:
    case HCI_WRITE_INQUIRY_SCAN_TYPE:
    case HCI_WRITE_INQUIRY_MODE:
    case HCI_SNIFF_SUBRATING:
    case HCI_USER_CONFIRMATION_REQUEST_REPLY:
    case HCI_USER_CONFIRMATION_REQUEST_NEGATIVE_REPLY:
    case HCI_WRITE_CURRENT_IAC_LAP:
#endif
        if (status)
        {
            HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_ERROR,"!!HCI: COMMAND COMPLETE returned status %X", status);
        }
        if (pHCI->status == HCI_STATUS_INITIALIZING && status == HCI_SUCCESS)
        {
            if (pHCI->initPhase==0)
            {
                break;/* Still waiting for HCI_RESET */
            }
            hciNextInitCommand();
        }
        break;
#if F_BT_BREDR_SUPPORT
    case HCI_LINK_KEY_REQUEST_REPLY:
    case HCI_LINK_KEY_REQUEST_NEG_REPLY:
        btsmHandleCmdCompHCI_LINK_KEY_REQUEST_REPLY(response, fp+pos, hciStatus(status));
        break;

    case HCI_PIN_CODE_REQUEST_REPLY:
    case HCI_PIN_CODE_REQUEST_NEG_REPLY:
    case HCI_WRITE_LINK_POLICY_SETTINGS:
    case HCI_WRITE_LINK_SUPERVISION_TIMEOUT:
        if (status != HCI_SUCCESS)
        {
            HCI_TRACE_PRINTF_2(HCI_TRACE_MASK_ERROR,"!!HCI: COMMAND COMPLETE for COMMAND %X returned ERROR %X",response,status);
        }
        break;

    case HCI_INQUIRY_CANCEL:
        pHCI->transaction = 0;
        hciListenCnf(hciStatus(status));
        break;

    case HCI_ROLE_DISCOVERY:
        {
            uint16_t handle;
            ThciHandleDesc *pHd;

            handle = CHAR2SHORT(fp+pos);  pos += 2;
            pHd = hciFindHandle(pHCI, handle);
            if (pHd)
            {
                pHd->currentRole = fp[pos++];
                hciChangeLinkPolicyAndRole(pHd);
                if ( pHd->switchRoleTries == 0 && pHd->currentRole == HCI_ROLE_MASTER )
                {
                    hciCommandWordWordParameter(HCI_WRITE_LINK_SUPERVISION_TIMEOUT, handle, pHCI->link_supervision_timeout);
                }
            }
        }
        break;

    case HCI_WRITE_EXTENDED_INQUIRY_RESPONSE:
    /* we have locally stored the EIR argument pointer in HCI. This pointer will be released upon
       arrival of the corresponding command complete (regardless of status. Releasing the buffer will
       trigger a callback in SDP layer, and pssibly a new EIR will be generated (EIR flow control).
     */
        if (pHCI->wEIRp)
        {
            osBufferRelease(pHCI->wEIRp);
            pHCI->wEIRp = NULL;
        }
        break;

    case HCI_READ_LOCAL_OOB_DATA:
        blueAPI_Handle_HCI_READ_LOCAL_OOB(fp, pos, status);
        break;

    case HCI_SEND_KEYPRESS_NOTIFICATION:
        {
            TBlueAPI_Cause cause = blueAPI_CauseSuccess;
            if (hciStatus(status) != HCI_SUCCESS)
            {
                cause = blueAPI_CauseInvalidState;
            }
            blueAPI_Send_KeypressNotificationRsp(cause);
            }
        break;

    case HCI_USER_PASSKEY_REQUEST_REPLY:
    case HCI_USER_PASSKEY_REQUEST_NEGATIVE_REPLY:
        {
            TBlueAPI_Cause param_cause = blueAPI_CauseSuccess;
            if (hciStatus(status) != HCI_SUCCESS)
            {
                param_cause = blueAPI_CauseInvalidState;
            }
            blueAPI_Send_UserPasskeyReqReplyRsp(param_cause);
        }
        break;
#endif	
    case HCI_WRITE_LE_HOST_SUPPORTED://fixme
        if (pHCI->status == HCI_STATUS_INITIALIZING && status == HCI_SUCCESS)
        {
            hciNextInitCommand();
        }
        break;

    default:
        if (HCI_OGF(response) == HCI_LE_CONTROL)
        {
            hciLEProcessCommandComplete(fp+ pos, response, hciStatus(status));
        }
        else if (HCI_OGF(response) == HCI_VENDOR)
        {
            hciProcessVendorCommandComplete(fp + pos, len, response, hciStatus(status));
        }
        else
        {
           HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_ERROR,"!!hciProcessCommandComplete: unhandled response %X", response);
        }
        break;
   } /* switch */
} /* hciProcessCommandComplete */


/**
* @brief    process command status
*
* @param    p: 
* @param    os:
*
* @return
*
*/
void hciProcessCommandStatus(uint8_t * fp, uint16_t pos)
{
    uint16_t response;
    uint8_t status;
    uint8_t bMatchTransactionCommand = FALSE;

    /* Extract args from HCI event */
    status         = fp[pos++];
    pHCI->numHCI   = fp[pos++];
    response       = CHAR2SHORT(fp+pos); pos += 2;

    if ( pHCI->initPhase == 0 && response != HCI_NOOP ) /* ? still waiting for HCI_COMMAND_STATUS(HCI_NOOP) */
    {
        /* discard all data received before hci_command_complete(HCI_RESET) or HCI_COMMAND_STATUS(HCI_NOOP) */
        HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_TRACE,
                            "hciProcessCommandStatus() discards hci_command_status(0x%X) received before baseband initialisation completed",
                            response
                            );
        return;
    }

    if(pHCI->transaction == response && pHCI->transaction != HCI_NOOP /* HCI_NOOP is never serialized */)
    {
        if (status)
        {
            pHCI->transaction = 0; /* kill transaction immediately if command was not accepted from BB */
        }
        bMatchTransactionCommand = TRUE;
    }
    else if(pHCI->transaction != 0) 
    {
        HCI_TRACE_PRINTF_2(HCI_TRACE_MASK_ERROR,"!!HCI: unexpected COMMAND_STATUS response %X Pending %X", response, pHCI->transaction);
    }

    switch (response)
    {
#if F_BT_BREDR_SUPPORT
    case HCI_CREATE_CONNECTION:
        switch (pHCI->linkType)
        {
        case HCI_LINKTYPE_ACL:
            l2cHandleHciConCnf(pHCI->transactionBdAddr, hciStatus(status));
            break;

#if (F_BT_SCO)
        case HCI_LINKTYPE_SCO:
            blueAPI_Handle_SCO_CON_CONF(pHCI->remoteBdAddr, hciStatus(status));
            break;
#endif

        default:
            HCI_TRACE_PRINTF_2(HCI_TRACE_MASK_ERROR,"!!HCI: COMMAND_STATUS with ill linkType %d status %x", pHCI->linkType, status);
            break;
        }
        break;
#endif
    case HCI_DISCONNECT:
        {
            ThciHandleDesc * hd;

            if (bMatchTransactionCommand && status)
            {
                pHCI->transaction = 0;
            }
            hd = hciFindHandle(pHCI, pHCI->transactionHandle);

            if (hd && hd->conType == BT_CONNECTION_TYPE_LE)
            {
                hciLEProcessCommandStatus(response, hciStatus(status));
                break;
            }
#if F_BT_BREDR_SUPPORT
            if (bMatchTransactionCommand && status)//fixme
            {
                if (hd)
                {
#if (F_BT_SCO)
                    hciDiscAllScoByAcl(hd, pHCI->transactionHandle, status);
#endif
                    l2cHandleHciDiscInd(pHCI->transactionHandle, status, hd->conType);
                    hciRemoveHandle(pHCI->transactionHandle);
                }
            }
#endif
        }
        break;
#if F_BT_BREDR_SUPPORT
    case HCI_INQUIRY:
        if (status)
        {
            hciListenCnf(hciStatus(status));
        }
        break;

    case HCI_SNIFF_MODE:
        pHCI->transaction = 0;
        if (status)	/*enter sniff fail*/
        {
            ThciHandleDesc * hd;
            THCI_MODECHANGE_IND tmc;

            hd = hciFindHandle(pHCI, pHCI->transactionHandle);
            if (!hd)
            {
                HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_ERROR,"!!HCI: HCI_SNIFF_MODE status: could not find desc for handle %x",pHCI->transactionHandle);
                break;
            }
            if ( hd->nModeState == HCI_MODE_STATE_NEW_SNIFF_PENDING )
            {
                /* try one more time to enter sniff mode */
                hciCommandSniffMode(hd->handle, hd->sniffMax, hd->sniffMin, hd->sniffAttempt, hd->sniffTimeout);
                hd->nModeState = HCI_MODE_STATE_SNIFF_PENDING;
            }
            else
            {
                hd->nModeState = HCI_MODE_STATE_ACTIVE;

                tmc.bLinkContext = NULL;
                memcpy((PVOID)tmc.bd, hd->bd, BD_ADDR_SIZE);
                tmc.status = hciStatus(status);
                tmc.mode = 0; /* active mode */
                tmc.interval = hd->sniffMax;
                blueAPI_Handle_HCI_MODE_CHANGE_IND(&tmc);
            }
        }
        break;

    case HCI_EXIT_SNIFF_MODE:
        pHCI->transaction = 0;
        if (status)
        {
            ThciHandleDesc * hd;
            THCI_MODECHANGE_IND tmc;

            hd = hciFindHandle(pHCI, pHCI->transactionHandle);
            if (!hd)
            {
                HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_ERROR,"!!HCI: HCI_EXIT_SNIFF_MODE status: could not find desc for handle %x",pHCI->transactionHandle);
                break;
            }
            if ( hd->nModeState == HCI_MODE_STATE_NEW_SNIFF_PENDING )
            {
                /* if new sniff req is pending we try one more time to exit sniff mode */ 
                hciCommandWordParameter(HCI_EXIT_SNIFF_MODE, hd->handle);
                hd->nModeState = HCI_MODE_STATE_WAKE_UP_SNIFF_PENDING;
            }
            else
            {
                hd->nModeState = HCI_MODE_STATE_SNIFF;

                tmc.bLinkContext = NULL;
                memcpy((PVOID)tmc.bd, hd->bd, BD_ADDR_SIZE);
                tmc.status = hciStatus(status);
                tmc.mode  = 2; /* sniff mode */
                tmc.interval = hd->sniffMax;
                blueAPI_Handle_HCI_MODE_CHANGE_IND(&tmc);
            }
        }
        break;

    case HCI_AUTHENTICATION_REQUESTED:
    case HCI_LINK_KEY_REQUEST_NEG_REPLY:
        if (status)
        {
            btsmHandleHciAuthConf(pHCI->keyBdAddr, pHCI->tid, hciStatus(status));
        }
        break;

    case HCI_REMOTE_NAME_REQUEST:
        if (status)
        {
            blueAPI_Handle_HCI_NAME_CONF(pHCI->transactionBdAddr, hciStatus(status), NULL);
        }
        break;

#if (F_BT_SCO)
    case HCI_ADD_SCO_CONNECTION:
    case HCI_SETUP_SYNCHRONOUS_CONNECTION:
        blueAPI_Handle_SCO_CON_CONF(pHCI->remoteBdAddr, hciStatus(status));
        break;
#endif

    case HCI_SET_CONNECTION_ENCRYPTION:
        /* if there was an error, signal it upstream */
        if (status)
        {
            hciEncryptInd(pHCI->remoteBdAddr, hciStatus(status), 0 /* dont know */);
        }
    break;

    case HCI_CHANGE_CONNECTION_PACKET_TYPE:
        if (status)
        {
           HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_ERROR,"!!HCI_CHANGE_CONNECTION_PACKET_TYPE failed with status %x", status);
        }
        break;

    case HCI_READ_REMOTE_SUPPORTED_FEATURES:
    case HCI_READ_REMOTE_EXTENDED_FEATURES:
        {
            ThciHandleDesc * hd;

            hd = hciFindHandle(pHCI, pHCI->transactionHandle);
            if (!hd)
            {
                HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_ERROR,"!!HCI: HCI_READ_REMOTE_XXXXXXXX_FEATURES status: could not find desc for handle %x",pHCI->transactionHandle);
                break;
            }
            if (status)   /* Signal only if negative */
            {
                btsmHandleHciAclConnection(hd->bd, ACL_CONNECTION_NONSSP);
            }
        }
        break;
#endif
    default:
        if (HCI_OGF(response) == HCI_LE_CONTROL)
        {
            hciLEProcessCommandStatus(response, hciStatus(status));
        }
        else
        {
            HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_ERROR,"!!HCI: unhandled COMMAND_STATUS response %X", response);
        }
        break;
    } /* switch */
} /* hciProcessCommandStatus */

/**
* @brief  handle event packet
* 
* @param  fp: point to packet(without packet type)
*
* @return  
*
*/
void hciProcessEventPacket(uint8_t * fp)
{
    uint8_t event;
    uint16_t pos = 0;
    uint16_t len;
    uint16_t handle;
    uint8_t status;
    uint8_t reason;
    uint8_t numhan;

    event   = fp[pos++];
    len     = fp[pos++];

    if ( pHCI->initPhase == 0
    && event != HCI_COMMAND_COMPLETE    /* waiting for HCI_RESET completed */
    && event != HCI_COMMAND_STATUS      /* waiting for HCI_NOOP status */
    ) /* ? waiting for baseband initialisation completed */
    {
        /* discard all data received before baseband initialisation complete,
        * except HCI_COMMAND_COMPLETE(HCI_RESET) or HCI_COMMAND_STATUS(HCI_NOOP) */
        HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_TRACE,
                            "hciProcessEventPacket() discards hci_event(0x%X) received before baseband initialisation completed",
                            event
                            );
        return;
    }

    switch (event)
    {
    case HCI_COMMAND_COMPLETE:
        hciProcessCommandComplete(fp, pos, len);
        break;

    case HCI_COMMAND_STATUS:
        hciProcessCommandStatus(fp, pos);
        break;
#if F_BT_BREDR_SUPPORT
    case HCI_CONNECTION_REQUEST:
        {
            uint8_t * clas;
            uint8_t linktype;
            memcpy(pHCI->remoteBdAddr, fp+pos, BD_ADDR_SIZE); pos += BD_ADDR_SIZE;
            clas     = fp+pos; pos += BT_CLASS_SIZE;
            linktype = fp[pos++];
            if( linktype == HCI_LINKTYPE_ACL)
            {
                l2cHandleHciConInd(pHCI->remoteBdAddr, clas, linktype);
            }
#if (F_BT_SCO)
            else if(linktype ==  HCI_LINKTYPE_SCO || linktype ==  HCI_LINKTYPE_ESCO)
            {
                blueAPI_Handle_SCO_CON_IND(pHCI->remoteBdAddr, clas, linktype);
            }
#endif
            else 
            {
                HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_TRACE, "!!HCI: proccess event HCI_CONNECTION_REQUEST: unknown linktype %i", linktype);
                hciCommandBDAddrByteParameter(HCI_REJECT_CONNECTION_REQUEST,  pHCI->remoteBdAddr, HCI_ERR_REJECT_LIMITED_RESOURCES);
            }
        }
        break;

    case HCI_CONNECTION_COMPLETE:
        {
            uint8_t *             bd;
            uint8_t               encrypt;
            ThciHandleDesc *   hd;
#if (F_BT_SCO)
            uint8_t               linkType;
#endif
            status            = fp[pos++];
            handle            = CHAR2SHORT(fp+pos); pos += 2;
            bd                = fp + pos; pos += BD_ADDR_SIZE;
#if (F_BT_SCO)
            linkType          = fp[pos++];
#else
            pos++;      /*ignore link type*/
#endif
            encrypt           = fp[pos++];

            if ( pHCI->transaction == HCI_CREATE_CONNECTION && (memcmp(bd, pHCI->transactionBdAddr, BD_ADDR_SIZE) ==0))
            {
                pHCI->transaction = 0;   /* transaction completed */
            }
#if (F_BT_SCO)
            if (linkType == HCI_LINKTYPE_SCO)
            {
                HCI_TRACE_PRINTF_2(HCI_TRACE_MASK_TRACE, "HCI_CONNECTION_COMPLETE (SCO), handle %X status %X", handle, status);

                if (status == 0)
                {
                    if (hciNewHandle(bd, handle, BT_CONNECTION_TYPE_BR_SCO, BLUEFACE_BDTYPE_BR_EDR) == NULL)
                    {
                        HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_ERROR, "!!HCI_CONNECTION_COMPLETE(SCO), cannot allocate handle %X", handle);
                        status = HCI_ERR_NO_DESCRIPTOR;
                        hciCommandDisconnect(handle, HCI_ERR_OTHER_END_TERMINATE_13);
                    }
                }
                blueAPI_Handle_SCO_CON_ACT_IND(bd, handle, hciStatus(status), 2);  /* set default CVSD when receive connection complete evt???*/
            }
            else
#endif
            {
                if (status==0)
                {
                    hd = hciNewHandle(bd, handle, BT_CONNECTION_TYPE_BR_ACL, BLUEFACE_BDTYPE_BR_EDR);
                    if (hd == NULL)
                    {
                        HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_ERROR,"!!HCI_CONNECTION_COMPLETE, handle %X no descr", handle);
                        status = HCI_ERR_NO_DESCRIPTOR;
                        hciCommandDisconnect(handle, HCI_ERR_OTHER_END_TERMINATE_13);
                    }
                    l2cHandleHciConActInd(hciStatus(status), handle, bd);

                    if (encrypt)
                    {
                        hciEncryptInd(bd, status, encrypt);
                    }
                    hciCommandWordParameter(HCI_ROLE_DISCOVERY, handle);
                    hciChangeConnPacketType(handle);
                }
                else
                {
                    /* sending a CLEAR_ACL here might result in a CLEAR_ACL without
                    prior ADD_ACL, cause this is triggere too by connection complete
                    however it is needed in case of auth fails to inform security manager
                    about pin input timeout */
                    hciConfigureNoConnectionsInd(CLEAR_ACL_CONNECTION, hd->bd, hd->bdType);
                    l2cHandleHciConActInd(hciStatus(status), handle, bd);
                }
            }
        }
        break;

#if (F_BT_SCO)
    case HCI_SYNCHRONOUS_CONNECTION_COMPLETE:
    {
        uint8_t *  bd;
        uint8_t    linkType;
        uint8_t    airMode;

        status  = fp[pos++];
        handle  = CHAR2SHORT(fp+pos); pos += 2;
        bd  = fp+pos;
        pos += BD_ADDR_SIZE;
        linkType = fp[pos++];
        pos += 6; /* for sco skip: tx interval, re-tx window, rx packet len and tx packet len */
        airMode = fp[pos++];

        HCI_TRACE_PRINTF_4(HCI_TRACE_MASK_TRACE,
                            "HCI_SYNCHRONOUS_CONNECTION_COMPLETE(%s), handle %X status %X",
                            BTRACE_RAMDATA1(BLUEFACE_TRACE_MASK_TRACE, (linkType==HCI_LINKTYPE_ACL) ?"ACL" :((linkType==HCI_LINKTYPE_SCO) ?"SCO" :"eSCO")),
                            handle,
                            status,
                            NULL
                            );

        if (status == 0)
        {
            if (hciNewHandle(bd, handle, BT_CONNECTION_TYPE_BR_SCO, BLUEFACE_BDTYPE_BR_EDR) == NULL)
            {
                HCI_TRACE_PRINTF_2(HCI_TRACE_MASK_ERROR,
                                   "!!HCI_SYNCHRONOUS_CONNECTION_COMPLETE(%s), cannot allocate handle %X",
                                   BTRACE_RAMDATA1(BLUEFACE_TRACE_MASK_TRACE, (linkType==HCI_LINKTYPE_SCO) ?"SCO" :"eSCO"),
                                   handle
                                   );
                status = HCI_ERR_NO_DESCRIPTOR;
                hciCommandDisconnect(handle, HCI_ERR_OTHER_END_TERMINATE_13);
            }
        }
        blueAPI_Handle_SCO_CON_ACT_IND(bd, handle, hciStatus(status), airMode);
    }
        break;
#endif
#endif
    case HCI_DISCONNECTION_COMPLETE:
        {
            ThciHandleDesc * hd;
            status   = fp[pos++];//fixme
            handle   = CHAR2SHORT(fp+pos); pos += 2;
            reason   = fp[pos++];

            hd = hciFindHandle(pHCI, handle);

            if (!hd)
            {
                if ( pHCI->transaction == HCI_DISCONNECT )
                {
                    pHCI->transaction = 0;
                }
                HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_ERROR,"!!HCI: HCI_DISCONNECTION_COMPLETE: could not find desc for handle %x",handle);
                break;
            }
            else
            {
                if ( ( pHCI->transaction == HCI_DISCONNECT
                /* TI CC2564 sends no ENCRYPTION_CHANGE_EVENT if encryption fails */
                || pHCI->transaction == HCI_LE_START_ENCRYPTION
                || pHCI->transaction == HCI_LE_READ_REMOTE_USED_FEATURES
                || pHCI->transaction == HCI_LE_CONNECTION_UPDATE
                ) && handle == pHCI->transactionHandle )
                {
                   pHCI->transaction = 0;
                }
            }

            if (hd->conType == BT_CONNECTION_TYPE_LE)
            {
                hciLEHandleDisconnectionComplete(handle, hciStatus(reason));
                break;
            }
#if F_BT_BREDR_SUPPORT
#if (F_BT_SCO)
            hciDiscAllScoByAcl(hd, handle, reason);
#endif
            l2cHandleHciDiscInd(handle, hciStatus(reason), hd->conType);
            hciRemoveHandle(handle);
#endif
        }
        break;

    case HCI_NO_OF_COMPLETED_PACKETS://fixme
        numhan   = fp[pos++];
        while (numhan--)
        {
            uint16_t handle,done;
            PWORD pCount;
            ThciHandleDesc * hd;
            handle = CHAR2SHORT(fp+pos); pos += 2;
            done   = CHAR2SHORT(fp+pos); pos += 2;
            hd     = hciFindHandle(pHCI, handle);
            if (!hd)
            {
                HCI_TRACE_PRINTF_2(HCI_TRACE_MASK_ERROR,"!!HCI: HCI_NO_OF_COMPLETED_PACKETS hdl %x done %d -- cannot find handle", handle, done);
                /* update global buffer count */
                pHCI->dsGlobalFreeBufferCount += done;
                hciGlobalFreeBufferCountUpdated();
                continue;
            }
            else
            {
#if F_BT_BREDR_SUPPORT
                if (hd->conType == BT_CONNECTION_TYPE_BR_ACL)
                {
                    pCount = &hd->dsAclCount;
                }
                else if (hd->conType == BT_CONNECTION_TYPE_LE)
                {
                    pCount = &hd->leDsAclCount;
                }
                else
                {
                    continue;
                }
#else
                pCount = &hd->leDsAclCount;
#endif
                if (done <= *pCount)
                {
                    *pCount -= done;
                    HCI_TRACE_PRINTF_2(HCI_TRACE_MASK_ERROR,"HCI: HCI_NO_OF_COMPLETED_PACKETS hdl %x count %d", handle, *pCount);
                }
                else
                {
                    HCI_TRACE_PRINTF_3(HCI_TRACE_MASK_ERROR,"!!HCI: HCI_NO_OF_COMPLETED_PACKETS hdl %x done %d count %d", handle, done, *pCount);
                    pHCI->dsGlobalFreeBufferCount += done - *pCount;
                    *pCount = 0;
                    hciGlobalFreeBufferCountUpdated();
                } /* else */
            } /* else */
        } /* while (numHan) */
        break;
#if F_BT_BREDR_SUPPORT
    case HCI_INQUIRY_RESULT:
    case HCI_INQUIRY_RESULT_WITH_RSSI:
    case HCI_EXTENDED_INQUIRY_RESULT:
    {
        int offsetCOD;
        int offsetClock;
        int offsetExtendedResponse;
        int offsetRSSI;
        int offsetNextResult;
        int size = 0;
        int offsetFlag;

        THCI_CONF_INQUIRY_RESULT *pInquiryResult = NULL;

        numhan = fp[pos++];
        if (!pHCI->inqBuf)
        {
            break;           /* no memory -> no work (this should not happen!) */
        }

        if(event == HCI_INQUIRY_RESULT_WITH_RSSI
        || event == HCI_EXTENDED_INQUIRY_RESULT
        )
        {
            offsetCOD   = BD_ADDR_SIZE + 2;
            offsetClock = BD_ADDR_SIZE + 5;
            offsetRSSI  = offsetClock + 2;

            if(event == HCI_EXTENDED_INQUIRY_RESULT)
            {
                offsetExtendedResponse = offsetRSSI + 1;
                offsetNextResult = offsetExtendedResponse + 240 /*EXTENDED_INQUIRY_RESPONSE_SIZE*/ + 1;
            } 
            else
            {
            offsetNextResult = offsetRSSI + 1;
            offsetExtendedResponse = 0;
            }
        }
        else
        {
            offsetCOD        = BD_ADDR_SIZE + 3;
            offsetClock      = BD_ADDR_SIZE + 6;
            offsetNextResult = offsetClock  + 2;
            offsetRSSI  = 0;
            offsetExtendedResponse = 0;
        }

        while (numhan--)
        {
            /* check for duplicates: some controllers (e.h. ERICSSON and ATMEL show same BD mre than once) */
            int i;

            for (i = 0; i < pHCI->inqBuf->count; i++)
            {
                if (memcmp(pHCI->inqBuf->bd + i * BD_ADDR_SIZE, fp + pos, BD_ADDR_SIZE) == 0)
                {
                    break;
                }
            } 
            if (i < pHCI->inqBuf->count)
            {
                /* if a duplicate exists: skip this entry, continue with the next one */
                pos += BD_ADDR_SIZE + 8;
                continue;
            }
            /* Copy one handle to inqBuf */
            if (pHCI->inqBuf->count < pHCI->inqBuf->maxHdl)
            {
                uint8_t * bd        = pHCI->inqBuf->bd + pHCI->inqBuf->count * BD_ADDR_SIZE;
                uint8_t * cls       = pHCI->inqBuf->bd + pHCI->inqBuf->maxHdl * BD_ADDR_SIZE + pHCI->inqBuf->count * BT_CLASS_SIZE;
#if 0
                uint32_t  classCode = (((uint32_t) fp[pos + offsetCOD + 2]) << 16)
                                 + (((uint32_t) fp[pos + offsetCOD + 1]) << 8)
                                 +  ((uint32_t) fp[pos + offsetCOD + 0]);

                /* check if the current inquiry result matches the active classcode filter */
                if ((classCode & pHCI->classMask) != pHCI->classValue)
                {
                    /* if the classCode is not matching, skip this entry, continue with the next one */
                    HCI_TRACE_PRINTF_0(HCI_TRACE_MASK_ERROR,"!! (SKIP) BD");
                    pos += BD_ADDR_SIZE + 8;
                    continue;
                } /* not matching class code */
#endif
                /* copy the bd addr */
                memcpy(bd, fp+pos, BD_ADDR_SIZE);
                /* copy the device class also at the end of the bd list */
                memcpy(cls, fp + pos + offsetCOD, BT_CLASS_SIZE);

                /* next address please : only if space remains */
                if (pHCI->inqBuf->count < pHCI->inqBuf->maxHdl)
                    pHCI->inqBuf->count++;
                else
                {
                    HCI_TRACE_PRINTF_0(HCI_TRACE_MASK_ERROR,"!! HCI_INQUIRY_RESULT: too much handles (SKIP)");
                }
            }

            /* send a message containing the partial result upstream */
            size = sizeof(THCI_CONF_INQUIRY_RESULT);
 
            if(event == HCI_EXTENDED_INQUIRY_RESULT)
            {
            size += 239 ;
            }
            offsetFlag = offsetExtendedResponse;

            if (!((osPoolFree(BTSystemPoolID, size) > BT_HCI_INQIURY_RESULT_LOWWATER_COUNT) &&
                (osBufferGet(BTSystemPoolID, size, (PVOID  *)&pInquiryResult) == 0)))
            {
                if(offsetExtendedResponse)
                {
                    size -= 239;
                    if(osPoolFree(BTSystemPoolID, size) > BT_HCI_INQIURY_RESULT_LOWWATER_COUNT)
                    {
                        osBufferGet(BTSystemPoolID, size, (PVOID  *)&pInquiryResult);
                    }
                    offsetFlag = 0;
                }
            }

            if(pInquiryResult != NULL)
            {
                memcpy((PVOID)(pInquiryResult->bd), (PVOID)(fp + pos),BD_ADDR_SIZE);
                memcpy((PVOID)(pInquiryResult->classOfDevice), (PVOID)(fp + pos + offsetCOD), BT_CLASS_SIZE);
 
                if (offsetRSSI)
                {
                    pInquiryResult->rssi = fp[pos + offsetRSSI];
                }
                else
                {
                    pInquiryResult->rssi = (char)0xFF;
                } 
                if(offsetFlag)
                {
                    memcpy((PVOID) pInquiryResult->extendedResponse, (PVOID)(fp + pos + offsetExtendedResponse), 240);
                }
                else
                {
                    pInquiryResult->extendedResponse[0] = 0;
                } 
                HCI_TRACE_PRINTF_2(HCI_TRACE_MASK_ERROR,"SEND HCI_CONF_INQUIRY_RESULT %s, ind %s", TRACE_BDADDR1(HCI_TRACE_MASK_ERROR,fp+pos), TRACE_BDADDR2(HCI_TRACE_MASK_ERROR, pInquiryResult->bd));

                blueAPI_Handle_HCI_INQUIRY_RESULT(pInquiryResult); 	
                pInquiryResult = NULL;
            } 

            /* insert this entry into remdev cache at remPos */
            memcpy((PVOID)(pHCI->remDev[pHCI->remPos].bd), (PVOID)(fp + pos), BD_ADDR_SIZE);
            pHCI->remDev[pHCI->remPos].pageScanRepMode = fp[pos + BD_ADDR_SIZE];
            pHCI->remDev[pHCI->remPos].pageScanMode    = 0;
            pHCI->remDev[pHCI->remPos].clockOffset     = CHAR2SHORT(fp + pos + offsetClock);

            /* advance the entry position in the local cache ("random insert") */
            if (++pHCI->remPos >= HCI_MAX_REMDEV)
            {
                pHCI->remPos = 0;
            }
            /* advance to the next entry */
            pos += offsetNextResult;
        } /* while */
        break;
        } /* case HCI_INQUIRY_RESULT */

    case HCI_INQUIRY_COMPLETE:
        pHCI->transaction = 0;   /* transaction completed */

        if (!pHCI->inqBuf)
        {
            break;
        }
        status = fp[pos++];
        numhan = fp[pos++];

        hciListenCnf(hciStatus(status));
        break;

    case HCI_LINK_KEY_REQUEST:
        {
            uint8_t * bd;
            bd = fp + pos;
            HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_TRACE, "HCI: HCI_LINK_KEY_REQUEST bd %s", TRACE_BDADDR1(HCI_TRACE_MASK_TRACE, bd));
            btsmHandleHciKeyInd(bd, LINK_KEY_REQUEST);
        }
        break;

    case HCI_PIN_CODE_REQUEST:
        {
            uint8_t * bd;
            bd = fp + pos;
            HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_TRACE, "HCI: HCI_PIN_CODE_REQUEST bd %s", TRACE_BDADDR1(HCI_TRACE_MASK_TRACE, bd));
            btsmHandleHciKeyInd(bd, PIN_CODE_REQUEST);
        }
        break;

    case HCI_AUTHENTICATION_COMPLETE:
        {
            uint8_t status;
            uint16_t handle;
            ThciHandleDesc * hd;
            status = fp[pos++];
            handle = CHAR2SHORT(fp+pos); pos += 2;
            hd     = hciFindHandle(pHCI, handle);
            if (!hd)
            {
                HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_ERROR,"!!HCI: HCI_AUTH_COMPLETE: could not find desc for handle %x",handle);
                break;
            }
            btsmHandleHciAuthConf(hd->bd, hd->tid, hciStatus(status));
            hciAuthenticationOnLinkEncrypted(hd, HCI_EA_AUTH_CONF_IND);
        }
        break;

    case HCI_LINK_KEY_NOTIFICATION:
        {
            uint8_t * bd;
            uint8_t * key;
            uint8_t   type;
            bd   = fp + pos; pos += BD_ADDR_SIZE;
            key  = fp + pos; pos += LINK_KEY_SIZE;
            type = fp[pos++];
            HCI_TRACE_PRINTF_2(HCI_TRACE_MASK_TRACE, "HCI: HCI_LINK_KEY_NOTIFICATION bd %s type %d", TRACE_BDADDR1(HCI_TRACE_MASK_TRACE, bd), type);
            btsmHandleHCI_LINK_KEY_NOTIFICATION(bd, key, type);
        }
        break;

    case HCI_REMOTE_NAME_REQUEST_COMPLETE:
        {
            uint8_t status;
            uint8_t * bd;
            uint8_t * name;
            status = fp[pos++];
            bd     = fp + pos; pos += BD_ADDR_SIZE;
            name   = fp + pos; pos += NAME_LENGTH;
            HCI_TRACE_PRINTF_2(HCI_TRACE_MASK_TRACE, "HCI: HCI_REMOTE_NAME_REQUEST_COMPLETE bd %s status %x", TRACE_BDADDR1(HCI_TRACE_MASK_TRACE, bd), status);
            blueAPI_Handle_HCI_NAME_CONF(bd, hciStatus(status), name);
            pHCI->transaction = 0; /* transaction complete */
        }
        break;

    case HCI_READ_REMOTE_FEATURES_COMPLETE:
        {
            uint8_t SSP = 0;
            uint8_t Extended = 0;
            uint16_t handle;
            ThciHandleDesc * hd;

            status = fp[pos++];
            handle = CHAR2SHORT(fp+pos); pos += 2;
            hd     = hciFindHandle(pHCI, handle);
            if (!hd)
            {
                HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_ERROR,"!!HCI: HCI_READ_REMOTE_FEATURES_COMPLETE: could not find desc for handle %x",handle);
                break;
            }
            pos     += 6;
            SSP      = fp[pos++] & LMP_FEAT_SSP; /* SSP avail in Baseband */
            Extended = fp[pos++] & LMP_FEAT_EXT_FEAT; /* Extended Features */

            if(SSP && Extended)
            {
                hciCommandWordByteParameter(HCI_READ_REMOTE_EXTENDED_FEATURES, hd->handle, 1 /* page */);
            }
            else
            {
                btsmHandleHciAclConnection(hd->bd, ACL_CONNECTION_NONSSP);
            }
        }
        break;

    case HCI_READ_REMOTE_EXTENDED_FEATURES_COMPLETE:
        {
            uint8_t SSP = 0;
            uint8_t pageNo = 0;
            uint16_t handle;
            ThciHandleDesc * hd;

            status = fp[pos++];
            handle = CHAR2SHORT(fp+pos); pos += 2;
            hd     = hciFindHandle(pHCI, handle);
            if (!hd)
            {
                HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_ERROR,"!!HCI: HCI_READ_REMOTE_EXTENDED_FEATURES_COMPLETE: could not find desc for handle %x",handle);
                break;
            }
            pageNo = fp[pos++]; /* requested page */
            pos++;
            SSP    = fp[pos++] & LMP_EXTFEAT_SSP_HOST; /* SSP avail in Hoststack  ? */
            if((pageNo == 1) && SSP)
            {
                btsmHandleHciAclConnection(hd->bd, ACL_CONNECTION_SSP);
            }
            else
            {
                btsmHandleHciAclConnection(hd->bd, ACL_CONNECTION_NONSSP);
            }
        }
        break;

    case HCI_MAX_SLOTS_CHANGE:
        {
#if ((HCI_TRACE_VERBOSITY_COUNT >= 2)||(HCI_TRACE_VERBOSITY_COUNT < 0))
            uint16_t handle;
            uint8_t maxslots;
            handle   = CHAR2SHORT(fp+pos); pos += 2;
            maxslots = fp[pos++];
            HCI_TRACE_PRINTF_2(HCI_TRACE_MASK_TRACE, "HCI: MAX_SLOTS_CHANGE handle %x maxslots %x", handle, maxslots);
#endif
        }
        break;
#endif
    case HCI_HARDWARE_ERROR://fixme
        {
            uint8_t hardware_code = fp[pos++];
            HCI_TRACE_PRINTF_2(HCI_TRACE_MASK_ERROR,"!!hciProcessEventPacket: hardware error, manu %d code %x",pHCI->manuName, hardware_code);

            if (pHCI->manuName == COMPID_UNINITIALIZED)
            {
                /* we do not know the manufacturer and the controller gave a hardware error,
                            this happens e.g. with CSR after init sequence with 115200 baud. we simply
                             retry the reset here, there is still a timeout monitoring the event
                        */
                HCI_TRACE_PRINTF_0(HCI_TRACE_MASK_ERROR,"!!hciProcessEventPacket: retrying reset");
                pHCI->numHCI = 1;                          /* allow the packet to pass */
                hciCommandNoParameter(HCI_RESET);
            }
        }
        break;

    case HCI_ENCRYPTION_CHANGE:
        {
            uint8_t status,
                 enable;
            uint16_t handle;
            ThciHandleDesc * hd = NULL;

            status = fp[pos++];
            handle = CHAR2SHORT(fp+pos); pos += 2;
            enable = fp[pos++];
            hd     = hciFindHandle(pHCI, handle);
            if (((pHCI->transaction == HCI_LE_START_ENCRYPTION)
#if F_BT_BREDR_SUPPORT
                ||(pHCI->transaction == HCI_SET_CONNECTION_ENCRYPTION)
#endif
               )
              && handle == pHCI->transactionHandle )
            {
                pHCI->transaction = 0;
            }

            if (hd && hd->conType == BT_CONNECTION_TYPE_LE)
            {
                btsmHandleHci_LE_ENCRYPTION_CHANGE(handle, enable, hciStatus(status));
                break;
            }
#if F_BT_BREDR_SUPPORT
            if (!hd)
            {
                HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_ERROR,"!!HCI: HCI_ENCRYPTION_CHANGE: could not find desc for handle %x",handle);
            }
            else
            {
                hciEncryptInd(hd->bd, hciStatus(status), enable);
            }
#endif
        }
        break;
#if F_BT_BREDR_SUPPORT
    case HCI_MODE_CHANGE:
        {
            uint8_t                status;
            uint16_t                handle, interval;
            uint8_t                currentMode;
            ThciHandleDesc      * hd; 
            THCI_MODECHANGE_IND tmc;

            status      = fp[pos++];
            handle      = CHAR2SHORT(fp+pos); pos += 2;
            currentMode = fp[pos++];
            interval    = CHAR2SHORT(fp+pos); pos += 2;
            hd          = hciFindHandle(pHCI, handle);
            if (!hd)
            {
                HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_ERROR,"!!HCI: HCI_MODE_CHANGE: could not find desc for handle %x",handle);
                break;
            }

            switch ( currentMode )
            {
            case 0: /* active mode */
                if ( status == HCI_SUCCESS ) /* premeditated */
                {
                    if ( hd->nModeState == HCI_MODE_STATE_WAKE_UP_SNIFF_PENDING )
                    {
                        hciCommandSniffMode(hd->handle, hd->sniffMax, hd->sniffMin, hd->sniffAttempt, hd->sniffTimeout);
                        hd->nModeState = HCI_MODE_STATE_SNIFF_PENDING;
                    }
                    else
                    {
                        hd->nModeState = HCI_MODE_STATE_ACTIVE;
                    }
                }
                else if ( hd->nModeState == HCI_MODE_STATE_NEW_SNIFF_PENDING )
                {
                    /* if new sniff req is pending we try one more time to enter sniff mode */
                    hciCommandSniffMode(hd->handle, hd->sniffMax, hd->sniffMin, hd->sniffAttempt, hd->sniffTimeout);
                    hd->nModeState = HCI_MODE_STATE_SNIFF_PENDING;
                    }
                else
                {
                    hd->nModeState = HCI_MODE_STATE_ACTIVE;
                }
                break;

                case 2:  /* sniff mode */
                if ( status == HCI_SUCCESS ) /* premeditated */
                {
                    if ( hd->nModeState == HCI_MODE_STATE_SNIFF_PENDING )
                    {
                        hd->nModeState = HCI_MODE_STATE_SNIFF;
                    }
                    else if ( hd->nModeState == HCI_MODE_STATE_NEW_SNIFF_PENDING )
                    { 
                        hciCommandWordParameter(HCI_EXIT_SNIFF_MODE, hd->handle);	
                        hd->nModeState = HCI_MODE_STATE_WAKE_UP_SNIFF_PENDING;
                    }
                    else
                    {
                        hd->nModeState = HCI_MODE_STATE_SNIFF;
                    }
                }
                else if ( hd->nModeState == HCI_MODE_STATE_NEW_SNIFF_PENDING )
                {
                    /* if new sniff req is pending we try one more time to enter sniff mode */ 
                    hciCommandWordParameter(HCI_EXIT_SNIFF_MODE, hd->handle);
                    hd->nModeState = HCI_MODE_STATE_WAKE_UP_SNIFF_PENDING;
                }
                else
                {
                    hd->nModeState = HCI_MODE_STATE_SNIFF;
                }
                break;
            }

            tmc.bLinkContext = NULL;
            memcpy((PVOID)tmc.bd, hd->bd, BD_ADDR_SIZE);
            tmc.status = hciStatus(status);
            tmc.mode = currentMode;
            tmc.interval = interval;
            blueAPI_Handle_HCI_MODE_CHANGE_IND(&tmc);
        }
        break;

    case HCI_SNIFF_SUBRATING_EVENT:
        {
            ThciHandleDesc              *pHd;
            TBlueAPI_ACLStatusParam aclStatusParam;

            status  = fp[pos++];
            handle  = CHAR2SHORT(fp+pos); pos += 2;

            pHd = hciFindHandle(pHCI, handle);
            if (pHd == NULL)
            {
                HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_ERROR,"!!HCI: HCI_SNIFF_SUBRATING_EVENT: could not find desc for handle %x", handle);
                break;
            }

            aclStatusParam.sniffSubrate.maxTxLatency	 = CHAR2SHORT(fp+pos); pos += 2;
            aclStatusParam.sniffSubrate.maxRxLatency	 = CHAR2SHORT(fp+pos); pos += 2;
            aclStatusParam.sniffSubrate.minRemoteTimeout = CHAR2SHORT(fp+pos); pos += 2;
            aclStatusParam.sniffSubrate.minLocalTimeout  = CHAR2SHORT(fp+pos); pos += 2;

            blueAPI_Send_ACLStatusInfo(pHd->bd,
                                        blueAPI_RemoteBDTypeClassic,
                                        blueAPI_ACLConnectedSniffSubrate,
                                        &aclStatusParam);
            }
            break;

    case HCI_ROLE_CHANGE:
        {
            uint8_t * bd;
            ThciHandleDesc *hd;

            status  = fp[pos++];
            bd      = fp + pos; pos += BD_ADDR_SIZE;

            hd = hciFindBd(bd, BT_CONNECTION_TYPE_BR_ACL);
            if (hd == (ThciHandleDesc *)0)
            {
                break;
            }
            hd->currentRole = fp[pos++];
            if (status && (hd->piconetType == HCI_PICONET_MASTER_REQUIRED || hd->piconetType == HCI_PICONET_SLAVE_REQUIRED) && hd->switchRoleTries )
            {
                if ( hd->switchRoleTries > 0 )
                {
                    hd->switchRoleTries--;
                }
                if ( hd->switchRoleTries )
                {
                    hciChangeLinkPolicyAndRole(hd);
                }
                else
                {
                    hciCommandDisconnect(hd->handle, HCI_ERR_OTHER_END_TERMINATE_13);
                }
            }
            else
            {
                hd->switchRoleTries = 0;
                if ( hd->currentRole == HCI_ROLE_MASTER)
                {
                    hciCommandWordWordParameter(HCI_WRITE_LINK_SUPERVISION_TIMEOUT, hd->handle, pHCI->link_supervision_timeout);
                }

                blueAPI_Handle_HCI_ROLE_CHANGE((uint8_t)(hd->currentRole ?ROLE_CHANGE_TO_SLAVE :ROLE_CHANGE_TO_MASTER), bd, hd->bdType);

                if ( hd->piconetType == HCI_PICONET_MASTER_REQUIRED || hd->piconetType == HCI_PICONET_SLAVE_REQUIRED )
                {
                    hciCommandWordWordParameter(HCI_WRITE_LINK_POLICY_SETTINGS, hd->handle, (uint16_t)(hd->link_policy & ~HCI_LINK_POLICY_ENABLE_SWITCH) );	  
                }
            }
        }
        break;
#endif
    case HCI_ENCRYPTION_KEY_REFRESH_COMPLETE:
        {
            uint8_t status;
            uint16_t handle;
            ThciHandleDesc * hd = NULL;

            status = fp[pos++];
            handle = CHAR2SHORT(fp+pos); pos += 2;
            hd     = hciFindHandle(pHCI, handle);
            if (pHCI->transaction == HCI_LE_START_ENCRYPTION && handle == pHCI->transactionHandle)
            {
                pHCI->transaction = 0;
            }

            if (hd && hd->conType == BT_CONNECTION_TYPE_LE)
            {
                btsmHandleHCI_ENCRYPTION_KEY_REFRESH_COMPLETE(handle, status);
                break;
            }
        }
        break;
#if F_BT_BREDR_SUPPORT
    case HCI_IO_CAPABILITY_REQUEST:
        {
            uint8_t * bd;
            bd = fp + pos; pos += BD_ADDR_SIZE;
            HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_TRACE, "HCI: HCI_IO_CAPABILITY_REQUEST bd %s", TRACE_BDADDR1(HCI_TRACE_MASK_TRACE, bd));
            btsmHandleHCI_IO_CAPABILITY_REQUEST(bd);
        }
        break;

    case HCI_IO_CAPABILITY_RESPONSE:
        {
            uint8_t * bd;
            uint8_t   ioCapability;
            uint8_t   authRequirements;
            bd     = fp + pos; pos += BD_ADDR_SIZE;
            ioCapability = fp[pos++];
            pos++;
            authRequirements = fp[pos++];
            HCI_TRACE_PRINTF_3(HCI_TRACE_MASK_TRACE, "HCI: HCI_IO_CAPABILITY_RESPONSE bd %s io %x auth %x", TRACE_BDADDR1(HCI_TRACE_MASK_TRACE, bd),
                                                                                 ioCapability, authRequirements);
            btsmHandleHCI_IO_CAPABILITY_RESPONSE(bd, ioCapability, authRequirements );
        }
        break;

    case HCI_USER_CONFIRMATION_REQUEST:
        {
            uint8_t *  bd;
            uint32_t   value;
            bd     = fp + pos; pos += BD_ADDR_SIZE;
            value  = CHAR2LONG(fp + pos); pos += 4;
            HCI_TRACE_PRINTF_2(HCI_TRACE_MASK_TRACE, "HCI: HCI_USER_CONFIRMATION_REQUEST bd %s value %d", TRACE_BDADDR1(HCI_TRACE_MASK_TRACE, bd),value);
            btsmHandleHCI_USER_CONFIRMATION_REQUEST(bd, value );
        }
        break;

    case HCI_USER_PASSKEY_REQUEST:
        {
            uint8_t * bd;
            bd = fp + pos; pos += BD_ADDR_SIZE;
            HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_TRACE, "HCI: HCI_USER_PASSKEY_REQUEST bd %s", TRACE_BDADDR1(HCI_TRACE_MASK_TRACE, bd));
            blueAPI_Send_UserPasskeyReqInd(bd);
        }
        break;

    case HCI_REMOTE_OOB_DATA_REQUEST:
        {
            uint8_t * bd;
            bd = fp + pos; pos += BD_ADDR_SIZE;
            HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_TRACE, "HCI: HCI_REMOTE_OOB_DATA_REQUEST bd %s", TRACE_BDADDR1(HCI_TRACE_MASK_TRACE, bd));
            blueAPI_Send_LegacyRemoteOOBDataReqInd((uint8_t *)bd);
        }
        break;

    case HCI_SIMPLE_PAIRING_COMPLETE:
        {
            uint8_t status;
            uint8_t * bd;
            status = fp[pos++];
            bd = fp + pos; pos += BD_ADDR_SIZE;
            HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_TRACE, "HCI: HCI_SIMPLE_PAIRING_COMPLETE bd %s", TRACE_BDADDR1(HCI_TRACE_MASK_TRACE, bd));
            btsmHandleHCI_SIMPLE_PAIRING_COMPLETE(bd, hciStatus(status));
        }
        break;

    case HCI_USER_PASSKEY_NOTIFICATION:
        {
            uint8_t *  bd;
            uint32_t   value;
            bd     = fp + pos; pos += BD_ADDR_SIZE;
            value  = CHAR2LONG(fp + pos); pos += 4;
            HCI_TRACE_PRINTF_2(HCI_TRACE_MASK_TRACE, "HCI: HCI_USER_PASSKEY_NOTIFICATION bd %s value %x", TRACE_BDADDR1(HCI_TRACE_MASK_TRACE, bd),value);
            blueAPI_Send_UserPasskeyNotificationInfo(bd, value );
        }
        break;

    case HCI_KEYPRESS_NOTIFICATION:
        {
            uint8_t *  bd;
            uint8_t  value;
            bd     = fp + pos; pos += BD_ADDR_SIZE;
            value  = fp[pos++];
            HCI_TRACE_PRINTF_2(HCI_TRACE_MASK_TRACE, "HCI: HCI_KEYPRESS_NOTIFICATION bd %s value %d", TRACE_BDADDR1(HCI_TRACE_MASK_TRACE, bd),value);
            blueAPI_Send_KeypressNotificationInfo(bd, value);
        }
        break;
#endif
    case HCI_LE_EVENT:
        hciLEProcessEventPacket(fp, pos, len + 2);
        break;

    default:
        HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_ERROR,"!!hciProcessEventPacket: unknown Event 0x%X",event);
        break;
    } /* switch event */
} /* hciProcessEventPacket */

/**
* @brief  process acl packet, send to upstream
* 
* @param  fp
* @param  foffset
* @param  flength
*
* @return  
*
*/
void hciProcessAclPacket(PHCI pHCI, uint8_t * fp, int foffset, int flength)
{
    MESSAGE_T msg;
    uint16_t      temp;
    uint16_t      handle;
    uint8_t      PB_flag;

    temp   = CHAR2SHORT(fp + foffset);
    handle = temp & 0xfff;
#if F_BT_BREDR_SUPPORT
    if (pHCI->capabilities & HCI_CAP_HOST_FLOW)
    {
        uint32_t hd = (uint32_t)hciFindHandle(pHCI, handle);
        osBufferCallBackSet(fp, hciBufferCallback, hd);
    }
#endif
    PB_flag = (temp >> 12) & 0x03;

    msg.Command                        = HCI_DATA_IND;
    msg.MData.DataCBChan.BufferAddress = fp;
    msg.MData.DataCBChan.Offset        = foffset + (ACL_HDR_LENGTH -1);
    msg.MData.DataCBChan.Length        = flength - (ACL_HDR_LENGTH -1);
    if (PB_flag == 1)
    {
        msg.MData.DataCBChan.Flag  = DATA_CB_BLOCK_MIDDLE;
    }
    else
    {
        msg.MData.DataCBChan.Flag  = DATA_CB_BLOCK_FIRST;
    }
    msg.MData.DataCBChan.Flag   |= DATA_CB_RELEASE;
    msg.MData.DataCBChan.Channel = handle;
    osMessageSend(l2cQueueID, &msg);
}

/**
* @brief  handle data request from upstream
* 
* @param
*
* @return  
*
*/
void hciHandleUpDataReq(void)
{
    uint8_t * pAclHdr;
    uint16_t handle;
    uint16_t Count;
    uint16_t temp;
    uint16_t length;
    ThciHandleDesc * hd;

    if (pHCI->Message.MData.DataCBChan.Offset < (BT_L1_HCI_DS_OFFSET + ACL_HDR_LENGTH))
    {
        HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_ERROR, "!!hciHandleUpDataReq: wrong offset %X", pHCI->Message.MData.DataCBChan.Offset);
        assert(FALSE);
        return;
    }

    handle = pHCI->Message.MData.DataCBChan.Channel & 0xfff;
    hd = hciFindHandle(pHCI, handle);
    if (!hd)
    {
        HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_ERROR, "!!hciHandleUpDataReq: handle %x not found", handle);
        if (pHCI->Message.MData.DataCBChan.Flag & DATA_CB_RELEASE)
        {
            osBufferRelease(pHCI->Message.MData.DataCBChan.BufferAddress);
        }
        return;
    }

    osMessageQueueElementCountGet(hd->dsAclQueueID, &Count);
    if (Count >= BT_DS_BUFFER_COUNT)
    {
        HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_ERROR, "!!hciHandleUpDataReq: ACL queue full (%x)", Count);
        if (pHCI->Message.MData.DataCBChan.Flag & DATA_CB_RELEASE)
        {
            osBufferRelease(pHCI->Message.MData.DataCBChan.BufferAddress);
        }
        return;
    }

    length = pHCI->Message.MData.DataCBChan.Length;
    temp   = handle;

    pHCI->Message.MData.DataCBChan.Length += ACL_HDR_LENGTH;
    pHCI->Message.MData.DataCBChan.Offset -= ACL_HDR_LENGTH;
#if UPPER_STACK_USE_VIRTUAL_HCI
    pHCI->Message.MData.DataCBChan.Pkt_type = ACL_PKT;
#endif
    pAclHdr = &pHCI->Message.MData.DataCBChan.BufferAddress[pHCI->Message.MData.DataCBChan.Offset];

    if (pHCI->Message.MData.DataCB.Flag & DATA_CB_BLOCK_MIDDLE)
    {
        temp |= 0x1000;       /* PB = 01 - continuing packet */
    }
    else
    {
        /* value of "start packet" changed in BT2.1 (introduction on non/flushable packets) */
        /* PB = 00 - start packet; PB = 10 - start flushable packet*/
        if (pHCI->Message.MData.DataCBChan.Flag & DATA_CB_NON_RELIABLE_DATA)
        {
            temp |= 0x2000;
        }
    }

    *pAclHdr = ACL_PKT;          pAclHdr++;
    SHORT2CHAR(pAclHdr, temp);   pAclHdr += 2;
    SHORT2CHAR(pAclHdr, length); pAclHdr += 2;

    pHCI->Message.Command = PH_DATA_REQ;

    osMessageSend(hd->dsAclQueueID, &pHCI->Message);
}

/**
* @brief  hci controller init
*   Control structure rule RETURN/BREAK:
*      If a command is sent to the HCI controller, use RETURN (to break the while loop)
*      If no command is send, use BREAK (to continue the search for next command)
*
* @param
*
* @return  
*
*/
void hciNextInitCommand()
{
    while(1)
    {
        HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_TRACE,"hciNextInitCommand Phase: %d", pHCI->initPhase + 1);
        switch (++pHCI->initPhase)
        {
        case HCI_INIT_STATE_REREAD_LOCAL_VERSION:
        /* re-read local version after vendor specific patches */
        hciCommandNoParameter(HCI_READ_LOCAL_VERSION_INFORMATION);
        return;

        case 15:
        hciCommandNoParameter(HCI_READ_BD_ADDR);
        return;

        case 17:
        hciCommandNoParameter(HCI_READ_LOCAL_SUPPORTED_FEATURES);
        return;
#if F_BT_BREDR_SUPPORT
        case 18:
        if (pHCI->capabilities & HCI_CAP_HOST_FLOW)
        {
            hciCommandHostBufferSize(HCI_ACL_SIZE, 0, HCI_ACL_CNT, 0);
            return;
        }
        break;

        case 19:
        if (pHCI->capabilities & HCI_CAP_HOST_FLOW)
        {
            hciCommandByteParameter(HCI_SET_HOST_CONTROLLER_TO_HOST_FLOW, 1); /* Enable Host Flow */
            return;
        }
        break;
#endif
        case 20:
        hciCommandNoParameter(HCI_READ_BUFFER_SIZE);
        return;
#if F_BT_BREDR_SUPPORT
        case 25:
        hciCommandWordWordParameter(HCI_WRITE_PAGE_SCAN_ACTIVITY,
        pHCI->pageScanActivity.interval,
        pHCI->pageScanActivity.window);
        return;

        case 28:
        if (pHCI->pageTimeout > 0)
        {
            hciCommandWordParameter(HCI_WRITE_PAGE_TIMEOUT, pHCI->pageTimeout);
            return;
        }
        break;
#endif
        case 29:
        if (pHCI->hciVersion >= HCI_VERSION_21)   /* if we are dealing with a 2.1 controler */
        {
            uint8_t mask[8];

            mask[0] = mask[1] = mask[2] = mask[3] = mask[4] = 0xFF;
            mask[5] = 0xFF;
            mask[6] = 0xFF;
            mask[7] = 0x3D; /* give me all */
            hciCommandSetEventMask(mask, FALSE);
            return;
        }
        break;

        case 40:
        {
            uint8_t mask[8] = {0};
            mask[0] = 0x1F;
            hciCommandSetEventMask(mask, TRUE);
        }
        return;

        case 41:
        hciCommandNoParameter(HCI_LE_READ_BUFFER_SIZE);
        return;


        case 42:
        hciCommandNoParameter(HCI_LE_READ_LOCAL_SUPPORTED_FEATURES);
        return;

        case 43://fixme
        /*00 simultaneous LE Host disabled*/
        /*01 LE supported Host enabled */
        hciCommandWordParameter(HCI_WRITE_LE_HOST_SUPPORTED, 0x0001);
        return;
#if F_BT_BREDR_SUPPORT
        case 53:
        hciCommandByteParameter(HCI_WRITE_INQUIRY_MODE, 0x02 ); /* inqury result with RSSI or EIR*/
        return;
#endif
        case 60:                                    /* the final initialisation state */
        hciInitCompleted();
        return;
        } /* switch */
    } /* while */
} /* hciNextInitCommand */

/**
* @brief  hci init complete
*
* @param
*
* @return  
*
*/
void hciInitCompleted(void)
{
    blueAPI_Handle_HCI_ACT_CONF(pHCI->localBdAddr, 0);
#if F_BT_BREDR_SUPPORT
    l2cHandleHciInitCompleted(pHCI->leDsAclSize, 0, pHCI->dsAclSize);
#else
    l2cHandleHciInitCompleted(pHCI->leDsAclSize, 0, 0);
#endif
    pHCI->status = HCI_STATUS_READY;
    btsmUpdateLocalAddress(pHCI->localBdAddr);
}

void hciActivateStack(void)
{
    HCI_TRACE_PRINTF_0(HCI_TRACE_MASK_TRACE, "hciActivateStack");

    pHCI->status = HCI_STATUS_RESETTING;
    pHCI->numHCI = 1;
    hciLLOpen();
    hciLaterEntry();
}

