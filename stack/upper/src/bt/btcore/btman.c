/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       btman.c
* @brief     bt manager control
* @details   
*
* @author  	gordon
* @date      	2015-06-29
* @version	v0.1
*/

#include <flags.h>
#include <os_message.h>
#include <os_pool.h>
#include <os_mem.h>
#include <blueface.h>
#include <btcommon.h>
#include <bt_api.h>
#include <blueapi_types.h>
#include <btman.h>
#include <btsend.h>
#include <btglib.h>
#include <bluemgr.h>
#include <hci_api.h>
#include <l2c_api.h>
#include <sdp_api.h>
#include <btsm_api.h>
#include <gatt_api.h>
#include <blueapi_def.h>
#include <bluemgr.h>
#include <upper_stack_global.h>

#define TRACE_MODULE_ID     MID_BTSECMAN
extern PBTA pBTA;

#if (T_CHECK_UPSTREAM_DATA) || (T_CHECK_DOWNSTREAM_DATA)
void showLastBytes( LPSTR info, uint32_t prevVal, uint32_t crrVal, uint16_t i, uint8_t * pData, uint32_t dwLen)
{
    int j=i-20;
    int k,len;
    char bufftmp[3], buff[3*20+1];

    if ( j<0 ) j=0;
    if ( (uint32_t)j+20 >= dwLen )
    {
        len = dwLen-j-1;
    }
    else
    {
        len = 20;
    }
    buff[0]=0;
    for ( k=0; k<len; k++ )
    {
        sprintf(bufftmp, "%.2X ", pData[j+k]);
        strcat(buff, bufftmp);
    }

    debugOut(dbError, "%s: crr(%.8d) != prev(%.8d)+1: %s",
        info,
        crrVal,
        prevVal,
        buff
        );
}
#endif

#if (T_CHECK_UPSTREAM_DATA)
void checkUpStream( LPbtLink pLink, uint8_t * pData, uint32_t dwLen)
{
    uint16_t i;
    BOOL  b=FALSE;

    for ( i=0; i<dwLen; i++ )
    {
        if ( pData[i] == 0x0D )
        {
            continue;
        }
        else if ( pData[i] == 0x0A )
        {
            if ( pLink->DUcrrVal != (pLink->DUprevVal+1) )
            {
                showLastBytes( "DU", pLink->DUprevVal, pLink->DUcrrVal, i, pData, dwLen);
                b = TRUE;
            }
            pLink->DUprevVal = pLink->DUcrrVal;
            pLink->DUcrrVal  = 0;
            continue;
        }
        else
        {
            pLink->DUcrrVal *= 10;
            pLink->DUcrrVal += (pData[i] - 0x30);
        }
    }

    if ( b ) {
        debugOut( dbError, "!DU: %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X",
            pData[0],
            pData[1],
            pData[2],
            pData[3],
            pData[4],
            pData[5],
            pData[6],
            pData[7],
            pData[8],
            pData[9],
            pData[10]
            );
        debugOut( dbError, "!-----------------------");
    }
}
#endif

#if (T_CHECK_DOWNSTREAM_DATA)
void checkDownStream( LPbtLink pLink, uint8_t * pData, uint32_t dwLen)
{
    uint16_t i;
    BOOL  b=FALSE;

    for ( i=0; i<dwLen; i++ )
    {
        if ( pData[i] == 0x0D )
        {
            continue;
        }

        else if ( pData[i] == 0x0A )
        {
            if ( pLink->DDcrrVal != (pLink->DDprevVal+1) )
            {
                showLastBytes( "DD", pLink->DDprevVal, pLink->DDcrrVal, i, pData, dwLen);
                b = TRUE;
            }
            pLink->DDprevVal = pLink->DDcrrVal;
            pLink->DDcrrVal  = 0;
            continue;
        }
        else
        {
            pLink->DDcrrVal *= 10;
            pLink->DDcrrVal += (pData[i] - 0x30);
        }
    }

    if ( b ) {
        debugOut( dbError, "!DD: %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X",
            pData[0],
            pData[1],
            pData[2],
            pData[3],
            pData[4],
            pData[5],
            pData[6],
            pData[7],
            pData[8],
            pData[9],
            pData[10]
            );
        debugOut( dbError, "!-----------------------");
    }
}
#endif

/**
 * @brief  init blueface
 * @param
 * @return
 *
 */
void lblueFaceInit(void)
{
    pBTA = osMemoryClearAllocate(RAM_TYPE_DATA_ON, sizeof(TBTA));

    pBTA->localCause = 0xffff;           /* Init is not passed through */
    osCoroutineCreate(&btaQueueID, (TCoroutineEntry)NULL, (void *)pBTA);
    osMessageQueueCreate(&pBTA->btReqQueueID);

    pBTA->pLinksDon = osMemoryAllocate(RAM_TYPE_DATA_ON, sizeof(TbtLink)*otp_str_data.gEfuse_UpperStack_s.num_link_don);
    pBTA->pLinksDoff = osMemoryAllocate(RAM_TYPE_DATA_OFF, sizeof(TbtLink)*otp_str_data.gEfuse_UpperStack_s.num_link_doff);

    BLUEFACE_TRACE_PRINTF_4(BLUEFACE_TRACE_MASK_TRACE, "lblueFaceInit: BlueFace Version %x.%02x sizeof(TblueFaceMsg) %d",
        BLUEFACE_MAJOR_VERSION, BLUEFACE_MINOR_VERSION, sizeof(TblueFaceMsg),0);
}

/**
 * @brief  get queue ID from psm
 * @param  psm
 *
 */
uint8_t blueFacePSM2hQueue(uint16_t psm)
{
    switch(psm)
    {
    case BLUEFACE_PSM_SDP:
        return sdpQueueID;

    case BLUEFACE_PSM_GATT:
        return gattQueueID;

#if (F_BT_SCO)
    case BLUEFACE_PSM_SCO:
        return hciQueueID;
#endif

    default:
        BLUEFACE_TRACE_PRINTF_1(BLUEFACE_TRACE_MASK_ERROR,
            "!!BTMAN: blueFacePSM2hQueue: unknown PSM %X", psm);
        return 0;
    }
}

/**
 * @brief  blueface handle bt connect indicate
 * @param  link
 * @param  app
 * @return  
 *
 */
void blueFaceHandleConInd(LPbtLink link, LPCbtApplication app)
{
    /* send CON_IND to next application in sequence. IF app if NULL, then this
       is the first call, otherwise a subsequenct call.
       Search order is: secApp first (if it exists), then all apps in order
       if there are no more apps after app, reject the call
    */

    BOOL active = (app == pBTA->secApp);

    if (app == NULL && pBTA->secApp)
    {
        /* first call and secman exists: send to secman first */
        link->app = pBTA->secApp;
        blueFaceSendBT_CON_IND(link);
        return;
    }

    if (active && pBTA->app.used && ((LPbtApplication)(&pBTA->app) != pBTA->secApp))
    {
        /* found one application (which is not secMan):
            send message and stop working for now
            */
        link->app = &pBTA->app;
        blueFaceSendBT_CON_IND(link);
        return;
    }

    /* no application found: reject the incoming call and release the descriptor */
    switch (link->psm)
    {
    case BLUEFACE_PSM_GATT:
        btgSendConResp(blueFacePSM2hQueue(link->psm), link->handle, L2CAP_ERR_REFUS_INV_PSM, NULL);
        break;

    default:
        BLUEFACE_TRACE_PRINTF_1(BLUEFACE_TRACE_MASK_ERROR,"!!BTMAN: CON_RESP, unhandld PSM %X", link->psm);
        break;
    }

    blueFaceDeallocateLink(link);
} /* blueFaceHandleConInd */

/**
* @brief  blueface handle bt connect indicate
* @param  psm
* @param  pConInd
* @return  
*
*/
void blueFaceHandleBtConInd_2(uint16_t psm, PBtConInd  pConInd)
{
    LPbtLink   link;

    BLUEFACE_TRACE_PRINTF_4(BLUEFACE_TRACE_MASK_TRACE,
                            "BTMAN: CON_IND psm 0x%X cid 0x%X channel 0x%X bd %s",
                            psm, pConInd->cid, pConInd->channel,
                            TRACE_BDADDR1(BLUEFACE_TRACE_MASK_TRACE, pConInd->bd)
                            );

    link = blueFaceAllocateLink();     /* get a descriptor for this connection */
    if (link == NULL)
    {
        BLUEFACE_TRACE_PRINTF_1(BLUEFACE_TRACE_MASK_ERROR,"!!BTMAN: CON_IND: could not allocate descriptor for link cid %x", pConInd->cid);
        btgSendConResp(blueFacePSM2hQueue(psm), pConInd->cid, L2CAP_ERR_REFUS_NO_RESOURCE /* reject */, NULL);
        return;
    }
    link->handle        = pConInd->cid;
    link->psm           = psm;
    memcpy(link->bd, pConInd->bd, BD_ADDR_SIZE);

    link->state         = link_Incoming;
    link->encryptState  = ENCRYPT_IDLE;

    switch (psm)
    {
    case BLUEFACE_PSM_GATT:
        link->connIndParams.gatt.bdType  = pConInd->p.gatt.bdType;
        link->connIndParams.gatt.mtuSize = pConInd->p.gatt.mtuSize;
        break;

    default:
        break;
    }

    /* search for first registered application and ask if it wants to take it... */
    blueFaceHandleConInd(link, NULL);
} /* blueFaceHandleBtConInd */

/**
* @brief  allocate bta link
* @param  void
* @return  
*
*/
LPbtLink blueFaceAllocateLink(void)
{
    LPbtLink link;
    uint8_t i;

    for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_don; i++)
    {
        link = &pBTA->pLinksDon[i];
        if (!link->used)
        {
            memset(link, 0, sizeof(TbtLink));   /* clear the descriptor */
            link->magic[0]         = 'L';
            link->magic[1]         = 'I';
            link->used             = TRUE;
            link->selfIndex        = i; //gordon
            link->handle           = 0xffff;    /* undefined value */
            link->dataExt          = FALSE;

            /* we do not have do to null initializers here! (thanks to memset) */
            return link;
        }
    }

    for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_doff; i++)
    {
        link = &pBTA->pLinksDoff[i];
        if (!link->used)
        {
            memset(link, 0, sizeof(TbtLink));   /* clear the descriptor */
            link->magic[0]         = 'L';
            link->magic[1]         = 'I';
            link->used             = TRUE;
            link->selfIndex        = i + 0x10;
            link->handle           = 0xffff;    /* undefined value */
            link->dataExt          = FALSE;

            /* we do not have do to null initializers here! (thanks to memset) */
            return link;
        }
    }

    return NULL;
} /* blueFaceAllocateLink */

/**
* @brief	 deallocate link
*
* @param	pLink:
*
* @return
*
*/
void blueFaceDeallocateLink(LPbtLink pLink)
{
    /* Deallocate the link, it will not be used any more, destroy everything to prevent
           inaccidental reuse of dangling references
        */
    pLink->used     = FALSE;
    pLink->app      = NULL;
    pLink->magic[0] = 0;
    pLink->magic[1] = 0;
} /* blueFaceDeallocateLink */

/**
* @brief	 find link by handle
*
* @param	handle
* @param	psm: 
*
* @return
*
*/
LPbtLink blueFaceFindLinkByHandle(uint16_t handle, uint16_t psm)
{
    int i;
    LPbtLink ll;

    for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_don; i++)
    {
        ll = &pBTA->pLinksDon[i];
        if (ll->used && ll->handle == handle && ll->psm == psm)
        {
            return ll;
        }
    }

    for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_doff; i++)
    {
        ll = &pBTA->pLinksDoff[i];
        if (ll->used && ll->handle == handle && ll->psm == psm)
        {
            return ll;
        }
    }

    return NULL;
} /* blueFaceFindLinkByHandle */

/**
* @brief	 find link by bdaddr
*
* @param	bd
*
* @return
*
*/
LPbtLink blueFaceFindLinkByBD(LPCBYTE bd)
{
    int i;
    LPbtLink ll;
    LPbtLink resultLink = NULL;

    /* we search a link from a BD. there might be several links related to one BD. in this
           case we give priority to the RFCOMM control channel....
        */
    for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_don; i++)
    {
        ll = &pBTA->pLinksDon[i];

        if (ll->used && ll->app && memcmp(bd, ll->bd, BD_ADDR_SIZE) == 0)
        {
            /* do not overwrite activeauth links */
            if (resultLink && resultLink->activeAuth)
            {
                break;
            }

            if (ll->activeAuth)
            {
                resultLink = ll;
            }
        }
    }

    for (i = 0;i < otp_str_data.gEfuse_UpperStack_s.num_link_doff; i++)
    {
        ll = &pBTA->pLinksDoff[i];

        if (ll->used && ll->app && memcmp(bd, ll->bd, BD_ADDR_SIZE)==0)
        {
            /* do not overwrite activeauth links */
            if (resultLink && resultLink->activeAuth)
            {
                break;
            }

            if (ll->activeAuth)
            {
                resultLink = ll;
            }
        }
    }

    return resultLink;
} /* blueFaceFindLinkByBD */

#if (F_BT_SCO)
/**
 * @brief	 find link by address and psm
 *
 * @param	bd
 * @param	psm
 *
 * @return
 *
 */
LPbtLink blueFaceFindLinkByBDandPSM(LPCBYTE bd, uint16_t psm)
{
    int i;
    LPbtLink ll;

    for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_don; i++)
    {
        ll = &pBTA->pLinksDon[i];
        if (ll->used && (memcmp(bd, ll->bd, BD_ADDR_SIZE)==0) && ll->psm == psm)
        {
            return ll;
        }
    }

    for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_doff; i++)
    {
        ll = &pBTA->pLinksDoff[i];
        if (ll->used && (memcmp(bd, ll->bd, BD_ADDR_SIZE)==0) && ll->psm == psm)
        {
            return ll;
        }
    }

    return NULL;
}
#endif

/**
* @brief release active link semaphore
*
* @param  void
*
* @return  
*
*/
void blueFaceReleaseActiveLinkSemaphore(void)
{
    MESSAGE_T msg;

    pBTA->activeLink = NULL;

    while (pBTA->activeLink == NULL)
    {
        if (osMessageReceive(pBTA->btReqQueueID, &msg))
        {
            break;
        }
        /*only connect req will queue in the btReqQueueID*/
        btApiBT_HandleConReq((LPbtApplication)(msg.MData.BlueFace.MessageReq.ApplHandle), (LPblueFaceMsg)msg.MData.BlueFace.MessageReq.DataCB.BufferAddress);
    }   
}

