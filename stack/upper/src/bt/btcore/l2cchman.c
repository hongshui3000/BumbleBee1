/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       l2cchman.c
* @brief     channel management functions for L2CAP protocol layer
* @details   
*
* @author   	gordon
* @date      	2015-06-29
* @version	v0.1
*/

#include <flags.h>
#include <os_message.h>
#include <os_pool.h>
#include <os_mem.h>
#include <os_timer.h>
#include <blueface.h>
#include <btcommon.h>
#include <l2c.h>
#include <swtimer.h>
#include <upper_stack_global.h>
#include <trace_binary.h>

extern PL2C pL2c;
#define TRACE_MODULE_ID     MID_BT_L2C
/**
* @brief    l2cap start hci close timer
*
* @param    id
*
* @return
*
*/
#if F_BT_BREDR_SUPPORT
void l2cStartHCICloseDelay(P_ACLPOOLDESC pHciDesc)
{
    osStartTimer(&(pHciDesc->CloseDelayTimerHandle), l2cQueueID, CLOSEDELAY_TID, pHciDesc->closeId, L2CAP_CLOSE_DELAY, swTimerCallBack);
}

/**
* @brief  l2cap stop hci close delay
*		hci won't close immediately, will start a timer.
*		here stop timer, and use this hci channel continuely
*
* @param  id:
*
* @return  
*
*/
void l2cStopHCICloseDelay(P_ACLPOOLDESC pHciDesc)
{
	osDeleteTimer(&(pHciDesc->CloseDelayTimerHandle));
}
#endif
/**
* @brief  free hci descriptor
*
* @param  pHciDesc
*
* @return  
*
*/
void
l2cFreeHciDesc(P_ACLPOOLDESC pHciDesc)
{
#if F_BT_BREDR_SUPPORT
	if(pHciDesc->closeId != 0)
	{
		l2cStopHCICloseDelay(pHciDesc);
	}
	pHciDesc->closeId = 0;
#endif
	pHciDesc->handle  = HCI_HANDLE_INVALID;
	pHciDesc->state   = aclStateFree;
	pHciDesc->uses    = 0;
	if(pHciDesc->usPakOpen.len != 0)
	{
		osBufferRelease(pHciDesc->usPakOpen.pd);
		pHciDesc->usPakOpen.pd = NULL;
	}
	pHciDesc->usPakOpen.len = 0;
}

#if F_BT_LE_BT41_SUPPORT
void l2cChanRelease(P_L2CAP_CHANNEL pChan)
{
	L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_TRACE, "l2cChanRelease(): cid 0x%x ", pChan->LocalCid);
	PL2CTxData pTxData;
    uint8_t *      pBuffer;
#if 0
	if(pPatch_upper_stack_l2cChanRelease)
	{
		return pPatch_upper_stack_l2cChanRelease(pChan);
	}
#endif	
	if (pChan->SigIdSent != 0)
    {
    	l2cStopRTXTimeout(pChan);
    }
	
    if (pChan->usLEFrames.pd != NULL)
    {
        osBufferRelease(pChan->usLEFrames.pd);
        pChan->usLEFrames.pd = NULL;
    }
    pChan->usLEFrames.len = 0;
    pChan->usLEFrames.expected = 0;
    if (pChan->dsMessage.MData.DataCBChan.BufferAddress != NULL)
    {
        osBufferRelease(pChan->dsMessage.MData.DataCBChan.BufferAddress);
        pChan->dsMessage.MData.DataCBChan.BufferAddress = NULL;
    }

    pTxData = osQueueOut( &pChan->TxDataQueue );
    while ( pTxData != NULL )
    {
        pBuffer = pTxData->pBuffer;
        osBufferRelease( pBuffer );
        osQueueIn( &pChan->TxDataQueueFree, pTxData );
        pTxData = osQueueOut( &pChan->TxDataQueue );
    }
}

#endif
        
/**
* @brief  get a descriptor for an HCI ACL channel
*
* @param  remoteBd: the Bluetooth address of the remote device
* @param  conType
*
* @return  
*
*/
P_ACLPOOLDESC
l2cGetHciDesc(LPCBdAddr remoteBd, uint8_t conType)
{
	P_ACLPOOLDESC ret_desc = NULL;
    int i;

    for(i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_don; i++)    /* first check if we already have a descriptor in use for that bd */
	{
	    if(pL2c->pAclDescDon[i].state != aclStateFree)
	    {
			if(pL2c->pAclDescDon[i].conType == conType &&
			   memcmp(remoteBd, pL2c->pAclDescDon[i].remote_bd, sizeof(TBdAddr)) == 0)
			{
#if F_BT_BREDR_SUPPORT
			    if(pL2c->pAclDescDon[i].closeId != 0)
			    {
			        l2cStopHCICloseDelay(&pL2c->pAclDescDon[i]);
			        pL2c->pAclDescDon[i].closeId = 0;
			    }
#endif
			    pL2c->pAclDescDon[i].uses++;
			    return &pL2c->pAclDescDon[i];       /* found a descriptor */
			}
	    }
	}
    for(i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_doff; i++)    /* first check if we already have a descriptor in use for that bd */
	{
	    if(pL2c->pAclDescDoff[i].state != aclStateFree)
	    {
			if(pL2c->pAclDescDoff[i].conType == conType &&
			   memcmp(remoteBd, pL2c->pAclDescDoff[i].remote_bd, sizeof(TBdAddr)) == 0)
			{
#if F_BT_BREDR_SUPPORT
			    if(pL2c->pAclDescDoff[i].closeId != 0)
			    {
			        l2cStopHCICloseDelay(&pL2c->pAclDescDoff[i]);
			        pL2c->pAclDescDoff[i].closeId = 0;
			    }
#endif
			    pL2c->pAclDescDoff[i].uses++;
			    return &pL2c->pAclDescDoff[i];       /* found a descriptor */
			}
	    }
	}

	for(i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_don; i++)    /* now search for a free descriptor */
	{
		if(pL2c->pAclDescDon[i].state == aclStateFree)
		{
			ret_desc = &pL2c->pAclDescDon[i];
			goto TagFound2;
		}
	}
    for(i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_doff; i++)    /* now search for a free descriptor */
	{
		if(pL2c->pAclDescDoff[i].state == aclStateFree)
		{
			ret_desc = &pL2c->pAclDescDoff[i];
			break;
		}
	}

TagFound2:

	if (ret_desc != NULL)
	{
	   /* desriptor found now initializing */
	   ret_desc->state   = aclStateAllocated;
	   ret_desc->handle  = HCI_HANDLE_INVALID;
	   ret_desc->uses    = 1;
 	   ret_desc->usPakOpen.pd = NULL;
	   ret_desc->usPakOpen.expected = ret_desc->usPakOpen.len = 0;
	   ret_desc->conType = conType; 
       ret_desc->role      = 0;
	   ret_desc->SigIdSent = 0;
       memcpy(ret_desc->remote_bd, remoteBd, sizeof(TBdAddr)); 
#if F_BT_BREDR_SUPPORT       
	   ret_desc->mtu     = 0;                /* we do not have a valid MTU by now */
	   ret_desc->FlushTO = FLUSHTIMEOUT_DEFAULT;
	   ret_desc->remoteExtendedFeatures = 0;
       ret_desc->CloseDelayTimerHandle = 0;
#endif
       ret_desc->RTXTimerHandle = 0;
	}

	return ret_desc;
}

/**
* @brief  l2cap layer get hci descriptor using closeid
*
* @param  CloseId: used to identify the HCI descriptor
*
* @return  
*
*/
#if F_BT_BREDR_SUPPORT
P_ACLPOOLDESC
l2cSearchHciCloseId(uint8_t closeID)
{
    int i;

    for(i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_don; i++)
    {
        if(pL2c->pAclDescDon[i].closeId == closeID)
        {
            if(pL2c->pAclDescDon[i].uses == 0)      /* still not in use */
            {
                return &pL2c->pAclDescDon[i];
            }
        }
    }
    for(i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_doff; i++)
    {
        if(pL2c->pAclDescDoff[i].closeId == closeID)
        {
            if(pL2c->pAclDescDoff[i].uses == 0)      /* still not in use */
            {
                return &pL2c->pAclDescDoff[i];
            }
        }
    }

    return NULL;
}
#endif
/**
* @brief		get acl descriptor by handle
*
* @param	hdl
*
* @return
*
*/
P_ACLPOOLDESC
l2cSearchHciDescByHciHandle(uint16_t hdl)
{
	int i;

	for(i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_don; i++)
	{
	    if(pL2c->pAclDescDon[i].handle == hdl)
	    {
	        return &pL2c->pAclDescDon[i];
	    }
	}
	for(i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_doff; i++)
	{
	    if(pL2c->pAclDescDoff[i].handle == hdl)
	    {
	        return &pL2c->pAclDescDoff[i];
	    }
	}

    return NULL;
}

/**
* @brief		get l2cap acl descriptor by sig id sent
*
* @param	SigIdSent
*
* @return
*
*/

P_ACLPOOLDESC
l2cSearchHciDescBySigIdSent(uint8_t SigIdSent)
{
    int i;

    for(i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_don; i++)
    {
        if(pL2c->pAclDescDon[i].SigIdSent == SigIdSent)
        {
            return &pL2c->pAclDescDon[i];
        }
    }
    for(i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_doff; i++)
    {
        if(pL2c->pAclDescDoff[i].SigIdSent == SigIdSent)
        {
            return &pL2c->pAclDescDoff[i];
        }
    }


    return NULL;
}

#if (F_BT_L2C_ENHANCED_FEATURE_SUPPORT)
/**
* @brief  l2cap get free ext data
*
* @param
*
* @return  
*
*/
P_L2CAP_CHANNEL_EXT l2cGetFreeExtFData(void)
{
    int i;

    for (i = 0; i < L2CAP_MAX_ENHANCED_FEATURE_CHANNELS; i++)
    {
      if (!pL2c->ExtF[i].Used)
      {
        P_L2CAP_CHANNEL_EXT pExtF;
        uint8_t        TxQueueID;
        uint8_t        SentQueueID;
        uint8_t        RxQueueID;

        pExtF = &pL2c->ExtF[i];

        TxQueueID   = pExtF->TxQueueID;      /* save queue id */
        SentQueueID = pExtF->SentQueueID;
        RxQueueID   = pExtF->RxQueueID;

        memset(pExtF, 0, sizeof(T_L2CAP_CHANNEL_EXT));

        pExtF->TxQueueID   = TxQueueID;
        pExtF->SentQueueID = SentQueueID;
        pExtF->RxQueueID   = RxQueueID;

        pExtF->Used = TRUE;
        return(pExtF);
      }
    }

    return((P_L2CAP_CHANNEL_EXT)0);
}

/**
* @brief  l2cap delete ext data
*
* @param  pChan
*
* @return  
*
*/
void l2cDeleteExtFData(P_L2CAP_CHANNEL pChan)
{
    if (pChan->pExtF != (P_L2CAP_CHANNEL_EXT)0)
    {
        pChan->pExtF->Used = FALSE;
        pChan->pExtF       = (P_L2CAP_CHANNEL_EXT)0;
    }
}

/**
* @brief  update flush timeout
*
* @param  pHciDesc: hci descriptor
*
* @return  
*
*/
void l2cUpdateFlushTO(P_ACLPOOLDESC pHciDesc)
{
    int  i;
    uint16_t FlushTO = FLUSHTIMEOUT_DEFAULT;

    for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_l2c_chan_don; i++)
    {
        if (pL2c->pLLDataChanDon[i].pHciDesc == pHciDesc &&
            pL2c->pLLDataChanDon[i].ConfParaLocal.FlushTO != FLUSHTIMEOUT_DEFAULT)
        {
            FlushTO = pL2c->pLLDataChanDon[i].ConfParaLocal.FlushTO;
            goto TagFound1;
        }
    }
    
    for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_l2c_chan_doff; i++)
    {
        if (pL2c->pLLDataChanDoff[i].pHciDesc == pHciDesc &&
            pL2c->pLLDataChanDoff[i].ConfParaLocal.FlushTO != FLUSHTIMEOUT_DEFAULT)
        {
            FlushTO = pL2c->pLLDataChanDoff[i].ConfParaLocal.FlushTO;
            break;
        }
    }
    
TagFound1:
    
    if (pHciDesc->FlushTO != FlushTO)
    {
        pHciDesc->FlushTO = FlushTO;
        l2cSendLHciWriteAutomaticFlushTimeout(pHciDesc);
    }
}
#endif

#if F_BT_LE_BT41_SUPPORT
void l2cInitChannelList()
{
	int i;
	uint8_t tx_wsize;
	P_L2CAP_CHANNEL   pDataChan;

    pL2c->ChannelListHdr.First = NULL;      /* init channel list */
    pL2c->ChannelListHdr.Last  = NULL;
    pL2c->ChannelListHdr.Count = 0;

    pL2c->pChannelListCur   = NULL;
    //pL2c->LCidSearchStart   = CID_MIN;

	if(otp_str_data.gEfuse_UpperStack_s.le_data_max_tx_wsize != 0)
	{
		tx_wsize = otp_str_data.gEfuse_UpperStack_s.le_data_max_tx_wsize;

		for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_l2c_chan_don; i++)
    	{
        	pDataChan = &pL2c->pLLDataChanDon[i];

        	pDataChan->pTxData = osMemoryAllocate(RAM_TYPE_DATA_ON, sizeof(TL2CTxData)*tx_wsize);        
    	}

		for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_l2c_chan_doff; i++)
    	{
        	pDataChan = &pL2c->pLLDataChanDoff[i];

        	pDataChan->pTxData = osMemoryAllocate(RAM_TYPE_DATA_OFF, sizeof(TL2CTxData)*tx_wsize);
#if 0
            if(FALSE == DLPS_BUFFER_REG((UINT8*)&pDataChan->pTxData, sizeof(pDataChan->pTxData), FALSE))
        	{
            	L2CAP_TRACE_PRINTF_0(GATT_TRACE_MASK_ERROR,"L2C: DLPS_BUFFER_REG fail");

        	}
#endif
    	}
	}
}
#endif           

/**
* @brief  get cid from CID_MIN - CID_MAX
*
* @param
*
* @return  
*
*/
#if (F_BT_BREDR_SUPPORT)||(F_BT_LE_BT41_SUPPORT)
uint16_t l2cGetCid(uint8_t conType)
{
	uint16_t cid;
    uint16_t cid_max;
    if(conType == BT_CONNECTION_TYPE_LE)
    {
        cid_max = CID_LE_MAX;
    }
    else
    {
        cid_max = CID_MAX;
    }
	for(cid = CID_MIN; cid < cid_max; cid++)
	{	
		if(l2cSearchLcid(cid) == NULL)
		{ 	/* free CID */
			return cid;
		}
	}
	return 0;
}

/**
* @brief  create l2cap channel
*
* @param  rcid: 
* @param  remoteBd: rmote bdaddr
* @param  conType: connection type
*
* @return  
*
*/
P_L2CAP_CHANNEL l2cChannelCreate(uint16_t rcid, LPCBdAddr remoteBd, uint8_t conType)
{
    P_L2CAP_CHANNEL pNewCh = NULL;
    int             i;
    uint16_t            lcid;
#if F_BT_LE_BT41_SUPPORT
    PL2CTxData      pTxData = NULL;
#endif
    int             maxChanNum = otp_str_data.gEfuse_UpperStack_s.num_l2c_chan_don + otp_str_data.gEfuse_UpperStack_s.num_l2c_chan_doff;

    if(pL2c->ChannelListHdr.Count == maxChanNum)
    {
        L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_ERROR,"!!Connection list full!");
        return NULL;
    }
    /* Add check if already in list ?? or rely on that otherwhere ?*/

    for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_l2c_chan_don; i++)
    {
        if (pL2c->pLLDataChanDon[i].magic == 0)
        {
            pNewCh = &pL2c->pLLDataChanDon[i];
            break;
        }
    }
    if (pNewCh == NULL)
    {
        for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_l2c_chan_doff; i++)
        {
            if (pL2c->pLLDataChanDoff[i].magic == 0)
            {
                pNewCh = &pL2c->pLLDataChanDoff[i];
                break;
            }
        }
    }
    if (pNewCh == NULL)
    {
        L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_ERROR, "!!l2cChannelCreate:failed!");
        return NULL;
    }   
#if F_BT_LE_BT41_SUPPORT
    memset(pNewCh, 0, sizeof(T_L2CAP_CHANNEL)-sizeof(PL2CTxData)-2*sizeof(QUEUE_T));  
#else
    memset(pNewCh, 0, sizeof(T_L2CAP_CHANNEL) );
#endif

    pNewCh->pHciDesc = l2cGetHciDesc(remoteBd, conType);
    if(pNewCh->pHciDesc == NULL)
    {
        return NULL;
    }

    lcid = l2cGetCid(conType);

    pNewCh->State           = l2cStateClosed;
    pNewCh->role            = undef;                 /* not defined yet */
    pNewCh->magic           = L2CAP_CHANNEL_MAGIC;   /* channel is used */
    pNewCh->LocalCid        = lcid;
    pNewCh->RemoteCid       = rcid;
#if (F_BT_BREDR_SUPPORT)
    pNewCh->LocalMtu        = MTU_DEFAULT;
    pNewCh->RemoteMtu       = MTU_DEFAULT;

    if (conType == BT_CONNECTION_TYPE_LE)
    {
    	pNewCh->LocalUsMtu = L2CAP_SIGNAL_MTU_LE;
    }
    else
    {
    	pNewCh->LocalUsMtu = L2CAP_SIGNAL_MTU;
    }
    pNewCh->RemoteFlushTimeout = FLUSHTIMEOUT_DEFAULT;
    pNewCh->ACLDiscImmediate   = FALSE;
    pNewCh->OpenL2cDiscConf    = FALSE;
    //pNewCh->ReqIdRcv        = 0;
    pNewCh->Mode            = L2CAP_MODE_BASIC;

    /* (F_BT_L2C_QOS_SUPPORT) */
    //pNewCh->UnacceptableMode = 0;
    pNewCh->ConfParaLocal.flowControl.mode  = L2C_MODE_BASIC;
    pNewCh->ConfParaRemote.flowControl.mode = L2C_MODE_BASIC;
    pNewCh->ConfParaLocal.FlushTO = FLUSHTIMEOUT_DEFAULT;

    pNewCh->ConfParaLocal.flowControl.maxPDUSize  = BT_US_PDU_L2C_BYTE_COUNT;
    pNewCh->ConfParaRemote.flowControl.maxPDUSize = BT_DS_PDU_L2C_BYTE_COUNT;
    pNewCh->ConfParaRemote.mtuSize = MTU_DEFAULT;
    //pNewCh->ConfParaLocal.fcs  = 0;     /*champion 1->0*/     /* default 16 bit FCS */
    //pNewCh->ConfParaRemote.fcs = 0;     /*champion 1->0*/      /* default 16 bit FCS */
#endif
    //pNewCh->RTXTimerHandle = 0;
    //pNewCh->CONFIGTimerHandle = 0;

	//pNewCh->pNext = NULL;
#if F_BT_LE_BT41_SUPPORT
    pNewCh->TxDataQueueFree.Count = 0;
    pNewCh->TxDataQueueFree.First= NULL;
    pNewCh->TxDataQueueFree.Last= NULL;
        
    pNewCh->TxDataQueue.Count = 0;
    pNewCh->TxDataQueue.First= NULL;
    pNewCh->TxDataQueue.Last= NULL;

    pTxData = &pNewCh->pTxData[0];

    if(otp_str_data.gEfuse_UpperStack_s.le_data_max_tx_wsize != 0)
    {
        uint8_t tx_wsize = otp_str_data.gEfuse_UpperStack_s.le_data_max_tx_wsize;
		for ( i = 0; i < tx_wsize; i++ )
    	{
    		osQueueIn( &pNewCh->TxDataQueueFree, pTxData );
        	pTxData++;
    	}
    }
#endif
    return pNewCh;
}

/**
* @brief  insert channel to ChannelListHdr
*
* @param  pNewChannnel: 
*
* @return  
*
*/
void l2cInsertChannel(P_L2CAP_CHANNEL pNewChannnel)
{
    osQueueIn(&pL2c->ChannelListHdr, pNewChannnel);
    pL2c->pChannelListCur = pNewChannnel; /* Set current to new element */
} 

/**
* @brief	delete l2cap channel
*
* @param	pChannel
* @param	HCIDisconnected
*
* @return
*
*/
int l2cDeleteChannel(P_L2CAP_CHANNEL pChannel, BOOL HCIDisconnected)
{
    if(pL2c->ChannelListHdr.Count != 0)
    {
        osQueueDelete(&pL2c->ChannelListHdr, pChannel);
        pL2c->pChannelListCur = /*lint -e(826)*/(P_L2CAP_CHANNEL)pL2c->ChannelListHdr.First;

        pChannel->pHciDesc->uses--;
#if F_BT_LE_BT41_SUPPORT
        if(pChannel->pHciDesc->conType == BT_CONNECTION_TYPE_LE)
        {
            l2cChanRelease(pChannel);
        }
#endif
#if (F_BT_BREDR_SUPPORT)
        if(pChannel->pHciDesc->conType != BT_CONNECTION_TYPE_LE)
        {

            if(pChannel->pHciDesc->handle == HCI_HANDLE_INVALID                /* HCI channel not yet opened */
               || l2cSearchHciHandle(pChannel->pHciDesc->handle) == NULL) /* last channel on this HCI handle */
            {
    			if(HCIDisconnected == FALSE) 
    			{      /* the HCI chan has not already been closed by the controler */
    				if(pChannel->ACLDiscImmediate == TRUE)
    				{
    					l2cRemoveHCI(pChannel->pHciDesc, 0);
    				}
    				else
    				{
    					pChannel->pHciDesc->closeId = (uint8_t)pChannel->LocalCid;
    					l2cStartHCICloseDelay(pChannel->pHciDesc);
    				}
    			}
    			else    /* the HCI chan has already been closed by the controller so free the HCI desc */
    			{
    				l2cFreeHciDesc(pChannel->pHciDesc);
    			}
            }

#if (F_BT_L2C_ENHANCED_FEATURE_SUPPORT) //champion
            if (pChannel->pExtF != NULL)
            {
                l2cStateIsClosed(pChannel);
                l2cDeleteExtFData(pChannel);
            }
#endif

            pChannel->ConfParaLocal.FlushTO = FLUSHTIMEOUT_DEFAULT;
#if (F_BT_L2C_ENHANCED_FEATURE_SUPPORT) 
            if (pChannel->Mode == L2CAP_MODE_STREAMING && pChannel->pHciDesc->uses)
    		{
    			l2cUpdateFlushTO(pChannel->pHciDesc);
    		}
#endif		

            if (pChannel->authenticationActive)
            {
                pChannel->authenticationActive = 0;
                l2cSendSECMAN_AUTHENTICATION_IND(pChannel, 0 /* outgoing */, 0 /* active */);
            }
        }
        
#endif
        pChannel->magic = 0;   /* channel is free */

        L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_TRACE,"l2cDeleteChannel(): End");
        return 0;
    }
    return -2;
} /* l2cDeleteChannel */

/**
* @brief  Searches list for local cid and returns channel if found
*
* @param  lcid
*
* @return  
*
*/
P_L2CAP_CHANNEL l2cSearchLcid(uint16_t lcid)
{
    int i = 0;

    while(i < pL2c->ChannelListHdr.Count )
    {
        if(lcid == pL2c->pChannelListCur->LocalCid)
        {
            return pL2c->pChannelListCur;
        }

        /* Not yet found, keep on searching */
        pL2c->pChannelListCur =
            pL2c->pChannelListCur->pNext != NULL ?
                pL2c->pChannelListCur->pNext : /*lint -e(826)*/(P_L2CAP_CHANNEL) pL2c->ChannelListHdr.First;
        i++;
    }
    
    L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_TRACE, "l2cSearchLcid(): lcid 0x%X not found", lcid);

    return NULL;
}



/**
* @brief  Searches list for last request command ID and returns channel if found 
*
* @param  lcid
*
* @return  
*
*/
P_L2CAP_CHANNEL l2cSearchCmdId(uint8_t cmdid)
{
    int i = 0;

    while(i < pL2c->ChannelListHdr.Count )
    {
        if(cmdid == pL2c->pChannelListCur->SigIdSent) 
        {
            return pL2c->pChannelListCur;
        }

        /* Not yet found, keep on searching */
        pL2c->pChannelListCur =
            pL2c->pChannelListCur->pNext != NULL ?
                pL2c->pChannelListCur->pNext : /*lint -e(826)*/(P_L2CAP_CHANNEL) pL2c->ChannelListHdr.First;
        i++;
    }
    L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_TRACE, "l2cSearchCmdId(): not found");

    return NULL;
}

/**
* @brief get l2cap channel using hci handle
*
* @param  hciHdl: hci handle
*
* @return  l2cap found or null
*
*/

P_L2CAP_CHANNEL l2cSearchHciHandle(uint16_t hciHdl)
{
    uint16_t i = 0;

    while(i < pL2c->ChannelListHdr.Count )
    {
        if(hciHdl == pL2c->pChannelListCur->pHciDesc->handle)	/*found*/
        {
            return pL2c->pChannelListCur;
        }

        /* Not yet found, keep on searching */
        pL2c->pChannelListCur =
            pL2c->pChannelListCur->pNext != NULL ?
            pL2c->pChannelListCur->pNext : /*lint -e(826)*/(P_L2CAP_CHANNEL) pL2c->ChannelListHdr.First;
        i++;
    }

    return NULL;
}
#endif
/**
* @brief  Searches list for remote bd addr and calls the callback function if channel found 
*		if callback is null, just return found l2cap channel
*
* @param  bd: 
* @param  cb
* @param  pPar
*
* @return  
*
*/
#if F_BT_BREDR_SUPPORT
P_L2CAP_CHANNEL l2cSearchBdCallback(TCBdAddr bd, Channel_found_callback cb, PCVOID pPar)
{
	uint16_t i = 0;
	uint16_t initialElemCount = pL2c->ChannelListHdr.Count;
	P_L2CAP_CHANNEL pNextChan;

	while(i < initialElemCount) {
	    L2CAP_TRACE_PRINTF_2(L2CAP_TRACE_MASK_TRACE,"l2c_search_bd_cb(): bd %s HCIstate %d",
	                                  TRACE_BDADDR1(L2CAP_TRACE_MASK_TRACE, pL2c->pChannelListCur->pHciDesc->remote_bd),
	                                  pL2c->pChannelListCur->pHciDesc->state
	                                  );

	    /* get the element behind the current one, must be done here because the current one may
	       be deleted in the call back. */
	    pNextChan =
	        pL2c->pChannelListCur->pNext != NULL ?
	            pL2c->pChannelListCur->pNext : /*lint -e(826)*/(P_L2CAP_CHANNEL) pL2c->ChannelListHdr.First;

		if(memcmp(bd, pL2c->pChannelListCur->pHciDesc->remote_bd, sizeof(TBdAddr)) == 0)
		{
			if(cb)
			{
				if (pL2c->pChannelListCur->pHciDesc->conType == BT_CONNECTION_TYPE_BR_ACL)
				{
					cb(pL2c->pChannelListCur, pPar);
				}

			}else
			{
				return pL2c->pChannelListCur;
			}
		}

	    pL2c->pChannelListCur = pNextChan;
	    i++;
	}
			return NULL;
}
#endif

#if F_BT_LE_BT41_SUPPORT
P_L2CAP_CHANNEL l2cSearchRcid(PC_ACLPOOLDESC pHciDesc, uint16_t rcid)
{
    int i = 0;

    L2CAP_TRACE_PRINTF_2(L2CAP_TRACE_MASK_TRACE, "l2cSearchRcid(): HCI handle 0x%X remote_cid 0x%x", pHciDesc->handle, rcid);

    while (i < pL2c->ChannelListHdr.Count)
    {
        if (pL2c->pChannelListCur->pHciDesc == pHciDesc)
        {
            if (pL2c->pChannelListCur->RemoteCid == rcid)
            {
                return pL2c->pChannelListCur;
            }
        }
        pL2c->pChannelListCur =
            pL2c->pChannelListCur->pNext != NULL ?
            pL2c->pChannelListCur->pNext : (P_L2CAP_CHANNEL) pL2c->ChannelListHdr.First;
        i++;
    }

    L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_TRACE, "l2cSearchRcid(): Rcid 0x%X not found", rcid);

    return NULL;
}
#endif
/**
* @brief  find l2cap channel by cid, if not, allcate one
* 	called only on receiption off L2CAP_CONNECTION_REQUEST - function relys on this restriction
*  	Searches list based on hci descriptor, if found it sets remote cid
*  	and returns the channel. If hci handle is found but channel
*  	not yet 'active' it simply returns the found con.
*  	If previously in use, a new l2cap channel is created over the same
*  	hci handle.
*
* @param  pHciDesc
* @param  remote_cid
*
* @return  
*
*/
#if (F_BT_BREDR_SUPPORT)||(F_BT_LE_BT41_SUPPORT)
P_L2CAP_CHANNEL l2cCheckRcid(PC_ACLPOOLDESC pHciDesc, uint16_t remote_cid)
{
    int i = 0;
    P_L2CAP_CHANNEL pNewCh = NULL;

    while (i < pL2c->ChannelListHdr.Count) 
	{

        if(pL2c->pChannelListCur->pHciDesc == pHciDesc) 
		{
			if ((pL2c->pChannelListCur->RemoteCid == 0) &&
			(pL2c->pChannelListCur->role == terminate)) 
			{
				/* This is the first incoming l2cap channel over this hci handle
				(remote_cid is only set to 0 when receiving a new or closing down a
				baseband connection) */

				pL2c->pChannelListCur->RemoteCid = remote_cid;
				return pL2c->pChannelListCur;
			}
			else if(pL2c->pChannelListCur->RemoteCid == remote_cid) 
			{
				/* it seems that this L2Cap channel already exists, so just return the channel handle */
				return pL2c->pChannelListCur;
			}
        }
        /* Not yet found, keep on searching */
        pL2c->pChannelListCur =
            pL2c->pChannelListCur->pNext != NULL ?
                pL2c->pChannelListCur->pNext : /*lint -e(826)*/(P_L2CAP_CHANNEL) pL2c->ChannelListHdr.First;
        i++;
    }

    /* we have not found a matching channel, create a new one */

    pNewCh = l2cChannelCreate(remote_cid, (LPCBdAddr)pHciDesc->remote_bd, pHciDesc->conType);

    if(pNewCh) 
    {
        l2cInsertChannel(pNewCh);
    }
    return pNewCh;
}
#endif
/**
* @brief  l2cap check if flush timeout valid
*
* @param  pChan: 
* @param  FlushTO: flush timeout
*
* @return  
*
*/
#if F_BT_BREDR_SUPPORT
int l2cCheckFlushTO(PC_L2CAP_CHANNEL pChan, uint16_t FlushTO)
{
    P_ACLPOOLDESC pHciDesc = pChan->pHciDesc;               /* pointer to the HCI ACL connection descriptor */
    int           i;

    if (FlushTO == FLUSHTIMEOUT_DEFAULT)
	{
    	return(0);                              /* value is compatible */
	}

    for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_l2c_chan_don; i++)
    {
        if (pL2c->pLLDataChanDon[i].pHciDesc == pHciDesc &&
            pL2c->pLLDataChanDon[i].ConfParaLocal.FlushTO != FLUSHTIMEOUT_DEFAULT &&
            pL2c->pLLDataChanDon[i].ConfParaLocal.FlushTO != FlushTO)
        {
            return(-1);                        /* value is incompatible */
        }
    }
    for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_l2c_chan_doff; i++)
    {
        if (pL2c->pLLDataChanDoff[i].pHciDesc == pHciDesc &&
            pL2c->pLLDataChanDoff[i].ConfParaLocal.FlushTO != FLUSHTIMEOUT_DEFAULT &&
            pL2c->pLLDataChanDoff[i].ConfParaLocal.FlushTO != FlushTO)
        {
            return(-1);                        /* value is incompatible */
        }
    }
    
    return(0);                                 /* value is compatible */
}
#endif
