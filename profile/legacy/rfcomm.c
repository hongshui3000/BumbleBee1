/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       rfcomm.c
* @brief     rfcomm layer
* @details   
*
* @author   	gordon
* @date      	2015-06-29
* @version	v0.1
* @date      	2015-12-01
* @version	v0.2
*/

#define X_TEST_AGG_FLOW       0    /* experimental, leave off, use this to support agg flow */
#define X_GENERATE_COLLISION  0    /* experimental, leave off, use this to provoke DISC collision */

#include <flags.h>

//#include <os.h>
#include <crc8ets.h>
#include <rfc_code.h>
#include "rfcomm.h"
#include "rfc_api.h"
#include <blueapi.h>
//#include <rtl_memory.h>
#include <trace.h>
#include <os_mem.h>
#include <os_timer.h>
#include <common_defs.h>
//#include "l2c_api.h"
//#include "btsend.h"
//#include "btsm_api.h"
//#include <swtimer.h>
//#include "stosif.h"
//#include "hci_api.h"
//#include "../app/blueapi/blueapi_api.h"
//#include <trace_binary.h>
//#include "upper_stack_global.h"


//#define TRACE_MODULE_ID     MID_BT_RFC
enum { __FILE_NUM__ = 1 };

#define L2CAP_HDR_LENGTH 4

PRFC pRFC;
/** protocol descriptor array, which is used to find callback function when handle rfc message */
PRfcProDesc pRfcProDes;

void rfcSendPNCommand(TrfcChannel * tc);
void rfcSendSecManAuthenticationInd(PTrfcChannel tc, uint8_t outgoing, uint8_t active);


/**
* @brief  init rfcomm
* 
* @param DataSegment: pRFC memory
* @param TaskName: task Name
*
* @return  init result
*
*/
/** max supported protocol number, opp/hfp/vendor define */
#define MAX_RFCPRO_NUM       3 //use  otp_str_data instead

bool rfcInit(void)
{
	int i;
	TrfcChannel * lc;
	int queueID;

	pRFC = (PRFC)osMemoryAllocate(RAM_TYPE_DATA_ON, sizeof(TRFC));
	if (pRFC == NULL)
    {
        return false;
    }
    pRfcProDes =  (PRfcProDesc)osMemoryAllocate(RAM_TYPE_DATA_ON, sizeof(TRfcProDesc)*MAX_RFCPRO_NUM);
	if (pRfcProDes == NULL)
    {
        osMemoryFree(pRFC);
        return false;
    }
	memset(pRFC, 0, sizeof(TRFC));
	memset(pRfcProDes, 0, sizeof(TRfcProDesc)*MAX_RFCPRO_NUM);
	
	pRFC->writeOffset  = (ACL_HDR_LENGTH + L2CAP_HDR_LENGTH);
	pRFC->creditSupport= TRUE;			/*support to use credit*/
	pRFC->rfcchannelDon = (TrfcChannel *)osMemoryAllocate(RAM_TYPE_DATA_ON, sizeof(TrfcChannel)*otp_str_data.gEfuse_UpperStack_s.num_data_rfcom_dlci_don);
	if(pRFC->rfcchannelDon == NULL)
	{
		osMemoryFree(pRFC);
		osMemoryFree(pRfcProDes);
		return false;
	}
	pRFC->rfcchannelDoff = (TrfcChannel *)osMemoryAllocate(RAM_TYPE_DATA_OFF, sizeof(TrfcChannel)*otp_str_data.gEfuse_UpperStack_s.num_data_rfcom_dlci_doff);
	if(pRFC->rfcchannelDoff == NULL)
	{
		osMemoryFree(pRFC);
		osMemoryFree(pRfcProDes);
		osMemoryFree(pRFC->rfcchannelDon );
		return false;
	}

    for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_data_rfcom_dlci_don; i++)
    {
        lc = &pRFC->rfcchannelDon[i];
        lc->used   = FALSE;
        lc->handle = i+1;           /* do not use 0 as handle value ! */
    //    osMessageQueueCreate(&lc->dsQueueID);
	lc->dsQueueID= xQueueCreate(RFCOMM_DSQUEUE_NUM, sizeof(TRfcUpDataMsg));
	lc->TimerHandleT1 = 0;
	lc->TimerHandleT2 = 0;
    }

    for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_data_rfcom_dlci_doff; i++)
    {
        lc = &pRFC->rfcchannelDoff[i];
        lc->used   = FALSE;
        lc->handle = i+1;           /* do not use 0 as handle value ! */
     //   osMessageQueueCreate(&lc->dsQueueID);
     lc->dsQueueID= xQueueCreate(RFCOMM_DSQUEUE_NUM, sizeof(TRfcUpDataMsg));
        lc->TimerHandleT1 = 0;
        lc->TimerHandleT2 = 0;
    }

	queueID = mpa_RegisterProtocol(PSM_RFCOMM,rfcCallBack);
	if(queueID == -1)
	{
		/*get protocol queue id fail*/
		osMemoryFree(pRFC);
		osMemoryFree(pRfcProDes);
		osMemoryFree(pRFC->rfcchannelDon);
		osMemoryFree(pRFC->rfcchannelDoff);
		return false;
	}
	pRFC->QueueID = queueID;
	mpa_Sendl2cProtocolRegister(PSM_RFCOMM, pRFC->QueueID, 1);
	mpa_RegisterRFCAuthenticationCb(rfcAuthenticationRspCallBack);
	
	return true;
}

RfcResult rfcRegister( uint16_t  uuid, uint8_t* pChanneID,pRfcProCallback callback)
{

	uint8_t index = 0;
	PRfcProDesc tempDesc = NULL;
	if(pRFC==NULL)
	{	
		if(!rfcInit())
			return RFC_NO_RESOURCE;
	}

	for (index = 0; index < MAX_RFCPRO_NUM; index++)
	{
		tempDesc = &pRfcProDes[index];
	       if (tempDesc->uuid == 0)
	        {
			tempDesc->uuid = uuid;
			tempDesc->server_channel= index*2+3;
	     		tempDesc->cb= callback;	
			*pChanneID = tempDesc->server_channel;
	            return RFC_SUCCESS;
	        }
	        else if (tempDesc->uuid == uuid)
	        {
			tempDesc->cb = callback;
			*pChanneID = tempDesc->server_channel;
			return RFC_SUCCESS;
			   
	        }
    }
    return RFC_NO_RESOURCE;
}




/**
* @brief  rfcomm free channel
*		set used flag and reset timer 
*
* @param  tc
*
* @return  
*
*/
void rfcFreeChannel(TrfcChannel * tc)
{
	tc->used = FALSE;
	rfcResetTimer(RFCOMM_T1_ID, tc);
	rfcResetTimer(RFCOMM_T2_ID, tc);
}

void rfcTimerCallBack(void * xTimer)
{
  //  MESSAGE_T Message;
uint8_t QueueID;
uint8_t TimerID;
uint16_t TimerChannel;

  //  osGetTimerID(&xTimer, &Message.MData.Timer.QueueID,
   //     &Message.MData.Timer.TimerID, &Message.MData.Timer.TimerChannel);

    //osMessageSend(swQueueID, &Message);


   osGetTimerID(&xTimer, &QueueID,&TimerID, &TimerChannel);
   rfcHandleTimer(TimerChannel, TimerID);
   osDeleteTimer(&xTimer);
}

/**
* @brief  set rfcomm timer
* 
* @param  timerID
* @param  tc
* @param  seconds: timeout (seconds)
*
* @return  
*
*/
#if 1
void rfcSetTimer( uint8_t timerID, TrfcChannel * tc, uint16_t seconds)
{
    void ** ptimerhandle = 0;
    switch(timerID)
    {
    case RFCOMM_T1_ID:
        ptimerhandle = &(tc->TimerHandleT1);
        break;

    case RFCOMM_T2_ID:
        ptimerhandle = &(tc->TimerHandleT2);
        break;

    default:
        return;
    }

    if (*ptimerhandle)
    {
       osDeleteTimer(ptimerhandle);
    }
    osStartTimer(ptimerhandle, pRFC->QueueID, timerID, tc->handle, seconds*1000, rfcTimerCallBack);
}

#else
void rfcT1TimerCallback(xTimerHandle xTimer)
{
	uint16_t handle;
	handle= (portTickType) pvTimerGetTimerID( xTimer );
	rfcHandleTimer(handle,RFCOMM_T1_ID);

}
void rfcT2TimerCallback(xTimerHandle xTimer)
{
	uint16_t handle;
	handle=(portTickType) pvTimerGetTimerID( xTimer );
	rfcHandleTimer(handle,RFCOMM_T2_ID);

}

void rfcSetTimer( uint8_t timerID, TrfcChannel * tc, uint16_t seconds)
{
    void ** ptimerhandle = 0;
    switch(timerID)
    {
    case RFCOMM_T1_ID:
       ptimerhandle = &(tc->TimerHandleT1);
	if (*ptimerhandle)
   	{
       	xTimerDelete((xTimerHandle)*ptimerhandle, (portTickType)0) ;
    	}
	ptimerhandle=  xTimerCreate(NULL,
				(RFCOMM_T1_TIMEOUT*1000/portTICK_RATE_MS), pdFALSE, (void *)tc->handle, rfcT1TimerCallback);
	xTimerStart((xTimerHandle)*ptimerhandle, (portTickType)0) ;	
        break;

    case RFCOMM_T2_ID:
        ptimerhandle = &(tc->TimerHandleT2);
	if (*ptimerhandle)
   	{
       	xTimerDelete((xTimerHandle)*ptimerhandle, (portTickType)0) ;
	}
	ptimerhandle=  xTimerCreate(NULL,
				(RFCOMM_T1_TIMEOUT*1000/portTICK_RATE_MS), pdFALSE, (void *)tc->handle, rfcT2TimerCallback);
	xTimerStart((xTimerHandle)*ptimerhandle, (portTickType)0) ;
        break;

    default:
        return;
    }  
	
}
#endif
/**
* @brief  reset rfcomm timer
* 
* @param  timerID
* @param  tc
*
* @return  
*
*/
void rfcResetTimer( uint8_t timerID, TrfcChannel * tc)
{
    void ** ptimerhandle = 0;

    switch(timerID)
    {
    case RFCOMM_T1_ID:
        ptimerhandle = &(tc->TimerHandleT1);
        break;

    case RFCOMM_T2_ID:
        ptimerhandle = &(tc->TimerHandleT2);
        break;

    default:
        return;
    }

    if (*ptimerhandle)
    {
       osDeleteTimer(ptimerhandle);
    }
}
/**
* @brief  rfcomm handle timer
* 
* @param  chan
* @param  id
*
* @return  
*
*/
void rfcHandleTimer(uint8_t chan, uint8_t id)
{
	TrfcChannel * tc;
	DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "rfcHandleTimer: timeout id %X channel %X", 2,id, chan);

	/* Note: all timeout are channel related */
	tc = rfcFindHandle(chan);
	if (!tc)
	{
		DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "!!rfcHandleTimer: could not access channel %X id %X",2, chan, id);
		return;
	}
	if (!tc->used)  /* channel already free: timeout is a noop */
	{
		return;
	}

	if (id == RFCOMM_T1_ID)
	{
		rfcCloseChannels(tc->cc, RFCOMM_ERR | RFCOMM_ERR_TIMEOUT); /* Close user traffic channels on this link */
		//l2cHandleBTG_DISC_REQ(tc->cc->channel, FALSE);
		mpa_Sendl2cDiscReq(tc->cc->channel);
	}
	else if (id == RFCOMM_T2_ID)
	{
		/* timeout in waiting for MSC_IND from remote site, ignore this protocol */
		/* violation and process the normal stuff                                */
    /*    blueAPI_Handle_RFC_CON_ACT_IND(
                    				tc->maxUpstreamBuffers,
                    				tc->handle,
                    				tc->frameSize,
                    				tc->convergenceLayer == RFCOMM_CONVERGENCE_CREDIT_CONF
                    				);
     */
     	rfcNotifyUpstream(RFC_CONNECT_COMPLETE, RFCOMM_SUCCESS,tc,NULL);
      
	}
}

/**
* @brief  allocate rfcomm channel
*
* @param  pRFC
*
* @return  
*
*/
TrfcChannel * rfcAllocateDLCI(void)
{
    int i;		
	for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_data_rfcom_dlci_don; i++)
	{
		if (!pRFC->rfcchannelDon[i].used)
		{
			pRFC->rfcchannelDon[i].used    = TRUE;
			pRFC->rfcchannelDon[i].channel = 0;   /* no channel assigned so far */
			return &pRFC->rfcchannelDon[i];
		}
	}

	for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_data_rfcom_dlci_doff; i++)
	{
		if (!pRFC->rfcchannelDoff[i].used)
		{
			pRFC->rfcchannelDoff[i].used    = TRUE;
			pRFC->rfcchannelDoff[i].channel = 0;   /* no channel assigned so far */
			return &pRFC->rfcchannelDoff[i];
		}
	}
	DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "!!rfcAllocateDLCI: could not allocate DLCI", 0,0);	
	return NULL;
}

/**
* @brief  init rfcomm channel
*
* @param  tc: 
* @param  dlci
*
* @return  
*
*/
void rfcInitDLCI(TrfcChannel * tc,uint8_t dlci)
{
	/* Link level */
	tc->linitiator         = FALSE;
	tc->lstate             = linkIdle;
	tc->cc                 = NULL;
	tc->maxUpstreamBuffers = 0;

	/* DLCI level */
	tc->windowSize       = 0;
	tc->convergenceLayer = pRFC->creditSupport ? RFCOMM_CONVERGENCE_CREDIT_REQ : RFCOMM_CONVERGENCE_0;
	tc->initiator        = FALSE;
	tc->dlci             = dlci;
	tc->frameSize        = RFCOMM_N1;
	tc->N2               = RFCOMM_N2;
	tc->priority         = 0; /* not used in RFCOMM (dlci == 0) ? 0 : (((dlci / 8) * 8) + 7); ref GSM 07.10 p.42 */
	tc->state            = dlciIdle;
	tc->T1               = RFCOMM_T1;
	tc->used             = TRUE;
	tc->frameType        = RFCOMM_INFO_UIH;
	tc->usFlow           = 0;
	tc->usFlowBreak      = 0;
	tc->usFlowActive		= 0;
	tc->dsFlow           = 0;
	tc->mtuSize          = 0;
	tc->mscHandshake     = 0;     /* no MSC handshake done on this link */
	tc->remainingCredits = 0;

	tc->rls              = 0;
	tc->rpn.dlci         = (dlci << 2) | 2 | EA_BIT;
	tc->rpn.baudrate     = 0x07; /* 115200 Baud */
	tc->rpn.oc3          = 0x03; /* charformat 8N1 */
	tc->rpn.flctl        = 0x00; /* no flow control */
	tc->rpn.xon          = 0x11; /* Xon */
	tc->rpn.xoff         = 0x13; /* Xoff */
	tc->rpn.pm1          = 0x00; /* mask */
	tc->rpn.pm2          = 0x00; /* mask */
}

/**
* @brief  rfcomm allocate buffers
*		just set window size and back credits	
*
* @param  tc: 
* @param  maxCredits
*
* @return  
*
*/
void rfcAllocateBuffers(TrfcChannel * tc, uint16_t maxCredits)
{
	TrfcChannel * lc;
	int totalReserved = 0;
	int i;

#if 0
    for (i = 0; i < RFCOMM_MAX_DLCI; i++)
    {
        lc = &tc->pRFC->cl[i];
        if (lc != tc && lc->used)
    	{
        	totalReserved += lc->maxUpstreamBuffers;
    	}
    } 
#else
	for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_data_rfcom_dlci_don; i++)
	{
		lc = &pRFC->rfcchannelDon[i];
		if (lc != tc && lc->used)
		{
			totalReserved += lc->maxUpstreamBuffers;
		}
	}

	for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_data_rfcom_dlci_doff; i++)
	{
		lc = &pRFC->rfcchannelDoff[i];
		if (lc != tc && lc->used)
		{
			totalReserved += lc->maxUpstreamBuffers;
		}
	}
#endif
    /* there are some experiences behind this code:
    no of buffers over the air interface is critical to performance
    */
#if (BT_RFCOMM_CREDIT_COUNT > 0)
	tc->maxUpstreamBuffers = BT_RFCOMM_CREDIT_COUNT;
#else
	{
		int remaining;

		remaining = BT_US_BUFFER_COUNT - totalReserved;
		if (remaining >= 20)
		{
			tc->maxUpstreamBuffers = remaining - 6;
		}
		else if (remaining >= 6)
		{
			tc->maxUpstreamBuffers = remaining - 3;
		}
		else
		{
			tc->maxUpstreamBuffers = 1;
		}
	}
#endif

	/* If the upper layer sets a limit to the number of credits used, use this limit */
	if (maxCredits && tc->maxUpstreamBuffers > maxCredits)
	{
		tc->maxUpstreamBuffers = maxCredits;
	}
	DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "RFC: resv %d this %d total %d",4,
							totalReserved, tc->maxUpstreamBuffers, BT_US_BUFFER_COUNT, 0);

	if (tc->convergenceLayer == RFCOMM_CONVERGENCE_CREDIT_REQ || tc->convergenceLayer == RFCOMM_CONVERGENCE_CREDIT_CONF)
	{
		/* the number of credits signalled initially to the peer is 7 (by protocol definition)
		if we have more then that, we signal the remaining credits using returnCredits
		*/
		if (tc->maxUpstreamBuffers > 7)
		{
			tc->windowSize   = 7;
		}
		else
		{
			tc->windowSize   = ( uint8_t)(tc->maxUpstreamBuffers);
		}
		tc->backCredits      = tc->maxUpstreamBuffers - tc->windowSize;
	}
} /* rfcAllocateBuffers */

/**
* @brief  get rfcomm channel by dlci
*
* @param  pRFC: 
* @param  dlci
* @param  channel
*
* @return  
*
*/
TrfcChannel * rfcFindDLCI( uint8_t dlci, uint16_t channel)
{
	int i;

#if 0
	for (i = 0; i < RFCOMM_MAX_DLCI; i++)
	{
		if (pRFC->cl[i].used && pRFC->cl[i].dlci == dlci && pRFC->cl[i].cc->channel == channel)
		{
			return &pRFC->cl[i];
		}
	}
#else
	for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_data_rfcom_dlci_don; i++)
	{
		if (pRFC->rfcchannelDon[i].used && pRFC->rfcchannelDon[i].dlci == dlci && pRFC->rfcchannelDon[i].cc->channel == channel)
		{
			return &pRFC->rfcchannelDon[i];
		}
	}

	for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_data_rfcom_dlci_doff; i++)
	{
		if (pRFC->rfcchannelDoff[i].used && pRFC->rfcchannelDoff[i].dlci == dlci && pRFC->rfcchannelDoff[i].cc->channel == channel)
		{
			return &pRFC->rfcchannelDoff[i];
		}
	}
#endif
	DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "rfcFindDLCI: cannot find DLCI %X on CHANNEL %X ", dlci, channel);
	return NULL;
}

/**
* @brief  get rfcomm channel by handle
*
* @param  pRFC: 
* @param  handle: handle
*
* @return  
*
*/
TrfcChannel * rfcFindHandle(uint16_t handle)
{
#if 0	
    if (handle >= 1 && handle <= RFCOMM_MAX_DLCI)
	{
   		return &pRFC->cl[handle-1];
	}
	 RFCOMM_TRACE_VERBOSITY_1(RFCOMM_TRACE_PRINTF_1(RFCOMM_TRACE_MASK_ERROR,"!!rfcFindHandle: cannot find handle %X", handle));
   return NULL;
#else	
	if(handle&0x10)
		return &pRFC->rfcchannelDon[handle-1-0x10];
	else
		return &pRFC->rfcchannelDoff[handle-1];

#endif	
}

/**
* @brief  get rfcomm channel by bdaddr
*
* @param  pRFC: 
* @param  bd
* @param  dlci
*
* @return  
*
*/
TrfcChannel * rfcFindBd(TBdAddr bd,uint8_t dlci)
{
	int i;
	TrfcChannel * tc;

#if 0
	for (i = 0; i < RFCOMM_MAX_DLCI; i++)
	{
		tc = &pRFC->cl[i];
		/* note: we have to use the bd of the cc (not tc itself, it does not contain a bd */
		if (tc->used && tc->dlci == dlci && memcmp(tc->cc->bd, bd, BD_ADDR_SIZE) == 0)
		{
			return tc;
		}
	}
#else
	for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_data_rfcom_dlci_don; i++)
	{
		tc = &pRFC->rfcchannelDon[i];
		/* note: we have to use the bd of the cc (not tc itself, it does not contain a bd */
		if (tc->used && tc->dlci == dlci && memcmp(tc->cc->bd, bd, BD_ADDR_SIZE) == 0)
		{
			return tc;
		}
	}
	for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_data_rfcom_dlci_doff; i++)
	{
		tc = &pRFC->rfcchannelDoff[i];
		/* note: we have to use the bd of the cc (not tc itself, it does not contain a bd */
		if (tc->used && tc->dlci == dlci && memcmp(tc->cc->bd, bd, BD_ADDR_SIZE) == 0)
		{
			return tc;
		}
	}
#endif
	DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "rfcFindBd: cannot find bd %s dlci %d", 2,bd, dlci);
	return NULL;
}

/**
* @brief   Close all User Channels on Control Link cc
*
* @param  cc: 
* @param  status
*
* @return  
*
*/
void rfcCloseChannels(TrfcChannel * cc,uint16_t status)
{
	int i;
	TrfcChannel * tc;
	
	for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_data_rfcom_dlci_don; i++)
	{
		tc = &pRFC->rfcchannelDon[i];
		if (tc->used && tc->cc == cc) /*find all the user channel in this session*/
		{
			switch (tc->state)
			{
			case dlciIdle: /* nothing to do */
				break;

			default:
				if (tc->authenticationActive)
				{
					tc->authenticationActive = 0;
					rfcSendSecManAuthenticationInd(tc, 0 /* outgoing */, 0 /* active */);
				}

				if (tc->dlci)/*this is a user channel*/
				{
					//blueAPI_Handle_RFC_DISC_IND(tc->handle, tc->dlci, status);
					rfcNotifyUpstream(RFC_DISC_IND,status,tc,NULL);
				}
				break;
			} /* switch */

			rfcResetTimer(RFCOMM_T1_ID, tc);
			rfcCast(tc, dlciIdle);
			if (tc->dlci)             /* Control link is closed by l2cap operations */
			{
				rfcFreeChannel(tc);
				DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "rfcCloseChannels: free handle %x ", 1, i+1);
			}
		} /* if used */
	} /* for all */	

	for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_data_rfcom_dlci_doff; i++)
	{
		tc = &pRFC->rfcchannelDoff[i];
		if (tc->used && tc->cc == cc)
		{
			switch (tc->state)
			{
			case dlciIdle: /* nothing to do */
				break;

			default:
				if (tc->authenticationActive)
				{
					tc->authenticationActive = 0;
					rfcSendSecManAuthenticationInd(tc, 0 /* outgoing */, 0 /* active */);
				}

				if (tc->dlci)
				{
					//blueAPI_Handle_RFC_DISC_IND(tc->handle, tc->dlci, status);
					rfcNotifyUpstream(RFC_DISC_IND,status,tc,NULL);
				}
				break;
			} /* switch */

			rfcResetTimer(RFCOMM_T1_ID, tc);
			rfcCast(tc, dlciIdle);
			if (tc->dlci)             /* Control link is closed by l2cap operations */
			{
				rfcFreeChannel(tc);
				DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "rfcCloseChannels: free handle %x ", 1, i+1);

			}
		} /* if used */
	} /* for all */
	rfcDownstreamWork();
} /* rfcCloseChannels */

/**
* @brief   disconnect if no user channel
*
* @param  cc: 
*
* @return  
*
*/
void rfcDisconnectIfNoUserChannels(TrfcChannel * cc)
{
	int i;
	TrfcChannel * lc;
	int openCount = 0;

#if 0	
	/* look if there are any more open user channels on this control channel */
	for (i = 0; i < RFCOMM_MAX_DLCI; i++)
	{
		lc = &cc->pRFC->cl[i];
		if (lc->used && lc->cc == cc && lc != cc && lc->state != dlciIdle)
		{
			openCount++;
		}
	} 
#else
	for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_data_rfcom_dlci_don; i++)
	{
		lc = &pRFC->rfcchannelDon[i];
		if (lc->used && lc->cc == cc && lc != cc && lc->state != dlciIdle)  /*if cc is control channel*/
		{
			openCount++;
		}
	}

	for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_data_rfcom_dlci_doff; i++)
	{
		lc = &pRFC->rfcchannelDoff[i];
		if (lc->used && lc->cc == cc && lc != cc && lc->state != dlciIdle)
		{
			openCount++;
		}
	}
#endif

	if (openCount==0)/*if cc is user channel, openCount shall be 0*/
	{
		/* es existieren keine weiteren offenen USER verbindungen
		auf dieser rfcomm session --> schliesse den mux kanal
		*/
		DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "RFC: last link closed, closing mux channel",0,0);
		rfcCast(cc, dlciDisconnecting);
		rfcSetTimer(RFCOMM_T1_ID, cc, RFCOMM_T1_TIMEOUT);
		rfcSendDISC(cc, TRUE /* COMMAND */, TRUE /* POLL */);
	}
}

/**
* @brief  set rfcomm state
*
* @param  tc: 
* @param  state
*
* @return  
*
*/
void rfcCast(TrfcChannel * tc,uint8_t state)
{
	DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "rfcCast : Cast DLCI %X (handle %X) to state %X",
							4,tc->dlci, tc->handle, state, 0);
	tc->state = (TdlciState)state;
}

/**
* @brief  set rfcomm l2cap state
*
* @param  tc: 
* @param  state
*
* @return  
*
*/
void rfcLCast(TrfcChannel * cc, uint8_t state)
{
	DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "rfcLCast : Cast channel %X to state %X",2,cc->channel, state);
	cc->lstate = (TAcllinkState)state;
}

/**
* @brief  rfcomm send frame
* 
* @param  tc
* @param  dlci: dlci
* @param  ctl: control field without P/F
* @param  cr: command or response
* @param  poll: P/F
* @param  pl: info
* @param  pllen: info length
* @param  highPrio
*
* @return  
*
*/
void rfcSendFrame(TrfcChannel * cc, uint8_t dlci, uint8_t ctl, uint8_t cr, uint8_t poll, uint8_t * pl, uint16_t pllen, int highPrio)
{
	uint8_t * buf;
//	MESSAGE_T msg;
	uint8_t * p;
	uint8_t hsize;
	uint16_t mlen;
	uint8_t offset;
	uint16_t cid;
	uint16_t length;
	
	
	/*length in head maybe 1 byte or 2 byte
	first byte is formated as 1bit EA and 7bit length. 
	if EA = 1, length is 1byte(7bit);else length is 2byte(15bit)
	*/
	if (pllen > 127)/*127 = 0x7E, largest in 7bit*/
	{
		hsize = 4;	/*2 byte length*/
	}
	else
	{
		hsize = 3;	/*1 byte length*/
	}

	mlen = (uint16_t)(pllen + hsize + 1 + pRFC->writeOffset);
	//if (osBufferGet(BTSystemPoolID, mlen, (void FAR *)&buf))
	if (blueAPI_CauseSuccess !=
		blueAPI_BufferGet(cc->dsPoolID, 
						mlen, 
						0, 
						(void * *)&buf) )
	{
	    DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "!!rfcSendframe : NO MEMORY dlci %X mlen %d",2, dlci, mlen);
//	    assert(FALSE);
	    return;
	}
	p = buf + pRFC->writeOffset;

	/*Address Field:
	|EA(1bit) | C/R(1bit) | DLCI(6bit)|*/
	/* insert dlci and command/response bit */
	*p = (dlci << 2) | EA_BIT;
	if ((cr && cc->initiator) || (!cr && !cc->initiator))
	{
		*p |= 2;
	}
	p++;

	/*control field*/
	/* insert frame type and poll bit */
	*p = ctl;
	if (poll)
	{
		*p |= RFCOMM_POLL;
	}
	p++;

	/*length field*/
	/* insert length field */
	if (pllen > 127)
	{
	    *p++ = (pllen << 1);
	    *p++ = pllen >> 7;
	}
	else
	{
	    *p++ = (pllen << 1) | EA_BIT;
	}
	if (pllen)
	{
	    memcpy(p, pl, pllen);
	    p += pllen;
	}

	/* for UIH, check only first 2 bytes */
	*p++ = crc8EtsGen(buf + pRFC->writeOffset, (uint16_t)(ctl == RFCOMM_UIH ? 2 : 3));
/*
	msg.MData.DataCBChan.BufferAddress = buf;
	msg.MData.DataCBChan.Flag          = DATA_CB_RELEASE;
	msg.MData.DataCBChan.Channel       = cc->channel;
	msg.MData.DataCBChan.Offset        = pRFC->writeOffset;
	msg.MData.DataCBChan.Length        = pllen + hsize + 1;
*/
	cid = cc->channel;
	length = pllen + hsize + 1;
	offset = pRFC->writeOffset;

	if(highPrio)
	{
		//l2cHandleL2C_DATA_REQ(&msg, TRUE);
		mpa_Sendl2cDataReq(buf, offset, cid,  length);
	}
	else
	{
		//l2cHandleL2C_DATA_REQ(&msg, FALSE);
		mpa_Sendl2cDataReq(buf, offset, cid,  length);
	}
}

/**
* @brief  rfcomm send sabm
* 
* @param  tc
* @param  command
* @param  poll
*
* @return  
*
*/
void rfcSendSABM(TrfcChannel * tc, uint8_t command, uint8_t poll)
{
	rfcSendFrame(tc->cc, tc->dlci, RFCOMM_SABM, command, poll, NULL, 0, 0 /* lo prio */);
}

/**
* @brief  rfcomm send ua
* 
* @param  tc
* @param  command
* @param  poll
*
* @return  
*
*/
void rfcSendUA(TrfcChannel * tc, uint8_t command, uint8_t poll)
{
	rfcSendFrame(tc->cc, tc->dlci, RFCOMM_UA, command, poll, NULL, 0, 0 /* lo prio */);
}

/**
* @brief  rfcomm send dm
* 
* @param  cc
* @param  dlci
* @param  command
* @param  poll
*
* @return  
*
*/
void rfcSendDM(TrfcChannel * cc, uint8_t dlci, uint8_t command, uint8_t poll)
{
	rfcSendFrame(cc, dlci, RFCOMM_DM, command, poll, NULL, 0, 0 /* lo prio */);
}

/**
* @brief  send dis msg
* 
* @param  tc
* @param  command
* @param  poll
*
* @return  
*
*/
void rfcSendDISC(TrfcChannel * tc, uint8_t command, uint8_t poll)
{
	rfcSendFrame(tc->cc, tc->dlci, RFCOMM_DISC, command, poll, NULL, 0, 0 /* lo prio */);
}

/**
* @brief  send UIH msg
* 
* @param  tc
* @param  typ
* @param  cr
* @param  p
* @param  len
*
* @return  
*
*/
void rfcSendUIH(TrfcChannel * tc, uint8_t typ, uint8_t cr, uint8_t * p, uint16_t len)
{
	uint8_t buf[32];  /* not especially elegant */
	buf[0] = (typ << 2) | (cr << 1) | EA_BIT;
	buf[1] = (len << 1 ) + EA_BIT;

	/* limit the length (safety) */
	if (len > sizeof(buf)-2)
	{
		DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "!!rfcSendUIH: buffer too small, cutting (len %d)", 1,len);
		len = sizeof(buf) - 2;
	}
	if (len)
	{
    	memcpy(buf + 2, p, len);
	}
	rfcSendFrame(
				tc->cc,
				tc->dlci,
				RFCOMM_UIH,
				1 /* COMMAND */,
				0 /* NO POLL */,
				buf, (uint16_t)(len+2),
				cr ? 0 : 1 /* only responses are high priority */
				);
}

/**
* @brief  rfcomm send modem status command
* 
* @param  tc
* @param  cr
* @param  dlci
* @param  status
* @param  sbreak
*
* @return  
*
*/
void rfcSendMSC(TrfcChannel * tc, uint8_t cr, uint8_t dlci, uint8_t status, uint8_t sbreak)
{
	uint8_t buf[3];
	uint8_t len = 2;

	buf[0] = (dlci << 2) | 2 | EA_BIT;
	buf[1] = status | EA_BIT;
	if (sbreak & 1)
	{
		buf[1] = status;
		buf[2] = sbreak;
		len    = 3;
	}
	rfcSendUIH(tc, RFCOMM_TYPE_MSC, cr, buf, len);
} /* rfcSendMSC */

/**
* @brief  Send a Remote Line Status to Peer
*
* @param  tc:
* @param  cr: 
* @param  dlci
* @param  status
*
* @return  
*
*/
void rfcSendRls(TrfcChannel * tc, uint8_t cr, uint8_t dlci, uint8_t status)
{
	uint8_t buf[2];

	buf[0] = (dlci << 2) | 2 | EA_BIT;
	buf[1] = status | EA_BIT;

	rfcSendUIH(tc, RFCOMM_TYPE_RLS, cr, buf, sizeof(buf));
} /* rfcSendRls */

/**
* @brief  rfcomm send upstream connect indicate
* 
* @param  dlci
* @param  bd
* @param  handle
* @param  frameSize
* @param  creditBased
* @param  outgoing
*
* @return  
*
*/

void rfcSendUConInd(uint8_t dlci, TBdAddr bd, uint8_t handle, uint16_t frameSize, bool creditBased, uint8_t outgoing)
{
#if 0
	TBtConInd tconInd;

	memcpy(tconInd.bd, bd, BD_ADDR_SIZE);
	tconInd.cid               = handle;
	tconInd.channel           = (uint8_t)(dlci>>1); /* upper layer does not see direction bit of dlci */
	tconInd.p.rfc.frameSize   = frameSize;
	tconInd.p.rfc.creditBased = creditBased;
	tconInd.p.rfc.outgoing    = outgoing;   
	blueFaceHandleBtConInd_2(BLUEFACE_PSM_RFCOMM, &tconInd);	
#endif	
	
}

/**
* @brief  send rfcomm upstream flow indicate
*
* @param  handle
* @param  status
* @param  command
* @param  sbreak
*
* @return  
*
*/
void rfcSendUFlowInd(uint8_t handle, uint8_t status, uint8_t command, uint8_t sbreak)
{
#if 0
	LPbtLink pLink;

	pLink = blueFaceFindLinkByHandle(handle, BLUEFACE_PSM_RFCOMM);
	if (pLink==NULL)
	{
		DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "!!rfcSendUFlowInd: cannot find handle %X", 1,handle);
		return;
	}
	if (command)
	{
		if ((pLink->rxMsc != status) || (sbreak & 0x01))    /* change in MSCstatus or break */
		{
			/* remember the state */
			pLink->rxMsc = status;
		}
	}
#endif 
	
}


/**
* @brief  rfcomm send upstream rpn indicator
*
* @param  handle
* @param  rpn
* @param  len
*
* @return  
*
*/

void rfcSendURpn(uint8_t handle, LPrpn rpn, uint8_t len)
{
/*
	LPbtLink pLink;

	pLink = blueFaceFindLinkByHandle(handle, BLUEFACE_PSM_RFCOMM);
	if (pLink == NULL)
	{
		DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "!!rfcSendURpn: cannot find handle %X",1, handle);
		return;
	}
    rfcHandleRfcRpnResp((uint8_t)pLink->handle, (uint8_t *)rpn, len);
*/
}

/**
* @brief  check if rfcomm downstream is blocked
*
* @param  tc:
*
* @return 
*
*/
bool rfcDsBlocked(PTrfcChannel tc)
{
    if (!tc->used)
	{
    	return FALSE;     /* closed channels are always free... */
	}
    if (tc->cc->dsFlow & BLUEFACE_FLOW_BIT)
	{
    	return TRUE;      /* aggregate flow control (on mux channel) stop all channels */
	}

    if (tc->convergenceLayer == RFCOMM_CONVERGENCE_CREDIT_CONF)
    {
        if(tc->remainingCredits == 0)    /* all credits exhausted ? */
        {
              DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "RFC: C0",0,0);
            return TRUE;
        }
        /* the road is free for credit based ... */
        return FALSE;
    }

    if (tc->dsFlow & BLUEFACE_FLOW_BIT)
	{
    	return TRUE;      /* channel specific flow control */
	}

    /* the road is free ... */
    return FALSE;
}

void rfcDownstreamWorker(uint8_t type)
{
//	MESSAGE_T msg;
//	DATA_CB_CHAN_T *cb;

//	DATA_CB_CHAN_T	rfcFrame;
	uint16_t pos;
	uint8_t * buf;
	uint16_t len;
	PTrfcChannel tc, tcStart;
	uint8_t crc;
	int i, maxCount;
	uint8_t cr; /* command / response bit */
	bool creditField;
	TRfcUpDataMsg temp;
	
	/* repeat until no more can be done now. if a condition is found that
	cannot be retried immediately: return / break. if a condition is found
	that could be resolved immediately: continue
	*/
	i = 0;

	if (type == 1)
	{
		maxCount = otp_str_data.gEfuse_UpperStack_s.num_data_rfcom_dlci_don;
		tcStart = pRFC->rfcchannelDon;
	}
	else
	{
		maxCount = otp_str_data.gEfuse_UpperStack_s.num_data_rfcom_dlci_doff;
		tcStart = pRFC->rfcchannelDoff;
	}

	while(i < maxCount)
	{
		tc          = &tcStart[i];
//		cb          = &msg.MData.DataCBChan;                        /* addressing shortcut */
		creditField = FALSE;

		/* we are handling a priority scheme between backcredits and payload frames here:
		if groups of backcredits are available, they prioritize over payload. single
		credits are passed into the packet if there is enough space (i.e. the payload is below
		N1
		*/

		/* for credit based flow control, set the flag creditField if backCredits groups are available */
		if (tc->used
			&& (tc->convergenceLayer == RFCOMM_CONVERGENCE_CREDIT_CONF)
			&& tc->backCredits
			/* return backcredits in groups of 4 or immediately, if only small fraction left */
			&& ((tc->backCredits >= 4) || ((tc->maxUpstreamBuffers - tc->backCredits) <= 3))
			)
		{
			creditField = TRUE;
		}

	//	cb->BufferAddress = NULL;
		temp.BufferAddress  = NULL;

		/* if no prio credits and downstream trafic is allowable, check for data
		in the queue
		*/
		if (!creditField && !rfcDsBlocked(tc))
		{
//			if (osMessageReceive(tc->dsQueueID, &msg) == 0)
			if(xQueueReceive(tc->dsQueueID, &temp, portTICK_RATE_MS * 1) == pdPASS)			
			{
				/* message received */
				if (temp.payloadLength==0)//(cb->Length == 0)
				{
					/* message is empty, discard */
					blueAPI_BufferRelease(temp.BufferAddress);
					temp.BufferAddress = NULL;
				//	cb->bufferAddress= NULL;
				//	if (cb->Flag & DATA_CB_RELEASE)
				//	{
				//		osBufferRelease(cb->BufferAddress);
				//	}
				//	cb->BufferAddress = NULL;
				}
				else if ((!tc->used) || (tc->state != dlciConnected))
				{
					DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "!!rfcDownstreamWork: dlci %X in wrong state",1, tc->dlci);
					blueAPI_BufferRelease(temp.BufferAddress);
					temp.BufferAddress = NULL;
				//	if (cb->Flag & DATA_CB_RELEASE)
				//	{
				//		osBufferRelease(cb->BufferAddress);
				//	}
				//	cb->BufferAddress = NULL;
				}
			} /* message received */
			else
			{
				/* continue with next channel */
				i++;
			}
		} /* not blocked */
		else
		{
			/* flow is blocked or prio to credits, continue with next channel */
			i++;
		}

		if ( (temp.BufferAddress == NULL)&& creditField)	//((cb->BufferAddress == NULL) && creditField)
		{
			/* no message to send and backCredits available, prepare empty message */
			uint16_t offset = pRFC->writeOffset
							+ RFCOMM_SIZE_UIH  /* rfcomm header length */
							+ 1;               /* for credit field */

	//		if (osBufferGet(BTSystemPoolID, offset, (void FAR *)&cb->BufferAddress))
			if(blueAPI_CauseSuccess !=
				blueAPI_BufferGet(tc->dsPoolID, 
								offset,
								0,
								(void * *)&temp.BufferAddress )	)
			{
				  DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR,
							"!!rfc UIH: no memory for backCredits on dlci %X (backCredits %X)",2,
							tc->dlci, tc->backCredits);
			//	cb->BufferAddress = NULL;
				temp.BufferAddress = NULL;
			}
			else
			{
				//cb->Length = 0;                 /* there is no payload for this message */
				//cb->Flag   = DATA_CB_RELEASE;   /* must release this buffer */
				//cb->Offset = offset;
				temp.payloadLength= 0;
				temp.Offset =  offset;
				
			}
		}

		/* really nothing found that could be done, continue with next channel */
		if (temp.BufferAddress== NULL) //(cb->BufferAddress == NULL)
		{
			continue;
		}

		/* check if we cann piggy pack credits on this frame, this is only possible if
		payload < N1 (cf. RFCOMM spec. p.419
		*/
		if (!creditField
			&& (tc->convergenceLayer == RFCOMM_CONVERGENCE_CREDIT_CONF)
			&& tc->backCredits
			&& (temp.payloadLength< tc->frameSize)//(cb->Length < tc->frameSize)
			)
		{
			creditField = TRUE;
		}

		  DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO,
							"rfcDownstreamWork: desc %X handle %x addr %lp length %d offset %d",4,
							i, tc->channel, temp.BufferAddress,temp.Length);
		 // DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO,
		//					"rfcDownstreamWork: desc %X handle %x addr %lp length %d offset %d",5,
						//	i, cb->Channel, cb->BufferAddress, cb->Length, cb->Offset);

	//	assert(cb->Length <= tc->frameSize);
	//	assert(cb->Offset >= RFCOMM_SIZE_UIH + 1);
		/* minimum no of bytes we require (1 for credit field */
	//	len = cb->Length;
	//	buf = cb->BufferAddress;
	//	pos = cb->Offset - 3;             /* assume 3 byte RFCOMM header */
		temp.Length  = temp.payloadLength;
		len = temp.payloadLength;
		buf = temp.BufferAddress;
		pos = temp.Offset - 3;

		if (len > 127)
		{
			pos--;                        /* extended length field */
		}

		if (creditField)
		{
			pos--;                        /* insert space for back credit field */
		}

		/* build the message header */
		if (tc->cc->initiator)
		{
			cr = 2;
		}
		else
		{
			cr = 0;
		}
		buf[pos+0] = (tc->dlci << 2) | cr | EA_BIT;
		buf[pos+1] = RFCOMM_UIH;

		/* for credit based flow control, set the poll bit if backCredits are available */
		if (creditField)
		{
			buf[pos+1]         |= RFCOMM_POLL;      /* switch header format to "credits included" */
			/* check against the maximum amount of credits that fit into one packet. if there
			are more an 255 backcredits, they will be sent in multiple fractions
			*/
			if (tc->backCredits > 255)
			{
				//buf[cb->Offset-1]   = 255;
				buf[temp.Offset-1]   = 255;
				tc->backCredits    -= 255;
			}
			else
			{
			//	buf[cb->Offset-1]   = (uint8_t) tc->backCredits;
				buf[temp.Offset-1]   = (uint8_t) tc->backCredits;
				tc->backCredits     = 0;
			}
		}

		crc  = crc8EtsGen(buf+pos,2);

		if (len > 127)
		{
			buf[pos+2] = (len << 1);
			buf[pos+3] = len >> 7;
		}
		else
		{
			buf[pos+2] = (len << 1) | EA_BIT;
		}
		/* OKAY: the header is ready now! */

		/* Increase message length by header length and set new offset */
	//	cb->Length += cb->Offset - pos;
	//	cb->Offset  = pos;
		temp.Length += temp.Offset - pos;
		temp.Offset = pos;
		/* data payload is already in place, add only checksum at end of payload */
	//	buf[pos + cb->Length++] = crc;
		buf[pos + temp.Length++] = crc;

	//	cb->Channel        = tc->cc->channel;
		temp.Channel = tc->cc->channel;

//		l2cHandleL2C_DATA_REQ(&msg, FALSE);
		mpa_Sendl2cDataReq(temp.BufferAddress, 0,temp.Channel , temp.Length );
//		cb->BufferAddress  = NULL;
		if ((tc->convergenceLayer == RFCOMM_CONVERGENCE_CREDIT_CONF) && len)
		{
			/* decrement credits only for non zero packets */
			tc->remainingCredits--;
		}
	} /* while */
}

/**
* @brief  handle rfcomm downstream data
*
* @param  pRFC:
* @param  msg: 
*
* @return 
*
*/
void rfcDownstreamWork(void)
{
#if 0
    MESSAGE_T msg;
    DATA_CB_CHAN_T * cb;
    uint16_t pos;
    uint8_t * buf;
    uint16_t len;
    PTrfcChannel tc;
    uint8_t crc;
    int i;
    uint8_t cr; /* command / response bit */
    bool creditField;

    /* repeat until no more can be done now. if a condition is found that
       cannot be retried immediately: return / break. if a condition is found
       that could be resolved immediately: continue
    */
    i = 0;

    while(i < RFCOMM_MAX_DLCI)
    {
        tc          = &pRFC->cl[i];
        cb          = &msg.MData.DataCBChan;                        /* addressing shortcut */
        creditField = FALSE;

        /* we are handling a priority scheme between backcredits and payload frames here:
           if groups of backcredits are available, they prioritize over payload. single
           credits are passed into the packet if there is enough space (i.e. the payload is below
           N1
        */

        /* for credit based flow control, set the flag creditField if backCredits groups are available */
        if (tc->used
            && (tc->convergenceLayer == RFCOMM_CONVERGENCE_CREDIT_CONF)
            && tc->backCredits
            /* return backcredits in groups of 4 or immediately, if only small fraction left */
            && ((tc->backCredits >= 4) || ((tc->maxUpstreamBuffers - tc->backCredits) <= 3))
            /* return back credits only if the channel is fully open */
            && (tc->mscHandshake == 3)
            )
        {
            creditField = TRUE;
        }

        cb->BufferAddress = NULL;

        /* if no prio credits and downstream trafic is allowable, check for data
           in the queue
        */
        if (!creditField && !rfcDsBlocked(tc))
        {
            if (osMessageReceive(tc->dsQueueID, &msg) == 0)
            {
                /* message received */
                if (cb->Length == 0)
                {
                    /* message is empty, discard */
                	if (cb->Flag & DATA_CB_RELEASE)
                    {
                        osBufferRelease(cb->BufferAddress);
                    }
                    cb->BufferAddress = NULL;
                }
                else if ((!tc->used) || (tc->state != dlciConnected))
                {
                    RFCOMM_TRACE_VERBOSITY_1(RFCOMM_TRACE_PRINTF_1(RFCOMM_TRACE_MASK_ERROR,"!!rfcDownstreamWork: dlci %X in wrong state",tc->dlci));
                    if (cb->Flag & DATA_CB_RELEASE)
                    {
                        osBufferRelease(cb->BufferAddress);
                    }
                    cb->BufferAddress = NULL;
                }
            } /* message received */
            else
            {
                /* continue with next channel */
                i++;
            }
        } /* not blocked */
        else
        {
            /* flow is blocked or prio to credits, continue with next channel */
            i++;
        }

        if ((cb->BufferAddress == NULL) && creditField)
        {
            /* no message to send and backCredits available, prepare empty message */
            uint16_t offset = pRFC->writeOffset
                             + RFCOMM_SIZE_UIH  /* rfcomm header length */
                             + 1;               /* for credit field */

            if (osBufferGet(BTSystemPoolID, offset, (void FAR *)&cb->BufferAddress))
            {
                RFCOMM_TRACE_VERBOSITY_1(RFCOMM_TRACE_PRINTF_2(RFCOMM_TRACE_MASK_ERROR,"!!rfc UIH: no memory for backCredits on dlci %X (backCredits %X)",
                                              tc->dlci, tc->backCredits));
                cb->BufferAddress = NULL;
            }
            else
            {
								cb->Length = 0;                 /* there is no payload for this message */
								cb->Flag   = DATA_CB_RELEASE;   /* must release this buffer */
								cb->Offset = offset;
            }
        }

        /* really nothing found that could be done, continue with next channel */
        if (cb->BufferAddress == NULL)
    	{
        	continue;
    	}

        /* check if we cann piggy pack credits on this frame, this is only possible if
           payload < N1 (cf. RFCOMM spec. p.419
        */
        if (!creditField
            && (tc->convergenceLayer == RFCOMM_CONVERGENCE_CREDIT_CONF)
            && tc->backCredits
            && (cb->Length < tc->frameSize)
            )
        {
            creditField = TRUE;
        }

        RFCOMM_TRACE_VERBOSITY_3(RFCOMM_TRACE_PRINTF_8(RFCOMM_TRACE_MASK_TRACE,"rfcDownstreamWork: desc %X handle %x addr %lp length %d offset %d",
                                      i, cb->Channel, cb->BufferAddress, cb->Length, cb->Offset, 0, 0, 0));

        assert(cb->Length <= tc->frameSize);
        assert(cb->Offset >= RFCOMM_SIZE_UIH + 1);
                                          /* minimum no of bytes we require (1 for credit field */
        len = cb->Length;
        buf = cb->BufferAddress;
        pos = cb->Offset - 3;             /* assume 3 byte RFCOMM header */

        if (len > 127)
        {
            pos--;                        /* extended length field */
        }

        if (creditField)
        {
            pos--;                        /* insert space for back credit field */
        }

        /* build the message header */
        if (tc->cc->initiator)
    	{
        	cr = 2;
    	}
        else
    	{
        	cr = 0;
    	}
        buf[pos+0] = (tc->dlci << 2) | cr | EA_BIT;
        buf[pos+1] = RFCOMM_UIH;

        /* for credit based flow control, set the poll bit if backCredits are available */
        if (creditField)
        {
            buf[pos+1]         |= RFCOMM_POLL;      /* switch header format to "credits included" */
            /* check against the maximum amount of credits that fit into one packet. if there
               are more an 255 backcredits, they will be sent in multiple fractions
            */
            if (tc->backCredits > 255)
            {
                buf[cb->Offset-1]   = 255;
                tc->backCredits    -= 255;
            }
            else
            {
                buf[cb->Offset-1]   = (uint8_t) tc->backCredits;
                tc->backCredits     = 0;
            }
        }

        crc  = crc8EtsGen(buf+pos,2);

        if (len > 127)
        {
            buf[pos+2] = (len << 1);
            buf[pos+3] = len >> 7;
        }
        else
        {
            buf[pos+2] = (len << 1) | EA_BIT;
        }
        /* OKAY: the header is ready now! */

        /* Increase message length by header length and set new offset */
        cb->Length += cb->Offset - pos;
        cb->Offset  = pos;

        /* data payload is already in place, add only checksum at end of payload */
        buf[pos + cb->Length++] = crc;

        //msg.Command        = L2C_DATA_REQ;    /* this is normal data (not express) */
        cb->Channel        = tc->cc->channel;

			gl2cHandleL2C_DATA_REQ(&msg, FALSE);
        cb->BufferAddress  = NULL;

        if ((tc->convergenceLayer == RFCOMM_CONVERGENCE_CREDIT_CONF) && len)
        {
            /* decrement credits only for non zero packets */
            tc->remainingCredits--;
        }
    } /* while */
#else		
	rfcDownstreamWorker(1);
	rfcDownstreamWorker(0);
#endif		
} /* rfcDownStreamWork */

/**
* @brief  send rfcomm param neg command
* 
* @param  tc
*
* @return  
*
*/
void rfcSendPNCommand(TrfcChannel * tc)
{
    uint8_t pn[RFCOMM_SIZE_PN];

    /* check if the current framesize setting on l2cap level is compatible with the
       current N1 negotiation (enough room for RFCOMM header). NOTE: for credit
       based flow control, the credit field is logically not part of the header, but
       part of the data payload field, because it reduces -- if present -- thhe maximum
       allowable payload
    	*/

    /* check if frameSize setting for this channel is acceptable according to L2CAP mtuSize
       "1" is for the checksum!
    	*/

    uint16_t headerSize = RFCOMM_SIZE_UIH + 1;

#if (X_TEST_AGG_FLOW == 0)
    if ((tc->frameSize + headerSize) > tc->cc->mtuSize)
    {
        tc->frameSize = tc->cc->mtuSize - headerSize;
       DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "rfcSendPNCommand: reducing frameSize to %d l2cap %d",2, tc->frameSize, tc->cc->mtuSize);
    }
#endif

    rfcSavePN(tc, pn);
    rfcCast(tc, dlciConfiging);
    rfcSetTimer(RFCOMM_T1_ID, tc, RFCOMM_T1_TIMEOUT_EXTENDED);
    rfcSendUIH(tc->cc,RFCOMM_TYPE_PN, 1, pn, sizeof(pn));
}

/**
* @brief  rfcomm send connect response to remote device
* 
* @param  pRFC
* @param  pmsg
*
* @return  
*
*/
void rfcSendConResp(uint16_t    cid,               /* Identifier for this connection */
							     uint16_t    channel,            /* protocol specific channel info */
							     uint16_t    status,             /* result of the connect indication  */
							     TBtConRespRfc	rfc		)
							  	//PBtConRespPSpecifc p       /* protocol specific part         */)
{
    TrfcChannel * tc;

    tc = rfcFindHandle((uint8_t)cid);

    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO,
					"RFC: CON_RESP desc %X cid %X accept %X MSCstatus %X maxCredits %x frameS %d",6,
					tc, cid, status, rfc.mscStatus, rfc.maxCredits, rfc.frameSize);
	
    if (!tc)
    {
        DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "!!RFC: RFC_CON_RESP: could not find desc for handle %X",1, cid);
        return;
    }

    if (status)   /* this is a reject !*/
    {
        DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "RFC: connection setup on dlci %X refused (initiator %d)", 2,tc->dlci, tc->initiator);

        /* if we are the initiator of the link, we do not have to answer with a DM */
        if (!tc->initiator)
        {
            if (tc->dlci == 0)
        	{
           		rfcSendDM(tc->cc, tc->dlci, FALSE /* RESPONSE */, TRUE /* POLL */);
        	}
            else
            {
				/* do not answer with DM, but answer with PN because some Microsoft implementations have problems
				  with TYPE_PN / DM exchange and seem to prefer TYPE_PN / TYPE_PN followed  by SABM / DM.
				  The channeldescriptor for this DLCI is deallocated, so the subsequent SABM will occur on an
				  unconfigured DLCI, therefore being rejected.
				*/
				uint8_t pn[RFCOMM_SIZE_PN];
				rfcSavePN(tc, pn);                                    /* answer with unchanged configuration */
				rfcSendUIH(tc->cc, RFCOMM_TYPE_PN, 0, pn, sizeof(pn)); /* Response on Control Channel ! */
            }
        }

        /* if mux session is refused, close the l2cap link also */
        if (tc->dlci == 0)
        {
            rfcCloseChannels(tc, RFCOMM_ERR | RFCOMM_ERR_REJECTED);
		//	l2cHandleBTG_DISC_REQ(tc->channel, FALSE);
		mpa_Sendl2cDiscReq(tc->channel);
        }

        rfcFreeChannel(tc);
        return;
    }

   DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "RFC: connection setup on dlci %X accepted (initiator %d)",
        					tc->dlci, tc->initiator);

    if (tc->cc->lstate == linkDisconnectRequested)
    {
        DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "RFC: link is going down dlci %X",1, tc->dlci);
        return;
    }
    if ((tc->dlci==0) && tc->initiator)
    {
        /* We received a ConResp to MUX channel where we are initiator, start SABM etc */
        rfcSetTimer(RFCOMM_T1_ID, tc, RFCOMM_T1_TIMEOUT_EXTENDED);
        rfcSendSABM(tc, TRUE /* COMMAND */, TRUE /* POLL */);
        return;
    }

    if (tc->state == dlciConfigIndicated)
    {
        /* opening a channel where we received a PN command for channel configuration */
        uint8_t pn[RFCOMM_SIZE_PN];
        tc->usFlow = rfc.mscStatus;

        /* response from application with larger framesize than indicated */
        if (rfc.frameSize > tc->frameSize)
        {
          	DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "RFC: CON_RESP reducing framesize from %d to %d",2,
                                        rfc.frameSize, tc->frameSize);

          	rfc.frameSize = tc->frameSize;
        }

        tc->frameSize = rfc.frameSize;
        rfcAllocateBuffers(tc, rfc.maxCredits);
        rfcCast(tc, dlciConfigAccepted);
        rfcSavePN(tc, pn);                                    /* answer with (changed) configuration */
        rfcSendUIH(tc->cc, RFCOMM_TYPE_PN, 0, pn,sizeof(pn)); /* Response on Control Channel ! */
        return;
    }

    rfcSendUA(tc, FALSE /* RESPONSE */, TRUE /* POLL */);
    /* send initial MSC command on all non cc links */
    if (tc->dlci)
    {
        tc->usFlow = rfc.mscStatus;
        tc->usFlowActive |=  1; 	 /* one MSC request outstanding */
        rfcSetTimer(RFCOMM_T1_ID, tc->cc, RFCOMM_T1_TIMEOUT);
        rfcSendMSC(tc->cc, TRUE /* COMMAND */, tc->dlci, tc->usFlow, 0);
    }
    rfcResetTimer(RFCOMM_T1_ID, tc);
    rfcCast(tc, dlciConnected);

	if (tc->dlci != 0)
	{
		rfcSetTimer(RFCOMM_T2_ID, tc, RFCOMM_T2_TIMEOUT);
	}
}


#if defined(UPF_TEST)
void rfcHandleRfcErrorReq(MESSAGE_T * msg)
{
    TrfcErrorReq * err = (TrfcErrorReq *) msg->MData.MessageData;
    TrfcChannel * tc;

    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO,"rfcHandleRfcErrorReq handle %X type %d length %X",4,err->handle, err->type, err->length, 0);
    tc = rfcFindHandle(err->handle);
    if (!tc || tc->state != dlciConnected)
    {
        DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR,"!!rfcHandleRfcErrorReq could not found or wrong state hdl %X",1,err->handle);
        return;
    }
//    rfcSendUIH(tc->cc, 0x34, 1 /* COMMAND */, NULL, 0 );
    rfcSendUIH(tc->cc, err->type, 1 /* COMMAND */, err->data, err->length );
}
#endif  /* defined(UPF_TEST) */


void rfcHandleUIHPN(TrfcChannel *tc, uint8_t tcr, uint8_t * p, uint16_t pos, uint16_t len)
{
	uint8_t pn[RFCOMM_SIZE_PN];
	uint8_t dlci;
	TrfcChannel * lc;
	bool oldlc = FALSE;

	dlci = p[pos];
	DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO,"rfcHandleUIH: RFCOMM_TYPE_PN dlci %X cr %X", 2,dlci, tcr);
	if (tcr) /* this is a command */
	{
		lc = rfcFindDLCI(dlci, tc->channel);
		if (lc)
		{
			DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "rfcHandleUIH: RFCOMM_TYPE_PN on existing dlci %X", 1,dlci);
			oldlc = TRUE; /* remember, this descriptor already existed */
		}
		else
		{
			lc = rfcAllocateDLCI();
			if (!lc)
			{
				/* could not allocate descriptor, we are generating an unsolicited DM, send without poll bit */
				rfcSendDM(tc, dlci, FALSE /* RESPONSE */, FALSE /* NOT POLL */);
				return;
			}
			rfcInitDLCI(lc,dlci);
			lc->cc = tc;					 /* this is our control channel */

			/* test feature: reject everything above dlci 4 */
			rfcCast(lc, dlciConfigured);
		}
		/* check if the link is already open, if the link is open, do reject any changes to its configuration  */
		if (((lc->state != dlciConnected) || oldlc) && (lc->state != dlciConfigAccepted))
		{
			rfcLoadPN(lc, p+pos, (uint16_t)(len-pos));	 /* load desired configuration */

			/* change config to local requirements HERE */
			/* check if mtu size requested matches l2cap capabilities */
			DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "rfcHandleUIH: RFCOMM_TYPE_PN , requested frameSize %d l2cap %d converg %X initial credits %d",
										 4, lc->frameSize, lc->cc->mtuSize, lc->convergenceLayer, lc->windowSize);

			/* check if credit based flow control is requested */
			if (lc->convergenceLayer == RFCOMM_CONVERGENCE_CREDIT_REQ)
			{
				if (pRFC->creditSupport)
				{
					lc->convergenceLayer = RFCOMM_CONVERGENCE_CREDIT_CONF;
					DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "rfcHandleUIH: RFCOMM_TYPE_PN , accept CREDIT on dlci %X credits %d",2, dlci, lc->windowSize);
					lc->remainingCredits = lc->windowSize;
					/* the number of credits signalled initially to the peer is 7 (by protocol definition)
							 have more then that, we signal the remaining credits using returnCredits */
					if (lc->maxUpstreamBuffers > 7)
					{
						lc->windowSize	 = 7;
					}
					else
					{
						lc->windowSize	 = (uint8_t)lc->maxUpstreamBuffers;
					}
					tc->backCredits 	 = lc->maxUpstreamBuffers - lc->windowSize;
				}
				else
				{
					lc->convergenceLayer = RFCOMM_CONVERGENCE_0;
					DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "rfcHandleUIH: RFCOMM_TYPE_PN , rejected CREDIT on dlci %X",1, dlci);
				}
			} /* if (credit based flow control) */

			/* check if frameSize setting for this channel is acceptable according to L2CAP mtuSize */
			if ((lc->frameSize + RFCOMM_SIZE_UIH + 1) > lc->cc->mtuSize)
			{
				lc->frameSize = lc->cc->mtuSize - RFCOMM_SIZE_UIH - 1;
				DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "rfcHandleUIH: RFCOMM_TYPE_PN reducing frameSize to %d (l2c %d)",
										2,lc->frameSize, lc->cc->mtuSize);
			}

			/* config request for new channel arrived: signal con_ind to upper layer.... */
			rfcCast(lc, dlciConfigIndicated);
			rfcSendSecManAuthenticationInd(lc, 0 /* incoming */, 1 /* active */);
			return;
		}
		else
		{
			DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "rfcHandleUIH: RFCOMM_TYPE_PN on open dlci %X -- reject changes",1, dlci);
		}
		rfcSavePN(lc, pn);								  /* answer with (changed) configuration */
		rfcSendUIH(tc, RFCOMM_TYPE_PN, 0, pn, sizeof(pn)); /* Response on Control Channel ! */
	}
	else
	{	/* this is a response */
		uint16_t framesize;

		lc = rfcFindDLCI(dlci, tc->channel);
		if (!lc)
		{
			DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "!!rfcHandleUIH: could not find desc for dlci %X", 1,dlci);
			return;
		}

		if (lc->state == dlciConnected)
		{
			/* this is a RFCOMM_TYPE_PN response on an open channel, a reconfig-request
					  @@@@ this is only supported for UPF4, ignore responses!
					 */
			DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "rfcHandleUIH: RFCOMM_TYPE_PN response on open channel, ignore, dlci %X", 1,dlci);
			return;
		}

		framesize = lc->frameSize;

		/* load desired configuration into descriptor record */
		rfcLoadPN(lc, p+pos, (uint16_t)(len-pos));	 /* load desired configuration */

		/* peer increased framesize in response, use request value (better: disconnect link) */
		if (lc->frameSize > framesize)
		{
			DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "rfcHandleUIH: RFCOMM_TYPE_PN reducing invalid framesize %d > %d!!",
										2,lc->frameSize, framesize);
			lc->frameSize = framesize;
		}

		/* recheck the configuration that was sent by remote entity */
		if (lc->convergenceLayer == RFCOMM_CONVERGENCE_CREDIT_REQ)
		{
			/* if the request is still set, then remote side does not
					  understand credit based flow control ... (rfcomm p.15) */
			lc->convergenceLayer = RFCOMM_CONVERGENCE_0;
		}
		if (lc->convergenceLayer == RFCOMM_CONVERGENCE_CREDIT_CONF)
		{
			DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "rfcHandleUIH: RFCOMM_TYPE_PN ,credit based is active on dlci %X credits %X",2, dlci, lc->windowSize);
			lc->remainingCredits = lc->windowSize;
		}

		rfcCast(lc, dlciConnecting);
        	rfcResetTimer(RFCOMM_T1_ID, lc);

		/* start SABM timer */
		rfcSetTimer(RFCOMM_T1_ID, lc, RFCOMM_T1_TIMEOUT_EXTENDED);
		rfcSendSABM(lc, TRUE /* COMMAND */, TRUE /* POLL */);
	}
}

void rfcHandleUIHMSC(TrfcChannel *tc, uint8_t tcr, uint8_t * p, uint16_t pos)
{
	uint8_t dlci;
	uint8_t status;
	uint8_t sbreak = 0;
	TrfcChannel * lc;

	dlci   = p[pos++] >> 2;
	status = p[pos++];
	if ((status & 1) == 0) /* EA Bit not set: break exists */
	{
		sbreak = p[pos++];
	}

	DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "rfcHandleUI: RFCOMM_TYPE_MSC received, dlci %X status %X tcr %X brk %X",4, dlci, status, tcr, sbreak);

	lc = rfcFindDLCI(dlci, tc->channel);
	if (!lc)
	{
		DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "!!rfcHandleUI : RFCOMM_TYPE_MSC received, dlci %X not found",1, dlci);
		return;
	}

	/* remember, if this was a MSC response */
	if (!tcr)
	{
		/* set flag: MSC response seen */
		lc->mscHandshake |= 2;
		/*not required any more when coming from remote side, 
		is locally generated rfcSendUFlowInd(lc->upperQueueId, lc->pRFC->TaskName, lc->handle, status, tcr, sbreak); */
		/* no more outstanding MSCs */
		lc->usFlowActive &= ~1;
		if (lc->usFlowActive & 2) 
		{
			/* at least one intermediate request is stored locally --> send it now */
			lc->usFlowActive |=  1; 	 /* one MSC request outstanding */
			lc->usFlowActive &= ~2; 	 /* nothing stored locally any more */
			rfcSetTimer(RFCOMM_T1_ID, lc->cc, RFCOMM_T1_TIMEOUT);
			rfcSendMSC(lc->cc, 1 /* COMMAND */, lc->dlci, lc->usFlow, lc->usFlowBreak);
		}
	}
	else
	{
		if ((lc->mscHandshake & 1) == 0 && (lc->TimerHandleT2 != 0))
		{
			/* handshake never seen and T3 is not expired, so this is the first indication. Use this as trigger */
			rfcResetTimer(RFCOMM_T2_ID, lc);
        /*	blueAPI_Handle_RFC_CON_ACT_IND(
                        				lc->maxUpstreamBuffers,
                        				lc->handle,
                        				lc->frameSize,
                        				lc->convergenceLayer == RFCOMM_CONVERGENCE_CREDIT_CONF
                        				);
                       */
                   rfcNotifyUpstream(RFC_CONNECT_COMPLETE, RFCOMM_SUCCESS,lc,NULL);
		}

	//	rfcSendUFlowInd(lc->handle, status, tcr, sbreak);
		rfcNotifyUpstream(RFC_FLOW_IND, RFCOMM_SUCCESS, lc,NULL);
		/* set flag: msc command seen */
		lc->mscHandshake |= 1;

		/* reply the command with a response, copy local values */
		rfcSendMSC(tc, FALSE /* RESPONSE */, dlci, status, sbreak);
		lc->dsFlow = status;
	}

	/* there might have been changes that affect the ability to send data */
	rfcDownstreamWork();
}

void rfcHandleUIHRLS(TrfcChannel *tc, uint8_t tcr, uint8_t * p, uint16_t pos)
{
	uint8_t dlci, status;
	TrfcChannel * lc;

	dlci   = p[pos++] >> 2;
	status = p[pos++];

	DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "rfcHandleUI: RFCOMM_TYPE_RLS received, dlci %X status %X",2, dlci, status);

	lc = rfcFindDLCI(dlci, tc->channel);
	if (!lc)
	{
	    DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "!!rfcHandleUI : RFCOMM_TYPE_RLS received, dlci %X not found", 1,dlci);
	    return;
	}

	/* if this is a response, then we are done (ignore everything) */
	if (!tcr)
	{
	    return;
	}

	/* reply the command with a response, copy local values */
	rfcSendRls(tc, FALSE /* RESPONSE */, dlci, status);
	lc->rls = status;
}

void rfcHandleUIHRPN(TrfcChannel *tc, uint8_t tcr, uint8_t * p, uint16_t pos, uint16_t tlen)
{
    TrfcChannel * lc;
    LPrpn       rpn   = (LPrpn) (p+pos);
    uint8_t        dlci;
  //  bool        newDesc = FALSE;

    dlci = rpn->dlci>>2;

    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "rfcHandleUI: RFCOMM_TYPE_RPN received, len %d dlci %X",2, tlen, dlci);

    if (tlen != 1 && tlen != 8)
    {
        DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "!!rfcHandleUI: RFCOMM_TYPE_RPN illegal length received, dlci %X len %d", 2,rpn->dlci, tlen);
        return;
    }

    lc = rfcFindDLCI(dlci, tc->channel);
    if (!lc)
    {
        DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "!!rfcHandleUI : RFCOMM_TYPE_RPN received, dlci %X not found : allocating",1, dlci);
        lc = rfcAllocateDLCI();

        if (!lc)   /* could not allocate the descriptor */
    	{
        	return;
    	}

 //   newDesc = TRUE;
        rfcInitDLCI(lc,dlci);
        lc->cc = tc;                      /* this is our control channel */
	  memcpy(&lc->rpn,rpn,(uint8_t)tlen);
#if 0
        if (dlci && ((1<<(dlci>>1)) & tc->pRFC->listenChannelMask))
        {
            RFCOMM_TRACE_PRINTF_1(RFCOMM_TRACE_MASK_ERROR, "!!rfcHandleUI : RFCOMM_TYPE_RPN received, on listen channel %d", (dlci>>1));
            lc->upperQueueId = tc->pRFC->listenQueueID;
        }
        else
    	{
        	lc->upperQueueId = tc->pRFC->usQueueID;
    	}
#endif
    }

    /* if this is a response, send confirmation to upper layer (and DONE) */
    if (!tcr)
    {        
        return;
    }

    /* if this is a command, send indication to upper layer */
#if 0
    if (!newDesc)
	{
    	rfcSendURpn(lc->handle, rpn, (uint8_t)tlen);
	}
    else
    {
        /* reply the command with a response of local values */
        rfcSendUIH(tc, RFCOMM_TYPE_RPN, FALSE /* RESPONSE */, (uint8_t *) &lc->rpn, sizeof(lc->rpn));
    }
#else
	rfcSendUIH(tc, RFCOMM_TYPE_RPN, FALSE /* RESPONSE */, (uint8_t *) &lc->rpn, sizeof(lc->rpn));
#endif
}
/**
* @brief  handle rfcomm uid msg(dlci == 0)
* 
* @param  tc
* @param  pmsg
* @param  cr
* @param  poll
*
* @return  
*
*/

//void rfcHandleUIH(TrfcChannel * tc, MESSAGE_T *pmsg, uint8_t cr, uint8_t poll)
void rfcHandleUIH(TrfcChannel * tc, PBlueAPI_L2cDataInd pL2cDataInd, uint8_t cr, uint8_t poll)
{
    uint8_t   typ;
    uint8_t   tcr;	/*C/R bit2*/
    uint16_t   len =pL2cDataInd->length;// pmsg->MData.DataCBChan.Length;
    uint16_t   tlen;
    uint16_t   pos = 0;
   // uint8_t * p = pmsg->MData.DataCBChan.BufferAddress + pmsg->MData.DataCBChan.Offset;// buf + offset;
	uint8_t * p     =pL2cDataInd->buf+pL2cDataInd->dataOffset; 
   
    typ = p[pos++];
    tcr = typ & 2;
    typ >>= 2;
    tlen = p[pos++];
    if (tlen & EA_BIT)
	{
   		tlen >>= 1;
	}
    else
	{
    	tlen = (tlen >> 1) | (p[pos++]<<7);
	}

    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "rfcHandleUIH: dlci %X len %d, typ %X tlen %d", 4,tc->dlci, len, typ, tlen);

    if (poll)
    {
        DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "!!rfcHandleUIH: poll bit is set!!",0,0);
    }

    /* if this is a response, then stop the wait for response timer */
    if (!tcr && (typ != RFCOMM_TYPE_PN))
    {
        rfcResetTimer(RFCOMM_T1_ID, tc);
    }

    switch (typ)
    {
    case RFCOMM_TYPE_PN:
    	rfcHandleUIHPN(tc, tcr, p, pos, len);
        break;

    case RFCOMM_TYPE_FCON:
    {
        DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "rfcHandleUI: RFCOMM_TYPE_FCON received",0,0);
      //  rfcSendUFlowInd(tc->handle, 0, tcr, 0);
        tc->usFlow = 0;
      	 rfcNotifyUpstream(RFC_FLOW_IND, RFCOMM_SUCCESS, tc,NULL);
        if (tcr)
        {
            tc->dsFlow = FALSE;
            rfcSendUIH(tc, RFCOMM_TYPE_FCON, FALSE /* RESPONSE */, NULL, 0);
            rfcDownstreamWork();
        }
    }
        break;

    case RFCOMM_TYPE_FCOFF:
        DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "rfcHandleUI: RFCOMM_TYPE_FCOFF received",0,0);
      //  rfcSendUFlowInd(tc->handle, BLUEFACE_FLOW_BIT, tcr, 0);
       tc->usFlow = 1;
       rfcNotifyUpstream(RFC_FLOW_IND, RFCOMM_SUCCESS, tc,NULL);
        if (tcr)
        {
            tc->dsFlow = BLUEFACE_FLOW_BIT;
            rfcSendUIH(tc, RFCOMM_TYPE_FCOFF, FALSE /* RESPONSE */, NULL, 0);
        }
        break;

    case RFCOMM_TYPE_MSC:
    	rfcHandleUIHMSC(tc, tcr, p, pos);
        break;

    case RFCOMM_TYPE_RLS:
    	rfcHandleUIHRLS(tc, tcr, p, pos);
        break;

    case RFCOMM_TYPE_RPN:
    	rfcHandleUIHRPN(tc, tcr, p, pos, tlen);
        break;

    case RFCOMM_TYPE_TEST:
        DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "rfcHandleUI: RFCOMM_TYPE_TEST received, cr %X len %d", 2,tcr, tlen);
        if (!tcr)
            break;   /* ignore response */

        /* Send back identical data as response */
        rfcSendUIH(tc, RFCOMM_TYPE_TEST, 0 /* RESPONSE */, p+pos, tlen);
        break;

    case RFCOMM_TYPE_NSC:
        DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "rfcHandleUI: RFCOMM_TYPE_NSC received for type %X",1, p[pos]);

        /* NSC received: remote side does not understand us: shutdown all user channels and close the link! */
        rfcCloseChannels(tc->cc, RFCOMM_ERR | RFCOMM_ERR_NSC); /* Close user traffic channels on this link */
	//	l2cHandleBTG_DISC_REQ(tc->cc->channel, FALSE);
	mpa_Sendl2cDiscReq(tc->cc->channel);
		break;

    default:
        DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "rfcHandleUIH: !!unhandled Type %X", typ);
        if (cr) /* this is a command */
        {
            rfcSendUIH(tc, RFCOMM_TYPE_NSC, 0, &p[0], 1); /* reply with non supported command */
        }
		break;
    }
} /* rfcHandleUIH */

/**
* @brief  rfcomm handle sabm
* 
* @param  tc
* @param  command
* @param  poll
*
* @return  
*
*/
void rfcHandleSABM(TrfcChannel * tc, uint8_t command, uint8_t poll)
{
    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "RFC: SABM on dlci %X from bd %s", 2,tc->dlci,  tc->cc->bd);
    if (!command || !poll)
    {
        DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "!!RFC: SABM, no command or no poll",0,0);
        return;
    }
    if (tc->state == dlciConfigAccepted)
    {
        /* dlci is now open */
        rfcCast(tc, dlciConnected);
        rfcSendUA(tc, FALSE /* RESPONSE */, TRUE /* POLL */);
        /* send initial MSC command on channel... */
        tc->usFlowActive |=  1; 	 /* one MSC request outstanding */
        rfcSetTimer(RFCOMM_T1_ID, tc->cc, RFCOMM_T1_TIMEOUT);
        rfcSendMSC(tc->cc, TRUE /* COMMAND */, tc->dlci, tc->usFlow, 0 /* no break */);

        /* User channel is now connected, we must wait for MSC_IND before we can */
        /* signal to upper layer (Peiker - Verizone problem                      */

        DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "RFC SABM received, wait for MSC_IND",0,0);
        rfcSetTimer(RFCOMM_T2_ID, tc, RFCOMM_T2_TIMEOUT);
        return;
    }

    if (tc->state == dlciConnected)
    {
        /* dlci is already online, replace directly with UA */
        DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "!!rfcHandleSABM: SABM on Open Channel %X",1, tc->dlci);
        rfcSendUA(tc, FALSE /* RESPONSE */, TRUE /* POLL */);
        return;
    }

    if (tc->state == dlciIdle && tc->dlci != 0)
    {
       /* SABM on traffic channel received without prior TYPE_PN exchange: refuse by sending DM */
        DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "!!rfcHandleSABM: no PN on dlci %X",1, tc->dlci);
        rfcSendDM(tc->cc, tc->dlci, FALSE /* RESPONSE */, TRUE /* POLL */);
        rfcFreeChannel(tc);
        return;
    }

    rfcCast(tc, dlciIndicated);

    if (tc->dlci == 0)
    {
    	TBtConRespRfc	rfc;
	rfc.frameSize = tc->frameSize;
	rfc.maxCredits = 7;
	rfc.mscStatus = 0;
      /* send to self: accept control channel */
	rfcSendConResp(tc->handle, (tc->dlci>>1), 0, rfc);
    }
    else
    {
      //  rfcSendUConInd(tc->dlci, tc->cc->bd, tc->handle, tc->frameSize,
      //  				tc->convergenceLayer == RFCOMM_CONVERGENCE_CREDIT_CONF, FALSE);
     	//	 rfcNotifyUpstream(RFC_CONNECT_IND,0,tc);
     /*	TBtConRespPSpecifc TExt;
		
	TExt.rfc.frameSize = tc->frameSize;
	TExt.rfc.maxCredits = 7;
	TExt.rfc.mscStatus = 0;
	rfcSendConResp(tc->handle, (tc->dlci>>1), 0, rfc); //move to app
	*/
	rfcNotifyUpstream(RFC_CONNECT_IND, RFCOMM_SUCCESS, tc,NULL);
	
    }
} /* rfcHandleSABM */

/**
* @brief  rfcomm handle UA
* 
* @param  tc
* @param  command
* @param  poll
*
* @return  
*
*/
void rfcHandleUA(TrfcChannel * tc, uint8_t command, uint8_t poll)
{
    if (command || !poll)
    {
        DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "!!RFC: UA, command or not  final",0,0);
        return;
    }
    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "RFC: UA",0,0);
    if (tc->state == dlciDisconnecting)
    {
		/* we are closing this channel */
		if (tc->dlci)
		{
		   // blueAPI_Handle_RFC_DISC_IND(tc->handle, tc->dlci, 0);
		    //nofity hfp ? 
		}
		rfcResetTimer(RFCOMM_T1_ID, tc);
		rfcCast(tc, dlciIdle);
		if (tc->dlci == 0)               /* Closing the control channel */
		{
		    rfcCloseChannels(tc, RFCOMM_ERR | RFCOMM_SUCCESS);                    /* Close user traffic channels on this link */
			//l2cHandleBTG_DISC_REQ(tc->channel, FALSE);
			mpa_Sendl2cDiscReq(tc->channel);
		}
		else
		{
		    rfcDisconnectIfNoUserChannels(tc->cc);
		    rfcFreeChannel(tc);
		}
		return;
    }

    /* UPF6: axis is sending one additional UA in an open RFCOMM session, ignore that */
    if (tc->state == dlciConnected || tc->state == dlciIdle)
    {
        DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "!!RFC: UA on channel in state %d, ignored", 1,tc->state);
        return;
    }

    /* we are opening this channel */
    rfcResetTimer(RFCOMM_T1_ID, tc);
    rfcCast(tc, dlciConnected);
    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "RFC: dlci %x now connected", 1,tc->dlci);
    if (tc->dlci == 0) /* Control channel is now connected */
    {
        int i;
			
#if 0			
        for (i = 0; i < RFCOMM_MAX_DLCI; i++)
        {
            TrfcChannel * lc = &tc->pRFC->cl[i];
            if (lc->used && lc->state == dlciOpening && lc->cc == tc)
            {
                rfcSendSecManAuthenticationInd(lc, 1 /* outgoing */, 1 /* active */);
            } /* opening channel found */
        } /* for all channels */
#else
        for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_data_rfcom_dlci_don; i++)
        {
            TrfcChannel * lc = &pRFC->rfcchannelDon[i];
            if (lc->used && lc->state == dlciOpening && lc->cc == tc)
            {
                rfcSendSecManAuthenticationInd(lc, 1 /* outgoing */, 1 /* active */);
			return;
            } /* opening channel found */
        } /* for all channels */

        for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_data_rfcom_dlci_doff; i++)
        {
            TrfcChannel * lc = &pRFC->rfcchannelDoff[i];
            if (lc->used && lc->state == dlciOpening && lc->cc == tc)
            {
                rfcSendSecManAuthenticationInd(lc, 1 /* outgoing */, 1 /* active */);
            } /* opening channel found */
        } /* for all channels */				
#endif
    } /* control channel */
    else
    {
		/* User channel is now connected, we must wait for MSC_IND before we can */
		/* signal to upper layer (Peiker - Verizone problem                      */
		DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "RFC UA received, wait for MSC_IND",0,0);
		rfcSetTimer(RFCOMM_T2_ID, tc, RFCOMM_T2_TIMEOUT);
		/* Send inital MSC command */
       	tc->usFlowActive |=  1; 	 /* one MSC request outstanding */
		rfcSetTimer(RFCOMM_T1_ID, tc->cc, RFCOMM_T1_TIMEOUT);
		rfcSendMSC(tc->cc, TRUE /* COMMAND */, tc->dlci, tc->usFlow, 0 /* no break */ );
    } /* other channels */
} /* rfcHandleUA */

/**
* @brief  rfcomm handle DM
* 
* @param  tc
* @param  command
* @param  poll
*
* @return  
*
*/
void rfcHandleDM(TrfcChannel * tc, uint8_t command)
{
    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "rfcHandleDM dlci %X",1, tc->dlci);
    if (command)
    {
        /* short comment: do not check for poll bit setting in DM (might be set/not set) */
        DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "!!rfcHandleDM: illegal command setting in DM dlci %X", 1,tc->dlci);
        return;
    }
    if (!tc->used || tc->state == dlciIdle)
    {
        DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "!!rfcHandleDM: illegal descriptor state dlci %X", 1,tc->dlci);
        return;
    }
    rfcResetTimer(RFCOMM_T1_ID, tc);
    if (tc->dlci) /* this is a user channel */
    {
        switch (tc->state) 
		{
        case dlciConfigured:
            /* if the channel is only configured (pn exchanged, then no upper layer visible
                   action occured, do not notify upper layer in this case
                   */
            break;
        default:
           // blueAPI_Handle_RFC_DISC_IND(tc->handle, tc->dlci, RFCOMM_ERR | RFCOMM_ERR_REJECTED);
             //nofity hfp ? 
			break;
        } /* switch */
        rfcCast(tc, dlciIdle);
        rfcDisconnectIfNoUserChannels(tc->cc);
    }
    else
    {
        /* DM on control channel */
        rfcCast(tc, dlciIdle);
        rfcCloseChannels(tc, RFCOMM_ERR | RFCOMM_ERR_REJECTED);
	//	l2cHandleBTG_DISC_REQ(tc->channel, FALSE);
	 mpa_Sendl2cDiscReq(tc->channel);
    }
    rfcFreeChannel(tc);
    rfcDownstreamWork();
} /* rfcHandleDM */

/**
* @brief  rfcomm handle DISC
* 
* @param  tc
* @param  command
* @param  poll
*
* @return  
*
*/
void rfcHandleDISC(TrfcChannel * tc, uint8_t command, uint8_t poll)
{
     DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "rfcHandleDISC dlci %X", 1,tc->dlci);
     if (!command || !poll)
     {
         DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "!!rfcHandleDISC: not command or not poll",0,0);
         return;
     }
     switch (tc->state)
     {
     case dlciConnected:
#if (X_GENERATE_COLLISION)
         if (tc->dlci)
         {
             DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "!!RFCOMM: Generate DISC Collision dlci %x", 1,tc->dlci);
             rfcSendDISC(tc, TRUE /* COMMAND */, TRUE /* POLL */ );
             rfcCast(tc, dlciDisconnecting);
             return;
         }
#endif
         rfcSendUA(tc, FALSE /* RESPONSE */, TRUE /* POLL */ );
         break;

     case dlciDisconnecting:
         DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO,"RFCOMM: DISC Collision dlci %x",1,tc->dlci);
         rfcSendUA(tc, FALSE /* RESPONSE */, TRUE /* POLL */ );

         /* we handle this internally a if a UA was received */
         if (tc->dlci)
         {
            // blueAPI_Handle_RFC_DISC_IND(tc->handle, tc->dlci, 0);
            //notify hfp profile?
         }
         rfcResetTimer(RFCOMM_T1_ID, tc);
         rfcCast(tc, dlciIdle);
         if (tc->dlci == 0)               /* Closing the control channel */
         {
             rfcCloseChannels(tc, RFCOMM_ERR | RFCOMM_SUCCESS);                    /* Close user traffic channels on this link */
		//	 l2cHandleBTG_DISC_REQ(tc->channel, FALSE);
		mpa_Sendl2cDiscReq(tc->channel);
         }
         else
         {
             rfcDisconnectIfNoUserChannels(tc->cc);
             rfcFreeChannel(tc);                        /* Free this channel */
         }
         return;

     default:
         /* this is a DISC on a not open channel */
         rfcSendDM(tc->cc, tc->dlci, FALSE /* RESPONSE */, TRUE /* POLL */ );
		 break;
	}

	if (tc->dlci)
	{
		//blueAPI_Handle_RFC_DISC_IND(tc->handle, tc->dlci, 0);
	}
	rfcCast(tc, dlciIdle);
	if (tc->dlci == 0)
	{
		rfcCloseChannels(tc, RFCOMM_ERR | RFCOMM_ERR_REJECTED);
		/* close of l2cap link will be done by initiating peer */
	}
	else
	{
		rfcFreeChannel(tc);
	}

     rfcDownstreamWork();
} /* rfcHandleDisc */




/**
* @brief  rfcomm handle rpn response
*
* @param  handle
* @param  rpn
* @param  len
*
* @return  
*
*/
#if 0
void rfcHandleRfcRpnResp(uint8_t handle, uint8_t* rpn, uint16_t len)
{
	TrfcChannel * tc;
	Trpn trpn;
	DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "rfcHandleRfcRpnResp: handle %X length %d", 2,handle, len);

	tc = rfcFindHandle(handle);
	if (!tc) /* do not check state here, because RPN_RESP might occur before SABM / UA exchange */
	{
		DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "!!rfcHandleRfcRpnResp: could not find or wrong state ", 0,0);
		return;
	}
	memcpy((void*)&trpn, rpn, len);
	trpn.dlci = (tc->dlci<<2) | 2 | EA_BIT;
	rfcSendUIH(tc->cc, RFCOMM_TYPE_RPN, 0 /* RESPONSE */, (uint8_t *) &trpn, len);
}
#endif 

/**
* @brief  rfcomm handle connect request from upstream
*
* @param  bd: 
* @param  flowControlMode
* @param  serverChannel
* @param  frameSize
* @param  status
* @param  maxCredits
* @param  uuid
*
* @return  
*
*/
//void rfchandleUpConReq(LPCuint8_t bd, uint8_t flowControlMode, uint8_t serverChannel, uint16_t frameSize, uint8_t status, uint8_t maxCredits, uint16_t uuid)
RfcResult rfcHandleUpConReq(TBdAddr bd, uint8_t serverChannel, uint16_t frameSize, uint8_t status,  uint8_t maxCredits,uint16_t uuid)
{
	uint8_t dlci;
	TrfcChannel * tc;   /* Traffic Channel */
	TrfcChannel * cc;   /* Control Channel */
	bool newCC = FALSE;

	DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO,
						"rfchandleUpConReq: chan %x bd %s frameSize %X status %X maxCredits %d",
						5,serverChannel,  bd,
						frameSize, status, maxCredits);

	if (frameSize == 0)
	{
		frameSize = BT_RFCOMM_MTU_COUNT;
		DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "rfchandleUpConReq: ServerChan %x set default frameSize to %d", 2,serverChannel, frameSize);
	}

	dlci = serverChannel << 1;
	cc = rfcFindBd(bd, 0);
	if (!cc) /*this is a new session, allocat control link*/
	{
		DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "rfcHandleRfcConReq: new connection to %s ServerChan %x",  2,bd, serverChannel);
		cc = rfcAllocateDLCI();
		if (!cc)
		{
			DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "!!rfcHandleRfcConReq: could not allocate cc",0,0);
//			blueAPI_Handle_RFC_CON_CONF(dlci, (uint8_t *)bd, 0, RFCOMM_ERR + RFCOMM_ERR_NORESOURCES);
			return RFC_NO_RESOURCE;
		}
		rfcInitDLCI(cc, 0);
		cc->cc           = cc;					/*control channel is itself ?*/
		memcpy(cc->bd, bd, BD_ADDR_SIZE);
		newCC            = TRUE;
	}
	if (cc->state == dlciIdle || cc->initiator)
	{
		/* we are / will be initiator of the connection: our direction bit is 1 */
		/* the direction bit of the remote side is "0" */
	}
	else
	{
		dlci |= 1;
	}

	/* now the DLCI also contains the direction bit */
	tc =rfcFindBd(bd, dlci);
	if (tc)
	{
		DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "!!rfcHandleRfcConReq: link already exists dlci %x", 1,dlci);
		//blueAPI_Handle_RFC_CON_CONF(dlci, (uint8_t *)bd, 0, RFCOMM_ERR + RFCOMM_ERR_ILL_PARAMETER);		
		if (newCC)
		{
			/* kill the assoc control link */
			cc->used = FALSE;
		}
		return  RFC_WRONG_STATE;
	}
	/* prepare channel */
	tc = rfcAllocateDLCI();
	if (!tc)
	{
	//	blueAPI_Handle_RFC_CON_CONF(dlci,(uint8_t *) bd, 0, RFCOMM_ERR + RFCOMM_ERR_NORESOURCES);	
		if (newCC)
		{
			/* kill the assoc control link */
			cc->used = FALSE;
		}
		return RFC_NO_RESOURCE;
	}
	rfcInitDLCI(tc, dlci);
	rfcAllocateBuffers(tc, maxCredits);
	tc->cc           = cc;
	tc->usFlow       = status;  /* Copy status value for MSC command into descriptor */
	tc->uuid         = uuid;
	rfcCast(tc, dlciOpening);
	tc->frameSize = frameSize;

	/* send message to upper layer: accepted */
//	blueAPI_Handle_RFC_CON_CONF(dlci, (uint8_t *)bd, tc->handle, 0 /* no error */);
	if (cc->lstate == linkIdle) /* no l2cap connection */
	{
#if 0
		LP_L2C_Conf_para pConfPara;

		pConfPara = commonPrepareL2cConfPara(RFCOMM_HEADER_SIZE + BT_RFCOMM_MTU_COUNT, BT_US_PDU_L2C_uint8_t_COUNT, 0);
		if(pConfPara == 0)
		{
			DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "!!rfchandleUpConReq: out of memory", 0,0);
			blueAPI_Handle_RFC_CON_CONF(dlci, (uint8_t *)bd, 0, RFCOMM_ERR + RFCOMM_ERR_NORESOURCES);
			if (newCC)
			{
				/* kill the assoc control link */
				cc->used = FALSE;
			}
			return;
		}
		l2cUSendConReq(rfcQueueID, (LPBYTE)bd, PSM_RFCOMM, uuid, 0 /* frameSize */, pConfPara);
#endif
		mpa_Sendl2cConReq(PSM_RFCOMM,UUID_RFCOMM,pRFC->QueueID, RFCOMM_HEADER_SIZE + BT_RFCOMM_MTU_COUNT,bd);
		cc->linitiator = TRUE;
//		cc->uuid       = uuid; /*conctrol DLCI 0 shall not have uuid*/
		rfcLCast(cc, linkConnecting);
		return RFC_SUCCESS;
	}
	if (cc->lstate != linkConnected)
	{
		return RFC_WRONG_STATE;
	}
	if (cc->state == dlciIdle)
	{
		cc->initiator = TRUE;
		rfcCast(cc, dlciConnecting);
		rfcSetTimer(RFCOMM_T1_ID, cc, RFCOMM_T1_TIMEOUT_EXTENDED);
		rfcSendSABM(cc, TRUE /* COMMAND */, TRUE /* POLL */);
	}
	if (cc->state != dlciConnected) /*control channel do not need send Authentication*/
	{
		return  RFC_SUCCESS;  
	}
	/* connect user channel*/
	rfcSendSecManAuthenticationInd(tc, 1 /* outgoing */, 1 /* active */);	
	return RFC_SUCCESS;
}
/**
* @brief  rfcomm handle disconnect request from upstream
* 
* @param  cid
*
* @return  
*
*/
RfcResult rfcHandleUpDiscReq(uint16_t handle )//uint16_t cid
{
    TrfcChannel * tc;

    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "rfcHandleUpDiscReq CID %X",1, handle);
    tc = rfcFindHandle(handle);//cid
    if (!tc)
    {
        DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "!!rfcHandleRfcDiscReq could not find descr for CID %X",1, handle);//cid
   //     blueAPI_Handle_RFC_DISC_IND(cid, 0 /* do not know dlci */, RFCOMM_ERR | RFCOMM_ERR_ILL_PARAMETER);
        return RFC_WRONG_STATE;
    }

    if (tc->state == dlciIdle)
    {
        DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "!!rfcHandleRfcDiscReq CID %X is idle",1, handle);
   //     blueAPI_Handle_RFC_DISC_IND(cid, tc->dlci, RFCOMM_ERR | RFCOMM_ERR_ILL_PARAMETER);
        return RFC_WRONG_STATE;
    }

    if (tc->cc->lstate == linkConnecting && tc->cc->channel==0)
    {
        DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "!!RFC_DISC_REQ on connecting L2CAP CID %X",1, handle);
        rfcLCast(tc->cc, linkDisconnectRequested);
        return RFC_WRONG_STATE;
    }

    if (tc->state != dlciConnected)
    {
      DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "RFC_DISC_REQ on opening channel CID %X state %d",2,
           						handle, tc->state);
      rfcCloseChannels(tc->cc, 0);                      /* Close user traffic channels on this link */
	//rfcDisconnectIfNoUserChannels is better ??
	//	l2cHandleBTG_DISC_REQ(tc->cc->channel, FALSE);
	mpa_Sendl2cDiscReq(tc->cc->channel);
	rfcLCast(tc->cc, linkDisconnectRequested);
      return RFC_SUCCESS;
    }

    rfcCast(tc, dlciDisconnecting);
    rfcSetTimer(RFCOMM_T1_ID, tc, RFCOMM_T1_TIMEOUT);
    rfcSendDISC(tc, TRUE /* COMMAND */, TRUE /* POLL */ );

    /* Link status is changed, call rfcDownstreamWork to check for blocked data etc */
    rfcDownstreamWork();
    return RFC_SUCCESS;
}

/**
* @brief  rfcomm handle flow request
*
* @param  handle
* @param  status
* @param  sbreak 
*
* @return  
*
*/
RfcResult rfcHandleUpFlowReq(uint8_t handle, uint8_t flowStatus, uint8_t sbreak)
{
	TrfcChannel * tc;

	DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "rfcHandleRfcFlowReq: handle %X status %X",2, handle, flowStatus);
	tc = rfcFindHandle(handle);
	if (!tc || tc->state != dlciConnected)
	{
		DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "!!rfcHandleRfcFlowReq: could not found or wrong state ", 0,0);
		return RFC_WRONG_STATE;
	}

#if (X_TEST_AGG_FLOW)
	/* technique to support agg flow commands: if bit 6 is set (normally unused),
	perform agg flow according to flow bit, other bits ignored
	*/
	if (flowStatus & 0x40)
	{
		DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "RFC: agg flow %x",1, flowStatus);
		tc = tc->cc; /* switch to control channel */
	}
	/* shortcut to create a rfcomm test request */
	if (flowStatus & 0x20)
	{
		rfcSendUIH(tc->cc, RFCOMM_TYPE_TEST,
					1 /* COMMAND */,
					"TEST",
					4
					);
					return;
	}
#endif

	/* store new flow state */
	tc->usFlow      = flowStatus;
	tc->usFlowBreak = sbreak;

	if (tc->dlci==0)
	{
		/* Flow Control Command from App Layer on Control Channel, this is aggregate flow control */
		rfcSetTimer(RFCOMM_T1_ID, tc, RFCOMM_T1_TIMEOUT);
		rfcSendUIH(tc, (uint8_t)((tc->usFlow & BLUEFACE_FLOW_BIT)==0 ? RFCOMM_TYPE_FCON : RFCOMM_TYPE_FCOFF),
					TRUE /* COMMAND */, NULL, 0);
	} /* DLCI == 0 */
	else
	{
		/* generateMSC_CONF locally and immediate */
		//rfcSendUFlowInd(tc->handle, tc->usFlow, FALSE /* response */, tc->usFlowBreak);
		rfcNotifyUpstream(RFC_FLOW_IND, RFCOMM_SUCCESS, tc,NULL);
		/* Flow Control Command from App Layer on User Channel, send Modem Status Command to Peer */
		if (tc->usFlowActive & 1)
		{
			/* there is an outstanding MSC on this link: do not send now, but only store parameter values (already done)
			and remember the fact in bit 1*/
			tc->usFlowActive |= 2;
		} 
		else
		{
			/* there is currently no outstanding MSC, send the MSC request and indicate MSC outstanding */
			tc->usFlowActive |= 1;
			rfcSetTimer(RFCOMM_T1_ID, tc->cc, RFCOMM_T1_TIMEOUT);
			rfcSendMSC(tc->cc, 1 /* COMMAND */, tc->dlci, tc->usFlow, sbreak);
		} /* no outstanding MSC */
	} /* DLCI != 0 */
	return RFC_SUCCESS;
}

/**
* @brief  handle rfcomm data request from upstream
*
* @param  pRFC:
* @param  msg: 
*
* @return 
*
*/
RfcResult rfcHandleUpDataReq( TBdAddr bd, uint8_t dlci,uint8_t* pbuffer,uint32_t length)
{
	TrfcChannel * tc;
	TRfcUpDataMsg temp;
	temp.Offset= pRFC->writeOffset
							+ RFCOMM_SIZE_UIH  /* rfcomm header length */
							+ 1;               /* for credit field */

	//tc = rfcFindHandle((uint8_t)handle);
	tc = rfcFindBd(bd, dlci);
	if (!tc)
	{
		DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "!!rfcHandleRfcDataReq: illegal dlci %X",1, dlci);
		return RFC_WRONG_STATE;//return 0;
	}
	if ((length > tc->frameSize) || (length == 0))
	{
		DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "rfcHandleUpDataReq: length not right", 0, 0);
	}
	
	temp.payloadLength = length;
	if (blueAPI_CauseSuccess ==
		blueAPI_BufferGet(tc->dsPoolID, 
						temp.payloadLength, 
						temp.Offset, 
						(void * *)&temp.BufferAddress) )
	{
		memcpy(temp.BufferAddress + temp.Offset,pbuffer, length);
		if (xQueueSend(tc ->dsQueueID, &temp, 0) == errQUEUE_FULL)
		{
			DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "rfcHandleUpDataReq: data queue if full", 0, 0);
		}
		rfcDownstreamWork();
		//return length;
		return RFC_SUCCESS;
	}
	else
	{
		DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "rfcHandleUpDataReq: blueAPI_BufferGet failed", 0, 0);
		return RFC_NO_RESOURCE;
	}	
	
}
/**
* @brief  rfcomm handle data response from remote device
*
* @param  handle: handle
*
* @return  
*
*/
RfcResult rfcHandleUpDataResp(uint8_t handle)
{
	TrfcChannel  * tc;

	tc = rfcFindHandle(handle);
	if (!tc)
	{
		DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "!!rfcHandleRfcDataResp: illegal handle %X", 1,handle);
		return RFC_WRONG_STATE;
	}

	if (tc->convergenceLayer == RFCOMM_CONVERGENCE_CREDIT_CONF && tc->state != dlciDisconnecting)
	{
		tc->backCredits++;
		DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "rfcHandleRfcDataResp: dlci %X backCredits %X now",
									  2,tc->dlci, tc->backCredits);
	}

	rfcDownstreamWork();
	return RFC_SUCCESS;
}

/**
* @brief  Notify upstream that rfcomm state has changed 
*
* @param  pRFC:
* @param  msg: 
*
* @return 
*
*/
void rfcNotifyUpstream(RfcMsgType msg_type,uint16_t status,TrfcChannel * pRfcChannel,PBlueAPI_L2cDataInd pL2cDataInd)
{
	pRfcProCallback protocol_callback;
	uint8_t index = 0;
	TRfcProMsg rfc_msg;
	PRfcProDesc tempDesc = NULL;
	 for (index = 0; index < MAX_RFCPRO_NUM; index++)
	{
		tempDesc = &pRfcProDes[index];
	      if (tempDesc->uuid == pRfcChannel->uuid)
	      {	        	
			 if (tempDesc->cb != NULL)
		        {
				rfc_msg.status = status;
				if(pRfcChannel!=NULL)
				{
					rfc_msg.handle = pRfcChannel->handle;
					rfc_msg.dlci = pRfcChannel->dlci;
					rfc_msg.uuid =pRfcChannel->uuid;
					memcpy(rfc_msg.bd, pRfcChannel->bd, BD_ADDR_SIZE); /* Remember BD of Remote Side */
					rfc_msg.frameSize = pRfcChannel->frameSize;
					rfc_msg.creditBased = 0;// pRfcChannel->convergenceLayer
					rfc_msg.fc = pRfcChannel->usFlow;	
					//		rfc_msg.outgoing =
				}	 
				if(RFC_DATA_IND == msg_type && pL2cDataInd != NULL )
				{
					rfc_msg.buf = pL2cDataInd->buf+pL2cDataInd->dataOffset; 
					rfc_msg.dataLength =  pL2cDataInd->length;
				}
				protocol_callback = (pRfcProCallback)tempDesc->cb;
			 	protocol_callback(msg_type,rfc_msg);
		        }
	        }
	        
	}
}
/**
* @brief  rfcomm handle l2cap messages
*	register to mpa layer
* @param  buf pointer to msg buffer
* @param  l2c_msg l2c msg type
*
* @return  
*
*/
void rfcCallBack(void* buf, ProtocolUsMsg l2c_msg)
{
	switch(l2c_msg)
	{
	case L2CAP_CONNECT_IND:
		rfcHandleL2cConInd((PBlueAPI_L2cConInd)buf);
		break;
	case L2CAP_CONNECT_RSP:
		rfcHandleL2cConRsp((PBlueAPI_L2cConRsp)buf);
		break;
	case L2CAP_CONNECT_COMPLETE:
		rfcHandleL2cConComplete((PBlueAPI_L2cConActInd)buf);
		break;
	case L2CAP_DATA_IND:
		rfcHandleL2cDataInd((PBlueAPI_L2cDataInd)buf);
		break;
	case L2CAP_DISC_IND:
		rfcHandleL2cDiscInd((PBlueAPI_L2cDiscInd)buf);
		break;
	case L2CAP_DISC_RSP:
		rfcHandleL2cDiscRsp((PBlueAPI_L2cDiscConf)buf);
		break;
	case PROTOCOL_SECURITY_REGISTER_RSP:
		rfcHandleSecRegRsp((PBlueAPI_L2cSecurityRegisterRsp)buf);
		break;
	case PROTOCOL_AUTHORIZATION_IND:
		rfcHandleAuthorizationInd((PBlueAPI_UserAuthorizationReqInd)buf);
		break;
	default:
		DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR,"rfcCallBack: unknown msg",0,0);
	}
}

/**
* @brief  rfcomm handle l2cap connection indicate
*	connect Initated by remote device
* @param  localCID
* @param  remote_bd
*
* @return  
*
*/
void rfcHandleL2cConInd (PBlueAPI_L2cConInd pL2cConInd)
{
	TrfcChannel * cc;
	uint16_t acceptCall = L2CAP_ERR_REFUS_NO_RESOURCE;

	DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "rfcHandleL2cConInd: cid %X bd %s", 2, pL2cConInd->cid,pL2cConInd->remote_BD);
	cc = rfcAllocateDLCI();
	if (!cc)
	{
		DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "!!rfcHandleL2cConInd: could not allocate descriptor", 0,0);
	}
	else
	{			
		acceptCall = L2CAP_CONNECTION_ACCEPT;
		rfcInitDLCI(cc, 0);					   /* Init Descriptor for Link and Control Channel */
		cc->channel		= pL2cConInd->cid;
		cc->cc			= cc;				   /* We are our own control channel ! */
		memcpy(cc->bd, pL2cConInd->remote_BD, BD_ADDR_SIZE); /* Remember BD of Remote Side */
		rfcLCast(cc,linkConnecting);
	}
	mpa_Sendl2cConConf(acceptCall, cc->channel);	
}


/**
* @brief  rfcomm handle l2cap connect confirm
* local init connect request, l2cap send this  response message intermedia
* @param  remote_bd
* @param  cid
* @param  status
*
* @return  
*
*/

void rfcHandleL2cConRsp(PBlueAPI_L2cConRsp pL2cConRsp)
{
	TrfcChannel * cc;

	DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "!!rfcHandleL2cConConf:  bd %s cid %X status %X", 3, pL2cConRsp->remote_BD,pL2cConRsp->cid,pL2cConRsp->status);
	/*tc do not set bd addr, so here just found cc*/
	cc = rfcFindBd(pL2cConRsp->remote_BD, 0);
	if (!cc)
	{
		DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "!!rfcHandleL2cConConf: cannot find bd %s", 1, pL2cConRsp->remote_BD);
		return;
	}

	if (pL2cConRsp->status) /* error case  */
	{
		rfcCloseChannels(cc, pL2cConRsp->status);
		rfcLCast(cc,linkIdle);
		rfcFreeChannel(cc);
		return;
	} /* error case */

	cc->channel = pL2cConRsp->cid;

	//FIXME JW not needed anymore
	if (cc->lstate == linkDisconnectRequested)
	{
		DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "!!rfcHandleL2cConConf: disc requested cid %d", 1, pL2cConRsp->cid);
		rfcCloseChannels(cc, pL2cConRsp->status);
		mpa_Sendl2cDiscReq(pL2cConRsp->cid);
		return;
	} /* Disconnect Requested */
}

/**
* @brief  rfcomm handle l2cap connect act indicate
*  l2cap connect complete
* @param  cid
* @param  status
* @param  mtuSize
*
* @return  
*
*/
void rfcHandleL2cConComplete(PBlueAPI_L2cConActInd pL2cConActInd)
{
	TrfcChannel * cc;
	DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "rfcHandleL2cConActInd: cid %X ", 1, pL2cConActInd->cid);
	cc = rfcFindDLCI(0, pL2cConActInd->cid);
	if (!cc)
	{
		DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "!!rfcHandleL2cConActInd: cid  not found",0,0);
		mpa_Sendl2cDiscReq(pL2cConActInd->cid);
		return;
	}
	cc->mtuSize = pL2cConActInd->remoteMTU;
	cc->dsPoolID = pL2cConActInd->dsPoolID;
	rfcLCast(cc, linkConnected);

	if (cc->state == dlciIdle && cc->linitiator)
	{
		cc->initiator = TRUE;
		rfcCast(cc, dlciConnecting);
		rfcSetTimer(RFCOMM_T1_ID, cc, RFCOMM_T1_TIMEOUT_EXTENDED);
		rfcSendSABM(cc, TRUE /* COMMAND */, TRUE /* POLL */);
	}
}

/**
* @brief  rfcomm handle l2cap disconnect indicate
*
* @param  localCID
* @param  mtuSize
*
* @return  
*
*/

void rfcHandleL2cDiscInd(PBlueAPI_L2cDiscInd pL2cDiscInd)
{
	TrfcChannel * cc;

	DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "rfcHandleL2cDiscInd: cid %X status %X", 2, pL2cDiscInd->cid,pL2cDiscInd->cause);
	mpa_Sendl2cDiscConf(pL2cDiscInd->cid);
	cc = rfcFindDLCI(0, pL2cDiscInd->cid);
	if (!cc)
	{
		DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "!!rfcHandleL2cDiscInd:channel  not found",0,0);
		return;
	}

	/* close the link */
	rfcLCast(cc, linkIdle);
	/* close all open channels */
	rfcCloseChannels(cc,pL2cDiscInd->cause);
	/* free the control channel */
	rfcFreeChannel(cc);

}

/**
* @brief  rfcomm handle l2cap disconnect confirm
*
* @param  localCID
* @param  result
*
* @return  
*
*/

void rfcHandleL2cDiscRsp(PBlueAPI_L2cDiscConf pL2cDiscConf)
{
	TrfcChannel * cc;

	DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "rfcHandleL2cDiscConf: cid %X ", 1, pL2cDiscConf->cid);

	cc = rfcFindDLCI(0, pL2cDiscConf->cid);
	if (!cc)
	{
		DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "!!rfcHandleL2CDiscConf: channel not found ", 0,0);
		return;
	}
	/* close the link */		
	rfcLCast(cc, linkIdle); 
	/* close all open channels */
	rfcCloseChannels(cc,0);
	rfcFreeChannel(cc);
}

/**
* @brief  rfcomm handle data indicate
* 
* @param  pRFC
* @param  pmsg
*
* @return   
*
*/
void rfcHandleL2cDataInd(PBlueAPI_L2cDataInd pL2cDataInd)
{
    uint8_t * p     =pL2cDataInd->buf+pL2cDataInd->dataOffset; //pmsg->MData.DataCBChan.BufferAddress + pmsg->MData.DataCBChan.Offset;
    uint16_t len     =pL2cDataInd->length; //pmsg->MData.DataCBChan.Length;
    uint16_t channel = pL2cDataInd->cid;//pmsg->MData.DataCBChan.Channel;
    uint16_t pos = 0;
    TrfcChannel * tc = NULL;
    uint8_t dlci = 0;
    uint8_t cr = 0;                   /* cr bit from frame */
    uint8_t ctl = 0;
    uint8_t poll = 0;                 /* poll bit from frame */
    uint16_t rlen = 0;
//    bool releaseBuffer = TRUE; /* a reasonable default */
    bool crcok;
    uint8_t returnCredits = 0;

    if (len < 4)
    {
    //    RFCOMM_TRACE_PRINTF_1(RFCOMM_TRACE_MASK_ERROR, "!!rfcHandleDataInd: frame too short (len %d) - discard)", len);
    	DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "!!rfcHandleDataInd: frame too short (len %d) - discard) ",1,len);
		
    }
    else
    {
        dlci = p[pos] >> 2;
        cr   = (p[pos++] >> 1) & 1;
        ctl  = p[pos] & ~RFCOMM_POLL;
        poll = p[pos++] & RFCOMM_POLL;
        rlen = p[pos++];
        if (rlen & 1)
        {
			rlen >>= 1;
        }
        else
        {
            rlen >>= 1;
            rlen |= (uint16_t) p[pos++] << 7;
        }

        if (dlci && (ctl == RFCOMM_UIH) && poll)
        {
            /* this is a UIH frame on traffic channel mit poll bit set: it contains backCredits field! */
            returnCredits = p[pos++]; /* fetch the return credits and remove the field from the payload */
        }
	DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "RFC: DATA_IND len %d DLCI %X CR %X Control %X PF %X RLEN %d",8,
												len, dlci, cr, ctl, poll, rlen, 0, 0);
	
        /* make some additional syntax checks on the frame */
        if ((pos + rlen + 1) > len)
        {
        	DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR,"!!RFC: DATA_IND length mismatch len %d pos %d rlen %d",4, len, pos, rlen, 0);
        }
        else
        {
            /* check if the rfcomm contents is smaller than the l2cap frame:
               if this is the case, we tolerate (garbage at the end of the frame
             */
            if ((pos + rlen + 1) < len)
            {
              DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR,"!!RFC: DATA_IND rfc length smaller than l2cap len %d pos %d rlen %d",4, len, pos, rlen, 0);
            }

            crcok = crc8EtsCheck(p, (uint16_t)(ctl == RFCOMM_UIH ? 2 : 3), p[pos+rlen]);
            if (!crcok)
            {
            	DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "!!rfcHandleInd: CRC ERROR",0,0);
                /* discard : by not getting a tc ! */
            }
            else
            {
                tc = rfcFindDLCI(dlci, channel);
                if (!tc && ctl == RFCOMM_SABM)
                /* channel of SABM not yet existing: allocate and initialize. the control channel
                   should always exisit, because it is initialized together with the l2cap link
                */
                {
                    tc = rfcAllocateDLCI();
                    if (tc)
                    {
                        TrfcChannel * cc;
                        rfcInitDLCI(tc, dlci);
				cc = rfcFindDLCI(0, channel);
                        if (!cc)
                        {
                       //     RFCOMM_TRACE_PRINTF_2(RFCOMM_TRACE_MASK_ERROR, "!!rfcHandleDataInd: could not find control channel dlci %x channel %x", dlci, channel);
          			DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "!!rfcHandleDataInd: could not find control channel dlci %x channel %x",2, dlci, channel);
				cc = tc; /* prevent tc->cc from being set to null, enforce to have correct cc setting */
                        }
                        tc->cc = cc;
                    }
                }

                if (!tc && (ctl != RFCOMM_DM))
                {
                    /* we ignore DM on non existing channels! */
               //     RFCOMM_TRACE_PRINTF_2(RFCOMM_TRACE_MASK_ERROR, "!!RFC: command %X on close channel dlci %X", ctl, dlci);
               	DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "!!RFC: command %X on close channel dlci %X",2, ctl, dlci);
                    if (dlci)
                    {
                        TrfcChannel * cc;
                        cc = rfcFindDLCI(0, channel);
                        if (cc &&
                            ((cr && !cc->initiator) || (!cr && cc->initiator))
                           )
                        {
                         //   RFCOMM_TRACE_PRINTF_2(RFCOMM_TRACE_MASK_ERROR, "!!RFC: command %X on close channel dlci %X, sending DM", ctl, dlci);
				DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "!!RFC: command %X on close channel dlci %X, sending DM",2, ctl, dlci);
                          rfcSendDM(cc, dlci, FALSE /* RESPONSE */ , FALSE /* NOT POLL */);
                        } /* control channel found */
                    } /* dlci */
                } /* traffic channel not found */
            } /* no crc error */
        } /* no length mismatch */
    } /* frame not too short */

    if (tc)
    {
        uint8_t command;  /* command flag (extracted from frame and role (initiator) */

        /* frame coning from peer: it is a command if cr=1 and peer is initiator or
           if cr=0 and peer is not initiator
        */
        if ((cr && !tc->cc->initiator) || (!cr && tc->cc->initiator))
            command = TRUE;
        else
            command = FALSE;

        switch (ctl)
        {
        case RFCOMM_SABM:
            rfcHandleSABM(tc,command, poll);
            break;

        case RFCOMM_UA:
            rfcHandleUA(tc, command, poll);
            break;

        case RFCOMM_DM:
            rfcHandleDM(tc,command);
            break;

        case RFCOMM_DISC:
            rfcHandleDISC(tc,command, poll);
            break;

        case RFCOMM_UIH:

 		DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO,"RFC: UIH auf DLCI %X",1, dlci);
            /* check if the link is connected, answer with DM if not */
            if (tc->state != dlciConnected)
            {
               DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "!!RFC: UIH on closed dlci - send DM",1, tc->dlci);
               rfcSendDM(tc->cc, tc->dlci, FALSE /* RESPONSE */, FALSE /* NOT POLL */);
               break;
            }
            if (dlci == 0)
            {
     //           pmsg->MData.DataCBChan.Length--;       /* remove checksum */
     //           pmsg->MData.DataCBChan.Offset += pos;
      //          pmsg->MData.DataCBChan.Length -= pos;
			pL2cDataInd->length--;				/* remove checksum */
			pL2cDataInd->dataOffset += pos;
			pL2cDataInd->length-= pos;			/* remove header */			
	             rfcHandleUIH(tc,pL2cDataInd,command,poll);     	 
                break;
            }
            /* user data on dlci */
            if ((tc->convergenceLayer == RFCOMM_CONVERGENCE_CREDIT_CONF) && poll && returnCredits)
            {
                /* there is a return credit field in the packet */
		  tc->remainingCredits += returnCredits;

                DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "RFC: getting return credits %X (credits now %X)",
                						2,returnCredits, tc->remainingCredits);

                rfcDownstreamWork();
            } /* return credit field exists */

          //  pmsg->MData.DataCBChan.Length--;       /* remove checksum */
          //  pmsg->MData.DataCBChan.Offset += pos;
          //  pmsg->MData.DataCBChan.Length -= pos;
		pL2cDataInd->length--;				/* remove checksum */
		pL2cDataInd->dataOffset += pos;
		pL2cDataInd->length-= pos;			/* remove header */
			
         //   if (pmsg->MData.DataCBChan.Length == 0)
         	if (pL2cDataInd->length == 0)
            {
                break; /* stop any further processing / this message is not visible to higher layer */
            } /* length == 0 */

       //     blueAPI_Handle_RFC_DATA_IND(pmsg, tc->handle);
       //	notify hfp profile
		rfcNotifyUpstream(RFC_DATA_IND,RFCOMM_SUCCESS,tc,pL2cDataInd);
	 //	rfcHandleUpDataResp(tc->handle); //move to app 
	 	break;

        default:
            //RFCOMM_TRACE_PRINTF_1(RFCOMM_TRACE_MASK_ERROR, "!!RFC: UNKNOWN COMMAND %X", ctl);
		DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR,"!!RFC: UNKNOWN COMMAND %X", 1,ctl);
			break;
        } /* switch */
    } /* if tc */

}

void rfcHandleSecRegRsp(PBlueAPI_L2cSecurityRegisterRsp prsp)
{
DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO,"psm %d, server_channel %d ,outgoing %d, active %d,uuid %d,cause %d",6,
										prsp->psm,
										prsp->server_channel,
										prsp->outgoing,
										prsp->active,
										prsp->uuid,
										prsp->cause);

}
void rfcHandleAuthorizationInd(PBlueAPI_UserAuthorizationReqInd pind)
{


}
/**
* @brief  rfcomm  authetication response
* 
* @param  bd
* @param  ref
* @param  channelId
* @param  uuid
* @param  outgoing
* @param  cause
*
* @return  
*
*/
/*void rfcHandleSECMAN_AUTHENTICATION_RESP(  TBdAddr bd,
																  uint16_t    ref,
																  uint16_t    channelId,
																  uint16_t    uuid,
																  uint8_t    outgoing,
																  uint16_t    cause)*/
void rfcAuthenticationRspCallBack(PBlueAPI_RFCAuthenticationRsp prsp)
{
    PTrfcChannel lc;
	

    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO,
					"rfcHandleSECMAN_AUTHENTICATION_RESP bd %s channel %x DLCI %X  outgoing %d cause %x",6,
                 prsp->bd,
                  prsp->channel, //ref,
                   prsp->dlci,//channelID
                   prsp->outgoing,
                   prsp->cause);

    lc = rfcFindDLCI(prsp->dlci,prsp->channel);//((uint8_t)channelId, ref);
    if (lc == (PTrfcChannel)0)    /* not active */
	{
		return;
	}
    lc->authenticationActive = 0;

    if (prsp->outgoing)//(outgoing)
    {
		if( prsp->cause) //(cause)
		{
			/* outgoing connection rejected by security manager */
	//		blueAPI_Handle_RFC_DISC_IND(lc->handle, lc->dlci, cause);

			rfcCast(lc, dlciIdle);
			rfcDisconnectIfNoUserChannels(lc->cc);
			return;
		}
		if (lc->state == dlciOpening)
		{
			rfcSendPNCommand(lc);
		}
    }
    else
    {
		if (prsp->cause)//(cause)
		{
			/* incoming connection rejected by security manager */

			/* do not answer with DM, but answer with PN because some Microsoft implementations have problems
			   with TYPE_PN / DM exchange and seem to prefer TYPE_PN / TYPE_PN followed  by SABM / DM.
			   The channeldescriptor for this DLCI is deallocated, so the subsequent SABM will occur on an
			   unconfigured DLCI, therefore being rejected.
			*/
			uint8_t pn[RFCOMM_SIZE_PN];
			rfcSavePN(lc, pn);                                    /* answer with unchanged configuration */
			rfcSendUIH(lc->cc, RFCOMM_TYPE_PN, 0, pn, sizeof(pn)); /* Response on Control Channel ! */

			rfcFreeChannel(lc);
			return;
		}

		if (lc->state == dlciConfigIndicated)
		{
		//	rfcSendUConInd(lc->dlci, lc->cc->bd, lc->handle, lc->frameSize,
		//					lc->convergenceLayer == RFCOMM_CONVERGENCE_CREDIT_CONF, FALSE);
			rfcNotifyUpstream(RFC_CONNECT_IND, RFCOMM_SUCCESS, lc,NULL);
		}
    }
}
/**
* @brief  rfcomm send authetication indicator
* 
* @param  pRFC
* @param  tc
* @param  outgoing
* @param  active
*
* @return  
*
*/
void rfcSendSecManAuthenticationInd(PTrfcChannel tc, uint8_t outgoing, uint8_t active)
{
//	btsmSendMsgAuthenticationInd(tc->cc->bd, tc->cc->channel, tc->dlci, tc->uuid ,outgoing, active, rfcQueueID, BLUEFACE_CON_TYPE_BR_EDR);
  	mpa_SendRFCAuthenticationReq(tc->cc->bd, tc->cc->channel,tc->dlci,tc->uuid,outgoing,active);
	tc->authenticationActive = 1;
   	//(uint8_t * bd, uint16_t channel, uint16_t dlci, uint16_t uuid, uint8_t outgoing, uint8_t active)
}


