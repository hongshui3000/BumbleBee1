/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       btsmutil.c
* @brief     bt security manager util func
* @details   
*
* @author   	gordon
* @date      	2015-07-09
* @version	v0.1
*/

#include <flags.h>
#include <rtl_types.h>
#include <message.h>
#include <btsm.h>
#include <btsmprot.h>
#include <btcommon.h>
#include <swtimer.h>
#include <os_message.h>
#include <os_timer.h>
#include <upper_stack_global.h>
#include <blueapi_api.h>

#define TRACE_MODULE_ID     MID_BTSECMAN


/**
* @brief  queue in msg to deferred queue
* 
* @param  pLink
*
* @return  
*
*/
void btsmDeferredQueueIn(LPTSECLINK pLink)
{
	BTSECMAN_TRACE_PRINTF_2(BTSECMAN_TRACE_MASK_ERROR,
	                         "btsm: queue command 0x%x for link %s",
	                         pBtSM->message.Command,
	                         TRACE_BDADDR1(BTSECMAN_TRACE_MASK_ERROR, pLink ? pLink->bd : NULL)
	                         );

	osMessageSend(pBtSM->deferredQueueID, &pBtSM->message);
} /* btsmDeferredQueueIn */

/**
* @brief  queue out from deferredQueue 
*
*
* @return  
*
*/
void btsmDeferredQueueOut()
{
	MESSAGE_T message;

	while (osMessageReceive(pBtSM->deferredQueueID, &message) == 0)
	{
		BTSECMAN_TRACE_PRINTF_1(BTSECMAN_TRACE_MASK_ERROR,
		                           "btsm: retry command 0x%x",
		                           message.Command
		                           );

		osMessageSend(btsmQueueID, &message);
	}
} /* btsmDeferredQueueIn */


/**
* @brief  starts a timer for a secman link descriptor
*
* @param  pLink: 
* @param  timerID:
* @param  seconds
*
* @return  
*
*/
void btsmStartTimer(LPTSECLINK pLink, uint8_t timerID, uint16_t seconds)
{
    if(pLink->timerHandle != 0)
    {
        BTSECMAN_TRACE_PRINTF_0(BTSECMAN_TRACE_MASK_ERROR,
				                        "!!SM: btsmStartTimer handle != 0"
				                        );
        pLink->timerHandle = 0;
    }
        
    osStartTimer(&pLink->timerHandle, btsmQueueID, timerID, pLink->index,seconds*1000, swTimerCallBack);
}

/**
* @brief  return allocated secman link descriptor or allocate new descriptor
*
* @param  bdType: 
* @param  bd:
*
* @return  
*
*/
LPTSECLINK btsmFindAllocateLink(uint8_t bdType, TBdAddr bd)
{
    uint16_t       i;
    LPTSECLINK pFreeLink = NULL;
    uint8_t index = 0;
    LPTSECLINK pLink = NULL;

    for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_don; i++)
    {
        pLink = &pBtSM->plinksDon[i];
        if ((pLink->used != SM_LINK_FREE) &&
                (pLink->bdType == bdType) &&
                (memcmp(pLink->bd, bd, BD_ADDR_SIZE) == 0)
           )
        {
            return pLink;
        }
    }

    for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_doff; i++)
    {
        pLink = &pBtSM->plinksDoff[i];
        if ((pLink->used != SM_LINK_FREE) &&
                (pLink->bdType == bdType) &&
                (memcmp(pLink->bd, bd, BD_ADDR_SIZE) == 0)
           )
        {
            return pLink;
        }
    }

    for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_don; i++)
    {
        pLink = &pBtSM->plinksDon[i];
        if ((pLink->used == SM_LINK_FREE) &&
                 (pFreeLink == NULL)
                )
        {
            index = i + 0x10;
            pFreeLink = pLink;
            goto TagFound;
        }
    }

    for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_doff; i++)
    {
        pLink = &pBtSM->plinksDoff[i];
        if ((pLink->used == SM_LINK_FREE) &&
                 (pFreeLink == NULL)
                )
        {
            index = i;
            pFreeLink = pLink;
            goto TagFound;
        }
    }
    pFreeLink = NULL;

TagFound:
    if (pFreeLink != NULL)
    {
        memset(pFreeLink, 0x00, sizeof(TSECLINK));
        pFreeLink->used   = SM_LINK_USED;
        pFreeLink->index = index;
        pFreeLink->bdType = bdType;
        memcpy(pFreeLink->bd, bd, BD_ADDR_SIZE);
        return pFreeLink;
    }	

	BTSECMAN_TRACE_PRINTF_1(BTSECMAN_TRACE_MASK_ERROR,
	                         "!!btsmFindAllocateLink: could not allocate link for bd:%s",
	                         TRACE_BDADDR1(BTSECMAN_TRACE_MASK_ERROR, bd)
	                         );
	return NULL;
} /* btsmFindAllocateLink */

/**
* @brief  free a secman link descriptor
*
* @param  pLink: 
*
* @return  
*
*/
void btsmDeAllocateLink(LPTSECLINK pLink)
{
	if (pLink)
	{
		pLink->used = SM_LINK_FREE;
	}
	else
	{
        uint16_t i;
        for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_don; i++)
        {
            pBtSM->plinksDon[i].used = SM_LINK_FREE;
        }

        for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_doff; i++)
        {
            pBtSM->plinksDoff[i].used = SM_LINK_FREE;
        }
			
	}
} /* btsmDeAllocateLink */

/**
* @brief  get secman link descriptor by bdaddr
*
* @param  bd: 
*
* @return  
*
*/
LPTSECLINK btsmFindLink(TBdAddr bd)
{	
	uint16_t i;
    LPTSECLINK pLink = NULL;
    for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_don; i++)
    {
        pLink = &pBtSM->plinksDon[i];
        if ((pLink->used != SM_LINK_FREE) && memcmp(pLink->bd, bd, BD_ADDR_SIZE) == 0)
        {
            return pLink;
        }
    }

    for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_doff; i++)
    {
        pLink = &pBtSM->plinksDoff[i];
        if ((pLink->used != SM_LINK_FREE) && memcmp(pLink->bd, bd, BD_ADDR_SIZE) == 0)
        {
            return pLink;
        }
    }
	BTSECMAN_TRACE_PRINTF_1(BTSECMAN_TRACE_MASK_ERROR,
	                 "btsmFindLink: could not find link for bd:%s",
	                 TRACE_BDADDR1(BTSECMAN_TRACE_MASK_ERROR, bd)
	                 );
	return NULL;
} /* btsmFindLink */



/**
* @brief  hex to string 
*
* @param  value
* @param  output
* @param  length
*
* @return  
*
*/
void btsmHexString_r(uint8_t * value, LPSTR output, uint16_t length)
{
	uint16_t pos = 0;
	uint8_t nib;
	uint16_t i;

	/* multiple-octet values written in hexadecimal notation have
	* the most significant octet towards the left and
	* the least significant octet towards the right
	*/
	for (i = 0; i < length; i++)
	{
		nib           = (value[(length -1) - i] >> 4) & 0x0F;
		output[pos++] = (nib < 0x0A) ? ('0' + nib) : ('A' - 0x0A + nib);

		nib           =  value[(length -1) - i] & 0x0F;
		output[pos++] = (nib < 0x0A) ? ('0' + nib) : ('A' - 0x0A + nib);
	}

	output[pos] = '\0';
}

/**
* @brief  hex to string 
*
* @param  value
* @param  length
*
* @return  
*
*/
LPSTR btsmHexString(uint8_t * value, uint16_t length)
{
	if (length > 16)
	{
		length = 16;
	}

    btsmHexString_r(value, gHexBuffer, length);

    return gHexBuffer;

}



