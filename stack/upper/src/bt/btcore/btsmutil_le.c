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
#include <os_timer.h>
#include <upper_stack_global.h>
#include <blueapi_api.h>
#include <os_pool.h>
#include <Os_sched.h>

#define TRACE_MODULE_ID     MID_BTSECMAN

#if F_BT_LE_BT41_SUPPORT
LPTSEC_CONF_LE_SECURITY btsmAllocateLESecEntry(LPTSEC_CONF_LE_SECURITY newEntry)
{
    int i;
    for (i = 0; i < BT_LE_SECMAN_POLICY_COUNT; i++)
    {
        LPTSEC_CONF_LE_SECURITY entry = &pBtSM->secLeEntries[i];
        if (!entry->active)
        {
            memcpy(entry, newEntry, sizeof(TSEC_CONF_LE_SECURITY));
            BTSECMAN_TRACE_PRINTF_3(BTSECMAN_TRACE_MASK_TRACE,
                                    "btsmAllocateLESecEntry: id:%d psm:0x%04x mode:0x%x",
                                    i, entry->psm, entry->secMode
                                   );
            return entry;
        } /* if */
    } /* for */

    BTSECMAN_TRACE_PRINTF_0(BTSECMAN_TRACE_MASK_ERROR,
                            "!!btsmAllocateLESecEntry: could not allocate entry"
                           );
    return NULL;
} /* btsmAllocateSecEntry */

/* deallocate a secman link descriptor */
uint16_t btsmDeAllocateLESecEntry(LPTSEC_CONF_LE_SECURITY delEntry)
{
    /* find corresponding entry in security database and delete it */
    LPTSEC_CONF_LE_SECURITY entry = btsmFindLESecEntry(delEntry->psm);
    if (entry)
    {
        entry->active = FALSE;
        return 0; /* success */
    }
    /* item not found, return error to the caller */
    return (SECMAN_ERR | SECMAN_ERR_NOENTRY);
} /* btsmDeAllocateSecEntry */

LPTSEC_CONF_LE_SECURITY btsmFindLESecEntry(uint16_t psm)
{
    /* find security requirements according to values */
    int i;

    for (i = 0; i < BT_LE_SECMAN_POLICY_COUNT; i++)
    {
        LPTSEC_CONF_LE_SECURITY entry = &pBtSM->secLeEntries[i];
        if (entry->active)
        {
            if (entry->psm == psm)
                return entry;
        }
    } /* for */

    return NULL; /* nothing relevant found */
} /* btsmFindSecEntry */


BOOL btsmCheckNewLESecEntry(LPTSEC_CONF_LE_SECURITY sec)
{
    BOOL Valid = TRUE;

    if (Valid &&
            sec->active &&
            sec->psm == 0 )
    {
        Valid = FALSE;
    }

    /* check for double entries */
    if (Valid &&
            sec->active &&
            btsmFindLESecEntry(sec->psm))
    {
        Valid = FALSE;
    }
    return Valid;
} /* btsmCheckNewSecEntry */
#endif

/**
* @brief  find link by handle
*
* @param  handle: 
*
* @return  
*
*/
LPTSECLINK btsmFindLELinkByHandle(uint16_t handle)
{	
    uint16_t i;
    LPTSECLINK pLink = NULL;

    for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_don; i++)
    {
        pLink = &pBtSM->plinksDon[i];


        if ((pLink->used != SM_LINK_FREE) &&
                (pLink->bdType & BLUEFACE_BDTYPE_LE_MASK) &&
                (pLink->handle == handle)
           )
        {
            return pLink;
        }
    }

    for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_doff; i++)
    {
        pLink = &pBtSM->plinksDoff[i];
        if ((pLink->used != SM_LINK_FREE) &&
                (pLink->bdType & BLUEFACE_BDTYPE_LE_MASK) &&
                (pLink->handle == handle)
           )
        {
            return pLink;
        }
    }	

	BTSECMAN_TRACE_PRINTF_1(BTSECMAN_TRACE_MASK_ERROR,
	                         "btsmFindLELinkByHandle: could not find link for handle:0x%x",
	                         handle
	                         );
	return NULL;
}
/**
* @brief	 btsm get callback buffer
*
* @param	pMessage: 
* @param     command
* @param	BufferPtr
* @param	length
* @param	offset
*
* @return
*
*/
int btsmDataCBBufferGet(MESSAGE_P pMessage, uint16_t command,
                               void  *  *BufferPtr, uint16_t length, uint16_t offset)
{
    uint8_t * pBuffer;
    int    result = osBufferGet(BTSystemPoolID, length + offset, (PVOID)&pBuffer);

    if (!result)
    {
        pMessage->Command         = command;
        pMessage->MData.DataCB.BufferAddress  = (uint8_t *)pBuffer;
        pMessage->MData.DataCB.Length         = length;
        pMessage->MData.DataCB.Offset         = offset;
        pMessage->MData.DataCB.Flag           = DATA_CB_RELEASE;

        /* return pointer to data */
        *BufferPtr = pBuffer + offset;
    }
    else
    {
        BTSECMAN_TRACE_PRINTF_1(BTSECMAN_TRACE_MASK_ERROR,
                                   "!!btsmDataCBBufferGet: No Memory for command 0x%x",
                                   command
                                   );
    }

    return result;
} /* btsmDataCBBufferGet */

/**
* @brief  bt sm le get msg buf
*
* @param  pMessage
* @param  pLEMsg
* @param  length
* @param  msgType
* @param  handle
*
* @return  
*
*/
int btsmLEMsgBufferGet2(MESSAGE_P pMessage, LPLEMessage * pLEMsg, uint16_t length, uint16_t msgType, uint16_t handle)
{
	uint8_t * pBuffer;
	int    result;

	length += offsetof(TLEMessage, p);
	result = osBufferGet(BTSystemPoolID, length + BT_LE_MESSAGE_OFFSET, (PVOID  *)&pBuffer);

	if (!result)
	{
		pMessage->MData.DataCB.BufferAddress = pBuffer;
		pMessage->MData.DataCB.Offset        = BT_LE_MESSAGE_OFFSET;
		*pLEMsg = (LPLEMessage)(pBuffer + BT_LE_MESSAGE_OFFSET);
		(*pLEMsg)->msgType = msgType;
		(*pLEMsg)->handle  = handle;
	}
	else
	{
		BTSECMAN_TRACE_PRINTF_1(BTSECMAN_TRACE_MASK_ERROR,
		                           "!!btsmLEMsgBufferGet: No Memory for msgType:0x%x",
		                           msgType
		                           );
	}

	return result;
} /* btsmLEMsgBufferGet */

/**
* @brief  btsm allocate smp data
*
* @param  pLink
*
* @return  
*
*/
LPLE_SMP_DATA btsmAllocateSMPData(LPTSECLINK pLink)
{
	if ((pLink->used == SM_LINK_FREE) ||
	  !(pLink->bdType & BLUEFACE_BDTYPE_LE_MASK) ||
	  (pLink->pSMPData != NULL)
	 )
	{
		return NULL;
	}

	if (osBufferGet(BTSystemPoolID, sizeof(TLE_SMP_DATA), (PVOID  *)&pLink->pSMPData))
	{
		return NULL;
	}

	memset(pLink->pSMPData, 0x00, sizeof(TLE_SMP_DATA));
	return pLink->pSMPData;
}

/**
* @brief  btsm deallocate SMP data
*
* @param  pLink
*
* @return  
*
*/
void btsmDeAllocateSMPData(LPTSECLINK pLink)
{
	if (pLink->pSMPData != NULL)
	{
		osBufferRelease((PVOID)pLink->pSMPData);
		pLink->pSMPData = NULL;
	}
}

/**
* @brief  btsm start smp timer
*
* @param  pLink
*
* @return  
*
*/
void btsmStartSMPTimer(LPTSECLINK pLink)
{
	LPLE_SMP_DATA pSMPData = pLink->pSMPData;
	if(pSMPData->smpState & LE_SMPSTATE_TIMEOUT_RUNNING)
	{
        BTSECMAN_TRACE_PRINTF_0(BTSECMAN_TRACE_MASK_TRACE,
				                        "SM: restart timer"
				                        );
        osRestartTimer(&pLink->timerHandle, btsmQueueID, SM_TIMER_SMP_TIMEOUT, pLink->index,SMP_TIMEOUT, swTimerCallBack);
	}
	else
	{
        BTSECMAN_TRACE_PRINTF_0(BTSECMAN_TRACE_MASK_TRACE,
				                        "SM: set timer"
				                        );
        osStartTimer(&pLink->timerHandle, btsmQueueID, SM_TIMER_SMP_TIMEOUT, pLink->index,SMP_TIMEOUT, swTimerCallBack);
        pSMPData->smpState |= LE_SMPSTATE_TIMEOUT_RUNNING;
	}
}

/**
* @brief  btsm cancel SMP timer
*
* @param  pLink
*
* @return  
*
*/
void btsmCancelSMPTimer(LPTSECLINK pLink)
{
    BTSECMAN_TRACE_PRINTF_0(BTSECMAN_TRACE_MASK_TRACE,
				                        "SM: reset timer"
				                        );
    osDeleteTimer(&pLink->timerHandle);

    if (pLink->pSMPData)
    {
        pLink->pSMPData->smpState &= ~(LE_SMPSTATE_TIMEOUT_RUNNING);
    }

}

/**
* @brief  get smp auth requirement
*
* @param  pLink
*
* @return  
*
*/
uint8_t btsmGetSMPAuthRequirement(LPTSECLINK pLink)
{
	uint8_t result = 0x00;

	if ((pBtSM->devSecurity.authenSettings & DEVAUTH_SETTING_MITM) ||
	  (pLink->mode & LINKMODE_MITM_REQUIRED)
	 )
	{
		result |= LE_SMP_AUTHREQ_MITM;
	}

	if ((pBtSM->devSecurity.authenSettings & DEVAUTH_SETTING_BOND_MASK) != DEVAUTH_SETTING_NO_BOND)
	{
		result |= LE_SMP_AUTHREQ_BONDING;
	}

	return result;
}

/**
* @brief  btsm get smp key distribution
*
* @param  pLink
*
* @return  
*
*/
uint16_t btsmGetSMPKeyDistribution(LPTSECLINK pLink)
{
	uint16_t result = 0;

	UNUSED_PARAMETER(pLink);

	if (pBtSM->devSecurity.authenSettings & DEVAUTH_SETTING_BOND_MASK)
	{
		/* request slave key, to allow reconnect with same role */
		result |= LE_SMP_KEYDIST_ENCKEY;
		/* request IRK from peer with private resolvable address */
		if ((pLink->unresolved_bdType == BLUEFACE_BDTYPE_LE_RANDOM) &&
		    ((pLink->unresolved_bd[5] & BLUEFACE_RANDBD_MASK) == BLUEFACE_RANDBD_RESOLVABLE)
		   )
		{
			if (pLink->role == LE_ROLE_MASTER)
			{
				result |= LE_SMP_KEYDIST_IDKEY;
			}
			else
			{
				result |= (LE_SMP_KEYDIST_IDKEY << 8);
			}
		}
	}

    if (otp_str_data.gEfuse_UpperStack_s.bqb_en)
    {
        if(pBtSM->devSecurity.authenSettings & DEVAUTH_SETTING_BOND_MASK)
        {
            result = LE_SMP_KEYDIST_ENCKEY | (LE_SMP_KEYDIST_ENCKEY << 8);
        }
        else
        {
            result = 0;
        }
        
    }
    
	return result;
}

/**
* @brief  security manager resolve random addr
* 
* @param  bd:  
* @param  pBdType
*
* @return 
*
*/
uint8_t btsmResolveRandomAddress(TBdAddr bd, uint8_t * pBdType)
{
	uint16_t          i;
	PRESOLVEENTRY pFreeEntry    = NULL;
	PRESOLVEENTRY pOldNegEntry  = NULL;

	/* check for resolvable address */
	if ((*pBdType != BLUEFACE_BDTYPE_LE_RANDOM) ||
	  ((bd[5] & BLUEFACE_RANDBD_MASK) != BLUEFACE_RANDBD_RESOLVABLE)
	 )
	{
		return LE_RESOLVE_ENTRY_ERROR_INVALID_TYPE;
	}

	for (i = 0; i < LE_RESOLVE_CACHE_SIZE; i++)
	{
		PRESOLVEENTRY pSearch = &pBtSM->LE_resolveCache[i];

		switch (pSearch->state)
		{
		/* remember first free entry for later allocation */
		case LE_RESOLVE_ENTRY_STATE_FREE: 
			if (pFreeEntry == NULL)
			{
				pFreeEntry = pSearch;
			}
			break;

		/* remember oldest negative entry for later allocation */
		case LE_RESOLVE_ENTRY_STATE_NEGATIVE: 
			if ((pOldNegEntry == NULL) || (pSearch->timestamp < pOldNegEntry->timestamp))
			{
				pOldNegEntry = pSearch;
			}
			break;

		default:
			break;
		}

		/* check if bd is already resolved / currently resolving */
		if ((pSearch->state != LE_RESOLVE_ENTRY_STATE_FREE) &&
		    (memcmp(pSearch->random_bd, bd, BD_ADDR_SIZE) == 0)
		   )
		{
			if (pSearch->state == LE_RESOLVE_ENTRY_STATE_POSITIVE)
			{
				memcpy(bd, pSearch->resolved_bd, BD_ADDR_SIZE);
				*pBdType = pSearch->resolved_bdType;
			}

			/* update timestamp on resolved entries (record last use) */
			if (pSearch->state & LE_RESOLVE_ENTRY_STATE_RESOLVED_MASK)
			{
				pSearch->timestamp = osGetSystemTime();
			}

			return pSearch->state;
		}
	}

	/* use oldest negative entry if no free one is available */
	if (pFreeEntry == NULL)
	{
		pFreeEntry = pOldNegEntry;
		if (pFreeEntry == NULL)
		{
			return LE_RESOLVE_ENTRY_ERROR_NO_RESSOURCE;
		}
	}

	BTSECMAN_TRACE_PRINTF_1(BTSECMAN_TRACE_MASK_ERROR,
	                         "btsm: start IRK resolving for bd:%s",
	                         TRACE_BDADDR1(BTSECMAN_TRACE_MASK_ERROR, bd)
	                         );

	memset(pFreeEntry, 0x00, sizeof(TRESOLVEENTRY));
	memcpy(pFreeEntry->random_bd, bd, BD_ADDR_SIZE);
	pFreeEntry->state = LE_RESOLVE_ENTRY_STATE_STORE_GET;

	/* request all remote IRKs from store */
	btsmSendDEVICE_DATA_GET_IND_EXT(pFreeEntry, NULL,
	                              BLUEFACE_BDTYPE_ANY,
	                              DEVICE_DATA_TYPE_SECMAN_IRK_REMOTE,
	                              0x0000);

	/* queue up message */
	return LE_RESOLVE_ENTRY_STATE_QUEUE;
} /* btsmResolveRandomAddress */

/**
* @brief  btsecurity map random address
*
* @param  bd
* @param  pBdType
*
* @return  
*
*/
uint8_t btsmMapRandomAddress(TBdAddr bd, uint8_t * pBdType)
{
	uint16_t i;

	/* check for resolved address */
	if (*pBdType != (BLUEFACE_BDTYPE_LE_MASK | BLUEFACE_BDTYPE_LE_RESOLVED_MASK))
	{
		return LE_RESOLVE_ENTRY_ERROR_INVALID_TYPE;
	}

	for (i = 0; i < LE_RESOLVE_CACHE_SIZE; i++)
	{
		PRESOLVEENTRY pSearch = &pBtSM->LE_resolveCache[i];

		if ((pSearch->state == LE_RESOLVE_ENTRY_STATE_POSITIVE) &&
		    (pSearch->resolved_bdType == *pBdType) &&
		    (memcmp(pSearch->resolved_bd, bd, BD_ADDR_SIZE) == 0)
		   )
		{
			memcpy(bd, pSearch->random_bd, BD_ADDR_SIZE);
			*pBdType = BLUEFACE_BDTYPE_LE_RANDOM;
			return LE_RESOLVE_ENTRY_STATE_POSITIVE;
		}
	}

	return LE_RESOLVE_ENTRY_STATE_NEGATIVE;
} /* btsmResolveRandomAddress */

