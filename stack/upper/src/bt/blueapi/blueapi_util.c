/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       blueapi_util.c
* @brief     profile application utility functions
* @details   
*
* @author   	gordon
* @date      	2015-07-08
* @version	v0.1
*/

#include <flags.h>
#include <rtl_types.h>
#include <blueface.h>
#include <os_timer.h>
#include <swtimer.h>
#include <sdp_code.h>
#include <sdplib.h>
#include <btglbdef.h>
#include <bt_api.h>
#include <os_message.h>
#include <os_pool.h>
#include <sdp_code.h>
#include <blueapi_types.h>
#include <bluemgr.h>
#include <blueapi.h>
#include <btsend.h>
#include <btman.h>
#include <btcommon.h>
#include <hci_api.h>
#include <sdp_api.h>
#include <gatt_api.h>
#include <blueapi_def.h>
#include <blueapi_osif.h>
#include <upper_stack_global.h>

#define TRACE_MODULE_ID     MID_BT_L2C

/* File Id for COM_InternalEventInfo */
#define BLUE_API_SOURCE_FILE_ID     0x05

#if BLUEAPI_TRACE_VERBOSITY_COUNT

const char pBlueAPIDumpPrefix[] TRACE_DATA = "====>:";

/**
* @brief  blueapi dump link info
*
* @param  pLinkContext
* @param  pPrefix: no link, print pPrefix
*
* @return
*
*/
void blueAPI_DumpLink(PBlueAPI_LinkDescriptor pLinkContext, const char* pPrefix)
{
	if((pLinkContext)&&
	((pLinkContext->linkState      != blueAPI_LinkStateReserved)||
	(pLinkContext->linkSubRole    != blueAPI_SubRoleUndefined)||
	(pLinkContext->linkConfigType != blueAPI_LinkConfigHDPDontCare)||
	(pLinkContext->pMDL)||
	(pLinkContext->pMCL)
	)
	)
	{
		BLUEAPI_TRACE_PRINTF_7(BLUEAPI_TRACE_MASK_TRACE,
		                        "%s link[0x%8.8X] state[%d] role[%d] conf[%d] MDL[0x%8.8X] MCL[0x%8.8X]",
		                        pPrefix,
		                        (uint32_t)pLinkContext,
		                        pLinkContext->linkState,
		                        pLinkContext->linkSubRole,
		                        pLinkContext->linkConfigType,
		                        (uint32_t)pLinkContext->pMDL,
		                        (uint32_t)pLinkContext->pMCL
		                        );

	} else if(pLinkContext==NULL)
	{
		BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE,
		                        "%s link[NULL]",
		                        pPrefix
		                        );
	}
}

/**
* @brief  blueapi dump mcl link info
*
* @param  pLinkContext
* @param  pPrefix: no mcl link, print pPrefix
*
* @return
*
*/
void blueAPI_DumpMCL(PBlueAPI_MCL pMCL,const char* pPrefix)
{
	if((pMCL)&&
	((pMCL->state!=blueAPI_DS_Reserved))
	)
	{
		BLUEAPI_TRACE_PRINTF_4(BLUEAPI_TRACE_MASK_TRACE,
		                        "%s MCL[0x%8.8X] bd[%s] state[%d] count[%d]",
		                        pPrefix,
		                        (uint32_t)pMCL,
		                        BLUEAPI_TRACE_BDADDR1(pMCL->bd),
		                        pMCL->state
		                        );

		if((pMCL->DS_CommandInProgress != blueAPI_EventIdle)||
		 (pMCL->US_CommandInProgress != blueAPI_EventIdle)||
		)
		{
			BLUEAPI_TRACE_PRINTF_3(BLUEAPI_TRACE_MASK_TRACE,
			                       "%s --------------- penDS_COM[%d] penUS_REQ[%d] penCause[%d]",
			                       pPrefix,
			                       pMCL->DS_CommandInProgress,
			                       pMCL->US_CommandInProgress
			                       );
		}
	} else if(pMCL == NULL)
	{
		BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE,
		                        "%s dev[NULL]",
		                        pPrefix
		                        );
	}
}

/**
* @brief  blueapi dump mdl link info
*
* @param  pLinkContext
* @param  pPrefix: no mdl link, print pPrefix
*
* @return
*
*/
void blueAPI_DumpMDL(PBlueAPI_MDL pMDL, const char* pPrefix)
{
	if((pMDL)&&
	(/*(blueAPI_BDValidCheck(pMDL->remoteBd))||*/
	(pMDL->local_MDL_ID)               ||
	(pMDL->local_MDEP_ID)              ||
	(pMDL->remote_MDEP_ID)             ||
	(pMDL->pendingFlags)
	)
	)
	{
		BLUEAPI_TRACE_PRINTF_8(BLUEAPI_TRACE_MASK_TRACE,
		                        "%s MDL[0x%8.8X] bd[%s] locMDL[%d] locMDEP[%d] remMDEP[%d] flags[%x]",
		                        pPrefix,
		                        (uint32_t)pMDL,
		                        BLUEAPI_TRACE_BDADDR1(pMDL->remoteBd),
		                        pMDL->local_MDL_ID ,
		                        pMDL->local_MDEP_ID ,
		                        pMDL->remote_MDEP_ID,
		                        pMDL->pendingFlags
		                        );

	} else if(pMDL == NULL)
	{
		BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE,
		                        "%s MDL[NULL]",
		                        pPrefix
		                        );
	}
}

/**
* @brief  blueapi dump system info
*
* @param  pBlueAPIdata
*
* @return
*
*/
void blueAPI_DumpSystem(PBlueAPI_Data pBlueAPIdata)
{
	uint16_t                    loop;
	PBlueAPI_MCL            pMCL;
	PBlueAPI_LinkDescriptor pLinkContext;
	PBlueAPI_MDL            pMDL;
	static const char       pEmpty[]   TRACE_DATA = "";
	static const char       pTextMDL[] TRACE_DATA = "MDL:";

#if 0

	for(loop = 0; loop < BLUE_API_MCL_COUNT; loop++)
	{  
		pMCL=&pBlueAPIdata->MCLDescriptorTable[loop];

		TRACE_DUMP_MCL(pMCL, pEmpty);
	}

	for(loop = 0;loop < BLUE_API_LINKTABLE_COUNT; loop++)
	{ 
		pLinkContext=&pBlueAPIdata->linkDescriptorTable[loop];

		TRACE_DUMP_LINK(pLinkContext, pEmpty);
	}

	for(loop = 0;loop < BLUE_API_MDL_COUNT; loop++)
	{ 
		pMDL=&pBlueAPIdata->MDLDescriptorTable[loop];

		TRACE_DUMP_MDL(pMDL,pTextMDL);
	}
#else
    for(loop = 0; loop < otp_str_data.gEfuse_UpperStack_s.num_link_don; loop++)
	{  
		pMCL=&pBlueAPIdata->pMCLDescriptorTableDon[loop];

		TRACE_DUMP_MCL(pMCL, pEmpty);
	}
    for(loop = 0; loop < otp_str_data.gEfuse_UpperStack_s.num_link_doff; loop++)
	{  
		pMCL=&pBlueAPIdata->pMCLDescriptorTableDoff[loop];

		TRACE_DUMP_MCL(pMCL, pEmpty);
	}

    for(loop = 0;loop < otp_str_data.gEfuse_UpperStack_s.num_link_don; loop++)
	{ 
		pLinkContext=&pBlueAPIdata->plinkDescriptorTableDon[loop];

		TRACE_DUMP_LINK(pLinkContext, pEmpty);
	}
    for(loop = 0;loop < otp_str_data.gEfuse_UpperStack_s.num_link_doff; loop++)
	{ 
		pLinkContext=&pBlueAPIdata->plinkDescriptorTableDoff[loop];

		TRACE_DUMP_LINK(pLinkContext, pEmpty);
	}

    for(loop = 0;loop < otp_str_data.gEfuse_UpperStack_s.num_link_don; loop++)
	{ 
		pMDL=&pBlueAPIdata->pMDLDescriptorTableDon[loop];

		TRACE_DUMP_MDL(pMDL,pTextMDL);
	}
    for(loop = 0;loop < otp_str_data.gEfuse_UpperStack_s.num_link_doff; loop++)
	{ 
		pMDL=&pBlueAPIdata->pMDLDescriptorTableDoff[loop];

		TRACE_DUMP_MDL(pMDL,pTextMDL);
  	}  

#endif
}
#endif  /* BLUEAPI_TRACE_VERBOSITY_COUNT */

/**
 * @brief  check if link config type valid
 *
 * @param  psm:
 * @param  linkConfigType: 
 *
 * @return 
 *
 */
BOOL blueAPI_CheckLinkConfigType(uint16_t psm, TBlueAPI_LinkConfigType linkConfigType)
{
    BOOL validLinkConfig = TRUE;

    switch (linkConfigType)
    {
    case blueAPI_LinkConfigGATT: 
        if (psm != BLUEFACE_PSM_GATT)
        {
            validLinkConfig = FALSE;
        }
        break;

    case blueAPI_LinkConfigSDP: 
        if (psm != BLUEFACE_PSM_SDP)
        {
            validLinkConfig = FALSE;
        }
        break;

    default: 
        validLinkConfig = FALSE;
        break;
    }

    return validLinkConfig;
}


/**
 * @brief  start blueapi timer
 *
 * @param  pMCL
 * @param  timerID
 * @param  timeout_ms
 *
 * @return 
 *
 */
void blueAPI_TimerStart(PBlueAPI_MCL pMCL, TBlueAPI_MCLTimerID timerID, uint32_t timeout_ms)
{
    void **pHandle = 0;

    BLUEAPI_TRACE_PRINTF_3(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI: MCL[0x%8.8X] start timer ID[%d] timeout[%d]",
        (uint32_t)pMCL, timerID, timeout_ms);

    if (timeout_ms > 0xFFFF)
    {
        timeout_ms = 0xFFFF;
    }

    switch (timerID)
    {
    case blueAPI_TimerID_LEScanTO:
        pHandle = &(pMCL->TimeHandleLEScanTO);
        break;

     default:
        return;
    }

    osStartTimer(pHandle, blueAPIQueueID, (uint8_t)timerID,pMCL->number, timeout_ms, swTimerCallBack);
}

/**
 * @brief  restart timer
 *
 * @param  pMCL
 * @param  timerID: timer ID
 * @param  timeout_ms: timeout(ms)
 *
 * @return
 *
 */
void blueAPI_TimerRetrigger(PBlueAPI_MCL pMCL, TBlueAPI_MCLTimerID timerID, uint32_t timeout_ms)
{
    void **pHandle = 0;

    BLUEAPI_TRACE_PRINTF_3(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI: MCL[0x%8.8X] retrigger timer ID[%d] timeout[%d]",
        (uint32_t)pMCL, timerID, timeout_ms);

    if (timeout_ms > 0xFFFF)
    {
        timeout_ms = 0xFFFF;
    }

    switch (timerID)
    {
    case blueAPI_TimerID_LEScanTO:
        pHandle = &(pMCL->TimeHandleLEScanTO);
        break;

    default:
        return;
    }

    osRestartTimer(pHandle, blueAPIQueueID, (uint8_t)timerID, pMCL->number, timeout_ms, swTimerCallBack);
}

/**
 * @brief  stop timer
 *
 * @param  pMCL
 * @param  startTimerMask
 *
 * @return
 *
 */
void blueAPI_TimerStop(PBlueAPI_MCL pMCL, TBlueAPI_MCLTimerID timerID)
{
    void **pHandle = 0;

    BLUEAPI_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_TRACE, "blueAPI: MCL[0x%8.8X] stop timer ID[%d]",
        (uint32_t)pMCL, timerID);

    switch (timerID)
    {
    case blueAPI_TimerID_LEScanTO:
        pHandle = &(pMCL->TimeHandleLEScanTO);
        break;

     default:
        return;
    }

    osDeleteTimer(pHandle);
}

/**
* @brief  btmos timeout handle func
* 
* @param  pBlueAPIdata:  
* @param  timerID: timeout id
* @param  timerChannel: timeout channel
* @return  
*
*/
void blueAPI_TimerTimeout(PBlueAPI_Data pBlueAPIdata, TBlueAPI_MCLTimerID timerID, uint16_t timerChannel)
{
    PBlueAPI_MCL pMCL = blueAPI_MCLFindByTimerChannel(pBlueAPIdata, timerChannel); /*use timechannel to get mcl*/

    BLUEAPI_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_TRACE,
                     "blueAPI: MCL[0x%8.8X] timer timeout ID[%d]",
                     (uint32_t)pMCL, timerID
                     );

    if (pMCL)
    {
        switch (timerID)
        {
        case blueAPI_TimerID_LEScanTO:
            BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE,
                                      "====>: LE scan timeout => disconnect"
                                      );

            blueAPI_ChannelDisconnect(pBlueAPIdata, pMCL->pDataChannel, TRUE);
            break;

        default:
            break;
        }
    }
}

#if SEND_MCL_STATUS_INFO
TBlueAPI_MCLStatus blueAPI_MCLState2Status(TBlueAPI_MCLState state)
{
    switch (state)
    {
    case blueAPI_DS_Reserved:
        return blueAPI_MCLReleased;

    case blueAPI_DS_Idle:
        return blueAPI_MCLIdle;

    case blueAPI_DS_ControlConnected:
        return blueAPI_MCLControlConnected;

    case blueAPI_DS_DataConnecting:
        return blueAPI_MCLDataConnecting;

    case blueAPI_DS_DataConnected:
        return blueAPI_MCLDataConnected;

    case blueAPI_DS_DataDisconnecting:
        return blueAPI_MCLDataDisconnecting;

    case blueAPI_DS_DataListening:
        return blueAPI_MCLDataListening;

    default:
        return blueAPI_MCLReleased;  /* TODO: correct? */
    }
}
#endif

/**
* @brief  set MCL State
* 
* @param  pMCL:  point to MCL
* @param  newState: new State
* @return  
*
*/
void blueAPI_MCLSetState(PBlueAPI_MCL pMCL,TBlueAPI_MCLState newState)
{
	BLUEAPI_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_TRACE,
	                         "blueAPI: ------- MCL change state from[%d] to[%d] ----------",
	                         pMCL->state,
	                         newState
	                         );

	if(pMCL->state != newState)
	{
		pMCL->state = newState;
		/*delete status info*/
	}
}

/**
* @brief  covert link config type to MCL type
*
* @param  linkConfigType
*
* @return  mcl type
*
*/
TBlueAPI_MCLType blueAPI_LinkConfigType2MCLType(TBlueAPI_LinkConfigType linkConfigType)
{
	switch (linkConfigType)
	{
	case blueAPI_LinkConfigGATT: 
		return blueAPI_MCLType_GATT;

	case blueAPI_LinkConfigSDP:
		return blueAPI_MCLType_SDP;

	default: 
		BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE,
		                        "!!blueAPI_LinkConfigTyp2MCLType: unknown type"
		                        );
		return blueAPI_MCLType_Invalid;
	}
}

/**
* @brief  get mcl by bdaddr
*
* @param  pBlueAPIdata:
* @param  pBD: bdaddr
* @param  mclType:  mcltype
*
* @return mcl
*
*/
PBlueAPI_MCL blueAPI_MCLFindByBD(uint8_t *pBD, TBlueAPI_MCLType mclType)
{
    uint16_t loop;
    PBlueAPI_MCL pMCL;

    for (loop = 0; loop < otp_str_data.gEfuse_UpperStack_s.num_link_don; loop++)
    {
        pMCL = &pBlueAPIData->pMCLDescriptorTableDon[loop];

        if ((pMCL->state != blueAPI_DS_Reserved) && (pMCL->mclType == mclType) &&
            !memcmp(pMCL->bd, pBD, BD_ADDR_SIZE))
        {
            return pMCL;
        }
    }
    for (loop = 0; loop < otp_str_data.gEfuse_UpperStack_s.num_link_doff; loop++)
    {
        pMCL = &pBlueAPIData->pMCLDescriptorTableDoff[loop];

        if ((pMCL->state != blueAPI_DS_Reserved) && (pMCL->mclType == mclType) &&
            !memcmp(pMCL->bd, pBD, BD_ADDR_SIZE))
        {
            return pMCL;
        }
    }

    return NULL;
}

/**
* @brief  find MCL by local mdl id
*
* @param  pBlueAPIdata: 
* @param  pMCL
*
* @return  
*
*/
PBlueAPI_MCL blueAPI_MCLFindByLocal_MDL_ID(uint16_t local_MDL_ID)
{
    uint16_t loop1;
    PBlueAPI_MCL pMCL;
    PBlueAPI_MDL pMDL;
    PBlueAPI_LinkDescriptor pLinkContext;

    for (loop1 = 0; loop1 < otp_str_data.gEfuse_UpperStack_s.num_link_don; loop1++)
    {
        pMCL = &pBlueAPIData->pMCLDescriptorTableDon[loop1];

        pLinkContext = pMCL->pDataChannel;
        if (pLinkContext)
        {
            pMDL = pLinkContext->pMDL;
            if (pMDL && (pMDL->local_MDL_ID == local_MDL_ID))
            {
                return pMCL;
            }
        }
    }

    for (loop1 = 0; loop1 < otp_str_data.gEfuse_UpperStack_s.num_link_doff; loop1++)
    {
        pMCL = &pBlueAPIData->pMCLDescriptorTableDoff[loop1];

        pLinkContext = pMCL->pDataChannel;
        if (pLinkContext)
        {
            pMDL = pLinkContext->pMDL;
            if (pMDL && (pMDL->local_MDL_ID == local_MDL_ID))
            {
                return pMCL;
            }
        }
    }

    return NULL;
}

/**
* @brief  get mcl from timechannel
* 
* @param  pBlueAPIdata:  
* @param  timerChannel: timeout channel
* @return  PBlueAPI_MCL
*
*/
PBlueAPI_MCL blueAPI_MCLFindByTimerChannel(PBlueAPI_Data pBlueAPIdata, uint16_t timerChannel)
{
    TBlueAPI_MCL *pMCLDescriptorTable = NULL;


    if (timerChannel > 0x10)
    {
        pMCLDescriptorTable = &pBlueAPIdata->pMCLDescriptorTableDon[timerChannel - 0x10 -1];
    }
    else
    {
        pMCLDescriptorTable = &pBlueAPIdata->pMCLDescriptorTableDoff[timerChannel - 1];
    }

    if (pMCLDescriptorTable->state != blueAPI_DS_Reserved)
    {
        return (pMCLDescriptorTable);
    }
    else
    {
        return NULL;
    }

}

/**
* @brief  release mcl
*
* @param  pBlueAPIdata
* @param  pMCL
*
* @return
*
*/
void blueAPI_MCLRelease(PBlueAPI_Data pBlueAPIdata, PBlueAPI_MCL pMCL)
{
    if (pMCL)
    {
        if (pMCL->state == blueAPI_DS_Reserved)
        {
            BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE, "!!!blueAPI: release reserved MCL!");
        }
        else
        {
            BLUEAPI_TRACE_PRINTF_3(BLUEAPI_TRACE_MASK_TRACE,
                               "blueAPI: release MCL [0x%8.8X]: BD[%s] state[%d]",
                               (uint32_t)pMCL,
                               TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE, pMCL->bd),
                               pMCL->state
                               );

            memset(pMCL,0,sizeof(TBlueAPI_MCL));

            pMCL->state                = blueAPI_DS_Reserved;
            pMCL->DS_CommandInProgress = blueAPI_EventIdle;
            pMCL->US_CommandInProgress = blueAPI_EventIdle;
        }
    }
    else
    {
        BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE, "!!!blueAPI: release NULL MCL!");
    }
}

/**
* @brief   allocate mcl
*
* @param  pBlueAPIdata:
* @param  pBD: 
* @param  bdType: 
* @param  mclType
* @param  remote_Control_PSM
* @param  remote_Data_PSM
*
* @return  point to comapp
*
*/
PBlueAPI_MCL blueAPI_MCLAllocate(uint8_t * pBD, TBlueAPI_RemoteBDType bdType,
    TBlueAPI_MCLType mclType)
{
    uint16_t loop;
    PBlueAPI_MCL pMCL;
    BOOL bFound = FALSE;
    BOOL bLinkDon = FALSE;

    for (loop = 0; loop < otp_str_data.gEfuse_UpperStack_s.num_link_don; loop++)
    {
        pMCL = &pBlueAPIData->pMCLDescriptorTableDon[loop];

        if (pMCL->state == blueAPI_DS_Reserved)
        {
            bFound = TRUE;
            bLinkDon = TRUE;
            goto TagEND;
        }
    }

    for (loop = 0; loop < otp_str_data.gEfuse_UpperStack_s.num_link_doff; loop++)
    {
        pMCL = &pBlueAPIData->pMCLDescriptorTableDoff[loop];

        if (pMCL->state == blueAPI_DS_Reserved)
        {
            bFound = TRUE;
            bLinkDon = FALSE;
            goto  TagEND;
        }
    }

    goto TagEND;

TagEND:
    if (bFound == TRUE)
    {
        memset(pMCL, 0, sizeof(TBlueAPI_MCL));

        memcpy(pMCL->bd, pBD, 6);
        pMCL->bdType                   = bdType;
        pMCL->mclType                  = mclType;

        pMCL->state                    = blueAPI_DS_Idle;
        pMCL->DS_CommandInProgress     = blueAPI_EventIdle;
        pMCL->US_CommandInProgress     = blueAPI_EventIdle;

        if (bLinkDon == TRUE)
        {
            pMCL->number = loop + 1 + 0x10;
        }
        else
        {
            pMCL->number = loop + 1;
        }

        BLUEAPI_TRACE_PRINTF_4(BLUEAPI_TRACE_MASK_TRACE,
            "blueAPI: alloc MCL [0x%08X]:====>: BD[%s] bdType[%d] state[%d]", (uint32_t)pMCL,
            TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE, pMCL->bd), bdType, pMCL->state);
    }
    else
    {
        BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_ERROR, "!!blueAPI: alloc MCL failed");
        pMCL = NULL;
    }

    return pMCL;
}

/**
* @brief  clear upstream context
*
* @param pMCL
*
* @return 
*
*/
void blueAPI_MCL_US_ContextClear(PBlueAPI_MCL pMCL)
{
    BLUEAPI_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_TRACE, "blueAPI: MCL[%8.8X] US_COM_Command[%d] done",
        (uint32_t)pMCL, pMCL->US_CommandInProgress);

    pMCL->US_CommandInProgress = blueAPI_EventIdle;
    memset(&pMCL->US_CommandData, 0, sizeof(TBlueAPI_UsCommandData));
}

/**
* @brief  find link by MDL ID
*
* @param  pBlueAPIdata: 
* @param  local_MDL_ID: local mdl id
*
* @return  
*
*/
PBlueAPI_LinkDescriptor blueAPI_ChannelFindByLocal_MDL_ID(uint16_t local_MDL_ID)
{
    PBlueAPI_LinkDescriptor pLinkContext;
    uint16_t loop;

    for (loop = 0; loop < otp_str_data.gEfuse_UpperStack_s.num_link_don; loop++)
    {
        pLinkContext = &pBlueAPIData->plinkDescriptorTableDon[loop];

        if (pLinkContext->linkState != blueAPI_LinkStateReserved)
        {
            if (pLinkContext->pMDL)
            {
                if (pLinkContext->pMDL->local_MDL_ID == local_MDL_ID)
                {
                    return pLinkContext;
                }
            }
        }
    }

    for (loop = 0; loop < otp_str_data.gEfuse_UpperStack_s.num_link_doff; loop++)
    {
        pLinkContext = &pBlueAPIData->plinkDescriptorTableDoff[loop];

        if (pLinkContext->linkState != blueAPI_LinkStateReserved)
        {
            if (pLinkContext->pMDL)
            {
                if (pLinkContext->pMDL->local_MDL_ID == local_MDL_ID)
                {
                    return pLinkContext;
                }
            }
        }
    }

    BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_ERROR,
        "!!blueAPI_ChannelFindByLocal_MDL_ID:local_MDL_ID[0x%x] return NULL", local_MDL_ID);

    return NULL;
}

/**
* @brief  get mcl link using state
*
* @param  pMCL: 
* @param  linkState: link state
*
* @return  
*
*/
PBlueAPI_LinkDescriptor blueAPI_MCLFindLinkByState(PBlueAPI_MCL pMCL,TBlueAPI_LinkState linkState)
{
    PBlueAPI_LinkDescriptor pLinkContext;

#if CHECK_API_PARAM
    if (!pMCL)
    {
        BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE, "!!blueAPI: find link in NULL MCL!");
        return NULL;
    }

    if (pMCL->state == blueAPI_DS_Reserved)
    {
        BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE, "!!blueAPI: find link in idle MCL!");
        return NULL;
    }
#endif

    pLinkContext = pMCL->pDataChannel;

    if (pLinkContext)
    {
        if (pLinkContext->linkState == linkState)
        {
            return pLinkContext;
        }
    }

    return NULL;
}

/**
* @brief  add mcl data channel
*
* @param  pMCL:
* @param  pLinkContext:  data link
*
* @return  add result
*
*/
BOOL blueAPI_MCLAddDataChannel(PBlueAPI_MCL pMCL, PBlueAPI_LinkDescriptor pLinkContext)
{
    if (pMCL)
    {
        if (pMCL->pDataChannel == NULL)
        {  
            pMCL->pDataChannel = pLinkContext;
            return FALSE;
        }

        BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE,
                                "!!blueAPI: no space for data channel in MCL!!!"
                                );
    }
    else
    {
        BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE,
                                "!!blueAPI: add data channel in NULL MCL!!!"
                                );
    }

   return TRUE; /* error */
}

/**
* @brief  release channel
*
* @param  pBlueAPIdata: 
* @param  pLinkContext
*
* @return  
*
*/
void blueAPI_ChannelRelease(PBlueAPI_Data pBlueAPIdata, PBlueAPI_LinkDescriptor pLinkContext)
{
    if(pLinkContext)
    {
        if(pLinkContext->linkState == blueAPI_LinkStateReserved)
        {
            BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE,
                               "!!blueAPI: release unused linkContext!"
                               );
        }
        else
        {
            BLUEAPI_TRACE_PRINTF_4(BLUEAPI_TRACE_MASK_TRACE,
                               "====>: MCL[0x%8.8X] BD[%s] state[%d] subRole[%d]",
                               (uint32_t)pLinkContext->pMCL,
                               TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE,pLinkContext->pMCL ? pLinkContext->pMCL->bd : NULL),
                               pLinkContext->linkState,
                               pLinkContext->linkSubRole
                               );

            pLinkContext->pMCL->pDataChannel = NULL;

            memset(pLinkContext,0,sizeof(TBlueAPI_LinkDescriptor));
            pLinkContext->linkState = blueAPI_LinkStateReserved;
        }
    }
    else
    {
        BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE,
                        "!!blueAPI: release NULL linkContext!"
                        );
    }
}

/**
* @brief  set channel configuration
*
* @param  pBlueAPIdata:
* @param  pLinkContext: 
* @param  linkConfigType: link  config type
*
* @return  
*
*/
void blueAPI_ChannelSetConfiguration(PBlueAPI_Data pBlueAPIdata,PBlueAPI_LinkDescriptor pLinkContext,TBlueAPI_LinkConfigType linkConfigType)
{
	switch(linkConfigType)
	{
	case blueAPI_LinkConfigGATT:
	case blueAPI_LinkConfigSDP: 
		pLinkContext->linkConfigType = linkConfigType;	/*just set link config type*/
		break;

	default:
#if SEND_INT_EVENT_INFO      
		blueAPI_Send_InternalEventInfo(blueAPI_InternalEventInvalidRemoteEvent,
		                             BLUE_API_GENERATE_EVENT_ID,
		                             blueAPI_CauseInvalidParameter
		                             );
#endif
		break;
	}
}

/**
* @brief  allocate channel link des
*
* @param  pBlueAPIdata:
* @param  pMCL: 
* @param  linkSubRole: sub role
* @param  linkConfigType: link  config type
*
* @return  
*
*/
PBlueAPI_LinkDescriptor blueAPI_ChannelAllocate(PBlueAPI_MCL pMCL, TBlueAPI_LinkSubRole linkSubRole, TBlueAPI_LinkConfigType linkConfigType)
{
    PBlueAPI_LinkDescriptor pLinkContext;
    uint16_t                 loop;

    for (loop = 0; loop < otp_str_data.gEfuse_UpperStack_s.num_link_don; loop++)
    {
        pLinkContext = &pBlueAPIData->plinkDescriptorTableDon[loop];

        if (pLinkContext->linkState == blueAPI_LinkStateReserved)
        {
            goto TagFOUND;
        }
    }

    for (loop = 0; loop < otp_str_data.gEfuse_UpperStack_s.num_link_doff; loop++)
    {
        pLinkContext = &pBlueAPIData->plinkDescriptorTableDoff[loop];

        if (pLinkContext->linkState == blueAPI_LinkStateReserved)
        {
            goto TagFOUND;
        }
    }
    pLinkContext = NULL;
    BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_ERROR,
        "!!blueAPI_ChannelAllocate: alloc linkContext failed!");
TagFOUND:
    if (pLinkContext != NULL)
    {
        memset(pLinkContext, 0, sizeof(TBlueAPI_LinkDescriptor));

        if (pMCL)
        {
            if (blueAPI_MCLAddDataChannel(pMCL, pLinkContext))
            {
                BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_ERROR,
                    "!!blueAPI: add while alloc linkContext failed!");

                return NULL;
            }
        }
        pLinkContext->pMCL           = pMCL;
        pLinkContext->linkState      = blueAPI_Idle;
        pLinkContext->linkSubRole    = linkSubRole;
        
        blueAPI_ChannelSetConfiguration(pBlueAPIData, pLinkContext, linkConfigType);
    }

    return pLinkContext;
}

/**
* @brief  connect channel
*
* @param  pBlueAPIdata:
* @param  pMCL: 
* @param  pLinkContext: 
*
* @return  
*
*/
void blueAPI_ChannelConnect(PBlueAPI_Data pBlueAPIdata, PBlueAPI_MCL pMCL, PBlueAPI_LinkDescriptor pLinkContext)
{
    blueAPI_MCLSetState(pMCL, blueAPI_DS_DataConnecting);

    pLinkContext->linkState = blueAPI_Connecting;

    switch (pMCL->mclType)
    {
    case blueAPI_MCLType_GATT:
        {
            TCON_REQ_GATT tpGATTPara;
            PBlueAPI_ConnectMDLReq pConnectMDLReq = &pMCL->DS_CommandMsg->p.ConnectMDLReq;

            BLUEAPI_TRACE_PRINTF_3(BLUEAPI_TRACE_MASK_TRACE,
                "blueAPI_: send BT_CON_REQ link[0x%X] bd[%s] type[%d]", pLinkContext,
                TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE, pMCL->bd), pConnectMDLReq->remote_BD_Type);

            tpGATTPara.bdType = pConnectMDLReq->remote_BD_Type;
            tpGATTPara.localBdType = pConnectMDLReq->local_BD_Type;
            tpGATTPara.mtuSize  = 0;  /* not used .. */
            tpGATTPara.scanInterval = pConnectMDLReq->p.gatt.scanInterval;
            tpGATTPara.scanWindow = pConnectMDLReq->p.gatt.scanWindow;
            tpGATTPara.connIntervalMin = pConnectMDLReq->p.gatt.connIntervalMin;
            tpGATTPara.connIntervalMax = pConnectMDLReq->p.gatt.connIntervalMax;
            tpGATTPara.connLatency = pConnectMDLReq->p.gatt.connLatency;
            tpGATTPara.supervisionTimeout = pConnectMDLReq->p.gatt.supervisionTimeout;
            tpGATTPara.CE_Length = pConnectMDLReq->p.gatt.CE_Length;

            btApiBT_CON_REQ(pBlueAPIdata->AppHandle, pLinkContext, pMCL->bd, 0, BLUEFACE_PSM_GATT, 0,
                sizeof(tpGATTPara), (uint8_t *)&tpGATTPara);

            /* start LE scan timeout (0 == wait forever) */
            if ((pConnectMDLReq->remote_BD_Type != blueAPI_RemoteBDTypeClassic) &&
                (pConnectMDLReq->p.gatt.scanTimeout != 0))
            {
                blueAPI_TimerStart(pMCL, blueAPI_TimerID_LEScanTO, pConnectMDLReq->p.gatt.scanTimeout*10);
            }
        }
        break;

    case blueAPI_MCLType_SDP: 
        btApiBT_CON_REQ(pBlueAPIdata->AppHandle, pLinkContext, pMCL->bd, 0, BLUEFACE_PSM_SDP, 0, 0, NULL);
        break;

    default: 
        BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_ERROR, "!!blueAPI: failed to connect with invalid MCL type");
        break;
    }
}

/**
* @brief  accept channel
*
* @param  pBlueAPIdata:
* @param  pLinkContext: 
* @param  accept: 
*
* @return  
*
*/
void blueAPI_ChannelAccept(PBlueAPI_Data pBlueAPIdata, PBlueAPI_LinkDescriptor pLinkContext, BOOL accept)
{
    if (pLinkContext)
    {
        BLUEAPI_TRACE_PRINTF_3(BLUEAPI_TRACE_MASK_TRACE,
            "blueAPI: send BT_CON_RESP link[0x%X] bd[%s] accept[%d]", pLinkContext,
            TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE, pLinkContext->pMCL->bd), accept);

        switch (pLinkContext->pMCL->mclType)
        {
        case blueAPI_MCLType_GATT: 
            btApiBT_CON_RESP(pLinkContext->handle, (BLINKHANDLE)pLinkContext,
                accept ? BLUEFACE_CON_ACCEPT : BLUEFACE_CON_REJECT, 0, NULL);
            break;

        default:
            BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE,
                "!!blueAPI: failed to accept with invalid MCL type");
            break;
        }
    }
}

/**
* @brief  disconnect blueapi channel
* 
* @param  pBlueAPIdata:  
* @param  pLinkContext: link
* @param  ignoreMCLState: weather set mcl state
* @return  
*
*/
void blueAPI_ChannelDisconnect(PBlueAPI_Data pBlueAPIdata, PBlueAPI_LinkDescriptor pLinkContext, BOOL ignoreMCLState)
{
    if(pLinkContext)
    {
        BLUEAPI_TRACE_PRINTF_4(BLUEAPI_TRACE_MASK_TRACE,
                                "blueAPI: send BT_DISC_REQ link[0x%X] BD[%s] MCL[%8.8X] locMDL[%d]",
                                pLinkContext,
                                TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE, pLinkContext->pMCL->bd),
                                (uint32_t)pLinkContext->pMCL,
                                (pLinkContext->pMDL) ? pLinkContext->pMDL->local_MDL_ID : 0
                                );

        if (ignoreMCLState == FALSE)
        {
            blueAPI_MCLSetState(pLinkContext->pMCL, blueAPI_DS_DataDisconnecting);
        }

        pLinkContext->linkState = blueAPI_Disconnecting;

        btApiBT_DISC_REQ(pLinkContext->handle, 0 /* cause */,0 /* holdlink=0 => ACL connection is shutdown immediately !!!! */);
    }
    else
    {
        BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE, "!!blueAPI: disconnect with NULL link descriptor");
    }
}

/**
* @brief   set pending flags
*
* @param  pMCL: 
* @param  cause
*
* @return  
*
*/
void blueAPI_MDLAddFlags(PBlueAPI_MDL pMDL, uint16_t addFlags)
{
#if (BLUEAPI_TRACE_VERBOSITY_COUNT > 0)
#if DEBUG_MDL_FLAGS 
	BLUEAPI_TRACE_PRINTF_3(BLUEAPI_TRACE_MASK_TRACE,
	                         "blueAPI: MDL[0x%8.8X] flags[%x] add[%x]",
	                         (uint32_t)pMDL,
	                         pMDL->pendingFlags,
	                         addFlags)
	                         );
#endif   
#endif
	pMDL->pendingFlags |= addFlags;
}

/**
* @brief  delete MDL pending flags
*
* @param  pMDL
* @param  subFlags
*
* @return  
*
*/
void blueAPI_MDLSubFlags(PBlueAPI_MDL pMDL, uint16_t subFlags)
{
#if (BLUEAPI_TRACE_VERBOSITY_COUNT > 0)
	BLUEAPI_TRACE_PRINTF_3(BLUEAPI_TRACE_MASK_TRACE,
	                         "blueAPI: MDL[0x%8.8X] flags[%d] sub[%d]",
	                         (uint32_t)pMDL,
	                         pMDL->pendingFlags,
	                         subFlags
	                         );
#endif

	pMDL->pendingFlags &= ~subFlags;
}

/**
* @brief  find MDL by flags
*
* @param  pBlueAPIdata: 
* @param  pMCL
* @param  flags
*
* @return  
*
*/
PBlueAPI_MDL blueAPI_MDLFindByFlags(PBlueAPI_Data pBlueAPIdata, PBlueAPI_MCL pMCL, uint16_t flags)
{
	PBlueAPI_MDL pMDL;
	uint16_t loop;

	for (loop = 0; loop < otp_str_data.gEfuse_UpperStack_s.num_link_don; loop++)
    {
        pMDL = &pBlueAPIdata->pMDLDescriptorTableDon[loop];

				if(!memcmp(pMDL->remoteBd, pMCL->bd, 6))
				{  
						if((pMDL->pendingFlags)&&((pMDL->pendingFlags & flags) == flags))
						{  
								return pMDL;
						}
				}
    }

    for (loop = 0; loop < otp_str_data.gEfuse_UpperStack_s.num_link_doff; loop++)
    {
        pMDL = &pBlueAPIdata->pMDLDescriptorTableDoff[loop];

				if(!memcmp(pMDL->remoteBd, pMCL->bd, 6))
				{  
						if((pMDL->pendingFlags)&&((pMDL->pendingFlags & flags) == flags))
						{  
								return pMDL;
						}
				}
    }
	
	return NULL;
}

/**
* @brief  find mdl by local mdl id
*
* @param  pBlueAPIdata: 
* @param  local_MDL_ID
*
* @return  
*
*/
PBlueAPI_MDL blueAPI_MDLFindByLocal_MDL_ID(PBlueAPI_Data pBlueAPIdata, uint16_t local_MDL_ID)
{
    PBlueAPI_MDL pMDL;
    uint16_t      loop;
    
    for (loop = 0; loop < otp_str_data.gEfuse_UpperStack_s.num_link_don; loop++)
    {
        pMDL = &pBlueAPIdata->pMDLDescriptorTableDon[loop];

        if (pMDL->local_MDL_ID == local_MDL_ID)
        {
            return pMDL;
        }
    }

    for (loop = 0; loop < otp_str_data.gEfuse_UpperStack_s.num_link_doff; loop++)
    {
        pMDL = &pBlueAPIdata->pMDLDescriptorTableDoff[loop];

        if (pMDL->local_MDL_ID == local_MDL_ID)
        {
            return pMDL;
        }
    }

    BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE, "!!blueAPI: local MDL_ID not found");

    return NULL;
}

/**
* @brief  release MDL
*
* @param  pMDL: 
*
* @return  
*
*/
void blueAPI_MDLRelease(PBlueAPI_MDL pMDL)
{
    if (pMDL)
    {
        if (pMDL->local_MDL_ID == BLUE_API_INVALID_MDL_ID)
        {
            BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE,
                               "!!blueAPI: release unallocated MDL[0x%8.8X] context!",
                               (uint32_t)pMDL
                               );
        }
        else
        {
            BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE,
                               "blueAPI: release MDL [0x%8.8X]:",
                               (uint32_t)pMDL
                               );

            BLUEAPI_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_TRACE,
                               "====>: BD[%s] locMDL[%d] locMDEP[%d]",
                               TRACE_BDADDR1(BLUEAPI_TRACE_MASK_TRACE, pMDL->remoteBd),
                               pMDL->local_MDL_ID
                               );
            memset(pMDL, 0, sizeof(TBlueAPI_MDL));
            pMDL->local_MDL_ID = BLUE_API_INVALID_MDL_ID;
        }
    }
    else
    {
        BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE, "!!blueAPI: release NULL MDL context!");
    }
}

/**
* @brief  get next mdl local id
*
* @param  pBlueAPIdata:
*
* @return  point to comapp
*
*/
//#if (BLUE_API_MDL_COUNT != 2)
uint16_t blueAPI_MDLGetNextLocal(PBlueAPI_Data pBlueAPIdata)
{
    uint16_t local_MDL_ID;
    uint16_t loop;
    PBlueAPI_MDL pMDL = NULL;

    do
    {
        local_MDL_ID = pBlueAPIdata->nextLocal_MDL_ID++;
        if (pBlueAPIdata->nextLocal_MDL_ID == BLUE_API_LAST_LOCAL_MDL_ID)
        {
            pBlueAPIdata->nextLocal_MDL_ID = BLUE_API_FIRST_MDL_ID;
        }

        /* check that new local_MDL_ID is not in use */
        for (loop = 0; loop < otp_str_data.gEfuse_UpperStack_s.num_link_don; loop++)
        {
            pMDL = &pBlueAPIdata->pMDLDescriptorTableDon[loop];

            if (pMDL->local_MDL_ID == local_MDL_ID)
            {
                local_MDL_ID = BLUE_API_INVALID_MDL_ID;
                break;
            }
        }

        for (loop = 0; loop < otp_str_data.gEfuse_UpperStack_s.num_link_doff; loop++)
        {
            pMDL = &pBlueAPIdata->pMDLDescriptorTableDoff[loop];

            if (pMDL->local_MDL_ID == local_MDL_ID)
            {
                local_MDL_ID = BLUE_API_INVALID_MDL_ID;
                break;
            }
        }

    } while (local_MDL_ID == BLUE_API_INVALID_MDL_ID);

    return local_MDL_ID;
} /* blueAPI_MDLGetNextLocal */
//#endif

/**
* @brief  allcoate mdl
*
* @param  pBlueAPIdata:
* @param  pReqMsg: 
* @param  remote_BD: 
*
* @return  point to comapp
*
*/
PBlueAPI_MDL blueAPI_MDLAllocate(PBlueAPI_MCL pMCL, TBlueAPI_LinkConfigType linkConfigType)
{
    PBlueAPI_MDL pMDL;
    uint16_t loop;

    for (loop = 0; loop < otp_str_data.gEfuse_UpperStack_s.num_link_don; loop++)
    {
        pMDL = &pBlueAPIData->pMDLDescriptorTableDon[loop];

        if (pMDL->local_MDL_ID == BLUE_API_INVALID_MDL_ID)
        {
            goto TagSuccess;
        }
    }

    for (loop = 0; loop < otp_str_data.gEfuse_UpperStack_s.num_link_doff; loop++)
    {
        pMDL = &pBlueAPIData->pMDLDescriptorTableDoff[loop];

        if (pMDL->local_MDL_ID == BLUE_API_INVALID_MDL_ID)
        {
            goto TagSuccess;
        }
    }
    goto TagFail;

TagSuccess:
    memset(pMDL, 0, sizeof(TBlueAPI_MDL));
    memcpy(pMDL->remoteBd, pMCL->bd, BD_ADDR_SIZE);
    pMDL->remoteBdType = pMCL->bdType;
    pMDL->local_MDL_ID = blueAPI_MDLGetNextLocal(pBlueAPIData);
    pMDL->linkConfigType = linkConfigType;

    blueAPI_MDLAddFlags(pMDL, BLUE_API_MDL_FLAG_NOT_ACTIVATED);

    TRACE_DUMP_MDL(pMDL,"alloc MDL:");

    return pMDL;

TagFail:
    BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_ERROR,
        "!!blueAPI_MDLAllocate: alloc MDL context failed!");
    return NULL;
}

/**
* @brief  get next com command in queue
*
* @param  pMCL
*
* @return  
*/
PBlueAPI_COMCommandElement blueAPI_GetNextCOMCommand(PBlueAPI_MCL pMCL)
{
#if BLUE_API_MCL_ELEMENT_COUNT
    if (pMCL->COMCommandQueue.Count)
    {
        return (PBlueAPI_COMCommandElement)osQueueOut(&pMCL->COMCommandQueue);
    }
    else
    {
        return NULL;
    }
#else
    return NULL;
#endif
}

/**
* @brief  get COMCommandQueue element
*
* @param  pMCL:
*
* @return 
*
*/
PBlueAPI_COMCommandElement blueAPI_GetCOMCommandElement(PBlueAPI_MCL pMCL)
{
#if BLUE_API_MCL_ELEMENT_COUNT
    uint16_t                    loop;
    PBlueAPI_COMCommandElement pElement;

    for (loop = 0; loop < BLUE_API_MCL_ELEMENT_COUNT; loop++)
    {  
        pElement = &pMCL->COMCommandElement[loop];
        if (!pElement->used)
        {  
            pElement->used = TRUE;
            return pElement;
        }
    }
#endif
   return NULL;
}

/**
* @brief  QUEUE com msg to COMCommandQueue
*
* @param  pMCL:
* @param  pCOMCommand: 
*
* @return 
*
*/
BOOL blueAPI_QueueCOMCommand(PBlueAPI_MCL pMCL, PBlueAPI_DsMessage pCOMCommand)
{
#if BLUE_API_MCL_ELEMENT_COUNT
    PBlueAPI_COMCommandElement pElement = blueAPI_GetCOMCommandElement(pMCL);

    if(pElement)
    {  
        pElement->lpCOMMsg = pCOMCommand;
        osQueueIn(&pMCL->COMCommandQueue, pElement);

        BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE,
                                "blueAPI: queue COM command %d",
                                pCOMCommand->Command
                                );

        return FALSE;
    } else
    {
        BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE,
                                "!!blueAPI: out of queue elements"
                                );

        return TRUE;
    }
#else
	return TRUE;
#endif
}

/**
* @brief  release com command element
*
* @param  pElement
*
* @return  
*
*/
void blueAPI_ReleaseCOMCommandElement(PBlueAPI_COMCommandElement pElement)
{
   pElement->used = FALSE;
}

/**
* @brief  check if mcl is in transition
*
* @param  pMCL
*
* @return
*
*/
BOOL blueAPI_MCLInTransition(PBlueAPI_MCL pMCL)
{
    if (pMCL->state == blueAPI_DS_DataListening)
    {
        return TRUE;
    }

    if ((pMCL->US_CommandInProgress == blueAPI_EventIdle)&&
        (pMCL->DS_CommandInProgress == blueAPI_EventIdle)
       )
    {
        return FALSE;
    }

    return TRUE;
}

/**
* @brief  retrigger mcl
*
* @param  pBlueAPIdata
* @param  pMCL
*
* @return
*
*/
BOOL blueAPI_MCLRetrigger(PBlueAPI_Data pBlueAPIdata, PBlueAPI_MCL pMCL)
{
	PBlueAPI_MDL               pMDL         = blueAPI_MDLFindByFlags(pBlueAPIdata,pMCL, BLUE_API_MDL_FLAG_ANNY_FLAG);
	PBlueAPI_COMCommandElement pNextCommand;
//	PBlueAPI_LinkDescriptor    pLinkContext;

	/* check for deferred actions */
	while(pMDL)
	{  
		if(pMDL->pendingFlags & BLUE_API_MDL_FLAG_DISCONNECT_IND)
		{
			BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE,
			                       "blueAPI: continue MDL[0x%4.4X] with pending MDLDisconnectInd",
			                       (uint32_t)pMDL
			                       );

			blueAPI_Send_DisconnectMDLInd( pMCL, pMDL, pMDL->pendingCause);
			return TRUE; /* we are busy again... */
		}

		if(pMDL->pendingFlags & BLUE_API_MDL_FLAG_DELETE_INFO)
		{
			BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE,
			                       "blueAPI: continue MDL[0x%4.4X] with pending MDLDeleteInfo",
			                       (uint32_t)pMDL
			                       );

			blueAPI_Send_DeleteMDLInfo(pMDL);
		}
		if(pMDL->pendingFlags & BLUE_API_MDL_FLAG_DELETE)
		{
			BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE,
			                       "blueAPI: continue MDL[0x%4.4X] with pending MDLDelete",
			                       (uint32_t)pMDL
			                       );

			blueAPI_MDLRelease(pMDL);
		}

		if(pMDL->pendingFlags&BLUE_API_MDL_FLAG_NOT_ACTIVATED)
		{
			BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE,
			                       "blueAPI: clean Up MDL[0x%4.4X] not activated",
			                       (uint32_t)pMDL
			                       );

			blueAPI_Send_DeleteMDLInfo(pMDL);
			blueAPI_MDLRelease(pMDL);
		}

		pMDL = blueAPI_MDLFindByFlags(pBlueAPIdata, pMCL, BLUE_API_MDL_FLAG_ANNY_FLAG);
	}

	/* check for deferred COM commands */
	pNextCommand = blueAPI_GetNextCOMCommand(pMCL);

	if(pNextCommand)
	{
		BLUEAPI_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_TRACE,
		                        "blueAPI: continue MCL[0x%4.4X] with pending[%d]",
		                        (uint32_t)pMCL,
		                        pNextCommand->lpCOMMsg->Command
		                        );
		blueAPI_Handle_Command(pNextCommand->lpCOMMsg);
		blueAPI_ReleaseCOMCommandElement(pNextCommand);
		return TRUE; /* we are busy again... */
	}

	return FALSE; /* we are completely done and stable */
}

/**
* @brief  check if mcl link disconnect
*
* @param  pBlueAPIdata
* @param  pMCL
*
* @return
*
*/
void blueAPI_CheckMCLDisconnect(PBlueAPI_Data pBlueAPIdata, PBlueAPI_MCL pMCL)
{
    if (blueAPI_MCLInTransition(pMCL))
    {
        if (pMCL->US_CommandInProgress != blueAPI_EventIdle)
        {
            BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE,
                "blueAPI: MCL wait for App confirmation to[%d]", pMCL->US_CommandInProgress);
        }
        else if (pMCL->DS_CommandInProgress != blueAPI_EventIdle)
        {
            BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE,
                "blueAPI: MCL working on App request[%d]", pMCL->DS_CommandInProgress);
        }
        else if (pMCL->state == blueAPI_DS_DataListening)
        {
            BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE,
                "blueAPI: MCL waiting for incomming data channel to connect");
        }
        else
        {
            BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE,
                "!!!blueAPI: MCL in unknown transition!!!");
        }
    }
    else if (pMCL->pDataChannel != NULL)
    {
        /* we still have data channels */
        /* ==> keep MCL  */
        BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE,
            "blueAPI: MCL stable with data channel connected");

        blueAPI_MCLRetrigger(pBlueAPIdata,pMCL);
    }
    else if (blueAPI_MCLRetrigger(pBlueAPIdata,pMCL))
    {
        /* continue...  */
    }
    else
    {
        BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE,
            "blueAPI: MCL Idle with all data channel and control channel closed => done");

        blueAPI_MCLRelease(pBlueAPIdata, pMCL);
        pMCL = NULL;
    }
}

/**
* @brief  set sdp downstream cmd
*
* @param  pBlueAPIdata: 
* @param pMsg
*
* @return  
*
*/
void blueAPI_SDP_DS_ContextSet(PBlueAPI_Data pBlueAPIdata, PBlueAPI_DsMessage pMsg)
{
    pBlueAPIdata->SM_SDP_DS_Command = (TBlueAPI_Command)pMsg->Command;
    pBlueAPIdata->SM_SDP_DS_CommandMsg = pMsg;

    BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE,
        "blueAPI: SM_SDP_DS_COM_Command[%d] pending ----------------------------------",
        pBlueAPIdata->SM_SDP_DS_Command);
}

/**
* @brief  clear sdp downstream
*
* @param  pBlueAPIdata: 
* @param pMsg
*
* @return  
*
*/
void blueAPI_SDP_DS_ContextClear(PBlueAPI_Data pBlueAPIdata, PBlueAPI_DsMessage pMsg)
{
    if (pMsg == NULL)
    {
        BLUEAPI_TRACE_PRINTF_0(BLUEAPI_TRACE_MASK_TRACE, "!!==>: pReqMsg is NULL");
    }
    else if (pBlueAPIdata->SM_SDP_DS_CommandMsg == pMsg)
    {
        BLUEAPI_TRACE_PRINTF_1(BLUEAPI_TRACE_MASK_TRACE,
            "blueAPI: SM_SDP_DS_Command[%d] done **********************************",
            pBlueAPIdata->SM_SDP_DS_Command);

        pBlueAPIdata->SM_SDP_DS_Command = blueAPI_EventIdle;
        pBlueAPIdata->SM_SDP_DS_CommandMsg = NULL;
    }
}

/**
* @brief  blueapi gatt convert cause
*
* @param  gattCause:
*
* @return  
*
*/
TBlueAPI_Cause blueAPI_GATTConvertCause(uint16_t wCause)
{
    TBlueAPI_Cause cause;

    if ((wCause & 0xFF00) == ATT_ERR)
    {
        cause = blueAPI_CauseLowerLayerError;
    }
    else
    {
        switch (wCause)
        {
        case GATT_ERR_INTERNAL:
            cause = blueAPI_CauseUnspecified;
            break;

        case GATT_SUCCESS:
            cause = blueAPI_CauseSuccess;
            break;

        case GATT_ERR_OUT_OF_RESOURCE:
            cause = blueAPI_CauseResourceError;
            break;

        case GATT_ERR_UNSUPPORTED:
            cause = blueAPI_CauseNotSupported;
            break;

        case GATT_ERR_ILLEGAL_ROLE:
        case GATT_ERR_ILLEGAL_STATE:
        case GATT_ERR_LINK_DEACTIVATED:
            cause = blueAPI_CauseInvalidState;
            break;

        case GATT_ERR_ILLEGAL_CODING:
        case GATT_ERR_ILLEGAL_HANDLE:
        case GATT_ERR_ILLEGAL_PARAMETER:
            cause = blueAPI_CauseInvalidParameter;
            break;

        case GATT_ERR_NOTIF_IND_CFG:
        case GATT_ERR_NOTIF_IND_CONF_PD:
            cause = blueAPI_CauseAccept;
            break;

        case GATT_ERR_NOT_ALLOWED:
        case GATT_ERR_NOTIF_IND_NOT_CFG:
            cause = blueAPI_CauseReject;
            break;

        case GATT_ERR_TIMEOUT:
            cause = blueAPI_CauseInitTimeout;
            break;

        /* invalid parameter (check in hci_le.c) */
        case (HCI_ERR | HCI_ERR_INVALID_HCI_PARAMETER_VALUE):
            cause = blueAPI_CauseInvalidParameter;
            break;

        /* no hci handle / invalid parameter (check in l2c_le.c) */
        case (L2CAP_ERR | L2CAP_ERR_ILLEGAL_PARAMETER):
            cause = blueAPI_CauseInvalidParameter;
            break;

        /* connection update in progress */
        case (L2CAP_ERR | L2CAP_ERR_PENDING):
            cause = blueAPI_CauseInvalidState;
            break;

        /* created in L2CAP, master denied parameter update */
        case (HCI_ERR | HCI_ERR_UNACCEPTABLE_CONNECTION_INTERVAL):
            cause = blueAPI_CauseReject;
            break;

        case (HCI_ERR | HCI_ERR_DIRECTED_ADVERTISING_TIMEOUT):
            cause = blueAPI_CauseConnectionDisconnect;
            break;

        case (HCI_ERR | HCI_ERR_COMMAND_DISALLOWED):
            cause = blueAPI_CauseInvalidState;
            break;

        case SECMAN_ERR|SECMAN_ERR_LE_BD_NOT_RESOLVED:
            cause = blueAPI_CauseAddressNotResolved;
            break;

        default:
            cause = blueAPI_CauseUnspecified;
            break;
        }
    }

    return cause;
}

#if F_BT_LE_BT41_SUPPORT
TBlueAPI_Cause blueAPI_L2CAPConvertCause(uint16_t l2cCause)
{
    TBlueAPI_Cause  cause;

    if ( (l2cCause & 0xFF00) != L2CAP_ERR)
    {
        if (l2cCause == 0)
            cause = blueAPI_CauseSuccess;
        else
            cause = blueAPI_CauseUnspecified;
    }
    else
    {
        l2cCause = l2cCause & 0x00FF;
        switch ( l2cCause )
        {
        case L2CAP_NO_CAUSE:
            cause = blueAPI_CauseSuccess;
            break;
        case L2CAP_ERR_NO_RESOURCE:
            cause = blueAPI_CauseResourceError;
            break;

        case L2CAP_ERR_LE_CHANNEL_NOT_OPEN:
        case L2CAP_ERR_ILLEGAL_STATE:
            cause = blueAPI_CauseInvalidState;
            break;
        case L2CAP_ERR_LINK_NOT_EXIST:
            cause = blueAPI_CauseConnectionDisconnect;
			break;

        case L2CAP_ERR_ILLEGAL_PARAMETER:
            cause = blueAPI_CauseInvalidParameter;
            break;

        default:
            cause = blueAPI_CauseLowerLayerError;
            break;
        }
    }

    return ( cause );
}
#endif

/**
* @brief  blueapi set device name
*
* @param  pBlueAPIdata:
* @param  pName: device name
*
* @return  
*
*/
void blueAPI_SetDeviceName(PBlueAPI_Data pBlueAPIdata, uint8_t * pName)
{
    LPbtApplication  app = pBlueAPIdata->AppHandle;
    uint16_t   sourceLen;

    sourceLen = strlen((char *)pName);

    if (sourceLen > BLUE_API_DEVICE_NAME_LENGTH - 1)
    {
        sourceLen = BLUE_API_DEVICE_NAME_LENGTH - 1;
    }

    hciCommandChangeLocalName(pName, sourceLen);
    hciLaterEntry();

    sdpHandleCONFIG_GAP_DEVICE_NAME(pName, sourceLen);

    /* we assume that an application doing name requests handles sec man issues
           and handles all security mode 3 non link related stuff
        */
    if (app->bta->secApp == NULL)
    {
        app->bta->secApp = app;
        BLUEFACE_TRACE_PRINTF_1(BLUEFACE_TRACE_MASK_TRACE,"blueAPI_SetDeviceName: setting sec app to %lx", app);
    }
} /* blueAPI_SetDeviceName */

/**
* @brief  get eir device name
* 
* @param  pEIR: eir buf
* @param  pName: name get to 
* @param  nameSize: name size
*
* @return  
*
*/
void blueAPI_GetEIRDeviceName(uint8_t * pEIR, uint8_t * pName, uint16_t nameSize)
{
	uint16_t pos  = 0;
	uint16_t len  = pEIR[pos];

	pName[0] = '\0';

	while (len && (pos < BLUEFACE_EXTENDED_INQUIRY_RESPONSE_SIZE))
	{
		switch (pEIR[pos + 1])
		{
		case SDP_EIR_NAME_SHORT:
		case SDP_EIR_NAME:
			{
				uint16_t copyLen = ((len -1) <= (nameSize -1)) ? (len -1) : (nameSize -1);
				memcpy(pName, &pEIR[pos + 2], copyLen);
				pName[copyLen] = '\0';
			}
			break;

		default:
			break;
	}

	pos += len +1;   /* type/value length + length byte */
	len  = pEIR[pos];
	}
} /* blueAPI_GetEIRDeviceName */

/**
* @brief  connect sdp
*
* @param  pBlueAPIdata:
* @param  pReqMsg: 
* @param  remote_BD: 
*
* @return  point to comapp
*
*/
TBlueAPI_Cause blueAPI_SDP_Connect(PBlueAPI_Data pBlueAPIdata, PBlueAPI_DsMessage pReqMsg, uint8_t * remote_BD)
{
    PBlueAPI_MCL pMCL;
    PBlueAPI_MDL pMDL;
    PBlueAPI_LinkDescriptor pLinkContext;

#if CHECK_API_PARAM
    pMCL = blueAPI_MCLFindByBD(pBlueAPIdata, remote_BD, blueAPI_MCLType_SDP);
    if (pMCL != NULL)
    {
        return blueAPI_CauseInvalidState;
    }
#endif

    pMCL = blueAPI_MCLAllocate(remote_BD, blueAPI_RemoteBDTypeClassic, blueAPI_MCLType_SDP);
    if (pMCL == NULL)
    {
        return blueAPI_CauseResourceError;
    }

    pMDL = blueAPI_MDLAllocate(pMCL, blueAPI_LinkConfigSDP);

    if (pMDL == NULL)
    {
        blueAPI_MCLRelease(pBlueAPIdata, pMCL);
        return blueAPI_CauseResourceError;
    }

    pLinkContext = blueAPI_ChannelAllocate(pMCL, blueAPI_SubRoleInitiator, pMDL->linkConfigType);

    if (pLinkContext == NULL)
    {
        blueAPI_MDLRelease(pMDL);
        blueAPI_MCLRelease(pBlueAPIdata, pMCL);
        return blueAPI_CauseResourceError;
    }

    pLinkContext->pMDL = pMDL;
    blueAPI_ChannelConnect(pBlueAPIdata, pMCL, pLinkContext);
    pMCL->DS_CommandInProgress = (TBlueAPI_Command)pReqMsg->Command;
    pMCL->DS_CommandMsg = pReqMsg;

    return blueAPI_CauseSuccess;
} /* blueAPI_SDP_Connect */

/**
* @brief  sdp disconnect
*
* @param  pBlueAPIdata: 
* @param  pLinkContext
* @param  cause
*
* @return  
*
*/
void blueAPI_SDP_Disconnect(PBlueAPI_Data           pBlueAPIdata,
                            PBlueAPI_LinkDescriptor pLinkContext,
                            TBlueAPI_Cause          cause)
{
	switch (pLinkContext->pMCL->DS_CommandInProgress)
	{
	case blueAPI_EventSDPDiscoveryReq:
		blueAPI_Send_SDPDiscoveryRsp(pBlueAPIdata,
		                           pBlueAPIdata->SM_SDP_DS_CommandMsg,
		                           cause);
		break;
	case blueAPI_EventGATTSDPDiscoveryReq: 
		blueAPI_Send_GATTSDPDiscoveryRsp(
		                               pBlueAPIdata->SM_SDP_DS_CommandMsg,
		                               cause);
		break;
	case blueAPI_EventAuthReq: 
		blueAPI_Send_AuthRsp(
		                   pBlueAPIdata->SM_SDP_DS_CommandMsg,
		                   pBlueAPIdata->SM_SDP_DS_CommandMsg->p.AuthReq.remote_BD,
		                   cause);

		pBlueAPIdata->SM_SEC_DS_CommandMsg = NULL;
		blueAPI_SDP_DS_ContextClear(pBlueAPIdata, pBlueAPIdata->SM_SDP_DS_CommandMsg);
		break;
	default: 
		break;
	}

	if (pBlueAPIdata->pSearchConf)
	{
		osBufferRelease(((uint8_t *)pBlueAPIdata->pSearchConf - offsetof(TblueFaceMsg, p.sdpSearchConfirmation)));
		pBlueAPIdata->pSearchConf = NULL;
	}

	pLinkContext->pMCL->DS_CommandInProgress = blueAPI_EventIdle;
	pLinkContext->pMCL->DS_CommandMsg = NULL;
	blueAPI_MDLRelease(pLinkContext->pMDL);
	pBlueAPIdata->pSDPLinkContext = NULL;
} /* blueAPI_SDP_Disconnect */
