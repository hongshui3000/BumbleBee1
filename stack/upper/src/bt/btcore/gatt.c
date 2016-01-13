/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       gatt.c
* @brief     gatt init
* @details   
*
* @author   	gordon
* @date      	2015-07-09
* @version	v0.1
*/

#include <flags.h>
#include <gattdef.h>
#include <btcommon.h>
#include <l2c_api.h>
#include <upper_stack_global.h>
#include <os_mem.h>

#define TRACE_MODULE_ID     MID_BT_GATT
/**
* @brief  init gatt
* 
* @param pDataSegment: gatt memory
* @param TaskName: task name
*
* @return  init result
*
*/
BOOL  gattInit(void)
{
    pGATT = osMemoryAllocate(RAM_TYPE_DATA_ON, sizeof(TGATT));

	if(osCoroutineCreate(&gattQueueID, NULL, (void *)pGATT))
	{	
		return(FALSE);
	}

	gattInitData();
    
#if F_BT_BREDR_SUPPORT
    l2cUSendListenReq(PSM_ATT, gattQueueID, 1 );
#endif

    if (otp_str_data.gEfuse_UpperStack_s.bqb_en == 0) /* tifnan: no need in bqb mode */
    {
	    gattGATTServiceBuiltinInit();
    }

	return(TRUE);
}
