/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       blueapi_base.c
* @brief     default config set
* @details   
*
* @author   	gordon
* @date      	2015-06-26
* @version	v0.1
*/

#include <flags.h>
#include <rtl_types.h>
#include <blueface.h>
#include <btglbdef.h>
#include <bt_api.h>
#include <blueapi_types.h>
#include <os_pool.h>
#include <blueapi_def.h>
#include <bluemgr.h>
#include <blueapi_osif.h>
#include <hci_api.h>
#include <upper_stack_global.h>

#define TRACE_MODULE_ID  MID_BT_HDP
/**
* @brief  allocate com app
*
* @param  pBlueAPIdata:
*
* @return  
*
*/
PBlueAPI_App blueAPI_AllocCOMApp(PBlueAPI_Data pBlueAPIdata)
{
    if(pBlueAPIdata->appDescriptorTable.used == FALSE)
    {
        memset(&pBlueAPIdata->appDescriptorTable, 0x00, sizeof(TBlueAPI_App));
        pBlueAPIdata->appDescriptorTable.used         = TRUE;
        pBlueAPIdata->appDescriptorTable.pBlueAPIdata = pBlueAPIdata;
        return &pBlueAPIdata->appDescriptorTable;
    }

    return NULL;
}

/**
* @brief  get user set default param
* 
* @param pBlueAPIdata
* @return  
*
*/
void blueAPI_ConfigParameterInitUserDefault(PBlueAPI_Parameter pConfigParameter)
{
    pConfigParameter->StoreBondMode = blueAPI_StoreBondModeNVStore;
    pConfigParameter->StoreBondSize = 4;
}

/**
* @brief  
* 
* @param pBlueAPIdata
* @return  
*
*/
PVOID blueAPI_Main(PBlueAPI_Data pBlueAPIdata)
{
    TblueFaceReg reg;
    uint16_t loop;

    PBlueAPI_MCL pMCL = NULL;
    PBlueAPI_MDL pMDL = NULL;

    /* Multi MCL support, Init all descriptors */
    for (loop = 0; loop < otp_str_data.gEfuse_UpperStack_s.num_link_don; loop++)
    {
        pMCL = &pBlueAPIdata->pMCLDescriptorTableDon[loop];

        pMCL->state                = blueAPI_DS_Reserved;
        pMCL->DS_CommandInProgress = blueAPI_EventIdle;
        pMCL->US_CommandInProgress = blueAPI_EventIdle;
    }

    for (loop = 0; loop < otp_str_data.gEfuse_UpperStack_s.num_link_doff; loop++)
    {
        pMCL = &pBlueAPIdata->pMCLDescriptorTableDoff[loop];

        pMCL->state                = blueAPI_DS_Reserved;
        pMCL->DS_CommandInProgress = blueAPI_EventIdle;
        pMCL->US_CommandInProgress = blueAPI_EventIdle;
    }
    pBlueAPIdata->nextLocal_MDL_ID = BLUE_API_FIRST_MDL_ID;

    for (loop = 0; loop < otp_str_data.gEfuse_UpperStack_s.num_link_don; loop++)
    {
        pMDL = &pBlueAPIdata->pMDLDescriptorTableDon[loop];

        pMDL->local_MDL_ID = BLUE_API_INVALID_MDL_ID;
    }

    for (loop = 0; loop < otp_str_data.gEfuse_UpperStack_s.num_link_doff; loop++)
    {
        pMDL = &pBlueAPIdata->pMDLDescriptorTableDoff[loop];

        pMDL->local_MDL_ID = BLUE_API_INVALID_MDL_ID;
    }

    pBlueAPIdata->pActiveApp = &pBlueAPIdata->appDescriptorTable;

    pBlueAPIdata->SM_SDP_DS_Command     = blueAPI_EventIdle;
    pBlueAPIdata->SM_SDP_DS_CommandMsg  = NULL;

    blueAPI_ConfigParameterInitUserDefault(&pBlueAPIdata->ConfigParameter);

    reg.appContext = pBlueAPIdata;

    lblueFaceRegister(&reg);

    return (PVOID)pBlueAPIdata;
}
