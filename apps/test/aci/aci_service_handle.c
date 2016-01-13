enum { __FILE_NUM__ = 0 };
/**
*********************************************************************************************************
*               Copyright(c) 2014, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file      aci_service_handle.c
* @brief     low power handle when using ACI.
* @details   none.
* @author    Tifnan
* @date      2014-11-19
* @version   v0.1
* *********************************************************************************************************
*/
#include "btltp.h"
#include <ltplib.h>
#include "aci_service_handle.h"
//#include "rtl876x_flash_storage.h"
#include "trace.h"

ServiceTcb ServiceStore;
uint8_t DatabaseBuffer[ACI_SERVICE_DATABASE_MAX_SIZE];
uint16_t DataBasePos = 0;

void * ACI_GetBuffer(uint16_t length)
{
    uint16_t remain_len = ACI_SERVICE_DATABASE_MAX_SIZE - DataBasePos;
    void * pBuffer;
    if (length > remain_len)
    {
        DBG_BUFFER(MODULE_LTP, LEVEL_ERROR, "!!!LTP: ACI_GetBuffer failed remain %d len %d", 2, remain_len, length);
        return NULL;
    }
    pBuffer = &DatabaseBuffer[DataBasePos];
    DataBasePos += length;
    DBG_BUFFER(MODULE_LTP, LEVEL_INFO, "ACI_GetBuffer len %d DataBasePos %d", 2, length, DataBasePos);
    return pBuffer;
}

BOOL ACI_FreeBuffer(void * pBuffer, uint16_t length)
{
    void * pBuffer1 = (uint8_t *)pBuffer + length;
    void * pBuffer2 = &DatabaseBuffer[DataBasePos];
    if (pBuffer1 == pBuffer2)
    {
        DataBasePos -= length;
        DBG_BUFFER(MODULE_LTP, LEVEL_INFO, "!!!LTP: ACI_FreeBuffer success", 0);
        return TRUE;
    }
    DBG_BUFFER(MODULE_LTP, LEVEL_ERROR, "!!!LTP: ACI_FreeBuffer failed", 0);
    return FALSE;
}

uint8_t BTACIFindGATTServiceHandle(PBTLtp pBTLtp, void *serviceHandle)
{
    uint16_t i;
    for (i = 0; i < BT_GATT_SERVER_MAX_SERVICES_COUNT; i++)
    {
        if (pBTLtp->gattServiceTable[i].isUsed)
        {
            if (pBTLtp->gattServiceTable[i].serviceHandle == serviceHandle)
            {
                return i;
            }
        }
    }
    DBG_BUFFER(MODULE_LTP, LEVEL_ERROR, "!!!LTP: BTLTPFindGATTServiceHandle failed %x", 1, serviceHandle);
    return 0;
}

PGattServiceTable BTACIAllocateGATTServiceHandle(PBTLtp pBTLtp)
{
    uint16_t i;
    for (i = 0; i < BT_GATT_SERVER_MAX_SERVICES_COUNT; i++)
    {
        if (pBTLtp->gattServiceTable[i].isUsed == FALSE)
        {
            return &(pBTLtp->gattServiceTable[i]);
        }
    }

    return NULL;
}


void * BTLTPLookupGATTServiceHandle(PBTLtp pBTLtp, uint8_t shortServiceHandle)
{
#if ACI_EN
    if (shortServiceHandle < BT_GATT_SERVER_MAX_SERVICES_COUNT)
    {
        if (pBTLtp->gattServiceTable[shortServiceHandle].isUsed)
        {
            return pBTLtp->gattServiceTable[shortServiceHandle].serviceHandle;
        }
        else
        {
            return NULL;
        }
    }
#else
    if (shortServiceHandle < BT_GATT_SERVER_MAX_SERVICES_COUNT)
    {
        return pBTLtp->gattServiceHandle[shortServiceHandle];
    }
#endif
    return NULL;
}

PGattServiceTable BTACILookupGATTServiceTableByHostService(PBTLtp pBTLtp, uint32_t hostServiceHandle)
{
    uint16_t i;
    for (i = 0; i < BT_GATT_SERVER_MAX_SERVICES_COUNT; i++)
    {
        if (pBTLtp->gattServiceTable[i].isUsed)
        {
            if (pBTLtp->gattServiceTable[i].host_service == hostServiceHandle)
            {
                return &(pBTLtp->gattServiceTable[i]);
            }
        }
    }

    return NULL;
}

uint8_t BTLTPAllocateFindGATTServiceHandle(PBTLtp pBTLtp, void *serviceHandle)
{
    uint8_t i;
    uint8_t firstUnused = BT_GATT_SERVER_MAX_SERVICES_COUNT;

    for (i = 0; i < BT_GATT_SERVER_MAX_SERVICES_COUNT; i++)
    {
        if (pBTLtp->gattServiceHandle[i] == serviceHandle)
        {
            return i;
        }
        else if ((pBTLtp->gattServiceHandle[i] == NULL) &&
                 (firstUnused == BT_GATT_SERVER_MAX_SERVICES_COUNT)
                )
        {
            firstUnused = i;
        }
    }

    if (firstUnused != BT_GATT_SERVER_MAX_SERVICES_COUNT)
    {
        pBTLtp->gattServiceHandle[firstUnused] = serviceHandle;
    }
    
    return firstUnused;
}


#if 0
uint32_t fs_save_ServiceTcb_struct(ServiceTcb *pdata)
{
    return fs_save_vendor_data(pdata, sizeof(ServiceTcb), 0);
}

uint32_t fs_load_ServiceTcb_struct(ServiceTcb *pdata)
{
    return fs_load_vendor_data(pdata, sizeof(ServiceTcb), 0);
}

uint32_t fs_save_ServiceTable_struct(ServiceTable *pdata, uint16_t offset)
{
    return fs_save_vendor_data(pdata, sizeof(ServiceTable), offset);
}

uint32_t fs_load_ServiceTable_struct(ServiceTable *pdata, uint16_t offset)
{
    return fs_load_vendor_data(pdata, sizeof(ServiceTable), offset);
}
#endif
BOOL fs_save_New_ServiceTable(PGattServiceTable pdata)
{
	#if 0
    ServiceTable ServiceSave;
    if (ServiceStore.service_num == BT_GATT_SERVER_MAX_SERVICES_COUNT)
    {
        DBG_BUFFER(MODULE_LTP, LEVEL_ERROR,  "fs_save_New_ServiceTable can not add service\n", 0);
        return FALSE;
    }
    if (ServiceStore.database_offset + pdata->database_length > ACI_SERVICE_DATABASE_END_OFFSET)
    {
        DBG_BUFFER(MODULE_LTP, LEVEL_ERROR,  "fs_save_New_ServiceTable no flash space database_offset = %d database_length = %d\n", 2, ServiceStore.database_offset, pdata->database_length);
        return FALSE;
    }
    ServiceSave.database_length = pdata->database_length;
    ServiceSave.host_service = pdata->host_service;
    ServiceSave.start_offset = ServiceStore.database_offset;
    fs_save_vendor_data(pdata->pService, ServiceSave.database_length, ServiceSave.start_offset);
    fs_save_ServiceTable_struct(&ServiceSave, ServiceStore.service_offset);
    ServiceStore.service_num++;
    ServiceStore.service_offset += sizeof(ServiceTable);
    ServiceStore.database_offset += ServiceSave.database_length;
    fs_save_ServiceTcb_struct(&ServiceStore);
	#endif
    return TRUE;
}

BOOL fs_clear_all_ServiceTable(PBTLtp pBTLtp)
{
    uint16_t size = sizeof(TGattServiceTable) * BT_GATT_SERVER_MAX_SERVICES_COUNT;
	#if 0
    if (ServiceStore.service_num == 0)
        return TRUE;
  #endif
    DataBasePos = 0;
    memset(pBTLtp->gattServiceTable, 0 , size);
    for (uint16_t i = 0; i < BT_GATT_SERVER_MAX_SERVICES_COUNT; i++)
    {
        P_BtLtp->gattServiceTable[i].self_idx = i;
    }
		#if 0
    ServiceStore.service_offset = ACI_SERVICE_TABLE_START_OFFSET;
    ServiceStore.database_offset = ACI_SERVICE_DATABASE_START_OFFSET;
    ServiceStore.service_num = 0;
    fs_save_ServiceTcb_struct(&ServiceStore);
		#endif
    return TRUE;
}

void fs_init(PBTLtp pBTLtp)
{
	#if 0
    ServiceTable ServiceSave;
    PGattServiceTable pGattService;
    uint16_t idx;
    uint16_t offset;
    fs_load_ServiceTcb_struct(&ServiceStore);
    if (ServiceStore.service_offset == 0xffff)
    {
        DBG_BUFFER(MODULE_LTP, LEVEL_ERROR,  "fs_init not init flash\n", 0);
        ServiceStore.service_offset = ACI_SERVICE_TABLE_START_OFFSET;
        ServiceStore.database_offset = ACI_SERVICE_DATABASE_START_OFFSET;
        ServiceStore.service_num = 0;
        fs_save_ServiceTcb_struct(&ServiceStore);
    }
    DBG_BUFFER(MODULE_LTP, LEVEL_INFO,  "fs_init service_num = %d\n", 1, ServiceStore.service_num);
    if (ServiceStore.service_num != 0)
    {
        for (idx = 0; idx < ServiceStore.service_num; idx++)
        {
            offset = ACI_SERVICE_TABLE_START_OFFSET + idx * sizeof(ServiceTable);
            fs_load_ServiceTable_struct(&ServiceSave, offset);
            pGattService = BTACIAllocateGATTServiceHandle(pBTLtp);
            if (pGattService == NULL)
            {
                DBG_BUFFER(MODULE_LTP, LEVEL_ERROR,  "fs_init pGattService == NULL idx = %d\n", 1, idx);
            }
            else
            {
                pGattService->pService = ACI_GetBuffer(ServiceSave.database_length);
                if (pGattService->pService == NULL)
                {
                    DBG_BUFFER(MODULE_LTP, LEVEL_ERROR,  "fs_init allocate failed len = %d\n", 1, ServiceSave.database_length);
                    return;
                }
                else
                {
                    pGattService->isUsed = TRUE;
                    pGattService->host_service = ServiceSave.host_service;
                    pGattService->database_length = ServiceSave.database_length;
                    pGattService->nbrOfAttrib = ServiceSave.database_length / sizeof(TAttribAppl);
                    fs_load_vendor_data(pGattService->pService, ServiceSave.database_length, ServiceSave.start_offset);
                }
            }
        }

    }
		#endif
    return;
}

