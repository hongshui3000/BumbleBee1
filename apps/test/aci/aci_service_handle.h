/**
*********************************************************************************************************
*               Copyright(c) 2014, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file      aci_service_handle.h
* @brief     low power handle when using ACI.
* @details   none.
* @author    tifnan
* @date      2014-11-19
* @version   v0.1
* *********************************************************************************************************
*/

#ifndef     _ACI_SERVICE_HANDLE_H_
#define     _ACI_SERVICE_HANDLE_H_

#define ACI_SERVICE_DATABASE_START    1
#define ACI_SERVICE_DATABASE_CONTINUE 2
#define ACI_SERVICE_DATABASE_END      3

#define ACI_SERVICE_TABLE_START_OFFSET     8
#define ACI_SERVICE_DATABASE_START_OFFSET  104
#define ACI_SERVICE_DATABASE_END_OFFSET   2048
#define ACI_SERVICE_DATABASE_MAX_SIZE  (ACI_SERVICE_DATABASE_END_OFFSET - ACI_SERVICE_DATABASE_START_OFFSET)

typedef struct
{
    uint16_t service_offset;
    uint16_t database_offset;
    uint8_t service_num;
    uint8_t padding[3];
} ServiceTcb;

typedef struct
{
    uint32_t host_service;
    uint16_t database_length;
    uint16_t start_offset;
} ServiceTable;
typedef ServiceTable * PServiceTable;

typedef enum
{
    Load_CauseSuccess,
    Load_CauseServiceExist,
    Load_CauseServiceNotExist,
    Load_CauseServiceTooMuch,
    Load_CauseServiceNoResource,
    Load_CauseLengthError,
    Load_CauseInvalidPDU,
} LoadDatabaseCause;

void * ACI_GetBuffer(uint16_t length);
BOOL ACI_FreeBuffer(void * pBuffer, uint16_t length);
void fs_init(PBTLtp pBTLtp);
BOOL fs_save_New_ServiceTable(PGattServiceTable pdata);
BOOL fs_clear_all_ServiceTable(PBTLtp pBTLtp);

PGattServiceTable    BTACIAllocateGATTServiceHandle(PBTLtp pBTLtp);
uint8_t                 BTACIFindGATTServiceHandle(PBTLtp pBTLtp, void *serviceHandle);
PGattServiceTable    BTACILookupGATTServiceTableByHostService(PBTLtp pBTLtp, uint32_t hostServiceHandle);
void *               BTLTPLookupGATTServiceHandle(PBTLtp pBTLtp, uint8_t shortServiceHandle);
uint8_t                 BTLTPAllocateFindGATTServiceHandle(PBTLtp pBTLtp, void *serviceHandle);

#endif
