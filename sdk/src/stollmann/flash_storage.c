/**
*********************************************************************************************************
*               Copyright(c) 2014, Realtek Semiconductor Corporation. All rights reserved.
**********************************************************************************************************
* @file     flash_storage.c
* @brief    Store data to EEPROM source file.
* @details  Interfaces to Store data to EEPROM.
* @author   ethan_su
* @date     2015-6-30
* @version  v0.1
*********************************************************************************************************
*/
#include "FreeRTOS.h"
//#include "task.h"
#include "queue.h"
//#include "stm32f10x.h"
#include "flash_storage.h"
#include "rtl_types.h"
//#include "log_print.h"
#include <string.h>

typedef struct
{
    remote_BD_struct block_rem_bd_struct;
    LTK_struct block_ltk_struct;
    remLTK_struct block_rem_ltk_struct;
    IRK_struct block_irk_struct;
    Local_name_struct block_loc_name_struct;
    cccData_struct block_ccc_data_struct;
} Ram_Data_Block;

/* define gRamDataBlock here to temp store bonding info. Here will occupy a block of ram. */
static Ram_Data_Block gRamDataBlock;

uint32_t fs_load_remote_BD_struct(remote_BD_struct* pdata)
{
    //return m24c64_Read((void *)pdata,sizeof(remote_BD_struct),USR_REMOTE_BD_ADDR);
    memcpy((void *)pdata,(void *)&gRamDataBlock.block_rem_bd_struct,sizeof(remote_BD_struct));
    return TRUE;
}

uint32_t fs_save_remote_BD_struct(remote_BD_struct* pdata)
{
    //return m24c64_Write((void *)pdata,sizeof(remote_BD_struct),USR_REMOTE_BD_ADDR);
    memcpy((void *)&gRamDataBlock.block_rem_bd_struct,(void *)pdata,sizeof(remote_BD_struct));
    return TRUE;
}

uint32_t fs_load_LTK_struct(LTK_struct* pdata)
{
    //return m24c64_Read((void *)pdata,sizeof(LTK_struct),USR_LTK_ADDR);
    memcpy((void *)pdata,(void *)&gRamDataBlock.block_ltk_struct,sizeof(LTK_struct));
    return TRUE;
}

uint32_t fs_save_LTK_struct(LTK_struct* pdata)
{
    //return m24c64_Write((void *)pdata,sizeof(LTK_struct),USR_LTK_ADDR);
    memcpy((void *)&gRamDataBlock.block_ltk_struct,(void *)pdata,sizeof(LTK_struct));
    return TRUE;
}

uint32_t fs_load_remLTK_struct(remLTK_struct* pdata)
{
    //return m24c64_Read((void *)pdata,sizeof(remLTK_struct),USR_REM_LTK_ADDR);
    memcpy((void *)pdata,(void *)&gRamDataBlock.block_rem_ltk_struct,sizeof(remLTK_struct));
    return TRUE;
}

uint32_t fs_save_remLTK_struct(remLTK_struct* pdata)
{
    //return m24c64_Write((void *)pdata,sizeof(remLTK_struct),USR_REM_LTK_ADDR);
    memcpy((void *)&gRamDataBlock.block_rem_ltk_struct,(void *)pdata,sizeof(remLTK_struct));
    return TRUE;
}

uint32_t fs_load_IRK_struct(IRK_struct *pdata)
{
    //return m24c64_Read((void *)pdata,sizeof(IRK_struct),USR_IRK_ADDR);
    memcpy((void *)pdata,(void *)&gRamDataBlock.block_irk_struct,sizeof(IRK_struct));
    return TRUE;
}

uint32_t fs_save_IRK_struct(IRK_struct *pdata)
{
    //return m24c64_Write((void *)pdata,sizeof(IRK_struct),USR_IRK_ADDR);
    memcpy((void *)&gRamDataBlock.block_irk_struct,(void *)pdata,sizeof(IRK_struct));
    return TRUE;
}

uint32_t fs_load_Local_name_struct(Local_name_struct *pdata)
{
    //return m24c64_Read((void *)pdata,sizeof(Local_name_struct),USR_LOCAL_NAME_ADDR);
    memcpy((void *)pdata,(void *)&gRamDataBlock.block_loc_name_struct,sizeof(Local_name_struct));
    return TRUE;
}

uint32_t fs_save_Local_name_struct(Local_name_struct *pdata)
{
    //return m24c64_Write((void *)pdata,sizeof(Local_name_struct),USR_LOCAL_NAME_ADDR);
    memcpy((void *)&gRamDataBlock.block_loc_name_struct,(void *)pdata,sizeof(Local_name_struct));
    return TRUE;
}

uint32_t fs_load_cccData_struct(cccData_struct *pdata)
{
    //return m24c64_Read((void *)pdata,sizeof(cccData_struct),USR_CCC_DATA_ADDR);
    memcpy((void *)pdata,(void *)&gRamDataBlock.block_ccc_data_struct,sizeof(cccData_struct));
    return TRUE;
}

uint32_t fs_save_cccData_struct(cccData_struct *pdata)
{
    //return m24c64_Write((void *)pdata,sizeof(cccData_struct),USR_CCC_DATA_ADDR);
    memcpy((void *)&gRamDataBlock.block_ccc_data_struct,(void *)pdata,sizeof(cccData_struct));
    return TRUE;
}

#if 0
uint32_t fs_load_testData1_struct(test_data_struct *pdata)
{
    //return m24c64_Read((void *)pdata,sizeof(test_data_struct),USR_TEST_DATA1_ADDR);
    return TRUE;
}

uint32_t fs_save_testData1_struct(test_data_struct *pdata)
{
    //return m24c64_Write((void *)pdata,sizeof(test_data_struct),USR_TEST_DATA1_ADDR);
    return TRUE;
}

uint32_t fs_load_testData2_struct(test_data_struct *pdata)
{
    //return m24c64_Read((void *)pdata,sizeof(test_data_struct),USR_TEST_DATA2_ADDR);
    return TRUE;
}

uint32_t fs_save_testData2_struct(test_data_struct *pdata)
{
    //return m24c64_Write((void *)pdata,sizeof(test_data_struct),USR_TEST_DATA2_ADDR);
    return TRUE;
}


void m24c64_StorageTest()
{
    test_data_struct temp_data;
    int i;
    uint32_t errno;
    errno = fs_load_testData1_struct(&temp_data);
    if(temp_data.test_data[0] > 0)
    {
        m24c64_Delay();
        gattdCmdPrint( NULL, ">> TEST_INFO   Data stored in test page 1: <<\r\n>>> TEST_INFO");
        for(i = 0; i < 8; i++)
            gattdCmdPrint( NULL, "   %d",temp_data.test_data[i]);
        gattdCmdPrint( NULL, "<<\r\n>>> TEST_INFO   Store data to test page 2. <<\r\n>");
        //memcpy((void *)&temp_data,(void *)&test_full_0_data,sizeof(test_data_struct));
        errno = fs_save_testData1_struct(&test_full_0_data);
        m24c64_Delay();
        errno = fs_save_testData2_struct(&test_full_1_data);
    }
    else
    {
        m24c64_Delay();
        gattdCmdPrint( NULL, ">> TEST_INFO   Data stored in test page 2: <<\r\n>>> TEST_INFO");
        errno = fs_load_testData2_struct(&temp_data);
        m24c64_Delay();
        for(i = 0; i < 8; i++)
            gattdCmdPrint( NULL, "   %d",temp_data.test_data[i]);
        gattdCmdPrint( NULL, "<<\r\n>>> TEST_INFO   Store data to test page 1. <<\r\n>");
        //memcpy((void *)&temp_data,(void *)&test_full_0_data,sizeof(test_data_struct));
        errno = fs_save_testData1_struct(&test_full_1_data);
        m24c64_Delay();
        errno = fs_save_testData2_struct(&test_full_0_data);
    }
    gattdCmdPrint( NULL, ">> TEST_INFO   errno = %d <<\r\n>",errno);
    return;
}

void staticBufferTest()
{
    uint8_t full_0_data[8] = {0};
    uint8_t full_1_data[8] = {1,1,1,1,1,1,1,1};
    uint8_t i;
    if(static_buffer[0] > 0)
    {
        gattdCmdPrint( NULL, ">> TEST_INFO   Data stored in test page >0: <<\r\n>>> TEST_INFO");
        for(i = 0; i < 8; i++)
            gattdCmdPrint( NULL, "   %d",static_buffer[i]);
        gattdCmdPrint( NULL, "<<\r\n>>> TEST_INFO   Store data to test page 2. <<\r\n>");
        //memcpy((void *)&temp_data,(void *)&test_full_0_data,sizeof(test_data_struct));
        memcpy(static_buffer,full_0_data,8);
    }
    else
    {
        gattdCmdPrint( NULL, ">> TEST_INFO   Data stored in test page =0: <<\r\n>>> TEST_INFO");
        for(i = 0; i < 8; i++)
            gattdCmdPrint( NULL, "   %d",static_buffer[i]);
        gattdCmdPrint( NULL, "<<\r\n>>> TEST_INFO   Store data to test page 1. <<\r\n>");
        //memcpy((void *)&temp_data,(void *)&test_full_0_data,sizeof(test_data_struct));
        memcpy(static_buffer,full_1_data,8);
    }
    return;
}
#endif

