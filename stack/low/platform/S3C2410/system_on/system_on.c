/***************************************************************************
 Copyright (C) RealTek Ltd.
 This module is a confidential and proprietary property of RealTek and
 a possession or use of this module requires written permission of RealTek.
 ***************************************************************************/

enum { __FILE_NUM__= 217 };

#include "system_on.h"
#include "platform.h"
#include "power_control.h"
#include "UartPrintf.h"
#include "logger.h"
#include "DataType.h"
#include "new_io.h"

#ifdef _SUPPORT_INFO_FROM_SYSTEM_ON_

UINT8 g_chip_id = 0;
UINT8 g_chip_ver = 0;

UINT8 getHwId(void)
{
    /* avoid hw false alarm */
#if defined(_RTL8703B_SPECIFIC_)
    return CHIP_ID_8703B;
#elif defined(_RTL8822B_SPECIFIC_)
    return CHIP_ID_8822B;
#elif defined(_RTL8723D_SPECIFIC_)
    return CHIP_ID_8723D;
#elif defined(_RTL8821C_SPECIFIC_)
    return CHIP_ID_8821C;
#else
    return 0;
#endif

#if 0
    if(CHECK_CHIP_ID_BY_EFUSE)
    {
        return (otp_str_data.efuse_btongpio_init_d8)>>4;
    }
    else
    {
        return RD_8BIT_COMBO_SYSON_IO(PAGE0_REG_HW_ID);
    }
#endif    
}

UINT8 getchipver(void)
{
#if defined(_RTL8703B_SPECIFIC_) || defined(_RTL8822B_SPECIFIC_)
   return (RD_8BIT_COMBO_SYSON_IO(PAGE0_REG_CHIP_VER)&0xF0)>>4;
#endif

#if defined(_RTL8723D_SPECIFIC_) || defined(_RTL8821C_SPECIFIC_)
   return ((RD_8BIT_SYSON_IO(PAGE0_REG_CHIP_VER))&0xF0)>>4;
#endif

}


UINT8 getfinalhci(void)
{

#if defined(_RTL8703B_SPECIFIC_) || defined(_RTL8822B_SPECIFIC_)
    //UINT8 u8hci;
    if(g_chip_id == CHIP_ID_8703B)
    {
        // 8703b and before
        // 2bits hci
        
        PAGE0_REG_0xF4_BYTE_READ_S reg_hci;

        reg_hci.d8= RD_8BIT_COMBO_SYSON_IO(PAGE0_REG_8703B_FINAL_HCI_SEL);

        return reg_hci.b.hci_selection_2bit;
        
    }
    else
    {
        // 8822b
        // 3bits hci
        
        PAGE0_REG_0xF5_BYTE_READ_S reg_hci;

        reg_hci.d8 = RD_8BIT_COMBO_SYSON_IO(PAGE0_REG_8822B_FINAL_HCI_SEL);

        return reg_hci.b.hci_selection_3bit;
    }
#endif

#if defined(_RTL8723D_SPECIFIC_) || defined(_RTL8821C_SPECIFIC_)
    //if(g_chip_id == CHIP_ID_8723D)
    {
        // 8703b and before
        // 2bits hci
        
        PAGE0_REG_0xF4_BYTE_READ_S reg_hci;

        reg_hci.d8= RD_8BIT_SYSON_IO(PAGE0_REG_8822B_FINAL_HCI_SEL);

        return reg_hci.b.hci_selection_2bit;
        
    }
#endif
#if 0
#if defined(_RTL8821C_SPECIFIC_)
    {
        // 8822b, 8821c
        // 3bits hci
        
        PAGE0_REG_0xF5_BYTE_READ_S reg_hci;

        reg_hci.d8 = RD_8BIT_SYSON_IO(PAGE0_REG_8822B_FINAL_HCI_SEL);

        return reg_hci.b.hci_selection_3bit;
    }
#endif
#endif
    
    //return u8hci;
}

#if 0
void update_lmp_version()
{
    switch (g_chip_id)
    {
        case CHIP_ID_8703B:
            fw_lmp_sub_version = 0x8703;
            fw_hci_sub_version = 0x000B;
            break;
        case CHIP_ID_8822B:
            fw_lmp_sub_version = 0x8822;
            fw_hci_sub_version = 0x000B;
            break;
        default:
            fw_lmp_sub_version = 0xBEEF;
            fw_hci_sub_version = 0xDEAD; 
            break;
    }
}
#endif
#endif


