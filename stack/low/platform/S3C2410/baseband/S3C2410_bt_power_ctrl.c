/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/
enum { __FILE_NUM__= 70 };

#ifdef POWER_SAVE_FEATURE
#include "S3C2410_bt_power_ctrl.h"
#include "power_control.h"

#ifdef _YL_LPS
#else
#define  write_power_control_register(val) \
  *((volatile UINT16*)(POWER_CTRL)) = (UINT16) val
static UCHAR power_control = BB_CLK_EN | RF_CLK_EN;
#endif

#if 0
// TODO: need to set exit_dsm = 1??
void S3C2410_init_power_control(void)
{
#ifdef _YL_LPS
    BT_SYS_CTRL_REG_S_TYPE bt_sys_reg;
    bt_sys_reg.d32 = VENDOR_READ(BT_SYS_CTRL_REG);
    bt_sys_reg.b.external_bluewiz_clk_gat =1;
    bt_sys_reg.b.cpu_40mhz_en =1;    
    VENDOR_WRITE(BT_SYS_CTRL_REG, bt_sys_reg.d32);    
#else
    power_control = 
                    BB_CLK_EN | 
                    RF_CLK_EN;  

    write_power_control_register(power_control);
#endif    
}
#endif

void S3C2410_set_dsm_exit(void)
{
#ifdef _YL_LPS
    BT_SYS_CTRL_REG_S_TYPE bt_sys_reg;
    bt_sys_reg.d32 = VENDOR_READ(BT_SYS_CTRL_REG);
    bt_sys_reg.b.exist_deep_sleep_mode =1;
    VENDOR_WRITE(BT_SYS_CTRL_REG, bt_sys_reg.d32);    
#else
    power_control |= DSM_EXIT;  
    write_power_control_register(power_control);
#endif    
}

void S3C2410_clear_dsm_exit(void)
{
#ifdef _YL_LPS
    BT_SYS_CTRL_REG_S_TYPE bt_sys_reg;
    bt_sys_reg.d32 = VENDOR_READ(BT_SYS_CTRL_REG);
    bt_sys_reg.b.exist_deep_sleep_mode =0;
    VENDOR_WRITE(BT_SYS_CTRL_REG, bt_sys_reg.d32);    
#else
    power_control &= (~DSM_EXIT);   
    write_power_control_register(power_control);
#endif    
}

#if 0
void S3C2410_set_bb_clk_en(void)
{
#ifdef _YL_LPS
    BT_SYS_CTRL_REG_S_TYPE bt_sys_reg;
    bt_sys_reg.d32 = VENDOR_READ(BT_SYS_CTRL_REG);
    bt_sys_reg.b.external_bluewiz_clk_gat =1;
    VENDOR_WRITE(BT_SYS_CTRL_REG, bt_sys_reg.d32);    
#else
    power_control |= BB_CLK_EN;     
    write_power_control_register(power_control);
#endif    
}

void S3C2410_clear_bb_clk_en(void)
{
#ifdef _YL_LPS
    BT_SYS_CTRL_REG_S_TYPE bt_sys_reg;
    bt_sys_reg.d32 = VENDOR_READ(BT_SYS_CTRL_REG);
    bt_sys_reg.b.external_bluewiz_clk_gat =0;
    VENDOR_WRITE(BT_SYS_CTRL_REG, bt_sys_reg.d32);    
#else
    power_control &= (~BB_CLK_EN);  
    write_power_control_register(power_control);
#endif    
}

void S3C2410_set_rf_clk_en(void)
{
#ifdef _YL_LPS
    BT_SYS_CTRL_REG_S_TYPE bt_sys_reg;
    bt_sys_reg.d32 = VENDOR_READ(BT_SYS_CTRL_REG);
    bt_sys_reg.b.cpu_40mhz_en =1;
    VENDOR_WRITE(BT_SYS_CTRL_REG, bt_sys_reg.d32);    
#else
    power_control |= RF_CLK_EN;     
    write_power_control_register(power_control);
#endif    
}

void S3C2410_clear_rf_clk_en(void)
{
#ifdef _YL_LPS
    BT_SYS_CTRL_REG_S_TYPE bt_sys_reg;
    bt_sys_reg.d32 = VENDOR_READ(BT_SYS_CTRL_REG);
    bt_sys_reg.b.cpu_40mhz_en =0;
    VENDOR_WRITE(BT_SYS_CTRL_REG, bt_sys_reg.d32);    
#else
    power_control &= (~RF_CLK_EN);  
    write_power_control_register(power_control);
#endif    
}
#endif

#endif //#ifdef POWER_SAVE_FEATURE

