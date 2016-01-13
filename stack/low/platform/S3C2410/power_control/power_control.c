/***************************************************************************
 Copyright (C) RealTek Ltd.
 This module is a confidential and proprietary property of RealTek and
 a possession or use of this module requires written permission of RealTek.
 ***************************************************************************/
 
enum { __FILE_NUM__= 122 };

#include "power_control.h"
#include "rlx4181.h"
#include "gpio.h"
#include "otp.h"
#include "hci_td.h"
#include "lc.h"
#include "h5.h"
#include "timer.h"
#include "dma_usb.h"
#include "le_ll.h"
#include "new_io.h"

#ifdef _SUPPORT_INFO_FROM_SYSTEM_ON_
#include "system_on.h"
extern UINT8 g_chip_id;
#endif


#ifdef _CCH_LPS_
extern UINT32 g_lps_timer_counter;
extern UINT32 g_timer2_init_value;
#endif

#if defined(_FONGPIN_TEST_USBCV_REMOTEWAKEUP_BY_GPIO0_)
extern UINT32 g_fongpin_test_timer_counter;
#endif

#if defined(_SUPPORT_INFO_FROM_SYSTEM_ON_) && defined(_SUPPORT_BT_CTRL_FM_)
extern UINT8 g_u8support_fm;
#endif


#ifdef _ENABLE_BTON_POWER_SAVING_

UINT8 isr_switch = 0;
BTON_POW_CTRL_REG_S_TYPE pow_ctrl_int_rec;
extern CHIP_FUN_AND_INTERFACE_S_TYPE g_fun_interface_info;
MIPS_CAUSE_REG_S cpu_filter;
TimerHandle_t hci_wake_up_tid_timer = NULL;
HOST_SLEEP_PKT_MANAGER_S_TYPE host_sleep_pkt_man;

#ifdef _ROM_CODE_PATCHED_
//For rom code patch function point
PF_ROM_CODE_PATCH_FUNC rcp_execute_lps_mode_procedure = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_pow_ctrl_intr_handle = NULL;
PF_ROM_CODE_PATCH_VOID rcp_enter_lps_mode = NULL;
PF_ROM_CODE_PATCH_VOID rcp_power_init = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_execute_wakeup_procedure = NULL;
PF_ROM_CODE_PATCH_VOID rcp_wake_up_host = NULL;
#ifndef _8821A_BTON_DESIGN_       
PF_ROM_CODE_PATCH_FUNC rcp_trun_on_off_afe_ldo = NULL;
#else
PF_ROM_CODE_PATCH_FUNC rcp_execute_bton_entering_pdn_sus = NULL;
#endif

PF_ROM_CODE_PATCH_FUNC rcp_execute_lps_mode_procedure_6128_func = NULL; 
PF_ROM_CODE_PATCH_FUNC rcp_enter_lps_mode_6128_func = NULL; 
PF_ROM_CODE_PATCH_FUNC rcp_pow_ctrl_intr_handle_end_func = NULL;
#ifdef _YL_RTL8723A_B_CUT
#ifdef _8821A_BTON_DESIGN_
PF_ROM_CODE_PATCH_FUNC rcp_enable_lps_mode_setting_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_disable_lps_mode_setting_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_bton_clr_wakeup_sts_func = NULL;
#else
PF_ROM_CODE_PATCH_VOID rcp_enable_lps_mode_setting_func = NULL;
#endif
#endif
#ifdef _CCH_8723_RCP_
PF_ROM_CODE_PATCH_FUNC rcp_pow_timer_lps = NULL;
#endif
#ifdef _CCH_8821_RCP_
PF_ROM_CODE_PATCH_VOID rcp_bton_32k_cal_en = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_bton_32k_cal_chk_lock = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_lps_handle_timer2_mode_sel1 = NULL;
#endif
#ifdef _CCH_2801_RCP_
PF_ROM_CODE_PATCH_VOID rcp_bton_32k_cal_ini = NULL;
#endif
#endif

/**
 * Function     : rlx4081_isr_sw_unmask
 *
 * Description  : This function is used to set 4081 isr switch for sw
 *
 * Parameters  : None
 *
 * Returns      : None
 *
 * Side Effects : None
 *
 */
void rlx4081_isr_sw_unmask()
{
    *(UINT32*)&cpu_filter |= CPU_FILTER_MASK;
}

/**
 * Function     : rlx4081_isr_sw_mask
 *
 * Description  : This function is used to set 4081 isr switch for sw
 *
 * Parameters  : None
 *
 * Returns      : None
 *
 * Side Effects : None
 *
 */
void rlx4081_isr_sw_mask()
{
    *(UINT32*)&cpu_filter &= ~CPU_FILTER_MASK;
}


/**
 * Function     : enable_lps_mode_setting
 *
 * Description  : This function initialises wake up signal in LPS mode
 *
 * Parameters  : None
 *
 * Returns      : None
 *
 * Side Effects : None
 *
 */
void enable_lps_mode_setting()
{
    BTON_POW_CTRL_REG_S_TYPE pow_ctrl;

#ifndef _8821A_BTON_DESIGN_
#ifdef _ROM_CODE_PATCHED_
#ifdef _YL_RTL8723A_B_CUT
    /* yilinli
     *    1. set pow_ctrl.b.gps_gpio_wakeup_en = 1 when gpio_host_wake_bt is enabled
     *
     */
    if (rcp_enable_lps_mode_setting_func != NULL)
    {
        rcp_enable_lps_mode_setting_func();
        return;
    }
#endif    
#endif    
#endif
    pow_ctrl.d32 = VENDOR_READ(BTON_POW_CTRL_REG);
    
    pow_ctrl.b.mailbox_wakeup_en = 1;

#ifdef LPS_NO_WAKEUP_WHEN_NO_SCAN
#ifdef _MODI_LPS_AFTER_RTL8821B_TC_
    if(( lc_check_lps_for_idle(0) == TRUE )&&(g_efuse_lps_setting_2.lps_nowakeup_when_no_scan))
#else
    if( lc_check_lps_for_idle(0) == TRUE )
#endif
    {
        pow_ctrl.b.bluewiz_wakeup_en = 0;
    }
    else
#endif	
    {
        pow_ctrl.b.bluewiz_wakeup_en = 1;
    }

    pow_ctrl.b.cal_timer_intr_en = 1;

    switch (g_fun_interface_info.b.hci_sel)
    {
#ifndef _SUPPORT_3BITS_HCI_SELECTION_
#ifndef _SUPPORT_2BIT_HCI_AND_PCIE_UART_
        case RTK8723_S:
#else
        case PACKAGE_SDIO_UART:
        case PACKAGE_PCIE_UART:
#endif
#else
        case PACKAGE_SDIO_UART:
        case PACKAGE_PCIE_UART:
#ifdef _RTL8822B_SPECIFIC_            
        case PACKAGE_MPCIE_UART:
#endif            
#endif
            pow_ctrl.b.uart0_wakeup_en = 1;
            pow_ctrl.b.usb_sus_intr_en = 0;
#ifndef _REMOVE_HCI_PCIE_
            pow_ctrl.b.pcie_sus_intr_en = 0;
#endif
#if defined(_SUPPORT_INFO_FROM_SYSTEM_ON_) && defined(_SUPPORT_BT_CTRL_FM_)
            if(g_u8support_fm == 1)
#endif                
            {
#if defined(_SUPPORT_BT_CTRL_FM_) && defined(_REMOVE_HCI_PCIE_)
                pow_ctrl.b.fm_wake_bt_int_en = 1;
#endif
            }
            pow_ctrl.b.usb_wakeup_en = 0;
            break;
#ifndef _SUPPORT_3BITS_HCI_SELECTION_
#ifndef _SUPPORT_2BIT_HCI_AND_PCIE_UART_
        case RTK8723_U:
#else
        case PACKAGE_USB_USB:
#endif
#else
        case PACKAGE_USB_USB:
#endif
            pow_ctrl.b.uart0_wakeup_en = 0;
            pow_ctrl.b.usb_sus_intr_en = 1;
#ifndef _REMOVE_HCI_PCIE_            
            pow_ctrl.b.pcie_sus_intr_en = 0;
#endif
#if defined(_SUPPORT_INFO_FROM_SYSTEM_ON_) && defined(_SUPPORT_BT_CTRL_FM_)
            if(g_u8support_fm == 1)
#endif                
            {

#if defined(_SUPPORT_BT_CTRL_FM_) && defined(_REMOVE_HCI_PCIE_)
                pow_ctrl.b.fm_wake_bt_int_en = 0;
#endif
            }

            pow_ctrl.b.usb_wakeup_en = 1;           
            break;
#ifndef _REMOVE_HCI_PCIE_            
        case RTK8723_EE:
            pow_ctrl.b.uart0_wakeup_en = 0;
            pow_ctrl.b.usb_sus_intr_en = 0;
            pow_ctrl.b.pcie_sus_intr_en = 1;
            pow_ctrl.b.usb_wakeup_en = 0;
            break;
#endif



/*
#ifdef _RTL8822B_SPECIFIC_ 
        case RTK8822B_ME:
#endif
*/


#ifndef _SUPPORT_3BITS_HCI_SELECTION_
#ifndef _SUPPORT_2BIT_HCI_AND_PCIE_UART_
        case RTK8723_E:
#else
        case PACKAGE_PCIE_USB: 
#endif
#else
#ifdef _RTL8822B_SPECIFIC_
        case PACKAGE_MPCIE_USB:
#endif            
        case PACKAGE_PCIE_USB: 
#endif
            pow_ctrl.b.uart0_wakeup_en = 0;
            pow_ctrl.b.usb_sus_intr_en = g_fun_interface_info.b.bt_fun_sts;
#ifdef _8821A_BTON_DESIGN_            
#ifndef _REMOVE_HCI_PCIE_            
            pow_ctrl.b.pcie_sus_intr_en = 0;
#endif
            //pow_ctrl.b.fm_wake_bt_int_en = 0;


#else
            pow_ctrl.b.pcie_sus_intr_en =  g_fun_interface_info.b.gps_fun_sts;
#endif            
            pow_ctrl.b.usb_wakeup_en = g_fun_interface_info.b.bt_fun_sts;
            break;
        default:
            break;
    }

#ifndef _8821A_BTON_DESIGN_     
    pow_ctrl.b.gps_gpio_wakeup_en = g_fun_interface_info.b.gps_fun_sts;
#endif    
    pow_ctrl.b.bt_gpio_wakeup_en = g_fun_interface_info.b.bt_fun_sts;


#ifdef _FONGPIN_TEST_MUX_DEV_WAKEUP_HOST_BY_GPIO13_
#ifdef _FONGPIN_TEST_MUX_DEV_WAKEUP_HOST_BY_GPIO13_LPS_
    pow_ctrl.b.bt_gpio13_wk_wakeup_en = 1;
#endif
#endif    
#ifdef _8821A_BTON_DESIGN_
#ifdef _ROM_CODE_PATCHED_
#ifdef _YL_RTL8723A_B_CUT
    /* yilinli
     *    1. set pow_ctrl.b.gps_gpio_wakeup_en = 1 when gpio_host_wake_bt is enabled
     *    2. replace pow_ctrl.b.bt_gpio_wakeup_en = g_fun_interface_info.b.bt_fun_sts;
     *       to
     *       BTON_INTERFACE_CTRL_REG_S_TYPE bton_0x44;
     *       bton_0x44.d32= VENDOR_READ(BTON_INTERFACE_CTRL_REG);
     *       pow_ctrl.b.bt_gpio_wakeup_en = bton_0x44.b.gpio_hw_pdn_sts;
     *    3. Reg_WiFi_OnOff_intr 0x40[26], Reg_BT_GPIO_WK 0x40[4], Reg_cal_timer_intr 
     *       Reg_cal_tmeter_intr, wakeup by CTS toggle ...
     */
    if (rcp_enable_lps_mode_setting_func != NULL)
    {
        if (rcp_enable_lps_mode_setting_func((void *)&pow_ctrl.d32))
        {
            return;
        }
    }
#endif    
#endif
#endif

    pow_ctrl_int_rec.d32 = pow_ctrl.d32;

    POW_DBG_LOG(BLUE, POWER_CTRL_REG_VALUE, 1, pow_ctrl.d32);    

    bton_pow_ctrl_avoid_int_sts_w1c(&pow_ctrl);

    VENDOR_WRITE(BTON_POW_CTRL_REG,pow_ctrl.d32);
}

/**
 * Function     : disable_lps_mode_setting
 *
 * Description  : This function is used to disable wake up signal in LPS mode
 *
 * Parameters  : None
 *
 * Returns      : None
 *
 * Side Effects : None
 *
 */
void disable_lps_mode_setting()
{
    BTON_POW_CTRL_REG_S_TYPE pow_ctrl;

    pow_ctrl.d32 = VENDOR_READ(BTON_POW_CTRL_REG);
    pow_ctrl.b.mailbox_wakeup_en = 0;
    pow_ctrl.b.bluewiz_wakeup_en = 0;
    pow_ctrl.b.uart0_wakeup_en = 0;
    pow_ctrl.b.usb_wakeup_en = 0;
#ifndef _8821A_BTON_DESIGN_    
    pow_ctrl.b.gps_gpio_wakeup_en = 0;
#endif    
    pow_ctrl.b.bt_gpio_wakeup_en = 0;
    pow_ctrl.b.cal_timer_intr_en = 0;
    pow_ctrl.b.usb_sus_intr_en = 0;
#ifdef _YL_LPS_POW_CTRL_INTR_GPIO13_WAKEUP_PROCESS_     
    pow_ctrl.b.bt_gpio13_wk_wakeup_en = 0;
#endif
#if defined(_SUPPORT_INFO_FROM_SYSTEM_ON_) && defined(_SUPPORT_BT_CTRL_FM_) 
    if(g_u8support_fm == 1)
#endif        
    {

#ifdef _SUPPORT_BT_CTRL_FM_             
        pow_ctrl.b.fm_wake_bt_int_en = 0;
#endif
    }

#ifdef _8821A_BTON_DESIGN_
#ifdef _ROM_CODE_PATCHED_
#ifdef _YL_RTL8723A_B_CUT
    /* yilinli
     *    1. set pow_ctrl.b.gps_gpio_wakeup_en = 1 when gpio_host_wake_bt is enabled
     *    2. Reg_WiFi_OnOff_intr 0x40[26], Reg_BT_GPIO_WK 0x40[4], Reg_cal_timer_intr 
     *       Reg_cal_tmeter_intr, wakeup by CTS toggle ? ...
     */
    if (rcp_disable_lps_mode_setting_func != NULL)
    {
        if (rcp_disable_lps_mode_setting_func((void *)&pow_ctrl.d32))
        {
            return;
        }
    }
#endif    
#endif
#endif

    pow_ctrl_int_rec.d32 = pow_ctrl.d32;

#ifdef _8821A_BTON_DESIGN_
    pow_ctrl.b.wifi_onoff_intr_sts = 0; /* to avoid falsely W1C wifi_onoff_intr_sts */
#endif    
    VENDOR_WRITE(BTON_POW_CTRL_REG,pow_ctrl.d32);
#ifdef _FONGPIN_TEST_MUX_DEV_WAKEUP_HOST_BY_GPIO13_LPS_
    RT_BT_LOG(GREEN, FONGPIN_TEST_MUX_LEAVE_LPS_BY_GPIO13, 0,0);
#endif
}


#if !defined(_MOVE_TO_PATCH_) && !defined(SPI_FLASH_BOOT) // TODO:  to be modified/implemented in PATCH
/**
 * Function     : enable_32k_timer_setting
 *
 * Description  : This function is used to set 32k timer
 *
 * Parameters  : None
 *
 * Returns      : None
 *
 * Side Effects : None
 *
 */
void enable_32k_timer_setting(UINT8 is_on)
{
    BTON_POW_CTRL_REG_S_TYPE pow_ctrl;

    pow_ctrl.d32 = VENDOR_READ(BTON_POW_CTRL_REG);
    
    pow_ctrl.b.cal_timer_intr_en = is_on;

    pow_ctrl_int_rec.d32 = pow_ctrl.d32;

    POW_DBG_LOG(BLUE, POWER_CTRL_REG_VALUE, 1, pow_ctrl.d32);    

    VENDOR_WRITE(BTON_POW_CTRL_REG,pow_ctrl.d32);
}
#endif

/**
 * Function     : enable_uart_suspend
 *
 * Description  : This function is used to handle uart suspend event
 *
 * Parameters  : None
 *
 * Returns      : None
 *
 * Side Effects : None
 *
 */
void enable_uart_suspend()
    {
    BTON_POW_CTRL_REG_S_TYPE pow_ctrl;
    POW_OPTION_AND_32K_CTRL_S_TYPE pow_option;

    pow_option.d32 = VENDOR_READ(BTON_POW_OPTION_AND_32K_CTRL);
    pow_option.b.uart0_sus_en = 1;

    pow_ctrl.d32 = VENDOR_READ(BTON_POW_CTRL_REG);
    pow_ctrl.b.uart0_resume_en = 1;
    pow_ctrl_int_rec.d32 = pow_ctrl.d32;

//    POW_DBG_LOG(BLUE, POWER_CTRL_REG_VALUE, 1, pow_ctrl.d32);    

    VENDOR_WRITE(BTON_POW_CTRL_REG,pow_ctrl.d32);
    VENDOR_WRITE(BTON_POW_OPTION_AND_32K_CTRL,pow_option.d32);

}

#ifdef _8821A_BTON_DESIGN_
/* this functioin executes necessary procedures/settings before
 * entering BTON "OFF Mode": LDO_DIS */
 #if 0
void execute_bton_entering_pdn_sus(UINT8 option)
{

#ifdef _ROM_CODE_PATCHED_
    if (rcp_execute_bton_entering_pdn_sus != NULL)
    {
        if (rcp_execute_bton_entering_pdn_sus((void *)&option))
            return;
    }
#endif

    switch (option)
    {
        case BTON_ENTERING_PDN:    
            bton_set_pcm_pull_low(PCM_PULL_OPTION_PDN_SUS_ST);
            break;
        case BTON_ENTERING_SUS:
            bton_set_pcm_pull_low(PCM_PULL_OPTION_PDN_SUS_ST);
            break;
        default:
            break;  
    }
}
#else
void execute_bton_entering_pdn_sus(BTON_POW_CTRL_REG_S_TYPE *pow_ctrl_ptr)
{
    BTON_POW_CTRL_REG_S_TYPE pow_ctrl = *pow_ctrl_ptr;
    BTON_UART_INFO_REG_S_TYPE bton_uart_info;
    EFUSE_POW_SETTING_2_S efuse_pow_setting_2;
    *(UINT8*)&efuse_pow_setting_2 = otp_str_data.efuse_pow_setting_2_d8;
    EFUSE_POW_SETTING_3_S efuse_pow_setting_3;
    *(UINT8*)&efuse_pow_setting_3 = otp_str_data.efuse_pow_setting_3_d8;
    GENERAL_CONTROL_S_TYPE bt_general_ctrl;
    bt_general_ctrl.d16 = otp_str_data.general_control;
//    UINT8 exe_pdn_flag = 0;
//    UINT8 exe_sus_flag = 0;
    
    
#ifdef _ROM_CODE_PATCHED_
    if (rcp_execute_bton_entering_pdn_sus != NULL)
    {
        if (rcp_execute_bton_entering_pdn_sus((void *)pow_ctrl_ptr))
        {
           return;
        }
    }
#endif

#ifdef _YL_IGNORE_UNNECESSARY_ENTERING_PDN_SUS_
    if ( (pow_ctrl_ptr->b.bt_sus_en == 0) && (pow_ctrl_ptr->b.bt_hwpdn_en == 0) )
    {
        return;
    }
#endif

    bton_uart_info.d32 = VENDOR_READ(BTON_UART_INFO_REG);
    bton_uart_info.b.bton_pre_boot_state = 0;  
    if ((pow_ctrl.b.bt_hwpdn_en))
    {
//        exe_pdn_flag = 1;
        if (efuse_pow_setting_2.avoid_pdn_with_sus) /* not required by HW simulation */
        {
            pow_ctrl.b.bt_sus_en = 0;
        }                            
        bton_set_pcm_pull_low(PCM_PULL_OPTION_PDN_SUS_ST);
        bton_uart_info.b.bton_pre_boot_state |= BIT0;        
    }
    else if ((pow_ctrl.b.bt_sus_en))
    {
        bton_set_pcm_pull_low(PCM_PULL_OPTION_PDN_SUS_ST);
    }              
    
    if ((pow_ctrl.b.bt_sus_en))
    {
//        exe_sus_flag = 1;
        bton_uart_info.b.bton_pre_boot_state |= BIT1;        
    }
    
    if (efuse_pow_setting_3.record_bton_pre_boot_state && (bton_uart_info.b.bton_pre_boot_state!=0))
    {
        VENDOR_WRITE(BTON_UART_INFO_REG, bton_uart_info.d32);        
    }
    
#ifdef _YL_LPS_COMBO_PLATFORM
#ifdef LPS_MODIFY_POW_CTRL_W1C
    bton_pow_ctrl_avoid_int_sts_w1c(&(pow_ctrl));
#endif        
#endif


#if defined (_FIX_8703B_LED_SHARE_WITH_WIFI_)&&defined(_SUPPORT_INFO_FROM_SYSTEM_ON_)
        if(g_chip_id == CHIP_ID_8703B)
        {
            //bt_wifi_share_led_8703b_eco(g_wpan_led_num);
            bt_led_control(g_wpan_led_num, OUTPUT_HI_Z);
        }
#endif  


    VENDOR_WRITE(BTON_POW_CTRL_REG, pow_ctrl.d32); 

    if (bt_general_ctrl.b.pdn_sus_fa_recov_en && (pow_ctrl.b.bt_sus_en || pow_ctrl.b.bt_hwpdn_en))
    {
        pf_delay_us(5<<(bt_general_ctrl.b.pdn_sus_fa_recov_delay));
        
        RT_BT_LOG(RED, POW_LOG_001, 3, pow_ctrl.b.bt_hwpdn_en, pow_ctrl.b.bt_sus_en, 5<<(bt_general_ctrl.b.pdn_sus_fa_recov_delay));
        
        pow_ctrl.b.bt_hwpdn_en = 0;
        pow_ctrl.b.bt_sus_en = 0;      
        VENDOR_WRITE(BTON_POW_CTRL_REG, pow_ctrl.d32); 
        bton_set_pcm_pull_low(PCM_PULL_OPTION_ACT_ST);
    }

    
}
        
#endif
#endif

/**
 * Function     : pow_ctrl_intr_handle
 *
 * Description  : This function is used to handle power event
 *
 * Parameters  : None
 *
 * Returns      : None
 *
 * Side Effects : None
 *
 */
void pow_ctrl_intr_handle()
{
    BTON_POW_CTRL_REG_S_TYPE pow_ctrl;
    BT_SYS_CTRL_REG_S_TYPE bt_sys_reg;

    pow_ctrl.d32 = VENDOR_READ(BTON_POW_CTRL_REG);

#ifdef _ROM_CODE_PATCHED_
    /*  PATCH for
     *  e.g. 1. cal_fail_intr (0x5C, LPS-wakeup)
     *       2. wifi_onoff_intr_sts (0x40)
     *       3. bt_gpio13_wk_wakeup_sts (0x40, LPS-wakeup)
     */
    if (rcp_pow_ctrl_intr_handle != NULL)
    {
        if (rcp_pow_ctrl_intr_handle((void*)&pow_ctrl.d32))
        {
            return;
        }
    }
#endif

#ifdef _FONGPIN_TEST_MUX_DEV_WAKEUP_HOST_BY_GPIO13_    
    RT_BT_LOG(GREEN, FONGPIN_TEST_MUX_HOST_WAKEUP_BT_MUX_OK, 0,0);
    VENDOR_WRITE(BTON_POW_CTRL_REG,pow_ctrl.d32);
#endif
//    POW_DBG_LOG(BLUE, POWER_CTRL_REG_VALUE, 1, pow_ctrl.d32);
//    POW_DBG_LOG(YELLOW, POWER_CTRL_GLOBAL_REG_VALUE, 1, pow_ctrl_int_rec.d32);

    /* use short-circuit evaluation style to speed up fw process time - austin */

#ifdef _8821A_BTON_DESIGN_
    if ( (isr_switch == 0) || (g_efuse_lps_setting_4.lps_pow_ctrl_intr_check_isr_switch == 0) )
#endif
    {
        do 
        {
            UINT8 temp = isr_switch; /* save isr_switch */        
            isr_switch = 1;
            
            if (pow_ctrl_int_rec.b.mailbox_wakeup_en && pow_ctrl.b.mailbox_wakeup_sts)
            {            
                
                POW_DBG_LOG(BLUE, MAILBOX_WAKEUP_INT, 0, 0);
                break;
            }

            //handle bluewiz wake up instance
            if (pow_ctrl_int_rec.b.bluewiz_wakeup_en && pow_ctrl.b.bluewiz_wakeup_sts)
            {
                POW_DBG_LOG(BLUE, BLUEWIZ_WAKEUP_INT, 0, 0);
                break;
            }

            //handle uart0
            if (pow_ctrl_int_rec.b.uart0_wakeup_en && pow_ctrl.b.uart0_wakeup_sts)
            {
                POW_DBG_LOG(BLUE, UART0_WAKEUP_INT, 0, 0);
                break;
            }

            //handle usb
            if (pow_ctrl_int_rec.b.usb_wakeup_en && pow_ctrl.b.usb_wakeup_sts)
            {
                POW_DBG_LOG(BLUE, USB_WAKEUP_INT, 0, 0);
                break;
            }

            //handle thermeter interrupt
            if (pow_ctrl_int_rec.b.cal_thermeter_intr_en && pow_ctrl.b.cal_thermeter_intr_sts)
            {
                POW_DBG_LOG(BLUE, THERMETER_WAKEUP_INT, 0, 0);
                break;
            }

            //handle 32K timer interrupt
            if (pow_ctrl_int_rec.b.cal_timer_intr_en && pow_ctrl.b.cal_timer_intr_sts)
            {
                POW_DBG_LOG(BLUE, TIMER_32K_WAKEUP, 0, 0);
                break;
            }

            //handle bt gpio power down enable
            if (pow_ctrl_int_rec.b.bt_gpio_wakeup_en && pow_ctrl.b.bt_gpio_wakeup_sts)
            {
                POW_DBG_LOG(BLUE, EX_BT_GPIO_PDN_EN, 0, 0);
                break;
            }
#ifndef _8821A_BTON_DESIGN_
            //handle gps gpio power down enable
            if (pow_ctrl_int_rec.b.gps_gpio_wakeup_en && pow_ctrl.b.gps_gpio_wakeup_sts)
            {
                POW_DBG_LOG(BLUE, EX_GPS_GPIO_PDN_EN, 0, 0);
                break;
            }
#endif

#ifndef _REMOVE_HCI_PCIE_
            //handle pcie suspend enable
            if (pow_ctrl_int_rec.b.pcie_sus_intr_en && pow_ctrl.b.pcie_sus_intr_sts)
            {
                POW_DBG_LOG(BLUE, PCIE_SUSPEND_EN, 0, 0);
                break;
            }            
#endif
#if defined(_SUPPORT_INFO_FROM_SYSTEM_ON_) && defined(_SUPPORT_BT_CTRL_FM_) 
            if(g_u8support_fm == 1)
#endif                
            {

#if defined(_REMOVE_HCI_PCIE_) && defined(_SUPPORT_BT_CTRL_FM_)
                //handle fm suspend enable
                if (pow_ctrl_int_rec.b.fm_wake_bt_int_en && pow_ctrl.b.fm_wake_bt_int_sts)
                {
                    POW_DBG_LOG(BLUE, FM_WAKEUP_INT, 0, 0);
                    break;
                }
#endif
            }
            //handle usb suspend enable
            if (pow_ctrl_int_rec.b.usb_sus_intr_en && pow_ctrl.b.usb_sus_intr_sts)
            {
                POW_DBG_LOG(BLUE, USB_SUSPEND_EN, 0, 0);
                break;
            }

#ifdef _YL_LPS_POW_CTRL_INTR_GPIO13_WAKEUP_PROCESS_
            //handle gpio13 (host_wake_bt) enable
            if (pow_ctrl_int_rec.b.bt_gpio13_wk_wakeup_en && pow_ctrl.b.bt_gpio13_wk_wakeup_sts)
            {                            
                POW_DBG_LOG(BLUE, DMA_UART_202, 0, 0);
                break;
            }
#endif
            isr_switch = temp; /* no hit cases, restore isr_swotch */
        }
        while (0);
    }

#ifdef _8821A_BTON_DESIGN_
    if ( (isr_switch || (g_efuse_lps_setting_4.lps_isr_mask_check_isr_switch==0)) && 
          (pow_ctrl.d32 & 0x3FF00) )
#else    
    if (pow_ctrl.d32 & 0x3FF00) 
#endif
    {
#ifdef _CCH_LPS_
        if (pow_ctrl.d32 & ( (UINT32) g_efuse_lps_setting_3.timer2_lps_isr_mask << 8 ) ) 
        {
            g_lps_timer_counter = g_timer2_init_value;
        }	
#endif

        //Double check that lps_mode_en bit was cleared.
        pow_ctrl.b.lps_mode_en = 0;
//        POW_DBG_LOG(RED, OTHER_STATUS_APPEAR, 1, pow_ctrl.d32);
#ifdef _8821A_BTON_DESIGN_
        pow_ctrl.b.wifi_onoff_intr_sts = 0; /* to avoid falsely W1C wifi_onoff_intr_sts */
#endif        
        VENDOR_WRITE(BTON_POW_CTRL_REG,pow_ctrl.d32);

#ifdef _YL_LPS_1
        {
            UINT32 temp32;
            UCHAR log_en = 0;
        	
            if( pow_ctrl.b.lps_mode_en )
            {
        	    temp32 = pow_ctrl.d32;
                log_en = 1;	
        	}

            if( log_en )
            {
                UINT32 temp_native_clock; 

                temp_native_clock =  BB_read_native_clock();

                LPS_DBG_LOG(RED, LPS_LOG_030, 2, temp_native_clock, temp32);		
            }	
        }
#endif

    }

    //enable interrupt
    if (isr_switch)
    {
// if need to move disable_lps_mode_setting here
#ifdef _ROM_CODE_PATCHED_
        /* reserved rom code patch by yilinli for
          * e.g. 
          *    1. restore sleep_mode_param.bb_sm_sts if needed
          *    2. block some interrupt if needed (to avoid falsely access BB reg) then recovered by lc_handle_sm_intr()
          *        (by e.g. cpu_filter or cpu interrupt mask setting)
          *    3. better DEEP_SLEEP-to-ACTIVE transition protection 
          */
        if (rcp_pow_ctrl_intr_handle_end_func != NULL)
        {
            if(rcp_pow_ctrl_intr_handle_end_func((void *)&pow_ctrl.d32))
                return;
        }
#endif

#ifdef LPS_PROTECT_DSM_BB_ACCESS
        isr_switch = 0;
        cpu_filter.bluewiz_int = 1;
        cpu_filter.gpio_int = 1;
//        cpu_filter.timer_int = 1;  /* to be tested and confirmed */
//        cpu_filter.hci_dma_int = 1; /* to be tested and confirmed */
//        cpu_filter.bzdma_int = 1; /* to be tested and confirmed */
//        cpu_filter.uart_int = 1; /* to be tested and confirmed */
#endif    

        disable_lps_mode_setting();
#ifdef _ENABLE_32K_CLK_WAKE_UP_ISR_
        switch_hci_dma_setting(LEAVE_SUSPEND_MODE);
        wake_up_interrupt_switch(DISABLE_WAKE_UP_ISR);
#endif
        //swt cpu clock to 32KMHz
        bt_sys_reg.d32 = VENDOR_READ(BT_SYS_CTRL_REG);
        bt_sys_reg.b.cpu_40mhz_en = 1;
        bt_sys_reg.b.exist_deep_sleep_mode = 1;
        VENDOR_WRITE(BT_SYS_CTRL_REG, bt_sys_reg.d32);

#ifdef _UART_H5
        //If the controller is UART interface and the baudrate need UART PLL,
        //it need to enable PLL when the controller wake up from LPS mode.
        hci_uart_lps_clock_on_procedure();
#endif

#ifndef IS_BTSOC
#ifdef _ENABLE_BTON_POWER_SAVING_
        EnableIntForLPS();
#endif
#endif
        //Enable Uart PLL 

    }

}

/**
 * Function     : enter_lps_mode
 *
 * Description  : This function is used to enter LPS mode
 *
 * Parameters  : None
 *
 * Returns      : None
 *
 * Side Effects : None
 *
 */
void enter_lps_mode()
{
    BB_POWER_CONTROL_REG_S_TYPE bb_pow_ctrl;
    BTON_POW_CTRL_REG_S_TYPE pow_ctrl;
    BT_SYS_CTRL_REG_S_TYPE bt_sys_reg;
    //UINT32 ven_reg_0 = 0;
    UINT8 bluewiz_wakeup = 0;
    EFUSE_POW_PARA_S_TYPE efuse_pow_para;

#ifdef _ROM_CODE_PATCHED_
    if (rcp_enter_lps_mode!=NULL)
    {
        rcp_enter_lps_mode();
        return;
    }
#endif

    efuse_pow_para.d16 = otp_str_data.power_seq_param;

#ifndef _8821A_BTON_DESIGN_     
    //From efuse check if ldo need set to standy by mode
    if (efuse_pow_para.b.bt_core_ldo_standyby_en)
    {
        CORE_AND_AFE_LDO_CTRL_S_TYPE core_ctrl_reg;        
        core_ctrl_reg.d32 = VENDOR_READ(BTON_CORE_AFE_LDO_CTRL_REG);
        core_ctrl_reg.b.core_ldo_standby_en = 1;
        VENDOR_WRITE(BTON_CORE_AFE_LDO_CTRL_REG,core_ctrl_reg.d32);
    }
#endif
   // 1. No matter if fw set bluewiz wakeup time register or bluewiz power register, 
   //     it will wake up automatically. Except using bluewiz wake up, fw can't set
   //      these register
   //  2. If fw need to use bluewiz to wake up, it need to wait for one slot time.
   //      Reason: hw need to recount from native clock boundary.
    if (bluewiz_wakeup)
    {
        //set bluewiz control
        bb_pow_ctrl.d16 = BB_read_baseband_register(POWER_CONTROL_REGISTER);
        bb_pow_ctrl.b.enter_deep_sleep = 1;
        bb_pow_ctrl.b.lpo_freq_sel = 1;
        bb_pow_ctrl.b.osc_startup_delay = 0x8;
        BB_write_baseband_register(POWER_CONTROL_REGISTER,bb_pow_ctrl.d16);
        //delay one slot time
#ifndef FOR_SIMULATION
        pf_delay_us(625);
#endif
    }

    //swt cpu clock to 32KMHz
    bt_sys_reg.d32 = VENDOR_READ(BT_SYS_CTRL_REG);
    bt_sys_reg.b.cpu_40mhz_en = 0;
    bt_sys_reg.b.exist_deep_sleep_mode =0;
    VENDOR_WRITE(BT_SYS_CTRL_REG, bt_sys_reg.d32);

    //Wait 625us
#ifndef FOR_SIMULATION
    pf_delay_us(625);
#endif
    //set BTON LPS mode request
    pow_ctrl.d32 = VENDOR_READ(BTON_POW_CTRL_REG);
    pow_ctrl.b.lps_mode_en = 1;
    VENDOR_WRITE(BTON_POW_CTRL_REG,pow_ctrl.d32);

}


/**
 * Function     : execute_lps_mode_procedure
 *
 * Description  : This function is used to enter LPS mode
 *
 * Parameters  : 
 *
 * Returns      : None
 *
 * Side Effects : None
 *
 */
void execute_lps_mode_procedure(UINT8 bz_wake, UINT16 bz_wake_time)
{
    UINT16 wakeup_instance = 0;
    UINT32 cur_clock =0;

#ifdef _ROM_CODE_PATCHED_
    if (rcp_execute_lps_mode_procedure != NULL)
    {
        if (rcp_execute_lps_mode_procedure((void *)&bz_wake, bz_wake_time))
        {
            return;
        }
    }
#endif

    DEF_CRITICAL_SECTION_STORAGE;

    // TODO: place MINT_OS_ENTER_CRITICAL(); here or ???
    //When Uart is h5, fw need to send sleep message to host.
#ifdef _UART_H5
    //If Uart is H5, controller need to send sleep message before executing lps mode.
    hci_uart_h5_go_lps();
    //If the interface is UART, the controller need to check if it need disable PLL.
    hci_uart_lps_clock_off_procedure();
#endif

    if (bz_wake)
    {
        cur_clock = (INT32)BB_read_baseband_register(NATIVE_CLOCK2_REGISTER);
        cur_clock = cur_clock << 16;
        cur_clock |= (INT32)BB_read_baseband_register(NATIVE_CLOCK1_REGISTER);

        wakeup_instance = ((cur_clock>>1) + bz_wake_time) & 0xFFFF;  
    }
    // TODO: wakeup_instance = bz_wake_time
    
    //change hci dma buffer size to zero, the purpose is that
    //usb SIE can nak host traffic.
    switch_hci_dma_setting(ENTER_SUSPEND_MODE);
    wake_up_interrupt_switch(ENABLE_WAKE_UP_ISR);


    MINT_OS_ENTER_CRITICAL();
    //sw mask
    rlx4081_isr_sw_mask();

    //Disable all 4081 interrupt; except hci dma interrupt 
#ifndef IS_BTSOC
    DisableIntForLPS();
#endif
    MINT_OS_EXIT_CRITICAL();

    //set leave LPS mode parameter
    enable_lps_mode_setting();

    //set bz wake up time
    if (bz_wake)
    {
        BB_write_baseband_register(WAKEUP_INSTANT_REGISTER,wakeup_instance);
    }
    enter_lps_mode();

}

#ifdef _YL_LPS
void enter_lps_mode_6128(UINT8 bluewiz_wakeup)
{
    BB_POWER_CONTROL_REG_S_TYPE bb_pow_ctrl;
    BTON_POW_CTRL_REG_S_TYPE pow_ctrl;
    BT_SYS_CTRL_REG_S_TYPE bt_sys_reg;
    //UINT32 ven_reg_0 = 0;
//    UINT8 bluewiz_wakeup = 0;
    EFUSE_POW_PARA_S_TYPE efuse_pow_para;
#ifdef LPS_ABORT_AFTER_ENTER_LPS    
    THERMAL_METER_CTRL_S_TYPE thermal_meter_ctrl_reg;
    UINT8 ii; 
#endif

#ifdef _YL_LPS    
    LPS_DBG_LOG(YELLOW, LPS_LOG_009, 1, bluewiz_wakeup);
#endif

#ifdef _ROM_CODE_PATCHED_
    /* Reserved com code patch by yilinli:
      * e.g. 
      * protect USB suspend status !!!
      */
#ifdef _YL_RTL8723A_B_CUT
    // * protect USB suspend status !!!
    // * protect GPIO missed transition !!!
    // * should not w1c wakeup interrupt status (0x40): avoided by write MASK to BTON_POW_CTRL_REG
    // * if(USB & !suspend)
    //        r_LOP_EXTL = 1;
    // * options: 
    //        if (g_host_wake_bt)
    //           giveup enter LPS
    //
#endif      

    if (rcp_enter_lps_mode_6128_func != NULL)
    {
        if ( rcp_enter_lps_mode_6128_func((void*)(&bluewiz_wakeup) ))
        {
            return;
        }
    }
#endif

    efuse_pow_para.d16 = otp_str_data.power_seq_param;

    //From efuse check if ldo need set to standy by mode
#ifndef _8821A_BTON_DESIGN_     
#ifdef _YL_LPS_COMBO_PLATFORM 
    if (efuse_pow_para.b.bt_core_ldo_standyby_en)
    {
        CORE_AND_AFE_LDO_CTRL_S_TYPE core_ctrl_reg;
        core_ctrl_reg.d32 = VENDOR_READ(BTON_CORE_AFE_LDO_CTRL_REG);
        core_ctrl_reg.b.core_ldo_standby_en = 1;
        VENDOR_WRITE(BTON_CORE_AFE_LDO_CTRL_REG,core_ctrl_reg.d32);
    }
#endif
#endif
   // 1. No matter if fw set bluewiz wakeup time register or bluewiz power register, 
   //     it will wake up automatically. Except using bluewiz wake up, fw can't set
   //      these register
   //  2. If fw need to use bluewiz to wake up, it need to wait for one slot time.
   //      Reason: hw need to recount from native clock boundary.

#ifdef _YL_LPS
    // TODO: check with Chris/Chinwen this new code
    bt_sys_reg.d32 = VENDOR_READ(BT_SYS_CTRL_REG);
    bt_sys_reg.b.exist_deep_sleep_mode =0;
    bt_sys_reg.b.cpu_40mhz_en =1;
    VENDOR_WRITE(BT_SYS_CTRL_REG, bt_sys_reg.d32);
#endif

    if (bluewiz_wakeup)
    {
    //set bluewiz control
        bb_pow_ctrl.d16 = BB_read_baseband_register(POWER_CONTROL_REGISTER);
        bb_pow_ctrl.b.enter_deep_sleep = 1;
#ifdef _YL_LPS        
#else
        bb_pow_ctrl.b.lpo_freq_sel = 1;
        bb_pow_ctrl.b.osc_startup_delay = 0x8;
#endif

        BB_write_baseband_register(POWER_CONTROL_REGISTER,bb_pow_ctrl.d16);

        lps_gpio_one_pull_high(LPS_GPIO_DSM_PERIOD);

        // TODO: /need add function to delay one slot time
#ifdef _YL_LPS        

#ifndef FOR_SIMULATION
        /* For BB: to sync slot boundary for switching between clock sources */
        pf_delay_us(625);
#else
        UINT32 us = 625;
        UINT32 index;
        UINT32 max = us * 10;
        for(index = 0x0; index < max; index++);
#endif

#endif
    }

    //swt cpu clock to 32KMHz
    bt_sys_reg.d32 = VENDOR_READ(BT_SYS_CTRL_REG);
#ifdef _YL_LPS_COMBO_PLATFORM    
    bt_sys_reg.b.cpu_40mhz_en = 0; /* shall NOT be used in COMBO PF: BTON wakeup interrupt will fail to be triggered */
#else
  #ifdef _YL_LPS_DSM_KEEP_CUP_40MHZ
    bt_sys_reg.b.cpu_40mhz_en = 1; 
  #else
    bt_sys_reg.b.cpu_40mhz_en = 0;
  #endif    
#endif  
    bt_sys_reg.b.exist_deep_sleep_mode =0;
    VENDOR_WRITE(BT_SYS_CTRL_REG, bt_sys_reg.d32);
    
#ifdef _YL_LPS // TODO: ?   Note: CPU clock may be 32K?
    // TODO: Wait 625us    (REQUIRED?);  Double-check with Chinwen/Chris/Andre
    if(g_efuse_lps_setting_1.delay625us_before_bton_lps_req)
    {
#ifndef FOR_SIMULATION
        /* For BB: to sync slot boundary for switching between clock sources */
        pf_delay_us(625);
#else
        UINT32 us = 625;
        UINT32 index;
        UINT32 max = us * 10;
        for(index = 0x0; index < max; index++);
#endif
    }
#else
        pf_delay_us(625);
#endif


#ifdef _YL_RTL8723A_B_CUT
    /*
     *   // TODO:
     *   some delay
     *   read bt usb suspend_b and protect undetected change
     *   if (bt usb suspend && g_host_state == 0)
     *   {
     *       write pow_ctrl.d32.b.lps_mode_en = 0;
     *   }
     *   read gpio_host_wake_bt and protect undetected change
     */   
#endif

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_	
    if(!sleep_mode_param.run_dlps_flow)
#endif
    {
#ifdef _YL_LPS_COMBO_PLATFORM    
    //set BTON LPS mode request
    pow_ctrl.d32 = VENDOR_READ(BTON_POW_CTRL_REG);
#ifdef LPS_MODIFY_POW_CTRL_W1C
    bton_pow_ctrl_avoid_int_sts_w1c(&(pow_ctrl));
#endif
    pow_ctrl.b.lps_mode_en = 1;
    
#ifdef _8821A_BTON_DESIGN_
    pow_ctrl.b.wifi_onoff_intr_sts = 0; /* to avoid falsely W1C wifi_onoff_intr_sts */
#endif        
    VENDOR_WRITE(BTON_POW_CTRL_REG,pow_ctrl.d32);
#endif    
    }

#ifdef LPS_ABORT_AFTER_ENTER_LPS // TODO: (yl) to be reviewed and modified
    *** to be reviewed and modified ***
    /* CCH/yilinli, post-protection to avoid falsely entering LPS states */
    if(lc_lps_check_abort_lps_cond() == API_FAILURE)
    {
        /* abort LPS procedures: */
        /* wait for BTON state-transition occurs, then exist_deep_sleep_mode can be used */
        /* bt_sys_reg.b.exist_deep_sleep_mode = 1 (or pow_ctrl.b.lps_mode_en = 0 ?)*/
        for (ii = 0; ii<5; ii++)
        {
            thermal_meter_ctrl_reg.d32 = VENDOR_READ(BTON_THERMAL_METER_CTRL_REG);
            if ( thermal_meter_ctrl_reg.b.cur_power_state == 6)
            {
                break;
            }
        }   
        bt_sys_reg.b.exist_deep_sleep_mode =1;
        VENDOR_WRITE(BT_SYS_CTRL_REG, bt_sys_reg.d32);
    }
#endif

}

/* Mask the W1C interrupt statuses to avoid falsely clear them */
/* NOTE: wifi_onoff_intr_sts W1C should be avoided during initialization
 *       Else, wifi_onoff_intr_sts status initialization should be executed 
 *       after FW initialization finishs */
void bton_pow_ctrl_avoid_int_sts_w1c(BTON_POW_CTRL_REG_S_TYPE *pow_ctrl)
{
    // TODO: (yl) To be reviewed and modified for RTL8821A
    /* mask LPS wakeup interrupt status */
    (*pow_ctrl).b.mailbox_wakeup_sts = 0; 
    (*pow_ctrl).b.bluewiz_wakeup_sts = 0; 
    (*pow_ctrl).b.uart0_wakeup_sts = 0; 
    (*pow_ctrl).b.usb_wakeup_sts = 0; 
    (*pow_ctrl).b.bt_gpio13_wk_wakeup_sts = 0; 
    (*pow_ctrl).b.bt_gpio_wakeup_sts = 0; 
    (*pow_ctrl).b.cal_thermeter_intr_sts = 0; 
    (*pow_ctrl).b.cal_timer_intr_sts = 0; 
#ifndef _REMOVE_HCI_PCIE_
    (*pow_ctrl).b.pcie_sus_intr_sts = 0; 
#endif
#if defined(_SUPPORT_INFO_FROM_SYSTEM_ON_) && defined(_SUPPORT_BT_CTRL_FM_)
    if(g_u8support_fm == 1)
#endif        
    {
#if defined(_REMOVE_HCI_PCIE_) && defined(_SUPPORT_BT_CTRL_FM_)
        (*pow_ctrl).b.fm_wake_bt_int_sts = 0;
#endif
    }
    (*pow_ctrl).b.usb_sus_intr_sts = 0;    
    /* mask St_WiFi_OnOff_intr interrupt status */
    (*pow_ctrl).b.wifi_onoff_intr_sts = 0;     
}

#ifdef _8821A_BTON_DESIGN_
void bton_32k_ctrl_avoid_int_sts_w1c(UINT32 *bton_32k_ctrl_d32_ptr)
{

#ifdef _NEW_BTON_DESIGN_AFTER_RTL8821B_TC_
    BTON_5C_REG_S_TYPE bton_5c;
    bton_5c.d32 = *bton_32k_ctrl_d32_ptr;
    bton_5c.st_cal_fail_intr = 0;
    *bton_32k_ctrl_d32_ptr = bton_5c.d32;
#else
    // TODO: (yl) To be reviewed and modified for RTL8821A
    BTON_32K_CTRL_REG_S_TYPE bton_32k_ctrl;
    bton_32k_ctrl.d32 = *bton_32k_ctrl_d32_ptr;
    bton_32k_ctrl.st_cal_fail_intr = 0;
    *bton_32k_ctrl_d32_ptr = bton_32k_ctrl.d32;
#endif	
}
#endif

void bton_clr_wakeup_sts(void)
{
 
#ifdef _ROM_CODE_PATCHED_
    /***************************************************
     * Patch reserved by yilinli, possible usages are:
     *  1. Also clear wifi_onoff_intr_sts by setting pow_ctrl.b.wifi_onoff_intr_sts = 1
     *  2. Reg_WiFi_OnOff_intr 0x40[26], Reg_BT_GPIO_WK 0x40[4], Reg_cal_timer_intr 
     *     Reg_cal_tmeter_intr, (wakeup by CTS toggle?) ...
     ***************************************************/
    if (rcp_bton_clr_wakeup_sts_func != NULL)
    {
        if(rcp_bton_clr_wakeup_sts_func(NULL))
        {
            return;
        }
    }
#endif
    // TODO: (yl) To be reviewed and modified for RTL8821A
    /* enter critical to avoid interrupted (e.g. GpioIntr) */
    DEF_CRITICAL_SECTION_STORAGE;
    MINT_OS_ENTER_CRITICAL();

    BTON_POW_CTRL_REG_S_TYPE pow_ctrl;
    pow_ctrl.d32 = VENDOR_READ(BTON_POW_CTRL_REG);    
#ifdef _8821A_BTON_DESIGN_    
    pow_ctrl.b.wifi_onoff_intr_sts = 0; /* to avoid falsely clear the status */
#endif
    VENDOR_WRITE(BTON_POW_CTRL_REG,pow_ctrl.d32); 

#ifdef _8821A_BTON_DESIGN_    

#ifdef _NEW_BTON_DESIGN_AFTER_RTL8821B_TC_
    BTON_5C_REG_S_TYPE bton_5c;
    bton_5c.d32 = VENDOR_READ(BTON_5C_REG);
    VENDOR_WRITE(BTON_5C_REG, bton_5c.d32);   
#else
    BTON_32K_CTRL_REG_S_TYPE bton_32k_ctrl;
    bton_32k_ctrl.d32 = VENDOR_READ(BTON_32K_CTRL_REG);
    VENDOR_WRITE(BTON_32K_CTRL_REG,bton_32k_ctrl.d32); 
#endif	
#endif 

    MINT_OS_EXIT_CRITICAL();
}

/**
 * Function     : execute_lps_mode_procedure
 *
 * Description  : This function is used to enter LPS mode
 *
 * Parameters  : 
 *
 * Returns      : None
 *
 * Side Effects : None
 *
 */
void execute_lps_mode_procedure_6128(UINT8 bz_wake, UINT16 bz_wake_time)
{
    UINT16 wakeup_instance = 0;
    UINT32 cur_clock =0;


    DEF_CRITICAL_SECTION_STORAGE;

#ifdef _ROM_CODE_PATCHED_
    /* Reserved com code patch by yilinli:
      * e.g. 
      * 1. when bz_wake = 0: set sleep_mode_param.bb_sm_sts = XTAL_OFF_MODE
      */
    if (rcp_execute_lps_mode_procedure_6128_func != NULL)
    {
        if ( rcp_execute_lps_mode_procedure_6128_func((void*)(&bz_wake), &bz_wake_time) )
        {
            return;
        }
    }
#endif
    
#ifdef _MODEM_LNA_CONSTRAINT_
    lc_set_modem_lna_constraint_on_off(0);
#endif

#ifdef _UART_H5
    //If Uart is H5, controller need to send sleep message before executing lps mode.
    hci_uart_h5_go_lps(); 
    //If the interface is UART, the controller need to check if it need disable PLL.
    hci_uart_lps_clock_off_procedure();
#endif

#ifdef _YL_LPS
    if(bz_wake)
    {
        wakeup_instance = bz_wake_time;
    }
    else
    {
        /* 2^16*625us = 40s */
        // TODO: is this ok?
        // TODO: how to sleep as long as possible?


        cur_clock = (INT32)BB_read_baseband_register(NATIVE_CLOCK2_REGISTER);
        cur_clock = cur_clock << 16;
        cur_clock |= (INT32)BB_read_baseband_register(NATIVE_CLOCK1_REGISTER);
        wakeup_instance = ((cur_clock>>1) + 0xFF00) & 0xFFFF;  
    }
#else    
    if (bz_wake)
    {
        cur_clock = (INT32)BB_read_baseband_register(NATIVE_CLOCK2_REGISTER);
        cur_clock = cur_clock << 16;
        cur_clock |= (INT32)BB_read_baseband_register(NATIVE_CLOCK1_REGISTER);

        wakeup_instance = ((cur_clock>>1) + bz_wake_time) & 0xFFFF;  
    }
#endif

    LPS_DBG_LOG(YELLOW, LPS_LOG_008, 2, bz_wake, wakeup_instance);

    //change hci dma buffer size to zero, the purpose is that
    //usb SIE can nak host traffic.
#ifdef _YL_LPS_COMBO_PLATFORM
    switch_hci_dma_setting(ENTER_SUSPEND_MODE); // TODO: need this? Would Cause Problems???? Add EFUSE

#ifdef _8821A_BTON_DESIGN_
    if (g_efuse_lps_setting_2.lps_enable_hci_dma_wakeup_isr) /* 0 is recommended */
#endif
    {
        wake_up_interrupt_switch(ENABLE_WAKE_UP_ISR);
    }

    MINT_OS_ENTER_CRITICAL();

    //sw mask
    rlx4081_isr_sw_mask(); // TODO: need this, Would cause Problems ??? Add EFUSE
    //Disable all 4081 interrupt; except hci dma interrupt
#ifndef IS_BTSOC
    DisableIntForLPS();
#endif
    MINT_OS_EXIT_CRITICAL();
    //set leave LPS mode parameter
    enable_lps_mode_setting();
#else
//    wake_up_interrupt_switch(DISABLE_WAKE_UP_ISR); // For FPGA Test only To Override BTON
#endif



    //set bz wake up time
//    if (bz_wake) // cch modify 20110625
        BB_write_baseband_register(WAKEUP_INSTANT_REGISTER,wakeup_instance);

    enter_lps_mode_6128(bz_wake);

}


#ifndef _YL_LPS_COMBO_PLATFORM
// TODO: To be reviewed
// TODO: To be modified for COMBO(8723)
void exit_lps_mode_6128(void)
{
//    BB_POWER_CONTROL_REG_S_TYPE bb_pow_ctrl;
    BT_SYS_CTRL_REG_S_TYPE bt_sys_reg;

    // TODO: Other functions???
    //swt cpu clock to 32KMHz
    bt_sys_reg.d32 = VENDOR_READ(BT_SYS_CTRL_REG);
    bt_sys_reg.b.cpu_40mhz_en = 1;
    bt_sys_reg.b.exist_deep_sleep_mode =1;
    VENDOR_WRITE(BT_SYS_CTRL_REG, bt_sys_reg.d32);
//    lps_gpio_one_pull_low(2);
    
//    EnableIntForLPS();
//    switch_hci_dma_setting(ENTER_SUSPEND_MODE);
//    wake_up_interrupt_switch(ENABLE_WAKE_UP_ISR);
}

// TODO: !!!!! Which place"s" to execute this function?
// TODO: Exit SM Interrupt?  pow_ctrl_interrupt()?  others???
void execute_exit_lps_mode_procedure_6128(void)
{
    // TODO: 
    exit_lps_mode_6128();
#ifdef _YL_LPS_COMBO_PLATFORM    
    EnableIntForLPS(); // TODO: need this?
    wake_up_interrupt_switch(DISABLE_WAKE_UP_ISR); // TODO: need this?
    switch_hci_dma_setting(LEAVE_SUSPEND_MODE); // TODO: need this?
#endif    
}
#endif

#endif

//===========================================================
#ifdef _YL_LPS
#ifdef _YL_LPS_TEST
UINT8 g_test_gpio_state=0;
void lps_test_at_keep_alive_event(void)
{
    LPS_DBG_LOG(WHITE, LPS_LOG_012, 6, 
                            sleep_mode_param.bb_sm_sts, 
                            sleep_mode_param.bb_sm_wakeup_inst, 
                            sleep_mode_param.bb_sm_piconet_id,
                            BB_read_baseband_register(INTERRUPT_REGISTER),
                            VENDOR_READ(0x00),
                            VENDOR_READ(0x40));
    LPS_DBG_LOG(WHITE, LPS_LOG_014, 5,
                            lmp_self_device_data.scan_enable,
                            lmp_self_device_data.total_no_of_sco_conn, 
                            lmp_self_device_data.number_of_esco_connections,
                            lmp_self_device_data.number_of_hlc, 
                            lmp_self_device_data.number_of_parked_dev);                     
                            
    #ifdef _YL_LPS_TEST_GPIO0_TOGGLE_WHEN_ALIVE
            if(g_test_gpio_state)
                lps_gpio_one_pull_high(0);
            else
                lps_gpio_one_pull_low(0);
            g_test_gpio_state=!g_test_gpio_state;
    #endif
#ifndef _REDUCE_LPS_AFTER_RTL8703B_	
    #ifdef _YL_LPS_TEST_ENTER_LPS_WHEN_ALIVE    
            lc_post_dsm_mode_signal_test(200, 0);    
    #endif
#endif	
    #ifdef _YL_LPS_TEST_TOGGLE_CPU_CLK_WHEN_ALIVE    
            BT_SYS_CTRL_REG_S_TYPE bt_sys_reg;            
            bt_sys_reg.d32 = VENDOR_READ(BT_SYS_CTRL_REG);
            if(g_test_gpio_state)
            {
                lps_gpio_one_pull_high(0);
                bt_sys_reg.b.cpu_40mhz_en = 1;                
            }
            else
            {
                lps_gpio_one_pull_low(0);
                bt_sys_reg.b.cpu_40mhz_en = 0;
            }
            g_test_gpio_state=!g_test_gpio_state;
            VENDOR_WRITE(BT_SYS_CTRL_REG, bt_sys_reg.d32);            
    #endif
    #ifdef _YL_LPS_TEST_FORCE_CPU_32K_WHEN_ALIVE    
            BT_SYS_CTRL_REG_S_TYPE bt_sys_reg;            
            bt_sys_reg.d32 = VENDOR_READ(BT_SYS_CTRL_REG);
            bt_sys_reg.b.cpu_40mhz_en = 0;                
            VENDOR_WRITE(BT_SYS_CTRL_REG, bt_sys_reg.d32);            
    #endif
}
#endif
#endif
//==================================================================


#ifdef LPS_LOP_SETTING
void lop_sop_low_power_setting_init(void)
{
    POW_OPTION_AND_32K_CTRL_S_TYPE pow_option;
    pow_option.d32 = VENDOR_READ(BTON_POW_OPTION_AND_32K_CTRL);  

    /* UART Setting */
    pow_option.b.bandgap_mbias_sus_en = 0;
    pow_option.b.xtal_sus_en = 0;
    pow_option.b.efuse_power_clk_sus_en = 0;
    pow_option.b.swr_2_pwm_sus_en = 0;
    pow_option.b.swr_sus_en = 0; /* to be reviewed */

    pow_option.b.bandgap_mbias_lps_en = 0;
    pow_option.b.xtal_lps_en = 0;
    pow_option.b.efuse_power_clk_lps_en = 0;
    pow_option.b.swr_2_pwm_lps_en = 0;
    pow_option.b.swr_lps_en = 0; /* to be reviewed */

#if 1 // TODO: (yl) To be confirmed
    pow_option.b.efuse_power_clk_lps_en = 1;
#endif
    if (g_fun_interface_info.b.bt_interface == USB_INTERFACE)
    {
        pow_option.b.swr_sus_en = 1;
        
        pow_option.b.bandgap_mbias_lps_en = 1;
        pow_option.b.xtal_lps_en = 1;
        //pow_option.b.efuse_power_clk_lps_en = 1; ??
        pow_option.b.swr_lps_en = 1; 
    } 
#ifndef _REMOVE_HCI_PCIE_     
    if (g_fun_interface_info.b.bt_interface == PCIE_INTERFACE)
    {
        pow_option.b.bandgap_mbias_sus_en = 1;
        pow_option.b.swr_sus_en = 1;     
        
        pow_option.b.bandgap_mbias_lps_en = 1;
        //pow_option.b.efuse_power_clk_lps_en = 1; ??
        pow_option.b.swr_lps_en = 1;     
    }
#endif    
    VENDOR_WRITE(BTON_POW_OPTION_AND_32K_CTRL,pow_option.d32);    
}

void lps_lop_setting_set_xtal_en(UINT8 val)
{
    POW_OPTION_AND_32K_CTRL_S_TYPE pow_option;
    pow_option.d32 = VENDOR_READ(BTON_POW_OPTION_AND_32K_CTRL);      
    pow_option.b.bandgap_mbias_lps_en = val;
    pow_option.b.xtal_lps_en = val;
    VENDOR_WRITE(BTON_POW_OPTION_AND_32K_CTRL,pow_option.d32);    
}
#endif
#ifdef _8821A_BTON_DESIGN_
void bton_set_pcm_pull_low(UINT8 option)
{
    {
        EFUSE_POW_PARA_S_TYPE efuse_pow_para;
        efuse_pow_para.d16 = otp_str_data.power_seq_param;
        UINT16 pcm_pull_low_0_3;
#ifdef _NEW_BTON_DESIGN_AFTER_RTL8821B_TC_

        BTON_AC_REG_S_TYPE bton_0xac;
        if (efuse_pow_para.b.pcm_pull_low_en)
        {
            bton_0xac.d32 = VENDOR_READ(BTON_AC_REG);
             
            if (option == PCM_PULL_OPTION_ACT_ST)
            {
                pcm_pull_low_0_3 = efuse_pow_para.b.pcm_pull_low_act_0_3;
            }
            else if (option == PCM_PULL_OPTION_PDN_SUS_ST)
            {
                pcm_pull_low_0_3 = efuse_pow_para.b.pcm_pull_low_pdn_sus_0_3;                
            }
            else
            {
                return;
            }
            bton_0xac.BT_G3_G0_PLEN = pcm_pull_low_0_3;
            VENDOR_WRITE(BTON_AC_REG, bton_0xac.d32);
        }
#else
        BTON_BT_RFE_PAD_CTRL_REG_S_TYPE bton_0xac;
        if (efuse_pow_para.b.pcm_pull_low_en)
        {
            bton_0xac.d32 = VENDOR_READ(BTON_BT_RFE_PAD_CTRL_REG);
             
            if (option == PCM_PULL_OPTION_ACT_ST)
            {
                pcm_pull_low_0_3 = efuse_pow_para.b.pcm_pull_low_act_0_3;
            }
            else if (option == PCM_PULL_OPTION_PDN_SUS_ST)
            {
                pcm_pull_low_0_3 = efuse_pow_para.b.pcm_pull_low_pdn_sus_0_3;                
            }
            else
            {
                return;
            }
            bton_0xac.BT_G3_G0_PLEN = pcm_pull_low_0_3;
            VENDOR_WRITE(BTON_BT_RFE_PAD_CTRL_REG, bton_0xac.d32);
        }
#endif


    }
}    
#endif

/**
 * Function     : power_init
 *
 * Description  : This function is used to initial power parameter
 *
 * Parameters  : 
 *
 * Returns      : None
 *
 * Side Effects : None
 *
 */
 void power_init()
{
    POW_OPTION_AND_32K_CTRL_S_TYPE pow_option;
    EFUSE_POW_OPTION_S_TYPE update_pow_option;
    EFUSE_POW_PARA_S_TYPE efuse_pow_para;
    GENERAL_CONTROL_S_TYPE bt_general_ctrl;
#ifdef _8821A_BTON_DESIGN_
#ifdef _NEW_BTON_DESIGN_AFTER_RTL8821B_TC_
    BTON_AC_REG_S_TYPE bton_ac;
#else
    BTON_BT_RFE_PAD_CTRL_REG_S_TYPE bt_rfe_pad_ctrl_reg;
#endif
#endif

#ifdef _ROM_CODE_PATCHED_
    if (rcp_power_init != NULL)
    {
        rcp_power_init();
        return;
    }
#endif

    efuse_pow_para.d16 = otp_str_data.power_seq_param;
    //the setting value loaded from efuse
    bt_general_ctrl.d16 = otp_str_data.general_control;

    
    pow_option.d32 = VENDOR_READ(BTON_POW_OPTION_AND_32K_CTRL);
    //update power option
    update_pow_option.d16 = otp_str_data.power_option;

    if (efuse_pow_para.b.efuse_pow_option_en)
    {
        pow_option.b.bandgap_mbias_lps_en = update_pow_option.b.bandgap_mbias_lps_en;
        pow_option.b.xtal_lps_en = update_pow_option.b.xtal_lps_en;
        pow_option.b.efuse_power_clk_lps_en = update_pow_option.b.efuse_power_clk_lps_en;
        pow_option.b.swr_2_pwm_lps_en = update_pow_option.b.swr_2_pwm_lps_en;
        pow_option.b.swr_lps_en = update_pow_option.b.swr_lps_en;
        pow_option.b.bandgap_mbias_sus_en = update_pow_option.b.bandgap_mbias_sus_en;
        pow_option.b.xtal_sus_en = update_pow_option.b.xtal_sus_en;
        pow_option.b.efuse_power_clk_sus_en = update_pow_option.b.efuse_power_clk_sus_en;
        pow_option.b.swr_2_pwm_sus_en = update_pow_option.b.swr_2_pwm_sus_en;
        pow_option.b.swr_sus_en = update_pow_option.b.swr_sus_en;

#ifdef _8821A_BTON_DESIGN_
#ifdef _NEW_BTON_DESIGN_AFTER_RTL8821B_TC_

        bton_ac.d32 = VENDOR_READ(BTON_AC_REG);
        bton_ac.r_LOP_ACKF = update_pow_option.b.lop_ackf;
        bton_ac.r_SOP_ACKF = update_pow_option.b.sop_ackf;
        bton_ac.r_SOP_ERCK = update_pow_option.b.sop_erck;
        VENDOR_WRITE(BTON_AC_REG, bton_ac.d32);


        BTON_5C_REG_S_TYPE bton_5c;
        bton_5c.d32 = VENDOR_READ(BTON_5C_REG);
        bton_5c.reg_POW_CKGEN_32 = update_pow_option.b.sop_pdn_32k_en;
        VENDOR_WRITE(BTON_5C_REG, bton_5c.d32);  
#else

        bt_rfe_pad_ctrl_reg.d32 = VENDOR_READ(BTON_BT_RFE_PAD_CTRL_REG);
        bt_rfe_pad_ctrl_reg.r_LOP_ACKF = update_pow_option.b.lop_ackf;
        bt_rfe_pad_ctrl_reg.r_SOP_ACKF = update_pow_option.b.sop_ackf;
        bt_rfe_pad_ctrl_reg.r_SOP_ERCK = update_pow_option.b.sop_erck;
        VENDOR_WRITE(BTON_BT_RFE_PAD_CTRL_REG, bt_rfe_pad_ctrl_reg.d32);

        BTON_32K_CTRL_REG_S_TYPE bton_32k_ctrl;
        bton_32k_ctrl.d32 = VENDOR_READ(BTON_32K_CTRL_REG);
        bton_32k_ctrl.reg_POW_CKGEN_32 = update_pow_option.b.sop_pdn_32k_en;
        VENDOR_WRITE(BTON_32K_CTRL_REG, bton_32k_ctrl.d32);
#endif
#endif        
    }

    //USB need XTAL
    if (g_fun_interface_info.b.bt_interface == USB_INTERFACE)
    {
        EFUSE_POW_SETTING_2_S efuse_pow_setting_2;
        *(UINT8*)&efuse_pow_setting_2 = otp_str_data.efuse_pow_setting_2_d8;
        if (efuse_pow_setting_2.disable_force_usb_r_lop_extl_en == 0)
        {
            pow_option.b.xtal_lps_en = 1;
        }
    }
    
    VENDOR_WRITE(BTON_POW_OPTION_AND_32K_CTRL,pow_option.d32);
#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
    if(sleep_mode_param.dlps_restore_flow == FALSE)
#endif
    {
        RT_BT_LOG(BLUE,EFUSE_POWER_INFO,1,otp_str_data.power_option);
    }

#ifdef LPS_LOP_SETTING
    {
        // TODO: (yl) USB interface need other related procedures for LPS 
        /* other related procedures:
         *   lps_lop_setting_set_xtal_en(0) at suspend with remote-wakeup
         *   lps_lop_setting_set_xtal_en(1) at resume
         */
        EFUSE_POW_SETTING_1_S efuse_pow_setting_1;
        *(UINT8*)&efuse_pow_setting_1 = otp_str_data.efuse_pow_setting_1_d8;
        if (efuse_pow_setting_1.lop_sop_low_power_setting_en)
        {
            lop_sop_low_power_setting_init();
        }
    }
#endif

    //enable sw isr
    rlx4081_isr_sw_unmask();

    gpio_power_on_check();
#ifndef _8821A_BTON_DESIGN_ // TODO: (yl) to confirm
    if (bt_general_ctrl.b.gpio_wake_up_fun_en || bt_general_ctrl.b.uart_h5_wakeup_en)
#endif
    {
        host_sleep_pkt_man.queue_index = 0;
    }
    
#ifdef _8821A_BTON_DESIGN_
    bton_set_pcm_pull_low(PCM_PULL_OPTION_ACT_ST);
#endif    
    
}

/**
 * Function     : execute_wakeup_procedure
 *
 * Description  : This function is used to wake up host and queue rx packet
 *
 * Parameters  : pkt_type: acl, sco, or event
 *                       buf: buffer point for data
 *                       len:  rx packet payload length
 *
 * Returns      : None
 *
 * Side Effects : None
 *
 */
BOOLEAN execute_wakeup_procedure(HCI_TRANSPORT_PKT_TYPE pkt_type,
                                         UCHAR *buf, UINT16 len)
{
    UINT32 index = host_sleep_pkt_man.queue_index;
    UINT8 wait_time = (otp_str_data.control_time_parameter >> 4);
    BOOLEAN result = TRUE;
        
#ifdef _ROM_CODE_PATCHED_
#ifdef _YL_RTL8723A_B_CUT
    /* yilinli
     * 1. g_lps_timer_counter =  g_timer2_init_value;
     * 2. 
     *
     * TODO: add optional disable timer event
     * TODO: USB suspend/resume interrupt shall maintain g_host_state
     * TODO: add Event wakeup option
     * TODO: add optional timer at wakeup(USB suspend=0) and host_wake_bt=0/event to reschedule HCI RX
     * TODO: add lc_program_lps check g_host_state(host_wake_bt) and 
     * TODO: USB: remote_wakeup_fun_en set by Vendor CMD or read SIE ?
     * TODO: When in USB remote wakeup mode: need to dynamicallly toggle _EXTL and double cheusck USB suspend stat
     * TODO: Add option to decide if buffer HCI RX
     * TODO: re-sch and g_wake_first_time = 1 should be moved to where g_host_state = 0 occurs
     *          (instead of hci_wake_up_trigger_one_shot_event(wait_time))
     *
     */
#endif
    if (rcp_execute_wakeup_procedure != NULL)
    {
#ifdef _YL_RTL8723A_B_CUT    
        return rcp_execute_wakeup_procedure((void *)&pkt_type, buf, len);
#else
        rcp_execute_wakeup_procedure((void *)&pkt_type, buf, len);
#endif
    }
#endif

   DEF_CRITICAL_SECTION_STORAGE;

   MINT_OS_ENTER_CRITICAL();
   if (g_host_state)
   {
       if (g_wake_first_time)
       {           
           g_wake_first_time = 0;
           wake_up_host();
           hci_wake_up_trigger_one_shot_event(wait_time);
       }
        
       if (host_sleep_pkt_man.queue_index > HOST_SLEEP_QUEUE_PACKET_NUM)
       {
           //Free rx packet
           switch (pkt_type)
           {
               case HCI_TRANSPORT_EVENT_PKT_TYPE:
                   OS_FREE_BUFFER(tx_table[HCI_EVENT_HANDLER_TASK].pool_handle,buf);
               break;
               case HCI_TRANSPORT_ACL_DATA_PKT_TYPE:
                   OS_FREE_BUFFER(tx_table[ACL_DATA_HANDLER_TASK].pool_handle,buf);
               break;
               case HCI_TRANSPORT_SYNC_DATA_PKT_TYPE:
                   OS_FREE_BUFFER(synchronous_data_to_host_pool_id,buf);
               break;
               default:
               break;
           }
       }
       else
       {
           //Queue RX packet to software manager            
           host_sleep_pkt_man.queue_man[index].buf = buf;
           host_sleep_pkt_man.queue_man[index].len = len;
           host_sleep_pkt_man.queue_man[index].type = (UINT8) pkt_type;
           host_sleep_pkt_man.queue_index++;
           
       }
       
       result = FALSE;
   }    

   MINT_OS_EXIT_CRITICAL();

   return result;
   
}

/**
 * Function     : wake_up_host
 *
 * Description  : This function is used to wake up host with gpio or h5 function
 *
 * Parameters  : None
 *
 * Returns      : None
 *
 * Side Effects : None
 *
 */
void wake_up_host()
{
    GENERAL_CONTROL_S_TYPE bt_general_ctrl;

#ifdef _ROM_CODE_PATCHED_    
    if (rcp_wake_up_host != NULL)
    {
        rcp_wake_up_host();
        return;
    }
#endif

    //the setting value loaded from efuse
    bt_general_ctrl.d16 = otp_str_data.general_control;

    if (bt_general_ctrl.b.gpio_wake_up_fun_en)
    {
        gpio_wake_up_host();
    }
    else if (bt_general_ctrl.b.uart_h5_wakeup_en)
    {
#ifdef _UART_H5
        hci_uart_h5_check_sleep_before_hci_rx();
#endif
    }
#ifdef _YL_RTL8723A_B_CUT
// TODO: add USB resume option
// TODO: add GPIO wakeup option with level-active
// TODO: add Event wakeup option
#endif
	
}

/**
 * Function     : hci_wake_up_trigger_one_shot_event
 *
 * Description  : This function is used to create timer task to monitor host state
 *
 * Parameters  : time_ms: the poll period 
 *
 * Returns      : None
 *
 * Side Effects : None
 *
 */
void hci_wake_up_trigger_one_shot_event(UINT32 time_ms)
{
    if (hci_wake_up_tid_timer != NULL)
    {
        OS_DELETE_TIMER(&hci_wake_up_tid_timer);
    }

    /* Create a tid timer */
    OS_CREATE_TIMER(ONESHOT_TIMER, &hci_wake_up_tid_timer,
            hci_wake_up_timer_callback, NULL, 0);

    OS_START_TIMER(hci_wake_up_tid_timer, time_ms);
}

/**
 * Function     : hci_wake_up_timer_callback
 *
 * Description  : This function is used to monitor host state and re-send rx packet to HCI DMA
 *
 * Parameters  : timer_handle: no use
 *                       index: no use
 *
 * Returns      : None
 *
 * Side Effects : None
 *
 */
void hci_wake_up_timer_callback(TimerHandle_t timer_handle)
{
    UINT8 i;
    UINT8 wait_time = (otp_str_data.control_time_parameter >> 4);

    DEF_CRITICAL_SECTION_STORAGE;
    
    if (g_host_state)
    {
        hci_wake_up_trigger_one_shot_event(wait_time);
    }
    else
    {
        MINT_OS_ENTER_CRITICAL();
        g_wake_first_time = 1;
        //Send the packet, which software manager queued, to HCI DMA
        for (i=0;i<host_sleep_pkt_man.queue_index;i++)
        {
            pf_hci_transport_write((HCI_TRANSPORT_PKT_TYPE) host_sleep_pkt_man.queue_man[i].type,
                           host_sleep_pkt_man.queue_man[i].buf, 
                           host_sleep_pkt_man.queue_man[i].len, 0);
        }
        host_sleep_pkt_man.queue_index = 0;
        MINT_OS_EXIT_CRITICAL();        
    }
}

/**
 * Function     : trun_on_off_afe_ldo
 *
 * Description  : This function is used to turn on/off 
 *
 * Parameters  : on_off: is used to on or off afe ldo
 *
 * Returns      : None
 *
 * Side Effects : None
 *
 */
#ifndef _8821A_BTON_DESIGN_       
void trun_on_off_afe_ldo(UINT8 on_off)
{
    CORE_AND_AFE_LDO_CTRL_S_TYPE bton_core_afe_ldo_ctrl_reg;

#ifdef _ROM_CODE_PATCHED_
    if (rcp_trun_on_off_afe_ldo != NULL)
    {
        rcp_trun_on_off_afe_ldo((void *)&on_off);
        return;
    }
#endif

    if (on_off)
    {
        //Enable AFE LDO by sw
        bton_core_afe_ldo_ctrl_reg.d32 = VENDOR_READ(BTON_CORE_AFE_LDO_CTRL_REG);
        bton_core_afe_ldo_ctrl_reg.b.ldo_en = 1;
        VENDOR_WRITE(BTON_CORE_AFE_LDO_CTRL_REG,bton_core_afe_ldo_ctrl_reg.d32);
        /* Wait for LDO stable, ~10us, 200us is recommended */
#ifndef FOR_SIMULATION
        pf_delay(1); /* 1 msec delay */
#endif

        //Disable AFE ISO by sw
        bton_core_afe_ldo_ctrl_reg.d32 = VENDOR_READ(BTON_CORE_AFE_LDO_CTRL_REG);
        bton_core_afe_ldo_ctrl_reg.b.afe_ldo_iso_en = 0;
        VENDOR_WRITE(BTON_CORE_AFE_LDO_CTRL_REG,bton_core_afe_ldo_ctrl_reg.d32); 

    }
    else
    {
        //Enable AFE ISO by sw
        bton_core_afe_ldo_ctrl_reg.d32 = VENDOR_READ(BTON_CORE_AFE_LDO_CTRL_REG);
        bton_core_afe_ldo_ctrl_reg.b.afe_ldo_iso_en = 1;
        VENDOR_WRITE(BTON_CORE_AFE_LDO_CTRL_REG,bton_core_afe_ldo_ctrl_reg.d32);
        
        //Disable AFE LDO by sw
        bton_core_afe_ldo_ctrl_reg.d32 = VENDOR_READ(BTON_CORE_AFE_LDO_CTRL_REG);
        bton_core_afe_ldo_ctrl_reg.b.ldo_en = 0;
        VENDOR_WRITE(BTON_CORE_AFE_LDO_CTRL_REG,bton_core_afe_ldo_ctrl_reg.d32);

        //Enable fw control AFE LDO and isolation
        bton_core_afe_ldo_ctrl_reg.d32 = VENDOR_READ(BTON_CORE_AFE_LDO_CTRL_REG);
        bton_core_afe_ldo_ctrl_reg.b.afe_ldo_hw_ctr_en = 0;  
        VENDOR_WRITE(BTON_CORE_AFE_LDO_CTRL_REG,bton_core_afe_ldo_ctrl_reg.d32);        
    }
    
}
#endif

#if defined(_ENABLE_BTON_POWER_SAVING_) && defined(_TIMER_LPS_MODE_EN_)
#ifndef _REDUCE_LPS_AFTER_RTL8703B_
void lps_handle_timer2_mode_sel0(void)
{ 
    EFUSE_POW_PARA_S_TYPE efuse_pow_para;
    efuse_pow_para.d16 = otp_str_data.power_seq_param;

    /* timer2_mode_sel is 0 */
    // original timer2
    g_lps_timer_counter++;

    if( (g_lps_timer_counter& 0xff) == 0)
    {
        LPS_DBG_LOG(RED, LPS_LOG_026,1,g_lps_timer_counter);
    }
            
    //3 Handle LPS mode
    if (g_lps_timer_counter > otp_str_data.lps_mode_max_time)
    {
        g_lps_timer_counter = 0;

        if ( (efuse_pow_para.b.lps_timer_en) && 
            (g_efuse_lps_setting_2.timer2_lps_on) )
        {               
            if (lc_check_link_idle_state())
            {
                if(g_efuse_lps_setting_4.lps_chk_g_host_wake_bt != 0)   
               {  
                    if(g_host_wake_bt)
                    {  // if g_host_wake_bt == 1, can not enter lps mode
                        return;
                    }   
                }

                if(g_efuse_lps_setting_2.timer2_mode0_opt == 0)
                {
                    execute_lps_mode_procedure(0,0);
                }
                else
                {                
                    if( g_efuse_lps_setting_4.lps_use_intr )
                    {    
                        lc_post_lps_mode_signal(LPS_WO_WAKEUP_INSTANCE, 0, 0);
                    }
                    else						
                    {
                        lc_post_lps_mode_signal(LPS_TIMER2_WO_SCAN, 0, 0);
                    }
                }   
             }
         }
    }
    
}
#endif

void lps_handle_timer2_mode_sel1(void)
{
    EFUSE_POW_PARA_S_TYPE efuse_pow_para;
    efuse_pow_para.d16 = otp_str_data.power_seq_param;


#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8821_RCP_
    if (rcp_lps_handle_timer2_mode_sel1 != NULL)
    {
        if ( rcp_lps_handle_timer2_mode_sel1(NULL) )
        {
            return;
        }
    }     
#endif		
#endif

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
    if(sleep_mode_param.dlps_restore_flow == TRUE)
    {
        return;
    }
#endif

    /* timer2_mode_sel is 1 */            
    if (g_lps_timer_counter > 0)
    {
        if (( g_efuse_lps_setting_4.lps_use_state)||(g_efuse_lps_setting_4.lps_use_intr ))
        {
            if (( sleep_mode_param.lps_procedure_start) && 
                ( sleep_mode_param.bb_sm_sts == BB_NORMAL_MODE ))
            {
                lc_check_lps_for_resume();                   
            }                   
        }   

        g_lps_timer_counter --;

#ifdef _DLPS_SIMU_TEST_
        if(g_lps_timer_counter == 2)
        {
            lmp_self_device_data.scan_enable = 2;
            lc_start_write_scan_mode(lmp_self_device_data.scan_enable);
        }
#endif

    }
    else
    {
        LPS_DBG_LOG(RED, LPS_LOG_026,1,g_lps_timer_counter);
        
        if(g_efuse_lps_setting_4.lps_chk_g_host_wake_bt != 0)   
       {  
            if(g_host_wake_bt)
            {  // if g_host_wake_bt == 1, can not enter lps mode
                return;
            }   
        }
                    
        if ((efuse_pow_para.b.lps_timer_en) &&
            (g_efuse_lps_setting_2.timer2_lps_on))
        {
#ifndef _REDUCE_LPS_AFTER_RTL8703B_
            if( g_efuse_lps_setting_4.lps_use_intr )
            {    
                if(lc_check_lps_for_idle(0))
                {
                    lc_post_lps_mode_signal(LPS_WO_WAKEUP_INSTANCE, 0,0);  
                }  
            }
            else           
#endif				
            {
                if(g_efuse_lps_setting_4.lps_use_state)
                {

                    UCHAR lps_enable_flag = 0;            
                    if ( lc_check_lps_for_idle(1) )
                    {
                        lps_enable_flag = 1;                    
#ifdef _CCH_NEED_TO_CHK_
// need to move to task handle
#endif
                        UINT8 state = sleep_mode_param.lps_period_state;
                        UINT8 bitmap = sleep_mode_param.lps_period_state_bitmap;
                        UINT8 count = sleep_mode_param.lps_period_count;    
                        sleep_mode_param.lps_lpm_lps_mode = LPS_TIMER2_WITH_SCAN;  

#ifdef _MODI_LPS_STATE_WITH_INTR_
                        if(g_efuse_rsvd_2.lps_state_with_intr)
                       	{
                            lc_post_lps_stste_signal(LPS_TIMER2_WITH_SCAN);
                        }
                        else				
#endif
                        {
                            lps_period_state_machine(state, bitmap, count, LPS_TIMER2_WITH_SCAN, 0, 0);
                        }
                        return;
                    }


                    if(( lc_check_lps_for_link(0,0,1))&&(sleep_mode_param.lps_lpm_end_flag)&&g_efuse_lps_setting_2.sniff_lps_on)
                    {
#ifdef _LPS_LOG_EN_1
                        LPS_CCH_LOG(YELLOW, YL_DBG_HEX_4, 4, sleep_mode_param.lps_period_state, sleep_mode_param.lps_period_state_bitmap, 
	                               sleep_mode_param.lps_period_count, sleep_mode_param.lps_lpm_wakeup_instance);
#endif					
                        lps_enable_flag = 1;
#ifdef _CCH_NEED_TO_CHK_
// need to move to task handle
#endif
                        UINT8 state = sleep_mode_param.lps_period_state;
                        UINT8 bitmap = sleep_mode_param.lps_period_state_bitmap;
                        UINT8 count = sleep_mode_param.lps_period_count;              
                    
#ifdef _MODI_LPS_STATE_WITH_INTR_
                        if(g_efuse_rsvd_2.lps_state_with_intr)
                       	{
                            lc_post_lps_stste_signal(LPS_TIMER2_WITH_SCAN);
                        }
                        else				
#endif
                        {
                            lps_period_state_machine(state, bitmap, count, LPS_TIMER2_WITH_SCAN, 0, 0);
                        }
                    }                                 
           
                    if ( lps_enable_flag != 1 )
                    {
                       return;
                    }   

         

                }
            }    
        }           
    }
}


/**
 * Function     : pow_timer_lps
 *
 * Description  : This function is used to check if the lps time is expired.
 *                      if the time is exipred, the controller need to execute lps procedure.
 *
 * Parameters  : None
 *
 * Returns      : None
 *
 * Side Effects : None
 *
 */
void pow_timer_lps()
{
    EFUSE_POW_PARA_S_TYPE efuse_pow_para;
    efuse_pow_para.d16 = otp_str_data.power_seq_param;

#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8723_RCP_
    if (rcp_pow_timer_lps != NULL)
    {
        if ( rcp_pow_timer_lps(NULL) )
        {
            return;
        }
    }     
#endif		
#endif

    if(!(g_efuse_lps_setting_2.timer2_lps_on))
    {
        /* quit here if no enable lps mode -- austin */
        return;
    }

    if (IS_USE_FOR_BQB || 
        (lmp_self_device_data.test_mode == HCI_DUT_LOOPBACK_MODE) ||
        (lmp_self_device_data.test_mode == HCI_DEVICE_UNDER_TEST_MODE))        
    {
        /* no enter lps mode in BQB or DUT test mode */
        return;
    }

    if (sleep_mode_param.bb_sm_sts != BB_NORMAL_MODE )
    {
        LPS_DBG_LOG(RED, LPS_LOG_038, 2, g_lps_timer_counter, sleep_mode_param.bb_sm_sts);
        return;
    }
    else
    {

        if( g_lps_timer_counter > otp_str_data.lps_mode_max_time )
        {
            g_timer2_init_value = otp_str_data.lps_mode_max_time;	
            g_lps_timer_counter = otp_str_data.lps_mode_max_time;
        }            

        if(( (g_lps_timer_counter & 0x7f) == 0)&&(g_lps_timer_counter >0))
        {
        
#ifdef _2801_BTON_DESIGN_
            // Read Vendor Register Check external 32k exist   
            if(g_efuse_lps_setting_3.use_ext_32k) 
            {
                if(g_efuse_lps_setting_5.lps_check_ext_32k_exist)
                {
                    UINT32 temp_u32;
                    temp_u32 = VENDOR_READ(BTON_32K_DET_REG);
                    temp_u32 &= 0x3FFFFFFF;        
                    VENDOR_WRITE(BTON_32K_DET_REG, temp_u32);	
                    LPS_CCH_LOG(GRAY, CCH_DBG_106, 2, temp_u32, g_efuse_lps_setting_5.lps_check_ext_32k_exist);			
    	        }
            }			
#endif

            LPS_DBG_LOG(GRAY, LPS_LOG_039, 2, g_lps_timer_counter, bb_write_baseband_register_func_imp_count);

            LPS_DBG_LOG(GRAY, CCH_DBG_147, 13, lmp_self_device_data.number_of_hlc, ll_manager.conn_unit.enable, ll_manager.conn_unit.connection_cnts,
                               lmp_connection_entity[0].ce_status, lmp_connection_entity[1].ce_status, lmp_connection_entity[2].ce_status, lmp_connection_entity[3].ce_status,
                               lmp_connection_entity[4].ce_status, lmp_connection_entity[5].ce_status, lmp_connection_entity[6].ce_status, lmp_connection_entity[7].ce_status,
                               lmp_connection_entity[8].ce_status, lmp_connection_entity[9].ce_status);

        } 

    }
#ifndef _REDUCE_LPS_AFTER_RTL8703B_
    if ( g_efuse_lps_setting_2.timer2_mode_sel == 0 )	
    {
        lps_handle_timer2_mode_sel0();
    }
    else
#endif		
    {
        lps_handle_timer2_mode_sel1();
    }
}
#endif

#if !defined(_MOVE_TO_PATCH_) && !defined(SPI_FLASH_BOOT) // TODO:  to be modified/implemented in PATCH
void hw_timer_cal_func(UINT8 is_on, 
                              UINT8 cal_hw_en,
                              UINT8 wake_lps_en,
                              UINT32 timer_counter)
{
    //set timer interrupt
    TIME_CTRL_REG_S_TYPE time_ctrl_reg;
    time_ctrl_reg.d32 = VENDOR_READ(BTON_TIMER_CTRL_REG);

    time_ctrl_reg.b.timer_hw_cal_en = cal_hw_en;
    time_ctrl_reg.b.cal_timer_period = 0;
    time_ctrl_reg.b.cal_timer_en = 0;                
    VENDOR_WRITE(BTON_TIMER_CTRL_REG,time_ctrl_reg.d32);
        
    time_ctrl_reg.d32 = VENDOR_READ(BTON_TIMER_CTRL_REG);
    time_ctrl_reg.b.cal_timer_period = (timer_counter & 0x7FFFFF);
    time_ctrl_reg.b.cal_timer_en = is_on;

    enable_32k_timer_setting(wake_lps_en);

    VENDOR_WRITE(BTON_TIMER_CTRL_REG,time_ctrl_reg.d32);
}
#endif
#if !defined(_MOVE_TO_PATCH_) && !defined(SPI_FLASH_BOOT) // TODO:  to be modified/implemented in PATCH
#ifndef _CCH_REMOVE_THERMAL_METER_
void hw_thermal_cal_func(UINT8 thermal_en, 
                                 UINT8 hw_cal_en, 
                                 UINT8 diff, 
                                 UINT16 timer_period)
{
    THERMAL_METER_CTRL_S_TYPE thermal_meter_ctrl;

    thermal_meter_ctrl.d32 = VENDOR_READ(BTON_THERMAL_METER_CTRL_REG);
    thermal_meter_ctrl.b.thermal_timer_period = (timer_period & 0xFFF);
    thermal_meter_ctrl.b.thermal_delta = (diff & 0x3F);
    thermal_meter_ctrl.b.thermal_timer_en = thermal_en;
    thermal_meter_ctrl.b.thermal_timer_hw_en = hw_cal_en;

    VENDOR_WRITE(BTON_THERMAL_METER_CTRL_REG, thermal_meter_ctrl.d32);
}
#endif
#endif

#ifdef _2801_BTON_DESIGN_
void bton_32k_cal_ini()
{

#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_2801_RCP_
    if (rcp_bton_32k_cal_ini != NULL)
    {
        rcp_bton_32k_cal_ini();
        return;		
    }    
#endif
#endif


#ifdef _NEW_BTON_DESIGN_AFTER_RTL8821B_TC_

    BTON_INDIR_32K_00_REG_S_TYPE bton_indir_32k_00;
    BTON_INDIR_32K_01_REG_S_TYPE bton_indir_32k_01;
    BTON_INDIR_32K_02_REG_S_TYPE bton_indir_32k_02;		
    BTON_INDIR_32K_04_REG_S_TYPE bton_indir_32k_04;		
    BTON_INDIR_32K_08_REG_S_TYPE bton_indir_32k_08;

    bton_indir_32k_00.d16 = RD_16BIT_32K_SYSON_IO(0x0);
    bton_indir_32k_01.d16 = RD_16BIT_32K_SYSON_IO(0x1);
    bton_indir_32k_02.d16 = RD_16BIT_32K_SYSON_IO(0x2);
    bton_indir_32k_04.d16 = RD_16BIT_32K_SYSON_IO(0x4);	
    bton_indir_32k_08.d16 = RD_16BIT_32K_SYSON_IO(0x8);	

    // Choose Use New 32k Calibration 	
    if(g_efuse_lps_setting_5.lps_use_new_cal)
    {
        bton_indir_32k_00.sw_cal_en = 0;      
        bton_indir_32k_00.reg_reset_n = 1;    
        bton_indir_32k_00.cal_mode = 2;  // dsm cal mode    
        bton_indir_32k_00.cnt_read_mode = 1;  // Enable report	
        bton_indir_32k_00.inc_mode = 0;	
        bton_indir_32k_00.dsm_fin = 3;        // 00:1M 01:2M 10:4M 11:8MHz
        bton_indir_32k_00.dsm_fo_sel = 0;     // 0: 32k, 1:32.768k
		
        bton_indir_32k_02.num_32k_cyc = 5;    // before 8821Btc 16(0),32(1),64(2), 128(3) x 256 cycle option
                                              // (8821Btc 4,8,16,32,64,128,256,512) <- (use2~6)
    }else
    {
        bton_indir_32k_00.sw_cal_en = 0;  
        bton_indir_32k_00.reg_reset_n = 0;    
        bton_indir_32k_00.cal_mode = 0;  // linear cal mode
        bton_indir_32k_00.cnt_read_mode = 1;
        bton_indir_32k_00.inc_mode = 0;
        bton_indir_32k_00.dsm_fin = 3;     
        bton_indir_32k_08.RCAL_h = 0;          
        bton_indir_32k_02.kt_lim = 0x7;		
        bton_indir_32k_02.num_32k_cyc = 5; // 7: 0x4E20, 11: 0x7530		
        bton_indir_32k_01.center_cnt_fref_15_0 = 0xE200; //0x5000;
        bton_indir_32k_04.center_cnt_fref_21_16 = 4;
        bton_indir_32k_04.num_settle = 1;		
        bton_indir_32k_08.criter0 = 10;  // (After 8821B) criter1 = 2*criter0



        BTON_5C_REG_S_TYPE bton_5c;
        bton_5c.d32 = VENDOR_READ(BTON_5C_REG);
        bton_5c.d32 &= (~(BIT18|BIT17));
        bton_5c.d32 |= BIT17;        
        VENDOR_WRITE(BTON_5C_REG, bton_5c.d32);		

        POW_OPTION_AND_32K_CTRL_S_TYPE bton_32k_reg;
        bton_32k_reg.d32 = VENDOR_READ(BTON_POW_OPTION_AND_32K_CTRL);
        bton_32k_reg.b.reserved_0_13 = 2;    	
        bton_32k_reg.b.clkgen_0 = 0;
        VENDOR_WRITE(BTON_POW_OPTION_AND_32K_CTRL, bton_32k_reg.d32);
    }

    WR_16BIT_32K_SYSON_IO(0x0, bton_indir_32k_00.d16);
    WR_16BIT_32K_SYSON_IO(0x1, bton_indir_32k_01.d16);
    WR_16BIT_32K_SYSON_IO(0x2, bton_indir_32k_02.d16);
    WR_16BIT_32K_SYSON_IO(0x4, bton_indir_32k_04.d16);	
    WR_16BIT_32K_SYSON_IO(0x8, bton_indir_32k_08.d16);	
#else

    BTON_32K_TEST_REG_S_TYPE bton_32k_test;
    BTON_32K_CTRL_REG_S_TYPE bton_32k_ctrl;
    BTON_32K_CSR0_REG_S_TYPE bton_32k_csr0;

    bton_32k_test.d32 = VENDOR_READ(BTON_32K_TEST_REG);
    bton_32k_ctrl.d32 = VENDOR_READ(BTON_32K_CTRL_REG);	
    bton_32k_csr0.d32 = VENDOR_READ(BTON_32K_CSR0_REG);

    // Choose Use New 32k Calibration 	
    if(g_efuse_lps_setting_5.lps_use_new_cal)
    {
        bton_32k_ctrl.OSC32K_DBG_SEL = 1;
        bton_32k_ctrl.cnt_read_mode = 1;    // Enable report


#ifdef _LPS_NEW_32K_FW_CAL_
        if( g_efuse_lps_setting_5.lps_use_new_cal_fw_mode )
        {
            bton_32k_csr0.inc_mode = 1;

            bton_32k_test.center_cnt_fref = 625;
            bton_32k_test.tm_RCAL_32k= 0;
		    
        }else
#endif        
        {
            bton_32k_csr0.inc_mode = 0;
        }		

        bton_32k_csr0.num_32k_cyc = 1; // 16(0),32(1),64(2), 128(3) x 256 cycle option
        
    }else
    {
        bton_32k_ctrl.OSC32K_DBG_SEL = 0;
        bton_32k_ctrl.cnt_read_mode = 0;
        bton_32k_csr0.inc_mode = 0;
        bton_32k_csr0.num_32k_cyc = 7;		
        bton_32k_test.center_cnt_fref = 0x4E20;	
        bton_32k_test.tm_RCAL_32k= 0x1800;        		
    }
	

    VENDOR_WRITE(BTON_32K_TEST_REG, bton_32k_test.d32);	
    VENDOR_WRITE(BTON_32K_CTRL_REG, bton_32k_ctrl.d32);
    VENDOR_WRITE(BTON_32K_CSR0_REG, bton_32k_csr0.d32);		
    LPS_CCH_LOG(GRAY, YL_DBG_HEX_3, 3, bton_32k_test.d32, bton_32k_ctrl.d32, bton_32k_csr0.d32);

#endif
	
}
#endif

void bton_32k_cal_en()
{

#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8821_RCP_
    if (rcp_bton_32k_cal_en != NULL)
    {
        rcp_bton_32k_cal_en();
    }    
#endif
#endif

#ifdef _NEW_BTON_DESIGN_AFTER_RTL8821B_TC_


    BTON_INDIR_32K_00_REG_S_TYPE bton_indir_32k_00;

    bton_indir_32k_00.d16 = RD_16BIT_32K_SYSON_IO(0);


    if( g_efuse_lps_setting_3.use_ext_32k )
    {  // for external 32k    
        // Read Vendor Register Check external 32k exist   
        if(g_efuse_lps_setting_5.lps_check_ext_32k_exist)
        {
            UINT32 temp_u32;
            temp_u32 = VENDOR_READ(BTON_32K_DET_REG);
	        temp_u32 &= 0x3FFFFFFF;        
            VENDOR_WRITE(BTON_32K_DET_REG, temp_u32);	
            LPS_CCH_LOG(GRAY, CCH_DBG_106, 2, temp_u32, g_efuse_lps_setting_5.lps_check_ext_32k_exist);			
    	}
        return;		
    }		

    if ((bton_indir_32k_00.sw_cal_en) == 0)	
    {
        //enable fw calibration
        bton_indir_32k_00.sw_cal_en = 1;
        bton_indir_32k_00.reg_reset_n = 1;
        BTON_5C_REG_S_TYPE bton_5c;
        bton_5c.d32 = VENDOR_READ(BTON_5C_REG);
        bton_32k_ctrl_avoid_int_sts_w1c(&bton_5c.d32);
        VENDOR_WRITE(BTON_5C_REG, bton_5c.d32);
    }
	
    WR_16BIT_32K_SYSON_IO(0, bton_indir_32k_00.d16);
	
#else


#ifdef _CCH_TEST_CAL_HANG_
    // 8821 Flash HW bug only
    BTON_32K_TEST_REG_S_TYPE bton_32k_test;
    bton_32k_test.d32 = VENDOR_READ(BTON_32K_TEST_REG);
	UINT32 temp_u32 = bton_32k_test.d32;
    bton_32k_test.center_cnt_fref = 0x0050;
    VENDOR_WRITE(BTON_32K_TEST_REG, bton_32k_test.d32);
    RT_BT_LOG(YELLOW, YL_DBG_HEX_2, 2, temp_u32, bton_32k_test.d32);
#endif	

    BTON_32K_CTRL_REG_S_TYPE bton_32k_ctrl;

    bton_32k_ctrl.d32 = VENDOR_READ(BTON_32K_CTRL_REG);

    if( g_efuse_lps_setting_3.use_ext_32k )
    {  // for external 32k    
    
#ifdef _2801_BTON_DESIGN_
        // Read Vendor Register Check external 32k exist   
        if(g_efuse_lps_setting_5.lps_check_ext_32k_exist)
        {
            UINT32 temp_u32;
            temp_u32 = VENDOR_READ(BTON_32K_DET_REG);
	        temp_u32 &= 0x3FFFFFFF;        
            VENDOR_WRITE(BTON_32K_DET_REG, temp_u32);	
            LPS_CCH_LOG(GRAY, CCH_DBG_106, 2, temp_u32, g_efuse_lps_setting_5.lps_check_ext_32k_exist);			
    	}
        return;		
#else
        return;
#endif
    }		


#ifdef _8821A_BTON_DESIGN_
    if ((bton_32k_ctrl.cal_en) == 0)	
    {
        //enable fw calibration
        bton_32k_ctrl.cal_en = 1;
                         
        bton_32k_ctrl_avoid_int_sts_w1c(&bton_32k_ctrl.d32);

        VENDOR_WRITE(BTON_32K_CTRL_REG, bton_32k_ctrl.d32);
    }
#else
#ifdef _CCH_RTL8723A_B_CUT
// _CCH_ECO_LPS_
    if ((bton_32k_ctrl & 0x40000) == 0)	
    {
        //enable fw calibration
        bton_32k_ctrl |= 0x40000;

#ifdef _8821A_BTON_DESIGN_
        bton_32k_ctrl_avoid_int_sts_w1c(&bton_32k_ctrl);
#endif
        VENDOR_WRITE(BTON_32K_CTRL_REG, bton_32k_ctrl);
    }
#else
    //enable fw calibration
    bton_32k_ctrl |= 0x40000;

#ifdef _8821A_BTON_DESIGN_
    bton_32k_ctrl_avoid_int_sts_w1c(&bton_32k_ctrl);
#endif
    VENDOR_WRITE(BTON_32K_CTRL_REG, bton_32k_ctrl);
#endif	
#endif

#endif

}


// Check if (big - small) in range 0 ~ diff
UINT8 check_in_range(UINT32 big, UINT32 small, UINT32 diff)
{

    UINT32 temp_diff;
    
    if(big < small)
    {
        return FALSE;
    }else
    {
        temp_diff = big - small;
        
        if( temp_diff > diff )
        {
            return FALSE;
        }
    }

    return TRUE;
}

#ifdef _CCH_RTL8723A_B_CUT
// If unlock will check calibration enable, if disable will enable it 
UINT8 bton_32k_cal_chk_lock()
{

#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8821_RCP_
    if (rcp_bton_32k_cal_chk_lock != NULL)
    {
        UINT8 return_status;
        rcp_bton_32k_cal_chk_lock((void *)&return_status);
        return return_status;
    }    
#endif
#endif


    if( g_efuse_lps_setting_3.use_ext_32k )
    {  // for external 32k    
    

        // Read Vendor Register Check external 32k exist   
        UINT32 temp_u32;
        temp_u32 = VENDOR_READ(BTON_32K_DET_REG);
        LPS_CCH_LOG(GRAY, CCH_DBG_106, 2, temp_u32, g_efuse_lps_setting_5.lps_check_ext_32k_exist);
	    temp_u32 = (temp_u32>>30);
        if(( temp_u32 != 3 )&&(g_efuse_lps_setting_5.lps_check_ext_32k_exist))
        {
            LPS_CCH_LOG(GRAY, CCH_DBG_033, 1, temp_u32);
            return FALSE;
    	}else
        {
            temp_u32 &= 0x3FFFFFFF;        
            VENDOR_WRITE(BTON_32K_DET_REG, temp_u32);                    
            return TRUE;
        }	

    }	

#ifdef _NEW_BTON_DESIGN_AFTER_RTL8821B_TC_

    BTON_INDIR_32K_00_REG_S_TYPE bton_indir_32k_00;
    BTON_INDIR_32K_11_REG_S_TYPE bton_indir_32k_11;
    bton_indir_32k_00.d16 = RD_16BIT_32K_SYSON_IO(0x00);
    bton_indir_32k_11.d16 = RD_16BIT_32K_SYSON_IO(0x11);


    // 0: Not ready
    // 1: Lock
    // 2: harsh mode
    // 3: Fail
    UINT8 lock_val = 0;
    lock_val = (UINT8)bton_indir_32k_11.LOCK;

    if(( lock_val == 0 )|| (bton_indir_32k_00.sw_cal_en == 1))
    {
        lock_val = 0;    
        return lock_val;
    }


    // Choose Use New 32k Calibration 
    if(g_efuse_lps_setting_5.lps_use_new_cal)
    {   // ALways Lock

        BTON_INDIR_32K_02_REG_S_TYPE bton_indir_32k_02;
        bton_indir_32k_02.d16 = RD_16BIT_32K_SYSON_IO(0x02);			

        UINT32 Cnt_fref_rep = 0;
		
#ifndef _NEW_BTON_DESIGN_AFTER_RTL8703B_TC_     

        if(bton_indir_32k_02.num_32k_cyc < 2)
        {
            lock_val = 1;
            return lock_val;
        }
		
        BTON_INDIR_32K_0D_REG_S_TYPE bton_indir_32k_0D;		
        BTON_INDIR_32K_0E_REG_S_TYPE bton_indir_32k_0E;				
        bton_indir_32k_0D.d16 = RD_16BIT_32K_SYSON_IO(0x0D);		
        bton_indir_32k_0E.d16 = RD_16BIT_32K_SYSON_IO(0x0E);	

        //for 8821b tc only: 
        UINT32 rep_target = 625<<(bton_indir_32k_02.num_32k_cyc - 2)<<bton_indir_32k_00.dsm_fin<<bton_indir_32k_00.dsm_fin;
        Cnt_fref_rep = ((bton_indir_32k_0E.Cnt_fref_rep_3_21_16)<<16)+ bton_indir_32k_0D.Cnt_fref_rep_3_15_0;
		
#else
        BTON_INDIR_32K_0A_REG_S_TYPE bton_indir_32k_0A;		
        BTON_INDIR_32K_0E_REG_S_TYPE bton_indir_32k_0E;				
        bton_indir_32k_0A.d16 = RD_16BIT_32K_SYSON_IO(0x0A);		
        bton_indir_32k_0E.d16 = RD_16BIT_32K_SYSON_IO(0x0E);	

        // bton_indir_32k_00.dsm_fin = 3;        // 00:1M 01:2M 10:4M 11:8MHz
        // bton_32k_csr0.num_32k_cyc = 6;        // 4,8,16,32,64,128,256,512 x 256 cycle option
        // 2^(2+num_32k_cyc[2:0]) * 256 * 80M/dsm_fin(ex: 8M)
#ifndef _DLPS_FAKE_32K_        
        UINT32 rep_target = 2560<<(2+(bton_indir_32k_02.num_32k_cyc&0x7)+3-bton_indir_32k_00.dsm_fin);
#else
        // FPGA use 20M but not 8M
        UINT32 rep_target = 1024<<(2+(bton_indir_32k_02.num_32k_cyc&0x7)+3-bton_indir_32k_00.dsm_fin);
#endif
        Cnt_fref_rep = ((bton_indir_32k_0E.Cnt_fref_rep_0_21_16)<<16)+ bton_indir_32k_0A.Cnt_fref_rep_0_15_0;
#endif

        UINT32 rep_target_122ppm = (rep_target>>13);
		

        if(( Cnt_fref_rep <= (rep_target + rep_target_122ppm))
           && ( Cnt_fref_rep >= (rep_target - rep_target_122ppm)))
        {
            lock_val = 1;
        }else
        {
            lock_val = 2;
        }	

#ifdef _LPS_LOG_EN_		
        RT_BT_LOG(YELLOW, YL_DBG_HEX_6, 6, lock_val, bton_indir_32k_00.dsm_fin, bton_indir_32k_02.num_32k_cyc, rep_target, rep_target_122ppm, Cnt_fref_rep);
#endif
        lock_val = 1;
    }
    else
    {
        if(g_efuse_lps_setting_5.lps_use_new_cal_ini_by_old)
        {
            RT_BT_LOG(BLUE, YL_DBG_HEX_3, 3, 0x00FF, g_efuse_lps_setting_5.lps_use_new_cal, lock_val);        
            g_efuse_lps_setting_5.lps_use_new_cal = 1; 	
            bton_32k_cal_ini();
            lock_val = 3;
        }
        else
        {
            return lock_val;
        }
	
    }

    return lock_val;   
  
#else

    BTON_32K_CTRL_REG_S_TYPE bton_32k_ctrl;	

    bton_32k_ctrl.d32 = VENDOR_READ(BTON_32K_CTRL_REG);	

#ifdef _8821A_BTON_DESIGN_
    // 0: Not ready
    // 1: Lock
    // 2: harsh mode
    // 3: Fail
    UINT8 lock_val = 0;
    lock_val = (UINT8)bton_32k_ctrl.LOCK;

    if(( lock_val == 0 )|| (bton_32k_ctrl.cal_en == 1))
    {
        return lock_val;
    }

#ifdef _2801_BTON_DESIGN_ 
    // Choose Use New 32k Calibration 
    if(g_efuse_lps_setting_5.lps_use_new_cal)
    {   // ALways Lock
        UINT32 temp_ctrl1, temp_ctrl2;
        BTON_32K_CSR0_REG_S_TYPE bton_32k_csr0;		

        temp_ctrl1 = VENDOR_READ(BTON_32K_NEW_CTRL1_REG);
        temp_ctrl1 = (temp_ctrl1&0xFFFFF);
		
        temp_ctrl2 = VENDOR_READ(BTON_32K_NEW_CTRL2_REG);
        temp_ctrl2 = (temp_ctrl2&0xFFFFF);		
        bton_32k_csr0.d32 = VENDOR_READ(BTON_32K_CSR0_REG);

        if(( temp_ctrl2 <= ((0x9C40 + NEW_CAL_100PPM) <<(bton_32k_csr0.num_32k_cyc)))
           && ( temp_ctrl2 >= ((0x9C40 - NEW_CAL_100PPM) <<(bton_32k_csr0.num_32k_cyc))))
        {
            lock_val = 1;
        }else
        {
            lock_val = 2;
        }		
    }
    else
    {
        if(g_efuse_lps_setting_5.lps_use_new_cal_ini_by_old)
        {
            RT_BT_LOG(BLUE, YL_DBG_HEX_2, 2, 0xFFFF, g_efuse_lps_setting_5.lps_use_new_cal);        
            g_efuse_lps_setting_5.lps_use_new_cal = 1; 	
            lock_val = 3;
        }
        else
        {
            return lock_val;
        }
	
    }
#endif

    return lock_val;   
#else

    if ((bton_32k_ctrl & BIT18) == 0)	
    {
        if ((bton_32k_ctrl & BIT19) !=0)	
        {
            //calibration done and lock !!!       
            return TRUE;
			
        }else if ((bton_32k_ctrl & BIT20) !=0)	
        {
            //calibration done but diverge !!!       		
        }

        //calibration unlock !!!
        //need to enable fw calibration

        bton_32k_ctrl |= BIT18;
#ifdef _8821A_BTON_DESIGN_
        bton_32k_ctrl_avoid_int_sts_w1c(&bton_32k_ctrl);
#endif
        VENDOR_WRITE(BTON_32K_CTRL_REG, bton_32k_ctrl);	
        return FALSE;		
    }

    // still in calibrate state
    return FALSE;	
#endif  

#endif


}

#endif
#endif
#ifdef _ENABLE_USB_REMOTE_WAKEUP 
void usb_remote_wakeup(void)
{
#ifdef _8821A_BTON_DESIGN_    
    BTON_POW_CTRL_REG_S_TYPE pow_ctrl;
    pow_ctrl.d32 = VENDOR_READ(BTON_POW_CTRL_REG);
    bton_pow_ctrl_avoid_int_sts_w1c(&pow_ctrl);
    pow_ctrl.b.resume_to_sie_en = 0;
    
    VENDOR_WRITE(BTON_POW_CTRL_REG, pow_ctrl.d32); // added by yilinli
    pow_ctrl.b.resume_to_sie_en = 1;
    VENDOR_WRITE(BTON_POW_CTRL_REG, pow_ctrl.d32);
    pow_ctrl.b.resume_to_sie_en = 0;    
    VENDOR_WRITE(BTON_POW_CTRL_REG, pow_ctrl.d32);        
#else
    UINT32 temp;
    //RT_BT_LOG(BLUE,LMP_CH_GOOD,0,0);
    temp = VENDOR_READ(BTON_POW_CTRL_REG);
    temp &= ~(0x3FF << 8);
    temp &= ~(BIT25); // added by yilinli
    VENDOR_WRITE(BTON_POW_CTRL_REG, temp); // added by yilinli
    VENDOR_WRITE(BTON_POW_CTRL_REG, temp | BIT25);
    VENDOR_WRITE(BTON_POW_CTRL_REG, temp);    
#endif
}
#endif

#ifdef _ACTIVE_CHECK_WIFI_ON_OFF_
UINT8 check_wifi_alive(void)
{
    if (DONT_CHECK_WIFI_ALIVE || 
        (VENDOR_READ(BTON_INTERFACE_CTRL_REG) & BIT29))
    {
        is_wifi_alive = TRUE;
    }
    else
    {
        is_wifi_alive = FALSE;        
    }

    return is_wifi_alive;
}
#endif

#ifdef _ENABLE_USB_REMOTE_WAKEUP 
#if defined(_FONGPIN_TEST_USBCV_REMOTEWAKEUP_BY_GPIO0_)
void test_usb_remotewakeup_by_detect_gpio0_high(void)
{
    UINT32 dword = 0;
    g_fongpin_test_timer_counter ++;
    
    if(g_fongpin_test_timer_counter<500)
        return;

    if(g_host_state == 0)
    {// normal
        RT_BT_LOG(WHITE, YL_DBG_HEX_4, 4, VENDOR_READ(REG_BTON_REG_CTRL0), dword, g_host_state, g_fongpin_test_timer_counter);
        g_fongpin_test_timer_counter = 0;
        return;
    }
    
    if(g_host_state!=0)
    { // host low power
        // detect gpio state flow
        dword = VENDOR_READ(REG_BTON_REG_CTRL0);
        dword &= (~BIT21);// set gpio0 as input
        VENDOR_WRITE(REG_BTON_REG_CTRL0, dword);

        // read gpio0 
        dword = VENDOR_READ(REG_BTON_REG_CTRL1); 
        RT_BT_LOG(YELLOW, YL_DBG_HEX_4, 4, VENDOR_READ(REG_BTON_REG_CTRL0), dword, g_host_state, g_fongpin_test_timer_counter);

        if((dword&BIT10)&&(g_host_state == 1))// device in d2, set gpio0 to 1 to wake host
        {
            //state = 1;// normal mode
            RT_BT_LOG(GREEN, YL_DBG_HEX_4, 4, VENDOR_READ(REG_BTON_REG_CTRL0), dword, g_host_state, g_fongpin_test_timer_counter);
            g_fongpin_test_timer_counter = 0;
            usb_remote_wakeup();
        }
        
    }
    
}
#endif
#endif


