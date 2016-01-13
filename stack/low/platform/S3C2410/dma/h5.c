/***************************************************************************
 Copyright (C) RealTek Ltd.
 This module is a confidential and proprietary property of RealTek and
 a possession or use of this module requires written permission of RealTek.
 ***************************************************************************/

enum { __FILE_NUM__= 199 };

/* ========================= Include File Section ========================= */
#include "dma_usb.h"
#include "logger.h"
#include "UartPrintf.h"
#include "bt_fw_os.h"
#include "S3C2410_task.h"
#include "bb_driver.h"
#include "hci_td.h"
#include "bt_fw_hci.h"
#include "lc.h"
#include "uart.h"
#ifdef LE_MODE_EN
#include "le_ll.h"
#include "le_hci_4_0.h"
#endif
#include "gpio.h"
#include "bb_driver.h"
#include "otp.h"
#include "hci_vendor_defines.h"

#include "bz_fw_isoch.h"
#include "bt_fw_hci_internal.h"

#include "h5.h"
#include "timer.h"
#include "platform.h"
#include "lc_internal.h"

#ifdef _ROM_CODE_PATCHED_
PF_ROM_CODE_PATCH_FUNC rcp_hci_uart_h5_isr_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_hci_uart_h5_timing_adapt_func = NULL;
PF_ROM_CODE_PATCH_VOID rcp_hci_uart_h5_backup_func = NULL;
PF_ROM_CODE_PATCH_VOID rcp_hci_uart_h5_restore_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_hci_uart_h5_poll_wake_callback_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_hci_uart_h5_go_sleep_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_hci_uart_h5_go_lps_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_hci_uart_h4h5_config_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_hci_uart_vendor_set_baud_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_hci_uart_send_long_break_callback_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_hci_uart_h5_wakeup_utility_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_hci_uart_change_baudrate_event_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_hci_uart_h4_err_event_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_hci_uart_falling_cnt_isr_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_hci_uart_baud_est_init_preprocessing_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_hci_uart_baud_est_init_postprocessing_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_hci_uart_baud_recovery_func = NULL;
#endif

#ifdef _RTK8723_UART_INIT_
RLT8723_DATA_UART_SETTINGS_S g_data_uart_settings;
#ifdef _UART_H5
RLT8723_DATA_UART_SETTINGS_2_S g_data_uart_settings_2;
RLT8723_DATA_UART_SETTINGS_3_S g_data_uart_settings_3;
#ifdef _UART_BAUD_ESTIMATE_
EFUSE_BAUD_EST_SETTINGS_1_S g_efuse_baud_est_setting_1;
EFUSE_BAUD_EST_SETTINGS_2_S g_efuse_baud_est_setting_2;
EFUSE_BAUD_EST_SETTINGS_3_S g_efuse_baud_est_setting_3;
EFUSE_BAUD_EST_SETTINGS_4_S g_efuse_baud_est_setting_4;
#endif
#endif
#endif

#ifdef _UART_H5

#if 1 // _YL_H5_TIMER_FUNC_CHK
#include "mint_os_timer_internal.h"
extern OS_TIMER timer_pool[OS_MAX_TIMER_BUCKETS];
#endif

#ifdef _YL_H5_TEST
//1 For Test Only 
UINT32 h5_go_sleep_test_times = 0;
UINT32 h5_go_sleep_times = 0;
UINT32 h5_go_sleep_succ_times = 0;
UINT32 h5_go_sleep_isr_status_times = 0;
UINT32 h5_go_sleep_dma_triggered_times = 0;
UINT32 h5_go_sleep_dma_fifo_conflict_times=0;
UINT32 h5_restore_times = 0;
UINT16 h5_baud_alternate_flag = 0;
void hci_uart_h5_test_stop_reset_restore(void);
#endif


#ifdef _CCH_LPS_
extern UINT32 g_lps_timer_counter;
extern UINT32 g_timer2_init_value;
#endif

/* global variable */
HCI_UART_MAN_TYPE hci_uart_man;

TimerHandle_t h5_poll_wake_timer_id = NULL;

#ifdef _UART_BAUD_ESTIMATE_
SECTION_SRAM HCI_UART_MAN_SRAM_TYPE hci_uart_man_sram;
#endif

/* ================== For rom code patch function point ================== */
#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8723_RCP_
PF_ROM_CODE_PATCH_VOID rcp_hci_uart_h5_isr_lps_reset = NULL;
#endif
#endif

void  hci_uart_set_dlab(UINT8 value)
{
    DMA_UART_LINE_CTL_REG_S_TYPE dma_uart_line_ctl_reg;
    dma_uart_line_ctl_reg.d8 = (UINT8)UART_DWORD_READ(DMA_UART_LINE_CTL_REG_OFF);
    if(value)
        dma_uart_line_ctl_reg.b.dlab = 1;
    else
        dma_uart_line_ctl_reg.b.dlab = 0;
    UART_DWORD_WRITE(DMA_UART_LINE_CTL_REG_OFF, dma_uart_line_ctl_reg.d8);
}
#endif

#ifdef _RTK8723_UART_INIT_
BAUDRATE_FORMAT_TYPE hci_uart_read_hw_setting(void)
{
    // TODO: To Test 
    UINT32 dll, dlh, sts;
    DMA_UART_SOFEWARE_RESET_REG_S_TYPE uart_sw_rst_reg; 
#ifdef _8821A_NEW_UART_DESIGN_
    DMA_UART_REG1C_S_TYPE dma_uart_reg1c;
#else
    BTON_UART_SPI_AFE_CTRL_REG_S_TYPE bton_uart_spi_afe_ctrl_reg;
#endif
    BAUDRATE_FORMAT_TYPE baud;
    baud.d32 = 0;

    // set DLAB bit to 1
#ifdef _UART_H5    
    hci_uart_set_dlab(1);
#else    
    UART_DWORD_WRITE(DMA_UART_LINE_CTL_REG_OFF, DMA_UART_LINE_CTL_REG_DLAB1);
#endif    
    dll = UART_DWORD_READ(DMA_UART_DLL_OFF);
    dlh = UART_DWORD_READ(DMA_UART_DLH_OFF);
    sts = UART_DWORD_READ(DMA_UART_STS_REG);
    // clear DLAB bit to 0
#ifdef _UART_H5    
    hci_uart_set_dlab(0);
#else    
    UART_DWORD_WRITE(DMA_UART_LINE_CTL_REG_OFF, DMA_UART_LINE_CTL_REG_DLAB0);
#endif    

    uart_sw_rst_reg.d8 = VENDOR_BYTE_READ(DMA_UART_SOFEWARE_RESET_REG);
#ifdef _8821A_NEW_UART_DESIGN_
    dma_uart_reg1c.d32 = UART_DWORD_READ(DMA_UART_REG1C);
#else
    bton_uart_spi_afe_ctrl_reg.d32 = VENDOR_READ(BTON_UART_SPI_AFE_CTRL_REG);
#endif    
    // to optimize?
    baud.b2.uart_divisor = (dll & 0xff) + ((dlh & 0xff) << 8);
    baud.b2.uart_ovsr = ((sts & 0xf0) >> 4);
    baud.b2.uart_clk_sel = uart_sw_rst_reg.b.uart_hci_sel;    
#ifdef _8821A_NEW_UART_DESIGN_    
    baud.b2.uart_ovsr_adj = dma_uart_reg1c.b.ovsr_adj;
#else
    baud.b2.uart_pll_d = bton_uart_spi_afe_ctrl_reg.b.d;
    baud.b2.uart_pll_sel = bton_uart_spi_afe_ctrl_reg.b.sel;
#endif    
    
    return baud;
}


#ifdef _UART_H5


void hci_uart_h5_set_sleep_msg_state(UINT16 value)
{
    if(value)
        hci_uart_man.h5_sleep_msg_state=1;
    else
        hci_uart_man.h5_sleep_msg_state=0;

    if(g_data_uart_settings.h5_set_g_host_state)
        g_host_state = hci_uart_man.h5_sleep_msg_state;
}


/* Note: should be synchronized with hci_uart_h5_restore() */
void hci_uart_h5_backup(void)
{
#ifdef _8821A_BTON_DESIGN_
    BTON_INTERFACE_CTRL_REG_S_TYPE bton_interface_ctrl_reg;
#else
    BTON_UART_INFO_REG_S_TYPE bton_uart_info_reg;
#endif    
    DMA_UART_H5_INTSTS_REG_S_TYPE dma_uart_h5_intsts_reg;
    H5_BTON_TYPE h5_bton;

#ifdef _ROM_CODE_PATCHED_
    /* Reserved com code patch by yilinli:
      * e.g. 
      *    1. 
      */    
    if (rcp_hci_uart_h5_backup_func != NULL)
    {
        rcp_hci_uart_h5_backup_func();
        return;
    }        
#endif

    /* collect required states and variables to h5_bton structure */
    dma_uart_h5_intsts_reg.d32 = UART_DWORD_READ(DMA_UART_H5_INTSTS);
    h5_bton.b.h5_hw_state_d16 = dma_uart_h5_intsts_reg.b.h5_current_st_d16;
    h5_bton.b.h5_sleep_msg_state = hci_uart_man.h5_sleep_msg_state;

    /* Store to BTON register */
#ifdef _8821A_BTON_DESIGN_
    bton_interface_ctrl_reg.d32 = VENDOR_READ(BTON_INTERFACE_CTRL_REG);
    bton_interface_ctrl_reg.b.reg_uart_info_h5_sts_d16 = h5_bton.d16;
    VENDOR_WRITE(BTON_INTERFACE_CTRL_REG, bton_interface_ctrl_reg.d32);
#else
    bton_uart_info_reg.d32 = VENDOR_READ(BTON_UART_INFO_REG);
    bton_uart_info_reg.b.reg_uart_info_d16 = h5_bton.d16;
    VENDOR_WRITE(BTON_UART_INFO_REG, bton_uart_info_reg.d32);
#endif    
    
    return ;
}

/* Note: should be synchronized with hci_uart_h5_backup() */
void hci_uart_h5_restore(void)
{
#ifdef _8821A_BTON_DESIGN_
    BTON_INTERFACE_CTRL_REG_S_TYPE bton_interface_ctrl_reg;
#else
    BTON_UART_INFO_REG_S_TYPE bton_uart_info_reg;
#endif    
    DMA_UART_H5_INTEN_REG_S_TYPE dma_uart_h5_inten_reg;
    H5_BTON_TYPE h5_bton;
    DEF_CRITICAL_SECTION_STORAGE;

#ifdef _ROM_CODE_PATCHED_
    /* Reserved com code patch by yilinli:
      * e.g. 
      *    1. 
      */    
    if (rcp_hci_uart_h5_restore_func != NULL)
    {
        rcp_hci_uart_h5_restore_func();
        return;
    }   
#endif

    MINT_OS_ENTER_CRITICAL();



#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
    if(sleep_mode_param.dlps_restore_flow)
    {
        DMA_UART_H5_INTSTS_REG_S_TYPE dma_uart_h5_intsts_reg;

        dma_uart_h5_intsts_reg.d32=g_sram_patch_uart_status;
        /* restore to proper registers and variables */
        dma_uart_h5_inten_reg.d32 = UART_DWORD_READ(DMA_UART_H5_INTEN);
        dma_uart_h5_inten_reg.b.h5_backup_st_d16 = dma_uart_h5_intsts_reg.b.h5_current_st_d16;
        UART_DWORD_WRITE(DMA_UART_H5_INTEN,dma_uart_h5_inten_reg.d32);

        hci_uart_h5_set_sleep_msg_state(0);

        MINT_OS_EXIT_CRITICAL();
        return;
    }
#endif


    /* read from BTON register to h5_bton structure */
    /* Note: h5_bton includes NOT ONLY h5 hardware state BUT ALSO other variables (e.g. sleep_msg_state) */
#ifdef _8821A_BTON_DESIGN_
    bton_interface_ctrl_reg.d32=VENDOR_READ(BTON_INTERFACE_CTRL_REG);
    h5_bton.d16 = bton_interface_ctrl_reg.b.reg_uart_info_h5_sts_d16;
#else
    bton_uart_info_reg.d32=VENDOR_READ(BTON_UART_INFO_REG);
    h5_bton.d16 = bton_uart_info_reg.b.reg_uart_info_d16;
#endif    
    /* restore to proper registers and variables */
    dma_uart_h5_inten_reg.d32 = UART_DWORD_READ(DMA_UART_H5_INTEN);
    dma_uart_h5_inten_reg.b.h5_backup_st_d16 = h5_bton.b.h5_hw_state_d16;
    UART_DWORD_WRITE(DMA_UART_H5_INTEN,dma_uart_h5_inten_reg.d32);

    hci_uart_h5_set_sleep_msg_state(h5_bton.b.h5_sleep_msg_state);
//    hci_uart_man.h5_sleep_msg_state = h5_bton.b.h5_sleep_msg_state;
//    g_host_state = h5_bton.b.h5_sleep_msg_state;

    MINT_OS_EXIT_CRITICAL();
}

/* Note: this funnction should be re-tuned for different clock/pll architectures */
/* return 0 when divisor is 0, which is abnormal*/
UINT32  hci_uart_calculate_baudrate_hz(BAUDRATE_FORMAT_TYPE baud)
{
    UINT32 baudrate_hz;
    if (baud.b2.uart_divisor==0)
    {
        baudrate_hz = 0;
    }
    else
    {
#ifdef _8821A_NEW_UART_DESIGN_
    UINT32 uart_clk;
    UINT16 uart_ovsr_x11 = (baud.b2.uart_ovsr+5)*11;
    UINT16 uart_ovsr_adj = baud.b2.uart_ovsr_adj;
    UINT8 ii;
    for (ii = 0; ii<11; ii++)
    {   
        if (uart_ovsr_adj & 0x0001)
        {
            uart_ovsr_x11 +=1 ;
        }
        uart_ovsr_adj = uart_ovsr_adj >> 1;
    }
#ifdef IS_BTSOC
    uart_clk = cpu_clk;
#else
    if (baud.b2.uart_clk_sel)
    {
        uart_clk = 120000000;
    }
    else
    {
        uart_clk = 40000000;
    }
#endif
    baudrate_hz = (uart_clk*11) / (uart_ovsr_x11*baud.b2.uart_divisor);
    
#else
        baudrate_hz = g_uart_clk / ((baud.b2.uart_ovsr+5)*baud.b2.uart_divisor);
        if (baud.b2.uart_clk_sel)
        {
              baudrate_hz >>= 1;
              baudrate_hz *= (baud.b2.uart_pll_d+2);
              switch (baud.b2.uart_pll_sel)
              {
                  case 0:
                      baudrate_hz >>= 2;
                      break;
                  case 1:
                      baudrate_hz /= 5;
                      break;
                  case 2:
                  default: 
                      baudrate_hz /= 9;
                      break;
              }
        }
#endif        
    }
    return baudrate_hz;
}        

#ifndef _8821A_NEW_UART_DESIGN_
void hci_uart_divisor_ovsr_config(UINT16 divisor, UINT8 ovsr)
#else
#ifdef _UART_BAUD_ESTIMATE_
void hci_uart_set_toggle_mon_en(UINT8 val)
{
    DMA_UART_TOGGLE_MON_CTRL_REG_S_TYPE uart_toggle_mon;
    uart_toggle_mon.d32 = UART_DWORD_READ(DMA_UART_TOGGLE_MON_CTRL);
    uart_toggle_mon.b.uart_toggle_mon_en = (val!=0);
    UART_DWORD_WRITE(DMA_UART_TOGGLE_MON_CTRL,uart_toggle_mon.d32);
}
void hci_uart_set_falling_cnt_intr_en(UINT8 val)
{
    DMA_UART_H5_INTEN_REG_S_TYPE uart_h5_inten;
    uart_h5_inten.d32 = UART_DWORD_READ(DMA_UART_H5_INTEN);
    uart_h5_inten.b.uart_falling_cnt_intr_en = (val!=0);
    UART_DWORD_WRITE(DMA_UART_H5_INTEN,uart_h5_inten.d32);
}
void hci_uart_set_falling_cnt_th(UINT16 val)
{
    DMA_UART_TOGGLE_MON_CTRL_REG_S_TYPE uart_toggle_mon;
    uart_toggle_mon.d32 = UART_DWORD_READ(DMA_UART_TOGGLE_MON_CTRL);
    uart_toggle_mon.b.uart_falling_cnt_th = val;
    UART_DWORD_WRITE(DMA_UART_TOGGLE_MON_CTRL,uart_toggle_mon.d32);
}
#endif

void hci_uart_divisor_ovsr_config(UINT16 divisor, UINT8 ovsr, UINT16 ovsr_adj)
#endif
{
        UINT8 dlh;
        UINT8 dll;
        UINT8 sts;   
#ifdef _8821A_NEW_UART_DESIGN_        
        DMA_UART_REG1C_S_TYPE dma_uart_reg1c;
#endif        
//        DMA_UART_LINE_CTL_REG_S_TYPE dma_uart_line_ctl_reg;        
        
        dll = divisor & 0xff;
        dlh = (divisor & 0xff00)>>8;
        sts = ((ovsr)&0xf)<<4;        
        /* Note: DLL and DLH should be programmed when DLAB = 1 */       
        hci_uart_set_dlab(1);       
        UART_DWORD_WRITE(DMA_UART_DLL_OFF, dll);
        UART_DWORD_WRITE(DMA_UART_DLH_OFF, dlh);
        UART_DWORD_WRITE(DMA_UART_STS_REG, sts);      
        hci_uart_set_dlab(0);        
        DMA_UART_DBG_LOG(BLUE, DMA_UART_002_DBG_DEC, 1, dll);
        DMA_UART_DBG_LOG(BLUE, DMA_UART_002_DBG_DEC, 1, dlh);
        DMA_UART_DBG_LOG(BLUE, DMA_UART_002_DBG_DEC, 1, sts);
#ifdef _8821A_NEW_UART_DESIGN_
        dma_uart_reg1c.d32 = UART_DWORD_READ(DMA_UART_REG1C);
        dma_uart_reg1c.b.ovsr_adj = ovsr_adj;
        UART_DWORD_WRITE(DMA_UART_REG1C, dma_uart_reg1c.d32);
        DMA_UART_DBG_LOG(BLUE, DMA_UART_003_DBG_HEX, 1, ovsr_adj);        
        DMA_UART_DBG_LOG(BLUE, DMA_UART_003_DBG_HEX, 1, dma_uart_reg1c.d32);        
        DMA_UART_DBG_LOG(BLUE, DMA_UART_003_DBG_HEX, 1, UART_DWORD_READ(DMA_UART_REG1C));        
#endif        
}        
/* read data UART CTSN value and invert it
 * return value:
 *   1: CTSN = 0, CTS = 1, means granted to send
 *   0: CTSN = 1, CTS = 0, means NOT granted to send  */
UINT8 hci_uart_read_cts_in_status(void)
{
    return ((UART_DWORD_READ(UART_MODEM_STATUS_REG_OFF)&BIT4)==0);
}

void hci_uart_set_hw_fctrl_on(UINT8 value)
{
        UINT8 mcr_reg_d8;
        mcr_reg_d8 = UART_DWORD_READ(UART_MODEM_CTL_REG_OFF);
        if(value)
        {
            mcr_reg_d8 |= 0x20;
        }
        else
        {
            mcr_reg_d8 &= 0xdf;
        }
        UART_DWORD_WRITE(UART_MODEM_CTL_REG_OFF, mcr_reg_d8);
        DMA_UART_DBG_LOG(BLUE,DMA_UART_075,1, mcr_reg_d8);        
}

#ifdef _8821A_BTON_DESIGN_
// Note: PIN RTSN = ~RTS //
void hci_uart_set_mcr_rtsn(UINT8 value)
{
        UINT8 mcr_reg_d8;
        mcr_reg_d8 = UART_DWORD_READ(UART_MODEM_CTL_REG_OFF);
        if(value)
        {
            mcr_reg_d8 &= (~BIT1);
        }
        else
        {
            mcr_reg_d8 |= BIT1;
        }
        UART_DWORD_WRITE(UART_MODEM_CTL_REG_OFF, mcr_reg_d8);
        DMA_UART_DBG_LOG(BLUE,DMA_UART_075,1, mcr_reg_d8);        
}
#endif

void hci_uart_set_h4_error_interrupt_en(UINT8 value)
{
        UINT8 mcr_reg_d8;
        mcr_reg_d8 = UART_DWORD_READ(UART_MODEM_CTL_REG_OFF);
        if(value)
        {
            mcr_reg_d8 |= 0x04;
        }
        else
        {
            mcr_reg_d8 &= 0xfb;
        }
        UART_DWORD_WRITE(UART_MODEM_CTL_REG_OFF, mcr_reg_d8);
        DMA_UART_DBG_LOG(BLUE,DMA_UART_075,1, mcr_reg_d8);        
}

/* set UART_MODEM_CTL_REG_OFF[7] = 0 or 1 */
void hci_uart_set_hci_mode_en(UINT8 value)
{
        UINT8 mcr_reg_d8;
        mcr_reg_d8 = UART_DWORD_READ(UART_MODEM_CTL_REG_OFF);
        if(value)
        {
            mcr_reg_d8 |= 0x80;
        }
        else
        {   
            mcr_reg_d8 &= 0x7f;
        }
        
        UART_DWORD_WRITE(UART_MODEM_CTL_REG_OFF, mcr_reg_d8);
        DMA_UART_DBG_LOG(BLUE,DMA_UART_075,1, mcr_reg_d8);
}

/* set H5_CTL1.h5_en = 0 or 1 */
void hci_uart_set_h5_en(UINT8 value)
{
        DMA_UART_H5_CTL1_REG_S_TYPE dma_uart_h5_ctl1_reg;
        dma_uart_h5_ctl1_reg.d32 = UART_DWORD_READ(DMA_UART_H5_CTL1);
        dma_uart_h5_ctl1_reg.b.h5_en = value;
        UART_DWORD_WRITE(DMA_UART_H5_CTL1, dma_uart_h5_ctl1_reg.d32);
}

void hci_uart_h5_inten_config(void)
{
    DMA_UART_H5_INTEN_REG_S_TYPE dma_uart_h5_inten_reg;
    dma_uart_h5_inten_reg.d32 = UART_DWORD_READ(DMA_UART_H5_INTEN);
    dma_uart_h5_inten_reg.d32 &= 0xffff0000;
    if (g_data_uart_settings_2.h5_en)
    {
        dma_uart_h5_inten_reg.d32 |= g_data_uart_settings_2.h5_int_en;
    }  
    UART_DWORD_WRITE(DMA_UART_H5_INTEN, dma_uart_h5_inten_reg.d32);
}

/* configure H5, EXCEPT h5_en */
/* note: this function would set CTL0 as all-zero except active_sync */
/* note: this function would set CTL0.active_sync and CTL1 according to EFUSE */
void hci_uart_h5_config_init()
{
    DMA_UART_H5_CTL1_REG_S_TYPE dma_uart_h5_ctl1_reg;
    DMA_UART_H5_CTL0_REG_S_TYPE dma_uart_h5_ctl0_reg;
    //UINT16 h5_en_temp;

    dma_uart_h5_ctl0_reg.d32 = 0;   
    if (g_data_uart_settings_2.h5_en)
    {
        dma_uart_h5_ctl0_reg.b.active_sync = g_data_uart_settings_2.h5_active_sync;
        dma_uart_h5_ctl0_reg.b.force_no_oof = g_data_uart_settings_2.h5_force_no_oof;
    }
    dma_uart_h5_ctl0_reg.b.host_to_val = g_data_uart_settings_2.h5_host_to_val;
    UART_DWORD_WRITE(DMA_UART_H5_CTL0, dma_uart_h5_ctl0_reg.d32);

    
    //h5_en_temp = dma_uart_h5_ctl1_reg.b.h5_en; /* uninitialized value ?? */
    dma_uart_h5_ctl1_reg.d32 = 0;   
    //dma_uart_h5_ctl1_reg.b.h5_en = h5_en_temp;
    if (g_data_uart_settings_2.h5_en)
    {
        dma_uart_h5_ctl1_reg.b.ignore_sync = g_data_uart_settings_2.h5_ignore_sync;
        dma_uart_h5_ctl1_reg.b.force_oof_ctrl = g_data_uart_settings_2.h5_force_oof_ctrl;
        dma_uart_h5_ctl1_reg.b.wake_to_val = g_data_uart_settings_2.h5_wake_to_val;
        dma_uart_h5_ctl1_reg.b.ack_to_val = g_data_uart_settings_2.h5_ack_to_val;
        dma_uart_h5_ctl1_reg.b.sync_to_val = g_data_uart_settings_2.h5_sync_to_val;
        dma_uart_h5_ctl1_reg.b.resend_to_val = g_data_uart_settings_2.h5_resend_to_val;
        dma_uart_h5_ctl1_reg.b.retry_limit = g_data_uart_settings_2.h5_retry_limit;     
    }
    UART_DWORD_WRITE(DMA_UART_H5_CTL1, dma_uart_h5_ctl1_reg.d32);
    
    // TODO: move to where ???
    hci_uart_h5_inten_config();         
}

/* Executed after hci_uart_line_ctrl_config at Initialization or Baudrate Change */
API_RESULT hci_uart_h5_timing_adapt(UINT32 baudrate_hz)
{

//    UINT16 hci_uart_target_resend_to_ms; // = (250>>g_hci_uart_resend_target);
    UINT32 temp_resend_to_val_d32; // = hci_uart_target_resend_to_ms * (baudrate_hz>>10);
    UINT16 temp_resend_to_val_d16_shift;
    UINT8 temp_adapt_resend_to_val;
    UINT8 adapt_resend_to_val;
    INT8 temp_retry_limit_signed;
    UINT8 adapt_retry_limit;
    DMA_UART_H5_CTL1_REG_S_TYPE dma_uart_h5_ctl1_reg;    

#ifdef _ROM_CODE_PATCHED_
    /* Reserved com code patch by yilinli:
      * e.g. 
      *    1. 
      */    
    if (rcp_hci_uart_h5_timing_adapt_func != NULL)
    {
        return rcp_hci_uart_h5_timing_adapt_func((void*)(&baudrate_hz));
    }    
#endif

    if (baudrate_hz == 0)
    {
        return API_FAILURE;
     }
//    if (g_data_uart_settings_3.parity_en)
//        temp_resend_to_val *= temp_resend_to_val*23;
//    else
//        temp_resend_to_val *= temp_resend_to_val*25;
//    temp_resend_to_val = temp_resend_to_val>>8;
   /* *23/256*(1s/4)/2^g_hci_uart_resend_target/2^shift(=4) */
    /* It is better to limit the maximum allowable baudrate_hz and gate it*/
    temp_resend_to_val_d32 = (baudrate_hz*23) >> (14+g_data_uart_settings_3.h5_resend_target); 
    temp_resend_to_val_d16_shift = (temp_resend_to_val_d32>0xffff) ? 0xffff : (UINT16)temp_resend_to_val_d32;
    
    /* this may be further optimized for code size reduction */
    if (temp_resend_to_val_d16_shift>>13) // >=2^(13+4)(250ms @ 5.77MBaud)
        temp_adapt_resend_to_val = 8; 
    else if (temp_resend_to_val_d16_shift>>12) // >=2^(12+4)(250ms @ 2.88MBaud )
        temp_adapt_resend_to_val = 7; 
    else if (temp_resend_to_val_d16_shift>>11) // >=2^(11+4)(250ms @ 1.44MBaud)
        temp_adapt_resend_to_val = 6;
    else if (temp_resend_to_val_d16_shift>>10) // >=2^(10+4)(250ms @ 720kBaud)
        temp_adapt_resend_to_val = 5;
    else if (temp_resend_to_val_d16_shift>>9) // >=2^(9+4)(250ms @ 360kBaud)
        temp_adapt_resend_to_val = 4;
    else if (temp_resend_to_val_d16_shift>>8) // >=2^(8+4)(250ms @ 180kBaud)
        temp_adapt_resend_to_val = 3;
    else if (temp_resend_to_val_d16_shift>>7) // >=2^(7+4)(250ms @ 90kBaud)
        temp_adapt_resend_to_val = 2;
    else if (temp_resend_to_val_d16_shift>>6) // >=2^(6+4)(250ms @ 45kBaud)
        temp_adapt_resend_to_val = 1;
    else     // <2^(5+4)(250ms @ 22.5kBaud)
        temp_adapt_resend_to_val = 0;        

    adapt_resend_to_val = MAX(MIN(temp_adapt_resend_to_val, g_data_uart_settings_3.h5_resend_time_adapt_max),
                                                                                                g_data_uart_settings_3.h5_resend_time_adapt_min);
    
    /* We want (h5_resend_timeout(ms)*h5_retry_limit_new(times)) = (h5_resend_target(ms)*h5_retry_limit(times)) */
    temp_retry_limit_signed = g_data_uart_settings_2.h5_retry_limit;
    if (g_data_uart_settings_3.h5_retry_limit_adapt_en)
    {
        // TODO: To double-confirm (and test)
        temp_retry_limit_signed += (temp_adapt_resend_to_val - adapt_resend_to_val);
    }
    adapt_retry_limit = MAX(MIN(temp_retry_limit_signed, 7), 0);
    
    dma_uart_h5_ctl1_reg.d32 = UART_DWORD_READ(DMA_UART_H5_CTL1);    
    dma_uart_h5_ctl1_reg.b.retry_limit = adapt_retry_limit;
    dma_uart_h5_ctl1_reg.b.resend_to_val = adapt_resend_to_val;

    // TODO: make sure if need to set h5_en = 0 and h5_en = 1 to avoid "counter overrun risk"
    UART_DWORD_WRITE(DMA_UART_H5_CTL1, dma_uart_h5_ctl1_reg.d32);    
    DMA_UART_LOG(GRAY, DMA_UART_086, 4, baudrate_hz, g_data_uart_settings_3.h5_resend_target, adapt_resend_to_val, adapt_retry_limit);        

    return API_SUCCESS;
}

/* configure UART lower layer */
void hci_uart_line_ctrl_config(void)
{
    DMA_UART_LINE_CTL_REG_S_TYPE dma_uart_line_ctl_reg;
#if 1
    dma_uart_line_ctl_reg.d8 = 0;
//    dma_uart_line_ctl_reg.b.dlab = 0;
//    dma_uart_line_ctl_reg.b.stick_parity = 0;
    dma_uart_line_ctl_reg.b.word_len = 3; /* 3: 8-bit */
    dma_uart_line_ctl_reg.b.parity_en = g_data_uart_settings_3.parity_en;
    dma_uart_line_ctl_reg.b.parity_even = g_data_uart_settings_3.parity_even;
    UART_DWORD_WRITE(DMA_UART_LINE_CTL_REG_OFF, dma_uart_line_ctl_reg.d8);
    
    /* set UART_MODEM_CTL_REG_OFF as 0xa0 to enable flow control */
    if (g_data_uart_settings.uart_set_flow_control_after_hci_en == 0)
    {
        hci_uart_set_hw_fctrl_on(g_data_uart_settings_3.hw_fctrl_on);
    }
#else
    /* set UART_MODEM_CTL_REG_OFF as 0xa0 to enable flow control */
    UART_DWORD_WRITE(UART_MODEM_CTL_REG_OFF, 0xa0);
#endif
}
#endif


void hci_uart_set_vendor_hci_en(UINT8 value)
{
        DMA_UART_SOFEWARE_RESET_REG_S_TYPE dma_uart_software_reset_reg;
        dma_uart_software_reset_reg.d8 = VENDOR_BYTE_READ(DMA_UART_SOFEWARE_RESET_REG);
        dma_uart_software_reset_reg.b.uart_hci_en = value;
        VENDOR_BYTE_WRITE(DMA_UART_SOFEWARE_RESET_REG, dma_uart_software_reset_reg.d8);
}        

#if 0 // To be check by compiler flag: SPI_FLASH_BOOT
/* NOTE: need to be executed before FW enabling UART PLL */
UINT8 hci_uart_check_spi_mode(void)
{
    CORE_AND_AFE_LDO_CTRL_S_TYPE bton_core_afe_ldo_ctrl_reg;
    bton_core_afe_ldo_ctrl_reg.d32 = VENDOR_READ(BTON_CORE_AFE_LDO_CTRL_REG);
    UINT8 spi_mode = bton_core_afe_ldo_ctrl_reg.b.uart_pll_en_by_hw_sts;
    return spi_mode;
}
#endif

#ifndef _8821A_NEW_UART_DESIGN_
UINT8 wait_uart_spi_pll_ready(void)
{
#ifdef _NEW_BTON_DESIGN_AFTER_RTL8821B_TC_
    BTON_74_REG_S_TYPE bton_74;
#else
    CORE_AND_AFE_LDO_CTRL_S_TYPE bton_core_afe_ldo_ctrl_reg;
#endif

    UINT8 pll_ckrdy_bt;
    UINT32 pll_ckrdy_times ; /* should use UINT32 to avoid Over-Flow */
    UINT8 pll_ckrdy_timeout;
    UINT8 exit_flag;
    BAUDRATE_FORMAT_TYPE baud_dummy;
    
   /* wait for PLL READY */
   pll_ckrdy_times = 0;
   pll_ckrdy_timeout = 0;
   exit_flag = 0;
   while (exit_flag == 0)
   {
#ifdef _NEW_BTON_DESIGN_AFTER_RTL8821B_TC_

       bton_74.d32 = VENDOR_READ(BTON_74_REG);                     
       // TODO: Register read can not be test. Therefore should make sure the register access
       pll_ckrdy_bt = bton_74.PLL_CKRDY_BT;
#else
       bton_core_afe_ldo_ctrl_reg.d32 = VENDOR_READ(BTON_CORE_AFE_LDO_CTRL_REG);
#ifdef _DBG_UART_FORCE_SETTINGS_
//                     bton_core_afe_ldo_ctrl_reg.d32 |= BIT26;
#endif                        
       // TODO: Register read can not be test. Therefore should make sure the register access
       pll_ckrdy_bt = bton_core_afe_ldo_ctrl_reg.b.uart_spi_pll_ready_sts;
#endif

       ++pll_ckrdy_times;

       if ((g_data_uart_settings.pll_ckrdy_timeout_en) && 
            (pll_ckrdy_times>=PLL_CKRDY_TIMEOUT_LOOPS))
           pll_ckrdy_timeout = 1;
       else
           pll_ckrdy_timeout = 0;

       //1 For debug?   YES! Debugging Log   /* YL20110328 */
       /*
       if ( ((pll_ckrdy_times % 100000) == 1) && (pll_ckrdy_times <1000000))
       {
           DMA_UART_LOG(RED, DMA_UART_011, 2, pll_ckrdy_times, PLL_CKRDY_TIMEOUT_LOOPS);
           DMA_UART_LOG(YELLOW, DMA_UART_018, 1, bton_core_afe_ldo_ctrl_reg.d32);
       }
       */
       
       if (pll_ckrdy_bt || pll_ckrdy_timeout)
       {
           exit_flag = 1;
           
           if ((g_data_uart_settings.ck_timeout_afe_off_en) && (pll_ckrdy_timeout))
           {
                /* Before Turn Off LDO and PLL, should make sure 
                that isolation between analog/digital is done */
                trun_on_off_uart_spi_pll(PLL_OFF, baud_dummy);
           }
       }
   } /* end of while(exit_flag == 1) */

#ifdef _NEW_BTON_DESIGN_AFTER_RTL8821B_TC_
   DMA_UART_LOG(RED, DMA_UART_021, 4, pll_ckrdy_times, 
                                      pll_ckrdy_bt, 
                                      pll_ckrdy_timeout, 
                                      bton_74.d32);
#else
   DMA_UART_LOG(RED, DMA_UART_021, 4, pll_ckrdy_times, 
                                      pll_ckrdy_bt, 
                                      pll_ckrdy_timeout, 
                                      bton_core_afe_ldo_ctrl_reg.d32);
#endif

   return pll_ckrdy_timeout;
}
#endif

//API_RESULT uart_set_baud_clk_RTL8723(BAUDRATE_FORMAT_TYPE baud, UINT16 re_init_flag)
/**************************************************************************
 * read hardware PLL setting
 * optionally set uart_hci_en = 0 (recommended due to clk transient)
 * if(spi_mode)
 *    check if spi mode PLL conflict;
 * else
 *    turn on/off(optioinal) AFE/PLL if needed 
 *
 * if PLL conflict or PLL program fail
 *    return API_FAILURE 
 * 
 * configuration UART physical layer (parity, divisor, ovsr, hw_fctrl)
 * 
 ***************************************************************************/
API_RESULT uart_set_baud_clk_RTL8723(BAUDRATE_FORMAT_TYPE baud, 
                                                UINT16 re_init_flag, 
                                                UINT16 uart_reset_flag)
{
#ifndef _8821A_NEW_UART_DESIGN_    
    BAUDRATE_FORMAT_TYPE baud_hw_setting;
#endif    
    DMA_UART_SOFEWARE_RESET_REG_S_TYPE dma_uart_software_reset_reg;
#ifndef _8821A_NEW_UART_DESIGN_    
    BTON_UART_SPI_AFE_CTRL_REG_S_TYPE bton_uart_spi_afe_ctrl_reg;
//    CORE_AND_AFE_LDO_CTRL_S_TYPE bton_core_afe_ldo_ctrl_reg;
    UINT8 spi_mode; 
#endif

#ifndef _8821A_NEW_UART_DESIGN_    
    /* use for FW execution flow */
    UINT8 goto_afe_conflict = 0;
//    UINT8 pll_ckrdy_bt;
//    UINT32 pll_ckrdy_times = 0; /* should use UINT32 to avoid Over-Flow */
    UINT8 pll_ckrdy_timeout = 0;
//    UINT8 exit_flag = 0;
#endif

    UINT16 divisor;
    UINT8 ovsr;
#ifdef _8821A_NEW_UART_DESIGN_    
    UINT16 ovsr_adj;
#endif    
    UINT32 baudrate_hz;
#ifndef _UART_H5    
    UINT8 dlh;
    UINT8 dll;
    UINT8 sts;    
#endif    
    
#ifndef _8821A_NEW_UART_DESIGN_    
    baud_hw_setting = hci_uart_read_hw_setting();
    bton_uart_spi_afe_ctrl_reg.d32 = VENDOR_READ(BTON_UART_SPI_AFE_CTRL_REG);
//    bton_core_afe_ldo_ctrl_reg.d32 = VENDOR_READ(BTON_CORE_AFE_LDO_CTRL_REG);
    // TODO: Need to double-check the detection flag, If need to also check bton_core_afe_ldo_ctrl_reg.b.uart_pll_en??
    // TODO: Register read can not be test. Therefore should make sure the register access
//    spi_mode = bton_core_afe_ldo_ctrl_reg.b.uart_pll_ldo_en_by_hw_sts;
    spi_mode = hci_uart_check_spi_mode();
    DMA_UART_LOG(YELLOW, DMA_UART_017, 1, bton_uart_spi_afe_ctrl_reg.d32);    
    RT_BT_LOG(GREEN, DMA_UART_015, 3, baud.d32, re_init_flag,baud_hw_setting.d32);    
#else
//    RT_BT_LOG(GREEN, DMA_UART_130, 2, baud.d32, re_init_flag);    
#endif
    
//    DMA_UART_LOG(YELLOW, DMA_UART_018, 1, bton_core_afe_ldo_ctrl_reg.d32);

#ifdef _DBG_UART_FORCE_SETTINGS_
//    spi_mode = 1;
//   baud_hw_setting.b2.uart_pll_d =  !(baud.b2.uart_pll_d);
//    baud_hw_setting.b2.uart_pll_sel = baud.b2.uart_pll_sel;
#endif   

    DMA_UART_DBG_LOG(BLUE, DMA_UART_002_DBG_DEC, 1, PLL_CKRDY_TIMEOUT_LOOPS);

    if (g_data_uart_settings.en0_before_set_en)
    {
        dma_uart_software_reset_reg.d8 = VENDOR_BYTE_READ(DMA_UART_SOFEWARE_RESET_REG);
        dma_uart_software_reset_reg.b.uart_hci_en = 0;
        VENDOR_BYTE_WRITE(DMA_UART_SOFEWARE_RESET_REG, dma_uart_software_reset_reg.d8);
        DMA_UART_DBG_LOG(BLUE, DMA_UART_003_DBG_HEX, 1, dma_uart_software_reset_reg.d8);
    }
    
#ifndef _8821A_NEW_UART_DESIGN_    
    if(spi_mode)
    {
        if(baud.b2.uart_clk_sel)
        {
            goto_afe_conflict = 0;
            if (baud_hw_setting.b2.uart_pll_d ==  baud.b2.uart_pll_d)
            {
                if (!re_init_flag)
                {
                    if (baud_hw_setting.b2.uart_pll_sel != baud.b2.uart_pll_sel)
                    {
                        if (!g_data_uart_settings.tune_pll_sel_en)
                            goto_afe_conflict = 1;
                        else
                        {
                            //1 Note Make sure that it can't impact SPI clock
                             /* modify pll_sel setting, should not affect the SPI path */
                             /* short uart_clk glitch is expected to be less than 0.1us*/
                             /* so g_hci_uart_en0_before_set_en = 1 is recommended */
                             bton_uart_spi_afe_ctrl_reg.b.sel = baud.b2.uart_pll_sel;
                             VENDOR_WRITE(BTON_UART_SPI_AFE_CTRL_REG,bton_uart_spi_afe_ctrl_reg.d32);
                             DMA_UART_LOG(WHITE, DMA_UART_022, 1, bton_uart_spi_afe_ctrl_reg.d32);                        
                        }
                    } /* end of if(baud_hw_setting.b2.uart_pll_sel != baud.b2.uart_pll_sel) */
                } /* end of if(re_init_flag == 0) */
            }
            else /* else of if(baud_hw_setting.b2.uart_pll_d ==  baud.b2.uart_pll_d) */
            {
                if(!re_init_flag)
                    goto_afe_conflict = 1;
            }
        }/* end of if(baud.b2.uart_clk_sel != 0) */
    }
    else /* else of if(spi_mode != 0) */
#endif    
    {
#ifndef _8821A_NEW_UART_DESIGN_    
        if (baud.b2.uart_clk_sel)
        {
            if (!re_init_flag)
            {
                // TODO: Need to make sure WiFi/AFE/Bandgap is ON
                // TODO: Need to make sure 3.3V and 1.5V is ON
                //1 Add the golabl variable for power option setting ?
                
                /* AFE Initialization by UART Init function */
                /* Need to make sure WiFi/AFE/Bandgap is ON */
                /* Need to make sure 3.3V and 1.5V is ON */
                /* LDO ON, PLL setting, PLL OFF */
                trun_on_off_uart_spi_pll(PLL_ON, baud);
                pll_ckrdy_timeout = wait_uart_spi_pll_ready();
            }
        }
        else /* else of if(baud.b2.uart_clk_sel != 0) */
#endif        
        {
#ifndef _8821A_NEW_UART_DESIGN_        
            if (re_init_flag == 0)
            {
                if (g_data_uart_settings.afe_turn_off_en)
                {
#ifdef _DBG_UART_FORCE_SETTINGS_
//                    bton_uart_spi_afe_ctrl_reg.b.pow_ldo = 1;
#endif
                    if ((bton_uart_spi_afe_ctrl_reg.b.pow_pll) || 
                        (bton_uart_spi_afe_ctrl_reg.b.pow_ldo))
                    {
                        /* Before Turn Off LDO and PLL, should make sure 
                        that isolation between analog/digital is done */
                        trun_on_off_uart_spi_pll(PLL_OFF, baud);
                        DMA_UART_LOG(WHITE, DMA_UART_022, 1, bton_uart_spi_afe_ctrl_reg.d32);                        
                    }
                } /* end of if(g_hci_uart_afe_turn_off_en) */
            } /* end of if(re_init_flag == 0) */
#endif            
        } /* end of if(baud.b2.uart_clk_sel != 0) */
    }

#ifndef _8821A_NEW_UART_DESIGN_    
    if( (goto_afe_conflict != 0) || (pll_ckrdy_timeout != 0) )
    {
        if(goto_afe_conflict != 0)
        {
        }
        else if(pll_ckrdy_timeout != 0) 
        {
        }
        DMA_UART_LOG(RED, DMA_UART_020, 2, goto_afe_conflict, pll_ckrdy_timeout);
        return API_FAILURE;
    }
    else
#endif    
    {
        if (baud.b2.uart_clk_sel)
        {
            trun_on_off_uart_spi_pll(PLL_ON);
        }
    
        dma_uart_software_reset_reg.d8 = VENDOR_BYTE_READ(DMA_UART_SOFEWARE_RESET_REG);
#ifdef _8821A_NEW_UART_DESIGN_        
        dma_uart_software_reset_reg.b.uart_hci_sel = baud.b2.uart_clk_sel;
#else
        dma_uart_software_reset_reg.b.uart_hci_sel = baud.b2.uart_clk_sel;
#endif
        // TODO: double-confirm with Kevin
        dma_uart_software_reset_reg.b.uart_hci_en = 1;
        if(uart_reset_flag) 
        {
            dma_uart_software_reset_reg.b.uart_hci_rst = 1;
            VENDOR_BYTE_WRITE(DMA_UART_SOFEWARE_RESET_REG, dma_uart_software_reset_reg.d8);
            DMA_UART_DBG_LOG(BLUE, DMA_UART_003_DBG_HEX, 1, dma_uart_software_reset_reg.d8);
            dma_uart_software_reset_reg.b.uart_hci_rst = 0;
            VENDOR_BYTE_WRITE(DMA_UART_SOFEWARE_RESET_REG, dma_uart_software_reset_reg.d8);
            DMA_UART_DBG_LOG(BLUE, DMA_UART_003_DBG_HEX, 1, dma_uart_software_reset_reg.d8);
        }
        else
        {
            dma_uart_software_reset_reg.b.uart_hci_rst = 0;
            VENDOR_BYTE_WRITE(DMA_UART_SOFEWARE_RESET_REG, dma_uart_software_reset_reg.d8);
            DMA_UART_DBG_LOG(BLUE, DMA_UART_003_DBG_HEX, 1, dma_uart_software_reset_reg.d8);
        }

#ifndef _UART_H5        
        /* set UART_MODEM_CTL_REG_OFF as 0xa0 to enable flow control */
        UART_DWORD_WRITE(UART_MODEM_CTL_REG_OFF, 0xa0);
#else  
        hci_uart_line_ctrl_config();
/* the following code is moved to higher level hierarchy: hci_uart_reset_init_RTL8723()
        if (g_data_uart_settings_2.h5_en)
        {
            hci_uart_set_h5_en(0);
            hci_uart_h5_config_init();
            hci_uart_set_h5_en(1);
        }
        else
        {
            hci_uart_set_h5_en(0);
        }
*/        
#endif


        /* set UART to the desired state */
        divisor = baud.b2.uart_divisor;
        ovsr = baud.b2.uart_ovsr;
#ifdef _8821A_NEW_UART_DESIGN_        
        ovsr_adj = baud.b2.uart_ovsr_adj;
#endif        
#ifdef _UART_H5        
#ifndef _8821A_NEW_UART_DESIGN_
        hci_uart_divisor_ovsr_config(divisor,ovsr);
#else
        hci_uart_divisor_ovsr_config(divisor,ovsr,ovsr_adj);
#endif
#else
        dll = divisor & 0xff;
        dlh = (divisor & 0xff00)>>8;
        /* Note: DLL and DLH should be programmed when DLAB = 1 */
#ifdef _UART_H5        
        dma_uart_line_ctl_reg.d8 = (UINT8)UART_DWORD_READ(DMA_UART_LINE_CTL_REG_OFF);
        dma_uart_line_ctl_reg.b.dlab = 1;
        UART_DWORD_WRITE(DMA_UART_LINE_CTL_REG_OFF, dma_uart_line_ctl_reg.d8);
#else        
        UART_DWORD_WRITE(DMA_UART_LINE_CTL_REG_OFF, DMA_UART_LINE_CTL_REG_DLAB1);
#endif        
        UART_DWORD_WRITE(DMA_UART_DLL_OFF, dll);
        UART_DWORD_WRITE(DMA_UART_DLH_OFF, dlh);
        UART_DWORD_WRITE(DMA_UART_STS_REG, sts);
#ifdef _UART_H5        
        dma_uart_line_ctl_reg.b.dlab = 0;
        UART_DWORD_WRITE(DMA_UART_LINE_CTL_REG_OFF, dma_uart_line_ctl_reg.d8);
#else
        UART_DWORD_WRITE(DMA_UART_LINE_CTL_REG_OFF, DMA_UART_LINE_CTL_REG_DLAB0);
#endif        
        DMA_UART_DBG_LOG(BLUE, DMA_UART_002_DBG_DEC, 1, dll);
        DMA_UART_DBG_LOG(BLUE, DMA_UART_002_DBG_DEC, 1, dlh);
        DMA_UART_DBG_LOG(BLUE, DMA_UART_002_DBG_DEC, 1, sts);
#endif
        
        
        baudrate_hz = hci_uart_calculate_baudrate_hz(baud);

#ifdef _8821A_NEW_UART_DESIGN_    
#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
        if(sleep_mode_param.dlps_restore_flow == FALSE)
#endif
        {
            RT_BT_LOG(GREEN, DMA_UART_131, 3, baudrate_hz, baud.d32, re_init_flag);
        }
#else
        RT_BT_LOG(GREEN, DMA_UART_041, 2, baudrate_hz, baud.d32);
#endif        
#ifdef _UART_BAUD_ESTIMATE_ 
//        g_baud_d32_setting_record = baud.d32;
        hci_uart_man_sram.baud_current_setting = baud;
#endif        
        return API_SUCCESS;
    } /* end of if( (goto_afe_conflict != 0) || (pll_ckrdy_timeout != 0) ) */
}




#ifdef _UART_H5
API_RESULT hci_uart_set_baud_detect_mode(UINT16 re_init_flag, UINT8 uart_reset_flag)
{
    API_RESULT uart_set_result;
    BAUDRATE_FORMAT_TYPE baud;
    baud.b.d24 = BUAD_FORMAT_DETECTION;
    uart_set_result = uart_set_baud_clk_RTL8723(baud, re_init_flag, uart_reset_flag);
    hci_uart_set_hci_mode_en(1);        
            
    // check if success; Note: it should be success
    if (uart_set_result == API_FAILURE)
        DMA_UART_LOG(YELLOW, DMA_UART_010, 1, baud.d32);

    // enable baudrate detection
    // set DLAB bit to 1
    UART_DWORD_WRITE(DMA_UART_STS_REG, UART_STS_REG_BAUD_DET_EN0);
    UART_DWORD_WRITE(DMA_UART_STS_REG, UART_STS_REG_BAUD_DET_EN1);   
    return uart_set_result;
}



#ifdef _8821A_BTON_DESIGN_
void hci_uart_baud_record_to_bton(UINT32 baud_d32)
{
    BTON_UART_INFO_REG_S_TYPE bton_uart_info_reg;
    bton_uart_info_reg.d32 = VENDOR_READ(BTON_UART_INFO_REG);

    bton_uart_info_reg.b.reg_uart_info_baud_d28 = baud_d32;
//    if (g_data_uart_settings.wr_bton_en)
    {
        VENDOR_WRITE(BTON_UART_INFO_REG,bton_uart_info_reg.d32);
        DMA_UART_LOG(WHITE, DMA_UART_129, 1, bton_uart_info_reg.d32);
    }    
}
UINT32 hci_uart_read_bton_baud_record(void)
{
    BTON_UART_INFO_REG_S_TYPE bton_uart_info_reg;
    bton_uart_info_reg.d32 = VENDOR_READ(BTON_UART_INFO_REG);
    DMA_UART_LOG(YELLOW, DMA_UART_128, 1, bton_uart_info_reg.d32);    
    return bton_uart_info_reg.b.reg_uart_info_baud_d28;
}
#endif


// TODO: Need to be patchable;  Need to double-confirm the flow
// TODO: Need to set H5 timing parameters? 
// TODO: Need H5 go_idle?
// TODO: Need uart reset? re-init?
/* prefer to be executed after warm reset event H5 ACK
  * return API_FAILURE: not UART interface or fail to set new baudrate
  * return API_SUCCESS: succeed setting new baudrate 
  */
API_RESULT hci_uart_vendor_set_baud(BAUDRATE_FORMAT_TYPE baud_new)
{
#ifdef _8821A_BTON_DESIGN_
    //BTON_UART_INFO_REG_S_TYPE bton_uart_info_reg;
#else
    BTON_INTERFACE_CTRL_REG_S_TYPE bton_intf_ctrl_reg;    
#endif
    BAUDRATE_FORMAT_TYPE baud_old; 

#ifdef _8821A_BTON_DESIGN_    
    //bton_uart_info_reg.d32 = VENDOR_READ(BTON_UART_INFO_REG);
#else
    bton_intf_ctrl_reg.d32 = VENDOR_READ(BTON_INTERFACE_CTRL_REG);
#endif
    UINT32 baudrate_hz;

#ifdef _ROM_CODE_PATCHED_    
    /* Reserved com code patch by yilinli:
      * e.g. 
      *    1. 
      */    
    if (rcp_hci_uart_vendor_set_baud_func != NULL)
    {
        return rcp_hci_uart_vendor_set_baud_func((void*)(&baud_new));
    }   
#endif

    baud_old = hci_uart_read_hw_setting();

    if (g_fun_interface_info.b.bt_interface != UART_INTERFACE)
        return API_FAILURE;
    
    // TODO: if controller is spi mode, fw need to jump sram code for changing PLL
    
    
    DMA_UART_LOG(RED, DMA_UART_036, 2, baud_new.d32, baud_old.d32);
    
    /* call uart_set_baud_clk_RTL8723 with re_init_flag = 0 and uart_reset_flag = 0 */
    if (uart_set_baud_clk_RTL8723(baud_new, 0, 0) == API_FAILURE)
    {
        DMA_UART_LOG(RED, DMA_UART_037, 0, 0);
        /* call uart_set_baud_clk_RTL8723 with re_init_flag = 0 and uart_reset_flag = 0 */
        if (uart_set_baud_clk_RTL8723(baud_old, 0, 0) == API_FAILURE)
        {
            /* Unexpected to happen */
            /* For safe, re-call the hci_uart_reset_init_RTL8723 to restore */
            DMA_UART_LOG(RED, DMA_UART_038, 1, 1); 
            // TODO: add log; Need to write to BTON?
            // TODO: execute or not?
//            hci_uart_reset_init_RTL8723(1);
        }
        return API_FAILURE;
    }
    else
    {
        DMA_UART_LOG(RED, DMA_UART_039, 0, 0);    
        // TODO: Need to write to BTON


#ifdef _8821A_BTON_DESIGN_    
        if (g_data_uart_settings.wr_bton_en)
        {
            hci_uart_baud_record_to_bton(baud_new.b.d24);
        }
#else        
        bton_intf_ctrl_reg.b.uart_baudrate_record = baud_new.b.d24;
        if (g_data_uart_settings.wr_bton_en)
        {
            VENDOR_WRITE(BTON_INTERFACE_CTRL_REG,bton_intf_ctrl_reg.d32);
            DMA_UART_LOG(WHITE, DMA_UART_009, 1, bton_intf_ctrl_reg.d32);
        }
#endif            
        
        // TODO: call hci_uart_h5_timing_adapt()?? or in uart_set_baud_clk_RTL8723() ??
        if (g_data_uart_settings_2.h5_en && g_data_uart_settings_3.h5_resend_time_adapt_en)
        {
            baudrate_hz = hci_uart_calculate_baudrate_hz(baud_new);
            hci_uart_h5_timing_adapt(baudrate_hz); // TODO: need any other method to return counter to zero?
        }        
        return API_SUCCESS;
    }
}
#endif


#ifdef _UART_BAUD_ESTIMATE_
void hci_uart_baud_est_log(void)
{
    DMA_UART_H5_INTSTS_REG_S_TYPE uart_h5_intsts;
    DMA_UART_H5_INTEN_REG_S_TYPE uart_h5_inten;
    DMA_UART_TOGGLE_MON_CTRL_REG_S_TYPE uart_toggle_mon;

    DEF_CRITICAL_SECTION_STORAGE;
    if (!g_efuse_baud_est_setting_1.baud_mon_en)
    {
        return;
    }

    uart_h5_intsts.d32 = UART_DWORD_READ(DMA_UART_H5_INTSTS);
    uart_h5_inten.d32 = UART_DWORD_READ(DMA_UART_H5_INTEN);
    uart_toggle_mon.d32 = UART_DWORD_READ(DMA_UART_TOGGLE_MON_CTRL);

    if (g_data_uart_settings.baud_mon_log_en)
    {
        RT_BT_LOG(GREEN, DMA_UART_161, 10, 
                            uart_h5_intsts.d32, 
                            uart_h5_inten.d32,
                            uart_toggle_mon.d32,
//                            hci_uart_man_sram.hci_uart_man_sram_valid_signature,
                            VENDOR_READ(BTON_A0_REG),
                            hci_uart_man_sram.baud_current_setting.d32,
                            hci_uart_man_sram.hci_uart_baud_est_state,
                            hci_uart_man_sram.hci_uart_baud_est_intr_cnt,
                            hci_uart_man_sram.est_min_low_period_record,
                            hci_uart_man_sram.est_min_falling_space_record,
                            uart_toggle_mon.b.uart_min_low_period);
    }
                        
    if (g_data_uart_settings.baud_mon_resume_every_log)
    {
        MINT_OS_ENTER_CRITICAL();    
        if (hci_uart_man_sram.hci_uart_baud_est_state == HCI_UART_BAUD_EST_ST_STOP)
        {
            hci_uart_set_toggle_mon_en(0);
            hci_uart_set_toggle_mon_en(1);
        }
        MINT_OS_EXIT_CRITICAL();
    }

    EFUSE_POW_SETTING_3_S efuse_pow_setting_3;
    *(UINT8*)&efuse_pow_setting_3 = otp_str_data.efuse_pow_setting_3_d8;
    if (efuse_pow_setting_3.enable_gpio_power_on_check_at_est_log)
    {
        gpio_power_on_check();
    }
}                        

void hci_uart_baud_est_init_preprocessing(UINT16 re_init_flag) // TODO: to be completed
{

#ifdef _ROM_CODE_PATCHED_    
    // (yilinli) to reserved maximum flexibility after PATCH //
    if (rcp_hci_uart_baud_est_init_preprocessing_func != NULL)
    {
        if (rcp_hci_uart_baud_est_init_preprocessing_func((void*)&re_init_flag))
        {
            return;
        }
    }   
#endif

    if (!g_efuse_baud_est_setting_1.baud_est_en)
    {
        hci_uart_man.baud_sram_recov_en = 0;
        hci_uart_man_sram.hci_uart_baud_est_state = HCI_UART_BAUD_EST_ST_STOP;
    }
    else
    {
        BTON_A0_REG_S_TYPE bton_a0_reg;
        bton_a0_reg.d32 = VENDOR_READ(BTON_A0_REG);
        
        hci_uart_man.baud_sram_recov_en = 0;    
        if (bton_a0_reg.is_hci_uart_man_sram_valid)
        {
            // to force baudrate recovery when HCI_UART_BAUD_EST_ST_STOP //
            if ( (hci_uart_man_sram.hci_uart_baud_est_state == HCI_UART_BAUD_EST_ST_STOP) && g_efuse_baud_est_setting_1.baud_recov_at_state_stop_preset)
            {
                hci_uart_man.baud_sram_recov_en = 1;
            }
        
            if (re_init_flag)  // TODO: to be confirmed, May be executed when UART H4 Error Recovery!
            {
                hci_uart_man.baud_sram_recov_en = g_efuse_baud_est_setting_1.baud_recov_at_reinit;
            }
            // Decision of baud_sram_recov_en for different WatchDog Timeout conditions //
            else if (IS_H5_LINKRESET)
            {
                hci_uart_man.baud_sram_recov_en = g_efuse_baud_est_setting_1.baud_recov_at_h5_linkreset;
            }
            else if (IS_FW_TRIG_WDG_TO)
            {
                hci_uart_man.baud_sram_recov_en = g_efuse_baud_est_setting_1.baud_recov_at_fw_trig_wdg;
            }
            else
            {
                hci_uart_man.baud_sram_recov_en = g_efuse_baud_est_setting_1.baud_recov_at_other_wdg;
            }

            // to force baudrate recovery when HCI_UART_BAUD_EST_ST_STOP //
            if ( (hci_uart_man_sram.hci_uart_baud_est_state == HCI_UART_BAUD_EST_ST_STOP) && g_efuse_baud_est_setting_1.baud_recov_at_state_stop_postset)
            {
                hci_uart_man.baud_sram_recov_en = 1;
            }
            // TODO: how about HCI_UART_BAUD_EST_ST_TRIAL

            if (hci_uart_man.baud_sram_recov_en == 0)
            {
                if (g_efuse_baud_est_setting_1.baud_est_restart_at_baud_sram_recov_0)
                {
                    hci_uart_man_sram.hci_uart_baud_est_state = HCI_UART_BAUD_EST_ST_START;
                    bton_a0_reg.is_hci_uart_man_sram_valid = 0;  // to re-start the baud-est //
                }
            }

            if (IS_H5_LINKRESET)
            {
                if (g_efuse_baud_est_setting_1.baud_est_restart_at_h5_linkreset)
                {
                    hci_uart_man_sram.hci_uart_baud_est_state = HCI_UART_BAUD_EST_ST_START;
                    bton_a0_reg.is_hci_uart_man_sram_valid = 0; // to re-start the baud-est //
                }
            }

            VENDOR_WRITE(BTON_A0_REG, bton_a0_reg.d32);
        }
        else
        {
            hci_uart_man_sram.hci_uart_baud_est_state = HCI_UART_BAUD_EST_ST_START;
        }
    }

}

// NOTE: need to be executed in CRITICAL_SECTION or ISR or before InterruptEnable
void hci_uart_baud_est_init_postprocessing(UINT16 re_init_flag) // TODO: to be completed
{

#ifdef _ROM_CODE_PATCHED_    
    // (yilinli) to reserved maximum flexibility after PATCH //
    if (rcp_hci_uart_baud_est_init_postprocessing_func != NULL)
    {
        if (rcp_hci_uart_baud_est_init_postprocessing_func((void*)&re_init_flag))
        {
            return;
        }
    }   
#endif

    if (re_init_flag) // for H4
    {
        if (!g_efuse_baud_est_setting_1.exec_baud_est_init_post_at_reinit)
        {
            return;
        }
    }

    hci_uart_set_falling_cnt_th((g_efuse_baud_est_setting_2.baud_est_falling_cnt_th_1st)<<2);
    if (hci_uart_man_sram.hci_uart_baud_est_state == HCI_UART_BAUD_EST_ST_STOP)
    {
        if (g_efuse_baud_est_setting_1.baud_mon_en)
        {
            hci_uart_set_toggle_mon_en(0);
            hci_uart_set_toggle_mon_en(1);
        }
        else
        {

        }
    }
    else
    {
        hci_uart_set_toggle_mon_en(0);
        hci_uart_set_toggle_mon_en(1);   
        hci_uart_set_toggle_mon_en(0); // clear and hold falling cnt; reset uart_toggle_min_xxx
        hci_uart_set_toggle_mon_en(1);
        hci_uart_set_falling_cnt_intr_en(1);
    }


    BTON_A0_REG_S_TYPE bton_a0_reg;
    bton_a0_reg.d32 = VENDOR_READ(BTON_A0_REG);    
    if (bton_a0_reg.is_hci_uart_man_sram_valid)
    {
    }
    else
    {
        hci_uart_man_sram.est_min_low_period_record = 0xffff;
        hci_uart_man_sram.est_min_falling_space_record = 0xffff;
        hci_uart_man_sram.hci_uart_baud_est_intr_cnt = 0; // TODO: 
        hci_uart_man_sram.hci_vendor_uart_sync_received = 0;
        hci_uart_man_sram.hci_vendor_set_baud_received = 0; 
        hci_uart_man_sram.hci_vendor_set_baud_executed = 0;
        bton_a0_reg.is_hci_uart_man_sram_valid = 1;
    }
    VENDOR_WRITE(BTON_A0_REG, bton_a0_reg.d32);    
    
    hci_uart_man_sram.baud_est_original_baud_det_done = 0; // used when baud_det_en = 1 and baud_det is going-on //
    
}

BAUDRATE_FORMAT_TYPE hci_uart_baud_recovery(UINT16 re_init_flag)
{
    BAUDRATE_FORMAT_TYPE baud;
    API_RESULT uart_set_result;

#ifdef _ROM_CODE_PATCHED_
    // (yilinli) to reserved maximum flexibility after PATCH //
    if (rcp_hci_uart_baud_recovery_func != NULL)
    {
        rcp_hci_uart_baud_recovery_func((void*)&baud, re_init_flag);
        return baud;
    }   
#endif
    
    baud = hci_uart_man_sram.baud_current_setting;
    uart_set_result = uart_set_baud_clk_RTL8723(baud, re_init_flag, 1);
    return baud;
}


#endif

/* 
*  function: API_RESULT hci_uart_goto_LPS_mode(void);
*  parameters: 
*  return value: API_SUCCESS or API_FAILURE
*/
/*
API_RESULT hci_uart_goto_LPS_mode(void)
{
    // TODO: To add LPS function after BaudRate Detection Timeout
    return API_SUCCESS;
}
*/

BAUDRATE_FORMAT_TYPE hci_uart_baud_init_original(UINT16 re_init_flag)
{
//    UINT16 sts_read, det_succ, det_fail, det_timeout;
    UINT32 det_times;  /* should use UINT32 to avoid Over-Flow */
    BAUDRATE_FORMAT_TYPE baud; //, baud_det;
#ifdef _8821A_BTON_DESIGN_ 
//    BTON_UART_INFO_REG_S_TYPE bton_uart_info_reg;
    UINT32 baud_bton_record_d32;
#else    
    BTON_INTERFACE_CTRL_REG_S_TYPE bton_intf_ctrl_reg;
#endif    
    API_RESULT uart_set_result = API_FAILURE;
    UINT16 exit_flag = 0;
    
    baud.d32 = 0;
    

#ifdef _8821A_BTON_DESIGN_
//    bton_uart_info_reg.d32 = VENDOR_READ(BTON_UART_INFO_REG);
    baud_bton_record_d32 =  hci_uart_read_bton_baud_record();
#else
    bton_intf_ctrl_reg.d32 = VENDOR_READ(BTON_INTERFACE_CTRL_REG);
    DMA_UART_LOG(YELLOW, DMA_UART_008, 1, bton_intf_ctrl_reg.d32);
#endif    
    // TODO: When RESETB Pin Reset is sensed, BTON/Baudrate_record should be reset to 0
    /* declare variables for FW execution flow */
    UINT16 goto_check_default_flag = 0;
#if 0 // this function is not used; marked to save ROM size    
    UINT16 goto_LPS_mode_flag = 0; /* only used in 'auto-det timeout' case */
#endif    
    /* determine if need to read BT_ON/baudrate_record */
#ifdef _8821A_BTON_DESIGN_
    if (((g_data_uart_settings.chk_bton_en) || (re_init_flag)) &&
        (baud_bton_record_d32))
#else
    if (((g_data_uart_settings.chk_bton_en) || (re_init_flag)) &&
        (bton_intf_ctrl_reg.b.uart_baudrate_record))
#endif        
    {
        DMA_UART_LOG(YELLOW, DMA_UART_006, 0, 0);
#ifdef _8821A_BTON_DESIGN_
        baud.b.d24 = baud_bton_record_d32;
#else
        baud.b.d24 = bton_intf_ctrl_reg.b.uart_baudrate_record;
#endif        
        uart_set_result = uart_set_baud_clk_RTL8723(baud, re_init_flag, 1);
        goto_check_default_flag = (uart_set_result==API_SUCCESS) ? 0 : 1;
        DMA_UART_DBG_LOG(YELLOW, DMA_UART_003_DBG_HEX, 1, baud.d32);
        DMA_UART_DBG_LOG(YELLOW, DMA_UART_002_DBG_DEC, 1, uart_set_result);
    }
    else
    {    
        /* check if need to do baudrate detection */ 
        if ((!g_data_uart_settings.baud_det_en) ||
            ((re_init_flag) && (!g_data_uart_settings.baud_redet_en)))
        {
            goto_check_default_flag = 1;
        }    
	    /* do baudrate detection */
	    else 
        {
            DMA_UART_LOG(YELLOW, DMA_UART_007, 0, 0);
                  
            // set baud and BT_ON/baudrate_record = 0
#ifdef _8821A_BTON_DESIGN_
            if (g_data_uart_settings.wr_bton_en)
            {
                hci_uart_baud_record_to_bton(0);
            }
#else
            bton_intf_ctrl_reg.b.uart_baudrate_record = 0;
            if (g_data_uart_settings.wr_bton_en)
            {
                VENDOR_WRITE(BTON_INTERFACE_CTRL_REG,bton_intf_ctrl_reg.d32);
                DMA_UART_LOG(WHITE, DMA_UART_009, 1, bton_intf_ctrl_reg.d32);
            }
#endif                
                    
            // set uart/AFE to desired state
#ifdef _UART_H5
#ifdef _8821A_BTON_DESIGN_
            // to avoid UART reset or NOT//
            uart_set_result = hci_uart_set_baud_detect_mode(re_init_flag, g_data_uart_settings.baud_det_init_with_reset);
#else
            uart_set_result = hci_uart_set_baud_detect_mode(re_init_flag, 1);
#endif            
#else
            baud.b.d24 = BUAD_FORMAT_DETECTION;
            uart_set_result = uart_set_baud_clk_RTL8723(baud, re_init_flag, 1);
            hci_uart_set_hci_mode_en(1);        
                    
            // check if success; Note: it should be success
            if (uart_set_result == API_FAILURE)
                DMA_UART_LOG(YELLOW, DMA_UART_010, 1, baud.d32);
    
            // enable baudrate detection
            // set DLAB bit to 1
            UART_DWORD_WRITE(DMA_UART_STS_REG, UART_STS_REG_BAUD_DET_EN0);
            UART_DWORD_WRITE(DMA_UART_STS_REG, UART_STS_REG_BAUD_DET_EN1);          
#endif
                    
                    
            det_times = 0;
#ifndef _8821A_BTON_DESIGN_            
            if (re_init_flag ? g_data_uart_settings.redet_wait_en : g_data_uart_settings.det_wait_en)
            {
                /* go to waiting loops for detection results */
                exit_flag = 0;
            
    			/* YL20110328, modify structure for NOT disabling DETECT when wait_en = 0 */
                while (exit_flag==0)
                {
                     /* wait for detection result or timeout */
                     sts_read = (UINT16)UART_DWORD_READ(DMA_UART_STS_REG);
                              
                     det_succ = (sts_read & 0x4)>>2;
                     det_fail = (sts_read & 0x2)>>1;

    			     if ((g_data_uart_settings.baud_det_timeout_en) &&  /* YL20110328 */
                        ((++det_times)>=BAUD_DET_TIMEOUT_LOOPS))
                        det_timeout = 1;
                     else
                        det_timeout = 0;

    			         //1 For Debug?  YES! Debugging Log   /* YL20110328 */
    			     if ( ((det_times % 100000) == 1) && (det_times<1000000)) /* YL20110328 */
                     {
                         DMA_UART_LOG(RED, DMA_UART_011, 2, det_times, BAUD_DET_TIMEOUT_LOOPS);
                     }

                     if (det_succ)
                     {
                         exit_flag = 1;
                         baud_det = hci_uart_read_hw_setting();
                         baud = baud_det;
#ifdef _UART_BAUD_ESTIMATE_ 
                        hci_uart_man_sram.baud_current_setting = baud;
#endif
    //                  baud.b2.uart_divisor = baud_det.b2.uart_divisor;
    //                  baud.b2.uart_ovsr = baud_det.b2.uart_ovsr;
                         goto_check_default_flag = 0;
                         DMA_UART_LOG(GREEN, DMA_UART_029, 0, 0);
                     }
                     else if (det_fail)
                     {
                         exit_flag = 1;
                         goto_check_default_flag = 1;              
                         DMA_UART_LOG(RED, DMA_UART_030, 0, 0);
                     }
                     else if (det_timeout)
                     {
                         exit_flag = 1;
                         goto_check_default_flag = 1;
#if 0 // this function is not used; marked to save ROM size                         
//                         goto_LPS_mode_flag = g_data_uart_settings.det_timeout_LPS_en ? 1 : 0;
                         goto_LPS_mode_flag = 0;
#endif                         
                         DMA_UART_LOG(RED, DMA_UART_031, 0, 0);
                     }
                 } /* end of while(exit_flag==0) */
    			 //Disable uart auto-detectt
    			 UART_DWORD_WRITE(DMA_UART_STS_REG, UART_STS_REG_BAUD_DET_EN0);                
                 RT_BT_LOG(YELLOW, DMA_UART_043, 2, det_times, baud.d32);
            }
            else
#endif            
            {
                /* bypass the waiting loops and return */
                /* the UART IP would keep receiving according to the ovsr and divisor, even if detection fails */
                /* if the final Baudrate is not correct, then UART indicator(0x01, 0x02, 0x03) error interrupt is expected */
                /* to be tested more! */
                // TODO: to be tested more!
                exit_flag = 1;                    
             } /* end of else of if (re_init_flag ? g_data_uart_settings.redet_wait_en : g_data_uart_settings.det_wait_en) */
//             RT_BT_LOG(YELLOW, DMA_UART_012, 4, det_times, goto_check_default_flag, goto_LPS_mode_flag, baud.d32);
           /* YL20110328, modify structure for NOT disabling DETECT when wait_en = 0 */

         } /* else end of if( g_data_uart_settings.baud_det_en == 0 ) */
    } /* else end of if(   ((g_data_uart_settings.chk_bton_en != 0) || (re_init_flag != 0))   &&    (baud_bton_record_d32 != 0)   ) */

     // TODO: Add LOG Here
	 
    DMA_UART_LOG(YELLOW, DMA_UART_013, 4, goto_check_default_flag, 
//                                           goto_LPS_mode_flag, 
                                            0, 
                                           g_data_uart_settings.baud_default_en, g_baudRate);
     
    if (goto_check_default_flag)
    {
        if (g_data_uart_settings.baud_default_en)
        {
            baud.b.d24= BUAD_FORMAT_DEFAULT;
        }
        else
        {
            baud.b.d24 = g_baudRate; /* format translating from INT to BAUD_FORMAT_TYPE */
        }

#if 0
#ifdef _8821A_BTON_DESIGN_ // TODO: to be reiewed/modified
        if (IS_FW_TRIG_WDG_TO)
        {
#ifdef _UART_BAUD_ESTIMATE_
            if (g_data_uart_settings.baud_recov_at_fw_trig_wdg)
            {
                baud.b.d24 = hci_uart_man.baud_current_setting.b.d24;
            }
            hci_uart_man.baud_est_en = g_data_uart_settings.baud_est_en_at_fw_trig_wdg;
#endif                
        }
#endif
#endif

        uart_set_result = uart_set_baud_clk_RTL8723(baud, re_init_flag, 1);
         
        if(uart_set_result == API_FAILURE)
        {
            baud.d32= BUAD_FORMAT_DEFAULT;
            uart_set_result = uart_set_baud_clk_RTL8723(baud, re_init_flag, 1);
            if(uart_set_result == API_FAILURE)
            {
                 // TODO: LOG ??
            }
        }
         
        if(uart_set_result == API_SUCCESS)
        {
#ifdef _8821A_BTON_DESIGN_
            if (g_data_uart_settings.wr_bton_en)
            {
                hci_uart_baud_record_to_bton(baud.b.d24);
            }
#else
            bton_intf_ctrl_reg.b.uart_baudrate_record = baud.b.d24;
            if (g_data_uart_settings.wr_bton_en)
            {
                VENDOR_WRITE(BTON_INTERFACE_CTRL_REG,bton_intf_ctrl_reg.d32);
                DMA_UART_LOG(WHITE, DMA_UART_009, 1, bton_intf_ctrl_reg.d32);
            }
#endif                                     
        }
    } /* end of if(goto_check_default_flag != 0) */

    return baud;
}

/*
*  function: BAUDRATE_FORMAT_TYPE  hci_uart_reset_init_RTL8723(UINT16 re_init_flag);
*  parameters: re_init_flag (0: first time; 1: re-initialization, check bton first and do not modify AFE setting
*  return value: the final baud setting (type: BAUDRATE_FORMAT_TYPE)
*/
BAUDRATE_FORMAT_TYPE  hci_uart_reset_init_RTL8723(UINT16 re_init_flag)
{
    // TODO: review SPI mode operation
    // TODO: review LPS/SUS clock on/off processing
    // TODO: review clock gating
    // TODO: review baud bit number and corresponding data structure
 
    BAUDRATE_FORMAT_TYPE baud;
    

#ifdef _UART_BAUD_ESTIMATE_ // TODO: to be reviewed/modified
    hci_uart_baud_est_init_preprocessing(re_init_flag);
    if (hci_uart_man.baud_sram_recov_en)
    {
        baud = hci_uart_baud_recovery(re_init_flag);  // Also executes UART reset //                 
    }
    else
#endif                
    {
        baud = hci_uart_baud_init_original(re_init_flag);  // Also executes UART reset //
    }


#ifndef _8821A_BTON_DESIGN_
    bton_intf_ctrl_reg.d32 = VENDOR_READ(BTON_INTERFACE_CTRL_REG);
//    RT_BT_LOG(GREEN, DMA_UART_027, 1, bton_intf_ctrl_reg.d32);
    DMA_UART_LOG(GREEN, DMA_UART_027, 1, bton_intf_ctrl_reg.d32);
#endif

#ifdef _ROM_CODE_PATCHED_
    /* Reserved com code patch by yilinli:
      * e.g. 
      *    1. replace the following setting
      */    
    if (rcp_hci_uart_h4h5_config_func != NULL)
    {
        if(rcp_hci_uart_h4h5_config_func((void*)(&baud)))
        {
            return baud;
        }
    }        
#endif

#ifdef _UART_H5
    hci_uart_set_h5_en(0);
    if (g_data_uart_settings_2.h5_en)
    {
        hci_uart_h5_restore(); /* including H5 manager initializatioin */
        hci_uart_h5_config_init();
        if (g_data_uart_settings_3.h5_resend_time_adapt_en)
        {
            UINT32 baudrate_hz = hci_uart_calculate_baudrate_hz(baud);
            hci_uart_h5_timing_adapt(baudrate_hz);
        }
        /* non-BTON variable initialization */
        // TODO: required to be saved in BTON?
        hci_uart_set_h5_en(1); // TODO: required?
//        hci_uart_set_hci_mode_en(1);
    }
    else
    {
        hci_uart_set_h4_error_interrupt_en(g_data_uart_settings.h4_err_intr_en);
    }
    
#ifdef _8821A_BTON_DESIGN_
    if (g_data_uart_settings.uart_set_flow_control_after_hci_en == 0)
    {
        hci_uart_set_mcr_rtsn(g_data_uart_settings.hci_uart_mcr_rtsn);
    }
#endif    
#endif
    hci_uart_man.h5_retry_alarm_state = 0;
    hci_uart_man.h5_retry_fail_state = 0;

    if (h5_poll_wake_timer_id != NULL)
    {
        if (IN_ISR())
        {
            BaseType_t high_pri_task_woken = pdFALSE;
            xTimerPendFunctionCallFromISR(
                    (PendedFunction_t) os_wrapper_delete_timer,
                    &h5_poll_wake_timer_id, 0, &high_pri_task_woken);
            portYIELD_FROM_ISR(high_pri_task_woken);
        }
        else
        {
            OS_DELETE_TIMER(&h5_poll_wake_timer_id);
        }
    }
    
    hci_uart_man.h5_trx_active_mask = g_data_uart_settings_3.h5_trx_active_mask_opt ? 
                                        DMA_ISR_H5_TRX_ACTIVE_CHECK_MASK1 : DMA_ISR_H5_TRX_ACTIVE_CHECK_MASK0;

    hci_uart_man.h4_err_event_wait_rx_state = 0;
    hci_uart_man.h4_err_event_wait_recov_timer_state = 0;
    hci_uart_man.h4_hw_err_event_code = g_data_uart_settings.err_event_err_code_opt ? 
                                        HARDWARE_FAILURE_ERROR_RTK_H4 : HARDWARE_FAILURE_ERROR;

    hci_uart_man.baud_new_valid_flag = 0;
    hci_uart_man.chg_para_valid_flag = 0;
    hci_uart_man.baud_new.d32= 0;

#ifdef _UART_BAUD_ESTIMATE_    // TODO: !!!! To be completed !!!!
    hci_uart_man.baud_est_h5_state_record = 0xFF;

    hci_uart_baud_est_init_postprocessing(re_init_flag);
#endif

    hci_uart_set_hci_mode_en(1);  // TODO: to be reviewed
    // to keep the flow control setting as "STOP" before HCI MODE is ready, it is important to H4 //
    if (g_data_uart_settings.uart_set_flow_control_after_hci_en)
    {
        hci_uart_set_mcr_rtsn(g_data_uart_settings.hci_uart_mcr_rtsn);
        hci_uart_set_hw_fctrl_on(g_data_uart_settings_3.hw_fctrl_on);
    }

    return baud;
    
}

#if 0
void trun_on_off_ck120m(UINT8 on_off)
{
    CORE_AND_AFE_LDO_CTRL_S_TYPE bton_core_afe_ldo_ctrl_reg;    
    bton_core_afe_ldo_ctrl_reg.d32 = VENDOR_READ(BTON_CORE_AFE_LDO_CTRL_REG);
    if (on_off)
    {
        bton_core_afe_ldo_ctrl_reg.b.en_ck120m = 1;
    }
    else
    {
        bton_core_afe_ldo_ctrl_reg.b.en_ck120m = 0;
    }  
    VENDOR_WRITE(BTON_CORE_AFE_LDO_CTRL_REG, bton_core_afe_ldo_ctrl_reg.d32);
}
#endif
#ifdef _8821A_BTON_DESIGN_

UINT32 get_uint32_from_uint8_array(UINT8 *uint8_array)
{
    return ((UINT32)(uint8_array[0] | 
                   (uint8_array[1]<<8) |
                   (uint8_array[2]<<16) | 
                   (uint8_array[3]<<24)));
}

void trun_on_off_uart_spi_pll(UINT8 on_off)
#else
void trun_on_off_uart_spi_pll(UINT8 on_off, BAUDRATE_FORMAT_TYPE uart_pll_ctrl)
#endif
{
#ifdef _8821A_BTON_DESIGN_

#ifdef _NEW_BTON_DESIGN_AFTER_RTL8821B_TC_
    BTON_74_REG_S_TYPE bton_74;    
    bton_74.d32 = VENDOR_READ(BTON_74_REG);
    if (on_off)
    {
        bton_74.reg_ck_120m_en = 1;
    }
    else
    {
        bton_74.reg_ck_120m_en = 0;
    }  
    VENDOR_WRITE(BTON_74_REG, bton_74.d32);
#else
    CORE_AND_AFE_LDO_CTRL_S_TYPE bton_core_afe_ldo_ctrl_reg;    
    bton_core_afe_ldo_ctrl_reg.d32 = VENDOR_READ(BTON_CORE_AFE_LDO_CTRL_REG);
    if (on_off)
    {
        bton_core_afe_ldo_ctrl_reg.b.en_ck120m = 1;
    }
    else
    {
        bton_core_afe_ldo_ctrl_reg.b.en_ck120m = 0;
    }  
    VENDOR_WRITE(BTON_CORE_AFE_LDO_CTRL_REG, bton_core_afe_ldo_ctrl_reg.d32);
#endif

#else
    BTON_UART_SPI_AFE_CTRL_REG_S_TYPE bton_uart_spi_afe_ctrl_reg;
    CORE_AND_AFE_LDO_CTRL_S_TYPE bton_core_afe_ldo_ctrl_reg;
    UINT32 i = 0;
    
    if (on_off)
    {
        bton_uart_spi_afe_ctrl_reg.d32 = VENDOR_READ(BTON_UART_SPI_AFE_CTRL_REG);
        bton_uart_spi_afe_ctrl_reg.b.pow_ldo = 1;
        bton_uart_spi_afe_ctrl_reg.b.pow_pll = 0;
        bton_uart_spi_afe_ctrl_reg.b.d = uart_pll_ctrl.b2.uart_pll_d;
        bton_uart_spi_afe_ctrl_reg.b.sel = uart_pll_ctrl.b2.uart_pll_sel;
        VENDOR_WRITE(BTON_UART_SPI_AFE_CTRL_REG,bton_uart_spi_afe_ctrl_reg.d32);
        DMA_UART_LOG(WHITE, DMA_UART_022, 1, bton_uart_spi_afe_ctrl_reg.d32);   

        /* Wait for LDO stable, ~10us, 200us is recommended */
        for(i=0; i< (PLL_LDO_STABLE_LOOPS*10); i++);

        bton_core_afe_ldo_ctrl_reg.d32 = VENDOR_READ(BTON_CORE_AFE_LDO_CTRL_REG);
        bton_core_afe_ldo_ctrl_reg.b.uart_spi_pll_iso_en = 0;
        VENDOR_WRITE(BTON_CORE_AFE_LDO_CTRL_REG,bton_core_afe_ldo_ctrl_reg.d32);

        bton_uart_spi_afe_ctrl_reg.d32 = VENDOR_READ(BTON_UART_SPI_AFE_CTRL_REG);
        bton_uart_spi_afe_ctrl_reg.b.pow_pll = 1;
        VENDOR_WRITE(BTON_UART_SPI_AFE_CTRL_REG,bton_uart_spi_afe_ctrl_reg.d32);
        DMA_UART_LOG(WHITE, DMA_UART_022, 1, bton_uart_spi_afe_ctrl_reg.d32);  
    }
    else
    {
        bton_uart_spi_afe_ctrl_reg.d32 = VENDOR_READ(BTON_UART_SPI_AFE_CTRL_REG);
        bton_uart_spi_afe_ctrl_reg.b.pow_pll = 0;
        VENDOR_WRITE(BTON_UART_SPI_AFE_CTRL_REG,bton_uart_spi_afe_ctrl_reg.d32);
        DMA_UART_LOG(WHITE, DMA_UART_022, 1, bton_uart_spi_afe_ctrl_reg.d32);

        bton_core_afe_ldo_ctrl_reg.d32 = VENDOR_READ(BTON_CORE_AFE_LDO_CTRL_REG);
        bton_core_afe_ldo_ctrl_reg.b.uart_spi_pll_iso_en = 1;
        VENDOR_WRITE(BTON_CORE_AFE_LDO_CTRL_REG,bton_core_afe_ldo_ctrl_reg.d32);

        bton_uart_spi_afe_ctrl_reg.d32 = VENDOR_READ(BTON_UART_SPI_AFE_CTRL_REG);
        bton_uart_spi_afe_ctrl_reg.b.pow_ldo = 0;
        VENDOR_WRITE(BTON_UART_SPI_AFE_CTRL_REG,bton_uart_spi_afe_ctrl_reg.d32);
        DMA_UART_LOG(WHITE, DMA_UART_022, 1, bton_uart_spi_afe_ctrl_reg.d32);
    }
#endif    
}

#endif


// For ISSC Tool Only
//#ifdef _YL_H5_ISSC_NOP_EVT
#if 1
void hci_send_nope_command_status_event(void)
{
    UCHAR *pkt_buf;
    if (OS_ALLOC_BUFFER(tx_table[HCI_EVENT_HANDLER_TASK].pool_handle,
                                (void **)(&pkt_buf)) != BT_ERROR_OK)
    {
        return;
    }
    pkt_buf[0] = 0x0F;
    pkt_buf[1] = 0x04;    
    pkt_buf[2] = 0x00;
    pkt_buf[3] = g_efuse_lps_setting_2.issc_nope_num + 1;
    pkt_buf[4] = 0x00;
    pkt_buf[5] = 0x00;

    hci_td_deliver_event_to_host(pkt_buf);
}
#endif


#ifdef _UART_H5
/* To be executed when going to LPS to safely close  the SPI/UART PLL and UART block */
API_RESULT hci_uart_lps_clock_off_procedure(void)
{
#ifndef _8821A_NEW_UART_DESIGN_    
    BAUDRATE_FORMAT_TYPE baud_dummy;
    UINT8 spi_mode = hci_uart_check_spi_mode();
#endif    
    if ((g_fun_interface_info.b.bt_interface != UART_INTERFACE))
        return API_SUCCESS;
    
    /* disable hci_en to avoid unstable clock go into UART block */
    hci_uart_set_vendor_hci_en(0);

    /* turn off pll and pllldo if needed */
#ifdef _8821A_NEW_UART_DESIGN_        
    EFUSE_POW_SETTING_1_S efuse_pow_setting_1;
    *(UINT8*)&efuse_pow_setting_1 = otp_str_data.efuse_pow_setting_1_d8;	
    if (efuse_pow_setting_1.turn_off_ck120m_at_lps)
    {
        DMA_UART_SOFEWARE_RESET_REG_S_TYPE dma_uart_sw_rst_reg;
        dma_uart_sw_rst_reg.d8 = VENDOR_BYTE_READ(DMA_UART_SOFEWARE_RESET_REG);
//        if (!hci_uart_check_spi_mode())
#ifndef SPI_FLASH_BOOT
        {
            if(dma_uart_sw_rst_reg.b.uart_hci_sel)
            {
                trun_on_off_uart_spi_pll(PLL_OFF);
            }
        }
#endif        
    }
#else
    if(!spi_mode)
    {
        trun_on_off_uart_spi_pll(PLL_OFF, baud_dummy);
    }
#endif    
    return API_SUCCESS;    
}
/* To be executed when exiting from LPS to safely restore the SPI/UART PLL and UART block 
  * return API_SUCCESS: every is ok
  * return API_FAILURE: pll turn-on timeout
  */
API_RESULT hci_uart_lps_clock_on_procedure(void)
{
    // TODO: need to confirm that hci_uart_read_hw_setting() results is correct after LPS and PLL_OFF
#ifndef _8821A_NEW_UART_DESIGN_        
#ifdef _YL_RTL8723A_B_CUT
    BAUDRATE_FORMAT_TYPE baud_hw;
    UINT8 spi_mode;
    UINT8 pll_ckrdy_timeout;
    UINT32 uart_sw_rst_reg; 
    BTON_UART_SPI_AFE_CTRL_REG_S_TYPE bton_uart_spi_afe_ctrl_reg;
    
    if ((g_fun_interface_info.b.bt_interface != UART_INTERFACE))
        return API_SUCCESS;
    spi_mode = hci_uart_check_spi_mode();

    uart_sw_rst_reg = VENDOR_READ(DMA_UART_SOFEWARE_RESET_REG);
    bton_uart_spi_afe_ctrl_reg.d32 = VENDOR_READ(BTON_UART_SPI_AFE_CTRL_REG);
    
    // to optimize?
    baud_hw.d32 = 0;
    baud_hw.b2.uart_clk_sel = (uart_sw_rst_reg & BIT0);
    baud_hw.b2.uart_pll_d = bton_uart_spi_afe_ctrl_reg.b.d;
    baud_hw.b2.uart_pll_sel = bton_uart_spi_afe_ctrl_reg.b.sel;
#else
    BAUDRATE_FORMAT_TYPE baud_hw = hci_uart_read_hw_setting();
    UINT8 spi_mode = hci_uart_check_spi_mode();
    UINT8 pll_ckrdy_timeout;
    if ((g_fun_interface_info.b.bt_interface != UART_INTERFACE))
        return API_SUCCESS;
#endif
#else
    if ((g_fun_interface_info.b.bt_interface != UART_INTERFACE))
        return API_SUCCESS;
#endif
        
    /* turn off pll and pllldo if needed */
#ifdef _8821A_NEW_UART_DESIGN_     
    EFUSE_POW_SETTING_1_S efuse_pow_setting_1;
    *(UINT8*)&efuse_pow_setting_1 = otp_str_data.efuse_pow_setting_1_d8;	
    if (efuse_pow_setting_1.turn_off_ck120m_at_lps)
    {
        DMA_UART_SOFEWARE_RESET_REG_S_TYPE dma_uart_sw_rst_reg;
        dma_uart_sw_rst_reg.d8 = VENDOR_BYTE_READ(DMA_UART_SOFEWARE_RESET_REG);
//        if (!hci_uart_check_spi_mode())
#ifndef SPI_FLASH_BOOT
        {
            if(dma_uart_sw_rst_reg.b.uart_hci_sel)
            {
                trun_on_off_uart_spi_pll(PLL_ON);
            }
        }
#endif        
    }
#else
    if(!spi_mode)
    {
        if(baud_hw.b2.uart_clk_sel)
        {
            trun_on_off_uart_spi_pll(PLL_ON, baud_hw);
            pll_ckrdy_timeout = wait_uart_spi_pll_ready();
            if(pll_ckrdy_timeout)
            {
                // TODO: hci_uart_set_vendor_hci_en(1) whether pll turn-on timeout? to avoid dead-lock
                //2 this should never occur; So we hope pll would get ready eventually
                hci_uart_set_vendor_hci_en(1);
                return API_FAILURE;
            }
        }
    }    
#endif    
    /* enable hci_en after clock is stable to avoid unstable clock go into UART block */
    hci_uart_set_vendor_hci_en(1);
    return API_SUCCESS;    
}



/* Only Valid for H5 Mode 
  * H5 hardware sends a 2-character break signal
  * return API_FAILURE: not UART & H5 interface
  */
API_RESULT hci_uart_h5_send_short_break(void)
{
    DMA_UART_LOG(GRAY,DMA_UART_108,0,0);
    DMA_UART_H5_CTL0_REG_S_TYPE dma_uart_h5_ctl0_reg;

    if ((g_fun_interface_info.b.bt_interface != UART_INTERFACE) || (!g_data_uart_settings_2.h5_en))
        return API_FAILURE;
    
    dma_uart_h5_ctl0_reg.d32=UART_DWORD_READ(DMA_UART_H5_CTL0);
    dma_uart_h5_ctl0_reg.b.send_break=1;
    UART_DWORD_WRITE(DMA_UART_H5_CTL0,dma_uart_h5_ctl0_reg.d32); /* auto-cleard when done */
    return API_SUCCESS;    
}

// TODO: should call any function or set any flag?
void hci_uart_send_long_break_callback(TimerHandle_t timer_handle)
{
    DMA_UART_LINE_CTL_REG_S_TYPE dma_uart_line_ctl_reg;

#ifdef _ROM_CODE_PATCHED_
    /* Reserved com code patch by yilinli:
      * e.g. 
      *    1. 
      */    
    if (rcp_hci_uart_send_long_break_callback_func != NULL)
    {
        if(rcp_hci_uart_send_long_break_callback_func(timer_handle))
        {
            return;
        }
    }   
#endif

    /* free software timer */    
    if (timer_handle != NULL)
    {
        OS_DELETE_TIMER(&timer_handle);
    }    
    
    /* set set_break as 0 */
    dma_uart_line_ctl_reg.d8= UART_DWORD_READ(DMA_UART_LINE_CTL_REG_OFF);
    dma_uart_line_ctl_reg.b.set_break = 0;
    UART_DWORD_WRITE(DMA_UART_LINE_CTL_REG_OFF,dma_uart_line_ctl_reg.d8);
}

// TODO: ???
/* Valid for both H4 and H5 modes 
 * return API_FAILURE: A long break is on-going or No available TIMER or Not UART Interface
 * return API_SUCCESS: A long break is issued and on-going
 */
API_RESULT hci_uart_send_long_break(void)
{    
    TimerHandle_t long_break_timer_id = NULL;
    DMA_UART_LINE_CTL_REG_S_TYPE dma_uart_line_ctl_reg;
    dma_uart_line_ctl_reg.d8 = UART_DWORD_READ(DMA_UART_LINE_CTL_REG_OFF);
    const UINT32 TIME_MS = ((UINT32)10) << g_data_uart_settings_3.long_break_duration;

    if (g_fun_interface_info.b.bt_interface != UART_INTERFACE) 
        return API_FAILURE;
    
    // TODO: Note: should avoid timer deleted by other functions ... e.g. hci_reset 
    if (dma_uart_line_ctl_reg.b.set_break)
    {
        /* A long break is on-going */
        return API_FAILURE;
    }

    /* Create a tid timer */    
    if (OS_CREATE_TIMER(ONESHOT_TIMER, &long_break_timer_id,
            hci_uart_send_long_break_callback, NULL, 0) != BT_ERROR_OK)
    {
        return API_FAILURE;
    }
    
    DMA_UART_LOG(GRAY,DMA_UART_109,1, (UINT32)long_break_timer_id);    
    
    dma_uart_line_ctl_reg.b.set_break = 1;
    UART_DWORD_WRITE(DMA_UART_LINE_CTL_REG_OFF,dma_uart_line_ctl_reg.d8);    
    OS_START_TIMER(long_break_timer_id, TIME_MS);
    return API_SUCCESS;
}

// TODO: How to used?
/* For DMA: 
  *    pretend as "ack all pending resend packets"
  *    clear own bits of RX descriptors for those packets pending to resend. 
  *    and also interrupt FW about RX done !!! 
  *    DMA will also fetch new RX packet if these is one in SRAM
  *    designed for handle retry fail interrupt (retry count reach max. value) (should be optional)
  *
  * On the other hand, if FW want to stop any DMA actions, shall soft-reset DMA and clear all RX descriptor own bit
  */
void hci_uart_h5_clr_unack_pkt(void)
{
    DMA_UART_LOG(GRAY,DMA_UART_110,0,0);
    DMA_UART_H5_CTL0_REG_S_TYPE dma_uart_h5_ctl0_reg;
    dma_uart_h5_ctl0_reg.d32 = UART_DWORD_READ(DMA_UART_H5_CTL0);
    dma_uart_h5_ctl0_reg.b.clr_unack_pkt = 1;
    UART_DWORD_WRITE(DMA_UART_H5_CTL0, dma_uart_h5_ctl0_reg.d32); /* auto-cleard when done */
}

// TODO: to test
void hci_uart_h5_set_go_park(UINT8 value)
{
    DMA_UART_H5_CTL0_REG_S_TYPE dma_uart_h5_ctl0_reg;
    
    dma_uart_h5_ctl0_reg.d32 = UART_DWORD_READ(DMA_UART_H5_CTL0);
    if (value)
        dma_uart_h5_ctl0_reg.b.go_park = 1;
    else
        dma_uart_h5_ctl0_reg.b.go_park = 0;
    UART_DWORD_WRITE(DMA_UART_H5_CTL0,dma_uart_h5_ctl0_reg.d32);
}

// TODO: to test
void hci_uart_h5_set_go_idle(UINT8 value)
{
    DMA_UART_H5_CTL0_REG_S_TYPE dma_uart_h5_ctl0_reg;
    
    dma_uart_h5_ctl0_reg.d32 = UART_DWORD_READ(DMA_UART_H5_CTL0);
    if (value)
        dma_uart_h5_ctl0_reg.b.go_idle = 1;
    else
        dma_uart_h5_ctl0_reg.b.go_idle = 0;
    UART_DWORD_WRITE(DMA_UART_H5_CTL0,dma_uart_h5_ctl0_reg.d32);
}

// TODO: to test
#if 0 // not used function; marked to save ROM size
UINT16 hci_uart_h5_read_tx_ack(void)
{
    DMA_UART_H5_INTSTS_REG_S_TYPE dma_uart_h5_intsts_reg;
    H5_HW_STATE_TYPE h5_hw_state;
    
    dma_uart_h5_intsts_reg.d32 = UART_DWORD_READ(DMA_UART_H5_INTSTS);
    h5_hw_state.d16 = dma_uart_h5_intsts_reg.b.h5_current_st_d16;
    return h5_hw_state.b.tx_ack_num;
}
#endif

UINT16 hci_uart_h5_read_ln_state(void)
{
    DMA_UART_H5_INTSTS_REG_S_TYPE dma_uart_h5_intsts_reg;
    H5_HW_STATE_TYPE h5_current_st;
    dma_uart_h5_intsts_reg.d32 = UART_DWORD_READ(DMA_UART_H5_INTSTS);
    h5_current_st.d16 = dma_uart_h5_intsts_reg.b.h5_current_st_d16;    
    return h5_current_st.b.ln_state;
}

/* return API_FAILURE: not in ACTIVE STATE to send SLEEP message
  * return API_SUCCESS: send_sleep command has been issued, and the sending is ON-GOING
  */
API_RESULT hci_uart_h5_send_sleep_msg(void)
{
    DMA_UART_LOG(GRAY,DMA_UART_111,0,0);

    DEF_CRITICAL_SECTION_STORAGE;
    
    MINT_OS_ENTER_CRITICAL();

    DMA_UART_H5_CTL0_REG_S_TYPE dma_uart_h5_ctl0_reg;
    if (hci_uart_h5_read_ln_state() != H5_STATE_ACTIVE)
    {
        MINT_OS_EXIT_CRITICAL();
        return API_FAILURE;            
    }
    
    dma_uart_h5_ctl0_reg.d32=UART_DWORD_READ(DMA_UART_H5_CTL0);
    dma_uart_h5_ctl0_reg.b.send_sleep=1;
    UART_DWORD_WRITE(DMA_UART_H5_CTL0,dma_uart_h5_ctl0_reg.d32); /* auto-cleard when done */
    hci_uart_h5_set_sleep_msg_state(1);    
//    hci_uart_man.h5_sleep_msg_state=1;
//    g_host_state = 1;
    MINT_OS_EXIT_CRITICAL();
    return API_SUCCESS;
}

/* return API_FAILURE: not in ACTIVE STATE to send SLEEP message OR send_sleep timeout;
  * return API_SUCCESS: SLEEP message has been sent
  */
  // TODO: to be used in LPS or SUS or both?
API_RESULT hci_uart_h5_send_sleep_msg_wait_done(void)
{
    
    DMA_UART_H5_CTL0_REG_S_TYPE dma_uart_h5_ctl0_reg;
    if (hci_uart_h5_send_sleep_msg() != API_SUCCESS)
    {
        return API_FAILURE; 
    }
    if(!g_data_uart_settings_2.h5_lowpow_sleep_msg_wait_en)
    {
        return API_SUCCESS;     
    }
    
    /* to wait for the procedures is done by HW */
        
    UINT32 baudrate_hz = hci_uart_calculate_baudrate_hz(hci_uart_read_hw_setting());
    UINT16 baudrate_khz = (baudrate_hz>>10);
    UINT16 byte_us, i;
    if (baudrate_khz)
    {    
        /* worst-case: 10 bytes * 13T/byte */
        byte_us = 13000/baudrate_khz+1; 
    }    
    else
    {
        byte_us = 1;
    }
    
    DMA_UART_LOG(GRAY,DMA_UART_112,2, baudrate_khz, byte_us);    

    /* extra 2-byte margin for waiting done */
    for ( i = 0; i<12; i++) 
    {
        pf_delay_us(byte_us);    
        dma_uart_h5_ctl0_reg.d32=UART_DWORD_READ(DMA_UART_H5_CTL0);
        if (!dma_uart_h5_ctl0_reg.b.send_sleep)
        {
            /* before lleaving, delay one more byte_us for ending 0xC0 */
            pf_delay_us(byte_us);
#ifdef _YL_H5_TEST_SEND_SHORT_BREAK_WHEN_SLEEP_WAIT      
            pf_delay_us(byte_us*2);            
            hci_uart_h5_send_short_break();
#endif            
            return API_SUCCESS;
        }
    }        
    return API_FAILURE;
}


// TODO: should call any function or set any flag?
void hci_uart_h5_poll_wake_callback(TimerHandle_t timer_handle)
{
    DMA_UART_H5_CTL0_REG_S_TYPE dma_uart_h5_ctl0_reg;
    DMA_UART_DBG_LOG(RED, DMA_UART_002_DBG_DEC, 1, timer_handle);

#ifdef _ROM_CODE_PATCHED_
    /* Reserved com code patch by yilinli:
      * e.g. 
      *    1. 
      */    
    if (rcp_hci_uart_h5_poll_wake_callback_func != NULL)
    {
        if(rcp_hci_uart_h5_poll_wake_callback_func(timer_handle))
            return;
    }      
#endif

    /* check if there is mismatch between h5_man and callback */
    /* Note: shall make sure no Timer Leakage */
    if (timer_handle != h5_poll_wake_timer_id)
    {
        OS_DELETE_TIMER(&timer_handle);
        return;
    }
    /* free software timer */    
    if (timer_handle != NULL)
    {
        OS_DELETE_TIMER(&timer_handle);
    }    
    
    /* set set_break as 0 */
    dma_uart_h5_ctl0_reg.d32 = UART_DWORD_READ(DMA_UART_H5_CTL0);
    if (dma_uart_h5_ctl0_reg.b.poll_wake)
    {
        //2 poll_wake fails to received response from Host
        // TODO: should call any function or set any flag?
        dma_uart_h5_ctl0_reg.b.poll_wake = 0;
        UART_DWORD_WRITE(DMA_UART_H5_CTL0, dma_uart_h5_ctl0_reg.d32); 
        DMA_UART_LOG(RED, DMA_UART_076, 1, timer_handle);
    }
}


// TODO: FW need to process the case of poll_wake but NO reply => Send Break or Normal Data?
// TODO: when to use
// TODO: to test
/* Note: Only valid when when  H5 link is established (i.e. DMA_UART_H5_INTSTS_REG.h5_current_st.ln_state=3)  
  * return API_FAILURE: not in ACTIVE state OR create timer fail OR a poll_wake in on-going
  * return API_SUCCESS: a poll_wake is successfully issued
  */
API_RESULT hci_uart_h5_poll_wake(void)
{
    DMA_UART_LOG(GRAY,DMA_UART_113,0,0);
//    TIMER_ID poll_wake_timer_id = OS_INVALID_HANDLE;
    DMA_UART_H5_CTL0_REG_S_TYPE dma_uart_h5_ctl0_reg;
    
    if (hci_uart_h5_read_ln_state() != H5_STATE_ACTIVE)
    {
        return API_FAILURE;
    }
    
    DMA_UART_LOG(GRAY,DMA_UART_114,0,0);
    
    
    dma_uart_h5_ctl0_reg.d32 = UART_DWORD_READ(DMA_UART_H5_CTL0);
    const UINT32 TIME_MS = ((UINT32)10) << (g_data_uart_settings_3.h5_poll_wake_duration+g_data_uart_settings_3.h5_poll_wake_duration);


    // TODO: Note: should avoid timer deleted by other functions ... e.g. hci_reset 
    if (dma_uart_h5_ctl0_reg.b.poll_wake)
    {
        /* A long break is on-going */
        DMA_UART_LOG(GRAY,DMA_UART_115,0,0);
        return API_FAILURE;
    }
    if (g_data_uart_settings_3.h5_poll_wake_duration == 7) 
    {
        /* Persistent send WAKEUP until receiving TX packets or somewhere set poll_wake = 1 */
         dma_uart_h5_ctl0_reg.b.poll_wake = 1;
        UART_DWORD_WRITE(DMA_UART_H5_CTL0, dma_uart_h5_ctl0_reg.d32);
    }
    else
    {
        /* Check if there is a timer on-going, if YES, delete it*/
        if (h5_poll_wake_timer_id != NULL)
        {
            OS_DELETE_TIMER(&h5_poll_wake_timer_id);
        }

        /* Create a tid timer */    
        if (OS_CREATE_TIMER(ONESHOT_TIMER, &h5_poll_wake_timer_id,
                hci_uart_h5_poll_wake_callback, NULL, 0) != BT_ERROR_OK)
        {
            return API_FAILURE;
        }
        DMA_UART_LOG(GRAY,DMA_UART_116,1,(UINT32)h5_poll_wake_timer_id);
        OS_START_TIMER(h5_poll_wake_timer_id, TIME_MS);
        
        dma_uart_h5_ctl0_reg.b.poll_wake = 1;
        UART_DWORD_WRITE(DMA_UART_H5_CTL0, dma_uart_h5_ctl0_reg.d32); 
        
    }
    return API_SUCCESS;    
}

// TODO: To be executed before LPS??
/* <<< To be executed before BT protocol low power modes with digital power-on (UART H5 HW maintains its state information) >>>
  * send H5 low-power SLEEP message if enabled by g_data_uart_settings_3.h5_lowpow_sleep_msg_en
  */
API_RESULT hci_uart_h5_go_lps(void)
{
#ifdef _ROM_CODE_PATCHED_
    /* Reserved com code patch by yilinli:
      * e.g. 
      *    1. 
      */    
    if (rcp_hci_uart_h5_go_lps_func != NULL)
    {
        return rcp_hci_uart_h5_go_lps_func(NULL);
    }     
#endif

    if ((g_fun_interface_info.b.bt_interface != UART_INTERFACE) || (!g_data_uart_settings_2.h5_en))
        return API_SUCCESS;

    if (g_data_uart_settings_3.h5_lowpow_sleep_msg_en)
    {
        hci_uart_h5_send_sleep_msg_wait_done();
    }
    return API_SUCCESS;                    
}    

// TODO: to include hci_uart_h5_send_sleep_msg_wait_done()?
// TODO: To be executed before SUSPEND??
/* <<< To be executed before BT digital power-off (UART H5 HW would loss its state information) >>>
  * Note: hci_uart_h5_go_sleep() shall be excuted in CRITICAL_SECTION or ISR !!!
  *          (Else, it shall extrally check hci_uart_man.h5_btdma_triggered and HCI DMA isr_status to confirm no USB_DMA TRX INT happens!!!)
  *
  * go_sleep() should only be used when HCI RX and TX is idle/empty
  * There may be packets occur just before go_sleep, therefore, a double-check after go_sleep() is MUST!
  * If the confirmation fails, then abort sleep by hci_uart_h5_set_go_idle(0);
  * After hci_uart_h5_go_sleep() return API_FAILURE: FW shall NOT enter Low Power State
  * After hci_uart_h5_go_sleep() return API_SUCCESS: 
  *         if FW decides NOT to deep sleep(UART would be reset), hci_uart_h5_abort_sleep() shall be executed!!!
  * ** Note: if not UART & H5 Interface, it also return API_SUCCESS to make the follwing LPS flow going forward
  */              
API_RESULT hci_uart_h5_go_sleep(void)
{
//    DMA_UART_H5_CTL0_REG_S_TYPE dma_uart_h5_ctl0_reg;
    DMA_UART_H5_INTSTS_REG_S_TYPE dma_uart_h5_intsts_reg;
    UINT16 i;

#ifdef _ROM_CODE_PATCHED_
    /* Reserved com code patch by yilinli:
      * e.g. 
      *    1. 
      */    
    if (rcp_hci_uart_h5_go_sleep_func != NULL)
    {
        return rcp_hci_uart_h5_go_sleep_func(NULL);
    }       
#endif

    if ((g_fun_interface_info.b.bt_interface != UART_INTERFACE) || (!g_data_uart_settings_2.h5_en))
        return API_SUCCESS;
    
//    const UINT16 H5_GO_STOP_TIMEOUT = 10;
    DMA_UART_LOG(GRAY,DMA_UART_117,0,0);

    
    /* set go_park and go_idle = 0 to make park_ok_int W1C possible */
    hci_uart_h5_set_go_park(0);
    hci_uart_h5_set_go_idle(0);

    /* clear park_ok_int by W1C before issue the go_idle command */
    dma_uart_h5_intsts_reg.d32 = 0;
    dma_uart_h5_intsts_reg.b.park_ok_int = 1;
    UART_DWORD_WRITE(DMA_UART_H5_INTSTS,dma_uart_h5_intsts_reg.d32);

//    hci_uart_man.h5_btdma_triggered = 0;    

    /* issue the go_idle command */
    hci_uart_h5_set_go_idle(1); /* go_idle is preferred than go_park*/
    
    /* wait for UART_H5_INTSTS[8](PARK_OK_INT) */
    for (i = 0; i<10; i++)
    {
        /* poll park_on_int to check the park status */
        dma_uart_h5_intsts_reg.d32 = UART_DWORD_READ(DMA_UART_H5_INTSTS);
        if (dma_uart_h5_intsts_reg.b.park_ok_int)
        {
            DMA_UART_DBG_LOG(GRAY,DMA_UART_002_DBG_DEC,1,i);
            /* wait more than 1us to finsih DMA processing */
            pf_delay_us(2);
             
            /* after 40 cycles: check isr_status to make sure not pending HCI TX/RX interrupts*/
            ISR_STATUS_TYPE isr_status;
            isr_status.d32 = DMA_DWORD_READ(ISR_STATUS_REG);

            if (isr_status.d32 & DMA_ISR_H5_GO_SLEEP_CHECK_MASK)
            {
#ifdef _YL_H5_TEST
                h5_go_sleep_isr_status_times++;
#endif
                DMA_UART_DBG_LOG(GRAY,DMA_UART_089, 0, 0);
                break;
             }
             else
             {
#ifdef _8821A_BTON_DESIGN_             
                 if (g_data_uart_settings.uart_h5_go_sleep_backup_en)
#endif                 
                 {
                     hci_uart_h5_backup();
                 }
                 DMA_UART_DBG_LOG(GRAY,DMA_UART_088,3,
                                       dma_uart_h5_intsts_reg.b.park_ok_int,
                                       0,
                                       dma_uart_h5_intsts_reg.d32);                    
                 return API_SUCCESS;
             }

        }
    }

    /* Abort Sleep before return API_FAILURE */
    hci_uart_h5_set_go_idle(0);
    DMA_UART_DBG_LOG(GRAY,DMA_UART_087,3,
                          dma_uart_h5_intsts_reg.b.park_ok_int,
                          0,
                          dma_uart_h5_intsts_reg.d32);
    return API_FAILURE;
}


void hci_uart_h5_wakeup_utility_task(API_RESULT (*hdlr)(void), uint32_t no_arg)
{
    hdlr();
}

API_RESULT hci_uart_h5_wakeup_utility_task_wrapper(API_RESULT (*hdlr)(void))
{
    API_RESULT ret;
    if (IN_ISR())
    {
        BaseType_t high_pri_task_woken = pdFALSE;
        if (xTimerPendFunctionCallFromISR(
                (PendedFunction_t) hci_uart_h5_wakeup_utility_task,
                hdlr, 0, &high_pri_task_woken) == pdPASS)
        {
            ret = API_SUCCESS;
        }
        else
        {
            ret = API_FAILURE;
        }
        portYIELD_FROM_ISR(high_pri_task_woken);
    }
    else
    {
        ret = hdlr();
    }
    return ret;
}

/* used for g_data_uart_settings_3.h5_retry_alarm_opt, g_data_uart_settings_3.h5_retry_fail_opt, g_data_uart_settings_3.h5_lowpow_wakeup_opt */
API_RESULT hci_uart_h5_wakeup_utility(UINT8 option)
{
#ifdef _ROM_CODE_PATCHED_
    /* Reserved com code patch by yilinli:
      * e.g. 
      *    1. 
      */    
    if (rcp_hci_uart_h5_wakeup_utility_func != NULL)
    {
        return rcp_hci_uart_h5_wakeup_utility_func((void*)(&option)); 
    }      
#endif

    API_RESULT result;  
    if (option == H5_WAKEUP_OPT_NONE)
    {
        result = API_SUCCESS;
    }
    else if (option == H5_WAKEUP_OPT_SHORT_BREAK)
    {
        hci_uart_h5_send_short_break();
        result = API_SUCCESS;
    }
    else if (option == H5_WAKEUP_OPT_LONG_BREAK)
    {    
        result = hci_uart_h5_wakeup_utility_task_wrapper(hci_uart_send_long_break);
    }    
    else if (option == H5_WAKEUP_OPT_POLL_WAKE)
    {    
        result = hci_uart_h5_wakeup_utility_task_wrapper(hci_uart_h5_poll_wake);
    }        
    else
    {
        result = API_FAILURE;
    }
    return result;
}

/* Used only for H5 Mode, Low Power Message Options 
 * return API_FAILURE: Some action is on-going (determined by g_data_uart_settings_3.h5_lowpow_wakeup_opt)
 *                                H5 protocol: DMA RX is also permitted, which will be re-transmitted thereafter.
 *                                But it is ECO-dependent!!! If not ECO-ed, FW shall implement the blocking function.
 * return API_SUCCESS: DMA RX is permitted and to be issued
 *
 * *Recommended Usage: FW codes may refer to hci_uart_man.h5_sleep_msg_state to blocking HCI RX traffic or not
 *                                  Every time HCI RX traffic occurs, it calls this function to do poll_wake
 *                                  After poll_wake issued, at (poll_wake_int OR rx_woken_int) reschedule HCI RX packets
 */
API_RESULT hci_uart_h5_check_sleep_before_hci_rx(void)
{    
    // TODO:  sleep_msg_state, g_data_uart_settings_2.h5_en, g_data_uart_settings_3.h5_lowpow_wakeup_opt
    if ((g_fun_interface_info.b.bt_interface != UART_INTERFACE) || (!g_data_uart_settings_2.h5_en))
        return API_SUCCESS;
        
    if (hci_uart_man.h5_sleep_msg_state && (g_data_uart_settings_3.h5_lowpow_wakeup_opt))
    {        
        return hci_uart_h5_wakeup_utility(g_data_uart_settings_3.h5_lowpow_wakeup_opt);
    }
    else
    {
        return API_SUCCESS;
    }
}


// TODO: Priority and Excution Order???
// TODO: shall be patch-able!!!
// TODO: Time-Consuming ???
void hci_uart_h5_isr(UINT32 hci_dma_isr_status_d32)
{
    DMA_UART_H5_INTSTS_REG_S_TYPE dma_uart_h5_intsts_reg;
    UINT32 d32_w1c;

#ifdef _ROM_CODE_PATCHED_
    /* Reserved com code patch by yilinli:
      * e.g. 
      */
    if (rcp_hci_uart_h5_isr_func != NULL)
    {
        if ( rcp_hci_uart_h5_isr_func((void*)(&hci_dma_isr_status_d32)) )
        {
            return;
        }
    }    
#endif

    dma_uart_h5_intsts_reg.d32 = UART_DWORD_READ(DMA_UART_H5_INTSTS);

    //2 recover the h5 sleep_msg_state when received TX packets or when RX packets ACK-ed
    //2 SCO may be unreliable, so RX interrupt does not mean it is ACK-ed
    //2 Also set sleep_msg_state in  hci_uart_h5_send_sleep_msg()    


//    DMA_UART_DBG_LOG(YELLOW,DMA_UART_078, 1, hci_uart_man.h5_trx_active_mask);            
    if (hci_dma_isr_status_d32 & hci_uart_man.h5_trx_active_mask)
    {
        hci_uart_h5_set_sleep_msg_state(0);    
//        hci_uart_man.h5_sleep_msg_state = 0;
//        g_host_state = 0;
        if (g_data_uart_settings_2.h5_retry_state_clr_when_trx)
        {
            hci_uart_man.h5_retry_alarm_state = 0;
            hci_uart_man.h5_retry_fail_state = 0;
        }
    }

//    hci_uart_man.h5_btdma_triggered = 1;

    if (dma_uart_h5_intsts_reg.d32 & g_data_uart_settings_2.h5_int_en)
    {        
#if 1
        d32_w1c = dma_uart_h5_intsts_reg.d32;
        d32_w1c &= (0xffff0000 | g_data_uart_settings_2.h5_int_en);
        UART_DWORD_WRITE(DMA_UART_H5_INTSTS, d32_w1c);
#endif
        RT_BT_LOG(YELLOW,DMA_UART_077, 1, dma_uart_h5_intsts_reg.d32);            

    
#ifdef _CCH_LPS_
#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8723_RCP_
        if (rcp_hci_uart_h5_isr_lps_reset!= NULL)
        {
            rcp_hci_uart_h5_isr_lps_reset();
        }
        else
#endif	
#endif
	{
            if(g_efuse_lps_setting_3.timer2_reset_at_uart_isr_en)
            {
                g_lps_timer_counter =  g_timer2_init_value;
            }
        }
#endif

        if (dma_uart_h5_intsts_reg.b.link_est_int)
        {
#ifdef _UART_BAUD_ESTIMATE_
            hci_uart_man_sram.hci_uart_baud_est_state = HCI_UART_BAUD_EST_ST_STOP;
            if (g_data_uart_settings.baud_record_at_h5_link_est)
            {
                // for original BuadDet to avoid baudrate un-sync-ed //
                hci_uart_man_sram.baud_current_setting = hci_uart_read_hw_setting();
            }
            if (g_data_uart_settings.baud_est_wr_bton_en_at_h5link_uartsync)
            {
                BAUDRATE_FORMAT_TYPE baud_hw = hci_uart_read_hw_setting();
                hci_uart_baud_record_to_bton(baud_hw.d32);
            }
#endif
            if( g_efuse_lps_setting_2.issc_nope_en )
            {	
//            #ifdef _YL_H5_ISSC_NOP_EVT

                //2 Note: used only for non-Loopback and ISSC Tool Mode
                hci_send_nope_command_status_event();
//            #endif
            }
        }
        if (dma_uart_h5_intsts_reg.b.link_reset_int)
        {
            hci_uart_h5_set_sleep_msg_state(0);        
//             hci_uart_man.h5_sleep_msg_state=0;    
//             g_host_state = 0;        
//            if(g_data_uart_settings_2.h5_linkreset_restart)
            if(1)
            {
#ifdef _8821A_BTON_DESIGN_ // TODO: sign another signature at link-reset
                // Reset by WATCH_DOG timeout
                if (g_data_uart_settings.h5_linkrst_sign_fw_trig_wdg)
                {
                    SIGN_FW_TRIG_WDG_TO_SIGNATURE;
                }
                if (g_data_uart_settings.h5_sign_linkrst_signature)
                {
                    SIGN_H5_LINKRESET_SIGNATURE;
                }
#endif                
                WDG_TIMER_TIMEOUT_SOON;
                while(1)
            {
                    d32_w1c = 0;
                }
            }
        }
        if (dma_uart_h5_intsts_reg.b.rx_wakeup_int)
        {
             hci_uart_h5_set_sleep_msg_state(0);            
//             hci_uart_man.h5_sleep_msg_state=0;    
//             g_host_state = 0;
        }
        if (dma_uart_h5_intsts_reg.b.rx_woken_int)
        {
            hci_uart_h5_set_sleep_msg_state(0);            
//            hci_uart_man.h5_sleep_msg_state=0;
//            g_host_state = 0;
        }
        if (dma_uart_h5_intsts_reg.b.rx_sleep_int)
        {
            hci_uart_h5_set_sleep_msg_state(1);            
//            hci_uart_man.h5_sleep_msg_state=1;
//            g_host_state = 1;
           DMA_UART_LOG(GRAY,DMA_UART_119,1, hci_uart_man.h5_sleep_msg_state);
        }
#if 0 // unused function; marked to save ROM size        
        if (dma_uart_h5_intsts_reg.b.slip_err_int)
        {
        }
#endif        
        if (dma_uart_h5_intsts_reg.b.poll_wake_int)
        {
            /* alternative: check rx_woken to decide whether WOKEN or other slip packet received (may have CRC error) */
            /* alternative: decide whether to re-poll_wake or not */
            hci_uart_h5_set_sleep_msg_state(0);            
//            hci_uart_man.h5_sleep_msg_state=0;            
//            g_host_state = 0;
#if 0 // unused function; marked to save ROM size  
            if (dma_uart_h5_intsts_reg.b.rx_woken_int)
            {
                /* if poll_wkae is higher than DATA and runs concurrently: do nothing */
                /* else: optionally re-schedule HCI RX packets which are blocked by hci_uart_man.h5_sleep_msg_state */
                DMA_UART_LOG(GRAY,DMA_UART_120,0,0);
            }
            else
            {
                /* if poll_wkae is higher than DATA and runs concurrently: do nothing */
                /* else: optionally re-schedule HCI RX packets which are blocked by hci_uart_man.h5_sleep_msg_state */
                DMA_UART_LOG(GRAY,DMA_UART_121,0,0);
            }
#endif            
        }
        if (dma_uart_h5_intsts_reg.b.retry_alarm_int)
        {
            hci_uart_man.h5_retry_alarm_state = 1;
            hci_uart_h5_wakeup_utility(g_data_uart_settings_3.h5_retry_alarm_opt);
        }
        if (dma_uart_h5_intsts_reg.b.retry_fail_int)
        {
            hci_uart_man.h5_retry_fail_state = 1;
            hci_uart_h5_wakeup_utility(g_data_uart_settings_3.h5_retry_fail_opt);
            // TODO: extra option: hci_uart_h5_clr_unack_pkt();
            if (g_data_uart_settings.h5_linkfail_clr_unack)
            {
                hci_uart_h5_clr_unack_pkt();
            }           
        }
#if 0 // unused function; marked to save ROM size            
        if (dma_uart_h5_intsts_reg.b.park_ok_int)
        {
            //2 implemented in hci_uart_h5_go_sleep(); To be called by LPS manager
         }
#endif

     }
    
}


#ifdef _UART_BAUD_ESTIMATE_

const UINT16 g_uart_ovsr_adj_table[9]={0x000, 0x008, 0x222, 0x24A, 0x555, 0x5AD, 0x3BB, 0x7EF, 0x7FF};
#define HCI_UART_BAUD_ESTIMTE_DET_TABLE_SIZE 12
const UINT16 g_hci_uard_baud_est_det_ovsrdivx8_clk_sel_0[HCI_UART_BAUD_ESTIMTE_DET_TABLE_SIZE]={
//  3250000,    1500000,    921600, 460800, 230400, 115200, 76800,  57600,  38400,  28800,  19200,  14400
    98,         213,        347,    694,    1389,   2778,   4167,   5556,   8333,   11111,  16667,  22222};
const UINT16 g_hci_uard_baud_est_det_baud_ovsr_div_table[HCI_UART_BAUD_ESTIMTE_DET_TABLE_SIZE]={
//  3250000,                1500000,                921600,                 460800,
//  230400,                 115200,                 768000,                 57600,
//  38400,                  28800,                  19200,                  14400
(1 | ((12-5) << 12)),   (3 | ((9-5) << 12)),    (4 | ((11-5) << 12)),   ((8)   | ((11-5) << 12)),
(11 | ((16-5) << 12)),  (29 | ((12-5) << 12)),  (52 | ((10-5) << 12)),  (58 | ((12-5) << 12)),
(104 | ((10-5) << 12)), (116 | ((12-5) << 12)), (208 | ((10-5) << 12)), (232 | ((12-5) << 12))};



BAUDRATE_FORMAT_TYPE hci_uart_baud_est_exhaustive(UINT32 ovsr_div_x8_est, BAUDRATE_FORMAT_TYPE baud_old)
{
    UINT8 ovsrx8;
    UINT8 ovsrx8_min = 0;
    UINT32 div;
    UINT32 div_min = 0;
    INT16 err;
    UINT16 err_min=0xffff;
    UINT8 ovsrx8_grid;

    ovsrx8_grid = (1<<g_efuse_baud_est_setting_1.baud_est_ovsrx8_grid);
    
    for (ovsrx8 = ((g_efuse_baud_est_setting_3.baud_est_ovsr_low_bound+5)<<3); ovsrx8<=(21<<3); ovsrx8+=ovsrx8_grid)
    {
        div = (ovsr_div_x8_est+(ovsrx8>>1))/ovsrx8; // div = round(x/ovsrx8) //
        err = div*ovsrx8 - ovsr_div_x8_est;
        err = ABS(err);
        if (err < err_min)
        {
            err_min = (UINT16)err;
            div_min = div;
            ovsrx8_min = ovsrx8;
        }
    }

    baud_old.b2.uart_divisor = div_min;
    if (ovsrx8_min >= (21<<3))
    {
        baud_old.b2.uart_ovsr = (20-5);
        baud_old.b2.uart_ovsr_adj = g_uart_ovsr_adj_table[8];
    }
    else if (ovsrx8_min < (5<<3))
    {
        baud_old.b2.uart_ovsr = (5-5);
        baud_old.b2.uart_ovsr_adj = g_uart_ovsr_adj_table[0];
    }
    else
    {
        baud_old.b2.uart_ovsr = ((ovsrx8_min>>3)-5);
        baud_old.b2.uart_ovsr_adj = g_uart_ovsr_adj_table[(ovsrx8_min & 0x07)];
    }

    return baud_old;
    
}

BAUDRATE_FORMAT_TYPE hci_uart_baud_est_detect(UINT32 ovsr_div_x8_est, BAUDRATE_FORMAT_TYPE baud_old, UINT32 *ovsr_div_x8_err_min)
{
    UINT8 ii;
    UINT8 ii_min;
    BAUDRATE_FORMAT_TYPE baud_new;
    INT32 ovsr_div_x8_err_temp;
    UINT32 ovsr_div_x8_min;
    UINT8 ovsrdivx8_scaling;

    ii_min = 0xFF;
    *ovsr_div_x8_err_min = 0xFFFFFFFF;
    if (baud_old.b2.uart_clk_sel)
    {
        ovsrdivx8_scaling = HCI_UART_CLK_SEL_1_RATIO;
    }
    else
    {
        ovsrdivx8_scaling = 1;
    }
    
    for (ii = 0; ii<HCI_UART_BAUD_ESTIMTE_DET_TABLE_SIZE; ii++)
    {
        if ((g_efuse_baud_est_setting_3.det_allow_table>>ii) & BIT0)
        {
            ovsr_div_x8_err_temp = g_hci_uard_baud_est_det_ovsrdivx8_clk_sel_0[ii]*ovsrdivx8_scaling - ovsr_div_x8_est;
            ovsr_div_x8_err_temp = ABS(ovsr_div_x8_err_temp);
            if (ovsr_div_x8_err_temp < (*ovsr_div_x8_err_min))
            {
                *ovsr_div_x8_err_min = ovsr_div_x8_err_temp;
                ii_min = ii;
                ovsr_div_x8_min = g_hci_uard_baud_est_det_ovsrdivx8_clk_sel_0[ii]*ovsrdivx8_scaling;
            }
        }
    }
    
    if (ii_min == 0xFF)
    {
        baud_new.d32 = baud_old.d32;
    }
    else
    {
        if (g_efuse_baud_est_setting_2.baud_det_finetune_by_exhaust_est)
        {
            baud_new = hci_uart_baud_est_exhaustive(g_hci_uard_baud_est_det_ovsrdivx8_clk_sel_0[ii_min]*ovsrdivx8_scaling, baud_old);
        }
        else
        {
            baud_new.b2.uart_divisor = (g_hci_uard_baud_est_det_baud_ovsr_div_table[ii_min] & 0x0FFF)*ovsrdivx8_scaling;
            baud_new.b2.uart_ovsr = (g_hci_uard_baud_est_det_baud_ovsr_div_table[ii_min] >> 12) & 0x000F;
            baud_new.b2.uart_ovsr_adj = 0;
        }
    }
    
    return baud_new;
}


// TODO: dynamic uart_clk
// TODO: H4?
// TODO: hybrid method with original baud_det
void hci_uart_baud_est_and_update(void)
{
    DMA_UART_TOGGLE_MON_CTRL_REG_S_TYPE uart_toggle_mon;
    BAUDRATE_FORMAT_TYPE baud_new, baud_hw;
    UINT32 ovsr_div_x8_est;
    UINT16 min_low_falling_low_bound;
    UINT8 min_low_overflow;
    UINT8 min_falling_overflow;
    UINT8 min_low_underflow;
    UINT8 min_falling_underflow;
    UINT8 actual_baud_est_combine_opt; 
    UINT32 falling_low_ratiox8;
    UINT16 sts_read;
    UINT8 det_enable;
    UINT8 det_succ;
    UINT8 det_fail;

    // TODO: ADD PATCH???

#ifdef _YL_H5_TEST_BAUD_DETECT_LOG_
//    RT_BT_LOG(WHITE, YL_DBG_DEC_1, 1, 1000001);
#endif

    uart_toggle_mon.d32 = UART_DWORD_READ(DMA_UART_TOGGLE_MON_CTRL);    
    

    if (g_data_uart_settings.baud_det_en)
    {
        if (g_data_uart_settings.baud_est_use_original_buad_det_once && hci_uart_man_sram.baud_est_original_baud_det_done)
        {
            
        }
        else
        {        
            // NOTE: it has the risk of 
            sts_read = (UINT16)UART_DWORD_READ(DMA_UART_STS_REG);
            det_enable = (sts_read & BIT3)>>3; 
            det_succ = (sts_read & BIT2)>>2;
            det_fail = (sts_read & BIT1)>>1;        
            if (det_enable && det_succ)
            {
                // TODO: baud det success re-det function is required to avoid dead-lock
                hci_uart_man_sram.baud_current_setting = hci_uart_read_hw_setting();
                // set det_enable = 0, avoid W1C hci_indicator_error //
                UART_DWORD_WRITE(DMA_UART_STS_REG, sts_read & (~BIT0) & (~BIT3));
                if (g_data_uart_settings.baud_est_use_original_buad_det_once)
                {
                    hci_uart_man_sram.baud_est_original_baud_det_done = 1;
                    return;
                }
            }
            else if (det_enable && det_fail) // TODO: to be tested
            {
                // re-detect; expected to be used when g_data_uart_settings.baud_est_use_original_buad_det_only //
                if (g_data_uart_settings.baud_est_baud_det_fail_redet_en &&
                    (!g_data_uart_settings.baud_est_use_original_buad_det_once) )
                {
                    hci_uart_set_baud_detect_mode(1, 0);
//                    UART_DWORD_WRITE(DMA_UART_STS_REG, UART_STS_REG_BAUD_DET_EN0);
//                    UART_DWORD_WRITE(DMA_UART_STS_REG, UART_STS_REG_BAUD_DET_EN1);            
                }
                else
                {
                    BAUDRATE_FORMAT_TYPE baud;
                    // recover to default baudrate setting //
                    UART_DWORD_WRITE(DMA_UART_STS_REG, sts_read & (~BIT0) & (~BIT3));
                    baud.d32 = 0; 
                    baud.b.d24 = g_baudRate;
                    uart_set_baud_clk_RTL8723(baud,0,0);
                }
                if (g_data_uart_settings.baud_est_use_original_buad_det_once)
                {
                    hci_uart_man_sram.baud_est_original_baud_det_done = 1;
                    // not return, to go to baud-est decision //
                }
            }
            else
            {
                if (g_data_uart_settings.baud_est_baud_det_trial_redet_en)
                {
//                    if ((hci_uart_man_sram.hci_uart_baud_est_state == HCI_UART_BAUD_EST_ST_TRIAL) &&
//                        (hci_uart_man_sram.hci_uart_baud_est_intr_cnt>=(1<<g_efuse_baud_est_setting_4.hci_uart_baud_est_h5init_retry_th)) )
                    if (hci_uart_man_sram.hci_uart_baud_est_intr_cnt>=(1<<g_efuse_baud_est_setting_4.hci_uart_baud_est_h5init_retry_th))
                    {
                        hci_uart_set_baud_detect_mode(1, 0);
//                        UART_DWORD_WRITE(DMA_UART_STS_REG, UART_STS_REG_BAUD_DET_EN0);
//                        UART_DWORD_WRITE(DMA_UART_STS_REG, UART_STS_REG_BAUD_DET_EN1);            
                    }                
                }
                if (g_data_uart_settings.baud_est_use_original_buad_det_once)
                {
                    BAUDRATE_FORMAT_TYPE baud;
                    // recover to default baudrate setting //
                    UART_DWORD_WRITE(DMA_UART_STS_REG, sts_read & (~BIT0) & (~BIT3));
                    baud.d32 = 0; 
                    baud.b.d24 = g_baudRate;
                    uart_set_baud_clk_RTL8723(baud,0,0);        
                    // not return, to go to baud-est decision //
                    hci_uart_man_sram.baud_est_original_baud_det_done = 1;
                }
            }
        }
        
        if (g_data_uart_settings.baud_est_use_original_buad_det_only) // TODO: to be extended for hybrid cases
        {
            return;
        }
    }
    
    // check validity of observed waveform characteristics //    
    
min_low_overflow = (uart_toggle_mon.b.uart_min_low_period == 0xFFF);
    min_falling_overflow = (uart_toggle_mon.b.uart_min_falling_space == 0xFFF);

    min_low_falling_low_bound = 16+(g_efuse_baud_est_setting_1.min_low_falling_udfl_th_opt<<4);
    min_low_underflow = (uart_toggle_mon.b.uart_min_low_period <= min_low_falling_low_bound);
    min_falling_underflow = (uart_toggle_mon.b.uart_min_falling_space <= min_low_falling_low_bound);
    falling_low_ratiox8 = (uart_toggle_mon.b.uart_min_falling_space<<3)/uart_toggle_mon.b.uart_min_low_period;


    if ( (falling_low_ratiox8 >= 24) || (falling_low_ratiox8 <= 12) ) // 3~1.5 is expected //
    {
        if (min_falling_overflow == 0) // don't check this item when falling space is overflow //
        {
            if (g_efuse_baud_est_setting_2.ignore_invalid_falling_low_ratio)
            {
                return;
            }
        }
    }

    actual_baud_est_combine_opt = g_efuse_baud_est_setting_2.baud_est_combine_opt;
    if (min_falling_overflow && (!min_low_overflow))
    {
        if (g_efuse_baud_est_setting_2.min_falling_ovfl_chg_to_min_low)
        {
            actual_baud_est_combine_opt = 0;
        }
    }
    else if (min_low_underflow && (!min_falling_underflow))
    {
        if (g_efuse_baud_est_setting_2.min_low_udfl_chg_to_min_falling)        
        {
            actual_baud_est_combine_opt = 1;
        }
    }

    // TODO: dynamic uart_clk?    
    if (min_falling_underflow && min_low_underflow)
    {
        actual_baud_est_combine_opt = g_efuse_baud_est_setting_2.baud_est_combine_opt_both_udfl;
    }


    // TODO: dynamic uart_clk?    
    if (min_falling_overflow && min_low_overflow)
    {
        return;
    }

    if ((hci_uart_h5_read_ln_state() == H5_STATE_INIT) && g_efuse_baud_est_setting_4.baud_est_by_low_period_at_h5_initialized)
    {
        actual_baud_est_combine_opt = 0;
    }
    

#ifdef _YL_H5_TEST_BAUD_DETECT_LOG_
    RT_BT_LOG(RED, YL_DBG_DEC_1, 1, uart_toggle_mon.b.mon_data_valid);
#endif

    if (uart_toggle_mon.b.mon_data_valid || g_data_uart_settings.buad_est_bypass_mon_valid_check)
    {
#ifdef _YL_H5_TEST_BAUD_DETECT_LOG_
        RT_BT_LOG(BLUE, YL_DBG_DEC_1, 1, actual_baud_est_combine_opt);
#endif    
        hci_uart_man_sram.est_min_low_period_record = uart_toggle_mon.b.uart_min_low_period;
        hci_uart_man_sram.est_min_falling_space_record = uart_toggle_mon.b.uart_min_falling_space;
        
        switch (actual_baud_est_combine_opt)
        {
            case 1:
                ovsr_div_x8_est = (uart_toggle_mon.b.uart_min_falling_space<<2);
                break;
            case 2:
                ovsr_div_x8_est = (uart_toggle_mon.b.uart_min_low_period<<2) + (uart_toggle_mon.b.uart_min_falling_space<<1);
                break;
            case 0:
            default:
                ovsr_div_x8_est = (uart_toggle_mon.b.uart_min_low_period<<3);
                break;
        }

        if (g_efuse_baud_est_setting_2.est_bias_sign)
        {
            ovsr_div_x8_est += (g_efuse_baud_est_setting_2.est_bias_value<<1);
        }
        else
        {
            ovsr_div_x8_est -= (g_efuse_baud_est_setting_2.est_bias_value<<1);
        }

        UINT32 ovsr_div_x8_err;
        baud_hw = hci_uart_read_hw_setting();
        switch(g_efuse_baud_est_setting_4.baud_est_opt)
        {
            case 0: // exhaustive search
                baud_new = hci_uart_baud_est_exhaustive(ovsr_div_x8_est, baud_hw);
                break;
            case 1: // limited detection
                baud_new = hci_uart_baud_est_detect(ovsr_div_x8_est, baud_hw, &ovsr_div_x8_err);
                break;
            case 2: // detection, if err>(est/2^(1+x)) ==> exhaustive search
                baud_new = hci_uart_baud_est_detect(ovsr_div_x8_est, baud_hw, &ovsr_div_x8_err);
                if (ovsr_div_x8_err)
                {
                    if (ovsr_div_x8_est <= (ABS(ovsr_div_x8_err)<<g_efuse_baud_est_setting_4.det_err_chg_to_est_th))
                    {
                        baud_new = hci_uart_baud_est_exhaustive(ovsr_div_x8_est, baud_hw);
                    }
                }
                break;
            case 3:
            default:
                break;
        }
#ifdef _YL_H5_TEST_BAUD_DETECT_LOG_
        RT_BT_LOG(RED, YL_DBG_HEX_2, 2, g_efuse_baud_est_setting_4.baud_est_opt,baud_new.d32);
#endif                    
        if (baud_hw.b.d24 != baud_new.b.d24) // only re-programs baudrate when it is to be modified //
        {
            uart_set_baud_clk_RTL8723(baud_new, 0, 0);
        }
        
    }
}


void hci_uart_baud_est_delay(void)
{
    if (g_efuse_baud_est_setting_4.baud_est_delay) 
    {
        pf_delay(1<<(g_efuse_baud_est_setting_4.baud_est_delay-1));
    }                
}

void hci_uart_falling_cnt_isr(UINT32 hci_dma_isr_status_d32) // TODO: to be completed
{
    DMA_UART_H5_INTSTS_REG_S_TYPE dma_uart_h5_intsts_reg;
    DMA_UART_H5_INTEN_REG_S_TYPE uart_h5_inten;
    UINT16 h5_ln_state;

#ifdef _ROM_CODE_PATCHED_
    /* Reserved com code patch by yilinli:
      * e.g. 
      */
    if (rcp_hci_uart_falling_cnt_isr_func != NULL)
    {
        if (rcp_hci_uart_falling_cnt_isr_func((void*)&hci_dma_isr_status_d32))
        {
            return;
        }
    }    
#endif
    
    dma_uart_h5_intsts_reg.d32 = UART_DWORD_READ(DMA_UART_H5_INTSTS);
    uart_h5_inten.d32 = UART_DWORD_READ(DMA_UART_H5_INTEN);
    
    if (dma_uart_h5_intsts_reg.b.uart_falling_cnt_intr_sts && uart_h5_inten.b.uart_falling_cnt_intr_en)
    {
        // W1C the interrupt status
        dma_uart_h5_intsts_reg.d32 &= (0xffff0000);
        dma_uart_h5_intsts_reg.b.uart_falling_cnt_intr_sts = 1;
        UART_DWORD_WRITE(DMA_UART_H5_INTSTS, dma_uart_h5_intsts_reg.d32);


        switch (hci_uart_man_sram.hci_uart_baud_est_state)
        {
            case HCI_UART_BAUD_EST_ST_START:
            case HCI_UART_BAUD_EST_ST_TRIAL:
//                if (hci_uart_man_sram.hci_uart_baud_est_state == HCI_UART_BAUD_EST_ST_TRIAL)
                {
                    if (hci_uart_man_sram.hci_uart_baud_est_intr_cnt != 0xFF)
                    {
                        hci_uart_man_sram.hci_uart_baud_est_intr_cnt++;
                    }
                }
            
                hci_uart_set_falling_cnt_intr_en(0);
                hci_uart_set_falling_cnt_th(g_efuse_baud_est_setting_4.baud_est_falling_cnt_th_2nd<<2);

                // TODO: shall avoid racing of H5 Link Status and Baud Est Interrupts

                if (g_data_uart_settings_2.h5_en)
                {  //***** H5 case ****//

                    if (g_data_uart_settings.baud_est_delay_before_h5_state_check)
                    {
                        hci_uart_baud_est_delay(); // wait for a while; to make sure the reception is finished //
                    }

                    // update h5 state; Reset the trial_cnt when entering H5_STATE_INIT //
                    h5_ln_state = hci_uart_h5_read_ln_state();

                    RT_BT_LOG(YELLOW, DMA_UART_163, 4,  hci_uart_man_sram.hci_uart_baud_est_state, 
                                                        hci_uart_man_sram.hci_uart_baud_est_intr_cnt, 
                                                        h5_ln_state, 
                                                        UART_DWORD_READ(DMA_UART_STS_REG));
                    
                    if ( (hci_uart_man.baud_est_h5_state_record != H5_STATE_INIT) &&
                            (h5_ln_state == H5_STATE_INIT) )
                    {
                        hci_uart_man_sram.hci_uart_baud_est_intr_cnt = 1;  
                    }

                    hci_uart_man.baud_est_h5_state_record = h5_ln_state;

                    
                    if (h5_ln_state == H5_STATE_UNINIT) // Both H5 SYNC and SYNC_RSP is good for estimation //
                    {
//                        if ((hci_uart_man_sram.hci_uart_baud_est_state == HCI_UART_BAUD_EST_ST_START) ||
//                            (hci_uart_man_sram.hci_uart_baud_est_intr_cnt >= (1<<g_efuse_baud_est_setting_4.hci_uart_baud_est_h5uninit_retry_th) )
                        {
                            if (!g_data_uart_settings.baud_est_delay_before_h5_state_check)
                            {
                                hci_uart_baud_est_delay();
                            }
                            hci_uart_set_toggle_mon_en(0);
                            hci_uart_baud_est_and_update();
                            if (hci_uart_man_sram.hci_uart_baud_est_state == HCI_UART_BAUD_EST_ST_START)
                            {
                                hci_uart_man_sram.hci_uart_baud_est_state = HCI_UART_BAUD_EST_ST_TRIAL;
                            }
                            hci_uart_set_toggle_mon_en(1);
                            hci_uart_man_sram.hci_uart_baud_est_intr_cnt = 0;
                        }
                        // TODO: ???
                    }
                    else if (h5_ln_state == H5_STATE_INIT)
                    {
//                        if (hci_uart_man_sram.hci_uart_baud_est_state != HCI_UART_BAUD_EST_ST_TRIAL) // to avoid dead-lock //
//                        {
//                            hci_uart_man_sram.hci_uart_baud_est_intr_cnt++;
//                        }
                        if (hci_uart_man_sram.hci_uart_baud_est_intr_cnt >= (1<<g_efuse_baud_est_setting_4.hci_uart_baud_est_h5init_retry_th))
                        {
                            if (g_efuse_baud_est_setting_1.baud_est_update_at_h5_initialized)
                            {
                                if (!g_data_uart_settings.baud_est_delay_before_h5_state_check)
                                {
                                    hci_uart_baud_est_delay();
                                }
                                hci_uart_set_toggle_mon_en(0);
                                hci_uart_baud_est_and_update();
                                if (hci_uart_man_sram.hci_uart_baud_est_state == HCI_UART_BAUD_EST_ST_START)
                                {
                                    hci_uart_man_sram.hci_uart_baud_est_state = HCI_UART_BAUD_EST_ST_TRIAL;
                                }
                                hci_uart_set_toggle_mon_en(1);
                                hci_uart_man_sram.hci_uart_baud_est_intr_cnt = 0;
                            }
                        }
                    }
                    else if (h5_ln_state == H5_STATE_ACTIVE)
                    {
                        // hci_uart_set_toggle_mon_en(0);
                        // hci_uart_set_toggle_mon_en(1);
                    }
                }
                else
                {  //***** H4 case ****//
                    // TODO:
                }
                
                if (g_efuse_baud_est_setting_3.w1c_at_fallint_cnt_intr_end)
                {
                    UART_DWORD_WRITE(DMA_UART_H5_INTSTS, dma_uart_h5_intsts_reg.d32);
                }
                hci_uart_set_falling_cnt_intr_en(1);                                
                
                break;
            case HCI_UART_BAUD_EST_ST_STOP:
                if (!g_efuse_baud_est_setting_1.baud_mon_en)
                {
                    hci_uart_set_toggle_mon_en(0);
                }
                hci_uart_set_falling_cnt_intr_en(0);
                break;
            default: 
                break;
        }

        // Decide whether keep monitoring/enabling interrupt

        // TODO:  refer to H4/H5 CMD_TX_FIFO, set_baud_received AND/OR uart_sync_received
        
        
#ifdef _YL_TEST_UART_BAUD_ESTIMATE_        
        hci_uart_set_toggle_mon_en(0);
        hci_uart_set_toggle_mon_en(1);
        RT_BT_LOG(YELLOW,DMA_UART_171, 1, dma_uart_h5_intsts_reg.d32);
#endif        
//        ....
    }
}
#endif 

/* for H4 error w1c after event RX */
void hci_uart_h4_err_event_recov_callback(TimerHandle_t timer_handle)
{
    /* free software timer */    
    if (timer_handle != NULL)
    {
        OS_DELETE_TIMER(&timer_handle);
    }    
    
    // TODO: option: UART H4 reset?
    
    /* w1c H4 ERROR */
    UINT32 uart_reg_status = UART_DWORD_READ(DMA_UART_STS_REG);
    UART_DWORD_WRITE(DMA_UART_STS_REG, uart_reg_status);
    
    /* re-endable H4 ERROR Interrupt */
    hci_uart_set_h4_error_interrupt_en(g_data_uart_settings.h4_err_intr_en);
    /* clear hci_uart_man.h4_err_event_wait_recov_timer_state */
    // TODO: need to block HCI RX packets before hci_uart_man.h4_err_event_wait_recov_timer_state = 0??
    hci_uart_man.h4_err_event_wait_recov_timer_state = 0;
#ifdef _YL_H4_TEST_SEND_LONG_BREAK_AFTER_ERR_CALLBACK
    hci_uart_send_long_break();
#endif
        
}

API_RESULT hci_uart_h4_err_event_complete_check(UCHAR *buffer)
{
#ifdef _ROM_CODE_PATCHED_
    /* Reserved com code patch by yilinli:
      * e.g. 
      *    1. 
      */    
    if (rcp_hci_uart_h4_err_event_func != NULL)
    {
        return rcp_hci_uart_h4_err_event_func((void*)buffer);
    }
#endif

    if ((g_fun_interface_info.b.bt_interface != UART_INTERFACE) || (g_data_uart_settings_2.h5_en))
    {
        return API_SUCCESS;
    }
    if (g_data_uart_settings_3.err_send_event_delayw1c_opt >= 2)
    {
        if (hci_uart_man.h4_err_event_wait_rx_state)
        {
            if (buffer[0] == HCI_HARDWARE_ERROR_EVENT && 
                    buffer[1] == HCI_HARDWARE_ERROR_EVENT_LEN &&
                    buffer[2] == hci_uart_man.h4_hw_err_event_code)
            {
                hci_uart_man.h4_err_event_wait_rx_state = 0;

                /* create a timer for DELAYED "h4 error w1c AND interrupt enable" */
                TimerHandle_t err_event_recov_timer_id = NULL;
                const UINT32 TIME_MS = ((UINT32)10) << (g_data_uart_settings_3.err_send_event_delayw1c_opt-2); 

                if (OS_CREATE_TIMER(ONESHOT_TIMER, &err_event_recov_timer_id,
                        hci_uart_h4_err_event_recov_callback, NULL, 0) != BT_ERROR_OK)
                {
                    return API_FAILURE;
                }
                DMA_UART_LOG(GRAY,DMA_UART_122,1, (UINT32)err_event_recov_timer_id);    
                OS_START_TIMER(err_event_recov_timer_id, TIME_MS);
            }
        }    
    }
    return API_SUCCESS;
}            

/* return API_SUCCESS: no effect or change baudrate successfuly
  * return API_FAILURE: change baudrate fail
  */
API_RESULT hci_uart_change_baudrate_event_complete_check(UCHAR *buffer)
{
#ifdef _ROM_CODE_PATCHED_
    /* Reserved com code patch by yilinli:
      * e.g. 
      *    1. 
      */    
    if (rcp_hci_uart_change_baudrate_event_func != NULL)
    {
        return rcp_hci_uart_change_baudrate_event_func((void*)buffer);
    }
#endif    

    if ((g_fun_interface_info.b.bt_interface != UART_INTERFACE))
    {
        return API_SUCCESS;
    }
    if(!hci_uart_man.baud_new_valid_flag)
        return API_SUCCESS;
     

     UINT16 opcode = ((buffer[4]<<8) + buffer[3]);
     API_RESULT set_baud_new_results = 0;
     DMA_UART_DBG_LOG(GREEN, DMA_UART_003_DBG_HEX, 1, buffer[0]);
     DMA_UART_DBG_LOG(GREEN, DMA_UART_003_DBG_HEX, 1, opcode);
//     DMA_UART_DBG_LOG(GREEN, DMA_UART_127, 2, hci_uart_man.baud_new.d32, set_baud_new_results);
     if (buffer[0] == HCI_COMMAND_COMPLETE_EVENT && 
         opcode == HCI_VENDOR_SET_BAUDRATE)
     {
        // NOTE: this section is run at ISR //
        hci_uart_man.baud_new_valid_flag = 0;
#ifdef _UART_BAUD_ESTIMATE_
        hci_uart_man_sram.hci_vendor_set_baud_executed = 1;
        hci_uart_man_sram.hci_uart_baud_est_state = HCI_UART_BAUD_EST_ST_STOP;
#endif
        // The following code will re-new hci_uart_man_sram.baud_current_setting //
        set_baud_new_results = hci_uart_vendor_set_baud(hci_uart_man.baud_new);
        DMA_UART_LOG(GREEN, DMA_UART_127, 2, hci_uart_man.baud_new.d32, set_baud_new_results);
     }

    return set_baud_new_results;
}           
#endif

#ifdef _8821A_BTON_DESIGN_
API_RESULT hci_uart_change_parameter_event_complete_check(UCHAR *buffer)
{
    HCI_EVENT_PKT *event_pkt= (HCI_EVENT_PKT*) buffer;
    UINT16 opcode = ((buffer[4]<<8) + buffer[3]);
    BAUDRATE_FORMAT_TYPE baud_now;
    
#ifdef _YL_H5_TEST_UART_PARA_CHG
    RT_BT_LOG(GREEN, YL_DBG_HEX_20, 20, opcode, buffer[0], buffer[1], buffer[2], buffer[3], buffer[4],
                                        buffer[5], buffer[6], buffer[7], buffer[8], buffer[9], 
                                        buffer[10], buffer[11], buffer[12], buffer[13], buffer[14],
                                        buffer[15], buffer[16], buffer[17], buffer[18]);
#endif    
    if (buffer[0] == HCI_COMMAND_COMPLETE_EVENT && opcode == HCI_VENDOR_UART_PARA_CHANGE)
    {
        *(UINT32*)&g_data_uart_settings = get_uint32_from_uint8_array(&(event_pkt->event_parameter[5]));
        *(UINT32*)&g_data_uart_settings_2 = get_uint32_from_uint8_array(&(event_pkt->event_parameter[9]));
        *(UINT32*)&g_data_uart_settings_3 = get_uint32_from_uint8_array(&(event_pkt->event_parameter[13]));
        if (event_pkt->event_parameter[4] & BIT6)
        {
            baud_now = hci_uart_read_hw_setting();
        }
        if (event_pkt->event_parameter[4] & BIT7)
        {
            dma_init(INIT_RUN_TIME);
        }
        switch (event_pkt->event_parameter[4] & 0x0F)
        {
            case 1:
                hci_uart_reset_init_RTL8723(1);
                break;
            default:
                hci_uart_reset_init_RTL8723(0);
                break;
        }        
        if (event_pkt->event_parameter[4] & BIT6)
        {
            uart_set_baud_clk_RTL8723(baud_now, 0, 0);
        }        
        // NOTE: this section is run at ISR //
        hci_uart_man.chg_para_valid_flag = 0;        
    }

    return API_SUCCESS;
}     
#endif

#ifdef _UART_H5
#ifdef _YL_TEST_UART_BAUD_ESTIMATE_
void hci_uart_baud_est_test_log(UINT32 dbg_count)
{
    DMA_UART_H5_INTSTS_REG_S_TYPE uart_h5_intsts;
    DMA_UART_H5_INTEN_REG_S_TYPE uart_h5_inten;
    DMA_UART_TOGGLE_MON_CTRL_REG_S_TYPE uart_toggle_mon;
    uart_h5_intsts.d32 = UART_DWORD_READ(DMA_UART_H5_INTSTS);
    uart_h5_inten.d32 = UART_DWORD_READ(DMA_UART_H5_INTEN);
    uart_toggle_mon.d32 = UART_DWORD_READ(DMA_UART_TOGGLE_MON_CTRL);

    RT_BT_LOG(GREEN, DMA_UART_162, 10, 
                        uart_h5_inten.b.uart_falling_cnt_intr_en,
                        uart_h5_intsts.b.uart_falling_cnt_intr_sts, 
                        uart_toggle_mon.b.uart_toggle_mon_en,
                        uart_toggle_mon.b.mon_data_valid,
                        uart_toggle_mon.b.uart_min_falling_space,
                        uart_toggle_mon.b.uart_min_low_period,
                        uart_toggle_mon.b.uart_falling_cnt_th,
                        hci_uart_man_sram.hci_vendor_uart_sync_received,
                        hci_uart_man_sram.hci_vendor_set_baud_executed,
                        hci_uart_man_sram.hci_vendor_set_baud_received);
                        
    if (dbg_count==0)
    {
        hci_uart_set_toggle_mon_en(0);
        hci_uart_set_toggle_mon_en(1);   
        hci_uart_set_toggle_mon_en(0); // clear and hold falling cnt
        hci_uart_set_falling_cnt_th(32);
        hci_uart_set_falling_cnt_intr_en(1);
        hci_uart_set_toggle_mon_en(1);
    }
#if 0
    if (uart_h5_intsts.b.uart_falling_cnt_intr_sts)
    {
        hci_uart_set_toggle_mon_en(0);
        hci_uart_set_toggle_mon_en(1);        
    }
#endif

    if ( (dbg_count%10) == 9)
    {
        if (0)
        {
            WDG_TIMER_TIMEOUT_SOON;
            pf_delay(1000);
        }
        else if (0)
        {        
            enable_uart_suspend();
        }
        else if (0)
        {  /* new */
            UINT32 clock, wakeup_instant;
            bton_clr_wakeup_sts();
            DEF_CRITICAL_SECTION_STORAGE;
            MINT_OS_ENTER_CRITICAL();        
            clock = BB_read_native_clock();
            clock = clock >> 1;
            wakeup_instant = clock  + 0xff00;
            wakeup_instant = wakeup_instant & 0xFFFF;   
            execute_lps_mode_procedure_6128(1, wakeup_instant);
            MINT_OS_EXIT_CRITICAL();
#ifdef LPS_PROTECT_DSM_BB_ACCESS // YL TEST 2
            {
                while(sleep_mode_param.bb_sm_sts != BB_NORMAL_MODE)
                {
                    pf_delay_us(1);
                }
            }
#endif                    
        }     
    }
}
#endif


#ifdef _YL_H5_TEST
//1 For Test Only 
void hci_uart_h5_test_at_keep_alive_event(void)
{
    DMA_UART_H5_INTSTS_REG_S_TYPE dma_uart_h5_intsts_reg;
    H5_HW_STATE_TYPE h5_hw_state;
    UINT32 cmd_free, acl_free, sco_free;
    
    dma_uart_h5_intsts_reg.d32 = UART_DWORD_READ(DMA_UART_H5_INTSTS);
    h5_hw_state.d16=dma_uart_h5_intsts_reg.b.h5_current_st_d16;        
    DMA_UART_DBG_LOG(WHITE,DMA_UART_050, 11,
                                        h5_hw_state.b.tx_seq_num,
                                        h5_hw_state.b.tx_ack_num,  
                                        h5_hw_state.b.ln_state,
                                        h5_hw_state.b.sliding_wsize,
                                        h5_hw_state.b.oof_fctrl,
                                        h5_hw_state.b.data_chec_en,
                                        h5_hw_state.b.reserved_13_15,
                                        dma_uart_h5_intsts_reg.d32, 
                                        UART_DWORD_READ(DMA_UART_H5_CTL0),
                                        UART_DWORD_READ(DMA_UART_H5_CTL1),
                                        DMA_DWORD_READ(ISR_STATUS_REG));
    DMA_UART_DBG_LOG(WHITE,DMA_UART_092, 3, hci_uart_man.h5_retry_alarm_state,         
                                    hci_uart_man.h5_retry_fail_state, 
                                    hci_uart_man.h5_sleep_msg_state);
                                            
#if 1                                                                                
        cmd_free = DMA_DWORD_READ(  CMD_TX_BUF_FREE_RX_START_CTL_REG);
        acl_free = DMA_DWORD_READ(  ACL_TX_BUF_FREE_REG);
        sco_free = DMA_DWORD_READ(  SCO_TX_BUF_FREE_REG);
        if(cmd_free && acl_free && sco_free )
            DMA_UART_DBG_LOG(WHITE,DMA_UART_072,3, cmd_free, acl_free, sco_free);
        else
            DMA_UART_DBG_LOG(RED,DMA_UART_072,3, cmd_free, acl_free, sco_free);              
#endif        

#ifdef _YL_H5_TEST_SEND_SHORT_BREAK_WHEN_ALIVE
          hci_uart_h5_send_short_break();
#endif
#ifdef _YL_H5_TEST_SEND_LONG_BREAK_WHEN_ALIVE
          hci_uart_send_long_break();
#endif
#ifdef _YL_H5_TEST_SEND_SLEEP_WHEN_ALIVE
          hci_uart_h5_send_sleep_msg();
#endif
#ifdef _YL_H5_TEST_SEND_SLEEP_WAIT_WHEN_ALIVE
          hci_uart_h5_send_sleep_msg_wait_done();
#endif
#ifdef _YL_H5_TEST_POLL_WAKE_WHEN_ALIVE
        hci_uart_h5_poll_wake();
#endif
#ifdef _YL_H5_TEST_SET_BAUD_WHEN_ALIVE
        BAUDRATE_FORMAT_TYPE baud_new;
        baud_new.d32 = 0;
        baud_new.b.d24 = BAUD_FORMAT_D24_921600;            
        hci_uart_vendor_set_baud(baud_new);
#endif 
#ifdef _YL_H5_TEST_ALTERNATE_BAUD_WHEN_ALIVE
        BAUDRATE_FORMAT_TYPE baud_new;
        baud_new.d32 = 0;
        if(h5_baud_alternate_flag)
            baud_new.b.d24 = BAUD_FORMAT_D24_921600;            
        else
            baud_new.b.d24 = BAUD_FORMAT_D24_115200;            
        hci_uart_vendor_set_baud(baud_new);
        h5_baud_alternate_flag=!h5_baud_alternate_flag;
#endif 
#ifdef _YL_H5_TEST_STOP_RESTORE_WHEN_ALIVE_2
        DEF_CRITICAL_SECTION_STORAGE;    
        MINT_OS_ENTER_CRITICAL();    
        UINT16 dma_not_idle = (DMA_DWORD_READ(CMD_TX_BUF_FREE_RX_START_CTL_REG)!=0xf)
                                                        || (DMA_DWORD_READ(  ACL_TX_BUF_FREE_REG)!=0xff)
                                                        || (DMA_DWORD_READ(  SCO_TX_BUF_FREE_REG)!=0xffff);
        h5_go_sleep_test_times++;                                                            
        if(!dma_not_idle)
        {
              h5_go_sleep_times++;            
              API_RESULT h5_go_stop_result = hci_uart_h5_go_sleep(); /* by go_park or go_idle(recommended) */
              if(h5_go_stop_result == API_SUCCESS)
              {      
                    h5_go_sleep_succ_times ++;
                    dma_not_idle = (DMA_DWORD_READ(CMD_TX_BUF_FREE_RX_START_CTL_REG)!=0xf)
                                                        || (DMA_DWORD_READ(  ACL_TX_BUF_FREE_REG)!=0xff)
                                                        || (DMA_DWORD_READ(  SCO_TX_BUF_FREE_REG)!=0xffff);                        
                    if (dma_not_idle)
                    {
                        h5_go_sleep_dma_fifo_conflict_times++;
                    }
                    hci_uart_h5_test_stop_reset_restore();
              }
        }
        MINT_OS_EXIT_CRITICAL();    
        DMA_UART_DBG_LOG(GRAY, DMA_UART_091, 6, 
                                    h5_go_sleep_test_times, 
                                    h5_go_sleep_times, 
                                    h5_go_sleep_succ_times,
                                    h5_go_sleep_dma_triggered_times,
                                    h5_go_sleep_isr_status_times,
                                    h5_go_sleep_dma_fifo_conflict_times);
#endif
}


void hci_uart_h5_test_stop_reset_restore(void)
{
    DMA_UART_H5_INTSTS_REG_S_TYPE dma_uart_h5_intsts_reg;
    DMA_UART_H5_INTEN_REG_S_TYPE dma_uart_h5_inten_reg;
    H5_HW_STATE_TYPE h5_hw_state;
    
    /* STORE the status for H5 */
    dma_uart_h5_intsts_reg.d32 = UART_DWORD_READ(DMA_UART_H5_INTSTS);
    h5_hw_state.d16=dma_uart_h5_intsts_reg.b.h5_current_st_d16;

    /* emulate uart_reinit h5_reinit*/
    
    /* RESTORE the status for H5 */
    // TODO:
     /*  (uart_hci_en = 0 ?) */
     /*  set UART clock */
     /*  wait uart pll settle */
     /*  uart_hci_en = 1 */
     /*  set uart divisor, ovsr, ... */
     /*  set uart H5 configuration */
     /*  write backup_ln_state   BY  hci_uart_h5_set_hw_backup_state(); */
     /*  restore hci_uart_man.h5_sleep_msg_state and TIMING parameters */
     /*  set UART H5 enable */
    
    #if 0 // modeling hardware reset
        hci_uart_set_hci_mode_en(0);
        hci_uart_set_h5_en(0);
    #else                            
        BAUDRATE_FORMAT_TYPE baud_temp;
        baud_temp.d32 = 0;
        baud_temp.b.d24 = BUAD_FORMAT_DEFAULT;
        uart_set_baud_clk_RTL8723(baud_temp,0,1);
        hci_uart_h5_config_init();
        
        #ifdef _YL_H5_UART_DBG_PORT_EN // for debug port
            VENDOR_WRITE(0x30, (VENDOR_READ(0x30)&BIT31) | 0xc0);
            UART_DWORD_WRITE(0x1c, _YL_H5_UART_DBG_PORT);
        #endif
        #ifdef _YL_H5_DMA_DBG_PORT_EN // for debug port
            VENDOR_WRITE(0x30, (VENDOR_READ(0x30)&BIT31) | 0xe0);
            DMA_DWORD_WRITE(0x80, _YL_H5_DMA_DBG_PORT);
        #endif                                    
    #endif
    
    dma_uart_h5_inten_reg.d32 = UART_DWORD_READ(DMA_UART_H5_INTEN);
    h5_hw_state.b.reserved_13_15 = 0;
    dma_uart_h5_inten_reg.b.h5_backup_st_d16 = h5_hw_state.d16;
    UART_DWORD_WRITE(DMA_UART_H5_INTEN,dma_uart_h5_inten_reg.d32);
    hci_uart_set_h5_en(1);
    hci_uart_set_hci_mode_en(1);
    
    /* emulate the restoration from LPS mode */
    // TODO: need to jointly considered with UART/H5_INIT, LPS Restortion
    // TODO: included as a single function

    //hci_uart_h5_abort_sleep();                 
}

#endif
#endif


