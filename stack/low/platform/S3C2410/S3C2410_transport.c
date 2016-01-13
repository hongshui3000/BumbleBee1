/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/** 
 * \file S3C2410_transport.c
 *  Implements the transport interface which meets the BlueWiz Reference
 *  Platform transport interface.
 * 
 * \author Santhosh kumar M
 * \date 2006-10-07
 */
enum { __FILE_NUM__= 103 };

#ifndef NULL_HCI_TRANSPORT

#include "platform.h"
#include "uart.h"
#include "logger.h"
#include "S3C2410_transport.h"
#include "dma_usb.h"
#include "gpio.h"
#ifdef _ENABLE_BTON_POWER_SAVING_
#include "power_control.h"
#endif
#include "h5.h"

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
#include "lc_internal.h"
#endif

#if defined(_SUPPORT_FW_INDIRECT_READ_SIE_)
#include "new_io.h"
#endif

#ifdef _SUPPORT_POLLING_BASED_LPM_L1_
extern LPM_POLL_CONTROL g_lpm_poll_control_s;
#endif

#ifdef _SUPPORT_USB_LOG_ENABLE_
extern UINT8 g_usb_misc;
#endif

#ifdef _SUPPORT_INFO_FROM_SYSTEM_ON_
#include "system_on.h"
#endif

void pf_hci_transport_init(void)
{

#ifdef _RTK8723_INTERFACE_INFO_
#ifndef _SUPPORT_3BITS_HCI_SELECTION_
    POW_OPTION_AND_32K_CTRL_S_TYPE pow_option;
    BTON_INTERFACE_CTRL_REG_S_TYPE bton_interface_info;

    pow_option.d32 = VENDOR_READ(BTON_POW_OPTION_AND_32K_CTRL);
    bton_interface_info.d32  = VENDOR_READ(BTON_INTERFACE_CTRL_REG);

    /* to fixed RTL8723A bton_interface_info.b.bt_fen_sts BUG */
    if (bton_interface_info.b.hci_uart_fen_sts)
    {
        bton_interface_info.b.bt_fen_sts = 1;
    }

    g_fun_interface_info.b.hci_sel = pow_option.b.hci_selection;
    g_fun_interface_info.b.bt_fun_sts= bton_interface_info.b.bt_fen_sts;
    

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
    if(sleep_mode_param.dlps_restore_flow == FALSE)
#endif
    {
         RT_BT_LOG(BLUE,PACKAGE_TO_HCI,1,g_fun_interface_info.b.hci_sel);
    }

    
    switch (pow_option.b.hci_selection)
    {
        //case RTK8723_S:
        case PACKAGE_SDIO_UART:
#ifndef _BT_ONLY_            
        case PACKAGE_PCIE_UART:
#endif            
            g_fun_interface_info.b.bt_interface = UART_INTERFACE;
#ifndef _8821A_BTON_DESIGN_            
            g_fun_interface_info.b.bt_fun_sts= bton_interface_info.b.bt_fen_sts;
            g_fun_interface_info.b.gps_fun_sts = 0;
#endif            
            //RT_BT_LOG(BLUE,RTK8723_SDIO_UART,1,pow_option.b.hci_selection);
            break;
        //case RTK8723_U:
        case PACKAGE_USB_USB:
            g_fun_interface_info.b.bt_interface = USB_INTERFACE;
#ifndef _8821A_BTON_DESIGN_            
            g_fun_interface_info.b.bt_fun_sts= bton_interface_info.b.bt_fen_sts;
            g_fun_interface_info.b.gps_fun_sts = bton_interface_info.b.usb_gps_fen_sts;
            g_fun_interface_info.b.gps_interface = USB_INTERFACE;            
#endif            
            //RT_BT_LOG(BLUE,RTK8723_USB_MULTI,1,pow_option.b.hci_selection);
            break;

#ifndef _REMOVE_HCI_PCIE_
#if !defined(_RTL8703B_SPECIFIC_) && !defined(_RTL8822B_SPECIFIC_) 
        case RTK8723_EE:
            g_fun_interface_info.b.bt_interface = PCIE_INTERFACE;
#ifndef _8821A_BTON_DESIGN_                        
            g_fun_interface_info.b.bt_fun_sts= bton_interface_info.b.bt_fen_sts;
            g_fun_interface_info.b.gps_fun_sts = bton_interface_info.b.pcie_gps_fen_sts;
            g_fun_interface_info.b.gps_interface = PCIE_INTERFACE;            
#endif            
            //RT_BT_LOG(BLUE,RTK8723_PCIE_MULTI,1,pow_option.b.hci_selection);
            break;
#else
        case RTK8822B_ME:
            g_fun_interface_info.b.bt_interface = USB_INTERFACE;
            break;
#endif
#endif

#if defined(_BT_ONLY_) || (!defined(_RTL8703B_SPECIFIC_) && !defined(_RTL8822B_SPECIFIC_)) 
        //case RTK8723_E:
        case PACKAGE_PCIE_USB:
            g_fun_interface_info.b.bt_interface = USB_INTERFACE;
#ifndef _8821A_BTON_DESIGN_                        
            g_fun_interface_info.b.bt_fun_sts= bton_interface_info.b.bt_fen_sts;
            g_fun_interface_info.b.gps_fun_sts = bton_interface_info.b.pcie_gps_fen_sts;
            g_fun_interface_info.b.gps_interface = PCIE_INTERFACE;
#endif            
            //RT_BT_LOG(BLUE,RTK8723_PCIE_USB,1,pow_option.b.hci_selection);
            break;
#endif

        default:
            break;
    }
#else

#if defined(_BT_ONLY_)//dape mark since seems all ruled by _BT_ONLY ||!defined(_SUPPORT_INFO_FROM_SYSTEM_ON_)
    POW_OPTION_AND_32K_CTRL_S_TYPE pow_option;
    BTON_INTERFACE_CTRL_REG_S_TYPE bton_interface_info;
    pow_option.d32 = VENDOR_READ(BTON_POW_OPTION_AND_32K_CTRL);
    bton_interface_info.d32  = VENDOR_READ(BTON_INTERFACE_CTRL_REG);
    g_fun_interface_info.b.hci_sel = pow_option.b.hci_selection;    
#else
    //PAGE0_REG_0xF5_BYTE_READ_S  page0_hci_sel;
    BTON_INTERFACE_CTRL_REG_S_TYPE bton_interface_info;
    //page0_hci_sel.d8 = RD_8BIT_COMBO_SYSON_IO(PAGE0_REG_0xF5);
    bton_interface_info.d32  = VENDOR_READ(BTON_INTERFACE_CTRL_REG);
    //g_fun_interface_info.b.hci_sel = page0_hci_sel.b.hci_selection; 
    g_fun_interface_info.b.hci_sel = getfinalhci(); 
    
#endif

     /* to fixed RTL8723A bton_interface_info.b.bt_fen_sts BUG */
     if (bton_interface_info.b.hci_uart_fen_sts)
     {
         bton_interface_info.b.bt_fen_sts = 1;
     }
     
     g_fun_interface_info.b.bt_fun_sts= bton_interface_info.b.bt_fen_sts;

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
    if(sleep_mode_param.dlps_restore_flow == FALSE)
#endif
    {
         RT_BT_LOG(BLUE,PACKAGE_TO_HCI,1,g_fun_interface_info.b.hci_sel);
    }
  
     switch (g_fun_interface_info.b.hci_sel)
     {
         case PACKAGE_SDIO_UART:
         case PACKAGE_PCIE_UART:
#ifdef _RTL8822B_SPECIFIC_            
         case PACKAGE_MPCIE_UART:
#endif            
             g_fun_interface_info.b.bt_interface = UART_INTERFACE;
#ifndef _8821A_BTON_DESIGN_            
             g_fun_interface_info.b.bt_fun_sts= bton_interface_info.b.bt_fen_sts;
             g_fun_interface_info.b.gps_fun_sts = 0;
#endif            
             //RT_BT_LOG(BLUE,RTK8723_SDIO_UART,1,pow_option.b.hci_selection);
             break;
         case PACKAGE_USB_USB:
#ifdef _RTL8822B_SPECIFIC_            
         case PACKAGE_MPCIE_USB:
#endif            
         case PACKAGE_PCIE_USB:    
             g_fun_interface_info.b.bt_interface = USB_INTERFACE;
#ifndef _8821A_BTON_DESIGN_            
             g_fun_interface_info.b.bt_fun_sts= bton_interface_info.b.bt_fen_sts;
             g_fun_interface_info.b.gps_fun_sts = bton_interface_info.b.usb_gps_fen_sts;
             g_fun_interface_info.b.gps_interface = USB_INTERFACE;            
#endif            
             //RT_BT_LOG(BLUE,RTK8723_USB_MULTI,1,pow_option.b.hci_selection);            
             break;
/* 
#if defined(_BT_ONLY_)||!defined(_RTL8703B_SPECIFIC_)
         case RTK8723_E:
             g_fun_interface_info.b.bt_interface = USB_INTERFACE;
#ifndef _8821A_BTON_DESIGN_                        
             g_fun_interface_info.b.bt_fun_sts= bton_interface_info.b.bt_fen_sts;
             g_fun_interface_info.b.gps_fun_sts = bton_interface_info.b.pcie_gps_fen_sts;
             g_fun_interface_info.b.gps_interface = PCIE_INTERFACE;
#endif            
             //RT_BT_LOG(BLUE,RTK8723_PCIE_USB,1,pow_option.b.hci_selection);
             break;
#endif
*/ 
         default:
             RT_BT_LOG(RED,PACKAGE_ERROR_SELECTION,0,0);
             break;
     }



#endif /* end of #else _SUPPORT_3BITS_HCI_SELECTION_ */


#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
    if(sleep_mode_param.dlps_restore_flow == FALSE)
#endif
    {
#ifndef _8821A_BTON_DESIGN_                                                                    
        RT_BT_LOG(BLUE,RTK8723_INTERFACE_INFO,4,g_fun_interface_info.b.bt_fun_sts,
                                            g_fun_interface_info.b.bt_interface,
                                            g_fun_interface_info.b.gps_fun_sts,
                                            g_fun_interface_info.b.gps_interface);
#else    
        RT_BT_LOG(BLUE,RTK8723_INTERFACE_INFO2,5,g_fun_interface_info.b.bt_fun_sts,
                                            g_fun_interface_info.b.bt_interface,
                                            bton_interface_info.b.bt_fen_sts,
                                            bton_interface_info.b.hci_uart_fen_sts,
                                            bton_interface_info.b.gpio_reset_rf_sts);
#endif
    }

    if (bton_interface_info.b.hci_uart_fen_sts)
    {
         g_fun_interface_info.b.bt_interface = UART_INTERFACE;
//         RT_BT_LOG(BLUE,DATA_UART_EN,0,0);
    }


#else

#ifdef _ENABLE_UART_DMA_
    g_fun_interface_info.b.bt_interface = UART_INTERFACE;
#elif _ENABLE_PCIE_DMA_
#ifndef _REMOVE_HCI_PCIE_ 
    g_fun_interface_info.b.bt_interface = PCIE_INTERFACE;
#endif

#else
    g_fun_interface_info.b.bt_interface = USB_INTERFACE;
#endif

#endif

    /*	Added by Wallice Su for USB LPM.	2012/03/01	*/
    /* be ware of besl or hird */
    // 0xFE65[3]: BESEL enable
    // 0xFE65[4]: Baseline BESL value
    // 0xFE65[5]: deep BESL value
    //RT_BT_LOG(BLUE, MSG_USBLPM_REG,1,bton_USB_LPM_reg.d32);

    if ((g_fun_interface_info.b.bt_interface == USB_INTERFACE))
    {
    
#ifdef _SUPPORT_USB_LOG_ENABLE_
        // force to NYET first
        g_usb_misc = otp_str_data.USB_misc;
#endif

        //RT_BT_LOG(YELLOW, YL_DBG_HEX_1, 1,g_usb_misc);
    
        BTON_USB_LPM_REG_S_TYPE bton_USB_LPM_reg;    
    	bton_USB_LPM_reg.d32 = 0x0;

#ifndef _SUPPORT_POLLING_BASED_LPM_L1_   
        if (otp_str_data.USB_LPM_Allow == 0x1)
        {
       	    bton_USB_LPM_reg.b.LPM_Allow = 1;
        }            
#else

        g_lpm_poll_control_s = (LPM_POLL_CONTROL)(otp_str_data.USB_LPM_Control);
        bton_USB_LPM_reg.b.LPM_Allow = g_lpm_poll_control_s.b.lpm_en;
#ifdef _SUPPORT_FW_INDIRECT_READ_SIE_
        USB_SIE_0x65_REG_S_TYPE sieReg0x65;
        sieReg0x65.d8 = (UINT8)safe_indirect_read_sie(READ_SIE_BYTE, SIE_REG_0x65);
        if( sieReg0x65.b.besl_en != 0)
        {   // BESL
            if(sieReg0x65.b.baseline_besl_valid != 0 )
            {   // set baseline BESL to bton reg
                USB_SIE_0x7A_REG_S_TYPE sieReg0x7A;
                sieReg0x7A.d8 = safe_indirect_read_sie(READ_SIE_BYTE, SIE_REG_0x7A);
                 *((unsigned char *)&bton_USB_LPM_reg) = sieReg0x7A.b.baseline_besl_value;//from wifi efuse
            }
        }
        else
#endif            

#endif

        {   // HIRD
            *((unsigned char *)&bton_USB_LPM_reg) = otp_str_data.USB_LPM_HIRDBESL_Thrd;// from bt efuse
        }
	    
	    VENDOR_WRITE(BTON_USB_LPM, bton_USB_LPM_reg.d32);// for hird
        

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
        if(sleep_mode_param.dlps_restore_flow == FALSE)
#endif
        {
            RT_BT_LOG(BLUE, MSG_USBLPM_REG,1,bton_USB_LPM_reg.d32);	
        }			
    }
/*	End Added by Wallice Su for USB LPM.	2012/03/01	*/


    g_baudRate = otp_str_data.SYS_hci_uart_baudrate;

#ifdef _RTK8723_UART_INIT_
    // New EFUSE parameters
    *(UINT32*)&g_data_uart_settings = otp_str_data.rtl8723_data_uart_settings;
#ifdef _UART_H5
    *(UINT32*)&g_data_uart_settings_2 = otp_str_data.rtl8723_data_uart_settings_2;
    *(UINT32*)&g_data_uart_settings_3 = otp_str_data.rtl8723_data_uart_settings_3;
#ifdef _UART_BAUD_ESTIMATE_
    g_efuse_baud_est_setting_1.d16 = otp_str_data.efuse_baud_est_setting_1_d16;
    g_efuse_baud_est_setting_2.d16 = otp_str_data.efuse_baud_est_setting_2_d16;
    g_efuse_baud_est_setting_3.d16 = otp_str_data.efuse_baud_est_setting_3_d16;
    g_efuse_baud_est_setting_4.d16 = otp_str_data.efuse_baud_est_setting_4_d16;
#endif

#ifdef _UART_H5_FPGA_EFUSE_FORCE
   // TODO: To be included into EFUSE
        g_data_uart_settings.baud_default_en = 0;
   #ifdef _YL_H5_MODE_ENABLE
       g_data_uart_settings_3.parity_en = 1; // 1b
       g_data_uart_settings_3.parity_even = 1; // 1b
       g_data_uart_settings_3.hw_fctrl_on = 0; // 1b    
   #else
       /* H4 Mode EFUSE Setting */
        g_data_uart_settings_3.parity_en = 0; // 1b
        g_data_uart_settings_3.parity_even = 0; // 1b
        #ifdef _IN_BQB_
        g_data_uart_settings_3.hw_fctrl_on = 0; // 1b
        #else
        g_data_uart_settings_3.hw_fctrl_on = 1; // 1b
        #endif
   #endif   
    g_data_uart_settings_3.long_break_duration = 3; // 3b; 2^(x)*10ms

    #ifdef _YL_H5_MODE_ENABLE
        g_data_uart_settings_2.h5_en = 1; // 1b
   #else
       /* H4 Mode EFUSE Setting */
        g_data_uart_settings_2.h5_en = 0; // 1b
    #endif
    g_data_uart_settings_3.h5_poll_wake_duration = 5; // 3b; 7: persistent poll; else: 2^(2x)*10ms
    g_data_uart_settings_3.h5_retry_alarm_opt = H5_WAKEUP_OPT_NONE; // 2b; 0: nothing, 1: sent short break, 2: send long break; 3: poll_wake
    g_data_uart_settings_3.h5_retry_fail_opt = H5_WAKEUP_OPT_NONE; // 2b; 0: nothing, 1: sent short break, 2: send long break; 3: poll_wake
    g_data_uart_settings_3.h5_lowpow_wakeup_opt = H5_WAKEUP_OPT_NONE; // 2b; if(sleep_msg_state==1 && RX is to be send to HCI DMA), 0: nothing, 1: sent short break, 2: send long break; 3: poll_wake
    #ifdef _YL_H5_FORCE_SLEEP_MSG_EN
        g_data_uart_settings_3.h5_lowpow_sleep_msg_en = 1; // 1b; 1: send_sleep_msg() before go to sleep
    #else
        g_data_uart_settings_3.h5_lowpow_sleep_msg_en = 0; // 1b; 1: send_sleep_msg() before go to sleep
    #endif

    #ifdef _YL_H5_FORCE_SCO_REL
        g_data_uart_settings_3.h5_scounrel_force_en = 1; //  required? 1b 
        g_data_uart_settings_3.h5_scounrel_force_value = 0; // required? 1b; valid when force_en = 1, 0: reliable, 1: unreliable
    #else
        g_data_uart_settings_3.h5_scounrel_force_en = 0; // 1b
        g_data_uart_settings_3.h5_scounrel_force_value = 0; // 1b; valid when force_en = 1, 0: reliable, 1: unreliable
    #endif
   
    g_data_uart_settings_3.h5_resend_time_adapt_en = 1; // 1b; enable retry_limit adaption according to baudrate and g_hci_uart_resend_target
                                                                          // should not be enabled when "baud_det_en & (det_wait_en = 0 | det_timeout_en)" ==> Baudrate may be strange
    g_data_uart_settings_3.h5_resend_time_adapt_max = 5; // 3b; the maximum adapted resend_to_val;
    g_data_uart_settings_3.h5_resend_time_adapt_min = 3; // 3b; the minimum adapted resend_to_val
    g_data_uart_settings_3.h5_resend_target = 0; // 2b; valid when resend_time_adapt_en = 1; target resend timing for adaption = 250ms/2^x
    g_data_uart_settings_3.h5_retry_limit_adapt_en = 1; // 1b; valid when resend_time_adapt_en = 1; enable retry_limit adaption according baud and resend_to_val
    g_data_uart_settings_3.h5_trx_active_mask_opt = 0; // 1b;
    #ifdef _YL_H4_FORCE_ERR_EVT_DLYW1C_EN
        g_data_uart_settings_3.err_send_event_delayw1c_opt = _YL_H4_FORCE_ERR_EVT_DLYW1C_VALUE; // 3b
    #else
        g_data_uart_settings_3.err_send_event_delayw1c_opt = 0; // 3b
    #endif

    g_data_uart_settings_2.h5_host_to_val = 0; // 2b; HCI TX abort timeout(to avoid hangup bugs); 2^(2x+10) characters; 3: never timeout
    g_data_uart_settings_2.h5_active_sync = 0;  // 1b
   
    #ifdef _YL_H5_FORCE_ACTIVE_IGNORE_SYNC
        g_data_uart_settings_2.h5_ignore_sync = 1;  // 1b; 1: ignore SYNC at ACTIVE STATE
    #else
        g_data_uart_settings_2.h5_ignore_sync = 0;  // 1b; 1: ignore SYNC at ACTIVE STATE
    #endif
    g_data_uart_settings_2.h5_force_oof_ctrl = 0; // 1b; force out-of-frame software flow control ON (XON/XOFF)
    g_data_uart_settings_2.h5_force_no_oof = 0; // 1b; force out-of-frame software flow control OFF (XON/XOFF)
    g_data_uart_settings_2.h5_retry_limit = 7; // 3b; LinkFailure: 2^(x+1) retries; LinkAlarm: 2^(x) retries
    g_data_uart_settings_2.h5_resend_to_val = 3; // 3b; 0~5; resend(retry) period: 2^(x+9) T_char; 115200: T_char ~ 95.5us; 921600: T_char ~ 11.9us
    g_data_uart_settings_2.h5_sync_to_val = 3; // 3b; 0~4; link establishment period; 2^(2x+6) T_char; 115200: T_char ~ 95.5us; 921600: T_char ~ 11.9us
    g_data_uart_settings_2.h5_ack_to_val = 0; // 2b; 0~3; pure ACK wait time: 2^(2x+2) T_char; 115200: T_char ~ 95.5us; 921600: T_char ~ 11.9us
    g_data_uart_settings_2.h5_wake_to_val = 1; // 2b; WAKEUP period;  2^(2x+2) T_char; 115200: T_char ~ 95.5us; 921600: T_char ~ 11.9us
    g_data_uart_settings_2.h5_int_en = (BIT0|BIT1|BIT2|BIT3|BIT4|BIT5|BIT6|BIT7 |BIT9); // 16b; No PARK_OK_INT
//    #ifdef _YL_H5_FORCE_LINKRESET_RESTART
//        g_data_uart_settings_2.h5_linkreset_restart = 1;    
//    #else
//        g_data_uart_settings_2.h5_linkreset_restart = 0;
//    #endif
    g_data_uart_settings_2.h5_lowpow_sleep_msg_wait_en = 0; // 1b; valid when h5_lowpow_sleep_msg_en = 1; 0: exit hci_uart_h5_send_sleep_msg_wait_done() without waiting done
    #ifdef _YL_H5_FORCE_RETRY_NOT_CLEAR_WHEN_TRX
        g_data_uart_settings_2.h5_retry_state_clr_when_trx = 0; // 1b; 
    #else
        g_data_uart_settings_2.h5_retry_state_clr_when_trx = 1; // 1b; 
    #endif

    g_data_uart_settings.err_event_err_code_opt = 0; // 1b; 
    #ifdef _YL_H5_FORCE_LINKFAIL_CLR_UNACK
        g_data_uart_settings.h5_linkfail_clr_unack = 1; // 1b;
    #else
        g_data_uart_settings.h5_linkfail_clr_unack = 0; // 1b;
    #endif
#ifdef _8821A_BTON_DESIGN_     
    g_data_uart_settings.hci_uart_mcr_rtsn = 0;
#endif

#ifdef _YL_H5_FORCE_BAUD_DETECT

//    hci_uart_man_sram.hci_uart_man_sram_valid_signature = 0;

    g_data_uart_settings.baud_det_en = 0;
    g_data_uart_settings.baud_mon_log_en = 1;
    g_data_uart_settings.baud_est_baud_det_fail_redet_en = 1;
    g_data_uart_settings.baud_est_use_original_buad_det_only = 0;
    g_data_uart_settings.baud_est_use_original_buad_det_once = 1;
    g_data_uart_settings.baud_record_at_rx_vendor_sync = 0; // ????
    g_data_uart_settings.baud_est_baud_det_trial_redet_en = 0;
    g_data_uart_settings.baud_est_delay_before_h5_state_check = 1; 
    g_data_uart_settings.buad_est_bypass_mon_valid_check = 1;
    g_data_uart_settings.baud_record_at_h5_link_est = 0;
    g_data_uart_settings.baud_det_init_with_reset = 1;
    g_data_uart_settings.baud_est_wr_bton_en_at_h5link_uartsync = 0;
    g_data_uart_settings.h5_sign_linkrst_signature = 1;
    g_data_uart_settings.h5_linkrst_sign_fw_trig_wdg = 0;
    g_data_uart_settings.baud_est_stop_at_rx_vendor_sync = 1;
    g_data_uart_settings.baud_mon_resume_every_log = 1;


    
    g_efuse_baud_est_setting_1.baud_mon_en = 1;
    g_efuse_baud_est_setting_1.baud_est_en = 1;
    g_efuse_baud_est_setting_1.baud_recov_at_fw_trig_wdg = 1;
    if (_YL_H5_FORCE_BAUD_DETECT_MODE==1)
    {
        g_efuse_baud_est_setting_1.baud_recov_at_h5_linkreset = 0;
    }
    else
    {
        g_efuse_baud_est_setting_1.baud_recov_at_h5_linkreset = 1;
    }
    g_efuse_baud_est_setting_1.baud_recov_at_reinit = 0;
    g_efuse_baud_est_setting_1.baud_recov_at_other_wdg = 0;
    if (_YL_H5_FORCE_BAUD_DETECT_MODE==1)
    {
        g_efuse_baud_est_setting_1.baud_recov_at_state_stop_postset = 0;
    }
    else
    {
        g_efuse_baud_est_setting_1.baud_recov_at_state_stop_postset = 1;
    }
    g_efuse_baud_est_setting_1.baud_recov_at_state_stop_preset = 0;
    if (_YL_H5_FORCE_BAUD_DETECT_MODE==1)
    {
        g_efuse_baud_est_setting_1.baud_est_restart_at_h5_linkreset = 1;
    }
    else
    {
        g_efuse_baud_est_setting_1.baud_est_restart_at_h5_linkreset = 0;
    }
    g_efuse_baud_est_setting_1.exec_baud_est_init_post_at_reinit = 1;
    g_efuse_baud_est_setting_1.baud_est_update_at_h5_initialized = 1;
    g_efuse_baud_est_setting_1.baud_est_restart_at_baud_sram_recov_0 = 1;
    g_efuse_baud_est_setting_1.baud_est_ovsrx8_grid = 0;
    g_efuse_baud_est_setting_1.min_low_falling_udfl_th_opt = 2;



    g_efuse_baud_est_setting_2.baud_est_falling_cnt_th_1st = 3;
    g_efuse_baud_est_setting_2.baud_est_combine_opt = 1;
    g_efuse_baud_est_setting_2.baud_est_combine_opt_both_udfl = 1;
    g_efuse_baud_est_setting_2.min_falling_ovfl_chg_to_min_low = 1;
    g_efuse_baud_est_setting_2.min_low_udfl_chg_to_min_falling = 1;
    g_efuse_baud_est_setting_2.ignore_invalid_falling_low_ratio = 0;
    g_efuse_baud_est_setting_2.est_bias_value = 0;
    g_efuse_baud_est_setting_2.est_bias_sign = 0;
    g_efuse_baud_est_setting_2.baud_det_finetune_by_exhaust_est = 0;

    g_efuse_baud_est_setting_3.det_allow_table = 0xFFF;
    g_efuse_baud_est_setting_3.baud_est_ovsr_low_bound = 4;
    g_efuse_baud_est_setting_3.w1c_at_fallint_cnt_intr_end = 1;

    g_efuse_baud_est_setting_4.baud_est_opt = _YL_H5_FORCE_BAUD_DETECT_OPT;
    g_efuse_baud_est_setting_4.det_err_chg_to_est_th = 3;
    g_efuse_baud_est_setting_4.baud_est_delay = 2;
    g_efuse_baud_est_setting_4.hci_uart_baud_est_h5init_retry_th = 3;
    g_efuse_baud_est_setting_4.baud_est_by_low_period_at_h5_initialized = 0;
    g_efuse_baud_est_setting_4.baud_est_falling_cnt_th_2nd = 14;
#endif // End of #ifdef _YL_H5_FORCE_BAUD_DETECT


#endif // End of #ifdef _UART_H5_EFUSE_FORCE
#endif // End of #ifdef _UART_H5
   
#endif

#if defined(_8821A_NEW_UART_DESIGN_) && defined(_RTK8723_UART_INIT_)
//    g_uart_clk = _HCI_UART_CLK_FREQ;
#else
    g_uart_clk = otp_str_data.crystal_clk;
#endif

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
    if(sleep_mode_param.dlps_restore_flow == FALSE)
#endif
    {
        dma_init(INIT_FIRST);
    }
#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
    else
    {
        dma_init(INIT_FROM_DLPS);
    }
#endif	
}

#ifdef ENABLE_LOGGER
void pf_logger_transport_reinit(void)
{
    logger_init(S3C2410UartLogWr, uart_logger_transport_is_free);
}

void pf_logger_transport_init(PF_TP_INIT_COMPLETED_CB init_completed_cb,
        PF_TP_TX_COMPLETED_CB tx_completed_cb, 
        PF_UART_DBG_RECD_CB uart_dbg_recd_cb)
{
    S3C2410UartLogInit(tx_completed_cb, uart_dbg_recd_cb);
    pf_logger_transport_reinit();
}
#endif /* ENABLE_LOGGER */

#endif /* NULL_HCI_TRANSPORT */

