#ifndef _PATCH_LOWERSTACK_H_
#define _PATCH_LOWERSTACK_H_

#include "rtl_types.h"
#include "section_config.h"


typedef void (*PF_ORI_ROM_CODE_PATCH_VOID)(void);
typedef UINT8 (*PF_ORI_ROM_CODE_PATCH_FUNC)(void *buf, ...);

//Lory IAR   
#define PF_ROM_CODE_PATCH_VOID		PF_ORI_ROM_CODE_PATCH_VOID
#define PF_ROM_CODE_PATCH_FUNC		PF_ORI_ROM_CODE_PATCH_FUNC 

//Low Stack reinit patch function pointer
extern VoidPatchFun pPatch_DLPS_LowerTask_function_install;

//lc_rf.c
extern PF_ROM_CODE_PATCH_VOID rcp_rf_iqk_func;

//hci_vendor_cmds.c
extern PF_ROM_CODE_PATCH_FUNC rcp_hci_vendor_cmd_func;
extern PF_ROM_CODE_PATCH_FUNC rcp_hci_handle_vendor_write_bb_reg_cmd;
extern PF_ROM_CODE_PATCH_FUNC rcp_hci_vendor_generate_cmd_complete_func;

//le_ll_isr.c
extern PF_ROM_CODE_PATCH_FUNC rcp_ll_hit_conn_adv_isr_func;

//le_hci_4_0.c
extern PF_ROM_CODE_PATCH_VOID rcp_hci_le_adjust_conn_req_func;

//vectors.c
extern PF_ROM_CODE_PATCH_VOID rcp_baseband_interrupt_handler;

//power_control.c
extern PF_ROM_CODE_PATCH_FUNC rcp_pow_ctrl_intr_handle;
extern PF_ROM_CODE_PATCH_VOID rcp_power_init;
extern PF_ROM_CODE_PATCH_FUNC rcp_execute_wakeup_procedure;
extern PF_ROM_CODE_PATCH_FUNC rcp_execute_bton_entering_pdn_sus; // yilinli
#ifdef _CCH_8821_RCP_
extern PF_ROM_CODE_PATCH_VOID rcp_bton_32k_cal_en;
extern PF_ROM_CODE_PATCH_FUNC rcp_bton_32k_cal_chk_lock;
#endif

#ifdef _CCH_2801_RCP_
extern PF_ROM_CODE_PATCH_VOID rcp_bton_32k_cal_ini;
#endif


//main.c
extern PF_ROM_CODE_PATCH_FUNC rcp_dbg_msg_timer_callback;
extern PF_ROM_CODE_PATCH_VOID rcp_hw_pre_init_func;
extern PF_ROM_CODE_PATCH_VOID rcp_main_hci_uart_init_func;  //yilinli
extern PF_ROM_CODE_PATCH_VOID rcp_hw_reg_reinit_func;

//bt_fw_hci_hcbb_info.c
extern PF_ROM_CODE_PATCH_FUNC rcp_hci_handle_reset_command_func; // yilinli
extern PF_ROM_CODE_PATCH_VOID rcp_hci_reset_dma_init_func; // yilinli, RTL8821A

//bt_fw_hci_tasks.c
#ifdef _CCH_8723_RCP_
extern PF_ROM_CODE_PATCH_FUNC rcp_hci_execute_link_control_command_packet_case; // cch
#endif
extern PF_ROM_CODE_PATCH_FUNC rcp_hci_command_handle; //austin

//lmp.c

//lmp_utils.c

#ifdef _CCH_8821_RCP_
extern PF_ROM_CODE_PATCH_VOID rcp_lmp_stop_regular_sw_timers;
extern PF_ROM_CODE_PATCH_VOID rcp_lmp_start_regular_sw_timers;
#endif

//lc.c
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_handle_sm_intr_func;  //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_program_lps_mode_func;
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_check_lps_mode;
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_check_lps_mode_end;
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_check_lps_for_resume;
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_program_lps_check_cal;

//lc_rf.c
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_init_radio_phy_func; //yilinli, RTL8821A
extern PF_ROM_CODE_PATCH_FUNC rcp_rtk_read_modem_radio_reg; //yilinli, RTL8723B/BTONLY
extern PF_ROM_CODE_PATCH_FUNC rcp_rtk_write_modem_radio_reg; //yilinli, RTL8723B/BTONLY
#ifdef _NEW_MODEM_PI_ACCESS_
extern PF_ROM_CODE_PATCH_FUNC rcp_rtk_read_modem_radio_reg_pi; //yilinli, RTL8723B/BTONLY 
extern PF_ROM_CODE_PATCH_FUNC rcp_rtk_write_modem_radio_reg_pi; //yilinli, RTL8723B/BTONLY 
#endif
#ifdef _YL_MODEM_RSSI_MAPPING
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_modem_rssi_mapping; //yilinli,
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_set_modem_lna_constraint_on_off; //yilinli,
extern PF_ROM_CODE_PATCH_FUNC rcp_rtl8821_btrf_lok; //yilinli,
extern PF_ROM_CODE_PATCH_FUNC rcp_rtl8723_btfr_SetTxGainTable; //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_rtl8723_btrf_TxPowerTrack; //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_rtl8723_btrf_UpdateThermalValueTimer; //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_psd_modem_init; //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_GetStartGainIndexTable PATCH_POINTER_SECTION;
#endif

#ifdef _CCH_8723_RCP_
extern PF_ROM_CODE_PATCH_VOID rcp_lc_baseband_reset;// cch
#endif

#ifdef _CCH_8703B_RCP_
extern PF_ROM_CODE_PATCH_VOID rcp_enter_deep_lps_mode;
extern PF_ROM_CODE_PATCH_VOID rcp_dlps_save_io_mem_to_partial_on;
extern PF_ROM_CODE_PATCH_VOID rcp_dlps_restore_io_mem_from_partial_on;
extern PF_ROM_CODE_PATCH_VOID rcp_dlps_restore_io_mem_from_partial_on_after_clk_rdy;
#endif

//lc_isr.c
#ifdef _CCH_8821_RCP_
extern PF_ROM_CODE_PATCH_FUNC rcp_bb_scan_end_intr;
#endif

//bt_fw_hci_events.c
#ifdef _CCH_8723_RCP_
extern PF_ROM_CODE_PATCH_FUNC rcp_hci_generate_command_complete_event;// cch
#endif


//lmp_pdu.c
// ll_isr.c
#ifdef _DAPE_8723_RCP_
extern PF_ROM_CODE_PATCH_FUNC rcp_ll_update_slot_in_ce_end_func;
extern PF_ROM_CODE_PATCH_FUNC rcp_ll_ce_begin_func;    
#endif
#ifdef _CCH_8821_RCP_
extern PF_ROM_CODE_PATCH_VOID rcp_ll_cleanup_rx_status_ae_end;
#endif
extern PF_ROM_CODE_PATCH_FUNC rcp_ll_cleanup_rx_status_ce_end; 

#ifdef _CCH_8821B_TC_RCP_
extern PF_ROM_CODE_PATCH_VOID rcp_ll_handle_event_end_interrupt;
#endif

//bzdma.c
extern PF_ROM_CODE_PATCH_FUNC rcp_bzdma_update_fw_rptr_of_ble_data_ring_fifo;

extern PF_ROM_CODE_PATCH_FUNC rcp_ll_isr_func;  
extern PF_ROM_CODE_PATCH_FUNC rcp_ll_isr_func_end;  
extern PF_ROM_CODE_PATCH_FUNC rcp_ll_handle_conn_interrupt;  
extern PF_ROM_CODE_PATCH_FUNC rcp_ll_handle_scan_start_intr_func;
extern PF_ROM_CODE_PATCH_FUNC rcp_ll_early_func;

//le_ll.c
extern PF_ROM_CODE_PATCH_FUNC rcp_ll_handle_adv_channel_pdu;
extern PF_ROM_CODE_PATCH_FUNC rcp_ll_update_instant_func;
extern PF_ROM_CODE_PATCH_FUNC rcp_llc_sent_llc_pdu_ack_recd_func;

//le_ll_driver.c
extern PF_ROM_CODE_PATCH_FUNC rcp_ll_data_cmd_decision;
extern PF_ROM_CODE_PATCH_FUNC rcp_ll_driver_wake_from_slave_latency;

// le_ll_general.c
#ifdef _DAPE_8723_RCP_
extern PF_ROM_CODE_PATCH_FUNC rcp_ll_update_slot_decision_func;
#endif 
//power_control.c
extern PF_ROM_CODE_PATCH_FUNC rcp_execute_lps_mode_procedure_6128_func;  //yilinli
extern PF_ROM_CODE_PATCH_VOID rcp_lps_mode_setting;
extern PF_ROM_CODE_PATCH_FUNC rcp_enter_lps_mode_6128_func;  //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_disable_lps_mode_setting_func; // yilinli
//h5.c
extern PF_ROM_CODE_PATCH_FUNC rcp_hci_uart_h5_isr_func;  //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_hci_uart_h5_timing_adapt_func;  //yilinli
extern PF_ROM_CODE_PATCH_VOID rcp_hci_uart_h5_backup_func;  //yilinli
extern PF_ROM_CODE_PATCH_VOID rcp_hci_uart_h5_restore_func;  //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_hci_uart_h5_poll_wake_callback_func;  //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_hci_uart_h5_go_sleep_func;  //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_hci_uart_h5_go_lps_func;  //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_hci_uart_h4h5_config_func;  //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_hci_uart_vendor_set_baud_func;  //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_hci_uart_send_long_break_callback_func;  //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_hci_uart_h5_wakeup_utility_func;  //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_hci_uart_change_baudrate_event_func;  //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_hci_uart_h4_err_event_func;  //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_hci_uart_falling_cnt_isr_func;  //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_hci_uart_baud_est_init_preprocessing_func;  //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_hci_uart_baud_est_init_postprocessing_func;  //yilinli
#ifdef _CCH_8723_RCP_
extern PF_ROM_CODE_PATCH_VOID rcp_hci_uart_h5_isr_lps_reset; // cch
#endif
extern PF_ROM_CODE_PATCH_FUNC rcp_hci_uart_baud_recovery_func;

//usb_dma.c
extern PF_ROM_CODE_PATCH_FUNC rcp_hci_dma_intr_h4_error_func;  //yilinli
#ifdef _CCH_8723_RCP_
extern PF_ROM_CODE_PATCH_FUNC rcp_USB_DMA_IntrHandler_lps_reset; // cch
#endif
extern PF_ROM_CODE_PATCH_FUNC rcp_usb_dma_intrhandler_func;   //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_S3C2410Usb_Dma_wake_host; //yilinli


//bb_driver.c
extern PF_ROM_CODE_PATCH_FUNC rcp_bb_write_baseband_register_func;  //yilinli

//bt_fw_acl_q.c

//lmp_ch_assessment.c

#endif

