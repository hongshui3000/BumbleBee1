/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  Contains the functions that handle the Host controller and
 *  Baseband commands.
 */

/********************************* Logger *************************/
enum { __FILE_NUM__= 10 };
/********************************* Logger *************************/

/* Includes */
#include "bt_fw_hci_internal.h"
#include "lmp_internal.h"
#include "lmp.h"

#include "bt_fw_acl_q.h"
#include "mem.h"

#include "dma_usb.h"
#include "bzdma.h"

#ifdef LE_MODE_EN
#include "le_ll.h"
#include "le_hw_reg.h"
#endif

#include "power_control.h"
#include "mailbox.h"
#include "pta.h"
#include "gpio.h"
#include "hci_vendor_defines.h"
#include "lmp_ch_assessment.h"
#include "lmp_sco.h"
// 20120109 morgan
#ifdef PTA_EXTENSION
#include "pta_meter.h"
#endif
#include "timer.h"
#ifdef _3DD_FUNCTION_SUPPORT_
#include "bt_3dd.h"
#endif
#include "plc.h"
#ifdef MWS_ENABLE  
#include "mws.h"
#endif
#ifdef CONFIG_TV_FEATURE
#include "tv.h"
#endif
#ifdef CONFIG_FM
#include "fm.h"
#endif

#ifdef _SUPPORT_POLLING_BASED_LPM_L1_
#include "dma_usb.h"
#endif
#ifdef _SUPPORT_INFO_FROM_SYSTEM_ON_    
#include "system_on.h"
#endif

#ifdef _SUPPORT_INFO_FROM_SYSTEM_ON_    
extern UINT8 g_chip_id;
#ifdef _SUPPORT_BT_CTRL_FM_  
extern UINT8 g_u8support_fm;
#endif
#endif  
#ifdef _DAPE_DETECT_WHCK_P2P
extern UINT8 g_whck_running;
extern UINT8 g_whck_item;

#endif

#ifdef _DAPE_TEST_GEN_FAKE_ESCO_DATA
extern UINT16 g_handle;
extern UINT8 g_change;
extern HCI_SYNC_DATA_PKT *pgen_synchronous_pkt; 
extern UINT8 g_change_flag;
#endif
#ifdef _DAPE_TEST_CHK_ESCO_DATA_RX
extern UINT8 g_esco_print_log;
#endif

extern LMP_FHS_PKT_RECD  *prev_fhs_ptr_for_eir;

#if defined(TEST_MODE)
extern UINT8 test_mode_sched_acl_pkt;
extern UINT8 test_mode_tx_mode_force_ack;
extern UINT8 test_mode_tx_mode_buf_in_used;
#endif

#ifdef _ENABLE_BTON_POWER_SAVING_
extern UINT8 isr_switch;
extern BTON_POW_CTRL_REG_S_TYPE pow_ctrl_int_rec;
#endif

#ifdef LE_MODE_EN
#ifdef _DAPE_TEST_SLOT_OFFSET_FOR_LE
extern UINT16 g_le_conn_interval_changed;
extern UCHAR  g_le_conn_interval_set;
#endif
#endif

#ifdef _SUPPORT_POLLING_BASED_LPM_L1_
extern UINT32 g_check_lpm_timer_cnt;
extern LPM_POLL_CONTROL g_lpm_poll_control_s;
#endif


extern UINT32 g_lps_timer_counter;
extern UINT32 g_timer2_init_value;
extern UINT8 bqb_receive_sco_esco_pkt_in_ed_test;

#ifdef _FONGPIN_TEST_PI_ACCESS_3RD_MODEM_
extern UINT32 g_fongpin_test_timer_counter;
#endif

#ifdef _ROM_CODE_PATCHED_
#ifdef _YL_RTL8723A_B_CUT
PF_ROM_CODE_PATCH_FUNC rcp_hci_handle_reset_command_func = NULL;
#endif
PF_ROM_CODE_PATCH_VOID rcp_hci_reset_dma_init_func = NULL;
#endif

#ifdef _LE_SPEED_UP_HCI_ACL_TX_PKT_HANDLE_
extern LL_HCI_ACL_DATA_PKT *pll_hci_pre_pend_list_head;
extern LL_HCI_ACL_DATA_PKT *pll_hci_pre_pend_list_tail;
#endif

extern void dbg_rtk_set_debug_port(void);

void bt_fw_stack_reset(void);

/**
 * Resets all the modules. It initializes the global variables,
 * Self device structure and connection entity database.
 * It also deletes all the buffer pools created by HCI, LMP and LC module.
 *
 * \param None.
 *
 * \return HCI_COMMAND_SUCCEEDED or HARDWARE_FAILURE_ERROR.
 *
 */
UCHAR hci_handle_reset_command(void)
{
    UINT8 index;
    UINT8 init_type = INIT_RUN_TIME;
    EFUSE_POW_PARA_S_TYPE efuse_pow_para;
    efuse_pow_para.d16 = otp_str_data.power_seq_param;
    GENERAL_CONTROL_S_TYPE bt_general_ctrl;
    //the setting value loaded from efuse
    bt_general_ctrl.d16 = otp_str_data.general_control;

    DEF_CRITICAL_SECTION_STORAGE;

#ifdef POWER_SAVE_FEATURE
    LC_EXIT_SM_MODE();
#endif

    /*------------------------------------*/
    /*      Enter Critical Section        */        
    /*------------------------------------*/

#ifdef _ROM_CODE_PATCHED_
#ifdef _YL_RTL8723A_B_CUT
    /* yilinli, 
     * for initialization of power related variables and state
     * e.g.
     *  1. g_host_state ?? 
     *  2. g_wake_first_time ??
     *  3. g_host_wake_bt ?? (gpio_host_wake_bt?)
     *  4. gpio_bt_wake_host ??
     *  5. host_sleep_pkt_man.queue_index
     *  6. 
     */
    if (rcp_hci_handle_reset_command_func != NULL)
    {
        if (rcp_hci_handle_reset_command_func((void *)&efuse_pow_para, &bt_general_ctrl))
        {
            return HCI_COMMAND_SUCCEEDED;
        }
    }
#endif    
#endif
    MINT_OS_ENTER_CRITICAL();


    /* Resetting LC module */
    lc_module_reset();

    lc_baseband_reset();

    /* Reset the Firmware stack */
    bt_fw_stack_reset();

    /* Resetting HCI module */
    hci_module_reset();

    /* Reset BZDMA module */
    bzdma_init();

#ifdef _PLC_FUNC_ENABLE_
    plc_init();
#endif

#ifdef LE_MODE_EN
    if (IS_BT40)
    {
        /* Resetting LL module */
        for (index = 0; index < LL_MAX_CONNECTION_UNITS; index++)
        {
            ll_fw_reset_remote_conn_entry_param(index);
        }    
        ll_init();
    }
#endif

#ifdef _3DD_FUNCTION_SUPPORT_
    if (IS_SUPPORT_3DG_APP)
    {
        bt_3dd_driver_init();
    }
#endif

    /* Resetting LMP module */
    for (index = 0; index < LMP_MAX_CE_DATABASE_ENTRIES; index++)
    {
        lmp_reset_timers( (UINT16)index);
    }
    lmp_module_reset();
    
#ifdef _ROM_CODE_PATCHED_
    /* Patch code reserved by yilinli, for
     * e.g. (For UART H5)
     *    step 1. H5 go_sleep.
     *    step 2. Backup H5 status.
     *    step 3. dma_init();
     *    step 4. Restore H5 status.
     *    step 5. Send a number(7 or win_size) of NOP COMMAND STATUS EVENTs to host.
     *            before sending HCI_RESET COMMAND COMPLETE EVENT.
     *            This step is to avoid Host-Controller ACK/SEQ status mismatch
     *    step 6. Send HCI_RESET COMMAND COMPLETE EVENT to notify host.
     */
    if (rcp_hci_reset_dma_init_func != NULL)
    {
        rcp_hci_reset_dma_init_func();
    }
    else
#endif
    {
        if (g_fun_interface_info.b.bt_interface != UART_INTERFACE)
        {
            dma_init(init_type);
        }
        else
        {
#ifdef _UART_H5    
#ifdef _UART_H5_FPGA_EFUSE_FORCE
#ifdef _YL_H5_MODE_ENABLE
            efuse_pow_para.b.uart_hci_reset_en = 0;
#else
            efuse_pow_para.b.uart_hci_reset_en = 1;
#endif
#endif
#endif
            if (efuse_pow_para.b.uart_hci_reset_en || g_uart_h5_fw_allow)
            {
                dma_init(init_type);
            }
        }
    }


#ifdef MWS_ENABLE  
    if (IS_BT_MWS)
    {
        mws_init();
    }
#endif

#ifdef CONFIG_TV_FEATURE
    tv_app_init();
#endif

#ifdef CONFIG_FM
#ifdef _SUPPORT_INFO_FROM_SYSTEM_ON_
    if (g_u8support_fm)
#endif        
    {
        fm_init();
    }
#endif

#ifdef _LE_SPEED_UP_HCI_ACL_TX_PKT_HANDLE_
    pll_hci_pre_pend_list_head = NULL;                
    pll_hci_pre_pend_list_tail = NULL;
#endif

#if defined(TEST_MODE)
    test_mode_sched_acl_pkt = FALSE;
    test_mode_tx_mode_force_ack = FALSE;
    test_mode_tx_mode_buf_in_used = FALSE;

#ifdef _DUT_DELAYED_LOOPBACK_MODE_
    dut_mode_manager.buf[0].ppkt = NULL;
    dut_mode_manager.buf[1].ppkt = NULL;
    dut_mode_manager.work_space = 0;
#endif
#endif

    prev_fhs_ptr_for_eir = NULL;

#ifdef _ENABLE_BTON_POWER_SAVING_
    isr_switch = 0;
    pow_ctrl_int_rec.d32 = 0;
#endif
    wakeup_isr_en = 0;

#ifdef _CCH_LPS_
    g_lps_timer_counter = g_timer2_init_value;		
#else
    g_lps_timer_counter = 0;
#endif

#ifdef _FONGPIN_TEST_PI_ACCESS_3RD_MODEM_
    g_fongpin_test_timer_counter = 0;
#endif

#ifdef LE_MODE_EN
#ifdef _DAPE_TEST_SLOT_OFFSET_FOR_LE
    g_le_conn_interval_changed = 0;
    g_le_conn_interval_set = 0;
#endif
#endif

#ifdef _ENABLE_MAILBOX_
    mailbox_out_data_busy = 0;
    mailbox_count = 0;
    g_wifi_ch = 0;
#endif
    //indicate if the fw is hic loopback mode
    enable_hci_loopback_mode = 0;
    //indicate if the host entered sleep mode
    g_host_state = 0;

    alternatset_change_tx_clear = 0;
    alternatset_change_rx_clear = 0;

    //For uart h5 link reset
    g_uart_h5_fw_allow = 0;

    //For wake up function index
    g_wake_first_time = 1;

#ifdef _YL_RTL8723A_B_CUT
#ifdef _GPIO_POWER_SEQUENCE_ENABLE
#ifdef _8821A_BTON_DESIGN_
    EFUSE_POW_SETTING_2_S efuse_pow_setting_2;
    *(UINT8*)&efuse_pow_setting_2 = otp_str_data.efuse_pow_setting_2_d8;
    if (efuse_pow_setting_2.hci_reset_init_power_var)
#else
    if (g_efuse_lps_setting_2.hci_reset_init_power_var)
#endif
    {
        g_host_wake_bt = 1;
#ifdef _8821A_BTON_DESIGN_
//        EFUSE_POW_SETTING_2_S efuse_pow_setting_2;
//        *(UINT8*)&efuse_pow_setting_2 = otp_str_data.efuse_pow_setting_2_d8;
        if (efuse_pow_setting_2.gpio_host_wake_bt_en)
#else
        if (g_efuse_lps_setting_2.gpio_host_wake_bt_en)
#endif        
        {
            g_host_wake_bt = get_gpio_host_wake_bt();
        }
        /* No effect When bt_general_ctrl.b.gpio_wake_up_type == 0 */
        /* bt_general_ctrl.b.gpio_wake_up_type serve as Enable and Pin Select */
        /* the polarity of gpio_bt_wake_host is controlled by bt_general_ctrl.b.gpio_wake_up_polarity*/
#ifdef _8821A_BTON_DESIGN_    
        if (efuse_pow_setting_2.bt_wake_host_init_opt)
        {
            set_gpio_bt_wake_host(g_host_state);
        }
        else
        {   
            set_gpio_bt_wake_host(!g_host_state);
        }
#else
        set_gpio_bt_wake_host(g_host_state);
#endif
    }
#endif
#endif
    
#ifdef _ENABLE_RTK_PTA_
    pta_init();

#ifndef _DISABLE_OBSOLETE_MAILBOX_CMD_
    /* disable the timer */
    timer_on_off(TIMER2_ID, 0, 0);    
#endif
#endif

#ifdef _SUPPORT_MAILBOX_MP_COMMAND_
    memset(&patch_mp_misc_param, 0, sizeof(MAILBOX_WIFI_MP_MISC_PARAM_S));
#endif

    os_reset_reserved_buffer();

    dbg_rtk_set_debug_port();

#ifdef PTA_EXTENSION
    fnPtaHciMeterInit();
#endif

#ifdef MINICARD_BT_LED_CONTROL
    BT_LED_WPAN_OFF();
#endif

    dbg_vendor_log_interface = VENDOR_LOG_PACKET_TYPE_INVALID;
    dbg_vendor_log_conn_handle = 0;
    dbg_vendor_log_buf = NULL;
    dbg_vendor_set_log_complete_event_buf = (UINT8*)0xDEADBEEF;      
    bqb_receive_sco_esco_pkt_in_ed_test = FALSE;

#ifdef _RTK_PTA_TEST_
    pta_test_in_hci_reset_func(); 
#endif

#ifdef _DAPE_TEST_SHUTDN_RF_WHEN_RESET
    rtk_write_modem_radio_reg(0x00, TYPE_RF, 0x00);
#endif

#ifdef _DAPE_TEST_AUTO_CONN_TEST
    g_host_state = 1;
#endif

#ifndef _IS_ASIC_
    ///// dape test
    // rtk_modem_init(); /* need re-init ?? (austin) */ /* re-marked by yilinli */
#endif

#ifdef _DAPE_TEST_GEN_FAKE_ESCO_DATA
    g_handle = 0xff;
    g_change = 0;
    pgen_synchronous_pkt = NULL; 
    g_change_flag = 0;
#endif

#ifdef _DAPE_TEST_CHK_ESCO_DATA_RX
    g_esco_print_log = 0;
#endif
#ifdef _DAPE_DETECT_WHCK_P2P
    g_whck_running = 0;
    g_whck_item = 0xff;
#endif

    MINT_OS_EXIT_CRITICAL();

    /*------------------------------------*/
    /*      Exit Critical Section        */        
    /*------------------------------------*/

#if 0    
#ifdef _YL_NEW_MODEM_SRAM_DEBUG
    g_modem_sram_debug_captured_flag = 0;
    rtl8821_btrf_modem_sram_debug_set_en(1);
#endif
#endif
    return HCI_COMMAND_SUCCEEDED;
}

/**
 * Inteprets the Set Event Mask command packet coming from
 * the host and sets the global variable event_mask which is used by the
 * Link Manager to determine which events are to be sent to the host.
 *
 * \param hci_cmd_ptr Pointer to the command packet.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_set_event_mask_command(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR offset;

    BT_FW_HCI_INF(SETTING_EVENT_MASK,0,0);

    for (offset=0; offset<LMP_EVENT_MASK_SIZE; offset++)
    {
        lmp_self_device_data.event_mask[offset] =
            hci_cmd_ptr->cmd_parameter[offset];
    }

    /* Command complete cannot be masked and Command status cannot be masked */
    lmp_self_device_data.event_mask[1] |= 0x60;

    /* Num_complete_pkts cannot be masked */
    lmp_self_device_data.event_mask[2] |= 0x04;

    return HCI_COMMAND_SUCCEEDED ;
}


/**
 * Inteprets the Set Event Filter command packet coming
 * from the host and stores the Event Filter in its appropriate instance.
 * The Event Filters are used by the Link Manager to determine the
 * behaviour of the Link Manager to the incoming connection or inquiry
 * requests
 *
 * \param hci_cmd_ptr Pointer to the command packet.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_set_event_filter_command(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR offset = 0;
    UCHAR filter_type;
    UCHAR filter_condition_type;
    UINT32 class_of_device = 0x00, class_of_device_mask = 0x00 ;
    UCHAR auto_accept_flag = 0x00;
    UCHAR bd_addr[LMP_BD_ADDR_SIZE];

    BT_FW_HCI_INF(SETTING_EVENT_FILTERS,0,0);

    if(lmp_event_filters_written >= LMP_MAX_EVENT_FILTERS)
    {
        BT_FW_HCI_ERR(MAXIMUM_SET_EVENT_FILTERS_REACHED,0,0);
        return MEMORY_FULL_ERROR;
    }

    filter_type = hci_cmd_ptr->cmd_parameter[offset];
    offset += 1;

    if(filter_type > 2)
    {
        BT_FW_HCI_ERR(INVALID_FILTER_TYPE_RECD_FILTER_TYPE,1,filter_type);
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR ;
    }

    if(filter_type == 0x00)
    {
        /* If additional parameters are supplied, generate error */
        if (hci_cmd_ptr->param_total_length > 0x01)
        {
            BT_FW_HCI_ERR(INVALID_PARAM_TOTAL_LENGTH_RECD_PARAM_TOTAL_LENGTH,1,hci_cmd_ptr->param_total_length);
            return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
        }

        BT_FW_HCI_INF(INITIALIZING_ALL_THE_EVENT_FILTERS,0,0);
        /* Clear all the filters */
        lmp_initialize_event_filters();
        return HCI_COMMAND_SUCCEEDED ;
    }

    filter_condition_type = hci_cmd_ptr->cmd_parameter[offset];
    offset += 1;

    if(filter_condition_type > 2)
    {
        BT_FW_HCI_ERR(INVALID_FILTER_CONDN_TYPE_RECD_FILTER_CONDN_TYPE,1,filter_condition_type);
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR ;
    }

    /* Validate filter type and filter condition type parametes. */

    switch (filter_type)
    {
        case INQUIRY_RESULT_FILTER :
            /* Inquiry Result type */
            BT_FW_HCI_INF(SETTING_UP_INQUIRY_RESULT_FILTER,0,0);
            switch (filter_condition_type)
            {
                case HCI_NEW_DEV_RESP :
                    /* Reset Inquiry result event filters. */
                    BT_FW_HCI_INF(INITIALIZING_ALL_INQUIRY_RESULT_EVENT_FILTERS,0,0);
                    lmp_initialize_inquiry_result_event_filters();
                    return HCI_COMMAND_SUCCEEDED ;

                case HCI_NEW_DEV_COD :
                    class_of_device = (hci_cmd_ptr->cmd_parameter[offset + 2] << 16) |
                                       (hci_cmd_ptr->cmd_parameter[offset + 1] << 8) |
                                        hci_cmd_ptr->cmd_parameter[offset];
                    offset += 3;
                    BT_FW_HCI_INF(CLASS_OF_DEVICE_RECD,1,class_of_device);
                    class_of_device_mask = (hci_cmd_ptr->cmd_parameter[offset + 2] << 16) |
                                           (hci_cmd_ptr->cmd_parameter[offset + 1] << 8) |
                                            hci_cmd_ptr->cmd_parameter[offset];
                    offset += 3;
                    BT_FW_HCI_INF(CLASS_OF_DEVICE_RECD,1,class_of_device_mask);
                    break;

                case HCI_NEW_DEV_BD_ADDR :
                    memcpy(bd_addr, &hci_cmd_ptr->cmd_parameter[offset],
                           LMP_BD_ADDR_SIZE);
                    offset += LMP_BD_ADDR_SIZE;
                    break;

                default :
                    return INVALID_HCI_COMMAND_PARAMETERS_ERROR;

            } /* Inquiry result filter */
            break;

        case CONNECTION_SETUP_FILTER :
            /* Connection Setup type */
            BT_FW_HCI_INF(SETTING_UP_CONNECTION_SETUP_FILTER,0,0);
            switch (filter_condition_type)
            {
                case HCI_ALLOW_ALL_CONNECTIONS :
                    auto_accept_flag =  hci_cmd_ptr->cmd_parameter[offset];
#ifdef _DAPE_DISABLE_EVENT_FILTER_ROLE_SW
                    auto_accept_flag = 2;
#endif
                    offset += 1;
                    BT_FW_HCI_INF(ALLOW_ALL_CONNECTIONS_AUTO_ACCEPT_FLAG,1,auto_accept_flag);
                    /*    Validate Auto accept flag */
                    if((auto_accept_flag < 0x01) || (auto_accept_flag > 0x03))
                    {
                        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
                    }
                    break;

                case HCI_ALLOW_COD_CONNECTIONS :
                    BT_FW_HCI_INF(ALLOW_SPECIFIC_COD_CONNECTIONS,0,0);
                    class_of_device = (hci_cmd_ptr->cmd_parameter[offset + 2] << 16) |
                                       (hci_cmd_ptr->cmd_parameter[offset + 1] << 8) |
                                        hci_cmd_ptr->cmd_parameter[offset];
                    offset += 3;

                    class_of_device_mask = (hci_cmd_ptr->cmd_parameter[offset + 2] << 16) |
                                           (hci_cmd_ptr->cmd_parameter[offset + 1] << 8) |
                                            hci_cmd_ptr->cmd_parameter[offset];
                    offset += 3;
                    auto_accept_flag = hci_cmd_ptr->cmd_parameter[offset];
                    offset += 1;

                    /* Validate Auto accept flag */
                    if((auto_accept_flag < 0x01) || (auto_accept_flag > 0x03))
                    {
                        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
                    }
                    BT_FW_HCI_INF(CLASS_OF_DEVICE_RECD_COD_MASK,2,class_of_device,class_of_device_mask);
                    BT_FW_HCI_INF(AUTO_ACCEPT_FLAG,1,auto_accept_flag);
                    break;

                case HCI_ALL_BD_ADDR_CONNECTIONS :
                    BT_FW_HCI_INF(ALLOW_SPECIFIC_BD_ADDR_CONNECTIONS,0,0);
                    memcpy(bd_addr, &hci_cmd_ptr->cmd_parameter[offset],
                           LMP_BD_ADDR_SIZE);
                    offset += LMP_BD_ADDR_SIZE;

                    auto_accept_flag = hci_cmd_ptr->cmd_parameter[offset];
                    offset += 1;

                    /* Validate Auto accept flag */
                    if((auto_accept_flag < 0x01) || (auto_accept_flag > 0x03))
                    {
                        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
                    }
                    BT_FW_HCI_INF(AUTO_ACCEPT_FLAG,1,auto_accept_flag);
                    break;

                default :
                    return INVALID_HCI_COMMAND_PARAMETERS_ERROR;

            }/*Connection setup filter */
            break;

        default :
            return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    /* All the parameters are validated now store it in conn entity. */
    LMP_EVENT_FILTER *ef;
    ef = &lmp_self_device_data.event_filter[lmp_event_filters_written];

    ef->filter_type = filter_type;
    ef->filter_condition_type = filter_condition_type;
    ef->class_of_device = class_of_device;
    ef->class_of_device_mask = class_of_device_mask;
    memcpy(ef->bd_addr, bd_addr, LMP_BD_ADDR_SIZE);

    /*  Auto accept the connection */
    ef->auto_accept_flag = auto_accept_flag ;

    /* Increment the Event Filter Count by one (add protection by austin) */
    if (lmp_event_filters_written < (LMP_MAX_EVENT_FILTERS-1))
    {
        lmp_event_filters_written++;
    }

    BT_FW_HCI_INF(SUCCESSFULLY_STORED_EVENT_FILTER,1, lmp_event_filters_written);

    return HCI_COMMAND_SUCCEEDED ;
}

/**
 * Stores the Length of host ACL/SCO data packets
 * and number of ACL/SCO buffers.
 *
 * \param hci_cmd_ptr Pointer to the command packet.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_host_buffer_size_command(HCI_CMD_PKT *hci_cmd_pkt_ptr)
{
    UCHAR pkt_status = HCI_COMMAND_SUCCEEDED ;

    BT_FW_HCI_INF(HOST_BUFFER_SIZE_INFO_COMMAND,0,0);

    if (lmp_self_device_data.flow_control_hc_to_host == 0)
    {
        /* HCI ACL and SCO are flow control off - ignore this command */
        return pkt_status;
    }

    if (lmp_self_device_data.flow_control_hc_to_host & 0x01)
    {
        /* change the HCI ACL Data Packets Settings if Flow control on */
        BT_FW_EXTRACT_16_BITS(lmp_host_info_data.host_acl_data_pkt_len,
                              &(hci_cmd_pkt_ptr->cmd_parameter[0]));

        lmp_host_info_data.host_total_num_acl_data_pkts  =
                            hci_cmd_pkt_ptr->cmd_parameter[3] |
                            (hci_cmd_pkt_ptr->cmd_parameter[4] << 8);
    }

    if (lmp_self_device_data.flow_control_hc_to_host & 0x02)
    {
        /* change the HCI SCO Data Packets Settings if Flow control on */
        lmp_host_info_data.host_sco_data_pkt_len =
            hci_cmd_pkt_ptr->cmd_parameter[2];

        lmp_host_info_data.host_total_num_sco_data_pkts =
            hci_cmd_pkt_ptr->cmd_parameter[5] |
            (hci_cmd_pkt_ptr->cmd_parameter[6] << 8);
    }

    return pkt_status;
}

/**
 * Discards all the ACL data packets pending for
 * transmission in the host controller for the specified connection
 * handle
 *
 * \param hci_cmd_ptr Pointer to the command packet.
 * \param ce_index    [OUT]Connection Entity Index.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_flush_command(HCI_CMD_PKT *hci_cmd_ptr, UINT16 *ce_index)
{
    UINT16 conn_handle;

    LMP_CONNECTION_ENTITY *ce_ptr;

    BT_FW_EXTRACT_16_BITS(conn_handle, &(hci_cmd_ptr->cmd_parameter[0]));

    /* Obtain the index corresponding to the connection handle */
    if(LMP_GET_CE_INDEX_FROM_CONN_HANDLE(conn_handle, ce_index) == API_FAILURE)
    {
        return NO_CONNECTION_ERROR;
    }

    ce_ptr = &lmp_connection_entity[*ce_index];

    if (ce_ptr->flush_running == TRUE)
    {
        return COMMAND_DISALLOWED_ERROR;
    }

    if (aclq_mark_all_for_flush(ce_ptr->am_addr, ce_ptr->phy_piconet_id) == TRUE)
    {
        ce_ptr->flush_running = TRUE;
    }
    else
    {
        hci_generate_command_complete_event(
            HCI_FLUSH_OPCODE,
            HCI_COMMAND_SUCCEEDED,
            *ce_index,
            NULL);
    }

    return HCI_COMMAND_SUCCEEDED;
}

/**
 * validates the parameters of Write_Num_Broadcast_Retransmission
 * command and updates the self device data structure with
 * the value received.This command is to write the number of broadcast
 * retransmissions parameter value for the device.
 *
 * \param hci_cmd_ptr Pointer to the command packet.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_write_num_brcast_retran(HCI_CMD_PKT *hci_cmd_ptr)
{
#ifdef BROADCAST_DATA
    lmp_self_device_data.num_broadcast_retran = hci_cmd_ptr->cmd_parameter[0];
#ifdef _ENABLE_BC_HIGH_FUN_
    BB_write_baseband_register(0x1A4, 0x800);    
#endif
#endif /* BROADCAST_DATA */

    return HCI_COMMAND_SUCCEEDED ;
}

#ifndef _DISABLE_HCI_HOST_FLOW_CONTROL_
#ifdef ENABLE_SCO
/**
 * Validates the parameters of Write_SCO_Flow_Control_Enable
 * command and updates the self device sco_flow_control_enable paramter
 * with value received from the Host.
 *
 * \param hci_cmd_ptr Pointer to the command packet.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_write_sco_ctrl_enable(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR offset = 0;

    if (hci_cmd_ptr->cmd_parameter[offset] > 0x01)
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    lmp_self_device_data.sco_flow_control_enable = 
                                    hci_cmd_ptr->cmd_parameter[offset];

    return HCI_COMMAND_SUCCEEDED ;
}
#endif /* ENABLE_SCO */

/**
 * Validates the parameters of Set_Host_Controller_To_Host_Flow_control
 * command and updates the self device flow_control_hc_
 * to_host paramter with value received from the Host.
 *
 * \param hci_cmd_ptr Pointer to the command packet.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_set_hc_to_host_flow_ctrl(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR offset = 0;

    if (hci_cmd_ptr->cmd_parameter[offset] > 0x03)
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    //RT_BT_LOG(GREEN, BT_FW_HCI_HCBB_INFO_461, 1, hci_cmd_ptr->cmd_parameter[offset]);

    lmp_self_device_data.flow_control_hc_to_host = hci_cmd_ptr->cmd_parameter[offset];

    return HCI_COMMAND_SUCCEEDED ;
}
#endif /* end of _DISABLE_HCI_HOST_FLOW_CONTROL_ */

/**
 * Validates the parameters of Write_Current_IAC_LAP command
 * and updates the self device iac_lap paramter with the lap values
 * received from the Host.
 *
 * \param hci_cmd_ptr Pointer to the command packet.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_write_current_iac_lap(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR offset = 0;
    UCHAR num_current_IAC = 0xFF;
    UCHAR tmp_index;
    UINT32 lap ;
    UCHAR iac_lap[LMP_MAX_IAC_LAPS][3];

    /* Check for allowed number of current IAC */
    if (hci_cmd_ptr->cmd_parameter[offset] > 0x40 ||
            hci_cmd_ptr->cmd_parameter[offset] < 0x01)
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }
    else
    {
        num_current_IAC = hci_cmd_ptr->cmd_parameter[offset];
        offset++;
    }

    if (hci_cmd_ptr->param_total_length != ((num_current_IAC * 3) + 1))
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    tmp_index=0;

    memset(iac_lap, 0, (LMP_MAX_IAC_LAPS*3));

    while ((tmp_index < num_current_IAC) && (tmp_index < LMP_MAX_IAC_LAPS))
    {
        iac_lap[tmp_index][0] = hci_cmd_ptr->cmd_parameter[offset];
        offset++;

        iac_lap[tmp_index][1] = hci_cmd_ptr->cmd_parameter[offset];
        offset++;

        iac_lap[tmp_index][2] = hci_cmd_ptr->cmd_parameter[offset];
        offset++;

        {
            lap = iac_lap[tmp_index][2] ;
            lap <<= 8;
            lap |= iac_lap[tmp_index][1] ;
            lap <<= 8;
            lap |= iac_lap[tmp_index][0] ;
            BT_FW_HCI_INF(LAP_RECEIVED,1,lap);

            if(lap < LC_INQUIRY_LAP_MIN || lap > LC_INQUIRY_LAP_MAX)
            {
                BT_FW_HCI_ERR(INVALID_LAP_VALUE_RECEIVED,1,lap);
                return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
            }
        }
        tmp_index++;
    }

    tmp_index=0;

    while ((tmp_index < num_current_IAC) && (tmp_index < LMP_MAX_IAC_LAPS))
    {
        lmp_self_device_data.iac_lap[tmp_index][0] = iac_lap[tmp_index][0];
        lmp_self_device_data.iac_lap[tmp_index][1] = iac_lap[tmp_index][1];
        lmp_self_device_data.iac_lap[tmp_index][2] = iac_lap[tmp_index][2];
        if(tmp_index >= 1)
        {
            lmp_self_device_data.diac_lap = TRUE;
            BT_FW_HCI_INF(DIAC_LAP_IS_RECEIVED_DIAC_LAP,1,lmp_self_device_data.diac_lap);
        }
        else
        {
            lmp_self_device_data.diac_lap = FALSE;
            BT_FW_HCI_INF(GIAC_LAP_IS_RECEIVED_DIAC_LAP,1,lmp_self_device_data.diac_lap);
        }

        tmp_index++;
    }

    lmp_self_device_data.num_supported_iac = tmp_index;

    /* Reprogram scans */
    lc_handle_scan_mode_command();

    return HCI_COMMAND_SUCCEEDED ;
}

/**
 * Reads the Transmit_Power_Level values for the given
 * connection handle received as parameter from host.
 *
 * \param hci_cmd_ptr Pointer to the command packet.
 * \param ce_index    [OUT] Pointer to the Connection Entity Index.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_read_transmit_pwr_lvl(HCI_CMD_PKT *hci_cmd_ptr, UINT16 *ce_index)
{
    UINT16 conn_handle;
    UCHAR type;

    UCHAR am_addr;
    UINT16 address = 0;
    INT16 read ;

    BT_FW_EXTRACT_16_BITS(conn_handle, &(hci_cmd_ptr->cmd_parameter[0]));

    type = hci_cmd_ptr->cmd_parameter[2];

    if (type > 0x01)
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    /* Obtain the index corresponding to the connection handle */
    if (LMP_GET_CE_INDEX_FROM_CONN_HANDLE(conn_handle, ce_index) != API_SUCCESS)
    {
#ifdef LE_MODE_EN
        if (IS_BT40)
        {
            LL_CONN_HANDLE_UNIT *ll_handle;
            
            ll_handle = ll_fw_search_handle_unit_via_conn_handle(conn_handle);

            if (ll_handle != NULL)
            {
                if (type == 0)
                {
                    if (le_tx_power_max_index < ll_handle->cur_tx_power_index)
                    {
                        ll_handle->cur_tx_power_index = le_tx_power_max_index;
                    }
                    ll_handle->tx_power_level = (-3) * (le_tx_power_max_index - 
                                                ll_handle->cur_tx_power_index);
                }
                else
                {
                    ll_handle->tx_power_level = 0;
                }
                *ce_index = ll_handle->unit_id + LMP_MAX_CE_DATABASE_ENTRIES;
                return HCI_COMMAND_SUCCEEDED ;
            }
        }
#endif
        return NO_CONNECTION_ERROR;
    }

    if(type == 0)
    {
        /* Read the value from the LUT and convert thevalue corresponding
         to Db */
        am_addr = lmp_connection_entity[*ce_index].am_addr ;
        address = reg_MASTER_UPPER_LUT[am_addr];
        read = (INT16) BB_read_baseband_register( (UCHAR) address);

        read = (INT16) (read & 0x0E00);
        read >>= 9 ;
    }
    else
    {
        /* Read Max Transmit power level. */
        /* The 3bit value which is written into the BB LUT... */
        read = MAX_RADIO_TX_POWER;
    }

    /* Convert the value to the Db value....*/
    /* Value 0 is max power i,e 0dbm,
    The value 0-7 shows the decrement of the power in the range of -4dbm 
    (in 0379 rf, 
     The value 0-MAX_RADIO_TX_POWER shows the decrement of the power 
     in the range of -3dbm) */

    read = (INT16)((MAX_RADIO_TX_POWER - read)*-3);

    BT_FW_HCI_INF(TX_POWER_VALUE_FROM_THE_BASEBAND_REGISTER_VALUE,1,address,read);

    lmp_connection_entity[*ce_index].transmit_power_level = (INT8) read;
    return HCI_COMMAND_SUCCEEDED ;
}

/**
 * Validates the parameters of the Read_Automatic_Flush_Timeout
 * command and get the index of connection entity for the given
 * connection handle from the Host. This command will read the value of
 * Flush_Timeout parameter.
 *
 * \param hci_cmd_ptr Pointer to the command packet.
 * \param ce_index    [OUT] Pointer to Connection Entity Index.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_read_automatic_flush_timeout(HCI_CMD_PKT *hci_cmd_ptr,
        UINT16 *ce_index)
{
    UINT16 conn_handle;

    BT_FW_EXTRACT_16_BITS(conn_handle, &(hci_cmd_ptr->cmd_parameter[0]));

    /* Obtain the index corresponding to the connection handle */
    if(LMP_GET_CE_INDEX_FROM_CONN_HANDLE(conn_handle, ce_index)
            == API_FAILURE)
    {
        return NO_CONNECTION_ERROR;
    }

    return HCI_COMMAND_SUCCEEDED ;
}

/**
 * Validates the parameters of the Write_Automatic_Flush_Timeout
 * command and updates the flush timeout value for the given
 * connection handle from the Host.
 *
 * \param hci_cmd_ptr Pointer to the command packet.
 * \param ce_index    Pointer to the connection entity index.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_write_automatic_flush_timeout(HCI_CMD_PKT *hci_cmd_ptr,
        UINT16 *ce_index)
{
    UINT16 conn_handle;
    UINT16 flush_timeout;

    BT_FW_EXTRACT_16_BITS(conn_handle, &(hci_cmd_ptr->cmd_parameter[0]));

    BT_FW_EXTRACT_16_BITS(flush_timeout, &(hci_cmd_ptr->cmd_parameter[2]));
    
    if (flush_timeout > 0x07FF)
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    /* Obtain the index corresponding to the connection handle */
    if(LMP_GET_CE_INDEX_FROM_CONN_HANDLE(conn_handle, ce_index)
            == API_FAILURE)
    {
        return NO_CONNECTION_ERROR;
    }
#if defined(_RTK_PTA_TEST_) && defined(_ENABLE_PTA_FLUSH_PKT_FUN_) 
    //RT_BT_LOG(BLUE,FLUSH_TIME_OUT_VALUE,1,flush_timeout);
#endif
    lmp_connection_entity[*ce_index].flush_timeout = flush_timeout;
    LMP_INF(FLUSH_TIME_VALUE,1,lmp_connection_entity[*ce_index].flush_timeout);

    return HCI_COMMAND_SUCCEEDED ;
}

#ifndef _DISABLE_HCI_HOST_FLOW_CONTROL_
/**
 * Validates the parameters of the Host_Number_Of_Completed_Packets
 * command. This command is used by host to indicate that host to
 * is ready to receive more Hci packets for any connection handle.
 *
 * \param hci_cmd_ptr Pointer to the command packet.
 * \param ce_index    [OUT] Pointer to connection entity index.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_host_num_of_comp_pkts(HCI_CMD_PKT *hci_cmd_ptr,
                                       UINT16   *ce_index)
{
    UCHAR tmp_val;
    UCHAR no_of_conn_handles;
    UINT16 conn_handle;
    UINT16 host_num_comp_pkts;
#ifdef ENABLE_SCO
    UINT16 i;
    UCHAR sco_con_handle;
#endif
    UINT16 ce_index1 = 0;
    UCHAR offset = 0;

    if(lmp_self_device_data.flow_control_hc_to_host == 0)
    {
        /* Ignore any host num comp pkts sent before
           set_controller_to_host_flow_control
         */
        return HCI_COMMAND_SUCCEEDED;
    }

    no_of_conn_handles = hci_cmd_ptr->cmd_parameter[offset];
    offset++;

    tmp_val = 0;
    while (tmp_val < no_of_conn_handles)
    {
        if (hci_cmd_ptr->param_total_length < (1 + ((tmp_val+1) << 2)))
        {
            return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
        }

        /* Extract the handle */
        conn_handle = hci_cmd_ptr->cmd_parameter[offset] |
                     (hci_cmd_ptr->cmd_parameter[offset + 1] << 8);
        offset += 2;

        host_num_comp_pkts = hci_cmd_ptr->cmd_parameter[offset] |
                             (hci_cmd_ptr->cmd_parameter[offset + 1] << 8);
        offset += 2;

#ifdef ENABLE_SCO
        sco_con_handle = FALSE;

        if (lmp_get_sco_ce_index_from_conn_handle(conn_handle, &i)
                == API_SUCCESS)
        {
            sco_con_handle = TRUE;
        }

        if(sco_con_handle != TRUE)
#endif
        {
            if(LMP_GET_CE_INDEX_FROM_CONN_HANDLE(conn_handle,&ce_index1) ==
                    API_FAILURE)
            {
                return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
            }
        }

        tmp_val++;

    } /* End while() */

    offset = 0;
    no_of_conn_handles = hci_cmd_ptr->cmd_parameter[offset];
    offset++;

    tmp_val = 0;
    while (tmp_val < no_of_conn_handles)
    {
        /* Extract the handle */
        conn_handle = hci_cmd_ptr->cmd_parameter[offset] |
                      (hci_cmd_ptr->cmd_parameter[offset + 1] << 8);
        offset += 2;

        host_num_comp_pkts = hci_cmd_ptr->cmd_parameter[offset] |
                            (hci_cmd_ptr->cmd_parameter[offset + 1] << 8);
        offset += 2;

#ifdef ENABLE_SCO
        sco_con_handle = FALSE;
        if (lmp_get_sco_ce_index_from_conn_handle(conn_handle, &i)
                == API_SUCCESS)
        {
            sco_con_handle = TRUE;
        }

        if (sco_con_handle != TRUE)
#endif
        {
            if(LMP_GET_CE_INDEX_FROM_CONN_HANDLE(conn_handle,&ce_index1) ==
                    API_FAILURE)
            {
                return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
            }
            else
            {
                LMP_CONNECTION_ENTITY *ce_ptr;
                ce_ptr = &lmp_connection_entity[ce_index1];

                /* Decrement the number of outstanding buffers for this
                 * Connection Hnadle */
                if (ce_ptr->out_standing_data_to_host >= host_num_comp_pkts)
                {
                    ce_ptr->out_standing_data_to_host =
                       ce_ptr->out_standing_data_to_host - host_num_comp_pkts;

                    BT_FW_HCI_INF(RECEIVED_HOST_NUM_COMP_PKTS_FOR_CH_HOST_NUM_COMP_PKTS,
                                        1,conn_handle,host_num_comp_pkts);
                    BT_FW_HCI_INF(UPDATING_OUT_STANDING_DATA_TO_HOST,1,
                                        ce_ptr->out_standing_data_to_host);
                    handle_host_num_of_comp_pkts(host_num_comp_pkts);
                }
                else
                {
                    BT_FW_HCI_INF(INVALID_HOST_NUM_COMP_PACKETS_FOR_THE_CONN,3,
                                  conn_handle,ce_ptr->out_standing_data_to_host,
                                  host_num_comp_pkts);
                }
            }
        }
        tmp_val++;

    } /* End while() */

    return HCI_COMMAND_SUCCEEDED ;
}

/**
 * Resets the number of completed data packet structure.
 *
 * \param hci_cmd_ptr Pointer to the command packet.
 * \param ce_index    Connection Entity Index.
 *
 * \return None.
 *
 */
void hci_reset_host_num_complete(UINT16 conn_handle, UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY *ce_ptr;
    ce_ptr = &lmp_connection_entity[ce_index];

    if ((lmp_self_device_data.flow_control_hc_to_host == 0x01) ||
        (lmp_self_device_data.flow_control_hc_to_host == 0x03))
    {
        BT_FW_HCI_INF(HCI_RESET_HOST_NUM_COMPLETE_CALLED,0,0);
        if (ce_ptr->out_standing_data_to_host > 0)
        {
            BT_FW_HCI_INF(RESETTING_HOST_NUM_COMPLETE_OUT_STANDING_DATA_TO_HOST,
                                        1, ce_ptr->out_standing_data_to_host);
            handle_host_num_of_comp_pkts(ce_ptr->out_standing_data_to_host);
            ce_ptr->out_standing_data_to_host = 0;
        }
    }
}
#endif /* end of #ifndef _DISABLE_HCI_HOST_FLOW_CONTROL_*/

/**
 * Inteprets the Write Scan Enable command packet coming
 * from the host and puts the local device in the type of listening mode
 * as specified in the command
 *
 * \param hci_cmd_ptr Pointer to the command packet.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_write_scan_enable_command( HCI_CMD_PKT *hci_cmd_pkt_ptr)
{
    UCHAR scan_enable;

    scan_enable = hci_cmd_pkt_ptr->cmd_parameter[0];

#ifdef _BRUCE_PLC_OPEN_VIA_WRITE_SCAN
    if(scan_enable ==0x03)
    {
        /*(bruce) setting 0xE0[11]=1,codec_plc_en.
                (bruce) 0xE0[3]=1;CRC error is equal to data loss,
                will be reflected in 0x25c[7]=0*/
        UINT16 read;
        read = BB_read_baseband_register(VOICE_SETTING_REGISTER);
        read |= (UINT16)(BIT11);
        read |= (UINT16)(BIT3);
        RT_BT_LOG(RED,DAPE_TEST_LOG293,1,3333);
        RT_BT_LOG(GREEN,YL_DBG_HEX_2,2,VOICE_SETTING_REGISTER,read);
        BB_write_baseband_register(VOICE_SETTING_REGISTER,read);
#if 1        
        read= BB_read_baseband_register(PLC_CTRL_REGISTER);
        //0x23E[9]=1 :stop the extrapolation waveform,but data still go through the plc.(MUTE)   
        read |= (UINT16)(BIT9);
        RT_BT_LOG(RED,YL_DBG_HEX_2,2,PLC_CTRL_REGISTER,read);
        BB_write_baseband_register(PLC_CTRL_REGISTER, read);
#endif 
#if 0

        read= BB_read_baseband_register(PLC_EXT_REGISTER);    
    /*  0x254[1:0]      option_pkt_mse_type_sel. 
            0:   disable mse threshold.        1:     choose plaod_mse_est;  
            2:   choose header_mse_est;    3:     choose err_max_est;    0x254[2],
            0: normal mode        1: The length of ola depends on the length of the erasure
            0x254[3],
            0: when not found pitch, pitch equal to max pitch
            1: when not found pitch, pitch equal to min pitch
            0x254[14],
            0: normal mode
            1: msbc recovery takes 3ms, therefore, one more PKT loss*/
        read |= (BIT0);
        RT_BT_LOG(RED,YL_DBG_HEX_2,2,PLC_EXT_REGISTER,read);
        BB_write_baseband_register(PLC_EXT_REGISTER, read); 
#endif 
#if 0
        PLC_REG_CTRL plc_ctrl_reg;
        UINT16 read2;
        *(UINT16*)&plc_ctrl_reg = BB_read_baseband_register(PLC_CTRL_REGISTER);
        plc_ctrl_reg.opt_cf = 2;
        RT_BT_LOG(RED,YL_DBG_HEX_2,2,PLC_EXT_REGISTER,plc_ctrl_reg.opt_cf);
        BB_write_baseband_register(PLC_CTRL_REGISTER, *(UINT16*)&plc_ctrl_reg); 
#endif

    }
    else if(scan_enable ==0x02)
    {
        UINT16 reg_val;
        //(bruce) setting 0xE0[11]=0,codec_plc_disable.
        reg_val = BB_read_baseband_register(VOICE_SETTING_REGISTER);
        reg_val &=(UINT16)~(BIT11);
        reg_val &=(UINT16)~(BIT2);
        reg_val &=(UINT16)~(BIT3);
        RT_BT_LOG(RED,DAPE_TEST_LOG293,1,2222);
        RT_BT_LOG(GREEN,YL_DBG_HEX_2,2,VOICE_SETTING_REGISTER,reg_val);
        BB_write_baseband_register(VOICE_SETTING_REGISTER,reg_val);   
#if 1      
        reg_val= BB_read_baseband_register(PLC_CTRL_REGISTER);
        reg_val &= (UINT16)~(BIT9);
        RT_BT_LOG(RED,YL_DBG_HEX_2,2,PLC_CTRL_REGISTER,reg_val);
        BB_write_baseband_register(PLC_CTRL_REGISTER, reg_val);
#endif
#if 0

        reg_val= BB_read_baseband_register(PLC_EXT_REGISTER);
       /*  0x254[1:0]      option_pkt_mse_type_sel. 
               0:   disable mse threshold.        1:     choose plaod_mse_est;  
               2:   choose header_mse_est;    3:     choose err_max_est;    0x254[2],
               0: normal mode        1: The length of ola depends on the length of the erasure
               0x254[3],
               0: when not found pitch, pitch equal to max pitch
               1: when not found pitch, pitch equal to min pitch
               0x254[14],
               0: normal mode
               1: msbc recovery takes 3ms, therefore, one more PKT loss*/
       reg_val &= ~(BIT0);
       RT_BT_LOG(RED,YL_DBG_HEX_2,2,PLC_EXT_REGISTER,reg_val);
       BB_write_baseband_register(PLC_EXT_REGISTER, reg_val);      
#endif
#if 0
    PLC_REG_CTRL plc_ctrl_reg;
    UINT16 read2;
    *(UINT16*)&plc_ctrl_reg = BB_read_baseband_register(PLC_CTRL_REGISTER);
    plc_ctrl_reg.opt_cf = 0;
    RT_BT_LOG(RED,YL_DBG_HEX_2,2,PLC_EXT_REGISTER,plc_ctrl_reg.opt_cf);
    BB_write_baseband_register(PLC_CTRL_REGISTER, *(UINT16*)&plc_ctrl_reg); 
#endif

    }
    else
    {
        UINT16 reg_val;
        //(bruce) setting 0xE0[11]=0,codec_plc_disable.
        reg_val = BB_read_baseband_register(VOICE_SETTING_REGISTER);
        reg_val &=(UINT16)~(BIT11);
        RT_BT_LOG(RED,DAPE_TEST_LOG293,1,1111);
        RT_BT_LOG(GREEN,YL_DBG_HEX_2,2,VOICE_SETTING_REGISTER,reg_val);
        BB_write_baseband_register(VOICE_SETTING_REGISTER,reg_val);   
    }
#endif

    BT_FW_HCI_INF(SCAN_ENABLE_COMMAND_RECEIVED_FROM_THE_HOST,1,scan_enable);

    /* Verify the relationship between offset, length & total_length */
    if(scan_enable > 0x03)
    {
        BT_FW_HCI_ERR(INVALID_SCAN_PARAM_OR_WRONG_LENGTH,0,0);
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    lmp_self_device_data.scan_enable = scan_enable;

    if(lc_handle_scan_mode_command() == API_FAILURE)
    {
        BT_FW_HCI_ERR(LC_FAILED_TO_HANDLE_WRITE_SCAN_COMMAND,0,0);
        return HARDWARE_FAILURE_ERROR;
    }

#ifdef MINICARD_BT_LED_CONTROL
    BT_LED_WPAN_ON();
#endif

    return HCI_COMMAND_SUCCEEDED ;
}

/**
 * Inteprets the Write Page Scan Activity command packet
 * coming from the host and sets the Page Scan Interval and Page Scan
 * Window values to be used by the local device for scanning.
 *
 * \param hci_cmd_ptr Pointer to the command packet.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_write_page_scan_activity_command(HCI_CMD_PKT *hci_cmd_pkt_ptr)
{
    UINT16 page_scan_interval,page_scan_window;

    BT_FW_EXTRACT_16_BITS(page_scan_interval, &(hci_cmd_pkt_ptr->cmd_parameter[0]));

    BT_FW_EXTRACT_16_BITS(page_scan_window, &(hci_cmd_pkt_ptr->cmd_parameter[2]));

    BT_FW_HCI_INF(PAGE_SCAN_WINDOW_PAGE_SCAN_INTERVAL,1, page_scan_window,page_scan_interval);

    /*
     *Check the default Max and Min values for interval and window.
     *Verify the relationship between offset, length & total_length
     */
    if ((page_scan_interval < LMP_MIN_PAGE_SCAN_INTERVAL) ||
            (page_scan_interval > LMP_MAX_PAGE_SCAN_INTERVAL) ||
            (page_scan_window < LMP_MIN_PAGE_SCAN_WINDOW) ||
            (page_scan_window > LMP_MAX_PAGE_SCAN_WINDOW) ||
            (page_scan_interval < page_scan_window))
    {
        BT_FW_HCI_ERR(INVALID_PAGE_SCAN_ACTIVITY_PARAMS,0,0);
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    lmp_self_device_data.page_scan_interval = page_scan_interval;
    lmp_self_device_data.page_scan_window = page_scan_window;


#ifdef _CCH_LPS_
    lmp_update_lps_para();
#endif

    if ((page_scan_interval == page_scan_window) &&
            (page_scan_interval <= 0x0800))
    {
        lmp_self_device_data.page_scan_repetition_mode = SR_R0;
    }
    else if(page_scan_interval > 0x0800)
    {
        lmp_self_device_data.page_scan_repetition_mode = SR_R2;
    }
    else
    {
        lmp_self_device_data.page_scan_repetition_mode = SR_R1;
    }

    /* Reprogram scans */
    lc_handle_scan_mode_command();

    return HCI_COMMAND_SUCCEEDED;
}

/**
 * Inteprets the Write Inquiry Scan Activity command packet
 * coming from the host and sets the Inquiry Scan Interval and Inquiry
 * Scan Window values to be used by the local device for scanning.
 *
 * \param hci_cmd_ptr Pointer to the command packet.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_write_inquiry_scan_activity_command(HCI_CMD_PKT
        *hci_cmd_pkt_ptr)
{
    UINT16 inquiry_scan_interval,inquiry_scan_window;

    BT_FW_EXTRACT_16_BITS(inquiry_scan_interval, &(hci_cmd_pkt_ptr->cmd_parameter[0]));

    BT_FW_EXTRACT_16_BITS(inquiry_scan_window, &(hci_cmd_pkt_ptr->cmd_parameter[2]));

    BT_FW_HCI_INF(INQ_SCAN_WINDOW_INQ_SCAN_INTERVAL,1,inquiry_scan_window,inquiry_scan_interval);

    /* Verify the relationship between offset, length & total_length */
    if ((inquiry_scan_interval < LMP_MIN_INQ_SCAN_INTERVAL) ||
            (inquiry_scan_interval > LMP_MAX_INQ_SCAN_INTERVAL) ||
            (inquiry_scan_window < LMP_MIN_INQ_SCAN_WINDOW) ||
            (inquiry_scan_window > LMP_MAX_INQ_SCAN_WINDOW) ||
            (inquiry_scan_interval < inquiry_scan_window))
    {
        BT_FW_HCI_ERR(INVALID_INQ_SCAN_ACTIVITY_PARAMS,0,0);
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    lmp_self_device_data.inquiry_scan_interval = inquiry_scan_interval;

    lmp_self_device_data.inquiry_scan_window = inquiry_scan_window;

#ifdef _CCH_LPS_
    lmp_update_lps_para();
#endif

    /* Reprogram scan */
    lc_handle_scan_mode_command();

    return HCI_COMMAND_SUCCEEDED;
}

/**
 * Inteprets the Change Local Name command packet coming
 * from the host and updates the name of the self device as specified in
 * the command.
 *
 * \param hci_cmd_ptr Pointer to the command packet.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_change_local_name_command(HCI_CMD_PKT *hci_cmd_pkt_ptr)
{
    UINT32 i;

    if(hci_cmd_pkt_ptr->param_total_length > LMP_MAX_NAME_LENGTH)
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    memcpy(lmp_self_device_data.local_name,
           &hci_cmd_pkt_ptr->cmd_parameter[0], LMP_MAX_NAME_LENGTH);

#if 0
    lmp_self_device_data.local_name_len = (UCHAR) strlen((const char *)
                                          lmp_self_device_data.local_name);
#endif

    for(i=0; i<LMP_MAX_NAME_LENGTH; i++)
    {
        if(lmp_self_device_data.local_name[i] == 0x0)
            break;
    }
    lmp_self_device_data.local_name_len = (UCHAR) i;

    return HCI_COMMAND_SUCCEEDED;
}


/**
 * Inteprets the Write Connection Accept Timeout command
 * packet coming from the host and sets the connection timeout parameter
 * of the local device as specified in the command. This is used by the
 * Link Manager to decide as to how long should it wait before it aborts
 * the connection process with the remote device.
 *
 * \param hci_cmd_ptr Pointer to the command packet.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_write_connection_accept_timeout_command(HCI_CMD_PKT
        *hci_cmd_pkt_ptr)
{
    UINT16 conn_accept_timeout ;

    BT_FW_EXTRACT_16_BITS(conn_accept_timeout, &(hci_cmd_pkt_ptr->cmd_parameter[0]));

    /* Validate the connection accept timeout value.
       If it is not within the specified range, set it to default values.
     */

    /* Verify the relationship between offset, length & total_length */
    if ((conn_accept_timeout < 0x0001)||(conn_accept_timeout > 0xB540))
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    lmp_self_device_data.conn_accept_timeout = SLOT_VAL_TO_TIMER_VAL
            (conn_accept_timeout);
    BT_FW_HCI_INF(CONNECTION_ACCEPT_TIMEOUT_IN_SLOTS,1,conn_accept_timeout);
    BT_FW_HCI_INF(CONNECTION_ACCEPT_TIMEOUT_IN_MILLI_SECS,1,lmp_self_device_data.conn_accept_timeout);

    return HCI_COMMAND_SUCCEEDED;
}

/**
 * Inteprets the Write Page Timeout command packet coming
 * from the host and sets the page timeout parameter of the local device
 * as specified in the command. This is used by the Link Manager
 * to decide as to how long should it wait before it aborts the paging
 * process.
 *
 * \param hci_cmd_ptr Pointer to the command packet.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_write_page_timeout_command(HCI_CMD_PKT *hci_cmd_pkt_ptr)
{
    UINT16 page_timeout ;

    BT_FW_EXTRACT_16_BITS(page_timeout, &(hci_cmd_pkt_ptr->cmd_parameter[0]));

    BT_FW_HCI_INF(PAGE_TIMEOUT_RECD,1,page_timeout);

    /* Verify the relationship between offset, length & total_length */
    if(page_timeout < 0x0001)
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    lmp_self_device_data.page_timeout = page_timeout ;

    return HCI_COMMAND_SUCCEEDED;
}

/**
 * Writes the class of device parameter. The class of device
 * parameter is used to indicate the capabilities of the local device to
 * other devices.
 *
 * \param hci_cmd_ptr Pointer to the command packet.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_write_class_of_device_command(HCI_CMD_PKT *hci_cmd_pkt_ptr)
{
    UINT32 temp_param ;

    temp_param = hci_cmd_pkt_ptr->cmd_parameter[0] |
                (hci_cmd_pkt_ptr->cmd_parameter[1] << 8) |
                (hci_cmd_pkt_ptr->cmd_parameter[2] << 16);

    lmp_self_device_data.class_of_device = temp_param ;

    BB_write_baseband_register(CLASS_OF_DEVICE_REGISTER1, temp_param);    
    BB_write_baseband_register_lower_octet(CLASS_OF_DEVICE_REGISTER2, temp_param >> 16);

    return HCI_COMMAND_SUCCEEDED;
}

/**
 * Will write the values for the voice settings parameter
 * in self device database.
 *
 * \param hci_cmd_ptr Pointer to the command packet.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_write_voice_setting_command(HCI_CMD_PKT *hci_cmd_pkt_ptr)
{
    UINT16 voice_setting ;

    /* Input coding is in 8-9 bits in voice settings */
    UINT16 input_coding;
    UCHAR air_mode;
#ifdef _DAPE_TEST_FIX_HFP_NO_RECORDING_FOR_LENOVO
    UINT8 temp_var;
    LMP_CONNECTION_ENTITY *ce_ptr;
    UCHAR sco_disconn_search_result = 0;
#ifdef ENABLE_SCO
    /* to see if there is any sco link that is disconnecting. */
    if (lmp_self_device_data.total_no_of_sco_conn == 1)
    {
        for (temp_var = 0; temp_var < LMP_MAX_CE_DATABASE_ENTRIES; temp_var++)
        {
            ce_ptr = &lmp_connection_entity[temp_var];

            if(ce_ptr->entity_status == ASSIGNED)
            {
                /*if there is any sco link that is disconnecting,
                  then this value will be false. */
                sco_disconn_search_result = lmp_is_remove_sco_link_possible(temp_var);
            }
        }
    }
    if((lmp_self_device_data.total_no_of_sco_conn > 1) ||
        ((lmp_self_device_data.total_no_of_sco_conn == 1)
            &&(sco_disconn_search_result)))
    {
        BT_FW_HCI_INF(COMAND_IS_REJECTED_SCO_CONN_ALREADY_EXISTS,0,0);
        return COMMAND_DISALLOWED_ERROR ;
    }
#endif
#else
#ifdef ENABLE_SCO
    if(lmp_self_device_data.total_no_of_sco_conn >= 1)
    {
        BT_FW_HCI_INF(COMAND_IS_REJECTED_SCO_CONN_ALREADY_EXISTS,0,0);
        return COMMAND_DISALLOWED_ERROR ;
    }
#endif
#endif
    BT_FW_EXTRACT_16_BITS(voice_setting, &(hci_cmd_pkt_ptr->cmd_parameter[0]));
    air_mode = (UCHAR) (voice_setting & 0x03);

    RT_BT_LOG(GREEN, BT_FW_HCI_HCBB_INFO_1204, 1, voice_setting);

    input_coding = (UINT16) (voice_setting & INPUT_CODING_MASK);

    if(input_coding == RESERVED_FOR_FUTURE)
    {
        BT_FW_HCI_ERR(INVALID_INPUT_CODING_VALUE_INPUT_CODING_VALUE,1, input_coding);
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    /* Codec interface changes */
    switch(air_mode)
    {
        case 0:
        {
            switch(input_coding)
            {
                case INPUT_CODING_LINEAR:
                    break;

                case INPUT_CODING_MU_LAW: /* Fall through. */
                case INPUT_CODING_A_LAW:
                    return INVALID_HCI_COMMAND_PARAMETERS_ERROR;

                default:
                    break;
            }

            break;
        }

        case 1:
        case 2:
        case 3:
        {
            break;
        }

        default:
        {
            return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
        }
    }

    lmp_self_device_data.voice_setting = voice_setting;
    lmp_self_device_data.lmp_air_mode =
        lmp_convert_air_mode(air_mode, HCI_LAYER, LMP_LAYER);

    return HCI_COMMAND_SUCCEEDED;
}

/**
 * Handles the write like supervision timeout command and updates
 * link_supervision_timeout value in connection entity.
 *
 * \param hci_cmd_ptr Pointer to the command packet.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_write_link_supervision_timeout_command
(HCI_CMD_PKT *hci_cmd_ptr, UINT16 *ce_index)
{
    UINT16 connection_handle;
    UINT16 link_super_timeout ;
    UCHAR parameter_list[LMP_SUPERVISION_TIMEOUT_LEN];

#ifdef BT_FW_HCI_DEBUG
    UCHAR am_addr ;
#endif
    LMP_CONNECTION_ENTITY *ce_ptr = NULL;

    /* Extract the connection handle */
    BT_FW_EXTRACT_16_BITS(connection_handle, &(hci_cmd_ptr->cmd_parameter[0]));

    BT_FW_EXTRACT_16_BITS(link_super_timeout, &(hci_cmd_ptr->cmd_parameter[2]));

    /* Validate connection handle. */
    /* Obtain the index corresponding to the connection handle */
    if(LMP_GET_CE_INDEX_FROM_CONN_HANDLE(connection_handle, ce_index)
            == API_FAILURE)
    {
        return NO_CONNECTION_ERROR;
    }

    ce_ptr = &lmp_connection_entity[*ce_index];

    if ( (link_super_timeout < LMP_MIN_LINK_SUPERVISION_TIMEOUT) &&
            (link_super_timeout != 0) )
    {
        return PARAMETER_OUT_OF_MANDATORY_RANGE;
    }

    /* If the self device is a master send link supervision timeout to
     * its slave.
     */
    if(ce_ptr->remote_dev_role == SLAVE)
    {
        /* Check for low power modes */
#if defined(COMPILE_SNIFF_MODE) || defined(COMPILE_PARK_MODE)
        switch(ce_ptr->ce_status)
        {
#ifdef COMPILE_SNIFF_MODE
            case LMP_SNIFF_MODE: /* Fall through */
                if(ce_ptr->sniff_interval > link_super_timeout &&
                        link_super_timeout != 0)
                {
                    return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
                }
                break;
#endif
#ifdef COMPILE_PARK_MODE
            case LMP_PARK_MODE:
                return COMMAND_DISALLOWED_ERROR;
#endif
            default:
                break;
        }
#endif

        parameter_list[0] = LMP_SUPERVISION_TIMEOUT_OPCODE;
        parameter_list[2] = LSB(link_super_timeout);
        parameter_list[3] = MSB(link_super_timeout);
        lmp_generate_pdu(*ce_index, parameter_list,
                         LMP_SUPERVISION_TIMEOUT_LEN, MASTER_TID, LMP_NO_STATE_CHANGE);

        ce_ptr->stored_link_supervision_timeout = link_super_timeout;
    }
    else
    {
        /* As slave we cannot set link supervision timeout. */
        return COMMAND_DISALLOWED_ERROR;
    }

    return HCI_COMMAND_SUCCEEDED;
}

/**
 * Handles the write page scan period mode command.
 *
 * \param hci_cmd_ptr Pointer to the command packet.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_write_page_scan_period_mode_command(
    HCI_CMD_PKT *hci_cmd_pkt_ptr)
{
    UCHAR temp_param;

    temp_param = hci_cmd_pkt_ptr->cmd_parameter[0] ;

    /* Verify the relationship between offset, length & total_length */
    if(temp_param > 0x02 )
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    lmp_self_device_data.page_scan_period_mode = temp_param ;

    return HCI_COMMAND_SUCCEEDED;
}

/**
 * Handles the write page scan mode command.
 *
 * \param hci_cmd_ptr           Pointer to the command packet.
 * \param sent_event_flag [OUT] TRUE if an event is sent to host.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_write_page_scan_mode_command(HCI_CMD_PKT *hci_cmd_pkt_ptr,
        UCHAR *sent_event_flag)
{
    UCHAR temp_param;

    temp_param = hci_cmd_pkt_ptr->cmd_parameter[0];

    /* Verify the relationship between offset, length & total_length */
    if(temp_param > 0x01)
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    lmp_self_device_data.page_scan_mode = temp_param ;


    if(lmp_self_device_data.page_scan_mode == 0)
    {
        lmp_self_device_data.opt_page_scan_flag = FALSE ;
        *sent_event_flag = FALSE ;
        /* Mandatotry page scan mode... Don't send the page mode
        req PDU to the remote device....*/
        return HCI_COMMAND_SUCCEEDED ;
    }

    return HCI_COMMAND_SUCCEEDED;
}

/**
 * Handles the read link supervision timeout command.
 *
 * \param hci_cmd_ptr     Pointer to the command packet.
 * \param sent_event_flag [OUT] TRUE if an event is sent to host.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_read_link_supervision_timeout_command(HCI_CMD_PKT *hci_cmd_ptr,
        UCHAR* sent_event_flag)
{
    UINT16 connection_handle ;
    UCHAR ret_error = HCI_COMMAND_SUCCEEDED ;
    UINT16 ce_index;

    /* Extract the connection handle */
    BT_FW_EXTRACT_16_BITS(connection_handle, &(hci_cmd_ptr->cmd_parameter[0]));

    BT_FW_HCI_INF(READ_LINK_SUPERVISION_TIMEOUT_CONN_HANDLE,1,connection_handle);

    /* Validate connection handle.*/
    /* Obtain the index corresponding to the connection handle */
    if(LMP_GET_CE_INDEX_FROM_CONN_HANDLE(connection_handle, &ce_index)
            == API_FAILURE)
    {
        ret_error = NO_CONNECTION_ERROR ;
        ce_index = connection_handle;
    }

    /* Send comand complete event to the host. */
    hci_generate_command_complete_event(hci_cmd_ptr->cmd_opcode,
                                        ret_error, ce_index, NULL);

    *sent_event_flag = TRUE ;

    return HCI_COMMAND_SUCCEEDED ;
}

#ifdef TEST_MODE
/**
 * Validates the parameters of read loopback command.
 * This command will read the value of the test mode set for the device.
 *
 * \param hci_cmd_ptr Pointer to the command packet.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_read_loopback_command(HCI_CMD_PKT *hci_cmd_ptr)
{
    return HCI_COMMAND_SUCCEEDED;
}

/**
 * Validates the parameters of Write_loopback_command
 * and updates the self device test mode paramter with value received
 * from the Host.
 *
 * \param hci_cmd_ptr Pointer to the command packet.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_write_loopback_command(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR offset = 0;
    UCHAR i;
    UCHAR status;
    UCHAR reason;

    switch (hci_cmd_ptr->cmd_parameter[offset])
    {
        case HCI_NO_LOOPBACK_MODE:
            if (lmp_self_device_data.test_mode == HCI_LOCAL_LOOPBACK_MODE)
            {
                /* Send Disconnect events to Host */
                status = 0;
                reason = 0x13;
                hci_generate_disconnection_complete_event(status,0x01,reason);

                for (i = 0; i < HCI_LOOPBACK_NUM_SCO_LINKS; i++)
                {
                    hci_generate_disconnection_complete_event(
                        status,(UINT16)(i+2),reason);
                }
            }
/*  Added by Wallice Su for RTK LoopBack Mode.  2012/06/04  */
            enable_hci_loopback_mode = 0;            
/*  End Added by Wallice Su for RTK LoopBack Mode.  2012/06/04  */

            break;

        case HCI_LOCAL_LOOPBACK_MODE:
            if (lmp_self_device_data.test_mode != HCI_NO_LOOPBACK_MODE)
            {
                return COMMAND_DISALLOWED_ERROR;
            }

            /* Check if any connection exists. If so disallow this command */
            if (lmp_self_device_data.number_of_acl_conn != 0)
            {
                return COMMAND_DISALLOWED_ERROR;
            }


            /*
             *Generate 1 ACL connection handle and 3 SCO connection handles from LMP
             *and generate connection complete events for them.
             */
            hci_generate_test_mode_connection_complete_event(
                HCI_COMMAND_SUCCEEDED, 0x01,
                otp_str_data.bt_bd_addr, ACL_LINK,0x00);


            for (i = 0; i < HCI_LOOPBACK_NUM_SCO_LINKS; i++)
            {
                hci_generate_test_mode_connection_complete_event(
                    HCI_COMMAND_SUCCEEDED,(UINT16) (i+2),
                    otp_str_data.bt_bd_addr,SCO_LINK,0x00);
            }
            break;

        case HCI_REMOTE_LOOPBACK_MODE:
            if (lmp_self_device_data.test_mode != HCI_NO_LOOPBACK_MODE)
            {
                return COMMAND_DISALLOWED_ERROR;
            }

            if (lmp_self_device_data.number_of_acl_conn > 1)
            {
                return COMMAND_DISALLOWED_ERROR;
            }
            break;

/*  Added by Wallice Su for RTK LoopBack Mode.  2012/06/04  */
        case HCI_RTK_LOCAL_LOOPBACK_MODE:           
            enable_hci_loopback_mode = 1;            
            break;            
/*  End Added by Wallice Su for RTK LoopBack Mode.  2012/06/04  */

        default:
            return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    } /* End switch() */
#ifdef TEST_MODE
    //Make sure free or allocate buffer more time
    if ((hci_cmd_ptr->cmd_parameter[offset] == HCI_NO_LOOPBACK_MODE) ||
            (hci_cmd_ptr->cmd_parameter[offset] == HCI_LOCAL_LOOPBACK_MODE))
    {
        pf_switch_hci_dma_parameter(LEAVE_TEST_MODE);
    }
    else if (hci_cmd_ptr->cmd_parameter[offset] == HCI_REMOTE_LOOPBACK_MODE)
    {
        pf_switch_hci_dma_parameter(ENTER_TEST_MODE);
    }
    else
    {
        RT_BT_LOG(GREEN,NOT_SUPPORT_TEST_MODE,0,0);
    }
#endif
    /* Now update the test mode state in self device data */

    lmp_self_device_data.test_mode = hci_cmd_ptr->cmd_parameter[offset];


    RT_BT_LOG(RED, BT_FW_HCI_HCBB_INFO_1633, 1, lmp_self_device_data.test_mode);

    return HCI_COMMAND_SUCCEEDED ;
}

/**
 * Validates the input parameter and sets the device under
 * Test mode.
 *
 * \param hci_cmd_ptr Pointer to the command packet.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_enable_dut_command(HCI_CMD_PKT *hci_cmd_ptr)
{
    /* Controller will enter TEST_MODE on recieving LMP PDU */
    lmp_self_device_data.host_enable_test_mode = TRUE;
#ifdef TEST_MODE
    pf_switch_hci_dma_parameter(ENTER_TEST_MODE);
#endif
    return HCI_COMMAND_SUCCEEDED;
}

#endif /* TEST_MODE */

#ifdef COMPILE_HOLD_MODE
/**
* Validates the input parameters and sets the hold mode activity.
*
* \param hci_cmd_ptr Pointer to the command packet.
*
* \return HCI_COMMAND_SUCCEEDED or HCI error code.
*
*/
UCHAR hci_handle_write_hold_mode_activity_command(HCI_CMD_PKT *hci_cmd_pkt_ptr)
{
    UCHAR temp_param;

    temp_param = hci_cmd_pkt_ptr->cmd_parameter[0] ;

    /* Parameter validation. */
    if(temp_param > (BIT0 | BIT1 | BIT2) )
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    if(temp_param != 0)
    {
        lmp_self_device_data.hold_mode_activity = temp_param;
    }

    return HCI_COMMAND_SUCCEEDED;
}
#endif


