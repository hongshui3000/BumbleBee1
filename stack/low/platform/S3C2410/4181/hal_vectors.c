#include "platform.h"
#include "bzdma.h"
#include "timer.h"
#include "lc.h"
#include "mailbox.h"
#include "power_control.h"
#include "lmp_vendor_defines.h"


void BTMAC_Handler(void)
{
    BTMAC_ISR_STATUS_REG reg = btmac_isr_status_reg_read();

    if (reg.timer_0_1_int)
    {
        TimerIntrHandler();
    }

    if (reg.bt_bluewiz_int)
    {
        baseband_interrupt_handler();
    }

    if (reg.bt_hcidma_int)
    {
        USB_DMA_IntrHandler();
    }

    if (reg.bt_bzdma_int)
    {
        bzdma_int_handler();
    }
}


UINT8 g_ext_timer1_isr_bg_flag = FALSE;

void isr_ext_timer1_callback_bg_func(void)
{
#ifdef _SUPPORT_POLLING_BASED_LPM_L1_
    if (IS_SUPPORT_USB_LPM_L1_FSM &&
                (g_fun_interface_info.b.bt_interface == USB_INTERFACE))
    {
#ifdef _SUPPORT_NEW_USB_LPM_L1_DATA_PATH_150629_
        DEF_CRITICAL_SECTION_STORAGE;
        MINT_OS_ENTER_CRITICAL();
        hci_dma_usb_check_and_dequeue_notification();
        MINT_OS_EXIT_CRITICAL();
#endif /* end of #ifdef _SUPPORT_NEW_USB_LPM_L1_DATA_PATH_150629_   */
    }
#endif

#ifdef _SUPPORT_TIMER1_SHUTDOWN_LOG_
    timer1_shutdown_log();
#endif

#ifdef _SUPPORT_USB_TIMEOUT_INTERRUPT_
    timer1_disbale_scoreboard_int();
#endif

#if defined(FONGPIN_TEST)
    fongpin_test_entry();
#endif


#ifdef _ENABLE_MAILBOX_
    mailbox_task_check();
#endif

#ifdef _MODEM_LNA_CONSTRAINT_
    if (bBtLNAConstraint && (!CHECK_WIFI_ALIVE_FOR_PTA))
    {
        lc_set_modem_lna_constraint_on_off(0);
    }
#endif

#ifdef _BRUCE_TEST_PLC_PKT_STATUS
    plc_pkt_status_var.g_plc_print_clk++;
    if(plc_pkt_status_var.g_plc_print_clk%500==0)
    {
        plc_print_esco_pkt_status();
    }
#endif
#ifdef _DAPE_ENABLE_MWS_FRAME_SYNC
    bt_mws_var.frame_sync_detect_timer ++;
    if (bt_mws_var.frame_sync_detect_timer > bt_mws_var.frame_sync_disable_threshold)
    {
        mws_disable_frame_sync_update();
    }
#endif
#ifdef _PLC_TEST_MODE_ENABLE_
    //bruce_test_count
    bt_clk_ctrl++;
    if(bt_clk_ctrl%2==0)
    {
    plc_test_emu_toggle_bluetooth_clock();
    }
#endif
#ifdef _DAPE_DETECT_WHCK_P2P
    if (g_whck_p2p_running_detect_countdown)
    {
        g_whck_p2p_running_detect_countdown --;
    }
#endif

#ifdef _DAPE_TEST_CHK_MWS_FRAME_SYNC
    if (g_gen_frame_sync)
    {
        mws_write_register(0x300,(UINT32)(BIT31|BIT30),0xf);
        mws_write_register(0x300,(UINT32)BIT31,0xf);
    }
#if 0
    UINT32 reg1, reg2, reg3, reg4;
    mws_read_register(0x300, &reg1);
    mws_read_register(0x304, &reg2);

    mws_read_register(0x200, &reg3);
    mws_read_register(0x204, &reg4);

    RT_BT_LOG(WHITE, DAPE_TEST_LOG525, 6, BB_read_native_clock(), reg1, reg2, reg3, reg4,0);
#endif
#endif
#ifdef _DAPE_ADD_CNT_FOR_TIMER1
   g_10ms_timer_ticks++;

    if (g_10ms_timer_ticks == 100)
    {
#ifdef _DAPE_DETECT_WHCK_P2P
        if (g_whck_running)
        {
            RT_BT_LOG(GREEN, DAPE_TEST_LOG553, 2, g_whck_item, g_p2p_detect_step);
        }
#endif
#ifdef _TEST_ADAPTIVITY_FUNC_2
        UINT32 reg5C = RTK_READ_MODEM_REG_PI(MODEM_PI_PAGE_3, TRANS_MODEM_REG(0x5C));
        UINT8 reg_tx_gain_set_manu_en = (reg5C >> 6) & BIT0;
        UINT8 reg_tx_gain_set_manu_val_7_0 = (reg5C >> 7) & 0xFF;
        RT_BT_LOG(GRAY, YL_DBG_HEX_5, 5,
            RTK_READ_MODEM_REG_PI(MODEM_PI_PAGE_6, TRANS_MODEM_REG(0x14)),
            RTK_READ_MODEM_REG_PI(MODEM_PI_PAGE_0, TRANS_MODEM_REG(0x7E)),
            RTK_READ_MODEM_REG_PI(MODEM_PI_PAGE_3, TRANS_MODEM_REG(0x5C)),
            reg_tx_gain_set_manu_en, reg_tx_gain_set_manu_val_7_0);
#endif

        g_10ms_timer_ticks = 0;
    }
#endif

#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_
    rssi_app_msft_check_all_counter_every_100ms();
#endif

    g_ext_timer1_isr_bg_flag = FALSE;
}

void  isr_extended_task_function ( OS_SIGNAL *signal )
{
    switch ( signal->type )
    {
#ifdef ENABLE_LOGGER
    case ISR_EXT_UART_RX:
        {
            UINT16 rptr = (UINT16)((UINT32 )signal->param);
            UINT16 len = (UINT16)((UINT32 )signal->ext_param);
            uart_dbg_receive_packet_delayed(rptr, len);
        }
        break;
#endif

#ifdef _INQRES_DBG_CCH_
    case ISR_EXT_INQ_RES_LOG:
        BB_handle_tx_int_debug_inqres();
        break;
#endif

    case ISR_EXT_SLOT_OFST_LOG:
        global_slot_offset_log();
        break;

#if defined(_ENABLE_MAILBOX_)
    case ISR_EXT_RESET_MAILBOX_TASK:
        mailbox_reset_task();
        break;
#endif
#ifdef _DAPE_FRAME_SYNC_PROCESS_TO_BG
    case ISR_EXT_FRAME_SYNC_INTR:
        mws_handle_lte_frame_sync_isr();
        break;
#endif
#if defined(_CCH_LPS_) && defined(_YL_LPS)
#ifdef _REDUCE_USED_STACK_SIZE_IN_DLPS_MODE_
        case ISR_EXT_ENTER_LPS_MODE:
        {
            UINT32 wakeup_instant = (UINT32)(signal->param);
            UCHAR lps_mode =  (UCHAR)((UINT32)(signal->ext_param) >> 8);
            UCHAR piconet_id = (UCHAR)((UINT32)(signal->ext_param) & 0xff);
            UINT8 ret_value;
            ret_value = LC_PROGRAM_LPS_MODE(lps_mode, wakeup_instant, piconet_id);
            if( ret_value != API_SUCCESS)
            {
#ifdef _MODI_LPS_AFTER_RTL8821B_TC_
                if( ret_value == API_FAILURE)
                {
                    lc_check_lps_for_resume();
                }
#else
                lc_check_and_enable_scans_in_scatternet();
#endif
            }
        }
        break;
#endif
#endif

    case ISR_EXT_TIMER1_CALLBACK_BG:
        isr_ext_timer1_callback_bg_func();
        break;


    case ISR_EXT_BOTTOM_HALF_CALLBACK_PROCESS:
        {
            PF_ISR_EXT_BOTTOM_HALF_FUNC callback;
            callback = (PF_ISR_EXT_BOTTOM_HALF_FUNC)signal->param;
            callback(signal->ext_param);
        }
        break;

    default:
        break;
    }
}
