enum { __FILE_NUM__= 111 };

#include "platform.h"
#include "rlx4181.h"
#include "UartPrintf.h"
#include "timer.h"
#include "gpio.h"
#include "uart.h"
#include "lbm.h"
#include "mint_os_task_internal.h"
#include "mint_os_queue_internal.h"
#include "lc.h"
#include "logger.h"
#include "bb_driver.h"
#include "timer.h"
#include "bzdma.h"
#ifdef _ENABLE_BTON_POWER_SAVING_
#include "power_control.h"
#endif
#include "mailbox.h"
#include "plc.h"
#ifdef _FIX_CROSSBAR_ERROR_DEBUG_
#include "mint_os_timer_internal.h"
#include "bz_utils.h"
#endif

#ifdef MWS_ENABLE
#include "mws.h"
#include "mws_imp.h"
#endif

#include "lmp_vendor_defines.h"
#ifdef _SUPPORT_INFO_FROM_SYSTEM_ON_    
#include "system_on.h"
#endif

#ifdef _SUPPORT_INFO_FROM_SYSTEM_ON_    
extern UINT8 g_chip_id;
#endif            

#ifdef _DAPE_TEST_CHK_MWS_FRAME_SYNC
extern UINT8 g_gen_frame_sync;
#endif
#ifdef _DAPE_TEST_FRAME_SYNC_UPDATE_CHOOSE_WAY
extern UINT8 g_modify_way;
#endif
#ifndef FOR_SIMULATION
#ifdef ENABLE_LOGGER
extern lbm_queue_t lbm_queue;
#endif
#endif
#ifdef _DAPE_DETECT_WHCK_P2P
UINT16 g_whck_p2p_running_detect_countdown = 0;
UINT8 g_p2p_detect_step = 0;
extern UINT8 g_whck_running;
extern UINT8 g_whck_item;
#endif 
#ifdef _DAPE_ADD_CNT_FOR_TIMER1
UINT16 g_10ms_timer_ticks = 0;
#endif

volatile unsigned char InISR = 0; /* add by austin for critical section
                                     enhanced protect and it can used to check
                                     certain function is called in ISR or 
                                     background */
volatile unsigned char IntCtrlSP = 0;

UINT8 g_ext_timer1_isr_bg_flag = FALSE;

#ifdef _TASK_CALLBACK_DBG_
UINT16 dbg_hw_int_count = 0;
#endif

#ifdef _BRUCE_WDG_TIMEOUT_DEBUG
extern SECTION_SRAM UINT8 patch_in_isr;
extern SECTION_SRAM UINT8 patch_isr_num;
extern SECTION_SRAM UINT8 patch_in_cpu_idle;
extern SECTION_SRAM UINT16 patch_signal_id;
extern SECTION_SRAM UINT32 patch_isr_cause;
extern SECTION_SRAM UINT32 patch_isr_cnts_during_one_callback;
extern SECTION_SRAM UINT32 patch_last_epc;
extern SECTION_SRAM UINT32 patch_ra;
extern unsigned char _intr_stack[];

#endif

#ifdef _ROM_CODE_PATCHED_
PF_ROM_CODE_PATCH_FUNC rcp_bt_interrupt_handler = NULL;
#endif

#ifdef _NEW_PLATFORM_XB_INTR_EN
#define XB_BUS_REG_BASE                  0xb0002000 /* only for 8703b on V6 */
//#define XB_BUS_REG_BASE                  0xb0003000 /* other project */
#define WR_U32_XB_BUS_REG(offset, val)   WR_32BIT_IO(XB_BUS_REG_BASE, offset, val)
#define RD_U32_XB_BUS_REG(offset)        RD_32BIT_IO(XB_BUS_REG_BASE, offset)
#define ERR_ACCESS_ADDR                 0x40
#define ERR_ACCESS_CMD_LEN              0x44
#define ERR_ACCESS_TAGID                0x48
#define ERR_ACCESS_INFO                 0x4C
#endif

#ifdef _FIX_CROSSBAR_ERROR_DEBUG_
extern SECTION_SRAM OS_TIMER dbg_os_timer;
#endif

void ErrExcptHandler(void);

#ifdef _INQRES_DBG_CCH_
extern void BB_handle_tx_int_debug_inqres(void);
#endif

#ifdef _SUPPORT_USB_TIMEOUT_INTERRUPT_
extern void timer1_disbale_scoreboard_int(void);
#endif


extern OS_TASK_MGR task_mgr;

#ifdef ENABLE_LOGGER
extern OS_HANDLE bz_logger_task_handle;
#endif

extern UINT8 is_uart_init;

#ifdef _ENABLE_BTON_POWER_SAVING_
extern UINT8 isr_switch;
extern MIPS_CAUSE_REG_S cpu_filter;
#endif

/**
 * Function     : BtIntrHandler
 *
 * Description  : This function is for handling all interrupt
 *                      CPU: 4081
 *                      case bit 2: bluewiz interrupt
 *                      case bit 3: usb(hci dma) interrupt
 *                      case bit 4: bzdma interrupt
 *                      case bit 5: gpio interrupt
 *                      case bit 6: timer interrupt
 *                      case bit 7: uart interrupt
 *                      CPU: 4181
 *                      ecase bit 8: usb interrupt
 *                      ecase bit 9: rf interrupt
 *                      ecase bit 10: gpio interrupt
 *                      ecase bit 11: timer interrupt
 *                      ecase bit 12: uart interrupt
 *                      
 * Parameters   : None.
 *
 * Returns      : None
 *
 * Side Effects : None
 *
*/
SECTION_ISR void BtIntrHandler(void)
{
    MIPS_CAUSE_REG_S cause;

    InISR = 1;
#ifdef _DAPE_TEST_ACL_RX_FIFO_DBG
    //gpio_one_pull_high(2);
#endif
#ifdef _TASK_CALLBACK_DBG_
    dbg_hw_int_count++;
#endif

    *(UINT32*)&cause = Rlx4181ReadCause();

#ifdef _ROM_CODE_PATCHED_
    if (rcp_bt_interrupt_handler != NULL)
    {
        if (rcp_bt_interrupt_handler(&cause))
        {
            return;
        }
    }
#endif

#ifdef _ENABLE_BTON_POWER_SAVING_
    /* for reduce the latency after receive hw interrupt in same logic 
       - austin */
    *(UINT32*)&cause &= (*(UINT32*)&cpu_filter) | 0xFFFF0BFF;
#endif

#ifdef _BRUCE_WDG_TIMEOUT_DEBUG
    patch_in_isr = TRUE;
    patch_isr_cause = *(UINT32*)&cause;
    patch_isr_cnts_during_one_callback++;
    patch_isr_num = 0;
    patch_last_epc = Rlx4181ReadEPC();
    patch_ra = *((UINT32 *)(&(_intr_stack[112])));
#endif

    if (cause.exc_code)
    {
    
#ifdef _BRUCE_WDG_TIMEOUT_DEBUG
        patch_isr_num |= BIT0; 
#endif
    
        /* any abnormal exception (fw needs to analyze why happen this) */  
        ErrExcptHandler();
    }

    if (cause.timer_int)
    {
#ifdef _BRUCE_WDG_TIMEOUT_DEBUG
        patch_isr_num |= BIT1; 
#endif
        /* hw timer interrupt is triggered */
        TimerIntrHandler();
    }

    if (cause.bluewiz_int)
    {
#ifdef _BRUCE_WDG_TIMEOUT_DEBUG
            patch_isr_num |= BIT2; 
#endif
        
        /* bluewiz or le controller interrupt is triggered */
        baseband_interrupt_handler();
    }

    if (cause.bzdma_int)
    {
#ifdef _BRUCE_WDG_TIMEOUT_DEBUG
        patch_isr_num |= BIT3; 
#endif   
        
        /* bzdma or mailbox or scoreboard interrupt is triggered */
    	bzdma_int_handler();
    }
    if (cause.hci_dma_int)
    {
#ifdef _ENABLE_BTON_POWER_SAVING_
        if (cpu_filter.hci_dma_int)
#endif
        {
#ifdef _BRUCE_WDG_TIMEOUT_DEBUG
            patch_isr_num |= BIT4; 
#endif       
            /* hci dma interrupt (usb/uart/pcie) is triggered */        
            USB_DMA_IntrHandler();
        }
#ifdef _BRUCE_WDG_TIMEOUT_DEBUG
        patch_isr_num |= BIT5; 
#endif
#ifdef _ENABLE_BTON_POWER_SAVING_
        /*BTON interrupt*/
        pow_ctrl_intr_handle();
#endif
    }

#if defined(ENABLE_LOGGER)
    if (cause.uart_int)
    {
#ifdef _NEW_PLATFORM_XB_INTR_EN
        UINT32 error_access;
        UINT32 base_address_plus = 0;

#if !defined(_BT_ONLY_)		
#if defined(_SUPPORT_INFO_FROM_SYSTEM_ON_)
#if !defined(_RTL8703B_SPECIFIC_)
        if (g_chip_id != CHIP_ID_8703B) /* for not 8703B V6*/
        {
            base_address_plus = 0x1000;
        }
        /* 8703B, already defined. */
        /*else
        {
            base_address_plus = 0;
        }*/
#endif        
#endif
#else
        base_address_plus = 0x1000;   /* for V5 ?? */
#endif
		error_access = RD_U32_XB_BUS_REG(base_address_plus + ERR_ACCESS_INFO);

        if (error_access)
        {
            UINT32 program_counter, err_addr, err_ofst, err_tagid;

            err_addr = RD_U32_XB_BUS_REG(base_address_plus + ERR_ACCESS_ADDR);
            err_ofst = RD_U32_XB_BUS_REG(base_address_plus + ERR_ACCESS_CMD_LEN);
            err_tagid = RD_U32_XB_BUS_REG(base_address_plus + ERR_ACCESS_TAGID);

            program_counter = VENDOR_READ(0x60);
            RT_BT_LOG(RED, DAPE_TEST_LOG585, 6, base_address_plus,
                program_counter, err_addr, err_ofst, err_tagid, error_access);
            
#ifdef _FIX_CROSSBAR_ERROR_DEBUG_
            print_timer_info();
/*            RT_BT_LOG(RED, YL_DBG_HEX_10, 10,
                    dbg_os_timer.handle, 
                    dbg_os_timer.cal_table_index,
                    dbg_os_timer.timer_count,
                    dbg_os_timer.timer_value,
                    (UINT32)dbg_os_timer.timeout_function,
                    dbg_os_timer.timer_type,
                    (UINT32)dbg_os_timer.args,
                    (UINT32)dbg_os_timer.next,
                    dbg_os_timer.state,
                    dbg_os_timer.sniff_tick_timer);*/
#endif
            
        }
        else
#endif
        {
#ifdef _BRUCE_WDG_TIMEOUT_DEBUG
            patch_isr_num |= BIT6; 
#endif
            /* debugger uart interrupt is triggered */          
            UartIntrHandler();
        }
    }
#endif

#if defined(_GPIO_POWER_SEQUENCE_ENABLE)
    if (cause.gpio_int)
    {
#ifdef _BRUCE_WDG_TIMEOUT_DEBUG
        patch_isr_num |= BIT7; 
#endif  
        /* gpio interrupt is triggered */   
        GpioIntrHandler();
    }
#endif//RADIO_SWITCH_SUPPORT

#ifdef _ENABLE_BTON_POWER_SAVING_
    if (isr_switch)
    {
    
#ifdef _BRUCE_WDG_TIMEOUT_DEBUG
        patch_isr_num |= BIT8; 
#endif   
        isr_switch = 0;
        rlx4081_isr_sw_unmask();
    }
#endif

#ifdef _BRUCE_WDG_TIMEOUT_DEBUG    
    patch_isr_num = 0xff;
    patch_in_isr = FALSE;
    patch_isr_cause = 0;
#endif

#ifdef _DAPE_TEST_ACL_RX_FIFO_DBG
    //gpio_one_pull_low(2);
#endif

    InISR = 0;
}

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

#ifdef EXCEPTION_REPORT_SUPPORT
//====================================================
//an while(1) loop, error exception will be trap here.
//[1] report CP0_CAUSE
//[2] reprot CP0_EPC
//[3] todo: reset whole system? (use watchdog reset now)
//====================================================

extern unsigned char _intr_stack[];

void uart_direct_gen_msg(UINT32 str_id, UINT16 len, UINT8 *buf)
{    
    UINT16 i;
    
    //l_msg[0] = l_msg[1] = 0xA5;
    UARTBytePut(0xA5);
    UARTBytePut(0xA5);    

    //l_msg[2] = ((color&0x7)<<5)               //color
    //  | 0x04                              //1=has file number
    //  | 0x02;                             //1=has line number
    if (len > 0)
    {
        UARTBytePut(0x4E);
    }
    else
    {
        UARTBytePut(0x46);
    }

    //l_msg[3]                              //reserved byte
    UARTBytePut(0x5A);

    //l_msg[4] = (UCHAR)(log_str_index & 0xff);
    //l_msg[5] = (UCHAR)((log_str_index & 0xff00)>>8);
    //l_msg[6] = (UCHAR)((log_str_index & 0xff0000)>>16);
    //l_msg[7] = (UCHAR)((log_str_index & 0xff000000)>>24);
    UARTBytePut(str_id);
    UARTBytePut(str_id >> 8); 
    UARTBytePut(str_id >> 16);     
    UARTBytePut(str_id >> 24); 

    //l_msg[8] = (UCHAR)(file_num&0xff);            //8 bits file_num
    //l_msg[9] = (UCHAR)(line_num&0xff);            //8 bits line_num
    //l_msg[10] = (UCHAR)((line_num&0xff00)>>8);    //8 bits line_num
    UARTBytePut(0);
    UARTBytePut(0);
    UARTBytePut(0);

    if (len > 0)
    {
        //l_msg[11] = (UCHAR)para_num;
        UARTBytePut((len + 3)>> 2);

        for (i = 0; i < len; i++)
        {
            UARTBytePut(buf[i]);
        }

        while (i & 0x03)
        {
            /* fill enough data */
            UARTBytePut(0);
            i++;
        }
    }    
}

UINT16 uart_direct_print_all_log_buf(void)
{
#ifdef ENABLE_LOGGER
    UINT16 i;
    UINT16 len = MIN(lbm_queue.length, LBM_BUFFER_SIZE);
    UINT16 rptr = lbm_queue.rd_ptr;    

    for (i = 0; i < len; i++)
    {
        UARTBytePut(lbm_queue.buf[rptr]);
        rptr = (rptr + 1) & (LBM_BUFFER_SIZE - 1);
    }

    return len;
#else
    return 0;
#endif
}

void ErrExcptHandler(void)
{
    UINT8 buf[24];
    UINT32 log_len;
    
    WDG_TIMER_DISABLE;

    //BTON_WDG_TIMER_DISABLE;

    *((UINT32*)buf) = Rlx4181ReadCause();                       /* cp0 cause */
    *((UINT32*)(buf + 4)) = *((UINT32 *)(&(_intr_stack[124]))); /* cp0 epc */
    *((UINT32*)(buf + 8)) = *((UINT32 *)(&(_intr_stack[112]))); /* cp0 ra */
    *((UINT32*)(buf + 12))= *((UINT32 *)(&(_intr_stack[104]))); /* cp0 sp */
    *((UINT32*)(buf + 16)) = *((UINT32 *)(&(_intr_stack[4])));  /* cp0 v0 */
    *((UINT32*)(buf + 20)) = *((UINT32 *)(&(_intr_stack[8])));  /* cp0 v1 */

    if (!is_uart_init)
    {
        /* avoid to enter exception before initialize uart */
        UARTInit(115200, UART_PARITY_DISABLE, UART_STOP_1BIT,
            UART_DATA_LEN_8BIT, 0x81, 0x00);
    }

    uart_direct_gen_msg(ERR_EXP_REPORT_CP0_CAUSE, 24, buf);

    log_len = uart_direct_print_all_log_buf();

    uart_direct_gen_msg(ERR_EXP_EXPORT_LOG_BUF_END, 4, (UINT8*)&log_len);
    uart_direct_gen_msg(ERR_EXP_SPACE, 0, NULL);
    uart_direct_gen_msg(ERR_EXP_SPACE, 0, NULL);
    uart_direct_gen_msg(ERR_EXP_SPACE, 0, NULL); 

    /* reset the CPU via watchdog timeout */
    WDG_TIMER_TIMEOUT_SOON;
    
    for (;;);
}
#endif//EXCEPTION_REPORT_SUPPORT

