enum { __FILE_NUM__= 110 };

#include "UartPrintf.h"
#include "platform.h"
#include "timer.h"
#include "bz_log_defines.h"
#include "otp.h"
#include "gpio.h"
#ifdef _YL_LPS
#include "lc.h"
#endif

#ifdef MWS_ENABLE
#include "mws_imp.h"
#include "mws.h"
#endif


#ifdef _ENABLE_BTON_POWER_SAVING_
#include "power_control.h"
#endif

#include "bb_driver.h"
#include "mailbox.h"

#ifdef _ENABLE_RTK_PTA_
#include "pta.h"
#endif

#include "lc_internal.h"
#include "rlx4181.h"
#if ((defined _DAPE_NO_TRX_WHEN_LE_SEND_CONN_REQ_HW_TIMER) || \
	(defined _DAPE_NO_TRX_WHEN_LE_CONN_UPDT))
#include "le_hw_reg.h"
#endif	

#include "bt_fw_signals.h"

#ifdef _SUPPORT_FW_INDIRECT_READ_SIE_
#include "new_io.h"    
#endif


#ifdef _SUPPORT_POLLING_BASED_LPM_L1_
#include "dma_usb.h"
#endif

#ifdef _SUPPORT_WL_BT_SCOREBOARD_
#include "scoreboard.h"
#endif

#if defined(_SUPPORT_FW_INDIRECT_READ_SIE_)|| \
defined(_FONGPIN_TEST_USBCV_REMOTEWAKEUP_BY_GPIO0_)|| \
defined(_FONGPIN_TEST_BT_READ_PAGE0_)
UINT32 g_fongpin_test_timer_counter = 0;
#endif

#ifdef _FONGPIN_TEST_PI_ACCESS_3RD_MODEM_
UINT32 g_fongpin_test_timer_counter;
#endif


TIMER_ISR_BITMAP_S_TYPE timer_isr_en;
TIMER_HANDLE timer_isr_fun[HW_TIMER_NUM];
UINT32 g_lps_timer_counter;

#ifdef _SUPPORT_USB_TIMEOUT_INTERRUPT_
UINT16 g_u16usb_token_timeout_timer_cnt = 0;
#endif

#ifdef _DAPE_TEST_CHK_MWS_FRAME_SYNC
UINT8 g_gen_frame_sync = 0;
#endif

extern UINT32 g_timer2_init_value;
extern OS_HANDLE isr_extended_task_handle;
extern UINT8 g_ext_timer1_isr_bg_flag;

#ifdef _SUPPORT_POLLING_BASED_LPM_L1_
extern LPM_POLL_CONTROL g_lpm_poll_control_s;
#endif


#if defined(_FONGPIN_TEST_BT_READ_PAGE0_) || \
    defined(_FONGPIN_TEST_BT_READ_SIE_) || \
    defined(_FONGPIN_TEST_TIMER_POLLING_REG_) || \
    defined(_FONGPIN_TEST_PI_ACCESS_3RD_MODEM_) || \
    defined(_FONGPIN_TEST_USBCV_REMOTEWAKEUP_BY_GPIO0_)

#define FONGPIN_TEST
void fongpin_test_entry();
#endif

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
void pf_timer_init(UINT8 dlps_flow)
#else
void pf_timer_init(void)
#endif
{

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
    if(dlps_flow == FALSE)
#endif
    {
#ifdef _CCH_LPS_   
        g_lps_timer_counter = g_timer2_init_value;
#else
        g_lps_timer_counter = 0;
#endif
    }

    g_ext_timer1_isr_bg_flag = FALSE;

    timer_var_init();
#ifndef IS_BTSOC
    timer_on_off(TIMER1_ID, 10*1000, 1); /* program 10 ms now */
#endif
}

void timer_var_init(void)
{
    UINT8 i;
    UINT16 offset;
    
    timer_isr_en.d8 = 0;
    timer_isr_fun[0] = timer1_handle;
    timer_isr_fun[1] = timer2_handle;
#ifndef IS_BTSOC
    timer_isr_fun[2] = timer3_handle;
#ifdef _NEW_8812B_CPU_PLATFORM_    
    timer_isr_fun[3] = timer4_handle;    
#endif
#endif
    
    /* disable all timer and timer interrupt */
    offset = TIMER1_CTL_REG_OFF;
    for (i = 0; i < HW_TIMER_NUM; i++)
    {
        TIMER_WRITE(offset, 0x06);
        offset += 0X14;
    }

    /* clear any pening timer interrupts */
    TIMER_READ(TIMERS_EOI_OFF);
}

/**************************************************************************
 * Function     : timer_on_off
 *
 * Description  : This function is used to enable/disable the hardware timer.
 *
 * Parameters   : timer_id: the id of timer (0 ~ 3 means timer 1 ~ 4)
 *                usec: the expired time (unit: us)
 *                is_on: 1 means ON and 0 means OFF
 * Returns      : None
 *
 *************************************************************************/
void timer_on_off(UINT8 timer_id, UINT32 usec, UINT8 is_on)
{
    UINT32 timer_ctl_reg_off;
    UINT32 timer_load_count_off;
    UINT32 timer_load_count;

    if (timer_id < HW_TIMER_NUM)
    {
        timer_ctl_reg_off = TIMER1_CTL_REG_OFF + (0x14*timer_id);
        
        //Enable timer
        if (is_on)
        {            
            timer_load_count_off = TIMER1_LOAD_COUNT_OFF + (0x14*timer_id);
        
            if (!usec)
            {
                usec = 1;
                RT_BT_LOG(RED,TIMER_TIK_ERROR,0,0);
            }

            timer_load_count = ((usec / 1000) * TIMER_TICK_COUNT_PER_MS) +
                   (((usec % 1000) * TIMER_TICK_COUNT_PER_MS)/1000); 
            /*
            set Timer1ControlReg
            0: Timer enable (0,disable; 1,enable)
            1: Timer Mode (0, free-running mode; 1, user-defined count mode)
            2: Timer Interrupt Mask (0, not masked; 1,masked)
            */
            // set timer control register
            TIMER_WRITE(timer_ctl_reg_off, 0x06);

            // set Timer LoadCount Register 
            TIMER_WRITE(timer_load_count_off, timer_load_count);

            //record timer id status
            timer_isr_en.d8 |= (1<<timer_id);

            // set timer control register
            TIMER_WRITE(timer_ctl_reg_off, 0x03);
        }
        else
        {
            //record timer id status
            timer_isr_en.d8 &= ~(1<<timer_id);
            
            //Disable timer
            TIMER_WRITE(timer_ctl_reg_off, 0x06);
        }
    }
    else
    {
        RT_BT_LOG(RED,NO_SUPPORT_TIMER,0,0);
    }

}

/**************************************************************************
 * Function     : timer_update_expired_time
 *
 * Description  : This function is used to update the expired time to assigned 
                  hardware timer.
 *
 * Parameters   : timer_id: the id of timer (0 ~ 3 means timer 1 ~ 4)
 *                usec: the expired time (unit: us)
 * Returns      : None
 *
 *************************************************************************/
void timer_update_expired_time(UINT8 timer_id, UINT32 usec)
{
    timer_on_off(timer_id, 0, 0);
    timer_on_off(timer_id, usec, 1);
}

/**************************************************************************
 * Function     : timer1_handle
 *
 * Description  : This function is an interrupt service routine of HW timer 1. 
 *                It is used to handle 10 ms periodical timeout event. It is 
 *                also the 10 ms Hardware TICK generator of software timer.
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
void timer1_handle(void)
{
#ifndef IS_BTSOC
    pf_timer_handle_tick();
#endif

#if defined(_ENABLE_BTON_POWER_SAVING_) && defined(_TIMER_LPS_MODE_EN_)
    pow_timer_lps();
#endif

    if (g_ext_timer1_isr_bg_flag == FALSE)
    {
        OS_SIGNAL sig_send;
        sig_send.type = ISR_EXT_TIMER1_CALLBACK_BG;  
        OS_ISR_SEND_SIGNAL_TO_TASK(isr_extended_task_handle, sig_send); 
        g_ext_timer1_isr_bg_flag = TRUE;
    }
}


/**************************************************************************
 * Function     : timer2_handle
 *
 * Description  : This function is an interrupt service routine of HW Timer 2.
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
void timer2_handle(void)
{
    /* disable the timer */
    timer_on_off(TIMER2_ID, 0, 0);  

#ifdef _ALLOW_ENTER_USB_LPM_L1_VIA_HW_TIMER2_
    /* check hci dma rx desc is free or not ? */    
    if ((otp_str_data.USB_LPM_Control & BIT2) && 
                (get_lpm_l1_token_reponse() == L1_NYET) &&
                   checkRxDescAllFree(CHECK_BY_OWN))
    {
        /* update new lpm l1 token response to usb sie */
        set_lpm_l1_token_reponse(L1_ACK);
    }   

    g_usb_lpm_l1_hw_timer_is_running = 0;   
#endif

#ifdef _ENABLE_RTK_PTA_
#ifndef _DISABLE_OBSOLETE_MAILBOX_CMD_
    if (tdma_state)
    {
        pta_execule_tdma_procedure();

        UINT32 expred_time;

        expred_time = (pta_tdma_manage.wlan_active_time + 
                     pta_tdma_manage.bt_max_active_time) * 1000;

        timer_on_off(TIMER2_ID, expred_time, 1);
    }
#endif    
#endif    
}

/**************************************************************************
 * Function     : timer3_handle
 *
 * Description  : This function is an interrupt service routine of HW Timer 3.
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
void timer3_handle(void)
{
    /* TODO */
#ifdef _DAPE_EN_8821_MP_LE_SCAN_INTR
    /* TODO */
    timer_on_off(TIMER3_ID, 0, 0);  
    BZ_REG_S_PRI_CTRL2 pri_ctrl2;            
    *(UINT16*)&pri_ctrl2 = BB_read_baseband_register(SCA_PRIORITY_REGISTER3);
//    RT_BT_LOG(WHITE, DAPE_TEST_LOG213, 2, pri_ctrl2.block_legacy_acl, BB_read_native_clock());
    /* If Legacy ACL is ON */
    if (pri_ctrl2.block_legacy_acl == FALSE)
    {
        if (bb_switch_scheduler_to_le())
        {
            /* Turn on Timer3 for turn on trx*/
            timer_on_off(TIMER3_ID, 10*1000, 1);  
        }
    }
    else
    {
        bb_switch_scheduler_to_legacy();
    }
#endif
#if ((defined _DAPE_NO_TRX_WHEN_LE_SEND_CONN_REQ_HW_TIMER) || \
	(defined _DAPE_NO_TRX_WHEN_LE_CONN_UPDT))
    if (IS_TIMER3_FOR_LE_DUAL_WHCK)
    {
        /* TODO */
        timer_on_off(TIMER3_ID, 0, 0);  
        BZ_REG_S_PRI_CTRL1 pri_ctrl1;            
        *(UINT16*)&pri_ctrl1 = BB_read_baseband_register(SCA_PRIORITY_REGISTER2);
        RT_BT_LOG(GREEN, DAPE_TEST_LOG213, 2, pri_ctrl1.mask_txrx_timing, BB_read_native_clock());
    	
        /* If TRX is ON */
        if (pri_ctrl1.mask_txrx_timing == TRUE)
        {
            UINT16 reg_e0 = BB_read_baseband_register(VOICE_SETTING_REGISTER);
            if (!(reg_e0 & BIT12))
            {
                bb_switch_scheduler_to_le();
                /* Turn on Timer3 for turn on trx*/
                timer_on_off(TIMER3_ID, 10*1000, 1);  
        	}
            else
            {
                /* It maybe already in page/inq resp state, so do not pause trx.*/
                RT_BT_LOG(RED, DAPE_TEST_LOG213, 2,VOICE_SETTING_REGISTER, reg_e0);
                timer_on_off(TIMER3_ID, 5*1000, 1);  
            }
        }
        else
        {	
            /* If TRX is OFF */
            /* Turn on trx */ 
            bb_switch_scheduler_to_legacy();	
        }
    }
#endif

#ifdef _DAPE_TEST_NO_TRX_WHEN_LE
    timer_on_off(TIMER3_ID, 0, 0);  
    BZ_REG_S_PRI_CTRL1 pri_ctrl1;            
    *(UINT16*)&pri_ctrl1 = BB_read_baseband_register(SCA_PRIORITY_REGISTER2);

    /* If TRX is ON */
    if (pri_ctrl1.mask_txrx_timing == TRUE)
    {
        UINT16 reg_e0 = BB_read_baseband_register(VOICE_SETTING_REGISTER);
        if (!(reg_e0 & BIT12))
        {
            /* Turn off scan to make sure we do not read VOICE_SETTING_REGISTER in
               the transcient state.*/
            /* Kill all scan procedures in the baseband */
            lc_kill_scan_mode();
            /* Turn off trx */ 
            pri_ctrl1.mask_txrx_timing = FALSE;
            BB_write_baseband_register(SCA_PRIORITY_REGISTER2, *(UINT16*)&pri_ctrl1);
            lc_check_and_enable_scans_in_scatternet();			
            /* Turn on Timer3 for turn on trx*/
            timer_on_off(TIMER3_ID, 90*1000, 1);  
    	}
        else
        {
            /* It maybe already in page/inq resp state, so do not pause trx.*/
            //RT_BT_LOG(GREEN, DAPE_TEST_LOG213, 2,VOICE_SETTING_REGISTER, reg_e0);
			timer_on_off(TIMER3_ID, 90*1000, 1);  
        }
    }
    else
    {	
        /* If TRX is OFF */
        /* Turn on trx */ 
        pri_ctrl1.mask_txrx_timing = TRUE;
        BB_write_baseband_register(SCA_PRIORITY_REGISTER2, *(UINT16*)&pri_ctrl1);
    }
#endif

}

#ifdef _NEW_8812B_CPU_PLATFORM_    
/**************************************************************************
 * Function     : timer4_handle
 *
 * Description  : This function is an interrupt service routine of HW Timer 4.
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
void timer4_handle(void)
{
    /* disable the timer */
    timer_on_off(TIMER4_ID, 0, 0);
}  
#endif

SECTION_ISR void TimerIntrHandler(void)
{
    UINT8 timer_id;
    UINT32 raw_int_status;

    /* contain all masked acktive interrupts */
    raw_int_status = TIMER_READ(TIMERS_INT_STATUS_OFF);

    /* clear all interrupt */
    TIMER_READ(TIMERS_EOI_OFF);

    raw_int_status &= timer_isr_en.d8;

    for (timer_id = 0; timer_id < HW_TIMER_NUM; timer_id++)
    {        
        if (raw_int_status & (1 << timer_id))
        {
            (*timer_isr_fun[timer_id])();
        }
    }
}

inline void reset_vendor_counter(void)
{
    /* reser the counter */
    VENDOR_WRITE(0x17C, (1 << 31));
    
    /* set the counter */
    VENDOR_WRITE(0x17C, 0);    
}

inline UINT32 read_vendor_counter(UINT8 index)
{
    UINT32 cur_tick;

    cur_tick = VENDOR_READ(0x17C) & 0x3FFFFFF;
    RT_BT_LOG(YELLOW, BZ_VENDOR_TIMER_MEASURE, 2, index, cur_tick);
    return cur_tick;
}

inline UINT32 read_vendor_counter_no_display(void)
{
    return (VENDOR_READ(0x17C) & 0x3FFFFFF);
}

#ifdef _CP3_COUNTER_SUPPORTED_
void reset_cp3_counter(void)
{
    rlx4081Cp3CntInit();
}

void stop_cp3_counter(UINT8 index)
{
    UINT32 ic0, ic1, ic2, ic3;
    ic0 = rlx4081Cp3StopCnt0();
    ic1 = rlx4081Cp3StopCnt1();
    ic2 = rlx4081Cp3StopCnt2();
    ic3 = rlx4081Cp3StopCnt3();

//    RT_TMP_LOG("INS COUNTER: id %d cycle %d I_cache_miss %d I_cache_miss_cycle %d I_fetch_cycle %d\n", 0, ic0, ic1, ic2, ic3);

    RT_BT_LOG(YELLOW, MSG_CP3_COUNTER_MEASURE, 5, 0, ic0, ic1, ic2, ic3);
    return;
}
#endif

#if ((defined _DAPE_NO_TRX_WHEN_LE_SEND_CONN_REQ_HW_TIMER) || \
	(defined _DAPE_NO_TRX_WHEN_LE_CONN_UPDT))
void turn_on_timer3_for_le(UINT16 conn_win_size_ofst, UINT16 conn_win_ofst)
{
    UINT32 first_pkt = (conn_win_size_ofst + conn_win_ofst)*1250;
    UINT32 turn_off_time = 0;
    /* HW timer*/
    if (first_pkt > 5000)
    {
        turn_off_time = first_pkt - 5000;
    }
    timer_on_off(TIMER3_ID, turn_off_time, 1);  
    BZ_REG_S_PRI_CTRL1 pri_ctrl1;            
    *(UINT16*)&pri_ctrl1 = BB_read_baseband_register(SCA_PRIORITY_REGISTER2);
    
    /* enable TRX again to make sure that when timer3 is called it will turn 
        off trx.*/
    pri_ctrl1.mask_txrx_timing = TRUE;
    BB_write_baseband_register(SCA_PRIORITY_REGISTER2, *(UINT16*)&pri_ctrl1);
    
    RT_BT_LOG(RED, DAPE_TEST_LOG525, 6, 
        RD_LE_REG(0x22), conn_win_size_ofst, conn_win_ofst, 
        BB_read_native_clock(), turn_off_time, RD_LE_REG(0x24));
}
#endif

#if defined(_FONGPIN_TEST_BT_READ_PAGE0_) || \
    defined(_FONGPIN_TEST_BT_READ_SIE_) || \
    defined(_FONGPIN_TEST_TIMER_POLLING_REG_) || \
    defined(_FONGPIN_TEST_PI_ACCESS_3RD_MODEM_) || \
    defined(_FONGPIN_TEST_USBCV_REMOTEWAKEUP_BY_GPIO0_)

void fongpin_test_entry()
{

#if defined(_FONGPIN_TEST_BT_READ_PAGE0_)
    //RT_BT_LOG(WHITE, YL_DBG_DEC_1, 1,0);
    indirect_access_read_syson_test();
#endif

#if defined(_FONGPIN_TEST_BT_READ_SIE_)
    //RT_BT_LOG(WHITE, YL_DBG_DEC_1, 1,1);
    indirect_access_read_sie_test();
#endif
    
#if defined(_FONGPIN_TEST_PI_ACCESS_3RD_MODEM_)
        //RT_BT_LOG(WHITE, YL_DBG_DEC_1, 1,2);
    ThirdGenModemPiAccessTest();
#endif
    
#if defined(_FONGPIN_TEST_USBCV_REMOTEWAKEUP_BY_GPIO0_)
    //RT_BT_LOG(WHITE, YL_DBG_DEC_1, 1,3);
    test_usb_remotewakeup_by_detect_gpio0_high();
#endif
    
#if defined(_FONGPIN_TEST_TIMER_POLLING_REG_)
    //RT_BT_LOG(WHITE, YL_DBG_DEC_1, 1,4);
    timer_polling_io_dbg();
#endif

}

#endif

