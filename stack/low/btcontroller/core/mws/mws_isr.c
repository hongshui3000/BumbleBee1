

/********************************* Logger *************************/
enum { __FILE_NUM__= 114 };
/********************************* Logger *************************/
#include "mws.h"
#include "mws_imp.h"
#include "mws_isr.h"
#include "bb_driver.h"
#include "lmp_defines.h"
#include "lmp.h"

extern UINT8 g_mws_tx ;
extern UINT8 g_mws_rx ;
#ifdef _DAPE_TEST_CHK_MWS_FRAME_SYNC
UINT8 g_modify_clk = 0;
UINT8 g_modify_clk_value = 0;
UINT8 g_modify_clk_cnt = 0;
#endif
#ifdef _DAPE_ENABLE_MWS_FRAME_SYNC
extern LMP_SELF_DEVICE_DATA lmp_self_device_data;
extern const UINT16 reg_PICONET_INFO[4];
#endif
#ifdef _DAPE_FRAME_SYNC_PROCESS_TO_BG 
extern OS_HANDLE isr_extended_task_handle;
#endif                
#ifdef _DAPE_TEST_FRAME_SYNC_UPDATE_CHOOSE_WAY
UINT8 g_modify_way = 0;
#endif
#ifdef _ROM_CODE_PATCHED_
PF_ROM_CODE_PATCH_FUNC rcp_mws_int_handler = NULL;
#endif          

UINT32 mws_current_interrupt_enable;
SECTION_ISR void mws_int_handler(void)
{
#ifdef _ROM_CODE_PATCHED_
		if (rcp_mws_int_handler != NULL)
		{
			if ( rcp_mws_int_handler(NULL))
			{
				return;
			}
		}
#endif

    UINT32 mws_interrupt_status_reg;
    UINT32 mws_interrupt_status_valid_reg;
    
    mws_interrupt_status_reg = mws_read_register_new(INTERRUPT_STATUS_REG);
    mws_interrupt_status_valid_reg =  (mws_interrupt_status_reg&mws_current_interrupt_enable);

    if( mws_interrupt_status_valid_reg )
    {
        if (NRT_BT_TX_DONE_INTERRUPT_STATUS(mws_interrupt_status_valid_reg))
        {
            mws_handle_nrt_bt_tx_done_interrupt(mws_interrupt_status_valid_reg);
        }
        if (NRT_RX_INTERRUPT_STATUS(mws_interrupt_status_valid_reg))
        {
            mws_handle_nrt_rx_interrupt();
        }
        if (UART_LSR_INTERRUPT_STATUS(mws_interrupt_status_valid_reg))
        {
            mws_handle_uart_lsr_interrupt();
        }
        if (RT_RX_INTERRUPT_STATUS(mws_interrupt_status_valid_reg))
        {
            mws_handle_rt_rx_interrupt();
        }
        if (UART_NRT_RX_TIMEOUT_STATUS(mws_interrupt_status_valid_reg))
        {
            mws_handle_uart_nrt_rx_timeout_interrupt();
        }
        if (NRT_RX_FULL_STATUS(mws_interrupt_status_valid_reg))
        {
            mws_handle_nrt_rx_full_interrupt(mws_interrupt_status_valid_reg);
        }
        if (RT_RX_FULL_STATUS(mws_interrupt_status_valid_reg))
        {
            mws_handle_rt_rx_full_interrupt(mws_interrupt_status_valid_reg);
        }
        if (UART_RT_INTERRUPT_STATUS(mws_interrupt_status_valid_reg))
        {
            mws_handle_uart_rt_rx_timeout_interrupt();
        }
        if (LTE_TX_TO_STATUS(mws_interrupt_status_valid_reg))
        {
            mws_handle_lte_tx_to_interrupt(mws_interrupt_status_valid_reg);
        }
        if (LTE_RX_TO_STATUS(mws_interrupt_status_valid_reg))
        {
            mws_handle_lte_rx_to_interrupt(mws_interrupt_status_valid_reg);
        }
        if (LTE_INT_PATTERN1_STATUS(mws_interrupt_status_valid_reg))
        {
            mws_handle_lte_int_pattern1_interrupt(mws_interrupt_status_valid_reg);
        }
        if (LTE_INT_PATTERN2_STATUS(mws_interrupt_status_valid_reg))
        {
            mws_handle_lte_int_pattern2_interrupt(mws_interrupt_status_valid_reg);
        }
        if (LTE_FRAMESYNC_STATUS(mws_interrupt_status_valid_reg))
        {
            mws_handle_lte_framesync_interrupt(mws_interrupt_status_valid_reg);
        }
        if (LTE_TX_ON_STATUS(mws_interrupt_status_valid_reg))
        {
            mws_handle_lte_tx_on_interrupt(mws_interrupt_status_valid_reg);
        }
        if (UART_RT_INTERRUPT_STATUS(mws_interrupt_status_valid_reg))
        {
            mws_handle_uart_rt_rx_timeout_interrupt();
        }
        if (LTE_TX_TO_IDLE_RX_STATUS(mws_interrupt_status_valid_reg))
        {
            mws_handle_lte_tx_to_idle_rx_interrupt(mws_interrupt_status_valid_reg);
        }
        if (NRT_WL_TX_DONE_INTERRUPT_STATUS(mws_interrupt_status_valid_reg))
        {
            mws_handle_nrt_wlan_tx_done_interrupt(mws_interrupt_status_valid_reg);
        }
    }
	
}
SECTION_ISR void mws_handle_lte_rx_to_interrupt(UINT32 interrupt_status)
{
    g_mws_rx = 0;
    if(!mws_write_register(INTERRUPT_STATUS_REG,(interrupt_status|BIT15),0xf))
    {
        RT_BT_LOG(RED,MWS_WRITE_REG_FAIL,0,0);
        return  ;
    }
}

SECTION_ISR void mws_handle_lte_tx_to_interrupt(UINT32 interrupt_status)
{   
    g_mws_tx = 0;
    if(!mws_write_register(INTERRUPT_STATUS_REG,(interrupt_status|BIT14),0xf))
    {
        RT_BT_LOG(RED,MWS_WRITE_REG_FAIL,0,0);
        return  ;
    }
}

SECTION_ISR void mws_handle_lte_int_pattern2_interrupt(UINT32 interrupt_status)
{
    if(!mws_write_register(INTERRUPT_STATUS_REG,(interrupt_status|BIT13),0xf))
    {
        RT_BT_LOG(RED,MWS_WRITE_REG_FAIL,0,0);
        return  ;
    }
}
SECTION_ISR void mws_handle_lte_int_pattern1_interrupt(UINT32 interrupt_status)
{
    if(!mws_write_register(INTERRUPT_STATUS_REG,(interrupt_status|BIT12),0xf))
    {
        RT_BT_LOG(RED,MWS_WRITE_REG_FAIL,0,0);
        return  ;
    }
}

SECTION_ISR void mws_handle_lte_framesync_interrupt(UINT32 interrupt_status)
{
    //RT_BT_LOG(GREEN,YL_DBG_HEX_2,2,359,read_vendor_counter(1));
#ifdef _DAPE_ENABLE_MWS_FRAME_SYNC
#ifdef _DAPE_FRAME_SYNC_PROCESS_TO_BG
    OS_SIGNAL sig_send;
    sig_send.type = ISR_EXT_FRAME_SYNC_INTR;  
    OS_ISR_SEND_SIGNAL_TO_TASK(isr_extended_task_handle, sig_send); 
#endif    
#else
    
#ifndef _DAPE_TEST_CHK_MWS_FRAME_SYNC
    RT_BT_LOG(GREEN,YL_DBG_DEC_1,1,699);
#else
#ifndef _DAPE_TEST_CHK_MWS_FRAME_SYNC_LATCH_CLK_CORRECTNESS
    if (g_modify_clk)
#else
    g_modify_clk_cnt ++;
    g_modify_clk_cnt %= 100;
    if (g_modify_clk_cnt == 0)
#endif
    {
        UINT16 reg274 = BB_read_baseband_register(MWS_CLKADJ_CTRL4);
        UINT16 reg276 = BB_read_baseband_register(MWS_CLKADJ_RPT0);
        UINT16 reg27a = BB_read_baseband_register(MWS_CLKADJ_RPT2);
        
        BB_write_baseband_register(MWS_CLKADJ_CTRL4, reg274&(~BIT15));
        RT_BT_LOG(BLUE, DAPE_TEST_LOG559, 6,
        BB_read_native_clock(),
        BB_read_baseband_register(MWS_CLKADJ_CTRL4),
        reg274, reg276, reg27a,reg274&((UINT16)(0x03FF))
        );

        g_modify_clk ++;
        if (g_modify_clk == 6)
       	{
            g_modify_clk = 0;
        }        
    }
#endif    
#endif    
    if(!mws_write_register(INTERRUPT_STATUS_REG,(interrupt_status|BIT11),0xf))
    {
        RT_BT_LOG(RED,MWS_WRITE_REG_FAIL,0,0);
        return  ;
    }
}

SECTION_ISR void mws_handle_lte_tx_to_idle_rx_interrupt(UINT32 interrupt_status)
{
	g_mws_tx = 0;
    if(!mws_write_register(INTERRUPT_STATUS_REG,(interrupt_status|BIT10),0xf))
    {
        RT_BT_LOG(RED,MWS_WRITE_REG_FAIL,0,0);
        return  ;
    }
}

SECTION_ISR void mws_handle_lte_tx_on_interrupt(UINT32 interrupt_status)
{
	g_mws_tx = 1;
    if(!mws_write_register(INTERRUPT_STATUS_REG,(interrupt_status|BIT9),0xf))
    {
        RT_BT_LOG(RED,MWS_WRITE_REG_FAIL,0,0);
        return  ;
    }
}

SECTION_ISR void mws_handle_nrt_bt_tx_done_interrupt(UINT32 status)
{
    //0x200 bit[0] W1C
    if(!mws_write_register(INTERRUPT_STATUS_REG,(status|BIT0),0xf))
    {
        RT_BT_LOG(RED,MWS_WRITE_REG_FAIL,0,0);
        return  ;
    }
}

SECTION_ISR void mws_handle_nrt_wlan_tx_done_interrupt(UINT32 interrupt_status)
{   
    //0x200 bit[1] W1C
    mws_write_register(INTERRUPT_STATUS_REG,(interrupt_status|BIT1),0xf);
}

SECTION_ISR void mws_handle_nrt_rx_interrupt(void)
{
#ifdef MWS_16_BIT_MODE
    mws_to_bt_message_16bit(MWS_NRT_MESSAGE);
#elif defined (MWS_8_BIT_MODE)
    mws_to_bt_message_8bit(MWS_NRT_MESSAGE);
#endif
}

SECTION_ISR void mws_handle_rt_rx_interrupt(void)
{
#ifdef MWS_16_BIT_MODE
    mws_to_bt_message_16bit(MWS_RT_MESSAGE);
#elif defined (MWS_8_BIT_MODE)
    mws_to_bt_message_8bit(MWS_RT_MESSAGE);
#endif
}

SECTION_ISR void mws_handle_uart_nrt_rx_timeout_interrupt(void)
{
#ifdef MWS_16_BIT_MODE
    mws_to_bt_message_16bit(MWS_NRT_MESSAGE);
#elif defined(MWS_8_BIT_MODE)
    mws_to_bt_message_8bit(MWS_NRT_MESSAGE);
#endif
}

SECTION_ISR void mws_handle_uart_rt_rx_timeout_interrupt(void)
{
#ifdef MWS_16_BIT_MODE
    mws_to_bt_message_16bit(MWS_RT_MESSAGE);
#elif defined(MWS_8_BIT_MODE)
    mws_to_bt_message_8bit(MWS_RT_MESSAGE);
#endif
}

SECTION_ISR void mws_handle_uart_lsr_interrupt(void)
{
    UINT32 uart_lsr_status;
    uart_lsr_status = mws_read_register_new(UART_LINE_STATUS_REG);
    if(uart_lsr_status & BIT20)
    {
        RT_BT_LOG(RED,MWS_RT_BREAK_ERR_INT,0,0);
    }
    if(uart_lsr_status & BIT19)
    {
        RT_BT_LOG(RED,MWS_RT_FRAME_ERR_INT,0,0);
    }
    if(uart_lsr_status & BIT18)
    {
        RT_BT_LOG(RED,MWS_RT_PARITY_ERR_INT,0,0);
    }
    if(uart_lsr_status & BIT17)
    {
        RT_BT_LOG(RED,MWS_RT_OVERRUN_ERR_INT,0,0);
    }
    if(uart_lsr_status & BIT16)
    {
    #ifdef MWS_16_BIT_MODE
        mws_to_bt_message_16bit(MWS_RT_MESSAGE);
    #elif defined (MWS_8_BIT_MODE)
        mws_to_bt_message_8bit(MWS_RT_MESSAGE);
    #endif
        RT_BT_LOG(RED,MWS_RT_RXFIFO_RDY_INT,0,0);
    }
    if(uart_lsr_status & BIT7)
    {
        RT_BT_LOG(RED,MWS_UART_RX_ERROR_INT,0,0);
    }
    if(uart_lsr_status & BIT4)
    {
        RT_BT_LOG(RED,MWS_NRT_BREAK_ERR_INT,0,0);
    }
    if(uart_lsr_status & BIT3)
    {
        RT_BT_LOG(RED,MWS_NRT_FRAME_ERR_INT,0,0);
    }
    if(uart_lsr_status & BIT2)
    {
        RT_BT_LOG(RED,MWS_NRT_PARITY_ERR_INT,0,0);
    }
    if(uart_lsr_status & BIT1)
    {
        RT_BT_LOG(RED,MWS_NRT_OVERRUN_ERR_INT,0,0);
    }
    if(uart_lsr_status & BIT0)
    {
        RT_BT_LOG(RED,MWS_NRT_RXFIFO_RDY_INT,0,0);
    }
    //return ;
}

SECTION_ISR void mws_handle_nrt_rx_full_interrupt(UINT32 interrupt_status)
{
#ifdef MWS_16_BIT_MODE
            mws_to_bt_message_16bit(MWS_NRT_MESSAGE);
#elif defined(MWS_8_BIT_MODE)
            mws_to_bt_message_8bit(MWS_NRT_MESSAGE);
#endif
    if(!mws_write_register(INTERRUPT_STATUS_REG,(interrupt_status|BIT7),0xf))
    {
        RT_BT_LOG(RED,MWS_WRITE_REG_FAIL,0,0);
        return  ;
    }
}

SECTION_ISR void mws_handle_rt_rx_full_interrupt(UINT32 interrupt_status)
{
#ifdef MWS_16_BIT_MODE
            mws_to_bt_message_16bit(MWS_RT_MESSAGE);
#elif defined(MWS_8_BIT_MODE)
            mws_to_bt_message_8bit(MWS_RT_MESSAGE);
#endif
    if(!mws_write_register(INTERRUPT_STATUS_REG,(interrupt_status|BIT8),0xf))
    {
        RT_BT_LOG(RED,MWS_WRITE_REG_FAIL,0,0);
        return  ;
    }
}

