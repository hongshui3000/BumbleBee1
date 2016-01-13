/***************************************************************************
 Copyright (C) Realtek
 This module is a confidential and proprietary property of Realtek and
 a possession or use of this module requires written permission of Realtek.
 ***************************************************************************/

enum { __FILE_NUM__= 113 };
#include "mws_imp.h"
#include "bb_driver.h"
#include "mws.h"
#include "timer.h"
#include "logger.h"
#include "platform.h"
#include "bt_fw_hci_spec_defines.h"
#include "bt_fw_os.h"
#include "lmp_ch_assessment.h"
#include "mem.h"

UINT16 mws_retry_count;         //MSG[3:0]
UINT16 lte_band;                //MSG[9:4]
UINT16 mws_intf_band_used;      //MSG[10]
UINT8  g_mws_pattern_index;

UINT8  g_mws_tx = 0;
UINT8  g_mws_rx = 0;
UINT8  g_ignore_mws_trx = 0;
UINT8  g_is_slot_mode = 0;

extern UINT32 mws_current_interrupt_enable;
extern UINT8 wlan_off_to_on_flag;
extern UINT8 wlan_on_to_off_flag;

#ifdef _ROM_CODE_PATCHED_
PF_ROM_CODE_PATCH_FUNC rcp_mws_init_func_1 = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_mws_init_func_2 = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_mws_16_bit_mode = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_mws_8_bit_mode = NULL;
PF_ROM_CODE_PATCH_VOID rcp_mws_baudrate_config = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_mws_frame_sync_protection_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_mws_write_bb_frame_sync_updt_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_mws_frame_sync_update_value_for_next_time_func = NULL;
#endif


#ifdef _DAPE_ENABLE_MWS_FRAME_SYNC
BT_MWS_MANAGER_S bt_mws_var;
#endif   
#ifdef _DAPE_TEST_FRAME_SYNC_UPDATE_CHOOSE_WAY
extern UINT8 g_modify_way;
#endif                    

#ifdef MWS_POLL_TIMEOUT
#define MWS_NUM_POLL_MAX 200

#define MWS_POLL_TIMEOUT_LOG() RT_BT_LOG(RED, LOG_MWS_POLL_TIMEOUT, 0, 0)

#define MWS_WAIT_FOR_READY_RET_FALSE() \
    do {\
        int n = 1;\
        while (!mws_is_ready())\
        {\
            if (n > MWS_NUM_POLL_MAX)\
            {\
                MWS_POLL_TIMEOUT_LOG();\
                break;\
            }\
            ++n;\
        }\
    } while (0)
#else
#define MWS_WAIT_FOR_READY_RET_FALSE() do {} while (!mws_is_ready())
#endif

void mws_reg_init()
{
#ifdef _ROM_CODE_PATCHED_
    if (rcp_mws_init_func_1 != NULL)
    {
        if ( rcp_mws_init_func_1(NULL) )
        {
            return;
        }
    }
#endif
    mws_write_register(UART_MISC_CTRL_REG,0x101,0xf);
#ifdef MWS_16_BIT_MODE
    mws_write_register(LTECOEX_CTRL_REG,0x5519,0xf);
#endif

#ifdef MWS_8_BIT_MODE
    mws_write_register(LTECOEX_CTRL_REG,0x8888,0xf);
#endif

#ifdef MWS_HUAWEI_3_WIRED_MODE
    mws_write_register(LTE_TX_SETUP_TIME_REG,1500,0x3); //1.5ms delay
    mws_write_register(LTE_RX_SETUP_TIME_REG,200,0x3);  //200us delay
    mws_write_register(LTE_HW_RT2LTE_EN_REG,0x3f,0xf); //bt/wl sw control WLAN_PRIORITY enable
#endif
    mws_baudrate_setup();
    
#ifdef	MWS_ENABLE
#ifdef _ACTIVE_CHECK_WIFI_ON_OFF_
		if(check_wifi_alive())
		{
			pf_os_trigger_mailbox_task(MAILBOX_WRITE,0x02063A, 0x0);
		}
		else
#endif            
		{
			mws_write_register(LTECOEX_CTRL_REG,mws_read_register_new(LTECOEX_CTRL_REG)|BIT7,0xf);
		}
#endif

    //clear the RT/NRT rx fifo,w 1 to clear the fifo , but not W1C...
    mws_write_register(UART_FIFO_CTRL_REG,0x3,0xf);
    mws_write_register(UART_FIFO_CTRL_REG,0x0,0xf);

    //enable RT CMD send to FW via RT rx fifo
    mws_write_register(FW_RX_RT_CMD_EN_REG,0x1,0xf);

    //enable WL TRX and BT TRX
    //mws_write_register(LTE_HW_RT2LTE_EN_REG,0xf,0xf);

    //pattern initialize
    mws_write_register(LTECOEX_PATTERN0_REG,0x02409,0xf); //DSUUUDSUUU , original pattern , but need inverse when writing into register
    mws_write_register(LTECOEX_PATTERN1_REG,0x42509,0xf); //DSUUDDSUUD
    mws_write_register(LTECOEX_PATTERN2_REG,0x52549,0xf); //DSUDDDSUDD
    mws_write_register(LTECOEX_PATTERN3_REG,0x55409,0xf); //DSUUUDDDDD
    mws_write_register(LTECOEX_PATTERN4_REG,0x55509,0xf); //DSUUDDDDDD
    mws_write_register(LTECOEX_PATTERN5_REG,0x55549,0xf); //DSUDDDDDDD
    mws_write_register(LTECOEX_PATTERN6_REG,0x42409,0xf); //DSUUUDSUUD

    mws_current_interrupt_enable = mws_read_register_new(INTERRUPT_EN_REG);
    mws_write_register(UART_MISC_CTRL_REG,0x0,0xf);
    //mws_write_register(INTERRUPT_EN_REG,0xffff,0xf);
#ifdef _DAPE_TEST_CHK_MWS_FRAME_SYNC  
    mws_write_register(INTERRUPT_EN_REG,BIT11,0xf);
    //mws_write_register(INTERRUPT_EN_REG, 0xFFFF,0xf);

    UINT16 reg26e = BB_read_baseband_register(MWS_CLKADJ_CTRL1);
    BB_write_baseband_register(MWS_CLKADJ_CTRL1, (reg26e & (UINT16)(0xCFFF)));
    RT_BT_LOG(WHITE, DAPE_TEST_LOG213, 2, mws_read_register_new(INTERRUPT_EN_REG), BB_read_baseband_register(MWS_CLKADJ_CTRL1));
#endif    

#ifdef _ROM_CODE_PATCHED_
    if (rcp_mws_init_func_2 != NULL)
    {
        if ( rcp_mws_init_func_2(NULL) )
        {
            return;
        }
    }
#endif	
}

// flow chart 1(write)
void mws_init()
{    
    mws_sw_init();
    mws_reg_init();
}

void mws_sw_init()
{
#ifdef _DAPE_ENABLE_MWS_FRAME_SYNC
    mws_frame_sync_manager_init();
#endif    
}


BOOLEAN mws_uart_nrt_tx_write_reg(UINT32 data,UINT16 num_byte)
{
    UINT32 read_reg;
    read_reg = mws_read_register_new(UART_BT_TX_HOLD_REG);
    if( (read_reg&BIT30) )
    {
        if(!mws_write_register(UART_BT_TX_HOLD_REG, (data|BIT31),0xf))
        {
            RT_BT_LOG(RED,MWS_READ_REG_FAIL,0,0);
            return  MWS_W_R_FAIL;
        }
    }
    return TRUE;     
}
/*BOOLEAN mws_read_register(UINT16 addr , UINT32 *data)
{
    BT2LTECOEX_INDIRECT_ACCESS_REG0 reg;
    MWS_WAIT_FOR_READY_RET_FALSE();
    reg.ltecoex_write_mode = READ_OPERATION;
    reg.ltecoex_access_start = 1;
    reg.ltecoex_reg_addr = addr;
    bt_lte_coex_indirect_access_reg(reg); 
    MWS_WAIT_FOR_READY_RET_FALSE();      
    *data = bt_lte_coex_indirect_access_reg_read();
    return MWS_W_R_SUCCESS;
}*/

UINT32 mws_read_register_new(UINT16 addr)
{
    BT2LTECOEX_INDIRECT_ACCESS_REG0 reg;
    MWS_WAIT_FOR_READY_RET_FALSE();
    reg.ltecoex_write_mode = READ_OPERATION;
    reg.ltecoex_access_start = 1;
    reg.ltecoex_reg_addr = addr;
    bt_lte_coex_indirect_access_reg(reg); 
    MWS_WAIT_FOR_READY_RET_FALSE();      
    return bt_lte_coex_indirect_access_reg_read();
}


BOOLEAN mws_write_register(UINT16 addr,UINT32 data,UINT16 num_byte)
{
    BT2LTECOEX_INDIRECT_ACCESS_REG0 reg;
    MWS_WAIT_FOR_READY_RET_FALSE();
    reg.ltecoex_write_mode = WRITE_OPERATION;
    reg.ltecoex_access_start = 1;
    reg.ltecoex_reg_addr = addr;
    reg.write_byte_enable = (BIT0|BIT1|BIT2|BIT3);
    bt_lte_coex_indirect_access_reg_write(data);
    bt_lte_coex_indirect_access_reg(reg);
    MWS_WAIT_FOR_READY_RET_FALSE();
    return MWS_W_R_SUCCESS;
}

#ifdef MWS_16_BIT_MODE
void mws_to_bt_message_16bit(UCHAR is_rt_msg)
{
    UINT32 reg;
#ifdef _ROM_CODE_PATCHED_
		if (rcp_mws_16_bit_mode != NULL)
		{
			if ( rcp_mws_16_bit_mode((void *)&is_rt_msg) )
			{
				return;
			}
		}
#endif

    if( is_rt_msg == 1)
    {
        /*if(!mws_read_register(UART_RT_RX_BUF_REG,&reg))
        {
            RT_BT_LOG(WHITE,MWS_READ_REG_FAIL,0,0);
        }*/
        reg = mws_read_register_new(UART_RT_RX_BUF_REG);
    }
    else
    {
        /*if(!mws_read_register(UART_NRT_RX_BUF_REG,&reg))
        {
            RT_BT_LOG(WHITE,MWS_READ_REG_FAIL,0,0);
        }*/
        reg = mws_read_register_new(UART_NRT_RX_BUF_REG);
    }

    UINT16 type = reg&0x3;;
    UINT16 data = (reg>>3)&0x7ff;
    
    if(reg == 0x0)
    {
        if(mws_read_register_new(LTECOEX_CTRL_REG)&BIT4)  //slot mode , waveform is all 0
        {
            g_mws_rx = 0;
            g_mws_tx = 0;
        } 
        else        //pattern mode , the all 0 waveform means pattern index is 0
        {
            g_mws_pattern_index = 0;
        }
    }
    else if(reg == 0xff)
    {
        RT_BT_LOG(RED,MWS_FETCH_RX_ERROR,0,0);
        return ;
    }
    switch(type)
    {
        case RT_SIGNAL_MESSAGE:  
            {
                if(data&BIT0) //FRAME_SYNC
                {
                    RT_BT_LOG(BLUE,MWS_UART_TRX_SUCCESS,0,0);
                }
                if(data&BIT1) //MW_RX
                {
                    if( g_mws_rx == 0 ) //MWS_rx start
                    {
                        g_mws_rx = 1;
                    }
                    RT_BT_LOG(BLUE,MWS_UART_TRX_SUCCESS,0,0);
                }
                if(data&BIT2) //MWS_TX
                {
                    if( g_mws_tx == 0 ) //MWS_tx start
                    {
                        g_mws_tx = 1 ;
                    }
                    RT_BT_LOG(BLUE,MWS_UART_TRX_SUCCESS,0,0);
                }
                if(data&(BIT3|BIT4|BIT5)) // MWS_PATTERNS
                {
                    UINT16 mws_pattern_index;
                    mws_pattern_index = (data>>3)&0xf;  
                    if( mws_pattern_index !=0x7 )
                    {
                        g_mws_pattern_index = mws_pattern_index;
                    }
                    RT_BT_LOG(BLUE,MWS_UART_TRX_SUCCESS,0,0);
                }
                if(data&BIT6) // IGNORE_MWS_TRX
                {
                    RT_BT_LOG(BLUE,MWS_UART_TRX_SUCCESS,0,0);
                }
            }
            break;
            
        case UPDATE_OF_RT_SIGNAL:
            {
                if(data&BIT0)
                {
                    RT_BT_LOG(BLUE,MWS_UART_TRX_SUCCESS,0,0);
                }
                else
                {
                    RT_BT_LOG(RED,MWS_ERROR,0,0);
                }
            }
            
            break;
            
        case NRT_SIGNAL_MESSAGE:
            {
                UINT16 mws_retry_count;         //MSG[3:0]
                UINT16 lte_band;                //MSG[9:4]
                UINT16 mws_intf_band_used;      //MSG[10]
                mws_retry_count = data&0xf;
                lte_band = data>>4&0x3f;
                mws_intf_band_used = data>>10;
                RT_BT_LOG(BLUE,MWS_UART_TRX_SUCCESS,0,0);
            }
            break;
            
        case MWS_INACT_DURATION_MESSAGE:
            {
                UINT16 mws_inact_duration;
                if(data != 0) 
                {
                    mws_inact_duration = data * 5 ; //Inactivity_time = MWS_Inactivity Duration * 5 mS
                    RT_BT_LOG(BLUE,MWS_UART_TRX_SUCCESS,0,0);
                }
                else
                {
                    //inact duration end , notify hw to open LTECOEX
                }
            }
            break;
        case MWS_SCAN_MESSAGE:
            {
                RT_BT_LOG(BLUE,MWS_UART_TRX_SUCCESS,0,0);
            }
            break;
        case MWS_FRAME_SETUP_TIME:
            {
                UINT8 setuptime_select;
                UINT8 mws_setuptime;
                setuptime_select = data>>8;
                mws_setuptime = data&0xff;
                switch(setuptime_select)
                {
                    case 0x0: //frame sync
                    {
                        mws_write_register(LTE_FRAMESYNC_SETUP_REG,mws_setuptime,0xf);
                        RT_BT_LOG(BLUE,MWS_UART_TRX_SUCCESS,0,0);
                    }
                    break;
                    
                    case 0x1: //TX setup time
                    {
                        mws_write_register(LTE_TX_SETUP_TIME_REG,mws_setuptime,0xf);
                        RT_BT_LOG(BLUE,MWS_UART_TRX_SUCCESS,0,0);
                    }
                    break;

                    case 0x2: //RX setup time
                    {
                        mws_write_register(LTE_RX_SETUP_TIME_REG,mws_setuptime,0xf);
                        RT_BT_LOG(BLUE,MWS_UART_TRX_SUCCESS,0,0);
                    }
                    break;

                    case 0x3: //pattern setup time
                    {
                        mws_write_register(LTE_PATTERN_SETUP_TIME_REG,mws_setuptime,0xf);
                        RT_BT_LOG(BLUE,MWS_UART_TRX_SUCCESS,0,0);
                    }
                    break;

                    case 0x4: //LTE inacitivity duration setup time
                    {
                        RT_BT_LOG(BLUE,MWS_UART_TRX_SUCCESS,0,0);
                    }
                    break;

                    case 0x5: //LTE scan setup time
                    {
                        RT_BT_LOG(BLUE,MWS_UART_TRX_SUCCESS,0,0);
                    }
                    break;
                }
            }
            break;
        case WLAN_BEACON_AND_URGENT:
            {
                
            }    
            break;
        case BT_HIGH_PRI_RX_AND_URGENT:
            {
                
            }
            break;
            
        default:
            break;
    }
    reg=0xffff;
}
#endif

#ifdef MWS_8_BIT_MODE
UCHAR mws_to_bt_message_8bit(UCHAR is_rt_msg)
{
#ifdef _ROM_CODE_PATCHED_
		if (rcp_mws_8_bit_mode != NULL)
		{
			if ( rcp_mws_8_bit_mode(is_rt_msg) )
			{
				return;
			}
		}
#endif
    UART_MSG_REG_8bit reg;
    reg = mws_read_register_new(UART_RT_RX_BUF_REG);
    //switch(reg.type)
    //{
//TODO
    //}
    
}
#endif

void mws_baudrate_setup(void)
{
#ifdef _ROM_CODE_PATCHED_
	if (rcp_mws_baudrate_config != NULL)
	{
		rcp_mws_baudrate_config();
	}
#endif
}
#ifdef MWS_ENABLE
void mws_hand_shake_timer(TimerHandle_t timer_handle)
{
	RT_BT_LOG(RED,MWS_HAND_SHAKE_DONE1,0,0)	;
}
#endif

#ifdef _DAPE_ENABLE_MWS_FRAME_SYNC
void mws_set_frame_sync_update_way(UINT8 modify_way)
{
    bt_mws_var.modify_way = modify_way;
    bt_mws_var.us_increasing = 0;
    bt_mws_var.us_decreasing = 0;  
}
void mws_handle_lte_frame_sync_isr(void)
{
    bt_mws_var.latched_clk = BB_read_baseband_register(MWS_CLKADJ_RPT0)|
        (BB_read_baseband_register(MWS_CLKADJ_RPT1)<< 16);
    bt_mws_var.latched_clk_us = BB_read_baseband_register(MWS_CLKADJ_RPT2);
    if (bt_mws_var.init)
    {
        switch (bt_mws_var.frame_sync_case)
        {
            case MWS_FRAME_SYNC_DISABLE:
                break;
            case MWS_FRAME_SYNC_SELF_UPDT:
            {
                BZ_REG_S_MWS_CLKADJ_CTRL1 reg_26e;
                BZ_REG_S_MWS_CLKADJ_CTRL4 reg_274;
                UINT8 write_clk1_0 = 0;
                UINT8 latched_clk1_0;
                UINT8 last_clk1_0;
                UINT16 write_clk_cnt = 0;
                UINT8 cant_update = 0;
                UINT8 need_to_modify = 0;
                UINT8 write_case = 0;
                UINT32 cur_clk;
                UINT32 cur_us;

                mws_get_frame_sync_clk_info(&cur_clk, &cur_us, &latched_clk1_0, &last_clk1_0);
                
#ifdef _DAPE_TEST_FRAME_SYNC_UPDATE_CHOOSE_WAY
                if (g_modify_way)

#else
                if (lmp_self_device_data.number_of_hlc == 0)
#endif                    
                {
                    bt_mws_var.total_offset_us_write = bt_mws_var.total_offset_us_target;
                    mws_set_frame_sync_update_way(MWS_FRAME_SYNC_IDLE);
                    write_case = 1;
                }
                else                    
                {                         
                    if (bt_mws_var.total_offset_us_latched != bt_mws_var.total_offset_us_target)
                    {
                        mws_check_if_frame_sync_need_modify(&need_to_modify);
                        if (need_to_modify)
                        {
                            mws_decide_modify_way();
                            mws_decide_write_value_when_need_to_modify(&cant_update, &write_case);
                        } 
                        else
                        {
                            bt_mws_var.total_offset_us_write = bt_mws_var.total_offset_us_target;
                            write_case = 7;
                        } /* else of if (need_to_modify)*/
                    }
                    else
                    {
                        mws_decide_write_value_when_target_met(&cant_update, &write_case);
                    }                    
                }/* else of if (lmp_self_device_data.number_of_hlc == 0)*/                    

                mws_decide_write_value_protection(&cant_update, &write_case);

                bt_mws_var.total_offset_us_write = (bt_mws_var.total_offset_us_write % 1250);

                /* process write_clk1_0*/
                mws_calculate_clk_and_us(bt_mws_var.total_offset_us_write, &write_clk1_0, &write_clk_cnt);

                mws_write_bb_frame_sync_updt(cant_update, &reg_26e, &reg_274, write_clk1_0, write_clk_cnt);
                if ( bt_mws_var.log_en)
                {
                    RT_BT_LOG(BLUE, DAPE_TEST_LOG579, 19,
                    cur_clk,
                    bt_mws_var.latched_clk, bt_mws_var.latched_clk_us, write_clk1_0, write_clk_cnt,
                    bt_mws_var.total_offset_us_last_time, 
                    bt_mws_var.total_offset_us_latched, 
                    bt_mws_var.total_offset_us_current,
                    bt_mws_var.total_offset_us_write,
                    bt_mws_var.last_total_offset_us_write,
                    bt_mws_var.total_offset_us_target,
                    bt_mws_var.target_clk1_0, bt_mws_var.target_clk_us,
                    bt_mws_var.modify_way,*(UINT16*)&reg_274, need_to_modify, cant_update,
                    write_case);
                }
                mws_frame_sync_update_value_for_next_time(cant_update, latched_clk1_0);
            }
                break;
            default:
                break;
    
        }
    }
    else
    {
        bt_mws_var.init = 1;
    }
#ifndef _DAPE_GET_FRAME_UPDATED_CLK_FROM_HW    
    bt_mws_var.last_clk = bt_mws_var.latched_clk;
    bt_mws_var.last_clk_us = bt_mws_var.latched_clk_us;
#else
// bt_mws_var.last_clk is updated by HW in the next time.
    bt_mws_var.last_clk_us = bt_mws_var.latched_clk_us;
#endif
    bt_mws_var.last_modify_way = bt_mws_var.modify_way;
    bt_mws_var.frame_sync_detect_timer = 0;
}

void mws_disable_frame_sync_update(void)
{
    BZ_REG_S_MWS_CLKADJ_CTRL4 reg_274;

    *(UINT16*)&reg_274 = BB_read_baseband_register(MWS_CLKADJ_CTRL4);
    reg_274.enable_clk_sync = FALSE;
    BB_write_baseband_register(MWS_CLKADJ_CTRL4, *(UINT16*)&reg_274);
    bt_mws_var.last_total_offset_us_write = 0xFFFF;
    bt_mws_var.modify_way = MWS_FRAME_SYNC_IDLE;
}
void mws_frame_sync_manager_init(void)
{
    memset(&bt_mws_var, 0, sizeof(BT_MWS_MANAGER_S));    
    bt_mws_var.frame_sync_case = MWS_FRAME_SYNC_SELF_UPDT;
    bt_mws_var.wraparound_protect_value = 1000;  
    bt_mws_var.modification_threshold = 10;
    bt_mws_var.modify_us = 5;
    bt_mws_var.target_clk_us = 624;
    bt_mws_var.target_clk1_0 = 0;
    bt_mws_var.force_modify_value = 10;
    bt_mws_var.last_total_offset_us_write = 0xFFFF;
    bt_mws_var.frame_sync_disable_threshold = 500;

}
void mws_get_frame_sync_clk_info(UINT32 *cur_clk, UINT32 *cur_us,
    UINT8 *latched_clk1_0, UINT8 *last_clk1_0)
{
    lc_get_high_dpi_native_clock(cur_clk, cur_us);
#ifdef _DAPE_GET_FRAME_UPDATED_CLK_FROM_HW    
    bt_mws_var.last_clk = ((*cur_clk & 0xFFFFFFF0) | (BB_read_baseband_register(MWS_CLKADJ_RPT1)>>12));
#endif
    *latched_clk1_0 = (bt_mws_var.latched_clk & (BIT1|BIT0));
    *last_clk1_0 = (bt_mws_var.last_clk & (BIT1|BIT0));

    bt_mws_var.total_offset_us_current =  (((*cur_clk&(0x00000003)) >> 1) * 625) + *cur_us;
    bt_mws_var.total_offset_us_latched = ((*latched_clk1_0 >> 1) * 625) + (624 - bt_mws_var.latched_clk_us);
    bt_mws_var.total_offset_us_target = ((bt_mws_var.target_clk1_0 >> 1) * 625) + (624 - bt_mws_var.target_clk_us);
    bt_mws_var.total_offset_us_last_time = ((*last_clk1_0 >> 1) * 625) + (624 - bt_mws_var.last_clk_us);

}

void mws_check_if_frame_sync_need_modify(UINT8 *need_to_modify)
{
    UINT16 diff;
    UINT16 diff2;
    UINT16 total_offset_us_target2;

    if (bt_mws_var.total_offset_us_latched > bt_mws_var.total_offset_us_target)
    {
        diff = bt_mws_var.total_offset_us_latched - bt_mws_var.total_offset_us_target;
    }
    else
    {
        diff = bt_mws_var.total_offset_us_target - bt_mws_var.total_offset_us_latched;
    }
    total_offset_us_target2 = bt_mws_var.total_offset_us_target + 1250;
    diff2 = total_offset_us_target2 - bt_mws_var.total_offset_us_latched;
    if ((diff > bt_mws_var.force_modify_value) 
        && (diff2 > bt_mws_var.force_modify_value))
    {
        *need_to_modify = 1;
    }
    else
    {
        *need_to_modify = 0;
    }
}
void mws_decide_modify_way(void)
{
    if (bt_mws_var.modify_way == MWS_FRAME_SYNC_IDLE)
    {
        /* this case means "total us" is increasing. we should decrease it. */
        if ((bt_mws_var.total_offset_us_latched > bt_mws_var.total_offset_us_last_time) && 
              /* make sure the "bigger" case is not caused by wrap around. */
              ((bt_mws_var.total_offset_us_latched - bt_mws_var.total_offset_us_last_time) < bt_mws_var.wraparound_protect_value))
        {
            bt_mws_var.us_increasing ++;
        }
        else
        {
            bt_mws_var.us_decreasing ++;
        }
/*             RT_BT_LOG(WHITE, YL_DBG_DEC_6, 6, 
        bt_mws_var.modification_threshold,
        bt_mws_var.us_increasing, 
        bt_mws_var.us_decreasing, 
total_offset_us_last_time,
total_offset_us_current,total_offset_us_target);*/
        if (bt_mws_var.us_increasing > bt_mws_var.modification_threshold)
        {
            mws_set_frame_sync_update_way(MWS_FRAME_SYNC_DECREASING);   
        }                    
        else if (bt_mws_var.us_decreasing > bt_mws_var.modification_threshold)
        {
            mws_set_frame_sync_update_way(MWS_FRAME_SYNC_INCREASING);
        }
    }
}
void mws_decide_write_value_when_need_to_modify(UINT8 *cant_update, UINT8 *write_case)
{
    if ((bt_mws_var.last_total_offset_us_write != 0xFFFF) &&
        (bt_mws_var.modify_way == bt_mws_var.last_modify_way))
    {
        switch (bt_mws_var.modify_way)
        {
            case MWS_FRAME_SYNC_DECREASING:
                if (bt_mws_var.last_total_offset_us_write >= bt_mws_var.modify_us)
                {
                    bt_mws_var.total_offset_us_write = MAX(bt_mws_var.last_total_offset_us_write
                                  - bt_mws_var.modify_us, bt_mws_var.total_offset_us_target);
                    *cant_update = 0;
                    *write_case = 2;
                }
                else
                {
                    bt_mws_var.total_offset_us_write = bt_mws_var.total_offset_us_target;
                    *cant_update = 0;
                    *write_case = 3;
                }                               
                break;
            case MWS_FRAME_SYNC_INCREASING:
                bt_mws_var.total_offset_us_write = MIN(bt_mws_var.last_total_offset_us_write + bt_mws_var.modify_us, bt_mws_var.total_offset_us_target+1250);
                *cant_update = 0;
                *write_case = 4;
                break;
            default:
                bt_mws_var.total_offset_us_write = bt_mws_var.last_total_offset_us_write;
                *cant_update = 1;
                *write_case = 5;
                break;
    
        }
    }
    else
    {
        bt_mws_var.total_offset_us_write = bt_mws_var.total_offset_us_latched;
        *cant_update = 1;
        *write_case = 6;
    }
}
void mws_decide_write_value_when_target_met(UINT8 *cant_update, UINT8 *write_case)
{
    mws_set_frame_sync_update_way(MWS_FRAME_SYNC_IDLE);
    bt_mws_var.total_offset_us_write = bt_mws_var.total_offset_us_target;
    //bt_mws_var.last_total_offset_us_write = 0xFFFF;
    *write_case = 8;
    *cant_update = 0;
}

void mws_decide_write_value_protection(UINT8 *cant_update, UINT8 *write_case)
{
#ifdef _ROM_CODE_PATCHED_
    if (rcp_mws_frame_sync_protection_func != NULL)
    {
        if (rcp_mws_frame_sync_protection_func((void*)cant_update, write_case))
        {
            return;
        }
    }
#endif
    if (((bt_mws_var.total_offset_us_write > bt_mws_var.total_offset_us_latched) &&
        (bt_mws_var.total_offset_us_write - bt_mws_var.total_offset_us_latched) > 
                bt_mws_var.wraparound_protect_value))
    {
        bt_mws_var.total_offset_us_write = bt_mws_var.total_offset_us_latched;
        *write_case = 9;
        *cant_update = 0;
    }
#if 0                
    /* Protect for the case that the */
    if ((total_offset_us_write > total_offset_us_current) && 
        ((total_offset_us_write - total_offset_us_current)> bt_mws_var.force_modify_value))
    {
       cant_update = 1;
       write_case = 10;
    }
    if (total_offset_us_current > total_offset_us_write)
    {
        if ((total_offset_us_current - total_offset_us_write) <= 
                                       bt_mws_var.force_modify_value)
        {
            total_offset_us_write = total_offset_us_current;
            write_case = 11;
        }
        else
        {
            cant_update = 1;
            write_case = 12;
        }
    }
#endif                
    
}
void mws_calculate_clk_and_us(UINT16 total_offset_us_write, UINT8 *write_clk1_0, UINT16 *write_clk_cnt)
{
    total_offset_us_write = (total_offset_us_write % 1250);

    if (total_offset_us_write> 624)
    {
        *write_clk1_0 |= BIT1;
    }
    *write_clk_cnt = 624 - (total_offset_us_write % 625);
    if (*write_clk_cnt < 312)
    {
        *write_clk1_0 |= BIT0;
    }
}
void mws_write_bb_frame_sync_updt(UINT8 cant_update, 
                                BZ_REG_S_MWS_CLKADJ_CTRL1 *reg_26e, 
                                BZ_REG_S_MWS_CLKADJ_CTRL4 *reg_274,
                                UINT8 write_clk1_0, UINT16 write_clk_cnt)
{
    UCHAR piconet_id;
    piconet_id = lc_get_master_piconet_id();

    //BZ_REG_S_MWS_CLKADJ_CTRL1 reg_26e;
    //BZ_REG_S_MWS_CLKADJ_CTRL4 reg_274;

    *(UINT16*)reg_26e = BB_read_baseband_register(MWS_CLKADJ_CTRL1);
    *(UINT16*)reg_274 = BB_read_baseband_register(MWS_CLKADJ_CTRL4);

    UINT8 reg_pn3_info = 0;
    if (lmp_self_device_data.number_of_hlc == 0)
    {
        piconet_id = 3;
        reg_pn3_info = BB_read_baseband_register(reg_PICONET_INFO[piconet_id]);
        /* Set PN Info Reg[0] to master for HW to latch correct piconet clk. */
        OR_val_with_bb_reg(reg_PICONET_INFO[piconet_id], BIT0);
        
    }
    /*
    else
    {}*/

    if (cant_update == 0)
    {
        /* disable clk_adj */
        reg_26e->enable_clk_adj = FALSE;
        reg_26e->clk_adj_pn = piconet_id;
        BB_write_baseband_register(MWS_CLKADJ_CTRL1, *(UINT16*)reg_26e);

        /* enable frame sync clk update. */
        reg_274->enable_clk_sync = TRUE;
        reg_274->sync_n_clk1_0 = write_clk1_0;
        reg_274->sync_n_cnt9_0 = write_clk_cnt;
        BB_write_baseband_register(MWS_CLKADJ_CTRL4, *(UINT16*)reg_274);

    }
    else
    {
        reg_274->enable_clk_sync = FALSE;
        BB_write_baseband_register(MWS_CLKADJ_CTRL4, *(UINT16*)reg_274);
    }
#ifdef _ROM_CODE_PATCHED_
    if (rcp_mws_write_bb_frame_sync_updt_func != NULL)
    {
        rcp_mws_write_bb_frame_sync_updt_func((void*)reg_274, reg_26e,
            write_clk1_0, write_clk_cnt, cant_update);
    }
#endif

    
}
void mws_frame_sync_update_value_for_next_time(UINT8 cant_update, 
                                     UINT8 latched_clk1_0)
{
    if (cant_update == 0)
    {
        UINT8 write_clk1_0 = 0;
        UINT16 write_clk_cnt = 0;
        
        mws_calculate_clk_and_us(bt_mws_var.last_total_offset_us_write, 
            &write_clk1_0, &write_clk_cnt);

#ifndef _DAPE_GET_FRAME_UPDATED_CLK_FROM_HW
    
        UINT32 tmp_clk = bt_mws_var.latched_clk;
        
        bt_mws_var.latched_clk &= ((UINT32)(0xFFFFFFFC));

        

        bt_mws_var.latched_clk |= write_clk1_0;
        bt_mws_var.latched_clk_us = write_clk_cnt;
        if (latched_clk1_0 > write_clk1_0)
        {
            bt_mws_var.latched_clk += 0x00000004;
        }
#else
        bt_mws_var.latched_clk_us = write_clk_cnt;
#endif        
//RT_BT_LOG(YELLOW, DAPE_TEST_LOG525, 6, tmp_clk, bt_mws_var.latched_clk, write_clk1_0, bt_mws_var.latched_clk_us,0,0);
    }
    if (bt_mws_var.total_offset_us_write == bt_mws_var.total_offset_us_target)
    {
        /* this is used for the case that we don't have frame sync 
           for a long time. */
        //bt_mws_var.last_total_offset_us_write = 0xFFFF;
        mws_set_frame_sync_update_way(MWS_FRAME_SYNC_IDLE);   
    }                
    {
        bt_mws_var.last_total_offset_us_write = bt_mws_var.total_offset_us_write;
    }
#ifdef _ROM_CODE_PATCHED_
    if (rcp_mws_frame_sync_update_value_for_next_time_func != NULL)
    {
        rcp_mws_frame_sync_update_value_for_next_time_func((void*)&cant_update,
            latched_clk1_0);
    }
#endif
    
}
#endif

