/***************************************************************************
 Copyright (C) RealTek Ltd.
 This module is a confidential and proprietary property of RealTek and
 a possession or use of this module requires written permission of RealTek.
 ***************************************************************************/
enum { __FILE_NUM__= 123 };

#include "plc.h"
#include "bb_driver.h"
#include "common_utils.h"
#include "logger.h"
#include "lc_internal.h"
#include "lc.h"
#include "mem.h"
#include "hci_vendor_defines.h"

#ifdef _ROM_CODE_PATCHED_
PF_ROM_CODE_PATCH_FUNC rcp_plc_init_func = NULL;
#endif

#ifdef _PLC_TEST_MODE_ENABLE_
PLC_TEST_MANAGER plt_test_var;
#endif

void plc_init(void)
{

#ifdef _BRUCE_TEST_PLC_PKT_STATUS
    memset(&plc_pkt_status_var,0,sizeof(PLC_CHECK_PKT_STATUS));
#endif
#ifdef _PLC_TEST_MODE_ENABLE_
    plc_test_mode_init();
#endif 

    PLC_REG_CTRL plc_ctrl_reg;
    *(UINT16*)&plc_ctrl_reg = BB_read_baseband_register(PLC_CTRL_REGISTER);
    plc_ctrl_reg.opt_ex_fir = 0;
    plc_ctrl_reg.optin_at_cf_cr = 1;
    plc_ctrl_reg.opt_cf = 0;
    plc_ctrl_reg.opt_atc_sub_sel_mode = 0;
    plc_ctrl_reg.opt_atc_sel_mode = 0;
    BB_write_baseband_register(PLC_CTRL_REGISTER, *(UINT16*)&plc_ctrl_reg);  

    UINT16 read= BB_read_baseband_register(PLC_CTRL2_REGISTER);
    /*  0x254[2],
            0: normal mode        1: The length of ola depends on the length of the erasure
            0x254[3],
            0: when not found pitch, pitch equal to max pitch
            1: when not found pitch, pitch equal to min pitch*/
    read |= (BIT2|BIT3);
    BB_write_baseband_register(PLC_CTRL2_REGISTER, read); 
#if 0    
    UINT16 read2= BB_read_baseband_register(PLC_CTRL3_REGISTER);
    /*0x256[15:0]   pkt_mse_thrshd value. It isused to denote u(16,16f)   */    
    read2 &= 0xF5C;
    RT_BT_LOG(RED,YL_DBG_DEC_1,1,read2);
    BB_write_baseband_register(PLC_CTRL3_REGISTER, read2); 
#endif

#ifdef _BRUCE_FIX_MSBC_DECODER
    read= BB_read_baseband_register(BB_PCMOUT_SHIFT_REG);
    //0x25e[8]=auto payload header alignment , 0x25e[9]=fix msbc decoder bug
    read |= (BIT8|BIT9);
    BB_write_baseband_register(BB_PCMOUT_SHIFT_REG, read); 
#endif
    
#ifdef _ROM_CODE_PATCHED_
    if (rcp_plc_init_func != NULL)
    {
        rcp_plc_init_func(NULL);
    }
#endif
   return;
}

#ifdef _PLC_TEST_MODE_ENABLE_
const UINT16 plc_test_500hz_sine_16bit_sample[16] = {
    0x0000, 0x30FB, 0x5A81, 0x7640, 0x7FFF, 0x7640, 0x5A81, 0x30FB,
    0x0000, 0xCF05, 0xA57F, 0x89C0, 0x8001, 0x89C0, 0xA57F, 0xCF05
};

const UINT16 plc_test_1khz_sine_16bit_sample[8] = {
    0x0000, 0x5A81, 0x7FFF, 0x5A81, 0x0000, 0xA57F, 0x8001, 0xA57F    
};

const UINT16 plc_test_2khz_sine_16bit_sample[4] = {
    0x0000, 0x7FFF, 0x0000, 0x8001
};

#define PLC_TEST_ESCO_INTERVAL                  16
#define PLC_TEST_ESCO_PKT_TYPE                  BB_2_EV3
#define PLC_TEST_ESCO_RX_PKT_LEN                160
#define PLC_TEST_ENQ_CLK_START_OFFSET           4
#define PLC_TEST_DEQ_CLK_START_OFFSET           8

UINT16 plc_test_in_buffer[2][PLC_TEST_ESCO_RX_PKT_LEN/2];
UINT8  plc_test_in_err_flag_buffer[2];
UINT16 plc_test_out_buffer[PLC_TEST_ESCO_RX_PKT_LEN/2];

void plc_test_mode_init(void)
{
    UINT16 regValue;
    PLC_REG_CTRL plc_ctrl_reg;
    UINT16 esco_rx_len_reg;
    UINT16 read2;
    UINT16 bt_clk_ctrl=0;    
    //bruce setting psd_pcm_pkt_dur 
    plt_test_var.psd_pcm_pkt_dur_counter = 0;
    plt_test_var.erasecnt = 0;
    plt_test_var.CMD_Flow_count_new=0;
    plt_test_var.CMD_Flow_count_old=0;
    plt_test_var.tx_seqn=0; 

    plt_test_var.bb_clk_value = 0;
    plt_test_var.esco_interval = PLC_TEST_ESCO_INTERVAL;
    plt_test_var.esco_pkt_type = PLC_TEST_ESCO_PKT_TYPE;
    plt_test_var.esco_rx_pkt_len = PLC_TEST_ESCO_RX_PKT_LEN;
    plt_test_var.enq_clk_start_offset = PLC_TEST_ENQ_CLK_START_OFFSET;
    plt_test_var.enq_clk_cnt = 0;
    plt_test_var.is_enq_err = 0;
    plt_test_var.deq_clk_start_offset = PLC_TEST_DEQ_CLK_START_OFFSET;    
    plt_test_var.deq_clk_cnt = 0;
    plt_test_var.in_buf_src = (UINT16*)plc_test_500hz_sine_16bit_sample;
    plt_test_var.in_buf_src_offset = 0;
    plt_test_var.in_buf_src_wlen = 16;
    plt_test_var.out_buf_offset = 0;
    plt_test_var.out_buf_wlen = PLC_TEST_ESCO_RX_PKT_LEN/2; /* word length */

    plt_test_var.in_buf = &plc_test_in_buffer[0][0];
    plt_test_var.out_buf = plc_test_out_buffer;

    BB_write_baseband_register(CONNECTOR_REGISTER,
                                (UINT16)( (1 << 5) | (0 << 11) ) );

    BB_write_baseband_register(reg_MASTER_LOWER_LUT[1], 
                                LC_MASTER_DEFAULT_PACKET_TYPE);

    regValue = BB_read_baseband_register(reg_MASTER_UPPER_LUT[1]);       
    /* Initialize active bit */
    regValue |= ACTIVE_BIT;        
    BB_write_baseband_register(reg_MASTER_UPPER_LUT[1], regValue);

    BB_write_baseband_register(PICONET1_INFO_REGISTER, 0x03);
    
    /* set ext_codec_en, pkt_mute_en, codec_plc_en */
    regValue = BB_read_baseband_register(VOICE_SETTING_REGISTER);
    regValue |= BIT0 | BIT2 | BIT11;
    BB_write_baseband_register(VOICE_SETTING_REGISTER, regValue);

    /* set link_over_hci */
    BB_write_baseband_register(TEST_CONTROL_REGISTER, 0x07);

    OR_val_with_bb_reg(ENCRYPTION_ENABLE_REGISTER, BIT9);
    
    read2 = BB_read_baseband_register(ENCRYPTION_ENABLE_REGISTER); 
    //RT_BT_LOG(GREEN,BRUCE_TEST_LOG,1,read2); 

    /* set codec type -- transparent */
    BB_write_baseband_register(BB_CODEC_CODE_TABLE_REG, 0);   
    
    BB_write_baseband_register(RADIO_SELECT_REGISTER, 0x0107);

    /* set esco 1, am_addr 1, make bito */    
    BB_write_baseband_register(CONNECTOR_REGISTER, 0x0631);    
  
    /* set esco interval = 12 slots */
    BB_write_baseband_register(ESCO_INTERVAL_REGISTER, 
                                plt_test_var.esco_interval << 8);

    esco_rx_len_reg = (UINT16) ((plt_test_var.esco_pkt_type << 12) |
                                        plt_test_var.esco_rx_pkt_len);
    BB_write_baseband_register(ESCO_RX_LENGTH_TYPE_REGISTER,  esco_rx_len_reg);

    /* Programming the Desco register. */
    BB_write_baseband_register(ESCO_WINDOW_DESCO_REGISTER, 0x0800);

    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_EXECUTE);

    /* enable BZDMA HW and loopback mode */
    WR_U32_BZDMA_REG(BZDMA_REG_CTRL, 0x03);
   
    *(UINT16*)&plc_ctrl_reg = BB_read_baseband_register(PLC_CTRL_REGISTER);
    plc_ctrl_reg.plc_test_mode = 1;
    plc_ctrl_reg.psd_pcm_pkt_dur = 1;
    plc_ctrl_reg.psd_sco_slot = 0;
    plc_ctrl_reg.opt_lnr_dt = 1;
    plc_ctrl_reg.bluetooth_clock = 0;
    BB_write_baseband_register(PLC_CTRL_REGISTER, *(UINT16*)&plc_ctrl_reg);
}

void plc_test_emu_flush_and_push_sco_rx_fifo(UINT16 *pbuf, UINT16 word_len, UINT8 err)
{
    UINT8 i;
    PLC_REG_CTRL plc_ctrl_reg;
    *(UINT16*)&plc_ctrl_reg = BB_read_baseband_register(PLC_CTRL_REGISTER);

    /* set psd_sco_slot to high */
    plc_ctrl_reg.psd_sco_slot = 1;
    BB_write_baseband_register(PLC_CTRL_REGISTER, *(UINT16*)&plc_ctrl_reg); 

    lc_flush_esco_rx_fifo();
    
    for (i = 0; i < word_len; i++)
    {
        BB_write_baseband_register(BB_LB_SCO0_RX_FIFO_WR, pbuf[i]);
    }

    /* set psd_pcm_pkt_dur to low or high (based on simulate condition) */
    plc_ctrl_reg.psd_pcm_pkt_dur = err ? 0 : 1;
    BB_write_baseband_register(PLC_CTRL_REGISTER, *(UINT16*)&plc_ctrl_reg); 

    /* set psd_sco_slot to low */
    plc_ctrl_reg.psd_sco_slot = 0;
    BB_write_baseband_register(PLC_CTRL_REGISTER, *(UINT16*)&plc_ctrl_reg); 
    //RT_BT_LOG(BLUE, BRUCE_TEST_LOG_d,1,plc_ctrl_reg.psd_pcm_pkt_dur); 

}

void plc_test_emu_pop_codec_data(UINT16 *pbuf, UINT16 word_len)
{
    UINT8 i;
    for (i = 0; i < word_len; i++)
    {
        pbuf[i] = BB_read_baseband_register(PLC_DATA_REGISTER);
    }
}
#ifdef _PLC_TEST_USED_VENDOR_COMMAND_
UCHAR hci_vendor_handle_send_test_data_cmd(HCI_CMD_PKT *hci_cmd_ptr)
{
    UINT8 seqn;
    UINT8 err_flag;
    UINT8 len;
    
    if (hci_cmd_ptr->param_total_length <= 2)
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    len = hci_cmd_ptr->param_total_length - 2;

    if (len != plt_test_var.esco_rx_pkt_len)
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;        
    }
    
    seqn = hci_cmd_ptr->cmd_parameter[0];
    err_flag = hci_cmd_ptr->cmd_parameter[1];

    /* update error status */
    plc_test_in_err_flag_buffer[plt_test_var.wptr] = err_flag;
    
    /* fill test data */
    memcpy((UINT8*)&plc_test_in_buffer[plt_test_var.wptr][0], 
                    &hci_cmd_ptr->cmd_parameter[2], len);

    /* update sequence number */
    plt_test_var.rx_seqn = seqn;

    /* inverse write pointer (ping-pong fifo) */
    plt_test_var.wptr =  !plt_test_var.wptr;
    
    plt_test_var.CMD_Flow_count_new++;
    return HCI_COMMAND_SUCCEEDED;
}

HCI_EVENT_PKT *hci_generate_test_data_event(void)
{
    HCI_EVENT_PKT *hci_event_pkt_ptr ;
    OS_SIGNAL signal_send;
    UINT8 len = plt_test_var.out_buf_wlen << 1;
    
    if (plt_test_var.tx_seqn & BIT7)
    {
        plt_test_var.CMD_Flow_count_old=0;
        plt_test_var.CMD_Flow_count_new=0;
        return NULL;
    }        
	
    if(OS_ALLOC_BUFFER(tx_table[HCI_EVENT_HANDLER_TASK].pool_handle,
                       (OS_ADDRESS *)&hci_event_pkt_ptr) != BT_ERROR_OK)
    {
        BT_FW_HCI_ERR(UNABLE_TO_ALLOCATE_MEMORY_FROM_EVENT_BUFFER_POOL,0,0);
        return NULL;
    }

    if (plt_test_var.tx_seqn == (plt_test_var.rx_seqn & 0x7f))
    {
        if (plt_test_var.rx_seqn & BIT7)
        {
            plt_test_var.tx_seqn |= BIT7;
            RT_BT_LOG(GREEN, BRUCE_TEST_LOG_7003,0,0);
            RT_BT_LOG(GREEN, BRUCE_TEST_LOG,1,plt_test_var.tx_seqn);
        }        
    }

    hci_event_pkt_ptr->event_opcode = HCI_VENDOR_SPECIFIC_EVENT ;        
    hci_event_pkt_ptr->event_parameter[0] = 0x24;
    hci_event_pkt_ptr->event_parameter[1] = 0x00;
    hci_event_pkt_ptr->event_parameter[2] = plt_test_var.tx_seqn;
    hci_event_pkt_ptr->event_parameter[3] = 0x00; /* reserved field */
    memcpy(&hci_event_pkt_ptr->event_parameter[4], 
                                        (UINT8*)plt_test_var.out_buf, len);

    RT_BT_LOG(BLUE, MSG_PLC_PCM_OUTPUT_DATA, 22,
            plt_test_var.bb_clk_value, 1,
            plt_test_var.out_buf[0], plt_test_var.out_buf[1], plt_test_var.out_buf[2], plt_test_var.out_buf[3], plt_test_var.out_buf[4],  
            plt_test_var.out_buf[5], plt_test_var.out_buf[6], plt_test_var.out_buf[7], plt_test_var.out_buf[8], plt_test_var.out_buf[9], 
            plt_test_var.out_buf[10], plt_test_var.out_buf[11], plt_test_var.out_buf[12], plt_test_var.out_buf[13], plt_test_var.out_buf[14], 
            plt_test_var.out_buf[15], plt_test_var.out_buf[16], plt_test_var.out_buf[17], plt_test_var.out_buf[18], plt_test_var.out_buf[19]);    
            
    hci_event_pkt_ptr->param_total_length = 4 + len;
    if (!(plt_test_var.tx_seqn & BIT7))
    {
        plt_test_var.tx_seqn=(plt_test_var.tx_seqn+1)& 0x7f;
    }

    RT_BT_LOG(RED, BRUCE_TEST_LOG,1,plt_test_var.rx_seqn);
    RT_BT_LOG(RED, BRUCE_TEST_LOG,1,plt_test_var.tx_seqn);
    /* Signal to event task for delivering the event */
    signal_send.type = HCI_DELIVER_HCI_EVENT_SIGNAL ;
    signal_send.param =(OS_ADDRESS) hci_event_pkt_ptr ;

    if(OS_SEND_SIGNAL_TO_TASK(hci_event_task_handle, signal_send) != BT_ERROR_OK)
    {
        BT_FW_HCI_ERR(LOG_LEVEL_HIGH, OS_SEND_SIGNAL_TO_TASK_FAILED,0,0);
    }
    return hci_event_pkt_ptr ;
}
#endif /* end of _PLC_TEST_USED_VENDOR_COMMAND_ */

void plc_test_emu_toggle_bluetooth_clock(void)
{    
    PLC_REG_CTRL plc_ctrl_reg;
#ifndef _PLC_TEST_USED_VENDOR_COMMAND_
    UINT8 i;
#endif

    plt_test_var.bb_clk_value++;
    if (plt_test_var.bb_clk_value & BIT27)
    {
        plt_test_var.bb_clk_value = 0;
    }
    if (plt_test_var.bb_clk_value & BIT0)
    {
        return;
    }
        
    /* toggle bb clock polarity */
    *(UINT16*)&plc_ctrl_reg = BB_read_baseband_register(PLC_CTRL_REGISTER);    
    if (plt_test_var.bb_clk_value & BIT1)
    {
        plc_ctrl_reg.bluetooth_clock = 1;
    }
    else
    {
        plc_ctrl_reg.bluetooth_clock = 0;        
    }
    BB_write_baseband_register(PLC_CTRL_REGISTER, *(UINT16*)&plc_ctrl_reg);        

    /* start to push data into sco rx fifo */
    if (plt_test_var.bb_clk_value >= (UINT32)plt_test_var.enq_clk_start_offset)
    {
        if (!(plt_test_var.bb_clk_value & BIT1))
        {
            /* handle in low level */        
            plt_test_var.enq_clk_cnt++;
            if (plt_test_var.enq_clk_cnt >= (plt_test_var.esco_interval >> 1))
            {    
                plt_test_var.enq_clk_cnt = 0;
#ifndef _PLC_TEST_USED_VENDOR_COMMAND_
                for (i = 0; i < (PLC_TEST_ESCO_RX_PKT_LEN >> 1); i++)
                {
                    plt_test_var.in_buf[i] = plt_test_var.in_buf_src[plt_test_var.in_buf_src_offset];
                    plt_test_var.in_buf_src_offset++;
                    if (plt_test_var.in_buf_src_offset >= plt_test_var.in_buf_src_wlen)
                    {
                        plt_test_var.in_buf_src_offset = 0;
                    }
                }                
                //bruce setting psd_pcm_pkt_dur
                plt_test_var.psd_pcm_pkt_dur_counter++;
                if (plt_test_var.psd_pcm_pkt_dur_counter%11==0)
                {
                    plc_test_emu_flush_and_push_sco_rx_fifo(plt_test_var.in_buf, 
                                        (PLC_TEST_ESCO_RX_PKT_LEN >> 1), 1);
                            plt_test_var.erasecnt++;
                            if(plt_test_var.erasecnt==2)
			{
			        plt_test_var.psd_pcm_pkt_dur_counter=0;                        
                	plt_test_var.erasecnt=0;
			    }
			    else
			    { 
			        plt_test_var.psd_pcm_pkt_dur_counter=10;                            		
			    }                        
                }
		    else
                {
                    plc_test_emu_flush_and_push_sco_rx_fifo(plt_test_var.in_buf, 
                                        (PLC_TEST_ESCO_RX_PKT_LEN >> 1), 0);
                }   				
#else
                plc_test_emu_flush_and_push_sco_rx_fifo(
                            &plc_test_in_buffer[plt_test_var.rptr][0],
                            (plt_test_var.esco_rx_pkt_len >> 1),
                            plc_test_in_err_flag_buffer[plt_test_var.rptr]);

                /* inverse the read pointer */
                plt_test_var.rptr = !plt_test_var.rptr;                
#endif
            }
        }           
    }

    /* start to pop data from plc module */
    if (plt_test_var.bb_clk_value >= (UINT32)plt_test_var.deq_clk_start_offset)
    {
        if (!(plt_test_var.bb_clk_value & BIT1))
        {
            /* handle in low level */
            UINT16 *obuf;
            obuf =  plt_test_var.out_buf + plt_test_var.out_buf_offset;  
                
            /* pop 10 samples every simulated 1.25 ms from hw buffer(1.25 ms / 0.125 ms = 10 samples) */
            if ((plt_test_var.out_buf_offset + 10) > plt_test_var.out_buf_wlen)
            {
                /* exception case !! */                
                UINT8 diff;
                diff =  plt_test_var.out_buf_wlen - plt_test_var.out_buf_offset;
                plc_test_emu_pop_codec_data(obuf, diff);     
                plt_test_var.out_buf_offset = plt_test_var.out_buf_wlen;
            }
            else
            {
                plc_test_emu_pop_codec_data(obuf, 10); 
                plt_test_var.out_buf_offset += 10;
            }

            plt_test_var.deq_clk_cnt++;
            if (plt_test_var.deq_clk_cnt >= (plt_test_var.esco_interval >> 1))
            {                
                plt_test_var.deq_clk_cnt = 0;
                plt_test_var.out_buf_offset = 0;

#ifndef _PLC_TEST_USED_VENDOR_COMMAND_
                UINT8 i;
                for (i = 0; i < 4; i++)
                {
                    obuf = plt_test_var.out_buf + (20 * i);
                    RT_BT_LOG(BLUE, MSG_PLC_PCM_OUTPUT_DATA, 22,
                            plt_test_var.bb_clk_value, i,
                            obuf[0], obuf[1], obuf[2], obuf[3], obuf[4],  
                            obuf[5], obuf[6], obuf[7], obuf[8], obuf[9], 
                            obuf[10], obuf[11], obuf[12], obuf[13], obuf[14], 
                            obuf[15], obuf[16], obuf[17], obuf[18], obuf[19]);    
                }
#else
                RT_BT_LOG(BLUE, YL_DBG_DEC_2,2,plt_test_var.CMD_Flow_count_new,				                                   plt_test_var.CMD_Flow_count_old)
                if ((plt_test_var.CMD_Flow_count_new>1) & (plt_test_var.CMD_Flow_count_new>plt_test_var.CMD_Flow_count_old))
                { 							
                    RT_BT_LOG(YELLOW, YL_DBG_DEC_1,1,8888);
                    plt_test_var.CMD_Flow_count_old=plt_test_var.CMD_Flow_count_new;		
                hci_generate_test_data_event();
		}
#endif
            }
        }           
    }    
}
#endif /* end of _PLC_TEST_MODE_ENABLE_ */

#ifdef _BRUCE_TEST_PLC_PKT_STATUS

void plc_print_esco_pkt_status(void)
{
    PLC_CHECK_PKT_STATUS temp;  
    DEF_CRITICAL_SECTION_STORAGE;
    MINT_OS_ENTER_CRITICAL();
    memcpy(&temp,&plc_pkt_status_var,8);  
    MINT_OS_EXIT_CRITICAL();
    if(temp.g_plc_tot_num !=0)
    {
        //mse_header & mse_payload are used to denote u(16,16f)
        temp.g_plc_pkt_mse_avg=temp.g_plc_pkt_mse/temp.g_plc_tot_num;
    }         

    UINT16 read3= BB_read_baseband_register(PLC_CTRL2_REGISTER);
    UINT16 read2 = BB_read_baseband_register(VOICE_SETTING_REGISTER);
    UINT16 read4 = BB_read_baseband_register(BB_PCMOUT_SHIFT_REG);
    UINT16 read5 = BB_read_baseband_register(PLC_CTRL_REGISTER);
    #if 0
    //Use patch to open this LOG,when verify chip
    RT_BT_LOG(RED,BRUCE_DBG_PLC_14,14,plc_pkt_status_var.g_plc_tot_num,
                                      plc_pkt_status_var.g_plc_crc_err,                
                                      plc_pkt_status_var.g_plc_hec_err,
                                      plc_pkt_status_var.g_plc_pkt_miss,
                                      plc_pkt_status_var.g_plc_HW_pkt_miss,
                                      plc_pkt_status_var.g_plc_pkt_miss_burst_error,
                                      plc_pkt_status_var.g_plc_pkt_miss_erasecnt_2,
                                      plc_pkt_status_var.g_plc_pkt_miss_erasecnt_3,
                                      plc_pkt_status_var.g_plc_pkt_miss_erasecnt_5,
                                      temp.g_plc_pkt_mse_avg,
                                read2,read3,read4,read5);
    #endif
    memset(&plc_pkt_status_var,0,sizeof(PLC_CHECK_PKT_STATUS));  
}
#endif

    
