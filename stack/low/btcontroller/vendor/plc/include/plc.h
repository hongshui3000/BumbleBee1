
/***************************************************************************
 Copyright (C) RealTek Ltd.
 This module is a confidential and proprietary property of RealTek and
 a possession or use of this module requires written permission of RealTek.
 ***************************************************************************/

#ifndef __PLC_H__
#define __PLC_H__

#include "DataType.h"
#include "bt_fw_hci_external_defines.h"

/* register definition */
#define	PLC_CTRL_REGISTER        0x23E  
#define	PLC_DATA_REGISTER        0x240

#define	PLC_CTRL2_REGISTER        0x254  
#define	PLC_CTRL3_REGISTER        0x256

/* The Structure of PLC_CTRL_REGISTER (0x23E) */
typedef struct PLC_REG_CTRL_ {
    UINT16 opt_ol_ext:1;            /* bit[0], 
                                        0: length ola = 1/4 pitch
                                        1: length ola = 40 samples */
    UINT16 opt_cf:2;                /* bit[2:1], 
                                        00: FIR coeff x1 , IIR coeff x 0.996
                                        01: FIR coeff x1 , IIR coeff x 0.994
                                        10: FIR coeff x 0.96, IIR coeff 0.96
                                        11: FIR coeff x 0.95, IIR coeff 0.95 */
    UINT16 opt_atc_sub_sel_mode:1;  /* bit[3], 00: search cross correlation. 
                                        search maximal value, 1 next to the 
                                        other mode */
    UINT16 opt_atc_sel_mode:1;      /* bit[4], er mode */
    UINT16 opt_add_ers:1;           /* bit[5], 0: normal mode. 1: activate 
                                        samples producing comsuming time diff 
                                        compensation */
    UINT16 opt_ex_fir:2;            /* bit[7:6], 00: 4 order lp, 01: 8 order lp
                                        10: 12 order lp, 11: 16 order lp */
    UINT16 opt_lnr_dt:1;            /* bit[8], option_lnr_dt */
    UINT16 option_plc_halt:1;       /* bit[9], 1:stop the extrapolation waveform,but data still 
                                                                          go through the plc.(MUTE)     0:disable*/                                
    UINT16 optin_at_cf_cr:1;        /*bit[10], 1:Auto lower down the LP order to aviod the coeff saturation.
                                                                        0:disable.*/                                                                        
    UINT16 rsvd:1;                  /* bit[11], reserved */
    UINT16 psd_sco_slot:1;          /* bit[12], psd_sco_slot */
    UINT16 psd_pcm_pkt_dur:1;       /* bit[13], psd_pcm_pkt_dur */
    UINT16 bluetooth_clock:1;       /* bit[14], bluetooth_clock */
    UINT16 plc_test_mode:1;         /* bit[15], plc_test_mode */
} PLC_REG_CTRL, *PPLC_REG_CTRL;

#ifdef _PLC_TEST_MODE_ENABLE_
UINT16 bt_clk_ctrl;
/*  The Structure of PLC_TEST_MANAGER */
typedef struct PLC_TEST_MANAGER_ {
    UINT32 bb_clk_value;  
    UINT8  esco_interval;
    UINT8  esco_pkt_type;
    UINT8  esco_rx_pkt_len;
    UINT8  enq_clk_start_offset;
    UINT8  enq_clk_cnt;
    UINT8  is_enq_err;    
    UINT8  deq_clk_start_offset;    
    UINT8  deq_clk_cnt;
    
    UINT16 in_buf_src_offset;
    UINT16 in_buf_src_wlen;
    UINT16 out_buf_offset;
    UINT16 out_buf_wlen;
    
    UINT16 *in_buf_src;    
    UINT16 *in_buf;    
    UINT16 *out_buf;   

    UINT8  rx_seqn;
    UINT8  tx_seqn;
    UINT8  wptr;
    UINT8  rptr;
    
    //bruce setting psd_pcm_pkt_dur 
    UINT16 psd_pcm_pkt_dur_counter;
	UINT16 erasecnt;
    UINT16 CMD_Flow_count_old;
    UINT16 CMD_Flow_count_new;
} PLC_TEST_MANAGER;
#endif

void plc_init(void);

#ifdef _PLC_TEST_MODE_ENABLE_
extern PLC_TEST_MANAGER plt_test_var;
void plc_test_mode_init(void);
void plc_test_emu_flush_and_push_sco_rx_fifo(UINT16 *pbuf, UINT16 word_len, UINT8 err);
void plc_test_emu_pop_codec_data(UINT16 *pbuf, UINT16 word_len);
void plc_test_emu_toggle_bluetooth_clock(void);

#ifdef _PLC_TEST_USED_VENDOR_COMMAND_
UCHAR hci_vendor_handle_send_test_data_cmd(HCI_CMD_PKT *hci_cmd_ptr);
HCI_EVENT_PKT *hci_generate_test_data_event(void);
#endif
#endif

#ifdef _BRUCE_TEST_PLC_PKT_STATUS
void plc_print_esco_pkt_status(void);
#endif


#endif

