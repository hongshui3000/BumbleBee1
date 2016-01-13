
/***************************************************************************
 Copyright (C) RealTek Ltd.
 This module is a confidential and proprietary property of RealTek and
 a possession or use of this module requires written permission of RealTek.
 ***************************************************************************/

#ifndef MWS_H_
#define MWS_H_

#include "DataType.h"
#include "platform.h"
#include "bt_fw_hci_external_defines.h"
#include "bb_driver.h"
//#include "mws_imp.h"
void mws_init();
void mws_sw_init();
void mws_reg_init(void);
//This function for MWS read register
//UCHAR mws_read_register(UINT16 addr , UINT32 *data);
UINT32 mws_read_register_new(UINT16 addr);

void mws_baudrate_setup(void);
#ifdef _DAPE_ENABLE_MWS_FRAME_SYNC
void mws_set_frame_sync_update_way(UINT8 modify_way);
void mws_handle_lte_frame_sync_isr(void);
void mws_disable_frame_sync_update(void);
void mws_frame_sync_manager_init(void);
void mws_get_frame_sync_clk_info(UINT32 *cur_clk, UINT32 *cur_us,
    UINT8 *latched_clk1_0, UINT8 *last_clk1_0);
void mws_check_if_frame_sync_need_modify(UINT8 *need_to_modify);
void mws_decide_modify_way(void);
void mws_decide_write_value_when_need_to_modify(UINT8 *cant_update, UINT8 *write_case);
void mws_decide_write_value_when_target_met(UINT8 *cant_update, UINT8 *write_case);
void mws_decide_write_value_protection(UINT8 *cant_update, UINT8 *write_case);
void mws_calculate_clk_and_us(UINT16 total_offset_us_write, UINT8 *write_clk1_0, UINT16 *write_clk_cnt);
void mws_write_bb_frame_sync_updt(UINT8 cant_update, 
                                BZ_REG_S_MWS_CLKADJ_CTRL1 *reg_26e, 
                                BZ_REG_S_MWS_CLKADJ_CTRL4 *reg_274,
                                UINT8 write_clk1_0, UINT16 write_clk_cnt);
void mws_frame_sync_update_value_for_next_time(UINT8 cant_update, 
                                                               UINT8 latched_clk1_0);

#endif
//This function for MWS write register

BOOLEAN mws_write_register(UINT16 addr, UINT32 data , UINT16 num_byte);

BOOLEAN mws_uart_nrt_tx_write_reg(UINT32 data,UINT16 num_byte);
#ifdef MWS_ENABLE
void mws_hand_shake_timer(TimerHandle_t timer_handle);
#endif

/*this function for parse LTE-to-BT real /nonreal-time uart message */
#ifdef MWS_16_BIT_MODE
void mws_to_bt_message_16bit(UCHAR is_rt_msg);
#endif

#ifdef MWS_8_BIT_MODE

void mws_to_bt_message_8bit(UCHAR is_rt_msg);
#endif

//MWS related REG defined 

#define RT_RECEIVER_BUF_REG                                  0xFF0
#define NRT_RECEIVER_BUF_REG                                 0xFF1
#define MWS_INT_STATUS_REG_0                                 0xFF2
#define MWS_INT_STATUS_REG_1                                 0xFF3
#define MWS_INT_ebable_REG_0                                 0xFF4
#define MWS_INT_ebable_REG_1                                 0xFF5
#define UART_LINE_STATUS_REG_0                               0xFF6
#define UART_LINE_STATUS_REG_1                               0xFF7
#define UART_FIFO_COMTROL_REG_0                              0xFF8
#define UART_FIFO_COMTROL_REG_1                              0xFF9
#define UART_NRT_BT_TX_HOLDING_REG_0                         0xFFA
#define UART_NRT_BT_TX_HOLDING_REG_1                         0xFFB
#define UART_MISCR                                           0xFFC     
#define UART_LINE_CONTROL_REG                                0xFFD
#define BT2LTE_COEX_INDIRECT_ACCESS_REG_0                    0xFFE
#define BT2LTE_COEX_INDIRECT_ACCESS_REG_1                    0xFFF


#define READY            0x1
#define READ_OPERATION   0x0
#define WRITE_OPERATION  0x1

#define MWS_W_R_SUCCESS  0x1
#define MWS_W_R_FAIL     0x0



#ifdef _DAPE_ENABLE_MWS_FRAME_SYNC
typedef enum
{
    MWS_FRAME_SYNC_DISABLE = 0x00,   /**< Disable MWS Frame Sync */
    MWS_FRAME_SYNC_SELF_UPDT,        /**< Self align Frame sync */
    MWS_FRAME_SYNC_UPDT_BY_REQUEST   /**< Update Clock By Request */
} MWS_FRAME_SYNC_TYPE;
typedef enum
{
    MWS_FRAME_SYNC_IDLE = 0x00, /**< Disable MWS Frame Sync */
    MWS_FRAME_SYNC_INCREASING , /**< Disable MWS Frame Sync */
    MWS_FRAME_SYNC_DECREASING,        /**< Self align Frame sync */
} MWS_FRAME_SYNC_UPDT_WAY;
/*==========================================================*/
/*          SW Structure for MWS Frame Sync Unit            */
/*==========================================================*/
typedef struct BT_MWS_MANAGER_S_ {    
    UINT32 latched_clk;               /* clk[27:0] of latched frame sync */
    UINT32 last_clk;                  /* clk[27:0] of last frame sync */

    UINT16 latched_clk_us;            /* clk_cnt of latched frame sync. (count-down)*/  
    UINT16 last_clk_us;               /* clk_cnt of last frame sync. (count-down)*/
    UINT16 target_clk_us;             /* target clk_cnt (count-down)*/
    UINT16 total_offset_us_current;
    
    UINT16 total_offset_us_latched;
    UINT16 total_offset_us_target;
    UINT16 total_offset_us_last_time;
    UINT16 total_offset_us_write;
    
    UINT16 modify_us;                 /* the value of cnt update of every frame sync. */
    UINT16 wraparound_protect_value;  /* the value that we can't count in that is bigger than. */
    UINT16 force_modify_value;        /* the differencr between target and current value that we can direct change. */
    UINT16 last_total_offset_us_write; /* the total offset wrote last time. */
    UINT16 frame_sync_disable_threshold; /* the threshold that we disable Frame sync update.*/
    UINT16 frame_sync_detect_timer;   /* the time (unit:10ms) that we didn't receive frame sync. It is reset when
                                         we receive frame sync interrupt. */
    UINT8  frame_sync_case:3;         /* the case about mws frame sync */
    UINT8  target_clk1_0:2;           /* target_clk[1:0] */
    UINT8  used_piconet_id:2;         /* used piconet id to get native latch_clk_info. this is
                                         used for other function to be ware of this pn-info
                                         register[0]. */
    UINT8  init:1;                    /* if has received frame sync intr before or not.
                                         since last_clk & last_clk_us need frame sync intr 
                                         at the first time. */
    UINT8  modification_threshold;    /* the threshold of the increasing/decreasing time to modify clock.*/
    UINT8  us_increasing;             /* the occurrence time that latched time of frame sync is increasing.*/
    UINT8  us_decreasing;             /* the occurrence time that latched time of frame sync is decreasing.*/
    UINT8  modify_way:3;              /* the direction about frame sync update. */
    UINT8  last_modify_way:3;         /* the direction about frame sync update of last frame sync intr. */
    UINT8  log_en:1;                  /* enable log or not. */
    UINT8  rsvd:1;                    /* reserved */
    
} BT_MWS_MANAGER_S;
extern BT_MWS_MANAGER_S bt_mws_var;

#endif

#endif /* MWS_H_ */

