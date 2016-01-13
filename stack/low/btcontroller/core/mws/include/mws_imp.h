/***************************************************************************
 Copyright (C) RealTek Ltd.
 This module is a confidential and proprietary property of RealTek and
 a possession or use of this module requires written permission of RealTek.
 ***************************************************************************/

#ifndef MWS_IMP_H_
#define MWS_IMP_H_

#include "mws.h"



//MWS related REG defined 


#define READY                0x1
#define MWS_READ_OPERATION   0x0
#define MWS_WRITE_OPERATION  0x1


//REG define
#define BTLTECOEX_INDIRECT_ACCESS_REG_0     0xB000A370
#define BTLTECOEX_INDIRECT_ACCESS_REG_1     0xB000A374
#define BTLTECOEX_INDIRECT_ACCESS_REG_2     0xB000A378
#define UART_DLL_REG                        0x0
#define UART_DLM_REG                        0x04
#define UART_XFACTOR_REG                    0x08
#define	UART_XFACTOR_ADJ_REG                0x0C
#define UART_LINE_CTRL_REG                  0x10
#define UART_MISC_CTRL_REG                  0x14
#define UART_BT_TX_HOLD_REG                 0x18
#define UART_WL_TX_HOLD_REG                 0x1C
#define UART_FIFO_CTRL_REG                  0x20
#define UART_LINE_STATUS_REG                0x24
#define DEBUG_PORT_SEL_REG                  0x28
#define DEBUG_PORT_OUTPUT_REG               0x2C
#define UART_NRT_RX_BUF_REG                 0x30
#define UART_RT_RX_BUF_REG                  0x34
#define LTECOEX_CTRL_REG                    0x38
#define CUST_MODE_RT_TYPE_CTRL_REG          0x40
#define CUST_MODE_RT_SIGNAL_CTRL_REG        0x44
#define INTERRUPT_PATTERN1_REG              0x48
#define INTERRUPT_PATTERN2_REG              0x4C
#define FW_RX_RT_CMD_EN_REG                 0x50
#define LTE_COEX_STATUS_REG                 0x54
#define LTECOEX_PATTERN0_REG                0x58
#define LTECOEX_PATTERN1_REG                0x5C
#define LTECOEX_PATTERN2_REG                0x60
#define LTECOEX_PATTERN3_REG                0x64
#define LTECOEX_PATTERN4_REG                0x68
#define LTECOEX_PATTERN5_REG                0x6C
#define LTECOEX_PATTERN6_REG                0x70
#define LTECOEX_SLOT_TIME_REG               0x74
#define LTECOEX_TX_TO_REG                   0x78
#define LTECOEX_RX_TO_REG                   0x7C
#define LTE_FRAMESYNC_SETUP_REG             0x80
#define LTE_RX_SETUP_TIME_REG               0x84
#define LTE_TX_SETUP_TIME_REG               0x88
#define LTE_PATTERN_SETUP_TIME_REG          0x8C
#define LTE_IGNORE_TRX_SETUP_TIME_REG       0x90
#define LTE_HW_RT2LTE_EN_REG                0x94
#define PTA_WL_TRX_CTRL_REG                 0xA0
#define PTA_BT_TRX_CTRL_REG                 0xA4
#define PTA_WL_BREAK_LTE_CTRL_REG           0xA8
#define PTA_BT_BREAK_LTE_CTRL_REG           0xAC
#define PTA_LTE_BREAK_WL_CTRL_REG           0xB0
#define PTA_LTE_BREAK_BT_CTRL_REG           0xB4
#define LTE_TX_PRT_CNT_REG                  0xB8
#define LTE_RX_PRT_CNT_REG                  0xBC
#define LTE_T_LTE_CNT_REG                   0xC0
#define INTERRUPT_STATUS_REG                0x200
#define INTERRUPT_EN_REG                    0x204
#define MWS_FPGA_CONFIG                     0x300
#define MWS_FPGA_DBGO                       0x304

//real time message type 
#define RT_SIGNAL_MESSAGE                   0x0
#define UPDATE_OF_RT_SIGNAL                 0x1
#define NRT_SIGNAL_MESSAGE                  0x2
#define MWS_INACT_DURATION_MESSAGE          0x3
#define MWS_SCAN_MESSAGE                    0x4
#define MWS_FRAME_SETUP_TIME                0x5
#define WLAN_BEACON_AND_URGENT              0x6
#define BT_HIGH_PRI_RX_AND_URGENT           0x7

//interrupt 
#define MWS_NRT_MESSAGE                     0x0
#define MWS_RT_MESSAGE                      0x1

//FPGA register
#define LTECOEX_FPGA_EN                     BIT31
#define UART_FRAME_SYNC                     BIT30
#define LTE_RX                              BIT17
#define LTE_TX                              BIT16
#define BT_PRI                              BIT11
#define BT_RX                               BIT10
#define BT_TX                               BIT9
#define GNT_BT_PTA                          BIT8
#define WL_PRI                              BIT3
#define WL_RX                               BIT2
#define WL_TX                               BIT1
#define GNT_WL_PTA                          BIT0



//ERROR MSG






//this struct for BT2LTECOEX indirect access reg0 (vendor register file of BC) A370
typedef struct BT2LTECOEX_INDIRECT_ACCESS_REG0_{
    UINT32  ltecoex_reg_addr:16;            /* LTECOEX register address field
                                               Note: this field be protected not to be written when ready bit is 0 */
                                                
    UINT32  write_byte_enable:4;            /* write byte enable when write operation */
    UINT32  rsvd:9;                         
    UINT32  ltecoex_ready_bit:1;            /* When FW poll this bit is 1, FW can start to issue a ltecoex access command.
                                               Note This bit is 1 when 1. cpu reset release.
                                                                       2. LTECOEX2BT_inf assert ready bit
                                               Otherwise, when LTECOEX_Access_Start (bit 31) set as 1. this bit is cleared. */
    UINT32  ltecoex_write_mode:1;           /* 0: read operation,1: write operation
                                               Note: this field be protected not to be written when ready bit is 0*/
    
    UINT32  ltecoex_access_start:1;         /* The bit is set for issue a read/write operation, so the corresponding I/O
                                               value must be written at the same time. i.e., read/write bit, dummy bits, 
                                               LTECOEX register address, and LTECOEX write data when write.
                                               Note: 
                                                1. Before this bit set, FW has to check the ready bit is 1'b1.
                                                2.When the ready_bit == 1, the ltecoex_access_start write 1 is valid. Otherwise, the write is invalid
                                                3. The write 1 internally is a one cycle pulse when the write is valid. */

}BT2LTECOEX_INDIRECT_ACCESS_REG0;

static inline UINT32 bt_lte_coex_indirect_access_reg_read(void)
{
    return RD_REG_MMIO(UINT32, BTLTECOEX_INDIRECT_ACCESS_REG_2);
} 
/*static inline void bt_lte_coex_indirect_access_reg_read(UINT32 *reg)
{
    *reg = RD_REG_MMIO(UINT32, BTLTECOEX_INDIRECT_ACCESS_REG_2);
}*/

static inline void bt_lte_coex_indirect_access_reg_write(UINT32 reg)
{
    WR_REG_MMIO(UINT32, BTLTECOEX_INDIRECT_ACCESS_REG_1, reg);
}

static inline void bt_lte_coex_indirect_access_reg(BT2LTECOEX_INDIRECT_ACCESS_REG0 reg)
{
    WR_REG_MMIO(BT2LTECOEX_INDIRECT_ACCESS_REG0, BTLTECOEX_INDIRECT_ACCESS_REG_0, reg);
}

/*static inline void bt_lte_coex_access_reg_write(UINT32 reg ,UINT16 addr)
{
    WR_REG_MMIO(UINT32, addr, reg);
}*/

static inline UINT8 mws_is_ready()
{
    return (RD_REG_MMIO(BT2LTECOEX_INDIRECT_ACCESS_REG0, BTLTECOEX_INDIRECT_ACCESS_REG_0).ltecoex_ready_bit);
}



#if 0
SECTION_ISR void mws_handle_nrt_bt_tx_done_interrupt(UINT32 status);
SECTION_ISR void mws_handle_nrt_wlan_tx_done_interrupt(void);
SECTION_ISR void mws_handle_nrt_rx_interrupt(void);
SECTION_ISR void mws_handle_rt_rx_interrupt(void);
SECTION_ISR void mws_handle_uart_lsr_interrupt(void);
SECTION_ISR void mws_handle_uart_nrt_rx_timeout_interrupt(void);
SECTION_ISR void mws_handle_uart_rt_rx_timeout_interrupt(void);
SECTION_ISR void mws_handle_nrt_rx_full_interrupt(UINT32 interrupt_status);
SECTION_ISR void mws_handle_rt_rx_full_interrupt(UINT32 interrupt_status);
#endif

#endif
