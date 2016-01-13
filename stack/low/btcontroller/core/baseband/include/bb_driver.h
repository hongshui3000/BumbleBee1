/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  Baseband driver interface.
 */

/** \addtogroup bb_driver Baseband Driver Module
 *  @{ */
#ifndef __BB_DRIVER_H__
#define __BB_DRIVER_H__

#include "bt_fw_types.h"
#include "bt_fw_os.h"
#include "platform.h"

/* Define the macros to determine interrupt source */

#define BB_HLC_INT(int_stat_reg)\
    ((int_stat_reg) & (UINT16)0x0020)

#define BB_RX_INT(int_stat_reg)\
    ((int_stat_reg) & (UINT16)0x0001)

#define BB_RX_MASK(int_mask_reg)\
    ((int_mask_reg) & (UINT16)0x0001)

#define BB_TX_INT(int_stat_reg)\
    ((int_stat_reg) & (UINT16)0x02)

#define BB_TX_MASK(int_mask_reg)\
    ((int_mask_reg) & (UINT16)0x02)

#define BB_MODE_STAT_INT(int_stat_reg)\
    ((int_stat_reg) & (UINT16)0x04)

#define BB_MODE_STAT_MASK(int_mask_reg)\
    ((int_mask_reg) & (UINT16)0x04)

#define BB_ITO_MASK(int_mask_reg)\
    ((int_mask_reg) & (UINT16)0x08)

#define BB_SLOT_COUNT_MASK(int_mask_reg)\
    (((int_mask_reg) & (UINT16)0x100)>>8)

#define BB_ITO_INT(int_stat_reg)\
    ((int_stat_reg) & (UINT16)0x08)

#define BB_SNIFF_INT(int_stat_reg)\
        ((int_stat_reg) & 0x0200)

#define BB_ACCESS_CODE_MISS_INT(int_stat_reg) \
        ((int_stat_reg) & 0x1000)

#define BB_ACCESS_CODE_MISS_MASK(int_mask_reg) \
        ((int_mask_reg) & (UINT16)0x1000)

#define BB_ESCO_START_OF_WINDOW_MASK(int_mask_reg) \
        ((int_mask_reg) & (UINT16)0x400)

#define BB_ESCO_END_OF_WINDOW_MASK(int_mask_reg) \
        ((int_mask_reg) & (UINT16)0x800)
#define BB_TXFIFO_INT(int_stat_reg)\
    (((int_stat_reg) & (UINT16)0x10) >> 3)

#define BB_SLOT_COUNTER_INT(int_stat_reg) \
    ((int_stat_reg & (UINT16)0x0100))

#define BB_LAM_INT(int_stat_reg)        \
    ((int_stat_reg) & (UINT16)0x2000)

#define BB_HW_CONN_INTR(mode_status_reg)\
    (((mode_status_reg) & 0x0010)>>4)

#define BB_ESCO_START_OF_WINDOW_INT(int_stat_reg) \
    ((int_stat_reg) & 0x0400)

#define BB_ESCO_END_OF_WINDOW_INT(int_stat_reg) \
    ((int_stat_reg) & 0x0800)

#define BB_EXIT_POWER_CTRL_INT(int_stat_reg) \
    ((int_stat_reg) & 0x4000)

#define BB_EXIT_POWER_CTRL_MASK(int_mask_reg) \
    ((int_mask_reg) & 0x4000)

#ifdef LE_MODE_EN
#define BB_LE_MODE_INT(int_stat_reg) \
    ((int_stat_reg) & 0x8000)

#define BB_LE_MODE_MASK(int_mask_reg) \
    ((int_mask_reg) & 0x8000)
#endif

//MWS interrupt section

#define NRT_BT_TX_DONE_INTERRUPT_STATUS(mws_interrupt_status_reg) \
((mws_interrupt_status_reg) & (UINT16)0x01)

#define NRT_WL_TX_DONE_INTERRUPT_STATUS(mws_interrupt_status_reg) \
((mws_interrupt_status_reg) & (UINT16)0x0002)

#define NRT_RX_INTERRUPT_STATUS(mws_interrupt_status_reg) \
((mws_interrupt_status_reg) & (UINT16)0x04)

#define RT_RX_INTERRUPT_STATUS(mws_interrupt_status_reg) \
((mws_interrupt_status_reg) & (UINT16)0x0008)

#define UART_LSR_INTERRUPT_STATUS(mws_interrupt_status_reg) \
((mws_interrupt_status_reg) & (UINT16)0x0010)

#define UART_NRT_RX_TIMEOUT_STATUS(mws_interrupt_status_reg) \
((mws_interrupt_status_reg) & (UINT16)0x0020)

#define UART_RT_INTERRUPT_STATUS(mws_interrupt_status_reg) \
((mws_interrupt_status_reg) & (UINT16)0x0040)

#define NRT_RX_FULL_STATUS(mws_interrupt_status_reg) \
((mws_interrupt_status_reg) & (UINT16)0x0080)

#define RT_RX_FULL_STATUS(mws_interrupt_status_reg) \
((mws_interrupt_status_reg) & (UINT16)0x0100)

#define LTE_TX_ON_STATUS(mws_interrupt_status_reg) \
((mws_interrupt_status_reg) & (UINT16)0x0200)

#define LTE_TX_TO_IDLE_RX_STATUS(mws_interrupt_status_reg) \
((mws_interrupt_status_reg) & (UINT16)0x0400)

#define LTE_FRAMESYNC_STATUS(mws_interrupt_status_reg) \
((mws_interrupt_status_reg) & (UINT16)0x0800)

#define LTE_INT_PATTERN1_STATUS(mws_interrupt_status_reg) \
((mws_interrupt_status_reg) & (UINT16)0x01000)

#define LTE_INT_PATTERN2_STATUS(mws_interrupt_status_reg) \
((mws_interrupt_status_reg) & (UINT16)0x02000)

#define LTE_TX_TO_STATUS(mws_interrupt_status_reg) \
    ((mws_interrupt_status_reg) & (UINT16)0x04000)

#define LTE_RX_TO_STATUS(mws_interrupt_status_reg) \
((mws_interrupt_status_reg) & (UINT16)0x08000)








/* Macros to extract different fields of payload header and packet header */
#define BB_GET_ACK_FROM_RXSTAT_REG(rx_status)\
    (((rx_status) & 0x0200)>>9)

#define BB_GET_HW_FLOW_BIT(pkt_hdr)\
    (((pkt_hdr) & 0x0400)>>10)

#define BB_GET_SW_FLOW_BIT(payload_hdr)\
    (((payload_hdr) & L_FLOW_BIT)>>14)

#define BB_GET_LCH_FROM_PAY_HDR(payload_hdr)\
    ((UCHAR)(((payload_hdr) & L_CH_MASK) >> 12))

#define BB_GETSEQN_FROM_RXSTAT_REG(pkt_hdr)\
    (((pkt_hdr) & 0x0100)>>8)

#define BB_GET_AMADDR_FROM_RXSTAT_REG(pkt_hdr)\
        (((pkt_hdr) & 0x3800)>>11)

#define BB_GET_PKT_TYPE_FROM_RXSTAT_REG(pkt_hdr)\
    (((pkt_hdr) & 0x00f0)>>4)

#define BB_IS_ARQN_BIT(packet_header) \
        ((packet_header) & 0x0200)


#define BB_PKTHDR_HEC_ERROR(pkt_hdr)\
    (((pkt_hdr) & 0x0002)>>1)

#define BB_PKTHDR_CRC_ERROR(pkt_hdr)\
    (((pkt_hdr) & 0x0008)>>3)

#define BB_GET_ERROR_FROM_RXSTAT_REG(pkt_hdr)\
    ((pkt_hdr) & 0x0008)

#define BB_WRITE_INQ_PARITY_BITS(val)    \
        bb_write_inq_parity_bit(val)

#define BB_WRITE_REMOTE_PARITY_BITS(val, piconet_id)    \
        bb_write_remote_parity_bit(val, piconet_id)

#define BB_WRITE_LOCAL_PARITY_BITS(val)    \
        bb_write_local_parity_bit(val)

#define BB_SET_BIT(var, pos, val)\
    (UINT16) ((val) ? ((var) | (1 << (pos))) : ((var) & ~(1 << (pos))))

#ifndef _CCH_AFH_E0_AES_IN_CAM_
#define            AFH_ENABLE_BIT        0x8000
#endif

#define BB_FLOW                0
#define BB_SEQN                1
#define BB_ARQN                2

#define BB_NAK                 0
#define BB_ACK                 1
#define BB_GO                  1
#define BB_STOP                0

#define BB_L2CAP_FLOW_BIT_POS  5

#define SCA_PICONET_ID_MASTER_PICONET                      0x00
#define SCA_PICONET_ID_SLAVE1_PICONET                      0x01
#define SCA_PICONET_ID_SLAVE2_PICONET                      0x02
#define SCA_PICONET_ID_SLAVE3_PICONET                      0x03
#define SCA_PICONET_ID_SLAVE4_PICONET                      0x04

#define SCA_SLAVE1_LOWER_LUT                              0x100
#define SCA_SLAVE1_UPPER_LUT                              0x102
#define SCA_SLAVE2_LOWER_LUT                              0x104
#define SCA_SLAVE2_UPPER_LUT                              0x106
#define SCA_SLAVE3_LOWER_LUT                              0x13E
#define SCA_SLAVE3_UPPER_LUT                              0x140
#define SCA_SLAVE4_LOWER_LUT                              0x142
#define SCA_SLAVE4_UPPER_LUT                              0x144

#define LC_SCA_SLAVE_1_LUT                                 0x08
#define LC_SCA_SLAVE_2_LUT                                 0x09
#define LC_SCA_SLAVE_3_LUT                                 10
#define LC_SCA_SLAVE_4_LUT                                 11

/* Max Retransmission effort (N-Retrans) */

#define BB_MAX_RE_TRANS        0x8

/*    Define Error Code Masks */
#define BB_ABORT               0
#define BB_RETURN              1
#define BB_HW_ERROR            2


/* Defines for interrupt numbers. */
#define RX_INTR                      0x0001
#define TX_INTR                      0x0002
#define MODE_STS_INTR                0x0004
#define TIMEOUT_INTR                 0x0008
#define HLC_INTR                     0x0020
#define SLOT_COUNTER_INTR            0x0100
#define SNIFF_INTR                   0x0200
#define START_ESCO                   0x0400
#define END_ESCO                     0x0800
#define ACC_CODE_MISS_INTR           0x1000
#define LAM_INTR                     0x2000
#define SM_INTR                      0x4000

#ifdef LE_MODE_EN
#define LL_INTR                      0x8000
#endif

#define RX_ID_INTR                   0x0001
#define START_OF_SYNC_INTR           0x0002
#define SCO_TX_INTR                  0x0004
#define SCO_RX_INTR                  0x0008

#define BB_INTR1_EN_MASK_ORI ((RX_INTR) | (TIMEOUT_INTR) | (MODE_STS_INTR) | \
                             (TX_INTR) | (SLOT_COUNTER_INTR) | (SNIFF_INTR) | \
                             (START_ESCO) | (END_ESCO) | (LAM_INTR) | \
                             (SM_INTR) | (ACC_CODE_MISS_INTR) | (HLC_INTR))
#ifdef LE_MODE_EN
#define BB_INTR1_MASK_ALL   ((BB_INTR1_EN_MASK_ORI) | (LL_INTR))
#else
#define BB_INTR1_MASK_ALL   BB_INTR1_EN_MASK_ORI
#endif
#define BB_INTR2_MASK_ALL   ((START_OF_SYNC_INTR)|(SCO_TX_INTR)|(SCO_RX_INTR))

#define PICONET1_INFO_REGISTER                               0x108
#define PICONET2_INFO_REGISTER                               0x10A
#define PICONET3_INFO_REGISTER                               0x146
#define PICONET4_INFO_REGISTER                               0x148

/* Baseband Control */
#define	INSTRUCTION_REGISTER	                              0x00
#define	CONNECTOR_REGISTER                                    0x02

/* Baseband status control*/
#define	MODE_STATUS_REGISTER	                              0x00
#define	VERSION_REGISTER                                      0x60

#define PCD_HIDDEN_STATUS_REGISTER                            0xFC

#define	RECEIVED_PAYLOAD_HEADER_REGISTER                      0x04
#define	RECEIVED_STATUS_REGISTER                              0x06
#define	TRANSMIT_STATUS_REGISTER                              0x08
#define	TRANSMIT_FIFO_STATUS_REGISTER          	              0x0A
#define	RECEIVE_FIFO_STATUS_REGISTER1	                      0x0C
#define	RECEIVE_FIFO_STATUS_REGISTER2          	              0x0E

/* Device Configuration */

#define	INQUIRY_PARITY_BITS_REGISTER1	                      0x10
#define	INQUIRY_PARITY_BITS_REGISTER2	                      0x12
#define	GIAC_DAIC_REGISTER1                                   0x14
#define	INQUIRY_PARITY_BITS_REGISTER3	                      0x16

#define	PICONET1_PARITY_BITS_REGISTER3                        0x16
#define	PICONET2_PARITY_BITS_REGISTER3                       0x11A

#define	LOCAL_PARITY_BITS_REGISTER3                           0x16
#define	GIAC_DAIC_REGISTER2                                   0x16

#define EIR_ENABLE_REGISTER                                   0x16
#define EIR_ENABLE_BIT                                       0x200
#define EIR_RECV_ENABLE_BIT                                  0x100

#define PICONET1_BD_ADDR1_REGISTER                            0x18
#define	PICONET1_BD_ADDR2_REGISTER                            0x1a
#define	PICONET1_BD_ADDR3_REGISTER                            0x1c
#define	PICONET1_PARITY_BITS_REGISTER1                        0x1e
#define	PICONET1_PARITY_BITS_REGISTER2	                      0x20

#define	PICONET2_BD_ADDR1_REGISTER                           0x110
#define	PICONET2_BD_ADDR2_REGISTER                           0x112
#define	PICONET2_BD_ADDR3_REGISTER                           0x114

#define	PICONET2_PARITY_BITS_REGISTER1                       0x116
#define	PICONET2_PARITY_BITS_REGISTER2	                     0x118

#define	PICONET3_BD_ADDR1_REGISTER                           0x14A
#define	PICONET3_BD_ADDR2_REGISTER                           0x14C
#define	PICONET3_BD_ADDR3_REGISTER                           0x14E
#define	PICONET3_PARITY_BITS_REGISTER1                       0x150
#define	PICONET3_PARITY_BITS_REGISTER2	                     0x152
#define	PICONET3_PARITY_BITS_REGISTER3	                     0x154

#define	PICONET4_BD_ADDR1_REGISTER                           0x156
#define	PICONET4_BD_ADDR2_REGISTER                           0x158
#define	PICONET4_BD_ADDR3_REGISTER                           0x15A
#define	PICONET4_PARITY_BITS_REGISTER1                       0x15C
#define	PICONET4_PARITY_BITS_REGISTER2	                     0x15E
#define	PICONET4_PARITY_BITS_REGISTER3	                     0x160

#define	REMOTE_DEVICE_BD_ADDR1_REGISTER                       0x18
#define	REMOTE_DEVICE_BD_ADDR2_REGISTER                       0x1a
#define	REMOTE_DEVICE_BD_ADDR3_REGISTER                       0x1c
#define	REMOTE_PARITY_BITS_REGISTER1                          0x1e
#define	REMOTE_PARITY_BITS_REGISTER2                          0x20
#define	LOCAL_BD_ADDR1_REGISTER	                              0x22
#define	LOCAL_BD_ADDR2_REGISTER                               0x24
#define	LOCAL_BD_ADDR3_REGISTER	                              0x26
#define	LOCAL_PARITY_BITS_REGISTER1          	              0x28
#define	LOCAL_PARITY_BITS_REGISTER2                           0x2A

/* Protocol registers */

#define	N_INQUIRY_REGISTER                                    0x2C
#define	N_PAGE_REGISTER	                                      0x2C
#define	INQUIRY_TIMEOUT_REGISTER                              0x2E
#define	PAGE_TIMEOUT_REGISTER                                 0x2E
#define	PAGE_SCAN_INTERVAL_REGISTER                           0x30
#define	PAGE_SCAN_WINDOW_REGISTER                             0x32
#define	INQ_SCAN_INTERVAL_REGISTER                            0x34
#define	INQ_SCAN_WINDOW_REGISTER                              0x36
#define	TIMEOUT_STATUS_REGISTER	                              0x38

/* Low Power Mode register */

#define	T_B_REGISTER                                          0x3A
#define	D_B_REGISTER                                          0x3C
#define	D_ACCESS_REGISTER                                     0x3E
#define	N_ACC_SLOTS_REGISTER	                              0x3E
#define	DELTA_B_REGISTER                                      0x40
#define	T_ACCESS_REGISTER                                     0x40
#define	M_ACCESS_REGISTER                                     0x42
#define	N_POLL_REGISTER	                                      0x42
#define	ACCESS_REQUEST_ADDRESS_REGISTER	                      0x44
#define	RE_TRANSMISSION_COUNT_REGISTER                        0x46
#define	BEACON_PACKET_REPETETION_NO_REGISTER                  0x46
#define	SNIFF_SLOT_OFFSET_INTERVAL_REGISTER                   0x48
#define	SNIFF_ATTEMPT_REGISTER                                0x4A
#define	SNIFF_TIMEOUT_REGISTER                                0x4C
#ifdef _USE_SNIFF_NO_ACL_COUNT_REGISTER
#define SNIFF_NO_ACL_COUNT_REGISTER                           ACCESS_REQUEST_ADDRESS_REGISTER
#endif
#define	TPOLL_HOLD_SNIFF_INTERVAL_REGISTER                    0x4E
#define	X_VALUE_FOR_TOLERANCE_REGISTER                        0x50
#define	PHASE_OFFSET_REGISTER                                 0x52
#define	X_VALUE_FOR_TOLERANCE1_REGISTER                       0x192

#ifdef _RTL8821B_TC_SPECIFIC_
#define BTON_INDIRECT_READ_PAGE0_REGISTER    0x54
#endif

#ifndef _NEW_BTON_DESIGN_AFTER_RTL8821B_TC_
#define BTON_32K_TEST_REG                  0x54
#define BTON_32K_CSR0_REG                  0x58
#define BTON_32K_CSR1_REG                  0x5C
#else
#define BTON_INDIRECT_READ_PAGE0_REGISTER    0x54
#endif

#if defined(_CCH_RETENTION_FLOW_FOR_DLPS_) || defined(_SUPPORT_WL_BT_SCOREBOARD_)
#define	BTON_DLPS_CONTROL_REGISTER                                0xBC
#endif

/* SCO/ESCO Registers */
#define	SCO_INTERVAL_REGISTER                                 0x56
#define	ESCO_INTERVAL_REGISTER                                0x56
#define	SCO_SLOT_OFFSET_INTERVAL_REGISTER                     0x58
#define	ESCO_WINDOW_DESCO_REGISTER                            0x58
#define	ESCO_END_STATUS_REGISTER                              0x5A
#define	ESCO_RX_LENGTH_TYPE_REGISTER                          0x5A
#define	SNIFF_ESCO_START_STATUS_REGISTER                      0x5C
#define	SCO_PACKET_TYPE_REGISTER                              0x5e
#define	ESCO_FIFO_FLUSH_LENGTH_AND_TX_PKT_LEN_REGISTER	      0x60


/* Clock & Power control registers */

#define	CLOCK_OFFSET_REGISTER	                              0x62
#define	NATIVE_CLOCK1_REGISTER	                              0x64
#define	NATIVE_CLOCK2_REGISTER	                              0x66
#define	PICONET_CLOCK1_REGISTER	                              0x68
#define	PICONET_CLOCK2_REGISTER                               0x6A

#define	PICONET2_CLOCK1_REGISTER                             0x10c
#define	PICONET2_CLOCK2_REGISTER                             0x10e
#define PICONET3_CLOCK1_REGISTER                             0x162
#define PICONET3_CLOCK2_REGISTER                             0x164
#define PICONET4_CLOCK1_REGISTER                             0x16E
#define PICONET4_CLOCK2_REGISTER                             0x170

#define NATIVE_CLK_COUNDOWN_US_REG                           0x1DA
#define PICONET1_CLK_COUNDOWN_US_REG                         0x1DC
#define PICONET2_CLK_COUNDOWN_US_REG                         0x1DE
#define PICONET3_CLK_COUNDOWN_US_REG                         0x1E0
#define PICONET4_CLK_COUNDOWN_US_REG                         0x1E2

#define	POWER_CONTROL_REGISTER	                              0x6C
#define	POWER_STATUS_REGISTER	                              0x6C
#define	WAKEUP_INSTANT_REGISTER	                              0x6E

/* Link Control Registers */

#define	PICONET_LOOK_UP_TABLE_BASE_ADDR	                      0x70
#define MASTER_BROADCAST_LOWER_LUT_ADDR                       0x70
#define MASTER_BROADCAST_UPPER_LUT_ADDR                       0x72

#define	SCA_SLAVE1_LOWER_LUT_ADDR                            0x100
#define	SCA_SLAVE1_UPPER_LUT_ADDR                            0x102
#define	SCA_SLAVE2_LOWER_LUT_ADDR                            0x104
#define	SCA_SLAVE2_UPPER_LUT_ADDR                            0x106
#define	SCA_SLAVE3_LOWER_LUT_ADDR                            0x13E
#define	SCA_SLAVE3_UPPER_LUT_ADDR                            0x140
#define	SCA_SLAVE4_LOWER_LUT_ADDR                            0x142
#define	SCA_SLAVE4_UPPER_LUT_ADDR                            0x144
#define SCA_PRIORITY_REGISTER                                0x11C
//merged from BZ 2.1.2
#define SCA_PRIORITY_REGISTER2                               0x11E
#ifdef _DAPE_TEST_NEW_HW
#define SCA_PRIORITY_REGISTER3                               0x120
#endif
#ifdef _DAPE_NEW_HW_AFTER_8821MP
#define OPT_REGISTER                                         0x21C
#endif
/* FIFO Registers */
#define BB_DWORD_FIFO_REG                                     0x400
#define	BB_RXFIFO_BASE_REG	                              0x94
#define	BB_TXFIFO_BASE_REG	                              0x94
#define	BB_SCATTERED_TXFIFO_BASE_REG                          0xA4
#define	BB_ESCO_TX_FIFO_BASE_REG	                      0xF0
#define	BB_ESCO_RX_FIFO_BASE_REG	                      0xF0

/* FIFO Registers for CPU push or pop in 0380 loopback mode - added by austin */
#define BB_LB_ACL_TX_FIFO_RD                                  0x166
#define BB_LB_ACL_RX_FIFO_WR                                  0x166
#define BB_LB_SCAT_TX_FIFO_RD                                 0xA4
#define BB_LB_BCST_TX_FIFO_RD                                 0x96
#define BB_LB_SCO0_TX_FIFO_RD                                 0x16A
#define BB_LB_SCO1_TX_FIFO_RD                                 0x16C
#define BB_LB_SCO0_RX_FIFO_WR                                 0x16A
#define BB_LB_SCO1_RX_FIFO_WR                                 0x16C

#define BB_RX_BKPRT0_ADDR                                     0x1B4
#define BB_RX_BKPRT1_ADDR                                     0x1B6
#define BB_RX_BKPRT2_ADDR                                     0x1B8
#define BB_RX_BKPRT3_ADDR                                     0x1D6
#define BB_RX_BKPRT4_ADDR                                     0x1D8
#define BB_RX_BKPRT0_DATA                                     0x1B4
#define BB_RX_BKPRT1_DATA                                     0x1B6
#define BB_RX_BKPRT2_DATA                                     0x1B8
#define BB_RX_BKPRT3_DATA                                     0x1D6
#define BB_RX_BKPRT4_DATA                                     0x1D8

#ifdef _YL_LPS
#define BB_NATIVE_COUNTER_REG                                 0x1DA
#endif

#define BB_SNIFF_START_REG                                    0x1E8
#define BB_SNIFF_END_REG                                      0x1EA

#ifdef _RTL8821A_
#define BB_INSTRUCTION_STATUS_REG                             0x1F0
#define BB_INSTRUCTION_STATUS_SEL_REG                         0x1F0
#define BB_SNIFF_ANCHOR_REG                                   0x1F2
#define BB_SCAN_ANCHOR_REG                                    0x1F4
#define BB_SCAN_ANCHOR_SEL_REG                                0x1F4
#define BB_RSSI_STACK_REG                                     0x1F6

#define BB_SCO_DEFAULT_REG                                    0x1FC

/* 3DD Application Register Group */
#define BB_3DD_RSCD_REG                                       0x200
#define BB_3DD_RSOD_REG                                       0x202
#define BB_3DD_LSCD_REG                                       0x204
#define BB_3DD_LSOD_REG                                       0x206
#define BB_3DD_CON_REG                                        0x208
#define BB_3DD_BCN_ITV_REG                                    0x20A
#define BB_3DD_SYNC_ITV_REG                                   0x20C
#define BB_3DD_SYNC_BB_TIME0_REG                              0x20E
#define BB_3DD_SYNC_BB_TIME1_REG                              0x210
#define BB_3DD_SYNC_BB_TIME_US_REG                            0x230
#define BB_3DD_SYNC_FRAME_PERIOD_REP_REG                      0x232 //for read
#define BB_3DD_SYNC_FRAME_PERIOD_REG                          0x232 //for write
#define BB_3DD_SYNC_FRAME_PERIOD_FRACTION_REG                 0x234 //for write
#endif
#ifdef _SUPPORT_CSB_RECEIVER_
#define BB_CSB_RX_SYNC_CLK_OFST_REP0_REG                      0x242
#define BB_CSB_RX_SYNC_CLK_OFST_REP1_REG                      0x244
#define BB_CSB_RX_SYNC_CLK_OFST0_REG                          0x242
#define BB_CSB_RX_SYNC_CLK_OFST1_REG                          0x244
#define BB_CSB_RX_SYNC_SCAN_INTV_REG                          0x246
#define BB_CSB_RX_SYNC_SCAN_WINDOW_REG                        0x248
#define BB_CSB_RX_SYNC_PARITY0_REG                            0x24A
#define BB_CSB_RX_SYNC_PARITY1_REG                            0x24C
#define BB_CSB_RX_SYNC_LAP0_REG                               0x24E
#define BB_CSB_RX_SYNC_LAP1_REG                               0x250

#define BB_CSB_RX_SYNC_SCAN_MAP_REG                           0x252
#define BB_CSB_RX_SKIP_REG                                    0x258
#endif

#ifdef _NEW_MODEM_PSD_SCAN_
#define BB_PSD_END_START                                    0x220
#define BB_PSD_STEP_MODE                                    0x222
#define BB_PSD_TIMEOUT                                      0x224
#define PSD_PARA_HALF_SLOT_MODE                             0
#define PSD_PARA_FULL_SLOT_MODE                             1
#define PSD_PARA_HALF_SLOT_TIMEOUT_DEFAULT                  0x3ff // 68, NOTE should add RF STBY time
#define PSD_PARA_FULL_SLOT_TIMEOUT_DEFAULT                  0x3ff // 350, NOTE should add RF STBY time
#endif

#define TRANS_MODEM_REG(x)		(((x) >> 1) | 0x40)  /* this is used to translate byte address to word address*/

#define TX_AGC_1M_LUT0_REG									 0x136
#define TX_AGC_1M_LUT1_REG									 0x138
#define TX_AGC_1M_LUT2_REG									 0x13A
#define TX_AGC_1M_LUT3_REG									 0x13C

#define TX_AGC_2M_LUT0_REG									 0x1BC
#define TX_AGC_2M_LUT1_REG									 0x1BE
#define TX_AGC_2M_LUT2_REG									 0x1C0
#define TX_AGC_2M_LUT3_REG									 0x1C2

#define TX_AGC_3M_LUT0_REG									 0x1C4
#define TX_AGC_3M_LUT1_REG									 0x1C6
#define TX_AGC_3M_LUT2_REG									 0x1C8
#define TX_AGC_3M_LUT3_REG									 0x1CA

#define TX_EDR_GAIN0_REG								     0x1CC
#define TX_EDR_GAIN1_REG								     0x1CE
#define TX_EDR_GAIN2_REG								     0x1D0
#define TX_EDR_GAIN3_REG								     0x1D2
#define TX_EDR_GAIN4_REG								     0x1D4

/* note: HW removes host buffer in 0380 */
/* Address of buffers */
#define PCMRX_TOCPU_FIFO                                      0xA0
#define BBRX_TOCPU_FIFO                                       0x9E
#define CPU_TOPCMTX_FIFO                                      0x9E
#define CPU_TOBBTX_FIFO                                       0xA0

/* Status Registers */
#define RXPATH_STS_REG                                        0xA2
#define TXPATH_STS_REG                                        0xA4
#define HBUF_INTSTS_REG                                       0xDE

/*Packet Composer/Decomposer Registers */

#define	CLASS_OF_DEVICE_REGISTER1                             0xA8
#define	CLASS_OF_DEVICE_REGISTER2                             0xAA
#define	FHS_PARAMETER_REGISTER	                              0xAA
#define	ENCRYPTION_ENABLE_REGISTER                            0xAC
#define	ESCO_STOP_RX_REGISTER	                              0xAC
#define INQ_RESP_FREEZ_REGISTER                               0xAC

#ifndef _CCH_AFH_E0_AES_IN_CAM_
#define	ENCRYPTION_KEY_REGISTER1                              0xAE
#define	ENCRYPTION_KEY_REGISTER2                              0xB0
#define	ENCRYPTION_KEY_REGISTER3                              0xB2
#define	ENCRYPTION_KEY_REGISTER4                              0xB4
#define	ENCRYPTION_KEY_REGISTER5                              0xB6
#define	ENCRYPTION_KEY_REGISTER6                              0xB8
#define	ENCRYPTION_KEY_REGISTER7                              0xBA
#define	ENCRYPTION_KEY_REGISTER8                              0xBC
#endif

/* Host interface registers */
#define	INTERRUPT_REGISTER                                    0xBE
#define	INTERRUPT_MASK_REGISTER	                              0xBE
#define	INTERRUPT_MASK_REGISTER2                              0xC0

/* Radio Interface Registers */
#define	CORRELATOR_THRESOLD_REGISTER          	              0xC2
#define	CHANNEL_REGISTER                                      0xC4
#define	RADIO_CONTROL_REGISTER	                              0xD6
#define	RADIO_ACCESS_REGISTER	                              0xD6
#define	RADIO_SELECT_REGISTER	                              0xD8
#define	EXTENDED_HOST_ACCESS_REGISTER          	              0xDA
#define	RADIO_CONFIG_REGISTER	                              0xDC

/* CODEC interface registers */

#define	VOICE_SETTING_REGISTER	                              0xE0
/* no use in 0380 */
#define PCM_HOST_PATH_TX_CTRL_REGISTER                        0x98
#define PCM_HOST_PATH_RX_CTRL_REGISTER                        0x9A

#define BB_CODEC_TX_PATH1_REG                                 0xA0
#define BB_CODEC_TX_PATH2_REG                                 0x98
#define BB_CODEC_TX_PATH3_REG                                 0x9A
#define BB_CODEC_RX_PATH_REG                                  0xA2
#define BB_CODEC_RX_FIFO_POP_REG                              0xA0
#define BB_CODEC_RX_FIFO_READ_REG                             0x98
#define BB_CODEC_CODE_TABLE_REG                               0x9E
#define BB_SCO_CONV_TABLE_REG                                 0xDE

#define BB_PCM_CTRL1_REG                                      0x1EC
#define BB_PCM_CTRL2_REG                                      0x1EE
#define BB_PCM_CTRL3_REG                                      0x23C
#define BB_PCMOUT_SHIFT_REG                                   0x25E

#ifdef _BRUCE_DROP_MUTED_ISO_OUT_PKTS_OVER_CODEC
#define BB_MUTED_ISO_OUT_PKTS_CTRL1_REG                       0x260
//pcm_in_mute_sample_cnt_threshold
#define BB_MUTED_ISO_OUT_PKTS_CTRL2_REG                       0x27E
#endif

#define MWS_CLKADJ_CTRL0                                      0x26C
#define MWS_CLKADJ_CTRL1                                      0x26E
#define MWS_CLKADJ_CTRL2                                      0x270
#define MWS_CLKADJ_CTRL3                                      0x272
#define MWS_CLKADJ_CTRL4                                      0x274
#define MWS_CLKADJ_RPT0                                       0x276
#define MWS_CLKADJ_RPT1                                       0x278
#define MWS_CLKADJ_RPT2                                       0x27A

/* AFH registers */
#ifdef _CCH_AFH_E0_AES_IN_CAM_
#define	AFH_CHANNEL_MAP_EN_REG                                0xE4
#else
#define	AFH_CHANNEL_MAP_REGISTER0                             0xE4
#define	AFH_CHANNEL_MAP_REGISTER1                             0xE6
#define	AFH_CHANNEL_MAP_REGISTER2                             0xE8
#define	AFH_CHANNEL_MAP_REGISTER3                             0xEA
#define	AFH_CHANNEL_MAP_REGISTER4                             0xEC
#endif


/* Testing related registers */
#define TEST_CONTROL_REGISTER                                 0xEE

/*
 * 8th bit set, read rssi for access errors also.
 * 9th bit set, assess rssi on all channels.
 * merged from BZ 2.1.2
 */
#define AFH_CH_BB_ASSESSMENT_CONTROL_REGISTER                 0x12C

/* Test control register - TEST_CONTROL_REGISTER 5:4 bits*/
#define ENABLE_BOTH_IDS                             0x00
#define ENABLE_FIRST_ID                             0x10
#define ENABLE_SECOND_ID                            0x20
#define ENABLE_NO_IDS                               0x30

#if defined(ENABLE_SCO) || defined (SCO_OVER_HCI) || defined (COMPILE_ESCO)
/* New registers for synchronous links */
#define SYNC_FIFO_CONFIG_REGISTER                    0xEE
#define SYNC_FIFO_CONFIG_REGISTER2                   0x9C
#define SYNC_RX_FIFO1_REGISTER                       0xF0
#define SYNC_RX_FIFO2_REGISTER                       0xA6
#define SYNC_TX_FIFO1_REGISTER                       0xF0
#define SYNC_TX_FIFO2_REGISTER                       0xA6

/* in 0380, HW has removed Host Buffer and uses FIFO 1 */
#define PCM_RECORD_FIFO                              SYNC_RX_FIFO1_REGISTER
#define PCM_PLAY_FIFO                                SYNC_TX_FIFO1_REGISTER

/* remove FIFO2 in 0380 */
#define SYNC_CONFIG_REGISTER(num)               SYNC_FIFO1_CONFIG_REGISTER
#define SYNC_TX_FIFO_BASE(num)                  SYNC_TX_FIFO1_REGISTER
#define SYNC_RX_FIFO_BASE(num)                  SYNC_RX_FIFO1_REGISTER

#endif /* (SCO_OVER_HCI || ENABLE_SCO || COMPILE_ESCO) */

/////////////////////////////////////////////////////////////

#define     RF_DELAY_H_REGISTER      0xC6
#define     RF_DELAY_F_REGISTER      0xC8
#define     RF_PHD_H_REGISTER        0xCA
#define     RF_PHD_F_REGISTER        0xCC
#define     CORR_DIS_H_REGISTER      0xCE
#define     TRIG_1STID_REGISTER      0xD0
#define     TRIG_RECFHS_REGISTER     0xD2
#define     TRIG_2NDID_REGISTER      0xD4

/////////////////////////////////////////////////////////////

/* BASEBAND CONTROLLER OPCODES (Please Sort the OpCode) */
#define      BB_NOP                                 0X00
#define      BB_INQUIRY                             0X01
#define      BB_INQUIRY_SCAN_FIRST_ID               0X02
#define      BB_PAGE                                0X03
#define      BB_PAGE_SCAN                           0X04
#define      BB_EXECUTE                             0X05
#define      BB_HOLD                                0X06
#define      BB_SNIFF                               0X07
#define      BB_START_BEACON                        0X08
#define      BB_UNPARK_REQUEST                      0X09
#define      BB_INQUIRY_SCAN_SECOND_ID              0X0A
#define      BB_START_SLEEP                         0X0B
#ifdef _RTL8821A_
#define      BB_INQUIRY_SCAN_3DD                    0X0B
#endif
#define      BB_FLUSH                               0X0C
#define      BB_SEND_PACKET                         0X0D
#define      BB_MASTER_SLAVE_SWITCH                 0X0E
#define      BB_LOAD_OFFSET                         0X0F

#define      BB_SWITCH_MASTER_PICONET               0X10
#define      BB_PRIORITY_SEND_PACKET                0x12
#define      BB_KILL_SEND_PACKET                    0x13
#ifdef _RTL8821A_
#define      BB_START_SYNC_3DD                      0x14
#define      BB_START_BEACON_3DD                    0x15
#endif
#ifdef _SUPPORT_CSB_RECEIVER_
#define      BB_START_SYNC_SCAN_3DD                 0x16
#define      BB_START_CSB_SCAN_3DD                  0x17
#endif
#define      BB_SWITCH_SLAVE_PICONET                0x18

#define      BB_SEQN_INIT                           0x21
#define      BB_SEND_FLOW_ZERO                      0x23
#define      BB_SEND_FLOW_GO                        0x24

#define      BB_START_TPOLL                         0x30
#define      BB_STOP_TPOLL                          0x31
#ifdef _NEW_MODEM_PSD_SCAN_
#define      BB_SET_PSD_EN                          0x40
#define      BB_CLR_PSD_EN                          0x41
#endif

#define      BB_RESET                               0X80
#define      BB_KILL_INQUIRY                        0X81
#define      BB_KILL_PAGE                           0X82
#define      BB_KILL_HOLD                           0X83
#define      BB_KILL_SNIFF                          0x84
#define      BB_KILL_SLEEP                          0x85
#define      BB_EXIT_BEACON                         0x86
#define      BB_KILL_BEACON                         0x87
#define      BB_ENTER_SNIFF_TRAN_MODE_DURING_EXIT_SNIFF_MASTER    0x89
#define      BB_ENTER_SNIFF_TRAN_MODE               0x89
#define      BB_EXIT_SNIFF_TRAN_MODE                0x8c
#ifdef _RTL8821A_
#ifdef _DAPE_TEST_NEW_HW_INSTRUCTION
#define      BB_KILL_INQUIRY_SCAN                   0X8E
#define      BB_KILL_PAGE_SCAN                      0X8F
#endif
#define      BB_KILL_SYNC_3DD                       0x90
#define      BB_KILL_BEACON_3DD                     0x91
#endif
#ifdef _SUPPORT_CSB_RECEIVER_
#define      BB_KILL_SYNC_SCAN_3DD                  0x92
#define      BB_KILL_CSB_SCAN_3DD                   0x93
#endif

/*  Baseband Packet Types defines for LUT */
#define      BB_NULL_LUT                          0x0000
#define      BB_POLL_LUT                          0x1000
#define      BB_FHS_LUT                           0x2000
#define      BB_DM1_LUT                           0x3000
#define      BB_DH1_LUT                           0x4000
#define      BB_HV1_LUT                           0x5000
#define      BB_HV2_LUT                           0x6000
#define      BB_HV3_LUT                           0x7000
#define      BB_DV_LUT                            0x8000
#define      BB_AUX1_LUT                          0x9000
#define      BB_DM3_LUT                           0xa000
#define      BB_DH3_LUT                           0xb000
#define      BB_DM5_LUT                           0xe000
#define      BB_DH5_LUT                           0xf000
#define      BB_ERR_PKT                           0xc000

#define      BB_2_DH1_LUT                         BB_DH1_LUT
#define      BB_3_DH1_LUT                         BB_DV_LUT
#define      BB_2_DH3_LUT                         BB_DM3_LUT
#define      BB_3_DH3_LUT                         BB_DH3_LUT
#define      BB_2_DH5_LUT                         0xE000
#define      BB_3_DH5_LUT                         0xF000


#define      BB_EV3_LUT                           BB_HV3_LUT
#define      BB_EV4_LUT                           0xC000
#define      BB_EV5_LUT                           0xD000

#define      BB_2_EV3_LUT                         BB_HV2_LUT
#define      BB_3_EV3_LUT                         BB_HV3_LUT
#define      BB_2_EV5_LUT                         BB_EV4_LUT
#define      BB_3_EV5_LUT                         BB_EV5_LUT

/*  Baseband Packet Types defines from Rx Status Register */
#define      BB_NULL                                 0x0
#define      BB_POLL                                 0x1
#define      BB_FHS                                  0x2
#define      BB_DM1                                  0x3
#define      BB_DH1                                  0x4
#define      BB_HV1                                  0x5
#define      BB_HV2                                  0x6
#define      BB_HV3                                  0x7
#define      BB_EV3                                  0x7
#define      BB_DV                                   0x8
#define      BB_AUX1                                 0x9
#define      BB_DM3                                  0xa
#define      BB_DH3                                  0xb
#define      BB_EV4                                  0xc
#define      BB_EV5                                  0xd
#define      BB_DM5                                  0xe
#define      BB_DH5                                  0xf

#define      BB_2_DH1                             BB_DH1
#define      BB_3_DH1                             BB_DV
#define      BB_2_DH3                             BB_DM3
#define      BB_3_DH3                             BB_DH3
#define      BB_2_DH5                             BB_DM5
#define      BB_3_DH5                             BB_DH5
#define      BB_2_EV3                             BB_HV2
#define      BB_3_EV3                             BB_HV3
#define      BB_2_EV5                             BB_EV4
#define      BB_3_EV5                             BB_EV5

/* Optional paging releted defines */
#define      BB_MANDATORY_PAGING_SCHEME           0x0000
#define      BB_OPTIONAL_PAGING_SCHEME_1          0x1000
#define      BB_OPTIONAL_PAGING_SCHEME_2          0x2000
#define      BB_OPTIONAL_PAGING_SCHEME_3          0x3000
#define      BB_MANDATORY_AND_PAGING_SCHEME_1     0x8000


/* Host packet types */
#define      HOST_DM1                             0x0008
#define      HOST_DH1                             0x0010
#define      HOST_HV1                             0x0020
#define      HOST_HV2                             0x0040
#define      HOST_HV3                             0x0080
#define      HOST_DM3                             0x0400
#define      HOST_DH3                             0x0800
#define      HOST_DM5                             0x4000
#define      HOST_DH5                             0x8000
#define      HOST_2_DH1                           0x0002
#define      HOST_3_DH1                           0x0004
#define      HOST_2_DH3                           0x0100
#define      HOST_3_DH3                           0x0200
#define      HOST_2_DH5                           0x1000
#define      HOST_3_DH5                           0x2000

#define ALL_FEC_PACKETS                           (HOST_DM1 | HOST_DM3 | HOST_DM5)
#define ALL_BR_ONE_SLOT_PACKETS                   (HOST_DM1 | HOST_DH1)
#define ALL_BR_THREE_SLOT_PACKETS                 (HOST_DM3 | HOST_DH3)
#define ALL_BR_FIVE_SLOT_PACKETS                  (HOST_DM5 | HOST_DH5)
#define ALL_BR_PACKETS                            (HOST_DM1 | HOST_DM3 | HOST_DM5 | HOST_DH1 | HOST_DH3 | HOST_DH5)

#define ALL_EDR_2MBPS_PACKETS                     (HOST_2_DH1 | HOST_2_DH3 | HOST_2_DH5)
#define ALL_EDR_3MBPS_PACKETS                     (HOST_3_DH1 | HOST_3_DH3 | HOST_3_DH5)
#define ALL_EDR_ONE_SLOT_PACKETS                  (HOST_2_DH1 | HOST_3_DH1)
#define ALL_EDR_THREE_SLOT_PACKETS                (HOST_2_DH3 | HOST_3_DH3)
#define ALL_EDR_FIVE_SLOT_PACKETS                 (HOST_2_DH5 | HOST_3_DH5)
#define ALL_EDR_PACKETS                           (ALL_EDR_2MBPS_PACKETS | ALL_EDR_3MBPS_PACKETS)

/* Flow Bit defines */
#define      L2CAP_FLOW_GO                        0x0020
#define      L2CAP_FLOW_STOP                      0x0000


/* Power control defines */
#ifdef POWER_SAVE_FEATURE
#define BB_PROGRAM_SM_MODE                        0x0001
#define BB_PROGRAM_DSM_MODE                       0x0002
#endif

/* L_CH values */
#define      BB_L_CH_UNDEFINED                    0x0000
#define      L_CH_L2CAP_NON_FLUSH                    0x0
#define      L_CH_L2CAP_START                        0x2
#define      L_CH_L2CAP_CONT                         0x1
#define      L_CH_LMP                                0x3
#define      L_CH_PBD                                0x2

/*  Baseband Logical channel defines for LUT*/
#define      BB_L2CAP_START_PKT                   (L_CH_L2CAP_START << 10)
#define      BB_L2CAP_CONTI_PKT                   (L_CH_L2CAP_CONT << 10)
#define      BB_LMP_PKT                           (L_CH_LMP << 10)
#define      BB_PBD_PKT                           (L_CH_PBD << 10)

#define      PKT_LENGTH_MASK                      0x03FF
#define      PKT_TYPE_MASK                        0x00F0
#define      L_CH_MASK                            0x3000
#define      L_FLOW_BIT                           0x4000
#define      AM_ADDR_MASK                         0x3800
#define      PTT_BIT_MASK                         0x8000

/* Baseband Status Definitions for Mode Status Register */
#define      BB_INQUIRY_STATUS                       0x1
#define      BB_INQUIRY_SCAN_STATUS                  0x2
#define      BB_PAGE_STATUS                          0x4
#define      BB_PAGE_SCAN_STATUS                     0x8
#define      BB_CONN_STATUS                         0x10
#define      BB_INQUIRY_RESP_STATUS                 0x20
#define      BB_PAGE_RESP_STATUS                    0x40

#ifdef _NEW_MODEM_PSD_SCAN_
#define      BB_IMODE_PSD_END                     0x0008
#endif
#ifdef _SUPPORT_CSB_RECEIVER_
#define      BB_IMODE_TRUNCATED_PAGE_DONE         0x0004
#endif
#ifdef _SUPPORT_PCA_ADJUST
#define      BB_IMODE_CLK_ADJ_INSTANT             0x0002
#endif
#define      BB_PARK_MODE_IND                     0x0010
#define      BB_START_BEACON_IND                  0x0400
#define      BB_START_ACCESS_WINDOW_IND           0x0800
#define      BB_END_ACCESS_WINDOW_IND             0x1000

#define      BB_ID_TRANSMIT_STATUS                0x0100

#define      BB_ACCESS_CODE_MISS_STATUS           0x1000

#define      BB_ITO_PICONET_ID_MASK               0x8000

/*  Baseband Interrupt defines */
#define      BB_ITO_INQUIRY_TIMEOUT               0x0001
#define      BB_ITO_PAGE_TIMEOUT                  0x0004

#define      BB_PAGERESP_TIMEOUT                  0x0008
#define      BB_ITO_UNPARK_REQ                    0x0020
#define      BB_ITO_NC_TIMEOUT                    0x0002
#define      BB_ITO_NBC_TIMEOUT                   0x0010
#define      BB_ITO_HOLD_START_TIMEOUT            0x0100
#define      BB_ITO_HOLD_END_TIMEOUT              0x0200
#define      BB_ITO_MSS_SUCCESS                   0x0040
#define      BB_ITO_PARK_NCTO                     0x0080
#define      BB_RANDOM_START_TIMEOUT              0x4000
#define      BB_HOLD1_TIMEOUT                     0X0100
#define      BB_HOLD2_TIMEOUT                     0X0200
#define      BB_HOLD3_TIMEOUT                     0X0400
#define      BB_HOLD4_TIMEOUT                     0X0800

#define      BB_HOLD5_TIMEOUT                     0x1000
#define      BB_HOLD6_TIMEOUT                     0x2000
#define      BB_HOLD7_TIMEOUT                     0x4000
#define      BB_PKT_RX                              0x01
#define      BB_PKT_TX                              0x01
#define      BB_ID_PKT_RX_INDICATION              0x4000
#define      BB_EIR_RX_INDICATION                 0x0004
#define      BB_ERROR                             0x000E

/* MSS_PICONET_ID bit is set in connector-register if the old-piconet-id and
 * new piconet ID are different. If they are same, then this bit has to be
 * cleared. This bit needs to be set beofre MSS is started at BB level.
 */
#define      MSS_PICONET_ID                       0x8000

/*
 * Common for CRC,FEC and HEC error,Action taken for any of the errors is same.
**/

#define      BB_PKT_TYPE                          0X00F0
#define      BB_ARQN_BIT                          0X0200
#define      BB_SEQN_BIT                          0X0100
#define      BB_AM_ADDR                           0X3800
#define      FHS_PKT_LEN                              18
#define      BB_FLOW_BIT                          0x1

/* Initial SEQN, after HLC, successful MSS, or unpark.
 * Remote SEQN is initialized to 2, to accept any SEQN for the first
 * CRC packet. Though spec mandates that the SEQN of first CRC packet
 * has to be 1, some devices that have v1.1 implementation violate this.
 * Initial value of 2 will ensure interoperability with them. */
#define      BB_REMOTE_INIT_SEQN_VAL                   2
#define      BB_LOCAL_INIT_SEQN_VAL                    1


/* BB related codec params. */

#define LMP_U_LAW_LOG               0
#define LMP_A_LAW_LOG               1
#define LMP_CVSD                    2
#define LMP_TRANSPARENT_DATA        3
#define INPUT_DATA_FORMAT_MASK        0xC0

#define LMP_SCO_AIR_MODE                             CVSD

#define VOICE_SETTINGS_U_LAW_MASK  (0x0061)
#define VOICE_SETTINGS_A_LAW_MASK  (0x0062)
#define VOICE_SETTINGS_CVSD_MASK   (0x0060)
#define VOICE_SETTINGS_TRANSPARENT_DATA_MASK   (0x0063)

#define AIR_MODE_MASK                 0x03
#define INPUT_CODING_MASK             0X300
#define INPUT_CODING_LINEAR           0X00
#define INPUT_CODING_MU_LAW           0x100
#define INPUT_CODING_A_LAW            0x200
#define RESERVED_FOR_FUTURE           0x300

#define AIR_MU_LAW_INPUT_LINEAR               0X0
#define AIR_MU_LAW_INPUT_MU_LAW               0X08
#define AIR_MU_LAW_INPUT_A_LAW                0X30

#define AIR_A_LAW_INPUT_LINEAR                0X04
#define AIR_A_LAW_INPUT_MU_LAW                0X20
#define AIR_A_LAW_INPUT_A_LAW                 0X08

#define AIR_TRANSPARENT_LAW_INPUT_LINEAR      0X08
#define AIR_TRANSPARENT_LAW_INPUT_MU_LAW      0X08
#define AIR_TRANSPARENT_LAW_INPUT_A_LAW       0X08




#define AIR_CODING_CVSD               0X00
#define AIR_CODING_MU_LAW             0X01
#define AIR_CODING_A_LAW              0X02
#define AIR_CODING_TRANSPARENT        0X03

#define OTHER_CODEC_PARAMS_MASK			0xF807
#define TX_CODEC_CONVERSION				6
#define RX_CODEC_CONVERSION				3

#define TX                              1
#define RX                              2

/* SCO_OVER_HCI */
#define MAX_SYNC_FIFO                                   2

#define SYNC_FIFO1_CONFIG_MASK               0x07
#define SYNC_FIFO2_CONFIG_MASK               SYNC_FIFO1_CONFIG_MASK
#define SYNC_FIFO1_OVER_HCI                  0x07
#define SYNC_FIFO2_OVER_HCI                  SYNC_FIFO1_OVER_HCI
#define SYNC_FIFO1_USING_PCM                 0x05
#define SYNC_FIFO2_USING_PCM                 SYNC_FIFO1_USING_PCM
#define SYNC_FIFO1_USING_UDA                 0x01
#define SYNC_FIFO2_USING_UDA                 SYNC_FIFO1_USING_UDA
#define SYNC_FIFO_CONFIG_REG_RESET_VALUE     0x00

/**
 * \addtogroup enc_control_opcodes Encryption Control Opcodes
 * Defines the possible control opcodes for controlling the BB level
 * encryption.
 * @{ */

/** Both the transmission and the reception are unencrypted */
#define BB_ENC_TX_DISBALE_RX_DISABLE    0
/** Transmission is unencrypted and the Reception is encrypted */
#define BB_ENC_TX_DISABLE_RX_ENABLE     1
/** Transmission is encrypted and the Reception is unencrypted */
#define BB_ENC_TX_ENABLE_RX_DISABLE     2
/** Both the transmission and the reception are encrypted */
#define BB_ENC_TX_ENABLE_RX_ENABLE      3

/** @} end: enc_control_opcodes */

#define SCA_PICONET_FIRST                                  0x00
#define SCA_PICONET_SECOND                                 0x01
#define SCA_PICONET_SECOND_SLAVE                           0x02
#define SCA_PICONET_THIRD                                  0x02
#define SCA_PICONET_FOURTH                                 0x03
#define SCA_PICONET_MAX                                    SCA_PICONET_FOURTH
#define SCA_PICONET_INVALID                                0xFF

#define BB_read_baseband_register(bb_reg_offset) \
	((UINT16)*((volatile UINT16*)(BB_BASE_ADDR+bb_reg_offset)))

/* the MACRO for Connector Register Configuration */
#define SET_BZ_CONN_LT_ADDR(addr, pid)  ((((addr)&0x07)<<5)|(((pid)&0x03)<<11))

/*===========================================================*/
/*  The Data Structure of Bluewiz HW Register                */
/*===========================================================*/
/* The Structure of CONNECTOR_REGISTER (0x02) - RW */
typedef struct BZ_REG_S_CONNECTOR_ {
    UINT16 sco_esco_index:3;  /* bit[2:0], sco index (bitmap) or esco index */
    UINT16 acl_or_sco_link:1; /* bit[3], TRUE: create an ACL link; FALSE: create
                                        an SCO link */
    UINT16 make_or_kill:1;    /* bit[4], TRUE mean make and FALSE mean kill */
    UINT16 lt_addr:3;         /* bit[7:5], the logical transport address */
    UINT16 tdd:1;             /* bit[8], used during role switch. FW sets this
                                 bit to initiate role switch at the role switch
                                 instant. The bit must be cleared by FW after
                                 role switch failure/success */
    UINT16 esco:1;            /* bit[9], this bit is set by FW to create/delete
                                         an eSCO connection */
    UINT16 codec_link:1;      /* bit[10], this link is over internel Codec */
    UINT16 piconet_id:2;      /* bit[12:11], the piconet index */
    UINT16 rsvd:1;            /* bit[13], reserved */
    UINT16 mss_pid1:1;        /* bit[14], role switch occur in piconet 1 */
    UINT16 mss_pid0:1;        /* bit[15], role switch occur in piconet 0 */
} BZ_REG_S_CONNECTOR, *PBZ_REG_S_CONNECTOR;

/* The Structure of RECEIVED_PAYLOAD_HEADER_REGISTER (0x04) - RC */
typedef struct BZ_REG_S_RX_PL_HDR_ {
    UINT16 len:10;      /* bit[9:0], the payload length */
    UINT16 pid_h:1;     /* bit[10], indicate which piconet (bit1) of rx pkt */
#ifdef _CCH_SC_ECDH_P256_
    UINT16 mic_err:1;   /* bit[11], secure connection with mic error */
#else
    UINT16 rsvd:1;      /* bit[11], reserved */
#endif
    UINT16 llid:2;      /* bit[13:12], the logical link id */
    UINT16 flow:1;      /* bit[14], the FLOW bit of payload header from rx pkt.
                                    it controls the L2CAP channel.
                                    flow=1 means GO, flow=0 means STOP */
    UINT16 sco_rx:1;    /* bit[15], is it sco rx pkt ? */
} BZ_REG_S_RX_PL_HDR, *PBZ_REG_S_RX_PL_HDR;

/* The Structure of RECEIVED_STATUS_REGISTER (0x06) - RC */
typedef struct BZ_REG_S_RX_STATUS_ {
    UINT16 rx_pkt:1;    /* bit[0], is this a rx pkt ? */
    UINT16 hec_err:1;   /* bit[1], HEC error ? */
    UINT16 eir:1;       /* bit[2], is this pkt an EIR ? */
    UINT16 crc_err:1;   /* bit[3], CRC error ? */
    UINT16 pkt_type:4;  /* bit[7:4], the pkt type of baseband */
    UINT16 seqn:1;      /* bit[8], the SEQN bit of packet header from rx pkt */
    UINT16 arqn:1;      /* bit[9], the ARQN bit of packet header from rx pkt */
    UINT16 flow:1;      /* bit[10], the FLOW bit of packet header from rx pkt */
    UINT16 lt_addr:3;   /* bit[13:11], the LT_ADDR field of packet header
                                       from rx pkt */
    UINT16 ar_id:1;     /* bit[14], is this an ID pkt
                                    (only role switch or parked slave) ? */
    UINT16 pid_l:1;     /* bit[15], indicate which piconet (bit0) of rx pkt */
} BZ_REG_S_RX_STATUS, *PBZ_REG_S_RX_STATUS;

/* The Structure of TRANSMIT_STATUS_REGISTER (0x08) - RC */
typedef struct BZ_REG_S_TX_STATUS_ {
    UINT16 tx_pkt:1;    /* bit[0], is this a tx pkt ? */
    UINT16 lt_addr:3;   /* bit[3:1], the LT_ADDR field of packet header
                                     from tx pkt */
    UINT16 pkt_type:4;  /* bit[7:4], sent pkt type of baseband */
    UINT16 tx_id:1;     /* bit[8], transmit ID pkt ? */
    UINT16 pid_l:1;     /* bit[9], indicate which piconet (bit0) of rx pkt */
    UINT16 sco_slot:1;  /* bit[10], current is sco tx slot ? */
    UINT16 pid_h:1;     /* bit[11], indicate which piconet (bit1) of rx pkt */
#ifdef _NEW_3DD_HW_
    UINT16 Is3ddBea:1;  /* bit[12], indicate 3dd beacon or not */
    UINT16 rsvd:3;      /* bit[15:11], reserved */
#else
    UINT16 rsvd:4;      /* bit[15:12], reserved */
#endif
} BZ_REG_S_TX_STATUS, *PBZ_REG_S_TX_STATUS;

/* The Structure of PAGE_SCAN_WINDOW_REGISTER (0x32) - RW */
typedef struct BZ_REG_S_PAGE_SCAN_WINDOW_ {
    UINT16 page_scan_window:12; /* bit[11:0], page scan window size */
    UINT16 override_bb_scan:1;  /* bit[12], debug purpose */
    UINT16 ps_int_en:1;         /* bit[13], page scan start interrupt enable*/
    UINT16 ps_timer_ctrl:1;     /* bit[14], page scan control flag */
    UINT16 ps_timer_run:1;      /* bit[15], page scan timer run flag */
} BZ_REG_S_PAGE_SCAN_WINDOW, *PBZ_REG_S_PAGE_SCAN_WINDOW;

/* The Structure of INQUIRY_SCAN_WINDOW_REGISTER (0x36) - RW */
typedef struct BZ_REG_S_INQ_SCAN_WINDOW_ {
    UINT16 inq_scan_window:12; /* bit[11:0], inquiry scan window size */
    UINT16 scan_type:1;        /* bit[12], 1/0 interlaced /normal scan type */
    UINT16 is_int_en:1;        /* bit[13], inquiry scan start interrupt enable*/
    UINT16 scan_incr_en:1;     /* bit[14], scan incremental enable */
    UINT16 reload_en:1;        /* bit[15], reload enable */
} BZ_REG_S_INQ_SCAN_WINDOW, *PBZ_REG_S_INQ_SCAN_WINDOW;

/* The Structure of LUT_LOWER_REGISTER (0x70+4*k) */
typedef struct BZ_REG_S_LUT_LOWER_ {
    UINT16 len:10;    /* bit[9:0], the length field of payload header */
    UINT16 llid:2;    /* bit[11:10], the LLID filed of payload header */
    UINT16 tx_type:4; /* bit[15:12], the packet type field of packet header */
} BZ_REG_S_LUT_LOWER, *PBZ_REG_S_LUT_LOWER;

/* The Structure of LUT_LOWER_REGISTER (0x72+4*k) */
typedef struct BZ_REG_S_LUT_UPPER_ {
    UINT16 pkt_flow:1;  /* bit[0], FLOW bit of the packet header */
    UINT16 seqn:1;      /* bit[1], SEQN bit of packet header */
    UINT16 arqn:1;      /* bit[2], AEQN bit of packet header */
    UINT16 in_sniff:1;  /* bit[3], in sniff mode ? */
    UINT16 in_hold:1;   /* bit[4], in hold mode ? */
    UINT16 pl_flow:1;   /* bit[5], FLOW bit of the payload header */
    UINT16 sco2:1;      /* bit[6], active SCO connection and assigned ID 2 */
    UINT16 sco1:1;      /* bit[7], active SCO connection and assigned ID 1 */
    UINT16 active:1;    /* bit[8], the link is active ? */
    UINT16 tx_gain_or_esco_prim_addr:3; /* bit[11:9], Tx power for the tx pkt
                                           and store primary address of eSCO
                                           link */
    UINT16 sniff_end:1; /* bit[12], indicateds that sniff has ended */
    UINT16 sinff_start:1; /* bit[13], indicateds that sniff has started */
    UINT16 esco:1;      /* bit[14], active eSCO connection */
    UINT16 ptt:1;       /* bit[15], indicates if EDR is enabled for the link */
} BZ_REG_S_LUT_UPPER, *PBZ_REG_S_LUT_UPPER;

/* The Structure of PN_INFO_REGISTER (0x108/0x10A/0x146/0x148) */
typedef struct BZ_REG_S_PICONET_INFO_ {
    UINT16 master:1;        /* bit[0], master role in current piconet ? */
    UINT16 lt_addr:3;       /* bit[3:1], the LT address (this value is used
                               only when device is a slave in the piconet */
    UINT16 retx_count_en:1; /* bit[4], enable retransmission counter */
    UINT16 park_config:2;   /* bit[6:5], 10B:Parked Master 01B:Parked Slave */
    UINT16 role_switch:1;   /* bit[7], role switch flag ? */
    UINT16 rsvd:8;          /* bit[15:8], reserved */
} BZ_REG_S_PICONET_INFO, *PBZ_REG_S_PICONET_INFO;

///aust
#ifndef _DAPE_TEST_NEW_HW
/* The Structure of PRI_CTRL_REGISTER (0x11C) */
typedef struct BZ_REG_S_PRI_CTRL_ {
    UINT16 inq_page_rsp_high_than_sco:1; /* bit[0], the priority of inquiry and
                                            page response is higher than sco */
    UINT16 tpoll_high_than_sco_pn0:1;    /* bit[1], the priority of tpoll is
                                            higher than sco in piconet 0 */
    UINT16 tpoll_high_than_sco_pn1:1;    /* bit[2], the priority of tpoll is
                                            higher than sco in piconet 1 */
    UINT16 en_scan_timer:1;              /* bit[3], enable scan timer ? */
    UINT16 en_esco_force_nak_pn0:1;      /* bit[4], if no receive pkt, force
                                            hw send nak on esco link in pn0 */
    UINT16 en_esco_force_nak_pn1:1;      /* bit[5], if no receive pkt, force
                                            hw send nak on esco link in pn1 */
    UINT16 page_high_than_acl:1;         /* bit[6], the priority of paging is
                                            higher than acl */
    UINT16 pause_sco:1;                  /* bit[7], pause sco tx traffic ? */
    UINT16 rsvd1:1;                      /* bit[8], reserved */
    UINT16 page_high_than_acl_def:1;     /* bit[9], the priority of paging is
                                            higher than acl in default state */
    UINT16 scan_inquiry_high_than_acl:1; /* bit[10], the priority of scan and
                                            inquiry is higher than acl taffic */
    UINT16 block_acl_for_esco:1;         /* bit[11], when esco reserve and acl collision, block acl */
    UINT16 lpm_preempt_pn0:1;            /* bit[12], if we have lower power mode
                                            traffic in any piconet, it can
                                            preempt to schedule from pn 0 */
    UINT16 lpm_preempt_pn1:1;            /* bit[13], if we have lower power mode
                                            traffic in any piconet, it can
                                            preempt to schedule from pn 1 */
    UINT16 tpoll_lpm_pri_pn0:1;          /* bit[14], if we have lower power mode
                                            traffic in any piconet, hw will be
                                            switch that piconet from pn 0 */
    UINT16 tpoll_lpm_pri_pn1:1;          /* bit[15], if we have lower power mode
                                            traffic in any piconet, hw will be
                                            switch that piconet from pn 1 */
} BZ_REG_S_PRI_CTRL, *PBZ_REG_S_PRI_CTRL;
#else
/* The Structure of PRI_CTRL_REGISTER (0x11C) */
typedef struct BZ_REG_S_PRI_CTRL_ {
    UINT16 page_rsp_high_than_sco:1;     /* bit[0], the priority of
                                            page response is higher than sco */
    UINT16 tpoll_high_than_sco_pn0:1;    /* bit[1], the priority of tpoll is
                                            higher than sco in piconet 0 */
    UINT16 tpoll_high_than_sco_pn1:1;    /* bit[2], the priority of tpoll is
                                            higher than sco in piconet 1 */
    UINT16 en_scan_timer:1;              /* bit[3], enable scan timer ? */
    UINT16 en_esco_force_nak_pn0:1;      /* bit[4], if no receive pkt, force
                                            hw send nak on esco link in pn0 */
    UINT16 en_esco_force_nak_pn1:1;      /* bit[5], if no receive pkt, force
                                            hw send nak on esco link in pn1 */
    UINT16 page_high_than_acl:1;         /* bit[6], the priority of paging is
                                            higher than acl */
    UINT16 pause_sco:1;                  /* bit[7], pause sco tx traffic ? */
    UINT16 rsvd1:1;                      /* bit[8], reserved */
    UINT16 page_high_than_acl_def:1;     /* bit[9], the priority of paging is
                                            higher than acl in default state */
    UINT16 scan_inquiry_high_than_acl:1; /* bit[10], the priority of scan and
                                            inquiry is higher than acl taffic */
///// dape modified
//    UINT16 rsvd2:1;                      /* bit[11], reserved */
    UINT16 block_acl_for_esco:1;         /* bit[11], when esco reserve and acl collision, block acl */
    UINT16 lpm_preempt_pn0:1;            /* bit[12], if we have lower power mode
                                            traffic in any piconet, it can
                                            preempt to schedule from pn 0 */
    UINT16 lpm_preempt_pn1:1;            /* bit[13], if we have lower power mode
                                            traffic in any piconet, it can
                                            preempt to schedule from pn 1 */
    UINT16 tpoll_lpm_pri_pn0:1;          /* bit[14], if we have lower power mode
                                            traffic in any piconet, hw will be
                                            switch that piconet from pn 0 */
    UINT16 tpoll_lpm_pri_pn1:1;          /* bit[15], if we have lower power mode
                                            traffic in any piconet, hw will be
                                            switch that piconet from pn 1 */
} BZ_REG_S_PRI_CTRL, *PBZ_REG_S_PRI_CTRL;

#endif
/* The Structure of PRI_CTRL_REGISTER (0x11E) */
typedef struct BZ_REG_S_PRI_CTRL1_ {
    UINT16 acl_pause:3;                  /* bit[2:0], reserve how many slots
                                            to pause acl tx traffic early before
                                            next anchor point will come in
                                            sniffer mode
                                            Please notice, if acl_pause = n, the hardware
                                            will block (n + 1) slots to protect the anchor point. */
    UINT16 slave_crc_pri:1;              /* bit[3], In slave role, when crc pkt
                                            is prepare in another piconet, we
                                            will move to that piconet after we
                                            finish this rxtx traffic */
    UINT16 mask_txrx_timing:1;           /* bit[4], set 0 to stop txrx timing */
    UINT16 en_esco_force_nak_pn2:1;      /* bit[5], if no receive pkt, force
                                            hw send nak on esco link in pn2 */
    UINT16 en_esco_force_nak_pn3:1;      /* bit[6], if no receive pkt, force
                                            hw send nak on esco link in pn3 */
    UINT16 rsvd1:1;                      /* bit[7], reserved */
    UINT16 lpm_preempt_pn2:1;            /* bit[8], if we have lower power mode
                                            traffic in any piconet, it can
                                            preempt to schedule from pn 2 */
    UINT16 lpm_preempt_pn3:1;            /* bit[9], if we have lower power mode
                                            traffic in any piconet, it can
                                            preempt to schedule from pn 3 */
    UINT16 tpoll_lpm_pri_pn2:1;          /* bit[10], if we have lower power mode
                                            traffic in any piconet, hw will be
                                            switch that piconet from pn 2 */
    UINT16 tpoll_lpm_pri_pn3:1;          /* bit[11], if we have lower power mode
                                            traffic in any piconet, hw will be
                                            switch that piconet from pn 3 */
    UINT16 tpoll_high_than_sco_pn2:1;    /* bit[12], the priority of tpoll is
                                            higher than sco in piconet 2 */
    UINT16 tpoll_high_than_sco_pn3:1;    /* bit[13], the priority of tpoll is
                                            higher than sco in piconet 3 */
    UINT16 legacy_high_than_le:1;        /* bit[14], the priority of legacy is
                                            higher than LE */
    UINT16 sco_sup_le:1;                      /* bit[15], sco_sup_le */
} BZ_REG_S_PRI_CTRL1, *PBZ_REG_S_PRI_CTRL1;

#ifdef _DAPE_TEST_NEW_HW
/* The Structure of PRI_CTRL2_REGISTER (0x120) */
typedef struct BZ_REG_S_PRI_CTRL2_ {
    UINT16 le_adv_pri:1;              /* bit[0], the priority of LE unconnected
                                                 and legacy unconnected states.
                                                 1: LE priority higher than
                                                 inquiry/page. */
    UINT16 pause_esco:1;              /* bit[1], pause esco */
    UINT16 inquiry_acl_pri:1;         /* bit[2], the priority of inquiry and acl.
	                                             1: inquiry priority higher than
	                                             acl. */
    UINT16 inq_rsp_high_than_sco:1;   /* bit[3],  the priority of
                                                 inq response is higher than sco */
    UINT16 inq_high_than_acl_def:1;   /* bit[4], the priority of inquiry is
                                                 higher than acl in default state */
    UINT16 le_conn_high_than_page_inq:1;      /* bit[5],  the priority of
                                                 le-connected state is higher than page/inq */
    UINT16 le_6_interval_lower:1;               /* bit[6], le 6 interval lower. 0:le higher. */
#ifdef _DAPE_EN_8821_MP_LE_SCAN_INTR
    UINT16 block_legacy_acl:1;                   /* bit[7], pause legacy acl scheduler */
#else
    UINT16 rsvd1:1;                   /* bit[7], reserved */
#endif
    UINT16 rsvd2:8;                   /* bit[15:8], reserved */
} BZ_REG_S_PRI_CTRL2, *PBZ_REG_S_PRI_CTRL2;

/* The Structure of CORR_DIS_H_REGISTER (0xCE) */
typedef struct BZ_REG_S_CORR_DIS_H_ {
    UINT16 disable_value:9;              /*  bit[8:0], disable value */
    UINT16 load_value_conn8_7:2;         /* bit[10:9], the time when slave corrects the timing,
                                                       shoud be set to the value:
                                                       (load_value2-7). */
    UINT16 corr_disable_f_4_0:5;         /* bit[15:11], the time of rx_on,
                                                        load_value_conn[4:0].
                                                        the real value =
                                                        (340-corr_disable_f_4_0)
                                                        default:0.
                                         */
} BZ_REG_S_CORR_DIS_H, *PBZ_REG_S_CORR_DIS_H;

typedef struct BZ_REG_HW_INSTRUCTION_STATUS_ {
        UINT16 page:1;                     //[0]
        UINT16 inquiry:1;                  //[1]
        UINT16 page_scan:1;                //[2]
        UINT16 inq_scan2:1;                //[3]
        UINT16 inq_scan:1;                 //[4]
        UINT16 sniff:1;                    //[5]
        UINT16 tpoll:1;                    //[6]
        UINT16 park:1;                     //[7]
        UINT16 hold:1;                     //[8]
        UINT16 rsvd:7;                     //[15:9]
}BZ_REG_HW_INSTRUCTION_STATUS, *PBZ_REG_HW_INSTRUCTION_STATUS;
#endif
#ifdef _DAPE_NEW_HW_AFTER_8821MP
/* The Structure of OPT_REG_REGISTER (0x21C) */
typedef struct BZ_REG_S_OPT_REG_ {
    UINT16 esco_bugfix:1;        /*  bit[0], 0:enable the bug fixed. */
    UINT16 bug_fixed1:1;         /*  bit[1], 1:enable the bug_fixed1.*/
    UINT16 bug_fix2:1;           /*  bit[2], 0:enable the bug_fix2. */
    UINT16 bug_fixed3:1;         /*  bit[3], 1:enable the bug_fixed3.*/
    UINT16 bcn_blk_func:1;       /*  bit[4], 0:disable. 1:enable.*/
    UINT16 clr_crc_opt:1;        /*  bit[5], 0: Clear crc_frame_pnx signal evey time received
                                                unicast packet
                                             1: Do not clear crc_frame_pnx signal
                                             evey time received  unicast packet.
                                             ( the original one)*/
    UINT16 slv_sniff_opt:1;      /*  bit[6], 1: slave sniff state machine reset
                                    timeout counter as acl packet received.
                                     0: slave sniff state machine reset timeout
                                     counter as received any packet.*/
    UINT16 slv_blk_tx:1;         /*  bit[7], 0: allow block tx as transaction pair
                                             1 : not allow. */
    UINT16 beacon_block_time:4;  /* bit[11:8], 0: 1 slot pair.1: 2 slot pairs....*/
#ifdef _DAPE_HW_SINCE_8821B_MP
    UINT16 rsvd:2;                 /* bit[12:13]*/
    UINT16 en_3dd_clk_shift:1;     /* bit[14]*/
    UINT16 delay_esco_early_intr:1; /*bit[15]*/
#else
    UINT16 rsvd:4;               /* bit[15:12]*/
#endif
} BZ_REG_S_OPT_REG, *PBZ_REG_S_OPT_REG;
#endif

/* The Structure of MWS_CLKADJ_CTRL1 (0x26e)*/
typedef struct BZ_REG_S_MWS_CLKADJ_CTRL1_ {
    UINT16 clk_adj_val27_16:12;  /*  bit[11:0], clk_adj_val[27:16]. */
    UINT16 clk_adj_pn:2;         /*  bit[13:12], piconet of MWS frame sync or PCA. */
    UINT16 rsvd:1;               /*  bit[14], reserved */
    UINT16 enable_clk_adj:1;     /*  bit[15], 1:enable clk_adj*/
}BZ_REG_S_MWS_CLKADJ_CTRL1, *PBZ_REG_S_MWS_CLKADJ_CTRL1;

/* The Structure of MWS_CLKADJ_CTRL3 (0x272)*/
typedef struct BZ_REG_S_MWS_CLKADJ_CTRL3_ {
    UINT16 clk_adj_cnt9_0:10;     /*  bit[9:0], at clk_adj_instant, native counter will apply this value. */
    UINT16 clk_adj_inst21_16:6;   /*  bit[15:10], clk_adj_instant[21:16] */
}BZ_REG_S_MWS_CLKADJ_CTRL3, *PBZ_REG_S_MWS_CLKADJ_CTRL3;

/* The Structure of MWS_CLKADJ_CTRL4 (0x274)*/
typedef struct BZ_REG_S_MWS_CLKADJ_CTRL4_ {
    UINT16 sync_n_cnt9_0:10;     /*  bit[9:0], as mws frame sync arises, native counter will apply this value. */
    UINT16 sync_n_clk1_0:2;      /*  bit[11:10], piconet of MWS frame sync or PCA. */
    UINT16 rsvd:3;               /*  bit[14:12], reserved */
    UINT16 enable_clk_sync:1;    /*  bit[15], 1:enable frame sync. */
}BZ_REG_S_MWS_CLKADJ_CTRL4, *PBZ_REG_S_MWS_CLKADJ_CTRL4;
#if defined(_CCH_SNIFF_NEG_TIMEOUT_) || defined(_CCH_RETENTION_FLOW_FOR_DLPS_)
/* The Structure of EFUSE_RSVD_2_S (0x1E6) */
typedef struct EFUSE_RSVD_2_S_ {
    UINT16 sniff_neg_time:3;        /*  bit[2:0], 0:sniff_req times for accept rempote sniff parameter. */
    UINT16 enable_sniff_neg:1;      /*  bit[3],   1:enable for sniff use remote sniff parameters.*/
    UINT16 enable_deep_lps_mode:1;  /*  bit[4],   default 0, 1: enable hw power off LPS */
    UINT16 dlps_guard_time:3;       /*  bit[7:5]  default 0,  unit: slot (dlps total guard time = dlps_guard_time + dsm_guard_interval) */
    UINT16 le_link_lps_enable:1;    /*  bit[8],   default 0, to enable lps at le link mode(link or initiator). */
    UINT16 rsvd_for_sco_slot:1;     /*  bit[9] rsvd. */
    UINT16 lps_state_with_intr:1;   /*  bit[10], default 0, 1: enable lps state trigger by interrupt */
    UINT16 rsvd:5;                  /*  bit[15:11] rsvd. */
} EFUSE_RSVD_2_S;
#endif

/* Prototypes of Baseband Driver Functions exported to other modules */
void BB_write_baseband_register(UINT16 bb_reg_offset, UINT16 val);

/** Bluewiz register: CON_3DD_REG (0x208) */
typedef struct BZ_REG_CON_3DD_
{
    /** [7:0] Service data in CSB mode and 3DD version in RP mode. */
    UINT16 service_data : 8;
    UINT16 stream_mode : 1;
    UINT16 rsvd_am_addr : 3;
    UINT16 max_random_delay : 4;
} BZ_REG_CON_3DD;

#define BZ_REG_CON_3DD_MMIO BB_REG_ADDR(BB_3DD_CON_REG)

static inline BZ_REG_CON_3DD bz_reg_con_3dd_read()
{
    return RD_REG_MMIO(BZ_REG_CON_3DD, BZ_REG_CON_3DD_MMIO);
}

static inline void bz_reg_con_3dd_write(BZ_REG_CON_3DD reg)
{
    BB_write_baseband_register(BB_3DD_CON_REG, *((UINT16 *) &reg));
}

void BB_write_baseband_TX_FIFO(UCHAR *buffer, UINT16 length, UCHAR am_addr,
        UCHAR piconet_id);
void BB_write_baseband_TX_FIFO_aclu(UCHAR *buffer, UINT16 length,
        UCHAR am_addr, UCHAR piconet_id);

void BB_write_baseband_TX_FIFO_scatternet(UCHAR *buffer, UINT16 length,
                                  UCHAR am_addr, UCHAR lcl_which_fifo_to_use);

void BB_read_baseband_RX_FIFO(UCHAR * buf, UINT16 length, UINT8 wait_complete);
void BB_read_baseband_RX_FIFO_and_flush(UINT16 length, UINT8 wait_complete);

#if 0
void BB_write_baseband_SYNC_TX_FIFO(const UCHAR *buffer,
									UCHAR odd_compendate_char, UCHAR fifo_num, UINT16 length);
void BB_read_baseband_SYNC_RX_FIFO(UCHAR *buf, UCHAR fifo_num, UINT16 length);
#endif

void BB_flush_baseband_SYNC_FIFO(UCHAR fifo_num, UINT16 pkt_length,
								 UINT16 fifo_addr, int is_mrl);

void BB_write_baseband_register_upper_octet(UINT16 bb_reg_offset, UINT16 val);
void BB_write_baseband_register_lower_octet(UINT16 bb_reg_offset, UINT16 val);
void AND_val_with_bb_reg(UINT16 reg_offset, UINT16 val);
void AND_val_with_bb_reg_isr(UINT16 reg_offset, UINT16 val);
void OR_val_with_bb_reg(UINT16 reg_offset, UINT16 val);
void OR_val_with_bb_reg_isr(UINT16 reg_offset, UINT16 val);
void Update_val_with_bb_reg(UINT16 offset, UINT16 val, UINT16 mask);
void Update_val_with_bb_reg_isr(UINT16 offset, UINT16 val, UINT16 mask);

UINT16 BB_read_baseband_register_upper_octet(UCHAR bb_reg_offset);

/**
 * Flush \a pkt_length bytes of data from the Synchronous connection Tx Fifo
 * given by \a fifo_num.
 *
 * \param fifo_num Fifo number (SYNC_FIFO1/SYNC_FIFO2).
 * \param pkt_length Number of bytes to flush.
 *
 * \return None.
 */
#define BB_flush_baseband_SYNC_TX_FIFO(fifo_num, pkt_length)    \
        BB_flush_baseband_SYNC_FIFO(fifo_num, pkt_length, 0x00A0, 0)

/**
 * Flush \a pkt_length bytes of data from the Synchronous connection Rx Fifo
 * given by \a fifo_num.
 *
 * \param fifo_num Fifo number (SYNC_FIFO1/SYNC_FIFO2).
 * \param pkt_length Number of bytes to flush.
 *
 * \return None.
 */
#define BB_flush_baseband_SYNC_RX_FIFO(fifo_num, pkt_length)    \
        BB_flush_baseband_SYNC_FIFO(fifo_num, pkt_length, 0x00B0, 0)

/**
 * Flush \a pkt_length bytes of data from the Synchronous connection Tx Fifo
 * given by \a fifo_num. It flushes the Most Recently Loaded packet (flush
 * from Write Pointer).
 *
 * \param fifo_num Fifo number (SYNC_FIFO1/SYNC_FIFO2).
 * \param pkt_length Number of bytes to flush.
 *
 * \return None.
 */
#define BB_MRL_flush_baseband_SYNC_TX_FIFO(fifo_num, pkt_length) \
        BB_flush_baseband_SYNC_FIFO(fifo_num, pkt_length, 0x00A0, 1)
void    bb_write_inq_parity_bit(UINT16 val);

void bb_write_remote_parity_bit(UINT16 val, UCHAR piconet_id);

void    bb_write_local_parity_bit(UINT16 val);

UINT16 bb_new_program_codec(UCHAR air_mode,UINT16 input_coding,
                    UINT16 lmp_ce_index,UINT8 linear_format,UINT8 linear_16bit);

void BB_reset_variables(void);

void BB_start_tpoll(UCHAR am_addr, UINT16 t_poll, UCHAR piconet_id);
void BB_stop_tpoll(UCHAR am_addr, UCHAR piconet_id);
void BB_write_afh_map(UCHAR am_addr, UCHAR phy_piconet_id, UCHAR* afh_map);
void BB_disable_afh(UCHAR am_addr, UCHAR phy_piconet_id);
void BB_disable_NBC(UCHAR phy_piconet_id);
void BB_enable_NBC(UCHAR phy_piconet_id, UCHAR nbc_timeout,
    UCHAR enable_slot_based_nbc);

void BB_write_encryption_keys(UCHAR am_addr, UCHAR phy_piconet_id,
                                                    UCHAR* key_dash);

//#ifdef _CCH_SC_ECDH_P256_
#define CAM_ADDR_REG                    0x236
#define CAM_DATA_REG0                   0x238
#define CAM_DATA_REG1                   0x23A

#ifdef _CCH_AFH_E0_AES_IN_CAM_

// 11 LUT from 1~11
#define CAM_ADDR_OFFSET_LAYER1          0
#define CAM_ADDR_OFFSET_LAYER1_SIZE     12
#define CAM_ADDR_OFFSET_IV              1
#define CAM_ADDR_OFFSET_AES_KEY         3
#define CAM_ADDR_OFFSET_ENC_TX_CNT      7
#define CAM_ADDR_OFFSET_DEC_RX_CNT      9
#define CAM_ADDR_OFFSET_DAY_CNT         11

// 12 LUT from 0~11
#define CAM_ADDR_OFFSET_LAYER2          132
#define CAM_ADDR_OFFSET_LAYER2_SIZE     7
#define CAM_ADDR_OFFSET_AFH             0    // Content Bitorder Reversed
#define CAM_ADDR_OFFSET_E0_KEY          3    // Content Bitorder Reversed


#define CAM_ADDR_OFFSET_SIZE            216
#endif

//#endif


#ifdef _CCH_SC_ECDH_P256_
void BB_write_sc_cam_ini(UINT16 ce_index);
void BB_write_sc_encryption_keys(UINT16 ce_index, UCHAR* key_dash);
void BB_write_sc_iv(UINT16 ce_index, UCHAR* iv);
void BB_write_sc_encry_clear(UINT16 ce_index, UCHAR opcode);
void BB_read_sc_count(UINT16 ce_index, UINT32 *tx_msb, UINT32 *tx_lsb,
                      UINT32 *rx_msb, UINT32 *rx_lsb, UINT32 *daycounter);
void BB_write_sc_esco_first_flag(UINT16 ce_index, UINT32 set_val);
void BB_read_sc_esco_daycounter(UINT16 ce_index, UINT32 *read);
void BB_write_sc_rxpkcnt_ignore_seqcompare(UINT16 ce_index, UINT8 set_val);
void BB_write_sc_cam(UINT16 cam_addr, UINT32 wdata);
UINT32 BB_read_sc_cam(UINT16 cam_addr);
#endif

#ifdef _CCH_AFH_E0_AES_IN_CAM_
void BB_write_afh_in_cam(UINT16 ce_index, UCHAR *afh_map);
void BB_write_e0_key_in_cam(UINT16 ce_index, UCHAR *key_dash);
#endif


void BB_encryption_control(UCHAR am_addr, UCHAR phy_piconet_id, UCHAR opcode);

void bb_kill_sniff_in_scatternet(UCHAR am_addr, UCHAR phy_piconet_id, UCHAR ssr_flag);

#if 0
UINT32 bb_read_baseband_clock(UCHAR role);
#endif

void bb_flush_broadcast_fifo(void);
void BB_write_flow_stop(void);
void BB_write_flow_go(void);
INLINE UCHAR BB_get_current_flow(void);

/**
 * Sets Slave Full Bandwidth flag in baseband. When this flag is set,
 * LUT_PACKET_TYPE (other than NULL) will be given more priority over
 * sco instants. It is applicable only for the Slave. As the name suggests,
 * it has to be set only when full bandwidth is occupied
 * (when lc_acl_full_bandwidth_flag = TRUE).
 *
 * \param None.
 *
 * \return None.
 */
#define BB_set_slave_full_bandwidth_flag()   \
    OR_val_with_bb_reg(SYNC_FIFO_CONFIG_REGISTER, BIT10)

/**
 * Clears Slave Full Bandwidth flag in baseband.
 *
 * \param None.
 *
 * \return None.
 */
#define BB_clear_slave_full_bandwidth_flag() \
    AND_val_with_bb_reg(SYNC_FIFO_CONFIG_REGISTER, (UINT16)(~BIT10))

/**
 * Sets Single Slave Full Bandwidth flag in baseband. When this flag is set,
 * BB_SEND_PACKET is given higher priority over sco instants. It means that
 * BB_SEND_PACKET will override sco instants. It is applicable only for the
 * master. As the name suggests, it has to be set only when full bandwidth is
 * occupied and the master (our device) has single slave.
 *
 * \param None.
 *
 * \return None.
 */
#define BB_set_single_slave_full_bandwidth_flag()   \
    OR_val_with_bb_reg(SCO_PACKET_TYPE_REGISTER, BIT10)

/**
 * Clears Single Slave Full Bandwidth flag in baseband.
 *
 * \param None.
 *
 * \return None.
 */
#define BB_clear_single_slave_full_bandwidth_flag() \
    AND_val_with_bb_reg(SCO_PACKET_TYPE_REGISTER, (UINT16)(~BIT10))

/**
 * Sets Multi Slave Full Bandwidth flag in baseband. When this flag is set,
 * BB_SEND_PACKET, Tpoll, and Tsniff are given higher priority over sco
 * instants. It means that BB_SEND_PACKET and periodic POLLs (interleaved with
 * Tpoll or Tsniff as duration) will override sco instants. It is applicable
 * only for the master. As the name suggests, it has to be set only when the
 * whole bandwidth is occupied and the master (our device) has more than one
 * slave.
 *
 * \param None.
 *
 * \return None.
 */
#define BB_set_multi_slave_full_bandwidth_flag()    \
    OR_val_with_bb_reg(SCO_PACKET_TYPE_REGISTER, BIT9)

/**
 * Clears Multi Slave Full Bandwidth flag in baseband.
 *
 * \param None.
 *
 * \return None.
 */
#define BB_clear_multi_slave_full_bandwidth_flag()  \
    AND_val_with_bb_reg(SCO_PACKET_TYPE_REGISTER, (UINT16)(~BIT9))

/**
 * Sets Sniff priority flag in baseband. When this flag is set,
 * Tsniff is given more priority than any other pkt.
 *
 * \param None.
 *
 * \return None.
 */
#define BB_set_sniff_priority_flag()    \
    OR_val_with_bb_reg(SCO_PACKET_TYPE_REGISTER, BIT11)

/**
 * Clears Sniff priority flag in baseband.
 *
 * \param None.
 *
 * \return None.
 */
#define BB_clear_sniff_priority_flag()  \
    AND_val_with_bb_reg(SCO_PACKET_TYPE_REGISTER, (UINT16)(~BIT11))

void bb_enable_eir(UCHAR enable);
void bb_write_eir_data(UINT16 length, UINT16 packet_type, UCHAR *data);
void bb_enable_eir_recv(UCHAR enable);
void BB_modify_xtol_in_scatternet(UCHAR val, UCHAR phy_piconet_id);

#ifdef _YL_PATCH_UTILITY
UCHAR BB_read_xtol_in_scatternet(UCHAR phy_piconet_id); /* added by yilinli */
UINT16 BB_read_native_counter(void);
void BB_write_native_counter(UINT16 value);
API_RESULT BB_native_counter_shift(INT16 shift_us, UINT16 shift_guard_time_us);
#endif
#ifdef _DAPE_TEST_PATCH_UTILITY
#if 0
UINT32 BB_read_debug_port_state(INT16 debug_port);
#endif
UINT16 BB_read_instruction_status(UINT16 ce_index);
#endif
void bb_pause_sco(UCHAR pause);
#ifdef _DAPE_TEST_NEW_HW_PAUSE_ESCO_WHEN_RETX
void bb_pause_esco(UCHAR pause);
#endif
//void bb_legacy_priority_higher_than_le_acl(UCHAR flag);

#if ((defined _DAPE_NO_TRX_WHEN_LE_SEND_CONN_REQ_HW_TIMER) || \
	(defined _DAPE_NO_TRX_WHEN_LE_CONN_UPDT) || \
	(defined _DAPE_EN_8821_MP_LE_SCAN_INTR))
void bb_switch_scheduler_to_legacy();
UINT8 bb_switch_scheduler_to_le();
#endif
UINT32 BB_read_native_clock(void);

#ifdef BZ_2_1_2_HID_FTP
void BB_update_sniff_max_slot();
#endif

#ifdef _BRUCE_FLUSH_STALED_PKT_IN_ESCO_LINK_OVER_HCI
void DMA_read_RXFIFO_and_flush_for_sync_link(UINT16 length, UINT8 wait_complete,UINT16 esco_ch);
#endif

#endif /* __BB_DRIVER_H__ */

/** @} end: bb_driver */
