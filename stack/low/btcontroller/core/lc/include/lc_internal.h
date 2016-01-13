/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  Contains the LC module internal API.
 */

/** \addtogroup lc_internal
 *   @{ */
#ifndef __H_LC_INTERNAL__
#define __H_LC_INTERNAL__
/* Main level header files */
#include "bt_fw_common.h"/* OS header files */
#include "bt_fw_os.h"
#include "bt_fw_types.h"
#include "dma_usb.h"

/* Module header files */
#include "lmp.h"
#include "lmp_defines.h"
#include "bt_fw_hci.h"
#include "hci_td.h"
#include "bb_driver.h"

#include "lc_1_2_internal.h"
#include "mint_os_queue_internal.h"

#include "bzdma.h"

#define MAX_NO_OF_LUT_INDICES                                    12

extern UCHAR failed_lut_index[MAX_NO_OF_LUT_INDICES];
extern UCHAR paused_lut_index[MAX_NO_OF_LUT_INDICES];
extern UCHAR flow_enabled_lut_index[MAX_NO_OF_LUT_INDICES];

/**
 * Radio types supported.
 */
typedef enum
{
    RT_SIW_RADIO = 0x00,            /**< Silicon Wave Radio */
    RT_MT_CUSTOM_RADIO_1,           /**< MT_CUSTOM_RADIO_1 Radio */
    RT_BLUERF_RADIO,                /**< BlueRF Radio */
#ifdef COMPILE_MT_CUSTOM_RADIO_2
    RT_MT_CUSTOM_RADIO_2,
#endif
    RT_INVALID_RADIO_TYPE = 0xff      /**< Invalid Radio Type */
} RADIO_TYPE;

#define PICONET1_TX_FIFO                                         0x01
#define PICONET2_TX_FIFO                                         0x02

#define MAX_PICONET_CNT                            LMP_MAX_PICONETS_SUPPORTED

/* modified by austin for 4 piconet support
+-----------+-----------+-----------+-----------+-------+
| Piconet 0 | Piconet 1 | Piconet 2 | Piconet 3 | case
+-----------+-----------+-----------+-----------+-------+
|     -     |     -     |     -     |     -     | 0
|     M     |     -     |     -     |     -     | 1
|     M     |     S     |     -     |     -     | 2
|     M     |     -     |     S     |     -     | 3
|     M     |     -     |     -     |     S     | 4
|     M     |     S1    |     S2    |     -     | 5
|     M     |     S1    |     -     |     S2    | 6
|     M     |     -     |     S1    |     S2    | 7
|     M     |     S1    |     S2    |     S3    | 8
|     -     |     M     |     -     |     -     | 9
|     S     |     M     |     -     |     -     | 10
|     -     |     M     |     S     |     -     | 11
|     -     |     M     |     -     |     S     | 12
|     S1    |     M     |     S2    |     -     | 13
|     S1    |     M     |     -     |     S2    | 14
|     -     |     M     |     S1    |     S2    | 15
|     S1    |     M     |     S2    |     S3    | 16
|     -     |     -     |     M     |     -     | 17
|     S     |     -     |     M     |     -     | 18
|     -     |     S     |     M     |     -     | 19
|     -     |     -     |     M     |     S     | 20
|     S1    |     S2    |     M     |     -     | 21
|     S1    |     -     |     M     |     S2    | 22
|     -     |     S1    |     M     |     S2    | 23
|     S1    |     S2    |     M     |     S3    | 24
|     -     |     -     |     -     |     M     | 25
|     S     |     -     |     -     |     M     | 26
|     -     |     S     |     -     |     M     | 27
|     -     |     -     |     S     |     M     | 28
|     S1    |     S2    |     -     |     M     | 29
|     S1    |     -     |     S2    |     M     | 30
|     -     |     S1    |     S2    |     M     | 31
|     S1    |     S2    |     S3    |     M     | 32
|     S     |     -     |     -     |     -     | 33
|     -     |     S     |     -     |     -     | 34
|     -     |     -     |     S     |     -     | 35
|     -     |     -     |     -     |     S     | 36
|     S1    |     S2    |     -     |     -     | 37
|     S1    |     -     |     S2    |     -     | 38
|     S1    |     -     |     -     |     S2    | 39
|     -     |     S1    |     S2    |     -     | 40
|     -     |     S1    |     -     |     S2    | 41
|     -     |     -     |     S1    |     S2    | 42
|     S1    |     S2    |     S3    |     -     | 43
|     S1    |     S2    |     -     |     S3    | 44
|     S1    |     -     |     S2    |     S3    | 45
|     -     |     S1    |     S2    |     S3    | 46
|     S1    |     S2    |     S3    |     S4    | 47
+-----------+-----------+-----------+-----------+------+ */
typedef enum
{
    SCA_IDLE                        = 0,
    SCA_MASTER                      = 1,
    SCA_MASTER_SLAVE                = 2,
    SCA_MASTER_IDLE_SLAVE           = 3,
    SCA_MASTER_IDLE_IDLE_SLAVE      = 4,
    SCA_MASTER_SLAVE1_SLAVE2        = 5,
    SCA_MASTER_SLAVE1_IDLE_SLAVE2   = 6,
    SCA_MASTER_IDLE_SLAVE1_SLAVE2   = 7,
    SCA_MASTER_SLAVE1_SLAVE2_SLAVE3 = 8,
    SCA_IDLE_MASTER                 = 9,
    SCA_SLAVE_MASTER                = 10,
    SCA_IDLE_MASTER_SLAVE           = 11,
    SCA_IDLE_MASTER_IDLE_SLAVE      = 12,
    SCA_SLAVE1_MASTER_SLAVE2        = 13,
    SCA_SLAVE1_MASTER_IDLE_SLAVE2   = 14,
    SCA_IDLE_MASTER_SLAVE1_SLAVE2   = 15,
    SCA_SLAVE1_MASTER_SLAVE2_SLAVE3 = 16,
    SCA_IDLE_IDLE_MASTER            = 17,
    SCA_SLAVE_IDLE_MASTER           = 18,
    SCA_IDLE_SLAVE_MASTER           = 19,
    SCA_IDLE_IDLE_MASTER_SLAVE      = 20,
    SCA_SLAVE1_SLAVE2_MASTER        = 21,
    SCA_SLAVE1_IDLE_MASTER_SLAVE2   = 22,
    SCA_IDLE_SLAVE1_MASTER_SLAVE2   = 23,
    SCA_SLAVE1_SLAVE2_MASTER_SLAVE3 = 24,
    SCA_IDLE_IDLE_IDLE_MASTER       = 25,
    SCA_SLAVE_IDLE_IDLE_MASTER      = 26,
    SCA_IDLE_SLAVE_IDLE_MASTER      = 27,
    SCA_IDLE_IDLE_SLAVE_MASTER      = 28,
    SCA_SLAVE1_SLAVE2_IDLE_MASTER   = 29,
    SCA_SLAVE1_IDLE_SLAVE2_MASTER   = 30,
    SCA_IDLE_SLAVE1_SLAVE2_MASTER   = 31,
    SCA_SLAVE1_SLAVE2_SLAVE3_MASTER = 32,
    SCA_SLAVE                       = 33,
    SCA_IDLE_SLAVE                  = 34,
    SCA_IDLE_IDLE_SLAVE             = 35,
    SCA_IDLE_IDLE_IDLE_SLAVE        = 36,
    SCA_SLAVE1_SLAVE2               = 37,
    SCA_SLAVE1_IDLE_SLAVE2          = 38,
    SCA_SLAVE1_IDLE_IDLE_SLAVE2     = 39,
    SCA_IDLE_SLAVE1_SLAVE2          = 40,
    SCA_IDLE_SLAVE1_IDLE_SLAVE2     = 41,
    SCA_IDLE_IDLE_SLAVE1_SLAVE2     = 42,
    SCA_SLAVE1_SLAVE2_SLAVE3        = 43,
    SCA_SLAVE1_SLAVE2_IDLE_SLAVE3   = 44,
    SCA_SLAVE1_IDLE_SLAVE2_SLAVE3   = 45,
    SCA_IDLE_SLAVE1_SLAVE2_SLAVE3   = 46,
    SCA_SLAVE1_SLAVE2_SLAVE3_SLAVE4 = 47,
    SCA_INVALID                     = 0xFF,
} LC_CUR_SCATTERNET_STATE;

typedef struct {
    UINT8 master:1; /* my role is master ? */
    UINT8 active:1; /* this piconet info is active ? */
    UINT8 rsvd:6;   /* reserved */
} LC_SCA_PICONET_INFO;

typedef struct {
    UINT8 master_cnt; /* the toal amount of master role in all piconets */
    UINT8 slave_cnt;  /* the total amount of slave role in all piconets */
    UINT8 master_id;  /* the piconet id when i am master role */
    UINT8 bm_slave;   /* the piconet bitmap when i am slave role */
    LC_SCA_PICONET_INFO pnet[MAX_PICONET_CNT];
} LC_SCA_MANAGER_S;

extern LC_SCA_MANAGER_S lc_sca_manager;

extern LC_CUR_SCATTERNET_STATE lc_current_scatternet_state;

#define LC_MAX_NUM_OF_LUT_EX_TABLES                   MAX_NO_OF_LUT_INDICES

#define NUM_PKTS_TIMER_VALUE                                     0x14

#define BZDMA_ACL_TX_BUF_SIZE                 32

extern UINT8 lc_cont_crc_rx_cnt[LC_MAX_NUM_OF_LUT_EX_TABLES];
extern UINT8 lc_cont_crc_tx_cnt[LC_MAX_NUM_OF_LUT_EX_TABLES];

#ifdef FW_DEFAULT_PACKET_RULE
/* add by austin to improve our scheduler */
#define LC_CONT_CRC_RX_RISE_STEP             1
#define LC_CONT_CRC_RX_FALL_STEP             2
#define LC_CONT_CRC_TX_RISE_STEP             1
#define LC_CONT_CRC_TX_FALL_STEP             2
extern UINT8 lc_cont_crc_rx_cnt_st[LC_MAX_NUM_OF_LUT_EX_TABLES];
extern UINT8 lc_cont_crc_tx_cnt_st[LC_MAX_NUM_OF_LUT_EX_TABLES];
#endif

extern CHAR *lc_tx_task_name;
extern CHAR *lc_rx_task_name;

/* Comamnd and Event Task handles */
extern TASK_ID lc_tx_task_handle ;
extern TASK_ID lc_rx_task_handle ;

#define LC_MIN_SNIFF_INT_FOR_ALLOWING_5_SLOTS                       21
#define LC_MIN_SNIFF_INT_FOR_ALLOWING_3_SLOTS                       10

#define FORCE_FLUSH_CLOCK                          (UINT32)0xFFFFFFFF

#define SNIFF_END_OF_ATTEMPT                                   0x1000
#define SNIFF_START_OF_ATTEMPT                                 0x2000

#define LC_MIN_SLOTS_REQD_FOR_XTOL                                 80
#define LC_OFFSET_FROM_SNIFF_ANCHOR_POINT      0x02
#define LC_MIN_SLOTS_BEFORE_SNIFF_STOP_POLLING 0x0E
#define LC_MAX_SLOTS_TO_FORCE_NBC_FOR_SNIFF_PKT 32

#define LC_MIN_SLOTS_REQD_FOR_XTOL               80

#define LC_MIN_DSNIFF_VALUE                                         8
#define LC_BUF_SNIFF_OFFSET_VALUE_FOR_TOL                           2

#define LC_MIN_SNIFF_INT_FOR_XTOL                                  80
#define LC_MIN_HOLD_INT_FOR_XTOL                                   80
#define LMP_UNSNIFF_TPOLL                                        0x07

extern UINT16 lc_min_sniff_interval_allow_5_slot_pkt;
extern UINT16 lc_min_sniff_interval_allow_3_slot_pkt;
#define LC_MIN_SNIFF_INTERVAL_ALLOW_5_SLOT_PKT                    lc_min_sniff_interval_allow_5_slot_pkt
#define LC_MIN_SNIFF_INTERVAL_ALLOW_3_SLOT_PKT                    lc_min_sniff_interval_allow_3_slot_pkt

#define LC_MIN_DBEACON_VALUE                                        8

#define ACTIVE_BIT                                             0x0100
#define FLOW_BIT                                               0x0020
#define LC_UPPER_LUT_HOLD_MODE                                 0x0010

#define      BB_RX_ISR_NULL                          (0x01 << BB_NULL)
#define      BB_RX_ISR_POLL                          (0x01 << BB_POLL)
#define      BB_RX_ISR_FHS                            (0x01 << BB_FHS)
#define      BB_RX_ISR_DM1                            (0x01 << BB_DM1)
#define      BB_RX_ISR_DH1                            (0x01 << BB_DH1)
#define      BB_RX_ISR_HV1                            (0x01 << BB_HV1)
#define      BB_RX_ISR_HV2                            (0x01 << BB_HV2)
#define      BB_RX_ISR_HV3                            (0x01 << BB_HV3)
#define      BB_RX_ISR_DV                              (0x01 << BB_DV)
#define      BB_RX_ISR_DM3                            (0x01 << BB_DM3)
#define      BB_RX_ISR_DH3                            (0x01 << BB_DH3)
#define      BB_RX_ISR_DM5                            (0x01 << BB_DM5)
#define      BB_RX_ISR_DH5                            (0x01 << BB_DH5)
#define      BB_RX_ISR_AUX1                           (0x01 << BB_AUX1)
#define      BB_RX_ISR_EV3                            (0x01 << BB_EV3)
#define      BB_RX_ISR_EV4                            (0x01 << BB_EV4)
#define      BB_RX_ISR_EV5                            (0x01 << BB_EV5)

#define      BB_RX_ISR_2_DH1                             BB_RX_ISR_DH1
#define      BB_RX_ISR_3_DH1                              BB_RX_ISR_DV
#define      BB_RX_ISR_2_DH3                             BB_RX_ISR_DM3
#define      BB_RX_ISR_3_DH3                             BB_RX_ISR_DH3
#define      BB_RX_ISR_2_DH5                             BB_RX_ISR_DM5
#define      BB_RX_ISR_3_DH5                             BB_RX_ISR_DH5
#define      BB_RX_ISR_2_EV3                             BB_RX_ISR_HV2
#define      BB_RX_ISR_3_EV3                             BB_RX_ISR_HV3
#define      BB_RX_ISR_2_EV5                             BB_RX_ISR_EV4
#define      BB_RX_ISR_3_EV5                             BB_RX_ISR_EV5


#define  BB_RX_ISR_HV1_HV2_HV3                                         \
                 ( BB_RX_ISR_HV2 | BB_RX_ISR_HV3 | BB_RX_ISR_HV1 )

#define  BB_RX_ISR_HV1_HV2_HV3_DV                                      \
                 ( BB_RX_ISR_HV2 | BB_RX_ISR_HV3 | BB_RX_ISR_HV1 |     \
                 BB_RX_ISR_DV )

#define  BB_RX_ISR_POLL_NULL_FHS                                       \
                 ( BB_RX_ISR_POLL | BB_RX_ISR_NULL | BB_RX_ISR_FHS )

#define  BB_RX_ISR_NULL_FHS                                       \
                 ( BB_RX_ISR_NULL | BB_RX_ISR_FHS )

#define  BB_RX_ISR_HV2_HV3                                             \
                 ( BB_RX_ISR_HV2 | BB_RX_ISR_HV3 )

#define  BB_RX_ISR_CRC_PKT                                             \
                 ( BB_RX_ISR_DM1 | BB_RX_ISR_DH1 | BB_RX_ISR_DM3 |     \
                   BB_RX_ISR_DH3 | BB_RX_ISR_DM5 | BB_RX_ISR_DH5 |     \
                   BB_RX_ISR_2_DH1 | BB_RX_ISR_2_DH3| BB_RX_ISR_2_DH5| \
                   BB_RX_ISR_3_DH1 | BB_RX_ISR_3_DH3| BB_RX_ISR_3_DH5| \
                   BB_RX_ISR_DV )

#define  BB_RX_ISR_CRC_2M_PKT                                             \
                 ( BB_RX_ISR_2_DH1 | BB_RX_ISR_2_DH3| BB_RX_ISR_2_DH5)

#define  BB_RX_ISR_CRC_3M_PKT                                             \
                 ( BB_RX_ISR_3_DH1 | BB_RX_ISR_3_DH3| BB_RX_ISR_3_DH5)

#define  BB_RX_ISR_CRC_AND_FHS_PKT                                     \
                 ( BB_RX_ISR_CRC_PKT | BB_RX_ISR_FHS )


#define  BB_RX_ISR_CRC_AND_FHS_AND_AUX1_PKT               \
                 ( BB_RX_ISR_CRC_PKT | BB_RX_ISR_FHS | BB_RX_ISR_AUX1)

#define  BB_RX_ISR_POLL_NULL_CRC_PKT                                   \
                 ( BB_RX_ISR_POLL | BB_RX_ISR_NULL | BB_RX_ISR_CRC_PKT )

#define  BB_RX_ISR_DM_DH_PKT \
            ( BB_RX_ISR_DM1 | BB_RX_ISR_DM3 | BB_RX_ISR_DM5 | \
              BB_RX_ISR_DH1 | BB_RX_ISR_DH3 | BB_RX_ISR_DH5 )

#define  BB_RX_ISR_DM_DH_ALL_PKT \
            ( BB_RX_ISR_DM_DH_PKT | \
              BB_RX_ISR_2_DH1 | BB_RX_ISR_2_DH3| BB_RX_ISR_2_DH5| \
              BB_RX_ISR_3_DH1 | BB_RX_ISR_3_DH3| BB_RX_ISR_3_DH5 )

#define  BB_RX_ISR_NO_CRC_DATA_PKT \
        ( BB_RX_ISR_HV1 | BB_RX_ISR_HV2 | BB_RX_ISR_HV3 | BB_RX_ISR_AUX1 )

#define  BB_RX_ISR_ESCO_DATA_PKT \
        ( BB_RX_ISR_EV3 | BB_RX_ISR_EV4 | BB_RX_ISR_EV5 | \
          BB_RX_ISR_2_EV3 | BB_RX_ISR_2_EV5 | BB_RX_ISR_3_EV3 | BB_RX_ISR_3_EV5)

#define  BB_RX_ISR_SCO_VOICE_PKT \
        ( BB_RX_ISR_HV1 | BB_RX_ISR_HV2 | BB_RX_ISR_HV3 )

#define BB_RX_ISR_VOICE_DATA_PKT \
        ( BB_RX_ISR_ESCO_DATA_PKT | BB_RX_ISR_SCO_VOICE_PKT )

#define BB_RX_ISR_PTT0_ONE_SLOT_ACL_PKT     (BB_RX_ISR_DM1 | BB_RX_ISR_DH1)
#define BB_RX_ISR_PTT0_THREE_SLOT_ACL_PKT   (BB_RX_ISR_DM3 | BB_RX_ISR_DH3)
#define BB_RX_ISR_PTT0_FIVE_SLOT_ACL_PKT    (BB_RX_ISR_DM5 | BB_RX_ISR_DH5)
#define BB_RX_ISR_PTT1_ONE_SLOT_ACL_PKT     (BB_RX_ISR_2_DH1 | BB_RX_ISR_3_DH1)
#define BB_RX_ISR_PTT1_THREE_SLOT_ACL_PKT   (BB_RX_ISR_2_DH3 | BB_RX_ISR_3_DH3)
#define BB_RX_ISR_PTT1_FIVE_SLOT_ACL_PKT    (BB_RX_ISR_2_DH5 | BB_RX_ISR_3_DH5)

#define AND_val_with_bb_reg_macro(reg_offset,val)                      \
{                                                                      \
    UINT16 read;                                                       \
    read = BB_read_baseband_register(reg_offset);                      \
    read &= val;                                                       \
    BB_write_baseband_register(reg_offset, read);                      \
}

#define OR_val_with_bb_reg_macro(reg_offset, val)                      \
{                                                                      \
    UINT16 read;                                                       \
    read = BB_read_baseband_register(reg_offset);                      \
    read |= val;                                                       \
    BB_write_baseband_register(reg_offset,read);                       \
}

#define LC_SET_PTT_BIT_IN_LUT(lut_address)                             \
{                                                                      \
    UINT16 temp_var;                                                   \
    temp_var = BB_read_baseband_register(lut_address);                 \
    temp_var |= PTT_BIT_MASK;                                          \
    BB_write_baseband_register(lut_address, temp_var);                 \
}                                                                      \

#define LC_RESET_PTT_BIT_IN_LUT(lut_address)                           \
{                                                                      \
    UINT16 temp_var;                                                   \
    temp_var = BB_read_baseband_register(lut_address);                 \
    temp_var &= (~PTT_BIT_MASK);                                       \
    BB_write_baseband_register(lut_address, temp_var);                 \
}                                                                      \

/* Set 2M/3M Tx Power with LUT register - add by austin */
extern void lc_set_2m_tx_power(UINT8 lut_idx, UINT8 txgain_idx);
extern void lc_set_3m_tx_power(UINT8 lut_idx, UINT8 txgain_idx);
#define LC_SET_2M_TX_POWER(lut, txgain) lc_set_2m_tx_power(lut, txgain)
#define LC_SET_3M_TX_POWER(lut, txgain) lc_set_3m_tx_power(lut, txgain)

#define CONNECTOR_REGISTER_MAKE_BIT                              0x10
#define CONNECTOR_REGISTER_FOR_ACL_BIT                           0x08

#define LC_MASTER_DEFAULT_PACKET_TYPE                          0x1c00
#define LC_SLAVE_DEFAULT_PACKET_TYPE                           0x0c00

#define ENC_REG_NO_RETX_BIT                                   0x0040U
#define ENC_REG_NO_SCO_BIT                                    0x0020U

#define LC_INVALID_PACKET_TYPE                                 0xc000

#define INVALID_LUT_INDEX                                        0xFF

#define INVALID_AM_ADDR                                          0xFF
#define LC_MAX_RETX_LIMIT                                           1

#define LC_HOP_79                                              0x0020
#define LC_HOP_1                                               0x0060
#define LC_DEFAULT_HOP                                      LC_HOP_79
//#define LC_DEFAULT_HOP                                      LC_HOP_1
#define MAX_RAND                                                 1023

#define LC_SCO_PKT_TYPE_MASTER                                 0x0010
#define LC_SCO_PKT_TYPE_SLAVE                                  0xFFEF

#define LC_CORRELATOR_THRESOLD_VAL                             0x003c

/*
 * Values to be used for bitmap 24 [siw radio intg]
 * is tx_on ==> 0x00cf
 *    rx_on ==> 0x80b3
 *    phd_de==> 0x0093
 */

/*
 * Values to be used for bitmap 4 [siw radio intg]
 * is tx_on ==> 0x00bb
 *    rx_on ==> 0x50ac
 *    phd_de==> 0x007d
 */
#define LC_TX_ON_DELAY_VAL                                     0x00cf
#define LC_RX_ON_DELAY_VAL                                     0x80b3
#define LC_PHD_DELAY_VAL                                       0x0093
#define RADIO_SELECT_VAL                                         0x02

extern UINT8 max_rtk_radio_tx_step_index;
extern UINT8 default_rtk_radio_tx_step_index;

/* Default value for silicon wave radio. */
#define RADIO_POWER_VAL                       default_rtk_radio_tx_step_index

/* Maximum radio power value ... 0db */
#define MAX_RADIO_TX_POWER                    max_rtk_radio_tx_step_index
#define MIN_RADIO_TX_POWER                    0x00

#define LC_SIW_RF_NUM_REG                                          52

#define RTK_WRITE_MODEM_REG(waddr, value) \
    rtk_write_modem_radio_reg(waddr, TYPE_MODEM, value)
#define RTK_WRITE_RF_REG(waddr, value) \
    rtk_write_modem_radio_reg(waddr, TYPE_RF, value)

#define RTK_READ_MODEM_REG(waddr) rtk_read_modem_radio_reg(waddr, TYPE_MODEM)
#define RTK_READ_RF_REG(waddr)    rtk_read_modem_radio_reg(waddr, TYPE_RF)

#define RTK_UPDATE_MODEM_REG(waddr, bm_mask, value) \
{ \
    UINT16 temp = RTK_READ_MODEM_REG(waddr); \
    RTK_WRITE_MODEM_REG(waddr, (temp & ~(bm_mask)) | ((bm_mask) & (value))); \
}
#define RTK_UPDATE_RF_REG(waddr, bm_mask, value) \
{ \
    UINT16 temp = RTK_READ_RF_REG(waddr); \
    RTK_WRITE_RF_REG(waddr, (temp & ~(bm_mask)) | ((bm_mask) & (value))); \
}

#ifdef _NEW_MODEM_PI_ACCESS_
#define RTK_WRITE_MODEM_REG_PI(modem_page, waddr, value) \
    rtk_write_modem_radio_reg_pi(modem_page, waddr, TYPE_MODEM, value)
#define RTK_READ_MODEM_REG_PI(modem_page, waddr) rtk_read_modem_radio_reg_pi(modem_page, waddr, TYPE_MODEM)
#define RTK_UPDATE_MODEM_REG_PI(modem_page, waddr, bm_mask, value) \
{ \
    UINT16 temp = RTK_READ_MODEM_REG_PI(modem_page, waddr); \
    RTK_WRITE_MODEM_REG_PI(modem_page, waddr, (temp & ~(bm_mask)) | ((bm_mask) & (value))); \
}
#endif

#ifdef _NEW_RFC_PI_ACCESS_
UINT16 rtk_read_rfc_reg_pi(UINT8 rfc_addr);
void rtk_write_rfc_reg_pi(UINT8 u8rfc_addr, UINT16 assignvalue, UINT8 park_rfc_pi_sel);

#define REG_BTRFC_PI_IF                 0x348

enum {PI_RFC_ARBITER_MODEM=0, PI_RFC_ARBITER_CPU};
enum {PI_RFC_READ=0, PI_RFC_WRITE};

typedef struct VENDOR_PI_ACCESS_RFC_STR_ {
    union {
        struct {
            UINT32 bt_ioq_rfc       :16;    /* [15:0], r/w data             */
            UINT32 bt_ioad_rfc      :8;     /* [23:16], addr                */
            UINT32 rf_iow           :1;     /* [24] r/w control             */
            UINT32 rfc_pi_sel       :1;     /* [25] important!!!, arbiter   */
            UINT32 rsvd_31_26       :6;     /* [31:26] reserved             */
        };
        UINT32 d32;
    };
}VENDOR_PI_ACCESS_RFC_STR;

#endif /* #ifdef _NEW_RFC_PI_ACCESS_ */

#ifdef _IS_ASIC_
extern INT8 g_rtl8723_btrf_txpower_track_n_old;
extern INT8 g_rtl8723_btrf_cfo_track_n_old;
extern INT8 g_rtl8723_btrf_rxgain_track_n_old;
extern UINT8 g_rtl8723_btrf_thermal_old;
extern INT8 g_rtl8723_btrf_lok_track_n_old;

//#define MAX_RTL8723RF_TX_POWER_INDEX            6
extern UINT8 g_max_rtl8723rf_tx_power_index;

#endif
#define ABS(x)     (((x) > 0) ? (x) : ((~(x))+1))

//typedef struct
//{
//    UCHAR addr[LC_SIW_RF_NUM_REG + 1];
//    UCHAR val[LC_SIW_RF_NUM_REG + 1];
//    UCHAR delay[LC_SIW_RF_NUM_REG + 1]; /* 10 usec Unit */
//}SIW_STARTUP_SEQ;


#ifdef POWER_SAVE_FEATURE

#define LPS_32K_UNLOCK_MAX_TIME 100

#ifndef _LPS_FOR_8821_
#define LPS_SCAN_PROTECT_TIME 5
#endif

typedef enum
{
    BB_NORMAL_MODE,
    BB_SLEEP_MODE,
    BB_DEEP_SLEEP_MODE
}BB_MODE;

#ifdef _CCH_RTL8723A_B_CUT
#ifdef _LPS_FOR_8821_
#define LPS_TABLE_NUM    ( LMP_MAX_CE_DATABASE_ENTRIES + 8 + 4 )
// ( LMP_MAX_CE_DATABASE_ENTRIES + LL_MAX_CONNECTION_UNITS + 4 )
// 0~9: Legacy
// 10~17: LE
// 18: PS
// 19: IS
// 20: LE adv
// 21: LE scan

#define LPS_MODE_MAX_TIME_VAL 200

// Only Internal 32k Need These
#define LPS_SNIFF_QUIT_LPS_M  0xC80 //   2 sec
#define LPS_SNIFF_QUIT_LPS_S  0xFFFF // xx sec

#define LPS_32K_CAL_DIFF_MAX 0xC80 //2sec
#define LPS_32K_CAL_UNLOCK_MAX 0x50 //0x50 //50msec

#define LPS_32K_CAL_100PPM_LOW_BOND 9374
#define LPS_32K_CAL_100PPM_UP_BOND 9376

#define LPS_32K_CAL_1000PPM_LOW_BOND 9366
#define LPS_32K_CAL_1000PPM_UP_BOND 9384


#define LPS_32K_CAL_MAXPPM_LOW_BOND 0
#define LPS_32K_CAL_MAXPPM_UP_BOND 0x3FFF

enum CAL_TH_INDEX {
    CAL_TH_MAXPPM = 0,
    CAL_TH_1000PPM = 1,
    CAL_TH_100PPM = 2
};


#endif

#ifdef _MODI_LPS_AFTER_RTL8703B_
#define LPS_PERIOD_STATE_NUM 9
#else
#define LPS_PERIOD_STATE_NUM 7
#endif

typedef enum
{
    LPS_PERIOD_END,
    LPS_PERIOD_INQ_SCAN,
    LPS_PERIOD_PAG_SCAN,
    LPS_PERIOD_LE_ADV,
    LPS_PERIOD_LE_SCAN,
    LPS_PERIOD_LE_INIT,
    LPS_PERIOD_RSVD1,
    LPS_PERIOD_RSVD2,
    LPS_PERIOD_NULL
}LPS_PERIOD_STATE;


#endif


typedef struct
{
  BB_MODE bb_sm_sts;
  UINT32 bb_sm_wakeup_inst;
#ifdef _YL_LPS
  UCHAR bb_sm_piconet_id;
#endif

#ifdef _CCH_RTL8723A_B_CUT
// _CCH_ECO_LPS_
  UINT8 lps_lpm_end_flag;
  UINT8 lps_period_count;
  UINT8 lps_period_state;                 // current state

  UINT8  lps_period_state_bitmap;
  UINT8  lps_period_state_num[LPS_PERIOD_STATE_NUM];     // unit: 10msec
  UINT16 lps_period_state_interval[LPS_PERIOD_STATE_NUM]; // unit: 1 slot (0.625msec)
  UINT8  lps_lpm_lps_mode;
  UCHAR  lps_lpm_pid;
  UINT32 lps_lpm_wakeup_instance;

  UINT16 lps_period_interval;              // unit: 1 slot (0.625msec)
  UINT8  lps_procedure_start;
  UINT8  lps_task_flag;
  UINT16 lps_32k_cal_count;
#ifdef _MODI_LPS_AFTER_RTL8703B_
  UINT16 lps_32k_cal_max;
#endif
  UINT16 lps_link_interval;
#ifdef LPS_NEW_CAL_XTOL
  UINT16 lps_drift_ppm_for_xtol;        // yilinli, for lc_calculation_tolerance(): can provide flexible window widening
#endif
#endif

#ifdef _LPS_FOR_8821_
  UINT32 lps_table_instance[LPS_TABLE_NUM];
  UINT32 lps_table_ing_bitmap;   // bitmap is the on-going process, during sniff window, and etc
  UINT8  wakeup_scan_enable;
  UINT8  lps_table_instance_index;      // test use only

  UINT8   cal_lock_th_flag;
  UINT32  cal_enable_nat_clk;
  UINT8   scan_end_flag;         // Bit0=pagescan Bit1=inqscan Bit2=adv Bit3=lescan
#endif

#ifdef _MODI_LPS_AFTER_RTL8703B_
  UINT16 le_interrupt_status;// LE_REG_STATUS_ADV_TIMEOUT_CTRL
  UINT8  le_adv_num;
  UINT8  le_adv_count;
#endif

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
  UINT8 run_dlps_flow;
  UINT8 dlps_restore_flow;
#endif

  UINT32 clock_temp1;
  UINT32 clock_temp2;
  UINT32 lps_wakeup_count;

  UINT8 lps_log_enable;

} LC_SLEEP_MODE_PARAM;


extern LC_SLEEP_MODE_PARAM sleep_mode_param;
extern UINT16 bb_write_baseband_register_func_imp_count;
extern OS_QUEUE_MGR queue_mgr;

#ifdef _ENABLE_BTON_POWER_SAVING_
extern void rlx4081_isr_sw_unmask();
#endif

#define LC_SM_INVALID_INSTANT          0

extern UCHAR lc_start_of_beacon;
extern UINT16 lmp_unpark_ce_index;

extern UCHAR lc_cur_connecting_am_addr;
extern UINT16 lc_paging_bd_addr[3];
extern UINT16 lc_paging_parity_bits[3];
extern UINT16 lc_slave_bd_addr[3];
extern UINT16 lc_slave_parity_bits[3];
extern UCHAR random_backoff_status;

extern UINT8 lc_waiting_for_crc_pkt_ack[LMP_MAX_PICONETS_SUPPORTED];
extern UINT16 lc_current_interrupt_mask;
extern UINT16 lc_current_interrupt_mask2;

#ifdef COMPILE_ESCO
extern UINT16 lc_var_esco_fifo_flush_length_and_tx_pkt_len_reg;

extern LMP_ESCO_CONNECTION_ENTITY
       lmp_esco_connection_entity[LMP_MAX_ESCO_CONN_ENTITIES];
#endif


void lc_init_sleep_mode_param(void);
#ifndef _REDUCE_LPS_AFTER_RTL8703B_
API_RESULT lc_post_sm_mode_signal(UINT32 wakeup_instant, UCHAR piconet_id);

void lc_program_sm_mode(UINT32 wakeup_instant, UCHAR piconet_id);
void lc_program_dsm_mode(UINT32 prog_val, UCHAR piconet_id);
#endif

#if defined(_CCH_RTL8723A_B_CUT) && defined(_CCH_LPS_USING_STATE_)
// _CCH_ECO_LPS_
void lps_period_state_machine(UINT8 state, UINT8 state_bitmap, UINT8 count, UINT8 lps_mode, UCHAR piconet_id, UINT32 wakeup_instant);
void lps_period_state_enter_lps(UINT8 lps_mode);

#ifdef _MODI_LPS_AFTER_RTL8821B_TC_
void lps_period_state_machine_fast(UINT32 ps_clock, UINT16 ps_duration);
#endif

#ifdef _MODI_LPS_AFTER_RTL8703B_
void lps_kill_inq_scan();
void lps_kill_page_scan();
#endif

#endif

#if defined(_CCH_LPS_) && defined(_YL_LPS)
UINT8 lc_program_lps_mode(UCHAR lps_mode, UINT32 wakeup_instant, UCHAR piconet_id);
#endif

API_RESULT lc_post_dsm_mode_signal(UINT32 wakeup_instant, UCHAR piconet_id);

void lc_exit_sm_mode(void);

void lc_handle_sm_intr(void);

void lc_calculate_num_park_sleep_slots(UINT16 *num_sleep_slots);

void lc_program_sniff_sm_mode(UINT16 ce_index);

void lc_program_hold_sm_mode(UINT16 hold_interval, UINT32 hold_instant);

void lc_program_park_sm_mode(UINT16 interval);


#define LC_INIT_SLEEP_MODE_PARAM()                                            \
    lc_init_sleep_mode_param();

#ifndef _REDUCE_LPS_AFTER_RTL8703B_
#define LC_PROGRAM_SM_MODE(wakeup_instant,piconet_id)                         \
    lc_program_sm_mode(wakeup_instant, piconet_id);

#define LC_PROGRAM_DSM_MODE(wakeup_instant, piconet_id)                        \
    lc_program_dsm_mode(wakeup_instant, piconet_id);
#endif

#if defined(_CCH_LPS_) && defined(_YL_LPS)
#define LC_PROGRAM_LPS_MODE(lps_mode, wakeup_instant, piconet_id)                        \
    lc_program_lps_mode(lps_mode, wakeup_instant, piconet_id);
#endif

#define LC_EXIT_SM_MODE()                                                     \
    lc_exit_sm_mode();

#define LC_HANDLE_SM_INTR()                                       \
    lc_handle_sm_intr();

#define LC_CALCULATE_NUM_INQUIRY_SCAN_SLEEP_SLOTS(num_sleep_slots)            \
    lc_calculate_num_inquiry_scan_sleep_slots(num_sleep_slots);

#define LC_CALCULATE_NUM_PAGE_SCAN_SLEEP_SLOTS(num_sleep_slots)               \
    lc_calculate_num_page_scan_sleep_slots(num_sleep_slots);

#define LC_CALCULATE_NUM_PARK_SLEEP_SLOTS(num_sleep_slots)                    \
    lc_calculate_num_park_sleep_slots(num_sleep_slots);

#define LC_PROGRAM_SNIFF_SM_MODE(count)                                       \
    lc_program_sniff_sm_mode(count);

#define LC_PROGRAM_HOLD_SM_MODE(hold_interval, hold_instant)                  \
    lc_program_hold_sm_mode(hold_interval, hold_instant);

#define LC_PROGRAM_PARK_SM_MODE(interval)                                     \
    lc_program_park_sm_mode(interval);

#else
#define LC_INIT_SLEEP_MODE_PARAM()
#define LC_PROGRAM_SM_MODE(wakeup_instant, piconet_id)
#define LC_EXIT_SM_MODE()
#define LC_HANDLE_SM_INTR()
#define LC_CALCULATE_NUM_PARK_SLEEP_SLOTS(num_sleep_slots)

#define LC_PROGRAM_SNIFF_SM_MODE(count)
#define LC_PROGRAM_HOLD_SM_MODE(hold_interval, hold_instant)

#define LC_PROGRAM_PARK_SM_MODE(interval)
#define LC_PROGRAM_DSM_MODE(wakeup_instant,piconet_id)

#endif

#ifdef _CCH_RTL8723A_B_CUT
// _CCH_ISSC_VENDOR_
#define HCI_VENDOR_COMPUTE_ISSC_SEC(state)	\
	hci_vendor_compute_issc_sec(state)

#endif

#define LC_DATA_ACTIVE_FLAG                                         0
#define LC_ACTIVE_BC_FLAG                                           1
#define LC_PNET_BC_FLAG                                             2
#define LC_INQUIRY_LAP_MIN                                   0X9E8B00
#define LC_INQUIRY_LAP_MAX                                   0X9E8B3F
#define LC_GEN_INQUIRY_LAP                                   0X9E8B33
#define LC_DEFAULT_PAGE_RES_TIMEOUT                              0x08
#define LC_INQUNIT_TO_SLOT_CONV_FACTOR                           1024



#define MANDATORY_PAGING_SCHEME                                     0
#define OPTIONAL_PAGING_SCHEME_1                                    1
#define OPTIONAL_PAGING_SCHEME_2                                    2
#define OPTIONAL_PAGING_SCHEME_3                                    3

#define  PAGING_SCHEME_SETTING_R0                                   0
#define  PAGING_SCHEME_SETTING_R1                                   1
#define  PAGING_SCHEME_SETTING_R2                                   2

#define LC_GIACP1                                              0X6E1E
#define LC_GIACP2                                              0X88D6
#define LC_GIACP3                                              0X0000

#define LC_GIAC1                                               0x8b33
#define LC_GIAC2                                               0x009e

#define LC_MAX_AM_ADDR                                              8

#define LC_L_CH                                                0x0C00

#define LC_FLOW_LUT                                            0x0800
#define LC_PN_SEQ_LSB                                      0xbbcc54fc
#define LC_PN_SEQ_MSB                                      0x83848d96
#define LC_GEN_POLY_MSB                                    0xB0AE27B5
#define LC_GEN_POLY_LSB                                    0x20000000

#if 1 /* ndef FIFO_WORD_WRITE */

#define LC_MIN_NUMBER_OF_BYTES_IN_DV_PACKET                         0
#define LC_MAX_NUMBER_OF_BYTES_IN_DV_PACKET                         9
#define LMP_MAX_NUMBER_OF_BYTES_IN_DV_PACKET                        9

#define LC_MIN_NUMBER_OF_BYTES_IN_DH1_PACKET                        0
#define LC_MAX_NUMBER_OF_BYTES_IN_DH1_PACKET                       27

#define LC_MIN_NUMBER_OF_BYTES_IN_DM1_PACKET                        0
#define LC_MAX_NUMBER_OF_BYTES_IN_DM1_PACKET                       17

#define LC_MIN_NUMBER_OF_BYTES_IN_DH3_PACKET                       28
#define LC_MAX_NUMBER_OF_BYTES_IN_DH3_PACKET                      183

#define LC_MIN_NUMBER_OF_BYTES_IN_DM3_PACKET                       18
#define LC_MAX_NUMBER_OF_BYTES_IN_DM3_PACKET                      121

/*
 * Our buffer size is equal to 339, so this packet will never be
 * fragmented. Hence using 339 as is.
 */
#define LC_MIN_NUMBER_OF_BYTES_IN_DH5_PACKET                      184
#define LC_MAX_NUMBER_OF_BYTES_IN_DH5_PACKET                      339

#define LC_MIN_NUMBER_OF_BYTES_IN_DM5_PACKET                      122
#define LC_MAX_NUMBER_OF_BYTES_IN_DM5_PACKET                      224

#define LC_MIN_NUMBER_OF_BYTES_IN_3_DH1_PACKET                      1
#define LC_MAX_NUMBER_OF_BYTES_IN_3_DH1_PACKET                     83

#define LC_MIN_NUMBER_OF_BYTES_IN_3_DH3_PACKET                     84
#define LC_MAX_NUMBER_OF_BYTES_IN_3_DH3_PACKET                    552

#define LC_MIN_NUMBER_OF_BYTES_IN_3_DH5_PACKET                    553
#define LC_MAX_NUMBER_OF_BYTES_IN_3_DH5_PACKET                   1021

#define LC_MIN_NUMBER_OF_BYTES_IN_2_DH1_PACKET                      1
#define LC_MAX_NUMBER_OF_BYTES_IN_2_DH1_PACKET                     54

#define LC_MIN_NUMBER_OF_BYTES_IN_2_DH3_PACKET                     53
#define LC_MAX_NUMBER_OF_BYTES_IN_2_DH3_PACKET                    367

#define LC_MIN_NUMBER_OF_BYTES_IN_2_DH5_PACKET                    368
#define LC_MAX_NUMBER_OF_BYTES_IN_2_DH5_PACKET                    679

#define LC_MAX_NUMBER_OF_BYTES_IN_BC_PACKET                        17

#else
#endif /* FIFO_WORD_WRITE */

#define EDR_PADDING                                                 0


#define LC_MAX_NUM_OF_PKTS_SUPPORTED                               16

/* These indexs are used for packet selection bitmap */

 /* This is just for LC */
#define LC_DV_INDEX                                                 0
#define LC_2_DH1_INDEX                                              1
#define LC_3_DH1_INDEX                                              2
#define LC_DM1_INDEX                                                3
#define LC_DH1_INDEX                                                4
#define LC_2_DH3_INDEX                                              8
#define LC_3_DH3_INDEX                                              9
#define LC_DM3_INDEX                                               10
#define LC_DH3_INDEX                                               11
#define LC_2_DH5_INDEX                                             12
#define LC_3_DH5_INDEX                                             13
#define LC_DM5_INDEX                                               14
#define LC_DH5_INDEX                                               15

#define LC_HOST_PACKET_TYPE_DM1                                0x0008
#define LC_HOST_PACKET_TYPE_DH1                                0x0010
#define LC_HOST_PACKET_TYPE_DM3                                0x0400
#define LC_HOST_PACKET_TYPE_DH3                                0x0800
#define LC_HOST_PACKET_TYPE_DM5                                0x4000
#define LC_HOST_PACKET_TYPE_DH5                                0x8000

/* we set to 1 because HW shares the same TX FIFO for ACL or Scatternet.
   HW scheduler will refer fw's tx command of BZDMA to copy data from SRAM
   to TX FIFO - austin */
#define LC_MAX_SCH_INFO   /* Make sure that this is 2^n */          2
#if (LC_MAX_SCH_INFO != 2)
#error "Disallow LC_MAX_SCH_INFO Setting for 0380 HW Scheduler"
#endif

#define LMP_MAX_UNPARK_TRAILS                                      10

/* The Unit formthe below calculation is .625ms => 2 slots */
#define LC_MSS_MIN_SLOT_DIFF                    8  /* 16 slots => 5 ms */
#define LC_MSS_CAN_WAIT_MIN_SLOT_DIFF           24 /* 48 + 16 slots => 36ms */

/************************ Structures ***********************************/

/*
 * This structure is used to keep all the required information
 * corresponding each active member used by firmware
 */
typedef struct {
    UCHAR bb_flow;
    /* This parameter contains sequence bit for the previously
    received CRC packet */
    UCHAR remote_seqn_bit;

    UINT16 index_in_CE;

    UINT16 upper_lut_address;
    UINT16 lower_lut_address;

    /* Used only in lut-8 and lut-9 and lut-10 and lut-11 */
    UCHAR bc_seqn_bit;
} LUT_EXTENSION_TABLE;

typedef struct {
    UINT16 payload_header;
    UINT16 packet_header;
    UCHAR flush_flag;

    /* For padding. */
    UCHAR dummy;

#ifdef COMPILE_RSSI_REPORTING
    /* Used only for FHS pkt. */
    INT16 rssi;
    UINT32 clk;
#endif

    UINT16 ce_index;
}PKT_HEADER ;


typedef struct {
    UCHAR read_pointer;
    UCHAR write_pointer;
    UINT16 dummy_16; /* For padding. */

    PKT_HEADER pkt_header_queue[LC_MAX_NUMBER_OF_HEADERS];
}PKT_HEADER_QUEUE;

/* LC enums for data transmission */

typedef enum {
    LC_NULL=0,
    LC_PDU,
    LC_L2CAP
}LC_PKT_SRC;

typedef enum {
    LC_TX_IDLE = 0,
    LC_TX_READY,
    LC_TX_SENT
}LC_TX_STATUS;

typedef enum  {
    SCHEDULED_PKT_TYPE_NONE = 0,
    SCHEDULED_PKT_TYPE_LMP_PDU,
    SCHEDULED_PKT_TYPE_ACLU_DATA
} SCHEDULED_PKT_TYPES;

typedef enum  {
    RESCHEDULE_FLAG_NONE = 0,
    RESCHEDULE_FLAG_ACL,
    RESCHEDULE_FLAG_ZERO_LENGTH
} RESCHEDULE_FLAGS;

/* The defination of Packet_Status_Flag of HCI Synchronous Data Packet in
   HCI spec - austin */
typedef enum {
    ERR_CORRECT_DATA, /* all (e)SCO rx pkts are marked as "good data" */
    ERR_INVALID_DATA, /* Some eSCO rx pkts are marked as "data with possible
                         errors" and all others are marked as "good data" */
    ERR_LOST_DATA,    /* all (e)SCO rx pkts are marked as "lost data"
                         (the corresponding HCI payload data shall all be 0) */
    ERR_PARTIAL_DATA  /* Some (e)SCO rx pkts are marked as "lost data"
                         (the corresponding HCI payload data shall be 0 in
                          missing (e)SCO pkts */
}ERR_STATUS_FLAGS;

typedef struct {
    BZDMA_TX_DESC_SEGMENT txdesc[BZDMA_UNICAST_ACL_TX_SEG];
    UINT32 fw_sent_time;    /* the sent time (BB clock) of scheduled pkt */
    UINT16 fw_sent_time_us; /* the sent time (us) of scheduled pkt */
    UINT16 frag_len;        /* the fragmented length */
    UINT16 pkt_type_lut;    /* the pkt type of BB (in LUT) */
    UINT16 lch;             /* the LLID indication of LC layer (in LUT) */
    UINT8 ce_index;         /* the index of connection entity */
    UINT8 txdesc_used_cnt; /* the used count of tx descriptor */
    UCHAR tx_status;        /* tx status */
    UCHAR pkt_src;          /* the packet source */
    UCHAR selected_am_addr; /* the lt_addr of selected remote device */
    UCHAR bc_flag;          /* the broadcast flag (h2c) */
    UCHAR tx_count;         /* the count of tx done interrupt to selected
                               remote device after we schedule the pkt to hw */
    UCHAR rx_count;         /* the count of rx pkt from selected remote device
                               after we schedule the pkt to hw */
    UCHAR flow_ctrl_count;  /* the flow control count of rx pkt from selected
                               remote device after we schedule the pkt to hw */
    UCHAR wait_count;       /* the counter for waiting */
    void* packet_ptr;       /* scheduled packet pointer */
} LC_SCHEDULED_PKT;

typedef struct {
    UCHAR rptr:1;                   /* read pointer (0/1) */
    UCHAR wptr:1;                   /* write pointer (0/1) */
    UCHAR rsvd:6;                   /* reserved */
    UCHAR lc_allowed_pkt_cnt;       /* allowed packet count */
    UCHAR donot_schedule_pkt;       /* does not scheduled packet count */
    UCHAR wait_for_clear_pkt_count; /* wait for clear packet count */
    LC_SCHEDULED_PKT lc_scheduled_pkt_info[LC_MAX_SCH_INFO];
}LC_PICONET_SCHEDULER;


typedef enum {
    LC_INQUIRY = 0x1,
    LC_PAGE = 0x2,
    LC_IDLE = 0x0
}LC_PICONET_STATE;

#define LC_CUR_STATE_IDLE             0x00
#define LC_CUR_STATE_INQUIRY          0x01
#define LC_CUR_STATE_PAGE             0x02

typedef enum {
    PICONET_MASTER = 0,
    PICONET_SLAVE,
    PICONET_IDLE
}LC_PICONET_ROLE;

typedef struct  {
    LC_PICONET_ROLE lc_piconet_role;
}SCATTERNET_TABLE;


#ifdef _CCH_LPS_
typedef enum {
    LPS_SNIFF_SM = 0,
    LPS_SNIFF_DSM,
    LPS_TIMER2_WITH_SCAN,
    LPS_TIMER2_WO_SCAN,
    LPS_WITH_WAKEUP_INSTANCE,
    LPS_WO_WAKEUP_INSTANCE
}LPS_STATUS;
#endif


/* In 0380, we can support maximum 4 piconets so need more bit be indication */
/* Here piconet id is physical piconet id */
#define LC_EXTRACT_PICONET_ID_FROM_BB(mode_sts_reg) \
        ((((mode_sts_reg) & 0x2000) >> 13) | (((mode_sts_reg) & 0x8000) >> 14))

/* Here piconet id is physical piconet id */
#define LC_EXTRACT_PICONET_ID_FROM_RX_STS_REG(packet_header, payload_header) \
            ((((packet_header) & 0x8000) >> 15) | \
             (((payload_header) & 0x0400) >> 9))

#define LC_EXTRACT_PICONET_ID_FROM_TX_STS_REG(read) \
            ((((read) & 0x0200) >> 9) | (((read) & 0x0800) >> 10))

#ifdef POWER_CONTROL

#define LC_INCR_POWER_LEVEL(power_level) \
        lc_incr_power_level(power_level)

#define LC_DECR_POWER_LEVEL(power_level) \
        lc_decr_power_level(power_level)

#define LC_CHECK_FOR_DECR_POWER_LEVEL(power_level)\
            lc_check_for_decr_power_level(power_level)

#define LC_CHECK_FOR_INCR_POWER_LEVEL(power_level)\
            lc_check_for_incr_power_level(power_level)

#endif

typedef struct {
    UINT16 type;
    UINT16 pkt_type;
    UCHAR max_slot;
}PKT_TYPE_INFO;
#define SYNC_FIFO_SIZE          (512)

typedef struct DUT_MODE_BUFFER_ {
	HCI_ACL_DATA_PKT *ppkt;
	UINT16 lut_type;
	UINT16 lut_lower;
	UINT16 len;
} DUT_MODE_BUFFER;

typedef struct DUT_MODE_MANAGER_ {
    union {
        struct {
        	UINT8 rx_buf_id:1;
        	UINT8 tx_buf_id:1;
        	UINT8 test_data_begin:1;
        	UINT8 rsvd:5;
        };
        UINT8 work_space;
    };

	DUT_MODE_BUFFER buf[2];
} DUT_MODE_MANAGER;

typedef struct BT_RX_INT_ARGUMENT_ {
    UINT16 payload_hdr;
    UINT16 packet_hdr;
#ifdef _DAPE_TEST_NEW_HW_READ_RSSI_STACK
    UINT16 rssi;
#endif
} BT_RX_INT_ARGUMENT;

typedef struct BB_RX_PARAMS_ {
    union {
        struct {
            UINT16 payload_header;
            UINT16 packet_header;
        };
        struct {
            BZ_REG_S_RX_PL_HDR pl_hdr;
            BZ_REG_S_RX_STATUS rx_status;
        };
    };
    UINT32 lcl_rx_pkt_type;
    UINT8 am_addr;
    UINT8 phy_piconet_id;
    UINT8 lut_index;
    UINT8 lc_baseband_flow;
    UINT8 rx_channel;
    UINT8 tx_channel;
    UINT16 rssi;
    UINT16 ce_index;
    UINT8 cur_schd_rptr;
    LC_SCHEDULED_PKT *schd;
} BB_RX_PARAMS;

#if defined(TEST_MODE) && defined(_DUT_DELAYED_LOOPBACK_MODE_)
extern DUT_MODE_MANAGER dut_mode_manager;
#endif

/**************************** Functions proto type *******************/

INLINE UCHAR lc_get_lut_index_from_ce_index(UINT16 ce_index);

UCHAR lc_start_inquiry(HCI_CMD_PKT *cmd_buffer);
void lc_start_periodic_inquiry(TimerHandle_t timer_handle);

void lc_start_write_scan_mode(UCHAR scan_enable);
void lc_write_scan_mode(UCHAR scan_enable);

UCHAR lc_start_paging(HCI_CMD_PKT *Cmd_buffer);
void lc_handle_random_backoff_timeout(TimerHandle_t timer_handle);

UCHAR lc_is_pkt_timeout(HCI_ACL_DATA_PKT *acl_pkt, LMP_CONNECTION_ENTITY *ce_ptr);

void lc_handle_ack_received(UINT8 schd_read_index, UCHAR phy_piconet_id, UINT32 debug_clock);

UINT8 lc_handle_received_packet_in_scatternet(void);

void lc_handle_inq_fhs_sent_intr(void);

#ifdef VER_1_1_SPECIFIC
void lc_handle_random_start_intr(void);
void lc_handle_inq_resp_timeout(void);
void lc_handle_random_end_intr(void);
#endif

void lc_kill_hardware_level_conn_in_scatternet(UINT16 ce_index);

UCHAR lc_send_packet_in_scatternet(UCHAR piconet_id);

void lc_handle_start_inq_scan_state(void);

UINT16 lc_generate_rand_number(UINT16 max_rand);

void lc_generate_parity_bits(UINT32 lap, UCHAR *parity_bits);

void lc_generate_true_parity_bits(UINT32 lap,UCHAR * parity_bits);

void lc_rx_task(OS_SIGNAL *signal_ptr);
void lc_tx_task(OS_SIGNAL *signal_ptr);

void lc_decide_packet_type(UINT16 *length, UINT16 *pkt_type_lut, UINT16 index);
void lc_handle_host_data_pkt(HCI_ACL_DATA_PKT *pkt);

void lc_get_clock_in_scatternet(UINT32 *native_clock, UCHAR phy_piconet_id);

void lc_reset_global_variables(void);

void lc_reset_lut_ex_table(UCHAR lut_index);
#ifdef _DAPE_TEST_NEW_HW_INSTRUCTION_KILL_SCAN
void lc_kill_inquiry_instruction(void);
void lc_kill_inquiry_scan_instruction(void);
void lc_kill_page_scan_instruction(void);
#endif
void lc_kill_scan_mode(void);

void lc_update_pkts_allowed(UINT16 ce_index);

SECTION_ISR_LOW void BB_handle_ito_interrupt(void);
SECTION_ISR void BB_handle_imode_interrupt(void);
SECTION_ISR void BB_handle_tx_interrupt(void);
SECTION_ISR void BB_handle_rx_interrupt(BT_RX_INT_ARGUMENT *argu);
SECTION_ISR void BB_handle_full_rx_interrupt(BT_RX_INT_ARGUMENT *argu);
SECTION_ISR void BB_handle_sniff_interrupt(UINT16 sniff_interrupt_status);
SECTION_ISR_LOW void BB_handle_access_code_miss_interrupt(void);
SECTION_ISR_LOW void bb_handle_slot_counter_intr(void);
#ifdef _LPS_WITH_MULTI_LINK_
SECTION_ISR_LOW void bb_handle_scan_end_intr(UINT16 reg_int);
#endif

void AND_val_with_bb_reg(UINT16 reg_offset, UINT16 val);
void OR_val_with_bb_reg(UINT16 reg_offset, UINT16 val);

void lc_free_lmp_pdus(UINT16 ce_index, UCHAR all_flag);

void lc_program_beacon(UINT16 ce_index);
API_RESULT lc_init(void);
API_RESULT lc_shutdown(void);
void lc_invoke_scheduler(UCHAR piconet_id);


#ifdef TEST_MODE
void lc_generate_and_send_dut_burst_data(UINT16 ce_index, UCHAR piconet_id);
#endif /* TEST_MODE */

#ifdef POWER_CONTROL
UCHAR lc_incr_power_level(UCHAR power_level);
UCHAR lc_decr_power_level(UCHAR power_level);
UCHAR lc_check_for_incr_power_level(UCHAR power_level);
UCHAR lc_check_for_decr_power_level(UCHAR power_level);
#endif

UCHAR lc_program_tolerance(UINT16 inactive_slots, UINT16 ce_index);

#ifdef COMPILE_DYNAMIC_POLLING
void lc_update_dynamic_polling(void);
void lc_program_dynamic_tpoll(UINT16 ce_index);
#endif

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
void lc_init_radio(UINT8 dlps_flow);
#else
void lc_init_radio(void);
#endif

#ifdef _MODEM_LNA_CONSTRAINT_
void lc_set_modem_lna_constraint_on_off(UINT8 is_on);
void lc_force_modem_lna_constraint_off(void);
#endif

#ifdef _NEW_MODEM_DESIGN_PHY_SETTING_
typedef struct EFUSE_MODEM_SETTING_1_S_ {
    UINT8 not_init_old_agc_table:1;          // [0] 1b; (default: 1);  1: Not initialize the OLD RXAGC Table
    UINT8 not_init_new_agc_table:1;          // [1] 1b; (default: 0);  1: Not initialize the NEW RXAGC Table
  #ifdef _NEW_MODEM_PI_ACCESS_BY_VENDOR_REG_
    UINT8 modem_agc_init_by_pi:1;            // [2] 1b; (default: 1);
    UINT8 rsvd:1;                            // [3] 1b;
  #else
    UINT8 modem_pi_enable:1;                 // [2] 1b; (default: 1);
    UINT8 modem_reg_rw_fast_mode:1;          // [3] 1b; (default: 1) 1: fasten the RTK_READ/WRITE_MODEM_REG() with PI
  #endif
    UINT8 modem_lna_con_on_by_bk_reg:1;      // [4] 1b; (default: 0 ?)
    UINT8 modem_lna_con_on_value:3;          // [7:5] 3b; (default: 0); reg_lna_constraint value for WiFi-base LNA-backoff-control
} EFUSE_MODEM_SETTING_1_S;
#endif

#if 0
typedef struct EFUSE_RF_SETTING_1_S_ {
#if 1
    UINT8 rsvd:8;                           // [7:0]
#else
    UINT8 execute_lok_at_boot:1;            // [0] 1b;
    UINT8 execute_lok_at_boot_avg_num:3;    // [3:1] 3b; average times = 2^x
    UINT8 execute_lok_at_boot_dummy_lok:1;  // [4] 1b;
    UINT8 lok_dont_set_rf_off:1;              // [5] 1b;
    UINT8 lok_force_adda_idle_on:1;         // [6] 1b;
    UINT8 rsvd:1;                           // [7]
#endif
} EFUSE_RF_SETTING_1_S;
#endif

#ifdef _YL_MODEM_RSSI_MAPPING
UINT8 lc_modem_rssi_mapping(UINT8 rssi_with_offset);
#endif

extern UINT8 g_thermal_track_pause;


#if 0
#ifdef _MODEM_HCI_VENDOR_8821A_LOK_
UINT16 rtl8821_btrf_lok(UINT8 delay_ms, UINT8 lok_rf_ch, UINT8 lok_tx_gain);
#endif
#endif

#define EFUSE_INVALID_TXSCALEFACTOR 0xFE
#define EFUSE_INVALID_TXDAC_CURRENT 0xFE


#ifdef _NEW_MODEM_DESIGN_
void pf_sram_modem_access_enable(void);
void pf_sram_modem_access_disable(void);

#define NEW_MODEM_IF_1P4    0x23D71
#define NEW_MODEM_IF_N1P4   0xDC28F
#define NEW_MODEM_IF_1P6    0x28F5C
#define NEW_MODEM_IF_N1P6   0xD70A4
#define NEW_MODEM_IF_2P5    0x40000
#define NEW_MODEM_IF_N2P5   0xC0000

typedef union OTP_OLD_AGC_TABLE_S {
    UINT16 d16;
    struct
    {
        UINT16 addr:8;                      //[7:0]
        UINT16 value:8;                     //[15:8]
    };
}OTP_OLD_AGC_TABLE_S_TYPE;
typedef union MODEM_AGC_TABLE_S {
    UINT16 d16;
    struct
    {
        UINT16 addr:6;                      //[5:0]
        UINT16 value:10;                    //[15:6]
    };
}MODEM_AGC_TABLE_S_TYPE;

typedef union MODEM_REG_S {
    UINT16 d16;
    struct
    {
        UINT16 reg_sram_debug_en:1;         //[0]
        UINT16 reg_sram_soft_rst:1;         //[1]
        UINT16 rsvd1:2;                     //[3:2]
        UINT16 r_AGC_pw_mon_th:2;           //[5:4]
        UINT16 r_CFOE_break_en:1;           //[6]
        UINT16 reg_sram_debug_mode:3;       //[9:7]; refer to MODEM_SRAM_DEBUG_ELEMENT_S_TYPE
        UINT16 rsvd2:1;                     //[10]
        UINT16 r_AGC_sat_num:1;             //[11]
        UINT16 reg_sram_pktg_size:3;        //[14:12] 0:16 1:32 2:64 3:128 max saving points for each packet , 7: only buffer one packet (for adc_re)
        UINT16 reg_sram_debug_filter_en:1;  //[15] 1: for filter out unwanted data into sram
    }reg08;
    struct
    {
        UINT16 r_wifi_si_en_fpga:1;         //[0] FOR FPGA ONLY, 1: for WIFI SI format, 20-bit DATA
        UINT16 r_wifi_msb_fpga:4;           //[4:1] FOR FPGA ONLY, Data[19:16] for WIFI SI
#ifdef _NEW_MODEM_DESIGN_AFTER_RTL8821B_TC_
        UINT16 rsvd1:1;                     //[5]
        UINT16 reg_lna_constraint:3;        //[8:6]
#else
        UINT16 rsvd1:4;                     //[8:5]
#endif
        UINT16 r_DPSK_sync_break_en:1;      //[9]
        UINT16 rsvd2:1;                     //[10]
        UINT16 r_dbg_port_select:5;         //[15:11]
    }reg0a;
    struct
    {
        UINT16 r_tx_mode1:4;                //[3:0]
        UINT16 r_tx_mode2:4;                //[7:4]
        UINT16 r_rx_mode1:4;                //[11:8]
        UINT16 r_rx_mode2:4;                //[15:12]
    }reg0c;
    struct
    {
        UINT16 r_rx_mode3:4;                //[3:0]
        UINT16 r_rx_mode4:4;                //[7:4]
        UINT16 r_rx_mode5:4;                //[11:8]
        UINT16 rsvd:2;                      //[13:12]
        UINT16 r_DFIR_corner_TX:2;          //[16:14]
    }reg0e;
    struct
    {
        UINT16 r_sync_GIAC_en:1;            //[0]
        UINT16 r_afe_test_oe:1;             //[1]
        UINT16 rsvd1:2;                     //[3:2]
        UINT16 r_afe_test_mode:4;           //[7:4]
        UINT16 reg_psd_mode_en:1;           //[8]
        UINT16 reg_psd_sram_report_addr_offsetx2:7; //[15:9]
    }reg14;
  #ifndef _NEW_MODEM_DESIGN_AFTER_RTL8821B_TC_
    struct
    {
        UINT16 r_tx_mode1_bt4:4;            //[3:0]
        UINT16 r_tx_mode2_bt4:4;            //[7:4]
        UINT16 r_rx_mode1_bt4:4;            //[11:8]
        UINT16 r_rx_mode2_bt4:4;            //[15:12]
    }reg36;
    struct
    {
        UINT16 r_rx_mode3_bt4:4;            //[3:0]
        UINT16 r_rx_mode4_bt4:4;            //[7:4]
        UINT16 r_rx_mode5_bt4:4;            //[11:8]
        UINT16 rsvd:4;                      //[15:12]
    }reg3a;
  #endif
    struct
    {
        UINT16 r_sync_mf_th_BT4:5;          //[4:0]
        UINT16 r_BT4:1;                     //[5]
        UINT16 r_table_sel:2;               //[7:6]
        UINT16 r_aagc_8723A_en:1;           //[8]
        UINT16 r_txmode_dly:2;              //[10:9]
        UINT16 r_sync_BT4:1;                //[11]
        UINT16 reg_tx_gain_set:1;           //[12] 1: to fix TX Gain Bug, Modem will overwrite the TX Gain setting (valid after RTL8821A-MP, 8723B-MP, 2801-MP)
        UINT16 rsvd1:1;                     //[13]
        UINT16 rsvd2:2;                     //[15:14]
    }reg3c;
    struct
    {
        UINT16 AFE_AD_0_TX:4;               //[3:0]
        UINT16 AFE_AD_0_RX:4;               //[7:4]
        UINT16 AFE_DA_TX:2;                 //[9:8]
        UINT16 AFE_DA_RX:2;                 //[11:10]
        UINT16 r_RF_page_sel:1;             //[12] 1 for page selection of RFC @ FPGA mode only
        UINT16 rsvd:3;                      //[15:13]
    }reg62;
    struct
    {
        UINT16 r_ini_gain_mp:6;             //[5:0]
        UINT16 others:10;                   //[15:6]
    }reg66;
    /*===== page 2 =====*/
    struct
    {
        UINT16 reg_btm_if_freq_bt2_15_0:16; //[15:0]  S(20,21f)*20MHz
    }p2reg08;
    struct
    {
        UINT16 reg_btm_if_freq_bt2_19_16:4; //[3:0]
        UINT16 reg_btm_bt4_dfir_bw:3;       //[6:4]
        UINT16 reg_btm_gfsk_dfir_bw:3;      //[9:7]
        UINT16 reg_btm_dpsk_dfir_bw:3;      //[12:10]
        UINT16 rsvd:3;                      //[15:13]
    }p2reg0a;
    struct
    {
        UINT16 reg_tx_if_freq:16;           //[15:0]  S(16,16f)*10MHz
    }p2reg2e;
    struct
    {
        UINT16 reg_btm_if_freq_bt4_15_0:16; //[15:0]  S(20,21f)*20MHz
    }p2reg30;
    struct
    {
        UINT16 reg_btm_if_freq_bt4_19_16:4; //[3:0]
#ifdef _YL_NEW_PSD_REG_AFTER_8703B_    
        UINT16 reg_if_bw_bt2:3;             //[6:4]
        UINT16 reg_if_bw_bt4:3;             //[9:7]
        UINT16 rsvd1:6;                     //[15:10]
#else
        UINT16 rsvd:12;                     //[15:4]
#endif
    }p2reg32;
    struct
    {
        UINT16 reg_bb_pw_time_bt4:3;        //[2:0]
        UINT16 reg_bb_pw_time_bt2:3;        //[5:3]
        UINT16 reg_bb_pw_time2_bt4:3;       //[8:6]
        UINT16 reg_bb_pw_time2_bt2:3;       //[11:9]
        UINT16 reg_agc_stop:1;              //[12]
        UINT16 reg_share_lna_en:1;          //[13]
        UINT16 reg_sharelna_adc_gat_en:1;   //[14]
        UINT16 rsvd:1;                      //[15]
    }p2reg34;
    struct
    {
        UINT16 reg_aagc_fix_gain_idx:7;     //[6:0]
        UINT16 reg_aagc_fix_lna_idx:3;      //[9:7]
        UINT16 reg_aagc_fix_en:1;           //[10]
        UINT16 reg_aagc_period:5;           //[15:11]
    }p2reg36;
    struct
    {
        UINT16 reg_aagc_fix_gain_idx1:5;     //[4:0]
        UINT16 reg_aagc_fix_lna_idx1:3;      //[7:5]
        UINT16 reg_aagc_fix_gain_idx2:5;     //[12:8]
        UINT16 reg_aagc_fix_lna_idx2:3;      //[15:13]
    }p2reg38;
    struct
    {
        UINT16 reg_aagc_fix_gain_idx3:5;     //[4:0]
        UINT16 reg_aagc_fix_lna_idx3:3;      //[7:5]
        UINT16 reg_aagc_fix_gain_idx4:5;     //[12:8]
        UINT16 reg_aagc_fix_lna_idx4:3;      //[15:13]
    }p2reg3A;
    struct
    {
#ifdef _YL_NEW_PSD_REG_AFTER_8703B_
        UINT16 rsvd1:16;                     //[15:0]
#else
        UINT16 reg_if_bw_bt2:3;              //[2:0]
        UINT16 reg_if_bw_bt4:3;              //[5:3]
        UINT16 rsvd:10;                      //[15:6]
#endif
    }p2reg3c;
    struct
    {
        UINT16 reg_settling_time_bt2_2:6;   //[5:0]
        UINT16 reg_settling_time_bt2_3:6;   //[11:6]
  #ifdef _NEW_MODEM_DESIGN_AFTER_RTL8821B_TC_
        UINT16 rsvd:4;                      //[15:12]
  #else
        UINT16 reg_lna_constraint:3;        //[14:12]
        UINT16 rsvd:1;                      //[15]
  #endif
    }p2reg48;
  #ifdef _YL_EXTEND_MODEM_SRAM_ADDR_14BIT_    
    struct
    {
        UINT16 reg_settling_time_bt4_2:6;   //[5:0]
        UINT16 reg_settling_time_bt4_3:6;   //[11:6]
        UINT16 reg_psd_sram_report_addr_base_msb:1; //[12]
        UINT16 rsvd1:3;                     //[15:13]
    }p2reg50;
  #endif    
    struct
    {
        UINT16 reg_sram_start_addr:13;      //[12:0] sram start address for modem debug
        UINT16 rsvd:3;                      //[15:13]
    }p2reg52;
    struct
    {
        UINT16 reg_sram_end_addr:13;        //[12:0] sram end address for modem debug
        UINT16 reg_sram_debug_pattern:1;    //[13]
        UINT16 reg_sram_adc_rate:1;         //[14] 0 for 20M , 1 for 10M
        UINT16 reg_sram_adc_msb:1;          //[15] valid for 8821A-MP;  0 for sync_done , 1 for hssi as msb bit;
    }p2reg54;
#ifdef _NEW_MODEM_PSD_SCAN_
    struct
    {
        UINT16 reg_psd_sram_report_addr_base:13; //[12:0] sran base address for psd report
        UINT16 reg_psd_agc_time:3;          //[15:13] 0 for aagc off ; 1~6 for wait 2us*2^(1~6) before stop aggc ; 7 aagc free run
    }p2reg56;
    struct
    {
        UINT16 reg_psd_cal_dly:3;           //[2:0] "wait time before calculating PSD ; 2us*2^(0~7) + 4uswhen reg_psd_agc_time is 0~6: reg_psd_cal_dly should >= reg_psd_agc_time"
        UINT16 reg_psd_cal_opt:2;           //[4:3] 0/1/2/3 for 16/32/64/128 DFT points for PSD ( dcm2 @ 10M)
        UINT16 reg_psd_iq2pwr_opt:1;        //5] ABS option for dcm2 ; 0 for I^2+Q^2 , 1 for max + min /2
        UINT16 reg_psd_iq2pwr2_opt:1;       //[6] ABS option for dcm4 ; 0 for I^2+Q^2 , 1 for max + min /2
        UINT16 reg_psd_avg_opt:2;           //[8:7] DFT average times for dcm2 @10M; 0~3 for average 4/8/16/32
        UINT16 reg_psd_avg2_opt:3;          //[11:9] DFT average times for dcm4 @  5M; 0~7 for average 32/64/128/256/512/1024/2048/4096
        UINT16 reg_psd_avg_scale:3;         //[14:12] Scaled value (0~5) for DFT averaged value (dcm2); scale = 2^x
        UINT16 reg_psd_fix_gain:1;          //[15] 1 for fix gain; Used by AGC FSM
    }p2reg58;
    struct
    {
        UINT16 reg_btm_if_freq_psd:16;      //[15:0] "IF frequency setting @ PSD; s(16,17f)*20MHz  16'h47AE: 1.4MHz  7"
    }p2reg5a;
    struct
    {
        UINT16 reg_psd_deltaf:8;            //[7:0] "delta value for phase accumulation for postive and negtive band; s(8,8f)*5MHz8'h33: 1MHz"
        UINT16 reg_if_bw_psd:3;             //[10:8] RFC IF/BW (RFC0x00[2:0]) for PSD Mode
        UINT16 reg_psd_en_bpf_psd:1;		//[11] "RFC EN_BPF_PSD (RFC0x01[2]) for PSD Mode;1: double the RFC BPF IF/BW (Note: the Synthesizer is not affected)"
        UINT16 reg_btm_dfir_bw_psd:3;       //[14:12] U(3,0f), DFIR bandwidth for PSD Mode
#ifdef _YL_NEW_PSD_REG_AFTER_8703B_
        UINT16 reg_psd_share_lna_en:1;      //[15]
#else
        UINT16 rsvd:1;                      //[15]
#endif        
    }p2reg5c;
#ifdef _YL_NEW_PSD_REG_AFTER_8703B_
  #ifdef _YL_EXTEND_MODEM_SRAM_ADDR_14BIT_
    struct
    {
        UINT16 reg_sram_start_addr_msb:1;   //[0]
        UINT16 reg_sram_end_addr_msb:1;     //[1]
        UINT16 rsvd1:14;                    //[15:2]
    }p2reg5e;
  #else
    struct
    {
        UINT16 rsvd1:16;                    //[15:0]
    }p2reg5e;
  #endif
#else
    struct
    {
        UINT16 reg_psd_ini_gain_mp:6;       //[5:0]
        UINT16 reg_psd_share_lna_en:1;      //[6]
        UINT16 rsvd:9;                      //[15:7]
    }p2reg5e;
#endif    
  #ifdef _NEW_MODEM_DESIGN_AFTER_RTL8821B_TC_
    struct
    {
        UINT16 r_tx_mode1_bt4:4;            //[4:0]
        UINT16 r_tx_mode2_bt4:4;            //[9:5]
        UINT16 r_rx_mode1_bt4:4;            //[10]
        UINT16 r_rx_mode2_bt4:4;            //[11]
    }p3reg02;
    struct
    {
        UINT16 r_rx_mode3_bt4:4;            //[4:0]
        UINT16 r_rx_mode4_bt4:4;            //[9:5]
        UINT16 r_rx_mode5_bt4:4;            //[10]
        UINT16 rsvd:4;                      //[15:12]
    }p3reg04;
#ifdef _YL_NEW_PSD_REG_AFTER_8703B_    
    struct
    {
        UINT16 reg_psd_scale_margin:3;      //[2:0]
        UINT16 reg_psd_scale_en:1;          //[3]
        UINT16 reg_pre_dfir_scale_margin:3; //[6:4]
        UINT16 reg_pre_dfir_scale_en:1;     //[7]
        UINT16 reg_psd_ini_gain_mp:6;       //[13:8]
        UINT16 rsvd1:2;                     //[15:14]
    }p3reg06;
#endif
    struct
    {
        UINT16 reg_adc_backoff:5;           //[4:0]
        UINT16 reg_if_pw_offset:5;          //[9:5]
        UINT16 reg_wifi_txon_en:1;          //[10]
        UINT16 reg_wifi_txon_mode:1;        //[11]
        UINT16 rsvd:4;                      //[15:12]
    }p3reg3a;
  #endif
#endif
#ifdef _YL_NEW_PSD_REG_AFTER_8703B_    
    struct
    {
        UINT16 reg_pd_coeff_b1_le2m:11;     //[10:0]
        UINT16 reg_if_bw_le2m:3;            //[13:11]
        UINT16 rsvd1:2;                     //[15:14]
    }p6reg1e;
#endif    
#if 0
    struct
    {
        UINT16 reg_sram_pkt_num:9;          //[8:0] (end_addr-start_addr)/pkt_size
        UINT16 rsvd:7;                      //[15:9]
    }p2reg56;
#endif
}MODEM_REG_S_TYPE;

#ifdef _NEW_MODEM_PSD_SCAN_
// for MODEM PSD SCAN IF Setting: pwreg5a.reg_btm_if_freq_psd //
#define MODEM_PSD_IF_1P4                  0x23D7
#define MODEM_PSD_IF_1P6                  0x28F5
#define MODEM_PSD_IF_2P5                  0x4000
#define MODEM_PSD_IF_2P8                  0x47AE
#define MODEM_PSD_IF_3P0                  0x4CCC
#define MODEM_PSD_IF_3P2                  0x51EB
#define MODEM_PSD_MODE_HALF_SLOT          0
#define MODEM_PSD_MODE_FULL_SLOT          1

// TODO: may be re-defined in PATCH, for different IC or bug-fixing
typedef union MODEM_PSD_REPORT_ELEMENT_S{
    UINT32 d32_array[4];
#if 0
    struct // RTL8761A
    {
        UINT32 psd_avg_dc:16;               //[15:0]
        UINT32 psd_avg_pos:16;              ///[31:16]
        UINT32 psd_avg_neg:16;              //[47:32]
        UINT32 mp_gain_idx:6;               //[53:48]
        UINT32 gnt_bt_hold:1;               //[54]
        UINT32 gnt_bt_final:1;              //[55]
        UINT32 rsvd1:8;                     //[63:56]
        UINT32 psd_avg_dfir_lsb:16;         //[79:64]
        UINT32 psd_avg_dfir_msb:4;          //[83:80]
        UINT32 psd_clip_ratio_start:6;      //[89:84]
        UINT32 psd_clip_ratio_end:6;        //[95:90]
        UINT32 agc_ste:4;                   //[99:96]
        UINT32 lna_idx_use:3;               //[102:100]
        UINT32 bpf_mode_use:1;              //[103]
        UINT32 gain_idx_use:5;              //[108:104]
        UINT32 rf_pd_en_use:1;              //[109]
        UINT32 rsvd2:2;                     //[111:110]
        UINT32 rsvd3:16;                    //[127:112]
    };
#else
    struct // RTL8703B, RTL8822B, RTL8821B-MP
    {
        UINT32 psd_avg_dc:16;               //[15:0]
        UINT32 psd_avg_pos:16;              ///[31:16]
        UINT32 psd_avg_neg:16;              //[47:32]
        UINT32 mp_gain_idx:6;               //[53:48]
        UINT32 gnt_bt_hold:1;               //[54]
        UINT32 gnt_bt_final:1;              //[55]

        UINT32 psd_u28_scale:3;             //[58:56]
        UINT32 gnt_wl_hold:1;               //[59]
        UINT32 gnt_wl_final:1;              //[60]
        UINT32 wifi_txon_hold:1;            //[61]
        UINT32 wifi_txon_final:1;           //[62]
        UINT32 rsvd1:1;                     //[63]

        UINT32 psd_avg_dfir_lsb:16;         //[79:64]
        UINT32 psd_avg_dfir_msb:4;          //[83:80]
        UINT32 psd_clip_ratio_start:6;      //[89:84]
        UINT32 psd_clip_ratio_end:6;        //[95:90]
        UINT32 agc_ste:4;                   //[99:96]
        UINT32 lna_idx_use:3;               //[102:100]
        UINT32 bpf_mode_use:1;              //[103]
        UINT32 gain_idx_use:5;              //[108:104]
        UINT32 rf_pd_en_use:1;              //[109]
        UINT32 rsvd2:2;                     //[111:110]
        UINT32 rsvd3:16;                    //[127:112]
    };
#endif
}MODEM_PSD_REPORT_ELEMENT_S_TYPE;

typedef struct MODEM_PSD_FW_RECORD_ELEMENT_S {
    UINT32 psd;
    UINT8 mp_gain_idx;
} MODEM_PSD_FW_RECORD_ELEMENT_S_TYPE;

void lc_psd_modem_set_agc(UINT8 psd_cal_dly, UINT8 psd_agc_time, UINT8 psd_fix_gain, UINT8 psd_ini_gain_mp);
void lc_psd_bluewiz_set_parameters(UINT8 psd_ch_start, UINT8 psd_ch_step, UINT8 psd_ch_end, UINT8 psd_mode, UINT16 psd_timeout_us);
void lc_psd_set_psd_en(UINT8 psd_en);


//extern ALIGN(16) SECTION_SRAM MODEM_PSD_REPORT_ELEMENT_S_TYPE g_modem_psd_report_array[MODEM_PSD_REPORT_ENTRY_NUM];
extern MODEM_PSD_REPORT_ELEMENT_S_TYPE* g_modem_psd_report_array;
extern UINT16 g_modem_psd_report_entry_num;
void lc_psd_modem_init(void);
INT8 log10x10(UINT16 xin);
UINT8 modem_psd_psdavg2dbm(UINT16 psd_avg, INT8 mp_gain_idx);

#endif


#define MODEM_SRAM_DEBUG_PKT_SIZE_16      0
#define MODEM_SRAM_DEBUG_PKT_SIZE_32      1
#define MODEM_SRAM_DEBUG_PKT_SIZE_64      2
#define MODEM_SRAM_DEBUG_PKT_SIZE_128     3
#define MODEM_SRAM_DEBUG_PKT_SIZE_1PKT    7
#define MODEM_SRAM_DEBUG_ENDING_PATTERN 0x00000000FFFFFFFF


typedef union MODEM_SRAM_DEBUG_ELEMENT_S {
    UINT32 d32_array[2];
    struct
    {
        UINT32 adc_re7:8;                   //[7:0] 8821A-TEST: sign-ext; 8821A-MP: {sync/hssi,s(7,0f)}
        UINT32 adc_re6:8;                   //[15:8]
        UINT32 adc_re5:8;                   //[23:16]
        UINT32 adc_re4:8;                   //[31:24]
        UINT32 adc_re3:8;                   //[39:32]
        UINT32 adc_re2:8;                   //[47:40]
        UINT32 adc_re1:8;                   //[55:48]
        UINT32 adc_re0:8;                   //[63:56]
    }mode0; // ADC @ 20MHz
    struct
    {
        UINT32 agc_ste:4;                   //bit[3:0]   u(4,0f)
        UINT32 lna_idx_use:3;               //bit[6:4]   u(3,0f)
        UINT32 gain_idx_use:5;              //bit[11:7]  u(5,0f)
        UINT32 mp_gain_idx:6;               //bit[17:12] u(6,0f)
        UINT32 pga_gain_offset:6;           //bit[23:18] s(6,0f)
        UINT32 lna_constraint:3;            //bit[26:24] u(3,0f)
        UINT32 share_lna:1;                 //bit[27]    u(1,0f)
        UINT32 lna_wifi:3;                  //bit[30:28] u(3,0f)
        UINT32 bt_rfpd_0:1;                 //bit[31]    u(4,0f)
        UINT32 bt_rfpd_3_1:3;               //bit[34:32]
        UINT32 bpf_mode_use:1;              //bit[35]    u(1,0f)
        UINT32 rf_pd_en_use:1;              //bit[36]    u(1,0f)
        UINT32 sync_done:1;                 //bit[37]    u(1,0f)
        UINT32 cfoe_post_fir_80M:15;        //bit[52:38] s(15,18f)
        UINT32 rsvd:11;                     //bit[63:53]
    }mode1; // AGC signals @ (10MHz and agc_ste changes)
    struct
    {
        UINT32 dc_acc:21;                   //[20:0] // (dc_acc_r @matlab)
        UINT32 eq_out_d_10_0:11;            //[31:21] // (eq_out_gfsk @matlab)
        UINT32 eq_out_d_15_11:5;            //[36:32] // (eq_out_gfsk @matlab)
        UINT32 cfoe_post_fir_14_3:12;       //[48:37]
        UINT32 cfoe_pre_fir_14_3:12;        //[60:49]
        UINT32 rsvd:3;                      //[63:61]
    }mode2; // GFSK signals @ 1MHz
    struct
    {
        UINT32 ffe_gfsk_coef2_20_9:12;      //bit[11:0]
        UINT32 ffe_gfsk_coef3_20_9:12;      //bit[23:12]
        UINT32 ffe_gfsk_coef4_16_9:8;       //bit[31:24]
	    UINT32 ffe_gfsk_coef4_20_17:4;      //bit[35:32]
        UINT32 ffe_gfsk_coef5_20_9:12;      //bit[47:36]
        UINT32 ffe_gfsk_coef6_20_9:12;      //bit[59:48]
	    UINT32 rsvd:4;                      //bit[63:60]
    }mode3; // GFSK signals @ 1MHz
    struct
    {
        UINT32 ffe_gfsk_coef7_20_9:12;       //bit[11:0]
        UINT32 ffe_gfsk_coef8_20_9:12;       //bit[23:12]
        UINT32 fbf_gfsk_coef1_16_9:8;        //bit[31:24]
	    UINT32 fbf_gfsk_coef1_20_17:4;       //bit[35:32]
        UINT32 fbf_gfsk_coef2_20_9:12;       //bit[47:36]
        UINT32 fbf_gfsk_coef3_20_9:12;       //bit[59:48]
	    UINT32 rsvd:4;                       //bit[63:60]
    }mode4; // GFSK signals @ 1MHz
	struct
    {
        UINT32 zk_re:12;                    //bit[11:0]
        UINT32 zk_im:12;                    //bit[23:12]
        UINT32 ki_acc_r_11_4:8;             //bit[31:24]
	    UINT32 ki_acc_r_19_12:8;            //bit[39:32]
        UINT32 eq_out_re:12;                //bit[51:40]
        UINT32 eq_out_im:12;                //bit[63:52]
    }mode5; // DPSK signals @ 1MHz
    struct
    {
        UINT32 eq_coef0_qz_re:12;           //bit[11:0]
        UINT32 eq_coef0_qz_im:12;           //bit[23:12]
        UINT32 eq_coef1_qz_re_7_0:8;        //bit[31:24]
	    UINT32 eq_coef1_qz_re_11_8:4;       //bit[35:32]
        UINT32 eq_coef2_qz_im:12;           //bit[47:36]
        UINT32 eq_coef2_qz_re:12;           //bit[59:48]
	    UINT32 rsvd:4;                      //bit[63:60]
    }mode6; // DPSK signals @ 1MHz
}MODEM_SRAM_DEBUG_ELEMENT_S_TYPE;


#ifdef _YL_NEW_MODEM_SRAM_DEBUG
//#define MODEM_SRAM_DBG_ENTRY_NUM (256*2)
#ifdef _DAPE_MODEM_SRAM_DBG_VER2
typedef enum
{
    MODEM_SRAM_DEBUG_MODE_ADC = 0x00,       /*0 *< ADC debug */
    MODEM_SRAM_DEBUG_MODE_AGC,              /*1 *< AGC debug */
    MODEM_SRAM_DEBUG_MODE_GFSK_A,           /*2 *< GFSK_A debug */
    MODEM_SRAM_DEBUG_MODE_GFSK_B,           /*3 *< GFSK_B debug */
    MODEM_SRAM_DEBUG_MODE_GFSK_C,           /*4 *< GFSK_C debug */
    MODEM_SRAM_DEBUG_MODE_EDR_A,            /*5 *< DPSK_A debug */
    MODEM_SRAM_DEBUG_MODE_EDR_B,            /*6 *< DPSK_B debug */
    MODEM_SRAM_DEBUG_MODE_INVALID = 0xff    /**< Invalid Radio Type */
} MODEM_DBG_TYPE;
#else
#define MODEM_SRAM_DEBUG_MODE_ADC 0
#define MODEM_SRAM_DEBUG_MODE_AGC 1
#define MODEM_SRAM_DEBUG_MODE_EDR_A 5
#endif
#define MODEM_SRAM_DEBUG_MODE_SEL MODEM_SRAM_DEBUG_MODE_EDR_A
#define MODEM_SRAM_DEBUG_ADC_RATE_10M 1
#define MODEM_SRAM_DEBUG_ADC_RATE_20M 0
#if _DAPE_MODEM_SRAM_DBG_VER2
#define MODEM_SRAM_DEBUG_ADC_RATE_SEL MODEM_SRAM_DEBUG_ADC_RATE_20M
#else
#define MODEM_SRAM_DEBUG_ADC_RATE_SEL MODEM_SRAM_DEBUG_ADC_RATE_10M
#endif
//#define MODEM_SRAM_DBG_BASE_ADDR 0x80000000
//extern SECTION_SRAM ALIGN(2048) MODEM_SRAM_DEBUG_ELEMENT_S_TYPE g_modem_sram_debug_array[MODEM_SRAM_DBG_ENTRY_NUM];
extern MODEM_SRAM_DEBUG_ELEMENT_S_TYPE* g_modem_sram_debug_array;
extern UINT16 g_modem_sram_debug_captured_flag;
extern UINT16 g_modem_sram_debug_log_count;    // DBSS variables would be initialized as 0 //
extern UINT8 g_modem_sram_debug_en;            // DBSS variables would be initialized as 0 //
extern UINT8 g_modem_sram_debug_log_en;
extern UINT8 g_modem_sram_debug_xdh5_trig_crc_ok;
extern UINT8 g_modem_sram_debug_xdh5_trig_en;
extern UINT8 g_modem_sram_debug_xdh5_xdh3_3dh1_error_log_en;
extern UINT8 g_modem_sram_debug_le_trig_en;
extern UINT8 g_modem_sram_debug_le_trig_crc_ok;
#ifdef _DAPE_MODEM_SRAM_DBG_VER2
extern UINT8 g_modem_sram_debug_modem_test;
extern UINT8 g_modem_sram_debug_le_normal_link_trig;
void rtl8821_btrf_modem_sram_debug_init(MODEM_SRAM_DEBUG_ELEMENT_S_TYPE* modem_sram_debug_array, UINT8 mode);
#else
void rtl8821_btrf_modem_sram_debug_init(MODEM_SRAM_DEBUG_ELEMENT_S_TYPE* modem_sram_debug_array);
#endif
void rtl8821_btrf_modem_sram_debug_set_mode(UINT8 value);
void rtl8821_btrf_modem_sram_debug_set_size(UINT8 value);
void rtl8821_btrf_modem_sram_debug_set_en(UINT8 value);
UINT8 rtl8821_btrf_modem_sram_debug_get_en(void);
void rtl8821_btrf_modem_sram_debug_set_rst(UINT8 value);
void rtl8821_btrf_modem_sram_debug_test_log(MODEM_SRAM_DEBUG_ELEMENT_S_TYPE* modem_sram_debug_array);
#endif

#ifdef _YL_TEST_NEW_MODEM_PI_ACCESS_
void lc_modem_pi_access_test_log(UINT32 dbg_count);
#endif
#endif


#ifdef _NEW_MODEM_PI_ACCESS_
UCHAR lc_read_modem_pi_page(void);
void lc_write_modem_pi_page(UCHAR modem_page);
UINT32 rtk_read_modem_radio_reg_pi(UCHAR modem_page, UCHAR addr, UCHAR type);
//void rtk_write_modem_radio_reg_pi(UCHAR modem_page, UCHAR addr, UCHAR type, UINT32 val);
void rtk_write_modem_radio_reg_pi(UCHAR modem_page, UCHAR addr, UCHAR type, UINT16 val);
#endif

void lc_write_radio_reg(UCHAR addr, UINT32 Val);
UINT32 lc_read_radio_reg(UCHAR addr);

#define TYPE_RF         0x0
#define TYPE_MODEM      0x1
UINT32 rtk_read_modem_radio_reg(UCHAR addr, UCHAR type);
void rtk_write_modem_radio_reg(UCHAR addr, UCHAR type, UINT32 val);
void __attribute__ ((noinline)) write_table_to_io(UINT32 addr, UINT32* table, UINT32 table_size);
//void rtk_set_tx_gain(UINT8 db_index);
void rtk_modem_init();
void rtk_rf_init();
void rtk_modem_rx_agc_tbl_init();

void lc_save_bluewiz_phy_settings(UINT16 * buf);
void lc_load_bluewiz_phy_settings(UINT16 * buf);

#ifndef _REDUCE_LPS_AFTER_RTL8703B_
UINT8 lc_check_link_idle_state(void);
#endif

#if 1 /* moved from _IS_ASIC_ */
void rtl8723_btrf_RxAGCTableInit(void);
#endif
#ifdef _IS_ASIC_
void rtl8723_btrf_PHYInit(void);
void rtl8723_btrf_PHYInit_partial(UINT16 *buf, UINT16 start, UINT16 end);
void rtl8703b_acut_rf_parameter_parse(void);
#define RF_INIT_AD_BIT_MODE  BIT9
#define RF_INIT_AD_BIT_VAL_0 0x0000
#define RF_INIT_AD_BIT_VAL_1 BIT8
#define RF_INIT_VAL_BIT_LOC_0 0x0000 /* NOTE: RF_INIT_BIT_LOC_N 0xN000 */
#define RF_INIT_VAL_BIT_LOC_1 0x1000 /* NOTE: RF_INIT_BIT_LOC_N 0xN000 */

#if 0 /* moved out of _IS_ASIC_ */
void rtl8723_btrf_RxAGCTableInit(void);
#endif
void rtl8723_btrf_TxGainTableInit(void);
void rtl8723_btrf_start_thermal_timer(void);
void rtl8723_btrf_RFIQK(void);
#endif

#ifdef _YL_NEW_MODEM_PGA_SETTLINGTIME_
#define PGA_SETTLING_X             3
#define PGA_SETTLINGTIME_UP        PGA_SETTLING_X /* (x+1)*0.5us, 6: 3.5us */
#define PGA_SETTLINGTIME_BIG       PGA_SETTLING_X
#define PGA_SETTLINGTIME_SML       PGA_SETTLING_X
#define PGA_SETTLINGTIME_DN        PGA_SETTLING_X
#define PGA_SETTLINGTIME_UP_BT4    PGA_SETTLING_X /* (x+1)*0.5us, 6: 3.5us */
#define PGA_SETTLINGTIME_BIG_BT4   PGA_SETTLING_X
#define PGA_SETTLINGTIME_SML_BT4   PGA_SETTLING_X
#define PGA_SETTLINGTIME_DN_BT4    PGA_SETTLING_X
#endif

#ifdef _RF_CONFIG_PROTECTION_
extern INT8 lc_rf_ctrl_stack;
extern UINT16 lc_rf_off_mask;

#ifdef _NEW_MODEM_PI_ACCESS_
extern UINT16 lc_host_addr_11_6_mask;
#define RF_CTRL_HOST_ADDR_11_6_BITS                  lc_host_addr_11_6_mask
#else
#define RF_CTRL_HOST_ADDR_11_6_BITS                  0
#endif


#ifdef _NEW_MODEM_PI_ACCESS_
#ifdef _NEW_MODEM_PI_ACCESS_BY_VENDOR_REG_
#define REG_BTPHY_FPI_IF                0x354
#else
extern UINT8 g_efuse_modem_pi_enable;
#endif
extern UINT8 g_modem_init_completed;
#define MODEM_PI_PAGE_0                 0
#define MODEM_PI_PAGE_1                 1
#define MODEM_PI_PAGE_2                 2
#define MODEM_PI_PAGE_3                 3
#define MODEM_PI_PAGE_4                 4
#define MODEM_PI_PAGE_5                 5
#define MODEM_PI_PAGE_6                 6
#define MODEM_PI_PAGE_NEW_AGC_TAB0      8
#define MODEM_PI_PAGE_NEW_AGC_TAB1      9
#define MODEM_PI_PAGE_NEW_AGC_TAB2      10
#define MODEM_PI_PAGE_NEW_AGC_TAB3      11
#define MODEM_PI_PAGE_OLD_AGC_TAB0      12
#define MODEM_PI_PAGE_W_ADDR0_START       MODEM_PI_PAGE_NEW_AGC_TAB0
#define MODEM_PI_PAGE_W_ADDR0_END         MODEM_PI_PAGE_OLD_AGC_TAB0
#endif

#define RF_CTRL_SET_RF_OFF_INVALID \
{ \
    if (lc_rf_ctrl_stack != 0) { \
        BB_write_baseband_register(EXTENDED_HOST_ACCESS_REGISTER, RF_CTRL_HOST_ADDR_11_6_BITS); \
        lc_rf_off_mask = 0; \
    } \
    lc_rf_ctrl_stack = 1; \
}

#define RF_CTRL_CLEAR_RF_OFF_INVALID    {lc_rf_ctrl_stack = 0;}

#define RF_CTRL_SET_RF_OFF \
{ \
    if (lc_rf_ctrl_stack == 0) { \
        BB_write_baseband_register(EXTENDED_HOST_ACCESS_REGISTER, RF_CTRL_HOST_ADDR_11_6_BITS | BIT8); \
        lc_rf_off_mask = BIT8; \
    } \
    lc_rf_ctrl_stack++; \
}
#define RF_CTRL_CLR_RF_OFF \
{ \
    if (lc_rf_ctrl_stack > 0) { \
        lc_rf_ctrl_stack--; \
    } \
    if (lc_rf_ctrl_stack == 0) { \
        BB_write_baseband_register(EXTENDED_HOST_ACCESS_REGISTER, RF_CTRL_HOST_ADDR_11_6_BITS); \
        lc_rf_off_mask = 0; \
    } \
}
#define RF_CTRL_WAIT_RF_OFF_READY \
{ \
    if (lc_rf_ctrl_stack == 1) { \
        UINT32 tmp = 0; \
        do { \
            if (BB_read_baseband_register(EXTENDED_HOST_ACCESS_REGISTER) & BIT9) break; \
            if (tmp > 100000) break; /* avoid bluewiz no go back idle state */ \
            tmp++; \
        } while (1); \
    } \
}
#define RF_CTRL_RF_OFF_BIT                  lc_rf_off_mask
#else
#define RF_CTRL_SET_RF_OFF_INVALID
#define RF_CTRL_CLEAR_RF_OFF_INVALID
#define RF_CTRL_SET_RF_OFF
#define RF_CTRL_CLR_RF_OFF
#define RF_CTRL_WAIT_RF_OFF_READY
#define RF_CTRL_RF_OFF_BIT                  0
#endif                                                                                    \

extern ALIGN(4) SECTION_SRAM UINT8 bzdma_tx_buf[LMP_MAX_PICONETS_SUPPORTED][BZDMA_ACL_TX_BUF_SIZE];

#ifdef LC
/*
 * Global variables
 */
PKT_HEADER_QUEUE    lc_rx_pkt_header_q;

LUT_EXTENSION_TABLE lut_ex_table[LC_MAX_NUM_OF_LUT_EX_TABLES];

#ifdef COMPILE_PERIODIC_INQUIRY
UINT32 lc_periodic_inq_max_interval;
UINT32 lc_periodic_inq_min_interval;
UINT32 lc_periodic_inquiry_interval;
#endif

/* Free byte in case of single fifo */
TimerHandle_t periodic_inquiry_timer = NULL;

LC_PICONET_SCHEDULER  lc_piconet_scheduler[LMP_MAX_PICONETS_SUPPORTED];

#else /* LC */

extern PKT_HEADER_QUEUE lc_rx_pkt_header_q;

extern LUT_EXTENSION_TABLE lut_ex_table[LC_MAX_NUM_OF_LUT_EX_TABLES];

#ifdef COMPILE_PERIODIC_INQUIRY
extern UINT32 lc_periodic_inq_max_interval;
extern UINT32 lc_periodic_inq_min_interval;
extern UINT32 lc_periodic_inquiry_interval;
#endif

extern UCHAR lmp_current_nb;
extern UCHAR lc_cur_connecting_am_addr;
extern TimerHandle_t periodic_inquiry_timer;

extern UINT8 lc_cont_crc_rx_cnt[LC_MAX_NUM_OF_LUT_EX_TABLES];
extern UINT8 lc_cont_crc_tx_cnt[LC_MAX_NUM_OF_LUT_EX_TABLES];

extern UINT8 lc_waiting_for_crc_pkt_ack[LMP_MAX_PICONETS_SUPPORTED];

extern UINT16 lc_slave_bd_addr[3];
extern UINT16 lc_slave_parity_bits[3];

#ifdef COMPILE_ESCO
extern UCHAR lc_esco_pkt_tx[8];
#endif

extern UCHAR lc_cur_connecting_am_addr;
extern TimerHandle_t random_backoff_timer;
extern UINT16 lc_paging_bd_addr[3];
extern UINT16 lc_paging_parity_bits[3];
extern UINT16 lc_slave_bd_addr[3];
extern UINT16 lc_slave_parity_bits[3];

extern UINT32 lc_sniff_cont_count[LC_MAX_NUM_OF_LUT_EX_TABLES];


extern TimerHandle_t random_backoff_timer;
extern UCHAR random_backoff_status;

extern LC_PICONET_SCHEDULER lc_piconet_scheduler[LMP_MAX_PICONETS_SUPPORTED];

void lc_update_pkts_allowed(UINT16 ce_index);

#endif /* LC */

UINT16 lc_read_rssi_register(void);

#ifdef SCO_OVER_HCI
#define SCO_HCI_INVALID                                    ((UCHAR)0)
#define SCO_HCI_FIFO_LOADED                                ((UCHAR)1)
#define SCO_HCI_TRANSMITTED                                ((UCHAR)2)
#define SCO_HCI_INVALID_SCO_CE_INDEX                    ((UCHAR)0xff)

extern UCHAR lc_beacon_running_flag;
extern UINT16 programmable_rand;

#ifdef COMPILE_ESCO
extern ESCO_SCHEDULER esco_scheduler;
extern UCHAR lc_esco_stop_reception[8];
extern UCHAR lc_esco_pkt_tx[8];

/* Indicates if a packet is stored in the global esco buffer */
extern UCHAR global_esco_buf_valid[8];

/* Indicates if the data stored in the global buffer has CRC error */
extern UCHAR global_esco_data_correct[8];

#ifdef _ESCO_NEW_SCHEDULER_FOR_FULL_BW_
extern UCHAR lc_esco_window_overlap[8];
#endif
#endif //#ifdef COMPILE_ESCO

#ifdef  _SUPPORT_BT_CONTROL_PCM_MUX_
void restore_pcm_mux_state_when_disconnect();
//void switch_pcm_mux_after_con_established();
void switch_pcm_mux_after_con_established_pcmoutctrl(UINT16 pcmctrl3);
#endif

void lc_handle_sco_rx_packet(int sco_ce_index);
void lc_handle_sco_instant_interrupt(UCHAR sco_ce_index);
void lc_handle_sco_rx_interrupt(UCHAR sco_ce_index);
void lc_handle_sco_erroneous_data(UCHAR sco_ce_index, UCHAR data_status);
#endif /* SCO_OVER_HCI */

#ifdef _IMPROVE_PCM_MUTE_CTRL_TIMING_
extern UINT8 g_sco_no_rx_count;
extern UINT8 g_sco_no_rx_force_output_zero_en;
void lc_pcm_mute_control(void);
#endif

extern void lc_clear_queues(UINT16 ce_index, UCHAR acl_flag);
extern void lc_calculate_and_store_rssi(UINT16 rssi, UINT16 ce_index);
extern INT8 lc_calculate_log_from_rssi(UINT16 rssi);

extern UINT16 lc_current_interrupt_mask;
extern UINT16 lc_current_interrupt_mask2;
extern UINT16 lc_var_esco_fifo_flush_length_and_tx_pkt_len_reg;
extern TimerHandle_t random_backoff_timer;

#ifdef  COMPILE_ROLE_SWITCH
extern void lc_handle_role_switch_complete_in_scatternet(void);
extern UCHAR lmp_start_switch_instant_timer(UINT16 ce_index);
#endif

extern void lc_handle_lc_hlc_signal(UCHAR am_addr, UCHAR phy_piconet_id);
extern void lc_reschedule(UCHAR phy_piconet_id, UCHAR bb_flow);

extern void lc_update_radio_timings(void);



INLINE extern void lc_flush_piconet1_tx_fifo(void);
INLINE extern void lc_flush_broadcast_tx_fifo(void);
INLINE extern void lc_flush_esco_rx_fifo(void);
INLINE extern void lc_flush_esco_tx_fifo(void);

extern void lc_flush_rx_fifo(void);
void lc_update_scatternet_status_after_mss(UINT16 ce_index);
void lc_update_scatternet_state_deletion(UCHAR phy_piconet_id);
UINT8 lc_update_scatternet_state_addition(LC_CUR_SCATTERNET_STATE new_state,
        UCHAR new_piconet_id);
void lc_init_seqn_scatternet(UCHAR am_addr, LUT_EXTENSION_TABLE  *ex_lut,
                                                        UCHAR piconet_id);
UCHAR lc_get_piconet_id_for_pagescan(void);
UCHAR lc_get_connected_piconet_id(void);

#ifdef COMPILE_SNIFF_MODE
SECTION_ISR_LOW void lc_update_next_sniff_instant();
#endif /* COMPILE_SNIFF_MODE */

INLINE void lc_set_lc_cur_connecting_am_addr(UCHAR val);
void lc_optimize_data_transfer_during_sniff(UINT16 ce_index);
void lc_reset_lc_piconet_scheduler(UINT32 index);
#endif /*__LC_INTERNAL_H__ */

void lc_write_rf_rx_backup_status_rpt_reg(UINT8 index,
                     UINT16 addr, UINT8 legacy_on, UINT8 le_on);
UINT16 lc_read_rf_rx_backup_status_rpt_reg(UINT8 index);

#ifdef _DAPE_SNIFF_PKT_TEST
UINT32 lc_get_least_sniff_interval(UINT16 *sniff_ce_index, UINT16 *sniff_attempt);
#else
UINT32 lc_get_least_sniff_interval(void);
#endif

#if defined(_CCH_LPS_) && defined(_YL_LPS)
API_RESULT lc_post_lps_mode_signal(UCHAR lps_mode, UINT32 wakeup_instant, UCHAR piconet_id);
#endif
#ifdef _MODI_LPS_STATE_WITH_INTR_
API_RESULT lc_post_lps_stste_signal(UCHAR lps_mode);
#endif
/** @} end: lc_internal */
